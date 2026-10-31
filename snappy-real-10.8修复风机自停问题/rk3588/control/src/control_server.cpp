/**
 * @file control_server.cpp
 * @brief 无传感器控制服务实现
 *
 * @details
 * - 请求路径只执行 JSON 解析、控制状态和邮箱操作。
 * - 不在控制状态锁内执行物理停止。
 * - 不在 status/telemetry 路径访问硬件。
 * - 物理执行由 ActuatorHub worker 完成。
 * - 所有状态读取均来自内存快照。
 */

#include "snappy_snappy/control_server.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <utility>

#include "snappy/control/sensor_service.hpp"
#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy_snappy {

namespace {

constexpr auto kG20LeaseConfirmWindowFloor =
    std::chrono::milliseconds(4000);

/*
 * control.acquire 幂等重放窗口。
 */
constexpr auto kAcquireIdempotencyWindow =
    std::chrono::seconds(2);

std::string text_value(
    const nlohmann::json& object,
    const char* key) {
  if (!object.is_object() ||
      !object.contains(key) ||
      !object.at(key).is_string()) {
    return {};
  }

  return object.at(key).get<std::string>();
}

bool optional_epoch_matches(
    const nlohmann::json& request,
    std::uint64_t expected) {
  if (!request.contains("control_epoch")) {
    return true;
  }

  try {
    std::uint64_t value = 0;

    if (request.at("control_epoch").is_string()) {
      value =
          std::stoull(
              request.at("control_epoch")
                  .get<std::string>());
    } else {
      value =
          request.at("control_epoch")
              .get<std::uint64_t>();
    }

    /*
     * 兼容旧 Android/G20 适配器。
     * control_epoch=0 表示不附加 epoch 限制。
     */
    return value == 0 ||
           value == expected;
  } catch (...) {
    return false;
  }
}

bool is_actuator_action(
    const std::string& action) {
  return action.rfind("brush.", 0) == 0 ||
         action.rfind("brush_push.", 0) == 0 ||
         action.rfind("scraper_push.", 0) == 0 ||
         action.rfind("push.", 0) == 0 ||
         action.rfind("climber.", 0) == 0 ||
         action.rfind("fan.", 0) == 0;
}

std::string sample_quality_to_string(
    snappy::drivers::SampleQuality quality) {
  switch (quality) {
    case snappy::drivers::SampleQuality::kGood: return "GOOD";
    case snappy::drivers::SampleQuality::kStale: return "STALE";
    case snappy::drivers::SampleQuality::kInvalid: return "INVALID";
    case snappy::drivers::SampleQuality::kUnknown: default: return "UNKNOWN";
  }
}

std::string sensor_health_to_string(
    snappy::control::SensorCommunicationHealth health) {
  switch (health) {
    case snappy::control::SensorCommunicationHealth::kGood: return "GOOD";
    case snappy::control::SensorCommunicationHealth::kDegraded: return "DEGRADED";
    case snappy::control::SensorCommunicationHealth::kFault: return "FAULT";
    case snappy::control::SensorCommunicationHealth::kUnknown: default: return "UNKNOWN";
  }
}

bool is_stop_action(
    const std::string& action) {
  return action == "brush.stop" ||
         action == "brush_push.stop" ||
         action == "scraper_push.stop" ||
         action == "push.stop" ||
         action == "climber.stop" ||
         action == "fan.stop" ||
         action == "fan.disable";
}

}  // namespace

ControlServer::ControlServer(
    ActuatorHub& hub,
    RuntimeConfig config)
    : hub_(hub),
      config_(std::move(config)),
      socket_path_(config_.socket_path),
      sensors_(config_.sensors) {}

ControlServer::~ControlServer() {
  stop();
}

bool ControlServer::start() {
  if (running_.load()) {
    return true;
  }

  const auto parent =
      std::filesystem::path(socket_path_)
          .parent_path();

  if (!parent.empty()) {
    std::error_code error;

    std::filesystem::create_directories(
        parent,
        error);
  }

  /*
   * 只移除当前服务自己的旧 socket。
   */
  ::unlink(socket_path_.c_str());

  server_fd_ =
      ::socket(
          AF_UNIX,
          SOCK_SEQPACKET | SOCK_CLOEXEC,
          0);

  if (server_fd_ < 0) {
    return false;
  }

  sockaddr_un address{};
  address.sun_family = AF_UNIX;

  if (socket_path_.size() >=
      sizeof(address.sun_path)) {
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  std::strncpy(
      address.sun_path,
      socket_path_.c_str(),
      sizeof(address.sun_path) - 1);

  if (::bind(
          server_fd_,
          reinterpret_cast<
              const sockaddr*>(&address),
          sizeof(address)) != 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    return false;
  }

  if (::listen(server_fd_, 16) != 0) {
    ::close(server_fd_);
    server_fd_ = -1;
    ::unlink(socket_path_.c_str());
    return false;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    state_ = State::kDisarmed;
    lease_held_ = false;
    lease_source_.clear();
    lease_session_.clear();
    lease_id_.clear();
    safety_reason_.clear();
    last_g20_seen_ = {};
  }

  running_.store(true);

  accept_thread_ =
      std::thread(
          [this]() {
            accept_loop();
          });

  return true;
}

void ControlServer::stop() noexcept {
  const bool was_running =
      running_.exchange(false);

  if (server_fd_ >= 0) {
    ::shutdown(
        server_fd_,
        SHUT_RDWR);

    ::close(server_fd_);
    server_fd_ = -1;
  }

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  ::unlink(socket_path_.c_str());

  if (was_running) {
    /*
     * 控制 socket 关闭时必须同步进入安全停止。
     */
    hub_.safe_stop();
  }
}

void ControlServer::accept_loop() {
  while (running_.load()) {
    pollfd descriptor{};
    descriptor.fd = server_fd_;
    descriptor.events = POLLIN;

    const int result =
        ::poll(
            &descriptor,
            1,
            100);

    if (result <= 0) {
      continue;
    }

    if ((descriptor.revents & POLLIN) == 0) {
      continue;
    }

    const int client =
        ::accept4(
            server_fd_,
            nullptr,
            nullptr,
            SOCK_CLOEXEC);

    if (client < 0) {
      if (errno == EINTR ||
          errno == EAGAIN ||
          errno == EWOULDBLOCK) {
        continue;
      }

      break;
    }

    /*
     * 每个请求只处理一个数据包。
     * serve_client 内部有固定 200 ms 超时，
     * 不会无限等待客户端。
     */
    serve_client(client);
  }
}

void ControlServer::serve_client(
    int client_fd) {
  pollfd descriptor{};
  descriptor.fd = client_fd;
  descriptor.events = POLLIN;

  if (::poll(
          &descriptor,
          1,
          200) <= 0) {
    ::close(client_fd);
    return;
  }

  char buffer[65536]{};

  const auto received =
      ::recv(
          client_fd,
          buffer,
          sizeof(buffer),
          0);

  if (received <= 0) {
    ::close(client_fd);
    return;
  }

  const auto response =
      handle_request(
          std::string(
              buffer,
              static_cast<std::size_t>(
                  received)));

  /*
   * ACK、状态和遥测均为一次性快照响应，
   * 不建立发送队列。
   */
  static_cast<void>(
      ::send(
          client_fd,
          response.data(),
          response.size(),
          MSG_NOSIGNAL));

  ::close(client_fd);
}

std::string ControlServer::handle_request(
    const std::string& request) {
  json message;

  try {
    message =
        json::parse(request);
  } catch (const std::exception& error) {
    return make_ack(
        json::object(),
        "FAILED",
        std::string("invalid JSON: ") +
            error.what());
  }

  const std::string action =
      text_value(
          message,
          "action");

  /*
   * 快照请求只复制内存状态。
   * 不访问 ActuatorHub 的物理设备。
   */
  if (action == "gateway.status_snapshot" ||
      action == "control.status") {
    return make_status();
  }

  if (action == "gateway.telemetry_snapshot" ||
      action == "control.telemetry") {
    return make_telemetry();
  }

  if (action == "gateway.capabilities" ||
      action == "control.capabilities") {
    return make_capabilities();
  }

  if (action == "control.acquire") {
    const auto source =
        text_value(
          message,
          "source_id");

    const auto session =
        text_value(
          message,
          "session_id");

    const auto message_id =
        text_value(
            message,
            "message_id");

    /*
     * 幂等重放判定。
     */
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (!message_id.empty() &&
          message_id == last_acquire_message_id_ &&
          control_epoch_ == last_acquire_epoch_ &&
          lease_held_ &&
          lease_source_ == source &&
          lease_session_ == session &&
          std::chrono::steady_clock::now() - last_acquire_at_ <=
              kAcquireIdempotencyWindow) {
        SNAPPY_DRIVER_LOG_INFO(
            "control",
            "control acquire idempotent replay: message_id=" +
                message_id +
                " lease_id=" + lease_id_ +
                " control_epoch=" +
                std::to_string(control_epoch_));

        return make_ack(
            message,
            "SUCCEEDED",
            "control lease acquired (idempotent replay)");
      }
    }

    /*
     * 租约冲突判定。
     */
    const auto lease_conflict =
        [this](
            const std::string& candidate_source,
            const std::string& candidate_session) {
          if (!lease_held_) {
            return false;
          }

          if (lease_source_.empty() &&
              lease_session_.empty()) {
            return false;
          }

          if (state_ == State::kSafeHold ||
              state_ == State::kEstopLatched ||
              state_ == State::kDisarmed ||
              state_ == State::kBoot) {
            return false;
          }

          return lease_source_ != candidate_source ||
                 lease_session_ != candidate_session;
        };

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (lease_conflict(
              source,
              session)) {
        return make_ack(
           message,
           "FAILED",
           "control lease is held by another session");
      }
    }

    constexpr auto kSafetyTransitionTimeout = std::chrono::seconds(5);

    if (!hub_.wait_for_safety_clear(
          kSafetyTransitionTimeout)) {
      return make_ack(
        message,
        "FAILED",
        "actuator safety stop is still pending or failed");
    }

    /*
     * 取得控制权即进入可控状态。
     */
    const auto gate =
        hub_.enable_commands();

    if (!gate.ok) {
      return make_ack(
          message,
          "FAILED",
          gate.detail);
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      /*
       * 再次校验租约。
       */
      if (lease_conflict(
              source,
              session)) {
        return make_ack(
            message,
            "FAILED",
            "control lease is held by another session");
      }

    const bool taking_over =
        lease_held_ &&
        (!lease_source_.empty() ||
         !lease_session_.empty()) &&
        (lease_source_ != source ||
         lease_session_ != session);

    const std::string previous_source = lease_source_;
    const std::string previous_session = lease_session_;

    lease_held_ = true;
    lease_source_ = source;
    lease_session_ = session;

    lease_id_ =
        "lease-" +
        std::to_string(
            ++control_epoch_);

    /* 记录本次 acquire，供重复投递的命令幂等重放。 */
    last_acquire_message_id_ = message_id;
    last_acquire_epoch_ = control_epoch_;
    last_acquire_at_ = std::chrono::steady_clock::now();

    if (taking_over) {
      /*
       * 新 session 从锁停态接管旧租约：必须留痕，否则"控制权被谁拿走了"在日志里查不到。
       */
      SNAPPY_DRIVER_LOG_WARN(
          "control",
          "control lease taken over while latched: "
          "old_source=" + previous_source +
          " old_session=" + previous_session +
          " new_source=" + lease_source_ +
          " new_session=" + lease_session_);
    }

    last_g20_seen_ =
        std::chrono::steady_clock::now();

    /* 已经显式取过控制权，不再走上电自动就绪路径。 */
    auto_control_ = false;

    state_ = State::kArmedIdle;
    safety_reason_.clear();
  }

  return make_ack(
      message,
      "SUCCEEDED",
      "control lease acquired");
  }

  if (action == "control.heartbeat") {
    std::string status;
    std::string detail;

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (!lease_valid_locked(message)) {
        status = "FAILED";
        detail =
            "control lease is invalid";
      } else {
        last_g20_seen_ =
            std::chrono::steady_clock::now();

        status = "SUCCEEDED";
        detail =
            "heartbeat accepted";
      }
    }

    return make_ack(
        message,
        status,
        detail);
  }

  if (action == "control.arm") {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (!lease_valid_locked(message)) {
        return make_ack(
            message,
            "FAILED",
            "control lease is invalid");
      }

      if (state_ != State::kAcquired &&
          state_ != State::kArmedIdle) {
        return make_ack(
            message,
            "FAILED",
            "control state does not allow arm");
      }
    }

    /*
     * enable_commands 只打开 ActuatorHub 命令门，不在这里执行设备 I/O。
     */
    const auto result =
        hub_.enable_commands();

    if (!result.ok) {
      return make_ack(
          message,
          "FAILED",
          result.detail);
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      /*
       * enable_commands 返回后再次验证租约和状态， 防止 arm 期间控制权已被安全停止逻辑撤销。
       */
      if (!lease_valid_locked(message) ||
          (state_ != State::kAcquired &&
           state_ != State::kArmedIdle)) {
        hub_.safe_stop();

        return make_ack(
            message,
            "FAILED",
            "control state changed during arm");
      }

      state_ = State::kArmedIdle;
      last_g20_seen_ =
          std::chrono::steady_clock::now();
    }

    return make_ack(
        message,
        "SUCCEEDED",
        "control armed");
  }

  /*
   * safe_hold、disarm、estop、release 和 link_down都必须进入执行器安全停止流程。
   */
  if (action == "control.safe_hold" ||
      action == "control.disarm" ||
      action == "control.estop" ||
      action == "control.release" ||
      action == "control.link_down") {
    const bool link_down =
        action == "control.link_down";

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      /*
       * link_down 是通信丢失保护动作，即使租约已经失效也必须允许执行。
       */
      if (!link_down &&
          !lease_valid_locked(message)) {
        return make_ack(
            message,
            "FAILED",
            "control lease is invalid");
      }
    }

    /*
     * 释放状态锁后再执行安全停止。
     */
    hub_.safe_stop();

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      /*
       * 暂停键、“安全停止”、急停、失去联系之后，必须重新点一次“控制权”才能恢复控制。
       */
      auto_control_ = false;

      if (action == "control.estop") {
        state_ = State::kEstopLatched;
      } else if (
          action == "control.safe_hold" ||
          action == "control.link_down") {
        state_ = State::kSafeHold;
      } else {
        state_ = State::kDisarmed;
      }

      safety_reason_ = action;

      if (action == "control.release") {
        lease_held_ = false;
        lease_source_.clear();
        lease_session_.clear();
        lease_id_.clear();

        /*
         * 租约已撤销，重放记录必须同步作废，否则 release 之后一条迟到的重复 acquire 会被"重放"成一张并不存在的租约。
         */
        last_acquire_message_id_.clear();
        last_acquire_epoch_ = 0;
        last_acquire_at_ = std::chrono::steady_clock::time_point{};
      }
    }

    SNAPPY_DRIVER_LOG_WARN(
        "control",
        "safety stop accepted: action=" +
            action);

    return make_ack(
        message,
        "SUCCEEDED",
        "safe stop accepted");
  }

  if (action == "fault.clear") {
  /*
   * 故障清除是“安全停止 + 恢复控制”的兜底操作。
   */

  const auto result =
      hub_.submit(
          "fault.clear",
          message.value(
              "params",
              json::object()));

  if (!result.ok) {
    return make_ack(
        message,
        "FAILED",
        result.detail);
  }

  {
    std::lock_guard<
        std::recursive_mutex>
        lock(mutex_);

    /*
     * 清除故障是屏幕端明确的恢复操作，复位完成之后不需要重新点“控制权”。
     */
    auto_control_ = true;

    state_ = State::kDisarmed;

    safety_reason_ = "fault.clear";
  }

  return make_ack(
      message,
      "ACCEPTED",
      "fault clear accepted; "
      "actuators are resetting while control lease is preserved");
  }

  /*
   * 所有执行器动作都要求：
   * 1. 当前会话持有租约；
   * 2. 控制状态已经 arm；
   * 3. ActuatorHub 命令门仍然打开。
   */
  if (is_actuator_action(action)) {
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (!lease_valid_locked(message)) {
        return make_ack(
            message,
            "FAILED",
            "control lease is invalid");
      }

      if (state_ != State::kArmedIdle &&
          state_ != State::kActive) {
        return make_ack(
            message,
            "FAILED",
            "control is not armed");
      }
    }

    const auto result =
        hub_.submit(
            action,
            message.value(
                "params",
                json::object()));

    if (!result.ok) {
      return make_ack(
          message,
          "FAILED",
          result.detail);
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      /*
       * 只有动作提交成功后才改变控制状态。
       */
      state_ =
          is_stop_action(action)
              ? State::kArmedIdle
              : State::kActive;

      last_g20_seen_ =
          std::chrono::steady_clock::now();
    }

    return make_ack(
        message,
        "ACCEPTED",
        result.detail);
  }

  return make_ack(
      message,
      "FAILED",
      "unknown action");
}

std::string ControlServer::make_ack(
    const json& request,
    const std::string& status,
    const std::string& detail) {
  State state;
  std::uint64_t epoch;
  bool lease_held;
  std::string lease_id;

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    state = state_;
    epoch = control_epoch_;
    lease_held = lease_held_;
    lease_id = lease_id_;
  }

  json response;
  response["schema_version"] = 1;
  response["kind"] = "ack";
  response["message_id"] =
      text_value(
          request,
          "message_id");
  response["seq"] =
      request.value(
          "seq",
          0ULL);
  response["status"] = status;
  response["detail"] = detail;
  response["state"] =
      state_name(state);
  response["ack_time_ms"] =
      now_ms();
  response["control_epoch"] =
      epoch;

  if (lease_held) {
    response["lease_id"] =
        lease_id;
  }

  return response.dump();
}

std::string ControlServer::make_status()
    const {
  /*
   * ActuatorHub::snapshot() 只复制内存快照。
   * 此处不轮询 RS485、CAN 或执行器。
   */
  const auto snapshot =
      hub_.snapshot();

  State state;
  bool lease_held;
  std::string lease_id;
  std::string lease_owner;
  std::uint64_t epoch;
  std::string safety_reason;
  bool g20_online = false;

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    state = state_;
    lease_held = lease_held_;
    lease_id = lease_id_;
    lease_owner = lease_source_;
    epoch = control_epoch_;
    safety_reason = safety_reason_;

    if (last_g20_seen_
            .time_since_epoch()
            .count() != 0) {
      g20_online =
          std::chrono::steady_clock::now() -
              last_g20_seen_ <=
          config_.hardware.safety
              .g20_lease_timeout;
    }
  }

  const auto& hardware =
      config_.hardware;

  json response;
  response["schema_version"] = 1;
  response["kind"] = "status";
  response["robot_id"] =
      hardware.robot_id;
  response["state"] =
      state_name(state);

  response["armed"] =
      state == State::kArmedIdle ||
      state == State::kActive;

  response["control_started"] =
      snapshot.started;
  response["partial_start"] =
      snapshot.partial_start;
  response["hardware_ready_for_arm"] =
      snapshot.ready_for_arm;

  /*
   * 硬件就绪，且当前状态允许接管或已经在控制中。
   */
  response["ready_for_arm"] =
      snapshot.ready_for_arm &&
      (state == State::kBoot ||
       state == State::kDisarmed ||
       state == State::kAcquired ||
       state == State::kArmedIdle);

  response["lease_held"] =
      lease_held;
  response["lease_id"] =
      lease_id;
  response["lease_owner"] =
      lease_owner;
  response["control_epoch"] =
      epoch;
  response["g20_online"] =
      g20_online;
  response["detail"] =
      safety_reason;
  response["safety_reason"] =
      safety_reason;

  response["updated_at_ms"] =
      snapshot.updated_at_ms == 0
          ? now_ms()
          : snapshot.updated_at_ms;

  /*
   * 保留原 G20/遥控端网络配置字段。
   */
  response["g20_udp"]["listen_address"] =
      hardware.g20_udp.listen_address;
  response["g20_udp"]["command_port"] =
      hardware.g20_udp.command_listen_port;
  response["g20_udp"]["status_port"] =
      hardware.g20_udp.status_publish_port;
  response["g20_udp"]["telemetry_port"] =
      hardware.g20_udp.telemetry_publish_port;
  response["g20_udp"]["lease_timeout_ms"] =
      hardware.g20_udp.lease_timeout.count();

  /*
   * 保留原摄像头路径、设备和播放地址。
   * 本服务不改变摄像头发送程序。
   */
  response["video"]["path"] =
      hardware.video.path;
  response["video"]["target_address"] =
      hardware.video.target_address;
  response["video"]["camera1_device"] =
      hardware.video.camera1_device;
  response["video"]["camera2_device"] =
      hardware.video.camera2_device;
  response["video"]["camera1_port"] =
      hardware.video.camera1_udp_port;
  response["video"]["camera2_port"] =
      hardware.video.camera2_udp_port;
  response["video"]["camera1_url"] =
      hardware.video.camera1_play_url;
  response["video"]["camera2_url"] =
      hardware.video.camera2_play_url;

  response["actuators"]["brush"]["state"] =
      snapshot.brush_state;
  response["actuators"]["brush"]["speed_raw"] =
      snapshot.brush_speed_raw;

  response["actuators"]["fan"]["active"] =
      snapshot.fan_active ||
      snapshot.fan_left_raw != 0 ||
      snapshot.fan_right_raw != 0;
  response["actuators"]["fan"]["left_raw"] =
      snapshot.fan_left_raw;
  response["actuators"]["fan"]["right_raw"] =
      snapshot.fan_right_raw;
  response["actuators"]["fan"]["synchronized"] =
      snapshot.fan_left_raw ==
      snapshot.fan_right_raw;

  response["actuators"]["brush_push"]["state"] =
      snapshot.brush_push_state;
  response["actuators"]["brush_push"]["remaining_ms"] =
      snapshot.brush_push_remaining_ms;
  response["actuators"]["brush_push"]["last_duration_ms"] =
      snapshot.brush_push_last_duration_ms;
  response["actuators"]["brush_push"]["extension_mm"] =
      snapshot.brush_push_extension_mm;

  response["actuators"]["scraper_push"]["state"] =
      snapshot.scraper_push_state;
  response["actuators"]["scraper_push"]["remaining_ms"] =
      snapshot.scraper_push_remaining_ms;
  response["actuators"]["scraper_push"]["last_duration_ms"] =
      snapshot.scraper_push_last_duration_ms;
  response["actuators"]["scraper_push"]["extension_mm"] =
      snapshot.scraper_push_extension_mm;

  /*
   * 保留旧协议 push.* 兼容别名。
   */
  response["actuators"]["push"]["state"] =
      snapshot.brush_push_state;
  response["actuators"]["push"]["remaining_ms"] =
      snapshot.brush_push_remaining_ms;
  response["actuators"]["push"]["last_duration_ms"] =
      snapshot.brush_push_last_duration_ms;
  response["actuators"]["push"]["extension_mm"] =
      snapshot.brush_push_extension_mm;

  response["actuators"]["climber"]["state"] =
      snapshot.climber_state;
  response["actuators"]["climber"]["speed_rpm"] =
      snapshot.climber_speed_rpm;

  /*
   * 传感器状态只读取 SensorService 内存缓存。
   * 即使某个传感器正在 timeout，这里也不会等待硬件事务。
   */
  response["sensors"]["disabled"] =
      sensors_ == nullptr;
  response["sensors"]["mode"] =
      sensors_ != nullptr ? "background_cache" : "disabled";
  response["sensors"]["cache_count"] = 0;
  response["sensors"]["samples"] = json::array();
  response["sensors"]["devices"] = json::array();

  if (sensors_ != nullptr) {
    const auto samples = sensors_->cached_samples();
    const auto devices = sensors_->device_statuses();
    response["sensors"]["cache_count"] = samples.size();

    for (const auto& sample : samples) {
      json item;
      item["name"] = sample.logical_name;
      item["unit"] = sample.unit;
      item["timestamp_ms"] = sample.timestamp_ms;
      item["age_ms"] = sample.age_ms;
      item["quality"] = sample_quality_to_string(sample.quality);
      item["value"] = sample.value ? json(*sample.value) : json(nullptr);
      response["sensors"]["samples"].push_back(std::move(item));
    }

    for (const auto& device : devices) {
      json item;
      item["name"] = device.logical_name;
      item["online"] = device.online;
      item["healthy"] = device.online;
      item["health"] = device.online ? "READY" : "FAULT";
      item["communication_health"] = sensor_health_to_string(device.communication_health);
      item["consecutive_failures"] = device.consecutive_failures;
      item["failure_count"] = device.consecutive_failures;
      item["last_poll_ms"] = device.last_poll_ms;
      item["last_success_ms"] = device.last_success_ms;
      item["last_failure_ms"] = device.last_failure_ms;
      item["last_error_ms"] = device.last_failure_ms;
      item["last_error"] = device.last_error;
      response["sensors"]["devices"].push_back(std::move(item));
    }
  }

  response["watchdog"]["enabled"] =
      hardware.watchdog.enabled;
  response["watchdog"]["active"] =
      false;
  response["watchdog"]["healthy"] =
      true;

  response["diagnostics"]["rs485_ready"] =
      snapshot.rs485_ready;
  response["diagnostics"]["can_ready"] =
      snapshot.can_ready;
  response["diagnostics"]["relay_ready"] =
      snapshot.relay_ready;
  response["diagnostics"]["brush_ready"] =
      snapshot.brush_ready;
  response["diagnostics"]["climber_ready"] =
      snapshot.climber_ready;
  response["diagnostics"]["fan_ready"] =
      snapshot.fan_ready;
  response["diagnostics"]["fan_worker_running"] =
      snapshot.fan_worker_running;
  response["diagnostics"]["fan_online"] =
      snapshot.fan_online;
  response["diagnostics"]["last_error"] =
      snapshot.last_error;

  return response.dump();
}

std::string ControlServer::make_telemetry()
    const {
  /*
   * 遥测接口只读取执行器和传感器缓存快照。
   * 不执行任何物理总线查询。
   */
  const auto snapshot =
      hub_.snapshot();

  json response;
  response["schema_version"] = 1;
  response["kind"] = "telemetry";
  response["robot_id"] =
      config_.hardware.robot_id;
  response["updated_at_ms"] =
      now_ms();
  response["timestamp_ms"] =
      response["updated_at_ms"];
  response["mode"] =
      sensors_ != nullptr ? "actuator_plus_sensor" : "actuator_only";
  response["sensors_disabled"] =
      sensors_ == nullptr;

  response["samples"] = json::array();
  if (sensors_ != nullptr) {
    for (const auto& sample : sensors_->cached_samples()) {
      json item;
      item["name"] = sample.logical_name;
      item["value"] = sample.value ? json(*sample.value) : json(nullptr);
      item["unit"] = sample.unit;
      item["quality"] = sample_quality_to_string(sample.quality);
      item["timestamp_ms"] = sample.timestamp_ms;
      item["age_ms"] = sample.age_ms;
      response["samples"].push_back(std::move(item));
    }
  }

  response["actuators"]["brush_state"] =
      snapshot.brush_state;
  response["actuators"]["brush_speed_raw"] =
      snapshot.brush_speed_raw;
  response["actuators"]["fan_active"] =
      snapshot.fan_active;
  response["actuators"]["fan_left_raw"] =
      snapshot.fan_left_raw;
  response["actuators"]["fan_right_raw"] =
      snapshot.fan_right_raw;
  response["actuators"]["brush_push_state"] =
      snapshot.brush_push_state;
  response["actuators"]["brush_push_remaining_ms"] =
      snapshot.brush_push_remaining_ms;
  response["actuators"]["scraper_push_state"] =
      snapshot.scraper_push_state;
  response["actuators"]["scraper_push_remaining_ms"] =
      snapshot.scraper_push_remaining_ms;
  response["actuators"]["climber_state"] =
      snapshot.climber_state;
  response["actuators"]["climber_speed_rpm"] =
      snapshot.climber_speed_rpm;

  return response.dump();
}

std::string ControlServer::make_capabilities()
    const {
  const auto& hardware =
      config_.hardware;

  json response;
  response["schema_version"] = 1;
  response["kind"] =
      "capabilities";
  response["robot_id"] =
      hardware.robot_id;
  response["updated_at_ms"] =
      now_ms();
  response["sensors_disabled"] =
      sensors_ == nullptr;
  response["sensors_enabled"] =
      sensors_ != nullptr;

  response["control_actions"] =
      json::array({
          "control.acquire",
          "control.arm",
          "control.disarm",
          "control.safe_hold",
          "control.estop",
          "control.release",
          "control.link_down",
          "control.heartbeat",
          "fault.clear",
      });

  response["actuators"]["brush"]["actions"] =
      json::array({
          "brush.forward",
          "brush.reverse",
          "brush.stop",
      });
  response["actuators"]["brush"]["max_speed_raw"] =
      15;

  response["actuators"]["fan"]["actions"] =
      json::array({
          "fan.prepare",
          "fan.enable",
          "fan.disable",
          "fan.left_set",
          "fan.right_set",
          "fan.both_set",
          "fan.stop",
      });
  response["actuators"]["fan"]["synchronized"] =
      true;
  response["actuators"]["fan"]["base_raw"] =
      100;
  response["actuators"]["fan"]["step_raw"] =
      400;
  response["actuators"]["fan"]["max_raw"] =
      8191;

  response["actuators"]["brush_push"]["actions"] =
      json::array({
          "brush_push.extend",
          "brush_push.retract",
          "brush_push.extend_timed",
          "brush_push.retract_timed",
          "brush_push.stop",
      });
  response["actuators"]["brush_push"]["max_duration_ms"] =
      hardware.safety
          .brush_push_motor_max_on_time
          .count();

  response["actuators"]["scraper_push"]["actions"] =
      json::array({
          "scraper_push.extend",
          "scraper_push.retract",
          "scraper_push.extend_timed",
          "scraper_push.retract_timed",
          "scraper_push.stop",
      });
  response["actuators"]["scraper_push"]["max_duration_ms"] =
      hardware.safety
          .scraper_push_motor_max_on_time
          .count();

  response["actuators"]["climber"]["actions"] =
      json::array({
          "climber.ascend",
          "climber.descend",
          "climber.stop",
      });
  response["actuators"]["climber"]["speed_unit"] =
      "rpm";
  response["actuators"]["climber"]["initial_speed_rpm"] =
      500;
  response["actuators"]["climber"]["step_rpm"] =
      100;
  response["actuators"]["climber"]["max_speed_rpm"] =
      3000;

  response["links"]["g20_udp"]["listen_address"] =
      hardware.g20_udp.listen_address;
  response["links"]["g20_udp"]["command_port"] =
      hardware.g20_udp.command_listen_port;
  response["links"]["g20_udp"]["status_port"] =
      hardware.g20_udp.status_publish_port;
  response["links"]["g20_udp"]["telemetry_port"] =
      hardware.g20_udp.telemetry_publish_port;

  response["video"]["path"] =
      hardware.video.path;
  response["video"]["target_address"] =
      hardware.video.target_address;
  response["video"]["camera1_device"] =
      hardware.video.camera1_device;
  response["video"]["camera2_device"] =
      hardware.video.camera2_device;
  response["video"]["camera1_port"] =
      hardware.video.camera1_udp_port;
  response["video"]["camera2_port"] =
      hardware.video.camera2_udp_port;
  response["video"]["camera1_url"] =
      hardware.video.camera1_play_url;
  response["video"]["camera2_url"] =
      hardware.video.camera2_play_url;

  return response.dump();
}

bool ControlServer::lease_valid_locked(
    const json& request) const {
  if (!lease_held_) {
    return false;
  }

  /*
   * 上电自动就绪期间还没有任何会话取过控制权，
   * lease_source_ 与 lease_session_ 都为空，
   * 此时接受任意来源的执行器命令。
   */
  if (lease_source_.empty() &&
      lease_session_.empty()) {
    return true;
  }

  if (lease_source_ !=
          text_value(
              request,
              "source_id") ||
      lease_session_ !=
          text_value(
              request,
              "session_id")) {
    return false;
  }

  if (request.contains("lease_id") &&
      request.at("lease_id").is_string() &&
      !request.at("lease_id")
           .get<std::string>()
           .empty() &&
      request.at("lease_id")
           .get<std::string>() !=
          lease_id_) {
    return false;
  }

  return optional_epoch_matches(
      request,
      control_epoch_);
}

void ControlServer::tick() {
  /*
   * 上电自动就绪。
   */
  if (auto_control_) {
    bool waiting =
        false;

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      waiting =
          state_ == State::kBoot ||
          state_ == State::kDisarmed;
    }

    if (waiting &&
        hub_.enable_commands().ok) {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      state_ = State::kArmedIdle;

      lease_held_ = true;

      safety_reason_.clear();
    }
  }

  bool expired = false;

  long long expired_gap_ms = 0;

  long long confirm_window_ms = 0;

  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (!lease_held_ ||
        !config_.hardware.safety
             .safe_stop_on_g20_loss) {
      return;
    }

    /*
     * 初始状态没有收到过心跳，
     * 不使用无效的零时间点计算超时。
     */
    if (last_g20_seen_
            .time_since_epoch()
            .count() == 0) {
      return;
    }

    const auto age =
        std::chrono::steady_clock::now() -
        last_g20_seen_;

    const auto confirm_window =
        std::max(
            kG20LeaseConfirmWindowFloor,
            std::max(
                config_.hardware.safety
                        .g20_lease_timeout *
                    2,
                config_.hardware.safety
                        .g20_loss_confirm *
                    2));

    confirm_window_ms =
        static_cast<long long>(
            confirm_window.count());

    if (age > confirm_window) {
      expired_gap_ms =
          static_cast<long long>(
              std::chrono::duration_cast<
                  std::chrono::milliseconds>(
                  age)
                  .count());

      lease_held_ = false;
      lease_source_.clear();
      lease_session_.clear();
      lease_id_.clear();
      auto_control_ = false;
      state_ = State::kSafeHold;
      safety_reason_ =
          "g20_lease_timeout";
      expired = true;
    }
  }

  /*
   * 释放状态锁后再触发物理安全停止。
   */
  if (expired) {
    SNAPPY_DRIVER_LOG_ERROR(
        "control",
        "G20 lease lost; entering SAFE_HOLD and stopping all "
        "actuators: gap_ms=" +
            std::to_string(
                expired_gap_ms) +
            " confirm_window_ms=" +
            std::to_string(
                confirm_window_ms));

    hub_.safe_stop();
  }
}

const char* ControlServer::state_name(
    State state) noexcept {
  switch (state) {
    case State::kBoot:
      return "BOOT";

    case State::kDisarmed:
      return "DISARMED";

    case State::kAcquired:
      return "ACQUIRED";

    case State::kArmedIdle:
      return "ARMED_IDLE";

    case State::kActive:
      return "ACTIVE";

    case State::kSafeHold:
      return "SAFE_HOLD";

    case State::kEstopLatched:
      return "ESTOP_LATCHED";

    case State::kFaultLatched:
      return "FAULT_LATCHED";
  }

  return "UNKNOWN";
}

std::uint64_t ControlServer::now_ms() {
  const auto now =
      std::chrono::system_clock::now();

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          now.time_since_epoch())
          .count());
}

}  // namespace snappy_snappy
