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
 * - 链路丢失不再收回控制租约，也不会自动 arm。
 */

#include "snappy_snappy/control_server.hpp"

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <utility>

/*
 * 本文件在 tick() 里使用 SNAPPY_DRIVER_LOG_WARN。
 * 原有 include 链（control_server.hpp -> actuator_hub.hpp -> drivers/*）
 * 里没有 driver_log.hpp，必须显式包含，否则编不过。
 */
#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy_snappy {

namespace {

/*
 * ============================================================
 * 链路丢失处理策略
 * ============================================================
 *
 * kRetainLeaseOnLinkLoss = true（当前默认）：
 *
 * 1. 链路丢失 / 租约超时后，执行器照常“立即物理停止”，
 *    状态进入 SAFE_HOLD —— 安全语义与原来完全一致。
 *
 * 2. 但不再收回控制租约。
 *    操作员按一次“控制权”时 control.acquire 会立即成功，
 *    不会再出现 “control lease is invalid”，
 *    也不需要等待原来的 5 秒安全过渡超时。
 *
 * 3. 失联超过 g20_lease_release_after() 后仍然真正释放租约，
 *    避免永久失联的旧会话占着控制权，其他会话无法接管。
 *
 * false：与旧行为完全一致（租约超时立即清空控制租约）。
 *
 * ------------------------------------------------------------
 * 为什么不做“链路恢复后自动回到 ARMED_IDLE”
 * ------------------------------------------------------------
 * 遥控端在摇杆保持离中时，会对风机和爬绳器周期性重复发令：
 *   g20_button_mapper.cpp 的 handleFanX / handleWheel
 *   按 kRepeatInterval 循环调用 adjustFan / adjustClimber，
 *   对应 fan.left_set / fan.right_set / climber.ascend / descend。
 *
 * 若链路恢复后自动回到 ARMED_IDLE 并打开命令门，
 * 风机和爬绳器会在操作员没有任何新动作的情况下自行恢复运行。
 * 因此这里保持“停机 + 等人工确认恢复”。
 */
constexpr bool kRetainLeaseOnLinkLoss = true;

/*
 * 租约释放倍数与下限：
 * 失联超过 max(倍数 * g20_lease_timeout, 下限) 才真正释放租约。
 */
constexpr std::int64_t kG20LeaseReleaseFactor = 3;

constexpr auto kG20LeaseReleaseFloor =
    std::chrono::seconds(10);

std::chrono::milliseconds g20_lease_release_after(
    std::chrono::milliseconds lease_timeout) {
  const auto scaled =
      lease_timeout * kG20LeaseReleaseFactor;

  return scaled > kG20LeaseReleaseFloor
             ? scaled
             : kG20LeaseReleaseFloor;
}

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
      socket_path_(config_.socket_path) {}

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
    link_hold_ = false;
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
     * serve_client 内部有固定 50 ms 超时，
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

  /*
   * 超时由 200ms 收紧到 50ms。
   *
   * 接受线程是串行的：任何“连上但不发数据”的客户端如果占用
   * 200ms，就会顺延后面所有请求，包括 250ms 周期的心跳。
   * 心跳被饿死会让 tick() 误判 G20 租约超时。
   *
   * 不能取得更小：网关侧 perform_request 是 connect 成功后
   * 紧接着 send，若这里只有 10ms，在 CPU 被抢占的瞬间
   * 服务端可能已经 close，导致请求静默丢失。
   * 若现场出现请求偶发失败，把 50 改回 200。
   */
  if (::poll(
          &descriptor,
          1,
          50) <= 0) {
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

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (lease_held_ &&
          (lease_source_ != source ||
           lease_session_ != session)) {
        return make_ack(
           message,
           "FAILED",
           "control lease is held by another session");
      }
    }

    /*
     * 安全停止过渡最多等 1500ms。
     *
     * 本请求在串行的接受线程里同步执行，等待过久会让后续的
     * control.heartbeat 无法及时处理，反而把 G20 租约拖超时。
     * 1500ms 远小于 g20_lease_timeout，失败时操作员再按一次即可。
     */
    constexpr auto kSafetyTransitionTimeout =
        std::chrono::milliseconds(1500);

    if (!hub_.wait_for_safety_clear(
          kSafetyTransitionTimeout)) {
      return make_ack(
        message,
        "FAILED",
        "actuator safety stop is still pending or failed");
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (lease_held_ &&
          (lease_source_ != source ||
           lease_session_ != session)) {
        return make_ack(
            message,
            "FAILED",
            "control lease is held by another session");
      }
      lease_held_ = true;
      lease_source_ = source;
      lease_session_ = session;

      lease_id_ =
          "lease-" +
          std::to_string(
              ++control_epoch_);

      last_g20_seen_ =
          std::chrono::steady_clock::now();

      /*
       * 操作员明确恢复控制：清掉链路挂起标志。
       */
      link_hold_ = false;

      state_ = State::kAcquired;
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
        /*
         * 心跳只刷新存活时间。
         *
         * 这里刻意不做任何“自动恢复”：
         * 执行器保持停止，状态保持 SAFE_HOLD，
         * 必须由操作员按“控制权”确认后才恢复。
         */
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
     * enable_commands 只打开 ActuatorHub 命令门，
     * 不在这里执行设备 I/O。
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
       * enable_commands 返回后再次验证租约和状态，
       * 防止 arm 期间控制权已被安全停止逻辑撤销。
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
      link_hold_ = false;
      last_g20_seen_ =
          std::chrono::steady_clock::now();
    }

    return make_ack(
        message,
        "SUCCEEDED",
        "control armed");
  }

  /*
   * safe_hold、disarm、estop、release 和 link_down
   * 都必须进入执行器安全停止流程。
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
       * link_down 是通信丢失保护动作，
       * 即使租约已经失效也必须允许执行。
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

    if (link_down) {
      SNAPPY_DRIVER_LOG_WARN(
          "control",
          "control.link_down received -> SAFE_HOLD; "
          "actuators stopped, control lease retained");
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

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

      if (link_down) {
        /*
         * 链路丢失只挂起，不收回控制租约。
         *
         * 执行器已经物理停止，安全语义不变；
         * 操作员随后按“控制权”时 acquire 会立即成功，
         * 不会出现 “control lease is invalid”。
         *
         * kRetainLeaseOnLinkLoss 为 false 时保持旧行为。
         */
        if (kRetainLeaseOnLinkLoss) {
          link_hold_ = true;
        } else {
          lease_held_ = false;
          lease_source_.clear();
          lease_session_.clear();
          lease_id_.clear();
        }
      }

      if (action == "control.release") {
        /*
         * release 是操作员主动交还控制权，
         * 与链路丢失无关，仍然收回租约。
         */
        lease_held_ = false;
        lease_source_.clear();
        lease_session_.clear();
        lease_id_.clear();
        link_hold_ = false;
      }
    }

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

    safety_reason_ = "fault.clear";

    /*
     * 原实现只改 safety_reason_，state_ 仍停留在 SAFE_HOLD，
     * 面板复位后状态没有真正恢复。
     *
     * 现在把安全保持状态一并解除（含链路丢失挂起）。
     * ESTOP_LATCHED / FAULT_LATCHED 不受影响，
     * 仍然必须人工处理。
     */
    if (state_ == State::kSafeHold) {
      state_ = State::kDisarmed;
      link_hold_ = false;
    }
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

  response["ready_for_arm"] =
      snapshot.ready_for_arm &&
      state == State::kAcquired;

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
   * Android 端继续收到 sensors 字段，
   * 但执行器专用进程不创建任何传感器对象。
   */
  response["sensors"]["disabled"] =
      true;
  response["sensors"]["mode"] =
      "actuator_only";
  response["sensors"]["cache_count"] =
      0;
  response["sensors"]["samples"] =
      json::array();
  response["sensors"]["devices"] =
      json::array();

  response["watchdog"]["enabled"] =
      hardware.watchdog.enabled;
  response["watchdog"]["active"] =
      hardware.watchdog.enabled;
  response["watchdog"]["healthy"] =
      hardware.watchdog.enabled;;

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
   * 遥测接口只读取执行器缓存快照。
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
      "actuator_only";
  response["sensors_disabled"] =
      true;

  /*
   * TelemetryEnvelope 保留 samples 数组，
   * 但执行器专用版本不会填入传感器数据。
   */
  response["samples"] =
      json::array();

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
      true;

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
  bool need_safe_stop = false;
  bool log_transition = false;
  bool lease_released = false;
  std::int64_t age_ms = 0;
  std::int64_t timeout_ms = 0;
  std::int64_t release_after_ms = 0;

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

    const auto timeout =
        config_.hardware.safety.g20_lease_timeout;

    if (age <= timeout) {
      return;
    }

    age_ms =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(age)
            .count();

    timeout_ms = timeout.count();

    const auto release_after =
        g20_lease_release_after(timeout);

    release_after_ms = release_after.count();

    const bool first_detection = !link_hold_;

    log_transition = first_detection;

    if (!kRetainLeaseOnLinkLoss) {
      /*
       * 旧行为：租约超时立即清空控制租约，
       * 必须人工重新获取控制权。
       */
      lease_held_ = false;
      lease_source_.clear();
      lease_session_.clear();
      lease_id_.clear();
      link_hold_ = false;
      state_ = State::kSafeHold;
      safety_reason_ = "g20_lease_timeout";
      lease_released = true;
      need_safe_stop = true;
    } else if (age >= release_after) {
      /*
       * 第二阶段：失联时间已经超过释放窗口。
       *
       * 真正释放控制租约，避免一条永久失联的旧会话
       * 一直占着控制权，其他会话无法接管。
       */
      link_hold_ = false;
      lease_held_ = false;
      lease_source_.clear();
      lease_session_.clear();
      lease_id_.clear();
      state_ = State::kSafeHold;
      safety_reason_ = "g20_lease_timeout_released";
      lease_released = true;
      need_safe_stop = first_detection;
    } else if (first_detection) {
      /*
       * 第一阶段：刚超过租约时间。
       *
       * 执行器必须停止（安全语义不变），状态进入 SAFE_HOLD；
       * 但控制租约保留，所以操作员随后按“控制权”时
       * acquire 会立即成功。
       */
      link_hold_ = true;
      state_ = State::kSafeHold;
      safety_reason_ = "g20_lease_timeout";
      need_safe_stop = true;
    }
  }

  /*
   * 日志与物理停止都放在状态锁之外。
   *
   * 持有状态锁时做日志 I/O 会阻塞并发的
   * control.heartbeat，反而制造新的租约超时。
   */
  if (log_transition) {
    SNAPPY_DRIVER_LOG_WARN(
        "control",
        "G20 lease timeout -> SAFE_HOLD, "
        "age_ms=" +
            std::to_string(age_ms) +
            " timeout_ms=" +
            std::to_string(timeout_ms) +
            " release_after_ms=" +
            std::to_string(release_after_ms) +
            " lease_released=" +
            (lease_released ? "true" : "false"));
  }

  if (need_safe_stop) {
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
