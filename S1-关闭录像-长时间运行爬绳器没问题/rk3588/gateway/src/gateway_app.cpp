/**
 * @file gateway_app.cpp
 * @brief G20 gateway 主循环和 control 快照后台缓存实现
 *
 * @details
 *
 * - UDP 主循环不等待 status/telemetry IPC。
 * - ControlSnapshotWorker 独立查询 control。
 * - 查询失败时保留最近一次成功快照。
 * - 同一快照类型最多一个 pending/in-flight 请求。
 * - IPC 失败后进入退避，不会每轮重复请求。
 */

#include "snappy/gateway/gateway_app.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <deque>
#include <mutex>

#include "snappy/gateway/diagnostics/gateway_log.hpp"

#include <nlohmann/json.hpp>

#include "snappy/gateway/g20/g20_packet_codec.hpp"

namespace snappy::gateway {
namespace {

constexpr auto kStatusIpcBackoff =
    std::chrono::milliseconds(250);

constexpr auto kTelemetryIpcBackoff =
    std::chrono::milliseconds(250);

constexpr auto kVideoStatePeriod =
    std::chrono::milliseconds(100);

/*
 * 单轮入站报文批处理的时间上限。
 *
 * 每条往返 control 的命令都是一次同步 IPC，自带 deadline。
 * 原来固定批处理 8 条，最坏情况下主循环会连续几百毫秒
 * 走不到 router_.tick()，于是 control.heartbeat 无法生成，
 * control 侧租约超时后撤销控制权并整机安全停止。
 *
 * 加上时间预算后，每轮都会及时回到 router_.tick()，
 * 剩余报文留到下一轮继续处理，不丢报文。
 */
constexpr auto kMaxInboundBatchDuration =
    std::chrono::milliseconds(100);

bool is_option(
    const std::string& value) {
  return value.rfind("--", 0) == 0;
}

bool payload_kind_is(
    const std::string& payload,
    const char* kind) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    return
        root.is_object() &&
        root.value(
            "kind",
            "") == kind;
  } catch (...) {
    return false;
  }
}

std::uint64_t snapshot_age_ms(
    std::chrono::steady_clock::time_point
        updated_at) {
  if (updated_at.time_since_epoch().count() ==
      0) {
    return 0;
  }

  const auto now =
      std::chrono::steady_clock::now();

  if (now <= updated_at) {
    return 0;
  }

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          now - updated_at)
          .count());
}

}  // namespace

/* ============================================================
 * 命令行参数
 * ============================================================ */

GatewayAppOptions parse_gateway_app_options(
    int argc,
    char** argv) {
  GatewayAppOptions options;

  for (int index = 1;
       index < argc;
       ++index) {
    const std::string argument =
        argv[index];

    if (argument == "--help" ||
        argument == "-h") {
      options.show_help = true;
    } else if (
        argument == "--config" &&
        index + 1 < argc) {
      options.config_path =
          argv[++index];
    } else if (
        index == 1 &&
        !is_option(argument)) {
      options.config_path =
          argument;
    }
  }

  return options;
}

void print_gateway_app_usage(
    const char* argv0) {
  std::cout
      << "Usage: "
      << argv0
      << " [rk3588/config/hardware.snappy.json] "
         "[--config path]\n";
}

/* ============================================================
 * 构造与析构
 * ============================================================ */

GatewayApp::GatewayApp(
    GatewayRuntimeConfig config)
    : config_(std::move(config)),
      control_(
          config_.control_channel),
      video_(
          config_.video),
      transport_(
          config_.g20_udp,
          peer_),
      router_(
          config_.robot_id,
          control_,
          &video_),
      status_publisher_(
          config_.robot_id),
      telemetry_publisher_(
          config_.robot_id),
      telemetry_enabled_(
          config_.telemetry_enabled) {
  router_.set_telemetry_control(
      [this](bool enabled) {
        telemetry_enabled_ =
            enabled;
      });
}

GatewayApp::~GatewayApp() {
  stop_snapshot_worker();
}

/* ============================================================
 * control 查询命令
 * ============================================================ */

std::string GatewayApp::make_control_query(
    const std::string& action) const {
  nlohmann::json root;

  const auto timestamp =
      g20::wall_time_ms();

  root["schema_version"] = 1;
  root["kind"] = "command";
  root["robot_id"] =
      config_.robot_id;
  root["link_id"] =
      "g20_udp";
  root["source_id"] =
      "snappy-gateway";
  root["session_id"] =
      "gateway-local";
  root["message_id"] =
      action + "-" +
      std::to_string(timestamp);
  root["seq"] = 0;
  root["issued_at_ms"] =
      timestamp;
  root["ttl_ms"] = 250;
  root["control_epoch"] = 0;
  root["action"] = action;
  root["params"] =
      nlohmann::json::object();

  return root.dump();
}

/* ============================================================
 * 快照 worker 生命周期
 * ============================================================ */

void GatewayApp::start_snapshot_worker() {
  if (snapshot_worker_running_
          .exchange(true)) {
    return;
  }

  snapshot_worker_ =
      std::thread(
          [this]() {
            snapshot_worker_loop();
          });
}

void GatewayApp::stop_snapshot_worker()
    noexcept {
  if (!snapshot_worker_running_
           .exchange(false)) {
    return;
  }

  snapshot_cv_.notify_all();

  if (snapshot_worker_.joinable()) {
    snapshot_worker_.join();
  }

  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  status_pending_ = false;
  status_inflight_ = false;
  telemetry_pending_ = false;
  telemetry_inflight_ = false;
}

/* ============================================================
 * 查询调度
 * ============================================================ */

void GatewayApp::schedule_status_query(
    std::chrono::steady_clock::time_point
        now) {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  if (!snapshot_worker_running_.load() ||
      status_pending_ ||
      status_inflight_ ||
      now < status_retry_at_) {
    return;
  }

  status_pending_ = true;
  snapshot_cv_.notify_one();
}

void GatewayApp::schedule_telemetry_query(
    std::chrono::steady_clock::time_point
        now) {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  if (!snapshot_worker_running_.load() ||
      telemetry_pending_ ||
      telemetry_inflight_ ||
      now < telemetry_retry_at_) {
    return;
  }

  telemetry_pending_ = true;
  snapshot_cv_.notify_one();
}

/* ============================================================
 * 查询完成
 * ============================================================ */

void GatewayApp::complete_status_query(
    std::optional<std::string> response,
    std::chrono::steady_clock::time_point
        now) {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  status_inflight_ = false;

  if (response.has_value() &&
      payload_kind_is(
          *response,
          "status")) {
    status_cache_.payload =
        std::move(*response);
    status_cache_.updated_at =
        now;
    status_cache_.ipc_online =
        true;
    status_cache_.detail =
        "control status snapshot current";

    status_retry_at_ = {};
  } else {
    /*
     * 保留上一次成功 payload。
     */
    status_cache_.ipc_online =
        false;
    status_cache_.detail =
        "control status IPC unavailable";

    status_retry_at_ =
        now +
        std::max(
            config_.status_period,
            kStatusIpcBackoff);
  }
}

void GatewayApp::complete_telemetry_query(
    std::optional<std::string> response,
    std::chrono::steady_clock::time_point
        now) {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  telemetry_inflight_ = false;

  if (response.has_value() &&
      payload_kind_is(
          *response,
          "telemetry")) {
    telemetry_cache_.payload =
        std::move(*response);
    telemetry_cache_.updated_at =
        now;
    telemetry_cache_.ipc_online =
        true;
    telemetry_cache_.detail =
        "control telemetry snapshot current";

    telemetry_retry_at_ = {};
  } else {
    telemetry_cache_.ipc_online =
        false;
    telemetry_cache_.detail =
        "control telemetry IPC unavailable";

    telemetry_retry_at_ =
        now +
        std::max(
            config_.telemetry_period,
            kTelemetryIpcBackoff);
  }
}

/* ============================================================
 * 快照 worker
 * ============================================================ */

void GatewayApp::snapshot_worker_loop() {
  while (snapshot_worker_running_.load()) {
    enum class Work {
      kNone,
      kStatus,
      kTelemetry,
    };

    Work work = Work::kNone;

    {
      std::unique_lock<std::mutex> lock(
          snapshot_mutex_);

      snapshot_cv_.wait(
          lock,
          [this]() {
            return
                !snapshot_worker_running_.load() ||
                status_pending_ ||
                telemetry_pending_;
          });

      if (!snapshot_worker_running_.load()) {
        break;
      }

      /*
       * 状态优先于遥测。
       * 两者始终只允许一个 in-flight。
       */
      if (status_pending_) {
        status_pending_ = false;
        status_inflight_ = true;
        work = Work::kStatus;
      } else if (telemetry_pending_) {
        telemetry_pending_ = false;
        telemetry_inflight_ = true;
        work = Work::kTelemetry;
      }
    }

    if (work == Work::kStatus) {
      auto response =
          control_.request_snapshot(
              make_control_query(
                  "gateway.status_snapshot"));

      complete_status_query(
          std::move(response),
          std::chrono::steady_clock::now());

      continue;
    }

    if (work == Work::kTelemetry) {
      auto response =
          control_.request_snapshot(
              make_control_query(
                  "gateway.telemetry_snapshot"));

      complete_telemetry_query(
          std::move(response),
          std::chrono::steady_clock::now());
    }
  }
}

/* ============================================================
 * 缓存复制
 * ============================================================ */

GatewayApp::CachedControlPayload
GatewayApp::cached_status() const {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  return status_cache_;
}

GatewayApp::CachedControlPayload
GatewayApp::cached_telemetry() const {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  return telemetry_cache_;
}

/* ============================================================
 * 发布状态缓存
 * ============================================================ */

bool GatewayApp::publish_control_status(
    bool g20_online) {
  const auto cached =
      cached_status();

  if (cached.payload.has_value()) {
    try {
      auto root =
          nlohmann::json::parse(
              *cached.payload);

      root["g20_online"] =
          g20_online;

      root["control_ipc_online"] =
          cached.ipc_online;

      root["control_snapshot_age_ms"] =
          snapshot_age_ms(
              cached.updated_at);

      root["control_snapshot_detail"] =
          cached.detail;

      return status_publisher_.publish_raw(
          transport_,
          root.dump());
    } catch (...) {
    }
  }

  /*
   * 尚未得到任何 control 状态。
   * 与“传感器缓存为空”使用不同 detail。
   */
  return status_publisher_.publish(
      transport_,
      StatusSnapshot{
          control::ControlState::kSafeHold,
          false,
          g20_online,
          false,
          "gateway online; no control status snapshot",
      });
}

/* ============================================================
 * 发布遥测缓存
 * ============================================================ */

bool GatewayApp::publish_control_telemetry() {
  const auto cached =
      cached_telemetry();

  if (cached.payload.has_value()) {
    try {
      auto root =
          nlohmann::json::parse(
              *cached.payload);

      root["control_ipc_online"] =
          cached.ipc_online;

      root["control_snapshot_age_ms"] =
          snapshot_age_ms(
              cached.updated_at);

      root["control_snapshot_detail"] =
          cached.detail;

      return telemetry_publisher_.publish_raw(
          transport_,
          root.dump());
    } catch (...) {
    }
  }

  /*
   * 明确表示 IPC 尚未取得遥测。
   * 不调用 control，不触发传感器读取。
   */
  nlohmann::json response;

  response["schema_version"] = 1;
  response["kind"] = "telemetry";
  response["robot_id"] =
      config_.robot_id;
  response["updated_at_ms"] =
      g20::wall_time_ms();
  response["timestamp_ms"] =
      response["updated_at_ms"];
  response["samples"] =
      nlohmann::json::array();
  response["control_ipc_online"] =
      false;
  response["control_snapshot_age_ms"] =
      0;
  response["control_snapshot_detail"] =
      "no control telemetry snapshot";

  return telemetry_publisher_.publish_raw(
      transport_,
      response.dump());
}

/* ============================================================
 * Gateway 主循环
 * ============================================================ */

int GatewayApp::run(
    std::atomic_bool& running) {
  if (!transport_.start()) {
    std::cerr
        << "{\"process\":\"snappy-gateway\","
        << "\"state\":\"configure_error\","
        << "\"detail\":\"G20 UDP transport "
           "failed to start\"}\n";
    return 2;
  }

  start_snapshot_worker();

  std::deque<InboundMessage> reliable_queue;
  std::deque<InboundMessage> command_queue;
  std::optional<InboundMessage> latest_normal_channel;

  constexpr std::size_t kReliableQueueCapacity = 64;
  constexpr std::size_t kCommandQueueCapacity = 192;

  std::mutex inbound_mutex;
  std::atomic_bool receive_running{true};

  std::size_t reliable_drop_count = 0;
  std::size_t normal_drop_count = 0;

  enum class InboundClass {
    kCommand,
    kReliableChannel,
    kNormalChannel,
  };

  const auto json_bool =
      [](const nlohmann::json& root,
         const char* key,
         bool fallback) {
        if (!root.contains(key)) {
          return fallback;
        }

        try {
          const auto& value = root.at(key);

          if (value.is_boolean()) {
            return value.get<bool>();
          }

          if (value.is_number()) {
            return value.get<double>() != 0.0;
          }

          if (value.is_string()) {
            const auto text =
                value.get<std::string>();

            return text == "true" ||
                   text == "TRUE" ||
                   text == "yes" ||
                   text == "YES" ||
                   text == "1" ||
                   text == "reliable" ||
                   text == "RELIABLE";
          }
        } catch (...) {
        }

        return fallback;
      };

  const auto classify =
      [&](const InboundMessage& inbound) {
        try {
          const auto root =
              nlohmann::json::parse(
                  inbound.payload);

          if (!root.is_object()) {
            return InboundClass::kCommand;
          }

          const std::string kind =
              root.value("kind", "");

          const std::string type =
              root.value("type", "");

          const bool has_channels =
              root.contains("channels") &&
              root.at("channels").is_array();

          const bool has_buttons =
              root.contains("buttons") &&
              root.at("buttons").is_array();

          const bool is_g20_packet =
              kind == "g20_channels" ||
              kind == "channels" ||
              type == "g20_channels" ||
              type == "channels" ||
              type == "G20" ||
              type == "buttons" ||
              type == "rc_buttons";

          if (!is_g20_packet ||
              (!has_channels && !has_buttons)) {
            return InboundClass::kCommand;
          }

          const bool reliable =
              json_bool(
                  root,
                  "reliable",
                  false);

          /*
           * 与 G20CommandRouter 使用相同的可靠包判定：
           * 可靠包必须使用 channels 数组。
           */
          if (reliable &&
              has_channels &&
              (kind == "g20_channels" ||
               kind == "channels" ||
               type == "g20_channels" ||
               type == "channels" ||
               type == "G20" ||
               root.contains("message_id"))) {
            return InboundClass::kReliableChannel;
          }

          return InboundClass::kNormalChannel;
        } catch (...) {
          return InboundClass::kCommand;
        }
      };

  std::thread receive_thread(
      [&]() {
        while (receive_running.load()) {
          auto inbound =
              transport_.receive();

          if (!inbound.has_value()) {
            std::this_thread::yield();
            continue;
          }

          const auto inbound_class =
              classify(*inbound);

          if (inbound_class ==
              InboundClass::kReliableChannel) {
            bool queued = false;

            {
              std::lock_guard<std::mutex> lock(
                  inbound_mutex);

              if (reliable_queue.size() <
                  kReliableQueueCapacity) {
                reliable_queue.push_back(
                    std::move(*inbound));
                queued = true;
              }
            }

            if (queued) {
              continue;
            }

            /*
             * 可靠事件不能静默丢弃。
             * 直接返回 FAILED,retryable=true，
             * 让 Android 保留当前可靠事件并继续重试。
             */
            bool ack_published = false;
            std::string message_id;

            try {
              const auto root =
                  nlohmann::json::parse(
                      inbound->payload);

              message_id =
                  root.value(
                      "message_id",
                      root.value("id", ""));

              const auto ack_payload =
                  g20::encode_ack(
                      config_.robot_id,
                      root,
                      "FAILED",
                      "reliable gateway queue full",
                      true);

              ack_published =
                  transport_.publish(
                      PublishedMessage{
                          "snappy/v1/robots/" +
                              config_.robot_id +
                              "/up/ack",
                          ack_payload,
                          1,
                          false,
                      });
            } catch (...) {
              /*
               * classify() 已经成功解析过可靠包，
               * 这里失败只记录，不影响接收线程。
               */
            }

            ++reliable_drop_count;

            SNAPPY_GATEWAY_LOG_ERROR(
                "g20.queue",
                "reliable queue full; "
                "message_id=" +
                    message_id +
                    " count=" +
                    std::to_string(
                        reliable_drop_count) +
                    " ack=" +
                    (ack_published
                         ? "sent"
                         : "failed"));

            continue;
          }

          if (inbound_class ==
              InboundClass::kNormalChannel) {
            std::lock_guard<std::mutex> lock(
                inbound_mutex);

            /*
             * 普通周期舵量采用最新状态语义。
             * 新状态覆盖旧状态，不影响控制命令队列。
             */
            if (latest_normal_channel
                    .has_value()) {
              ++normal_drop_count;
            }

            latest_normal_channel =
                std::move(*inbound);

            continue;
          }

          /*
           * 非 G20 周期包，例如 acquire、arm、
           * safe_hold、release 等，保持 FIFO。
           * 不把这些按钮命令当作普通状态覆盖。
           */
          {
            std::lock_guard<std::mutex> lock(
                inbound_mutex);

            if (command_queue.size() >=
                kCommandQueueCapacity) {
              command_queue.pop_front();
              ++normal_drop_count;
            }

            command_queue.push_back(
                std::move(*inbound));
          }
        }
      });

  /*
   * 屏幕控制权恢复命令必须能够越过已经排队的旧可靠报文，
   * 但每个主循环仍先给可靠队列一次机会，确保刚到达的 CH13
   * 安全停止不会被屏幕恢复命令抢先。
   */
  const auto is_screen_recovery_command =
      [](const InboundMessage& inbound) {
        try {
          const auto root =
              nlohmann::json::parse(
                  inbound.payload);

          if (!root.is_object()) {
            return false;
          }

          const std::string action =
              root.value("action", "");

          return action == "control.acquire" ||
                 action == "control.arm";
        } catch (...) {
          return false;
        }
      };

  bool reliable_serviced_this_cycle = false;

  auto try_pop_inbound =
      [&]() -> std::optional<InboundMessage> {
    std::lock_guard<std::mutex> lock(
        inbound_mutex);

    /*
     * 每轮先处理一个可靠报文，保证 CH13 安全停止有机会执行。
     */
    if (!reliable_serviced_this_cycle &&
        !reliable_queue.empty()) {
      reliable_serviced_this_cycle = true;

      InboundMessage inbound =
          std::move(reliable_queue.front());

      reliable_queue.pop_front();
      return inbound;
    }

    /*
     * 控制权恢复命令优先于剩余的旧可靠报文。
     * 只提升 acquire / arm，不改变其他普通命令的 FIFO 顺序。
     */
    for (auto it = command_queue.begin();
         it != command_queue.end();
         ++it) {
      if (!is_screen_recovery_command(*it)) {
        continue;
      }

      InboundMessage inbound =
          std::move(*it);

      command_queue.erase(it);
      return inbound;
    }

    if (!reliable_queue.empty()) {
      InboundMessage inbound =
          std::move(reliable_queue.front());

      reliable_queue.pop_front();
      return inbound;
    }

    if (!command_queue.empty()) {
      InboundMessage inbound =
          std::move(command_queue.front());

      command_queue.pop_front();
      return inbound;
    }

    if (latest_normal_channel.has_value()) {
      InboundMessage inbound =
          std::move(*latest_normal_channel);

      latest_normal_channel.reset();
      return inbound;
    }

    return std::nullopt;
  };

  auto next_status =
      std::chrono::steady_clock::now();

  auto next_telemetry =
      next_status;

  auto next_video_state =
      next_status;

  bool was_g20_online = false;

  while (running.load()) {
    reliable_serviced_this_cycle = false;
    /*
     * 只有当前线程访问 router_。
     * 接收线程只负责 receive、分类和入队。
     */
    const auto inbound_batch_started =
        std::chrono::steady_clock::now();

    for (int count = 0; count < 8; ++count) {
      /*
       * 至少处理一条（保证队列有推进），
       * 之后按时间预算让出主循环，保证心跳及时发送。
       */
      if (count > 0 &&
          std::chrono::steady_clock::now() -
                  inbound_batch_started >=
              kMaxInboundBatchDuration) {
        break;
      }

      auto inbound =
          try_pop_inbound();

      if (!inbound.has_value()) {
        break;
      }

      router_.route(
          *inbound,
          transport_);
    }

    router_.tick(transport_);

    const auto now =
        std::chrono::steady_clock::now();

    const bool g20_online =
        peer_.online(
            config_.g20_udp.lease_timeout);

    /*
     * 掉线下降沿必须使用带确认窗口的判定。
     *
     * G20 遥控数据走 UDP，偶发丢包是正常的；用单次采样
     * 直接判定掉线会误触发 link_down，把已经拿到的控制权
     * 无谓撤销，表现为“控制模块忽然不能控制”。
     */
    const bool g20_confirmed_online =
        peer_.online_confirmed(
            config_.g20_udp.lease_timeout);

    if (config_.safe_stop_on_g20_loss &&
        was_g20_online &&
        !g20_confirmed_online) {
      control_.notify_link_down(
          control::LinkId::kG20Udp);
    }

    was_g20_online = g20_confirmed_online;

    if (now >= next_status) {
      schedule_status_query(now);

      static_cast<void>(
          publish_control_status(
              g20_online));

      next_status =
          now + config_.status_period;
    }

    if (telemetry_enabled_ &&
        now >= next_telemetry) {
      schedule_telemetry_query(now);

      static_cast<void>(
          publish_control_telemetry());

      next_telemetry =
          now + config_.telemetry_period;
    }

    if (now >= next_video_state) {
      static_cast<void>(
          router_.publish_video_state(
              transport_));

      next_video_state =
          now + kVideoStatePeriod;
    }

    std::this_thread::sleep_for(
        config_.loop_period);
  }

  receive_running.store(false);

  if (receive_thread.joinable()) {
    receive_thread.join();
  }

  stop_snapshot_worker();
  video_.stop();
  transport_.stop();

  SNAPPY_GATEWAY_LOG_INFO(
      "g20.queue",
      "receive queue stopped "
      "reliable_drops=" +
          std::to_string(
              reliable_drop_count) +
          " normal_drops=" +
          std::to_string(
              normal_drop_count));

  std::cout
      << "{\"process\":\"snappy-gateway\","
      << "\"state\":\"stopped\","
      << "\"detail\":\"snappy-gateway stopped\"}\n";

  return 0;
}

}  // namespace snappy::gateways