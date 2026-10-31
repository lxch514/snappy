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
#include <optional>

#include "snappy/gateway/diagnostics/gateway_log.hpp"

#include <nlohmann/json.hpp>

#include "snappy/gateway/g20/g20_packet_codec.hpp"

namespace snappy::gateway {
namespace {

constexpr auto kStatusIpcBackoff = std::chrono::milliseconds(250);

constexpr auto kTelemetryIpcBackoff = std::chrono::milliseconds(250);

constexpr auto kVideoStatePeriod = std::chrono::milliseconds(100);

/* 单轮入站报文批处理的时间上限。 */
constexpr auto kMaxInboundBatchDuration = std::chrono::milliseconds(100);

/* link_down 通知投递失败后的重试间隔。 */
constexpr auto kLinkDownRetryInterval = std::chrono::milliseconds(500);

/* 接收线程在传输层不可用时的等待时间。 */
constexpr auto kUnhealthyReceivePollInterval = std::chrono::milliseconds(50);

bool is_option(const std::string& value) {
  return value.rfind("--", 0) == 0;
}

bool payload_kind_is(const std::string& payload, const char* kind) {
  try {
    const auto root = nlohmann::json::parse(payload);

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
  if (updated_at.time_since_epoch().count() == 0) {
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
  root["robot_id"] = config_.robot_id;
  root["link_id"] = "g20_udp";
  root["source_id"] = "snappy-gateway";
  root["session_id"] = "gateway-local";
  root["message_id"] = action + "-" + std::to_string(timestamp);
  root["seq"] = 0;
  root["issued_at_ms"] = timestamp;
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

  std::lock_guard<std::mutex> lock(snapshot_mutex_);

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
  std::lock_guard<std::mutex> lock(snapshot_mutex_);

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
  std::lock_guard<std::mutex> lock(snapshot_mutex_);

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
  std::lock_guard<std::mutex> lock(snapshot_mutex_);

  status_inflight_ = false;

  if (response.has_value() && payload_kind_is(*response, "status")) {
    status_cache_.payload = std::move(*response);
    status_cache_.updated_at = now;
    status_cache_.ipc_online = true;
    status_cache_.detail = "control status snapshot current";

    status_retry_at_ = {};
  } else {
    /*
     * 保留上一次成功 payload。
     */
    status_cache_.ipc_online = false;
    status_cache_.detail = "control status IPC unavailable";

    status_retry_at_ = now + std::max(config_.status_period, kStatusIpcBackoff);
  }
}

void GatewayApp::complete_telemetry_query(
    std::optional<std::string> response,
    std::chrono::steady_clock::time_point
        now) {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);

  telemetry_inflight_ = false;

  if (response.has_value() && payload_kind_is(*response, "telemetry")) {
    telemetry_cache_.payload = std::move(*response);
    telemetry_cache_.updated_at = now;
    telemetry_cache_.ipc_online = true;
    telemetry_cache_.detail = "control telemetry snapshot current";

    telemetry_retry_at_ = {};
  } else {
    telemetry_cache_.ipc_online = false;
    telemetry_cache_.detail = "control telemetry IPC unavailable";

    telemetry_retry_at_ = now + std::max(config_.telemetry_period, kTelemetryIpcBackoff);
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
      std::unique_lock<std::mutex> lock(snapshot_mutex_);

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
      auto response = control_.request_snapshot(make_control_query("gateway.status_snapshot"));

      complete_status_query(
          std::move(response),
          std::chrono::steady_clock::now());

      continue;
    }

    if (work == Work::kTelemetry) {
      auto response = control_.request_snapshot(make_control_query("gateway.telemetry_snapshot"));

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
  std::lock_guard<std::mutex> lock(snapshot_mutex_);

  return status_cache_;
}

GatewayApp::CachedControlPayload
GatewayApp::cached_telemetry() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);

  return telemetry_cache_;
}

/* ============================================================
 * 发布状态缓存
 * ============================================================ */

bool GatewayApp::publish_control_status(
    bool g20_online) {
  const auto cached = cached_status();

  if (cached.payload.has_value()) {
    try {
      auto root = nlohmann::json::parse(*cached.payload);

      root["g20_online"] = g20_online;

      root["control_ipc_online"] = cached.ipc_online;

      root["control_snapshot_age_ms"] = snapshot_age_ms(cached.updated_at)

      root["control_snapshot_detail"] = cached.detail;

      return status_publisher_.publish_raw(transport_, g20::compact_control_snapshot(root.dump()));
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
  const auto cached = cached_telemetry();

  if (cached.payload.has_value()) {
    try {
      auto root = nlohmann::json::parse(*cached.payload);

      root["control_ipc_online"] = cached.ipc_online;

      root["control_snapshot_age_ms"] = snapshot_age_ms(cached.updated_at);

      root["control_snapshot_detail"] = cached.detail;

      return telemetry_publisher_.publish_raw(
          transport_,
          g20::compact_control_snapshot(root.dump()));
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
  response["robot_id"] = config_.robot_id;
  response["updated_at_ms"] = g20::wall_time_ms();
  response["timestamp_ms"] = response["updated_at_ms"];
  response["samples"] = nlohmann::json::array();
  response["control_ipc_online"] = false;
  response["control_snapshot_age_ms"] = 0;
  response["control_snapshot_detail"] = "no control telemetry snapshot";

  return telemetry_publisher_.publish_raw(transport_, response.dump());
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
            const auto text = value.get<std::string>();

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
            /* 传输层不可用时按 50ms 轮询，避免空转 CPU。 */
            if (!transport_.healthy()) {
              std::this_thread::sleep_for(
                  kUnhealthyReceivePollInterval);
            } else {
              std::this_thread::yield();
            }

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

            /* 可靠事件不能静默丢弃，返回可重试失败让 App 继续重试。 */
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

          /* 非 G20 周期包（acquire / arm / safe_hold / release）保持 FIFO。 */
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

  /* 屏幕控制权恢复命令可越过已排队的旧可靠报文。 */
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

  /* link_down 只有被 control 确认才算送达，否则下一轮重试。 */
  bool link_down_undelivered = false;

  /*
   * 内核判定"到对端路由不可达"的起始时刻（掉载波宽限窗口的计时起点）。
   * 在链路恢复或确认掉线时清空。
   */
  std::optional<std::chrono::steady_clock::time_point>
      route_lost_since;

  /*
   * 本轮掉载波是否已经确认并下过 link_down。
   * 只用于恢复日志的措辞，避免"已判掉线"却打印"未判掉线"。
   */
  bool route_loss_declared = false;

  auto next_link_down_attempt =
      std::chrono::steady_clock::now();

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

    /* 掉线下降沿使用带确认窗口的判定，避免偶发丢包误判。 */
    /* 掉线确认窗口 = max(配置值, 2 × 租约时间)。 */
    const auto g20_loss_confirm_window =
        std::max(
            config_.g20_loss_confirm,
            config_.g20_udp.lease_timeout * 2);

    /* 第二判据：发送路径被内核判定为不可达。 */
    const bool g20_route_gone =
        transport_.unreachable();

    /* 先告知本端链路可用性，再做在线判定。 */
    peer_.set_link_available(!g20_route_gone);

    /* 判定与留痕取自同一次采样。 */
    const auto g20_window =
        peer_.breakdown(
            config_.g20_udp.lease_timeout,
            g20_loss_confirm_window);

    const bool g20_confirmed_online =
        g20_window.online;

    bool g20_route_loss_confirmed = false;

    if (g20_route_gone) {
      if (!route_lost_since.has_value()) {
        route_lost_since = now;
        route_loss_declared = false;

        SNAPPY_GATEWAY_LOG_WARN(
            "g20.link",
            "peer route lost (sendto ENETUNREACH); riding through up to " +
                std::to_string(
                    config_.g20_route_loss_confirm.count()) +
                "ms before declaring link down");
      }

      g20_route_loss_confirmed =
          now - *route_lost_since >=
          config_.g20_route_loss_confirm;

      if (g20_route_loss_confirmed) {
        route_loss_declared = true;
      }
    } else if (route_lost_since.has_value()) {
      const auto held_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              now - *route_lost_since)
              .count();

      /* 区分“被容忍的闪断”与“已判掉线后的恢复”。 */
      const auto g20_blips =
          peer_.blip_stats();

      SNAPPY_GATEWAY_LOG_INFO(
          "g20.link",
          "peer route recovered after " +
              std::to_string(held_ms) + "ms" +
              " blips=" +
              std::to_string(g20_blips.count) +
              " blip_worst_ms=" +
              std::to_string(g20_blips.worst.count()) +
              " blip_total_ms=" +
              std::to_string(g20_blips.total.count()) +
              (route_loss_declared
                   ? " (link WAS declared down; "
                     "whole-machine SAFE_HOLD was already ordered)"
                   : "; link was NOT declared down (tolerated blip)"));

      route_lost_since.reset();
      route_loss_declared = false;
    }

    /* 上报给 App 的 g20_online 与安全判定同源。 */
    const bool g20_reported_online =
        g20_confirmed_online &&
        !g20_route_loss_confirmed;

    if (g20_reported_online) {
      /* 链路恢复留痕，便于与 DOWN 成对分析。 */
      if (!was_g20_online || link_down_undelivered) {
        SNAPPY_GATEWAY_LOG_INFO(
            "g20.link",
            "g20 link ONLINE: silence_ms=" +
                std::to_string(g20_window.raw.count()) +
                " effective_ms=" +
                std::to_string(g20_window.effective.count()) +
                " resume_grace_ms=" +
                std::to_string(g20_window.resume_grace.count()) +
                " confirm_window_ms=" +
                std::to_string(g20_window.confirm.count()));
      }

      was_g20_online = true;
      link_down_undelivered = false;
    } else if (was_g20_online) {
      if (config_.safe_stop_on_g20_loss) {
        /*
         * 只有上一轮投递失败时才需要等重试间隔；
         * 首次判定立即动作，不引入额外延迟。
         */
        if (now >= next_link_down_attempt) {
          const std::string reason =
              g20_route_gone
                  ? "peer route unreachable"
                  : "peer silence";

          /* 停机前留痕：记录原因与各段时长。 */
          SNAPPY_GATEWAY_LOG_WARN(
              "g20.link",
              "g20 link declared DOWN: reason=" +
                  reason +
                  " silence_ms=" +
                  std::to_string(g20_window.raw.count()) +
                  " link_unavailable_ms=" +
                  std::to_string(
                      g20_window.link_unavailable.count()) +
                  " resume_grace_ms=" +
                  std::to_string(g20_window.resume_grace.count()) +
                  " excluded_ms=" +
                  std::to_string(g20_window.excluded.count()) +
                  " effective_ms=" +
                  std::to_string(g20_window.effective.count()) +
                  " link_available=" +
                  (g20_route_gone ? "false" : "true") +
                  " confirm_window_ms=" +
                  std::to_string(g20_window.confirm.count()) +
                  " lease_timeout_ms=" +
                  std::to_string(config_.g20_udp.lease_timeout.count()) +
                  " blips=" +
                  std::to_string(peer_.blip_stats().count) +
                  " unreachable_sends=" +
                  std::to_string(
                      transport_.unreachable_send_count()) +
                  "; sending control.link_down (whole-machine SAFE_HOLD)");

          const bool delivered =
              control_.notify_link_down(
                  control::LinkId::kG20Udp);

          if (delivered) {
            was_g20_online = false;
            link_down_undelivered = false;
          } else {
            /*
             * 没送到就不能当作"已经处理过"。
             * 保持 was_g20_online=true 让下一轮重试。
             */
            link_down_undelivered = true;
            next_link_down_attempt =
                now + kLinkDownRetryInterval;

            SNAPPY_GATEWAY_LOG_ERROR(
                "g20.link",
                "control.link_down NOT delivered; "
                "keeping link marked down and retrying");
          }
        }
      } else {
        was_g20_online = false;
      }
    }

    if (now >= next_status) {
      schedule_status_query(now);

      static_cast<void>(
          publish_control_status(
              g20_reported_online));

      next_status =
          now + config_.status_period;
    }

    if (telemetry_enabled_ && now >= next_telemetry) {
      schedule_telemetry_query(now);

      static_cast<void>(publish_control_telemetry());

      next_telemetry = now + config_.telemetry_period;
    }

    if (now >= next_video_state) {
      static_cast<void>(router_.publish_video_state(transport_));

      next_video_state = now + kVideoStatePeriod;
    }

    std::this_thread::sleep_for(config_.loop_period);
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