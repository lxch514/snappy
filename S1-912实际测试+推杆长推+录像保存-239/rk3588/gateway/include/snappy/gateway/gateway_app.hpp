#pragma once

/**
 * @file gateway_app.hpp
 * @brief G20 网关主循环和 control 快照后台 worker
 *
 * @details
 *
 * - UDP 接收、遥控路由和心跳不执行周期快照 IPC。
 * - 独立 worker 查询 control 状态和遥测。
 * - 同一类型最多一个 pending/in-flight 请求。
 * - IPC 超时保留最近一次成功快照并进入退避。
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "snappy/gateway/control_channel.hpp"
#include "snappy/gateway/g20/g20_command_router.hpp"
#include "snappy/gateway/g20/g20_udp_transport.hpp"
#include "snappy/gateway/g20/g20_video.hpp"
#include "snappy/gateway/gateway_config.hpp"
#include "snappy/gateway/peer_tracker.hpp"
#include "snappy/gateway/status_publisher.hpp"
#include "snappy/gateway/telemetry_publisher.hpp"

namespace snappy::gateway {

struct GatewayAppOptions {
  std::string config_path{
      "rk3588/config/hardware.snappy.json"};

  bool show_help{false};
};

GatewayAppOptions parse_gateway_app_options(
    int argc,
    char** argv);

void print_gateway_app_usage(
    const char* argv0);

class GatewayApp {
 public:
  explicit GatewayApp(
      GatewayRuntimeConfig config);

  ~GatewayApp();

  GatewayApp(
      const GatewayApp&) = delete;

  GatewayApp& operator=(
      const GatewayApp&) = delete;

  int run(std::atomic_bool& running);

 private:
  struct CachedControlPayload {
    std::optional<std::string> payload;
    std::chrono::steady_clock::time_point
        updated_at{};

    bool ipc_online{false};
    std::string detail{
        "control snapshot not received"};
  };

  std::string make_control_query(
      const std::string& action) const;

  void start_snapshot_worker();
  void stop_snapshot_worker() noexcept;
  void snapshot_worker_loop();

  void schedule_status_query(
      std::chrono::steady_clock::time_point now);

  void schedule_telemetry_query(
      std::chrono::steady_clock::time_point now);

  void complete_status_query(
      std::optional<std::string> response,
      std::chrono::steady_clock::time_point now);

  void complete_telemetry_query(
      std::optional<std::string> response,
      std::chrono::steady_clock::time_point now);

  [[nodiscard]] CachedControlPayload
  cached_status() const;

  [[nodiscard]] CachedControlPayload
  cached_telemetry() const;

  bool publish_control_status(
      bool g20_online);

  bool publish_control_telemetry();

  GatewayRuntimeConfig config_;

  PeerTracker peer_;
  SeqpacketControlChannel control_;
  g20::G20VideoAdapter video_;
  g20::G20UdpTransport transport_;
  g20::G20CommandRouter router_;

  StatusPublisher status_publisher_;
  TelemetryPublisher telemetry_publisher_;

  bool telemetry_enabled_{true};

  std::atomic_bool snapshot_worker_running_{
      false};

  std::thread snapshot_worker_;

  mutable std::mutex snapshot_mutex_;
  std::condition_variable snapshot_cv_;

  bool status_pending_{false};
  bool status_inflight_{false};

  bool telemetry_pending_{false};
  bool telemetry_inflight_{false};

  std::chrono::steady_clock::time_point
      status_retry_at_{};

  std::chrono::steady_clock::time_point
      telemetry_retry_at_{};

  CachedControlPayload status_cache_;
  CachedControlPayload telemetry_cache_;
};

}  // namespace snappy::gateway