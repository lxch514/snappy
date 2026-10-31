#pragma once

/**
 * @file control_server.hpp
 * @brief 与原 Gateway IPC 兼容的无传感器控制服务
 *
 * @details
 * - 使用原 Unix SOCK_SEQPACKET socket。
 * - 保持原 action、参数和 ACK 字段。
 * - 不创建命令 FIFO。
 * - status/telemetry 只复制 ActuatorHub 快照。
 * - 不使用 DeviceRegistry、SensorService、
 *   GatewayCommandDispatcher 作为运行时核心。
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "snappy_snappy/actuator_hub.hpp"

namespace snappy_snappy {

class ControlServer {
 public:
  ControlServer(
      ActuatorHub& hub,
      RuntimeConfig config);

  ~ControlServer();

  ControlServer(const ControlServer&) = delete;
  ControlServer& operator=(
      const ControlServer&) = delete;

  bool start();

  void stop() noexcept;

  /*
   * 主循环周期调用。
   * 这里只处理控制租约超时和安全停止，
   * 不执行任何物理总线查询。
   */
  void tick();

 private:
  using json = nlohmann::json;

  enum class State {
    kBoot,
    kDisarmed,
    kAcquired,
    kArmedIdle,
    kActive,
    kSafeHold,
    kEstopLatched,
    kFaultLatched,
  };

  void accept_loop();

  void serve_client(
      int client_fd);

  std::string handle_request(
      const std::string& request);

  std::string make_ack(
      const json& request,
      const std::string& status,
      const std::string& detail);

  std::string make_status() const;

  std::string make_telemetry() const;

  std::string make_capabilities() const;

  bool lease_valid_locked(
      const json& request) const;

  static const char* state_name(
      State state) noexcept;

  static std::uint64_t now_ms();

  ActuatorHub& hub_;

  RuntimeConfig config_;

  std::atomic_bool running_{false};

  std::thread accept_thread_;

  /*
   * 仅由 start/stop/accept_loop 使用。
   * 客户端处理线程不会访问此字段。
   */
  int server_fd_{-1};

  std::string socket_path_;

  mutable std::recursive_mutex mutex_;

  State state_{State::kBoot};

  bool lease_held_{false};

  std::string lease_source_;

  std::string lease_session_;

  std::string lease_id_;

  std::uint64_t control_epoch_{0};

  std::chrono::steady_clock::time_point
      last_g20_seen_{};

  std::string safety_reason_;
};

}  // namespace snappy_snappy