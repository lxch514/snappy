#pragma once

/**
 * @file actuator_hub.hpp
 * @brief 无传感器执行器控制中心
 *
 * @details
 * - 只管理毛刷、推杆、爬绳器和双风机。
 * - 不创建 DeviceRegistry。
 * - 不创建 SensorService。
 * - 不创建 GatewayCommandDispatcher。
 * - 不读取任何传感器。
 * - 每个执行器控制域拥有独立 worker。
 * - 普通命令使用最新值邮箱，不使用 FIFO 命令队列。
 * - RS485 事务由 DirectRs485Bus 按物理端口独立保护。
 * - status/telemetry 只读取内存快照。
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "snappy/control/actuator_service.hpp"
#include "snappy/control/hardware_config.hpp"
#include "snappy/drivers/bus/can_socket_bus.hpp"
#include "snappy/drivers/devices/brush_motor.hpp"
#include "snappy/drivers/devices/climber.hpp"
#include "snappy/drivers/devices/propulsion_fan.hpp"
#include "snappy/drivers/devices/push_motor.hpp"
#include "snappy/drivers/devices/relay.hpp"

#include "snappy_snappy/actuator_snapshot.hpp"
#include "snappy_snappy/command_mailbox.hpp"
#include "snappy_snappy/direct_rs485_bus.hpp"

namespace snappy_snappy {

struct RuntimeConfig {
  snappy::control::HardwareConfig hardware;

  std::string socket_path{
      "/run/snappy/control.sock"};
};

class ActuatorHub {
 public:
  explicit ActuatorHub(
      RuntimeConfig config);

  ~ActuatorHub();

  ActuatorHub(const ActuatorHub&) = delete;
  ActuatorHub& operator=(
      const ActuatorHub&) = delete;

  snappy::drivers::Result start();

  void stop() noexcept;

  /*
   * 只接受协议动作并写入对应的最新值邮箱，
   * 或修改风机驱动的目标缓存。
   */
  snappy::drivers::Result submit(
      const std::string& action,
      const nlohmann::json& params);

  /*
   * 关闭命令门、递增 generation、取消未执行命令，
   * 并由 maintenance worker 执行最终物理安全停止。
   */
  void safe_stop() noexcept;

  [[nodiscard]] snappy::drivers::Result
  enable_commands();

  [[nodiscard]] bool wait_for_safety_clear(
    std::chrono::milliseconds timeout);

  [[nodiscard]] ActuatorSnapshot
  snapshot() const;

  [[nodiscard]] bool ready_for_arm()
      const noexcept;

  [[nodiscard]] bool started()
      const noexcept;

 private:
  using Clock = std::chrono::steady_clock;

  /*
   * 检查命令门和 generation。
   * 成功后 active_commands_ 加一，
   * safe_stop 会等待这些命令完成。
   */
  bool begin_command(
      std::uint64_t generation);

  void end_command();

  void brush_loop() noexcept;
  void push_loop() noexcept;
  void climber_loop() noexcept;
  void maintenance_loop() noexcept;
  void health_loop() noexcept;
  void publish_snapshot();

  void execute(
      CommandSlot slot,
      const PendingCommand& command) noexcept;

  void execute_safe_stop() noexcept;

  void update_result(
      const snappy::drivers::Result& result);

  void wait_for_active_commands();

  snappy::drivers::Result request_fault_clear();

  static std::uint64_t now_ms();

  RuntimeConfig config_;

  std::unique_ptr<DirectRs485Bus>
      rs485_bus_;

  std::unique_ptr<
      snappy::drivers::bus::SocketCanBus>
      can_bus_;

  std::unique_ptr<
      snappy::drivers::devices::RelayController>
      relay_;

  std::unique_ptr<
      snappy::drivers::devices::PushMotor>
      brush_push_;

  std::unique_ptr<
      snappy::drivers::devices::PushMotor>
      scraper_push_;

  std::unique_ptr<
      snappy::drivers::devices::BrushMotor>
      brush_;

  std::unique_ptr<
      snappy::drivers::devices::Climber>
      climber_;

  std::unique_ptr<
      snappy::drivers::devices::PropulsionFanController>
      fan_;

  std::unique_ptr<
      snappy::control::ActuatorService>
      actuator_service_;

  snappy::drivers::devices::RelayControllerConfig
    relay_config_{};

  snappy::drivers::devices::BrushMotorConfig
    brush_config_{};

  snappy::drivers::devices::ClimberConfig
    climber_config_{};

  /*
   * 每个 slot 最多保存一条最新命令。
   * 新命令覆盖旧命令，不积累 FIFO。
   */
  CommandMailbox mailbox_;

  std::atomic_bool started_{false};
  std::atomic_bool running_{false};
  std::atomic_bool ready_for_arm_{false};

  std::thread brush_thread_;
  std::thread push_thread_;
  std::thread climber_thread_;
  std::thread maintenance_thread_;
  std::thread health_thread_;

  mutable std::mutex gate_mutex_;
  std::condition_variable active_cv_;

  std::condition_variable safety_cv_;

  /*
   * generation 变化后，旧命令即使从邮箱取出也不能执行。
   */
  std::uint64_t generation_{1};

  std::size_t active_commands_{0};

  bool accepting_commands_{false};
  bool safe_stop_pending_{false};
  bool fault_clear_pending_{false};

  /*
   * snapshot_ 是总线和设备状态的内存发布副本。
   * 设备查询路径不能在此锁内执行物理 I/O。
   */
  mutable std::mutex snapshot_mutex_;
  ActuatorSnapshot snapshot_;
};

}  // namespace snappy_snappy