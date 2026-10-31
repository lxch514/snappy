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
#include "snappy/control/sensor_service.hpp"
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
      "/run/snappy-real/control.sock"};

  /* 只读 SensorService 缓存；ControlServer 不在请求路径执行传感器 I/O。 */
  snappy::control::SensorService* sensors{nullptr};
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

  /* 供 snappy-real 的 SensorRuntime 复用同一 RS485 调度器。
   * 这样 ttyACM0/1/2 仍共享 WCH 事务仲裁，且每个物理端口只有一个 worker。
   */
  [[nodiscard]] DirectRs485Bus& rs485_bus() noexcept {
    return *rs485_bus_;
  }

  [[nodiscard]] const DirectRs485Bus& rs485_bus() const noexcept {
    return *rs485_bus_;
  }

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

  /*
   * 安全停止没有确认成功时，按固定节拍重试。
   *
   * 单次 RS485 抖动不应该让整机停在“已安全停止”的假象上，
   * 更不应该让爬绳器保持上一次转速继续旋转。
   */
  void maybe_retry_safe_stop() noexcept;

  /*
   * 命令门关闭期间，如果爬绳器上一次动作还没有被确认停止，
   * 周期性重申一次爬绳器安全停止。
   *
   * 只在“确实可能还在转”时才会产生 RS485 流量，
   * 正常停机状态下不会有任何额外报文。
   */
  void maybe_reassert_climber_stop() noexcept;

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
   * 安全停止重试记账。
   *
   * safe_stop_attempts_：本轮安全停止已经尝试的次数。
   * next_safe_stop_attempt_：下一次允许重试的时间点。
   */
  std::uint32_t safe_stop_attempts_{0};
  Clock::time_point next_safe_stop_attempt_{};

  /*
   * 爬绳器“可能仍在转动”标志。
   *
   * 只要下发过一次非零爬绳器转速命令就置位；
   * 只有确认执行成功的安全停止 / climber.stop 才会清除。
   * 置位且命令门关闭时，维护线程会周期性重申停止。
   */
  std::atomic_bool climber_may_be_running_{false};

  /*
   * 爬绳器重申已经连续失败的次数。
   *
   * 连续失败时把重申周期从 2s 退避到 10s，
   * 既不会放弃对“可能还在转”的爬绳器持续下发停止，
   * 也不会在设备确实离线时刷屏日志、长期占用共享 RS485。
   */
  std::uint32_t climber_reassert_attempts_{0};

  Clock::time_point safety_reassert_deadline_{};

  /*
   * snapshot_ 是总线和设备状态的内存发布副本。
   * 设备查询路径不能在此锁内执行物理 I/O。
   */
  mutable std::mutex snapshot_mutex_;
  ActuatorSnapshot snapshot_;
};

}  // namespace snappy_snappy