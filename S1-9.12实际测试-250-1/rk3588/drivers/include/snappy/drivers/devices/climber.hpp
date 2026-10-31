#pragma once

/**
 * @file climber.hpp
 * @brief L7P 爬绳器驱动接口
 */

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/climber_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kClimberDeviceId = "climber";

inline constexpr std::uint8_t kClimberStationId = protocols::kClimberL7PStationId;

struct ClimberConfig {
  std::string device_id{kClimberDeviceId};

  std::uint8_t station_id{kClimberStationId};

  protocols::ClimberModbusProfile profile{};
  
  bool configure_on_start{false};

  bool auto_clear_alarm_on_start{true};

  /*
   * 抱闸释放等待：
   *
   * P04.38 + margin
   */
  std::chrono::milliseconds
      brake_release_margin{10};

  /*
   * 正常 STOP 的轮询周期。
   */
  std::chrono::milliseconds
      stop_poll_period{20};

  /*
   * 正常 STOP 最大等待时间。
   */
  std::chrono::milliseconds
      stop_timeout{1000};

  /*
   * 清报警后的稳定等待。
   */
  std::chrono::milliseconds
      alarm_reset_settle_time{50};
};

struct ClimberReadI32 {
  Result result;
  std::optional<std::int32_t> value;
};

struct ClimberReadDouble {
  Result result;
  std::optional<double> value;
};

[[nodiscard]] ClimberConfig make_climber_config();

class Climber {
 public:
  Climber(
      IRs485Bus& bus,
      ClimberConfig config);

  /*
   * 生命周期
   */
  Result start();

  /*
   * 正常、可验证 STOP。
   *
   * P03.04=0
   * -> 实际速度下降
   * -> SRV-OFF
   * -> RUNNING=0
   * -> BRK-OFF=0
   */
  Result disable();

  /*
   * 故障清除。
   */
  Result clear_current_alarm();
  Result clear_history_alarm();
  Result clear_fault();

  /*
   * 运动控制。
   *
   * speed > 0 : 正转
   * speed < 0 : 反转
   * speed = 0 : 完整 STOP
   */
  Result set_target_speed_raw(
      std::int32_t speed_rpm);

  Result set_target_speed_rad_per_s(
      double speed_rad_per_s);

  /*
   * 保持原有 API 名称。
   *
   * 当前项目原实现实际上也是按 rad/s
   * 处理，因此不改变 ABI。
   */
  Result set_target_speed_rps(
      double speed_rad_per_s);

  /*
   * 状态读取
   */
  ClimberReadI32 read_alarm();

  ClimberReadI32 read_actual_speed_raw();

  ClimberReadDouble
  read_actual_speed_rad_per_s();

  ClimberReadDouble
  read_actual_speed_rps();

  ClimberReadI32
  read_bus_voltage_raw();

  /*
   * 快速安全停止。
   *
   * 不等待速度反馈、不等待抱闸反馈。
   */
  [[nodiscard]] Result
  safe_stop() noexcept;

  void stop() noexcept;

  [[nodiscard]] DriverHealth
  health() const noexcept;

 private:
  enum class SrvOnState {
    kUnknown,
    kOff,
    kOn,
  };

  /*
   * Modbus 基础读写
   */
  protocols::ModbusRegisterRead
  read_registers(
      std::uint16_t start_register,
      std::uint16_t register_count);

  ClimberReadI32 read_register(
      std::uint16_t register_address);

  ClimberReadI32 read_s32(
      std::uint16_t high_register_address);

  Result write_register(
      std::uint16_t register_address,
      std::uint16_t value,
      const char* operation);

  Result write_if_changed(
      std::uint16_t register_address,
      std::uint16_t expected_value,
      const char* operation,
      bool& changed);

  Result write_target_speed_rpm(
      std::int32_t speed_rpm);

  /*
   * 驱动配置 / READY 检查
   */
  Result ensure_drive_configured();

  Result ensure_srv_on_assignment(
      bool& changed);

  Result inspect_drive_ready(
      bool clear_alarm_if_needed);

  /*
   * 运动使能前置流程。
   *
   * 检查状态、必要时完整 STOP，
   * SRV-ON、等待抱闸释放，
   * 最后确认 READY + BRK-OFF。
   */
  Result preflight_motion();

  /*
   * SRV-ON / SRV-OFF
   */
  Result set_srv_on(bool enabled);

  /*
   * 强制 SRV-OFF。
   *
   * 不允许 srv_on_state_ 缓存短路。
   */
  Result force_srv_off();

  /*
   * 等待抱闸释放。
   */
  Result wait_for_brake_release();

  /*
   * 等实际速度下降到停止阈值。
   */
  Result wait_for_speed_below(
      std::int32_t rpm_threshold);

  /*
   * 同时确认：
   *
   * RUNNING = 0
   * BRK-OFF = 0
   */
  Result wait_for_disabled_and_brake_closed();

  /*
   * 读取一次：
   *
   * P0B.05 RUNNING
   * P0B.18 DO3
   *
   * 两个寄存器一次 Modbus 读取完成，
   * 减少共享 /dev/ttyS0 的占用。
   */
  Result read_stop_state(
      bool& running,
      bool& brake_off);

  /*
   * 项目1式完整正常 STOP。
   *
   * 这是整个驱动的唯一正常停止链路。
   */
  Result stop_sequence();

  mutable std::recursive_mutex
      device_mutex_;

  IRs485Bus& bus_;

  ClimberConfig config_;

  DriverHealth health_{
      DriverHealth::kUnconfigured};

  bool configured_{false};

  /*
   * 这是软件最后一次 SRV-ON 写入状态，
   * 不是物理输入反馈。
   */
  SrvOnState srv_on_state_{
      SrvOnState::kUnknown};

  /*
   * 只有完整 preflight 成功后才为 true。
   */
  bool motion_ready_{false};

  /*
   * 最近一次成功写入的目标速度。
   *
   * 不是实际速度。
   */
  std::int32_t target_speed_rpm_{0};

  bool target_speed_known_{false};

  /*
   * SRV-ON 对应的 DI。
   */
  int srv_on_di_index_{-1};

  /*
   * SRV-ON DI 是否常闭。
   */
  bool srv_on_normally_closed_{false};
};

}  // namespace snappy::drivers::devices