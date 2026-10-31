#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/brush_motor_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kBrushMotorDeviceId = "brush_motor";
inline constexpr std::uint8_t kBrushMotorStationId = 0x03;

struct BrushMotorConfig {
  std::string device_id{kBrushMotorDeviceId};
  std::uint8_t station_id{kBrushMotorStationId};
  std::uint16_t min_speed_raw{0};
  std::uint16_t max_speed_raw{15};
  std::chrono::milliseconds direction_dead_time{200};
  protocols::BrushMotorModbusProfile profile{};
};

struct BrushMotorFault {
  Result result;
  std::optional<std::uint16_t> raw_fault;
  std::optional<protocols::BrushMotorFaultCode> fault_code;
};

/*
 * 毛刷运行寄存器（run_register）的采样结果。
 *
 * 只读的观测结构，用于在运行过程中持续确认毛刷是否仍处于转动状态，
 * 不参与任何控制判断，也不改变驱动既有行为。
 */
struct BrushMotorRunState {
  Result result;
  std::uint16_t run_register_value{0};
  bool running{false};

  /*
   * 当前故障寄存器（current_fault_register）的采样。
   *
   * 只在运行寄存器读取成功时才采样，避免链路断开时两次超时叠加阻塞。
   * fault_available == false 表示这一次没拿到故障信息（不代表无故障）。
   */
  bool fault_available{false};
  std::optional<std::uint16_t> fault_raw;
  std::optional<protocols::BrushMotorFaultCode> fault_code;
};

[[nodiscard]] BrushMotorConfig make_brush_motor_config();

class BrushMotor {
 public:
  BrushMotor(IRs485Bus& bus, BrushMotorConfig config);

  Result start();
  Result stop_motion();
  Result set_speed_raw(std::uint16_t speed_raw);
  Result set_direction(protocols::BrushMotorDirection direction);
  Result run(protocols::BrushMotorDirection direction, std::uint16_t speed_raw);
  Result configure_motion(std::uint16_t acceleration_raw,
                          std::uint16_t deceleration_raw,
                          std::uint16_t run_time_20ms_units,
                          std::uint16_t direction_delay_20ms_units,
                          std::uint16_t max_input_power_w);
  Result set_lock_after_stop(bool locked);
  Result clear_history_fault();
  BrushMotorFault read_fault();
  BrushMotorRunState read_run_state();
  void safe_stop() noexcept;
  void stop() noexcept;

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  Result write_and_expect(const std::vector<std::uint8_t>& request_payload,
                          std::uint16_t expected_register,
                          std::uint16_t expected_value);

  IRs485Bus& bus_;
  BrushMotorConfig config_;
  std::optional<protocols::BrushMotorDirection> current_direction_;
  bool running_{false};
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices