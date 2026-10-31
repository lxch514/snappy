#pragma once
/**
 * @file brush_motor.hpp
 * @brief 毛刷电机设备驱动声明
 *
 * @details
 * - 通过 RS485 Modbus 设置转速与方向，并读取故障码。
 * - 限速上限由配置给出，超出直接拒绝而不是截断。
 */

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/brush_motor_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kBrushMotorDeviceId = "brush_motor";
inline constexpr std::uint8_t kBrushMotorStationId = 0x64;

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