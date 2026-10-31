#pragma once
/**
 * @file brush_motor_modbus.hpp
 * @brief 毛刷电机 Modbus 报文编解码声明
 *
 * @details
 * - 定义方向枚举、故障码枚举和寄存器映射。
 * - 提供转速读写与故障码解析入口。
 */

#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

enum class BrushMotorDirection {
  kClockwise,
  kCounterClockwise,
};

enum class BrushMotorFaultCode : std::uint16_t {
  kNone = 0,
  kHallFault = 1,
  kOverTemperature = 2,
  kOverCurrent = 3,
  kStall = 4,
  kUnderVoltage = 6,
  kOverVoltage = 7,
  kUnknown = 0xFFFF,
};

struct BrushMotorModbusProfile {
  std::uint16_t run_register{100};
  std::uint16_t direction_register{101};
  std::uint16_t speed_register{102};
  std::uint16_t acceleration_register{103};
  std::uint16_t deceleration_register{104};
  std::uint16_t run_time_register{106};
  std::uint16_t lock_after_stop_register{107};
  std::uint16_t direction_switch_delay_register{108};
  std::uint16_t max_input_power_register{109};
  std::uint16_t control_register{110};
  std::uint16_t current_fault_register{111};
  std::uint16_t history_fault_start_register{112};
  std::uint16_t history_fault_count{4};

  std::uint16_t clockwise_value{0};
  std::uint16_t counter_clockwise_value{1};
  std::uint16_t run_value{1};
  std::uint16_t stop_value{0};
};

struct BrushMotorFaultRead {
  Result result;
  std::optional<std::uint16_t> raw_fault;
  std::optional<BrushMotorFaultCode> fault_code;
};

[[nodiscard]] BrushMotorFaultCode brush_motor_fault_code_from_raw(
    std::uint16_t raw) noexcept;

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_enable(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    bool enabled);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_direction(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    BrushMotorDirection direction);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_speed(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t speed_raw);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_acceleration(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t acceleration_raw);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_deceleration(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t deceleration_raw);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_run_time(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t run_time_20ms_units);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_lock_after_stop(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    bool locked);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_direction_switch_delay(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t delay_20ms_units);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_max_input_power(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t power_w);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_clear_history_fault(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_restore_defaults(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile);

[[nodiscard]] std::vector<std::uint8_t> encode_brush_motor_read_fault(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile);

[[nodiscard]] Result decode_brush_motor_write_response(
    std::uint8_t station_id,
    std::uint16_t expected_register,
    std::uint16_t expected_value,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] BrushMotorFaultRead decode_brush_motor_fault_response(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols