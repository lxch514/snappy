#include "snappy/drivers/protocols/brush_motor_modbus.hpp"
/**
 * @file brush_motor_modbus.cpp
 * @brief 毛刷电机 Modbus 报文编解码实现
 *
 * @details
 * - 构造转速与方向写请求，并解析故障码寄存器。
 */
 
#include <string>
#include <utility>

namespace snappy::drivers::protocols {
namespace {

Result brush_protocol_failure(ErrorCode code, std::string detail) {
  return Result::failure(code,
                         std::move(detail),
                         "decode",
                         "brush_motor_modbus");
}

std::vector<std::uint8_t> write_register(std::uint8_t station_id,
                                         std::uint16_t address,
                                         std::uint16_t value) {
  return encode_write_single_register({station_id, address, value});
}

}  // namespace

BrushMotorFaultCode brush_motor_fault_code_from_raw(std::uint16_t raw) noexcept {
  switch (raw) {
    case 0:
      return BrushMotorFaultCode::kNone;
    case 1:
      return BrushMotorFaultCode::kHallFault;
    case 2:
      return BrushMotorFaultCode::kOverTemperature;
    case 3:
      return BrushMotorFaultCode::kOverCurrent;
    case 4:
      return BrushMotorFaultCode::kStall;
    case 6:
      return BrushMotorFaultCode::kUnderVoltage;
    case 7:
      return BrushMotorFaultCode::kOverVoltage;
    default:
      return BrushMotorFaultCode::kUnknown;
  }
}

std::vector<std::uint8_t> encode_brush_motor_enable(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    bool enabled) {
  return write_register(station_id,
                        profile.run_register,
                        enabled ? profile.run_value : profile.stop_value);
}

std::vector<std::uint8_t> encode_brush_motor_direction(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    BrushMotorDirection direction) {
  const auto value = direction == BrushMotorDirection::kClockwise
                         ? profile.clockwise_value
                         : profile.counter_clockwise_value;
  return write_register(station_id, profile.direction_register, value);
}

std::vector<std::uint8_t> encode_brush_motor_speed(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t speed_raw) {
  return write_register(station_id, profile.speed_register, speed_raw);
}

std::vector<std::uint8_t> encode_brush_motor_acceleration(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t acceleration_raw) {
  return write_register(station_id,
                        profile.acceleration_register,
                        acceleration_raw);
}

std::vector<std::uint8_t> encode_brush_motor_deceleration(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t deceleration_raw) {
  return write_register(station_id,
                        profile.deceleration_register,
                        deceleration_raw);
}

std::vector<std::uint8_t> encode_brush_motor_run_time(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t run_time_20ms_units) {
  return write_register(station_id, profile.run_time_register, run_time_20ms_units);
}

std::vector<std::uint8_t> encode_brush_motor_lock_after_stop(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    bool locked) {
  return write_register(station_id,
                        profile.lock_after_stop_register,
                        locked ? 1U : 0U);
}

std::vector<std::uint8_t> encode_brush_motor_direction_switch_delay(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t delay_20ms_units) {
  return write_register(station_id,
                        profile.direction_switch_delay_register,
                        delay_20ms_units);
}

std::vector<std::uint8_t> encode_brush_motor_max_input_power(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile,
    std::uint16_t power_w) {
  return write_register(station_id, profile.max_input_power_register, power_w);
}

std::vector<std::uint8_t> encode_brush_motor_clear_history_fault(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile) {
  return write_register(station_id, profile.control_register, 0x0002);
}

std::vector<std::uint8_t> encode_brush_motor_restore_defaults(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile) {
  return write_register(station_id, profile.control_register, 0x0001);
}

std::vector<std::uint8_t> encode_brush_motor_read_fault(
    std::uint8_t station_id,
    const BrushMotorModbusProfile& profile) {
  return encode_read_registers(
      {station_id, kModbusReadHoldingRegisters, profile.current_fault_register, 1});
}

Result decode_brush_motor_write_response(
    std::uint8_t station_id,
    std::uint16_t expected_register,
    std::uint16_t expected_value,
    const std::vector<std::uint8_t>& frame) {
  bool saw_exception = false;
  bool saw_valid_write = false;
  std::uint16_t first_valid_register = 0;
  std::uint16_t first_valid_value = 0;

  for (std::size_t offset = 0; offset + 5U <= frame.size(); ++offset) {
    if (frame[offset] != station_id) {
      continue;
    }

    const auto function_code = frame[offset + 1U];
    const auto exception_code =
        static_cast<std::uint8_t>(kModbusWriteSingleRegister | 0x80U);

    if (function_code == exception_code) {
      if (offset + 5U > frame.size()) {
        continue;
      }
      const std::vector<std::uint8_t> candidate(
          frame.begin() + static_cast<std::ptrdiff_t>(offset),
          frame.begin() + static_cast<std::ptrdiff_t>(offset + 5U));
      if (modbus_crc_valid(candidate)) {
        saw_exception = true;
      }
      continue;
    }

    if (function_code != kModbusWriteSingleRegister) {
      continue;
    }

    if (offset + 8U > frame.size()) {
      continue;
    }

    const std::vector<std::uint8_t> candidate(
        frame.begin() + static_cast<std::ptrdiff_t>(offset),
        frame.begin() + static_cast<std::ptrdiff_t>(offset + 8U));
    if (!modbus_crc_valid(candidate)) {
      continue;
    }

    const auto address = read_u16_be(candidate, 2U);
    const auto value = read_u16_be(candidate, 4U);
    saw_valid_write = true;
    first_valid_register = address;
    first_valid_value = value;

    if (address == expected_register && value == expected_value) {
      return Result::success("BD-201S write acknowledged");
    }
  }

  if (saw_exception) {
    return brush_protocol_failure(ErrorCode::kRs485ExceptionResponse,
                                  "BD-201S Modbus exception response received");
  }

  if (saw_valid_write) {
    return brush_protocol_failure(
        ErrorCode::kIoError,
        "BD-201S write echo does not match request: register=" +
            std::to_string(first_valid_register) + " value=" +
            std::to_string(first_valid_value) + " expected_register=" +
            std::to_string(expected_register) + " expected_value=" +
            std::to_string(expected_value));
  }

  return brush_protocol_failure(ErrorCode::kRs485Timeout,
                                "BD-201S write response not found");
}

BrushMotorFaultRead decode_brush_motor_fault_response(
    std::uint8_t station_id,
    const BrushMotorModbusProfile&,
    const std::vector<std::uint8_t>& frame) {
  const auto window =
      find_modbus_frame(station_id, kModbusReadHoldingRegisters, frame);
  if (!window.result.ok) {
    return {window.result, std::nullopt, std::nullopt};
  }

  const std::vector<std::uint8_t> candidate(
      frame.begin() + static_cast<std::ptrdiff_t>(window.offset),
      frame.begin() + static_cast<std::ptrdiff_t>(window.offset + window.size));

  const auto decoded = decode_read_registers_response(
      station_id, kModbusReadHoldingRegisters, 1, candidate);
  if (!decoded.result.ok) {
    return {decoded.result, std::nullopt, std::nullopt};
  }
  if (decoded.values.empty()) {
    return {brush_protocol_failure(ErrorCode::kIoError,
                                   "brush motor fault response is empty"),
            std::nullopt,
            std::nullopt};
  }

  const auto raw = decoded.values[0];
  return {Result::success(), raw, brush_motor_fault_code_from_raw(raw)};
}

}  // namespace snappy::drivers::protocols