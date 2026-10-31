#include "snappy/drivers/protocols/climber_modbus.hpp"

#include <cstdint>
#include <vector>

namespace snappy::drivers::protocols {

std::vector<std::uint8_t> encode_climber_read_registers(
    std::uint8_t station_id,
    std::uint16_t start_register,
    std::uint16_t register_count) {
  return encode_read_registers(
      {station_id,
       kModbusReadHoldingRegisters,
       start_register,
       register_count});
}

std::vector<std::uint8_t> encode_climber_write_register(
    std::uint8_t station_id,
    std::uint16_t register_address,
    std::uint16_t value) {
  return encode_write_single_register(
      {station_id,
       register_address,
       value});
}

std::vector<std::uint8_t> encode_climber_write_target_speed(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile,
    std::int32_t speed_rpm) {
  return encode_climber_write_register(
      station_id,
      profile.target_speed_register,
      encode_climber_signed_i16(speed_rpm));
}

std::vector<std::uint8_t> encode_climber_write_virtual_io(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile,
    std::uint16_t virtual_mask) {
  return encode_climber_write_register(
      station_id,
      profile.virtual_io_register,
      virtual_mask);
}

std::vector<std::uint8_t> encode_climber_reset_current_alarm(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile) {
  return encode_climber_write_register(
      station_id,
      profile.auxiliary_function_register,
      profile.reset_current_alarm_value);
}

std::vector<std::uint8_t> encode_climber_reset_history_alarm(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile) {
  return encode_climber_write_register(
      station_id,
      profile.auxiliary_function_register,
      profile.reset_history_alarm_value);
}

ModbusRegisterRead decode_climber_read_registers_response(
    std::uint8_t station_id,
    std::uint16_t expected_register_count,
    const std::vector<std::uint8_t>& frame) {
  return decode_read_registers_response(
      station_id,
      kModbusReadHoldingRegisters,
      expected_register_count,
      frame);
}

Result decode_climber_write_response(
    std::uint8_t station_id,
    std::uint16_t expected_register,
    std::uint16_t expected_value,
    const std::vector<std::uint8_t>& frame) {
  return decode_write_single_response(
             station_id,
             kModbusWriteSingleRegister,
             expected_register,
             expected_value,
             frame)
      .result;
}

std::int32_t join_climber_i32_high_first(
    std::uint16_t high,
    std::uint16_t low) noexcept {
  const auto raw =
      (static_cast<std::uint32_t>(high) << 16U) | static_cast<std::uint32_t>(low);

  return static_cast<std::int32_t>(raw);
}

std::uint16_t encode_climber_signed_i16(
    std::int32_t value) noexcept {
  return static_cast<std::uint16_t>(static_cast<std::int16_t>(value));
}

}  // namespace snappy::drivers::protocols