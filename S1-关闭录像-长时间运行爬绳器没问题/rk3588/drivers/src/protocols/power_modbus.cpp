#include "snappy/drivers/protocols/power_modbus.hpp"

namespace snappy::drivers::protocols {
namespace {

std::optional<double> scaled_at(const std::vector<std::uint16_t>& values,
                                std::uint16_t index,
                                double scale) {
  if (index >= values.size()) {
    return std::nullopt;
  }
  return static_cast<double>(values[index]) * scale;
}

PowerOutputMode output_mode_from_raw(std::uint16_t raw) {
  switch (raw) {
    case 0:
      return PowerOutputMode::kOff;
    case 1:
      return PowerOutputMode::kConstantCurrent;
    case 2:
      return PowerOutputMode::kConstantVoltage;
    default:
      return PowerOutputMode::kUnknown;
  }
}

}  // namespace

std::vector<std::uint8_t> encode_power_read(
    std::uint8_t station_id,
    const PowerModbusProfile& profile) {
  return encode_read_registers({station_id,
                                kModbusReadInputRegisters,
                                profile.input_start_register,
                                profile.input_register_count});
}

PowerRead decode_power_response(std::uint8_t station_id,
                                const PowerModbusProfile& profile,
                                const std::vector<std::uint8_t>& frame) {
  const auto decoded = decode_read_registers_response(
      station_id, kModbusReadInputRegisters, profile.input_register_count, frame);
  if (!decoded.result.ok) {
    return {decoded.result,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt};
  }

  if (profile.voltage_index >= decoded.values.size() ||
      profile.current_index >= decoded.values.size() ||
      profile.power_index >= decoded.values.size() ||
      profile.temperature_index >= decoded.values.size() ||
      profile.output_mode_index >= decoded.values.size() ||
      profile.operation_index >= decoded.values.size()) {
    return {Result::failure(ErrorCode::kIoError,
                            "power response register index out of range",
                            "decode",
                            "power_modbus"),
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt};
  }

  return {Result::success(),
          scaled_at(decoded.values, profile.voltage_index,
                    profile.voltage_v_per_count),
          scaled_at(decoded.values, profile.current_index,
                    profile.current_a_per_count),
          scaled_at(decoded.values, profile.power_index,
                    profile.power_w_per_count),
          scaled_at(decoded.values, profile.temperature_index,
                    profile.temperature_celsius_per_count),
          output_mode_from_raw(decoded.values[profile.output_mode_index]),
          decoded.values[profile.operation_index] != 0};
}

std::vector<std::uint8_t> encode_power_set_operation(
    std::uint8_t station_id,
    const PowerModbusProfile& profile,
    bool enabled) {
  return encode_write_single_register(
      {station_id,
       profile.operation_register,
       static_cast<std::uint16_t>(enabled ? 1U : 0U)});
}

Result decode_power_set_operation_response(std::uint8_t station_id,
                                           const PowerModbusProfile& profile,
                                           bool enabled,
                                           const std::vector<std::uint8_t>& frame) {
  return decode_write_single_response(station_id,
                                      kModbusWriteSingleRegister,
                                      profile.operation_register,
                                      enabled ? 1U : 0U,
                                      frame)
      .result;
}

}  // namespace snappy::drivers::protocols