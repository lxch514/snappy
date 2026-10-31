#include "snappy/drivers/protocols/temperature_modbus.hpp"
/**
 * @file temperature_modbus.cpp
 * @brief 温湿度 Modbus 报文编解码实现
 *
 * @details
 * - 解析温度与湿度寄存器，超出量程按无效处理。
 */

namespace snappy::drivers::protocols {
namespace {

double decode_temperature_raw(std::uint16_t raw, double scale) {
  if (raw >= 10000U) {
    return -static_cast<double>(raw - 10000U) * scale;
  }
  return static_cast<double>(raw) * scale;
}

}  // namespace

std::vector<std::uint8_t> encode_temperature_read(
    std::uint8_t station_id,
    const TemperatureModbusProfile& profile) {
  return encode_read_registers({station_id,
                                profile.read_function_code,
                                profile.start_register,
                                profile.register_count});
}

TemperatureHumidityRead decode_temperature_response(
    std::uint8_t station_id,
    const TemperatureModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  const auto decoded = decode_read_registers_response(
      station_id, profile.read_function_code, profile.register_count, frame);
  if (!decoded.result.ok) {
    return {decoded.result, std::nullopt, std::nullopt};
  }

  if (profile.temperature_index >= decoded.values.size() ||
      profile.humidity_index >= decoded.values.size()) {
    return {Result::failure(ErrorCode::kIoError,
                            "temperature response register index out of range",
                            "decode",
                            "temperature_modbus"),
            std::nullopt,
            std::nullopt};
  }

  const auto temperature_raw = decoded.values[profile.temperature_index];
  const auto humidity_raw = decoded.values[profile.humidity_index];

  return {Result::success(),
          decode_temperature_raw(temperature_raw,
                                 profile.temperature_celsius_per_count),
          static_cast<double>(humidity_raw) * profile.humidity_rh_per_count};
}

}  // namespace snappy::drivers::protocols