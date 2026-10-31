#include "snappy/drivers/protocols/anemometer_modbus.hpp"
/**
 * @file anemometer_modbus.cpp
 * @brief 风速仪 Modbus 报文编解码实现
 *
 * @details
 * - 构造读寄存器请求，并校验响应站号、功能码与字节数。
 */
 
namespace snappy::drivers::protocols {

std::vector<std::uint8_t> encode_anemometer_read_speed(
    std::uint8_t station_id,
    const AnemometerModbusProfile& profile) {
  return encode_read_registers({station_id,
                                kModbusReadHoldingRegisters,
                                profile.wind_speed_register,
                                profile.register_count});
}

WindSpeedRead decode_anemometer_speed_response(
    std::uint8_t station_id,
    const AnemometerModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  const auto decoded = decode_read_registers_response(
      station_id, kModbusReadHoldingRegisters, profile.register_count, frame);
  if (!decoded.result.ok) {
    return {decoded.result, std::nullopt, std::nullopt, 0};
  }
  if (decoded.values.empty()) {
    return {Result::failure(ErrorCode::kIoError,
                            "anemometer response has no speed register",
                            "decode",
                            "anemometer_modbus"),
            std::nullopt,
            std::nullopt,
            0};
  }

  const auto raw = decoded.values[0];
  const double mps = static_cast<double>(raw) * profile.mps_per_count;
  if (profile.max_valid_mps && mps > *profile.max_valid_mps) {
    return {Result::failure(ErrorCode::kSensorStale,
                            "anemometer speed is outside valid range",
                            "decode",
                            "anemometer_modbus",
                            true),
            std::nullopt,
            std::nullopt,
            raw};
  }

  return {Result::success(), mps, mps * 3.6, raw};
}

}  // namespace snappy::drivers::protocols