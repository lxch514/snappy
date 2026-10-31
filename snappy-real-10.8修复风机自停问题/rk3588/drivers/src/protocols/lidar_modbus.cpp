#include "snappy/drivers/protocols/lidar_modbus.hpp"
/**
 * @file lidar_modbus.cpp
 * @brief 激光测距 Modbus 报文编解码实现
 *
 * @details
 * - 按型号选用寄存器映射，并校验响应长度。
 */

namespace snappy::drivers::protocols {
namespace {

double ks114_raw_to_mm(std::uint16_t raw, const LidarModbusProfile& profile) {
  if (profile.ks114_command_returns_flight_time_us) {
    return static_cast<double>(raw) * 10.0 / 58.0;
  }
  return static_cast<double>(raw);
}

Result lidar_protocol_error(const char* detail) {
  return Result::failure(ErrorCode::kIoError, detail, "decode", "lidar_modbus");
}

}  // namespace

std::vector<std::uint8_t> encode_lidar_read_distance(
    std::uint8_t station_id,
    const LidarModbusProfile& profile) {
  if (profile.model == LidarModel::kKs114) {
    return encode_read_registers(
        {station_id, kModbusReadHoldingRegisters, 0x0002, profile.ks114_command});
  }

  return encode_read_registers({station_id,
                                kModbusReadInputRegisters,
                                profile.ubs_start_register,
                                profile.ubs_register_count});
}

LidarDistanceRead decode_lidar_distance_response(
    std::uint8_t station_id,
    const LidarModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  const auto function_code = profile.model == LidarModel::kKs114
                                 ? kModbusReadHoldingRegisters
                                 : kModbusReadInputRegisters;
  const auto expected_count =
      profile.model == LidarModel::kKs114 ? 1 : profile.ubs_register_count;

  const auto decoded = decode_read_registers_response(
      station_id, function_code, expected_count, frame);
  if (!decoded.result.ok) {
    return {decoded.result, std::nullopt, std::nullopt, std::nullopt, 0};
  }
  if (decoded.values.empty()) {
    return {lidar_protocol_error("lidar response has no distance register"),
            std::nullopt,
            std::nullopt,
            std::nullopt,
            0};
  }

  const auto raw = decoded.values[0];
  if (profile.invalid_raw && raw == *profile.invalid_raw) {
    return {Result::failure(ErrorCode::kSensorStale,
                            "lidar distance is marked invalid",
                            "decode",
                            "lidar_modbus",
                            true),
            std::nullopt,
            std::nullopt,
            std::nullopt,
            raw};
  }

  if (profile.model == LidarModel::kKs114) {
    return {Result::success(),
            ks114_raw_to_mm(raw, profile),
            std::nullopt,
            profile.ks114_command_returns_flight_time_us
                ? std::optional<std::uint16_t>(raw)
                : std::nullopt,
            raw};
  }

  std::optional<double> temperature_celsius;
  std::optional<std::uint16_t> flight_time_us;
  if (decoded.values.size() >= 2) {
    temperature_celsius =
        static_cast<double>(static_cast<std::int16_t>(decoded.values[1]));
  }
  if (decoded.values.size() >= 3) {
    flight_time_us = decoded.values[2];
  }

  return {Result::success(),
          static_cast<double>(raw) * profile.ubs_distance_mm_per_count,
          temperature_celsius,
          flight_time_us,
          raw};
}

}  // namespace snappy::drivers::protocols