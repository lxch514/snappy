#include "snappy/drivers/protocols/wit_motion_modbus.hpp"
/**
 * @file wit_motion_modbus.cpp
 * @brief 维特智能 Modbus 报文编解码实现
 *
 * @details
 * - 分别解析姿态角与 GPS 定位两组寄存器。
 */

namespace snappy::drivers::protocols {
namespace {

std::optional<double> scaled_i16(const std::vector<std::uint16_t>& values,
                                 std::uint16_t index,
                                 double scale) {
  if (index >= values.size()) return std::nullopt;
  return static_cast<double>(static_cast<std::int16_t>(values[index])) * scale;
}

bool has_value(const std::optional<double>& value) {
  return value.has_value();
}

Result missing_motion_registers() {
  return Result::failure(ErrorCode::kIoError,
                         "wit motion response does not contain required registers",
                         "decode",
                         "wit_motion_modbus");
}

}  // namespace

std::vector<std::uint8_t> encode_wit_motion_read_motion(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile) {
  return encode_read_registers({station_id,
                                kModbusReadHoldingRegisters,
                                profile.motion_start_register,
                                profile.motion_register_count});
}

WitMotionSample decode_wit_motion_motion_response(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  const auto decoded = decode_read_registers_response(
      station_id, kModbusReadHoldingRegisters, profile.motion_register_count, frame);
  if (!decoded.result.ok) {
    return {decoded.result};
  }

  WitMotionSample sample;
  sample.result = Result::success();
  sample.accel_x_g = scaled_i16(decoded.values, profile.accel_x_index,
                                profile.accel_g_per_count);
  sample.accel_y_g = scaled_i16(decoded.values, profile.accel_y_index,
                                profile.accel_g_per_count);
  sample.accel_z_g = scaled_i16(decoded.values, profile.accel_z_index,
                                profile.accel_g_per_count);
  sample.gyro_x_dps = scaled_i16(decoded.values, profile.gyro_x_index,
                                 profile.gyro_dps_per_count);
  sample.gyro_y_dps = scaled_i16(decoded.values, profile.gyro_y_index,
                                 profile.gyro_dps_per_count);
  sample.gyro_z_dps = scaled_i16(decoded.values, profile.gyro_z_index,
                                 profile.gyro_dps_per_count);
  sample.roll_deg = scaled_i16(decoded.values, profile.roll_index,
                               profile.angle_deg_per_count);
  sample.pitch_deg = scaled_i16(decoded.values, profile.pitch_index,
                                profile.angle_deg_per_count);
  sample.yaw_deg = scaled_i16(decoded.values, profile.yaw_index,
                              profile.angle_deg_per_count);
  sample.temperature_celsius = scaled_i16(decoded.values, profile.temperature_index,
                                          profile.temperature_celsius_per_count);

  if (!has_value(sample.roll_deg) || !has_value(sample.pitch_deg) ||
      !has_value(sample.yaw_deg)) {
    sample.result = missing_motion_registers();
  }
  return sample;
}

std::vector<std::uint8_t> encode_wit_motion_read_attitude(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile) {
  return encode_wit_motion_read_motion(station_id, profile);
}

WitMotionSample decode_wit_motion_attitude_response(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  return decode_wit_motion_motion_response(station_id, profile, frame);
}

}  // namespace snappy::drivers::protocols