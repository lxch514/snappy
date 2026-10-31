#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

struct WitMotionModbusProfile {
  std::uint16_t motion_start_register{0x0034};
  std::uint16_t motion_register_count{13};

  std::uint16_t accel_x_index{0};
  std::uint16_t accel_y_index{1};
  std::uint16_t accel_z_index{2};
  std::uint16_t gyro_x_index{3};
  std::uint16_t gyro_y_index{4};
  std::uint16_t gyro_z_index{5};
  std::uint16_t roll_index{9};
  std::uint16_t pitch_index{10};
  std::uint16_t yaw_index{11};
  std::uint16_t temperature_index{12};

  double accel_g_per_count{16.0 / 32768.0};
  double gyro_dps_per_count{2000.0 / 32768.0};
  double angle_deg_per_count{180.0 / 32768.0};
  double temperature_celsius_per_count{0.01};

  std::uint16_t gps_position_start_register{0x0049};
  std::uint16_t gps_position_register_count{5};

  std::uint16_t gps_quality_start_register{0x0055};
  std::uint16_t gps_quality_register_count{4};

  double gps_altitude_m_per_count{0.1};
};

struct WitMotionSample {
  Result result;

  std::optional<double> accel_x_g;
  std::optional<double> accel_y_g;
  std::optional<double> accel_z_g;

  std::optional<double> gyro_x_dps;
  std::optional<double> gyro_y_dps;
  std::optional<double> gyro_z_dps;

  std::optional<double> roll_deg;
  std::optional<double> pitch_deg;
  std::optional<double> yaw_deg;

  std::optional<double> temperature_celsius;

  std::optional<double> longitude_deg;
  std::optional<double> latitude_deg;
  std::optional<double> gps_altitude_m;
  std::optional<double> gps_satellites;
};

struct WitMotionGpsSample {
  Result result;

  std::optional<double> longitude_deg;
  std::optional<double> latitude_deg;
  std::optional<double> gps_altitude_m;
  std::optional<double> gps_satellites;
};


[[nodiscard]] std::vector<std::uint8_t> encode_wit_motion_read_motion(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile);

[[nodiscard]] WitMotionSample decode_wit_motion_motion_response(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] std::vector<std::uint8_t> encode_wit_motion_read_attitude(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile);

[[nodiscard]] WitMotionSample decode_wit_motion_attitude_response(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] std::vector<std::uint8_t>
encode_wit_motion_read_gps_position(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile);

[[nodiscard]] WitMotionGpsSample
decode_wit_motion_gps_position_response(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] std::vector<std::uint8_t>
encode_wit_motion_read_gps_quality(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile);

[[nodiscard]] WitMotionGpsSample
decode_wit_motion_gps_quality_response(
    std::uint8_t station_id,
    const WitMotionModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols