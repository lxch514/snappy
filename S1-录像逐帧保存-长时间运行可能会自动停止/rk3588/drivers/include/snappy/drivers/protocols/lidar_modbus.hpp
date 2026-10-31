#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

enum class LidarModel {
  kUbs1000C18r,
  kKs114,
};

struct LidarModbusProfile {
  LidarModel model{LidarModel::kUbs1000C18r};

  std::uint16_t ubs_start_register{0x0000};
  std::uint16_t ubs_register_count{3};
  double ubs_distance_mm_per_count{0.1};

  std::uint16_t ks114_command{0x0001};
  bool ks114_command_returns_flight_time_us{false};

  std::optional<std::uint16_t> invalid_raw{};
};

struct LidarDistanceRead {
  Result result;
  std::optional<double> distance_mm;
  std::optional<double> temperature_celsius;
  std::optional<std::uint16_t> flight_time_us;
  std::uint16_t raw_distance{0};
};

[[nodiscard]] std::vector<std::uint8_t> encode_lidar_read_distance(
    std::uint8_t station_id,
    const LidarModbusProfile& profile);

[[nodiscard]] LidarDistanceRead decode_lidar_distance_response(
    std::uint8_t station_id,
    const LidarModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols