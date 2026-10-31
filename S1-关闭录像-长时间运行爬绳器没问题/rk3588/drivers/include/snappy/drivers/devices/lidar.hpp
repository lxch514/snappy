#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/lidar_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kLidarBrushLeftDeviceId = "lidar_brush_left";
inline constexpr const char* kLidarBrushRightDeviceId = "lidar_brush_right";
inline constexpr const char* kLidarBodyFrontDeviceId = "lidar_body_front";
inline constexpr const char* kLidarBodyRearDeviceId = "lidar_body_rear";

inline constexpr std::uint8_t kLidarBrushLeftStationId = 0x23;
inline constexpr std::uint8_t kLidarBrushRightStationId = 0x24;
inline constexpr std::uint8_t kLidarBodyFrontStationId = 0xD0;
inline constexpr std::uint8_t kLidarBodyRearStationId = 0xD2;

struct DistanceSensorConfig {
  std::string device_id;
  std::uint8_t station_id{0};
  protocols::LidarModbusProfile profile{};
};

struct DistanceSample {
  Result result;
  std::optional<double> distance_mm;
  std::optional<double> temperature_celsius;
  std::optional<std::uint16_t> flight_time_us;
  std::uint16_t raw_distance{0};
};

[[nodiscard]] DistanceSensorConfig make_lidar_brush_left_config();
[[nodiscard]] DistanceSensorConfig make_lidar_brush_right_config();
[[nodiscard]] DistanceSensorConfig make_lidar_body_front_config();
[[nodiscard]] DistanceSensorConfig make_lidar_body_rear_config();

class DistanceSensor {
 public:
  DistanceSensor(IRs485Bus& bus, DistanceSensorConfig config);

  Result start();
  void stop() noexcept;
  DistanceSample read_distance();

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  IRs485Bus& bus_;
  DistanceSensorConfig config_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

using Lidar = DistanceSensor;

}  // namespace snappy::drivers::devices