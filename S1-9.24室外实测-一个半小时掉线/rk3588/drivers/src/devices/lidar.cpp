#include "snappy/drivers/devices/lidar.hpp"

#include <utility>

namespace snappy::drivers::devices {
namespace {

protocols::LidarModbusProfile ubs_profile() {
  protocols::LidarModbusProfile profile;
  profile.model = protocols::LidarModel::kUbs1000C18r;
  return profile;
}

protocols::LidarModbusProfile ks114_profile() {
  protocols::LidarModbusProfile profile;
  profile.model = protocols::LidarModel::kKs114;
  profile.ks114_command = 0x0001;
  profile.ks114_command_returns_flight_time_us = false;
  return profile;
}

}  // namespace

DistanceSensorConfig make_lidar_brush_left_config() {
  return {kLidarBrushLeftDeviceId, kLidarBrushLeftStationId, ubs_profile()};
}

DistanceSensorConfig make_lidar_brush_right_config() {
  return {kLidarBrushRightDeviceId, kLidarBrushRightStationId, ubs_profile()};
}

DistanceSensorConfig make_lidar_body_front_config() {
  return {kLidarBodyFrontDeviceId, kLidarBodyFrontStationId, ks114_profile()};
}

DistanceSensorConfig make_lidar_body_rear_config() {
  return {kLidarBodyRearDeviceId, kLidarBodyRearStationId, ks114_profile()};
}

DistanceSensor::DistanceSensor(IRs485Bus& bus, DistanceSensorConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result DistanceSensor::start() {
  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "distance sensor config is incomplete",
                           "start",
                           config_.device_id);
  }
  health_ = DriverHealth::kReady;
  return Result::success("distance sensor ready");
}

void DistanceSensor::stop() noexcept {
  health_ = DriverHealth::kStopped;
}

DriverHealth DistanceSensor::health() const noexcept {
  return health_;
}

DistanceSample DistanceSensor::read_distance() {
  if (health_ != DriverHealth::kReady) {
    return {Result::failure(ErrorCode::kConfigRequired,
                            "distance sensor is not ready",
                            "read_distance",
                            config_.device_id),
            std::nullopt,
            std::nullopt,
            std::nullopt,
            0};
  }

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_lidar_read_distance(config_.station_id, config_.profile);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return {response.result, std::nullopt, std::nullopt, std::nullopt, 0};
  }

  const auto decoded = protocols::decode_lidar_distance_response(
      config_.station_id, config_.profile, response.payload);
  return {decoded.result,
          decoded.distance_mm,
          decoded.temperature_celsius,
          decoded.flight_time_us,
          decoded.raw_distance};
}

}  // namespace snappy::drivers::devices