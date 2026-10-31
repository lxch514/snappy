#include "snappy/drivers/devices/anemometer.hpp"

#include <utility>

namespace snappy::drivers::devices {

AnemometerConfig make_anemometer_left_config() {
  return {kAnemometerLeftDeviceId, kAnemometerLeftStationId, {}};
}

AnemometerConfig make_anemometer_right_config() {
  return {kAnemometerRightDeviceId, kAnemometerRightStationId, {}};
}

Anemometer::Anemometer(IRs485Bus& bus, AnemometerConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result Anemometer::start() {
  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "anemometer config is incomplete",
                           "start",
                           config_.device_id);
  }
  health_ = DriverHealth::kReady;
  return Result::success("anemometer ready");
}

void Anemometer::stop() noexcept {
  health_ = DriverHealth::kStopped;
}

DriverHealth Anemometer::health() const noexcept {
  return health_;
}

AnemometerSample Anemometer::read_speed() {
  if (health_ != DriverHealth::kReady) {
    return {Result::failure(ErrorCode::kConfigRequired,
                            "anemometer is not ready",
                            "read_speed",
                            config_.device_id),
            std::nullopt,
            std::nullopt};
  }

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_anemometer_read_speed(config_.station_id, config_.profile);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return {response.result, std::nullopt, std::nullopt};
  }

  const auto decoded = protocols::decode_anemometer_speed_response(
      config_.station_id, config_.profile, response.payload);
  return {decoded.result, decoded.speed_mps, decoded.speed_kmh};
}

}  // namespace snappy::drivers::devices