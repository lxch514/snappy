#include "snappy/drivers/devices/power.hpp"

#include <utility>

namespace snappy::drivers::devices {

PowerMonitorConfig make_power_24v_config() {
  return {kPower24vDeviceId, kPower24vStationId, {}};
}

PowerMonitorConfig make_power_48v_config() {
  return {kPower48vDeviceId, kPower48vStationId, {}};
}

PowerMonitor::PowerMonitor(IRs485Bus& bus, PowerMonitorConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result PowerMonitor::start() {
  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "power monitor config is incomplete",
                           "start",
                           config_.device_id);
  }
  health_ = DriverHealth::kReady;
  return Result::success("power monitor ready");
}

DriverHealth PowerMonitor::health() const noexcept {
  return health_;
}

PowerSample PowerMonitor::read_power() {
  if (health_ != DriverHealth::kReady) {
    return {Result::failure(ErrorCode::kConfigRequired,
                            "power monitor is not ready",
                            "read_power",
                            config_.device_id),
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt};
  }

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload = protocols::encode_power_read(config_.station_id, config_.profile);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return {response.result,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            std::nullopt};
  }

  const auto decoded = protocols::decode_power_response(
      config_.station_id, config_.profile, response.payload);
  return {decoded.result,
          decoded.voltage_v,
          decoded.current_a,
          decoded.power_w,
          decoded.temperature_celsius,
          decoded.output_mode,
          decoded.operation_enabled};
}

Result PowerMonitor::set_operation(bool enabled) {
  if (health_ != DriverHealth::kReady) {
    return Result::failure(ErrorCode::kConfigRequired,
                           "power monitor is not ready",
                           "set_operation",
                           config_.device_id);
  }

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_power_set_operation(config_.station_id, config_.profile, enabled);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return response.result;
  }
  return protocols::decode_power_set_operation_response(
      config_.station_id, config_.profile, enabled, response.payload);
}

Result PowerMonitor::turn_on() {
  return set_operation(true);
}

Result PowerMonitor::turn_off() {
  return set_operation(false);
}

void PowerMonitor::safe_stop() noexcept {
  static_cast<void>(turn_off());
}

void PowerMonitor::stop() noexcept {
  health_ = DriverHealth::kStopped;
}

}  // namespace snappy::drivers::devices