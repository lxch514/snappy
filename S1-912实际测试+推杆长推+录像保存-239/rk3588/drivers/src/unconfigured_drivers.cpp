#include "snappy/drivers/unconfigured_drivers.hpp"

namespace snappy::drivers {
namespace {

constexpr const char* kCanPending =
    "CAN backend unconfigured: bitrate, device IDs and frame definitions are pending";
constexpr const char* kRs485Pending =
    "RS485 backend unconfigured: ports, baud rates, station IDs and frames are pending";
constexpr const char* kRelayPending =
    "relay backend unconfigured: physical channel mapping and feedback are pending";
constexpr const char* kSensorPending =
    "sensor backend unconfigured: models, station IDs and register maps are pending";

}  // namespace

Result UnconfiguredCanBus::start() {
  return Result::failure(
      ErrorCode::kConfigRequired, kCanPending, "start", "can_bus");
}
Result UnconfiguredCanBus::send(const CanFrame&) {
  return Result::failure(
      ErrorCode::kConfigRequired, kCanPending, "send", "can_bus");
}
void UnconfiguredCanBus::safe_stop() noexcept {}
DriverHealth UnconfiguredCanBus::health() const noexcept {
  return DriverHealth::kUnconfigured;
}

Result UnconfiguredRs485Bus::start() {
  return Result::failure(
      ErrorCode::kConfigRequired, kRs485Pending, "start", "rs485_bus");
}
Rs485Response UnconfiguredRs485Bus::exchange(const Rs485Request&) {
  return {Result::failure(
              ErrorCode::kConfigRequired, kRs485Pending, "exchange", "rs485_bus"),
          {}};
}
void UnconfiguredRs485Bus::safe_stop() noexcept {}
DriverHealth UnconfiguredRs485Bus::health() const noexcept {
  return DriverHealth::kUnconfigured;
}

Result UnconfiguredRelayBank::start() {
  return Result::failure(
      ErrorCode::kConfigRequired, kRelayPending, "start", "relay_bank");
}
Result UnconfiguredRelayBank::set(const std::string&, RelayState) {
  return Result::failure(
      ErrorCode::kConfigRequired, kRelayPending, "set", "relay_bank");
}
void UnconfiguredRelayBank::deenergize_all() noexcept {}
DriverHealth UnconfiguredRelayBank::health() const noexcept {
  return DriverHealth::kUnconfigured;
}

Result UnconfiguredSensorSuite::start() {
  return Result::failure(
      ErrorCode::kConfigRequired, kSensorPending, "start", "sensor_suite");
}
std::vector<SensorSample> UnconfiguredSensorSuite::read_all() { return {}; }
void UnconfiguredSensorSuite::stop() noexcept {}
DriverHealth UnconfiguredSensorSuite::health() const noexcept {
  return DriverHealth::kUnconfigured;
}

}  // namespace snappy::drivers
