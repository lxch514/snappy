#include "snappy/simulator/fake_hardware.hpp"

#include <utility>

namespace snappy::simulator {

drivers::Result FakeCanBus::start() {
  health_ = drivers::DriverHealth::kReady;
  return drivers::Result::success("fake CAN ready");
}

drivers::Result FakeCanBus::send(const drivers::CanFrame& frame) {
  if (health_ != drivers::DriverHealth::kReady) {
    return drivers::Result::failure("fake CAN is not started");
  }
  if (send_failure_) {
    return *send_failure_;
  }
  sent_frames_.push_back(frame);
  return drivers::Result::success();
}

void FakeCanBus::safe_stop() noexcept {
  safe_stop_called_ = true;
  sent_frames_.clear();
}

drivers::DriverHealth FakeCanBus::health() const noexcept { return health_; }
const std::vector<drivers::CanFrame>& FakeCanBus::sent_frames() const noexcept {
  return sent_frames_;
}
bool FakeCanBus::safe_stop_called() const noexcept { return safe_stop_called_; }
void FakeCanBus::set_send_failure(drivers::ErrorCode code, std::string detail) {
  send_failure_ = drivers::Result::failure(
      code, std::move(detail), "send", "fake_can", code == drivers::ErrorCode::kCanTxTimeout);
}
void FakeCanBus::clear_send_failure() noexcept { send_failure_.reset(); }

drivers::Result FakeRs485Bus::start() {
  health_ = drivers::DriverHealth::kReady;
  return drivers::Result::success("fake RS485 ready");
}

drivers::Rs485Response FakeRs485Bus::exchange(const drivers::Rs485Request& request) {
  if (health_ != drivers::DriverHealth::kReady) {
    return {drivers::Result::failure("fake RS485 is not started"), {}};
  }
  requests_.push_back(request);
  const auto failure = failures_.find(request.logical_device);
  if (failure != failures_.end()) {
    return {failure->second, {}};
  }
  const auto response = responses_.find(request.logical_device);
  if (response == responses_.end()) {
    return {drivers::Result::failure("no fake response configured"), {}};
  }
  return {drivers::Result::success(), response->second};
}

void FakeRs485Bus::safe_stop() noexcept { safe_stop_called_ = true; }
drivers::DriverHealth FakeRs485Bus::health() const noexcept { return health_; }
void FakeRs485Bus::set_response(std::string logical_device,
                                std::vector<std::uint8_t> response) {
  responses_[std::move(logical_device)] = std::move(response);
}
void FakeRs485Bus::set_failure(std::string logical_device,
                               drivers::ErrorCode code,
                               std::string detail) {
  failures_[std::move(logical_device)] = drivers::Result::failure(
      code, std::move(detail), "exchange", "fake_rs485",
      code == drivers::ErrorCode::kRs485Timeout);
}
void FakeRs485Bus::clear_failure(const std::string& logical_device) {
  failures_.erase(logical_device);
}
const std::vector<drivers::Rs485Request>& FakeRs485Bus::requests() const noexcept {
  return requests_;
}
bool FakeRs485Bus::safe_stop_called() const noexcept { return safe_stop_called_; }

drivers::Result FakeRelayBank::start() {
  health_ = drivers::DriverHealth::kReady;
  return drivers::Result::success("fake relay bank ready");
}

drivers::Result FakeRelayBank::set(const std::string& logical_channel,
                                   drivers::RelayState state) {
  if (health_ != drivers::DriverHealth::kReady) {
    return drivers::Result::failure("fake relay bank is not started");
  }
  const auto interlock = interlocks_.find(logical_channel);
  if (interlock != interlocks_.end() && interlock->second &&
      state == drivers::RelayState::kEnergized) {
    return drivers::Result::failure(drivers::ErrorCode::kRelayInterlockActive,
                                    "fake relay interlock is active",
                                    "set",
                                    "fake_relay_bank");
  }
  states_[logical_channel] = state;
  return drivers::Result::success();
}

void FakeRelayBank::deenergize_all() noexcept {
  deenergize_all_called_ = true;
  for (auto& [channel, state] : states_) {
    static_cast<void>(channel);
    state = drivers::RelayState::kDeenergized;
  }
}

drivers::DriverHealth FakeRelayBank::health() const noexcept { return health_; }
drivers::RelayState FakeRelayBank::state_of(const std::string& logical_channel) const {
  const auto found = states_.find(logical_channel);
  return found == states_.end() ? drivers::RelayState::kDeenergized : found->second;
}
bool FakeRelayBank::deenergize_all_called() const noexcept {
  return deenergize_all_called_;
}
void FakeRelayBank::set_interlock(std::string logical_channel, bool active) {
  interlocks_[std::move(logical_channel)] = active;
}

drivers::Result FakeSensorSuite::start() {
  health_ = drivers::DriverHealth::kReady;
  return drivers::Result::success("fake sensors ready");
}
std::vector<drivers::SensorSample> FakeSensorSuite::read_all() { return samples_; }
void FakeSensorSuite::stop() noexcept { health_ = drivers::DriverHealth::kStopped; }
drivers::DriverHealth FakeSensorSuite::health() const noexcept { return health_; }
void FakeSensorSuite::set_samples(std::vector<drivers::SensorSample> samples) {
  samples_ = std::move(samples);
}

}  // namespace snappy::simulator
