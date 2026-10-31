#include "snappy/drivers/devices/push_motor.hpp"

#include <thread>
#include <utility>

namespace snappy::drivers::devices {

PushMotorConfig make_push_motor_config() {
  return {};
}

PushMotor::PushMotor(RelayController& relay, PushMotorConfig config)
    : relay_(relay), config_(std::move(config)) {}

Result PushMotor::start() {
  if (config_.device_id.empty() || config_.extend_channel == 0 ||
      config_.retract_channel == 0 ||
      config_.extend_channel == config_.retract_channel) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "push motor config is incomplete",
                           "start",
                           config_.device_id);
  }
  if (relay_.health() != DriverHealth::kReady) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "push motor relay is not ready",
                           "start",
                           config_.device_id);
  }

  const auto stopped = stop_motion();
  if (!stopped.ok) {
    health_ = DriverHealth::kFault;
    return stopped;
  }

  health_ = DriverHealth::kReady;
  return Result::success("push motor ready");
}

DriverHealth PushMotor::health() const noexcept {
  return health_;
}

PushMotorPosition PushMotor::position() const noexcept {
  return position_;
}

Result PushMotor::switch_direction(std::uint16_t on_channel,
                                   std::uint16_t off_channel,
                                   PushMotorPosition moving_state) {
  if (health_ != DriverHealth::kReady) {
    return Result::failure(ErrorCode::kConfigRequired,
                           "push motor is not ready",
                           "switch_direction",
                           config_.device_id);
  }

  const auto off = relay_.set_channel(off_channel, false);
  if (!off.ok) return off;

  std::this_thread::sleep_for(config_.dead_time);

  const auto on = relay_.set_channel(on_channel, true);
  if (!on.ok) return on;

  position_ = moving_state;
  energized_at_ = std::chrono::steady_clock::now();
  return Result::success("push motor direction switched");
}

Result PushMotor::extend() {
  return switch_direction(config_.extend_channel,
                          config_.retract_channel,
                          PushMotorPosition::kMovingExtend);
}

Result PushMotor::retract() {
  return switch_direction(config_.retract_channel,
                          config_.extend_channel,
                          PushMotorPosition::kMovingRetract);
}

Result PushMotor::stop_motion() {
  const auto off_extend = relay_.set_channel(config_.extend_channel, false);
  const auto off_retract = relay_.set_channel(config_.retract_channel, false);
  energized_at_.reset();

  if (!off_extend.ok) return off_extend;
  if (!off_retract.ok) return off_retract;

  if (position_ == PushMotorPosition::kMovingExtend ||
      position_ == PushMotorPosition::kMovingRetract) {
    position_ = PushMotorPosition::kAssumedReached;
  }
  return Result::success("push motor stopped");
}

Result PushMotor::poll_safety() {
  if (!energized_at_) {
    return Result::success();
  }

  const auto elapsed = std::chrono::steady_clock::now() - *energized_at_;
  if (elapsed < config_.max_on_time) {
    return Result::success();
  }

  const auto stopped = stop_motion();
  if (!stopped.ok) {
    return stopped;
  }

  return Result::failure(ErrorCode::kSafeHoldActive,
                         "push motor max energize time reached",
                         "poll_safety",
                         config_.device_id,
                         true);
}

void PushMotor::safe_stop() noexcept {
  static_cast<void>(stop_motion());
}

void PushMotor::stop() noexcept {
  safe_stop();
  health_ = DriverHealth::kStopped;
}

}  // namespace snappy::drivers::devices