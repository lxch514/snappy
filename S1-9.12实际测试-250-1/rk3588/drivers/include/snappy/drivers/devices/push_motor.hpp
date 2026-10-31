#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "snappy/drivers/devices/relay.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kPushMotorDeviceId = "push_motor";
inline constexpr std::uint16_t kPushMotorExtendRelayChannel = 1;
inline constexpr std::uint16_t kPushMotorRetractRelayChannel = 2;

enum class PushMotorPosition {
  kUnknown,
  kMovingExtend,
  kMovingRetract,
  kAssumedReached,
};

struct PushMotorConfig {
  std::string device_id{kPushMotorDeviceId};
  std::uint16_t extend_channel{kPushMotorExtendRelayChannel};
  std::uint16_t retract_channel{kPushMotorRetractRelayChannel};
  std::chrono::milliseconds dead_time{200};
  std::chrono::milliseconds max_on_time{8000};
};

[[nodiscard]] PushMotorConfig make_push_motor_config();

class PushMotor {
 public:
  PushMotor(RelayController& relay, PushMotorConfig config);

  Result start();
  Result extend();
  Result retract();
  Result stop_motion();
  Result poll_safety();
  void safe_stop() noexcept;
  void stop() noexcept;

  [[nodiscard]] DriverHealth health() const noexcept;
  [[nodiscard]] PushMotorPosition position() const noexcept;

 private:
  Result switch_direction(std::uint16_t on_channel,
                          std::uint16_t off_channel,
                          PushMotorPosition moving_state);

  RelayController& relay_;
  PushMotorConfig config_;
  PushMotorPosition position_{PushMotorPosition::kUnknown};
  std::optional<std::chrono::steady_clock::time_point> energized_at_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices