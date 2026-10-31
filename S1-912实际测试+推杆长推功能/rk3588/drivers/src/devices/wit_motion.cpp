#include "snappy/drivers/devices/wit_motion.hpp"

#include <utility>

namespace snappy::drivers::devices {

WitMotionConfig make_wit_motion_config() {
  return {};
}

WitMotion::WitMotion(IRs485Bus& bus, WitMotionConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result WitMotion::start() {
  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "wit motion config is incomplete",
                           "start",
                           config_.device_id);
  }
  health_ = DriverHealth::kReady;
  return Result::success("wit motion ready");
}

void WitMotion::stop() noexcept {
  health_ = DriverHealth::kStopped;
}

DriverHealth WitMotion::health() const noexcept {
  return health_;
}

ImuMotionSample WitMotion::read_motion() {
  if (health_ != DriverHealth::kReady) {
    ImuMotionSample sample;
    sample.result =
        Result::failure(
            ErrorCode::kConfigRequired,
            "wit motion is not ready",
            "read_motion",
            config_.device_id);
    return sample;
  }

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_wit_motion_read_motion(
          config_.station_id,
          config_.profile);

  const auto response =
      bus_.exchange(request);

  if (!response.result.ok) {
    ImuMotionSample sample;
    sample.result = response.result;
    return sample;
  }

  return protocols::decode_wit_motion_motion_response(
      config_.station_id,
      config_.profile,
      response.payload);
}

ImuAttitudeSample WitMotion::read_attitude() {
  const auto motion = read_motion();
  return {motion.result, motion.roll_deg, motion.pitch_deg, motion.yaw_deg};
}

}  // namespace snappy::drivers::devices