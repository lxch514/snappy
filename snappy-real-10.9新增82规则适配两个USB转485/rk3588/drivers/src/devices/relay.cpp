#include "snappy/drivers/devices/relay.hpp"
/**
 * @file relay.cpp
 * @brief 继电器板设备驱动实现
 *
 * @details
 * - 写入后回读校验，不一致按失败处理。
 * - 全部断开作为安全兜底路径，任何情况下都可调用。
 */

#include <utility>

namespace snappy::drivers::devices {

RelayControllerConfig make_relay_push_motor_config() {
  RelayControllerConfig config;
  config.profile.channel_count = kRelayChannelCount;
  return config;
}

RelayController::RelayController(IRs485Bus& bus, RelayControllerConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result RelayController::start() {
  if (config_.device_id.empty() || config_.station_id == 0 ||
      config_.profile.channel_count != kRelayChannelCount) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "relay controller config is incomplete",
                           "start",
                           config_.device_id);
  }
  health_ = DriverHealth::kReady;
  return Result::success("relay controller ready");
}

DriverHealth RelayController::health() const noexcept {
  return health_;
}

Result RelayController::set_channel(std::uint16_t channel_one_based,
                                    bool energized) {
  if (health_ != DriverHealth::kReady) {
    return Result::failure(ErrorCode::kConfigRequired,
                           "relay controller is not ready",
                           "set_channel",
                           config_.device_id);
  }
  if (channel_one_based == 0 ||
      channel_one_based > config_.profile.channel_count) {
    return Result::failure(ErrorCode::kInvalidArgument,
                           "relay channel out of range",
                           "set_channel",
                           config_.device_id);
  }

  protocols::RelayChannelCommand command{
      config_.station_id, channel_one_based, energized};

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload = protocols::encode_relay_set_channel(config_.profile, command);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return response.result;
  }
  return protocols::decode_relay_set_channel_response(
      config_.profile, command, response.payload);
}

Result RelayController::set_all(const std::vector<bool>& energized) {
  if (health_ != DriverHealth::kReady) {
    return Result::failure(ErrorCode::kConfigRequired,
                           "relay controller is not ready",
                           "set_all",
                           config_.device_id);
  }
  if (energized.size() != config_.profile.channel_count) {
    return Result::failure(ErrorCode::kInvalidArgument,
                           "relay output count mismatch",
                           "set_all",
                           config_.device_id);
  }

  protocols::RelayAllCommand command{config_.station_id, energized};

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload = protocols::encode_relay_set_all(config_.profile, command);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return response.result;
  }
  return protocols::decode_relay_set_all_response(
      config_.profile, command, response.payload);
}

Result RelayController::deenergize_all() {
  if (health_ != DriverHealth::kReady) {
    return Result::failure(ErrorCode::kConfigRequired,
                           "relay controller is not ready",
                           "deenergize_all",
                           config_.device_id);
  }

  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_relay_deenergize_all_by_register(config_.station_id,
                                                         config_.profile);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return response.result;
  }
  return protocols::decode_relay_deenergize_all_by_register_response(
      config_.station_id, config_.profile, response.payload);
}

Result RelayController::set_comm_loss_timeout_100ms(
    std::uint16_t timeout_100ms_units) {
  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_relay_set_comm_loss_timeout(config_.station_id,
                                                    config_.profile,
                                                    timeout_100ms_units);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return response.result;
  }
  return protocols::decode_relay_set_comm_loss_timeout_response(
      config_.station_id, config_.profile, timeout_100ms_units, response.payload);
}

RelayReadback RelayController::read_all() {
  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_relay_read_all(config_.station_id, config_.profile);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return {response.result, {}};
  }

  auto decoded = protocols::decode_relay_read_all_response(
      config_.station_id,
      config_.profile,
      response.payload);

  return {decoded.result, std::move(decoded.energized)};
}

void RelayController::safe_stop() noexcept {
  static_cast<void>(deenergize_all());
}

void RelayController::stop() noexcept {
  safe_stop();
  health_ = DriverHealth::kStopped;
}

}  // namespace snappy::drivers::devices