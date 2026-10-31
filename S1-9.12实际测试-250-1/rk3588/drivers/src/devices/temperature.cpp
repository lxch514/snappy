#include "snappy/drivers/devices/temperature.hpp"

#include <utility>

namespace snappy::drivers::devices {
namespace {

/* ============================================================
 * 温湿度读取辅助
 * ============================================================
 *
 * 当前温湿度模块默认按说明书使用：
 * - 站号：0x33
 * - 波特率：9600
 * - FC04 读取输入寄存器 0x0000~0x0001
 *
 * 为了兼容部分模块被配置成 FC03 读取保持寄存器的情况，
 * 这里先按配置读取；如果失败，再自动尝试 FC03/FC04 的另一种读法。
 */
TemperatureSample read_with_profile(
    IRs485Bus& bus,
    const TemperatureSensorConfig& config,
    const protocols::TemperatureModbusProfile& profile) {
  Rs485Request request;
  request.logical_device = config.device_id;
  request.payload =
      protocols::encode_temperature_read(config.station_id, profile);

  const auto response = bus.exchange(request);
  if (!response.result.ok) {
    return {response.result, std::nullopt, std::nullopt};
  }

  const auto decoded = protocols::decode_temperature_response(
      config.station_id, profile, response.payload);
  return {decoded.result, decoded.temperature_celsius, decoded.humidity_rh};
}

std::uint8_t alternate_read_function(std::uint8_t function_code) {
  if (function_code == protocols::kModbusReadInputRegisters) {
    return protocols::kModbusReadHoldingRegisters;
  }

  if (function_code == protocols::kModbusReadHoldingRegisters) {
    return protocols::kModbusReadInputRegisters;
  }

  return protocols::kModbusReadInputRegisters;
}

}  // namespace

TemperatureSensorConfig make_temperature_cabinet_config() {
  return {};
}

TemperatureSensor::TemperatureSensor(IRs485Bus& bus,
                                     TemperatureSensorConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result TemperatureSensor::start() {
  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "temperature sensor config is incomplete",
                           "start",
                           config_.device_id);
  }

  health_ = DriverHealth::kReady;
  return Result::success("temperature sensor ready");
}

void TemperatureSensor::stop() noexcept {
  health_ = DriverHealth::kStopped;
}

DriverHealth TemperatureSensor::health() const noexcept {
  return health_;
}

TemperatureSample TemperatureSensor::read() {
  if (health_ != DriverHealth::kReady) {
    return {Result::failure(ErrorCode::kConfigRequired,
                            "temperature sensor is not ready",
                            "read",
                            config_.device_id),
            std::nullopt,
            std::nullopt};
  }

  const auto primary =
      read_with_profile(bus_, config_, config_.profile);
  if (primary.result.ok) {
    return primary;
  }

  auto fallback_profile = config_.profile;
  fallback_profile.read_function_code =
      alternate_read_function(config_.profile.read_function_code);

  if (fallback_profile.read_function_code ==
      config_.profile.read_function_code) {
    return primary;
  }

  const auto fallback =
      read_with_profile(bus_, config_, fallback_profile);
  if (fallback.result.ok) {
    return fallback;
  }

  return primary;
}

}  // namespace snappy::drivers::devices