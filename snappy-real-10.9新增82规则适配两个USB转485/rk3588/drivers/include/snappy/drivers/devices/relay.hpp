#pragma once
/**
 * @file relay.hpp
 * @brief 继电器板设备驱动声明
 *
 * @details
 * - 通过 RS485 Modbus 控制单路、多路和全通道通断。
 * - 支持回读校验，写入结果与实际状态不一致时按失败处理。
 */

#include <cstdint>
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/relay_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kRelayPushMotorDeviceId = "relay_push_motor";
inline constexpr std::uint8_t kRelayPushMotorStationId = 0x43;
inline constexpr std::uint16_t kRelayChannelCount = 4;

struct RelayControllerConfig {
  std::string device_id{kRelayPushMotorDeviceId};
  std::uint8_t station_id{kRelayPushMotorStationId};
  protocols::RelayModbusProfile profile{};
};

struct RelayReadback {
  Result result;
  std::vector<bool> energized;
};

[[nodiscard]] RelayControllerConfig make_relay_push_motor_config();

class RelayController {
 public:
  RelayController(IRs485Bus& bus, RelayControllerConfig config);

  Result start();
  Result set_channel(std::uint16_t channel_one_based, bool energized);
  Result set_all(const std::vector<bool>& energized);
  Result deenergize_all();
  Result set_comm_loss_timeout_100ms(std::uint16_t timeout_100ms_units);
  RelayReadback read_all();
  void safe_stop() noexcept;
  void stop() noexcept;

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  IRs485Bus& bus_;
  RelayControllerConfig config_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices