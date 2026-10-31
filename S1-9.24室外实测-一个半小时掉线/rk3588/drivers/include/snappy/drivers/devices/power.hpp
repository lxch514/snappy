#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/power_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kPower24vDeviceId = "power_24v";
inline constexpr const char* kPower48vDeviceId = "power_48v";

inline constexpr std::uint8_t kPower24vStationId = 0x71;
inline constexpr std::uint8_t kPower48vStationId = 0x72;

struct PowerMonitorConfig {
  std::string device_id;
  std::uint8_t station_id{0};
  protocols::PowerModbusProfile profile{};
};

struct PowerSample {
  Result result;
  std::optional<double> voltage_v;
  std::optional<double> current_a;
  std::optional<double> power_w;
  std::optional<double> temperature_celsius;
  std::optional<protocols::PowerOutputMode> output_mode;
  std::optional<bool> operation_enabled;
};

[[nodiscard]] PowerMonitorConfig make_power_24v_config();
[[nodiscard]] PowerMonitorConfig make_power_48v_config();

class PowerMonitor {
 public:
  PowerMonitor(IRs485Bus& bus, PowerMonitorConfig config);

  Result start();
  Result set_operation(bool enabled);
  Result turn_on();
  Result turn_off();
  void safe_stop() noexcept;
  void stop() noexcept;
  PowerSample read_power();

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  IRs485Bus& bus_;
  PowerMonitorConfig config_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices