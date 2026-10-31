#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/temperature_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kTemperatureCabinetDeviceId = "temperature_cabinet";
inline constexpr std::uint8_t kTemperatureCabinetStationId = 0x33;

struct TemperatureSensorConfig {
  std::string device_id{kTemperatureCabinetDeviceId};
  std::uint8_t station_id{kTemperatureCabinetStationId};
  protocols::TemperatureModbusProfile profile{};
};

struct TemperatureSample {
  Result result;
  std::optional<double> temperature_celsius;
  std::optional<double> humidity_rh;
};

[[nodiscard]] TemperatureSensorConfig make_temperature_cabinet_config();

class TemperatureSensor {
 public:
  TemperatureSensor(IRs485Bus& bus, TemperatureSensorConfig config);

  Result start();
  void stop() noexcept;
  TemperatureSample read();

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  IRs485Bus& bus_;
  TemperatureSensorConfig config_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices