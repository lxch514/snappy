#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/anemometer_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kAnemometerLeftDeviceId = "anemometer_left";
inline constexpr const char* kAnemometerRightDeviceId = "anemometer_right";

inline constexpr std::uint8_t kAnemometerLeftStationId = 0x31;
inline constexpr std::uint8_t kAnemometerRightStationId = 0x32;

struct AnemometerConfig {
  std::string device_id;
  std::uint8_t station_id{0};
  protocols::AnemometerModbusProfile profile{};
};

struct AnemometerSample {
  Result result;
  std::optional<double> speed_mps;
  std::optional<double> speed_kmh;
};

[[nodiscard]] AnemometerConfig make_anemometer_left_config();
[[nodiscard]] AnemometerConfig make_anemometer_right_config();

class Anemometer {
 public:
  Anemometer(IRs485Bus& bus, AnemometerConfig config);

  Result start();
  void stop() noexcept;
  AnemometerSample read_speed();

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  IRs485Bus& bus_;
  AnemometerConfig config_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices