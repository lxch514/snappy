#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

struct AnemometerModbusProfile {
  std::uint16_t wind_speed_register{0x0000};
  std::uint16_t register_count{1};
  double mps_per_count{0.01};
  std::optional<double> max_valid_mps{75.0};
};

struct WindSpeedRead {
  Result result;
  std::optional<double> speed_mps;
  std::optional<double> speed_kmh;
  std::uint16_t raw_speed{0};
};

[[nodiscard]] std::vector<std::uint8_t> encode_anemometer_read_speed(
    std::uint8_t station_id,
    const AnemometerModbusProfile& profile);

[[nodiscard]] WindSpeedRead decode_anemometer_speed_response(
    std::uint8_t station_id,
    const AnemometerModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols