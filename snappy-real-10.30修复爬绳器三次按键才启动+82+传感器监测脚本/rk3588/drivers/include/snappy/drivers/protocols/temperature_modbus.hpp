#pragma once
/**
 * @file temperature_modbus.hpp
 * @brief 温湿度 Modbus 报文编解码声明
 *
 * @details
 * - 定义寄存器映射，并提供温湿度解析入口。
 */

#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

struct TemperatureModbusProfile {
  std::uint16_t start_register{0x0000};
  std::uint16_t register_count{2};
  std::uint16_t temperature_index{0};
  std::uint16_t humidity_index{1};
  std::uint8_t read_function_code{kModbusReadInputRegisters};
  double temperature_celsius_per_count{0.1};
  double humidity_rh_per_count{0.1};
};

struct TemperatureHumidityRead {
  Result result;
  std::optional<double> temperature_celsius;
  std::optional<double> humidity_rh;
};

[[nodiscard]] std::vector<std::uint8_t> encode_temperature_read(
    std::uint8_t station_id,
    const TemperatureModbusProfile& profile);

[[nodiscard]] TemperatureHumidityRead decode_temperature_response(
    std::uint8_t station_id,
    const TemperatureModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols