#pragma once
/**
 * @file power_modbus.hpp
 * @brief 电源 Modbus 报文编解码声明
 *
 * @details
 * - 定义输出模式与寄存器映射。
 * - 提供输出电压、电流、温度的整帧解析入口。
 */

#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

enum class PowerOutputMode {
  kOff = 0,
  kConstantCurrent = 1,
  kConstantVoltage = 2,
  kUnknown = 255,
};

struct PowerModbusProfile {
  std::uint16_t input_start_register{0x0000};
  std::uint16_t input_register_count{10};
  std::uint16_t operation_register{0x0000};

  std::uint16_t voltage_index{0};
  std::uint16_t current_index{1};
  std::uint16_t power_index{2};
  std::uint16_t temperature_index{3};
  std::uint16_t output_mode_index{4};
  std::uint16_t operation_index{5};

  double voltage_v_per_count{0.01};
  double current_a_per_count{0.01};
  double power_w_per_count{0.01};
  double temperature_celsius_per_count{0.1};
};

struct PowerRead {
  Result result;
  std::optional<double> voltage_v;
  std::optional<double> current_a;
  std::optional<double> power_w;
  std::optional<double> temperature_celsius;
  std::optional<PowerOutputMode> output_mode;
  std::optional<bool> operation_enabled;
};

[[nodiscard]] std::vector<std::uint8_t> encode_power_read(
    std::uint8_t station_id,
    const PowerModbusProfile& profile);

[[nodiscard]] PowerRead decode_power_response(
    std::uint8_t station_id,
    const PowerModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] std::vector<std::uint8_t> encode_power_set_operation(
    std::uint8_t station_id,
    const PowerModbusProfile& profile,
    bool enabled);

[[nodiscard]] Result decode_power_set_operation_response(
    std::uint8_t station_id,
    const PowerModbusProfile& profile,
    bool enabled,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols