#pragma once
/**
 * @file relay_modbus.hpp
 * @brief 继电器 Modbus 报文编解码声明
 *
 * @details
 * - 定义单路、多路与全通道命令结构。
 * - 提供通断命令构造与通道回读解析。
 */

#include <cstdint>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

struct RelayModbusProfile {
  std::uint16_t first_coil_address{0x0000};
  std::uint16_t channel_count{4};
  std::uint16_t all_output_holding_register{0x0034};
  std::uint16_t output_bitmap_holding_register{0x0035};
  std::uint16_t comm_loss_timeout_register{0x0030};
};

struct RelayChannelCommand {
  std::uint8_t station_id{0};
  std::uint16_t channel_one_based{0};
  bool energized{false};
};

struct RelayAllCommand {
  std::uint8_t station_id{0};
  std::vector<bool> energized;
};

struct RelayRead {
  Result result;
  std::vector<bool> energized;
};

[[nodiscard]] std::vector<std::uint8_t> encode_relay_set_channel(
    const RelayModbusProfile& profile,
    const RelayChannelCommand& command);

[[nodiscard]] std::vector<std::uint8_t> encode_relay_set_all(
    const RelayModbusProfile& profile,
    const RelayAllCommand& command);

[[nodiscard]] std::vector<std::uint8_t> encode_relay_read_all(
    std::uint8_t station_id,
    const RelayModbusProfile& profile);

[[nodiscard]] std::vector<std::uint8_t> encode_relay_deenergize_all_by_register(
    std::uint8_t station_id,
    const RelayModbusProfile& profile);

[[nodiscard]] std::vector<std::uint8_t> encode_relay_set_comm_loss_timeout(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    std::uint16_t timeout_100ms_units);

[[nodiscard]] Result decode_relay_set_channel_response(
    const RelayModbusProfile& profile,
    const RelayChannelCommand& command,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] Result decode_relay_set_all_response(
    const RelayModbusProfile& profile,
    const RelayAllCommand& command,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] Result decode_relay_deenergize_all_by_register_response(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] Result decode_relay_set_comm_loss_timeout_response(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    std::uint16_t timeout_100ms_units,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] RelayRead decode_relay_read_all_response(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    const std::vector<std::uint8_t>& frame);

}  // namespace snappy::drivers::protocols