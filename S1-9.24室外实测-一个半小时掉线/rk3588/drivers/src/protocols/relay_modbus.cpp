#include "snappy/drivers/protocols/relay_modbus.hpp"

namespace snappy::drivers::protocols {
namespace {

std::uint16_t coil_address(const RelayModbusProfile& profile,
                           std::uint16_t channel_one_based) {
  return static_cast<std::uint16_t>(
      profile.first_coil_address + channel_one_based - 1U);
}

}  // namespace

std::vector<std::uint8_t> encode_relay_set_channel(
    const RelayModbusProfile& profile,
    const RelayChannelCommand& command) {
  return encode_write_single_coil(
      {command.station_id,
       coil_address(profile, command.channel_one_based),
       command.energized});
}

std::vector<std::uint8_t> encode_relay_set_all(
    const RelayModbusProfile& profile,
    const RelayAllCommand& command) {
  return encode_write_multiple_coils(
      {command.station_id, profile.first_coil_address, command.energized});
}

std::vector<std::uint8_t> encode_relay_read_all(
    std::uint8_t station_id,
    const RelayModbusProfile& profile) {
  return encode_read_coils(
      {station_id, kModbusReadCoils, profile.first_coil_address,
       profile.channel_count});
}

std::vector<std::uint8_t> encode_relay_deenergize_all_by_register(
    std::uint8_t station_id,
    const RelayModbusProfile& profile) {
  return encode_write_single_register(
      {station_id, profile.all_output_holding_register, 0x0000});
}

std::vector<std::uint8_t> encode_relay_set_comm_loss_timeout(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    std::uint16_t timeout_100ms_units) {
  return encode_write_single_register(
      {station_id, profile.comm_loss_timeout_register, timeout_100ms_units});
}

Result decode_relay_set_channel_response(
    const RelayModbusProfile& profile,
    const RelayChannelCommand& command,
    const std::vector<std::uint8_t>& frame) {
  const auto expected_address = coil_address(profile, command.channel_one_based);
  const auto expected_value =
      static_cast<std::uint16_t>(command.energized ? 0xFF00 : 0x0000);
  return decode_write_single_response(command.station_id,
                                      kModbusWriteSingleCoil,
                                      expected_address,
                                      expected_value,
                                      frame)
      .result;
}

Result decode_relay_set_all_response(
    const RelayModbusProfile& profile,
    const RelayAllCommand& command,
    const std::vector<std::uint8_t>& frame) {
  return decode_write_multiple_coils_response(
             command.station_id,
             profile.first_coil_address,
             static_cast<std::uint16_t>(command.energized.size()),
             frame)
      .result;
}

Result decode_relay_deenergize_all_by_register_response(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  return decode_write_single_response(station_id,
                                      kModbusWriteSingleRegister,
                                      profile.all_output_holding_register,
                                      0x0000,
                                      frame)
      .result;
}

Result decode_relay_set_comm_loss_timeout_response(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    std::uint16_t timeout_100ms_units,
    const std::vector<std::uint8_t>& frame) {
  return decode_write_single_response(station_id,
                                      kModbusWriteSingleRegister,
                                      profile.comm_loss_timeout_register,
                                      timeout_100ms_units,
                                      frame)
      .result;
}

RelayRead decode_relay_read_all_response(
    std::uint8_t station_id,
    const RelayModbusProfile& profile,
    const std::vector<std::uint8_t>& frame) {
  const auto decoded = decode_read_coils_response(
      station_id, kModbusReadCoils, profile.channel_count, frame);
  return {decoded.result, decoded.values};
}

}  // namespace snappy::drivers::protocols