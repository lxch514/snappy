#pragma once

/**
 * @file modbus_rtu.hpp
 * @brief Modbus RTU 编码、解码和增量响应成帧
 *
 * @details
 * - make_modbus_response_framer() 绑定单次请求的站号、功能码、
 *   响应长度和写回显。
 * - 成帧器只判断协议帧，不访问串口。
 * - Rs485Port 通过成帧器过滤启动噪声、迟到帧和粘包。
 */

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::drivers::protocols {

constexpr std::uint8_t kModbusReadCoils =
    0x01;

constexpr std::uint8_t
    kModbusReadDiscreteInputs =
        0x02;

constexpr std::uint8_t
    kModbusReadHoldingRegisters =
        0x03;

constexpr std::uint8_t
    kModbusReadInputRegisters =
        0x04;

constexpr std::uint8_t
    kModbusWriteSingleCoil =
        0x05;

constexpr std::uint8_t
    kModbusWriteSingleRegister =
        0x06;

constexpr std::uint8_t
    kModbusWriteMultipleCoils =
        0x0F;

constexpr std::uint8_t
    kModbusWriteMultipleRegisters =
        0x10;

struct ModbusReadRegistersRequest {
  std::uint8_t station_id{0};

  std::uint8_t function_code{
      kModbusReadHoldingRegisters};

  std::uint16_t start_address{0};
  std::uint16_t register_count{0};
};

struct ModbusReadCoilsRequest {
  std::uint8_t station_id{0};

  std::uint8_t function_code{
      kModbusReadCoils};

  std::uint16_t start_address{0};
  std::uint16_t coil_count{0};
};

struct ModbusWriteSingleRegisterRequest {
  std::uint8_t station_id{0};
  std::uint16_t register_address{0};
  std::uint16_t value{0};
};

struct ModbusWriteSingleCoilRequest {
  std::uint8_t station_id{0};
  std::uint16_t coil_address{0};
  bool energized{false};
};

struct ModbusWriteMultipleCoilsRequest {
  std::uint8_t station_id{0};
  std::uint16_t start_address{0};
  std::vector<bool> values;
};

struct ModbusWriteMultipleRegistersRequest {
  std::uint8_t station_id{0};
  std::uint16_t start_address{0};
  std::vector<std::uint16_t> values;
};

struct ModbusRegisterRead {
  Result result;
  std::vector<std::uint16_t> values;
};

struct ModbusCoilRead {
  Result result;
  std::vector<bool> values;
};

struct ModbusWriteEcho {
  Result result;
  std::uint16_t address{0};
  std::uint16_t value{0};
};

struct ModbusFrameWindow {
  Result result;
  std::size_t offset{0};
  std::size_t size{0};
  bool exception{false};
};

[[nodiscard]] std::uint16_t modbus_crc16(
    const std::uint8_t* data,
    std::size_t size) noexcept;

[[nodiscard]] std::uint16_t modbus_crc16(
    const std::vector<std::uint8_t>& frame) noexcept;

[[nodiscard]] bool modbus_crc_valid(
    const std::vector<std::uint8_t>& frame) noexcept;

/*
 * 标准 Modbus FC 01/02/03/04 是无副作用读取。
 */
[[nodiscard]] bool
modbus_request_is_side_effect_free_read(
    const std::vector<std::uint8_t>& request) noexcept;

/*
 * 为指定请求构造增量成帧器。
 *
 * strict_read_byte_count:
 * - true：读取响应 byte_count 必须符合请求数量字段。
 * - false：仅验证站号、功能码、帧长度和 CRC。
 *
 * KS114 的 FC03 数量字段属于设备专用命令语义，因此使用 false。
 */
[[nodiscard]] Rs485ResponseFramer
make_modbus_response_framer(
    const std::vector<std::uint8_t>& request,
    bool strict_read_byte_count = true);

[[nodiscard]] std::vector<std::uint8_t>
encode_read_registers(
    const ModbusReadRegistersRequest& request);

[[nodiscard]] std::vector<std::uint8_t>
encode_read_coils(
    const ModbusReadCoilsRequest& request);

[[nodiscard]] std::vector<std::uint8_t>
encode_write_single_register(
    const ModbusWriteSingleRegisterRequest& request);

[[nodiscard]] std::vector<std::uint8_t>
encode_write_single_coil(
    const ModbusWriteSingleCoilRequest& request);

[[nodiscard]] std::vector<std::uint8_t>
encode_write_multiple_coils(
    const ModbusWriteMultipleCoilsRequest& request);

[[nodiscard]] std::vector<std::uint8_t>
encode_write_multiple_registers(
    const ModbusWriteMultipleRegistersRequest& request);

[[nodiscard]] ModbusRegisterRead
decode_read_registers_response(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    std::uint16_t expected_register_count,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] ModbusCoilRead
decode_read_coils_response(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    std::uint16_t expected_coil_count,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] ModbusWriteEcho
decode_write_single_response(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    std::uint16_t expected_address,
    std::uint16_t expected_value,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] ModbusWriteEcho
decode_write_multiple_coils_response(
    std::uint8_t expected_station_id,
    std::uint16_t expected_start_address,
    std::uint16_t expected_coil_count,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] ModbusWriteEcho
decode_write_multiple_registers_response(
    std::uint8_t expected_station_id,
    std::uint16_t expected_start_address,
    std::uint16_t expected_register_count,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] ModbusFrameWindow
find_modbus_frame(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    const std::vector<std::uint8_t>& buffer);

[[nodiscard]] std::optional<std::int16_t>
as_int16(
    std::uint16_t raw) noexcept;

[[nodiscard]] std::uint16_t read_u16_be(
    const std::vector<std::uint8_t>& frame,
    std::size_t offset) noexcept;

void append_u16_be(
    std::vector<std::uint8_t>& frame,
    std::uint16_t value);

}  // namespace snappy::drivers::protocols