#include "snappy/drivers/protocols/modbus_rtu.hpp"

/**
 * @file modbus_rtu.cpp
 * @brief Modbus RTU 编码、解码和请求绑定成帧实现
 *
 * @details
 * - 成帧器识别完整标准 Modbus 帧。
 * - CRC 正确但站号不匹配的帧作为 stray frame 丢弃。
 * - 目标站号但功能码、长度、CRC 或写回显不匹配时返回稳定协议错误。
 */

#include <cstddef>
#include <string>
#include <utility>

namespace snappy::drivers::protocols {
namespace {

Result protocol_failure(
    ErrorCode code,
    std::string detail) {
  return Result::failure(
      code,
      std::move(detail),
      "modbus_rtu",
      "modbus_rtu",
      code !=
          ErrorCode::kRs485ExceptionResponse);
}

bool is_read_register_function(
    std::uint8_t function_code) {
  return function_code ==
             kModbusReadHoldingRegisters ||
         function_code ==
             kModbusReadInputRegisters;
}

bool is_read_coil_function(
    std::uint8_t function_code) {
  return function_code ==
             kModbusReadCoils ||
         function_code ==
             kModbusReadDiscreteInputs;
}

bool is_read_function(
    std::uint8_t function_code) {
  return is_read_register_function(
             function_code) ||
         is_read_coil_function(
             function_code);
}

bool is_write_function(
    std::uint8_t function_code) {
  return function_code ==
             kModbusWriteSingleCoil ||
         function_code ==
             kModbusWriteSingleRegister ||
         function_code ==
             kModbusWriteMultipleCoils ||
         function_code ==
             kModbusWriteMultipleRegisters;
}

bool known_function(
    std::uint8_t function_code) {
  return is_read_function(
             function_code) ||
         is_write_function(
             function_code);
}

void append_crc(
    std::vector<std::uint8_t>& frame) {
  const auto crc =
      modbus_crc16(frame);

  frame.push_back(
      static_cast<std::uint8_t>(
          crc & 0xFF));

  frame.push_back(
      static_cast<std::uint8_t>(
          (crc >> 8) & 0xFF));
}

std::optional<std::size_t>
response_frame_size_at(
    const std::vector<std::uint8_t>& buffer,
    std::size_t offset) {
  if (offset + 2U >
      buffer.size()) {
    return std::nullopt;
  }

  const auto function_code =
      buffer[offset + 1U];

  if ((function_code & 0x80U) != 0U) {
    return 5U;
  }

  if (is_read_function(
          function_code)) {
    if (offset + 3U >
        buffer.size()) {
      return std::nullopt;
    }

    return static_cast<std::size_t>(
        3U +
        buffer[offset + 2U] +
        2U);
  }

  if (is_write_function(
          function_code)) {
    return 8U;
  }

  return std::nullopt;
}

bool candidate_crc_valid(
    const std::vector<std::uint8_t>& buffer,
    std::size_t offset,
    std::size_t size) {
  if (offset + size >
      buffer.size()) {
    return false;
  }

  const std::vector<std::uint8_t>
      candidate(
          buffer.begin() +
              static_cast<std::ptrdiff_t>(
                  offset),
          buffer.begin() +
              static_cast<std::ptrdiff_t>(
                  offset + size));

  return modbus_crc_valid(candidate);
}

std::uint16_t request_u16(
    const std::vector<std::uint8_t>& request,
    std::size_t offset) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(
           request[offset])
       << 8U) |
      static_cast<std::uint16_t>(
          request[offset + 1U]));
}

std::optional<std::uint8_t>
expected_read_byte_count(
    const std::vector<std::uint8_t>& request) {
  if (request.size() < 8U) {
    return std::nullopt;
  }

  const auto function_code =
      request[1U];

  const auto count =
      request_u16(
          request,
          4U);

  if (is_read_register_function(
          function_code)) {
    const auto bytes =
        static_cast<std::uint32_t>(
            count) *
        2U;

    if (bytes > 255U) {
      return std::nullopt;
    }

    return static_cast<std::uint8_t>(
        bytes);
  }

  if (is_read_coil_function(
          function_code)) {
    return static_cast<std::uint8_t>(
        (count + 7U) / 8U);
  }

  return std::nullopt;
}

bool write_echo_matches_request(
    const std::vector<std::uint8_t>& request,
    const std::vector<std::uint8_t>& buffer,
    std::size_t response_offset) {
  if (request.size() < 8U ||
      response_offset + 8U >
          buffer.size()) {
    return false;
  }

  /*
   * FC05/06/0F/10 的响应前六字节均回显：
   * station + function + start/address + value/count。
   */
  for (std::size_t index = 0;
       index < 6U;
       ++index) {
    if (request[index] !=
        buffer[response_offset +
               index]) {
      return false;
    }
  }

  return true;
}

Rs485FrameScanResult need_more() {
  return {
      Rs485FrameScanState::kNeedMore,
      0,
      0,
      Result::success()};
}

Rs485FrameScanResult discard_prefix(
    std::size_t bytes) {
  return {
      Rs485FrameScanState::kDiscardPrefix,
      bytes,
      0,
      Result::success()};
}

Rs485FrameScanResult protocol_error(
    ErrorCode code,
    std::string detail,
    std::size_t offset,
    std::size_t size,
    bool retryable = true) {
  return {
      Rs485FrameScanState::kProtocolError,
      offset,
      size,
      Result::failure(
          code,
          std::move(detail),
          "frame",
          "modbus_rtu",
          retryable)};
}

bool base_response_valid(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    const std::vector<std::uint8_t>& frame,
    Result* out) {
  if (frame.size() < 5U) {
    *out = protocol_failure(
        ErrorCode::kRs485FrameIncomplete,
        "Modbus response is too short");

    return false;
  }

  if (frame[0] !=
      expected_station_id) {
    *out = protocol_failure(
        ErrorCode::
            kRs485AddressMismatch,
        "Modbus response station id mismatch");

    return false;
  }

  if (frame[1] ==
      static_cast<std::uint8_t>(
          expected_function_code |
          0x80U)) {
    *out = protocol_failure(
        ErrorCode::
            kRs485ExceptionResponse,
        "Modbus exception response received");

    return false;
  }

  if (frame[1] !=
      expected_function_code) {
    *out = protocol_failure(
        ErrorCode::
            kRs485FunctionMismatch,
        "Modbus response function code mismatch");

    return false;
  }

  if (!modbus_crc_valid(frame)) {
    *out = protocol_failure(
        ErrorCode::kRs485CrcMismatch,
        "Modbus response CRC mismatch");

    return false;
  }

  return true;
}

}  // namespace

std::uint16_t modbus_crc16(
    const std::uint8_t* data,
    std::size_t size) noexcept {
  std::uint16_t crc = 0xFFFF;

  for (std::size_t index = 0;
       index < size;
       ++index) {
    crc ^= data[index];

    for (int bit = 0;
         bit < 8;
         ++bit) {
      crc =
          (crc & 0x0001U) != 0U
              ? static_cast<std::uint16_t>(
                    (crc >> 1U) ^
                    0xA001U)
              : static_cast<std::uint16_t>(
                    crc >> 1U);
    }
  }

  return crc;
}

std::uint16_t modbus_crc16(
    const std::vector<std::uint8_t>& frame)
    noexcept {
  return modbus_crc16(
      frame.data(),
      frame.size());
}

bool modbus_crc_valid(
    const std::vector<std::uint8_t>& frame)
    noexcept {
  if (frame.size() < 4U) {
    return false;
  }

  const auto expected =
      modbus_crc16(
          frame.data(),
          frame.size() - 2U);

  const auto actual =
      static_cast<std::uint16_t>(
          frame[frame.size() - 2U]) |
      (static_cast<std::uint16_t>(
           frame[frame.size() - 1U])
       << 8U);

  return expected == actual;
}

bool modbus_request_is_side_effect_free_read(
    const std::vector<std::uint8_t>& request)
    noexcept {
  if (request.size() != 8U ||
      !modbus_crc_valid(request)) {
    return false;
  }

  return is_read_function(
      request[1U]);
}

Rs485ResponseFramer
make_modbus_response_framer(
    const std::vector<std::uint8_t>& request,
    bool strict_read_byte_count) {
  if (request.size() != 8U ||
      !modbus_crc_valid(request) ||
      !known_function(request[1U])) {
    return [](const std::vector<std::uint8_t>&)
        -> Rs485FrameScanResult {
      return protocol_error(
          ErrorCode::kInvalidArgument,
          "invalid Modbus request for response framer",
          0,
          0,
          false);
    };
  }

  const std::uint8_t expected_station =
      request[0U];

  const std::uint8_t expected_function =
      request[1U];

  const auto expected_bytes =
      strict_read_byte_count
          ? expected_read_byte_count(
                request)
          : std::nullopt;

  return [
      request,
      expected_station,
      expected_function,
      expected_bytes](
      const std::vector<std::uint8_t>& buffer)
      -> Rs485FrameScanResult {
    if (buffer.empty()) {
      return need_more();
    }

    for (std::size_t offset = 0;
         offset < buffer.size();
         ++offset) {
      if (buffer[offset] !=
          expected_station) {
        const auto stray_size =
            response_frame_size_at(
                buffer,
                offset);

        if (stray_size &&
            offset + *stray_size <=
                buffer.size() &&
            candidate_crc_valid(
                buffer,
                offset,
                *stray_size)) {
          return discard_prefix(
              offset + *stray_size);
        }

        continue;
      }

      /*
       * 当前缓冲内存在目标站号前缀。
       * 若前面是噪声，先丢弃噪声，保留目标前缀继续收包。
       */
      if (offset + 2U >
          buffer.size()) {
        return offset > 0U
                   ? discard_prefix(offset)
                   : need_more();
      }

      const auto response_function =
          buffer[offset + 1U];

      const bool exception =
          response_function ==
          static_cast<std::uint8_t>(
              expected_function |
              0x80U);

      if (!exception &&
          response_function !=
              expected_function) {
        const auto wrong_size =
            response_frame_size_at(
                buffer,
                offset);

        if (!wrong_size) {
          return offset > 0U
                     ? discard_prefix(offset)
                     : need_more();
        }

        if (offset + *wrong_size >
            buffer.size()) {
          return offset > 0U
                     ? discard_prefix(offset)
                     : need_more();
        }

        if (!candidate_crc_valid(
                buffer,
                offset,
                *wrong_size)) {
          return protocol_error(
              ErrorCode::
                  kRs485CrcMismatch,
              "target Modbus station response CRC mismatch",
              offset,
              *wrong_size);
        }

        return protocol_error(
            ErrorCode::
                kRs485FunctionMismatch,
            "target Modbus station function code mismatch",
            offset,
            *wrong_size);
      }

      const auto frame_size =
          response_frame_size_at(
              buffer,
              offset);

      if (!frame_size) {
        return offset > 0U
                   ? discard_prefix(offset)
                   : need_more();
      }

      if (offset + *frame_size >
          buffer.size()) {
        return offset > 0U
                   ? discard_prefix(offset)
                   : need_more();
      }

      if (!candidate_crc_valid(
              buffer,
              offset,
              *frame_size)) {
        return protocol_error(
            ErrorCode::kRs485CrcMismatch,
            "target Modbus response CRC mismatch",
            offset,
            *frame_size);
      }

      if (exception) {
        return protocol_error(
            ErrorCode::
                kRs485ExceptionResponse,
            "target Modbus exception response",
            offset,
            *frame_size,
            false);
      }

      if (expected_bytes &&
          is_read_function(
              expected_function) &&
          buffer[offset + 2U] !=
              *expected_bytes) {
        return protocol_error(
            ErrorCode::
                kRs485LengthMismatch,
            "target Modbus read byte count mismatch",
            offset,
            *frame_size);
      }

      if (is_write_function(
              expected_function) &&
          !write_echo_matches_request(
              request,
              buffer,
              offset)) {
        return protocol_error(
            ErrorCode::
                kRs485AddressMismatch,
            "target Modbus write echo mismatch",
            offset,
            *frame_size);
      }

      return {
          Rs485FrameScanState::kMatched,
          offset,
          *frame_size,
          Result::success()};
    }

    /*
     * 未找到可识别的完整帧。
     * 若最后存在目标站号，保留该前缀，前面的字节可丢弃。
     */
    for (std::size_t index =
             buffer.size();
         index > 0U;
         --index) {
      const auto offset = index - 1U;

      if (buffer[offset] ==
          expected_station) {
        return offset > 0U
                   ? discard_prefix(offset)
                   : need_more();
      }
    }

    return discard_prefix(
        buffer.size());
  };
}

std::uint16_t read_u16_be(
    const std::vector<std::uint8_t>& frame,
    std::size_t offset) noexcept {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(
           frame[offset])
       << 8U) |
      static_cast<std::uint16_t>(
          frame[offset + 1U]));
}

void append_u16_be(
    std::vector<std::uint8_t>& frame,
    std::uint16_t value) {
  frame.push_back(
      static_cast<std::uint8_t>(
          (value >> 8U) & 0xFFU));

  frame.push_back(
      static_cast<std::uint8_t>(
          value & 0xFFU));
}

std::vector<std::uint8_t>
encode_read_registers(
    const ModbusReadRegistersRequest& request) {
  std::vector<std::uint8_t> frame;

  frame.reserve(8U);

  frame.push_back(request.station_id);
  frame.push_back(request.function_code);

  append_u16_be(
      frame,
      request.start_address);

  append_u16_be(
      frame,
      request.register_count);

  append_crc(frame);

  return frame;
}

std::vector<std::uint8_t>
encode_read_coils(
    const ModbusReadCoilsRequest& request) {
  std::vector<std::uint8_t> frame;

  frame.reserve(8U);

  frame.push_back(request.station_id);
  frame.push_back(request.function_code);

  append_u16_be(
      frame,
      request.start_address);

  append_u16_be(
      frame,
      request.coil_count);

  append_crc(frame);

  return frame;
}

std::vector<std::uint8_t>
encode_write_single_register(
    const ModbusWriteSingleRegisterRequest&
        request) {
  std::vector<std::uint8_t> frame;

  frame.reserve(8U);

  frame.push_back(request.station_id);
  frame.push_back(
      kModbusWriteSingleRegister);

  append_u16_be(
      frame,
      request.register_address);

  append_u16_be(
      frame,
      request.value);

  append_crc(frame);

  return frame;
}

std::vector<std::uint8_t>
encode_write_single_coil(
    const ModbusWriteSingleCoilRequest&
        request) {
  std::vector<std::uint8_t> frame;

  frame.reserve(8U);

  frame.push_back(request.station_id);
  frame.push_back(
      kModbusWriteSingleCoil);

  append_u16_be(
      frame,
      request.coil_address);

  append_u16_be(
      frame,
      request.energized
          ? 0xFF00U
          : 0x0000U);

  append_crc(frame);

  return frame;
}

std::vector<std::uint8_t>
encode_write_multiple_coils(
    const ModbusWriteMultipleCoilsRequest&
        request) {
  const auto count =
      static_cast<std::uint16_t>(
          request.values.size());

  const auto byte_count =
      static_cast<std::uint8_t>(
          (count + 7U) / 8U);

  std::vector<std::uint8_t> frame;

  frame.reserve(
      9U + byte_count);

  frame.push_back(request.station_id);
  frame.push_back(
      kModbusWriteMultipleCoils);

  append_u16_be(
      frame,
      request.start_address);

  append_u16_be(
      frame,
      count);

  frame.push_back(byte_count);

  for (std::uint8_t byte_index = 0;
       byte_index < byte_count;
       ++byte_index) {
    std::uint8_t packed = 0;

    for (int bit = 0;
         bit < 8;
         ++bit) {
      const auto value_index =
          static_cast<std::size_t>(
              byte_index * 8U +
              bit);

      if (value_index <
              request.values.size() &&
          request.values[value_index]) {
        packed |=
            static_cast<std::uint8_t>(
                1U << bit);
      }
    }

    frame.push_back(packed);
  }

  append_crc(frame);

  return frame;
}

std::vector<std::uint8_t>
encode_write_multiple_registers(
    const ModbusWriteMultipleRegistersRequest&
        request) {
  const auto count =
      static_cast<std::uint16_t>(
          request.values.size());

  const auto byte_count =
      static_cast<std::uint8_t>(
          count * 2U);

  std::vector<std::uint8_t> frame;

  frame.reserve(
      9U + byte_count);

  frame.push_back(request.station_id);
  frame.push_back(
      kModbusWriteMultipleRegisters);

  append_u16_be(
      frame,
      request.start_address);

  append_u16_be(
      frame,
      count);

  frame.push_back(byte_count);

  for (const auto value :
       request.values) {
    append_u16_be(
        frame,
        value);
  }

  append_crc(frame);

  return frame;
}

ModbusRegisterRead
decode_read_registers_response(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    std::uint16_t expected_register_count,
    const std::vector<std::uint8_t>& frame) {
  if (!is_read_register_function(
          expected_function_code)) {
    return {
        protocol_failure(
            ErrorCode::kInvalidArgument,
            "expected function code is not a register read"),
        {}};
  }

  Result base;

  if (!base_response_valid(
          expected_station_id,
          expected_function_code,
          frame,
          &base)) {
    return {
        base,
        {}};
  }

  const auto expected_byte_count =
      static_cast<std::uint16_t>(
          expected_register_count *
          2U);

  if (frame[2U] !=
      expected_byte_count) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus register byte count mismatch"),
        {}};
  }

  if (frame.size() !=
      static_cast<std::size_t>(
          3U +
          expected_byte_count +
          2U)) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus register response length mismatch"),
        {}};
  }

  std::vector<std::uint16_t> values;

  values.reserve(
      expected_register_count);

  for (std::uint16_t index = 0;
       index < expected_register_count;
       ++index) {
    values.push_back(
        read_u16_be(
            frame,
            3U +
                static_cast<std::size_t>(
                    index * 2U)));
  }

  return {
      Result::success(),
      values};
}

ModbusCoilRead
decode_read_coils_response(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    std::uint16_t expected_coil_count,
    const std::vector<std::uint8_t>& frame) {
  if (!is_read_coil_function(
          expected_function_code)) {
    return {
        protocol_failure(
            ErrorCode::kInvalidArgument,
            "expected function code is not a coil read"),
        {}};
  }

  Result base;

  if (!base_response_valid(
          expected_station_id,
          expected_function_code,
          frame,
          &base)) {
    return {
        base,
        {}};
  }

  const auto expected_byte_count =
      static_cast<std::uint8_t>(
          (expected_coil_count +
           7U) /
          8U);

  if (frame[2U] !=
      expected_byte_count) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus coil byte count mismatch"),
        {}};
  }

  if (frame.size() !=
      static_cast<std::size_t>(
          3U +
          expected_byte_count +
          2U)) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus coil response length mismatch"),
        {}};
  }

  std::vector<bool> values;

  values.reserve(
      expected_coil_count);

  for (std::uint16_t index = 0;
       index < expected_coil_count;
       ++index) {
    const auto byte =
        frame[
            3U +
            index / 8U];

    values.push_back(
        (byte &
         (1U <<
          (index % 8U))) !=
        0U);
  }

  return {
      Result::success(),
      values};
}

ModbusWriteEcho
decode_write_single_response(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    std::uint16_t expected_address,
    std::uint16_t expected_value,
    const std::vector<std::uint8_t>& frame) {
  if (frame.size() != 8U) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus write response length mismatch"),
        0,
        0};
  }

  Result base;

  if (!base_response_valid(
          expected_station_id,
          expected_function_code,
          frame,
          &base)) {
    return {
        base,
        0,
        0};
  }

  const auto address =
      read_u16_be(
          frame,
          2U);

  const auto value =
      read_u16_be(
          frame,
          4U);

  if (address !=
          expected_address ||
      value !=
          expected_value) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485AddressMismatch,
            "Modbus write echo does not match request"),
        address,
        value};
  }

  return {
      Result::success(),
      address,
      value};
}

ModbusWriteEcho
decode_write_multiple_coils_response(
    std::uint8_t expected_station_id,
    std::uint16_t expected_start_address,
    std::uint16_t expected_coil_count,
    const std::vector<std::uint8_t>& frame) {
  if (frame.size() != 8U) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus multiple coil response length mismatch"),
        0,
        0};
  }

  Result base;

  if (!base_response_valid(
          expected_station_id,
          kModbusWriteMultipleCoils,
          frame,
          &base)) {
    return {
        base,
        0,
        0};
  }

  const auto address =
      read_u16_be(
          frame,
          2U);

  const auto count =
      read_u16_be(
          frame,
          4U);

  if (address !=
          expected_start_address ||
      count !=
          expected_coil_count) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485AddressMismatch,
            "Modbus multiple coil echo does not match request"),
        address,
        count};
  }

  return {
      Result::success(),
      address,
      count};
}

ModbusWriteEcho
decode_write_multiple_registers_response(
    std::uint8_t expected_station_id,
    std::uint16_t expected_start_address,
    std::uint16_t expected_register_count,
    const std::vector<std::uint8_t>& frame) {
  if (frame.size() != 8U) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485LengthMismatch,
            "Modbus multiple register response length mismatch"),
        0,
        0};
  }

  Result base;

  if (!base_response_valid(
          expected_station_id,
          kModbusWriteMultipleRegisters,
          frame,
          &base)) {
    return {
        base,
        0,
        0};
  }

  const auto address =
      read_u16_be(
          frame,
          2U);

  const auto count =
      read_u16_be(
          frame,
          4U);

  if (address !=
          expected_start_address ||
      count !=
          expected_register_count) {
    return {
        protocol_failure(
            ErrorCode::
                kRs485AddressMismatch,
            "Modbus multiple register echo does not match request"),
        address,
        count};
  }

  return {
      Result::success(),
      address,
      count};
}

ModbusFrameWindow
find_modbus_frame(
    std::uint8_t expected_station_id,
    std::uint8_t expected_function_code,
    const std::vector<std::uint8_t>& buffer) {
  for (std::size_t offset = 0;
       offset + 4U <=
           buffer.size();
       ++offset) {
    if (buffer[offset] !=
        expected_station_id) {
      continue;
    }

    const auto function_code =
        buffer[offset + 1U];

    const bool exception =
        function_code ==
        static_cast<std::uint8_t>(
            expected_function_code |
            0x80U);

    if (!exception &&
        function_code !=
            expected_function_code) {
      continue;
    }

    const auto frame_size =
        response_frame_size_at(
            buffer,
            offset);

    if (!frame_size ||
        offset + *frame_size >
            buffer.size()) {
      continue;
    }

    if (!candidate_crc_valid(
            buffer,
            offset,
            *frame_size)) {
      continue;
    }

    return {
        Result::success(),
        offset,
        *frame_size,
        exception};
  }

  return {
      Result::failure(
          ErrorCode::kRs485Timeout,
          "no valid Modbus frame found",
          "find_frame",
          "modbus_rtu",
          true),
      0,
      0,
      false};
}

std::optional<std::int16_t> as_int16(
    std::uint16_t raw) noexcept {
  return static_cast<std::int16_t>(
      raw);
}

}  // namespace snappy::drivers::protocols