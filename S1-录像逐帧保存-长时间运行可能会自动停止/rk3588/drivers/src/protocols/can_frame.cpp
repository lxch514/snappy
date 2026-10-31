#include "snappy/drivers/protocols/can_frame.hpp"

namespace snappy::drivers::protocols {

Result validate_classic_can_frame(const CanFrame& frame, std::size_t expected_size) {
  if (frame.extended) {
    if (frame.id > kExtendedCanIdMask) {
      return Result::failure(ErrorCode::kCanFrameInvalid,
                             "extended CAN id out of range",
                             "validate",
                             "can_frame");
    }
  } else if (frame.id > kStandardCanIdMask) {
    return Result::failure(ErrorCode::kCanFrameInvalid,
                           "standard CAN id out of range",
                           "validate",
                           "can_frame");
  }

  if (frame.payload.size() > kClassicCanPayloadSize) {
    return Result::failure(ErrorCode::kCanFrameInvalid,
                           "classic CAN payload exceeds 8 bytes",
                           "validate",
                           "can_frame");
  }

  if (expected_size != 0 && frame.payload.size() != expected_size) {
    return Result::failure(ErrorCode::kCanFrameInvalid,
                           "classic CAN payload length mismatch",
                           "validate",
                           "can_frame");
  }

  return Result::success();
}

CanFrame make_can_frame(std::uint32_t id,
                        bool extended,
                        const std::vector<std::uint8_t>& payload) {
  return {id, extended, payload};
}

void append_u16_le(std::vector<std::uint8_t>& payload, std::uint16_t value) {
  payload.push_back(static_cast<std::uint8_t>(value & 0xFF));
  payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
}

void append_i16_le(std::vector<std::uint8_t>& payload, std::int16_t value) {
  append_u16_le(payload, static_cast<std::uint16_t>(value));
}

void append_u32_le(std::vector<std::uint8_t>& payload, std::uint32_t value) {
  payload.push_back(static_cast<std::uint8_t>(value & 0xFF));
  payload.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
  payload.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
  payload.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));
}

void append_i32_le(std::vector<std::uint8_t>& payload, std::int32_t value) {
  append_u32_le(payload, static_cast<std::uint32_t>(value));
}

std::optional<std::uint16_t> read_u16_le(const std::vector<std::uint8_t>& payload,
                                         std::size_t offset) noexcept {
  if (offset + 2U > payload.size()) return std::nullopt;
  return static_cast<std::uint16_t>(payload[offset]) |
         (static_cast<std::uint16_t>(payload[offset + 1U]) << 8);
}

std::optional<std::int16_t> read_i16_le(const std::vector<std::uint8_t>& payload,
                                        std::size_t offset) noexcept {
  const auto raw = read_u16_le(payload, offset);
  if (!raw) return std::nullopt;
  return static_cast<std::int16_t>(*raw);
}

std::optional<std::uint32_t> read_u32_le(const std::vector<std::uint8_t>& payload,
                                         std::size_t offset) noexcept {
  if (offset + 4U > payload.size()) return std::nullopt;
  return static_cast<std::uint32_t>(payload[offset]) |
         (static_cast<std::uint32_t>(payload[offset + 1U]) << 8) |
         (static_cast<std::uint32_t>(payload[offset + 2U]) << 16) |
         (static_cast<std::uint32_t>(payload[offset + 3U]) << 24);
}

std::optional<std::int32_t> read_i32_le(const std::vector<std::uint8_t>& payload,
                                        std::size_t offset) noexcept {
  const auto raw = read_u32_le(payload, offset);
  if (!raw) return std::nullopt;
  return static_cast<std::int32_t>(*raw);
}

bool payload_has_prefix(const std::vector<std::uint8_t>& payload,
                        const std::vector<std::uint8_t>& prefix) noexcept {
  if (prefix.size() > payload.size()) return false;
  for (std::size_t i = 0; i < prefix.size(); ++i) {
    if (payload[i] != prefix[i]) return false;
  }
  return true;
}

}  // namespace snappy::drivers::protocols