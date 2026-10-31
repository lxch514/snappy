#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::drivers::protocols {

constexpr std::size_t kClassicCanPayloadSize = 8;
constexpr std::uint32_t kStandardCanIdMask = 0x7FF;
constexpr std::uint32_t kExtendedCanIdMask = 0x1FFFFFFF;

[[nodiscard]] Result validate_classic_can_frame(const CanFrame& frame,
                                                std::size_t expected_size = 0);

[[nodiscard]] CanFrame make_can_frame(std::uint32_t id,
                                      bool extended,
                                      const std::vector<std::uint8_t>& payload);

void append_u16_le(std::vector<std::uint8_t>& payload, std::uint16_t value);
void append_i16_le(std::vector<std::uint8_t>& payload, std::int16_t value);
void append_u32_le(std::vector<std::uint8_t>& payload, std::uint32_t value);
void append_i32_le(std::vector<std::uint8_t>& payload, std::int32_t value);

[[nodiscard]] std::optional<std::uint16_t> read_u16_le(
    const std::vector<std::uint8_t>& payload,
    std::size_t offset) noexcept;
[[nodiscard]] std::optional<std::int16_t> read_i16_le(
    const std::vector<std::uint8_t>& payload,
    std::size_t offset) noexcept;
[[nodiscard]] std::optional<std::uint32_t> read_u32_le(
    const std::vector<std::uint8_t>& payload,
    std::size_t offset) noexcept;
[[nodiscard]] std::optional<std::int32_t> read_i32_le(
    const std::vector<std::uint8_t>& payload,
    std::size_t offset) noexcept;

[[nodiscard]] bool payload_has_prefix(const std::vector<std::uint8_t>& payload,
                                      const std::vector<std::uint8_t>& prefix) noexcept;

}  // namespace snappy::drivers::protocols