#pragma once

/**
 * @file propulsion_fan_can.hpp
 * @brief Hobbywing X6 Plus HW-UAVCAN 协议声明
 *
 * @details
 * - 使用 CAN 2.0B 29-bit 扩展帧。
 * - Throttle Select=0 表示 CAN 数字油门。
 * - Ready=0xAA 表示使能，0x55 表示失能。
 * - RawCommand 使用四轴单帧格式。
 * - MSG1/MSG2/MSG3 用于缓存电调实际状态。
 */

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::drivers::protocols {

constexpr std::uint8_t kHobbywingPriorityHighest = 0;
constexpr std::uint8_t kHobbywingPriorityMedium = 16;

constexpr std::uint16_t
    kHobbywingEscThrReadyControlDataTypeId = 20012;
constexpr std::uint16_t
    kHobbywingGetEscIdDataTypeId = 20013;
constexpr std::uint16_t
    kHobbywingMsg1DataTypeId = 20050;
constexpr std::uint16_t
    kHobbywingMsg2DataTypeId = 20051;
constexpr std::uint16_t
    kHobbywingMsg3DataTypeId = 20052;
constexpr std::uint16_t
    kHobbywingRawCommandDataTypeId = 20100;

constexpr std::uint8_t
    kHobbywingSetIdServiceTypeId = 210;
constexpr std::uint8_t
    kHobbywingThrottleSelectServiceTypeId = 215;

constexpr std::uint8_t
    kHobbywingDefaultSourceNodeId = 0x64;
constexpr std::uint8_t
    kHobbywingSingleFrameTail = 0xC0;
constexpr std::uint8_t
    kHobbywingMaxTransferId = 31;

constexpr std::uint16_t
    kHobbywingMaxThrottleRaw = 8191;
constexpr std::uint16_t
    kHobbywingMinThrottleRaw = 0;

struct PropulsionFanCanProfile {
  std::uint8_t source_node_id{
      kHobbywingDefaultSourceNodeId};
};

struct PropulsionFanRawCommand {
  PropulsionFanCanProfile profile;
  std::uint8_t transfer_id{0};
  std::array<std::uint16_t, 4> throttle_raw{};
};

struct PropulsionFanEncodeResult {
  Result result;
  CanFrame frame;
};

enum class PropulsionFanTelemetryKind {
  kMsg1,
  kMsg2,
  kMsg3,
};

struct PropulsionFanTelemetry {
  PropulsionFanTelemetryKind kind{
      PropulsionFanTelemetryKind::kMsg1};

  std::uint8_t node_id{0};

  std::optional<std::uint16_t> rpm;
  std::optional<std::uint16_t> throttle_raw;
  std::optional<std::uint16_t> status;

  std::optional<double> voltage_v;
  std::optional<double> current_a;
  std::optional<double> temperature_celsius;

  std::optional<double> mos_temperature_celsius;
  std::optional<double> capacitor_temperature_celsius;
  std::optional<double> motor_temperature_celsius;
};

struct PropulsionFanTelemetryDecodeResult {
  Result result;
  PropulsionFanTelemetry telemetry;
};

[[nodiscard]] std::uint32_t hobbywing_message_id(
    std::uint8_t priority,
    std::uint16_t data_type_id,
    std::uint8_t source_node_id) noexcept;

[[nodiscard]] std::uint32_t hobbywing_service_request_id(
    std::uint8_t priority,
    std::uint8_t service_type_id,
    std::uint8_t destination_node_id,
    std::uint8_t source_node_id) noexcept;

[[nodiscard]] std::uint16_t hobbywing_message_type_id(
    std::uint32_t can_id) noexcept;

[[nodiscard]] std::uint8_t hobbywing_source_node_id(
    std::uint32_t can_id) noexcept;

[[nodiscard]] std::uint8_t hobbywing_tail_byte(
    std::uint8_t transfer_id) noexcept;

[[nodiscard]] std::vector<std::uint8_t>
hobbywing_pack4_throttle(
    std::uint16_t ch0,
    std::uint16_t ch1,
    std::uint16_t ch2,
    std::uint16_t ch3);

[[nodiscard]] PropulsionFanEncodeResult
encode_propulsion_fan_raw_command(
    const PropulsionFanRawCommand& command);

[[nodiscard]] PropulsionFanEncodeResult
encode_propulsion_fan_ready_control(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id,
    bool enabled);

[[nodiscard]] PropulsionFanEncodeResult
encode_propulsion_fan_select_can(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id,
    std::uint8_t esc_node_id);

[[nodiscard]] PropulsionFanEncodeResult
encode_propulsion_fan_get_esc_id(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id);

[[nodiscard]] PropulsionFanEncodeResult
encode_propulsion_fan_set_id(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id,
    std::uint8_t old_esc_node_id,
    std::uint8_t new_esc_node_id,
    std::uint8_t throttle_channel_one_based);

[[nodiscard]] PropulsionFanTelemetryDecodeResult
decode_propulsion_fan_telemetry(
    const CanFrame& frame);

}  // namespace snappy::drivers::protocols