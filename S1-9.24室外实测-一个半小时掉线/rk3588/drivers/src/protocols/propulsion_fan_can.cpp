#include "snappy/drivers/protocols/propulsion_fan_can.hpp"

/**
 * @file propulsion_fan_can.cpp
 * @brief Hobbywing X6 Plus HW-UAVCAN 编解码实现
 */

#include <cstddef>
#include <vector>

namespace snappy::drivers::protocols {
namespace {

bool valid_source_node_id(
    std::uint8_t node_id) {
  return node_id >= 1 &&
         node_id <= 0x7F;
}

bool valid_esc_node_id(
    std::uint8_t node_id) {
  return node_id >= 1 &&
         node_id <= 0x7D;
}

bool valid_transfer_id(
    std::uint8_t transfer_id) {
  return transfer_id <=
         kHobbywingMaxTransferId;
}

bool valid_throttle(
    std::uint16_t throttle) {
  return throttle <=
         kHobbywingMaxThrottleRaw;
}

bool valid_single_frame_tail(
    std::uint8_t value) {
  return (value & 0xC0U) ==
         kHobbywingSingleFrameTail;
}

std::uint16_t little_u16(
    const std::vector<std::uint8_t>& payload,
    std::size_t offset) {
  return static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(
          payload[offset]) |
      (static_cast<std::uint16_t>(
           payload[offset + 1])
       << 8U));
}

PropulsionFanEncodeResult make_ext_frame(
    std::uint32_t id,
    std::vector<std::uint8_t> payload) {
  if (payload.size() > 8) {
    return {
        Result::failure(
            ErrorCode::kCanFrameInvalid,
            "Hobbywing CAN payload exceeds 8 bytes",
            "encode",
            "propulsion_fan_can"),
        {}};
  }

  return {
      Result::success(),
      {id, true, std::move(payload)}};
}

Result validate_profile(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id) {
  if (!valid_source_node_id(
          profile.source_node_id) ||
      !valid_transfer_id(
          transfer_id)) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "invalid Hobbywing UAVCAN profile",
        "encode",
        "propulsion_fan_can");
  }

  return Result::success();
}

Result invalid_telemetry(
    const std::string& detail) {
  return Result::failure(
      ErrorCode::kCanFrameInvalid,
      detail,
      "decode",
      "propulsion_fan_can");
}

}  // namespace

std::uint32_t hobbywing_message_id(
    std::uint8_t priority,
    std::uint16_t data_type_id,
    std::uint8_t source_node_id) noexcept {
  return
      ((static_cast<std::uint32_t>(
            priority) &
        0x1FU)
       << 24U) |
      (static_cast<std::uint32_t>(
           data_type_id)
       << 8U) |
      (source_node_id & 0x7FU);
}

std::uint32_t hobbywing_service_request_id(
    std::uint8_t priority,
    std::uint8_t service_type_id,
    std::uint8_t destination_node_id,
    std::uint8_t source_node_id) noexcept {
  return
      ((static_cast<std::uint32_t>(
            priority) &
        0x1FU)
       << 24U) |
      (static_cast<std::uint32_t>(
           service_type_id)
       << 16U) |
      (1U << 15U) |
      ((static_cast<std::uint32_t>(
            destination_node_id) &
        0x7FU)
       << 8U) |
      (1U << 7U) |
      (source_node_id & 0x7FU);
}

std::uint16_t hobbywing_message_type_id(
    std::uint32_t can_id) noexcept {
  return static_cast<std::uint16_t>(
      (can_id >> 8U) & 0xFFFFU);
}

std::uint8_t hobbywing_source_node_id(
    std::uint32_t can_id) noexcept {
  return static_cast<std::uint8_t>(
      can_id & 0x7FU);
}

std::uint8_t hobbywing_tail_byte(
    std::uint8_t transfer_id) noexcept {
  return static_cast<std::uint8_t>(
      kHobbywingSingleFrameTail |
      (transfer_id &
       kHobbywingMaxTransferId));
}

std::vector<std::uint8_t>
hobbywing_pack4_throttle(
    std::uint16_t ch0,
    std::uint16_t ch1,
    std::uint16_t ch2,
    std::uint16_t ch3) {
  const std::uint16_t values[4] = {
      static_cast<std::uint16_t>(
          ch0 & 0x3FFFU),
      static_cast<std::uint16_t>(
          ch1 & 0x3FFFU),
      static_cast<std::uint16_t>(
          ch2 & 0x3FFFU),
      static_cast<std::uint16_t>(
          ch3 & 0x3FFFU),
  };

  std::vector<int> bits;
  bits.reserve(56);

  for (const auto value : values) {
    const auto low =
        static_cast<std::uint8_t>(
            value & 0xFFU);

    const auto high =
        static_cast<std::uint8_t>(
            (value >> 8U) & 0x3FU);

    for (int bit = 7; bit >= 0; --bit) {
      bits.push_back(
          (low >> bit) & 1U);
    }

    for (int bit = 5; bit >= 0; --bit) {
      bits.push_back(
          (high >> bit) & 1U);
    }
  }

  std::vector<std::uint8_t> out(7, 0);

  for (int index = 0;
       index < 56;
       ++index) {
    const auto byte_index =
        static_cast<std::size_t>(
            index / 8);

    out[byte_index] =
        static_cast<std::uint8_t>(
            (out[byte_index] << 1U) |
            bits[static_cast<std::size_t>(
                index)]);
  }

  return out;
}

PropulsionFanEncodeResult
encode_propulsion_fan_raw_command(
    const PropulsionFanRawCommand& command) {
  const auto valid =
      validate_profile(
          command.profile,
          command.transfer_id);

  if (!valid.ok) {
    return {valid, {}};
  }

  for (const auto throttle :
       command.throttle_raw) {
    if (!valid_throttle(throttle)) {
      return {
          Result::failure(
              ErrorCode::kInvalidArgument,
              "Hobbywing throttle must be 0..8191",
              "encode",
              "propulsion_fan_can"),
          {}};
    }
  }

  auto payload =
      hobbywing_pack4_throttle(
          command.throttle_raw[0],
          command.throttle_raw[1],
          command.throttle_raw[2],
          command.throttle_raw[3]);

  payload.push_back(
      hobbywing_tail_byte(
          command.transfer_id));

  return make_ext_frame(
      hobbywing_message_id(
          kHobbywingPriorityHighest,
          kHobbywingRawCommandDataTypeId,
          command.profile.source_node_id),
      std::move(payload));
}

PropulsionFanEncodeResult
encode_propulsion_fan_ready_control(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id,
    bool enabled) {
  const auto valid =
      validate_profile(
          profile,
          transfer_id);

  if (!valid.ok) {
    return {valid, {}};
  }

  return make_ext_frame(
      hobbywing_message_id(
          kHobbywingPriorityMedium,
          kHobbywingEscThrReadyControlDataTypeId,
          profile.source_node_id),
      {
          static_cast<std::uint8_t>(
              enabled ? 0xAA : 0x55),
          hobbywing_tail_byte(
              transfer_id),
      });
}

PropulsionFanEncodeResult
encode_propulsion_fan_select_can(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id,
    std::uint8_t esc_node_id) {
  const auto valid =
      validate_profile(
          profile,
          transfer_id);

  if (!valid.ok) {
    return {valid, {}};
  }

  if (!valid_esc_node_id(
          esc_node_id)) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "Hobbywing ESC node id must be 1..0x7D",
            "encode",
            "propulsion_fan_can"),
        {}};
  }

  return make_ext_frame(
      hobbywing_service_request_id(
          kHobbywingPriorityMedium,
          kHobbywingThrottleSelectServiceTypeId,
          esc_node_id,
          profile.source_node_id),
      {
          0x00,
          hobbywing_tail_byte(
              transfer_id),
      });
}

PropulsionFanEncodeResult
encode_propulsion_fan_get_esc_id(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id) {
  const auto valid =
      validate_profile(
          profile,
          transfer_id);

  if (!valid.ok) {
    return {valid, {}};
  }

  return make_ext_frame(
      hobbywing_message_id(
          kHobbywingPriorityMedium,
          kHobbywingGetEscIdDataTypeId,
          profile.source_node_id),
      {
          0x00,
          hobbywing_tail_byte(
              transfer_id),
      });
}

PropulsionFanEncodeResult
encode_propulsion_fan_set_id(
    const PropulsionFanCanProfile& profile,
    std::uint8_t transfer_id,
    std::uint8_t old_esc_node_id,
    std::uint8_t new_esc_node_id,
    std::uint8_t throttle_channel_one_based) {
  const auto valid =
      validate_profile(
          profile,
          transfer_id);

  if (!valid.ok) {
    return {valid, {}};
  }

  if (!valid_esc_node_id(
          old_esc_node_id) ||
      !valid_esc_node_id(
          new_esc_node_id) ||
      throttle_channel_one_based < 1 ||
      throttle_channel_one_based > 8) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "invalid Hobbywing ESC id or channel",
            "encode",
            "propulsion_fan_can"),
        {}};
  }

  return make_ext_frame(
      hobbywing_service_request_id(
          kHobbywingPriorityMedium,
          kHobbywingSetIdServiceTypeId,
          old_esc_node_id,
          profile.source_node_id),
      {
          new_esc_node_id,
          throttle_channel_one_based,
          hobbywing_tail_byte(
              transfer_id),
      });
}

PropulsionFanTelemetryDecodeResult
decode_propulsion_fan_telemetry(
    const CanFrame& frame) {
  if (!frame.extended) {
    return {
        invalid_telemetry(
            "Hobbywing telemetry must use extended CAN"),
        {}};
  }

  const auto message_type =
      hobbywing_message_type_id(
          frame.id);

  PropulsionFanTelemetry telemetry;
  telemetry.node_id =
      hobbywing_source_node_id(
          frame.id);

  if (!valid_esc_node_id(
          telemetry.node_id)) {
    return {
        invalid_telemetry(
            "invalid Hobbywing telemetry node id"),
        {}};
  }

  if (message_type ==
      kHobbywingMsg1DataTypeId) {
    if (frame.payload.size() != 7 ||
        !valid_single_frame_tail(
            frame.payload[6])) {
      return {
          invalid_telemetry(
              "invalid Hobbywing MSG1 frame"),
          {}};
    }

    telemetry.kind =
        PropulsionFanTelemetryKind::kMsg1;

    telemetry.rpm =
        little_u16(
            frame.payload,
            0);

    telemetry.throttle_raw =
        little_u16(
            frame.payload,
            2);

    telemetry.status =
        little_u16(
            frame.payload,
            4);

    return {
        Result::success(
            "Hobbywing MSG1 decoded"),
        telemetry};
  }

  if (message_type ==
      kHobbywingMsg2DataTypeId) {
    if (frame.payload.size() != 6 ||
        !valid_single_frame_tail(
            frame.payload[5])) {
      return {
          invalid_telemetry(
              "invalid Hobbywing MSG2 frame"),
          {}};
    }

    telemetry.kind =
        PropulsionFanTelemetryKind::kMsg2;

    telemetry.voltage_v =
        static_cast<double>(
            little_u16(
                frame.payload,
                0)) /
        10.0;

    telemetry.current_a =
        static_cast<double>(
            little_u16(
                frame.payload,
                2)) /
        10.0;

    telemetry.temperature_celsius =
        static_cast<double>(
            frame.payload[4]);

    return {
        Result::success(
            "Hobbywing MSG2 decoded"),
        telemetry};
  }

  if (message_type ==
      kHobbywingMsg3DataTypeId) {
    if (frame.payload.size() != 8 ||
        !valid_single_frame_tail(
            frame.payload[7])) {
      return {
          invalid_telemetry(
              "invalid Hobbywing MSG3 frame"),
          {}};
    }

    telemetry.kind =
        PropulsionFanTelemetryKind::kMsg3;

    telemetry.mos_temperature_celsius =
        static_cast<double>(
            frame.payload[0]);

    telemetry.capacitor_temperature_celsius =
        static_cast<double>(
            frame.payload[1]);

    telemetry.motor_temperature_celsius =
        static_cast<double>(
            frame.payload[2]);

    return {
        Result::success(
            "Hobbywing MSG3 decoded"),
        telemetry};
  }

  return {
      Result::failure(
          ErrorCode::kUnsupported,
          "unsupported Hobbywing telemetry frame",
          "decode",
          "propulsion_fan_can"),
      {}};
}

}  // namespace snappy::drivers::protocols