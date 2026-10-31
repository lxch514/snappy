/**
 * @file g20_command_router.cpp
 * @brief G20 command routing implementation.
 */

#include "snappy/gateway/g20/g20_command_router.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "snappy/gateway/diagnostics/gateway_log.hpp"
#include "snappy/gateway/g20/g20_packet_codec.hpp"

namespace snappy::gateway::g20 {
namespace {

constexpr std::size_t kMaxCommandsPerG20Packet = 4;
constexpr std::size_t kMaxCommandsPerPeriodicTick = 2;

constexpr auto kReliableRecordTtl =
    std::chrono::seconds(30);

constexpr std::size_t kMaxReliableRecords = 256;

int int_param(
    const nlohmann::json& command,
    const char* key,
    int fallback) {
  if (!command.contains("params") ||
      !command.at("params").is_object()) {
    return fallback;
  }

  const auto& params = command.at("params");

  if (!params.contains(key)) {
    return fallback;
  }

  const auto& value = params.at(key);

  try {
    if (value.is_number_integer()) {
      return value.get<int>();
    }

    if (value.is_number()) {
      return static_cast<int>(value.get<double>());
    }

    if (value.is_string()) {
      return std::stoi(value.get<std::string>());
    }
  } catch (...) {
  }

  return fallback;
}

std::string string_value(
    const nlohmann::json& object,
    const char* key,
    const std::string& fallback = {}) {
  if (!object.contains(key)) {
    return fallback;
  }

  try {
    if (object.at(key).is_string()) {
      return object.at(key).get<std::string>();
    }
  } catch (...) {
  }

  return fallback;
}

std::uint64_t sequence_value(
    const nlohmann::json& object) {
  const nlohmann::json* value = nullptr;

  if (object.contains("seq")) {
    value = &object.at("seq");
  } else if (object.contains("sequence")) {
    value = &object.at("sequence");
  }

  if (value == nullptr) {
    return 0;
  }

  try {
    if (value->is_number_unsigned()) {
      return value->get<std::uint64_t>();
    }

    if (value->is_number_integer()) {
      const auto signed_value =
          value->get<std::int64_t>();

      return signed_value >= 0
          ? static_cast<std::uint64_t>(signed_value)
          : 0;
    }

    if (value->is_string()) {
      return std::stoull(value->get<std::string>());
    }
  } catch (...) {
  }

  return 0;
}

bool bool_value(
    const nlohmann::json& object,
    const char* key,
    bool fallback = false) {
  if (!object.contains(key)) {
    return fallback;
  }

  try {
    const auto& value = object.at(key);

    if (value.is_boolean()) {
      return value.get<bool>();
    }

    if (value.is_number_integer()) {
      return value.get<int>() != 0;
    }

    if (value.is_string()) {
      std::string text =
          value.get<std::string>();

      std::transform(
          text.begin(),
          text.end(),
          text.begin(),
          [](unsigned char character) {
            return static_cast<char>(
                std::toupper(character));
          });

      return text == "TRUE" ||
             text == "YES" ||
             text == "Y" ||
             text == "1" ||
             text == "RELIABLE";
    }
  } catch (...) {
  }

  return fallback;
}

bool is_reliable_channel_packet(
    const nlohmann::json& packet) {
  if (!bool_value(packet, "reliable", false)) {
    return false;
  }

  if (!packet.contains("channels") ||
      !packet.at("channels").is_array()) {
    return false;
  }

  const std::string kind =
      string_value(packet, "kind");

  const std::string type =
      string_value(packet, "type");

  return kind == "g20_channels" ||
         kind == "channels" ||
         type == "g20_channels" ||
         type == "channels" ||
         type == "G20" ||
         packet.contains("message_id");
}

/*
 * 保活报文判定（g20_keepalive）。
 *
 * 遥控端以 10Hz 发送这条报文，只表示"App 还活着"，没有任何动作需要执行。
 * 但它既没有 "reliable" 字段（is_reliable_channel_packet 第一行就返回 false），
 * 也不是 legacy 信封（legacy_.accepts 只认 g20_channels/buttons/rc_buttons），
 * 于是原实现每个保活都回一条 FAILED "unsupported G20 packet"。
 *
 * 实测（2026-09-24 14:47 抓包，6.56 秒窗口）：
 *   64 × 156B 保活  ->  64 × 182B FAILED 应答，即 10 次/秒、一次不落；
 *   Android logcat 4 分钟内累积 2310 条同类日志。
 *
 * 链路存活性本来就不依赖这条应答：G20UdpTransport::receive() 在解析
 * 报文之前就调用 peer_.observe()。所以保活必须被静默接受、不回 ACK。
 */
bool is_keepalive_packet(
    const nlohmann::json& packet) {
  if (!packet.is_object()) {
    return false;
  }

  const std::string kind =
      string_value(packet, "kind");

  const std::string type =
      string_value(packet, "type");

  return kind == "g20_keepalive" ||
         kind == "keepalive" ||
         type == "g20_keepalive" ||
         type == "keepalive";
}

bool valid_reliable_channel_packet(
    const nlohmann::json& packet,
    const std::string& expected_robot_id,
    std::string& detail) {
  const std::string robot_id =
      string_value(packet, "robot_id");

  const std::string source_id =
      string_value(
          packet,
          "source_id",
          string_value(
              packet,
              "source",
              string_value(
                  packet,
                  "sender_id")));

  const std::string session_id =
      string_value(
          packet,
          "session_id",
          string_value(packet, "session"));

  const std::string message_id =
      string_value(
          packet,
          "message_id",
          string_value(packet, "id"));

  const std::uint64_t sequence =
      sequence_value(packet);

  if (!expected_robot_id.empty() &&
      robot_id != expected_robot_id) {
    detail =
        "reliable G20 robot_id mismatch";
    return false;
  }

  if (source_id.empty()) {
    detail =
        "reliable G20 source_id is required";
    return false;
  }

  if (session_id.empty()) {
    detail =
        "reliable G20 session_id is required";
    return false;
  }

  if (message_id.empty()) {
    detail =
        "reliable G20 message_id is required";
    return false;
  }

  if (sequence == 0) {
    detail =
        "reliable G20 sequence is required";
    return false;
  }

  if (!packet.contains("channels") ||
      !packet.at("channels").is_array() ||
      packet.at("channels").empty()) {
    detail =
        "reliable G20 channels are required";
    return false;
  }

  detail.clear();
  return true;
}

std::string uppercase_ascii(
    std::string value) {
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char character) {
        return static_cast<char>(
            std::toupper(character));
      });

  return value;
}

bool json_bool_value(
    const nlohmann::json& object,
    const char* key,
    bool fallback = false) {
  if (!object.contains(key)) {
    return fallback;
  }

  try {
    const auto& value = object.at(key);

    if (value.is_boolean()) {
      return value.get<bool>();
    }

    if (value.is_number_integer()) {
      return value.get<int>() != 0;
    }

    if (value.is_string()) {
      const std::string text =
          uppercase_ascii(
              value.get<std::string>());

      return text == "TRUE" ||
             text == "YES" ||
             text == "Y" ||
             text == "1" ||
             text == "OK" ||
             text == "SUCCESS";
    }
  } catch (...) {
  }

  return fallback;
}

bool ack_succeeded(
    const std::string& payload) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    if (!root.is_object()) {
      return false;
    }

    const char* boolean_keys[] = {
        "ok",
        "success",
        "accepted",
        "succeeded",
    };

    for (const auto* key : boolean_keys) {
      if (json_bool_value(root, key)) {
        return true;
      }
    }

    if (!root.contains("status") ||
        !root.at("status").is_string()) {
      return false;
    }

    const std::string status =
        uppercase_ascii(
            root.at("status").get<std::string>());

    return status == "SUCCEEDED" ||
           status == "SUCCESS" ||
           status == "OK" ||
           status == "DONE" ||
           status == "COMPLETED" ||
           status == "EXECUTED" ||
           status == "ACCEPTED" ||
           status == "RUNNING";
  } catch (...) {
    return false;
  }
}

std::string ack_status(
    const std::string& payload) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    if (!root.is_object()) {
      return "INVALID";
    }

    if (root.contains("status") &&
        root.at("status").is_string()) {
      return uppercase_ascii(
          root.at("status").get<std::string>());
    }

    if (json_bool_value(root, "succeeded") ||
        json_bool_value(root, "success") ||
        json_bool_value(root, "ok")) {
      return "SUCCEEDED";
    }

    if (json_bool_value(root, "accepted")) {
      return "ACCEPTED";
    }

    return "UNKNOWN";
  } catch (...) {
    return "INVALID";
  }
}

bool ack_execution_succeeded(
    const std::string& payload) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    if (!root.is_object()) {
      return false;
    }

    const char* boolean_keys[] = {
        "execution_succeeded",
        "executed",
        "completed",
        "succeeded",
        "success",
        "ok",
    };

    for (const auto* key : boolean_keys) {
      if (json_bool_value(root, key)) {
        return true;
      }
    }

    if (!root.contains("status") ||
        !root.at("status").is_string()) {
      return false;
    }

    const std::string status =
        uppercase_ascii(
            root.at("status").get<std::string>());

    return status == "SUCCEEDED" ||
           status == "SUCCESS" ||
           status == "OK" ||
           status == "DONE" ||
           status == "COMPLETED" ||
           status == "EXECUTED";
  } catch (...) {
    return false;
  }
}

bool ack_execution_pending(
    const std::string& payload) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    if (!root.is_object()) {
      return false;
    }

    if (json_bool_value(root, "pending") ||
        json_bool_value(root, "running")) {
      return true;
    }

    if (!root.contains("status") ||
        !root.at("status").is_string()) {
      return false;
    }

    const std::string status =
        uppercase_ascii(
            root.at("status").get<std::string>());

    return status == "RUNNING" ||
           status == "PENDING" ||
           status == "QUEUED" ||
           status == "IN_PROGRESS" ||
           status == "PROCESSING";
  } catch (...) {
    return false;
  }
}

bool ack_retryable(
    const std::string& payload) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    if (!root.is_object()) {
      return false;
    }

    return json_bool_value(
        root,
        "retryable",
        false);
  } catch (...) {
    return false;
  }
}

std::string ack_detail(
    const std::string& payload) {
  try {
    const auto root =
        nlohmann::json::parse(payload);

    if (!root.is_object()) {
      return "invalid control ACK";
    }

    if (root.contains("detail") &&
        root.at("detail").is_string()) {
      return root.at("detail").get<std::string>();
    }

    if (root.contains("message") &&
        root.at("message").is_string()) {
      return root.at("message").get<std::string>();
    }

    return {};
  } catch (...) {
    return "invalid control ACK";
  }
}

std::string canonical_json(
    const nlohmann::json& value) {
  if (value.is_object()) {
    std::vector<std::string> keys;

    for (auto it = value.begin();
         it != value.end();
         ++it) {
      keys.push_back(it.key());
    }

    std::sort(
        keys.begin(),
        keys.end());

    std::string output = "{";

    for (std::size_t index = 0;
         index < keys.size();
         ++index) {
      if (index != 0) {
        output += ",";
      }

      output +=
          nlohmann::json(keys[index]).dump();
      output += ":";
      output +=
          canonical_json(
              value.at(keys[index]));
    }

    output += "}";
    return output;
  }

  if (value.is_array()) {
    std::string output = "[";

    for (std::size_t index = 0;
         index < value.size();
         ++index) {
      if (index != 0) {
        output += ",";
      }

      output +=
          canonical_json(value.at(index));
    }

    output += "]";
    return output;
  }

  return value.dump();
}

std::string payload_fingerprint(
    const nlohmann::json& packet) {
  return std::to_string(
      std::hash<std::string>{}(
          canonical_json(packet)));
}

RoutedCommandResult make_result(
    bool processed,
    bool control_succeeded,
    bool execution_succeeded,
    bool execution_pending,
    bool ack_published,
    bool retryable,
    std::string detail) {
  RoutedCommandResult result;

  result.processed = processed;
  result.control_succeeded =
      control_succeeded;
  result.execution_succeeded =
      execution_succeeded;
  result.execution_pending =
      execution_pending;
  result.ack_published =
      ack_published;
  result.retryable = retryable;
  result.forwarded =
      control_succeeded;
  result.detail =
      std::move(detail);

  return result;
}

void log_route_result(
    const nlohmann::json& command,
    const RoutedCommandResult& result) {
  const std::string action =
      command.value(
          "action",
          "unknown");

  const std::string text =
      "action=" +
      action +
      " processed=" +
      (result.processed ? "true" : "false") +
      " control_succeeded=" +
      (result.control_succeeded
           ? "true"
           : "false") +
      " execution_succeeded=" +
      (result.execution_succeeded
           ? "true"
           : "false") +
      " execution_pending=" +
      (result.execution_pending
           ? "true"
           : "false") +
      " retryable=" +
      (result.retryable
           ? "true"
           : "false") +
      " detail=" +
      result.detail;

  if (action == "control.heartbeat") {
    SNAPPY_GATEWAY_LOG_DEBUG(
        "g20.result",
        text);
  } else {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.result",
        text);
  }
}

}  // namespace

G20CommandRouter::G20CommandRouter(
    std::string robot_id,
    IControlChannel& control_channel,
    IVideoAdapter* video_adapter)
    : robot_id_(std::move(robot_id)),
      control_channel_(control_channel),
      video_adapter_(video_adapter) {}

void G20CommandRouter::set_telemetry_control(
    std::function<void(bool)> callback) {
  telemetry_control_ =
      std::move(callback);
}

bool G20CommandRouter::publish_ack(
    ITransport& transport,
    const nlohmann::json& command,
    const std::string& status,
    const std::string& detail,
    bool retryable) const {
  return transport.publish(
      PublishedMessage{
          "snappy/v1/robots/" +
              robot_id_ +
              "/up/ack",
          encode_ack(
              robot_id_,
              command,
              status,
              detail,
              retryable),
          1,
          false,
      });
}

bool G20CommandRouter::publish_video_state(
    ITransport& transport) const {
  if (video_adapter_ == nullptr) {
    return false;
  }

  const auto state =
      video_adapter_->video_state();

  nlohmann::json root;

  root["schema_version"] = 1;
  root["kind"] = "video";
  root["robot_id"] = robot_id_;
  root["running"] = state.running;
  root["active_camera"] = state.active_camera;
  root["active_url"] = state.active_url;
  root["camera1_url"] = state.camera1_url;
  root["camera2_url"] = state.camera2_url;
  root["target_address"] = state.target_address;
  root["camera1_port"] = state.camera1_port;
  root["camera2_port"] = state.camera2_port;
  root["detail"] = state.detail;
  root["updated_at_ms"] = wall_time_ms();

  return transport.publish(
      PublishedMessage{
          "snappy/v1/robots/" +
              robot_id_ +
              "/up/video",
          root.dump(),
          1,
          false,
      });
}

void G20CommandRouter::
    prune_reliable_channel_deliveries() {
  const auto now =
      std::chrono::steady_clock::now();

  for (auto it =
           reliable_channel_deliveries_.begin();
       it != reliable_channel_deliveries_.end();) {
    const auto state =
        it->second.state;

    const bool completed =
        state ==
            ReliableDeliveryState::kAccepted ||
        state ==
            ReliableDeliveryState::kSucceeded ||
        state ==
            ReliableDeliveryState::kFailedPermanent;

    if (completed &&
        now - it->second.updated_at >
            kReliableRecordTtl) {
      it =
          reliable_channel_deliveries_.erase(it);
    } else {
      ++it;
    }
  }
}

RoutedCommandResult G20CommandRouter::route(
    const InboundMessage& inbound,
    ITransport& reply_transport) {
  const auto decoded =
      decode_command(
          inbound.payload,
          robot_id_);

  if (decoded.ok) {
    const auto result =
        route_command_json(
            decoded.command,
            reply_transport);

    log_route_result(
        decoded.command,
        result);

    return result;
  }

  nlohmann::json legacy_packet;

  try {
    legacy_packet =
        nlohmann::json::parse(
            inbound.payload);
  } catch (...) {
    nlohmann::json empty_command;

    empty_command["message_id"] = "";
    empty_command["seq"] = 0;

    const bool published =
        publish_ack(
            reply_transport,
            empty_command,
            "FAILED",
            decoded.detail,
            false);

    return make_result(
        true,
        false,
        false,
        false,
        published,
        false,
        decoded.detail);
  }

  const bool reliable =
      is_reliable_channel_packet(
          legacy_packet);

  /*
   * 保活报文：静默接受，不回 ACK。
   *
   * 放在 accepts/reliable 判定之前，否则它会落进下面的
   * "unsupported G20 packet" 分支，每秒产生 10 条 FAILED 应答
   * （详见 is_keepalive_packet 的说明）。
   */
  if (is_keepalive_packet(legacy_packet)) {
    return make_result(
        true,
        false,
        false,
        false,
        false,
        false,
        "g20 keepalive accepted");
  }

  /*
   * Reliable packets may use a compatible legacy envelope.
   * Non-reliable packets still require the original adapter format.
   */
  if (!legacy_.accepts(legacy_packet) &&
      !reliable) {
    nlohmann::json empty_command;

    empty_command["message_id"] = "";
    empty_command["seq"] = 0;

    const bool published =
        publish_ack(
            reply_transport,
            empty_command,
            "FAILED",
            "unsupported G20 packet",
            false);

    return make_result(
        true,
        false,
        false,
        false,
        published,
        false,
        "unsupported G20 packet");
  }

  if (!reliable) {
    const auto commands =
        legacy_.adapt(legacy_packet);

    return route_command_batch(
        commands,
        reply_transport,
        kMaxCommandsPerG20Packet,
        "G20 channels adapted");
  }

  std::string validation_detail;

  if (!valid_reliable_channel_packet(
          legacy_packet,
          robot_id_,
          validation_detail)) {
    const bool published =
        publish_ack(
            reply_transport,
            legacy_packet,
            "FAILED",
            validation_detail,
            false);

    SNAPPY_GATEWAY_LOG_WARN(
        "g20.delivery",
        "event=invalid seq=" +
            std::to_string(
                sequence_value(legacy_packet)) +
            " detail=" +
            validation_detail +
            " ack=" +
            (published ? "sent" : "failed"));

    return make_result(
        true,
        false,
        false,
        false,
        published,
        false,
        validation_detail);
  }

  const std::string source_id =
      string_value(
          legacy_packet,
          "source_id",
          string_value(
              legacy_packet,
              "source",
              string_value(
                  legacy_packet,
                  "sender_id")));

  const std::string session_id =
      string_value(
          legacy_packet,
          "session_id",
          string_value(
              legacy_packet,
              "session"));

  const std::string message_id =
      string_value(
          legacy_packet,
          "message_id",
          string_value(
              legacy_packet,
              "id"));

  const std::uint64_t sequence =
      sequence_value(legacy_packet);

  /*
   * Deduplication identity is source + session + message ID.
   * Sequence and payload are used for conflict detection.
   */
  const std::string key =
      source_id +
      "\x1f" +
      session_id +
      "\x1f" +
      message_id;

  const std::string fingerprint =
      payload_fingerprint(legacy_packet);

  prune_reliable_channel_deliveries();

  auto found =
      reliable_channel_deliveries_.find(key);

  if (found !=
      reliable_channel_deliveries_.end()) {
    auto& record =
        found->second;

    if (record.sequence != sequence) {
      const bool published =
          publish_ack(
              reply_transport,
              legacy_packet,
              "FAILED",
              "message_id sequence conflict",
              false);

      return make_result(
          true,
          false,
          false,
          false,
          published,
          false,
          "message_id sequence conflict");
    }

    if (record.payload_hash != fingerprint) {
      const bool published =
          publish_ack(
              reply_transport,
              legacy_packet,
              "FAILED",
              "message_id payload conflict",
              false);

      return make_result(
          true,
          false,
          false,
          false,
          published,
          false,
          "message_id payload conflict");
    }

    if (record.state ==
        ReliableDeliveryState::kPending) {
      const auto now =
          std::chrono::steady_clock::now();

      if (now - record.updated_at >
          kReliableRecordTtl) {
        record.state =
            ReliableDeliveryState::kFailedRetryable;
        record.ack_status = "FAILED";
        record.retryable = true;
        record.detail =
            "delivery pending timeout";
        record.updated_at = now;
      } else {
        const bool published =
            publish_ack(
                reply_transport,
                legacy_packet,
                "RUNNING",
                "delivery is in progress",
                false);

        SNAPPY_GATEWAY_LOG_INFO(
            "g20.delivery",
            "event=duplicate source_id=" +
                source_id +
                " session_id=" +
                session_id +
                " message_id=" +
                message_id +
                " seq=" +
                std::to_string(sequence) +
                " action=ignored status=RUNNING ack=" +
                (published ? "resent" : "failed"));

        auto result =
            make_result(
                true,
                true,
                false,
                true,
                published,
                false,
                "delivery is in progress");

        result.forwarded = false;
        return result;
      }
    }

    /*
     * Terminal records resend the original state and never
     * call the mapper or control channel again.
     */
    const bool terminal =
        record.state ==
            ReliableDeliveryState::kAccepted ||
        record.state ==
            ReliableDeliveryState::kSucceeded ||
        record.state ==
            ReliableDeliveryState::kFailedPermanent;

    if (terminal) {
      const bool published =
          publish_ack(
              reply_transport,
              legacy_packet,
              record.ack_status,
              record.detail,
              record.retryable);

      const bool control_succeeded =
          record.state !=
          ReliableDeliveryState::kFailedPermanent;

      const bool execution_succeeded =
          record.state ==
          ReliableDeliveryState::kSucceeded;

      SNAPPY_GATEWAY_LOG_INFO(
          "g20.delivery",
          "event=duplicate source_id=" +
              source_id +
              " session_id=" +
              session_id +
              " message_id=" +
              message_id +
              " seq=" +
              std::to_string(sequence) +
              " action=ignored status=" +
              record.ack_status +
              " ack=" +
              (published ? "resent" : "failed"));

      auto result =
          make_result(
              true,
              control_succeeded,
              execution_succeeded,
              false,
              published,
              record.retryable,
              record.detail);

      /*
       * This invocation did not forward anything, even though
       * the cached delivery was successful.
       */
      result.forwarded = false;
      return result;
    }

    /*
     * Only kFailedRetryable reaches the retry path below.
     * The already adapted command batch is reused.
     */
  } else {
    if (reliable_channel_deliveries_.size() >=
        kMaxReliableRecords) {
      const bool published =
          publish_ack(
              reply_transport,
              legacy_packet,
              "FAILED",
              "reliable delivery cache full",
              true);

      return make_result(
          true,
          false,
          false,
          false,
          published,
          true,
          "reliable delivery cache full");
    }

    ReliableDeliveryRecord record;

    record.sequence = sequence;
    record.updated_at =
        std::chrono::steady_clock::now();
    record.state =
        ReliableDeliveryState::kPending;
    record.payload_hash = fingerprint;
    record.commands_cached = false;
    record.ack_status = "RUNNING";
    record.retryable = false;
    record.detail =
        "delivery is in progress";

    found =
        reliable_channel_deliveries_.emplace(
            key,
            std::move(record)).first;
  }

  auto& delivery =
      found->second;

  delivery.updated_at =
      std::chrono::steady_clock::now();

  try {
    /*
     * Adapt exactly once. Retries reuse the original
     * command batch so edge-triggered actions are preserved.
     */
    if (!delivery.commands_cached) {
      delivery.adapted_commands =
          legacy_.adapt(legacy_packet);
      delivery.commands_cached = true;
    }

    RoutedCommandResult result;

    if (delivery.adapted_commands.empty()) {
      result =
          make_result(
              true,
              true,
              false,
              false,
              false,
              false,
              "no mapped action");
    } else {
      result =
          route_command_batch(
              delivery.adapted_commands,
              reply_transport,
              kMaxCommandsPerG20Packet,
              "G20 channels adapted");
    }

    delivery.updated_at =
        std::chrono::steady_clock::now();

    if (result.control_succeeded &&
        result.execution_succeeded &&
        !result.execution_pending) {
      delivery.state =
          ReliableDeliveryState::kSucceeded;
      delivery.ack_status = "SUCCEEDED";
      delivery.retryable = false;
    } else if (result.control_succeeded &&
               result.execution_pending) {
      delivery.state =
          ReliableDeliveryState::kPending;
      delivery.ack_status = "RUNNING";
      delivery.retryable = false;
    } else if (result.control_succeeded) {
      delivery.state =
          ReliableDeliveryState::kAccepted;
      delivery.ack_status = "ACCEPTED";
      delivery.retryable = false;
    } else if (result.retryable) {
      delivery.state =
          ReliableDeliveryState::kFailedRetryable;
      delivery.ack_status = "FAILED";
      delivery.retryable = true;
    } else {
      delivery.state =
          ReliableDeliveryState::kFailedPermanent;
      delivery.ack_status = "FAILED";
      delivery.retryable = false;
    }

    delivery.detail = result.detail;

    const bool published =
        publish_ack(
            reply_transport,
            legacy_packet,
            delivery.ack_status,
            delivery.detail,
            delivery.retryable);

    result.ack_published = published;
    result.forwarded =
        result.control_succeeded;

    SNAPPY_GATEWAY_LOG_INFO(
        "g20.delivery",
        "event=received source_id=" +
            source_id +
            " session_id=" +
            session_id +
            " message_id=" +
            message_id +
            " seq=" +
            std::to_string(sequence) +
            " commands=" +
            std::to_string(
                delivery.adapted_commands.size()) +
            " status=" +
            delivery.ack_status +
            " forwarded=" +
            (result.forwarded ? "true" : "false") +
            " ack=" +
            (published ? "sent" : "failed"));

    return result;
  } catch (const std::exception& error) {
    delivery.updated_at =
        std::chrono::steady_clock::now();
    delivery.state =
        ReliableDeliveryState::kFailedRetryable;
    delivery.ack_status = "FAILED";
    delivery.retryable = true;
    delivery.detail =
        std::string(
            "reliable delivery exception: ") +
        error.what();

    const bool published =
        publish_ack(
            reply_transport,
            legacy_packet,
            delivery.ack_status,
            delivery.detail,
            true);

    return make_result(
        true,
        false,
        false,
        false,
        published,
        true,
        delivery.detail);
  }
}

RoutedCommandResult G20CommandRouter::tick(
    ITransport& reply_transport) {
  const auto commands =
      legacy_.tick();

  if (commands.empty()) {
    return make_result(
        false,
        false,
        false,
        false,
        false,
        false,
        "no periodic G20 command");
  }

  return route_command_batch(
      commands,
      reply_transport,
      kMaxCommandsPerPeriodicTick,
      "periodic G20 command handled");
}

RoutedCommandResult
G20CommandRouter::route_command_batch(
    const std::vector<nlohmann::json>& commands,
    ITransport& reply_transport,
    std::size_t max_commands,
    const std::string& handled_detail) {
  if (commands.empty()) {
    return make_result(
        true,
        true,
        false,
        false,
        false,
        false,
        "no mapped action");
  }

  const std::size_t limit =
      std::min(
          commands.size(),
          std::max<std::size_t>(
              1,
              max_commands));

  bool all_control_succeeded = true;
  bool all_execution_succeeded = true;
  bool any_execution_pending = false;
  bool all_ack_published = true;
  bool any_failure = false;
  bool all_failures_retryable = true;

  for (std::size_t index = 0;
       index < limit;
       ++index) {
    const auto result =
        route_command_json(
            commands[index],
            reply_transport);

    log_route_result(
        commands[index],
        result);

    all_control_succeeded =
        all_control_succeeded &&
        result.control_succeeded;

    all_execution_succeeded =
        all_execution_succeeded &&
        result.execution_succeeded;

    any_execution_pending =
        any_execution_pending ||
        result.execution_pending;

    all_ack_published =
        all_ack_published &&
        result.ack_published;

    if (!result.control_succeeded) {
      any_failure = true;

      all_failures_retryable =
          all_failures_retryable &&
          result.retryable;
    }
  }

  if (commands.size() > limit) {
    return make_result(
        true,
        false,
        false,
        false,
        all_ack_published,
        true,
        "G20 command batch throttled");
  }

  const bool control_succeeded =
      all_control_succeeded &&
      !any_failure;

  const bool execution_succeeded =
      control_succeeded &&
      all_execution_succeeded &&
      !any_execution_pending;

  const bool retryable =
      any_failure &&
      all_failures_retryable;

  return make_result(
      true,
      control_succeeded,
      execution_succeeded,
      any_execution_pending,
      all_ack_published,
      retryable,
      handled_detail);
}

RoutedCommandResult
G20CommandRouter::route_command_json(
    const nlohmann::json& command,
    ITransport& reply_transport) {
  const std::string action =
      command.value("action", "");

  const std::string command_text =
      "action=" +
      action +
      " command=" +
      command.dump();

  if (action == "control.heartbeat") {
    SNAPPY_GATEWAY_LOG_DEBUG(
        "g20.route",
        command_text);
  } else {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.route",
        command_text);
  }

  if (action == "video.start" ||
      action == "video.toggle") {
    if (video_adapter_ == nullptr ||
        !video_adapter_->available()) {
      const bool published =
          publish_ack(
              reply_transport,
              command,
              "FAILED",
              "G20 RTP video adapter unavailable",
              false);

      return make_result(
          true,
          false,
          false,
          false,
          published,
          false,
          "video adapter unavailable");
    }

    const auto state =
        video_adapter_->video_state();

    if (action == "video.toggle" &&
        state.running) {
      video_adapter_->stop();

      legacy_.sync_external_command(command);

      const bool published =
          publish_ack(
              reply_transport,
              command,
              "SUCCEEDED",
              "video stop queued",
              false);

      publish_video_state(
          reply_transport);

      return make_result(
          true,
          true,
          true,
          false,
          published,
          false,
          "video stop queued");
    }

    const bool start_queued =
        video_adapter_->start_g20_stream();

    bool switch_queued = true;

    const int requested_camera =
        int_param(
            command,
            "active_camera",
            1);

    if (start_queued &&
        requested_camera != 1) {
      switch_queued =
          video_adapter_->switch_g20_camera(
              requested_camera);
    }

    const bool accepted =
        start_queued &&
        switch_queued;

    const std::string detail =
        accepted
            ? "video command queued"
            : "video command queue failed";

    const bool published =
        publish_ack(
            reply_transport,
            command,
            accepted
                ? "SUCCEEDED"
                : "FAILED",
            detail,
            false);

    if (accepted) {
      legacy_.sync_external_command(command);
    }

    publish_video_state(
        reply_transport);

    return make_result(
        true,
        accepted,
        accepted,
        false,
        published,
        false,
        detail);
  }

  if (action == "video.stop") {
    if (video_adapter_ != nullptr) {
      video_adapter_->stop();
    }

    legacy_.sync_external_command(command);

    const bool published =
        publish_ack(
            reply_transport,
            command,
            "SUCCEEDED",
            "video stop queued",
            false);

    publish_video_state(
        reply_transport);

    return make_result(
        true,
        true,
        true,
        false,
        published,
        false,
        "video stop queued");
  }

  if (action == "video.switch_camera") {
    if (video_adapter_ == nullptr ||
        !video_adapter_->available()) {
      const bool published =
          publish_ack(
              reply_transport,
              command,
              "FAILED",
              "G20 RTP video adapter unavailable",
              false);

      return make_result(
          true,
          false,
          false,
          false,
          published,
          false,
          "video adapter unavailable");
    }

    const int requested_camera =
        int_param(
            command,
            "active_camera",
            1);

    const bool queued =
        video_adapter_->switch_g20_camera(
            requested_camera);

    const std::string detail =
        queued
            ? "camera switch queued"
            : "camera switch queue failed";

    if (queued) {
      legacy_.sync_external_command(command);
    }

    const bool published =
        publish_ack(
            reply_transport,
            command,
            queued
                ? "SUCCEEDED"
                : "FAILED",
            detail,
            false);

    publish_video_state(
        reply_transport);

    return make_result(
        true,
        queued,
        queued,
        false,
        published,
        false,
        detail);
  }

  if (action == "telemetry.enable" ||
      action == "telemetry.disable") {
    const bool enabled =
        action == "telemetry.enable";

    if (telemetry_control_) {
      telemetry_control_(enabled);
    }

    const std::string detail =
        enabled
            ? "telemetry enabled"
            : "telemetry disabled";

    const bool published =
        publish_ack(
            reply_transport,
            command,
            "SUCCEEDED",
            detail,
            false);

    return make_result(
        true,
        true,
        true,
        false,
        published,
        false,
        detail);
  }

  const auto ack =
      control_channel_.request(
          command.dump());

  if (!ack.has_value()) {
    const std::string detail =
        "control IPC unavailable";

    const bool published =
        publish_ack(
            reply_transport,
            command,
            "FAILED",
            detail,
            true);

    return make_result(
        true,
        false,
        false,
        false,
        published,
        true,
        detail);
  }

  const std::string status =
      ack_status(*ack);

  const bool control_succeeded =
      ack_succeeded(*ack);

  const bool execution_succeeded =
      ack_execution_succeeded(*ack);

  const bool execution_pending =
      ack_execution_pending(*ack);

  const bool retryable =
      status == "FAILED" &&
      ack_retryable(*ack);

  const std::string detail =
      ack_detail(*ack);

  const std::string control_text =
      "action=" +
      action +
      " status=" +
      status +
      " ack=" +
      *ack;

  if (action == "control.heartbeat") {
    SNAPPY_GATEWAY_LOG_DEBUG(
        "g20.control",
        control_text);
  } else if (control_succeeded) {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.control",
        control_text);
  } else {
    SNAPPY_GATEWAY_LOG_WARN(
        "g20.control",
        control_text);
  }

  if (control_succeeded) {
    legacy_.sync_external_command(command);
  }

  const bool published =
      reply_transport.publish(
          PublishedMessage{
              "snappy/v1/robots/" +
                  robot_id_ +
                  "/up/ack",
              *ack,
              1,
              false,
          });

  return make_result(
      true,
      control_succeeded,
      execution_succeeded,
      execution_pending,
      published,
      retryable,
      published
          ? detail
          : "ACK publish failed");
}

}  // namespace snappy::gateway::g20