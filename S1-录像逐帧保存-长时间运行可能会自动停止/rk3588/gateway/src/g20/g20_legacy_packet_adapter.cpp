/**
 * @file g20_legacy_packet_adapter.cpp
 * @brief G20 原始舵量解析、旧按键状态机适配和输入日志实现
 *
 * @details
 * - 保持原有 G20 通道到命令的映射逻辑不变。
 * - 记录原始通道、映射动作和外部状态同步，便于按 .log 回放现场操作。
 */

#include "snappy/gateway/g20/g20_legacy_packet_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <sstream>
#include <string>
#include <utility>

#include "snappy/gateway/diagnostics/gateway_log.hpp"
#include "snappy/gateway/g20/g20_packet_codec.hpp"

namespace snappy::gateway::g20 {
namespace {

std::string json_string_or(const nlohmann::json& object,
                           const char* key,
                           const std::string& fallback) {
  if (!object.is_object() ||
      !object.contains(key) ||
      !object.at(key).is_string()) {
    return fallback;
  }

  const auto value = object.at(key).get<std::string>();
  return value.empty() ? fallback : value;
}

std::uint64_t json_u64_or(const nlohmann::json& object,
                          const char* key,
                          std::uint64_t fallback) {
  if (!object.is_object() || !object.contains(key)) {
    return fallback;
  }

  try {
    const auto& value = object.at(key);
    if (value.is_number_unsigned()) {
      return value.get<std::uint64_t>();
    }
    if (value.is_number_integer()) {
      const auto parsed = value.get<long long>();
      return parsed < 0 ? fallback : static_cast<std::uint64_t>(parsed);
    }
    if (value.is_string()) {
      return static_cast<std::uint64_t>(
          std::stoull(value.get<std::string>()));
    }
  } catch (...) {
  }

  return fallback;
}

std::string json_value_to_string(const nlohmann::json& value) {
  if (value.is_string()) {
    return value.get<std::string>();
  }

  if (value.is_boolean()) {
    return value.get<bool>() ? "true" : "false";
  }

  if (value.is_number_integer()) {
    return std::to_string(value.get<long long>());
  }

  if (value.is_number_unsigned()) {
    return std::to_string(value.get<unsigned long long>());
  }

  if (value.is_number_float()) {
    std::ostringstream oss;
    oss << value.get<double>();
    return oss.str();
  }

  return {};
}

std::map<std::string, std::string> params_to_map(const nlohmann::json& command) {
  std::map<std::string, std::string> params;

  if (!command.is_object() ||
      !command.contains("params") ||
      !command.at("params").is_object()) {
    return params;
  }

  for (auto it = command.at("params").begin();
       it != command.at("params").end();
       ++it) {
    params[it.key()] = json_value_to_string(it.value());
  }

  return params;
}

std::string channels_to_string(const std::vector<int>& channels) {
  std::ostringstream out;
  out << '[';

  for (std::size_t index = 0;
       index < channels.size();
       ++index) {
    if (index != 0) {
      out << ',';
    }
    out << channels[index];
  }

  out << ']';
  return out.str();
}

std::string channel_name(std::size_t index) {
  switch (index) {
    case 0:
      return "right_stick_x";
    case 1:
      return "right_stick_y";
    case 2:
      return "left_stick_y";
    case 3:
      return "left_stick_x";
    case 4:
      return "GS";
    case 6:
      return "L2";
    case 7:
      return "camera";
    case 10:
      return "L1";
    case 11:
      return "SPAY";
    case 12:
      return "pause";
    case 13:
      return "wheel";
    case 14:
      return "R1";
    case 15:
      return "R2";
    default:
      return "CH" + std::to_string(index + 1);
  }
}

std::string changed_channels_to_string(
    const std::vector<int>& previous,
    const std::vector<int>& current) {
  const std::size_t count =
      std::max(previous.size(), current.size());
  std::ostringstream out;
  bool changed = false;

  for (std::size_t index = 0; index < count; ++index) {
    const bool has_previous = index < previous.size();
    const bool has_current = index < current.size();
    if (has_previous && has_current &&
        previous[index] == current[index]) {
      continue;
    }

    if (changed) {
      out << ',';
    }
    changed = true;
    out << channel_name(index) << "(ch" << (index + 1) << "):";
    if (has_previous) {
      out << previous[index];
    } else {
      out << '-';
    }
    out << "->";
    if (has_current) {
      out << current[index];
    } else {
      out << '-';
    }
  }

  return changed ? out.str() : "none";
}

std::string params_to_string(
    const std::map<std::string, std::string>& params) {
  std::ostringstream out;
  out << '{';

  std::size_t index = 0;
  for (const auto& [key, value] : params) {
    if (index++ != 0) {
      out << ',';
    }
    out << key << '=' << value;
  }

  out << '}';
  return out.str();
}

std::string input_name_for_action(const std::string& action) {
  if (action == "brush.forward" ||
      action == "brush.stop") {
    return "L1/L2";
  }
  if (action == "brush.reverse") {
    return "L2";
  }
  if (action == "fan.both_set" ||
      action == "fan.stop") {
    return "right_stick";
  }
  if (action == "climber.ascend") {
    return "R1/wheel";
  }
  if (action == "climber.descend") {
    return "R2/wheel";
  }
  if (action == "climber.stop") {
    return "R1/R2";
  }
  if (action == "video.start" ||
      action == "video.stop") {
    return "camera";
  }
  if (action == "video.switch_camera") {
    return "SPAY";
  }
  if (action == "control.safe_hold") {
    return "pause";
  }
  if (action.find("push") != std::string::npos) {
    return "GS/left_stick";
  }
  if (action == "control.acquire" ||
      action == "control.arm") {
    return "initial_packet";
  }
  return "external";
}

}  // namespace

bool G20LegacyPacketAdapter::accepts(const nlohmann::json& packet) const {
  const std::string kind = json_string_or(packet, "kind", "");
  const std::string type = json_string_or(packet, "type", "");

  return kind == "g20_channels" ||
         type == "g20_channels" ||
         type == "buttons" ||
         type == "rc_buttons";
}

void G20LegacyPacketAdapter::update_identity(const nlohmann::json& packet) {
  identity_.robot_id = json_string_or(packet, "robot_id", identity_.robot_id);

  identity_.source_id = json_string_or(
      packet,
      "source_id",
      json_string_or(packet, "source", identity_.source_id));

  identity_.session_id = json_string_or(
      packet,
      "session_id",
      identity_.session_id);

  const auto packet_seq = json_u64_or(
      packet,
      "seq",
      json_u64_or(packet, "sequence", 0));

  if (packet_seq > generated_seq_) {
    generated_seq_ = packet_seq;
  }
}

std::uint64_t G20LegacyPacketAdapter::next_seq() {
  generated_seq_ += 1;
  return generated_seq_;
}

std::vector<int> G20LegacyPacketAdapter::read_channels(
    const nlohmann::json& packet) {
  const nlohmann::json* source = nullptr;

  if (packet.contains("channels") &&
      packet.at("channels").is_array()) {
    source = &packet.at("channels");
  } else if (packet.contains("buttons") &&
             packet.at("buttons").is_array()) {
    source = &packet.at("buttons");
  }

  if (source == nullptr) {
    return {};
  }

  std::vector<int> channels;
  channels.reserve(source->size());

  for (const auto& value : *source) {
    if (value.is_number_integer()) {
      channels.push_back(value.get<int>());
    } else if (value.is_number()) {
      channels.push_back(static_cast<int>(value.get<double>()));
    } else {
      channels.push_back(1500);
    }
  }

  return channels;
}

nlohmann::json G20LegacyPacketAdapter::to_command_json(
    const LegacyRemoteCommand& command) {
  nlohmann::json params = nlohmann::json::object();

  for (const auto& [key, value] : command.params) {
    params[key] = value;
  }

  const auto seq = next_seq();

  return {
      {"schema_version", 1},
      {"kind", "command"},
      {"robot_id", identity_.robot_id},
      {"link_id", "g20_udp"},
      {"source_id", identity_.source_id},
      {"source", identity_.source_id},
      {"session_id", identity_.session_id},
      {"message_id", command.action + "-" + std::to_string(seq)},
      {"seq", seq},
      {"issued_at_ms", wall_time_ms()},
      {"ttl_ms", 250},
      {"control_epoch", 0},
      {"action", command.action},
      {"params", params},
  };
}

std::vector<nlohmann::json> G20LegacyPacketAdapter::adapt(
    const nlohmann::json& packet) {
  if (!accepts(packet)) {
    return {};
  }

  update_identity(packet);

  const auto channels = read_channels(packet);
  if (channels.empty()) {
    SNAPPY_GATEWAY_LOG_WARN(
        "g20.input",
        "G20 packet accepted but contains no channels");
    return {};
  }

  const bool first_input =
      previous_channels_.empty() ||
      previous_session_id_ != identity_.session_id;
  const bool channels_changed =
      first_input ||
      previous_channels_.size() != channels.size() ||
      !std::equal(
          previous_channels_.begin(),
          previous_channels_.end(),
          channels.begin());
  const std::string changed_channels =
      first_input
          ? "initial"
          : changed_channels_to_string(
                previous_channels_,
                channels);
  const std::string input_log =
      "received robot_id=" + identity_.robot_id +
      " source_id=" + identity_.source_id +
      " session_id=" + identity_.session_id +
      " changed=" + (channels_changed ? "true" : "false") +
      " changed_channels=" + changed_channels +
      " channels=" + channels_to_string(channels);

  SNAPPY_GATEWAY_LOG_DEBUG(
      "g20.input",
      input_log);

  if (channels_changed) {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.input",
        "state=" +
            std::string(first_input ? "initial" : "changed") +
            " changed_channels=" + changed_channels +
            " channels=" + channels_to_string(channels));
  }

  /* ----------------------------------------------------------
   * 下面仍然使用原有 mapper，不用日志状态参与动作判定。
   * ---------------------------------------------------------- */

  const auto mapped =
      mapper_.update(channels, std::chrono::steady_clock::now());

  previous_channels_ = channels;
  previous_session_id_ = identity_.session_id;

  std::vector<nlohmann::json> commands;
  commands.reserve(mapped.size());

  for (const auto& command : mapped) {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.mapping",
        "changed_channels=" + changed_channels +
            " channels=" + channels_to_string(channels) +
            " input=" + input_name_for_action(command.action) +
            " action=" + command.action +
            " params=" + params_to_string(command.params));

    commands.push_back(to_command_json(command));
  }
  
  if (channels_changed && mapped.empty()) {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.mapping",
        "result=no_action changed_channels=" +
            changed_channels +
            " channels=" + channels_to_string(channels));
  }

  return commands;
}

std::vector<nlohmann::json> G20LegacyPacketAdapter::tick() {
  const auto mapped =
      mapper_.tick(std::chrono::steady_clock::now());

  std::vector<nlohmann::json> commands;
  commands.reserve(mapped.size());

    for (const auto& command : mapped) {
    /*
     * 保留风机、爬绳器和推杆长按调节产生的周期动作；
     * 心跳不属于操作动作，不写入日志。
     */
    if (command.action != "control.heartbeat") {
      SNAPPY_GATEWAY_LOG_INFO(
          "g20.mapping",
          "periodic action=" + command.action +
              " params=" + params_to_string(command.params));
    }

    commands.push_back(to_command_json(command));
  }

  return commands;
}

void G20LegacyPacketAdapter::sync_external_command(
    const nlohmann::json& command) {
  update_identity(command);

  if (!command.is_object() ||
      !command.contains("action") ||
      !command.at("action").is_string()) {
    return;
  }

    const std::string action =
      command.at("action").get<std::string>();

  mapper_.syncExternalCommand(
      action,
      params_to_map(command));

  /*
   * 心跳是 250 ms 周期内部保活，不写入操作日志。
   * 其他外部命令仍完整记录，便于确认控制端响应。
   */
  if (action != "control.heartbeat") {
    SNAPPY_GATEWAY_LOG_INFO(
        "g20.sync",
        "external action=" + action +
            " command=" + command.dump());
  }
}

}  // namespace snappy::gateway::g20
