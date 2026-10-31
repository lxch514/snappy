#include "snappy/gateway/g20/g20_packet_codec.hpp"

#include <chrono>
#include <exception>

namespace snappy::gateway::g20 {

std::uint64_t wall_time_ms() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

CommandDecodeResult decode_command(const std::string& payload,
                                   const std::string& expected_robot_id) {
  try {
    auto root = nlohmann::json::parse(payload);
    if (!root.is_object()) {
      return {false, {}, "payload must be a JSON object"};
    }
    if (root.value("schema_version", 0) != 1) {
      return {false, {}, "schema_version must be 1"};
    }
    if (root.value("kind", "") != "command") {
      return {false, {}, "kind must be command"};
    }
    if (!expected_robot_id.empty() &&
        root.value("robot_id", "") != expected_robot_id) {
      return {false, {}, "robot_id mismatch"};
    }
    if (root.value("source_id", "").empty()) {
      return {false, {}, "source_id is required"};
    }
    if (root.value("message_id", "").empty()) {
      return {false, {}, "message_id is required"};
    }
    if (root.value("action", "").empty()) {
      return {false, {}, "action is required"};
    }
    if (!root.contains("params") || root["params"].is_null()) {
      root["params"] = nlohmann::json::object();
    }
    if (!root["params"].is_object()) {
      return {false, {}, "params must be an object"};
    }

    root["link_id"] = "g20_udp";
    return {true, std::move(root), {}};
  } catch (const std::exception& e) {
    return {false, {}, std::string("invalid JSON: ") + e.what()};
  }
}

std::string encode_ack(
    const std::string& robot_id,
    const nlohmann::json& command,
    const std::string& status,
    const std::string& detail,
    bool retryable) {
  nlohmann::json ack;

  ack["schema_version"] = 1;
  ack["kind"] = "ack";
  ack["robot_id"] = robot_id;
  ack["message_id"] =
      command.value("message_id", "");
  ack["seq"] =
      command.value(
          "seq",
          command.value("sequence", 0));
  ack["status"] = status;
  ack["retryable"] = retryable;
  ack["detail"] = detail;
  ack["received_at_ms"] = wall_time_ms();

  if (
      command.contains("source_id") &&
      command.at("source_id").is_string()
  ) {
    ack["source_id"] =
        command.at("source_id");
  }

  if (
      command.contains("session_id") &&
      command.at("session_id").is_string()
  ) {
    ack["session_id"] =
        command.at("session_id");
  }

  return ack.dump();
}

const char* control_state_wire(control::ControlState state) {
  switch (state) {
    case control::ControlState::kBoot:
      return "BOOT";
    case control::ControlState::kSelfTest:
      return "SELF_TEST";
    case control::ControlState::kDisarmed:
      return "DISARMED";
    case control::ControlState::kAcquired:
      return "ACQUIRED";
    case control::ControlState::kArmedIdle:
      return "ARMED_IDLE";
    case control::ControlState::kActive:
      return "ACTIVE";
    case control::ControlState::kSafeHold:
      return "SAFE_HOLD";
    case control::ControlState::kFaultLatched:
      return "FAULT_LATCHED";
    case control::ControlState::kEstopLatched:
      return "ESTOP_LATCHED";
  }
  return "DISARMED";
}

const char* sample_quality_wire(drivers::SampleQuality quality) {
  switch (quality) {
    case drivers::SampleQuality::kGood:
      return "GOOD";
    case drivers::SampleQuality::kStale:
      return "STALE";
    case drivers::SampleQuality::kInvalid:
      return "INVALID";
    case drivers::SampleQuality::kUnknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

std::string encode_status(const std::string& robot_id,
                          control::ControlState state,
                          bool ready_for_arm,
                          bool g20_online,
                          bool lease_held,
                          const std::string& detail) {
  nlohmann::json root;
  root["schema_version"] = 1;
  root["kind"] = "status";
  root["robot_id"] = robot_id;
  root["state"] = control_state_wire(state);
  root["ready_for_arm"] = ready_for_arm;
  root["g20_online"] = g20_online;
  root["lease_held"] = lease_held;
  root["detail"] = detail;
  root["updated_at_ms"] = wall_time_ms();
  return root.dump();
}

std::string encode_telemetry(const std::string& robot_id,
                             const std::vector<drivers::SensorSample>& samples) {
  nlohmann::json root;
  root["schema_version"] = 1;
  root["kind"] = "telemetry";
  root["robot_id"] = robot_id;
  root["updated_at_ms"] = wall_time_ms();
  root["samples"] = nlohmann::json::array();

  for (const auto& sample : samples) {
    nlohmann::json item;
    item["name"] = sample.logical_name;
    if (sample.value) {
      item["value"] = *sample.value;
    } else {
      item["value"] = nullptr;
    }
    item["unit"] = sample.unit;
    item["quality"] = sample_quality_wire(sample.quality);
    item["timestamp_ms"] = sample.timestamp_ms;
    item["age_ms"] = sample.age_ms;
    root["samples"].push_back(std::move(item));
  }

  return root.dump();
}

}  // namespace snappy::gateway::g20