#include "snappy/gateway/g20/g20_packet_codec.hpp"
/**
 * @file g20_packet_codec.cpp
 * @brief G20 报文编解码实现
 *
 * @details
 * - 命令解码、应答编码、状态与遥测编码。
 * - 状态与遥测编码后不得超过 UDP 单报文上限。
 * - compact_control_snapshot 把 control 快照压缩到上限以内，字段语义不变。
 */
 
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway::g20 {

namespace {

// 把样本逻辑名映射成紧凑短键，未登记的名字原样保留。
struct SampleNameAlias {
  const char* logical;
  const char* wire;
};

constexpr SampleNameAlias kSampleNameAliases[] = {
    {"anemometer_left_speed", "anl_spd"},
    {"anemometer_right_speed", "anr_spd"},
    {"lidar_body_front_distance", "lbf_dist"},
    {"lidar_body_rear_distance", "lbr_dist"},
    {"lidar_brush_left_distance", "lbl_dist"},
    {"lidar_brush_left_distance_temperature", "lbl_temp"},
    {"lidar_brush_right_distance", "lbrr_dist"},
    {"lidar_brush_right_distance_temperature", "lbrr_temp"},
    {"power_24v_current", "p24_cur"},
    {"power_24v_operation_enabled", "p24_en"},
    {"power_24v_power", "p24_pwr"},
    {"power_24v_temperature", "p24_temp"},
    {"power_24v_voltage", "p24_volt"},
    {"power_48v_current", "p48_cur"},
    {"power_48v_operation_enabled", "p48_en"},
    {"power_48v_power", "p48_pwr"},
    {"power_48v_temperature", "p48_temp"},
    {"power_48v_voltage", "p48_volt"},
    {"temperature_cabinet_humidity", "cab_hum"},
    {"temperature_cabinet_temperature", "cab_temp"},
    {"wit_motion_accel_x", "imu_ax"},
    {"wit_motion_accel_y", "imu_ay"},
    {"wit_motion_accel_z", "imu_az"},
    {"wit_motion_gyro_x", "imu_gx"},
    {"wit_motion_gyro_y", "imu_gy"},
    {"wit_motion_gyro_z", "imu_gz"},
    {"wit_motion_pitch", "imu_pitch"},
    {"wit_motion_roll", "imu_roll"},
    {"wit_motion_temperature", "imu_temp"},
    {"wit_motion_yaw", "imu_yaw"},
};

// 返回样本的紧凑短键。
std::string short_sample_name(const std::string& logical) {
  for (const auto& entry : kSampleNameAliases) {
    if (logical == entry.logical) {
      return entry.wire;
    }
  }
  return logical;
}

// 返回质量枚举的紧凑编码。
std::string sample_quality_code(const std::string& wire) {
  if (wire == "GOOD") {
    return "G";
  }
  if (wire == "STALE") {
    return "S";
  }
  if (wire == "INVALID") {
    return "I";
  }
  return "U";
}

// 把质量字段压成单字符；已是单字符或非字符串时原样保留。
nlohmann::json compact_quality_field(const nlohmann::json& value) {
  if (!value.is_string()) {
    return value;
  }
  return sample_quality_code(value.get<std::string>());
}

// 把浮点值收敛到 6 位小数；整数值写成整数，非有限值写成 null。
nlohmann::json normalize_sample_value(const nlohmann::json& value) {
  if (!value.is_number_float()) {
    return value;
  }
  const double raw = value.get<double>();
  if (!std::isfinite(raw)) {
    return nullptr;
  }
  const double rounded = std::round(raw * 1e6) / 1e6;
  if (rounded == std::trunc(rounded) && std::fabs(rounded) < 1e15) {
    return static_cast<long long>(rounded);
  }
  return rounded;
}

// 把样本数组压成 {n,v,q} 三元组，只保留界面实际读取的字段。
nlohmann::json compact_sample_array(const nlohmann::json& samples) {
  if (!samples.is_array()) {
    return samples;
  }

  nlohmann::json out = nlohmann::json::array();
  for (const auto& item : samples) {
    if (!item.is_object()) {
      out.push_back(item);
      continue;
    }

    nlohmann::json compact;
    compact["n"] = short_sample_name(item.value("name", item.value("logical_name", std::string())));
    if (item.contains("v")) {
      compact["v"] = normalize_sample_value(item.at("v"));
    } else if (item.contains("value")) {
      compact["v"] = normalize_sample_value(item.at("value"));
    } else {
      compact["v"] = nullptr;
    }
    compact["q"] = compact_quality_field(
        item.contains("q") ? item.at("q")
                           : item.value("quality", nlohmann::json("UNKNOWN")));
    out.push_back(std::move(compact));
  }
  return out;
}

// 判断顶层键是否需要保留。
bool kept_top_level_key(const std::string& key) {
  static const char* kKept[] = {
      "schema_version", "kind",      "robot_id",  "state",
      "ready_for_arm",  "g20_online", "lease_held", "detail",
      "control_epoch",  "lease_id",   "lease_owner", "safety_reason",
      "armed",          "control_started", "partial_start",
      "hardware_ready_for_arm", "updated_at_ms", "timestamp_ms",
      "watchdog",       "g20_udp",    "actuators",  "video",
      "samples",
  };
  for (const char* candidate : kKept) {
    if (key == candidate) {
      return true;
    }
  }
  return false;
}

// 只保留界面读取的三个视频字段。
nlohmann::json compact_video_block(const nlohmann::json& video) {
  if (!video.is_object()) {
    return video;
  }
  nlohmann::json out = nlohmann::json::object();
  for (const char* key : {"target_address", "camera1_port", "camera2_port"}) {
    if (video.contains(key)) {
      out[key] = video.at(key);
    }
  }
  return out;
}

}  // namespace

std::uint64_t wall_time_ms() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

CommandDecodeResult decode_command(const std::string& payload, const std::string& expected_robot_id) {
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
    if (!expected_robot_id.empty() && root.value("robot_id", "") != expected_robot_id) {
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
  ack["message_id"] = command.value("message_id", "");
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
    ack["source_id"] = command.at("source_id");
  }

  if (
      command.contains("session_id") && command.at("session_id").is_string()
  ) {
    ack["session_id"] = command.at("session_id");
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

// 生成遥测报文：只发界面读取的字段，并保证编码后不超过 UDP 上限。
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
    item["n"] = short_sample_name(sample.logical_name);
    if (sample.value) {
      item["v"] = normalize_sample_value(nlohmann::json(*sample.value));
    } else {
      item["v"] = nullptr;
    }
    item["q"] = sample_quality_code(sample_quality_wire(sample.quality));
    root["samples"].push_back(std::move(item));
  }

  const std::size_t total = root["samples"].size();
  for (std::size_t keep = total + 1; keep > 0; --keep) {
    const std::size_t kept = keep - 1;
    nlohmann::json candidate = root;
    candidate["samples"] = nlohmann::json::array();
    for (std::size_t i = 0; i < kept; ++i) {
      candidate["samples"].push_back(root["samples"].at(i));
    }
    if (kept < total) {
      candidate["samples_dropped"] = total - kept;
    }
    const std::string encoded = candidate.dump();
    if (encoded.size() <= kUdpPayloadLimit) {
      return encoded;
    }
  }

  return root.dump();
}

// 把 control 快照压到 UDP 上限以内，字段与取值语义保持不变。
std::string compact_control_snapshot(const std::string& payload) {
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(payload);
  } catch (const std::exception&) {
    return payload;
  }

  if (!root.is_object()) {
    return payload;
  }

  const bool is_telemetry = root.value("kind", "") == "telemetry";

  nlohmann::json compact = nlohmann::json::object();
  for (auto it = root.begin(); it != root.end(); ++it) {
    const std::string key = it.key();
    if (!kept_top_level_key(key)) {
      continue;
    }
    if (is_telemetry && (key == "actuators" || key == "timestamp_ms")) {
      continue;
    }
    if (key == "samples") {
      compact["samples"] = compact_sample_array(it.value());
    } else if (key == "video") {
      compact["video"] = compact_video_block(it.value());
    } else {
      compact[key] = it.value();
    }
  }

  if (compact.contains("samples") && compact["samples"].is_array()) {
    const nlohmann::json full = compact["samples"];
    const std::size_t total = full.size();
    for (std::size_t keep = total + 1; keep > 0; --keep) {
      const std::size_t kept = keep - 1;
      nlohmann::json candidate = compact;
      candidate["samples"] = nlohmann::json::array();
      for (std::size_t i = 0; i < kept; ++i) {
        candidate["samples"].push_back(full.at(i));
      }
      if (kept < total) {
        candidate["samples_dropped"] = total - kept;
      }
      const std::string encoded = candidate.dump();
      if (encoded.size() <= kUdpPayloadLimit) {
        return encoded;
      }
    }
    compact["samples"] = full;
  }

  return compact.dump();
}

}  // namespace snappy::gateway::g20
