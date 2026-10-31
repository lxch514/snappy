#pragma once
/**
 * @file g20_packet_codec.hpp
 * @brief G20 报文编解码声明
 *
 * @details
 * - 命令解码、应答编码、状态与遥测编码。
 * - 状态与遥测编码后不得超过 UDP 单报文上限。
 * - compact_control_snapshot 把 control 快照压缩到上限以内，字段语义不变。
 */
 
#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "snappy/control/model.hpp"
#include "snappy/drivers/interfaces.hpp"

namespace snappy::gateway::g20 {

struct CommandDecodeResult {
  bool ok{false};
  nlohmann::json command;
  std::string detail;
};

[[nodiscard]] std::uint64_t wall_time_ms();

[[nodiscard]] CommandDecodeResult decode_command(
    const std::string& payload,
    const std::string& expected_robot_id);

[[nodiscard]] std::string encode_ack(
    const std::string& robot_id,
    const nlohmann::json& command,
    const std::string& status,
    const std::string& detail,
    bool retryable = false);

[[nodiscard]] std::string encode_status(const std::string& robot_id,
                                        control::ControlState state,
                                        bool ready_for_arm,
                                        bool g20_online,
                                        bool lease_held,
                                        const std::string& detail);

[[nodiscard]] std::string encode_telemetry(
    const std::string& robot_id,
    const std::vector<drivers::SensorSample>& samples);

[[nodiscard]] std::string compact_control_snapshot(const std::string& payload);

[[nodiscard]] const char* control_state_wire(control::ControlState state);
[[nodiscard]] const char* sample_quality_wire(drivers::SampleQuality quality);

}  // namespace snappy::gateway::g20