#pragma once

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

[[nodiscard]] const char* control_state_wire(control::ControlState state);
[[nodiscard]] const char* sample_quality_wire(drivers::SampleQuality quality);

}  // namespace snappy::gateway::g20