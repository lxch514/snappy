#pragma once

#include "snappy/gateway/g20/g20_button_mapper.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace snappy::gateway::g20 {

class G20LegacyPacketAdapter {
 public:
  G20LegacyPacketAdapter() = default;

  [[nodiscard]] bool accepts(const nlohmann::json& packet) const;

  std::vector<nlohmann::json> adapt(const nlohmann::json& packet);
  std::vector<nlohmann::json> tick();

  void sync_external_command(const nlohmann::json& command);

 private:
  struct LegacyIdentity {
    std::string robot_id{"snappy-001"};
    std::string source_id{"g20-remote"};
    std::string session_id{"default"};
  };

  G20ButtonMapper mapper_;
  LegacyIdentity identity_;
  std::uint64_t generated_seq_{0};
  std::vector<int> previous_channels_;
  std::string previous_session_id_;

  nlohmann::json to_command_json(const LegacyRemoteCommand& command);

  void update_identity(const nlohmann::json& packet);
  std::uint64_t next_seq();

  static std::vector<int> read_channels(const nlohmann::json& packet);
};

}  // namespace snappy::gateway::g20
