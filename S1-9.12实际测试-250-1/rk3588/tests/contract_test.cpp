#include <fstream>
#include <initializer_list>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace {

nlohmann::json load(const std::string& file) {
  std::ifstream input(std::string(SNAPPY_SOURCE_DIR) +
                      "/contracts/v1/test-vectors/" + file);
  if (!input) {
    throw std::runtime_error("cannot open test vector " + file);
  }
  return nlohmann::json::parse(input);
}

bool has_all(const nlohmann::json& value,
             const std::initializer_list<const char*> fields) {
  if (!value.is_object()) {
    return false;
  }
  for (const auto* field : fields) {
    if (!value.contains(field)) {
      return false;
    }
  }
  return true;
}

bool valid_link(const nlohmann::json& value) {
  return value.is_string() && (value == "g20_udp" || value == "ec20_mqtt");
}

bool valid_command(const nlohmann::json& value) {
  if (!has_all(value,
               {"schema_version", "kind", "robot_id", "link_id", "source_id",
                "boot_id", "session_id", "message_id", "seq", "issued_at_ms",
                "ttl_ms", "control_epoch", "lease_id", "action", "params"})) {
    return false;
  }
  return value.at("schema_version") == 1 && value.at("kind") == "command" &&
         valid_link(value.at("link_id")) && value.at("message_id").is_string() &&
         !value.at("message_id").get<std::string>().empty() &&
         value.at("ttl_ms").is_number_unsigned() &&
         value.at("ttl_ms").get<std::uint64_t>() >= 1 &&
         value.at("ttl_ms").get<std::uint64_t>() <= 1000 &&
         value.at("params").is_object();
}

bool valid_ack(const nlohmann::json& value) {
  static const std::set<std::string> statuses{
      "ACCEPTED", "RUNNING", "SUCCEEDED", "FAILED", "ABORTED", "TIMED_OUT"};
  if (!has_all(value,
               {"schema_version", "kind", "robot_id", "link_id", "boot_id",
                "message_id", "ack_id", "control_epoch", "status", "code", "detail"})) {
    return false;
  }
  return value.at("schema_version") == 1 && value.at("kind") == "ack" &&
         valid_link(value.at("link_id")) && value.at("message_id").is_string() &&
         statuses.count(value.at("status").get<std::string>()) == 1;
}

bool valid_state(const nlohmann::json& value) {
  if (!has_all(value,
               {"schema_version", "kind", "robot_id", "boot_id", "seq",
                "emitted_at_ms", "desired", "actual", "quality", "age_ms",
                "link_health"})) {
    return false;
  }
  const auto& health = value.at("link_health");
  return value.at("schema_version") == 1 && value.at("kind") == "state" &&
         value.at("desired").is_object() && value.at("actual").is_object() &&
         has_all(health, {"g20_udp", "ec20_mqtt"});
}

}  // namespace

int main() {
  try {
    if (!valid_command(load("command.valid.json"))) {
      std::cerr << "valid command rejected\n";
      return 1;
    }
    if (valid_command(load("command.invalid.missing-message-id.json"))) {
      std::cerr << "invalid command accepted\n";
      return 1;
    }
    if (!valid_ack(load("ack.valid.json"))) {
      std::cerr << "valid ACK rejected\n";
      return 1;
    }
    if (!valid_state(load("state.valid.json"))) {
      std::cerr << "valid state rejected\n";
      return 1;
    }
    std::cout << "C++ golden contract vectors passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
