#include "snappy/gateway/diagnostics/gateway_log.hpp"

namespace snappy::gateway::diagnostics {

void configure_from_environment(const std::string& process_name) {
  ::snappy::drivers::diagnostics::configure_from_environment(process_name);
}

void log(LogLevel level,
         const std::string& component,
         const std::string& message) {
  ::snappy::drivers::diagnostics::log(
      level,
      "gateway." + component,
      message);
}

void debug(const std::string& component, const std::string& message) {
  ::snappy::gateway::diagnostics::log(
      LogLevel::kDebug,
      component,
      message);
}

void info(const std::string& component, const std::string& message) {
  ::snappy::gateway::diagnostics::log(
      LogLevel::kInfo,
      component,
      message);
}

void warn(const std::string& component, const std::string& message) {
  ::snappy::gateway::diagnostics::log(
      LogLevel::kWarn,
      component,
      message);
}

void error(const std::string& component, const std::string& message) {
  ::snappy::gateway::diagnostics::log(
      LogLevel::kError,
      component,
      message);
}

bool packet_trace_enabled() noexcept {
  return ::snappy::drivers::diagnostics::packet_trace_enabled();
}

void trace_packet(const std::string& bus,
                  const std::string& direction,
                  const std::string& endpoint,
                  const std::vector<std::uint8_t>& bytes,
                  const std::string& detail) {
  ::snappy::drivers::diagnostics::trace_packet(
      bus,
      direction,
      endpoint,
      bytes,
      detail);
}

}  // namespace snappy::gateway::diagnostics