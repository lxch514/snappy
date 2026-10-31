#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy::gateway::diagnostics {

using LogLevel = ::snappy::drivers::diagnostics::LogLevel;

void configure_from_environment(const std::string& process_name = "snappy-gateway");

void log(LogLevel level,
         const std::string& component,
         const std::string& message);

void debug(const std::string& component, const std::string& message);
void info(const std::string& component, const std::string& message);
void warn(const std::string& component, const std::string& message);
void error(const std::string& component, const std::string& message);

[[nodiscard]] bool packet_trace_enabled() noexcept;

void trace_packet(const std::string& bus,
                  const std::string& direction,
                  const std::string& endpoint,
                  const std::vector<std::uint8_t>& bytes,
                  const std::string& detail = "");

}  // namespace snappy::gateway::diagnostics

#define SNAPPY_GATEWAY_LOG_DEBUG(component, message) \
  ::snappy::gateway::diagnostics::debug((component), (message))
#define SNAPPY_GATEWAY_LOG_INFO(component, message) \
  ::snappy::gateway::diagnostics::info((component), (message))
#define SNAPPY_GATEWAY_LOG_WARN(component, message) \
  ::snappy::gateway::diagnostics::warn((component), (message))
#define SNAPPY_GATEWAY_LOG_ERROR(component, message) \
  ::snappy::gateway::diagnostics::error((component), (message))
  