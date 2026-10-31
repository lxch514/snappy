/**
 * @file app_log.cpp
 * @brief 控制进程日志门面实现
 *
 * @details
 * - 转发到驱动层日志，组件名加上 control. 前缀。
 */
 
#include "snappy/control/diagnostics/app_log.hpp"

namespace snappy::control::diagnostics {

void configure_from_environment(const std::string& process_name) {
  ::snappy::drivers::diagnostics::configure_from_environment(process_name);
}

void log(LogLevel level,
         const std::string& component,
         const std::string& message) {
  ::snappy::drivers::diagnostics::log(
      level,
      "control." + component,
      message);
}

void debug(const std::string& component, const std::string& message) {
  ::snappy::control::diagnostics::log(
      LogLevel::kDebug,
      component,
      message);
}

void info(const std::string& component, const std::string& message) {
  ::snappy::control::diagnostics::log(
      LogLevel::kInfo,
      component,
      message);
}

void warn(const std::string& component, const std::string& message) {
  ::snappy::control::diagnostics::log(
      LogLevel::kWarn,
      component,
      message);
}

void error(const std::string& component, const std::string& message) {
  ::snappy::control::diagnostics::log(
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

}  // namespace snappy::control::diagnostics