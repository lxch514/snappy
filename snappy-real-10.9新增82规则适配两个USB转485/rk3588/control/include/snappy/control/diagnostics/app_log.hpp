#pragma once
/**
 * @file app_log.hpp
 * @brief 控制进程日志门面声明
 *
 * @details
 * - 统一转发到驱动层日志实现，组件名自动加上 control. 前缀。
 * - 业务线程只入队，落盘由独立写入线程完成，不阻塞控制路径。
 */

#include <cstdint>
#include <string>
#include <vector>

#include "snappy/drivers/diagnostics/driver_log.hpp"
#include "snappy/control/sensor_service.hpp"

namespace snappy::control::diagnostics {

using LogLevel = ::snappy::drivers::diagnostics::LogLevel;

void configure_from_environment(const std::string& process_name = "snappy-control");

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

}  // namespace snappy::control::diagnostics

#define SNAPPY_APP_LOG_DEBUG(component, message) \
  ::snappy::control::diagnostics::debug((component), (message))
#define SNAPPY_APP_LOG_INFO(component, message) \
  ::snappy::control::diagnostics::info((component), (message))
#define SNAPPY_APP_LOG_WARN(component, message) \
  ::snappy::control::diagnostics::warn((component), (message))
#define SNAPPY_APP_LOG_ERROR(component, message) \
  ::snappy::control::diagnostics::error((component), (message))
  