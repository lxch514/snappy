/**
 * @file driver_log.hpp
 * @brief 驱动层、程序端和网关统一日志接口
 *
 * @details
 * - 日志文件按自然日追加保存为 <log_dir>/<YYYY-MM-DD>.log。
 * - debug 级别用于完整记录 G20 原始舵量和数据包，info 级别记录操作动作。
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace snappy::drivers::diagnostics {

enum class LogLevel {
  kDebug,
  kInfo,
  kWarn,
  kError,
};

struct LogConfig {
  std::string process_name{"snappy"};
  std::string log_dir{"/var/log/snappy"};
  bool console{true};
  bool debug{false};
  bool packet_trace{false};
  std::uint64_t max_line_bytes{4096};
};

void configure(LogConfig config);
void configure_from_environment(const std::string& process_name = "snappy");

void log(LogLevel level,
         const std::string& component,
         const std::string& message);

void debug(const std::string& component, const std::string& message);
void info(const std::string& component, const std::string& message);
void warn(const std::string& component, const std::string& message);
void error(const std::string& component, const std::string& message);

[[nodiscard]] bool packet_trace_enabled() noexcept;

std::string hex_dump(const std::vector<std::uint8_t>& bytes);

void trace_packet(const std::string& bus,
                  const std::string& direction,
                  const std::string& endpoint,
                  const std::vector<std::uint8_t>& bytes,
                  const std::string& detail = "");

/*
 * RS485 诊断关联上下文。
 *
 * 这些值只用于日志关联，不参与任何通信、调度、重试或控制判断。
 * 使用 thread_local 是为了让 scheduler worker 调用 port->transact() 时，
 * 串口层可以看到同一笔事务的诊断编号，而不修改 Rs485Request /
 * Rs485Transaction 的数据结构。
 */
inline std::atomic<std::uint64_t> g_rs485_diagnostic_sequence{0};
inline thread_local std::uint64_t g_rs485_diagnostic_tx_id = 0;
inline thread_local std::string g_rs485_diagnostic_logical_device;
inline thread_local std::string g_rs485_diagnostic_endpoint;

inline std::uint64_t begin_rs485_diagnostic_transaction(
    const std::string& logical_device,
    const std::string& endpoint) {
  const auto id =
      g_rs485_diagnostic_sequence.fetch_add(
          1, std::memory_order_relaxed) + 1;

  g_rs485_diagnostic_tx_id = id;
  g_rs485_diagnostic_logical_device = logical_device;
  g_rs485_diagnostic_endpoint = endpoint;

  return id;
}

inline std::uint64_t current_rs485_diagnostic_tx_id() noexcept {
  return g_rs485_diagnostic_tx_id;
}

inline const std::string&
current_rs485_diagnostic_logical_device() noexcept {
  return g_rs485_diagnostic_logical_device;
}

inline const std::string&
current_rs485_diagnostic_endpoint() noexcept {
  return g_rs485_diagnostic_endpoint;
}

inline void end_rs485_diagnostic_transaction() noexcept {
  g_rs485_diagnostic_tx_id = 0;
  g_rs485_diagnostic_logical_device.clear();
  g_rs485_diagnostic_endpoint.clear();
}

}  // namespace snappy::drivers::diagnostics

#define SNAPPY_DRIVER_LOG_DEBUG(component, message) \
  ::snappy::drivers::diagnostics::debug((component), (message))
#define SNAPPY_DRIVER_LOG_INFO(component, message) \
  ::snappy::drivers::diagnostics::info((component), (message))
#define SNAPPY_DRIVER_LOG_WARN(component, message) \
  ::snappy::drivers::diagnostics::warn((component), (message))
#define SNAPPY_DRIVER_LOG_ERROR(component, message) \
  ::snappy::drivers::diagnostics::error((component), (message))
  