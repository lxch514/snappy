/**
 * @file driver_log.cpp
 * @brief 驱动层和应用层统一日志初始化与输出实现
 *
 * @details
 * - 按自然日创建 .log 文件，同一天内所有进程和重启实例继续追加。
 * - 控制台输出仍由运行时配置决定，不改变原有终端交互。
 * - 日志内容保留进程名和组件名，便于区分程序端、网关和驱动模块。
 */

#include "snappy/drivers/diagnostics/driver_log.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <utility>

#if defined(_WIN32)
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

namespace snappy::drivers::diagnostics {
namespace {

std::mutex& log_mutex() {
  static std::mutex mutex;
  return mutex;
}

LogConfig& global_config() {
  static LogConfig config;
  return config;
}

std::string& active_log_path() {
  static std::string path;
  return path;
}

bool& configured_flag() {
  static bool configured = false;
  return configured;
}

bool truthy_env(const char* value) {
  if (value == nullptr) {
    return false;
  }

  std::string text(value);
  std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  return !(text.empty() || text == "0" || text == "false" ||
           text == "off" || text == "no");
}

std::string env_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    return fallback;
  }
  return value;
}

bool mkdir_one(const std::string& path) {
  if (path.empty()) {
    return true;
  }

#if defined(_WIN32)
  return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
  return ::mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
#endif
}

void mkdir_recursive(std::string path) {
  if (path.empty()) {
    return;
  }

  std::replace(path.begin(), path.end(), '\\', '/');

  std::string current;
  std::size_t start = 0;
  if (!path.empty() && path[0] == '/') {
    current = "/";
    start = 1;
  }

  while (start <= path.size()) {
    const auto slash = path.find('/', start);
    const auto part = path.substr(start, slash - start);
    if (!part.empty()) {
      if (!current.empty() && current.back() != '/') {
        current.push_back('/');
      }
      current += part;
      mkdir_one(current);
    }

    if (slash == std::string::npos) {
      break;
    }
    start = slash + 1;
  }
}

std::string now_string() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch())
                      .count() %
                  1000;

  std::tm tm {};
#if defined(_WIN32)
  localtime_s(&tm, &seconds);
#else
  localtime_r(&seconds, &tm);
#endif

  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.'
      << std::setw(3) << std::setfill('0') << ms;
  return out.str();
}

const char* level_name(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return "debug";
    case LogLevel::kInfo:
      return "info";
    case LogLevel::kWarn:
      return "warn";
    case LogLevel::kError:
      return "error";
  }
  return "info";
}

bool should_write(LogLevel level) {
  const auto& config = global_config();
  return level != LogLevel::kDebug || config.debug;
}

std::string trim_line(std::string line, std::uint64_t max_bytes) {
  for (char& c : line) {
    if (c == '\n' || c == '\r') {
      c = ' ';
    }
  }

  if (max_bytes > 0 && line.size() > max_bytes) {
    line.resize(static_cast<std::size_t>(max_bytes));
    line += "...";
  }
  return line;
}

std::string log_path() {
  return active_log_path();
}

/* ============================================================
 * 按自然日生成日志文件
 *
 * 同一天内 control、gateway 和驱动线程都追加到同一个文件；
 * 跨午夜后的第一条日志自动切换到新日期文件。
 * ============================================================ */

std::string daily_log_path(const std::string& base_dir) {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);

  std::tm tm {};
#if defined(_WIN32)
  localtime_s(&tm, &seconds);
#else
  localtime_r(&seconds, &tm);
#endif

  std::ostringstream date;
  date << std::put_time(&tm, "%Y-%m-%d");

  mkdir_recursive(base_dir);
  return base_dir + "/" + date.str() + ".log";
}

void ensure_configured() {
  if (configured_flag()) {
    return;
  }
  configure_from_environment(global_config().process_name);
}

}  // namespace

void configure(LogConfig config) {
  std::lock_guard<std::mutex> lock(log_mutex());

  if (config.process_name.empty()) {
    config.process_name = "snappy";
  }
  if (config.log_dir.empty()) {
    config.log_dir = "/var/log/snappy";
  }

  mkdir_recursive(config.log_dir);
  active_log_path() = daily_log_path(config.log_dir);
  global_config() = std::move(config);
  configured_flag() = true;

  std::ofstream file(active_log_path(), std::ios::app);
  if (file) {
    file << '[' << now_string() << "] [info] ["
         << global_config().process_name
         << "] [logger] === 日志系统初始化完成 ===\n";
    file << '[' << now_string() << "] [info] ["
         << global_config().process_name
         << "] [logger] 日志文件: "
         << active_log_path() << '\n';
  }
}

void configure_from_environment(const std::string& process_name) {
  LogConfig config;
  config.process_name = process_name.empty() ? "snappy" : process_name;
  config.log_dir = env_or("SNAPPY_LOG_DIR", "/var/log/snappy");
  config.console = !truthy_env(std::getenv("SNAPPY_LOG_NO_CONSOLE"));
  config.debug = truthy_env(std::getenv("SNAPPY_DEBUG")) ||
                 env_or("SNAPPY_LOG_LEVEL", "") == "debug";
  config.packet_trace = truthy_env(std::getenv("SNAPPY_PACKET_TRACE")) ||
                        truthy_env(std::getenv("SNAPPY_TRACE_PACKETS"));

  configure(std::move(config));
}

void log(LogLevel level,
         const std::string& component,
         const std::string& message) {
  ensure_configured();
  if (!should_write(level)) {
    return;
  }

  std::lock_guard<std::mutex> lock(log_mutex());
  const auto& config = global_config();

  /* ----------------------------------------------------------
   * 长时间运行跨过午夜时，自动切换到当天的日志文件。
   * ---------------------------------------------------------- */
  active_log_path() = daily_log_path(config.log_dir);

  std::ostringstream line;
  line << '[' << now_string() << "] [" << level_name(level) << "] ["
       << config.process_name << "] [" << component << "] "
       << trim_line(message, config.max_line_bytes) << '\n';

  std::ofstream file(log_path(), std::ios::app);
  if (file) {
    file << line.str();
  }

  if (config.console) {
    if (level == LogLevel::kError || level == LogLevel::kWarn) {
      std::cerr << line.str();
    } else {
      std::cout << line.str();
    }
  }
}

void debug(const std::string& component, const std::string& message) {
  log(LogLevel::kDebug, component, message);
}

void info(const std::string& component, const std::string& message) {
  log(LogLevel::kInfo, component, message);
}

void warn(const std::string& component, const std::string& message) {
  log(LogLevel::kWarn, component, message);
}

void error(const std::string& component, const std::string& message) {
  log(LogLevel::kError, component, message);
}

bool packet_trace_enabled() noexcept {
  return global_config().packet_trace;
}

std::string hex_dump(const std::vector<std::uint8_t>& bytes) {
  std::ostringstream out;
  out << std::hex << std::uppercase << std::setfill('0');

  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i != 0) {
      out << ' ';
    }
    out << std::setw(2) << static_cast<int>(bytes[i]);
  }

  return out.str();
}

void trace_packet(const std::string& bus,
                  const std::string& direction,
                  const std::string& endpoint,
                  const std::vector<std::uint8_t>& bytes,
                  const std::string& detail) {
  if (!packet_trace_enabled()) {
    return;
  }

  std::ostringstream out;
  out << bus << ' ' << direction << ' ' << endpoint
      << " bytes=" << bytes.size();

  if (!detail.empty()) {
    out << " detail=" << detail;
  }

  out << " hex=" << hex_dump(bytes);

  /*
   * packet_trace 只放开数据包日志，不改变普通 debug 日志级别。
   */
  if (global_config().debug) {
    debug("packet", out.str());
  } else {
    log(LogLevel::kInfo, "packet", out.str());
  }
}

}  // namespace snappy::drivers::diagnostics
