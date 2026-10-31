#include "snappy/control/watchdog.hpp"

#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>

#include "snappy/control/diagnostics/app_log.hpp"

#if defined(__linux__)
#include <cerrno>
#include <fcntl.h>
#include <linux/watchdog.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace snappy::control {
namespace {

std::uint64_t wall_time_ms() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

bool env_disables_watchdog() {
  const char* value = std::getenv("SNAPPY_WATCHDOG_DISABLE");
  if (value == nullptr) return false;
  const std::string text(value);
  return text == "1" || text == "true" || text == "TRUE" || text == "yes";
}

std::string env_or(const char* key, const std::string& fallback) {
  const char* value = std::getenv(key);
  return value == nullptr || value[0] == '\0' ? fallback : std::string(value);
}

}  // namespace

LinuxWatchdog::LinuxWatchdog(WatchdogConfig config)
    : config_(std::move(config)) {}

bool LinuxWatchdog::enabled() const noexcept {
  return config_.enabled;
}

bool LinuxWatchdog::active() const noexcept {
  return active_;
}

drivers::Result LinuxWatchdog::start() {
  if (!config_.enabled) {
    last_detail_ = "watchdog disabled by config";
    return drivers::Result::success(last_detail_);
  }

  if (env_disables_watchdog()) {
    last_detail_ = "watchdog disabled by SNAPPY_WATCHDOG_DISABLE";
    SNAPPY_APP_LOG_WARN("watchdog", last_detail_);
    return drivers::Result::success(last_detail_);
  }

  config_.device_path = env_or("SNAPPY_WATCHDOG_DEVICE", config_.device_path);

#if defined(__linux__)
  if (active_) {
    return drivers::Result::success("watchdog already active");
  }

  fd_ = ::open(config_.device_path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd_ < 0) {
    last_detail_ =
        "open " + config_.device_path + " failed: " + std::strerror(errno);
    SNAPPY_APP_LOG_ERROR("watchdog", last_detail_);
    return drivers::Result::failure(drivers::ErrorCode::kIoError,
                                    last_detail_,
                                    "start",
                                    "watchdog",
                                    true);
  }

  if (config_.timeout_sec > 0) {
    int timeout = config_.timeout_sec;
    if (::ioctl(fd_, WDIOC_SETTIMEOUT, &timeout) != 0) {
      SNAPPY_APP_LOG_WARN(
          "watchdog",
          std::string("WDIOC_SETTIMEOUT failed: ") + std::strerror(errno));
    } else {
      config_.timeout_sec = timeout;
    }
  }

  int actual_timeout = 0;
  if (::ioctl(fd_, WDIOC_GETTIMEOUT, &actual_timeout) == 0 &&
      actual_timeout > 0) {
    config_.timeout_sec = actual_timeout;
  }

  active_ = true;
  last_detail_ = "watchdog active";
  SNAPPY_APP_LOG_INFO(
      "watchdog",
      "started " + config_.device_path +
          " timeout_sec=" + std::to_string(config_.timeout_sec));

  return feed();
#else
  last_detail_ = "linux watchdog requires Linux";
  SNAPPY_APP_LOG_WARN("watchdog", last_detail_);
  return drivers::Result::failure(drivers::ErrorCode::kUnsupported,
                                  last_detail_,
                                  "start",
                                  "watchdog");
#endif
}

drivers::Result LinuxWatchdog::feed() {
  if (!config_.enabled || !active_) {
    return drivers::Result::success("watchdog inactive");
  }

#if defined(__linux__)
  int dummy = 0;
  if (::ioctl(fd_, WDIOC_KEEPALIVE, &dummy) != 0) {
    const char byte = '\0';
    if (::write(fd_, &byte, 1) < 0) {
      last_detail_ =
          std::string("watchdog feed failed: ") + std::strerror(errno);
      SNAPPY_APP_LOG_ERROR("watchdog", last_detail_);
      return drivers::Result::failure(drivers::ErrorCode::kIoError,
                                      last_detail_,
                                      "feed",
                                      "watchdog",
                                      true);
    }
  }

  last_feed_ = std::chrono::steady_clock::now();
  last_feed_wall_ms_ = wall_time_ms();
  last_detail_ = "watchdog fed";
  return drivers::Result::success(last_detail_);
#else
  return drivers::Result::failure(drivers::ErrorCode::kUnsupported,
                                  "linux watchdog requires Linux",
                                  "feed",
                                  "watchdog");
#endif
}

drivers::Result LinuxWatchdog::feed_if_due(
    std::chrono::steady_clock::time_point now) {
  if (!config_.enabled || !active_) {
    return drivers::Result::success("watchdog inactive");
  }

  if (last_feed_.time_since_epoch().count() != 0 &&
      now - last_feed_ < config_.feed_interval) {
    return drivers::Result::success("watchdog feed not due");
  }

  return feed();
}

void LinuxWatchdog::stop() noexcept {
#if defined(__linux__)
  if (fd_ >= 0) {
    if (config_.stop_on_clean_shutdown) {
      const char magic = 'V';
      static_cast<void>(::write(fd_, &magic, 1));
    }

    ::close(fd_);
    fd_ = -1;
  }
#endif

  if (active_) {
    SNAPPY_APP_LOG_INFO("watchdog", "stopped");
  }

  active_ = false;
  last_detail_ = "watchdog stopped";
}

WatchdogStatus LinuxWatchdog::status() const {
  WatchdogStatus status;
  status.configured = config_.enabled;
  status.active = active_;
  status.device_path = config_.device_path;
  status.timeout_sec = config_.timeout_sec;
  status.last_feed_ms = last_feed_wall_ms_;
  status.detail = last_detail_;
  return status;
}

}  // namespace snappy::control