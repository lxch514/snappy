#pragma once
/**
 * @file watchdog.hpp
 * @brief Linux 硬件看门狗封装声明
 *
 * @details
 * - 打开 /dev/watchdog 后按配置周期喂狗。
 * - 未配置或打开失败时保持不启用，不影响其他模块启动。
 */

#include <chrono>
#include <cstdint>
#include <string>

#include "snappy/control/hardware_config.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::control {

struct WatchdogStatus {
  bool configured{false};
  bool active{false};
  std::string device_path;
  int timeout_sec{0};
  std::uint64_t last_feed_ms{0};
  std::string detail;
};

class LinuxWatchdog final {
 public:
  explicit LinuxWatchdog(WatchdogConfig config);

  drivers::Result start();
  drivers::Result feed();
  drivers::Result feed_if_due(std::chrono::steady_clock::time_point now);
  void stop() noexcept;

  [[nodiscard]] bool enabled() const noexcept;
  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] WatchdogStatus status() const;

 private:
  WatchdogConfig config_;
  int fd_{-1};
  bool active_{false};
  std::chrono::steady_clock::time_point last_feed_{};
  std::uint64_t last_feed_wall_ms_{0};
  std::string last_detail_{"watchdog not started"};
};

}  // namespace snappy::control