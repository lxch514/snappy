#pragma once

/**
 * @file command_application.hpp
 * @brief 本地命令调试入口声明
 *
 * @details
 * - 对应 snappy-control --stdin / --local-test 模式。
 * - 现场不接 G20 时，可以在终端直接测试毛刷、推杆、风机、爬绳器。
 * - 传感器命令只读取 SensorService 的缓存快照，不直接访问底层 RS485。
 * - sensor-status 用于查看每个传感器最近在线状态、失败次数和最后错误。
 */

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

#include "snappy/control/device_registry.hpp"
#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::control {

struct CommandApplicationOptions {
  bool show_help{false};
  bool stdin_enabled{false};
  bool sensor_once{false};
  bool print_sensor_json{false};

  std::string config_path{"rk3588/config/hardware.snappy.json"};

  std::chrono::milliseconds loop_period{20};
  std::chrono::milliseconds sensor_period{100};
};

[[nodiscard]] CommandApplicationOptions parse_command_application_options(
    int argc,
    char** argv);

void print_command_application_usage(const char* argv0);

class CommandApplication {
 public:
  CommandApplication(DeviceRegistry& registry,
                     CommandApplicationOptions options);

  int run(std::atomic_bool& running);

 private:
  void print_status(const std::string& state,
                    bool ready,
                    const std::string& detail) const;

  void print_result(const std::string& command,
                    const drivers::Result& result) const;

  void print_samples(
      const std::vector<drivers::SensorSample>& samples) const;

  void print_sensor_statuses() const;

  bool execute_command(const std::string& line);

  DeviceRegistry& registry_;
  CommandApplicationOptions options_;
  bool quit_requested_{false};
};

}  // namespace snappy::control