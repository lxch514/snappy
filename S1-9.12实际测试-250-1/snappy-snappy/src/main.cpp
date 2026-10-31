/**
 * @file main.cpp
 * @brief snappy-snappy 执行器专用进程入口
 *
 * @details
 * - 使用原工程 load_hardware_config()。
 * - 不创建 DeviceRegistry。
 * - 不创建 SensorService。
 * - 不创建 GatewayCommandDispatcher。
 * - 不读取任何传感器。
 * - systemd 负责自启动和异常重启。
 */

#include "snappy_snappy/actuator_hub.hpp"
#include "snappy_snappy/control_server.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "snappy/control/hardware_config.hpp"
#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace {

std::atomic_bool g_running{true};

void on_signal(int) {
  g_running.store(false);
}

void print_help(const char* argv0) {
  std::cout
      << "Usage: "
      << argv0
      << " [--config path] [--socket path]\n";
}

}  // namespace

int main(
    int argc,
    char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  snappy::drivers::diagnostics::
      configure_from_environment(
          "snappy-snappy-control");

  std::string config_path =
      "snappy-snappy/config/"
      "hardware.debug.actuator-only.json";

  std::string socket_path =
      "/run/snappy/control.sock";

  for (int index = 1;
       index < argc;
       ++index) {
    const std::string argument =
        argv[index];

    if (argument == "--help" ||
        argument == "-h") {
      print_help(argv[0]);
      return 0;
    }

    if (argument == "--config" &&
        index + 1 < argc) {
      config_path =
          argv[++index];
      continue;
    }

    if (argument == "--socket" &&
        index + 1 < argc) {
      socket_path =
          argv[++index];
      continue;
    }

    std::cerr
        << "unknown argument: "
        << argument
        << "\n";

    return 2;
  }

  const auto loaded =
      snappy::control::load_hardware_config(
          config_path);

  if (!loaded.result.ok) {
    SNAPPY_DRIVER_LOG_ERROR(
        "snappy-snappy",
        loaded.result.detail);

    std::cerr
        << "hardware config failed: "
        << loaded.result.detail
        << "\n";

    return 2;
  }

  snappy_snappy::RuntimeConfig config;
  config.hardware =
      loaded.config;
  config.socket_path =
      socket_path;

  SNAPPY_DRIVER_LOG_INFO(
      "snappy-snappy",
      "starting actuator-only control process");

  snappy_snappy::ActuatorHub hub(config);
  const auto hub_result =
      hub.start();

  const bool allow_partial =
      config.hardware.bringup
          .allow_partial_devices;

  if (!hub_result.ok &&
      !allow_partial) {
    SNAPPY_DRIVER_LOG_ERROR(
        "snappy-snappy",
        hub_result.detail);

    hub.stop();
    return 1;
  }

  snappy_snappy::ControlServer server(
      hub,
      config);

  if (!server.start()) {
    SNAPPY_DRIVER_LOG_ERROR(
        "snappy-snappy",
        "control socket start failed: " +
            config.socket_path);

    hub.stop();
    return 1;
  }

  std::cout
      << "snappy-snappy-control ready, socket="
      << config.socket_path
      << "\n";

  const auto period =
      std::chrono::milliseconds(20);

  auto next =
      std::chrono::steady_clock::now();

  while (g_running.load()) {
    server.tick();

    next += period;

    const auto now =
        std::chrono::steady_clock::now();

    if (next <= now) {
      next = now + period;
    }

    std::this_thread::sleep_until(next);
  }

  server.stop();
  hub.stop();

  SNAPPY_DRIVER_LOG_INFO(
      "snappy-snappy",
      "actuator-only control process stopped");

  return 0;
}