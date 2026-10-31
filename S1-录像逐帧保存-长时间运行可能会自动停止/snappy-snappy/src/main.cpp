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
 * - 主循环存活期间喂硬件看门狗；主循环卡死时由 SoC 硬件复位，
 *   用于覆盖“板子卡死、网络与 SSH 全部失联”这类软件无法自救的情况。
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
#include "snappy/control/watchdog.hpp"
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

  std::string socket_path = "/run/snappy/control.sock";

  for (int index = 1;
       index < argc;
       ++index) {
    const std::string argument = argv[index];

    if (argument == "--help" || argument == "-h") {
      print_help(argv[0]);
      return 0;
    }

    if (argument == "--config" && index + 1 < argc) {
      config_path = argv[++index];
      continue;
    }

    if (argument == "--socket" && index + 1 < argc) {
      socket_path = argv[++index];
      continue;
    }

    std::cerr
        << "unknown argument: "
        << argument
        << "\n";

    return 2;
  }

  const auto loaded =
      snappy::control::load_hardware_config(config_path);

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
  config.hardware = loaded.config;
  config.socket_path = socket_path;

  SNAPPY_DRIVER_LOG_INFO(
      "snappy-snappy",
      "starting actuator-only control process");

  /*
   * 进程级硬件看门狗对象。
   *
   * 这里只构造，不启动。
   *
   * 真正 start() 放在 hub / server 都就绪、进入主循环之前：
   * 设备初始化或失败重试的耗时有可能接近甚至超过 timeout_sec，
   * 如果更早就把看门狗挂上，会变成“启动阶段被看门狗复位”的死循环。
   */
  snappy::control::LinuxWatchdog watchdog(
      config.hardware.watchdog);

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

  snappy_snappy::ControlServer server(hub, config);

  if (!server.start()) {
    SNAPPY_DRIVER_LOG_ERROR(
        "snappy-snappy",
        "control socket start failed: " + config.socket_path);

    hub.stop();
    return 1;
  }

  std::cout
      << "snappy-snappy-control ready, socket="
      << config.socket_path
      << "\n";

  /*
   * 启动看门狗。
   */
  const auto watchdog_result = watchdog.start();

  const auto watchdog_status = watchdog.status();

  if (watchdog_status.active) {
    SNAPPY_DRIVER_LOG_INFO(
        "snappy-snappy",
        "hardware watchdog armed: device=" +
            watchdog_status.device_path +
            " timeout_sec=" +
            std::to_string(
                watchdog_status.timeout_sec) +
            " feed_interval_ms=" +
            std::to_string(
                config.hardware.watchdog
                    .feed_interval.count()));
  } else {
    SNAPPY_DRIVER_LOG_WARN(
        "snappy-snappy",
        "hardware watchdog NOT active: " + watchdog_status.detail);
  }

  const auto period = std::chrono::milliseconds(20);

  auto next = std::chrono::steady_clock::now();

  while (g_running.load()) {
    server.tick();

    /*
     * 喂狗。
     */
    static_cast<void>(
        watchdog.feed_if_due(
            std::chrono::steady_clock::now()));

    next += period;

    const auto now = std::chrono::steady_clock::now();

    if (next <= now) {
      next = now + period;
    }

    std::this_thread::sleep_until(next);
  }

  /*
   * 先停看门狗，再走关停流程。
   *
   * server.stop() 与 hub.stop() 内部要做物理安全停止，
   * 耗时可能接近 systemd 的 TimeoutStopSec；
   * 如果等到最后才写 Magic Close，
   * 有可能在关停过程中被看门狗复位。
   */
  watchdog.stop();

  server.stop();
  hub.stop();

  SNAPPY_DRIVER_LOG_INFO(
      "snappy-snappy",
      "actuator-only control process stopped");

  return 0;
}
