/**
 * @file main_real.cpp
 * @brief snappy-real：10传感器 + 4执行器 + 摄像头整机控制入口。
 *
 * 运行时核心：
 *   - 执行器：snappy-snappy 已验证的 ActuatorHub + DirectRs485Bus；
 *   - 传感器：原工程 SensorService，使用同一个 DirectRs485Bus；
 *   - 不创建 DeviceRegistry，不启动旧 ActuatorService 控制链路；
 *   - 所有 RS485 物理端口仍由同一个 scheduler 管理，每端口独立 worker，
 *     ttyACM0/1/2 共享 WCH 事务仲裁；
 *   - status/telemetry 只读取内存快照。
 */

#include "snappy/control/diagnostics/app_log.hpp"
#include "snappy/control/hardware_config.hpp"
#include "snappy/control/sensor_runtime.hpp"
#include "snappy/control/watchdog.hpp"
#include "snappy/drivers/diagnostics/driver_log.hpp"
#include "snappy_snappy/actuator_hub.hpp"
#include "snappy_snappy/control_server.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) {
  g_running.store(false);
}

void print_help(const char* argv0) {
  std::cout
      << "Usage: " << argv0
      << " [--config path] [--socket path]\n";
}

}  // namespace

int main(int argc, char** argv) {
  snappy::control::diagnostics::configure_from_environment("snappy-real-control");
  snappy::drivers::diagnostics::configure_from_environment("snappy-real-control");

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::string config_path = "rk3588/config/hardware.debug.test.json";
  std::string socket_path = "/run/snappy-real/control.sock";

  for (int index = 1; index < argc; ++index) {
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

    std::cerr << "unknown argument: " << argument << "\n";
    print_help(argv[0]);
    return 2;
  }

  const auto loaded = snappy::control::load_hardware_config(config_path);

  if (!loaded.result.ok) {
    SNAPPY_DRIVER_LOG_ERROR(
        "snappy-real",
        "hardware config failed: " + loaded.result.detail);
    return 2;
  }

  snappy_snappy::RuntimeConfig runtime;
  runtime.hardware = loaded.config;
  runtime.socket_path = socket_path;

  SNAPPY_DRIVER_LOG_INFO("snappy-real", "starting full control process: 10 sensors + 4 actuators + camera");

  /*
   * 进程级硬件看门狗对象。
   */
  snappy::control::LinuxWatchdog watchdog(runtime.hardware.watchdog);

  snappy_snappy::ActuatorHub hub(runtime);

  /*
   * 传感器复用 ActuatorHub 内部的同一个 DirectRs485Bus。
   * 这样所有 RS485 端口只有一个 scheduler：
   *   - ttyS0 / ttyS3 各自独立 worker；
   *   - ttyACM0 / ttyACM1 / ttyACM2 各自独立 worker；
   *   - ACM0~2 继续通过同一个 WCH shared_transaction_group 仲裁。
   */
  snappy::control::SensorRuntime sensors(
      runtime.hardware,
      hub.rs485_bus());

  const auto sensor_configured = sensors.configure();
  if (!sensor_configured.ok) {
    SNAPPY_DRIVER_LOG_ERROR("snappy-real", "sensor runtime configure failed: " + sensor_configured.detail);
    return 2;
  }

  runtime.sensors = &sensors.service();

  const auto hub_result = hub.start();
  const bool allow_partial = runtime.hardware.bringup.allow_partial_devices;

  if (!hub_result.ok && !allow_partial) {
    SNAPPY_DRIVER_LOG_ERROR("snappy-real", "actuator hub start failed: " + hub_result.detail);
    hub.stop();
    return 1;
  }

  if (!hub_result.ok) {
    SNAPPY_DRIVER_LOG_WARN("snappy-real", "actuator hub started with partial hardware: " + hub_result.detail);
  }

  const auto sensor_result = sensors.start();
  if (!sensor_result.ok && !allow_partial) {
    SNAPPY_DRIVER_LOG_ERROR("snappy-real", "sensor runtime start failed: " + sensor_result.detail);
    hub.stop();
    return 1;
  }

  if (!sensor_result.ok) {
    SNAPPY_DRIVER_LOG_WARN("snappy-real", "sensor runtime started with isolated failures: " + sensor_result.detail);
  }

  snappy_snappy::ControlServer server(hub, runtime);
  if (!server.start()) {
    SNAPPY_DRIVER_LOG_ERROR("snappy-real", "control socket start failed: " + socket_path);
    sensors.stop();
    hub.stop();
    return 1;
  }

  std::cout
      << "snappy-real-control ready, socket="
      << socket_path << "\n";

  /*
   * 启动看门狗。
   */
  static_cast<void>(watchdog.start());

  const auto watchdog_status = watchdog.status();

  if (watchdog_status.active) {
    SNAPPY_DRIVER_LOG_INFO(
        "snappy-real-control",
        "hardware watchdog armed: device=" +
            watchdog_status.device_path +
            " timeout_sec=" +
            std::to_string(
                watchdog_status.timeout_sec) +
            " feed_interval_ms=" +
            std::to_string(
                runtime.hardware.watchdog
                    .feed_interval.count()));
  } else if (watchdog.enabled()) {
    SNAPPY_DRIVER_LOG_WARN(
        "snappy-real-control",
        "hardware watchdog NOT active: " + watchdog_status.detail);
  }

  constexpr auto kLoopPeriod = std::chrono::milliseconds(20);
  auto next = std::chrono::steady_clock::now();

  while (g_running.load()) {
    server.tick();

    /*
     * 主循环存活期间喂硬件看门狗。
     *
     * 只有主循环真正转起来才喂狗：
     * server.tick() 和传感器轮询都在独立线程里，
     * 主循环卡死即判定进程失活，由 SoC 复位兜底。
     */
    static_cast<void>(
        watchdog.feed_if_due(
            std::chrono::steady_clock::now()));

    next += kLoopPeriod;
    const auto now = std::chrono::steady_clock::now();
    if (next <= now) {
      next = now + kLoopPeriod;
    }

    std::this_thread::sleep_until(next);
  }

  /*
   * 先停看门狗，再走关停流程。
   */
  watchdog.stop();

  server.stop();
  sensors.stop();
  hub.stop();

  SNAPPY_DRIVER_LOG_INFO(
      "snappy-real",
      "full control process stopped");

  return 0;
}
