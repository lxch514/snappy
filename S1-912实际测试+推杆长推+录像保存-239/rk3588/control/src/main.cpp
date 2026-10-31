/**
 * @file main.cpp
 * @brief snappy-control 进程入口和整机生命周期
 *
 * @details
 *
 * - 加载并校验硬件配置。
 * - 构建设备注册表。
 * - 启动 dispatcher、IPC 和 watchdog。
 * - 主循环只执行短周期 tick，不同步读取传感器。
 * - 退出时先停止 IPC 接收，再停止控制和硬件后台线程。
 */

#include "snappy/control/command_application.hpp"
#include "snappy/control/device_registry.hpp"
#include "snappy/control/diagnostics/app_log.hpp"
#include "snappy/control/gateway_command_dispatcher.hpp"
#include "snappy/control/gateway_ipc_server.hpp"
#include "snappy/control/hardware_config.hpp"
#include "snappy/control/watchdog.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

namespace {

std::atomic_bool g_running{true};

void on_signal(int) {
  g_running.store(false);
}

void print_help(const char* argv0) {
  std::cout
      << "Usage: "
      << argv0
      << " [--config path] [--socket path] [--stdin]\n"
      << "  default mode: gateway IPC server on "
         "/run/snappy/control.sock\n"
      << "  --stdin: local command test mode\n"
      << "  SNAPPY_WATCHDOG_DISABLE=1 disables "
         "/dev/watchdog at runtime\n";
}

}  // namespace

int main(int argc, char** argv) {
  snappy::control::diagnostics::
      configure_from_environment(
          "snappy-control");

  SNAPPY_APP_LOG_INFO(
      "main",
      "snappy-control starting");

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::string config_path =
      "rk3588/config/hardware.snappy.json";

  std::string socket_path =
      "/run/snappy/control.sock";

  bool stdin_mode = false;

  /* ==========================================================
   * 命令行参数
   * ========================================================== */

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

    if (argument == "--stdin" ||
        argument == "--local-test") {
      stdin_mode = true;
      continue;
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

    SNAPPY_APP_LOG_ERROR(
        "main",
        "unknown argument: " +
            argument);

    print_help(argv[0]);
    return 2;
  }

  /* ==========================================================
   * 配置加载
   * ========================================================== */

  SNAPPY_APP_LOG_INFO(
      "main",
      "loading hardware config: " +
          config_path);

  auto loaded =
      snappy::control::
          load_hardware_config(
              config_path);

  if (!loaded.result.ok) {
    std::cerr
        << "load hardware config failed: "
        << loaded.result.detail
        << "\n";

    SNAPPY_APP_LOG_ERROR(
        "main",
        "load hardware config failed: " +
            loaded.result.detail);

    return 2;
  }

  const std::string robot_id =
      loaded.config.robot_id;

  const auto safety =
      loaded.config.safety;

  const auto g20_udp =
      loaded.config.g20_udp;

  const auto video =
      loaded.config.video;

  const auto watchdog_config =
      loaded.config.watchdog;

  SNAPPY_APP_LOG_INFO(
      "main",
      "config loaded robot_id=" +
          robot_id);

  /* ==========================================================
   * 设备注册表
   * ========================================================== */

  snappy::control::DeviceRegistry registry(
      std::move(loaded.config));

  const auto configured =
      registry.configure();

  if (!configured.ok) {
    std::cerr
        << "device registry configure failed: "
        << configured.detail
        << "\n";

    SNAPPY_APP_LOG_ERROR(
        "main",
        "device registry configure failed: " +
            configured.detail);

    return 2;
  }

  /* ==========================================================
   * 本地 stdin 调试模式
   * ========================================================== */

  if (stdin_mode) {
    SNAPPY_APP_LOG_INFO(
        "main",
        "starting stdin local-test mode");

    snappy::control::CommandApplicationOptions
        options;

    options.config_path =
        config_path;

    options.stdin_enabled =
        true;

    snappy::control::CommandApplication app(
        registry,
        options);

    const int exit_code =
        app.run(g_running);

    SNAPPY_APP_LOG_INFO(
        "main",
        "stdin local-test stopped code=" +
            std::to_string(exit_code));

    return exit_code;
  }

  /* ==========================================================
   * Dispatcher
   * ========================================================== */

  SNAPPY_APP_LOG_INFO(
      "main",
      "starting gateway command dispatcher");

  snappy::control::GatewayCommandDispatcher
      dispatcher(
          registry,
          robot_id,
          safety,
          g20_udp,
          video);

  const auto dispatcher_started =
      dispatcher.start();

  if (!dispatcher_started.ok) {
    std::cerr
        << "control dispatcher start failed: "
        << dispatcher_started.detail
        << "\n";

    SNAPPY_APP_LOG_ERROR(
        "main",
        "control dispatcher start failed: " +
            dispatcher_started.detail);

    /*
     * dispatcher.start() 已经执行失败清理。
     * registry 析构时仍会进行一次幂等 stop。
     */
    return 1;
  }

  /* ==========================================================
   * IPC 服务
   * ========================================================== */

  snappy::control::GatewayIpcServer ipc(
      {socket_path});

  const bool ipc_started =
      ipc.start(
          [&dispatcher](
              const std::string& request) {
            return dispatcher.handle_request(
                request);
          });

  if (!ipc_started) {
    std::cerr
        << "control IPC start failed\n";

    SNAPPY_APP_LOG_ERROR(
        "main",
        "control IPC start failed: " +
            socket_path);

    dispatcher.stop();
    return 1;
  }

  /* ==========================================================
   * Watchdog
   * ========================================================== */

  snappy::control::LinuxWatchdog watchdog(
      watchdog_config);

  const auto watchdog_started =
      watchdog.start();

  dispatcher.set_watchdog_status(
      watchdog.status());

  if (!watchdog_started.ok) {
    const std::string detail =
        "watchdog start failed: " +
        watchdog_started.detail;

    if (watchdog_config
            .fail_if_unavailable) {
      SNAPPY_APP_LOG_ERROR(
          "main",
          detail);

      ipc.stop();
      dispatcher.stop();
      watchdog.stop();

      return 1;
    }

    SNAPPY_APP_LOG_WARN(
        "main",
        detail);
  }

  std::cout
      << "snappy-control ready, ipc="
      << socket_path
      << "\n";

  SNAPPY_APP_LOG_INFO(
      "main",
      "snappy-control ready ipc=" +
          socket_path);

  /* ==========================================================
   * 主循环
   *
   * 传感器由 SensorService 后台线程采集。
   * dispatcher.tick() 只读取健康状态和缓存快照。
   * ========================================================== */

  constexpr auto kLoopPeriod =
      std::chrono::milliseconds(20);

  auto next_tick =
      std::chrono::steady_clock::now();

  auto next_tick_error_log =
      std::chrono::steady_clock::time_point{};

  while (g_running.load()) {
    const auto now =
        std::chrono::steady_clock::now();

    const auto tick_result =
        dispatcher.tick(now);

    if (!tick_result.ok &&
        (next_tick_error_log
                 .time_since_epoch()
                 .count() == 0 ||
         now >= next_tick_error_log)) {
      std::cerr
          << "control tick failed: "
          << tick_result.detail
          << "\n";

      SNAPPY_APP_LOG_WARN(
          "main",
          "control tick failed: " +
              tick_result.detail);

      /*
       * 防止故障设备让日志每 20 ms 刷屏。
       */
      next_tick_error_log =
          now +
          std::chrono::seconds(1);
    }

    const auto watchdog_feed =
        watchdog.feed_if_due(now);

    if (!watchdog_feed.ok) {
      SNAPPY_APP_LOG_ERROR(
          "main",
          "watchdog feed failed: " +
              watchdog_feed.detail);

      if (watchdog_config
              .fail_if_unavailable) {
        g_running.store(false);
      }
    }

    dispatcher.set_watchdog_status(
        watchdog.status());

    next_tick += kLoopPeriod;

    const auto after_tick =
        std::chrono::steady_clock::now();

    /*
     * 如果一次 tick 超时，不连续补跑大量旧 tick。
     */
    if (next_tick <= after_tick) {
      next_tick =
          after_tick +
          kLoopPeriod;
    }

    std::this_thread::sleep_until(
        next_tick);
  }

  /* ==========================================================
   * 退出
   *
   * 1. 停止 IPC，禁止新命令进入。
   * 2. 锁存安全状态并停止所有设备。
   * 3. 停止 watchdog。
   * ========================================================== */

  SNAPPY_APP_LOG_INFO(
      "main",
      "snappy-control stopping");

  ipc.stop();
  dispatcher.stop();
  watchdog.stop();

  SNAPPY_APP_LOG_INFO(
      "main",
      "snappy-control stopped");

  return 0;
}