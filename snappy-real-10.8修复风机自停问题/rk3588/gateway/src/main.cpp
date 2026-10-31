/**
 * @file main.cpp
 * @brief snappy-gateway 进程入口
 *
 * @details
 * - 解析启动参数，加载配置后创建并运行网关应用。
 * - 注册信号处理，收到退出信号时按序停止。
 */

#include <atomic>
#include <csignal>
#include <iostream>

#include "snappy/gateway/diagnostics/gateway_log.hpp"
#include "snappy/gateway/gateway_app.hpp"
#include "snappy/gateway/gateway_config.hpp"

namespace {

std::atomic_bool g_running{true};

void on_signal(int) {
  g_running = false;
}

}  // namespace

int main(int argc, char** argv) {
  snappy::gateway::diagnostics::configure_from_environment("snappy-gateway");
  SNAPPY_GATEWAY_LOG_INFO("main", "snappy-gateway starting");

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  const auto options = snappy::gateway::parse_gateway_app_options(argc, argv);
  if (options.show_help) {
    snappy::gateway::print_gateway_app_usage(argv[0]);
    return 0;
  }

  SNAPPY_GATEWAY_LOG_INFO("main", "loading gateway config: " + options.config_path);
  const auto loaded = snappy::gateway::load_gateway_config(options.config_path);
  if (!loaded.result.ok) {
    std::cerr << "{\"process\":\"snappy-gateway\","
              << "\"state\":\"config_error\","
              << "\"detail\":\"" << loaded.result.detail << "\"}\n";
    SNAPPY_GATEWAY_LOG_ERROR("main",
                             "load gateway config failed: " + loaded.result.detail);
    return 2;
  }

  snappy::gateway::GatewayApp app(loaded.config);
  const int code = app.run(g_running);

  SNAPPY_GATEWAY_LOG_INFO("main", "snappy-gateway stopped code=" + std::to_string(code));
  return code;
}