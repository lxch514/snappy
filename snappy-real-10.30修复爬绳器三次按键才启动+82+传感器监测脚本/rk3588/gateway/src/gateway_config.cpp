#include "snappy/gateway/gateway_config.hpp"
/**
 * @file gateway_config.cpp
 * @brief 网关配置加载实现
 *
 * @details
 * - 从 JSON 读取并校验网关运行参数。
 * - 校验失败返回带说明的错误，由启动流程决定是否继续。
 */

#include <fstream>
#include <utility>

#include <nlohmann/json.hpp>

namespace snappy::gateway {
namespace {

using Json = nlohmann::json;

drivers::Result config_error(const std::string& detail) {
  return drivers::Result::failure(drivers::ErrorCode::kConfigRequired,
                                  detail,
                                  "load",
                                  "gateway_config");
}

bool has_value(const Json& object, const char* key) {
  const auto it = object.find(key);
  return it != object.end() && !it->is_null();
}

int int_or(const Json& object, const char* key, int fallback) {
  return has_value(object, key) ? object.at(key).get<int>() : fallback;
}

bool bool_or(const Json& object, const char* key, bool fallback) {
  return has_value(object, key) ? object.at(key).get<bool>() : fallback;
}

std::string string_or(const Json& object,
                      const char* key,
                      const std::string& fallback = {}) {
  return has_value(object, key) ? object.at(key).get<std::string>() : fallback;
}

std::chrono::milliseconds ms_or(const Json& object,
                                const char* key,
                                int fallback_ms) {
  return std::chrono::milliseconds(int_or(object, key, fallback_ms));
}

}  // namespace

GatewayConfigLoadResult load_gateway_config(
    const std::string& hardware_config_path) {
  auto loaded = control::load_hardware_config(hardware_config_path);
  if (!loaded.result.ok) {
    return {loaded.result, {}};
  }

  GatewayRuntimeConfig config;
  config.hardware_config_path = hardware_config_path;
  config.robot_id =
      loaded.config.robot_id.empty() ? "snappy-001" : loaded.config.robot_id;
  config.control_channel.robot_id = config.robot_id;

  config.g20_udp.listen_address = loaded.config.g20_udp.listen_address;
  config.g20_udp.command_port = loaded.config.g20_udp.command_listen_port;
  config.g20_udp.status_port = loaded.config.g20_udp.status_publish_port;
  config.g20_udp.telemetry_port = loaded.config.g20_udp.telemetry_publish_port;
  config.g20_udp.remote_address = loaded.config.g20_udp.remote_address;
  config.g20_udp.remote_port = loaded.config.g20_udp.remote_port;
  config.g20_udp.heartbeat = loaded.config.g20_udp.heartbeat;
  config.g20_udp.lease_timeout = loaded.config.g20_udp.lease_timeout;
  config.safe_stop_on_g20_loss = loaded.config.safety.safe_stop_on_g20_loss;
  config.g20_loss_confirm = loaded.config.safety.g20_loss_confirm;
  config.g20_route_loss_confirm =
      loaded.config.safety.g20_route_loss_confirm;

  config.video.path = loaded.config.video.path;
  config.video.target_address = loaded.config.video.target_address;
  config.video.camera1_device = loaded.config.video.camera1_device;
  config.video.camera2_device = loaded.config.video.camera2_device;
  config.video.camera1_udp_port = loaded.config.video.camera1_udp_port;
  config.video.camera2_udp_port = loaded.config.video.camera2_udp_port;
  config.video.camera1_play_url = loaded.config.video.camera1_play_url;
  config.video.camera2_play_url = loaded.config.video.camera2_play_url;
  config.video.width = loaded.config.video.width;
  config.video.height = loaded.config.video.height;
  config.video.fps = loaded.config.video.fps;
  config.video.bps = loaded.config.video.bps;
  config.video.gop = loaded.config.video.gop;
  config.video.qp_init = loaded.config.video.qp_init;
  config.video.auto_start_when_target_known =
      loaded.config.video.auto_start_when_target_known;

  if (loaded.config.g20_udp.remote_address) {
    config.video.target_address = *loaded.config.g20_udp.remote_address;
  }

  std::ifstream input(hardware_config_path);
  if (!input) {
    return {config_error("cannot reopen hardware config"), {}};
  }

  Json root;
  try {
    input >> root;
  } catch (const std::exception& e) {
    return {config_error(std::string("invalid gateway json: ") + e.what()), {}};
  }

  if (has_value(root, "gateway")) {
    const auto& gateway = root.at("gateway");
    config.control_channel.socket_path =
        string_or(gateway, "control_socket_path", "/run/snappy/control.sock");
    config.control_channel.timeout =
        ms_or(gateway, "control_timeout_ms", 200);
    config.loop_period = ms_or(gateway, "loop_period_ms", 20);
    config.status_period = ms_or(gateway, "status_period_ms", 250);
    config.telemetry_period = ms_or(gateway, "telemetry_period_ms", 100);
    config.telemetry_enabled = bool_or(gateway, "telemetry_enabled", true);

    /*
     * 允许在 gateway 段覆盖掉线确认窗口，
     * 现场不重新编译也能调这个阈值。
     */
    config.g20_loss_confirm =
        ms_or(gateway,
              "g20_loss_confirm_ms",
              static_cast<int>(config.g20_loss_confirm.count()));

    /*
     * 掉载波宽限窗口同样允许在 gateway 段覆盖，
     * 现场调阈值不需要重新编译。
     */
    config.g20_route_loss_confirm =
        ms_or(gateway,
              "g20_route_loss_confirm_ms",
              static_cast<int>(config.g20_route_loss_confirm.count()));
  }

  if (has_value(root, "video")) {
    const auto& video = root.at("video");

    config.video.start_command_camera1 =
        string_or(video, "start_command_camera1",
                  config.video.start_command_camera1);
    config.video.start_command_camera2 =
        string_or(video, "start_command_camera2",
                  config.video.start_command_camera2);
    config.video.command_template =
        string_or(video, "command_template", config.video.command_template);
  }

  return {drivers::Result::success("gateway config loaded"), std::move(config)};
}

}  // namespace snappy::gateway