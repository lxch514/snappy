#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "snappy/control/hardware_config.hpp"
#include "snappy/drivers/result.hpp"
#include "snappy/gateway/control_channel.hpp"
#include "snappy/gateway/g20/g20_video.hpp"

namespace snappy::gateway {

struct G20UdpRuntimeConfig {
  std::string listen_address{"0.0.0.0"};
  std::uint16_t command_port{14550};
  std::uint16_t status_port{14551};
  std::uint16_t telemetry_port{14552};
  std::optional<std::string> remote_address;
  std::optional<std::uint16_t> remote_port;
  std::chrono::milliseconds heartbeat{250};
  std::chrono::milliseconds lease_timeout{750};
};

struct GatewayRuntimeConfig {
  std::string robot_id{"snappy-001"};
  std::string hardware_config_path{"rk3588/config/hardware.snappy.json"};

  ControlChannelConfig control_channel;
  G20UdpRuntimeConfig g20_udp;
  g20::G20VideoConfig video;

  std::chrono::milliseconds loop_period{20};
  std::chrono::milliseconds status_period{250};
  std::chrono::milliseconds telemetry_period{100};
  bool telemetry_enabled{true};
  bool safe_stop_on_g20_loss{true};
};

struct GatewayConfigLoadResult {
  drivers::Result result;
  GatewayRuntimeConfig config;
};

[[nodiscard]] GatewayConfigLoadResult load_gateway_config(
    const std::string& hardware_config_path);

}  // namespace snappy::gateway