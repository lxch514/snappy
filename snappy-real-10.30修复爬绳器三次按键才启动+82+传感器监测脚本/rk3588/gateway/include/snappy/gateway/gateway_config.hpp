#pragma once
/**
 * @file gateway_config.hpp
 * @brief 网关运行配置声明
 *
 * @details
 * - 描述 G20 UDP、控制通道、状态与遥测周期等运行参数。
 * - 加载结果同时携带错误码与说明，便于启动阶段定位问题。
 */
 
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

  /*
   * 「G20 链路是否掉线」的确认窗口。
   *
   * 只要窗口内收到过任意一个 G20 报文，就认为链路正常。
   *
   * 旧实现把窗口写死为 max(2 × lease_timeout, 1500ms) = 1500ms。
   * 2026-09-24 现场日志证明：多执行器同时运行时，该窗口会被
   * 1.5s 量级的正常抖动误触发——10 分钟内 6 次下发
   * control.link_down，每次都导致整机安全停止。
   *
   * 现在改为可配置（对应 safety 段的 g20_loss_confirm_ms），
   * 默认 8s，并且不允许小于 2 × lease_timeout。
   */
  std::chrono::milliseconds g20_loss_confirm{8000};

  /*
   * 掉载波后的有界宽限窗口（对应 safety 段的
   * g20_route_loss_confirm_ms，默认 3s）。
   *
   * 掉载波后重新自协商固定要 ~2.05 秒，这段时间的静默远小于
   * g20_loss_confirm，本来不该停机。所以 ENETUNREACH 只启动计时，
   * 必须持续超过本窗口才判定掉线：
   *   - 2 秒级自愈闪断 -> 容忍，不停机；
   *   - 真实断链       -> 3 秒内停机（仍快于 8 秒静默判据）。
   */
  std::chrono::milliseconds g20_route_loss_confirm{3000};
};

struct GatewayConfigLoadResult {
  drivers::Result result;
  GatewayRuntimeConfig config;
};

[[nodiscard]] GatewayConfigLoadResult load_gateway_config(
    const std::string& hardware_config_path);

}  // namespace snappy::gateway