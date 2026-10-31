#pragma once

/**
 * @file control_channel.hpp
 * @brief gateway 到 control 的 Unix SEQPACKET 通道
 *
 * @details
 *
 * - 遥控命令和周期快照使用独立请求门禁。
 * - 慢快照请求不会使遥控命令 try_lock 失败。
 * - 每个请求具有统一的 connect/send/recv deadline。
 * - IPC 失败只影响当前请求。
 */

#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>
#include <string>

#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway {

struct ControlChannelConfig {
  std::string socket_path{
      "/run/snappy/control.sock"};

  std::string robot_id{
      "snappy-001"};

  // 遥控命令等待实际 ACK 的最大时间。
  std::chrono::milliseconds timeout{200};

  // 周期状态和遥测查询使用更短超时。
  std::chrono::milliseconds snapshot_timeout{80};

  // 链路掉线通知只允许非常短的同步等待。
  std::chrono::milliseconds link_down_timeout{40};

  std::size_t max_payload_bytes{
      kStreamPayloadLimit};
};

class SeqpacketControlChannel final
    : public IControlChannel {
 public:
  explicit SeqpacketControlChannel(
      ControlChannelConfig config = {});

  /*
   * 遥控命令路径。
   * 该接口仍等待 control 返回真实 ACK。
   */
  std::optional<std::string> request(
      const std::string& command_json) override;

  /*
   * 周期快照专用路径。
   * 与遥控命令使用不同互斥量。
   */
  std::optional<std::string> request_snapshot(
      const std::string& command_json);

  void notify_link_down(
      control::LinkId link_id) noexcept override;

 private:
  std::optional<std::string> perform_request(
      const std::string& command_json,
      std::chrono::milliseconds timeout);

  ControlChannelConfig config_;

  std::mutex command_mutex_;
  std::mutex snapshot_mutex_;
};

}  // namespace snappy::gateway