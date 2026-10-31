#pragma once
/**
 * @file g20_command_router.hpp
 * @brief G20 命令路由与可靠投递声明
 *
 * @details
 * - 把上层命令翻译成 G20 能识别的动作，并按优先级排队下发。
 * - 维护可靠投递记录，按 ACK 结果推进 pending/accepted/succeeded/failed 状态。
 * - 记录每条命令的重试次数与超时，供上层查询投递进度。
 */
 
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "snappy/gateway/g20/g20_legacy_packet_adapter.hpp"
#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway::g20 {

enum class ReliableDeliveryState {
  kPending,
  kAccepted,
  kSucceeded,
  kFailedRetryable,
  kFailedPermanent,
};

struct ReliableDeliveryRecord {
  std::uint64_t sequence{0};
  std::chrono::steady_clock::time_point updated_at{};
  ReliableDeliveryState state{
      ReliableDeliveryState::kPending};
  std::string payload_hash;
  std::vector<nlohmann::json> adapted_commands;
  bool commands_cached{false};
  std::string ack_status;
  bool retryable{false};
  std::string detail;
};

struct RoutedCommandResult {
  bool processed{false};
  bool control_succeeded{false};
  bool execution_succeeded{false};
  bool execution_pending{false};
  bool ack_published{false};
  bool retryable{false};

  /*
   * 兼容现有测试和外部调用方。
   * 新逻辑不能再使用该字段判断 control 是否成功。
   */
  bool forwarded{false};

  std::string detail;

  RoutedCommandResult() = default;

  RoutedCommandResult(
      bool processed_value,
      bool forwarded_value,
      std::string detail_value)
      : processed(processed_value),
        control_succeeded(forwarded_value),
        execution_succeeded(forwarded_value),
        ack_published(forwarded_value),
        forwarded(forwarded_value),
        detail(std::move(detail_value)) {}
};

class G20CommandRouter {
 public:
  G20CommandRouter(
      std::string robot_id,
      IControlChannel& control_channel,
      IVideoAdapter* video_adapter);

  void set_telemetry_control(
      std::function<void(bool)> callback);

  RoutedCommandResult route(
      const InboundMessage& inbound,
      ITransport& reply_transport);

  RoutedCommandResult tick(
      ITransport& reply_transport);

  bool publish_video_state(
      ITransport& transport) const;

 private:
  RoutedCommandResult route_command_json(
      const nlohmann::json& command,
      ITransport& reply_transport);

  RoutedCommandResult route_command_batch(
      const std::vector<nlohmann::json>& commands,
      ITransport& reply_transport,
      std::size_t max_commands,
      const std::string& handled_detail);

  bool publish_ack(
      ITransport& transport,
      const nlohmann::json& command,
      const std::string& status,
      const std::string& detail,
      bool retryable = false) const;

  void prune_reliable_channel_deliveries();

  std::string robot_id_;
  IControlChannel& control_channel_;
  IVideoAdapter* video_adapter_{nullptr};

  std::function<void(bool)> telemetry_control_;
  G20LegacyPacketAdapter legacy_;

  std::unordered_map<
      std::string,
      ReliableDeliveryRecord>
      reliable_channel_deliveries_;
};

}  // namespace snappy::gateway::g20