#pragma once
/**
 * @file fake_mqtt_transport.hpp
 * @brief 假 MQTT 传输声明
 *
 * @details
 * - 用内存队列替代真实 MQTT，供无网络环境跑通链路。
 */

#include <deque>
#include <vector>

#include "snappy/gateway/interfaces.hpp"

namespace snappy::simulator {

class FakeMqttTransport final : public gateway::ITransport {
 public:
  [[nodiscard]] control::LinkId link_id() const noexcept override;
  [[nodiscard]] std::size_t payload_limit() const noexcept override;
  [[nodiscard]] bool healthy() const noexcept override;
  std::optional<gateway::InboundMessage> receive() override;
  bool publish(const gateway::PublishedMessage& message) override;

  void enqueue(gateway::InboundMessage message);
  void set_healthy(bool healthy) noexcept;
  [[nodiscard]] const std::vector<gateway::PublishedMessage>& published() const noexcept;

 private:
  bool healthy_{true};
  std::deque<gateway::InboundMessage> inbound_;
  std::vector<gateway::PublishedMessage> published_;
};

}  // namespace snappy::simulator
