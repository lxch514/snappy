#include "snappy/simulator/fake_mqtt_transport.hpp"
/**
 * @file fake_mqtt_transport.cpp
 * @brief 假 MQTT 传输实现
 *
 * @details
 * - 收发全部在内存中完成，不建立任何连接。
 */

#include <utility>

namespace snappy::simulator {

control::LinkId FakeMqttTransport::link_id() const noexcept {
  return control::LinkId::kEc20Mqtt;
}
std::size_t FakeMqttTransport::payload_limit() const noexcept {
  return gateway::kStreamPayloadLimit;
}
bool FakeMqttTransport::healthy() const noexcept { return healthy_; }
std::optional<gateway::InboundMessage> FakeMqttTransport::receive() {
  if (inbound_.empty()) {
    return std::nullopt;
  }
  auto message = std::move(inbound_.front());
  inbound_.pop_front();
  return message;
}
bool FakeMqttTransport::publish(const gateway::PublishedMessage& message) {
  if (!healthy_) {
    return false;
  }
  published_.push_back(message);
  return true;
}
void FakeMqttTransport::enqueue(gateway::InboundMessage message) {
  inbound_.push_back(std::move(message));
}
void FakeMqttTransport::set_healthy(bool healthy) noexcept { healthy_ = healthy; }
const std::vector<gateway::PublishedMessage>& FakeMqttTransport::published() const noexcept {
  return published_;
}

}  // namespace snappy::simulator
