#include "snappy/simulator/fake_g20_transport.hpp"
/**
 * @file fake_g20_transport.cpp
 * @brief 假 G20 UDP 传输实现
 *
 * @details
 * - 收发包全部在内存中完成，不打开任何套接字。
 */

#include <utility>

namespace snappy::simulator {

control::LinkId FakeG20Transport::link_id() const noexcept {
  return control::LinkId::kG20Udp;
}
std::size_t FakeG20Transport::payload_limit() const noexcept {
  return gateway::kUdpPayloadLimit;
}
bool FakeG20Transport::healthy() const noexcept { return healthy_; }
std::optional<gateway::InboundMessage> FakeG20Transport::receive() {
  if (inbound_.empty()) {
    return std::nullopt;
  }
  auto message = std::move(inbound_.front());
  inbound_.pop_front();
  return message;
}
bool FakeG20Transport::publish(const gateway::PublishedMessage& message) {
  if (!healthy_) {
    return false;
  }
  published_.push_back(message);
  return true;
}
void FakeG20Transport::enqueue(gateway::InboundMessage message) {
  inbound_.push_back(std::move(message));
}
void FakeG20Transport::set_healthy(bool healthy) noexcept { healthy_ = healthy; }
const std::vector<gateway::PublishedMessage>& FakeG20Transport::published() const noexcept {
  return published_;
}

}  // namespace snappy::simulator
