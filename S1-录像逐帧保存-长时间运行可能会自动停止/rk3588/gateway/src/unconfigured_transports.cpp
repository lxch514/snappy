#include "snappy/gateway/unconfigured_transports.hpp"

namespace snappy::gateway {

control::LinkId UnconfiguredG20UdpTransport::link_id() const noexcept {
  return control::LinkId::kG20Udp;
}

std::size_t UnconfiguredG20UdpTransport::payload_limit() const noexcept {
  return kUdpPayloadLimit;
}

bool UnconfiguredG20UdpTransport::healthy() const noexcept {
  return false;
}

std::optional<InboundMessage> UnconfiguredG20UdpTransport::receive() {
  return std::nullopt;
}

bool UnconfiguredG20UdpTransport::publish(const PublishedMessage&) {
  return false;
}

control::LinkId UnconfiguredMqttTransport::link_id() const noexcept {
  return control::LinkId::kEc20Mqtt;
}

std::size_t UnconfiguredMqttTransport::payload_limit() const noexcept {
  return kStreamPayloadLimit;
}

bool UnconfiguredMqttTransport::healthy() const noexcept {
  return false;
}

std::optional<InboundMessage> UnconfiguredMqttTransport::receive() {
  return std::nullopt;
}

bool UnconfiguredMqttTransport::publish(const PublishedMessage&) {
  return false;
}

std::optional<std::string> UnconfiguredSeqpacketChannel::request(
    const std::string&) {
  return std::nullopt;
}

void UnconfiguredSeqpacketChannel::notify_link_down(control::LinkId) noexcept {}

bool UnconfiguredG20VideoAdapter::available() const noexcept {
  return false;
}

bool UnconfiguredG20VideoAdapter::start_g20_stream() {
  return false;
}

bool UnconfiguredG20VideoAdapter::switch_g20_camera(int) {
  return false;
}

void UnconfiguredG20VideoAdapter::stop() noexcept {}

VideoStreamState UnconfiguredG20VideoAdapter::video_state() const {
  VideoStreamState state;
  state.running = false;
  state.active_camera = 1;
  state.detail = "video adapter unconfigured";
  return state;
}

}  // namespace snappy::gateway