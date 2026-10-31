/**
 * @file unconfigured_transports.cpp
 * @brief 未配置传输占位实现
 *
 * @details
 * - 所有收发接口返回未配置错误，不产生真实网络行为。
 */
 
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

bool UnconfiguredSeqpacketChannel::notify_link_down(
    control::LinkId) noexcept {
  /*
   * 未配置的控制通道无法投递 link_down。
   * 必须如实返回 false，让调用方知道"没送出去"并持续重试，
   * 而不是假装成功、把一次真实的链路掉线吞掉。
   */
  return false;
}

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