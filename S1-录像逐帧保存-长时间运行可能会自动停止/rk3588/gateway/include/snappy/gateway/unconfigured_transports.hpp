#pragma once

#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway {

class UnconfiguredG20UdpTransport final : public ITransport {
 public:
  [[nodiscard]] control::LinkId link_id() const noexcept override;
  [[nodiscard]] std::size_t payload_limit() const noexcept override;
  [[nodiscard]] bool healthy() const noexcept override;
  std::optional<InboundMessage> receive() override;
  bool publish(const PublishedMessage& message) override;
};

class UnconfiguredMqttTransport final : public ITransport {
 public:
  [[nodiscard]] control::LinkId link_id() const noexcept override;
  [[nodiscard]] std::size_t payload_limit() const noexcept override;
  [[nodiscard]] bool healthy() const noexcept override;
  std::optional<InboundMessage> receive() override;
  bool publish(const PublishedMessage& message) override;
};

class UnconfiguredSeqpacketChannel final : public IControlChannel {
 public:
  std::optional<std::string> request(const std::string& command_json) override;
  void notify_link_down(control::LinkId link_id) noexcept override;
};

class UnconfiguredG20VideoAdapter final : public IVideoAdapter {
 public:
  [[nodiscard]] bool available() const noexcept override;
  bool start_g20_stream() override;
  bool switch_g20_camera(int active_camera) override;
  void stop() noexcept override;
  [[nodiscard]] VideoStreamState video_state() const override;
};

}  // namespace snappy::gateway