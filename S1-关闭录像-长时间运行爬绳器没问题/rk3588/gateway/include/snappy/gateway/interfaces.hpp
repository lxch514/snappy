#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "snappy/control/model.hpp"

namespace snappy::gateway {

constexpr std::size_t kUdpPayloadLimit = 1200;
constexpr std::size_t kStreamPayloadLimit = 64 * 1024;

struct InboundMessage {
  std::string topic;
  std::string payload;
};

struct PublishedMessage {
  std::string topic;
  std::string payload;
  int qos{0};
  bool retain{false};
};

class ITransport {
 public:
  virtual ~ITransport() = default;
  [[nodiscard]] virtual control::LinkId link_id() const noexcept = 0;
  [[nodiscard]] virtual std::size_t payload_limit() const noexcept = 0;
  [[nodiscard]] virtual bool healthy() const noexcept = 0;
  virtual std::optional<InboundMessage> receive() = 0;
  virtual bool publish(const PublishedMessage& message) = 0;
};

class IControlChannel {
 public:
  virtual ~IControlChannel() = default;
  virtual std::optional<std::string> request(const std::string& command_json) = 0;
  virtual void notify_link_down(control::LinkId link_id) noexcept = 0;
};

struct VideoStreamState {
  bool running{false};
  int active_camera{1};
  std::string active_url;
  std::string camera1_url;
  std::string camera2_url;
  std::string target_address;
  std::uint16_t camera1_port{5600};
  std::uint16_t camera2_port{5602};
  std::string detail;
};

class IVideoAdapter {
 public:
  virtual ~IVideoAdapter() = default;
  [[nodiscard]] virtual bool available() const noexcept = 0;
  virtual bool start_g20_stream() = 0;
  virtual bool switch_g20_camera(int active_camera) = 0;
  virtual void stop() noexcept = 0;
  [[nodiscard]] virtual VideoStreamState video_state() const = 0;
};

}  // namespace snappy::gateway