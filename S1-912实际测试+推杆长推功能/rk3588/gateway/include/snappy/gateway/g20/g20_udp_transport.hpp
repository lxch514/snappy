#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "snappy/gateway/gateway_config.hpp"
#include "snappy/gateway/interfaces.hpp"
#include "snappy/gateway/peer_tracker.hpp"

namespace snappy::gateway::g20 {

class G20UdpTransport final : public ITransport {
 public:
  G20UdpTransport(G20UdpRuntimeConfig config, PeerTracker& peer);
  ~G20UdpTransport();

  [[nodiscard]] bool start();
  void stop() noexcept;

  [[nodiscard]] control::LinkId link_id() const noexcept override;
  [[nodiscard]] std::size_t payload_limit() const noexcept override;
  [[nodiscard]] bool healthy() const noexcept override;
  std::optional<InboundMessage> receive() override;
  bool publish(const PublishedMessage& message) override;

 private:
  [[nodiscard]] std::optional<std::string> target_address() const;
  [[nodiscard]] std::uint16_t publish_port(const PublishedMessage& message) const;

  G20UdpRuntimeConfig config_;
  PeerTracker& peer_;
  int fd_{-1};
  bool started_{false};
};

}  // namespace snappy::gateway::g20