#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace snappy::gateway {

struct PeerEndpoint {
  std::string address;
  std::uint16_t source_port{0};
  std::chrono::steady_clock::time_point last_seen{};
};

class PeerTracker {
 public:
  void observe(std::string address, std::uint16_t source_port);
  void clear() noexcept;

  [[nodiscard]] std::optional<PeerEndpoint> latest() const;
  [[nodiscard]] bool online(std::chrono::milliseconds timeout) const;

 private:
  mutable std::mutex mutex_;
  std::optional<PeerEndpoint> peer_;
};

}  // namespace snappy::gateway