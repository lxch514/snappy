#include "snappy/gateway/peer_tracker.hpp"

#include <utility>

namespace snappy::gateway {

void PeerTracker::observe(std::string address, std::uint16_t source_port) {
  std::lock_guard<std::mutex> lock(mutex_);
  peer_ = PeerEndpoint{
      std::move(address),
      source_port,
      std::chrono::steady_clock::now(),
  };
}

void PeerTracker::clear() noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    peer_.reset();
  } catch (...) {
  }
}

std::optional<PeerEndpoint> PeerTracker::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peer_;
}

bool PeerTracker::online(std::chrono::milliseconds timeout) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!peer_) {
    return false;
  }
  return std::chrono::steady_clock::now() - peer_->last_seen <= timeout;
}

}  // namespace snappy::gateway