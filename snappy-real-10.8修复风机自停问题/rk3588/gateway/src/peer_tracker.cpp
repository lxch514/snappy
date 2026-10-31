/**
 * @file peer_tracker.cpp
 * @brief 遥控器对端在线跟踪实现
 */

#include "snappy/gateway/peer_tracker.hpp"

#include <algorithm>
#include <utility>

namespace snappy::gateway {

/* 记录一次对端收包，并清零本次静默的两段扣除量。 */
void PeerTracker::observe(std::string address, std::uint16_t source_port) {
  std::lock_guard<std::mutex> lock(mutex_);

  peer_ = PeerEndpoint{
      std::move(address),
      source_port,
      std::chrono::steady_clock::now(),
  };

  link_down_since_.reset();
  link_unavailable_total_ = std::chrono::steady_clock::duration{};
  resume_since_.reset();
  resume_total_ = std::chrono::steady_clock::duration{};
}

/* 清空对端信息与扣除量；抖动统计与对端身份无关，保留。 */
void PeerTracker::clear() noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);

    peer_.reset();
    link_down_since_.reset();
    link_unavailable_total_ = std::chrono::steady_clock::duration{};
    resume_since_.reset();
    resume_total_ = std::chrono::steady_clock::duration{};
  } catch (...) {
  }
}

/* 最近一次对端信息。 */
std::optional<PeerEndpoint> PeerTracker::latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return peer_;
}

/* 距上一次收到该 peer 报文的原始墙钟静默。 */
std::chrono::milliseconds PeerTracker::silence() const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!peer_) {
    return std::chrono::milliseconds{0};
  }

  const auto age = std::chrono::steady_clock::now() - peer_->last_seen;

  if (age.count() <= 0) {
    return std::chrono::milliseconds{0};
  }

  return std::chrono::duration_cast<std::chrono::milliseconds>(age);
}

/* 链路不可用累计（含正在进行的这一段）。 */
std::chrono::steady_clock::duration
PeerTracker::unavailable_total_locked() const {
  auto total = link_unavailable_total_;

  if (link_down_since_.has_value()) {
    const auto ongoing =
        std::chrono::steady_clock::now() - *link_down_since_;

    if (ongoing.count() > 0) {
      total += ongoing;
    }
  }

  return total;
}

/* 收敛宽限累计（含正在进行的这一段，单段按 confirm/3 封顶）。 */
std::chrono::steady_clock::duration
PeerTracker::resume_grace_locked(
    std::chrono::milliseconds confirm) const {
  auto total = resume_total_;

  if (resume_since_.has_value()) {
    const auto ongoing =
        std::chrono::steady_clock::now() - *resume_since_;

    if (ongoing.count() > 0) {
      total += ongoing;
    }
  }

  if (confirm.count() > 0) {
    const auto cap =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            confirm / 3);

    if (total > cap) {
      total = cap;
    }
  }

  return total;
}

/* 由调用方告知本端发送路径是否可用，并维护两段扣除量与抖动统计。 */
void PeerTracker::set_link_available(bool available) noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = std::chrono::steady_clock::now();

    if (available) {
      /* 链路回来：结账掉载波段，并开始新一轮收敛宽限。 */
      if (link_down_since_.has_value()) {
        const auto ended = now - *link_down_since_;

        if (ended.count() > 0) {
          link_unavailable_total_ += ended;
          blip_total_ += ended;

          if (ended > blip_worst_) {
            blip_worst_ = ended;
          }
        }

        link_down_since_.reset();
        resume_since_ = now;
      }

      return;
    }

    /* 链路断开：先结账还在进行的宽限段，再记本段的起点。 */
    if (resume_since_.has_value()) {
      const auto held = now - *resume_since_;

      if (held.count() > 0) {
        resume_total_ += held;
      }

      resume_since_.reset();
    }

    if (!link_down_since_.has_value()) {
      link_down_since_ = now;
      ++blip_count_;
    }
  } catch (...) {
  }
}

/* 原始链路不可用累计（未封顶）。 */
std::chrono::milliseconds
PeerTracker::link_unavailable_total() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return std::chrono::duration_cast<std::chrono::milliseconds>(
      unavailable_total_locked());
}

/* 原始收敛宽限累计（未封顶）。 */
std::chrono::milliseconds
PeerTracker::resume_grace_total() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return std::chrono::duration_cast<std::chrono::milliseconds>(
      resume_grace_locked(std::chrono::milliseconds{0}));
}

/* 载波抖动统计。 */
PeerTracker::BlipStats PeerTracker::blip_stats() const {
  std::lock_guard<std::mutex> lock(mutex_);

  BlipStats stats;
  stats.count = blip_count_;

  stats.worst =
      std::chrono::duration_cast<std::chrono::milliseconds>(blip_worst_);

  stats.total =
      std::chrono::duration_cast<std::chrono::milliseconds>(blip_total_);

  if (link_down_since_.has_value()) {
    stats.ongoing =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - *link_down_since_);
  }

  return stats;
}

/* 清零抖动统计。 */
void PeerTracker::reset_blip_stats() noexcept {
  try {
    std::lock_guard<std::mutex> lock(mutex_);

    blip_count_ = 0;
    blip_total_ = std::chrono::steady_clock::duration{};
    blip_worst_ = std::chrono::steady_clock::duration{};
  } catch (...) {
  }
}

/* 是否在 timeout 内收到过报文（不做扣除）。 */
bool PeerTracker::online(std::chrono::milliseconds timeout) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!peer_) {
    return false;
  }
  return std::chrono::steady_clock::now() - peer_->last_seen <= timeout;
}

/* 一次取回判定的全部中间量。 */
PeerWindowBreakdown PeerTracker::breakdown(
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds confirm) const {
  PeerWindowBreakdown window;

  if (timeout.count() <= 0) {
    window.online = false;
    return window;
  }

  /* 调用方给不出比租约更长的窗口时，按 2 倍租约（至少 1.5s）兜底。 */
  if (confirm <= timeout) {
    confirm = std::max(
        timeout * 2,
        std::chrono::milliseconds(1500));
  }

  window.confirm = confirm;

  std::lock_guard<std::mutex> lock(mutex_);

  if (!peer_) {
    window.online = false;
    return window;
  }

  window.has_peer = true;

  window.raw =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - peer_->last_seen);

  window.link_unavailable =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          unavailable_total_locked());

  window.resume_grace =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          resume_grace_locked(confirm));

  /* 两段扣除量合计被一个确认窗口封顶。 */
  window.excluded = std::min(
      window.link_unavailable + window.resume_grace,
      confirm);

  window.effective = window.raw - window.excluded;
  window.online = window.effective < confirm;

  return window;
}

/* 是否在确认窗口内收到过报文（复用 breakdown，判定与留痕同源）。 */
bool PeerTracker::online_confirmed(
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds confirm) const {
  return breakdown(timeout, confirm).online;
}

}  // namespace snappy::gateway
