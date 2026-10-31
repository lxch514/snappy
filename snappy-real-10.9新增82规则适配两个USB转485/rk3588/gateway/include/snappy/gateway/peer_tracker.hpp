#pragma once
/**
 * @file peer_tracker.hpp
 * @brief 遥控器对端在线跟踪声明
 */

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace snappy::gateway {

/* 对端地址、来源端口与最后收包时刻。 */
struct PeerEndpoint {
  std::string address;
  std::uint16_t source_port{0};
  std::chrono::steady_clock::time_point last_seen{};
};

/* 一次静默判定的全部中间量，供判定与留痕共用同一份数字。 */
struct PeerWindowBreakdown {
  bool has_peer{false};                           /* 是否收到过对端报文 */
  std::chrono::milliseconds raw{0};               /* 原始静默（墙钟） */
  std::chrono::milliseconds link_unavailable{0};  /* 本端链路不可用累计 */
  std::chrono::milliseconds resume_grace{0};      /* 链路恢复后的收敛宽限 */
  std::chrono::milliseconds excluded{0};          /* 实际扣除量（已封顶） */
  std::chrono::milliseconds effective{0};         /* raw - excluded */
  std::chrono::milliseconds confirm{0};           /* 本次使用的确认窗口 */
  bool online{true};                              /* effective < confirm */
};

/* 对端在线跟踪：记录最后收包时刻，并按确认窗口判定是否掉线。 */
class PeerTracker {
 public:
  /* 记录一次对端收包。 */
  void observe(std::string address, std::uint16_t source_port);

  /* 清空对端信息与本次静默的扣除量（抖动统计保留）。 */
  void clear() noexcept;

  /* 最近一次对端信息。 */
  [[nodiscard]] std::optional<PeerEndpoint> latest() const;

  /* 是否在 timeout 内收到过报文。 */
  [[nodiscard]] bool online(std::chrono::milliseconds timeout) const;

  /* 是否在确认窗口内收到过报文。 */
  [[nodiscard]] bool online_confirmed(
      std::chrono::milliseconds timeout,
      std::chrono::milliseconds confirm =
          std::chrono::milliseconds{0}) const;

  /* 一次取回判定的全部中间量（判定与留痕同源）。 */
  [[nodiscard]] PeerWindowBreakdown breakdown(
      std::chrono::milliseconds timeout,
      std::chrono::milliseconds confirm =
          std::chrono::milliseconds{0}) const;

  /* 由调用方告知本端发送路径是否可用。 */
  void set_link_available(bool available) noexcept;

  /* 自上次收包以来本端链路累计不可用时长（未封顶，仅留痕）。 */
  [[nodiscard]] std::chrono::milliseconds
      link_unavailable_total() const;

  /* 自上次收包以来"链路已恢复但未收到包"的累计时长（未封顶，仅留痕）。 */
  [[nodiscard]] std::chrono::milliseconds
      resume_grace_total() const;

  /* 距上一次收到该 peer 报文的静默时长（原始墙钟）。 */
  [[nodiscard]] std::chrono::milliseconds silence() const;

  /* 载波抖动统计。 */
  struct BlipStats {
    std::uint64_t count{0};                /* 累计掉载波次数 */
    std::chrono::milliseconds ongoing{0};  /* 当前这一段已持续多久 */
    std::chrono::milliseconds worst{0};    /* 最长的一次 */
    std::chrono::milliseconds total{0};    /* 累计总时长 */
  };

  /* 读取抖动统计。 */
  [[nodiscard]] BlipStats blip_stats() const;

  /* 清零抖动统计。 */
  void reset_blip_stats() noexcept;

 private:
  mutable std::mutex mutex_;
  std::optional<PeerEndpoint> peer_;

  /* 本端链路当前这一段不可用的起点。 */
  std::optional<std::chrono::steady_clock::time_point>
      link_down_since_;

  /* 已结束的链路不可用区间累计。 */
  std::chrono::steady_clock::duration
      link_unavailable_total_{};

  /* 链路恢复后收敛宽限的起点。 */
  std::optional<std::chrono::steady_clock::time_point>
      resume_since_;

  /* 已结束的宽限区间累计。 */
  std::chrono::steady_clock::duration
      resume_total_{};

  /* 抖动统计累计。 */
  std::uint64_t blip_count_{0};
  std::chrono::steady_clock::duration blip_total_{};
  std::chrono::steady_clock::duration blip_worst_{};

  /* 已持锁时使用的链路不可用累计（含进行中段）。 */
  [[nodiscard]] std::chrono::steady_clock::duration
      unavailable_total_locked() const;

  /* 已持锁时使用的宽限累计（含进行中段，单段上限 confirm/3）。 */
  [[nodiscard]] std::chrono::steady_clock::duration
      resume_grace_locked(
          std::chrono::milliseconds confirm) const;
};

}  // namespace snappy::gateway
