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

  /*
   * 原始在线判定，语义与原来完全一致。
   * status 上报仍然使用这个值，字段含义不变。
   */
  [[nodiscard]] bool online(
      std::chrono::milliseconds timeout) const;

  /*
   * 带持续确认的在线判定，只给“链路掉线下降沿”使用。
   *
   * age = 从收到的最后一个包开始算的持续静默时间：
   * - age <  timeout           -> true
   * - timeout <= age < confirm -> true（容忍区间）
   * - age >= confirm           -> false
   *
   * confirm 必须大于 timeout，否则退化成原来的单次采样。
   * confirm == 0 时按 max(2 * timeout, 1500ms) 计算。
   */
  [[nodiscard]] bool online_confirmed(
      std::chrono::milliseconds timeout,
      std::chrono::milliseconds confirm =
          std::chrono::milliseconds{0}) const;

 private:
  mutable std::mutex mutex_;

  std::optional<PeerEndpoint> peer_;
};

}  // namespace snappy::gateway
