#pragma once
/**
 * @file g20_udp_transport.hpp
 * @brief G20 UDP 传输声明
 *
 * @details
 * - 绑定 UDP 端口收发命令、状态与遥测报文。
 * - 按对端存活情况判定链路健康，掉载波时立即标记不可达。
 * - 对端静默与内核判定不可达使用不同的确认窗口。
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "snappy/gateway/gateway_config.hpp"
#include "snappy/gateway/interfaces.hpp"
#include "snappy/gateway/peer_tracker.hpp"

namespace snappy::gateway::g20 {

/*
 * 连续多少次"内核判定不可达"的发送失败才算链路故障。
 */
constexpr int kUnreachableConfirmCount = 3;

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

  [[nodiscard]] bool unreachable() const noexcept override;

  [[nodiscard]] long long unreachable_send_count() const noexcept;
  [[nodiscard]] long long oversize_publish_count() const noexcept;

  /*
   * 接收路径连续失败（fd 失效 / recvfrom 致命错误）时，
   * 按此间隔重开 socket。0 表示不做节流。
   */
  static constexpr auto kSocketRestartBackoff =
      std::chrono::milliseconds(1000);

 private:
  [[nodiscard]] std::optional<std::string> target_address() const;
  [[nodiscard]] std::uint16_t publish_port(const PublishedMessage& message) const;

  /*
   * 关闭并重新创建 socket。
   */
  bool restart_socket();

  G20UdpRuntimeConfig config_;
  PeerTracker& peer_;

  /*
   * fd_/started_ 会被接收线程和主循环同时访问，必须是原子的。
   */
  std::atomic<int> fd_{-1};
  std::atomic<bool> started_{false};

  std::atomic<int> consecutive_unreachable_{0};
  std::atomic<long long> unreachable_send_count_{0};
  std::atomic<long long> oversize_publish_count_{0};

  std::chrono::steady_clock::time_point last_restart_attempt_{};
};

}  // namespace snappy::gateway::g20
