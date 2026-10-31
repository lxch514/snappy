#pragma once

/**
 * @file can_socket_bus.hpp
 * @brief Linux SocketCAN 总线声明
 *
 * @details
 * - 独占一个 SocketCAN fd。
 * - 发送、接收、恢复和关闭由内部锁串行保护。
 * - CAN 故障不会进入 RS485 scheduler。
 */

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "snappy/drivers/bus/can_bus.hpp"

namespace snappy::drivers::bus {

class SocketCanBus final : public ICanFrameBus {
 public:
  explicit SocketCanBus(CanBusConfig config);
  ~SocketCanBus() override;

  SocketCanBus(const SocketCanBus&) = delete;
  SocketCanBus& operator=(const SocketCanBus&) = delete;

  Result start() override;
  Result send(const CanFrame& frame) override;

  CanReceiveResult receive(
      const CanReceiveOptions& options) override;

  void safe_stop() noexcept override;
  void stop() noexcept override;

  [[nodiscard]] DriverHealth health() const noexcept override;
  [[nodiscard]] const CanBusConfig& config() const noexcept;

 private:
  Result configure_interface_locked();
  Result open_socket_locked();
  struct TxQueueUsage {
    bool valid{false};
    std::uint32_t limit{0};
    std::uint32_t used{0};
    std::uint64_t dropped{0};
    std::uint64_t sent{0};
  };

  Result reopen_socket_locked();
  TxQueueUsage tx_queue_usage_locked() const;
  Result send_once_locked(const CanFrame& frame);

  CanReceiveResult receive_once_locked(
      const CanReceiveOptions& options);

  Result recover_locked(
      const std::string& reason,
      bool socket_only_first);

  void close_socket_locked() noexcept;

  void apply_kernel_filter_locked(
      const std::vector<CanReceiveFilter>& filters);

  CanBusConfig config_;
  mutable std::mutex mutex_;

  int fd_{-1};
  DriverHealth health_{DriverHealth::kUnconfigured};
  int consecutive_failures_{0};
  std::vector<CanReceiveFilter> installed_kernel_filters_;
};

}  // namespace snappy::drivers::bus