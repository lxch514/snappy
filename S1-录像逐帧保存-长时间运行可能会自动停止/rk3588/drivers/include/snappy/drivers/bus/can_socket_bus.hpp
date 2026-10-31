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

#include <mutex>
#include <string>

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
  Result send_once_locked(const CanFrame& frame);

  CanReceiveResult receive_once_locked(
      const CanReceiveOptions& options);

  Result recover_locked(const std::string& reason);
  void close_socket_locked() noexcept;

  CanBusConfig config_;
  mutable std::mutex mutex_;

  int fd_{-1};
  DriverHealth health_{DriverHealth::kUnconfigured};
  int consecutive_failures_{0};
};

}  // namespace snappy::drivers::bus