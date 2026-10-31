#pragma once

/**
 * @file rs485_port.hpp
 * @brief Linux RS485 物理串口
 *
 * @details
 * - 一个 Rs485Port 只拥有一个串口 fd。
 * - open、close 和 transact 使用同一状态锁。
 * - 上层 PortWorker 是正常运行时的唯一事务调用者。
 * - 本类不处理业务状态，也不直接执行设备隔离。
 */

#include <mutex>

#include "snappy/drivers/bus/rs485_bus.hpp"

namespace snappy::drivers::bus {

class Rs485Port final
    : public IRs485Port {
 public:
  explicit Rs485Port(
      Rs485PortConfig config);

  ~Rs485Port() override;

  Rs485Port(
      const Rs485Port&) = delete;

  Rs485Port& operator=(
      const Rs485Port&) = delete;

  Result open() override;

  void close() noexcept override;

  Rs485TransactionResult transact(
      const Rs485Transaction& transaction) override;

  [[nodiscard]] const Rs485PortConfig&
  config() const noexcept override;

  [[nodiscard]] DriverHealth health()
      const noexcept override;

 private:
  Rs485PortConfig config_;

  mutable std::mutex mutex_;

  int fd_{-1};

  DriverHealth health_{
      DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::bus