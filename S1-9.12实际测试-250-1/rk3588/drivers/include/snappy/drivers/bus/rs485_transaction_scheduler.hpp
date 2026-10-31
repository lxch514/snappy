#pragma once

/**
 * @file rs485_transaction_scheduler.hpp
 * @brief RS485 多端口事务调度器
 *
 * @details
 * - 每个物理串口拥有独立 PortWorker 和有界队列。
 * - 同一个物理端口的 fd 只允许对应 PortWorker 访问。
 * - WCH ACM0~ACM3 共享公平 FIFO 完整事务门禁。
 * - 门禁覆盖清理、发送、接收、验帧和迟到帧恢复。
 * - 数值越小的 priority 越优先。
 * - 只有无副作用读取事务允许自动重试。
 * - 单个设备协议错误不会直接关闭整个物理端口。
 */

#include <chrono>
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "snappy/drivers/bus/rs485_bus.hpp"

namespace snappy::drivers::bus {

class Rs485TransactionScheduler final
    : public IRs485Bus {
 public:
  Rs485TransactionScheduler();

  ~Rs485TransactionScheduler() override;

  Rs485TransactionScheduler(
      const Rs485TransactionScheduler&) = delete;

  Rs485TransactionScheduler& operator=(
      const Rs485TransactionScheduler&) = delete;

  /*
   * 调试模式可以允许物理端口启动失败后继续运行，
   * worker 会按 recovery_delay 低频尝试重新打开。
   */
  void set_allow_missing_ports(
      bool allow) noexcept;

  /*
   * 配置单设备协议失败后的临时隔离。
   */
  void set_device_quarantine(
      bool enabled,
      int after_failures,
      std::chrono::milliseconds duration)
      noexcept;

  /*
   * 注册物理端口。
   * 必须在 start() 前调用。
   */
  Result add_port(
      const Rs485PortConfig& config);

  /*
   * 将逻辑设备绑定到唯一物理端口。
   * 必须在 start() 前调用。
   */
  Result bind_logical_device(
      const Rs485DeviceBinding& binding);

  /*
   * 打开端口并启动每端口 worker。
   */
  Result start() override;

  /*
   * 提交事务并同步等待对应 worker 的结果。
   * 串口 I/O 只发生在 worker 中。
   */
  Rs485Response exchange(
      const Rs485Request& request) override;

  /*
   * 取消尚未执行的周期读取和诊断读取。
   * 当前已经进入底层的事务不能中途截断。
   */
  void safe_stop() noexcept override;

  /*
   * 取消排队任务、唤醒门禁、join worker 并关闭端口。
   */
  void stop() noexcept;

  [[nodiscard]] DriverHealth health()
      const noexcept override;

  [[nodiscard]] std::vector<std::string>
  port_ids() const;

 private:
  struct PortSlot;
  struct PortJob;
  struct DeviceFailureState;
  struct TransactionArbiter;

  void worker_loop(
      const std::shared_ptr<PortSlot>& slot);

  [[nodiscard]] Rs485TransactionResult
  transact_direct(
      PortSlot& slot,
      const Rs485Transaction& transaction);

  [[nodiscard]] bool
  device_quarantined_locked(
      const std::string& logical_device,
      std::chrono::steady_clock::time_point now)
      const;

  void record_device_result_locked(
      const Rs485DeviceBinding& binding,
      const Result& result);

  mutable std::mutex mutex_;

  std::map<
      std::string,
      std::shared_ptr<PortSlot>>
      ports_;

  std::map<
      std::string,
      Rs485DeviceBinding>
      bindings_;

  std::map<
      std::string,
      std::shared_ptr<TransactionArbiter>>
      shared_arbiters_;

  std::map<
      std::string,
      DeviceFailureState>
      device_failures_;

  bool allow_missing_ports_{false};

  bool quarantine_devices_{false};

  int quarantine_after_failures_{2};

  std::chrono::milliseconds
      quarantine_duration_{2000};

  DriverHealth health_{
      DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::bus