#pragma once
/**
 * @file unconfigured_drivers.hpp
 * @brief 未配置硬件占位实现声明
 *
 * @details
 * - 保留 CAN、RS485、继电器、传感器的抽象边界可编译。
 * - 在参数确认前拒绝一切真实硬件访问，避免误动作。
 */

#include "snappy/drivers/interfaces.hpp"

namespace snappy::drivers {

// These placeholders are deliberate. They keep the real backend boundary
// compilable while refusing all hardware I/O until verified parameters exist.
class UnconfiguredCanBus final : public ICanBus {
 public:
  Result start() override;
  Result send(const CanFrame& frame) override;
  void safe_stop() noexcept override;
  [[nodiscard]] DriverHealth health() const noexcept override;
};

class UnconfiguredRs485Bus final : public IRs485Bus {
 public:
  Result start() override;
  Rs485Response exchange(const Rs485Request& request) override;
  void safe_stop() noexcept override;
  [[nodiscard]] DriverHealth health() const noexcept override;
};

class UnconfiguredRelayBank final : public IRelayBank {
 public:
  Result start() override;
  Result set(const std::string& logical_channel, RelayState state) override;
  void deenergize_all() noexcept override;
  [[nodiscard]] DriverHealth health() const noexcept override;
};

class UnconfiguredSensorSuite final : public ISensorSuite {
 public:
  Result start() override;
  [[nodiscard]] std::vector<SensorSample> read_all() override;
  void stop() noexcept override;
  [[nodiscard]] DriverHealth health() const noexcept override;
};

}  // namespace snappy::drivers
