#pragma once
/**
 * @file fake_hardware.hpp
 * @brief 假硬件后端声明
 *
 * @details
 * - 提供假的 CAN、RS485、继电器和传感器实现。
 * - 支持注入失败与延迟，用于验证故障隔离行为。
 */
 
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"

namespace snappy::simulator {

class FakeCanBus final : public drivers::ICanBus {
 public:
  drivers::Result start() override;
  drivers::Result send(const drivers::CanFrame& frame) override;
  void safe_stop() noexcept override;
  [[nodiscard]] drivers::DriverHealth health() const noexcept override;

  [[nodiscard]] const std::vector<drivers::CanFrame>& sent_frames() const noexcept;
  [[nodiscard]] bool safe_stop_called() const noexcept;
  void set_send_failure(drivers::ErrorCode code, std::string detail);
  void clear_send_failure() noexcept;

 private:
  drivers::DriverHealth health_{drivers::DriverHealth::kStopped};
  std::vector<drivers::CanFrame> sent_frames_;
  std::optional<drivers::Result> send_failure_;
  bool safe_stop_called_{false};
};

class FakeRs485Bus final : public drivers::IRs485Bus {
 public:
  drivers::Result start() override;
  drivers::Rs485Response exchange(const drivers::Rs485Request& request) override;
  void safe_stop() noexcept override;
  [[nodiscard]] drivers::DriverHealth health() const noexcept override;

  void set_response(std::string logical_device, std::vector<std::uint8_t> response);
  void set_failure(std::string logical_device,
                   drivers::ErrorCode code,
                   std::string detail);
  void clear_failure(const std::string& logical_device);
  [[nodiscard]] const std::vector<drivers::Rs485Request>& requests() const noexcept;
  [[nodiscard]] bool safe_stop_called() const noexcept;

 private:
  drivers::DriverHealth health_{drivers::DriverHealth::kStopped};
  std::map<std::string, std::vector<std::uint8_t>> responses_;
  std::map<std::string, drivers::Result> failures_;
  std::vector<drivers::Rs485Request> requests_;
  bool safe_stop_called_{false};
};

class FakeRelayBank final : public drivers::IRelayBank {
 public:
  drivers::Result start() override;
  drivers::Result set(const std::string& logical_channel,
                      drivers::RelayState state) override;
  void deenergize_all() noexcept override;
  [[nodiscard]] drivers::DriverHealth health() const noexcept override;

  [[nodiscard]] drivers::RelayState state_of(const std::string& logical_channel) const;
  [[nodiscard]] bool deenergize_all_called() const noexcept;
  void set_interlock(std::string logical_channel, bool active);

 private:
  drivers::DriverHealth health_{drivers::DriverHealth::kStopped};
  std::map<std::string, drivers::RelayState> states_;
  std::map<std::string, bool> interlocks_;
  bool deenergize_all_called_{false};
};

class FakeSensorSuite final : public drivers::ISensorSuite {
 public:
  drivers::Result start() override;
  [[nodiscard]] std::vector<drivers::SensorSample> read_all() override;
  void stop() noexcept override;
  [[nodiscard]] drivers::DriverHealth health() const noexcept override;

  void set_samples(std::vector<drivers::SensorSample> samples);

 private:
  drivers::DriverHealth health_{drivers::DriverHealth::kStopped};
  std::vector<drivers::SensorSample> samples_;
};

}  // namespace snappy::simulator
