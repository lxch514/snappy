#pragma once

/**
 * @file propulsion_fan.hpp
 * @brief Hobbywing X6 Plus 双推进风扇驱动声明
 *
 * @details
 * - start() 启动独立 CAN worker。
 * - worker 自动完成两台 ESC 初始化，不依赖遥控命令。
 * - 初始化顺序：Select CAN -> Ready -> 两秒 50 Hz 零油门。
 * - 初始化完成后以 50 Hz 持续发送缓存目标油门。
 * - 遥控和屏幕命令只修改目标缓存，不直接执行 CAN I/O。
 * - CAN 故障后目标速度清零，后台低频恢复，不自动恢复旧运动。
 */

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "snappy/drivers/bus/can_bus.hpp"
#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/propulsion_fan_can.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kPropulsionFanDeviceId =
    "propulsion_fan";

inline constexpr std::uint8_t
    kPropulsionFanLeftNodeId = 0x01;
inline constexpr std::uint8_t
    kPropulsionFanRightNodeId = 0x02;

inline constexpr std::size_t
    kPropulsionFanLeftChannelIndex = 0;
inline constexpr std::size_t
    kPropulsionFanRightChannelIndex = 1;

struct PropulsionFanControllerConfig {
  std::string device_id{
      kPropulsionFanDeviceId};

  protocols::PropulsionFanCanProfile profile{};

  std::uint8_t left_node_id{
      kPropulsionFanLeftNodeId};

  std::uint8_t right_node_id{
      kPropulsionFanRightNodeId};

  std::size_t left_channel_index{
      kPropulsionFanLeftChannelIndex};

  std::size_t right_channel_index{
      kPropulsionFanRightChannelIndex};

  std::uint16_t max_throttle_raw{
      protocols::kHobbywingMaxThrottleRaw};

  std::chrono::milliseconds command_period{20};
  std::chrono::milliseconds command_gap{100};
  std::chrono::milliseconds zero_throttle_hold{2000};
  std::chrono::milliseconds retry_backoff{1000};
  std::chrono::milliseconds startup_wait{4000};
};

struct PropulsionFanEscSnapshot {
  std::uint8_t node_id{0};
  bool online{false};

  std::optional<std::uint16_t> rpm;
  std::optional<std::uint16_t> throttle_raw;
  std::optional<std::uint16_t> status;

  std::optional<double> voltage_v;
  std::optional<double> current_a;
  std::optional<double> temperature_celsius;

  std::optional<double> mos_temperature_celsius;
  std::optional<double> capacitor_temperature_celsius;
  std::optional<double> motor_temperature_celsius;

  std::uint64_t age_ms{0};
};

struct PropulsionFanSnapshot {
  bool worker_running{false};
  bool initialized{false};
  bool ready_enabled{false};

  std::array<std::uint16_t, 4> target_raw{};
  std::array<std::uint16_t, 4> last_sent_raw{};

  PropulsionFanEscSnapshot left;
  PropulsionFanEscSnapshot right;

  Result last_result{
      Result::failure(
          ErrorCode::kConfigRequired,
          "propulsion fan has not started")};
};

[[nodiscard]] PropulsionFanControllerConfig
make_propulsion_fan_config();

class PropulsionFanController {
 public:
  PropulsionFanController(
      ICanBus& can_bus,
      PropulsionFanControllerConfig config);

  ~PropulsionFanController();

  PropulsionFanController(
      const PropulsionFanController&) = delete;

  PropulsionFanController& operator=(
      const PropulsionFanController&) = delete;

  Result start();

  /*
   * 兼容旧命令。
   * 初始化已由 start()/worker 完成，本函数不再重新初始化。
   */
  Result enter_can_standby();

  Result select_can_for_left_right();
  Result enable_throttle();
  Result disable_throttle();

  Result set_all_raw(
      const std::array<std::uint16_t, 4>& throttle_raw);

  Result set_left_right_raw(
      std::uint16_t left_raw,
      std::uint16_t right_raw);

  Result set_channel_raw(
      std::size_t channel_index,
      std::uint16_t throttle_raw);

  Result stop_all();

  void safe_stop() noexcept;
  void stop() noexcept;

  [[nodiscard]] DriverHealth health() const noexcept;

  [[nodiscard]] std::array<std::uint16_t, 4>
  last_command() const noexcept;

  [[nodiscard]] bool throttle_enabled() const noexcept;

  [[nodiscard]] PropulsionFanSnapshot snapshot() const;

 private:
  struct EscCache {
    PropulsionFanEscSnapshot snapshot;
    std::chrono::steady_clock::time_point received_at{};
  };

  [[nodiscard]] bool left_enabled() const noexcept;
  [[nodiscard]] bool right_enabled() const noexcept;

  [[nodiscard]] Result validate_config() const;
  [[nodiscard]] Result validate_throttle(
      std::uint16_t throttle_raw) const;

  [[nodiscard]] std::uint8_t
  next_transfer_id() noexcept;

  Result send_encoded(
      const protocols::PropulsionFanEncodeResult& encoded);

  Result send_raw_once(
      const std::array<std::uint16_t, 4>& values);

  Result initialize_once();
  void worker_loop();
  void poll_telemetry_once();
  void update_telemetry(
      const protocols::PropulsionFanTelemetry& telemetry);

  bool wait_interruptible(
      std::chrono::milliseconds duration);

  void mark_link_failure(
      const Result& result);

  void send_shutdown_sequence() noexcept;

  ICanBus& can_bus_;
  PropulsionFanControllerConfig config_;

  std::atomic<DriverHealth> health_{
      DriverHealth::kUnconfigured};

  std::atomic_bool running_{false};

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;

  bool first_attempt_done_{false};
  bool initialized_{false};
  bool ready_enabled_{false};
  bool ready_requested_{true};

  std::array<std::uint16_t, 4> target_raw_{};
  std::array<std::uint16_t, 4> last_sent_raw_{};

  std::uint8_t transfer_id_{0};

  EscCache left_cache_{};
  EscCache right_cache_{};

  Result last_result_{
      Result::failure(
          ErrorCode::kConfigRequired,
          "propulsion fan has not started",
          "start",
          kPropulsionFanDeviceId)};
};

}  // namespace snappy::drivers::devices