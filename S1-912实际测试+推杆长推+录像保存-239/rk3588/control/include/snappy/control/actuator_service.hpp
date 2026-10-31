#pragma once

/**
 * @file actuator_service.hpp
 * @brief 整机执行器服务声明
 *
 * @details
 * - 各执行器使用独立互斥锁，某一路设备阻塞时不占用其他设备锁。
 */

#include "snappy/drivers/devices/brush_motor.hpp"
#include "snappy/drivers/devices/climber.hpp"
#include "snappy/drivers/devices/propulsion_fan.hpp"
#include "snappy/drivers/devices/push_motor.hpp"
#include "snappy/drivers/result.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace snappy::control {

struct ActuatorServiceDevices {
  drivers::devices::BrushMotor* brush_motor{nullptr};
  drivers::devices::PropulsionFanController* propulsion_fan{nullptr};
  drivers::devices::PushMotor* brush_push_motor{nullptr};
  drivers::devices::PushMotor* scraper_push_motor{nullptr};
  drivers::devices::PushMotor* push_motor{nullptr};
  drivers::devices::Climber* climber{nullptr};
};

struct ActuatorServiceOptions {
  bool noop_missing_actuators{false};
  bool ignore_startup_failures{false};
  bool enable_recovery_worker{true};
};

struct ActuatorRuntimeState {
  std::string brush_state{"stopped"};
  int brush_speed_raw{0};

  bool fan_active{false};
  int fan_left_raw{0};
  int fan_right_raw{0};

  std::string brush_push_state{"stopped"};
  int brush_push_remaining_ms{0};
  int brush_push_last_duration_ms{0};
  int brush_push_extension_mm{0};

  std::string scraper_push_state{"stopped"};
  int scraper_push_remaining_ms{0};
  int scraper_push_last_duration_ms{0};
  int scraper_push_extension_mm{0};

  std::string push_state{"stopped"};
  int push_remaining_ms{0};
  int push_last_duration_ms{0};

  std::string climber_state{"stopped"};
  double climber_speed_rpm{0.0};
};

class ActuatorService {
 public:
  explicit ActuatorService(
      ActuatorServiceDevices devices,
      ActuatorServiceOptions options = {});
  ~ActuatorService();

  ActuatorService(const ActuatorService&) = delete;
  ActuatorService& operator=(const ActuatorService&) = delete;

  drivers::Result start();
  void stop() noexcept;

  drivers::Result brush_forward(int speed_raw);
  drivers::Result brush_reverse(int speed_raw);
  drivers::Result brush_stop();
  drivers::Result clear_faults();

  drivers::Result fan_prepare();
  drivers::Result fan_enable();
  drivers::Result fan_disable();
  drivers::Result fan_left_set(int speed_raw);
  drivers::Result fan_right_set(int speed_raw);
  drivers::Result fan_both_set(int left_raw, int right_raw);
  drivers::Result fan_stop_all();

  drivers::Result brush_push_extend();
  drivers::Result brush_push_retract();
  drivers::Result brush_push_extend_for(
      std::chrono::milliseconds duration,
      bool allow_beyond_max = false,
      bool hold_until_stop = false);
  drivers::Result brush_push_retract_for(
      std::chrono::milliseconds duration,
      bool force_retract_from_zero = false,
      bool hold_until_stop = false);
  drivers::Result brush_push_stop();

  drivers::Result scraper_push_extend();
  drivers::Result scraper_push_retract();
  drivers::Result scraper_push_extend_for(
      std::chrono::milliseconds duration,
      bool allow_beyond_max = false,
      bool hold_until_stop = false);
  drivers::Result scraper_push_retract_for(
      std::chrono::milliseconds duration,
      bool force_retract_from_zero = false,
      bool hold_until_stop = false);
  drivers::Result scraper_push_stop();

  drivers::Result push_extend();
  drivers::Result push_retract();
  drivers::Result push_extend_for(
      std::chrono::milliseconds duration);
  drivers::Result push_retract_for(
      std::chrono::milliseconds duration);
  drivers::Result push_stop();

  drivers::Result climber_ascend(
      double speed_rpm);
  drivers::Result climber_descend(
      double speed_rpm);
  drivers::Result climber_stop();

  drivers::Result recover_brush_motor();
  drivers::Result recover_relay();
  drivers::Result recover_climber();

  drivers::Result safe_stop_all();

  drivers::Result tick(
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now());

  ActuatorRuntimeState snapshot(
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now());

 private:
  enum class TimedPushDirection {
    kNone,
    kExtend,
    kRetract,
  };

  struct TimedPushState {
    TimedPushDirection direction{
        TimedPushDirection::kNone};
    std::chrono::steady_clock::time_point deadline{};
    std::chrono::steady_clock::time_point started_at{};
    int initial_extension_mm{0};
    bool hold_until_stop{false};
  };

  drivers::Result missing_device(
      const char* name) const;

  bool ignore_runtime_failure(
      const drivers::Result& result) const noexcept;

  drivers::Result start_timed_push(
      drivers::devices::PushMotor* motor,
      const char* name,
      std::string& state,
      int& last_duration_ms,
      TimedPushState& timed,
      TimedPushDirection direction,
      std::chrono::milliseconds duration,
      bool force_retract_from_zero = false,
      bool allow_extend_beyond_max = false,
      bool hold_until_stop = false);

  drivers::Result stop_push(
      drivers::devices::PushMotor* motor,
      const char* name,
      std::string& state,
      int& last_duration_ms,
      TimedPushState& timed,
      int& extension_mm,
      double speed_mm_per_second);

  void clear_timed_push(
      TimedPushState& timed);

  int remaining_ms(
      const TimedPushState& timed,
      std::chrono::steady_clock::time_point now) const;

  void update_timed_push_extension_locked(
      int& extension_mm,
      const TimedPushState& timed,
      int total_duration_ms,
      double speed_mm_per_second,
      std::chrono::steady_clock::time_point now) const;

  void clear_runtime_state();
  void clear_runtime_state_locked();

  void start_recovery_worker();
  void stop_recovery_worker() noexcept;
  void recovery_loop() noexcept;

  ActuatorServiceDevices devices_;
  ActuatorServiceOptions options_;

  std::mutex lifecycle_mutex_;
  std::mutex state_mutex_;
  std::mutex brush_mutex_;
  std::mutex fan_mutex_;
  std::mutex push_mutex_;
  std::mutex climber_mutex_;

  std::atomic_bool recovery_running_{false};
  std::mutex recovery_wait_mutex_;
  std::condition_variable recovery_wait_cv_;
  std::thread recovery_thread_;

  std::uint16_t fan_left_raw_{0};
  std::uint16_t fan_right_raw_{0};

  std::string brush_state_{"stopped"};
  int brush_speed_raw_{0};

  bool fan_active_{false};
  bool fan_stopping_{false};
  std::chrono::steady_clock::time_point
      fan_stop_next_step_{};

  std::string brush_push_state_{"stopped"};
  int brush_push_last_duration_ms_{0};
  TimedPushState brush_timed_push_{};
  int brush_push_extension_mm_{0};

  std::string scraper_push_state_{"stopped"};
  int scraper_push_last_duration_ms_{0};
  TimedPushState scraper_timed_push_{};
  int scraper_push_extension_mm_{0};

  std::string push_state_{"stopped"};
  int push_last_duration_ms_{0};

  std::string climber_state_{"stopped"};
  double climber_speed_rpm_{0.0};
};

}  // namespace snappy::control