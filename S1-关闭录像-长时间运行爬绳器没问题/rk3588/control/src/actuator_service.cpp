/**
 * @file actuator_service.cpp
 * @brief 整机执行器服务声明
 *
 * @details
 * - 各执行器使用独立互斥锁，某一路设备阻塞时不占用其他设备锁。
 */
 
#include "snappy/control/actuator_service.hpp"
#include "snappy/drivers/diagnostics/driver_log.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace snappy::control {
namespace {

constexpr int kBrushMaxSpeedRaw = 15;
constexpr int kFanMaxThrottleRaw = 8191;

constexpr int kPushMaxExtensionMm = 300;
constexpr double kBrushPushSpeedMmPerSecond = 33.0;
constexpr double kScraperPushSpeedMmPerSecond = 30.0;

constexpr std::uint16_t kFanStopStepRaw = 200;
constexpr auto kFanStopPeriod =
    std::chrono::milliseconds(200);

constexpr double kClimberMinSpeedRpm = 500.0;
constexpr double kClimberMaxSpeedRpm = 3000.0;

std::chrono::milliseconds clamp_push_duration(
    std::chrono::milliseconds duration) {
  constexpr auto kMin =
      std::chrono::milliseconds(100);
  constexpr auto kMax =
      std::chrono::milliseconds(10000);

  return std::max(
      kMin,
      std::min(duration, kMax));
}

std::uint16_t clamp_brush_raw(int value) {
  return static_cast<std::uint16_t>(
      std::clamp(value, 0, kBrushMaxSpeedRaw));
}

std::uint16_t clamp_fan_raw(int value) {
  return static_cast<std::uint16_t>(
      std::clamp(value, 0, kFanMaxThrottleRaw));
}

double clamp_climber_speed_rpm(double speed_rpm) {
  const auto requested =
      std::abs(speed_rpm);

  if (requested <= 0.0) {
    return kClimberMinSpeedRpm;
  }

  return std::clamp(
      requested,
      kClimberMinSpeedRpm,
      kClimberMaxSpeedRpm);
}

std::int32_t climber_speed_command_rpm(
    double speed_rpm,
    int direction) {
  const auto fixed =
      clamp_climber_speed_rpm(speed_rpm);

  const auto rounded =
      static_cast<std::int32_t>(
          std::lround(fixed));

  return direction >= 0 ? rounded : -rounded;
}

bool same_push_motor(
    drivers::devices::PushMotor* first,
    drivers::devices::PushMotor* second) {
  return first != nullptr && first == second;
}

bool retryable_or_missing(
    const drivers::Result& result) {
  return result.code ==
             drivers::ErrorCode::kConfigRequired ||
         result.code ==
             drivers::ErrorCode::kRs485Timeout ||
         result.code ==
             drivers::ErrorCode::kIoError ||
         result.code ==
             drivers::ErrorCode::kCanTxTimeout ||
         result.retryable;
}

bool push_motion_active(
    const std::string& state) {
  return state == "extend" ||
         state == "retract";
}

drivers::Result motion_interlock_failure(
    const char* device,
    const std::string& current_state,
    const char* requested_action) {
  return drivers::Result::failure(
      drivers::ErrorCode::kRelayInterlockActive,
      std::string(device) +
          " is already " +
          current_state +
          "; stop it before " +
          requested_action,
      requested_action,
      device);
}

void log_brush_result(
    const char* action,
    const drivers::Result& result,
    int speed_raw = -1) {
  std::string message =
      "action=" + std::string(action);

  if (speed_raw >= 0) {
    message +=
        " speed_raw=" +
        std::to_string(speed_raw);
  }

  message += result.ok
                 ? " accepted"
                 : " failed";

  if (!result.detail.empty()) {
    message +=
        " detail=" +
        result.detail;
  }

  if (result.ok) {
    drivers::diagnostics::info(
        "brush.command",
        message);
  } else {
    drivers::diagnostics::warn(
        "brush.command",
        message);
  }
}

}  // namespace

ActuatorService::ActuatorService(
    ActuatorServiceDevices devices,
    ActuatorServiceOptions options)
    : devices_(devices),
      options_(options) {}

ActuatorService::~ActuatorService() {
  stop();
}

drivers::Result ActuatorService::missing_device(
    const char* name) const {
  if (options_.noop_missing_actuators) {
    return drivers::Result::success(
        std::string("noop missing actuator: ") +
        name);
  }

  return drivers::Result::failure(
      drivers::ErrorCode::kConfigRequired,
      std::string(
          "actuator device is unavailable: ") +
          name,
      "actuator",
      name);
}

bool ActuatorService::ignore_runtime_failure(
    const drivers::Result& result) const noexcept {
  return options_.ignore_startup_failures &&
         retryable_or_missing(result);
}

void ActuatorService::start_recovery_worker() {
  if (!options_.ignore_startup_failures) {
    return;
  }

  if (recovery_running_.exchange(true)) {
    return;
  }

  try {
    recovery_thread_ = std::thread(
        [this]() {
          recovery_loop();
        });
  } catch (...) {
    recovery_running_.store(false);

    drivers::diagnostics::error(
        "actuator.service",
        "failed to create actuator recovery worker");
  }
}

void ActuatorService::stop_recovery_worker() noexcept {
  recovery_running_.store(false);
  recovery_wait_cv_.notify_all();

  if (recovery_thread_.joinable()) {
    recovery_thread_.join();
  }
}

void ActuatorService::recovery_loop() noexcept {
  while (recovery_running_.load()) {
    try {
      if (devices_.brush_motor != nullptr) {
        std::lock_guard<std::mutex> lock(
            brush_mutex_);

        if (devices_.brush_motor->health() !=
            drivers::DriverHealth::kReady) {
          const auto result =
              devices_.brush_motor->start();

          if (result.ok) {
            drivers::diagnostics::info(
                "actuator.service",
                "brush motor recovered");
          }
        }
      }

      {
        std::lock_guard<std::mutex> lock(
            push_mutex_);

        if (devices_.brush_push_motor != nullptr &&
            devices_.brush_push_motor->health() !=
                drivers::DriverHealth::kReady) {
          static_cast<void>(
              devices_.brush_push_motor->start());
        }

        if (devices_.scraper_push_motor != nullptr &&
            !same_push_motor(
                devices_.scraper_push_motor,
                devices_.brush_push_motor) &&
            devices_.scraper_push_motor->health() !=
                drivers::DriverHealth::kReady) {
          static_cast<void>(
              devices_.scraper_push_motor->start());
        }

        if (devices_.push_motor != nullptr &&
            !same_push_motor(
                devices_.push_motor,
                devices_.brush_push_motor) &&
            !same_push_motor(
                devices_.push_motor,
                devices_.scraper_push_motor) &&
            devices_.push_motor->health() !=
                drivers::DriverHealth::kReady) {
          static_cast<void>(
              devices_.push_motor->start());
        }
      }
    } catch (...) {
      drivers::diagnostics::warn(
          "actuator.service",
          "actuator recovery iteration failed");
    }

    std::unique_lock<std::mutex> lock(
        recovery_wait_mutex_);

    recovery_wait_cv_.wait_for(
        lock,
        std::chrono::seconds(1),
        [this]() {
          return !recovery_running_.load();
        });
  }
}

drivers::Result ActuatorService::recover_brush_motor() {
  if (devices_.brush_motor == nullptr) {
    return missing_device("brush_motor");
  }

  std::lock_guard<std::mutex> lock(
      brush_mutex_);

  if (devices_.brush_motor->health() ==
      drivers::DriverHealth::kReady) {
    return drivers::Result::success(
        "brush motor already ready");
  }

  return devices_.brush_motor->start();
}

drivers::Result ActuatorService::recover_relay() {
  if (devices_.brush_push_motor == nullptr) {
    return missing_device("brush_push_motor");
  }

  std::lock_guard<std::mutex> lock(
      push_mutex_);

  if (devices_.brush_push_motor->health() ==
      drivers::DriverHealth::kReady) {
    return drivers::Result::success(
        "relay already ready");
  }

  return devices_.brush_push_motor->start();
}

drivers::Result ActuatorService::recover_climber() {
  if (devices_.climber == nullptr) {
    return missing_device("climber");
  }

  std::lock_guard<std::mutex> lock(
      climber_mutex_);

  if (devices_.climber->health() ==
      drivers::DriverHealth::kReady) {
    return drivers::Result::success(
        "climber already ready");
  }

  return devices_.climber->start();
}


drivers::Result ActuatorService::start() {
  std::lock_guard<std::mutex> lifecycle_lock(
      lifecycle_mutex_);

  {
    std::lock_guard<std::mutex> lock(
        fan_mutex_);

    fan_stopping_ = false;
    fan_stop_next_step_ = {};
  }

  clear_runtime_state();

  drivers::Result first_failure =
      drivers::Result::success();

  bool have_failure = false;
  bool degraded = false;

  const auto record_start =
      [&](auto* device,
          const char* component) {
        if (device == nullptr) {
          return;
        }

        const auto result = device->start();

        if (result.ok) {
          return;
        }

        if (ignore_runtime_failure(result)) {
          degraded = true;
          return;
        }

        if (!have_failure) {
          first_failure =
              drivers::Result::failure(
                  result.code,
                  result.detail,
                  result.operation.empty()
                      ? "start"
                      : result.operation,
                  result.component.empty()
                      ? component
                      : result.component,
                  result.retryable);

          have_failure = true;
        }
      };

  {
    std::lock_guard<std::mutex> lock(
        fan_mutex_);

    record_start(
        devices_.propulsion_fan,
        "propulsion_fan");
  }

  {
    std::lock_guard<std::mutex> lock(
        brush_mutex_);

    record_start(
        devices_.brush_motor,
        "brush_motor");
  }

    {
    std::lock_guard<std::mutex> lock(
        push_mutex_);

    /*
     * USB-RS485/继电器上电后可能需要一点时间稳定。
     * 只在启动阶段等待，不改变运行期间的推杆控制逻辑。
     */
    std::this_thread::sleep_for(
        std::chrono::milliseconds(500));

    record_start(
        devices_.brush_push_motor,
        "brush_push_motor");

    if (devices_.brush_push_motor != nullptr &&
        devices_.brush_push_motor->health() !=
            drivers::DriverHealth::kReady) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(300));

      record_start(
          devices_.brush_push_motor,
          "brush_push_motor");
    }

    if (devices_.scraper_push_motor != nullptr &&
        !same_push_motor(
            devices_.scraper_push_motor,
            devices_.brush_push_motor)) {

      record_start(
          devices_.scraper_push_motor,
          "scraper_push_motor");

      if (devices_.scraper_push_motor->health() !=
          drivers::DriverHealth::kReady) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(300));

        record_start(
            devices_.scraper_push_motor,
            "scraper_push_motor");
      }
    }

    if (devices_.push_motor != nullptr &&
        !same_push_motor(
            devices_.push_motor,
            devices_.brush_push_motor) &&
        !same_push_motor(
            devices_.push_motor,
            devices_.scraper_push_motor)) {

      record_start(
          devices_.push_motor,
          "push_motor");

      if (devices_.push_motor->health() !=
          drivers::DriverHealth::kReady) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(300));

        record_start(
            devices_.push_motor,
            "push_motor");
      }
    }
  }

  {
    std::lock_guard<std::mutex> lock(
        climber_mutex_);

    record_start(
        devices_.climber,
        "climber");
  }

  if (have_failure) {
    return first_failure;
  }

  if (degraded) {
    start_recovery_worker();

    return drivers::Result::success(
        "actuators started with isolated device failures");
  }

  return drivers::Result::success(
      "actuators ready");
}

void ActuatorService::stop() noexcept {
  try {
    std::lock_guard<std::mutex> lifecycle_lock(
        lifecycle_mutex_);

    stop_recovery_worker();

    static_cast<void>(safe_stop_all());

    {
      std::lock_guard<std::mutex> lock(
          fan_mutex_);

      if (devices_.propulsion_fan != nullptr) {
        devices_.propulsion_fan->stop();
      }
    }

    {
      std::lock_guard<std::mutex> lock(
          brush_mutex_);

      if (devices_.brush_motor != nullptr) {
        devices_.brush_motor->stop();
      }
    }

    {
      std::lock_guard<std::mutex> lock(
          push_mutex_);

      if (devices_.brush_push_motor != nullptr) {
        devices_.brush_push_motor->stop();
      }

      if (devices_.scraper_push_motor != nullptr &&
          !same_push_motor(
              devices_.scraper_push_motor,
              devices_.brush_push_motor)) {
        devices_.scraper_push_motor->stop();
      }

      if (devices_.push_motor != nullptr &&
          !same_push_motor(
              devices_.push_motor,
              devices_.brush_push_motor) &&
          !same_push_motor(
              devices_.push_motor,
              devices_.scraper_push_motor)) {
        devices_.push_motor->stop();
      }
    }

    {
      std::lock_guard<std::mutex> lock(
          climber_mutex_);

      if (devices_.climber != nullptr) {
        devices_.climber->stop();
      }
    }

    clear_runtime_state();
  } catch (...) {
  }
}

drivers::Result ActuatorService::brush_forward(
    int speed_raw) {
  const auto raw =
      clamp_brush_raw(speed_raw);

  if (devices_.brush_motor == nullptr) {
    const auto result =
        missing_device("brush_motor");

    log_brush_result(
        "forward",
        result,
        static_cast<int>(raw));

    return result;
  }

  std::lock_guard<std::mutex> lock(
      brush_mutex_);

  std::string current_state;

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    current_state = brush_state_;
  }

  if (current_state == "reverse") {
    const auto result =
        motion_interlock_failure(
            "brush_motor",
            current_state,
            "forward");

    log_brush_result(
        "forward_rejected",
        result,
        static_cast<int>(raw));

    return result;
  }

  const auto result =
      devices_.brush_motor->run(
          drivers::protocols::
              BrushMotorDirection::kClockwise,
          raw);

  if (result.ok) {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    brush_state_ = "forward";
    brush_speed_raw_ =
        static_cast<int>(raw);
  }

  log_brush_result(
      "forward",
      result,
      static_cast<int>(raw));

  return result;
}

drivers::Result ActuatorService::brush_reverse(
    int speed_raw) {
  const auto raw =
      clamp_brush_raw(speed_raw);

  if (devices_.brush_motor == nullptr) {
    const auto result =
        missing_device("brush_motor");

    log_brush_result(
        "reverse",
        result,
        static_cast<int>(raw));

    return result;
  }

  std::lock_guard<std::mutex> lock(
      brush_mutex_);

  std::string current_state;

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    current_state = brush_state_;
  }

  if (current_state == "forward") {
    const auto result =
        motion_interlock_failure(
            "brush_motor",
            current_state,
            "reverse");

    log_brush_result(
        "reverse_rejected",
        result,
        static_cast<int>(raw));

    return result;
  }

  const auto result =
      devices_.brush_motor->run(
          drivers::protocols::
              BrushMotorDirection::
                  kCounterClockwise,
          raw);

  if (result.ok) {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    brush_state_ = "reverse";
    brush_speed_raw_ =
        static_cast<int>(raw);
  }

  log_brush_result(
      "reverse",
      result,
      static_cast<int>(raw));

  return result;
}

drivers::Result ActuatorService::brush_stop() {
  if (devices_.brush_motor == nullptr) {
    const auto result =
        missing_device("brush_motor");

    log_brush_result(
        "stop",
        result);

    return result;
  }

  std::lock_guard<std::mutex> lock(
      brush_mutex_);

  const auto result =
      devices_.brush_motor->stop_motion();

  if (result.ok) {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    brush_state_ = "stopped";
    brush_speed_raw_ = 0;
  }

  log_brush_result(
      "stop",
      result);

  return result;
}

drivers::Result ActuatorService::clear_faults() {
  /*
   * ============================================================
   * 清除故障：
   *
   * 核心原则：
   *
   * 1. 所有执行器立即停止；
   * 2. 不关闭 CAN；
   * 3. 不关闭 RS485；
   * 4. 不重新创建 / 销毁任何总线；
   * 5. 不清除推杆当前位置；
   * 6. 风机只清零输出，不关闭 CAN Ready；
   * 7. 毛刷清除驱动历史故障；
   * 8. 爬绳器重新执行抱闸、报警清除和状态确认；
   * 9. 清故障完成后，下一次遥控操作可以直接继续工作。
   * ============================================================
   */

  /*
   * 第一阶段：
   *
   * 立即停止所有执行器。
   *
   * safe_stop_all()：
   * - 推杆停止，但保持当前位置；
   * - 毛刷停止；
   * - 风机目标归零；
   * - 爬绳器执行安全停止 / 抱闸；
   * - 清除正在执行的定时动作。
   *
   * 它不会关闭 CAN / RS485。
   */
  const auto stop_result =
      safe_stop_all();

  drivers::Result first_failure =
      drivers::Result::success();

  bool have_failure = false;

  std::string detail =
      "fault clear:";

  const auto record =
      [&](const char* name,
          const drivers::Result& result) {
        detail += " ";
        detail += name;
        detail += "=";

        detail += result.ok
                      ? "ok"
                      : "failed";

        if (!result.detail.empty()) {
          detail += "(";
          detail += result.detail;
          detail += ")";
        }

        if (!result.ok &&
            !have_failure) {
          first_failure = result;
          have_failure = true;
        }
      };

  /*
   * safe_stop_all() 的结果不能丢掉。
   *
   * 特别是爬绳器：
   * 如果重新抱闸失败，不能假装整个故障已经成功清除。
   */
  record(
      "safe_stop",
      stop_result);

  /*
   * ============================================================
   * 毛刷
   * ============================================================
   */
  if (devices_.brush_motor != nullptr) {
    drivers::Result result;

    {
      std::lock_guard<std::mutex>
          lock(brush_mutex_);

      result =
          devices_.brush_motor
              ->clear_history_fault();
    }

    record(
        "brush_motor",
        result);
  }

  /*
   * ============================================================
   * 风机
   * ============================================================
   */
  if (devices_.propulsion_fan != nullptr) {
    devices_.propulsion_fan->safe_stop();

    record(
        "propulsion_fan",
        drivers::Result::success(
            "fan stopped; CAN/Ready state preserved"));
  }

  /*
   * ============================================================
   * 推杆
   * ============================================================
   */
  if (devices_.brush_push_motor != nullptr) {
    record(
        "brush_push_motor",
        drivers::Result::success(
            "relay outputs deenergized; "
            "position preserved"));
  }

  if (devices_.scraper_push_motor != nullptr) {
    record(
        "scraper_push_motor",
        drivers::Result::success(
            "relay outputs deenergized; "
            "position preserved"));
  }

  /*
   * ============================================================
   * 爬绳器
   * ============================================================
   */
  if (devices_.climber != nullptr) {
    drivers::Result result;

    {
      std::lock_guard<std::mutex>
          lock(climber_mutex_);

      result =
          devices_.climber
              ->clear_fault();
    }

    record(
        "climber",
        result);
  }

  /*
   * ============================================================
   * 软件运行状态同步
   * ============================================================
   */
  clear_runtime_state();

  /*
   * ============================================================
   * 最终结果
   * ============================================================
   */
  if (have_failure) {
    return drivers::Result::failure(
        first_failure.code,
        detail,
        first_failure.operation.empty()
            ? "clear_faults"
            : first_failure.operation,
        first_failure.component.empty()
            ? "actuator_service"
            : first_failure.component,
        first_failure.retryable);
  }

  return drivers::Result::success(detail);
}

drivers::Result ActuatorService::fan_prepare() {
  if (devices_.propulsion_fan == nullptr) {
    return missing_device("propulsion_fan");
  }

  return drivers::Result::success(
      "fan CAN initialization is automatic at actuator startup");
}

drivers::Result ActuatorService::fan_enable() {
  if (devices_.propulsion_fan == nullptr) {
    return missing_device("propulsion_fan");
  }

  std::lock_guard<std::mutex> lock(
      fan_mutex_);

  return devices_.propulsion_fan
      ->enable_throttle();
}

drivers::Result ActuatorService::fan_disable() {
  if (devices_.propulsion_fan == nullptr) {
    return missing_device("propulsion_fan");
  }

  drivers::Result result;

  {
    std::lock_guard<std::mutex> lock(
        fan_mutex_);

    result =
        devices_.propulsion_fan
            ->disable_throttle();

    fan_stopping_ = false;
    fan_stop_next_step_ = {};
  }

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    fan_left_raw_ = 0;
    fan_right_raw_ = 0;
    fan_active_ = false;
  }

  return result;
}

drivers::Result ActuatorService::fan_left_set(
    int speed_raw) {
  return fan_both_set(
      speed_raw,
      speed_raw);
}

drivers::Result ActuatorService::fan_right_set(
    int speed_raw) {
  return fan_both_set(
      speed_raw,
      speed_raw);
}

drivers::Result ActuatorService::fan_both_set(
    int left_raw,
    int right_raw) {
  if (devices_.propulsion_fan == nullptr) {
    return missing_device("propulsion_fan");
  }

  const auto synchronized_raw =
      clamp_fan_raw(
          std::max(left_raw, right_raw));

  drivers::Result result;

  {
    std::lock_guard<std::mutex> lock(
        fan_mutex_);

    result =
        devices_.propulsion_fan
            ->set_left_right_raw(
                synchronized_raw,
                synchronized_raw);

    if (result.ok) {
      fan_stopping_ = false;
      fan_stop_next_step_ = {};
    }
  }

  if (result.ok) {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    fan_left_raw_ = synchronized_raw;
    fan_right_raw_ = synchronized_raw;
    fan_active_ = synchronized_raw != 0;
  }

  return result;
}

drivers::Result ActuatorService::fan_stop_all() {
  if (devices_.propulsion_fan == nullptr) {
    return missing_device("propulsion_fan");
  }

  const auto now =
      std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> fan_lock(
      fan_mutex_);

  std::uint16_t current_raw = 0;

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    current_raw =
        std::max(
            fan_left_raw_,
            fan_right_raw_);
  }

  if (current_raw > 0) {
    fan_stopping_ = true;
    fan_stop_next_step_ =
        now + kFanStopPeriod;

    return drivers::Result::success(
        "propulsion fan deceleration started");
  }

  const auto result =
      devices_.propulsion_fan->stop_all();

  fan_stopping_ = false;
  fan_stop_next_step_ = {};

  if (result.ok) {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    fan_left_raw_ = 0;
    fan_right_raw_ = 0;
    fan_active_ = false;
  }

  return result;
}

drivers::Result
ActuatorService::brush_push_extend() {
  return brush_push_extend_for(
      std::chrono::milliseconds(10000));
}

drivers::Result
ActuatorService::brush_push_retract() {
  return brush_push_retract_for(
      std::chrono::milliseconds(10000));
}

drivers::Result
ActuatorService::brush_push_extend_for(
    std::chrono::milliseconds duration,
    bool allow_beyond_max,
    bool hold_until_stop) {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  return start_timed_push(
      devices_.brush_push_motor,
      "brush_push_motor",
      brush_push_state_,
      brush_push_last_duration_ms_,
      brush_timed_push_,
      TimedPushDirection::kExtend,
      duration,
      false,
      allow_beyond_max,
      hold_until_stop);
}

drivers::Result
ActuatorService::brush_push_retract_for(
    std::chrono::milliseconds duration,
    bool force_retract_from_zero,
    bool hold_until_stop) {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  return start_timed_push(
      devices_.brush_push_motor,
      "brush_push_motor",
      brush_push_state_,
      brush_push_last_duration_ms_,
      brush_timed_push_,
      TimedPushDirection::kRetract,
      duration,
      force_retract_from_zero,
      false,
      hold_until_stop);
}

drivers::Result
ActuatorService::brush_push_stop() {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  return stop_push(
      devices_.brush_push_motor,
      "brush_push_motor",
      brush_push_state_,
      brush_push_last_duration_ms_,
      brush_timed_push_,
      brush_push_extension_mm_,
      kBrushPushSpeedMmPerSecond);
}

drivers::Result
ActuatorService::scraper_push_extend() {
  return scraper_push_extend_for(
      std::chrono::milliseconds(10000));
}

drivers::Result
ActuatorService::scraper_push_retract() {
  return scraper_push_retract_for(
      std::chrono::milliseconds(10000));
}

drivers::Result
ActuatorService::scraper_push_extend_for(
    std::chrono::milliseconds duration,
    bool allow_beyond_max,
    bool hold_until_stop) {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  return start_timed_push(
      devices_.scraper_push_motor,
      "scraper_push_motor",
      scraper_push_state_,
      scraper_push_last_duration_ms_,
      scraper_timed_push_,
      TimedPushDirection::kExtend,
      duration,
      false,
      allow_beyond_max,
      hold_until_stop);
}

drivers::Result
ActuatorService::scraper_push_retract_for(
    std::chrono::milliseconds duration,
    bool force_retract_from_zero,
    bool hold_until_stop) {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  return start_timed_push(
      devices_.scraper_push_motor,
      "scraper_push_motor",
      scraper_push_state_,
      scraper_push_last_duration_ms_,
      scraper_timed_push_,
      TimedPushDirection::kRetract,
      duration,
      force_retract_from_zero,
      false,
      hold_until_stop);
}

drivers::Result
ActuatorService::scraper_push_stop() {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  return stop_push(
      devices_.scraper_push_motor,
      "scraper_push_motor",
      scraper_push_state_,
      scraper_push_last_duration_ms_,
      scraper_timed_push_,
      scraper_push_extension_mm_,
      kScraperPushSpeedMmPerSecond);
}

drivers::Result ActuatorService::push_extend() {
  return brush_push_extend();
}

drivers::Result ActuatorService::push_retract() {
  return brush_push_retract();
}

drivers::Result ActuatorService::push_extend_for(
    std::chrono::milliseconds duration) {
  return brush_push_extend_for(duration);
}

drivers::Result ActuatorService::push_retract_for(
    std::chrono::milliseconds duration) {
  return brush_push_retract_for(duration);
}

drivers::Result ActuatorService::push_stop() {
  std::lock_guard<std::mutex> lock(
      push_mutex_);

  if (devices_.brush_push_motor == nullptr &&
      devices_.scraper_push_motor == nullptr &&
      devices_.push_motor == nullptr) {
    return missing_device("push_motor");
  }

  drivers::Result first_failure =
      drivers::Result::success();

  bool have_failure = false;

  const auto record =
      [&](const drivers::Result& result) {
        if (!result.ok && !have_failure) {
          first_failure = result;
          have_failure = true;
        }
      };

  if (devices_.brush_push_motor != nullptr) {
    record(
        stop_push(
            devices_.brush_push_motor,
            "brush_push_motor",
            brush_push_state_,
            brush_push_last_duration_ms_,
            brush_timed_push_,
            brush_push_extension_mm_,
            kBrushPushSpeedMmPerSecond));
  }

  if (devices_.scraper_push_motor != nullptr &&
      !same_push_motor(
          devices_.scraper_push_motor,
          devices_.brush_push_motor)) {
    record(
        stop_push(
            devices_.scraper_push_motor,
            "scraper_push_motor",
            scraper_push_state_,
            scraper_push_last_duration_ms_,
            scraper_timed_push_,
            scraper_push_extension_mm_,
            kScraperPushSpeedMmPerSecond));
  }

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    push_state_ = "stopped";
    push_last_duration_ms_ = 0;
  }

  if (have_failure) {
    return first_failure;
  }

  return drivers::Result::success(
      "push motors stopped");
}

drivers::Result ActuatorService::start_timed_push(
    drivers::devices::PushMotor* motor,
    const char* name,
    std::string& state,
    int& last_duration_ms,
    TimedPushState& timed,
    TimedPushDirection direction,
    std::chrono::milliseconds duration,
    bool force_retract_from_zero,
    bool allow_extend_beyond_max,
    bool hold_until_stop) {
  if (motor == nullptr) {
    return missing_device(name);
  }

  const bool brush_push =
      std::string(name) == "brush_push_motor";

  const char* requested_action =
      direction == TimedPushDirection::kExtend
          ? "extend"
          : "retract";

  int current_extension_mm = 0;

  const double speed_mm_per_second =
      brush_push
          ? kBrushPushSpeedMmPerSecond
          : kScraperPushSpeedMmPerSecond;

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    if (push_motion_active(state)) {
      return motion_interlock_failure(
          name,
          state,
          requested_action);
    }

    if (timed.direction !=
        TimedPushDirection::kNone) {
      return motion_interlock_failure(
          name,
          "stopping",
          requested_action);
    }

    current_extension_mm =
        brush_push
            ? brush_push_extension_mm_
            : scraper_push_extension_mm_;

    current_extension_mm =
        std::clamp(
            current_extension_mm,
            0,
            kPushMaxExtensionMm);
  }

  const int available_extension_mm =
      direction == TimedPushDirection::kExtend
          ? (allow_extend_beyond_max
                 ? std::numeric_limits<int>::max()
                 : kPushMaxExtensionMm -
                       current_extension_mm)
          : (force_retract_from_zero &&
             current_extension_mm == 0)
                ? kPushMaxExtensionMm
                : current_extension_mm;

  if (available_extension_mm <= 0) {
    const auto stop_requested_at =
        std::chrono::steady_clock::now();

    const auto stopped =
        motor->stop_motion();

    if (stopped.ok) {
      std::lock_guard<std::mutex> state_lock(
          state_mutex_);

      if (brush_push) {
        brush_push_extension_mm_ =
            direction == TimedPushDirection::kExtend
                ? kPushMaxExtensionMm
                : 0;
      } else {
        scraper_push_extension_mm_ =
            direction == TimedPushDirection::kExtend
                ? kPushMaxExtensionMm
                : 0;
      }

      clear_timed_push(timed);
      state = "stopped";
      last_duration_ms = 0;

      if (brush_push) {
        push_state_ = "stopped";
        push_last_duration_ms_ = 0;
      }
    } else {
      std::lock_guard<std::mutex> state_lock(
          state_mutex_);

      timed.direction = direction;
      timed.deadline = stop_requested_at;
    }

    if (!stopped.ok) {
      return stopped;
    }

    return drivers::Result::success(
        "push already at stroke limit; relays deenergized");
  }

  const auto requested_duration =
      clamp_push_duration(duration);

  std::chrono::milliseconds effective_duration =
      requested_duration;

  if (!hold_until_stop) {
    const auto boundary_duration =
        std::chrono::milliseconds(
            static_cast<long long>(
                std::ceil(
                    available_extension_mm *
                    1000.0 /
                    speed_mm_per_second)));

    effective_duration =
        std::min(
            requested_duration,
            boundary_duration);
  }

  drivers::Result result;

  if (direction == TimedPushDirection::kExtend) {
    result = motor->extend();
  } else {
    result = motor->retract();
  }

  const auto started_at =
      std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  if (!result.ok) {
    clear_timed_push(timed);
    state = "stopped";
    last_duration_ms = 0;

    if (brush_push) {
      push_state_ = "stopped";
      push_last_duration_ms_ = 0;
    }

    return result;
  }

  timed.direction = direction;
  timed.started_at = started_at;
  timed.initial_extension_mm = current_extension_mm;
  timed.hold_until_stop = hold_until_stop;
  timed.deadline =
      hold_until_stop
          ? std::chrono::steady_clock::time_point::max()
          : started_at + effective_duration;

  state =
      direction == TimedPushDirection::kExtend
          ? "extend"
          : "retract";

  last_duration_ms =
      hold_until_stop
          ? 0
          : static_cast<int>(
                effective_duration.count());

  if (brush_push) {
    push_state_ = state;
    push_last_duration_ms_ =
        last_duration_ms;
  }

  return drivers::Result::success(
      "timed push started");
}

drivers::Result ActuatorService::stop_push(
    drivers::devices::PushMotor* motor,
    const char* name,
    std::string& state,
    int& last_duration_ms,
    TimedPushState& timed,
    int& extension_mm,
    double speed_mm_per_second) {
  if (motor == nullptr) {
    return missing_device(name);
  }

  const auto stop_requested_at =
      std::chrono::steady_clock::now();

  const auto result =
      motor->stop_motion();

  if (!result.ok) {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    if (timed.direction !=
        TimedPushDirection::kNone) {
      timed.deadline = stop_requested_at;
      timed.hold_until_stop = false;
    }

    return result;
  }

  const auto stopped_at =
      std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  update_timed_push_extension_locked(
      extension_mm,
      timed,
      last_duration_ms,
      speed_mm_per_second,
      stopped_at);

  clear_timed_push(timed);
  state = "stopped";
  last_duration_ms = 0;

  if (name == std::string("brush_push_motor")) {
    push_state_ = "stopped";
    push_last_duration_ms_ = 0;
  }

  return result;
}

drivers::Result ActuatorService::climber_ascend(
    double speed_rpm) {
  if (devices_.climber == nullptr) {
    return missing_device("climber");
  }

  const auto command_rpm =
      climber_speed_command_rpm(
          speed_rpm,
          1);

  std::lock_guard<std::mutex> lock(
      climber_mutex_);

  /*
   * 方向切换不再由 ActuatorService 拦截。
   *
   * Climber::set_target_speed_raw()
   * 自己负责：
   *
   * 当前反转
   *   -> 完整 STOP
   *   -> SRV-OFF
   *   -> 抱闸
   *   -> SRV-ON
   *   -> 正转
   */
  const auto result =
      devices_.climber
          ->set_target_speed_raw(
              command_rpm);

  if (result.ok) {
    std::lock_guard<std::mutex>
        state_lock(state_mutex_);

    climber_state_ =
        "ascend";

    climber_speed_rpm_ =
        std::abs(
            static_cast<double>(
                command_rpm));
  }

  return result;
}


drivers::Result ActuatorService::climber_descend(
    double speed_rpm) {
  if (devices_.climber == nullptr) {
    return missing_device("climber");
  }

  const auto command_rpm =
      climber_speed_command_rpm(
          speed_rpm,
          -1);

  std::lock_guard<std::mutex> lock(
      climber_mutex_);

  /*
   * 同样由 Climber 驱动层统一处理
   * 正转 -> STOP -> 反转。
   */
  const auto result =
      devices_.climber
          ->set_target_speed_raw(
              command_rpm);

  if (result.ok) {
    std::lock_guard<std::mutex>
        state_lock(state_mutex_);

    climber_state_ =
        "descend";

    climber_speed_rpm_ =
        std::abs(
            static_cast<double>(
                command_rpm));
  }

  return result;
}

drivers::Result ActuatorService::climber_stop() {
  if (devices_.climber == nullptr) {
    return missing_device("climber");
  }

  std::lock_guard<std::mutex> lock(climber_mutex_);

  const auto result = devices_.climber->disable();

  if (result.ok) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);

    climber_state_ = "stopped";
    climber_speed_rpm_ = 0.0;
  }
  return result;
}

drivers::Result ActuatorService::safe_stop_all() {
  /*
   * 安全停止必须“全部执行”，不能因为某一个设备失败
   * 就提前 return。
   *
   * 特别是：
   *
   *   brush_motor
   *   relay_push_motor
   *   climber
   *
   * 共用 ttyS0。
   *
   * brush 失败不能阻止 climber 执行 STOP。
   */

  {
    std::lock_guard<std::mutex>
        lock(fan_mutex_);

    if (devices_.propulsion_fan != nullptr) {
      devices_.propulsion_fan->safe_stop();
    }

    fan_stopping_ = false;
    fan_stop_next_step_ = {};
  }

  {
    std::lock_guard<std::mutex>
        lock(push_mutex_);

    const auto now =
        std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex>
          state_lock(state_mutex_);

      update_timed_push_extension_locked(
          brush_push_extension_mm_,
          brush_timed_push_,
          brush_push_last_duration_ms_,
          kBrushPushSpeedMmPerSecond,
          now);

      update_timed_push_extension_locked(
          scraper_push_extension_mm_,
          scraper_timed_push_,
          scraper_push_last_duration_ms_,
          kScraperPushSpeedMmPerSecond,
          now);
    }

    if (devices_.brush_push_motor != nullptr) {
      devices_.brush_push_motor->safe_stop();
    }

    if (devices_.scraper_push_motor != nullptr &&
        !same_push_motor(
            devices_.scraper_push_motor,
            devices_.brush_push_motor)) {
      devices_.scraper_push_motor->safe_stop();
    }

    if (devices_.push_motor != nullptr &&
        !same_push_motor(
            devices_.push_motor,
            devices_.brush_push_motor) &&
        !same_push_motor(
            devices_.push_motor,
            devices_.scraper_push_motor)) {
      devices_.push_motor->safe_stop();
    }
  }

  /*
   * 毛刷：
   *
   * safe_stop() 本身是 noexcept + best effort。
   *
   * 如果 BD-201S 不响应，不能让这个失败阻塞
   * 后面的 climber STOP。
   */
  {
    std::lock_guard<std::mutex>
        lock(brush_mutex_);

    if (devices_.brush_motor != nullptr) {
      devices_.brush_motor->safe_stop();
    }
  }

  /*
   * 爬绳器必须最后执行真正的安全停止。
   *
   * 这里仍然保留：
   *
   *   0
   *   -> 等实际速度
   *   -> SRV-OFF
   *   -> RUNNING=0
   *   -> DO3=0
   *
   * 不能改成简单写 0。
   */
  drivers::Result climber_stop_result =
      drivers::Result::success();

  {
    std::lock_guard<std::mutex>
        lock(climber_mutex_);

    if (devices_.climber != nullptr) {
      climber_stop_result =
          devices_.climber->safe_stop();
    }
  }

  clear_runtime_state();

  /*
   * 只有 climber 的安全停止失败，
   * 才阻止整个安全状态恢复
   */
  if (!climber_stop_result.ok) {
    return drivers::Result::failure(
        climber_stop_result.code,
        "climber safe stop failed: " +
            climber_stop_result.detail,
        "safe_stop",
        "climber",
        climber_stop_result.retryable);
  }

  return drivers::Result::success(
      "all actuators stopped; isolated actuator failures do not block others");
}

drivers::Result ActuatorService::tick(
    std::chrono::steady_clock::time_point now) {
  drivers::Result first_failure =
      drivers::Result::success();

  bool have_failure = false;

  const auto record_failure =
      [&](const drivers::Result& result) {
        if (!result.ok &&
            !ignore_runtime_failure(result) &&
            !have_failure) {
          first_failure = result;
          have_failure = true;
        }
      };

  {
    std::lock_guard<std::mutex> fan_lock(
        fan_mutex_);

    bool step_due = false;
    std::uint16_t current_raw = 0;

    {
      std::lock_guard<std::mutex> state_lock(
          state_mutex_);

      current_raw =
          std::max(
              fan_left_raw_,
              fan_right_raw_);

      step_due =
          fan_stopping_ &&
          now >= fan_stop_next_step_;
    }

    if (step_due) {
      const auto next_raw =
          current_raw > kFanStopStepRaw
              ? static_cast<std::uint16_t>(
                    current_raw -
                    kFanStopStepRaw)
              : static_cast<std::uint16_t>(0);

      drivers::Result result;

      if (devices_.propulsion_fan != nullptr) {
        result =
            devices_.propulsion_fan
                ->set_left_right_raw(
                    next_raw,
                    next_raw);
      } else {
        result =
            missing_device("propulsion_fan");
      }

      if (result.ok) {
        {
          std::lock_guard<std::mutex> state_lock(
              state_mutex_);

          fan_left_raw_ = next_raw;
          fan_right_raw_ = next_raw;
          fan_active_ = next_raw != 0;
        }

        if (next_raw == 0) {
          fan_stopping_ = false;
          fan_stop_next_step_ = {};
        } else {
          fan_stop_next_step_ =
              now + kFanStopPeriod;
        }
      } else {
        fan_stop_next_step_ =
            now + kFanStopPeriod;
      }

      record_failure(result);
    }
  }

  {
    std::lock_guard<std::mutex> push_lock(
        push_mutex_);

    const auto poll_push =
        [&](drivers::devices::PushMotor* motor,
            bool brush_push,
            std::string& state,
            int& last_duration_ms,
            TimedPushState& timed,
            int& extension_mm,
            double speed_mm_per_second) {
          if (motor == nullptr) {
            return;
          }

          bool deadline_reached = false;

          {
            std::lock_guard<std::mutex> state_lock(
                state_mutex_);

            deadline_reached =
                timed.direction !=
                    TimedPushDirection::kNone &&
                !timed.hold_until_stop &&
                now >= timed.deadline;
          }

          if (deadline_reached) {
            const auto stopped =
                motor->stop_motion();

            if (stopped.ok) {
              const auto stopped_at =
                  std::chrono::steady_clock::now();

              std::lock_guard<std::mutex> state_lock(
                  state_mutex_);

              update_timed_push_extension_locked(
                  extension_mm,
                  timed,
                  last_duration_ms,
                  speed_mm_per_second,
                  stopped_at);

              clear_timed_push(timed);
              state = "stopped";
              last_duration_ms = 0;

              if (brush_push) {
                push_state_ = "stopped";
                push_last_duration_ms_ = 0;
              }
            }

            record_failure(stopped);
            return;
          }

          // Recovery hold mode is intentionally controlled by the joystick:
          // it runs until the mapper sends stop (stick returns to center).
          // The normal PushMotor max_on_time watchdog would otherwise cut
          // this manual recovery after a few seconds. Normal timed pushes
          // still use poll_safety() unchanged.
          if (timed.hold_until_stop) {
            return;
          }

          const auto safety =
              motor->poll_safety();

          if (safety.ok) {
            return;
          }

          if (safety.code ==
              drivers::ErrorCode::kSafeHoldActive) {
            bool reached_endpoint = false;

            {
              const auto stopped_at =
                  std::chrono::steady_clock::now();

              std::lock_guard<std::mutex> state_lock(
                  state_mutex_);

              update_timed_push_extension_locked(
                  extension_mm,
                  timed,
                  last_duration_ms,
                  speed_mm_per_second,
                  stopped_at);

              reached_endpoint =
                  timed.direction ==
                          TimedPushDirection::kExtend
                      ? extension_mm >=
                            kPushMaxExtensionMm
                      : timed.direction ==
                                TimedPushDirection::kRetract &&
                            extension_mm <= 0;

              clear_timed_push(timed);
              state = "stopped";
              last_duration_ms = 0;

              if (brush_push) {
                push_state_ = "stopped";
                push_last_duration_ms_ = 0;
              }
            }

            if (!reached_endpoint) {
              record_failure(safety);
            }

            return;
          }

          {
            std::lock_guard<std::mutex> state_lock(
                state_mutex_);

            if (timed.direction ==
                TimedPushDirection::kNone) {
              if (state == "extend") {
                timed.direction =
                    TimedPushDirection::kExtend;
              } else if (state == "retract") {
                timed.direction =
                    TimedPushDirection::kRetract;
              }
            }

            if (timed.direction !=
                TimedPushDirection::kNone) {
              timed.deadline = now;
              timed.hold_until_stop = false;
            }
          }

          record_failure(safety);
        };

    poll_push(
        devices_.brush_push_motor,
        true,
        brush_push_state_,
        brush_push_last_duration_ms_,
        brush_timed_push_,
        brush_push_extension_mm_,
        kBrushPushSpeedMmPerSecond);

    poll_push(
        devices_.scraper_push_motor,
        false,
        scraper_push_state_,
        scraper_push_last_duration_ms_,
        scraper_timed_push_,
        scraper_push_extension_mm_,
        kScraperPushSpeedMmPerSecond);
  }

  if (have_failure) {
    return first_failure;
  }

  return drivers::Result::success();
}

ActuatorRuntimeState ActuatorService::snapshot(
    std::chrono::steady_clock::time_point now) {
  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  const int brush_remaining =
      remaining_ms(
          brush_timed_push_,
          now);

  const int scraper_remaining =
      remaining_ms(
          scraper_timed_push_,
          now);

  int brush_extension_mm =
      brush_push_extension_mm_;

  int scraper_extension_mm =
      scraper_push_extension_mm_;

  update_timed_push_extension_locked(
      brush_extension_mm,
      brush_timed_push_,
      brush_push_last_duration_ms_,
      kBrushPushSpeedMmPerSecond,
      now);

  update_timed_push_extension_locked(
      scraper_extension_mm,
      scraper_timed_push_,
      scraper_push_last_duration_ms_,
      kScraperPushSpeedMmPerSecond,
      now);

  return ActuatorRuntimeState{
      brush_state_,
      brush_speed_raw_,

      fan_active_,
      static_cast<int>(fan_left_raw_),
      static_cast<int>(fan_right_raw_),

      brush_push_state_,
      brush_remaining,
      brush_push_last_duration_ms_,
      brush_extension_mm,

      scraper_push_state_,
      scraper_remaining,
      scraper_push_last_duration_ms_,
      scraper_extension_mm,

      push_state_,
      brush_remaining,
      push_last_duration_ms_,

      climber_state_,
      climber_speed_rpm_};
}

void ActuatorService::clear_timed_push(
    TimedPushState& timed) {
  timed.direction =
      TimedPushDirection::kNone;
  timed.deadline = {};
  timed.started_at = {};
  timed.initial_extension_mm = 0;
  timed.hold_until_stop = false;
}

int ActuatorService::remaining_ms(
    const TimedPushState& timed,
    std::chrono::steady_clock::time_point now) const {
  if (timed.direction ==
          TimedPushDirection::kNone ||
      timed.hold_until_stop ||
      timed.deadline
              .time_since_epoch()
              .count() == 0 ||
      timed.deadline <= now) {
    return 0;
  }

  return static_cast<int>(
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          timed.deadline - now)
          .count());
}

void ActuatorService::
    update_timed_push_extension_locked(
        int& extension_mm,
        const TimedPushState& timed,
        int total_duration_ms,
        double speed_mm_per_second,
        std::chrono::steady_clock::time_point now) const {
  if (timed.direction ==
          TimedPushDirection::kNone ||
      (!timed.hold_until_stop && total_duration_ms <= 0) ||
      speed_mm_per_second <= 0.0) {
    return;
  }

  int elapsed_ms = 0;
  if (timed.hold_until_stop) {
    if (timed.started_at.time_since_epoch().count() != 0 &&
        now > timed.started_at) {
      elapsed_ms = static_cast<int>(
          std::min<std::int64_t>(
              std::chrono::duration_cast<
                  std::chrono::milliseconds>(now - timed.started_at).count(),
              std::numeric_limits<int>::max()));
    }
  } else {
    const int remaining = remaining_ms(timed, now);
    elapsed_ms =
        std::clamp(
            total_duration_ms - remaining,
            0,
            total_duration_ms);
  }

  const int delta_mm =
      static_cast<int>(
          std::lround(
              static_cast<double>(
                  elapsed_ms) *
              speed_mm_per_second / 1000.0));

  if (timed.direction == TimedPushDirection::kExtend) {
    const int base =
        timed.hold_until_stop
            ? timed.initial_extension_mm
            : extension_mm;
    extension_mm =
        std::clamp(
            base + delta_mm,
            0,
            kPushMaxExtensionMm);
  } else if (
      timed.direction ==
      TimedPushDirection::kRetract) {
    const int base =
        timed.hold_until_stop
            ? timed.initial_extension_mm
            : extension_mm;
    extension_mm =
        std::clamp(
            base - delta_mm,
            0,
            kPushMaxExtensionMm);
  }
}

void ActuatorService::clear_runtime_state() {
  std::lock_guard<std::mutex> state_lock(state_mutex_);

  clear_runtime_state_locked();
}

void ActuatorService::clear_runtime_state_locked() {
  fan_left_raw_ = 0;
  fan_right_raw_ = 0;

  brush_state_ = "stopped";
  brush_speed_raw_ = 0;

  fan_active_ = false;

  brush_push_state_ = "stopped";
  brush_push_last_duration_ms_ = 0;
  clear_timed_push(brush_timed_push_);

  scraper_push_state_ = "stopped";
  scraper_push_last_duration_ms_ = 0;
  clear_timed_push(scraper_timed_push_);

  push_state_ = "stopped";
  push_last_duration_ms_ = 0;

  climber_state_ = "stopped";
  climber_speed_rpm_ = 0.0;
}
}  // namespace snappy::control