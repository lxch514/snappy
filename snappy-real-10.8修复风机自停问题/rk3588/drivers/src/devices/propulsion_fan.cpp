#include "snappy/drivers/devices/propulsion_fan.hpp"

/**
 * @file propulsion_fan.cpp
 * @brief Hobbywing X6 Plus 双推进风扇后台 CAN 控制实现
 *
 * @details
 * - 程序启动后自动初始化，不等待遥控器命令。
 * - 两台 ESC 一起初始化、一起调速、一起停止。
 * - 遥控命令只更新目标缓存，CAN worker 固定 50 Hz 下发。
 * - CAN 故障时清除旧运动目标并后台重试。
 * - CAN 恢复后只恢复零油门待机，不自动恢复故障前速度。
 */

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy::drivers::devices {
namespace {

bool valid_node_id(
    std::uint8_t node_id) {
  return node_id >= 1 &&
         node_id <= 0x7D;
}

bool all_zero(
    const std::array<std::uint16_t, 4>& values) {
  return std::all_of(
      values.begin(),
      values.end(),
      [](std::uint16_t value) {
        return value == 0;
      });
}

std::uint64_t age_ms(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point then) {
  if (then.time_since_epoch().count() == 0 ||
      now <= then) {
    return 0;
  }

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          now - then)
          .count());
}

}  // namespace

PropulsionFanControllerConfig
make_propulsion_fan_config() {
  PropulsionFanControllerConfig config;

  config.profile.source_node_id =
      protocols::kHobbywingDefaultSourceNodeId;

  config.left_node_id =
      kPropulsionFanLeftNodeId;

  config.right_node_id =
      kPropulsionFanRightNodeId;

  config.left_channel_index =
      kPropulsionFanLeftChannelIndex;

  config.right_channel_index =
      kPropulsionFanRightChannelIndex;

  config.max_throttle_raw =
      protocols::kHobbywingMaxThrottleRaw;

  return config;
}

PropulsionFanController::PropulsionFanController(
    ICanBus& can_bus,
    PropulsionFanControllerConfig config)
    : can_bus_(can_bus),
      config_(std::move(config)) {
  left_cache_.snapshot.node_id =
      config_.left_node_id;

  right_cache_.snapshot.node_id =
      config_.right_node_id;
}

PropulsionFanController::~PropulsionFanController() {
  stop();
}

bool PropulsionFanController::left_enabled()
    const noexcept {
  return config_.left_node_id != 0;
}

bool PropulsionFanController::right_enabled()
    const noexcept {
  return config_.right_node_id != 0;
}

Result PropulsionFanController::validate_config()
    const {
  const bool have_left =
      left_enabled();

  const bool have_right =
      right_enabled();

  if (config_.device_id.empty() ||
      config_.profile.source_node_id == 0 ||
      (!have_left && !have_right) ||
      (have_left &&
       !valid_node_id(
           config_.left_node_id)) ||
      (have_right &&
       !valid_node_id(
           config_.right_node_id)) ||
      config_.left_channel_index >= 4 ||
      config_.right_channel_index >= 4 ||
      (have_left &&
       have_right &&
       config_.left_channel_index ==
           config_.right_channel_index) ||
      config_.max_throttle_raw == 0 ||
      config_.max_throttle_raw >
          protocols::kHobbywingMaxThrottleRaw ||
      config_.command_period.count() <= 0 ||
      config_.command_gap.count() < 0 ||
      config_.zero_throttle_hold.count() <= 0 ||
      config_.retry_backoff.count() <= 0 ||
      config_.startup_wait.count() <= 0) {
    return Result::failure(
        ErrorCode::kConfigRequired,
        "propulsion fan configuration is invalid",
        "validate",
        config_.device_id);
  }

  return Result::success();
}

Result PropulsionFanController::validate_throttle(
    std::uint16_t throttle_raw) const {
  if (throttle_raw >
          config_.max_throttle_raw ||
      throttle_raw >
          protocols::kHobbywingMaxThrottleRaw) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "propulsion fan throttle is out of range",
        "set_target",
        config_.device_id);
  }

  return Result::success();
}

std::uint8_t
PropulsionFanController::next_transfer_id()
    noexcept {
  const auto current =
      transfer_id_;

  transfer_id_ =
      static_cast<std::uint8_t>(
          (transfer_id_ + 1U) &
          protocols::kHobbywingMaxTransferId);

  return current;
}

Result PropulsionFanController::send_encoded(
    const protocols::PropulsionFanEncodeResult& encoded) {
  if (!encoded.result.ok) {
    return encoded.result;
  }

  return can_bus_.send(
      encoded.frame);
}

Result PropulsionFanController::send_raw_once(
    const std::array<std::uint16_t, 4>& values) {
  protocols::PropulsionFanRawCommand command;

  command.profile =
      config_.profile;

  command.transfer_id =
      next_transfer_id();

  command.throttle_raw =
      values;

  return send_encoded(
      protocols::encode_propulsion_fan_raw_command(
          command));
}

bool PropulsionFanController::wait_interruptible(
    std::chrono::milliseconds duration) {
  std::unique_lock<std::mutex> lock(mutex_);

  return condition_.wait_for(
      lock,
      duration,
      [this]() {
        return !running_.load();
      });
}

Result PropulsionFanController::select_can_for_left_right() {
  /*
   * 只允许 CAN worker 调用。
   * 遥控器和界面不能通过本函数重新初始化。
   */
  if (left_enabled()) {
    const auto selected =
        send_encoded(
            protocols::encode_propulsion_fan_select_can(
                config_.profile,
                next_transfer_id(),
                config_.left_node_id));

    if (!selected.ok) {
      return selected;
    }

    if (wait_interruptible(
            config_.command_gap)) {
      return Result::failure(
          ErrorCode::kIoError,
          "propulsion fan initialization stopped",
          "select_can",
          config_.device_id,
          true);
    }
  }

  if (right_enabled()) {
    const auto selected =
        send_encoded(
            protocols::encode_propulsion_fan_select_can(
                config_.profile,
                next_transfer_id(),
                config_.right_node_id));

    if (!selected.ok) {
      return selected;
    }

    if (wait_interruptible(
            config_.command_gap)) {
      return Result::failure(
          ErrorCode::kIoError,
          "propulsion fan initialization stopped",
          "select_can",
          config_.device_id,
          true);
    }
  }

  return Result::success(
      "both ESCs selected CAN throttle");
}

Result PropulsionFanController::initialize_once() {
  /*
   * 资料和示例程序规定的完整初始化顺序：
   *
   * 1. 两台 ESC 分别选择 CAN 数字油门；
   * 2. 广播 Ready=0xAA；
   * 3. 连续两秒按 50 Hz 发送零油门；
   * 4. 初始化完成后继续由 worker 保持 50 Hz。
   */

  target_raw_ = {0, 0, 0, 0};
  last_sent_raw_ = {0, 0, 0, 0};

  auto result =
      select_can_for_left_right();

  if (!result.ok) {
    return result;
  }

  result =
      send_encoded(
          protocols::encode_propulsion_fan_ready_control(
              config_.profile,
              next_transfer_id(),
              true));

  if (!result.ok) {
    return result;
  }

  if (wait_interruptible(
          config_.command_gap)) {
    return Result::failure(
        ErrorCode::kIoError,
        "propulsion fan initialization stopped",
        "ready",
        config_.device_id,
        true);
  }

  const auto zero_deadline =
      std::chrono::steady_clock::now() +
      config_.zero_throttle_hold;

  auto next_zero =
      std::chrono::steady_clock::now();

  while (running_.load() &&
         std::chrono::steady_clock::now() <
             zero_deadline) {
    result =
        send_raw_once(
            {0, 0, 0, 0});

    if (!result.ok) {
      return result;
    }

    next_zero +=
        config_.command_period;

    std::unique_lock<std::mutex> lock(mutex_);

    if (condition_.wait_until(
            lock,
            next_zero,
            [this]() {
              return !running_.load();
            })) {
      return Result::failure(
          ErrorCode::kIoError,
          "propulsion fan initialization stopped",
          "zero_hold",
          config_.device_id,
          true);
    }
  }

  if (!running_.load()) {
    return Result::failure(
        ErrorCode::kIoError,
        "propulsion fan initialization stopped",
        "initialize",
        config_.device_id,
        true);
  }

  return Result::success(
      "propulsion fan initialized");
}

Result PropulsionFanController::start() {
  const auto valid =
      validate_config();

  if (!valid.ok) {
    health_ =
        DriverHealth::kUnconfigured;
    return valid;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (running_.load()) {
      if (initialized_ &&
          ready_enabled_) {
        return Result::success(
            "propulsion fan already ready");
      }

      return last_result_;
    }

    first_attempt_done_ = false;
    initialized_ = false;
    ready_enabled_ = false;
    ready_requested_ = true;

    target_raw_ = {0, 0, 0, 0};
    last_sent_raw_ = {0, 0, 0, 0};
    transfer_id_ = 0;

    last_result_ =
        Result::failure(
            ErrorCode::kConfigRequired,
            "propulsion fan initializing",
            "start",
            config_.device_id,
            true);
  }

  health_ = DriverHealth::kStopped;
  running_.store(true);

  worker_ =
      std::thread(
          [this]() {
            worker_loop();
          });

  std::unique_lock<std::mutex> lock(mutex_);

  const bool completed =
      condition_.wait_for(
          lock,
          config_.startup_wait,
          [this]() {
            return first_attempt_done_ ||
                   !running_.load();
          });

  if (!completed) {
    return Result::failure(
        ErrorCode::kCanTxTimeout,
        "propulsion fan startup initialization timeout",
        "start",
        config_.device_id,
        true);
  }

  return last_result_;
}

Result PropulsionFanController::enter_can_standby() {
  /*
   * 兼容原有 fan.prepare 调用。
   * start() 已完成初始化，本函数只检查状态，
   * 绝不因为遥控命令重新执行两秒初始化。
   */
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ ||
      !ready_enabled_) {
    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "propulsion fan startup initialization "
        "is not complete",
        "standby",
        config_.device_id,
        true);
  }

  return Result::success(
      "propulsion fan already initialized");
}

Result PropulsionFanController::enable_throttle() {
  std::lock_guard<std::mutex> lock(mutex_);

  ready_requested_ = true;
  condition_.notify_all();

  if (!initialized_ ||
      !ready_enabled_) {
    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "propulsion fan is not ready",
        "enable",
        config_.device_id,
        true);
  }

  return Result::success(
      "propulsion fan throttle already enabled");
}

Result PropulsionFanController::disable_throttle() {
  std::lock_guard<std::mutex> lock(mutex_);

  target_raw_ = {0, 0, 0, 0};
  ready_requested_ = false;
  condition_.notify_all();

  return Result::success(
      "propulsion fan disable queued");
}

Result PropulsionFanController::set_all_raw(
    const std::array<std::uint16_t, 4>& throttle_raw) {
  for (const auto value :
       throttle_raw) {
    const auto valid =
        validate_throttle(value);

    if (!valid.ok) {
      return valid;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!running_.load() ||
      !initialized_ ||
      !ready_enabled_ ||
      !ready_requested_) {
    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "propulsion fan startup initialization "
        "is not ready",
        "set_target",
        config_.device_id,
        true);
  }

  target_raw_ = throttle_raw;
  condition_.notify_all();

  return Result::success(
      all_zero(throttle_raw)
          ? "propulsion fan target stopped"
          : "propulsion fan target updated");
}

Result PropulsionFanController::set_left_right_raw(
    std::uint16_t left_raw,
    std::uint16_t right_raw) {
  std::array<std::uint16_t, 4> next{};

  {
    std::lock_guard<std::mutex> lock(mutex_);
    next = target_raw_;
  }

  if (left_enabled()) {
    next[config_.left_channel_index] =
        left_raw;
  }

  if (right_enabled()) {
    next[config_.right_channel_index] =
        right_raw;
  }

  return set_all_raw(next);
}

Result PropulsionFanController::set_channel_raw(
    std::size_t channel_index,
    std::uint16_t throttle_raw) {
  if (channel_index >= 4) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "propulsion fan channel out of range",
        "set_channel",
        config_.device_id);
  }

  std::array<std::uint16_t, 4> next{};

  {
    std::lock_guard<std::mutex> lock(mutex_);
    next = target_raw_;
  }

  next[channel_index] =
      throttle_raw;

  return set_all_raw(next);
}

Result PropulsionFanController::stop_all() {
  return set_all_raw(
      {0, 0, 0, 0});
}

void PropulsionFanController::mark_link_failure(
    const Result& result) {
  /*
   * 链路故障会清零油门，必须留下可归因的日志。
   */
  diagnostics::warn(
      "propulsion.fan",
      "CAN link failure, throttle cleared: op=" +
          result.operation +
          " code=" +
          error_code_name(result.code) +
          " detail=" + result.detail);

  std::lock_guard<std::mutex> lock(mutex_);

  /*
   * CAN 恢复后不能自动恢复故障前的运动。
   * 目标速度立即清零，必须重新推动右摇杆。
   */
  initialized_ = false;
  ready_enabled_ = false;

  target_raw_ = {0, 0, 0, 0};
  last_sent_raw_ = {0, 0, 0, 0};

  last_result_ = result;
  health_ = DriverHealth::kFault;
}

void PropulsionFanController::update_telemetry(
    const protocols::PropulsionFanTelemetry& telemetry) {
  std::lock_guard<std::mutex> lock(mutex_);

  EscCache* cache = nullptr;

  if (telemetry.node_id ==
      config_.left_node_id) {
    cache = &left_cache_;
  } else if (
      telemetry.node_id ==
      config_.right_node_id) {
    cache = &right_cache_;
  }

  if (cache == nullptr) {
    return;
  }

  cache->snapshot.online = true;
  cache->received_at =
      std::chrono::steady_clock::now();

  if (telemetry.rpm) {
    cache->snapshot.rpm =
        telemetry.rpm;
  }

  if (telemetry.throttle_raw) {
    cache->snapshot.throttle_raw =
        telemetry.throttle_raw;
  }

  if (telemetry.status) {
    cache->snapshot.status =
        telemetry.status;
  }

  if (telemetry.voltage_v) {
    cache->snapshot.voltage_v =
        telemetry.voltage_v;
  }

  if (telemetry.current_a) {
    cache->snapshot.current_a =
        telemetry.current_a;
  }

  if (telemetry.temperature_celsius) {
    cache->snapshot.temperature_celsius =
        telemetry.temperature_celsius;
  }

  if (telemetry.mos_temperature_celsius) {
    cache->snapshot.mos_temperature_celsius =
        telemetry.mos_temperature_celsius;
  }

  if (telemetry.capacitor_temperature_celsius) {
    cache->snapshot.capacitor_temperature_celsius =
        telemetry.capacitor_temperature_celsius;
  }

  if (telemetry.motor_temperature_celsius) {
    cache->snapshot.motor_temperature_celsius =
        telemetry.motor_temperature_celsius;
  }
}

void PropulsionFanController::poll_telemetry_once() {
  auto* frame_bus =
      dynamic_cast<bus::ICanFrameBus*>(
          &can_bus_);

  if (frame_bus == nullptr) {
    return;
  }

  bus::CanReceiveOptions options;

  /*
   * 忽略优先级，只匹配 Message Type ID 和 ESC Source Node ID。
   */
  constexpr std::uint32_t kMessageAndNodeMask =
      0x00FFFF7FU;

  const std::uint16_t message_types[] = {
      protocols::kHobbywingMsg1DataTypeId,
      protocols::kHobbywingMsg2DataTypeId,
      protocols::kHobbywingMsg3DataTypeId,
  };

  const std::uint8_t node_ids[] = {
      config_.left_node_id,
      config_.right_node_id,
  };

  for (const auto message_type :
       message_types) {
    for (const auto node_id :
         node_ids) {
      if (node_id == 0) {
        continue;
      }

      options.filters.push_back(
          {
              static_cast<std::uint32_t>(
                  (static_cast<std::uint32_t>(
                       message_type)
                   << 8U) |
                  node_id),
              kMessageAndNodeMask,
              true,
          });
    }
  }

  options.timeout =
      std::chrono::milliseconds(1);

  /*
   * 一个控制周期最多读 4 帧，避免两台 ESC 的高频报文挤掉有效遥测。
   */
  constexpr int kMaxFramesPerCycle = 4;

  for (int index = 0;
       index < kMaxFramesPerCycle;
       ++index) {
    const auto received =
        frame_bus->receive(options);

    if (!received.result.ok) {
      return;
    }

    const auto decoded =
        protocols::decode_propulsion_fan_telemetry(
            received.frame);

    if (decoded.result.ok) {
      update_telemetry(
          decoded.telemetry);
    }
  }
}

void PropulsionFanController::worker_loop() {
  diagnostics::info(
      "propulsion.fan",
      "CAN worker started");

  while (running_.load()) {
    if (!initialized_) {
      const auto initialized =
          initialize_once();

      {
        std::lock_guard<std::mutex> lock(mutex_);

        first_attempt_done_ = true;
        last_result_ = initialized;

        if (initialized.ok) {
          initialized_ = true;
          ready_enabled_ = true;
          ready_requested_ = true;

          target_raw_ = {0, 0, 0, 0};
          last_sent_raw_ = {0, 0, 0, 0};

          health_ =
              DriverHealth::kReady;
        } else {
          initialized_ = false;
          ready_enabled_ = false;

          target_raw_ = {0, 0, 0, 0};
          last_sent_raw_ = {0, 0, 0, 0};

          health_ =
              DriverHealth::kFault;
        }
      }

      condition_.notify_all();

      if (!initialized.ok) {
        diagnostics::warn(
            "propulsion.fan",
            "startup initialization failed; "
            "other devices remain available: " +
                initialized.detail);

        if (wait_interruptible(
                config_.retry_backoff)) {
          break;
        }

        continue;
      }

      diagnostics::info(
          "propulsion.fan",
          "both ESCs initialized; "
          "50 Hz zero throttle active");
    }

    auto next_send =
        std::chrono::steady_clock::now();

    while (running_.load()) {
      std::array<std::uint16_t, 4> target{};
      bool ready_requested = true;
      bool initialized = false;
      bool ready_enabled = false;

      {
        std::lock_guard<std::mutex> lock(mutex_);

        target = target_raw_;
        ready_requested =
            ready_requested_;
        initialized =
            initialized_;
        ready_enabled =
            ready_enabled_;
      }

      if (!initialized) {
        break;
      }

      if (!ready_requested &&
          ready_enabled) {
        const auto zero =
            send_raw_once(
                {0, 0, 0, 0});

        if (!zero.ok) {
          mark_link_failure(zero);
          break;
        }

        const auto disabled =
            send_encoded(
                protocols::encode_propulsion_fan_ready_control(
                    config_.profile,
                    next_transfer_id(),
                    false));

        if (!disabled.ok) {
          mark_link_failure(
              disabled);
          break;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          ready_enabled_ = false;
          last_sent_raw_ = {0, 0, 0, 0};
          last_result_ = disabled;
        }
      }

      if (ready_requested &&
          !ready_enabled) {
        /*
         * 只用于显式 fan.enable 恢复。
         * 正常右摇杆启动不会进入这里，因为开机后 Ready 一直保持。
         */
        const auto enabled =
            send_encoded(
                protocols::encode_propulsion_fan_ready_control(
                    config_.profile,
                    next_transfer_id(),
                    true));

        if (!enabled.ok) {
          mark_link_failure(
              enabled);
          break;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          ready_enabled_ = true;
          target_raw_ = {0, 0, 0, 0};
          target = {0, 0, 0, 0};
          last_result_ = enabled;
        }
      }

      if (!ready_enabled) {
        std::unique_lock<std::mutex> lock(mutex_);

        condition_.wait_for(
            lock,
            config_.command_period,
            [this]() {
              return !running_.load() ||
                     ready_requested_;
            });

        continue;
      }

      const auto sent =
          send_raw_once(target);

      if (!sent.ok) {
        mark_link_failure(sent);
        break;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_sent_raw_ = target;
        last_result_ = sent;
      }

      /*
       * 读取本周期内到达的主动上报，总耗时不超过 4 ms。
       */
      poll_telemetry_once();

      next_send +=
          config_.command_period;

      const auto now =
          std::chrono::steady_clock::now();

      if (next_send <= now) {
        next_send =
            now +
            config_.command_period;
      }

      std::unique_lock<std::mutex> lock(mutex_);

      condition_.wait_until(
          lock,
          next_send,
          [this]() {
            return !running_.load();
          });
    }
  }

  send_shutdown_sequence();

  diagnostics::info(
      "propulsion.fan",
      "CAN worker stopped");
}

void PropulsionFanController::send_shutdown_sequence()
    noexcept {
  /*
   * 进程退出时发送多帧零油门，再发送 Ready=0x55。
   * 普通 fan.stop 和安全暂停只保持 Ready+零油门，
   * 后续重新向上推动右摇杆可以立即启动。
   */
  for (int index = 0;
       index < 5;
       ++index) {
    static_cast<void>(
        send_raw_once(
            {0, 0, 0, 0}));

    std::this_thread::sleep_for(
        config_.command_period);
  }

  static_cast<void>(
      send_encoded(
          protocols::encode_propulsion_fan_ready_control(
              config_.profile,
              next_transfer_id(),
              false)));
}

void PropulsionFanController::safe_stop()
    noexcept {
  std::lock_guard<std::mutex> lock(mutex_);

  /*
   * 安全停止立即清空目标。
   * worker 下一周期发送零油门，最大软件延迟约 20 ms。
   * 保持 Ready，故障清除后无需重新初始化即可由遥控启动。
   */
  target_raw_ = {0, 0, 0, 0};
  condition_.notify_all();
}

void PropulsionFanController::stop()
    noexcept {
  const bool was_running =
      running_.exchange(false);

  condition_.notify_all();

  if (worker_.joinable()) {
    worker_.join();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    initialized_ = false;
    ready_enabled_ = false;
    ready_requested_ = false;

    target_raw_ = {0, 0, 0, 0};
    last_sent_raw_ = {0, 0, 0, 0};

    if (was_running) {
      last_result_ =
          Result::success(
              "propulsion fan stopped");
    }
  }

  health_ = DriverHealth::kStopped;
}

DriverHealth PropulsionFanController::health()
    const noexcept {
  return health_.load();
}

std::array<std::uint16_t, 4>
PropulsionFanController::last_command()
    const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return target_raw_;
}

bool PropulsionFanController::throttle_enabled()
    const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);

  return initialized_ &&
         ready_enabled_ &&
         ready_requested_;
}

PropulsionFanSnapshot
PropulsionFanController::snapshot() const {
  const auto now =
      std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);

  PropulsionFanSnapshot out;

  out.worker_running =
      running_.load();

  out.initialized =
      initialized_;

  out.ready_enabled =
      ready_enabled_;

  out.target_raw =
      target_raw_;

  out.last_sent_raw =
      last_sent_raw_;

  out.left =
      left_cache_.snapshot;

  out.right =
      right_cache_.snapshot;

  out.left.age_ms =
      age_ms(
          now,
          left_cache_.received_at);

  out.right.age_ms =
      age_ms(
          now,
          right_cache_.received_at);

  /*
   * 超过 1 秒没有主动上报，只把实际反馈标记为离线。
   * 不影响目标缓存，也不阻塞其他设备。
   */
  if (out.left.age_ms > 1000) {
    out.left.online = false;
  }

  if (out.right.age_ms > 1000) {
    out.right.online = false;
  }

  out.last_result =
      last_result_;

  return out;
}

}  // namespace snappy::drivers::devices