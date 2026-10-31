/**
 * @file actuator_hub.cpp
 * @brief 无传感器执行器控制中心实现
 *
 * @details
 * - 执行器动作统一调用原 ActuatorService。
 * - 控制线程只写最新值邮箱或驱动目标缓存。
 * - RS485 物理事务只发生在执行器 worker 和驱动内部。
 * - 每个控制域拥有独立 worker。
 * - generation 用于阻止安全停止前的旧命令复活。
 * - status/telemetry 只读取内存快照。
 */

#include "snappy_snappy/actuator_hub.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <array>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy_snappy {

using nlohmann::json;

using snappy::control::ActuatorRuntimeState;
using snappy::control::ActuatorServiceDevices;
using snappy::control::ActuatorServiceOptions;
using snappy::control::DeviceConfig;
using snappy::drivers::DriverHealth;
using snappy::drivers::ErrorCode;
using snappy::drivers::Result;
using snappy::drivers::bus::Rs485DeviceBinding;
using snappy::drivers::devices::BrushMotorConfig;
using snappy::drivers::devices::ClimberConfig;
using snappy::drivers::devices::PropulsionFanControllerConfig;
using snappy::drivers::devices::PushMotorConfig;
using snappy::drivers::devices::RelayControllerConfig;

namespace {

const DeviceConfig* find_device(
    const snappy::control::HardwareConfig& config,
    const char* device_id) {
  const auto iterator =
      config.devices.find(device_id);

  if (iterator == config.devices.end() ||
      !iterator->second.enabled) {
    return nullptr;
  }

  return &iterator->second;
}

std::uint16_t relay_channel(
    const snappy::control::HardwareConfig& config,
    const char* output_id,
    std::uint16_t fallback) {
  const auto iterator =
      config.relay_outputs.find(output_id);

  if (iterator == config.relay_outputs.end() ||
      iterator->second.physical_channel == 0) {
    return fallback;
  }

  return iterator->second.physical_channel;
}

int json_int(
    const json& value,
    const char* name,
    int fallback) {
  if (!value.is_object() ||
      !value.contains(name)) {
    return fallback;
  }

  try {
    const auto& item =
        value.at(name);

    if (item.is_number_integer() ||
        item.is_number_unsigned()) {
      return item.get<int>();
    }

    if (item.is_number_float()) {
      return static_cast<int>(
          std::llround(
              item.get<double>()));
    }

    if (item.is_string()) {
      return std::stoi(
          item.get<std::string>(),
          nullptr,
          0);
    }
  } catch (...) {
  }

  return fallback;
}

double json_double(
    const json& value,
    const char* name,
    double fallback) {
  if (!value.is_object() ||
      !value.contains(name)) {
    return fallback;
  }

  try {
    const auto& item =
        value.at(name);

    if (item.is_number()) {
      return item.get<double>();
    }

    if (item.is_string()) {
      return std::stod(
          item.get<std::string>());
    }
  } catch (...) {
  }

  return fallback;
}

std::uint32_t duration_ms(
    const json& params,
    std::uint32_t fallback,
    std::uint32_t maximum) {
  const int value =
      json_int(
          params,
          "duration_ms",
          static_cast<int>(
              fallback));

  /*
   * 不下发时长（停止类命令，fallback = 0）必须原样返回 0。
   *
   * 不能落到下面的 clamp 里：100 的下限会把 0 顶成 100ms，
   * 让停止命令变成一次 3.3mm 的误动作。
   */
  if (value <= 0) {
    return 0U;
  }

  /*
   * maximum == 0 表示调用方未指定上限，使用默认上限 10000ms。
   *
   * 绝不能把 0 直接当作上限传给 std::clamp：lo(100) > hi(0) 是
   * 未定义行为，libstdc++ 实测会返回 lo，导致所有时长被压成 100ms，
   * 表现为推杆长推无论按多久都只走 3mm 左右。
   */
  const std::uint32_t effective_maximum =
      maximum == 0U
          ? 10000U
          : std::max(maximum, 100U);

  return std::clamp(
      static_cast<std::uint32_t>(value),
      100U,
      effective_maximum);
}

[[maybe_unused]] bool is_stop_action(
    const std::string& action) {
  return action == "brush.stop" ||
         action == "brush_push.stop" ||
         action == "scraper_push.stop" ||
         action == "push.stop" ||
         action == "climber.stop" ||
         action == "fan.stop" ||
         action == "fan.disable";
}

}  // namespace

ActuatorHub::ActuatorHub(
    RuntimeConfig config)
    : config_(std::move(config)) {
  rs485_bus_ =
      std::make_unique<DirectRs485Bus>(
          config_.hardware.rs485_ports);

  /*
   * 只为启用的 RS485 执行器建立逻辑绑定。
   *
   * 一个逻辑设备拥有独立 BindingState。
   * 某个设备失败时只累计自身失败次数，
   * 不关闭同一物理端口上的其他设备。
   */
  for (const auto& item :
       config_.hardware.devices) {
    const auto& device =
        item.second;

    if (!device.enabled ||
        config_.hardware.rs485_ports.find(
            device.bus_id) ==
            config_.hardware.rs485_ports.end()) {
      continue;
    }

    Rs485DeviceBinding binding;
    binding.logical_device =
        device.device_id;
    binding.port_id =
        device.bus_id;
    binding.timeout =
        device.response_deadline;
    binding.max_response_bytes =
        256;
    binding.max_consecutive_failures =
        device.max_consecutive_failures;
    binding.quarantine_duration =
        device.quarantine_duration;
    binding.read_retry_count =
        device.read_retry_count;
    binding.priority =
        device.priority;

    /*
     * 配置加载阶段已经验证端口存在。
     * 这里仍然保留失败隔离，避免构造阶段异常影响其他设备。
     */
    static_cast<void>(
        rs485_bus_->add_binding(
            binding));
  }

  const auto* relay_device =
      find_device(
          config_.hardware,
          "relay_push_motor");

  if (relay_device != nullptr) {
    RelayControllerConfig relay_config =
        snappy::drivers::devices::
            make_relay_push_motor_config();

    if (relay_device->station_id) {
      relay_config.station_id =
          *relay_device->station_id;
    }

    if (relay_device->channel_count) {
      relay_config.profile.channel_count =
          *relay_device->channel_count;
    }

    relay_ =
        std::make_unique<
            snappy::drivers::devices::
                RelayController>(
            *rs485_bus_,
            relay_config);

    relay_config_ = relay_config;

    PushMotorConfig brush_push_config =
        snappy::drivers::devices::
            make_push_motor_config();

    brush_push_config.device_id =
        "brush_push_motor";

    brush_push_config.extend_channel =
        relay_channel(
            config_.hardware,
            "brush_push_extend",
            1);

    brush_push_config.retract_channel =
        relay_channel(
            config_.hardware,
            "brush_push_retract",
            2);

    brush_push_config.dead_time =
        config_.hardware.safety
            .push_motor_dead_time;

    brush_push_config.max_on_time =
        config_.hardware.safety
            .brush_push_motor_max_on_time;

    brush_push_ =
        std::make_unique<
            snappy::drivers::devices::
                PushMotor>(
            *relay_,
            brush_push_config);

    PushMotorConfig scraper_push_config =
        snappy::drivers::devices::
            make_push_motor_config();

    scraper_push_config.device_id =
        "scraper_push_motor";

    scraper_push_config.extend_channel =
        relay_channel(
            config_.hardware,
            "scraper_push_extend",
            3);

    scraper_push_config.retract_channel =
        relay_channel(
            config_.hardware,
            "scraper_push_retract",
            4);

    scraper_push_config.dead_time =
        config_.hardware.safety
            .push_motor_dead_time;

    scraper_push_config.max_on_time =
        config_.hardware.safety
            .scraper_push_motor_max_on_time;

    scraper_push_ =
        std::make_unique<
            snappy::drivers::devices::
                PushMotor>(
            *relay_,
            scraper_push_config);
  }

  const auto* brush_device =
      find_device(
          config_.hardware,
          "brush_motor");

  if (brush_device != nullptr) {
    BrushMotorConfig brush_config =
        snappy::drivers::devices::
            make_brush_motor_config();

    if (brush_device->station_id) {
      brush_config.station_id =
          *brush_device->station_id;
    }

    brush_config.direction_dead_time =
        config_.hardware.safety
            .push_motor_dead_time;

    brush_ =
        std::make_unique<
            snappy::drivers::devices::
                BrushMotor>(
            *rs485_bus_,
            brush_config);
    
    brush_config_ = brush_config;
  }

  const auto* climber_device =
      find_device(
          config_.hardware,
          "climber");

  if (climber_device != nullptr) {
    ClimberConfig climber_config =
        snappy::drivers::devices::
            make_climber_config();

    if (climber_device->station_id) {
      climber_config.station_id =
          *climber_device->station_id;
    }

    climber_ =
        std::make_unique<
            snappy::drivers::devices::
                Climber>(
            *rs485_bus_,
            climber_config);
    
    climber_config_ = climber_config;
  }

  const auto* fan_device =
      find_device(
          config_.hardware,
          "propulsion_fan");

  if (fan_device != nullptr) {
    const auto can_iterator =
        config_.hardware.can_buses.find(
            fan_device->bus_id);

    if (can_iterator !=
        config_.hardware.can_buses.end()) {
      can_bus_ =
          std::make_unique<
              snappy::drivers::bus::
                  SocketCanBus>(
              can_iterator->second);

      PropulsionFanControllerConfig
          fan_config =
              snappy::drivers::devices::
                  make_propulsion_fan_config();

      if (fan_device->source_node_id) {
        fan_config.profile.source_node_id =
            *fan_device->source_node_id;
      }

      if (fan_device->left_node_id) {
        fan_config.left_node_id =
            *fan_device->left_node_id;
      }

      if (fan_device->right_node_id) {
        fan_config.right_node_id =
            *fan_device->right_node_id;
      }

      if (fan_device->left_throttle_channel) {
        fan_config.left_channel_index =
            *fan_device->left_throttle_channel;
      }

      if (fan_device->right_throttle_channel) {
        fan_config.right_channel_index =
            *fan_device->right_throttle_channel;
      }

      if (fan_device->max_throttle_raw) {
        fan_config.max_throttle_raw =
            *fan_device->max_throttle_raw;
      }

      fan_ =
          std::make_unique<
              snappy::drivers::devices::
                  PropulsionFanController>(
              *can_bus_,
              fan_config);
    }
  }

  ActuatorServiceDevices devices;
  devices.brush_motor = brush_.get();
  devices.propulsion_fan = fan_.get();
  devices.brush_push_motor = brush_push_.get();
  devices.scraper_push_motor = scraper_push_.get();

  /*
   * 保持原工程兼容：
   * push.* 始终指向毛刷推杆。
   */
  devices.push_motor = brush_push_.get();

  devices.climber = climber_.get();

  ActuatorServiceOptions options;
  options.noop_missing_actuators = config_.hardware.bringup.noop_missing_actuators;

  options.enable_recovery_worker = false;

/*
 * ActuatorHub 已经负责总线隔离和快照发布。
 * ActuatorService 仍负责原有动作恢复逻辑。
 */
  options.ignore_startup_failures =
    config_.hardware.bringup
        .ignore_device_startup_failures;

  actuator_service_ =
      std::make_unique<
          snappy::control::ActuatorService>(
          devices,
          options);

  snapshot_.brush_state = "stopped";
  snapshot_.brush_push_state = "stopped";
  snapshot_.scraper_push_state = "stopped";
  snapshot_.push_state = "stopped";
  snapshot_.climber_state = "stopped";
  snapshot_.fan_active = false;
}

ActuatorHub::~ActuatorHub() {
  stop();
}

Result ActuatorHub::start() {
  if (started_.load()) {
    return Result::success(
        "actuator hub already started");
  }

  bool partial = false;

  Result first_failure = Result::success();

  const auto record =
      [&partial, &first_failure](
          const Result& result) {
        if (result.ok) {
          return;
        }

        partial = true;

        if (first_failure.ok) {
          first_failure = result;
        }
      };

    record(rs485_bus_->start());

    if (relay_) {
      DirectRs485Bus::Scope scope(
          *rs485_bus_,
          Rs485AccessKind::kRecovery,
          std::chrono::milliseconds(5000));

      const auto relay_result = relay_->start();
      record(relay_result);

      if (!relay_result.ok) {
        SNAPPY_DRIVER_LOG_WARN(
            "snappy-snappy",
            "relay startup failed: " +
                relay_result.detail);
      }
    }

  if (can_bus_) {
    record(can_bus_->start());
  }

  const auto service_result = actuator_service_->start();

  record(service_result);

  if (partial &&
      !config_.hardware.bringup
          .allow_partial_devices) {
    actuator_service_->safe_stop_all();
    actuator_service_->stop();

    if (can_bus_) {
      can_bus_->stop();
    }

    rs485_bus_->stop();

    return first_failure.ok
               ? first_failure
               : Result::failure(
                     ErrorCode::kIoError,
                     "actuator startup failed",
                     "start",
                     "actuator_hub");
  }

  const auto driver_is_ready =
      [](const auto* driver) noexcept {
        return driver != nullptr &&
               driver->health() ==
                   DriverHealth::kReady;
      };

  const bool rs485_ready =
      rs485_bus_->health() ==
      DriverHealth::kReady;

  const bool can_ready =
      can_bus_ != nullptr &&
      can_bus_->health() ==
          DriverHealth::kReady;

  /*
   * 逻辑设备 Ready 必须同时满足：
   * 1. 对应驱动已完成本地启动；
   * 2. 对应物理总线当前已打开。
   *
   * 总线状态不能直接冒充所有逻辑设备状态。
   */
  const bool relay_ready =
      rs485_ready &&
      driver_is_ready(relay_.get());

  const bool brush_push_ready =
      relay_ready &&
      driver_is_ready(brush_push_.get());

  const bool scraper_push_ready =
      relay_ready &&
      driver_is_ready(scraper_push_.get());

  const bool brush_ready =
      rs485_ready &&
      driver_is_ready(brush_.get());

  const bool climber_ready =
      rs485_ready &&
      driver_is_ready(climber_.get());

  const bool fan_ready =
      can_ready &&
      driver_is_ready(fan_.get());

  const bool configured_actuator =
      relay_ != nullptr ||
      brush_push_ != nullptr ||
      scraper_push_ != nullptr ||
      brush_ != nullptr ||
      climber_ != nullptr ||
      fan_ != nullptr;

  /*
   * 部分调试模式下，只要至少一个已配置执行器可用，
   * 就允许取得控制权后进入 arm。
   * 缺失执行器的命令仍由对应驱动返回失败，
   * 不会阻塞其他执行器。
   */
  const bool any_actuator_ready =
      configured_actuator &&
      (relay_ready ||
       brush_push_ready ||
       scraper_push_ready ||
       brush_ready ||
       climber_ready ||
       fan_ready);

  const bool all_devices_ready =
      configured_actuator &&
      (!relay_ || relay_ready) &&
      (!brush_push_ || brush_push_ready) &&
      (!scraper_push_ || scraper_push_ready) &&
      (!brush_ || brush_ready) &&
      (!climber_ || climber_ready) &&
      (!fan_ || fan_ready);

  const bool arm_ready =
      config_.hardware.bringup
          .arm_requires_all_required_devices
          ? all_devices_ready
          : any_actuator_ready;

  ready_for_arm_.store(arm_ready);

  /*
   * 启动后先执行一次安全停止。
   * 该动作只发生在启动阶段，不属于控制请求路径。
   */
  actuator_service_->safe_stop_all();

  started_.store(true);
  running_.store(true);

  {
    std::lock_guard<std::mutex> lock(
        snapshot_mutex_);

    snapshot_.started = true;
    snapshot_.worker_running = true;
    snapshot_.partial_start = partial;
    snapshot_.ready_for_arm = arm_ready;
    snapshot_.rs485_ready = rs485_ready;
    snapshot_.can_ready = can_ready;
    snapshot_.relay_ready = relay_ready;
    snapshot_.brush_ready = brush_ready;
    snapshot_.climber_ready = climber_ready;
    snapshot_.fan_ready = fan_ready;
    snapshot_.updated_at_ms = now_ms();

    if (!first_failure.ok) {
      snapshot_.last_error =
          first_failure.detail;
    }
  }

  try {
    brush_thread_ =
        std::thread(
            [this]() {
              brush_loop();
            });

    push_thread_ =
        std::thread(
            [this]() {
              push_loop();
            });

    climber_thread_ =
        std::thread(
            [this]() {
              climber_loop();
            });

    maintenance_thread_ =
        std::thread(
            [this]() {
              maintenance_loop();
            });

    health_thread_ =
        std::thread(
            [this]() {
              health_loop();
            });

  } catch (...) {
    running_.store(false);
    mailbox_.close();

    if (brush_thread_.joinable()) {
      brush_thread_.join();
    }

    if (push_thread_.joinable()) {
      push_thread_.join();
    }

    if (climber_thread_.joinable()) {
      climber_thread_.join();
    }

    if (maintenance_thread_.joinable()) {
      maintenance_thread_.join();
    }

    actuator_service_->safe_stop_all();
    actuator_service_->stop();

    if (can_bus_) {
      can_bus_->stop();
    }

    rs485_bus_->stop();
    started_.store(false);

    return Result::failure(
        ErrorCode::kIoError,
        "failed to create actuator workers",
        "start",
        "actuator_hub");
  }

  if (partial) {
    return Result::success(
        "actuator hub started with isolated failures");
  }

  return Result::success(
      "actuator hub ready");
}

void ActuatorHub::stop() noexcept {
  const bool was_started =
      started_.exchange(false);

  if (!was_started &&
      !running_.load()) {
    /*
     * 即使启动过程失败，也尝试关闭已经创建的驱动。
     */
    if (actuator_service_) {
      actuator_service_->stop();
    }

    if (can_bus_) {
      can_bus_->stop();
    }

    if (rs485_bus_) {
      rs485_bus_->stop();
    }

    return;
  }

  safe_stop();

  running_.store(false);
  mailbox_.close();

  if (brush_thread_.joinable()) {
    brush_thread_.join();
  }

  if (push_thread_.joinable()) {
    push_thread_.join();
  }

  if (climber_thread_.joinable()) {
    climber_thread_.join();
  }

  if (maintenance_thread_.joinable()) {
    maintenance_thread_.join();
  }

  if (health_thread_.joinable()) {
    health_thread_.join();
  }

  /*
   * worker 退出后再执行一次同步安全停止，
   * 防止最后一笔正在结束的动作留下目标值。
   */
  if (actuator_service_) {
    actuator_service_->safe_stop_all();
    actuator_service_->stop();
  }

  if (can_bus_) {
    can_bus_->stop();
  }

  if (rs485_bus_) {
    rs485_bus_->stop();
  }

  ready_for_arm_.store(false);

  {
    std::lock_guard<std::mutex> lock(
        gate_mutex_);

    accepting_commands_ = false;
    safe_stop_pending_ = false;
    fault_clear_pending_ = false;
  }

  {
    std::lock_guard<std::mutex> lock(
        snapshot_mutex_);

    snapshot_.started = false;
    snapshot_.worker_running = false;
    snapshot_.partial_start = false;
    snapshot_.ready_for_arm = false;
    snapshot_.command_gate_open = false;
    snapshot_.safe_stop_pending = false;
    snapshot_.rs485_ready = false;
    snapshot_.can_ready = false;
    snapshot_.relay_ready = false;
    snapshot_.brush_ready = false;
    snapshot_.climber_ready = false;
    snapshot_.fan_ready = false;
    snapshot_.fan_worker_running = false;
    snapshot_.fan_online = false;
    snapshot_.fan_active = false;
    snapshot_.updated_at_ms = now_ms();
  }
}

Result ActuatorHub::submit(
    const std::string& action,
    const json& params) {
  if (action == "fault.clear") {
    return request_fault_clear();
  }

  if (!started_.load()) {
    return Result::failure(
        ErrorCode::kConfigRequired,
        "actuator hub is not started",
        "submit",
        action);
  }

  std::uint64_t generation = 0;

  {
    std::lock_guard<std::mutex> lock(
        gate_mutex_);

    if (!accepting_commands_) {
      return Result::failure(
          ErrorCode::kSafeHoldActive,
          "actuator command gate is closed",
          "submit",
          action);
    }

    generation = generation_;
  }

  PendingCommand command;
  command.generation = generation;

  CommandSlot slot;

  if (action == "brush.forward" ||
      action == "brush.reverse" ||
      action == "brush.stop") {
    slot = CommandSlot::kBrush;

    command.direction =
        action == "brush.forward"
            ? 1
            : action == "brush.reverse"
                  ? -1
                  : 0;

    command.speed_raw =
        std::clamp(
            json_int(
                params,
                "speed_raw",
                15),
            0,
            15);

    if (!mailbox_.submit(
            slot,
            command)) {
      return Result::failure(
          ErrorCode::kSafeHoldActive,
          "brush mailbox is closed",
          "submit",
          action);
    }

    return Result::success(
        "brush command accepted");
  }

  if (action == "brush_push.extend" ||
      action == "brush_push.retract" ||
      action == "brush_push.extend_timed" ||
      action == "brush_push.extend_recovery" ||
      action == "brush_push.retract_timed" ||
      action == "brush_push.retract_recovery" ||
      action == "brush_push.stop" ||
      action == "push.extend" ||
      action == "push.retract" ||
      action == "push.extend_timed" ||
      action == "push.retract_timed" ||
      action == "push.stop") {
    slot = CommandSlot::kBrushPush;

    const bool extend =
        action == "brush_push.extend" ||
        action == "brush_push.extend_timed" ||
        action == "brush_push.extend_recovery" ||
        action == "push.extend" ||
        action == "push.extend_timed";

    const bool retract =
        action == "brush_push.retract" ||
        action == "brush_push.retract_timed" ||
        action == "brush_push.retract_recovery" ||
        action == "push.retract" ||
        action == "push.retract_timed";

    command.direction = extend ? 1 : retract ? -1 : 0;

    const bool timed =
        action == "brush_push.extend_timed" ||
        action == "brush_push.extend_recovery" ||
        action == "brush_push.retract_timed" ||
        action == "brush_push.retract_recovery" ||
        action == "push.extend_timed" ||
        action == "push.retract_timed";

    command.timed = timed;
    command.force_retract_from_zero =
        action == "brush_push.retract_recovery";
    command.force_extend_beyond_max =
        action == "brush_push.extend_recovery";
    command.hold_until_stop =
        action == "brush_push.extend_recovery" ||
        action == "brush_push.retract_recovery";

    command.duration_ms =
        duration_ms(
            params,
            timed ? 1000U : 0U,
            0U);

    if (!mailbox_.submit(
            slot,
            command)) {
      return Result::failure(
          ErrorCode::kSafeHoldActive,
          "brush push mailbox is closed",
          "submit",
          action);
    }

    return Result::success(
        "brush push command accepted");
  }

  if (action == "scraper_push.extend" ||
      action == "scraper_push.retract" ||
      action == "scraper_push.extend_timed" ||
      action == "scraper_push.extend_recovery" ||
      action == "scraper_push.retract_timed" ||
      action == "scraper_push.retract_recovery" ||
      action == "scraper_push.stop") {
    slot = CommandSlot::kScraperPush;

    const bool extend =
        action == "scraper_push.extend" ||
        action == "scraper_push.extend_timed" ||
        action == "scraper_push.extend_recovery";

    const bool retract =
        action == "scraper_push.retract" ||
        action == "scraper_push.retract_timed" ||
        action == "scraper_push.retract_recovery";

    command.direction =
        extend ? 1 : retract ? -1 : 0;

    command.timed =
        action == "scraper_push.extend_timed" ||
        action == "scraper_push.extend_recovery" ||
        action == "scraper_push.retract_timed" ||
        action == "scraper_push.retract_recovery";

    command.force_retract_from_zero =
        action == "scraper_push.retract_recovery";
    command.force_extend_beyond_max =
        action == "scraper_push.extend_recovery";
    command.hold_until_stop =
        action == "scraper_push.extend_recovery" ||
        action == "scraper_push.retract_recovery";

    command.duration_ms =
        duration_ms(
            params,
            command.timed ? 1000U : 0U,
            0U);

    if (!mailbox_.submit(
            slot,
            command)) {
      return Result::failure(
          ErrorCode::kSafeHoldActive,
          "scraper push mailbox is closed",
          "submit",
          action);
    }

    return Result::success(
        "scraper push command accepted");
  }

    /*
   * 爬绳器命令使用独立邮箱。
   * 同一时刻只保留最后一条爬绳器命令，
   * 不积累 FIFO 队列。
   */
  if (action == "climber.ascend" ||
      action == "climber.descend" ||
      action == "climber.stop") {
    slot = CommandSlot::kClimber;

    command.direction =
        action == "climber.ascend"
            ? 1
            : action == "climber.descend"
                  ? -1
                  : 0;

    if (command.direction != 0) {
      if (params.contains("speed_rpm")) {
        command.speed_rpm =
            json_double(
                params,
                "speed_rpm",
                500.0);
      } else if (
          params.contains("speed_rad_per_s")) {
        command.speed_rpm =
            json_double(
                params,
                "speed_rad_per_s",
                500.0);
      } else {
        command.speed_rpm =
            json_double(
                params,
                "speed_rps",
                500.0);
      }
    }

    if (!mailbox_.submit(
            slot,
            command)) {
      return Result::failure(
          ErrorCode::kSafeHoldActive,
          "climber mailbox is closed",
          "submit",
          action);
    }

    return Result::success(
        "climber command accepted");
  }

  /*
   * 风机命令只修改 ActuatorService 的目标缓存。
   * CAN 物理发送仍由 PropulsionFanController
   * 内部 worker 以固定周期执行。
   *
   * begin_command/end_command 用于与安全停止协调，
   * 防止安全停止和风机目标更新并发时产生竞态。
   */
  if (!begin_command(generation)) {
    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "actuator command was cancelled",
        "submit",
        action);
  }

  Result result =
      Result::failure(
          ErrorCode::kUnsupported,
          "unsupported actuator action",
          "submit",
          action);

  try {
    if (action == "fan.prepare") {
      result =
          actuator_service_->fan_prepare();
    } else if (action == "fan.enable") {
      result =
          actuator_service_->fan_enable();
    } else if (action == "fan.disable") {
      result =
          actuator_service_->fan_disable();
    } else if (action == "fan.stop") {
      result =
          actuator_service_->fan_stop_all();
    } else if (action == "fan.left_set") {
      result =
          actuator_service_->fan_left_set(
              json_int(
                  params,
                  "speed_raw",
                  0));
    } else if (action == "fan.right_set") {
      result =
          actuator_service_->fan_right_set(
              json_int(
                  params,
                  "speed_raw",
                  0));
    } else if (action == "fan.both_set") {
      /*
       * 保持原有双风机同步控制方式。
       * 两台风机仍由同一次服务调用同时更新。
       */
      result =
          actuator_service_->fan_both_set(
              json_int(
                  params, "left_raw",
                  0),
              json_int(
                  params,
                  "right_raw",
                  0));
    }
  } catch (...) {
    result =
        Result::failure(
            ErrorCode::kIoError,
            "fan command threw an exception",
            "submit",
            action);
  }

  end_command();
  update_result(result);
  return result;
}

Result ActuatorHub::enable_commands() {
  std::lock_guard<std::mutex> lock(
      gate_mutex_);

  if (!started_.load()) {
    return Result::failure(
        ErrorCode::kConfigRequired,
        "actuator hub is not started",
        "enable_commands",
        "actuator_hub");
  }

  if (!ready_for_arm_.load()) {
    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "actuator hardware is not ready",
        "enable_commands",
        "actuator_hub");
  }

  /*
   * 安全停止和故障清除期间禁止重新取得控制权。
   */
  if (safe_stop_pending_ ||
      fault_clear_pending_) {
    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "actuator safety transition is pending",
        "enable_commands",
        "actuator_hub");
  }

  accepting_commands_ = true;

  {
    std::lock_guard<std::mutex>
        snapshot_lock(snapshot_mutex_);

    snapshot_.command_gate_open = true;
    snapshot_.safe_stop_pending =
        safe_stop_pending_;
    ++snapshot_.sequence;
    snapshot_.updated_at_ms = now_ms();
  }

  return Result::success(
      "actuator command gate opened");
}

bool ActuatorHub::wait_for_safety_clear(
    std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(gate_mutex_);

  return safety_cv_.wait_for(lock, timeout, [this] {
    return !safe_stop_pending_ &&
           !fault_clear_pending_;
  });
}

void ActuatorHub::safe_stop() noexcept {
  /*
   * 先关闭命令门并递增代次。
   * 已经取出但尚未执行的旧命令会因代次失效。
   */
  {
    std::lock_guard<std::mutex> lock(gate_mutex_);

    accepting_commands_ = false;
    ++generation_;
    safe_stop_pending_ = true;

    /*
     * 清除邮箱中还没有执行的普通命令，
     * 同时通知 maintenance worker 执行最终物理停止。
     */
    mailbox_.request_urgent_stop();
  }

  /*
   * 总线安全停止只取消当前事务的后续重试，
   * 不清除设备隔离状态。
   */
  if (rs485_bus_) {
    rs485_bus_->safe_stop();
  }

  /*
   * 风机安全停止只清零目标缓存，
   * 实际 CAN 发送由风机自己的 worker 完成。
   */
  if (fan_) {
    fan_->safe_stop();
  }

  {
    std::lock_guard<std::mutex>
        snapshot_lock(snapshot_mutex_);

    snapshot_.command_gate_open = false;
    snapshot_.safe_stop_pending = true;
    snapshot_.fan_active = false;
    ++snapshot_.sequence;
    snapshot_.updated_at_ms = now_ms();
  }
}

Result ActuatorHub::request_fault_clear() {
  if (!started_.load()) {
    return Result::failure(
        ErrorCode::kConfigRequired,
        "actuator hub is not started",
        "fault.clear",
        "actuator_hub");
  }

  std::uint64_t generation = 0;

  {
    std::lock_guard<std::mutex> lock(
        gate_mutex_);

    if (fault_clear_pending_) {
      return Result::failure(
          ErrorCode::kSafeHoldActive,
          "fault clear is already pending",
          "fault.clear",
          "actuator_hub");
    }

    accepting_commands_ = false;
    ++generation_;
    generation = generation_;

    safe_stop_pending_ = true;
    fault_clear_pending_ = true;

    /*
     * 清除旧的普通命令，优先执行安全停止。
     */
    mailbox_.request_urgent_stop();
  }

  /*
   * 故障清除不仅清除设备内部错误，
   * 还要同步清除 DirectRs485Bus 的逻辑设备隔离缓存。
   *
   * 该函数只操作内存状态，不执行串口 I/O。
   */
  if (rs485_bus_) {
    rs485_bus_->clear_faults();
  }

  if (fan_) {
    fan_->safe_stop();
  }

  PendingCommand command;
  command.generation = generation;

  if (!mailbox_.submit(
          CommandSlot::kClearFaults,
          command)) {
    {
      std::lock_guard<std::mutex>
          lock(gate_mutex_);

      fault_clear_pending_ = false;
      safe_stop_pending_ = false;
    }

    {
      std::lock_guard<std::mutex>
          snapshot_lock(snapshot_mutex_);

      snapshot_.safe_stop_pending = false;
      ++snapshot_.sequence;
      snapshot_.updated_at_ms =
          now_ms();
    }

    return Result::failure(
        ErrorCode::kSafeHoldActive,
        "fault clear mailbox is closed",
        "fault.clear",
        "actuator_hub");
  }

  {
    std::lock_guard<std::mutex>
        snapshot_lock(snapshot_mutex_);

    snapshot_.command_gate_open = false;
    snapshot_.safe_stop_pending = true;
    ++snapshot_.sequence;
    snapshot_.updated_at_ms = now_ms();
  }

  return Result::success(
      "fault clear accepted");
}

bool ActuatorHub::begin_command(
    std::uint64_t generation) {
  std::lock_guard<std::mutex> lock(
      gate_mutex_);

  if (!accepting_commands_ ||
      generation != generation_) {
    return false;
  }

  ++active_commands_;
  return true;
}

void ActuatorHub::end_command() {
  std::lock_guard<std::mutex> lock(
      gate_mutex_);

  if (active_commands_ == 0) {
    return;
  }

  --active_commands_;

  if (active_commands_ == 0) {
    active_cv_.notify_all();
  }
}

void ActuatorHub::wait_for_active_commands() {
  std::unique_lock<std::mutex> lock(
      gate_mutex_);

  active_cv_.wait(
      lock,
      [this]() {
        return active_commands_ == 0;
      });
}

void ActuatorHub::execute(
    CommandSlot slot,
    const PendingCommand& command) noexcept {
  /*
   * 安全停止和新命令之间通过 generation 隔离。
   * 旧代次命令即使已经从邮箱取出，也不会执行。
   */
  if (!begin_command(
          command.generation)) {
    return;
  }

  Result result =
    Result::failure(
        ErrorCode::kUnsupported,
        "command was not executed");

  std::unique_ptr<DirectRs485Bus::Scope>
      rs485_scope;

  if (rs485_bus_) {
    rs485_scope =
      std::make_unique<DirectRs485Bus::Scope>(
          *rs485_bus_,
          Rs485AccessKind::kCommand,
          std::chrono::milliseconds(4000));
  }

  try {
    switch (slot) {
      case CommandSlot::kBrush:
        if (command.direction > 0) {
          result = actuator_service_->brush_forward(command.speed_raw);
        } else if (
            command.direction < 0) {
          result = actuator_service_->brush_reverse(command.speed_raw);
        } else {
          result = actuator_service_->brush_stop();
        }
        break;

      case CommandSlot::kBrushPush:
        if (command.direction > 0) {
          if (command.timed) {
            result =
                actuator_service_
                    ->brush_push_extend_for(
                        std::chrono::milliseconds(
                            command.duration_ms),
                        command.force_extend_beyond_max,
                        command.hold_until_stop);
          } else {
            result =
                actuator_service_
                    ->brush_push_extend();
          }
        } else if (
            command.direction < 0) {
          if (command.timed) {
            result =
                actuator_service_
                    ->brush_push_retract_for(
                        std::chrono::milliseconds(
                            command.duration_ms),
                        command.force_retract_from_zero,
                        command.hold_until_stop);
          } else {
            result =
                actuator_service_
                    ->brush_push_retract();
          }
        } else {
          result =
              actuator_service_
                  ->brush_push_stop();
        }
        break;

      case CommandSlot::kScraperPush:
        if (command.direction > 0) {
          if (command.timed) {
            result =
                actuator_service_
                    ->scraper_push_extend_for(
                        std::chrono::milliseconds(
                            command.duration_ms),
                        command.force_extend_beyond_max,
                        command.hold_until_stop);
          } else {
            result =
                actuator_service_
                    ->scraper_push_extend();
          }
        } else if (
            command.direction < 0) {
          if (command.timed) {
            result =
                actuator_service_
                    ->scraper_push_retract_for(
                        std::chrono::milliseconds(
                            command.duration_ms),
                        command.force_retract_from_zero,
                        command.hold_until_stop);
          } else {
            result =
                actuator_service_
                    ->scraper_push_retract();
          }
        } else {
          result =
              actuator_service_
                  ->scraper_push_stop();
        }
        break;

      case CommandSlot::kClimber:
        if (command.direction > 0) {
          result =
              actuator_service_->climber_ascend(command.speed_rpm);
        } else if (
            command.direction < 0) {
          result =
              actuator_service_->climber_descend(command.speed_rpm);
        } else {
          result = actuator_service_->climber_stop();
        }
        break;

      case CommandSlot::kClearFaults:
        /*
         * 清除故障由 maintenance worker 串行处理，
         * 避免与安全停止或其他执行器动作并发。
         */
        result =
            Result::success(
                "fault clear handled by maintenance worker");
        break;
    }
  } catch (...) {
    result =
        Result::failure(
            ErrorCode::kIoError,
            "actuator command threw an exception",
            "execute",
            "actuator_hub");
  }

  end_command();
  update_result(result);

  publish_snapshot();
}

void ActuatorHub::brush_loop() noexcept {
  while (running_.load()) {
    PendingCommand command;

    if (!mailbox_.wait_take(
            CommandSlot::kBrush,
            &command,
            std::chrono::milliseconds(50))) {
      continue;
    }

    execute(
        CommandSlot::kBrush,
        command);
  }
}

void ActuatorHub::push_loop() noexcept {
  /*
   * 毛刷推杆和刮板推杆共用一个逻辑 worker，
   * 但各自仍有独立最新值槽。
   *
   * 每次轮询优先级交替，防止某一路持续发送时
   * 长时间得不到执行机会。
   */
  bool prefer_brush = true;

  while (running_.load()) {
    std::optional<PendingCommand>
        command;
    CommandSlot selected = CommandSlot::kBrushPush;

    if (prefer_brush) {
      command = mailbox_.take(CommandSlot::kBrushPush);

      if (command) {
        selected = CommandSlot::kBrushPush;
      } else {
        command = mailbox_.take(CommandSlot::kScraperPush);

        if (command) {
          selected = CommandSlot::kScraperPush;
        }
      }
    } else {
      command = mailbox_.take(CommandSlot::kScraperPush);

      if (command) {
        selected = CommandSlot::kScraperPush;
      } else {
        command = mailbox_.take(CommandSlot::kBrushPush);

        if (command) {
          selected = CommandSlot::kBrushPush;
        }
      }
    }

    if (!command) {
      prefer_brush = !prefer_brush;

      std::this_thread::sleep_for(
          std::chrono::milliseconds(5));
      continue;
    }

    prefer_brush = selected != CommandSlot::kBrushPush;

    execute(
        selected,
        *command);
  }
}

void ActuatorHub::climber_loop() noexcept {
  while (running_.load()) {
    PendingCommand command;

    if (!mailbox_.wait_take(
            CommandSlot::kClimber,
            &command,
            std::chrono::milliseconds(50))) {
      continue;
    }

    execute(
        CommandSlot::kClimber,
        command);
  }
}

void ActuatorHub::execute_safe_stop() noexcept {
  Result result =
      Result::success(
          "actuators safely stopped");

  try {
    if (actuator_service_) {
      /*
       * 安全停止本身使用 safety priority。
       *
       * 注意：
       * Scope 会记录底层所有 RS485 transaction 的失败。
       *
       * 但是：
       *
       *   brush_motor 单设备 timeout
       *
       * 不应该把整个 ActuatorHub 锁死。
       *
       * safe_stop_all() 自己只把 climber 的关键安全停止
       * 作为整体失败条件。
       */
      DirectRs485Bus::Scope scope(
          *rs485_bus_,
          Rs485AccessKind::kSafety,
          std::chrono::milliseconds(15000));

      result =
          actuator_service_->safe_stop_all();

      /*
       * 不再使用 scope.result() 覆盖一个成功的
       * safe_stop_all()。
       *
       * 原来的逻辑：
       *
       *   brush timeout
       *       ↓
       *   scope.result() = failure
       *       ↓
       *   safe_stop_pending_ = true
       *       ↓
       *   风机 / climber / 其他模块全部无法重新控制
       *
       * 这正是当前故障表现。
       *
       * 现在单个 logical device 的 timeout
       * 由 DirectRs485Bus 自己隔离。
       */
      static_cast<void>(scope);
    }
  } catch (...) {
    result =
        Result::failure(
            ErrorCode::kIoError,
            "safe stop threw an exception",
            "safe_stop",
            "actuator_hub");
  }

  update_result(result);

  {
    std::lock_guard<std::mutex>
        lock(gate_mutex_);

    safe_stop_pending_ =
        !result.ok;
  }

  safety_cv_.notify_all();

  {
    std::lock_guard<std::mutex>
        lock(snapshot_mutex_);

    snapshot_.safe_stop_pending =
        !result.ok;

    ++snapshot_.sequence;
    snapshot_.updated_at_ms =
        now_ms();
  }
}

void ActuatorHub::maintenance_loop() noexcept {
  while (running_.load()) {
    /*
     * 普通执行器由独立 worker 消费。
     * maintenance worker 只处理：
     * 1. 紧急安全停止；
     * 2. 故障清除；
     * 3. ActuatorService 的定时状态推进。
     */
    mailbox_.wait_for_event(
        std::chrono::milliseconds(10));

    if (mailbox_.take_urgent_stop()) {
      wait_for_active_commands();
      execute_safe_stop();
    }

    const auto clear =
        mailbox_.take(
            CommandSlot::kClearFaults);

        if (clear) {
      /*
       * 故障清除必须发生在最终安全停止之后，
       * 避免旧动作和清除动作同时操作同一设备。
       */
      wait_for_active_commands();

      Result result =
          Result::success(
              "faults cleared");

      try {
        if (actuator_service_) {
          DirectRs485Bus::Scope scope(
              *rs485_bus_,
              Rs485AccessKind::kSafety,
              std::chrono::milliseconds(15000));

          result =
              actuator_service_->clear_faults();

          if (result.ok &&
              !scope.result().ok) {
            result =
                scope.result();
          }
        }
      } catch (...) {
        result =
            Result::failure(
                ErrorCode::kIoError,
                "fault clear threw an exception",
                "fault.clear",
                "actuator_hub");
      }

      update_result(result);

      {
        std::lock_guard<std::mutex> lock(gate_mutex_);
        
        fault_clear_pending_ = false;

        if (result.ok) {
          safe_stop_pending_ = false;
          accepting_commands_ = true;
        } else {
          safe_stop_pending_ = true;
          accepting_commands_ = false;
        }
      }

      safety_cv_.notify_all();

      {
       std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);

       snapshot_.command_gate_open = result.ok;
       snapshot_.safe_stop_pending = !result.ok;

      ++snapshot_.sequence;
      snapshot_.updated_at_ms = now_ms();
      }
    }

    try {
      if (actuator_service_) {
        const auto result = actuator_service_->tick();

        if (!result.ok) {
          update_result(result);
        }
      }
    } catch (...) {
      update_result(
          Result::failure(
              ErrorCode::kIoError,
              "actuator tick threw an exception",
              "tick",
              "actuator_hub"));
    }
  }
}

void ActuatorHub::update_result(
    const Result& result) {
  if (result.ok) {
    return;
  }

  {
    std::lock_guard<std::mutex>
        lock(snapshot_mutex_);

    snapshot_.last_error =
        result.detail.empty()
            ? "actuator operation failed"
            : result.detail;
    ++snapshot_.sequence;
    snapshot_.updated_at_ms =
        now_ms();
  }

  SNAPPY_DRIVER_LOG_WARN(
      "snappy-snappy",
      result.detail.empty()
          ? "actuator operation failed"
          : result.detail);
}

ActuatorSnapshot ActuatorHub::snapshot() const {
  std::lock_guard<std::mutex> lock(
      snapshot_mutex_);

  return snapshot_;
}

void ActuatorHub::health_loop() noexcept {
  using namespace std::chrono_literals;

  /*
   * 后台健康轮询只保留：
   *
   *   1. relay_push_motor
   *   2. brush_motor
   *
   * 爬绳器不再由 health_loop 周期性读取：
   *
   *   - read_alarm()
   *   - read_actual_speed_raw()
   *   - recover_climber()
   *
   */
  const std::array<const char*, 2> devices{
      "relay_push_motor",
      "brush_motor"};

  std::array<Clock::time_point, 2>
      next_poll{};

  std::size_t index = 0;

  while (running_.load()) {
    const std::size_t current =
        index;

    index =
        (index + 1) % devices.size();

    const bool present =
        current == 0
            ? relay_ != nullptr
            : brush_ != nullptr;

    if (!present ||
        Clock::now() < next_poll[current]) {
      std::this_thread::sleep_for(5ms);
      continue;
    }

    Result probe =
        Result::success();

    try {
      const char* id =
          devices[current];

      const auto cached =
          rs485_bus_->device_status(id);

      /*
       * 设备处于 quarantine 时不主动发送后台探测。
       *
       * 控制命令和安全停止仍然可以进入调度器。
       */
      if (cached.quarantine_until != 0) {
        next_poll[current] =
            Clock::now() + 100ms;

        std::this_thread::sleep_for(5ms);
        continue;
      }

      {
        DirectRs485Bus::Scope scope(
            *rs485_bus_,
            Rs485AccessKind::kHealth,
            800ms);

        if (current == 0) {
          /*
           * Relay 健康检查保持原逻辑。
           */
          const auto readback =
              relay_->read_all();

          probe =
              readback.result;

          if (probe.ok &&
              readback.energized.size() == 4) {
            std::lock_guard<std::mutex>
                lock(snapshot_mutex_);

            std::copy(
                readback.energized.begin(),
                readback.energized.end(),
                snapshot_.relay_outputs.begin());
          }
        } else {
          /*
           * Brush 健康检查保持原逻辑。
           */
          const auto fault =
              brush_->read_fault();

          probe =
              fault.result;

          if (probe.ok &&
              fault.raw_fault.has_value()) {
            std::lock_guard<std::mutex>
                lock(snapshot_mutex_);

            snapshot_.brush_fault_raw =
                static_cast<int>(
                    *fault.raw_fault);
          }
        }

        if (probe.ok &&
            !scope.result().ok) {
          probe =
              scope.result();
        }
      }

      bool recovery_allowed = false;

      {
        std::lock_guard<std::mutex>
            lock(gate_mutex_);

        recovery_allowed =
            !safe_stop_pending_ &&
            !fault_clear_pending_;
      }

      if (probe.ok &&
          recovery_allowed &&
          running_.load()) {
        bool needs_recovery = false;

        if (current == 0) {
          needs_recovery =
              (brush_push_ &&
               brush_push_->health() !=
                   DriverHealth::kReady) ||
              (scraper_push_ &&
               scraper_push_->health() !=
                   DriverHealth::kReady) ||
              (relay_ &&
               relay_->health() !=
                   DriverHealth::kReady);
        } else {
          needs_recovery =
              brush_->health() !=
              DriverHealth::kReady;
        }

        if (needs_recovery) {
          DirectRs485Bus::Scope scope(
              *rs485_bus_,
              Rs485AccessKind::kRecovery,
              5000ms);

          if (current == 0) {
            probe =
                actuator_service_
                    ->recover_relay();
          } else {
            probe =
                actuator_service_
                    ->recover_brush_motor();
          }

          if (probe.ok &&
              !scope.result().ok) {
            probe =
                scope.result();
          }
        }
      }

      const auto status =
          rs485_bus_->device_status(id);

      next_poll[current] =
         Clock::now() +
         (!probe.ok ||
                 status.quarantine_until != 0
             ? 3000ms
             : 1000ms);

      if (!probe.ok &&
          probe.operation != "arbitration" &&
          probe.code !=
              ErrorCode::kSafeHoldActive) {
        update_result(probe);
      }

      publish_snapshot();
    } catch (...) {
      next_poll[current] =
          Clock::now() + 2000ms;

      update_result(
          Result::failure(
              ErrorCode::kIoError,
              "RS485 health iteration failed",
              "health",
              devices[current]));
    }

    /*
     * 给其他 worker / RS485 控制事务让出时间。
     */
    std::this_thread::sleep_for(15ms);
  }
}

void ActuatorHub::publish_snapshot() {
  if (!rs485_bus_) {
    return;
  }

  const auto relay_status = rs485_bus_->device_status("relay_push_motor");

  const auto brush_status = rs485_bus_->device_status("brush_motor");

  const auto climber_status = rs485_bus_->device_status("climber");

  const auto runtime =
      actuator_service_
          ? actuator_service_->snapshot()
          : ActuatorRuntimeState{};

  const auto fan_snapshot =
      fan_
          ? fan_->snapshot()
          : snappy::drivers::devices::
                PropulsionFanSnapshot{};

  const bool relay_initialized =
      relay_ &&
      relay_->health() ==
          DriverHealth::kReady;

  const bool brush_initialized =
      brush_ &&
      brush_->health() ==
          DriverHealth::kReady;

  const bool brush_push_initialized =
      brush_push_ &&
      brush_push_->health() ==
          DriverHealth::kReady;

  const bool scraper_push_initialized =
      scraper_push_ &&
      scraper_push_->health() ==
          DriverHealth::kReady;

  const bool climber_initialized =
      climber_ &&
      climber_->health() ==
          DriverHealth::kReady;

  const bool can_ready =
      can_bus_ &&
      can_bus_->health() ==
          DriverHealth::kReady;

  const bool relay_ready = relay_initialized && relay_status.online;

  const bool brush_ready = brush_initialized && brush_status.online;

  const bool climber_ready = climber_initialized && climber_status.online;

  const bool brush_push_ready = brush_push_initialized && relay_status.online;

  const bool scraper_push_ready = scraper_push_initialized && relay_status.online;

  const bool any_ready =
      relay_initialized ||
      brush_initialized ||
      brush_push_initialized ||
      scraper_push_initialized ||
      climber_initialized ||
      fan_snapshot.initialized;

  const bool all_ready =
      (!relay_ || relay_initialized) &&
      (!brush_ || brush_initialized) &&
      (!brush_push_ || brush_push_initialized) &&
      (!scraper_push_ || scraper_push_initialized) &&
      (!climber_ || climber_initialized) &&
      (!fan_ || fan_snapshot.initialized);

  const bool arm_ready =
      config_.hardware.bringup
          .arm_requires_all_required_devices
          ? all_ready
          : any_ready;

  ready_for_arm_.store(arm_ready);

  const bool rs485_ready =
    rs485_bus_->health() ==
    DriverHealth::kReady;

  std::lock_guard<std::mutex> lock(snapshot_mutex_);

  snapshot_.rs485_ready = rs485_ready;

  snapshot_.can_ready = can_ready;

  snapshot_.relay_status = relay_status;
  snapshot_.brush_status = brush_status;
  snapshot_.climber_status = climber_status;

  snapshot_.relay_ready = relay_ready;
  snapshot_.brush_ready = brush_ready;
  snapshot_.climber_ready = climber_ready;
  snapshot_.brush_push_ready = brush_push_ready;
  snapshot_.scraper_push_ready = scraper_push_ready;
  snapshot_.fan_ready = fan_snapshot.initialized && can_ready;

  snapshot_.fan_worker_running = fan_snapshot.worker_running;

  snapshot_.fan_online = fan_snapshot.left.online || fan_snapshot.right.online;

  snapshot_.brush_state = runtime.brush_state;
  snapshot_.brush_speed_raw = runtime.brush_speed_raw;

  snapshot_.fan_active = runtime.fan_active;
  snapshot_.fan_left_raw = runtime.fan_left_raw;
  snapshot_.fan_right_raw = runtime.fan_right_raw;

  snapshot_.brush_push_state = runtime.brush_push_state;
  snapshot_.brush_push_remaining_ms = runtime.brush_push_remaining_ms;
  snapshot_.brush_push_last_duration_ms = runtime.brush_push_last_duration_ms;
  snapshot_.brush_push_extension_mm = runtime.brush_push_extension_mm;

  snapshot_.scraper_push_state = runtime.scraper_push_state;
  snapshot_.scraper_push_remaining_ms = runtime.scraper_push_remaining_ms;
  snapshot_.scraper_push_last_duration_ms = runtime.scraper_push_last_duration_ms;
  snapshot_.scraper_push_extension_mm = runtime.scraper_push_extension_mm;

  snapshot_.push_state = runtime.push_state;
  snapshot_.push_extension_mm = runtime.brush_push_extension_mm;

  snapshot_.climber_state = runtime.climber_state;
  snapshot_.climber_speed_rpm =
      static_cast<int>(
          std::llround(
              runtime.climber_speed_rpm));

  snapshot_.ready_for_arm = arm_ready;
  snapshot_.updated_at_ms = now_ms();
  ++snapshot_.sequence;
}

std::uint64_t ActuatorHub::now_ms() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::system_clock::now()
              .time_since_epoch())
          .count());
}

}  // namespace snappy_snappy