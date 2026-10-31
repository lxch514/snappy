/**
 * @file gateway_command_dispatcher.cpp
 * @brief 整机网关命令调度实现
 *
 * @details
 * - 管理 acquire、arm、disarm、lease、心跳和安全状态。
 * - acquire 只取得控制权，不能直接执行非停止动作。
 * - status / telemetry 只读取 SensorService 缓存快照。
 * - dispatcher 状态锁不覆盖 CAN、RS485 或执行器 I/O。
 * - 调试模式允许故障设备隔离，其他可用设备继续工作。
 */

#include "snappy/control/gateway_command_dispatcher.hpp"

#include "snappy/control/sensor_service.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace snappy::control {
namespace {

using json = nlohmann::json;

constexpr std::uint16_t kBrushMaxSpeedRaw = 15;

/*
 * 风机基准转速必须与 gateway 侧 G20ButtonMapper 的 kFanBaseRaw 保持一致。
 *
 * 现场实测：G20 遥控器启动风机时下发 fan.both_set 的 left_raw/right_raw
 * 为 500（见 telemetry "fan_left_raw":500 / "fan_right_raw":500）；
 * g20_button_mapper.hpp 中 kFanBaseRaw = 100，
 * 而 dispatcher 这里曾一度被改成 800，三处数值互相矛盾。
 * 本常量只用于 capabilities 上报（base_raw），不参与实际限速；
 * 实际转速完全由命令参数决定，这里恢复为 100，与 HEAD 及 mapper 对齐。
 */
constexpr std::uint16_t kFanBaseRaw = 100;
constexpr std::uint16_t kFanStepRaw = 400;
constexpr std::uint16_t kFanMaxRaw = 8191;

constexpr std::int64_t kBrushPushMaxDurationMs = 9100;
constexpr std::int64_t kScraperPushMaxDurationMs = 10000;

/* ============================================================
 * 时间、状态和字符串转换
 * ============================================================ */

std::int64_t now_ms() {
  using namespace std::chrono;

  return duration_cast<milliseconds>(
             system_clock::now().time_since_epoch())
      .count();
}

std::string link_id_to_string(LinkId link_id) {
  switch (link_id) {
    case LinkId::kG20Udp:
      return "g20_udp";

    case LinkId::kEc20Mqtt:
      return "ec20_mqtt";
  }

  return "unknown";
}

LinkId link_id_from_string(
    const std::string& value) {
  return value == "ec20_mqtt"
             ? LinkId::kEc20Mqtt
             : LinkId::kG20Udp;
}

std::string control_state_to_string(
    ControlState state) {
  switch (state) {
    case ControlState::kBoot:
      return "BOOT";
    case ControlState::kSelfTest:
      return "SELF_TEST";
    case ControlState::kDisarmed:
      return "DISARMED";
    case ControlState::kAcquired:
      return "ACQUIRED";
    case ControlState::kArmedIdle:
      return "ARMED_IDLE";
    case ControlState::kActive:
      return "ACTIVE";
    case ControlState::kSafeHold:
      return "SAFE_HOLD";
    case ControlState::kFaultLatched:
      return "FAULT_LATCHED";
    case ControlState::kEstopLatched:
      return "ESTOP_LATCHED";
  }

  return "FAULT_LATCHED";
}

std::string ack_status_to_string(
    AckStatus status) {
  switch (status) {
    case AckStatus::kAccepted:
      return "ACCEPTED";
    case AckStatus::kRunning:
      return "RUNNING";
    case AckStatus::kSucceeded:
      return "SUCCEEDED";
    case AckStatus::kFailed:
      return "FAILED";
    case AckStatus::kAborted:
      return "ABORTED";
    case AckStatus::kTimedOut:
      return "TIMED_OUT";
  }

  return "FAILED";
}

std::string sample_quality_to_string(
    drivers::SampleQuality quality) {
  switch (quality) {
    case drivers::SampleQuality::kGood:
      return "GOOD";
    case drivers::SampleQuality::kStale:
      return "STALE";
    case drivers::SampleQuality::kInvalid:
      return "INVALID";
    case drivers::SampleQuality::kUnknown:
    default:
      return "UNKNOWN";
  }
}

std::string sensor_communication_health_to_string(
    SensorCommunicationHealth health) {
  switch (health) {
    case SensorCommunicationHealth::kGood:
      return "GOOD";

    case SensorCommunicationHealth::kDegraded:
      return "DEGRADED";

    case SensorCommunicationHealth::kFault:
      return "FAULT";

    case SensorCommunicationHealth::kUnknown:
    default:
      return "UNKNOWN";
  }
}

/* ============================================================
 * JSON 参数读取
 * ============================================================ */

std::uint16_t param_u16(
    const json& params,
    const std::string& key,
    std::uint16_t fallback) {
  if (!params.is_object() ||
      !params.contains(key)) {
    return fallback;
  }

  const auto& value = params.at(key);

  try {
    if (value.is_number_unsigned()) {
      return static_cast<std::uint16_t>(
          std::min<std::uint64_t>(
              value.get<std::uint64_t>(),
              65535));
    }

    if (value.is_number_integer()) {
      return static_cast<std::uint16_t>(
          std::clamp<std::int64_t>(
              value.get<std::int64_t>(),
              0,
              65535));
    }

    if (value.is_string()) {
      const auto parsed =
          std::stoul(value.get<std::string>());

      return static_cast<std::uint16_t>(
          std::min<unsigned long>(
              parsed,
              65535));
    }
  } catch (...) {
  }

  return fallback;
}

std::int64_t param_i64(
    const json& params,
    const std::string& key,
    std::int64_t fallback) {
  if (!params.is_object() ||
      !params.contains(key)) {
    return fallback;
  }

  const auto& value = params.at(key);

  try {
    if (value.is_number_integer()) {
      return value.get<std::int64_t>();
    }

    if (value.is_number_unsigned()) {
      const auto raw =
          value.get<std::uint64_t>();

      return static_cast<std::int64_t>(
          std::min<std::uint64_t>(
              raw,
              static_cast<std::uint64_t>(
                  std::numeric_limits<
                      std::int64_t>::max())));
    }

    if (value.is_number()) {
      return static_cast<std::int64_t>(
          value.get<double>());
    }

    if (value.is_string()) {
      return std::stoll(
          value.get<std::string>());
    }
  } catch (...) {
  }

  return fallback;
}

double param_double(
    const json& params,
    const std::string& key,
    double fallback) {
  if (!params.is_object() ||
      !params.contains(key)) {
    return fallback;
  }

  const auto& value = params.at(key);

  try {
    if (value.is_number()) {
      return value.get<double>();
    }

    if (value.is_string()) {
      return std::stod(
          value.get<std::string>());
    }
  } catch (...) {
  }

  return fallback;
}

double climber_speed_rpm(
    const json& params) {
  if (params.is_object() &&
      params.contains("speed_rpm")) {
    return param_double(
        params,
        "speed_rpm",
        0.0);
  }

  if (params.is_object() &&
      params.contains("speed_rad_per_s")) {
    return param_double(
        params,
        "speed_rad_per_s",
        0.0);
  }

  return param_double(
      params,
      "speed_rps",
      0.0);
}

std::uint64_t json_u64(
    const json& object,
    const std::string& key,
    std::uint64_t fallback = 0) {
  if (!object.is_object() ||
      !object.contains(key)) {
    return fallback;
  }

  const auto& value = object.at(key);

  try {
    if (value.is_number_unsigned()) {
      return value.get<std::uint64_t>();
    }

    if (value.is_number_integer()) {
      return static_cast<std::uint64_t>(
          std::max<std::int64_t>(
              0,
              value.get<std::int64_t>()));
    }

    if (value.is_string()) {
      return std::stoull(
          value.get<std::string>());
    }
  } catch (...) {
  }

  return fallback;
}

std::int64_t json_i64(
    const json& object,
    const std::string& key,
    std::int64_t fallback = 0) {
  if (!object.is_object() ||
      !object.contains(key)) {
    return fallback;
  }

  const auto& value = object.at(key);

  try {
    if (value.is_number_integer()) {
      return value.get<std::int64_t>();
    }

    if (value.is_number_unsigned()) {
      const auto raw =
          value.get<std::uint64_t>();

      return static_cast<std::int64_t>(
          std::min<std::uint64_t>(
              raw,
              static_cast<std::uint64_t>(
                  std::numeric_limits<
                      std::int64_t>::max())));
    }

    if (value.is_string()) {
      return std::stoll(
          value.get<std::string>());
    }
  } catch (...) {
  }

  return fallback;
}

std::string json_string(
    const json& object,
    const std::string& key,
    const std::string& fallback = "") {
  if (!object.is_object() ||
      !object.contains(key)) {
    return fallback;
  }

  const auto& value = object.at(key);

  if (value.is_string()) {
    return value.get<std::string>();
  }

  if (value.is_number_integer()) {
    return std::to_string(
        value.get<std::int64_t>());
  }

  if (value.is_number_unsigned()) {
    return std::to_string(
        value.get<std::uint64_t>());
  }

  if (value.is_boolean()) {
    return value.get<bool>()
               ? "true"
               : "false";
  }

  return fallback;
}

/* ============================================================
 * 命令分类
 * ============================================================ */

bool is_stop_action(
    const std::string& action) {
  return action == "brush.stop" ||
         action == "push.stop" ||
         action == "brush_push.stop" ||
         action == "scraper_push.stop" ||
         action == "fan.stop" ||
         action == "fan.disable" ||
         action == "climber.stop";
}

bool is_control_action(
    const std::string& action) {
  return action == "control.acquire" ||
         action == "control.arm" ||
         action == "control.disarm" ||
         action == "control.release" ||
         action == "control.heartbeat" ||
         action == "control.safe_hold" ||
         action == "control.estop" ||
         action == "control.link_down" ||
         action == "fault.clear";
}

/* ============================================================
 * 正式模式实时安全门禁
 *
 * 只读取驱动健康状态和 SensorService 缓存快照。
 * 不同步访问 CAN 或 RS485。
 * ============================================================ */

struct RequiredGateResult {
  bool ready{false};
  std::string detail;
};

template <typename Samples>
bool sensor_samples_healthy(
    const Samples& samples,
    const std::vector<std::string>& required_samples) {
  const auto sample_is_healthy =
      [](const auto& sample) {
        return sample.value.has_value() &&
               sample.quality ==
                   drivers::SampleQuality::kGood;
      };

  if (samples.empty()) {
    return false;
  }

  /*
   * 未配置 required_samples 时保持旧逻辑：
   * 所有缓存采样都必须有效。
   */
  if (required_samples.empty()) {
    return std::all_of(
        samples.begin(),
        samples.end(),
        sample_is_healthy);
  }

  /*
   * 配置了 required_samples 后，
   * 只检查指定的关键采样。
   */
  for (const auto& required_name :
       required_samples) {
    const auto it =
        std::find_if(
            samples.begin(),
            samples.end(),
            [&required_name](const auto& sample) {
              return sample.logical_name ==
                     required_name;
            });

    if (it == samples.end() ||
        !sample_is_healthy(*it)) {
      return false;
    }
  }

  return true;
}

RequiredGateResult evaluate_required_gate(
    DeviceRegistry& registry,
    const WatchdogStatus& watchdog) {
  const auto hardware =
      registry.hardware_ports();

  if (hardware.can.health() !=
          drivers::DriverHealth::kReady ||
      hardware.rs485.health() !=
          drivers::DriverHealth::kReady ||
      hardware.relays.health() !=
          drivers::DriverHealth::kReady ||
      hardware.sensors.health() !=
          drivers::DriverHealth::kReady) {
    return {
        false,
        "one or more required hardware backends "
        "are not ready"};
  }

  const auto samples =
      registry.sensors().cached_samples();

  const auto& required_samples =
      registry.config()
          .safety
          .required_samples;

  if (!sensor_samples_healthy(
          samples,
          required_samples)) {
    return {
        false,
        "required sensor snapshot is unknown, "
        "invalid or stale"};
  }

  const auto devices =
      registry.sensors().device_statuses();

  if (devices.empty()) {
    return {
        false,
        "required device health snapshot is empty"};
  }

  const bool devices_ready =
      std::all_of(
          devices.begin(),
          devices.end(),
          [](const auto& device) {
            return device.online &&
                   device.communication_health ==
                       SensorCommunicationHealth::kGood;
          });

  if (!devices_ready) {
    return {
        false,
        "one or more required devices are "
        "offline or communication degraded"};
  }

  if (!watchdog.configured ||
      !watchdog.active) {
    return {
        false,
        "hardware watchdog is not active"};
  }

  return {
      true,
      "required hardware and safety snapshot ready"};
}

}  // namespace

/* ============================================================
 * 构造和 watchdog 状态
 * ============================================================ */

GatewayCommandDispatcher::GatewayCommandDispatcher(
    DeviceRegistry& registry,
    std::string robot_id,
    SafetyConfig safety_config,
    G20LinkConfig g20_link,
    VideoConfig video_config)
    : registry_(registry),
      safety_(),
      robot_id_(std::move(robot_id)),
      safety_config_(std::move(safety_config)),
      g20_link_(std::move(g20_link)),
      video_config_(std::move(video_config)) {}

void GatewayCommandDispatcher::set_watchdog_status(
    WatchdogStatus status) {
  std::lock_guard<std::mutex> lock(mutex_);
  watchdog_status_ = std::move(status);
}

/* ============================================================
 * 启动控制链路
 * ============================================================ */

drivers::Result GatewayCommandDispatcher::start() {
  const auto& bringup =
      registry_.config().bringup;

  ControlApplicationOptions options;

  options.allow_partial_devices =
      bringup.allow_partial_devices ||
      bringup.ignore_device_startup_failures ||
      bringup.quarantine_failed_devices;

  options.arm_requires_all_required_devices =
      bringup.arm_requires_all_required_devices;

  auto control =
      std::make_unique<ControlApplication>(
          registry_.hardware_ports(),
          options);

  auto* control_ptr = control.get();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    control_ = std::move(control);
    state_ = ControlState::kSelfTest;
    state_detail_ = "starting hardware";

    control_started_ = false;
    ready_for_arm_ = false;
    partial_start_ = false;
  }

  const auto startup =
      control_ptr->start();

  if (!startup.started) {
    const std::string detail =
        "hardware start failed: " +
        startup.detail;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      control_started_ = false;
      ready_for_arm_ = false;
      partial_start_ = false;

      state_ = ControlState::kFaultLatched;
      state_detail_ = detail;

      safety_.enter_safe_hold(detail);
      ++command_epoch_;
    }

    registry_.safe_stop();

    return drivers::Result::failure(
        drivers::ErrorCode::kSafeHoldActive,
        detail,
        "start",
        "gateway_command_dispatcher");
  }

  const auto actuator_result =
      registry_.actuators().start();

  if (!actuator_result.ok &&
      !options.allow_partial_devices) {
    const std::string detail =
        "actuator start failed: " +
        actuator_result.detail;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      control_started_ = false;
      ready_for_arm_ = false;
      partial_start_ = false;

      state_ = ControlState::kFaultLatched;
      state_detail_ = detail;

      safety_.enter_safe_hold(detail);
      ++command_epoch_;
    }

    registry_.safe_stop();
    return actuator_result;
  }

  bool partial_start =
      startup.partial_start;

  std::string startup_detail =
      startup.detail;

  if (!actuator_result.ok) {
    partial_start = true;

    startup_detail +=
        "; actuator start degraded: ";
    startup_detail +=
        actuator_result.detail;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    control_started_ = true;
    ready_for_arm_ = startup.ready_for_arm;
    partial_start_ = partial_start;

    safety_.clear_for_disarmed();

    state_ = ControlState::kDisarmed;
    state_detail_ = startup_detail;

    ++command_epoch_;
  }

  return drivers::Result::success(
      "gateway command dispatcher ready");
}

/* ============================================================
 * 周期安全检查
 * ============================================================ */

drivers::Result GatewayCommandDispatcher::tick(
    std::chrono::steady_clock::time_point now) {
  const auto actuator_result =
      registry_.actuators().tick(now);

  const auto hardware =
      registry_.hardware_ports();

  const auto rs485_health =
      hardware.rs485.health();

  const auto can_health =
      hardware.can.health();

  bool sensor_health_ok = true;

  if (safety_config_.safe_stop_on_sensor_stale) {
    const auto samples =
        registry_.sensors().cached_samples();

    sensor_health_ok =
        sensor_samples_healthy(
            samples,
            safety_config_.required_samples);
  }

  bool need_safe_stop = false;
  std::string safe_stop_reason;

  drivers::Result tick_result =
      drivers::Result::success();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    const bool safety_latched =
        state_ == ControlState::kSafeHold ||
        state_ == ControlState::kFaultLatched ||
        state_ == ControlState::kEstopLatched;

    if (!actuator_result.ok) {
      if (partial_bringup_locked()) {
        partial_start_ = true;

        state_detail_ =
            "actuator tick degraded: " +
            actuator_result.detail;
      } else {
        tick_result = actuator_result;

        if (!safety_latched) {
          safe_stop_reason =
              "actuator tick failed: " +
              actuator_result.detail;

          enter_safe_hold_locked(
              safe_stop_reason);

          revoke_lease_locked(
              safe_stop_reason);

          need_safe_stop = true;
        }
      }
    }

    const auto lease =
        lease_.current();

    const bool g20_control_in_use =
        lease.held ||
        state_ == ControlState::kAcquired ||
        state_ == ControlState::kArmedIdle ||
        state_ == ControlState::kActive;

    if (!need_safe_stop &&
        safety_config_.safe_stop_on_g20_loss &&
        g20_control_in_use &&
        last_g20_seen_.time_since_epoch().count() != 0 &&
        now - last_g20_seen_ >
            safety_config_.g20_lease_timeout) {
      safe_stop_reason =
          "g20 link timeout";

      enter_safe_hold_locked(
          safe_stop_reason);

      revoke_lease_locked(
          safe_stop_reason);

      need_safe_stop = true;

      tick_result =
          drivers::Result::failure(
              drivers::ErrorCode::kIoError,
              safe_stop_reason,
              "tick",
              "gateway_command_dispatcher");
    }

    const bool armed_or_active =
        state_ == ControlState::kArmedIdle ||
        state_ == ControlState::kActive;

    if (!need_safe_stop &&
        armed_or_active &&
        safety_config_.safe_stop_on_rs485_bus_fault &&
        rs485_health ==
            drivers::DriverHealth::kFault) {
      safe_stop_reason =
          "rs485 bus fault";

      enter_safe_hold_locked(
          safe_stop_reason);

      revoke_lease_locked(
          safe_stop_reason);

      need_safe_stop = true;

      tick_result =
          drivers::Result::failure(
              drivers::ErrorCode::kIoError,
              safe_stop_reason,
              "tick",
              "gateway_command_dispatcher");
    }

    if (!need_safe_stop &&
        armed_or_active &&
        safety_config_.safe_stop_on_can_bus_fault &&
        can_health ==
            drivers::DriverHealth::kFault) {
      safe_stop_reason =
          "can bus fault";

      enter_safe_hold_locked(
          safe_stop_reason);

      revoke_lease_locked(
          safe_stop_reason);

      need_safe_stop = true;

      tick_result =
          drivers::Result::failure(
              drivers::ErrorCode::kIoError,
              safe_stop_reason,
              "tick",
              "gateway_command_dispatcher");
    }

    if (!need_safe_stop &&
        armed_or_active &&
        safety_config_.safe_stop_on_sensor_stale &&
        !sensor_health_ok) {
      safe_stop_reason =
          "sensor data stale, invalid or unknown";

      enter_safe_hold_locked(
          safe_stop_reason);

      revoke_lease_locked(
          safe_stop_reason);

      need_safe_stop = true;

      tick_result =
          drivers::Result::failure(
              drivers::ErrorCode::kSensorStale,
              safe_stop_reason,
              "tick",
              "gateway_command_dispatcher");
    }
  }

  if (need_safe_stop) {
    apply_safe_stop_unlocked(
        safe_stop_reason);
  }

  return tick_result;
}

/* ============================================================
 * IPC 请求入口
 * ============================================================ */

std::string GatewayCommandDispatcher::handle_request(
    const std::string& request_json) {
  try {
    const auto request =
        parse_request(request_json);

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!robot_id_.empty() &&
          !request.metadata.robot_id.empty() &&
          request.metadata.robot_id != robot_id_) {
        return make_ack_locked(
            request.metadata,
            AckStatus::kFailed,
            "robot_id mismatch");
      }

      mark_link_activity_locked(
          request.metadata,
          std::chrono::steady_clock::now());
    }

    if (request.action ==
        "gateway.status_snapshot") {
      return make_status();
    }

    if (request.action ==
        "gateway.telemetry_snapshot") {
      return make_telemetry();
    }

    if (request.action ==
        "gateway.capabilities") {
      return make_capabilities();
    }

    if (is_control_action(request.action)) {
      return handle_control(request);
    }

    return handle_actuator(request);
  } catch (const std::exception& error) {
    json response;

    response["schema_version"] = 1;
    response["kind"] = "ack";
    response["robot_id"] = robot_id_;
    response["action"] = "unknown";
    response["status"] =
        ack_status_to_string(
            AckStatus::kFailed);

    response["detail"] =
        std::string("bad request: ") +
        error.what();

    response["ack_time_ms"] =
        now_ms();

    return response.dump();
  }
}

/* ============================================================
 * 停止控制链路
 * ============================================================ */

void GatewayCommandDispatcher::stop() noexcept {
  ControlApplication* control = nullptr;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    control = control_.get();

    control_started_ = false;
    ready_for_arm_ = false;
    partial_start_ = false;

    revoke_lease_locked(
        "dispatcher stopped");

    state_ = ControlState::kDisarmed;
    state_detail_ = "stopped";

    ++command_epoch_;
  }

  registry_.safe_stop();

  if (control != nullptr) {
    control->stop();
  }

  registry_.stop();
}

/* ============================================================
 * 请求解析
 * ============================================================ */

GatewayCommandDispatcher::ParsedRequest
GatewayCommandDispatcher::parse_request(
    const std::string& request_json) const {
  const auto root =
      json::parse(request_json);

  ParsedRequest request;

  request.raw_json = request_json;
  request.kind =
      json_string(root, "kind", "command");
  request.action =
      json_string(root, "action");

  request.metadata.robot_id =
      json_string(root, "robot_id", robot_id_);

  request.metadata.link_id =
      link_id_from_string(
          json_string(
              root,
              "link_id",
              "g20_udp"));

  request.metadata.source_id =
      json_string(
          root,
          "source_id",
          "g20");

  request.metadata.boot_id =
      json_string(root, "boot_id");

  request.metadata.session_id =
      json_string(
          root,
          "session_id",
          "default");

  request.metadata.message_id =
      json_string(root, "message_id");

  request.metadata.sequence =
      json_u64(
          root,
          "seq",
          json_u64(
              root,
              "sequence",
              0));

  request.metadata.issued_at_ms =
      static_cast<std::uint64_t>(
          std::max<std::int64_t>(
              0,
              json_i64(
                  root,
                  "issued_at_ms",
                  now_ms())));

  const auto ttl_ms =
      std::clamp<std::int64_t>(
          json_i64(
              root,
              "ttl_ms",
              1000),
          0,
          static_cast<std::int64_t>(
              std::numeric_limits<
                  std::uint32_t>::max()));

  request.metadata.ttl_ms =
      static_cast<std::uint32_t>(
          ttl_ms);

  request.metadata.control_epoch =
      json_u64(
          root,
          "control_epoch",
          0);

  if (root.contains("lease_id") &&
      !root.at("lease_id").is_null()) {
    request.metadata.lease_id =
        json_string(root, "lease_id");
  }

  return request;
}

/* ============================================================
 * 控制权、arm、安全停止和清故障
 * ============================================================ */

std::string GatewayCommandDispatcher::handle_control(
    const ParsedRequest& request) {
  const auto& metadata =
      request.metadata;

  if (request.action ==
      "control.acquire") {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!control_started_) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "control application not started");
    }

    if (state_ ==
        ControlState::kEstopLatched) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "estop latched");
    }

    /*
     * 同一控制端重复 acquire 时保持当前状态，
     * 防止 ACTIVE 被意外改回 ACQUIRED。
     */
    if (lease_matches_locked(metadata)) {
      return make_ack_locked(
          metadata,
          AckStatus::kSucceeded,
          "lease already held");
    }

    LeaseRequest lease_request;

    lease_request.link_id =
        metadata.link_id;
    lease_request.source_id =
        metadata.source_id;
    lease_request.session_id =
        metadata.session_id;

    const auto acquired =
        lease_.acquire(
            lease_request);

    if (!acquired) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "lease is held by another owner");
    }

    lease_source_id_ =
        metadata.source_id;
    lease_session_id_ =
        metadata.session_id;
    lease_link_id_ =
        metadata.link_id;

    if (state_ !=
            ControlState::kSafeHold &&
        state_ !=
            ControlState::kFaultLatched) {
      safety_.clear_for_disarmed();
      state_ = ControlState::kAcquired;
      state_detail_ = "control acquired";
    }

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "lease acquired; arm required");
  }

  if (request.action ==
      "control.arm") {
    bool require_all = false;
    std::uint64_t arm_epoch = 0;
    WatchdogStatus watchdog;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!control_started_) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "control application not started");
      }

      if (!lease_matches_locked(metadata)) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "no valid control lease");
      }

      if (state_ ==
          ControlState::kEstopLatched) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "estop latched");
      }

      if (state_ ==
              ControlState::kSafeHold ||
          state_ ==
              ControlState::kFaultLatched) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "clear recoverable fault before arm");
      }

      if (state_ ==
              ControlState::kArmedIdle ||
          state_ ==
              ControlState::kActive) {
        return make_ack_locked(
            metadata,
            AckStatus::kSucceeded,
            "already armed");
      }

      if (state_ !=
          ControlState::kAcquired) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "control must be acquired before arm");
      }

      if (!ready_for_arm_) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "control application is not ready for arm");
      }

      require_all =
          registry_.config()
              .bringup
              .arm_requires_all_required_devices;

      arm_epoch = command_epoch_;
      watchdog = watchdog_status_;
    }

    if (require_all) {
      const auto gate =
          evaluate_required_gate(
              registry_,
              watchdog);

      if (!gate.ready) {
        std::lock_guard<std::mutex> lock(mutex_);

        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            gate.detail);
      }
    }

    const auto stopped =
        registry_.actuators().safe_stop_all();

    if (!stopped.ok &&
        require_all) {
      std::lock_guard<std::mutex> lock(mutex_);

      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "safe stop before arm failed: " +
              stopped.detail);
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (arm_epoch != command_epoch_ ||
        !lease_matches_locked(metadata) ||
        state_ !=
            ControlState::kAcquired) {
      return make_ack_locked(
          metadata,
          AckStatus::kAborted,
          "control state changed while arming");
    }

    state_ =
        ControlState::kArmedIdle;

    if (stopped.ok) {
      state_detail_ =
          "armed and idle";
    } else {
      partial_start_ = true;

      state_detail_ =
          "armed in partial debug mode: " +
          stopped.detail;
    }

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        state_detail_);
  }

  if (request.action ==
      "control.heartbeat") {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!lease_matches_locked(metadata)) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "no valid control lease");
    }

    mark_link_activity_locked(
        metadata,
        std::chrono::steady_clock::now());

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "heartbeat");
  }

  if (request.action ==
      "control.disarm") {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!lease_matches_locked(metadata)) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "no valid control lease");
      }

      state_ = ControlState::kDisarmed;
      state_detail_ = "disarmed";

      safety_.clear_for_disarmed();

      revoke_lease_locked(
          "remote disarm");

      ++command_epoch_;
    }

    const auto stopped =
        registry_.actuators().safe_stop_all();

    std::lock_guard<std::mutex> lock(mutex_);

    if (!stopped.ok) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "disarm safe stop failed: " +
              stopped.detail);
    }

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "disarmed");
  }

  if (request.action ==
      "control.release") {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!lease_matches_locked(metadata)) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "no valid control lease");
      }

      state_ = ControlState::kDisarmed;
      state_detail_ = "released";

      safety_.clear_for_disarmed();

      revoke_lease_locked(
          "remote release");

      ++command_epoch_;
    }

    const auto stopped =
        registry_.actuators().safe_stop_all();

    std::lock_guard<std::mutex> lock(mutex_);

    if (!stopped.ok) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "release safe stop failed: " +
              stopped.detail);
    }

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "released");
  }

  if (request.action ==
      "control.safe_hold") {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      enter_safe_hold_locked(
          "remote safe hold");

      revoke_lease_locked(
          "remote safe hold");
    }

    apply_safe_stop_unlocked(
        "remote safe hold");

    std::lock_guard<std::mutex> lock(mutex_);

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "safe hold");
  }

  if (request.action ==
      "control.estop") {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      enter_estop_locked(
          "remote estop");

      revoke_lease_locked(
          "remote estop");
    }

    apply_safe_stop_unlocked(
        "remote estop");

    std::lock_guard<std::mutex> lock(mutex_);

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "estop latched");
  }

  if (request.action ==
      "control.link_down") {
    bool need_safe_stop = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (safety_config_
              .safe_stop_on_g20_loss) {
        enter_safe_hold_locked(
            "gateway reported g20 link down");

        revoke_lease_locked(
            "g20 link down");

        need_safe_stop = true;
      }
    }

    if (need_safe_stop) {
      apply_safe_stop_unlocked(
          "gateway reported g20 link down");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "link down handled");
  }

  if (request.action ==
      "fault.clear") {
    std::uint64_t clear_epoch = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (state_ ==
          ControlState::kEstopLatched) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "estop latched");
      }

      if (!safety_config_
               .allow_remote_fault_clear) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "remote fault clear disabled");
      }

      if (!lease_matches_locked(metadata)) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "no valid control lease");
      }

      clear_epoch = command_epoch_;
    }

    const auto cleared =
        clear_recoverable_faults();

    std::lock_guard<std::mutex> lock(mutex_);

    if (!cleared.ok) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          cleared.detail);
    }

    if (clear_epoch != command_epoch_ ||
        state_ ==
            ControlState::kEstopLatched) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          "safety state changed while "
          "clearing faults");
    }

    safety_.clear_for_disarmed();

    state_ = ControlState::kDisarmed;
    state_detail_ = "fault cleared";

    revoke_lease_locked(
        "fault clear");

    ++command_epoch_;

    return make_ack_locked(
        metadata,
        AckStatus::kSucceeded,
        "faults cleared");
  }

  std::lock_guard<std::mutex> lock(mutex_);

  return make_ack_locked(
      metadata,
      AckStatus::kFailed,
      "unknown control action");
}

/* ============================================================
 * 执行器命令分发
 * ============================================================ */

std::string GatewayCommandDispatcher::handle_actuator(
    const ParsedRequest& request) {
  const auto& metadata =
      request.metadata;

  const bool stop_action =
      is_stop_action(request.action);

  bool require_all = false;
  std::uint64_t request_epoch = 0;
  WatchdogStatus watchdog;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::string denied_reason;

    if (stop_action) {
      if (!control_started_) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "control application not started");
      }

      if (!lease_matches_locked(metadata)) {
        return make_ack_locked(
            metadata,
            AckStatus::kFailed,
            "no valid control lease");
      }
    } else if (!actuator_allowed_locked(
                   metadata,
                   &denied_reason)) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          denied_reason);
    }

    require_all =
        registry_.config()
            .bringup
            .arm_requires_all_required_devices;

    request_epoch = command_epoch_;
    watchdog = watchdog_status_;
  }

  if (!stop_action &&
      require_all) {
    const auto gate =
        evaluate_required_gate(
            registry_,
            watchdog);

    if (!gate.ready) {
      std::lock_guard<std::mutex> lock(mutex_);

      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          gate.detail);
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (request_epoch != command_epoch_ ||
        !lease_matches_locked(metadata)) {
      return make_ack_locked(
          metadata,
          AckStatus::kAborted,
          "command superseded by safety state");
    }

    if (!stop_action) {
      std::string denied_reason;

      if (!actuator_allowed_locked(
              metadata,
              &denied_reason)) {
        return make_ack_locked(
            metadata,
            AckStatus::kAborted,
            denied_reason);
      }
    }
  }

  drivers::Result result =
      drivers::Result::failure(
          "unknown actuator action");

  try {
    result =
        execute_actuator(request);
  } catch (const std::exception& error) {
    result =
        drivers::Result::failure(
            std::string(
                "actuator command failed: ") +
            error.what());
  }

  std::string response;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!result.ok) {
      return make_ack_locked(
          metadata,
          AckStatus::kFailed,
          result.detail);
    }

    if (request_epoch != command_epoch_ ||
        !lease_matches_locked(metadata)) {
      response =
          make_ack_locked(
              metadata,
              AckStatus::kAborted,
              "command superseded by safety state");
    } else if (
        !stop_action &&
        state_ != ControlState::kArmedIdle &&
        state_ != ControlState::kActive) {
      response =
          make_ack_locked(
              metadata,
              AckStatus::kAborted,
              "control was disarmed during command");
    } else {
      if (stop_action) {
        /*
         * 停止命令不能隐式 arm。
         */
        if (state_ ==
                ControlState::kArmedIdle ||
            state_ ==
                ControlState::kActive) {
          state_ =
              ControlState::kArmedIdle;
        }
      } else {
        state_ =
            ControlState::kActive;
      }

      state_detail_ = request.action;

      response =
          make_ack_locked(
              metadata,
              AckStatus::kSucceeded,
              result.detail.empty()
                  ? "ok"
                  : result.detail);
    }
  }

  /*
   * ============================================================
   * 关键修复：命令被安全状态超越时，不再级联全机硬件安全停止
   * ============================================================
   *
   * 旧实现在此调用
   *   apply_safe_stop_unlocked("actuator command superseded ...")
   * 而该函数会执行
   *   registry_.safe_stop();
   *   control->enter_safe_hold(reason);
   * 即把风机、推杆、毛刷、爬绳器全部一起停掉，并把
   * ControlState 切到 SAFE_HOLD，同时撤销租约。
   *
   * 但“命令被超越”完全不代表硬件有故障，它只有三种来源：
   *
   *   1) request_epoch != command_epoch_
   *      command_epoch_ 只在 start() / acquire / arm / disarm /
   *      release / estop / fault.clear / safe_hold 以及 tick()
   *      判定出真实故障时递增。也就是说，只要在一条执行器命令的
   *      锁外执行期间（CAN/RS485 写入窗口内），恰好有一次
   *      acquire / arm / start 落地，这条本来已经成功下发的命令
   *      就会被判为 superseded，于是整机被无故障地安全停止。
   *
   *   2) !lease_matches_locked(metadata)
   *      只说明控制权所有者（link_id / source_id / session_id）
   *      与下发时不一致，即控制端换了，不是硬件故障。
   *
   *   3) state_ 不再是 ARMED_IDLE / ACTIVE
   *      对应日志 “control was disarmed during command”，
   *      本身就是因为安全状态已经被别的路径改掉。
   *
   * 而这条命令在锁外已经真正下发给执行器了。对于连续型动作
   * （fan.enable / fan.left_set / fan.right_set / fan.both_set /
   * brush.forward / brush.reverse / climber.ascend /
   * climber.descend），此刻把整机停掉就是现场观察到的现象：
   *   全部模块自己停止、风机等模块无故停机、且必须重新点“控制权”
   *   才能恢复（因为 SAFE_HOLD 会撤销租约，而屏幕侧的
   *   screen_recovery_required_ 又要求一次非 mapper 来源的
   *   control.acquire）。
   *
   * 这是“所有模块运行几分钟后自己停止”的第一位成因：
   * 爬绳器是连续型设备，一旦它进入运行，G20 侧就会持续产生
   * 成对的连续命令（含周期性心跳与按键持续包），叠加
   * control.acquire / control.arm / start() 的落地窗口，使某条
   * 命令被超越的概率显著上升；而爬绳器本身在 superseded 时
   * 同一条命令往往已经写进伺服，于是出现
   *   “爬绳器不停止，另一个模块停止”、“不用操作又能立刻恢复”
   * 这种特征现象。
   *
   * 安全停止的职责边界应保持单一：
   *   - tick() 负责真实故障：执行器 tick 失败、CAN / RS485 总线
   *     fault、传感器快照 stale/invalid、G20 链路超时；
   *   - handle_control() 负责显式控制面命令：control.safe_hold、
   *     control.estop、control.disarm、control.release、
   *     control.link_down、fault.clear。
   *
   * 因此这里只保留 ACK 的 ABORTED 语义（response 已在上面按分支
   * 构造完成），不再级联硬件安全停止。安全能力没有任何削弱：
   * 上述两条真实安全路径都保持原样。
   */

  return response;
}

/* ============================================================
 * 实际执行器调用
 *
 * 调用本函数时不得持有 mutex_。
 * ============================================================ */

drivers::Result
GatewayCommandDispatcher::execute_actuator(
    const ParsedRequest& request) {
  const auto root =
      json::parse(request.raw_json);

  const auto params =
      root.value(
          "params",
          json::object());

  if (request.action == "brush.forward") {
    return registry_.actuators()
        .brush_forward(
            param_u16(
                params,
                "speed_raw",
                kBrushMaxSpeedRaw));
  }

  if (request.action == "brush.reverse") {
    return registry_.actuators()
        .brush_reverse(
            param_u16(
                params,
                "speed_raw",
                kBrushMaxSpeedRaw));
  }

  if (request.action == "brush.stop") {
    return registry_.actuators().brush_stop();
  }

  if (request.action == "brush_push.extend") {
    return registry_.actuators()
        .brush_push_extend();
  }

  if (request.action == "brush_push.retract") {
    return registry_.actuators()
        .brush_push_retract();
  }

  if (request.action ==
      "brush_push.extend_timed") {
    const auto duration =
        std::clamp<std::int64_t>(
            param_i64(
                params,
                "duration_ms",
                1000),
            100,
            kBrushPushMaxDurationMs);

    return registry_.actuators()
        .brush_push_extend_for(
            std::chrono::milliseconds(
                duration));
  }

  if (request.action ==
      "brush_push.retract_timed") {
    const auto duration =
        std::clamp<std::int64_t>(
            param_i64(
                params,
                "duration_ms",
                1000),
            100,
            kBrushPushMaxDurationMs);

    return registry_.actuators()
        .brush_push_retract_for(
            std::chrono::milliseconds(
                duration));
  }

  if (request.action == "brush_push.stop") {
    return registry_.actuators()
        .brush_push_stop();
  }

  if (request.action == "scraper_push.extend") {
    return registry_.actuators()
        .scraper_push_extend();
  }

  if (request.action == "scraper_push.retract") {
    return registry_.actuators()
        .scraper_push_retract();
  }

  if (request.action ==
      "scraper_push.extend_timed") {
    const auto duration =
        std::clamp<std::int64_t>(
            param_i64(
                params,
                "duration_ms",
                1000),
            100,
            kScraperPushMaxDurationMs);

    return registry_.actuators()
        .scraper_push_extend_for(
            std::chrono::milliseconds(
                duration));
  }

  if (request.action ==
      "scraper_push.retract_timed") {
    const auto duration =
        std::clamp<std::int64_t>(
            param_i64(
                params,
                "duration_ms",
                1000),
            100,
            kScraperPushMaxDurationMs);

    return registry_.actuators()
        .scraper_push_retract_for(
            std::chrono::milliseconds(
                duration));
  }

  if (request.action == "scraper_push.stop") {
    return registry_.actuators()
        .scraper_push_stop();
  }

  /*
   * 兼容旧界面的 push.* 命令。
   */
  if (request.action == "push.extend") {
    return registry_.actuators()
        .brush_push_extend();
  }

  if (request.action == "push.retract") {
    return registry_.actuators()
        .brush_push_retract();
  }

  if (request.action == "push.extend_timed") {
    const auto duration =
        std::clamp<std::int64_t>(
            param_i64(
                params,
                "duration_ms",
                3000),
            100,
            kBrushPushMaxDurationMs);

    return registry_.actuators()
        .brush_push_extend_for(
            std::chrono::milliseconds(
                duration));
  }

  if (request.action == "push.retract_timed") {
    const auto duration =
        std::clamp<std::int64_t>(
            param_i64(
                params,
                "duration_ms",
                3000),
            100,
            kBrushPushMaxDurationMs);

    return registry_.actuators()
        .brush_push_retract_for(
            std::chrono::milliseconds(
                duration));
  }

  if (request.action == "push.stop") {
    return registry_.actuators().push_stop();
  }

  if (request.action == "fan.prepare") {
    return registry_.actuators().fan_prepare();
  }

  if (request.action == "fan.enable") {
    return registry_.actuators().fan_enable();
  }

  if (request.action == "fan.disable") {
    return registry_.actuators().fan_disable();
  }

  if (request.action == "fan.left_set") {
    return registry_.actuators()
        .fan_left_set(
            std::min<std::uint16_t>(
                param_u16(
                    params,
                    "speed_raw",
                    0),
                kFanMaxRaw));
  }

  if (request.action == "fan.right_set") {
    return registry_.actuators()
        .fan_right_set(
            std::min<std::uint16_t>(
                param_u16(
                    params,
                    "speed_raw",
                    0),
                kFanMaxRaw));
  }

  if (request.action == "fan.both_set") {
    const auto left =
        std::min<std::uint16_t>(
            param_u16(
                params,
                "left_raw",
                0),
            kFanMaxRaw);

    const auto right =
        std::min<std::uint16_t>(
            param_u16(
                params,
                "right_raw",
                0),
            kFanMaxRaw);

    return registry_.actuators()
        .fan_both_set(
            left,
            right);
  }

  if (request.action == "fan.stop") {
    return registry_.actuators()
        .fan_stop_all();
  }

  if (request.action == "climber.ascend") {
    return registry_.actuators()
        .climber_ascend(
            climber_speed_rpm(params));
  }

  if (request.action == "climber.descend") {
    return registry_.actuators()
        .climber_descend(
            climber_speed_rpm(params));
  }

  if (request.action == "climber.stop") {
    return registry_.actuators()
        .climber_stop();
  }

  return drivers::Result::failure(
      "unknown actuator action");
}

/* ============================================================
 * 控制权和动作准入
 * ============================================================ */

bool GatewayCommandDispatcher::lease_matches_locked(
    const CommandMetadata& metadata) const {
  const auto lease =
      lease_.current();

  if (!lease.held) {
    return false;
  }

  return lease_link_id_ ==
             metadata.link_id &&
         lease_source_id_ ==
             metadata.source_id &&
         lease_session_id_ ==
             metadata.session_id;
}

bool GatewayCommandDispatcher::actuator_allowed_locked(
    const CommandMetadata& metadata,
    std::string* reason) const {
  if (state_ ==
      ControlState::kEstopLatched) {
    *reason = "estop latched";
    return false;
  }

  if (state_ ==
          ControlState::kSafeHold ||
      state_ ==
          ControlState::kFaultLatched) {
    *reason = "control not armed";
    return false;
  }

  if (!control_started_) {
    *reason =
        "control application not started";
    return false;
  }

  if (!lease_matches_locked(metadata)) {
    *reason =
        "no valid control lease";
    return false;
  }

  if (state_ !=
          ControlState::kArmedIdle &&
      state_ !=
          ControlState::kActive) {
    *reason =
        "control must be armed before "
        "actuator command";
    return false;
  }

  return true;
}

bool GatewayCommandDispatcher::partial_bringup_locked()
    const {
  const auto& bringup =
      registry_.config().bringup;

  return bringup.allow_partial_devices ||
         bringup.ignore_device_startup_failures ||
         bringup.quarantine_failed_devices;
}

void GatewayCommandDispatcher::mark_link_activity_locked(
    const CommandMetadata& metadata,
    std::chrono::steady_clock::time_point now) {
  if (metadata.link_id ==
      LinkId::kG20Udp) {
    last_g20_seen_ = now;
  }
}

/* ============================================================
 * 安全状态锁存
 * ============================================================ */

void GatewayCommandDispatcher::enter_safe_hold_locked(
    const std::string& reason) {
  state_ = ControlState::kSafeHold;
  state_detail_ = reason;

  safety_.enter_safe_hold(reason);
  ++command_epoch_;
}

void GatewayCommandDispatcher::enter_estop_locked(
    const std::string& reason) {
  state_ = ControlState::kEstopLatched;
  state_detail_ = reason;

  safety_.enter_safe_hold(reason);
  ++command_epoch_;
}

void GatewayCommandDispatcher::revoke_lease_locked(
    const std::string& reason) {
  lease_.revoke(reason);

  lease_source_id_.clear();
  lease_session_id_.clear();
}

/* ============================================================
 * 锁外硬件安全停止
 * ============================================================ */

void GatewayCommandDispatcher::apply_safe_stop_unlocked(
    const std::string& reason) noexcept {
  ControlApplication* control = nullptr;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    control = control_.get();
  }

  registry_.safe_stop();

  if (control != nullptr) {
    control->enter_safe_hold(reason);
  }
}

/* ============================================================
 * ACK
 * ============================================================ */

std::string GatewayCommandDispatcher::make_ack_locked(
    const CommandMetadata& metadata,
    AckStatus status,
    const std::string& detail) const {
  json response;

  response["schema_version"] = 1;
  response["kind"] = "ack";
  response["robot_id"] = robot_id_;
  response["link_id"] =
      link_id_to_string(metadata.link_id);
  response["source_id"] =
      metadata.source_id;
  response["session_id"] =
      metadata.session_id;
  response["message_id"] =
      metadata.message_id;
  response["seq"] =
      metadata.sequence;
  response["status"] =
      ack_status_to_string(status);
  response["detail"] =
      detail;
  response["state"] =
      control_state_to_string(state_);
  response["ack_time_ms"] =
      now_ms();

  const auto lease =
      lease_.current();

  if (lease.held) {
    response["lease_id"] =
        lease.lease_id;
    response["control_epoch"] =
        lease.control_epoch;
  }

  return response.dump();
}

/* ============================================================
 * 状态缓存快照
 * ============================================================ */

std::string GatewayCommandDispatcher::make_status() {
  const auto actuator =
      registry_.actuators().snapshot();

  const auto samples =
      registry_.sensors().cached_samples();

  const auto devices =
      registry_.sensors().device_statuses();

  std::lock_guard<std::mutex> lock(mutex_);

  const auto lease =
      lease_.current();

  json response;

  response["schema_version"] = 1;
  response["kind"] = "status";
  response["robot_id"] = robot_id_;
  response["state"] =
      control_state_to_string(state_);

  response["armed"] =
      state_ == ControlState::kArmedIdle ||
      state_ == ControlState::kActive;

  response["control_started"] =
      control_started_;
  response["partial_start"] =
      partial_start_;
  response["hardware_ready_for_arm"] =
      ready_for_arm_;

  response["ready_for_arm"] =
      ready_for_arm_ &&
      state_ == ControlState::kAcquired;

  response["lease_held"] =
      lease.held;
  response["detail"] =
      state_detail_;

  response["control_epoch"] =
      lease.held
          ? lease.control_epoch
          : 0;

  response["lease_id"] =
      lease.held
          ? lease.lease_id
          : "";

  response["lease_owner"] =
      lease.held
          ? lease_source_id_
          : "";

  const auto now =
      std::chrono::steady_clock::now();

  response["g20_online"] =
      last_g20_seen_.time_since_epoch().count() != 0 &&
      now - last_g20_seen_ <=
          safety_config_.g20_lease_timeout;

  response["safety_reason"] =
      safety_.reason();

  response["updated_at_ms"] =
      now_ms();
  response["timestamp_ms"] =
      response["updated_at_ms"];

  response["g20_udp"]["listen_address"] =
      g20_link_.listen_address;
  response["g20_udp"]["command_port"] =
      g20_link_.command_listen_port;
  response["g20_udp"]["status_port"] =
      g20_link_.status_publish_port;
  response["g20_udp"]["telemetry_port"] =
      g20_link_.telemetry_publish_port;
  response["g20_udp"]["heartbeat_ms"] =
      g20_link_.heartbeat.count();
  response["g20_udp"]["lease_timeout_ms"] =
      g20_link_.lease_timeout.count();

  response["video"]["path"] =
      video_config_.path;
  response["video"]["target_address"] =
      video_config_.target_address;
  response["video"]["camera1_device"] =
      video_config_.camera1_device;
  response["video"]["camera2_device"] =
      video_config_.camera2_device;
  response["video"]["camera1_port"] =
      video_config_.camera1_udp_port;
  response["video"]["camera2_port"] =
      video_config_.camera2_udp_port;
  response["video"]["camera1_url"] =
      video_config_.camera1_play_url;
  response["video"]["camera2_url"] =
      video_config_.camera2_play_url;

  response["watchdog"]["enabled"] =
      watchdog_status_.configured;
  response["watchdog"]["active"] =
      watchdog_status_.active;
  response["watchdog"]["device_path"] =
      watchdog_status_.device_path;
  response["watchdog"]["timeout_sec"] =
      watchdog_status_.timeout_sec;
  response["watchdog"]["last_feed_ms"] =
      watchdog_status_.last_feed_ms;
  response["watchdog"]["detail"] =
      watchdog_status_.detail;

  response["actuators"]["brush"]["state"] =
      actuator.brush_state;
  response["actuators"]["brush"]["speed_raw"] =
      actuator.brush_speed_raw;

  response["actuators"]["fan"]["active"] =
      actuator.fan_active;
  response["actuators"]["fan"]["left_raw"] =
      actuator.fan_left_raw;
  response["actuators"]["fan"]["right_raw"] =
      actuator.fan_right_raw;
  response["actuators"]["fan"]["synchronized"] =
      actuator.fan_left_raw ==
      actuator.fan_right_raw;

  response["actuators"]["brush_push"]["state"] =
      actuator.brush_push_state;
  response["actuators"]["brush_push"]["remaining_ms"] =
      actuator.brush_push_remaining_ms;
  response["actuators"]["brush_push"]["last_duration_ms"] =
      actuator.brush_push_last_duration_ms;
  response["actuators"]["brush_push"]["extension_mm"] =
      actuator.brush_push_extension_mm;

  response["actuators"]["scraper_push"]["state"] =
      actuator.scraper_push_state;
  response["actuators"]["scraper_push"]["remaining_ms"] =
      actuator.scraper_push_remaining_ms;
  response["actuators"]["scraper_push"]["last_duration_ms"] =
      actuator.scraper_push_last_duration_ms;
  response["actuators"]["scraper_push"]["extension_mm"] =
      actuator.scraper_push_extension_mm;

  response["actuators"]["push"]["state"] =
      actuator.brush_push_state;
  response["actuators"]["push"]["remaining_ms"] =
      actuator.brush_push_remaining_ms;
  response["actuators"]["push"]["last_duration_ms"] =
      actuator.brush_push_last_duration_ms;
  response["actuators"]["push"]["extension_mm"] =
      actuator.brush_push_extension_mm;

  response["actuators"]["climber"]["state"] =
      actuator.climber_state;
  response["actuators"]["climber"]["speed_rpm"] =
      actuator.climber_speed_rpm;

  response["sensors"]["cache_count"] =
      samples.size();
  response["sensors"]["samples"] =
      json::array();

  for (const auto& sample : samples) {
    json item;

    item["name"] =
        sample.logical_name;
    item["unit"] =
        sample.unit;
    item["timestamp_ms"] =
        sample.timestamp_ms;
    item["age_ms"] =
        sample.age_ms;
    item["quality"] =
        sample_quality_to_string(
            sample.quality);

    item["value"] =
        sample.value
            ? json(*sample.value)
            : json(nullptr);

    response["sensors"]["samples"]
        .push_back(std::move(item));
  }

  response["sensors"]["devices"] =
      json::array();

  for (const auto& device : devices) {
    json item;

    item["name"] =
        device.logical_name;
    item["online"] =
        device.online;
    item["healthy"] =
        device.online;
    item["health"] =
        device.online
            ? "READY"
            : "FAULT";
    item["communication_health"] =
       sensor_communication_health_to_string(
           device.communication_health);
    item["consecutive_failures"] =
        device.consecutive_failures;
    item["failure_count"] =
        device.consecutive_failures;
    item["last_poll_ms"] =
        device.last_poll_ms;
    item["last_success_ms"] =
        device.last_success_ms;
    item["last_failure_ms"] =
        device.last_failure_ms;
    item["last_error_ms"] =
        device.last_failure_ms;
    item["last_error"] =
        device.last_error;

    response["sensors"]["devices"]
        .push_back(std::move(item));
  }

  return response.dump();
}

/* ============================================================
 * 遥测缓存快照
 * ============================================================ */

std::string GatewayCommandDispatcher::make_telemetry() {
  const auto samples =
      registry_.sensors().cached_samples();

  json response;

  response["schema_version"] = 1;
  response["kind"] = "telemetry";
  response["robot_id"] = robot_id_;
  response["updated_at_ms"] = now_ms();
  response["timestamp_ms"] =
      response["updated_at_ms"];
  response["samples"] =
      json::array();

  for (const auto& sample : samples) {
    json item;

    item["name"] =
        sample.logical_name;
    item["value"] =
        sample.value
            ? json(*sample.value)
            : json(nullptr);
    item["unit"] =
        sample.unit;
    item["timestamp_ms"] =
        sample.timestamp_ms;
    item["age_ms"] =
        sample.age_ms;
    item["quality"] =
        sample_quality_to_string(
            sample.quality);

    response["samples"].push_back(
        std::move(item));
  }

  return response.dump();
}

/* ============================================================
 * 能力说明
 * ============================================================ */

std::string
GatewayCommandDispatcher::make_capabilities() const {
  json response;

  response["schema_version"] = 1;
  response["kind"] = "capabilities";
  response["robot_id"] = robot_id_;
  response["updated_at_ms"] = now_ms();

  response["control_actions"] =
      json::array({
          "control.acquire",
          "control.arm",
          "control.disarm",
          "control.release",
          "control.heartbeat",
          "control.safe_hold",
          "control.estop",
          "control.link_down",
          "fault.clear",
      });

  response["actuators"]["brush"]["actions"] =
      json::array({
          "brush.forward",
          "brush.reverse",
          "brush.stop",
      });
  response["actuators"]["brush"]["max_speed_raw"] =
      kBrushMaxSpeedRaw;

  response["actuators"]["fan"]["actions"] =
      json::array({
          "fan.prepare",
          "fan.enable",
          "fan.disable",
          "fan.left_set",
          "fan.right_set",
          "fan.both_set",
          "fan.stop",
      });
  response["actuators"]["fan"]["synchronized"] =
      true;
  response["actuators"]["fan"]["base_raw"] =
      kFanBaseRaw;
  response["actuators"]["fan"]["step_raw"] =
      kFanStepRaw;
  response["actuators"]["fan"]["max_raw"] =
      kFanMaxRaw;

  response["actuators"]["brush_push"]["actions"] =
      json::array({
          "brush_push.extend",
          "brush_push.retract",
          "brush_push.extend_timed",
          "brush_push.retract_timed",
          "brush_push.stop",
      });
  response["actuators"]["brush_push"]["max_duration_ms"] =
      kBrushPushMaxDurationMs;

  response["actuators"]["scraper_push"]["actions"] =
      json::array({
          "scraper_push.extend",
          "scraper_push.retract",
          "scraper_push.extend_timed",
          "scraper_push.retract_timed",
          "scraper_push.stop",
      });
  response["actuators"]["scraper_push"]["max_duration_ms"] =
      kScraperPushMaxDurationMs;

  response["actuators"]["climber"]["actions"] =
      json::array({
          "climber.ascend",
          "climber.descend",
          "climber.stop",
      });
  response["actuators"]["climber"]["speed_unit"] =
      "rpm";
  response["actuators"]["climber"]["initial_speed_rpm"] =
      500;
  response["actuators"]["climber"]["step_rpm"] =
      100;
  response["actuators"]["climber"]["max_speed_rpm"] =
      3000;

  response["links"]["g20_udp"]["listen_address"] =
      g20_link_.listen_address;
  response["links"]["g20_udp"]["command_port"] =
      g20_link_.command_listen_port;
  response["links"]["g20_udp"]["status_port"] =
      g20_link_.status_publish_port;
  response["links"]["g20_udp"]["telemetry_port"] =
      g20_link_.telemetry_publish_port;
  response["links"]["g20_udp"]["heartbeat_ms"] =
      g20_link_.heartbeat.count();
  response["links"]["g20_udp"]["lease_timeout_ms"] =
      g20_link_.lease_timeout.count();

  response["video"]["path"] =
      video_config_.path;
  response["video"]["target_address"] =
      video_config_.target_address;
  response["video"]["camera1_device"] =
      video_config_.camera1_device;
  response["video"]["camera2_device"] =
      video_config_.camera2_device;
  response["video"]["camera1_port"] =
      video_config_.camera1_udp_port;
  response["video"]["camera2_port"] =
      video_config_.camera2_udp_port;
  response["video"]["camera1_url"] =
      video_config_.camera1_play_url;
  response["video"]["camera2_url"] =
      video_config_.camera2_play_url;
  response["video"]["width"] =
      video_config_.width;
  response["video"]["height"] =
      video_config_.height;
  response["video"]["fps"] =
      video_config_.fps;
  response["video"]["bps"] =
      video_config_.bps;
  response["video"]["gop"] =
      video_config_.gop;
  response["video"]["qp_init"] =
      video_config_.qp_init;

  return response.dump();
}

/* ============================================================
 * 清除可恢复故障
 * ============================================================ */

drivers::Result
GatewayCommandDispatcher::clear_recoverable_faults() {
  registry_.safe_stop();

  return registry_.actuators()
      .clear_faults();
}

}  // namespace snappy::control