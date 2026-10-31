#include "snappy/drivers/devices/climber.hpp"
/**
 * @file climber.cpp
 * @brief 爬绳器 L7P 设备驱动实现
 *
 * @details
 * - 负责启动、停机、抱闸控制和运行状态读取。
 * - 停机时先撤目标转速再落抱闸，避免带载制动。
 * - 安全停止有独立路径，不依赖正常停机流程。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy::drivers::devices {
namespace {

constexpr const char* kComponent = "climber_l7p";

constexpr double kPi = 3.14159265358979323846;

constexpr int kL7pDiCount = 8;

/*
 * 正常 STOP 的实际速度阈值。
 *
 * read_s32() 的单位是 0.1 RPM，
 * 因此 1 RPM == 10。
 */
constexpr std::int32_t
    kStopSpeedRpm = 1;

/*
 * P0B.18 bit2：
 *
 * 1 = BRK-OFF
 * 0 = BRK-ON / 抱闸关闭
 */
constexpr std::uint16_t kBrakeOffPhysicalMask = 0x0004U;

/*
 * P0B.05
 */
constexpr std::uint16_t kRunStateReadyMask = 0x0001U;

constexpr std::uint16_t kRunStateRunningMask = 0x0002U;

constexpr std::uint16_t kRunStateAlarmMask = 0x0004U;

Result climber_error(
    ErrorCode code,
    std::string detail,
    const char* operation,
    const std::string& device_id,
    bool retryable = false) {
  return Result::failure(
      code,
      std::move(detail),
      operation,
      device_id.empty()
          ? kComponent
          : device_id,
      retryable);
}

std::int32_t rad_per_s_to_rpm(double speed_rad_per_s) {
  return static_cast<std::int32_t>(
      std::lround(
          speed_rad_per_s * 60.0 / (2.0 * kPi)));
}

double rpm_raw_0p1_to_rad_per_s(std::int32_t raw) {
  const double rpm = static_cast<double>(raw) / 10.0;

  return rpm * (2.0 * kPi) / 60.0;
}

std::uint16_t virtual_io_bit_for_di(int index) {
  if (index < 0 || index >= 16) {
    return 0;
  }

  return static_cast<std::uint16_t>(1U << index);
}

std::uint16_t virtual_io_mask_for_di(
    int index,
    bool normally_closed,
    bool enabled) {
  const auto bit = virtual_io_bit_for_di(index);

  if (bit == 0) {
    return 0;
  }

  /*
   * 常闭 DI 的逻辑电平与 SRV-ON
   * 逻辑相反。
   */
  const bool bit_on =
      normally_closed
          ? !enabled
          : enabled;

  return bit_on ? bit : 0;
}

std::string hex_u16(std::uint16_t value) {
  std::ostringstream out;

  out << "0x"
      << std::uppercase
      << std::hex
      << std::setw(4)
      << std::setfill('0')
      << value;

  return out.str();
}

std::uint16_t di_function_register(
    const ClimberConfig& config,int index) {
  return static_cast<std::uint16_t>(
      config.profile
          .di1_function_register + index * 2);
}

}  // namespace

/* ============================================================
 * 配置
 * ============================================================ */

ClimberConfig make_climber_config() {
  ClimberConfig config;

  config.device_id = kClimberDeviceId;

  config.station_id = kClimberStationId;

  config.profile = protocols::ClimberModbusProfile{};

  /*
   * 默认只检查现场参数，不主动修改。
   */
  config.configure_on_start = false;

  config.auto_clear_alarm_on_start = true;

  config.brake_release_margin = std::chrono::milliseconds(10);

  config.stop_poll_period = std::chrono::milliseconds(20);

  config.stop_timeout = std::chrono::milliseconds(1000);

  config.alarm_reset_settle_time = std::chrono::milliseconds(50);

  return config;
}

/* ============================================================
 * 构造 / 状态
 * ============================================================ */

Climber::Climber(
    IRs485Bus& bus,
    ClimberConfig config)
    : bus_(bus),
      config_(std::move(config)) {}

DriverHealth Climber::health()
    const noexcept {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  return health_;
}

/* ============================================================
 * Modbus 基础读
 * ============================================================ */

protocols::ModbusRegisterRead
Climber::read_registers(
    std::uint16_t start_register,
    std::uint16_t register_count) {
  if (config_.station_id == 0 ||
      register_count == 0) {
    return {
        climber_error(
            ErrorCode::kInvalidArgument,
            "invalid L7P read request",
            "read_registers",
            config_.device_id),
        {}};
  }

  Rs485Request request;

  request.logical_device = config_.device_id;

  request.payload =
      protocols::encode_climber_read_registers(
          config_.station_id,
          start_register,
          register_count);

  /*
   * 明确告诉 RS485 scheduler：
   * 这是无副作用读取。
   *
   * 允许 scheduler 按现有策略重试。
   */
  request.side_effect_free_read = true;

  SNAPPY_DRIVER_LOG_DEBUG(
      kComponent,
      "L7P READ intent device=" +
          config_.device_id +
          " station=" +
          std::to_string(config_.station_id) +
          " start_register=" +
          hex_u16(start_register) +
          " count=" +
          std::to_string(register_count));

  const auto response = bus_.exchange(request);

  if (!response.result.ok) {
    SNAPPY_DRIVER_LOG_WARN(
        kComponent,
        "L7P READ failed device=" +
            config_.device_id +
            " station=" +
            std::to_string(config_.station_id) +
            " start_register=" +
            hex_u16(start_register) +
            " count=" +
            std::to_string(register_count) +
            " code=" +
            std::to_string(static_cast<int>(response.result.code)) +
            " detail=" +
            response.result.detail);
    return {
        response.result,
        {}};
  }

  const auto decoded =
      protocols::decode_climber_read_registers_response(
          config_.station_id,
          register_count,
          response.payload);

  if (!decoded.result.ok) {
    SNAPPY_DRIVER_LOG_WARN(
        kComponent,
        "L7P READ decode_failed device=" +
            config_.device_id +
            " station=" +
            std::to_string(config_.station_id) +
            " start_register=" +
            hex_u16(start_register) +
            " count=" +
            std::to_string(register_count) +
            " code=" +
            std::to_string(static_cast<int>(decoded.result.code)) +
            " detail=" +
            decoded.result.detail);
    return decoded;
  }

  std::ostringstream values;
  for (std::size_t i = 0; i < decoded.values.size(); ++i) {
    if (i != 0) {
      values << ",";
    }
    values << hex_u16(decoded.values[i]);
  }

  SNAPPY_DRIVER_LOG_DEBUG(
      kComponent,
      "L7P READ success device=" +
          config_.device_id +
          " station=" +
          std::to_string(config_.station_id) +
          " start_register=" +
          hex_u16(start_register) +
          " count=" +
          std::to_string(register_count) +
          " values=[" +
          values.str() +
          "]");

  return decoded;
}

ClimberReadI32
Climber::read_register(
    std::uint16_t address) {
  const auto read =
      read_registers(
          address,
          1);

  if (!read.result.ok ||
      read.values.empty()) {
    return {
        read.result,
        std::nullopt};
  }

  return {
      Result::success(),
      static_cast<std::int32_t>(
          read.values.front())};
}

ClimberReadI32
Climber::read_s32(
    std::uint16_t address) {
  const auto read =
      read_registers(
          address,
          2);

  if (!read.result.ok ||
      read.values.size() < 2) {
    return {
        read.result,
        std::nullopt};
  }

  return {
      Result::success(),
      protocols::
          join_climber_i32_high_first(
              read.values[0],
              read.values[1])};
}

/* ============================================================
 * Modbus 写
 * ============================================================ */

Result Climber::write_register(
    std::uint16_t address,
    std::uint16_t value,
    const char* operation) {
  if (config_.station_id == 0) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P station_id is not configured",
        operation,
        config_.device_id);
  }

  Rs485Request request;

  request.logical_device = config_.device_id;

  request.payload =
      protocols::encode_climber_write_register(
          config_.station_id,
          address,
          value);

  /*
   * FC06 写请求不设置
   * side_effect_free_read。
   *
   * 现有 scheduler 不会自动重放写操作。
   */
  SNAPPY_DRIVER_LOG_DEBUG(
      kComponent,
      "L7P WRITE intent device=" +
          config_.device_id +
          " station=" +
          std::to_string(config_.station_id) +
          " operation=" +
          std::string(operation ? operation : "") +
          " register=" +
          hex_u16(address) +
          " value=" +
          hex_u16(value));

  const auto response = bus_.exchange(request);

  if (!response.result.ok) {
    SNAPPY_DRIVER_LOG_WARN(
        kComponent,
        "L7P WRITE failed device=" +
            config_.device_id +
            " station=" +
            std::to_string(config_.station_id) +
            " operation=" +
            std::string(operation ? operation : "") +
            " register=" +
            hex_u16(address) +
            " value=" +
            hex_u16(value) +
            " code=" +
            std::to_string(static_cast<int>(response.result.code)) +
            " detail=" +
            response.result.detail);
    return response.result;
  }

  const auto decoded = protocols::decode_climber_write_response(
          config_.station_id,
          address,
          value,
          response.payload);

  if (!decoded.ok) {
    SNAPPY_DRIVER_LOG_WARN(
        kComponent,
        "L7P WRITE decode_failed device=" +
            config_.device_id +
            " station=" +
            std::to_string(config_.station_id) +
            " operation=" +
            std::string(operation ? operation : "") +
            " register=" +
            hex_u16(address) +
            " value=" +
            hex_u16(value) +
            " code=" +
            std::to_string(static_cast<int>(decoded.code)) +
            " detail=" +
            decoded.detail);
    return decoded;
  }

  SNAPPY_DRIVER_LOG_DEBUG(
      kComponent,
      "L7P WRITE success device=" +
          config_.device_id +
          " station=" +
          std::to_string(config_.station_id) +
          " operation=" +
          std::string(operation ? operation : "") +
          " register=" +
          hex_u16(address) +
          " value=" +
          hex_u16(value));

  return decoded;
}

Result Climber::write_if_changed(
    std::uint16_t address,
    std::uint16_t expected,
    const char* operation,
    bool& changed) {
  const auto current = read_register(address);

  if (!current.result.ok) {
    return current.result;
  }

  if (current.value &&
      static_cast<std::uint16_t>(
          *current.value) == expected) {
    return Result::success(
        "L7P register already configured");
  }

  const auto result =
      write_register(
          address,
          expected,
          operation);

  if (result.ok) {
    changed = true;
  }

  return result;
}

/* ============================================================
 * SRV-ON DI 配置
 * ============================================================ */

Result Climber::ensure_srv_on_assignment(
    bool& changed) {
  changed = false;

  int found = -1;

  bool normally_closed = false;

  for (int index = 0;
       index < kL7pDiCount;
       ++index) {
    const auto current =
        read_register(
            di_function_register(
                config_,
                index));

    if (!current.result.ok) {
      return current.result;
    }

    if (!current.value) {
      continue;
    }

    const auto function =
        static_cast<std::uint16_t>(
            *current.value);

    /*
     * 低 7 bit = 功能号
     * bit7 = 常闭
     */
    if ((function & 0x7FU) !=
        config_.profile
            .srv_on_di_function) {
      continue;
    }

    if (found >= 0) {
      return climber_error(
          ErrorCode::kConfigRequired,
          "SRV-ON is assigned to more than one DI; "
          "fix P04.00..P04.07",
          "ensure_srv_on_assignment",
          config_.device_id);
    }

    found = index;

    normally_closed =
        (function & 0x80U) != 0;
  }

  if (found < 0) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "No DI is assigned to SRV-ON; "
        "configure L7P P04.xx before use",
        "ensure_srv_on_assignment",
        config_.device_id);
  }

  srv_on_di_index_ =
      found;

  srv_on_normally_closed_ =
      normally_closed;

  return Result::success(
      "L7P SRV-ON input checked");
}

/* ============================================================
 * 驱动 READY 检查
 * ============================================================ */

Result Climber::inspect_drive_ready(
    bool clear_alarm_if_needed) {
  bool changed = false;

  auto result =
      ensure_srv_on_assignment(
          changed);

  if (!result.ok) {
    return result;
  }

  /*
   * DO3 必须配置为 BRK-OFF。
   */
  const auto do3 =
      read_register(
          config_.profile
              .do3_function_register);

  if (!do3.result.ok) {
    return do3.result;
  }

  if (!do3.value ||
      static_cast<std::uint16_t>(
          *do3.value) !=
          config_.profile
              .brake_off_do_function) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P DO3 is not configured as BRK-OFF; "
        "check P04.12",
        "inspect_drive_ready",
        config_.device_id);
  }

  /*
   * P00.01 = 1
   */
  const auto control_mode =
      read_register(
          config_.profile
              .control_mode_register);

  if (!control_mode.result.ok) {
    return control_mode.result;
  }

  if (!control_mode.value ||
      static_cast<std::uint16_t>(
          *control_mode.value) !=
          config_.profile
              .control_mode_speed) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P is not in speed control mode; "
        "check P00.01",
        "inspect_drive_ready",
        config_.device_id);
  }

  /*
   * P03.00 = 1
   */
  const auto speed_source =
      read_register(
          config_.profile
              .speed_source_register);

  if (!speed_source.result.ok) {
    return speed_source.result;
  }

  if (!speed_source.value ||
      static_cast<std::uint16_t>(
          *speed_source.value) !=
          config_.profile
              .speed_source_internal) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P speed source is not internal speed; "
        "check P03.00",
        "inspect_drive_ready",
        config_.device_id);
  }

  /*
   * P03.01 = 0
   *
   * 方向由 P03.04 正负号决定。
   */
  const auto direction =
      read_register(
          config_.profile
              .direction_register);

  if (!direction.result.ok) {
    return direction.result;
  }

  if (!direction.value ||
      static_cast<std::uint16_t>(
          *direction.value) !=
          config_.profile
              .direction_from_speed_sign) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P direction source is not "
        "P03.04 sign; check P03.01",
        "inspect_drive_ready",
        config_.device_id);
  }

  /*
   * P05.06 = 0
   *
   * 必须使用伺服制动停止模式。
   */
  const auto stop_mode =
      read_register(
          config_.profile
              .stop_mode_register);

  if (!stop_mode.result.ok) {
    return stop_mode.result;
  }

  if (!stop_mode.value ||
      static_cast<std::uint16_t>(
          *stop_mode.value) !=
          config_.profile
              .stop_mode_servo_brake) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P stop mode is not servo-brake mode; "
        "check P05.06",
        "inspect_drive_ready",
        config_.device_id);
  }

  /*
   * 报警。
   */
  auto alarm =
      read_alarm();

  if (!alarm.result.ok) {
    return alarm.result;
  }

  /*
   * 运行状态。
   */
  auto run_state =
      read_register(
          config_.profile
              .run_state_register);

  if (!run_state.result.ok) {
    return run_state.result;
  }

  if (!run_state.value) {
    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P run state is unavailable",
        "inspect_drive_ready",
        config_.device_id,
        true);
  }

  auto state =
      static_cast<std::uint16_t>(
          *run_state.value);

  bool alarm_active =
      (alarm.value &&
       *alarm.value != 0) ||
      ((state &
        kRunStateAlarmMask) != 0);

  /*
   * 启动时允许自动清当前报警。
   *
   * 注意：
   * 先完整 STOP，
   * 再清报警。
   */
  if (alarm_active &&
      clear_alarm_if_needed) {
    const auto stopped =
        stop_sequence();

    if (!stopped.ok) {
      return stopped;
    }

    const auto cleared =
        clear_current_alarm();

    if (!cleared.ok) {
      return cleared;
    }

    if (config_
            .alarm_reset_settle_time
            .count() > 0) {
      std::this_thread::sleep_for(
          config_
              .alarm_reset_settle_time);
    }

    alarm = read_alarm();

    if (!alarm.result.ok) {
      return alarm.result;
    }

    run_state =
        read_register(
            config_.profile
                .run_state_register);

    if (!run_state.result.ok) {
      return run_state.result;
    }

    if (!run_state.value) {
      return climber_error(
          ErrorCode::kSafeHoldActive,
          "L7P run state is unavailable "
          "after alarm clear",
          "inspect_drive_ready",
          config_.device_id,
          true);
    }

    state = static_cast<std::uint16_t>(*run_state.value);

    alarm_active =
        (alarm.value && *alarm.value != 0) ||
        ((state & kRunStateAlarmMask) != 0);
  }

  if (alarm_active) {
    const auto alarm_value =
        alarm.value
            ? static_cast<std::uint16_t>(
                  *alarm.value)
            : 0U;

    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P alarm is active: " +
            hex_u16(alarm_value),
        "inspect_drive_ready",
        config_.device_id,
        true);
  }

  if ((state & kRunStateReadyMask) == 0) {
    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P drive is not ready "
        "(P0B.05.RDY=0)",
        "inspect_drive_ready",
        config_.device_id,
        true);
  }

  configured_ = true;
  health_ = DriverHealth::kReady;

  return Result::success(
      "L7P drive checked and ready");
}

/* ============================================================
 * 可选参数配置
 * ============================================================ */

Result Climber::ensure_drive_configured() {
  if (configured_) {
    return Result::success("L7P already configured");
  }

  bool changed = false;

  auto result =
      write_if_changed(
          config_.profile
              .control_mode_register,
          config_.profile
              .control_mode_speed,
          "configure_control_mode",
          changed);

  if (!result.ok) {
    return result;
  }

  result =
      write_if_changed(
          config_.profile
              .speed_source_register,
          config_.profile
              .speed_source_internal,
          "configure_speed_source",
          changed);

  if (!result.ok) {
    return result;
  }

  result =
      write_if_changed(
          config_.profile
              .direction_register,
          config_.profile
              .direction_from_speed_sign,
          "configure_direction_source",
          changed);

  if (!result.ok) {
    return result;
  }

  /*
   * P05.06 = 0
   */
  result =
      write_if_changed(
          config_.profile
              .stop_mode_register,
          config_.profile
              .stop_mode_servo_brake,
          "configure_stop_mode",
          changed);

  if (!result.ok) {
    return result;
  }

  result =
      write_if_changed(
          config_.profile
              .acceleration_register,
          config_.profile
              .default_acceleration_raw,
          "configure_acceleration",
          changed);

  if (!result.ok) {
    return result;
  }

  result =
      write_if_changed(
          config_.profile
              .deceleration_register,
          config_.profile
              .default_deceleration_raw,
          "configure_deceleration",
          changed);

  if (!result.ok) {
    return result;
  }

  result = ensure_srv_on_assignment(changed);

  if (!result.ok) {
    return result;
  }

  result =
      write_if_changed(
          config_.profile
              .do3_function_register,
          config_.profile
              .brake_off_do_function,
          "configure_brake_output",
          changed);

  if (!result.ok) {
    return result;
  }

  if (changed) {
    result =
        write_register(
            config_.profile
                .save_register,
            config_.profile
                .save_all_parameters,
            "save_l7p_parameters");

    if (!result.ok) {
      return result;
    }
  }

  configured_ = true;

  return Result::success("L7P configured");
}

/* ============================================================
 * 生命周期启动
 * ============================================================ */

Result Climber::start() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  /*
   * 已经正常运行时，重复 start()
   * 不制造额外抱闸动作。
   */
  if (health_ == DriverHealth::kReady &&
      motion_ready_ &&
      srv_on_state_ == SrvOnState::kOn) {
    return Result::success("L7P climber already started");
  }

  srv_on_state_ = SrvOnState::kUnknown;

  target_speed_known_ = false;

  motion_ready_ = false;

  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;

    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P climber config is incomplete",
        "start",
        config_.device_id);
  }

  health_ = DriverHealth::kReady;

  /*
   * ============================================================
   * 启动即停（best-effort，无条件执行）
   *
   * 原因：
   * L7P 是速度模式驱动器，目标转速保存在驱动器内部。
   * 如果上一次运行结束时 SRV-OFF / P03.04=0 没有真正下发到
   * 驱动器（例如程序崩溃、总线占线、或下面两个步骤
   * ensure_drive_configured() / inspect_drive_ready() 提前
   * 返回失败），上一次的目标转速会残留在驱动器里，
   * 一旦使能就会立即按残留转速转动。
   *
   * 因此这里在进入任何可能失败的前置检查之前，
   * 先无条件尽力下发一次停止序列。
   *
   * 该步骤是 best-effort：
   * - 失败只记 WARN，不改变 start() 的返回值；
   * - 后面原有的严格 stop_sequence() 校验仍然保留，
   *   启动完成前的安全前置条件没有任何放宽。
   * ============================================================
   */
  {
    const Result pre_stop =
        stop_sequence();

    if (!pre_stop.ok) {
      SNAPPY_DRIVER_LOG_WARN(
          "climber",
          "start: pre-start best-effort stop_sequence "
          "not confirmed: " +
              pre_stop.detail);
    }
  }

  Result result =
      Result::success();

  /*
   * 如果配置开启，
   * 先确保参数正确。
   */
  if (config_.configure_on_start) {
    result =
        ensure_drive_configured();
  }

  /*
   * 然后做完整 READY 检查。
   */
  if (result.ok) {
    result =
        inspect_drive_ready(
            config_
                .auto_clear_alarm_on_start);
  }

  if (!result.ok) {
    health_ =
        DriverHealth::kFault;

    return result;
  }

  /*
   * 启动完成之前必须处于：
   *
   * P03.04=0
   * 实际速度=0
   * SRV-OFF
   * RUNNING=0
   * BRK-OFF=0
   */
  result =
      stop_sequence();

  if (!result.ok) {
    health_ =
        DriverHealth::kFault;

    return result;
  }

  /*
   * 确保减速度参数正确。
   */
  bool changed = false;

  result =
      write_if_changed(
          config_.profile
              .deceleration_register,
          config_.profile
              .default_deceleration_raw,
          "configure_deceleration_runtime",
          changed);

  if (!result.ok) {
    health_ =
        DriverHealth::kFault;

    return result;
  }

  health_ =
      DriverHealth::kReady;

  return Result::success(
      "L7P climber ready");
}

void Climber::stop() noexcept {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  const auto result =
      safe_stop();

  if (!result.ok) {
    SNAPPY_DRIVER_LOG_WARN(
        kComponent,
        "L7P stop: safe_stop failed: " +
            result.detail);
  }

  health_ =
      DriverHealth::kStopped;
}

/* ============================================================
 * SRV-ON
 * ============================================================ */

Result Climber::set_srv_on(
    bool enabled) {
  if (srv_on_di_index_ < 0) {
    bool changed = false;

    const auto assigned =
        ensure_srv_on_assignment(
            changed);

    if (!assigned.ok) {
      srv_on_state_ =
          SrvOnState::kUnknown;

      return assigned;
    }
  }

  const auto desired_state =
      enabled
          ? SrvOnState::kOn
          : SrvOnState::kOff;

  /*
   * 普通控制允许软件缓存短路。
   *
   * 注意：
   * force_srv_off() 不允许短路。
   */
  if (srv_on_state_ ==
      desired_state) {
    return Result::success(
        enabled
            ? "L7P SRV-ON already enabled"
            : "L7P SRV-ON already disabled");
  }

  const auto srv_bit =
      virtual_io_bit_for_di(
          srv_on_di_index_);

  const auto desired_bit =
      static_cast<std::uint16_t>(
          virtual_io_mask_for_di(
              srv_on_di_index_,
              srv_on_normally_closed_,
              enabled) &
          srv_bit);

  /*
   * P00.26 必须读改写。
   *
   * 不允许覆盖其他虚拟 DI。
   */
  const auto current =
      read_register(
          config_.profile
              .virtual_io_register);

  if (!current.result.ok ||
      !current.value) {
    srv_on_state_ =
        SrvOnState::kUnknown;

    /*
     * SRV-ON：
     * 读失败禁止盲写。
     */
    if (enabled) {
      if (!current.result.ok) {
        return current.result;
      }

      return climber_error(
          ErrorCode::kSafeHoldActive,
          "L7P P00.26 value is unavailable; "
          "SRV-ON was not enabled",
          "l7p_srv_on",
          config_.device_id,
          true);
    }

    /*
     * SRV-OFF：
     * 即使读取失败也允许安全回退写。
     */
    return write_register(
        config_.profile
            .virtual_io_register,
        desired_bit,
        "l7p_srv_off_fallback");
  }

  const auto current_mask =
      static_cast<std::uint16_t>(
          *current.value);

  const auto next_mask =
      static_cast<std::uint16_t>(
          (current_mask &
           static_cast<std::uint16_t>(
               ~srv_bit)) |
          desired_bit);

  const auto result =
      write_register(
          config_.profile
              .virtual_io_register,
          next_mask,
          enabled
              ? "l7p_srv_on"
              : "l7p_srv_off");

  srv_on_state_ =
      result.ok
          ? desired_state
          : SrvOnState::kUnknown;

  return result;
}

/* ============================================================
 * 强制 SRV-OFF
 * ============================================================ */

Result Climber::force_srv_off() {
  if (srv_on_di_index_ < 0) {
    bool changed = false;

    const auto assigned =
        ensure_srv_on_assignment(
            changed);

    if (!assigned.ok) {
      srv_on_state_ =
          SrvOnState::kUnknown;

      return assigned;
    }
  }

  const auto srv_bit =
      virtual_io_bit_for_di(
          srv_on_di_index_);

  /*
   * 注意：
   *
   * 常开：
   * OFF -> bit=0
   *
   * 常闭：
   * OFF -> bit=1
   */
  const auto desired_bit =
      static_cast<std::uint16_t>(
          virtual_io_mask_for_di(
              srv_on_di_index_,
              srv_on_normally_closed_,
              false) &
          srv_bit);

  /*
   * 安全 STOP 路径中，P00.26 的读只允许单次访问。
   *
   * 普通状态 READ 仍使用配置的 read_retry_count=2，
   * 即最多 3 次实际访问；但 force_srv_off() 不能被
   * 最多 900 ms 的普通 READ 重试拖住。
   *
   * 如果这一次读取失败，下面已有的安全 fallback 会
   * 直接写确定性的 SRV-OFF 电平。
   */
  Rs485Request current_request;
  current_request.logical_device = config_.device_id;
  current_request.payload =
      protocols::encode_climber_read_registers(
          config_.station_id,
          config_.profile.virtual_io_register,
          1);
  current_request.side_effect_free_read = true;
  current_request.retry_count = 0;

  const auto current_response =
      bus_.exchange(current_request);

  ClimberReadI32 current{
      current_response.result,
      std::nullopt};

  if (current_response.result.ok) {
    const auto decoded =
        protocols::decode_climber_read_registers_response(
            config_.station_id,
            1,
            current_response.payload);

    if (decoded.result.ok &&
        !decoded.values.empty()) {
      current.result = Result::success();
      current.value =
          static_cast<std::int32_t>(
              decoded.values.front());
    } else {
      current.result = decoded.result;
    }
  }

  std::uint16_t next_mask =
      desired_bit;

  if (current.result.ok &&
      current.value) {
    const auto current_mask =
        static_cast<std::uint16_t>(
            *current.value);

    next_mask =
        static_cast<std::uint16_t>(
            (current_mask &
             static_cast<std::uint16_t>(
                 ~srv_bit)) |
            desired_bit);
  } else {
    /*
     * 最终安全动作：
     * P00.26 读失败仍然发送确定性的
     * SRV-OFF 电平。
     */
    SNAPPY_DRIVER_LOG_WARN(
        kComponent,
        "L7P P00.26 read failed before "
        "force SRV-OFF; using safety fallback");
  }

  /*
   * 这里绝对不能被 srv_on_state_
   * 缓存短路。
   */
  const auto result =
      write_register(
          config_.profile
              .virtual_io_register,
          next_mask,
          "l7p_force_srv_off");

  srv_on_state_ =
      result.ok
          ? SrvOnState::kOff
          : SrvOnState::kUnknown;

  return result;
}

/* ============================================================
 * 抱闸释放
 * ============================================================ */

Result Climber::wait_for_brake_release() {
  std::chrono::milliseconds wait =
      config_.brake_release_margin;

  const auto brake_wait =
      read_register(
          config_.profile
              .brake_release_wait_register);

  if (brake_wait.result.ok &&
      brake_wait.value &&
      *brake_wait.value > 0) {
    wait +=
        std::chrono::milliseconds(
            static_cast<std::int32_t>(
                *brake_wait.value));
  }

  if (wait.count() > 0) {
    std::this_thread::sleep_for(
        wait);
  }

  return Result::success(
      "L7P brake release wait elapsed");
}

/* ============================================================
 * 停止状态读取
 * ============================================================ */

Result Climber::read_stop_state(
    bool& running,
    bool& brake_off) {
  /*
   * P0B.05 -> P0B.18
   *
   * 一次 FC03 读完整区间：
   *
   * [0]  P0B.05 RUNNING
   * ...
   * [13] P0B.18 DO3
   *
   * 这样每次轮询只占用一次共享 RS485。
   */
  const auto count =
      static_cast<std::uint16_t>(
          config_.profile
              .physical_do_register -
          config_.profile
              .run_state_register +
          1);

  const auto read =
      read_registers(
          config_.profile
              .run_state_register,
          count);

  if (!read.result.ok) {
    return read.result;
  }

  const auto do_offset =
      static_cast<std::size_t>(
          config_.profile
              .physical_do_register -
          config_.profile
              .run_state_register);

  if (read.values.size() <=
      do_offset) {
    return climber_error(
        ErrorCode::kRs485LengthMismatch,
        "L7P stop-state read returned "
        "too few registers",
        "read_stop_state",
        config_.device_id,
        true);
  }

  const auto run_state =
      read.values[0];

  const auto physical_do =
      read.values[do_offset];

  running =
      (run_state &
       kRunStateRunningMask) != 0;

  brake_off =
      (physical_do &
       kBrakeOffPhysicalMask) != 0;

  SNAPPY_DRIVER_LOG_DEBUG(
      kComponent,
      "L7P STOP feedback run_state=" +
          hex_u16(run_state) +
          " physical_do=" +
          hex_u16(physical_do) +
          " running=" +
          std::to_string(running ? 1 : 0) +
          " brake_off=" +
          std::to_string(brake_off ? 1 : 0));

  return Result::success();
}

/* ============================================================
 * 等待真正停止 + 抱闸
 * ============================================================ */

Result Climber::
    wait_for_disabled_and_brake_closed() {
  const auto deadline =
      std::chrono::steady_clock::now() +
      config_.stop_timeout;

  int stable_samples = 0;

  while (std::chrono::steady_clock::now() <
         deadline) {
    bool running = false;
    bool brake_off = false;

    const auto result =
        read_stop_state(
            running,
            brake_off);

    SNAPPY_DRIVER_LOG_DEBUG(
        kComponent,
        "L7P STOP brake_sample result_ok=" +
            std::to_string(result.ok ? 1 : 0) +
            " running=" +
            std::to_string(running ? 1 : 0) +
            " brake_off=" +
            std::to_string(brake_off ? 1 : 0) +
            " stable_samples=" +
            std::to_string(stable_samples));

    if (result.ok &&
        !running &&
        !brake_off) {
      ++stable_samples;

      /*
       * 连续两个采样：
       *
       * RUNNING=0
       * BRK-OFF=0
       */
      if (stable_samples >= 2) {
        return Result::success(
            "L7P disabled and brake closed");
      }
    } else {
      stable_samples = 0;
    }

    std::this_thread::sleep_for(
        config_.stop_poll_period);
  }

  return climber_error(
      ErrorCode::kSafeHoldActive,
      "L7P did not reach "
      "RUNNING=0 and BRK-OFF=0 "
      "before timeout",
      "wait_for_disabled_and_brake_closed",
      config_.device_id,
      true);
}

/* ============================================================
 * 等待实际速度下降
 * ============================================================ */

Result Climber::wait_for_speed_below(
    std::int32_t rpm_threshold) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      config_.stop_timeout;

  /*
   * 实际速度单位：
   * 0.1 RPM
   */
  const auto raw_threshold = std::abs(rpm_threshold) * 10;

  int stable_samples = 0;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto speed =
        read_s32(
            config_.profile
                .speed_filtered_register);

    SNAPPY_DRIVER_LOG_DEBUG(
        kComponent,
        "L7P STOP speed_sample result_ok=" +
            std::to_string(speed.result.ok ? 1 : 0) +
            " actual_raw=" +
            (speed.value
                 ? std::to_string(*speed.value)
                 : std::string("unknown")) +
            " threshold_raw=" +
            std::to_string(raw_threshold) +
            " stable_samples=" +
            std::to_string(stable_samples));

    if (speed.result.ok &&
        speed.value &&
        std::abs(*speed.value) <=
            raw_threshold) {
      ++stable_samples;

      if (stable_samples >= 2) {
        return Result::success("L7P speed below stop threshold");
      }
    } else {
      stable_samples = 0;
    }

    std::this_thread::sleep_for(
        config_.stop_poll_period);
  }

  return climber_error(
      ErrorCode::kSafeHoldActive,
      "L7P actual speed did not fall "
      "below stop threshold before timeout",
      "wait_for_speed_below",
      config_.device_id,
      true);
}

/* ============================================================
 * 项目1式完整 STOP
 * ============================================================ */

Result Climber::stop_sequence() {
  const auto stop_begin =
      std::chrono::steady_clock::now();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP sequence begin target_known=" +
          std::to_string(target_speed_known_ ? 1 : 0) +
          " target_rpm=" +
          std::to_string(target_speed_rpm_) +
          " motion_ready=" +
          std::to_string(motion_ready_ ? 1 : 0) +
          " srv_on=" +
          std::to_string(srv_on_state_ == SrvOnState::kOn ? 1 : 0));

  /*
   * 一旦进入 STOP，软件层立即禁止继续运动。
   */
  motion_ready_ = false;

  Result first_failure =
      Result::success();

  /*
   * ----------------------------------------------------------
   * 第一阶段：目标速度置零。
   *
   * 这里只负责给 L7P 下达减速/停止目标，
   * 不等待实际速度反馈。
   * ----------------------------------------------------------
   */
  const auto target_stage_begin =
      std::chrono::steady_clock::now();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=target_speed_zero");

  const auto zero =
      write_target_speed_rpm(0);

  const auto target_elapsed =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::steady_clock::now() -
          target_stage_begin)
          .count();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=target_speed_zero result_ok=" +
          std::to_string(zero.ok ? 1 : 0) +
          " elapsed_ms=" +
          std::to_string(target_elapsed));

  if (!zero.ok) {
    first_failure = zero;
  }

  /*
   * ----------------------------------------------------------
   * 第二阶段：立即 SRV-OFF。
   *
   * 关键安全原则：
   * 实际速度 READ 不能阻塞 SRV-OFF。
   * 即使 0x0B2C 连续 timeout，也必须尽快进入
   * 驱动禁能/抱闸路径。
   * ----------------------------------------------------------
   */
  const auto srv_stage_begin =
      std::chrono::steady_clock::now();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=srv_off");

  const auto srv_off =
      force_srv_off();

  const auto srv_elapsed =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::steady_clock::now() -
          srv_stage_begin)
          .count();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=srv_off result_ok=" +
          std::to_string(srv_off.ok ? 1 : 0) +
          " elapsed_ms=" +
          std::to_string(srv_elapsed));

  if (!srv_off.ok &&
      first_failure.ok) {
    first_failure = srv_off;
  }

  /*
   * ----------------------------------------------------------
   * 第三阶段：确认 L7P 已禁能并关闭抱闸输出。
   *
   * 这是安全状态确认，不再等待实际速度反馈。
   * ----------------------------------------------------------
   */
  const auto brake_stage_begin = std::chrono::steady_clock::now();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=disabled_and_brake_closed");

  const auto disabled =
      wait_for_disabled_and_brake_closed();

  const auto brake_elapsed =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::steady_clock::now() -
          brake_stage_begin)
          .count();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=disabled_and_brake_closed result_ok=" +
          std::to_string(disabled.ok ? 1 : 0) +
          " elapsed_ms=" +
          std::to_string(brake_elapsed));

  if (!disabled.ok &&
      first_failure.ok) {
    first_failure = disabled;
  }

  /*
   * ----------------------------------------------------------
   * 第四阶段：实际速度反馈确认。
   *
   * 注意：这一阶段只能用于“确认”，不能再阻塞前面的
   * SRV-OFF/抱闸安全动作。
   *
   * read_retry_count=2 时，scheduler 对无副作用 READ
   * 最多执行 3 次实际访问：
   *   300 ms × 3 = 900 ms（按当前 response_deadline）。
   * stop_timeout=1000 ms，给本阶段留下约 100 ms 裕量。
   * ----------------------------------------------------------
   */
  const auto speed_stage_begin =
      std::chrono::steady_clock::now();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=speed_below threshold_rpm=" +
          std::to_string(kStopSpeedRpm));

  const auto speed_stopped = wait_for_speed_below(kStopSpeedRpm);

  const auto speed_elapsed =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - speed_stage_begin)
          .count();

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP stage=speed_below result_ok=" +
          std::to_string(speed_stopped.ok ? 1 : 0) +
          " elapsed_ms=" +
          std::to_string(speed_elapsed));

  if (!speed_stopped.ok &&
      first_failure.ok) {
    first_failure = speed_stopped;
  }

  const auto total_elapsed =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          std::chrono::steady_clock::now() -
          stop_begin)
          .count();

  if (!first_failure.ok) {
    SNAPPY_DRIVER_LOG_ERROR(
        kComponent,
        "L7P STOP sequence failed operation=" +
            first_failure.operation +
            " code=" +
            std::to_string(static_cast<int>(first_failure.code)) +
            " detail=" +
            first_failure.detail +
            " total_elapsed_ms=" +
            std::to_string(total_elapsed));

    health_ = DriverHealth::kFault;

    return first_failure;
  }

  health_ = DriverHealth::kReady;

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P STOP sequence complete total_elapsed_ms=" +
          std::to_string(total_elapsed));

  return Result::success("L7P stop sequence completed");
}

/* ============================================================
 * 运动前置检查 / ENABLE
 * ============================================================ */

Result Climber::preflight_motion() {
  if (health_ != DriverHealth::kReady) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P climber is not ready",
        "preflight_motion",
        config_.device_id);
  }

  /*
   * 参数尚未确认时先检查。
   */
  if (!configured_) {
    const auto ready =
        inspect_drive_ready(
            config_
                .auto_clear_alarm_on_start);

    if (!ready.ok) {
      return ready;
    }
  }

  /*
   * 报警检查。
   */
  const auto alarm = read_alarm();

  if (!alarm.result.ok) {
    return alarm.result;
  }

  if (alarm.value &&
      *alarm.value != 0) {
    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P alarm is active: " +
            hex_u16(
                static_cast<
                    std::uint16_t>(
                    *alarm.value)),
        "preflight_motion",
        config_.device_id,
        true);
  }

  /*
   * READY / FAULT 检查。
   */
  const auto run_state =
      read_register(
          config_.profile
              .run_state_register);

  if (!run_state.result.ok) {
    return run_state.result;
  }

  if (!run_state.value) {
    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P run state is unavailable",
        "preflight_motion",
        config_.device_id,
        true);
  }

  const auto state = static_cast<std::uint16_t>( *run_state.value);

  if ((state &
       kRunStateAlarmMask) != 0 ||
      (state &
       kRunStateReadyMask) == 0) {
    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P drive is not ready for motion",
        "preflight_motion",
        config_.device_id,
        true);
  }

  /*
   * 读取实际 STOP 状态。
   */
  bool running = false;
  bool brake_off = false;

  const auto stop_state =
      read_stop_state(
          running,
          brake_off);

  if (!stop_state.ok) {
    return stop_state;
  }

  /*
   * 如果当前不是干净的：
   *
   * RUN=0
   * BRK-OFF=0
   * SRV-OFF
   *
   * 则先完整 STOP。
   */
  if (running ||
      brake_off ||
      srv_on_state_ !=
          SrvOnState::kOff) {
    const auto stopped = stop_sequence();

    if (!stopped.ok) {
      return stopped;
    }
  }

  /*
   * ----------------------------------------------------------
   * SRV-ON
   * ----------------------------------------------------------
   */
  const auto srv_on = set_srv_on(true);

  if (!srv_on.ok) {
    health_ = DriverHealth::kFault;

    return srv_on;
  }

  /*
   * ----------------------------------------------------------
   * 等待抱闸释放。
   *
   * P04.38 + margin
   * ----------------------------------------------------------
   */
  const auto brake =
      wait_for_brake_release();

  if (!brake.ok) {
    health_ = DriverHealth::kFault;

    return brake;
  }

  /*
   * ----------------------------------------------------------
   * SRV-ON 后重新读取 READY + DO3。
   *
   * P0B.05 READY=1
   * P0B.05 FAULT=0
   * P0B.18 DO3 BRK-OFF=1
   * ----------------------------------------------------------
   */
  const auto enabled_state =
      read_registers(
          config_.profile
              .run_state_register,
          static_cast<
              std::uint16_t>(
              config_.profile
                  .physical_do_register -
              config_.profile
                  .run_state_register +
              1));

  if (!enabled_state.result.ok) {
    health_ = DriverHealth::kFault;

    return enabled_state.result;
  }

  const auto do_offset =
      static_cast<std::size_t>(
          config_.profile
              .physical_do_register -
          config_.profile
              .run_state_register);

  if (enabled_state.values.size() <= do_offset) {
    health_ =
        DriverHealth::kFault;

    return climber_error(
        ErrorCode::kRs485LengthMismatch,
        "L7P enable-state read returned "
        "too few registers",
        "preflight_motion",
        config_.device_id,
        true);
  }

  const auto enabled_run_state = enabled_state.values[0];

  const auto enabled_do = enabled_state.values[do_offset];

  if ((enabled_run_state & kRunStateReadyMask) == 0 ||
      (enabled_run_state & kRunStateAlarmMask) != 0 ||
      (enabled_do & kBrakeOffPhysicalMask) == 0) {
    health_ = DriverHealth::kFault;

    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P SRV-ON/READY/BRK-OFF "
        "verification failed",
        "preflight_motion",
        config_.device_id,
        true);
  }

  motion_ready_ = true;

  return Result::success("L7P motion preflight completed");
}

/* ============================================================
 * 目标速度写入
 * ============================================================ */

Result Climber::write_target_speed_rpm(std::int32_t speed_rpm) {
  const auto limited =
      std::clamp(
          speed_rpm,
          -config_.profile.max_speed_rpm,
          config_.profile.max_speed_rpm);

  SNAPPY_DRIVER_LOG_DEBUG(
      kComponent,
      "L7P target speed write requested_rpm=" +
          std::to_string(speed_rpm) +
          " limited_rpm=" +
          std::to_string(limited) +
          " register=" +
          hex_u16(config_.profile.target_speed_register));

    auto result =
      write_register(
          config_.profile
              .target_speed_register,
          protocols::
              encode_climber_signed_i16(limited),
          "write_l7p_target_speed");

  if (!result.ok && result.code == ErrorCode::kRs485Timeout) {
    result = write_register(
            config_.profile
                .target_speed_register,
            protocols::
                encode_climber_signed_i16(limited),
            "write_l7p_target_speed");
  }


  if (result.ok) {
    target_speed_rpm_ = limited;

    target_speed_known_ = true;
  } else {
    target_speed_known_ = false;
  }

  return result;
}

/* ============================================================
 * 统一速度控制入口
 * ============================================================ */

Result Climber::set_target_speed_raw(std::int32_t speed_rpm) {
  std::lock_guard<std::recursive_mutex>
      lock(device_mutex_);

    if (health_ != DriverHealth::kReady) {
    /*
     * 故障态先用两帧只读复核尝试自愈，不让一次 300ms 超时把设备锁死。
     */
    if (health_ == DriverHealth::kFault) {
      const auto alarm = read_alarm();
      const auto run_state = read_register(config_.profile.run_state_register);
      if (alarm.result.ok && alarm.value && *alarm.value == 0 && run_state.result.ok && run_state.value) {
        const auto state = static_cast<std::uint16_t>(*run_state.value);
        if ((state & kRunStateAlarmMask) == 0 && (state & kRunStateReadyMask) != 0) {
          health_ = DriverHealth::kReady;
          SNAPPY_DRIVER_LOG_WARN(kComponent, "L7P climber recovered from fault by read-only probe");
        }
      }
    }
    if (health_ != DriverHealth::kReady) {
      return climber_error(
          ErrorCode::kConfigRequired,
          "L7P climber is not ready",
          "set_target_speed_raw",
          config_.device_id);
    }
  }


  const auto limited = std::clamp(speed_rpm, -config_.profile.max_speed_rpm, config_.profile.max_speed_rpm);

  SNAPPY_DRIVER_LOG_INFO(
      kComponent,
      "L7P SPEED command requested_rpm=" +
          std::to_string(speed_rpm) +
          " limited_rpm=" +
          std::to_string(limited) +
          " current_target_rpm=" +
          std::to_string(target_speed_rpm_) +
          " target_known=" +
          std::to_string(target_speed_known_ ? 1 : 0) +
          " motion_ready=" +
          std::to_string(motion_ready_ ? 1 : 0) +
          " srv_on=" +
          std::to_string(srv_on_state_ == SrvOnState::kOn ? 1 : 0));

  /*
   * ----------------------------------------------------------
   * 0 RPM = 真正 STOP
   *
   * 不再只是写 P03.04=0。
   * ----------------------------------------------------------
   */
  if (limited == 0) {
    return stop_sequence();
  }

  if (motion_ready_ &&
      srv_on_state_ ==
          SrvOnState::kOn &&
      target_speed_rpm_ != 0) {
    const bool same_direction =
        (target_speed_rpm_ > 0) ==
        (limited > 0);

    if (same_direction) {
      if (target_speed_rpm_ == limited) {
        return Result::success(
            "L7P target speed unchanged");
      }

      return write_target_speed_rpm(limited);
    }

    /*
     * 反向：
     * 必须完整 STOP。
     */
    const auto stopped = stop_sequence();

    if (!stopped.ok) {
      return stopped;
    }
  }

  /*
   * ----------------------------------------------------------
   * ENABLE / PRE-FLIGHT
   * ----------------------------------------------------------
   */
  const auto ready = preflight_motion();

  if (!ready.ok) {
    return ready;
  }

  /*
   * ----------------------------------------------------------
   * 最后才写真正的运行速度。
   * ----------------------------------------------------------
   */
  const auto result = write_target_speed_rpm(limited);

  if (!result.ok) {
    /*
     * 速度命令写失败以后，
     * 不允许继续认为已经允许运动。
     *
     * 立即回滚 SRV-OFF。
     */
    const auto rollback = force_srv_off();

    if (!rollback.ok) {
      SNAPPY_DRIVER_LOG_WARN(
          kComponent,
          "L7P target write failed and "
          "SRV-OFF rollback also failed: " +
              rollback.detail);
    }

    motion_ready_ = false;

    /*
     * 此时设备状态未知，
     * 必须进入故障状态，
     * 等待 clear_fault()。
     */
    health_ = DriverHealth::kFault;
  }

  return result;
}

Result Climber::
    set_target_speed_rad_per_s(double speed_rad_per_s) {
  return set_target_speed_raw(
      rad_per_s_to_rpm(speed_rad_per_s));
}

Result Climber::
    set_target_speed_rps(double speed_rad_per_s) {
  /*
   * 保持原有接口语义，
   * 不改变现有调用 ABI。
   */
  return set_target_speed_rad_per_s(speed_rad_per_s);
}

/* ============================================================
 * 正常 STOP
 * ============================================================ */

Result Climber::disable() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  /*
   * 唯一正常 STOP 入口。
   */
  return stop_sequence();
}

/* ============================================================
 * 快速安全 STOP
 * ============================================================ */

Result Climber::safe_stop() noexcept {
  SNAPPY_DRIVER_LOG_WARN(
      kComponent,
      "L7P SAFE_STOP begin target_known=" +
          std::to_string(target_speed_known_ ? 1 : 0) +
          " target_rpm=" +
          std::to_string(target_speed_rpm_) +
          " motion_ready=" +
          std::to_string(motion_ready_ ? 1 : 0));

  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  Result first_failure = Result::success();

  /*
   * 第一动作：
   * 尽力写 P03.04=0。
   *
   * 不等待实际速度。
   */
  if (!target_speed_known_ ||
      target_speed_rpm_ != 0) {
    const auto zero =
        write_target_speed_rpm(0);

    if (!zero.ok) {
      first_failure =
          zero;
    }
  }

  /*
   * 第二动作：
   * 立即强制 SRV-OFF。
   *
   * 不等待反馈。
   */
  const auto srv_off = force_srv_off();

  if (!srv_off.ok &&
      first_failure.ok) {
    first_failure =
        srv_off;
  }

  /*
   * 无论成功失败，
   * 软件都禁止继续运动。
   */
  motion_ready_ = false;

  if (!first_failure.ok) {
    SNAPPY_DRIVER_LOG_ERROR(
        kComponent,
        "L7P SAFE_STOP failed operation=" +
            first_failure.operation +
            " code=" +
            std::to_string(static_cast<int>(first_failure.code)) +
            " detail=" +
            first_failure.detail);

    health_ = DriverHealth::kFault;

    return first_failure;
  }

  SNAPPY_DRIVER_LOG_WARN(
      kComponent,
      "L7P SAFE_STOP complete");

  return Result::success(
      "L7P safe stop accepted");
}

/* ============================================================
 * 状态读取
 * ============================================================ */

ClimberReadI32
Climber::read_alarm() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  return read_register(
      config_.profile
          .alarm_register);
}

ClimberReadI32
Climber::read_actual_speed_raw() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  return read_s32(
      config_.profile
          .speed_filtered_register);
}

ClimberReadDouble
Climber::
    read_actual_speed_rad_per_s() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  const auto raw =
      read_s32(
          config_.profile
              .speed_filtered_register);

  if (!raw.result.ok ||
      !raw.value) {
    return {
        raw.result,
        std::nullopt};
  }

  return {
      Result::success(),
      rpm_raw_0p1_to_rad_per_s(
          *raw.value)};
}

ClimberReadDouble
Climber::
    read_actual_speed_rps() {
  return read_actual_speed_rad_per_s();
}

ClimberReadI32
Climber::read_bus_voltage_raw() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  return read_register(
      0x0B0AU);
}

/* ============================================================
 * 清报警
 * ============================================================ */

Result Climber::clear_current_alarm() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  if (config_.station_id == 0) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P station_id is not configured",
        "clear_current_alarm",
        config_.device_id);
  }

  Rs485Request request;

  request.logical_device = config_.device_id;

  request.payload =
      protocols::
          encode_climber_reset_current_alarm(
              config_.station_id,
              config_.profile);

  const auto response = bus_.exchange(request);

  if (!response.result.ok) {
    health_ = DriverHealth::kFault;

    return response.result;
  }

  const auto result =
      protocols::
          decode_climber_write_response(
              config_.station_id,
              config_.profile
                  .auxiliary_function_register,
              config_.profile
                  .reset_current_alarm_value,
              response.payload);

  if (!result.ok) {
    health_ = DriverHealth::kFault;
  }

  return result;
}

Result Climber::clear_history_alarm() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  if (config_.station_id == 0) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P station_id is not configured",
        "clear_history_alarm",
        config_.device_id);
  }

  Rs485Request request;

  request.logical_device = config_.device_id;

  request.payload =
      protocols::
          encode_climber_reset_history_alarm(
              config_.station_id,
              config_.profile);

  const auto response =
      bus_.exchange(request);

  if (!response.result.ok) {
    health_ = DriverHealth::kFault;

    return response.result;
  }

  const auto result =
      protocols::
          decode_climber_write_response(
              config_.station_id,
              config_.profile
                  .auxiliary_function_register,
              config_.profile
                  .reset_history_alarm_value,
              response.payload);

  if (!result.ok) {
    health_ = DriverHealth::kFault;
  }

  return result;
}

/* ============================================================
 * 故障恢复
 * ============================================================ */

Result Climber::clear_fault() {
  std::lock_guard<
      std::recursive_mutex>
      lock(device_mutex_);

  if (config_.station_id == 0) {
    return climber_error(
        ErrorCode::kConfigRequired,
        "L7P station_id is not configured",
        "clear_fault",
        config_.device_id);
  }

  /*
   * 清故障的第一原则：
   *
   * 先禁止运动。
   */
  motion_ready_ = false;

  /*
   * 1. 目标速度 = 0
   */
  const auto zero = write_target_speed_rpm(0);

  /*
   * 2. 强制 SRV-OFF
   */
  const auto srv_off = force_srv_off();

  /*
   * 3. 清当前报警
   */
  const auto current = clear_current_alarm();

  /*
   * 4. 清历史报警
   */
  const auto history =
      clear_history_alarm();

  if (config_
          .alarm_reset_settle_time
          .count() > 0) {
    std::this_thread::sleep_for(
        config_
            .alarm_reset_settle_time);
  }

  /*
   * SRV-OFF 是恢复控制权的硬条件。
   */
  if (!srv_off.ok) {
    health_ = DriverHealth::kFault;

    return srv_off;
  }

  if (!current.ok) {
    health_ = DriverHealth::kFault;

    return current;
  }

  if (!history.ok) {
    health_ = DriverHealth::kFault;

    return history;
  }

  /*
   * 5. 报警确认
   */
  const auto alarm = read_alarm();

  if (!alarm.result.ok) {
    health_ =
        DriverHealth::kFault;

    return alarm.result;
  }

  if (alarm.value &&
      *alarm.value != 0) {
    health_ =
        DriverHealth::kFault;

    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P alarm remains active after clear: " +
            hex_u16(
                static_cast<
                    std::uint16_t>(
                    *alarm.value)),
        "clear_fault",
        config_.device_id,
        true);
  }

  /*
   * 6. 只做一次硬件停止状态确认。
   *
   * 不进入 500ms 轮询。
   *
   * 这样清故障不会因为 RUN/DO3
   * 反馈异常而长期占用共享 RS485。
   */
  bool running = false;
  bool brake_off = false;

  const auto state =
      read_stop_state(
          running,
          brake_off);

  if (!state.ok) {
    health_ = DriverHealth::kFault;

    return state;
  }

  if (running ||
      brake_off) {
    health_ = DriverHealth::kFault;

    return climber_error(
        ErrorCode::kSafeHoldActive,
        "L7P clear-fault verification failed: "
        "RUNNING or BRK-OFF remains active",
        "clear_fault",
        config_.device_id,
        true);
  }

  /*
   * 7. 目标速度写失败也不能伪装成 Ready。
   */
  if (!zero.ok) {
    health_ = DriverHealth::kFault;

    return zero;
  }

  target_speed_rpm_ = 0;
  target_speed_known_ = true;

  srv_on_state_ = SrvOnState::kOff;

  motion_ready_ = false;

  health_ = DriverHealth::kReady;

  return Result::success("L7P fault cleared and climber ready");
}

}  // namespace snappy::drivers::devices