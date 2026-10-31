#include "snappy/drivers/devices/brush_motor.hpp"
/**
 * @file brush_motor.cpp
 * @brief 毛刷电机设备驱动实现
 *
 * @details
 * - 转速越界直接拒绝，不做静默截断。
 * - 写入后按需回读故障码，用于健康状态判定。
 */
 
#include <string>
#include <thread>
#include <utility>

#include <iostream>

namespace snappy::drivers::devices {
namespace {

constexpr std::uint16_t kBd201sMaxGear = 15;

// 毛刷和继电器共用 /dev/ttyACM2。写请求目前不会由 RS485 scheduler 自动重试，
// 因此对毛刷的状态型写入在超时/IO 异常时允许额外重发 1 次。
// 这些写入是方向、速度、运行/停止等“设置目标状态”的幂等操作。
constexpr int kBrushWriteRetryCount = 1;
constexpr std::uint16_t kBd201sMinDirectRpm = 200;
constexpr std::uint16_t kBd201sMaxDirectRpm = 3500;

constexpr std::uint16_t kBd201sGearRpm[16] = {
    1500, 1700, 1900, 2100,
    2300, 2500, 2700, 2900,
    3100, 3300, 3500, 3500,
    3500, 3500, 3500, 3500,
};

bool valid_speed_command(std::uint16_t speed_raw) {
  return speed_raw <= kBd201sMaxGear ||
         (speed_raw >= kBd201sMinDirectRpm &&
          speed_raw <= kBd201sMaxDirectRpm);
}

std::uint16_t speed_command_to_register_value(std::uint16_t speed_raw) {
  if (speed_raw <= kBd201sMaxGear) {
    return kBd201sGearRpm[speed_raw];
  }

  if (speed_raw > kBd201sMaxDirectRpm) {
    return kBd201sMaxDirectRpm;
  }

  return speed_raw;
}

Result read_back_register(IRs485Bus& bus,
                          const BrushMotorConfig& config,
                          std::uint16_t register_address,
                          std::uint16_t expected_value) {
  Rs485Request request;
  request.logical_device = config.device_id;
  request.payload = protocols::encode_read_registers(
      {config.station_id,
       protocols::kModbusReadHoldingRegisters,
       register_address,
       1});

  const auto response = bus.exchange(request);
  if (!response.result.ok) {
    return response.result;
  }

  const auto window = protocols::find_modbus_frame(
      config.station_id,
      protocols::kModbusReadHoldingRegisters,
      response.payload);
  if (!window.result.ok) {
    return window.result;
  }

  const std::vector<std::uint8_t> candidate(
      response.payload.begin() + static_cast<std::ptrdiff_t>(window.offset),
      response.payload.begin() +
          static_cast<std::ptrdiff_t>(window.offset + window.size));

  const auto decoded = protocols::decode_read_registers_response(
      config.station_id,
      protocols::kModbusReadHoldingRegisters,
      1,
      candidate);
  if (!decoded.result.ok) {
    return decoded.result;
  }
  if (decoded.values.empty()) {
    return Result::failure(ErrorCode::kIoError,
                           "brush motor readback is empty",
                           "read_back_register",
                           config.device_id);
  }

  if (decoded.values[0] != expected_value) {
    return Result::failure(
        ErrorCode::kIoError,
        "brush motor readback mismatch: register=" +
            std::to_string(register_address) + " expected=" +
            std::to_string(expected_value) + " actual=" +
            std::to_string(decoded.values[0]),
        "read_back_register",
        config.device_id);
  }

  return Result::success("brush motor write confirmed by readback");
}

}  // namespace

BrushMotorConfig make_brush_motor_config() {
  return {};
}

BrushMotor::BrushMotor(IRs485Bus& bus, BrushMotorConfig config)
    : bus_(bus), config_(std::move(config)) {}

Result BrushMotor::start() {
  if (config_.device_id.empty() || config_.station_id == 0) {
    health_ = DriverHealth::kUnconfigured;
    return Result::failure(ErrorCode::kConfigRequired,
                           "brush motor config is incomplete",
                           "start",
                           config_.device_id);
  }
  health_ = DriverHealth::kReady;
  return Result::success("brush motor ready");
}

DriverHealth BrushMotor::health() const noexcept {
  return health_;
}

Result BrushMotor::write_and_expect(
    const std::vector<std::uint8_t>& request_payload,
    std::uint16_t expected_register,
    std::uint16_t expected_value) {
  Rs485Request request{config_.device_id, request_payload};

  Result last_result = Result::failure(
      ErrorCode::kIoError,
      "brush motor write did not complete",
      "write_and_expect",
      config_.device_id);

  for (int attempt = 0; attempt <= kBrushWriteRetryCount; ++attempt) {
    const auto response = bus_.exchange(request);
    last_result = response.result;

    if (response.result.ok) {
      const auto decoded = protocols::decode_brush_motor_write_response(
          config_.station_id,
          expected_register,
          expected_value,
          response.payload);

      if (decoded.ok) {
        return decoded;
      }

      if (expected_register != config_.profile.control_register) {
        const auto verify =
            read_back_register(bus_, config_, expected_register, expected_value);
        if (verify.ok) {
          return verify;
        }
      }

      return decoded;
    }

    // 对共享 USB-RS485 总线上的瞬时超时/IO 错误，重发一次。
    // scheduler 的 recovery_delay_ms 会负责总线恢复间隔，不在这里额外 sleep，
    // 避免把所有正常命令的时延人为拉长。
    const bool retryable =
        response.result.code == ErrorCode::kRs485Timeout ||
        response.result.code == ErrorCode::kIoError;
    if (!retryable || attempt == kBrushWriteRetryCount) {
      break;
    }

    std::cerr << "[brush.command] write retry attempt=" << (attempt + 1)
              << " register=" << expected_register
              << " detail=" << response.result.detail << "\n";
  }

  // 对非控制寄存器，即使写应答丢失，也优先通过读回确认“写是否其实已经成功”。
  if (expected_register != config_.profile.control_register &&
      (last_result.code == ErrorCode::kRs485Timeout ||
       last_result.code == ErrorCode::kIoError)) {
    const auto verify =
        read_back_register(bus_, config_, expected_register, expected_value);
    if (verify.ok) {
      return verify;
    }
  }

  return last_result;
}

Result BrushMotor::stop_motion() {
  const auto result = write_and_expect(
      protocols::encode_brush_motor_enable(config_.station_id,
                                           config_.profile,
                                           false),
      config_.profile.run_register,
      config_.profile.stop_value);
  if (result.ok) {
    running_ = false;
  }
  return result;
}

Result BrushMotor::set_speed_raw(std::uint16_t speed_raw) {
  if (!valid_speed_command(speed_raw)) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "brush motor speed must be gear 0..15 or rpm 200..4500",
        "set_speed_raw",
        config_.device_id);
  }

  const auto register_speed = speed_command_to_register_value(speed_raw);

  return write_and_expect(
      protocols::encode_brush_motor_speed(config_.station_id,
                                          config_.profile,
                                          register_speed),
      config_.profile.speed_register,
      register_speed);
}

Result BrushMotor::set_direction(protocols::BrushMotorDirection direction) {
  if (running_ && current_direction_ && *current_direction_ != direction) {
    const auto stopped = stop_motion();
    if (!stopped.ok) {
      return stopped;
    }
    std::this_thread::sleep_for(config_.direction_dead_time);
  }

  const auto value = direction == protocols::BrushMotorDirection::kClockwise
                         ? config_.profile.clockwise_value
                         : config_.profile.counter_clockwise_value;

  const auto result = write_and_expect(
      protocols::encode_brush_motor_direction(config_.station_id,
                                              config_.profile,
                                              direction),
      config_.profile.direction_register,
      value);
  if (result.ok) {
    current_direction_ = direction;
  }
  return result;
}

Result BrushMotor::run(protocols::BrushMotorDirection direction,
                       std::uint16_t speed_raw) {
  if (health_ != DriverHealth::kReady) {
    return Result::failure(ErrorCode::kConfigRequired,
                           "brush motor is not ready",
                           "run",
                           config_.device_id);
  }

  const auto dir = set_direction(direction);
  if (!dir.ok) return dir;

  const auto speed = set_speed_raw(speed_raw);
  if (!speed.ok) return speed;

  const auto enabled = write_and_expect(
      protocols::encode_brush_motor_enable(config_.station_id,
                                           config_.profile,
                                           true),
      config_.profile.run_register,
      config_.profile.run_value);
  if (enabled.ok) {
    running_ = true;
  }
  return enabled;
}

Result BrushMotor::configure_motion(std::uint16_t acceleration_raw,
                                    std::uint16_t deceleration_raw,
                                    std::uint16_t run_time_20ms_units,
                                    std::uint16_t direction_delay_20ms_units,
                                    std::uint16_t max_input_power_w) {
  const auto acc = write_and_expect(
      protocols::encode_brush_motor_acceleration(config_.station_id,
                                                 config_.profile,
                                                 acceleration_raw),
      config_.profile.acceleration_register,
      acceleration_raw);
  if (!acc.ok) return acc;

  const auto dec = write_and_expect(
      protocols::encode_brush_motor_deceleration(config_.station_id,
                                                 config_.profile,
                                                 deceleration_raw),
      config_.profile.deceleration_register,
      deceleration_raw);
  if (!dec.ok) return dec;

  const auto run_time = write_and_expect(
      protocols::encode_brush_motor_run_time(config_.station_id,
                                             config_.profile,
                                             run_time_20ms_units),
      config_.profile.run_time_register,
      run_time_20ms_units);
  if (!run_time.ok) return run_time;

  const auto delay = write_and_expect(
      protocols::encode_brush_motor_direction_switch_delay(
          config_.station_id, config_.profile, direction_delay_20ms_units),
      config_.profile.direction_switch_delay_register,
      direction_delay_20ms_units);
  if (!delay.ok) return delay;

  return write_and_expect(
      protocols::encode_brush_motor_max_input_power(config_.station_id,
                                                    config_.profile,
                                                    max_input_power_w),
      config_.profile.max_input_power_register,
      max_input_power_w);
}

Result BrushMotor::set_lock_after_stop(bool locked) {
  return write_and_expect(
      protocols::encode_brush_motor_lock_after_stop(config_.station_id,
                                                    config_.profile,
                                                    locked),
      config_.profile.lock_after_stop_register,
      locked ? 1U : 0U);
}

Result BrushMotor::clear_history_fault() {
  return write_and_expect(
      protocols::encode_brush_motor_clear_history_fault(config_.station_id,
                                                        config_.profile),
      config_.profile.control_register,
      0x0002);
}

BrushMotorFault BrushMotor::read_fault() {
  Rs485Request request;
  request.logical_device = config_.device_id;
  request.payload =
      protocols::encode_brush_motor_read_fault(config_.station_id,
                                               config_.profile);

  const auto response = bus_.exchange(request);
  if (!response.result.ok) {
    return {response.result, std::nullopt, std::nullopt};
  }

  const auto decoded = protocols::decode_brush_motor_fault_response(
      config_.station_id,
      config_.profile,
      response.payload);

  return {decoded.result, decoded.raw_fault, decoded.fault_code};
}

void BrushMotor::safe_stop() noexcept {
  static_cast<void>(stop_motion());
}

void BrushMotor::stop() noexcept {
  safe_stop();
  health_ = DriverHealth::kStopped;
}

}  // namespace snappy::drivers::devices