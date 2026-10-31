#include "snappy/control/control_application.hpp"

/**
 * @file control_application.cpp
 * @brief 整机硬件启动、故障隔离和反向退出实现
 *
 * @details
 *
 * - 所有后端都会获得一次独立启动机会。
 * - SensorService 必须先于底层 RS485 关闭，避免采集线程访问已停止端口。
 * - ControlApplication 不执行同步传感器读取。
 */

#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace snappy::control {
namespace {

/* ============================================================
 * 启动错误文本
 * ============================================================ */

std::string result_detail(
    const std::string& backend,
    const drivers::Result& result) {
  std::string detail =
      backend + " start failed";

  if (!result.detail.empty()) {
    detail += ": ";
    detail += result.detail;
  } else {
    detail += ": ";
    detail +=
        drivers::error_code_name(
            result.code);
  }

  return detail;
}

std::string join_failures(
    const std::vector<std::string>& failures) {
  std::ostringstream out;

  for (std::size_t index = 0;
       index < failures.size();
       ++index) {
    if (index != 0) {
      out << "; ";
    }

    out << failures[index];
  }

  return out.str();
}

}  // namespace

/* ============================================================
 * 构造
 * ============================================================ */

ControlApplication::ControlApplication(
    HardwarePorts hardware,
    ControlApplicationOptions options)
    : hardware_(hardware),
      options_(options) {}

/* ============================================================
 * 启动
 * ============================================================ */

StartupReport ControlApplication::start() {
  std::lock_guard<std::mutex> lifecycle_lock(
      lifecycle_mutex_);

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    if (started_) {
      return {
          true,
          state_ == ControlState::kDisarmed,
          false,
          "control application already started",
      };
    }

    state_ = ControlState::kSelfTest;
    last_reason_ =
        "starting hardware backends";
  }

  std::vector<std::string> failures;

  /*
   * 即使一个后端失败，也继续尝试启动后续后端。
   * 这样调试模式不会因为 CAN 失败而跳过全部 RS485 设备。
   */
  const auto can_result =
      hardware_.can.start();

  if (!can_result.ok) {
    failures.push_back(
        result_detail(
            "CAN",
            can_result));
  }

  const auto rs485_result =
      hardware_.rs485.start();

  if (!rs485_result.ok) {
    failures.push_back(
        result_detail(
            "RS485",
            rs485_result));
  }

  const auto relay_result =
      hardware_.relays.start();

  if (!relay_result.ok) {
    failures.push_back(
        result_detail(
            "relay",
            relay_result));
  }

  /*
   * SensorService 自身负责逐总线后台线程和单设备隔离。
   * start() 不会同步读取全部传感器。
   */
  const auto sensor_result =
      hardware_.sensors.start();

  if (!sensor_result.ok) {
    failures.push_back(
        result_detail(
            "sensor service",
            sensor_result));
  }

  const bool all_ready =
      all_hardware_ready();

  if (!all_ready &&
      failures.empty()) {
    failures.push_back(
        "one or more hardware backends did not become ready");
  }

  if ((!failures.empty() || !all_ready) &&
      !options_.allow_partial_devices) {
    return fail_start_locked(
        join_failures(failures));
  }

  if (!all_ready &&
      !any_hardware_ready()) {
    return fail_start_locked(
        "no hardware backend became ready in partial bringup mode");
  }

  const auto report =
      make_startup_report(failures);

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    started_ = report.started;
    state_ = ControlState::kDisarmed;
    last_reason_ = report.detail;
  }

  return report;
}

/* ============================================================
 * 启动结果
 * ============================================================ */

StartupReport
ControlApplication::make_startup_report(
    const std::vector<std::string>& failures) const {
  if (failures.empty() &&
      all_hardware_ready()) {
    return {
        true,
        true,
        false,
        "hardware interfaces ready; actuator business logic remains disarmed",
    };
  }

  std::string detail =
      "partial hardware ready; unavailable backends remain isolated";

  if (!failures.empty()) {
    detail += ": ";
    detail += join_failures(failures);
  }

  return {
      true,
      !options_
           .arm_requires_all_required_devices,
      true,
      std::move(detail),
  };
}

/* ============================================================
 * 安全停止
 * ============================================================ */

void ControlApplication::safe_stop_outputs_locked()
    noexcept {
  /*
   * 先断开继电器输出，再停止总线执行器。
   *
   * RS485 scheduler 的 safe_stop() 不关闭采集 worker，
   * 实际 RS485 执行器停止由 ActuatorService/DeviceRegistry 下发。
   */
  hardware_.relays.deenergize_all();
  hardware_.can.safe_stop();
  hardware_.rs485.safe_stop();
}

void ControlApplication::enter_safe_hold(
    std::string reason) noexcept {
  std::lock_guard<std::mutex> lifecycle_lock(
      lifecycle_mutex_);

  safe_stop_outputs_locked();

  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  state_ = ControlState::kSafeHold;
  last_reason_ = std::move(reason);
}

/* ============================================================
 * 启动失败清理
 *
 * 调用时已经持有 lifecycle_mutex_。
 * ============================================================ */

StartupReport ControlApplication::fail_start_locked(
    std::string reason) noexcept {
  /*
   * SensorService 可能已经创建采集线程。
   * 必须先停止并 join，再处理底层总线。
   */
  hardware_.sensors.stop();
  safe_stop_outputs_locked();

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    started_ = false;
    state_ = ControlState::kSafeHold;
    last_reason_ = std::move(reason);
  }

  return {
      false,
      false,
      false,
      last_reason(),
  };
}

/* ============================================================
 * 正常退出
 *
 * 调用时已经持有 lifecycle_mutex_。
 * ============================================================ */

void ControlApplication::stop_backends_locked()
    noexcept {
  /*
   * 与启动顺序相反：
   * 1. 停止传感器后台线程；
   * 2. 断开继电器；
   * 3. 停止 RS485 输出；
   * 4. 停止 CAN 输出。
   *
   * 物理端口关闭由 DeviceRegistry::stop() 完成。
   */
  hardware_.sensors.stop();
  hardware_.relays.deenergize_all();
  hardware_.rs485.safe_stop();
  hardware_.can.safe_stop();
}

void ControlApplication::stop() noexcept {
  std::lock_guard<std::mutex> lifecycle_lock(
      lifecycle_mutex_);

  bool should_stop = false;

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    should_stop =
        started_ ||
        state_ == ControlState::kSelfTest ||
        state_ == ControlState::kSafeHold;

    started_ = false;
  }

  if (should_stop) {
    stop_backends_locked();
  }

  {
    std::lock_guard<std::mutex> state_lock(
        state_mutex_);

    state_ = ControlState::kDisarmed;
    last_reason_ =
        "control application stopped";
  }
}

/* ============================================================
 * 状态快照
 * ============================================================ */

ControlState ControlApplication::state()
    const noexcept {
  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  return state_;
}

std::string ControlApplication::last_reason()
    const {
  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  return last_reason_;
}

bool ControlApplication::started()
    const noexcept {
  std::lock_guard<std::mutex> state_lock(
      state_mutex_);

  return started_;
}

/* ============================================================
 * 后端健康判断
 * ============================================================ */

bool ControlApplication::all_hardware_ready()
    const noexcept {
  return
      hardware_.can.health() ==
          drivers::DriverHealth::kReady &&
      hardware_.rs485.health() ==
          drivers::DriverHealth::kReady &&
      hardware_.relays.health() ==
          drivers::DriverHealth::kReady &&
      hardware_.sensors.health() ==
          drivers::DriverHealth::kReady;
}

bool ControlApplication::any_hardware_ready()
    const noexcept {
  return
      hardware_.can.health() ==
          drivers::DriverHealth::kReady ||
      hardware_.rs485.health() ==
          drivers::DriverHealth::kReady ||
      hardware_.relays.health() ==
          drivers::DriverHealth::kReady ||
      hardware_.sensors.health() ==
          drivers::DriverHealth::kReady;
}

}  // namespace snappy::control