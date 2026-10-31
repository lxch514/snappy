#pragma once

/**
 * @file control_application.hpp
 * @brief 整机硬件启动、安全停止和退出顺序编排
 *
 * @details
 *
 * - 启动顺序：CAN -> RS485 -> 继电器 -> SensorService。
 * - 停止顺序：SensorService -> 继电器 -> RS485 -> CAN。
 * - started 表示控制进程可以继续运行。
 * - ready_for_arm 只表示启动阶段门禁，运行时门禁由 dispatcher 检查。
 */

#include <mutex>
#include <string>
#include <vector>

#include "snappy/control/model.hpp"
#include "snappy/drivers/interfaces.hpp"

namespace snappy::control {

/* ============================================================
 * ControlApplication 管理的硬件接口
 * ============================================================ */

struct HardwarePorts {
  drivers::ICanBus& can;
  drivers::IRs485Bus& rs485;
  drivers::IRelayBank& relays;
  drivers::ISensorSuite& sensors;
};

/* ============================================================
 * 启动模式
 * ============================================================ */

struct ControlApplicationOptions {
  /*
   * true：
   * 单个后端失败时继续启动其他后端。
   *
   * false：
   * 仍会完成全部后端的启动检查，但最终启动结果为失败。
   */
  bool allow_partial_devices{false};

  /*
   * true：
   * 所有必要硬件都 Ready 才允许进入 arm。
   *
   * false：
   * 调试模式中可用设备允许独立工作。
   */
  bool arm_requires_all_required_devices{true};
};

/* ============================================================
 * 启动结果
 * ============================================================ */

struct StartupReport {
  bool started{false};
  bool ready_for_arm{false};
  bool partial_start{false};
  std::string detail;
};

/* ============================================================
 * 整机硬件生命周期编排
 * ============================================================ */

class ControlApplication {
 public:
  explicit ControlApplication(
      HardwarePorts hardware,
      ControlApplicationOptions options = {});

  /*
   * 启动所有硬件后端。
   *
   * 调试隔离模式下不会在第一个失败设备处提前返回。
   */
  StartupReport start();

  /*
   * 立即让所有输出进入安全状态。
   *
   * SensorService 仍可继续采集，只有 stop() 才停止采集线程。
   */
  void enter_safe_hold(
      std::string reason) noexcept;

  /*
   * 停止后台采集并按反向顺序停止硬件。
   */
  void stop() noexcept;

  [[nodiscard]] ControlState state() const noexcept;
  [[nodiscard]] std::string last_reason() const;
  [[nodiscard]] bool started() const noexcept;

 private:
  [[nodiscard]] bool all_hardware_ready() const noexcept;
  [[nodiscard]] bool any_hardware_ready() const noexcept;

  [[nodiscard]] StartupReport make_startup_report(
      const std::vector<std::string>& failures) const;

  StartupReport fail_start_locked(
      std::string reason) noexcept;

  void safe_stop_outputs_locked() noexcept;
  void stop_backends_locked() noexcept;

  HardwarePorts hardware_;
  ControlApplicationOptions options_;

  /*
   * 串行化 start、safe-hold 和 stop。
   * 这把锁不属于 dispatcher，也不影响网关状态快照。
   */
  mutable std::mutex lifecycle_mutex_;

  /*
   * 只保护 ControlApplication 自身的内存状态。
   */
  mutable std::mutex state_mutex_;

  ControlState state_{ControlState::kBoot};
  std::string last_reason_{"booting"};
  bool started_{false};
};

}  // namespace snappy::control