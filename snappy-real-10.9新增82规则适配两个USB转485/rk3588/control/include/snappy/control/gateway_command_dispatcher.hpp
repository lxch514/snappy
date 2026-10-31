#pragma once

/**
 * @file gateway_command_dispatcher.hpp
 * @brief 整机网关命令调度声明
 *
 * @details
 * - 接收 G20 / Android 上位机发来的控制命令。
 * - 管理 acquire、arm、disarm、心跳、安全停止和故障清除。
 * - acquire 只取得控制权，不能直接执行非停止动作。
 * - 执行器命令统一转发给 ActuatorService。
 * - status 和 telemetry 只读取 SensorService 缓存快照。
 * - dispatcher 状态锁不覆盖 CAN、RS485 或执行器硬件 I/O。
 */

#include "snappy/control/actuator_service.hpp"
#include "snappy/control/control_application.hpp"
#include "snappy/control/device_registry.hpp"
#include "snappy/control/hardware_config.hpp"
#include "snappy/control/lease_manager.hpp"
#include "snappy/control/model.hpp"
#include "snappy/control/safety_manager.hpp"
#include "snappy/control/watchdog.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace snappy::control {

class GatewayCommandDispatcher {
 public:
  /**
   * @brief 创建网关命令调度器
   *
   * @param registry 设备注册表
   * @param robot_id 机器人唯一标识
   * @param safety_config 安全策略配置
   * @param g20_link G20 通信配置
   * @param video_config 视频配置
   */
  GatewayCommandDispatcher(
      DeviceRegistry& registry,
      std::string robot_id,
      SafetyConfig safety_config,
      G20LinkConfig g20_link,
      VideoConfig video_config);

  GatewayCommandDispatcher(
      const GatewayCommandDispatcher&) = delete;

  GatewayCommandDispatcher& operator=(
      const GatewayCommandDispatcher&) = delete;

  /**
   * @brief 启动整机控制链路
   *
   * 调试模式允许单个硬件后端启动失败后继续运行。
   */
  drivers::Result start();

  /**
   * @brief 执行周期安全检查和执行器超时处理
   *
   * 本函数只读取缓存快照，不同步轮询传感器。
   */
  drivers::Result tick(
      std::chrono::steady_clock::time_point now);

  /**
   * @brief 处理 gateway 发送的 JSON 请求
   *
   * @return JSON 格式的 ACK、状态、遥测或能力信息
   */
  std::string handle_request(
      const std::string& request_json);

  /**
   * @brief 停止调度器和全部硬件服务
   */
  void stop() noexcept;

  /**
   * @brief 更新硬件看门狗状态快照
   */
  void set_watchdog_status(
      WatchdogStatus status);

 private:
  /**
   * @brief 解析后的网关请求
   */
  struct ParsedRequest {
    std::string kind;
    std::string action;
    CommandMetadata metadata;
    std::string raw_json;
  };

  /**
   * @brief 解析 JSON 请求
   */
  ParsedRequest parse_request(
      const std::string& request_json) const;

  /**
   * @brief 处理 acquire、arm、disarm 和安全命令
   */
  std::string handle_control(
      const ParsedRequest& request);

  /**
   * @brief 处理执行器命令
   */
  std::string handle_actuator(
      const ParsedRequest& request);

  /**
   * @brief 在锁外执行实际执行器调用
   */
  drivers::Result execute_actuator(
      const ParsedRequest& request);

  /**
   * @brief 检查请求是否属于当前控制权持有者
   *
   * 调用前必须持有 mutex_。
   */
  [[nodiscard]] bool lease_matches_locked(
      const CommandMetadata& metadata) const;

  /**
   * @brief 检查普通执行器动作是否允许执行
   *
   * 普通动作必须处于 ARMED_IDLE 或 ACTIVE。
   * 调用前必须持有 mutex_。
   */
  [[nodiscard]] bool actuator_allowed_locked(
      const CommandMetadata& metadata,
      std::string* reason) const;

  /**
   * @brief 当前是否启用了调试模式部分启动
   *
   * 调用前必须持有 mutex_。
   */
  [[nodiscard]] bool partial_bringup_locked()
      const;

  /**
   * @brief 记录控制链路活动时间
   *
   * 调用前必须持有 mutex_。
   */
  void mark_link_activity_locked(
      const CommandMetadata& metadata,
      std::chrono::steady_clock::time_point now);

  /**
   * @brief 锁存安全停止状态
   *
   * 本函数只修改内存状态，不访问硬件。
   * 调用前必须持有 mutex_。
   */
  void enter_safe_hold_locked(
      const std::string& reason);

  /**
   * @brief 锁存软件急停状态
   *
   * 本函数只修改内存状态，不访问硬件。
   * 调用前必须持有 mutex_。
   */
  void enter_estop_locked(
      const std::string& reason);

  /**
   * @brief 撤销当前控制权
   *
   * 调用前必须持有 mutex_。
   */
  void revoke_lease_locked(
      const std::string& reason);

  /**
   * @brief 在状态锁外执行实际硬件安全停止
   *
   * 该函数可能访问 CAN、RS485、继电器和执行器。
   * 调用本函数时禁止持有 mutex_。
   */
  void apply_safe_stop_unlocked(
      const std::string& reason) noexcept;

  /**
   * @brief 生成命令 ACK
   *
   * 调用前必须持有 mutex_。
   */
  [[nodiscard]] std::string make_ack_locked(
      const CommandMetadata& metadata,
      AckStatus status,
      const std::string& detail) const;

  /**
   * @brief 生成整机状态缓存快照
   */
  [[nodiscard]] std::string make_status();

  /**
   * @brief 生成传感器遥测缓存快照
   */
  [[nodiscard]] std::string make_telemetry();

  /**
   * @brief 生成整机能力说明
   */
  [[nodiscard]] std::string make_capabilities()
      const;

  /**
   * @brief 清除可恢复执行器故障
   *
   * 调用本函数时禁止持有 mutex_。
   */
  drivers::Result clear_recoverable_faults();

  /* ==========================================================
   * 服务和配置对象
   * ========================================================== */

  DeviceRegistry& registry_;

  std::unique_ptr<ControlApplication>
      control_;

  LeaseManager lease_;
  SafetyManager safety_;

  std::string robot_id_;

  SafetyConfig safety_config_;
  G20LinkConfig g20_link_;
  VideoConfig video_config_;

  /* ==========================================================
   * Dispatcher 状态锁
   *
   * 只保护下面的状态、租约和 watchdog 快照。
   * 禁止在持有该锁时访问硬件或 ActuatorService。
   * ========================================================== */

  mutable std::mutex mutex_;

  ControlState state_{
      ControlState::kBoot};

  std::string state_detail_{
      "booting"};

  /**
   * ControlApplication 是否已经成功启动。
   */
  bool control_started_{false};

  /**
   * 启动阶段是否允许执行 control.arm。
   *
   * 正式模式仍需在 arm 和每次动作前检查实时安全快照。
   */
  bool ready_for_arm_{false};

  /**
   * 是否存在部分设备启动失败。
   */
  bool partial_start_{false};

  /* ==========================================================
   * 当前控制权所有者
   * ========================================================== */

  std::string lease_source_id_;
  std::string lease_session_id_;

  LinkId lease_link_id_{
      LinkId::kG20Udp};

  /* ==========================================================
   * 看门狗和链路状态快照
   * ========================================================== */

  WatchdogStatus watchdog_status_{};

  std::chrono::steady_clock::time_point
      last_g20_seen_{};

  /* ==========================================================
   * 命令失效代数
   *
   * safe hold、disarm、release、estop 或故障清除时递增。
   * 已经在锁外执行的旧动作完成后必须重新核对此值，
   * 防止旧动作覆盖新的安全停止状态。
   * ========================================================== */

  std::uint64_t command_epoch_{0};
};

}  // namespace snappy::control