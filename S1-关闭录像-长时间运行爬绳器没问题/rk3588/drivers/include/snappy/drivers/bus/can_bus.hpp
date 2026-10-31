#pragma once

/**
 * @file can_bus.hpp
 * @brief SocketCAN 总线配置和帧收发接口
 *
 * @details
 * - 推进风扇使用独立 can0，不与任何 RS485 串口共用锁。
 * - CAN 初始化失败时可以只隔离推进风扇，其他设备继续运行。
 * - allow_start_failure 只允许整机继续启动，不能伪造 Ready。
 * - backend 用于区分 socketcan、real 和调试替代后端。
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::drivers::bus {

inline constexpr const char* kPortCan0Id =
    "port_can_0";

inline constexpr const char* kPortCan0Interface =
    "can0";

inline constexpr std::size_t
    kClassicCanMaxPayloadBytes = 8;

/* ============================================================
 * CAN 总线配置
 * ============================================================ */

struct CanBusConfig {
  /*
   * 配置中的逻辑总线名称。
   */
  std::string bus_id{
      kPortCan0Id};

  /*
   * 后端名称。
   *
   * 当前真实后端使用：
   * - socketcan
   * - real
   *
   * 调试替代后端可以使用：
   * - fake
   * - noop
   */
  std::string backend{
      "socketcan"};

  /*
   * Linux SocketCAN 接口名称。
   */
  std::string interface_name{
      kPortCan0Interface};

  /*
   * Hobbywing X6 Plus 当前实机配置使用 1 Mbps。
   */
  std::uint32_t bitrate{
      1000000};

  /*
   * Hobbywing X6 Plus 实测可用 CAN 采样点。
   *
   * Linux 配置命令：
   * ip link set can0 type can bitrate 1000000 \
   *     sample-point 0.833
   *
   * RK3588 驱动实际显示为约 0.828，
   * 属于硬件时序量化后的结果。
   */
  double sample_point{
      0.833};

  /*
   * 对应：
   * ip link set can0 txqueuelen 1000
   */
  std::uint32_t tx_queue_length{
      1000};

  bool auto_configure_interface{
      true};

  bool restart_on_bus_off{
      true};

  std::chrono::milliseconds restart_ms{
      100};

  bool receive_own_messages{
      false};

  bool allow_start_failure{
      false};

  std::chrono::milliseconds
      default_tx_timeout{
          20};

  std::chrono::milliseconds
      default_rx_timeout{
          20};
};

/* ============================================================
 * CAN 接收过滤器
 * ============================================================ */

struct CanReceiveFilter {
  std::uint32_t id{
      0};

  std::uint32_t mask{
      0x1FFFFFFF};

  bool extended{
      false};
};

struct CanReceiveOptions {
  std::vector<CanReceiveFilter>
      filters;

  std::chrono::milliseconds timeout{
      20};
};

struct CanReceiveResult {
  Result result;
  CanFrame frame;
};

/* ============================================================
 * CAN 帧总线接口
 * ============================================================ */

class ICanFrameBus
    : public ICanBus {
 public:
  ~ICanFrameBus() override = default;

  virtual CanReceiveResult receive(
      const CanReceiveOptions& options) = 0;

  virtual void stop() noexcept = 0;
};

/* ============================================================
 * CAN 辅助函数
 * ============================================================ */

[[nodiscard]] bool can_frame_matches_filter(
    const CanFrame& frame,
    const CanReceiveFilter& filter) noexcept;

}  // namespace snappy::drivers::bus