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

inline constexpr const char* kPortCan0Id = "port_can_0";

inline constexpr const char* kPortCan0Interface = "can0";

inline constexpr std::size_t kClassicCanMaxPayloadBytes = 8;

/* ============================================================
 * CAN 总线配置
 * ============================================================ */

struct CanBusConfig {
  /*
   * 配置中的逻辑总线名称。
   */
  std::string bus_id{
      kPortCan0Id};
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
  std::uint32_t bitrate{1000000};

  /*
   * CAN 采样点，X6 Plus 按 DroneCAN 惯用值取 0.875。
   */
  double sample_point{0.875};

  /*
   * 重同步跳变宽度（个 tq），1 Mbps 下一帧只有 1 us，余量不足会反复进 ERROR-PASSIVE。
   */
  std::uint32_t sjw{4};

  std::uint32_t tx_queue_length{1000};

  bool auto_configure_interface{true};

  bool restart_on_bus_off{true};

  std::chrono::milliseconds restart_ms{100};

  bool receive_own_messages{false};

  bool allow_start_failure{false};

  /*
   * 单帧发送的最长等待，单位毫秒。
   * ESC 遥测占满总线时 20 ms 往往排不上队，放宽到 120 ms。
   */
  std::chrono::milliseconds
      default_tx_timeout{120};

  std::chrono::milliseconds
      default_rx_timeout{20};
};

/* ============================================================
 * CAN 接收过滤器
 * ============================================================ */

struct CanReceiveFilter {
  std::uint32_t id{0};

  std::uint32_t mask{0x1FFFFFFF};

  bool extended{false};
};

struct CanReceiveOptions {
  std::vector<CanReceiveFilter>
      filters;

  std::chrono::milliseconds timeout{20};
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

  virtual CanReceiveResult receive(const CanReceiveOptions& options) = 0;

  virtual void stop() noexcept = 0;
};

/* ============================================================
 * CAN 辅助函数
 * ============================================================ */

[[nodiscard]] bool can_frame_matches_filter(
    const CanFrame& frame,
    const CanReceiveFilter& filter) noexcept;

}  // namespace snappy::drivers::bus