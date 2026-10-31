#pragma once

/**
 * @file rs485_bus.hpp
 * @brief RS485 总线基础接口与事务配置
 *
 * @details
 * - 每个物理串口由一个独立 PortWorker 管理。
 * - 每个事务携带请求专属的完整帧成帧器。
 * - WCH ACM0~ACM3 使用共同的公平事务门禁。
 * - 迟到帧恢复静默期属于一次底层事务的一部分。
 * - 写操作不得由通信恢复逻辑自动重放。
 */

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::drivers::bus {

/* ============================================================
 * 默认端口名称
 * ============================================================ */

inline constexpr const char* kPortUsbRs4850Id =
    "port_usb_rs485_0";

inline constexpr const char* kPortUsbRs4850Path =
    "/dev/ttyACM0";

inline constexpr const char* kPortUsbRs4851Id =
    "port_usb_rs485_1";

inline constexpr const char* kPortUsbRs4851Path =
    "/dev/ttyACM1";

inline constexpr const char* kPortRs4850Id =
    "port_rs485_0";

inline constexpr const char* kPortRs4850Path =
    "/dev/ttyS0";

inline constexpr const char* kPortRs4853Id =
    "port_rs485_3";

inline constexpr const char* kPortRs4853Path =
    "/dev/ttyS3";

inline constexpr int kDefaultRs485BaudRate =
    9600;

inline constexpr std::size_t
    kDefaultRs485MaxResponseBytes = 256;

/* ============================================================
 * 串口参数
 * ============================================================ */

enum class Rs485Parity {
  kNone,
  kEven,
  kOdd,
};

enum class Rs485DirectionMode {
  /*
   * 不主动修改内核 RS485 方向控制。
   * 适用于 USB-RS485 自动方向模块。
   */
  kAuto,

  /*
   * 明确关闭内核 RS485 方向控制。
   */
  kNone,

  /*
   * RTS 高电平发送。
   */
  kKernelRtsHighOnSend,

  /*
   * RTS 低电平发送。
   */
  kKernelRtsLowOnSend,
};

struct Rs485SerialFormat {
  int data_bits{8};

  Rs485Parity parity{
      Rs485Parity::kNone};

  int stop_bits{1};
};

/* ============================================================
 * 物理端口配置
 * ============================================================ */

struct Rs485PortConfig {
  std::string port_id;

  /*
   * 当前真实串口后端使用 serial。
   * real 保留为兼容名称。
   */
  std::string backend{"serial"};

  std::string device_path;

  int baud_rate{
      kDefaultRs485BaudRate};

  Rs485SerialFormat format{};

  /*
   * 相邻事务之间的最小静默时间。
   */
  std::chrono::milliseconds
      inter_frame_gap{4};

  /*
   * 单次 poll 的空闲等待时间。
   * 它不能单独作为完整帧判断依据。
   */
  std::chrono::milliseconds
      read_idle_gap{4};

  /*
   * 事务失败后需要观察到的连续静默时间。
   */
  std::chrono::milliseconds
      late_frame_silence{8};

  /*
   * 迟到帧恢复过程的最长占用时间。
   */
  std::chrono::milliseconds
      late_frame_recovery_timeout{80};

  bool low_latency{true};

  /*
   * 旧版本兼容字段。
   */
  std::string shared_lock_group;

  /*
   * 跨端口完整事务门禁名称。
   */
  std::string shared_transaction_group;

  Rs485DirectionMode direction_mode{
      Rs485DirectionMode::kAuto};

  /*
   * 单端口待处理任务上限。
   */
  std::size_t max_pending_transactions{32};

  /*
   * 端口重新打开前的冷却时间。
   */
  std::chrono::milliseconds
      recovery_delay{50};
};

/* ============================================================
 * 逻辑设备绑定
 * ============================================================ */

struct Rs485DeviceBinding {
  std::string logical_device;
  std::string port_id;

  std::size_t max_response_bytes{
      kDefaultRs485MaxResponseBytes};

  std::chrono::milliseconds timeout{100};

  int max_consecutive_failures{3};

  std::chrono::milliseconds
      quarantine_duration{2000};

  /*
   * 仅无副作用读取可以重试。
   */
  int read_retry_count{1};

  /*
   * 数值越小，优先级越高。
   */
  int priority{10};
};

/* ============================================================
 * 单次物理事务
 * ============================================================ */

struct Rs485Transaction {
  std::string port_id;
  std::string logical_device;

  std::vector<std::uint8_t> request;

  std::size_t max_response_bytes{
      kDefaultRs485MaxResponseBytes};

  std::chrono::milliseconds timeout{100};

  std::chrono::steady_clock::time_point
      deadline{};

  int priority{10};

  /*
   * 生产事务必须为 true。
   */
  bool require_complete_frame{true};

  /*
   * 必须绑定当前请求的站号、功能码和长度。
   */
  Rs485ResponseFramer response_framer{};

  /*
   * 只有无副作用读取可以自动重试。
   */
  bool side_effect_free_read{false};

  int retry_count{0};
};

struct Rs485TransactionResult {
  Result result;
  std::vector<std::uint8_t> response;
};

/* ============================================================
 * 物理端口接口
 * ============================================================ */

class IRs485Port {
 public:
  virtual ~IRs485Port() = default;

  virtual Result open() = 0;

  virtual void close() noexcept = 0;

  virtual Rs485TransactionResult transact(
      const Rs485Transaction& transaction) = 0;

  [[nodiscard]] virtual const Rs485PortConfig&
  config() const noexcept = 0;

  [[nodiscard]] virtual DriverHealth health()
      const noexcept = 0;
};

[[nodiscard]] bool valid_rs485_port_config(
    const Rs485PortConfig& config) noexcept;

}  // namespace snappy::drivers::bus