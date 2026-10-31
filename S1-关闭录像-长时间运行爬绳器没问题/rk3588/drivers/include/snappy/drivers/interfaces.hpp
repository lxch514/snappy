#pragma once

/**
 * @file interfaces.hpp
 * @brief 驱动层通用接口
 *
 * @details
 * - Rs485ResponseFramer 是通用增量成帧回调。
 * - 总线层只调用成帧器，不直接依赖某个设备 decoder。
 * - 成帧成功后只返回当前事务匹配的完整帧。
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "snappy/drivers/result.hpp"

namespace snappy::drivers {

enum class DriverHealth {
  kUnconfigured,
  kStopped,
  kReady,
  kFault,
};

struct CanFrame {
  std::uint32_t id{0};
  bool extended{false};
  std::vector<std::uint8_t> payload;
};

/* ============================================================
 * RS485 通用增量成帧接口
 * ============================================================ */

enum class Rs485FrameScanState {
  kNeedMore,
  kMatched,
  kDiscardPrefix,
  kProtocolError,
};

struct Rs485FrameScanResult {
  Rs485FrameScanState state{
      Rs485FrameScanState::kNeedMore};

  /*
   * kMatched / kProtocolError:
   *   当前匹配帧在 buffer 中的起始偏移。
   *
   * kDiscardPrefix:
   *   需要从 buffer 前端丢弃的字节数。
   */
  std::size_t frame_offset{0};

  /*
   * kMatched / kProtocolError:
   *   完整帧长度。
   */
  std::size_t frame_size{0};

  Result result{
      Result::success()};
};

using Rs485ResponseFramer =
    std::function<Rs485FrameScanResult(
        const std::vector<std::uint8_t>& buffer)>;

struct Rs485Request {
  std::string logical_device;
  std::vector<std::uint8_t> payload;

  /*
   * 非空时由调用方提供请求专属的响应成帧器。
   * 为空时，当前 scheduler 仅对可识别的 Modbus RTU 请求
   * 自动安装 Modbus 成帧器。
   */
  Rs485ResponseFramer response_framer{};

  /*
   * 只有明确无副作用的读取允许自动重试。
   * scheduler 还会识别标准 Modbus FC 01/02/03/04。
   */
  bool side_effect_free_read{false};

  /*
   * -1 表示采用设备绑定的默认读取重试次数。
   * 写请求会强制忽略该值，禁止自动重放。
   */
  int retry_count{-1};
};

struct Rs485Response {
  Result result;
  std::vector<std::uint8_t> payload;
};

enum class RelayState {
  kDeenergized,
  kEnergized,
};

enum class SampleQuality {
  kGood,
  kStale,
  kInvalid,
  kUnknown,
};

struct SensorSample {
  std::string logical_name;
  std::optional<double> value;
  std::string unit;
  SampleQuality quality{
      SampleQuality::kUnknown};

  std::uint64_t timestamp_ms{0};
  std::uint64_t age_ms{0};
};

class ICanBus {
 public:
  virtual ~ICanBus() = default;

  virtual Result start() = 0;
  virtual Result send(
      const CanFrame& frame) = 0;

  virtual void safe_stop() noexcept = 0;

  [[nodiscard]] virtual DriverHealth health()
      const noexcept = 0;
};

class IRs485Bus {
 public:
  virtual ~IRs485Bus() = default;

  virtual Result start() = 0;

  virtual Rs485Response exchange(
      const Rs485Request& request) = 0;

  virtual void safe_stop() noexcept = 0;

  [[nodiscard]] virtual DriverHealth health()
      const noexcept = 0;
};

class IRelayBank {
 public:
  virtual ~IRelayBank() = default;

  virtual Result start() = 0;

  virtual Result set(
      const std::string& logical_channel,
      RelayState state) = 0;

  virtual void deenergize_all() noexcept = 0;

  [[nodiscard]] virtual DriverHealth health()
      const noexcept = 0;
};

class ISensorSuite {
 public:
  virtual ~ISensorSuite() = default;

  virtual Result start() = 0;

  [[nodiscard]] virtual std::vector<SensorSample>
  read_all() = 0;

  virtual void stop() noexcept = 0;

  [[nodiscard]] virtual DriverHealth health()
      const noexcept = 0;
};

}  // namespace snappy::drivers