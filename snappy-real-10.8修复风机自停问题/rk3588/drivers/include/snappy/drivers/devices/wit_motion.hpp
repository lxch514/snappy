#pragma once
/**
 * @file wit_motion.hpp
 * @brief 维特智能姿态与定位传感器驱动声明
 *
 * @details
 * - 通过 RS485 Modbus 读取姿态角和 GPS 定位数据。
 * - 角度使用原始值与比例因子还原，避免浮点累积误差。
 */

#include <cstdint>
#include <optional>
#include <string>
#include <chrono>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/protocols/wit_motion_modbus.hpp"

namespace snappy::drivers::devices {

inline constexpr const char* kWitMotionDeviceId = "wit_motion";
inline constexpr std::uint8_t kWitMotionStationId = 0x65;

struct WitMotionConfig {
  std::string device_id{kWitMotionDeviceId};
  std::uint8_t station_id{kWitMotionStationId};
  protocols::WitMotionModbusProfile profile{};
};

struct ImuAttitudeSample {
  Result result;
  std::optional<double> roll_deg;
  std::optional<double> pitch_deg;
  std::optional<double> yaw_deg;
};

using ImuMotionSample = protocols::WitMotionSample;

[[nodiscard]] WitMotionConfig make_wit_motion_config();

class WitMotion {
 public:
  WitMotion(IRs485Bus& bus, WitMotionConfig config);

  Result start();
  void stop() noexcept;
  ImuMotionSample read_motion();
  ImuAttitudeSample read_attitude();

  [[nodiscard]] DriverHealth health() const noexcept;

 private:
  IRs485Bus& bus_;
  WitMotionConfig config_;
  DriverHealth health_{DriverHealth::kUnconfigured};
};

}  // namespace snappy::drivers::devices