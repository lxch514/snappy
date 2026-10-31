#pragma once

/**
 * @file sensor_runtime.hpp
 * @brief 传感器运行时装配声明
 *
 * @details
 * - 依据硬件配置创建直连 RS485 总线与传感器采集服务。
 * - 负责两者的启动顺序、停止顺序和句柄生命周期。
 */

#include <memory>
#include <string>

#include "snappy/control/hardware_config.hpp"
#include "snappy/control/sensor_service.hpp"
#include "snappy/drivers/result.hpp"
#include "snappy_snappy/direct_rs485_bus.hpp"

namespace snappy::control {

/**
 * @brief 只负责创建/绑定/启动 10 个传感器的运行时。
 *
 * 传感器与执行器共享同一个 DirectRs485Bus，因此：
 * - 每个物理 RS485 端口仍由唯一 PortWorker 访问；
 * - ttyACM0/1/2 仍由同一个 WCH 事务仲裁器协调；
 * - 单个传感器失败只记录到自己的绑定和 SensorService cache；
 * - 不创建 DeviceRegistry，不启动旧 ActuatorService 执行器链路。
 */
class SensorRuntime final {
 public:
  SensorRuntime(const HardwareConfig& config,
                snappy_snappy::DirectRs485Bus& bus);

  ~SensorRuntime();

  SensorRuntime(const SensorRuntime&) = delete;
  SensorRuntime& operator=(const SensorRuntime&) = delete;

  [[nodiscard]] drivers::Result configure();
  [[nodiscard]] drivers::Result start();
  void stop() noexcept;

  [[nodiscard]] SensorService& service() noexcept {
    return *sensor_service_;
  }

  [[nodiscard]] const SensorService& service() const noexcept {
    return *sensor_service_;
  }

 private:
  const HardwareConfig& config_;
  snappy_snappy::DirectRs485Bus& bus_;

  std::unique_ptr<drivers::devices::DistanceSensor> lidar_brush_left_;
  std::unique_ptr<drivers::devices::DistanceSensor> lidar_brush_right_;
  std::unique_ptr<drivers::devices::DistanceSensor> lidar_body_front_;
  std::unique_ptr<drivers::devices::DistanceSensor> lidar_body_rear_;

  std::unique_ptr<drivers::devices::Anemometer> anemometer_left_;
  std::unique_ptr<drivers::devices::Anemometer> anemometer_right_;
  std::unique_ptr<drivers::devices::WitMotion> wit_motion_;

  std::unique_ptr<drivers::devices::TemperatureSensor> temperature_cabinet_;
  std::unique_ptr<drivers::devices::PowerMonitor> power_24v_;
  std::unique_ptr<drivers::devices::PowerMonitor> power_48v_;

  std::unique_ptr<SensorService> sensor_service_;
  bool configured_{false};
  bool started_{false};
};

}  // namespace snappy::control
