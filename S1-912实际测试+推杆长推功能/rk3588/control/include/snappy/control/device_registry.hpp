#pragma once

/**
 * @file device_registry.hpp
 * @brief 设备注册表声明
 *
 * @details
 * - 根据 hardware.snappy.json / hardware.debug.test.json 创建设备对象。
 * - 统一管理 CAN、RS485、继电器、传感器和执行器服务。
 * - 把配置里的轮询周期、响应超时、缓存失效时间和隔离参数传给底层。
 */

#include <memory>

#include "snappy/control/actuator_service.hpp"
#include "snappy/control/control_application.hpp"
#include "snappy/control/hardware_config.hpp"
#include "snappy/control/sensor_service.hpp"
#include "snappy/drivers/bus/can_socket_bus.hpp"
#include "snappy/drivers/bus/rs485_transaction_scheduler.hpp"
#include "snappy/drivers/devices/anemometer.hpp"
#include "snappy/drivers/devices/brush_motor.hpp"
#include "snappy/drivers/devices/climber.hpp"
#include "snappy/drivers/devices/lidar.hpp"
#include "snappy/drivers/devices/power.hpp"
#include "snappy/drivers/devices/propulsion_fan.hpp"
#include "snappy/drivers/devices/push_motor.hpp"
#include "snappy/drivers/devices/relay.hpp"
#include "snappy/drivers/devices/temperature.hpp"
#include "snappy/drivers/devices/wit_motion.hpp"
#include "snappy/drivers/interfaces.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::control {

class DeviceRegistry {
 public:
  explicit DeviceRegistry(HardwareConfig config);
  ~DeviceRegistry();

  DeviceRegistry(const DeviceRegistry&) = delete;
  DeviceRegistry& operator=(const DeviceRegistry&) = delete;

  [[nodiscard]] const HardwareConfig& config() const noexcept;

  drivers::Result configure();

  [[nodiscard]] HardwarePorts hardware_ports();

  [[nodiscard]] ActuatorService& actuators();
  [[nodiscard]] SensorService& sensors();

  void safe_stop() noexcept;
  void stop() noexcept;

 private:
  class RelayBankAdapter;

  [[nodiscard]] bool production_mode() const noexcept;
  [[nodiscard]] bool commissioning_noop_actuators() const noexcept;
  [[nodiscard]] bool commissioning_ignore_startup_failures() const noexcept;
  [[nodiscard]] bool commissioning_quarantine_devices() const noexcept;

  drivers::Result configure_buses();
  drivers::Result bind_rs485_devices();
  drivers::Result create_devices();
  drivers::Result create_services();

  HardwareConfig config_;
  bool configured_{false};

  std::unique_ptr<drivers::ICanBus> can_bus_;
  std::unique_ptr<drivers::IRs485Bus> rs485_bus_;
  std::unique_ptr<drivers::IRelayBank> relay_bank_;

  drivers::bus::Rs485TransactionScheduler* rs485_scheduler_{nullptr};

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

  std::unique_ptr<drivers::devices::RelayController> relay_push_motor_;
  std::unique_ptr<drivers::devices::BrushMotor> brush_motor_;
  std::unique_ptr<drivers::devices::Climber> climber_;
  std::unique_ptr<drivers::devices::PropulsionFanController> propulsion_fan_;

  std::unique_ptr<drivers::devices::PushMotor> brush_push_motor_;
  std::unique_ptr<drivers::devices::PushMotor> scraper_push_motor_;

  std::unique_ptr<ActuatorService> actuator_service_;
  std::unique_ptr<SensorService> sensor_service_;
};

}  // namespace snappy::control