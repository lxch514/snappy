#include "snappy/control/sensor_runtime.hpp"
/**
 * @file sensor_runtime.cpp
 * @brief 传感器运行时装配实现
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include "snappy/drivers/diagnostics/driver_log.hpp"
#include "snappy/drivers/devices/anemometer.hpp"
#include "snappy/drivers/devices/lidar.hpp"
#include "snappy/drivers/devices/power.hpp"
#include "snappy/drivers/devices/temperature.hpp"
#include "snappy/drivers/devices/wit_motion.hpp"

namespace snappy::control {
namespace {

const DeviceConfig* find_enabled_sensor(
    const HardwareConfig& config,
    const std::string& id) {
  const auto it = config.devices.find(id);

  if (it == config.devices.end() ||
      !it->second.enabled) {
    return nullptr;
  }

  return &it->second;
}

template <typename Config>
void apply_station_id(
    Config& target,
    const DeviceConfig& source) {
  if (source.station_id) {
    target.station_id = *source.station_id;
  }

  target.device_id = source.device_id;
}

bool is_sensor_id(
    const std::string& id) {
  return id == "lidar_brush_left" ||
         id == "lidar_brush_right" ||
         id == "lidar_body_front" ||
         id == "lidar_body_rear" ||
         id == "anemometer_left" ||
         id == "anemometer_right" ||
         id == "wit_motion" ||
         id == "temperature_cabinet" ||
         id == "power_24v" ||
         id == "power_48v";
}

std::string expected_port_for(
    const std::string& id) {
  if (id == "lidar_brush_left" ||
      id == "lidar_brush_right" ||
      id == "lidar_body_front" ||
      id == "lidar_body_rear") {
    return "port_usb_rs485_0";
  }

  if (id == "anemometer_left" ||
      id == "anemometer_right" ||
      id == "wit_motion" ||
      id == "power_24v") {
    return "port_usb_rs485_1";
  }

  return "port_rs485_3";
}

std::size_t max_response_bytes(
    const DeviceConfig& device) {
  if (device.protocol_profile == "wit_motion") {
    return 128;
  }

  if (device.protocol_profile == "pds3000") {
    return 96;
  }

  return 64;
}

SensorPollConfig make_poll_config(
    const HardwareConfig& config,
    const std::string& id) {
  SensorPollConfig poll;

  const auto it = config.devices.find(id);

  if (it != config.devices.end()) {
    poll.enabled =
        it->second.enabled;

    poll.bus_id =
        it->second.bus_id;

    poll.poll_interval =
        it->second.poll_interval;

    poll.stale_after =
        it->second.stale_after;

    poll.max_consecutive_failures =
        it->second.max_consecutive_failures;
  }

  return poll;
}

drivers::Result bind_sensor(
    const HardwareConfig& config,
    snappy_snappy::DirectRs485Bus& bus,
    const std::string& id) {
  const auto* device =
      find_enabled_sensor(config, id);

  if (device == nullptr) {
    return drivers::Result::success();
  }

  if (device->bus_id.empty() ||
      config.rs485_ports.find(device->bus_id) ==
          config.rs485_ports.end()) {
    return drivers::Result::failure(
        drivers::ErrorCode::kConfigRequired,
        "sensor references missing RS485 port: " + id,
        "configure",
        id);
  }

  const auto expected =
      expected_port_for(id);

  if (device->bus_id != expected) {
    SNAPPY_DRIVER_LOG_WARN(
        "snappy-real",
        "sensor bus differs from expected physical mapping: " +
            id +
            " configured=" +
            device->bus_id +
            " expected=" +
            expected);
  }

  snappy::drivers::bus::Rs485DeviceBinding binding;

  binding.logical_device =
      device->device_id;

  binding.port_id =
      device->bus_id;

  binding.max_response_bytes =
      max_response_bytes(*device);

  binding.timeout =
      device->response_deadline;

  binding.max_consecutive_failures =
      device->max_consecutive_failures;

  binding.quarantine_duration =
      device->quarantine_duration;

  binding.read_retry_count =
      device->read_retry_count;

  /*
   * 传感器优先级低于执行器。
   *
   * 执行器：
   *   priority ~= 5
   *
   * 传感器：
   *   priority >= 20
   *
   * 这样后台传感器轮询不会抢占执行器控制事务。
   */
  binding.priority =
      std::max(20, device->priority);

  const auto result =
      bus.add_binding(binding);

  /*
   * 重要：
   *
   * snappy-real 的统一 RS485 运行时可能已经根据
   * HardwareConfig 注册过该 logical_device。
   *
   * 这时再次从 SensorRuntime 注册同一个传感器，会得到：
   *
   *   duplicate RS485 device binding
   *
   * 这个 duplicate 并不是通信故障，也不是设备离线。
   *
   * 只要 logical_device 已经存在，传感器驱动可以直接复用
   * 已有的 DirectRs485Bus binding。
   *
   * 因此这里只对“duplicate binding”进行特殊处理。
   * 其他错误仍然必须原样返回，不能吞掉。
   */
  if (!result.ok &&
      result.detail == "duplicate RS485 device binding") {
    SNAPPY_DRIVER_LOG_INFO(
        "snappy-real",
        "sensor RS485 binding already exists, reusing: " +
            id);

    return drivers::Result::success(
        "sensor RS485 binding already exists");
  }

  return result;
}

}  // namespace

SensorRuntime::SensorRuntime(
    const HardwareConfig& config,
    snappy_snappy::DirectRs485Bus& bus)
    : config_(config),
      bus_(bus) {}

SensorRuntime::~SensorRuntime() {
  stop();
}

drivers::Result SensorRuntime::configure() {
  if (configured_) {
    return drivers::Result::success(
        "sensor runtime already configured");
  }

  /*
   * 这里只处理 10 个传感器。
   *
   * 执行器：
   *   brush_motor
   *   relay_push_motor
   *   climber
   *   fan
   *
   * 仍然由 ActuatorHub / ActuatorService 管理。
   */

  const char* ids[] = {
      "lidar_brush_left",
      "lidar_brush_right",
      "lidar_body_front",
      "lidar_body_rear",
      "anemometer_left",
      "anemometer_right",
      "wit_motion",
      "temperature_cabinet",
      "power_24v",
      "power_48v"};

  /*
   * 先确保 10 个传感器的 RS485 logical binding 存在。
   *
   * 如果已经存在，则复用已有 binding。
   *
   * 这样不会因为统一 RS485 初始化已经完成注册，
   * 导致整个 snappy-real-control 启动失败。
   */
  for (const char* id : ids) {
    const auto result =
        bind_sensor(config_, bus_, id);

    if (!result.ok) {
      return result;
    }
  }

  /*
   * 创建传感器驱动对象。
   *
   * 注意：
   * 这里只创建驱动对象，不执行物理串口访问。
   *
   * 真正的轮询由 SensorService 后台线程负责。
   */

  if (const auto* d = find_enabled_sensor(config_, "lidar_brush_left")) {
    auto c =
        drivers::devices::
            make_lidar_brush_left_config();

    apply_station_id(c, *d);

    lidar_brush_left_ =
        std::make_unique<
            drivers::devices::DistanceSensor>(bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "lidar_brush_right")) {
    auto c =
        drivers::devices::
            make_lidar_brush_right_config();

    apply_station_id(c, *d);

    lidar_brush_right_ =
        std::make_unique<
            drivers::devices::DistanceSensor>(
            bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "lidar_body_front")) {
    auto c =
        drivers::devices::
            make_lidar_body_front_config();

    apply_station_id(c, *d);

    lidar_body_front_ =
        std::make_unique<
            drivers::devices::DistanceSensor>(
            bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "lidar_body_rear")) {
    auto c =
        drivers::devices::
            make_lidar_body_rear_config();

    apply_station_id(c, *d);

    lidar_body_rear_ =
        std::make_unique<
            drivers::devices::DistanceSensor>(bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "anemometer_left")) {
    auto c =
        drivers::devices::
            make_anemometer_left_config();

    apply_station_id(c, *d);

    anemometer_left_ =
        std::make_unique<
            drivers::devices::Anemometer>(bus_, c);
  }

  if (const auto* d =
          find_enabled_sensor(
              config_,
              "anemometer_right")) {
    auto c =
        drivers::devices::
            make_anemometer_right_config();

    apply_station_id(c, *d);

    anemometer_right_ =
        std::make_unique<
            drivers::devices::Anemometer>(bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "wit_motion")) {
    auto c =
        drivers::devices::
            make_wit_motion_config();

    apply_station_id(c, *d);

    wit_motion_ =
        std::make_unique<
            drivers::devices::WitMotion>(bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "temperature_cabinet")) {
    auto c =
        drivers::devices::
            make_temperature_cabinet_config();

    apply_station_id(c, *d);

    temperature_cabinet_ =
        std::make_unique<
            drivers::devices::TemperatureSensor>(bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "power_24v")) {
    auto c =
        drivers::devices::
            make_power_24v_config();

    apply_station_id(c, *d);

    power_24v_ =
        std::make_unique<
            drivers::devices::PowerMonitor>(bus_, c);
  }

  if (const auto* d = find_enabled_sensor(config_, "power_48v")) {
    auto c =
        drivers::devices::
            make_power_48v_config();

    apply_station_id(c, *d);

    power_48v_ =
        std::make_unique<
            drivers::devices::PowerMonitor>(bus_, c);
  }

  /*
   * 将驱动对象交给 SensorService。
   *
   * SensorService 只负责后台轮询和缓存，
   * 不负责重新创建 RS485 scheduler。
   */
  SensorServiceDevices devices;

  devices.lidar_brush_left = lidar_brush_left_.get();

  devices.lidar_brush_right = lidar_brush_right_.get();

  devices.lidar_body_front = lidar_body_front_.get();

  devices.lidar_body_rear = lidar_body_rear_.get();

  devices.anemometer_left = anemometer_left_.get();

  devices.anemometer_right = anemometer_right_.get();

  devices.wit_motion = wit_motion_.get();

  devices.temperature_cabinet = temperature_cabinet_.get();

  devices.power_24v = power_24v_.get();

  devices.power_48v = power_48v_.get();

  SensorServiceOptions options;

  options.ignore_startup_failures = config_.bringup.ignore_device_startup_failures;

  options.background_polling = true;

  options.scheduler_period = std::chrono::milliseconds(20);

  options.poll_period = std::chrono::milliseconds(100);

  options.ultrasonic_request_gap = std::chrono::milliseconds(100);

  /*
   * 仅给 10 个传感器建立 SensorService 的轮询配置。
   *
   * 执行器不进入这里。
   */
  for (const auto& [id, device] :
       config_.devices) {
    if (device.enabled &&
        is_sensor_id(id)) {
      options.device_polls[id] = make_poll_config(config_, id);
    }
  }

  /*
   * SensorService 必须在传感器驱动对象创建完成后建立。
   */
  sensor_service_ =
      std::make_unique<SensorService>(
          devices,
          config_.safety.sensor_stale_timeout,
          std::move(options));

  configured_ = true;

  return drivers::Result::success("sensor runtime configured");
}

drivers::Result SensorRuntime::start() {
  if (started_) {
    return drivers::Result::success("sensor runtime already started");
  }

  if (!configured_) {
    const auto result = configure();

    if (!result.ok) {
      return result;
    }
  }

  const auto result = sensor_service_->start();

  /*
   * 传感器允许部分失败。
   *
   * 某一个传感器不存在、启动通信失败或者后续被隔离，
   * 不应该让 4 个执行器和整个控制进程退出。
   */
  if (!result.ok && !config_.bringup.allow_partial_devices) {
    return result;
  }

  started_ = true;

  if (result.ok) {
    return result;
  }

  return drivers::Result::success("sensor runtime started with isolated failures: " + result.detail);
}

void SensorRuntime::stop() noexcept {
  if (sensor_service_) {
    sensor_service_->stop();
  }

  started_ = false;
}

}  // namespace snappy::control