#include "snappy/control/device_registry.hpp"

/**
 * @file device_registry.cpp
 * @brief 设备注册表实现
 *
 * @details
 * - 从硬件配置创建 CAN、RS485、继电器、传感器和执行器对象。
 * - RS485 每个物理串口拥有独立 worker。
 * - SensorService 只向上层提供缓存快照。
 * - 停机时先结束设备 worker，再关闭底层总线。
 */

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "snappy/control/actuator_service.hpp"
#include "snappy/control/sensor_service.hpp"

#ifndef SNAPPY_ENABLE_REAL_DRIVERS
#define SNAPPY_ENABLE_REAL_DRIVERS 0
#endif

namespace snappy::control {
namespace {

drivers::Result registry_error(
    const std::string& detail,
    const std::string& operation =
        "configure") {
  return drivers::Result::failure(
      drivers::ErrorCode::kConfigRequired,
      detail,
      operation,
      "device_registry");
}

const DeviceConfig* find_enabled_device(
    const HardwareConfig& config,
    const std::string& device_id) {
  const auto it =
      config.devices.find(device_id);

  if (it == config.devices.end() ||
      !it->second.enabled) {
    return nullptr;
  }

  return &it->second;
}

bool device_uses_rs485(
    const HardwareConfig& config,
    const DeviceConfig& device) {
  return config.rs485_ports.count(
             device.bus_id) != 0;
}

bool device_uses_can(
    const HardwareConfig& config,
    const DeviceConfig& device) {
  return config.can_buses.count(
             device.bus_id) != 0;
}

std::set<std::string> used_rs485_ports(
    const HardwareConfig& config) {
  std::set<std::string> out;

  for (const auto& item :
       config.devices) {
    const auto& device =
        item.second;

    if (device.enabled &&
        device_uses_rs485(
            config,
            device)) {
      out.insert(device.bus_id);
    }
  }

  return out;
}

std::set<std::string> used_can_buses(
    const HardwareConfig& config) {
  std::set<std::string> out;

  for (const auto& item :
       config.devices) {
    const auto& device =
        item.second;

    if (device.enabled &&
        device_uses_can(
            config,
            device)) {
      out.insert(device.bus_id);
    }
  }

  return out;
}

std::size_t max_response_bytes_for_device(
    const DeviceConfig& device) {
  if (device.protocol_profile ==
      "wit_motion") {
    return 128;
  }

  if (device.protocol_profile ==
      "pds3000") {
    return 96;
  }

  if (device.protocol_profile ==
      "l7p_speed_mode") {
    return 96;
  }

  return 64;
}

template <typename Config>
void apply_station_id(
    Config& target,
    const DeviceConfig& source) {
  if (source.station_id) {
    target.station_id =
        *source.station_id;
  }

  target.device_id =
      source.device_id;
}

std::optional<std::uint16_t>
relay_channel(
    const HardwareConfig& config,
    const std::string& primary,
    const std::string& fallback = {}) {
  const auto it =
      config.relay_outputs.find(
          primary);

  if (it !=
          config.relay_outputs.end() &&
      it->second.physical_channel != 0) {
    return it->second.physical_channel;
  }

  if (!fallback.empty()) {
    const auto fallback_it =
        config.relay_outputs.find(
            fallback);

    if (fallback_it !=
            config.relay_outputs.end() &&
        fallback_it->second
                .physical_channel != 0) {
      return fallback_it->second
          .physical_channel;
    }
  }

  return std::nullopt;
}

drivers::devices::DistanceSensorConfig
make_distance_config_from_device(
    const DeviceConfig& device) {
  if (device.device_id ==
      drivers::devices::
          kLidarBrushLeftDeviceId) {
    auto config =
        drivers::devices::
            make_lidar_brush_left_config();

    apply_station_id(config, device);
    return config;
  }

  if (device.device_id ==
      drivers::devices::
          kLidarBrushRightDeviceId) {
    auto config =
        drivers::devices::
            make_lidar_brush_right_config();

    apply_station_id(config, device);
    return config;
  }

  if (device.device_id ==
      drivers::devices::
          kLidarBodyFrontDeviceId) {
    auto config =
        drivers::devices::
            make_lidar_body_front_config();

    apply_station_id(config, device);
    return config;
  }

  auto config =
      drivers::devices::
          make_lidar_body_rear_config();

  apply_station_id(config, device);
  return config;
}

bool is_sensor_device_id(
    const std::string& device_id) {
  return device_id ==
             "lidar_brush_left" ||
         device_id ==
             "lidar_brush_right" ||
         device_id ==
             "lidar_body_front" ||
         device_id ==
             "lidar_body_rear" ||
         device_id ==
             "anemometer_left" ||
         device_id ==
             "anemometer_right" ||
         device_id ==
             "wit_motion" ||
         device_id ==
             "temperature_cabinet" ||
         device_id ==
             "power_24v" ||
         device_id ==
             "power_48v";
}

void add_sensor_poll_config(
    SensorServiceOptions& options,
    const HardwareConfig& config,
    const std::string& device_id) {
  const auto it =
      config.devices.find(
          device_id);

  if (it == config.devices.end() ||
      !it->second.enabled) {
    return;
  }

  SensorPollConfig poll;
  poll.enabled = true;
  poll.bus_id =
      it->second.bus_id;
  poll.poll_interval =
      it->second.poll_interval;
  poll.stale_after =
      it->second.stale_after;
  poll.max_consecutive_failures =
      it->second.max_consecutive_failures;

  options.device_polls[device_id] =
      poll;
}

/* ============================================================
 * 调试模式空总线
 * ============================================================ */

class NoopCanBus final
    : public drivers::ICanBus {
 public:
  drivers::Result start() override {
    health_ =
        drivers::DriverHealth::kReady;

    return drivers::Result::success(
        "noop can bus ready");
  }

  drivers::Result send(
      const drivers::CanFrame&) override {
    return drivers::Result::success(
        "noop can frame ignored");
  }

  void safe_stop() noexcept override {}

  drivers::DriverHealth health()
      const noexcept override {
    return health_;
  }

 private:
  drivers::DriverHealth health_{
      drivers::DriverHealth::kStopped};
};

class NoopRs485Bus final
    : public drivers::IRs485Bus {
 public:
  drivers::Result start() override {
    health_ =
        drivers::DriverHealth::kReady;

    return drivers::Result::success(
        "noop rs485 bus ready");
  }

  drivers::Rs485Response exchange(
      const drivers::Rs485Request&
          request) override {
    return {
        drivers::Result::failure(
            drivers::ErrorCode::
                kConfigRequired,
            "noop rs485 bus has no binding",
            "exchange",
            request.logical_device),
        {}};
  }

  void safe_stop() noexcept override {}

  drivers::DriverHealth health()
      const noexcept override {
    return health_;
  }

 private:
  drivers::DriverHealth health_{
      drivers::DriverHealth::kStopped};
};

class NoopRelayBankAdapter final
    : public drivers::IRelayBank {
 public:
  drivers::Result start() override {
    return drivers::Result::success(
        "noop relay bank ready");
  }

  drivers::Result set(
      const std::string& logical_channel,
      drivers::RelayState) override {
    return drivers::Result::success(
        "noop relay ignored: " +
        logical_channel);
  }

  void deenergize_all()
      noexcept override {}

  drivers::DriverHealth health()
      const noexcept override {
    return drivers::DriverHealth::kReady;
  }
};

}  // namespace

/* ============================================================
 * 继电器适配器
 * ============================================================ */

class DeviceRegistry::RelayBankAdapter final
    : public drivers::IRelayBank {
 public:
  RelayBankAdapter(
      drivers::devices::RelayController&
          controller,
      std::map<std::string,
               RelayOutputConfig> outputs,
      bool ignore_startup_failures)
      : controller_(controller),
        outputs_(std::move(outputs)),
        ignore_startup_failures_(
            ignore_startup_failures) {}

  drivers::Result start() override {
    const auto started =
        controller_.start();

    if (!started.ok) {
      if (ignore_startup_failures_) {
        return drivers::Result::success(
            "relay bank start failure ignored: " +
            started.detail);
      }

      return started;
    }

    const auto off =
        controller_.deenergize_all();

    if (!off.ok) {
      if (ignore_startup_failures_) {
        return drivers::Result::success(
            "relay bank deenergize failure ignored: " +
            off.detail);
      }

      return off;
    }

    return drivers::Result::success(
        "relay bank ready");
  }

  drivers::Result set(
      const std::string& logical_channel,
      drivers::RelayState state) override {
    const auto it =
        outputs_.find(logical_channel);

    if (it == outputs_.end() ||
        it->second.physical_channel == 0) {
      return drivers::Result::failure(
          drivers::ErrorCode::
              kInvalidArgument,
          "unknown relay logical channel",
          "set",
          logical_channel);
    }

    return controller_.set_channel(
        it->second.physical_channel,
        state ==
            drivers::RelayState::
                kEnergized);
  }

  void deenergize_all()
      noexcept override {
    static_cast<void>(
        controller_.deenergize_all());
  }

  drivers::DriverHealth health()
      const noexcept override {
    return controller_.health();
  }

 private:
  drivers::devices::RelayController&
      controller_;

  std::map<std::string,
           RelayOutputConfig> outputs_;

  bool ignore_startup_failures_{
      false};
};

/* ============================================================
 * 注册表生命周期
 * ============================================================ */

DeviceRegistry::DeviceRegistry(
    HardwareConfig config)
    : config_(std::move(config)) {}

DeviceRegistry::~DeviceRegistry() {
  stop();
}

const HardwareConfig&
DeviceRegistry::config() const noexcept {
  return config_;
}

bool DeviceRegistry::production_mode()
    const noexcept {
  return config_.bringup.mode ==
         RuntimeMode::kProduction;
}

bool DeviceRegistry::
commissioning_noop_actuators()
    const noexcept {
  return config_.bringup.mode ==
             RuntimeMode::kCommissioning &&
         config_.bringup
             .noop_missing_actuators;
}

bool DeviceRegistry::
commissioning_ignore_startup_failures()
    const noexcept {
  return config_.bringup.mode ==
             RuntimeMode::kCommissioning &&
         config_.bringup
             .ignore_device_startup_failures;
}

bool DeviceRegistry::
commissioning_quarantine_devices()
    const noexcept {
  return config_.bringup.mode ==
             RuntimeMode::kCommissioning &&
         config_.bringup
             .quarantine_failed_devices;
}

/* ============================================================
 * 配置入口
 * ============================================================ */

drivers::Result DeviceRegistry::configure() {
  if (configured_) {
    return drivers::Result::success(
        "device registry already configured");
  }

#if !SNAPPY_ENABLE_REAL_DRIVERS
  /*
   * real 后端未参与本次构建时，禁止任何真实设备配置继续进入
   * 总线创建流程，确保 OFF 构建不会打开 /dev/tty* 或 can0。
   */
  if (config_.bringup.mode ==
      RuntimeMode::kProduction) {
    return registry_error(
        "production real hardware config requires "
        "SNAPPY_ENABLE_REAL_DRIVERS=ON");
  }

  for (const auto& item : config_.devices) {
    const auto& device = item.second;

    if (device.enabled &&
        device.backend == "real") {
      return registry_error(
          "real device backend is disabled at build time: " +
          item.first);
    }
  }

  for (const auto& item : config_.rs485_ports) {
    if (item.second.backend == "serial") {
      return registry_error(
          "real RS485 backend is disabled at build time: " +
          item.first);
    }
  }

  for (const auto& item : config_.can_buses) {
    if (item.second.backend == "socketcan" ||
        item.second.backend == "real") {
      return registry_error(
          "real CAN backend is disabled at build time: " +
          item.first);
    }
  }
#endif

  auto result = configure_buses();
  if (!result.ok) {
    return result;
  }

  result = bind_rs485_devices();
  if (!result.ok) {
    return result;
  }

  result = create_devices();
  if (!result.ok) {
    return result;
  }

  result = create_services();
  if (!result.ok) {
    return result;
  }

  configured_ = true;

  return drivers::Result::success(
      "device registry configured");
}

/* ============================================================
 * 创建 CAN 和 RS485 总线
 * ============================================================ */

drivers::Result
DeviceRegistry::configure_buses() {
  const auto can_used =
      used_can_buses(config_);

  const auto rs485_used =
      used_rs485_ports(config_);

  if (production_mode()) {
    if (config_.can_buses.empty()) {
      return registry_error(
          "production hardware config has no CAN buses");
    }

    if (config_.rs485_ports.empty()) {
      return registry_error(
          "production hardware config has no RS485 ports");
    }
  }

  if (!can_used.empty() ||
      production_mode()) {
    const auto can_it =
        !can_used.empty()
            ? config_.can_buses.find(
                  *can_used.begin())
            : config_.can_buses.find(
                  drivers::bus::
                      kPortCan0Id);

    const auto selected_it =
        can_it != config_.can_buses.end()
            ? can_it
            : config_.can_buses.begin();

    if (selected_it ==
        config_.can_buses.end()) {
      return registry_error(
          "enabled CAN device references missing CAN bus");
    }

    auto can_config =
        selected_it->second;

    /*
     * 调试模式必须允许进程在 can0 不存在或初始化失败时继续。
     * SocketCanBus 本身仍保持 Fault，不会把失败伪装成 Ready。
     */
    if (commissioning_ignore_startup_failures()) {
      can_config.allow_start_failure =
          true;
    }

    can_bus_ =
        std::make_unique<
            drivers::bus::SocketCanBus>(
            can_config);
  } else {
    can_bus_ =
        std::make_unique<NoopCanBus>();
  }

  if (!rs485_used.empty() ||
      production_mode()) {
    auto scheduler =
        std::make_unique<
            drivers::bus::
                Rs485TransactionScheduler>();

    if (commissioning_ignore_startup_failures()) {
      scheduler->set_allow_missing_ports(
          true);
    }

    scheduler->set_device_quarantine(
        commissioning_quarantine_devices(),
        config_.bringup
            .quarantine_after_failures,
        config_.bringup
            .quarantine_duration);

    if (production_mode()) {
      for (const auto& item :
           config_.rs485_ports) {
        const auto added =
            scheduler->add_port(
                item.second);

        if (!added.ok) {
          return added;
        }
      }
    } else {
      for (const auto& port_id :
           rs485_used) {
        const auto it =
            config_.rs485_ports.find(
                port_id);

        if (it ==
            config_.rs485_ports.end()) {
          return registry_error(
              "enabled device references missing RS485 port: " +
              port_id);
        }

        const auto added =
            scheduler->add_port(
                it->second);

        if (!added.ok) {
          return added;
        }
      }
    }

    rs485_scheduler_ =
        scheduler.get();

    rs485_bus_ =
        std::move(scheduler);
  } else {
    rs485_scheduler_ = nullptr;
    rs485_bus_ =
        std::make_unique<
            NoopRs485Bus>();
  }

  return drivers::Result::success();
}

/* ============================================================
 * 绑定 RS485 逻辑设备
 * ============================================================ */

drivers::Result
DeviceRegistry::bind_rs485_devices() {
  if (rs485_scheduler_ == nullptr) {
    return drivers::Result::failure(
        "rs485 scheduler not configured");
  }

  for (const auto& [device_id, device] :
       config_.devices) {
    if (!device.enabled ||
        !device_uses_rs485(
            config_,
            device)) {
      continue;
    }

    if (device.bus_id.empty()) {
      return drivers::Result::failure(
          "rs485 device " + device_id +
          " missing bus id");
    }

    drivers::bus::Rs485DeviceBinding binding;
    binding.logical_device = device.device_id;
    binding.port_id = device.bus_id;
    binding.max_response_bytes =
        max_response_bytes_for_device(device);
    binding.timeout = device.response_deadline;
    binding.max_consecutive_failures =
        device.max_consecutive_failures;
    binding.quarantine_duration =
        device.quarantine_duration;
    binding.read_retry_count =
        device.read_retry_count;
    binding.priority = device.priority;

    const auto bound =
        rs485_scheduler_->bind_logical_device(
            binding);

    if (!bound.ok) {
      return bound;
    }
  }

  return drivers::Result::success();
}

/* ============================================================
 * 创建设备对象
 * ============================================================ */

drivers::Result
DeviceRegistry::create_devices() {
  if (!can_bus_ ||
      !rs485_bus_) {
    return registry_error(
        "buses must be configured before devices");
  }

  const auto require_rs485 =
      [&](const DeviceConfig& device)
      -> drivers::Result {
        if (rs485_scheduler_ == nullptr) {
          return registry_error(
              "RS485 scheduler required for " +
              device.device_id);
        }

        return drivers::Result::success();
      };

  const auto create_distance =
      [&](const char* id,
          std::unique_ptr<
              drivers::devices::
                  DistanceSensor>& target)
      -> drivers::Result {
        const auto* device =
            find_enabled_device(
                config_,
                id);

        if (device == nullptr) {
          return drivers::Result::success();
        }

        const auto ready =
            require_rs485(*device);

        if (!ready.ok) {
          return ready;
        }

        auto device_config =
            make_distance_config_from_device(
                *device);

        target =
            std::make_unique<
                drivers::devices::
                    DistanceSensor>(
                *rs485_scheduler_,
                device_config);

        return drivers::Result::success();
      };

  auto result =
      create_distance(
          "lidar_brush_left",
          lidar_brush_left_);
  if (!result.ok) return result;

  result =
      create_distance(
          "lidar_brush_right",
          lidar_brush_right_);
  if (!result.ok) return result;

  result =
      create_distance(
          "lidar_body_front",
          lidar_body_front_);
  if (!result.ok) return result;

  result =
      create_distance(
          "lidar_body_rear",
          lidar_body_rear_);
  if (!result.ok) return result;

  if (const auto* device =
          find_enabled_device(
              config_,
              "anemometer_left")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_anemometer_left_config();

    apply_station_id(
        device_config,
        *device);

    anemometer_left_ =
        std::make_unique<
            drivers::devices::
                Anemometer>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "anemometer_right")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_anemometer_right_config();

    apply_station_id(
        device_config,
        *device);

    anemometer_right_ =
        std::make_unique<
            drivers::devices::
                Anemometer>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "wit_motion")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_wit_motion_config();

    apply_station_id(
        device_config,
        *device);

    wit_motion_ =
        std::make_unique<
            drivers::devices::
                WitMotion>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "temperature_cabinet")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_temperature_cabinet_config();

    apply_station_id(
        device_config,
        *device);

    temperature_cabinet_ =
        std::make_unique<
            drivers::devices::
                TemperatureSensor>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "power_24v")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_power_24v_config();

    apply_station_id(
        device_config,
        *device);

    power_24v_ =
        std::make_unique<
            drivers::devices::
                PowerMonitor>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "power_48v")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_power_48v_config();

    apply_station_id(
        device_config,
        *device);

    power_48v_ =
        std::make_unique<
            drivers::devices::
                PowerMonitor>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "relay_push_motor")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_relay_push_motor_config();

    apply_station_id(
        device_config,
        *device);

    if (device->channel_count) {
      device_config.profile.channel_count =
          *device->channel_count;
    }

    relay_push_motor_ =
        std::make_unique<
            drivers::devices::
                RelayController>(
            *rs485_scheduler_,
            device_config);

    relay_bank_ =
        std::make_unique<
            RelayBankAdapter>(
            *relay_push_motor_,
            config_.relay_outputs,
            commissioning_ignore_startup_failures());
  } else if (
      commissioning_noop_actuators()) {
    relay_bank_ =
        std::make_unique<
            NoopRelayBankAdapter>();
  } else {
    return registry_error(
        "relay_push_motor is required");
  }

  if (relay_push_motor_) {
    auto brush_push_config =
        drivers::devices::
            make_push_motor_config();

    brush_push_config.device_id =
        "brush_push_motor";

    brush_push_config.dead_time =
        config_.safety
            .push_motor_dead_time;

    brush_push_config.max_on_time =
        config_.safety
            .brush_push_motor_max_on_time;

    const auto brush_extend =
        relay_channel(
            config_,
            "brush_push_extend",
            "push_motor_extend");

    const auto brush_retract =
        relay_channel(
            config_,
            "brush_push_retract",
            "push_motor_retract");

    if (!brush_extend ||
        !brush_retract) {
      if (production_mode()) {
        return registry_error(
            "brush push motor relay outputs are required");
      }
    } else {
      brush_push_config.extend_channel =
          *brush_extend;
      brush_push_config.retract_channel =
          *brush_retract;

      brush_push_motor_ =
          std::make_unique<
              drivers::devices::
                  PushMotor>(
              *relay_push_motor_,
              brush_push_config);
    }

    auto scraper_push_config =
        drivers::devices::
            make_push_motor_config();

    scraper_push_config.device_id =
        "scraper_push_motor";

    scraper_push_config.dead_time =
        config_.safety
            .push_motor_dead_time;

    scraper_push_config.max_on_time =
        config_.safety
            .scraper_push_motor_max_on_time;

    const auto scraper_extend =
        relay_channel(
            config_,
            "scraper_push_extend",
            "relay_spare_3");

    const auto scraper_retract =
        relay_channel(
            config_,
            "scraper_push_retract",
            "relay_spare_4");

    if (!scraper_extend ||
        !scraper_retract) {
      if (production_mode()) {
        return registry_error(
            "scraper push motor relay outputs are required");
      }
    } else {
      scraper_push_config.extend_channel =
          *scraper_extend;
      scraper_push_config.retract_channel =
          *scraper_retract;

      scraper_push_motor_ =
          std::make_unique<
              drivers::devices::
                  PushMotor>(
              *relay_push_motor_,
              scraper_push_config);
    }
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "brush_motor")) {
    result = require_rs485(*device);
    if (!result.ok) return result;

    auto device_config =
        drivers::devices::
            make_brush_motor_config();

    apply_station_id(
        device_config,
        *device);

    brush_motor_ =
        std::make_unique<
            drivers::devices::
                BrushMotor>(
            *rs485_scheduler_,
            device_config);
  }

  if (const auto* device =
          find_enabled_device(
              config_,
              "propulsion_fan")) {
    if (!device_uses_can(
            config_,
            *device)) {
      return registry_error(
          "propulsion_fan requires a CAN bus_id");
    }

    auto device_config =
        drivers::devices::
            make_propulsion_fan_config();

    if (device->source_node_id) {
      device_config.profile
          .source_node_id =
          *device->source_node_id;
    }

    if (device->left_node_id) {
      device_config.left_node_id =
          *device->left_node_id;
    }

    if (device->right_node_id) {
      device_config.right_node_id =
          *device->right_node_id;
    }

    if (device->left_throttle_channel) {
      device_config.left_channel_index =
          *device->left_throttle_channel;
    }

    if (device->right_throttle_channel) {
      device_config.right_channel_index =
          *device->right_throttle_channel;
    }

    if (device->max_throttle_raw) {
      device_config.max_throttle_raw =
          *device->max_throttle_raw;
    }

    propulsion_fan_ =
        std::make_unique<
            drivers::devices::
                PropulsionFanController>(
            *can_bus_,
            device_config);
  }

  const DeviceConfig* climber_device =
      find_enabled_device(
          config_,
          "climber");

  if (climber_device == nullptr) {
    climber_device =
        find_enabled_device(
            config_,
            "rope_climber");
  }

  if (climber_device != nullptr) {
    result =
        require_rs485(
            *climber_device);

    if (!result.ok) {
      return result;
    }

    if (!climber_device->station_id) {
      return registry_error(
          "L7P climber requires station_id");
    }

    auto climber_config =
        drivers::devices::
            make_climber_config();

    apply_station_id(
        climber_config,
        *climber_device);

    climber_ =
        std::make_unique<
            drivers::devices::
                Climber>(
            *rs485_scheduler_,
            climber_config);
  }

  return drivers::Result::success();
}

/* ============================================================
 * 创建业务服务
 * ============================================================ */

drivers::Result
DeviceRegistry::create_services() {
  if (!relay_bank_) {
    if (commissioning_noop_actuators()) {
      relay_bank_ =
          std::make_unique<
              NoopRelayBankAdapter>();
    } else {
      return registry_error(
          "relay bank was not created");
    }
  }

  ActuatorServiceDevices
      actuator_devices;

  actuator_devices.brush_motor =
      brush_motor_.get();

  actuator_devices.propulsion_fan =
      propulsion_fan_.get();

  actuator_devices.brush_push_motor =
      brush_push_motor_.get();

  actuator_devices.scraper_push_motor =
      scraper_push_motor_.get();

  actuator_devices.push_motor =
      brush_push_motor_.get();

  actuator_devices.climber =
      climber_.get();

  ActuatorServiceOptions
      actuator_options;

  actuator_options.noop_missing_actuators =
      commissioning_noop_actuators();

  actuator_options.ignore_startup_failures =
      commissioning_ignore_startup_failures();

  actuator_service_ =
      std::make_unique<
          ActuatorService>(
          actuator_devices,
          actuator_options);

  SensorServiceDevices
      sensor_devices;

  sensor_devices.lidar_brush_left =
      lidar_brush_left_.get();

  sensor_devices.lidar_brush_right =
      lidar_brush_right_.get();

  sensor_devices.lidar_body_front =
      lidar_body_front_.get();

  sensor_devices.lidar_body_rear =
      lidar_body_rear_.get();

  sensor_devices.anemometer_left =
      anemometer_left_.get();

  sensor_devices.anemometer_right =
      anemometer_right_.get();

  sensor_devices.wit_motion =
      wit_motion_.get();

  sensor_devices.temperature_cabinet =
      temperature_cabinet_.get();

  sensor_devices.power_24v =
      power_24v_.get();

  sensor_devices.power_48v =
      power_48v_.get();

  SensorServiceOptions
      sensor_options;

  sensor_options.ignore_startup_failures =
      commissioning_ignore_startup_failures();

  sensor_options.background_polling =
      true;

  sensor_options.scheduler_period =
      std::chrono::milliseconds(20);

  sensor_options.poll_period =
      std::chrono::milliseconds(100);

  for (const auto& item :
       config_.devices) {
    if (item.second.enabled &&
        is_sensor_device_id(
            item.first)) {
      add_sensor_poll_config(
          sensor_options,
          config_,
          item.first);
    }
  }

  sensor_service_ =
      std::make_unique<
          SensorService>(
          sensor_devices,
          config_.safety
              .sensor_stale_timeout,
          sensor_options);

  return drivers::Result::success();
}

/* ============================================================
 * 服务访问
 * ============================================================ */

HardwarePorts
DeviceRegistry::hardware_ports() {
  return {
      *can_bus_,
      *rs485_bus_,
      *relay_bank_,
      *sensor_service_};
}

ActuatorService&
DeviceRegistry::actuators() {
  return *actuator_service_;
}

SensorService&
DeviceRegistry::sensors() {
  return *sensor_service_;
}

/* ============================================================
 * 整机安全停止
 * ============================================================ */

void DeviceRegistry::safe_stop()
    noexcept {
  if (actuator_service_) {
    static_cast<void>(
        actuator_service_
            ->safe_stop_all());
  }

  if (relay_bank_) {
    relay_bank_
        ->deenergize_all();
  }

  if (can_bus_) {
    can_bus_->safe_stop();
  }

  if (rs485_bus_) {
    rs485_bus_->safe_stop();
  }
}

/* ============================================================
 * 停止后台线程和底层总线
 *
 * 顺序：
 * 1. 停止传感器采集 worker。
 * 2. 停止执行器和推进风扇 worker。
 * 3. 断开继电器。
 * 4. 停止 RS485 worker。
 * 5. 关闭 SocketCAN。
 * ============================================================ */

void DeviceRegistry::stop() noexcept {
  if (sensor_service_) {
    sensor_service_->stop();
  }

  if (actuator_service_) {
    actuator_service_->stop();
  }

  if (relay_bank_) {
    relay_bank_
        ->deenergize_all();
  }

  if (rs485_scheduler_) {
    rs485_scheduler_->stop();
  }

  if (auto* socket_can =
          dynamic_cast<
              drivers::bus::
                  SocketCanBus*>(
              can_bus_.get())) {
    socket_can->stop();
  } else if (can_bus_) {
    can_bus_->safe_stop();
  }

  configured_ = false;
}

}  // namespace snappy::control