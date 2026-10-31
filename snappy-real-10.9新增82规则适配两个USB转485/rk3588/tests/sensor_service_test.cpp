/**
 * @file sensor_service_test.cpp
 * @brief 传感器采集服务测试
 *
 * @details
 * - 验证配置分组解析与失效判定。
 * - 验证单路设备失败不会阻塞同端口的其他设备。
 */

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "snappy/control/sensor_service.hpp"
#include "snappy/drivers/devices/anemometer.hpp"
#include "snappy/drivers/protocols/modbus_rtu.hpp"
#include "snappy/simulator/fake_hardware.hpp"

namespace {

using namespace std::chrono_literals;

bool expect(
    bool condition,
    const char* message) {
  if (condition) {
    return true;
  }

  std::cerr << message << '\n';
  return false;
}

std::vector<std::uint8_t> with_crc(
    std::vector<std::uint8_t> frame) {
  const auto crc =
      snappy::drivers::protocols::modbus_crc16(frame);

  frame.push_back(
      static_cast<std::uint8_t>(crc & 0xFFU));

  frame.push_back(
      static_cast<std::uint8_t>(
          (crc >> 8U) & 0xFFU));

  return frame;
}

const snappy::control::SensorDeviceStatus*
find_status(
    const std::vector<
        snappy::control::SensorDeviceStatus>& statuses,
    const std::string& name) {
  for (const auto& status : statuses) {
    if (status.logical_name == name) {
      return &status;
    }
  }

  return nullptr;
}

bool wait_for_sample(
    snappy::control::SensorService& service,
    const std::string& logical_name,
    snappy::drivers::SampleQuality quality,
    std::optional<double> expected_value =
        std::nullopt) {
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto sample =
        service.cached_sample(logical_name);

    if (sample &&
        sample->quality == quality &&
        (!expected_value ||
         (sample->value &&
          *sample->value == *expected_value))) {
      return true;
    }

    std::this_thread::sleep_for(2ms);
  }

  return false;
}

bool test_config_transaction_groups() {
  std::ifstream input(
      std::string(SNAPPY_SOURCE_DIR) +
      "/rk3588/config/hardware.debug.test.json");

  if (!expect(
          input.good(),
          "hardware.debug.test.json cannot be opened")) {
    return false;
  }

  const auto config =
      nlohmann::json::parse(input);

  const auto& ports =
      config.at("rs485_ports");

  // 三个 USB RS485 端口共用同一个 WCH 事务仲裁组。
  const auto& group0 =
      ports.at("port_usb_rs485_0")
           .at("shared_transaction_group");

  if (!expect(
          group0 == "wch_quad_rs485",
          "USB RS485 0 transaction group is invalid")) {
    return false;
  }

  if (!expect(
          ports.at("port_usb_rs485_1")
              .at("shared_transaction_group") == group0,
          "USB RS485 1 transaction group differs")) {
    return false;
  }

  if (!expect(
          ports.at("port_usb_rs485_2")
              .at("shared_transaction_group") == group0,
          "USB RS485 2 transaction group differs")) {
    return false;
  }

  return true;
}

bool test_failed_device_does_not_block_peer() {
  snappy::simulator::FakeRs485Bus bus;

  if (!expect(
          bus.start().ok,
          "fake RS485 did not start")) {
    return false;
  }

  bus.set_response(
      "anemometer_left",
      with_crc(
          {0x31, 0x03, 0x02, 0x03, 0xE8}));

  bus.set_failure(
      "anemometer_right",
      snappy::drivers::ErrorCode::kRs485Timeout,
      "injected right anemometer timeout");

  snappy::drivers::devices::Anemometer left(
      bus,
      snappy::drivers::devices::
          make_anemometer_left_config());

  snappy::drivers::devices::Anemometer right(
      bus,
      snappy::drivers::devices::
          make_anemometer_right_config());

  snappy::control::SensorServiceDevices devices;
  devices.anemometer_left = &left;
  devices.anemometer_right = &right;

  snappy::control::SensorServiceOptions options;
  options.scheduler_period = 1ms;
  options.ultrasonic_request_gap = 0ms;

  options.device_polls["anemometer_left"] = {
      true,
      "port_usb_rs485_1",
      5ms,
      200ms,
      3};

  options.device_polls["anemometer_right"] = {
      true,
      "port_usb_rs485_1",
      5ms,
      200ms,
      3};

  snappy::control::SensorService service(
      devices,
      200ms,
      options);

  if (!expect(
          service.start().ok,
          "sensor service did not start")) {
    return false;
  }

  const bool left_good =
      wait_for_sample(
          service,
          "anemometer_left_speed",
          snappy::drivers::SampleQuality::kGood,
          10.0);

  const bool right_invalid =
      wait_for_sample(
          service,
          "anemometer_right_speed",
          snappy::drivers::SampleQuality::kInvalid);

  if (!expect(
          left_good,
          "healthy peer did not update its cache")) {
    service.stop();
    return false;
  }

  if (!expect(
          right_invalid,
          "failed device did not become invalid")) {
    service.stop();
    return false;
  }

  const auto statuses =
      service.device_statuses();

  const auto* left_status =
      find_status(
          statuses,
          "anemometer_left");

  const auto* right_status =
      find_status(
          statuses,
          "anemometer_right");

  if (!expect(
          left_status != nullptr &&
              left_status->online,
          "healthy peer status is not online")) {
    service.stop();
    return false;
  }

  if (!expect(
          right_status != nullptr &&
              !right_status->online &&
              right_status->consecutive_failures > 0,
          "failed device status was not isolated")) {
    service.stop();
    return false;
  }

  bus.clear_failure(
      "anemometer_right");

  bus.set_response(
      "anemometer_right",
      with_crc(
          {0x32, 0x03, 0x02, 0x01, 0xF4}));

  const bool recovered =
      wait_for_sample(
          service,
          "anemometer_right_speed",
          snappy::drivers::SampleQuality::kGood,
          5.0);

  service.stop();

  return expect(
      recovered,
      "failed device did not recover");
}

}  // namespace

int main() {
  if (!test_config_transaction_groups()) {
    return 1;
  }

  if (!test_failed_device_does_not_block_peer()) {
    return 1;
  }

  std::cout
      << "sensor service isolation tests passed\n";

  return 0;
}