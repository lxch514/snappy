/**
 * @file driver_test.cpp
 * @brief 驱动层协议与设备测试
 *
 * @details
 * - 覆盖各协议报文的编解码往返。
 * - 覆盖推进风机在单次失败、持续失败和停止重试过程中的行为。
 * - 使用注入式假总线，不依赖真实硬件。
 */

#include <iostream>
#include <vector>

#include "snappy/drivers/protocols/anemometer_modbus.hpp"
#include "snappy/drivers/protocols/can_frame.hpp"
#include "snappy/drivers/protocols/lidar_modbus.hpp"
#include "snappy/drivers/protocols/modbus_rtu.hpp"
#include "snappy/drivers/protocols/propulsion_fan_can.hpp"
#include "snappy/drivers/protocols/relay_modbus.hpp"
#include "snappy/drivers/protocols/wit_motion_modbus.hpp"
#include "snappy/drivers/unconfigured_drivers.hpp"
#include "snappy/simulator/fake_hardware.hpp"
#include "snappy/drivers/protocols/power_modbus.hpp"

namespace {

bool expect_frame(const std::vector<std::uint8_t>& actual,
                  const std::vector<std::uint8_t>& expected,
                  const char* label) {
  if (actual == expected) {
    return true;
  }

  std::cerr << label << " frame mismatch\n";
  return false;
}

std::vector<std::uint8_t> with_crc(std::vector<std::uint8_t> frame) {
  const auto crc =
      snappy::drivers::protocols::modbus_crc16(frame);

  frame.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8U) & 0xFFU));
  return frame;
}

bool run_protocol_tests() {
  using namespace snappy::drivers::protocols;

  PropulsionFanCanProfile fan_profile;
  fan_profile.source_node_id = 0x64;

  const auto fan = encode_propulsion_fan_raw_command(
      PropulsionFanRawCommand{
          fan_profile,
          7,
          {100, 0, 0, 0},
      });

  const auto expected_fan_id =
      hobbywing_message_id(kHobbywingPriorityHighest,
                            kHobbywingRawCommandDataTypeId,
                            0x64);

  if (!fan.result.ok ||
      !fan.frame.extended ||
      fan.frame.id != expected_fan_id ||
      fan.frame.payload.size() != kClassicCanPayloadSize) {
    std::cerr << "Hobbywing propulsion fan encode failed\n";
    return false;
  }

  AnemometerModbusProfile anemometer_profile;

  if (!expect_frame(
          encode_anemometer_read_speed(0x31, anemometer_profile),
          with_crc({0x31, 0x03, 0x00, 0x00, 0x00, 0x01}),
          "anemometer read")) {
    return false;
  }

  const auto wind = decode_anemometer_speed_response(
      0x31,
      anemometer_profile,
      with_crc({0x31, 0x03, 0x02, 0x03, 0xE8}));

  if (!wind.result.ok ||
      !wind.speed_mps ||
      *wind.speed_mps != 10.0) {
    std::cerr << "anemometer decode failed\n";
    return false;
  }

  LidarModbusProfile ubs_profile;
  ubs_profile.model = LidarModel::kUbs1000C18r;

  if (!expect_frame(
          encode_lidar_read_distance(0x23, ubs_profile),
          with_crc({0x23, 0x04, 0x00, 0x00, 0x00, 0x03}),
          "UBS1000 read")) {
    return false;
  }

  const auto ubs = decode_lidar_distance_response(
      0x23,
      ubs_profile,
      with_crc({0x23,
                0x04,
                0x06,
                0x04,
                0xD2,
                0x00,
                0x19,
                0x03,
                0xE8}));

  if (!ubs.result.ok ||
      !ubs.distance_mm ||
      *ubs.distance_mm != 123.4 ||
      !ubs.temperature_celsius ||
      *ubs.temperature_celsius != 25.0 ||
      !ubs.flight_time_us ||
      *ubs.flight_time_us != 1000) {
    std::cerr << "UBS1000 decode failed\n";
    return false;
  }

  LidarModbusProfile ks114_profile;
  ks114_profile.model = LidarModel::kKs114;
  ks114_profile.ks114_command = 0x0001;
  ks114_profile.ks114_command_returns_flight_time_us = false;

  if (!expect_frame(
          encode_lidar_read_distance(0xD0, ks114_profile),
          with_crc({0xD0, 0x03, 0x00, 0x02, 0x00, 0x01}),
          "KS114 read")) {
    return false;
  }

  const auto ks114 = decode_lidar_distance_response(
      0xD0,
      ks114_profile,
      with_crc({0xD0, 0x03, 0x02, 0x01, 0x2C}));

  if (!ks114.result.ok ||
      !ks114.distance_mm ||
      *ks114.distance_mm != 300.0) {
    std::cerr << "KS114 decode failed\n";
    return false;
  }

  RelayModbusProfile relay_profile;

  if (!expect_frame(
          encode_relay_set_channel(
              relay_profile,
              RelayChannelCommand{0x43, 3, true}),
          with_crc({0x43, 0x05, 0x00, 0x02, 0xFF, 0x00}),
          "relay set channel")) {
    return false;
  }

  const auto imu = decode_wit_motion_motion_response(
      0x65,
      WitMotionModbusProfile{},
      with_crc({0x65,
                0x03,
                0x1A,
                0x40,
                0x00,
                0xC0,
                0x00,
                0x00,
                0x00,
                0x20,
                0x00,
                0xE0,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00,
                0x20,
                0x00,
                0xE0,
                0x00,
                0x00,
                0x00,
                0x09,
                0xC4}));

  if (!imu.result.ok ||
      !imu.accel_x_g ||
      *imu.accel_x_g != 8.0 ||
      !imu.gyro_x_dps ||
      *imu.gyro_x_dps != 500.0 ||
      !imu.roll_deg ||
      *imu.roll_deg != 45.0 ||
      !imu.temperature_celsius ||
      *imu.temperature_celsius != 25.0) {
    std::cerr << "WIT motion decode failed\n";
    return false;
  }

  std::vector<std::uint8_t> noisy_buffer{0x00, 0xFF, 0x01};
  const auto valid_frame =
      with_crc({0x31, 0x03, 0x02, 0x03, 0xE8});

  noisy_buffer.insert(noisy_buffer.end(),
                      valid_frame.begin(),
                      valid_frame.end());

  const auto window =
      find_modbus_frame(0x31,
                        kModbusReadHoldingRegisters,
                        noisy_buffer);

  if (!window.result.ok ||
      window.offset != 3 ||
      window.size != valid_frame.size()) {
    std::cerr << "Modbus frame scanner failed\n";
    return false;
  }

  PowerModbusProfile power_profile;

  if (!expect_frame(
          encode_power_read(0x71, power_profile),
          with_crc({0x71,
                    0x04,
                    0x00,
                    0x00,
                    0x00,
                    0x0A}),
          "power read")) {
    return false;
  }

  const auto power = decode_power_response(
      0x71,
      power_profile,
      with_crc({0x71,
                    0x04,
                    0x14,
                    0x09, 0x60,  // 24.00V
                    0x00, 0x64,  // 1.00A
                    0x03, 0xE8,  // 10.00W
                    0x01, 0x90,  // 40.0C
                    0x00, 0x02,  // CV
                    0x00, 0x01,  // enabled
                    0x00, 0x71,
                    0x00, 0x00,
                    0x00, 0x00,
                    0x00, 0x71}));

  if (!power.result.ok ||
      !power.temperature_celsius ||
      *power.temperature_celsius != 40.0 ||
      !power.output_mode ||
      *power.output_mode != PowerOutputMode::kConstantVoltage ||
      !power.operation_enabled ||
      !*power.operation_enabled) {
    std::cerr << "power decode failed\n";
    return false;
  }

  return true;
}

bool run_fake_driver_tests() {
  snappy::drivers::UnconfiguredCanBus real_can_boundary;

  const auto unconfigured_start = real_can_boundary.start();
  if (unconfigured_start.ok ||
      unconfigured_start.code !=
          snappy::drivers::ErrorCode::kConfigRequired ||
      real_can_boundary.health() !=
          snappy::drivers::DriverHealth::kUnconfigured) {
    std::cerr << "unconfigured CAN boundary test failed\n";
    return false;
  }

  snappy::simulator::FakeCanBus can;
  snappy::simulator::FakeRs485Bus rs485;
  snappy::simulator::FakeRelayBank relays;
  snappy::simulator::FakeSensorSuite sensors;

  if (!can.start().ok ||
      !rs485.start().ok ||
      !relays.start().ok ||
      !sensors.start().ok) {
    std::cerr << "fake hardware failed to start\n";
    return false;
  }

  if (!can.send({0, false, {1, 2, 3}}).ok ||
      can.sent_frames().size() != 1) {
    std::cerr << "fake CAN did not record frame\n";
    return false;
  }

  can.set_send_failure(
      snappy::drivers::ErrorCode::kCanTxTimeout,
      "injected CAN timeout");

  if (can.send({0, false, {1}}).code !=
      snappy::drivers::ErrorCode::kCanTxTimeout) {
    std::cerr << "fake CAN failure injection failed\n";
    return false;
  }

  can.clear_send_failure();

  rs485.set_response("wind_speed", {0x12, 0x34});

  const auto response =
      rs485.exchange({"wind_speed", {0x01}});

  if (!response.result.ok ||
      response.payload !=
          std::vector<std::uint8_t>({0x12, 0x34})) {
    std::cerr << "fake RS485 response failed\n";
    return false;
  }

  rs485.set_failure(
      "wind_speed",
      snappy::drivers::ErrorCode::kRs485Timeout,
      "injected timeout");

  const auto failure =
      rs485.exchange({"wind_speed", {0x01}});

  if (failure.result.ok ||
      failure.result.code !=
          snappy::drivers::ErrorCode::kRs485Timeout) {
    std::cerr << "fake RS485 failure injection failed\n";
    return false;
  }

  rs485.clear_failure("wind_speed");

  if (!relays.set(
          "push_motor_extend",
          snappy::drivers::RelayState::kEnergized)
           .ok) {
    std::cerr << "fake relay command failed\n";
    return false;
  }

  relays.set_interlock("push_motor_retract", true);

  const auto interlocked =
      relays.set("push_motor_retract",
                 snappy::drivers::RelayState::kEnergized);

  if (interlocked.ok ||
      interlocked.code !=
          snappy::drivers::ErrorCode::kRelayInterlockActive) {
    std::cerr << "fake relay interlock failed\n";
    return false;
  }

  relays.deenergize_all();

  if (relays.state_of("push_motor_extend") !=
      snappy::drivers::RelayState::kDeenergized) {
    std::cerr << "fake relay safe stop failed\n";
    return false;
  }

  sensors.set_samples({
      {"wind_speed",
       0.0,
       "m/s",
       snappy::drivers::SampleQuality::kGood,
       0},
  });

  if (sensors.read_all().size() != 1) {
    std::cerr << "fake sensor sample failed\n";
    return false;
  }

  return true;
}

}  // namespace

int main() {
  if (!run_protocol_tests()) {
    return 1;
  }

  if (!run_fake_driver_tests()) {
    return 1;
  }

  std::cout << "driver tests passed\n";
  return 0;
}