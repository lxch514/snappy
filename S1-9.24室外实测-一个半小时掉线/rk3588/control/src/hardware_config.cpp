#include "snappy/control/hardware_config.hpp"

/**
 * @file hardware_config.cpp
 * @brief RK3588 简易机硬件配置解析与启动前校验
 *
 * @details
 * - 解析 JSON 配置，生成控制程序使用的 HardwareConfig。
 * - 检查 CAN、RS485、设备、继电器、G20、视频和看门狗配置。
 * - 检查物理端口 backend、设备路径和事务恢复参数。
 * - 检查同一 RS485 端口内站号是否重复。
 * - 检查 WCH USB Quad Serial 的多个 ACM 通道是否使用同一个事务组。
 * - 检查设备 model 与 protocol_profile 是否匹配。
 * - 检查生产模式是否显式填写设备时序和安全样本。
 * - 不在配置阶段把失败设备标记为 Ready。
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace snappy::control {
namespace {

using Json = nlohmann::json;

/* ============================================================
 * 通用 JSON 辅助
 * ============================================================ */

drivers::Result config_error(
    const std::string& detail) {
  return drivers::Result::failure(
      drivers::ErrorCode::kConfigRequired,
      detail,
      "load",
      "hardware_config");
}

bool has_value(
    const Json& object,
    const char* key) {
  if (!object.is_object()) {
    return false;
  }

  const auto it =
      object.find(key);

  return it != object.end() &&
         !it->is_null();
}

std::string string_or(
    const Json& object,
    const char* key,
    const std::string& fallback = {}) {
  if (!has_value(object, key)) {
    return fallback;
  }

  return object.at(key)
      .get<std::string>();
}

std::vector<std::string>
string_array_or(
    const Json& object,
    const char* key) {
  if (!has_value(object, key)) {
    return {};
  }

  return object.at(key)
      .get<std::vector<std::string>>();
}

bool bool_or(
    const Json& object,
    const char* key,
    bool fallback) {
  if (!has_value(object, key)) {
    return fallback;
  }

  return object.at(key)
      .get<bool>();
}

int int_or(
    const Json& object,
    const char* key,
    int fallback) {
  if (!has_value(object, key)) {
    return fallback;
  }

  return object.at(key)
      .get<int>();
}

std::chrono::milliseconds ms_or(
    const Json& object,
    const char* key,
    int fallback_ms) {
  return std::chrono::milliseconds(
      int_or(
          object,
          key,
          fallback_ms));
}

/* ============================================================
 * 数字和 ID 解析
 * ============================================================ */

std::optional<std::uint32_t>
parse_decimal_or_hex_u32(
    const std::string& value) {
  if (value.empty()) {
    return std::nullopt;
  }

  try {
    std::size_t consumed = 0;

    const bool hexadecimal =
        value.size() > 2 &&
        value[0] == '0' &&
        (value[1] == 'x' ||
         value[1] == 'X');

    const int base =
        hexadecimal ? 16 : 10;

    const auto parsed =
        std::stoull(
            value,
            &consumed,
            base);

    if (consumed != value.size() ||
        parsed >
            std::numeric_limits<
                std::uint32_t>::max()) {
      return std::nullopt;
    }

    return static_cast<std::uint32_t>(
        parsed);
  } catch (...) {
    return std::nullopt;
  }
}

std::optional<std::uint32_t>
optional_u32(
    const Json& object,
    const char* key) {
  if (!has_value(object, key)) {
    return std::nullopt;
  }

  const auto& value =
      object.at(key);

  try {
    if (value.is_number_unsigned()) {
      const auto raw =
          value.get<std::uint64_t>();

      if (raw >
          std::numeric_limits<
              std::uint32_t>::max()) {
        return std::nullopt;
      }

      return static_cast<std::uint32_t>(
          raw);
    }

    if (value.is_number_integer()) {
      const auto raw =
          value.get<std::int64_t>();

      if (raw < 0 ||
          static_cast<std::uint64_t>(
              raw) >
              std::numeric_limits<
                  std::uint32_t>::max()) {
        return std::nullopt;
      }

      return static_cast<std::uint32_t>(
          raw);
    }

    if (value.is_string()) {
      return parse_decimal_or_hex_u32(
          value.get<std::string>());
    }
  } catch (...) {
    return std::nullopt;
  }

  return std::nullopt;
}

std::optional<std::uint8_t>
optional_u8(
    const Json& object,
    const char* key) {
  const auto raw =
      optional_u32(
          object,
          key);

  if (!raw ||
      *raw >
          std::numeric_limits<
              std::uint8_t>::max()) {
    return std::nullopt;
  }

  return static_cast<std::uint8_t>(
      *raw);
}

std::optional<std::uint16_t>
optional_u16(
    const Json& object,
    const char* key) {
  const auto raw =
      optional_u32(
          object,
          key);

  if (!raw ||
      *raw >
          std::numeric_limits<
              std::uint16_t>::max()) {
    return std::nullopt;
  }

  return static_cast<std::uint16_t>(
      *raw);
}

std::optional<std::size_t>
optional_size(
    const Json& object,
    const char* key) {
  const auto raw =
      optional_u32(
          object,
          key);

  if (!raw) {
    return std::nullopt;
  }

  return static_cast<std::size_t>(
      *raw);
}

/* ============================================================
 * 字符串和路径辅助
 * ============================================================ */

std::string to_lower_copy(
    std::string value) {
  std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char ch) {
        return static_cast<char>(
            std::tolower(ch));
      });

  return value;
}

bool contains_token(
    const std::string& value,
    const std::string& token) {
  return to_lower_copy(value)
             .find(
                 to_lower_copy(token)) !=
         std::string::npos;
}

bool is_wch_acm_path(
    const std::string& path) {
  const auto lower =
      to_lower_copy(path);

  return lower.find("ttyacm") !=
             std::string::npos ||
         lower.find("/dev/acm") !=
             std::string::npos;
}

/* ============================================================
 * backend 校验
 * ============================================================ */

bool valid_rs485_backend(
    const std::string& value) {
  const auto backend =
      to_lower_copy(value);

  return backend == "serial" ||
         backend == "real" ||
         backend == "fake" ||
         backend == "noop";
}

bool valid_can_backend(
    const std::string& value) {
  const auto backend =
      to_lower_copy(value);

  return backend == "socketcan" ||
         backend == "real" ||
         backend == "fake" ||
         backend == "noop";
}

bool valid_device_backend(
    const std::string& value) {
  const auto backend =
      to_lower_copy(value);

  return backend == "real" ||
         backend == "fake" ||
         backend == "noop";
}

/* ============================================================
 * RS485 配置解析
 * ============================================================ */

bool valid_parity_name(
    const std::string& value) {
  const auto lower =
      to_lower_copy(value);

  return lower == "none" ||
         lower == "even" ||
         lower == "odd";
}

drivers::bus::Rs485Parity
parse_parity(
    const std::string& value) {
  const auto lower =
      to_lower_copy(value);

  if (lower == "even") {
    return drivers::bus::Rs485Parity::kEven;
  }

  if (lower == "odd") {
    return drivers::bus::Rs485Parity::kOdd;
  }

  return drivers::bus::Rs485Parity::kNone;
}

bool valid_direction_mode_name(
    const std::string& value) {
  const auto lower =
      to_lower_copy(value);

  return lower == "auto" ||
         lower == "none" ||
         lower == "disabled" ||
         lower ==
             "kernel_rts_high_on_send" ||
         lower ==
             "rts_high_on_send" ||
         lower ==
             "kernel_rts_low_on_send" ||
         lower ==
             "rts_low_on_send";
}

drivers::bus::Rs485DirectionMode
parse_direction_mode(
    const std::string& value) {
  const auto lower =
      to_lower_copy(value);

  if (lower == "none" ||
      lower == "disabled") {
    return drivers::bus::
        Rs485DirectionMode::kNone;
  }

  if (lower ==
          "kernel_rts_high_on_send" ||
      lower ==
          "rts_high_on_send") {
    return drivers::bus::
        Rs485DirectionMode::
            kKernelRtsHighOnSend;
  }

  if (lower ==
          "kernel_rts_low_on_send" ||
      lower ==
          "rts_low_on_send") {
    return drivers::bus::
        Rs485DirectionMode::
            kKernelRtsLowOnSend;
  }

  return drivers::bus::
      Rs485DirectionMode::kAuto;
}

drivers::bus::Rs485PortConfig
parse_rs485_port(
    const std::string& port_id,
    const Json& value) {
  drivers::bus::Rs485PortConfig config;

  config.port_id =
      port_id;

  config.backend =
      string_or(
          value,
          "backend",
          "serial");

  config.device_path =
      string_or(
          value,
          "device_path");

  config.baud_rate =
      int_or(
          value,
          "baud_rate",
          drivers::bus::
              kDefaultRs485BaudRate);

  config.format.data_bits =
      int_or(
          value,
          "data_bits",
          8);

  config.format.parity =
      parse_parity(
          string_or(
              value,
              "parity",
              "none"));

  config.format.stop_bits =
      int_or(
          value,
          "stop_bits",
          1);

  config.inter_frame_gap =
      ms_or(
          value,
          "inter_frame_gap_ms",
          4);

  config.read_idle_gap =
      ms_or(
          value,
          "read_idle_gap_ms",
          4);

  config.late_frame_silence =
      ms_or(
          value,
          "late_frame_silence_ms",
          8);

  config.late_frame_recovery_timeout =
      ms_or(
          value,
          "late_frame_recovery_timeout_ms",
          80);

  config.low_latency =
      bool_or(
          value,
          "low_latency",
          true);

  config.shared_lock_group =
      string_or(
          value,
          "shared_lock_group");

  /*
   * WCH USB Quad Serial 的 ACM0~ACM3
   * 必须共用同一个完整事务互斥域。
   */
  config.shared_transaction_group =
      string_or(
          value,
          "shared_transaction_group");

  if (is_wch_acm_path(
          config.device_path) &&
      config.shared_transaction_group.empty()) {
    config.shared_transaction_group =
        "wch_usb_quad_0";
  }

  if (config.shared_transaction_group.empty()) {
    config.shared_transaction_group =
        config.shared_lock_group;
  }

  config.direction_mode =
      parse_direction_mode(
          string_or(
              value,
              "rs485_direction_mode",
              "auto"));

  config.max_pending_transactions =
      static_cast<std::size_t>(
          int_or(
              value,
              "max_pending_transactions",
              32));

  config.recovery_delay =
      ms_or(
          value,
          "recovery_delay_ms",
          50);

  return config;
}

/* ============================================================
 * CAN 配置解析
 * ============================================================ */

drivers::bus::CanBusConfig
parse_can_bus(
    const std::string& bus_id,
    const Json& value) {
  drivers::bus::CanBusConfig config;

  config.bus_id =
      bus_id;

  config.backend =
      string_or(
          value,
          "backend",
          "socketcan");

  config.interface_name =
      string_or(
          value,
          "interface_name",
          "can0");

  if (const auto bitrate =
          optional_u32(
              value,
              "bitrate")) {
    config.bitrate =
        *bitrate;
  } else if (
      const auto bitrate_bps =
          optional_u32(
              value,
              "bitrate_bps")) {
    config.bitrate =
        *bitrate_bps;
  }

  config.tx_queue_length =
      optional_u32(
          value,
          "tx_queue_length")
          .value_or(
              1000U);

  config.auto_configure_interface =
      bool_or(
          value,
          "auto_configure_interface",
          true);

  config.restart_on_bus_off =
      bool_or(
          value,
          "restart_on_bus_off",
          true);

  config.restart_ms =
      ms_or(
          value,
          "restart_ms",
          100);

  config.receive_own_messages =
      bool_or(
          value,
          "receive_own_messages",
          false);

  config.default_tx_timeout =
      ms_or(
          value,
          "default_tx_timeout_ms",
          20);

  config.default_rx_timeout =
      ms_or(
          value,
          "default_rx_timeout_ms",
          20);

  config.allow_start_failure =
      bool_or(
          value,
          "allow_start_failure",
          false);

  return config;
}

/* ============================================================
 * 运行模式解析
 * ============================================================ */

RuntimeMode parse_runtime_mode(
    const Json& value) {
  const auto mode =
      to_lower_copy(
          string_or(
              value,
              "mode",
              "production"));

  if (mode == "commissioning" ||
      mode == "bringup" ||
      mode == "debug" ||
      mode == "permissive_debug") {
    return RuntimeMode::kCommissioning;
  }

  return RuntimeMode::kProduction;
}

BringupConfig parse_bringup(
    const Json& value) {
  BringupConfig bringup;

  bringup.mode =
      parse_runtime_mode(value);

  bringup.allow_partial_devices =
      bool_or(
          value,
          "allow_partial_devices",
          false);

  bringup.noop_missing_actuators =
      bool_or(
          value,
          "noop_missing_actuators",
          false);

  bringup.ignore_device_startup_failures =
      bool_or(
          value,
          "ignore_device_startup_failures",
          false);

  bringup.quarantine_failed_devices =
      bool_or(
          value,
          "quarantine_failed_devices",
          false);

  bringup.quarantine_after_failures =
      int_or(
          value,
          "quarantine_after_failures",
          2);

  bringup.quarantine_duration =
      ms_or(
          value,
          "quarantine_duration_ms",
          2000);

  bringup.arm_requires_all_required_devices =
      bool_or(
          value,
          "arm_requires_all_required_devices",
          bringup.mode ==
              RuntimeMode::kProduction);

  return bringup;
}

/* ============================================================
 * 设备配置解析
 * ============================================================ */

bool is_sensor_like(
    const DeviceConfig& config) {
  return contains_token(
             config.role,
             "distance") ||
         contains_token(
             config.role,
             "wind") ||
         contains_token(
             config.role,
             "attitude") ||
         contains_token(
             config.role,
             "environment") ||
         contains_token(
             config.role,
             "power") ||
         config.protocol_profile ==
             "ks114" ||
         config.protocol_profile ==
             "ubs1000_c18r" ||
         config.protocol_profile ==
             "yn8588b" ||
         config.protocol_profile ==
             "wit_motion" ||
         config.protocol_profile ==
             "temperature_humidity" ||
         config.protocol_profile ==
             "pds3000";
}

int default_poll_interval_ms(
    const DeviceConfig& config) {
  if (config.protocol_profile ==
          "ks114" ||
      config.protocol_profile ==
          "ubs1000_c18r") {
    return 150;
  }

  if (config.protocol_profile ==
          "yn8588b" ||
      config.protocol_profile ==
          "wit_motion") {
    return 200;
  }

  if (config.protocol_profile ==
          "temperature_humidity" ||
      config.protocol_profile ==
          "pds3000") {
    return 1000;
  }

  return is_sensor_like(config)
             ? 500
             : 0;
}

int default_response_deadline_ms(
    const DeviceConfig& config) {
  if (config.protocol_profile ==
      "l7p_speed_mode") {
    return 300;
  }

  if (config.protocol_profile ==
          "bd_201s" ||
      config.protocol_profile ==
          "zs_dio_r_4_relay") {
    return 200;
  }

  if (config.protocol_profile ==
      "x6_plus_raw_command") {
    return 20;
  }

  if (config.protocol_profile ==
          "temperature_humidity" ||
      config.protocol_profile ==
          "pds3000") {
    return 1500;
  }

  return 120;
}

int default_stale_after_ms(
    const DeviceConfig& config) {
  const int poll_ms =
      default_poll_interval_ms(
          config);

  if (poll_ms <= 0) {
    return 1000;
  }

  return std::max(
      1000,
      poll_ms * 5);
}

DeviceConfig parse_device(
    const std::string& device_id,
    const Json& value) {
  DeviceConfig config;

  config.device_id =
      device_id;

  config.enabled =
      bool_or(
          value,
          "enabled",
          false);

  config.backend =
      string_or(
          value,
          "backend");

  config.bus_id =
      string_or(
          value,
          "bus_id");

  config.model =
      string_or(
          value,
          "model");

  config.protocol_family =
      string_or(
          value,
          "protocol_family");

  config.protocol_profile =
      string_or(
          value,
          "protocol_profile");

  config.role =
      string_or(
          value,
          "role");

  config.station_id =
      optional_u8(
          value,
          "station_id");

  config.source_node_id =
      optional_u8(
          value,
          "source_node_id");

  config.left_node_id =
      optional_u8(
          value,
          "left_node_id");

  config.right_node_id =
      optional_u8(
          value,
          "right_node_id");

  config.left_throttle_channel =
      optional_size(
          value,
          "left_throttle_channel");

  config.right_throttle_channel =
      optional_size(
          value,
          "right_throttle_channel");

  config.max_throttle_raw =
      optional_u16(
          value,
          "max_throttle_raw");

  config.channel_count =
      optional_u16(
          value,
          "channel_count");

  config.tx_can_id =
      optional_u32(
          value,
          "tx_can_id");

  config.rx_can_id =
      optional_u32(
          value,
          "rx_can_id");

  config.can_extended =
      bool_or(
          value,
          "can_extended",
          false);

  config.poll_interval =
      ms_or(
          value,
          "poll_interval_ms",
          default_poll_interval_ms(
              config));

  config.response_deadline =
      ms_or(
          value,
          "response_deadline_ms",
          default_response_deadline_ms(
              config));

  config.stale_after =
      ms_or(
          value,
          "stale_after_ms",
          default_stale_after_ms(
              config));

  config.priority =
      int_or(
          value,
          "priority",
          is_sensor_like(config)
              ? 20
              : 5);

  config.max_consecutive_failures =
      int_or(
          value,
          "max_consecutive_failures",
          3);

  config.quarantine_duration =
      ms_or(
          value,
          "quarantine_duration_ms",
          2000);

  config.read_retry_count =
      int_or(
          value,
          "read_retry_count",
          1);

  config.startup_required =
      bool_or(
          value,
          "startup_required",
          true);

  config.cache_failures =
      bool_or(
          value,
          "cache_failures",
          true);

  return config;
}

/* ============================================================
 * 安全、链路、视频和看门狗配置
 * ============================================================ */

SafetyConfig parse_safety(
    const Json& value) {
  SafetyConfig safety;

  safety.g20_lease_timeout =
      ms_or(
          value,
          "g20_lease_timeout_ms",
          750);

  safety.sensor_stale_timeout =
      ms_or(
          value,
          "sensor_stale_timeout_ms",
          1000);

  safety.rs485_retry_count =
      int_or(
          value,
          "rs485_retry_count",
          1);

  safety.required_samples =
      string_array_or(
          value,
          "required_samples");

  safety.push_motor_dead_time =
      ms_or(
          value,
          "push_motor_dead_time_ms",
          200);

  safety.push_motor_max_on_time =
      ms_or(
          value,
          "push_motor_max_on_time_ms",
          10000);

  safety.brush_push_motor_max_on_time =
      ms_or(
          value,
          "brush_push_motor_max_on_time_ms",
          static_cast<int>(
              safety.push_motor_max_on_time.count()));

  safety.scraper_push_motor_max_on_time =
      ms_or(
          value,
          "scraper_push_motor_max_on_time_ms",
          static_cast<int>(
              safety.push_motor_max_on_time.count()));

  safety.safe_stop_on_g20_loss =
      bool_or(
          value,
          "safe_stop_on_g20_loss",
          true);

  safety.safe_stop_on_rs485_bus_fault =
      bool_or(
          value,
          "safe_stop_on_rs485_bus_fault",
          true);

  safety.safe_stop_on_can_bus_fault =
      bool_or(
          value,
          "safe_stop_on_can_bus_fault",
          true);

  safety.safe_stop_on_sensor_stale =
      bool_or(
          value,
          "safe_stop_on_sensor_stale",
          true);

  safety.allow_remote_fault_clear =
      bool_or(
          value,
          "allow_remote_fault_clear",
          true);

  return safety;
}

G20LinkConfig parse_g20_link(
    const Json& value) {
  G20LinkConfig link;

  link.backend =
      string_or(
          value,
          "backend",
          "udp");

  link.listen_address =
      string_or(
          value,
          "listen_address",
          "0.0.0.0");

  link.listen_port =
      static_cast<std::uint16_t>(
          int_or(
              value,
              "listen_port",
              14550));

  link.command_listen_port =
      static_cast<std::uint16_t>(
          int_or(
              value,
              "command_listen_port",
              link.listen_port));

  link.status_publish_port =
      static_cast<std::uint16_t>(
          int_or(
              value,
              "status_publish_port",
              14551));

  link.telemetry_publish_port =
      static_cast<std::uint16_t>(
          int_or(
              value,
              "telemetry_publish_port",
              14552));

  if (has_value(
          value,
          "remote_address")) {
    link.remote_address =
        string_or(
            value,
            "remote_address");
  }

  if (const auto port =
          optional_u16(
              value,
              "remote_port")) {
    link.remote_port =
        *port;
  }

  link.heartbeat =
      ms_or(
          value,
          "heartbeat_ms",
          250);

  link.lease_timeout =
      ms_or(
          value,
          "lease_timeout_ms",
          750);

  return link;
}

VideoConfig parse_video(
    const Json& value) {
  VideoConfig video;

  video.path =
      string_or(
          value,
          "path",
          video.path);

  video.device =
      string_or(
          value,
          "device",
          video.device);

  video.target_address =
      string_or(
          value,
          "target_address",
          video.target_address);

  video.camera1_device =
      string_or(
          value,
          "camera1_device",
          string_or(
              value,
              "device",
              video.camera1_device));

  video.camera2_device =
      string_or(
          value,
          "camera2_device",
          video.camera2_device);

  video.camera1_udp_port =
      static_cast<std::uint16_t>(
          int_or(
              value,
              "camera1_udp_port",
              video.camera1_udp_port));

  video.camera2_udp_port =
      static_cast<std::uint16_t>(
          int_or(
              value,
              "camera2_udp_port",
              video.camera2_udp_port));

  video.camera1_play_url =
      string_or(
          value,
          "camera1_play_url",
          video.camera1_play_url);

  video.camera2_play_url =
      string_or(
          value,
          "camera2_play_url",
          video.camera2_play_url);

  video.width =
      int_or(
          value,
          "width",
          video.width);

  video.height =
      int_or(
          value,
          "height",
          video.height);

  video.fps =
      int_or(
          value,
          "fps",
          video.fps);

  video.bps =
      int_or(
          value,
          "bps",
          video.bps);

  video.gop =
      int_or(
          value,
          "gop",
          video.gop);

  video.qp_init =
      int_or(
          value,
          "qp_init",
          video.qp_init);

  video.auto_start_when_target_known =
      bool_or(
          value,
          "auto_start_when_target_known",
          video.auto_start_when_target_known);

  video.cloud_export =
      bool_or(
          value,
          "cloud_export",
          video.cloud_export);

  video.force_usb_host_mode =
      bool_or(
          value,
          "force_usb_host_mode",
          video.force_usb_host_mode);

  return video;
}

WatchdogConfig parse_watchdog(
    const Json& value) {
  WatchdogConfig watchdog;

  watchdog.enabled =
      bool_or(
          value,
          "enabled",
          false);

  watchdog.device_path =
      string_or(
          value,
          "device_path",
          watchdog.device_path);

  watchdog.timeout_sec =
      int_or(
          value,
          "timeout_sec",
          watchdog.timeout_sec);

  watchdog.feed_interval =
      ms_or(
          value,
          "feed_interval_ms",
          static_cast<int>(
              watchdog.feed_interval.count()));

  watchdog.stop_on_clean_shutdown =
      bool_or(
          value,
          "stop_on_clean_shutdown",
          true);

  watchdog.fail_if_unavailable =
      bool_or(
          value,
          "fail_if_unavailable",
          false);

  return watchdog;
}

/* ============================================================
 * 总线关系辅助
 * ============================================================ */

bool is_production(
    const HardwareConfig& config) {
  return config.bringup.mode ==
         RuntimeMode::kProduction;
}

bool uses_rs485(
    const HardwareConfig& config,
    const DeviceConfig& device) {
  return config.rs485_ports.count(
             device.bus_id) != 0;
}

bool uses_can(
    const HardwareConfig& config,
    const DeviceConfig& device) {
  return config.can_buses.count(
             device.bus_id) != 0;
}

/* ============================================================
 * 生产设备和继电器检查
 * ============================================================ */

drivers::Result
validate_required_relay_output(
    const HardwareConfig& config,
    const char* output_id) {
  const auto it =
      config.relay_outputs.find(
          output_id);

  if (it ==
      config.relay_outputs.end()) {
    return config_error(
        std::string(
            "production config missing relay output: ") +
        output_id);
  }

  if (it->second.controller_id.empty() ||
      it->second.physical_channel == 0) {
    return config_error(
        std::string(
            "invalid production relay output: ") +
        output_id);
  }

  return drivers::Result::success();
}

drivers::Result
validate_required_production_devices(
    const HardwareConfig& config) {
  static constexpr std::array<
      const char*,
      14>
      kRequiredDevices = {
          "lidar_brush_left",
          "lidar_brush_right",
          "lidar_body_front",
          "lidar_body_rear",
          "anemometer_left",
          "anemometer_right",
          "wit_motion",
          "temperature_cabinet",
          "power_24v",
          "power_48v",
          "relay_push_motor",
          "brush_motor",
          "climber",
          "propulsion_fan"};

  for (const char* device_id :
       kRequiredDevices) {
    const auto it =
        config.devices.find(
            device_id);

    if (it ==
        config.devices.end()) {
      return config_error(
          std::string(
              "production config missing device: ") +
          device_id);
    }

    if (!it->second.enabled) {
      return config_error(
          std::string(
              "production device must be enabled: ") +
          device_id);
    }
  }

  static constexpr std::array<
      const char*,
      4>
      kRequiredRelayOutputs = {
          "brush_push_extend",
          "brush_push_retract",
          "scraper_push_extend",
          "scraper_push_retract"};

  std::map<
      std::pair<std::string, std::uint16_t>,
      std::string>
      used_channels;

  for (const char* output_id :
       kRequiredRelayOutputs) {
    const auto valid =
        validate_required_relay_output(
            config,
            output_id);

    if (!valid.ok) {
      return valid;
    }

    const auto& relay =
        config.relay_outputs.at(
            output_id);

    const auto key =
        std::make_pair(
            relay.controller_id,
            relay.physical_channel);

    const auto inserted =
        used_channels.emplace(
            key,
            output_id);

    if (!inserted.second) {
      return config_error(
          "relay output physical channels "
          "must be unique per controller: " +
          inserted.first->second +
          " and " +
          output_id);
    }
  }

  return drivers::Result::success();
}

/* ============================================================
 * 设备 profile 检查
 * ============================================================ */

bool valid_model_profile_pair(
    const DeviceConfig& device) {
  static const std::map<
      std::string,
      std::set<std::string>>
      kAllowedProfiles = {
          {
              "UBS1000-C18R-485",
              {"ubs1000_c18r"}},
          {
              "KS114",
              {"ks114"}},
          {
              "YN8588B",
              {"yn8588b"}},
          {
              "WTGAHRS02A",
              {"wit_motion"}},
          {
              "temperature_humidity_module",
              {"temperature_humidity"}},
          {
              "24V 3000W",
              {"pds3000"}},
          {
              "48V 3000W",
              {"pds3000"}},
          {
              "ZS-DIO-R-10A-4-R-485",
              {"zs_dio_r_4_relay"}},
          {
              "BD-201S",
              {"bd_201s"}},
          {
              "L7P",
              {"l7p_speed_mode"}},
          {
              "X6 Plus",
              {"x6_plus_raw_command"}}};

  const auto it =
      kAllowedProfiles.find(
          device.model);

  /*
   * 未列入表中的型号交给具体驱动和实物联调阶段处理。
   */
  if (it ==
      kAllowedProfiles.end()) {
    return true;
  }

  return it->second.count(
             device.protocol_profile) != 0;
}

/* ============================================================
 * 安全样本名称
 * ============================================================ */

std::vector<std::string>
sample_names_for_device(
    const std::string& device_id) {
  std::vector<std::string> names;

  if (device_id ==
          "lidar_brush_left" ||
      device_id ==
          "lidar_brush_right") {
    names.push_back(
        device_id +
        "_distance");

    names.push_back(
        device_id +
        "_distance_temperature");

    return names;
  }

  if (device_id ==
          "lidar_body_front" ||
      device_id ==
          "lidar_body_rear") {
    names.push_back(
        device_id +
        "_distance");

    return names;
  }

  if (device_id ==
          "anemometer_left" ||
      device_id ==
          "anemometer_right") {
    names.push_back(
        device_id +
        "_speed");

    return names;
  }

  if (device_id ==
      "wit_motion") {
    return {
        "wit_motion_accel_x",
        "wit_motion_accel_y",
        "wit_motion_accel_z",
        "wit_motion_gyro_x",
        "wit_motion_gyro_y",
        "wit_motion_gyro_z",
        "wit_motion_roll",
        "wit_motion_pitch",
        "wit_motion_yaw",
        "wit_motion_temperature"};
  }

  if (device_id ==
      "temperature_cabinet") {
    return {
        "temperature_cabinet_temperature",
        "temperature_cabinet_humidity"};
  }

  if (device_id ==
          "power_24v" ||
      device_id ==
          "power_48v") {
    return {
        device_id + "_voltage",
        device_id + "_current",
        device_id + "_power",
        device_id + "_temperature",
        device_id + "_operation_enabled"};
  }

  return names;
}

std::set<std::string>
available_sample_names(
    const HardwareConfig& config) {
  std::set<std::string> names;

  for (const auto& item :
       config.devices) {
    if (!item.second.enabled) {
      continue;
    }

    const auto device_names =
        sample_names_for_device(
            item.first);

    names.insert(
        device_names.begin(),
        device_names.end());
  }

  return names;
}

drivers::Result
validate_required_samples(
    const HardwareConfig& config) {
  std::set<std::string> used_samples;

  if (is_production(config) &&
      config.safety.required_samples.empty()) {
    return config_error(
        "production config requires explicit "
        "safety.required_samples");
  }

  const auto available =
      available_sample_names(config);

  for (const auto& sample_name :
       config.safety.required_samples) {
    if (sample_name.empty()) {
      return config_error(
          "required sensor sample name must not be empty");
    }

    const auto inserted =
        used_samples.insert(
            sample_name);

    if (!inserted.second) {
      return config_error(
          "required sensor sample duplicated: " +
          sample_name);
    }

    if (available.count(
            sample_name) == 0) {
      return config_error(
          "required sensor sample is not produced "
          "by an enabled sensor: " +
          sample_name);
    }
  }

  return drivers::Result::success();
}

/* ============================================================
 * 设备时序检查
 * ============================================================ */

drivers::Result
validate_device_timing(
    const DeviceConfig& device) {
  if (device.response_deadline.count() <= 0) {
    return config_error(
        "device response_deadline_ms must be "
        "positive: " +
        device.device_id);
  }

  if (device.poll_interval.count() < 0) {
    return config_error(
        "device poll_interval_ms must not be "
        "negative: " +
        device.device_id);
  }

  if (is_sensor_like(device) &&
      device.poll_interval.count() <= 0) {
    return config_error(
        "sensor device poll_interval_ms must "
        "be positive: " +
        device.device_id);
  }

  if (is_sensor_like(device) &&
      device.stale_after.count() <= 0) {
    return config_error(
        "sensor device stale_after_ms must "
        "be positive: " +
        device.device_id);
  }

  if (device.priority < 0) {
    return config_error(
        "device priority must not be negative: " +
        device.device_id);
  }

  if (device.max_consecutive_failures <= 0) {
    return config_error(
        "device max_consecutive_failures must "
        "be positive: " +
        device.device_id);
  }

  if (device.quarantine_duration.count() < 0) {
    return config_error(
        "device quarantine_duration_ms must "
        "not be negative: " +
        device.device_id);
  }

  if (device.read_retry_count < 0) {
    return config_error(
        "device read_retry_count must not be "
        "negative: " +
        device.device_id);
  }

  return drivers::Result::success();
}

drivers::Result
validate_production_explicit_timing(
    const Json& root,
    const HardwareConfig& config) {
  if (!is_production(config)) {
    return drivers::Result::success();
  }

  if (!has_value(
          root,
          "devices") ||
      !root.at("devices").is_object()) {
    return config_error(
        "production config requires devices object");
  }

  const auto& devices =
      root.at("devices");

  for (const auto& item :
       config.devices) {
    const auto& device =
        item.second;

    if (!device.enabled) {
      continue;
    }

    const auto raw_it =
        devices.find(
            item.first);

    if (raw_it ==
            devices.end() ||
        !raw_it.value().is_object()) {
      return config_error(
          "production device JSON object missing: " +
          item.first);
    }

    const auto& raw =
        raw_it.value();

    static constexpr std::array<
        const char*,
        3>
        kRequiredCommonTiming = {
            "response_deadline_ms",
            "max_consecutive_failures",
            "quarantine_duration_ms"};

    for (const char* key :
         kRequiredCommonTiming) {
      if (!has_value(
              raw,
              key)) {
        return config_error(
            "production device " +
            item.first +
            " missing explicit field: " +
            key);
      }
    }

    if (uses_rs485(
            config,
            device) &&
        !has_value(
            raw,
            "read_retry_count")) {
      return config_error(
          "production rs485 device " +
          item.first +
          " missing explicit field: "
          "read_retry_count");
    }

    if (is_sensor_like(device)) {
      static constexpr std::array<
          const char*,
          2>
          kRequiredSensorTiming = {
              "poll_interval_ms",
              "stale_after_ms"};

      for (const char* key :
           kRequiredSensorTiming) {
        if (!has_value(
                raw,
                key)) {
          return config_error(
              "production sensor device " +
              item.first +
              " missing explicit field: " +
              key);
        }
      }
    }
  }

  return drivers::Result::success();
}

/* ============================================================
 * WCH 共享事务组检查
 * ============================================================ */

drivers::Result
validate_wch_transaction_groups(
    const HardwareConfig& config) {
  std::optional<std::string>
      wch_group;

  for (const auto& item :
       config.rs485_ports) {
    const auto& port =
        item.second;

    if (!is_wch_acm_path(
            port.device_path)) {
      continue;
    }

    if (port.shared_transaction_group.empty()) {
      return config_error(
          "WCH ACM port must define "
          "shared_transaction_group: " +
          item.first);
    }

    if (!wch_group) {
      wch_group =
          port.shared_transaction_group;
      continue;
    }

    if (*wch_group !=
        port.shared_transaction_group) {
      return config_error(
          "all WCH ACM0~ACM3 ports must use "
          "one shared_transaction_group: " +
          item.first);
    }
  }

  return drivers::Result::success();
}

/* ============================================================
 * 继电器输出检查
 * ============================================================ */

drivers::Result
validate_relay_outputs(
    const HardwareConfig& config) {
  std::map<
      std::pair<std::string, std::uint16_t>,
      std::string>
      used_channels;

  for (const auto& item :
       config.relay_outputs) {
    const auto& output =
        item.second;

    if (output.controller_id.empty()) {
      return config_error(
          "relay output missing controller_id: " +
          item.first);
    }

    if (output.physical_channel == 0) {
      return config_error(
          "relay output physical_channel must "
          "be positive: " +
          item.first);
    }

    const auto device_it =
        config.devices.find(
            output.controller_id);

    if (device_it ==
        config.devices.end()) {
      return config_error(
          "relay output references unknown "
          "controller: " +
          item.first);
    }

    if (!device_it->second.enabled) {
      return config_error(
          "relay output controller is disabled: " +
          item.first);
    }

    if (device_it->second.channel_count &&
        output.physical_channel >
            *device_it->second.channel_count) {
      return config_error(
          "relay output channel exceeds "
          "controller channel_count: " +
          item.first);
    }

    const auto key =
        std::make_pair(
            output.controller_id,
            output.physical_channel);

    const auto inserted =
        used_channels.emplace(
            key,
            item.first);

    if (!inserted.second) {
      return config_error(
          "relay output physical channel duplicated: " +
          inserted.first->second +
          " and " +
          item.first);
    }
  }

  return drivers::Result::success();
}

/* ============================================================
 * 总配置校验
 * ============================================================ */

drivers::Result
validate_config(
    const Json& root,
    const HardwareConfig& config) {
  if (config.schema_version != 1) {
    return config_error(
        "hardware config schema_version must be 1");
  }

  if (config.template_scope !=
      "snappy_real") {
    return config_error(
        "hardware config template_scope must be "
        "snappy_real");
  }

  if (config.robot_id.empty()) {
    return config_error(
        "hardware config robot_id is required");
  }

  const bool production =
      is_production(config);

  if (production &&
      (config.bringup.allow_partial_devices ||
       config.bringup.noop_missing_actuators ||
       config.bringup.ignore_device_startup_failures ||
       config.bringup.quarantine_failed_devices)) {
    return config_error(
        "partial/noop/startup-ignore/quarantine "
        "options are only allowed in commissioning "
        "mode");
  }

  if (production &&
      !config.bringup.arm_requires_all_required_devices) {
    return config_error(
        "production config must require all required "
        "devices before arm");
  }

  if (config.bringup.quarantine_after_failures <= 0) {
    return config_error(
        "quarantine_after_failures must be positive");
  }

  if (config.bringup.quarantine_duration.count() < 0) {
    return config_error(
        "quarantine_duration_ms must not be negative");
  }

  if (config.safety.g20_lease_timeout.count() <= 0) {
    return config_error(
        "g20_lease_timeout_ms must be positive");
  }

  if (config.safety.sensor_stale_timeout.count() <= 0) {
    return config_error(
        "sensor_stale_timeout_ms must be positive");
  }

  if (config.safety.rs485_retry_count < 0) {
    return config_error(
        "rs485_retry_count must not be negative");
  }

  if (config.safety.push_motor_dead_time.count() < 0 ||
      config.safety.push_motor_max_on_time.count() <= 0 ||
      config.safety.brush_push_motor_max_on_time.count() <= 0 ||
      config.safety.scraper_push_motor_max_on_time.count() <= 0) {
    return config_error(
        "push motor timing values are invalid");
  }

  const auto required_samples =
      validate_required_samples(
          config);

  if (!required_samples.ok) {
    return required_samples;
  }

  if (production) {
    if (config.can_buses.empty()) {
      return config_error(
          "production config requires at least "
          "one CAN bus");
    }

    if (config.rs485_ports.empty()) {
      return config_error(
          "production config requires at least "
          "one RS485 port");
    }

    const auto required =
        validate_required_production_devices(
            config);

    if (!required.ok) {
      return required;
    }
  } else if (config.devices.empty()) {
    return config_error(
        "commissioning config requires devices object");
  }

  /* ==========================================================
   * RS485 物理端口检查
   * ========================================================== */

  std::set<std::string>
      rs485_paths;

  for (const auto& item :
       config.rs485_ports) {
    const auto& port =
        item.second;

    if (port.port_id.empty()) {
      return config_error(
          "RS485 port id is required: " +
          item.first);
    }

    if (!valid_rs485_backend(
            port.backend)) {
      return config_error(
          "unsupported RS485 backend: " +
          item.first +
          " backend=" +
          port.backend);
    }

    if (port.device_path.empty()) {
      return config_error(
          "RS485 port missing device_path: " +
          item.first);
    }

    if (port.baud_rate <= 0) {
      return config_error(
          "RS485 baud_rate must be positive: " +
          item.first);
    }

    if (port.format.data_bits < 5 ||
        port.format.data_bits > 8) {
      return config_error(
          "RS485 data_bits must be between 5 and 8: " +
          item.first);
    }

    if (port.format.stop_bits != 1 &&
        port.format.stop_bits != 2) {
      return config_error(
          "RS485 stop_bits must be 1 or 2: " +
          item.first);
    }

    if (port.inter_frame_gap.count() < 0 ||
        port.read_idle_gap.count() <= 0) {
      return config_error(
          "RS485 frame gaps are invalid: " +
          item.first);
    }

    if (port.late_frame_silence.count() <= 0) {
      return config_error(
          "RS485 late_frame_silence_ms must be positive: " +
          item.first);
    }

    if (port.late_frame_recovery_timeout.count() <
        port.late_frame_silence.count()) {
      return config_error(
          "RS485 late frame recovery timeout must "
          "not be shorter than silence interval: " +
          item.first);
    }

    if (port.max_pending_transactions == 0) {
      return config_error(
          "RS485 max_pending_transactions must be "
          "positive: " +
          item.first);
    }

    if (port.recovery_delay.count() < 0) {
      return config_error(
          "RS485 recovery_delay_ms must not be negative: " +
          item.first);
    }

    if (!rs485_paths.insert(
            port.device_path)
             .second) {
      return config_error(
          "RS485 device_path is used by more than "
          "one port: " +
          port.device_path);
    }

    const auto raw_it =
        root.contains("rs485_ports") &&
                root.at("rs485_ports").is_object()
            ? root.at("rs485_ports").find(
                  item.first)
            : Json::const_iterator{};

    if (root.contains("rs485_ports") &&
        root.at("rs485_ports").is_object() &&
        raw_it !=
            root.at("rs485_ports").end() &&
        raw_it.value().is_object()) {
      const auto& raw =
          raw_it.value();

      if (has_value(
              raw,
              "parity") &&
          !valid_parity_name(
              string_or(
                  raw,
                  "parity"))) {
        return config_error(
            "invalid RS485 parity: " +
            item.first);
      }

      if (has_value(
              raw,
              "rs485_direction_mode") &&
          !valid_direction_mode_name(
              string_or(
                  raw,
                  "rs485_direction_mode"))) {
        return config_error(
            "invalid rs485_direction_mode: " +
            item.first);
      }
    }
  }

  const auto wch_groups =
      validate_wch_transaction_groups(
          config);

  if (!wch_groups.ok) {
    return wch_groups;
  }

  /* ==========================================================
   * CAN 总线检查
   * ========================================================== */

  for (const auto& item :
       config.can_buses) {
    const auto& bus =
        item.second;

    if (bus.bus_id.empty()) {
      return config_error(
          "CAN bus id is required: " +
          item.first);
    }

    if (!valid_can_backend(
            bus.backend)) {
      return config_error(
          "unsupported CAN backend: " +
          item.first +
          " backend=" +
          bus.backend);
    }

    if (bus.interface_name.empty()) {
      return config_error(
          "CAN bus missing interface_name: " +
          item.first);
    }

    if (bus.bitrate == 0) {
      return config_error(
          "CAN bus bitrate must be non-zero: " +
          item.first);
    }

    if (bus.tx_queue_length == 0) {
      return config_error(
          "CAN tx_queue_length must be non-zero: " +
          item.first);
    }

    if (bus.restart_ms.count() < 0) {
      return config_error(
          "CAN restart_ms must not be negative: " +
          item.first);
    }

    if (bus.default_tx_timeout.count() <= 0 ||
        bus.default_rx_timeout.count() <= 0) {
      return config_error(
          "CAN default timeouts must be positive: " +
          item.first);
    }

    if (bus.allow_start_failure &&
        production) {
      return config_error(
          "CAN allow_start_failure is only allowed "
          "in commissioning mode: " +
          item.first);
    }
  }

  /* ==========================================================
   * G20、视频和看门狗检查
   * ========================================================== */

  if (config.g20_udp.listen_port == 0 ||
      config.g20_udp.command_listen_port == 0 ||
      config.g20_udp.status_publish_port == 0 ||
      config.g20_udp.telemetry_publish_port == 0) {
    return config_error(
        "G20 UDP ports must be non-zero");
  }

  if (config.g20_udp.heartbeat.count() <= 0 ||
      config.g20_udp.lease_timeout.count() <= 0) {
    return config_error(
        "G20 heartbeat and lease timeout must be positive");
  }

  if (config.video.width <= 0 ||
      config.video.height <= 0 ||
      config.video.fps <= 0 ||
      config.video.bps <= 0 ||
      config.video.gop <= 0) {
    return config_error(
        "video dimensions, fps, bitrate and gop "
        "must be positive");
  }

  if (config.watchdog.enabled) {
    if (config.watchdog.device_path.empty()) {
      return config_error(
          "watchdog device_path is required when enabled");
    }

    if (config.watchdog.timeout_sec <= 0) {
      return config_error(
          "watchdog timeout_sec must be positive when enabled");
    }

    if (config.watchdog.feed_interval.count() <= 0) {
      return config_error(
          "watchdog feed_interval_ms must be positive when enabled");
    }

    if (config.watchdog.feed_interval.count() >=
        static_cast<std::int64_t>(
            config.watchdog.timeout_sec) *
            1000) {
      return config_error(
          "watchdog feed interval must be shorter "
          "than watchdog timeout");
    }
  }

  const auto relay_valid =
      validate_relay_outputs(
          config);

  if (!relay_valid.ok) {
    return relay_valid;
  }

  /* ==========================================================
   * 设备和站号检查
   * ========================================================== */

  bool any_enabled_device =
      false;

  std::map<
      std::pair<std::string, std::uint8_t>,
      std::string>
      rs485_addresses;

  for (const auto& item :
       config.devices) {
    const auto& device =
        item.second;

    if (device.device_id !=
        item.first) {
      return config_error(
          "device id does not match map key: " +
          item.first);
    }

    if (!device.enabled) {
      continue;
    }

    any_enabled_device =
        true;

    if (!valid_device_backend(
            device.backend)) {
      return config_error(
          "unsupported device backend: " +
          item.first +
          " backend=" +
          device.backend);
    }

    if (device.backend !=
        "real") {
      return config_error(
          "enabled device must use real backend: " +
          item.first);
    }

    if (device.bus_id.empty()) {
      return config_error(
          "enabled device missing bus_id: " +
          item.first);
    }

    const bool on_rs485 =
        uses_rs485(
            config,
            device);

    const bool on_can =
        uses_can(
            config,
            device);

    if (on_rs485 == on_can) {
      return config_error(
          "device must reference exactly one bus "
          "type: " +
          item.first);
    }

    if (on_rs485) {
      if (!device.station_id) {
        return config_error(
            "RS485 device missing station_id: " +
            item.first);
      }

      if (*device.station_id == 0 ||
          *device.station_id == 0xFF) {
        return config_error(
            "RS485 station_id is invalid: " +
            item.first);
      }

      const auto address_key =
          std::make_pair(
              device.bus_id,
              *device.station_id);

      const auto inserted =
          rs485_addresses.emplace(
              address_key,
              device.device_id);

      if (!inserted.second) {
        return config_error(
            "RS485 station_id duplicated on bus " +
            device.bus_id +
            ": " +
            inserted.first->second +
            " and " +
            device.device_id);
      }

      if (device.protocol_family.empty() ||
          device.protocol_profile.empty()) {
        return config_error(
            "RS485 device protocol fields are "
            "required: " +
            item.first);
      }
    }

    if (on_can) {
      if (device.protocol_family.empty() ||
          device.protocol_profile.empty()) {
        return config_error(
            "CAN device protocol fields are required: " +
            item.first);
      }

      if (device.protocol_profile ==
          "x6_plus_raw_command") {
        if (!device.source_node_id ||
            !device.left_node_id ||
            !device.right_node_id ||
            !device.left_throttle_channel ||
            !device.right_throttle_channel ||
            !device.max_throttle_raw) {
          return config_error(
              "propulsion fan CAN device missing "
              "node/channel fields: " +
              item.first);
        }

        if (*device.source_node_id == 0 ||
            *device.source_node_id > 0x7F ||
            *device.left_node_id == 0 ||
            *device.left_node_id > 0x7D ||
            *device.right_node_id == 0 ||
            *device.right_node_id > 0x7D) {
          return config_error(
              "propulsion fan CAN node id is invalid: " +
              item.first);
        }

        if (*device.left_throttle_channel >= 4 ||
            *device.right_throttle_channel >= 4 ||
            *device.left_throttle_channel ==
                *device.right_throttle_channel) {
          return config_error(
              "propulsion fan CAN throttle channels "
              "are invalid: " +
              item.first);
        }

        if (*device.max_throttle_raw == 0 ||
            *device.max_throttle_raw > 8191) {
          return config_error(
              "propulsion fan CAN max_throttle_raw is "
              "invalid: " +
              item.first);
        }
      }
    }

    if (!valid_model_profile_pair(
            device)) {
      return config_error(
          "device model/protocol_profile mismatch: " +
          item.first);
    }

    const auto timing =
        validate_device_timing(
            device);

    if (!timing.ok) {
      return timing;
    }
  }

  const auto explicit_timing =
      validate_production_explicit_timing(
          root,
          config);

  if (!explicit_timing.ok) {
    return explicit_timing;
  }

  if (!production &&
      !any_enabled_device &&
      !config.bringup.allow_partial_devices) {
    return config_error(
        "commissioning config with no enabled devices "
        "requires allow_partial_devices=true");
  }

  return drivers::Result::success(
      "hardware config validated");
}

}  // namespace

/* ============================================================
 * 对外 ID 解析函数
 * ============================================================ */

std::optional<std::uint32_t>
parse_u32_id(
    const std::string& value) {
  return parse_decimal_or_hex_u32(
      value);
}

std::optional<std::uint8_t>
parse_u8_id(
    const std::string& value) {
  const auto raw =
      parse_decimal_or_hex_u32(
          value);

  if (!raw ||
      *raw >
          std::numeric_limits<
              std::uint8_t>::max()) {
    return std::nullopt;
  }

  return static_cast<std::uint8_t>(
      *raw);
}

/* ============================================================
 * 配置文件加载
 * ============================================================ */

HardwareConfigLoadResult
load_hardware_config(
    const std::string& path) {
  std::ifstream input(
      path);

  if (!input) {
    return {
        config_error(
            "cannot open hardware config: " +
            path),
        {}};
  }

  Json root;

  try {
    input >> root;
  } catch (const std::exception& error) {
    return {
        config_error(
            std::string(
                "invalid hardware config json: ") +
            error.what()),
        {}};
  }

  try {
    if (!root.is_object()) {
      return {
          config_error(
              "hardware config root must be an object"),
          {}};
    }

    HardwareConfig config;

    config.schema_version =
        int_or(
            root,
            "schema_version",
            1);

    config.template_scope =
        string_or(
            root,
            "template_scope");

    config.robot_id =
        string_or(
            root,
            "robot_id");

    if (has_value(
            root,
            "bringup")) {
      config.bringup =
          parse_bringup(
              root.at(
                  "bringup"));
    }

    if (has_value(
            root,
            "can_buses")) {
      if (!root.at(
              "can_buses")
               .is_object()) {
        return {
            config_error(
                "can_buses must be an object"),
            {}};
      }

      for (const auto& item :
           root.at(
                   "can_buses")
               .items()) {
        config.can_buses[
            item.key()] =
            parse_can_bus(
                item.key(),
                item.value());
      }
    }

    if (has_value(
            root,
            "rs485_ports")) {
      if (!root.at(
              "rs485_ports")
               .is_object()) {
        return {
            config_error(
                "rs485_ports must be an object"),
            {}};
      }

      for (const auto& item :
           root.at(
                   "rs485_ports")
               .items()) {
        config.rs485_ports[
            item.key()] =
            parse_rs485_port(
                item.key(),
                item.value());
      }
    }

    if (has_value(
            root,
            "devices")) {
      if (!root.at(
              "devices")
               .is_object()) {
        return {
            config_error(
                "devices must be an object"),
            {}};
      }

      for (const auto& item :
           root.at(
                   "devices")
               .items()) {
        config.devices[
            item.key()] =
            parse_device(
                item.key(),
                item.value());
      }
    }

    if (has_value(
            root,
            "relay_outputs")) {
      if (!root.at(
              "relay_outputs")
               .is_object()) {
        return {
            config_error(
                "relay_outputs must be an object"),
            {}};
      }

      for (const auto& item :
           root.at(
                   "relay_outputs")
               .items()) {
        RelayOutputConfig relay;

        relay.controller_id =
            string_or(
                item.value(),
                "controller_id");

        relay.physical_channel =
            static_cast<std::uint16_t>(
                int_or(
                    item.value(),
                    "physical_channel",
                    0));

        config.relay_outputs[
            item.key()] =
            relay;
      }
    }

    if (has_value(
            root,
            "safety")) {
      config.safety =
          parse_safety(
              root.at(
                  "safety"));
    }

    if (has_value(
            root,
            "links") &&
        has_value(
            root.at(
                "links"),
            "g20_udp")) {
      config.g20_udp =
          parse_g20_link(
              root.at(
                      "links")
                  .at(
                      "g20_udp"));
    }

    if (has_value(
            root,
            "video")) {
      config.video =
          parse_video(
              root.at(
                  "video"));
    }

    if (has_value(
            root,
            "watchdog")) {
      config.watchdog =
          parse_watchdog(
              root.at(
                  "watchdog"));
    }

    const auto valid =
        validate_config(
            root,
            config);

    if (!valid.ok) {
      return {
          valid,
          {}};
    }

    return {
        drivers::Result::success(
            "hardware config loaded"),
        std::move(config)};
  } catch (const std::exception& error) {
    return {
        config_error(
            std::string(
                "hardware config field type error: ") +
            error.what()),
        {}};
  }
}

}  // namespace snappy::control