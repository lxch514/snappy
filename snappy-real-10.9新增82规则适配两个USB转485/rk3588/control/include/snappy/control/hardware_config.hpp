#pragma once

/**
 * @file hardware_config.hpp
 * @brief RK3588 简易机硬件配置结构
 *
 * @details
 * - 保存 CAN、RS485、设备、继电器、安全、视频和看门狗配置。
 * - 每个设备都有独立的轮询周期、响应截止时间和缓存失效时间。
 * - 每个 RS485 物理端口都有独立的事务恢复和队列配置。
 * - 正式模式不允许通过调试选项伪造硬件 Ready。
 * - 配置加载阶段完成端口、站号、协议和安全样本校验。
 */

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "snappy/drivers/bus/can_bus.hpp"
#include "snappy/drivers/bus/rs485_bus.hpp"
#include "snappy/drivers/result.hpp"

namespace snappy::control {

/* ============================================================
 * 运行模式
 * ============================================================ */

enum class RuntimeMode {
  kProduction,
  kCommissioning,
};

/* ============================================================
 * 启动和调试配置
 * ============================================================ */

struct BringupConfig {
  RuntimeMode mode{
      RuntimeMode::kProduction};

  /*
   * 允许单个硬件后端启动失败后继续运行。
   *
   * 该选项只控制整机启动编排，不会伪造设备 Ready，
   * 也不会把缺少的执行器替换成空操作对象。
   */
  bool allow_partial_devices{
      false};

  /*
   * 缺少执行器时使用空操作对象。
   * 只允许 commissioning/debug 模式启用。
   */
  bool noop_missing_actuators{
      false};

  /*
   * 忽略单个设备启动失败。
   * 只允许 commissioning/debug 模式启用。
   */
  bool ignore_device_startup_failures{
      false};

  /*
   * 连续通信失败后隔离设备。
   * 只允许 commissioning/debug 模式启用。
   */
  bool quarantine_failed_devices{
      false};

  int quarantine_after_failures{
      2};

  std::chrono::milliseconds
      quarantine_duration{
          2000};

  /*
   * 设备访问退避（传感器/执行器没接时的处理）。
   *
   * 上电后每个设备每轮尝试 max_consecutive_failures 次（配置为 3）；
   * 一轮全失败就把该设备隔离 device_backoff_initial_ms，
   * 下一轮再全失败按 multiplier 递增（默认 10s -> 20s -> 40s）；
   * 连续 device_backoff_offline_round 轮都失败即判定"永久离线"，
   * 后续不再访问该设备。
   *
   * 只影响被判定的那一个 logical device：
   * 同一 RS485 端口、同一共享事务门禁上的其他设备完全不受影响。
   * 恢复途径：遥控端「清故障」(clear_faults)、或重启 snappy-control；
   * 该设备若在安全停止(kSafety)路径下意外成功一次也会自动复活。
   */
  bool device_backoff_enabled{
      true};

  std::chrono::milliseconds
      device_backoff_initial{
          10000};

  int device_backoff_multiplier{
      2};

  int device_backoff_offline_round{
      4};

  std::chrono::milliseconds
      device_backoff_max{
          300000};

  /*
   * 生产模式必须为 true。
   * 取得控制权不等于允许执行危险动作。
   */
  bool arm_requires_all_required_devices{
      true};
};

/* ============================================================
 * 继电器输出配置
 * ============================================================ */

struct RelayOutputConfig {
  std::string controller_id;

  std::uint16_t physical_channel{
      0};
};

/* ============================================================
 * 逻辑设备配置
 * ============================================================ */

struct DeviceConfig {
  std::string device_id;

  bool enabled{
      false};

  /*
   * 当前真实设备使用 real。
   */
  std::string backend;

  std::string bus_id;
  std::string model;
  std::string protocol_family;
  std::string protocol_profile;
  std::string role;

  std::optional<std::uint8_t>
      station_id;

  std::optional<std::uint8_t>
      source_node_id;

  std::optional<std::uint8_t>
      left_node_id;

  std::optional<std::uint8_t>
      right_node_id;

  std::optional<std::size_t>
      left_throttle_channel;

  std::optional<std::size_t>
      right_throttle_channel;

  std::optional<std::uint16_t>
      max_throttle_raw;

  std::optional<std::uint16_t>
      channel_count;

  std::optional<std::uint32_t>
      tx_can_id;

  std::optional<std::uint32_t>
      rx_can_id;

  bool can_extended{
      false};

  /*
   * 执行器可以把轮询周期设为 0。
   * 传感器必须使用正数轮询周期。
   */
  std::chrono::milliseconds
      poll_interval{
          200};

  /*
   * 单次事务响应 deadline。
   */
  std::chrono::milliseconds
      response_deadline{
          120};

  /*
   * 最后一次成功样本允许保留的时间。
   */
  std::chrono::milliseconds
      stale_after{
          1000};

  /*
   * 数值越小，事务优先级越高。
   */
  int priority{
      10};

  /*
   * 连续协议失败达到阈值后进入设备隔离。
   */
  int max_consecutive_failures{
      3};

  std::chrono::milliseconds
      quarantine_duration{
          2000};

  /*
   * 只用于无副作用读取事务。
   * 写事务不会自动重放。
   */
  int read_retry_count{
      1};

  /*
   * 是否要求设备在启动阶段成功。
   */
  bool startup_required{
      true};

  /*
   * 失败时是否保留最后成功缓存。
   */
  bool cache_failures{
      true};
};

/* ============================================================
 * 安全配置
 * ============================================================ */

struct SafetyConfig {
  std::chrono::milliseconds
      g20_lease_timeout{
          750};

  /*
   * gateway 判定「G20 链路掉线」前允许的最大静默时长。
   *
   * 这是全系统真正生效的掉线阈值：gateway 一旦判定掉线就下发
   * control.link_down，控制端收到后立即 SAFE_HOLD。
   *
   * 控制端自己的兜底窗口由本值推导（见 control_server.cpp 的
   * G20 租约检查），从而保证「控制端窗口 > gateway 窗口」这个
   * 顺序不可能被配反。
   */
  std::chrono::milliseconds
      g20_loss_confirm{
          8000};

  /*
   * 掉载波（内核判定路由不可达）后的有界宽限窗口。
   *
   * 为什么需要它：以太网掉载波后重新自协商需要固定的 ~2.05 秒
   * （2026-09-24 实测 4531.070187 -> 4533.120194 = 2.050007s，
   *  另三次 2.050 / 2.050 / 2.052s）。这段"静默"只有 2 秒，
   * 远小于 g20_loss_confirm（8s），按静默判据本来不该停机。
   *
   * 但如果一看到 ENETUNREACH 就立即判掉线，每一次可自愈的 2 秒闪断
   * 都会升级成整机安全停止 —— 这正是现场"控制过程中反复停机"的直接原因。
   *
   * 因此：路由不可达只作为"强证据"启动计时，必须持续超过本窗口
   * 才判定掉线。真实断链仍会在 3 秒内停机（比 8 秒静默窗口更快），
   * 而 2 秒级自愈闪断被容忍。
   */
  std::chrono::milliseconds
      g20_route_loss_confirm{
          3000};

  std::chrono::milliseconds
      sensor_stale_timeout{
          1000};

  /*
   * 兼容旧配置的全局读取重试值。
   * 新代码优先使用 DeviceConfig::read_retry_count。
   */
  int rs485_retry_count{
      1};

  /*
   * 生产 arm 门禁使用的必需样本名。
   *
   * 生产模式必须显式配置。
   * commissioning/debug 模式允许为空。
   */
  std::vector<std::string>
      required_samples;

  std::chrono::milliseconds
      push_motor_dead_time{
          200};

  std::chrono::milliseconds
      push_motor_max_on_time{
          10000};

  std::chrono::milliseconds
      brush_push_motor_max_on_time{
          9100};

  std::chrono::milliseconds
      scraper_push_motor_max_on_time{
          10000};

  bool safe_stop_on_g20_loss{
      true};

  bool safe_stop_on_rs485_bus_fault{
      true};

  bool safe_stop_on_can_bus_fault{
      true};

  bool safe_stop_on_sensor_stale{
      true};

  bool allow_remote_fault_clear{
      true};
};

/* ============================================================
 * G20 链路配置
 * ============================================================ */

struct G20LinkConfig {
  std::string backend{
      "udp"};

  std::string listen_address{
      "0.0.0.0"};

  std::uint16_t listen_port{
      14550};

  std::uint16_t command_listen_port{
      14550};

  std::uint16_t status_publish_port{
      14551};

  std::uint16_t telemetry_publish_port{
      14552};

  std::optional<std::string>
      remote_address;

  std::optional<std::uint16_t>
      remote_port;

  std::chrono::milliseconds
      heartbeat{
          250};

  std::chrono::milliseconds
      lease_timeout{
          750};
};

/* ============================================================
 * 视频配置
 * ============================================================ */

struct VideoConfig {
  std::string path{
      "rk_usb_camera_dual_rtp_udp_to_g20_android"};

  std::string device{
      "/dev/video81"};

  std::string target_address{
      "192.168.144.11"};

  std::string camera1_device{
      "/dev/video81"};

  std::string camera2_device{
      "/dev/video82"};

  std::uint16_t camera1_udp_port{
      5600};

  std::uint16_t camera2_udp_port{
      5602};

  std::string camera1_play_url{
      "udp://@:5600"};

  std::string camera2_play_url{
      "udp://@:5602"};

  int width{
      640};

  int height{
      480};

  int fps{
      30};

  int bps{
      4000000};

  int gop{
      15};

  int qp_init{
      24};

  bool auto_start_when_target_known{
      false};

  bool cloud_export{
      false};

  /*
   * RK3588 USB 控制器切换到 host 后，
   * 再启动相机推流。
   */
  bool force_usb_host_mode{
      true};
};

/* ============================================================
 * 看门狗配置
 * ============================================================ */

struct WatchdogConfig {
  bool enabled{
      false};

  std::string device_path{
      "/dev/watchdog"};

  int timeout_sec{
      30};

  std::chrono::milliseconds
      feed_interval{
          10000};

  bool stop_on_clean_shutdown{
      true};

  bool fail_if_unavailable{
      false};
};

/* ============================================================
 * 总硬件配置
 * ============================================================ */

struct HardwareConfig {
  int schema_version{
      1};

  std::string template_scope;

  std::string robot_id;

  BringupConfig bringup;

  std::map<
      std::string,
      drivers::bus::CanBusConfig>
      can_buses;

  std::map<
      std::string,
      drivers::bus::Rs485PortConfig>
      rs485_ports;

  std::map<
      std::string,
      DeviceConfig>
      devices;

  std::map<
      std::string,
      RelayOutputConfig>
      relay_outputs;

  SafetyConfig safety;

  G20LinkConfig g20_udp;

  VideoConfig video;

  WatchdogConfig watchdog;
};

/* ============================================================
 * 配置加载结果
 * ============================================================ */

struct HardwareConfigLoadResult {
  drivers::Result result;
  HardwareConfig config;
};

[[nodiscard]] HardwareConfigLoadResult
load_hardware_config(
    const std::string& path);

[[nodiscard]] std::optional<std::uint32_t>
parse_u32_id(
    const std::string& value);

[[nodiscard]] std::optional<std::uint8_t>
parse_u8_id(
    const std::string& value);

}  // namespace snappy::control