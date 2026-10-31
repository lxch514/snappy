#pragma once

/**
 * @file sensor_service.hpp
 * @brief 传感器后台采集、缓存快照和通信健康管理
 *
 * @details
 *
 * - 每条物理总线使用独立采集线程。
 * - 同一物理总线中的设备按各自轮询周期串行读取。
 * - 不同物理总线可以并行调度。
 * - WCH ACM0~3 的事务串行由下层 RS485 调度器统一保证。
 * - 所有硬件读取均在缓存锁之外执行。
 * - read_all()、cached_samples() 和 device_statuses() 只读取内存。
 * - 单个设备失败只更新本设备缓存和通信状态。
 * - 最后成功值会保留，超过 stale_after 后才标记为 STALE。
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "snappy/drivers/devices/anemometer.hpp"
#include "snappy/drivers/devices/lidar.hpp"
#include "snappy/drivers/devices/power.hpp"
#include "snappy/drivers/devices/temperature.hpp"
#include "snappy/drivers/devices/wit_motion.hpp"
#include "snappy/drivers/interfaces.hpp"

namespace snappy::control {

/* ============================================================
 * SensorService 使用的设备对象
 * ============================================================ */

struct SensorServiceDevices {
  drivers::devices::DistanceSensor* lidar_brush_left{nullptr};
  drivers::devices::DistanceSensor* lidar_brush_right{nullptr};
  drivers::devices::DistanceSensor* lidar_body_front{nullptr};
  drivers::devices::DistanceSensor* lidar_body_rear{nullptr};

  drivers::devices::Anemometer* anemometer_left{nullptr};
  drivers::devices::Anemometer* anemometer_right{nullptr};

  drivers::devices::WitMotion* wit_motion{nullptr};
  drivers::devices::TemperatureSensor* temperature_cabinet{nullptr};

  drivers::devices::PowerMonitor* power_24v{nullptr};
  drivers::devices::PowerMonitor* power_48v{nullptr};
};

/* ============================================================
 * 单设备轮询配置
 * ============================================================ */

struct SensorPollConfig {
  bool enabled{true};

  /*
   * 物理总线逻辑名。
   *
   * 相同 bus_id 的设备由同一个 SensorService 线程轮询。
   * 底层 RS485 调度器仍负责端口事务队列和 WCH 共享门禁。
   */
  std::string bus_id;

  std::chrono::milliseconds poll_interval{200};
  std::chrono::milliseconds stale_after{1000};

  /*
   * 连续失败阈值只用于 SensorService 的通信健康判断。
   * 样本值质量仍由 last_success_at 和 stale_after 独立决定。
   */
  int max_consecutive_failures{3};
};

/* ============================================================
 * SensorService 总体配置
 * ============================================================ */

struct SensorServiceOptions {
  /*
   * 调试模式为 true：
   * 单个传感器启动失败时，保留失败状态并继续启动其他设备。
   */
  bool ignore_startup_failures{false};

  // true 时创建后台采集线程。
  bool background_polling{true};

  // 后台线程检查设备到期状态的最小周期。
  std::chrono::milliseconds scheduler_period{20};

  // 没有单独配置设备时使用的兼容轮询周期。
  std::chrono::milliseconds poll_period{100};

  /*
   * 相邻超声请求开始时间的最小间隔。
   *
   * KS114 的请求可触发测距时，该间隔用于声学错峰。
   * UBS 若处于持续测量模式，该间隔只错开主机读取时间。
   */
  std::chrono::milliseconds ultrasonic_request_gap{100};

  // key 为设备逻辑名，例如 lidar_body_front。
  std::map<std::string, SensorPollConfig> device_polls;
};

/* ============================================================
 * 单设备通信健康
 * ============================================================ */

enum class SensorCommunicationHealth {
  kUnknown,
  kGood,
  kDegraded,
  kFault,
};
/* ============================================================
 * 单设备通信状态快照
 * ============================================================ */


struct SensorDeviceStatus {
  std::string logical_name;

  // online 只表示最近一次实际读取是否成功。
  bool online{false};

  int consecutive_failures{0};

  SensorCommunicationHealth communication_health{
      SensorCommunicationHealth::kUnknown};

  std::uint64_t last_poll_ms{0};
  std::uint64_t last_success_ms{0};
  std::uint64_t last_failure_ms{0};

  std::string last_error;
};

/* ============================================================
 * 传感器服务
 * ============================================================ */

class SensorService final : public drivers::ISensorSuite {
 public:
  SensorService(SensorServiceDevices devices,
                std::chrono::milliseconds stale_timeout,
                SensorServiceOptions options = {});

  ~SensorService() override;

  /*
   * 启动传感器对象和后台采集线程。
   *
   * 调试隔离模式下，单个设备启动失败不会阻止其他设备启动。
   */
  drivers::Result start() override;

  /*
   * ISensorSuite 兼容接口。
   *
   * 本函数只返回缓存快照，不执行任何硬件读取。
   */
  [[nodiscard]] std::vector<drivers::SensorSample> read_all() override;

  // 只发出退出通知，不等待线程结束。
  void request_stop() noexcept;

  // 发出退出通知、join 所有线程并停止设备。
  void stop() noexcept override;

  /*
   * SensorService 的健康状态表示服务和采集线程是否正常运行。
   * 单个传感器故障通过 device_statuses() 单独上报。
   */
  [[nodiscard]] drivers::DriverHealth health() const noexcept override;

  // 只读取缓存，不访问串口。
  [[nodiscard]] std::vector<drivers::SensorSample> cached_samples() const;

  // 只读取指定缓存项，不访问串口。
  [[nodiscard]] std::optional<drivers::SensorSample> cached_sample(
      const std::string& logical_name) const;

  [[nodiscard]] std::chrono::steady_clock::time_point last_poll_time() const;

  // 返回每个设备的独立通信状态。
  [[nodiscard]] std::vector<SensorDeviceStatus> device_statuses() const;

 private:
  struct CachedSensorSample {
    drivers::SensorSample sample;
    std::string device_name;

    std::chrono::steady_clock::time_point last_success_at{};
    std::chrono::steady_clock::time_point last_attempt_at{};

    std::chrono::milliseconds stale_after{1000};

    bool attempted{false};
  };

  struct PollTarget {
    std::string device_name;
    std::string bus_id;
    bool ultrasonic_request{false};
  };

  struct SampleUpdate {
    std::string logical_name;
    std::string unit;
    std::optional<double> value;
  };

  /* ------------------------------------------------------------
   * 配置和轮询目标
   * ------------------------------------------------------------ */

  [[nodiscard]] SensorPollConfig poll_config_for(
      const std::string& device_name) const;

  [[nodiscard]] std::string bus_id_for(
      const std::string& device_name) const;

  [[nodiscard]] std::vector<PollTarget> enabled_poll_targets() const;

  [[nodiscard]] std::vector<std::string> enabled_poll_groups() const;

  [[nodiscard]] std::vector<SampleUpdate> expected_updates_for(
      const std::string& device_name) const;

  [[nodiscard]] bool should_poll_device(
      const std::string& device_name,
      std::chrono::steady_clock::time_point now) const;

  [[nodiscard]] bool wait_for_ultrasonic_slot();

  [[nodiscard]] SensorCommunicationHealth communication_health_locked(
      const std::string& device_name,
      const SensorDeviceStatus& status) const;

  /* ------------------------------------------------------------
   * 缓存和设备状态
   * ------------------------------------------------------------ */

  void initialize_cache_locked();

  void register_sample_locked(const std::string& device_name,
                              const std::string& logical_name,
                              const std::string& unit);

  void commit_device_read(
      const std::string& device_name,
      const drivers::Result& result,
      const std::vector<SampleUpdate>& updates,
      std::chrono::steady_clock::time_point completed_at);

  [[nodiscard]] drivers::SensorSample sample_with_age(
      const CachedSensorSample& cached,
      std::chrono::steady_clock::time_point now) const;

  /* ------------------------------------------------------------
   * 后台采集
   * ------------------------------------------------------------ */

  void poll_loop(std::string bus_id);
  void poll_once_for_bus(const std::string& bus_id);
  void poll_target(const PollTarget& target);

  void poll_distance(const std::string& device_name,
                     const std::string& sample_name,
                     drivers::devices::DistanceSensor* sensor);

  void poll_wind(const std::string& device_name,
                 const std::string& sample_name,
                 drivers::devices::Anemometer* sensor);

  void poll_wit_motion();
  void poll_temperature();

  void poll_power(const std::string& device_name,
                  const std::string& sample_prefix,
                  drivers::devices::PowerMonitor* sensor);

  void stop_devices() noexcept;

  SensorServiceDevices devices_;
  std::chrono::milliseconds stale_timeout_;
  SensorServiceOptions options_;

  std::atomic<drivers::DriverHealth> health_{
      drivers::DriverHealth::kUnconfigured};

  std::atomic_bool running_{false};

  /*
   * lifecycle_mutex_ 只保护 start/stop 和线程容器。
   * 不用于设备读取，也不用于快照读取。
   */
  mutable std::mutex lifecycle_mutex_;
  bool started_{false};
  std::vector<std::thread> poll_threads_;

  /*
   * stop_mutex_ 只配合 condition_variable 等待，
   * 不覆盖任何底层硬件事务。
   */
  mutable std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  /*
   * cache_mutex_ 只保护内存缓存和调度时间。
   * 调用任何 sensor->read_*() 时都不得持有此锁。
   */
  mutable std::mutex cache_mutex_;
  std::map<std::string, CachedSensorSample> cache_;
  std::map<std::string, SensorDeviceStatus> device_status_;

  std::map<std::string, std::chrono::steady_clock::time_point>
      next_device_poll_;

  std::chrono::steady_clock::time_point last_poll_time_{};

  /*
   * 超声请求间隔锁只保护最后一次请求开始时间。
   * 等待期间不会持有缓存锁。
   */
  mutable std::mutex ultrasonic_mutex_;
  std::chrono::steady_clock::time_point last_ultrasonic_request_{};
};

}  // namespace snappy::control