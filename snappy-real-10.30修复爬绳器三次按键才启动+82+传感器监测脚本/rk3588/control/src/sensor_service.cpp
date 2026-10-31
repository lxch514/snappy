#include "snappy/control/sensor_service.hpp"

/**
 * @file sensor_service.cpp
 * @brief 传感器后台采集、缓存快照和通信故障隔离实现
 *
 * @details
 *
 * - 每条物理总线使用独立后台线程。
 * - 每个设备维护独立轮询周期和下一次轮询时间。
 * - 底层设备读取在任何 SensorService 缓存锁之外执行。
 * - 单设备失败只更新该设备状态，不改变其他设备状态。
 * - 缓存从启动时就包含全部预期测量项。
 * - 未尝试读取为 UNKNOWN，首次失败且没有成功值为 INVALID。
 * - 已有成功值时保留最后值，超过 stale_after 后标记 STALE。
 * - read_all() 和 cached_samples() 只复制内存快照。
 */

#include <algorithm>
#include <chrono>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include <exception>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy::control {
namespace {

/* ============================================================
 * 失败设备退避
 *
 * 连续失败达到 max_consecutive_failures 的设备，
 * 后续按本间隔重试，而不是每个轮询周期都占满一整段
 * response_deadline。
 *
 * 目的：一台坏设备不能把同一条串口上的其他设备一起拖到 stale。
 * 只要读取成功一次，consecutive_failures 归零，
 * 轮询间隔立即恢复为配置的 poll_interval。
 * ============================================================ */

constexpr auto kFailedDeviceRetryInterval = std::chrono::milliseconds(5000);

/* ============================================================
 * 时间辅助
 * ============================================================ */

std::uint64_t wall_time_ms() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

std::uint64_t elapsed_ms(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point then) {
  if (then.time_since_epoch().count() == 0 || now <= then) {
    return 0;
  }

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - then)
          .count());
}

drivers::Result missing_device_result(const std::string& device_name) {
  return drivers::Result::failure(
      drivers::ErrorCode::kConfigRequired,
      "configured sensor backend is unavailable",
      "start",
      device_name);
}

/* ============================================================
 * 设备停止辅助
 * ============================================================ */

void stop_if_present(drivers::devices::DistanceSensor* sensor) {
  if (sensor != nullptr) {
    sensor->stop();
  }
}

void stop_if_present(drivers::devices::Anemometer* sensor) {
  if (sensor != nullptr) {
    sensor->stop();
  }
}

void stop_if_present(drivers::devices::WitMotion* sensor) {
  if (sensor != nullptr) {
    sensor->stop();
  }
}

void stop_if_present(drivers::devices::TemperatureSensor* sensor) {
  if (sensor != nullptr) {
    sensor->stop();
  }
}

void stop_if_present(drivers::devices::PowerMonitor* sensor) {
  if (sensor != nullptr) {
    sensor->stop();
  }
}

}  // namespace

/* ============================================================
 * 构造与析构
 * ============================================================ */

SensorService::SensorService(
    SensorServiceDevices devices,
    std::chrono::milliseconds stale_timeout,
    SensorServiceOptions options)
    : devices_(devices),
      stale_timeout_(
          stale_timeout.count() > 0
              ? stale_timeout
              : std::chrono::milliseconds(1000)),
      options_(std::move(options)) {}

SensorService::~SensorService() {
  stop();
}

/* ============================================================
 * 轮询配置
 * ============================================================ */

SensorPollConfig SensorService::poll_config_for(
    const std::string& device_name) const {
  const auto it = options_.device_polls.find(device_name);

  if (it != options_.device_polls.end()) {
    SensorPollConfig config = it->second;

    if (config.bus_id.empty()) {
      config.bus_id = "default_sensor_bus";
    }

    if (config.poll_interval.count() <= 0) {
      config.poll_interval =
          options_.poll_period.count() > 0
              ? options_.poll_period
              : std::chrono::milliseconds(100);
    }

    if (config.stale_after.count() <= 0) {
      config.stale_after = stale_timeout_;
    }

    if (config.max_consecutive_failures <= 0) {
      config.max_consecutive_failures = 3;
    }

    return config;
  }

  SensorPollConfig fallback;
  fallback.enabled = true;
  fallback.bus_id = "default_sensor_bus";
  fallback.poll_interval =
      options_.poll_period.count() > 0
          ? options_.poll_period
          : std::chrono::milliseconds(100);
  fallback.stale_after = stale_timeout_;
  fallback.max_consecutive_failures = 3;

  return fallback;
}

std::string SensorService::bus_id_for(
    const std::string& device_name) const {
  return poll_config_for(device_name).bus_id;
}

/* ============================================================
 * 轮询目标
 * ============================================================ */

std::vector<SensorService::PollTarget>
SensorService::enabled_poll_targets() const {
  std::vector<PollTarget> targets;

  const auto add_target =
      [this, &targets](const char* device_name,
                       bool object_present,
                       bool ultrasonic_request) {
        const auto configured = options_.device_polls.find(device_name);

        const bool explicitly_configured = configured != options_.device_polls.end();

        /*
         * 已构建设备对象，或者配置中明确启用了该设备，
         * 都必须创建缓存和通信状态。
         *
         * 这样即使设备构建失败，也不会从安全快照中消失。
         */
        if (!object_present && !explicitly_configured) {
          return;
        }

        const auto config = poll_config_for(device_name);

        if (!config.enabled) {
          return;
        }

        targets.push_back(
            {device_name, config.bus_id, ultrasonic_request});
      };

  add_target(
      "lidar_brush_left",
      devices_.lidar_brush_left != nullptr,
      true);

  add_target(
      "lidar_brush_right",
      devices_.lidar_brush_right != nullptr,
      true);

  add_target(
      "lidar_body_front",
      devices_.lidar_body_front != nullptr,
      true);

  add_target(
      "lidar_body_rear",
      devices_.lidar_body_rear != nullptr,
      true);

  add_target(
      "anemometer_left",
      devices_.anemometer_left != nullptr,
      false);

  add_target(
      "anemometer_right",
      devices_.anemometer_right != nullptr,
      false);

  add_target(
      "wit_motion",
      devices_.wit_motion != nullptr,
      false);

  add_target(
      "temperature_cabinet",
      devices_.temperature_cabinet != nullptr,
      false);

  add_target(
      "power_24v",
      devices_.power_24v != nullptr,
      false);

  add_target(
      "power_48v",
      devices_.power_48v != nullptr,
      false);

  return targets;
}

std::vector<std::string> SensorService::enabled_poll_groups() const {
  std::set<std::string> groups;

  for (const auto& target : enabled_poll_targets()) {
    groups.insert(target.bus_id);
  }

  return {groups.begin(), groups.end()};
}

/* ============================================================
 * 每个设备的固定缓存项
 * ============================================================ */

std::vector<SensorService::SampleUpdate>
SensorService::expected_updates_for(
    const std::string& device_name) const {
  std::vector<SampleUpdate> updates;

  if (device_name == "lidar_brush_left" || device_name == "lidar_brush_right") {
    const std::string distance_name = device_name + "_distance";

    updates.push_back({distance_name, "mm", std::nullopt});

    /*
     * UBS 返回的第二个寄存器是温度。
     * 名称保持与当前程序兼容。
     */
    updates.push_back({distance_name + "_temperature", "degC", std::nullopt});

    return updates;
  }

  if (device_name == "lidar_body_front" || device_name == "lidar_body_rear") {
    /*
     * 当前 KS114 配置只返回距离值，
     * 不创建不存在的温度测量项。
     */
    updates.push_back(
        {device_name + "_distance",
         "mm",
         std::nullopt});

    return updates;
  }

  if (device_name == "anemometer_left" || device_name == "anemometer_right") {
    updates.push_back(
        {device_name + "_speed",
         "m/s",
         std::nullopt});

    return updates;
  }

  if (device_name == "wit_motion") {
    updates.push_back(
        {"wit_motion_accel_x", "g", std::nullopt});
    updates.push_back(
        {"wit_motion_accel_y", "g", std::nullopt});
    updates.push_back(
        {"wit_motion_accel_z", "g", std::nullopt});

    updates.push_back(
        {"wit_motion_gyro_x", "deg/s", std::nullopt});
    updates.push_back(
        {"wit_motion_gyro_y", "deg/s", std::nullopt});
    updates.push_back(
        {"wit_motion_gyro_z", "deg/s", std::nullopt});

    updates.push_back(
        {"wit_motion_roll", "deg", std::nullopt});
    updates.push_back(
        {"wit_motion_pitch", "deg", std::nullopt});
    updates.push_back(
        {"wit_motion_yaw", "deg", std::nullopt});

    updates.push_back(
        {"wit_motion_temperature",
         "degC",
         std::nullopt});

    return updates;
  }

  if (device_name == "temperature_cabinet") {
    updates.push_back(
        {"temperature_cabinet_temperature",
         "degC",
         std::nullopt});

    updates.push_back(
        {"temperature_cabinet_humidity",
         "%RH",
         std::nullopt});

    return updates;
  }

  if (device_name == "power_24v" ||
      device_name == "power_48v") {
    updates.push_back(
        {device_name + "_voltage",
         "V",
         std::nullopt});

    updates.push_back(
        {device_name + "_current",
         "A",
         std::nullopt});

    updates.push_back(
        {device_name + "_power",
         "W",
         std::nullopt});

    updates.push_back(
        {device_name + "_temperature",
         "degC",
         std::nullopt});

    updates.push_back(
        {device_name + "_operation_enabled",
         "bool",
         std::nullopt});

    return updates;
  }

  return updates;
}

/* ============================================================
 * 缓存初始化
 * ============================================================ */

void SensorService::register_sample_locked(
    const std::string& device_name,
    const std::string& logical_name,
    const std::string& unit) {
  if (cache_.find(logical_name) != cache_.end()) {
    return;
  }

  CachedSensorSample cached;
  cached.sample.logical_name = logical_name;
  cached.sample.value = std::nullopt;
  cached.sample.unit = unit;
  cached.sample.quality = drivers::SampleQuality::kUnknown;
  cached.sample.timestamp_ms = 0;
  cached.sample.age_ms = 0;

  cached.device_name = device_name;
  cached.stale_after =
      poll_config_for(device_name).stale_after;

  cached.attempted = false;

  cache_.emplace(
      logical_name,
      std::move(cached));
}

void SensorService::initialize_cache_locked() {
  cache_.clear();
  device_status_.clear();
  next_device_poll_.clear();
  last_poll_time_ = {};

  for (const auto& target : enabled_poll_targets()) {
    auto& status =
        device_status_[target.device_name];

    status.logical_name = target.device_name;
    status.online = false;
    status.consecutive_failures = 0;
    status.communication_health = SensorCommunicationHealth::kUnknown;
    status.last_poll_ms = 0;
    status.last_success_ms = 0;
    status.last_failure_ms = 0;
    status.last_error.clear();

    for (const auto& update :
         expected_updates_for(target.device_name)) {
      register_sample_locked(
          target.device_name,
          update.logical_name,
          update.unit);
    }
  }
}

SensorCommunicationHealth
SensorService::communication_health_locked(
    const std::string& device_name,
    const SensorDeviceStatus& status) const {
  if (status.last_poll_ms == 0) {
    return SensorCommunicationHealth::kUnknown;
  }

  if (status.online &&
      status.consecutive_failures == 0) {
    return SensorCommunicationHealth::kGood;
  }

  const auto config = poll_config_for(device_name);

  const int failure_threshold = std::max(1, config.max_consecutive_failures);

  if (status.consecutive_failures >= failure_threshold) {
    return SensorCommunicationHealth::kFault;
  }

  return SensorCommunicationHealth::kDegraded;
}

/* ============================================================
 * 样本质量和年龄计算
 * ============================================================ */

drivers::SensorSample SensorService::sample_with_age(
    const CachedSensorSample& cached,
    std::chrono::steady_clock::time_point now) const {
  auto sample = cached.sample;

  const bool has_success =
      cached.last_success_at.time_since_epoch().count() != 0 && sample.value.has_value();

  if (!has_success) {
    sample.age_ms = 0;
    sample.timestamp_ms = 0;
    sample.quality =
        cached.attempted
            ? drivers::SampleQuality::kInvalid
            : drivers::SampleQuality::kUnknown;

    return sample;
  }

  sample.age_ms = elapsed_ms(now, cached.last_success_at);

  const auto stale_after =
      cached.stale_after.count() > 0
          ? cached.stale_after
          : stale_timeout_;

  if (sample.age_ms > static_cast<std::uint64_t>(stale_after.count())) {
    sample.quality = drivers::SampleQuality::kStale;
  } else {
    /*
     * 最近一次通信即使失败，只要最后成功值还没有超时，
     * 测量值仍保持 GOOD。
     *
     * 通信失败由 SensorDeviceStatus.online 和 last_error 表示。
     */
    sample.quality = drivers::SampleQuality::kGood;
  }

  return sample;
}

/* ============================================================
 * 原子提交一次设备读取结果
 *
 * 调用本函数之前，底层读取必须已经结束。
 * 本函数只短时间持有缓存锁。
 * ============================================================ */

void SensorService::commit_device_read(
    const std::string& device_name,
    const drivers::Result& result,
    const std::vector<SampleUpdate>& updates,
    std::chrono::steady_clock::time_point completed_at) {
  const std::uint64_t completed_wall_ms = wall_time_ms();

  bool recovered = false;
  int failure_count = 0;
  std::string failure_detail;

  {
    std::lock_guard<std::mutex> lock(cache_mutex_);

    auto& status = device_status_[device_name];

    if (status.logical_name.empty()) {
      status.logical_name = device_name;
    }

    const bool had_previous_poll = status.last_poll_ms != 0;

    status.last_poll_ms = completed_wall_ms;

    if (result.ok) {
      recovered =
          had_previous_poll &&
          (!status.online || status.consecutive_failures > 0);

      status.online = true;
      status.consecutive_failures = 0;
      status.last_success_ms = completed_wall_ms;
      status.last_error.clear();
    } else {
      status.online = false;
      ++status.consecutive_failures;
      status.last_failure_ms = completed_wall_ms;

      failure_detail =
          result.detail.empty()
              ? drivers::error_code_name(result.code)
              : result.detail;

      status.last_error = failure_detail;
      failure_count = status.consecutive_failures;
    }

    status.communication_health = communication_health_locked(device_name, status);

    const auto config = poll_config_for(device_name);

    /*
     * 失败设备退避。
     *
     * 连续失败已经达到 max_consecutive_failures 的设备，
     * 下一次轮询至少间隔 kFailedDeviceRetryInterval，
     * 而不是每个轮询周期都尝试一次。
     *
     * 这样一台坏设备不会再每轮都占满一整段 response_deadline；
     * 同一串口上的其他设备因此能按时完成轮询，
     * 不会因为它的超时被推到 stale_after 之外。
     *
     * 一旦读取成功，consecutive_failures 被清零，
     * 下一次轮询立刻回到配置的 poll_interval。
     */
    auto next_interval = config.poll_interval;

    const int failure_threshold = std::max(1, config.max_consecutive_failures);

    if (status.consecutive_failures >=
            failure_threshold &&
        next_interval < kFailedDeviceRetryInterval) {
      next_interval = kFailedDeviceRetryInterval;
    }

    next_device_poll_[device_name] = completed_at + next_interval;

    last_poll_time_ = completed_at;

    for (const auto& update : updates) {
      register_sample_locked(
          device_name,
          update.logical_name,
          update.unit);

      auto& cached = cache_.at(update.logical_name);

      cached.last_attempt_at = completed_at;
      cached.attempted = true;
      cached.stale_after = config.stale_after;

      if (result.ok && update.value.has_value()) {
        cached.sample.value = update.value;
        cached.sample.unit = update.unit;
        cached.sample.quality = drivers::SampleQuality::kGood;
        cached.sample.timestamp_ms = completed_wall_ms;
        cached.sample.age_ms = 0;

        cached.last_success_at = completed_at;
      } else if (
          cached.last_success_at.time_since_epoch().count() == 0) {
        /*
         * 从未取得成功值：
         * 读取失败或成功帧缺少该字段，都标记为 INVALID。
         */
        cached.sample.value = std::nullopt;
        cached.sample.quality = drivers::SampleQuality::kInvalid;
        cached.sample.timestamp_ms = 0;
        cached.sample.age_ms = 0;
      }

      /*
       * 已经存在成功值时不清空缓存。
       * sample_with_age() 会根据最后成功时间决定 GOOD/STALE。
       */
    }
  }

  /*
   * 日志输出放在缓存锁之外，避免日志后端延长快照锁占用时间。
   */
  if (recovered) {
    drivers::diagnostics::info(
        "sensor.service",
        "sensor recovered: " + device_name);
  }

  if (!result.ok && (failure_count == 1 || failure_count % 10 == 0)) {
    drivers::diagnostics::warn(
        "sensor.service",
        "sensor read failed " +
            device_name +
            ": " +
            failure_detail);
  }
}

/* ============================================================
 * 缓存快照
 * ============================================================ */

std::vector<drivers::SensorSample>
SensorService::cached_samples() const {
  const auto now =
      std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(cache_mutex_);

  std::vector<drivers::SensorSample> samples;
  samples.reserve(cache_.size());

  for (const auto& item : cache_) {
    samples.push_back(
        sample_with_age(item.second, now));
  }

  return samples;
}

std::optional<drivers::SensorSample>
SensorService::cached_sample(
    const std::string& logical_name) const {
  const auto now =
      std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(cache_mutex_);

  const auto it = cache_.find(logical_name);

  if (it == cache_.end()) {
    return std::nullopt;
  }

  return sample_with_age(it->second, now);
}

std::chrono::steady_clock::time_point
SensorService::last_poll_time() const {
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return last_poll_time_;
}

std::vector<SensorDeviceStatus>
SensorService::device_statuses() const {
  std::lock_guard<std::mutex> lock(
      cache_mutex_);

  std::vector<SensorDeviceStatus> statuses;
  statuses.reserve(device_status_.size());

  for (const auto& item :
       device_status_) {
    auto status = item.second;

    status.communication_health =
        communication_health_locked(
            item.first,
            status);

    statuses.push_back(
        std::move(status));
  }

  return statuses;
}

std::vector<drivers::SensorSample>
SensorService::read_all() {
  return cached_samples();
}

/* ============================================================
 * 逐设备调度
 * ============================================================ */

bool SensorService::should_poll_device(
    const std::string& device_name,
    std::chrono::steady_clock::time_point now) const {
  const auto config =
      poll_config_for(device_name);

  if (!config.enabled ||
      config.poll_interval.count() <= 0) {
    return false;
  }

  std::lock_guard<std::mutex> lock(cache_mutex_);

  const auto it =
      next_device_poll_.find(device_name);

  if (it == next_device_poll_.end()) {
    return true;
  }

  return now >= it->second;
}

/* ============================================================
 * 超声请求错峰
 * ============================================================ */

bool SensorService::wait_for_ultrasonic_slot() {
  const auto gap =
      options_.ultrasonic_request_gap;

  if (gap.count() <= 0) {
    return running_.load();
  }

  while (running_.load()) {
    std::chrono::steady_clock::duration wait_duration{};

    {
      std::lock_guard<std::mutex> lock(ultrasonic_mutex_);

      const auto now =
          std::chrono::steady_clock::now();

      if (last_ultrasonic_request_
              .time_since_epoch()
              .count() == 0 ||
          now - last_ultrasonic_request_ >= gap) {
        /*
         * 记录的是请求即将开始的时间。
         * 请求自身耗时已经包含在下一次间隔计算中。
         */
        last_ultrasonic_request_ = now;
        return true;
      }

      wait_duration =
          last_ultrasonic_request_ + gap - now;
    }

    std::unique_lock<std::mutex> stop_lock(stop_mutex_);

    stop_cv_.wait_for(
        stop_lock,
        wait_duration,
        [this]() {
          return !running_.load();
        });
  }

  return false;
}

/* ============================================================
 * 测距设备读取
 * ============================================================ */

void SensorService::poll_distance(
    const std::string& device_name,
    const std::string& sample_name,
    drivers::devices::DistanceSensor* sensor) {
  auto updates =
      expected_updates_for(device_name);

  if (sensor == nullptr) {
    commit_device_read(
        device_name,
        missing_device_result(device_name),
        updates,
        std::chrono::steady_clock::now());

    return;
  }

  /*
   * 底层读取可能等待串口响应。
   * 这里没有持有 cache_mutex_ 或 lifecycle_mutex_。
   */
  const auto read =
      sensor->read_distance();

  const auto completed_at =
      std::chrono::steady_clock::now();

  for (auto& update : updates) {
    if (update.logical_name == sample_name) {
      update.value = read.distance_mm;
    } else if (
        update.logical_name ==
        sample_name + "_temperature") {
      update.value = read.temperature_celsius;
    }
  }

  commit_device_read(
      device_name,
      read.result,
      updates,
      completed_at);
}

/* ============================================================
 * 风速设备读取
 * ============================================================ */

void SensorService::poll_wind(
    const std::string& device_name,
    const std::string& sample_name,
    drivers::devices::Anemometer* sensor) {
  auto updates =
      expected_updates_for(device_name);

  if (sensor == nullptr) {
    commit_device_read(
        device_name,
        missing_device_result(device_name),
        updates,
        std::chrono::steady_clock::now());

    return;
  }

  const auto read =
      sensor->read_speed();

  const auto completed_at =
      std::chrono::steady_clock::now();

  for (auto& update : updates) {
    if (update.logical_name == sample_name) {
      update.value = read.speed_mps;
    }
  }

  commit_device_read(
      device_name,
      read.result,
      updates,
      completed_at);
}

/* ============================================================
 * 姿态设备读取
 * ============================================================ */

void SensorService::poll_wit_motion() {
  constexpr const char* kDeviceName =
      "wit_motion";

  auto updates =
      expected_updates_for(kDeviceName);

  if (devices_.wit_motion == nullptr) {
    commit_device_read(
        kDeviceName,
        missing_device_result(kDeviceName),
        updates,
        std::chrono::steady_clock::now());

    return;
  }

  const auto read =
      devices_.wit_motion->read_motion();

  const auto completed_at =
      std::chrono::steady_clock::now();

  for (auto& update : updates) {
    if (update.logical_name ==
        "wit_motion_accel_x") {
      update.value = read.accel_x_g;
    } else if (
        update.logical_name ==
        "wit_motion_accel_y") {
      update.value = read.accel_y_g;
    } else if (
        update.logical_name ==
        "wit_motion_accel_z") {
      update.value = read.accel_z_g;
    } else if (
        update.logical_name ==
        "wit_motion_gyro_x") {
      update.value = read.gyro_x_dps;
    } else if (
        update.logical_name ==
        "wit_motion_gyro_y") {
      update.value = read.gyro_y_dps;
    } else if (
        update.logical_name ==
        "wit_motion_gyro_z") {
      update.value = read.gyro_z_dps;
    } else if (
        update.logical_name ==
        "wit_motion_roll") {
      update.value = read.roll_deg;
    } else if (
        update.logical_name ==
        "wit_motion_pitch") {
      update.value = read.pitch_deg;
    } else if (
        update.logical_name ==
        "wit_motion_yaw") {
      update.value = read.yaw_deg;
    } else if (
        update.logical_name ==
        "wit_motion_temperature") {
      update.value = read.temperature_celsius;
    }
  }

  commit_device_read(
      kDeviceName,
      read.result,
      updates,
      completed_at);
}

/* ============================================================
 * 温湿度设备读取
 * ============================================================ */

void SensorService::poll_temperature() {
  constexpr const char* kDeviceName =
      "temperature_cabinet";

  auto updates =
      expected_updates_for(kDeviceName);

  if (devices_.temperature_cabinet == nullptr) {
    commit_device_read(
        kDeviceName,
        missing_device_result(kDeviceName),
        updates,
        std::chrono::steady_clock::now());

    return;
  }

  const auto read =
      devices_.temperature_cabinet->read();

  const auto completed_at =
      std::chrono::steady_clock::now();

  for (auto& update : updates) {
    if (update.logical_name ==
        "temperature_cabinet_temperature") {
      update.value =
          read.temperature_celsius;
    } else if (
        update.logical_name ==
        "temperature_cabinet_humidity") {
      update.value =
          read.humidity_rh;
    }
  }

  commit_device_read(
      kDeviceName,
      read.result,
      updates,
      completed_at);
}

/* ============================================================
 * 开关电源状态读取
 * ============================================================ */

void SensorService::poll_power(
    const std::string& device_name,
    const std::string& sample_prefix,
    drivers::devices::PowerMonitor* sensor) {
  auto updates =
      expected_updates_for(device_name);

  if (sensor == nullptr) {
    commit_device_read(
        device_name,
        missing_device_result(device_name),
        updates,
        std::chrono::steady_clock::now());

    return;
  }

  const auto read =
      sensor->read_power();

  const auto completed_at =
      std::chrono::steady_clock::now();

  std::optional<double> operation_enabled;

  if (read.operation_enabled.has_value()) {
    operation_enabled =
        *read.operation_enabled ? 1.0 : 0.0;
  }

  for (auto& update : updates) {
    if (update.logical_name ==
        sample_prefix + "_voltage") {
      update.value = read.voltage_v;
    } else if (
        update.logical_name ==
        sample_prefix + "_current") {
      update.value = read.current_a;
    } else if (
        update.logical_name ==
        sample_prefix + "_power") {
      update.value = read.power_w;
    } else if (
        update.logical_name ==
        sample_prefix + "_temperature") {
      update.value =
          read.temperature_celsius;
    } else if (
        update.logical_name ==
        sample_prefix + "_operation_enabled") {
      update.value = operation_enabled;
    }
  }

  commit_device_read(
      device_name,
      read.result,
      updates,
      completed_at);
}

/* ============================================================
 * 轮询目标分发
 * ============================================================ */

void SensorService::poll_target(
    const PollTarget& target) {
  if (target.device_name ==
      "lidar_brush_left") {
    poll_distance(
        target.device_name,
        "lidar_brush_left_distance",
        devices_.lidar_brush_left);

    return;
  }

  if (target.device_name ==
      "lidar_brush_right") {
    poll_distance(
        target.device_name,
        "lidar_brush_right_distance",
        devices_.lidar_brush_right);

    return;
  }

  if (target.device_name ==
      "lidar_body_front") {
    poll_distance(
        target.device_name,
        "lidar_body_front_distance",
        devices_.lidar_body_front);

    return;
  }

  if (target.device_name ==
      "lidar_body_rear") {
    poll_distance(
        target.device_name,
        "lidar_body_rear_distance",
        devices_.lidar_body_rear);

    return;
  }

  if (target.device_name ==
      "anemometer_left") {
    poll_wind(
        target.device_name,
        "anemometer_left_speed",
        devices_.anemometer_left);

    return;
  }

  if (target.device_name ==
      "anemometer_right") {
    poll_wind(
        target.device_name,
        "anemometer_right_speed",
        devices_.anemometer_right);

    return;
  }

  if (target.device_name ==
      "wit_motion") {
    poll_wit_motion();
    return;
  }

  if (target.device_name ==
      "temperature_cabinet") {
    poll_temperature();
    return;
  }

  if (target.device_name ==
      "power_24v") {
    poll_power(
        target.device_name,
        "power_24v",
        devices_.power_24v);

    return;
  }

  if (target.device_name ==
      "power_48v") {
    poll_power(
        target.device_name,
        "power_48v",
        devices_.power_48v);
  }
}

/* ============================================================
 * 单物理总线轮询
 * ============================================================ */

void SensorService::poll_once_for_bus(
    const std::string& bus_id) {
  if (!running_.load() ||
      health_.load() !=
          drivers::DriverHealth::kReady) {
    return;
  }

  /*
   * 一个物理串口对应一个 SensorService worker。
   * 单个设备异常只能影响当前设备，不能退出整个串口线程。
   */
  for (const auto& target :
       enabled_poll_targets()) {
    if (!running_.load()) {
      return;
    }

    if (target.bus_id != bus_id) {
      continue;
    }

    const auto now =
        std::chrono::steady_clock::now();

    if (!should_poll_device(
            target.device_name,
            now)) {
      continue;
    }

    if (target.ultrasonic_request &&
        !wait_for_ultrasonic_slot()) {
      return;
    }

    try {
      /*
       * 底层通信只在当前串口 worker 中执行。
       * cache_mutex_ 不跨越底层通信调用。
       */
      poll_target(target);
    } catch (const std::exception& error) {
      const auto completed_at = std::chrono::steady_clock::now();

      const auto updates = expected_updates_for(target.device_name);

      const auto failure =
          drivers::Result::failure(
              drivers::ErrorCode::kIoError,
              std::string(
                  "sensor polling exception: ") +
                  error.what(),
              "sensor.poll",
              target.device_name,
              true);

      /*
       * 失败只写入当前设备的缓存状态。
       * 其他设备继续轮询，旧快照保留并由 stale 逻辑处理。
       */
      commit_device_read(
          target.device_name,
          failure,
          updates,
          completed_at);

      drivers::diagnostics::info(
          "sensor.service",
          "isolated sensor exception device=" +
              target.device_name +
              " detail=" +
              error.what());
    } catch (...) {
      const auto completed_at =
          std::chrono::steady_clock::now();

      const auto updates =
          expected_updates_for(
              target.device_name);

      const auto failure =
          drivers::Result::failure(
              drivers::ErrorCode::kIoError,
              "unknown sensor polling exception",
              "sensor.poll",
              target.device_name,
              true);

      commit_device_read(
          target.device_name,
          failure,
          updates,
          completed_at);

      drivers::diagnostics::info(
          "sensor.service",
          "isolated unknown sensor exception device=" +
              target.device_name);
    }
  }
}

/* ============================================================
 * 单物理总线后台线程
 * ============================================================ */

void SensorService::poll_loop(std::string bus_id) {
  drivers::diagnostics::info(
      "sensor.service",
      "sensor polling thread started bus=" + bus_id);

  const auto scheduler_period =
      options_.scheduler_period.count() > 0
          ? options_.scheduler_period
          : std::chrono::milliseconds(20);

  while (running_.load()) {
    poll_once_for_bus(bus_id);

    std::unique_lock<std::mutex> lock(stop_mutex_);

    stop_cv_.wait_for(
        lock,
        scheduler_period,
        [this]() {
          return !running_.load();
        });
  }

  drivers::diagnostics::info(
      "sensor.service",
      "sensor polling thread stopped bus=" + bus_id);
}

/* ============================================================
 * 启动
 * ============================================================ */

drivers::Result SensorService::start() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  if (started_) {
    return drivers::Result::success("sensor service already started");
  }

  {
    std::lock_guard<std::mutex> cache_lock(cache_mutex_);

    initialize_cache_locked();
  }

  {
    std::lock_guard<std::mutex> ultrasonic_lock(ultrasonic_mutex_);

    last_ultrasonic_request_ = {};
  }

  const auto targets = enabled_poll_targets();

  const auto target_enabled =
      [&targets](const std::string& device_name) {
        return std::any_of(
            targets.begin(),
            targets.end(),
            [&device_name](const PollTarget& target) {
              return target.device_name == device_name;
            });
      };

  bool startup_failure = false;
  drivers::Result first_failure = drivers::Result::success();

  /*
   * 所有设备都会尝试启动。
   * 调试隔离模式下，不会在第一个失败处提前返回。
   */
  const auto start_sensor =
      [this,
       &target_enabled,
       &startup_failure,
       &first_failure](
          const char* device_name,
          auto* sensor) {
        if (!target_enabled(device_name)) {
          return;
        }

        const drivers::Result result =
            sensor != nullptr
                ? sensor->start()
                : missing_device_result(device_name);

        if (result.ok) {
          return;
        }

        if (!startup_failure) {
          first_failure = result;
        }

        startup_failure = true;

        commit_device_read(
            device_name,
            result,
            expected_updates_for(device_name),
            std::chrono::steady_clock::now());
      };

  start_sensor(
      "lidar_brush_left",
      devices_.lidar_brush_left);

  start_sensor(
      "lidar_brush_right",
      devices_.lidar_brush_right);

  start_sensor(
      "lidar_body_front",
      devices_.lidar_body_front);

  start_sensor(
      "lidar_body_rear",
      devices_.lidar_body_rear);

  start_sensor(
      "anemometer_left",
      devices_.anemometer_left);

  start_sensor(
      "anemometer_right",
      devices_.anemometer_right);

  start_sensor(
      "wit_motion",
      devices_.wit_motion);

  start_sensor(
      "temperature_cabinet",
      devices_.temperature_cabinet);

  start_sensor(
      "power_24v",
      devices_.power_24v);

  start_sensor(
      "power_48v",
      devices_.power_48v);

  if (startup_failure &&
      !options_.ignore_startup_failures) {
    stop_devices();

    health_ = drivers::DriverHealth::kFault;

    return first_failure;
  }

  health_ = drivers::DriverHealth::kReady;

  const auto groups = enabled_poll_groups();

  if (options_.background_polling &&
      !groups.empty()) {
    running_.store(true);

    for (const auto& bus_id : groups) {
      poll_threads_.emplace_back(
          [this, bus_id]() {
            poll_loop(bus_id);
          });
    }

    drivers::diagnostics::info(
        "sensor.service",
        "background polling started groups=" +
            std::to_string(groups.size()));
  }

  started_ = true;

  if (startup_failure) {
    return drivers::Result::success("sensor service ready with isolated device failures");
  }

  return drivers::Result::success(
      "sensor service ready");
}

/* ============================================================
 * 停止
 * ============================================================ */

void SensorService::request_stop() noexcept {
  running_.store(false);
  stop_cv_.notify_all();
}

void SensorService::stop_devices() noexcept {
  stop_if_present(devices_.lidar_brush_left);

  stop_if_present(devices_.lidar_brush_right);

  stop_if_present(devices_.lidar_body_front);

  stop_if_present(devices_.lidar_body_rear);

  stop_if_present(devices_.anemometer_left);

  stop_if_present(devices_.anemometer_right);

  stop_if_present(devices_.wit_motion);

  stop_if_present(devices_.temperature_cabinet);

  stop_if_present(devices_.power_24v);

  stop_if_present(devices_.power_48v);
}

void SensorService::stop() noexcept {
  request_stop();

  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  for (auto& thread : poll_threads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  poll_threads_.clear();

  if (started_ ||
      health_.load() != drivers::DriverHealth::kUnconfigured) {
    stop_devices();
  }

  started_ = false;

  health_ = drivers::DriverHealth::kStopped;
}

/* ============================================================
 * 服务健康
 * ============================================================ */

drivers::DriverHealth
SensorService::health() const noexcept {
  return health_.load();
}

}  // namespace snappy::control