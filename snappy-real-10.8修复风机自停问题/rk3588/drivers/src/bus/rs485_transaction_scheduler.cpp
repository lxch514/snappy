#include "snappy/drivers/bus/rs485_transaction_scheduler.hpp"

/**
 * @file rs485_transaction_scheduler.cpp
 * @brief RS485 多端口后台事务调度实现
 *
 * @details
 * - 每个物理串口由一个独立 worker 串行访问。
 * - WCH ACM0~ACM3 使用同一个公平票据门禁（硬件约束，
 *   四路共用同一 USB 端点与芯片内部分时逻辑）。
 * - 单端口队列有固定上限并按优先级和 deadline 排序。
 * - 同一个设备未完成的读取不会重复堆积。
 * - 队列满时，高优先级任务可以淘汰队尾低优先级读取。
 * - timeout、短帧、CRC、站号、功能码和长度错误归属目标设备。
 * - 只有 fd、POLLERR、POLLHUP、EIO 等错误触发端口恢复。
 * - 写事务无论如何配置都不会自动重放。
 * - 连续失败设备会被降级为最低优先级并进入软限流，
 *   保证同端口健康设备永远优先获得总线。
 * - 单次事务有墙钟兜底，端口锁不会被长期占住。
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "snappy/drivers/bus/rs485_port.hpp"
#include "snappy/drivers/diagnostics/driver_log.hpp"
#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::bus {
namespace {

/* ============================================================
 * 调度常量
 * ============================================================ */

constexpr const char* kWchUsbQuadGroup =
    "wch_usb_quad_0";

constexpr int kSensorPriorityFloor = 20;

constexpr int kMaximumReadRetries = 3;
constexpr int kPortReopenFailureThreshold = 9;

constexpr auto kPortReopenCooldown = std::chrono::seconds(2);

/* ============================================================
 * 同端口设备级公平性
 * ============================================================ */

/*
 * 连续失败多少次后进入软限流。
 */
constexpr int kDeviceStarvationFailures = 3;

/*
 * 限流设备的最小放行间隔。
 */
constexpr auto kDeviceThrottleInterval =
    std::chrono::milliseconds(2500);

/*
 * 单次事务允许占用的最大墙钟时间（兜底）。
 */
constexpr auto kMaxTransactionWallClock =
    std::chrono::milliseconds(6000);

/*
 * 持续故障设备的隔离时长上限。
 */
constexpr auto kMaxQuarantineDuration =
    std::chrono::milliseconds(30000);

/*
 * 环境变量开关读取。
 *
 * 只接受 0 / false / FALSE / no / off 为关闭；
 * 未设置或其它取值一律使用默认值。
 */
bool env_flag_enabled(
    const char* name,
    bool fallback) {
  const char* raw =
      ::getenv(name);

  if (raw == nullptr) {
    return fallback;
  }

  const std::string value(
      raw);

  if (value == "0" ||
      value == "false" ||
      value == "FALSE" ||
      value == "no" ||
      value == "off") {
    return false;
  }

  if (value == "1" ||
      value == "true" ||
      value == "TRUE" ||
      value == "yes" ||
      value == "on") {
    return true;
  }

  return fallback;
}

/* ============================================================
 * 通用辅助
 * ============================================================ */

bool has_deadline(
    std::chrono::steady_clock::time_point value) {
  return value.time_since_epoch().count() != 0;
}

std::chrono::steady_clock::time_point
make_deadline(
    std::chrono::milliseconds timeout) {
  return std::chrono::steady_clock::now() +
         timeout;
}

bool is_wch_acm0_to_acm3(
    const std::string& path) {
  return path == "/dev/ttyACM0" ||
         path == "/dev/ttyACM1" ||
         path == "/dev/ttyACM2" ||
         path == "/dev/ttyACM3" ||
         path == "/dev/ACM0" ||
         path == "/dev/ACM1" ||
         path == "/dev/ACM2" ||
         path == "/dev/ACM3";
}

bool is_ks114_logical_device(
    const std::string& logical_device) {
  return logical_device ==
             "lidar_body_front" ||
         logical_device ==
             "lidar_body_rear";
}

bool should_count_for_quarantine(
    const Result& result) {
  if (result.ok) {
    return false;
  }

  switch (result.code) {
    case ErrorCode::kRs485Timeout:
    case ErrorCode::kRs485FrameIncomplete:
    case ErrorCode::kRs485CrcMismatch:
    case ErrorCode::kRs485AddressMismatch:
    case ErrorCode::kRs485FunctionMismatch:
    case ErrorCode::kRs485LengthMismatch:
    case ErrorCode::kRs485ExceptionResponse:
      return true;

    default:
      return false;
  }
}

bool should_retry_read(
    const Result& result) {
  if (result.ok) {
    return false;
  }

  switch (result.code) {
    case ErrorCode::kRs485Timeout:
    case ErrorCode::kRs485FrameIncomplete:
    case ErrorCode::kRs485CrcMismatch:
    case ErrorCode::kRs485AddressMismatch:
    case ErrorCode::kRs485FunctionMismatch:
    case ErrorCode::kRs485LengthMismatch:
      return true;

    default:
      return false;
  }
}

bool transaction_precedes(
    const Rs485Transaction& left,
    const Rs485Transaction& right) {
  if (left.priority != right.priority) {
    return left.priority <
           right.priority;
  }

  const bool left_has_deadline =
      has_deadline(left.deadline);

  const bool right_has_deadline =
      has_deadline(right.deadline);

  if (left_has_deadline !=
      right_has_deadline) {
    return left_has_deadline;
  }

  if (left_has_deadline &&
      left.deadline !=
          right.deadline) {
    return left.deadline <
           right.deadline;
  }

  return false;
}

Result cancelled_result(
    const Rs485Transaction& transaction,
    const std::string& detail) {
  return Result::failure(
      ErrorCode::kIoError,
      detail,
      "queue",
      transaction.logical_device,
      true);
}

}  // namespace

/* ============================================================
 * WCH 公平事务门禁
 *
 * 一个 ticket 覆盖：
 * 清理 -> TX -> RX -> 验帧 -> 迟到帧恢复。
 * ============================================================ */

struct Rs485TransactionScheduler::
    TransactionArbiter {
  enum class AcquireResult {
    kAcquired,
    kTimedOut,
    kStopped,
  };

  AcquireResult acquire(
      std::chrono::steady_clock::time_point deadline,
      std::uint64_t* ticket_out) {
    std::unique_lock<std::mutex> lock(
        mutex);

    if (stopping) {
      return AcquireResult::kStopped;
    }

    const std::uint64_t ticket =
        next_ticket++;

    while (!stopping &&
           ticket != serving_ticket) {
      if (!has_deadline(deadline)) {
        cv.wait(lock);
        continue;
      }

      if (cv.wait_until(
              lock,
              deadline) ==
          std::cv_status::timeout) {
        cancelled.insert(ticket);
        advance_cancelled_locked();
        cv.notify_all();

        return AcquireResult::kTimedOut;
      }
    }

    if (stopping) {
      cancelled.insert(ticket);
      advance_cancelled_locked();
      cv.notify_all();

      return AcquireResult::kStopped;
    }

    if (ticket_out != nullptr) {
      *ticket_out = ticket;
    }

    return AcquireResult::kAcquired;
  }

  void release(
      std::uint64_t ticket) noexcept {
    std::lock_guard<std::mutex> lock(
        mutex);

    if (ticket == serving_ticket) {
      ++serving_ticket;
      advance_cancelled_locked();
    } else {
      cancelled.insert(ticket);
    }

    cv.notify_all();
  }

  void stop() noexcept {
    std::lock_guard<std::mutex> lock(
        mutex);

    stopping = true;
    cv.notify_all();
  }

  void reset() noexcept {
    std::lock_guard<std::mutex> lock(
        mutex);

    stopping = false;
    next_ticket = 0;
    serving_ticket = 0;
    cancelled.clear();

    cv.notify_all();
  }

 private:
  void advance_cancelled_locked() {
    while (cancelled.erase(
               serving_ticket) != 0) {
      ++serving_ticket;
    }
  }

  std::mutex mutex;
  std::condition_variable cv;

  std::uint64_t next_ticket{0};
  std::uint64_t serving_ticket{0};

  std::set<std::uint64_t> cancelled;

  bool stopping{false};
};

/* ============================================================
 * 内部任务和端口状态
 * ============================================================ */

struct Rs485TransactionScheduler::PortJob {
  Rs485Transaction transaction;

  std::promise<Rs485TransactionResult>
      completion;
  std::atomic_bool abandoned{false};

  std::atomic_bool started{false};
};

struct Rs485TransactionScheduler::PortSlot {
  std::string port_id;

  std::unique_ptr<IRs485Port> port;

  /*
   * 一个端口一次只能执行一个完整事务。
   */
  std::mutex transaction_mutex;

  /*
   * WCH 多端口附加共享门禁。
   */
  std::shared_ptr<TransactionArbiter> shared_arbiter;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;

  std::deque<std::shared_ptr<PortJob>> queue;

  std::thread worker;

  bool worker_running{false};
  bool worker_stop{false};

  /*
   * 以下状态只由 start、stop 或本端口 worker 访问。
   */
  bool open_failed{false};

  std::chrono::steady_clock::time_point
      next_open_attempt{};

  std::string last_open_error;
  int consecutive_transport_failures{0};
};

struct Rs485TransactionScheduler::
    DeviceFailureState {
  int consecutive_failures{0};

  std::chrono::steady_clock::time_point
      quarantine_until{};

  /*
   * 软限流放行节拍。
   */
  std::chrono::steady_clock::time_point
      next_release_attempt{};

  /*
   * 连续被隔离的次数（一次成功通信立即清零）。
   */
  int quarantine_streak{0};
};

/* ============================================================
 * 构造和析构
 * ============================================================ */

Rs485TransactionScheduler::
    Rs485TransactionScheduler() = default;

Rs485TransactionScheduler::
    ~Rs485TransactionScheduler() {
  stop();
}

/* ============================================================
 * 调试和隔离选项
 * ============================================================ */

void Rs485TransactionScheduler::
set_allow_missing_ports(
    bool allow) noexcept {
  std::lock_guard<std::mutex> lock(
      mutex_);

  allow_missing_ports_ =
      allow;
}

void Rs485TransactionScheduler::
set_device_quarantine(
    bool enabled,
    int after_failures,
    std::chrono::milliseconds duration)
    noexcept {
  std::lock_guard<std::mutex> lock(
      mutex_);

  quarantine_devices_ =
      enabled;

  quarantine_after_failures_ =
      std::max(
          1,
          after_failures);

  quarantine_duration_ =
      duration.count() > 0
          ? duration
          : std::chrono::milliseconds(
                2000);
}

/* ============================================================
 * 注册物理端口
 * ============================================================ */

Result Rs485TransactionScheduler::add_port(
    const Rs485PortConfig& config) {
  if (!valid_rs485_port_config(
          config)) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "invalid rs485 port config",
        "add_port",
        config.port_id);
  }

  std::lock_guard<std::mutex> lock(
      mutex_);

  if (health_ ==
      DriverHealth::kReady) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "cannot add rs485 port after scheduler start",
        "add_port",
        config.port_id);
  }

  if (ports_.count(
          config.port_id) != 0) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "duplicate rs485 port id",
        "add_port",
        config.port_id);
  }

  for (const auto& item :
       ports_) {
    if (item.second->port->config()
            .device_path ==
        config.device_path) {
      return Result::failure(
          ErrorCode::kInvalidArgument,
          "duplicate rs485 device path",
          "add_port",
          config.device_path);
    }
  }

  auto slot =
      std::make_shared<PortSlot>();

  slot->port_id =
      config.port_id;

  slot->port =
      std::make_unique<Rs485Port>(
          config);

  /*
   * WCH 组事务门禁开关。
   */
  const bool wch_group_lock_enabled =
      env_flag_enabled(
          "SNAPPY_RS485_WCH_GROUP_LOCK",
          true);

  std::string shared_group;

  if (is_wch_acm0_to_acm3(
          config.device_path)) {
    if (wch_group_lock_enabled) {
      shared_group =
          kWchUsbQuadGroup;
    }
  } else if (
      !config.shared_transaction_group
           .empty()) {
    shared_group =
        config.shared_transaction_group;
  } else {
    shared_group =
        config.shared_lock_group;
  }

  if (!shared_group.empty()) {
    auto& arbiter =
        shared_arbiters_[
            shared_group];

    if (!arbiter) {
      arbiter =
          std::make_shared<
              TransactionArbiter>();
    }

    slot->shared_arbiter =
        arbiter;
  }

  ports_[
      config.port_id] =
      std::move(slot);

  if (health_ ==
      DriverHealth::kUnconfigured) {
    health_ =
        DriverHealth::kStopped;
  }

  diagnostics::info(
      "rs485.scheduler",
      "registered port " +
          config.port_id +
          " path=" +
          config.device_path +
          " backend=" +
          config.backend +
          (shared_group.empty()
               ? ""
               : " transaction_group=" +
                     shared_group));

  return Result::success(
      "rs485 port registered");
}

/* ============================================================
 * 绑定逻辑设备
 * ============================================================ */

Result Rs485TransactionScheduler::
bind_logical_device(
    const Rs485DeviceBinding& binding) {
  if (binding.logical_device.empty() ||
      binding.port_id.empty() ||
      binding.max_response_bytes == 0 ||
      binding.timeout.count() <= 0 ||
      binding.max_consecutive_failures <= 0 ||
      binding.quarantine_duration.count() < 0 ||
      binding.read_retry_count < 0 ||
      binding.priority < 0) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "invalid rs485 device binding",
        "bind_logical_device",
        binding.logical_device);
  }

  std::lock_guard<std::mutex> lock(
      mutex_);

  if (health_ ==
      DriverHealth::kReady) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "cannot bind rs485 device after scheduler start",
        "bind_logical_device",
        binding.logical_device);
  }

  if (ports_.count(
          binding.port_id) == 0) {
    return Result::failure(
        ErrorCode::kConfigRequired,
        "binding references unknown rs485 port",
        "bind_logical_device",
        binding.logical_device);
  }

  if (bindings_.count(
          binding.logical_device) != 0) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "duplicate rs485 logical device",
        "bind_logical_device",
        binding.logical_device);
  }

  bindings_[
      binding.logical_device] =
      binding;

  device_failures_[
      binding.logical_device] =
      DeviceFailureState{};

  diagnostics::info(
      "rs485.scheduler",
      "bound device " +
          binding.logical_device +
          " to " +
          binding.port_id);

  return Result::success(
      "rs485 device binding registered");
}

/* ============================================================
 * 启动端口和 worker
 * ============================================================ */

Result Rs485TransactionScheduler::start() {
  std::vector<std::shared_ptr<PortSlot>>
      slots;

  bool allow_missing_ports =
      false;

  {
    std::lock_guard<std::mutex> lock(
        mutex_);

    if (health_ ==
        DriverHealth::kReady) {
      return Result::success(
          "rs485 scheduler already started");
    }

    if (ports_.empty()) {
      health_ =
          DriverHealth::kUnconfigured;

      return Result::failure(
          ErrorCode::kConfigRequired,
          "no rs485 ports configured",
          "start",
          "rs485");
    }

    for (auto& item :
         shared_arbiters_) {
      item.second->reset();
    }

    for (const auto& item :
         ports_) {
      slots.push_back(
          item.second);
    }

    allow_missing_ports =
        allow_missing_ports_;
  }

  Result first_failure =
      Result::success();

  std::vector<std::shared_ptr<PortSlot>>
      opened_slots;

  for (const auto& slot :
       slots) {
    const auto opened =
        slot->port->open();

    if (!opened.ok) {
      slot->open_failed =
          true;

      slot->last_open_error =
          opened.detail;

      slot->next_open_attempt =
          std::chrono::steady_clock::now() +
          slot->port->config()
              .recovery_delay;

      if (first_failure.ok) {
        first_failure =
            opened;
      }

      diagnostics::error(
          "rs485.scheduler",
          "open failed for " +
              slot->port_id +
              ": " +
              opened.detail);

      if (!allow_missing_ports) {
        for (const auto& opened_slot :
             opened_slots) {
          opened_slot->port->close();
        }

        std::lock_guard<std::mutex> lock(
            mutex_);

        health_ =
            DriverHealth::kFault;

        return opened;
      }

      continue;
    }

    slot->open_failed = false;

    slot->last_open_error.clear();

    slot->next_open_attempt = {};

    slot->consecutive_transport_failures = 0;

    opened_slots.push_back(slot);
  }

  /*
   * 先设置全部 worker 状态，再创建线程。
   */
  for (const auto& slot :
       slots) {
    std::lock_guard<std::mutex> lock(
        slot->queue_mutex);

    if (slot->worker.joinable()) {
      std::lock_guard<std::mutex>
          state_lock(mutex_);

      health_ =
          DriverHealth::kFault;

      return Result::failure(
          ErrorCode::kIoError,
          "rs485 worker remained joinable before start",
          "start",
          slot->port_id);
    }

    slot->worker_stop = false;

    slot->worker_running = true;
  }

  try {
    for (const auto& slot :
         slots) {
      slot->worker =
          std::thread(
              [this, slot]() {
                worker_loop(slot);
              });
    }
  } catch (...) {
    stop();

    return Result::failure(
        ErrorCode::kIoError,
        "failed to create rs485 port worker",
        "start",
        "rs485");
  }

  {
    std::lock_guard<std::mutex> lock(
        mutex_);

    health_ =
        DriverHealth::kReady;
  }

  diagnostics::info(
      "rs485.scheduler",
      "rs485 scheduler started with per-port workers");

  if (!first_failure.ok &&
      allow_missing_ports) {
    return Result::success(
        "rs485 scheduler started with unavailable ports");
  }

  return Result::success(
      "rs485 scheduler ready");
}

/* ============================================================
 * 提交事务
 * ============================================================ */

Rs485Response
Rs485TransactionScheduler::exchange(
    const Rs485Request& request) {
  if (request.payload.empty()) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "rs485 request payload is empty",
            "exchange",
            request.logical_device),
        {}};
  }

  Rs485DeviceBinding binding;

  std::shared_ptr<PortSlot> slot;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (health_ != DriverHealth::kReady) {
      return {
          Result::failure(
              ErrorCode::kConfigRequired,
              "rs485 scheduler is not ready",
              "exchange",
              request.logical_device),
          {}};
    }

    const auto binding_it = bindings_.find(request.logical_device);

    if (binding_it == bindings_.end()) {
      return {
          Result::failure(
              ErrorCode::kConfigRequired,
              "no rs485 binding for logical device",
              "exchange",
              request.logical_device),
          {}};
    }

    const auto now = std::chrono::steady_clock::now();

    if (device_quarantined_locked(
            request.logical_device,
            now)) {
      return {
          Result::failure(
              ErrorCode::kIoError,
              "rs485 device temporarily quarantined",
              "exchange",
              request.logical_device,
              true),
          {}};
    }

    binding = binding_it->second;

    const auto port_it = ports_.find(binding.port_id);

    if (port_it == ports_.end()) {
      return {
          Result::failure(
              ErrorCode::kConfigRequired,
              "unknown rs485 port",
              "exchange",
              binding.port_id),
          {}};
    }

    slot = port_it->second;
  }

  Rs485Transaction transaction;

  transaction.port_id = binding.port_id;

  transaction.logical_device = request.logical_device;

  transaction.request = request.payload;

  transaction.max_response_bytes = binding.max_response_bytes;

  transaction.timeout = binding.timeout;

  transaction.priority = binding.priority;

  transaction.require_complete_frame = true;

  transaction.response_framer = request.response_framer;

  /*
   * 当前实机 RS485 profile 均为 Modbus RTU。
   * 调用方没有提供 framer 时按当前请求构造。
   */
  if (!transaction.response_framer) {
    transaction.response_framer =
        protocols::
            make_modbus_response_framer(
                request.payload,
                !is_ks114_logical_device(
                    request.logical_device));
  }

  if (!transaction.response_framer) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "unable to create rs485 response framer",
            "exchange",
            request.logical_device),
        {}};
  }

  transaction.side_effect_free_read =
      request.side_effect_free_read ||
      protocols::
          modbus_request_is_side_effect_free_read(
              request.payload);

  /*
   * 设备级公平性降级。
   */
  bool throttled = false;

  if (transaction.side_effect_free_read) {
    {
      std::lock_guard<std::mutex> lock(
          mutex_);

      throttled =
          device_should_yield_locked(
              request.logical_device,
              std::chrono::steady_clock::now());
    }

    if (throttled) {
      transaction.priority =
          std::numeric_limits<int>::max();
    }
  }

  const int requested_retries =
      request.retry_count >= 0
          ? request.retry_count
          : binding.read_retry_count;

  /*
   * 写事务强制不重试。
   */
  transaction.retry_count =
      transaction.side_effect_free_read
          ? std::clamp(
                requested_retries,
                0,
                kMaximumReadRetries)
          : 0;

  if (throttled &&
      transaction.retry_count > 0) {
    transaction.retry_count = 0;
  }

  transaction.deadline =
      make_deadline(
          transaction.timeout *
          (transaction.retry_count + 1));

  auto job = std::make_shared<PortJob>();

  job->transaction = transaction;

  auto future = job->completion.get_future();

  std::shared_ptr<PortJob>
      evicted_job;

  {
    std::lock_guard<std::mutex> lock(
        slot->queue_mutex);

    if (slot->worker_stop ||
        !slot->worker_running) {
      return {
          Result::failure(
              ErrorCode::kConfigRequired,
              "rs485 port worker is not running",
              "exchange",
              request.logical_device,
              true),
          {}};
    }

    /*
     * 同设备无副作用读取只允许有一个排队任务。
     */
    if (transaction.side_effect_free_read) {
      const auto duplicate =
          std::find_if(
              slot->queue.begin(),
              slot->queue.end(),
              [&](const auto& queued_job) {
                return queued_job &&
                       queued_job->transaction
                               .side_effect_free_read &&
                       queued_job->transaction
                               .logical_device ==
                           transaction.logical_device;
              });

      if (duplicate != slot->queue.end()) {
        return {
            Result::failure(
                ErrorCode::kIoError,
                "duplicate rs485 device read already queued",
                "queue",
                request.logical_device,
                true),
            {}};
      }
    }

    const std::size_t max_pending =
        slot->port->config()
            .max_pending_transactions;

    if (slot->queue.size() >= max_pending) {
      /*
       * 队列按优先级排列，末尾是当前最不紧急任务。
       * 新任务更紧急时允许淘汰队尾任务。
       */
      const auto& worst = slot->queue.back();

      if (worst &&
          transaction_precedes(
              transaction,
              worst->transaction)) {
        evicted_job = worst;

        slot->queue.pop_back();
      } else {
        return {
            Result::failure(
                ErrorCode::kIoError,
                "rs485 port queue is full",
                "queue",
                request.logical_device,
                true),
            {}};
      }
    }

    auto insert_at = slot->queue.end();

    for (auto it =
             slot->queue.begin();
         it != slot->queue.end();
         ++it) {
      if (*it &&
          transaction_precedes(
              transaction,
              (*it)->transaction)) {
        insert_at = it;
        break;
      }
    }

    slot->queue.insert(
        insert_at,
        job);
  }

  if (evicted_job) {
    evicted_job->completion.set_value(
        {
            cancelled_result(
                evicted_job->transaction,
                "lower-priority rs485 task evicted"),
            {}});
  }

  slot->queue_cv.notify_one();

  if (has_deadline(transaction.deadline)) {
    if (future.wait_until(transaction.deadline) ==
        std::future_status::timeout) {
      const bool already_started =
          job->started.load(
              std::memory_order_acquire);

      const auto timeout_result =
          Result::failure(
              ErrorCode::kRs485Timeout,
              already_started
                  ? "rs485 transaction exceeded deadline after start"
                  : "rs485 transaction completion wait exceeded deadline",
              already_started
                  ? "transaction"
                  : "arbitration",
              request.logical_device,
              true);

      /*
       * 让 worker 放弃这笔已经没人等待的纯读取。
       * 写事务不能被放弃。
       */
      if (job->transaction.side_effect_free_read) {
        job->abandoned.store(
            true,
            std::memory_order_release);
      }

      return {
          timeout_result,
          {}};
    }
  } else {
    future.wait();
  }

  const auto completed = future.get();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    record_device_result_locked(
        binding,
        completed.result);
  }

  return {
      completed.result,
      completed.response};
}

/* ============================================================
 * 每端口 worker
 * ============================================================ */

void Rs485TransactionScheduler::worker_loop(
    const std::shared_ptr<PortSlot>& slot) {
  diagnostics::info(
      "rs485.scheduler",
      "worker started for " +
          slot->port_id);

  while (true) {
    std::shared_ptr<PortJob> job;

    {
      std::unique_lock<std::mutex> lock(
          slot->queue_mutex);

      slot->queue_cv.wait(
          lock,
          [&]() {
            return slot->worker_stop ||
                   !slot->queue.empty();
          });

      if (slot->worker_stop &&
          slot->queue.empty()) {
        break;
      }

      job = slot->queue.front();

      slot->queue.pop_front();
    }

    if (!job) {
      continue;
    }

    /*
     * 调用方已经放弃的纯读取不再占用总线执行。
     * 写事务不受影响，永远执行。
     */
    if (job->transaction.side_effect_free_read &&
        job->abandoned.load(
            std::memory_order_acquire)) {
      job->completion.set_value(
          {
              cancelled_result(
                  job->transaction,
                  "abandoned rs485 read skipped"),
              {}});

      continue;
    }

    job->started.store(
        true,
        std::memory_order_release);

    try {
      job->completion.set_value(
          transact_direct(
              *slot,
              job->transaction));
    } catch (...) {
      job->completion.set_value(
          {
              Result::failure(
                  ErrorCode::kIoError,
                  "rs485 worker transaction threw exception",
                  "worker",
                  job->transaction
                      .logical_device,
                  true),
              {}});
    }
  }

  diagnostics::info(
      "rs485.scheduler",
      "worker stopped for " +
          slot->port_id);
}

/* ============================================================
 * 软件安全停止的队列处理
 * ============================================================ */

void Rs485TransactionScheduler::safe_stop()
    noexcept {
  std::vector<std::shared_ptr<PortSlot>>
      slots;

  {
    std::lock_guard<std::mutex> lock(
        mutex_);

    for (const auto& item :
         ports_) {
      slots.push_back(
          item.second);
    }
  }

  for (const auto& slot :
       slots) {
    std::vector<std::shared_ptr<PortJob>>
        cancelled_jobs;

    {
      std::lock_guard<std::mutex> lock(
          slot->queue_mutex);

      auto it =
          slot->queue.begin();

      while (it !=
             slot->queue.end()) {
        const auto& job =
            *it;

        const bool cancel =
            job &&
            (job->transaction
                     .side_effect_free_read ||
             job->transaction.priority >=
                 kSensorPriorityFloor);

        if (!cancel) {
          ++it;
          continue;
        }

        cancelled_jobs.push_back(
            job);

        it =
            slot->queue.erase(it);
      }
    }

    for (const auto& job :
         cancelled_jobs) {
      job->completion.set_value(
          {
              cancelled_result(
                  job->transaction,
                  "rs485 read cancelled by safe stop"),
              {}});
    }

    slot->queue_cv.notify_all();
  }
}

/* ============================================================
 * 停止调度器
 * ============================================================ */

void Rs485TransactionScheduler::stop()
    noexcept {
  std::vector<std::shared_ptr<PortSlot>>
      slots;

  std::vector<std::shared_ptr<TransactionArbiter>>
      arbiters;

  {
    std::lock_guard<std::mutex> lock(
        mutex_);

    if (ports_.empty()) {
      health_ =
          DriverHealth::kUnconfigured;

      return;
    }

    health_ =
        DriverHealth::kStopped;

    for (const auto& item :
         ports_) {
      slots.push_back(
          item.second);
    }

    for (const auto& item :
         shared_arbiters_) {
      arbiters.push_back(
          item.second);
    }
  }

  /*
   * 先唤醒正在等待共享事务门禁的 worker。
   */
  for (const auto& arbiter :
       arbiters) {
    arbiter->stop();
  }

  for (const auto& slot :
       slots) {
    std::vector<std::shared_ptr<PortJob>>
        cancelled_jobs;

    {
      std::lock_guard<std::mutex> lock(
          slot->queue_mutex);

      slot->worker_stop =
          true;

      while (!slot->queue.empty()) {
        cancelled_jobs.push_back(
            slot->queue.front());

        slot->queue.pop_front();
      }
    }

    for (const auto& job :
         cancelled_jobs) {
      if (!job) {
        continue;
      }

      job->completion.set_value(
          {
              cancelled_result(
                  job->transaction,
                  "rs485 scheduler stopped"),
              {}});
    }

    slot->queue_cv.notify_all();
  }

  /*
   * 当前正在执行的底层事务按自身 deadline 退出。
   */
  for (const auto& slot :
       slots) {
    if (slot->worker.joinable()) {
      slot->worker.join();
    }
  }

  for (const auto& slot : slots) {
    slot->port->close();

    {
      std::lock_guard<std::mutex> lock(
          slot->queue_mutex);

      slot->worker_running =
          false;

      slot->worker_stop =
          false;
    }

    slot->open_failed =
        false;

    slot->next_open_attempt = {};

    slot->last_open_error.clear();

    slot->consecutive_transport_failures = 0;

  }

  {
    std::lock_guard<std::mutex> lock(
        mutex_);

    for (auto& item :
         device_failures_) {
      item.second =
          DeviceFailureState{};
    }

    health_ =
        DriverHealth::kStopped;
  }
}

/* ============================================================
 * 状态查询
 * ============================================================ */

DriverHealth
Rs485TransactionScheduler::health()
    const noexcept {
  std::lock_guard<std::mutex> lock(
      mutex_);

  return health_;
}

std::vector<std::string>
Rs485TransactionScheduler::port_ids()
    const {
  std::lock_guard<std::mutex> lock(
      mutex_);

  std::vector<std::string> ids;

  ids.reserve(
      ports_.size());

  for (const auto& item :
       ports_) {
    ids.push_back(
        item.first);
  }

  return ids;
}

/* ============================================================
 * 单设备协议失败隔离
 * ============================================================ */

bool Rs485TransactionScheduler::
device_quarantined_locked(
    const std::string& logical_device,
    std::chrono::steady_clock::time_point now)
    const {
  if (!quarantine_devices_) {
    return false;
  }

  const auto it =
      device_failures_.find(
          logical_device);

  if (it ==
      device_failures_.end()) {
    return false;
  }

  return has_deadline(
             it->second
                 .quarantine_until) &&
         now <
             it->second
                 .quarantine_until;
}

bool Rs485TransactionScheduler::
device_should_yield_locked(
    const std::string& logical_device,
    std::chrono::steady_clock::time_point now)
    const {
  const auto it =
      device_failures_.find(
          logical_device);

  if (it ==
      device_failures_.end()) {
    return false;
  }

  const auto& state =
      it->second;

  /*
   * 未达到阈值：正常参与调度。
   */
  if (state.consecutive_failures <
      kDeviceStarvationFailures) {
    return false;
  }

  /*
   * 没有安排放行节拍：视为可正常参与。
   */
  if (!has_deadline(
          state.next_release_attempt)) {
    return false;
  }

  /*
   * 仍在限流窗口内：降级为最低优先级。
   */
  return now <
         state.next_release_attempt;
}
void Rs485TransactionScheduler::
record_device_result_locked(
    const Rs485DeviceBinding& binding,
    const Result& result) {
  auto& state =
      device_failures_[
          binding.logical_device];

  const auto now =
      std::chrono::steady_clock::now();

  if (result.ok) {
    /*
     * 成功一次即完全恢复：
     * - 失败计数清零 → 软限流自动解除
     * - 限流窗口清空
     * - 下一轮轮询立刻回到配置的 poll_interval
     */
    state.consecutive_failures =
        0;

    state.quarantine_until = {};

    state.next_release_attempt = {};

    /* 通信恢复：隔离时长退回配置的基础值。 */
    state.quarantine_streak =
        0;

    return;
  }

  /*
   * 设备类故障计数。
   */
  if (result.operation == "arbitration" ||
      result.operation == "queue" ||
      result.operation == "cancel" ||
      result.operation == "isolation" ||
      result.operation == "quarantine") {
    return;
  }

  if (!should_count_for_quarantine(
          result)) {
    return;
  }

  ++state.consecutive_failures;

  /*
   * 软限流窗口：连续失败达到阈值后开始生效。
   */
  if (state.consecutive_failures >=
      kDeviceStarvationFailures) {
    state.next_release_attempt =
        now + kDeviceThrottleInterval;
  }

  if (!quarantine_devices_) {
    return;
  }

  const int threshold =
      binding.max_consecutive_failures > 0
          ? binding.max_consecutive_failures
          : quarantine_after_failures_;

  if (state.consecutive_failures <
      threshold) {
    return;
  }

  auto duration =
      binding.quarantine_duration.count() > 0
          ? binding.quarantine_duration
          : quarantine_duration_;

  /*
   * 隔离时长逐级翻倍（上限 kMaxQuarantineDuration）。
   */
  if (duration.count() > 0) {
    ++state.quarantine_streak;

    for (int level = 1;
         level < state.quarantine_streak;
         ++level) {
      if (duration >=
          kMaxQuarantineDuration) {
        break;
      }

      duration *= 2;
    }

    duration =
        std::min(
            duration,
            kMaxQuarantineDuration);
  }

  state.quarantine_until =
      now + duration;

  diagnostics::warn(
      "rs485.scheduler",
      "quarantined device after failures: " +
          binding.logical_device +
          " duration_ms=" +
          std::to_string(
              duration.count()) +
          " streak=" +
          std::to_string(
              state.quarantine_streak));
}

/* ============================================================
 * 在 worker 中执行底层事务
 * ============================================================ */

Rs485TransactionResult
Rs485TransactionScheduler::transact_direct(
    PortSlot& slot,
    const Rs485Transaction& transaction) {
  const std::string endpoint =
      transaction.port_id +
      "/" +
      transaction.logical_device;

  const auto run_attempt =
      [&](const Rs485Transaction& attempt)
      -> Rs485TransactionResult {
        if (has_deadline(
                attempt.deadline) &&
            std::chrono::steady_clock::now() >=
                attempt.deadline) {
          return {
              Result::failure(
                  ErrorCode::kRs485Timeout,
                  "rs485 transaction expired before execution",
                  "deadline",
                  endpoint,
                  true),
              {}};
        }

        /*
         * 本端口锁在共享 WCH 门禁之前取得。
         * 同一个端口不会出现两个底层事务。
         */
        std::unique_lock<std::mutex>
            port_lock(
                slot.transaction_mutex);

        /*
         * 取到端口锁后必须再次校验 deadline。
         */
        if (has_deadline(attempt.deadline) &&
            std::chrono::steady_clock::now() >=
                attempt.deadline) {
          return {
              Result::failure(
                  ErrorCode::kRs485Timeout,
                  "rs485 transaction deadline expired waiting for port",
                  "arbitration",
                  endpoint,
                  true),
              {}};
        }

        std::uint64_t ticket = 0;
        bool gate_acquired = false;

        if (slot.shared_arbiter) {
          /*
           * 共享门禁的等待上限必须受本次事务的绝对 deadline 约束。
           */
          const auto gate_deadline =
              has_deadline(attempt.deadline)
                  ? attempt.deadline
                  : (std::chrono::steady_clock::now() +
                     attempt.timeout);

          const auto acquired =
              slot.shared_arbiter->acquire(
                  gate_deadline,
                  &ticket);

          if (acquired ==
              TransactionArbiter:: AcquireResult::kTimedOut) {
            return {
                Result::failure(
                    ErrorCode::kRs485Timeout,
                   "shared rs485 transaction gate timed out",
                   "arbitration",
                    endpoint,
                   true),
                {}};
           }

          if (acquired == TransactionArbiter:: AcquireResult::kStopped) {
            return {
                Result::failure(
                    ErrorCode::kIoError,
                    "shared rs485 transaction gate stopped",
                    "arbitration",
                    endpoint,
                    true),
                {}};
          }

          gate_acquired = true;
        }

        const auto release_gate =
            [&]() noexcept {
              if (gate_acquired) {
                slot.shared_arbiter->release(
                    ticket);

                gate_acquired =
                    false;
              }
            };

        try {
          const auto now =
              std::chrono::steady_clock::now();

          if (slot.open_failed &&
              has_deadline(
                  slot.next_open_attempt) &&
              now <
                  slot.next_open_attempt) {
            release_gate();

            return {
                Result::failure(
                    ErrorCode::kIoError,
                    "rs485 port reopen cooldown active",
                    "open",
                    endpoint,
                    true),
                {}};
          }

          /* 每笔事务前确认端口：open() 校验 fd 与串口参数，失效时就地重开重配。 */
          constexpr bool kVerifyPortBeforeTransaction = true;

          if (kVerifyPortBeforeTransaction) {
            const auto opened =
                slot.port->open();

            if (!opened.ok) {
              slot.open_failed =
                  true;

              slot.last_open_error =
                  opened.detail;

              slot.next_open_attempt =
                  std::chrono::steady_clock::now() +
                  slot.port->config()
                      .recovery_delay;

              release_gate();

              return {
                  Result::failure(
                      opened.code,
                      "rs485 port unavailable: " +
                          opened.detail,
                      "open",
                      endpoint,
                      true),
                  {}};
            }

            slot.open_failed = false;

            slot.last_open_error.clear();

            slot.next_open_attempt = {};
            slot.consecutive_transport_failures = 0;
          }

          Rs485Transaction execution = attempt;

          /*
           * 设备级事务的 deadline。
           *
           * 事务组门禁中的排队时间不属于设备响应时间：
           * 只有真正拿到物理总线之后，才开始本次 Modbus
           * 事务的 deadline 计时。
           */
          execution.deadline =
              std::chrono::steady_clock::now() +
              execution.timeout;

          /*
           * ========================================================
           * 墙钟兜底：单次事务绝不允许无限期占用端口锁
           *
           * 端口锁 transaction_mutex 串行化了同一物理串口上的
           * 全部设备。正常情况下 deadline 保证每个事务有界退出；
           * 但若驱动层在 poll / recovery / 迟到帧恢复中卡住，
           * 端口锁会被长期占住，同端口其他健康设备全部排队饿死
           * —— 这就是"一条线上一个设备拖住其他设备"。
           *
           * 做法：把 execution.deadline 再收紧为
           *   min(原 deadline, now + kMaxTransactionWallClock)
           * 复用 Rs485Port 内部已有的 deadline 机制执行，
           * 不在本层引入异步取消。
           * ========================================================
           */
          if (!has_deadline(
                  execution.deadline) ||
              execution.deadline >
                  std::chrono::steady_clock::now() +
                      kMaxTransactionWallClock) {
            execution.deadline =
                std::chrono::steady_clock::now() +
                kMaxTransactionWallClock;
          }

/*
 * ============================================================
 * 诊断日志：事务发送意图
 *
 * 注意：
 * tx_id 只用于日志关联。
 * 不进入 Rs485Transaction / Rs485Request，
 * 不参与调度、重试、超时或任何控制判断。
 * ============================================================
 */
                    const auto diagnostic_tx_id =
                        diagnostics::begin_rs485_diagnostic_transaction(
                            execution.logical_device,
                            endpoint);

                    diagnostics::debug(
                        "rs485.scheduler",
                        "tx_intent"
                        " tx_id=" +
                            std::to_string(
                        diagnostic_tx_id) +
                        " logical_device=" +
                        execution.logical_device +
                        " endpoint=" +
                        endpoint +
                        " bytes=" +
                       std::to_string(
                            execution.request.size()) +
                        " timeout_ms=" +
                        std::to_string(
                            execution.timeout.count()));

                    diagnostics::trace_packet(
                        "rs485",
                        "tx",
                        endpoint,
                        execution.request,
                        "tx_id=" +
                            std::to_string(
                                diagnostic_tx_id) +
                            " timeout_ms=" +
                            std::to_string(
                                execution.timeout.count()));

                    auto result = slot.port->transact(execution);

          if (result.result.ok) {slot.consecutive_transport_failures =0;

            diagnostics::trace_packet(
                "rs485",
                "rx",
                endpoint,
                result.response,
                "tx_id=" +
                    std::to_string(diagnostic_tx_id) +
                " bytes=" +
                std::to_string(result.response.size()));

            diagnostics::debug(
                "rs485.scheduler",
                "rx_frame_received"
                " tx_id=" +
                    std::to_string(diagnostic_tx_id) +
                " logical_device=" +
                execution.logical_device +
                " endpoint=" +
                endpoint +
                " response_bytes=" +
                std::to_string(result.response.size()));
          } else {
            diagnostics::warn(
                "rs485.scheduler",
                "rx_frame_failed"
                " tx_id=" +
                    std::to_string(diagnostic_tx_id) +
                  " logical_device=" +
                  execution.logical_device +
                  " endpoint=" + endpoint + " code=" +
                  std::to_string(
                      static_cast<int>(result.result.code)) +
                  " detail=" +
                  result.result.detail);

            diagnostics::warn(
                "rs485.scheduler",
                "transaction failed " +
                    endpoint +
                    ": " +
                    result.result.detail);

            /*
             * 设备无响应/帧错误属于 logical device 故障，不能累计成
             * 物理端口故障。
             *
             * 否则同一串口上的一个死设备连续超时后，这里会主动
             * close() 整个端口，把同端口其他健康设备一起拖下线，
             * 这是"一个设备失败拖垮其他设备"的直接来源。
             *
             * Rs485Port 只有在 fd/poll 等端口级 I/O 错误时才会进入
             * kFault；普通 timeout、短帧、CRC 或地址错误都保持端口
             * 可用，并由设备级失败隔离处理。
             */
            if (slot.port->health() ==
                DriverHealth::kFault) {
              ++slot.consecutive_transport_failures;

              if (slot.consecutive_transport_failures >=
                  kPortReopenFailureThreshold) {
                diagnostics::warn(
                    "rs485.scheduler",
                    "reopening isolated port after consecutive transport failures: " +
                        slot.port_id);

                slot.port->close();

                slot.open_failed = true;
                slot.last_open_error =
                    result.result.detail;

                slot.next_open_attempt =
                    std::chrono::steady_clock::now() +
                    kPortReopenCooldown;

                slot.consecutive_transport_failures =
                    0;
              }
            }
          }

          diagnostics::end_rs485_diagnostic_transaction();

          release_gate();

          return result;
        } catch (...) {
          diagnostics::end_rs485_diagnostic_transaction();

          release_gate();
          throw;
        }
      };

  Rs485TransactionResult last_result{
      Result::failure(
          ErrorCode::kRs485Timeout,
          "rs485 transaction not attempted",
          "transaction",
          endpoint,
          true),
      {}};

  const int attempts =
      std::max(
          1,
          transaction.retry_count + 1);

  for (int attempt_index = 0;
       attempt_index < attempts;
       ++attempt_index) {
    if (has_deadline(
            transaction.deadline) &&
        std::chrono::steady_clock::now() >=
            transaction.deadline) {
      break;
    }

    Rs485Transaction attempt =
        transaction;

    const auto single_deadline =
        std::chrono::steady_clock::now() +
        transaction.timeout;

    attempt.deadline =
        has_deadline(
            transaction.deadline)
            ? std::min(
                  single_deadline,
                  transaction.deadline)
            : single_deadline;

    last_result =
        run_attempt(
            attempt);

    if (last_result.result.ok) {
      return last_result;
    }

    if (!transaction.side_effect_free_read ||
        !should_retry_read(
            last_result.result) ||
        attempt_index + 1 >=
            attempts) {
      return last_result;
    }

    /*
     * 重试退避不占用 WCH 事务门禁。
     */
    std::this_thread::sleep_for(
        std::chrono::milliseconds(2));
  }

  return last_result;
}

}  // namespace snappy::drivers::bus
