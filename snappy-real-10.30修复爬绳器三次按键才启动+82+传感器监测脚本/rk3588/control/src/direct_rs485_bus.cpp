#include "snappy_snappy/direct_rs485_bus.hpp"
/** @file direct_rs485_bus.cpp 无传感器直连 RS485 总线实现 */

#include <algorithm>
#include <chrono>
#include <utility>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy_snappy {

using snappy::drivers::DriverHealth;
using snappy::drivers::ErrorCode;
using snappy::drivers::Result;
using snappy::drivers::Rs485Request;
using snappy::drivers::Rs485Response;
using snappy::drivers::bus::Rs485DeviceBinding;
using snappy::drivers::bus::Rs485PortConfig;
using snappy::drivers::bus::Rs485TransactionScheduler;

using Clock = std::chrono::steady_clock;

namespace {

/* 设备隔离退避参数：隔离时长 = min(base << streak, kMaxIsolationDuration)。 */
constexpr auto kMaxIsolationDuration = std::chrono::milliseconds(30000);

constexpr int kMaxIsolationShift = 4;

/* 旧退避阶梯：仅在 DeviceBackoffPolicy::enabled == false 时使用。 */
std::chrono::milliseconds isolation_duration_for(
    std::chrono::milliseconds base,
    int streak) {
  if (base.count() <= 0) {
    base = std::chrono::milliseconds(2000);
  }

  const int shift = std::clamp(streak, 0, kMaxIsolationShift);

  const auto scaled = base * (1 << shift);

  return scaled < kMaxIsolationDuration
             ? scaled
             : kMaxIsolationDuration;
}

/* 新退避阶梯：第 round 轮失败后的隔离时长，超过 max_delay 截断。 */
std::chrono::milliseconds isolation_duration_for_round(
    const DeviceBackoffPolicy& policy,
    int round) {
  if (round <= 1) {
    return policy.initial_delay;
  }

  const int multiplier = std::max(1, policy.multiplier);

  auto delay = policy.initial_delay;

  for (int index = 1; index < round; ++index) {
    if (delay >= policy.max_delay) {
      return policy.max_delay;
    }

    const auto next = delay * multiplier;

    if (next >= policy.max_delay) {
      return policy.max_delay;
    }

    delay = next;
  }

  return delay;
}

/*
 * 永久离线设备的自动探测间隔。
 */
constexpr auto kOfflineProbeInterval = std::chrono::milliseconds(10000);

}  // namespace

thread_local DirectRs485Bus::Scope*
    DirectRs485Bus::current_scope_ = nullptr;

DirectRs485Bus::Scope::Scope(
    DirectRs485Bus& bus,
    Rs485AccessKind kind,
    std::chrono::milliseconds budget)
    : bus_(bus),
      kind_(kind),
      generation_(bus.cancellation_generation_.load(std::memory_order_acquire)),
      deadline_(Clock::now() + budget),
      previous_(DirectRs485Bus::current_scope_) {
  DirectRs485Bus::current_scope_ = this;
}

DirectRs485Bus::Scope::~Scope() {
  DirectRs485Bus::current_scope_ = previous_;
}

bool DirectRs485Bus::Scope::expired()
    const noexcept {
  return Clock::now() >= deadline_;
}

bool DirectRs485Bus::Scope::cancelled()
    const noexcept {
  return generation_ != bus_.cancellation_generation_.load(std::memory_order_acquire);
}

void DirectRs485Bus::Scope::record(const Result& result) {
  if (!result.ok && result_.ok) {
    result_ = result;
  }
}

DirectRs485Bus::DirectRs485Bus(
    std::map<std::string, Rs485PortConfig> configs,
    DeviceBackoffPolicy backoff)
    : backoff_(backoff),
      scheduler_(
          std::make_unique<
              Rs485TransactionScheduler>()) {
  /* 调试执行器程序允许某个串口暂时不可用，不影响其他已打开的串口。 */
  scheduler_->set_allow_missing_ports(true);

  /* 打开调度器级设备硬隔离：连续失败的设备摘出队列，不占满共享 WCH 事务门禁。 */
  scheduler_->set_device_quarantine(
      true,
      3,
      std::chrono::milliseconds(2000));

  for (const auto& item : configs) {
    const auto result = scheduler_->add_port(item.second);

    if (!result.ok) {
      SNAPPY_DRIVER_LOG_WARN(
          "snappy-snappy",
          "RS485 port registration failed: " +
              item.first +
              ": " +
              result.detail);
    }
  }
}

DirectRs485Bus::~DirectRs485Bus() {
  stop();
}

Result DirectRs485Bus::add_binding(
    const Rs485DeviceBinding& binding) {
  if (binding.logical_device.empty() ||
      binding.port_id.empty() ||
      binding.timeout.count() <= 0 ||
      binding.max_response_bytes == 0 ||
      binding.max_consecutive_failures <= 0 ||
      binding.quarantine_duration.count() < 0 ||
      binding.read_retry_count < 0) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "invalid RS485 device binding",
        "add_binding",
        binding.logical_device);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (bindings_.count(binding.logical_device) != 0) {
      return Result::failure(
          ErrorCode::kInvalidArgument,
          "duplicate RS485 device binding",
          "add_binding",
          binding.logical_device);
    }
  }

  const auto registered = scheduler_->bind_logical_device(binding);

  if (!registered.ok) {
    return registered;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    bindings_.emplace(
        binding.logical_device,
        std::make_shared<BindingState>(binding));
  }

  return Result::success(
      "RS485 device binding added");
}

std::shared_ptr<
    DirectRs485Bus::BindingState>
DirectRs485Bus::find_binding(
    const std::string& logical_device) const {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto iterator =
      bindings_.find(logical_device);

  if (iterator == bindings_.end()) {
    return nullptr;
  }

  return iterator->second;
}

Result DirectRs485Bus::start() {
  stopping_.store(false);

  const auto result =
      scheduler_->start();

  if (!result.ok) {
    return result;
  }

  return Result::success("RS485 ports started with independent workers");
}

Rs485Response DirectRs485Bus::exchange(const Rs485Request& request) {
  const auto binding = find_binding(request.logical_device);

  Scope* scope = current_scope_;

  const auto failure =
      [&](Result result) {
        if (scope != nullptr) {
          scope->record(result);
        }

        return Rs485Response{
            std::move(result),
            {}};
      };

  if (!binding) {
    return failure(
        Result::failure(
            ErrorCode::kConfigRequired,
            "RS485 device is not bound",
            "exchange",
            request.logical_device,
            true));
  }

  if (request.payload.empty()) {
    return failure(
        Result::failure(
            ErrorCode::kInvalidArgument,
            "RS485 request payload is empty",
            "exchange",
            request.logical_device));
  }

  if (stopping_.load()) {
    return failure(
        Result::failure(
            ErrorCode::kSafeHoldActive,
            "RS485 scheduler is stopping",
            "exchange",
            request.logical_device,
            true));
  }

  if (scope != nullptr) {
    if (scope->cancelled()) {
      return failure(
          Result::failure(
              ErrorCode::kSafeHoldActive,
              "RS485 transaction cancelled",
              "cancel",
              request.logical_device,
              true));
    }

    if (scope->expired()) {
      return failure(
          Result::failure(
              ErrorCode::kRs485Timeout,
              "RS485 access scope expired",
              "arbitration",
              request.logical_device,
              true));
    }
  }

  const bool safety = scope != nullptr && scope->kind() == Rs485AccessKind::kSafety;

  /* 本次是否为一次永久离线探测。 */
  bool offline_probe = false;

  {
    std::lock_guard<std::mutex> lock(binding->mutex);

    /* 永久离线设备：不再下发事务；kSafety 例外，到探测时刻也放行一帧。 */
    if (!safety &&
        binding->permanently_offline) {
      if (Clock::now() < binding->next_probe_at) {
        return failure(
            Result::failure(
                ErrorCode::kIoError,
                "RS485 device is permanently offline",
                "offline",
                request.logical_device,
                true));
      }

      offline_probe = true;
    }

    if (!safety && !offline_probe && Clock::now() < binding->quarantine_until) {
      const auto remaining =
          std::chrono::duration_cast<
              std::chrono::milliseconds>(
              binding->quarantine_until -
              Clock::now())
              .count();

      return failure(
          Result::failure(
              ErrorCode::kIoError,
              "RS485 device is isolated for " +
                  std::to_string(remaining) +
                  " ms",
              "isolation",
              request.logical_device,
              true));
    }
  }

  /* 串口 I/O 只在 Rs485TransactionScheduler 为该端口创建的独立 worker 中执行。 */
  auto response = scheduler_->exchange(request);

  if (scope != nullptr) {
    scope->record(response.result);
  }

  if (response.result.ok) {
    record_success(binding);
  } else if (counts_as_device_failure(response.result)) {
    /* 只记录当前 logical device，不关闭端口、不影响同端口其他设备。 */
    record_failure(
        binding,
        response.result);
  }

  if (offline_probe && !response.result.ok) {
    /* 探测失败：保持永久离线判定，只把下次探测推后。 */
    std::lock_guard<std::mutex> lock(binding->mutex);

    binding->next_probe_at = Clock::now() + kOfflineProbeInterval;
  }

  return response;
}

bool DirectRs485Bus::counts_as_device_failure(
    const Result& result) noexcept {
  if (result.ok) {
    return false;
  }

  /* 只有目标设备自身回错 / 不回才计入本设备失败，排队与隔离类不计入。 */
  if (result.operation == "queue" ||
      result.operation == "cancel" ||
      result.operation == "arbitration" ||
      result.operation == "exchange" ||
      result.operation == "isolation" ||
      result.operation == "quarantine" ||
      result.operation == "offline" ||
      result.operation == "deadline" ||
      result.operation == "open" ||
      result.operation == "worker") {
    return false;
  }

  if (result.code == ErrorCode::kSafeHoldActive ||
      result.code == ErrorCode::kConfigRequired ||
      result.code == ErrorCode::kInvalidArgument ||
      result.code == ErrorCode::kUnsupported) {
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
    case ErrorCode::kIoError:
      return true;

    default:
      return false;
  }
}

void DirectRs485Bus::record_success(
    const std::shared_ptr<BindingState>& binding) {
  std::lock_guard<std::mutex> lock(binding->mutex);

  const bool was_offline =
      binding->permanently_offline;

  binding->online = true;
  binding->last_success = Clock::now();
  binding->consecutive_failures = 0;
  binding->last_error.clear();
  binding->quarantine_until = {};

  /* 通信恢复：退避轮次与隔离时长都退回起点。 */
  binding->isolation_streak = 0;
  binding->isolation_round = 0;
  binding->permanently_offline = false;
  binding->next_probe_at = {};

  if (was_offline) {
    /* 永久离线设备通信成功即直接复活。 */
    SNAPPY_DRIVER_LOG_INFO(
        "direct-rs485",
        "logical device recovered from permanent offline: " +
            binding->binding.logical_device);
  }
}

void DirectRs485Bus::record_failure(
    const std::shared_ptr<BindingState>& binding,
    const Result& result) {
  std::lock_guard<std::mutex> lock(binding->mutex);

  binding->last_error =
      result.detail.empty()
          ? "RS485 transaction failed"
          : result.detail;

  if (binding->permanently_offline) {
    /* 已判定永久离线：不累计失败、不续期隔离窗口、不重复打日志。 */
    return;
  }

  ++binding->consecutive_failures;

  const int threshold =
      std::max(
          1,
          binding->binding.max_consecutive_failures);

  if (binding->consecutive_failures < threshold) {
    return;
  }

  /* 一轮结束：threshold 次连续失败。 */
  binding->consecutive_failures = 0;
  binding->online = false;

  ++binding->isolation_round;

  const bool enabled = backoff_.enabled;

  const int offline_round =
      std::max(1, backoff_.permanent_offline_round);

  const auto duration =
      enabled
          ? isolation_duration_for_round(
                backoff_,
                binding->isolation_round)
          : isolation_duration_for(
                binding->binding.quarantine_duration,
                binding->isolation_streak);

  if (enabled &&
      binding->isolation_round >= offline_round) {
    /* 连续 offline_round 轮都访问不到即判定永久离线。 */
    binding->permanently_offline = true;
    binding->quarantine_until = {};
    binding->next_probe_at =
        Clock::now() + kOfflineProbeInterval;

    SNAPPY_DRIVER_LOG_ERROR(
        "direct-rs485",
        "logical device offline (permanent): " +
            binding->binding.logical_device +
            " rounds=" +
            std::to_string(binding->isolation_round) +
            " failures_per_round=" +
            std::to_string(threshold) +
            ", detail=" +
            binding->last_error +
            " next_probe_ms=" +
            std::to_string(kOfflineProbeInterval.count()));

    return;
  }

  binding->quarantine_until =
      Clock::now() + duration;

  if (!enabled &&
      binding->isolation_streak <
          kMaxIsolationShift) {
    ++binding->isolation_streak;
  }

  /* 日志追加 duration_ms / round。 */
  SNAPPY_DRIVER_LOG_WARN(
      "direct-rs485",
      "logical device isolated: " +
          binding->binding.logical_device +
          " duration_ms=" +
          std::to_string(duration.count()) +
          " round=" +
          std::to_string(binding->isolation_round) +
          " streak=" +
          std::to_string(
              binding->isolation_streak) +
          ", detail=" +
          binding->last_error);
}

bool DirectRs485Bus::device_available(
    const std::string& logical_device) const {
  return device_status(logical_device).online;
}

Rs485DeviceStatus DirectRs485Bus::device_status(
    const std::string& logical_device) const {
  Rs485DeviceStatus result;

  const auto binding = find_binding(logical_device);

  if (!binding) {
    return result;
  }

  std::lock_guard<std::mutex> lock(binding->mutex);

  const auto now = Clock::now();

  const bool seen = binding->last_success.time_since_epoch().count() != 0;

  if (seen) {
    result.age_ms =
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                now - binding->last_success)
                .count());

    const auto system_now =
        std::chrono::system_clock::now();

    const auto success_system =
        system_now -
        std::chrono::duration_cast<
            std::chrono::system_clock::duration>(
            now - binding->last_success);

    result.last_success =
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                success_system.time_since_epoch())
                .count());
  }

  const bool isolated = now < binding->quarantine_until;

  if (isolated) {
    const auto system_now =
        std::chrono::system_clock::now();

    const auto quarantine_system =
        system_now +
        std::chrono::duration_cast<
            std::chrono::system_clock::duration>(
            binding->quarantine_until - now);

    result.quarantine_until =
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<
                std::chrono::milliseconds>(
                quarantine_system.time_since_epoch())
                .count());
  }

  result.online =
      seen &&
      binding->online &&
      !isolated &&
      !binding->permanently_offline &&
      result.age_ms <= 3000;

  result.consecutive_failures = binding->consecutive_failures;

  result.last_error = binding->last_error;

  result.isolation_round = binding->isolation_round;

  result.permanently_offline =
      binding->permanently_offline;

  return result;
}

void DirectRs485Bus::safe_stop() noexcept {
  cancellation_generation_.fetch_add(
      1,
      std::memory_order_acq_rel);

  if (scheduler_) {
    scheduler_->safe_stop();
  }
}

void DirectRs485Bus::clear_faults() noexcept {
  cancellation_generation_.fetch_add(
      1,
      std::memory_order_acq_rel);

  std::lock_guard<std::mutex> lock(mutex_);

  int revived = 0;

  for (auto& item : bindings_) {
    std::lock_guard<std::mutex>
        binding_lock(item.second->mutex);

    if (item.second->permanently_offline) {
      ++revived;
    }

    item.second->online = false;
    item.second->last_success = {};
    item.second->consecutive_failures = 0;
    item.second->last_error.clear();
    item.second->quarantine_until = {};
    item.second->isolation_streak = 0;
    item.second->isolation_round = 0;
    item.second->permanently_offline = false;
    item.second->next_probe_at = {};
  }

  if (revived > 0) {
    /* 清故障把永久离线设备放回探测队列。 */
    SNAPPY_DRIVER_LOG_INFO(
        "direct-rs485",
        "clear_faults revived permanently-offline devices: " +
            std::to_string(revived));
  }
}

DriverHealth DirectRs485Bus::health()
    const noexcept {
  if (!scheduler_) {
    return DriverHealth::kUnconfigured;
  }

  return scheduler_->health();
}

void DirectRs485Bus::stop() noexcept {
  const bool was_stopping = stopping_.exchange(true);

  cancellation_generation_.fetch_add(
      1,
      std::memory_order_acq_rel);

  if (scheduler_ && !was_stopping) {
    scheduler_->stop();
  }

  std::lock_guard<std::mutex> lock(mutex_);

  for (auto& item : bindings_) {
    std::lock_guard<std::mutex>
        binding_lock(item.second->mutex);

    item.second->online = false;
    item.second->quarantine_until = {};
    item.second->isolation_streak = 0;
    item.second->isolation_round = 0;
    item.second->permanently_offline = false;
    item.second->next_probe_at = {};
  }
}

}  // namespace snappy_snappy