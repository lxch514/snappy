#include "snappy_snappy/direct_rs485_bus.hpp"

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

thread_local DirectRs485Bus::Scope*
    DirectRs485Bus::current_scope_ = nullptr;

DirectRs485Bus::Scope::Scope(
    DirectRs485Bus& bus,
    Rs485AccessKind kind,
    std::chrono::milliseconds budget)
    : bus_(bus),
      kind_(kind),
      generation_(
          bus.cancellation_generation_.load(
              std::memory_order_acquire)),
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
  return generation_ !=
         bus_.cancellation_generation_.load(
             std::memory_order_acquire);
}

void DirectRs485Bus::Scope::record(
    const Result& result) {
  if (!result.ok && result_.ok) {
    result_ = result;
  }
}

DirectRs485Bus::DirectRs485Bus(
    std::map<std::string, Rs485PortConfig> configs)
    : scheduler_(
          std::make_unique<
              Rs485TransactionScheduler>()) {
  /*
   * 调试执行器程序允许某个串口暂时不可用，
   * 但不会影响其他已打开串口。
   */
  scheduler_->set_allow_missing_ports(true);

  /*
   * ============================================================
   * 第三轮关键修复：打开调度器级「设备硬隔离」
   * ============================================================
   *
   * 这个开关以前只有 CLI 调试程序会调用
   * （device_registry.cpp 里的 scheduler->set_device_quarantine）。
   * snappy-real-control 这条真实路径从来没有调用过它，
   * 所以 Rs485TransactionScheduler 的 quarantine_devices_
   * 一直是头文件里的默认值 false，于是：
   *
   *   device_quarantined_locked() 永远返回 false
   *     -> 连续失败的设备永远不会被摘出队列
   *     -> 它每一轮都照旧去抢共享 WCH 门禁
   *
   * 这正好解释 2026-09-18 现场日志：
   * brush_motor 每 3s 左右失败一笔，每一笔都占满
   * timeout * (retry + 1) 的共享门禁时间；
   * 而同门禁上的激光 / 风速计只有 120ms 预算，
   * 于是被挤到 stale_after_ms(1000ms) 之外，
   * 日志里就是 sensor read failed +
   * "shared rs485 transaction gate timed out"。
   *
   * 打开之后的行为：
   * - 阈值与时长仍按每台设备自己的配置走：
   *   binding.max_consecutive_failures（配置为 3）、
   *   binding.quarantine_duration（配置为 2000ms）；
   *   下面这两个参数只是配置缺省时的兜底值；
   * - 隔离窗口内直接拒绝，完全不碰总线；
   * - 一次成功通信立即解除隔离，正常设备不受影响；
   * - 配合调度器里已经加好的隔离时长指数退避，
   *   持续故障设备占用共享门禁的比例降到 1% 量级。
   */
  scheduler_->set_device_quarantine(
      true,
      3,
      std::chrono::milliseconds(2000));

  for (const auto& item : configs) {
    const auto result =
        scheduler_->add_port(item.second);

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

  const auto registered =
      scheduler_->bind_logical_device(binding);

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

Rs485Response DirectRs485Bus::exchange(
    const Rs485Request& request) {
  const auto binding =
      find_binding(request.logical_device);

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

  const bool safety =
      scope != nullptr &&
      scope->kind() == Rs485AccessKind::kSafety;

  {
    std::lock_guard<std::mutex> lock(binding->mutex);

    if (!safety &&
        Clock::now() < binding->quarantine_until) {
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

  /*
   * 实际串口 I/O 只在 Rs485TransactionScheduler
   * 为该物理端口创建的独立 worker 中执行。
   */
  auto response = scheduler_->exchange(request);

  if (scope != nullptr) {
    scope->record(response.result);
  }

  if (response.result.ok) {
    record_success(binding);
  } else if (counts_as_device_failure(response.result)) {
    /*
     * 只记录当前 logical device。
     * 不关闭物理端口，不影响同端口其他设备。
     */
    record_failure(
        binding,
        response.result);
  }

  return response;
}

bool DirectRs485Bus::counts_as_device_failure(
    const Result& result) noexcept {
  if (result.ok ||
      result.operation == "queue" ||
      result.operation == "cancel" ||
      result.code == ErrorCode::kSafeHoldActive ||
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

  binding->online = true;
  binding->last_success = Clock::now();
  binding->consecutive_failures = 0;
  binding->last_error.clear();
  binding->quarantine_until = {};
}

void DirectRs485Bus::record_failure(
    const std::shared_ptr<BindingState>& binding,
    const Result& result) {
  std::lock_guard<std::mutex> lock(binding->mutex);

  binding->last_error =
      result.detail.empty()
          ? "RS485 transaction failed"
          : result.detail;

  ++binding->consecutive_failures;

  const int threshold =
      std::max(
          1,
          binding->binding.max_consecutive_failures);

  if (binding->consecutive_failures < threshold) {
    return;
  }

  binding->online = false;
  binding->quarantine_until =
      Clock::now() +
      binding->binding.quarantine_duration;
  binding->consecutive_failures = 0;

  SNAPPY_DRIVER_LOG_WARN(
      "direct-rs485",
      "logical device isolated: " +
          binding->binding.logical_device +
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
      result.age_ms <= 3000;

  result.consecutive_failures = binding->consecutive_failures;

  result.last_error = binding->last_error;

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

  for (auto& item : bindings_) {
    std::lock_guard<std::mutex>
        binding_lock(item.second->mutex);

    item.second->online = false;
    item.second->last_success = {};
    item.second->consecutive_failures = 0;
    item.second->last_error.clear();
    item.second->quarantine_until = {};
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
  }
}

}  // namespace snappy_snappy