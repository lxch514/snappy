#pragma once

/**
 * @file command_mailbox.hpp
 * @brief 执行器最新命令邮箱
 *
 * @details
 * - 每个执行器域只有一个最新值槽。
 * - 新命令覆盖同一域的旧命令。
 * - 不使用 FIFO 队列。
 * - 不使用 future。
 * - 安全停止会清空所有尚未执行的普通命令。
 * - 物理 I/O 只由 ActuatorHub worker 执行。
 */

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>

namespace snappy_snappy {

enum class CommandSlot {
  kBrush,
  kBrushPush,
  kScraperPush,
  kClimber,
  kClearFaults,
};

struct PendingCommand {
  /*
   * 命令产生时的控制代次。
   * 安全停止后旧代次命令不能继续执行。
   */
  std::uint64_t generation{0};

  int direction{0};
  int speed_raw{0};
  double speed_rpm{0.0};

  std::uint32_t duration_ms{0};
  bool timed{false};
  bool force_retract_from_zero{false};
  bool force_extend_beyond_max{false};
  // Recovery push runs until the operator returns the left stick to center.
  bool hold_until_stop{false};
};

class CommandMailbox {
 public:
  bool submit(
      CommandSlot slot,
      const PendingCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (closed_) {
      return false;
    }

    switch (slot) {
      case CommandSlot::kBrush:
        brush_ = command;
        break;

      case CommandSlot::kBrushPush:
        brush_push_ = command;
        break;

      case CommandSlot::kScraperPush:
        scraper_push_ = command;
        break;

      case CommandSlot::kClimber:
        climber_ = command;
        break;

      case CommandSlot::kClearFaults:
        clear_faults_ = command;
        break;
    }

    condition_.notify_all();
    return true;
  }

  bool wait_take(
      CommandSlot slot,
      PendingCommand* command,
      std::chrono::milliseconds timeout) {
    if (command == nullptr) {
      return false;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    const bool awakened =
        condition_.wait_for(
            lock,
            timeout,
            [this, slot]() {
              return closed_ || has_slot_locked(slot);
            });

    if (!awakened || closed_) {
      return false;
    }

    auto* value = slot_ptr_locked(slot);

    if (value == nullptr || !value->has_value()) {
      return false;
    }

    *command = **value;
    value->reset();
    return true;
  }

  std::optional<PendingCommand> take(CommandSlot slot) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto* value = slot_ptr_locked(slot);

    if (value == nullptr || !value->has_value()) {
      return std::nullopt;
    }

    PendingCommand command = **value;
    value->reset();
    return command;
  }

  void request_urgent_stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      /*
       * 安全停止必须先清除旧命令。
       * 否则 worker 完成停止动作后可能继续执行旧动作。
       */
      urgent_stop_ = true;
      clear_slots_locked();
    }

    condition_.notify_all();
  }

  bool take_urgent_stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    const bool value = urgent_stop_;
    urgent_stop_ = false;
    return value;
  }

  bool wait_for_event(
      std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);

    return condition_.wait_for(
        lock,
        timeout,
        [this]() {
          return closed_ ||
                 urgent_stop_ ||
                 clear_faults_.has_value();
        });
  }

  bool closed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      closed_ = true;
      clear_slots_locked();
    }

    condition_.notify_all();
  }

 private:
  std::optional<PendingCommand>* slot_ptr_locked(
      CommandSlot slot) {
    switch (slot) {
      case CommandSlot::kBrush:
        return &brush_;

      case CommandSlot::kBrushPush:
        return &brush_push_;

      case CommandSlot::kScraperPush:
        return &scraper_push_;

      case CommandSlot::kClimber:
        return &climber_;

      case CommandSlot::kClearFaults:
        return &clear_faults_;
    }

    return nullptr;
  }

  bool has_slot_locked(CommandSlot slot) const {
    switch (slot) {
      case CommandSlot::kBrush:
        return brush_.has_value();

      case CommandSlot::kBrushPush:
        return brush_push_.has_value();

      case CommandSlot::kScraperPush:
        return scraper_push_.has_value();

      case CommandSlot::kClimber:
        return climber_.has_value();

      case CommandSlot::kClearFaults:
        return clear_faults_.has_value();
    }

    return false;
  }

  void clear_slots_locked() {
    brush_.reset();
    brush_push_.reset();
    scraper_push_.reset();
    climber_.reset();
    clear_faults_.reset();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;

  bool urgent_stop_{false};
  bool closed_{false};

  std::optional<PendingCommand> brush_;
  std::optional<PendingCommand> brush_push_;
  std::optional<PendingCommand> scraper_push_;
  std::optional<PendingCommand> climber_;
  std::optional<PendingCommand> clear_faults_;
};

}  // namespace snappy_snappy