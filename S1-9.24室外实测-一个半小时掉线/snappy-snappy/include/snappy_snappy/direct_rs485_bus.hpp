#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "snappy/drivers/bus/rs485_transaction_scheduler.hpp"
#include "snappy/drivers/interfaces.hpp"
#include "snappy_snappy/actuator_snapshot.hpp"

namespace snappy_snappy {

enum class Rs485AccessKind {
  kCommand,
  kHealth,
  kRecovery,
  kSafety,
};

class DirectRs485Bus final
    : public snappy::drivers::IRs485Bus {
 public:
  explicit DirectRs485Bus(
      std::map<
          std::string,
          snappy::drivers::bus::Rs485PortConfig>
          configs);

  ~DirectRs485Bus() override;

  DirectRs485Bus(
      const DirectRs485Bus&) = delete;

  DirectRs485Bus& operator=(
      const DirectRs485Bus&) = delete;

  snappy::drivers::Result add_binding(
      const snappy::drivers::bus::Rs485DeviceBinding&
          binding);

  snappy::drivers::Result start() override;

  snappy::drivers::Rs485Response exchange(
      const snappy::drivers::Rs485Request&
          request) override;

  void safe_stop() noexcept override;

  void clear_faults() noexcept;

  [[nodiscard]] snappy::drivers::DriverHealth
  health() const noexcept override;

  [[nodiscard]] bool device_available(
      const std::string& logical_device) const;

  [[nodiscard]] Rs485DeviceStatus device_status(
      const std::string& logical_device) const;

  void stop() noexcept;

  class Scope {
   public:
    Scope(
        DirectRs485Bus& bus,
        Rs485AccessKind kind,
        std::chrono::milliseconds budget =
            std::chrono::milliseconds(5000));

    ~Scope();

    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;

    [[nodiscard]] Rs485AccessKind kind()
        const noexcept {
      return kind_;
    }

    [[nodiscard]] std::uint64_t generation()
        const noexcept {
      return generation_;
    }

    [[nodiscard]] bool expired()
        const noexcept;

    [[nodiscard]] bool cancelled()
        const noexcept;

    [[nodiscard]] std::chrono::steady_clock::time_point
    deadline() const noexcept {
      return deadline_;
    }

    [[nodiscard]] const snappy::drivers::Result&
    result() const noexcept {
      return result_;
    }

    void record(
        const snappy::drivers::Result&
            result);

   private:
    DirectRs485Bus& bus_;
    Rs485AccessKind kind_;
    std::uint64_t generation_{0};
    std::chrono::steady_clock::time_point deadline_{};
    snappy::drivers::Result result_{
        snappy::drivers::Result::success()};
    Scope* previous_{nullptr};
  };

 private:
  using Clock = std::chrono::steady_clock;

  struct BindingState {
    explicit BindingState(
        snappy::drivers::bus::Rs485DeviceBinding value)
        : binding(std::move(value)) {}

    snappy::drivers::bus::Rs485DeviceBinding binding;

    mutable std::mutex mutex;

    bool online{false};
    Clock::time_point last_success{};
    int consecutive_failures{0};
    std::string last_error;
    Clock::time_point quarantine_until{};
  };

  std::shared_ptr<BindingState> find_binding(
      const std::string& logical_device) const;

  void record_success(
      const std::shared_ptr<BindingState>&
          binding);

  void record_failure(
      const std::shared_ptr<BindingState>&
          binding,
      const snappy::drivers::Result&
          result);

  static bool counts_as_device_failure(
      const snappy::drivers::Result&
          result) noexcept;

  std::unique_ptr<
      snappy::drivers::bus::Rs485TransactionScheduler>
      scheduler_;

  mutable std::mutex mutex_;

  std::unordered_map<
      std::string,
      std::shared_ptr<BindingState>>
      bindings_;

  std::atomic<std::uint64_t>
      cancellation_generation_{0};

  std::atomic_bool stopping_{false};

  static thread_local Scope*
      current_scope_;
};

}  // namespace snappy_snappy