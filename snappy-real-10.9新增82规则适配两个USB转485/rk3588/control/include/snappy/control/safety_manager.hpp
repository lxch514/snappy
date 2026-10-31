#pragma once
/**
 * @file safety_manager.hpp
 * @brief 控制安全状态机声明
 *
 * @details
 * - 管理解锁门禁、安全保持和故障清除。
 * - 进入安全保持后不自动恢复，必须显式重新取得控制权。
 */

#include <string>

#include "snappy/control/ports.hpp"

namespace snappy::control {

class SafetyManager final : public ISafetySupervisor {
 public:
  SafetyManager() = default;

  [[nodiscard]] bool arm_allowed() const override;
  void enter_safe_hold(std::string reason) noexcept override;
  [[nodiscard]] ControlState state() const noexcept override;

  void clear_for_disarmed() noexcept;
  [[nodiscard]] const std::string& reason() const noexcept;

 private:
  ControlState state_{ControlState::kDisarmed};
  std::string reason_;
};

}  // namespace snappy::control