#include "snappy/control/safety_manager.hpp"
/**
 * @file safety_monitor.cpp
 * @brief 控制安全状态机实现
 *
 * @details
 * - 安全保持为终态，需重新取得控制权后才能解除。
 */

#include <utility>

namespace snappy::control {

bool SafetyManager::arm_allowed() const {
  return state_ == ControlState::kDisarmed ||
         state_ == ControlState::kArmedIdle;
}

void SafetyManager::enter_safe_hold(std::string reason) noexcept {
  state_ = ControlState::kSafeHold;
  reason_ = std::move(reason);
}

ControlState SafetyManager::state() const noexcept {
  return state_;
}

void SafetyManager::clear_for_disarmed() noexcept {
  state_ = ControlState::kDisarmed;
  reason_.clear();
}

const std::string& SafetyManager::reason() const noexcept {
  return reason_;
}

}  // namespace snappy::control