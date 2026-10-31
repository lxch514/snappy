#pragma once

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