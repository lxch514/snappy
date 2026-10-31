#pragma once

#include <optional>
#include <string>

#include "snappy/control/model.hpp"

namespace snappy::control {

// Stable business boundaries only. Concrete actuator behavior is intentionally
// absent until hardware protocols and safety thresholds are confirmed.
class ILeaseManager {
 public:
  virtual ~ILeaseManager() = default;
  virtual std::optional<LeaseView> acquire(const LeaseRequest& request) = 0;
  virtual bool renew(const CommandMetadata& metadata) = 0;
  virtual void revoke(std::string reason) noexcept = 0;
  [[nodiscard]] virtual LeaseView current() const = 0;
};

class ICommandDispatcher {
 public:
  virtual ~ICommandDispatcher() = default;
  virtual AckStatus dispatch(const CommandMetadata& metadata,
                             const std::string& action,
                             const std::string& params_json) = 0;
  virtual void cancel_all(std::string reason) noexcept = 0;
};

class ISafetySupervisor {
 public:
  virtual ~ISafetySupervisor() = default;
  [[nodiscard]] virtual bool arm_allowed() const = 0;
  virtual void enter_safe_hold(std::string reason) noexcept = 0;
  [[nodiscard]] virtual ControlState state() const noexcept = 0;
};

}  // namespace snappy::control
