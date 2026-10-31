#pragma once
/**
 * @file lease_manager.hpp
 * @brief 控制权租约管理声明
 *
 * @details
 * - 负责控制权的取得、续租和撤销。
 * - 只有持有有效租约的上位机才能下发控制命令。
 */

#include <cstdint>
#include <string>

#include "snappy/control/ports.hpp"

namespace snappy::control {

class LeaseManager final : public ILeaseManager {
 public:
  LeaseManager() = default;

  std::optional<LeaseView> acquire(const LeaseRequest& request) override;
  bool renew(const CommandMetadata& metadata) override;
  void revoke(std::string reason) noexcept override;
  [[nodiscard]] LeaseView current() const override;

 private:
  LeaseView lease_{};
  std::string owner_source_id_;
  std::string owner_session_id_;
  std::string revoke_reason_;
};

}  // namespace snappy::control