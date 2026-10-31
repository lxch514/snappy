#pragma once
/**
 * @file model.hpp
 * @brief 控制层基础模型定义
 *
 * @details
 * - 定义链路标识、控制状态、应答状态和命令元数据。
 * - 租约申请与租约视图也在这里描述，供控制与网关两侧共用。
 */

#include <cstdint>
#include <optional>
#include <string>

namespace snappy::control {

enum class LinkId {
  kG20Udp,
  kEc20Mqtt,
};

enum class ControlState {
  kBoot,
  kSelfTest,
  kDisarmed,
  kAcquired,
  kArmedIdle,
  kActive,
  kSafeHold,
  kFaultLatched,
  kEstopLatched,
};

enum class AckStatus {
  kAccepted,
  kRunning,
  kSucceeded,
  kFailed,
  kAborted,
  kTimedOut,
};

struct CommandMetadata {
  std::string robot_id;
  LinkId link_id{LinkId::kG20Udp};
  std::string source_id;
  std::string boot_id;
  std::string session_id;
  std::string message_id;
  std::uint64_t sequence{0};
  std::uint64_t issued_at_ms{0};
  std::uint32_t ttl_ms{0};
  std::uint64_t control_epoch{0};
  std::optional<std::string> lease_id;
};

struct LeaseRequest {
  LinkId link_id{LinkId::kG20Udp};
  std::string source_id;
  std::string session_id;
};

struct LeaseView {
  bool held{false};
  LinkId owner{LinkId::kG20Udp};
  std::string lease_id;
  std::uint64_t control_epoch{0};
};

}  // namespace snappy::control
