#pragma once
/**
 * @file status_publisher.hpp
 * @brief 控制状态发布声明
 *
 * @details
 * - 把控制状态快照编码后经传输发出。
 * - 只在状态变化或到达周期时发送，不逐帧刷屏。
 */
 
#include <string>

#include "snappy/control/model.hpp"
#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway {

struct StatusSnapshot {
  control::ControlState state{control::ControlState::kSafeHold};
  bool ready_for_arm{false};
  bool g20_online{false};
  bool lease_held{false};
  std::string detail;
};

class StatusPublisher {
 public:
  explicit StatusPublisher(std::string robot_id);

  bool publish(ITransport& transport, const StatusSnapshot& snapshot) const;
  bool publish_raw(ITransport& transport, const std::string& payload) const;

 private:
  std::string robot_id_;
};

}  // namespace snappy::gateway