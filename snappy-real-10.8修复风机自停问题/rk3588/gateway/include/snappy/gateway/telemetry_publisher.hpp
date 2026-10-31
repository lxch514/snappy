#pragma once
/**
 * @file telemetry_publisher.hpp
 * @brief 遥测发布声明
 *
 * @details
 * - 从控制进程快照中取出传感器数值后编码下发。
 * - 只读取快照，不触发任何硬件访问。
 */
 
#include <string>
#include <vector>

#include "snappy/drivers/interfaces.hpp"
#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway {

class TelemetryPublisher {
 public:
  explicit TelemetryPublisher(std::string robot_id);

  bool publish(ITransport& transport,
               const std::vector<drivers::SensorSample>& samples) const;
  bool publish_raw(ITransport& transport, const std::string& payload) const;

 private:
  std::string robot_id_;
};

}  // namespace snappy::gateway