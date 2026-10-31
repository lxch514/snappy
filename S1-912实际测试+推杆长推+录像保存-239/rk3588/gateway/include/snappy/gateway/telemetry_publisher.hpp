#pragma once

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