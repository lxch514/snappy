/**
 * @file telemetry_publisher.cpp
 * @brief 遥测发布实现
 *
 * @details
 * - 只消费控制进程提供的快照，不直接访问硬件。
 */
 
#include "snappy/gateway/telemetry_publisher.hpp"

#include <utility>

#include "snappy/gateway/g20/g20_packet_codec.hpp"

namespace snappy::gateway {

TelemetryPublisher::TelemetryPublisher(std::string robot_id)
    : robot_id_(std::move(robot_id)) {}

bool TelemetryPublisher::publish(
    ITransport& transport,
    const std::vector<drivers::SensorSample>& samples) const {
  return publish_raw(transport, g20::encode_telemetry(robot_id_, samples));
}

bool TelemetryPublisher::publish_raw(ITransport& transport,
                                     const std::string& payload) const {
  return transport.publish(PublishedMessage{
      "snappy/v1/robots/" + robot_id_ + "/up/telemetry",
      payload,
      0,
      false,
  });
}

}  // namespace snappy::gateway