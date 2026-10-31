#include "snappy/gateway/status_publisher.hpp"

#include <utility>

#include "snappy/gateway/g20/g20_packet_codec.hpp"

namespace snappy::gateway {

StatusPublisher::StatusPublisher(std::string robot_id)
    : robot_id_(std::move(robot_id)) {}

bool StatusPublisher::publish(ITransport& transport,
                              const StatusSnapshot& snapshot) const {
  return publish_raw(
      transport,
      g20::encode_status(robot_id_,
                         snapshot.state,
                         snapshot.ready_for_arm,
                         snapshot.g20_online,
                         snapshot.lease_held,
                         snapshot.detail));
}

bool StatusPublisher::publish_raw(ITransport& transport,
                                  const std::string& payload) const {
  return transport.publish(PublishedMessage{
      "snappy/v1/robots/" + robot_id_ + "/up/state",
      payload,
      0,
      false,
  });
}

}  // namespace snappy::gateway