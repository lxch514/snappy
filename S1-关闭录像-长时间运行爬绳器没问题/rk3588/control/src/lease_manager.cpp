#include "snappy/control/lease_manager.hpp"

#include <sstream>
#include <utility>

namespace snappy::control {
namespace {

std::string make_lease_id(const LeaseRequest& request,
                          std::uint64_t epoch) {
  std::ostringstream out;
  out << "lease-" << static_cast<int>(request.link_id)
      << "-" << request.source_id
      << "-" << request.session_id
      << "-" << epoch;
  return out.str();
}

}  // namespace

std::optional<LeaseView> LeaseManager::acquire(const LeaseRequest& request) {
  if (lease_.held &&
      (request.link_id != lease_.owner ||
       request.source_id != owner_source_id_ ||
       request.session_id != owner_session_id_)) {
    return std::nullopt;
  }

  lease_.held = true;
  lease_.owner = request.link_id;
  lease_.control_epoch += 1;
  lease_.lease_id = make_lease_id(request, lease_.control_epoch);
  owner_source_id_ = request.source_id;
  owner_session_id_ = request.session_id;
  revoke_reason_.clear();

  return lease_;
}

bool LeaseManager::renew(const CommandMetadata& metadata) {
  if (!lease_.held || !metadata.lease_id) {
    return false;
  }

  return *metadata.lease_id == lease_.lease_id &&
         metadata.link_id == lease_.owner &&
         metadata.source_id == owner_source_id_ &&
         metadata.session_id == owner_session_id_;
}

void LeaseManager::revoke(std::string reason) noexcept {
  lease_.held = false;
  lease_.lease_id.clear();
  owner_source_id_.clear();
  owner_session_id_.clear();
  revoke_reason_ = std::move(reason);
}

LeaseView LeaseManager::current() const {
  return lease_;
}

}  // namespace snappy::control