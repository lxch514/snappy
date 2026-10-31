/**
 * @file control_channel.cpp
 * @brief gateway 到 control 的有 deadline IPC 请求实现
 *
 * @details
 *
 * - 每次请求使用短生命周期 SEQPACKET 连接。
 * - 命令和周期快照具有独立互斥量。
 * - connect、send 和 recv 共用一个绝对 deadline。
 * - 服务端 send 后 close 产生 POLLIN|POLLHUP 时优先读取数据。
 */

#include "snappy/gateway/control_channel.hpp"
#include "snappy/gateway/diagnostics/gateway_log.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <thread>

#include <nlohmann/json.hpp>

#if defined(__linux__)
#include <cerrno>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace snappy::gateway {
namespace {

constexpr auto kMinimumTimeout =
    std::chrono::milliseconds(10);

constexpr auto kMaximumCommandTimeout =
    std::chrono::milliseconds(500);

constexpr auto kMaximumSnapshotTimeout =
    std::chrono::milliseconds(200);

const char* link_name(
    control::LinkId link_id) noexcept {
  switch (link_id) {
    case control::LinkId::kG20Udp:
      return "g20_udp";
    case control::LinkId::kEc20Mqtt:
      return "ec20_mqtt";
  }

  return "unknown";
}

std::uint64_t wall_time_ms() {
  const auto now =
      std::chrono::system_clock::now()
          .time_since_epoch();

  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          now)
          .count());
}

std::chrono::milliseconds bounded_timeout(
    std::chrono::milliseconds configured,
    std::chrono::milliseconds maximum) {
  if (configured < kMinimumTimeout) {
    return kMinimumTimeout;
  }

  if (configured > maximum) {
    return maximum;
  }

  return configured;
}

#if defined(__linux__)

using Deadline =
    std::chrono::steady_clock::time_point;

int remaining_ms(Deadline deadline) {
  const auto now =
      std::chrono::steady_clock::now();

  if (now >= deadline) {
    return 0;
  }

  const auto remaining =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          deadline - now);

  return std::max(
      1,
      static_cast<int>(
          remaining.count()));
}

bool wait_fd(
    int fd,
    short events,
    Deadline deadline) {
  while (std::chrono::steady_clock::now() <
         deadline) {
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = events;

    const int ready =
        ::poll(
            &descriptor,
            1,
            remaining_ms(deadline));

    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }

      return false;
    }

    if (ready == 0) {
      return false;
    }

    /*
     * 服务端 send 后 close 时可能同时出现
     * POLLIN 和 POLLHUP，必须先处理目标事件。
     */
    if ((descriptor.revents & events) != 0) {
      return true;
    }

    if ((descriptor.revents &
         (POLLERR | POLLNVAL)) != 0) {
      return false;
    }

    if ((descriptor.revents & POLLHUP) != 0) {
      return false;
    }
  }

  return false;
}

bool connect_with_deadline(
    int fd,
    const sockaddr_un& address,
    Deadline deadline) {
  const int result =
      ::connect(
          fd,
          reinterpret_cast<
              const sockaddr*>(
              &address),
          sizeof(address));

  if (result == 0) {
    return true;
  }

  if (errno != EINPROGRESS &&
      errno != EALREADY &&
      errno != EAGAIN &&
      errno != EWOULDBLOCK) {
    return false;
  }

  if (!wait_fd(
          fd,
          POLLOUT,
          deadline)) {
    return false;
  }

  int socket_error = 0;
  socklen_t error_size =
      sizeof(socket_error);

  if (::getsockopt(
          fd,
          SOL_SOCKET,
          SO_ERROR,
          &socket_error,
          &error_size) != 0) {
    return false;
  }

  return socket_error == 0;
}

bool send_with_deadline(
    int fd,
    const std::string& payload,
    Deadline deadline) {
  while (std::chrono::steady_clock::now() <
         deadline) {
    const ssize_t sent =
        ::send(
            fd,
            payload.data(),
            payload.size(),
            MSG_NOSIGNAL | MSG_DONTWAIT);

    if (sent ==
        static_cast<ssize_t>(
            payload.size())) {
      return true;
    }

    if (sent >= 0) {
      return false;
    }

    if (errno == EINTR) {
      continue;
    }

    if (errno == EAGAIN ||
        errno == EWOULDBLOCK) {
      if (!wait_fd(
              fd,
              POLLOUT,
              deadline)) {
        return false;
      }

      continue;
    }

    return false;
  }

  return false;
}

std::optional<std::string>
receive_with_deadline(
    int fd,
    std::size_t max_payload_bytes,
    Deadline deadline) {
  if (!wait_fd(
          fd,
          POLLIN,
          deadline)) {
    return std::nullopt;
  }

  std::string output;
  output.resize(
      max_payload_bytes + 1);

  while (std::chrono::steady_clock::now() <
         deadline) {
    const ssize_t received =
        ::recv(
            fd,
            output.data(),
            output.size(),
            MSG_DONTWAIT);

    if (received > 0) {
      const auto size =
          static_cast<std::size_t>(
              received);

      if (size > max_payload_bytes) {
        return std::nullopt;
      }

      output.resize(size);
      return output;
    }

    if (received == 0) {
      return std::nullopt;
    }

    if (errno == EINTR) {
      continue;
    }

    if (errno == EAGAIN ||
        errno == EWOULDBLOCK) {
      if (!wait_fd(
              fd,
              POLLIN,
              deadline)) {
        return std::nullopt;
      }

      continue;
    }

    return std::nullopt;
  }

  return std::nullopt;
}

#endif

}  // namespace

SeqpacketControlChannel::SeqpacketControlChannel(
    ControlChannelConfig config)
    : config_(std::move(config)) {}

/* ============================================================
 * 通用请求
 * ============================================================ */

std::optional<std::string>
SeqpacketControlChannel::perform_request(
    const std::string& command_json,
    std::chrono::milliseconds timeout) {
#if !defined(__linux__)
  (void)command_json;
  (void)timeout;
  return std::nullopt;
#else
  if (command_json.empty() ||
      command_json.size() >
          config_.max_payload_bytes) {
    return std::nullopt;
  }

  const auto deadline =
      std::chrono::steady_clock::now() +
      timeout;

  const int fd =
      ::socket(
          AF_UNIX,
          SOCK_SEQPACKET |
              SOCK_CLOEXEC |
              SOCK_NONBLOCK,
          0);

  if (fd < 0) {
    return std::nullopt;
  }

  sockaddr_un address{};
  address.sun_family = AF_UNIX;

  if (config_.socket_path.size() >=
      sizeof(address.sun_path)) {
    ::close(fd);
    return std::nullopt;
  }

  std::strncpy(
      address.sun_path,
      config_.socket_path.c_str(),
      sizeof(address.sun_path) - 1);

  if (!connect_with_deadline(
          fd,
          address,
          deadline)) {
    ::close(fd);
    return std::nullopt;
  }

  if (!send_with_deadline(
          fd,
          command_json,
          deadline)) {
    ::close(fd);
    return std::nullopt;
  }

  auto response =
      receive_with_deadline(
          fd,
          config_.max_payload_bytes,
          deadline);

  ::close(fd);
  return response;
#endif
}

/* ============================================================
 * 遥控命令路径
 * ============================================================ */

std::optional<std::string>
SeqpacketControlChannel::request(
    const std::string& command_json) {
  std::unique_lock<std::mutex> lock(
      command_mutex_,
      std::try_to_lock);

  if (!lock.owns_lock()) {
    return std::nullopt;
  }

  return perform_request(
      command_json,
      bounded_timeout(
          config_.timeout,
          kMaximumCommandTimeout));
}

/* ============================================================
 * 周期快照路径
 * ============================================================ */

std::optional<std::string>
SeqpacketControlChannel::request_snapshot(
    const std::string& command_json) {
  std::unique_lock<std::mutex> lock(
      snapshot_mutex_,
      std::try_to_lock);

  if (!lock.owns_lock()) {
    return std::nullopt;
  }

  return perform_request(
      command_json,
      bounded_timeout(
          config_.snapshot_timeout,
          kMaximumSnapshotTimeout));
}

/* ============================================================
 * 链路掉线通知
 * ============================================================ */

void SeqpacketControlChannel::notify_link_down(
    control::LinkId link_id) noexcept {
  try {
    nlohmann::json command;

    const auto timestamp =
        wall_time_ms();

    command["schema_version"] = 1;
    command["kind"] = "command";
    command["robot_id"] =
        config_.robot_id;
    command["link_id"] =
        link_name(link_id);
    command["source_id"] =
        "snappy-gateway";
    command["session_id"] =
        "gateway-link-monitor";
    command["message_id"] =
        std::string("link-down-") +
        link_name(link_id) +
        "-" +
        std::to_string(timestamp);
    command["seq"] = 0;
    command["issued_at_ms"] =
        timestamp;
    command["ttl_ms"] =
        static_cast<std::uint32_t>(
            std::max<std::int64_t>(
                10,
                config_
                    .link_down_timeout
                    .count()));
    command["control_epoch"] = 0;
    command["action"] =
        "control.link_down";
    command["params"] = {
        {"link_id",
         link_name(link_id)},
    };

    /*
     * link_down 不能静默丢弃。
     *
     * 原实现用 try_to_lock：只要此刻有快照请求占用命令锁就直接
     * return，而调用方已经把 was_g20_online 置为 false，不会再补发。
     *
     * 这里改为最多等待 200ms；仍拿不到锁时记录 WARN。
     */
    constexpr auto kLinkDownLockWait =
        std::chrono::milliseconds(200);

    std::unique_lock<std::mutex> lock(
        command_mutex_,
        std::defer_lock);

    const auto lock_deadline =
        std::chrono::steady_clock::now() +
        kLinkDownLockWait;

    while (!lock.try_lock()) {
      if (std::chrono::steady_clock::now() >=
          lock_deadline) {
        diagnostics::warn(
            "control.link",
            "link_down notification dropped: "
            "control command channel busy");

        return;
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(2));
    }

    static_cast<void>(
        perform_request(
            command.dump(),
            bounded_timeout(
                config_.link_down_timeout,
                kMaximumSnapshotTimeout)));
  } catch (...) {
  }
}

}  // namespace snappy::gateway