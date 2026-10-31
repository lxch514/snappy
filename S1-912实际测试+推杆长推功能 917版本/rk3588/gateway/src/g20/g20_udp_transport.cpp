/**
 * @file g20_udp_transport.cpp
 * @brief G20 UDP 收发传输实现
 *
 * @details
 * - 接收 Android 遥控端下发的命令和舵量数据。
 * - 向最近一次通信的 Android 地址发送 ACK、状态和遥测。
 * - 增大 UDP 接收缓冲区，降低短时间突发数据造成的丢包。
 * - 不改变现有端口、消息格式和路由逻辑。
 */

#include "snappy/gateway/g20/g20_udp_transport.hpp"

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "snappy/gateway/diagnostics/gateway_log.hpp"

#if defined(__linux__)
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace snappy::gateway::g20 {
namespace {

constexpr int kRequestedReceiveBufferBytes =
    4 * 1024 * 1024;

constexpr int kRequestedSendBufferBytes =
    1024 * 1024;

constexpr long kReceiveTimeoutMicroseconds =
    5 * 1000;

std::vector<std::uint8_t> bytes_from_string(
    const std::string& value) {
  std::vector<std::uint8_t> bytes;
  bytes.reserve(value.size());

  for (const char character : value) {
    bytes.push_back(
        static_cast<std::uint8_t>(
            character));
  }

  return bytes;
}

std::string endpoint(
    const std::string& address,
    std::uint16_t port) {
  return address + ":" +
         std::to_string(port);
}

#if defined(__linux__)

/*
 * 设置整数类型的 socket 参数。
 * 设置失败只记录告警，不改变原有启动流程。
 */
bool set_socket_integer_option(
    int socket_fd,
    int option,
    int value,
    const std::string& option_name) {
  if (::setsockopt(
          socket_fd,
          SOL_SOCKET,
          option,
          &value,
          sizeof(value)) == 0) {
    return true;
  }

  diagnostics::warn(
      "g20.udp",
      option_name +
          " configuration failed: " +
          std::strerror(errno));

  return false;
}

/*
 * 读取 Linux 实际分配的 socket 缓冲区大小。
 * Linux 可能会对申请值进行调整，因此启动日志记录实际值。
 */
int socket_integer_option(
    int socket_fd,
    int option) {
  int value = 0;
  socklen_t value_size =
      sizeof(value);

  if (::getsockopt(
          socket_fd,
          SOL_SOCKET,
          option,
          &value,
          &value_size) != 0) {
    return 0;
  }

  return value;
}

#endif

}  // namespace

G20UdpTransport::G20UdpTransport(
    G20UdpRuntimeConfig config,
    PeerTracker& peer)
    : config_(std::move(config)),
      peer_(peer) {}

G20UdpTransport::~G20UdpTransport() {
  stop();
}

control::LinkId
G20UdpTransport::link_id() const noexcept {
  return control::LinkId::kG20Udp;
}

std::size_t
G20UdpTransport::payload_limit() const noexcept {
  return kUdpPayloadLimit;
}

bool G20UdpTransport::healthy() const noexcept {
  return started_ && fd_ >= 0;
}

/* ============================================================
 * UDP socket 启动
 * ============================================================ */

bool G20UdpTransport::start() {
#if defined(__linux__)
  if (started_) {
    return true;
  }

  fd_ = ::socket(
      AF_INET,
      SOCK_DGRAM | SOCK_CLOEXEC,
      0);

  if (fd_ < 0) {
    diagnostics::error(
        "g20.udp",
        std::string("socket failed: ") +
            std::strerror(errno));

    return false;
  }

  int reuse = 1;

  static_cast<void>(
      set_socket_integer_option(
          fd_,
          SO_REUSEADDR,
          reuse,
          "SO_REUSEADDR"));

  int broadcast = 1;

  static_cast<void>(
      set_socket_integer_option(
          fd_,
          SO_BROADCAST,
          broadcast,
          "SO_BROADCAST"));

  /*
   * Android 会周期发送普通舵量，并在按键变化时发送可靠事件。
   * 增大内核缓冲区可以容纳短时间突发数据，避免 control IPC
   * 短暂阻塞时接收队列过小。
   */
  static_cast<void>(
      set_socket_integer_option(
          fd_,
          SO_RCVBUF,
          kRequestedReceiveBufferBytes,
          "SO_RCVBUF"));

  static_cast<void>(
      set_socket_integer_option(
          fd_,
          SO_SNDBUF,
          kRequestedSendBufferBytes,
          "SO_SNDBUF"));

  timeval timeout {};
  timeout.tv_sec = 0;
  timeout.tv_usec =
      kReceiveTimeoutMicroseconds;

  if (::setsockopt(
          fd_,
          SOL_SOCKET,
          SO_RCVTIMEO,
          &timeout,
          sizeof(timeout)) != 0) {
    diagnostics::warn(
        "g20.udp",
        std::string(
            "SO_RCVTIMEO configuration failed: ") +
            std::strerror(errno));
  }

  sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port =
      htons(config_.command_port);

  if (config_.listen_address.empty() ||
      config_.listen_address == "0.0.0.0") {
    address.sin_addr.s_addr =
        htonl(INADDR_ANY);
  } else if (
      ::inet_pton(
          AF_INET,
          config_.listen_address.c_str(),
          &address.sin_addr) != 1) {
    diagnostics::error(
        "g20.udp",
        "invalid listen address: " +
            config_.listen_address);

    ::close(fd_);
    fd_ = -1;
    return false;
  }

  if (::bind(
          fd_,
          reinterpret_cast<sockaddr*>(
              &address),
          sizeof(address)) != 0) {
    diagnostics::error(
        "g20.udp",
        std::string("bind failed: ") +
            std::strerror(errno));

    ::close(fd_);
    fd_ = -1;
    return false;
  }

  const int actual_receive_buffer =
      socket_integer_option(
          fd_,
          SO_RCVBUF);

  const int actual_send_buffer =
      socket_integer_option(
          fd_,
          SO_SNDBUF);

  started_ = true;

  diagnostics::info(
      "g20.udp",
      "listening command udp " +
          endpoint(
              config_.listen_address.empty()
                  ? std::string("0.0.0.0")
                  : config_.listen_address,
              config_.command_port) +
          " status_port=" +
          std::to_string(
              config_.status_port) +
          " telemetry_port=" +
          std::to_string(
              config_.telemetry_port) +
          " receive_buffer=" +
          std::to_string(
              actual_receive_buffer) +
          " send_buffer=" +
          std::to_string(
              actual_send_buffer) +
          " receive_timeout_us=" +
          std::to_string(
              kReceiveTimeoutMicroseconds));

  return true;
#else
  diagnostics::error(
      "g20.udp",
      "udp transport requires Linux");

  return false;
#endif
}

/* ============================================================
 * UDP socket 停止
 * ============================================================ */

void G20UdpTransport::stop() noexcept {
#if defined(__linux__)
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif

  if (started_) {
    diagnostics::info(
        "g20.udp",
        "udp transport stopped");
  }

  started_ = false;
}

/* ============================================================
 * Android 数据接收
 * ============================================================ */

std::optional<InboundMessage>
G20UdpTransport::receive() {
#if defined(__linux__)
  if (!healthy()) {
    return std::nullopt;
  }

  char buffer[kUdpPayloadLimit + 1] {};

  sockaddr_in source {};
  socklen_t source_size =
      sizeof(source);

  const ssize_t received =
      ::recvfrom(
          fd_,
          buffer,
          kUdpPayloadLimit,
          0,
          reinterpret_cast<sockaddr*>(
              &source),
          &source_size);

  if (received < 0) {
    if (
        errno == EAGAIN ||
        errno == EWOULDBLOCK ||
        errno == EINTR) {
      return std::nullopt;
    }

    diagnostics::warn(
        "g20.udp",
        std::string("recvfrom failed: ") +
            std::strerror(errno));

    started_ = false;
    return std::nullopt;
  }

  char ip[INET_ADDRSTRLEN] {};
  std::string source_ip = "unknown";

  const std::uint16_t source_port =
      ntohs(source.sin_port);

  if (::inet_ntop(
          AF_INET,
          &source.sin_addr,
          ip,
          sizeof(ip)) != nullptr) {
    source_ip = ip;

    /*
     * ACK 和状态消息发送给最近一次发送数据的 Android。
     */
    peer_.observe(
        ip,
        source_port);
  }

  const std::string payload(
      buffer,
      static_cast<std::size_t>(
          received));

  diagnostics::trace_packet(
      "udp",
      "rx",
      endpoint(
          source_ip,
          source_port),
      bytes_from_string(payload),
      "topic=snappy/v1/g20/down/command");

  return InboundMessage{
      "snappy/v1/g20/down/command",
      payload,
  };
#else
  return std::nullopt;
#endif
}

/* ============================================================
 * 返回地址和目标端口
 * ============================================================ */

std::optional<std::string>
G20UdpTransport::target_address() const {
  if (
      config_.remote_address &&
      !config_.remote_address->empty()) {
    return *config_.remote_address;
  }

  const auto peer =
      peer_.latest();

  if (!peer ||
      peer->address.empty()) {
    return std::nullopt;
  }

  return peer->address;
}

std::uint16_t
G20UdpTransport::publish_port(
    const PublishedMessage& message) const {
  if (
      message.topic.find("telemetry") !=
      std::string::npos) {
    return config_.telemetry_port;
  }

  return config_.status_port;
}

/* ============================================================
 * ACK、状态和遥测发送
 * ============================================================ */

bool G20UdpTransport::publish(
    const PublishedMessage& message) {
#if defined(__linux__)
  if (!healthy()) {
    return false;
  }

  const auto target =
      target_address();

  if (!target) {
    diagnostics::debug(
        "g20.udp",
        "publish skipped, target unknown");

    return false;
  }

  const std::uint16_t port =
      publish_port(message);

  sockaddr_in address {};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);

  if (::inet_pton(
          AF_INET,
          target->c_str(),
          &address.sin_addr) != 1) {
    diagnostics::warn(
        "g20.udp",
        "invalid target address: " +
            *target);

    return false;
  }

  diagnostics::trace_packet(
      "udp",
      "tx",
      endpoint(
          *target,
          port),
      bytes_from_string(
          message.payload),
      "topic=" + message.topic);

  const ssize_t sent =
      ::sendto(
          fd_,
          message.payload.data(),
          message.payload.size(),
          0,
          reinterpret_cast<sockaddr*>(
              &address),
          sizeof(address));

  const bool succeeded =
      sent >= 0 &&
      static_cast<std::size_t>(sent) ==
          message.payload.size();

  if (!succeeded) {
    diagnostics::warn(
        "g20.udp",
        std::string("sendto failed: ") +
            std::strerror(errno));
  }

  return succeeded;
#else
  (void)message;
  return false;
#endif
}

}  // namespace snappy::gateway::g20