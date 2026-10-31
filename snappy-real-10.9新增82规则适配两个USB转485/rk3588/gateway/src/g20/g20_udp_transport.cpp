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
  return started_.load() && fd_.load() >= 0;
}

bool G20UdpTransport::unreachable() const noexcept {
  return consecutive_unreachable_.load() >=
         kUnreachableConfirmCount;
}

long long
G20UdpTransport::unreachable_send_count() const noexcept {
  return unreachable_send_count_.load();
}

long long
G20UdpTransport::oversize_publish_count() const noexcept {
  return oversize_publish_count_.load();
}

/* ============================================================
 * socket 自愈重启
 *
 * 原实现遇到非 EAGAIN/EINTR 的 recvfrom 错误就把 started_ 永久
 * 置 false：receive()/publish() 之后全部短路，接收线程 100% 空转，
 * 一次瞬时错误（ENOBUFS/ENOMEM/ECONNREFUSED 等）会让整条链路
 * 永久死亡，表现为"遥控器显示设备掉线 + 8s 后整机安全停止"。
 *
 * 这里改为有限次退避重启，让传输层能够自愈。
 * ============================================================ */

bool G20UdpTransport::restart_socket() {
  last_restart_attempt_ =
      std::chrono::steady_clock::now();

  stop();

  const bool ok = start();

  if (!ok) {
    diagnostics::error(
        "g20.udp",
        "socket restart failed; will retry");
  }

  return ok;
}

/* ============================================================
 * UDP socket 启动
 * ============================================================ */

bool G20UdpTransport::start() {
#if defined(__linux__)
  if (started_.load()) {
    return true;
  }

  /*
   * 全部配置步骤都先作用在局部 fd 上，成功之后才一次性发布到
   * fd_/started_。这样接收线程不会看到一个半初始化的 socket。
   */
  const int fd = ::socket(
      AF_INET,
      SOCK_DGRAM | SOCK_CLOEXEC,
      0);

  if (fd < 0) {
    diagnostics::error(
        "g20.udp",
        std::string("socket failed: ") +
            std::strerror(errno));

    return false;
  }

  int reuse = 1;

  static_cast<void>(
      set_socket_integer_option(
          fd,
          SO_REUSEADDR,
          reuse,
          "SO_REUSEADDR"));

  int broadcast = 1;

  static_cast<void>(
      set_socket_integer_option(
          fd,
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
          fd,
          SO_RCVBUF,
          kRequestedReceiveBufferBytes,
          "SO_RCVBUF"));

  static_cast<void>(
      set_socket_integer_option(
          fd,
          SO_SNDBUF,
          kRequestedSendBufferBytes,
          "SO_SNDBUF"));

  timeval timeout {};
  timeout.tv_sec = 0;
  timeout.tv_usec =
      kReceiveTimeoutMicroseconds;

  if (::setsockopt(
          fd,
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

    ::close(fd);
    return false;
  }

  if (::bind(
          fd,
          reinterpret_cast<sockaddr*>(
              &address),
          sizeof(address)) != 0) {
    diagnostics::error(
        "g20.udp",
        std::string("bind failed: ") +
            std::strerror(errno));

    ::close(fd);
    return false;
  }

  const int actual_receive_buffer =
      socket_integer_option(
          fd,
          SO_RCVBUF);

  const int actual_send_buffer =
      socket_integer_option(
          fd,
          SO_SNDBUF);

  fd_.store(fd);
  started_.store(true);

  /*
   * socket 是新的，之前累计的"不可达"结论作废。
   */
  consecutive_unreachable_.store(0);

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
  const int fd = fd_.exchange(-1);

  if (fd >= 0) {
    ::close(fd);
  }
#endif

  if (started_.exchange(false)) {
    diagnostics::info(
        "g20.udp",
        "udp transport stopped");
  }
}

/* ============================================================
 * Android 数据接收
 * ============================================================ */

std::optional<InboundMessage>
G20UdpTransport::receive() {
#if defined(__linux__)
  if (!healthy()) {
    /*
     * socket 当前不可用（启动失败或刚刚重启失败）。
     * 按退避间隔继续尝试自愈，绝不永久放弃。
     */
    const auto now =
        std::chrono::steady_clock::now();

    if (now - last_restart_attempt_ >=
        kSocketRestartBackoff) {
      static_cast<void>(restart_socket());
    }

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

    /*
     * 致命接收错误：不能把传输层永久判死。
     *
     * 原实现只把 started_ 置 false 就返回，之后 receive()/publish()
     * 全部短路、接收线程 100% 空转，一次瞬时错误就会让整条链路
     * 永久失效（遥控器显示设备掉线，8s 后整机安全停止）。
     * 这里立刻重开 socket；失败则由上层按退避继续重试。
     */
    diagnostics::error(
        "g20.udp",
        std::string("recvfrom failed: ") +
            std::strerror(errno) +
            "; reopening receive socket");

    static_cast<void>(restart_socket());

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

  /*
   * UDP 契约检查（只告警，不改行为）。
   *
   * docs/设备编码规范.md:348 与 docs/snappy v1 精简架构设计.md:108 都要求
   * "UDP 数据报上限 1200 B，禁止分片"，kUdpPayloadLimit 也是 1200。
   * 但实际发出的状态/遥测报文有 8095 B / 3950 B（6/3 个 IP 分片）。
   *
   * 这里故意只做可见化，不直接拒绝：改小报文属于 wire 契约变更，
   * 必须两端一起改（Android 的 PacketCodec 也要同步），
   * 单方面截断会让操作界面静默丢字段，比"分片"更危险。
   */
  if (message.payload.size() > kUdpPayloadLimit) {
    const long long oversized =
        oversize_publish_count_.fetch_add(1) + 1;

    if (oversized == 1 || (oversized % 1000) == 0) {
      diagnostics::warn(
          "g20.udp",
          "payload exceeds UDP contract limit: size=" +
              std::to_string(
                  message.payload.size()) +
              " limit=" +
              std::to_string(kUdpPayloadLimit) +
              " topic=" + message.topic +
              " count=" +
              std::to_string(oversized));
    }
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

  const auto fd = fd_.load();

  if (fd < 0) {
    return false;
  }

  const ssize_t sent =
      ::sendto(
          fd,
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

  if (succeeded) {
    /*
     * 发送恢复：结算上一轮不可达计数并留痕，
     * 便于现场把"掉线开始/结束"和链路事件对齐。
     */
    const int previous =
        consecutive_unreachable_.exchange(0);

    if (previous >= kUnreachableConfirmCount) {
      diagnostics::warn(
          "g20.udp",
          "sendto recovered after " +
              std::to_string(previous) +
              " consecutive unreachable sends");
    }

    return true;
  }

  const int send_error = errno;

  const bool route_gone =
      send_error == ENETUNREACH ||
      send_error == ENETDOWN ||
      send_error == EHOSTUNREACH ||
      send_error == ENETRESET;

  if (!route_gone) {
    consecutive_unreachable_.store(0);

    diagnostics::warn(
        "g20.udp",
        std::string("sendto failed: ") +
            std::strerror(send_error));

    return false;
  }

  /*
   * 内核明确表示"到对端的路由不存在"。
   *
   * 这是确定性故障，不是丢包抖动，因此单独计数并上报给主循环，
   * 由主循环立即触发 link_down，而不用再等静默确认窗口。
   *
   * 日志按首个 / 达到确认阈值 / 每 200 次输出一次：
   * 掉载波期间原本会产生 26 行/秒的重复告警（当天 116061 行），
   * 既淹没其它证据也浪费 CPU。
   */
  const int consecutive =
      consecutive_unreachable_.fetch_add(1) + 1;

  unreachable_send_count_.fetch_add(1);

  if (consecutive == 1 ||
      consecutive == kUnreachableConfirmCount ||
      (consecutive % 200) == 0) {
    diagnostics::warn(
        "g20.udp",
        std::string("sendto failed: ") +
            std::strerror(send_error) +
            " (peer route is gone, consecutive=" +
            std::to_string(consecutive) +
            ", total=" +
            std::to_string(
                unreachable_send_count_.load()) +
            ", topic=" + message.topic + ")");
  }

  return false;
#else
  (void)message;
  return false;
#endif
}

}  // namespace snappy::gateway::g20