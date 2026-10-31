#include "snappy/drivers/bus/can_socket_bus.hpp"

/**
 * @file can_socket_bus.cpp
 * @brief RK3588 SocketCAN 配置、收发与独立故障恢复
 *
 * @details
 * - 按 CAN HAL 的顺序配置接口：down、bitrate、restart、
 *   txqueuelen、up。
 * - allow_start_failure 只允许整机继续启动，不把失败总线标记为 Ready。
 * - CAN 故障只由 CAN 模块恢复，不影响 RS485 端口和设备。
 */

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "snappy/drivers/diagnostics/driver_log.hpp"

namespace snappy::drivers::bus {

bool can_frame_matches_filter(const CanFrame& frame,
    const CanReceiveFilter& filter) noexcept {
  if (frame.extended != filter.extended) {
    return false;
  }

  return (frame.id & filter.mask) == (filter.id & filter.mask);
}

SocketCanBus::SocketCanBus(CanBusConfig config)
    : config_(std::move(config)) {}

SocketCanBus::~SocketCanBus() {
  stop();
}

const CanBusConfig& SocketCanBus::config() const noexcept {
  return config_;
}

DriverHealth SocketCanBus::health() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return health_;
}

void SocketCanBus::safe_stop() noexcept {
  /*
   * 总线层不知道具体设备的停止协议。
   * 推进风扇的零油门和 Ready=0x55 由设备驱动发送。
   */
}

}  // namespace snappy::drivers::bus

#if defined(__linux__)

#include <cerrno>
#include <fstream>
#include <optional>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace snappy::drivers::bus {
namespace {

/*
 * 总线瞬时拥塞只有几十毫秒，连续 2 次失败就重建接口会把运行中的风机打停。
 */
constexpr int kRecoverAfterConsecutiveFailures = 40;

constexpr auto kRecoverDelay = std::chrono::milliseconds(50);

std::optional<std::uint64_t> read_u64_file(
    const std::string& path) {
  std::ifstream file(path);

  if (!file.is_open()) {
    return std::nullopt;
  }

  unsigned long long value = 0;

  file >> value;

  if (!file) {
    return std::nullopt;
  }

  return static_cast<std::uint64_t>(value);
}

std::string component_name(const CanBusConfig& config) {
  return config.bus_id + ":" + config.interface_name;
}

bool is_safe_interface_name(const std::string& value) {
  if (value.empty() || value.size() >= IFNAMSIZ) {
    return false;
  }

  for (const unsigned char ch : value) {
    if (!std::isalnum(ch) &&
        ch != '_' &&
        ch != '-' &&
        ch != '.' &&
        ch != ':') {
      return false;
    }
  }

  return true;
}

int remaining_ms(std::chrono::steady_clock::time_point deadline) {
  const auto now = std::chrono::steady_clock::now();

  if (now >= deadline) {
    return 0;
  }

  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);

  return std::max(1, static_cast<int>(left.count()));
}

bool poll_has_error(short revents) {
  return (revents & (POLLERR | POLLHUP | POLLNVAL)) != 0;
}

bool matches_any_filter(
    const CanFrame& frame,
    const std::vector<CanReceiveFilter>& filters) {
  if (filters.empty()) {
    return true;
  }

  return std::any_of(
      filters.begin(),
      filters.end(),
      [&](const CanReceiveFilter& filter) {
        return can_frame_matches_filter(frame,filter);
      });
}

bool same_filters(
    const std::vector<CanReceiveFilter>& left,
    const std::vector<CanReceiveFilter>& right) {
  if (left.size() != right.size()) {
    return false;
  }

  for (std::size_t index = 0;
       index < left.size();
       ++index) {
    if (left[index].id != right[index].id ||
        left[index].mask != right[index].mask ||
        left[index].extended != right[index].extended) {
      return false;
    }
  }

  return true;
}

std::string can_id_text(const CanFrame& frame) {
  std::ostringstream out;
  out << "0x" << std::hex << std::uppercase << frame.id;
  return out.str();
}

std::string can_detail(const CanFrame& frame) {
  std::ostringstream out;

  out << "id=" << can_id_text(frame)
      << " extended="
      << (frame.extended ? "true" : "false")
      << " dlc=" << frame.payload.size();

  return out.str();
}

Result validate_frame(const CanFrame& frame,const CanBusConfig& config) {
  if (frame.payload.size() > kClassicCanMaxPayloadBytes) {
    return Result::failure(
        ErrorCode::kCanFrameInvalid,
        "classic CAN payload exceeds 8 bytes",
        "send",
        component_name(config));
  }

  if (frame.extended) {
    if (frame.id > CAN_EFF_MASK) {
      return Result::failure(
          ErrorCode::kCanFrameInvalid,
          "extended CAN id out of range",
          "send",
          component_name(config));
    }
  } else if (frame.id > CAN_SFF_MASK) {
    return Result::failure(
        ErrorCode::kCanFrameInvalid,
        "standard CAN id out of range",
        "send",
        component_name(config));
  }

  return Result::success();
}

CanFrame from_linux_frame(
    const can_frame& raw) {
  CanFrame frame;

  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;

  frame.id =
      raw.can_id &
      (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);

  frame.payload.assign(raw.data,raw.data + raw.can_dlc);

  return frame;
}

bool should_recover(const Result& result) {
  if (result.ok ||
      !result.retryable) {
    return false;
  }

  return result.code ==
             ErrorCode::kIoError ||
         result.code ==
             ErrorCode::kConfigRequired ||
         result.code ==
             ErrorCode::kCanTxTimeout;
}

}  // namespace

Result SocketCanBus::configure_interface_locked() {
  if (!config_.auto_configure_interface) {
    return Result::success("SocketCAN auto configuration disabled");
  }

  if (!is_safe_interface_name(config_.interface_name)) {
    health_ = DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kInvalidArgument,
        "unsafe or invalid CAN interface name",
        "configure",
        component_name(config_));
  }

  if (config_.bitrate == 0 ||
      config_.tx_queue_length == 0 ||
      !std::isfinite(config_.sample_point) ||
      config_.sample_point <= 0.0 ||
      config_.sample_point >= 1.0 ||
      config_.sjw == 0 ||
      config_.sjw > 128 ||
      config_.restart_ms.count() < 0) {
    health_ = DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kInvalidArgument,
        "invalid SocketCAN interface configuration",
        "configure",
        component_name(config_));
  }

  std::ostringstream command;

  command
      << "ip link set "
      << config_.interface_name
      << " down >/dev/null 2>&1 && "
      << "ip link set "
      << config_.interface_name
      << " type can bitrate "
      << config_.bitrate
      << " sample-point "
      << std::fixed
      << std::setprecision(3)
      << config_.sample_point
      << " sjw "
      << config_.sjw;

  if (config_.restart_on_bus_off) {
    command
        << " restart-ms "
        << config_.restart_ms.count();
  }

  command
      << " >/dev/null 2>&1 && "
      << "ip link set "
      << config_.interface_name
      << " txqueuelen "
      << config_.tx_queue_length
      << " >/dev/null 2>&1 && "
      << "ip link set "
      << config_.interface_name
      << " up >/dev/null 2>&1";

  const int rc =
      std::system(command.str().c_str());

  if (rc != 0) {
    health_ = DriverHealth::kFault;

    diagnostics::error(
        "can.socket",
        "failed to configure " +
            component_name(config_));

    return Result::failure(
        ErrorCode::kIoError,
        "ip link failed while configuring SocketCAN",
        "configure",
        component_name(config_),
        true);
  }

  diagnostics::info(
      "can.socket",
      "configured " +
          component_name(config_) +
          " bitrate=" +
          std::to_string(config_.bitrate) +
          " sjw=" +
          std::to_string(config_.sjw) +
          " txqueuelen=" +
          std::to_string(config_.tx_queue_length));

  return Result::success("SocketCAN interface configured");
}

Result SocketCanBus::open_socket_locked() {
  if (fd_ >= 0 && health_ == DriverHealth::kReady) {
    return Result::success("SocketCAN already open");
  }

  close_socket_locked();

  const int next_fd =
      ::socket(
          PF_CAN,
          SOCK_RAW | SOCK_CLOEXEC,
          CAN_RAW);

  if (next_fd < 0) {
    health_ = DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kIoError,
        std::string("SocketCAN socket failed: ") + std::strerror(errno),
        "open",
        component_name(config_),
        true);
  }

  const int receive_own = config_.receive_own_messages ? 1 : 0;

  static_cast<void>(
      ::setsockopt(
          next_fd,
          SOL_CAN_RAW,
          CAN_RAW_RECV_OWN_MSGS,
          &receive_own,
          sizeof(receive_own)));

  ifreq interface_request{};
  std::strncpy(
      interface_request.ifr_name,
      config_.interface_name.c_str(),
      IFNAMSIZ - 1);

  if (::ioctl(
          next_fd,
          SIOCGIFINDEX,
          &interface_request) < 0) {
    const std::string detail =
        std::string("SIOCGIFINDEX failed: ") +
        std::strerror(errno);

    ::close(next_fd);
    health_ = DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kIoError,
        detail,
        "open",
        component_name(config_),
        true);
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_request.ifr_ifindex;

  if (::bind(
          next_fd,
          reinterpret_cast<sockaddr*>(&address),
          sizeof(address)) < 0) {
    const std::string detail =
        std::string("SocketCAN bind failed: ") +
        std::strerror(errno);

    ::close(next_fd);
    health_ = DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kIoError,
        detail,
        "bind",
        component_name(config_),
        true);
  }

  const int flags =
      ::fcntl(next_fd, F_GETFL, 0);

  if (flags >= 0) {
    static_cast<void>(
        ::fcntl(
            next_fd,
            F_SETFL,
            flags | O_NONBLOCK));
  }

  fd_ = next_fd;
  consecutive_failures_ = 0;
  health_ = DriverHealth::kReady;

  diagnostics::info("can.socket", "SocketCAN ready " + component_name(config_));

  return Result::success("SocketCAN ready");
}

Result SocketCanBus::start() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (fd_ >= 0 && health_ == DriverHealth::kReady) {
    return Result::success("SocketCAN already started");
  }

  const auto configured = configure_interface_locked();

  if (!configured.ok) {
    close_socket_locked();
    health_ = DriverHealth::kFault;

    if (config_.allow_start_failure) {
      diagnostics::warn(
          "can.socket",
          "CAN unavailable; isolated from other "
          "devices: " + configured.detail);

      return Result::success("CAN unavailable but isolated");
    }

    return configured;
  }

  const auto opened = open_socket_locked();

  if (!opened.ok) {
    close_socket_locked();
    health_ = DriverHealth::kFault;

    if (config_.allow_start_failure) {
      diagnostics::warn(
          "can.socket",
          "CAN open failure isolated: " + opened.detail);

      return Result::success("CAN unavailable but isolated");
    }

    return opened;
  }

  return opened;
}

void SocketCanBus::close_socket_locked() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

SocketCanBus::TxQueueUsage
SocketCanBus::tx_queue_usage_locked() const {
  TxQueueUsage usage{};

  if (config_.interface_name.empty()) {
    return usage;
  }

  const auto base =
      std::string("/sys/class/net/") +
      config_.interface_name + "/";

  std::ifstream len_file(base + "tx_queue_len");

  if (!len_file.is_open()) {
    return usage;
  }

  unsigned long long limit = 0;

  len_file >> limit;

  if (!len_file) {
    return usage;
  }

  usage.limit = static_cast<std::uint32_t>(limit);

  const auto drops_before = read_u64_file(base + "statistics/tx_dropped");
  const auto packets_before = read_u64_file(base + "statistics/tx_packets");

  if (!drops_before || !packets_before) {
    return usage;
  }

  std::this_thread::sleep_for(
      std::chrono::milliseconds(2));

  const auto drops_after = read_u64_file(base + "statistics/tx_dropped");
  const auto packets_after = read_u64_file(base + "statistics/tx_packets");

  if (!drops_after || !packets_after) {
    return usage;
  }

  usage.valid = true;
  usage.dropped = *drops_after - *drops_before;
  usage.sent = *packets_after - *packets_before;

  /*
   * 队列使用量 = 限制 - 空闲槽位。内核不直接暴露空闲槽位，
   * 用"限制内是否发生了丢弃"近似判断：有丢弃即认为已满。
   */
  usage.used =
      usage.dropped > 0
          ? usage.limit
          : 0U;

  return usage;
}

Result SocketCanBus::reopen_socket_locked() {
  /*
   * 接口仍然 up、只是 socket 发送队列堵住时只换 fd，不拆接口。
   */
  close_socket_locked();
  installed_kernel_filters_.clear();
  health_ = DriverHealth::kUnconfigured;

  return open_socket_locked();
}

Result SocketCanBus::recover_locked(
    const std::string& reason,
    bool socket_only_first) {
  diagnostics::warn(
      "can.socket",
      "recovering " +
          component_name(config_) +
          " reason=" + reason +
          " consecutive_failures=" +
          std::to_string(consecutive_failures_));

  if (socket_only_first) {
    const auto reopened = reopen_socket_locked();

    if (reopened.ok) {
      diagnostics::info(
          "can.socket",
          "socket reopened " +
              component_name(config_));

      return reopened;
    }
  }

  close_socket_locked();
  health_ = DriverHealth::kFault;

  std::this_thread::sleep_for(kRecoverDelay);

  const auto configured = configure_interface_locked();

  if (!configured.ok) {
    close_socket_locked();
    health_ = DriverHealth::kFault;
    return configured;
  }

  const auto opened = open_socket_locked();

  if (!opened.ok) {
    close_socket_locked();
    health_ = DriverHealth::kFault;
    return opened;
  }

  diagnostics::info("can.socket", "recovered " + component_name(config_));

  return opened;
}

void SocketCanBus::apply_kernel_filter_locked(
    const std::vector<CanReceiveFilter>& filters) {
  if (same_filters(filters, installed_kernel_filters_)) {
    return;
  }

  if (filters.empty()) {
    /*
     * 空过滤集在内核里表示拒收全部报文，必须显式写成全收。
     */
    can_filter accept_all{};
    accept_all.can_id = 0;
    accept_all.can_mask = 0;

    static_cast<void>(
        ::setsockopt(
            fd_,
            SOL_CAN_RAW,
            CAN_RAW_FILTER,
            &accept_all,
            sizeof(accept_all)));

    installed_kernel_filters_.clear();
    return;
  }

  std::vector<can_filter> raw_filters;
  raw_filters.reserve(filters.size());

  for (const auto& filter : filters) {
    const std::uint32_t id_mask =
        filter.extended
            ? CAN_EFF_MASK
            : CAN_SFF_MASK;

    can_filter raw{};
    raw.can_id = filter.id & id_mask;
    raw.can_mask = filter.mask & id_mask;

    if (filter.extended) {
      raw.can_id |= CAN_EFF_FLAG;
      raw.can_mask |= CAN_EFF_FLAG;
    }

    raw_filters.push_back(raw);
  }

  const int rc =
      ::setsockopt(
          fd_,
          SOL_CAN_RAW,
          CAN_RAW_FILTER,
          raw_filters.data(),
          static_cast<socklen_t>(
              raw_filters.size() *
              sizeof(can_filter)));

  if (rc != 0) {
    diagnostics::warn(
        "can.socket",
        "kernel CAN filter not applied: " +
            std::string(std::strerror(errno)));

    return;
  }

  installed_kernel_filters_ = filters;
}

Result SocketCanBus::send_once_locked(
    const CanFrame& frame) {
  if (fd_ < 0 || health_ != DriverHealth::kReady) {
    return Result::failure(
        ErrorCode::kConfigRequired,
        "SocketCAN bus is not open",
        "send",
        component_name(config_),
        true);
  }

  const auto valid = validate_frame(frame, config_);

  if (!valid.ok) {
    return valid;
  }

  can_frame raw{};
  raw.can_id =
      frame.id |
      (frame.extended
           ? CAN_EFF_FLAG
           : 0U);

  raw.can_dlc = static_cast<__u8>(frame.payload.size());

  if (!frame.payload.empty()) {
    std::memcpy(
        raw.data,
        frame.payload.data(),
        frame.payload.size());
  }

  const auto deadline = std::chrono::steady_clock::now() + config_.default_tx_timeout;

  diagnostics::trace_packet(
      "can",
      "tx",
      component_name(config_) + "/" + can_id_text(frame),
      frame.payload,
      can_detail(frame));

  while (true) {
    const int wait_ms = remaining_ms(deadline);

    if (wait_ms <= 0) {
      return Result::failure(
          ErrorCode::kCanTxTimeout,
          "SocketCAN transmit timeout",
          "send",
          component_name(config_),
          true);
    }

    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = POLLOUT;

    const int poll_result =
        ::poll(
            &descriptor,
            1,
            wait_ms);

    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }

      return Result::failure(
          ErrorCode::kIoError,
          std::string("CAN transmit poll failed: ") + std::strerror(errno),
          "send",
          component_name(config_),
          true);
    }

    if (poll_result == 0) {
      /*
       * POLLOUT 超时不等于发送失败。
       * 总线被 ESC 遥测占满时控制器内核直到整帧发完才唤醒 TX 队列，
       * 20 ms 内常常等不到 POLLOUT，但 TX 队列本身仍是空的，可以写。
       * 这里只在队列确实满时才报错，否则直接尝试 write。
       */
      const auto queued = tx_queue_usage_locked();

      if (queued.valid && queued.used >= queued.limit) {
        return Result::failure(
            ErrorCode::kCanTxTimeout,
            "SocketCAN transmit queue full",
            "send",
            component_name(config_),
            true);
      }
    }

    if (poll_has_error(descriptor.revents) && !(descriptor.revents & POLLOUT)) {
      return Result::failure(
          ErrorCode::kIoError,
          "SocketCAN transmit fd error",
          "send",
          component_name(config_),
          true);
    }

    const ssize_t written =
        ::write(
            fd_,
            &raw,
            sizeof(raw));

    if (written == static_cast<ssize_t>(sizeof(raw))) {
      consecutive_failures_ = 0;
      return Result::success("CAN frame sent");
    }

    if (written < 0 &&
        (errno == EINTR ||
         errno == EAGAIN ||
         errno == EWOULDBLOCK ||
         errno == ENOBUFS)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    return Result::failure(
        ErrorCode::kIoError,
        std::string("SocketCAN write failed: ") +
            std::strerror(errno),
        "send",
        component_name(config_),
        true);
  }
}

Result SocketCanBus::send(const CanFrame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (fd_ < 0 || health_ != DriverHealth::kReady) {
    const auto recovered = recover_locked("lazy recovery before send", false);

    if (!recovered.ok) {
      return recovered;
    }
  }

  auto result = send_once_locked(frame);

  if (result.ok) {
    return result;
  }

  if (!should_recover(result)) {
    return result;
  }

  ++consecutive_failures_;

  if (consecutive_failures_ < kRecoverAfterConsecutiveFailures) {
    return result;
  }

  /*
   * 发送超时只是总线瞬时拥塞，先只换 socket，保住接口和整条链路。
   */
  const auto recovered = recover_locked(result.detail, result.code == ErrorCode::kCanTxTimeout);

  if (!recovered.ok) {
    return recovered;
  }

  /*
   * 推进风扇使用的 Select、Ready 和 RawCommand
   * 都是幂等状态命令，恢复后允许重发本帧。
   */
  return send_once_locked(frame);
}

CanReceiveResult SocketCanBus::receive_once_locked(
    const CanReceiveOptions& options) {
  if (fd_ < 0 || health_ != DriverHealth::kReady) {
    return {
        Result::failure(
            ErrorCode::kConfigRequired,
            "SocketCAN bus is not open",
            "receive",
            component_name(config_),
            true),
        {}};
  }

  apply_kernel_filter_locked(options.filters);

  const auto timeout =
      options.timeout.count() > 0
          ? options.timeout
          : config_.default_rx_timeout;

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (true) {
    const int wait_ms = remaining_ms(deadline);

    if (wait_ms <= 0) {
      return {
          Result::failure(
              ErrorCode::kIoError,
              "SocketCAN receive timeout",
              "receive",
              component_name(config_),
              true),
          {}};
    }

    pollfd descriptor{};
    descriptor.fd = fd_;
    descriptor.events = POLLIN;

    const int poll_result =
        ::poll(
            &descriptor,
            1,
            wait_ms);

    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string("CAN receive poll failed: ") + std::strerror(errno),
              "receive",
              component_name(config_),
              true),
          {}};
    }

    if (poll_result == 0) {
      return {
          Result::failure(
              ErrorCode::kIoError,
              "SocketCAN receive timeout",
              "receive",
              component_name(config_),
              true),
          {}};
    }

    if (poll_has_error(descriptor.revents)) {
      return {
          Result::failure(
              ErrorCode::kIoError,
              "SocketCAN receive fd error",
              "receive",
              component_name(config_),
              true),
          {}};
    }

    can_frame raw{};

    const ssize_t received =
        ::read(
            fd_,
            &raw,
            sizeof(raw));

    if (received < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string("SocketCAN read failed: ") + std::strerror(errno),
              "receive",
              component_name(config_),
              true),
          {}};
    }

    if (received != static_cast<ssize_t>(sizeof(raw))) {
      continue;
    }

    if ((raw.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) {
      continue;
    }

    const auto frame = from_linux_frame(raw);

    if (!matches_any_filter(frame, options.filters)) {
      continue;
    }

    diagnostics::trace_packet(
        "can",
        "rx",
        component_name(config_) + "/" + can_id_text(frame),
        frame.payload,
        can_detail(frame));

    return {Result::success("CAN frame received"), frame};
  }
}

CanReceiveResult SocketCanBus::receive(const CanReceiveOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (fd_ < 0 || health_ != DriverHealth::kReady) {
    return {
        Result::failure(
            ErrorCode::kConfigRequired,
            "SocketCAN bus is unavailable",
            "receive",
            component_name(config_),
            true),
        {}};
  }

  return receive_once_locked(options);
}

void SocketCanBus::stop() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);

  close_socket_locked();
  consecutive_failures_ = 0;
  installed_kernel_filters_.clear();
  health_ = DriverHealth::kStopped;

  diagnostics::info("can.socket", "closed " + component_name(config_));
}

}  // namespace snappy::drivers::bus

#else

namespace snappy::drivers::bus {

Result SocketCanBus::configure_interface_locked() {
  health_ = DriverHealth::kFault;

  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "configure",
      config_.bus_id);
}

Result SocketCanBus::open_socket_locked() {
  health_ = DriverHealth::kFault;

  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "open",
      config_.bus_id);
}

Result SocketCanBus::start() {
  std::lock_guard<std::mutex> lock(mutex_);

  health_ = DriverHealth::kFault;

  if (config_.allow_start_failure) {
    return Result::success("SocketCAN unsupported but isolated");
  }

  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "start",
      config_.bus_id);
}

Result SocketCanBus::send_once_locked(const CanFrame&) {
  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "send",
      config_.bus_id,
      true);
}

Result SocketCanBus::send(const CanFrame&) {
  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "send",
      config_.bus_id,
      true);
}

CanReceiveResult SocketCanBus::receive_once_locked(const CanReceiveOptions&) {
  return {
      Result::failure(
          ErrorCode::kUnsupported,
          "SocketCAN real backend requires Linux",
          "receive",
          config_.bus_id,
          true),
      {}};
}

CanReceiveResult SocketCanBus::receive(const CanReceiveOptions&) {
  return receive_once_locked({});
}

Result SocketCanBus::reopen_socket_locked() {
  health_ = DriverHealth::kFault;

  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "reopen",
      config_.bus_id);
}

Result SocketCanBus::recover_locked(const std::string&, bool) {
  health_ = DriverHealth::kFault;

  return Result::failure(
      ErrorCode::kUnsupported,
      "SocketCAN real backend requires Linux",
      "recover",
      config_.bus_id,
      true);
}

void SocketCanBus::apply_kernel_filter_locked(
    const std::vector<CanReceiveFilter>&) {}

void SocketCanBus::close_socket_locked() noexcept {}

void SocketCanBus::stop() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  health_ = DriverHealth::kStopped;
}

}  // namespace snappy::drivers::bus

#endif