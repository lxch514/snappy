#include "snappy/drivers/bus/rs485_port.hpp"
#include "snappy/drivers/diagnostics/driver_log.hpp"

/**
 * @file rs485_port.cpp
 * @brief Linux RS485 串口完整事务实现
 *
 * @details
 * - 一个对象只拥有并访问一个串口文件描述符。
 * - 请求发送前等待总线静默。
 * - 响应必须由请求专属 ResponseFramer 判定为完整帧。
 * - 短帧、CRC、站号、功能码和长度错误不能返回成功。
 * - 协议错误只影响当前设备，不直接标记物理端口故障。
 * - EIO、POLLERR、POLLHUP 等 fd 错误才标记端口故障。
 * - 失败事务在返回前完成有界迟到帧清理。
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace snappy::drivers::bus {

/* ============================================================
 * 配置和对象基础状态
 * ============================================================ */

bool valid_rs485_port_config(
    const Rs485PortConfig& config) noexcept {
  return !config.port_id.empty() &&
         !config.backend.empty() &&
         !config.device_path.empty() &&
         config.baud_rate > 0 &&
         config.format.data_bits >= 5 &&
         config.format.data_bits <= 8 &&
         config.format.stop_bits >= 1 &&
         config.format.stop_bits <= 2 &&
         config.inter_frame_gap.count() >= 0 &&
         config.read_idle_gap.count() > 0 &&
         config.late_frame_silence.count() > 0 &&
         config.late_frame_recovery_timeout.count() >=
             config.late_frame_silence.count() &&
         config.max_pending_transactions > 0 &&
         config.recovery_delay.count() >= 0;
}

Rs485Port::Rs485Port(
    Rs485PortConfig config)
    : config_(std::move(config)) {}

Rs485Port::~Rs485Port() {
  close();
}

const Rs485PortConfig&
Rs485Port::config() const noexcept {
  return config_;
}

DriverHealth Rs485Port::health()
    const noexcept {
  std::lock_guard<std::mutex> lock(
      mutex_);

  return health_;
}

}  // namespace snappy::drivers::bus

#if defined(__linux__)

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#if defined(TIOCGSERIAL) || \
    defined(TIOCSSERIAL) || \
    defined(TIOCGRS485) || \
    defined(TIOCSRS485)
#include <linux/serial.h>
#endif

namespace snappy::drivers::bus {
namespace {

/* ============================================================
 * 通用辅助函数
 * ============================================================ */

std::string component_name(
    const Rs485PortConfig& config) {
  return config.port_id +
         ":" +
         config.device_path;
}

bool baud_to_speed(
    int baud_rate,
    speed_t* speed) {
  if (speed == nullptr) {
    return false;
  }

  switch (baud_rate) {
    case 4800:
      *speed = B4800;
      return true;

    case 9600:
      *speed = B9600;
      return true;

    case 19200:
      *speed = B19200;
      return true;

    case 38400:
      *speed = B38400;
      return true;

    case 57600:
      *speed = B57600;
      return true;

    case 115200:
      *speed = B115200;
      return true;

#ifdef B230400
    case 230400:
      *speed = B230400;
      return true;
#endif

#ifdef B460800
    case 460800:
      *speed = B460800;
      return true;
#endif

#ifdef B921600
    case 921600:
      *speed = B921600;
      return true;
#endif

    default:
      return false;
  }
}

bool has_deadline(
    std::chrono::steady_clock::time_point value) {
  return value.time_since_epoch().count() != 0;
}

int remaining_ms(
    std::chrono::steady_clock::time_point deadline) {
  const auto remaining =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          deadline -
          std::chrono::steady_clock::now());

  if (remaining.count() <= 0) {
    return 0;
  }

  return static_cast<int>(
      remaining.count());
}

bool poll_has_error(
    short revents) {
  return (revents &
          (POLLERR |
           POLLHUP |
           POLLNVAL)) != 0;
}

bool is_port_level_error(
    const Result& result) {
  return !result.ok &&
         result.code ==
             ErrorCode::kIoError;
}

std::chrono::milliseconds
estimate_tx_drain_time(
    std::size_t bytes,
    const Rs485PortConfig& config) {
  int bits_per_character =
      1 +
      config.format.data_bits +
      config.format.stop_bits;

  if (config.format.parity !=
      Rs485Parity::kNone) {
    ++bits_per_character;
  }

  const int baud_rate =
      config.baud_rate > 0
          ? config.baud_rate
          : kDefaultRs485BaudRate;

  const auto total_bits =
      bytes *
      static_cast<std::size_t>(
          bits_per_character);

  const auto milliseconds =
      static_cast<int>(
          (total_bits * 1000U +
           static_cast<std::size_t>(
               baud_rate) -
           1U) /
          static_cast<std::size_t>(
              baud_rate));

  return std::chrono::milliseconds(
      std::clamp(
          milliseconds + 2,
          2,
          100));
}

/* ============================================================
 * 串口方向和 termios 配置
 * ============================================================ */

void try_set_low_latency(
    int fd) {
#if defined(TIOCGSERIAL) && \
    defined(TIOCSSERIAL) && \
    defined(ASYNC_LOW_LATENCY)
  serial_struct serial {};

  if (::ioctl(
          fd,
          TIOCGSERIAL,
          &serial) == 0) {
    serial.flags |=
        ASYNC_LOW_LATENCY;

    static_cast<void>(
        ::ioctl(
            fd,
            TIOCSSERIAL,
            &serial));
  }
#else
  static_cast<void>(fd);
#endif
}

Result configure_rs485_direction(
    int fd,
    const Rs485PortConfig& config) {
  /*
   * USB-RS485 自动换向模块不使用内核 RTS。
   * 每次打开串口时都主动清除可能残留的内核 RS485 状态。
   */
  if (config.direction_mode ==
          Rs485DirectionMode::kAuto ||
      config.direction_mode ==
          Rs485DirectionMode::kNone) {
#if defined(TIOCGRS485) && \
    defined(TIOCSRS485) && \
    defined(SER_RS485_ENABLED)
    serial_rs485 rs485 {};

    if (::ioctl(
            fd,
            TIOCGRS485,
            &rs485) == 0) {
      rs485.flags &=
          ~SER_RS485_ENABLED;

#ifdef SER_RS485_RTS_ON_SEND
      rs485.flags &=
          ~SER_RS485_RTS_ON_SEND;
#endif

#ifdef SER_RS485_RTS_AFTER_SEND
      rs485.flags &=
          ~SER_RS485_RTS_AFTER_SEND;
#endif

      rs485.delay_rts_before_send = 0;
      rs485.delay_rts_after_send = 0;

      /*
       * 某些 USB-RS485 驱动不支持 TIOCSRS485。
       * 这种情况不能阻止其他模块启动。
       */
      static_cast<void>(
          ::ioctl(
              fd,
              TIOCSRS485,
              &rs485));
    }
#endif

    return Result::success(
        "rs485 automatic direction mode ready");
  }

#if defined(TIOCSRS485) && \
    defined(SER_RS485_ENABLED)
  serial_rs485 desired {};

  if (config.direction_mode ==
      Rs485DirectionMode::
          kKernelRtsHighOnSend) {
#if defined(SER_RS485_RTS_ON_SEND) && \
    defined(SER_RS485_RTS_AFTER_SEND)
    desired.flags =
        SER_RS485_ENABLED |
        SER_RS485_RTS_ON_SEND;

    desired.flags &=
        ~SER_RS485_RTS_AFTER_SEND;
#else
    return Result::failure(
        ErrorCode::kUnsupported,
        "kernel RS485 RTS flags unavailable",
        "configure_direction",
        component_name(config));
#endif
  } else if (
      config.direction_mode ==
          Rs485DirectionMode::
              kKernelRtsLowOnSend) {
#if defined(SER_RS485_RTS_ON_SEND) && \
    defined(SER_RS485_RTS_AFTER_SEND)
    desired.flags =
        SER_RS485_ENABLED |
        SER_RS485_RTS_AFTER_SEND;

    desired.flags &=
        ~SER_RS485_RTS_ON_SEND;
#else
    return Result::failure(
        ErrorCode::kUnsupported,
        "kernel RS485 RTS flags unavailable",
        "configure_direction",
        component_name(config));
#endif
  } else {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "unknown rs485 direction mode",
        "configure_direction",
        component_name(config));
  }

  if (::ioctl(
          fd,
          TIOCSRS485,
          &desired) != 0) {
    return Result::failure(
        ErrorCode::kIoError,
        std::string(
            "TIOCSRS485 failed: ") +
            std::strerror(errno),
        "configure_direction",
        component_name(config),
        true);
  }

  return Result::success(
      "kernel RS485 direction configured");
#else
  return Result::failure(
      ErrorCode::kUnsupported,
      "kernel RS485 direction unavailable",
      "configure_direction",
      component_name(config));
#endif
}

Result configure_serial(
    int fd,
    const Rs485PortConfig& config) {
  speed_t speed {};

  if (!baud_to_speed(
          config.baud_rate,
          &speed)) {
    return Result::failure(
        ErrorCode::kInvalidArgument,
        "unsupported rs485 baud rate",
        "configure_serial",
        component_name(config));
  }

  termios tty {};

  if (::tcgetattr(
          fd,
          &tty) != 0) {
    return Result::failure(
        ErrorCode::kIoError,
        std::string(
            "tcgetattr failed: ") +
            std::strerror(errno),
        "configure_serial",
        component_name(config),
        true);
  }

  ::cfmakeraw(&tty);

  tty.c_cflag |=
      CLOCAL |
      CREAD;

  tty.c_cflag &=
      ~CSIZE;

  switch (config.format.data_bits) {
    case 5:
      tty.c_cflag |= CS5;
      break;

    case 6:
      tty.c_cflag |= CS6;
      break;

    case 7:
      tty.c_cflag |= CS7;
      break;

    case 8:
      tty.c_cflag |= CS8;
      break;

    default:
      return Result::failure(
          ErrorCode::kInvalidArgument,
          "unsupported rs485 data bits",
          "configure_serial",
          component_name(config));
  }

  tty.c_cflag &=
      ~(PARENB |
        PARODD);

  if (config.format.parity ==
      Rs485Parity::kEven) {
    tty.c_cflag |= PARENB;
  } else if (
      config.format.parity ==
          Rs485Parity::kOdd) {
    tty.c_cflag |=
        PARENB |
        PARODD;
  }

  if (config.format.stop_bits == 2) {
    tty.c_cflag |= CSTOPB;
  } else {
    tty.c_cflag &= ~CSTOPB;
  }

#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif

  tty.c_iflag &=
      ~(IXON |
        IXOFF |
        IXANY);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::cfsetispeed(
          &tty,
          speed) != 0 ||
      ::cfsetospeed(
          &tty,
          speed) != 0) {
    return Result::failure(
        ErrorCode::kIoError,
        "cfset speed failed",
        "configure_serial",
        component_name(config),
        true);
  }

  if (::tcsetattr(
          fd,
          TCSANOW,
          &tty) != 0) {
    return Result::failure(
        ErrorCode::kIoError,
        std::string(
            "tcsetattr failed: ") +
            std::strerror(errno),
        "configure_serial",
        component_name(config),
        true);
  }

  /*
   * 显式方向模式失败必须使端口启动失败。
   * AUTO 在不支持内核方向查询时仍会返回成功。
   */
  const auto direction =
      configure_rs485_direction(
          fd,
          config);

  if (!direction.ok) {
    return direction;
  }

  /*
   * 只允许在端口刚打开时清理历史数据。
   * 普通事务开始前不能直接 TCIFLUSH。
   */
  if (::tcflush(
          fd,
          TCIOFLUSH) != 0) {
    return Result::failure(
        ErrorCode::kIoError,
        std::string(
            "tcflush failed: ") +
            std::strerror(errno),
        "configure_serial",
        component_name(config),
        true);
  }

  if (config.low_latency) {
    try_set_low_latency(fd);
  }

  return Result::success(
      "rs485 serial configured");
}

/* ============================================================
 * 请求发送
 * ============================================================ */

Result wait_writable(
    int fd,
    std::chrono::steady_clock::time_point deadline,
    const Rs485PortConfig& config) {
  while (true) {
    const int wait_ms =
        remaining_ms(deadline);

    if (wait_ms <= 0) {
      return Result::failure(
          ErrorCode::kRs485Timeout,
          "rs485 write timeout",
          "write",
          component_name(config),
          true);
    }

    pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLOUT;

    const int ret =
        ::poll(
            &pfd,
            1,
            wait_ms);

    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }

      return Result::failure(
          ErrorCode::kIoError,
          std::string(
              "rs485 write poll failed: ") +
              std::strerror(errno),
          "write",
          component_name(config),
          true);
    }

    if (ret == 0) {
      return Result::failure(
          ErrorCode::kRs485Timeout,
          "rs485 write poll timeout",
          "write",
          component_name(config),
          true);
    }

    if (poll_has_error(
            pfd.revents)) {
      return Result::failure(
          ErrorCode::kIoError,
          "rs485 write poll reported fd error",
          "write",
          component_name(config),
          true);
    }

    return Result::success();
  }
}

Result write_all(
    int fd,
    const std::vector<std::uint8_t>& payload,
    std::chrono::steady_clock::time_point deadline,
    const Rs485PortConfig& config) {
  std::size_t written_total = 0;

  while (written_total <
         payload.size()) {
    const auto writable =
        wait_writable(
            fd,
            deadline,
            config);

    if (!writable.ok) {
      return writable;
    }

    const ssize_t written =
        ::write(
            fd,
            payload.data() +
                written_total,
            payload.size() -
                written_total);

    if (written < 0) {
      if (errno == EINTR ||
          errno == EAGAIN ||
          errno == EWOULDBLOCK) {
        continue;
      }

      return Result::failure(
          ErrorCode::kIoError,
          std::string(
              "rs485 write failed: ") +
              std::strerror(errno),
          "write",
          component_name(config),
          true);
    }

    if (written == 0) {
      continue;
    }

    written_total +=
        static_cast<std::size_t>(
            written);
  }

  const auto drain_time =
      estimate_tx_drain_time(
          payload.size(),
          config);

  if (std::chrono::steady_clock::now() +
          drain_time >
      deadline) {
    return Result::failure(
        ErrorCode::kRs485Timeout,
        "rs485 request sent but tx drain exceeded deadline",
        "write",
        component_name(config),
        true);
  }

  std::this_thread::sleep_for(
      drain_time);

  return Result::success(
      "rs485 request written");
}

/* ============================================================
 * 总线静默和迟到帧清理
 * ============================================================ */

struct DrainResult {
  Result result;
  std::size_t discarded_bytes{0};
};

DrainResult drain_until_silent(
    int fd,
    std::chrono::milliseconds silence_duration,
    std::chrono::milliseconds maximum_duration,
    const Rs485PortConfig& config) {
  if (silence_duration.count() <= 0 ||
      maximum_duration.count() <= 0) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "invalid rs485 drain duration",
            "drain",
            component_name(config)),
        0};
  }

  const auto started =
      std::chrono::steady_clock::now();

  const auto deadline =
      started +
      maximum_duration;

  auto last_activity =
      started;

  std::size_t discarded_bytes = 0;

  while (true) {
    const auto now =
        std::chrono::steady_clock::now();

    if (now -
            last_activity >=
        silence_duration) {
      return {
          Result::success(
              "rs485 bus reached silent interval"),
          discarded_bytes};
    }

    if (now >= deadline) {
      return {
          Result::failure(
              ErrorCode::kRs485Timeout,
              "rs485 late-frame recovery did not reach silence",
              "drain",
              component_name(config),
              true),
          discarded_bytes};
    }

    const auto until_silent =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            silence_duration -
            (now - last_activity));

    const auto until_deadline =
        std::chrono::duration_cast<
            std::chrono::milliseconds>(
            deadline - now);

    const auto wait_duration =
        std::max(
            std::chrono::milliseconds(1),
            std::min(
                until_silent,
                until_deadline));

    pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLIN;

    const int ret =
        ::poll(
            &pfd,
            1,
            static_cast<int>(
                wait_duration.count()));

    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string(
                  "rs485 drain poll failed: ") +
                  std::strerror(errno),
              "drain",
              component_name(config),
              true),
          discarded_bytes};
    }

    if (ret == 0) {
      continue;
    }

    if (poll_has_error(
            pfd.revents)) {
      return {
          Result::failure(
              ErrorCode::kIoError,
              "rs485 drain poll reported fd error",
              "drain",
              component_name(config),
              true),
          discarded_bytes};
    }

    std::uint8_t buffer[256] {};

    const ssize_t read_count =
        ::read(
            fd,
            buffer,
            sizeof(buffer));

    if (read_count < 0) {
      if (errno == EINTR ||
          errno == EAGAIN ||
          errno == EWOULDBLOCK) {
        continue;
      }

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string(
                  "rs485 drain read failed: ") +
                  std::strerror(errno),
              "drain",
              component_name(config),
              true),
          discarded_bytes};
    }

    if (read_count > 0) {
      discarded_bytes +=
          static_cast<std::size_t>(
              read_count);

      last_activity =
          std::chrono::steady_clock::now();
    }
  }
}

bool should_recover_late_frame(
    const Result& result) {
  switch (result.code) {
    case ErrorCode::kRs485Timeout:
    case ErrorCode::kRs485FrameIncomplete:
    case ErrorCode::kRs485CrcMismatch:
    case ErrorCode::kRs485AddressMismatch:
    case ErrorCode::kRs485FunctionMismatch:
    case ErrorCode::kRs485LengthMismatch:
    case ErrorCode::kRs485ExceptionResponse:
      return true;

    default:
      return false;
  }
}

/* ============================================================
 * 完整帧处理
 * ============================================================ */

std::vector<std::uint8_t> slice_frame(
    const std::vector<std::uint8_t>& buffer,
    std::size_t offset,
    std::size_t size) {
  if (size == 0 ||
      offset >= buffer.size() ||
      offset + size >
          buffer.size()) {
    return {};
  }

  return {
      buffer.begin() +
          static_cast<std::ptrdiff_t>(
              offset),
      buffer.begin() +
          static_cast<std::ptrdiff_t>(
              offset + size)};
}

std::optional<Rs485TransactionResult>
consume_framed_buffer(
    std::vector<std::uint8_t>& buffer,
    const Rs485Transaction& transaction,
    const Rs485PortConfig& config) {
  while (!buffer.empty()) {
    const auto scan =
        transaction.response_framer(
            buffer);

    switch (scan.state) {
      case Rs485FrameScanState::kNeedMore:
        return std::nullopt;

      case Rs485FrameScanState::kDiscardPrefix: {
        std::size_t discard =
            scan.frame_offset;

        if (discard == 0) {
          discard = 1;
        }

        discard =
            std::min(
                discard,
                buffer.size());

        buffer.erase(
            buffer.begin(),
            buffer.begin() +
                static_cast<std::ptrdiff_t>(
                    discard));

        break;
      }

      case Rs485FrameScanState::kProtocolError: {
        auto frame =
            slice_frame(
                buffer,
                scan.frame_offset,
                scan.frame_size);

        if (frame.empty()) {
          frame = buffer;
        }

        return Rs485TransactionResult{
            scan.result,
            std::move(frame)};
      }

      case Rs485FrameScanState::kMatched: {
        auto frame =
            slice_frame(
                buffer,
                scan.frame_offset,
                scan.frame_size);

        if (frame.empty()) {
          return Rs485TransactionResult{
              Result::failure(
                  ErrorCode::kRs485FrameIncomplete,
                  "response framer returned invalid frame window",
                  "frame",
                  component_name(config),
                  true),
              buffer};
        }

        return Rs485TransactionResult{
            Result::success(
                "rs485 complete response frame"),
            std::move(frame)};
      }
    }
  }

  return std::nullopt;
}

Rs485TransactionResult read_response(
    int fd,
    const Rs485Transaction& transaction,
    std::chrono::steady_clock::time_point deadline,
    const Rs485PortConfig& config) {
  std::vector<std::uint8_t> response;

  response.reserve(
      std::min<std::size_t>(
          transaction.max_response_bytes,
          512));

  while (true) {
    const auto framed =
        consume_framed_buffer(
            response,
            transaction,
            config);

    if (framed) {
      return *framed;
    }

    const int wait_ms =
        remaining_ms(deadline);

    if (wait_ms <= 0) {
      if (response.empty()) {
        return {
            Result::failure(
                ErrorCode::kRs485Timeout,
                "rs485 response timeout",
                "read",
                component_name(config),
                true),
            {}};
      }

      return {
          Result::failure(
              ErrorCode::kRs485FrameIncomplete,
              "rs485 response frame incomplete before deadline",
              "read",
              component_name(config),
              true),
          response};
    }

    pollfd pfd {};
    pfd.fd = fd;
    pfd.events = POLLIN;

    const int poll_timeout =
        std::max(
            1,
            std::min(
                wait_ms,
                static_cast<int>(
                    config.read_idle_gap.count())));

    const int ret =
        ::poll(
            &pfd,
            1,
            poll_timeout);

    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string(
                  "rs485 read poll failed: ") +
                  std::strerror(errno),
              "read",
              component_name(config),
              true),
          response};
    }

    if (ret == 0) {
      /*
       * 暂时静默不等于帧完成。
       * 继续等待总事务 deadline。
       */
      continue;
    }

    if (poll_has_error(
            pfd.revents)) {
      return {
          Result::failure(
              ErrorCode::kIoError,
              "rs485 read poll reported fd error",
              "read",
              component_name(config),
              true),
          response};
    }

    if (response.size() >=
        transaction.max_response_bytes) {
      return {
          Result::failure(
              ErrorCode::kRs485LengthMismatch,
              "rs485 response exceeded maximum frame size",
              "read",
              component_name(config),
              true),
          response};
    }

    std::uint8_t buffer[256] {};

    const std::size_t available =
        std::min(
            sizeof(buffer),
            transaction.max_response_bytes -
                response.size());

    const ssize_t read_count =
        ::read(
            fd,
            buffer,
            available);

    if (read_count < 0) {
      if (errno == EINTR ||
          errno == EAGAIN ||
          errno == EWOULDBLOCK) {
        continue;
      }

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string(
                  "rs485 read failed: ") +
                  std::strerror(errno),
              "read",
              component_name(config),
              true),
          response};
    }

    if (read_count == 0) {
      continue;
    }

    response.insert(
        response.end(),
        buffer,
        buffer +
            static_cast<std::size_t>(
                read_count));
  }
}

void mark_port_fault(
    int* fd,
    DriverHealth* health) {
  if (fd != nullptr &&
      *fd >= 0) {
    ::close(*fd);
    *fd = -1;
  }

  if (health != nullptr) {
    *health =
        DriverHealth::kFault;
  }
}

void append_recovery_result(
    Result* transaction_result,
    const DrainResult& recovery) {
  if (transaction_result == nullptr) {
    return;
  }

  transaction_result->detail +=
      "; discarded_late_bytes=" +
      std::to_string(
          recovery.discarded_bytes);

  if (!recovery.result.ok) {
    transaction_result->detail +=
        "; late_frame_recovery=" +
        recovery.result.detail;
  }
}

struct FlushAfterTransaction {
  int fd{-1};

  ~FlushAfterTransaction() {
    if (fd >= 0) {
      static_cast<void>(
          ::tcflush(
              fd,
              TCIOFLUSH));
    }
  }
};

}  // namespace

/* ============================================================
 * 端口打开和关闭
 * ============================================================ */

Result Rs485Port::open() {
  std::lock_guard<std::mutex> lock(
      mutex_);

  if (fd_ >= 0 &&
      health_ ==
          DriverHealth::kReady) {
    return Result::success(
        "rs485 port already open");
  }

  if (!valid_rs485_port_config(
          config_)) {
    health_ =
        DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kInvalidArgument,
        "invalid rs485 port config",
        "open",
        config_.port_id);
  }

  if (config_.backend != "serial" &&
      config_.backend != "real") {
    health_ =
        DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kUnsupported,
        "unsupported rs485 backend: " +
            config_.backend,
        "open",
        config_.port_id);
  }

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  fd_ =
      ::open(
          config_.device_path.c_str(),
          O_RDWR |
              O_NOCTTY |
              O_NONBLOCK |
              O_CLOEXEC);

  if (fd_ < 0) {
    health_ =
        DriverHealth::kFault;

    return Result::failure(
        ErrorCode::kIoError,
        std::string(
            "open rs485 port failed: ") +
            std::strerror(errno),
        "open",
        component_name(config_),
        true);
  }

  const auto configured =
      configure_serial(
          fd_,
          config_);

  if (!configured.ok) {
    mark_port_fault(
        &fd_,
        &health_);

    return configured;
  }

  health_ =
      DriverHealth::kReady;

  return Result::success(
      "rs485 port open");
}

void Rs485Port::close()
    noexcept {
  std::lock_guard<std::mutex> lock(
      mutex_);

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  health_ =
      DriverHealth::kStopped;
}

/* ============================================================
 * 单次完整事务
 * ============================================================ */

Rs485TransactionResult
Rs485Port::transact(
    const Rs485Transaction& transaction) {
  std::lock_guard<std::mutex> lock(
      mutex_);

  if (fd_ < 0 ||
      health_ !=
          DriverHealth::kReady) {
    return {
        Result::failure(
            ErrorCode::kConfigRequired,
            "rs485 port is not open",
            "transact",
            component_name(config_),
            true),
        {}};
  }

  if (transaction.request.empty() ||
      transaction.max_response_bytes == 0 ||
      transaction.timeout.count() <= 0) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "invalid rs485 transaction",
            "transact",
            component_name(config_)),
        {}};
  }

  /*
   * P0-3：真实事务不允许依靠“串口暂时静默”
   * 将任意已读字节视为成功。
   */
  if (!transaction.require_complete_frame ||
      !transaction.response_framer) {
    return {
        Result::failure(
            ErrorCode::kInvalidArgument,
            "rs485 transaction requires a complete-frame framer",
            "transact",
            component_name(config_)),
        {}};
  }

  FlushAfterTransaction
    flush_after_transaction{fd_};

  const auto deadline =
      has_deadline(
          transaction.deadline)
          ? transaction.deadline
          : std::chrono::steady_clock::now() +
                transaction.timeout;

  if (std::chrono::steady_clock::now() >=
      deadline) {
    return {
        Result::failure(
            ErrorCode::kRs485Timeout,
            "rs485 transaction expired before execution",
            "deadline",
            component_name(config_),
            true),
        {}};
  }

    /* ============================================================
   * 相邻 RS485 事务最小静默时间
   *
   * 开关电源协议要求控制器相邻数据包至少间隔 12.5ms。
   * ============================================================ */
  if (config_.inter_frame_gap.count() > 0) {
    const auto gap_deadline =
        std::chrono::steady_clock::now() +
        config_.inter_frame_gap;

    if (gap_deadline > deadline) {
      return {
          Result::failure(
              ErrorCode::kRs485Timeout,
              "rs485 inter-frame gap exceeded transaction deadline",
              "inter_frame_gap",
              component_name(config_),
              true),
          {}};
    }

    std::this_thread::sleep_until(gap_deadline);
  }

    /*
     * 每次事务开始前清除上一笔事务残留数据。
     */
    if (::tcflush(
            fd_,
            TCIFLUSH) != 0) {
      mark_port_fault(
          &fd_,
          &health_);

      return {
          Result::failure(
              ErrorCode::kIoError,
              std::string(
                  "tcflush input failed: ") +
                  std::strerror(errno),
              "transact",
              component_name(config_),
              true),
          {}};
    }

  auto written =
      write_all(
          fd_,
          transaction.request,
          deadline,
          config_);

  if (written.ok) {
  diagnostics::debug(
      "rs485.port",
      "tx_wire_sent"
      " tx_id=" +
          std::to_string(
              diagnostics::
                  current_rs485_diagnostic_tx_id()) +
          " logical_device=" +
          diagnostics::
              current_rs485_diagnostic_logical_device() +
          " endpoint=" +
          diagnostics::
              current_rs485_diagnostic_endpoint() +
          " bytes=" +
          std::to_string(
              transaction.request.size()));
  } else {
  diagnostics::warn(
      "rs485.port",
      "tx_wire_failed"
      " tx_id=" +
          std::to_string(
              diagnostics::
                  current_rs485_diagnostic_tx_id()) +
          " logical_device=" +
          diagnostics::
              current_rs485_diagnostic_logical_device() +
          " endpoint=" +
          diagnostics::
              current_rs485_diagnostic_endpoint() +
          " bytes=" +
          std::to_string(
              transaction.request.size()) +
          " code=" +
          std::to_string(
              static_cast<int>(
                  written.code)) +
          " detail=" +
          written.detail);
 }

  if (!written.ok) {
    if (is_port_level_error(
            written)) {
      mark_port_fault(
          &fd_,
          &health_);

      return {
          written,
          {}};
    }

    /*
     * 请求可能只发送了一部分。
     * 返回前仍需清理可能出现的迟到响应。
     */
    const auto recovery =
        drain_until_silent(
            fd_,
            config_.late_frame_silence,
            config_.late_frame_recovery_timeout,
            config_);

    append_recovery_result(
        &written,
        recovery);

    if (is_port_level_error(
            recovery.result)) {
      mark_port_fault(
          &fd_,
          &health_);
    }

    return {
        written,
        {}};
  }

  auto response =
    read_response(
        fd_,
        transaction,
        deadline,
        config_);

/*
 * ============================================================
 * 诊断日志：响应帧结果
 *
 * 注意：
 * 这里只记录 read_response() 已经得到的结果。
 * 不增加任何新的读取、不改变恢复流程。
 * ============================================================
 */
  if (response.result.ok) {
  diagnostics::debug(
      "rs485.port",
      "rx_frame_received"
      " tx_id=" +
          std::to_string(
              diagnostics::
                  current_rs485_diagnostic_tx_id()) +
          " logical_device=" +
          diagnostics::
              current_rs485_diagnostic_logical_device() +
          " endpoint=" +
          diagnostics::
              current_rs485_diagnostic_endpoint() +
          " bytes=" +
          std::to_string(
              response.response.size()));
  } else {
  diagnostics::warn(
      "rs485.port",
      "rx_frame_failed"
      " tx_id=" +
          std::to_string(
              diagnostics::
                  current_rs485_diagnostic_tx_id()) +
          " logical_device=" +
          diagnostics::
              current_rs485_diagnostic_logical_device() +
          " endpoint=" +
          diagnostics::
              current_rs485_diagnostic_endpoint() +
          " code=" +
          std::to_string(
              static_cast<int>(
                  response.result.code)) +
          " detail=" +
          response.result.detail);
  }

/*
 * timeout、短帧、CRC、地址、功能码、长度错误
 * 均在释放上层 WCH 门禁前完成迟到帧恢复。
 */
  if (!response.result.ok &&
    should_recover_late_frame(
        response.result)) {
    const auto recovery =
        drain_until_silent(
            fd_,
            config_.late_frame_silence,
            config_.late_frame_recovery_timeout,
            config_);

    append_recovery_result(
        &response.result,
        recovery);

    if (is_port_level_error(
            recovery.result)) {
      mark_port_fault(
          &fd_,
          &health_);

      return {
          recovery.result,
          response.response};
    }
  }

  if (is_port_level_error(
          response.result)) {
    mark_port_fault(
        &fd_,
        &health_);
  }

  return response;
 }

}  // namespace snappy::drivers::bus

#else

namespace snappy::drivers::bus {

/* ============================================================
 * 非 Linux 平台
 * ============================================================ */

Result Rs485Port::open() {
  std::lock_guard<std::mutex> lock(
      mutex_);

  health_ =
      DriverHealth::kFault;

  return Result::failure(
      ErrorCode::kUnsupported,
      "rs485 real backend requires Linux",
      "open",
      config_.port_id);
}

void Rs485Port::close()
    noexcept {
  std::lock_guard<std::mutex> lock(
      mutex_);

  fd_ = -1;

  health_ =
      DriverHealth::kStopped;
}

Rs485TransactionResult
Rs485Port::transact(
    const Rs485Transaction&) {
  return {
      Result::failure(
          ErrorCode::kUnsupported,
          "rs485 real backend requires Linux",
          "transact",
          config_.port_id),
      {}};
}

}  // namespace snappy::drivers::bus

#endif