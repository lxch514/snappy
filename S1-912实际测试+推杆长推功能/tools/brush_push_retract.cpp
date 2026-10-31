/**
 * @file brush_push_retract.cpp
 * @brief 毛刷滚筒推杆“缩回”专用单文件工具（独立于整机程序）
 *
 * ============================================================
 * 用途
 * ============================================================
 * 现场问题：继电器驱动毛刷推杆伸出后整机断电，推杆停在伸出位置；
 * 上电后整机程序把它当成“新的零点”，遥控放下（缩回）会认为
 * “已经在 0 位、不需要动”，于是无法回到最初零点。
 *
 * 本工具绕过整机程序，直接对继电器模块写线圈，让毛刷推杆缩回。
 *
 * ============================================================
 * 硬件与协议（与 rk3588/config 中 actuator-only 配置一致）
 * ============================================================
 *   继电器模块 : ZS-DIO-R-10A-4-R-485
 *   串口       : /dev/ttyACM2   9600 8N1（与毛刷滚筒共用该 USB-RS485）
 *   站号       : 0x43
 *   线圈       : 通道1 = 0x0000 = 毛刷推杆 伸出
 *                通道2 = 0x0001 = 毛刷滚筒 缩回   <-- 本工具默认操作
 *                通道3 = 0x0002 = 刮条推杆 伸出
 *                通道4 = 0x0003 = 刮条推杆 缩回
 *   Modbus     : FC05 写单线圈（ON=0xFF00 / OFF=0x0000）
 *                FC01 读线圈（回读校验）
 *
 * ============================================================
 * 编译与运行
 * ============================================================
 *   g++ -O2 -std=c++17 -Wall -Wextra -o brush_push_retract brush_push_retract.cpp
 *
 *   # 1. 必须先停掉自启动，避免两个进程抢同一个串口
 *   sudo systemctl stop snappy-control.service snappy-gateway.service
 *
 *   # 2. 缩回毛刷推杆（默认通电 9100 ms，等于配置里的最大通电时间）
 *   sudo ./brush_push_retract
 *
 *   # 3. 只断电、不动
 *   sudo ./brush_push_retract --stop
 *
 *   # 4. 自定义通电时间 / 不校验
 *   sudo ./brush_push_retract --ms 6000
 *   sudo ./brush_push_retract --ms 6000 --port /dev/ttyACM2 --force
 *
 *   # 5. 只读毛刷驱动器寄存器（判断“按一下动一下”是不是驱动器自己停机）
 *   sudo ./brush_push_retract --brush-status
 *
 *   # 6. 只读开关电源电流（确认推杆电走哪一路电源）
 *   sudo ./brush_push_retract --psu-status --ms 15000
 *
 *   # 7. 电流判停：一边缩一边看电流，到底就停（不靠猜时间）
 *   sudo ./brush_push_retract --current-stop --psu-addr 0x71 --delta-a 1.0
 *
 * 关于“怎么知道完全缩回”
 * ------------------------------------------------------------
 * 推杆是继电器直驱，程序拿不到位置（没有编码器/到位信号）。
 * 整机程序也是按时间估算：行程上限 300 mm ÷ 33 mm/s ≈ 9.09 s，
 * 对应配置里的 brush_push_motor_max_on_time_ms = 9100。
 * 所以：
 *   - 默认会通电 9100 ms 后断电，不会一直缩；
 *   - 想知道“真到底”，两条路：
 *       a) 分次试：先 --ms 4000 看位置，不够再加 2000，宁短勿长；
 *       b) --current-stop：用开关电源电流跳升作为到底信号自动断电。
 *
 * 退出码：0 成功；1 参数或串口错误；2 运行中出现通信失败
 * ============================================================
 */

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <dirent.h>
#include <fcntl.h>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

/* ============================================================
 * 可调常量
 * ============================================================ */

constexpr const char* kDefaultPort = "/dev/ttyACM2";
constexpr int kDefaultBaud = 9600;
constexpr std::uint8_t kDefaultStationId = 0x43;

/* 毛刷滚筒驱动器（BD-201S）站号 */
constexpr std::uint8_t kDefaultBrushStationId = 0x03;

/* PDS3000 开关电源（用电流判断推杆到底） */
constexpr const char* kDefaultPsuPort = "/dev/ttyS3";
constexpr std::uint8_t kDefaultPsuStationId = 0x71;   // 24V 电源
constexpr int kDefaultBaselineMs = 1000;
constexpr double kDefaultDeltaA = 1.0;
constexpr double kDefaultMaxA = 6.0;

/* 默认操作通道：2 = 毛刷滚筒缩回 */
constexpr std::uint16_t kDefaultRetractChannel = 2;
constexpr std::uint16_t kDefaultExtendChannel = 1;

/* 与配置 hardware.debug.actuator-only.json 对齐 */
constexpr int kDefaultHoldMs = 9100;      // brush_push_motor_max_on_time_ms
constexpr int kDeadTimeMs = 200;          // push_motor_dead_time_ms
constexpr int kKeepAliveMs = 500;         // 继电器看门狗保活周期
constexpr int kReadTimeoutMs = 300;       // response_deadline_ms
constexpr int kInterFrameGapMs = 13;      // inter_frame_gap_ms

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) {
  g_stop = 1;
}

/* ============================================================
 * Modbus RTU 基础
 * ============================================================ */

std::uint16_t crc16_modbus(const std::vector<std::uint8_t>& data) {
  std::uint16_t crc = 0xFFFF;
  for (std::uint8_t byte : data) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x0001U) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001U)
                            : static_cast<std::uint16_t>(crc >> 1);
    }
  }
  return crc;
}

void append_crc(std::vector<std::uint8_t>& frame) {
  const std::uint16_t crc = crc16_modbus(frame);
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFFU));
}

void append_u16_be(std::vector<std::uint8_t>& frame, std::uint16_t value) {
  frame.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
  frame.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

std::string hex_of(const std::vector<std::uint8_t>& data) {
  static const char* digits = "0123456789ABCDEF";
  std::string text;
  for (std::size_t index = 0; index < data.size(); ++index) {
    if (index != 0) {
      text.push_back(' ');
    }
    text.push_back(digits[(data[index] >> 4) & 0x0FU]);
    text.push_back(digits[data[index] & 0x0FU]);
  }
  return text;
}

std::vector<std::uint8_t> build_write_single_coil(
    std::uint8_t station_id, std::uint16_t coil_address, bool energized) {
  std::vector<std::uint8_t> frame;
  frame.reserve(8);
  frame.push_back(station_id);
  frame.push_back(0x05);
  append_u16_be(frame, coil_address);
  append_u16_be(frame, energized ? 0xFF00U : 0x0000U);
  append_crc(frame);
  return frame;
}

std::vector<std::uint8_t> build_read_coils(
    std::uint8_t station_id, std::uint16_t first_coil, std::uint16_t count) {
  std::vector<std::uint8_t> frame;
  frame.reserve(8);
  frame.push_back(station_id);
  frame.push_back(0x01);
  append_u16_be(frame, first_coil);
  append_u16_be(frame, count);
  append_crc(frame);
  return frame;
}

/** FC03/FC04 读寄存器：读毛刷驱动器（FC03）和电源模块（FC04）用 */
std::vector<std::uint8_t> build_read_registers(
    std::uint8_t station_id, std::uint8_t function_code,
    std::uint16_t first_register, std::uint16_t count) {
  std::vector<std::uint8_t> frame;
  frame.reserve(8);
  frame.push_back(station_id);
  frame.push_back(function_code);
  append_u16_be(frame, first_register);
  append_u16_be(frame, count);
  append_crc(frame);
  return frame;
}

/** FC03 读保持寄存器 */
std::vector<std::uint8_t> build_read_holding(
    std::uint8_t station_id, std::uint16_t first_register, std::uint16_t count) {
  return build_read_registers(station_id, 0x03, first_register, count);
}

/** FC04 读输入寄存器：PDS3000 电源模块用 */
std::vector<std::uint8_t> build_read_input(
    std::uint8_t station_id, std::uint16_t first_register, std::uint16_t count) {
  return build_read_registers(station_id, 0x04, first_register, count);
}

/* ============================================================
 * 串口
 * ============================================================ */

speed_t baud_constant(int baud) {
  switch (baud) {
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return 0;
  }
}

/**
 * @brief 扫描 /proc，判断串口是否已被其他进程打开
 *
 * 整机程序运行时会一直持有 /dev/ttyACM2；
 * 两个进程同时写同一个串口会把帧搅乱，必须先停服务。
 */
bool port_held_by_other_process(const std::string& port) {
  DIR* proc = ::opendir("/proc");
  if (proc == nullptr) {
    return false;
  }

  bool held = false;
  const std::string self = std::to_string(::getpid());

  while (dirent* entry = ::readdir(proc)) {
    const std::string name = entry->d_name;
    if (name.empty() || name[0] < '0' || name[0] > '9' || name == self) {
      continue;
    }

    const std::string fd_dir = "/proc/" + name + "/fd";
    DIR* fds = ::opendir(fd_dir.c_str());
    if (fds == nullptr) {
      continue;
    }

    while (dirent* fd = ::readdir(fds)) {
      const std::string fd_name = fd->d_name;
      if (fd_name == "." || fd_name == "..") {
        continue;
      }

      const std::string link = fd_dir + "/" + fd_name;
      char target[512] = {};
      const ssize_t length =
          ::readlink(link.c_str(), target, sizeof(target) - 1);

      if (length > 0 && port == std::string(target, static_cast<std::size_t>(length))) {
        std::printf("[警告] 串口 %s 已被进程 PID=%s 打开，"
                    "请先停止自启动服务。\n",
                    port.c_str(), name.c_str());
        held = true;
        break;
      }
    }

    ::closedir(fds);

    if (held) {
      break;
    }
  }

  ::closedir(proc);
  return held;
}

class RelayPort {
 public:
  ~RelayPort() {
    close_port();
  }

  bool open_port(const std::string& port, int baud) {
    const speed_t speed = baud_constant(baud);
    if (speed == 0) {
      std::fprintf(stderr, "[错误] 不支持的波特率: %d\n", baud);
      return false;
    }

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      std::fprintf(stderr, "[错误] 无法打开 %s: %s\n", port.c_str(),
                   std::strerror(errno));
      std::fprintf(stderr, "       检查：设备是否存在 / 权限（sudo）/ "
                           "USB-RS485 是否已进入 host 模式\n");
      return false;
    }

    termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
      std::fprintf(stderr, "[错误] tcgetattr 失败: %s\n", std::strerror(errno));
      close_port();
      return false;
    }

    ::cfsetispeed(&options, speed);
    ::cfsetospeed(&options, speed);

    options.c_cflag &= ~PARENB;   // 无校验
    options.c_cflag &= ~CSTOPB;   // 1 位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;       // 8 位数据
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= CREAD | CLOCAL;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                         ICRNL);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
      std::fprintf(stderr, "[错误] tcsetattr 失败: %s\n", std::strerror(errno));
      close_port();
      return false;
    }

    ::tcflush(fd_, TCIOFLUSH);

    /* 清掉上电残留的半个帧 */
    std::this_thread::sleep_for(std::chrono::milliseconds(kInterFrameGapMs * 2));
    ::tcflush(fd_, TCIFLUSH);

    std::printf("[串口] %s 已打开（%d 8N1）\n", port.c_str(), baud);
    return true;
  }

  void close_port() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  [[nodiscard]] bool is_open() const {
    return fd_ >= 0;
  }

  /**
   * @brief 发送一帧并收取一帧
   * @param expected_min_length 期望的最小响应长度
   */
  bool transact(const std::vector<std::uint8_t>& request,
                std::vector<std::uint8_t>& response,
                std::size_t expected_min_length) {
    response.clear();

    if (fd_ < 0) {
      return false;
    }

    /* 遵守 inter_frame_gap，保证总线空闲后再发 */
    std::this_thread::sleep_for(std::chrono::milliseconds(kInterFrameGapMs));
    ::tcflush(fd_, TCIOFLUSH);

    const ssize_t written = ::write(fd_, request.data(), request.size());
    if (written != static_cast<ssize_t>(request.size())) {
      std::fprintf(stderr, "[错误] 写串口失败: %s\n", std::strerror(errno));
      return false;
    }

    ::tcdrain(fd_);

    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::milliseconds(kReadTimeoutMs);

    /*
     * 帧结束判定按“总线空闲间隔”，不能只按最小长度：
     * FC01 的应答长度依赖 byte_count，提前截断会把 CRC 丢掉。
     * read_idle_gap_ms 配置为 6 ms，这里留一点余量。
     */
    const auto idle_gap = std::chrono::milliseconds(8);
    auto last_data_at = start;

    std::uint8_t buffer[256];

    while (std::chrono::steady_clock::now() < deadline) {
      fd_set read_set;
      FD_ZERO(&read_set);
      FD_SET(fd_, &read_set);

      timeval timeout{};
      timeout.tv_sec = 0;
      timeout.tv_usec = 5000;   // 5 ms

      const int ready = ::select(fd_ + 1, &read_set, nullptr, nullptr, &timeout);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        std::fprintf(stderr, "[错误] select 失败: %s\n", std::strerror(errno));
        return false;
      }

      const auto now = std::chrono::steady_clock::now();

      if (ready > 0) {
        const ssize_t received = ::read(fd_, buffer, sizeof(buffer));
        if (received > 0) {
          response.insert(response.end(), buffer,
                          buffer + static_cast<std::size_t>(received));
          last_data_at = now;
          continue;
        }

        if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
            errno != EINTR) {
          std::fprintf(stderr, "[错误] 读串口失败: %s\n", std::strerror(errno));
          return false;
        }
      }

      if (!response.empty() &&
          response.size() >= expected_min_length &&
          now - last_data_at >= idle_gap) {
        break;   // 帧已经收完
      }
    }

    if (response.empty()) {
      return false;
    }

    return true;
  }

 private:
  int fd_{-1};
};

/* ============================================================
 * 继电器操作
 * ============================================================ */

bool check_crc(const std::vector<std::uint8_t>& frame) {
  if (frame.size() < 4) {
    return false;
  }

  const std::vector<std::uint8_t> body(frame.begin(), frame.end() - 2);
  const std::uint16_t received =
      static_cast<std::uint16_t>(frame[frame.size() - 2]) |
      static_cast<std::uint16_t>(
          static_cast<std::uint16_t>(frame[frame.size() - 1]) << 8);

  return received == crc16_modbus(body);
}

/**
 * @brief 写单个线圈（FC05）
 *
 * 内部最多重试 3 次；写应答丢失时用 FC01 回读兜底确认。
 */
bool set_coil(RelayPort& port, std::uint8_t station_id, std::uint16_t channel,
              bool energized, bool verbose) {
  if (channel == 0) {
    std::fprintf(stderr, "[错误] 通道号不能为 0\n");
    return false;
  }

  const std::uint16_t coil_address =
      static_cast<std::uint16_t>(channel - 1U);   // 通道 1 -> 0x0000

  const auto frame =
      build_write_single_coil(station_id, coil_address, energized);

  for (int attempt = 0; attempt < 3; ++attempt) {
    std::vector<std::uint8_t> response;

    if (verbose) {
      std::printf("[发送] %s\n", hex_of(frame).c_str());
    }

    if (port.transact(frame, response, 8) && response.size() >= 8 &&
        response[0] == station_id && response[1] == 0x05 && check_crc(response)) {
      const std::uint16_t echoed_address =
          static_cast<std::uint16_t>(response[2]) << 8 | response[3];
      const std::uint16_t echoed_value =
          static_cast<std::uint16_t>(response[4]) << 8 | response[5];

      if (echoed_address == coil_address &&
          echoed_value == (energized ? 0xFF00U : 0x0000U)) {
        if (verbose) {
          std::printf("[接收] %s\n", hex_of(response).c_str());
        }
        return true;
      }

      std::fprintf(stderr,
                   "[警告] 写应答内容不匹配: addr=0x%04X value=0x%04X\n",
                   echoed_address, echoed_value);
    } else if (verbose) {
      std::printf("[接收] %s\n", response.empty() ? "(无应答)"
                                                  : hex_of(response).c_str());
    }

    if (response.size() >= 3 && response[1] == 0x85) {
      std::fprintf(stderr, "[错误] 继电器返回异常码 0x%02X\n", response[2]);
      return false;
    }

    if (attempt < 2) {
      std::printf("[重试] 通道 %u 写入未确认，第 %d 次重试\n",
                  static_cast<unsigned>(channel), attempt + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(80));
    }
  }

  return false;
}

/**
 * @brief 读线圈状态（FC01），失败返回空
 */
bool read_coils(RelayPort& port, std::uint8_t station_id,
                std::uint16_t first_coil, std::uint16_t count,
                std::vector<bool>& states) {
  states.clear();

  const auto frame = build_read_coils(station_id, first_coil, count);
  std::vector<std::uint8_t> response;

  if (!port.transact(frame, response, 5)) {
    return false;
  }

  if (response.size() < 5 || response[0] != station_id || response[1] != 0x01 ||
      !check_crc(response)) {
    return false;
  }

  const std::uint8_t byte_count = response[2];
  const std::size_t expected = static_cast<std::size_t>(byte_count) + 5U;
  if (response.size() < expected) {
    return false;
  }

  for (std::uint16_t index = 0; index < count; ++index) {
    const std::uint8_t byte = response[3 + (index / 8U)];
    states.push_back(((byte >> (index % 8U)) & 0x01U) != 0);
  }

  return true;
}

void print_coil_states(const char* title, const std::vector<bool>& states) {
  std::printf("%s", title);
  if (states.empty()) {
    std::printf("（读取失败）\n");
    return;
  }

  for (std::size_t index = 0; index < states.size(); ++index) {
    std::printf(" ch%zu=%s", index + 1, states[index] ? "ON" : "off");
  }
  std::printf("\n");
}

/**
 * @brief 读寄存器（FC03 / FC04 通用）
 */
bool read_registers(RelayPort& port, std::uint8_t station_id,
                    std::uint8_t function_code, std::uint16_t first_register,
                    std::uint16_t count, std::vector<std::uint16_t>& values) {
  values.clear();

  const auto frame =
      build_read_registers(station_id, function_code, first_register, count);
  std::vector<std::uint8_t> response;

  if (!port.transact(frame, response, 5)) {
    return false;
  }

  if (response.size() < 5 || response[0] != station_id ||
      response[1] != function_code || !check_crc(response)) {
    return false;
  }

  const std::uint8_t byte_count = response[2];
  if (static_cast<std::size_t>(byte_count) + 5U > response.size()) {
    return false;
  }

  for (std::uint8_t offset = 0; offset + 1 < byte_count; offset += 2) {
    values.push_back(static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(response[3 + offset]) << 8) |
        response[4 + offset]));
  }

  return !values.empty();
}

bool read_holding(RelayPort& port, std::uint8_t station_id,
                  std::uint16_t first_register, std::uint16_t count,
                  std::vector<std::uint16_t>& values) {
  return read_registers(port, station_id, 0x03, first_register, count, values);
}

/* ============================================================
 * PDS3000 开关电源（用来判断推杆是否到底）
 *
 * 推杆是继电器直驱，程序拿不到位置；但推杆电机电流来自开关电源，
 * 机械到底后电机堵转 -> 电流跳升，可以用电流跳变当作“到底”信号。
 * 寄存器表（FC04，起始 0x0000，共 10 个）：
 *   index 0 电压 x0.01V / index 1 电流 x0.01A / index 2 功率 x0.01W
 *   index 3 温度 x0.1C  / index 4 输出模式     / index 5 开关状态
 * ============================================================ */

struct PowerSample {
  bool ok{false};
  double voltage_v{0.0};
  double current_a{0.0};
  double power_w{0.0};
  double temperature_c{0.0};
};

PowerSample read_power_supply(RelayPort& port, std::uint8_t station_id) {
  PowerSample sample;

  std::vector<std::uint16_t> values;

  if (!read_registers(port, station_id, 0x04, 0x0000, 10, values) ||
      values.size() < 4) {
    return sample;
  }

  sample.ok = true;
  sample.voltage_v = static_cast<double>(values[0]) * 0.01;
  sample.current_a = static_cast<double>(values[1]) * 0.01;
  sample.power_w = static_cast<double>(values[2]) * 0.01;
  sample.temperature_c = static_cast<double>(values[3]) * 0.1;

  return sample;
}

const char* brush_fault_name(std::uint16_t raw) {
  switch (raw) {
    case 0:
      return "无故障";
    case 1:
      return "霍尔故障";
    case 2:
      return "过热";
    case 3:
      return "过流";
    case 4:
      return "堵转";
    case 6:
      return "欠压";
    case 7:
      return "过压";
    default:
      return "未知/保留";
  }
}

/**
 * @brief 打印毛刷驱动器关键寄存器（BD-201S，默认站号 0x03）
 *
 * 这是判断“程序问题还是驱动器问题”的关键证据：
 *   - 106 运行时间：非 0 表示驱动器只转这么久就自己停 -> 表现为“按一下动一下”
 *   - 111 当前故障：过流/堵转/过热也会让驱动器自己停
 *   - 100/101/102：确认程序写进去的启停、方向、速度到底是什么
 */
void print_brush_registers(std::uint8_t station_id,
                           std::uint16_t first_register,
                           const std::vector<std::uint16_t>& values) {
  static const char* names[] = {
      "100 启停(1=运行/0=停止)",
      "101 方向(0=CW正转/1=CCW反转)",
      "102 速度(挡位对应转速, rpm)",
      "103 加速",
      "104 减速",
      "105 预留",
      "106 运行时间(单位20ms, 非0=到点自动停)",
      "107 停止后锁定",
      "108 换向延时",
      "109 最大输入功率",
      "110 控制位",
      "111 当前故障",
  };

  std::printf("---- 毛刷驱动器 0x%02X 寄存器 ----\n",
              static_cast<unsigned>(station_id));

  for (std::size_t index = 0; index < values.size(); ++index) {
    const std::uint16_t address =
        static_cast<std::uint16_t>(first_register + index);
    const std::uint16_t value = values[index];

    std::string label = "寄存器 " + std::to_string(address);

    if (address >= 100 && address <= 111) {
      label = names[address - 100];
    }

    std::printf("  %-46s = %u (0x%04X)", label.c_str(),
                static_cast<unsigned>(value), static_cast<unsigned>(value));

    if (address == 111) {
      std::printf("  <- %s", brush_fault_name(value));
    }

    std::printf("\n");
  }

  /* 结论提示 */
  for (std::size_t index = 0; index < values.size(); ++index) {
    const std::uint16_t address =
        static_cast<std::uint16_t>(first_register + index);

    if (address == 106 && values[index] != 0) {
      std::printf(
          "\n[结论提示] 寄存器 106 运行时间 = %u（约 %u ms），"
          "非 0 时驱动器会自己在到点后停机，\n"
          "           这正是“按一下只动一下、不能连续转”的典型原因。"
          "建议写 0（不限制）\n",
          static_cast<unsigned>(values[index]),
          static_cast<unsigned>(values[index]) * 20U);
    }

    if (address == 111 && values[index] != 0) {
      std::printf(
          "\n[结论提示] 当前故障 = %s，驱动器会自我保护停机；"
          "先清故障并降速起转\n",
          brush_fault_name(values[index]));
    }
  }
}

void print_power_sample(std::uint8_t station_id, const PowerSample& sample) {
  if (!sample.ok) {
    std::printf("[电源] 0x%02X  读取失败（端口/站号/接线）\n",
                static_cast<unsigned>(station_id));
    return;
  }

  std::printf("[电源] 0x%02X  %.2f V  %.3f A  %.1f W  %.1f C\n",
              static_cast<unsigned>(station_id), sample.voltage_v,
              sample.current_a, sample.power_w, sample.temperature_c);
}

void usage(const char* program) {
  std::printf(
      "毛刷推杆缩回工具（直接操作继电器，绕过整机程序）\n"
      "\n"
      "用法: %s [选项]\n"
      "\n"
      "默认动作: 缩回（继电器通道 %u 通电 %d ms，然后断电）\n"
      "\n"
      "继电器/动作选项:\n"
      "  --ms <毫秒>        通电时间，默认 %d（= 300mm / 33mm·s⁻¹ 整行程估算）\n"
      "  --stop             只断电，不动作\n"
      "  --extend           反向动作（伸出，通道 %u）\n"
      "  --channel <N>      自定义通电通道（1~4）\n"
      "  --port <设备>      继电器串口，默认 %s\n"
      "  --baud <波特率>    默认 %d\n"
      "  --addr <站号>      继电器站号，默认 0x%02X（可写 0x43 / 67）\n"
      "\n"
      "诊断选项:\n"
      "  --brush-status     只读毛刷驱动器寄存器（106 运行时间 / 111 故障），不动作\n"
      "  --brush-addr <站号> 毛刷驱动器站号，默认 0x%02X（可写 0x03 / 3）\n"
      "  --psu-status       只读电源模块电流电压（配合 --ms 决定观察时长），不动作\n"
      "  --psu-port <设备>  电源串口，默认 %s\n"
      "  --psu-addr <站号>  电源站号：24V=0x%02X，48V=0x72\n"
      "\n"
      "电流判停（解决“怎么知道完全缩回”）:\n"
      "  --current-stop     一边通电一边看电源电流，电流跳升即认为到底，立即断电\n"
      "  --baseline-ms <毫秒> 通电前采基线时长，默认 %d\n"
      "  --delta-a <安>     相对基线跳升多少判为到底，默认 %.1f\n"
      "  --max-a <安>       绝对上限，超过立即断电，默认 %.1f\n"
      "\n"
      "其他:\n"
      "  --force            即使串口被别的进程占用也继续\n"
      "  --quiet            只打印关键信息\n"
      "  -h, --help         帮助\n",
      program, static_cast<unsigned>(kDefaultRetractChannel), kDefaultHoldMs,
      kDefaultHoldMs, static_cast<unsigned>(kDefaultExtendChannel),
      kDefaultPort, kDefaultBaud, static_cast<unsigned>(kDefaultStationId),
      static_cast<unsigned>(kDefaultBrushStationId), kDefaultPsuPort,
      static_cast<unsigned>(kDefaultPsuStationId), kDefaultBaselineMs,
      kDefaultDeltaA, kDefaultMaxA);
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::string port_path = kDefaultPort;
  int baud = kDefaultBaud;
  std::uint8_t station_id = kDefaultStationId;
  int hold_ms = kDefaultHoldMs;
  std::uint16_t retract_channel = kDefaultRetractChannel;
  std::uint16_t extend_channel = kDefaultExtendChannel;
  bool stop_only = false;
  bool extend_action = false;
  bool brush_status_only = false;
  std::uint8_t brush_station_id = kDefaultBrushStationId;
  bool psu_status_only = false;
  bool current_stop = false;
  std::string psu_device = kDefaultPsuPort;
  std::uint8_t psu_station_id = kDefaultPsuStationId;
  int baseline_ms = kDefaultBaselineMs;
  double delta_a = kDefaultDeltaA;
  double max_a = kDefaultMaxA;
  bool force = false;
  bool verbose = true;
  bool channel_overridden = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];

    const auto next_value = [&](const char* name) -> const char* {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "[错误] %s 缺少参数\n", name);
        return nullptr;
      }
      return argv[++index];
    };

    if (argument == "-h" || argument == "--help") {
      usage(argv[0]);
      return 0;
    }

    if (argument == "--ms") {
      const char* value = next_value("--ms");
      if (value == nullptr) {
        return 1;
      }
      hold_ms = std::atoi(value);
      continue;
    }

    if (argument == "--channel") {
      const char* value = next_value("--channel");
      if (value == nullptr) {
        return 1;
      }
      retract_channel =
          static_cast<std::uint16_t>(std::strtoul(value, nullptr, 0));
      channel_overridden = true;
      continue;
    }

    if (argument == "--port") {
      const char* value = next_value("--port");
      if (value == nullptr) {
        return 1;
      }
      port_path = value;
      continue;
    }

    if (argument == "--baud") {
      const char* value = next_value("--baud");
      if (value == nullptr) {
        return 1;
      }
      baud = std::atoi(value);
      continue;
    }

    if (argument == "--addr") {
      const char* value = next_value("--addr");
      if (value == nullptr) {
        return 1;
      }
      station_id = static_cast<std::uint8_t>(std::strtoul(value, nullptr, 0));
      continue;
    }

    if (argument == "--stop") {
      stop_only = true;
      continue;
    }

    if (argument == "--brush-status") {
      brush_status_only = true;
      continue;
    }

    if (argument == "--brush-addr") {
      const char* value = next_value("--brush-addr");
      if (value == nullptr) {
        return 1;
      }
      brush_station_id =
          static_cast<std::uint8_t>(std::strtoul(value, nullptr, 0));
      continue;
    }

    if (argument == "--psu-status") {
      psu_status_only = true;
      continue;
    }

    if (argument == "--psu-port") {
      const char* value = next_value("--psu-port");
      if (value == nullptr) {
        return 1;
      }
      psu_device = value;
      continue;
    }

    if (argument == "--psu-addr") {
      const char* value = next_value("--psu-addr");
      if (value == nullptr) {
        return 1;
      }
      psu_station_id =
          static_cast<std::uint8_t>(std::strtoul(value, nullptr, 0));
      continue;
    }

    if (argument == "--current-stop") {
      current_stop = true;
      continue;
    }

    if (argument == "--baseline-ms") {
      const char* value = next_value("--baseline-ms");
      if (value == nullptr) {
        return 1;
      }
      baseline_ms = std::atoi(value);
      continue;
    }

    if (argument == "--delta-a") {
      const char* value = next_value("--delta-a");
      if (value == nullptr) {
        return 1;
      }
      delta_a = std::atof(value);
      continue;
    }

    if (argument == "--max-a") {
      const char* value = next_value("--max-a");
      if (value == nullptr) {
        return 1;
      }
      max_a = std::atof(value);
      continue;
    }

    if (argument == "--extend") {
      extend_action = true;
      continue;
    }

    if (argument == "--force") {
      force = true;
      continue;
    }

    if (argument == "--quiet") {
      verbose = false;
      continue;
    }

    std::fprintf(stderr, "[错误] 未知参数: %s\n", argument.c_str());
    usage(argv[0]);
    return 1;
  }

  if (hold_ms < 100 || hold_ms > 60000) {
    std::fprintf(stderr, "[错误] --ms 取值范围 100~60000\n");
    return 1;
  }

  if (retract_channel < 1 || retract_channel > 4) {
    std::fprintf(stderr, "[错误] --channel 取值范围 1~4\n");
    return 1;
  }

  const std::uint16_t active_channel =
      channel_overridden
          ? retract_channel
          : (extend_action ? extend_channel : retract_channel);

  /*
   * 四路继电器是两两成对的：
   *   1/2 = 毛刷推杆（伸出/缩回），3/4 = 刮条推杆（伸出/缩回）。
   * 同一对的两个通道绝对不能同时通电。
   */
  const std::uint16_t paired_channel =
      static_cast<std::uint16_t>((active_channel % 2U) == 1U
                                     ? active_channel + 1U
                                     : active_channel - 1U);

  const char* active_name =
      channel_overridden ? "指定通道"
                         : (extend_action ? "毛刷推杆伸出" : "毛刷滚筒缩回");

  std::printf("============================================================\n");
  std::printf("毛刷推杆 / 毛刷驱动器工具\n");
  std::printf("  串口      : %s  %d 8N1\n", port_path.c_str(), baud);
  if (brush_status_only) {
    std::printf("  动作      : 只读毛刷驱动器寄存器（站号 0x%02X）\n",
                static_cast<unsigned>(brush_station_id));
  } else if (psu_status_only) {
    std::printf("  动作      : 只读电源模块 0x%02X（%s，观察 %d ms）\n",
                static_cast<unsigned>(psu_station_id), psu_device.c_str(),
                hold_ms);
  } else {
    std::printf("  站号      : 0x%02X\n", static_cast<unsigned>(station_id));
    std::printf("  动作      : %s\n",
                stop_only ? "只断电（4 路全部断开）" : active_name);
    if (!stop_only) {
      std::printf(
          "  通电通道  : %u（上限 %d ms，同对的通道 %u 会先断电并等待 %d ms）\n",
          static_cast<unsigned>(active_channel), hold_ms,
          static_cast<unsigned>(paired_channel), kDeadTimeMs);
      if (current_stop) {
        std::printf(
            "  电流判停  : 开（%s 0x%02X，基线 %d ms，Δ>%.2f A 或 >%.2f A 即断电）\n",
            psu_device.c_str(), static_cast<unsigned>(psu_station_id),
            baseline_ms, delta_a, max_a);
      } else {
        std::printf(
            "  电流判停  : 关（想按“到底电流跳升”自动停请加 --current-stop）\n");
      }
    }
  }
  std::printf("============================================================\n");

  if (port_held_by_other_process(port_path) && !force) {
    std::fprintf(stderr,
                 "\n[中止] 请先执行:\n"
                 "  sudo systemctl stop snappy-control.service\n"
                 "  sudo systemctl stop snappy-gateway.service\n"
                 "确认没有别的进程占用后重跑；确实要继续可加 --force。\n");
    return 1;
  }

  RelayPort port;
  if (!port.open_port(port_path, baud)) {
    return 1;
  }

  /* ------------------------------------------------------------
   * 附加模式：只读毛刷驱动器寄存器
   *
   * 用来判定“按一下动一下”是程序问题还是驱动器问题：
   *   - 106 运行时间非 0 -> 驱动器自己限时停机（硬件/驱动参数）
   *   - 111 当前故障非 0 -> 过流/堵转等自我保护停机
   *   - 100/101/102     -> 程序实际写进去的启停、方向、速度
   * ------------------------------------------------------------ */
  if (brush_status_only) {
    std::vector<std::uint16_t> values;

    const bool read_ok = read_holding(port, brush_station_id, 100, 12, values);

    port.close_port();

    if (!read_ok) {
      std::fprintf(stderr,
                   "[错误] 读取毛刷驱动器 0x%02X 寄存器 100~111 失败\n"
                   "       检查：站号拨码 / 485 接线 / 是否已停掉整机程序\n",
                   static_cast<unsigned>(brush_station_id));
      return 2;
    }

    print_brush_registers(brush_station_id, 100, values);
    return 0;
  }

  /* ------------------------------------------------------------
   * 附加模式：只读电源模块（用来确认推杆电是哪个电源、电流多大）
   *
   * 用法：保持现场不动，手动让推杆动作，看哪一路电源电流跳起来。
   * ------------------------------------------------------------ */
  if (psu_status_only) {
    RelayPort psu_link;

    if (!psu_link.open_port(psu_device, kDefaultBaud)) {
      port.close_port();
      return 1;
    }

    std::printf("[观察] 每 500 ms 采一次，最多 %d ms，Ctrl+C 退出\n", hold_ms);

    const auto started = std::chrono::steady_clock::now();

    while (!g_stop) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - started)
                               .count();

      print_power_sample(psu_station_id, read_power_supply(psu_link, psu_station_id));

      if (elapsed >= hold_ms) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    psu_link.close_port();
    port.close_port();
    return 0;
  }

  /* ------------------------------------------------------------
   * 第一步：读一次当前继电器输出，确认接线与通道
   * ------------------------------------------------------------ */
  std::vector<bool> states;
  if (read_coils(port, station_id, 0x0000, 4, states)) {
    print_coil_states("[回读] 当前继电器输出:", states);
  } else {
    std::printf("[回读] 读取失败（模块可能不支持 FC01 或总线异常，继续执行）\n");
  }

  bool ok = true;
  bool tripped = false;

  /* ------------------------------------------------------------
   * 电流判停准备：先采一段基线（此时推杆还没通电）
   * ------------------------------------------------------------ */
  RelayPort psu_link;
  bool psu_ready = false;
  double baseline_a = 0.0;

  if (current_stop && !stop_only) {
    if (!psu_link.open_port(psu_device, kDefaultBaud)) {
      std::printf("[警告] 电源模块 %s 打不开，退化为纯计时模式\n",
                  psu_device.c_str());
    } else {
      const auto started = std::chrono::steady_clock::now();
      double sum = 0.0;
      int samples = 0;

      while (!g_stop) {
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started)
                .count();

        if (elapsed >= baseline_ms) {
          break;
        }

        const auto sample = read_power_supply(psu_link, psu_station_id);
        if (sample.ok) {
          sum += sample.current_a;
          ++samples;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      if (samples > 0) {
        baseline_a = sum / static_cast<double>(samples);
        psu_ready = true;

        std::printf("[基线] 电源 0x%02X 静态电流 %.3f A（%d 次采样）\n",
                    static_cast<unsigned>(psu_station_id), baseline_a, samples);
        std::printf("[判停] 通电后电流 > %.3f A 或 > %.2f A 立即断电\n",
                    baseline_a + delta_a, max_a);
      } else {
        std::printf("[警告] 读不到电源电流，退化为纯计时模式\n");
        psu_link.close_port();
      }
    }
  }

  /* ------------------------------------------------------------
   * 第二步：先把同对的两个通道都断电，再按需要给目标通道通电
   *
   * 先断开另一个方向并等待 dead_time，避免两个继电器同时吸合
   * 造成推杆电源短路（整机程序同样遵守 break-before-make）。
   * ------------------------------------------------------------ */
  if (stop_only) {
    for (std::uint16_t channel = 1; channel <= 4; ++channel) {
      if (!set_coil(port, station_id, channel, false, verbose)) {
        std::fprintf(stderr, "[错误] 通道 %u 断电失败\n",
                     static_cast<unsigned>(channel));
        ok = false;
      }
    }
  } else {
    if (!set_coil(port, station_id, paired_channel, false, verbose)) {
      std::fprintf(stderr, "[错误] 通道 %u 断电失败\n",
                   static_cast<unsigned>(paired_channel));
      ok = false;
    }

    if (ok && !set_coil(port, station_id, active_channel, false, false)) {
      std::fprintf(stderr, "[警告] 通道 %u 预断电未确认，继续\n",
                   static_cast<unsigned>(active_channel));
    }

    if (ok) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kDeadTimeMs));

      if (!set_coil(port, station_id, active_channel, true, verbose)) {
        std::fprintf(stderr, "[错误] 通道 %u 通电失败\n",
                     static_cast<unsigned>(active_channel));
        ok = false;
      } else {
        std::printf("[动作] 通道 %u 已通电，开始计时 %d ms\n",
                    static_cast<unsigned>(active_channel), hold_ms);
      }
    }
  }

  /* ------------------------------------------------------------
   * 第三步：保持通电，并按 keep-alive 周期重发
   *
   * ZS-DIO 模块有通信中断看门狗寄存器（0x0030）；
   * 周期重发 ON 可以避免看门狗把输出复位。
   * ------------------------------------------------------------ */
  if (ok && !stop_only) {
    const auto started = std::chrono::steady_clock::now();
    auto next_keep_alive = started + std::chrono::milliseconds(kKeepAliveMs);
    auto next_report = started + std::chrono::milliseconds(1000);
    auto next_psu_sample = started + std::chrono::milliseconds(200);

    double last_current_a = 0.0;

    while (!g_stop) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - started)
                               .count();

      if (elapsed >= hold_ms) {
        std::printf("[计时] 到达 %d ms 上限，断电\n", hold_ms);
        break;
      }

      if (now >= next_keep_alive) {
        if (!set_coil(port, station_id, active_channel, true, false)) {
          std::fprintf(stderr, "[警告] keep-alive 重发失败（已过去 %lld ms）\n",
                       static_cast<long long>(elapsed));
        }
        next_keep_alive = now + std::chrono::milliseconds(kKeepAliveMs);
      }

      /*
       * 电流判停：推杆到底或堵转时电流会明显跳升。
       * 采样比 keep-alive 更密（200 ms），以便尽早断电。
       */
      if (psu_ready && now >= next_psu_sample) {
        const auto sample = read_power_supply(psu_link, psu_station_id);

        if (sample.ok) {
          last_current_a = sample.current_a;
          const double rise = sample.current_a - baseline_a;

          if (sample.current_a > max_a || rise > delta_a) {
            tripped = true;

            std::printf(
                "\n[判停] 电流 %.3f A（基线 %.3f A，Δ %.3f A）"
                "→ 判定已经到底/堵转，立即断电\n",
                sample.current_a, baseline_a, rise);
            break;
          }
        }

        next_psu_sample = now + std::chrono::milliseconds(200);
      }

      if (now >= next_report) {
        if (psu_ready) {
          std::printf("[进行] 已通电 %lld / %d ms，电流 %.3f A\n",
                      static_cast<long long>(elapsed), hold_ms, last_current_a);
        } else {
          std::printf("[进行] 已通电 %lld / %d ms\n",
                      static_cast<long long>(elapsed), hold_ms);
        }
        next_report = now + std::chrono::milliseconds(1000);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (g_stop) {
      std::printf("\n[中断] 收到 Ctrl+C，立即断电\n");
    }
  }

  /* ------------------------------------------------------------
   * 第四步：无论如何都断电（含异常/中断路径）
   *   同一对的两个通道都必须断开，推杆不能带电停留。
   * ------------------------------------------------------------ */
  const bool off_active =
      set_coil(port, station_id, active_channel, false, verbose);
  const bool off_paired =
      set_coil(port, station_id, paired_channel, false, false);

  if (!off_active || !off_paired) {
    std::fprintf(stderr,
                 "[错误] 断电未确认！请立即手动断开推杆电源或重跑 --stop\n");
    ok = false;
  } else {
    std::printf("[完成] 通道 %u / %u 均已断电\n",
                static_cast<unsigned>(active_channel),
                static_cast<unsigned>(paired_channel));
  }

  /* ------------------------------------------------------------
   * 第五步：最终回读
   * ------------------------------------------------------------ */
  if (read_coils(port, station_id, 0x0000, 4, states)) {
    print_coil_states("[回读] 最终继电器输出:", states);
  }

  port.close_port();
  psu_link.close_port();

  if (!ok) {
    return 2;
  }

  if (stop_only) {
    std::printf("[成功] 继电器已全部断电\n");
  } else if (tripped) {
    std::printf("[成功] 电流判停：推杆已到底并断电（无需再计时）\n");
  } else if (current_stop && psu_ready) {
    std::printf(
        "[注意] 到 %d ms 上限时电流仍未跳升，可能行程没走完；\n"
        "       可加大 --ms（或加大 --delta-a 灵敏度）再试一次\n",
        hold_ms);
  } else {
    std::printf("[成功] 按 %d ms 计时完成并断电（未做位置判定）\n", hold_ms);
  }

  return 0;
}
