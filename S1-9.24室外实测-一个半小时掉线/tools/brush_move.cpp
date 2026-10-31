/*
 * @file brush_diag.cpp
 *
 * BD-201S 毛刷驱动器诊断读取工具
 *
 * 用途：
 *   停止 snappy-snappy-control 后，直接通过当前项目使用的
 *   /dev/ttyACM2 + Modbus RTU 读取 BD-201S 的关键寄存器。
 *
 * 当前项目参数：
 *   串口：       /dev/ttyACM2
 *   波特率：     9600
 *   数据位：     8
 *   校验：       N
 *   停止位：     1
 *   Modbus ID：  0x03
 *
 * 只读，不写任何 BD-201S 寄存器。
 *
 * 读取：
 *   200：电机号
 *   109：最大输入功率
 *   102：速度
 *   111：当前故障码
 */

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char* kSerialDevice = "/dev/ttyACM2";
constexpr std::uint8_t kStationId = 0x03;
constexpr speed_t kBaudRate = B9600;
constexpr int kTimeoutMs = 500;

constexpr std::uint16_t kMotorNumberRegister = 200;
constexpr std::uint16_t kMaxInputPowerRegister = 109;
constexpr std::uint16_t kSpeedRegister = 102;
constexpr std::uint16_t kFaultRegister = 111;

std::uint16_t crc16(const std::uint8_t* data, std::size_t len) {
    std::uint16_t crc = 0xFFFF;

    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x0001U) {
                crc = static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

bool write_all(int fd, const std::uint8_t* data, std::size_t len) {
    std::size_t written = 0;

    while (written < len) {
        const ssize_t n = ::write(fd, data + written, len - written);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[ERROR] write failed: "
                      << std::strerror(errno) << "\n";
            return false;
        }

        if (n == 0) {
            return false;
        }

        written += static_cast<std::size_t>(n);
    }

    return true;
}

bool wait_readable(int fd, int timeout_ms) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    const int rc = ::select(fd + 1, &readfds, nullptr, nullptr, &tv);

    if (rc < 0) {
        if (errno == EINTR) {
            return false;
        }

        std::cerr << "[ERROR] select failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    return rc > 0 && FD_ISSET(fd, &readfds);
}

bool read_exact(int fd,
                std::uint8_t* buffer,
                std::size_t length,
                int timeout_ms) {
    std::size_t total = 0;

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(timeout_ms);

    while (total < length) {
        const auto now = std::chrono::steady_clock::now();

        if (now >= deadline) {
            return false;
        }

        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now)
                .count();

        if (!wait_readable(fd, static_cast<int>(remaining))) {
            return false;
        }

        const ssize_t n =
            ::read(fd, buffer + total, length - total);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }

        if (n == 0) {
            return false;
        }

        total += static_cast<std::size_t>(n);
    }

    return true;
}

bool open_serial(int& fd) {
    fd = ::open(kSerialDevice, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd < 0) {
        std::cerr << "[ERROR] 无法打开 " << kSerialDevice
                  << ": " << std::strerror(errno) << "\n";
        return false;
    }

    termios tty{};

    if (::tcgetattr(fd, &tty) != 0) {
        std::cerr << "[ERROR] tcgetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd);
        fd = -1;
        return false;
    }

    ::cfmakeraw(&tty);

    ::cfsetispeed(&tty, kBaudRate);
    ::cfsetospeed(&tty, kBaudRate);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "[ERROR] tcsetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd);
        fd = -1;
        return false;
    }

    ::tcflush(fd, TCIOFLUSH);

    return true;
}

bool read_register(int fd,
                   std::uint16_t address,
                   std::uint16_t& value) {
    // Modbus RTU:
    // ID + function(03) + address_hi + address_lo +
    // count_hi + count_lo + CRC_lo + CRC_hi
    std::uint8_t request[8] = {
        kStationId,
        0x03,
        static_cast<std::uint8_t>((address >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(address & 0xFFU),
        0x00,
        0x01,
        0x00,
        0x00
    };

    const std::uint16_t request_crc =
        crc16(request, 6);

    request[6] =
        static_cast<std::uint8_t>(request_crc & 0xFFU);
    request[7] =
        static_cast<std::uint8_t>((request_crc >> 8U) & 0xFFU);

    ::tcflush(fd, TCIFLUSH);

    if (!write_all(fd, request, sizeof(request))) {
        return false;
    }

    // 正常响应：
    // ID + function + byte_count + data_hi + data_lo + CRC_lo + CRC_hi
    std::uint8_t header[3]{};

    if (!read_exact(fd, header, sizeof(header), kTimeoutMs)) {
        std::cerr << "[ERROR] 读取寄存器 "
                  << address << " 响应头超时\n";
        return false;
    }

    if (header[0] != kStationId) {
        std::cerr << "[ERROR] 寄存器 " << address
                  << " 响应站号错误: 0x"
                  << std::hex << static_cast<int>(header[0])
                  << std::dec << "\n";
        return false;
    }

    if (header[1] & 0x80U) {
        std::uint8_t exception[2]{};

        if (!read_exact(fd, exception, sizeof(exception), kTimeoutMs)) {
            std::cerr << "[ERROR] Modbus 异常响应不完整\n";
            return false;
        }

        std::cerr << "[ERROR] BD-201S Modbus exception: code="
                  << static_cast<unsigned>(exception[0]) << "\n";
        return false;
    }

    if (header[1] != 0x03 || header[2] != 0x02) {
        std::cerr << "[ERROR] 寄存器 " << address
                  << " 响应格式错误: function=0x"
                  << std::hex << static_cast<int>(header[1])
                  << " byte_count=0x"
                  << static_cast<int>(header[2])
                  << std::dec << "\n";
        return false;
    }

    std::uint8_t body[4]{};

    if (!read_exact(fd, body, sizeof(body), kTimeoutMs)) {
        std::cerr << "[ERROR] 读取寄存器 "
                  << address << " 数据超时\n";
        return false;
    }

    std::uint8_t frame[7] = {
        header[0],
        header[1],
        header[2],
        body[0],
        body[1],
        body[2],
        body[3]
    };

    const std::uint16_t calculated =
        crc16(frame, 5);

    const std::uint16_t received =
        static_cast<std::uint16_t>(body[2]) |
        (static_cast<std::uint16_t>(body[3]) << 8U);

    if (calculated != received) {
        std::cerr << "[ERROR] 寄存器 " << address
                  << " CRC 错误: calculated=0x"
                  << std::hex << calculated
                  << " received=0x" << received
                  << std::dec << "\n";
        return false;
    }

    value =
        (static_cast<std::uint16_t>(body[0]) << 8U) |
        static_cast<std::uint16_t>(body[1]);

    return true;
}

const char* fault_name(std::uint16_t fault) {
    switch (fault) {
        case 0: return "无故障";
        case 1: return "霍尔故障";
        case 2: return "过热";
        case 3: return "过流";
        case 4: return "堵转";
        case 5: return "欠压";
        case 6: return "过压";
        default: return "未知/保留";
    }
}

void print_value(const char* name,
                 std::uint16_t address,
                 std::uint16_t value) {
    std::cout << "  " << name
              << " (" << address << ") = "
              << value
              << " (0x"
              << std::hex << std::uppercase
              << std::setw(4) << std::setfill('0')
              << value
              << std::dec << std::nouppercase
              << ")\n";
}

}  // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "BD-201S 毛刷驱动器诊断读取\n";
    std::cout << "串口: /dev/ttyACM2\n";
    std::cout << "波特率: 9600 8N1\n";
    std::cout << "Modbus ID: 0x03\n";
    std::cout << "只读模式：不会修改任何寄存器\n";
    std::cout << "========================================\n\n";

    int fd = -1;

    if (!open_serial(fd)) {
        return 1;
    }

    std::uint16_t motor_number = 0;
    std::uint16_t max_power = 0;
    std::uint16_t speed = 0;
    std::uint16_t fault = 0;

    bool ok = true;

    if (!read_register(fd, kMotorNumberRegister, motor_number)) {
        std::cerr << "[FAIL] 读取 200 电机号失败\n";
        ok = false;
    }

    if (!read_register(fd, kMaxInputPowerRegister, max_power)) {
        std::cerr << "[FAIL] 读取 109 最大输入功率失败\n";
        ok = false;
    }

    if (!read_register(fd, kSpeedRegister, speed)) {
        std::cerr << "[FAIL] 读取 102 速度失败\n";
        ok = false;
    }

    if (!read_register(fd, kFaultRegister, fault)) {
        std::cerr << "[FAIL] 读取 111 故障码失败\n";
        ok = false;
    }

    ::close(fd);

    if (!ok) {
        std::cerr << "\n诊断读取未完整成功。\n";
        std::cerr << "请确认 snappy-snappy-control 已停止，"
                     "且 /dev/ttyACM2 上只有 BD-201S 在响应。\n";
        return 2;
    }

    std::cout << "\n========== 读取结果 ==========\n";

    print_value("200 电机号", kMotorNumberRegister, motor_number);
    print_value("109 最大输入功率(W)", kMaxInputPowerRegister, max_power);
    print_value("102 当前速度", kSpeedRegister, speed);
    print_value("111 当前故障码", kFaultRegister, fault);

    std::cout << "\n========== 结果解释 ==========\n";

    if (fault == 0) {
        std::cout << "111 故障码：无故障\n";
    } else {
        std::cout << "111 故障码："
                  << fault_name(fault) << "\n";
    }

    std::cout << "========================================\n";

    return 0;
}
