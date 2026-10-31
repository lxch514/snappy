/**
 * @file command_application.cpp
 * @brief 本地命令调试入口实现
 *
 * @details
 * - 终端调试命令统一从这里进入。
 * - 执行器命令仍然走 ActuatorService，和 G20 按键 / 屏幕按钮共用同一套业务逻辑。
 * - 传感器命令只读取 SensorService 缓存快照，不直接碰底层 RS485。
 * - 调试模式下允许局部设备启动，未接设备不应阻塞已接设备。
 */

#include "snappy/control/command_application.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "snappy/control/actuator_service.hpp"
#include "snappy/control/control_application.hpp"
#include "snappy/control/sensor_service.hpp"

#if defined(_WIN32)
#include <streambuf>
#else
#include <poll.h>
#include <unistd.h>
#endif

namespace snappy::control {
namespace {

/* ============================================================
 * JSON 输出辅助
 * ============================================================ */
std::string json_escape(const std::string& value) {
  std::string out;
  out.reserve(value.size());

  for (const char ch : value) {
    if (ch == '\\') {
      out += "\\\\";
    } else if (ch == '"') {
      out += "\\\"";
    } else if (ch == '\n') {
      out += "\\n";
    } else if (ch == '\r') {
      out += "\\r";
    } else {
      out += ch;
    }
  }

  return out;
}

std::vector<std::string> split_words(const std::string& line) {
  std::istringstream input(line);
  std::vector<std::string> words;
  std::string word;

  while (input >> word) {
    words.push_back(word);
  }

  return words;
}

bool is_option(const std::string& value) {
  return value.rfind("--", 0) == 0;
}

/* ============================================================
 * 参数解析辅助
 * ============================================================ */
bool parse_positive_ms(const std::string& value,
                       std::chrono::milliseconds* out) {
  try {
    std::size_t consumed = 0;
    const auto parsed = std::stoll(value, &consumed, 0);

    if (consumed != value.size() || parsed <= 0) {
      return false;
    }

    *out = std::chrono::milliseconds(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_u16(const std::string& value,
               std::uint16_t* out) {
  try {
    std::size_t consumed = 0;
    const auto parsed = std::stoul(value, &consumed, 0);

    if (consumed != value.size() ||
        parsed > std::numeric_limits<std::uint16_t>::max()) {
      return false;
    }

    *out = static_cast<std::uint16_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_double(const std::string& value,
                  double* out) {
  try {
    std::size_t consumed = 0;
    const auto parsed = std::stod(value, &consumed);

    if (consumed != value.size()) {
      return false;
    }

    *out = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

drivers::Result invalid_command(const std::string& detail) {
  return drivers::Result::failure(
      drivers::ErrorCode::kInvalidArgument,
      detail,
      "local_command",
      "command_application");
}

const char* quality_name(drivers::SampleQuality quality) noexcept {
  switch (quality) {
    case drivers::SampleQuality::kGood:
      return "good";
    case drivers::SampleQuality::kStale:
      return "stale";
    case drivers::SampleQuality::kInvalid:
      return "invalid";
    case drivers::SampleQuality::kUnknown:
      return "unknown";
  }

  return "unknown";
}

/* ============================================================
 * 非阻塞读取 stdin
 * ============================================================ */
bool read_stdin_line_if_ready(std::string* line) {
#if defined(_WIN32)
  if (std::cin.rdbuf()->in_avail() <= 0) {
    return false;
  }

  return static_cast<bool>(std::getline(std::cin, *line));
#else
  pollfd stdin_fd{};
  stdin_fd.fd = STDIN_FILENO;
  stdin_fd.events = POLLIN;

  const int ready = poll(&stdin_fd, 1, 0);
  if (ready <= 0) {
    return false;
  }

  if ((stdin_fd.revents &
       (POLLERR | POLLHUP | POLLNVAL)) != 0) {
    return false;
  }

  if ((stdin_fd.revents & POLLIN) == 0) {
    return false;
  }

  return static_cast<bool>(std::getline(std::cin, *line));
#endif
}

/* ============================================================
 * 清故障命令别名
 * ============================================================ */
bool is_fault_clear_command(
    const std::vector<std::string>& args) {
  if (args.empty()) {
    return false;
  }

  const std::string& root = args[0];

  if (root == "clear-fault" ||
      root == "clear-faults" ||
      root == "fault-clear" ||
      root == "fault-reset") {
    return true;
  }

  if (args.size() >= 2 &&
      root == "fault" &&
      (args[1] == "clear" ||
       args[1] == "reset")) {
    return true;
  }

  if (args.size() >= 2 &&
      root == "clear" &&
      (args[1] == "fault" ||
       args[1] == "faults")) {
    return true;
  }

  return false;
}

}  // namespace

CommandApplicationOptions parse_command_application_options(
    int argc,
    char** argv) {
  CommandApplicationOptions options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h") {
      options.show_help = true;
    } else if (arg == "--stdin" ||
               arg == "--local-test") {
      options.stdin_enabled = true;
    } else if (arg == "--once") {
      options.sensor_once = true;
    } else if (arg == "--print-sensors") {
      options.print_sensor_json = true;
    } else if (arg == "--config" &&
               i + 1 < argc) {
      options.config_path = argv[++i];
    } else if (arg == "--loop-ms" &&
               i + 1 < argc) {
      std::chrono::milliseconds parsed;

      if (parse_positive_ms(argv[++i], &parsed)) {
        options.loop_period = parsed;
      }
    } else if (arg == "--sensor-ms" &&
               i + 1 < argc) {
      std::chrono::milliseconds parsed;

      if (parse_positive_ms(argv[++i], &parsed)) {
        options.sensor_period = parsed;
      }
    } else if (i == 1 && !is_option(arg)) {
      options.config_path = arg;
    }
  }

  return options;
}

void print_command_application_usage(const char* argv0) {
  std::cout
      << "Usage: " << argv0
      << " [config.json] [--config path] [--stdin] [--once] "
         "[--print-sensors] [--loop-ms 20] [--sensor-ms 100]\n"
      << "stdin commands:\n"
      << "  help\n"
      << "  sensors | cache\n"
      << "  sensor-status\n"
      << "  brush forward [speed_raw]\n"
      << "  brush reverse [speed_raw]\n"
      << "  brush stop\n"
      << "  brush_push extend\n"
      << "  brush_push retract\n"
      << "  brush_push extend_for duration_ms\n"
      << "  brush_push retract_for duration_ms\n"
      << "  brush_push stop\n"
      << "  scraper_push extend\n"
      << "  scraper_push retract\n"
      << "  scraper_push extend_for duration_ms\n"
      << "  scraper_push retract_for duration_ms\n"
      << "  scraper_push stop\n"
      << "  push extend | push retract | push stop\n"
      << "  fan prepare | fan enable | fan disable\n"
      << "  fan left raw | fan right raw | fan both left right | fan stop\n"
      << "  climber ascend speed_rpm\n"
      << "  climber descend speed_rpm\n"
      << "  climber stop\n"
      << "  fault clear | fault reset | clear-fault\n"
      << "  safe-stop | quit\n";
}

CommandApplication::CommandApplication(
    DeviceRegistry& registry,
    CommandApplicationOptions options)
    : registry_(registry), options_(std::move(options)) {}

void CommandApplication::print_status(
    const std::string& state,
    bool ready,
    const std::string& detail) const {
  std::cout
      << "{\"process\":\"snappy-control\","
      << "\"state\":\"" << json_escape(state) << "\","
      << "\"ready_for_arm\":" << (ready ? "true" : "false")
      << ",\"detail\":\"" << json_escape(detail)
      << "\"}\n";
}

void CommandApplication::print_result(
    const std::string& command,
    const drivers::Result& result) const {
  std::cout
      << "{\"type\":\"command_result\","
      << "\"command\":\"" << json_escape(command) << "\","
      << "\"ok\":" << (result.ok ? "true" : "false")
      << ",\"component\":\"" << json_escape(result.component)
      << "\",\"operation\":\"" << json_escape(result.operation)
      << "\",\"detail\":\"" << json_escape(result.detail)
      << "\"}\n";
}

void CommandApplication::print_samples(
    const std::vector<drivers::SensorSample>& samples) const {
  std::cout << "{\"type\":\"sensor_samples\",\"samples\":[";

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto& sample = samples[i];

    if (i != 0) {
      std::cout << ",";
    }

    std::cout
        << "{\"name\":\""
        << json_escape(sample.logical_name)
        << "\",";

    if (sample.value) {
      std::cout << "\"value\":" << *sample.value << ",";
    } else {
      std::cout << "\"value\":null,";
    }

    std::cout
        << "\"unit\":\"" << json_escape(sample.unit) << "\","
        << "\"quality\":\"" << quality_name(sample.quality) << "\","
        << "\"timestamp_ms\":" << sample.timestamp_ms << ","
        << "\"age_ms\":" << sample.age_ms
        << "}";
  }

  std::cout << "]}\n";
}

void CommandApplication::print_sensor_statuses() const {
  const auto statuses = registry_.sensors().device_statuses();

  std::cout << "{\"type\":\"sensor_status\",\"devices\":[";

  for (std::size_t i = 0; i < statuses.size(); ++i) {
    const auto& status = statuses[i];

    if (i != 0) {
      std::cout << ",";
    }

    std::cout
        << "{\"name\":\"" << json_escape(status.logical_name) << "\","
        << "\"online\":" << (status.online ? "true" : "false") << ","
        << "\"consecutive_failures\":" << status.consecutive_failures << ","
        << "\"last_poll_ms\":" << status.last_poll_ms << ","
        << "\"last_success_ms\":" << status.last_success_ms << ","
        << "\"last_failure_ms\":" << status.last_failure_ms << ","
        << "\"last_error\":\"" << json_escape(status.last_error) << "\""
        << "}";
  }

  std::cout << "]}\n";
}

/* ============================================================
 * 本地命令执行
 * ============================================================ */
bool CommandApplication::execute_command(const std::string& line) {
  const auto args = split_words(line);

  if (args.empty()) {
    return true;
  }

  const std::string& root = args[0];

  if (root == "help") {
    print_command_application_usage("snappy-control");
    return true;
  }

  if (root == "quit" || root == "exit") {
    return false;
  }

  if (root == "sensors" || root == "cache") {
    print_samples(registry_.sensors().cached_samples());
    return true;
  }

  if (root == "sensor-status" ||
      root == "sensor_status" ||
      root == "sensors-status") {
    print_sensor_statuses();
    return true;
  }

  if (root == "safe-stop" ||
      root == "stop-all") {
    registry_.safe_stop();

    print_result(
        line,
        drivers::Result::success(
            "all actuators safe-stopped"));
    return true;
  }

  if (is_fault_clear_command(args)) {
    print_result(
        line,
        registry_.actuators().clear_faults());
    return true;
  }

  drivers::Result result =
      invalid_command("unknown local command");

  if (root == "brush") {
    if (args.size() < 2) {
      result = invalid_command(
          "brush command requires action");
    } else if (args[1] == "stop") {
      result = registry_.actuators().brush_stop();
    } else if (args[1] == "forward" ||
               args[1] == "reverse") {
      std::uint16_t speed = 15;

      if (args.size() >= 3 &&
          !parse_u16(args[2], &speed)) {
        result = invalid_command(
            "brush speed_raw must be uint16");
      } else if (args[1] == "forward") {
        result = registry_.actuators().brush_forward(speed);
      } else {
        result = registry_.actuators().brush_reverse(speed);
      }
    } else {
      result = invalid_command(
          "unsupported brush action");
    }
  } else if (root == "brush_push") {
    if (args.size() < 2) {
      result = invalid_command(
          "brush_push command requires action");
    } else if (args[1] == "extend") {
      result = registry_.actuators().brush_push_extend();
    } else if (args[1] == "retract") {
      result = registry_.actuators().brush_push_retract();
    } else if (args[1] == "stop") {
      result = registry_.actuators().brush_push_stop();
    } else if (args[1] == "extend_for" ||
               args[1] == "retract_for") {
      std::chrono::milliseconds duration;

      if (args.size() < 3 ||
          !parse_positive_ms(args[2], &duration)) {
        result = invalid_command(
            "brush_push duration must be positive milliseconds");
      } else if (args[1] == "extend_for") {
        result =
            registry_.actuators().brush_push_extend_for(duration);
      } else {
        result =
            registry_.actuators().brush_push_retract_for(duration);
      }
    } else {
      result = invalid_command(
          "unsupported brush_push action");
    }
  } else if (root == "scraper_push") {
    if (args.size() < 2) {
      result = invalid_command(
          "scraper_push command requires action");
    } else if (args[1] == "extend") {
      result = registry_.actuators().scraper_push_extend();
    } else if (args[1] == "retract") {
      result = registry_.actuators().scraper_push_retract();
    } else if (args[1] == "stop") {
      result = registry_.actuators().scraper_push_stop();
    } else if (args[1] == "extend_for" ||
               args[1] == "retract_for") {
      std::chrono::milliseconds duration;

      if (args.size() < 3 ||
          !parse_positive_ms(args[2], &duration)) {
        result = invalid_command(
            "scraper_push duration must be positive milliseconds");
      } else if (args[1] == "extend_for") {
        result =
            registry_.actuators().scraper_push_extend_for(duration);
      } else {
        result =
            registry_.actuators().scraper_push_retract_for(duration);
      }
    } else {
      result = invalid_command(
          "unsupported scraper_push action");
    }
  } else if (root == "push") {
    if (args.size() < 2) {
      result = invalid_command(
          "push command requires action");
    } else if (args[1] == "extend") {
      result = registry_.actuators().brush_push_extend();
    } else if (args[1] == "retract") {
      result = registry_.actuators().brush_push_retract();
    } else if (args[1] == "stop") {
      result = registry_.actuators().push_stop();
    } else {
      result = invalid_command(
          "unsupported push action");
    }
  } else if (root == "fan") {
    if (args.size() < 2) {
      result = invalid_command(
          "fan command requires action");
    } else if (args[1] == "prepare") {
      result = registry_.actuators().fan_prepare();
    } else if (args[1] == "enable") {
      result = registry_.actuators().fan_enable();
    } else if (args[1] == "disable") {
      result = registry_.actuators().fan_disable();
    } else if (args[1] == "stop") {
      result = registry_.actuators().fan_stop_all();
    } else if (args[1] == "left" ||
               args[1] == "right") {
      std::uint16_t raw = 0;

      if (args.size() < 3 ||
          !parse_u16(args[2], &raw)) {
        result = invalid_command(
            "fan raw throttle must be uint16");
      } else if (args[1] == "left") {
        result = registry_.actuators().fan_left_set(raw);
      } else {
        result = registry_.actuators().fan_right_set(raw);
      }
    } else if (args[1] == "both") {
      std::uint16_t left = 0;
      std::uint16_t right = 0;

      if (args.size() < 4 ||
          !parse_u16(args[2], &left) ||
          !parse_u16(args[3], &right)) {
        result = invalid_command(
            "fan both requires left and right uint16 throttle");
      } else {
        result = registry_.actuators().fan_both_set(left, right);
      }
    } else {
      result = invalid_command(
          "unsupported fan action");
    }
  } else if (root == "climber") {
    if (args.size() < 2) {
      result = invalid_command(
          "climber command requires action");
    } else if (args[1] == "stop") {
      result = registry_.actuators().climber_stop();
    } else if (args[1] == "ascend" ||
               args[1] == "descend") {
      double speed_rpm = 0.0;

      if (args.size() < 3 ||
          !parse_double(args[2], &speed_rpm)) {
        result = invalid_command(
            "climber speed must be rpm");
      } else if (args[1] == "ascend") {
        result =
            registry_.actuators().climber_ascend(speed_rpm);
      } else {
        result =
            registry_.actuators().climber_descend(speed_rpm);
      }
    } else {
      result = invalid_command(
          "unsupported climber action");
    }
  }

  print_result(line, result);
  return true;
}

/* ============================================================
 * 本地调试主循环
 * ============================================================ */
int CommandApplication::run(std::atomic_bool& running) {
  const auto& bringup = registry_.config().bringup;

  ControlApplicationOptions control_options;
  control_options.allow_partial_devices =
      bringup.allow_partial_devices ||
      bringup.ignore_device_startup_failures ||
      bringup.quarantine_failed_devices;

  control_options.arm_requires_all_required_devices =
      bringup.arm_requires_all_required_devices;

  ControlApplication control(
      registry_.hardware_ports(),
      control_options);

  const auto startup = control.start();

  if (!startup.started) {
    registry_.safe_stop();
    print_status("safe_hold", false, startup.detail);
    return 1;
  }

  const auto actuators = registry_.actuators().start();

  if (!actuators.ok &&
      !control_options.allow_partial_devices) {
    control.enter_safe_hold(actuators.detail);
    registry_.safe_stop();

    print_status(
        "safe_hold",
        false,
        actuators.detail);

    return 1;
  }

  std::string start_detail = startup.detail;
  if (!actuators.ok) {
    start_detail += "; actuator startup degraded: ";
    start_detail += actuators.detail;
  }

  const bool ready_for_local_command =
      startup.ready_for_arm &&
      (actuators.ok || control_options.allow_partial_devices);

  print_status(
      startup.partial_start ? "degraded" : "disarmed",
      ready_for_local_command,
      start_detail);

  if (options_.sensor_once) {
    std::this_thread::sleep_for(options_.sensor_period);

    print_samples(
        registry_.sensors().cached_samples());

    control.stop();
    registry_.stop();
    return 0;
  }

  auto next_sensor_print =
      std::chrono::steady_clock::now();

  std::string last_tick_error;

  while (running.load() && !quit_requested_) {
    const auto now =
        std::chrono::steady_clock::now();

    const auto tick =
        registry_.actuators().tick(now);

    if (!tick.ok) {
      if (!control_options.allow_partial_devices) {
        control.enter_safe_hold(tick.detail);
        registry_.safe_stop();

        print_status(
            "safe_hold",
            false,
            tick.detail);

        return 1;
      }

      if (tick.detail != last_tick_error) {
        last_tick_error = tick.detail;

        print_status(
            "degraded",
            ready_for_local_command,
            tick.detail);
      }
    }

    if (options_.print_sensor_json &&
        now >= next_sensor_print) {
      print_samples(
          registry_.sensors().cached_samples());

      next_sensor_print =
          now + options_.sensor_period;
    }

    if (options_.stdin_enabled) {
      std::string line;

      while (read_stdin_line_if_ready(&line)) {
        if (!execute_command(line)) {
          quit_requested_ = true;
          break;
        }
      }
    }

    std::this_thread::sleep_for(
        options_.loop_period);
  }

  control.stop();
  registry_.stop();

  print_status(
      "stopped",
      false,
      "snappy-control stopped");

  return 0;
}

}  // namespace snappy::