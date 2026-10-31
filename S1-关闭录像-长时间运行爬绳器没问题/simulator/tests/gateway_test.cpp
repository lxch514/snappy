/**
 * @file gateway_test.cpp
 * @brief G20 遥控映射、可靠传输和安全恢复测试
 *
 * @details
 * - 验证毛刷、风机、爬绳器、推杆和摄像头映射。
 * - 验证暂停键进入安全停止状态。
 * - 验证暂停后只点击一次屏幕控制权即可恢复。
 * - 验证 mapper 自己产生的 acquire 不能绕过安全恢复。
 * - 验证可靠舵量包 ACK 和重复包去重。
 * - 验证操作记录写入日志文件。
 */

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "snappy/drivers/diagnostics/driver_log.hpp"
#include "snappy/gateway/g20/g20_command_router.hpp"
#include "snappy/gateway/g20/g20_legacy_packet_adapter.hpp"
#include "snappy/gateway/interfaces.hpp"
#include "snappy/gateway/unconfigured_transports.hpp"

namespace {

constexpr int kCenter = 1500;
constexpr int kLow = 1050;
constexpr int kHigh = 1950;

constexpr int kRightStickX = 0;
constexpr int kRightStickY = 1;
constexpr int kLeftStickY = 2;
constexpr int kLeftStickX = 3;
constexpr int kGs = 4;
constexpr int kL2 = 6;
constexpr int kCamera = 7;
constexpr int kL1 = 10;
constexpr int kSpay = 11;
constexpr int kPause = 12;
constexpr int kWheel = 13;
constexpr int kR1 = 14;
constexpr int kR2 = 15;

/* ============================================================
 * 通用测试辅助函数
 * ============================================================ */

bool expect(
    bool condition,
    const std::string& message) {
  if (condition) {
    return true;
  }

  std::cerr << message << '\n';
  return false;
}

nlohmann::json packet(
    const std::vector<int>& channels) {
  return {
      {"schema_version", 1},
      {"kind", "g20_channels"},
      {"robot_id", "snappy-001"},
      {"source_id", "g20"},
      {"session_id", "mapper-test"},
      {"channels", channels},
  };
}

nlohmann::json reliable_packet(
    const std::vector<int>& channels,
    std::uint64_t sequence) {
  return {
      {"schema_version", 1},
      {"kind", "g20_channels"},
      {"robot_id", "snappy-001"},
      {"source_id", "g20-test"},
      {"session_id", "reliable-session"},
      {"message_id",
       "g20-channels-" +
           std::to_string(sequence)},
      {"seq", sequence},
      {"timestamp_ms", sequence * 10},
      {"device", "test-g20"},
      {"channels", channels},
      {"reliable", true},
  };
}

bool has_action(
    const std::vector<nlohmann::json>& commands,
    const std::string& action) {
  for (const auto& command : commands) {
    if (command.value("action", "") ==
        action) {
      return true;
    }
  }

  return false;
}

nlohmann::json find_action(
    const std::vector<nlohmann::json>& commands,
    const std::string& action) {
  for (const auto& command : commands) {
    if (command.value("action", "") ==
        action) {
      return command;
    }
  }

  return nlohmann::json::object();
}

bool param_is(
    const nlohmann::json& command,
    const std::string& key,
    const std::string& expected) {
  if (!command.contains("params") ||
      !command.at("params").is_object()) {
    return false;
  }

  return command.at("params").value(
             key,
             "") == expected;
}

std::vector<nlohmann::json> set_channel(
    snappy::gateway::g20::
        G20LegacyPacketAdapter& adapter,
    std::vector<int>& channels,
    int index,
    int value) {
  channels[
      static_cast<std::size_t>(index)] =
      value;

  return adapter.adapt(
      packet(channels));
}

void center_channel(
    snappy::gateway::g20::
        G20LegacyPacketAdapter& adapter,
    std::vector<int>& channels,
    int index) {
  channels[
      static_cast<std::size_t>(index)] =
      kCenter;

  static_cast<void>(
      adapter.adapt(
          packet(channels)));
}

std::vector<nlohmann::json> pulse_high(
    snappy::gateway::g20::
        G20LegacyPacketAdapter& adapter,
    std::vector<int>& channels,
    int index) {
  auto commands =
      set_channel(
          adapter,
          channels,
          index,
          kHigh);

  center_channel(
      adapter,
      channels,
      index);

  return commands;
}

void center_all(
    snappy::gateway::g20::
        G20LegacyPacketAdapter& adapter,
    std::vector<int>& channels) {
  channels.assign(
      16,
      kCenter);

  static_cast<void>(
      adapter.adapt(
          packet(channels)));
}

/* ============================================================
 * Router 可靠传输测试对象
 * ============================================================ */

class TestControlChannel final
    : public snappy::gateway::IControlChannel {
 public:
  std::optional<std::string> request(
      const std::string& command_json)
      override {
    const auto command =
        nlohmann::json::parse(
            command_json);

    requests.push_back(command);

    nlohmann::json ack;
    ack["schema_version"] = 1;
    ack["kind"] = "ack";
    ack["robot_id"] =
        command.value(
            "robot_id",
            "snappy-001");
    ack["message_id"] =
        command.value(
            "message_id",
            "");
    ack["seq"] =
        command.value("seq", 0);
    ack["status"] = "SUCCEEDED";
    ack["detail"] =
        "test control acknowledged";
    ack["lease_id"] = "test-lease";
    ack["control_epoch"] = 1;
    ack["ack_time_ms"] = 1;

    return ack.dump();
  }

  void notify_link_down(
      snappy::control::LinkId)
      noexcept override {
  }

  std::size_t action_count(
      const std::string& action) const {
    std::size_t count = 0;

    for (const auto& request : requests) {
      if (request.value("action", "") ==
          action) {
        ++count;
      }
    }

    return count;
  }

  std::vector<nlohmann::json> requests;
};

class TestTransport final
    : public snappy::gateway::ITransport {
 public:
  snappy::control::LinkId
  link_id() const noexcept override {
    return snappy::control::LinkId::kG20Udp;
  }

  std::size_t
  payload_limit() const noexcept override {
    return snappy::gateway::kUdpPayloadLimit;
  }

  bool healthy() const noexcept override {
    return true;
  }

  std::optional<
      snappy::gateway::InboundMessage>
  receive() override {
    return std::nullopt;
  }

  bool publish(
      const snappy::gateway::
          PublishedMessage& message)
      override {
    published.push_back(message);
    return true;
  }

  std::size_t ack_count(
      const std::string& message_id) const {
    std::size_t count = 0;

    for (const auto& message : published) {
      try {
        const auto root =
            nlohmann::json::parse(
                message.payload);

        if (root.value("kind", "") ==
                "ack" &&
            root.value("message_id", "") ==
                message_id) {
          ++count;
        }
      } catch (...) {
      }
    }

    return count;
  }

  std::vector<
      snappy::gateway::PublishedMessage>
      published;
};

/* ============================================================
 * 基础按键映射测试
 * ============================================================ */

bool test_basic_mapping() {
  snappy::gateway::g20::
      G20LegacyPacketAdapter adapter;

  std::vector<int> channels(
      16,
      kCenter);

  auto commands =
      adapter.adapt(
          packet(channels));

  if (!expect(
          has_action(
              commands,
              "control.acquire"),
          "initial acquire was not emitted")) {
    return false;
  }

  if (!expect(
          has_action(
              commands,
              "control.arm"),
          "initial arm was not emitted")) {
    return false;
  }

  if (!expect(
          !has_action(
              commands,
              "brush.forward") &&
              !has_action(
                  commands,
                  "fan.both_set") &&
              !has_action(
                  commands,
                  "climber.ascend"),
          "initial packet started an actuator")) {
    return false;
  }

  /* ----------------------------------------------------------
   * 毛刷
   * ---------------------------------------------------------- */

  commands =
      pulse_high(
          adapter,
          channels,
          kL1);

  auto action =
      find_action(
          commands,
          "brush.forward");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "speed_raw",
                  "15"),
          "L1 did not start brush forward at 15")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kL1);

  if (!expect(
          has_action(
              commands,
              "brush.stop"),
          "second L1 did not stop brush")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kL2);

  action =
      find_action(
          commands,
          "brush.reverse");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "speed_raw",
                  "15"),
          "L2 did not start brush reverse at 15")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kL2);

  if (!expect(
          has_action(
              commands,
              "brush.stop"),
          "second L2 did not stop brush")) {
    return false;
  }

  /* ----------------------------------------------------------
   * 推进风机
   * ---------------------------------------------------------- */

  center_all(
      adapter,
      channels);

  commands =
      set_channel(
          adapter,
          channels,
          kRightStickY,
          kLow);

  action =
      find_action(
          commands,
          "fan.both_set");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "left_raw",
                  "100") &&
              param_is(
                  action,
                  "right_raw",
                  "100"),
          "fan did not start at 100")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kRightStickY);

  commands =
      set_channel(
          adapter,
          channels,
          kRightStickX,
          kHigh);

  action =
      find_action(
          commands,
          "fan.both_set");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "left_raw",
                  "500") &&
              param_is(
                  action,
                  "right_raw",
                  "500"),
          "fan did not increase to 500")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kRightStickX);

  commands =
      set_channel(
          adapter,
          channels,
          kRightStickX,
          kLow);

  action =
      find_action(
          commands,
          "fan.both_set");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "left_raw",
                  "100") &&
              param_is(
                  action,
                  "right_raw",
                  "100"),
          "fan did not decrease to 100")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kRightStickX);

  commands =
      set_channel(
          adapter,
          channels,
          kRightStickY,
          kHigh);

  if (!expect(
          has_action(
              commands,
              "fan.stop"),
          "fan stop was not emitted")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kRightStickY);

  commands =
      set_channel(
          adapter,
          channels,
          kRightStickX,
          kHigh);

  if (!expect(
          !has_action(
              commands,
              "fan.both_set"),
          "fan adjustment remained active after stop")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kRightStickX);

  /* ----------------------------------------------------------
   * 爬绳器
   * ---------------------------------------------------------- */

  center_all(
      adapter,
      channels);

  commands =
      pulse_high(
          adapter,
          channels,
          kR1);

  action =
      find_action(
          commands,
          "climber.ascend");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "speed_rpm",
                  "500"),
          "R1 did not start ascend at 500")) {
    return false;
  }

  commands =
      set_channel(
          adapter,
          channels,
          kWheel,
          kHigh);

  action =
      find_action(
          commands,
          "climber.ascend");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "speed_rpm",
                  "600"),
          "wheel did not increase climber to 600")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kWheel);

  commands =
      set_channel(
          adapter,
          channels,
          kWheel,
          kLow);

  action =
      find_action(
          commands,
          "climber.ascend");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "speed_rpm",
                  "500"),
          "wheel did not decrease climber to 500")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kWheel);

  commands =
      pulse_high(
          adapter,
          channels,
          kR1);

  if (!expect(
          has_action(
              commands,
              "climber.stop"),
          "second R1 did not stop climber")) {
    return false;
  }

  center_all(
      adapter,
      channels);

  commands =
      pulse_high(
          adapter,
          channels,
          kR2);

  action =
      find_action(
          commands,
          "climber.descend");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "speed_rpm",
                  "500"),
          "R2 did not start descend at 500")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kR2);

  if (!expect(
          has_action(
              commands,
              "climber.stop"),
          "second R2 did not stop climber")) {
    return false;
  }

  /* ----------------------------------------------------------
   * GS 和推杆
   * ---------------------------------------------------------- */

  center_all(
      adapter,
      channels);

  commands =
      set_channel(
          adapter,
          channels,
          kLeftStickY,
          kHigh);

  if (!expect(
          commands.empty(),
          "push moved while GS was centered")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kLeftStickY);

  static_cast<void>(
      set_channel(
          adapter,
          channels,
          kGs,
          kHigh));

  commands =
      set_channel(
          adapter,
          channels,
          kLeftStickY,
          kHigh);

  if (!expect(
          commands.empty(),
          "first scraper up did not enter setup")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kLeftStickY);

  for (int index = 0;
       index < 3;
       ++index) {
    commands =
        set_channel(
            adapter,
            channels,
            kLeftStickX,
            kHigh);

    if (!expect(
            commands.empty(),
            "scraper time adjustment emitted command")) {
      return false;
    }

    center_channel(
        adapter,
        channels,
        kLeftStickX);
  }

  commands =
      set_channel(
          adapter,
          channels,
          kLeftStickY,
          kHigh);

  action =
      find_action(
          commands,
          "scraper_push.extend_timed");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "duration_ms",
                  "3000"),
          "scraper did not extend for 3000ms")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kLeftStickY);

  center_all(
      adapter,
      channels);

  static_cast<void>(
      set_channel(
          adapter,
          channels,
          kGs,
          kLow));

  commands =
      set_channel(
          adapter,
          channels,
          kLeftStickY,
          kLow);

  if (!expect(
          commands.empty(),
          "first brush push down did not enter setup")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kLeftStickY);

  for (int index = 0;
       index < 4;
       ++index) {
    static_cast<void>(
        set_channel(
            adapter,
            channels,
            kLeftStickX,
            kHigh));

    center_channel(
        adapter,
        channels,
        kLeftStickX);
  }

  static_cast<void>(
      set_channel(
          adapter,
          channels,
          kLeftStickX,
          kLow));

  center_channel(
      adapter,
      channels,
      kLeftStickX);

  commands =
      set_channel(
          adapter,
          channels,
          kLeftStickY,
          kLow);

  action =
      find_action(
          commands,
          "brush_push.retract_timed");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "duration_ms",
                  "3000"),
          "brush push did not retract for 3000ms")) {
    return false;
  }

  center_channel(
      adapter,
      channels,
      kLeftStickY);

  /* ----------------------------------------------------------
   * 摄像头
   * ---------------------------------------------------------- */

  center_all(
      adapter,
      channels);

  commands =
      pulse_high(
          adapter,
          channels,
          kCamera);

  action =
      find_action(
          commands,
          "video.start");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "active_camera",
                  "1"),
          "camera button did not start camera 1")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kSpay);

  action =
      find_action(
          commands,
          "video.switch_camera");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "active_camera",
                  "2"),
          "SPAY did not switch to camera 2")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kSpay);

  action =
      find_action(
          commands,
          "video.switch_camera");

  if (!expect(
          !action.empty() &&
              param_is(
                  action,
                  "active_camera",
                  "1"),
          "SPAY did not switch back to camera 1")) {
    return false;
  }

  commands =
      pulse_high(
          adapter,
          channels,
          kCamera);

  if (!expect(
          has_action(
              commands,
              "video.stop"),
          "second camera press did not stop video")) {
    return false;
  }

  return true;
}

/* ============================================================
 * 暂停后单次控制权恢复测试
 * ============================================================ */

bool test_single_click_recovery() {
  snappy::gateway::g20::
      G20LegacyPacketAdapter adapter;

  std::vector<int> channels(
      16,
      kCenter);

  static_cast<void>(
      adapter.adapt(
          packet(channels)));

  /*
   * 按下暂停键进入安全停止。
   */
  auto commands =
      pulse_high(
          adapter,
          channels,
          kPause);

  if (!expect(
          has_action(
              commands,
              "control.safe_hold"),
          "pause did not emit safe hold")) {
    return false;
  }

  /*
   * 安全停止后，遥控动作不能自动恢复。
   */
  commands =
      pulse_high(
          adapter,
          channels,
          kL1);

  if (!expect(
          !has_action(
              commands,
              "control.acquire") &&
              !has_action(
                  commands,
                  "control.arm") &&
              !has_action(
                  commands,
                  "brush.forward"),
          "remote bypassed screen recovery")) {
    return false;
  }

  /*
   * mapper 自己产生的 acquire 不能解除恢复锁。
   */
  adapter.sync_external_command(
    {
        {"action", "control.acquire"},
        {
            "params",
            {
                {"origin", "g20_mapper"},
            },
        },
    });

  commands =
      pulse_high(
          adapter,
          channels,
          kL1);

  if (!expect(
          !has_action(
              commands,
              "control.arm") &&
              !has_action(
                  commands,
                  "brush.forward"),
          "mapper acquire bypassed safety recovery")) {
    return false;
  }

  /*
   * 模拟用户只点击一次屏幕控制权按钮。
   * 屏幕 acquire 没有 origin=g20_mapper。
   */
  adapter.sync_external_command(
    {
        {"action", "control.acquire"},
    });

  /*
   * 即使 Android 的自动 arm ACK 暂时没有同步回来，
   * 下一次遥控动作也必须先补发 arm 并执行动作，
   * 不得要求第二次点击控制权。
   */
  commands =
      pulse_high(
          adapter,
          channels,
          kL1);

  if (!expect(
          !has_action(
              commands,
              "control.acquire"),
          "single screen acquire requested acquire again")) {
    return false;
  }

  if (!expect(
          has_action(
              commands,
              "control.arm"),
          "single screen acquire did not allow arm recovery")) {
    return false;
  }

  if (!expect(
          has_action(
              commands,
              "brush.forward"),
          "single screen acquire did not restore operation")) {
    return false;
  }

  return true;
}

/* ============================================================
 * 可靠舵量 ACK 和重复包测试
 * ============================================================ */

bool test_reliable_delivery() {
  TestControlChannel control;
  TestTransport transport;

  snappy::gateway::
      UnconfiguredG20VideoAdapter video;

  snappy::gateway::g20::
      G20CommandRouter router(
          "snappy-001",
          control,
          &video);

  std::vector<int> channels(
      16,
      kCenter);

  const auto initial =
      reliable_packet(
          channels,
          100);

  const auto first =
      router.route(
          {
              "snappy/v1/g20/down/command",
              initial.dump(),
          },
          transport);

  if (!expect(
          first.processed,
          "first reliable packet was not processed")) {
    return false;
  }

  if (!expect(
          transport.ack_count(
              "g20-channels-100") == 1,
          "first reliable ACK was not published")) {
    return false;
  }

  const std::size_t requests_after_first =
      control.requests.size();

  const auto duplicate =
      router.route(
          {
              "snappy/v1/g20/down/command",
              initial.dump(),
          },
          transport);

  if (!expect(
          duplicate.processed &&
              !duplicate.forwarded,
          "duplicate reliable packet was not ignored")) {
    return false;
  }

  if (!expect(
          control.requests.size() ==
              requests_after_first,
          "duplicate packet repeated control request")) {
    return false;
  }

  if (!expect(
          transport.ack_count(
              "g20-channels-100") == 2,
          "duplicate ACK was not resent")) {
    return false;
  }

  channels[
      static_cast<std::size_t>(kR1)] =
      kHigh;

  const auto r1 =
      reliable_packet(
          channels,
          101);

  router.route(
      {
          "snappy/v1/g20/down/command",
          r1.dump(),
      },
      transport);

  if (!expect(
          control.action_count(
              "climber.ascend") == 1,
          "reliable R1 event was not executed")) {
    return false;
  }

  router.route(
      {
          "snappy/v1/g20/down/command",
          r1.dump(),
      },
      transport);

  if (!expect(
          control.action_count(
              "climber.ascend") == 1,
          "R1 retry executed climber twice")) {
    return false;
  }

  if (!expect(
          transport.ack_count(
              "g20-channels-101") == 2,
          "R1 duplicate ACK was not resent")) {
    return false;
  }

  return true;
}

/* ============================================================
 * 日志文件测试
 * ============================================================ */

bool test_log_file(
    const std::filesystem::path& log_root) {
  std::string log_text;

  if (std::filesystem::exists(log_root)) {
    for (
        const auto& entry :
        std::filesystem::
            recursive_directory_iterator(
                log_root)) {
      if (!entry.is_regular_file() ||
          entry.path().extension() !=
              ".log") {
        continue;
      }

      std::ifstream file(
          entry.path());

      log_text.append(
          std::istreambuf_iterator<char>(
              file),
          std::istreambuf_iterator<char>());
    }
  }

  return expect(
      !log_text.empty() &&
          log_text.find(
              "[logger] === 日志系统初始化完成 ===") !=
              std::string::npos &&
          log_text.find(
              "[gateway.g20.input]") !=
              std::string::npos &&
          log_text.find(
              "[gateway.g20.mapping]") !=
              std::string::npos &&
          log_text.find(
              "action=brush.forward") !=
              std::string::npos &&
          log_text.find(
              "speed_raw=15") !=
              std::string::npos &&
          log_text.find(
              "[gateway.g20.delivery]") !=
              std::string::npos &&
          log_text.find(
              "event=duplicate") !=
              std::string::npos,
      "log file did not contain required records");
}

}  // namespace

int main() {
  const auto log_root =
      std::filesystem::temp_directory_path() /
      "snappy-gateway-test-log";

  std::error_code filesystem_error;

  std::filesystem::remove_all(
      log_root,
      filesystem_error);

  snappy::drivers::diagnostics::
      LogConfig log_config;

  log_config.process_name =
      "snappy-gateway-test";

  log_config.log_dir =
      log_root.string();

  log_config.console = false;
  log_config.debug = true;

  snappy::drivers::diagnostics::configure(
      std::move(log_config));

  snappy::gateway::
      UnconfiguredG20VideoAdapter video;

  if (!expect(
          !video.available() &&
              !video.start_g20_stream(),
          "unconfigured video unexpectedly available")) {
    return 1;
  }

  if (!test_basic_mapping()) {
    return 1;
  }

  if (!test_single_click_recovery()) {
    return 1;
  }

  if (!test_reliable_delivery()) {
    return 1;
  }

  if (!test_log_file(log_root)) {
    return 1;
  }

  std::cout
      << "gateway mapper, single-click recovery "
         "and reliable delivery tests passed\n";

  return 0;
}