#pragma once

/**
 * @file g20_button_mapper.hpp
 * @brief G20 遥控通道到整机命令的映射状态机
 */

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace snappy::gateway::g20 {

struct LegacyRemoteCommand {
  std::string action;
  std::map<std::string, std::string> params;
};

class G20ButtonMapper {
 public:
  G20ButtonMapper();

  std::vector<LegacyRemoteCommand> update(
      const std::vector<int>& channels,
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now());

  std::vector<LegacyRemoteCommand> tick(
      std::chrono::steady_clock::time_point now =
          std::chrono::steady_clock::now());

  void syncExternalCommand(
      const std::string& action,
      const std::map<std::string, std::string>& params = {});

  void reset();

 private:
  enum class BrushMode {
    kStopped,
    kForward,
    kReverse,
  };

  enum class ClimberMode {
    kStopped,
    kAscend,
    kDescend,
  };

  enum class PushMode {
    kIdle,
    kExtendSetup,
    kRetractSetup,
    kRetractRecovery,
    kExtendRecovery,
  };

  enum class PushTarget {
    kNone,
    kBrush,
    kScraper,
  };

  static constexpr int kChannelCount = 16;

  static constexpr int kRightStickX = 0;
  static constexpr int kRightStickY = 1;
  static constexpr int kLeftStickY = 2;
  static constexpr int kLeftStickX = 3;
  static constexpr int kGs = 4;
  static constexpr int kL2 = 6;
  static constexpr int kCamera = 7;
  static constexpr int kL1 = 10;
  static constexpr int kSpay = 11;
  static constexpr int kPause = 12;
  static constexpr int kWheel = 13;
  static constexpr int kR1 = 14;
  static constexpr int kR2 = 15;

  static constexpr int kLowThreshold = 1300;
  static constexpr int kHighThreshold = 1700;

  static constexpr int kFanBaseRaw = 100;
  static constexpr int kFanStepRaw = 400;
  static constexpr int kFanMaxRaw = 8191;

  static constexpr int kBrushMaxRaw = 15;

  static constexpr int kClimberInitialRpm = 500;
  static constexpr int kClimberStepRpm = 100;
  static constexpr int kClimberMinRpm = 500;
  static constexpr int kClimberMaxRpm = 3000;

  static constexpr int kPushInitialSeconds = 0;
  static constexpr int kPushMinSeconds = 0;
  static constexpr int kBrushPushMaxSeconds = 10;
  static constexpr int kScraperPushMaxSeconds = 10;
  static constexpr double kBrushPushSpeedMmPerSecond = 33.0;
  static constexpr double kScraperPushSpeedMmPerSecond = 30.0;

  static constexpr std::chrono::milliseconds kRepeatInterval{500};

  /*
   * 左摇杆长推进入恢复动作前需要持续推住的时长。
   * 先按住 1 秒，之后才开始持续伸出/缩回，避免误触发。
   */
  static constexpr std::chrono::milliseconds kPushRecoveryDelay{1000};

  static constexpr std::chrono::milliseconds kHeartbeatInterval{250};

  bool initialized_{false};
  bool lease_active_{false};
  bool control_armed_{false};
  bool screen_recovery_required_{false};

  std::vector<int> last_channels_;

  int prev_l1_level_{0};
  int prev_l2_level_{0};
  int prev_r1_level_{0};
  int prev_r2_level_{0};
  int prev_camera_level_{0};
  int prev_spay_level_{0};
  int prev_pause_level_{0};

  int gs_level_{0};
  int fan_y_level_{0};
  int fan_x_level_{0};
  int wheel_level_{0};
  int push_y_level_{0};
  int push_x_level_{0};

  BrushMode brush_mode_{BrushMode::kStopped};

  bool fan_active_{false};
  int fan_speed_raw_{kFanBaseRaw};

  ClimberMode climber_mode_{ClimberMode::kStopped};
  int climber_speed_rpm_{kClimberInitialRpm};

  bool video_on_{false};
  int active_camera_{1};

  PushTarget push_target_{PushTarget::kNone};
  PushMode push_mode_{PushMode::kIdle};
  int push_seconds_{kPushInitialSeconds};

  std::chrono::steady_clock::time_point last_fan_adjust_{};
  std::chrono::steady_clock::time_point last_climber_adjust_{};
  std::chrono::steady_clock::time_point last_push_adjust_{};

  /*
   * 长推恢复的按下计时起点。
   * 非零表示左摇杆正被持续推住，正在等待进入恢复动作。
   */
  std::chrono::steady_clock::time_point push_recovery_started_{};
  std::chrono::steady_clock::time_point push_extend_recovery_started_{};

  std::chrono::steady_clock::time_point last_heartbeat_{};

  static int levelAt(
      const std::vector<int>& channels,
      int index);

  static bool buttonChanged(
      int current_level,
      int& previous_level);

  void ensureControl(
      std::vector<LegacyRemoteCommand>& out);

  void stopLocalMotionState();

  void handleGs(int level);
  int currentPushMaxSeconds() const;
  int currentPushDistanceMm() const;
  int currentPushDurationMs() const;

  void handleBrushForward(
      std::vector<LegacyRemoteCommand>& out);

  void handleBrushReverse(
      std::vector<LegacyRemoteCommand>& out);

  void handleFanY(
      int level,
      std::vector<LegacyRemoteCommand>& out);

  void handleFanX(
      int level,
      std::chrono::steady_clock::time_point now,
      std::vector<LegacyRemoteCommand>& out,
      bool edge_only);

  void adjustFan(
      int direction,
      std::vector<LegacyRemoteCommand>& out);

  void handleClimberButton(
      ClimberMode target,
      std::vector<LegacyRemoteCommand>& out);

  void handleWheel(
      int level,
      std::chrono::steady_clock::time_point now,
      std::vector<LegacyRemoteCommand>& out,
      bool edge_only);

  void adjustClimber(
      int direction,
      std::vector<LegacyRemoteCommand>& out);

  void handlePushY(
      int level,
      std::chrono::steady_clock::time_point now,
      std::vector<LegacyRemoteCommand>& out);

  void handlePushX(
      int level,
      std::chrono::steady_clock::time_point now,
      std::vector<LegacyRemoteCommand>& out,
      bool edge_only);

  void adjustPushSeconds(int direction);

  /*
   * 左摇杆长推恢复。
   * 按住达到 kPushRecoveryDelay 后启动持续伸出/缩回；
   * 摇杆回正立即发送 stop。
   */
  void handlePushRecovery(
      std::chrono::steady_clock::time_point now,
      std::vector<LegacyRemoteCommand>& out);

  void handlePushExtendRecovery(
      std::chrono::steady_clock::time_point now,
      std::vector<LegacyRemoteCommand>& out);

  static LegacyRemoteCommand command(
      std::string action);

  static LegacyRemoteCommand command(
      std::string action,
      std::map<std::string, std::string> params);
};

}  // namespace snappy::gateway::g20
