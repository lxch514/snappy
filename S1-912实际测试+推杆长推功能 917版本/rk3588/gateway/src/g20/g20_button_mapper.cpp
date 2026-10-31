

/**
 * @file g20_button_mapper.cpp
 * @brief G20 遥控器通道映射实现
 *
 * @details
 * - 首次收到遥控数据时自动获取控制权并执行 arm。
 * - 安全停止后必须通过屏幕控制权按钮恢复。
 * - 屏幕 acquire 成功后立即解除遥控恢复锁。
 * - 如果 Android 的 arm ACK 延迟，下一次动作会先补发 arm。
 * - 不改变毛刷、风机、爬绳器、推杆和摄像头控制逻辑。
 */

#include "snappy/gateway/g20/g20_button_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace snappy::gateway::g20 {
namespace {

template <typename T>
T clamp_value(
    T value,
    T min_value,
    T max_value) {
  return std::max(
      min_value,
      std::min(value, max_value));
}

int param_int(
    const std::map<std::string, std::string>& params,
    const std::string& key,
    int fallback) {
  const auto iterator =
      params.find(key);

  if (iterator == params.end()) {
    return fallback;
  }

  try {
    return std::stoi(iterator->second);
  } catch (...) {
    return fallback;
  }
}

constexpr int kPushMaxDistanceMm = 300;

}  // namespace

G20ButtonMapper::G20ButtonMapper() {
  reset();
}

/* ============================================================
 * 状态复位
 * ============================================================ */

void G20ButtonMapper::reset() {
  initialized_ = false;
  lease_active_ = false;
  control_armed_ = false;
  screen_recovery_required_ = false;

  last_channels_.assign(
      kChannelCount,
      1500);

  prev_l1_level_ = 0;
  prev_l2_level_ = 0;
  prev_r1_level_ = 0;
  prev_r2_level_ = 0;
  prev_camera_level_ = 0;
  prev_spay_level_ = 0;
  prev_pause_level_ = 0;

  gs_level_ = 0;
  fan_y_level_ = 0;
  fan_x_level_ = 0;
  wheel_level_ = 0;
  push_y_level_ = 0;
  push_x_level_ = 0;

  stopLocalMotionState();

  /*
   * 整机初始化时才复位视频状态。
   * 安全停止不能关闭摄像头。
   */
  video_on_ = false;
  active_camera_ = 1;

  const auto now =
      std::chrono::steady_clock::now();

  last_fan_adjust_ = now;
  last_climber_adjust_ = now;
  last_push_adjust_ = now;
  push_recovery_started_ = {};
  push_extend_recovery_started_ = {};
  last_heartbeat_ = now;
}

/* ============================================================
 * 舵量解析
 * ============================================================ */

int G20ButtonMapper::levelAt(
    const std::vector<int>& channels,
    int index) {
  if (index < 0 ||
      index >=
          static_cast<int>(
              channels.size())) {
    return 0;
  }

  const int value =
      channels[
          static_cast<std::size_t>(
              index)];

  if (value >= kHighThreshold) {
    return 1;
  }

  if (value <= kLowThreshold) {
    return -1;
  }

  return 0;
}

bool G20ButtonMapper::buttonChanged(
    int current_level,
    int& previous_level) {
  if (current_level == 0) {
    previous_level = 0;
    return false;
  }

  const bool changed =
      current_level != previous_level;

  previous_level = current_level;
  return changed;
}

LegacyRemoteCommand
G20ButtonMapper::command(
    std::string action) {
  return {
      std::move(action),
      {},
  };
}

LegacyRemoteCommand
G20ButtonMapper::command(
    std::string action,
    std::map<std::string, std::string> params) {
  return {
      std::move(action),
      std::move(params),
  };
}

/* ============================================================
 * 控制权和 arm
 *
 * G20 不要求用户额外操作 arm。
 * 正常情况下按顺序发送：
 *
 *   control.acquire
 *   control.arm
 *
 * 安全停止后，必须先由屏幕控制权按钮明确恢复。
 * ============================================================ */

void G20ButtonMapper::ensureControl(
    std::vector<LegacyRemoteCommand>& out) {
  /*
   * 安全停止后的遥控动作不能自行解除保护。
   */
  if (screen_recovery_required_) {
    return;
  }

  if (!lease_active_) {
    out.push_back(
        command(
            "control.acquire",
            {{"origin", "g20_mapper"}}));

    lease_active_ = true;
  }

  if (!control_armed_) {
    out.push_back(
        command(
            "control.arm",
            {{"origin", "g20_mapper"}}));

    control_armed_ = true;
  }

  last_heartbeat_ =
      std::chrono::steady_clock::now();
}

/* ============================================================
 * 本地运动状态复位
 * ============================================================ */

void G20ButtonMapper::stopLocalMotionState() {
  brush_mode_ = BrushMode::kStopped;

  fan_active_ = false;
  fan_speed_raw_ = kFanBaseRaw;

  climber_mode_ = ClimberMode::kStopped;
  climber_speed_rpm_ =
      kClimberInitialRpm;

  push_target_ = PushTarget::kNone;
  push_mode_ = PushMode::kIdle;
  push_seconds_ = kPushInitialSeconds;
  push_recovery_started_ = {};
  push_extend_recovery_started_ = {};
}

/* ============================================================
 * 同步屏幕或其他链路的状态
 * ============================================================ */

void G20ButtonMapper::syncExternalCommand(
    const std::string& action,
    const std::map<std::string, std::string>& params) {
  if (action == "control.safe_hold" ||
      action == "control.estop" ||
      action == "control.release" ||
      action == "control.disarm") {
    stopLocalMotionState();

    lease_active_ = false;
    control_armed_ = false;

    /*
     * 安全停止后必须等待屏幕控制权按钮恢复。
     */
    screen_recovery_required_ = true;
    return;
  }

  if (action == "fault.clear") {
    stopLocalMotionState();

    handleGs(gs_level_);

    lease_active_ = false;
    control_armed_ = false;

    /*
     * 清除故障属于屏幕端明确的恢复操作。
     * Android 随后会自动执行 acquire 和 arm。
     */
    screen_recovery_required_ = false;
    return;
  }

  if (action == "control.acquire") {
    const auto origin =
        params.find("origin");

    const bool from_g20_mapper =
        origin != params.end() &&
        origin->second == "g20_mapper";

    const bool recovering_from_safety =
        screen_recovery_required_;

    lease_active_ = true;
    control_armed_ = false;

    /*
     * 屏幕控制权按钮发送的 acquire 表示用户已经明确
     * 同意解除安全停止后的遥控保护。
     *
     * 此处只解除 screen_recovery_required_，仍保持
     * control_armed_=false。正常情况下 Android 紧接着发送
     * control.arm；如果 arm ACK 延迟或 arm 包短暂丢失，
     * 下一次遥控动作会由 ensureControl() 先补发 arm，
     * 不需要再次点击屏幕控制权按钮。
     *
     * mapper 自己产生的 acquire 不能解除安全保护。
     */
        if (!from_g20_mapper &&
            recovering_from_safety) {
          screen_recovery_required_ = false;

          handleGs(gs_level_);
        }
     return;
  }

  if (action == "control.arm") {
    const bool recovering_from_safety =
        screen_recovery_required_;

    lease_active_ = true;
    control_armed_ = true;

    const auto origin =
        params.find("origin");

    const bool from_g20_mapper =
        origin != params.end() &&
        origin->second == "g20_mapper";

    /*
     * 保留 arm 阶段的恢复处理作为兜底。
     * 正常屏幕流程在 acquire 成功时已经解除恢复锁。
     */
    if (!from_g20_mapper) {
      screen_recovery_required_ = false;

      if (recovering_from_safety) {
        handleGs(gs_level_);
      }
    }

    return;
  }

  if (action == "control.heartbeat") {
    lease_active_ = true;
    return;
  }

  if (action == "brush.forward") {
    brush_mode_ = BrushMode::kForward;
    return;
  }

  if (action == "brush.reverse") {
    brush_mode_ = BrushMode::kReverse;
    return;
  }

  if (action == "brush.stop") {
    brush_mode_ = BrushMode::kStopped;
    return;
  }

  if (action == "fan.prepare" ||
      action == "fan.enable") {
    return;
  }

  if (action == "fan.stop" ||
      action == "fan.disable") {
    fan_active_ = false;
    fan_speed_raw_ = kFanBaseRaw;
    return;
  }

  if (action == "fan.both_set") {
    const int left =
        param_int(
            params,
            "left_raw",
            fan_speed_raw_);

    const int right =
        param_int(
            params,
            "right_raw",
            fan_speed_raw_);

    const int next = std::max(left, right);

    if (next > 0) {
      fan_active_ = true;
      fan_speed_raw_ =
          clamp_value(
              next,
              kFanBaseRaw,
              kFanMaxRaw);
    } else {
      fan_active_ = false;
      fan_speed_raw_ = kFanBaseRaw;
    }

    return;
  }

  if (action == "fan.left_set" ||
      action == "fan.right_set") {
    const int next =
        param_int(
            params,
            "speed_raw",
            fan_speed_raw_);

    if (next > 0) {
      fan_active_ = true;
      fan_speed_raw_ =
          clamp_value(
              next,
              kFanBaseRaw,
              kFanMaxRaw);
    } else {
      fan_active_ = false;
      fan_speed_raw_ = kFanBaseRaw;
    }

    return;
  }

  if (action == "climber.ascend" ||
      action == "climber.descend") {
    const int fallback =
        climber_speed_rpm_ > 0
            ? climber_speed_rpm_
            : kClimberInitialRpm;

    const int requested =
        param_int(
            params,
            "speed_rpm",
            fallback);

    climber_speed_rpm_ =
        clamp_value(
            requested > 0
                ? requested
                : kClimberInitialRpm,
            kClimberMinRpm,
            kClimberMaxRpm);

    climber_mode_ =
        action == "climber.ascend"
            ? ClimberMode::kAscend
            : ClimberMode::kDescend;

    return;
  }

  if (action == "climber.stop") {
    climber_mode_ = ClimberMode::kStopped;
    climber_speed_rpm_ =
        kClimberInitialRpm;
    return;
  }

  if (action == "video.start") {
    video_on_ = true;

    active_camera_ =
        clamp_value(
            param_int(
                params,
                "active_camera",
                1),
            1,
            2);

    return;
  }

  if (action == "video.stop") {
    video_on_ = false;
    active_camera_ = 1;
    return;
  }

  if (action == "video.switch_camera" &&
      video_on_) {
    active_camera_ =
        clamp_value(
            param_int(
                params,
                "active_camera",
                active_camera_),
            1,
            2);
  }
}

/* ============================================================
 * GS 推杆选择
 *
 * G 选择刮条推杆，S 选择毛刷滚筒推杆。
 * ============================================================ */

void G20ButtonMapper::handleGs(int level) {
  if (level > 0) {
    push_target_ =
        PushTarget::kScraper;
  } else if (level < 0) {
    push_target_ =
        PushTarget::kBrush;
  } else {
    push_target_ =
        PushTarget::kNone;
  }

  push_mode_ = PushMode::kIdle;
  push_seconds_ = kPushInitialSeconds;
}

int G20ButtonMapper::
    currentPushMaxSeconds() const {
  if (push_target_ ==
      PushTarget::kBrush) {
    return kBrushPushMaxSeconds;
  }

  if (push_target_ ==
      PushTarget::kScraper) {
    return kScraperPushMaxSeconds;
  }

  return 0;
}

int G20ButtonMapper::
    currentPushDistanceMm() const {
  const double speed =
      push_target_ == PushTarget::kBrush
          ? kBrushPushSpeedMmPerSecond
          : kScraperPushSpeedMmPerSecond;

  return clamp_value(
      static_cast<int>(
          std::lround(
              static_cast<double>(
                  push_seconds_) *
              speed)),
      0,
      kPushMaxDistanceMm);
}

int G20ButtonMapper::
    currentPushDurationMs() const {
  const double speed =
      push_target_ == PushTarget::kBrush
          ? kBrushPushSpeedMmPerSecond
          : kScraperPushSpeedMmPerSecond;

  if (speed <= 0.0) {
    return 0;
  }

  return static_cast<int>(
      std::ceil(
          static_cast<double>(
              currentPushDistanceMm()) *
          1000.0 /
          speed));
}

/* ============================================================
 * 遥控数据入口
 * ============================================================ */

std::vector<LegacyRemoteCommand>
G20ButtonMapper::update(
    const std::vector<int>& channels,
    std::chrono::steady_clock::time_point now) {
  std::vector<LegacyRemoteCommand> out;

  if (channels.empty()) {
    return out;
  }

  last_channels_ = channels;

  const int l1_level =
      levelAt(channels, kL1);

  const int l2_level =
      levelAt(channels, kL2);

  const int r1_level =
      levelAt(channels, kR1);

  const int r2_level =
      levelAt(channels, kR2);

  const int camera_level =
      levelAt(channels, kCamera);

  const int spay_level =
      levelAt(channels, kSpay);

  const int pause_level =
      levelAt(channels, kPause);

  if (!initialized_) {
    initialized_ = true;

    prev_l1_level_ = l1_level;
    prev_l2_level_ = l2_level;
    prev_r1_level_ = r1_level;
    prev_r2_level_ = r2_level;
    prev_camera_level_ = camera_level;
    prev_spay_level_ = spay_level;
    prev_pause_level_ = pause_level;

    gs_level_ =
        levelAt(channels, kGs);

    handleGs(gs_level_);

    fan_y_level_ =
        levelAt(
            channels,
            kRightStickY);

    fan_x_level_ =
        levelAt(
            channels,
            kRightStickX);

    wheel_level_ =
        levelAt(channels, kWheel);

    push_y_level_ =
        levelAt(
            channels,
            kLeftStickY);

    push_x_level_ =
        levelAt(
            channels,
            kLeftStickX);

    /*
     * 若初始化时左摇杆已经处于缩回方向且 G/S 已选中推杆，
     * 记下按下起点，让 tick() 能继续完成 1 秒长按判定。
     */
    if (gs_level_ != 0 &&
        push_y_level_ < 0) {
      push_mode_ = PushMode::kRetractSetup;
      push_recovery_started_ = now;
    }

    ensureControl(out);
    return out;
  }

  /*
   * 暂停键优先级最高。
   */
  if (buttonChanged(
          pause_level,
          prev_pause_level_)) {
    out.push_back(
        command("control.safe_hold"));

    stopLocalMotionState();

    lease_active_ = false;
    control_armed_ = false;
    screen_recovery_required_ = true;

    return out;
  }

  if (buttonChanged(
          camera_level,
          prev_camera_level_)) {
    ensureControl(out);

    if (video_on_) {
      video_on_ = false;
      active_camera_ = 1;

      out.push_back(
          command("video.stop"));
    } else {
      video_on_ = true;
      active_camera_ = 1;

      out.push_back(
          command(
              "video.start",
              {{"active_camera", "1"}}));
    }
  }

  if (buttonChanged(
          spay_level,
          prev_spay_level_) &&
      video_on_) {
    ensureControl(out);

    active_camera_ =
        active_camera_ == 1
            ? 2
            : 1;

    out.push_back(
        command(
            "video.switch_camera",
            {{"active_camera",
              std::to_string(
                  active_camera_)}}));
  }

  /*
   * 安全停止后只更新通道基准，不产生设备动作。
   */
  if (screen_recovery_required_) {
    prev_l1_level_ = l1_level;
    prev_l2_level_ = l2_level;
    prev_r1_level_ = r1_level;
    prev_r2_level_ = r2_level;

    gs_level_ =
        levelAt(channels, kGs);

    fan_y_level_ =
        levelAt(
            channels,
            kRightStickY);

    fan_x_level_ =
        levelAt(
            channels,
            kRightStickX);

    wheel_level_ =
        levelAt(channels, kWheel);

    push_y_level_ =
        levelAt(
            channels,
            kLeftStickY);

    push_x_level_ =
        levelAt(
            channels,
            kLeftStickX);

    return out;
  }

  const int current_gs =
      levelAt(channels, kGs);

  if (current_gs != gs_level_) {
    /*
     * 长推恢复过程中切换 G/S 目标（或松开 G/S）时，
     * 先停掉当前正在恢复的那一路推杆，
     * 避免旧目标继续运行。
     */
    if ((push_mode_ == PushMode::kRetractRecovery ||
         push_mode_ == PushMode::kExtendRecovery) &&
        push_target_ != PushTarget::kNone) {
      ensureControl(out);

      out.push_back(
          command(
              push_target_ == PushTarget::kBrush
                  ? "brush_push.stop"
                  : "scraper_push.stop"));
    }

    gs_level_ = current_gs;
    handleGs(current_gs);
  }

  if (buttonChanged(
          l1_level,
          prev_l1_level_)) {
    handleBrushForward(out);
  }

  if (buttonChanged(
          l2_level,
          prev_l2_level_)) {
    handleBrushReverse(out);
  }

  if (buttonChanged(
          r1_level,
          prev_r1_level_)) {
    handleClimberButton(
        ClimberMode::kAscend,
        out);
  }

  if (buttonChanged(
          r2_level,
          prev_r2_level_)) {
    handleClimberButton(
        ClimberMode::kDescend,
        out);
  }

  const int current_fan_y =
      levelAt(
          channels,
          kRightStickY);

  if (current_fan_y !=
      fan_y_level_) {
    fan_y_level_ = current_fan_y;

    handleFanY(
        current_fan_y,
        out);
  }

  const int current_fan_x =
      levelAt(
          channels,
          kRightStickX);

  if (current_fan_x !=
      fan_x_level_) {
    fan_x_level_ = current_fan_x;

    handleFanX(
        current_fan_x,
        now,
        out,
        true);
  }

  const int current_wheel =
      levelAt(channels, kWheel);

  if (current_wheel !=
      wheel_level_) {
    wheel_level_ = current_wheel;

    handleWheel(
        current_wheel,
        now,
        out,
        true);
  }

  const int current_push_y =
      levelAt(
          channels,
          kLeftStickY);

  if (current_push_y !=
      push_y_level_) {
    push_y_level_ = current_push_y;

    handlePushY(
        current_push_y,
        now,
        out);
  }

  const int current_push_x =
      levelAt(
          channels,
          kLeftStickX);

  if (current_push_x !=
      push_x_level_) {
    push_x_level_ = current_push_x;

    handlePushX(
        current_push_x,
        now,
        out,
        true);
  }

  return out;
}

/* ============================================================
 * 周期任务和心跳
 * ============================================================ */

std::vector<LegacyRemoteCommand>
G20ButtonMapper::tick(
    std::chrono::steady_clock::time_point now) {
  std::vector<LegacyRemoteCommand> out;

  if (!initialized_ ||
      screen_recovery_required_) {
    return out;
  }

  if (lease_active_ &&
      now - last_heartbeat_ >=
          kHeartbeatInterval) {
    out.push_back(
        command("control.heartbeat"));

    last_heartbeat_ = now;
  }

  handleFanX(
      fan_x_level_,
      now,
      out,
      false);

  handleWheel(
      wheel_level_,
      now,
      out,
      false);

  handlePushX(
      push_x_level_,
      now,
      out,
      false);

  /*
   * 左摇杆长推 1 秒判定在这里完成：
   * 摇杆保持不动时 update() 不会再次调用 handlePushY，
   * 必须由周期 tick() 推动。
   *
   * 先判断伸出，再判断缩回；两者互斥
   * （push_y_level_ 不可能同时 > 0 和 < 0）。
   */
  handlePushExtendRecovery(
      now,
      out);

  handlePushRecovery(
      now,
      out);

  return out;
}

/* ============================================================
 * 毛刷控制
 * ============================================================ */

void G20ButtonMapper::handleBrushForward(
    std::vector<LegacyRemoteCommand>& out) {
  /*
   * 反转时不能直接切换正转。
   */
  if (brush_mode_ ==
      BrushMode::kReverse) {
    return;
  }

  ensureControl(out);

  if (brush_mode_ ==
      BrushMode::kForward) {
    brush_mode_ =
        BrushMode::kStopped;

    out.push_back(
        command("brush.stop"));

    return;
  }

  brush_mode_ =
      BrushMode::kForward;

  out.push_back(
      command(
          "brush.forward",
          {{"speed_raw",
            std::to_string(
                kBrushMaxRaw)}}));
}

void G20ButtonMapper::handleBrushReverse(
    std::vector<LegacyRemoteCommand>& out) {
  /*
   * 正转时不能直接切换反转。
   */
  if (brush_mode_ ==
      BrushMode::kForward) {
    return;
  }

  ensureControl(out);

  if (brush_mode_ ==
      BrushMode::kReverse) {
    brush_mode_ =
        BrushMode::kStopped;

    out.push_back(
        command("brush.stop"));

    return;
  }

  brush_mode_ =
      BrushMode::kReverse;

  out.push_back(
      command(
          "brush.reverse",
          {{"speed_raw",
            std::to_string(
                kBrushMaxRaw)}}));
}

/* ============================================================
 * 推进风机控制
 * ============================================================ */

void G20ButtonMapper::handleFanY(
    int level,
    std::vector<LegacyRemoteCommand>& out) {
  if (level < 0) {
    ensureControl(out);

    fan_active_ = true;
    fan_speed_raw_ = kFanBaseRaw;

    out.push_back(
        command(
            "fan.both_set",
            {
                {"left_raw",
                 std::to_string(
                     fan_speed_raw_)},
                {"right_raw",
                 std::to_string(
                     fan_speed_raw_)},
            }));

    return;
  }

  if (level > 0) {
    ensureControl(out);

    fan_active_ = false;
    fan_speed_raw_ = kFanBaseRaw;

    out.push_back(
        command("fan.stop"));
  }
}

void G20ButtonMapper::handleFanX(
    int level,
    std::chrono::steady_clock::time_point now,
    std::vector<LegacyRemoteCommand>& out,
    bool edge_only) {
  if (!fan_active_ ||
      level == 0) {
    return;
  }

  if (!edge_only &&
      now - last_fan_adjust_ <
          kRepeatInterval) {
    return;
  }

  adjustFan(
      level < 0 ? -1 : 1,
      out);

  last_fan_adjust_ = now;
}

void G20ButtonMapper::adjustFan(
    int direction,
    std::vector<LegacyRemoteCommand>& out) {
  ensureControl(out);

  const int next =
      clamp_value(
          fan_speed_raw_ +
              direction *
                  kFanStepRaw,
          kFanBaseRaw,
          kFanMaxRaw);

  if (next == fan_speed_raw_) {
    return;
  }

  fan_speed_raw_ = next;

  out.push_back(
      command(
          "fan.both_set",
          {
              {"left_raw",
               std::to_string(
                   fan_speed_raw_)},
              {"right_raw",
               std::to_string(
                   fan_speed_raw_)},
          }));
}

/* ============================================================
 * 爬绳器控制
 * ============================================================ */

void G20ButtonMapper::handleClimberButton(
    ClimberMode target,
    std::vector<LegacyRemoteCommand>& out) {
  ensureControl(out);

  if (climber_mode_ == target) {
    climber_mode_ =
        ClimberMode::kStopped;

    climber_speed_rpm_ =
        kClimberInitialRpm;

    out.push_back(
        command("climber.stop"));

    return;
  }

  climber_mode_ = target;
  climber_speed_rpm_ =
      kClimberInitialRpm;

  if (target ==
      ClimberMode::kAscend) {
    out.push_back(
        command(
            "climber.ascend",
            {{"speed_rpm",
              std::to_string(
                  climber_speed_rpm_)}}));
  } else {
    out.push_back(
        command(
            "climber.descend",
            {{"speed_rpm",
              std::to_string(
                  climber_speed_rpm_)}}));
  }
}

void G20ButtonMapper::handleWheel(
    int level,
    std::chrono::steady_clock::time_point now,
    std::vector<LegacyRemoteCommand>& out,
    bool edge_only) {
  if (climber_mode_ ==
          ClimberMode::kStopped ||
      level == 0) {
    return;
  }

  if (!edge_only &&
      now - last_climber_adjust_ <
          kRepeatInterval) {
    return;
  }

  adjustClimber(
      level < 0 ? -1 : 1,
      out);

  last_climber_adjust_ = now;
}

void G20ButtonMapper::adjustClimber(
    int direction,
    std::vector<LegacyRemoteCommand>& out) {
  ensureControl(out);

  const int next =
      clamp_value(
          climber_speed_rpm_ +
              direction *
                  kClimberStepRpm,
          kClimberMinRpm,
          kClimberMaxRpm);

  if (next ==
      climber_speed_rpm_) {
    return;
  }

  climber_speed_rpm_ = next;

  if (climber_mode_ ==
      ClimberMode::kAscend) {
    out.push_back(
        command(
            "climber.ascend",
            {{"speed_rpm",
              std::to_string(
                  climber_speed_rpm_)}}));
  } else if (
      climber_mode_ ==
      ClimberMode::kDescend) {
    out.push_back(
        command(
            "climber.descend",
            {{"speed_rpm",
              std::to_string(
                  climber_speed_rpm_)}}));
  }
}

/* ============================================================
 * 推杆左摇杆控制
 * ============================================================ */

void G20ButtonMapper::handlePushY(
    int level,
    std::chrono::steady_clock::time_point now,
    std::vector<LegacyRemoteCommand>& out) {
  if (gs_level_ == 0 ||
      push_target_ ==
          PushTarget::kNone) {
    return;
  }

  /*
   * 回正：只有“长推恢复动作”已经启动时，才立即发送停止。
   * 普通的推杆时间设置流程保持原样，
   * 第一次向下只是进入设置，不会误触发停止。
   */
  if (level == 0) {
    if (push_mode_ == PushMode::kRetractRecovery ||
        push_mode_ == PushMode::kExtendRecovery) {
      ensureControl(out);

      out.push_back(
          command(
              push_target_ == PushTarget::kBrush
                  ? "brush_push.stop"
                  : "scraper_push.stop"));

      push_mode_ = PushMode::kIdle;
      push_seconds_ = kPushInitialSeconds;
      push_recovery_started_ = {};
      push_extend_recovery_started_ = {};
      last_push_adjust_ = now;
    }

    return;
  }

  /*
   * 高舵量对应左摇杆向上，进入或执行原有伸出流程。
   * 不改变原来的“先进入设置、再调整秒数、再次向上执行”逻辑。
   */
  if (level > 0) {
    if (push_mode_ == PushMode::kExtendRecovery) {
      return;
    }

    if (push_mode_ !=
        PushMode::kExtendSetup) {
      push_mode_ =
          PushMode::kExtendSetup;

      push_seconds_ =
          kPushInitialSeconds;

      push_recovery_started_ = {};
      push_extend_recovery_started_ = now;
      last_push_adjust_ = now;
      return;
    }

    if (push_seconds_ <= 0) {
      /*
       * 没有设置秒数时，保持原有的“不动作”行为，
       * 由 tick() 里的 1 秒长按判定决定是否进入长推伸出。
       */
      if (push_extend_recovery_started_.time_since_epoch().count() == 0) {
        push_extend_recovery_started_ = now;
      }
      return;
    }

    ensureControl(out);

    out.push_back(
        command(
            push_target_ ==
                    PushTarget::kBrush
                ? "brush_push.extend_timed"
                : "scraper_push.extend_timed",
            {
                {"duration_ms",
                 std::to_string(
                     currentPushDurationMs())},
                {"seconds",
                 std::to_string(
                     push_seconds_)},
                {"distance_mm",
                 std::to_string(
                     currentPushDistanceMm())},
            }));

    push_mode_ = PushMode::kIdle;
    push_seconds_ = kPushInitialSeconds;
    push_recovery_started_ = {};
    push_extend_recovery_started_ = {};
    last_push_adjust_ = now;
    return;
  }

  /*
   * 低舵量对应左摇杆向下。
   *
   * 保留原有“第一次向下进入缩回设置”的行为；
   * 当当前没有设置推杆秒数（push_seconds_ == 0）时，
   * 同时启动 1 秒长按检测。持续超过 1 秒后由 tick()
   * 进入“长推缩回”：
   *   - 前 1 秒不动作；
   *   - 第 1 秒之后开始持续缩回；
   *   - 摇杆一回正立即 stop；
   *   - 恢复动作不受当前行程计数为 0 的限制；
   *   - 行程计数会让 ActuatorService 仍按实际运行时间计算，
   *     没有计数时扣减会被钳制在 0，不会产生负数。
   */
  if (level < 0) {
    if (push_mode_ == PushMode::kRetractRecovery) {
      return;
    }

    if (push_mode_ !=
        PushMode::kRetractSetup) {
      push_mode_ =
          PushMode::kRetractSetup;

      push_seconds_ =
          kPushInitialSeconds;

      push_recovery_started_ = now;
      last_push_adjust_ = now;
      return;
    }

    if (push_seconds_ <= 0) {
      /*
       * 普通首次向下已经在上面进入 setup；
       * 此处只负责持续按下时的恢复计时，真正启动由 tick() 完成。
       */
      if (push_recovery_started_.time_since_epoch().count() == 0) {
        push_recovery_started_ = now;
      }
      return;
    }

    /* 有设置秒数时保持原有的第二次向下执行缩回逻辑。 */
    ensureControl(out);

    out.push_back(
        command(
            push_target_ ==
                    PushTarget::kBrush
                ? "brush_push.retract_timed"
                : "scraper_push.retract_timed",
            {
                {"duration_ms",
                 std::to_string(
                     currentPushDurationMs())},
                {"seconds",
                 std::to_string(
                     push_seconds_)},
                {"distance_mm",
                 std::to_string(
                     currentPushDistanceMm())},
            }));

    push_mode_ = PushMode::kIdle;
    push_seconds_ = kPushInitialSeconds;
    push_recovery_started_ = {};
    push_extend_recovery_started_ = {};
    last_push_adjust_ = now;
  }
}

void G20ButtonMapper::handlePushExtendRecovery(
    std::chrono::steady_clock::time_point now,
    std::vector<LegacyRemoteCommand>& out) {
  /*
   * 长推伸出判定：
   *   - G/S 已选中推杆；
   *   - 摇杆持续向上；
   *   - 未设置秒数（设置秒数时走原有定时逻辑）；
   *   - 已持续推住 kPushRecoveryDelay。
   */
  if (gs_level_ == 0 ||
      push_target_ == PushTarget::kNone ||
      push_mode_ != PushMode::kExtendSetup ||
      push_seconds_ != 0 ||
      push_y_level_ <= 0 ||
      push_extend_recovery_started_.time_since_epoch().count() == 0) {
    return;
  }

  if (now - push_extend_recovery_started_ <
      kPushRecoveryDelay) {
    return;
  }

  ensureControl(out);

  out.push_back(
      command(
          push_target_ == PushTarget::kBrush
              ? "brush_push.extend_recovery"
              : "scraper_push.extend_recovery",
          {}));

  push_mode_ = PushMode::kExtendRecovery;
  last_push_adjust_ = now;
}

void G20ButtonMapper::handlePushRecovery(
    std::chrono::steady_clock::time_point now,
    std::vector<LegacyRemoteCommand>& out) {
  /*
   * 长推缩回判定：
   *   - G/S 已选中推杆；
   *   - 摇杆持续向下；
   *   - 未设置秒数（设置秒数时走原有定时逻辑）；
   *   - 已持续推住 kPushRecoveryDelay。
   */
  if (gs_level_ == 0 ||
      push_target_ == PushTarget::kNone ||
      push_mode_ != PushMode::kRetractSetup ||
      push_seconds_ != 0 ||
      push_y_level_ >= 0 ||
      push_recovery_started_.time_since_epoch().count() == 0) {
    return;
  }

  if (now - push_recovery_started_ <
      kPushRecoveryDelay) {
    return;
  }

  ensureControl(out);

  out.push_back(
      command(
          push_target_ == PushTarget::kBrush
              ? "brush_push.retract_recovery"
              : "scraper_push.retract_recovery",
          {}));

  push_mode_ = PushMode::kRetractRecovery;
  last_push_adjust_ = now;
}

void G20ButtonMapper::handlePushX(
    int level,
    std::chrono::steady_clock::time_point now,
    std::vector<LegacyRemoteCommand>& out,
    bool edge_only) {
  /*
   * out 保留在函数签名中，以维持原有接口。
   * 推杆时间调整只改变本地设置，不立即发送动作。
   */
  static_cast<void>(out);

  if (gs_level_ == 0 ||
      push_target_ ==
          PushTarget::kNone ||
      push_mode_ ==
          PushMode::kIdle ||
      level == 0) {
    return;
  }

  /*
   * 左右短推立即调整一次；
   * 长推时每 500ms 再调整一次。
   */
  if (!edge_only &&
      now - last_push_adjust_ <
          kRepeatInterval) {
    return;
  }

  adjustPushSeconds(
      level < 0 ? -1 : 1);

  last_push_adjust_ = now;
}

void G20ButtonMapper::adjustPushSeconds(
    int direction) {
  push_seconds_ =
      clamp_value(
          push_seconds_ + direction,
          kPushMinSeconds,
          currentPushMaxSeconds());
}

}  // namespace snappy::gateway::g20