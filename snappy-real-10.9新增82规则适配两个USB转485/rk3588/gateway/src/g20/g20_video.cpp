/**
 * @file g20_video.cpp
 * @brief G20 摄像头 RTP 推流及本地录像实现
 *
 * @details
 * - G20 摄像头启停和切换全部由独立 worker 线程执行。
 * - gateway 主循环不直接操作 USB、V4L2 或 GStreamer。
 * - G20 推流严格保持同一时间只有一个摄像头。
 * - 保留原来的 camera1 / camera2 切换逻辑。
 * - 新增 camera1 / camera2 独立本地录像。
 * - 本地录像不参与 G20 推流切换。
 * - camera1 / camera2 本地录像可以同时运行。
 * - 摄像头异常不会阻塞 RS485、CAN、传感器和执行器控制。
 */

#include "snappy/gateway/g20/g20_video.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <cerrno>
#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace snappy::gateway::g20 {
namespace {

#if defined(__linux__)

bool process_alive(int pid) {
  if (pid <= 0) {
    return false;
  }

  if (::kill(pid, 0) == 0) {
    return true;
  }

  return errno == EPERM;
}

bool process_group_alive(int process_group_id) {
  if (process_group_id <= 0) {
    return false;
  }

  if (::kill(-process_group_id, 0) == 0) {
    return true;
  }

  return errno == EPERM;
}

bool char_device_exists(const std::string& path) {
  struct stat st {};

  return !path.empty() &&
         ::stat(path.c_str(), &st) == 0 &&
         S_ISCHR(st.st_mode);
}

bool is_video_node_path(const std::string& path) {
  constexpr const char* prefix = "/dev/video";
  constexpr std::size_t prefix_length = 10;

  if (path.rfind(prefix, 0) != 0 ||
      path.size() <= prefix_length) {
    return false;
  }

  for (std::size_t i = prefix_length; i < path.size(); ++i) {
    if (!std::isdigit(
            static_cast<unsigned char>(path[i]))) {
      return false;
    }
  }

  return true;
}

int video_node_number(const std::string& path) {
  if (!is_video_node_path(path)) {
    return -1;
  }

  return std::atoi(path.substr(10).c_str());
}

std::string video_device_sysfs_path(
    const std::string& path) {
  const int number = video_node_number(path);

  if (number < 0) {
    return {};
  }

  const std::string sysfs_path =
      "/sys/class/video4linux/video" +
      std::to_string(number) +
      "/device";

  char resolved[PATH_MAX] {};

  if (::realpath(sysfs_path.c_str(), resolved) ==
      nullptr) {
    return {};
  }

  return std::string(resolved);
}

bool same_video_device(
    const std::string& lhs,
    const std::string& rhs) {
  if (lhs.empty() || rhs.empty()) {
    return false;
  }

  if (lhs == rhs) {
    return true;
  }

  const auto left =
      video_device_sysfs_path(lhs);

  const auto right =
      video_device_sysfs_path(rhs);

  return !left.empty() && left == right;
}

bool video_device_is_usb(
    const std::string& path) {
  const auto sysfs_path =
      video_device_sysfs_path(path);

  return sysfs_path.find("/usb") !=
             std::string::npos ||
         sysfs_path.find(".usb/") !=
             std::string::npos;
}

bool video_device_has_capture_caps(
    const std::string& path) {
  int fd = ::open(
      path.c_str(),
      O_RDWR | O_NONBLOCK | O_CLOEXEC);

  if (fd < 0) {
    fd = ::open(
        path.c_str(),
        O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  }

  if (fd < 0) {
    return false;
  }

  v4l2_capability capability {};

  const bool queried =
      ::ioctl(
          fd,
          VIDIOC_QUERYCAP,
          &capability) == 0;

  ::close(fd);

  if (!queried) {
    return false;
  }

  const std::uint32_t capabilities =
      (capability.capabilities &
       V4L2_CAP_DEVICE_CAPS)
          ? capability.device_caps
          : capability.capabilities;

  return
      (capabilities &
       (V4L2_CAP_VIDEO_CAPTURE |
        V4L2_CAP_VIDEO_CAPTURE_MPLANE)) != 0;
}

bool usable_capture_device(
    const std::string& path) {
  return is_video_node_path(path) &&
         char_device_exists(path) &&
         video_device_has_capture_caps(path);
}

std::vector<std::string>
scan_usb_capture_devices() {
  std::vector<std::string> devices;

  DIR* dir = ::opendir("/dev");

  if (dir == nullptr) {
    return devices;
  }

  while (dirent* entry = ::readdir(dir)) {
    const std::string name =
        entry->d_name;

    if (name.rfind("video", 0) != 0 ||
        name.size() <= 5) {
      continue;
    }

    const std::string path =
        "/dev/" + name;

    if (usable_capture_device(path) &&
        video_device_is_usb(path)) {
      devices.push_back(path);
    }
  }

  ::closedir(dir);

  std::sort(
      devices.begin(),
      devices.end(),
      [](const std::string& lhs,
         const std::string& rhs) {
        const int left =
            video_node_number(lhs);

        const int right =
            video_node_number(rhs);

        if (left != right) {
          return left < right;
        }

        return lhs < rhs;
      });

  return devices;
}

bool child_still_running_after_start(
    int& pid,
    int camera,
    const std::string& device,
    std::string* detail) {
  if (pid <= 0) {
    return false;
  }

  std::this_thread::sleep_for(
      std::chrono::milliseconds(300));

  int status = 0;

  const pid_t waited =
      ::waitpid(
          pid,
          &status,
          WNOHANG);

  if (waited == pid) {
    std::ostringstream message;

    message
        << "camera"
        << camera
        << " stream exited immediately device="
        << device
        << " status="
        << status;

    if (detail != nullptr) {
      *detail = message.str();
    }

    pid = -1;
    return false;
  }

  if (waited < 0 &&
      errno == ECHILD) {
    std::ostringstream message;

    message
        << "camera"
        << camera
        << " stream child disappeared device="
        << device;

    if (detail != nullptr) {
      *detail = message.str();
    }

    pid = -1;
    return false;
  }

  return process_alive(pid);
}

#endif

std::string getenv_string(
    const std::string& key) {
  const char* value =
      std::getenv(key.c_str());

  return value == nullptr
             ? std::string{}
             : std::string(value);
}

int getenv_int_range(
    const char* key,
    int fallback,
    int minimum,
    int maximum) {
  const char* value =
      std::getenv(key);

  if (value == nullptr ||
      value[0] == '\0') {
    return fallback;
  }

  try {
    const int parsed =
        std::stoi(value);

    if (parsed < minimum ||
        parsed > maximum) {
      return fallback;
    }

    return parsed;
  } catch (...) {
    return fallback;
  }
}

/*
 * 布尔型环境变量开关。
 *
 * 未设置或无法识别时返回 fallback，
 * 因此默认值由调用方决定，行为与改动前一致。
 */
bool getenv_flag(
    const char* key,
    bool fallback) {
  const char* value =
      std::getenv(key);

  if (value == nullptr ||
      value[0] == '\0') {
    return fallback;
  }

  const std::string text(value);

  if (text == "0" || text == "false" ||
      text == "FALSE" || text == "no" ||
      text == "NO" || text == "off" ||
      text == "OFF") {
    return false;
  }

  if (text == "1" || text == "true" ||
      text == "TRUE" || text == "yes" ||
      text == "YES" || text == "on" ||
      text == "ON") {
    return true;
  }

  return fallback;
}

void replace_all(
    std::string& text,
    const std::string& from,
    const std::string& to) {
  std::size_t position = 0;

  while ((position =
              text.find(from, position)) !=
         std::string::npos) {
    text.replace(
        position,
        from.size(),
        to);

    position += to.size();
  }
}

std::string shell_quote(
    const std::string& value) {
  std::string result = "'";

  for (const char ch : value) {
    if (ch == '\'') {
      result += "'\\''";
    } else {
      result += ch;
    }
  }

  result += "'";

  return result;
}

}  // namespace

G20VideoAdapter::G20VideoAdapter(
    G20VideoConfig config)
    : config_(std::move(config)) {
  state_snapshot_.running = false;

  state_snapshot_.active_camera = 1;

  state_snapshot_.active_url =
      config_.camera1_play_url;

  state_snapshot_.camera1_url =
      config_.camera1_play_url;

  state_snapshot_.camera2_url =
      config_.camera2_play_url;

  state_snapshot_.target_address =
      target_address();

  state_snapshot_.camera1_port =
      config_.camera1_udp_port;

  state_snapshot_.camera2_port =
      config_.camera2_udp_port;

  state_snapshot_.detail =
      "video worker starting";

  try {
    std::lock_guard<std::mutex> lock(
        state_mutex_);

    worker_started_ = true;

    worker_thread_ =
        std::thread(
            &G20VideoAdapter::worker_loop,
            this);
  } catch (...) {
    worker_started_ = false;

    last_status_ =
        "video worker thread start failed";

    state_snapshot_.detail =
        last_status_;
  }
}

G20VideoAdapter::~G20VideoAdapter() {
  {
    std::lock_guard<std::mutex> lock(
        state_mutex_);

    if (worker_started_ &&
        !worker_stop_requested_) {
      worker_stop_requested_ = true;

      commands_.clear();

      commands_.push_back(
          VideoCommand{
              VideoCommandType::kShutdown,
              1});
    }
  }

  command_cv_.notify_one();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool G20VideoAdapter::available()
    const noexcept {
  try {
    return !target_address().empty() &&
           (!config_.camera1_device.empty() ||
            !config_.camera2_device.empty());
  } catch (...) {
    return false;
  }
}

bool G20VideoAdapter::start_g20_stream() {
  return enqueue_command(
      VideoCommand{
          VideoCommandType::kStart,
          1});
}

bool G20VideoAdapter::switch_g20_camera(
    int active_camera) {
  if (active_camera != 1 &&
      active_camera != 2) {
    set_status(
        "invalid active camera");

    return false;
  }

  return enqueue_command(
      VideoCommand{
          VideoCommandType::kSwitch,
          active_camera});
}

void G20VideoAdapter::stop() noexcept {
  static_cast<void>(
      enqueue_command(
          VideoCommand{
              VideoCommandType::kStop,
              1}));
}

bool G20VideoAdapter::enqueue_command(
    VideoCommand command) noexcept {
  try {
    {
      std::lock_guard<std::mutex> lock(
          state_mutex_);

      if (!worker_started_ ||
          worker_stop_requested_) {
        return false;
      }

      if (command.type ==
          VideoCommandType::kStop) {
        commands_.clear();
      } else if (
          command.type ==
          VideoCommandType::kStart) {
        const bool already_queued =
            std::any_of(
                commands_.begin(),
                commands_.end(),
                [](const VideoCommand& item) {
                  return item.type ==
                      VideoCommandType::kStart;
                });

        if (already_queued) {
          return true;
        }
      } else if (
          command.type ==
          VideoCommandType::kSwitch) {
        commands_.erase(
            std::remove_if(
                commands_.begin(),
                commands_.end(),
                [](const VideoCommand& item) {
                  return item.type ==
                      VideoCommandType::kSwitch;
                }),
            commands_.end());
      }

      commands_.push_back(command);

      switch (command.type) {
        case VideoCommandType::kStart:
          last_status_ =
              "video start queued";
          break;

        case VideoCommandType::kStop:
          last_status_ =
              "video stop queued";
          break;

        case VideoCommandType::kSwitch:
          last_status_ =
              "camera" +
              std::to_string(command.camera) +
              " switch queued";
          break;

        case VideoCommandType::kShutdown:
          last_status_ =
              "video shutdown queued";
          break;
      }

      state_snapshot_.detail =
          last_status_;
    }

    command_cv_.notify_one();

    return true;
  } catch (...) {
    return false;
  }
}

void G20VideoAdapter::worker_loop()
    noexcept {
  while (true) {
    VideoCommand command;

    {
      std::unique_lock<std::mutex> lock(
          state_mutex_);

      command_cv_.wait(
          lock,
          [this] {
            return !commands_.empty();
          });

      command =
          commands_.front();

      commands_.pop_front();
    }

    try {
      if (command.type ==
          VideoCommandType::kShutdown) {
        stop_stream_impl();

        stop_recording_all();

        update_snapshot();

        return;
      }

      switch (command.type) {
        case VideoCommandType::kStart:
          static_cast<void>(
              start_stream_impl());
          break;

        case VideoCommandType::kStop:
          stop_stream_impl();
          break;

        case VideoCommandType::kSwitch:
          static_cast<void>(
              switch_stream_impl(
                  command.camera));
          break;

        case VideoCommandType::kShutdown:
          break;
      }

      update_snapshot();
    } catch (
        const std::exception& error) {
      set_status(
          std::string(
              "video worker exception: ") +
          error.what());

      update_snapshot();
    } catch (...) {
      set_status(
          "video worker unknown exception");

      update_snapshot();
    }
  }
}

void G20VideoAdapter::set_status(
    const std::string& status) {
  std::lock_guard<std::mutex> lock(
      state_mutex_);

  last_status_ = status;

  state_snapshot_.detail =
      last_status_;
}

void G20VideoAdapter::update_snapshot() {
  bool running = false;

#if defined(__linux__)
  /*
   * 注意：
   * running 只表示 G20 RTP 推流是否运行。
   *
   * 本地录像进程不改变这个状态。
   */
  running =
      process_alive(camera1_pid_) ||
      process_alive(camera2_pid_) ||
      process_group_alive(
          camera1_process_group_id_) ||
      process_group_alive(
          camera2_process_group_id_);
#endif

  VideoStreamState snapshot;

  snapshot.running = running;

  snapshot.active_camera =
      active_camera_;

  snapshot.active_url =
      play_url(active_camera_);

  snapshot.camera1_url =
      config_.camera1_play_url;

  snapshot.camera2_url =
      config_.camera2_play_url;

  snapshot.target_address =
      target_address();

  snapshot.camera1_port =
      config_.camera1_udp_port;

  snapshot.camera2_port =
      config_.camera2_udp_port;

  {
    std::lock_guard<std::mutex> lock(
        state_mutex_);

    snapshot.detail =
        last_status_;

    state_snapshot_ =
        std::move(snapshot);
  }
}

VideoStreamState
G20VideoAdapter::video_state() const {
  std::lock_guard<std::mutex> lock(
      state_mutex_);

  return state_snapshot_;
}

bool G20VideoAdapter::running()
    const noexcept {
  std::lock_guard<std::mutex> lock(
      state_mutex_);

  return state_snapshot_.running;
}

std::string G20VideoAdapter::status()
    const {
  std::lock_guard<std::mutex> lock(
      state_mutex_);

  return state_snapshot_.detail;
}

std::string G20VideoAdapter::target_address()
    const {
  const auto environment_target =
      getenv_string(
          config_.target_env);

  if (!environment_target.empty()) {
    return environment_target;
  }

  return config_.target_address;
}

std::string G20VideoAdapter::camera_device(
    int camera) const {
#if defined(__linux__)

  if (camera == 1) {
    const auto environment_device =
        getenv_string(
            config_.camera1_env);

    if (!environment_device.empty() &&
        usable_capture_device(
            environment_device)) {
      return environment_device;
    }

    const auto legacy_device =
        getenv_string(
            config_.legacy_device_env);

    if (!legacy_device.empty() &&
        usable_capture_device(
            legacy_device)) {
      return legacy_device;
    }

    if (usable_capture_device(
            config_.camera1_device)) {
      return config_.camera1_device;
    }

    const auto devices =
        scan_usb_capture_devices();

    if (!devices.empty()) {
      return devices.front();
    }

    return config_.camera1_device;
  }

  const auto camera1 =
      camera_device(1);

  const auto environment_device =
      getenv_string(
          config_.camera2_env);

  if (!environment_device.empty() &&
      usable_capture_device(
          environment_device) &&
      !same_video_device(
          environment_device,
          camera1)) {
    return environment_device;
  }

  if (usable_capture_device(
          config_.camera2_device) &&
      !same_video_device(
          config_.camera2_device,
          camera1)) {
    return config_.camera2_device;
  }

  const auto devices =
      scan_usb_capture_devices();

  for (const auto& device : devices) {
    if (!same_video_device(
            device,
            camera1)) {
      return device;
    }
  }

  return {};

#else

  return camera == 2
             ? config_.camera2_device
             : config_.camera1_device;

#endif
}

std::string G20VideoAdapter::play_url(
    int camera) const {
  return camera == 2
             ? config_.camera2_play_url
             : config_.camera1_play_url;
}

std::uint16_t G20VideoAdapter::udp_port(
    int camera) const {
  return camera == 2
             ? config_.camera2_udp_port
             : config_.camera1_udp_port;
}

/*
 * ============================================================
 * 原来的 G20 RTP 推流命令
 * ============================================================
 *
 * 这里必须保持为“单摄像头、单 RTP pipeline”。
 *
 * 不在这里加入 tee。
 * 不在这里加入 splitmuxsink。
 * 不在这里加入第二个 udpsink。
 *
 * 这样就不会改变原来的 G20 推流和摄像头切换逻辑。
 */
std::string G20VideoAdapter::make_default_command(
    int camera) const {
  const int width =
      getenv_int_range("G20_VIDEO_WIDTH",
                       config_.width,
                       160,
                       3840);

  const int height =
      getenv_int_range("G20_VIDEO_HEIGHT",
                       config_.height,
                       120,
                       2160);

  const int fps =
      getenv_int_range("G20_VIDEO_FPS",
                       config_.fps,
                       1,
                       60);

  const int bps =
      getenv_int_range("G20_VIDEO_BPS",
                       config_.bps,
                       200000,
                       50000000);

  const int gop =
      getenv_int_range("G20_VIDEO_GOP",
                       config_.gop,
                       1,
                       120);

  const int qp_init =
      getenv_int_range("G20_VIDEO_QP_INIT",
                       config_.qp_init,
                       1,
                       51);

  /*
   * ============================================================
   * 当前摄像头同时执行：
   *
   *   1. G20 RTP 推流
   *   2. 本地 MP4 录像
   *
   * 当前实际设备：
   *   Camera 1 -> /dev/video81
   *   Camera 2 -> /dev/video83
   *
   * 录像目录：
   *   /root/var/video/camera1/
   *   /root/var/video/camera2/
   *
   * 每个 MP4 约 10 分钟。
   *
   * H264 编码后使用 tee 正确分成两路：
   *
   *                    ┌──> h264parse -> RTP -> G20
   * H264 -> tee -------┤
   *                    └──> h264parse -> MP4
   *
   * 注意：
   * 这里必须使用 tee。
   * 不能把 h264parse 本身当作 tee 使用。
   * ============================================================
   */

  const std::string record_dir =
      "/root/var/video/camera" +
      std::to_string(camera);

  const std::string record_pattern =
      record_dir +
      "/camera" +
      std::to_string(camera) +
      "-%05d.mp4";

  std::ostringstream command;

  command
      << "mkdir -p "
      << shell_quote(record_dir)
      << " && "

      << "exec gst-launch-1.0 -e -q "

      /*
       * USB 摄像头
       */
      << "v4l2src device="
      << shell_quote(camera_device(camera))
      << " do-timestamp=true "

      /*
       * 摄像头原始 MJPEG
       */
      << "! image/jpeg,width="
      << width
      << ",height="
      << height
      << ",framerate="
      << fps
      << "/1 "

      /*
       * MJPEG -> NV12
       */
      << "! mppjpegdec "

      /*
       * NV12 -> H264
       *
       * 保持原来的编码参数。
       */
      << "! mpph264enc bps="
      << bps
      << " rc-mode=cbr gop="
      << gop
      << " qp-init="
      << qp_init
      << " "

      /*
       * ========================================================
       * 正确分流
       * ========================================================
       */
      << "! tee name=encoded_h264 "

      /*
       * --------------------------------------------------------
       * 分支 1：原来的 G20 RTP 推流
       * --------------------------------------------------------
       */
      << "encoded_h264. "
      << "! queue max-size-buffers=1 "
         "max-size-bytes=0 "
         "max-size-time=0 "
         "leaky=upstream "

      << "! h264parse "

      << "! rtph264pay "
         "config-interval=-1 "
         "pt=96 "
         "mtu=1200 "

      << "! udpsink host="
      << shell_quote(target_address())
      << " port="
      << udp_port(camera)
      << " sync=false "
         "async=false "

      /*
       * --------------------------------------------------------
       * 分支 2：本地录像
       * --------------------------------------------------------
       *
       * 不重新编码。
       * 直接保存已经编码好的 H264。
       *
       * splitmuxsink 每 600 秒切一个 MP4。
       */
      << "encoded_h264. "
      << "! queue max-size-buffers=30 "
         "max-size-bytes=0 "
         "max-size-time=1000000000 "
         "leaky=downstream "

      << "! h264parse "

      << "! splitmuxsink "
      << "location="
      << shell_quote(record_pattern)
      << " "
      << "max-size-time=600000000000 "
      << "muxer-factory=mp4mux "
      << "async-finalize=true";

  return command.str();
}

/*
 * ============================================================
 * 本地录像命令
 * ============================================================
 *
 * 这是新增功能。
 *
 * 它与 G20 RTP 推流完全独立。
 *
 * Camera1：
 *   /root/var/video/camera1/
 *
 * Camera2：
 *   /root/var/video/camera2/
 *
 * 每 10 分钟生成一个 MP4。
 *
 * 文件名：
 *   camera1-00000.mp4
 *   camera1-00001.mp4
 *   ...
 *
 * Camera2 同理。
 */
std::string G20VideoAdapter::make_recording_command(
    int camera) const {
  const int width =
      getenv_int_range(
          "G20_VIDEO_WIDTH",
          config_.width,
          160,
          3840);

  const int height =
      getenv_int_range(
          "G20_VIDEO_HEIGHT",
          config_.height,
          120,
          2160);

  const int fps =
      getenv_int_range(
          "G20_VIDEO_FPS",
          config_.fps,
          1,
          60);

  const int bps =
      getenv_int_range(
          "G20_VIDEO_BPS",
          config_.bps,
          200000,
          50000000);

  const int gop =
      getenv_int_range(
          "G20_VIDEO_GOP",
          config_.gop,
          1,
          120);

  const int qp_init =
      getenv_int_range(
          "G20_VIDEO_QP_INIT",
          config_.qp_init,
          1,
          51);

  const std::string record_dir =
      "/root/var/video/camera" +
      std::to_string(camera);

  const std::string record_pattern =
      record_dir +
      "/camera" +
      std::to_string(camera) +
      "-%05d.mp4";

  std::ostringstream command;

  command
      << "mkdir -p "
      << shell_quote(record_dir)
      << " && "

      << "exec gst-launch-1.0 -q "

      << "v4l2src device="
      << shell_quote(
          camera_device(camera))
      << " do-timestamp=true "

      << "! image/jpeg,width="
      << width
      << ",height="
      << height
      << ",framerate="
      << fps
      << "/1 "

      << "! queue leaky=upstream "

      << "! mppjpegdec "

      << "! queue leaky=upstream "

      << "! mpph264enc bps="
      << bps
      << " rc-mode=cbr gop="
      << gop
      << " qp-init="
      << qp_init

      << " "

      << "! h264parse "

      << "! splitmuxsink "
      << "location="
      << shell_quote(record_pattern)
      << " "
      << "max-size-time=600000000000 "
      << "muxer-factory=mp4mux "
      << "async-finalize=true";

  return command.str();
}

std::string G20VideoAdapter::command_for_camera(
    int camera) const {
  std::string command =
      camera == 1
          ? config_.start_command_camera1
          : config_.start_command_camera2;

  if (command.empty()) {
    command = config_.command_template;
  }

  if (command.empty()) {
    return make_default_command(camera);
  }

  /*
   * 保持原来的自定义 command 行为。
   *
   * 如果配置文件明确提供了 G20 推流命令，
   * 仍然优先使用配置里的命令。
   */
  replace_all(
      command,
      "{camera}",
      std::to_string(camera));

  replace_all(
      command,
      "{device}",
      camera_device(camera));

  replace_all(
      command,
      "{target}",
      target_address());

  replace_all(
      command,
      "{port}",
      std::to_string(
          udp_port(camera)));

  replace_all(
      command,
      "{width}",
      std::to_string(
          config_.width));

  replace_all(
      command,
      "{height}",
      std::to_string(
          config_.height));

  replace_all(
      command,
      "{fps}",
      std::to_string(
          config_.fps));

  replace_all(
      command,
      "{bps}",
      std::to_string(
          config_.bps));

  replace_all(
      command,
      "{gop}",
      std::to_string(
          config_.gop));

  replace_all(
      command,
      "{qp_init}",
      std::to_string(
          config_.qp_init));

  return command;
}

/*
 * ============================================================
 * G20 推流启动
 * ============================================================
 */
bool G20VideoAdapter::start_one(
    int camera) {
#if defined(__linux__)

  int& pid =
      camera == 1
          ? camera1_pid_
          : camera2_pid_;

  int& process_group_id =
      camera == 1
          ? camera1_process_group_id_
          : camera2_process_group_id_;

  if (process_alive(pid)) {
    return true;
  }

  if (process_group_alive(
          process_group_id)) {
    stop_one(
        pid,
        process_group_id);
  }

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(1200);

  std::string device =
      camera_device(camera);

  while (!usable_capture_device(device) &&
         std::chrono::steady_clock::now() <
             deadline) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));

    device =
        camera_device(camera);
  }

  if (!usable_capture_device(device)) {
    set_status(
        "camera" +
        std::to_string(camera) +
        " device unavailable: " +
        device);

    return false;
  }

  const auto target =
      target_address();

  if (target.empty()) {
    set_status(
        "video target address is empty");

    return false;
  }

  const std::string command =
      command_for_camera(camera);

  const pid_t child =
      ::fork();

  if (child < 0) {
    set_status(
        "fork failed for camera" +
        std::to_string(camera));

    return false;
  }

  if (child == 0) {
    ::setpgid(0, 0);

    ::execlp(
        "sh",
        "sh",
        "-c",
        command.c_str(),
        static_cast<char*>(nullptr));

    _exit(127);
  }

  process_group_id =
      static_cast<int>(child);

  static_cast<void>(
      ::setpgid(child, child));

  pid =
      static_cast<int>(child);

  std::string detail;

  if (!child_still_running_after_start(
          pid,
          camera,
          device,
          &detail)) {
    process_group_id = -1;

    set_status(
        detail.empty()
            ? "camera stream exited immediately"
            : detail);

    return false;
  }

  std::ostringstream message;

  message
      << "camera"
      << camera
      << " stream started device="
      << device
      << " target="
      << target
      << ":"
      << udp_port(camera)
      << " pid="
      << pid
      << " pgid="
      << process_group_id;

  set_status(message.str());

  return true;

#else

  (void)camera;

  set_status(
      "video adapter requires Linux");

  return false;

#endif
}

/*
 * ============================================================
 * 本地录像启动
 * ============================================================
 */
bool G20VideoAdapter::start_recording_one(
    int camera) {
#if defined(__linux__)

  int& pid =
      camera == 1
          ? camera1_recording_pid_
          : camera2_recording_pid_;

  int& process_group_id =
      camera == 1
          ? camera1_recording_process_group_id_
          : camera2_recording_process_group_id_;

  if (process_alive(pid)) {
    return true;
  }

  if (process_group_alive(
          process_group_id)) {
    stop_recording_one(
        pid,
        process_group_id);
  }

  const std::string device =
      camera_device(camera);

  if (!usable_capture_device(device)) {
    set_status(
        "camera" +
        std::to_string(camera) +
        " recording device unavailable: " +
        device);

    return false;
  }

  const std::string command =
      make_recording_command(camera);

  const pid_t child =
      ::fork();

  if (child < 0) {
    set_status(
        "fork failed for camera" +
        std::to_string(camera) +
        " recording");

    return false;
  }

  if (child == 0) {
    ::setpgid(0, 0);

    ::execlp(
        "sh",
        "sh",
        "-c",
        command.c_str(),
        static_cast<char*>(nullptr));

    _exit(127);
  }

  process_group_id =
      static_cast<int>(child);

  static_cast<void>(
      ::setpgid(child, child));

  pid =
      static_cast<int>(child);

  /*
   * 录像进程只检查启动是否成功。
   *
   * 它不会影响 G20 RTP 状态。
   */
  std::this_thread::sleep_for(
      std::chrono::milliseconds(300));

  int status = 0;

  const pid_t waited =
      ::waitpid(
          pid,
          &status,
          WNOHANG);

  if (waited == pid) {
    pid = -1;
    process_group_id = -1;

    set_status(
        "camera" +
        std::to_string(camera) +
        " recording exited immediately");

    return false;
  }

  if (waited < 0 &&
      errno == ECHILD) {
    pid = -1;
    process_group_id = -1;

    set_status(
        "camera" +
        std::to_string(camera) +
        " recording child disappeared");

    return false;
  }

  return true;

#else

  (void)camera;

  return false;

#endif
}

/*
 * ============================================================
 * 启动两路本地录像
 * ============================================================
 */
void G20VideoAdapter::start_recording_all() {
#if defined(__linux__)

  /*
   * 本地录像总开关。
   *
   * 默认 1（开启），不设置环境变量时行为与本版本完全一致；
   * 设为 0 时完全不启动本地 MP4 录像，G20 实时推流不受影响。
   *
   * 用于现场 A/B 验证本地录像负载与长时间运行稳定性的关系。
   */
  if (!getenv_flag("G20_VIDEO_RECORD", true)) {
    set_status(
        "local camera recording disabled by G20_VIDEO_RECORD");

    return;
  }

  /*
   * 录像独立于 G20 RTP。
   *
   * 两个摄像头都尝试启动录像。
   *
   * 即使其中一个录像失败，
   * 也不影响 G20 推流。
   */
  const bool camera1_ok =
      start_recording_one(1);

  const bool camera2_ok =
      start_recording_one(2);

  if (camera1_ok && camera2_ok) {
    set_status(
        "camera1/camera2 local recording running");
  } else if (camera1_ok) {
    set_status(
        "camera1 local recording running; "
        "camera2 recording unavailable");
  } else if (camera2_ok) {
    set_status(
        "camera2 local recording running; "
        "camera1 recording unavailable");
  } else {
    set_status(
        "local camera recording unavailable");
  }

#endif
}

/*
 * ============================================================
 * 停止两路本地录像
 * ============================================================
 */
void G20VideoAdapter::stop_recording_all()
    noexcept {
  stop_recording_one(
      camera1_recording_pid_,
      camera1_recording_process_group_id_);

  stop_recording_one(
      camera2_recording_pid_,
      camera2_recording_process_group_id_);
}

/*
 * ============================================================
 * 停止单路 G20 RTP
 * ============================================================
 */
void G20VideoAdapter::stop_one(
    int& pid,
    int& process_group_id) noexcept {
#if defined(__linux__)

  if (pid <= 0 &&
      process_group_id <= 0) {
    return;
  }

  if (process_group_id > 0 &&
      process_group_alive(
          process_group_id)) {
    ::kill(
        -process_group_id,
        SIGINT);
  } else if (
      pid > 0 &&
      process_alive(pid)) {
    ::kill(
        pid,
        SIGINT);
  }

  for (int attempt = 0;
       attempt < 30;
       ++attempt) {
    bool child_done =
        pid <= 0;

    bool group_done =
        process_group_id <= 0;

    if (!child_done) {
      int status = 0;

      const pid_t waited =
          ::waitpid(
              pid,
              &status,
              WNOHANG);

      child_done =
          waited == pid ||
          (waited < 0 &&
           errno == ECHILD);
    }

    if (!group_done) {
      group_done =
          !process_group_alive(
              process_group_id);
    }

    if (child_done &&
        group_done) {
      pid = -1;
      process_group_id = -1;
      return;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));
  }

  if (process_group_id > 0 &&
      process_group_alive(
          process_group_id)) {
    ::kill(
        -process_group_id,
        SIGKILL);
  } else if (
      pid > 0 &&
      process_alive(pid)) {
    ::kill(
        pid,
        SIGKILL);
  }

  if (pid > 0) {
    static_cast<void>(
        ::waitpid(
            pid,
            nullptr,
            0));
  }

  pid = -1;
  process_group_id = -1;

#else

  (void)pid;
  (void)process_group_id;

#endif
}

/*
 * ============================================================
 * 停止单路本地录像
 * ============================================================
 */
void G20VideoAdapter::stop_recording_one(
    int& pid,
    int& process_group_id) noexcept {
#if defined(__linux__)

  if (pid <= 0 &&
      process_group_id <= 0) {
    return;
  }

  /*
   * 使用 SIGINT，让 splitmuxsink 有机会
   * 正常关闭当前 MP4 文件。
   */
  if (process_group_id > 0 &&
      process_group_alive(
          process_group_id)) {
    ::kill(
        -process_group_id,
        SIGINT);
  } else if (
      pid > 0 &&
      process_alive(pid)) {
    ::kill(
        pid,
        SIGINT);
  }

  for (int attempt = 0;
       attempt < 30;
       ++attempt) {
    bool child_done =
        pid <= 0;

    bool group_done =
        process_group_id <= 0;

    if (!child_done) {
      int status = 0;

      const pid_t waited =
          ::waitpid(
              pid,
              &status,
              WNOHANG);

      child_done =
          waited == pid ||
          (waited < 0 &&
           errno == ECHILD);
    }

    if (!group_done) {
      group_done =
          !process_group_alive(
              process_group_id);
    }

    if (child_done &&
        group_done) {
      pid = -1;
      process_group_id = -1;
      return;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(100));
  }

  /*
   * 正常退出超时才强制杀掉。
   */
  if (process_group_id > 0 &&
      process_group_alive(
          process_group_id)) {
    ::kill(
        -process_group_id,
        SIGKILL);
  } else if (
      pid > 0 &&
      process_alive(pid)) {
    ::kill(
        pid,
        SIGKILL);
  }

  if (pid > 0) {
    static_cast<void>(
        ::waitpid(
            pid,
            nullptr,
            0));
  }

  pid = -1;
  process_group_id = -1;

#else

  (void)pid;
  (void)process_group_id;

#endif
}

/*
 * ============================================================
 * 停止 G20 推流
 * ============================================================
 *
 * 注意：
 * 本地录像也停止。
 *
 * 这样在程序 shutdown / stop 时不会遗留
 * 两个后台 gst-launch 进程。
 */
void G20VideoAdapter::stop_stream_impl()
    noexcept {
  /*
   * 先停止 G20 RTP。
   *
   * 与原程序完全一致：
   * 两个 slot 都清理一次。
   */
  stop_one(
      camera1_pid_,
      camera1_process_group_id_);

  stop_one(
      camera2_pid_,
      camera2_process_group_id_);

  /*
   * 新增：
   * 停止本地录像。
   */
  stop_recording_all();

  active_camera_ = 1;

  set_status(
      "RTP video stream stopped");
}

/*
 * ============================================================
 * 启动视频
 * ============================================================
 */
bool G20VideoAdapter::start_stream_impl() {
#if defined(__linux__)

  /*
   * 保留原程序行为：
   *
   * 启动时先清理所有 G20 stream。
   */
  stop_one(
      camera1_pid_,
      camera1_process_group_id_);

  stop_one(
      camera2_pid_,
      camera2_process_group_id_);

  std::this_thread::sleep_for(
      std::chrono::milliseconds(150));

  active_camera_ = 1;

  /*
   * 原来的 G20 推流：
   * 只启动 camera1。
   */
  if (!start_one(active_camera_)) {
    set_status(
        "camera1 RTP video stream start failed");

    /*
     * 即使 G20 推流失败，
     * 也尝试启动本地录像。
     */
    start_recording_all();

    return false;
  }

  set_status(
      "camera1 RTP video stream running");

  /*
   * 新增：
   * G20 camera1 推流成功以后，
   * 启动两个摄像头的独立录像。
   *
   * 这里不会启动 camera2 的 RTP。
   */
  start_recording_all();

  return true;

#else

  set_status(
      "video adapter requires Linux");

  return false;

#endif
}

/*
 * ============================================================
 * G20 摄像头切换
 * ============================================================
 *
 * 这是最重要的地方：
 *
 * 原来的切换逻辑保持不变。
 *
 * Camera1 → Camera2：
 *
 *   stop camera1 RTP
 *        ↓
 *   等待 150ms
 *        ↓
 *   start camera2 RTP
 *
 * 本地录像进程完全不参与这里的切换。
 */
bool G20VideoAdapter::switch_stream_impl(
    int requested_camera) {
  if (requested_camera != 1 &&
      requested_camera != 2) {
    set_status(
        "invalid active camera");

    return false;
  }

#if defined(__linux__)

  const bool camera1_running =
      process_alive(camera1_pid_) ||
      process_group_alive(
          camera1_process_group_id_);

  const bool camera2_running =
      process_alive(camera2_pid_) ||
      process_group_alive(
          camera2_process_group_id_);

  if (!camera1_running &&
      !camera2_running) {
    set_status(
        "cannot switch camera while video stopped");

    return false;
  }

  const int previous_camera =
      active_camera_;

  if (requested_camera ==
      previous_camera) {
    set_status(
        "camera" +
        std::to_string(
            requested_camera) +
        " already active");

    return true;
  }

  /*
   * ========================================================
   * 保留原程序的核心逻辑：
   *
   * 同一时间严格只允许一个 G20 摄像头 pipeline。
   *
   * 注意：
   * 这里停止的是 camera1_pid_ /
   * camera2_pid_。
   *
   * 不停止：
   *
   * camera1_recording_pid_
   * camera2_recording_pid_
   *
   * 因此本地录像不会因为切换屏幕画面而停止。
   * ========================================================
   */
  stop_one(
      camera1_pid_,
      camera1_process_group_id_);

  stop_one(
      camera2_pid_,
      camera2_process_group_id_);

  std::this_thread::sleep_for(
      std::chrono::milliseconds(150));

  /*
   * 启动新的 G20 RTP。
   */
  if (start_one(requested_camera)) {
    active_camera_ =
        requested_camera;

    set_status(
        "active camera switched to camera" +
        std::to_string(
            active_camera_));

    /*
     * 本地录像完全不需要重新启动。
     *
     * camera1 / camera2 recording PID
     * 在整个切换过程中保持不变。
     */

    return true;
  }

  /*
   * 如果目标摄像头启动失败，
   * 保持原来的恢复逻辑。
   */
  active_camera_ =
      previous_camera;

  if (start_one(previous_camera)) {
    set_status(
        "camera" +
        std::to_string(
            requested_camera) +
        " start failed; restored camera" +
        std::to_string(
            previous_camera));
  } else {
    set_status(
        "camera switch failed; no video stream running");
  }

  return false;

#else

  set_status(
      "video adapter requires Linux");

  return false;

#endif
}

}  // namespace snappy::gateway::g20