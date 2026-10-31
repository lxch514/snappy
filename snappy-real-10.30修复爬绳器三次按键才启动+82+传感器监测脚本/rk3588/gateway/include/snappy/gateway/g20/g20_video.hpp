/**
 * @file g20_video.hpp
 * @brief G20 摄像头 RTP 推流控制接口
 *
 * @details
 * - 摄像头 G20 推流仍然由独立 worker 线程管理。
 * - gateway 主循环只投递启动、停止、切换命令。
 * - 视频状态通过缓存快照读取。
 * - G20 推流严格保持“同一时间最多一个摄像头”。
 * - 增加 camera1 / camera2 独立本地录像。
 * - 本地录像不参与 G20 摄像头切换。
 * - 摄像头异常不会阻塞 control、传感器或 RS485 通信。
 */

#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "snappy/gateway/interfaces.hpp"

namespace snappy::gateway::g20 {

struct G20VideoConfig {
  std::string path{"rk_usb_camera_dual_rtp_udp_to_g20_android"};

  std::string target_address{"192.168.144.11"};
  std::string target_env{"G20_VIDEO_TARGET"};

  std::string camera1_device{"/dev/video81"};
  std::string camera2_device{"/dev/video83"};

  std::string camera1_env{"G20_VIDEO_DEVICE_1"};
  std::string camera2_env{"G20_VIDEO_DEVICE_2"};
  std::string legacy_device_env{"G20_VIDEO_DEVICE"};

  std::uint16_t camera1_udp_port{5600};
  std::uint16_t camera2_udp_port{5602};

  std::string camera1_play_url{"udp://@:5600"};
  std::string camera2_play_url{"udp://@:5602"};

  int width{640};
  int height{480};
  int fps{30};
  int bps{4000000};
  int gop{15};
  int qp_init{24};

  bool auto_start_when_target_known{false};

  std::string start_command_camera1;
  std::string start_command_camera2;
  std::string command_template;
};

class G20VideoAdapter final : public IVideoAdapter {
 public:
  explicit G20VideoAdapter(G20VideoConfig config = {});
  ~G20VideoAdapter() override;

  G20VideoAdapter(const G20VideoAdapter&) = delete;
  G20VideoAdapter& operator=(const G20VideoAdapter&) = delete;

  [[nodiscard]] bool available() const noexcept override;

  bool start_g20_stream() override;
  bool switch_g20_camera(int active_camera) override;
  void stop() noexcept override;

  [[nodiscard]] VideoStreamState video_state() const override;
  [[nodiscard]] bool running() const noexcept;
  [[nodiscard]] std::string status() const;

 private:
  enum class VideoCommandType {
    kStart,
    kStop,
    kSwitch,
    kShutdown,
  };

  struct VideoCommand {
    VideoCommandType type{VideoCommandType::kStart};
    int camera{1};
  };

  [[nodiscard]] bool enqueue_command(
      VideoCommand command) noexcept;

  void worker_loop() noexcept;

  bool start_stream_impl();
  bool switch_stream_impl(int active_camera);
  void stop_stream_impl() noexcept;

  void update_snapshot();
  void set_status(const std::string& status);

  [[nodiscard]] std::string target_address() const;
  [[nodiscard]] std::string camera_device(int camera) const;
  [[nodiscard]] std::string play_url(int camera) const;
  [[nodiscard]] std::uint16_t udp_port(int camera) const;

  /*
   * G20 原有推流命令。
   *
   * 注意：
   * 这里不再加入本地录像。
   * 保持原来的单路 RTP pipeline。
   */
  [[nodiscard]] std::string command_for_camera(
      int camera) const;

  [[nodiscard]] std::string make_default_command(
      int camera) const;

  /*
   * 新增：独立本地录像 pipeline。
   */
  [[nodiscard]] std::string make_recording_command(
      int camera) const;

  bool start_one(int camera);

  /*
   * 新增：启动 / 停止单路本地录像。
   */
  bool start_recording_one(int camera);
  void stop_recording_one(
      int& pid,
      int& process_group_id) noexcept;

  void start_recording_all();
  void stop_recording_all() noexcept;

#if defined(__linux__)
  void stop_one(
      int& pid,
      int& process_group_id) noexcept;
#else
  void stop_one(
      int& pid,
      int& process_group_id) noexcept;
#endif

  G20VideoConfig config_;

  /*
   * G20 推流进程。
   *
   * 与原程序保持一致：
   * 同一时间最多只启动其中一个。
   */
  int camera1_pid_{-1};
  int camera2_pid_{-1};

  int camera1_process_group_id_{-1};
  int camera2_process_group_id_{-1};

  /*
   * 本地录像进程。
   *
   * camera1_recording_pid_ 和
   * camera2_recording_pid_ 可以同时运行。
   *
   * 它们与 G20 推流进程完全独立。
   */
  int camera1_recording_pid_{-1};
  int camera2_recording_pid_{-1};

  int camera1_recording_process_group_id_{-1};
  int camera2_recording_process_group_id_{-1};

  int active_camera_{1};

  mutable std::mutex state_mutex_;
  std::condition_variable command_cv_;
  std::deque<VideoCommand> commands_;
  std::thread worker_thread_;

  bool worker_started_{false};
  bool worker_stop_requested_{false};

  VideoStreamState state_snapshot_;
  std::string last_status_{"video idle"};
};

}  // namespace snappy::gateway::g20