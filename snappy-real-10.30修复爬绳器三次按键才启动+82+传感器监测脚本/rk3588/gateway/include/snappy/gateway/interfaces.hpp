#pragma once
/**
 * @file interfaces.hpp
 * @brief 网关层抽象接口声明
 *
 * @details
 * - 定义传输、控制通道和视频适配三类边界。
 * - 收发消息结构与视频状态结构也在此定义。
 */
 
#include <cstddef>
#include <optional>
#include <string>

#include "snappy/control/model.hpp"

namespace snappy::gateway {

constexpr std::size_t kUdpPayloadLimit = 1200;
constexpr std::size_t kStreamPayloadLimit = 64 * 1024;

struct InboundMessage {
  std::string topic;
  std::string payload;
};

struct PublishedMessage {
  std::string topic;
  std::string payload;
  int qos{0};
  bool retain{false};
};

class ITransport {
 public:
  virtual ~ITransport() = default;
  [[nodiscard]] virtual control::LinkId link_id() const noexcept = 0;
  [[nodiscard]] virtual std::size_t payload_limit() const noexcept = 0;
  [[nodiscard]] virtual bool healthy() const noexcept = 0;
  virtual std::optional<InboundMessage> receive() = 0;
  virtual bool publish(const PublishedMessage& message) = 0;

  /*
   * 发送路径是否已被内核判定为不可达。
   *
   * 与"对端静默"是两件不同的事：
   *   - 对端静默（silence）既可能是链路断了，也可能只是抖动，
   *     所以必须给一个较长的确认窗口；
   *   - ENETUNREACH/ENETDOWN/EHOSTUNREACH 是内核明确告诉我们
   *     "这条路由现在不存在"，属于确定性故障，不需要等窗口。
   *
   * 掉载波（carrier down）时路由会被摘掉，sendto 立即返回
   * ENETUNREACH，因此这个信号比静默窗口快一个数量级。
   */
  [[nodiscard]] virtual bool unreachable() const noexcept {
    return false;
  }
};

class IControlChannel {
 public:
  virtual ~IControlChannel() = default;
  virtual std::optional<std::string> request(const std::string& command_json) = 0;

  /*
   * 返回 true 表示 control 已确认收到 link_down。
   *
   * 原来的返回类型是 void，调用方无法区分"已投递"和"被丢弃"，
   * 于是只能无条件把 was_g20_online 置 false，真实断链有可能
   * 永远停不下来。改成 bool 之后调用方必须处理失败并重试。
   */
  virtual bool notify_link_down(control::LinkId link_id) noexcept = 0;
};

struct VideoStreamState {
  bool running{false};
  int active_camera{1};
  std::string active_url;
  std::string camera1_url;
  std::string camera2_url;
  std::string target_address;
  std::uint16_t camera1_port{5600};
  std::uint16_t camera2_port{5602};
  std::string detail;
};

class IVideoAdapter {
 public:
  virtual ~IVideoAdapter() = default;
  [[nodiscard]] virtual bool available() const noexcept = 0;
  virtual bool start_g20_stream() = 0;
  virtual bool switch_g20_camera(int active_camera) = 0;
  virtual void stop() noexcept = 0;
  [[nodiscard]] virtual VideoStreamState video_state() const = 0;
};

}  // namespace snappy::gateway