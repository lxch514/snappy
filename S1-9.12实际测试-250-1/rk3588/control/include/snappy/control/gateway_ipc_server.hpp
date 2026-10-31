#pragma once

/**
 * @file gateway_ipc_server.hpp
 * @brief control 端有界多 worker Unix IPC 服务
 *
 * @details
 *
 * - accept、客户端收包、控制命令和快照请求分别处理。
 * - 慢客户端不会占住唯一的 IPC handler。
 * - 安全命令、普通命令和周期快照使用独立有界队列。
 * - 周期快照不会排在安全停止命令之前。
 * - 不为每个连接创建无上限线程。
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace snappy::control {

struct GatewayIpcServerConfig {
  std::string socket_path{
      "/run/snappy/control.sock"};

  std::size_t max_payload_bytes{
      64 * 1024};

  int listen_backlog{32};

  std::size_t max_pending_connections{32};
  std::size_t max_pending_critical{8};
  std::size_t max_pending_commands{16};
  std::size_t max_pending_snapshots{4};

  std::size_t reader_worker_count{2};
  std::size_t critical_worker_count{1};
  std::size_t command_worker_count{2};
  std::size_t snapshot_worker_count{1};

  std::chrono::milliseconds
      client_read_timeout{100};

  std::chrono::milliseconds
      client_write_timeout{100};
};

class GatewayIpcServer {
 public:
  using Handler =
      std::function<std::string(
          const std::string&)>;

  explicit GatewayIpcServer(
      GatewayIpcServerConfig config = {});

  ~GatewayIpcServer();

  GatewayIpcServer(
      const GatewayIpcServer&) = delete;

  GatewayIpcServer& operator=(
      const GatewayIpcServer&) = delete;

  bool start(Handler handler);
  void stop() noexcept;

  [[nodiscard]] bool running() const noexcept {
    return running_.load();
  }

  [[nodiscard]] const std::string&
  socket_path() const noexcept {
    return config_.socket_path;
  }

 private:
  enum class RequestClass {
    kCritical,
    kCommand,
    kSnapshot,
  };

  struct RequestJob {
    int client_fd{-1};
    std::string payload;
  };

  void accept_loop();
  void reader_loop();
  void handler_loop(RequestClass request_class);

  [[nodiscard]] bool enqueue_client(
      int client_fd);

  [[nodiscard]] std::optional<int>
  take_client();

  [[nodiscard]] bool enqueue_request(
      RequestClass request_class,
      RequestJob job);

  [[nodiscard]] std::optional<RequestJob>
  take_request(
      RequestClass request_class);

  [[nodiscard]] RequestClass classify_request(
      const std::string& payload) const;

  void close_server_socket() noexcept;
  void close_queued_clients_locked() noexcept;

  GatewayIpcServerConfig config_;
  Handler handler_;

  std::atomic_bool running_{false};
  std::atomic_int server_fd_{-1};

  mutable std::mutex lifecycle_mutex_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  std::deque<int> pending_clients_;
  std::deque<RequestJob> critical_requests_;
  std::deque<RequestJob> command_requests_;
  std::deque<RequestJob> snapshot_requests_;

  std::thread accept_thread_;
  std::vector<std::thread> reader_threads_;
  std::vector<std::thread> critical_threads_;
  std::vector<std::thread> command_threads_;
  std::vector<std::thread> snapshot_threads_;
};

}  // namespace snappy::control