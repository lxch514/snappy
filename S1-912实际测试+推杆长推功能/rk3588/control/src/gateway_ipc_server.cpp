#include "snappy/control/gateway_ipc_server.hpp"

/**
 * @file gateway_ipc_server.cpp
 * @brief control 端有界、分类、多 worker IPC 服务实现
 *
 * @details
 *
 * - accept 线程只接收连接并放入有界队列。
 * - reader worker 使用 deadline 读取完整 SEQPACKET。
 * - 安全命令、普通命令和快照请求进入不同 worker 池。
 * - 慢客户端、慢快照和普通设备命令不会占住唯一 handler。
 */

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#if defined(__linux__)
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#endif

namespace snappy::control {
namespace {

using json = nlohmann::json;

#if defined(__linux__)

using Deadline =
    std::chrono::steady_clock::time_point;

int remaining_ms(Deadline deadline) {
  const auto now =
      std::chrono::steady_clock::now();

  if (now >= deadline) {
    return 0;
  }

  const auto remaining =
      std::chrono::duration_cast<
          std::chrono::milliseconds>(
          deadline - now);

  return std::max(
      1,
      static_cast<int>(
          remaining.count()));
}

bool wait_fd(
    int fd,
    short events,
    Deadline deadline) {
  while (std::chrono::steady_clock::now() <
         deadline) {
    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = events;

    const int result =
        ::poll(
            &descriptor,
            1,
            remaining_ms(deadline));

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }

      return false;
    }

    if (result == 0) {
      return false;
    }

    if ((descriptor.revents & events) != 0) {
      return true;
    }

    if ((descriptor.revents &
         (POLLERR | POLLNVAL)) != 0) {
      return false;
    }

    if ((descriptor.revents & POLLHUP) != 0) {
      return false;
    }
  }

  return false;
}

void make_nonblocking_cloexec(int fd) {
  const int status_flags =
      ::fcntl(fd, F_GETFL, 0);

  if (status_flags >= 0) {
    static_cast<void>(
        ::fcntl(
            fd,
            F_SETFL,
            status_flags | O_NONBLOCK));
  }

  const int descriptor_flags =
      ::fcntl(fd, F_GETFD, 0);

  if (descriptor_flags >= 0) {
    static_cast<void>(
        ::fcntl(
            fd,
            F_SETFD,
            descriptor_flags | FD_CLOEXEC));
  }
}

std::optional<std::string> receive_packet(
    int fd,
    std::size_t max_payload_bytes,
    std::chrono::milliseconds timeout) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      timeout;

  if (!wait_fd(fd, POLLIN, deadline)) {
    return std::nullopt;
  }

  std::vector<char> buffer(
      max_payload_bytes + 1,
      '\0');

  while (std::chrono::steady_clock::now() <
         deadline) {
    const ssize_t received =
        ::recv(
            fd,
            buffer.data(),
            buffer.size(),
            MSG_DONTWAIT);

    if (received > 0) {
      const auto size =
          static_cast<std::size_t>(
              received);

      if (size > max_payload_bytes) {
        return std::nullopt;
      }

      return std::string(
          buffer.data(),
          size);
    }

    if (received == 0) {
      return std::nullopt;
    }

    if (errno == EINTR) {
      continue;
    }

    if (errno == EAGAIN ||
        errno == EWOULDBLOCK) {
      if (!wait_fd(fd, POLLIN, deadline)) {
        return std::nullopt;
      }

      continue;
    }

    return std::nullopt;
  }

  return std::nullopt;
}

bool send_packet(
    int fd,
    const std::string& payload,
    std::chrono::milliseconds timeout) {
  const auto deadline =
      std::chrono::steady_clock::now() +
      timeout;

  while (std::chrono::steady_clock::now() <
         deadline) {
    const ssize_t sent =
        ::send(
            fd,
            payload.data(),
            payload.size(),
            MSG_NOSIGNAL | MSG_DONTWAIT);

    if (sent ==
        static_cast<ssize_t>(
            payload.size())) {
      return true;
    }

    if (sent >= 0) {
      return false;
    }

    if (errno == EINTR) {
      continue;
    }

    if (errno == EAGAIN ||
        errno == EWOULDBLOCK) {
      if (!wait_fd(fd, POLLOUT, deadline)) {
        return false;
      }

      continue;
    }

    return false;
  }

  return false;
}

void close_client(int fd) noexcept {
  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
}

void send_busy_and_close(
    int fd,
    std::chrono::milliseconds timeout,
    const char* detail) noexcept {
  try {
    json response;

    response["schema_version"] = 1;
    response["kind"] = "ack";
    response["status"] = "FAILED";
    response["detail"] = detail;

    static_cast<void>(
        send_packet(
            fd,
            response.dump(),
            timeout));
  } catch (...) {
  }

  close_client(fd);
}

#endif

bool is_critical_action(
    const std::string& action) {
  return action == "control.estop" ||
         action == "control.safe_hold" ||
         action == "control.disarm" ||
         action == "control.release" ||
         action == "control.link_down" ||
         action == "brush.stop" ||
         action == "fan.stop" ||
         action == "climber.stop" ||
         action == "push.stop" ||
         action == "brush_push.stop" ||
         action == "scraper_push.stop";
}

bool is_snapshot_action(
    const std::string& action) {
  return
      action == "gateway.status_snapshot" ||
      action ==
          "gateway.telemetry_snapshot" ||
      action == "gateway.capabilities";
}

}  // namespace

GatewayIpcServer::GatewayIpcServer(
    GatewayIpcServerConfig config)
    : config_(std::move(config)) {}

GatewayIpcServer::~GatewayIpcServer() {
  stop();
}

/* ============================================================
 * 启动
 * ============================================================ */

bool GatewayIpcServer::start(
    Handler handler) {
  std::lock_guard<std::mutex> lifecycle_lock(
      lifecycle_mutex_);

  if (running_.load()) {
    return true;
  }

  handler_ = std::move(handler);

  if (!handler_) {
    std::cerr
        << "[control-ipc] missing request handler\n";
    return false;
  }

#if !defined(__linux__)
  std::cerr
      << "[control-ipc] Unix socket is only "
         "available on Linux\n";
  return false;
#else
  const auto slash =
      config_.socket_path.find_last_of('/');

  if (slash != std::string::npos) {
    const std::string directory =
        config_.socket_path.substr(
            0,
            slash);

    if (!directory.empty()) {
      static_cast<void>(
          ::mkdir(
              directory.c_str(),
              0755));
    }
  }

  const int server_fd =
      ::socket(
          AF_UNIX,
          SOCK_SEQPACKET |
              SOCK_CLOEXEC |
              SOCK_NONBLOCK,
          0);

  if (server_fd < 0) {
    std::cerr
        << "[control-ipc] socket failed: "
        << std::strerror(errno)
        << "\n";

    return false;
  }

  sockaddr_un address{};
  address.sun_family = AF_UNIX;

  if (config_.socket_path.size() >=
      sizeof(address.sun_path)) {
    std::cerr
        << "[control-ipc] socket path too long: "
        << config_.socket_path
        << "\n";

    ::close(server_fd);
    return false;
  }

  std::strncpy(
      address.sun_path,
      config_.socket_path.c_str(),
      sizeof(address.sun_path) - 1);

  static_cast<void>(
      ::unlink(
          config_.socket_path.c_str()));

  if (::bind(
          server_fd,
          reinterpret_cast<sockaddr*>(
              &address),
          sizeof(address)) != 0) {
    std::cerr
        << "[control-ipc] bind failed: "
        << std::strerror(errno)
        << "\n";

    ::close(server_fd);
    return false;
  }

  static_cast<void>(
      ::chmod(
          config_.socket_path.c_str(),
          0660));

  if (::listen(
          server_fd,
          std::max(
              1,
              config_.listen_backlog)) != 0) {
    std::cerr
        << "[control-ipc] listen failed: "
        << std::strerror(errno)
        << "\n";

    ::close(server_fd);

    static_cast<void>(
        ::unlink(
            config_.socket_path.c_str()));

    return false;
  }

  server_fd_.store(server_fd);
  running_.store(true);

  accept_thread_ =
      std::thread(
          [this]() {
            accept_loop();
          });

  const auto create_workers =
      [this](
          std::vector<std::thread>& threads,
          std::size_t count,
          auto function) {
        const std::size_t actual_count =
            std::max<std::size_t>(
                1,
                count);

        threads.reserve(actual_count);

        for (std::size_t index = 0;
             index < actual_count;
             ++index) {
          threads.emplace_back(function);
        }
      };

  create_workers(
      reader_threads_,
      config_.reader_worker_count,
      [this]() {
        reader_loop();
      });

  create_workers(
      critical_threads_,
      config_.critical_worker_count,
      [this]() {
        handler_loop(
            RequestClass::kCritical);
      });

  create_workers(
      command_threads_,
      config_.command_worker_count,
      [this]() {
        handler_loop(
            RequestClass::kCommand);
      });

  create_workers(
      snapshot_threads_,
      config_.snapshot_worker_count,
      [this]() {
        handler_loop(
            RequestClass::kSnapshot);
      });

  std::cout
      << "[control-ipc] listening on "
      << config_.socket_path
      << "\n";

  return true;
#endif
}

/* ============================================================
 * 停止
 * ============================================================ */

void GatewayIpcServer::stop() noexcept {
  std::lock_guard<std::mutex> lifecycle_lock(
      lifecycle_mutex_);

  const bool was_running =
      running_.exchange(false);

  close_server_socket();
  queue_cv_.notify_all();

  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }

  {
    std::lock_guard<std::mutex> queue_lock(
        queue_mutex_);

    close_queued_clients_locked();
  }

  queue_cv_.notify_all();

  const auto join_threads =
      [](std::vector<std::thread>& threads) {
        for (auto& thread : threads) {
          if (thread.joinable()) {
            thread.join();
          }
        }

        threads.clear();
      };

  join_threads(reader_threads_);
  join_threads(critical_threads_);
  join_threads(command_threads_);
  join_threads(snapshot_threads_);

#if defined(__linux__)
  if (was_running) {
    static_cast<void>(
        ::unlink(
            config_.socket_path.c_str()));
  }
#else
  (void)was_running;
#endif

  handler_ = {};
}

void GatewayIpcServer::close_server_socket()
    noexcept {
#if defined(__linux__)
  const int fd =
      server_fd_.exchange(-1);

  if (fd >= 0) {
    ::shutdown(fd, SHUT_RDWR);
    ::close(fd);
  }
#endif
}

void GatewayIpcServer::close_queued_clients_locked()
    noexcept {
#if defined(__linux__)
  for (const int fd : pending_clients_) {
    close_client(fd);
  }

  pending_clients_.clear();

  const auto close_jobs =
      [](std::deque<RequestJob>& jobs) {
        for (const auto& job : jobs) {
          close_client(job.client_fd);
        }

        jobs.clear();
      };

  close_jobs(critical_requests_);
  close_jobs(command_requests_);
  close_jobs(snapshot_requests_);
#endif
}

/* ============================================================
 * accept 线程
 * ============================================================ */

void GatewayIpcServer::accept_loop() {
#if defined(__linux__)
  while (running_.load()) {
    const int listen_fd =
        server_fd_.load();

    if (listen_fd < 0) {
      break;
    }

    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(100);

    if (!wait_fd(
            listen_fd,
            POLLIN,
            deadline)) {
      continue;
    }

    while (running_.load()) {
      const int client_fd =
          ::accept(
              listen_fd,
              nullptr,
              nullptr);

      if (client_fd < 0) {
        if (errno == EINTR) {
          continue;
        }

        if (errno == EAGAIN ||
            errno == EWOULDBLOCK) {
          break;
        }

        if (running_.load()) {
          std::cerr
              << "[control-ipc] accept failed: "
              << std::strerror(errno)
              << "\n";
        }

        break;
      }

      make_nonblocking_cloexec(client_fd);

      if (!enqueue_client(client_fd)) {
        send_busy_and_close(
            client_fd,
            config_.client_write_timeout,
            "IPC connection queue is full");
      }
    }
  }
#endif
}

/* ============================================================
 * 客户端连接队列
 * ============================================================ */

bool GatewayIpcServer::enqueue_client(
    int client_fd) {
  std::lock_guard<std::mutex> lock(
      queue_mutex_);

  if (!running_.load() ||
      pending_clients_.size() >=
          config_.max_pending_connections) {
    return false;
  }

  pending_clients_.push_back(
      client_fd);

  queue_cv_.notify_all();
  return true;
}

std::optional<int>
GatewayIpcServer::take_client() {
  std::unique_lock<std::mutex> lock(
      queue_mutex_);

  queue_cv_.wait(
      lock,
      [this]() {
        return !running_.load() ||
               !pending_clients_.empty();
      });

  if (!running_.load()) {
    return std::nullopt;
  }

  const int client_fd =
      pending_clients_.front();

  pending_clients_.pop_front();

  return client_fd;
}

/* ============================================================
 * reader worker
 * ============================================================ */

void GatewayIpcServer::reader_loop() {
#if defined(__linux__)
  while (running_.load()) {
    const auto client =
        take_client();

    if (!client.has_value()) {
      break;
    }

    const int client_fd = *client;

    const auto payload =
        receive_packet(
            client_fd,
            config_.max_payload_bytes,
            config_.client_read_timeout);

    if (!payload.has_value() ||
        payload->empty()) {
      send_busy_and_close(
          client_fd,
          config_.client_write_timeout,
          "invalid, oversized or timed-out IPC request");

      continue;
    }

    RequestJob job;
    job.client_fd = client_fd;
    job.payload = *payload;

    const auto request_class =
        classify_request(job.payload);

    if (!enqueue_request(
            request_class,
            std::move(job))) {
      send_busy_and_close(
          client_fd,
          config_.client_write_timeout,
          "IPC request queue is full");
    }
  }
#endif
}

/* ============================================================
 * 请求分类和队列
 * ============================================================ */

GatewayIpcServer::RequestClass
GatewayIpcServer::classify_request(
    const std::string& payload) const {
  try {
    const auto root =
        json::parse(payload);

    const std::string action =
        root.value(
            "action",
            "");

    if (is_critical_action(action)) {
      return RequestClass::kCritical;
    }

    if (is_snapshot_action(action)) {
      return RequestClass::kSnapshot;
    }
  } catch (...) {
    /*
     * 无效 JSON 进入普通命令队列，由 dispatcher 返回解析错误。
     */
  }

  return RequestClass::kCommand;
}

bool GatewayIpcServer::enqueue_request(
    RequestClass request_class,
    RequestJob job) {
  std::lock_guard<std::mutex> lock(
      queue_mutex_);

  if (!running_.load()) {
    return false;
  }

  std::deque<RequestJob>* queue = nullptr;
  std::size_t limit = 0;

  switch (request_class) {
    case RequestClass::kCritical:
      queue = &critical_requests_;
      limit = config_.max_pending_critical;
      break;

    case RequestClass::kCommand:
      queue = &command_requests_;
      limit = config_.max_pending_commands;
      break;

    case RequestClass::kSnapshot:
      queue = &snapshot_requests_;
      limit = config_.max_pending_snapshots;
      break;
  }

  if (queue == nullptr ||
      queue->size() >= limit) {
    return false;
  }

  queue->push_back(
      std::move(job));

  queue_cv_.notify_all();
  return true;
}

std::optional<GatewayIpcServer::RequestJob>
GatewayIpcServer::take_request(
    RequestClass request_class) {
  std::unique_lock<std::mutex> lock(
      queue_mutex_);

  const auto queue_not_empty =
      [this, request_class]() {
        switch (request_class) {
          case RequestClass::kCritical:
            return !critical_requests_.empty();
          case RequestClass::kCommand:
            return !command_requests_.empty();
          case RequestClass::kSnapshot:
            return !snapshot_requests_.empty();
        }

        return false;
      };

  queue_cv_.wait(
      lock,
      [this, &queue_not_empty]() {
        return !running_.load() ||
               queue_not_empty();
      });

  if (!running_.load()) {
    return std::nullopt;
  }

  std::deque<RequestJob>* queue = nullptr;

  switch (request_class) {
    case RequestClass::kCritical:
      queue = &critical_requests_;
      break;
    case RequestClass::kCommand:
      queue = &command_requests_;
      break;
    case RequestClass::kSnapshot:
      queue = &snapshot_requests_;
      break;
  }

  if (queue == nullptr ||
      queue->empty()) {
    return std::nullopt;
  }

  RequestJob job =
      std::move(queue->front());

  queue->pop_front();

  return job;
}

/* ============================================================
 * handler worker
 * ============================================================ */

void GatewayIpcServer::handler_loop(
    RequestClass request_class) {
#if defined(__linux__)
  while (running_.load()) {
    auto job =
        take_request(request_class);

    if (!job.has_value()) {
      break;
    }

    std::string response;

    try {
      response =
          handler_(job->payload);
    } catch (const std::exception& exception) {
      json failure;

      failure["schema_version"] = 1;
      failure["kind"] = "ack";
      failure["status"] = "FAILED";
      failure["detail"] =
          "control handler exception";

      response = failure.dump();

      std::cerr
          << "[control-ipc] handler exception: "
          << exception.what()
          << "\n";
    } catch (...) {
      response =
          R"({"schema_version":1,"kind":"ack","status":"FAILED","detail":"unknown control handler exception"})";
    }

    if (!response.empty()) {
      static_cast<void>(
          send_packet(
              job->client_fd,
              response,
              config_.client_write_timeout));
    }

    close_client(job->client_fd);
  }
#else
  (void)request_class;
#endif
}

}  // namespace snappy::control