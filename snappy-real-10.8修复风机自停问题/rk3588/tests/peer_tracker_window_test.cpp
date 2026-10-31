/**
 * @file peer_tracker_window_test.cpp
 * @brief 掉线确认窗口回归测试
 */

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "snappy/gateway/peer_tracker.hpp"

namespace {

using namespace std::chrono_literals;

using PeerTracker = snappy::gateway::PeerTracker;

int failures = 0;

void expect(bool ok, const std::string& what) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what.c_str());

  if (!ok) {
    ++failures;
  }
}

void nap(std::chrono::milliseconds ms) {
  std::this_thread::sleep_for(ms);
}


constexpr auto kConfirm = std::chrono::milliseconds(800);
constexpr auto kTimeout = std::chrono::milliseconds(75);

std::chrono::milliseconds since(
    std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0);
}

/* 忙等到时间轴上的目标时刻，避免 sleep 抖动累积。 */
void wait_until(
    std::chrono::steady_clock::time_point t0,
    std::chrono::milliseconds target) {
  while (since(t0) < target) {
    nap(1ms);
  }
}

void scenario_back_to_back_carrier_loss() {
  std::printf("场景 1：背靠背两次掉载波 + 对端额外静默\n");

  PeerTracker peer;
  const auto t0 = std::chrono::steady_clock::now();

  peer.observe("192.168.144.11", 14550);

  wait_until(t0, 29ms);
  peer.set_link_available(false);

  wait_until(t0, 232ms);
  peer.set_link_available(true);

  wait_until(t0, 336ms);
  peer.set_link_available(false);

  wait_until(t0, 533ms);
  peer.set_link_available(true);

  wait_until(t0, 801ms);

  const auto raw = peer.silence();
  const auto excluded = peer.link_unavailable_total();

  std::printf("    原始静默=%lldms  链路不可用累计=%lldms  确认窗口=%lldms\n",
      static_cast<long long>(raw.count()),
      static_cast<long long>(excluded.count()),
      static_cast<long long>(kConfirm.count()));

  expect(raw >= kConfirm, "原始墙钟静默已越过确认窗口（旧实现正是在此误判为掉线）");

  expect(excluded >= 350ms && excluded <= 470ms, "链路不可用时长被正确核销为约 400ms（2025+1967 按 1:10）");

  bool declared_down = false;

  for (int i = 0; i < 16; ++i) {
    if (!peer.online_confirmed(kTimeout, kConfirm)) {
      declared_down = true;
      break;
    }

    nap(5ms);
  }

  expect(!declared_down, "判定点之后 80ms 内始终判为在线 -> 整机不会被误停");
}

void scenario_link_up_peer_dead() {
  std::printf("场景 2：链路正常、对端真死（安全零降级）\n");

  PeerTracker peer;
  const auto t0 = std::chrono::steady_clock::now();

  peer.observe("192.168.144.11", 14550);

  bool declared_down = false;
  std::chrono::milliseconds declared_at{0};

  while (since(t0) < 950ms) {
    peer.set_link_available(true);

    if (!peer.online_confirmed(kTimeout, kConfirm)) {
      declared_down = true;
      declared_at = since(t0);
      break;
    }

    nap(5ms);
  }

  std::printf("    判定时刻≈%lldms（确认窗口 %lldms）\n",
              static_cast<long long>(declared_at.count()),
              static_cast<long long>(kConfirm.count()));

  expect(peer.link_unavailable_total().count() == 0,
         "链路一直可用时核销量为 0");

  expect(declared_down, "仍然判定掉线");

  expect(declared_down && declared_at < 900ms,
         "判定时刻仍在确认窗口附近，没有变晚");
}

void scenario_sustained_link_down() {
  std::printf("场景 3：链路持续不可用（真实断缆）\n");

  PeerTracker peer;
  const auto t0 = std::chrono::steady_clock::now();

  peer.observe("192.168.144.11", 14550);
  peer.set_link_available(false);

  /* 第一阶段：raw 刚越过确认窗口。 */
  bool early_down = false;

  while (since(t0) < 1100ms) {
    peer.set_link_available(false);

    if (!peer.online_confirmed(kTimeout, kConfirm)) {
      early_down = true;
      break;
    }

    nap(5ms);
  }

  const auto raw = peer.silence();
  const auto excluded = peer.link_unavailable_total();

  std::printf("    raw=%lldms  核销=%lldms\n",
              static_cast<long long>(raw.count()),
              static_cast<long long>(excluded.count()));

  expect(excluded + 50ms >= raw,
         "链路一直不可用时，核销量跟随原始静默一起增长");

  expect(!early_down,
         "raw 刚越过确认窗口时静默判据不触发，让位给 6s 的路由判据");

  /* 第二阶段：验证封顶带来的硬上界 2 x confirm。 */
  bool late_down = false;
  std::chrono::milliseconds late_at{0};

  while (since(t0) < 2200ms) {
    peer.set_link_available(false);

    if (!peer.online_confirmed(kTimeout, kConfirm)) {
      late_down = true;
      late_at = since(t0);
      break;
    }

    nap(5ms);
  }

  std::printf("    二次判定时刻≈%lldms（2 x 确认窗口 = %lldms）\n",
              static_cast<long long>(late_at.count()),
              static_cast<long long>((kConfirm * 2).count()));

  expect(late_down,
         "raw 越过 2 x 确认窗口后必然判定掉线（封顶给出的硬上界）");

  expect(late_down && late_at < 2000ms,
         "该上界不超过 2 x 确认窗口 + 采样余量");
}

/*
 * 场景 4：短暂掉载波之后链路恢复，对端却再也不发包。
 * 核销量固定为那 400ms，因此判定时刻应从 800ms 推迟到约 1200ms
 * （= 2 x 确认窗口）。这条断言给出「最坏延迟」的硬上界。
 */
void scenario_wall_clock_ceiling() {
  std::printf("场景 4：掉载波 400ms 后恢复，对端再也不发包\n");

  PeerTracker peer;
  const auto t0 = std::chrono::steady_clock::now();

  peer.observe("192.168.144.11", 14550);

  peer.set_link_available(false);
  wait_until(t0, 400ms);
  peer.set_link_available(true);

  const auto excluded = peer.link_unavailable_total();

  bool declared_down = false;
  std::chrono::milliseconds declared_at{0};

  while (since(t0) < 1500ms) {
    peer.set_link_available(true);

    if (!peer.online_confirmed(kTimeout, kConfirm)) {
      declared_down = true;
      declared_at = since(t0);
      break;
    }

    nap(5ms);
  }

  std::printf(
      "    核销=%lldms  判定时刻≈%lldms（期望 800 + 400 = 1200ms）\n",
      static_cast<long long>(excluded.count()),
      static_cast<long long>(declared_at.count()));

  expect(excluded >= 380ms && excluded <= 430ms,
         "核销量就是那 400ms 的掉载波区间");

  expect(declared_down, "最终仍然判定掉线");

  expect(declared_down && declared_at < 1400ms,
         "最坏判定延迟不超过 2 x 确认窗口（硬上界）");
}

}  // namespace

int main() {
  std::printf("PeerTracker 掉线确认窗口回归测试\n");
  std::printf(
      "确认窗口=%lldms  租约=%lldms（由 8000/750 按 1:10 压缩）\n\n",
      static_cast<long long>(kConfirm.count()),
      static_cast<long long>(kTimeout.count()));

  scenario_back_to_back_carrier_loss();
  std::printf("\n");

  scenario_link_up_peer_dead();
  std::printf("\n");

  scenario_sustained_link_down();
  std::printf("\n");

  scenario_wall_clock_ceiling();

  std::printf("\n%s（failures=%d）\n",
              failures == 0 ? "ALL PASS" : "HAS FAILURES",
              failures);

  return failures == 0 ? 0 : 1;
}
