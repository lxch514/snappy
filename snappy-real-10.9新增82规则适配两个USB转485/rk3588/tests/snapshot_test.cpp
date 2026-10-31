/**
 * @file snapshot_test.cpp
 * @brief 执行器内存快照字段测试
 */

#include "snappy_snappy/actuator_snapshot.hpp"

#include <cassert>

int main() {
  using namespace snappy_snappy;

  ActuatorSnapshot snapshot;

  assert(snapshot.sequence == 0);
  assert(snapshot.updated_at_ms == 0);
  assert(!snapshot.started);
  assert(!snapshot.worker_running);
  assert(!snapshot.ready_for_arm);
  assert(!snapshot.command_gate_open);
  assert(!snapshot.safe_stop_pending);
  assert(!snapshot.rs485_ready);
  assert(!snapshot.can_ready);
  assert(!snapshot.fan_active);

  assert(snapshot.brush_state == "stopped");
  assert(snapshot.brush_push_state == "stopped");
  assert(snapshot.scraper_push_state == "stopped");
  assert(snapshot.push_state == "stopped");
  assert(snapshot.climber_state == "stopped");

  assert(snapshot.brush_speed_raw == 0);
  assert(snapshot.fan_left_raw == 0);
  assert(snapshot.fan_right_raw == 0);
  assert(snapshot.brush_push_remaining_ms == 0);
  assert(snapshot.scraper_push_remaining_ms == 0);

  /*
   * 快照是普通内存结构，
   * 更新字段不会触发任何物理通信。
   */
  snapshot.sequence = 7;
  snapshot.updated_at_ms = 1234;
  snapshot.started = true;
  snapshot.worker_running = true;
  snapshot.ready_for_arm = true;
  snapshot.command_gate_open = true;
  snapshot.fan_active = true;

  snapshot.brush_state = "forward";
  snapshot.brush_speed_raw = 15;
  snapshot.brush_push_state = "extending";
  snapshot.brush_push_remaining_ms = 800;
  snapshot.scraper_push_state = "retracting";
  snapshot.climber_state = "ascending";
  snapshot.climber_speed_rpm = 500;
  snapshot.fan_left_raw = 400;
  snapshot.fan_right_raw = 400;

  assert(snapshot.sequence == 7);
  assert(snapshot.updated_at_ms == 1234);
  assert(snapshot.started);
  assert(snapshot.worker_running);
  assert(snapshot.ready_for_arm);
  assert(snapshot.command_gate_open);
  assert(snapshot.fan_active);
  assert(snapshot.brush_state == "forward");
  assert(snapshot.brush_speed_raw == 15);
  assert(snapshot.brush_push_remaining_ms == 800);
  assert(snapshot.fan_left_raw == 400);
  assert(snapshot.fan_right_raw == 400);

  return 0;
}