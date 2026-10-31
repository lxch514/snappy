/**
 * @file command_mailbox_test.cpp
 * @brief CommandMailbox 最新值覆盖和安全停止测试
 */

#include "snappy_snappy/command_mailbox.hpp"

#include <cassert>
#include <chrono>

int main() {
  using namespace snappy_snappy;

  CommandMailbox mailbox;

  PendingCommand first;
  first.generation = 10;
  first.speed_raw = 3;

  PendingCommand latest;
  latest.generation = 11;
  latest.speed_raw = 15;

  /*
   * 同一执行器域只保留最新命令。
   */
  assert(
      mailbox.submit(
          CommandSlot::kBrush,
          first));

  assert(
      mailbox.submit(
          CommandSlot::kBrush,
          latest));

  const auto brush = mailbox.take(CommandSlot::kBrush);

  assert(brush.has_value());
  assert(brush->generation == 11);
  assert(brush->speed_raw == 15);

  assert(
      !mailbox.take(
          CommandSlot::kBrush)
           .has_value());

  /*
   * 不同执行器域互不覆盖。
   */
  PendingCommand climber;
  climber.generation = 12;
  climber.direction = 1;
  climber.speed_rpm = 500.0;

  PendingCommand push;
  push.generation = 12;
  push.direction = -1;
  push.timed = true;
  push.duration_ms = 1000;

  assert(
      mailbox.submit(
          CommandSlot::kClimber,
          climber));

  assert(
      mailbox.submit(
          CommandSlot::kBrushPush,
          push));

  assert(
      mailbox.take(
          CommandSlot::kClimber)
          .has_value());

  assert(
      mailbox.take(
          CommandSlot::kBrushPush)
          .has_value());

  /*
   * 紧急停止必须清除所有尚未执行槽。
   */
  assert(
      mailbox.submit(
          CommandSlot::kBrush,
          first));

  assert(
      mailbox.submit(
          CommandSlot::kScraperPush,
          push));

  mailbox.request_urgent_stop();

  assert(mailbox.take_urgent_stop());

  assert(!mailbox.take_urgent_stop());

  assert(
      !mailbox.take(
          CommandSlot::kBrush)
           .has_value());

  assert(
      !mailbox.take(
          CommandSlot::kScraperPush)
           .has_value());

  /*
   * close 后不再接受命令，
   * wait_take 也不会阻塞等待。
   */
  mailbox.close();

  assert(
      !mailbox.submit(
          CommandSlot::kBrush,
          latest));

  PendingCommand output;

  assert(
      !mailbox.wait_take(
          CommandSlot::kBrush,
          &output,
          std::chrono::milliseconds(1)));

  return 0;
}