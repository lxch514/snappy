#pragma once

/**
 * @file actuator_snapshot.hpp
 * @brief 无传感器执行器状态缓存快照
 *
 * @details
 * - 状态和遥测接口只读取本结构中的内存数据。
 * - 不在快照读取路径访问 RS485、CAN 或传感器。
 * - 状态值由 ActuatorService 和各设备工作线程更新。
 * - 保留原 push.* 兼容字段以及双风机状态字段。
 */

#include <cstdint>
#include <string>
#include <array>

namespace snappy_snappy {

struct Rs485DeviceStatus {
  bool online{false};

  std::uint64_t last_success{0};
  std::uint64_t age_ms{0};
  std::uint64_t quarantine_until{0};

  int consecutive_failures{0};
  std::string last_error;
};

struct ActuatorSnapshot {

  Rs485DeviceStatus relay_status;
  Rs485DeviceStatus brush_status;
  Rs485DeviceStatus climber_status;

  std::array<bool, 4> relay_outputs{};

  int brush_fault_raw{0};
  int climber_alarm_raw{0};
  int climber_actual_speed_raw{0};

  bool brush_push_ready{false};
  bool scraper_push_ready{false};

  std::uint64_t sequence{0};
  std::uint64_t updated_at_ms{0};

  bool started{false};
  bool worker_running{false};
  bool partial_start{false};
  bool ready_for_arm{false};

  bool command_gate_open{false};
  bool safe_stop_pending{false};

  /*
   * 总线和逻辑设备状态均为缓存值。
   * 查询这些字段不会触发任何物理通信。
   */
  bool rs485_ready{false};
  bool can_ready{false};

  bool relay_ready{false};
  bool brush_ready{false};
  bool climber_ready{false};
  bool fan_ready{false};

  bool fan_worker_running{false};
  bool fan_online{false};
  bool fan_active{false};

  std::string brush_state{"stopped"};
  int brush_speed_raw{0};

  std::string brush_push_state{"stopped"};
  int brush_push_remaining_ms{0};
  int brush_push_last_duration_ms{0};
  int brush_push_extension_mm{0};

  std::string scraper_push_state{"stopped"};
  int scraper_push_remaining_ms{0};
  int scraper_push_last_duration_ms{0};
  int scraper_push_extension_mm{0};

  std::string push_state{"stopped"};

  int push_extension_mm{0};

  std::string climber_state{"stopped"};
  int climber_speed_rpm{0};

  /*
   * 左右风机目标仍分别保留在协议快照中。
   */
  int fan_left_raw{0};
  int fan_right_raw{0};

  std::string last_error;
};

}  // namespace snappy_snappy