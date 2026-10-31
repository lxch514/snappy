#pragma once

#include <cstdint>
#include <vector>

#include "snappy/drivers/protocols/modbus_rtu.hpp"

namespace snappy::drivers::protocols {

inline constexpr std::uint8_t kClimberL7PStationId = 0x02;

struct ClimberModbusProfile {
  // P00
  std::uint16_t control_mode_register{0x0003};        // P00.01
  std::uint16_t save_register{0x0033};                // P00.25
  std::uint16_t auxiliary_function_register{0x0033};  // P00.25
  std::uint16_t virtual_io_register{0x0035};           // P00.26

  // P03
  std::uint16_t speed_source_register{0x0301};        // P03.00
  std::uint16_t direction_register{0x0303};           // P03.01
  std::uint16_t target_speed_register{0x0309};        // P03.04
  std::uint16_t acceleration_register{0x0319};        // P03.12
  std::uint16_t deceleration_register{0x031B};        // P03.13

  // P05
  // P05.06 = 0: servo-brake stop mode.
  std::uint16_t stop_mode_register{0x050D};           // P05.06

  // P04
  std::uint16_t di1_function_register{0x0401};        // P04.00
  std::uint16_t do3_function_register{0x0419};        // P04.12
  std::uint16_t brake_release_wait_register{0x044D};  // P04.38

  // P0B
  std::uint16_t alarm_register{0x0B03};               // P0B.03
  std::uint16_t no_rotate_reason_register{0x0B04};    // P0B.04
  std::uint16_t run_state_register{0x0B05};           // P0B.05
  std::uint16_t input_io_state_register{0x0B11};      // P0B.17
  std::uint16_t physical_do_register{0x0B12};         // P0B.18
  std::uint16_t position_encoder_register{0x0B1C};    // P0B.28
  std::uint16_t speed_filtered_register{0x0B2C};      // 32-bit, 0.1 rpm
  std::uint16_t torque_register{0x0B2E};              // signed, 0.1%

  // Expected parameter values.
  std::uint16_t control_mode_speed{1};
  std::uint16_t speed_source_internal{1};

  // P03.01 = 0:
  // direction is determined by the sign of P03.04.
  std::uint16_t direction_from_speed_sign{0};

  // P05.06 = 0:
  // servo-brake stop mode.
  std::uint16_t stop_mode_servo_brake{0};

  // Parameter save/reset values.
  std::uint16_t save_all_parameters{0x2211};
  std::uint16_t reset_current_alarm_value{0x1111};
  std::uint16_t reset_history_alarm_value{0x1122};

  // SRV-ON DI function.
  std::uint16_t srv_on_di_function{0x03};

  // DO3 = BRK-OFF.
  std::uint16_t brake_off_do_function{0x03};

  // Default acceleration/deceleration parameters.
  std::uint16_t default_acceleration_raw{100};
  std::uint16_t default_deceleration_raw{100};

  // External control uses RPM directly.
  std::int32_t max_speed_rpm{3000};
};

[[nodiscard]] std::vector<std::uint8_t>
encode_climber_read_registers(
    std::uint8_t station_id,
    std::uint16_t start_register,
    std::uint16_t register_count);

[[nodiscard]] std::vector<std::uint8_t>
encode_climber_write_register(
    std::uint8_t station_id,
    std::uint16_t register_address,
    std::uint16_t value);

[[nodiscard]] std::vector<std::uint8_t>
encode_climber_write_target_speed(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile,
    std::int32_t speed_rpm);

[[nodiscard]] std::vector<std::uint8_t>
encode_climber_write_virtual_io(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile,
    std::uint16_t virtual_mask);

[[nodiscard]] std::vector<std::uint8_t>
encode_climber_reset_current_alarm(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile);

[[nodiscard]] std::vector<std::uint8_t>
encode_climber_reset_history_alarm(
    std::uint8_t station_id,
    const ClimberModbusProfile& profile);

[[nodiscard]] ModbusRegisterRead
decode_climber_read_registers_response(
    std::uint8_t station_id,
    std::uint16_t expected_register_count,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] Result
decode_climber_write_response(
    std::uint8_t station_id,
    std::uint16_t expected_register,
    std::uint16_t expected_value,
    const std::vector<std::uint8_t>& frame);

[[nodiscard]] std::int32_t
join_climber_i32_high_first(
    std::uint16_t high,
    std::uint16_t low) noexcept;

[[nodiscard]] std::uint16_t
encode_climber_signed_i16(
    std::int32_t value) noexcept;

}  // namespace snappy::drivers::protocols