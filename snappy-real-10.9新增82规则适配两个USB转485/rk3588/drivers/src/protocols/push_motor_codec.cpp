#include "snappy/drivers/protocols/relay_modbus.hpp"
/**
 * @file push_motor_codec.cpp
 * @brief 推杆继电器通道编码
 *
 * @details
 * - 该机型推杆没有独立串口或 CAN 协议，由继电器通道驱动。
 * - 方向互锁、死区和最长通电保护在 devices/push_motor.* 中实现。
 */

namespace snappy::drivers::protocols::push_motor_codec {

// MSB500 push motor has no independent RS485/CAN protocol in this machine.
// It is driven by relay output channels. Direction interlock, dead time and
// max-on-time protection are implemented in devices/push_motor.*.

constexpr bool kUsesRelayProtocol = true;

}  // namespace snappy::drivers::protocols::push_motor_codec