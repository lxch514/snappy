#include "snappy/drivers/protocols/relay_modbus.hpp"

namespace snappy::drivers::protocols::push_motor_codec {

// MSB500 push motor has no independent RS485/CAN protocol in this machine.
// It is driven by relay output channels. Direction interlock, dead time and
// max-on-time protection are implemented in devices/push_motor.*.

constexpr bool kUsesRelayProtocol = true;

}  // namespace snappy::drivers::protocols::push_motor_codec