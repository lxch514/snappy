#include <iostream>

#include "snappy/control/control_application.hpp"
#include "snappy/simulator/fake_hardware.hpp"

int main() {
  snappy::simulator::FakeCanBus can;
  snappy::simulator::FakeRs485Bus rs485;
  snappy::simulator::FakeRelayBank relays;
  snappy::simulator::FakeSensorSuite sensors;
  snappy::control::ControlApplication app({can, rs485, relays, sensors});

  const auto report = app.start();
  if (!report.ready_for_arm || app.state() != snappy::control::ControlState::kDisarmed) {
    std::cerr << "fake control startup did not reach DISARMED\n";
    return 1;
  }

  relays.set("squeegee_extend", snappy::drivers::RelayState::kEnergized);
  app.enter_safe_hold("test fault");
  if (app.state() != snappy::control::ControlState::kSafeHold ||
      !can.safe_stop_called() || !rs485.safe_stop_called() ||
      !relays.deenergize_all_called() ||
      relays.state_of("squeegee_extend") != snappy::drivers::RelayState::kDeenergized) {
    std::cerr << "SAFE_HOLD did not reach all fake hardware boundaries\n";
    return 1;
  }

  std::cout << "control lifecycle tests passed\n";
  return 0;
}
