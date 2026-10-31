/**
 * @file main.cpp
 * @brief snappy-sim 仿真进程入口
 *
 * @details
 * - 装配假硬件并跑通启动与安全停止。
 * - 以 JSON 输出 fake_hardware_ready 和 safe_stop_called 两项结果。
 */
 
#include <iostream>

#include <nlohmann/json.hpp>

#include "snappy/control/control_application.hpp"
#include "snappy/simulator/fake_hardware.hpp"

int main() {
  try {
    snappy::simulator::FakeCanBus can;
    snappy::simulator::FakeRs485Bus rs485;
    snappy::simulator::FakeRelayBank relays;
    snappy::simulator::FakeSensorSuite sensors;

    snappy::control::ControlApplication control({can, rs485, relays, sensors});
    const auto startup = control.start();
    if (!startup.ready_for_arm) {
      std::cerr << startup.detail << '\n';
      return 1;
    }

    control.enter_safe_hold("simulator shutdown");

    nlohmann::json report{
        {"fake_hardware_ready", startup.ready_for_arm},
        {"safe_stop_called",
         can.safe_stop_called() && rs485.safe_stop_called() &&
             relays.deenergize_all_called()},
    };

    std::cout << report.dump(2) << '\n';

    return report.at("fake_hardware_ready").get<bool>() &&
                   report.at("safe_stop_called").get<bool>()
               ? 0
               : 1;
  } catch (const std::exception& error) {
    std::cerr << "snappy-sim failed: " << error.what() << '\n';
    return 1;
  }
}