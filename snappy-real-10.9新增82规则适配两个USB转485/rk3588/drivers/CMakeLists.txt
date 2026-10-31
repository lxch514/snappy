
# ============================================================
# @file CMakeLists.txt
# @brief 驱动层静态库构建配置
#
# 汇聚协议编解码、总线、设备驱动与日志等源文件。
# ============================================================

add_library(snappy_drivers
  src/result.cpp
  src/unconfigured_drivers.cpp

  src/diagnostics/driver_log.cpp

  src/bus/can_socket_bus.cpp
  src/bus/rs485_port.cpp
  src/bus/rs485_transaction_scheduler.cpp

  src/protocols/anemometer_modbus.cpp
  src/protocols/brush_motor_modbus.cpp
  src/protocols/can_frame.cpp
  src/protocols/lidar_modbus.cpp
  src/protocols/modbus_rtu.cpp
  src/protocols/power_modbus.cpp
  src/protocols/propulsion_fan_can.cpp
  src/protocols/push_motor_codec.cpp
  src/protocols/relay_modbus.cpp
  src/protocols/temperature_modbus.cpp
  src/protocols/wit_motion_modbus.cpp

  src/devices/anemometer.cpp
  src/devices/brush_motor.cpp
  src/devices/climber.cpp
  src/devices/lidar.cpp
  src/devices/power.cpp
  src/devices/propulsion_fan.cpp
  src/devices/push_motor.cpp
  src/devices/relay.cpp
  src/devices/temperature.cpp
  src/devices/wit_motion.cpp

  src/protocols/climber_modbus.cpp
)

target_include_directories(snappy_drivers
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_options(snappy_drivers PRIVATE -Wall -Wextra -Wpedantic)