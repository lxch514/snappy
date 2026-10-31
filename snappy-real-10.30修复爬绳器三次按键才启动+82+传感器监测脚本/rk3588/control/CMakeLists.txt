# ============================================================
# @file CMakeLists.txt
# @brief 控制层构建配置
#
# 生成控制静态库，以及 snappy-control 与 snappy-real-control 两个可执行程序。
# ============================================================

find_package(Threads REQUIRED)

add_library(snappy_control
  src/control_application.cpp
  src/hardware_config.cpp
  src/device_registry.cpp
  src/actuator_service.cpp
  src/sensor_service.cpp
  src/lease_manager.cpp
  src/command_application.cpp
  src/safety_monitor.cpp
  src/watchdog.cpp
  src/gateway_ipc_server.cpp
  src/gateway_command_dispatcher.cpp
  src/diagnostics/app_log.cpp
)

target_include_directories(snappy_control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(snappy_control
  PUBLIC
    snappy_drivers
    nlohmann_json::nlohmann_json
)

add_executable(snappy-control
  src/main.cpp
)

target_link_libraries(snappy-control
  PRIVATE
    snappy_control
)

# ============================================================
# snappy-real：10传感器 + 4执行器整机控制进程
# ============================================================

add_executable(snappy-real-control
  src/main_real.cpp
  src/actuator_hub.cpp
  src/direct_rs485_bus.cpp
  src/control_server.cpp
  src/sensor_runtime.cpp
)

target_include_directories(snappy-real-control
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(snappy-real-control
  PRIVATE
    snappy_control
    snappy_drivers
    nlohmann_json::nlohmann_json
    Threads::Threads
)

target_compile_options(snappy-real-control
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
