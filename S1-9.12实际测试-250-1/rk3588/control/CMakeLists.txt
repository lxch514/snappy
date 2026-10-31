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