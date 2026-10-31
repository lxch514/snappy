cmake_minimum_required(VERSION 3.16)

project(snappy_snappy
  VERSION 0.2.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable(snappy-snappy-control
  src/main.cpp
  src/actuator_hub.cpp
  src/direct_rs485_bus.cpp
  src/control_server.cpp
)

target_include_directories(snappy-snappy-control
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# snappy_control 中包含原始 HardwareConfig 和 ActuatorService。
# 本程序只显式构造 ActuatorService，不创建：
# DeviceRegistry、SensorService、GatewayCommandDispatcher。
#
# snappy_control 和 snappy_drivers 仍然复用原工程的设备驱动实现，
# 但运行时核心由 ActuatorHub、CommandMailbox 和 DirectRs485Bus 负责。
target_link_libraries(snappy-snappy-control
  PRIVATE
    snappy_control
    snappy_drivers
    nlohmann_json::nlohmann_json
    Threads::Threads
)

target_compile_options(snappy-snappy-control
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# 单独进入 snappy-snappy 目录构建时也可以启用测试。
# 根工程已经启用 CTest 时，这里不会改变已有测试结构。
if(SNAPPY_BUILD_TESTS)
  include(CTest)

  if(NOT BUILD_TESTING)
    enable_testing()
  endif()

  add_executable(command_mailbox_test
    tests/command_mailbox_test.cpp
  )

  target_include_directories(command_mailbox_test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(command_mailbox_test
    PRIVATE
      Threads::Threads
  )

  add_test(
    NAME snappy_snappy_command_mailbox_test
    COMMAND command_mailbox_test
  )

  add_executable(snapshot_test
    tests/snapshot_test.cpp
  )

  target_include_directories(snapshot_test
    PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )

  target_link_libraries(snapshot_test
    PRIVATE
      Threads::Threads
  )

  add_test(
    NAME snappy_snappy_snapshot_test
    COMMAND snapshot_test
  )
endif()