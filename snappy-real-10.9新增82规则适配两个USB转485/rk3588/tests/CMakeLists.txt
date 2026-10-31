# ============================================================
# @file CMakeLists.txt
# @brief RK3588 侧 C++ 测试注册
#
# 依次注册契约、驱动、控制、传感器采集与命令邮箱五组测试。
# ============================================================

add_executable(snappy_contract_tests contract_test.cpp)
target_link_libraries(snappy_contract_tests PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(snappy_contract_tests PRIVATE
  SNAPPY_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)
target_compile_options(snappy_contract_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME cpp_contract_tests COMMAND snappy_contract_tests)

add_executable(snappy_driver_tests driver_test.cpp)
target_link_libraries(snappy_driver_tests PRIVATE
  snappy_drivers
  snappy_fake_hardware
)
target_compile_options(snappy_driver_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME cpp_driver_tests COMMAND snappy_driver_tests)

add_executable(snappy_control_tests control_test.cpp)
target_link_libraries(snappy_control_tests PRIVATE
  snappy_control
  snappy_fake_hardware
)
target_compile_options(snappy_control_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME cpp_control_tests COMMAND snappy_control_tests)
add_executable(snappy_sensor_service_tests
  sensor_service_test.cpp
)

target_link_libraries(snappy_sensor_service_tests PRIVATE
  snappy_control
  snappy_drivers
  snappy_fake_hardware
  nlohmann_json::nlohmann_json
)

target_compile_definitions(snappy_sensor_service_tests PRIVATE
  SNAPPY_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_compile_options(snappy_sensor_service_tests PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

add_test(
  NAME cpp_sensor_service_tests
  COMMAND snappy_sensor_service_tests
)

# ============================================================
# CommandMailbox / ActuatorSnapshot 单元测试
#
# 原先属于 snappy-snappy/tests/，随 snappy-snappy/ 目录一起迁到这里。
#
# 两个测试都是 header-only，只 include 以下被测头：
#   snappy_snappy/command_mailbox.hpp
#   snappy_snappy/actuator_snapshot.hpp
# 这两个头位于 rk3588/control/include/，所以只需要把 include 根
# 指过去，不需要链接 snappy_control / snappy_drivers。
# ============================================================

# Threads::Threads 由 find_package(Threads) 创建，而 CMake 的
# imported target 默认只在"创建它的目录及其子目录"可见；
# rk3588/tests 与 rk3588/control 是兄弟目录，看不到对方创建的
# Threads::Threads，因此这里必须自己再声明一次。
find_package(Threads REQUIRED)

add_executable(snappy_command_mailbox_tests
  command_mailbox_test.cpp
)

target_include_directories(snappy_command_mailbox_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/rk3588/control/include
)

# CommandMailbox 内部使用 std::mutex / std::condition_variable，
# 需要线程库支撑；与迁移前的链接方式保持一致。
target_link_libraries(snappy_command_mailbox_tests PRIVATE
  Threads::Threads
)

target_compile_options(snappy_command_mailbox_tests PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

add_test(
  NAME cpp_command_mailbox_tests
  COMMAND snappy_command_mailbox_tests
)

add_executable(snappy_snapshot_tests
  snapshot_test.cpp
)

target_include_directories(snappy_snapshot_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/rk3588/control/include
)

target_link_libraries(snappy_snapshot_tests PRIVATE
  Threads::Threads
)

target_compile_options(snappy_snapshot_tests PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

add_test(
  NAME cpp_snapshot_tests
  COMMAND snappy_snapshot_tests
)
