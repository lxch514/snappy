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
