add_executable(snappy_gateway_tests gateway_test.cpp)
target_link_libraries(snappy_gateway_tests PRIVATE
  snappy_gateway
  snappy_fake_g20
  snappy_fake_mqtt
  nlohmann_json::nlohmann_json
)
target_compile_options(snappy_gateway_tests PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME cpp_gateway_tests COMMAND snappy_gateway_tests)

add_test(NAME fake_end_to_end_smoke COMMAND snappy-sim)
