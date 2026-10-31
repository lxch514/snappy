add_library(snappy_fake_mqtt src/fake_mqtt_transport.cpp)
target_include_directories(snappy_fake_mqtt
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(snappy_fake_mqtt PUBLIC snappy_gateway)
target_compile_options(snappy_fake_mqtt PRIVATE -Wall -Wextra -Wpedantic)
