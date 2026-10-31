find_package(Threads REQUIRED)

add_library(snappy_gateway
  src/unconfigured_transports.cpp
  src/control_channel.cpp
  src/gateway_app.cpp
  src/gateway_config.cpp
  src/peer_tracker.cpp
  src/status_publisher.cpp
  src/telemetry_publisher.cpp
  src/diagnostics/gateway_log.cpp
  src/g20/g20_button_mapper.cpp
  src/g20/g20_command_router.cpp
  src/g20/g20_legacy_packet_adapter.cpp
  src/g20/g20_packet_codec.cpp
  src/g20/g20_udp_transport.cpp
  src/g20/g20_video.cpp
)

target_include_directories(snappy_gateway
  PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(snappy_gateway
  PUBLIC snappy_control
  PRIVATE nlohmann_json::nlohmann_json Threads::Threads
)

target_compile_options(snappy_gateway PRIVATE -Wall -Wextra -Wpedantic)

add_executable(snappy-gateway src/main.cpp)
target_link_libraries(snappy-gateway PRIVATE snappy_gateway)
target_compile_options(snappy-gateway PRIVATE -Wall -Wextra -Wpedantic)