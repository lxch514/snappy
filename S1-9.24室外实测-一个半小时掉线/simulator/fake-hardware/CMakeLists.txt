add_library(snappy_fake_hardware src/fake_hardware.cpp)
target_include_directories(snappy_fake_hardware
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(snappy_fake_hardware PUBLIC snappy_drivers)
target_compile_options(snappy_fake_hardware PRIVATE -Wall -Wextra -Wpedantic)
