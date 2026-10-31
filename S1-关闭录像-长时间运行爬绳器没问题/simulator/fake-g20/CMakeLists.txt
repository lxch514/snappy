add_library(snappy_fake_g20 src/fake_g20_transport.cpp)
target_include_directories(snappy_fake_g20
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(snappy_fake_g20 PUBLIC snappy_gateway)
target_compile_options(snappy_fake_g20 PRIVATE -Wall -Wextra -Wpedantic)
