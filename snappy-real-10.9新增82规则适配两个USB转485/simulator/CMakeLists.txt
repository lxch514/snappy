# ============================================================
# @file CMakeLists.txt
# @brief Snappy simulator 子目录构建配置
#
# - 根目录 CMakeLists.txt 负责添加 rk3588、gateway 和 simulator。
# - 本文件只负责 simulator 目录下的模拟硬件、模拟 G20、模拟 MQTT 和 snappy-sim 主程序。
# ============================================================

add_subdirectory(fake-hardware)
add_subdirectory(fake-g20)
add_subdirectory(fake-mqtt)

# ============================================================
# 构建模拟器主程序
# ============================================================

add_executable(snappy-sim main.cpp)

target_link_libraries(snappy-sim PRIVATE
  snappy_control
  snappy_gateway
  snappy_fake_hardware
  snappy_fake_g20
  snappy_fake_mqtt
  nlohmann_json::nlohmann_json
)

target_compile_definitions(snappy-sim PRIVATE
  SNAPPY_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
)

target_compile_options(snappy-sim PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)