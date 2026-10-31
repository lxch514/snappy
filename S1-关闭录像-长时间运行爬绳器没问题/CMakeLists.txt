cmake_minimum_required(VERSION 3.16)

project(snappy VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SNAPPY_BUILD_TESTS
       "Build dependency-free fake/contract tests"
       ON)

option(SNAPPY_ENABLE_REAL_DRIVERS
       "Enable real hardware/network backends"
       OFF)

if(SNAPPY_ENABLE_REAL_DRIVERS)
  set(SNAPPY_REAL_DRIVERS_COMPILE_VALUE 1)
else()
  set(SNAPPY_REAL_DRIVERS_COMPILE_VALUE 0)
endif()

add_compile_definitions(
  SNAPPY_ENABLE_REAL_DRIVERS=${SNAPPY_REAL_DRIVERS_COMPILE_VALUE}
)

find_package(nlohmann_json 3.2.0 REQUIRED)

add_subdirectory(rk3588/drivers)
add_subdirectory(rk3588/control)
add_subdirectory(rk3588/gateway)
add_subdirectory(simulator)

# snappy-snappy 是实际执行器进程，不能依赖测试开关。
add_subdirectory(snappy-snappy)

if(SNAPPY_BUILD_TESTS)
  include(CTest)
  enable_testing()

  add_subdirectory(rk3588/tests)
  add_subdirectory(simulator/tests)

  find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)

  add_test(
    NAME python_contract_and_cloud_tests
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover -s cloud/tests -v
  )

  set_tests_properties(
    python_contract_and_cloud_tests
    PROPERTIES WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
  )
endif()