cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_imu C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(imu_test
  imu_test.c
  lsm9ds1_reg.c
)
target_link_libraries(imu_test hardware_spi pico_stdlib)

pico_enable_stdio_usb(imu_test 1)
pico_enable_stdio_uart(imu_test 0)

pico_add_extra_outputs(imu_test)

