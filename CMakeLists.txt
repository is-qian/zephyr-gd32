cmake_minimum_required(VERSION 3.20.0)
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zephyr-gd32)

target_sources(app PRIVATE src/main.c src/dxl.c src/imu.c)
