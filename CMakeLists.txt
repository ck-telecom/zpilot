# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Add the board
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor/l3g4200d)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor/lsm303dlh)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zpilot)

# mavlink v2 library header files
target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/libs/c_library_v2/standard)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/sensor.c)
target_sources(app PRIVATE src/pwm.c)

target_sources(app PRIVATE src/telemetry/telemetry.c)
