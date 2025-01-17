# MIT License
# 
# Copyright (c) 2022 Daniel Robertson
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(hx711test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

pico_sdk_init()

add_compile_options(
        -Wall
        -Wextra
        -Werror
        -Wfatal-errors
        -Wfloat-equal
        -Wunreachable-code
        -Wno-unused-function
        -Wno-maybe-uninitialized
        -Wno-ignored-qualifiers
        -Wno-stringop-overflow          # rp2040_usb.c:61:3
        -Wno-array-bounds               # rp2040_usb.c:61:3
        )

add_subdirectory(i2c_slave)

# main/example program config
add_executable(main
        ${CMAKE_CURRENT_LIST_DIR}/main.c
        ${CMAKE_CURRENT_LIST_DIR}/src/hx711.c
        ${CMAKE_CURRENT_LIST_DIR}/src/mass.c
        ${CMAKE_CURRENT_LIST_DIR}/src/scale.c
        ${CMAKE_CURRENT_LIST_DIR}/src/util.c
        )

pico_generate_pio_header(main
        ${CMAKE_CURRENT_LIST_DIR}/src/hx711_noblock.pio
        )

target_link_libraries(main
        hardware_clocks
        hardware_pio
        hardware_i2c
        hardware_timer
        pico_divider
        pico_double
        pico_stdlib
        pico_sync
        pico_time
        )

pico_add_extra_outputs(main)

target_compile_options(main PRIVATE -Wall)

target_link_libraries(main i2c_slave pico_stdlib)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

# calibration prog config
add_executable(calibration
        ${CMAKE_CURRENT_LIST_DIR}/calibration.c
        ${CMAKE_CURRENT_LIST_DIR}/src/hx711.c
        ${CMAKE_CURRENT_LIST_DIR}/src/mass.c
        ${CMAKE_CURRENT_LIST_DIR}/src/scale.c
        ${CMAKE_CURRENT_LIST_DIR}/src/util.c
        )

pico_generate_pio_header(calibration
        ${CMAKE_CURRENT_LIST_DIR}/src/hx711_noblock.pio
        )

target_link_libraries(calibration
        hardware_clocks
        hardware_pio
        hardware_timer
        pico_divider
        pico_double
        pico_stdlib
        pico_sync
        pico_time
        )

pico_add_extra_outputs(calibration)

pico_enable_stdio_usb(calibration 1)
pico_enable_stdio_uart(calibration 1)


# test program config
add_executable(test
        ${CMAKE_CURRENT_LIST_DIR}/tests/test.c
        ${CMAKE_CURRENT_LIST_DIR}/src/hx711.c
        )

pico_generate_pio_header(test
        ${CMAKE_CURRENT_LIST_DIR}/src/hx711_noblock.pio
        )

target_link_libraries(test
        hardware_clocks
        hardware_pio
        hardware_timer
        pico_divider
        pico_double
        pico_stdlib
        pico_sync
        pico_time
        )

pico_add_extra_outputs(test)

pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 1)
