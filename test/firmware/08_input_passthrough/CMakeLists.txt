# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
# Free for noncommercial use; commercial use only with the author's consent.
#
# Step 8: input passthrough. Every TTL input line goes out once, at the
# input's own sync, with the analysis on top.
#
# $ mkdir build && cd build
# $ cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
# $ make -j
#
# Builds InputPassthrough.uf2.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(InputPassthrough C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(InputPassthrough main.cpp)
pico_generate_pio_header(InputPassthrough ${CMAKE_CURRENT_LIST_DIR}/PassOut.pio)
pico_generate_pio_header(InputPassthrough ${CMAKE_CURRENT_LIST_DIR}/InputCapture.pio)
pico_generate_pio_header(InputPassthrough ${CMAKE_CURRENT_LIST_DIR}/FastSample.pio)
target_link_libraries(InputPassthrough pico_stdlib pico_multicore hardware_pio
                      hardware_watchdog
                      hardware_dma hardware_pwm)
pico_enable_stdio_usb(InputPassthrough 1)
pico_enable_stdio_uart(InputPassthrough 0)
pico_add_extra_outputs(InputPassthrough)
