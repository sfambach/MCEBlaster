# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
# Free for noncommercial use; commercial use only with the author's consent.
#
# Step 7: input test with the measured pixel clock. The TTL input picture,
# sampled once per input pixel, in the background, the analysis on top.
#
# $ mkdir build && cd build
# $ cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
# $ make -j
#
# Builds InputTestPixelClock.uf2.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(InputTestPixelClock C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(InputTestPixelClock main.cpp)
pico_generate_pio_header(InputTestPixelClock ${CMAKE_CURRENT_LIST_DIR}/VGAOut.pio)
pico_generate_pio_header(InputTestPixelClock ${CMAKE_CURRENT_LIST_DIR}/InputCapture.pio)
pico_generate_pio_header(InputTestPixelClock ${CMAKE_CURRENT_LIST_DIR}/FastSample.pio)
target_link_libraries(InputTestPixelClock pico_stdlib pico_multicore hardware_pio
                      hardware_dma hardware_pwm)
pico_enable_stdio_usb(InputTestPixelClock 1)
pico_enable_stdio_uart(InputTestPixelClock 0)
pico_add_extra_outputs(InputTestPixelClock)
