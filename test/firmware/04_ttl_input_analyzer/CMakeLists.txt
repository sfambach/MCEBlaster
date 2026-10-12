# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
# Free for noncommercial use; commercial use only with the author's consent.
#
# Step 4: measures the TTL input (MDA/CGA/EGA) and shows the results on VGA.
#
# $ mkdir build && cd build
# $ cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
# $ make -j
#
# Builds TTLInputAnalyzer.uf2.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(TTLInputAnalyzer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(TTLInputAnalyzer main.cpp)
pico_generate_pio_header(TTLInputAnalyzer ${CMAKE_CURRENT_LIST_DIR}/VGAOut.pio)
target_link_libraries(TTLInputAnalyzer pico_stdlib hardware_pio hardware_dma
                      hardware_pwm)
pico_enable_stdio_usb(TTLInputAnalyzer 1)
pico_enable_stdio_uart(TTLInputAnalyzer 0)
pico_add_extra_outputs(TTLInputAnalyzer)
