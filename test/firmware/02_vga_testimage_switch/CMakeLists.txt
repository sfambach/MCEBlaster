# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
# Free for noncommercial use; commercial use only with the author's consent.
#
# Step 2: VGA test image for the MCE Blaster, the buttons switch between
# 640x400, 640x480 and 800x600.
#
# $ mkdir build && cd build
# $ cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
# $ make -j
#
# Builds VGATestImageSwitch.uf2.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(VGATestImageSwitch C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(VGATestImageSwitch main.cpp)
pico_generate_pio_header(VGATestImageSwitch ${CMAKE_CURRENT_LIST_DIR}/VGAOut.pio)
target_link_libraries(VGATestImageSwitch pico_stdlib hardware_pio hardware_dma)
pico_enable_stdio_usb(VGATestImageSwitch 1)
pico_enable_stdio_uart(VGATestImageSwitch 0)
pico_add_extra_outputs(VGATestImageSwitch)
