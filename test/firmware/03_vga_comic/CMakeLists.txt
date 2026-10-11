# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
# Free for noncommercial use; commercial use only with the author's consent.
#
# Step 3: comic picture in 800x600 for the MCE Blaster (400x300 pixels,
# each pixel shown 2x2).
#
# $ mkdir build && cd build
# $ cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
# $ make -j
#
# Builds VGAComic.uf2.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(VGAComic C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(VGAComic main.cpp)
pico_generate_pio_header(VGAComic ${CMAKE_CURRENT_LIST_DIR}/VGAOut.pio)
target_link_libraries(VGAComic pico_stdlib hardware_pio hardware_dma)
pico_enable_stdio_usb(VGAComic 1)
pico_enable_stdio_uart(VGAComic 0)
pico_add_extra_outputs(VGAComic)
