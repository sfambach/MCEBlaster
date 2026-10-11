# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in this folder.
# Free for noncommercial use; commercial use only with the author's consent.
#
# VGA test picture for the MCE Blaster.
#
# $ mkdir build && cd build
# $ cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
# $ make -j
#
# Builds one UF2 per VGA mode: VGATestbild_640x400.uf2, VGATestbild_640x480.uf2
# and VGATestbild_800x600.uf2.

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(VGATestbild C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

function(add_testbild NAME MODE)
  add_executable(${NAME} main.cpp)
  pico_generate_pio_header(${NAME} ${CMAKE_CURRENT_LIST_DIR}/VGAOut.pio)
  target_compile_definitions(${NAME} PRIVATE VGA_MODE=${MODE})
  target_link_libraries(${NAME} pico_stdlib hardware_pio hardware_dma)
  pico_enable_stdio_usb(${NAME} 1)
  pico_enable_stdio_uart(${NAME} 0)
  pico_add_extra_outputs(${NAME})
endfunction()

add_testbild(VGATestbild_640x400 0)
add_testbild(VGATestbild_640x480 1)
add_testbild(VGATestbild_800x600 2)
