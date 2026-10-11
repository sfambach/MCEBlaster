# Step 1: VGA test image

Static test image for the MCE Blaster's VGA output. It does not read the TTL input, so it only checks the VGA side: Pico, resistor DAC, VGA connector, cable and monitor.

The picture shows:
- a white frame (2 pixels) along the edge of the visible area. All four sides must be visible and straight.
- the VGA mode, line frequency (H), frame frequency (V) and pixel clock as text, plus a thank-you to Scrap Computing.

The same values are printed once every 2 seconds over USB serial (e.g. `cat /dev/ttyACM0`).

## Modes

| UF2 | Mode | H | V | Pixel clock |
|---|---|---|---|---|
| `VGATestImage_640x400.uf2` | 640x400, HSync −, VSync + | 31.500 kHz | 70.16 Hz | 25.2 MHz |
| `VGATestImage_640x480.uf2` | 640x480, HSync −, VSync − | 31.500 kHz | 60.00 Hz | 25.2 MHz |
| `VGATestImage_800x600.uf2` | 800x600, HSync +, VSync + | 35.156 kHz | 56.25 Hz | 36.0 MHz |

These are the three modes the MCE Blaster outputs. The Pico runs at 252 MHz so that both pixel clocks are an integer divider (/10 and /7); 25.2 MHz is within the tolerance of the 25.175 MHz VGA standard.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy the UF2 to the Pico as described in the main README (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## How it works

Each distinct scanline (blank, vsync, top/bottom frame, side frame, one per text row) is stored once, including the sync bits (bit 6 HSync, bit 7 VSync, same GPIOs 14-21 as the MCE Blaster). A table holds one pointer per scanline. A DMA control channel feeds these pointers to a data channel that streams the line into a one-instruction PIO program (`out pins, 8`), so the picture runs without the CPU. An interrupt only rewinds the table once per frame.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). This folder contains no code from the MCE Blaster firmware; the pin assignment and the VGA timings follow the MCE Blaster so the test runs on its hardware. The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
