# Step 9: frame buffer

[Step 7](../07_input_test_pixel_clock/) without scaling: every input pixel and every input line is stored once in a frame buffer and shown 1:1, smaller, with a black border, on a normal 640x480 VGA screen that every monitor, scaler and grabber takes.

- Core 0 measures the input, latches the mode and the pixel clock and draws the text, as in step 7. The title line shows the build time, so you can tell which firmware is on the Pico.
- Core 1 samples each line at the input pixel clock and stores the picture window, one nibble (16 colours) per input pixel, in the frame buffer: up to 720 x 480 pixels, 168 KB. "lag", "late" and "stall" on the Frames line mean the same as in step 7.
- The output runs on its own at 640x480 timing (31.5 kHz, 60 Hz, both syncs negative), but with 720 pixels per line at 28 MHz, so a 720-pixel input line fits 1:1. A DMA IRQ on core 0 converts the next screen line from the frame buffer and puts the text on top, one line ahead of the output.
- Picture placement: centred; 350 or 400 lines go out 1:1, modes with at most 240 lines (CGA, EGA 200) are doubled. Input and output run at their own rates, so a moving picture can tear; a still one cannot.
- Colours: blue, green, red and intensity. EGA's secondary colour lines count as intensity, so EGA's 64 colours are folded onto 16; colour 6 is brown as on a CGA monitor. MDA: black, grey, white.
- `PIXEL CLOCK` button: hide or show the text.
- `AUTO ADJUST` button: move the sampling point by half an input pixel (phase 0/1).

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `FrameBuffer.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
