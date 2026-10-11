# Step 2: VGA test image with mode switch

Same test image as [step 1](../01_vga_testimage/) (white frame, mode, H/V frequency, pixel clock), but one firmware for all three VGA modes of the MCE Blaster:

- `PIXEL CLOCK` button: next mode
- `AUTO ADJUST` button: previous mode
- Order: 640x400 → 640x480 → 800x600 → back to 640x400. Starts in 640x480.

The image also shows the mode number and the button hint. Every switch is printed over USB serial, plus a status line every 2 seconds.

This checks that the buttons work and that the monitor syncs to every mode the MCE Blaster outputs.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `VGATestImageSwitch.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## What changed compared to step 1

- The timing is a pointer (`TP`) instead of a constant, so it can change at run time.
- `switchMode()` stops the DMA channels and the PIO, rebuilds the scanlines for the new timing, sets the new PIO clock divider and restarts the output. The data channel is unchained first, so aborting it does not restart the control channel.
- `Button` debounces a button (3 equal readings, 10 ms apart) and reports each press once.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.
