# Step 7: input test with the measured pixel clock

[Step 5](../05_input_test/) with the pixel clock of [step 6](../06_pixel_clock/): the input is sampled once per input pixel instead of 320 times per line, so the picture is much sharper.

- Core 0 measures the input once per second (sync, mode, pins). Each line is sampled from the end of the back porch, and the visible width (720 in VGA text) is scaled to 640 output pixels; 720 to 640 drops the empty ninth column of each character cell. The capture runs at the measured pixel clock snapped to the nearest known clock within 1.5 %, or at the mode's usual clock while there is no measurement. A watchdog resets the Pico if core 0 hangs for 4 s; the line "Hang reset" then shows where both cores were. A detected mode is latched and held until three measurements in a row leave it (HSync more than 2 %, VSync more than 2 Hz, or a polarity change); "held 1/3" on the mode line counts those misses. The pixel clock is measured once per latched mode.
- Core 1 samples 768 pixels per line from the end of HSync at that clock and shows the middle 640 at full width (640x480 VGA, 240 lines, each shown twice). Wider modes such as 720 pixel VGA text lose a few columns on each side; with more than 240 lines per frame some lines are skipped.
- `PIXEL CLOCK` button: hide or show the text.
- `AUTO ADJUST` button: move the sampling point by half an input pixel (phase 0/1). Use the phase with the cleaner picture.
- "Frames" shows how many input frames per second were captured, and where it stopped if not.

## How the capture works

- The PIO waits for the end of HSync, waits the phase delay, then takes one sample per input pixel (two PIO instructions per sample, the PIO runs at twice the pixel clock with a fractional divider).
- Two DMA channels take turns, one line each, chained to each other. While one fills its line buffer, core 1 converts the other one to VGA colours, writes it into the output line and puts the text on top.
- The text is prepared as two small bitmaps (glyphs and black outline) once per update, so it fits into the line time.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `InputTestPixelClock.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
