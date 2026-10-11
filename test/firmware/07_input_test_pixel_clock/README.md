# Step 7: input test with the measured pixel clock

[Step 5](../05_input_test/) with the pixel clock of [step 6](../06_pixel_clock/): the input is sampled once per input pixel instead of 320 times per line, so the picture is much sharper.

- Core 0 measures the input once per second (sync, mode, pins). For a known mode the picture window comes from the standard porches and size of the mode; for an unknown mode it grows to every non-black pixel and line seen (checked every 8th frame). The line "Window" shows it. "lag" on the Frames line is how many lines core 1 fell behind the capture; a frame in which it falls behind by the whole ring is dropped ("late") and the last picture stays. "stall" counts frames in which the capture had to wait for the DMA. The window width is scaled to 640 output pixels (720 to 640 drops the empty ninth column of each character cell) and its height to 240 rows. The capture runs at the measured pixel clock snapped to the nearest known clock within 1.5 %, or at the mode's usual clock while there is no measurement. A watchdog resets the Pico if core 0 hangs for 4 s; the line "Hang reset" then shows where both cores were. A detected mode is latched and held until three measurements in a row leave it (HSync more than 2 %, VSync more than 2 Hz, or a polarity change); "held 1/3" on the mode line counts those misses. The pixel clock is measured once per latched mode.
- Core 1 samples each line at that clock, from shortly before the expected picture to shortly before the next HSync, and shows the window on a 640x480 VGA screen (240 lines, each shown twice). With more than 240 picture lines some lines are skipped; [step 8](../08_input_passthrough/) shows all of them.
- `PIXEL CLOCK` button: hide or show the text.
- `AUTO ADJUST` button: move the sampling point by half an input pixel (phase 0/1). Use the phase with the cleaner picture.
- "Frames" shows how many input frames per second were captured, and where it stopped if not.

## How the capture works

- The PIO waits for the end of HSync, waits the phase delay, then takes one sample per input pixel (two PIO instructions per sample, the PIO runs at twice the pixel clock with a fractional divider).
- One DMA channel writes the whole frame into a ring buffer; the DMA wraps the address in hardware, so it can never write outside it. Core 1 follows a few lines behind, converts each line to VGA colours, writes it into the output line and puts the text on top.
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
