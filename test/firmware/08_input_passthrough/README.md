# Step 8: input passthrough

The input picture at full height: the VGA output runs at the input's own timing, and every input line goes out once. Nothing is dropped or repeated, so the missing lines and the flicker of [step 7](../07_input_test_pixel_clock/) are gone.

- The output HSync and VSync are copies of the input's (one PIO state machine per pin), so the monitor sees the input mode itself. For a VGA monitor the input must have VGA timing, e.g. 720x400 at 70 Hz from a VGA card's 9-pin TTL output. CGA, EGA and MDA (15–22 kHz) are passed through as they are; most VGA monitors do not show those.
- Capture and output share one ring and one clock. Core 1's capture writes each line at the input pixel clock; the output DMA reads the same bytes three lines later and the output PIO sends them at the same clock, after the same delay behind HSync. Nothing is scaled or copied, so each line goes out exactly as it came in, borders included.
- Core 1 only turns MDA samples into grey levels and puts the text into each line before the output reaches it. "lag" on the Frames line is how many lines core 1 fell behind the capture.
- Capture and output restart together in every VSync, so both count the same lines.
- Core 0 measures the input once per second, latches the mode, snaps the pixel clock to the nearest known clock and draws the text (as in step 7). The text bitmaps are built on core 0, so core 1 never misses an input frame.
- Without an input signal there is no sync to send a picture with, so the monitor shows no signal. Use [step 7](../07_input_test_pixel_clock/) to look at the analysis then.
- `PIXEL CLOCK` button: hide or show the text.
- `AUTO ADJUST` button: move the sampling point by half an input pixel (phase 0/1). Use the phase with the cleaner picture.
- A watchdog resets the Pico if core 0 hangs for 4 s; the line "Hang reset" then shows where both cores were.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `InputPassthrough.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
