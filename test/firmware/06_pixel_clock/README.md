# Step 6: pixel clock

Measures the pixel clock of the TTL input. Planned as the "auto pixel clock" entry of a later menu.

The TTL output of a video card has no clock line, so the clock is found from the picture itself:

1. A PIO samples the colour pins at 126 MHz (252 MHz / 2), 24 bursts of 16384 samples spread over different lines.
2. Every run of one colour lasts a whole number of pixels. The shortest frequent run length is one pixel.
3. Averaging all runs as multiples of that pixel time gives the pixel time to a fraction of a sample.
4. Pixels per line = pixel clock / line frequency must be a whole number; rounding it and multiplying back by the measured line frequency gives the exact clock.

Shown on screen and over USB serial: pixel time, pixel clock, pixels per line (total, including blanking), the matching standard clock if any (CGA 14.318 MHz, EGA/MDA 16.257 MHz, VGA 25.175 / 28.322 MHz), the number of runs used and how well they fit (rms deviation in pixels).

It needs fine detail on the screen: single pixel changes. The shaded block characters 176/177 (░ ▒) are ideal; a plain DOS prompt may not be enough.

Expected for VGA text mode (720x400, 31.469 kHz): 28.322 MHz, 900 pixels per line.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `PixelClock.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
