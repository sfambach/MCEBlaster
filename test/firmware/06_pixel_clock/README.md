# Step 6: pixel clock

Measures the pixel clock of the TTL input. Planned as the "auto pixel clock" entry of a later menu.

The TTL output of a video card has no clock line, so the clock is found from the picture itself:

1. A PIO samples the colour pins at 126 MHz (252 MHz / 2), 24 bursts of 16384 samples spread over different lines.
2. Every run of one colour lasts a whole number of pixels. The shortest frequent run length is one pixel.
3. Averaging all runs as multiples of that pixel time gives a first pixel time, good to about 1 %.
4. Then the distances between rising edges of the same pin are used, in three passes about 8, 40 and 200 pixels apart: each is a whole number of pixels, and the longer the distance, the finer the result (well below 0.1 %). Rising edges of one pin only, so slow or fast edges do not bias it.
5. Pixels per line = pixel clock / line frequency must be a whole number; rounding it and multiplying back by the measured line frequency gives the exact clock.

Shown on screen and over USB serial: pixel time, pixel clock, pixels per line (total, including blanking), the matching standard clock if any (CGA 14.318 MHz, EGA/MDA 16.257 MHz, VGA 25.175 / 28.322 MHz, 8x NTSC 28.636 MHz, used by some Trident cards), the number of runs used and how well they fit (rms deviation in pixels).

It needs fine detail on the screen: single pixel changes. The shaded block characters 176/177 (░ ▒) are ideal; a plain DOS prompt may not be enough.

Expected for VGA text mode (720x400, 31.469 kHz): 28.322 MHz and 900 pixels per line, or 28.636 MHz and 910 pixels per line on cards that use the 8x NTSC clock.

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
