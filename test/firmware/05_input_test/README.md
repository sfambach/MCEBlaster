# Step 5: input test

The input picture in the background, the measurement of [step 4](../04_ttl_input_analyzer/) on top. Planned as the "input test" entry of a later menu.

- Core 0 measures the TTL input once per second: HSync and VSync frequency, polarity and pulse width, lines per frame, detected mode, which input pins toggle.
- Core 1 captures the input picture frame by frame and shows it on a 640x480 VGA screen (320x240 pixels, each shown 2x2).
- The measurement is a centered text block; every line is left aligned with the title "MCE BLASTER - INPUT TEST". The text has a black outline, so the input picture stays visible around it. The values also go out over USB serial.
- `PIXEL CLOCK` button: hide or show the text, to see the whole input picture.
- "Frames" shows how many input frames per second were captured; if frames fail, it also says where (no VSync, VSync end, or the line where the capture stopped).

Besides MDA, CGA and EGA it recognises VGA 720x400 and 640x480 timing, which some VGA cards send on their 9-pin TTL output. The MCE Blaster firmware itself does not support those.

## How the capture works

- Which pins are shown is picked from the measurement: CGA GPIO 8-13 if they toggle, else EGA GPIO 0-5, else MDA GPIO 26-27 (V, I). The 6 colour bits go to the VGA DAC unchanged, as in the MCE Blaster firmware; MDA becomes grey, or white with intensity.
- A PIO program waits for the end of each HSync pulse and then samples 8 input pins 320 times, spread over 80 % of the line period. The sample rate follows the measured line frequency, so every mode fills the width. It is not locked to the card's pixel clock, so fine detail may shimmer.
- Core 1 waits for the end of VSync, restarts the capture and moves each line by DMA into a 320x240 buffer; with more than 240 lines per frame some lines are skipped.
- After each frame core 1 maps the samples to VGA colours and draws the text on top.
- The VGA output is the same PIO + DMA scheme as in steps 1-3, with every byte held for two pixel clocks.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `InputTest.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
