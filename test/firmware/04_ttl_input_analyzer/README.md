# Step 4: TTL input analyzer

Measures the MDA/CGA/EGA input once per second and shows the results on a 640x480 VGA screen and over USB serial:

- HSync and VSync: frequency, polarity (pos/neg) and pulse width
- Lines per frame (HSync / VSync)
- Detected mode, compared with the MCE Blaster's `TimingsTTL.def`: MDA 720x350 50 Hz, CGA/EGA 640x200 60 Hz, EGA 640x350 60 Hz
- Every input pin: `*` toggles, `+` always high, `-` always low (EGA GPIO 0-5, CGA GPIO 8-13, MDA GPIO 26-27, VSync/HSync GPIO 6-7)

Without a signal it shows `NO SIGNAL`; with the pull-ups all pins then read `+`. A pin that stays `+` or `-` while the PC shows a coloured picture points to a broken level shifter, solder joint or cable.

Expected for a CGA card: HSync about 15.70 kHz, VSync about 59.9 Hz, about 262 lines per frame, both syncs positive.

## How it measures

- HSync edges are counted in hardware: GPIO 7 is a PWM B input, the PWM counter runs in rising edge mode.
- VSync edges are counted by a GPIO interrupt.
- During the same second the CPU reads all pins as fast as it can: the share of samples where a sync is high gives its polarity and pulse width, OR/AND of all samples shows which pins toggle.
- The VGA output runs on its own (PIO + DMA as in step 2), the text lines are redrawn after each measurement.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `TTLInputAnalyzer.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. `Font8x8.h` is public domain (Daniel Hepper). The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
