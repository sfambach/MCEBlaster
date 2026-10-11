# Step 3: comic picture in 800x600

A 64-colour comic picture (SteFam in a suit) in 800x600 @ 56 Hz. Steps 1 and 2 only use black and white; this one uses all 64 colours of the 6-bit DAC, so a missing or swapped colour bit shows up at once.

- 400x300 pixels, every pixel shown 2x2: the PIO holds each byte for two pixel clocks (`out pins, 8 [1]`) and every picture line is sent twice.
- The picture is stored run-length encoded in flash (`Comic.h`, ~27 KB) and unpacked into RAM at start (~150 KB line buffers).
- USB serial prints a status line every 2 seconds.

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `VGAComic.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## Changing the picture

`tools/mkheader.py <400x300 png> Comic.h` converts a 400x300 PNG (colours rounded to 0/85/170/255 per channel) into `Comic.h`.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. The MCE Blaster itself is GPLv2 by Scrap Computing.
