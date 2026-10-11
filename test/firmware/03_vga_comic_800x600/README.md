# Step 3: comic picture at 800x600

A full-screen comic picture (SteFam in a suit) at 800x600@56 Hz.

- The picture is 400x300 pixels with all 64 colours of the MCE Blaster DAC (2 bits per colour).
- Every byte is held for two pixel clocks and every line is sent twice, so it fills 800x600. The PIO runs at 18 MHz (252 MHz / 14) and all horizontal timing values are halved.
- No buttons; status over USB serial every 2 seconds.

This checks all 64 colours, the full visible area and the 800x600 mode.

![Comic](Comic.png)

## Build

```
mkdir build && cd build
cmake -DPICO_BOARD=<pico|pico2> -DPICO_SDK_PATH=/path/to/pico-sdk ..
make -j
```

Copy `VGAComic.uf2` to the Pico (BOOTSEL). Flash the normal MCE Blaster firmware again afterwards.

## Use your own picture

`Picture.h` is generated from `Comic.png`, do not edit it by hand. To show your own picture:

1. Install Python 3 with Pillow (`pip install pillow`) and the pico-sdk.
2. Convert the picture; any size and format works, it is cropped to 4:3 around the centre and scaled to 400x300:
   ```
   python3 tools/png2header.py MyPicture.jpg Picture.h
   ```
   For photos add `--dither`: it mixes neighbouring pixels so gradients look smoother.
3. Check `Picture_preview.png`: that is what the monitor will show.
4. Build as above and copy `VGAComic.uf2` to the Pico.

Tips for a nice picture with only 64 colours (4 levels per colour):

- Flat areas and strong outlines work best: comics, logos, pixel art, posters.
- Without `--dither`, photos turn into a posterised comic look; skin tones and soft gradients get blotchy.
- Put the important part in the middle, so the 4:3 crop does not cut it off.
- Text should be at least about 10 pixels high in the 400x300 picture.

`tools/make_comic.py` made `Comic.png` from the original photo (draws the suit, adds the speech bubble and the banner). It is tuned to that one photo; use it as an example, not for other pictures.

## What changed compared to step 2

- One fixed mode, no buttons.
- One line buffer per picture line (300 x 512 bytes in RAM); the line table points to each buffer twice.

## License

Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net

PolyForm Noncommercial 1.0.0, see [LICENSE.md](LICENSE.md): free to use, change and share for noncommercial purposes; commercial use only with my consent. This also covers the picture. The MCE Blaster itself is GPLv2 by Scrap Computing.

## Prebuilt UF2

Ready-to-flash files are in [`uf2/pico`](uf2/pico/) (RP2040) and [`uf2/pico2`](uf2/pico2/) (RP2350).
