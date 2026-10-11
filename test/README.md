# Tests

Test programs for the MCE Blaster, collected in one place.

## Layout

| Folder | Content |
|---|---|
| `firmware/` | Test firmware for the Pico on the MCE Blaster, numbered as a step-by-step commissioning guide (see below). |
| `dos/` | Programs that run on the IBM PC / XT / AT and drive the MDA, CGA or EGA card (test patterns, mode switches, colour bars). |
| `host/` | Programs that run on a PC/Linux host, e.g. to read the Pico over USB. |

Each test lives in its own subfolder with a short `README.md`: what it shows, how to build it, what the expected picture is.

## Commissioning steps (test firmware)

| Step | Folder | Checks |
|---|---|---|
| 1 | [`firmware/01_vga_testimage`](firmware/01_vga_testimage/) | VGA output: static test image (frame, mode, frequencies), one UF2 per mode |
| 2 | [`firmware/02_vga_testimage_switch`](firmware/02_vga_testimage_switch/) | Buttons and all VGA modes: switch modes with PIXEL CLOCK / AUTO ADJUST |

Each step stays as it is; later steps build on the earlier ones in a new folder.

## AI disclosure

This folder was created with the help of an AI assistant (Claude); tests added here may be written with AI support as well.
