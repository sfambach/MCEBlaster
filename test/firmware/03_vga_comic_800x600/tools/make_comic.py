# Copyright (C) 2026 Stefan Fambach (SteFam), https://www.fambach.net
# License: PolyForm Noncommercial 1.0.0, see LICENSE.md in the parent folder.
#
# Turns a photo into the 400x300, 64 colour comic picture of step 3: draws
# a suit, crops to 4:3, reduces to flat colours, adds ink lines and text.

import sys
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageOps, ImageEnhance, ImageFont

# $ python3 make_comic.py photo.png Comic  (writes Comic.png)
src, out = sys.argv[1], sys.argv[2]
im = Image.open(src).convert('RGB')

# Suit, in source pixel coordinates.
d = ImageDraw.Draw(im)
Jacket = (70, 70, 80)
Lapel = (95, 95, 105)
d.polygon([(0, 435), (0, 385), (70, 345), (160, 318), (178, 330), (222, 435)], fill=Jacket)
d.polygon([(477, 435), (477, 350), (400, 318), (300, 300), (288, 330), (240, 435)], fill=Jacket)
d.polygon([(178, 330), (222, 435), (244, 435), (288, 330), (262, 360), (233, 372), (204, 360)], fill=(245, 245, 245))
d.polygon([(222, 372), (244, 372), (240, 390), (226, 390)], fill=(200, 20, 20))
d.polygon([(226, 390), (240, 390), (250, 425), (233, 435), (216, 425)], fill=(220, 30, 30))
d.line([(178, 330), (160, 385), (212, 430)], fill=(20, 20, 20), width=3)
d.line([(288, 330), (306, 385), (254, 430)], fill=(20, 20, 20), width=3)

# 4:3 crop from y=45, padded left and right with the wall.
top = 0
h = im.height - top
w = h * 4 // 3
canvas = Image.new('RGB', (w, h))
crop = im.crop((0, top, im.width, im.height))
off = (w - im.width) // 2
canvas.paste(crop, (off, 0))
left = crop.crop((0, 0, 1, h)).resize((off, h))
right = crop.crop((im.width - 1, 0, im.width, h)).resize((w - off - im.width, h))
canvas.paste(left, (0, 0))
canvas.paste(right, (off + im.width, 0))

img = canvas.resize((400, 300), Image.LANCZOS)
img = ImageOps.autocontrast(img, cutoff=1)
a = np.asarray(img).astype(np.float32) / 255.0
a = a ** 0.75
img = Image.fromarray((a * 255).astype(np.uint8))
smooth = img.filter(ImageFilter.MedianFilter(5)).filter(ImageFilter.MedianFilter(5))

# k-means to a few flat comic colours, each mapped to the 64 colour palette.
a = np.asarray(smooth).astype(np.float32).reshape(-1, 3)
rng = np.random.default_rng(1)
K = 12
C = a[rng.choice(len(a), K, replace=False)]
for _ in range(25):
    lab = np.argmin(((a[:, None, :] - C[None]) ** 2).sum(-1), 1)
    for k in range(K):
        if (lab == k).any():
            C[k] = a[lab == k].mean(0)
lev = np.array([0, 85, 170, 255], np.float32)
pal = np.array([[r, g, b] for r in lev for g in lev for b in lev], np.float32)
def luma(x): return x @ np.array([0.30, 0.59, 0.11], np.float32)
def nearest(c):
    dl = (luma(pal) - luma(c)) ** 2
    dc = 1.0 * (((pal - luma(pal)[:, None]) - (c - luma(c))) ** 2).sum(-1)
    return pal[np.argmin(1.5 * dl + dc)]
L = luma(C)[:, None]
Cs = np.clip(L + (C - L) * 2.2, 0, 255)
mapped = np.array([nearest(c) for c in Cs])
q = (mapped[lab] / 85).astype(np.uint8).reshape(300, 400, 3)

# Ink lines from a difference of blurs.
g = img.convert('L')
g1 = np.asarray(g.filter(ImageFilter.GaussianBlur(1.0))).astype(np.float32)
g2 = np.asarray(g.filter(ImageFilter.GaussianBlur(2.2))).astype(np.float32)
ink = (g2 - g1) > 6
q[ink] = 0

rgb = Image.fromarray((q * 85).astype(np.uint8))
dr = ImageDraw.Draw(rgb)
dr.fontmode = '1'
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 15)
small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 11)
# Speech bubble.
dr.ellipse((8, 26, 152, 80), fill=(255, 255, 255), outline=(0, 0, 0), width=2)
dr.polygon([(118, 70), (140, 74), (158, 100)], fill=(255, 255, 255), outline=(0, 0, 0))
dr.line([(119, 71), (139, 73)], fill=(255, 255, 255), width=3)
dr.text((80, 45), 'Greetings', font=font, fill=(0, 0, 0), anchor='mm')
dr.text((80, 62), 'from SteFam!', font=font, fill=(0, 0, 0), anchor='mm')
# Caption.
dr.rectangle((0, 0, 399, 17), fill=(255, 255, 0))
dr.line([(0, 18), (399, 18)], fill=(0, 0, 0))
dr.text((200, 9), 'MCE BLASTER 800x600 @ 56 Hz  -  Thanks to Scrap Computing', font=small, fill=(0, 0, 0), anchor='mm')
# White frame.
dr.rectangle((0, 0, 399, 299), outline=(255, 255, 255))

a = np.asarray(rgb)
q = np.clip(np.round(a / 85.0), 0, 3).astype(np.uint8)
Image.fromarray((q * 85).astype(np.uint8)).save(out + '.png')
