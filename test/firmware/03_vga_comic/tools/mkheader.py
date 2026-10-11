# Converts the 400x300 comic canvas into an RLE C header (6-bit RRGGBB).
import sys
from PIL import Image
im = Image.open(sys.argv[1]).convert('RGB')
assert im.size == (400, 300)
px = im.load()
def c6(r, g, b):
    return ((r // 85) << 4) | ((g // 85) << 2) | (b // 85)  # RR GG BB, R in bits 4-5
data = []
for y in range(300):
    row = [c6(*px[x, y]) for x in range(400)]
    x = 0
    while x < 400:
        n = 1
        while x + n < 400 and n < 255 and row[x + n] == row[x]:
            n += 1
        data += [n, row[x]]
        x += n
out = ['// Generated from the comic portrait, do not edit.',
       '// 400x300 pixels, 6-bit colour, run-length encoded per line as',
       '// (count, colour) pairs; every line holds exactly 400 pixels.',
       '#pragma once', '#include <cstdint>', '',
       'static constexpr uint32_t ComicW = 400;',
       'static constexpr uint32_t ComicH = 300;',
       f'static const uint8_t ComicRLE[{len(data)}] = {{']
for i in range(0, len(data), 16):
    out.append('    ' + ', '.join(f'0x{v:02x}' for v in data[i:i + 16]) + ',')
out.append('};')
open(sys.argv[2], 'w').write('\n'.join(out) + '\n')
print(len(data), 'bytes')
