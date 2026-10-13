#!/usr/bin/env python3
"""Measurements on dsdual4's TIFF/DNG output for the simulator tests
(standard library only).

  imgcheck.py neutral IN.tiff            mean |R-G|, |B-G| (fraction of full scale)
  imgcheck.py noise CLEAN NOISY [dark]   rms difference (green), optionally only
                                         where CLEAN < dark
  imgcheck.py dng IN.dng                 DNG structure summary
  imgcheck.py mean IN.bmp                mean level of a 24-bit BMP (0..1), e.g. a
                                         DNG as decoded by macOS (sips -s format bmp)
"""
import array
import struct
import sys


def read(path):
    d = open(path, 'rb').read()
    assert d[:2] == b'II', 'little-endian TIFF/DNG expected'
    ifd = struct.unpack_from('<I', d, 4)[0]
    n = struct.unpack_from('<H', d, ifd)[0]
    tags = {}
    for i in range(n):
        tag, typ, cnt = struct.unpack_from('<HHI', d, ifd + 2 + 12 * i)
        size = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 7: 1, 10: 8}[typ] * cnt
        off = ifd + 2 + 12 * i + 8 if size <= 4 else struct.unpack_from('<I', d, ifd + 2 + 12 * i + 8)[0]
        fmt = {1: 'B', 2: 'c', 3: 'H', 4: 'I', 5: 'II', 7: 'B', 10: 'ii'}[typ]
        if typ in (5, 10):
            v = struct.unpack_from('<' + fmt[0] * (2 * cnt), d, off)
            tags[tag] = [v[2 * k] / v[2 * k + 1] for k in range(cnt)]
        elif typ == 2:
            tags[tag] = d[off:off + cnt].rstrip(b'\0').decode()
        else:
            tags[tag] = list(struct.unpack_from('<%d%s' % (cnt, fmt), d, off))
    w, h, spp = tags[256][0], tags[257][0], tags.get(277, [1])[0]
    bps, sf = tags[258][0], tags.get(339, [1])[0]
    px = b''.join(d[o:o + c] for o, c in zip(tags[273], tags[279]))
    if sf == 3 and bps == 16:
        v = [x[0] for x in struct.iter_unpack('<e', px)]
    elif sf == 3:
        v = array.array('f', px)
    elif bps == 16:
        v = [x / 65535.0 for x in array.array('H', px)]
    else:
        v = [x / 255.0 for x in px]
    return tags, w, h, spp, v


def bmp_mean(path):
    d = open(path, 'rb').read()
    assert d[:2] == b'BM', 'BMP expected'
    off = struct.unpack_from('<I', d, 10)[0]
    w, h, _, bpp, comp = struct.unpack_from('<iihhI', d, 18)
    assert bpp == 24 and comp == 0, '24-bit uncompressed BMP expected'
    stride = (w * 3 + 3) & ~3
    s = sum(sum(d[off + y * stride:off + y * stride + w * 3]) for y in range(abs(h)))
    return s / (255.0 * w * 3 * abs(h))


def main():
    cmd = sys.argv[1]
    if cmd == 'neutral':
        _, w, h, spp, v = read(sys.argv[2])
        n = w * h
        dr = sum(abs(v[3 * i] - v[3 * i + 1]) for i in range(0, n, 3)) / (n / 3)
        db = sum(abs(v[3 * i + 2] - v[3 * i + 1]) for i in range(0, n, 3)) / (n / 3)
        print('%.4f %.4f' % (dr, db))
    elif cmd == 'noise':
        _, w, h, spp, a = read(sys.argv[2])
        _, _, _, _, b = read(sys.argv[3])
        dark = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
        c = 1 if spp == 3 else 0
        s, k = 0.0, 0
        for y in range(8, h - 8):  # HDR edge rows use the short pass only
            for x in range(w):
                i = (y * w + x) * spp + c
                if a[i] < dark:
                    s += (b[i] - a[i]) ** 2
                    k += 1
        print('%.6f %d' % ((s / k) ** 0.5 if k else -1, k))
    elif cmd == 'dng':
        t, w, h, spp, v = read(sys.argv[2])
        print('DNGVersion %s photometric %d %dx%d spp %d bps %d sampleformat %d colormatrix %s neutral %s' % (
            '.'.join(map(str, t[50706])), t[262][0], w, h, spp, t[258][0], t.get(339, [1])[0],
            'yes' if 50721 in t else 'no', ['%.3f' % x for x in t.get(50728, [])]))
    elif cmd == 'mean':
        print('%.4f' % bmp_mean(sys.argv[2]))


main()
