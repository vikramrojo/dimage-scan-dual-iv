#!/usr/bin/env python3
"""
Differential test of the auto-exposure kernel port (src/minolta_ae.c)
against the original FUN_10036090 of MFSLib2891 running in the emulator.

    ae_diff.py [MFSLib2891] [--quick] [--seed N] [--real FILE.raw WxH]...

Every case builds an AutoExposure-style input (16 bit pixel-interleaved
prescan, one-line black / light reference lines, AE area, film type,
product type), runs the original via emu.py and the port via ae_driver.c,
and compares the return value and all 0x70 result bytes bit for bit.

Synthetic prescans: colour negative with orange mask, slides, B&W
negative/positive, very dark, clipped, low contrast, gradients, noise,
several sizes (up to 780x1170) and AE areas, RGB / RGBI / mono layouts.
Film types 5 (both gain modes), 6, 10 and the unsupported ones (1, 2, 9,
0x100, 0x200, 0x400 -> -1), products 3,4,8,9,11,12 and an invalid one.
--real adds a headerless little-endian RGB16 file (e.g. a real scan,
box-downsampled to prescan size) analysed as film types 2, 6 and 10.
"""
import array
import math
import os
import random
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, HERE)
from emu import Emu, STACK_TOP  # noqa: E402

AE = 0x10036090


def build_driver():
    if os.environ.get('AE_DRIVER'):
        return os.environ['AE_DRIVER']
    out = os.path.join(tempfile.gettempdir(), 'dsd4_ae_driver')
    src = [os.path.join(HERE, 'ae_driver.c'), os.path.join(ROOT, 'src', 'minolta_ae.c')]
    deps = src + [os.path.join(ROOT, 'src', 'minolta_ae.h')]
    if not os.path.exists(out) or any(os.path.getmtime(s) > os.path.getmtime(out) for s in deps):
        subprocess.check_call(['clang', '-O2', '-std=c11', '-ffp-contract=off',
                               '-I' + os.path.join(ROOT, 'src')] + src + ['-o', out])
    return out


# ------------------------------------------------------------ images

def clip16(v):
    return 0 if v < 0 else 65535 if v > 65535 else int(v)


def make_image(rng, kind, w, h, nch):
    """pixel interleaved u16 samples (array('H'), host order)"""
    a = array.array('H', bytes(2 * w * h * nch))
    # per-kind parameters: base level per channel, contrast, noise
    if kind in ('cneg', 'sky', 'bimodal'):  # orange mask: R high, B low, scene modulates density
        base, amp = [rng.uniform(30000, 52000), rng.uniform(16000, 32000), rng.uniform(7000, 18000)], rng.uniform(0.35, 0.8)
    elif kind == 'slide':
        base, amp = [rng.uniform(15000, 40000)] * 3, rng.uniform(0.6, 0.98)
    elif kind == 'bwneg':
        v = rng.uniform(20000, 45000)
        base, amp = [v * rng.uniform(.97, 1.03) for _ in range(3)], rng.uniform(0.4, 0.8)
    elif kind == 'bwpos':
        base, amp = [rng.uniform(20000, 40000)] * 3, rng.uniform(0.7, 0.98)
    elif kind == 'dark':
        base, amp = [rng.uniform(300, 3000) for _ in range(3)], rng.uniform(0.2, 0.9)
    elif kind == 'bright':
        base, amp = [rng.uniform(60000, 90000) for _ in range(3)], rng.uniform(0.05, 0.4)
    elif kind == 'flat':
        base, amp = [rng.uniform(8000, 50000) for _ in range(3)], rng.uniform(0.0, 0.05)
    elif kind == 'noise':
        base, amp = [rng.uniform(5000, 60000) for _ in range(3)], 1.0
    else:  # gradient
        base, amp = [rng.uniform(10000, 60000) for _ in range(3)], rng.uniform(0.3, 1.0)
    noise = rng.uniform(0, 2500)
    gam = [rng.uniform(0.85, 1.15) for _ in range(3)]
    fx, fy, ph = rng.uniform(0.5, 4), rng.uniform(0.5, 4), rng.uniform(0, 6.3)
    blobs = [(rng.uniform(0, w), rng.uniform(0, h), rng.uniform(w / 12, w / 3), rng.uniform(-1, 1))
             for _ in range(rng.randint(0, 5))]
    border = rng.choice([0, 0, w // 30])
    # 'sky': a large uniform patch -> narrow histogram spike (flare paths)
    patch = (rng.uniform(0, .5) * w, rng.uniform(0, .5) * h, rng.uniform(.3, .7) * w, rng.uniform(.3, .7) * h)
    plev, pnoise = rng.uniform(0.5, 1.0), rng.uniform(0, 300)
    k = 0
    for y in range(h):
        for x in range(w):
            if kind == 'noise':
                s = rng.random()
            elif kind == 'grad':
                s = (x / w + y / h) / 2
            elif kind == 'bimodal':  # dense patch on a thin, bright frame
                inside = patch[0] <= x < patch[0] + patch[2] and patch[1] <= y < patch[1] + patch[3]
                s = (0.85 + 0.1 * rng.random()) if inside else 0.05 * rng.random()
            elif kind == 'sky' and patch[0] <= x < patch[0] + patch[2] and patch[1] <= y < patch[1] + patch[3]:
                s = plev
            else:
                s = 0.5 + 0.35 * math.sin(fx * 6.28 * x / w + ph) * math.cos(fy * 6.28 * y / h)
                for bx, by, br, bv in blobs:
                    d = ((x - bx) ** 2 + (y - by) ** 2) / (br * br)
                    if d < 1:
                        s += 0.5 * bv * (1 - d)
                s = min(max(s, 0.0), 1.0)
            edge = border and (x < border or y < border or x >= w - border or y >= h - border)
            for c in range(nch):
                if c >= 3:
                    v = 30000 + rng.uniform(-noise, noise)
                elif edge:  # unexposed film rim / holder
                    v = base[c] * 1.15 if kind in ('cneg', 'bwneg') else 600
                else:
                    t = 1 - amp * (s ** gam[c])
                    v = base[c] * t + rng.uniform(-noise, noise)
                    if kind == 'sky' and s == plev:
                        v = base[c] * t + rng.uniform(-pnoise, pnoise)
                a[k] = clip16(v)
                k += 1
    return a


def make_line(rng, w, nch, lo, hi):
    a = array.array('H', bytes(2 * w * nch))
    lev = [rng.uniform(lo, hi) for _ in range(nch)]
    for x in range(w):
        prof = 1 - 0.15 * ((2 * x / w - 1) ** 2)
        for c in range(nch):
            a[x * nch + c] = clip16(lev[c] * prof + rng.uniform(-200, 200))
    return a


# ------------------------------------------------------------ cases

class Case:
    """film, product, flag, area, buffers (array('H')), 9 plane descriptors"""

    def __init__(self, name, film, product, flag, area, bufs, planes):
        self.name, self.film, self.product, self.flag = name, film, product, flag
        self.area, self.bufs, self.planes = area, bufs, planes


def planes_for(w, h, nch, mono, slack):
    """GetAEInputData layout: buffer 0 image, 1 dark line, 2 light line"""
    pl = []
    for c in range(3):
        if mono and c:
            pl.append((0xFFFFFFFF, 0, 0, 0, 0, 0, 0))
        else:
            pl.append((0, 2 * c, w, h, 2, 2 * nch, 2 * nch * w))
    for b in (1, 2):
        for c in range(3):
            if mono and c:
                pl.append((0xFFFFFFFF, 0, 0, 0, 0, 0, 0))
            else:
                pl.append((b, 2 * (1 if mono else c), w, 1, 2, 2 * 3, 0))
    return pl


def gen_case(rng, idx, quick):
    kinds = ['cneg', 'sky', 'bimodal', 'slide', 'bwneg', 'bwpos', 'dark', 'bright', 'flat', 'noise', 'grad']
    kind = kinds[idx % len(kinds)] if idx < 18 else rng.choice(kinds)
    sizes = [(780, 1170), (1170, 780), (256, 384), (200, 300), (96, 144), (72, 80), (40, 48), (33, 260)]
    w, h = sizes[0] if (idx in (0, 1) and not quick) else rng.choice(sizes[2 if quick else 1:])
    if kind in ('cneg', 'sky', 'bimodal'):
        film = rng.choice([6, 6, 6, 5])
    elif kind in ('bwneg', 'bwpos'):
        film = rng.choice([10, 10, 6, 5])
    else:
        film = rng.choice([5, 6, 10, 6, 5, 10])
    if rng.random() < 0.06:
        film = rng.choice([1, 2, 9, 0x100, 0x200, 0x400, 0])
    mono = film == 10 and rng.random() < 0.5
    nch = 1 if mono else rng.choice([3, 3, 4])
    product = rng.choice([3, 4, 8, 9, 11, 12, 8, 8])
    if idx % 25 == 7:  # colour negative with a product FUN_1003eff0 rejects
        film, w, h, area = 6, 96, 144, (0, 0, 96, 144)
    if rng.random() < 0.05 or idx % 25 == 7:
        product = rng.choice([0, 1, 2, 5, 13])
    flag = rng.choice([0, 1])
    if rng.random() < 0.6:
        area = (0, 0, w, h)
    else:
        aw = rng.randint(min(32, w), w)
        ah = rng.randint(min(32, h), h)
        area = (rng.randint(0, w - aw), rng.randint(0, h - ah), aw, ah)
        if rng.random() < 0.05:
            area = (0, 0, rng.randint(8, 31), h)  # too small -> -1
    # film 6 adds the area origin to block positions taken over the whole
    # image (FUN_1003a540), so FUN_1003ab30 may read the reference line past
    # its width: give the lines real data up to 2w + 64 samples
    slack = w + 64
    # FUN_1003a540/FUN_1003af80 shift the block grid by ((size & 7) >> 1)
    # without clipping, so an AE area touching the bottom edge reads up to 3
    # lines past the image: append 8 real lines (both sides see the same bytes)
    img = make_image(rng, kind, w, h + 8, nch)
    dark = make_line(rng, w + slack, 3, 200, 3000)
    light = make_line(rng, w + slack, 3, 30000, 65000)
    return Case('%d:%s %dx%d nch%d film%#x prod%d flag%d area%s' % (idx, kind, w, h, nch, film, product, flag, area),
                film, product, flag, area, [img, dark, light], planes_for(w, h, nch, mono, slack))


def real_cases(path, w, h):
    raw = open(path, 'rb').read()
    img = array.array('H')
    img.frombytes(raw[:2 * 3 * w * h])
    img.extend(img[len(img) - 3 * w * 8:])  # 8 lines past the image (see gen_case)
    if sys.byteorder != 'little':
        img.byteswap()
    rng = random.Random(1)
    dark = make_line(rng, 2 * w + 64, 3, 400, 1200)
    light = make_line(rng, 2 * w + 64, 3, 50000, 60000)
    out = []
    for film in (2, 6, 10):
        for area in ((0, 0, w, h), (w // 8, h // 8, 3 * w // 4, 3 * h // 4)):
            out.append(Case('real %s film%d area%s' % (os.path.basename(path), film, area), film, 8, 0, area,
                            [img, dark, light], planes_for(w, h, 3, False, 64)))
    return out


# ------------------------------------------------------------ runners

def init_result(rng):
    # random real fields (exercise "not written" semantics), zero padding
    raw = bytearray(rng.getrandbits(8) for _ in range(0x70))
    for c in range(3):
        raw[0x30 + 0x14 * c + 11] = 0
    raw[0x6c:0x70] = bytes(4)
    return bytes(raw)


def run_port(drv, case, init):
    hdr = struct.pack('<7I', case.film, case.product, case.flag, *case.area)
    for p in case.planes:
        hdr += struct.pack('<7I', *p)
    hdr += init
    with tempfile.NamedTemporaryFile(delete=False) as f:
        f.write(hdr)
        for b in case.bufs:
            d = b.tobytes() if sys.byteorder == 'little' else array.array('H', b).byteswap() or b.tobytes()
            f.write(struct.pack('<I', len(d)) + d)
        name = f.name
    outn = name + '.out'
    subprocess.check_call([drv, name, outn])
    r = open(outn, 'rb').read()
    os.unlink(name)
    os.unlink(outn)
    return struct.unpack('<i', r[:4])[0], r[4:]


def run_orig(m, case, init):
    m.reset_heap()
    m.write(STACK_TOP - 0x200000, bytes(0x210000))  # deterministic stale stack
    addrs = []
    for b in case.bufs:
        be = array.array('H', b)
        if sys.byteorder == 'little':
            be.byteswap()
        addrs.append(m.alloc(len(be) * 2 + 64, be.tobytes() + bytes(64)))
    inp = bytearray(0xf4)
    struct.pack_into('>I', inp, 0, case.film)
    for i, (bi, off, w, h, nb, ps, rs) in enumerate(case.planes):
        ptr = 0 if bi == 0xFFFFFFFF else addrs[bi] + off
        o = 4 + 0x18 * i
        struct.pack_into('>III', inp, o, ptr, w, h)
        inp[o + 0xc] = nb
        struct.pack_into('>II', inp, o + 0x10, ps, rs)
    struct.pack_into('>4I', inp, 0xdc, *case.area)
    inp[0xec] = case.flag
    struct.pack_into('>I', inp, 0xf0, case.product)
    ia = m.alloc(0xf4, inp)
    ra = m.alloc(0x70, init)
    r3, _ = m.call(AE, [ia, ra])
    return struct.unpack('>i', struct.pack('>I', r3))[0], m.read(ra, 0x70)


FIELDS = [('gain[%d]' % c, 8 * c, 8) for c in range(3)] + \
    [('ch[%d].%s' % (c, n), 0x18 + 8 * c + o, 2) for c in range(3) for n, o in
     (('exposure', 0), ('dark', 2), ('r1c', 4), ('r1e', 6))] + \
    [('tone[%d].%s' % (c, n), 0x30 + 0x14 * c + o, s) for c in range(3) for n, o, s in
     (('level', 0, 2), ('k46', 2, 2), ('lo', 4, 2), ('hi', 6, 2), ('shift', 8, 2), ('mode', 10, 1),
      ('weight', 12, 8))]


def describe(a, b):
    out = []
    for n, o, s in FIELDS:
        if a[o:o + s] != b[o:o + s]:
            if s == 8:
                x, y = struct.unpack('>d', a[o:o + 8])[0], struct.unpack('>d', b[o:o + 8])[0]
                out.append('%s orig %r port %r' % (n, x, y))
            else:
                out.append('%s orig %s port %s' % (n, a[o:o + s].hex(), b[o:o + s].hex()))
    return '; '.join(out)


def main():
    args = sys.argv[1:]
    quick = '--quick' in args
    seed = 1
    reals = []
    if '--seed' in args:
        seed = int(args[args.index('--seed') + 1])
    i = 0
    rest = []
    while i < len(args):
        if args[i] == '--real':
            w, h = map(int, args[i + 2].split('x'))
            reals.append((args[i + 1], w, h))
            i += 3
        elif args[i] == '--seed':
            i += 2
        elif args[i].startswith('--'):
            i += 1
        else:
            rest.append(args[i])
            i += 1
    lib = rest[0] if rest else os.path.join(ROOT, '..', 'DS_Dual4_extracted', 'MFSLib2891')
    m = Emu(lib)
    drv = build_driver()
    rng = random.Random(seed)
    cases = [gen_case(rng, i, quick) for i in range(12 if quick else 120)]
    for p, w, h in reals:
        cases += real_cases(p, w, h)
    bad = 0
    stats = {}
    for case in cases:
        init = init_result(rng)
        ro, a = run_orig(m, case, init)
        rp, b = run_port(drv, case, init)
        ok = ro == rp and a == b
        stats.setdefault(ro, 0)
        stats[ro] += 1
        if not ok:
            bad += 1
            print('MISMATCH %s: rc orig %d port %d; %s' % (case.name, ro, rp, describe(a, b)))
        else:
            g = struct.unpack('>3d', a[:24])
            print('ok   %s rc=%d gains=%.5f %.5f %.5f' % (case.name, ro, *g))
        sys.stdout.flush()
    print('%d cases, %d mismatches, return codes %s' % (len(cases), bad, stats))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
