#!/usr/bin/env python3
"""
Differential test of the tone-curve / LUT kernel port (src/minolta_lut.c)
against the original MFSLib2891 running in the emulator (emu.py).

    lut_diff.py [MFSLib2891] [--quick] [--seed N]

Compares bit for bit, for many parameter sets:
  * FUN_10036f40 (mfs_lut_film_curve): the three film LUTs, return value and
    the FUN_10040990 statics, for film types 5, 6 (fixed and AE), 9, 10
    (fixed and AE), 0x100, 0x200 and invalid ones, several product types and
    LUT bit-depth codes;
  * FUN_10037280 (mfs_lut_apply_user_curve): final LUT and stretch factor for
    film curves from the kernel composed with random monotone / S / inverted
    user curves of several depths, and without user curve;
  * FUN_10037d70 (mfs_lut_convert): every in/out code combination.

The native driver (lut_driver.c) is built with clang into a temp directory.
"""
import os
import random
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
sys.path.insert(0, HERE)
from emu import Emu  # noqa: E402

MAX = [255, 1023, 4095, 16383, 65535]
LUT_SCRATCH = 0x20000 + 16

KERNEL, USERCURVE, CONVERT = 0x10036F40, 0x10037280, 0x10037D70


def build_driver():
    out = os.path.join(tempfile.gettempdir(), 'dsd4_lut_driver')
    src = [os.path.join(HERE, 'lut_driver.c'), os.path.join(ROOT, 'src', 'minolta_lut.c')]
    if not os.path.exists(out) or any(os.path.getmtime(s) > os.path.getmtime(out) for s in
                                      src + [os.path.join(ROOT, 'src', 'minolta_lut.h')]):
        subprocess.check_call(['clang', '-O2', '-std=c11', '-ffp-contract=off',
                               '-I' + os.path.join(ROOT, 'src')] + src + ['-o', out])
    return out


def le(*words):
    return b''.join(struct.pack('<I', w & 0xFFFFFFFF) for w in words)


def tbytes(code_in, code_out):
    return (MAX[code_in] + 1) * (2 if code_out else 1)


def dbits(d):
    u = struct.unpack('>Q', struct.pack('>d', d))[0]
    return u >> 32, u & 0xFFFFFFFF


class Native:
    def __init__(self, exe):
        self.exe = exe

    def run(self, data):
        return subprocess.run([self.exe], input=data, stdout=subprocess.PIPE, check=True).stdout


def ae_blob(gain, levels, nega, pad=0):
    """MFSIPAEResult, big-endian 0x70 bytes"""
    b = bytearray(0x70)
    for c in range(3):
        struct.pack_into('>d', b, 8 * c, gain[c])
        struct.pack_into('>4H', b, 0x18 + 8 * c, *levels[c])
        n = nega[c]
        struct.pack_into('>4HhBBd', b, 0x30 + 0x14 * c, n['contrast'], n['base'], n['ref_level'],
                         n['ref_target'], n['adj'], n['mode'], n.get('pad', 0), n['balance'])
    struct.pack_into('>I', b, 0x6c, pad)
    return bytes(b)


class Tester:
    def __init__(self, lib, exe):
        self.m = Emu(lib)
        self.nat = Native(exe)
        self.fails = 0
        self.cases = 0
        self.ulp_notes = []

    # ------------------------------------------------------------ helpers
    def emu_table(self, codes, data=None):
        n = tbytes(*codes)
        return self.m.alloc(max(n, LUT_SCRATCH), data)

    def compare(self, name, a, b, extra=''):
        self.cases += 1
        if a == b:
            return True
        self.fails += 1
        # first differing entries
        diffs = [i for i in range(min(len(a), len(b))) if a[i] != b[i]]
        print('FAIL %s %s: %d differing bytes, first at %s (len %d/%d)' %
              (name, extra, len(diffs), diffs[:8], len(a), len(b)))
        return False

    # ------------------------------------------------------ FUN_10036f40
    def film_curve(self, name, film_type, fixed, gains, ptype, ae, codes, state=(0.0, 0)):
        m = self.m
        m.reset_heap()
        words = [film_type, (fixed & 0xFF) << 24]
        for g in gains:
            words += list(dbits(g))
        words += [ptype, 0xDEADBEEF]
        words += list(struct.unpack('>28I', ae))
        tabs = [self.emu_table(c) for c in codes]
        desc = m.alloc(36, b''.join(struct.pack('>3I', tabs[i], *codes[i]) for i in range(3)))
        words.append(desc)
        # FUN_10040990 statics
        m.write(m.toc - 0x77F0, struct.pack('>dI', state[0], state[1]))
        r3, _ = m.call_words(KERNEL, words)
        e_ratio, e_ref = struct.unpack('>dI', m.read(m.toc - 0x77F0, 12))
        e_tabs = [m.read(tabs[i], tbytes(*codes[i])) for i in range(3)]

        inp = le(1, *dbits(state[0]), state[1], *words) + b''.join(le(*c) for c in codes)
        out = self.nat.run(inp)
        n_ret, = struct.unpack('<i', out[:4])
        hi, lo, n_ref = struct.unpack('<3I', out[4:16])
        n_ratio = struct.unpack('>d', struct.pack('>II', hi, lo))[0]
        pos = 16
        ok = self.compare(name + ' ret', struct.pack('>i', r3 - (1 << 32) if r3 & 0x80000000 else r3),
                          struct.pack('>i', n_ret))
        if r3 == 0:
            ok &= self.compare(name + ' state', struct.pack('>dI', e_ratio, e_ref),
                               struct.pack('>dI', n_ratio, n_ref))
            for c in range(3):
                n = tbytes(*codes[c])
                ok &= self.compare(name, e_tabs[c], out[pos:pos + n], 'ch%d' % c)
                pos += n
        return ok, (e_ratio, e_ref), e_tabs

    # ------------------------------------------------------ FUN_10037280
    def user_curve(self, name, film_codes, film_tab, user, dst_codes):
        m = self.m
        m.reset_heap()
        f = self.emu_table(film_codes, film_tab)
        if user is not None:
            ucodes, utab = user
            u = self.emu_table(ucodes, utab)
        else:
            ucodes, u = (4, 4), 0
        d = self.emu_table(dst_codes)
        g = m.alloc(8, struct.pack('>d', -1.0))
        r3, _ = m.call_words(USERCURVE, [f, film_codes[0], film_codes[1], u, ucodes[0], ucodes[1],
                                         d, dst_codes[0], dst_codes[1], g])
        e_k = m.read(g, 8)
        e_tab = m.read(d, tbytes(*dst_codes))
        inp = le(2, *film_codes) + film_tab
        inp += le(1, *ucodes) + user[1] if user is not None else le(0)
        inp += le(*dst_codes)
        out = self.nat.run(inp)
        n_ret, hi, lo = struct.unpack('<iII', out[:12])
        ok = self.compare(name + ' ret', struct.pack('>I', r3), struct.pack('>i', n_ret))
        ok &= self.compare(name + ' stretch', e_k, struct.pack('>II', hi, lo),
                           '%r' % struct.unpack('>d', e_k))
        ok &= self.compare(name, e_tab, out[12:])
        return ok, struct.unpack('>d', e_k)[0]

    # ------------------------------------------------------ FUN_10037d70
    def convert(self, name, src_codes, src_tab, dst_codes):
        m = self.m
        m.reset_heap()
        s = self.emu_table(src_codes, src_tab)
        d = self.emu_table(dst_codes)
        r3, _ = m.call_words(CONVERT, [s, src_codes[0], src_codes[1], d, dst_codes[0], dst_codes[1]])
        e_tab = m.read(d, tbytes(*dst_codes))
        out = self.nat.run(le(3, *src_codes) + src_tab + le(*dst_codes))
        ok = self.compare(name + ' ret', struct.pack('>I', r3), out[:4][::-1])
        ok &= self.compare(name, e_tab, out[4:])
        return ok


# ------------------------------------------------------------ generators
def nega_params(rng, realistic=True):
    if realistic:
        return dict(contrast=rng.randint(30, 80), base=rng.randint(120, 230),
                    ref_level=rng.randint(10, 80), ref_target=rng.randint(180, 240),
                    adj=rng.randint(-10, 10), mode=rng.choice([0, 1, 4]),
                    balance=rng.choice([0.0, 0.5, 1.0, rng.random()]))
    return dict(contrast=rng.choice([0, 5, 10, 100, 150, 0xFFFF, rng.randint(0, 200)]),
                base=rng.choice([0, 1, 255, 300, rng.randint(0, 0xFFFF)]),
                ref_level=rng.choice([0, 255, rng.randint(0, 300)]),
                ref_target=rng.choice([0, 255, rng.randint(0, 300)]),
                adj=rng.choice([-32768, -100, 0, 100, 32767, rng.randint(-200, 200)]),
                mode=rng.choice([0, 1, 2, 3, 4, 5, 0xFF]),
                balance=rng.choice([0.0, -1.0, 2.0, 1e9, rng.uniform(-3, 3)]))


def be_table(vals, codes):
    if codes[1] == 0:
        return bytes(v & 0xFF for v in vals)
    return b''.join(struct.pack('>H', v & 0xFFFF) for v in vals)


def user_table(rng, codes, kind):
    n = MAX[codes[0]] + 1
    top = MAX[codes[1]]
    if kind == 'gamma':
        g = rng.uniform(0.4, 2.5)
        v = [int(round(top * (i / (n - 1)) ** g)) for i in range(n)]
    elif kind == 's':
        import math
        k = rng.uniform(2, 8)
        v = [int(top * (1 / (1 + math.exp(-k * (i / (n - 1) - 0.5))))) for i in range(n)]
    elif kind == 'inv':
        v = [top - int(top * i / (n - 1)) for i in range(n)]
    elif kind == 'steps':
        q = rng.randint(2, 64)
        v = [min(top, (i * q // n) * (top // q)) for i in range(n)]
    elif kind == 'clip':
        a, b = sorted(rng.sample(range(n), 2))
        v = [0 if i < a else top if i > b else int(top * (i - a) / max(1, b - a)) for i in range(n)]
    else:  # random walk
        v, x = [], rng.randint(0, top)
        for i in range(n):
            x = max(0, min(top, x + rng.randint(-top // 50 - 1, top // 50 + 1)))
            v.append(x)
    return be_table(v, codes)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    quick = '--quick' in sys.argv
    seed = 1
    for a in sys.argv[1:]:
        if a.startswith('--seed='):
            seed = int(a.split('=')[1])
    lib = args[0] if args else os.path.join(ROOT, '..', 'DS_Dual4_extracted', 'MFSLib2891')
    t = Tester(lib, build_driver())
    rng = random.Random(seed)
    unit = [1.0, 1.0, 1.0]
    real_gain = [1.5, 3.3, 5.0]
    lv0 = [(0, 0, 0, 0)] * 3
    neg0 = [dict(contrast=0, base=0, ref_level=0, ref_target=0, adj=0, mode=0, balance=0.0)] * 3
    zero_ae = ae_blob(unit, lv0, neg0)
    c44 = [(4, 4)] * 3
    film_luts = {}

    # ---------------- film curves: fixed / simple types
    for ft in (5, 9, 0x100, 0x200, 10):
        for codes in (c44, [(4, 0)] * 3, [(0, 0), (2, 4), (3, 1)]):
            for fixed in (0, 1):
                if ft != 10 and fixed == 0 and codes is not c44:
                    continue
                ok, _, tabs = t.film_curve('type %#x fixed=%d codes=%s' % (ft, fixed, codes[0]),
                                           ft, fixed, real_gain, 0xC, zero_ae, codes)
                if codes is c44:
                    film_luts[(ft, fixed)] = tabs
    for ptype in (3, 5, 8, 0xB, 0xC, 0x22):
        for codes in (c44, [(4, 0), (1, 4), (0, 1)]):
            ok, _, tabs = t.film_curve('colour nega fixed ptype %#x %s' % (ptype, codes[0]), 6, 1,
                                       real_gain, ptype, zero_ae, codes)
            if codes is c44 and ptype == 0xC:
                film_luts[(6, 1)] = tabs
    for ft in (0, 1, 2, 3, 4, 7, 8, 11, 0x300, 0x10006, 0x1000a):
        t.film_curve('invalid type %#x' % ft, ft, 0, unit, 0xC, zero_ae, c44)

    # ---------------- B&W negative with AE levels
    n_bw = 4 if quick else 12
    for i in range(n_bw):
        levels = [(rng.choice([0, 20, 0x380, 0x381, 0xFFFF, rng.randint(0, 0x3FF)]),
                   rng.choice([0, 30, 0x3FF, rng.randint(0, 0x3FF)]), 0, 0) for _ in range(3)]
        if i == 0:
            levels = [(40, 60, 0, 0), (35, 70, 0, 0), (45, 50, 0, 0)]
        elif i < n_bw // 2:  # AE-like: exposure level <= 895, 10-bit black level
            e, d = rng.randint(300, 895), rng.randint(0, 120)
            levels = [(e, d, 0, 0)] * 3
        codes = c44 if i % 3 != 2 else [(2, 4), (4, 0), (1, 1)]
        ok, _, tabs = t.film_curve('bw nega AE %s' % (levels,), 10, 0, unit,
                                   0xC, ae_blob(unit, levels, neg0), codes)
        if i == 0:
            film_luts[(10, 0)] = tabs

    # ---------------- colour negative with AE
    state = (0.0, 0)
    n_cn = 6 if quick else 30
    for i in range(n_cn):
        realistic = i < n_cn * 2 // 3
        nega = [nega_params(rng, realistic) for _ in range(3)]
        if i == 0:
            nega = [dict(contrast=55, base=200, ref_level=40, ref_target=215, adj=0, mode=1,
                         balance=0.5),
                    dict(contrast=60, base=170, ref_level=40, ref_target=215, adj=0, mode=1,
                         balance=0.5),
                    dict(contrast=65, base=140, ref_level=40, ref_target=215, adj=0, mode=1,
                         balance=0.5)]
        if i == 1:
            nega = neg0  # all-zero AE record (preferences never written)
        ptype = rng.choice([3, 4, 8, 9, 0xB, 0xC]) if i > 1 else 0xC
        codes = c44 if i % 4 else rng.choice([[(4, 0)] * 3, [(2, 4), (4, 2), (0, 0)]])
        if i == 0:
            codes = c44
        ok, state, tabs = t.film_curve('colour nega AE #%d ptype %#x' % (i, ptype), 6, 0, real_gain,
                                       ptype, ae_blob(real_gain, lv0, nega), codes, state)
        if i == 0:
            film_luts[(6, 0)] = tabs

    # ---------------- user curve composition
    kinds = ['gamma', 's', 'inv', 'steps', 'clip', 'walk']
    ucodes_all = [(4, 4), (0, 0), (2, 2), (0, 4), (4, 0), (1, 3)]
    keys = sorted(film_luts)
    n_uc = 10 if quick else 40
    stretches = []
    for i in range(n_uc):
        key = keys[i % len(keys)]
        film = film_luts[key][i % 3]
        if i % 7 == 6:
            uc = None
        else:
            ucodes = ucodes_all[i % len(ucodes_all)]
            uc = (ucodes, user_table(rng, ucodes, kinds[i % len(kinds)]))
        dst = [(4, 4), (4, 0), (2, 4), (0, 0)][(i // 2) % 4]
        ok, k = t.user_curve('user curve #%d film %s user %s dst %s' %
                             (i, key, None if uc is None else (uc[0], kinds[i % len(kinds)]), dst),
                             (4, 4), film, uc, dst)
        stretches.append(k)
    # non-4/4 film source
    for fc in ((2, 4), (0, 0), (4, 0)):
        film = user_table(rng, fc, 'gamma')
        t.user_curve('user curve film codes %s' % (fc,), fc, film,
                     ((4, 4), user_table(rng, (4, 4), 's')), (4, 4))

    # ---------------- LUT conversion
    combos = [(a, b) for a in range(5) for b in range(5)]
    n_cv = 1 if quick else 2
    for (si, so) in combos:
        for (di, do) in ([(4, 4), (0, 0), (4, 0), (0, 4)] if not quick else [(4, 4), (0, 0)]):
            for r in range(n_cv):
                kind = kinds[(si * 5 + so + r) % len(kinds)]
                src = user_table(rng, (si, so), kind)
                t.convert('convert %s->%s %s' % ((si, so), (di, do), kind), (si, so), src, (di, do))

    print('user curve stretch factors: %s (%d of %d use the spline re-fit)' %
          (' '.join('%.4g' % k for k in stretches), sum(k > 1.0 for k in stretches), len(stretches)))
    print('%d comparisons, %d failures' % (t.cases, t.fails))
    return 1 if t.fails else 0


if __name__ == '__main__':
    sys.exit(main())
