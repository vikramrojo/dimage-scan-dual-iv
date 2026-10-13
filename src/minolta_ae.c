/*
 * Port of the auto-exposure analysis kernel of MFSLib2891 (FUN_10036090 and
 * its call tree), see minolta_ae.h.
 *
 * Every function carries the Ghidra address of the original.  The floating
 * point arithmetic follows the PowerPC code instruction by instruction:
 * fmadd/fmsub/fnmsub are written as fma(), products are evaluated in the
 * original association order, fctiwz is fctiwz() and FUN_1002a320 is d2u().
 *
 * Two pieces of state live in memory exactly as in the original, big-endian
 * and at the original offsets, because the original reads them outside the
 * bounds of the arrays that wrote them:
 *   - the 0x3cd4 byte work area of FUN_1003d220 / FUN_10041490 / FUN_1003f350
 *     (stack frame of FUN_10036090 at +0x2578), and
 *   - the part of the stack below FUN_10036090's frame that FUN_1003baa0,
 *     FUN_1003c770 and FUN_10041490 use for their local arrays (FUN_10041490
 *     reads past its arrays into data left there by FUN_1003c770/baa0).
 * The original never initialises some work fields before reading them
 * (stale stack from a previous AE run); the port starts from zeroes.
 */
#include "minolta_ae.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* constant table at *(0x10047fc4); index = original double index */
static const double K[72] = {
    1.0, 895.0, 96.0, -999.0, 0.5, 0.0, 0.7, 1.2,                 /*  0 */
    0.2, 0.3, 4503599627370496.0, 10.0, 4503601774854144.0,      /*  8 */
    128.0, 255.0, 7.0,                                           /* 13 */
    4294967295.0, 429467295.0, 1e-07, 3.0, 2.0, 100.0, 1.46, 1.15,/* 16 */
    1.4, 1.1, 0.01, 0.8, 80.0, 0.45, 0.55, 1e+99,                /* 24 */
    20.0, 50.0, 0.05, 0.15, 30.0, 0.37, 0.27, 0.35,              /* 32 */
    0.29, 0.68, 0.64, 0.335, 5.0, 25.0, 15.0, 55.0,              /* 40 */
    0.15000000000000002, 0.1, -20.0, 0.73, 0.6, 0.25, 200.0, 12.0,/* 48 */
    0.012, 4.0, 0.45454545454545453, 0.02, 2.2, 0.69,            /* 56 */
    0.3333333333333333, 0.6666666666666666,                      /* 62 */
    1.4142, 0.56, 65535.0, 0.122, 1.7, -0.55, 1.5, 0.0417,       /* 64 */
};
/* table at *(0x10047f90) */
static const double K_PEAKFRAC[3] = { 0.1919, 0.1919, 0.1919 };

/* ------------------------------------------------------------- helpers */

/* FUN_1002a320: double -> unsigned */
static uint32_t d2u(double d)
{
    if (d < 0.0) return 0;
    if (d >= 4294967296.0) return 0xffffffffu;
    if (d >= 2147483648.0) return (uint32_t)(int32_t)(d - 2147483648.0) + 0x80000000u;
    if (d != d) return 0x80000000u; /* fctiwz(NaN) */
    return (uint32_t)(int32_t)d;
}

/* fctiwz: round toward zero, saturating */
static int32_t fctiwz(double d)
{
    if (d != d) return INT32_MIN;
    if (d >= 2147483647.0) return INT32_MAX;
    if (d <= -2147483648.0) return INT32_MIN;
    return (int32_t)d;
}

static double u2d(uint32_t v) { return (double)v; }
static double i2d(int32_t v) { return (double)v; }

/* divwu: QEMU (and the 7400 in practice) return 0 for x / 0 */
static uint32_t divu(uint32_t a, uint32_t b) { return b ? a / b : 0; }

/* FUN_10042210: film type has 3 independent channels */
static int is_color(uint32_t film) { return film == 5 || film == 6; }

static uint32_t sample(const mfs_ae_plane *p, const uint8_t *a)
{
    if (p->bytes == 1) return *a;
    uint16_t v;
    memcpy(&v, a, 2);
    return v;
}

/* big-endian memory (work area, simulated stack) */
static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void wr32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static void wr16(uint8_t *p, uint32_t v) { p[0] = v >> 8; p[1] = v; }
static double rdd(const uint8_t *p)
{
    uint64_t b = (uint64_t)rd32(p) << 32 | rd32(p + 4);
    double d;
    memcpy(&d, &b, 8);
    return d;
}
static void wrd(uint8_t *p, double d)
{
    uint64_t b;
    memcpy(&b, &d, 8);
    wr32(p, (uint32_t)(b >> 32));
    wr32(p + 4, (uint32_t)b);
}

/* ---- work area layout (FUN_1003d220 et al.) */
#define WK_SIZE   0x3cd4
#define CH(c)     ((c) * 0x1420)
#define HIST      0x000  /* u32[256] histogram of gain corrected 8 bit RGB */
#define HIST2     0x400  /* u32[256] histogram of "neutral" pixels */
#define CUM2      0x800  /* u32[256] cumulative HIST2 */
#define CUM       0xc00  /* u32[256] cumulative HIST */
#define RCUM      0x1000 /* u32[256] cumulative HIST from the top */
#define PEAK      0x1400 /* u16 histogram peak */
#define PEAKSUM   0x1408 /* u32 HIST sum around the peak */
#define EDGE      0x140c /* u16 steepest drop above the peak */
#define LO1       0x140e /* u16 1% point */
#define LO5       0x1410 /* u16 5% point */
#define HI1       0x1412 /* u16 99% point */
#define HI5       0x1414 /* u16 95% point */
#define HI15      0x1416 /* u16 85% point */
#define FLO1      0x1418 /* u16 1% point of HIST2 */
#define FLO5      0x141a /* u16 5% point of HIST2 */
#define CUMAT     0x141c /* u32 CUM[LO5 + 25] */
#define ST(c)     (0x3c60 + (c) * 0x18) /* u32 means: +0 all, +4 low, +8 edge, +c mid, +10 centre, +14 border */
#define SHIFT0    0x3ca8 /* s16[3] R/B shift */
#define NNEUTRAL  0x3cb0
#define NPIX      0x3cb4
#define NVALID    0x3cb8
#define SCORE     0x3cbc /* double: FUN_10036090 local_ac */
#define SCORE_A   0x3cc4 /* double: FUN_1003baa0 result */
#define SCORE_B   0x3ccc /* double: FUN_1003c770 result */

#define WH(w, c, i)   ((w) + CH(c) + HIST + 4 * (i))
#define W16(w, c, f)  rd16((w) + CH(c) + (f))

/* ---- simulated stack below FUN_10036090's frame; STK(s, X) = Ghidra local_X */
#define STK_SIZE 0x2600
#define STK(s, x) ((s) + STK_SIZE - (x))

/* FUN_1003a540 output (0x24 bytes) */
typedef struct {
    uint16_t min[3];  /* smallest 8x8 block average */
    uint16_t max[3];  /* largest block average (film 6/10: mean with peak pixel) */
    int32_t pos[3][2];/* x, y of the largest block */
} ae_blocks;

/* ------------------------------------------------------------ kernel */

/* FUN_1003ad40: per channel average of the black reference line, 10 bit */
static int ae_ref_level(uint16_t out[3], uint32_t film, const mfs_ae_plane *pl)
{
    uint32_t avg[3] = { 0, 0, 0 };
    for (unsigned c = 0; c < 3; c++) {
        if (!pl[c].data) return -1;
        uint32_t s = 0;
        for (uint32_t x = 0; x < pl[c].width; x++)
            s += sample(&pl[c], pl[c].data + x * pl[c].pixel_stride);
        s = divu(s, pl[c].width);
        avg[c] = pl[c].bytes == 1 ? s << 2 : s >> 6;
        if (!is_color(film)) break;
    }
    for (unsigned c = 0; c < 3; c++) out[c] = (uint16_t)(is_color(film) ? avg[c] : avg[0]);
    return 0;
}

/* FUN_1003ab30: light reference: line maximum / 8 sample mean at pos x */
static void ae_ref_ratio(double out[3], uint32_t film, const mfs_ae_plane *pl, const int32_t pos[3][2])
{
    double r[3] = { 0, 0, 0 };
    for (unsigned c = 0; c < 3; c++) {
        const uint8_t *p = pl[c].data;
        int32_t ps = (int32_t)pl[c].pixel_stride;
        uint32_t mx = 0, s = 0;
        for (uint32_t x = 0; x < pl[c].width; x++) {
            uint32_t v = sample(&pl[c], p + (int32_t)(x * ps));
            if (mx < v) mx = v;
        }
        for (uint32_t k = 0; k < 8; k++) s += sample(&pl[c], p + (int32_t)(ps * (int32_t)(k + pos[c][0])));
        s >>= 3;
        if (!s) s = 1;
        r[c] = u2d(mx) / u2d(s);
        if (!is_color(film)) break;
    }
    for (unsigned c = 0; c < 3; c++) out[c] = is_color(film) ? r[c] : r[0];
}

/* FUN_1003a540: 8x8 block statistics over the AE area (film 6: whole image) */
static int ae_block_stats(ae_blocks *o, uint32_t film, const mfs_ae_plane *pl,
                          int32_t ax, int32_t ay, uint32_t aw, uint32_t ah)
{
    uint16_t bw, bh;
    if (film == 6) { bw = (uint16_t)(pl[0].width >> 3); bh = (uint16_t)(pl[0].height >> 3); }
    else { bw = (uint16_t)(aw >> 3); bh = (uint16_t)(ah >> 3); }
    int32_t offx = (int32_t)(pl[0].width & 7) >> 1, offy = (int32_t)(pl[0].height & 7) >> 1;
    uint32_t border = 0;
    if ((pl[0].width == aw && pl[0].height == ah) || film == 6) {
        border = 4;
        if (bw < 9 || bh < 9) return -1;
    }
    uint16_t pmax[3] = { 0 }, amin[3] = { 0 }, amax[3] = { 0 };
    int32_t pos[3][2] = { { 0 } };
    for (unsigned c = 0; c < 3; c++) {
        int32_t ps = (int32_t)pl[c].pixel_stride, rs = (int32_t)pl[c].row_stride;
        int32_t off = film == 6 ? offx * ps + offy * rs : ps * (ax + offx) + rs * (ay + offy);
        const uint8_t *base = pl[c].data + off;
        pmax[c] = 0;
        amax[c] = 0;
        amin[c] = pl[c].bytes == 1 ? 255 : 65535;
        for (uint32_t by = border; (int32_t)(by & 0xffff) < (int32_t)(bh - border); by++)
            for (uint32_t bx = border; (int32_t)(bx & 0xffff) < (int32_t)(bw - border); bx++) {
                uint32_t s = 0;
                for (uint32_t j = 0; j < 8; j++)
                    for (uint32_t i = 0; i < 8; i++) {
                        int32_t a = ps * (int32_t)(i + (bx & 0xffff) * 8) + rs * (int32_t)(j + (by & 0xffff) * 8);
                        uint32_t v = sample(&pl[c], base + a);
                        if (pmax[c] < v) pmax[c] = (uint16_t)v;
                        s += v;
                    }
                s >>= 6;
                if (amax[c] < s) {
                    amax[c] = (uint16_t)s;
                    pos[c][0] = (int32_t)(bx & 0xffff) * 8 + ax + offx;
                    pos[c][1] = (int32_t)(by & 0xffff) * 8 + ay + offy;
                }
                if (s < amin[c]) amin[c] = (uint16_t)s;
            }
        if (!is_color(film)) break;
    }
    for (unsigned c = 0; c < 3; c++) {
        unsigned k = is_color(film) ? c : 0;
        o->min[c] = amin[k];
        if (film == (is_color(film) ? 6u : 10u)) o->max[c] = (uint16_t)(((uint32_t)amax[k] + pmax[k]) >> 1);
        else o->max[c] = amax[k];
        o->pos[c][0] = pos[k][0];
        o->pos[c][1] = pos[k][1];
    }
    return 0;
}

/* FUN_1003af80: mean of the block averages between 1% and 50% of min..max */
static int ae_mid_level(uint32_t out[3], uint32_t film, const mfs_ae_plane *pl,
                        int32_t ax, int32_t ay, uint32_t aw, uint32_t ah, const ae_blocks *bs)
{
    uint32_t bh = (ah >> 3) & 0xffff, bw = (aw >> 3) & 0xffff;
    uint32_t border = 0;
    if (pl[0].width == aw && pl[0].height == ah) {
        border = 4;
        if ((uint16_t)(aw >> 3) < 9 || bh < 9) return -1;
    }
    double lo[3] = { 0 }, hi[3] = { 0 };
    uint16_t cnt[3] = { 0 };
    uint32_t sum[3] = { 0 };
    for (unsigned c = 0; c < 3; c++) {
        double d = i2d((int32_t)((uint32_t)bs->max[c] - bs->min[c]));
        lo[c] = fma(K[26], d, u2d(bs->min[c]));
        hi[c] = fma(K[4], d, u2d(bs->min[c]));
        int32_t ps = (int32_t)pl[c].pixel_stride, rs = (int32_t)pl[c].row_stride;
        const uint8_t *base = pl[c].data + ps * (ax + ((int32_t)(pl[0].width & 7) >> 1))
                                         + rs * (ay + ((int32_t)(pl[0].height & 7) >> 1));
        for (uint32_t by = border; (int32_t)(by & 0xffff) < (int32_t)(bh - border); by++)
            for (uint32_t bx = border; (int32_t)(bx & 0xffff) < (int32_t)(bw - border); bx++) {
                uint32_t s = 0;
                for (uint32_t j = 0; j < 8; j++)
                    for (uint32_t i = 0; i < 8; i++)
                        s += (uint16_t)sample(&pl[c], base + ps * (int32_t)(i + (bx & 0xffff) * 8)
                                                           + rs * (int32_t)(j + (by & 0xffff) * 8));
                s >>= 6;
                if (lo[c] < u2d(s) && u2d(s) < hi[c]) { cnt[c]++; sum[c] += s; }
            }
        if (!is_color(film)) break;
    }
    for (unsigned c = 0; c < 3; c++) {
        if (!is_color(film) && c) { out[c] = divu(sum[0], cnt[0]); continue; }
        if (cnt[c] == 0) { sum[c] = d2u((lo[c] + hi[c]) * K[4]); cnt[c] = 1; }
        out[c] = is_color(film) ? divu(sum[c], cnt[c]) : divu(sum[0], cnt[0]);
    }
    return 0;
}

/* FUN_1003b4e0: 8 bit histograms (excluding dark orange-mask-free pixels)
 * and a log-mean density per channel.  h: per channel 0x300 words:
 * [0..255] histogram, [256..511] cumulative, [512..767] reverse cumulative */
static void ae_neg_hist(const mfs_ae_plane *pl, double dens[3], uint32_t *h)
{
    double w = u2d(pl[0].width), ht = u2d(pl[0].height);
    int32_t mx = fctiwz(-fma(K[27], w, -w) * K[4]);
    int32_t my = fctiwz(-fma(K[27], ht, -ht) * K[4]);
    for (unsigned c = 0; c < 3; c++)
        for (unsigned i = 0; i < 256; i++) h[c * 0x300 + i] = 0;
    for (uint32_t y = (uint32_t)my; (y & 0xffff) < pl[1].height - ((uint32_t)my & 0xffff); y++)
        for (uint32_t x = (uint32_t)mx; (x & 0xffff) < pl[1].width - ((uint32_t)mx & 0xffff); x++) {
            uint32_t xx = x & 0xffff, yy = y & 0xffff, v[3];
            for (unsigned c = 0; c < 3; c++) {
                const uint8_t *a = pl[c].data + xx * pl[c].pixel_stride + yy * pl[c].row_stride;
                if (pl[0].bytes == 1) v[c] = *a;
                else { uint16_t s; memcpy(&s, a, 2); v[c] = s >> 8; }
            }
            if (v[0] > 30 || v[1] > 30 || v[0] - v[2] < 15 || v[1] - v[2] < 15) {
                h[v[0]]++;
                h[v[1] + 0x300]++;
                h[v[2] + 0x600]++;
            }
        }
    for (unsigned c = 0; c < 3; c++) {
        uint32_t *g = h + c * 0x300;
        g[0x100] = g[0];
        g[0x200] = g[0xff];
    }
    for (unsigned i = 1; i < 256; i++)
        for (unsigned c = 0; c < 3; c++) {
            uint32_t *g = h + c * 0x300;
            g[0x100 + i] = g[0xff + i] + g[i];
            g[0x200 + i] = g[0x1ff + i] + g[0xff - i];
        }
    for (unsigned c = 0; c < 3; c++) {
        uint32_t *g = h + c * 0x300;
        double acc = K[5];
        for (unsigned i = 1; i <= 254; i++) acc = fma(u2d(g[i]), log10(K[14] - u2d(i)), acc);
        uint32_t n = g[0x1fe] - g[0x100];
        if (n == 0) { dens[0] = dens[1] = dens[2] = K[28]; return; }
        /* MathLib pow(10, x).  The base goes through a volatile so that the
         * compiler cannot turn pow(10, x) into exp10(x), which differs from
         * pow in the last bit (clang does this on Darwin). */
        volatile double ten = K[11];
        dens[c] = K[14] - pow(ten, acc / u2d(n));
    }
}

/* simulated-stack u32 arrays */
#define SW(base, i) ((base) + 4 * (i))

/* FUN_1003baa0: shadow-side colour correlation score of the cumulative
 * histograms (local arrays on the simulated stack). */
static double ae_corr_cum(const uint32_t *h, uint8_t *stk)
{
    uint8_t *a1 = STK(stk, 0x14c8), *a0 = STK(stk, 0x10c8), *a2 = STK(stk, 0x8c8);
    for (int i = 0; i < 0x200; i++) { wr32(SW(a0, i), 0); wr32(SW(a2, i), 0); }
    for (int i = 0; i < 0x100; i++) wr32(SW(a1, i), 0);
    const uint32_t *c0 = h + 0x100, *c1 = h + 0x400, *c2 = h + 0x700;

    /* percentile indices: p1 = #entries below 1%, mean of 45/50/55 % points */
    uint16_t p1_0 = 0, p1_2 = 0, p1_1 = 0;
    uint32_t n45 = 0, n50 = 0, i55 = 0;
    for (int i = 0; i < 256; i++) {
        if (c0[i] < d2u(K[26] * u2d(c0[255]))) p1_0++;
        if (c0[i] < d2u(K[29] * u2d(c0[255]))) n45++;
        if (c0[i] < d2u(K[4] * u2d(c0[255]))) n50++;
        if (d2u(K[30] * u2d(c0[255])) <= c0[i]) { i55 = (uint32_t)i; break; }
    }
    uint32_t m0 = (i55 + (n45 & 0xffff) + (n50 & 0xffff)) / 3;
    n45 = n50 = i55 = 0;
    for (int i = 0; i < 256; i++) {
        if (c2[i] < d2u(K[26] * u2d(c2[255]))) p1_2++;
        if (c2[i] < d2u(K[29] * u2d(c2[255]))) n45++;
        if (c2[i] < d2u(K[4] * u2d(c2[255]))) n50++;
        if (d2u(K[30] * u2d(c2[255])) <= c2[i]) { i55 = (uint32_t)i; break; }
    }
    uint32_t m2 = (i55 + (n45 & 0xffff) + (n50 & 0xffff)) / 3;
    n45 = n50 = i55 = 0;
    for (int i = 0; i < 256; i++) {
        if (c1[i] < d2u(K[26] * u2d(c1[255]))) p1_1++;
        if (c1[i] < d2u(K[29] * u2d(c1[255]))) n45++;
        if (c1[i] < d2u(K[4] * u2d(c1[255]))) n50++;
        if (d2u(K[30] * u2d(c1[255])) <= c1[i]) { i55 = (uint32_t)i; break; }
    }
    uint32_t m1 = (i55 + (n45 & 0xffff) + (n50 & 0xffff)) / 3;
    uint32_t m1s = m1 & 0xffff;

    double s0 = K[5];
    for (uint16_t s = p1_0; (int)(int16_t)s < (int)(m0 & 0xffff); s++) {
        wr32(SW(a0, (int16_t)s + 0x80), c0[(int16_t)s]);
        s0 += u2d(rd32(SW(a0, (int16_t)s + 0x80)));
    }
    uint32_t t0 = c0[255] >> 1;
    wr32(SW(a0, (m0 & 0xffff) + 0x80), t0);
    double s2 = K[5];
    for (uint16_t s = p1_2; (int)(int16_t)s < (int)(m2 & 0xffff); s++) {
        wr32(SW(a2, (int16_t)s + 0x80), c2[(int16_t)s]);
        s2 += u2d(rd32(SW(a2, (int16_t)s + 0x80)));
    }
    uint32_t t2 = c2[255] >> 1;
    wr32(SW(a2, (m2 & 0xffff) + 0x80), t2);
    double s1 = K[5];
    for (int16_t s = 0; (int)s < (int)m1s; s++) {
        wr32(SW(a1, s), c1[s]);
        s1 += u2d(rd32(SW(a1, s)));
    }
    wr32(SW(a1, m1s), c1[255] >> 1);
    s1 += u2d(rd32(SW(a1, m1s)));

    int16_t mid1 = (int16_t)(p1_1 + (int16_t)((int)(m1s - p1_1) / 2));
    int16_t mid0 = (int16_t)(p1_0 + (int16_t)((int)((m0 & 0xffff) - p1_0) / 2));
    int16_t mid2 = (int16_t)(p1_2 + (int16_t)((int)((m2 & 0xffff) - p1_2) / 2));

    double max0 = K[5], min0 = K[31], max2 = K[5], min2 = K[31];
    for (int16_t d = -0x40; d < 0x41; d++) {
        double acc = K[5];
        for (uint16_t u = p1_1; (int)(int16_t)u < (int)m1s; u++) {
            int idx = (int16_t)u + d + (mid0 - mid1) + 0x80;
            if (idx < 0 || idx >= 0x200) break;
            acc = fma(u2d(rd32(SW(a1, (int16_t)u))), u2d(rd32(SW(a0, idx))), acc);
        }
        if (max0 < acc) max0 = acc;
        if (acc < min0) min0 = acc;
    }
    for (int16_t d = -0x40; d < 0x41; d++) {
        double acc = K[5];
        for (uint16_t u = p1_1; (int)(int16_t)u < (int)m1s; u++) {
            int idx = (int16_t)u + d + (mid2 - mid1) + 0x80;
            if (idx < 0 || idx >= 0x200) break;
            acc = fma(u2d(rd32(SW(a1, (int16_t)u))), u2d(rd32(SW(a2, idx))), acc);
        }
        if (max2 < acc) max2 = acc;
        if (acc < min2) min2 = acc;
    }
    int da = abs(mid2 - mid1), db = abs(mid0 - mid1);
    int16_t dist = (int16_t)(da < db ? db : da);
    double fd;
    if (dist < 20) fd = K[0];
    else if (dist < 0x47) fd = K[0] - (i2d(dist) - K[32]) / K[33];
    else fd = K[5];

    uint16_t w[3] = { (uint16_t)((int16_t)m0 - p1_0), (uint16_t)((int16_t)m1 - p1_1), (uint16_t)((int16_t)m2 - p1_2) };
    uint16_t wmax = 0, wmin = 0xff;
    for (int c = 0; c < 3; c++) {
        if (wmax < w[c]) wmax = w[c];
        if (w[c] < wmin) wmin = w[c];
    }
    double fw = wmax == 0 ? u2d(wmin) : u2d(wmin) / u2d(wmax);
    double r0, r2;
    if (K[5] == s1) { r0 = max0 - min0; r2 = max2 - min2; }
    else {
        r0 = ((s0 + u2d(t0)) * (max0 - min0)) / s1;
        r2 = ((s2 + u2d(t2)) * (max2 - min2)) / s1;
    }
    if (K[5] == r0) return K[5];
    if (r0 <= r2) return fw * (fd * (r0 / r2));
    return fw * (fd * (r2 / r0));
}

/* FUN_1003c770: highlight-side colour correlation score of the reverse
 * cumulative histograms; -999 when more than 1/30 of the pixels clip. */
static double ae_corr_rcum(const uint32_t *h, uint8_t *stk)
{
    const uint32_t *r0 = h + 0x200, *r1 = h + 0x500, *r2 = h + 0x800;
    if (r0[255] / 30 < r0[0] || r1[255] / 30 < r1[0] || r2[255] / 30 < r2[0]) return K[3];
    uint8_t *a1 = STK(stk, 0x14d8), *a0 = STK(stk, 0x10d8), *a2 = STK(stk, 0x8d8);
    for (int i = 0; i < 0x200; i++) { wr32(SW(a0, i), 0); wr32(SW(a2, i), 0); }
    for (int i = 0; i < 0x100; i++) wr32(SW(a1, i), 0);

    int16_t b0 = 0, b2 = 0, b1 = 0;
    for (int16_t i = 0; !(d2u(K[26] * u2d(r0[255])) <= r0[i]); i++) b0++;
    for (int16_t i = 0; !(d2u(K[26] * u2d(r2[255])) <= r2[i]); i++) b2++;
    for (int16_t i = 0; !(d2u(K[26] * u2d(r1[255])) <= r1[i]); i++) b1++;
    int16_t e0 = b0 + 20 > 0xff ? 0xff : b0 + 20;
    int16_t e2 = b2 + 20 > 0xff ? 0xff : b2 + 20;
    int16_t e1 = b1 + 20 > 0xff ? 0xff : b1 + 20;
    uint32_t thr = d2u(K[34] * u2d(r1[255]));
    const uint32_t *rr[3] = { r0, r2, r1 };
    int16_t *ee[3] = { &e0, &e2, &e1 };
    for (int k = 0; k < 3; k++) {
        int16_t e = *ee[k];
        const uint32_t *r = rr[k];
        if (e != 0 && e != 0xff &&
            (thr < (uint32_t)abs((int32_t)(r[e] - r[e - 1])) || thr < (uint32_t)abs((int32_t)(r[e] - r[e + 1]))))
            *ee[k] = e + 5 < 0x100 ? e + 5 : 0xff;
    }
    double s0 = K[5], s2 = K[5], s1 = K[5];
    for (int16_t s = b0; s < e0 && s + 0x80 < 0x200; s++) { wr32(SW(a0, s + 0x80), r0[s]); s0 += u2d(rd32(SW(a0, s + 0x80))); }
    for (int16_t s = b2; s < e2 && s + 0x80 < 0x200; s++) { wr32(SW(a2, s + 0x80), r2[s]); s2 += u2d(rd32(SW(a2, s + 0x80))); }
    for (int16_t s = b1; s < e1 && s < 0x100; s++) { wr32(SW(a1, s), r1[s]); s1 += u2d(rd32(SW(a1, s))); }

    double max0 = K[5], min0 = K[31], max2 = K[5], min2 = K[31];
    for (int16_t d = -0x40; d < 0x41; d++) {
        double acc = K[5];
        for (int16_t s = b1; s <= e1; s++) {
            int idx = s + d + (b0 - b1) + 0x80;
            if (idx < 0 || idx >= 0x200) break;
            acc = fma(u2d(rd32(SW(a1, s))), u2d(rd32(SW(a0, idx))), acc);
        }
        if (max0 < acc) max0 = acc;
        if (acc < min0) min0 = acc;
    }
    for (int16_t d = -0x40; d < 0x41; d++) {
        double acc = K[5];
        for (int16_t s = b1; s <= e1; s++) {
            int idx = s + d + (b2 - b1) + 0x80;
            if (idx < 0 || idx >= 0x200) break;
            acc = fma(u2d(rd32(SW(a1, s))), u2d(rd32(SW(a2, idx))), acc);
        }
        if (max2 < acc) max2 = acc;
        if (acc < min2) min2 = acc;
    }
    double c0 = max0 - min0, c2 = max2 - min2;
    double mx = s1, mn = s1;
    if (s0 <= s2) { if (s1 < s2) mx = s2; if (s0 < s1) mn = s0; }
    else { if (s1 < s0) mx = s0; if (s2 < s1) mn = s2; }
    if (K[5] != mx) mn = mn / mx;
    int da = abs(b2 - b1), db = abs(b0 - b1);
    int16_t dist = (int16_t)(da < db ? db : da);
    double fd;
    if (dist < 20) fd = K[0];
    else if (dist < 0x47) fd = K[0] - (i2d(dist) - K[32]) / K[33];
    else fd = K[5];
    if (K[5] == c0) return K[5];
    if (c0 <= c2) return fd * (mn * (c0 / c2));
    return fd * (mn * (c2 / c0));
}

/* FUN_1003eff0: chromaticity window of "neutral" negative pixels per product
 * (two windows: below / above the G 1% point + 20).  t[0..13] = original
 * doubles at +4 .. +0x34 of the table (A..O); the +-0.0 offsets the original
 * applies afterwards do not change any value. */
typedef struct { uint16_t margin; double t[14]; } ae_window;

static int ae_window_for(ae_window *w, uint32_t product)
{
    switch (product) {
    case 3: case 4: case 8: case 9: case 11: case 12: break;
    default: return -1;
    }
    w->margin = 0x14;
    double t[14] = { K[37], K[38], (product == 3 || product == 4) ? K[39] : K[37], K[40], K[43], K[41], K[42],
                     K[39], K[40], K[39], K[40], K[43], K[6], K[42] };
    memcpy(w->t, t, sizeof t);
    return 0;
}

/* FUN_1003d220: gain corrected 8 bit histograms, percentile points, means
 * and the histogram of pixels whose chromaticity lies in the neutral window */
static int ae_neg_stats(uint8_t *wk, const mfs_ae_plane *pl, const double gain[3], uint32_t product)
{
    if (!(pl[0].height == pl[1].height && pl[2].height == pl[1].height &&
          pl[0].width == pl[1].width && pl[2].width == pl[1].width))
        return -1;
    wr32(wk + NPIX, pl[0].width * pl[0].height);
    double w0 = u2d(pl[0].width), h0 = u2d(pl[0].height);
    uint32_t mx = (uint32_t)fctiwz(-fma(K[27], w0, -w0) * K[4]);
    uint32_t my = (uint32_t)fctiwz(-fma(K[27], h0, -h0) * K[4]);
    uint16_t bw = (uint16_t)fctiwz(fma(K[30], u2d(pl[1].width), K[4]));
    uint16_t bh = (uint16_t)fctiwz(fma(K[30], u2d(pl[1].height), K[4]));
    uint32_t sum_in[3] = { 0 }, sum_out[3] = { 0 }, n_in = 0, n_out = 0;
    for (int c = 0; c < 3; c++) {
        wr32(wk + NVALID, 0);
        wr32(wk + NNEUTRAL, 0);
        wr32(wk + CH(c) + PEAKSUM, 0);
        for (int i = 0; i < 256; i++) {
            wr32(wk + CH(c) + HIST + 4 * i, 0);
            wr32(wk + CH(c) + HIST2 + 4 * i, 0);
            wr32(wk + CH(c) + CUM + 4 * i, 0);
            wr32(wk + CH(c) + RCUM + 4 * i, 0);
        }
    }
    int b8 = pl[0].bytes == 1;
    uint32_t W = pl[0].width, H = pl[0].height;
    int32_t hx = (int32_t)bw >> 1, hy = (int32_t)bh >> 1;
    for (uint32_t y = my; (y & 0xffff) < pl[1].height - (my & 0xffff); y++)
        for (uint32_t x = mx; (x & 0xffff) < pl[1].width - (mx & 0xffff); x++) {
            uint32_t xx = x & 0xffff, yy = y & 0xffff, v[3];
            for (int c = 0; c < 3; c++) {
                const uint8_t *a = pl[c].data + xx * pl[c].pixel_stride + yy * pl[c].row_stride;
                uint32_t s;
                if (b8) s = *a; else { uint16_t t; memcpy(&t, a, 2); s = t; }
                uint32_t g = d2u(gain[c] * u2d(s));
                if (b8) v[c] = g > 0xff ? 0xff : g;
                else v[c] = (g > 0xffff ? 0xffff : g) >> 8;
            }
            if (v[0] != 0xff && v[1] != 0xff && v[2] != 0xff && v[0] && v[1] && v[2]) {
                for (int c = 0; c < 3; c++) {
                    uint8_t *p = WH(wk, c, v[c]);
                    wr32(p, rd32(p) + 1);
                }
                wr32(wk + NVALID, rd32(wk + NVALID) + 1);
            }
            if (xx < (W >> 1) - (uint32_t)hx || (W >> 1) + (uint32_t)hx < xx ||
                yy < (H >> 1) - (uint32_t)hy || (H >> 1) + (uint32_t)hy < yy) {
                for (int c = 0; c < 3; c++) sum_out[c] += 0xff - v[c];
                n_out++;
            } else {
                for (int c = 0; c < 3; c++) sum_in[c] += 0xff - v[c];
                n_in++;
            }
        }
    if (u2d(rd32(wk + NVALID)) / u2d(rd32(wk + NPIX)) < K[4])
        for (int i = 0; i < 5; i++)
            for (int c = 0; c < 3; c++) wr32(WH(wk, c, i), 0);

    uint16_t mid[3];
    for (int c = 0; c < 3; c++) {
        uint8_t *b = wk + CH(c);
#define Hc(i) rd32(b + HIST + 4 * (i))
        uint32_t m = 0;
        for (int s = 1; s < 0xff; s++)
            if (m < Hc(s)) { m = Hc(s); wr16(b + PEAK, s); }
        for (int s = -5; s < 6; s++) {
            int i = s + rd16(b + PEAK);
            if (i >= 0 && i < 0x100) wr32(b + PEAKSUM, rd32(b + PEAKSUM) + rd32(b + HIST + 4 * i));
        }
        int32_t drop = 0;
        int16_t s13 = (int16_t)rd16(b + PEAK), s4;
        for (;;) {
            s4 = s13;
            s13 = (int16_t)(s4 + 1);
            if (!((int)s13 < (int)(rd16(b + PEAK) + 20))) break;
            if (s13 > 0xff) { wr16(b + EDGE, 0); break; }
            int32_t d = (int32_t)(rd32(b + HIST + 4 * (s13 - 1)) - rd32(b + HIST + 4 * s13));
            if (drop < d) { wr16(b + EDGE, (uint16_t)(s4 + 3)); drop = d; }
        }
        if ((uint32_t)rd16(b + PEAK) + 20 < rd16(b + EDGE)) wr16(b + EDGE, 0);
        wr32(b + CUM, 0);
        for (int s = 1; s < 0xff; s++) wr32(b + CUM + 4 * s, rd32(b + CUM + 4 * (s - 1)) + Hc(s));
        wr32(b + CUM + 0x3fc, rd32(b + CUM + 0x3f8));
        if (!rd32(b + CUM + 0x3fc)) wr32(b + CUM + 0x3fc, rd32(wk + NVALID));
        wr32(b + RCUM, 0);
        for (int s = 1; s < 0xff; s++) wr32(b + RCUM + 4 * s, rd32(b + RCUM + 4 * (s - 1)) + Hc(0xff - s));
        wr32(b + RCUM + 0x3fc, rd32(b + RCUM + 0x3f8));
        if (!rd32(b + RCUM + 0x3fc)) wr32(b + RCUM + 0x3fc, rd32(wk + NVALID));
        double nc = u2d(rd32(b + CUM + 0x3fc)), nr = u2d(rd32(b + RCUM + 0x3fc));
        wr16(b + LO1, 1);
        for (int16_t s = 1; s < 0xff; s++)
            if (K[26] < u2d(rd32(b + CUM + 4 * s)) / nc) { wr16(b + LO1, s); break; }
        wr16(b + LO5, 1);
        for (int16_t s = (int16_t)rd16(b + LO1); s < 0xff; s++)
            if (K[34] < u2d(rd32(b + CUM + 4 * s)) / nc) { wr16(b + LO5, s); break; }
        wr16(b + HI1, 0xfe);
        for (int16_t s = 1; s < 0xff; s++)
            if (K[26] < u2d(rd32(b + RCUM + 4 * s)) / nr) { wr16(b + HI1, s); break; }
        wr16(b + HI5, 0xfe);
        for (int16_t s = (int16_t)rd16(b + HI1); s < 0xff; s++)
            if (K[34] < u2d(rd32(b + RCUM + 4 * s)) / nr) { wr16(b + HI5, s); break; }
        wr16(b + HI15, 0xfe);
        for (int16_t s = (int16_t)rd16(b + HI5); s < 0xff; s++)
            if (K[35] < u2d(rd32(b + RCUM + 4 * s)) / nr) { wr16(b + HI15, s); break; }
        wr16(b + HI1, 0xff - rd16(b + HI1));
        wr16(b + HI5, 0xff - rd16(b + HI5));
        wr16(b + HI15, 0xff - rd16(b + HI15));
        mid[c] = (uint16_t)(rd16(b + LO1) + (int16_t)((int)(rd16(b + HI1) - rd16(b + LO1)) / 2));
        if (rd16(b + LO5) + 25 < 0xff) wr32(b + CUMAT, rd32(b + CUM + 4 * (rd16(b + LO5) + 25)));
        else wr32(b + CUMAT, 0);
    }
    for (int c = 0; c < 3; c++) {
        uint8_t *b = wk + CH(c), *st = wk + ST(c);
        uint32_t acc = 0;
        uint32_t cum254 = rd32(b + CUM + 0x3f8);
        for (int16_t s = 1; s < 0xff; s++) acc += (uint32_t)s * Hc(s);
        wr32(st + 0, cum254 == rd32(b + CUM) ? 0x80 : divu(acc, cum254 - rd32(b + CUM)));
        uint16_t edge = rd16(b + EDGE);
        acc = 0;
        for (int16_t s = (int16_t)edge; s < 0xff; s++) acc += (uint32_t)s * Hc(s);
        uint32_t ce = rd32(b + CUM + 4 * edge);
        if (cum254 == ce) wr32(st + 8, cum254 == 0 ? 0x80 : divu(acc, cum254));
        else wr32(st + 8, divu(acc, cum254 - ce));
        uint16_t md = mid[c];
        acc = 0;
        for (int16_t s = 2; (int)s < (int)md; s++) acc += (uint32_t)s * Hc(s);
        uint32_t cm = rd32(b + CUM + 4 * (md - 1)), c1 = rd32(b + CUM + 4);
        wr32(st + 4, cm == c1 ? 0 : divu(acc, cm - c1));
        acc = 0;
        for (int16_t s = (int16_t)edge; s < 0x7f; s++) acc += (uint32_t)s * Hc(s);
        uint32_t c126 = rd32(b + CUM + 0x1f8), cl = rd32(b + CUM + 4 * (edge - 1));
        wr32(st + 0xc, c126 == cl ? 0 : divu(acc, c126 - cl));
        wr32(st + 0x10, 0xff - divu(sum_in[c], n_in));
        wr32(st + 0x14, 0xff - divu(sum_out[c], n_out));
#undef Hc
    }
    ae_window win;
    if (ae_window_for(&win, product)) return -1;
    const double *t = win.t;
    for (uint32_t y = my; (y & 0xffff) < pl[1].height - (my & 0xffff); y++)
        for (uint32_t x = mx; (x & 0xffff) < pl[1].width - (mx & 0xffff); x++) {
            uint32_t xx = x & 0xffff, yy = y & 0xffff, v[3];
            double jit = (i2d((int32_t)(xx % 3)) - K[0]) / K[36];
            for (int c = 0; c < 3; c++) {
                const uint8_t *a = pl[c].data + xx * pl[c].pixel_stride + yy * pl[c].row_stride;
                uint32_t s;
                if (b8) s = *a; else { uint16_t q; memcpy(&q, a, 2); s = q; }
                uint32_t g = d2u((gain[c] + jit) * u2d(s));
                if (b8) v[c] = g > 0xff ? 0xff : g;
                else v[c] = (g > 0xffff ? 0xffff : g) >> 8;
            }
            if (v[0] == 0xff || v[1] == 0xff || v[2] == 0xff || !v[0] || !v[1] || !v[2]) continue;
            uint32_t den = 0x2fd - v[1] - v[0] - v[2];
            double r = u2d(0xff - v[0]) / u2d(den), g = u2d(0xff - v[1]) / u2d(den);
            int in;
            if (v[1] < (uint32_t)rd16(wk + CH(1) + LO1) + win.margin)
                in = t[1] < r && r < t[0] && t[3] < g && g < t[2] && (t[4] < g || t[6] < r + g) && r + g < t[5];
            else
                in = t[8] < r && r < t[7] && t[10] < g && g < t[9] && (t[11] < g || t[13] < r + g) && r + g < t[12];
            if (in) {
                for (int c = 0; c < 3; c++) {
                    uint8_t *p = wk + CH(c) + HIST2 + 4 * v[c];
                    wr32(p, rd32(p) + 1);
                }
                wr32(wk + NNEUTRAL, rd32(wk + NNEUTRAL) + 1);
            }
        }
    double nv = u2d(rd32(wk + NVALID));
    for (int c = 0; c < 3; c++) {
        uint8_t *b = wk + CH(c);
        for (int s = 0; s < 0x100; s++) wr32(b + CUM2 + 4 * s, 0);
        for (int s = 1; s < 0xff; s++) wr32(b + CUM2 + 4 * s, rd32(b + CUM2 + 4 * (s - 1)) + rd32(b + HIST2 + 4 * s));
        wr32(b + CUM2 + 0x3fc, rd32(b + CUM2 + 0x3f8));
        wr16(b + FLO1, 0xff);
        wr16(b + FLO5, 0xff);
        for (int16_t s = 1; s < 0xff; s++)
            if (K[26] * nv < u2d(rd32(b + CUM2 + 4 * s))) { wr16(b + FLO1, s); break; }
        for (int16_t s = 1; s < 0xff; s++)
            if (K[34] * nv < u2d(rd32(b + CUM2 + 4 * s))) { wr16(b + FLO5, s); break; }
    }
    return 0;
}

/* FUN_10040760: weight of highlight flare (excess pixels at the peak) */
static double ae_neg_flare(const uint8_t *wk, const uint32_t thr[3])
{
    for (int c = 0; c < 3; c++) {
        if (W16(wk, c, EDGE) == 0) return K[5];
        if ((int32_t)(int16_t)rd32(wk + ST(c) + 4) - (int32_t)W16(wk, c, PEAK) < 1) return K[5];
    }
    int32_t hi = ((int32_t)W16(wk, 2, HI5) + W16(wk, 0, HI5) + W16(wk, 1, HI5)) / 3;
    int32_t lo = ((int32_t)W16(wk, 2, LO5) + W16(wk, 0, LO5) + W16(wk, 1, LO5)) / 3;
    int32_t d = hi - lo;
    double f;
    if (d < 200) f = K[0];
    else if (d <= 220) f = K[0] - (i2d(d) - K[54]) / K[32];
    else f = K[5];
    if (!(rd32(wk + CH(0) + PEAKSUM) > thr[0] || rd32(wk + CH(1) + PEAKSUM) > thr[1] ||
          rd32(wk + CH(2) + PEAKSUM) > thr[2]))
        return K[5];
    double acc = K[5];
    double n = u2d(rd32(wk + NVALID));
    for (int c = 0; c < 3; c++) {
        double t = u2d(thr[c]);
        acc = acc + (u2d(rd32(wk + CH(c) + PEAKSUM)) - t) / fma(K[4], n, -t);
    }
    return (acc / K[19]) * f;
}

/* FUN_10041490: R and B shift against G from the cross correlation of the
 * neutral pixel histograms (arrays on the simulated stack) */
static void ae_neg_shift(uint8_t *wk, uint8_t *stk)
{
    uint8_t *A = STK(stk, 0x24e8), *R = STK(stk, 0x20e8), *B = STK(stk, 0x18e8);
    uint8_t *C0 = STK(stk, 0x10e8), *C2 = STK(stk, 0x8e8);
    int16_t lo0 = 0, lo2 = 0, lo1 = 0, hi0 = 0xff, hi2 = 0xff, hi1 = 0xff;
    for (int i = 0; i < 0x100; i++) wr32(SW(A, i), 0);
    for (int i = 0; i < 0x200; i++) { wr32(SW(R, i), 0); wr32(SW(B, i), 0); }
    for (int i = 0; i < 0x100; i++) {
        const uint8_t *b1 = wk + CH(1), *b0 = wk + CH(0), *b2 = wk + CH(2);
        if (rd32(b1 + CUM2 + 4 * i) < d2u(K[26] * u2d(rd32(b1 + CUM2 + 0x3fc)))) lo1 = (int16_t)i;
        if (rd32(b1 + CUM2 + 4 * i) < rd32(b1 + CUM2 + 0x3fc)) { wr32(SW(A, i), rd32(b1 + HIST2 + 4 * i)); hi1 = (int16_t)i; }
        if (rd32(b0 + CUM2 + 4 * i) < d2u(K[26] * u2d(rd32(b0 + CUM2 + 0x3fc)))) lo0 = (int16_t)i;
        if (rd32(b0 + CUM2 + 4 * i) < rd32(b0 + CUM2 + 0x3fc)) { wr32(SW(R, i + 0x80), rd32(b0 + HIST2 + 4 * i)); hi0 = (int16_t)i; }
        if (rd32(b2 + CUM2 + 4 * i) < d2u(K[26] * u2d(rd32(b2 + CUM2 + 0x3fc)))) lo2 = (int16_t)i;
        if (rd32(b2 + CUM2 + 4 * i) < rd32(b2 + CUM2 + 0x3fc)) { wr32(SW(B, i + 0x80), rd32(b2 + HIST2 + 4 * i)); hi2 = (int16_t)i; }
    }
    double best0 = K[5], best2 = K[5];
    int16_t m1 = (int16_t)(lo1 + (int16_t)((hi1 - lo1) / 2));
    int16_t m0 = (int16_t)(lo0 + (int16_t)((hi0 - lo0) / 2));
    int16_t m2 = (int16_t)(lo2 + (int16_t)((hi2 - lo2) / 2));
    int32_t sh0 = 0, sh2 = 0;
    for (int k = 0, j = -0x20; j < 0x21; j++, k++) {
        wrd(C0 + 8 * k, K[5]);
        wrd(C2 + 8 * k, K[5]);
        for (int i = lo1; i <= hi1; i++) {
            /* indices are not range checked in the original */
            wrd(C0 + 8 * k, fma(i2d((int32_t)rd32(SW(A, i))), i2d((int32_t)rd32(SW(R, i + j + (m0 - m1) + 0x80))), rdd(C0 + 8 * k)));
            wrd(C2 + 8 * k, fma(i2d((int32_t)rd32(SW(A, i))), i2d((int32_t)rd32(SW(B, i + j + (m2 - m1) + 0x80))), rdd(C2 + 8 * k)));
        }
        if (best0 < rdd(C0 + 8 * k)) { best0 = rdd(C0 + 8 * k); sh0 = j + (m0 - m1); }
        if (best2 < rdd(C2 + 8 * k)) { best2 = rdd(C2 + 8 * k); sh2 = j + (m2 - m1); }
    }
    double f29 = 0.0, f22 = 0.0; /* callee-saved FPRs in the original */
    for (int pass = 0; pass < 2; pass++) {
        const uint8_t *Cv = pass ? C2 : C0;
        int32_t base = pass ? (m2 - m1) : (m0 - m1);
        double best = pass ? best2 : best0;
        double P[20], Q[20];
        int n = 0;
#define CV(i) rdd(Cv + 8 * (i))
        if (CV(1) - CV(0) < K[5]) { P[0] = CV(0); Q[0] = i2d(base - 0x20); n = 1; }
        for (int i = 1; i < 0x41; i++) {
            if (K[5] < CV(i) - CV(i - 1) && CV(i + 1) - CV(i) < K[5]) { P[n] = CV(i); Q[n] = i2d(i + base - 0x20); n++; }
            if (n >= 20) break;
        }
        if (n < 20 && K[5] < CV(0x41) - CV(0x40)) { P[n] = CV(0x40); Q[n] = i2d(base + 0x20); n++; }
#undef CV
        double p1 = K[5], p2 = K[5];
        for (int i = 0; i < n; i++)
            if (p1 < P[i]) { p1 = P[i]; f29 = Q[i]; }
        for (int i = 0; i < n; i++)
            if (f29 != Q[i] && p2 < P[i]) { p2 = P[i]; f22 = Q[i]; }
        if (best == p1 && K[27] * p1 < p2) {
            int32_t v = fctiwz(fma((p2 / p1) * (f22 - f29), K[4], f29));
            if (pass) sh2 = v; else sh0 = v;
        }
    }
    wr16(wk + SHIFT0, (uint16_t)sh0);
    wr16(wk + SHIFT0 + 2, 0);
    wr16(wk + SHIFT0 + 4, (uint16_t)sh2);
    double ratio = rd32(wk + NVALID) == 0 ? K[5] : u2d(rd32(wk + NNEUTRAL)) / u2d(rd32(wk + NVALID));
    if (ratio < K[34]) {
        uint32_t u0 = (0xffu - rd32(wk + ST(0))) & 0xff, u1 = (0xffu - rd32(wk + ST(1))) & 0xff;
        uint32_t u2 = (0xffu - rd32(wk + ST(2))) & 0xff;
        double den = i2d((int32_t)(u2 + u0 + u1));
        double r = u2d(u0) / den, g = u2d(u1) / den;
        double e0 = fabs(r - K[62]), e1 = fabs(g - K[62]), e = K[64] * fabs((r + g) - K[63]);
        if (e0 <= e1) { if (e < e1) e = e1; }
        else if (e < e0) e = e0;
        double f = e <= K[49] ? (K[49] - e) / K[49] : K[5];
        double s = K[0] - (K[34] - ratio) / K[34];
        int16_t d0 = (int16_t)(rd32(wk + ST(0) + 4) - rd32(wk + ST(1) + 4));
        int16_t d2 = (int16_t)(rd32(wk + ST(2) + 4) - rd32(wk + ST(1) + 4));
        wr16(wk + SHIFT0, (uint16_t)fctiwz(s * fma(f, i2d(d0), (K[0] - f) * i2d((int16_t)rd16(wk + SHIFT0)))));
        wr16(wk + SHIFT0 + 2, 0);
        wr16(wk + SHIFT0 + 4, (uint16_t)fctiwz(s * fma(f, i2d(d2), (K[0] - f) * i2d((int16_t)rd16(wk + SHIFT0 + 4)))));
    }
    for (int k = 0; k < 6; k += 4) {
        int16_t v = (int16_t)rd16(wk + SHIFT0 + k);
        if (v > 0x23) wr16(wk + SHIFT0 + k, 0x23);
        else if (v < -0x23) wr16(wk + SHIFT0 + k, (uint16_t)-0x23);
    }
}

static double clamp01(double d) { if (K[5] <= d) { if (K[0] < d) d = K[0]; } else d = K[5]; return d; }

/* FUN_1003f350: tone curve parameters for colour negatives */
static void ae_neg_tone(const uint8_t *wk, mfs_ae_tone *T, const mfs_ae_plane *pl,
                        int32_t ax, int32_t ay, uint32_t aw, uint32_t ah, const double gain[3])
{
    int16_t d0 = (int16_t)(W16(wk, 0, FLO5) - W16(wk, 1, FLO5));
    int16_t d2 = (int16_t)(W16(wk, 2, FLO5) - W16(wk, 1, FLO5));
    double sl = i2d((int32_t)((uint32_t)W16(wk, 1, LO5) - W16(wk, 1, LO1))) / K[44];
    int32_t base = fctiwz(fma(sl, u2d(W16(wk, 1, LO1)), u2d(W16(wk, 1, LO5))) / (K[0] + sl));
    int16_t lt[3];
    for (int c = 0; c < 3; c++) lt[c] = (int16_t)(W16(wk, c, FLO5) - W16(wk, c, FLO1));
    double f25;
    if (lt[1] < 10) f25 = K[0];
    else if (lt[1] <= 0x19) f25 = (K[45] - i2d(lt[1])) / K[46];
    else f25 = K[5];
    uint32_t l0 = W16(wk, 0, LO1), l1 = W16(wk, 1, LO1), l2 = W16(wk, 2, LO1), lm;
    if (l1 < l0) lm = l2 < l0 ? l0 : l2;
    else lm = l1 < l2 ? l2 : l1;
    double f26;
    if (lm < 0x37) f26 = K[0];
    else if (lm <= 0x46) f26 = K[0] - (u2d(lm) - K[47]) / K[46];
    else f26 = K[5];
    double nr = rd32(wk + NVALID) == 0 ? K[5] : u2d(rd32(wk + NNEUTRAL)) / u2d(rd32(wk + NVALID));
    double f27;
    if (nr > K[8]) f27 = K[0];
    else if (K[34] <= nr) f27 = K[0] - (K[8] - nr) / K[48];
    else f27 = K[5];
    double sc = rdd(wk + SCORE), f23;
    if (sc > K[4]) f23 = K[0];
    else f23 = K[0] - (K[4] - sc) / K[4];
    int16_t a = (int16_t)abs(lt[0] - lt[1]), b = (int16_t)abs(lt[2] - lt[1]), cc = (int16_t)abs(lt[0] - lt[2]), m;
    if (a > b) m = a > cc ? a : cc;
    else m = b > cc ? b : cc;
    double f24 = m > 10 ? K[0] : K[0] - (K[11] - i2d(m)) / K[11];
    int32_t o0 = fctiwz(fma(K[0] - f24, i2d(d0), K[27] * (f24 * i2d((int16_t)rd16(wk + SHIFT0)))));
    int32_t o2 = fctiwz(fma(K[0] - f24, i2d(d2), K[27] * (f24 * i2d((int16_t)rd16(wk + SHIFT0 + 4)))));
    T[1].lo = (uint16_t)base;
    T[0].lo = (int)(int16_t)base + (int16_t)o0 < 0 ? 0 : (uint16_t)(base + o0);
    T[2].lo = (int)(int16_t)base + (int16_t)o2 < 0 ? 0 : (uint16_t)(base + o2);
    for (int c = 0; c < 3; c++) T[c].weight = f23 * (f27 * (f25 * f26));
    T[0].shift = (int16_t)rd16(wk + SHIFT0);
    T[1].shift = 0;
    T[2].shift = (int16_t)rd16(wk + SHIFT0 + 4);

    if (!(pl[0].width == aw && pl[0].height == ah)) {
        uint32_t avg[3];
        for (int c = 0; c < 3; c++) {
            const uint8_t *p = pl[c].data + ax * pl[c].pixel_stride + ay * pl[c].row_stride;
            uint32_t s = 0, n = 0;
            for (uint32_t y = 0; (y & 0xffff) < ah; y++)
                for (uint32_t x = 0; (x & 0xffff) < aw; x++) {
                    const uint8_t *q = p + (x & 0xffff) * pl[c].pixel_stride + (y & 0xffff) * pl[c].row_stride;
                    uint32_t v;
                    if (pl[c].bytes == 1) { v = d2u(gain[c] * u2d(*q)); if (v > 0xff) v = 0xff; }
                    else { uint16_t t; memcpy(&t, q, 2); v = d2u(gain[c] * u2d(t)); if (v > 0xffff) v = 0xffff; v >>= 8; }
                    s += v;
                    n++;
                }
            avg[c] = divu(s, n);
        }
        for (int c = 0; c < 3; c++) {
            T[c].level = (uint16_t)((avg[2] + avg[0] + avg[1]) / 3);
            T[c].k46 = 0x46;
            T[c].hi = 0;
            T[c].mode = 4;
        }
        return;
    }
    uint32_t hiP = ((uint32_t)W16(wk, 2, HI15) + W16(wk, 0, HI15) + W16(wk, 1, HI15)) / 3 & 0xffff;
    uint32_t loP = ((uint32_t)W16(wk, 2, LO5) + W16(wk, 0, LO5) + W16(wk, 1, LO5)) / 3 & 0xffff;
    double rng = i2d((int32_t)(hiP - loP));
#define S(c, o) rd32(wk + ST(c) + (o))
    double dd = (u2d(S(2, 0x14) + S(0, 0x14) + S(1, 0x14))) / K[19] - (u2d(S(2, 0x10) + S(0, 0x10) + S(1, 0x10))) / K[19];
    double mr = u2d(S(1, 8)) / u2d(S(1, 4)); /* 0x3c80 / 0x3c7c */
    double cf = (u2d(rd32(wk + CH(2) + CUMAT) + rd32(wk + CH(0) + CUMAT) + rd32(wk + CH(1) + CUMAT)) / K[19]) / u2d(rd32(wk + NVALID));
    uint32_t thr[3];
    for (int c = 0; c < 3; c++) thr[c] = d2u(fma(u2d(rd32(wk + NVALID)), K_PEAKFRAC[c], K[4]));
    uint32_t dens = 0xff - (S(2, 0) + S(0, 0) + S(1, 0)) / 3;
    double q = u2d(dens) / rng;
    uint32_t lowAvg = (S(2, 4) + S(0, 4) + S(1, 4)) / 3 & 0xffff;
    uint32_t midAvg = (S(2, 0xc) + S(0, 0xc) + S(1, 0xc)) / 3 & 0xffff;
    if (K[30] <= q) {
        if (q <= K[51]) {
            for (int c = 0; c < 3; c++) {
                T[c].level = (uint16_t)((S(2, 4) + S(0, 4) + S(1, 4)) / 3);
                T[c].k46 = 0x46;
                T[c].hi = 0xff;
                T[c].mode = dd <= K[11] ? 1 : 3;
            }
        } else {
            double fl = ae_neg_flare(wk, thr);
            if (K[52] < cf || K[5] != fl) {
                fl = clamp01(fl);
                if (K[46] < dd) fl = K[5];
                for (int c = 0; c < 3; c++) {
                    T[c].level = (uint16_t)fctiwz(fma(fl, u2d(midAvg), (K[0] - fl) * u2d(lowAvg)));
                    T[c].k46 = 0x46;
                    T[c].mode = dd <= K[46] ? 1 : 3;
                    T[c].hi = 0xff;
                }
            } else {
                uint16_t p = (uint16_t)(W16(wk, 1, LO5) + 0x1e);
                double f = clamp01((K[52] - cf) / K[9]);
                for (int c = 0; c < 3; c++) {
                    T[c].level = (uint16_t)fctiwz(fma(f, u2d(p), (K[0] - f) * u2d(lowAvg)));
                    T[c].k46 = 0x46;
                    T[c].hi = 0xf0;
                    T[c].mode = K[5] == f ? 1 : 3;
                }
            }
        }
    } else if (K[20] < mr || cf < K[9]) {
        uint16_t p = (uint16_t)(W16(wk, 1, LO1) + 0x14);
        double f = K[9] <= cf ? (mr - K[20]) / K[27] : (K[9] - cf) / K[49];
        f = clamp01(f);
        if (dd < K[50]) f = K[5];
        for (int c = 0; c < 3; c++) {
            T[c].level = (uint16_t)fctiwz(fma(f, u2d(p), (K[0] - f) * u2d(lowAvg)));
            T[c].k46 = 0x46;
            T[c].mode = K[5] == f ? 1 : 3;
            T[c].hi = 0xf0;
        }
    } else {
        double f = clamp01(ae_neg_flare(wk, thr));
        for (int c = 0; c < 3; c++) {
            T[c].level = (uint16_t)fctiwz(fma(f, u2d(midAvg), (K[0] - f) * u2d(lowAvg)));
            T[c].k46 = 0x46;
            T[c].mode = 1;
            T[c].hi = 0xff;
        }
    }
    if (nr < K[34]) {
        uint16_t hmax = 0, lmin = 0xff;
        int ih = 0, il = 0;
        for (int c = 0; c < 3; c++) {
            if (hmax < W16(wk, c, HI5)) { hmax = W16(wk, c, HI5); ih = c; }
            if (W16(wk, c, LO5) < lmin) { lmin = W16(wk, c, LO5); il = c; }
        }
        double f = (K[34] - nr) / K[34];
        uint16_t u = (uint16_t)fctiwz(K[32] + fma(u2d(S(ih, 0) - S(il, 0)), K[53], u2d(S(il, 0))));
        for (int c = 0; c < 3; c++) {
            T[c].lo = 0;
            T[c].shift = (int16_t)fctiwz(i2d((int16_t)rd16(wk + SHIFT0 + 2 * c)) * (K[0] - f));
            T[c].level = (uint16_t)fctiwz(fma(K[0] - f, u2d(T[c].level), f * u2d(u)));
            T[c].k46 = 0x46;
            T[c].mode = 0;
            T[c].hi = 0xff;
        }
    }
#undef S
    if (T[0].mode == 3 && (T[0].lo == 0 || T[1].lo == 0 || T[2].lo == 0))
        for (int c = 0; c < 3; c++) T[c].mode = 1;
}

/* (full * r + dark) / (max * r + dark): FUN_10036090 gain from the
 * brightest block, light profile ratio r and black level */
static double ae_gain(uint32_t full, double r, uint16_t dark, uint16_t max)
{
    return fma(u2d(full), r, u2d(dark)) / fma(u2d(max), r, u2d(dark));
}

/* FUN_10036090 */
int mfs_ae_analyze(const mfs_ae_input *in, mfs_ae_result *out)
{
    const mfs_ae_plane *img = in->image;
    uint32_t film = in->film_type;
    int32_t ax = (int32_t)in->area_x, ay = (int32_t)in->area_y;
    uint32_t aw = in->area_w, ah = in->area_h;
    if (aw < 0x20 || ah < 0x20 || img[0].width < in->area_x + aw || img[0].height < in->area_y + ah) return -1;
    for (int c = 0; c < 3; c++) {
        out->gain[c] = K[0];
        out->ch[c].exposure = out->ch[c].dark = out->ch[c].r1c = out->ch[c].r1e = 0;
    }
    uint32_t full = img[0].bytes == 1 ? 255 : 65535;
    uint16_t dark[3];
    ae_blocks bs, bs2;
    double r[3];
    if (ae_ref_level(dark, film, in->ref_dark)) return -1;
    if (ae_block_stats(&bs, film, img, ax, ay, aw, ah)) return -1;

    if (film == 6) {
        uint8_t wk[WK_SIZE], stk[STK_SIZE]; /* original: stack frame */
        uint32_t hist[0x900];
        memset(wk, 0, sizeof wk);
        memset(stk, 0, sizeof stk);
        bs2 = bs;
        double q[3], dens[3], g[3], b[3], lim[3];
        ae_ref_ratio(r, film, in->ref_light, bs2.pos);
        for (int c = 0; c < 3; c++) {
            q[c] = ae_gain(full, r[c], dark[c], bs2.max[c]);
            out->ch[c].dark = dark[c];
        }
        ae_neg_hist(img, dens, hist);
        double s1 = ae_corr_cum(hist, stk);
        double s2 = ae_corr_rcum(hist, stk);
        wrd(wk + SCORE_A, s1);
        wrd(wk + SCORE_B, s2);
        for (int c = 0; c < 3; c++) g[c] = K[2] / dens[c];
        if (g[1] < q[1]) {
            g[0] = q[1] * (g[0] / g[1]);
            g[2] = q[1] * (g[2] / g[1]);
            g[1] = q[1];
        }
        if (K[3] == s2) {
            for (int c = 0; c < 3; c++) out->gain[c] = g[c];
            wrd(wk + SCORE, s1);
        } else {
            double sum = s1 + s2;
            if (sum > K[4]) {
                for (int c = 0; c < 3; c++) b[c] = fma(s1, g[c], s2 * q[c]) / (s1 + s2);
            } else if (K[5] == s1 && K[5] == s2) {
                for (int c = 0; c < 3; c++) b[c] = (g[c] + q[c]) * K[4];
            } else {
                double m = s1 <= s2 ? K[4] * (s1 / s2) : K[4] * (s2 / s1);
                double w2 = s2 < m ? m : s2, w1 = s1 < m ? m : s1;
                for (int c = 0; c < 3; c++) b[c] = fma(w1, g[c], w2 * q[c]) / (w1 + w2);
            }
            wrd(wk + SCORE, sum);
            lim[0] = K[6] * q[1];
            lim[1] = q[1];
            lim[2] = K[7] * q[1];
            for (int c = 0; c < 3; c++) {
                double v = (g[c] + q[c]) * K[4];
                if (lim[c] > v) lim[c] = v;
            }
            double t;
            if (sum > K[4]) t = K[0];
            else if (sum < K[8]) t = K[5];
            else t = (sum - K[8]) / K[9];
            for (int c = 0; c < 3; c++) out->gain[c] = fma(t, b[c], (K[0] - t) * lim[c]);
        }
        int rc = ae_neg_stats(wk, img, out->gain, in->product);
        if (rc) return rc;
        ae_neg_shift(wk, stk);
        ae_neg_tone(wk, out->tone, img, ax, ay, aw, ah, out->gain);
        return 0;
    }
    if (film == 5) {
        unsigned k = 0;
        if (in->per_channel_pos != 1) {
            if (bs.max[0] >= bs.max[1] && bs.max[0] >= bs.max[2]) k = 0;
            else if (bs.max[1] >= bs.max[0] && bs.max[1] >= bs.max[2]) k = 1;
            else if (bs.max[2] >= bs.max[0] && bs.max[2] >= bs.max[1]) k = 2;
        }
        ae_ref_ratio(r, film, in->ref_light, bs.pos);
        for (int c = 0; c < 3; c++) {
            unsigned j = in->per_channel_pos == 1 ? (unsigned)c : k;
            out->gain[c] = ae_gain(full, r[j], dark[j], bs.max[j]);
        }
        return 0;
    }
    if (film == 10) {
        uint32_t mid[3], mid_area[3], e[3];
        if (aw == img[0].width && ah == img[0].height) {
            bs2 = bs;
            if (ae_mid_level(mid, film, img, ax, ay, aw, ah, &bs)) return -1;
        } else {
            if (ae_mid_level(mid_area, film, img, ax, ay, aw, ah, &bs)) return -1;
            if (ae_block_stats(&bs2, film, img, 0, 0, img[0].width, img[0].height)) return -1;
            if (ae_mid_level(mid, film, img, 0, 0, img[0].width, img[0].height, &bs2)) return -1;
            double f = u2d(mid_area[0]) / u2d(mid[0]);
            uint32_t m = d2u(u2d(mid[0]) * f);
            mid[0] = mid[1] = mid[2] = m;
        }
        ae_ref_ratio(r, film, in->ref_light, bs2.pos);
        for (int c = 0; c < 3; c++) {
            out->gain[c] = ae_gain(full, r[0], dark[0], bs2.max[0]);
            e[c] = d2u(u2d(mid[0]) * out->gain[0]);
            out->ch[c].dark = dark[0];
            e[c] = img[0].bytes == 1 ? e[c] << 2 : e[c] >> 6;
        }
        double f = K[0];
        for (int c = 0; c < 3; c++)
            if (K[1] < f * u2d(e[c])) f = K[1] / u2d(e[c]);
        for (int c = 0; c < 3; c++) out->ch[c].exposure = (uint16_t)fctiwz(f * u2d(e[c]));
        return 0;
    }
    return -1;
}

/* ------------------------------------------------------- byte layout */

void mfs_ae_result_from_be(const uint8_t raw[0x70], mfs_ae_result *o)
{
    for (int c = 0; c < 3; c++) {
        o->gain[c] = rdd(raw + 8 * c);
        const uint8_t *p = raw + 0x18 + 8 * c;
        o->ch[c].exposure = rd16(p);
        o->ch[c].dark = rd16(p + 2);
        o->ch[c].r1c = rd16(p + 4);
        o->ch[c].r1e = rd16(p + 6);
        const uint8_t *t = raw + 0x30 + 0x14 * c;
        o->tone[c].level = rd16(t);
        o->tone[c].k46 = rd16(t + 2);
        o->tone[c].lo = rd16(t + 4);
        o->tone[c].hi = rd16(t + 6);
        o->tone[c].shift = (int16_t)rd16(t + 8);
        o->tone[c].mode = t[10];
        o->tone[c].pad0b = t[11];
        o->tone[c].weight = rdd(t + 12);
    }
    memcpy(o->pad6c, raw + 0x6c, 4);
}

void mfs_ae_result_to_be(const mfs_ae_result *o, uint8_t raw[0x70])
{
    for (int c = 0; c < 3; c++) {
        wrd(raw + 8 * c, o->gain[c]);
        uint8_t *p = raw + 0x18 + 8 * c;
        wr16(p, o->ch[c].exposure);
        wr16(p + 2, o->ch[c].dark);
        wr16(p + 4, o->ch[c].r1c);
        wr16(p + 6, o->ch[c].r1e);
        uint8_t *t = raw + 0x30 + 0x14 * c;
        wr16(t, o->tone[c].level);
        wr16(t + 2, o->tone[c].k46);
        wr16(t + 4, o->tone[c].lo);
        wr16(t + 6, o->tone[c].hi);
        wr16(t + 8, (uint16_t)o->tone[c].shift);
        t[10] = o->tone[c].mode;
        t[11] = o->tone[c].pad0b;
        wrd(t + 12, o->tone[c].weight);
    }
    memcpy(raw + 0x6c, o->pad6c, 4);
}
