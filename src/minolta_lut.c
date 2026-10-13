/*
 * Port of the tone-curve / LUT kernel of MFSLib2891 (see minolta_lut.h).
 *
 * The arithmetic follows the PowerPC code instruction by instruction:
 * every fmadd / fnmsub of the original is an explicit fma() here, all other
 * operations are separately rounded doubles (FP contraction is disabled for
 * this file), integer <-> double conversions use the original's helpers
 * (d2u = FUN_1002a320, fctiwz = the fctiwz instruction), and 16-bit loop
 * counters are masked like the original's clrlwi.
 *
 * Constants are those of the table at 0x146398 (named K_xxx after their
 * value) and the curve tables at 0x146108..0x146358.
 */
#include "minolta_lut.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#pragma STDC FP_CONTRACT OFF

const uint16_t mfs_lut_max[8] = { 255, 1023, 4095, 16383, 65535, 0, 0, 0 };

/* constants from 0x146398 (index = offset / 8) */
#define K_EPS      1e-7            /* +0x90 */
#define K_MAXU32   4294967295.0    /* +0x80 */
#define K_MAXU32_T 429467295.0     /* +0x88 (sic, a typo in the original) */

/* ---------------------------------------------------------------- helpers */

/* FUN_1002a320: double -> unsigned, truncating; negatives -> 0, values
 * >= 2^32 and NaN -> 0xffffffff (its "bgelr" is taken for unordered) */
static uint32_t d2u(double d)
{
    if (d < 0.0) return 0;
    if (!(d < 4294967296.0)) return 0xffffffffu;
    return (uint32_t)d;
}

/* PowerPC fctiwz: truncate toward zero, saturate, NaN -> INT32_MIN */
static int32_t fctiwz(double d)
{
    if (d != d) return INT32_MIN;
    if (d >= 2147483647.0) return INT32_MAX;
    if (d <= -2147483648.0) return INT32_MIN;
    return (int32_t)d;
}

/* FUN_10038ac0: clamp an unsigned value to [lo, hi] */
static uint32_t clampu(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void lut_put(void *t, uint32_t i, uint32_t is16, uint32_t v)
{
    if (is16) ((uint16_t *)t)[i] = (uint16_t)v;
    else ((uint8_t *)t)[i] = (uint8_t)v;
}

static uint32_t lut_get(const void *t, uint32_t i, uint32_t is16)
{
    return is16 ? ((const uint16_t *)t)[i] : ((const uint8_t *)t)[i];
}

/* ------------------------------------------------------------ cubic spline */

/* Coefficient table of the original (global at 0x2a3968: a, b, c, d arrays
 * of 256 doubles at +0, +0x800, +0x1000, +0x1800).  Segment i evaluates
 * a + b t + c t^2 + d t^3 with t = x - xs[i]. */
typedef struct {
    double a[256], b[256], c[256], d[256];
} spline_t;

/* FUN_10038ca0: fit the spline through (xs[i], ys[i]), i < n.  The original
 * also solves a tridiagonal system whose result it then overwrites with
 * central differences; that dead computation is omitted. */
static void spline_fit(spline_t *s, const double *xs, const double *ys, uint32_t n)
{
    double slope[256], h[256], del[256];
    n &= 0xffff;
    memset(slope, 0, sizeof slope);
    memset(h, 0, sizeof h);
    memset(del, 0, sizeof del);

    double w = xs[1] - xs[0];
    if (0.0 == w) w = K_EPS;
    slope[0] = (ys[1] - ys[0]) / w;
    w = xs[n - 1] - xs[n - 2];
    if (0.0 == w) w = K_EPS;
    slope[n - 1] = (ys[n - 1] - ys[n - 2]) / w;
    h[0] = 0.0;
    for (uint32_t i = 1; i < n; i++) {
        h[i] = xs[i] - xs[i - 1];
        if (0.0 == h[i]) h[i] = K_EPS;
        del[i] = (ys[i] - ys[i - 1]) / h[i];
    }
    for (int i = 1; i < (int)n - 1; i++) {
        w = xs[i + 1] - xs[i - 1];
        if (0.0 == w) w = K_EPS;
        slope[i] = (ys[i + 1] - ys[i - 1]) / w;
    }
    for (int i = 0; i < (int)n - 1; i++) {
        double d = del[i + 1];
        double e = -fma(2.0, d, -(slope[i] + slope[i + 1])); /* fnmsub */
        double hh = h[i + 1];
        s->c[i] = ((d - slope[i]) - e) / hh;
        s->d[i] = e / (hh * hh);
    }
    for (int i = 1; i < (int)n - 1; i++)
        s->b[i] = (ys[i + 1] - ys[i - 1]) / (xs[i + 1] - xs[i - 1]);
    for (uint32_t i = 0; i < n; i++) s->a[i] = ys[i];
    s->b[0] = slope[0];
    s->b[n - 1] = slope[n - 1];
}

/* FUN_10038b10: evaluate the spline at x */
static double spline_eval(const spline_t *s, double x, const double *xs,
                          const double *ys, uint32_t n)
{
    n &= 0xffff;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) {
        double y = ys[n - 1];
        return y > K_MAXU32 ? K_MAXU32_T : y;
    }
    double r = 0.0;
    for (int i = 0; i < (int)n - 1; i++) {
        if (x >= xs[i] && x < xs[i + 1]) {
            double t = x - xs[i];
            double e = fma(t, t * (s->d[i] * t), t * (s->c[i] * t));
            r = s->a[i] + fma(s->b[i], t, e);
            break;
        }
    }
    if (r > K_MAXU32) r = K_MAXU32;
    if (r < 0.0) r = 0.0;
    return r;
}

/* ---------------------------------------------------------- LUT analysis */

/* FUN_10038890: index after the last change of the table value (the first
 * index from which the table is constant up to its end). */
static uint32_t lut_last_change(const void *t, uint32_t in_code, uint32_t out_code)
{
    uint32_t max = mfs_lut_max[in_code];
    uint32_t is16 = out_code != 0;
    uint32_t last = lut_get(t, max, is16);
    for (int32_t i = (int32_t)max - 1; i >= 0; i--)
        if (lut_get(t, (uint32_t)i, is16) != last) return (uint32_t)(i + 1) & 0xffff;
    return max;
}

/* FUN_10038970: 0 if the table uses at least (max_out+1)/35 distinct
 * values, else -1.  The histogram is 16-bit like the original's. */
static int lut_enough_levels(const void *t, uint32_t in_code, uint32_t out_code)
{
    uint32_t nin = mfs_lut_max[in_code] + 1u;
    uint32_t nout = mfs_lut_max[out_code] + 1u;
    uint32_t need = (nout / 35) & 0xffff;
    uint16_t *hist = calloc(nout, sizeof *hist);
    if (!hist) return -1;
    for (uint32_t i = 0; i < nin; i++) hist[lut_get(t, i, out_code != 0)]++;
    uint32_t cnt = 0;
    int r = -1;
    for (uint32_t i = 0; i < nout; i++) {
        if (hist[i] != 0 && (++cnt & 0xffff) >= need) { r = 0; break; }
    }
    free(hist);
    return r;
}

/* ------------------------------------------------- per film type curves */

/* curve tables of the original */
static const double posi_x[20] = { /* 0x146108, 12-bit input */
    0, 1, 2, 3, 4, 5, 6, 7, 10, 20, 30, 70, 140, 300, 500, 820, 1500, 2400, 3300, 4095 };
static const double posi_y[20] = { /* 0x1461a8 */
    0, 10, 20, 30, 40, 50, 60, 70, 100, 200, 300, 500, 750, 1200, 1580, 2000, 2700, 3380,
    3850, 4095 };
static const double bwnega_y[5] = { 0, 135, 512, 950, 1023 }; /* 0x146248 */
static const double nega_x[8] = { 25, 64, 96, 128, 160, 192, 224, 255 };  /* 0x146298, 0x146318 */
static const double nega_y[8] = { 255, 161, 117, 87, 62, 38, 15, 0 };     /* 0x1462d8, 0x146358 */

/* write spline(i) for i in [first, nin] into lut, clamped to nout */
static void put_spline(const mfs_lut_desc *l, const spline_t *s, const double *x,
                       const double *y, uint32_t n, uint32_t first)
{
    uint32_t nin = mfs_lut_max[l->in_code], nout = mfs_lut_max[l->out_code];
    for (uint32_t i = first; i < nin + 1; i++) {
        double v = spline_eval(s, (double)i, x, y, n);
        lut_put(l->table, i, l->out_code != 0, clampu(d2u(v), 0, nout));
    }
}

/* FUN_10039260: positive film S curve (20 knots on a 12-bit scale) */
static void curve_positive(const mfs_lut_desc *l, uint16_t scale)
{
    spline_t s;
    double x[20], y[20];
    double nin = mfs_lut_max[l->in_code], nout = mfs_lut_max[l->out_code];
    double kx = nin / 4095.0, ky = nout / 4095.0, k = (double)scale / 100.0;
    for (int i = 0; i < 20; i++) {
        x[i] = posi_x[i] / k;
        x[i] = x[i] * kx;
        y[i] = posi_y[i] * ky;
    }
    spline_fit(&s, x, y, 20);
    lut_put(l->table, 0, l->out_code != 0, 0);
    put_spline(l, &s, x, y, 20, 1);
}

/* FUN_10039530: B&W negative with AE (5 knots on a 10-bit scale, from the
 * AE shadow / highlight points) */
static void curve_bw_nega_ae(const mfs_lut_desc *l, uint16_t shadow, uint16_t scale,
                             uint16_t highlight)
{
    spline_t s;
    double x[10] = { 0 }, y[10];
    uint32_t nin = mfs_lut_max[l->in_code], nout = mfs_lut_max[l->out_code];
    double kx = (double)nin / 1023.0, ky = (double)nout / 1023.0;
    uint32_t lo = shadow;
    if (lo > 0x380) lo = 0;
    int32_t hi = 0x3ff - (int32_t)highlight;
    double k = (double)scale / 100.0;
    memcpy(y, bwnega_y, sizeof bwnega_y);
    memset(y + 5, 0, 5 * sizeof(double));
    x[0] = 0.0 / k;
    x[1] = (double)(int32_t)(lo / 3) / k;
    x[2] = (double)lo / k;
    x[3] = (double)((int32_t)lo + ((hi - (int32_t)lo) / 3) * 2) / k;
    x[4] = (double)hi / k;
    for (int i = 0; i < 5; i++) {
        x[i] = x[i] * kx;
        y[i] = y[i] * ky;
        y[i] = (double)nout - y[i];
    }
    spline_fit(&s, x, y, 5);
    lut_put(l->table, 0, l->out_code != 0, nout);
    put_spline(l, &s, x, y, 5, 1);
}

/* FUN_10039950: straight line, 0x100 film type */
static void curve_linear(const mfs_lut_desc *l, uint16_t scale)
{
    uint32_t nin = mfs_lut_max[l->in_code], nout = mfs_lut_max[l->out_code];
    double k = (double)scale / 100.0;
    k = k * ((double)(int32_t)(nout + 1) / (double)(int32_t)(nin + 1));
    for (uint32_t i = 0; i <= nin; i++) {
        uint32_t v = (uint32_t)fctiwz(k * (double)i) & 0xffff;
        lut_put(l->table, i, l->out_code != 0, clampu(v, 0, nout));
    }
}

/* FUN_10039b30: inverted straight line, 0x200 film type */
static void curve_linear_inv(const mfs_lut_desc *l, uint16_t scale)
{
    uint32_t nin = mfs_lut_max[l->in_code], nout = mfs_lut_max[l->out_code];
    /* the original also computes k * (nout+1)/(nin+1) (stored to a stack
     * slot) but the loop uses the plain scale: out = nout - min(i, nout) */
    double k = (double)scale / 100.0;
    for (uint32_t i = 0; i <= nin; i++) {
        uint32_t v = clampu((uint32_t)fctiwz(k * (double)i) & 0xffff, 0, nout);
        lut_put(l->table, i, l->out_code != 0, nout - v);
    }
}

/* FUN_10039d20 (colour negative) / FUN_1003a1c0 (B&W negative) without AE:
 * fixed 8-knot inversion curve on an 8-bit scale, scaled per channel. */
static void curve_nega_fixed(const mfs_lut_desc *l, double chgain)
{
    spline_t s;
    double x[8], y[8];
    uint32_t nin = mfs_lut_max[l->in_code], nout = mfs_lut_max[l->out_code];
    double kx = (double)nin / 255.0, ky = (double)nout / 255.0;
    for (int i = 0; i < 8; i++) {
        x[i] = kx * nega_x[i];
        y[i] = chgain * nega_y[i];
        if (y[i] > 255.0) y[i] = 255.0;
        y[i] = ky * y[i];
    }
    spline_fit(&s, x, y, 8);
    put_spline(l, &s, x, y, 8, 0);
}

/* channel gain of FUN_10039d20 by product type */
static double nega_fixed_gain(int32_t ptype, uint32_t ch)
{
    ch &= 0xff;
    switch (ptype) {
    case 3:
        if (ch == 0) return 1.46;
        if (ch == 1) return 1.15;
        return 1.0;
    case 8: case 9: case 0xb: case 0xc:
        if (ch == 0) return 1.4;
        if (ch == 1) return 1.1;
        return 1.0;
    default:
        return 1.0;
    }
}

/* FUN_10041340: transmission of density x for the product's film gamma:
 * 1 / 10^(g * log10(x)).  Returns -1 in *ok for undefined product types. */
static double nega_transmission(double x, uint32_t ch, int32_t ptype, int *ok)
{
    double g;
    switch (ptype) {
    case 3: case 4: g = 0.7; break;
    case 8: case 9: case 0xb: case 0xc: g = 0.69; break;
    default: *ok = 0; g = 0.0; break; /* uninitialised in the original */
    }
    (void)ch; /* the three channel entries are identical */
    if (x <= 0.0) return 1.0;
    double l = log10(x);
    return 1.0 / pow(10.0, g * l);
}

/* FUN_10040990: colour negative with AE.  Builds a characteristic curve of
 * the film from the AE parameters, fits a spline to it and maps it through
 * a 1/2.2 gamma with a linear toe; channel 1 (green) fixes the reference
 * level that R and B are balanced to (statics in st). */
static int curve_nega_ae(const mfs_lut_desc *l, const mfs_lut_ae_nega *p, uint32_t ch,
                         int32_t ptype, mfs_lut_state *st)
{
    double X[256], Y[256];
    spline_t s;
    int ok = 1;
    uint32_t nin_i = mfs_lut_max[l->in_code], nout_i = mfs_lut_max[l->out_code];
    double nin = nin_i, nout = nout_i;
    uint32_t contrast = p->contrast;
    if (contrast > 100) contrast = 100;
    else if (contrast < 10) contrast = 10;

    const double x0 = 12.0, x1 = 20.0 * x0;            /* +0x1b8, +0x100 */
    const double t0 = 0.012;                             /* +0x1c0 */
    double xmid = x0 + ((double)p->base * (x1 - x0)) / 255.0;
    double g0 = nega_transmission(x0, ch, ptype, &ok);
    if (!ok) return -1;
    double f19 = 1.0 / (g0 - t0);
    double f28 = nega_transmission(xmid, ch, ptype, &ok);
    double t50 = 255.0 * (f19 * (f28 - t0));
    double f25 = (f19 * (double)contrast) / t50;
    double f20 = 255.0 * (f25 * (f28 - t0));
    double f18;
    if (f25 > f19) f18 = ((255.0 - f20) / 255.0) / (g0 - f28);
    else f18 = f25;

    uint32_t n = 0;
    double lim = 4.0 * nout;
    for (int32_t i = 0;; ) {
        /* first 50 densities in steps of 1, then steps of 30 up to 4*x1 */
        if (i < 50) { /* loop 1 */ }
        else if (!((double)i < 4.0 * x1)) break;
        double x = (double)i;
        double tr = nega_transmission(x, ch, ptype, &ok);
        double f26;
        if (tr <= f28) f26 = (f25 * (tr - t0)) * nin;
        else f26 = fma(f18, tr - f28, f20 / 255.0) * nin;
        X[n] = nin - f26;
        Y[n] = nout * ((x - x0) / (x1 - x0));
        n++;
        int stop = Y[n - 1] > lim;
        if (i < 50) {
            i = stop ? 50 : i + 1;
        } else {
            if (stop) break;
            i += 30;
        }
    }
    spline_fit(&s, X, Y, n);

    double xq = (double)(int32_t)((int32_t)contrast + p->contrast_adj) / 255.0;
    double s1 = spline_eval(&s, -fma(xq, nin, -nin), X, Y, n);
    double t58 = (((double)p->base / s1) / 255.0) * nout;
    double xr = (double)p->ref_level / 255.0;
    double ref = t58 * spline_eval(&s, -fma(xr, nin, -nin), X, Y, n);
    ref = nout * pow(ref / nout, 0.45454545454545453);
    double target = ((double)p->ref_target / 255.0) * nout;

    switch (p->balance_mode) {
    case 0:
        st->ratio = 1.0;
        break;
    case 4:
        if ((ch & 0xff) == 1) {
            st->ref = d2u(ref);
            st->ratio = 1.0;
        } else {
            st->ratio = (double)st->ref / ref;
        }
        break;
    case 1:
        if ((ch & 0xff) == 1) {
            st->ratio = fma(p->balance, target - ref, ref) / ref;
            if (st->ratio > 1.0) {
                st->ref = d2u(fma(p->balance, target - ref, ref));
            } else {
                st->ref = d2u(ref);
                st->ratio = 1.0;
            }
        } else {
            st->ratio = (double)st->ref / ref;
        }
        break;
    default:
        st->ratio = fma(p->balance, target - ref, ref) / ref;
        break;
    }

    double toe = 0.02 * nout;
    double off = 0.02 - pow(0.1, 2.2);
    for (int32_t i = 0; i <= (int32_t)nin_i; i++) {
        double v = t58 * spline_eval(&s, nin - (double)i, X, Y, n);
        if (v < 0.0) v = 0.0;
        if (v < toe) v = st->ratio * (5.0 * v);
        else v = st->ratio * (nout * pow(v / nout - off, 0.45454545454545453));
        lut_put(l->table, (uint32_t)i, l->out_code != 0, clampu(d2u(v), 0, nout_i));
    }
    return 0;
}

/* ----------------------------------------------------------- dispatcher */

void mfs_lut_ae_from_be(mfs_lut_ae *ae, const uint8_t b[0x70])
{
#define BE16(o) ((uint16_t)(b[o] << 8 | b[(o) + 1]))
#define BE32(o) ((uint32_t)b[o] << 24 | (uint32_t)b[(o) + 1] << 16 | (uint32_t)b[(o) + 2] << 8 | b[(o) + 3])
    memset(ae, 0, sizeof *ae);
    for (int c = 0; c < 3; c++) {
        uint64_t u = (uint64_t)BE32(8 * c) << 32 | BE32(8 * c + 4);
        memcpy(&ae->gain[c], &u, 8);
        ae->level[c].shadow = BE16(0x18 + 8 * c);
        ae->level[c].highlight = BE16(0x1a + 8 * c);
        ae->level[c].u1c = BE16(0x1c + 8 * c);
        ae->level[c].u1e = BE16(0x1e + 8 * c);
        const int o = 0x30 + 0x14 * c;
        mfs_lut_ae_nega *n = &ae->nega[c];
        n->contrast = BE16(o);
        n->base = BE16(o + 2);
        n->ref_level = BE16(o + 4);
        n->ref_target = BE16(o + 6);
        n->contrast_adj = (int16_t)BE16(o + 8);
        n->balance_mode = b[o + 10];
        n->pad_0b = b[o + 11];
        u = (uint64_t)BE32(o + 12) << 32 | BE32(o + 16);
        memcpy(&n->balance, &u, 8);
    }
    ae->pad_6c = BE32(0x6c);
#undef BE16
#undef BE32
}

static mfs_lut_state g_state; /* TOC-0x77f0 / TOC-0x77e8 */

/* FUN_10036f40 */
int mfs_lut_film_curve(const mfs_lut_params *p, const mfs_lut_desc lut[3],
                       mfs_lut_state *st)
{
    if (!st) st = &g_state;
    for (int c = 0; c < 3; c++)
        if (lut[c].in_code > 4 || lut[c].out_code > 4) return -1;
    switch (p->film_type) {
    case MFS_FILM_BW_NEGA:
        for (uint32_t c = 0; c < 3; c++) {
            if (p->fixed_curve == 1)
                curve_nega_fixed(&lut[c], 1.0); /* FUN_1003a1c0 */
            else
                curve_bw_nega_ae(&lut[c], p->ae.level[c].shadow, 100, p->ae.level[c].highlight);
        }
        return 0;
    case MFS_FILM_COLOR_NEGA:
        if (p->fixed_curve == 1) {
            for (uint32_t c = 0; c < 3; c++)
                curve_nega_fixed(&lut[c], nega_fixed_gain(p->product_type, c));
            return 0;
        }
        /* green first: it sets the balance reference for red and blue */
        if (curve_nega_ae(&lut[1], &p->ae.nega[1], 1, p->product_type, st)) return -1;
        if (curve_nega_ae(&lut[0], &p->ae.nega[0], 0, p->product_type, st)) return -1;
        if (curve_nega_ae(&lut[2], &p->ae.nega[2], 2, p->product_type, st)) return -1;
        return 0;
    case MFS_FILM_COLOR_POSI:
    case MFS_FILM_BW_POSI:
        for (int c = 0; c < 3; c++) curve_positive(&lut[c], 100);
        return 0;
    case MFS_FILM_BW_LINEAR:
        for (int c = 0; c < 3; c++) curve_linear_inv(&lut[c], 100);
        return 0;
    case MFS_FILM_COLOR_LINEAR:
        for (int c = 0; c < 3; c++) curve_linear(&lut[c], 100);
        return 0;
    default:
        return -1;
    }
}

/* ------------------------------------------------------- LUT conversion */

/* FUN_10037d70 */
int mfs_lut_convert(const mfs_lut_desc *src, const mfs_lut_desc *dst)
{
    if (!src->table || !dst->table) return -1;
    if (src->in_code > 4 || src->out_code > 4 || dst->in_code > 4 || dst->out_code > 4)
        return -1;
    const uint32_t nin_s = mfs_lut_max[src->in_code], nout_s = mfs_lut_max[src->out_code];
    const uint32_t nin_d = mfs_lut_max[dst->in_code], nout_d = mfs_lut_max[dst->out_code];
    const double kx = (double)nin_d / (double)nin_s;
    const double ky = (double)nout_d / (double)nout_s;
    const uint32_t s16 = src->out_code != 0, d16 = dst->out_code != 0;
    const void *S = src->table;
    void *D = dst->table;

    if (!s16 && d16) {
        /* byte valued source into a 16-bit table: piecewise linear over runs
         * of equal values (max 3) or of equal steps */
        const uint8_t *s8 = S;
        uint32_t j = 0;
        while ((j & 0xffff) < nin_s) {
            uint32_t j16 = j & 0xffff, k = 1;
            double y0 = s8[j16], y1 = s8[j16 + 1];
            while (y0 == y1) {
                if ((int32_t)(j16 + (k & 0xffff) + 1) >= (int32_t)(nin_s + 1)) break;
                y1 = s8[j16 + (k & 0xffff) + 1];
                k++;
                if ((k & 0xffff) > 3) break;
            }
            if ((k & 0xffff) <= 3) {
                while ((k & 0xffff) > 1) {
                    uint32_t k16 = k & 0xffff;
                    if ((int32_t)(j16 + k16 + 1) >= (int32_t)(nin_s + 1)) break;
                    if ((int32_t)s8[j16 + k16] - s8[j16 + k16 - 1] !=
                        (int32_t)s8[j16 + k16 + 1] - s8[j16 + k16]) break;
                    y1 = s8[j16 + k16 + 1];
                    k++;
                }
            } else {
                k = 1;
            }
            const uint32_t base = (uint32_t)fctiwz((double)j16 * kx) & 0xffff;
            for (uint32_t m = 0; (double)(m & 0xffff) < kx * (double)(k & 0xffff); m++) {
                double t = (double)(m & 0xffff) / (kx * (double)(k & 0xffff));
                double v = ky * fma(t, y1 - y0, y0);
                uint32_t idx = (m & 0xffff) + base;
                if (idx <= nin_d) ((uint16_t *)D)[idx] = (uint16_t)clampu(d2u(v), 0, nout_d);
            }
            j += k;
        }
        ((uint16_t *)D)[nin_d] = (uint16_t)clampu(d2u(ky * (double)s8[nin_s]), 0, nout_d);
        return 0;
    }

    /* linear interpolation at i / kx */
    for (uint32_t i = 0; (i & 0xffff) < nin_d; i++) {
        double fi = (double)(i & 0xffff);
        uint32_t idx = (uint32_t)fctiwz(fi / kx) & 0xffff;
        double y0 = lut_get(S, idx, s16), y1 = lut_get(S, idx + 1, s16);
        double t = -fma(kx, (double)idx, -fi) / kx;
        double v = ky * fma(t, y1 - y0, y0);
        lut_put(D, i & 0xffff, d16, clampu(d2u(v), 0, nout_d));
    }
    lut_put(D, nin_d, d16, clampu(d2u(ky * (double)lut_get(S, nin_s, s16)), 0, nout_d));
    return 0;
}

/* ------------------------------------------------ user curve composition */

/* FUN_10037280 */
int mfs_lut_apply_user_curve(const mfs_lut_desc *film, const mfs_lut_desc *user,
                             const mfs_lut_desc *dst, double *stretch_out)
{
    if (!stretch_out || !dst->table || !film->table) return -1;
    if (dst->in_code > 4 || dst->out_code > 4) return -1;
    uint16_t *tmp_a = malloc(0x20000); /* global at 0x283968 */
    uint16_t *tmp_b = malloc(0x20000 + 4); /* global at 0x263968 (knot list) */
    if (!tmp_a || !tmp_b) { free(tmp_a); free(tmp_b); return -1; }

    /* A = film curve as a 16/16 bit table */
    const uint16_t *A;
    if (film->in_code == 4 && film->out_code == 4) {
        A = film->table;
    } else {
        mfs_lut_desc t = { tmp_a, 4, 4 };
        mfs_lut_convert(film, &t);
        A = tmp_a;
    }
    const uint32_t last_a = lut_last_change(A, 4, 4);

    /* C = user o A */
    const uint16_t *C = A;
    if (user && user->table) {
        const uint16_t *U;
        if (user->in_code == 4 && user->out_code == 4) {
            U = user->table;
        } else {
            mfs_lut_desc t = { tmp_b, 4, 4 };
            mfs_lut_convert(user, &t);
            U = tmp_b;
        }
        for (uint32_t i = 0; i < 0x10000; i++) tmp_a[i] = U[A[i]];
        C = tmp_a;
    }

    const mfs_lut_desc cd = { (void *)C, 4, 4 };
    const uint32_t nin_c = mfs_lut_max[4], nout_c = mfs_lut_max[4];
    const uint32_t nin_d = mfs_lut_max[dst->in_code], nout_d = mfs_lut_max[dst->out_code];
    double k = 1.0;

    if (lut_enough_levels(C, 4, 4) != 0) {
        mfs_lut_convert(&cd, dst);
        goto done;
    }
    const uint32_t last_c = lut_last_change(C, 4, 4);
    k = (double)nin_c / (double)(int32_t)(last_c + (nin_c - last_a));
    if (k > 10.0) k = 10.0;
    if (k < 1.0) k = 1.0;
    if (!(k > 1.0)) {
        mfs_lut_convert(&cd, dst);
        k = 1.0;
        goto done;
    }

    /* knots: every 1/40 of the used range or where the slope changes */
    uint16_t *K = tmp_b;
    uint32_t n = 0, cnt = 0;
    K[n++ & 0xffff] = 0;
    int32_t dprev = (int32_t)C[1] - C[0];
    for (uint32_t i = 1; i < last_c; i++) {
        if (cnt > (uint32_t)((int32_t)last_c / 40)) {
            K[n++ & 0xffff] = (uint16_t)i;
            cnt = 0;
            dprev = (int32_t)C[i + 1] - C[i];
        } else {
            int32_t d = (int32_t)C[i + 1] - C[i];
            if (dprev - d > 4 || dprev - d < -4) {
                K[n++ & 0xffff] = (uint16_t)i;
                cnt = 0;
            }
            dprev = d;
            cnt++;
        }
    }
    K[n++ & 0xffff] = (uint16_t)last_c;
    if ((n & 0xffff) >= 0x100 || (n & 0xffff) < 2) {
        mfs_lut_convert(&cd, dst);
        k = 1.0;
        goto done;
    }
    {
        double X[256], Y[256];
        spline_t s;
        for (uint32_t j = 0; j < n; j++) {
            X[j] = (k * (double)K[j]) * ((double)nin_d / (double)nin_c);
            Y[j] = ((double)nout_d / (double)nout_c) * (double)C[K[j]];
        }
        spline_fit(&s, X, Y, n);
        for (uint32_t i = 0; i < nin_d + 1; i++) {
            double v = spline_eval(&s, (double)i, X, Y, n);
            lut_put(dst->table, i, dst->out_code != 0, clampu(d2u(v), 0, nout_d));
        }
    }
done:
    *stretch_out = k;
    free(tmp_a);
    free(tmp_b);
    return 0;
}
