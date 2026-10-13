/*
 * Host-side multi-pass processing that the original software did not have:
 * outlier-rejecting multi-sample averaging and HDR (two exposures, aligned
 * and merged in linear light).
 */
#include "dsd4_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static void sort_d(double *v, unsigned n)
{
    for (unsigned i = 1; i < n; i++)
        for (unsigned j = i; j > 0 && v[j - 1] > v[j]; j--) { double x = v[j]; v[j] = v[j - 1]; v[j - 1] = x; }
}

static double median_sorted(const double *v, unsigned n)
{
    return n & 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/* Multi-sample: plain mean below 4 samples; otherwise the mean of the
 * samples within 3 sigma (from the median absolute deviation) of the median,
 * so a single disturbed read does not shift the pixel.  The floor of 256
 * (0.4 % of full scale) keeps quantisation-level spreads from rejecting
 * good samples. */
double dsd4_robust_mean16(const uint16_t *v, unsigned n, size_t stride)
{
    double sum = 0;
    if (n > 16) n = 16;
    if (n < 4) {
        for (unsigned i = 0; i < n; i++) sum += v[i * stride];
        return sum / n;
    }
    double a[16], d[16];
    for (unsigned i = 0; i < n; i++) a[i] = v[i * stride];
    sort_d(a, n);
    const double med = median_sorted(a, n);
    for (unsigned i = 0; i < n; i++) d[i] = fabs(a[i] - med);
    sort_d(d, n);
    double thr = 3.0 * 1.4826 * median_sorted(d, n);
    if (thr < 256) thr = 256;
    unsigned k = 0;
    for (unsigned i = 0; i < n; i++)
        if (fabs(a[i] - med) <= thr) { sum += a[i]; k++; }
    return k ? sum / k : med;
}

/* ------------------------------------------------------------------ HDR */

#define CLIP_LO 0.90 /* long exposure fully trusted below this */
#define CLIP_HI 0.98 /* and not at all above */

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

/* normalised cross-correlation of the short pass (scaled by 'gain' and
 * clipped like the long pass) against the long pass shifted by d lines,
 * over the central half of the width */
static double ncc(const dsd4_image *a, const dsd4_image *b, int c, double gain, int d)
{
    const int ch = a->channels, w = a->width, h = a->height;
    const float *A = (const float *)a->data, *B = (const float *)b->data;
    double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
    long n = 0;
    for (int y = 8; y < h - 8; y += 1) {
        int yb = y + d;
        if (yb < 0 || yb >= h) continue;
        for (int x = w / 4; x < 3 * w / 4; x += 2) {
            double va = A[((size_t)y * w + x) * ch + c] * gain, vb = B[((size_t)yb * w + x) * ch + c];
            if (va > CLIP_LO) va = CLIP_LO;
            if (vb > CLIP_LO) vb = CLIP_LO;
            sa += va; sb += vb; saa += va * va; sbb += vb * vb; sab += va * vb;
            n++;
        }
    }
    if (n < 16) return -1;
    double cov = sab - sa * sb / n, va = saa - sa * sa / n, vb = sbb - sb * sb / n;
    return va > 0 && vb > 0 ? cov / sqrt(va * vb) : -1;
}

/* vertical offset of b relative to a (b(y + shift) matches a(y)), sub-pixel */
static double find_shift(const dsd4_image *a, const dsd4_image *b, double gain)
{
    enum { R = 4 };
    const int c = a->channels == 3 ? 1 : 0;
    double cc[2 * R + 1];
    int best = 0;
    for (int d = -R; d <= R; d++) {
        cc[d + R] = ncc(a, b, c, gain, d);
        if (cc[d + R] > cc[best + R]) best = d;
    }
    double shift = best;
    if (best > -R && best < R) {
        double l = cc[best + R - 1], m = cc[best + R], r = cc[best + R + 1];
        double den = l - 2 * m + r;
        if (den < 0) shift += 0.5 * (l - r) / den;
    }
    return shift;
}

/* long/short ratio per channel from pixels both passes captured well */
static double measure_ratio(const dsd4_image *a, const dsd4_image *b, int c, double shift, double nominal)
{
    const int ch = a->channels, w = a->width, h = a->height;
    const float *A = (const float *)a->data, *B = (const float *)b->data;
    size_t cap = 400000, n = 0;
    float *r = malloc(cap * sizeof *r);
    if (!r) return nominal;
    const int di = (int)floor(shift);
    const double f = shift - di;
    for (int y = 0; y < h && n < cap; y += 3) {
        int y0 = y + di, y1 = y0 + 1;
        if (y0 < 0 || y1 >= h) continue;
        for (int x = 0; x < w && n < cap; x += 3) {
            double s = A[((size_t)y * w + x) * ch + c];
            double l = (1 - f) * B[((size_t)y0 * w + x) * ch + c] + f * B[((size_t)y1 * w + x) * ch + c];
            if (s > 0.01 && s < 0.25 && l < 0.85) r[n++] = (float)(l / s);
        }
    }
    double ratio = nominal;
    if (n >= 1000) {
        qsort(r, n, sizeof *r, cmp_f);
        ratio = r[n / 2];
    }
    free(r);
    return ratio;
}

int dsd4_scan_hdr(dsd4 *s, const dsd4_scan_params *sp, double ev, dsd4_image *img, dsd4_hdr_info *info,
                  dsd4_progress_fn cb, void *ud)
{
    memset(img, 0, sizeof *img);
    if (info) memset(info, 0, sizeof *info);
    if (!(ev > 0 && ev <= 6) || sp->preview) return DSD4_ERR_PARAM;
    dsd4_scan_params p = *sp;
    p.float_out = 1;
    p.raw = 1;
    int r;
    /* frame and focus once, shared by both passes */
    if (p.frame > 0) {
        if (cb) cb(ud, "moving to frame", -1);
        if ((r = dsd4_select_frame(s, p.frame))) return r;
        p.frame = 0;
    }
    if (p.focus == DSD4_FOCUS_AUTO) {
        int best = 0;
        if ((r = dsd4_autofocus(s, &p, &best, cb, ud))) return r;
        p.focus = best;
    }
    if (cb) cb(ud, "HDR pass 1 of 2", -1);
    dsd4_image a, b;
    if ((r = dsd4_scan(s, &p, &a, cb, ud))) return r;

    dsd4_scan_params q = p;
    double std[3];
    dsd4_film_default_exposure(p.film, std);
    const double k = pow(2.0, ev);
    for (int c = 0; c < 3; c++) q.exposure[c] = (p.exposure[c] > 0 ? p.exposure[c] : std[c]) * k;
    q.unlimited_exposure = 1;
    if (cb) cb(ud, "HDR pass 2 of 2", -1);
    if ((r = dsd4_scan(s, &q, &b, cb, ud))) { dsd4_image_free(&a); return r; }
    if (a.width != b.width || a.height != b.height || a.channels != b.channels) {
        dsd4_image_free(&a);
        dsd4_image_free(&b);
        return DSD4_ERR_SCANNER;
    }

    if (cb) cb(ud, "aligning and merging", -1);
    const int ch = a.channels, w = a.width, h = a.height;
    const double shift = find_shift(&a, &b, k);
    double ratio[3] = { k, k, k };
    for (int c = 0; c < ch; c++) ratio[c] = measure_ratio(&a, &b, c, shift, k);
    if (ch == 1) ratio[1] = ratio[2] = ratio[0];
    const double got = log2((ratio[0] + ratio[1] + ratio[2]) / 3);
    dsd4_logf(s, 1, "HDR: second pass +%.2f EV (ratio %.2f/%.2f/%.2f), offset %.2f lines", got, ratio[0],
              ratio[1], ratio[2], shift);
    if (got < 0.5) dsd4_logf(s, 1, "HDR: the scanner could not expose the second pass much longer; little gain");

    float *A = (float *)a.data;
    const float *B = (const float *)b.data;
    const int di = (int)floor(shift);
    const double f = shift - di;
    for (int y = 0; y < h; y++) {
        int y0 = y + di, y1 = y0 + 1;
        if (y0 < 0 || y1 >= h) continue; /* edge rows: short pass only */
        for (int x = 0; x < w; x++)
            for (int c = 0; c < ch; c++) {
                size_t i = ((size_t)y * w + x) * ch + c;
                double l = (1 - f) * B[((size_t)y0 * w + x) * ch + c] + f * B[((size_t)y1 * w + x) * ch + c];
                double t = l <= CLIP_LO ? 1 : l >= CLIP_HI ? 0 : (CLIP_HI - l) / (CLIP_HI - CLIP_LO);
                t = t * t * (3 - 2 * t);
                A[i] = (float)(t * l / ratio[c] + (1 - t) * A[i]);
            }
    }
    dsd4_image_free(&b);
    *img = a;
    if (info) {
        info->ev = got;
        info->shift = shift;
        memcpy(info->ratio, ratio, sizeof ratio);
    }
    return DSD4_OK;
}
