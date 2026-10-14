/*
 * Image helpers shared by deblur and enhance: float planes (row-major,
 * w x h), mirrored edges; FFT tiles with cross-faded blending.
 */
#include "dsd4_internal.h"
#include "imgops.h"

#include <dispatch/dispatch.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static void gauss_1d(float *k, int r, double s)
{
    double sum = 0;
    for (int i = -r; i <= r; i++) sum += k[i + r] = (float)exp(-0.5 * i * i / (s * s));
    for (int i = 0; i <= 2 * r; i++) k[i] = (float)(k[i] / sum);
}

/* separable gaussian blur, mirrored edges; out may not alias in */
int io_gauss_blur(const float *in, float *out, int w, int h, double s)
{
    const int r = (int)ceil(3 * s);
    float *k = malloc((size_t)(2 * r + 1) * sizeof *k), *t = malloc((size_t)w * h * sizeof *t);
    if (!k || !t) { free(k); free(t); return -1; }
    gauss_1d(k, r, s);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float a = 0;
            for (int i = -r; i <= r; i++) a += k[i + r] * in[(size_t)y * w + io_mirror(x + i, w)];
            t[(size_t)y * w + x] = a;
        }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float a = 0;
            for (int i = -r; i <= r; i++) a += k[i + r] * t[(size_t)io_mirror(y + i, h) * w + x];
            out[(size_t)y * w + x] = a;
        }
    free(k);
    free(t);
    return 0;
}

int io_cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

/* 5x5 median: removes dust specks and hairs, which are sharp (on the film,
 * not in the photograph): they would make the picture look sharper than it
 * is, and ring if deconvolved.  Rows in parallel. */
typedef struct { const float *in; float *out; int w, h; } med_t;

static void median5_row(void *ctx, size_t row)
{
    const med_t *m = ctx;
    const float *in = m->in;
    const int w = m->w, h = m->h, y = (int)row;
    float *out = m->out;
    {
        for (int x = 0; x < w; x++) {
            float v[25];
            int n = 0;
            for (int j = -2; j <= 2; j++)
                for (int i = -2; i <= 2; i++) {
                    float a = in[(size_t)io_mirror(y + j, h) * w + io_mirror(x + i, w)];
                    int k = n++;
                    while (k > 0 && v[k - 1] > a) { v[k] = v[k - 1]; k--; }
                    v[k] = a;
                }
            out[(size_t)y * w + x] = v[12];
        }
    }
}

void io_median5(const float *in, float *out, int w, int h)
{
    med_t m = { in, out, w, h };
    dispatch_apply_f((size_t)h, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), &m, median5_row);
}

/* running min or max over a (2r+1)^2 window, separable; mirrored edges */
int io_minmax_filter(const float *in, float *out, int w, int h, int r, int want_max)
{
    float *t = malloc((size_t)w * h * sizeof *t);
    if (!t) return -1;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float m = in[(size_t)y * w + x];
            for (int i = -r; i <= r; i++) {
                float v = in[(size_t)y * w + io_mirror(x + i, w)];
                m = want_max ? (v > m ? v : m) : (v < m ? v : m);
            }
            t[(size_t)y * w + x] = m;
        }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            float m = t[(size_t)y * w + x];
            for (int i = -r; i <= r; i++) {
                float v = t[(size_t)io_mirror(y + i, h) * w + x];
                m = want_max ? (v > m ? v : m) : (v < m ? v : m);
            }
            out[(size_t)y * w + x] = m;
        }
    free(t);
    return 0;
}

/* an n x n window of a plane, top left at (x0, y0), mirrored at the edges */
void io_tile_load(const float *p, int w, int h, int x0, int y0, int n, float *out)
{
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) out[y * n + x] = p[(size_t)io_mirror(y0 + y, h) * w + io_mirror(x0 + x, w)];
}

/* add an n x n tile (with pad px of context on each side) into acc/wsum:
 * weight 1 in the middle, ramps over pad/2 each side so that neighbouring
 * tiles (stride n - 2 pad) cross-fade */
void io_tile_blend(float *acc, float *wsum, int w, int h, int x0, int y0, int n, int pad, const float *t,
                   dispatch_semaphore_t lock)
{
    /* tiles must reach at least pad - ramp past an image edge (callers keep
     * pad of mirrored margin there), so edge pixels always have weight */
    const int ramp = pad / 2;
    dispatch_semaphore_wait(lock, DISPATCH_TIME_FOREVER);
    for (int y = pad - ramp; y < n - pad + ramp; y++) {
        const int iy = y0 + y;
        if (iy < 0 || iy >= h) continue;
        float wy = y < pad + ramp ? (float)(y - (pad - ramp) + 0.5f) / (2 * ramp)
                 : y >= n - pad - ramp ? (float)(n - pad + ramp - y - 0.5f) / (2 * ramp) : 1;
        for (int x = pad - ramp; x < n - pad + ramp; x++) {
            const int ix = x0 + x;
            if (ix < 0 || ix >= w) continue;
            float wx = x < pad + ramp ? (float)(x - (pad - ramp) + 0.5f) / (2 * ramp)
                     : x >= n - pad - ramp ? (float)(n - pad + ramp - x - 0.5f) / (2 * ramp) : 1;
            const size_t o = (size_t)iy * w + ix;
            acc[o] += wx * wy * t[y * n + x];
            wsum[o] += wx * wy;
        }
    }
    dispatch_semaphore_signal(lock);
}
