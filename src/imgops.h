/* image helpers shared by deblur and enhance (imgops.c) */
#ifndef DSD4_IMGOPS_H
#define DSD4_IMGOPS_H

#include "dsd4.h"

#include <dispatch/dispatch.h>

static inline int io_mirror(int i, int n)
{
    while (i < 0 || i >= n) i = i < 0 ? -i - 1 : 2 * n - i - 1;
    return i;
}

/* sample c of pixel (x, y), 0..1 (float: as stored) */
static inline float io_sample(const dsd4_image *im, int x, int y, int c)
{
    const uint8_t *row = im->data + (size_t)y * im->stride;
    const size_t i = (size_t)x * im->channels + c;
    return im->bits == 8 ? row[i] / 255.0f : im->bits == 16 ? ((const uint16_t *)row)[i] / 65535.0f
                                                           : ((const float *)row)[i];
}

static inline void io_put(dsd4_image *im, int x, int y, int c, float v)
{
    uint8_t *row = im->data + (size_t)y * im->stride;
    const size_t i = (size_t)x * im->channels + c;
    if (im->bits == 32) { ((float *)row)[i] = v > 0 ? v : 0; return; }
    v = v < 0 ? 0 : v > 1 ? 1 : v;
    if (im->bits == 16) ((uint16_t *)row)[i] = (uint16_t)(v * 65535.0f + 0.5f);
    else row[i] = (uint8_t)(v * 255.0f + 0.5f);
}

int  io_cmp_f(const void *a, const void *b);
int  io_gauss_blur(const float *in, float *out, int w, int h, double s);
void io_median5(const float *in, float *out, int w, int h);
int  io_minmax_filter(const float *in, float *out, int w, int h, int r, int want_max);
void io_tile_load(const float *p, int w, int h, int x0, int y0, int n, float *out);
void io_tile_blend(float *acc, float *wsum, int w, int h, int x0, int y0, int n, int pad, const float *t,
                   dispatch_semaphore_t lock);

#endif
