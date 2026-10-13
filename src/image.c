/*
 * Output helpers: a dependency-free baseline TIFF writer (8/16 bit, RGB or
 * gray, with resolution tags) and a simple negative inversion.
 */
#include "dsd4_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static void w16(FILE *f, uint16_t v) { fputc(v & 0xff, f); fputc(v >> 8, f); }
static void w32(FILE *f, uint32_t v) { w16(f, (uint16_t)v); w16(f, (uint16_t)(v >> 16)); }

static void entry(FILE *f, uint16_t tag, uint16_t type, uint32_t count, uint32_t value)
{
    w16(f, tag);
    w16(f, type);
    w32(f, count);
    if (type == 3 && count == 1) { w16(f, (uint16_t)value); w16(f, 0); }
    else w32(f, value);
}

int dsd4_write_tiff(const dsd4_image *img, const char *path)
{
    return dsd4_write_tiff_icc(img, path, NULL, 0);
}

int dsd4_write_tiff_icc(const dsd4_image *img, const char *path, const void *icc, size_t icc_len)
{
    if (!icc) icc_len = 0;
    FILE *f = fopen(path, "wb");
    if (!f) return DSD4_ERR_IO;
    const uint32_t bytes = (uint32_t)(img->stride * img->height);
    const uint16_t nent = icc_len ? 15 : 14;
    const uint32_t ifd = 8;
    const uint32_t extra = ifd + 2 + nent * 12 + 4;   /* after the IFD */
    const uint32_t bps_off = extra;                    /* 3 x SHORT */
    const uint32_t xres_off = bps_off + 8;
    const uint32_t yres_off = xres_off + 8;
    const uint32_t sw_off = yres_off + 8;
    static const char sw[] = "dsdual4 (DiMAGE Scan Dual IV driver)";
    const uint32_t icc_off = (sw_off + (uint32_t)sizeof sw + 3) & ~3u;
    const uint32_t data_off = (icc_off + (uint32_t)icc_len + 15) & ~15u;

    fputs("II", f);
    w16(f, 42);
    w32(f, ifd);
    w16(f, nent);
    entry(f, 256, 4, 1, (uint32_t)img->width);
    entry(f, 257, 4, 1, (uint32_t)img->height);
    if (img->channels == 3) entry(f, 258, 3, 3, bps_off);
    else entry(f, 258, 3, 1, (uint32_t)img->bits);
    entry(f, 259, 3, 1, 1);                                 /* no compression */
    entry(f, 262, 3, 1, img->channels == 3 ? 2 : 1);        /* RGB / BlackIsZero */
    entry(f, 273, 4, 1, data_off);
    entry(f, 274, 3, 1, 1);
    entry(f, 277, 3, 1, (uint32_t)img->channels);
    entry(f, 278, 4, 1, (uint32_t)img->height);
    entry(f, 279, 4, 1, bytes);
    entry(f, 282, 5, 1, xres_off);
    entry(f, 283, 5, 1, yres_off);
    entry(f, 296, 3, 1, 2);                                 /* inch */
    entry(f, 305, 2, (uint32_t)sizeof sw, sw_off);
    if (icc_len) entry(f, 34675, 7, (uint32_t)icc_len, icc_off); /* InterColorProfile */
    w32(f, 0);
    for (int i = 0; i < 3; i++) w16(f, (uint16_t)img->bits);
    w16(f, 0);
    w32(f, (uint32_t)img->dpi); w32(f, 1);
    w32(f, (uint32_t)img->dpi); w32(f, 1);
    fwrite(sw, 1, sizeof sw, f);
    if (icc_len) {
        while ((uint32_t)ftell(f) < icc_off) fputc(0, f);
        fwrite(icc, 1, icc_len, f);
    }
    while ((uint32_t)ftell(f) < data_off) fputc(0, f);
    if (img->bits == 16) {
        /* TIFF "II" means little-endian samples */
        const size_t n = (size_t)img->width * img->height * img->channels;
        const uint16_t *p = (const uint16_t *)img->data;
        uint8_t buf[8192];
        size_t k = 0;
        for (size_t i = 0; i < n; i++) {
            put_le16(buf + k, p[i]);
            k += 2;
            if (k == sizeof buf) { fwrite(buf, 1, k, f); k = 0; }
        }
        fwrite(buf, 1, k, f);
    } else {
        fwrite(img->data, 1, bytes, f);
    }
    int ok = !ferror(f);
    fclose(f);
    return ok ? DSD4_OK : DSD4_ERR_IO;
}

void dsd4_to_display(dsd4_image *im)
{
    if (im->bits != 16) return;
    const size_t n = (size_t)im->width * im->height * im->channels;
    uint16_t *p = (uint16_t *)im->data;
    uint32_t *hist = calloc(65536, sizeof *hist);
    uint8_t *lut = malloc(65536);
    if (!hist || !lut) { free(hist); free(lut); return; }
    for (size_t i = 0; i < n; i++) hist[p[i]]++;
    size_t tail = n / 1000, acc = 0;
    int lo = 0, hi = 65535;
    while (lo < 65535 && acc + hist[lo] <= tail) acc += hist[lo++];
    acc = 0;
    while (hi > lo && acc + hist[hi] <= tail) acc += hist[hi--];
    if (hi - lo < 256) { lo = 0; hi = 65535; }
    for (int v = 0; v < 65536; v++) {
        double x = (double)(v - lo) / (hi - lo);
        x = x < 0 ? 0 : x > 1 ? 1 : x;
        lut[v] = (uint8_t)(pow(x, 1 / 2.2) * 255 + 0.5);
    }
    for (size_t i = 0; i < n; i++) im->data[i] = lut[p[i]];
    free(hist);
    free(lut);
    im->bits = 8;
    im->stride = (size_t)im->width * im->channels;
}

/* Invert a colour negative: per-channel black/white points from the 0.1 % and
 * 99.9 % histogram tails (removes the orange mask), then invert. */
void dsd4_invert_negative(dsd4_image *img, int auto_levels)
{
    const int ch = img->channels;
    const size_t n = (size_t)img->width * img->height;
    const int max = img->bits == 16 ? 65535 : 255;
    for (int c = 0; c < ch; c++) {
        int lo = 0, hi = max;
        if (auto_levels) {
            uint32_t *hist = calloc((size_t)max + 1, sizeof *hist);
            if (!hist) return;
            for (size_t i = 0; i < n; i++) {
                int v = img->bits == 16 ? ((uint16_t *)img->data)[i * ch + c] : img->data[i * ch + c];
                hist[v]++;
            }
            size_t acc = 0, tail = n / 1000;
            for (lo = 0; lo < max && acc + hist[lo] <= tail; lo++) acc += hist[lo];
            acc = 0;
            for (hi = max; hi > lo && acc + hist[hi] <= tail; hi--) acc += hist[hi];
            free(hist);
            if (hi <= lo) { lo = 0; hi = max; }
        }
        const double scale = (double)max / (hi - lo);
        for (size_t i = 0; i < n; i++) {
            size_t k = i * ch + c;
            int v = img->bits == 16 ? ((uint16_t *)img->data)[k] : img->data[k];
            double o = (hi - v) * scale;
            int r = o < 0 ? 0 : o > max ? max : (int)(o + 0.5);
            if (img->bits == 16) ((uint16_t *)img->data)[k] = (uint16_t)r;
            else img->data[k] = (uint8_t)r;
        }
    }
}
