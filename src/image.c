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

/* Negative -> positive.  A negative records density, so the inversion works
 * on log values: per channel the film base (the clearest film, including the
 * orange mask) is the 99.5th percentile and the densest image area the 0.5th.
 * Densities relative to the base are normalised to that range per channel,
 * which removes the mask and the channels' different contrast, then mapped
 * back to linear light over a 2.0 density (about 6.6 stop) range.
 * auto_levels 0 uses the extreme values instead of percentiles. */
void dsd4_invert_negative(dsd4_image *img, int auto_levels)
{
    if (img->bits != 8 && img->bits != 16) return;
    const int ch = img->channels;
    const size_t n = (size_t)img->width * img->height;
    const int max = img->bits == 16 ? 65535 : 255;
    const double R = 2.0, lfloor = pow(10, -R);
    uint32_t *hist = malloc(((size_t)max + 1) * sizeof *hist);
    uint16_t *lut = malloc(((size_t)max + 1) * sizeof *lut);
    if (!hist || !lut) { free(hist); free(lut); return; }
    for (int c = 0; c < ch; c++) {
        memset(hist, 0, ((size_t)max + 1) * sizeof *hist);
        for (size_t i = 0; i < n; i++)
            hist[img->bits == 16 ? ((uint16_t *)img->data)[i * ch + c] : img->data[i * ch + c]]++;
        const size_t tail = auto_levels ? n / 200 : 0;
        size_t acc = 0;
        int lo = 0, hi = max;
        while (lo < max && acc + hist[lo] <= tail) acc += hist[lo++];
        acc = 0;
        while (hi > lo && acc + hist[hi] <= tail) acc += hist[hi--];
        const double base = hi > 0 ? hi : 1, dense = lo > 0 ? lo : 0.5;
        double dmax = log10(base / dense);
        if (dmax < 0.05) dmax = 0.05;
        for (int v = 0; v <= max; v++) {
            double d = log10(base / (v > 0 ? v : 0.5)) / dmax;
            d = d < 0 ? 0 : d > 1 ? 1 : d;
            double l = (pow(10, R * (d - 1)) - lfloor) / (1 - lfloor);
            lut[v] = (uint16_t)(l * max + 0.5);
        }
        for (size_t i = 0; i < n; i++) {
            size_t k = i * ch + c;
            if (img->bits == 16) ((uint16_t *)img->data)[k] = lut[((uint16_t *)img->data)[k]];
            else img->data[k] = (uint8_t)lut[img->data[k]];
        }
    }
    free(hist);
    free(lut);
}

/* Minimal TIFF reader: one IFD, uncompressed, chunky, 8/16 bit, 1 or 3
 * samples, any strip layout, either byte order. */
int dsd4_read_tiff(const char *path, dsd4_image *img)
{
    memset(img, 0, sizeof *img);
    FILE *f = fopen(path, "rb");
    if (!f) return DSD4_ERR_IO;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size < 16) { fclose(f); return DSD4_ERR_PARAM; }
    uint8_t *d = malloc((size_t)size);
    if (!d) { fclose(f); return DSD4_ERR_NOMEM; }
    if (fread(d, 1, (size_t)size, f) != (size_t)size) { fclose(f); free(d); return DSD4_ERR_IO; }
    fclose(f);
    int be = d[0] == 'M';
#define U16(o) (uint32_t)(be ? be16(d + (o)) : le16(d + (o)))
#define U32(o) (be ? be32(d + (o)) : ((uint32_t)le16(d + (o)) | (uint32_t)le16(d + (o) + 2) << 16))
    int r = DSD4_ERR_PARAM;
    uint32_t ifd = U32(4);
    if ((d[0] != 'I' && d[0] != 'M') || U16(2) != 42 || ifd + 2 > (uint32_t)size) goto out;
    uint32_t n = U16(ifd), w = 0, h = 0, bps = 8, spp = 1, comp = 1, planar = 1, xres_off = 0;
    uint32_t so_cnt = 0, so_off = 0, so_type = 4, sb_off = 0, sb_type = 4;
    for (uint32_t i = 0; i < n && ifd + 2 + 12 * (i + 1) <= (uint32_t)size; i++) {
        uint32_t e = ifd + 2 + 12 * i, tag = U16(e), type = U16(e + 2), cnt = U32(e + 4);
        uint32_t val = type == 3 ? U16(e + 8) : U32(e + 8);
        switch (tag) {
        case 256: w = val; break;
        case 257: h = val; break;
        case 258: bps = cnt == 1 ? val : U16(U32(e + 8)); break;
        case 259: comp = val; break;
        case 277: spp = val; break;
        case 284: planar = val; break;
        case 282: xres_off = U32(e + 8); break;
        case 273: so_cnt = cnt; so_type = type; so_off = cnt == 1 ? (uint32_t)-1 : U32(e + 8);
                  if (cnt == 1) so_off = val | 0x80000000u; break;
        case 279: sb_type = type; sb_off = cnt == 1 ? (val | 0x80000000u) : U32(e + 8); break;
        }
    }
    if (!w || !h || comp != 1 || planar != 1 || (bps != 8 && bps != 16) || (spp != 1 && spp != 3) || !so_cnt) goto out;
    size_t bytes = (size_t)w * h * spp * (bps / 8);
    uint8_t *px = malloc(bytes);
    if (!px) { r = DSD4_ERR_NOMEM; goto out; }
    size_t done = 0;
    for (uint32_t s = 0; s < so_cnt && done < bytes; s++) {
        uint32_t off, len;
        if (so_off & 0x80000000u && so_cnt == 1) off = so_off & 0x7fffffffu;
        else off = so_type == 3 ? U16(so_off + 2 * s) : U32(so_off + 4 * s);
        if (sb_off & 0x80000000u && so_cnt == 1) len = sb_off & 0x7fffffffu;
        else len = sb_type == 3 ? U16(sb_off + 2 * s) : U32(sb_off + 4 * s);
        if ((uint64_t)off + len > (uint64_t)size) { free(px); goto out; }
        if (len > bytes - done) len = (uint32_t)(bytes - done);
        memcpy(px + done, d + off, len);
        done += len;
    }
    if (done != bytes) { free(px); goto out; }
    if (bps == 16 && be)
        for (size_t i = 0; i < bytes; i += 2) { uint8_t t = px[i]; px[i] = px[i + 1]; px[i + 1] = t; }
    img->width = (int)w;
    img->height = (int)h;
    img->channels = (int)spp;
    img->bits = (int)bps;
    img->stride = (size_t)w * spp * (bps / 8);
    img->data = px;
    img->dpi = 0;
    if (xres_off && xres_off + 8 <= (uint32_t)size && U32(xres_off + 4))
        img->dpi = (int)(U32(xres_off) / U32(xres_off + 4));
    for (int c = 0; c < 3; c++) img->digital_gain[c] = 1.0;
    r = DSD4_OK;
out:
#undef U16
#undef U32
    free(d);
    return r;
}
