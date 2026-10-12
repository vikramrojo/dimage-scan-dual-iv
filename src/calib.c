/*
 * Shading calibration math.  Ports of GetAverageData, BubbleSort,
 * CalculateDarkShadingData, CalculateWhiteShadingData and DarkSmoothProcess
 * from MFSBaseLib2891.
 *
 * Calibration format (READ 0x60, 32 bytes):
 *   0-1  pixels per line (BE)      2 bytes per channel   3 lines
 *   4    state flags               5 ability (bit 2: dark data, bit 3: color)
 *   9,11,13  white shading targets R,G,B (BE)
 *   15,17,19 dark shading targets  R,G,B (BE, 0xffff = none)
 *   21,23,25 base exposure R,G,B   27 max exposure   29 min exposure (LE)
 */
#include "dsd4_internal.h"

#include <stdlib.h>
#include <string.h>

/* BubbleSort: move the darkest third to the front and average the rest. */
uint16_t cal_bubble_average(uint16_t *v, unsigned n)
{
    n &= 0xff;
    unsigned third = n / 3;
    for (unsigned i = 0; i < third; i++)
        for (unsigned j = (i + 1) & 0xff; j < n; j++)
            if (v[j] < v[i]) { uint16_t t = v[i]; v[i] = v[j]; v[j] = t; }
    uint32_t sum = 0;
    uint8_t cnt = 0;
    for (unsigned i = third; i < n; i++) { sum += v[i]; cnt++; }
    if (cnt) sum /= cnt;
    return (uint16_t)sum;
}

/* GetAverageData: for every sample position (pixel x channel) average the
 * calibration lines.  8 bit data is promoted with <<8 (SwapWORD of a byte),
 * 16 bit data is little-endian. */
uint16_t *cal_average(const uint8_t fmt[32], uint32_t count, const uint8_t *raw)
{
    const unsigned lines = fmt[3];
    const unsigned bpc = fmt[2];
    uint16_t *tmp = malloc((lines ? lines : 1) * sizeof *tmp);
    uint16_t *out = calloc(count ? count : 1, sizeof *out);
    if (!tmp || !out) { free(tmp); free(out); return NULL; }
    const size_t stride = (size_t)count * bpc;
    for (uint32_t i = 0; i < count; i++) {
        if (bpc == 1) {
            for (unsigned j = 0; j < lines; j++) tmp[j] = (uint16_t)(raw[i + j * stride] << 8);
        } else {
            for (unsigned j = 0; j < lines; j++) tmp[j] = le16(raw + (size_t)i * 2 + j * stride);
        }
        out[i] = cal_bubble_average(tmp, lines);
    }
    free(tmp);
    return out;
}

/* CalculateDarkShadingData: subtract the dark target, clamping at zero. */
void cal_dark_shading(dsd4 *s, const uint8_t fmt[32], uint32_t count, uint16_t *dark)
{
    const unsigned ch = s->cal_mode == 2 ? 4 : 3;
    uint16_t def = s->ab.shading_target == 0xffff ? 0 : s->ab.shading_target;
    uint16_t t[4] = { def, def, def, def };
    const uint16_t r = be16(fmt + 15), g = be16(fmt + 17), b = be16(fmt + 19);
    if (s->image_mode == 4 || (be32(fmt + 4) & 0x80000)) {
        if (r != 0xffff) t[0] = r;
        if (g != 0xffff) t[1] = g;
        if (b != 0xffff) t[2] = b;
        if (s->cal_mode == 2 && r != 0xffff) t[3] = r;
    } else if (r != 0xffff) {
        t[0] = t[1] = t[2] = r;
        if (s->cal_mode == 2) t[3] = r;
    }
    if (s->is16)
        for (unsigned i = 0; i < ch; i++) t[i] = (uint16_t)(t[i] << 8);
    for (uint32_t i = 0; i < count; i++) {
        uint16_t tt = t[i % ch];
        dark[i] = tt < dark[i] ? (uint16_t)(dark[i] - tt) : 0;
    }
}

/* CalculateWhiteShadingData: per-sample gain = target * 0x4000 / white. */
void cal_white_shading(dsd4 *s, const uint8_t fmt[32], uint32_t count, uint16_t *white)
{
    const unsigned ch = s->cal_mode == 2 ? 4 : 3;
    uint32_t def = s->ab.shading_target ? s->ab.shading_target : 0xfff0;
    uint16_t thr[4], tgt[4];
    for (unsigned i = 0; i < 4; i++) { tgt[i] = (uint16_t)(def << 4); thr[i] = 0x4000; }
    const uint16_t r = be16(fmt + 9), g = be16(fmt + 11), b = be16(fmt + 13);
    if (s->image_mode == 4 || (be32(fmt + 4) & 0x80000)) {
        if (r) { s->white_target[0] = r; tgt[0] = r; }
        if (g) { s->white_target[1] = g; tgt[1] = g; }
        if (b) { s->white_target[2] = b; tgt[2] = b; }
        if (s->cal_mode == 2 && r) { s->white_target[3] = r; tgt[3] = r; }
    } else if (r) {
        tgt[0] = tgt[1] = tgt[2] = r;
        if (s->cal_mode == 2) tgt[3] = r;
    }
    if (s->is16) {
        for (unsigned i = 0; i < ch; i++) {
            tgt[i] = (uint16_t)(tgt[i] << 8);
            if (s->cal_mode == 3) s->white_target[i] = (uint16_t)(s->white_target[i] << 8);
        }
    } else {
        for (unsigned i = 0; i < ch; i++)
            if (tgt[i] < 0x100) tgt[i] = (uint16_t)(tgt[i] << 4);
    }
    for (uint32_t i = 0; i < count; i++) {
        unsigned c = i % ch;
        if (white[i] < thr[c]) white[i] = tgt[c];
        else white[i] = (uint16_t)(int)(0.5 + ((double)tgt[c] * (double)thr[c]) / (double)white[i]);
    }
    if (!s->is16 && r < 0x100)
        for (uint32_t i = 0; i < count; i++) white[i] = (uint16_t)(white[i] << 4);
}

/* DarkSmoothProcess: replace dark values that are within 35 of a 600 pixel
 * running average by that average. */
int cal_dark_smooth(uint16_t *d, uint32_t count)
{
    const uint32_t n = count / 3;
    if (n < 600) return 1;
    uint16_t *sm = malloc((size_t)n * 6 + 100);
    if (!sm) return 0;
    for (uint32_t i = 0; i < n; i++) {
        uint32_t from, len;
        if (i < 300) { from = 0; len = i + 300; }
        else if (i < n - 300) { from = i - 300; len = 600; }
        else { from = i - 300; len = (n - i) + 300; }
        uint32_t s0 = 0, s1 = 0, s2 = 0;
        for (uint32_t k = 0; k < len; k++) {
            const uint16_t *p = d + (size_t)(from + k) * 3;
            s0 += p[0]; s1 += p[1]; s2 += p[2];
        }
        sm[i * 3] = (uint16_t)(s0 / len);
        sm[i * 3 + 1] = (uint16_t)(s1 / len);
        sm[i * 3 + 2] = (uint16_t)(s2 / len);
    }
    for (uint32_t i = 0; i < n * 3; i++) {
        int diff = (int)sm[i] - (int)d[i];
        if (diff < 0) diff = -diff;
        if (diff < 0x23) d[i] = sm[i];
    }
    free(sm);
    return 1;
}
