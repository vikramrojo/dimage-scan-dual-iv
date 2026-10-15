/* Experimental 4000 dpi processing on synthetic film: a slanted edge and a
 * flat area carrying fine grain, blurred by a known scanner MTF (gaussian
 * 0.8 px), plus white sensor noise.  The MTF profile is measured by the
 * slanted-edge code on a clean edge, as `research mtf` would.
 *   sharpen:      the edge's MTF50 rises by at least 30 %
 *   denoise low:  flat-area noise not above the input, colour noise down
 *   denoise high: flat-area noise down by at least 30 %
 *   nothing on:   the image is untouched; 4000 dpi: the size is x1.25 */
#include "dsd4.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { W = 1024, H = 1024 };
#define SIG 0.8      /* scanner blur */
#define GRAIN 0.05   /* film grain: relative, per film pixel */
#define SENSOR 0.003 /* sensor noise per read */

static uint64_t rng = 0x9e3779b97f4a7c15ull;
static double urand(void)
{
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    return ((rng >> 11) + 0.5) / 9007199254740992.0;
}
static double nrand(void) { return sqrt(-2 * log(urand())) * cos(6.283185307179586 * urand()); }

static void blur(float *p)
{
    int r = (int)ceil(3 * SIG);
    double k[16], s = 0;
    for (int i = -r; i <= r; i++) s += k[i + r] = exp(-0.5 * i * i / (SIG * SIG));
    for (int i = 0; i <= 2 * r; i++) k[i] /= s;
    float *t = malloc((size_t)W * H * sizeof *t);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            double a = 0;
            for (int i = -r; i <= r; i++) { int xx = x + i < 0 ? 0 : x + i >= W ? W - 1 : x + i; a += k[i + r] * p[y * W + xx]; }
            t[y * W + x] = (float)a;
        }
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            double a = 0;
            for (int i = -r; i <= r; i++) { int yy = y + i < 0 ? 0 : y + i >= H ? H - 1 : y + i; a += k[i + r] * t[yy * W + x]; }
            p[y * W + x] = (float)a;
        }
    free(t);
}

/* film: left half an opaque slanted edge (blade), right half flat; grain on
 * all of it; then the scanner (blur, sensor noise); 16-bit RGB */
static void make(dsd4_image *im, double grain, double sensor)
{
    const double th = 5 * M_PI / 180;
    float *pl[3];
    for (int c = 0; c < 3; c++) pl[c] = malloc((size_t)W * H * sizeof(float));
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            double v;
            if (x < W / 2) {
                double d = cos(th) * (x - W / 4) + sin(th) * (y - H / 2);
                v = d > 0 ? 0.05 : 0.6;
            } else v = 0.4;
            const double g = 1 + grain * nrand(), gc = grain * 0.5;
            pl[0][y * W + x] = (float)(v * g * (1 + gc * nrand()));
            pl[1][y * W + x] = (float)(v * g);
            pl[2][y * W + x] = (float)(v * g * (1 + gc * nrand()));
        }
    for (int c = 0; c < 3; c++) blur(pl[c]);
    memset(im, 0, sizeof *im);
    im->width = W; im->height = H; im->channels = 3; im->bits = 16; im->dpi = 3200;
    im->stride = (size_t)W * 6;
    im->data = malloc(im->stride * H);
    for (int c = 0; c < 3; c++) im->digital_gain[c] = 1;
    for (int i = 0; i < W * H; i++)
        for (int c = 0; c < 3; c++) {
            double v = (pl[c][i] + sensor * nrand()) * 65535;
            ((uint16_t *)im->data)[i * 3 + c] = (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v + 0.5);
        }
    for (int c = 0; c < 3; c++) free(pl[c]);
}

static dsd4_image copy(const dsd4_image *a)
{
    dsd4_image b = *a;
    b.data = malloc(a->stride * a->height);
    memcpy(b.data, a->data, a->stride * a->height);
    return b;
}

/* rms of green, and of red - green, in the flat area minus its mean */
static void flat_noise(const dsd4_image *im, double *g, double *rg)
{
    double s = 0, s2 = 0, t = 0, t2 = 0;
    long n = 0;
    const int x0 = im->width * 5 / 8, x1 = im->width * 7 / 8, y0 = im->height / 8, y1 = im->height * 7 / 8;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) {
            const uint16_t *p = (const uint16_t *)(im->data + (size_t)y * im->stride) + (size_t)x * 3;
            double G = p[1] / 65535.0, d = (p[0] - (double)p[1]) / 65535.0;
            s += G; s2 += G * G; t += d; t2 += d * d; n++;
        }
    *g = sqrt(s2 / n - (s / n) * (s / n));
    *rg = sqrt(t2 / n - (t / n) * (t / n));
}

/* MTF50 of the edge (left half only); curve (or NULL) gets the green MTF */
static double edge_mtf(const dsd4_image *im, double *curve)
{
    dsd4_image l = *im;
    l.width = im->width / 2;
    dsd4_mtf_profile pr;
    if (dsd4_measure_edge_mtf(&l, &pr, NULL)) return -1;
    if (curve) memcpy(curve, pr.mtf[0][1], sizeof pr.mtf[0][1]);
    return pr.mtf50[0];
}
#define edge_mtf50(im) edge_mtf(im, NULL)

int main(void)
{
    int fail = 0;
    /* calibration: a clean blurred edge (no grain, no noise) */
    dsd4_image cal;
    make(&cal, 0, 0);
    dsd4_image half = cal;
    half.width = W / 2;
    dsd4_mtf_profile mtf;
    if (dsd4_measure_edge_mtf(&half, &mtf, NULL)) { printf("FAIL enhance (calibration)\n"); return 1; }
    dsd4_image_free(&cal);
    dsd4_noise_profile np;
    memset(&np, 0, sizeof np);
    np.valid = 1;
    np.levels = 2;
    for (int c = 0; c < 3; c++) {
        np.mean[c][0] = 0.9; np.mean[c][1] = 0.01;
        np.var[c][0] = np.var[c][1] = SENSOR * SENSOR;
    }
    for (int k = 0; k < DSD4_NPS_BINS; k++) np.shape_x[k] = np.shape_y[k] = 1;

    dsd4_image in;
    make(&in, GRAIN, SENSOR);
    double c0[DSD4_MTF_BINS], c1[DSD4_MTF_BINS];
    double m0 = edge_mtf(&in, c0), g0, rg0;
    flat_noise(&in, &g0, &rg0);
    dsd4_enhance_params ep = { 1, DSD4_DENOISE_OFF, 0, &mtf, &np, 1, 0 };
    dsd4_enhance_info ei;

    dsd4_image a = copy(&in);
    int r = dsd4_enhance(&a, &ep, &ei);
    double m1 = edge_mtf(&a, c1), g1, rg1;
    double peak = 0;
    for (int k = 0; k < DSD4_MTF_BINS; k++) peak = c1[k] > peak ? c1[k] : peak;
    printf("     MTF at 0.1/0.2/0.3/0.4/0.5 cycles/px: %.2f %.2f %.2f %.2f %.2f -> %.2f %.2f %.2f %.2f %.2f (peak %.2f)\n",
           c0[6], c0[13], c0[19], c0[26], c0[32], c1[6], c1[13], c1[19], c1[26], c1[32], peak);
    if (!(peak < 1.15)) { printf("FAIL enhance-no-overshoot\n"); fail++; } else printf("ok   enhance-no-overshoot\n");
    flat_noise(&a, &g1, &rg1);
    printf("     sharpen: edge MTF50 %.3f -> %.3f cycles/px (calibrated %.3f); flat noise %.4f -> %.4f; max boost %.2f\n",
           m0, m1, mtf.mtf50[0], g0, g1, ei.gain_max);
    if (r || !(m1 > 1.3 * m0)) { printf("FAIL enhance-sharpens\n"); fail++; } else printf("ok   enhance-sharpens\n");
    if (!(g1 < 1.25 * g0)) { printf("FAIL enhance-sharpen-spares-grain\n"); fail++; }
    else printf("ok   enhance-sharpen-spares-grain\n");
    dsd4_image_free(&a);

    ep.sharpen = 0;
    ep.denoise = DSD4_DENOISE_LOW;
    a = copy(&in);
    r = dsd4_enhance(&a, &ep, &ei);
    double g2, rg2;
    flat_noise(&a, &g2, &rg2);
    printf("     denoise low: flat noise %.4f -> %.4f, colour %.4f -> %.4f (grain profiled from %.0f%% of the frame)\n", g0,
           g2, rg0, rg2, 100 * ei.flat_share);
    if (r || !(g2 <= g0) || !(rg2 < 0.85 * rg0)) { printf("FAIL enhance-denoise-low\n"); fail++; }
    else printf("ok   enhance-denoise-low\n");
    dsd4_image_free(&a);

    ep.denoise = DSD4_DENOISE_HIGH;
    a = copy(&in);
    r = dsd4_enhance(&a, &ep, &ei);
    double g3, rg3;
    flat_noise(&a, &g3, &rg3);
    printf("     denoise high: flat noise %.4f -> %.4f, colour %.4f -> %.4f\n", g0, g3, rg0, rg3);
    if (r || !(g3 < 0.7 * g0)) { printf("FAIL enhance-denoise-high\n"); fail++; }
    else printf("ok   enhance-denoise-high\n");
    dsd4_image_free(&a);

    /* a dense sky in a raw negative (low signal, soft clouds, grain), as wide
     * as a real frame: denoise must not leave bands where its patches meet,
     * least of all at the frame's edge.  Film curves stretch dense areas, so
     * even 0.3 % in raw is visible: no column may move by more than that. */
    {
        enum { SW = 3120, SH = 512 };
        dsd4_image sky;
        memset(&sky, 0, sizeof sky);
        sky.width = SW; sky.height = SH; sky.channels = 3; sky.bits = 16; sky.dpi = 3200;
        sky.stride = (size_t)SW * 6;
        sky.data = malloc(sky.stride * SH);
        for (int c = 0; c < 3; c++) sky.digital_gain[c] = 1;
        for (int y = 0; y < SH; y++)
            for (int x = 0; x < SW; x++) {
                double cl = 0.06 + 0.02 * x / SW + 0.012 * exp(-((x - 2900.0) * (x - 2900.0) + (y - 200.0) * (y - 200.0)) / (2 * 300.0 * 300.0))
                          + 0.01 * sin(x / 170.0) * cos(y / 210.0);
                for (int c = 0; c < 3; c++) {
                    double v = cl * (1 + 0.05 * nrand()) + 0.003 * nrand();
                    ((uint16_t *)sky.data)[((size_t)y * SW + x) * 3 + c] = (uint16_t)(v * 65535 + 0.5);
                }
            }
        dsd4_image out = copy(&sky);
        ep = (dsd4_enhance_params){ 0, DSD4_DENOISE_HIGH, 0, NULL, &np, 1, 0, 1 };
        r = dsd4_enhance(&out, &ep, &ei);
        /* against the noise-free sky, over 9 columns (a band is wide; the
         * grain left in a single column's mean is not a band) */
        static double clean[SW], cin[SW], cout[SW];
        for (int x = 0; x < SW; x++) {
            double sc = 0, si = 0, so = 0;
            for (int y = 0; y < SH; y++) {
                sc += 0.06 + 0.02 * x / SW + 0.012 * exp(-((x - 2900.0) * (x - 2900.0) + (y - 200.0) * (y - 200.0)) / (2 * 300.0 * 300.0))
                    + 0.01 * sin(x / 170.0) * cos(y / 210.0);
                si += ((uint16_t *)sky.data)[((size_t)y * SW + x) * 3 + 1] / 65535.0;
                so += ((uint16_t *)out.data)[((size_t)y * SW + x) * 3 + 1] / 65535.0;
            }
            clean[x] = sc; cin[x] = si; cout[x] = so;
        }
        double wi = 0, wo = 0;
        int wx = 0;
        for (int x = 4; x < SW - 4; x++) {
            double c9 = 0, i9 = 0, o9 = 0;
            for (int k = -4; k <= 4; k++) { c9 += clean[x + k]; i9 += cin[x + k]; o9 += cout[x + k]; }
            const double di = fabs(i9 / c9 - 1), dout = fabs(o9 / c9 - 1);
            if (di > wi) wi = di;
            if (dout > wo) { wo = dout; wx = x; }
        }
        printf("     denoise on a dense sky: worst 9-column deviation from the clean sky %.2f%% (input %.2f%%; at %d px from the right edge)\n",
               100 * wo, 100 * wi, SW - 1 - wx);
        if (r || !(wo < 0.003)) { printf("FAIL enhance-denoise-no-bands\n"); fail++; }
        else printf("ok   enhance-denoise-no-bands\n");
        dsd4_image_free(&out);
        dsd4_image_free(&sky);
    }

    /* digital gain scales the sensor noise: 2x gain, 2x the noise assumed */
    {
        dsd4_image g1 = copy(&in), g2 = copy(&in);
        for (int c = 0; c < 3; c++) g2.digital_gain[c] = 2;
        dsd4_enhance_info e1, e2;
        ep = (dsd4_enhance_params){ 0, DSD4_DENOISE_LOW, 0, NULL, &np, 1, 0, 1 };
        dsd4_enhance(&g1, &ep, &e1);
        dsd4_enhance(&g2, &ep, &e2);
        printf("     sensor noise assumed: gain 1 %.5f, gain 2 %.5f\n", e1.sensor_sigma, e2.sensor_sigma);
        if (!(fabs(e2.sensor_sigma / e1.sensor_sigma - 2) < 0.05)) { printf("FAIL enhance-gain-scales-noise\n"); fail++; }
        else printf("ok   enhance-gain-scales-noise\n");
        dsd4_image_free(&g1);
        dsd4_image_free(&g2);
    }

    /* nothing asked: untouched */
    ep = (dsd4_enhance_params){ 0, DSD4_DENOISE_OFF, 0, NULL, NULL, 1, 0 };
    a = copy(&in);
    r = dsd4_enhance(&a, &ep, &ei);
    if (r || memcmp(a.data, in.data, in.stride * H)) { printf("FAIL enhance-off-untouched\n"); fail++; }
    else printf("ok   enhance-off-untouched\n");
    dsd4_image_free(&a);

    /* 4000 dpi */
    ep = (dsd4_enhance_params){ 1, DSD4_DENOISE_LOW, 4000, &mtf, &np, 1, 0 };
    a = copy(&in);
    r = dsd4_enhance(&a, &ep, &ei);
    printf("     4000 dpi: %dx%d at %d dpi -> %dx%d at %d dpi\n", W, H, in.dpi, a.width, a.height, a.dpi);
    if (r || a.width != 1280 || a.height != 1280 || a.dpi != 4000) { printf("FAIL enhance-4000dpi\n"); fail++; }
    else printf("ok   enhance-4000dpi\n");
    dsd4_image_free(&a);
    dsd4_image_free(&in);
    return fail != 0;
}
