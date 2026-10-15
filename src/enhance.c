/*
 * Experimental "4000 dpi" processing of a raw scan: the scanner's own MTF
 * compensated, noise reduced with measured profiles, then resampled.
 *
 * After Norman Koren's film-scanner MTF studies: the scanner's aperture and
 * lens cost a 3200 dpi scan much of its MTF50; compensating it is the largest
 * gain, provided grain is protected (luminance only, no boost where noise
 * dominates, structure-weighted, no halos).  Noise reduction follows Neat
 * Image's model: a noise profile per detail scale and brightness, colour
 * reduced more than brightness; here from two measured parts, the Dual IV's
 * sensor (dsd4 research noise) and the film's grain, profiled from the
 * flattest areas of each scan.
 *
 * Works on the raw linear scan, where both apply: the scanner blurs the light
 * that passes the film, and its sensor noise adds to it.
 */
#include "dsd4_internal.h"
#include "imgops.h"

#include <Accelerate/Accelerate.h>
#include <dispatch/dispatch.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define N 256         /* FFT tile */
#define LOG2 8
#define PAD 32        /* context each side; tiles step N - 2 PAD */
#define GMAX 3.0      /* largest MTF boost */
#define BLK 128       /* grain profile blocks */
#define NBIN 6        /* grain profile brightness bins */
#define NBAND DSD4_NPS_BINS
#define LF0 (1.0 / 64) /* denoise: none below (structures over 64 px) */
#define LF1 (1.0 / 32) /* full from (32 px) */

typedef struct {
    float *p[3];              /* planes: Y, R-G, B-G (gray: Y) */
    int np, W, H;
    /* outputs per plane: denoised; Y also sharpened */
    float *dn[3], *dw[3], *sh, *shw;
    float win[N * N];         /* Hann window for the periodograms */
    double wsum;
    const dsd4_enhance_params *ep;
    const dsd4_image *img;
    float hx[N / 2 + 1], hy[N / 2 + 1];   /* scanner MTF at FFT frequencies */
    /* grain variance density per plane, brightness bin, band; bin centres */
    double grain[3][NBIN][NBAND], bin_y[NBIN];
    int nbins;
    double sens_k;            /* sensor variance factor: 1 / samples (gain: per channel) */
    FFTSetup fft;
    dispatch_semaphore_t lock;
    int failed;
    double gain_max;
} ctx_t;

static double interp_band(const double *v, double f)
{
    double x = f / 0.5 * (NBAND - 1);
    int i = (int)x;
    if (i >= NBAND - 1) return v[NBAND - 1];
    return v[i] + (x - i) * (v[i + 1] - v[i]);
}

/* sensor variance density of a plane, at a tile's raw channel means */
static double sensor_var(const ctx_t *c, int plane, const double m[3])
{
    const dsd4_noise_profile *np = c->ep->noise;
    if (!np || !np->valid || c->ep->hdr) return 0;
    /* the profile is of raw reads: a channel's digital gain scales its
     * signal, and its noise variance by the gain squared */
    double v[3];
    for (int k = 0; k < (c->np == 1 ? 1 : 3); k++) {
        const double g = c->img->digital_gain[k] > 0 ? c->img->digital_gain[k] : 1;
        v[k] = dsd4_noise_var(np, c->np == 1 ? 1 : k, m[k] / g) * g * g * c->sens_k;
    }
    if (c->np == 1) return v[0];
    return plane == 0 ? (v[0] + 4 * v[1] + v[2]) / 16 : plane == 1 ? v[0] + v[1] : v[2] + v[1];
}

static double sensor_shape(const ctx_t *c, double fx, double fy)
{
    const dsd4_noise_profile *np = c->ep->noise;
    if (!np || !np->valid) return 1;
    return interp_band(np->shape_x, fabs(fx)) * interp_band(np->shape_y, fabs(fy));
}

/* grain variance density of a plane at brightness y and radial frequency f */
static double grain_var(const ctx_t *c, int plane, double y, double f)
{
    if (!c->nbins) return 0;
    int b = 0;
    while (b < c->nbins - 1 && c->bin_y[b + 1] <= y) b++;
    if (b == c->nbins - 1 || y <= c->bin_y[0]) return interp_band(c->grain[plane][b], f);
    const double t = (y - c->bin_y[b]) / (c->bin_y[b + 1] - c->bin_y[b]);
    return (1 - t) * interp_band(c->grain[plane][b], f) + t * interp_band(c->grain[plane][b + 1], f);
}

static void fft2(FFTSetup fft, float *re, float *im, int dir)
{
    DSPSplitComplex z = { re, im };
    vDSP_fft2d_zip(fft, &z, 1, 0, LOG2, LOG2, dir);
}

/* windowed periodogram, as variance density (white noise of variance v: v) */
static void periodogram(const ctx_t *c, const float *t, float *re, float *im, float *P)
{
    const float *win = c->win;
    const double wsum = c->wsum;
    double m = 0;
    for (int i = 0; i < N * N; i++) m += t[i];
    m /= N * N;
    for (int i = 0; i < N * N; i++) { re[i] = (float)((t[i] - m) * win[i]); im[i] = 0; }
    fft2(c->fft, re, im, FFT_FORWARD);
    for (int i = 0; i < N * N; i++) P[i] = (float)((re[i] * re[i] + im[i] * im[i]) / wsum);
    /* smooth over neighbouring frequencies: a single periodogram is noisy */
    memcpy(re, P, sizeof(float) * N * N);
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            float a = 0;
            for (int j = -1; j <= 1; j++)
                for (int i = -1; i <= 1; i++) a += re[((y + j + N) % N) * N + (x + i + N) % N];
            P[y * N + x] = a / 9;
        }
}

static void tile_job(void *vc, size_t idx)
{
    ctx_t *c = vc;
    const int step = N - 2 * PAD, gw = (c->W + step - 1) / step;
    const int tx = (int)(idx % (size_t)gw), ty = (int)(idx / (size_t)gw);
    /* at most PAD of mirrored margin beyond an edge (the FFT's wrap-around
     * stays outside the image): the last tile pulled back inside, else it
     * would be mostly mirrored content and judge noise and detail
     * differently from its neighbour (a band where they meet) */
    int x0 = tx * step - PAD, y0 = ty * step - PAD;
    if (x0 + N > c->W + PAD) x0 = c->W + PAD - N;
    if (y0 + N > c->H + PAD) y0 = c->H + PAD - N;
    const int n = N * N;
    float *buf = malloc((size_t)7 * n * sizeof *buf);
    if (!buf) { c->failed = 1; return; }
    float *t = buf, *re = buf + n, *im = buf + 2 * n, *P = buf + 3 * n, *fr = buf + 4 * n, *fi = buf + 5 * n,
          *o = buf + 6 * n;
    const dsd4_enhance_params *ep = c->ep;
    /* the tile's raw channel means (sensor noise depends on the signal) */
    double m[3] = { 0, 0, 0 }, ym = 0;
    {
        int k = 0;
        for (int y = PAD; y < N - PAD; y += 4)
            for (int x = PAD; x < N - PAD; x += 4) {
                const int ix = io_mirror(x0 + x, c->W), iy = io_mirror(y0 + y, c->H);
                for (int ch = 0; ch < (c->np == 3 ? 3 : 1); ch++) m[ch] += io_sample(c->img, ix, iy, ch);
                ym += c->p[0][(size_t)iy * c->W + ix];
                k++;
            }
        for (int ch = 0; ch < 3; ch++) m[ch] /= k;
        ym /= k;
    }
    const double kg = ep->denoise == DSD4_DENOISE_HIGH ? 0.6 : 0;      /* grain counted as noise (Y) */
    const double kc = ep->denoise == DSD4_DENOISE_HIGH ? 1.3 : 0.65;   /* chroma: more (Neat: +30 %) */
    const double floor_y = ep->denoise == DSD4_DENOISE_HIGH ? 0.25 : 0.4, floor_c = 0.15;
    for (int pl = 0; pl < c->np; pl++) {
        const int sharpen = pl == 0 && ep->sharpen;
        if (!sharpen && ep->denoise == DSD4_DENOISE_OFF) continue;
        io_tile_load(c->p[pl], c->W, c->H, x0, y0, N, t);
        periodogram(c, t, re, im, P);
        memcpy(fr, t, sizeof(float) * n);
        memset(fi, 0, sizeof(float) * n);
        fft2(c->fft, fr, fi, FFT_FORWARD);
        const double vs = sensor_var(c, pl, m);
        float *out = o;
        /* gains per frequency: denoise (Wiener on the counted noise), then
         * for Y the MTF boost where the picture is above the sensor noise */
        for (int y = 0; y < N; y++)
            for (int x = 0; x < N; x++) {
                const int i = y * N + x;
                const double fx = (x < N / 2 ? x : x - N) / (double)N, fy = (y < N / 2 ? y : y - N) / (double)N;
                const double f = hypot(fx, fy);
                const double ns = vs * sensor_shape(c, fx, fy), ng = grain_var(c, pl, ym, f);
                double nd = 0, fl = 1;
                if (ep->denoise != DSD4_DENOISE_OFF) {
                    nd = pl == 0 ? ns + kg * ng : kc * (ns + ng);
                    fl = pl == 0 ? floor_y : floor_c;
                }
                const double p = P[i];
                double wd = 1;
                if (nd > 0 && x + y > 0) {
                    const double so = p - nd > 0 ? p - nd : 0;
                    wd = so / (so + nd);
                    wd = wd < fl ? fl : wd;
                    /* coarse detail is left alone: grain and sensor noise
                     * live at fine scales, and a coarse correction differing
                     * between neighbouring tiles shows as a band (film
                     * curves stretch dense skies) */
                    const double lf = f <= LF0 ? 0 : f >= LF1 ? 1 : (f - LF0) / (LF1 - LF0);
                    wd = 1 - lf * (1 - wd);
                }
                double g = wd;
                if (sharpen && x + y > 0) {
                    const int kx = abs(x < N / 2 ? x : x - N), ky = abs(y < N / 2 ? y : y - N);
                    double h = (double)c->hx[kx] * c->hy[ky];
                    double b = h > 1.0 / GMAX ? 1 / h : GMAX;
                    /* taper to no boost at Nyquist (aliasing; no MTF there) */
                    const double fm = fabs(fx) > fabs(fy) ? fabs(fx) : fabs(fy);
                    double tap = fm <= 0.35 ? 1 : fm >= 0.5 ? 0 : 0.5 + 0.5 * cos(M_PI * (fm - 0.35) / 0.15);
                    /* boost where the film's detail (grain included) is above the sensor noise */
                    const double sf = p - ns > 0 ? p - ns : 0, snr = ns > 0 ? sf / (sf + ns) : 1;
                    const double st = ep->strength > 0 ? ep->strength : 1;
                    b = 1 + (b - 1) * tap * snr * st;
                    g = wd * b;
                    if (b > c->gain_max) c->gain_max = b; /* racy max: informational */
                }
                re[i] = fr[i] * (float)wd;
                im[i] = fi[i] * (float)wd;
                if (sharpen) { P[i] = fr[i] * (float)g; t[i] = fi[i] * (float)g; }
            }
        fft2(c->fft, re, im, FFT_INVERSE);
        const float sc = 1.0f / n;
        vDSP_vsmul(re, 1, &sc, out, 1, (vDSP_Length)n);
        io_tile_blend(c->dn[pl], c->dw[pl], c->W, c->H, x0, y0, N, PAD, out, c->lock);
        if (sharpen) {
            fft2(c->fft, P, t, FFT_INVERSE);
            vDSP_vsmul(P, 1, &sc, out, 1, (vDSP_Length)n);
            io_tile_blend(c->sh, c->shw, c->W, c->H, x0, y0, N, PAD, out, c->lock);
        }
    }
    free(buf);
}

/* ------------------------------------------------- grain auto-profile */

/* Flattest blocks of the frame (little structure at large scale): their
 * spectrum is grain + sensor noise.  Per brightness bin and band, minus the
 * sensor part: the grain profile (Neat Image's auto-profile). */
static void grain_profile(ctx_t *c, double *flat_share)
{
    const int bw = c->W / BLK, bh = c->H / BLK, nb = bw * bh;
    *flat_share = 0;
    c->nbins = 0;
    if (nb < 4) return;
    double *flat = malloc((size_t)nb * sizeof *flat), *mean = malloc((size_t)nb * sizeof *mean);
    if (!flat || !mean) { free(flat); free(mean); return; }
    /* structure: variance of the block after a 4x4 box (grain averages
     * out, picture stays), relative to the block's brightness */
    for (int b = 0; b < nb; b++) {
        const int bx = b % bw * BLK, by = b / bw * BLK;
        double s = 0, s2 = 0, ms = 0;
        int k = 0;
        for (int y = by; y + 4 <= by + BLK; y += 4)
            for (int x = bx; x + 4 <= bx + BLK; x += 4) {
                double a = 0;
                for (int j = 0; j < 4; j++)
                    for (int i = 0; i < 4; i++) a += c->p[0][(size_t)(y + j) * c->W + x + i];
                a /= 16;
                s += a; s2 += a * a; k++;
            }
        ms = s / k;
        mean[b] = ms;
        flat[b] = ms > 0.01 && ms < 0.95 ? (s2 / k - ms * ms) / (ms * ms) : 1e9;
    }
    double *srt = malloc((size_t)nb * sizeof *srt);
    if (!srt) { free(flat); free(mean); return; }
    memcpy(srt, flat, (size_t)nb * sizeof *srt);
    for (int i = 1; i < nb; i++)
        for (int j = i; j > 0 && srt[j - 1] > srt[j]; j--) { double t = srt[j]; srt[j] = srt[j - 1]; srt[j - 1] = t; }
    const double thr = srt[nb / 5];
    free(srt);
    /* selected blocks by brightness: up to NBIN bins of equal count */
    int ns = 0;
    int *sel = malloc((size_t)nb * sizeof *sel);
    if (!sel) { free(flat); free(mean); return; }
    for (int b = 0; b < nb; b++)
        if (flat[b] <= thr && flat[b] < 1e8) sel[ns++] = b;
    if (ns < 4) { free(flat); free(mean); free(sel); return; }
    for (int i = 1; i < ns; i++)
        for (int j = i; j > 0 && mean[sel[j - 1]] > mean[sel[j]]; j--) { int t = sel[j]; sel[j] = sel[j - 1]; sel[j - 1] = t; }
    const int nbins = ns >= 4 * NBIN ? NBIN : ns / 4 > 0 ? ns / 4 : 1;
    /* 128-point FFT blocks with a Hann window, radial bands */
    FFTSetup f7 = vDSP_create_fftsetup(7, FFT_RADIX2);
    float *re = malloc(BLK * BLK * sizeof *re), *im = malloc(BLK * BLK * sizeof *im);
    if (!f7 || !re || !im) { if (f7) vDSP_destroy_fftsetup(f7); free(re); free(im); free(flat); free(mean); free(sel); return; }
    double ws = 0;
    for (int y = 0; y < BLK; y++)
        for (int x = 0; x < BLK; x++) {
            double w = (0.5 - 0.5 * cos(2 * M_PI * (x + 0.5) / BLK)) * (0.5 - 0.5 * cos(2 * M_PI * (y + 0.5) / BLK));
            ws += w * w;
        }
    for (int bi = 0; bi < nbins; bi++) {
        const int a = bi * ns / nbins, z = (bi + 1) * ns / nbins;
        double ysum = 0;
        double acc[3][NBAND] = { { 0 } }, cnt[NBAND] = { 0 };
        double mch[3] = { 0, 0, 0 };
        for (int q = a; q < z; q++) {
            const int b = sel[q], bx = b % bw * BLK, by = b / bw * BLK;
            ysum += mean[b];
            for (int ch = 0; ch < (c->np == 3 ? 3 : 1); ch++) {
                double s = 0;
                for (int y = by; y < by + BLK; y += 2)
                    for (int x = bx; x < bx + BLK; x += 2) s += io_sample(c->img, x, y, ch);
                mch[ch] += s / (BLK * BLK / 4);
            }
            for (int pl = 0; pl < c->np; pl++) {
                double m = 0;
                for (int y = 0; y < BLK; y++)
                    for (int x = 0; x < BLK; x++) m += c->p[pl][(size_t)(by + y) * c->W + bx + x];
                m /= BLK * BLK;
                for (int y = 0; y < BLK; y++)
                    for (int x = 0; x < BLK; x++) {
                        double w = (0.5 - 0.5 * cos(2 * M_PI * (x + 0.5) / BLK)) * (0.5 - 0.5 * cos(2 * M_PI * (y + 0.5) / BLK));
                        re[y * BLK + x] = (float)((c->p[pl][(size_t)(by + y) * c->W + bx + x] - m) * w);
                        im[y * BLK + x] = 0;
                    }
                DSPSplitComplex zc = { re, im };
                vDSP_fft2d_zip(f7, &zc, 1, 0, 7, 7, FFT_FORWARD);
                for (int y = 0; y < BLK; y++)
                    for (int x = 0; x < BLK; x++) {
                        const double fx = (x < BLK / 2 ? x : x - BLK) / (double)BLK, fy = (y < BLK / 2 ? y : y - BLK) / (double)BLK;
                        const double f = hypot(fx, fy);
                        const int band = (int)lround(f / 0.5 * (NBAND - 1));
                        if (band >= NBAND || x + y == 0) continue;
                        acc[pl][band] += (re[y * BLK + x] * re[y * BLK + x] + im[y * BLK + x] * im[y * BLK + x]) / ws;
                        if (pl == 0) cnt[band] += 1;
                    }
            }
        }
        const int nq = z - a;
        for (int ch = 0; ch < 3; ch++) mch[ch] /= nq;
        c->bin_y[bi] = ysum / nq;
        for (int pl = 0; pl < c->np; pl++) {
            const double vs = sensor_var(c, pl, mch);
            for (int k = 0; k < NBAND; k++) {
                double g = cnt[k] > 0 ? acc[pl][k] / cnt[k] : 0;
                g -= vs * (c->ep->noise && c->ep->noise->valid ? interp_band(c->ep->noise->shape_x, k * 0.5 / (NBAND - 1)) : 1);
                c->grain[pl][bi][k] = g > 0 ? g : 0;
            }
            /* the lowest bands carry picture, not grain: as the next ones */
            c->grain[pl][bi][0] = c->grain[pl][bi][1] = c->grain[pl][bi][2] = c->grain[pl][bi][3];
        }
    }
    c->nbins = nbins;
    *flat_share = (double)ns / nb;
    vDSP_destroy_fftsetup(f7);
    free(re);
    free(im);
    free(flat);
    free(mean);
    free(sel);
}

/* ---------------------------------------------------------- resampling */

static inline double lanczos3(double x)
{
    x = fabs(x);
    if (x < 1e-9) return 1;
    if (x >= 3) return 0;
    return 3 * sin(M_PI * x) * sin(M_PI * x / 3) / (M_PI * M_PI * x * x);
}

/* one plane, separable Lanczos-3, clamped to the range of the nearest
 * source pixels (no ringing) */
static int resample_plane(const float *in, int w, int h, float *out, int ow, int oh)
{
    float *t = malloc((size_t)ow * h * sizeof *t);
    if (!t) return -1;
    const double sx = (double)w / ow, sy = (double)h / oh;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < ow; x++) {
            const double c = (x + 0.5) * sx - 0.5;
            const int i0 = (int)floor(c);
            double a = 0, ws = 0;
            for (int i = i0 - 2; i <= i0 + 3; i++) {
                const double k = lanczos3(c - i);
                a += k * in[(size_t)y * w + io_mirror(i, w)];
                ws += k;
            }
            float v = (float)(a / ws), lo = in[(size_t)y * w + io_mirror(i0, w)], hi = in[(size_t)y * w + io_mirror(i0 + 1, w)];
            if (lo > hi) { float q = lo; lo = hi; hi = q; }
            t[(size_t)y * ow + x] = v < lo ? lo : v > hi ? hi : v;
        }
    for (int y = 0; y < oh; y++) {
        const double c = (y + 0.5) * sy - 0.5;
        const int j0 = (int)floor(c);
        for (int x = 0; x < ow; x++) {
            double a = 0, ws = 0;
            for (int j = j0 - 2; j <= j0 + 3; j++) {
                const double k = lanczos3(c - j);
                a += k * t[(size_t)io_mirror(j, h) * ow + x];
                ws += k;
            }
            float v = (float)(a / ws), lo = t[(size_t)io_mirror(j0, h) * ow + x], hi = t[(size_t)io_mirror(j0 + 1, h) * ow + x];
            if (lo > hi) { float q = lo; lo = hi; hi = q; }
            out[(size_t)y * ow + x] = v < lo ? lo : v > hi ? hi : v;
        }
    }
    free(t);
    return 0;
}

/* --------------------------------------------------------------- driver */

int dsd4_enhance(dsd4_image *img, const dsd4_enhance_params *ep, dsd4_enhance_info *info)
{
    if (info) memset(info, 0, sizeof *info);
    if (img->bits != 16 && img->bits != 32) return DSD4_ERR_PARAM;
    if (img->channels != 1 && img->channels != 3) return DSD4_ERR_PARAM;
    const int W = img->width, H = img->height, ch = img->channels;
    const int sharpen = ep->sharpen && ep->mtf && ep->mtf->valid && img->dpi == ep->mtf->dpi;
    const int resample = ep->target_dpi > img->dpi && img->dpi > 0;
    if (!sharpen && ep->denoise == DSD4_DENOISE_OFF && !resample) return DSD4_OK;
    if (W < N || H < N) return DSD4_OK;
    const size_t n = (size_t)W * H;
    ctx_t c;
    memset(&c, 0, sizeof c);
    c.np = ch == 3 ? 3 : 1;
    c.W = W;
    c.H = H;
    c.img = img;
    dsd4_enhance_params epx = *ep;
    epx.sharpen = sharpen;
    c.ep = &epx;
    const int samples = ep->samples > 1 ? ep->samples : 1;
    c.sens_k = 1.0 / samples;
    int r = DSD4_ERR_NOMEM;
    for (int pl = 0; pl < c.np; pl++) {
        c.p[pl] = malloc(n * sizeof(float));
        c.dn[pl] = calloc(n, sizeof(float));
        c.dw[pl] = calloc(n, sizeof(float));
        if (!c.p[pl] || !c.dn[pl] || !c.dw[pl]) goto out;
    }
    if (sharpen && (!(c.sh = calloc(n, sizeof(float))) || !(c.shw = calloc(n, sizeof(float))))) goto out;
    /* planes */
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            const size_t i = (size_t)y * W + x;
            if (ch == 1) { c.p[0][i] = io_sample(img, x, y, 0); continue; }
            const float R = io_sample(img, x, y, 0), G = io_sample(img, x, y, 1), B = io_sample(img, x, y, 2);
            c.p[0][i] = 0.25f * R + 0.5f * G + 0.25f * B;
            c.p[1][i] = R - G;
            c.p[2][i] = B - G;
        }
    /* scanner MTF at the FFT frequencies: green (luminance), x and y */
    if (sharpen)
        for (int k = 0; k <= N / 2; k++) {
            const double f = (double)k / N * (DSD4_MTF_BINS - 1);
            const int i = (int)f;
            const double t = f - i;
            const double *mx = ep->mtf->mtf[0][1], *my = ep->mtf->mtf[1][1];
            double vx = i + 1 < DSD4_MTF_BINS ? mx[i] + t * (mx[i + 1] - mx[i]) : mx[i];
            double vy = i + 1 < DSD4_MTF_BINS ? my[i] + t * (my[i + 1] - my[i]) : my[i];
            c.hx[k] = (float)(vx > 0.02 ? vx : 0.02);
            c.hy[k] = (float)(vy > 0.02 ? vy : 0.02);
        }
    c.wsum = 0;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++) {
            double w = (0.5 - 0.5 * cos(2 * M_PI * (x + 0.5) / N)) * (0.5 - 0.5 * cos(2 * M_PI * (y + 0.5) / N));
            c.win[y * N + x] = (float)w;
            c.wsum += w * w;
        }
    double flat = 0;
    if (ep->denoise != DSD4_DENOISE_OFF || sharpen) grain_profile(&c, &flat);

    if (sharpen || ep->denoise != DSD4_DENOISE_OFF) {
        c.fft = vDSP_create_fftsetup(LOG2, FFT_RADIX2);
        c.lock = dispatch_semaphore_create(1);
        if (!c.fft) goto out;
        const int step = N - 2 * PAD, gw = (W + step - 1) / step, gh = (H + step - 1) / step;
        dispatch_apply_f((size_t)gw * gh, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), &c, tile_job);
        vDSP_destroy_fftsetup(c.fft);
        dispatch_release(c.lock);
        if (c.failed) goto out;
    }
    /* planes from the blended tiles; sharpening on Y weighted by structure
     * and kept within the local range (Koren: threshold; no halos) */
    {
        const int dn_on = ep->denoise != DSD4_DENOISE_OFF;
        for (int pl = 0; pl < c.np; pl++) {
            if (!(dn_on || (pl == 0 && sharpen))) continue;
            for (size_t i = 0; i < n; i++)
                if (c.dw[pl][i] > 0) c.dn[pl][i] /= c.dw[pl][i];
                else c.dn[pl][i] = c.p[pl][i];
        }
        if (sharpen) {
            float *gm = malloc(n * sizeof *gm), *gs = malloc(n * sizeof *gs), *lo = malloc(n * sizeof *lo),
                  *hi = malloc(n * sizeof *hi);
            if (!gm || !gs || !lo || !hi) { free(gm); free(gs); free(lo); free(hi); goto out; }
            for (size_t i = 0; i < n; i++) c.sh[i] = c.shw[i] > 0 ? c.sh[i] / c.shw[i] : c.dn[0][i];
            if (io_gauss_blur(c.dn[0], gs, W, H, 1.5)) { free(gm); free(gs); free(lo); free(hi); goto out; }
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++) {
                    const int xl = x > 0 ? x - 1 : x, xr = x < W - 1 ? x + 1 : x, yu = y > 0 ? y - 1 : y, yd = y < H - 1 ? y + 1 : y;
                    const float dx = (gs[(size_t)y * W + xr] - gs[(size_t)y * W + xl]) / (float)(xr - xl);
                    const float dy = (gs[(size_t)yd * W + x] - gs[(size_t)yu * W + x]) / (float)(yd - yu);
                    gm[(size_t)y * W + x] = hypotf(dx, dy) / (gs[(size_t)y * W + x] + 0.01f);
                }
            if (io_gauss_blur(gm, gs, W, H, 2.0) || io_minmax_filter(c.dn[0], lo, W, H, 2, 0) ||
                io_minmax_filter(c.dn[0], hi, W, H, 2, 1)) {
                free(gm); free(gs); free(lo); free(hi);
                goto out;
            }
            float knee;
            {
                size_t m = n / 65536 + 1, k = 0;
                for (size_t i = 0; i < n; i += m) gm[k++] = gs[i];
                qsort(gm, k, sizeof *gm, io_cmp_f);
                knee = 2 * gm[k / 2];
            }
            for (size_t i = 0; i < n; i++) {
                float s = c.sh[i] < lo[i] ? lo[i] : c.sh[i] > hi[i] ? hi[i] : c.sh[i];
                const float g2 = gs[i] * gs[i], w = g2 / (g2 + knee * knee + 1e-12f);
                c.dn[0][i] += w * (s - c.dn[0][i]);
            }
            free(gm); free(gs); free(lo); free(hi);
        }
        /* back to channels */
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                const size_t i = (size_t)y * W + x;
                if (ch == 1) { if (dn_on || sharpen) io_put(img, x, y, 0, c.dn[0][i]); continue; }
                const float Y = (dn_on || sharpen) ? c.dn[0][i] : c.p[0][i];
                const float d1 = dn_on ? c.dn[1][i] : c.p[1][i], d2 = dn_on ? c.dn[2][i] : c.p[2][i];
                const float G = Y - 0.25f * (d1 + d2);
                io_put(img, x, y, 0, G + d1);
                io_put(img, x, y, 1, G);
                io_put(img, x, y, 2, G + d2);
            }
    }
    if (info) {
        info->sharpened = sharpen;
        info->gain_max = sharpen ? c.gain_max : 0;
        info->flat_share = flat;
        double m3[3];
        for (int k = 0; k < 3; k++) m3[k] = 0.18 * (img->digital_gain[k] > 0 ? img->digital_gain[k] : 1);
        info->sensor_sigma = sqrt(sensor_var(&c, 0, m3));
        info->grain_sigma = c.nbins ? sqrt(interp_band(c.grain[0][c.nbins / 2], 0.25)) : 0;
    }

    if (resample) {
        const int ow = (int)lround((double)W * ep->target_dpi / img->dpi), oh = (int)lround((double)H * ep->target_dpi / img->dpi);
        const size_t on = (size_t)ow * oh;
        const int bytes = img->bits / 8;
        uint8_t *nd = malloc(on * ch * bytes);
        float *pin = malloc(n * sizeof *pin), *pout = malloc(on * sizeof *pout);
        if (!nd || !pin || !pout) { free(nd); free(pin); free(pout); goto out; }
        dsd4_image o = *img;
        o.width = ow;
        o.height = oh;
        o.stride = (size_t)ow * ch * bytes;
        o.data = nd;
        for (int k = 0; k < ch; k++) {
            for (int y = 0; y < H; y++)
                for (int x = 0; x < W; x++) pin[(size_t)y * W + x] = io_sample(img, x, y, k);
            if (resample_plane(pin, W, H, pout, ow, oh)) { free(nd); free(pin); free(pout); goto out; }
            for (int y = 0; y < oh; y++)
                for (int x = 0; x < ow; x++) io_put(&o, x, y, k, pout[(size_t)y * ow + x]);
        }
        free(pin);
        free(pout);
        free(img->data);
        o.dpi = ep->target_dpi;
        *img = o;
        if (info) info->resampled = 1;
    }
    r = DSD4_OK;
out:
    for (int pl = 0; pl < 3; pl++) { free(c.p[pl]); free(c.dn[pl]); free(c.dw[pl]); }
    free(c.sh);
    free(c.shw);
    return r;
}

void dsd4_set_enhance(dsd4 *s, const dsd4_enhance_params *ep)
{
    s->enhance = ep;
    memset(&s->enhance_info, 0, sizeof s->enhance_info);
}

void dsd4_enhance_result(const dsd4 *s, dsd4_enhance_info *info) { *info = s->enhance_info; }
