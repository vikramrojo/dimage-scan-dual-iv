/*
 * Deblur: reduces camera blur (missed focus, slight shake) that is in the
 * photograph itself, not in the scan.  Built to look like a sharper lens
 * rather than an effect:
 *
 *  - brightness only: colour is left as shot (no colour noise or fringes)
 *  - film grain and dust are left out of the deconvolution and added back
 *    unchanged: only the grain-free picture underneath is sharpened
 *  - the blur is measured, per region of about 3 mm of film (focus varies
 *    across the frame), by the edge re-blur method of Zhuo & Sim (2011) on a
 *    dust-free half-resolution copy; sharp frames are left alone
 *  - the blur is modelled as a soft-edged disk (a defocused lens), removed by
 *    Richardson-Lucy deconvolution with total-variation damping against
 *    ringing (Dey et al. 2006), and only partly: 60 % (low) or 90 % (high)
 *
 * Deconvolution is done in linear light, where blur happens; gamma-encoded
 * and negative data are converted for it.
 */
#include "dsd4_internal.h"
#include "imgops.h"

#include <Accelerate/Accelerate.h>
#include <dispatch/dispatch.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TILE 384      /* sigma map cell and deblur tile, pixels */
#define PAD 64        /* context around a tile (FFT size TILE + 2 PAD) */
#define NFFT 512
#define LOG2N 9
#define GRAIN 1.5     /* grain / picture split: gaussian sigma, pixels */
#define SCANNER 1.0   /* the scanner's own blur, sigma in pixels: not removed */
#define MAX_SIGMA 8.0
#define MIN_SIGMA 1.5 /* less is hardly visible (and within the estimate's bias) */
#define ITER 20
#define TV 0.002

/* ------------------------------------------------------- blur estimate */

/* blur sigma (full-resolution pixels) per TILE cell; NAN where there are
 * no usable edges */
static int estimate(const float *P, int W, int H, float *map, int gw, int gh)
{
    const int w = W / 2, h = H / 2, n = w * h;
    float *q = malloc((size_t)n * sizeof *q), *m = malloc((size_t)n * sizeof *m);
    float *b = malloc((size_t)n * sizeof *b), *r = malloc((size_t)n * sizeof *r);
    if (!q || !m || !b || !r) { free(q); free(m); free(b); free(r); return -1; }
    /* half resolution (grain averages out), log (edges in shadows count as
     * much as in highlights), no dust */
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            const float *p = P + (size_t)2 * y * W + 2 * x;
            q[(size_t)y * w + x] = logf(0.25f * (p[0] + p[1] + p[W] + p[W + 1]) + 0.002f);
        }
    io_median5(q, m, w, h);
    const double g = 1.0, s0 = 2.0;
    int err = io_gauss_blur(m, b, w, h, g) || io_gauss_blur(b, r, w, h, s0);
    free(q);
    free(m);
    if (err) { free(b); free(r); return -1; }
    /* gradient magnitudes, in place: b -> |grad b|, r -> |grad r| */
    float *gb = malloc((size_t)n * sizeof *gb), *gr = malloc((size_t)n * sizeof *gr);
    if (!gb || !gr) { free(b); free(r); free(gb); free(gr); return -1; }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int xl = x > 0 ? x - 1 : x, xr = x < w - 1 ? x + 1 : x, yu = y > 0 ? y - 1 : y, yd = y < h - 1 ? y + 1 : y;
            size_t i = (size_t)y * w + x;
            float dx = (b[(size_t)y * w + xr] - b[(size_t)y * w + xl]) / (float)(xr - xl);
            float dy = (b[(size_t)yd * w + x] - b[(size_t)yu * w + x]) / (float)(yd - yu);
            gb[i] = hypotf(dx, dy);
            dx = (r[(size_t)y * w + xr] - r[(size_t)y * w + xl]) / (float)(xr - xl);
            dy = (r[(size_t)yd * w + x] - r[(size_t)yu * w + x]) / (float)(yd - yu);
            gr[i] = hypotf(dx, dy);
        }
    free(b);
    free(r);
    const int T = TILE / 2;
    float *v = malloc((size_t)T * T * sizeof *v);
    if (!v) { free(gb); free(gr); return -1; }
    for (int ty = 0; ty < gh; ty++)
        for (int tx = 0; tx < gw; tx++) {
            map[ty * gw + tx] = NAN;
            int x0 = tx * T, y0 = ty * T, x1 = x0 + T < w ? x0 + T : w, y1 = y0 + T < h ? y0 + T : h;
            int k = 0;
            for (int y = y0; y < y1; y++)
                for (int x = x0; x < x1; x++) v[k++] = gb[(size_t)y * w + x];
            if (k < T * T / 4) continue;
            qsort(v, (size_t)k, sizeof *v, io_cmp_f);
            const float thr = v[(int)(0.97 * (k - 1))];
            if (thr < 0.03f) continue; /* no real edges here */
            int nr = 0;
            for (int y = y0; y < y1; y++)
                for (int x = x0; x < x1; x++) {
                    size_t i = (size_t)y * w + x;
                    if (gb[i] >= thr && gr[i] > 0) {
                        float R = gb[i] / gr[i];
                        if (R > 1.0005f) v[nr++] = R;
                    }
                }
            if (nr < 50) continue;
            qsort(v, (size_t)nr, sizeof *v, io_cmp_f);
            double R = v[nr / 2], s = s0 / sqrt(R * R - 1);
            s = s * s - g * g;
            map[ty * gw + tx] = (float)(2 * sqrt(s > 0 ? s : 0)); /* to full resolution */
        }
    free(v);
    free(gb);
    free(gr);
    return 0;
}

/* -------------------------------------------------------- deconvolution */

typedef struct {
    const float *base;   /* grain-free picture, positive linear */
    float *acc, *wsum;   /* blended result */
    const float *sig;    /* per tile: deconvolution sigma (0: none) */
    int W, H, gw, gh;
    FFTSetup fft;
    dispatch_semaphore_t lock;
    int failed;
} job_t;

/* out = in (*) psf, through its spectrum H (real: the psf is symmetric) */
static void conv(FFTSetup fft, const float *in, float *out, const float *Hs, float *re, float *im)
{
    const int n = NFFT * NFFT;
    memcpy(re, in, (size_t)n * sizeof *re);
    memset(im, 0, (size_t)n * sizeof *im);
    DSPSplitComplex z = { re, im };
    vDSP_fft2d_zip(fft, &z, 1, 0, LOG2N, LOG2N, FFT_FORWARD);
    vDSP_vmul(re, 1, Hs, 1, re, 1, (vDSP_Length)n);
    vDSP_vmul(im, 1, Hs, 1, im, 1, (vDSP_Length)n);
    vDSP_fft2d_zip(fft, &z, 1, 0, LOG2N, LOG2N, FFT_INVERSE);
    const float sc = 1.0f / (float)n;
    vDSP_vsmul(re, 1, &sc, out, 1, (vDSP_Length)n);
}

static void tile_job(void *ctx, size_t idx)
{
    job_t *j = ctx;
    const int tx = (int)(idx % (size_t)j->gw), ty = (int)(idx / (size_t)j->gw);
    const double s = j->sig[idx];
    if (!(s > 0)) return; /* acc/wsum keep the base: nothing to add */
    const int n = NFFT * NFFT;
    float *buf = malloc((size_t)6 * n * sizeof *buf);
    if (!buf) { j->failed = 1; return; }
    float *img = buf, *est = buf + n, *t = buf + 2 * n, *re = buf + 3 * n, *im = buf + 4 * n, *Hs = buf + 5 * n;
    const int x0 = tx * TILE - PAD, y0 = ty * TILE - PAD;
    io_tile_load(j->base, j->W, j->H, x0, y0, NFFT, img);

    /* psf: a disk of radius rd with soft edges (gaussian rd * 0.3); sigma
     * of the whole: s.  Spectrum = disk spectrum x gaussian transfer. */
    const double rd = s / sqrt(0.25 + 0.09), soft = 0.3 * rd;
    double sum = 0;
    memset(re, 0, (size_t)n * sizeof *re);
    memset(im, 0, (size_t)n * sizeof *im);
    const int rr = (int)ceil(rd) + 1;
    for (int y = -rr; y <= rr; y++)
        for (int x = -rr; x <= rr; x++) {
            double v = rd + 0.5 - hypot(x, y);
            v = v < 0 ? 0 : v > 1 ? 1 : v;
            re[((y + NFFT) % NFFT) * NFFT + (x + NFFT) % NFFT] = (float)v;
            sum += v;
        }
    DSPSplitComplex z = { re, im };
    vDSP_fft2d_zip(j->fft, &z, 1, 0, LOG2N, LOG2N, FFT_FORWARD);
    for (int y = 0; y < NFFT; y++)
        for (int x = 0; x < NFFT; x++) {
            double fx = (x < NFFT / 2 ? x : x - NFFT) / (double)NFFT, fy = (y < NFFT / 2 ? y : y - NFFT) / (double)NFFT;
            Hs[y * NFFT + x] = (float)(re[y * NFFT + x] / sum * exp(-2 * M_PI * M_PI * soft * soft * (fx * fx + fy * fy)));
        }

    /* Richardson-Lucy with total-variation damping */
    float mx = 0;
    for (int i = 0; i < n; i++) mx = img[i] > mx ? img[i] : mx;
    const float lo = 1e-6f * (mx > 0 ? mx : 1), hi = 4 * mx;
    for (int i = 0; i < n; i++) est[i] = img[i] > lo ? img[i] : lo;
    for (int it = 0; it < ITER; it++) {
        conv(j->fft, est, t, Hs, re, im);
        for (int i = 0; i < n; i++) t[i] = img[i] / (t[i] > lo ? t[i] : lo);
        conv(j->fft, t, t, Hs, re, im);
        /* divergence of the normalised gradient, anisotropic: the sum of
         * the signs of the differences to the four neighbours */
        const float eps = 1e-3f * mx;
        for (int y = 0; y < NFFT; y++)
            for (int x = 0; x < NFFT; x++) {
                const int i = y * NFFT + x;
                float div = 0;
                for (int d = -1; d <= 1; d += 2) {
                    const int xn = x + d, yn = y + d;
                    if (xn >= 0 && xn < NFFT) {
                        const float g = est[y * NFFT + xn] - est[i];
                        div += g / (fabsf(g) + eps);
                    }
                    if (yn >= 0 && yn < NFFT) {
                        const float g = est[yn * NFFT + x] - est[i];
                        div += g / (fabsf(g) + eps);
                    }
                }
                float den = 1 - TV * div;
                den = den < 0.2f ? 0.2f : den;
                re[i] = est[i] * t[i] / den;
            }
        for (int i = 0; i < n; i++) est[i] = re[i] < lo ? lo : re[i] > hi ? hi : re[i];
    }

    io_tile_blend(j->acc, j->wsum, j->W, j->H, x0, y0, NFFT, PAD, est, j->lock);
    free(buf);
}

/* ------------------------------------------------------------- driver */

int dsd4_deblur(dsd4_image *img, int level, int tone, dsd4_deblur_info *info)
{
    if (info) memset(info, 0, sizeof *info);
    if (level == DSD4_DEBLUR_OFF) return DSD4_OK;
    if (level != DSD4_DEBLUR_LOW && level != DSD4_DEBLUR_HIGH) return DSD4_ERR_PARAM;
    if (tone < DSD4_TONE_LINEAR || tone > DSD4_TONE_NEGATIVE) return DSD4_ERR_PARAM;
    if (img->bits != 8 && img->bits != 16 && img->bits != 32) return DSD4_ERR_PARAM;
    if (img->channels != 1 && img->channels != 3) return DSD4_ERR_PARAM;
    const int W = img->width, H = img->height, ch = img->channels;
    if (W < TILE / 2 || H < TILE / 2) return DSD4_OK; /* too small to measure */
    const size_t N = (size_t)W * H;
    const int gw = (W + TILE - 1) / TILE, gh = (H + TILE - 1) / TILE;
    float *P = malloc(N * sizeof *P), *base = malloc(N * sizeof *base);
    float *acc = calloc(N, sizeof *acc), *wsum = calloc(N, sizeof *wsum);
    float *map = malloc((size_t)gw * gh * sizeof *map), *sm = malloc((size_t)gw * gh * sizeof *sm);
    int r = DSD4_ERR_NOMEM;
    if (!P || !base || !acc || !wsum || !map || !sm) goto out;

    /* brightness in positive linear light.  Negatives: the film's density
     * follows log exposure (gamma about 0.6), so scene light ~ T^(-1/0.6) */
    static const float wl[3] = { 0.2126f, 0.7152f, 0.0722f };
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float Y = 0;
            for (int c = 0; c < ch; c++) {
                float v = io_sample(img, x, y, c);
                if (tone == DSD4_TONE_GAMMA22) v = powf(v > 0 ? v : 0, 2.2f);
                Y += (ch == 3 ? wl[c] : 1) * v;
            }
            if (tone == DSD4_TONE_NEGATIVE) Y = powf(Y > 1e-4f ? Y : 1e-4f, -1 / 0.6f);
            P[(size_t)y * W + x] = Y > 0 ? Y : 0;
        }
    /* scale: median 0.18 (the estimate's log offset assumes it) */
    {
        size_t m = N / 4096 + 1, k = 0;
        float *v = malloc((N / m + 1) * sizeof *v);
        if (!v) goto out;
        for (size_t i = 0; i < N; i += m) v[k++] = P[i];
        qsort(v, k, sizeof *v, io_cmp_f);
        float med = v[k / 2];
        free(v);
        if (med > 0)
            for (size_t i = 0; i < N; i++) P[i] *= 0.18f / med;
    }

    if (estimate(P, W, H, map, gw, gh)) goto out;
    /* blur beyond the scanner's own; the frame's typical value fills cells
     * without edges, and limits outliers */
    float *v = malloc((size_t)gw * gh * sizeof *v);
    if (!v) goto out;
    int nv = 0;
    for (int i = 0; i < gw * gh; i++) {
        if (isnan(map[i])) continue;
        double e = (double)map[i] * map[i] - SCANNER * SCANNER;
        map[i] = (float)sqrt(e > 0 ? e : 0);
        v[nv++] = map[i];
    }
    float med = 0, mn = 0, mxs = 0;
    if (nv) {
        qsort(v, (size_t)nv, sizeof *v, io_cmp_f);
        med = v[nv / 2];
    }
    free(v);
    if (info) { info->sigma = med; info->measured = nv; info->tiles = gw * gh; }
    r = DSD4_OK;
    if (nv < 3 || med < MIN_SIGMA) goto out; /* sharp enough, or nothing to measure */
    for (int i = 0; i < gw * gh; i++) {
        float s = isnan(map[i]) ? med : map[i];
        map[i] = s < 0.6f * med ? 0.6f * med : s > 1.6f * med ? 1.6f * med : s;
    }
    mn = 1e9f;
    for (int ty = 0; ty < gh; ty++)
        for (int tx = 0; tx < gw; tx++) {
            float a = 0;
            int k = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int yy = ty + dy, xx = tx + dx;
                    if (yy >= 0 && yy < gh && xx >= 0 && xx < gw) { a += map[yy * gw + xx]; k++; }
                }
            float s = a / k * (level == DSD4_DEBLUR_LOW ? 0.85f : 1.0f);
            s = s > MAX_SIGMA ? MAX_SIGMA : s;
            mn = s < mn ? s : mn;
            mxs = s > mxs ? s : mxs;
            /* the base already carries the grain split's gaussian */
            sm[ty * gw + tx] = (float)hypot(s, GRAIN);
        }
    if (info) { info->sigma_min = mn; info->sigma_max = mxs; }

    /* the picture to sharpen: no dust, no grain.  Nothing in it is finer
     * than the blur, so what the median and the gaussian take out is film
     * (grain, dust, hairs); it is added back unchanged */
    r = DSD4_ERR_NOMEM;
    io_median5(P, acc, W, H);
    if (io_gauss_blur(acc, base, W, H, GRAIN)) goto out;
    memset(acc, 0, N * sizeof *acc);
    job_t j = { base, acc, wsum, sm, W, H, gw, gh, vDSP_create_fftsetup(LOG2N, FFT_RADIX2),
                dispatch_semaphore_create(1), 0 };
    if (!j.fft) goto out;
    dispatch_apply_f((size_t)gw * gh, dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_DEFAULT, 0), &j, tile_job);
    vDSP_destroy_fftsetup(j.fft);
    dispatch_release(j.lock);
    if (j.failed) goto out;

    /* deringing: the sharpened picture stays within the range the blurred
     * one has nearby.  Edges get steeper, but nothing overshoots into a halo
     * (dust specks larger than the median would otherwise get dark rings) */
    const int rr = (int)ceil(1.5 * med);
    float *lo = malloc(N * sizeof *lo), *hi = malloc(N * sizeof *hi);
    if (!lo || !hi || io_minmax_filter(base, lo, W, H, rr, 0) || io_minmax_filter(base, hi, W, H, rr, 1)) {
        free(lo);
        free(hi);
        goto out;
    }

    /* structure weight: sharpen edges and texture, not smooth areas (skin,
     * sky), where it can only raise blotches from grain clumps.  Local
     * gradient of the picture against the frame's typical (mostly flat) one */
    float *gm = hi ? malloc(N * sizeof *gm) : NULL, *gs = gm ? malloc(N * sizeof *gs) : NULL;
    if (!gs) { free(gm); free(lo); free(hi); goto out; }
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            const size_t i = (size_t)y * W + x;
            const int xl = x > 0 ? x - 1 : x, xr = x < W - 1 ? x + 1 : x, yu = y > 0 ? y - 1 : y, yd = y < H - 1 ? y + 1 : y;
            const float dx = (base[(size_t)y * W + xr] - base[(size_t)y * W + xl]) / (float)(xr - xl);
            const float dy = (base[(size_t)yd * W + x] - base[(size_t)yu * W + x]) / (float)(yd - yu);
            /* relative: the same in shadows and highlights */
            gm[i] = hypotf(dx, dy) / (base[i] + 0.01f);
        }
    if (io_gauss_blur(gm, gs, W, H, 2.0)) { free(gm); free(gs); free(lo); free(hi); goto out; }
    float knee;
    {
        size_t m = N / 65536 + 1, k = 0;
        for (size_t i = 0; i < N; i += m) gm[k++] = gs[i];
        qsort(gm, k, sizeof *gm, io_cmp_f);
        knee = 2 * gm[k / 2];
    }
    free(gm);

    /* picture + part of the sharpening; grain and dust ride along unchanged */
    const float strength = level == DSD4_DEBLUR_LOW ? 0.6f : 0.9f;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            const size_t i = (size_t)y * W + x;
            if (!(wsum[i] > 0) || !(P[i] > 0)) continue;
            float sharp = acc[i] / wsum[i];
            sharp = sharp < lo[i] ? lo[i] : sharp > hi[i] ? hi[i] : sharp;
            const float g2 = gs[i] * gs[i], wgt = g2 / (g2 + knee * knee + 1e-12f);
            float np = P[i] + strength * wgt * (sharp - base[i]);
            float k = np / P[i];
            k = k < 0.2f ? 0.2f : k > 5 ? 5 : k;
            /* back to the image's encoding */
            float f = tone == DSD4_TONE_GAMMA22 ? powf(k, 1 / 2.2f) : tone == DSD4_TONE_NEGATIVE ? powf(k, -0.6f) : k;
            for (int c = 0; c < ch; c++) io_put(img, x, y, c, io_sample(img, x, y, c) * f);
        }
    free(lo);
    free(hi);
    free(gs);
    if (info) info->applied = 1;
    r = DSD4_OK;
out:
    free(P);
    free(base);
    free(acc);
    free(wsum);
    free(map);
    free(sm);
    return r;
}
