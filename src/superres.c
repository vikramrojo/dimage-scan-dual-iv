/*
 * Sub-pixel registration between scan passes: FFT helpers, phase
 * correlation and a Lucas-Kanade refinement on image tiles, and a fit of
 * the displacement model dx = a + m*(x - xc), dy = b + m*(y - yc).
 * Used by "dsd4 research" to measure what multi-pass super-resolution
 * could gain on this scanner.
 */
#include "dsd4_internal.h"
#include "superres.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* in-place radix-2 complex FFT, n a power of two; inverse when inv != 0
 * (unscaled) */
void sr_fft(double *re, double *im, int n, int inv)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        double a = (inv ? 2 : -2) * M_PI / len;
        double wr = cos(a), wi = sin(a);
        for (int i = 0; i < n; i += len) {
            double cr = 1, ci = 0;
            for (int k = 0; k < len / 2; k++) {
                int u = i + k, v = i + k + len / 2;
                double xr = re[v] * cr - im[v] * ci, xi = re[v] * ci + im[v] * cr;
                re[v] = re[u] - xr; im[v] = im[u] - xi;
                re[u] += xr; im[u] += xi;
                double t = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = t;
            }
        }
    }
}

static void fft2(double *re, double *im, int n, int inv)
{
    double *tr = malloc(sizeof(double) * n * 2), *ti = tr + n;
    for (int y = 0; y < n; y++) sr_fft(re + y * n, im + y * n, n, inv);
    for (int x = 0; x < n; x++) {
        for (int y = 0; y < n; y++) { tr[y] = re[y * n + x]; ti[y] = im[y * n + x]; }
        sr_fft(tr, ti, n, inv);
        for (int y = 0; y < n; y++) { re[y * n + x] = tr[y]; im[y * n + x] = ti[y]; }
    }
    free(tr);
}

static double px(const sr_plane *p, double x, double y)
{
    int x0 = (int)floor(x), y0 = (int)floor(y);
    double fx = x - x0, fy = y - y0;
    if (x0 < 0) { x0 = 0; fx = 0; }
    if (y0 < 0) { y0 = 0; fy = 0; }
    if (x0 >= p->w - 1) { x0 = p->w - 2; fx = 1; }
    if (y0 >= p->h - 1) { y0 = p->h - 2; fy = 1; }
    const float *r0 = p->v + (size_t)y0 * p->w, *r1 = r0 + p->w;
    return (1 - fy) * ((1 - fx) * r0[x0] + fx * r0[x0 + 1]) + fy * ((1 - fx) * r1[x0] + fx * r1[x0 + 1]);
}

/* Displacement of tile (tx, ty, size n) of b relative to a: b(x + dx, y + dy)
 * matches a(x, y).  Phase correlation for the integer part, then
 * Lucas-Kanade iterations on the bilinear-interpolated tile.  Returns the
 * tile's texture (sum of squared gradients per pixel); 0 when unusable. */
double sr_tile_shift(const sr_plane *a, const sr_plane *b, int tx, int ty, int n, double *dx, double *dy)
{
    *dx = *dy = 0;
    if (tx < 2 || ty < 2 || tx + n + 2 > a->w || ty + n + 2 > a->h) return 0;
    double *ar = calloc((size_t)n * n * 4, sizeof(double)), *ai = ar + n * n, *br = ai + n * n, *bi = br + n * n;
    double ma = 0, mb = 0;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) {
            ma += a->v[(size_t)(ty + y) * a->w + tx + x];
            mb += b->v[(size_t)(ty + y) * b->w + tx + x];
        }
    ma /= n * n;
    mb /= n * n;
    for (int y = 0; y < n; y++)
        for (int x = 0; x < n; x++) {
            double w = (0.5 - 0.5 * cos(2 * M_PI * (x + 0.5) / n)) * (0.5 - 0.5 * cos(2 * M_PI * (y + 0.5) / n));
            ar[y * n + x] = (a->v[(size_t)(ty + y) * a->w + tx + x] - ma) * w;
            br[y * n + x] = (b->v[(size_t)(ty + y) * b->w + tx + x] - mb) * w;
        }
    fft2(ar, ai, n, 0);
    fft2(br, bi, n, 0);
    for (int i = 0; i < n * n; i++) {
        /* cross power spectrum B * conj(A), normalised */
        double r = br[i] * ar[i] + bi[i] * ai[i], im = bi[i] * ar[i] - br[i] * ai[i];
        double m = sqrt(r * r + im * im) + 1e-20;
        ar[i] = r / m;
        ai[i] = im / m;
    }
    fft2(ar, ai, n, 1);
    int best = 0;
    for (int i = 1; i < n * n; i++) if (ar[i] > ar[best]) best = i;
    int ix = best % n, iy = best / n;
    if (ix > n / 2) ix -= n;
    if (iy > n / 2) iy -= n;
    free(ar);
    /* Lucas-Kanade refinement */
    double ux = ix, uy = iy, tex = 0;
    for (int it = 0; it < 8; it++) {
        double gxx = 0, gxy = 0, gyy = 0, bx = 0, by = 0;
        for (int y = 2; y < n - 2; y++)
            for (int x = 2; x < n - 2; x++) {
                int X = tx + x, Y = ty + y;
                double gx = 0.5 * (a->v[(size_t)Y * a->w + X + 1] - a->v[(size_t)Y * a->w + X - 1]);
                double gy = 0.5 * (a->v[(size_t)(Y + 1) * a->w + X] - a->v[(size_t)(Y - 1) * a->w + X]);
                double e = px(b, X + ux, Y + uy) - mb - (a->v[(size_t)Y * a->w + X] - ma);
                gxx += gx * gx; gxy += gx * gy; gyy += gy * gy;
                bx += gx * e; by += gy * e;
            }
        double det = gxx * gyy - gxy * gxy;
        if (det <= 1e-30) return 0;
        double sx = -(gyy * bx - gxy * by) / det, sy = -(gxx * by - gxy * bx) / det;
        ux += sx;
        uy += sy;
        tex = (gxx + gyy) / ((n - 4) * (n - 4));
        if (fabs(sx) < 1e-4 && fabs(sy) < 1e-4) break;
        if (fabs(ux - ix) > 2 || fabs(uy - iy) > 2) return 0; /* diverged */
    }
    *dx = ux;
    *dy = uy;
    return tex;
}

/* least-squares line v = c0 + c1*u, weights w; returns 0 on success */
int sr_fit_line(const double *u, const double *v, const double *w, int n, double *c0, double *c1, double *rms)
{
    double sw = 0, su = 0, sv = 0, suu = 0, suv = 0;
    for (int i = 0; i < n; i++) {
        sw += w[i]; su += w[i] * u[i]; sv += w[i] * v[i];
        suu += w[i] * u[i] * u[i]; suv += w[i] * u[i] * v[i];
    }
    double det = sw * suu - su * su;
    if (sw <= 0 || fabs(det) < 1e-20) return -1;
    *c1 = (sw * suv - su * sv) / det;
    *c0 = (sv - *c1 * su) / sw;
    double e = 0;
    for (int i = 0; i < n; i++) {
        double d = v[i] - (*c0 + *c1 * u[i]);
        e += w[i] * d * d;
    }
    if (rms) *rms = sqrt(e / sw);
    return 0;
}

/* green (or the only) channel of a float image as a plane */
int sr_plane_from_image(const dsd4_image *im, sr_plane *p)
{
    if (im->bits != 32) return -1;
    p->w = im->width;
    p->h = im->height;
    p->v = malloc((size_t)p->w * p->h * sizeof *p->v);
    if (!p->v) return -1;
    const float *s = (const float *)im->data;
    const int ch = im->channels, c = ch == 3 ? 1 : 0;
    for (size_t i = 0; i < (size_t)p->w * p->h; i++) p->v[i] = s[i * ch + c];
    return 0;
}

/* separable Gaussian blur in place.  Registration runs on low-passed copies:
 * detail above Nyquist aliases differently at each sample phase, so it does
 * not move with the image and would bias the measured shift. */
void sr_blur(sr_plane *p, double sigma)
{
    const int r = (int)ceil(3 * sigma);
    double k[64];
    if (r >= 32) return;
    double s = 0;
    for (int i = -r; i <= r; i++) s += k[i + r] = exp(-0.5 * i * i / (sigma * sigma));
    for (int i = 0; i <= 2 * r; i++) k[i] /= s;
    const int w = p->w, h = p->h;
    float *t = malloc(sizeof(float) * (size_t)(w > h ? w : h));
    if (!t) return;
    for (int y = 0; y < h; y++) {
        float *row = p->v + (size_t)y * w;
        for (int x = 0; x < w; x++) {
            double a = 0;
            for (int i = -r; i <= r; i++) { int xx = x + i < 0 ? 0 : x + i >= w ? w - 1 : x + i; a += k[i + r] * row[xx]; }
            t[x] = (float)a;
        }
        memcpy(row, t, sizeof(float) * w);
    }
    for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
            double a = 0;
            for (int i = -r; i <= r; i++) { int yy = y + i < 0 ? 0 : y + i >= h ? h - 1 : y + i; a += k[i + r] * p->v[(size_t)yy * w + x]; }
            t[y] = (float)a;
        }
        for (int y = 0; y < h; y++) p->v[(size_t)y * w + x] = t[y];
    }
    free(t);
}

/* mean |Laplacian| / mean level: sharpness that does not depend on exposure */
double sr_sharpness(const sr_plane *p)
{
    double s = 0, l = 0;
    for (int y = 1; y < p->h - 1; y++)
        for (int x = 1; x < p->w - 1; x++) {
            const float *c = p->v + (size_t)y * p->w + x;
            s += fabs(4 * c[0] - c[-1] - c[1] - c[-p->w] - c[p->w]);
            l += c[0];
        }
    return l > 0 ? s / l : 0;
}
