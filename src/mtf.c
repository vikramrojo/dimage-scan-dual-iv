/*
 * The scanner's own MTF, measured by the slanted-edge method (ISO 12233,
 * after Burns' sfrmat): a razor blade in a slide mount, its edges turned a
 * few degrees from the axes.  Along a slanted edge every pixel sits at a
 * different sub-pixel distance from it, so binning the pixels by distance
 * gives the edge profile (ESF) four times finer than the pixels; its
 * derivative is the line spread, whose Fourier transform is the MTF.
 *
 * The near-vertical edge gives the MTF across (x, along the CCD), the
 * near-horizontal edge the MTF along the carriage (y).  Both include the
 * pixel aperture, the lens and focus: everything the scanner does to film.
 */
#include "dsd4_internal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS 4          /* ESF bins per pixel */
#define HALF 16       /* ESF half-width, pixels (32 px window) */
#define NB (2 * HALF * OS)
#define MIN_DEG 2.0   /* slant needed: mount edges are straight but square */
#define MAX_DEG 20.0
#define MIN_RUN 256   /* rows of straight edge needed (a notch is not enough) */

static inline float px(const dsd4_image *im, int x, int y, int c)
{
    const uint8_t *row = im->data + (size_t)y * im->stride;
    const size_t i = (size_t)x * im->channels + (im->channels == 3 ? c : 0);
    return im->bits == 8 ? row[i] / 255.0f : im->bits == 16 ? ((const uint16_t *)row)[i] / 65535.0f
                                                           : ((const float *)row)[i];
}

/* green (or gray), transposed for the near-horizontal edge */
static inline float at(const dsd4_image *im, int t, int u, int v, int c)
{
    return t ? px(im, v, u, c) : px(im, u, v, c);
}

typedef struct { float u, v; } pt_t;

static uint64_t g_rng = 0x2545f4914f6cdd1dull;
static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 7; g_rng ^= g_rng << 17;
    return (uint32_t)(g_rng >> 32);
}

/* One edge direction.  t = 0: near-vertical edge, positions u = x along rows
 * v = y; t = 1: near-horizontal, u = y along columns v = x.  Finds the
 * longest straight slanted edge, fits u = a + b v, and returns its MTF per
 * channel (0 .. 1 cycles/pixel in DSD4_MTF_BINS steps) and the v range used. */
static double g_cu, g_cv; /* centre of the last edge measured (u, v) */

static int measure_dir(const dsd4_image *im, int t, double mtf[3][DSD4_MTF_BINS], int *v0, int *v1,
                       double *slant_deg, int seg, int nseg)
{
    const int U = t ? im->height : im->width, V = t ? im->width : im->height;
    const int g = im->channels == 3 ? 1 : 0;
    /* 1. edge points: local maxima of the gradient across the edge, much
     * stronger than along it; sub-pixel by the centroid of three */
    float gmax = 0;
    for (int v = 1; v < V - 1; v += 4)
        for (int u = 1; u < U - 1; u++) {
            float d = fabsf(at(im, t, u + 1, v, g) - at(im, t, u - 1, v, g));
            gmax = d > gmax ? d : gmax;
        }
    if (!(gmax > 0.02f)) return -1;
    const float thr = 0.25f * gmax;
    size_t cap = 1 << 16, n = 0;
    pt_t *p = malloc(cap * sizeof *p);
    if (!p) return DSD4_ERR_NOMEM;
    for (int v = 2; v < V - 2; v++)
        for (int u = 2; u < U - 2; u++) {
            float d0 = fabsf(at(im, t, u, v, g) - at(im, t, u - 1, v, g));
            float d1 = fabsf(at(im, t, u + 1, v, g) - at(im, t, u, v, g));
            float dv = fabsf(at(im, t, u, v + 1, g) - at(im, t, u, v - 1, g)) * 0.5f;
            if (d1 < thr || d1 < d0 || d1 < fabsf(at(im, t, u + 2, v, g) - at(im, t, u + 1, v, g)) || dv > 0.5f * d1)
                continue;
            /* centroid of the derivative around the step u..u+1 */
            float a = d0, b = d1, c = fabsf(at(im, t, u + 2, v, g) - at(im, t, u + 1, v, g));
            float pos = u + 0.5f + (c - a) / (2 * (a + b + c) + 1e-9f);
            if (n == cap) {
                pt_t *q = realloc(p, (cap *= 2) * sizeof *p);
                if (!q) { free(p); return DSD4_ERR_NOMEM; }
                p = q;
            }
            p[n++] = (pt_t){ pos, (float)v };
        }
    if (n < 64) { free(p); return -1; }
    /* 2. RANSAC: the line with the most points within 1 px, slant 2..20 deg */
    const double bmin = tan(MIN_DEG * M_PI / 180), bmax = tan(MAX_DEG * M_PI / 180);
    double ba = 0, bb = 0;
    size_t best = 0;
    for (int it = 0; it < 4000; it++) {
        const pt_t *P = &p[rnd() % n], *Q = &p[rnd() % n];
        if (fabsf(P->v - Q->v) < 40) continue;
        double b = (Q->u - P->u) / (double)(Q->v - P->v), a = P->u - b * P->v;
        if (fabs(b) < bmin || fabs(b) > bmax) continue;
        size_t k = 0;
        for (size_t i = 0; i < n; i += 3) k += fabs(p[i].u - (a + b * p[i].v)) < 1.0;
        if (k > best) { best = k; ba = a; bb = b; }
    }
    if (best < 20) { free(p); return -1; }
    /* a real edge over thousands of pixels is not straight to a fraction of
     * a pixel (the blade bows, the lens distorts a little): least squares
     * fit of a quadratic u = c0 + c1 w + c2 w^2 (w = v - vm), the inlier
     * band narrowing from 3 to 0.75 px (as sfrmat4's polynomial edge fit) */
    double vm = 0;
    {
        long k = 0;
        for (size_t i = 0; i < n; i++)
            if (fabs(p[i].u - (ba + bb * p[i].v)) < 1.0) { vm += p[i].v; k++; }
        vm = k ? vm / k : V / 2.0;
    }
    double q0 = ba + bb * vm, q1 = bb, q2 = 0;
    static const double tol[4] = { 3.0, 2.0, 1.0, 0.75 };
    for (int pass = 0; pass < 4; pass++) {
        double S[5] = { 0 }, T[3] = { 0 };
        long k = 0;
        for (size_t i = 0; i < n; i++) {
            const double w = p[i].v - vm;
            if (fabs(p[i].u - (q0 + q1 * w + q2 * w * w)) >= tol[pass]) continue;
            double wp = 1;
            for (int j = 0; j < 5; j++) { S[j] += wp; if (j < 3) T[j] += wp * p[i].u; wp *= w; }
            k++;
        }
        if (k < 20) { free(p); return -1; }
        /* normal equations, 3x3, Cramer */
        const double m[9] = { S[0], S[1], S[2], S[1], S[2], S[3], S[2], S[3], S[4] };
        const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
        if (fabs(det) < 1e-12) { free(p); return -1; }
        double sol[3];
        for (int c = 0; c < 3; c++) {
            double mm[9];
            memcpy(mm, m, sizeof mm);
            for (int rr = 0; rr < 3; rr++) mm[rr * 3 + c] = T[rr];
            sol[c] = (mm[0] * (mm[4] * mm[8] - mm[5] * mm[7]) - mm[1] * (mm[3] * mm[8] - mm[5] * mm[6]) +
                      mm[2] * (mm[3] * mm[7] - mm[4] * mm[6])) / det;
        }
        q0 = sol[0]; q1 = sol[1]; q2 = sol[2];
    }
#define EDGE_U(v) (q0 + q1 * ((v) - vm) + q2 * ((v) - vm) * ((v) - vm))
#define EDGE_SLOPE(v) (q1 + 2 * q2 * ((v) - vm))
    unsigned char *row = calloc((size_t)V, 1);
    if (!row) { free(p); return DSD4_ERR_NOMEM; }
    for (size_t i = 0; i < n; i++)
        if (fabs(p[i].u - EDGE_U(p[i].v)) < 0.75) row[(int)p[i].v] = 1;
    free(p);
    int rs = 0, rl = 0, cs = -1, gap = 0;
    for (int v = 0; v < V; v++) {
        if (row[v]) {
            if (cs < 0) cs = v;
            gap = 0;
            if (v - cs + 1 > rl) { rl = v - cs + 1; rs = cs; }
        } else if (cs >= 0 && ++gap > 4) { cs = -1; gap = 0; }
    }
    free(row);
    /* away from the run's ends (the blade's corner, the frame's edge) */
    int a0 = rs + 24, a1 = rs + rl - 24;
    if (a1 - a0 < MIN_RUN) return -1; /* too short: a notch, not an edge */
    if (nseg > 1) { /* one of nseg equal parts: field variation */
        int L = (a1 - a0) / nseg;
        a0 += seg * L;
        a1 = a0 + L;
    }
    *v0 = a0;
    *v1 = a1;
    g_cv = (a0 + a1) / 2.0;
    g_cu = EDGE_U(g_cv);
    *slant_deg = atan(EDGE_SLOPE((a0 + a1) / 2.0)) * 180 / M_PI;
    {   /* the slant has to hold along the part used */
        const double s0 = fabs(EDGE_SLOPE(a0)), s1 = fabs(EDGE_SLOPE(a1));
        if (s0 < bmin || s1 < bmin || s0 > bmax || s1 > bmax) return -1;
    }

    /* 3. ESF per channel: pixels binned by their distance to the edge
     * (along u, then perpendicular: times cos) */
    const int ch = im->channels == 3 ? 3 : 1;
    for (int c = 0; c < ch; c++) {
        double esf[NB] = { 0 }, cnt[NB] = { 0 };
        for (int v = a0; v < a1; v++) {
            const double e = EDGE_U(v), cs_ = cos(atan(EDGE_SLOPE(v)));
            for (int u = (int)floor(e) - HALF - 1; u <= (int)ceil(e) + HALF + 1; u++) {
                if (u < 0 || u >= U) continue;
                double d = (u - e) * cs_;
                int b = (int)floor((d + HALF) * OS);
                if (b < 0 || b >= NB) continue;
                esf[b] += at(im, t, u, v, c);
                cnt[b] += 1;
            }
        }
        for (int b = 0; b < NB; b++) {
            if (cnt[b] > 0) { esf[b] /= cnt[b]; continue; }
            /* empty bin (steep slant): from its neighbours */
            int l = b - 1, r = b + 1;
            while (l >= 0 && cnt[l] == 0) l--;
            while (r < NB && cnt[r] == 0) r++;
            esf[b] = l >= 0 && r < NB ? 0.5 * (esf[l] + esf[r]) : l >= 0 ? esf[l] : esf[r];
        }
        /* 4. LSF: derivative (central difference), Hamming window centred
         * on its peak, positive polarity */
        double lsf[NB];
        for (int b = 0; b < NB; b++) {
            int l = b > 0 ? b - 1 : b, r = b < NB - 1 ? b + 1 : b;
            lsf[b] = (esf[r] - esf[l]) / (r - l);
        }
        double s = 0;
        for (int b = 0; b < NB; b++) s += lsf[b];
        if (s < 0) for (int b = 0; b < NB; b++) lsf[b] = -lsf[b];
        double cen = 0, w = 0;
        for (int b = 0; b < NB; b++) { double a = fabs(lsf[b]); cen += a * b; w += a; }
        cen = w > 0 ? cen / w : NB / 2.0;
        for (int b = 0; b < NB; b++) {
            double x = (b - cen) / NB;
            double hw = fabs(x) < 0.5 ? 0.54 + 0.46 * cos(2 * M_PI * x) : 0.08;
            lsf[b] *= hw;
        }
        /* 5. MTF: |DFT| at 0 .. 1 cycles/pixel, normalised at 0, corrected
         * for the central difference (a sinc of two bins) */
        double z = 0;
        for (int b = 0; b < NB; b++) z += lsf[b];
        if (!(z > 0)) return -1;
        for (int k = 0; k < DSD4_MTF_BINS; k++) {
            const double f = (double)k / (DSD4_MTF_BINS - 1); /* cycles per pixel */
            double re = 0, im_ = 0;
            for (int b = 0; b < NB; b++) {
                double ph = 2 * M_PI * f * (b - cen) / OS;
                re += lsf[b] * cos(ph);
                im_ -= lsf[b] * sin(ph);
            }
            double v = hypot(re, im_) / z;
            double arg = M_PI * f * 2.0 / OS;
            double corr = arg > 1e-9 ? arg / sin(arg) : 1;
            mtf[c][k] = v * (corr < 10 ? corr : 10);
        }
    }
    if (ch == 1)
        for (int c = 1; c < 3; c++) memcpy(mtf[c], mtf[0], sizeof mtf[0]);
    return 0;
}

/* frequency (cycles/pixel) where the MTF first falls to 0.5 */
static double mtf50(const double *m)
{
    for (int k = 1; k < DSD4_MTF_BINS; k++)
        if (m[k] < 0.5) {
            double f0 = (double)(k - 1) / (DSD4_MTF_BINS - 1), f1 = (double)k / (DSD4_MTF_BINS - 1);
            return f0 + (m[k - 1] - 0.5) / (m[k - 1] - m[k]) * (f1 - f0);
        }
    return 1;
}

/* gaussian with the same MTF over its useful range: sigma (pixels) */
static double gauss_sigma(const double *m)
{
    double sw = 0, s = 0;
    for (int k = 1; k < DSD4_MTF_BINS; k++) {
        const double f = (double)k / (DSD4_MTF_BINS - 1);
        if (m[k] < 0.15 || m[k] >= 0.98) continue;
        double s2 = -log(m[k]) / (2 * M_PI * M_PI * f * f), w = m[k];
        s += w * s2;
        sw += w;
    }
    return sw > 0 ? sqrt(s / sw) : 0;
}

int dsd4_measure_edge_mtf(const dsd4_image *im, dsd4_mtf_profile *pr, FILE *report)
{
    memset(pr, 0, sizeof *pr);
    if (im->bits != 8 && im->bits != 16 && im->bits != 32) return DSD4_ERR_PARAM;
    if (im->width < 96 || im->height < 96) return DSD4_ERR_PARAM;
    pr->dpi = im->dpi;
    pr->measured[0] = pr->measured[1] = 0;
    pr->samples = 1;
    static const char *dn[2] = { "across (CCD, x)", "along (carriage, y)" };
    int found = 0;
    for (int t = 0; t < 2; t++) {
        int v0, v1;
        double slant;
        int r = measure_dir(im, t, pr->mtf[t], &v0, &v1, &slant, 0, 1);
        if (r == DSD4_ERR_NOMEM) return r;
        if (r) {
            if (report) fprintf(report, "  %s: no slanted edge found (turn the blade %g-%g degrees)\n", dn[t], MIN_DEG, MAX_DEG);
            continue;
        }
        found |= 1 << t;
        pr->measured[t] = 1;
        const int g = im->channels == 3 ? 1 : 0;
        pr->mtf50[t] = mtf50(pr->mtf[t][g]);
        pr->sigma[t] = gauss_sigma(pr->mtf[t][g]);
        const int ny = (DSD4_MTF_BINS - 1) / 2;
        pr->nyquist[t] = pr->mtf[t][g][ny];
        if (report) {
            const double lpmm = im->dpi > 0 ? im->dpi / 25.4 : 0;
            fprintf(report, "  %s: edge slanted %.1f deg, %d %s used\n", dn[t], slant, v1 - v0, t ? "columns" : "rows");
            fprintf(report, "    MTF50 %.3f cycles/px", pr->mtf50[t]);
            if (lpmm > 0) fprintf(report, " = %.1f lp/mm on film", pr->mtf50[t] * lpmm);
            fprintf(report, "; at Nyquist %.2f; like a gaussian of sigma %.2f px\n", pr->nyquist[t], pr->sigma[t]);
            if (im->channels == 3)
                fprintf(report, "    MTF50 red %.3f green %.3f blue %.3f\n", mtf50(pr->mtf[t][0]), mtf50(pr->mtf[t][1]),
                        mtf50(pr->mtf[t][2]));
            /* across the field: thirds of the edge */
            double part[3];
            int ok = 1;
            for (int sgi = 0; sgi < 3 && ok; sgi++) {
                double m3[3][DSD4_MTF_BINS];
                int a, b;
                double sl;
                ok = !measure_dir(im, t, m3, &a, &b, &sl, sgi, 3);
                part[sgi] = ok ? mtf50(m3[g]) : 0;
            }
            if (ok) fprintf(report, "    MTF50 along the edge (thirds): %.3f %.3f %.3f\n", part[0], part[1], part[2]);
        }
    }
    /* one edge only: assume the other direction alike */
    if (found == 1 || found == 2) {
        const int have = found == 1 ? 0 : 1;
        memcpy(pr->mtf[!have], pr->mtf[have], sizeof pr->mtf[0]);
        pr->mtf50[!have] = pr->mtf50[have];
        pr->sigma[!have] = pr->sigma[have];
        pr->nyquist[!have] = pr->nyquist[have];
        if (report) fprintf(report, "  only one edge: %s taken to be the same\n", dn[!have]);
    }
    pr->valid = found != 0;
    return found ? DSD4_OK : DSD4_ERR_PARAM;
}

/* ---------------------------------------------------------- profile file */

int dsd4_mtf_write(const dsd4_mtf_profile *pr, const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return DSD4_ERR_IO;
    fprintf(f, "# dsdual4 scanner MTF (slanted edge): 0..1 cycles/pixel in %d steps\n", DSD4_MTF_BINS);
    fprintf(f, "dpi=%d\nsamples=%d\nfocus=%d\nmeasured_x=%d\nmeasured_y=%d\n", pr->dpi, pr->samples, pr->focus,
            pr->measured[0], pr->measured[1]);
    static const char *dn[2] = { "x", "y" }, *cn[3] = { "r", "g", "b" };
    for (int t = 0; t < 2; t++) {
        fprintf(f, "mtf50_%s=%.5f\nsigma_%s=%.4f\nnyquist_%s=%.4f\n", dn[t], pr->mtf50[t], dn[t], pr->sigma[t], dn[t],
                pr->nyquist[t]);
        for (int c = 0; c < 3; c++) {
            fprintf(f, "%s_%s=", dn[t], cn[c]);
            for (int k = 0; k < DSD4_MTF_BINS; k++) fprintf(f, "%s%.5f", k ? " " : "", pr->mtf[t][c][k]);
            fputc('\n', f);
        }
    }
    int ok = !ferror(f);
    return fclose(f) == 0 && ok ? DSD4_OK : DSD4_ERR_IO;
}

static int mtf_parse(FILE *f, dsd4_mtf_profile *pr)
{
    memset(pr, 0, sizeof *pr);
    char line[4096];
    int got = 0;
    while (fgets(line, sizeof line, f)) {
        char *v = strchr(line, '=');
        if (!v || line[0] == '#') continue;
        *v++ = 0;
        if (!strcmp(line, "dpi")) pr->dpi = atoi(v);
        if (!strcmp(line, "samples")) pr->samples = atoi(v);
        if (!strcmp(line, "focus")) pr->focus = atoi(v);
        if (!strcmp(line, "measured_x")) pr->measured[0] = atoi(v);
        if (!strcmp(line, "measured_y")) pr->measured[1] = atoi(v);
        for (int t = 0; t < 2; t++) {
            const char d = t ? 'y' : 'x';
            char key[16];
            snprintf(key, sizeof key, "mtf50_%c", d);
            if (!strcmp(line, key)) pr->mtf50[t] = atof(v);
            snprintf(key, sizeof key, "sigma_%c", d);
            if (!strcmp(line, key)) pr->sigma[t] = atof(v);
            snprintf(key, sizeof key, "nyquist_%c", d);
            if (!strcmp(line, key)) pr->nyquist[t] = atof(v);
            for (int c = 0; c < 3; c++) {
                snprintf(key, sizeof key, "%c_%c", d, "rgb"[c]);
                if (strcmp(line, key)) continue;
                char *s = v;
                int k = 0;
                for (; k < DSD4_MTF_BINS; k++) {
                    char *e;
                    pr->mtf[t][c][k] = strtod(s, &e);
                    if (e == s) break;
                    s = e;
                }
                if (k == DSD4_MTF_BINS) got |= 1 << (t * 3 + c);
            }
        }
    }
    pr->valid = got == 0x3f && pr->dpi > 0;
    return pr->valid ? DSD4_OK : DSD4_ERR_PARAM;
}

int dsd4_mtf_read(const char *path, dsd4_mtf_profile *pr)
{
    memset(pr, 0, sizeof *pr);
    FILE *f = fopen(path, "r");
    if (!f) return DSD4_ERR_IO;
    int r = mtf_parse(f, pr);
    fclose(f);
    return r;
}

int dsd4_mtf_default(int multi, dsd4_mtf_profile *pr)
{
    const char *t = multi ? dsd4i_default_mtf_multi : dsd4i_default_mtf_1x;
    FILE *f = fmemopen((void *)t, strlen(t), "r");
    if (!f) return DSD4_ERR_NOMEM;
    int r = mtf_parse(f, pr);
    fclose(f);
    return r;
}

void dsd4_mtf_merge(dsd4_mtf_profile *pr, const dsd4_mtf_profile *old)
{
    if (!old->valid || old->dpi != pr->dpi || (old->samples > 1) != (pr->samples > 1)) return;
    for (int t = 0; t < 2; t++)
        if (!pr->measured[t] && old->measured[t]) {
            memcpy(pr->mtf[t], old->mtf[t], sizeof pr->mtf[t]);
            pr->mtf50[t] = old->mtf50[t];
            pr->sigma[t] = old->sigma[t];
            pr->nyquist[t] = old->nyquist[t];
            pr->measured[t] = 1;
        }
}

/* where the longest slanted edge of direction t is: its centre (x, y) and
 * length; -1 if none */
int dsd4_find_edge(const dsd4_image *im, int t, double *x, double *y, int *len)
{
    double m[3][DSD4_MTF_BINS], sl;
    int v0, v1;
    if (measure_dir(im, t, m, &v0, &v1, &sl, 0, 1)) return -1;
    *x = t ? g_cv : g_cu;
    *y = t ? g_cu : g_cv;
    *len = v1 - v0;
    return 0;
}

/* MTF50, Nyquist and gaussian sigma of direction t from its curves */
void dsd4i_mtf_stats(dsd4_mtf_profile *pr, int t)
{
    pr->mtf50[t] = mtf50(pr->mtf[t][1]);
    pr->sigma[t] = gauss_sigma(pr->mtf[t][1]);
    pr->nyquist[t] = pr->mtf[t][1][(DSD4_MTF_BINS - 1) / 2];
}
