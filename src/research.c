/*
 * "dsd4 research": measurements that decide whether multi-pass
 * super-resolution can add real detail on this scanner.
 *
 *   nyquist  is there detail above the noise near the 3200 dpi sampling limit?
 *   focus    how far does a focus change move the image (magnification), and
 *            how much sharpness does it cost?
 *   jitter   do repeated passes land at different sub-line positions?
 *   yres     does the scanner accept a vertical resolution above 3200 dpi
 *            (finer motor steps), and is the result really oversampled?
 *
 * Every test only scans; nothing on the scanner changes permanently.
 */
#include "dsd4_internal.h"
#include "superres.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FW 3120   /* frame width at 3200 dpi */
#define FH 4680

typedef struct {
    dsd4 *s;
    dsd4_scan_params base; /* film, focus resolved, frame already selected */
    FILE *rep;
    const char *outdir;
    dsd4_progress_fn cb;
    void *ud;
    int gate_nyquist, gate_yres, gate_jitter, gate_focus; /* -1 not run, 0/1 */
} rctx;

/* one raw float strip at 3200 dpi */
static int strip(rctx *r, int x, int y, int w, int h, int focus, int samples, float *var, dsd4_image *im)
{
    dsd4_scan_params p = r->base;
    p.dpi = 3200;
    p.preview = 0;
    p.raw = 1;
    p.float_out = 1;
    p.frame = 0;
    p.focus = focus;
    p.multi_sample = samples;
    p.x = x & ~3;
    p.y = y;
    p.w = (w + 3) & ~3;
    p.h = h;
    r->s->dbg_var = var;
    int e = dsd4_scan(r->s, &p, im, r->cb, r->ud);
    r->s->dbg_var = NULL;
    return e;
}

static void save(rctx *r, const dsd4_image *im, const char *name)
{
    if (!r->outdir) return;
    char path[1024];
    snprintf(path, sizeof path, "%s/%s.dng", r->outdir, name);
    if (dsd4_write_dng(im, path, 16, NULL, 0) == DSD4_OK) fprintf(r->rep, "  saved %s\n", path);
}

/* mean power spectrum of rows (horiz) or columns of a plane, Hann window;
 * band[k] = mean power in [k/10, (k+1)/10) x Nyquist, k = 0..9 */
static void spectrum(const sr_plane *p, int horiz, double band[10], double *win_energy)
{
    int len = horiz ? p->w : p->h, cnt = horiz ? p->h : p->w;
    int n = 1;
    while (n * 2 <= len) n *= 2;
    double *re = malloc(sizeof(double) * n * 2), *im = re + n, *pw = calloc((size_t)n / 2 + 1, sizeof(double));
    double we = 0;
    for (int i = 0; i < n; i++) { double w = 0.5 - 0.5 * cos(2 * M_PI * (i + 0.5) / n); we += w * w; }
    for (int j = 0; j < cnt; j++) {
        double m = 0;
        for (int i = 0; i < n; i++) m += horiz ? p->v[(size_t)j * p->w + i] : p->v[(size_t)i * p->w + j];
        m /= n;
        for (int i = 0; i < n; i++) {
            double w = 0.5 - 0.5 * cos(2 * M_PI * (i + 0.5) / n);
            re[i] = ((horiz ? p->v[(size_t)j * p->w + i] : p->v[(size_t)i * p->w + j]) - m) * w;
            im[i] = 0;
        }
        sr_fft(re, im, n, 0);
        for (int k = 1; k <= n / 2; k++) pw[k] += re[k] * re[k] + im[k] * im[k];
    }
    int nb[10] = { 0 };
    memset(band, 0, sizeof(double) * 10);
    for (int k = 1; k <= n / 2; k++) {
        int b = (int)(10.0 * k / (n / 2 + 1));
        if (b > 9) b = 9;
        band[b] += pw[k] / cnt;
        nb[b]++;
    }
    for (int b = 0; b < 10; b++) if (nb[b]) band[b] /= nb[b];
    *win_energy = we;
    free(re);
    free(pw);
}

static double mean_var_green(const float *var, const dsd4_image *im, int samples)
{
    const int ch = im->channels, c = ch == 3 ? 1 : 0;
    double s = 0;
    size_t n = (size_t)im->width * im->height;
    for (size_t i = 0; i < n; i++) s += var[i * ch + c];
    return s / n / samples; /* variance of the mean of 'samples' reads */
}

/* ---------------------------------------------------------------- A1 */

static int test_nyquist(rctx *r)
{
    fprintf(r->rep, "\n[nyquist] detail near the 3200 dpi sampling limit\n");
    const int w = 1024, h = 512, x = FW / 2 - w / 2, y = FH / 2 - h / 2, S = 8;
    float *var = malloc((size_t)w * h * 3 * sizeof *var);
    if (!var) return DSD4_ERR_NOMEM;
    dsd4_image im;
    int e = strip(r, x, y, w, h, r->base.focus, S, var, &im);
    if (e) { free(var); return e; }
    save(r, &im, "nyquist-8x");
    sr_plane p;
    if (sr_plane_from_image(&im, &p)) { free(var); dsd4_image_free(&im); return DSD4_ERR_NOMEM; }
    const double nv = mean_var_green(var, &im, S);
    double best = 0;
    for (int dir = 0; dir < 2; dir++) {
        double band[10], we;
        spectrum(&p, dir == 0, band, &we);
        const double noise = nv * we; /* white noise power per bin */
        fprintf(r->rep, "  %s signal/noise by frequency (x Nyquist):", dir == 0 ? "horizontal" : "vertical  ");
        for (int b = 3; b < 10; b++) fprintf(r->rep, " %.2f:%.1f", (b + 0.5) / 10, noise > 0 ? band[b] / noise : 0);
        double top = noise > 0 ? 0.5 * (band[8] + band[9]) / noise : 0;
        fprintf(r->rep, "\n  %s: %.1fx noise at 0.8-1.0 Nyquist\n", dir == 0 ? "horizontal" : "vertical", top);
        if (top > best) best = top;
    }
    fprintf(r->rep, "  noise: %.2e rms per pixel (8 samples)\n", sqrt(nv));
    r->gate_nyquist = best >= 2.0;
    fprintf(r->rep, "  => %s\n", r->gate_nyquist
            ? "there is detail at the sampling limit: extra sample positions can recover some of it"
            : "the optics already limit below 3200 dpi: more sample positions cannot add detail");
    free(p.v);
    free(var);
    dsd4_image_free(&im);
    return DSD4_OK;
}

/* ------------------------------------------------------- registration */

typedef struct {
    double a, m, rms;      /* dx = a + m * (x - FW/2) */
    double dy, dy_sd;      /* weighted mean vertical shift */
    int tiles;
} shift_fit;

static void fit_shifts_raw(const sr_plane *ref, const sr_plane *b, int x0, shift_fit *f);

/* register low-passed copies (aliased detail does not move with the image) */
static void fit_shifts(const sr_plane *ref, const sr_plane *b, int x0, shift_fit *f)
{
    sr_plane ra = { ref->w, ref->h, malloc(sizeof(float) * (size_t)ref->w * ref->h) };
    sr_plane rb = { b->w, b->h, malloc(sizeof(float) * (size_t)b->w * b->h) };
    memset(f, 0, sizeof *f);
    if (ra.v && rb.v) {
        memcpy(ra.v, ref->v, sizeof(float) * (size_t)ref->w * ref->h);
        memcpy(rb.v, b->v, sizeof(float) * (size_t)b->w * b->h);
        sr_blur(&ra, 1.5);
        sr_blur(&rb, 1.5);
        fit_shifts_raw(&ra, &rb, x0, f);
    }
    free(ra.v);
    free(rb.v);
}

static void fit_shifts_raw(const sr_plane *ref, const sr_plane *b, int x0, shift_fit *f)
{
    enum { T = 64, MAXT = 512 };
    double u[MAXT], vx[MAXT], vy[MAXT], w[MAXT], tex[MAXT];
    int n = 0;
    for (int ty = 4; ty + T + 4 <= ref->h && n < MAXT; ty += T)
        for (int tx = 4; tx + T + 4 <= ref->w && n < MAXT; tx += T) {
            double dx, dy, t = sr_tile_shift(ref, b, tx, ty, T, &dx, &dy);
            if (t <= 0) continue;
            u[n] = x0 + tx + T / 2 - FW / 2.0;
            vx[n] = dx; vy[n] = dy; tex[n] = t;
            n++;
        }
    memset(f, 0, sizeof *f);
    if (n < 3) return;
    /* weight by texture; ignore the flattest quarter */
    double s[MAXT];
    memcpy(s, tex, sizeof(double) * n);
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && s[j - 1] > s[j]; j--) { double q = s[j]; s[j] = s[j - 1]; s[j - 1] = q; }
    const double lo = s[n / 4];
    double sw = 0, sy = 0, syy = 0;
    for (int i = 0; i < n; i++) {
        w[i] = tex[i] >= lo ? sqrt(tex[i]) : 0;
        sw += w[i]; sy += w[i] * vy[i]; syy += w[i] * vy[i] * vy[i];
    }
    sr_fit_line(u, vx, w, n, &f->a, &f->m, &f->rms);
    f->dy = sy / sw;
    f->dy_sd = sqrt(fmax(0, syy / sw - f->dy * f->dy));
    f->tiles = n;
}

static double phase_dist(double d) { d -= floor(d); return d > 0.5 ? 1 - d : d; }

/* ---------------------------------------------------------------- A2 */

static int test_focus(rctx *r)
{
    fprintf(r->rep, "\n[focus] image shift and sharpness against focus offset (best focus %d)\n", r->base.focus);
    /* best-focus reference passes between the offsets: film drift (warming,
     * curl) is measured from them and removed, leaving the lens's own effect */
    static const int off[] = { 0, -20, 0, -10, 0, -5, 0, 5, 0, 10, 0, 20, 0 };
    enum { N = sizeof off / sizeof *off, NO = N / 2 };
    const int h = 384, y = FH / 2 - h / 2;
    sr_plane first = { 0 }, p;
    shift_fit fit[N];
    double sh[N];
    for (int k = 0; k < N; k++) {
        dsd4_image im;
        int f = r->base.focus + off[k];
        if (f < -200) f = -200;
        if (f > 200) f = 200;
        int e = strip(r, 0, y, FW, h, f, 1, NULL, &im);
        if (e) { free(first.v); return e; }
        if (sr_plane_from_image(&im, &p)) { dsd4_image_free(&im); free(first.v); return DSD4_ERR_NOMEM; }
        if (k == 0) save(r, &im, "focus-best");
        dsd4_image_free(&im);
        sh[k] = sr_sharpness(&p);
        if (k == 0) { first = p; memset(&fit[0], 0, sizeof fit[0]); continue; }
        fit_shifts(&first, &p, 0, &fit[k]);
        free(p.v);
    }
    free(first.v);
    fprintf(r->rep, "  drift of the best-focus passes (film moving as it warms):\n   ");
    for (int k = 0; k < N; k += 2) fprintf(r->rep, " %+.2fpx/%+.0fppm", fit[k].a, fit[k].m * 1e6);
    fprintf(r->rep, "\n  offset  sharpness   dx@left  dx@centre  dx@right   dy     scale(ppm)   (drift removed)\n");
    double u[NO], v[NO], w[NO], so[NO];
    int ou[NO];
    for (int i = 0; i < NO; i++) {
        const int k = 2 * i + 1;
        /* subtract the mean of the neighbouring reference passes */
        const double a = fit[k].a - 0.5 * (fit[k - 1].a + fit[k + 1].a);
        const double m = fit[k].m - 0.5 * (fit[k - 1].m + fit[k + 1].m);
        const double dy = fit[k].dy - 0.5 * (fit[k - 1].dy + fit[k + 1].dy);
        so[i] = sh[k] / (0.5 * (sh[k - 1] + sh[k + 1]));
        u[i] = off[k]; v[i] = m; w[i] = 1; ou[i] = off[k];
        fprintf(r->rep, "  %+5d   %5.1f %%   %+7.3f   %+7.3f   %+7.3f  %+7.3f   %+8.1f\n", off[k], 100 * so[i],
                a + m * (64 - FW / 2.0), a, a + m * (FW - 64 - FW / 2.0), dy, m * 1e6);
    }
    double c0, c1 = 0, rms;
    sr_fit_line(u, v, w, NO, &c0, &c1, &rms);
    int dof_lo = 0, dof_hi = 0;
    for (int i = 0; i < NO; i++)
        if (so[i] >= 0.9) { if (ou[i] < dof_lo) dof_lo = ou[i]; if (ou[i] > dof_hi) dof_hi = ou[i]; }
    int good = 0, tot = 0;
    for (int x = 0; x < FW; x += 8, tot++) {
        double best = 0;
        for (int o = dof_lo; o <= dof_hi; o++) {
            double d = phase_dist(c1 * o * (x - FW / 2.0));
            if (d > best) best = d;
        }
        if (best >= 0.3) good++;
    }
    fprintf(r->rep, "  scale change: %.2f ppm per focus step (fit rms %.1f ppm) = %.3f px at the frame edge per step\n",
            c1 * 1e6, rms * 1e6, fabs(c1) * FW / 2.0);
    fprintf(r->rep, "  depth of field (>= 90 %% sharpness): %+d .. %+d steps\n", dof_lo, dof_hi);
    fprintf(r->rep, "  width reaching a >= 0.3 px sub-pixel phase within it: %.0f %%\n", 100.0 * good / tot);
    r->gate_focus = good * 2 >= tot;
    fprintf(r->rep, "  => %s\n", r->gate_focus
            ? "focus changes can supply horizontal sub-pixel phases over most of the frame"
            : "focus changes shift too little inside the depth of field: horizontal gain limited to the edges");
    return DSD4_OK;
}

/* ---------------------------------------------------------------- A3 */

static int test_jitter(rctx *r)
{
    fprintf(r->rep, "\n[jitter] sub-line position of repeated passes\n");
    enum { N = 8 };
    const int w = 1024, h = 384, x = FW / 2 - w / 2, y = FH / 2 - h / 2;
    sr_plane ref = { 0 }, p;
    double dy[N] = { 0 }, dx[N] = { 0 };
    for (int k = 0; k < N; k++) {
        dsd4_image im;
        int e = strip(r, x, y, w, h, r->base.focus, 1, NULL, &im);
        if (e) { free(ref.v); return e; }
        if (sr_plane_from_image(&im, &p)) { dsd4_image_free(&im); free(ref.v); return DSD4_ERR_NOMEM; }
        dsd4_image_free(&im);
        if (k == 0) { ref = p; continue; }
        shift_fit fit;
        fit_shifts(&ref, &p, x, &fit);
        dx[k] = fit.a;
        dy[k] = fit.dy;
        free(p.v);
    }
    free(ref.v);
    double m = 0, sd = 0, spread = 0;
    fprintf(r->rep, "  pass:   ");
    for (int k = 0; k < N; k++) fprintf(r->rep, " %6d", k + 1);
    fprintf(r->rep, "\n  dy:     ");
    for (int k = 0; k < N; k++) { fprintf(r->rep, " %+6.3f", dy[k]); m += dy[k]; }
    fprintf(r->rep, "\n  dx:     ");
    for (int k = 0; k < N; k++) fprintf(r->rep, " %+6.3f", dx[k]);
    m /= N;
    /* phase coverage: largest gap between the passes' sub-line phases (mod 1) */
    double ph[N];
    for (int k = 0; k < N; k++) { sd += (dy[k] - m) * (dy[k] - m); ph[k] = dy[k] - floor(dy[k]); }
    for (int i = 1; i < N; i++) for (int j = i; j > 0 && ph[j - 1] > ph[j]; j--) { double q = ph[j]; ph[j] = ph[j - 1]; ph[j - 1] = q; }
    spread = 1 - ph[N - 1] + ph[0];
    for (int i = 1; i < N; i++) if (ph[i] - ph[i - 1] > spread) spread = ph[i] - ph[i - 1];
    sd = sqrt(sd / (N - 1));
    fprintf(r->rep, "\n  vertical repeatability: %.3f lines rms; largest gap between sub-line phases: %.2f line\n", sd,
            spread);
    r->gate_jitter = spread <= 0.5;
    fprintf(r->rep, "  => %s\n", r->gate_jitter
            ? "passes land at different sub-line phases: usable for vertical super-resolution"
            : "passes repeat to within a fraction of a line: no free vertical phases");
    return DSD4_OK;
}

/* ---------------------------------------------------------------- A4 */

/* vertical geometry of scan b against a 3200 dpi reference a (same columns):
 * the 3200 lines per b line (k) and the start offset, by correlation of
 * low-passed copies */
static double match_vertical(const sr_plane *a, const sr_plane *b, double *k_out, double *y0_out)
{
    double best = -2;
    for (double k = 0.30; k <= 1.205; k += 0.01)
        for (double y0 = 0; y0 + k * (b->h - 1) < a->h - 1; y0 += 0.5) {
            double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
            long n = 0;
            for (int yb = 4; yb < b->h - 4; yb += 2) {
                double ya = y0 + k * yb;
                int i0 = (int)ya;
                double f = ya - i0;
                for (int x = 8; x < a->w - 8; x += 8) {
                    double va = (1 - f) * a->v[(size_t)i0 * a->w + x] + f * a->v[(size_t)(i0 + 1) * a->w + x];
                    double vb = b->v[(size_t)yb * b->w + x];
                    sa += va; sb += vb; saa += va * va; sbb += vb * vb; sab += va * vb;
                    n++;
                }
            }
            double c = (sab - sa * sb / n) / sqrt((saa - sa * sa / n) * (sbb - sb * sb / n) + 1e-30);
            if (c > best) { best = c; *k_out = k; *y0_out = y0; }
        }
    return best;
}

static int test_yres(rctx *r)
{
    fprintf(r->rep, "\n[yres] vertical resolution above 3200 dpi (finer motor steps)\n");
    const int w = 1024, h = 128, x = FW / 2 - w / 2, y = FH / 2 - h / 2, S = 4;
    r->gate_yres = 0;
    /* 3200 dpi reference around the probe area, tall enough for a probe that
     * steps a whole 3200 line per line (2x the requested height) */
    dsd4_image ri;
    int e = strip(r, x, y - h / 2, w, 3 * h, r->base.focus, 1, NULL, &ri);
    if (e) return e;
    sr_plane ref;
    if (sr_plane_from_image(&ri, &ref)) { dsd4_image_free(&ri); return DSD4_ERR_NOMEM; }
    dsd4_image_free(&ri);
    sr_blur(&ref, 1.2);
    static const int yrs[] = { 4800, 6400 };
    for (int i = 0; i < 2; i++) {
        const int yr = yrs[i];
        const int lines = h * yr / 3200;
        float *var = malloc((size_t)w * lines * 3 * sizeof *var);
        if (!var) { free(ref.v); return DSD4_ERR_NOMEM; }
        dsd4_image im;
        r->s->probe_yres = (uint16_t)yr;
        e = strip(r, x, y, w, h, r->base.focus, S, var, &im);
        r->s->probe_yres = 0;
        if (e) {
            if (r->s->last_error)
                fprintf(r->rep, "  %d dpi: refused by the scanner (code 0x%02x: %s)\n", yr, r->s->last_error,
                        dsd4_scanner_error_text(r->s->last_error));
            else
                fprintf(r->rep, "  %d dpi: refused by the scanner (%s)\n", yr, dsd4_strerror(e));
            free(var);
            dsd4_home(r->s);
            continue;
        }
        char name[32];
        snprintf(name, sizeof name, "yres-%d", yr);
        save(r, &im, name);
        sr_plane p;
        if (sr_plane_from_image(&im, &p)) { free(var); dsd4_image_free(&im); free(ref.v); return DSD4_ERR_NOMEM; }
        sr_blur(&p, 1.2);
        double k = 0, y0 = 0;
        const double c = match_vertical(&ref, &p, &k, &y0);
        const double want = 3200.0 / yr;
        const int real = c > 0.98 && fabs(k - want) < 0.05 * want;
        fprintf(r->rep, "  %d dpi: accepted, %d lines; they advance %.3f lines of 3200 each (%.3f for real %d dpi"
                        " sampling; match %.4f) => %s\n", yr, im.height, k, want, yr, c,
                real ? "really oversampled" : "NOT finer: the scanner still steps at its own pitch");
        if (real) r->gate_yres = 1;
        free(p.v);
        free(var);
        dsd4_image_free(&im);
    }
    free(ref.v);
    fprintf(r->rep, "  => %s\n", r->gate_yres
            ? "the scanner oversamples vertically: one pass gives finer vertical sampling"
            : "no vertical oversampling from the window resolution");
    return DSD4_OK;
}

int dsd4_research(dsd4 *s, const dsd4_scan_params *sp, const char *which, const char *outdir, FILE *report,
                  dsd4_progress_fn cb, void *ud)
{
    rctx r = { s, *sp, report, outdir, cb, ud, -1, -1, -1, -1 };
    int e;
    if (r.base.frame > 0) {
        if ((e = dsd4_select_frame(s, r.base.frame))) return e;
        r.base.frame = 0;
    }
    if (r.base.focus == DSD4_FOCUS_AUTO) {
        int best = 0;
        if ((e = dsd4_autofocus(s, &r.base, &best, cb, ud))) return e;
        r.base.focus = best;
    }
    const int all = !strcmp(which, "all");
    fprintf(report, "dsd4 research: film type %d, focus %d\n", (int)r.base.film, r.base.focus);
    if ((all || !strcmp(which, "nyquist")) && (e = test_nyquist(&r))) return e;
    if ((all || !strcmp(which, "focus")) && (e = test_focus(&r))) return e;
    if ((all || !strcmp(which, "jitter")) && (e = test_jitter(&r))) return e;
    if ((all || !strcmp(which, "yres")) && (e = test_yres(&r))) return e;
    fprintf(report, "\nsummary\n");
    static const char *st[] = { "no", "yes" };
    if (r.gate_nyquist >= 0) fprintf(report, "  G1 detail at the sampling limit:          %s\n", st[r.gate_nyquist]);
    if (r.gate_yres >= 0) fprintf(report, "  G2 vertical oversampling via window:      %s\n", st[r.gate_yres]);
    if (r.gate_jitter >= 0) fprintf(report, "     vertical phases from pass jitter:      %s\n", st[r.gate_jitter]);
    if (r.gate_focus >= 0) fprintf(report, "  G3 horizontal phases from focus changes:  %s\n", st[r.gate_focus]);
    return DSD4_OK;
}

/* ------------------------------------------------- calibration: MTF, noise */

static int prep(dsd4 *s, const dsd4_scan_params *sp, rctx *r)
{
    int e;
    if (r->base.frame > 0) {
        if ((e = dsd4_select_frame(s, r->base.frame))) return e;
        r->base.frame = 0;
    }
    if (r->base.focus == DSD4_FOCUS_AUTO) {
        int best = 0;
        if ((e = dsd4_autofocus(s, &r->base, &best, r->cb, r->ud))) return e;
        r->base.focus = best;
    }
    (void)sp;
    return 0;
}

/* MTF50 (cycles/px) of the edge in a 3200 dpi strip at a focus; -1: none */
static double strip_mtf50(rctx *r, int t, int cx, int cy, int focus)
{
    const int w = t ? 1024 : 384, h = t ? 384 : 1024;
    int x = cx - w / 2, y = cy - h / 2;
    x = x < 0 ? 0 : x + w > 3120 ? 3120 - w : x;
    y = y < 0 ? 0 : y + h > 4680 ? 4680 - h : y;
    dsd4_image im;
    if (strip(r, x, y, w, h, focus, 2, NULL, &im)) return -1;
    im.dpi = 3200;
    dsd4_mtf_profile pr;
    double m = dsd4_measure_edge_mtf(&im, &pr, NULL) == DSD4_OK && pr.measured[t] ? pr.mtf50[t] : -1;
    dsd4_image_free(&im);
    return m;
}

int dsd4_research_mtf(dsd4 *s, const dsd4_scan_params *sp, const char *outdir, dsd4_mtf_profile pr[2], FILE *report,
                      dsd4_progress_fn cb, void *ud)
{
    rctx r = { s, *sp, report, outdir, cb, ud, -1, -1, -1, -1 };
    int e;
    if (r.base.frame > 0) {
        if ((e = dsd4_select_frame(s, r.base.frame))) return e;
        r.base.frame = 0;
    }
    fprintf(report, "dsd4 research mtf: razor blade\n");
    /* 1. find the blade: the whole frame at 800 dpi */
    const int f0 = r.base.focus == DSD4_FOCUS_AUTO ? 0 : r.base.focus;
    dsd4_scan_params q = r.base;
    q.dpi = 800;
    q.raw = 1;
    q.float_out = 1;
    q.focus = f0;
    q.preview = 0;
    q.multi_sample = 1;
    q.x = q.y = q.w = q.h = 0;
    dsd4_image lo;
    if ((e = dsd4_scan(s, &q, &lo, cb, ud))) return e;
    double ex, ey;
    int t = -1, len = 0, best_len = 0;
    for (int d = 0; d < 2; d++) {
        double x, y;
        if (!dsd4_find_edge(&lo, d, &x, &y, &len) && len > best_len) { best_len = len; t = d; ex = x; ey = y; }
    }
    const double k = 3200.0 / lo.dpi;
    dsd4_image_free(&lo);
    if (t < 0) {
        fprintf(report, "  no slanted edge: put a razor blade in a slide mount, its edge turned 2-20 degrees\n"
                        "  from the mount's sides (a corner in the window measures both directions)\n");
        return DSD4_ERR_PARAM;
    }
    const int cx = (int)(ex * k), cy = (int)(ey * k);
    fprintf(report, "  edge %s at (%d, %d)\n", t ? "near-horizontal" : "near-vertical", cx, cy);
    /* 2. focus: autofocus does not lock on a blade; sweep, MTF50 on a strip */
    int focus = r.base.focus;
    if (focus == DSD4_FOCUS_AUTO) {
        double bm = -1;
        int bf = 0;
        for (int f = -150; f <= 210; f += 30) {
            double m = strip_mtf50(&r, t, cx, cy, f);
            fprintf(report, "  focus %4d: MTF50 %.3f\n", f, m);
            if (m > bm) { bm = m; bf = f; }
        }
        for (int f = bf - 20; f <= bf + 20; f += 5) {
            if (f == bf) continue;
            double m = strip_mtf50(&r, t, cx, cy, f);
            fprintf(report, "  focus %4d: MTF50 %.3f\n", f, m);
            if (m > bm) { bm = m; bf = f; }
        }
        focus = bf;
        fprintf(report, "  best focus %d\n", focus);
    }
    /* 3. the whole frame at that focus: single reads, then 2x multi-sampled */
    for (int i = 0; i < 2; i++) {
        const int samples = i ? (r.base.multi_sample > 1 ? r.base.multi_sample : 2) : 1;
        fprintf(report, "  %s:\n", i ? "multi-sampled (the carriage stops for each line)" : "single reads");
        dsd4_image im;
        if ((e = strip(&r, 0, 0, 3120, 4680, focus, samples, NULL, &im))) return e;
        im.dpi = 3200;
        save(&r, &im, i ? "mtf-blade-multi" : "mtf-blade-1x");
        e = dsd4_measure_edge_mtf(&im, &pr[i], report);
        dsd4_image_free(&im);
        if (e) return e;
        pr[i].samples = samples;
        pr[i].focus = focus;
    }
    return DSD4_OK;
}

/* ------------------------------------------- MTF across, from film grain */

#define GT 128 /* grain tiles */

typedef struct { int x, y; } tile_t;

/* average axis spectra of the grain on the given tiles: nx[c][k] across
 * (frequency along x, fy ~ 0), ny[c][k] along; minus the sensor noise
 * expected there (one read's variance / samples, its spectrum shape) */
static void grain_axes(const dsd4_image *im, const tile_t *t, int nt, const dsd4_noise_profile *np, int samples,
                       double nx[3][GT / 2 + 1], double ny[3][GT / 2 + 1], double sx[3][GT / 2 + 1],
                       double sy[3][GT / 2 + 1])
{
    static double win[GT * GT];
    double ws = 0;
    for (int y = 0; y < GT; y++)
        for (int x = 0; x < GT; x++) {
            double w = (0.5 - 0.5 * cos(2 * M_PI * (x + 0.5) / GT)) * (0.5 - 0.5 * cos(2 * M_PI * (y + 0.5) / GT));
            win[y * GT + x] = w;
            ws += w * w;
        }
    double *re = malloc(GT * GT * sizeof *re), *im_ = malloc(GT * GT * sizeof *im_), *rr = malloc(GT * sizeof *rr),
           *ii = malloc(GT * sizeof *ii);
    const int ch = im->channels;
    memset(nx, 0, sizeof(double) * 3 * (GT / 2 + 1));
    memset(ny, 0, sizeof(double) * 3 * (GT / 2 + 1));
    memset(sx, 0, sizeof(double) * 3 * (GT / 2 + 1));
    memset(sy, 0, sizeof(double) * 3 * (GT / 2 + 1));
    for (int c = 0; c < ch; c++) {
        for (int q = 0; q < nt; q++) {
            double m = 0;
            for (int y = 0; y < GT; y++)
                for (int x = 0; x < GT; x++) m += ((float *)im->data)[((size_t)(t[q].y + y) * im->width + t[q].x + x) * ch + c];
            m /= GT * GT;
            for (int y = 0; y < GT; y++)
                for (int x = 0; x < GT; x++) {
                    re[y * GT + x] = (((float *)im->data)[((size_t)(t[q].y + y) * im->width + t[q].x + x) * ch + c] - m) * win[y * GT + x];
                    im_[y * GT + x] = 0;
                }
            /* 2-D FFT: rows, then columns */
            for (int y = 0; y < GT; y++) sr_fft(re + y * GT, im_ + y * GT, GT, 0);
            for (int x = 0; x < GT; x++) {
                for (int y = 0; y < GT; y++) { rr[y] = re[y * GT + x]; ii[y] = im_[y * GT + x]; }
                sr_fft(rr, ii, GT, 0);
                for (int y = 0; y < GT; y++) { re[y * GT + x] = rr[y]; im_[y * GT + x] = ii[y]; }
            }
            const double vs = np && np->valid ? dsd4_noise_var(np, c, m) / samples : 0;
            for (int k = 1; k <= GT / 2; k++) {
                double ax = 0, ay = 0;
                for (int d = -1; d <= 1; d++) {
                    const int o = (d + GT) % GT;
                    ax += (re[o * GT + k] * re[o * GT + k] + im_[o * GT + k] * im_[o * GT + k]);
                    ay += (re[k * GT + o] * re[k * GT + o] + im_[k * GT + o] * im_[k * GT + o]);
                }
                nx[c][k] += ax / 3 / ws;
                ny[c][k] += ay / 3 / ws;
                const double f = (double)k / GT, b = f / 0.5 * (DSD4_NPS_BINS - 1);
                const int bi = (int)b < DSD4_NPS_BINS - 1 ? (int)b : DSD4_NPS_BINS - 2;
                double shx = 1, shy = 1;
                if (np && np->valid) {
                    shx = np->shape_x[bi] + (b - bi) * (np->shape_x[bi + 1] - np->shape_x[bi]);
                    shy = np->shape_y[bi] + (b - bi) * (np->shape_y[bi + 1] - np->shape_y[bi]);
                }
                sx[c][k] += vs * shx;
                sy[c][k] += vs * shy;
            }
        }
        for (int k = 1; k <= GT / 2; k++) { nx[c][k] /= nt; ny[c][k] /= nt; sx[c][k] /= nt; sy[c][k] /= nt; }
    }
    if (ch == 1)
        for (int c = 1; c < 3; c++)
            for (int k = 0; k <= GT / 2; k++) { nx[c][k] = nx[0][k]; ny[c][k] = ny[0][k]; sx[c][k] = sx[0][k]; sy[c][k] = sy[0][k]; }
    free(re); free(im_); free(rr); free(ii);
}

int dsd4_research_grain(dsd4 *s, const dsd4_scan_params *sp, const char *outdir, const dsd4_noise_profile *np,
                        dsd4_mtf_profile pr[2], FILE *report, dsd4_progress_fn cb, void *ud)
{
    rctx r = { s, *sp, report, outdir, cb, ud, -1, -1, -1, -1 };
    int e = prep(s, sp, &r);
    if (e) return e;
    fprintf(report, "dsd4 research grain: the scanner's MTF across, from film grain (focus %d)\n", r.base.focus);
    if (!np || !np->valid) fprintf(report, "  (no sensor noise profile: grain taken as is; run research noise first)\n");
    const int ms = r.base.multi_sample > 1 ? r.base.multi_sample : 4;
    dsd4_image im[2];
    for (int i = 1; i >= 0; i--) {
        if ((e = strip(&r, 0, 0, 3120, 4680, r.base.focus, i ? ms : 1, NULL, &im[i]))) {
            if (i == 0) dsd4_image_free(&im[1]);
            return e;
        }
        save(&r, &im[i], i ? "grain-multi" : "grain-1x");
    }
    /* the flattest quarter of the frame (on the multi-sampled scan), away
     * from the edges and from clipped or near-black areas */
    const int W = im[1].width, H = im[1].height, ch = im[1].channels, g = ch == 3 ? 1 : 0;
    const int tw = (W - 2 * GT) / GT, th = (H - 2 * GT) / GT;
    tile_t *t = malloc((size_t)tw * th * sizeof *t);
    double *fl = malloc((size_t)tw * th * sizeof *fl);
    if (!t || !fl) { free(t); free(fl); dsd4_image_free(&im[0]); dsd4_image_free(&im[1]); return DSD4_ERR_NOMEM; }
    int nt = 0;
    for (int ty = 0; ty < th; ty++)
        for (int tx = 0; tx < tw; tx++) {
            const int x0 = GT + tx * GT, y0 = GT + ty * GT;
            double s1 = 0, s2 = 0, lo = 1e9, hi = -1;
            for (int by = 0; by < GT; by += 8)
                for (int bx = 0; bx < GT; bx += 8) {
                    double a = 0;
                    for (int y = 0; y < 8; y++)
                        for (int x = 0; x < 8; x++) {
                            float v = ((float *)im[1].data)[((size_t)(y0 + by + y) * W + x0 + bx + x) * ch + g];
                            a += v;
                            lo = v < lo ? v : lo;
                            hi = v > hi ? v : hi;
                        }
                    a /= 64;
                    s1 += a; s2 += a * a;
                }
            const double m = s1 / 256;
            if (m < 0.03 || hi > 0.95) continue;
            fl[nt] = sqrt(fmax(s2 / 256 - m * m, 0)) / m;
            t[nt++] = (tile_t){ x0, y0 };
        }
    for (int i = 1; i < nt; i++)
        for (int j = i; j > 0 && fl[j - 1] > fl[j]; j--) {
            double a = fl[j]; fl[j] = fl[j - 1]; fl[j - 1] = a;
            tile_t b = t[j]; t[j] = t[j - 1]; t[j - 1] = b;
        }
    nt = nt / 4;
    free(fl);
    if (nt < 16) {
        fprintf(report, "  too few flat areas: use a frame with sky, a wall or blank film\n");
        free(t); dsd4_image_free(&im[0]); dsd4_image_free(&im[1]);
        return DSD4_ERR_PARAM;
    }
    fprintf(report, "  grain from %d flat areas of %dx%d px\n", nt, GT, GT);
    static double nx[2][3][GT / 2 + 1], ny[2][3][GT / 2 + 1], sx[2][3][GT / 2 + 1], sy[2][3][GT / 2 + 1];
    for (int i = 0; i < 2; i++) grain_axes(&im[i], t, nt, np, i ? ms : 1, nx[i], ny[i], sx[i], sy[i]);
    free(t);
    dsd4_image_free(&im[0]);
    dsd4_image_free(&im[1]);
    /* ratio across / along of the grain's power (minus the sensor's), per
     * channel, smoothed; used where the grain stands well above the sensor
     * noise, held beyond */
    for (int i = 0; i < 2; i++) {
        fprintf(report, "  %s:\n", i ? "multi-sampled" : "single reads");
        double ratio[3][GT / 2 + 1];
        for (int c = 0; c < 3; c++) {
            double last = 1;
            for (int k = 1; k <= GT / 2; k++) {
                double rs = 0;
                int n = 0, ok = 1;
                for (int d = -1; d <= 1; d++) {
                    const int kk = k + d < 1 ? 1 : k + d > GT / 2 ? GT / 2 : k + d;
                    const double gx = nx[i][c][kk] - sx[i][c][kk], gy = ny[i][c][kk] - sy[i][c][kk];
                    if (gx < sx[i][c][kk] || gy < sy[i][c][kk] || gx <= 0 || gy <= 0) { ok = 0; continue; }
                    rs += gx / gy;
                    n++;
                }
                if (ok && n) last = rs / n;
                last = last < 0.25 ? 0.25 : last > 4 ? 4 : last;
                ratio[c][k] = last;
            }
            ratio[c][0] = 1;
        }
        fprintf(report, "    MTF across / along (green):");
        for (int k = 16; k <= GT / 2; k += 16) fprintf(report, " %.2f at %.3f", sqrt(ratio[1][k]), (double)k / GT);
        fputc('\n', report);
        /* across = along x sqrt(ratio); beyond Nyquist the last ratio */
        for (int c = 0; c < 3; c++)
            for (int b = 0; b < DSD4_MTF_BINS; b++) {
                const double f = (double)b / (DSD4_MTF_BINS - 1);
                double kk = f * GT;
                if (kk > GT / 2) kk = GT / 2;
                const int k0 = (int)kk, k1 = k0 < GT / 2 ? k0 + 1 : k0;
                const double rt = ratio[c][k0] + (kk - k0) * (ratio[c][k1] - ratio[c][k0]);
                double v = pr[i].mtf[1][c][b] * sqrt(rt);
                pr[i].mtf[0][c][b] = v > 1.05 ? 1.05 : v;
            }
        pr[i].measured[0] = 2; /* from grain */
        dsd4i_mtf_stats(&pr[i], 0);
        fprintf(report, "    across: MTF50 %.3f cycles/px = %.1f lp/mm, at Nyquist %.2f (along: MTF50 %.3f)\n",
                pr[i].mtf50[0], pr[i].mtf50[0] * 3200 / 25.4, pr[i].nyquist[0], pr[i].mtf50[1]);
    }
    /* the carriage: single reads against multi-sampled, along, on film */
    fprintf(report, "  along the carriage, single reads / multi-sampled (green):");
    for (int k = 16; k <= GT / 2; k += 16) {
        const double a = ny[0][1][k] - sy[0][1][k], b = ny[1][1][k] - sy[1][1][k];
        if (a > 0 && b > 0) fprintf(report, " %.2f at %.3f", sqrt(a / b), (double)k / GT);
    }
    fputc('\n', report);
    return DSD4_OK;
}

/* signal levels: exposure halved five times from the film's standard */
int dsd4_research_noise(dsd4 *s, const dsd4_scan_params *sp, const char *outdir, dsd4_noise_profile *np,
                        FILE *report, dsd4_progress_fn cb, void *ud)
{
    memset(np, 0, sizeof *np);
    rctx r = { s, *sp, report, outdir, cb, ud, -1, -1, -1, -1 };
    int e = prep(s, sp, &r);
    if (e) return e;
    fprintf(report, "dsd4 research noise: sensor noise from 8 reads of each pixel\n");
    enum { W = 1024, H = 256, S = 8, NX = 32 };
    double std[3];
    dsd4_film_default_exposure(r.base.film, std);
    double psx[DSD4_NPS_BINS] = { 0 }, psy[DSD4_NPS_BINS] = { 0 };
    for (int lv = 0; lv < DSD4_NOISE_LEVELS; lv++) {
        const double f = pow(0.5, lv);
        for (int c = 0; c < 3; c++) r.base.exposure[c] = std[c] * f;
        float *var = malloc((size_t)W * H * 3 * sizeof *var), *dev = malloc((size_t)W * H * 3 * sizeof *dev);
        if (!var || !dev) { free(var); free(dev); return DSD4_ERR_NOMEM; }
        dsd4_image im;
        s->dbg_dev = dev;
        e = strip(&r, (3120 - W) / 2, (4680 - H) / 2, W, H, r.base.focus, S, var, &im);
        s->dbg_dev = NULL;
        if (e) { free(var); free(dev); return e; }
        const int ch = im.channels, w = im.width, h = im.height;
        fprintf(report, "  exposure 1/%-3d", 1 << lv);
        for (int c = 0; c < ch; c++) {
            double m = 0, v = 0;
            for (int i = 0; i < w * h; i++) { m += ((float *)im.data)[i * ch + c]; v += var[i * ch + c]; }
            m /= (double)w * h;
            v /= (double)w * h;
            np->mean[c][lv] = m;
            np->var[c][lv] = v;
            fprintf(report, "  %c: signal %.3f noise %.5f (%.1f/65535)", "RGB"[ch == 3 ? c : 1], m, sqrt(v), sqrt(v) * 65535);
        }
        fputc('\n', report);
        if (ch == 1)
            for (int c = 1; c < 3; c++) { np->mean[c][lv] = np->mean[0][lv]; np->var[c][lv] = np->var[0][lv]; }
        /* spectrum of the deviations: rows (x) and columns (y), 32-point
         * periodograms, all channels */
        double re[NX], im_[NX];
        for (int c = 0; c < ch; c++) {
            for (int y = 0; y < h; y++)
                for (int x0 = 0; x0 + NX <= w; x0 += NX) {
                    for (int k = 0; k < NX; k++) { re[k] = dev[((size_t)y * w + x0 + k) * ch + c]; im_[k] = 0; }
                    sr_fft(re, im_, NX, 0);
                    for (int k = 0; k < DSD4_NPS_BINS; k++) psx[k] += re[k] * re[k] + im_[k] * im_[k];
                }
            for (int x = 0; x < w; x++)
                for (int y0 = 0; y0 + NX <= h; y0 += NX) {
                    for (int k = 0; k < NX; k++) { re[k] = dev[((size_t)(y0 + k) * w + x) * ch + c]; im_[k] = 0; }
                    sr_fft(re, im_, NX, 0);
                    for (int k = 0; k < DSD4_NPS_BINS; k++) psy[k] += re[k] * re[k] + im_[k] * im_[k];
                }
        }
        dsd4_image_free(&im);
        free(var);
        free(dev);
    }
    /* relative: mean 1 over the non-zero frequencies */
    double mx = 0, my = 0;
    for (int k = 1; k < DSD4_NPS_BINS; k++) { mx += psx[k]; my += psy[k]; }
    for (int k = 0; k < DSD4_NPS_BINS; k++) {
        np->shape_x[k] = mx > 0 ? psx[k] * (DSD4_NPS_BINS - 1) / mx : 1;
        np->shape_y[k] = my > 0 ? psy[k] * (DSD4_NPS_BINS - 1) / my : 1;
    }
    fprintf(report, "  spectrum (1 = white): across %.2f %.2f %.2f at 1/8, 1/4, 1/2 cycles/px; along %.2f %.2f %.2f\n",
            np->shape_x[4], np->shape_x[8], np->shape_x[16], np->shape_y[4], np->shape_y[8], np->shape_y[16]);
    np->levels = DSD4_NOISE_LEVELS;
    np->valid = 1;
    return DSD4_OK;
}

/* ---------------------------------------------------- noise profile file */

/* sensor noise follows read noise + photon noise: var = a + b x.  Between
 * measured levels: interpolated; beyond them (the scanner's shortest
 * exposure limits how dark calibration gets): extrapolated along that line
 * through the two outermost distinct levels, never below zero */
double dsd4_noise_var(const dsd4_noise_profile *np, int c, double x)
{
    const int n = np->levels;
    if (!np->valid || n < 1) return 0;
    int lo = -1, hi = -1, mn = 0, mx = 0;
    for (int i = 0; i < n; i++) {
        if (np->mean[c][i] < np->mean[c][mn]) mn = i;
        if (np->mean[c][i] > np->mean[c][mx]) mx = i;
        if (np->mean[c][i] <= x && (lo < 0 || np->mean[c][i] > np->mean[c][lo])) lo = i;
        if (np->mean[c][i] >= x && (hi < 0 || np->mean[c][i] < np->mean[c][hi])) hi = i;
    }
    if (lo >= 0 && hi >= 0 && np->mean[c][hi] > np->mean[c][lo]) {
        const double t = (x - np->mean[c][lo]) / (np->mean[c][hi] - np->mean[c][lo]);
        return np->var[c][lo] + t * (np->var[c][hi] - np->var[c][lo]);
    }
    if (lo >= 0 && hi >= 0) return np->var[c][lo];
    const double span = np->mean[c][mx] - np->mean[c][mn];
    if (span < 0.02) return np->var[c][lo >= 0 ? lo : hi];
    const double b = (np->var[c][mx] - np->var[c][mn]) / span;
    const double v = np->var[c][mn] + b * (x - np->mean[c][mn]);
    return v > 0 ? v : 0;
}

int dsd4_noise_write(const dsd4_noise_profile *np, const char *path)
{
    FILE *f = fopen(path, "w");
    if (!f) return DSD4_ERR_IO;
    fprintf(f, "# dsd4 sensor noise: one read's variance at raw signal levels; spectrum 0..0.5 cycles/px\n");
    fprintf(f, "levels=%d\n", np->levels);
    for (int c = 0; c < 3; c++) {
        fprintf(f, "mean_%c=", "rgb"[c]);
        for (int i = 0; i < np->levels; i++) fprintf(f, "%s%.6g", i ? " " : "", np->mean[c][i]);
        fprintf(f, "\nvar_%c=", "rgb"[c]);
        for (int i = 0; i < np->levels; i++) fprintf(f, "%s%.6g", i ? " " : "", np->var[c][i]);
        fputc('\n', f);
    }
    fprintf(f, "shape_x=");
    for (int k = 0; k < DSD4_NPS_BINS; k++) fprintf(f, "%s%.4f", k ? " " : "", np->shape_x[k]);
    fprintf(f, "\nshape_y=");
    for (int k = 0; k < DSD4_NPS_BINS; k++) fprintf(f, "%s%.4f", k ? " " : "", np->shape_y[k]);
    fputc('\n', f);
    int ok = !ferror(f);
    return fclose(f) == 0 && ok ? DSD4_OK : DSD4_ERR_IO;
}

static int read_list(const char *s, double *v, int n)
{
    int k = 0;
    for (; k < n; k++) {
        char *e;
        v[k] = strtod(s, &e);
        if (e == s) break;
        s = e;
    }
    return k;
}

static int noise_parse(FILE *f, dsd4_noise_profile *np)
{
    memset(np, 0, sizeof *np);
    char line[2048];
    int got = 0;
    while (fgets(line, sizeof line, f)) {
        char *v = strchr(line, '=');
        if (!v || line[0] == '#') continue;
        *v++ = 0;
        if (!strcmp(line, "levels")) np->levels = atoi(v);
        else if (!strcmp(line, "shape_x")) got |= (read_list(v, np->shape_x, DSD4_NPS_BINS) == DSD4_NPS_BINS) << 6;
        else if (!strcmp(line, "shape_y")) got |= (read_list(v, np->shape_y, DSD4_NPS_BINS) == DSD4_NPS_BINS) << 7;
        else
            for (int c = 0; c < 3; c++) {
                char key[8];
                snprintf(key, sizeof key, "mean_%c", "rgb"[c]);
                if (!strcmp(line, key)) got |= (read_list(v, np->mean[c], DSD4_NOISE_LEVELS) > 0) << c;
                snprintf(key, sizeof key, "var_%c", "rgb"[c]);
                if (!strcmp(line, key)) got |= (read_list(v, np->var[c], DSD4_NOISE_LEVELS) > 0) << (3 + c);
            }
    }
    np->valid = got == 0xff && np->levels > 0 && np->levels <= DSD4_NOISE_LEVELS;
    return np->valid ? DSD4_OK : DSD4_ERR_PARAM;
}

int dsd4_noise_read(const char *path, dsd4_noise_profile *np)
{
    memset(np, 0, sizeof *np);
    FILE *f = fopen(path, "r");
    if (!f) return DSD4_ERR_IO;
    int r = noise_parse(f, np);
    fclose(f);
    return r;
}

int dsd4_noise_default(dsd4_noise_profile *np)
{
    FILE *f = fmemopen((void *)dsd4i_default_noise, strlen(dsd4i_default_noise), "r");
    if (!f) return DSD4_ERR_NOMEM;
    int r = noise_parse(f, np);
    fclose(f);
    return r;
}
