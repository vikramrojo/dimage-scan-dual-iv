/*
 * "dsdual4 research": measurements that decide whether multi-pass
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
    fprintf(report, "dsdual4 research: film type %d, focus %d\n", (int)r.base.film, r.base.focus);
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
