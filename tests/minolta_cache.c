/* A Minolta preview followed by a Minolta scan of the same frame must reuse
 * the preview's auto-exposure (no second prescan) and produce the same AE
 * exposure as a scan without a preview. */
#include "dsd4.h"
#include "../src/transport.h"

#include <stdio.h>
#include <string.h>

static int reused;
static void on_log(void *ud, int level, const char *msg)
{
    (void)ud; (void)level;
    if (strstr(msg, "(from the preview)")) reused++;
}

static int session(int with_preview, double exp_out[3], int *prescans)
{
    char err[256];
    dsd4 *s;
    dsd4_transport *t = dsd4_sim_open(NULL, err, sizeof err);
    if (!t || dsd4_open_transport(&s, t)) return 1;
    dsd4_set_log(s, on_log, NULL, 1);
    if (dsd4_initialize(s, 0, NULL, NULL)) return 1;
    dsd4_scan_params p;
    dsd4_default_params(&p);
    p.film = DSD4_FILM_NEGATIVE_COLOR;
    p.frame = 2;
    p.focus = 37;
    p.dpi = 400;
    dsd4_image im;
    dsd4_minolta_info mi;
    reused = 0;
    if (with_preview) {
        if (dsd4_preview_minolta(s, &p, &im, &mi, NULL, NULL)) return 1;
        dsd4_image_free(&im);
    }
    if (dsd4_scan_minolta(s, &p, &im, &mi, NULL, NULL)) return 1;
    dsd4_image_free(&im);
    memcpy(exp_out, mi.exposure, sizeof mi.exposure);
    *prescans = reused ? 0 : 1;
    dsd4_close(s);
    return 0;
}

int main(void)
{
    double a[3], b[3];
    int pa, pb;
    if (session(1, a, &pa) || session(0, b, &pb)) { printf("FAIL minolta-preview-reuse: scan failed\n"); return 1; }
    int same = 1;
    for (int c = 0; c < 3; c++) same &= a[c] == b[c];
    if (pa == 0 && pb == 1 && same) { printf("ok   minolta-preview-reuse\n"); return 0; }
    printf("FAIL minolta-preview-reuse: prescans %d/%d, exposure %.4f/%.4f/%.4f vs %.4f/%.4f/%.4f\n", pa, pb, a[0],
           a[1], a[2], b[0], b[1], b[2]);
    return 1;
}
