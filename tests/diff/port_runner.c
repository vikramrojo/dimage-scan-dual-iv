/* Runs scenario.txt against libdsd4 and prints a canonical trace of
 * everything sent to / received from the (simulated) scanner. */
#include "dsd4_internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static dsd4_transport *g_inner;

static uint64_t fnv(const uint8_t *p, size_t n)
{
    uint64_t h = 1469598103934665603ull;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 1099511628211ull; }
    return h;
}

static int rec_exec(dsd4_transport *t, const uint8_t cdb[10], enum dsd4_dir dir, void *data, size_t len, uint8_t *st)
{
    int r = g_inner->exec(g_inner, cdb, dir, data, len, st);
    printf("CMD");
    for (int i = 0; i < 10; i++) printf(" %02x", cdb[i]);
    printf(" dir=%d len=%zu", dir, len);
    if (dir == DSD4_DIR_OUT) {
        printf(" out=");
        for (size_t i = 0; i < len; i++) printf("%02x", ((uint8_t *)data)[i]);
    } else if (dir == DSD4_DIR_IN) {
        printf(" in#%016llx", (unsigned long long)fnv(data, len));
    }
    printf(" st=%u\n", *st);
    return r;
}

static void rec_close(dsd4_transport *t) { g_inner->close(g_inner); free(t); }

int main(int argc, char **argv)
{
    const char *scen = argc > 1 ? argv[1] : "scenario.txt";
    const char *fw = argc > 2 ? argv[2] : NULL;
    setenv("DSD4_NO_CACHE", "1", 1);
    char err[128];
    g_inner = dsd4_sim_open(fw, err, sizeof err);
    dsd4_transport *t = calloc(1, sizeof *t);
    t->exec = rec_exec;
    t->close = rec_close;
    dsd4 *s;
    dsd4_open_transport(&s, t);
    s->verbose = 0;
    FILE *f = fopen(scen, "r");
    if (!f) { perror(scen); return 1; }
    char line[512];
    static uint8_t buf[1 << 22];
    while (fgets(line, sizeof line, f)) {
        char cmd[32] = "";
        if (line[0] == '#' || sscanf(line, "%31s", cmd) != 1) continue;
        if (!strcmp(cmd, "INIT")) {
            printf("== INIT\n");
            avl_initialize_driver(s);
            s->speed_mode = 1;
            printf("ret %d\n", avl_find_scanner(s));
            uint8_t st = 0;
            printf("ret %d state %u\n", avl_check_scanner_state(s, 2, &st), st);
            printf("ret %d\n", avl_get_scanner_ability(s));
        } else if (!strcmp(cmd, "CAL")) {
            printf("== CAL\n");
            printf("ret %d\n", avl_do_calibration(s));
        } else if (!strcmp(cmd, "SP")) {
            char name[32];
            unsigned v[23];
            int fl;
            if (sscanf(line, "SP %31s %u %u %u %u %u %u %u %u %i %u %u %u %u %u %i %u %u %u %u %u %u %u %u",
                       name, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &fl, &v[9], &v[10], &v[11],
                       &v[12], &v[13], (int *)&v[14], &v[15], &v[16], &v[17], &v[18], &v[19], &v[20], &v[21], &v[22]) != 24) {
                fprintf(stderr, "bad SP line\n");
                return 1;
            }
            sp_t p;
            memset(&p, 0, sizeof p);
            p.ulx = v[0]; p.uly = v[1]; p.width = v[2]; p.length = v[3]; p.pixels = v[4]; p.lines = v[5];
            p.mode = v[6]; p.bits = v[7]; p.flags = (uint16_t)fl; p.exp_r = v[9]; p.exp_g = v[10]; p.exp_b = v[11];
            p.xres = v[12]; p.yres = v[13]; p.focus = (int16_t)(int)v[14]; p.multi_lines = v[15]; p.multi_scan = v[16];
            p.multi_sample = v[17]; p.is_aps = v[18]; p.final_scan = v[19]; p.nonstd_dark = v[20];
            p.halftone = 1; p.highlight = 0xff; p.exp_ir = 100;
            printf("== SP %s\n", name);
            int ok = avl_set_scan_parameter(s, &p);
            printf("ret %d\n", ok);
            if (!ok) continue;
            printf("ret %d\n", avl_start_scan(s));
            uint32_t lb = s->gray_from_color ? s->line_bytes / 3u : s->line_bytes, total = v[21], chunk = v[22], done = 0;
            while (done < total) {
                uint32_t n = total - done < chunk ? total - done : chunk;
                int r = avl_read_scan(s, buf, n * lb);
                printf("IMG %d %u #%016llx\n", r, n * lb, (unsigned long long)fnv(buf, n * lb));
                done += n;
            }
            printf("ret %d\n", avl_stop_scan(s));
        } else if (!strcmp(cmd, "DARK")) {
            double e[3];
            sscanf(line, "DARK %lf %lf %lf", &e[0], &e[1], &e[2]);
            printf("== DARK\n");
            printf("ret %d\n", avl_dark_calibration(s, e));
        } else if (!strcmp(cmd, "FOCUS")) {
            int pos;
            sscanf(line, "FOCUS %d", &pos);
            printf("== FOCUS %d\n", pos);
            printf("ret %d\n", avl_set_optical_control(s, (int16_t)pos));
        } else if (!strcmp(cmd, "FOCUSSCAN")) {
            unsigned x, y, w, cnt;
            int pos[8], np = 0, off = 0;
            sscanf(line, "FOCUSSCAN %u %u %u %u%n", &x, &y, &w, &cnt, &off);
            char *q = line + off;
            int k;
            while (np < 8 && sscanf(q, "%d%n", &pos[np], &k) == 1) { q += k; np++; }
            printf("== FOCUSSCAN\n");
            sp_t p;
            memset(&p, 0, sizeof p);
            p.ulx = x; p.uly = y; p.width = w; p.length = 1; p.pixels = w; p.lines = 1;
            p.mode = 4; p.bits = 48; p.flags = (0x9048 & 0xf7ff) | 0x10a; p.exp_r = p.exp_g = p.exp_b = 152;
            p.xres = p.yres = 3200; p.focus = (int16_t)pos[0]; p.multi_lines = 1; p.multi_scan = (uint8_t)cnt;
            p.focus_mode = 1; p.halftone = 1; p.highlight = 0xff; p.exp_ir = 100;
            printf("ret %d\n", avl_set_scan_parameter(s, &p));
            printf("ret %d\n", avl_start_scan(s));
            for (int i = 0; i < np; i++) {
                printf("ret %d\n", avl_set_optical_control(s, (int16_t)pos[i]));
                for (unsigned c = 0; c < cnt; c++) {
                    int r = avl_read_scan(s, buf, s->line_bytes);
                    printf("IMG %d %u #%016llx\n", r, s->line_bytes, (unsigned long long)fnv(buf, s->line_bytes));
                }
            }
            printf("ret %d\n", avl_stop_scan(s));
        } else if (!strcmp(cmd, "HOLDER")) {
            int n;
            sscanf(line, "HOLDER %d", &n);
            printf("== HOLDER %d\n", n);
            printf("ret %d\n", avl_send_holder_control(s, (uint8_t)n));
        } else if (!strcmp(cmd, "FEED")) {
            printf("== FEED\n");
            printf("ret %d\n", avl_paper_feed(s));
        } else if (!strcmp(cmd, "SETSTATE")) {
            int n;
            sscanf(line, "SETSTATE %d", &n);
            uint8_t st = (uint8_t)n;
            printf("== SETSTATE %d\n", n);
            printf("ret %d\n", avl_check_scanner_state(s, 1, &st));
        } else if (!strcmp(cmd, "INITSCANNER")) {
            printf("== INITSCANNER\n");
            printf("ret %d\n", avl_initialize_scanner(s));
        } else if (!strcmp(cmd, "FWDL")) {
            int dl = 0;
            printf("== FWDL\n");
            printf("ret %d\n", fw ? avl_download_firmware(s, fw, &dl) : 0);
        } else if (!strcmp(cmd, "RESET")) {
            printf("== RESET\n");
            printf("ret %d\n", avl_reset_to_initial_state(s, 0));
        }
    }
    dsd4_close(s);
    return 0;
}
