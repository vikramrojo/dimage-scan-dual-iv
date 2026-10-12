/*
 * dsdual4 - command line front end for the DiMAGE Scan Dual IV driver.
 */
#include "dsdual4.h"
#include "../src/transport.h"
#include "../src/escl.h"

#include <getopt.h>
#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int g_verbose = 1;
static int g_progress_tty;

static void usage(void)
{
    fprintf(stderr,
"usage: dsdual4 [global options] <command> [command options]\n"
"\n"
"commands:\n"
"  info                      scanner identification and capabilities\n"
"  holder                    show the inserted film holder\n"
"  init                      warm up the lamp and calibrate\n"
"  frame <n>                 move the holder to frame n\n"
"  eject                     eject the film holder\n"
"  home                      return the carriage to its home position\n"
"  focus <pos>               move the focus motor (-200..200)\n"
"  autofocus [--frame n]     run the original autofocus sequence\n"
"  preview -o file.tiff      800 dpi preview of the current/selected frame\n"
"  scan -o file.tiff         scan one frame\n"
"  scan-all -o prefix        scan every frame of the holder (prefix-N.tiff)\n"
"  serve [--port n] [--film type] [--local] [--wait]\n"
"                            AirScan server: makes the scanner available in\n"
"                            Image Capture and Preview (settings page at /);\n"
"                            --wait: wait until the scanner is connected\n"
"\n"
"global options:\n"
"  --sim                     use the built-in protocol simulator instead of USB\n"
"  --firmware <file>         DS_Dual4.BIN used for the firmware check\n"
"  --no-firmware-check       never download firmware\n"
"  -v / -q                   more / less output (repeat -v for protocol traces)\n"
"\n"
"scan options:\n"
"  --frame <n>               frame number (default: current)\n"
"  --dpi <n>                 output resolution 50..3200 (default 3200)\n"
"  --bits <8|16>             bits per channel (default 16)\n"
"  --film <type>             posi, nega, bw-posi, bw-nega (default posi)\n"
"  --exposure <r,g,b|f>      exposure factors (default: film standard)\n"
"  --ev <stops>              exposure correction applied to all channels\n"
"  --focus <pos|auto|keep>   focus position (default auto)\n"
"  --area <x,y,w,h>          crop in pixels at the output resolution\n"
"  --samples <1|2|4|8>       multi-sample scanning (default 1)\n"
"  --invert                  invert negatives (auto levels per channel)\n"
"  --raw                     no host-side digital gain\n"
"  --flip-x / --flip-y       mirror the image\n"
"  --rotate <90|180|270>     rotate clockwise\n");
}

static void on_log(void *ud, int level, const char *msg)
{
    (void)ud;
    if (g_progress_tty) fputs("\r\033[K", stderr);
    fprintf(stderr, "%s%s\n", level <= 1 ? "" : "  ", msg);
}

static void on_progress(void *ud, const char *stage, double f)
{
    (void)ud;
    if (g_verbose < 1) return;
    if (g_progress_tty) {
        if (f >= 0) fprintf(stderr, "\r\033[K%s %3.0f%%", stage, f * 100);
        else fprintf(stderr, "\r\033[K%s...", stage);
        fflush(stderr);
    } else if (f < 0) {
        fprintf(stderr, "%s...\n", stage);
    }
}

static void done_progress(void)
{
    if (g_progress_tty && g_verbose >= 1) fputs("\r\033[K", stderr);
}

static const char *find_firmware(const char *argv0)
{
    static char buf[PATH_MAX];
    const char *env = getenv("DSD4_FIRMWARE");
    if (env && access(env, R_OK) == 0) return env;
    const char *cands[] = {
        "DS_Dual4.BIN",
        "firmware/DS_Dual4.BIN",
        "../DS_Dual4_extracted/DS Dual4/Firmware/DS_Dual4.BIN",
        "/usr/local/share/dsdual4/DS_Dual4.BIN",
        "/opt/homebrew/share/dsdual4/DS_Dual4.BIN",
    };
    for (size_t i = 0; i < sizeof cands / sizeof *cands; i++)
        if (access(cands[i], R_OK) == 0) return cands[i];
    const char *slash = strrchr(argv0, '/');
    if (slash) {
        snprintf(buf, sizeof buf, "%.*s/../share/dsdual4/DS_Dual4.BIN", (int)(slash - argv0), argv0);
        if (access(buf, R_OK) == 0) return buf;
    }
    return NULL;
}

static int parse_film(const char *s, dsd4_film *f)
{
    if (!strcmp(s, "posi") || !strcmp(s, "slide") || !strcmp(s, "positive")) *f = DSD4_FILM_POSITIVE_COLOR;
    else if (!strcmp(s, "nega") || !strcmp(s, "negative")) *f = DSD4_FILM_NEGATIVE_COLOR;
    else if (!strcmp(s, "bw-posi")) *f = DSD4_FILM_POSITIVE_BW;
    else if (!strcmp(s, "bw-nega") || !strcmp(s, "bw")) *f = DSD4_FILM_NEGATIVE_BW;
    else return -1;
    return 0;
}

static int check(int r, dsd4 *s, const char *what)
{
    if (r == DSD4_OK) return 0;
    done_progress();
    if (r == DSD4_ERR_SCANNER || r == DSD4_ERR_IO)
        fprintf(stderr, "dsdual4: %s: %s (scanner code 0x%02x: %s)\n", what, dsd4_strerror(r),
                dsd4_last_error(s), dsd4_scanner_error_text(dsd4_last_error(s)));
    else
        fprintf(stderr, "dsdual4: %s: %s\n", what, dsd4_strerror(r));
    return 1;
}

/* geometric post-processing */
static void flip_x(dsd4_image *im)
{
    size_t bpp = im->stride / im->width;
    uint8_t tmp[8];
    for (int y = 0; y < im->height; y++) {
        uint8_t *row = im->data + (size_t)y * im->stride;
        for (int a = 0, b = im->width - 1; a < b; a++, b--) {
            memcpy(tmp, row + a * bpp, bpp);
            memcpy(row + a * bpp, row + b * bpp, bpp);
            memcpy(row + b * bpp, tmp, bpp);
        }
    }
}

static void flip_y(dsd4_image *im)
{
    uint8_t *tmp = malloc(im->stride);
    if (!tmp) return;
    for (int a = 0, b = im->height - 1; a < b; a++, b--) {
        memcpy(tmp, im->data + (size_t)a * im->stride, im->stride);
        memcpy(im->data + (size_t)a * im->stride, im->data + (size_t)b * im->stride, im->stride);
        memcpy(im->data + (size_t)b * im->stride, tmp, im->stride);
    }
    free(tmp);
}

static int rotate90(dsd4_image *im)
{
    size_t bpp = im->stride / im->width;
    int nw = im->height, nh = im->width;
    uint8_t *d = malloc((size_t)nw * nh * bpp);
    if (!d) return -1;
    for (int y = 0; y < im->height; y++)
        for (int x = 0; x < im->width; x++)
            memcpy(d + ((size_t)x * nw + (nw - 1 - y)) * bpp, im->data + (size_t)y * im->stride + x * bpp, bpp);
    free(im->data);
    im->data = d;
    im->width = nw;
    im->height = nh;
    im->stride = (size_t)nw * bpp;
    return 0;
}

typedef struct {
    dsd4_scan_params p;
    const char *out;
    int invert, fx, fy, rot;
    double ev;
} scan_opts;

static int parse_scan_opts(int argc, char **argv, scan_opts *o)
{
    static const struct option lo[] = {
        { "frame", 1, 0, 'f' }, { "dpi", 1, 0, 'd' }, { "bits", 1, 0, 'b' }, { "film", 1, 0, 'F' },
        { "exposure", 1, 0, 'e' }, { "ev", 1, 0, 'E' }, { "focus", 1, 0, 'c' }, { "area", 1, 0, 'a' },
        { "samples", 1, 0, 's' }, { "invert", 0, 0, 'i' }, { "raw", 0, 0, 'r' }, { "flip-x", 0, 0, 'X' },
        { "flip-y", 0, 0, 'Y' }, { "rotate", 1, 0, 'R' }, { "output", 1, 0, 'o' }, { 0, 0, 0, 0 },
    };
    optind = 1;
    int c;
    while ((c = getopt_long(argc, argv, "o:", lo, NULL)) != -1) {
        switch (c) {
        case 'f': o->p.frame = atoi(optarg); break;
        case 'd': o->p.dpi = atoi(optarg); break;
        case 'b': o->p.bits = atoi(optarg); break;
        case 'F': if (parse_film(optarg, &o->p.film)) { fprintf(stderr, "bad film type %s\n", optarg); return -1; } break;
        case 'e': {
            double r, g, b;
            int n = sscanf(optarg, "%lf,%lf,%lf", &r, &g, &b);
            if (n == 1) g = b = r;
            else if (n != 3) { fprintf(stderr, "bad exposure %s\n", optarg); return -1; }
            o->p.exposure[0] = r; o->p.exposure[1] = g; o->p.exposure[2] = b;
            break;
        }
        case 'E': o->ev = atof(optarg); break;
        case 'c':
            if (!strcmp(optarg, "auto")) o->p.focus = DSD4_FOCUS_AUTO;
            else if (!strcmp(optarg, "keep")) o->p.focus = DSD4_FOCUS_KEEP;
            else o->p.focus = atoi(optarg);
            break;
        case 'a':
            if (sscanf(optarg, "%d,%d,%d,%d", &o->p.x, &o->p.y, &o->p.w, &o->p.h) != 4) {
                fprintf(stderr, "bad area %s\n", optarg);
                return -1;
            }
            break;
        case 's': o->p.multi_sample = atoi(optarg); break;
        case 'i': o->invert = 1; break;
        case 'r': o->p.raw = 1; break;
        case 'X': o->fx = 1; break;
        case 'Y': o->fy = 1; break;
        case 'R': o->rot = atoi(optarg); break;
        case 'o': o->out = optarg; break;
        default: return -1;
        }
    }
    return 0;
}

static int do_scan(dsd4 *s, scan_opts *o, const char *path)
{
    dsd4_scan_params p = o->p;
    if (o->ev != 0) {
        double std[3];
        dsd4_film_default_exposure(p.film, std);
        for (int c = 0; c < 3; c++) {
            double base = p.exposure[c] > 0 ? p.exposure[c] : std[c];
            p.exposure[c] = base * pow(2.0, o->ev);
        }
    }
    dsd4_image im;
    int r = dsd4_scan(s, &p, &im, on_progress, NULL);
    done_progress();
    if (check(r, s, p.preview ? "preview" : "scan")) return 1;
    if (o->invert) dsd4_invert_negative(&im, 1);
    if (o->fx) flip_x(&im);
    if (o->fy) flip_y(&im);
    if (o->rot == 90 || o->rot == 180 || o->rot == 270) {
        if (o->rot == 180) { flip_x(&im); flip_y(&im); }
        else {
            rotate90(&im);
            if (o->rot == 270) { flip_x(&im); flip_y(&im); }
        }
    }
    r = dsd4_write_tiff(&im, path);
    if (g_verbose >= 1)
        fprintf(stderr, "wrote %s: %dx%d, %d-bit %s, %d dpi, focus %d, digital gain %.2f/%.2f/%.2f\n",
                path, im.width, im.height, im.bits, im.channels == 3 ? "RGB" : "gray", im.dpi, im.focus,
                im.digital_gain[0], im.digital_gain[1], im.digital_gain[2]);
    dsd4_image_free(&im);
    return check(r, s, "write");
}

int main(int argc, char **argv)
{
    int use_sim = 0, fw_check = 1;
    const char *fw = NULL;
    int i = 1;
    for (; i < argc && argv[i][0] == '-'; i++) {
        if (!strcmp(argv[i], "--sim")) use_sim = 1;
        else if (!strcmp(argv[i], "--no-firmware-check")) fw_check = 0;
        else if (!strcmp(argv[i], "--firmware") && i + 1 < argc) fw = argv[++i];
        else if (!strncmp(argv[i], "-v", 2)) g_verbose += (int)strlen(argv[i]) - 1;
        else if (!strcmp(argv[i], "-q")) g_verbose = 0;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(); return 0; }
        else { fprintf(stderr, "unknown option %s\n", argv[i]); usage(); return 2; }
    }
    if (i >= argc) { usage(); return 2; }
    const char *cmd = argv[i];
    int cargc = argc - i;
    char **cargv = argv + i;
    g_progress_tty = isatty(2);
    if (!fw) fw = find_firmware(argv[0]);

    char err[256] = "";
    dsd4 *s = NULL;
    int wait = 0;
    for (int k = 1; k < cargc; k++) if (!strcmp(cargv[k], "--wait")) wait = 1;
    if (use_sim) {
        dsd4_transport *t = dsd4_sim_open(fw, err, sizeof err);
        if (!t || dsd4_open_transport(&s, t)) { fprintf(stderr, "dsdual4: %s\n", err); return 1; }
    } else {
        int said = 0;
        while (dsd4_open_usb(&s, err, sizeof err)) {
            if (!wait) { fprintf(stderr, "dsdual4: %s\n", err); return 1; }
            if (!said++) fprintf(stderr, "dsdual4: waiting for the scanner (%s)\n", err);
            sleep(5);
        }
    }
    dsd4_set_log(s, on_log, NULL, g_verbose);
    if (fw) dsd4_set_firmware_path(s, fw);

    int rc = 0;
    if (!strcmp(cmd, "info")) {
        dsd4_info in;
        if (check(dsd4_get_info(s, &in), s, "inquiry")) rc = 1;
        else {
            printf("vendor      %s\nproduct     %s\nfirmware    %s\noptical     %d dpi\nmax area    %d x %d px\n",
                   in.vendor, in.product, in.revision, in.optical_dpi, in.max_pixels, in.max_lines);
            dsd4_holder_info h;
            if (dsd4_get_holder(s, &h) == DSD4_OK || h.raw_type == 0xff) {
                static const char *names[] = { "none", "35 mm strip", "35 mm slide", "APS", "unknown" };
                printf("holder      %s (type %d, %d frames, at frame %d)\n", names[h.holder], h.raw_type,
                       h.frames, h.current_frame);
            }
            if (g_verbose >= 2) {
                printf("inquiry    ");
                for (int k = 0; k < 96; k++) printf("%s%02x", k % 16 ? " " : "\n  ", in.inquiry[k]);
                printf("\n");
            }
        }
    } else if (!strcmp(cmd, "holder")) {
        dsd4_holder_info h;
        int r = dsd4_get_holder(s, &h);
        if (r && r != DSD4_ERR_NO_HOLDER) rc = check(r, s, "holder");
        else {
            static const char *names[] = { "none", "35 mm strip", "35 mm slide", "APS", "unknown" };
            printf("%s, %d frames, current %d\n", names[h.holder], h.frames, h.current_frame);
        }
    } else if (!strcmp(cmd, "init")) {
        rc = check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize");
        done_progress();
        if (!rc) fprintf(stderr, "scanner ready\n");
    } else if (!strcmp(cmd, "frame") && cargc > 1) {
        rc = check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize") ||
             check(dsd4_select_frame(s, atoi(cargv[1])), s, "frame");
    } else if (!strcmp(cmd, "eject")) {
        rc = check(dsd4_eject(s), s, "eject");
    } else if (!strcmp(cmd, "home")) {
        rc = check(dsd4_home(s), s, "home");
    } else if (!strcmp(cmd, "focus") && cargc > 1) {
        rc = check(dsd4_set_focus(s, atoi(cargv[1])), s, "focus");
    } else if (!strcmp(cmd, "autofocus")) {
        scan_opts o = { 0 };
        dsd4_default_params(&o.p);
        if (parse_scan_opts(cargc, cargv, &o)) rc = 2;
        else if (!(rc = check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize"))) {
            if (o.p.frame && (rc = check(dsd4_select_frame(s, o.p.frame), s, "frame"))) goto out;
            int best = 0;
            rc = check(dsd4_autofocus(s, &o.p, &best, on_progress, NULL), s, "autofocus");
            done_progress();
            if (!rc) printf("best focus position: %d\n", best);
        }
    } else if (!strcmp(cmd, "preview") || !strcmp(cmd, "scan")) {
        scan_opts o = { 0 };
        dsd4_default_params(&o.p);
        o.p.preview = !strcmp(cmd, "preview");
        if (parse_scan_opts(cargc, cargv, &o) || !o.out) { usage(); rc = 2; }
        else if (!(rc = check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize")))
            rc = do_scan(s, &o, o.out);
    } else if (!strcmp(cmd, "scan-all")) {
        scan_opts o = { 0 };
        dsd4_default_params(&o.p);
        if (parse_scan_opts(cargc, cargv, &o) || !o.out) { usage(); rc = 2; }
        else if (!(rc = check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize"))) {
            dsd4_holder_info h;
            if (!(rc = check(dsd4_get_holder(s, &h), s, "holder"))) {
                for (int f = 1; f <= h.frames && !rc; f++) {
                    char path[PATH_MAX];
                    snprintf(path, sizeof path, "%s-%d.tiff", o.out, f);
                    o.p.frame = f;
                    if (g_verbose >= 1) fprintf(stderr, "frame %d/%d\n", f, h.frames);
                    rc = do_scan(s, &o, path);
                }
            }
        }
    } else if (!strcmp(cmd, "serve")) {
        dsd4_escl_options eo = { .port = 8090, .film = DSD4_FILM_POSITIVE_COLOR };
        for (int k = 1; k < cargc; k++) {
            if (!strcmp(cargv[k], "--port") && k + 1 < cargc) eo.port = atoi(cargv[++k]);
            else if (!strcmp(cargv[k], "--name") && k + 1 < cargc) eo.name = cargv[++k];
            else if (!strcmp(cargv[k], "--local")) eo.localhost_only = 1;
            else if (!strcmp(cargv[k], "--wait")) continue;
            else if (!strcmp(cargv[k], "--film") && k + 1 < cargc) {
                if (parse_film(cargv[++k], &eo.film)) { fprintf(stderr, "bad film type\n"); rc = 2; goto out; }
            } else { usage(); rc = 2; goto out; }
        }
        if (!(rc = check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize"))) {
            done_progress();
            rc = dsd4_escl_serve(s, &eo) ? 1 : 0;
        }
    } else {
        usage();
        rc = 2;
    }
out:
    dsd4_close(s);
    return rc;
}
