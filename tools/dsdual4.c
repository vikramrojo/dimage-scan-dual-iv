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
#include <sys/stat.h>
#include <time.h>

static int g_verbose = 1;
static int g_last_focus = DSD4_FOCUS_AUTO; /* focus of the last scan / autofocus (AUTO = none yet) */
static uint8_t *g_icc;      /* scanner input profile (MLTF3200p.icc, "Posi Linear") */
static size_t g_icc_len;
static char g_icc_path[PATH_MAX];
static uint8_t *g_icc_n;    /* MLTF3200n.icc ("Nega Input") for Minolta-processed negatives */
static size_t g_icc_n_len;
static int g_progress_tty;

static void usage(void)
{
    fprintf(stderr,
"usage: dsdual4                      interactive menu with instructions\n"
"       dsdual4 [global options] <command> [command options]\n"
"\n"
"commands:\n"
"  info                      scanner identification and capabilities\n"
"  holder                    show the inserted film holder\n"
"  init                      warm up the lamp and calibrate\n"
"  frame <n>                 move the holder to frame n\n"
"  eject                     eject the film holder\n"
"  home                      return the carriage to its home position\n"
"  reset                     reset the scanner and wait until it is ready\n"
"  focus <pos>               move the focus motor (-200..200)\n"
"  autofocus [--frame n]     run the original autofocus sequence\n"
"  preview -o file.tiff      800 dpi preview of the current/selected frame\n"
"  scan -o file.tiff         scan one frame\n"
"  scan-all -o prefix        scan every frame of the holder (prefix-N.tiff)\n"
"  convert IN.tiff -o OUT.tiff --film bw-nega|nega [--scan-exposure f]\n"
"                            apply the original Minolta negative processing to\n"
"                            an existing raw negative scan (no scanner needed)\n"
"  serve [--port n] [--film type] [--local] [--wait]\n"
"                            AirScan server: makes the scanner available in\n"
"                            Image Capture and Preview (settings page at /);\n"
"                            --wait: wait until the scanner is connected\n"
"\n"
"global options:\n"
"  --sim                     use the built-in protocol simulator instead of USB\n"
"  --firmware <file>         DS_Dual4.BIN used for the firmware check\n"
"  --profile <file.icc>      scanner input profile to embed in linear colour\n"
"                            slide scans (default: MLTF3200p.icc, found next to\n"
"                            the original software or in the install prefix)\n"
"  --no-profile              do not embed a profile\n"
"  --firmware-update         allow sending DS_Dual4.BIN if the scanner asks\n"
"                            for it (off by default; never needed to scan)\n"
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
"  --minolta                 negatives: the original Minolta auto-exposure and\n"
"                            film curves (prescan + positive result)\n"
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

/* Konica Minolta's input profile for linear positive scans, read from the
 * user's copy of the original software (not distributed with dsdual4). */
static void load_profile(const char *argv0, const char *explicit_path)
{
    char buf[PATH_MAX];
    const char *cands[8];
    int n = 0;
    if (explicit_path) cands[n++] = explicit_path;
    else {
        const char *env = getenv("DSD4_PROFILE");
        if (env) cands[n++] = env;
        cands[n++] = "MLTF3200p.icc";
        cands[n++] = "../DS_Dual4_extracted/DS Dual4/Profile/MLTF3200p.icc";
        cands[n++] = "/usr/local/share/dsdual4/MLTF3200p.icc";
        cands[n++] = "/opt/homebrew/share/dsdual4/MLTF3200p.icc";
        const char *slash = strrchr(argv0, '/');
        if (slash) {
            snprintf(buf, sizeof buf, "%.*s/../share/dsdual4/MLTF3200p.icc", (int)(slash - argv0), argv0);
            cands[n++] = buf;
        }
    }
    for (int i = 0; i < n; i++) {
        FILE *f = fopen(cands[i], "rb");
        if (!f) continue;
        uint8_t hdr[132];
        fseek(f, 0, SEEK_END);
        long len = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (len < 132 || len > (1 << 20) || fread(hdr, 1, 132, f) != 132 || memcmp(hdr + 36, "acsp", 4)) {
            fclose(f);
            if (explicit_path) fprintf(stderr, "dsdual4: %s is not an ICC profile\n", cands[i]);
            continue;
        }
        g_icc = malloc((size_t)len);
        fseek(f, 0, SEEK_SET);
        if (g_icc && fread(g_icc, 1, (size_t)len, f) == (size_t)len) {
            g_icc_len = (size_t)len;
            snprintf(g_icc_path, sizeof g_icc_path, "%s", cands[i]);
        } else {
            free(g_icc);
            g_icc = NULL;
        }
        fclose(f);
        if (g_icc) {
            /* the negative profile lives next to the positive one */
            char np[PATH_MAX];
            snprintf(np, sizeof np, "%s", g_icc_path);
            char *q = strstr(np, "MLTF3200p.icc");
            if (q) {
                memcpy(q, "MLTF3200n.icc", 13);
                FILE *fn = fopen(np, "rb");
                if (fn) {
                    fseek(fn, 0, SEEK_END);
                    long nl = ftell(fn);
                    fseek(fn, 0, SEEK_SET);
                    if (nl > 132 && nl < (1 << 20) && (g_icc_n = malloc((size_t)nl)) &&
                        fread(g_icc_n, 1, (size_t)nl, fn) == (size_t)nl)
                        g_icc_n_len = (size_t)nl;
                    else { free(g_icc_n); g_icc_n = NULL; }
                    fclose(fn);
                }
            }
            return;
        }
    }
    if (explicit_path) fprintf(stderr, "dsdual4: cannot read profile %s\n", explicit_path);
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
    int code = dsd4_last_error(s);
    if (r == DSD4_ERR_SCANNER && (code == 0x67 || code == 0x68 || code == 0x69 || code == 0xd1)) {
        fprintf(stderr, "dsdual4: %s: the scanner reports a film holder problem (code 0x%02x: %s).\n"
                "  Check that the front door is closed and no holder is half inserted (push it\n"
                "  in until the scanner grabs it, or pull it out completely), then retry.\n"
                "  If it persists, switch the scanner off and on, wait for the light to stop\n"
                "  blinking, and run init again ('dsdual4 reset' may also help).\n",
                what, code, dsd4_scanner_error_text(code));
        return 1;
    }
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
    int display;      /* 8-bit sRGB-like gamma output for viewing */
    int minolta;      /* negatives: original Minolta processing */
    int no_profile;   /* do not embed the scanner profile */
} scan_opts;

static int parse_scan_opts(int argc, char **argv, scan_opts *o)
{
    static const struct option lo[] = {
        { "frame", 1, 0, 'f' }, { "dpi", 1, 0, 'd' }, { "bits", 1, 0, 'b' }, { "film", 1, 0, 'F' },
        { "exposure", 1, 0, 'e' }, { "ev", 1, 0, 'E' }, { "focus", 1, 0, 'c' }, { "area", 1, 0, 'a' },
        { "samples", 1, 0, 's' }, { "invert", 0, 0, 'i' }, { "raw", 0, 0, 'r' }, { "flip-x", 0, 0, 'X' },
        { "flip-y", 0, 0, 'Y' }, { "rotate", 1, 0, 'R' }, { "output", 1, 0, 'o' }, { "minolta", 0, 0, 'M' },
        { 0, 0, 0, 0 },
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
        case 'M': o->minolta = 1; break;
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
    dsd4_minolta_info mi;
    const int minolta = o->minolta && !p.preview &&
                        (p.film == DSD4_FILM_NEGATIVE_COLOR || p.film == DSD4_FILM_NEGATIVE_BW);
    int r = minolta ? dsd4_scan_minolta(s, &p, &im, &mi, on_progress, NULL)
                    : dsd4_scan(s, &p, &im, on_progress, NULL);
    done_progress();
    if (check(r, s, p.preview ? "preview" : "scan")) return 1;
    if (minolta && g_verbose >= 1)
        fprintf(stderr, "Minolta processing: AE exposure %.3f/%.3f/%.3f, black level %d/%d/%d\n", mi.exposure[0],
                mi.exposure[1], mi.exposure[2], mi.black_level[0], mi.black_level[1], mi.black_level[2]);
    g_last_focus = im.focus;
    if (o->invert && !minolta) dsd4_invert_negative(&im, 1);
    if (o->display) dsd4_to_display(&im);
    if (o->fx) flip_x(&im);
    if (o->fy) flip_y(&im);
    if (o->rot == 90 || o->rot == 180 || o->rot == 270) {
        if (o->rot == 180) { flip_x(&im); flip_y(&im); }
        else {
            rotate90(&im);
            if (o->rot == 270) { flip_x(&im); flip_y(&im); }
        }
    }
    /* the scanner profile describes linear colour positives only */
    /* slides: Posi Linear profile; Minolta-processed colour negatives: Nega Input */
    const uint8_t *icc = NULL;
    size_t icc_len = 0;
    if (!o->no_profile && !o->display && im.channels == 3) {
        if (p.film == DSD4_FILM_POSITIVE_COLOR && !o->invert) { icc = g_icc; icc_len = g_icc_len; }
        else if (minolta) { icc = g_icc_n; icc_len = g_icc_n_len; }
    }
    const int embed = icc != NULL;
    r = dsd4_write_tiff_icc(&im, path, icc, icc_len);
    if (r != DSD4_OK) {
        fprintf(stderr, "dsdual4: cannot write %s\n", path);
        dsd4_image_free(&im);
        return 1;
    }
    if (g_verbose >= 1)
        fprintf(stderr, "wrote %s: %dx%d, %d-bit %s, %d dpi, focus %d, digital gain %.2f/%.2f/%.2f%s\n",
                path, im.width, im.height, im.bits, im.channels == 3 ? "RGB" : "gray", im.dpi, im.focus,
                im.digital_gain[0], im.digital_gain[1], im.digital_gain[2],
                embed ? (minolta ? ", Nega Input profile embedded" : ", scanner profile embedded") : "");
    dsd4_image_free(&im);
    return 0;
}


/* ------------------------------------------------------------------ menu */

static int g_color;
#define C(code) (g_color ? "\033[" code "m" : "")

static int ask(const char *prompt, char *buf, size_t n)
{
    printf("%s", prompt);
    fflush(stdout);
    if (!fgets(buf, (int)n, stdin)) return -1;
    buf[strcspn(buf, "\r\n")] = 0;
    return 0;
}

static int ask_int(const char *prompt, int def, int lo, int hi)
{
    char b[64], pr[256];
    snprintf(pr, sizeof pr, "%s [%d]: ", prompt, def);
    for (;;) {
        if (ask(pr, b, sizeof b) < 0) return def;
        if (!*b) return def;
        char *e;
        long v = strtol(b, &e, 10);
        if (!*e && v >= lo && v <= hi) return (int)v;
        printf("  please enter a number from %d to %d\n", lo, hi);
    }
}

static void menu_help(void)
{
    printf("\n%sHow to use the DiMAGE Scan Dual IV%s\n\n"
"  1. Connect the scanner by USB and switch it on. Wait until its light stops\n"
"     blinking.\n"
"  2. Choose \"Initialize\" once per power-on. The lamp warms up and the scanner\n"
"     calibrates (best with no holder inserted). This takes 15 s to 2 minutes.\n"
"  3. Load film into the FH-U1 strip holder (or slides into the SH-U1 holder),\n"
"     open the front door and push the holder in, arrow first, until the\n"
"     scanner grabs it, then close the door. Choose \"Refresh\" to see it.\n"
"     A \"holder position\" error usually means the door is open or a holder is\n"
"     half inserted; if it persists, switch the scanner off and on.\n"
"  4. Pick the film type in Settings (slide, colour negative, B&W).\n"
"  5. \"Preview\" makes a quick 800 dpi view of one frame and opens it in\n"
"     Preview. \"Scan\" makes the full-resolution file.\n"
"  6. \"Eject\" returns the holder when you are done.\n\n"
"  Files are saved in the output folder (Settings). Full scans are linear\n"
"  16-bit TIFFs for editing: they look dark until you apply a curve. Choose\n"
"  \"display-ready\" in Settings for files that look right straight away.\n"
"  If an image is mirrored, turn on \"flip horizontally\" in Settings.\n\n"
"  The same functions are available as commands, e.g. dsdual4 scan --frame 2\n"
"  -o out.tiff (see dsdual4 --help). Image Capture / Preview can also scan\n"
"  directly while \"dsdual4 serve\" is running.\n\n", C("1"), C("0"));
}

static const char *film_name(dsd4_film f)
{
    static const char *n[] = { "colour slide (positive)", "colour negative", "B&W positive", "B&W negative" };
    return n[f];
}

static void open_in_preview(const char *path)
{
    char cmd[PATH_MAX + 32];
    if (getenv("DSD4_NO_OPEN")) { printf("  (would open %s)\n", path); return; }
    snprintf(cmd, sizeof cmd, "open -a Preview '%s'", path);
    if (system(cmd) != 0) printf("  could not open %s\n", path);
}

enum { NEG_RAW = 0, NEG_INVERT = 1, NEG_MINOLTA = 2 };

/* preview settings (Device & output settings -> Preview) */
typedef struct {
    int dpi;        /* 100..800 */
    int autofocus;  /* 1: autofocus every preview, 0: reuse the last focus */
    int neg_raw;    /* negatives: 0 simple invert, 1 show the raw negative */
} preview_cfg;
static const char *neg_mode_name(int m)
{
    static const char *n[] = { "raw negative (invert later yourself)", "simple invert",
                               "Minolta processing (original film curves)" };
    return n[m];
}

static int is_negative(dsd4_film f) { return f == DSD4_FILM_NEGATIVE_COLOR || f == DSD4_FILM_NEGATIVE_BW; }

/* apply the menu's negative handling to one scan's options */
static void apply_neg_mode(scan_opts *o, int neg_mode)
{
    o->invert = 0;
    o->minolta = 0;
    if (!is_negative(o->p.film)) return;
    if (neg_mode == NEG_INVERT) o->invert = 1;
    else if (neg_mode == NEG_MINOLTA) o->minolta = 1;
}

static void area_text(const scan_opts *c, char *b, size_t n)
{
    if (c->p.w > 0 && c->p.h > 0) snprintf(b, n, "area %dx%d at %d,%d px", c->p.w, c->p.h, c->p.x, c->p.y);
    else snprintf(b, n, "full frame");
}

static void scan_settings(scan_opts *c, int *neg_mode)
{
    for (;;) {
        char area[64];
        area_text(c, area, sizeof area);
        printf("\n  %sScan settings%s\n", C("1"), C("0"));
        printf("    1  Film type     %s\n", film_name(c->p.film));
        printf("    2  Size          %s\n", area);
        printf("    3  Resolution    %d dpi\n", c->p.dpi);
        if (is_negative(c->p.film)) printf("    4  Negatives     %s\n", neg_mode_name(*neg_mode));
        else printf("    %s4  Negatives     (only for negative film)%s\n", C("2"), C("0"));
        char b[64];
        if (ask("  Change which (Enter = done)? ", b, sizeof b) < 0 || !*b) return;
        switch (b[0]) {
        case '1':
            printf("    1 colour slide (positive)  2 colour negative  3 B&W positive  4 B&W negative\n");
            c->p.film = (dsd4_film)(ask_int("    Film type", (int)c->p.film + 1, 1, 4) - 1);
            break;
        case '2':
            if (ask_int("    1 = full frame, 2 = custom area", c->p.w > 0 ? 2 : 1, 1, 2) == 1) {
                c->p.x = c->p.y = c->p.w = c->p.h = 0;
            } else {
                printf("    Area in pixels at %d dpi (a full 35 mm frame is %d x %d)\n", c->p.dpi,
                       3120 * c->p.dpi / 3200, 4680 * c->p.dpi / 3200);
                c->p.x = ask_int("    Left", c->p.x, 0, 4680);
                c->p.y = ask_int("    Top", c->p.y, 0, 4680);
                c->p.w = ask_int("    Width", c->p.w > 0 ? c->p.w : 3120 * c->p.dpi / 3200 - c->p.x, 4, 4680);
                c->p.h = ask_int("    Height", c->p.h > 0 ? c->p.h : 4680 * c->p.dpi / 3200 - c->p.y, 1, 4680);
            }
            break;
        case '3': {
            static const int dpis[] = { 400, 800, 1600, 3200 };
            printf("    1 = 400, 2 = 800, 3 = 1600, 4 = 3200 dpi, or type any dpi 50-3200\n");
            int d = ask_int("    Resolution", c->p.dpi, 1, 3200);
            c->p.dpi = d <= 4 ? dpis[d - 1] : d < 50 ? 50 : d;
            break;
        }
        case '4':
            if (!is_negative(c->p.film)) { printf("    Choose a negative film type first.\n"); break; }
            printf("    1 raw negative - keep the negative as scanned, to invert later in\n"
                   "      Negative Lab Pro, darktable, ColorPerfect, ...\n"
                   "    2 simple invert - quick per-channel inversion\n"
                   "    3 Minolta processing - the original software's auto-exposure and film\n"
                   "      curves (needs a prescan; tagged with the Nega Input profile)\n");
            *neg_mode = ask_int("    Negatives", *neg_mode + 1, 1, 3) - 1;
            break;
        default:
            printf("    unknown choice\n");
        }
    }
}

static void preview_settings(preview_cfg *pv)
{
    for (;;) {
        printf("\n    %sPreview%s\n", C("1"), C("0"));
        printf("      1  Resolution  %d dpi%s\n", pv->dpi, pv->dpi < 800 ? " (faster)" : "");
        printf("      2  Focus       %s\n", pv->autofocus ? "autofocus every preview" : "skip autofocus (reuse last focus)");
        printf("      3  Negatives   %s\n", pv->neg_raw ? "show the raw negative" : "simple invert");
        char b[32];
        if (ask("    Change which (Enter = back)? ", b, sizeof b) < 0 || !*b) return;
        switch (b[0]) {
        case '1': {
            printf("      1 = 100, 2 = 200, 3 = 400, 4 = 800 dpi, or type any dpi 50-800\n");
            static const int d[] = { 100, 200, 400, 800 };
            int v = ask_int("      Resolution", pv->dpi, 1, 800);
            pv->dpi = v <= 4 ? d[v - 1] : v < 50 ? 50 : v;
            break;
        }
        case '2': pv->autofocus = ask_int("      1 = autofocus, 0 = skip", pv->autofocus, 0, 1); break;
        case '3': pv->neg_raw = ask_int("      Negatives: 0 = simple invert, 1 = raw", pv->neg_raw, 0, 1); break;
        default: printf("      unknown choice\n");
        }
    }
}

static void device_settings(scan_opts *c, int *display, int *open_after, int *use_profile, char *outdir, size_t outlen,
                            preview_cfg *pv)
{
    for (;;) {
        char focus[32];
        if (c->p.focus == DSD4_FOCUS_AUTO) snprintf(focus, sizeof focus, "autofocus every scan");
        else snprintf(focus, sizeof focus, "manual, position %d", c->p.focus);
        printf("\n  %sDevice & output settings%s\n", C("1"), C("0"));
        printf("    1  Focus            %s\n", focus);
        printf("    2  Multi-sampling   %s\n", c->p.multi_sample > 1 ? (c->p.multi_sample == 2 ? "2x" : c->p.multi_sample == 4 ? "4x" : "8x") : "off");
        printf("    3  File format      %s\n", *display ? "display-ready 8-bit" : "16-bit linear (for editing)");
        printf("    4  Flip horizontal  %s\n", c->fx ? "on" : "off");
        printf("    5  Open in Preview  %s\n", *open_after ? "after every scan" : "previews only");
        printf("    6  Output folder    %s\n", outdir);
        printf("    7  Colour profile   %s\n", !g_icc ? "not found" : *use_profile ? "embed scanner profile" : "off");
        printf("    8  Preview          %d dpi, %s, negatives %s\n", pv->dpi, pv->autofocus ? "autofocus" : "no autofocus",
               pv->neg_raw ? "raw" : "inverted");
        char b[PATH_MAX];
        if (ask("  Change which (Enter = done)? ", b, sizeof b) < 0 || !*b) return;
        switch (b[0]) {
        case '1': {
            int v = ask_int("    0 = autofocus, or a manual position -200..200 (blank keeps)",
                            c->p.focus == DSD4_FOCUS_AUTO ? 0 : c->p.focus, -200, 200);
            c->p.focus = v == 0 ? DSD4_FOCUS_AUTO : v;
            break;
        }
        case '2': {
            int k = c->p.multi_sample == 8 ? 3 : c->p.multi_sample == 4 ? 2 : c->p.multi_sample == 2 ? 1 : 0;
            c->p.multi_sample = 1 << ask_int("    0 = off, 1 = 2x, 2 = 4x, 3 = 8x (less noise, slower)", k, 0, 3);
            break;
        }
        case '3': *display = ask_int("    0 = 16-bit linear, 1 = display-ready 8-bit", *display, 0, 1); break;
        case '4': c->fx = ask_int("    Flip horizontally? 1 = yes, 0 = no", c->fx, 0, 1); break;
        case '5': *open_after = ask_int("    Open full scans in Preview? 1 = yes, 0 = no", *open_after, 0, 1); break;
        case '6':
            printf("    Output folder [%s]: ", outdir);
            fflush(stdout);
            if (fgets(b, sizeof b, stdin)) {
                b[strcspn(b, "\r\n")] = 0;
                if (*b) snprintf(outdir, outlen, "%s", b);
            }
            break;
        case '8':
            preview_settings(pv);
            break;
        case '7':
            if (!g_icc) printf("    No profile found (see --profile).\n");
            else *use_profile = ask_int("    Embed the scanner profile in colour slide scans? 1 = yes", *use_profile, 0, 1);
            break;
        default:
            printf("    unknown choice\n");
        }
    }
}

/* menu settings persist in ~/Library/Application Support/dsdual4/menu.conf */
typedef struct {
    scan_opts *cfg;
    int *neg_mode, *display, *open_after, *use_profile;
    preview_cfg *pv;
    char *outdir;
    size_t outlen;
} menu_state;

static void settings_path(char *b, size_t n)
{
    const char *home = getenv("HOME");
    snprintf(b, n, "%s/Library/Application Support/dsdual4/menu.conf", home ? home : ".");
}

static void save_settings(const menu_state *m)
{
    char path[PATH_MAX], dir[PATH_MAX];
    settings_path(path, sizeof path);
    snprintf(dir, sizeof dir, "%s", path);
    *strrchr(dir, '/') = 0;
    for (char *q = dir + 1; *q; q++)
        if (*q == '/') { *q = 0; mkdir(dir, 0755); *q = '/'; }
    mkdir(dir, 0755);
    FILE *f = fopen(path, "w");
    if (!f) return;
    const dsd4_scan_params *p = &m->cfg->p;
    fprintf(f, "film=%d\ndpi=%d\narea=%d,%d,%d,%d\nnegatives=%d\nfocus=%d\nsamples=%d\ndisplay=%d\n"
               "flip=%d\nopen_after=%d\nprofile=%d\npreview_dpi=%d\npreview_af=%d\npreview_neg_raw=%d\noutdir=%s\n",
            (int)p->film, p->dpi, p->x, p->y, p->w, p->h, *m->neg_mode, p->focus, p->multi_sample, *m->display,
            m->cfg->fx, *m->open_after, *m->use_profile, m->pv->dpi, m->pv->autofocus, m->pv->neg_raw, m->outdir);
    fclose(f);
}

static void load_settings(menu_state *m)
{
    char path[PATH_MAX], line[PATH_MAX + 32];
    settings_path(path, sizeof path);
    FILE *f = fopen(path, "r");
    if (!f) return;
    dsd4_scan_params *p = &m->cfg->p;
    while (fgets(line, sizeof line, f)) {
        line[strcspn(line, "\r\n")] = 0;
        char *v = strchr(line, '=');
        if (!v) continue;
        *v++ = 0;
        int i = atoi(v);
        if (!strcmp(line, "film") && i >= 0 && i <= 3) p->film = (dsd4_film)i;
        else if (!strcmp(line, "dpi") && i >= 50 && i <= 3200) p->dpi = i;
        else if (!strcmp(line, "area")) sscanf(v, "%d,%d,%d,%d", &p->x, &p->y, &p->w, &p->h);
        else if (!strcmp(line, "negatives") && i >= 0 && i <= 2) *m->neg_mode = i;
        else if (!strcmp(line, "focus")) p->focus = i;
        else if (!strcmp(line, "samples") && (i == 1 || i == 2 || i == 4 || i == 8)) p->multi_sample = i;
        else if (!strcmp(line, "display")) *m->display = i != 0;
        else if (!strcmp(line, "flip")) m->cfg->fx = i != 0;
        else if (!strcmp(line, "open_after")) *m->open_after = i != 0;
        else if (!strcmp(line, "profile")) *m->use_profile = i != 0 && g_icc;
        else if (!strcmp(line, "preview_dpi") && i >= 50 && i <= 800) m->pv->dpi = i;
        else if (!strcmp(line, "preview_af")) m->pv->autofocus = i != 0;
        else if (!strcmp(line, "preview_neg_raw")) m->pv->neg_raw = i != 0;
        else if (!strcmp(line, "outdir") && *v) snprintf(m->outdir, m->outlen, "%s", v);
    }
    fclose(f);
}

static int run_menu(dsd4 *s, int fw_check)
{
    g_color = isatty(1);
    char outdir[PATH_MAX];
    const char *home = getenv("HOME");
    snprintf(outdir, sizeof outdir, "%s/Pictures/dsdual4", home ? home : ".");
    scan_opts cfg = { 0 };
    dsd4_default_params(&cfg.p);
    cfg.p.dpi = 3200;
    int display_scans = 0, open_after = 1, initialized = 0;
    int neg_mode = NEG_INVERT, use_profile = g_icc != NULL;
    preview_cfg pv = { 800, 1, 0 };
    menu_state ms = { &cfg, &neg_mode, &display_scans, &open_after, &use_profile, &pv, outdir, sizeof outdir };
    load_settings(&ms);
    char last[PATH_MAX] = "";
    dsd4_info info;
    int have_info = dsd4_get_info(s, &info) == DSD4_OK;

    printf("\n%sdsdual4%s - Konica Minolta DiMAGE Scan Dual IV\n", C("1"), C("0"));
    printf("Type the letter or number of an option. 'h' shows instructions.\n");
    for (;;) {
        dsd4_holder_info h;
        memset(&h, 0, sizeof h);
        int hr = dsd4_get_holder(s, &h);
        int holder_ok = hr == DSD4_OK && h.holder != DSD4_HOLDER_NONE && h.frames > 0;
        static const char *hn[] = { "none", "35 mm film strip", "35 mm slides", "APS", "unknown" };

        printf("\n%s──────────────────────────────────────────────────────%s\n", C("2"), C("0"));
        if (have_info) {
            char name[32] = "";
            int k = 0;
            for (int q = 8; q < 32 && k < 30; q++) /* "KONICA MINOLTA  FS-V31  " */
                if (info.inquiry[q] != ' ' || (k && name[k - 1] != ' ')) name[k++] = (char)info.inquiry[q];
            while (k && name[k - 1] == ' ') k--;
            name[k] = 0;
            printf(" Scanner    %s, firmware %s\n", name, info.revision);
        }
        printf(" Calibrated %s\n", initialized ? "yes" : "no - choose Initialize");
        if (holder_ok) printf(" Holder     %s, %d frames (at frame %d)\n", hn[h.holder], h.frames, h.current_frame);
        else printf(" Holder     not detected - push it in until the scanner grabs it\n");
        {
            char area[64];
            area_text(&cfg, area, sizeof area);
            printf(" Scan       %s, %s, %d dpi%s%s\n", film_name(cfg.p.film), area, cfg.p.dpi,
                   is_negative(cfg.p.film) ? ", " : "", is_negative(cfg.p.film) ? neg_mode_name(neg_mode) : "");
            printf(" Device     focus %s, %s, %s\n", cfg.p.focus == DSD4_FOCUS_AUTO ? "auto" : "manual",
                   display_scans ? "display-ready 8-bit" : "16-bit linear",
                   use_profile && g_icc ? "profile embedded" : "no profile");
            printf(" Output     %s\n", outdir);
        }
        printf("%s──────────────────────────────────────────────────────%s\n", C("2"), C("0"));

        /* scanning is never blocked up front: the action initialises if needed
         * and re-checks the holder at the moment it runs */
        const char *need_scan = NULL;
        struct { char key; const char *label; const char *blocked; } items[] = {
            { 'i', "Initialize (warm up lamp and calibrate)", NULL },
            { 'p', initialized ? "Preview a frame (opens in Preview)"
                               : "Preview a frame (initializes first)", need_scan },
            { 's', "Scan a frame", need_scan },
            { 'a', "Scan all frames", need_scan },
            { 'f', "Autofocus a frame", need_scan },
            { 'o', "Open the last file in Preview", *last ? NULL : "nothing scanned yet" },
            { 'e', "Eject the holder", holder_ok ? NULL : "no holder" },
            { 'x', "Reset the scanner (if it reports errors)", NULL },
            { 'c', "Scan settings (film, size, resolution, negatives)", NULL },
            { 'd', "Device & output settings", NULL },
            { 'r', "Refresh status", NULL },
            { 'h', "Instructions", NULL },
            { 'q', "Quit", NULL },
        };
        for (size_t k = 0; k < sizeof items / sizeof *items; k++) {
            if (items[k].blocked)
                printf("  %s%c  %-46s (%s)%s\n", C("2"), items[k].key, items[k].label, items[k].blocked, C("0"));
            else
                printf("  %s%c%s  %s\n", C("1"), items[k].key, C("0"), items[k].label);
        }
        char b[64];
        if (ask("> ", b, sizeof b) < 0) return 0;
        char key = b[0];
        const char *blocked = NULL;
        for (size_t k = 0; k < sizeof items / sizeof *items; k++)
            if (items[k].key == key) blocked = items[k].blocked;
        if (blocked) { printf("  not available yet: %s\n", blocked); continue; }

        { /* mkdir -p */
            char tmp[PATH_MAX];
            snprintf(tmp, sizeof tmp, "%s", outdir);
            for (char *q = tmp + 1; *q; q++)
                if (*q == '/') { *q = 0; mkdir(tmp, 0755); *q = '/'; }
            mkdir(tmp, 0755);
        }
        time_t now = time(NULL);
        char stamp[32];
        strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", localtime(&now));

        if (key == 'p' || key == 's' || key == 'a' || key == 'f') {
            if (!initialized) {
                printf("  Initializing first...\n");
                if (check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize")) continue;
                done_progress();
                initialized = 1;
            }
            /* fresh holder status; wait while a tray is still being pulled in */
            holder_ok = 0;
            for (int t = 0; t < 8 && !holder_ok; t++) {
                memset(&h, 0, sizeof h);
                hr = dsd4_get_holder(s, &h);
                holder_ok = h.holder != DSD4_HOLDER_NONE && h.frames > 0;
                if (!holder_ok) {
                    if (t == 0) printf("  Waiting for the holder...\n");
                    sleep(2);
                }
            }
            if (!holder_ok) {
                printf("  No holder detected. Push it in until the scanner grabs it, close the\n"
                       "  door, and try again (h for instructions).\n");
                continue;
            }
        }
        switch (key) {
        case 'i':
            printf("  Warming up and calibrating (best with no holder inserted)...\n");
            if (!check(dsd4_initialize(s, fw_check, on_progress, NULL), s, "initialize")) {
                done_progress();
                initialized = 1;
                printf("  %sScanner ready.%s\n", C("32"), C("0"));
            }
            break;
        case 'p': case 's': case 'f': {
            int fr = ask_int("  Frame", h.current_frame > 0 && h.current_frame <= h.frames ? h.current_frame : 1, 1, h.frames);
            scan_opts o = cfg;
            o.p.frame = fr;
            if (key == 'f') {
                int best = 0;
                if (!check(dsd4_select_frame(s, fr), s, "frame") &&
                    !check(dsd4_autofocus(s, &o.p, &best, on_progress, NULL), s, "autofocus")) {
                    done_progress();
                    printf("  best focus position for frame %d: %d\n", fr, best);
                    g_last_focus = best;
                    if (ask_int("  Use it for the following scans? 1 = yes, 0 = no", 1, 0, 1)) {
                        cfg.p.focus = best;
                        save_settings(&ms);
                    }
                }
                break;
            }
            o.p.preview = key == 'p';
            o.display = key == 'p' ? 1 : display_scans;
            apply_neg_mode(&o, neg_mode);
            o.no_profile = !use_profile;
            if (key == 'p') {
                o.p.dpi = pv.dpi;
                o.minolta = 0; /* the Minolta pipeline is for full scans */
                o.invert = is_negative(o.p.film) && !pv.neg_raw;
                if (!pv.autofocus)
                    o.p.focus = g_last_focus != DSD4_FOCUS_AUTO ? g_last_focus
                              : cfg.p.focus != DSD4_FOCUS_AUTO ? cfg.p.focus : 0;
            }
            char path[PATH_MAX];
            snprintf(path, sizeof path, "%s/%s-frame%d-%s.tiff", outdir, key == 'p' ? "preview" : "scan", fr, stamp);
            if (do_scan(s, &o, path) == 0) {
                snprintf(last, sizeof last, "%s", path);
                if (key == 'p' || open_after) open_in_preview(path);
            }
            break;
        }
        case 'a':
            for (int fr = 1; fr <= h.frames; fr++) {
                scan_opts o = cfg;
                o.p.frame = fr;
                o.display = display_scans;
                apply_neg_mode(&o, neg_mode);
            o.no_profile = !use_profile;
                char path[PATH_MAX];
                snprintf(path, sizeof path, "%s/scan-frame%d-%s.tiff", outdir, fr, stamp);
                printf("  frame %d of %d\n", fr, h.frames);
                if (do_scan(s, &o, path)) break;
                snprintf(last, sizeof last, "%s", path);
            }
            break;
        case 'o':
            open_in_preview(last);
            break;
        case 'x':
            printf("  Resetting the scanner...\n");
            if (!check(dsd4_reset(s, 60), s, "reset")) {
                initialized = 0;
                printf("  Scanner reset. Choose Initialize next.\n");
            }
            break;
        case 'e':
            if (!check(dsd4_eject(s), s, "eject")) printf("  Holder ejected.\n");
            break;
        case 'c':
            scan_settings(&cfg, &neg_mode);
            save_settings(&ms);
            break;
        case 'd':
            device_settings(&cfg, &display_scans, &open_after, &use_profile, outdir, sizeof outdir, &pv);
            save_settings(&ms);
            break;
        case 'r':
            break;
        case 'h': case '?':
            menu_help();
            break;
        case 'q':
            return 0;
        default:
            if (*b) printf("  unknown option '%s' (h for instructions)\n", b);
        }
    }
}

/* dsdual4 convert IN.tiff -o OUT.tiff --film bw-nega|nega [--scan-exposure f|r,g,b] */
static int cmd_convert(int argc, char **argv)
{
    const char *in = NULL, *out = NULL;
    dsd4_film film = DSD4_FILM_NEGATIVE_BW;
    double ex[3] = { 0, 0, 0 };
    int bits = 16;
    static const char *use = "usage: dsdual4 convert IN.tiff -o OUT.tiff --film bw-nega|nega [--scan-exposure f]\n";
    for (int k = 1; k < argc; k++) {
        if (!strcmp(argv[k], "-o") && k + 1 < argc) out = argv[++k];
        else if (!strcmp(argv[k], "--film") && k + 1 < argc) {
            if (parse_film(argv[++k], &film) || (film != DSD4_FILM_NEGATIVE_BW && film != DSD4_FILM_NEGATIVE_COLOR)) {
                fprintf(stderr, "dsdual4: convert needs --film nega or bw-nega\n");
                return 2;
            }
        } else if (!strcmp(argv[k], "--scan-exposure") && k + 1 < argc) {
            int n = sscanf(argv[++k], "%lf,%lf,%lf", &ex[0], &ex[1], &ex[2]);
            if (n == 1) ex[1] = ex[2] = ex[0];
            else if (n != 3) { fprintf(stderr, "bad --scan-exposure\n"); return 2; }
        } else if (!strcmp(argv[k], "--bits") && k + 1 < argc) bits = atoi(argv[++k]);
        else if (argv[k][0] != '-' && !in) in = argv[k];
        else { fputs(use, stderr); return 2; }
    }
    if (!in || !out) { fputs(use, stderr); return 2; }
    if (!(ex[0] > 0)) {
        dsd4_film_default_exposure(film, ex);
        fprintf(stderr, "assuming the scan was made at the %s standard exposure %.3f/%.3f/%.3f "
                "(--scan-exposure to override)\n", film == DSD4_FILM_NEGATIVE_BW ? "B&W negative" : "colour negative",
                ex[0], ex[1], ex[2]);
    }
    dsd4_image src, dst;
    int r = dsd4_read_tiff(in, &src);
    if (r) { fprintf(stderr, "dsdual4: cannot read %s (uncompressed 16-bit TIFF needed)\n", in); return 1; }
    if (src.bits != 16) { fprintf(stderr, "dsdual4: %s is not a 16-bit (raw) scan\n", in); dsd4_image_free(&src); return 1; }
    if (!src.dpi) src.dpi = 3200;
    dsd4_minolta_info info;
    r = dsd4_minolta_convert(&src, film, ex, &dst, &info);
    dsd4_image_free(&src);
    if (r == DSD4_ERR_IO) { fprintf(stderr, "dsdual4: no calibration cache: run 'dsdual4 init' with the scanner first\n"); return 1; }
    if (r) { fprintf(stderr, "dsdual4: conversion failed: %s\n", dsd4_strerror(r)); return 1; }
    if (bits == 8) {
        size_t n = (size_t)dst.width * dst.height * dst.channels;
        for (size_t i = 0; i < n; i++) dst.data[i] = (uint8_t)(((uint16_t *)dst.data)[i] >> 8);
        dst.bits = 8;
        dst.stride = (size_t)dst.width * dst.channels;
    }
    int embed = g_icc_n && dst.channels == 3;
    r = dsd4_write_tiff_icc(&dst, out, embed ? g_icc_n : NULL, embed ? g_icc_n_len : 0);
    fprintf(stderr, "wrote %s: %dx%d %d-bit %s, Minolta AE exposure %.3f/%.3f/%.3f, black level %d/%d/%d%s\n", out,
            dst.width, dst.height, dst.bits, dst.channels == 3 ? "RGB" : "gray", info.exposure[0], info.exposure[1],
            info.exposure[2], info.black_level[0], info.black_level[1], info.black_level[2],
            embed ? ", Nega Input profile embedded" : "");
    dsd4_image_free(&dst);
    return r ? 1 : 0;
}

int main(int argc, char **argv)
{
    int use_sim = 0, fw_check = 0, no_profile = 0;
    const char *profile = NULL;
    const char *fw = NULL;
    int i = 1;
    for (; i < argc && argv[i][0] == '-'; i++) {
        if (!strcmp(argv[i], "--sim")) use_sim = 1;
        else if (!strcmp(argv[i], "--no-firmware-check")) fw_check = 0;
        else if (!strcmp(argv[i], "--firmware-update")) fw_check = 1;
        else if (!strcmp(argv[i], "--firmware") && i + 1 < argc) fw = argv[++i];
        else if (!strcmp(argv[i], "--profile") && i + 1 < argc) profile = argv[++i];
        else if (!strcmp(argv[i], "--no-profile")) no_profile = 1;
        else if (!strncmp(argv[i], "-v", 2)) g_verbose += (int)strlen(argv[i]) - 1;
        else if (!strcmp(argv[i], "-q")) g_verbose = 0;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(); return 0; }
        else { fprintf(stderr, "unknown option %s\n", argv[i]); usage(); return 2; }
    }
    const char *cmd = i < argc ? argv[i] : "menu";
    if (i >= argc && !isatty(0)) { usage(); return 2; }
    int cargc = i < argc ? argc - i : 0;
    char **cargv = argv + i;
    g_progress_tty = isatty(2);
    if (!fw) fw = find_firmware(argv[0]);
    if (!no_profile) load_profile(argv[0], profile);

    if (!strcmp(cmd, "convert")) return cmd_convert(cargc, cargv);

    char err[256] = "";
    dsd4 *s = NULL;
    int wait = !strcmp(cmd, "menu");
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
    if (!strcmp(cmd, "menu")) {
        rc = run_menu(s, fw_check);
    } else if (!strcmp(cmd, "info")) {
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
    } else if (!strcmp(cmd, "reset")) {
        fprintf(stderr, "resetting the scanner...\n");
        rc = check(dsd4_reset(s, 60), s, "reset");
        if (!rc) {
            dsd4_holder_info h;
            dsd4_get_holder(s, &h);
            fprintf(stderr, "scanner ready (holder: %s). Run init next.\n",
                    h.holder == DSD4_HOLDER_NONE ? "none" : h.holder == DSD4_HOLDER_STRIP ? "35 mm strip" :
                    h.holder == DSD4_HOLDER_SLIDE ? "35 mm slides" : "other");
        }
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
        dsd4_escl_options eo = { .port = 8090, .film = DSD4_FILM_POSITIVE_COLOR, .icc = g_icc, .icc_len = g_icc_len,
                                  .icc_n = g_icc_n, .icc_n_len = g_icc_n_len };
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
