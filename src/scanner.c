/*
 * High level scanner control: port of CBaseScannerAVI / CBaseMethod2889
 * (MFSBaseLib2891) and of the parts of MFSLib2891 that decide exposure and
 * focus (GetStdGain, GetAEGain, LineScanSettingInit, AutoFocus, FocusAE,
 * FocusScan).
 */
#include "dsd4_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

int avl_initialize_scanner_cb(dsd4 *s, dsd4_progress_fn cb, void *ud);

/* Scan modes of MFSLineScanSetting[0] */
enum { MODE_FINAL = 0, MODE_PREVIEW = 1, MODE_INDEX = 2, MODE_FOCUS = 3 };

/* MFSFilmType bits: 1 posi, 2 nega, 4 color */
static int film_type_code(dsd4_film f)
{
    switch (f) {
    case DSD4_FILM_POSITIVE_COLOR: return 5;
    case DSD4_FILM_NEGATIVE_COLOR: return 6;
    case DSD4_FILM_POSITIVE_BW: return 9;  /* MFSFilmType codes: 5 colour posi, 6 colour nega, 9 B&W posi, 10 B&W nega */
    case DSD4_FILM_NEGATIVE_BW: return 10;
    }
    return 5;
}
static int is_nega(int t) { return (t >> 1) & 1; }
static int is_color(int t) { return (t & 4) || t == 0x400 || t == 0x100; }

/* CBaseMethod2889::GetImageAbility: {w, h, dpi} for max, preview, index */
typedef struct { uint32_t w, h; uint16_t dpi; } ability_area;
static int image_ability(int format_aps, ability_area a[4])
{
    if (format_aps) {
        a[0] = (ability_area){ 0x888, 0xed0, 0xc80 };
        a[1] = (ability_area){ 0x222, 0x3b4, 800 };
        a[2] = (ability_area){ 0x222, 0x3b4, 800 };
        a[3] = (ability_area){ 0x888, 0xed0, 0xc80 };
    } else {
        a[0] = (ability_area){ 0xc30, 0x1248, 0xc80 };
        a[1] = (ability_area){ 0x30c, 0x492, 800 };
        a[2] = (ability_area){ 0xbd, 0x11c, 0xc2 };
        a[3] = (ability_area){ 0xc30, 0x1248, 0xc80 };
    }
    return 0;
}

/* Everything CBaseScannerAVI needs from an MFSLineScanSetting */
typedef struct {
    int mode;           /* [0] */
    int format_aps;     /* [1] == 0x40 */
    int film;           /* [2] film type code */
    uint16_t frame;     /* [3] */
    uint32_t x, y, w, h;/* [4..7] */
    uint16_t dpi;       /* [8] */
    int gray;           /* [9] == 4 */
    int depth16;        /* byte 0x22 == 2 */
    uint8_t samples;    /* byte 0x5c */
    double exp[4];      /* +0x60 exposure factors */
    double cap[4];      /* +0x80 AE/standard gain (exposure cap base) */
    int16_t focus;      /* +0xa0 */
} line_setting;

/* --------------------------------------------------------------- logging */

void dsd4_set_log(dsd4 *s, dsd4_log_fn fn, void *ud, int verbose)
{
    s->log = fn;
    s->log_ud = ud;
    s->verbose = verbose;
}

void dsd4_set_firmware_path(dsd4 *s, const char *path)
{
    snprintf(s->firmware_path, sizeof s->firmware_path, "%s", path ? path : "");
}

int dsd4_last_error(dsd4 *s) { return s->last_error; }

const char *dsd4_strerror(int status)
{
    switch (status) {
    case DSD4_OK: return "success";
    case DSD4_ERR_IO: return "USB communication failed";
    case DSD4_ERR_SCANNER: return "scanner reported an error";
    case DSD4_ERR_PARAM: return "invalid parameter";
    case DSD4_ERR_NOMEM: return "out of memory";
    case DSD4_ERR_NO_HOLDER: return "no film holder inserted";
    case DSD4_ERR_WRONG_HOLDER: return "film holder does not match the requested film format";
    case DSD4_ERR_NO_FRAME: return "frame number exceeds the holder";
    case DSD4_ERR_CANCELLED: return "cancelled";
    case DSD4_ERR_FIRMWARE: return "firmware update failed";
    }
    return "unknown error";
}

/* error texts derived from CBaseScannerAVI::GetScannerError */
const char *dsd4_scanner_error_text(int code)
{
    switch (code) {
    case 0x00: return "no error";
    case 0x67: return "holder / film transport error";
    case 0x68: return "holder position error";
    case 0x69: return "film holder error";
    case 0x73: return "lamp error";
    case 0x7b: return "general scanner error";
    case 0x7f: return "out of memory";
    case 0x81: case 0x9b: return "scanner hardware error";
    case 0x83: return "USB timeout";
    case 0x96: case 0xa2: return "scanner busy";
    case 0x97: return "invalid scan parameter";
    case 0x98: return "out of memory";
    case 0x9a: return "REQUEST SENSE failed";
    case 0x9f: return "not scanning";
    case 0xa4: return "communication error";
    case 0xa6: case 0xfa: return "firmware file error";
    case 0xc8: return "mechanical error (0)";
    case 0xc9: return "mechanical error (1)";
    case 0xca: return "mechanical error (2)";
    case 0xcb: return "mechanical error (3)";
    case 0xcc: return "frame out of range";
    case 0xcd: return "mechanical error (5)";
    case 0xce: return "mechanical error (6)";
    case 0xcf: return "mechanical error (7)";
    case 0xd0: return "mechanical error (8)";
    case 0xd1: return "film holder inserted incorrectly";
    }
    return "scanner error";
}

static int fail(dsd4 *s)
{
    return s->last_error == 0x83 ? DSD4_ERR_IO : DSD4_ERR_SCANNER;
}

/* --------------------------------------------------------------- lifecycle */

int dsd4_open_transport(dsd4 **out, dsd4_transport *t)
{
    dsd4 *s = calloc(1, sizeof *s);
    if (!s) return DSD4_ERR_NOMEM;
    s->t = t;
    s->verbose = 1;
    avl_initialize_driver(s);
    *out = s;
    return DSD4_OK;
}

int dsd4_open_usb(dsd4 **out, char *err, size_t errlen)
{
    dsd4_transport *t = dsd4_usb_open(err, errlen);
    if (!t) return DSD4_ERR_IO;
    return dsd4_open_transport(out, t);
}

void dsd4_close(dsd4 *s)
{
    if (!s) return;
    if (s->scanning) avl_stop_scan(s);
    free(s->pack_buf);
    free(s->accum);
    if (s->t) s->t->close(s->t);
    free(s);
}

/* -------------------------------------------------------------- queries */

int dsd4_get_info(dsd4 *s, dsd4_info *info)
{
    memset(info, 0, sizeof *info);
    if (!s->ability_valid && !avl_get_scanner_ability(s)) return fail(s);
    const uint8_t *q = s->inquiry;
    memcpy(info->vendor, q + 8, 8);
    memcpy(info->product, q + 16, 16);
    memcpy(info->revision, q + 32, 4);
    for (int i = 7; i >= 0 && info->vendor[i] == ' '; i--) info->vendor[i] = 0;
    for (int i = 15; i >= 0 && info->product[i] == ' '; i--) info->product[i] = 0;
    info->optical_dpi = s->ab.optical;
    info->max_pixels = s->ab.max_pixels;
    info->max_lines = s->ab.max_lines;
    memcpy(info->inquiry, q, 96);
    return DSD4_OK;
}

static dsd4_holder holder_from_raw(int t)
{
    switch (t) {
    case 0: return DSD4_HOLDER_NONE;
    case 1: return DSD4_HOLDER_APS;
    case 2: return DSD4_HOLDER_STRIP;
    case 3: return DSD4_HOLDER_SLIDE;
    case 0xff: return DSD4_HOLDER_NONE;
    }
    return DSD4_HOLDER_UNKNOWN;
}

int dsd4_get_holder(dsd4 *s, dsd4_holder_info *h)
{
    uint8_t b[8];
    memset(h, 0, sizeof *h);
    int ok = avl_read_holder(s, b);
    h->raw_type = b[0];
    h->holder = holder_from_raw(b[0]);
    h->current_frame = b[1];
    /* CBaseScannerAVI::GetState: strip 6 frames, slide 4, APS from byte 2 */
    switch (h->holder) {
    case DSD4_HOLDER_STRIP: h->frames = 6; break;
    case DSD4_HOLDER_SLIDE: h->frames = 4; break;
    case DSD4_HOLDER_APS: h->frames = b[2]; break;
    default: h->frames = 0; break;
    }
    s->holder_type = b[0];
    s->frame_count = b[2];
    if (!ok) return b[0] == 0xff ? DSD4_ERR_NO_HOLDER : fail(s);
    return DSD4_OK;
}

/* ---------------------------------------------------------- bring-up */

/* CBaseScannerAVI::UpdateSetupData: exposure limits from the calibration
 * format (little-endian words at 21..30). */
static int update_setup_data(dsd4 *s)
{
    uint8_t fmt[32];
    if (!avl_get_calibration_format(s, fmt)) return 0;
    s->base_exp[0] = le16(fmt + 21);
    s->base_exp[1] = le16(fmt + 23);
    s->base_exp[2] = le16(fmt + 25);
    s->max_exp = le16(fmt + 27);
    s->min_exp = le16(fmt + 29);
    dsd4_logf(s, 2, "exposure base %u/%u/%u, range %u..%u", s->base_exp[0], s->base_exp[1],
              s->base_exp[2], s->min_exp, s->max_exp);
    return 1;
}

/* CBaseScannerAVI::DoSetupScanner */
static int do_setup_scanner(dsd4 *s, dsd4_progress_fn cb, void *ud)
{
    if (s->base_state == 1) {
        if (cb) cb(ud, "initializing scanner", -1);
        if (!avl_initialize_scanner_cb(s, cb, ud)) return 0;
        s->setup_done = 1;
    }
    if (s->base_state == 3) {
        if (cb) cb(ud, "calibrating", -1);
        if (!avl_do_calibration(s)) return 0;
    }
    s->base_state = 2;
    return update_setup_data(s);
}

int dsd4_initialize(dsd4 *s, int allow_fw, dsd4_progress_fn cb, void *ud)
{
    /* MFSBaseGetScannerInfo: firmware check once per session */
    if (!s->firmware_checked) {
        s->firmware_checked = 1;
        if (allow_fw && s->firmware_path[0]) {
            int dl = 0;
            if (cb) cb(ud, "checking firmware", -1);
            if (!avl_download_firmware(s, s->firmware_path, &dl)) {
                dsd4_logf(s, 1, "firmware check failed: %s", dsd4_scanner_error_text(s->last_error));
                if (dl) return DSD4_ERR_FIRMWARE;
            } else if (dl) {
                dsd4_logf(s, 1, "firmware downloaded");
            }
        }
    }
    /* CBaseScannerAVI::InitializeBaseScanner */
    avl_initialize_driver(s);
    s->speed_mode = 1;
    if (!avl_find_scanner(s)) return fail(s);
    if (!avl_check_scanner_state(s, 2, &s->base_state)) return fail(s);
    if (!avl_get_scanner_ability(s)) return fail(s);
    if (s->base_state == 0) {
        s->base_state = 1;
        if (!avl_find_scanner(s)) { s->base_state = 0; return fail(s); }
        if (!avl_check_scanner_state(s, 1, &s->base_state)) return fail(s);
    }
    dsd4_logf(s, 2, "scanner state %u", s->base_state);
    /* the original defers calibration (state 3) to the first scan; do it now */
    if (s->base_state == 1 || s->base_state == 3) {
        if (!do_setup_scanner(s, cb, ud)) return fail(s);
    } else if (!update_setup_data(s)) {
        return fail(s);
    }
    s->carriage_out = 0;
    return DSD4_OK;
}

/* DoScanCalibrationCheck */
static int scan_calibration_check(dsd4 *s)
{
    if (s->holder_type == 1) return 1; /* GetHolder() == APS: skipped */
    if (!avl_check_scanner_state(s, 2, &s->base_state)) return 0;
    if (s->base_state == 3 || s->base_state == 0) {
        if (s->base_state == 0) s->base_state = 1;
        return do_setup_scanner(s, NULL, NULL);
    }
    if (!s->have_shading) {
        s->base_state = 3;
        return do_setup_scanner(s, NULL, NULL);
    }
    return 1;
}

/* ------------------------------------------------------------- mechanics */

int dsd4_select_frame(dsd4 *s, int frame)
{
    if (frame < 1 || frame > 254) return DSD4_ERR_PARAM;
    dsd4_holder_info h;
    int r = dsd4_get_holder(s, &h);
    if (r) return r;
    if (h.holder == DSD4_HOLDER_NONE) return DSD4_ERR_NO_HOLDER;
    if (frame > h.frames) return DSD4_ERR_NO_FRAME;
    /* GetSpecificFrameNo is the identity on the Dual IV */
    if (!avl_send_holder_control(s, (uint8_t)frame)) return fail(s);
    return DSD4_OK;
}

int dsd4_eject(dsd4 *s)
{
    if (!avl_send_holder_control(s, 0xff)) return fail(s);
    s->carriage_out = 0;
    return DSD4_OK;
}

/* HomePositionExec */
int dsd4_home(dsd4 *s)
{
    if (s->scanning && !avl_stop_scan(s)) return fail(s);
    if (!avl_paper_feed(s) && s->last_error != 0 && s->last_error != 0x96 && s->last_error != 0xa2)
        return fail(s);
    s->carriage_out = 0;
    return DSD4_OK;
}

int dsd4_reset(dsd4 *s, int seconds)
{
    if (s->scanning) avl_stop_scan(s);
    avl_reset_to_initial_state(s, 0); /* may be refused while the holder is moving */
    for (int t = 0; t < seconds * 2; t++) {
        if (av_test_unit_ready(s)) {
            s->base_state = 0;
            s->carriage_out = 0;
            return DSD4_OK;
        }
        av_delay_ticks(30);
    }
    return fail(s);
}

int dsd4_set_focus(dsd4 *s, int pos)
{
    if (pos < -250 || pos > 250) return DSD4_ERR_PARAM;
    if (!avl_set_optical_control(s, (int16_t)pos)) return fail(s);
    return DSD4_OK;
}

/* -------------------------------------------------------- exposure model */

/* GetStdGain (glass-less 35 mm path) */
void dsd4_film_default_exposure(dsd4_film film, double e[3])
{
    int t = film_type_code(film);
    e[0] = e[1] = e[2] = 1.0;
    if (!is_nega(t)) {
        for (int i = 0; i < 3; i++) e[i] /= 0.7;
    } else if (is_color(t)) {
        e[0] /= 0.65;
        e[1] /= 0.3;
        e[2] /= 0.2;
    }
}

/* GetAEGain: like GetStdGain but positives stay at 1.0 */
static void ae_gain(int t, double e[3])
{
    e[0] = e[1] = e[2] = 1.0;
    if (is_nega(t) && is_color(t)) {
        e[0] /= 0.65;
        e[1] /= 0.3;
        e[2] /= 0.2;
    }
}

static int is_posi_std_exp(const double e[3])
{
    const double std = 1.0 / 0.7; /* 1.4285714285714286 */
    return e[0] == std && e[1] == std && e[2] == std;
}

/* GetMaxExpTime */
static uint32_t max_exp_time(dsd4 *s, const line_setting *ls)
{
    uint32_t lim = s->max_exp;
    if (ls->mode == MODE_INDEX || ls->mode < 0 || ls->mode > 3) return lim;
    if (ls->mode == MODE_FOCUS) lim = 25000;
    static const double add[3] = { 1.61, 1.64, 1.47 };
    if (is_color(ls->film)) {
        uint32_t t[3];
        for (int c = 0; c < 3; c++) {
            t[c] = (uint32_t)(ls->exp[c] * (double)s->base_exp[c]);
            double r;
            if (ls->mode == MODE_PREVIEW || ls->mode == MODE_FOCUS) {
                r = (double)t[c] / (double)lim;
                if (r < 2.0) r = 2.0;
            } else if (!is_nega(ls->film)) {
                r = 1.0;
            } else {
                r = (t[c] > lim ? (double)t[c] / (double)lim : 1.0) + add[c];
            }
            t[c] = (uint32_t)((double)t[c] / r);
        }
        lim = t[0];
        if (t[1] > lim) lim = t[1];
        if (t[2] > lim) lim = t[2];
    }
    if (s->max_exp < lim) lim = s->max_exp;
    return lim;
}

/* CheckGainLimit: clamp exposure time to the hardware range and move the
 * excess into a host-side digital gain. */
static void check_gain_limit(dsd4 *s, line_setting *ls, double dg[3])
{
    static const double capmul[3] = { 3.0, 2.2, 3.2 };
    uint32_t lim = s->max_exp;
    const uint32_t lo = s->min_exp + 10u;
    dg[0] = dg[1] = dg[2] = 1.0;
    if ((ls->mode == MODE_FINAL && is_nega(ls->film) && is_color(ls->film)) ||
        (ls->mode == MODE_PREVIEW && is_color(ls->film)) ||
        (ls->mode == MODE_FOCUS && s->carriage_out)) {
        lim = max_exp_time(s, ls);
        if (lim < lo) lim = lo;
    }
    for (int c = 0; c < 3; c++) {
        uint32_t base = is_color(ls->film) ? s->base_exp[c] : s->base_exp[1];
        uint32_t t = (uint32_t)(ls->exp[c] * (double)base);
        double f = ls->exp[c], g = 1.0;
        if (s->base_exp[c]) {
            if (t > lim) { g = (double)t / lim; f = (double)lim / s->base_exp[c]; }
            else if (t < lo) { g = (double)t / lo; f = (double)lo / s->base_exp[c]; }
        }
        if (ls->mode == MODE_FINAL) {
            double cap = ls->cap[c] * capmul[c];
            if (cap < f) { g *= f / cap; f = cap; }
        }
        if (g != 1.0) { ls->exp[c] = f; dg[c] = g; }
        if (!is_color(ls->film)) break;
    }
}

/* -------------------------------------------- AV scan parameter assembly */

/* GetAVLibScanParameter + CheckAVExtParameter */
static void av_scan_parameter(dsd4 *s, const line_setting *ls, sp_t *p)
{
    memset(p, 0, sizeof *p);
    p->flags = 0x9000;
    if (ls->gray) {
        p->mode = 2;
        p->bits = ls->depth16 ? 0x10 : 8;
    } else {
        p->mode = 4;
        p->bits = ls->depth16 ? 0x30 : 0x18;
    }
    ability_area a[4];
    image_ability(ls->format_aps, a);
    uint16_t w = 0, h = 0;
    switch (ls->mode) {
    case MODE_PREVIEW:
        /* the original previews at the image ability's 800 dpi; lower
         * previews use the scanner's native 100/200/400 dpi steps */
        p->xres = p->yres = ls->dpi ? ls->dpi : a[1].dpi;
        w = ls->w ? (uint16_t)ls->w : (uint16_t)a[1].w;
        h = ls->h ? (uint16_t)ls->h : (uint16_t)a[1].h;
        break;
    case MODE_INDEX:
        p->xres = p->yres = a[2].dpi;
        w = (uint16_t)a[2].w; h = (uint16_t)a[2].h;
        break;
    default:
        p->xres = p->yres = ls->dpi;
        w = (uint16_t)ls->w; h = (uint16_t)ls->h;
        break;
    }
    if (ls->samples != 1 && ls->mode != MODE_FOCUS) {
        p->multi_sample = ls->samples;
        p->flags |= SPF_MULTISAMPLE;
    }
    p->ulx = (uint16_t)ls->x;
    p->uly = (uint16_t)ls->y;
    p->width = w;
    p->length = h;
    p->flags |= SPF_BASE2400;
    p->pixels = w;
    p->lines = h;
    p->focus = ls->focus;
    p->halftone = 1;
    p->highlight = 0xff;
    p->exp_r = p->exp_g = p->exp_b = 100;
    p->exp_ir = 100;
    if (ls->gray) {
        p->exp_r = (uint16_t)(int)(100.0 * ls->exp[0]);
        p->exp_g = (uint16_t)(int)(100.0 * ls->exp[0]);
        p->exp_b = (uint16_t)(int)(100.0 * ls->exp[0]);
        p->mode = 2;
    } else {
        p->exp_r = (uint16_t)(int)(100.0 * ls->exp[0]);
        p->exp_g = (uint16_t)(int)(100.0 * ls->exp[1]);
        p->exp_b = (uint16_t)(int)(100.0 * ls->exp[2]);
        p->mode = 4;
    }
    if (!is_posi_std_exp(ls->exp)) p->nonstd_dark = ls->mode == MODE_FINAL;
    p->is_aps = ls->format_aps;
    p->film_flag = ls->film == 6 || ls->film == 10;
    p->focus_mode = ls->mode == MODE_FOCUS;
    p->final_scan = ls->mode == MODE_FINAL;
    /* CheckAVExtParameter: modes 0, 1 and 3 (without the "+0x2af" state) */
    p->flags |= SPF_FOCUS;
}

/* ------------------------------------------------------- digital gain */

static void apply_gain16(uint16_t *px, size_t n, int ch, const double g[3])
{
    for (int c = 0; c < ch; c++) {
        if (g[c] == 1.0) continue;
        for (size_t i = c; i < n; i += (size_t)ch) {
            uint32_t v = (uint32_t)(g[c] * px[i] + 0.5);
            px[i] = (uint16_t)(v > 0xffff ? 0xffff : v);
        }
    }
}

/* --------------------------------------------------------- the scan loop */

typedef struct {
    dsd4_progress_fn cb;
    void *ud;
    const char *stage;
} progress;

/* SetWindow + StartScan + read 'lines' lines + StopScan */
static int line_scan(dsd4 *s, line_setting *ls, uint8_t *out, size_t stride, uint32_t lines,
                     const double dg[3], progress *pg)
{
    sp_t p;
    av_scan_parameter(s, ls, &p);
    if (!avl_set_scan_parameter(s, &p)) return 0;
    if (!avl_start_scan(s)) {
        int e = s->last_error;
        avl_stop_scan(s);
        s->last_error = e ? e : AVE_GENERIC;
        return 0;
    }
    /* ReadScan lengths are in output bytes: one channel when gray is
     * extracted from a colour scan ([b222]) */
    const uint32_t lb = s->gray_from_color ? s->line_bytes / 3u : s->line_bytes;
    const unsigned samples = p.multi_sample ? p.multi_sample : 1;
    const uint32_t chunk_lines = samples > 1 ? 1 : (lb ? (256u * 1024u) / lb + 1 : 1);
    uint32_t *acc = samples > 1 ? malloc((size_t)lb / 2 * sizeof *acc) : NULL;
    uint32_t done = 0;
    int ok = 1;
    if (samples > 1 && !acc) { s->last_error = AVE_NOMEM; ok = 0; }
    while (ok && done < lines) {
        uint32_t n = lines - done < chunk_lines ? lines - done : chunk_lines;
        uint8_t *dst = out + (size_t)done * stride;
        if (samples > 1) {
            /* MultiSampleScan: the scanner sends every line 'samples' times */
            uint16_t *w = (uint16_t *)dst;
            memset(acc, 0, (size_t)lb / 2 * sizeof *acc);
            for (unsigned k = 0; k < samples && ok; k++) {
                if (!avl_read_scan(s, dst, lb) || s->read_failed) ok = 0;
                for (uint32_t i = 0; ok && i < lb / 2; i++) acc[i] += w[i];
            }
            if (!ok) break;
            for (uint32_t i = 0; i < lb / 2; i++) w[i] = (uint16_t)(acc[i] / samples);
        } else if (!avl_read_scan(s, dst, n * lb) || s->read_failed) {
            ok = 0;
            break;
        }
        if (ls->depth16) apply_gain16((uint16_t *)dst, (size_t)n * lb / 2, ls->gray ? 1 : 3, dg);
        done += n;
        if (pg && pg->cb) pg->cb(pg->ud, pg->stage, (double)done / lines);
    }
    free(acc);
    int e = s->last_error;
    if (!avl_stop_scan(s)) ok = 0;
    else if (!ok) s->last_error = e ? e : AVE_GENERIC;
    s->carriage_out = 1; /* GetCarriagePosAfterScan */
    return ok;
}

/* holder / film format check (DoFilmFormatCheck) */
static int film_format_check(dsd4 *s, const line_setting *ls)
{
    dsd4_holder_info h;
    int r = dsd4_get_holder(s, &h);
    if (r) return r;
    switch (h.raw_type) {
    case 0xff: return DSD4_ERR_NO_HOLDER;
    case 1:
        if (!ls->format_aps) return DSD4_ERR_WRONG_HOLDER;
        if (h.frames == 0) return DSD4_ERR_NO_FRAME;
        if (h.frames < ls->frame) return DSD4_ERR_NO_FRAME;
        break;
    case 2: case 3:
        if (ls->format_aps) return DSD4_ERR_WRONG_HOLDER;
        if (ls->frame > h.frames) return DSD4_ERR_NO_FRAME;
        break;
    default:
        return DSD4_ERR_NO_HOLDER;
    }
    return DSD4_OK;
}

/* ------------------------------------------------------------ autofocus */

/* FUN_10037ad0: |2v[i]-v[i-1]-v[i+1]| summed per channel on 8 bit values,
 * weighted 3:6:1 and square-rooted. */
static uint32_t sharpness(const uint16_t *line, unsigned n)
{
    uint32_t sum[3] = { 0 };
    for (unsigned c = 0; c < 3; c++)
        for (unsigned i = 1; i + 1 < n; i++) {
            int v = 2 * (line[i * 3 + c] >> 8) - (line[(i - 1) * 3 + c] >> 8) - (line[(i + 1) * 3 + c] >> 8);
            sum[c] += (uint32_t)(v < 0 ? -v : v);
        }
    return (uint32_t)sqrtf((float)((sum[2] + sum[0] * 3 + sum[1] * 6) / 10));
}

typedef struct {
    line_setting ls;
    double dg[3];
    int session;     /* FocusScanBegin issued */
    uint16_t line[256 * 3];
} af_state;

/* MultiSampleScan for the focus line: move the lens, read 'count' copies
 * of the line and average them. */
static int focus_read(dsd4 *s, af_state *af, int16_t pos, unsigned count, uint32_t *metric)
{
    if (!avl_set_optical_control(s, pos)) return 0;
    const uint32_t lb = s->line_bytes;
    uint32_t acc[256 * 3] = { 0 };
    uint16_t tmp[256 * 3];
    if (lb > sizeof tmp) { s->last_error = AVE_PARAM; return 0; }
    for (unsigned k = 0; k < count; k++) {
        if (!avl_read_scan(s, tmp, lb) || s->read_failed) return 0;
        for (unsigned i = 0; i < lb / 2; i++) acc[i] += tmp[i];
    }
    for (unsigned i = 0; i < lb / 2; i++) af->line[i] = (uint16_t)(acc[i] / count);
    apply_gain16(af->line, lb / 2, 3, af->dg);
    if (metric) *metric = sharpness(af->line, lb / 6);
    return 1;
}

/* FocusScan: the first call runs PreFocusScan, the next one FocusScanBegin */
static int focus_scan(dsd4 *s, af_state *af, int16_t pos, unsigned count, uint32_t *metric)
{
    af->ls.focus = pos;
    af->ls.samples = (uint8_t)count;
    if (!af->session) {
        /* FocusScanBegin: one line, scanned repeatedly ("multi scan") */
        check_gain_limit(s, &af->ls, af->dg);
        sp_t p;
        av_scan_parameter(s, &af->ls, &p);
        p.lines = 1;
        p.multi_scan = (uint8_t)count;
        p.multi_lines = (uint16_t)af->ls.h;
        p.flags = (uint16_t)((p.flags & 0xf7ff) | 0x10a);
        if (!avl_set_scan_parameter(s, &p)) return 0;
        if (!avl_start_scan(s)) { avl_stop_scan(s); return 0; }
        af->session = 1;
    }
    return focus_read(s, af, pos, count, metric);
}

/* PreFocusScan (used by FocusAE): a single line, then StopScan. */
static int pre_focus_scan(dsd4 *s, af_state *af, int16_t pos, unsigned count)
{
    af->ls.focus = pos;
    af->ls.samples = (uint8_t)count;
    check_gain_limit(s, &af->ls, af->dg);
    sp_t p;
    av_scan_parameter(s, &af->ls, &p);
    p.lines = 1;
    p.flags |= 0x8c;
    if (!avl_set_scan_parameter(s, &p)) return 0;
    if (!avl_start_scan(s)) { avl_stop_scan(s); return 0; }
    (void)count; /* GetPreFocusData reads a single line (ExtInfo byte 0x21 = 1) */
    int ok = focus_read(s, af, pos, 1, NULL);
    if (!avl_stop_scan(s)) ok = 0;
    return ok;
}

int dsd4_autofocus(dsd4 *s, const dsd4_scan_params *sp, int *best_out, dsd4_progress_fn cb, void *ud)
{
    af_state af;
    memset(&af, 0, sizeof af);
    const int film = film_type_code(sp->film);
    ability_area a[4];
    image_ability(0, a);
    af.ls.mode = MODE_FOCUS;
    af.ls.film = film;
    af.ls.frame = (uint16_t)(sp->frame > 0 ? sp->frame : 1);
    af.ls.dpi = a[0].dpi;          /* 3200 */
    af.ls.w = 0x100;               /* 256 pixel line */
    af.ls.h = 1;
    af.ls.x = a[0].w >> 1;         /* frame centre */
    af.ls.y = a[0].h >> 1;
    if (sp->w && sp->h && sp->dpi) {
        /* focus point = centre of the requested area, in 3200 dpi pixels */
        uint32_t cx = (uint32_t)((sp->x + sp->w / 2) * (3200.0 / sp->dpi));
        uint32_t cy = (uint32_t)((sp->y + sp->h / 2) * (3200.0 / sp->dpi));
        if (cx < 128) cx = 128;
        if (cx > a[0].w - 128) cx = a[0].w - 128;
        af.ls.x = cx - 128;
        af.ls.y = cy < a[0].h ? cy : a[0].h - 1;
    }
    af.ls.depth16 = 1;
    double g[3];
    ae_gain(film, g);
    for (int c = 0; c < 3; c++) af.ls.exp[c] = af.ls.cap[c] = g[c];
    af.ls.exp[3] = af.ls.cap[3] = 1.0;

    if (s->carriage_out) {
        if (!avl_paper_feed(s)) return fail(s);
        s->carriage_out = 0;
    }

    /* FocusAE: one line at focus 0, exposure so that the maximum is ~50 % */
    if (cb) cb(ud, "focus exposure", -1);
    int r = DSD4_OK;
    if (!pre_focus_scan(s, &af, 0, 3)) { r = fail(s); goto out; }
    uint32_t mx = 0;
    for (unsigned i = 0; i < 256 * 3; i++) if ((af.line[i] >> 8) > mx) mx = af.line[i] >> 8;
    if (!mx) mx = 1;
    double f = (128.0 * 255.0 / 255.0) / (double)mx;
    if (f > 7.0) f = 7.0;
    for (int c = 0; c < 3; c++) af.ls.exp[c] = af.ls.cap[c] = f * g[c];
    dsd4_logf(s, 2, "focus AE: max %u -> factor %.3f", mx, f);

    const int16_t half = 200, lo_lim = -200; /* focus range 400 */
    uint32_t m, bestm = 0;
    int16_t best = 0;
    /* backlash: approach from +250 */
    if (!focus_scan(s, &af, 250, 3, NULL)) { r = fail(s); goto out; }
    av_delay_ticks(6);
    /* coarse: +200 .. -200 step 10 */
    for (int i = lo_lim; i <= half; i += 10) {
        int16_t pos = (int16_t)-i;
        if (!focus_scan(s, &af, pos, 3, &m)) { r = fail(s); goto out; }
        dsd4_logf(s, 3, "focus %4d: %u", pos, m);
        if (m > bestm) { bestm = m; best = pos; }
        if (cb) cb(ud, "autofocus (coarse)", (i - lo_lim) / 400.0);
    }
    int16_t lo = (int16_t)(best - 50), hi = (int16_t)(best + 50);
    if (lo < lo_lim) { hi = (int16_t)(hi + (lo_lim - lo)); lo = lo_lim; }
    if (hi > half) { lo = (int16_t)(lo - (hi - half)); hi = half; }
    if (!focus_scan(s, &af, (int16_t)(hi + 50), 1, NULL)) { r = fail(s); goto out; }
    av_delay_ticks(6);
    bestm = 0;
    for (int i = lo; i < hi; i += 2) {
        int16_t pos = (int16_t)(hi + lo - i);
        if (!focus_scan(s, &af, pos, 3, &m)) { r = fail(s); goto out; }
        dsd4_logf(s, 3, "focus %4d: %u", pos, m);
        if (m > bestm) { bestm = m; best = pos; }
        if (cb) cb(ud, "autofocus (fine)", (i - lo) / (double)(hi - lo));
    }
    if (!focus_scan(s, &af, (int16_t)(best + 50), 3, NULL)) { r = fail(s); goto out; }
    av_delay_ticks(6);
    if (!focus_scan(s, &af, best, 3, NULL)) { r = fail(s); goto out; }
    dsd4_logf(s, 1, "autofocus: best position %d (sharpness %u)", best, bestm);
    if (best_out) *best_out = best;
out:
    /* HomePosition */
    if (s->scanning) avl_stop_scan(s);
    avl_paper_feed(s);
    s->carriage_out = 0;
    return r;
}

/* --------------------------------------------------------------- scanning */

void dsd4_default_params(dsd4_scan_params *p)
{
    memset(p, 0, sizeof *p);
    p->film = DSD4_FILM_POSITIVE_COLOR;
    p->dpi = 3200;
    p->bits = 16;
    p->focus = DSD4_FOCUS_AUTO;
    p->multi_sample = 1;
}

void dsd4_image_free(dsd4_image *img)
{
    free(img->data);
    memset(img, 0, sizeof *img);
}

/* box downsample of 16-bit interleaved data by an arbitrary ratio */
static uint8_t *resample16(const uint16_t *src, int sw, int sh, int ch, int dw, int dh)
{
    uint16_t *dst = malloc((size_t)dw * dh * ch * 2);
    if (!dst) return NULL;
    const double fx = (double)sw / dw, fy = (double)sh / dh;
    for (int y = 0; y < dh; y++) {
        int y0 = (int)(y * fy), y1 = (int)((y + 1) * fy);
        if (y1 <= y0) y1 = y0 + 1;
        if (y1 > sh) y1 = sh;
        for (int x = 0; x < dw; x++) {
            int x0 = (int)(x * fx), x1 = (int)((x + 1) * fx);
            if (x1 <= x0) x1 = x0 + 1;
            if (x1 > sw) x1 = sw;
            for (int c = 0; c < ch; c++) {
                uint64_t sum = 0;
                for (int yy = y0; yy < y1; yy++)
                    for (int xx = x0; xx < x1; xx++) sum += src[((size_t)yy * sw + xx) * ch + c];
                dst[((size_t)y * dw + x) * ch + c] = (uint16_t)(sum / ((uint64_t)(y1 - y0) * (x1 - x0)));
            }
        }
    }
    return (uint8_t *)dst;
}

int dsd4_scan(dsd4 *s, const dsd4_scan_params *sp, dsd4_image *img, dsd4_progress_fn cb, void *ud)
{
    memset(img, 0, sizeof *img);
    if (sp->bits != 8 && sp->bits != 16) return DSD4_ERR_PARAM;
    const int film = film_type_code(sp->film);
    const int gray = !is_color(film);
    line_setting ls;
    memset(&ls, 0, sizeof ls);
    ls.mode = sp->preview ? MODE_PREVIEW : MODE_FINAL;
    ls.format_aps = s->holder_type == 1;
    ls.film = film;
    ls.frame = (uint16_t)(sp->frame > 0 ? sp->frame : 1);
    ls.gray = gray;
    ls.depth16 = 1; /* always scan 16 bit; 8 bit output is derived on the host */
    ls.samples = (uint8_t)(sp->multi_sample > 0 ? sp->multi_sample : 1);
    if (ls.samples != 1 && ls.samples != 2 && ls.samples != 4 && ls.samples != 8) return DSD4_ERR_PARAM;

    /* the film format follows the inserted holder: read it before validating */
    dsd4_holder_info hi;
    int r = dsd4_get_holder(s, &hi);
    if (r) return r;
    ls.format_aps = hi.raw_type == 1;
    r = film_format_check(s, &ls);
    if (r) return r;

    if (sp->frame > 0) {
        if (cb) cb(ud, "moving to frame", -1);
        r = dsd4_select_frame(s, sp->frame);
        if (r) return r;
    }

    /* resolution: the final scan uses 800/1600/3200 dpi on the scanner; other
     * resolutions are scanned at the next native step and downsampled. */
    ability_area a[4];
    image_ability(ls.format_aps, a);
    int out_dpi = sp->preview ? (sp->dpi > 0 && sp->dpi < a[1].dpi ? sp->dpi : a[1].dpi) : sp->dpi;
    if (out_dpi < 50 || out_dpi > 3200) return DSD4_ERR_PARAM;
    int scan_dpi = sp->preview ? (out_dpi <= 100 ? 100 : out_dpi <= 200 ? 200 : out_dpi <= 400 ? 400 : a[1].dpi)
                               : (out_dpi <= 800 ? 800 : out_dpi <= 1600 ? 1600 : 3200);
    const double k = (double)scan_dpi / out_dpi;

    /* area at the scan resolution */
    uint32_t fw = (uint32_t)((uint64_t)a[0].w * scan_dpi / 3200), fh = (uint32_t)((uint64_t)a[0].h * scan_dpi / 3200);
    uint32_t x = 0, y = 0, w = fw, h = fh;
    int crop = !sp->preview && sp->w > 0 && sp->h > 0;
    uint32_t want_x = 0, want_w = 0, want_h = 0; /* requested area at the scan resolution */
    if (crop) {
        if (sp->x < 0 || sp->y < 0) return DSD4_ERR_PARAM;
        double fx = sp->x * k, fy = sp->y * k, fwd = sp->w * k, fhd = sp->h * k;
        if (fx + fwd > fw + 0.5 || fy + fhd > fh + 0.5) return DSD4_ERR_PARAM;
        want_x = (uint32_t)fx;
        x = want_x;
        y = (uint32_t)fy;
        want_w = (uint32_t)ceil(fwd);
        want_h = (uint32_t)ceil(fhd);
        h = want_h;
        if (y + h > fh) h = fh - y;
        w = (want_w + 3u) & ~3u; /* the scanner aligns colour lines to 4 pixels */
        if (w > fw) w = fw & ~3u;
        if ((uint64_t)x + w > fw) x = fw - w; /* keep the aligned width inside the frame */
    } else {
        w &= ~3u;
    }
    if ((uint64_t)x + w > fw || (uint64_t)y + h > fh || w < 4 || h < 1) return DSD4_ERR_PARAM;
    ls.x = x; ls.y = y; ls.w = w; ls.h = h;
    ls.dpi = (uint16_t)scan_dpi;

    /* exposure: standard film gain x user factor; cap base = standard gain */
    double stdg[3];
    dsd4_film_default_exposure(sp->film, stdg);
    for (int c = 0; c < 3; c++) {
        ls.cap[c] = stdg[c];
        ls.exp[c] = sp->exposure[c] > 0 ? sp->exposure[c] : stdg[c];
    }
    ls.exp[3] = ls.cap[3] = 1.0;

    /* focus */
    int focus = sp->focus;
    if (focus == DSD4_FOCUS_AUTO) {
        int best = 0;
        r = dsd4_autofocus(s, sp, &best, cb, ud);
        if (r) return r;
        focus = best;
    } else if (focus == DSD4_FOCUS_KEEP) {
        focus = s->last_focus;
    }
    if (focus < -250 || focus > 250) return DSD4_ERR_PARAM;
    ls.focus = (int16_t)focus;

    /* PrepareForScan */
    if (cb) cb(ud, "preparing", -1);
    if (!scan_calibration_check(s)) return fail(s);
    if (!update_setup_data(s)) return fail(s);
    double dg[3];
    check_gain_limit(s, &ls, dg);
    if (sp->raw) dg[0] = dg[1] = dg[2] = 1.0;
    dsd4_logf(s, 2, "exposure %.3f/%.3f/%.3f digital gain %.3f/%.3f/%.3f focus %d", ls.exp[0], ls.exp[1],
              ls.exp[2], dg[0], dg[1], dg[2], focus);
    if (ls.mode == MODE_FINAL) {
        if (s->carriage_out) {
            if (!avl_paper_feed(s) && s->last_error && s->last_error != 0xa2) return fail(s);
            s->carriage_out = 0;
        }
        if (!is_posi_std_exp(ls.exp)) {
            double de[3] = { ls.exp[0], ls.exp[1], ls.exp[2] };
            if (gray) { de[0] = 1.0; de[1] = ls.exp[0]; de[2] = 1.0; }
            if (cb) cb(ud, "dark calibration", -1);
            if (!avl_dark_calibration(s, de)) return fail(s);
        }
    }

    const int ch = gray ? 1 : 3;
    const size_t stride = (size_t)w * ch * 2;
    uint8_t *buf = malloc(stride * h);
    if (!buf) return DSD4_ERR_NOMEM;
    progress pg = { cb, ud, sp->preview ? "preview" : "scanning" };
    int scanned = line_scan(s, &ls, buf, stride, h, dg, &pg);
    int err = s->last_error;
    /* ScanExecution -> HomePosition: return the carriage after every scan */
    if (cb) cb(ud, "returning carriage", -1);
    if (avl_paper_feed(s)) s->carriage_out = 0;
    if (!scanned) {
        free(buf);
        s->last_error = err;
        return fail(s);
    }

    img->channels = ch;
    img->bits = 16;
    img->focus = focus;
    for (int c = 0; c < 3; c++) img->digital_gain[c] = dg[c];
    if (k != 1.0) {
        int dw = (int)(w / k), dh = (int)(h / k);
        if (dw < 1) dw = 1;
        if (dh < 1) dh = 1;
        uint8_t *rs = resample16((uint16_t *)buf, (int)w, (int)h, ch, dw, dh);
        free(buf);
        if (!rs) return DSD4_ERR_NOMEM;
        buf = rs;
        w = (uint32_t)dw;
        h = (uint32_t)dh;
    }
    if (crop) {
        /* trim the alignment padding so the result is exactly the requested size */
        const size_t bpp = (size_t)ch * 2;
        uint32_t ox = (uint32_t)((want_x - x) / k + 0.5);
        uint32_t ow = (uint32_t)sp->w, oh = (uint32_t)sp->h;
        if (ox > w) ox = w;
        if (ow > w - ox) ow = w - ox;
        if (oh > h) oh = h;
        if (ow != w || oh != h) {
            for (uint32_t yy = 0; yy < oh; yy++)
                memmove(buf + (size_t)yy * ow * bpp, buf + ((size_t)yy * w + ox) * bpp, (size_t)ow * bpp);
            w = ow;
            h = oh;
        }
    }
    img->width = (int)w;
    img->height = (int)h;
    img->dpi = out_dpi;
    img->stride = (size_t)w * ch * 2;
    img->data = buf;
    if (sp->bits == 8) {
        uint16_t *s16 = (uint16_t *)buf;
        size_t n = (size_t)w * h * ch;
        for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)(s16[i] >> 8);
        img->bits = 8;
        img->stride = (size_t)w * ch;
    }
    return DSD4_OK;
}
