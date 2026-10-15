/*
 * Port of the "AVI" scanner library linked into MFSBaseLib2891.
 *
 * Every function here corresponds to a function of the same name in the
 * original PowerPC code (InitializeDriver, GetScannerAbilityEx,
 * SetScanParameter, SetScanParameterEx, SendAsicShaddingData, StartScan,
 * ReadScan, GetUnpackData, ...).  The arithmetic is reproduced exactly,
 * including the original's integer truncation and single precision float
 * math, so that the bytes sent to the scanner match the original driver.
 *
 * Code paths that the Dual IV never takes (line packing [aa78 & 0x10],
 * infrared [aa94], the scanner-type-5 timing model, X scaling of non multiple
 * of 4 widths) are rejected explicitly instead of being ported.
 */
#include "dsd4_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

unsigned av_now_ticks(void);
#define ticks_now av_now_ticks

/* FUN_10009b00: double -> unsigned conversion used throughout */
static uint32_t d2u(double d)
{
    if (!(d > 0)) return 0;
    if (d >= 4294967295.0) return 0xffffffffu;
    return (uint32_t)d;
}

/* ------------------------------------------------------------------ setup */

/* InitializeDriver */
int avl_initialize_driver(dsd4 *s)
{
    s->ae_override[0] = s->ae_override[1] = s->ae_override[2] = 0.0;
    s->busy = 0;
    s->first_init = 1;
    s->state = 0;
    s->speed_mode = 1;
    s->ir_mode = 0;
    s->load_cache = 1;
    s->xsub = 1;
    s->ulx3200 = 0;
    for (int i = 0; i < 3; i++) s->white_target[i] = 64000;
    s->base_unit = 300;
    s->last_focus = -150; /* [a202] initial value 0xff6a */
    return 1;
}

/* GetScannerAbilityEx: parse the 96 byte INQUIRY answer. */
int avl_get_scanner_ability(dsd4 *s)
{
    if (!s->first_init && !av_is_ready(s, 0x168)) return 0;
    uint8_t q[96] = { 0 };
    if (!av_inquiry(s, q)) return 0;
    memcpy(s->inquiry, q, sizeof q);

    ability_t a;
    memset(&a, 0, sizeof a);
    a.flags2 = 0xfa716bff;
    const int ext = (q[39] & 4) != 0; /* extended (0x1e byte) inquiry */
    if (ext) {
        if (q[50] & 0x10) s->caps |= 0x002;
        if (q[51] & 0x04) s->caps |= 0x001;
        if (q[51] & 0x01) s->caps |= 0x008;
        if (q[52] & 0x02) s->caps |= 0x040;
        if (q[93] & 0x80) s->caps |= 0x020;
        if (q[93] & 0x20) s->caps |= 0x100;
        if (q[93] & 0x40) s->caps |= 0x400;
        if (q[52] & 0x01) s->caps |= 0x080;
        a.f7a = 0; a.f7c = 0;
    }
    memcpy(a.vendor, q + 8, 16);
    memcpy(a.product, q + 16, 8);
    memcpy(a.revision, q + 32, 4);
    s->scanner_type = q[91];
    if (s->scanner_type == 3) { s->cal_mode = 1; s->ir_mode = 0; }
    else if (s->scanner_type == 6) s->cal_mode = 3;
    a.f62 = 0xe0;
    a.flags = 0x10000;
    if (!(q[39] & 0x80)) a.flags |= 0x1 | 0x4;
    if (q[39] & 0x40) a.flags |= 0x100000 | 0x200000;
    if (ext) {
        if (q[50] & 0x80) { a.flags |= 0x10000000; a.flags2 |= 0x100000 | 0x200000; }
        if (a.flags & 4) { a.f7a = be16(q + 77); a.f7c = be16(q + 79); a.flags2 |= 0x1000000; }
        if (q[50] & 0x40) a.flags |= 0x800000;
        if (!(q[50] & 0x08)) a.flags |= 0x400000 | 0x40000 | 0x80000;
        else a.flags &= ~0x400000u;
        if (q[50] & 0x04) a.flags |= 0x1000000;
        if (q[50] & 0x02) a.flags |= 0x2000000;
        if (q[50] & 0x01) a.flags |= 0x4000000;
        if (q[51] & 0x80) a.flags |= 0x2;
        if (q[51] & 0x40) a.flags |= 0x200;
        if (q[51] & 0x20) a.flags |= 0x100;
        if (q[51] & 0x10) a.flags |= 0x20000000;
        if (q[52] & 0x80) a.flags |= 0x10;
        if (q[52] & 0x01) { a.f76 |= 4; s->caps |= 0x80; }
        if (q[52] & 0x40) a.flags |= 0x80;
        a.f76 |= 0x10;
        if (be16(q + 87) == 0xffff) a.flags |= 0x400;
        a.shading_target = be16(q + 75);
    }
    s->optical_res = (uint16_t)(q[37] * 100);
    a.optical = s->optical_res;
    if (ext) { a.optical = be16(q + 89); s->optical_res = a.optical; }
    a.max_x = a.max_y = (uint16_t)(q[38] * 100);
    if (q[39] & 0x20) { a.max_x = be16(q + 40); a.max_y = be16(q + 42); }
    a.max_x2 = a.max_y2 = (uint16_t)(q[38] * 100);
    if (q[39] & 0x20) { a.max_x2 = be16(q + 44); a.max_y2 = be16(q + 46); }
    if (a.max_x2 < a.optical) a.max_x = a.max_y = a.max_x2 = a.max_y2 = a.optical;
    a.f62 = 0xa0;
    if ((q[36] & 0x70) == 0x20) a.f62 |= 0x20 | 0x80;
    else if ((q[36] & 0x70) == 0) a.f62 |= 0x80;
    if ((q[36] & 7) == 1) s->caps |= 0x800; else s->caps &= ~0x800u;
    a.f64 = ext ? q[62] : 0x80;
    if (!(q[36] & 8)) { a.f64 |= 0x80; if (q[36] & 0x80) a.f64 |= 0x20; }
    else { a.f64 |= 0xff7f; if (q[36] & 0x80) a.f64 |= 0x40; }
    a.f68 = ext ? q[60] : 0xc0;
    a.f69 = ext ? q[61] : 0x90;
    a.f6a = ext ? q[63] : 5;
    a.f6b = ext ? q[64] : 1;
    a.align_lineart = 8; a.align_gray = 1; a.align_color = 4; a.f6f = 0x20; a.f70 = 0x20;
    if (ext && q[54] != 1) {
        a.align_lineart = q[57];
        a.align_gray = q[55];
        a.f71 = q[56];
        a.align_color = q[54];
        a.f6f = q[58];
        a.f70 = q[59];
    }
    s->line_distance = 0;
    if (q[50] & 0x20) { s->line_distance = 0x11; s->caps |= 0x10; }
    if (ext) {
        if (!(q[50] & 0x20)) s->b200 = 0x10;
        else { s->line_distance = (uint16_t)(q[53] + 1); s->b200 = q[53]; }
    }
    a.max_pixels = be16(q + 81);
    a.max_lines = be16(q + 83);
    s->ab = a;
    s->ability_valid = 1;
    dsd4_logf(s, 2, "ability: optical %u dpi, caps 0x%x, flags 0x%x, f64 0x%x, align %u/%u/%u, type %d",
              a.optical, s->caps, a.flags, a.f64, a.align_lineart, a.align_gray, a.align_color, s->scanner_type);
    if (s->caps & 0x80) {
        uint8_t nvm[0x80];
        if (av_read(s, DT_NVM, sizeof nvm, nvm))
            dsd4_logf(s, 2, "NVM word 0x14 = %u", be16(nvm + 0x14));
    }
    return 1;
}

/* GetCalibrationFormat: READ 0x60, 32 bytes.  Bytes 9..20 are big-endian,
 * bytes 21..30 little-endian (byte swapped by the original on PowerPC). */
int avl_get_calibration_format(dsd4 *s, uint8_t fmt[32])
{
    if (!av_is_ready(s, 0x168)) return 0;
    memset(fmt, 0, 32);
    if (!av_read(s, DT_CALIB_FORMAT, 32, fmt)) return 0;
    dsd4_logf(s, 2, "calibration format: %u px, %u B/ch, %u lines, flags %02x, ability %02x, "
              "white %u/%u/%u dark %u/%u/%u exp %u/%u/%u max %u min %u",
              be16(fmt), fmt[2], fmt[3], fmt[4], fmt[5], be16(fmt + 9), be16(fmt + 11), be16(fmt + 13),
              be16(fmt + 15), be16(fmt + 17), be16(fmt + 19), le16(fmt + 21), le16(fmt + 23),
              le16(fmt + 25), le16(fmt + 27), le16(fmt + 29));
    return 1;
}

/* ResetToInitialState: SEND 0x92 */
int avl_reset_to_initial_state(dsd4 *s, uint8_t mode)
{
    uint8_t b[8] = { mode > 3 ? 3 : mode };
    return av_send(s, DT_RESET, 0, 8, b);
}

/* ----------------------------------------------------- shading table cache
 * The original keeps the final dark/white tables in "darkAvg.bin" and
 * "whiteAvg.bin" (0x4b00 bytes each) in its preferences folder and reloads
 * them when the scanner reports it is already calibrated. */
static void cache_path(char *out, size_t n, const char *name)
{
    dsd4_support_path(name, out, n);
}

static int no_cache(void)
{
    const char *e = getenv("DSD4_NO_CACHE");
    return e && *e && strcmp(e, "0");
}

static void save_raw(dsd4 *s, const char *name, const void *data, size_t n)
{
    char dir[1024], path[1024];
    if (no_cache()) return;
    cache_path(dir, sizeof dir, "");
    for (char *q = dir + 1; *q; q++) /* mkdir -p */
        if (*q == '/') { *q = 0; mkdir(dir, 0755); *q = '/'; }
    mkdir(dir, 0755);
    cache_path(path, sizeof path, name);
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fwrite(data, 1, n, f);
    fclose(f);
    dsd4_logf(s, 3, "saved %s", path);
}

static int read_raw(const char *name, void *data, size_t n)
{
    char path[1024];
    if (no_cache()) return 0;
    cache_path(path, sizeof path, name);
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    size_t r = fread(data, 1, n, f);
    fclose(f);
    return r == n;
}

/* CheckScannerState */
int avl_check_scanner_state(dsd4 *s, int what, uint8_t *state)
{
    if (what == 1) {
        if (*state >= 4) return 0;
        s->state = *state;
        return 1;
    }
    if (what != 2) return 0;
    uint8_t fmt[32];
    if (!avl_get_calibration_format(s, fmt)) return 0;
    if (fmt[4] == 2) *state = 3;
    else if (fmt[4] == 1) { *state = 0; s->load_cache = 0; }
    else *state = 2;
    s->state = *state;
    if (s->load_cache) {
        if (!read_raw("darkAvg.bin", s->dark, sizeof s->dark) ||
            !read_raw("whiteAvg.bin", s->white, sizeof s->white)) {
            *state = 3;
            return 1;
        }
        s->load_cache = 0;
        s->have_shading = 1;
        s->white_target[0] = (uint16_t)(be16(fmt + 9) << 8);
        s->white_target[1] = (uint16_t)(be16(fmt + 11) << 8);
        s->white_target[2] = (uint16_t)(be16(fmt + 13) << 8);
        s->white_target[3] = (uint16_t)(be16(fmt + 9) << 8);
        dsd4_logf(s, 2, "loaded cached shading tables");
    }
    return 1;
}

/* FindScanner */
int avl_find_scanner(dsd4 *s)
{
    uint8_t fmt[32];
    if (!avl_get_calibration_format(s, fmt)) return 0;
    s->white_target[0] = be16(fmt + 9);
    s->white_target[1] = be16(fmt + 11);
    s->white_target[2] = be16(fmt + 13);
    return 1;
}

/* ---------------------------------------------------------------- warm up */

/* MyGetRawDataInfo: returns the average of channel 0 of one raw line. */
static int get_raw_data_info(dsd4 *s, uint16_t *avg0)
{
    uint8_t light = 0, info[0x20];
    if (!av_read(s, DT_LIGHT_STATUS, 1, &light)) return 0;
    if (!av_read(s, DT_RAW_INFO, 0x20, info)) return 0;
    uint32_t pixels = be16(info), channels = info[2], lines = info[3];
    if (info[5] & 8) { pixels *= 3; lines /= 3; }
    uint32_t size = lines * pixels * channels;
    if (!size || !channels) { *avg0 = 0; return 1; }
    uint8_t *raw = malloc(size);
    if (!raw) { s->last_error = AVE_NOMEM; return 0; }
    if (!av_read(s, DT_RAW_LINE, size, raw)) { free(raw); return 0; }
    uint32_t sum = 0;
    for (uint32_t i = 0; i < pixels; i++) sum += raw[i * channels];
    *avg0 = (uint16_t)(uint8_t)(sum / pixels);
    free(raw);
    return 1;
}

/* GetLightStatus */
static int get_light_status(dsd4 *s, uint8_t *st)
{
    if (!av_read(s, DT_LIGHT_STATUS, 1, st)) return 0;
    if (*st < 4) {
        if (*st > 1) *st = 3;
    } else if (*st < 6) {
        s->last_error = AVE_LAMP;
        return 0;
    }
    return 1;
}

/* WarmUpCheck: wait (max 120 s) until the lamp is no longer warming up, then
 * until two raw readings taken 10 s apart agree within 97 %. */
static int warm_up_check(dsd4 *s, dsd4_progress_fn cb, void *ud)
{
    unsigned t0 = ticks_now();
    uint8_t st = 0;
    do {
        if (!get_light_status(s, &st)) return 0;
        if (ticks_now() - t0 > 0x1c20) return 0;
        if (st == 3) {
            if (cb) cb(ud, "warming up lamp", (ticks_now() - t0) / (double)0x1c20);
            av_delay_ticks(30);
        }
    } while (st == 3);
    uint16_t prev, cur;
    if (!get_raw_data_info(s, &prev)) return 0;
    for (;;) {
        for (unsigned w = ticks_now(); ticks_now() - w < 0x259;) {
            if (cb) cb(ud, "stabilizing lamp", (ticks_now() - t0) / (double)0x1c21);
            av_delay_ticks(30);
        }
        if (!get_raw_data_info(s, &cur)) return 0;
        unsigned ratio;
        if (cur == 0 || prev == 0) ratio = 1;
        else if (cur < prev) ratio = cur * 100u / prev;
        else ratio = prev * 100u / cur;
        dsd4_logf(s, 2, "lamp stability %u -> %u (%u%%)", prev, cur, ratio);
        if (ratio > 0x61) return 1;
        prev = cur;
        if (ticks_now() - t0 >= 0x1c21) return 1;
    }
}

/* -------------------------------------------------------------- calibration */

/* SETTempScanParameter: one full width line at optical resolution. */
void avl_temp_scan_parameter(dsd4 *s, sp_t *p)
{
    memset(p, 0, sizeof *p);
    p->xres = p->yres = s->ab.optical;
    p->lines = 1;
    p->pixels = 0xc30;
    p->ulx = p->uly = 0;
    p->length = (uint16_t)((p->xres + p->lines * 300u - 1) / p->xres);
    p->width = (uint16_t)((p->yres + p->pixels * 300u - 1) / p->yres);
    if (!p->length) p->length = 1;
    p->mode = 4;
    p->b0d = 0;
    p->bits = s->cal_mode == 3 ? 0x30 : 0x18;
    p->b0f = 5;
    p->highlight = 0xff;
    p->shadow = 0;
    p->flags = 0x9008 | (s->cal_mode == 2 ? SPF_IR : 0);
    p->focus = 0;
    p->exp_r = p->exp_g = p->exp_b = 100;
    p->exp_ir = 100;
}

/* ReadCaliData / ReadDarkCaliData */
static uint8_t *read_cali_data(dsd4 *s, uint8_t fmt[32], uint32_t *count, uint8_t dtc)
{
    uint32_t ppl = be16(fmt);
    uint32_t size = (uint32_t)fmt[3] * ppl * fmt[2];
    if (s->image_mode == 4 || (fmt[5] & 8)) ppl *= 3;
    fmt[3] = fmt[3] / 3;
    *count = ppl;
    if (s->cal_mode == 2) { s->last_error = AVE_PARAM; return NULL; } /* IR layout: not on Dual IV */
    uint8_t *p = malloc(size ? size : 1);
    if (!p) { s->last_error = AVE_NOMEM; return NULL; }
    if (!av_read(s, dtc, size, p)) { free(p); return NULL; }
    return p;
}

/* CalibrationData: white (and optional dark) shading. */
static int build_and_send_shading_table(dsd4 *s, const uint8_t fmt[32], uint32_t count, uint16_t *dark, uint16_t *white)
{
    const unsigned ch = s->cal_mode == 2 ? 4 : 3;
    uint32_t ability = be32(fmt + 4);
    if (ability & 0x40000) cal_dark_shading(s, fmt, count, dark);
    cal_dark_smooth(dark, count);
    cal_white_shading(s, fmt, count, white);
    if (s->cal_mode == 3) {
        if (ability & 0x40000) {
            unsigned c = 0;
            for (uint32_t i = 0; i < count; i++) {
                if (i / ch < SHD_W) s->dark_raw[c][i / ch] = dark[i];
                if (++c == ch) c = 0;
            }
        }
        unsigned c = 0;
        for (uint32_t i = 0; i < count; i++) {
            if (i / ch < SHD_W) s->white_raw[c][i / ch] = white[i];
            if (++c == ch) c = 0;
        }
    }
    /* final tables are shifted one pixel to the left, last pixel repeated */
    uint32_t n = count / 3;
    if (n > SHD_W) n = SHD_W;
    for (unsigned c = 0; c < 3; c++)
        for (uint32_t i = 0; i < n; i++) {
            uint32_t src = (i == n - 1) ? n - 1 : i + 1;
            s->dark[c][i] = s->dark_raw[c][src];
            s->white[c][i] = s->white_raw[c][src];
        }
    s->have_shading = 1;
    save_raw(s, "darkAvg.bin", s->dark, sizeof s->dark);
    save_raw(s, "whiteAvg.bin", s->white, sizeof s->white);
    return 1;
}

static int calibration_data(dsd4 *s)
{
    uint8_t fmt[32];
    if (!avl_get_calibration_format(s, fmt)) return 0;
    uint8_t dtc = (s->image_mode == 4 || (be32(fmt + 4) & 0x80000)) ? DT_CALIB_COLOR : DT_CALIB_GRAY;
    if (!av_is_ready(s, 0x168)) return 0;
    uint32_t count = 0, dcount = 0;
    uint8_t *raw = read_cali_data(s, fmt, &count, dtc);
    if (!raw) return 0;
    uint16_t *white = cal_average(fmt, count, raw);
    free(raw);
    if (!white) { s->last_error = AVE_NOMEM; return 0; }
    fmt[3] = (uint8_t)(fmt[3] * 3);
    s->cal_count = count;
    if (!avl_paper_feed(s) || !av_is_ready(s, 0x168)) { free(white); return 0; }
    uint16_t *dark = NULL;
    if (be32(fmt + 4) & 0x40000) {
        if (!av_is_ready(s, 0x168)) { free(white); return 0; }
        uint8_t *draw = read_cali_data(s, fmt, &dcount, DT_CALIB_DARK);
        if (!draw) { free(white); return 0; }
        dark = cal_average(fmt, count, draw);
        free(draw);
        if (!dark) { free(white); s->last_error = AVE_NOMEM; return 0; }
        fmt[3] = (uint8_t)(fmt[3] * 3);
        for (uint32_t i = 0; i < count; i++) white[i] = (uint16_t)(white[i] - dark[i]);
    } else {
        dark = calloc(count ? count : 1, sizeof *dark);
    }
    s->load_cache = 0;
    int ok = build_and_send_shading_table(s, fmt, count, dark, white);
    free(dark);
    free(white);
    return ok;
}

/* DoFSV1Calibration: program the calibration window. */
static int do_fsv1_calibration(dsd4 *s, int16_t focus)
{
    sp_t p;
    s->ab.optical = 0xc80; /* [ab1c] forced to 3200 */
    avl_temp_scan_parameter(s, &p);
    p.focus = focus;
    if (!av_test_unit_ready(s)) return 0;
    avl_set_scan_parameter(s, &p);
    return av_test_unit_ready(s);
}

/* DoCalibration */
int avl_do_calibration(dsd4 *s)
{
    s->calibrating = 1;
    s->init_calibration = 1;
    if (!do_fsv1_calibration(s, 0)) return 0;
    int ok = calibration_data(s);
    s->calibrating = 0;
    s->init_calibration = 0;
    dsd4_logf(s, 1, "calibration %s", ok ? "done" : "failed");
    return ok;
}

/* DoInitializeScanner */
static int do_initialize_scanner(dsd4 *s, dsd4_progress_fn cb, void *ud)
{
    if (!avl_reset_to_initial_state(s, 0)) return 0;
    if (!av_test_unit_ready(s)) return 0;
    if (!warm_up_check(s, cb, ud)) return 0;
    if (!avl_reset_to_initial_state(s, 0)) return 0;
    if (!av_test_unit_ready(s)) return 0;
    av_delay_ticks(0x3d);
    if (cb) cb(ud, "calibrating", -1);
    s->calibrating = 1;
    s->init_calibration = 1;
    if (!do_fsv1_calibration(s, 0)) return 0; /* DoFSV1Calibration(h, 99, 0): focus is the 3rd arg */
    if (!calibration_data(s)) return 0;
    s->calibrating = 0;
    s->init_calibration = 0;
    s->state = 2;
    return 1;
}

/* InitializeScanner */
int avl_initialize_scanner(dsd4 *s)
{
    int ok = av_test_unit_ready(s);
    if (ok) {
        if (!s->first_init) {
            uint8_t st;
            s->state = (avl_check_scanner_state(s, 2, &st) && st == 0) ? 1 : 2;
        }
        if (s->state == 1 || s->state == 0) {
            s->ability_valid = 0;
            ok = do_initialize_scanner(s, NULL, NULL);
        }
    }
    return ok;
}

/* exported for the progress-reporting variant used by dsd4_initialize */
int avl_initialize_scanner_cb(dsd4 *s, dsd4_progress_fn cb, void *ud);
int avl_initialize_scanner_cb(dsd4 *s, dsd4_progress_fn cb, void *ud)
{
    int ok = av_test_unit_ready(s);
    if (ok && (s->state == 1 || s->state == 0)) {
        s->ability_valid = 0;
        ok = do_initialize_scanner(s, cb, ud);
    }
    return ok;
}

/* DarkCalibrationData: re-measure dark shading at a non standard exposure. */
int avl_dark_calibration(dsd4 *s, const double exp[3])
{
    if (!av_is_ready(s, 0x168)) return 0;
    sp_t p;
    s->ab.optical = 0xc80;
    avl_temp_scan_parameter(s, &p);
    p.exp_r = (uint16_t)(int)(100.0 * exp[0]);
    p.exp_g = (uint16_t)(int)(100.0 * exp[1]);
    p.exp_b = (uint16_t)(int)(100.0 * exp[2]);
    if (!av_test_unit_ready(s)) return 0;
    s->calibrating = 1;
    s->init_calibration = 0;
    avl_set_scan_parameter(s, &p);
    s->calibrating = 0;
    uint8_t fmt[32];
    if (!avl_get_calibration_format(s, fmt)) return 0;
    if (!av_test_unit_ready(s)) return 0;
    uint32_t count = 0;
    uint8_t *raw = read_cali_data(s, fmt, &count, DT_CALIB_DARK);
    if (!raw) return 0;
    uint16_t *dark = cal_average(fmt, count, raw);
    free(raw);
    if (!dark) { s->last_error = AVE_NOMEM; return 0; }
    fmt[3] = (uint8_t)(fmt[3] * 3);
    s->cal_count = count;
    /* BuildDarkShadingTable */
    cal_dark_shading(s, fmt, count, dark);
    cal_dark_smooth(dark, count);
    const unsigned ch = 3;
    if (s->cal_mode == 3) {
        unsigned c = 0;
        for (uint32_t i = 0; i < count; i++) {
            if (i / ch < SHD_W) s->dark_raw[c][i / ch] = dark[i];
            if (++c == ch) c = 0;
        }
    }
    uint32_t n = count / 3;
    if (n > SHD_W) n = SHD_W;
    for (unsigned c = 0; c < 3; c++)
        for (uint32_t i = 0; i < n; i++)
            s->dark_nonstd[c][i] = s->dark_raw[c][i == n - 1 ? n - 1 : i + 1];
    free(dark);
    return 1;
}

/* ------------------------------------------------------- mechanics, focus */

int avl_paper_feed(dsd4 *s)
{
    if (!av_is_ready(s, 0x168)) return 0;
    return av_object_position(s, 2);
}

/* ReadHolderInfo: READ 0x87 (8 bytes) followed by a readiness check */
int avl_read_holder(dsd4 *s, uint8_t info[8])
{
    memset(info, 0, 8);
    if (!av_read(s, DT_HOLDER, 8, info)) return 0;
    return av_is_ready(s, 0x168);
}

/* SendHolderControl: frame 1..N moves to that frame, 0xff ejects. */
int avl_send_holder_control(dsd4 *s, uint8_t frame)
{
    if (s->first_init && !av_is_ready(s, 0x168)) return 0;
    uint8_t info[8];
    if (!avl_read_holder(s, info)) return 0;
    if (info[2] < frame && frame != 0xff) {
        s->last_error = s->first_init ? 0xcc : AVE_PARAM;
        return 0;
    }
    uint8_t b[8] = { info[0], frame };
    return av_send(s, DT_HOLDER, 0, 8, b);
}

/* SetOpticalControl: SEND 0x88, big-endian word: pos|0x8000 for pos > 0,
 * -pos otherwise. */
int avl_set_optical_control(dsd4 *s, int16_t pos)
{
    if (!av_is_ready(s, 600)) return 0;
    if (pos == s->last_focus) return 1;
    uint8_t b[8] = { 0 };
    uint16_t w = pos < 1 ? (uint16_t)(-pos) : (uint16_t)(pos | 0x8000);
    put_be16(b, w);
    int ok = av_send(s, DT_FOCUS, 0, 8, b);
    s->last_focus = pos;
    return ok;
}

/* Convert16BitGammaTable + SetGammaTable */
int avl_set_gamma_table(dsd4 *s, const uint8_t *t, uint8_t channel, uint32_t size)
{
    uint8_t out[0x200];
    uint32_t per = size >> 8;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t sum = 0;
        for (uint32_t k = 0; k < per; k++) sum += t[k + i * per];
        uint16_t v;
        if (i == 0) v = (uint16_t)(t[0] << 8);
        else if (i == 1) v = (uint16_t)((sum + t[per] * 0x100u) >> 1);
        else if (i == 0xfe) v = (uint16_t)((sum + t[per * 0xfe] * 0x100u) >> 1);
        else if (i == 0xff) v = (uint16_t)(t[per * 0xff + 0xff] << 8);
        else v = (uint16_t)sum;
        put_le16(out + 2 * i, v);
    }
    uint16_t q = (channel == 0 || channel > 3) ? 2 : channel;
    return av_send(s, DT_GAMMA, (uint16_t)(q - 1), sizeof out, out);
}

/* ---------------------------------------------------- scan parameters */

/* ChkScanMainParm */
static int chk_scan_main_parm(dsd4 *s, sp_t *p)
{
    if (!(p->flags & SPF_PIXELMODE)) {
        if (!p->width || !p->length) return 0;
        p->xres = (uint16_t)((p->width + (uint32_t)p->pixels * s->base_unit - 1) / p->width);
        p->yres = (uint16_t)((p->length + (uint32_t)p->lines * s->base_unit - 1) / p->length);
    }
    if (!p->yres || !p->xres) return 0;
    if (!p->exp_r) p->exp_r = 100;
    if (!p->exp_g) p->exp_g = 100;
    if (!p->exp_b) p->exp_b = 100;
    return 1;
}

static int send_asic_shading_data(dsd4 *s, const sp_t *p);
static int set_scan_parameter_ex(dsd4 *s, sp_t *p);

/* SetScanParameter */
int avl_set_scan_parameter(dsd4 *s, const sp_t *in)
{
    /* MFSScsiGetScannerInfo(): the bus type of a USB scanner is 4 */
    const int bus_type = 4;
    if (!s->ability_valid && !avl_get_scanner_ability(s)) return 0;

    sp_t p = *in;
    if (!(p.flags & SPF_EXTENDED)) { p.nonstd_dark = 0; p.exp_ir = 0; } /* only 0x45 bytes copied */
    if (!p.xres || !p.yres) { s->last_error = AVE_PARAM; return 0; }

    s->ulx3200 = (uint16_t)((uint32_t)p.ulx * 0xc80u / p.xres);
    p.uly = (uint16_t)((uint32_t)p.uly * 0xc80u / p.yres);
    if (bus_type == 2)
        s->one_line = p.multi_sample && p.film_flag == 1 && p.focus_mode != 1;
    else
        s->one_line = p.multi_sample && p.focus_mode != 1 && p.film_flag == 1 && (p.flags & SPF_IR);
    if (s->one_line) { s->last_error = AVE_PARAM; return 0; } /* not used with USB */

    s->gray_from_color = p.mode == 2;
    if (s->gray_from_color) { p.mode = 4; p.bits = (uint8_t)(p.bits * 3); }
    s->base_unit = (p.flags & SPF_BASE2400) ? 0x960 : 300;
    p.ulx = s->ulx3200;
    if (!chk_scan_main_parm(s, &p)) { s->last_error = AVE_PARAM; return 0; }

    s->flag4000 = 0;
    if (!(p.flags & SPF_EXTENDED)) {
        s->caps &= ~0x200u;
    } else {
        s->flag4000 = (p.flags & SPF_4000) != 0;
        if (!s->first_init || !(p.flags & SPF_0800)) s->caps &= ~0x200u;
        else s->caps |= 0x200;
        if (p.flags & SPF_MULTI_SCAN) {
            p.lines = p.multi_lines;
            p.flags &= 0xfffe;
            s->ir_mode = 0;
            s->multi_scan = 1;
        } else {
            s->multi_scan = 0;
        }
    }
    s->yscale = 0;
    s->xscale = 0;
    if (s->ab.f64 & 8) {
        p.b0d = 0;
        if (!s->calibrating || s->init_calibration) avl_set_optical_control(s, p.focus);
        s->focus_flag = (p.flags & SPF_FOCUS) != 0;
    }
    s->req_lines = p.lines;
    uint16_t req_pixels = p.pixels;

    if (s->scanner_type != 5 && !s->speed_mode) {
        s->last_error = AVE_PARAM; /* the speed_mode == 0 timing path is never used */
        return 0;
    }
    if (p.mode > 1) {
        uint32_t r;
        if (!s->flag4000)
            r = p.yres ? p.yres : (p.length + (uint32_t)p.lines * s->base_unit - 1) / p.length;
        else
            r = (p.length + (uint32_t)p.lines * s->base_unit - 1) / p.length;
        r &= 0xffff;
        s->yscale = 1;
        if (p.final_scan != 1) {
            if (r == 100 || r == 200 || r == 400 || r == 800 || r == 0x640 || r == 0xc80) s->yscale = 0;
        } else {
            if (r == 800 || r == 0x640 || r == 0xc80) s->yscale = 0;
        }
        if (p.final_scan == 1) {
            if (r > 0x640) r = 0xc80;
            else if (r > 800) r = 0x640;
            else r = 800;
        } else {
            if (r > 0x640) r = 0xc80;
            else if (r > 800) r = 0x640;
            else if (r > 400) r = 800;
            else if (r > 200) r = 400;
            else if (r > 100) r = 200;
            else r = 100;
        }
        if (p.index_last) { r = 400; s->yscale = 1; }
        s->xsub = (uint16_t)(0xc80 / r);
        if (!p.index_last) {
            if (s->yscale && !p.index_not_last)
                p.lines = (uint16_t)(int)(0.5 + ((double)p.length * (double)r) / (double)p.yres);
        } else {
            p.lines = 0x249;
        }
        p.yres = (uint16_t)r;
    }
    if (s->probe_yres && p.mode > 1 && p.yres == s->ab.optical) {
        /* research probe: a vertical resolution above the optical one, sent
         * as is (the original clamps it and rescales on the host) */
        p.lines = (uint16_t)((uint32_t)p.lines * s->probe_yres / p.yres);
        s->req_lines = p.lines;
        p.yres = s->probe_yres;
        s->yscale = 0;
        s->xsub = 1;
    }
    if (s->b200) s->b202 = (uint16_t)(p.yres / (s->optical_res / s->b200));
    s->scan_lines = p.lines;
    if (s->caps & 0x10) { s->last_error = AVE_PARAM; return 0; } /* line packing: not on Dual IV */

    uint32_t align;
    if (p.mode == 4) align = s->ab.align_color;
    else if (p.mode > 3) { s->last_error = AVE_PARAM; return 0; }
    else if (p.mode < 2) align = s->ab.align_lineart;
    else align = s->ab.align_gray;
    if (!align) align = 1;

    s->line_bytes = (uint16_t)(((uint32_t)p.pixels * p.bits) >> 3);
    if (p.mode < 2) {
        s->line_bytes = (uint16_t)(((uint32_t)p.pixels * p.bits) >> 3);
    } else {
        s->xscale = s->yscale;
        p.pixels = (uint16_t)(align * ((align + ((uint32_t)p.yres * p.width) / p.xres - 1) / align));
        p.xres = p.yres;
        if (s->yscale == 0 && p.pixels < in->pixels) {
            p.pixels = (uint16_t)(align * ((in->pixels + (align - 1)) / align));
            s->xscale = 4;
            p.width = (uint16_t)((p.yres + (uint32_t)p.pixels * s->base_unit - 1) / p.yres);
        }
        if (in->pixels < p.pixels) s->xscale = 1;
        if (s->probe_yres && p.yres == s->probe_yres) {
            /* keep the horizontal sampling at the optical resolution */
            p.xres = s->ab.optical;
            p.pixels = (uint16_t)(align * ((in->pixels + (align - 1)) / align));
            s->xscale = 0;
        }
    }
    s->req_pixels = req_pixels;
    s->scan_line_bytes = (uint16_t)(((uint32_t)p.pixels * p.bits) >> 3);
    s->scan_pixels = p.pixels;
    if (!s->scan_line_bytes) return 0;
    if (p.pixels != req_pixels) {
        p.width = p.pixels;
        p.length = s->scan_lines;
    }
    if (s->xscale == 1) {
        dsd4_logf(s, 1, "width %u is not a multiple of %u: host X scaling is not supported", req_pixels, align);
        s->last_error = AVE_PARAM;
        return 0;
    }
    /* vertical resampling step [b258] (single precision as in the original) */
    const int deep = p.bits == 0x30 || p.bits == 0x10;
    const float scale = deep ? 10000.0f : 1000000.0f;
    float st;
    if (s->yscale == 2)
        st = (scale * (float)(int)(s->scan_lines + 1)) / (float)s->req_lines;
    else
        st = (scale * (float)(int)(s->req_lines + 1)) / (float)s->scan_lines;
    s->ystep = d2u((double)st);
    if (s->ystep > (uint32_t)scale) s->ystep = (uint32_t)scale;
    s->yacc = (uint32_t)scale;
    s->yden = 0;

    int ok = set_scan_parameter_ex(s, &p);
    if (ok && !s->calibrating) ok = send_asic_shading_data(s, &p);
    dsd4_logf(s, 2, "SetScanParameter %s: %ux%u px @ %u/%u dpi, line %u B (out %u B), yscale %u xscale %u",
              ok ? "ok" : "failed", s->scan_pixels, s->scan_lines, p.xres, p.yres,
              s->scan_line_bytes, s->line_bytes, s->yscale, s->xscale);
    return ok;
}

/* SetScanParameterEx: build and send the SET WINDOW block. */
static int set_scan_parameter_ex(dsd4 *s, sp_t *p)
{
    uint8_t w[0x48];
    memset(w, 0, sizeof w);
    uint8_t *d = w + 8; /* window descriptor */
    s->focus_param = (uint16_t)p->focus;

    int ext = 0;
    uint16_t er = 100, eg = 100, eb = 100;
    uint8_t eir = 100;
    if ((p->flags & SPF_EXTENDED) && (s->ab.flags & 2)) {
        ext = 1;
        if (s->ab.f64 & 8) {
            if (s->ae_override[0] == 0.0) {
                er = p->exp_r; eg = p->exp_g; eb = p->exp_b; eir = p->exp_ir;
            } else {
                er = (uint16_t)(int)(100.0 * s->ae_override[0]);
                eg = (uint16_t)(int)(100.0 * s->ae_override[1]);
                eb = (uint16_t)(int)(100.0 * s->ae_override[2]);
                eir = (uint8_t)(int)(100.0 * s->ae_override[0]);
                s->ae_override[0] = s->ae_override[1] = s->ae_override[2] = 0.0;
            }
        } else {
            er = eg = eb = 0; eir = 0;
        }
    }
    uint32_t xfer = ext ? 0x46 : 0x3b;
    put_be16(w + 6, ext ? 0x3e : 0x33);
    s->multisample = (p->flags & SPF_MULTISAMPLE) != 0;
    if (s->multisample) xfer++;

    put_be16(d + 2, p->xres);
    put_be16(d + 4, p->yres);
    put_be32(d + 6, p->ulx);
    put_be32(d + 10, p->uly);
    put_be32(d + 14, p->width);
    put_be32(d + 18, p->length);
    uint16_t v = (uint16_t)(p->brightness + 0x80);
    if (v > 0xff) v = 0xff;
    d[22] = (uint8_t)v;
    d[23] = 0x80;
    v = (uint16_t)(p->contrast + 0x80);
    if (v > 0xff) v = 0xff;
    d[24] = (uint8_t)v;
    switch (p->mode) {
    case 0: d[25] = 0; break;
    case 1: d[25] = 1; break;
    case 2: case 3: d[25] = 2; break;
    case 4: d[25] = 5; break;
    default: s->last_error = 0x75; return 0;
    }
    s->is16 = 1;
    uint8_t bpc = p->bits;
    if (bpc == 0x18) { bpc = 8; s->is16 = 0; }
    else if (bpc == 0x30 || bpc == 0x10) bpc = 12;
    d[26] = bpc;
    s->lineart = 0;
    uint16_t ht = v; /* the original reuses the contrast temporary here */
    if (d[25] == 0) { ht = 0; s->lineart = 1; }
    else if (d[25] == 1) {
        ht = p->halftone;
        if (s->ab.f6a < p->halftone || p->halftone == 0) ht = (uint16_t)(s->ab.f6a + 1);
        s->lineart = 1;
    }
    put_be16(d + 27, (uint16_t)(ht - 1));
    d[29] = p->invert ? 0x83 : 0x03;
    d[40] = 0xff;
    d[41] = ext ? 0x14 : 9;
    uint8_t bs1 = 0x40;
    s->mirror = 0;
    if (p->b0d == 1) {
        bs1 = 0xc0;
        if (s->caps & ~8u) s->mirror = 1;
    }
    s->image_mode = p->mode;
    s->b0d = p->b0d;
    if (p->mode == 4) bs1 |= 0x20;
    else if (p->mode == 2) {
        switch (p->gray_filter) {
        case 0: bs1 |= 0x30; break; /* (b1ea | 0x10) is never 0 */
        case 1: bs1 |= 0x08; break;
        case 2: bs1 |= 0x10; break;
        case 3: bs1 |= 0x18; break;
        default: break;
        }
    }
    d[42] = bs1;
    d[43] = p->highlight;
    d[44] = p->shadow;
    put_be16(d + 45, (uint16_t)(((uint32_t)p->pixels * p->bits) >> 3));
    put_be16(d + 47, p->lines);
    uint8_t bs2 = p->b0d == 2 ? 0x80 : 0;
    if (p->flags & SPF_0010) bs2 |= 4;
    if (p->flags & SPF_0100) bs2 |= 2;
    uint8_t bs3 = 0;
    if (p->flags & SPF_0004) bs3 |= 8;
    if (p->flags & SPF_MULTI_SCAN) { s->caps |= 0x1000; bs3 |= 4; d[58] = p->multi_scan; }
    else s->caps &= ~0x1000u;
    if (p->flags & SPF_MULTISAMPLE) { bs3 |= 2; d[61] = p->multi_sample; }
    if (p->flags & SPF_0080) bs3 |= 0x80;
    if (p->index_not_last == 1) bs3 |= 0x40;
    if ((p->flags & SPF_IR) && s->cal_mode == 2) { s->ir_mode = 1; bs3 |= 1; }
    else s->ir_mode = 0;
    bs3 |= 0x10;
    s->flag4000 = 0;
    if ((p->flags & SPF_EXTENDED) && (p->flags & SPF_4000)) { s->flag4000 = 1; bs2 |= 8; }
    d[49] = bs2;
    d[50] = eir;
    put_be16(d + 51, er);
    put_be16(d + 53, eg);
    put_be16(d + 55, eb);
    d[57] = bs3;

    if (!av_set_window(s, xfer, w)) return 0;

    s->image_mode = p->mode;
    s->aaa2 = p->mode == 4 ? s->scan_line_bytes / 3 : s->scan_line_bytes;
    s->raw_remaining = (uint32_t)s->scan_line_bytes * s->scan_lines;
    s->out_remaining = (uint32_t)s->line_bytes * s->req_lines;
    if (p->flags & SPF_MULTI_SCAN) {
        s->raw_remaining *= p->multi_scan;
        s->out_remaining *= p->multi_scan;
    }
    if (p->flags & SPF_MULTISAMPLE) {
        s->raw_remaining *= p->multi_sample;
        s->out_remaining *= p->multi_sample;
    }
    if (s->gray_from_color) s->out_remaining /= 3;
    s->ring_count = (uint16_t)((uint32_t)s->line_distance * (p->yres + 1u) / s->ab.optical + 2);
    s->y_line = 0;
    s->leftover = 0;
    return 1;
}

/* SendAsicShaddingData: box-average the 3200 dpi shading tables down to the
 * scan resolution and SEND them (little-endian words, dark = qualifiers 3..5,
 * white = 0..2).  With multi-sampling the tables are repeated. */
static int send_asic_shading_data(dsd4 *s, const sp_t *p)
{
    const uint32_t n = s->scan_pixels;
    const uint32_t sub = s->xsub ? s->xsub : 1;
    uint32_t start = s->ulx3200 + (p->is_aps == 1 ? 0x288 : 0);
    unsigned rep = 1;
    if (!s->ir_mode && p->multi_sample) rep = p->multi_sample == 2 ? 2 : 4;
    if (start + n * sub > SHD_W) {
        dsd4_logf(s, 1, "scan area exceeds the shading table (start %u + %u x %u)", start, n, sub);
        s->last_error = AVE_PARAM;
        return 0;
    }
    uint8_t *buf = malloc((size_t)n * 2 * rep);
    if (!buf) { s->last_error = AVE_ALLOC; return 0; }
    int ok = 1;
    for (int pass = 0; pass < 2 && ok; pass++) {
        const uint16_t (*tab)[SHD_W] = pass == 0 ? (p->nonstd_dark == 1 ? s->dark_nonstd : s->dark) : s->white;
        for (unsigned c = 0; c < 3 && ok; c++) {
            for (uint32_t i = 0; i < n; i++) {
                uint32_t sum = 0;
                for (uint32_t k = 0; k < sub; k++) sum += tab[c][start + i * sub + k];
                put_le16(buf + 2 * i, (uint16_t)(sum / sub));
            }
            for (unsigned r = 1; r < rep; r++) memcpy(buf + (size_t)r * n * 2, buf, (size_t)n * 2);
            ok = av_send(s, DT_SHADING, (uint16_t)(pass == 0 ? 3 + c : c), n * 2 * rep, buf);
        }
    }
    free(buf);
    return ok;
}

/* ---------------------------------------------------- acceleration table */

/* ui_FillTable: exponential speed ramp, single precision as on the PPC. */
static uint16_t ui_fill_table(dsd4 *s, float k)
{
    uint32_t e = s->acc_start;
    uint16_t n = 1;
    float v = (float)s->acc_start;
    s->acc_table[0] = (uint8_t)s->acc_start;
    for (;;) {
        if ((e & 0xffff) == s->acc_end) return n;
        float d = v - (float)s->acc_end;
        v = fmaf(-k, d, v);
        e = (uint32_t)(int)(0.5 + (double)v);
        s->acc_table[n] = (uint8_t)e;
        n++;
        if (n >= 999) return n;
    }
}

static uint32_t li_total_count(dsd4 *s, uint16_t n)
{
    uint32_t t = 0;
    for (uint16_t i = 0; i < n; i++) t += s->acc_table[i];
    return t;
}

/* FixTable: pad the ramp so that a * total is a multiple of b. */
static void fix_table(dsd4 *s, uint16_t n)
{
    uint32_t a = s->acc_a, b = s->acc_b;
    if (!a || !b) return;
    uint32_t t = li_total_count(s, n);
    uint32_t rem = (a * t) % b;
    uint32_t add = rem ? (b - rem) / a : 0;
    for (uint16_t i = 0; (int)i < (int)n - 1 && add; i++) {
        uint32_t room = 0xffu - s->acc_table[i];
        if (add < room) { s->acc_table[i] = (uint8_t)(s->acc_table[i] + add); add = 0; }
        else { s->acc_table[i] = (uint8_t)(s->acc_table[i] + room); add -= room; }
    }
}

/* GetAccTable: req = 24 byte answer of READ 0x6c (big-endian fields). */
uint16_t avl_get_acc_table(dsd4 *s, const uint8_t req[24])
{
    memset(s->acc_table, 0, sizeof s->acc_table);
    uint16_t total = be16(req), ramp = be16(req + 2);
    uint16_t steps = (uint16_t)(total - ramp);
    s->acc_a = be32(req + 4);
    s->acc_b = be32(req + 8);
    s->acc_start = be16(req + 12);
    s->acc_end = be16(req + 14);
    double hi = (double)1.0f, lo = (double)0.001f;
    while (1e-4 < (double)(float)(hi - lo)) {
        double mid = (double)((float)(hi + lo) * 0.5f);
        uint16_t cnt = ui_fill_table(s, (float)mid);
        if (cnt == (uint16_t)(steps + 1)) break;
        if ((uint16_t)(steps + 1) < cnt) lo = mid;
        else hi = mid;
    }
    for (; steps < total && steps + 1 < (int)sizeof s->acc_table; steps++)
        s->acc_table[steps + 1] = s->acc_table[steps];
    fix_table(s, total);
    for (uint16_t i = 0; i < total && i < sizeof s->acc_table; i++) s->acc_table[i]--;
    return total;
}

/* ------------------------------------------------------------ scanning */

static void free_pack_buf(dsd4 *s)
{
    free(s->pack_buf);
    s->pack_buf = NULL;
    free(s->accum);
    s->accum = NULL;
    s->pack_size = 0;
}

/* AllocatePackBuf */
static int allocate_pack_buf(dsd4 *s)
{
    uint32_t sz = s->flag4000 ? (0xa000u / s->scan_line_bytes) * s->scan_line_bytes : 0x10000;
    s->saved_line_bytes = s->line_bytes;
    s->saved_scan_line_bytes = s->scan_line_bytes;
    if (!s->scan_line_bytes) return 0;
    if (s->raw_remaining < sz) sz = s->scan_line_bytes * (s->raw_remaining / s->scan_line_bytes);
    else sz = s->scan_line_bytes * (sz / s->scan_line_bytes);
    if (sz == 0) sz = s->scan_line_bytes; /* line wider than 64 KiB: one line at a time */
    s->pack_buf = malloc(sz + 16);
    s->accum = calloc(0x10000, sizeof *s->accum);
    if (!s->pack_buf || !s->accum) { s->last_error = AVE_NOMEM; free_pack_buf(s); return 0; }
    s->pack_size = sz;
    s->b1d2 = sz / s->scan_line_bytes;
    return 1;
}

/* StartScan: RESERVE, SCAN, then answer the acceleration table requests. */
int avl_start_scan(dsd4 *s)
{
    s->scanning = 1;
    s->read_failed = 0;
    int ok = allocate_pack_buf(s);
    if (ok) {
        ok = 0;
        if (av_reserve(s)) {
            uint8_t flags = (uint8_t)((s->flag4000 ? 0x40 : 0) | (s->speed_mode ? 0x80 : 0));
            if (av_scan(s, flags)) {
                uint8_t idx = 0;
                for (;;) {
                    uint8_t req[0x18] = { 0 };
                    int r = av_read(s, DT_ACCEL_TABLE, sizeof req, req);
                    ok = r;
                    if (req[17] == 0) break; /* no (more) tables wanted */
                    uint16_t total = be16(req), ramp = be16(req + 2);
                    uint16_t lo = be16(req + 12), hi = be16(req + 14);
                    if (lo < hi || hi == 0 || total <= ramp) { ok = 0; break; }
                    if (req[16] == 0 && r) {
                        uint16_t n = avl_get_acc_table(s, req);
                        dsd4_logf(s, 2, "acceleration table %u: %u entries, %u->%u", idx, n, lo, hi);
                        av_send(s, DT_ACCEL_TABLE, idx, n, s->acc_table);
                    }
                    idx++;
                    if (idx >= req[17] || !r) break;
                }
            }
        }
    } else {
        free_pack_buf(s);
    }
    if (!ok) s->scanning = 0;
    return ok;
}

/* GetUnpackData: fill s->chunk_bytes bytes of 'dst'. */
static int get_unpack_data(dsd4 *s, uint8_t *dst)
{
    if (s->yscale == 2) { s->last_error = AVE_PARAM; return 0; }
    s->delivered = 0;
    uint8_t *out = dst;
    const uint32_t samples = (s->gray_from_color ? 1u : 3u) * s->req_pixels; /* uVar7 = iVar12 * [aaa4] */
    const int whole = s->gray_from_color && s->raw_remaining == s->chunk_bytes; /* bVar2 */
    uint32_t want = s->chunk_bytes;
    for (;;) {
        while (s->leftover == 0) {
            s->line_bytes = s->saved_line_bytes;
            s->scan_line_bytes = s->saved_scan_line_bytes;
            uint32_t n = s->pack_size;
            if (s->caps & 0x200) {
                uint32_t m = (uint32_t)s->saved_scan_line_bytes * (s->chunk_bytes / s->saved_line_bytes);
                if (m <= s->pack_size) n = m;
            }
            if (s->raw_remaining < n) n = s->raw_remaining;
            if (n == 0) {
                n = s->raw_remaining;
                if (n == 0) return 1;
            }
            s->raw_remaining -= n;
            uint8_t *b = s->pack_buf;
            if (!av_read(s, DT_IMAGE, n, b)) {
                memset(out, 0xff, s->chunk_bytes - s->delivered);
                return 0;
            }
            if (s->is16) {
                uint16_t *w = (uint16_t *)b;
                for (uint32_t i = 0; i < n / 2; i++) w[i] = le16(b + 2 * i);
            }
            if (s->is16) {
                uint16_t *w = (uint16_t *)b;
                if (s->gray_from_color) {
                    for (uint32_t i = 0; i < n / 6; i++)
                        w[i] = s->cal_mode == 3 ? w[i * 3 + 1] : (uint16_t)(w[i * 3 + 1] << 4);
                    n /= 3;
                    s->scan_line_bytes /= 3;
                    s->line_bytes /= 3;
                } else if (s->cal_mode != 3) {
                    for (uint32_t i = 0; i < n / 2; i++) w[i] = (uint16_t)(w[i] << 4);
                }
            } else if (s->gray_from_color) {
                for (uint32_t i = 0; i < n / 3; i++) b[i] = b[i * 3 + 1];
                n /= 3;
                s->scan_line_bytes /= 3;
                s->line_bytes /= 3;
            }
            /* horizontal: drop the alignment padding of every line */
            uint32_t nl = 0, total = n;
            uint8_t *src = b, *d2 = b;
            uint32_t step = whole ? n : s->scan_line_bytes;
            while (step && step <= n) {
                if (s->xscale == 0 || s->xscale == 4) memmove(d2, src, s->line_bytes);
                d2 += s->line_bytes;
                src += step;
                nl++;
                n -= step;
            }
            uint32_t bytes = whole ? total : (uint32_t)s->line_bytes * nl;
            /* vertical resampling (area averaging), [b20c] == 1 */
            if (s->yscale == 1) {
                uint8_t *o = b, *in = b;
                uint32_t emitted = 0;
                const uint32_t one = s->is16 ? 10000u : 1000000u;
                for (uint32_t left = bytes; left; left -= s->line_bytes) {
                    if (s->yacc < one) {
                        for (uint32_t k = 0; k < samples; k++)
                            s->accum[k] += (s->is16 ? ((uint16_t *)in)[k] : in[k]) * one;
                        s->yden += one;
                    } else {
                        s->yacc -= one;
                        s->yden += one - s->yacc;
                        for (uint32_t k = 0; k < samples; k++) {
                            uint32_t v = s->is16 ? ((uint16_t *)in)[k] : in[k];
                            uint32_t r = (uint32_t)(((uint64_t)s->accum[k] + (uint64_t)(one - s->yacc) * v) / s->yden);
                            if (s->is16) ((uint16_t *)o)[k] = (uint16_t)r;
                            else o[k] = (uint8_t)r;
                            s->accum[k] = s->yacc * v;
                        }
                        s->yden = s->yacc;
                        o += s->line_bytes;
                        emitted++;
                    }
                    in += s->line_bytes;
                    s->yacc += s->ystep;
                    s->y_line++;
                    if (left < s->line_bytes) break;
                }
                bytes = (uint32_t)s->line_bytes * emitted;
            }
            if (s->gray_from_color) {
                s->scan_line_bytes *= 3;
                s->line_bytes *= 3;
            }
            s->leftover = bytes;
            s->leftover_ptr = b;
        }
        if (s->leftover < want) {
            memcpy(out, s->leftover_ptr, s->leftover);
            out += s->leftover;
            want -= s->leftover;
            s->delivered += s->leftover;
            s->leftover = 0;
            continue;
        }
        memcpy(out, s->leftover_ptr, want);
        s->leftover -= want;
        s->leftover_ptr += want;
        s->delivered += want;
        return 1;
    }
}

/* ReadScan: deliver up to 'len' bytes (a whole number of output lines). */
int avl_read_scan(dsd4 *s, void *dst, uint32_t len)
{
    if (!s->scanning) { s->last_error = AVE_NOT_SCANNING; s->busy = 0; return 0; }
    s->busy = 1;
    if (s->out_remaining == 0) { s->last_error = AVE_PARAM; s->busy = 0; return 1; }
    if (s->leftover == 0) {
        s->line_bytes = s->saved_line_bytes;
        s->scan_line_bytes = s->saved_scan_line_bytes;
    }
    if (len % s->line_bytes != 0 && !s->gray_from_color) { s->last_error = AVE_PARAM; s->busy = 0; return 0; }
    uint32_t n = len < s->out_remaining ? len : s->out_remaining;
    if (s->gray_from_color) {
        s->chunk_bytes = (n / s->req_pixels) * s->line_bytes;
        if (len < s->chunk_bytes) s->chunk_bytes = len;
    } else {
        s->chunk_bytes = (n / s->line_bytes) * s->line_bytes;
    }
    int ok = 0;
    uint32_t saved = s->raw_remaining;
    if (!s->read_failed) {
        ok = get_unpack_data(s, dst);
        if (ok) {
            n = s->delivered;
            if (s->caps & 0x1000) s->raw_remaining = saved;
            else s->out_remaining -= n;
        }
    }
    if (!ok) {
        s->read_failed = 1;
        memset(dst, 0xff, len);
        s->busy = 0;
        s->out_remaining = s->out_remaining > n ? s->out_remaining - n : 0;
        if (s->out_remaining == 0) { s->read_failed = 0; s->focus_flag = 1; return 0; }
        if (s->last_error == 0x67) { free_pack_buf(s); return 0; }
        return 1;
    }
    if (s->mirror) dsd4_logf(s, 1, "mirror flag set: image is horizontally flipped");
    s->busy = 0;
    return 1;
}

/* StopScan */
int avl_stop_scan(dsd4 *s)
{
    s->busy = 1;
    free_pack_buf(s);
    int ok = av_release(s);
    s->need_release = !ok;
    s->scanning = 0;
    s->busy = 0;
    return ok;
}

/* ------------------------------------------------------------ firmware */

/* InternalDownLoadFirmware: only acts when READ 0x90 byte 0 is non zero and
 * the firmware file is not older than the running firmware. */
int avl_download_firmware(dsd4 *s, const char *path, int *downloaded)
{
    *downloaded = 0;
    uint8_t st[0x22] = { 0 };
    if (!av_read(s, DT_FIRMWARE, sizeof st, st)) return 0;
    dsd4_logf(s, 2, "firmware status byte %02x", st[0]);
    if (st[0] == 0) return 1;
    uint8_t q[96] = { 0 };
    av_inquiry(s, q);
    FILE *f = path ? fopen(path, "rb") : NULL;
    if (!f) { s->last_error = AVE_FW_FILE; return 0; }
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (size <= 0 || size > 0x80000) { fclose(f); s->last_error = AVE_FW_FILE; return 0; }
    uint8_t *img = malloc((size_t)size + 4);
    if (!img || fread(img, 1, (size_t)size, f) != (size_t)size) {
        free(img);
        fclose(f);
        s->last_error = AVE_FW_READ;
        return 0;
    }
    fclose(f);
    long i = 0;
    while (i < size && !(i + 6 <= size && memcmp(img + i, "FS-V31", 6) == 0)) i++;
    long v = i + 8;
    int newer_or_same = 1;
    if (v + 3 < size) {
        if (img[v] < q[32]) newer_or_same = 0;
        else if (img[v] == q[32]) {
            if (img[v + 2] < q[34]) newer_or_same = 0;
            else if (img[v + 2] == q[34] && img[v + 3] < q[35]) newer_or_same = 0;
        }
    }
    if (!newer_or_same) {
        free(img);
        av_test_unit_ready(s);
        av_delay_ticks(0xb4);
        return 1;
    }
    uint32_t sum = 0;
    for (long k = 0; k < size; k++) sum += img[k];
    put_le32(img + size, sum); /* SwapDWORD on a big-endian host */
    dsd4_logf(s, 1, "downloading firmware %s (%ld bytes)", path, size);
    av_send(s, DT_FIRMWARE, 0, (uint32_t)size + 4, img);
    int ok = av_test_unit_ready(s);
    av_delay_ticks(0xb4);
    free(img);
    *downloaded = 1;
    return ok;
}
