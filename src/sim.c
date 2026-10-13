/*
 * Protocol-level simulator of a DiMAGE Scan Dual IV.
 *
 * It answers the same command set the real scanner implements (as recovered
 * from the original driver), validates what the host sends (CDB layout, SET
 * WINDOW block, shading and acceleration table sizes) and produces synthetic
 * calibration and image data.  The image is a test chart whose sharpness
 * depends on the focus position, so autofocus can be exercised as well.
 *
 * INQUIRY data comes from the firmware image (DS_Dual4.BIN embeds the 96 byte
 * INQUIRY answer of the FS-V31) when available.
 */
#include "transport.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SIM_FOCUS_BEST 37

/* INQUIRY of firmware 1.04, as embedded in DS_Dual4.BIN (offset 48246) */
static const uint8_t k_inquiry[96] = {
    0x06, 0x80, 0x02, 0x42, 0x5b, 0x00, 0x00, 0x00, 'K', 'O', 'N', 'I', 'C', 'A', ' ', 'M',
    'I', 'N', 'O', 'L', 'T', 'A', ' ', ' ', 'F', 'S', '-', 'V', '3', '1', ' ', ' ',
    '1', '.', '0', '4', 0x20, 0x03, 0x06, 0x67, 0x01, 0x2c, 0x01, 0x2c, 0x01, 0x2c, 0x01, 0x2c,
    0x00, 0x00, 0xd9, 0x96, 0xfa, 0x08, 0x04, 0x04, 0x02, 0x08, 0x08, 0x08, 0xc0, 0xd2, 0xa8, 0x05,
    0x01, 0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x30, 0x12, 0x48, 0x0c, 0x30, 0x12, 0x48, 0x0c, 0x80, 0x06, 0x00, 0x20, 0x00, 0x00,
};

#define CAL_PPL   3200
#define CAL_LINES 12   /* 4 per colour */

typedef struct {
    uint8_t inquiry[96];
    uint8_t sense_key, asc, ascq;
    int calibrated, initialized;
    int light_reads;
    int reserved, scanning;
    int focus;
    int holder, frame, frames;
    /* window */
    int have_window;
    uint16_t xres, yres;
    uint32_t ulx, uly, width, length;
    uint8_t comp, bpc;
    uint16_t line_width, line_count;
    uint8_t bs1, bs2, bs3, multi_scan, multi_sample;
    uint16_t exp[3];
    /* streaming */
    uint64_t sent, total;
    uint32_t line_no;
    int acc_reads;
    int shading_sends;
    int errors;
    FILE *trace;
    /* test conditions (environment, see sim_open) */
    double noise;       /* DSD4_SIM_NOISE: read noise sigma, fraction of full scale */
    int nega;           /* DSD4_SIM_FILM=nega: neutral scene on orange-mask negative film */
    double shift;       /* DSD4_SIM_SHIFT: carriage offset (lines) of every second image scan */
    int outlier;        /* DSD4_SIM_OUTLIER=n: every n-th line read is disturbed */
    int star;           /* DSD4_SIM_CHART=star: fine detail beyond Nyquist through a lens PSF */
    double psf;         /* DSD4_SIM_PSF: lens blur sigma in pixels at best focus (star chart) */
    double breathing;   /* DSD4_SIM_BREATHING: magnification change, ppm per focus step */
    double jitter;      /* DSD4_SIM_JITTER: random sub-line offset per image scan, sigma in lines */
    int yres_ok;        /* DSD4_SIM_YRES=1: windows above 3200 dpi oversample; 2: accepted, 3200 pitch */
    double scan_off;    /* this scan's jitter offset */
    int image_scans, reads;
    uint64_t rng;
} sim_t;

static void sense(sim_t *m, uint8_t key, uint8_t asc, uint8_t ascq)
{
    m->sense_key = key; m->asc = asc; m->ascq = ascq;
}

static uint32_t rd24(const uint8_t *p) { return (uint32_t)p[0] << 16 | (uint32_t)p[1] << 8 | p[2]; }
static uint16_t rbe16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static uint32_t rbe32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void wle16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wbe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

static void bug(sim_t *m, const char *what)
{
    m->errors++;
    fprintf(stderr, "sim: protocol violation: %s\n", what);
    if (m->trace) fprintf(m->trace, "VIOLATION %s\n", what);
}

/* synthetic slide: concentric rings + colour gradient, blurred by defocus */
static double chart(double x, double y, int c, double blur)
{
    double cx = x - 1560, cy = y - 2340;
    double r = sqrt(cx * cx + cy * cy);
    double ring = 0.5 + 0.5 * cos(r / 6.0) * exp(-blur * blur * 0.02);
    double grad = (c == 0 ? x / 3120.0 : c == 1 ? y / 4680.0 : 1.0 - x / 3120.0);
    return 0.15 + 0.6 * (0.6 * ring + 0.4 * grad);
}

/* colour negative of a neutral scene: per-channel film base (orange mask)
 * and contrast, transmission = base * 10^(-gamma * log density of the scene) */
static double nega_chart(double x, double y, int c, double blur)
{
    static const double base[3] = { 0.60, 0.25, 0.15 }, gam[3] = { 0.55, 0.62, 0.70 };
    double s = chart(x, y, 1, blur);
    return base[c] * pow(10, -gam[c] * log10(s / 0.15));
}

/* Film-like test scene with detail beyond 3200 dpi: a sum of gratings at
 * random orientations, 0.05 to 1.2 cycles per pixel (Nyquist is 0.5).  The
 * lens (Gaussian, sigma = psf + defocus) and the pixel aperture (box) are
 * applied exactly as each grating's transfer factor, so the simulated scanner
 * aliases like a real one. */
#define NGRAT 48
static struct { double fx, fy, ph, amp; } g_grat[NGRAT];

static void init_gratings(void)
{
    uint64_t r = 0x2545f4914f6cdd1dull;
    for (int i = 0; i < NGRAT; i++) {
        double u[3];
        for (int k = 0; k < 3; k++) {
            r ^= r << 13; r ^= r >> 7; r ^= r << 17;
            u[k] = (r >> 11) / 9007199254740992.0;
        }
        double f = 0.05 * pow(24.0, (i + u[0]) / NGRAT); /* log-spaced 0.05 .. 1.2 */
        double th = M_PI * u[1];
        g_grat[i].fx = f * cos(th);
        g_grat[i].fy = f * sin(th);
        g_grat[i].ph = 2 * M_PI * u[2];
        g_grat[i].amp = 0.25 / sqrt(NGRAT);
    }
}

static double sinc(double x) { return fabs(x) < 1e-9 ? 1 : sin(M_PI * x) / (M_PI * x); }

static double star_pixel(sim_t *m, double x, double y, int c, double blur_steps)
{
    const double sig = m->psf + 0.04 * blur_steps;
    double v = 0.4;
    for (int i = 0; i < NGRAT; i++) {
        const double fx = g_grat[i].fx, fy = g_grat[i].fy, f2 = fx * fx + fy * fy;
        const double h = exp(-2 * M_PI * M_PI * sig * sig * f2) * sinc(fx) * sinc(fy);
        v += g_grat[i].amp * h * cos(2 * M_PI * (fx * x + fy * y) + g_grat[i].ph);
    }
    return v * (c == 0 ? 1.0 : c == 1 ? 0.95 : 0.9);
}

static double gauss(sim_t *m)
{
    double u[2];
    for (int k = 0; k < 2; k++) {
        m->rng ^= m->rng << 13; m->rng ^= m->rng >> 7; m->rng ^= m->rng << 17;
        u[k] = ((m->rng >> 11) + 0.5) / 9007199254740992.0;
    }
    return sqrt(-2 * log(u[0])) * cos(6.283185307179586 * u[1]);
}

static void make_line(sim_t *m, uint8_t *out, uint32_t line)
{
    const int deep = m->bpc > 8;
    const uint32_t px = m->line_width / (deep ? 6 : 3);
    const double scale = 3200.0 / m->xres;
    const double off = ((m->image_scans % 2 == 0) ? m->shift : 0) + m->scan_off; /* every second pass */
    const double blur = fabs((double)(m->focus - SIM_FOCUS_BEST));
    /* lens breathing: magnification about the optical axis (frame centre
     * horizontally; the sensor line sits 40 lines off it vertically) */
    const double mag = 1 + m->breathing * 1e-6 * (m->focus - SIM_FOCUS_BEST);
    /* DSD4_SIM_YRES=2 behaves like the real FS-V31: a finer vertical window is
     * accepted but the carriage still steps one 3200 dpi line per line */
    const double pitch = m->yres > 3200 && m->yres_ok == 2 ? 1.0 : 3200.0 / m->yres;
    const double yy = m->uly + ((double)line + off) * pitch + 40 * (1 - 1 / mag);
    const int disturbed = m->outlier > 0 && ++m->reads % m->outlier == 0;
    for (uint32_t i = 0; i < px; i++) {
        double xx = 1560 + (m->ulx + i * scale - 1560) / mag;
        for (int c = 0; c < 3; c++) {
            double v = (m->star ? star_pixel(m, xx, yy, c, blur)
                        : m->nega ? nega_chart(xx, yy, c, blur) : chart(xx, yy, c, blur)) *
                       (m->exp[c] ? m->exp[c] / 142.857 : 1.0);
            if (m->noise > 0) v += m->noise * gauss(m);
            if (disturbed) v += 0.25;
            if (v < 0) v = 0;
            if (v > 1.0) v = 1.0;
            if (deep) wle16(out + (i * 3 + c) * 2, (uint16_t)(v * 65535));
            else out[i * 3 + c] = (uint8_t)(v * 255);
        }
    }
}

static int do_read(sim_t *m, const uint8_t *c, uint8_t *data, size_t len)
{
    uint8_t dtc = c[2];
    uint32_t n = rd24(c + 6);
    if (n != len) { bug(m, "READ length mismatch"); return 2; }
    if (c[4] != 0x0a || c[5] != 0x0d) bug(m, "READ qualifier is not 0x0a0d");
    memset(data, 0, len);
    switch (dtc) {
    case 0x00: { /* image */
        if (!m->scanning) { sense(m, 5, 0x2c, 0); return 2; }
        if (!m->line_width || len % m->line_width) { bug(m, "image READ not a whole number of lines"); return 2; }
        int endless = (m->bs3 & 4) != 0; /* multi-scan: the line is re-scanned */
        for (size_t o = 0; o < len; o += m->line_width) {
            if (!endless && m->sent >= m->total) { bug(m, "image READ past end of window"); return 2; }
            make_line(m, data + o, endless ? 0 : m->line_no);
            m->sent += m->line_width;
            if (!endless) {
                unsigned per = m->multi_sample ? m->multi_sample : 1;
                if ((m->sent / m->line_width) % per == 0) m->line_no++;
            }
        }
        return 0;
    }
    case 0x60: /* calibration format */
        if (len != 32) bug(m, "calibration format length");
        wbe16(data, CAL_PPL);
        data[2] = 2;
        data[3] = CAL_LINES;
        data[4] = m->initialized ? (m->calibrated ? 0 : 2) : 1;
        data[5] = 0x0c; /* dark data available, colour */
        wbe16(data + 9, 0x00f0); wbe16(data + 11, 0x00f0); wbe16(data + 13, 0x00f0);
        wbe16(data + 15, 0x0002); wbe16(data + 17, 0x0002); wbe16(data + 19, 0x0002);
        wle16(data + 21, 1000); wle16(data + 23, 1100); wle16(data + 25, 1200);
        wle16(data + 27, 20000); wle16(data + 29, 200);
        return 0;
    case 0x62: case 0x61: case 0x66: { /* white / dark shading data, LE 16 bit */
        size_t want = (size_t)CAL_LINES * CAL_PPL * 2;
        if (len != want) { bug(m, "calibration data length"); return 2; }
        for (size_t i = 0; i < len / 2; i++) {
            uint32_t px = (uint32_t)(i % (CAL_PPL * 3)) / 3;
            double v = dtc == 0x66 ? 600 + (px % 7) * 3 : 40000 + 8000 * sin(px / 500.0);
            wle16(data + i * 2, (uint16_t)v);
        }
        if (dtc != 0x66) m->calibrated = 1;
        return 0;
    }
    case 0x6c: /* acceleration table request */
        if (len != 0x18) bug(m, "accel request length");
        m->acc_reads++;
        wbe16(data, 64);        /* total entries */
        wbe16(data + 2, 16);    /* constant tail */
        data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 3;     /* a */
        data[8] = 0; data[9] = 0; data[10] = 0x10; data[11] = 0; /* b */
        wbe16(data + 12, 200);  /* start */
        wbe16(data + 14, 50);   /* end */
        data[16] = 0;           /* host must compute */
        data[17] = m->acc_reads <= 1 ? 1 : 0;
        return 0;
    case 0x70:
        wbe16(data, 512); data[2] = 3; data[3] = 1; data[5] = 0;
        return 0;
    case 0x71:
        memset(data, 0xa0, len);
        return 0;
    case 0x87: /* holder */
        data[0] = (uint8_t)m->holder;
        data[1] = (uint8_t)m->frame;
        data[2] = (uint8_t)m->frames;
        return 0;
    case 0x90: /* firmware status: byte 0 != 0 asks the host for an image */
        if (getenv("DSD4_SIM_FW_PENDING")) data[0] = 1;
        return 0;
    case 0xa0: /* light status: warming up for the first reads */
        data[0] = m->light_reads++ < 2 ? 3 : 0;
        return 0;
    case 0x69:
        return 0;
    }
    sense(m, 5, 0x24, 0);
    return 2;
}

static int do_send(sim_t *m, const uint8_t *c, const uint8_t *data, size_t len)
{
    uint8_t dtc = c[2];
    uint16_t q = rbe16(c + 4);
    if (rd24(c + 6) != len) { bug(m, "SEND length mismatch"); return 2; }
    switch (dtc) {
    case 0x92: m->initialized = 1; return 0;
    case 0x88: {
        uint16_t w = rbe16(data);
        m->focus = (w & 0x8000) ? (w & 0x7fff) : -(int)w;
        return 0;
    }
    case 0x87:
        if (data[1] == 0xff) { m->holder = 0xff; m->frames = 0; m->frame = 0; return 0; }
        if (data[1] > m->frames) { sense(m, 3, 0x81, 4); return 2; }
        m->frame = data[1];
        return 0;
    case 0x82:
        if (!m->have_window) bug(m, "shading sent before SET WINDOW");
        if (q > 5) bug(m, "bad shading qualifier");
        if (len % 2) bug(m, "odd shading length");
        m->shading_sends++;
        return 0;
    case 0x6c:
        if (len != 64) bug(m, "accel table length");
        return 0;
    case 0x81:
        if (len != 512) bug(m, "gamma length");
        return 0;
    case 0x90: {
        uint32_t sum = 0;
        if (len < 5) { bug(m, "firmware image too short"); return 2; }
        for (size_t i = 0; i < len - 4; i++) sum += data[i];
        uint32_t got = (uint32_t)data[len - 4] | (uint32_t)data[len - 3] << 8 | (uint32_t)data[len - 2] << 16 |
                       (uint32_t)data[len - 1] << 24;
        if (got != sum) bug(m, "firmware checksum mismatch");
        if (m->trace) fprintf(m->trace, "FIRMWARE %zu bytes, checksum %08x ok\n", len - 4, sum);
        return 0;
    }
    }
    sense(m, 5, 0x24, 0);
    return 2;
}

static int do_set_window(sim_t *m, const uint8_t *w, size_t len)
{
    if (len < 0x3b) { bug(m, "window too short"); return 2; }
    uint16_t dl = rbe16(w + 6);
    if (dl != 0x33 && dl != 0x3e) bug(m, "window descriptor length");
    const uint8_t *d = w + 8;
    m->xres = rbe16(d + 2);
    m->yres = rbe16(d + 4);
    m->ulx = rbe32(d + 6);
    m->uly = rbe32(d + 10);
    m->width = rbe32(d + 14);
    m->length = rbe32(d + 18);
    m->comp = d[25];
    m->bpc = d[26];
    m->bs1 = d[42];
    m->line_width = rbe16(d + 45);
    m->line_count = rbe16(d + 47);
    m->bs2 = d[49];
    if (dl == 0x3e) {
        m->exp[0] = rbe16(d + 51);
        m->exp[1] = rbe16(d + 53);
        m->exp[2] = rbe16(d + 55);
        m->bs3 = d[57];
        m->multi_scan = d[58];
        m->multi_sample = (m->bs3 & 2) ? d[61] : 0;
    }
    if (!m->xres || !m->yres) bug(m, "zero resolution");
    if ((m->yres > 3200 || m->xres > 3200) && !(m->yres_ok && m->xres <= 3200)) {
        sense(m, 5, 0x26, 0); /* invalid field in parameter list */
        return 2;
    }
    if (m->comp != 5 && m->comp != 2) bug(m, "unexpected image composition");
    if (m->line_width == 0 || m->line_count == 0) bug(m, "empty window");
    m->have_window = 1;
    if (m->trace)
        fprintf(m->trace, "WINDOW %ux%u dpi ul %u,%u ext %ux%u line %u B x %u bits %u bs1 %02x bs2 %02x bs3 %02x exp %u/%u/%u ms %u\n",
                m->xres, m->yres, m->ulx, m->uly, m->width, m->length, m->line_width, m->line_count,
                m->bpc, m->bs1, m->bs2, m->bs3, m->exp[0], m->exp[1], m->exp[2], m->multi_sample);
    return 0;
}

static int sim_exec(dsd4_transport *t, const uint8_t c[10], enum dsd4_dir dir, void *data, size_t len,
                    uint8_t *status)
{
    sim_t *m = t->priv;
    int st = 0;
    if (m->trace) {
        fprintf(m->trace, "CDB");
        for (int i = 0; i < 10; i++) fprintf(m->trace, " %02x", c[i]);
        fprintf(m->trace, " %s %zu\n", dir == DSD4_DIR_IN ? "in" : dir == DSD4_DIR_OUT ? "out" : "-", len);
    }
    switch (c[0]) {
    case 0x00: st = 0; break;
    case 0x03:
        memset(data, 0, len);
        if (len >= 14) {
            ((uint8_t *)data)[0] = 0x70;
            ((uint8_t *)data)[2] = m->sense_key;
            ((uint8_t *)data)[12] = m->asc;
            ((uint8_t *)data)[13] = m->ascq;
        }
        break;
    case 0x12:
        if (dir != DSD4_DIR_IN || len != 0x60 || c[4] != 0x60) bug(m, "INQUIRY layout");
        memcpy(data, m->inquiry, len < 96 ? len : 96);
        break;
    case 0x16: m->reserved = 1; break;
    case 0x17: m->reserved = 0; m->scanning = 0; break;
    case 0x1b:
        if (!m->have_window) { bug(m, "SCAN without window"); st = 2; break; }
        if (!m->reserved) bug(m, "SCAN without RESERVE UNIT");
        m->scanning = 1;
        m->sent = 0;
        m->line_no = 0;
        if (m->line_count >= 64) { /* image scans, not focus/calibration lines */
            m->image_scans++;
            m->scan_off = m->jitter > 0 ? m->jitter * gauss(m) : 0;
            if (m->trace && m->jitter > 0) fprintf(m->trace, "SCANOFF %.5f\n", m->scan_off);
        }
        m->acc_reads = 0;
        m->total = (uint64_t)m->line_width * m->line_count * (m->multi_sample ? m->multi_sample : 1);
        break;
    case 0x24:
        if (dir != DSD4_DIR_OUT || rd24(c + 6) != len) bug(m, "SET WINDOW layout");
        st = do_set_window(m, data, len);
        break;
    case 0x28:
        if (dir != DSD4_DIR_IN) bug(m, "READ direction");
        st = do_read(m, c, data, len);
        break;
    case 0x2a:
        if (dir != DSD4_DIR_OUT) bug(m, "SEND direction");
        st = do_send(m, c, data, len);
        break;
    case 0x31:
        if (c[1] == 2) m->frame = m->frame; /* carriage home */
        break;
    default:
        sense(m, 5, 0x20, 0);
        st = 2;
    }
    if (st == 0) sense(m, 0, 0, 0);
    if (m->trace) fprintf(m->trace, "  -> status %d\n", st);
    *status = (uint8_t)st;
    return 0;
}

static void sim_close(dsd4_transport *t)
{
    sim_t *m = t->priv;
    if (m) {
        if (m->trace) fclose(m->trace);
        if (m->errors) fprintf(stderr, "sim: %d protocol violation(s)\n", m->errors);
        free(m);
    }
    free(t);
}

dsd4_transport *dsd4_sim_open(const char *firmware_path, char *err, size_t errlen)
{
    sim_t *m = calloc(1, sizeof *m);
    dsd4_transport *t = calloc(1, sizeof *t);
    if (!m || !t) { free(m); free(t); if (err) snprintf(err, errlen, "out of memory"); return NULL; }
    memcpy(m->inquiry, k_inquiry, 96);
    const char *src = "built-in";
    if (firmware_path) {
        FILE *f = fopen(firmware_path, "rb");
        if (f) {
            static uint8_t buf[0x80000];
            size_t n = fread(buf, 1, sizeof buf, f);
            fclose(f);
            for (size_t i = 24; i + 6 <= n; i++)
                if (memcmp(buf + i, "FS-V31", 6) == 0 && i >= 24 && i - 24 + 96 <= n) {
                    memcpy(m->inquiry, buf + i - 24, 96);
                    src = firmware_path;
                    break;
                }
        }
    }
    m->holder = 2;  /* 35 mm strip holder */
    m->frames = 6;
    if (getenv("DSD4_SIM_APS")) { m->holder = 1; m->frames = 25; } /* APS cartridge adapter */
    m->frame = 1;
    const char *ev;
    if ((ev = getenv("DSD4_SIM_NOISE"))) m->noise = atof(ev);
    if ((ev = getenv("DSD4_SIM_FILM"))) m->nega = !strcmp(ev, "nega");
    if ((ev = getenv("DSD4_SIM_SHIFT"))) m->shift = atof(ev);
    if ((ev = getenv("DSD4_SIM_OUTLIER"))) m->outlier = atoi(ev);
    if ((ev = getenv("DSD4_SIM_CHART"))) m->star = !strcmp(ev, "star");
    if (m->star) init_gratings();
    m->psf = (ev = getenv("DSD4_SIM_PSF")) ? atof(ev) : 0.35;
    if ((ev = getenv("DSD4_SIM_BREATHING"))) m->breathing = atof(ev);
    if ((ev = getenv("DSD4_SIM_JITTER"))) m->jitter = atof(ev);
    if ((ev = getenv("DSD4_SIM_YRES"))) m->yres_ok = atoi(ev);
    m->rng = 0x9e3779b97f4a7c15ull;
    const char *tr = getenv("DSD4_SIM_TRACE");
    if (tr) m->trace = fopen(tr, "w");
    t->priv = m;
    t->exec = sim_exec;
    t->close = sim_close;
    snprintf(t->description, sizeof t->description, "simulated FS-V31 (inquiry from %s)", src);
    return t;
}
