/*
 * Linear DNG writer: the scanner's raw linear RGB (or gray) for raw
 * converters.  One IFD, uncompressed strips, LinearRaw photometric.
 *
 * Colour: Konica Minolta characterised the Dual IV's linear output in
 * MLTF3200p.icc ("Posi Linear"), a matrix/TRC profile: XYZ(D50) =
 * M * TRC(rgb), where the per-channel curves are a near-linear correction of
 * the sensor (up to ~13 % in the shadows).  The writer reads both from the
 * profile, applies the curves to the samples and stores M (inverted, as DNG
 * wants XYZ -> camera, and adjusted for any per-channel gain the processed
 * output would have applied but a raw file leaves out) as ColorMatrix1, so
 * raw converters reproduce what ColorSync does with the TIFF.  Without a
 * profile the built-in colorants below are used with linear curves.
 *
 * An inverted negative is no longer the light the scanner measured but a
 * picture made from the film's densities, so the calibration does not
 * apply: it is written as linear sRGB (D65), neutral white balance, for apps
 * without a negative conversion of their own (Photos, Preview).
 */
#include "dsd4_internal.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const double K_CAM_TO_XYZ[9] = { /* rXYZ gXYZ bXYZ as columns, /65536 */
    32890 / 65536.0, 21684 / 65536.0, 8615 / 65536.0,
    14109 / 65536.0, 47333 / 65536.0, 4092 / 65536.0,
    89 / 65536.0,    -9736 / 65536.0, 63706 / 65536.0,
};

static const double K_SRGB_TO_XYZ[9] = { /* linear sRGB -> XYZ D65 */
    0.4124564, 0.3575761, 0.1804375,
    0.2126729, 0.7151522, 0.0721750,
    0.0193339, 0.1191920, 0.9503041,
};

typedef struct {
    double m[9];          /* camera -> XYZ (D50; D65 for inverted negatives) */
    int n[3];             /* curve points per channel (0: linear) */
    double gamma[3];      /* n == 1 */
    uint16_t *curve[3];
} profile_t;

static uint32_t rd32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }

/* matrix/TRC data of an ICC profile; 0 on success */
static int parse_icc(const uint8_t *d, size_t len, profile_t *pr)
{
    memset(pr, 0, sizeof *pr);
    if (len < 132 || memcmp(d + 36, "acsp", 4)) return -1;
    uint32_t n = rd32(d + 128);
    int have = 0;
    for (uint32_t i = 0; i < n && 132 + 12 * (i + 1) <= len; i++) {
        const uint8_t *e = d + 132 + 12 * i;
        uint32_t off = rd32(e + 4), sz = rd32(e + 8);
        if ((uint64_t)off + sz > len || sz < 12) continue;
        const uint8_t *v = d + off;
        static const char *xyz[3] = { "rXYZ", "gXYZ", "bXYZ" }, *trc[3] = { "rTRC", "gTRC", "bTRC" };
        for (int c = 0; c < 3; c++) {
            if (!memcmp(e, xyz[c], 4) && !memcmp(v, "XYZ ", 4) && sz >= 20) {
                for (int r = 0; r < 3; r++) pr->m[r * 3 + c] = (int32_t)rd32(v + 8 + 4 * r) / 65536.0;
                have |= 1 << c;
            }
            if (!memcmp(e, trc[c], 4) && !memcmp(v, "curv", 4)) {
                uint32_t cnt = rd32(v + 8);
                if (cnt == 1 && sz >= 14) { pr->n[c] = 1; pr->gamma[c] = (v[12] << 8 | v[13]) / 256.0; }
                else if (cnt > 1 && 12 + 2 * (uint64_t)cnt <= sz) {
                    pr->n[c] = (int)cnt;
                    pr->curve[c] = (uint16_t *)malloc(cnt * 2);
                    if (!pr->curve[c]) return -1;
                    for (uint32_t k = 0; k < cnt; k++) pr->curve[c][k] = (uint16_t)(v[12 + 2 * k] << 8 | v[13 + 2 * k]);
                }
                have |= 8 << c;
            }
        }
    }
    return have == 0x3f ? 0 : -1;
}

static void free_profile(profile_t *pr)
{
    for (int c = 0; c < 3; c++) free(pr->curve[c]);
}

/* TRC of channel c at x (linear extension above 1) */
static double trc(const profile_t *pr, int c, double x)
{
    if (x <= 0) return 0;
    if (pr->n[c] == 0) return x;
    if (pr->n[c] == 1) return x >= 1 ? x : pow(x, pr->gamma[c]);
    const int n = pr->n[c];
    if (x >= 1) return x * pr->curve[c][n - 1] / 65535.0;
    double f = x * (n - 1);
    int i = (int)f;
    double t = f - i;
    return ((1 - t) * pr->curve[c][i] + t * pr->curve[c][i + 1]) / 65535.0;
}

static int inv3(const double m[9], double o[9])
{
    double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
    double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    if (fabs(det) < 1e-12) return -1;
    o[0] = (e * i - f * h) / det; o[1] = (c * h - b * i) / det; o[2] = (b * f - c * e) / det;
    o[3] = (f * g - d * i) / det; o[4] = (a * i - c * g) / det; o[5] = (c * d - a * f) / det;
    o[6] = (d * h - e * g) / det; o[7] = (b * g - a * h) / det; o[8] = (a * e - b * d) / det;
    return 0;
}

enum { T_BYTE = 1, T_ASCII = 2, T_SHORT = 3, T_LONG = 4, T_RATIONAL = 5, T_SRATIONAL = 10, T_FLOAT = 11 };

typedef struct {
    uint16_t tag, type;
    uint32_t count;
    uint8_t data[96]; /* little-endian payload */
    uint32_t len;
} entry_t;

typedef struct {
    entry_t e[40];
    int n;
} ifd_t;

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

static entry_t *add(ifd_t *d, uint16_t tag, uint16_t type, uint32_t count)
{
    entry_t *e = &d->e[d->n++];
    memset(e, 0, sizeof *e);
    e->tag = tag;
    e->type = type;
    e->count = count;
    static const uint8_t sz[] = { 0, 1, 1, 2, 4, 8, 0, 0, 0, 0, 8, 4 };
    e->len = count * sz[type];
    return e;
}

static void add_short(ifd_t *d, uint16_t tag, int count, const uint16_t *v)
{
    entry_t *e = add(d, tag, T_SHORT, (uint32_t)count);
    for (int i = 0; i < count; i++) put16(e->data + 2 * i, v[i]);
}
static void add_long(ifd_t *d, uint16_t tag, uint32_t v) { put32(add(d, tag, T_LONG, 1)->data, v); }
static void add_ascii(ifd_t *d, uint16_t tag, const char *s)
{
    entry_t *e = add(d, tag, T_ASCII, (uint32_t)strlen(s) + 1);
    memcpy(e->data, s, e->len);
}
static void add_rational(ifd_t *d, uint16_t tag, uint16_t type, int count, const double *v)
{
    entry_t *e = add(d, tag, type, (uint32_t)count);
    for (int i = 0; i < count; i++) {
        put32(e->data + 8 * i, (uint32_t)(int32_t)lround(v[i] * 10000));
        put32(e->data + 8 * i + 4, 10000);
    }
}

static void add_float(ifd_t *d, uint16_t tag, int count, const float *v)
{
    entry_t *e = add(d, tag, T_FLOAT, (uint32_t)count);
    for (int i = 0; i < count; i++) {
        uint32_t u;
        memcpy(&u, &v[i], 4);
        put32(e->data + 4 * i, u);
    }
}

static int cmp_entry(const void *a, const void *b)
{
    return (int)((const entry_t *)a)->tag - (int)((const entry_t *)b)->tag;
}

/* IEEE 754 binary16, round to nearest even */
static uint16_t to_half(float f)
{
    uint32_t x;
    memcpy(&x, &f, 4);
    uint32_t sign = (x >> 16) & 0x8000, e = (x >> 23) & 0xff, m = x & 0x7fffff;
    if (e == 0xff) return (uint16_t)(sign | 0x7c00 | (m ? 0x200 : 0));
    int ee = (int)e - 127 + 15;
    if (ee >= 31) return (uint16_t)(sign | 0x7c00);
    if (ee <= 0) {
        if (ee < -10) return (uint16_t)sign;
        m |= 0x800000;
        uint32_t shift = (uint32_t)(14 - ee);
        uint32_t h = m >> shift, rem = m & ((1u << shift) - 1), half = 1u << (shift - 1);
        if (rem > half || (rem == half && (h & 1))) h++;
        return (uint16_t)(sign | h);
    }
    uint32_t h = (uint32_t)ee << 10 | m >> 13, rem = m & 0x1fff;
    if (rem > 0x1000 || (rem == 0x1000 && (h & 1))) h++;
    return (uint16_t)(sign | h);
}

int dsd4_write_dng(const dsd4_image *img, const char *path, int float_bits, const void *icc, size_t icc_len)
{
    if (img->bits != 16 && img->bits != 32) return DSD4_ERR_PARAM;
    if (img->channels != 1 && img->channels != 3) return DSD4_ERR_PARAM;
    if (float_bits != 0 && float_bits != 16 && float_bits != 32) return DSD4_ERR_PARAM;
    const int ch = img->channels, fl = img->bits == 32 && float_bits, pos = img->inverted, mino = img->minolta;
    const int bps = fl ? float_bits : 16;
    /* Apple's decoder ignores SampleFormat on one-channel DNGs and reads float
     * samples as integers (Preview showed B&W float DNGs flattened to ~0.2):
     * gray float images are written as three equal channels, neutral */
    const int expand = ch == 1 && fl, och = expand ? 3 : ch;
    const size_t row = (size_t)img->width * och * (bps / 8);
    uint32_t rps = (uint32_t)(row ? (1u << 20) / row : 1);
    if (rps < 1) rps = 1;
    if (rps > (uint32_t)img->height) rps = (uint32_t)img->height;
    const uint32_t strips = ((uint32_t)img->height + rps - 1) / rps;

    /* gain the raw data still needs per channel (1 for already processed data) */
    double pend[3];
    for (int c = 0; c < 3; c++) {
        double applied = img->digital_gain[c] > 0 ? img->digital_gain[c] : 1;
        double would = img->raw_gain[c] > 0 ? img->raw_gain[c] : applied;
        pend[c] = would / applied;
        if (!(pend[c] > 0) || pos || mino) pend[c] = 1;
    }
    double pmin = ch == 3 ? fmin(pend[0], fmin(pend[1], pend[2])) : pend[0];
    /* Minolta positives: colour in the Nega Input profile the TIFF embeds
     * (icc); gray as the gamma 2.2 the TIFF is tagged with.  Their curves turn
     * the encoded samples back into linear light. */
    profile_t pr;
    int srgb = pos;
    if (pos || (mino && ch == 1) || !icc || parse_icc((const uint8_t *)icc, icc_len, &pr)) {
        if (!pos && !(mino && ch == 1) && icc) free_profile(&pr);
        memset(&pr, 0, sizeof pr);
        srgb = pos || mino;
        memcpy(pr.m, srgb ? K_SRGB_TO_XYZ : K_CAM_TO_XYZ, sizeof pr.m);
        if (mino)
            for (int c = 0; c < 3; c++) { pr.n[c] = 1; pr.gamma[c] = 2.2; }
    }
    if (expand) {
        /* gray in RGB: neutral (sRGB primaries, D65 white, equal gains) */
        memcpy(pr.m, K_SRGB_TO_XYZ, sizeof pr.m);
        srgb = 1;
        pend[1] = pend[2] = pend[0];
    }

    ifd_t d = { .n = 0 };
    add_long(&d, 254, 0); /* NewSubFileType: main image */
    add_long(&d, 256, (uint32_t)img->width);
    add_long(&d, 257, (uint32_t)img->height);
    uint16_t v3[3] = { (uint16_t)bps, (uint16_t)bps, (uint16_t)bps };
    add_short(&d, 258, och, v3);
    uint16_t one = 1, lin = 34892, two = 2, spp = (uint16_t)och;
    add_short(&d, 259, 1, &one);
    add_short(&d, 262, 1, &lin);
    add_ascii(&d, 271, "KONICA MINOLTA");
    add_ascii(&d, 272, "DiMAGE Scan Dual IV");
    add(&d, 273, T_LONG, strips); /* StripOffsets, written below */
    add_short(&d, 274, 1, &one);
    add_short(&d, 277, 1, &spp);
    add_long(&d, 278, rps);
    add(&d, 279, T_LONG, strips); /* StripByteCounts */
    double dpi = img->dpi > 0 ? img->dpi : 3200;
    add_rational(&d, 282, T_RATIONAL, 1, &dpi);
    add_rational(&d, 283, T_RATIONAL, 1, &dpi);
    add_short(&d, 284, 1, &one);
    add_short(&d, 296, 1, &two);
    add_ascii(&d, 305, "dsd4 (DiMAGE Scan Dual IV driver)");
    uint16_t sf[3] = { fl ? 3 : 1, fl ? 3 : 1, fl ? 3 : 1 };
    add_short(&d, 339, och, sf);
    entry_t *e = add(&d, 50706, T_BYTE, 4);
    e->data[0] = 1; e->data[1] = 4;
    e = add(&d, 50707, T_BYTE, 4);
    e->data[0] = 1; e->data[1] = fl ? 4 : 1;
    add_ascii(&d, 50708, "Konica Minolta DiMAGE Scan Dual IV");
    add_long(&d, 50717, fl ? 1 : 65535); /* WhiteLevel */
    if (och == 3) {
        /* camera -> XYZ = K * diag(pend); ColorMatrix1 = its inverse */
        double m[9], inv[9], neutral[3];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) m[r * 3 + c] = pr.m[r * 3 + c] * pend[c];
        if (inv3(m, inv)) { free_profile(&pr); return DSD4_ERR_PARAM; }
        add_rational(&d, 50721, T_SRATIONAL, 9, inv);
        double nmax = 0;
        for (int c = 0; c < 3; c++) { neutral[c] = 1.0 / pend[c]; if (neutral[c] > nmax) nmax = neutral[c]; }
        for (int c = 0; c < 3; c++) neutral[c] /= nmax;
        add_rational(&d, 50728, T_RATIONAL, 3, neutral);
    }
    double be = log2(pmin);
    add_rational(&d, 50730, T_SRATIONAL, 1, &be); /* BaselineExposure */
    if (och == 3) {
        uint16_t illum = srgb ? 21 : 23; /* D65 : D50 */
        add_short(&d, 50778, 1, &illum); /* CalibrationIlluminant1 */
    }
    if (pos || mino) {
        /* finished positives: render the data as it is.  Raw converters
         * otherwise add their own contrast curve and black clip (Apple's raw
         * engine brightens a linear 0.5 to 0.8); identity ProfileToneCurve and
         * DefaultBlackRender "none" turn both off in Preview/Photos and
         * Lightroom alike */
        static const float ident[4] = { 0, 0, 1, 1 };
        add_ascii(&d, 50936, mino ? "dsd4 Minolta film curve" : "dsd4 inverted"); /* ProfileName */
        add_float(&d, 50940, 4, ident);  /* ProfileToneCurve */
        add_long(&d, 51110, 1);          /* DefaultBlackRender: none */
    }
    qsort(d.e, (size_t)d.n, sizeof *d.e, cmp_entry);

    /* layout: header, IFD, out-of-line values, strip tables, pixel data */
    const uint32_t ifd_off = 8, ifd_len = 2 + 12 * (uint32_t)d.n + 4;
    uint32_t off = ifd_off + ifd_len;
    uint32_t val_off[40];
    entry_t *eso = NULL, *esb = NULL;
    for (int i = 0; i < d.n; i++) {
        if (d.e[i].tag == 273) eso = &d.e[i];
        if (d.e[i].tag == 279) esb = &d.e[i];
    }
    for (int i = 0; i < d.n; i++) {
        val_off[i] = 0;
        if (d.e[i].tag == 273 || d.e[i].tag == 279) continue;
        if (d.e[i].len > 4) { val_off[i] = off; off += (d.e[i].len + 1) & ~1u; }
    }
    const uint32_t so_tab = off, sb_tab = so_tab + 4 * strips;
    off = sb_tab + 4 * strips;
    const uint32_t data_off = (off + 15) & ~15u;

    /* samples through the profile's curves, in the processed domain:
     * s' = TRC(s * pend) / pend; 16-bit data via a table per channel */
    uint16_t *lut = NULL;
    const int curves = pr.n[0] || pr.n[1] || pr.n[2];
    if (curves && img->bits == 16) {
        lut = malloc((size_t)3 * 65536 * sizeof *lut);
        if (!lut) { free_profile(&pr); return DSD4_ERR_NOMEM; }
        for (int c = 0; c < 3; c++)
            for (int v = 0; v < 65536; v++) {
                int tc = ch == 3 ? c : 1; /* gray scans use the green channel */
                double x = trc(&pr, tc, v / 65535.0 * pend[c]) / pend[c] * 65535.0;
                lut[(size_t)c * 65536 + v] = (uint16_t)(x > 65535 ? 65535 : x + 0.5);
            }
    }
    FILE *f = fopen(path, "wb");
    if (!f) { free(lut); free_profile(&pr); return DSD4_ERR_IO; }
    uint8_t b[12];
    fwrite("II", 1, 2, f);
    put16(b, 42);
    put32(b + 2, ifd_off);
    fwrite(b, 1, 6, f);
    put16(b, (uint16_t)d.n);
    fwrite(b, 1, 2, f);
    for (int i = 0; i < d.n; i++) {
        entry_t *x = &d.e[i];
        put16(b, x->tag);
        put16(b + 2, x->type);
        put32(b + 4, x->count);
        memset(b + 8, 0, 4);
        if (x == eso || x == esb) {
            if (strips == 1) put32(b + 8, x == eso ? data_off : (uint32_t)(row * img->height));
            else put32(b + 8, x == eso ? so_tab : sb_tab);
        } else if (x->len > 4) put32(b + 8, val_off[i]);
        else memcpy(b + 8, x->data, x->len);
        fwrite(b, 1, 12, f);
    }
    put32(b, 0);
    fwrite(b, 1, 4, f);
    for (int i = 0; i < d.n; i++)
        if (val_off[i]) {
            fwrite(d.e[i].data, 1, d.e[i].len, f);
            if (d.e[i].len & 1) fputc(0, f);
        }
    for (uint32_t s = 0; s < strips; s++) {
        put32(b, data_off + (uint32_t)(s * rps * row));
        fwrite(b, 1, 4, f);
    }
    for (uint32_t s = 0; s < strips; s++) {
        uint32_t rows = s + 1 < strips ? rps : (uint32_t)img->height - s * rps;
        put32(b, (uint32_t)(rows * row));
        fwrite(b, 1, 4, f);
    }
    while ((uint32_t)ftell(f) < data_off) fputc(0, f);

    /* pixels, little-endian */
    const size_t n = (size_t)img->width * img->height * och;
    uint8_t buf[8192];
    size_t k = 0;
    for (size_t o = 0; o < n; o++) {
        const int c = ch == 3 ? (int)(o % 3) : 0;
        const size_t i = (o / (size_t)och) * (size_t)ch + (size_t)c; /* source sample */
        float fv = 0;
        if (img->bits == 32) {
            fv = ((const float *)img->data)[i];
            if (curves) fv = (float)(trc(&pr, ch == 3 ? c : 1, fv * pend[c]) / pend[c]);
        }
        if (fl && bps == 32) {
            float v = fv;
            uint32_t u;
            memcpy(&u, &v, 4);
            put32(buf + k, u);
            k += 4;
        } else if (fl) {
            put16(buf + k, to_half(fv));
            k += 2;
        } else {
            uint16_t v;
            if (img->bits == 32) {
                double x = fv * 65535.0;
                v = (uint16_t)(x < 0 ? 0 : x > 65535 ? 65535 : x + 0.5);
            } else {
                v = ((const uint16_t *)img->data)[i];
                if (lut) v = lut[(size_t)c * 65536 + v];
            }
            put16(buf + k, v);
            k += 2;
        }
        if (k == sizeof buf) { fwrite(buf, 1, k, f); k = 0; }
    }
    fwrite(buf, 1, k, f);
    int ok = !ferror(f);
    fclose(f);
    free(lut);
    free_profile(&pr);
    return ok ? DSD4_OK : DSD4_ERR_IO;
}

/* ------------------------------------------------- deblur a DNG in place */

static uint16_t get16le(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t get32le(const uint8_t *p) { return (uint32_t)get16le(p) | (uint32_t)get16le(p + 2) << 16; }

static float from_half(uint16_t h)
{
    uint32_t sign = (uint32_t)(h & 0x8000) << 16, e = (h >> 10) & 0x1f, m = h & 0x3ff, x;
    if (e == 0) {
        if (!m) x = sign;
        else { /* subnormal */
            e = 113;
            while (!(m & 0x400)) { m <<= 1; e--; }
            x = sign | e << 23 | (m & 0x3ff) << 13;
        }
    } else if (e == 31) x = sign | 0x7f800000 | m << 13;
    else x = sign | (e + 112) << 23 | m << 13;
    float f;
    memcpy(&f, &x, 4);
    return f;
}

/* first value of a SHORT or LONG entry */
static uint32_t ifd_val(const uint8_t *e) { return get16le(e + 2) == T_SHORT ? get16le(e + 8) : get32le(e + 8); }

/* One of this writer's DNGs in memory: little-endian, one IFD, uncompressed
 * contiguous strips.  The samples are decoded into im (16-bit, or float for
 * half/32-bit float files). */
typedef struct {
    uint8_t *d;               /* the whole file */
    long len;
    uint8_t *samples;         /* where the pixel data starts in d */
    uint32_t bps;             /* bits per sample in the file */
    int profiled;             /* ProfileName says: this driver's finished positive */
} dng_file;

static int load_dng(const char *path, dng_file *df, dsd4_image *im)
{
    memset(df, 0, sizeof *df);
    memset(im, 0, sizeof *im);
    FILE *f = fopen(path, "rb");
    if (!f) return DSD4_ERR_IO;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *d = len > 16 ? malloc((size_t)len) : NULL;
    if (!d || fread(d, 1, (size_t)len, f) != (size_t)len) { fclose(f); free(d); return d ? DSD4_ERR_IO : DSD4_ERR_PARAM; }
    fclose(f);
    uint32_t w = 0, h = 0, spp = 0, bps = 0, fmt = 1, comp = 1, phot = 0, nstrip = 0, so = 0, dng = 0, xres = 0;
    int profiled = 0;
    if (memcmp(d, "II*\0", 4)) goto bad;
    uint32_t ifd = get32le(d + 4);
    if (ifd + 2 > (uint32_t)len) goto bad;
    unsigned n = get16le(d + ifd);
    if (ifd + 2 + 12ull * n > (uint64_t)len) goto bad;
    for (unsigned i = 0; i < n; i++) {
        const uint8_t *e = d + ifd + 2 + 12 * i;
        uint16_t tag = get16le(e);
        uint32_t cnt = get32le(e + 4);
        switch (tag) {
        case 256: w = ifd_val(e); break;
        case 257: h = ifd_val(e); break;
        case 258: bps = cnt <= 2 ? get16le(e + 8) : get16le(d + get32le(e + 8)); break;
        case 259: comp = ifd_val(e); break;
        case 262: phot = ifd_val(e); break;
        case 273: nstrip = cnt; so = cnt == 1 ? (uint32_t)(e - d) + 8 : get32le(e + 8);
                  if (get16le(e + 2) != T_LONG) nstrip = 0; break;
        case 277: spp = ifd_val(e); break;
        case 282: { uint32_t o = get32le(e + 8); if (o + 8 <= (uint32_t)len && get32le(d + o + 4)) xres = get32le(d + o) / get32le(d + o + 4); break; }
        case 339: fmt = cnt <= 2 ? get16le(e + 8) : get16le(d + get32le(e + 8)); break;
        case 50706: dng = 1; break;
        case 50936: { /* ProfileName: this driver's finished positives */
            uint32_t off = cnt <= 4 ? (uint32_t)(e - d) + 8 : get32le(e + 8);
            if (off + cnt <= (uint32_t)len)
                /* also files written before the rename to dsd4 */
                profiled = memmem(d + off, cnt, "dsd4 Minolta", 12) || memmem(d + off, cnt, "dsd4 inverted", 13) ||
                           memmem(d + off, cnt, "dsdual4 Minolta", 15) || memmem(d + off, cnt, "dsdual4 inverted", 16);
            break;
        }
        }
    }
    if (!dng || comp != 1 || phot != 34892 || !w || !h || (spp != 1 && spp != 3) || !nstrip ||
        so + 4ull * nstrip > (uint64_t)len || !((fmt == 1 && bps == 16) || (fmt == 3 && (bps == 16 || bps == 32))))
        goto bad;
    const uint32_t data = get32le(d + so);
    if (data + (uint64_t)w * h * spp * (bps / 8) > (uint64_t)len) goto bad;
    im->width = (int)w;
    im->height = (int)h;
    im->channels = (int)spp;
    im->bits = fmt == 1 ? 16 : 32;
    im->dpi = (int)xres;
    im->stride = (size_t)w * spp * (im->bits / 8);
    for (int c = 0; c < 3; c++) im->digital_gain[c] = 1.0;
    im->data = malloc(im->stride * h);
    if (!im->data) { free(d); return DSD4_ERR_NOMEM; }
    const size_t ns = (size_t)w * h * spp;
    const uint8_t *p = d + data;
    for (size_t i = 0; i < ns; i++) {
        if (im->bits == 16) ((uint16_t *)im->data)[i] = get16le(p + 2 * i);
        else if (bps == 16) ((float *)im->data)[i] = from_half(get16le(p + 2 * i));
        else { uint32_t u = get32le(p + 4 * i); memcpy((float *)im->data + i, &u, 4); }
    }
    *df = (dng_file){ d, len, d + data, bps, profiled };
    return DSD4_OK;
bad:
    free(d);
    return DSD4_ERR_PARAM;
}

int dsd4_read_dng(const char *path, dsd4_image *img)
{
    dng_file df;
    int r = load_dng(path, &df, img);
    if (!r) free(df.d);
    return r;
}

int dsd4_deblur_dng(const char *in, const char *out, int level, int tone, dsd4_deblur_info *info)
{
    if (info) memset(info, 0, sizeof *info);
    dng_file df;
    dsd4_image im;
    int r = load_dng(in, &df, &im);
    if (r) return r;
    r = DSD4_ERR_PARAM;
    if (tone < 0) {
        if (!df.profiled) goto out; /* raw scan: the caller has to say positive or negative */
        tone = DSD4_TONE_LINEAR;
    }
    if ((r = dsd4_deblur(&im, level, tone, info))) goto out;
    const size_t ns = (size_t)im.width * im.height * im.channels;
    uint8_t *p = df.samples;
    for (size_t i = 0; i < ns; i++) {
        if (im.bits == 16) put16(p + 2 * i, ((uint16_t *)im.data)[i]);
        else if (df.bps == 16) put16(p + 2 * i, to_half(((float *)im.data)[i]));
        else { uint32_t u; memcpy(&u, (float *)im.data + i, 4); put32(p + 4 * i, u); }
    }
    /* write beside the target, then replace it: never a half-written file */
    char tmp[4096];
    snprintf(tmp, sizeof tmp, "%s.deblur-tmp", out);
    r = DSD4_ERR_IO;
    FILE *f = fopen(tmp, "wb");
    if (!f) goto out;
    int ok = fwrite(df.d, 1, (size_t)df.len, f) == (size_t)df.len;
    ok = !fclose(f) && ok;
    if (!ok || rename(tmp, out)) { remove(tmp); goto out; }
    r = DSD4_OK;
out:
    free(im.data);
    free(df.d);
    return r;
}
