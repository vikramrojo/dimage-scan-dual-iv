/*
 * "Minolta processing" for negatives: the original DiMAGE Scan software's
 * auto-exposure and film-curve pipeline (MFSLib2891 ScanSequence with AE on),
 * built from the bit-exact kernel ports in minolta_ae.c and minolta_lut.c.
 *
 * Sequence, as in the original:
 *   1. autofocus (AutoFocus)
 *   2. AE prescan: preview-mode scan (800 dpi, 16 bit) of the whole frame at
 *      the AE base exposure (GetAEGain), colour or mono like the film
 *      (GetAEInputData type 2 / 4)
 *   3. AE analysis (FUN_10036090) with the scanner's shading tables as the
 *      black (dark table) and light (white gain table) reference lines,
 *      full-width (3120 px, APS: from 0x288, 0x9a8 px) as ReturnShdData
 *      delivers them
 *   4. gains = AE gains x AE base exposure (AutoExposure) -> exposure of the
 *      final scan (LutAndGainSetting with no user adjustment: stretch 1)
 *   5. film curves from the AE result (CalLUT -> FUN_10036f40, fixed_curve
 *      off, product type 12) converted to final 16 bit tables (FUN_10037d70)
 *   6. final scan, then every sample is looked up through its channel's
 *      table (ApplyLUT); gray uses table 0.
 * Colour matching (MCM with MLTF3200n.icc) is left to ColorSync: the result
 * is what the "Nega Input" profile describes.
 */
#include "dsd4_internal.h"
#include "minolta_ae.h"
#include "minolta_lut.h"

#include <stdlib.h>
#include <string.h>

#define PRODUCT_TYPE_DUAL4 12 /* MFSScannerInfo type of the FS-V31 */

/* GetAEGain for the film types handled here */
static void ae_base_gain(dsd4_film f, double g[3])
{
    g[0] = g[1] = g[2] = 1.0;
    if (f == DSD4_FILM_NEGATIVE_COLOR) {
        g[0] /= 0.65;
        g[1] /= 0.3;
        g[2] /= 0.2;
    }
}

/* ApplyLUT: p[c] = lut[c][p[c]] on interleaved 16 bit samples.  Float
 * samples (multi-sampling) are looked up between table entries, keeping
 * their extra precision. */
static void apply_lut(dsd4_image *im, uint16_t *const lut[3])
{
    const size_t n = (size_t)im->width * im->height;
    const int ch = im->channels;
    if (im->bits == 32) {
        float *f = (float *)im->data;
        for (size_t i = 0; i < n; i++)
            for (int c = 0; c < ch; c++) {
                double x = f[i * ch + c] * 65535.0;
                x = x < 0 ? 0 : x > 65535 ? 65535 : x;
                int k = (int)x;
                if (k > 65534) k = 65534;
                double t = x - k;
                f[i * ch + c] = (float)(((1 - t) * lut[c][k] + t * lut[c][k + 1]) / 65535.0);
            }
        return;
    }
    uint16_t *p = (uint16_t *)im->data;
    for (size_t i = 0; i < n; i++)
        for (int c = 0; c < ch; c++) p[i * ch + c] = lut[c][p[i * ch + c]];
}


/* Shared core: AE analysis of a prescan-sized 16-bit image taken at the AE
 * base exposure, then the final 16-bit film tables.  dark/white are the
 * scanner's final shading tables [3][3200].  gain_out: final exposure. */
static int ae_and_curves(const uint16_t (*dark_tab)[SHD_W], const uint16_t (*white_tab)[SHD_W], int aps,
                         const dsd4_image *pim, int color, double gain_out[3], uint16_t *final_tab[3],
                         uint16_t *film_store, mfs_ae_result *ae_out)
{
    const int film_type = color ? MFS_FILM_COLOR_NEGA : MFS_FILM_BW_NEGA;
    const uint32_t ref_n = aps ? 0x9a8 : 0xc30, ref_start = aps ? 0x288 : 0;
    uint16_t *dark = malloc((size_t)ref_n * 3 * 2), *light = malloc((size_t)ref_n * 3 * 2);
    if (!dark || !light) { free(dark); free(light); return DSD4_ERR_NOMEM; }
    for (uint32_t i = 0; i < ref_n; i++)
        for (int c = 0; c < 3; c++) {
            dark[i * 3 + c] = dark_tab[c][ref_start + i];
            light[i * 3 + c] = white_tab[c][ref_start + i];
        }
    /* The original's block grid (FUN_1003a540) can run up to 3 lines past the
     * last image row when the AE area reaches the bottom; give it 8 padding
     * lines repeating the last row instead of reading past the buffer. */
    const size_t row = (size_t)pim->width * pim->channels * 2;
    uint8_t *padded = malloc(row * ((size_t)pim->height + 8));
    if (!padded) { free(dark); free(light); return DSD4_ERR_NOMEM; }
    memcpy(padded, pim->data, row * pim->height);
    for (int k = 0; k < 8; k++) memcpy(padded + row * ((size_t)pim->height + k), pim->data + row * (pim->height - 1), row);

    mfs_ae_input in;
    memset(&in, 0, sizeof in);
    in.film_type = (uint32_t)film_type;
    const int pch = pim->channels;
    for (int c = 0; c < (color ? 3 : 1); c++) {
        in.image[c] = (mfs_ae_plane){ padded + 2 * c, (uint32_t)pim->width, (uint32_t)pim->height, 2,
                                      (uint32_t)(2 * pch), (uint32_t)(2 * pch * pim->width) };
        /* reference lines are interleaved RGB; mono uses the green sample */
        const int rc = color ? c : 1;
        in.ref_dark[c] = (mfs_ae_plane){ (const uint8_t *)(dark + rc), ref_n, 1, 2, 6, 0 };
        in.ref_light[c] = (mfs_ae_plane){ (const uint8_t *)(light + rc), ref_n, 1, 2, 6, 0 };
    }
    in.area_w = (uint32_t)pim->width;
    in.area_h = (uint32_t)pim->height;
    in.product = PRODUCT_TYPE_DUAL4;
    mfs_ae_result ae;
    memset(&ae, 0, sizeof ae);
    int r = mfs_ae_analyze(&in, &ae);
    free(padded);
    free(dark);
    free(light);
    if (r != 0) return DSD4_ERR_SCANNER;
    /* AutoExposure: gains x AE base gain (mono: every channel from the first) */
    double base[3];
    ae_base_gain(color ? DSD4_FILM_NEGATIVE_COLOR : DSD4_FILM_NEGATIVE_BW, base);
    for (int c = 0; c < 3; c++) {
        ae.gain[c] *= color ? base[c] : base[1];
        gain_out[c] = color ? ae.gain[c] : ae.gain[0];
    }
    uint8_t raw[0x70];
    mfs_ae_result_to_be(&ae, raw);
    mfs_lut_params lp;
    memset(&lp, 0, sizeof lp);
    lp.film_type = film_type;
    lp.fixed_curve = 0;
    lp.product_type = PRODUCT_TYPE_DUAL4;
    mfs_lut_ae_from_be(&lp.ae, raw);
    mfs_lut_desc film[3], fin[3];
    for (int c = 0; c < 3; c++) {
        film[c] = (mfs_lut_desc){ film_store + (size_t)c * 65536, 4, 4 };
        fin[c] = (mfs_lut_desc){ final_tab[c], 4, 4 };
    }
    mfs_lut_state st;
    memset(&st, 0, sizeof st);
    if (mfs_lut_film_curve(&lp, film, &st) != 0) return DSD4_ERR_SCANNER;
    for (int c = 0; c < (color ? 3 : 1); c++)
        if (mfs_lut_convert(&film[c], &fin[c]) != 0) return DSD4_ERR_SCANNER;
    if (ae_out) *ae_out = ae;
    return DSD4_OK;
}

static void to8(dsd4_image *img)
{
    size_t n = (size_t)img->width * img->height * img->channels;
    for (size_t i = 0; i < n; i++) img->data[i] = (uint8_t)(((uint16_t *)img->data)[i] >> 8);
    img->bits = 8;
    img->stride = (size_t)img->width * img->channels;
}

/* the frame under the lens, or -1 */
static int current_frame(dsd4 *s)
{
    dsd4_holder_info h;
    return dsd4_get_holder(s, &h) == DSD4_OK ? h.current_frame : -1;
}

/* remember an AE result for the current frame (Minolta preview -> scan) */
static void cache_store(dsd4 *s, int frame, dsd4_film film, const double gain[3], const mfs_ae_result *ae,
                        uint16_t *const tab[3])
{
    if (!s->ae_cache.tab) s->ae_cache.tab = malloc((size_t)3 * 65536 * sizeof *s->ae_cache.tab);
    if (!s->ae_cache.tab || frame < 0) { s->ae_cache.valid = 0; return; }
    for (int c = 0; c < 3; c++) {
        memcpy(s->ae_cache.tab + (size_t)c * 65536, tab[c], 65536 * sizeof *s->ae_cache.tab);
        s->ae_cache.gain[c] = gain[c];
        s->ae_cache.black[c] = ae->ch[c].dark;
    }
    s->ae_cache.frame = frame;
    s->ae_cache.film = (int)film;
    s->ae_cache.holder = s->holder_type;
    s->ae_cache.valid = 1;
}

static int cache_hit(dsd4 *s, int frame, dsd4_film film)
{
    return s->ae_cache.valid && frame >= 0 && s->ae_cache.frame == frame && s->ae_cache.film == (int)film &&
           s->ae_cache.holder == s->holder_type;
}

/* AE prescan parameters: preview mode, 800 dpi, 16 bit, whole frame, AE base exposure */
static dsd4_scan_params prescan_params(const dsd4_scan_params *p)
{
    dsd4_scan_params pre = *p;
    pre.preview = 1;
    pre.dpi = 800;
    pre.bits = 16;
    pre.raw = 0;
    pre.float_out = 0;
    pre.multi_sample = 1;
    pre.x = pre.y = pre.w = pre.h = 0;
    double base[3];
    ae_base_gain(p->film, base);
    for (int c = 0; c < 3; c++) pre.exposure[c] = base[c];
    return pre;
}

int dsd4_preview_minolta(dsd4 *s, const dsd4_scan_params *sp, dsd4_image *img, dsd4_minolta_info *info,
                         dsd4_progress_fn cb, void *ud)
{
    memset(img, 0, sizeof *img);
    if (info) memset(info, 0, sizeof *info);
    if (sp->film != DSD4_FILM_NEGATIVE_COLOR && sp->film != DSD4_FILM_NEGATIVE_BW) return DSD4_ERR_PARAM;
    const int color = sp->film == DSD4_FILM_NEGATIVE_COLOR;
    dsd4_scan_params p = *sp;
    int r;
    if (p.frame > 0) {
        if (cb) cb(ud, "moving to frame", -1);
        if ((r = dsd4_select_frame(s, p.frame))) return r;
        p.frame = 0;
    }
    if (p.focus == DSD4_FOCUS_AUTO) {
        int best = 0;
        if ((r = dsd4_autofocus(s, &p, &best, cb, ud))) return r;
        p.focus = best;
    }
    dsd4_scan_params pre = prescan_params(&p);
    if (cb) cb(ud, "preview (Minolta processing)", -1);
    if ((r = dsd4_scan(s, &pre, img, cb, ud))) return r;

    uint16_t *tabs = malloc((size_t)6 * 65536 * sizeof *tabs);
    if (!tabs) { dsd4_image_free(img); return DSD4_ERR_NOMEM; }
    uint16_t *final_tab[3] = { tabs + 3 * 65536, tabs + 4 * 65536, tabs + 5 * 65536 };
    double gain[3], base[3];
    mfs_ae_result ae;
    r = ae_and_curves((const uint16_t (*)[SHD_W])s->dark, (const uint16_t (*)[SHD_W])s->white,
                      s->holder_type == 1, img, color, gain, final_tab, tabs, &ae);
    if (r) {
        free(tabs);
        dsd4_image_free(img);
        return r == DSD4_ERR_NOMEM ? r : DSD4_ERR_SCANNER;
    }
    cache_store(s, current_frame(s), p.film, gain, &ae, final_tab);
    /* the final scan would be exposed at 'gain': scale the prescan likewise
     * (the response is linear), then the film curves */
    ae_base_gain(p.film, base);
    const int ch = img->channels;
    const size_t n = (size_t)img->width * img->height;
    uint16_t *px = (uint16_t *)img->data;
    for (int c = 0; c < ch; c++) {
        const double k = gain[color ? c : 0] / base[color ? c : 1];
        for (size_t i = 0; i < n; i++) {
            double v = px[i * ch + c] * k;
            px[i * ch + c] = final_tab[c][(uint16_t)(v > 65535 ? 65535 : v + 0.5)];
        }
    }
    free(tabs);
    if (sp->dpi > 0 && sp->dpi < 800) {
        int dw = img->width * sp->dpi / 800, dh = img->height * sp->dpi / 800;
        uint8_t *rs = dsd4_resample16(px, img->width, img->height, ch, dw > 0 ? dw : 1, dh > 0 ? dh : 1);
        if (rs) {
            free(img->data);
            img->data = rs;
            img->width = dw > 0 ? dw : 1;
            img->height = dh > 0 ? dh : 1;
            img->stride = (size_t)img->width * ch * 2;
            img->dpi = sp->dpi;
        }
    }
    if (sp->bits == 8) to8(img);
    img->minolta = 1;
    dsd4_logf(s, 1, "Minolta AE: exposure %.3f/%.3f/%.3f", gain[0], gain[1], gain[2]);
    if (info) {
        for (int c = 0; c < 3; c++) {
            info->exposure[c] = gain[c];
            info->black_level[c] = ae.ch[c].dark;
        }
        info->focus = p.focus;
    }
    return DSD4_OK;
}

int dsd4_scan_minolta(dsd4 *s, const dsd4_scan_params *sp, dsd4_image *img, dsd4_minolta_info *info,
                      dsd4_progress_fn cb, void *ud)
{
    memset(img, 0, sizeof *img);
    if (info) memset(info, 0, sizeof *info);
    if (sp->film != DSD4_FILM_NEGATIVE_COLOR && sp->film != DSD4_FILM_NEGATIVE_BW) return DSD4_ERR_PARAM;
    const int color = sp->film == DSD4_FILM_NEGATIVE_COLOR;
    dsd4_scan_params p = *sp;
    int r;

    /* 1. frame and focus first, so prescan and final scan share them */
    if (p.frame > 0) {
        if (cb) cb(ud, "moving to frame", -1);
        if ((r = dsd4_select_frame(s, p.frame))) return r;
        p.frame = 0;
    }
    if (p.focus == DSD4_FOCUS_AUTO) {
        int best = 0;
        if ((r = dsd4_autofocus(s, &p, &best, cb, ud))) return r;
        p.focus = best;
    }

    uint16_t *tabs = malloc((size_t)6 * 65536 * sizeof *tabs);
    if (!tabs) return DSD4_ERR_NOMEM;
    uint16_t *final_tab[3] = { tabs + 3 * 65536, tabs + 4 * 65536, tabs + 5 * 65536 };
    double gain[3];
    int black[3];
    const int frame = current_frame(s);
    if (cache_hit(s, frame, p.film)) {
        /* a Minolta preview of this frame already ran the AE prescan */
        for (int c = 0; c < 3; c++) {
            memcpy(final_tab[c], s->ae_cache.tab + (size_t)c * 65536, 65536 * sizeof *tabs);
            gain[c] = s->ae_cache.gain[c];
            black[c] = s->ae_cache.black[c];
        }
        dsd4_logf(s, 1, "Minolta AE: exposure %.3f/%.3f/%.3f (from the preview)", gain[0], gain[1], gain[2]);
    } else {
        /* 2. AE prescan */
        dsd4_scan_params pre = prescan_params(&p);
        if (cb) cb(ud, "Minolta auto-exposure prescan", -1);
        dsd4_image pim;
        if ((r = dsd4_scan(s, &pre, &pim, cb, ud))) { free(tabs); return r; }

        /* 3.-5. AE analysis and film curves */
        mfs_ae_result ae;
        r = ae_and_curves((const uint16_t (*)[SHD_W])s->dark, (const uint16_t (*)[SHD_W])s->white,
                          s->holder_type == 1, &pim, color, gain, final_tab, tabs, &ae);
        dsd4_image_free(&pim);
        if (r) {
            free(tabs);
            dsd4_logf(s, 1, "Minolta auto-exposure / film curve computation failed");
            s->last_error = AVE_GENERIC;
            return r == DSD4_ERR_NOMEM ? r : DSD4_ERR_SCANNER;
        }
        for (int c = 0; c < 3; c++) black[c] = ae.ch[c].dark;
        cache_store(s, frame, p.film, gain, &ae, final_tab);
        dsd4_logf(s, 1, "Minolta AE: exposure %.3f/%.3f/%.3f", gain[0], gain[1], gain[2]);
    }

    /* 6. final scan at the AE exposure, then the film curves */
    dsd4_scan_params fp = p;
    fp.preview = 0;
    fp.bits = 16;
    fp.float_out = sp->float_out; /* multi-sampled DNGs keep their precision */
    fp.raw = 0;
    for (int c = 0; c < 3; c++) fp.exposure[c] = gain[c];
    if ((r = dsd4_scan(s, &fp, img, cb, ud))) { free(tabs); return r; }
    if (cb) cb(ud, "applying film curves", -1);
    apply_lut(img, final_tab);
    free(tabs);
    if (sp->bits == 8 && img->bits == 16) to8(img); /* ApplyLUT's 8 bit output: lut value >> 8 */
    img->minolta = 1;
    if (info) {
        for (int c = 0; c < 3; c++) info->exposure[c] = gain[c];
        info->focus = p.focus;
        for (int c = 0; c < 3; c++) info->black_level[c] = black[c];
    }
    return DSD4_OK;
}

/* Offline variant for an existing raw (linear, uninverted) negative scan.
 * The AE prescan is synthesised by box-downsampling the scan to ~800 dpi and
 * scaling it from the scan's exposure to the AE base exposure; the AE result
 * exposure is then applied by scaling the scan likewise (the scanner's
 * response is linear, so this approximates the rescan the live pipeline
 * does; clipped highlights in the input stay clipped).  The shading tables
 * come from the calibration cache written by "dsdual4 init". */
int dsd4_minolta_convert(const dsd4_image *in, dsd4_film film, const double in_exposure[3], dsd4_image *out,
                         dsd4_minolta_info *info)
{
    memset(out, 0, sizeof *out);
    if (info) memset(info, 0, sizeof *info);
    if (film != DSD4_FILM_NEGATIVE_COLOR && film != DSD4_FILM_NEGATIVE_BW) return DSD4_ERR_PARAM;
    if (in->bits != 16 || (in->channels != 1 && in->channels != 3)) return DSD4_ERR_PARAM;
    const int color = film == DSD4_FILM_NEGATIVE_COLOR;
    if (color && in->channels != 3) return DSD4_ERR_PARAM;
    for (int c = 0; c < 3; c++) if (!(in_exposure[c] > 0)) return DSD4_ERR_PARAM;

    static uint16_t dark[3][SHD_W], white[3][SHD_W];
    char path[1024];
    const char *home = getenv("HOME");
    FILE *f;
    snprintf(path, sizeof path, "%s/Library/Application Support/dsdual4/darkAvg.bin", home ? home : ".");
    if (!(f = fopen(path, "rb")) || fread(dark, 1, sizeof dark, f) != sizeof dark) { if (f) fclose(f); return DSD4_ERR_IO; }
    fclose(f);
    snprintf(path, sizeof path, "%s/Library/Application Support/dsdual4/whiteAvg.bin", home ? home : ".");
    if (!(f = fopen(path, "rb")) || fread(white, 1, sizeof white, f) != sizeof white) { if (f) fclose(f); return DSD4_ERR_IO; }
    fclose(f);

    double base[3];
    ae_base_gain(film, base);
    const int ich = in->channels, och = color ? 3 : 1;
    const int k = in->dpi > 800 ? (in->dpi + 400) / 800 : 1;
    dsd4_image pim = { in->width / k, in->height / k, och, 16, 800, 0, NULL, { 1, 1, 1 }, 0, { 1, 1, 1 }, 0, 0 };
    pim.stride = (size_t)pim.width * och * 2;
    pim.data = malloc(pim.stride * pim.height);
    if (!pim.data) return DSD4_ERR_NOMEM;
    const uint16_t *src = (const uint16_t *)in->data;
    for (int y = 0; y < pim.height; y++)
        for (int x = 0; x < pim.width; x++)
            for (int c = 0; c < och; c++) {
                const int sc = ich == 3 ? (color ? c : 1) : 0; /* mono: green, like the gray scan mode */
                uint64_t sum = 0;
                for (int yy = 0; yy < k; yy++)
                    for (int xx = 0; xx < k; xx++)
                        sum += src[((size_t)(y * k + yy) * in->width + (x * k + xx)) * ich + sc];
                double v = (double)sum / (k * k) * (base[color ? c : 1] / in_exposure[color ? c : 1]);
                ((uint16_t *)pim.data)[((size_t)y * pim.width + x) * och + c] = (uint16_t)(v > 65535 ? 65535 : v + 0.5);
            }

    uint16_t *tabs = malloc((size_t)6 * 65536 * sizeof *tabs);
    if (!tabs) { free(pim.data); return DSD4_ERR_NOMEM; }
    uint16_t *final_tab[3] = { tabs + 3 * 65536, tabs + 4 * 65536, tabs + 5 * 65536 };
    double gain[3];
    mfs_ae_result ae;
    int r = ae_and_curves((const uint16_t (*)[SHD_W])dark, (const uint16_t (*)[SHD_W])white, 0, &pim, color,
                          gain, final_tab, tabs, &ae);
    free(pim.data);
    if (r) { free(tabs); return r; }

    *out = *in;
    out->channels = och;
    out->bits = 16;
    out->stride = (size_t)in->width * och * 2;
    out->data = malloc(out->stride * in->height);
    if (!out->data) { free(tabs); return DSD4_ERR_NOMEM; }
    uint16_t *dst = (uint16_t *)out->data;
    const size_t n = (size_t)in->width * in->height;
    for (int c = 0; c < och; c++) {
        const int sc = ich == 3 ? (color ? c : 1) : 0;
        const double scale = gain[color ? c : 0] / in_exposure[color ? c : 1];
        for (size_t i = 0; i < n; i++) {
            double v = src[i * ich + sc] * scale;
            uint16_t s16 = (uint16_t)(v > 65535 ? 65535 : v + 0.5);
            dst[i * och + c] = final_tab[c][s16];
        }
    }
    free(tabs);
    if (info) {
        for (int c = 0; c < 3; c++) info->exposure[c] = gain[c];
        for (int c = 0; c < 3; c++) info->black_level[c] = ae.ch[c].dark;
    }
    return DSD4_OK;
}
