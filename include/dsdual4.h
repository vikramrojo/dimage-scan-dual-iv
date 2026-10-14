/*
 * libdsdual4 - user-space driver for the Konica Minolta DiMAGE Scan Dual IV
 * (USB 132b:000a, firmware product id "FS-V31").
 *
 * Reimplemented from the PowerPC Mac driver (MFSBaseLib2891, MFSIOUsb2891).
 */
#ifndef DSDUAL4_H
#define DSDUAL4_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dsd4 dsd4;
typedef struct dsd4_transport dsd4_transport;

typedef void (*dsd4_log_fn)(void *ud, int level, const char *msg);
/* progress: stage text and fraction 0..1 (negative when unknown) */
typedef void (*dsd4_progress_fn)(void *ud, const char *stage, double fraction);

enum dsd4_status {
    DSD4_OK = 0,
    DSD4_ERR_IO = -1,          /* USB transport failure */
    DSD4_ERR_SCANNER = -2,     /* scanner reported an error, see dsd4_last_error() */
    DSD4_ERR_PARAM = -3,
    DSD4_ERR_NOMEM = -4,
    DSD4_ERR_NO_HOLDER = -5,
    DSD4_ERR_WRONG_HOLDER = -6,
    DSD4_ERR_NO_FRAME = -7,
    DSD4_ERR_CANCELLED = -8,
    DSD4_ERR_FIRMWARE = -9,
};

typedef enum {
    DSD4_HOLDER_NONE = 0,   /* nothing inserted */
    DSD4_HOLDER_STRIP,      /* 35 mm film strip holder (6 frames) */
    DSD4_HOLDER_SLIDE,      /* 35 mm mounted slide holder (4 slides) */
    DSD4_HOLDER_APS,        /* APS cartridge adapter */
    DSD4_HOLDER_UNKNOWN,
} dsd4_holder;

typedef enum {
    DSD4_FILM_POSITIVE_COLOR = 0,
    DSD4_FILM_NEGATIVE_COLOR,
    DSD4_FILM_POSITIVE_BW,
    DSD4_FILM_NEGATIVE_BW,
} dsd4_film;

typedef struct {
    char vendor[9];
    char product[17];
    char revision[5];
    int optical_dpi;          /* 3200 */
    int max_pixels;           /* 3120 at optical resolution, 35 mm */
    int max_lines;            /* 4680 */
    int firmware_needs_update;
    uint8_t inquiry[96];
} dsd4_info;

typedef struct {
    dsd4_holder holder;
    int raw_type;             /* READ 0x87 byte 0 */
    int frames;               /* number of frame positions */
    int current_frame;        /* READ 0x87 byte 1 */
} dsd4_holder_info;

typedef struct {
    dsd4_film film;
    int dpi;                  /* output resolution, 100..3200 */
    int bits;                 /* 8 or 16 per channel */
    int frame;                /* 1..N; 0 keeps the current position */
    /* area in pixels at 'dpi', relative to the frame; w/h 0 = whole frame */
    int x, y, w, h;
    double exposure[3];       /* per-channel exposure factors; 0 = film default */
    int focus;                /* focus motor position; DSD4_FOCUS_AUTO for AF */
    int preview;              /* 1: fast 800 dpi preview of the whole frame */
    int multi_sample;         /* 1 (default), 2, 4 or 8 */
    int raw;                  /* 1: no host-side digital gain */
    int af_mode;              /* DSD4_AF_FAST (default) or DSD4_AF_CLASSIC */
    int float_out;            /* 1: 32-bit float samples (image bits = 32), keeping
                                 the full precision of multi-sampling */
    int unlimited_exposure;   /* 1: allow exposures above the original software's
                                 3x cap, up to the hardware maximum (HDR) */
} dsd4_scan_params;

/* autofocus: fast (coarse/fine search with a peak fit, ~3x fewer lens moves)
 * or the original software's exhaustive sequence */
#define DSD4_AF_FAST    0
#define DSD4_AF_CLASSIC 1

#define DSD4_FOCUS_AUTO   0x7fff
#define DSD4_FOCUS_KEEP   0x7ffe

typedef struct {
    int width, height;        /* pixels */
    int channels;             /* 3 (RGB) or 1 (gray) */
    int bits;                 /* 8, 16, or 32 (float, 1.0 = full scale) */
    int dpi;
    size_t stride;            /* bytes per line */
    uint8_t *data;            /* 16 bit and float samples are host-endian */
    double digital_gain[3];   /* host gain applied */
    int focus;                /* focus position used */
    double raw_gain[3];       /* raw scans: the gain normal output would have
                                 applied (white balance for raw converters) */
    int inverted;             /* 1: a negative turned positive by
                                 dsd4_invert_negative (no longer scanner colour) */
    int minolta;              /* 1: Minolta-processed positive (film curves applied;
                                 colour in MLTF3200n "Nega Input" encoding, gray
                                 gamma 2.2) */
} dsd4_image;

/* lifecycle */
int  dsd4_open_usb(dsd4 **out, char *err, size_t errlen);
int  dsd4_open_transport(dsd4 **out, dsd4_transport *t);
void dsd4_close(dsd4 *s);
void dsd4_set_log(dsd4 *s, dsd4_log_fn fn, void *ud, int verbose);
void dsd4_set_firmware_path(dsd4 *s, const char *path);
int  dsd4_last_error(dsd4 *s);
const char *dsd4_strerror(int status);
const char *dsd4_scanner_error_text(int code);

/* queries */
int  dsd4_get_info(dsd4 *s, dsd4_info *info);
int  dsd4_get_holder(dsd4 *s, dsd4_holder_info *h);

/* bring-up: firmware check, lamp warm-up and shading calibration as done by
 * the original driver (MFSBaseLibInit / InitializeBaseScanner / Setup). */
int  dsd4_initialize(dsd4 *s, int allow_firmware_update, dsd4_progress_fn cb, void *ud);

/* mechanics */
int  dsd4_select_frame(dsd4 *s, int frame);
int  dsd4_eject(dsd4 *s);
int  dsd4_home(dsd4 *s);
/* SEND 0x92 (reset to initial state), then wait up to 'seconds' until the
 * scanner answers TEST UNIT READY. Clears calibration state in this session. */
int  dsd4_reset(dsd4 *s, int seconds);
int  dsd4_set_focus(dsd4 *s, int position);
int  dsd4_autofocus(dsd4 *s, const dsd4_scan_params *p, int *best, dsd4_progress_fn cb, void *ud);

/* scanning */
void dsd4_default_params(dsd4_scan_params *p);
void dsd4_film_default_exposure(dsd4_film film, double exp[3]);
int  dsd4_scan(dsd4 *s, const dsd4_scan_params *p, dsd4_image *img, dsd4_progress_fn cb, void *ud);
void dsd4_image_free(dsd4_image *img);

/* Negatives with the original Minolta software's processing: autofocus, an
 * auto-exposure prescan analysed by the original AE algorithm, a final scan
 * at the AE exposure and the original film curves (inversion, orange mask /
 * base removal).  The result is a positive 16-bit (or 8-bit) image in the
 * colour space described by Konica Minolta's MLTF3200n.icc ("Nega Input"). */
typedef struct {
    double exposure[3];       /* exposure factors chosen by auto-exposure */
    int focus;
    int black_level[3];       /* film base / black levels found (10 bit) */
} dsd4_minolta_info;
int  dsd4_scan_minolta(dsd4 *s, const dsd4_scan_params *p, dsd4_image *img, dsd4_minolta_info *info,
                       dsd4_progress_fn cb, void *ud);

/* Preview of a negative with the Minolta processing from a single 800 dpi
 * scan (it is also the AE prescan).  The AE result is remembered for the
 * frame, so a dsd4_scan_minolta() of the same frame skips its prescan. */
int  dsd4_preview_minolta(dsd4 *s, const dsd4_scan_params *p, dsd4_image *img, dsd4_minolta_info *info,
                          dsd4_progress_fn cb, void *ud);

/* HDR: a raw float scan at the requested exposure plus a second pass 'ev'
 * stops brighter (capped by the hardware), aligned to sub-pixel accuracy
 * and merged in linear light: about 'ev' more stops of clean shadow data. */
typedef struct {
    double ev;                /* exposure difference actually achieved */
    double ratio[3];          /* measured long/short ratio per channel */
    double shift;             /* vertical offset of the second pass, lines */
} dsd4_hdr_info;
int  dsd4_scan_hdr(dsd4 *s, const dsd4_scan_params *p, double ev, dsd4_image *img, dsd4_hdr_info *info,
                   dsd4_progress_fn cb, void *ud);
/* (p->raw = 0: the merged image gets the digital gain a normal scan applies) */

/* dsd4_scan_minolta with an HDR final scan (ev > 0; ev = 0: a normal one):
 * the film curves are applied to the merged, float image.  hdr may be NULL. */
int  dsd4_scan_minolta_hdr(dsd4 *s, const dsd4_scan_params *p, double ev, dsd4_image *img,
                           dsd4_minolta_info *info, dsd4_hdr_info *hdr, dsd4_progress_fn cb, void *ud);

/* Deblur: reduces camera blur (missed focus) in the photograph, measured per
 * region; frames that are sharp are left alone.  Brightness only, film
 * grain and dust kept as they are.  tone: how the samples encode light. */
#define DSD4_DEBLUR_OFF  0
#define DSD4_DEBLUR_LOW  1      /* 60 % of the measured blur removed */
#define DSD4_DEBLUR_HIGH 2      /* 90 % */
#define DSD4_TONE_LINEAR   0    /* positive, linear light */
#define DSD4_TONE_GAMMA22  1    /* positive, gamma-encoded (Minolta TIFFs) */
#define DSD4_TONE_NEGATIVE 2    /* negative, linear transmittance */
typedef struct {
    double sigma;             /* typical camera blur found, sigma in pixels */
    double sigma_min, sigma_max; /* range removed across the frame */
    int tiles, measured;      /* regions, and those with edges to measure */
    int applied;              /* 0: sharp enough (or nothing measurable), unchanged */
} dsd4_deblur_info;
int  dsd4_deblur(dsd4_image *img, int level, int tone, dsd4_deblur_info *info);
/* Deblur a DNG written by dsd4_write_dng, keeping it raw: only the samples
 * change, every tag stays.  out may equal in (replaced safely).  tone -1:
 * from the file (finished positives); raw scans need DSD4_TONE_LINEAR
 * (slides) or DSD4_TONE_NEGATIVE. */
int  dsd4_deblur_dng(const char *in, const char *out, int level, int tone, dsd4_deblur_info *info);

/* Linear DNG ("raw") output for raw converters (Lightroom, darktable, Negative
 * Lab Pro).  16-bit images are written as integers.  Float images: float_bits
 * 16 (half float, what Lightroom's HDR DNGs use and macOS decodes), 32 (full
 * float; darktable/RawTherapee, not macOS), or 0 (rounded to 16-bit integer).
 * icc: the scanner's input profile (MLTF3200p.icc, "Posi Linear"); its
 * matrix becomes the DNG colour matrix and its per-channel curves are applied
 * to the samples.  NULL: built-in matrix, linear curves.  An inverted
 * negative (img->inverted) is written as a linear sRGB positive instead:
 * the scanner's colour calibration describes the light through the film,
 * not the inverted picture. */
int  dsd4_write_dng(const dsd4_image *img, const char *path, int float_bits, const void *icc, size_t icc_len);

/* Diagnostics for multi-pass super-resolution ("dsdual4 research"): which =
 * "nyquist", "focus", "jitter", "yres" or "all".  Scans strips of the
 * current/selected frame and writes a report; outdir (or NULL) receives
 * DNG crops. */
int  dsd4_research(dsd4 *s, const dsd4_scan_params *p, const char *which, const char *outdir, FILE *report,
                   dsd4_progress_fn cb, void *ud);

/* Offline: apply the Minolta processing to an existing raw (linear, not
 * inverted) negative scan; in_exposure = the exposure factors it was scanned
 * with.  Needs the calibration cache written by dsd4_initialize(). */
int  dsd4_minolta_convert(const dsd4_image *in, dsd4_film film, const double in_exposure[3],
                          dsd4_image *out, dsd4_minolta_info *info);
/* Read an uncompressed 8/16-bit gray/RGB TIFF (as written by dsdual4 or
 * re-saved by Preview). */
int  dsd4_read_tiff(const char *path, dsd4_image *img);

/* simple post-processing helpers for raw scans */
/* negative -> positive in density space: film base (orange mask) removal and
 * per-channel contrast matching.  Linear output (8/16 bit or float); sets
 * img->inverted. */
void dsd4_invert_negative(dsd4_image *img, int auto_levels);
int  dsd4_write_tiff(const dsd4_image *img, const char *path);
/* same, embedding an ICC profile (TIFF tag 34675) when icc != NULL */
int  dsd4_write_tiff_icc(const dsd4_image *img, const char *path, const void *icc, size_t icc_len);
/* linear 16-bit -> 8-bit for viewing: 0.1 %/99.9 % stretch and gamma 2.2 */
void dsd4_to_display(dsd4_image *img);

#ifdef __cplusplus
}
#endif
#endif
