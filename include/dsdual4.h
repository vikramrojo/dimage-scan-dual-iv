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
} dsd4_scan_params;

#define DSD4_FOCUS_AUTO   0x7fff
#define DSD4_FOCUS_KEEP   0x7ffe

typedef struct {
    int width, height;        /* pixels */
    int channels;             /* 3 (RGB) or 1 (gray) */
    int bits;                 /* 8 or 16 */
    int dpi;
    size_t stride;            /* bytes per line */
    uint8_t *data;            /* 16 bit samples are host-endian */
    double digital_gain[3];
    int focus;                /* focus position used */
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
int  dsd4_set_focus(dsd4 *s, int position);
int  dsd4_autofocus(dsd4 *s, const dsd4_scan_params *p, int *best, dsd4_progress_fn cb, void *ud);

/* scanning */
void dsd4_default_params(dsd4_scan_params *p);
void dsd4_film_default_exposure(dsd4_film film, double exp[3]);
int  dsd4_scan(dsd4 *s, const dsd4_scan_params *p, dsd4_image *img, dsd4_progress_fn cb, void *ud);
void dsd4_image_free(dsd4_image *img);

/* simple post-processing helpers for raw scans */
void dsd4_invert_negative(dsd4_image *img, int auto_levels);
int  dsd4_write_tiff(const dsd4_image *img, const char *path);

#ifdef __cplusplus
}
#endif
#endif
