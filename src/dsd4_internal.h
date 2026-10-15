/*
 * Internal state of the DiMAGE Scan Dual IV driver.
 *
 * The scanner-control logic is a reimplementation of the "AVI" layer that is
 * statically linked into MFSBaseLib2891 (Konica Minolta's port of the Avision
 * scanner library).  Comments of the form [aa9e] give the address of the
 * corresponding global (0x1002xxxx) in the original PowerPC library so that
 * every field can be traced back to the disassembly; see docs/PROTOCOL.md.
 */
#ifndef DSD4_INTERNAL_H
#define DSD4_INTERNAL_H

#include "dsd4.h"
#include "transport.h"
#include <stdint.h>
#include <stdio.h>

/* ---- byte order helpers: the scanner mixes big- and little-endian fields */
static inline uint16_t be16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }
static inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[1] << 8 | p[0]); }
static inline uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static inline void put_be16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = (uint8_t)v; }
static inline void put_le16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = v >> 8; }
static inline void put_be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (uint8_t)v; }
static inline void put_le32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

/* ---- SCSI-style opcodes (CmdXxx in the AVI layer) */
enum {
    OP_TEST_UNIT_READY = 0x00,
    OP_REQUEST_SENSE   = 0x03,
    OP_INQUIRY         = 0x12,
    OP_RESERVE_UNIT    = 0x16,
    OP_RELEASE_UNIT    = 0x17,
    OP_SCAN            = 0x1b,
    OP_SET_WINDOW      = 0x24,
    OP_READ            = 0x28,
    OP_SEND            = 0x2a,
    OP_OBJECT_POSITION = 0x31,
};

/* ---- READ / SEND data type codes */
enum {
    DT_IMAGE           = 0x00,
    DT_CALIB_FORMAT    = 0x60, /* READ 32 bytes */
    DT_CALIB_GRAY      = 0x61, /* READ white shading data, gray */
    DT_CALIB_COLOR     = 0x62, /* READ white shading data, color */
    DT_CALIB_DARK      = 0x66, /* READ dark shading data */
    DT_NVM             = 0x69, /* READ 128 bytes */
    DT_ACCEL_TABLE     = 0x6c, /* READ 24 byte request / SEND table */
    DT_RAW_INFO        = 0x70, /* READ 32 bytes */
    DT_RAW_LINE        = 0x71,
    DT_GAMMA           = 0x81, /* SEND 512 bytes, qualifier = channel */
    DT_SHADING         = 0x82, /* SEND, qualifier 0-2 white RGB, 3-5 dark RGB */
    DT_HOLDER          = 0x87, /* READ/SEND 8 bytes */
    DT_FOCUS           = 0x88, /* SEND 8 bytes */
    DT_FIRMWARE        = 0x90, /* READ 34 bytes status / SEND image */
    DT_RESET           = 0x92, /* SEND 8 bytes */
    DT_LIGHT_STATUS    = 0xa0, /* READ 1 byte */
};

/* ---- error codes kept in [aa6c] (GetScannerStatusEx) */
enum {
    AVE_OK           = 0x00,
    AVE_LAMP         = 0x73,
    AVE_GENERIC      = 0x7b,
    AVE_NOMEM        = 0x7f,
    AVE_PARAM        = 0x97,
    AVE_ALLOC        = 0x98,
    AVE_SENSE_FAILED = 0x9a,
    AVE_NOT_SCANNING = 0x9f,
    AVE_BUSY         = 0xa2,
    AVE_FW_FILE      = 0xa6,
    AVE_FW_READ      = 0xfa,
};

/* The 0x60 byte scan parameter block passed to SetScanParameter.  It is a
 * host-side structure in the original library; layout documented in
 * docs/PROTOCOL.md ("AV scan parameter block"). */
typedef struct {
    uint16_t ulx, uly;          /* 0x00 0x02: origin (pixels at xres/yres) */
    uint16_t width, length;     /* 0x04 0x06: extent in base units or pixels */
    uint16_t pixels, lines;     /* 0x08 0x0a: requested pixels/lines */
    uint8_t  mode;              /* 0x0c: 0 lineart 1 halftone 2 gray 4 color */
    uint8_t  b0d;               /* 0x0d */
    uint8_t  bits;              /* 0x0e: bits per pixel (8,16,24,48) */
    uint8_t  b0f;               /* 0x0f */
    int8_t   contrast;          /* 0x10 */
    int8_t   brightness;        /* 0x11 */
    uint8_t  halftone;          /* 0x12 */
    uint8_t  highlight;         /* 0x13 */
    uint8_t  shadow;            /* 0x14 */
    uint8_t  gray_filter;       /* 0x15 */
    uint8_t  invert;            /* 0x16 */
    uint16_t flags;             /* 0x18 */
    uint16_t exp_r, exp_g, exp_b; /* 0x1a..0x1f: exposure in percent */
    uint16_t xres, yres;        /* 0x20 0x22 */
    int16_t  focus;             /* 0x2a */
    uint16_t multi_lines;       /* 0x2c */
    uint8_t  multi_scan;        /* 0x2e */
    uint8_t  multi_sample;      /* 0x2f */
    int32_t  is_aps;            /* 0x30 */
    int32_t  film_flag;         /* 0x34: film type 6 or 10 */
    int32_t  focus_mode;        /* 0x38 */
    int32_t  index_not_last;    /* 0x3c */
    int32_t  final_scan;        /* 0x40 */
    int32_t  index_last;        /* 0x44 */
    int32_t  nonstd_dark;       /* 0x48 */
    uint8_t  exp_ir;            /* 0x4c */
} sp_t;

/* flags in sp_t.flags */
enum {
    SPF_IR          = 0x0001,
    SPF_MULTI_SCAN  = 0x0002,
    SPF_0004        = 0x0004,
    SPF_FOCUS       = 0x0008,
    SPF_0010        = 0x0010,
    SPF_MULTISAMPLE = 0x0020,
    SPF_BASE2400    = 0x0040,
    SPF_0080        = 0x0080,
    SPF_0100        = 0x0100,
    SPF_0800        = 0x0800,
    SPF_PIXELMODE   = 0x1000, /* do not derive resolution from extent */
    SPF_4000        = 0x4000,
    SPF_EXTENDED    = 0x8000,
};

/* Host-side copy of GetScannerAbilityEx output (0x8e bytes at [aac4]). */
typedef struct {
    char     vendor[20], product[12];
    char     revision[5];
    uint32_t flags;      /* +0x44 [ab08] */
    uint32_t flags2;     /* +0x48 */
    uint16_t optical;    /* +0x58 [ab1c] */
    uint16_t max_x, max_y, max_x2, max_y2; /* +0x5a..0x60 */
    uint16_t f62;        /* +0x62 */
    uint16_t f64;        /* +0x64 [ab28] */
    uint8_t  f68, f69, f6a, f6b;
    uint8_t  align_lineart, align_gray, align_color; /* +0x6c..0x6e [ab30..32] */
    uint8_t  f6f, f70, f71;
    uint32_t f76;
    uint16_t f7a, f7c;
    uint16_t shading_target; /* +0x7e [ab42] */
    uint16_t max_pixels, max_lines; /* inquiry bytes 81-84 */
} ability_t;

#define SHD_W 3200 /* one shading table channel: 0x1900 bytes of uint16 */

struct dsd4 {
    dsd4_transport *t;
    dsd4_log_fn log;
    void *log_ud;
    int verbose;
    int cmd_delay_us;

    /* AV library state */
    int last_error;        /* [aa6c] */
    int need_release;      /* [10028568] RELEASE UNIT must be retried */
    int busy;              /* [10028584] */
    int scanning;          /* [100285bc] */
    int read_failed;       /* [100285b4] */
    int calibrating;       /* [10028598] */
    int init_calibration;  /* [10028594] */
    int first_init;        /* [aa68] */
    uint32_t caps;         /* [aa78] */
    int ability_valid;     /* [aa80] */
    int flag4000;          /* [aa84] */
    int speed_mode;        /* [aa88] */
    int lineart;           /* [aa8c] */
    int is16;              /* [aa90] scanner sends 16 bit little-endian samples */
    int ir_mode;           /* [aa94] */
    int cal_mode;          /* [aa98] 1 or 3 (from INQUIRY byte 91) */
    uint16_t image_mode;   /* [aa9a] */
    uint16_t b0d;          /* [aa9c] */
    uint16_t line_bytes;   /* [aa9e] bytes per output line */
    uint16_t scan_line_bytes; /* [aaa0] bytes per scanner line */
    uint16_t aaa2;         /* [aaa2] */
    uint16_t req_pixels;   /* [aaa4] */
    uint16_t req_lines;    /* [aaa6] */
    uint16_t scan_pixels;  /* [aaa8] */
    uint16_t scan_lines;   /* [aaaa] */
    uint16_t saved_line_bytes, saved_scan_line_bytes; /* [aaac] [aaae] */
    int multisample;       /* [aab0] */
    int load_cache;        /* [aab6] */
    uint16_t xsub;         /* [aaba] 3200 / y resolution */
    uint16_t ulx3200;      /* [aabc] */
    uint16_t white_target[4]; /* [aabe..aac4] */
    ability_t ab;          /* [aac4] */
    uint32_t raw_remaining;   /* [aa70] bytes still to READ from the scanner */
    uint32_t out_remaining;   /* [aa74] bytes still to hand to the caller */
    uint16_t ring_count;      /* [b1a2] */
    uint16_t line_distance;   /* [b1a4] */
    uint32_t chunk_bytes;     /* [b1c2] bytes delivered per ReadScan */
    uint32_t leftover;        /* [b1c6] bytes already unpacked but not delivered */
    uint8_t *leftover_ptr;    /* [b1b2] */
    uint32_t pack_size;       /* [b1ce] */
    uint32_t b1d2;            /* [b1d2] */
    uint32_t y_line;          /* [b1d8] */
    int scanner_type;         /* [b1ee] INQUIRY byte 91 */
    int mirror;               /* [b1fc] */
    uint16_t b200, b202;      /* line shift */
    uint16_t optical_res;     /* [b20a] */
    uint8_t yscale;           /* [b20c] 0 native, 1 accumulate, 2 upscale */
    uint8_t xscale;           /* [b20d] */
    uint32_t delivered;       /* [b21a] */
    int one_line;             /* [b21e] */
    int gray_from_color;      /* [b222] */
    int focus_flag;           /* [b226] */
    uint8_t state;            /* [b22a] */
    double ae_override[3];    /* [b22b] [b233] [b23b] */
    uint16_t focus_param;     /* [b24b] */
    int multi_scan;           /* [b24d] */
    uint32_t *accum;          /* [b254] 0x40000 byte accumulator */
    uint32_t ystep, yacc, yden; /* [b258] [b25c] [b260] */
    uint32_t cal_count;       /* [b264] pixels*3 of the calibration data */
    uint16_t base_unit;       /* [a200] 300 or 2400 */
    int16_t  last_focus;      /* [a202] */

    uint8_t *pack_buf;        /* [b1ba] */

    /* shading tables, 3 channels x 3200 uint16 each */
    uint16_t dark_raw[3][SHD_W];    /* [1002b268] */
    uint16_t white_raw[3][SHD_W];   /* [1002fd68] */
    uint16_t dark[3][SHD_W];        /* [10034868] */
    uint16_t white[3][SHD_W];       /* PTR_DAT_1002856c */
    uint16_t dark_nonstd[3][SHD_W]; /* PTR_DAT_100285a0 */
    int have_shading;

    /* acceleration table work area (PTR_DAT_100285c8..d8) */
    uint8_t  acc_table[1000];
    uint32_t acc_a, acc_b;
    uint16_t acc_start, acc_end;

    /* CBaseScannerAVI / high level state */
    uint8_t  base_state;      /* +0x2a9 */
    int      setup_done;      /* +0x29c */
    uint16_t base_exp[3];     /* setup data: base exposure R,G,B */
    uint16_t max_exp, min_exp;/* setup data +0xdac8, +0xdaca */
    int      holder_type;     /* last READ 0x87 byte 0 */
    int      frame_count;
    int      carriage_out;    /* +0x2a4 */
    int      firmware_checked;
    uint8_t  inquiry[96];
    char     firmware_path[1024];

    /* research hooks (src/research.c) */
    uint16_t probe_yres;      /* != 0: send this vertical resolution unchanged */
    float *dbg_var;           /* != NULL: per-sample variance of multi-sample copies */
    float *dbg_dev;           /* != NULL: first copy minus the mean (sensor noise, signed) */

    /* experimental processing inside dsd4_scan_minolta, before the film curves */
    const dsd4_enhance_params *enhance;
    dsd4_enhance_info enhance_info;

    /* Minolta preview: AE result of the last previewed frame */
    struct {
        int valid, frame, film, holder;
        double gain[3];
        int black[3];
        uint16_t *tab;        /* 3 x 65536 final film curves */
    } ae_cache;
};

/* avision.c */
int  av_command(dsd4 *s, const uint8_t *cdb, int cdblen, enum dsd4_dir dir, void *data, uint32_t len);
int  av_test_unit_ready(dsd4 *s);
int  av_inquiry(dsd4 *s, uint8_t buf[96]);
int  av_reserve(dsd4 *s);
int  av_release(dsd4 *s);
int  av_scan(dsd4 *s, uint8_t flags);
int  av_set_window(dsd4 *s, uint32_t len, const uint8_t *buf);
int  av_read(dsd4 *s, uint8_t dtc, uint32_t len, void *buf);
int  av_send(dsd4 *s, uint8_t dtc, uint16_t qual, uint32_t len, const void *buf);
int  av_object_position(dsd4 *s, uint8_t op);
int  av_is_ready(dsd4 *s, unsigned ticks);
void av_delay_ticks(unsigned ticks);
void dsd4_logf(dsd4 *s, int level, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

/* avlib.c */
int  avl_initialize_driver(dsd4 *s);
int  avl_get_scanner_ability(dsd4 *s);
int  avl_get_calibration_format(dsd4 *s, uint8_t fmt[32]);
int  avl_reset_to_initial_state(dsd4 *s, uint8_t mode);
int  avl_check_scanner_state(dsd4 *s, int what, uint8_t *state);
int  avl_find_scanner(dsd4 *s);
int  avl_initialize_scanner(dsd4 *s);
int  avl_do_calibration(dsd4 *s);
int  avl_dark_calibration(dsd4 *s, const double exp[3]);
int  avl_set_scan_parameter(dsd4 *s, const sp_t *p);
int  avl_start_scan(dsd4 *s);
int  avl_read_scan(dsd4 *s, void *dst, uint32_t len);
int  avl_stop_scan(dsd4 *s);
int  avl_paper_feed(dsd4 *s);
int  avl_read_holder(dsd4 *s, uint8_t info[8]);
int  avl_send_holder_control(dsd4 *s, uint8_t frame);
int  avl_set_optical_control(dsd4 *s, int16_t pos);
int  avl_set_gamma_table(dsd4 *s, const uint8_t *table, uint8_t channel, uint32_t size);
int  avl_download_firmware(dsd4 *s, const char *path, int *downloaded);
void avl_temp_scan_parameter(dsd4 *s, sp_t *p);
uint16_t avl_get_acc_table(dsd4 *s, const uint8_t req[24]); /* returns table length */

/* scanner.c */
uint8_t *dsd4_resample16(const uint16_t *src, int sw, int sh, int ch, int dw, int dh);

/* merge.c: mean of n samples (stride apart), rejecting outliers from n >= 4 */
double dsd4_robust_mean16(const uint16_t *v, unsigned n, size_t stride);

/* calib.c */
uint16_t cal_bubble_average(uint16_t *v, unsigned n);
uint16_t *cal_average(const uint8_t fmt[32], uint32_t count, const uint8_t *raw);
void cal_dark_shading(dsd4 *s, const uint8_t fmt[32], uint32_t count, uint16_t *dark);
void cal_white_shading(dsd4 *s, const uint8_t fmt[32], uint32_t count, uint16_t *white);
int  cal_dark_smooth(uint16_t *dark, uint32_t count);

/* mtf.c: recompute MTF50 / Nyquist / sigma of direction t */
void dsd4i_mtf_stats(dsd4_mtf_profile *pr, int t);

/* profiles.c: a typical Dual IV (profile file text) */
extern const char dsd4i_default_mtf_1x[], dsd4i_default_mtf_multi[], dsd4i_default_noise[];

#endif
