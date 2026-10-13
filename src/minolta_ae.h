/*
 * Port of the Minolta auto-exposure (AE) analysis kernel of MFSLib2891.
 *
 * AutoExposure (Ghidra 0x100078f0) analyses the 800 dpi prescan and calls
 * FUN_10036090(&input, result).  That function and its whole call tree are
 * ported bit-exactly in minolta_ae.c; mfs_ae_analyze() is FUN_10036090.
 *
 * How the original builds the input (AutoExposure / GetAEInputData
 * 0x100085e0, MFSFormedImageInfo = 3 plane descriptors of 6 words):
 *
 *  - film_type = film setting & ~0x10000.  Bits: 4 = colour, 2 = negative,
 *    1 = positive, 8 = monochrome.  The kernel handles 5 (colour positive),
 *    6 (colour negative) and 10 (B&W negative); everything else (9, 0x100,
 *    0x400, ...) returns -1.
 *  - image (input +0x04): GetAEInputData(buffer +0x6c, diffuser type 2/3/4):
 *    the AE prescan, 16 bit samples (bytes = 2), pixel interleaved.  Colour
 *    (type 2): RGB, 3 channels; dice/ICE (type 3): RGBI, 4 channels; mono
 *    (type 4): 1 channel, only plane 0 filled.  plane[c].data = buffer + 2c,
 *    pixel_stride = 2 * nch, row_stride = 2 * nch * width, width/height =
 *    the AE image size (image ability), i.e. the whole 800 dpi frame.
 *  - ref_dark (input +0x4c): GetAEInputData(buffer +0x60, type 1): one line
 *    (height 1, row_stride 0) of interleaved 16 bit RGB with the scanner's
 *    channel count as pixel stride (2 * nch); averaged per channel and
 *    scaled to 10 bit (FUN_1003ad40) -> black level.  Mono: one plane, the
 *    green sample (data = buffer + 2).
 *  - ref_light (input +0x94): GetAEInputData(buffer +0x54, type 0): same
 *    one-line layout; FUN_1003ab30 divides its maximum by the 8 sample
 *    average at the position of the brightest image block (light profile).
 *  - area: AE rectangle in image pixels (GetFullAEArea / CheckAEArea);
 *    must be >= 32x32 and inside the image.
 *  - per_channel_pos: 0 in AutoExposure; 1 on the "positive, AE area" path
 *    (per channel gains for slides, otherwise one gain from the strongest
 *    channel).
 *  - product: GetProductType (FUN_1003eff0 accepts 3, 4, 8, 9, 11, 12).
 *
 * The 16 bit samples are read natively (host byte order) by the port; the
 * original reads big-endian memory.  8 bit planes (bytes == 1) are handled
 * as in the original although the driver never produces them.
 */
#ifndef MINOLTA_AE_H
#define MINOLTA_AE_H

#include <stdint.h>

/* MFSFormedImageInfo entry (24 bytes in the original) */
typedef struct {
    const uint8_t *data;   /* +0x00 first sample of this channel */
    uint32_t width;        /* +0x04 pixels */
    uint32_t height;       /* +0x08 lines */
    uint8_t bytes;         /* +0x0c bytes per sample: 1 = 8 bit, else 16 bit */
    uint32_t pixel_stride; /* +0x10 bytes between horizontally adjacent pixels */
    uint32_t row_stride;   /* +0x14 bytes between lines */
} mfs_ae_plane;

/* input of FUN_10036090 (0xf4 bytes in the original) */
typedef struct {
    uint32_t film_type;        /* +0x00 */
    mfs_ae_plane image[3];     /* +0x04 R, G, B prescan planes */
    mfs_ae_plane ref_dark[3];  /* +0x4c black reference line (diffuser 1) */
    mfs_ae_plane ref_light[3]; /* +0x94 light reference line (diffuser 0) */
    uint32_t area_x, area_y;   /* +0xdc AE area origin */
    uint32_t area_w, area_h;   /* +0xe4 AE area size */
    uint8_t per_channel_pos;   /* +0xec */
    uint32_t product;          /* +0xf0 GetProductType */
} mfs_ae_input;

/* tone parameters per channel, 0x14 bytes at result +0x30 + 0x14 * c.
 * Only set for colour negatives (film type 6). */
typedef struct {
    uint16_t level;  /* +0x00 base density level (8 bit histogram scale) */
    uint16_t k46;    /* +0x02 always 0x46 (70) */
    uint16_t lo;     /* +0x04 shadow point: G low point (+ R/B shift) */
    uint16_t hi;     /* +0x06 0xff, 0xf0, or 0 (AE area not the full frame) */
    int16_t shift;   /* +0x08 R/B cast correction (-35..35), 0 for G */
    uint8_t mode;    /* +0x0a 0, 1, 3 or 4 */
    uint8_t pad0b;   /* +0x0b padding (preserved) */
    double weight;   /* +0x0c contrast confidence (0..1), 4-byte aligned */
} mfs_ae_tone;

/* MFSIPAEResult (0x70 bytes, PowerPC layout) */
typedef struct {
    double gain[3];         /* +0x00 exposure gain R, G, B */
    struct {
        uint16_t exposure;  /* +0x18 +8c: type 10: exposure level (<= 895) */
        uint16_t dark;      /* +0x1a +8c: black level (10 bit) */
        uint16_t r1c, r1e;  /* +0x1c/+0x1e: always 0 */
    } ch[3];
    mfs_ae_tone tone[3];    /* +0x30 */
    uint8_t pad6c[4];       /* +0x6c padding (preserved) */
} mfs_ae_result;

/* FUN_10036090: 0 on success, -1 on bad input / unsupported film type.
 * Fields not written by the original keep their previous value. */
int mfs_ae_analyze(const mfs_ae_input *in, mfs_ae_result *out);

void mfs_ae_result_from_be(const uint8_t raw[0x70], mfs_ae_result *out);
void mfs_ae_result_to_be(const mfs_ae_result *in, uint8_t raw[0x70]);

#endif
