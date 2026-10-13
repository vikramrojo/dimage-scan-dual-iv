/*
 * Tone-curve / LUT kernel of MFSLib2891 (DiMAGE Scan Dual IV), ported to C.
 *
 * Bit exact port of the original PowerPC functions
 *
 *   FUN_10036f40  film curve dispatcher       -> mfs_lut_film_curve()
 *   FUN_10037280  film curve x user curve     -> mfs_lut_apply_user_curve()
 *   FUN_10037d70  LUT resampling / conversion -> mfs_lut_convert()
 *
 * and everything they call (spline fit/eval, the per film type curve
 * generators, ...).  Function comments give the original address.  The
 * differential test is tests/diff/lut_diff.py.
 *
 * ======================================================================
 * LUT descriptors (MFSLutInfo)
 * ======================================================================
 * A LUT is described by three words {table, in_code, out_code}.  The two
 * codes index the "maximum value" table at 0x1460e0:
 *
 *     code   0     1      2      3       4
 *     max    255   1023   4095   16383   65535
 *
 * in_code gives the number of entries (max[in_code] + 1, i.e. the input is a
 * raw in-bit sample), out_code the output range (0 .. max[out_code]).  When
 * out_code == 0 the table holds bytes, otherwise uint16_t.  The film LUTs
 * built for a scan are {65536 x u16, 4, 4}; the final LUT is {.., 4, 4} for
 * 16-bit output and {.., 4, 0} for 8-bit output.
 *
 * MFSLutInfo as passed to LutAndGainSetting is an array of 6 descriptors:
 * [0..2] the final per channel LUT tables, [3..5] work tables that receive
 * the film curves.
 *
 * ApplyLUT (0x1000b460, args: pixels, u16 *lut[3], npixels, channels (1|3),
 * to8bit) applies the final LUTs in place to interleaved 16-bit samples:
 * p[c] = lut[c][p[c]], i.e. the raw 16-bit sample value is the index.  With
 * to8bit set the result is written packed as one byte per sample
 * (lut[c][p[c]] >> 8).  Gray data uses lut[0] only.
 *
 * ======================================================================
 * How the original drives the kernel
 * ======================================================================
 * LutAndGainSetting (0x100093d0, scanner, MFSActionSetting *act,
 * MFSLutInfo info[6], double gain_out[4], double base_out[4],
 * MFSLutInfo film[3], MFSLutInfo final[3], const double gain_in[3]):
 *   - for the normal colour / B&W path (setting.flags bit 2 set and film type
 *     != 0x400) it sets film[c] = {info[3+c].table, 4, 4} and
 *     final[c] = {info[c].table, 4, Is8BitScan ? 0 : 4} and calls CalLUT;
 *     the scanner exposure is then gain_out[c] = gain_in[c] * stretch[c]
 *     where stretch[] is CalLUT's double output (see apply_user_curve).
 *   - otherwise (raw / linear / "drag" colour) it writes a plain linear
 *     LUT  lut[i] = d2u(i * std*gain/ref + 0.5)  and no kernel is used.
 *
 * CalLUT (0x10009950, scanner, act, film[3], final[3], double stretch[3],
 * MFSPrefData *pref).  s = act->settings:
 *     s+0x00 holder, s+0x04 film type, s+0x08 u16 frame/slot, s+0x28..0x48
 *     five doubles (user exposure adjustments, -> GetAdjustGain),
 *     s+0x50 flags (bit0 = use AE result, bit2, 0x10 MCM, 0x40 ROC,
 *     0x80 GEM), s+0x58/0x59 bytes (saved-AE selectors), s+0x78 + 12*c the
 *     user tone curve descriptor per channel (table == NULL: none),
 *     s+0xac MCM parameters.
 *   1. GetFilmSettingArrayNo(holder, type) -> (holder_no, type_no).
 *   2. AE record (MFSIPAEResult, 0x70 bytes) is copied from the preferences:
 *        flags bit0 && s+0x58 == 1:
 *              pref + 0x16da8 + holder_no*0x690 + type_no*0x70
 *        else  pref + 0x80 + type_no*0x1858 + (frame&0xff)*0x98
 *      These are the AE results saved by the last AE run for this film
 *      setting (UseSavedAEResult reads the same slots).  The 0x98-byte slot
 *      is {AE result 0x70, ?, double gain[3] at +0x80 (used by
 *      LutAndGainSetting for base_out), MFSArea at +0x70 for IsSameAEArea}.
 *      Compiled-in defaults: none.  GetPrefData reads the whole MFSPrefData
 *      (0x30358 bytes) from the preferences file; if there is none,
 *      InitPrefData zero-fills it (every memset in Init/ResetPrefData stores
 *      byte 0x00; the "0x8000" fills truncate to 0).  So until an AE run has
 *      stored a result for the film setting / frame, the kernel sees an
 *      all-zero AE result: colour negatives with AE then give a constant
 *      0xffff LUT (NaN from 0/0, d2u(NaN) = 0xffffffff, clamped).
 *   3. adj[3] = GetAdjustGain(type, s+0x28.., flags&1) (starts at 1,1,1);
 *      std[3] = GetStdGain(holder, type, s+8, IsMcmOn).
 *   4. kernel parameters (mfs_lut_params): film_type = type & ~0x10000
 *      (glass bit removed), fixed_curve = !(flags & 1), gain[c] =
 *      std[c]*adj[c] when fixed_curve (else 0), product_type =
 *      GetProductType() (scanner table +0x30378, from MFSBaseGetScannerInfo),
 *      ae = the AE record of step 2, lut = film[].
 *   5. stretch[c] = 1.0.  Then, if digital ROC/GEM ("drag") is off:
 *        MCM off: mfs_lut_film_curve(); per channel: no user curve ->
 *                 mfs_lut_convert(film[c] -> final[c]); else
 *                 mfs_lut_apply_user_curve(film[c], user[c], final[c],
 *                 &stretch[c]); with exposure simulation (IsExpSimulation)
 *                 and stretch != 1 the stretch is instead applied digitally:
 *                 film[c] = final[c]; final[c][i] = film[c][d2u(i*stretch+0.5)]
 *                 (index >= 65536 -> film[c][65535]) and stretch[c] = 1.
 *        MCM on:  colour negatives still run mfs_lut_film_curve() (result
 *                 not composed); final[c] = mfs_lut_convert(user[c]) for
 *                 channels with a user curve.
 *      With ROC/GEM on the film curve (for positives only when MCM is off,
 *      always for negatives) is built into the final tables, then re-sampled
 *      linearly by std*adj/pref_gain (pref + 0x24e00 + holder_no*0x11f0 +
 *      frame*0x70) and composed with the user curve.
 *
 * Per film type (low bits of the MFS film type; 0x10000 = glass mount):
 *     5      colour positive   20-point S curve (FUN_10039260)
 *     9      B&W positive      same as 5
 *     6      colour negative   fixed_curve: 8-point inversion curve with
 *                              product-type channel gains (FUN_10039d20);
 *                              AE: density/gamma model from ae.nega[]
 *                              (FUN_10040990), channel 1 (G) first, its
 *                              level is the reference for R and B
 *     10     B&W negative      fixed_curve: 8-point inversion (FUN_1003a1c0);
 *                              AE: 5-point curve from ae.level[] (FUN_10039530)
 *     0x100  colour linear     straight line (FUN_10039950)
 *     0x200  B&W linear inv.   inverted straight line (FUN_10039b30)
 *     other                    error (-1)
 *
 * Product type: GetProductType() for the Dual IV is 0xb or 0xc (both treated
 * identically by every kernel here; firmware is only downloaded for 10/0xc).
 * Note: FUN_10041340 (used by the colour negative AE curve) only defines its
 * gamma for product types 3,4 (0.7) and 8,9,0xb,0xc (0.69); for any other
 * product type the original reads uninitialised stack.  This port returns
 * -1 from mfs_lut_film_curve() in that case.
 */
#ifndef MINOLTA_LUT_H
#define MINOLTA_LUT_H

#include <stddef.h>
#include <stdint.h>

/* film type codes (MFSFilmSetting.type) handled by the kernel */
enum {
    MFS_FILM_COLOR_POSI   = 5,
    MFS_FILM_COLOR_NEGA   = 6,
    MFS_FILM_BW_POSI      = 9,
    MFS_FILM_BW_NEGA      = 10,
    MFS_FILM_COLOR_LINEAR = 0x100,
    MFS_FILM_BW_LINEAR    = 0x200,
};

/* max value per bit-depth code (table at 0x1460e0) */
extern const uint16_t mfs_lut_max[8];

/* MFSLutInfo entry. table is uint8_t[] when out_code == 0, else uint16_t[] */
typedef struct {
    void    *table;
    uint32_t in_code;   /* number of entries = mfs_lut_max[in_code] + 1 */
    uint32_t out_code;  /* values 0 .. mfs_lut_max[out_code] */
} mfs_lut_desc;

/* Per channel colour negative parameters, MFSIPAEResult + 0x30 + 0x14*c.
 * Names describe how FUN_10040990 uses them. */
typedef struct {
    uint16_t contrast;     /* +0x00 slope of the density curve, clamped [10,100] */
    uint16_t base;         /* +0x02 film base level (0..255) */
    uint16_t ref_level;    /* +0x04 reference input level (0..255) */
    uint16_t ref_target;   /* +0x06 output target for the reference level (0..255) */
    int16_t  contrast_adj; /* +0x08 signed offset added to contrast for the base point */
    uint8_t  balance_mode; /* +0x0a 0: none, 1: mix with green, 4: lock to green, other: mix */
    uint8_t  pad_0b;       /* +0x0b */
    double   balance;      /* +0x0c mix factor for modes 1 / other */
} mfs_lut_ae_nega;

/* MFSIPAEResult (0x70 bytes, big-endian in the original); the fields this
 * kernel reads.  Same layout as mfs_ae_result in minolta_ae.h (ch[c] =
 * level[c], tone[c] = nega[c]); kept separate so both headers can be
 * included together. */
typedef struct {
    double   gain[3];        /* +0x00 exposure gains (AE kernel sets 1.0 first) */
    struct {
        uint16_t shadow;     /* +0x18+8c  B&W nega: lower knot (AE: exposure level <= 895; > 0x380 -> 0) */
        uint16_t highlight;  /* +0x1a+8c  B&W nega: upper knot = 1023 - this (AE: black level) */
        uint16_t u1c, u1e;   /* +0x1c/+0x1e not used by the LUT kernel */
    } level[3];
    mfs_lut_ae_nega nega[3];     /* +0x30 colour negative parameters */
    uint32_t pad_6c;         /* +0x6c */
} mfs_lut_ae;

/* Parameter block of FUN_10036f40 (passed by value in the original: word 0
 * in r3, the rest from r4 on and on the stack, see lut_diff.py). */
typedef struct {
    int32_t  film_type;    /* word 0  MFS film type, glass bit cleared */
    uint8_t  fixed_curve;  /* word 1  (byte 0) 1: AE not used -> fixed curves */
    double   gain[3];      /* words 2-7 std*adjust gain (not used by the kernel) */
    int32_t  product_type; /* word 8  GetProductType() */
    mfs_lut_ae ae;      /* words 10-37 (word 9 is padding) */
} mfs_lut_params;          /* word 38 = pointer to mfs_lut_desc[3] */

/* Statics of FUN_10040990 (globals at TOC-0x77f0 / TOC-0x77e8): the
 * green-channel reference used to balance R and B.  Persist across calls
 * in the original (initially zero). */
typedef struct {
    double   ratio;
    uint32_t ref;
} mfs_lut_state;

/* Parse a 0x70-byte big-endian MFSIPAEResult.  Minimal local parser, to be
 * replaced by the AE port's mfs_ae_result_from_be(). */
void mfs_lut_ae_from_be(mfs_lut_ae *ae, const uint8_t be[0x70]);

/* FUN_10036f40: build the three film LUTs for p->film_type.  st may be NULL
 * (a process wide state is used, like the original).  Returns 0 or -1. */
int mfs_lut_film_curve(const mfs_lut_params *p, const mfs_lut_desc lut[3],
                       mfs_lut_state *st);

/* FUN_10037280: final = user o film, re-fitted with a spline so that the
 * used input range is stretched by *stretch_out (1..10); the caller raises
 * the scanner exposure by that factor.  user->table may be NULL.
 * Returns 0 or -1. */
int mfs_lut_apply_user_curve(const mfs_lut_desc *film, const mfs_lut_desc *user,
                             const mfs_lut_desc *dst, double *stretch_out);

/* FUN_10037d70: resample / requantise src into dst.  Returns 0 or -1. */
int mfs_lut_convert(const mfs_lut_desc *src, const mfs_lut_desc *dst);

#endif
