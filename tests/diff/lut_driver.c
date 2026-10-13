/*
 * Native side of tests/diff/lut_diff.py: runs one call of the LUT kernel
 * port (src/minolta_lut.c) described on stdin and writes the results to
 * stdout.  All integers little-endian u32; tables are passed and returned
 * big-endian (as in the emulated memory).
 *
 *   op 1  film curve:   state.ratio(f64 bits u32 hi,lo) state.ref,
 *                       39 parameter words of FUN_10036f40 (word 38 ignored),
 *                       3 x (in_code, out_code)
 *         -> ret, state.ratio hi,lo, state.ref, 3 tables
 *   op 2  user curve:   film (in,out,table), has_user, [user (in,out,table)],
 *                       dst (in,out)
 *         -> ret, stretch hi,lo, dst table
 *   op 3  convert:      src (in,out,table), dst (in,out) -> ret, dst table
 *
 * Build: clang -O2 -std=c11 -ffp-contract=off -Isrc \
 *            tests/diff/lut_driver.c src/minolta_lut.c -o lut_driver
 */
#include "minolta_lut.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t rd32(void)
{
    uint8_t b[4];
    if (fread(b, 1, 4, stdin) != 4) { fprintf(stderr, "short input\n"); exit(2); }
    return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24;
}

static void wr32(uint32_t v)
{
    uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    fwrite(b, 1, 4, stdout);
}

static void wrd(double d)
{
    uint64_t u;
    memcpy(&u, &d, 8);
    wr32((uint32_t)(u >> 32));
    wr32((uint32_t)u);
}

static size_t tbl_bytes(uint32_t in, uint32_t out)
{
    return ((size_t)mfs_lut_max[in] + 1) * (out ? 2 : 1);
}

/* read a big-endian table into native memory */
static void *rd_table(uint32_t in, uint32_t out)
{
    size_t n = tbl_bytes(in, out);
    uint8_t *raw = malloc(n);
    if (fread(raw, 1, n, stdin) != n) { fprintf(stderr, "short table\n"); exit(2); }
    if (!out) return raw;
    uint16_t *t = malloc(n);
    for (size_t i = 0; i < n / 2; i++) t[i] = (uint16_t)(raw[2 * i] << 8 | raw[2 * i + 1]);
    free(raw);
    return t;
}

static void wr_table(const mfs_lut_desc *d)
{
    size_t n = mfs_lut_max[d->in_code] + 1u;
    if (!d->out_code) { fwrite(d->table, 1, n, stdout); return; }
    const uint16_t *t = d->table;
    for (size_t i = 0; i < n; i++) { putchar(t[i] >> 8); putchar(t[i] & 0xff); }
}

static mfs_lut_desc new_desc(uint32_t in, uint32_t out)
{
    mfs_lut_desc d = { calloc(1, tbl_bytes(in, out)), in, out };
    return d;
}

int main(void)
{
    uint32_t op = rd32();
    if (op == 1) {
        mfs_lut_state st;
        uint64_t u = (uint64_t)rd32() << 32;
        u |= rd32();
        memcpy(&st.ratio, &u, 8);
        st.ref = rd32();
        uint32_t w[39];
        for (int i = 0; i < 39; i++) w[i] = rd32();
        mfs_lut_params p;
        memset(&p, 0, sizeof p);
        p.film_type = (int32_t)w[0];
        p.fixed_curve = (uint8_t)(w[1] >> 24);
        for (int c = 0; c < 3; c++) {
            u = (uint64_t)w[2 + 2 * c] << 32 | w[3 + 2 * c];
            memcpy(&p.gain[c], &u, 8);
        }
        p.product_type = (int32_t)w[8];
        uint8_t ae[0x70];
        for (int i = 0; i < 28; i++) {
            ae[4 * i] = (uint8_t)(w[10 + i] >> 24);
            ae[4 * i + 1] = (uint8_t)(w[10 + i] >> 16);
            ae[4 * i + 2] = (uint8_t)(w[10 + i] >> 8);
            ae[4 * i + 3] = (uint8_t)w[10 + i];
        }
        mfs_lut_ae_from_be(&p.ae, ae);
        mfs_lut_desc lut[3];
        for (int c = 0; c < 3; c++) {
            uint32_t in = rd32(), out = rd32();
            lut[c] = new_desc(in, out);
        }
        int r = mfs_lut_film_curve(&p, lut, &st);
        wr32((uint32_t)r);
        wrd(st.ratio);
        wr32(st.ref);
        for (int c = 0; c < 3; c++) wr_table(&lut[c]);
    } else if (op == 2) {
        mfs_lut_desc film, user = { 0 }, dst;
        film.in_code = rd32();
        film.out_code = rd32();
        film.table = rd_table(film.in_code, film.out_code);
        if (rd32()) {
            user.in_code = rd32();
            user.out_code = rd32();
            user.table = rd_table(user.in_code, user.out_code);
        }
        uint32_t in = rd32(), out = rd32();
        dst = new_desc(in, out);
        double k = -1.0;
        int r = mfs_lut_apply_user_curve(&film, &user, &dst, &k);
        wr32((uint32_t)r);
        wrd(k);
        wr_table(&dst);
    } else if (op == 3) {
        mfs_lut_desc src, dst;
        src.in_code = rd32();
        src.out_code = rd32();
        src.table = rd_table(src.in_code, src.out_code);
        uint32_t in = rd32(), out = rd32();
        dst = new_desc(in, out);
        int r = mfs_lut_convert(&src, &dst);
        wr32((uint32_t)r);
        wr_table(&dst);
    } else {
        return 2;
    }
    return 0;
}
