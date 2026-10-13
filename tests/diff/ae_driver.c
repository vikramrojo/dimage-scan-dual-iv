/*
 * Driver for the AE kernel differential test (tests/diff/ae_diff.py).
 *
 *   clang -std=c11 -O2 -ffp-contract=off -I../../src ae_driver.c ../../src/minolta_ae.c -o ae_driver
 *   ae_driver case.bin out.bin
 *
 * case.bin (little-endian u32 unless noted):
 *   film, product, per_channel_pos, area_x, area_y, area_w, area_h
 *   9 planes (image[3], ref_dark[3], ref_light[3]), each:
 *       buffer (0..2, or 0xffffffff = NULL), offset, width, height, bytes,
 *       pixel_stride, row_stride
 *   0x70 bytes initial MFSIPAEResult (big-endian, PowerPC layout)
 *   3 buffers, each: u32 length, data (16 bit samples host little-endian)
 * out.bin: i32 return value, 0x70 bytes result (big-endian layout)
 */
#include "minolta_ae.h"

#include <stdio.h>
#include <stdlib.h>

static uint32_t rd(FILE *f)
{
    uint8_t b[4];
    if (fread(b, 1, 4, f) != 4) { fprintf(stderr, "short case file\n"); exit(2); }
    return b[0] | b[1] << 8 | b[2] << 16 | (uint32_t)b[3] << 24;
}

int main(int argc, char **argv)
{
    if (argc != 3) { fprintf(stderr, "usage: ae_driver case.bin out.bin\n"); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    mfs_ae_input in;
    in.film_type = rd(f);
    in.product = rd(f);
    in.per_channel_pos = (uint8_t)rd(f);
    in.area_x = rd(f);
    in.area_y = rd(f);
    in.area_w = rd(f);
    in.area_h = rd(f);
    uint32_t pd[9][7];
    for (int i = 0; i < 9; i++)
        for (int j = 0; j < 7; j++) pd[i][j] = rd(f);
    uint8_t raw[0x70];
    if (fread(raw, 1, sizeof raw, f) != sizeof raw) return 2;
    uint8_t *buf[3];
    for (int i = 0; i < 3; i++) {
        uint32_t n = rd(f);
        buf[i] = malloc(n ? n : 1);
        if (fread(buf[i], 1, n, f) != n) return 2;
    }
    fclose(f);
    mfs_ae_plane *pl[9];
    for (int i = 0; i < 3; i++) { pl[i] = &in.image[i]; pl[3 + i] = &in.ref_dark[i]; pl[6 + i] = &in.ref_light[i]; }
    for (int i = 0; i < 9; i++) {
        pl[i]->data = pd[i][0] == 0xffffffffu ? NULL : buf[pd[i][0]] + pd[i][1];
        pl[i]->width = pd[i][2];
        pl[i]->height = pd[i][3];
        pl[i]->bytes = (uint8_t)pd[i][4];
        pl[i]->pixel_stride = pd[i][5];
        pl[i]->row_stride = pd[i][6];
    }
    mfs_ae_result res;
    mfs_ae_result_from_be(raw, &res);
    int32_t rc = mfs_ae_analyze(&in, &res);
    mfs_ae_result_to_be(&res, raw);
    FILE *o = fopen(argv[2], "wb");
    if (!o) { perror(argv[2]); return 2; }
    uint8_t r4[4] = { (uint8_t)rc, (uint8_t)(rc >> 8), (uint8_t)(rc >> 16), (uint8_t)(rc >> 24) };
    fwrite(r4, 1, 4, o);
    fwrite(raw, 1, sizeof raw, o);
    fclose(o);
    return 0;
}
