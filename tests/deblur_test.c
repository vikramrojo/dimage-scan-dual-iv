/* Deblur on synthetic film: random-tone patches (edges) with multiplicative
 * grain, blurred by a known gaussian.  The measured blur must match, the
 * result must be closer to the sharp picture than the blurred one, and a
 * sharp picture must be left alone. */
#include "dsd4.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { W = 1536, H = 1536, B = 48 };
static uint64_t rng = 0x9e3779b97f4a7c15ull;
static double urand(void)
{
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    return ((rng >> 11) + 0.5) / 9007199254740992.0;
}
static double nrand(void) { return sqrt(-2 * log(urand())) * cos(6.283185307179586 * urand()); }

static void blur(const float *in, float *out, double s)
{
    int r = (int)ceil(3 * s);
    float *k = malloc((2 * r + 1) * sizeof *k), *t = malloc((size_t)W * H * sizeof *t);
    double sum = 0;
    for (int i = -r; i <= r; i++) sum += k[i + r] = (float)exp(-0.5 * i * i / (s * s));
    for (int i = 0; i <= 2 * r; i++) k[i] /= (float)sum;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float a = 0;
            for (int i = -r; i <= r; i++) { int xx = x + i < 0 ? 0 : x + i >= W ? W - 1 : x + i; a += k[i + r] * in[y * W + xx]; }
            t[y * W + x] = a;
        }
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            float a = 0;
            for (int i = -r; i <= r; i++) { int yy = y + i < 0 ? 0 : y + i >= H ? H - 1 : y + i; a += k[i + r] * t[yy * W + x]; }
            out[y * W + x] = a;
        }
    free(k);
    free(t);
}

/* rms difference of img to the sharp picture, central area */
static double rms(const dsd4_image *im, const float *sharp)
{
    double s = 0;
    long n = 0;
    for (int y = 64; y < H - 64; y++)
        for (int x = 64; x < W - 64; x++) {
            double d = ((const uint16_t *)im->data)[(y * W + x) * 3 + 1] / 65535.0 - 0.9 * sharp[y * W + x]; /* green: 0.9 */
            s += d * d;
            n++;
        }
    return sqrt(s / n);
}

static void make(dsd4_image *im, const float *pic, double sigma)
{
    float *b = malloc((size_t)W * H * sizeof *b);
    if (sigma > 0) blur(pic, b, sigma); else memcpy(b, pic, (size_t)W * H * sizeof *b);
    memset(im, 0, sizeof *im);
    im->width = W; im->height = H; im->channels = 3; im->bits = 16; im->dpi = 3200;
    im->stride = (size_t)W * 6;
    im->data = malloc(im->stride * H);
    for (int i = 0; i < W * H; i++) {
        double g = b[i] * (1 + 0.03 * nrand()); /* grain: on the film, after the camera's blur */
        for (int c = 0; c < 3; c++) {
            double v = g * (c == 0 ? 1.0 : c == 1 ? 0.9 : 0.8) * 65535;
            ((uint16_t *)im->data)[i * 3 + c] = (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v + 0.5);
        }
    }
    free(b);
}

/* first byte at which two files differ (-1: identical), and their sizes */
static long first_diff(const char *a, const char *b, long *size)
{
    FILE *f = fopen(a, "rb"), *g = fopen(b, "rb");
    long i = 0, d = -1;
    int x, y;
    if (!f || !g) { if (f) fclose(f); if (g) fclose(g); *size = -1; return -2; }
    do {
        x = fgetc(f); y = fgetc(g);
        if (x != y && d < 0) d = i;
        i++;
    } while (x != EOF || y != EOF);
    fclose(f); fclose(g);
    *size = x == y ? i - 1 : -1;
    return d;
}

/* deblur of a DNG: applied, samples changed, everything before them identical */
static int dng_case(dsd4_image *im, const char *dir, int float_bits)
{
    char a[1024], b[1024];
    snprintf(a, sizeof a, "%s/deblur-in-%d.dng", dir, float_bits);
    snprintf(b, sizeof b, "%s/deblur-out-%d.dng", dir, float_bits);
    dsd4_deblur_info di;
    long size;
    if (dsd4_write_dng(im, a, float_bits, NULL, 0) || dsd4_deblur_dng(a, b, DSD4_DEBLUR_HIGH, DSD4_TONE_LINEAR, &di))
        return 1;
    long d = first_diff(a, b, &size);
    const long pix = (long)W * H * 3 * (im->bits == 32 && float_bits ? float_bits / 8 : 2);
    printf("     DNG (%s): blur %.2f px, first changed byte %ld of %ld (samples start at %ld)\n",
           float_bits ? "half float" : "16-bit", di.sigma, d, size, size - pix);
    return !di.applied || size < 0 || d < size - pix;
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : "/tmp";
    float *pic = malloc((size_t)W * H * sizeof *pic);
    double tone[(W / B) * (H / B)];
    for (int i = 0; i < (W / B) * (H / B); i++) tone[i] = 0.1 + 0.7 * urand();
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) pic[y * W + x] = (float)(tone[(y / B) * (W / B) + x / B] * 0.9);
    int fail = 0;

    dsd4_image im;
    dsd4_deblur_info di;
    make(&im, pic, 3.0);
    double before = rms(&im, pic);
    int r = dsd4_deblur(&im, DSD4_DEBLUR_HIGH, DSD4_TONE_LINEAR, &di);
    double after = rms(&im, pic);
    /* gaussian 3 px, less the 1 px the scanner keeps: 2.8 px */
    printf("     deblur: blur 3.0 px measured %.2f (beyond the scanner's: 2.83); rms to sharp %.4f -> %.4f\n",
           di.sigma, before, after);
    if (r || !di.applied || di.sigma < 2.3 || di.sigma > 3.4) { printf("FAIL deblur-measures-blur\n"); fail++; }
    else printf("ok   deblur-measures-blur\n");
    if (!(after < 0.88 * before)) { printf("FAIL deblur-sharpens\n"); fail++; }
    else printf("ok   deblur-sharpens\n");
    dsd4_image_free(&im);

    /* the same through DNG files, 16-bit and half float */
    make(&im, pic, 3.0);
    int bad = dng_case(&im, dir, 0);
    dsd4_image f = im;
    f.bits = 32;
    f.stride = (size_t)W * 12;
    f.data = malloc(f.stride * H);
    for (int i = 0; i < W * H * 3; i++) ((float *)f.data)[i] = ((uint16_t *)im.data)[i] / 65535.0f;
    bad |= dng_case(&f, dir, 16);
    free(f.data);
    dsd4_image_free(&im);
    if (bad) { printf("FAIL deblur-dng-only-samples\n"); fail++; }
    else printf("ok   deblur-dng-only-samples\n");

    /* a full frame's width: the last tile column is narrow (3120 = 8 x 384
     * + 48); the frame's right edge must be no worse than its middle */
    {
        enum { FW = 3120, FH = 768 };
        float *fp = malloc((size_t)FW * FH * sizeof *fp);
        for (int y = 0; y < FH; y++)
            for (int x = 0; x < FW; x++) fp[y * FW + x] = (float)(tone[((y / B) * (W / B) + x / B % (W / B)) % ((W / B) * (H / B))] * 0.9);
        float *bl = malloc((size_t)FW * FH * sizeof *bl), *t = malloc((size_t)FW * FH * sizeof *t);
        enum { rr = 9 };
        double k[2 * rr + 1], ks = 0;
        for (int i = -rr; i <= rr; i++) ks += k[i + rr] = exp(-0.5 * i * i / 9.0);
        for (int i = 0; i <= 2 * rr; i++) k[i] /= ks;
        for (int y = 0; y < FH; y++)
            for (int x = 0; x < FW; x++) {
                double a = 0;
                for (int i = -rr; i <= rr; i++) { int xx = x + i < 0 ? 0 : x + i >= FW ? FW - 1 : x + i; a += k[i + rr] * fp[y * FW + xx]; }
                t[y * FW + x] = (float)a;
            }
        for (int y = 0; y < FH; y++)
            for (int x = 0; x < FW; x++) {
                double a = 0;
                for (int i = -rr; i <= rr; i++) { int yy = y + i < 0 ? 0 : y + i >= FH ? FH - 1 : y + i; a += k[i + rr] * t[yy * FW + x]; }
                bl[y * FW + x] = (float)a;
            }
        dsd4_image fr = { 0 };
        fr.width = FW; fr.height = FH; fr.channels = 3; fr.bits = 16; fr.dpi = 3200; fr.stride = (size_t)FW * 6;
        fr.data = malloc(fr.stride * FH);
        for (int i = 0; i < FW * FH; i++)
            for (int c = 0; c < 3; c++) {
                double v = bl[i] * (1 + 0.02 * nrand()) * 65535;
                ((uint16_t *)fr.data)[i * 3 + c] = (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v + 0.5);
            }
        dsd4_deblur(&fr, DSD4_DEBLUR_HIGH, DSD4_TONE_LINEAR, &di);
        double em = 0, ee = 0;
        long nm = 0, ne = 0;
        for (int y = 64; y < FH - 64; y++)
            for (int x = 0; x < FW; x++) {
                double d = ((uint16_t *)fr.data)[((size_t)y * FW + x) * 3 + 1] / 65535.0 - fp[y * FW + x];
                if (x >= FW - 48) { ee += d * d; ne++; }
                else if (x >= FW / 2 - 200 && x < FW / 2 + 200) { em += d * d; nm++; }
            }
        em = sqrt(em / nm); ee = sqrt(ee / ne);
        printf("     full-width frame: error to sharp, middle %.4f, right 48 px %.4f\n", em, ee);
        if (!di.applied || !(ee < 1.5 * em)) { printf("FAIL deblur-frame-edge\n"); fail++; }
        else printf("ok   deblur-frame-edge\n");
        dsd4_image_free(&fr);
        free(fp); free(bl); free(t);
    }

    make(&im, pic, 0.6);
    uint16_t *copy = malloc(im.stride * H);
    memcpy(copy, im.data, im.stride * H);
    r = dsd4_deblur(&im, DSD4_DEBLUR_HIGH, DSD4_TONE_LINEAR, &di);
    printf("     sharp picture: measured %.2f px, %s\n", di.sigma, di.applied ? "deblurred" : "unchanged");
    if (r || di.applied || memcmp(copy, im.data, im.stride * H)) { printf("FAIL deblur-leaves-sharp\n"); fail++; }
    else printf("ok   deblur-leaves-sharp\n");
    dsd4_image_free(&im);
    free(copy);
    free(pic);
    return fail != 0;
}
