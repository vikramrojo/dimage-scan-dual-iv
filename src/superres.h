/* sub-pixel registration helpers (superres.c) */
#ifndef DSD4_SUPERRES_H
#define DSD4_SUPERRES_H

#include "dsdual4.h"

typedef struct {
    int w, h;
    float *v;
} sr_plane;

void   sr_fft(double *re, double *im, int n, int inv);
double sr_tile_shift(const sr_plane *a, const sr_plane *b, int tx, int ty, int n, double *dx, double *dy);
int    sr_fit_line(const double *u, const double *v, const double *w, int n, double *c0, double *c1, double *rms);
int    sr_plane_from_image(const dsd4_image *im, sr_plane *p);
double sr_sharpness(const sr_plane *p);
void   sr_blur(sr_plane *p, double sigma);

#endif
