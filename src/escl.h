/* AirScan (eSCL) bridge for macOS Image Capture / Preview. */
#ifndef DSD4_ESCL_H
#define DSD4_ESCL_H

#include "dsdual4.h"

typedef void (*dsd4_escl_log_fn)(const char *msg);

typedef struct {
    int port;               /* default 8090 */
    const char *name;       /* Bonjour service name */
    dsd4_film film;         /* initial film type */
    int localhost_only;     /* bind to loopback and advertise on this Mac only */
    dsd4_escl_log_fn log;
    const void *icc;        /* scanner input profile for linear positives */
    size_t icc_len;
    const void *icc_n;      /* Nega Input profile for Minolta-processed colour negatives */
    size_t icc_n_len;
} dsd4_escl_options;

/* Runs forever (until the process is terminated). */
int dsd4_escl_serve(dsd4 *s, const dsd4_escl_options *o);

#endif
