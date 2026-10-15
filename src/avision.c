/*
 * SCSI-over-USB command layer.  Port of AVIOScanCommand, CmdRequestSense,
 * CmdTestUnitReady, CmdInquiry, CmdReserveUnit, CmdReleaseUnit, CmdScan,
 * CmdSetWindow, CmdRead, CmdSend, CmdObjectPosition and IsScannerReady from
 * MFSBaseLib2891.
 */
#include "dsd4_internal.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

void dsd4_logf(dsd4 *s, int level, const char *fmt, ...)
{
    if (!s || level > s->verbose) return;
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (s->log) s->log(s->log_ud, level, buf);
    else fprintf(stderr, "dsd4: %s\n", buf);
}

/* Mac OS ticks are 1/60 s.  The original library busy-waits on TickCount(). */
/* DSD4_TIME_SCALE (0..1) shortens all fixed waits; used by the simulator tests */
static double time_scale(void)
{
    static double k = -1;
    if (k < 0) {
        const char *e = getenv("DSD4_TIME_SCALE");
        k = e ? atof(e) : 1.0;
        if (k < 0 || k > 1) k = 1.0;
    }
    return k;
}

void av_delay_ticks(unsigned ticks)
{
    if (ticks) usleep((useconds_t)(ticks * 16667u * time_scale()));
}

unsigned av_now_ticks(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    double k = time_scale();
    double t = (ts.tv_sec * 60.0 + ts.tv_nsec / 16666667.0) / (k > 0 ? k : 1e-6);
    return (unsigned)(uint64_t)t;
}
#define now_ticks av_now_ticks



static void hexdump(dsd4 *s, const char *what, const uint8_t *p, uint32_t n)
{
    if (s->verbose < 4) return;
    char line[3 * 32 + 1];
    for (uint32_t o = 0; o < n && o < 256; o += 32) {
        int k = 0;
        for (uint32_t i = o; i < n && i < o + 32; i++) k += snprintf(line + k, sizeof line - k, "%02x ", p[i]);
        dsd4_logf(s, 4, "  %s+%03x: %s", what, o, line);
    }
}

/* CmdRequestSense: READ 22 bytes of sense and map to the driver's error code. */
static int request_sense(dsd4 *s, uint8_t *code)
{
    uint8_t cdb[6] = { OP_REQUEST_SENSE, 0, 0, 0, 0x16, 0 };
    uint8_t b[0x16] = { 0 };
    if (!av_command(s, cdb, 6, DSD4_DIR_IN, b, sizeof b)) return 0;
    dsd4_logf(s, 2, "sense: key %x asc %02x ascq %02x", b[2] & 0xf, b[12], b[13]);
    uint8_t c = AVE_GENERIC;
    if ((b[0] & 0x7f) == 0x70 || (b[0] & 0x7f) == 0x7f) {
        c = 0;
        if ((b[2] & 0xf) == 3) {
            if (b[12] == 0x81) {
                c = b[13] <= 8 ? (uint8_t)(200 + b[13]) : AVE_GENERIC;
            } else if (b[12] == 0x80) {
                switch (b[13]) {
                case 1: c = 0x67; break;
                case 2: c = 0x68; break;
                case 3: c = 0x69; break;
                case 5: c = 0xd1; break;
                default: c = AVE_GENERIC; break;
                }
            } else {
                c = AVE_GENERIC;
            }
        }
    }
    *code = c;
    return 1;
}

/* AVIOScanCommand */
int av_command(dsd4 *s, const uint8_t *cdb, int cdblen, enum dsd4_dir dir, void *data, uint32_t len)
{
    uint8_t c[10] = { 0 };
    memcpy(c, cdb, (size_t)cdblen);

    if (s->need_release) {
        s->need_release = 0;
        if (!av_release(s)) { s->need_release = 1; return 0; }
    }

    dsd4_logf(s, 3, "cmd %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  %s %u",
              c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7], c[8], c[9],
              dir == DSD4_DIR_IN ? "in" : dir == DSD4_DIR_OUT ? "out" : "-", len);
    if (dir == DSD4_DIR_OUT) hexdump(s, "out", data, len);

    uint8_t status = 0xff;
    int rc = s->t->exec(s->t, c, dir, data, len, &status);
    av_delay_ticks(s->cmd_delay_us ? 0 : 1);
    if (s->cmd_delay_us) usleep((useconds_t)s->cmd_delay_us);
    if (rc < 0) {
        dsd4_logf(s, 1, "USB transfer failed (%d) for opcode %02x", rc, c[0]);
        s->last_error = 0x83;
        return 0;
    }
    if (dir == DSD4_DIR_IN && status == 0) hexdump(s, "in", data, len);

    switch (status) {
    case 0:
        s->last_error = 0;
        return 1;
    case 2:
        if (c[0] == OP_REQUEST_SENSE) {
            s->last_error = AVE_GENERIC;
        } else {
            uint8_t code;
            s->last_error = request_sense(s, &code) ? code : AVE_SENSE_FAILED;
        }
        break;
    case 8:
        s->last_error = AVE_BUSY;
        break;
    default:
        s->last_error = status;
        break;
    }
    dsd4_logf(s, 2, "opcode %02x status %u -> error 0x%02x", c[0], status, s->last_error);
    return 0;
}

int av_test_unit_ready(dsd4 *s)
{
    uint8_t cdb[6] = { OP_TEST_UNIT_READY };
    return av_command(s, cdb, 6, DSD4_DIR_NONE, NULL, 0);
}

int av_inquiry(dsd4 *s, uint8_t buf[96])
{
    uint8_t cdb[6] = { OP_INQUIRY, 0, 0, 0, 0x60, 0 };
    return av_command(s, cdb, 6, DSD4_DIR_IN, buf, 0x60);
}

int av_reserve(dsd4 *s)
{
    uint8_t cdb[6] = { OP_RESERVE_UNIT };
    return av_command(s, cdb, 6, DSD4_DIR_NONE, NULL, 0);
}

int av_release(dsd4 *s)
{
    uint8_t cdb[6] = { OP_RELEASE_UNIT };
    return av_command(s, cdb, 6, DSD4_DIR_NONE, NULL, 0);
}

int av_scan(dsd4 *s, uint8_t flags)
{
    uint8_t cdb[6] = { OP_SCAN, 0, 0, 0, 1, flags };
    return av_command(s, cdb, 6, DSD4_DIR_NONE, NULL, 0);
}

int av_set_window(dsd4 *s, uint32_t len, const uint8_t *buf)
{
    uint8_t cdb[10] = { OP_SET_WINDOW };
    cdb[6] = (uint8_t)(len >> 16);
    cdb[7] = (uint8_t)(len >> 8);
    cdb[8] = (uint8_t)len;
    return av_command(s, cdb, 10, DSD4_DIR_OUT, (void *)buf, len);
}

/* CmdRead: data type qualifier is always 0x0a0d. */
int av_read(dsd4 *s, uint8_t dtc, uint32_t len, void *buf)
{
    uint8_t cdb[10] = { OP_READ, 0, dtc, 0, 0x0a, 0x0d };
    cdb[6] = (uint8_t)(len >> 16);
    cdb[7] = (uint8_t)(len >> 8);
    cdb[8] = (uint8_t)len;
    return av_command(s, cdb, 10, DSD4_DIR_IN, buf, len);
}

int av_send(dsd4 *s, uint8_t dtc, uint16_t qual, uint32_t len, const void *buf)
{
    uint8_t cdb[10] = { OP_SEND, 0, dtc, 0, (uint8_t)(qual >> 8), (uint8_t)qual };
    cdb[6] = (uint8_t)(len >> 16);
    cdb[7] = (uint8_t)(len >> 8);
    cdb[8] = (uint8_t)len;
    return av_command(s, cdb, 10, DSD4_DIR_OUT, (void *)buf, len);
}

/* CmdObjectPosition: only operations 0,1,2,4,5 and 8 are ever issued. */
int av_object_position(dsd4 *s, uint8_t op)
{
    if (!((op < 6 && op != 3) || op == 8)) return 0;
    uint8_t cdb[10] = { OP_OBJECT_POSITION, op };
    return av_command(s, cdb, 10, DSD4_DIR_NONE, NULL, 0);
}

/* IsScannerReady: poll TEST UNIT READY while the scanner reports BUSY.
 * [aa68] is always 1 after InitializeDriver, so the limit is in ticks. */
int av_is_ready(dsd4 *s, unsigned ticks)
{
    unsigned t0 = now_ticks();
    for (;;) {
        if (av_test_unit_ready(s)) return 1;
        if (s->last_error != AVE_BUSY) return 0;
        unsigned lim = s->first_init ? ticks : ticks * 30;
        if (now_ticks() - t0 > lim) return 0;
    }
}
