/*
 * Transport abstraction.
 *
 * The original Mac OS X driver (MFSIOUsb2891.bundle, MFSIOUsbExecCommand)
 * frames every SCSI-style command on the USB bulk pipes like this:
 *
 *   1. bulk-OUT : the CDB, always zero-padded to 10 bytes
 *   2. data     : bulk-OUT (host->scanner) or bulk-IN (scanner->host), optional
 *   3. bulk-IN  : one status byte (0 = GOOD, 2 = CHECK CONDITION, 8 = BUSY)
 *
 * A transport implements exactly that exchange.  Two implementations exist:
 * the IOKit one (usb_macos.c) for real hardware and a protocol-level
 * simulator (sim.c) used for testing without a scanner.
 */
#ifndef DSD4_TRANSPORT_H
#define DSD4_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

enum dsd4_dir { DSD4_DIR_NONE = 0, DSD4_DIR_IN = 1, DSD4_DIR_OUT = 2 };

typedef struct dsd4_transport dsd4_transport;

struct dsd4_transport {
    /* Execute one command.  Returns 0 on transport success and stores the
     * scanner's status byte in *status; returns a negative value if the USB
     * exchange itself failed. */
    int  (*exec)(dsd4_transport *t, const uint8_t cdb[10], enum dsd4_dir dir,
                 void *data, size_t len, uint8_t *status);
    void (*close)(dsd4_transport *t);
    void *priv;
    char description[128];
};

/* Konica Minolta DiMAGE Scan Dual IV (firmware product id "FS-V31"). */
#define DSD4_USB_VID 0x132b
#define DSD4_USB_PID 0x000a

dsd4_transport *dsd4_usb_open(char *err, size_t errlen);
int dsd4_usb_present(void); /* scanner attached (does not open it) */
dsd4_transport *dsd4_sim_open(const char *firmware_path, char *err, size_t errlen);

#endif
