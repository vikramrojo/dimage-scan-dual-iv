/*
 * IOKit user-space USB transport for the DiMAGE Scan Dual IV.
 *
 * Functional port of the PowerPC Mach-O bundle MFSIOUsb2891.bundle
 * (MFSIOUsbInit / FindUsbDevices / MFSIOUsbExecCommand / TakeFeedBack).
 * The original bundle:
 *   - matched IOUSBDevice with idVendor 0x132b and the product id passed in
 *     by the caller (0x000a for the Dual IV),
 *   - opened the first interface and located its bulk-in, bulk-out and
 *     interrupt pipes,
 *   - for every command wrote a 10 byte zero padded CDB to bulk-out, then the
 *     data phase, then read a single status byte ("feedback") from bulk-in.
 *
 * This file does the same with the modern IOUSBLib user client API, which is
 * still supported on current macOS on both Apple silicon and Intel.  No kernel
 * extension is required: the scanner is a vendor-specific device that no
 * Apple driver claims, so user space can open it directly.
 */
#include "transport.h"

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>
#include <mach/mach.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef IOUSBDeviceInterface320    **DevIf;
typedef IOUSBInterfaceInterface300 **IntfIf;

struct usb_priv {
    DevIf   dev;
    IntfIf  intf;
    UInt8   pipe_in, pipe_out, pipe_irq;
    UInt16  mps_in, mps_out;
};

/* Generous timeouts.  The original OS X bundle used untimed ReadPipe/WritePipe;
 * the classic Mac OS driver gave every command 60 s (0xE10 ticks).  Some
 * commands (warm-up, calibration reads, carriage moves) take a while before
 * the scanner returns status, so the status phase gets the longest timeout. */
#define CMD_TIMEOUT_MS     30000u
#define DATA_TIMEOUT_MS   120000u
#define STATUS_TIMEOUT_MS 180000u
#define CHUNK              (256u * 1024u) /* multiple of every bulk max packet size */

static void set_err(char *err, size_t n, const char *fmt, kern_return_t kr)
{
    if (err && n) snprintf(err, n, fmt, kr);
}

static IOReturn recover_pipe(struct usb_priv *p, UInt8 pipe, IOReturn kr)
{
    if (kr == kIOUSBPipeStalled || kr == kIOUSBTransactionTimeout)
        (*p->intf)->ClearPipeStallBothEnds(p->intf, pipe);
    return kr;
}

static int usb_exec(dsd4_transport *t, const uint8_t cdb[10], enum dsd4_dir dir,
                    void *data, size_t len, uint8_t *status)
{
    struct usb_priv *p = t->priv;
    IOReturn kr;
    uint8_t cmd[10];
    memcpy(cmd, cdb, 10);

    /* 1. CDB (OutWithoutIntermediateData / first WritePipe of the other two) */
    kr = (*p->intf)->WritePipeTO(p->intf, p->pipe_out, cmd, 10, CMD_TIMEOUT_MS, CMD_TIMEOUT_MS);
    if (kr != kIOReturnSuccess) { recover_pipe(p, p->pipe_out, kr); return -1; }

    /* 2. data phase */
    if (dir == DSD4_DIR_OUT && len) {
        uint8_t *b = data;
        size_t done = 0;
        while (done < len) {
            UInt32 n = (UInt32)((len - done) > CHUNK ? CHUNK : (len - done));
            kr = (*p->intf)->WritePipeTO(p->intf, p->pipe_out, b + done, n, DATA_TIMEOUT_MS, DATA_TIMEOUT_MS);
            if (kr != kIOReturnSuccess) { recover_pipe(p, p->pipe_out, kr); return -2; }
            done += n;
        }
    } else if (dir == DSD4_DIR_IN && len) {
        uint8_t *b = data;
        size_t done = 0;
        while (done < len) {
            UInt32 n = (UInt32)((len - done) > CHUNK ? CHUNK : (len - done));
            kr = (*p->intf)->ReadPipeTO(p->intf, p->pipe_in, b + done, &n, DATA_TIMEOUT_MS, DATA_TIMEOUT_MS);
            if (kr != kIOReturnSuccess) { recover_pipe(p, p->pipe_in, kr); return -3; }
            if (n == 0) return -4;
            done += n;
        }
    }

    /* 3. status byte (TakeFeedBack: ReadPipe of 1 byte on bulk-in) */
    uint8_t st = 0xff;
    UInt32 sn = 1;
    kr = (*p->intf)->ReadPipeTO(p->intf, p->pipe_in, &st, &sn, STATUS_TIMEOUT_MS, STATUS_TIMEOUT_MS);
    if (kr != kIOReturnSuccess || sn != 1) { recover_pipe(p, p->pipe_in, kr); return -5; }
    *status = st;
    return 0;
}

static void usb_close(dsd4_transport *t)
{
    struct usb_priv *p = t->priv;
    if (p) {
        if (p->intf) {
            (*p->intf)->USBInterfaceClose(p->intf);
            (*p->intf)->Release(p->intf);
        }
        if (p->dev) {
            (*p->dev)->USBDeviceClose(p->dev);
            (*p->dev)->Release(p->dev);
        }
        free(p);
    }
    free(t);
}

static IntfIf open_first_interface(DevIf dev, char *err, size_t errlen)
{
    IOUSBFindInterfaceRequest req = {
        .bInterfaceClass = kIOUSBFindInterfaceDontCare,
        .bInterfaceSubClass = kIOUSBFindInterfaceDontCare,
        .bInterfaceProtocol = kIOUSBFindInterfaceDontCare,
        .bAlternateSetting = kIOUSBFindInterfaceDontCare,
    };
    io_iterator_t it = 0;
    if ((*dev)->CreateInterfaceIterator(dev, &req, &it) != kIOReturnSuccess) {
        set_err(err, errlen, "CreateInterfaceIterator failed (0x%x)", 0);
        return NULL;
    }
    IntfIf intf = NULL;
    io_service_t svc;
    while (!intf && (svc = IOIteratorNext(it))) {
        IOCFPlugInInterface **plug = NULL;
        SInt32 score;
        kern_return_t kr = IOCreatePlugInInterfaceForService(svc, kIOUSBInterfaceUserClientTypeID,
                                                             kIOCFPlugInInterfaceID, &plug, &score);
        IOObjectRelease(svc);
        if (kr != kIOReturnSuccess || !plug) continue;
        (*plug)->QueryInterface(plug, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID300), (LPVOID *)&intf);
        (*plug)->Release(plug);
    }
    IOObjectRelease(it);
    if (!intf) set_err(err, errlen, "no USB interface found on scanner (0x%x)", 0);
    return intf;
}

dsd4_transport *dsd4_usb_open(char *err, size_t errlen)
{
    CFMutableDictionaryRef match = IOServiceMatching(kIOUSBDeviceClassName);
    if (!match) { set_err(err, errlen, "IOServiceMatching failed (0x%x)", 0); return NULL; }
    SInt32 vid = DSD4_USB_VID, pid = DSD4_USB_PID;
    CFNumberRef nv = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &vid);
    CFNumberRef np = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &pid);
    CFDictionarySetValue(match, CFSTR(kUSBVendorID), nv);
    CFDictionarySetValue(match, CFSTR(kUSBProductID), np);
    CFRelease(nv);
    CFRelease(np);

    io_iterator_t it = 0;
    kern_return_t kr = IOServiceGetMatchingServices(MACH_PORT_NULL, match, &it); /* consumes match */
    if (kr != KERN_SUCCESS) { set_err(err, errlen, "IOServiceGetMatchingServices failed (0x%x)", kr); return NULL; }

    DevIf dev = NULL;
    io_service_t svc;
    while (!dev && (svc = IOIteratorNext(it))) {
        IOCFPlugInInterface **plug = NULL;
        SInt32 score;
        kr = IOCreatePlugInInterfaceForService(svc, kIOUSBDeviceUserClientTypeID,
                                               kIOCFPlugInInterfaceID, &plug, &score);
        IOObjectRelease(svc);
        if (kr != kIOReturnSuccess || !plug) continue;
        (*plug)->QueryInterface(plug, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID320), (LPVOID *)&dev);
        (*plug)->Release(plug);
    }
    IOObjectRelease(it);
    if (!dev) {
        set_err(err, errlen, "DiMAGE Scan Dual IV (USB 132b:000a) not found%.0x", 0);
        return NULL;
    }

    kr = (*dev)->USBDeviceOpen(dev);
    if (kr != kIOReturnSuccess) {
        (*dev)->Release(dev);
        set_err(err, errlen, "cannot open scanner (another app has it open?) IOReturn 0x%x", kr);
        return NULL;
    }

    UInt8 cur = 0;
    (*dev)->GetConfiguration(dev, &cur);
    if (cur == 0) {
        IOUSBConfigurationDescriptorPtr cd = NULL;
        if ((*dev)->GetConfigurationDescriptorPtr(dev, 0, &cd) == kIOReturnSuccess && cd)
            (*dev)->SetConfiguration(dev, cd->bConfigurationValue);
    }

    IntfIf intf = open_first_interface(dev, err, errlen);
    if (!intf) { (*dev)->USBDeviceClose(dev); (*dev)->Release(dev); return NULL; }
    kr = (*intf)->USBInterfaceOpen(intf);
    if (kr != kIOReturnSuccess) {
        (*intf)->Release(intf);
        (*dev)->USBDeviceClose(dev);
        (*dev)->Release(dev);
        set_err(err, errlen, "cannot open scanner interface IOReturn 0x%x", kr);
        return NULL;
    }

    struct usb_priv *p = calloc(1, sizeof *p);
    p->dev = dev;
    p->intf = intf;
    UInt8 nep = 0;
    (*intf)->GetNumEndpoints(intf, &nep);
    for (UInt8 i = 1; i <= nep; i++) {
        UInt8 direction, number, type, interval;
        UInt16 mps;
        if ((*intf)->GetPipeProperties(intf, i, &direction, &number, &type, &mps, &interval) != kIOReturnSuccess)
            continue;
        if (type == kUSBBulk && direction == kUSBIn && !p->pipe_in) { p->pipe_in = i; p->mps_in = mps; }
        else if (type == kUSBBulk && direction == kUSBOut && !p->pipe_out) { p->pipe_out = i; p->mps_out = mps; }
        else if (type == kUSBInterrupt && !p->pipe_irq) p->pipe_irq = i;
    }
    dsd4_transport *t = calloc(1, sizeof *t);
    t->priv = p;
    t->exec = usb_exec;
    t->close = usb_close;
    if (!p->pipe_in || !p->pipe_out) {
        set_err(err, errlen, "scanner has no bulk in/out pipe pair%.0x", 0);
        usb_close(t);
        return NULL;
    }
    snprintf(t->description, sizeof t->description,
             "USB %04x:%04x bulk-in %u (mps %u) bulk-out %u (mps %u)",
             DSD4_USB_VID, DSD4_USB_PID, p->pipe_in, p->mps_in, p->pipe_out, p->mps_out);
    return t;
}
