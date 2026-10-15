/*
 * AirScan (eSCL) bridge: exposes the DiMAGE Scan Dual IV to macOS Image
 * Capture / Preview (and any other eSCL client) as a network scanner on this
 * Mac.  Discovery uses Bonjour (_uscan._tcp), images are encoded with ImageIO.
 *
 * Film specific settings (film type, frame, focus, inversion) cannot be
 * expressed in eSCL; they are set on the small web page served at "/" (also
 * advertised as the scanner's admin URL) and apply to the following scans.
 */
#include "dsd4_internal.h"
#include "escl.h"

#include <ApplicationServices/ApplicationServices.h>
#include <ImageIO/ImageIO.h>
#include <arpa/inet.h>
#include <dns_sd.h>
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define NS "xmlns:scan=\"http://schemas.hp.com/imaging/escl/2011/05/03\" xmlns:pwg=\"http://www.pwg.org/schemas/2010/12/sm\""
#define MAX_JOBS 16

typedef struct {
    char id[40];
    int used;
    int state;            /* 0 pending, 1 processing, 2 done, 3 failed, 4 canceled */
    int delivered;
    int thread_active;    /* a job_thread still owns this slot */
    time_t created;
    dsd4_scan_params p;
    int gray, bits16;
    char format[32];
    uint8_t *doc;
    size_t doc_len;
    int error;
} job_t;

typedef struct {
    dsd4 *s;
    pthread_mutex_t scanner_lock; /* one scanner operation at a time */
    pthread_mutex_t lock;         /* jobs and settings */
    pthread_cond_t cond;
    job_t jobs[MAX_JOBS];
    int port;
    char name[128];
    char uuid[40];
    char host[256];
    /* settings from the web page */
    dsd4_film film;
    int frame;          /* 0 = current */
    int autofocus;
    int invert;        /* negatives: 0 raw, 1 simple invert, 2 Minolta processing */
    int focus;
    double ev;
    int samples;
    int af_mode;       /* DSD4_AF_FAST / DSD4_AF_CLASSIC */
    char status[256];
    int busy;
    dsd4_escl_log_fn log;
    const void *icc;
    size_t icc_len;
    const void *icc_n;
    size_t icc_n_len;
    int shared;                   /* open the scanner per operation */
    void (*on_open)(dsd4 *, void *);
    void *ud;
} server_t;

static server_t g;

static void logmsg(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* scanner_lock must be held.  Shared mode: open and initialise the scanner
 * (quick when it is already calibrated).  Returns DSD4_OK or an error. */
static int acquire(void)
{
    if (g.s) return DSD4_OK;
    if (!g.shared) return DSD4_ERR_IO;
    char err[256];
    if (dsd4_open_usb(&g.s, err, sizeof err) != DSD4_OK) {
        g.s = NULL;
        return DSD4_ERR_IO;
    }
    if (g.on_open) g.on_open(g.s, g.ud);
    int r = dsd4_initialize(g.s, 0, NULL, NULL);
    if (r) {
        dsd4_close(g.s);
        g.s = NULL;
    }
    return r;
}

static void release(void)
{
    if (g.shared && g.s) {
        dsd4_close(g.s);
        g.s = NULL;
    }
}

static int scanner_code(void) { return g.s ? dsd4_last_error(g.s) : 0; }
static void logmsg(const char *fmt, ...)
{
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    if (g.log) g.log(b);
    else fprintf(stderr, "%s\n", b);
}

/* ------------------------------------------------------------ utilities */

typedef struct { char *p; size_t n, cap; } sbuf;

static void sb_add(sbuf *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_add(sbuf *b, const char *fmt, ...)
{
    va_list ap;
    for (;;) {
        va_start(ap, fmt);
        int k = vsnprintf(b->p ? b->p + b->n : NULL, b->p ? b->cap - b->n : 0, fmt, ap);
        va_end(ap);
        if (k < 0) return;
        if (b->p && b->n + (size_t)k < b->cap) { b->n += (size_t)k; return; }
        size_t cap = (b->cap + (size_t)k + 1) * 2;
        char *np = realloc(b->p, cap);
        if (!np) return; /* keep the old buffer; output is truncated */
        b->p = np;
        b->cap = cap;
    }
}

static void make_uuid(char *out)
{
    uint8_t r[16];
    arc4random_buf(r, sizeof r);
    r[6] = (r[6] & 0x0f) | 0x40;
    r[8] = (r[8] & 0x3f) | 0x80;
    sprintf(out, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", r[0], r[1], r[2], r[3],
            r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15]);
}

/* naive XML element text lookup ("<scan:XResolution>300</..." -> "300") */
static int xml_get(const char *xml, const char *tag, char *out, size_t n)
{
    const char *p = xml;
    size_t tl = strlen(tag);
    while ((p = strstr(p, tag))) {
        /* match "<tag>" or "<prefix:tag>" */
        const char *lt = p;
        while (lt > xml && lt[-1] != '<' && lt[-1] != ':' && lt[-1] != '/') lt--;
        if (lt > xml && (lt[-1] == '<' || lt[-1] == ':') && p[tl] == '>') {
            if (lt[-1] == ':') {
                const char *q = lt - 1;
                while (q > xml && q[-1] != '<' && q[-1] != '/') q--;
                if (q <= xml || q[-1] != '<') { p += tl; continue; }
            }
            const char *v = p + tl + 1, *e = strchr(v, '<');
            if (!e) return 0;
            size_t k = (size_t)(e - v);
            if (k >= n) k = n - 1;
            memcpy(out, v, k);
            out[k] = 0;
            return 1;
        }
        p += tl;
    }
    return 0;
}

/* ------------------------------------------------------------ encoding */

/* Encode for the client.  'icc' set: the image is linear 16-bit scanner RGB
 * described by the scanner profile; it is tagged with it (PNG/TIFF) or
 * converted to sRGB by ColorSync (JPEG/PDF).  Otherwise the image is already
 * display-ready 8-bit sRGB / gray. */
static uint8_t *encode(const dsd4_image *im, const char *fmt, const void *icc, size_t icc_len, int want16, size_t *len)
{
    const int ch = im->channels;
    const int jpeg = !strcmp(fmt, "image/jpeg"), pdf = !strcmp(fmt, "application/pdf");
    CGColorSpaceRef cs = NULL;
    if (icc && ch == 3) {
        CFDataRef d = CFDataCreate(NULL, icc, (CFIndex)icc_len);
        cs = CGColorSpaceCreateWithICCData(d);
        CFRelease(d);
    }
    if (!cs) {
        icc = NULL;
        cs = ch == 3 ? CGColorSpaceCreateWithName(kCGColorSpaceSRGB)
                     : CGColorSpaceCreateWithName(kCGColorSpaceGenericGrayGamma2_2);
    }
    const int bits = im->bits;
    const size_t stride = (size_t)im->width * ch * (bits / 8);
    uint8_t *px = malloc(stride * im->height);
    if (!px) { CGColorSpaceRelease(cs); return NULL; }
    const size_t n = (size_t)im->width * im->height * ch;
    if (bits == 16) /* CoreGraphics wants big-endian 16-bit unless told otherwise */
        for (size_t i = 0; i < n; i++) put_be16(px + 2 * i, ((uint16_t *)im->data)[i]);
    else
        memcpy(px, im->data, n);
    CGDataProviderRef dp = CGDataProviderCreateWithData(NULL, px, stride * im->height, NULL);
    CGImageRef img = CGImageCreate((size_t)im->width, (size_t)im->height, (size_t)bits, (size_t)(bits * ch), stride,
                                   cs, kCGImageAlphaNone | (bits == 16 ? kCGBitmapByteOrder16Big : 0), dp, NULL,
                                   false, kCGRenderingIntentPerceptual);
    CGDataProviderRelease(dp);
    CGColorSpaceRelease(cs);
    if (!img) { free(px); return NULL; }
    if (icc && (jpeg || pdf || bits == 8 || !want16)) {
        /* colour-convert the scanner RGB into an 8-bit sRGB image */
        CGColorSpaceRef srgb = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        CGContextRef bc = CGBitmapContextCreate(NULL, (size_t)im->width, (size_t)im->height, 8, 0, srgb,
                                                kCGImageAlphaNoneSkipLast);
        CGColorSpaceRelease(srgb);
        if (bc) {
            CGContextSetRenderingIntent(bc, kCGRenderingIntentPerceptual);
            CGContextDrawImage(bc, CGRectMake(0, 0, im->width, im->height), img);
            CGImageRef conv = CGBitmapContextCreateImage(bc);
            CGContextRelease(bc);
            if (conv) { CGImageRelease(img); img = conv; }
        }
    }
    CFMutableDataRef out = CFDataCreateMutable(NULL, 0);
    if (pdf) {
        double w = im->width * 72.0 / im->dpi, h = im->height * 72.0 / im->dpi;
        CGRect box = CGRectMake(0, 0, w, h);
        CGDataConsumerRef dc = CGDataConsumerCreateWithCFData(out);
        CGContextRef ctx = CGPDFContextCreate(dc, &box, NULL);
        CGPDFContextBeginPage(ctx, NULL);
        CGContextDrawImage(ctx, box, img);
        CGPDFContextEndPage(ctx);
        CGPDFContextClose(ctx);
        CGContextRelease(ctx);
        CGDataConsumerRelease(dc);
    } else {
        CFStringRef uti = jpeg ? CFSTR("public.jpeg") : !strcmp(fmt, "image/png") ? CFSTR("public.png") : CFSTR("public.tiff");
        CGImageDestinationRef d = CGImageDestinationCreateWithData(out, uti, 1, NULL);
        if (!d) {
            CGImageRelease(img);
            free(px);
            CFRelease(out);
            return NULL;
        }
        double dpi = im->dpi, q = 0.92;
        CFNumberRef ndpi = CFNumberCreate(NULL, kCFNumberDoubleType, &dpi);
        CFNumberRef nq = CFNumberCreate(NULL, kCFNumberDoubleType, &q);
        const void *k[] = { kCGImagePropertyDPIWidth, kCGImagePropertyDPIHeight, kCGImageDestinationLossyCompressionQuality };
        const void *v[] = { ndpi, ndpi, nq };
        CFDictionaryRef props = CFDictionaryCreate(NULL, k, v, 3, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
        CGImageDestinationAddImage(d, img, props); /* embeds the image's colour space */
        CGImageDestinationFinalize(d);
        CFRelease(props);
        CFRelease(ndpi);
        CFRelease(nq);
        CFRelease(d);
    }
    CGImageRelease(img);
    free(px);
    *len = (size_t)CFDataGetLength(out);
    uint8_t *res = malloc(*len ? *len : 1);
    if (res) memcpy(res, CFDataGetBytePtr(out), *len);
    CFRelease(out);
    return res;
}

/* ---------------------------------------------------------- scanning */

static void scan_progress(void *ud, const char *stage, double f)
{
    pthread_mutex_lock(&g.lock);
    if (f >= 0) snprintf(g.status, sizeof g.status, "%s %.0f%%", stage, f * 100);
    else snprintf(g.status, sizeof g.status, "%s", stage);
    pthread_mutex_unlock(&g.lock);
}

static void *job_thread(void *arg)
{
    job_t *j = arg;
    pthread_mutex_lock(&g.scanner_lock);
    pthread_mutex_lock(&g.lock);
    if (j->state == 4) {
        j->thread_active = 0;
        pthread_mutex_unlock(&g.lock);
        pthread_mutex_unlock(&g.scanner_lock);
        return NULL;
    }
    j->state = 1;
    g.busy = 1;
    pthread_mutex_unlock(&g.lock);

    dsd4_image im;
    const int negative = j->p.film == DSD4_FILM_NEGATIVE_COLOR || j->p.film == DSD4_FILM_NEGATIVE_BW;
    const int minolta = negative && g.invert == 2; /* previews: from one 800 dpi scan */
    scan_progress(NULL, "opening scanner", -1);
    int r = acquire();
    if (r == DSD4_OK)
        r = minolta && j->p.preview ? dsd4_preview_minolta(g.s, &j->p, &im, NULL, scan_progress, NULL)
            : minolta               ? dsd4_scan_minolta(g.s, &j->p, &im, NULL, scan_progress, NULL)
                                    : dsd4_scan(g.s, &j->p, &im, scan_progress, NULL);
    uint8_t *doc = NULL;
    size_t len = 0;
    if (r == DSD4_OK) {
        if (j->gray && im.channels == 3) {
            /* eSCL gray requested on colour film: luminance */
            size_t n = (size_t)im.width * im.height;
            uint16_t *p = (uint16_t *)im.data;
            for (size_t i = 0; i < n; i++)
                p[i] = (uint16_t)((p[i * 3] * 2126u + p[i * 3 + 1] * 7152u + p[i * 3 + 2] * 722u) / 10000u);
            im.channels = 1;
            im.stride = (size_t)im.width * 2;
        }
        if (g.invert && negative && !minolta) dsd4_invert_negative(&im, 1);
        /* linear colour slides keep their data and get the scanner profile;
         * everything else is made display-ready here */
        const int use_icc = g.icc && !negative && im.channels == 3 && j->p.film == DSD4_FILM_POSITIVE_COLOR;
        const int use_icc_n = g.icc_n && minolta && im.channels == 3;
        if (!use_icc && !use_icc_n && !minolta) dsd4_to_display(&im);
        else if (minolta && !use_icc_n && !j->bits16) {
            /* B&W Minolta output is already a display-ready positive */
            size_t n = (size_t)im.width * im.height * im.channels;
            for (size_t i = 0; i < n; i++) im.data[i] = (uint8_t)(((uint16_t *)im.data)[i] >> 8);
            im.bits = 8;
            im.stride = (size_t)im.width * im.channels;
        }
        doc = encode(&im, j->format, use_icc ? g.icc : use_icc_n ? g.icc_n : NULL, use_icc ? g.icc_len : g.icc_n_len,
                     j->bits16, &len);
        logmsg("scan job %s: %dx%d %d dpi -> %s, %zu bytes", j->id, im.width, im.height, im.dpi, j->format, len);
        dsd4_image_free(&im);
    } else {
        logmsg("scan job %s failed: %s (scanner code 0x%02x)%s", j->id, dsd4_strerror(r), scanner_code(),
               g.s ? "" : " - scanner not connected, or in use by another program");
        if (r == DSD4_ERR_IO && !g.shared) {
            /* scanner unplugged or powered off: exit so launchd restarts us and
             * we wait for the device again */
            logmsg("lost the USB connection to the scanner; exiting");
            _exit(75);
        }
    }
    release();
    pthread_mutex_lock(&g.lock);
    j->doc = doc;
    j->doc_len = len;
    j->error = r;
    if (j->state != 4) j->state = doc ? 2 : 3;
    else { free(doc); j->doc = NULL; j->doc_len = 0; }
    j->thread_active = 0;
    g.busy = 0;
    snprintf(g.status, sizeof g.status, "%s", doc ? "idle" : dsd4_strerror(r));
    pthread_cond_broadcast(&g.cond);
    pthread_mutex_unlock(&g.lock);
    pthread_mutex_unlock(&g.scanner_lock);
    return NULL;
}

/* ------------------------------------------------------------- HTTP */

typedef struct {
    char method[8], path[256];
    char *body;
    size_t body_len;
    int keep_alive;
} request;

static int send_all(int fd, const void *p, size_t n)
{
    const uint8_t *b = p;
    while (n) {
        ssize_t k = send(fd, b, n, 0);
        if (k < 0) { if (errno == EINTR) continue; return -1; }
        b += k;
        n -= (size_t)k;
    }
    return 0;
}

static void respond(int fd, const request *rq, int code, const char *reason, const char *ctype, const void *body,
                    size_t len, const char *extra)
{
    char h[1024];
    int k = snprintf(h, sizeof h,
                     "HTTP/1.1 %d %s\r\nServer: dsd4\r\nContent-Length: %zu\r\n%s%s%s%s\r\n",
                     code, reason, len, ctype ? "Content-Type: " : "", ctype ? ctype : "", ctype ? "\r\n" : "",
                     extra ? extra : "");
    send_all(fd, h, (size_t)k);
    if (len && strcmp(rq->method, "HEAD")) send_all(fd, body, len);
}

static int read_request(int fd, request *rq, char *buf, size_t cap, size_t *have)
{
    memset(rq, 0, sizeof *rq);
    for (;;) {
        buf[*have] = 0;
        char *end = strstr(buf, "\r\n\r\n");
        if (end) {
            size_t hl = (size_t)(end - buf) + 4;
            if (sscanf(buf, "%7s %255s", rq->method, rq->path) != 2) return -1;
            size_t cl = 0;
            const char *c = strcasestr(buf, "\r\nContent-Length:");
            if (c && c < end) cl = strtoul(c + 17, NULL, 10);
            const char *conn = strcasestr(buf, "\r\nConnection:");
            rq->keep_alive = !(conn && conn < end && strncasecmp(conn + 13, " close", 6) == 0);
            const char *ex = strcasestr(buf, "\r\nExpect: 100-continue");
            if (ex && ex < end && *have < hl + cl) send_all(fd, "HTTP/1.1 100 Continue\r\n\r\n", 25);
            if (cl > cap - hl - 1) return -1;
            while (*have < hl + cl) {
                ssize_t k = recv(fd, buf + *have, cap - *have - 1, 0);
                if (k <= 0) return -1;
                *have += (size_t)k;
            }
            rq->body = malloc(cl + 1);
            if (!rq->body) return -1;
            memcpy(rq->body, buf + hl, cl);
            rq->body[cl] = 0;
            rq->body_len = cl;
            memmove(buf, buf + hl + cl, *have - hl - cl);
            *have -= hl + cl;
            return 0;
        }
        if (*have >= cap - 1) return -1;
        ssize_t k = recv(fd, buf + *have, cap - *have - 1, 0);
        if (k <= 0) return -1;
        *have += (size_t)k;
    }
}

static const int k_res[] = { 75, 100, 150, 200, 300, 400, 600, 800, 1200, 1600, 2400, 3200 };

static void capabilities(sbuf *b)
{
    sb_add(b, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<scan:ScannerCapabilities " NS ">\n"
              "<pwg:Version>2.63</pwg:Version>\n<pwg:MakeAndModel>%s</pwg:MakeAndModel>\n"
              "<pwg:SerialNumber>FS-V31</pwg:SerialNumber>\n<scan:UUID>%s</scan:UUID>\n"
              "<scan:AdminURI>http://%s:%d/</scan:AdminURI>\n"
              "<scan:Platen><scan:PlatenInputCaps>\n"
              "<scan:MinWidth>16</scan:MinWidth><scan:MaxWidth>292</scan:MaxWidth>\n"
              "<scan:MinHeight>16</scan:MinHeight><scan:MaxHeight>438</scan:MaxHeight>\n"
              "<scan:MaxScanRegions>1</scan:MaxScanRegions>\n<scan:SettingProfiles><scan:SettingProfile>\n"
              "<scan:ColorModes><scan:ColorMode>RGB24</scan:ColorMode><scan:ColorMode>RGB48</scan:ColorMode>"
              "<scan:ColorMode>Grayscale8</scan:ColorMode><scan:ColorMode>Grayscale16</scan:ColorMode></scan:ColorModes>\n"
              "<scan:DocumentFormats>",
           g.name, g.uuid, g.host, g.port);
    static const char *fmts[] = { "image/jpeg", "image/png", "image/tiff", "application/pdf" };
    for (int i = 0; i < 4; i++) sb_add(b, "<pwg:DocumentFormat>%s</pwg:DocumentFormat>", fmts[i]);
    for (int i = 0; i < 4; i++) sb_add(b, "<scan:DocumentFormatExt>%s</scan:DocumentFormatExt>", fmts[i]);
    sb_add(b, "</scan:DocumentFormats>\n<scan:SupportedResolutions><scan:DiscreteResolutions>");
    for (size_t i = 0; i < sizeof k_res / sizeof *k_res; i++)
        sb_add(b, "<scan:DiscreteResolution><scan:XResolution>%d</scan:XResolution><scan:YResolution>%d</scan:YResolution></scan:DiscreteResolution>",
               k_res[i], k_res[i]);
    sb_add(b, "</scan:DiscreteResolutions></scan:SupportedResolutions>\n"
              "<scan:ColorSpaces><scan:ColorSpace>RGB</scan:ColorSpace></scan:ColorSpaces>\n"
              "</scan:SettingProfile></scan:SettingProfiles>\n"
              "<scan:SupportedIntents><scan:Intent>Photo</scan:Intent><scan:Intent>Preview</scan:Intent>"
              "<scan:Intent>Document</scan:Intent><scan:Intent>TextAndGraphic</scan:Intent></scan:SupportedIntents>\n"
              "<scan:MaxOpticalXResolution>3200</scan:MaxOpticalXResolution>"
              "<scan:MaxOpticalYResolution>3200</scan:MaxOpticalYResolution>\n"
              "<scan:RiskyLeftMargin>0</scan:RiskyLeftMargin><scan:RiskyRightMargin>0</scan:RiskyRightMargin>"
              "<scan:RiskyTopMargin>0</scan:RiskyTopMargin><scan:RiskyBottomMargin>0</scan:RiskyBottomMargin>\n"
              "</scan:PlatenInputCaps></scan:Platen>\n</scan:ScannerCapabilities>\n");
}

static const char *job_state_name(int st)
{
    switch (st) {
    case 0: return "Pending";
    case 1: return "Processing";
    case 2: return "Completed";
    case 4: return "Canceled";
    }
    return "Aborted";
}

static void status_xml(sbuf *b)
{
    sb_add(b, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<scan:ScannerStatus " NS ">\n"
              "<pwg:Version>2.63</pwg:Version>\n<pwg:State>%s</pwg:State>\n<scan:Jobs>\n",
           g.busy ? "Processing" : "Idle");
    time_t now = time(NULL);
    for (int i = 0; i < MAX_JOBS; i++) {
        job_t *j = &g.jobs[i];
        if (!j->used) continue;
        sb_add(b, "<scan:JobInfo><pwg:JobUri>/eSCL/ScanJobs/%s</pwg:JobUri><pwg:JobUuid>%s</pwg:JobUuid>"
                  "<scan:Age>%ld</scan:Age><pwg:ImagesCompleted>%d</pwg:ImagesCompleted>"
                  "<pwg:ImagesToTransfer>%d</pwg:ImagesToTransfer><pwg:JobState>%s</pwg:JobState>"
                  "<pwg:JobStateReasons><pwg:JobStateReason>%s</pwg:JobStateReason></pwg:JobStateReasons></scan:JobInfo>\n",
               j->id, j->id, (long)(now - j->created), j->state == 2 ? 1 : 0,
               (j->state == 2 && !j->delivered) ? 1 : 0, job_state_name(j->state),
               j->state == 2 ? "JobCompletedSuccessfully" : j->state == 1 ? "JobScanning" : j->state == 4 ? "JobCanceledByUser" : "None");
    }
    sb_add(b, "</scan:Jobs>\n</scan:ScannerStatus>\n");
}

static job_t *find_job(const char *id)
{
    for (int i = 0; i < MAX_JOBS; i++)
        if (g.jobs[i].used && !strcmp(g.jobs[i].id, id)) return &g.jobs[i];
    return NULL;
}

static job_t *new_job(void)
{
    job_t *oldest = NULL;
    for (int i = 0; i < MAX_JOBS; i++) {
        if (!g.jobs[i].used) return &g.jobs[i];
        if (g.jobs[i].state >= 2 && !g.jobs[i].thread_active &&
            (!oldest || g.jobs[i].created < oldest->created))
            oldest = &g.jobs[i];
    }
    if (oldest) { free(oldest->doc); memset(oldest, 0, sizeof *oldest); }
    return oldest;
}

static void post_scan_job(int fd, const request *rq)
{
    char v[64];
    int dpi = 300;
    if (xml_get(rq->body, "XResolution", v, sizeof v)) dpi = atoi(v);
    char mode[32] = "RGB24";
    xml_get(rq->body, "ColorMode", mode, sizeof mode);
    char fmt[32] = "image/jpeg";
    if (!xml_get(rq->body, "DocumentFormatExt", fmt, sizeof fmt)) xml_get(rq->body, "DocumentFormat", fmt, sizeof fmt);
    if (strcmp(fmt, "image/jpeg") && strcmp(fmt, "image/png") && strcmp(fmt, "image/tiff") && strcmp(fmt, "application/pdf"))
        strcpy(fmt, "image/jpeg");
    char intent[32] = "";
    xml_get(rq->body, "Intent", intent, sizeof intent);
    long w = 0, h = 0, x = 0, y = 0;
    if (xml_get(rq->body, "Width", v, sizeof v)) w = atol(v);
    if (xml_get(rq->body, "Height", v, sizeof v)) h = atol(v);
    if (xml_get(rq->body, "XOffset", v, sizeof v)) x = atol(v);
    if (xml_get(rq->body, "YOffset", v, sizeof v)) y = atol(v);
    if (dpi < 50) dpi = 50;
    if (dpi > 3200) dpi = 3200;
    /* clamp the client's region (1/300 inch) to one 35 mm frame */
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > 288) x = 288;
    if (y > 434) y = 434;
    if (w < 0 || w > 292 - x) w = 292 - x;
    if (h < 0 || h > 438 - y) h = 438 - y;

    pthread_mutex_lock(&g.lock);
    job_t *j = new_job();
    if (!j) {
        pthread_mutex_unlock(&g.lock);
        respond(fd, rq, 503, "Service Unavailable", NULL, NULL, 0, NULL);
        return;
    }
    memset(j, 0, sizeof *j);
    j->used = 1;
    make_uuid(j->id);
    j->created = time(NULL);
    snprintf(j->format, sizeof j->format, "%s", fmt);
    j->gray = !strncmp(mode, "Grayscale", 9);
    j->bits16 = strstr(mode, "48") || strstr(mode, "16");
    dsd4_default_params(&j->p);
    j->p.film = g.film;
    j->p.frame = g.frame;
    j->p.focus = g.autofocus ? DSD4_FOCUS_AUTO : g.focus;
    j->p.multi_sample = g.samples;
    j->p.af_mode = g.af_mode;
    j->p.bits = 16;
    /* region in 1/300 inch -> pixels at the requested resolution */
    const long fw = 292, fh = 438;
    j->p.preview = !strcmp(intent, "Preview") && dpi <= 800;
    j->p.dpi = dpi;
    if (w > 0 && h > 0 && !(x == 0 && y == 0 && w >= fw && h >= fh)) {
        j->p.x = (int)(x * dpi / 300);
        j->p.y = (int)(y * dpi / 300);
        j->p.w = (int)(w * dpi / 300);
        j->p.h = (int)(h * dpi / 300);
        j->p.preview = 0;
    }
    if (g.ev != 0) {
        double std[3];
        dsd4_film_default_exposure(g.film, std);
        for (int c = 0; c < 3; c++) j->p.exposure[c] = std[c] * pow(2.0, g.ev);
    }
    char loc[512];
    snprintf(loc, sizeof loc, "Location: http://%s:%d/eSCL/ScanJobs/%s\r\n", g.host, g.port, j->id);
    logmsg("new scan job %s: %d dpi %s %s%s", j->id, dpi, mode, fmt, j->p.preview ? " (preview)" : "");
    pthread_t th;
    j->thread_active = 1;
    if (pthread_create(&th, NULL, job_thread, j) != 0) {
        j->thread_active = 0;
        j->state = 3;
    } else {
        pthread_detach(th);
    }
    pthread_mutex_unlock(&g.lock);
    respond(fd, rq, 201, "Created", NULL, NULL, 0, loc);
}

static void next_document(int fd, const request *rq, const char *id)
{
    pthread_mutex_lock(&g.lock);
    job_t *j = find_job(id);
    if (!j) { pthread_mutex_unlock(&g.lock); respond(fd, rq, 404, "Not Found", NULL, NULL, 0, NULL); return; }
    while (j->used && !strcmp(j->id, id) && j->state < 2) pthread_cond_wait(&g.cond, &g.lock);
    if (!j->used || strcmp(j->id, id) || j->state != 2 || j->delivered) {
        pthread_mutex_unlock(&g.lock);
        respond(fd, rq, 404, "Not Found", NULL, NULL, 0, NULL);
        return;
    }
    j->delivered = 1;
    uint8_t *doc = j->doc;
    size_t len = j->doc_len;
    char fmt[32];
    snprintf(fmt, sizeof fmt, "%s", j->format);
    j->doc = NULL; /* ownership moves to this response */
    pthread_mutex_unlock(&g.lock);
    respond(fd, rq, 200, "OK", fmt, doc, len, NULL);
    free(doc);
}

/* ---------------------------------------------------------- web page */

static void url_decode(char *s)
{
    char *o = s;
    for (; *s; s++) {
        if (*s == '+') *o++ = ' ';
        else if (*s == '%' && s[1] && s[2]) { char h[3] = { s[1], s[2], 0 }; *o++ = (char)strtol(h, NULL, 16); s += 2; }
        else *o++ = *s;
    }
    *o = 0;
}

static const char *form_get(char *body, const char *key, char *out, size_t n)
{
    size_t kl = strlen(key);
    for (char *p = body; p && *p; p = strchr(p, '&') ? strchr(p, '&') + 1 : NULL) {
        if (!strncmp(p, key, kl) && p[kl] == '=') {
            const char *v = p + kl + 1;
            size_t k = strcspn(v, "&");
            if (k >= n) k = n - 1;
            memcpy(out, v, k);
            out[k] = 0;
            url_decode(out);
            return out;
        }
    }
    return NULL;
}

static void ui_page(sbuf *b, const char *msg)
{
    dsd4_holder_info h;
    memset(&h, 0, sizeof h);
    const char *conn = "";
    if (pthread_mutex_trylock(&g.scanner_lock) == 0) {
        if (g.shared && !dsd4_usb_present()) conn = " &middot; <b>scanner not connected</b>";
        else if (acquire() == DSD4_OK) dsd4_get_holder(g.s, &h);
        else conn = " &middot; <b>scanner busy (in use by another program?)</b>";
        release();
        pthread_mutex_unlock(&g.scanner_lock);
    }
    static const char *hn[] = { "no holder", "35 mm film strip", "35 mm slide mount", "APS adapter", "unknown" };
    static const char *fn[] = { "Colour positive (slide)", "Colour negative", "B&amp;W positive", "B&amp;W negative" };
    sb_add(b, "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width'>"
              "<title>%s</title><style>body{font:15px -apple-system,sans-serif;max-width:560px;margin:24px auto;padding:0 16px;"
              "color:#222;background:#fafafa}@media(prefers-color-scheme:dark){body{color:#ddd;background:#1e1e1e}"
              "fieldset{border-color:#444}}fieldset{border:1px solid #ccc;border-radius:8px;margin:12px 0;padding:12px}"
              "label{display:block;margin:6px 0}button{font:inherit;padding:6px 14px;margin:4px 4px 0 0}"
              ".msg{padding:8px;border-radius:6px;background:#2a7a2a22}</style></head><body>"
              "<h2>%s</h2><p>Status: <b>%s</b> &middot; Holder: <b>%s</b>%s</p>",
           g.name, g.name, g.status, hn[h.holder], conn);
    if (msg && *msg) sb_add(b, "<p class=msg>%s</p>", msg);
    sb_add(b, "<form method=post action=/settings><fieldset><legend>Settings for the next scan</legend><label>Film "
              "<select name=film>");
    for (int i = 0; i < 4; i++) sb_add(b, "<option value=%d%s>%s</option>", i, (int)g.film == i ? " selected" : "", fn[i]);
    sb_add(b, "</select></label><label>Frame <select name=frame><option value=0%s>current</option>", g.frame ? "" : " selected");
    int nf = h.frames ? h.frames : 6;
    for (int i = 1; i <= nf; i++) sb_add(b, "<option%s>%d</option>", g.frame == i ? " selected" : "", i);
    sb_add(b, "</select></label>"
              "<label><input type=checkbox name=af value=1%s> Autofocus before each scan</label>"
              "<label>Autofocus <select name=afmode><option value=0%s>fast</option>"
              "<option value=1%s>classic (original sequence)</option></select></label>"
              "<label>Manual focus position <input name=focus size=5 value=%d></label>"
              "<label>Exposure correction (EV) <input name=ev size=5 value=%.2f></label>"
              "<label>Multi-sampling <select name=samples>",
           g.autofocus ? " checked" : "", g.af_mode == DSD4_AF_CLASSIC ? "" : " selected",
           g.af_mode == DSD4_AF_CLASSIC ? " selected" : "", g.focus, g.ev);
    static const int ms[] = { 1, 2, 4, 8 };
    for (int i = 0; i < 4; i++) sb_add(b, "<option%s>%d</option>", g.samples == ms[i] ? " selected" : "", ms[i]);
    sb_add(b, "</select></label><label>Negatives <select name=invert><option value=0%s>raw (invert later)</option>"
              "<option value=1%s>simple invert</option><option value=2%s>Minolta processing</option></select></label>"
              "<button>Save</button></fieldset></form>"
              "<form method=post action=/action><fieldset><legend>Scanner</legend>"
              "<button name=do value=eject>Eject holder</button><button name=do value=home>Return carriage</button>"
              "<button name=do value=autofocus>Autofocus now</button><button name=do value=calibrate>Recalibrate</button>"
              "</fieldset></form><p>Scan from <b>Image Capture</b> or <b>Preview &rarr; File &rarr; Import from Scanner</b>. "
              "Scanner area: 0.97 &times; 1.46 in (one 35 mm frame). The settings above apply to every scan.</p>"
              "<p>Raw DNG and HDR scans are made from the dsd4 menu or command line (Image Capture cannot receive DNG).</p></body></html>",
           g.invert == 0 ? " selected" : "", g.invert == 1 ? " selected" : "", g.invert == 2 ? " selected" : "");
}

static void handle_ui(int fd, const request *rq)
{
    char msg[256] = "";
    if (!strcmp(rq->method, "POST") && !strcmp(rq->path, "/settings") && rq->body) {
        char v[64];
        pthread_mutex_lock(&g.lock);
        if (form_get(rq->body, "film", v, sizeof v)) g.film = (dsd4_film)atoi(v);
        if (form_get(rq->body, "frame", v, sizeof v)) g.frame = atoi(v);
        g.autofocus = form_get(rq->body, "af", v, sizeof v) != NULL;
        if (form_get(rq->body, "invert", v, sizeof v)) g.invert = atoi(v);
        if (form_get(rq->body, "focus", v, sizeof v)) g.focus = atoi(v);
        if (form_get(rq->body, "ev", v, sizeof v)) g.ev = atof(v);
        if (form_get(rq->body, "samples", v, sizeof v)) g.samples = atoi(v);
        if (form_get(rq->body, "afmode", v, sizeof v)) g.af_mode = atoi(v) ? DSD4_AF_CLASSIC : DSD4_AF_FAST;
        pthread_mutex_unlock(&g.lock);
        snprintf(msg, sizeof msg, "Settings saved.");
    } else if (!strcmp(rq->method, "POST") && !strcmp(rq->path, "/action") && rq->body) {
        char v[32] = "";
        form_get(rq->body, "do", v, sizeof v);
        pthread_mutex_lock(&g.scanner_lock);
        int r = acquire();
        if (r) ;
        else if (!strcmp(v, "eject")) r = dsd4_eject(g.s);
        else if (!strcmp(v, "home")) r = dsd4_home(g.s);
        else if (!strcmp(v, "calibrate")) {
            uint8_t st = 3;
            g.s->base_state = 3;
            avl_check_scanner_state(g.s, 1, &st);
            r = dsd4_initialize(g.s, 0, NULL, NULL);
        } else if (!strcmp(v, "autofocus")) {
            dsd4_scan_params p;
            dsd4_default_params(&p);
            p.film = g.film;
            p.frame = g.frame;
            p.af_mode = g.af_mode;
            int best = 0;
            if (g.frame) r = dsd4_select_frame(g.s, g.frame);
            if (!r) r = dsd4_autofocus(g.s, &p, &best, NULL, NULL);
            if (!r) { g.focus = best; snprintf(msg, sizeof msg, "Best focus position: %d", best); }
        }
        int code = scanner_code();
        release();
        pthread_mutex_unlock(&g.scanner_lock);
        if (r) snprintf(msg, sizeof msg, "Failed: %s (scanner code 0x%02x)", dsd4_strerror(r), code);
        else if (!*msg) snprintf(msg, sizeof msg, "Done.");
    }
    sbuf b = { 0 };
    ui_page(&b, msg);
    respond(fd, rq, 200, "OK", "text/html; charset=utf-8", b.p, b.n, NULL);
    free(b.p);
}

static void *conn_thread(void *arg)
{
    int fd = (int)(intptr_t)arg;
    size_t cap = 1 << 16, have = 0;
    char *buf = malloc(cap);
    request rq;
    while (read_request(fd, &rq, buf, cap, &have) == 0) {
        sbuf b = { 0 };
        if (!strcmp(rq.method, "GET") && !strcmp(rq.path, "/eSCL/ScannerCapabilities")) {
            capabilities(&b);
            respond(fd, &rq, 200, "OK", "text/xml", b.p, b.n, NULL);
        } else if (!strcmp(rq.method, "GET") && !strcmp(rq.path, "/eSCL/ScannerStatus")) {
            pthread_mutex_lock(&g.lock);
            status_xml(&b);
            pthread_mutex_unlock(&g.lock);
            respond(fd, &rq, 200, "OK", "text/xml", b.p, b.n, NULL);
        } else if (!strcmp(rq.method, "POST") && !strcmp(rq.path, "/eSCL/ScanJobs")) {
            post_scan_job(fd, &rq);
        } else if (!strncmp(rq.path, "/eSCL/ScanJobs/", 15)) {
            char id[64] = "";
            const char *p = rq.path + 15;
            size_t k = strcspn(p, "/");
            if (k < sizeof id) { memcpy(id, p, k); id[k] = 0; }
            const char *rest = p + k;
            if (!strcmp(rq.method, "GET") && !strcmp(rest, "/NextDocument")) {
                next_document(fd, &rq, id);
            } else if (!strcmp(rq.method, "DELETE") && !*rest) {
                pthread_mutex_lock(&g.lock);
                job_t *j = find_job(id);
                if (j && j->state < 2) j->state = 4;
                pthread_cond_broadcast(&g.cond);
                pthread_mutex_unlock(&g.lock);
                respond(fd, &rq, j ? 200 : 404, j ? "OK" : "Not Found", NULL, NULL, 0, NULL);
            } else if (!strcmp(rq.method, "GET") && !strcmp(rest, "/ScanImageInfo")) {
                sb_add(&b, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<scan:ScanImageInfo " NS ">"
                           "<pwg:JobUri>/eSCL/ScanJobs/%s</pwg:JobUri></scan:ScanImageInfo>\n", id);
                respond(fd, &rq, 200, "OK", "text/xml", b.p, b.n, NULL);
            } else {
                respond(fd, &rq, 404, "Not Found", NULL, NULL, 0, NULL);
            }
        } else if (!strcmp(rq.path, "/") || !strcmp(rq.path, "/settings") || !strcmp(rq.path, "/action")) {
            handle_ui(fd, &rq);
        } else {
            respond(fd, &rq, 404, "Not Found", NULL, NULL, 0, NULL);
        }
        free(b.p);
        free(rq.body);
        if (!rq.keep_alive) break;
    }
    free(buf);
    close(fd);
    return NULL;
}

static void DNSSD_API reg_reply(DNSServiceRef ref, DNSServiceFlags flags, DNSServiceErrorType err, const char *name,
                                const char *type, const char *domain, void *ctx)
{
    if (err) logmsg("Bonjour registration failed (%d)", err);
    else logmsg("advertised as \"%s\" (%s%s) - it should now appear in Image Capture and Preview", name, type, domain);
}


static DNSServiceRef bonjour_register(int local_only, const char *admin, const char *uuidtxt)
{
    TXTRecordRef txt;
    TXTRecordCreate(&txt, 0, NULL);
#define TXT(k, v) TXTRecordSetValue(&txt, k, (uint8_t)strlen(v), v)
    TXT("txtvers", "1");
    TXT("vers", "2.63");
    TXT("rs", "eSCL");
    TXT("ty", g.name);
    TXT("pdl", "image/jpeg,image/png,image/tiff,application/pdf");
    TXT("cs", "color,grayscale");
    TXT("is", "platen");
    TXT("duplex", "F");
    TXT("uuid", uuidtxt);
    TXT("adminurl", admin);
    TXT("note", "film scanner (dsd4)");
#undef TXT
    DNSServiceRef sd = NULL;
    DNSServiceErrorType e = DNSServiceRegister(&sd, 0, local_only ? kDNSServiceInterfaceIndexLocalOnly : 0,
                                               g.name, "_uscan._tcp", NULL, NULL, htons((uint16_t)g.port),
                                               TXTRecordGetLength(&txt), TXTRecordGetBytesPtr(&txt), reg_reply, NULL);
    TXTRecordDeallocate(&txt);
    if (e) {
        logmsg("Bonjour registration failed (%d); clients must be pointed at %s", e, admin);
        return NULL;
    }
    return sd;
}

int dsd4_escl_serve(dsd4 *s, const dsd4_escl_options *o)
{
    memset(&g, 0, sizeof g);
    g.s = s;
    g.shared = s == NULL;
    g.on_open = o->on_open;
    g.ud = o->ud;
    g.port = o->port ? o->port : 8090;
    g.log = o->log;
    g.icc = o->icc;
    g.icc_len = o->icc_len;
    g.icc_n = o->icc_n;
    g.icc_n_len = o->icc_n_len;
    snprintf(g.name, sizeof g.name, "%s", o->name ? o->name : "DiMAGE Scan Dual IV");
    gethostname(g.host, sizeof g.host);
    if (!strchr(g.host, '.')) strncat(g.host, ".local", sizeof g.host - strlen(g.host) - 1);
    if (o->localhost_only) snprintf(g.host, sizeof g.host, "localhost");
    make_uuid(g.uuid);
    g.film = o->film;
    g.autofocus = 1;
    g.invert = 2; /* negatives: Minolta processing */
    g.samples = 1;
    snprintf(g.status, sizeof g.status, "idle");
    pthread_mutex_init(&g.lock, NULL);
    pthread_mutex_init(&g.scanner_lock, NULL);
    pthread_cond_init(&g.cond, NULL);
    signal(SIGPIPE, SIG_IGN);

    int ls = socket(AF_INET6, SOCK_STREAM, 0);
    int one = 1, zero = 0;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    setsockopt(ls, IPPROTO_IPV6, IPV6_V6ONLY, &zero, sizeof zero);
    struct sockaddr_in6 a = { .sin6_family = AF_INET6, .sin6_port = htons((uint16_t)g.port) };
    a.sin6_addr = o->localhost_only ? in6addr_loopback : in6addr_any;
    if (bind(ls, (struct sockaddr *)&a, sizeof a) < 0 || listen(ls, 16) < 0) {
        logmsg("cannot listen on port %d: %s", g.port, strerror(errno));
        close(ls);
        return -1;
    }

    char admin[300], uuidtxt[48];
    snprintf(admin, sizeof admin, "http://%s:%d/", g.host, g.port);
    snprintf(uuidtxt, sizeof uuidtxt, "%s", g.uuid);
    logmsg("eSCL server listening on port %d (settings page: %s)", g.port, admin);
    if (g.shared) logmsg("shared mode: the scanner is opened only while scanning");

    DNSServiceRef sd = NULL;
    int present = -1;
    for (;;) {
        /* advertise to Image Capture only while the scanner is attached */
        int now = g.shared ? dsd4_usb_present() : 1;
        if (now != present) {
            present = now;
            if (present && !sd) sd = bonjour_register(o->localhost_only, admin, uuidtxt);
            else if (!present && sd) {
                DNSServiceRefDeallocate(sd);
                sd = NULL;
                logmsg("scanner disconnected: no longer advertised");
            }
        }
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(ls, &rf);
        int maxfd = ls, dfd = sd ? DNSServiceRefSockFD(sd) : -1;
        if (dfd >= 0) { FD_SET(dfd, &rf); if (dfd > maxfd) maxfd = dfd; }
        struct timeval tv = { 3, 0 };
        if (select(maxfd + 1, &rf, NULL, NULL, &tv) < 0) { if (errno == EINTR) continue; break; }
        if (dfd >= 0 && FD_ISSET(dfd, &rf)) DNSServiceProcessResult(sd);
        if (FD_ISSET(ls, &rf)) {
            int c = accept(ls, NULL, NULL);
            if (c < 0) continue;
            setsockopt(c, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
            pthread_t th;
            pthread_create(&th, NULL, conn_thread, (void *)(intptr_t)c);
            pthread_detach(th);
        }
    }
    if (sd) DNSServiceRefDeallocate(sd);
    close(ls);
    return 0;
}
