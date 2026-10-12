/* The protocol simulator exported with a flat C API (for the PowerPC
 * emulation harness, which loads it with ctypes). */
#include "transport.h"

void *simx_open(const char *fw)
{
    char e[128];
    return dsd4_sim_open(fw, e, sizeof e);
}

int simx_exec(void *t, const uint8_t *cdb, int dir, uint8_t *data, size_t len, uint8_t *status)
{
    dsd4_transport *tr = t;
    return tr->exec(tr, cdb, (enum dsd4_dir)dir, data, len, status);
}

void simx_close(void *t)
{
    ((dsd4_transport *)t)->close(t);
}
