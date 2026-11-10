#include "xcr_port.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

static void *xcr_qr_alloc(size_t size)
{
    return xcr_alloc(size, size <= 65536U ? XCR_MEM_FAST : XCR_MEM_LARGE);
}

static void *xcr_qr_calloc(size_t count, size_t size)
{
    return xcr_calloc(count, size,
                      count <= 65536U / (size ? size : 1U)
                          ? XCR_MEM_FAST : XCR_MEM_LARGE);
}

#undef malloc
#undef calloc
#undef free
#define malloc(size) xcr_qr_alloc((size))
#define calloc(count, size) xcr_qr_calloc((count), (size))
#define free(ptr) xcr_free((ptr))

#include "../../../../p0a_audit/xcr_work/src/symbologies/qrcode.c"
