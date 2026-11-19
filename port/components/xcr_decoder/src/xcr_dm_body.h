#pragma once
#include "barcode.h"
#include "xcr_port.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    float candidate_px;
    unsigned gray_attempts;
    int binarizer; /* 0: prepared local contrast; 1: ROI Otsu; 2: ROI local. */
} XcrDmBodyTrace;
void *xcr_dm_body_prepare(const Image *image, int step, const XcrDeadline *deadline,
    uint64_t *threshold_us);
int xcr_dm_body_finish(const Image *image, BarcodeResult *result, int step,
    void *state, const XcrDeadline *deadline, XcrDmBodyTrace *trace);
void xcr_dm_body_discard(void *state);
int xcr_dm_body_decode(const Image *image, BarcodeResult *result, int step,
    const XcrDeadline *deadline, uint64_t *threshold_us);
#ifdef __cplusplus
}
#endif
