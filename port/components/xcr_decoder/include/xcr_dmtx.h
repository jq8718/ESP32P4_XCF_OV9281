#ifndef XCR_DMTX_H
#define XCR_DMTX_H
#include <stdint.h>
#include "xcr_decoder.h"
enum { XCR_GEOMETRY_NONE, XCR_GEOMETRY_COARSE, XCR_GEOMETRY_REFINED };
int xcr_datamatrix_refine(const XcrGrayImage *image, XcrBarcodeResult *result);
typedef struct {
    float angle_degrees;
    uint64_t setup_us;
    uint64_t normalize_pass_us[2];
    unsigned normalize_attempts;
    uint64_t find_us;
    uint64_t decode_us;
    uint64_t total_us;
    uint64_t prescreen_us;
    uint64_t geometry_us;
    unsigned geometry_refined;
    unsigned qr_rejected;
    unsigned scale;
    unsigned candidates;
    unsigned decoded_messages;
    unsigned geometry_fallbacks;
    unsigned hint_hits;
    uint64_t marker_core0_us;
    uint64_t marker_core1_us;
    unsigned marker_parallel_used;
    unsigned marker_full_fallback;
    int marker_roi_valid;
    int marker_roi[4];
} XcrDmtxStats;
void xcr_dmtx_stats_get(XcrDmtxStats *out);
int xcr_dmtx_parallel_start(void);
#endif
