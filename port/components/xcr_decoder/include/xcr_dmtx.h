#ifndef XCR_DMTX_H
#define XCR_DMTX_H
#include <stdint.h>
#include "xcr_decoder.h"
enum { XCR_GEOMETRY_NONE, XCR_GEOMETRY_COARSE, XCR_GEOMETRY_REFINED };
int xcr_datamatrix_refine(const XcrGrayImage *image, XcrBarcodeResult *result);
typedef struct {
    float angle_degrees;
    uint64_t setup_us;
    uint64_t normalize_pass_us[3];
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
    unsigned marker_sample_step;
    int marker_roi_valid;
    int marker_roi[4];
} XcrDmtxStats;
void xcr_dmtx_stats_get(XcrDmtxStats *out);
int xcr_dmtx_parallel_start(void);
/* Per-frame state owned by the camera slot. No previous-frame position hint. */
typedef struct {
    const uint8_t *pixels;
    uint32_t sequence;
    int width, height;
    int found;
    int roi[4];
    int orientation;
    int sample_step; /* Axes and normalization must use the same half-step offset. */
    float axes[6];
    unsigned qr_rejected;
    uint64_t search_us;
} XcrDmtxPrepared;
int xcr_dmtx_prepare(const XcrGrayImage *image, XcrDmtxPrepared *prepared);
XcrStatus xcr_decoder_decode_prepared(XcrDecoder *decoder, const XcrGrayImage *image,
    const XcrDmtxPrepared *prepared, XcrBarcodeResult *results, int max_results,
    int *out_result_count);
#endif
