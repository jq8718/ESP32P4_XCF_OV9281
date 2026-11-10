#ifndef XCR_DECODER_H
#define XCR_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XCR_BARCODE_MAX_LEN 256

typedef enum {
    XCR_OK = 0,
    XCR_NO_RESULT,
    XCR_TIMEOUT_NO_RESULT,
    XCR_TIMEOUT_WITH_VALID_RESULT,
    XCR_CANCELLED,
    XCR_OOM,
    XCR_INVALID_ARGUMENT,
    XCR_INTERNAL_ERROR,
} XcrStatus;

typedef enum {
    XCR_SYM_UNKNOWN = 0,
    XCR_SYM_UPC_EAN = 1,
    XCR_SYM_CODE128 = 2,
    XCR_SYM_QR = 3,
    XCR_SYM_DATAMATRIX = 4,
} XcrSymbology;

typedef enum {
    XCR_COORDINATE_DECODE_ROI = 0,
    XCR_COORDINATE_SENSOR = 1,
    XCR_COORDINATE_LCD = 2,
} XcrCoordinateSpace;

enum {
    XCR_VALIDATION_CHECKSUM = 1U << 0,
    XCR_VALIDATION_FORMAT = 1U << 1,
    XCR_VALIDATION_LENGTH = 1U << 2,
};

typedef struct {
    int width;
    int height;
    int stride;
    const uint8_t *pixels;
    int roi_x;
    int roi_y;
    int roi_width;
    int roi_height;
    uint32_t sequence;
    uint32_t capture_generation;
    uint64_t timestamp_us;
    uint64_t deadline_us;
} XcrGrayImage;

typedef struct {
    char data[XCR_BARCODE_MAX_LEN];
    int length;
    int confidence;
    int symbology;
    int corners[8];
    int geometry_valid;
    int geometry_quality;
    int grid_rows;
    int grid_cols;
    int geometry_seed_valid;
    int geometry_seed[8];
    unsigned geometry_edge_mask;
    unsigned geometry_support;
    float geometry_residual;
    int subpixel_valid;
    float subpixel_corners[8];
    int coordinate_space;
    int roi_x;
    int roi_y;
    int roi_width;
    int roi_height;
    int direction;
    int scan_y;
    uint32_t sequence;
    uint32_t capture_generation;
    uint64_t timestamp_us;
    int failure_stage;
    int result_owner;
    uint64_t dedup_key;
    uint32_t validation_flags;
    XcrStatus status;
} XcrBarcodeResult;

typedef struct {
    bool enable_code128;
    bool enable_upc_ean;
    bool enable_qr;
    bool enable_datamatrix;
    bool polarity_retry;
    bool cross_symbology_veto;
    bool reject_ambiguous_retail_numeric;
    uint32_t decoder_timeout_ms;
    uint32_t image_budget_ms;
    uint8_t code128_direction_attempts;
    bool code128_fast_rows;
    bool code128_fast_rows_only;
    bool qr_quirc_only;
    bool qr_fast_search;
    bool datamatrix_defer_geometry;
} XcrDecoderConfig;

typedef struct {
    uint64_t decode_calls;
    uint64_t successful_calls;
    uint64_t no_result_calls;
    uint64_t timeout_calls;
    uint64_t cancelled_calls;
    uint64_t oom_calls;
    uint64_t total_decode_us;
    uint64_t max_decode_us;
} XcrDecoderStats;

typedef enum {
    XCR_QR_STAGE_NONE = 0,
    XCR_QR_STAGE_CENTER_8X,
    XCR_QR_STAGE_FULL_8X,
    XCR_QR_STAGE_CANDIDATE_4X,
    XCR_QR_STAGE_CANDIDATE_2X,
    XCR_QR_STAGE_FULL_4X,
    XCR_QR_STAGE_FULL_2X,
    XCR_QR_STAGE_FULL_4X_ADAPTIVE,
} XcrQrStage;

typedef struct {
    XcrQrStage stage;
    uint8_t mirror;
    uint8_t candidate_count;
    uint32_t full8_sample_us;
    uint32_t full8_identify_us;
    uint32_t full8_decode_us;
    uint32_t candidate_sample_us;
    uint32_t candidate_identify_us;
    uint32_t candidate_decode_us;
    uint32_t fallback_sample_us;
    uint32_t fallback_identify_us;
    uint32_t fallback_decode_us;
    uint32_t full8_refine_us;
    uint32_t candidate_refine_us;
    uint32_t fallback_refine_us;
    int corners[8];
    uint8_t geometry_valid;
    uint8_t sample_scale;
    uint8_t grid_size;
    uint8_t transposed;
    uint8_t finder_count;
    uint8_t incomplete_count;
    uint32_t geometry_refine_us;
    uint8_t geometry_refined;
} XcrQrTrace;

typedef struct XcrDecoder XcrDecoder;

void xcr_decoder_config_default(XcrDecoderConfig *config);
XcrDecoder *xcr_decoder_create(const XcrDecoderConfig *config);
void xcr_decoder_destroy(XcrDecoder *decoder);

XcrStatus xcr_decoder_decode(XcrDecoder *decoder,
                             const XcrGrayImage *image,
                             XcrBarcodeResult *results,
                             int max_results,
                             int *out_result_count);

void xcr_decoder_cancel(XcrDecoder *decoder);
void xcr_decoder_clear_cancel(XcrDecoder *decoder);
void xcr_decoder_get_stats(const XcrDecoder *decoder, XcrDecoderStats *out);
void xcr_decoder_get_qr_trace(const XcrDecoder *decoder, XcrQrTrace *out);
const char *xcr_qr_stage_name(XcrQrStage stage);
const char *xcr_status_name(XcrStatus status);

#ifdef __cplusplus
}
#endif

#endif
