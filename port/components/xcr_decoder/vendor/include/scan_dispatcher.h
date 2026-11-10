#ifndef SCAN_DISPATCHER_H
#define SCAN_DISPATCHER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCAN_DISPATCHER_MAX_DIRS 16
#define SCAN_DISPATCHER_MAX_SAMPLES 2048
#define SCAN_DISPATCHER_MAX_TURNS 256

/* Result status mirrors the decompiled dispatcher's use of status codes at ctx+60. */
typedef enum {
    SCAN_STATUS_NONE = 0,
    SCAN_STATUS_OK = 1,
    SCAN_STATUS_RETRY_3 = 3,
    SCAN_STATUS_RETRY_4 = 4,
    SCAN_STATUS_RETRY_7 = 7,
    SCAN_STATUS_FATAL = 20,
} ScanStatus;

typedef struct {
    int x;
    int y;
} ScanPoint;

typedef struct {
    int from_x;
    int from_y;
    int to_x;
    int to_y;
    int min_value;
} ScanSegment;

typedef struct {
    int count;
    int values[SCAN_DISPATCHER_MAX_TURNS];
} ScanRunList;

typedef struct {
    int count;
    ScanSegment segments[SCAN_DISPATCHER_MAX_TURNS];
    int feature_count;
    int features[SCAN_DISPATCHER_MAX_TURNS];
} ScanRunFeatures;

typedef int (*ScanDecodeFn)(void *ctx, int line_idx, int left, int right);

typedef struct {
    /* Mirrors *(ctx+116)+offset fields consumed by SmartSymbolRedef0001c3. */
    int32_t off_68;
    int32_t off_72;
    int32_t off_76;
    int32_t off_80;
    int32_t off_84;
    int32_t off_88;
    int32_t off_104;
    int32_t off_112;
    int32_t off_120;
    int32_t off_124;
    int32_t off_128;
    int32_t off_132;
    int32_t off_136;
    int32_t off_144;
    int32_t off_148;
    int32_t off_152;
    int32_t off_156;
    int32_t off_164;
    int32_t off_192;
    int32_t off_196;
    int32_t off_200;
} ScanEnableConfig;

typedef struct {
    uint8_t enable_304; /* decompiled *(cfg+304) */
    uint8_t enable_305; /* decompiled *(cfg+305) */
    uint8_t enable_306; /* decompiled *(cfg+306) */

    int edge_count;
    const int *edge_positions;
    int window_left_index;   /* maps ctx+20216 candidate left edge index */
    int window_right_index;  /* maps ctx+20220 candidate right edge index */
    int process_line_arg;    /* mirrors ProcessResult(a2) */
    int process_left_arg;    /* mirrors ProcessResult(a3) */
    int process_right_arg;   /* mirrors ProcessResult(a4) */

    ScanStatus status;
    int fatal_status;

    ScanDecodeFn decode_fmwlsymbol0062;
    ScanDecodeFn decode_fmwlsymbol0061;
    ScanDecodeFn decode_fmwlsymbol0060;
    ScanDecodeFn decode_fmwlsymbol0065;
    ScanDecodeFn decode_fmwlsymbol005f;
    ScanDecodeFn decode_6016277b;
    ScanDecodeFn decode_6016294b;
    ScanDecodeFn decode_fmwlsymbol0064;

    void *decode_ctx;
} ScanDispatchContext;

typedef int (*ScanScoreFn)(void *ctx, int cx, int cy, int dir_idx);
typedef int (*ScanSampleFn)(void *ctx, int x, int y);

typedef struct {
    int center_x;
    int center_y;
    int stride_x;
    int stride_y;
    int sample_radius;
} ScanDirectionChoice;

typedef struct {
    int raw_scores[SCAN_DISPATCHER_MAX_DIRS];
    int weighted_scores[SCAN_DISPATCHER_MAX_DIRS];
    int best_dir;
    int best_weighted_score;
    int refined_dir;
    int refine_enabled;
    int max_score;
    int min_score;
    int axis_x_q8;
    int axis_y_q8;
    int score_sum;
    int score_mean_q4;
    int abs_neighbor_delta_sum;
    int dispersion_q12;
} ScanDirectionScoreChain;

typedef struct {
    int x;
    int y;
    int value;
} ScanSample;

typedef struct {
    size_t count;
    ScanSample samples[SCAN_DISPATCHER_MAX_SAMPLES];
} ScanLineBuffer;

typedef struct {
    int offset;
    uint32_t bit_mask;
    const char *barcode_name;
    const char *condition;
} ScanEnableBitMap;

typedef enum {
    SCAN_RETRY_STAGE_ROTATE_SUM = 0,
    SCAN_RETRY_STAGE_ROTATE_DIF = 1,
    SCAN_RETRY_STAGE_ROTATE2_SUM = 2,
    SCAN_RETRY_STAGE_ROTATE2_DIF = 3,
} ScanRetryStage;

#define SCAN_RETRY_MAX_STAGES 4

typedef struct {
    int count;
    ScanRetryStage stages[SCAN_RETRY_MAX_STAGES];
} ScanRetryPlan;

size_t scan_dispatcher_enable_map(const ScanEnableBitMap **out_map);

int scan_dispatcher_run_fmwlsymbol009a(ScanDispatchContext *ctx);

int scan_dispatcher_result_is_retryable(int status_code);

int scan_dispatcher_build_retry_plan(
    int status_code,
    int max_stages,
    ScanRetryPlan *out_plan);

int scan_dispatcher_hub_smartsymbol_0001c3(
    const ScanEnableConfig *cfg,
    int edge_count,
    int searching,
    int mode,
    ScanDecodeFn call,
    void *call_ctx);

int scan_dispatcher_sub_13927c(
    int axis_forward,
    const int *sample_begin,
    const int *sample_end,
    const ScanRunList *turn_list,
    int baseline_min,
    ScanRunFeatures *out);

int scan_dispatcher_choose_direction(
    int center_x,
    int center_y,
    int prefer_diagonal,
    ScanScoreFn scorer,
    void *score_ctx,
    ScanDirectionChoice *out);

int scan_dispatcher_score_direction_chain(
    int center_x,
    int center_y,
    int prefer_diagonal,
    ScanScoreFn scorer,
    void *score_ctx,
    ScanDirectionScoreChain *out);

int scan_dispatcher_build_scanline(
    const ScanDirectionChoice *choice,
    ScanSampleFn sampler,
    void *sample_ctx,
    ScanLineBuffer *out_line);

int scan_dispatcher_build_scanline_stitched(
    const ScanDirectionChoice *choice,
    ScanSampleFn sampler,
    void *sample_ctx,
    ScanLineBuffer *out_line,
    size_t *out_center_index);

#ifdef __cplusplus
}
#endif

#endif
