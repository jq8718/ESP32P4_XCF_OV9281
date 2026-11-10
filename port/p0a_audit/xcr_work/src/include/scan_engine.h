#ifndef SCAN_ENGINE_H
#define SCAN_ENGINE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCAN_ENGINE_MAX_SAMPLES 4096
#define SCAN_ENGINE_MAX_TURNS 512
#define SCAN_ENGINE_HISTORY_CAP (SCAN_ENGINE_MAX_TURNS + 4)
#define SCAN_ENGINE_HASH_MOD 53
#define SCAN_ENGINE_MAX_FEATURES 512

typedef struct {
    int width;
    int height;
    int center_x;
    int center_y;
    uint8_t orientation_mode;
} ScanEngineFrame;

typedef struct {
    uint8_t enabled;
    int pct_56;
    int pct_60;
    int pct_64;
    int pct_68;
} ScanEngineAnchorConfig;

typedef struct {
    int x;
    int y;
} ScanEngineAnchor;

typedef struct {
    const uint8_t *pixels;
    int width;
    int height;
    int stride;
} ScanEngineImage;

typedef struct {
    int count;
    int x[SCAN_ENGINE_MAX_TURNS];
    int y[SCAN_ENGINE_MAX_TURNS];
} ScanEngineTurnList;

typedef struct {
    int count;
    int from_x[SCAN_ENGINE_MAX_FEATURES];
    int to_x[SCAN_ENGINE_MAX_FEATURES];
    int from_y[SCAN_ENGINE_MAX_FEATURES];
    int to_y[SCAN_ENGINE_MAX_FEATURES];
    int min_value[SCAN_ENGINE_MAX_FEATURES];
    int feature_count;
    int features[SCAN_ENGINE_MAX_FEATURES];
} ScanEngineRunFeatures;

typedef struct {
    int history_count;
    int history[SCAN_ENGINE_HISTORY_CAP];
    int expected_hash[SCAN_ENGINE_HISTORY_CAP];

    int decoded_count;
    int quality_score;
    int status_code;

    int (*confirm_fn)(void *user, int window_idx);
    void *confirm_user;
} ScanEngineLoopState;

int scan_engine_fmwlsymbol0040(
    const ScanEngineFrame *frame,
    const ScanEngineAnchorConfig *cfg,
    ScanEngineAnchor *out);

int scan_engine_fmwlsymbol0097(
    const ScanEngineImage *img,
    int start_x,
    int start_y,
    int end_x,
    int end_y,
    int *samples,
    size_t sample_cap,
    ScanEngineTurnList *turns,
    int *out_min,
    ScanEngineRunFeatures *out_features);

int scan_engine_sub_18c2ac(ScanEngineLoopState *state, int seed_delta);

#ifdef __cplusplus
}
#endif

#endif
