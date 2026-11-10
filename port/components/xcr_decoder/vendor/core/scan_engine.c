#include "scan_engine.h"
#include "scan_primitives.h"

#include <limits.h>
#include <string.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

static int div_pct_200(int pct, int extent)
{
    long long v = (long long)pct * (long long)(extent - 1);
    return (int)(v / 200LL);
}

static int clamp_i32(int v, int lo, int hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

int scan_engine_fmwlsymbol0040(
    const ScanEngineFrame *frame,
    const ScanEngineAnchorConfig *cfg,
    ScanEngineAnchor *out)
{
    int x;
    int y;
    int major_pct;
    int minor_pct;

    if (!frame || !cfg || !out || frame->width <= 0 || frame->height <= 0) {
        return 0;
    }

    if (!cfg->enabled) {
        out->x = frame->center_x;
        out->y = frame->center_y;
        return 1;
    }

    switch (frame->orientation_mode) {
    case 2:
        major_pct = 200 - cfg->pct_60 - cfg->pct_56;
        minor_pct = 200 - cfg->pct_68 - cfg->pct_64;
        break;
    case 1:
        major_pct = 200 - cfg->pct_68 - cfg->pct_64;
        minor_pct = cfg->pct_60 + cfg->pct_56;
        break;
    case 0:
        major_pct = cfg->pct_60 + cfg->pct_56;
        minor_pct = cfg->pct_68 + cfg->pct_64;
        break;
    default:
        major_pct = cfg->pct_68 + cfg->pct_64;
        minor_pct = 200 - cfg->pct_60 - cfg->pct_56;
        break;
    }

    x = div_pct_200(major_pct, frame->width);
    y = div_pct_200(minor_pct, frame->height);

    if (y == 0) {
        x = frame->center_x - (frame->width >> 1) + x;
        y = frame->center_y - (frame->height >> 1) + y;
        x = clamp_i32(x, 0, frame->width - 1);
        y = clamp_i32(y, 0, frame->height - 1);
    }

    out->x = x;
    out->y = y;
    return 1;
}

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
    ScanEngineRunFeatures *out_features)
{
    ScanLineIter it;
    int n;
    ptrdiff_t off;
    const uint8_t *p;
    int min_v;

    if (!img || !img->pixels || !samples || !turns || !out_features) {
        return 0;
    }
    if (img->width <= 0 || img->height <= 0 || img->stride <= 0) {
        return 0;
    }
    if (start_x < 0 || start_y < 0 || start_x >= img->width || start_y >= img->height) {
        return 0;
    }

    n = fmdclsymbol0049(&it, start_x, start_y, end_x, end_y, img->stride);
    if (n < 2 || (size_t)n > sample_cap || n > SCAN_ENGINE_MAX_SAMPLES) {
        return 0;
    }

    memset(turns, 0, sizeof(*turns));
    off = (ptrdiff_t)start_y * img->stride + start_x;
    p = img->pixels + off;

    samples[0] = *p;
    min_v = samples[0];

    if (n >= 2) {
        int step = AHEAD(&it);
        p += step;
        samples[1] = *p;
        if (samples[1] < min_v) {
            min_v = samples[1];
        }
    }

    for (int i = 2; i < n; ++i) {
        int step = AHEAD(&it);
        int prev2;
        int prev1;
        int cur;

        p += step;
        cur = *p;
        samples[i] = cur;
        if (cur < min_v) {
            min_v = cur;
        }

        prev2 = samples[i - 2];
        prev1 = samples[i - 1];

        if (((prev2 < prev1) && (prev1 >= cur)) || ((prev2 <= prev1) && (prev1 > cur))) {
            int x_new = i - 1;
            int y_new = prev1;

            if (turns->count == 0) {
                turns->x[turns->count] = x_new;
                turns->y[turns->count] = y_new;
                turns->count = 1;
            } else {
                while (turns->count >= 2) {
                    int i1 = turns->count - 1;
                    int i0 = turns->count - 2;
                    long long lhs = (long long)(turns->y[i1] - turns->y[i0]) * (x_new - turns->x[i1]);
                    long long rhs = (long long)(y_new - turns->y[i1]) * (turns->x[i1] - turns->x[i0]);
                    if (lhs < rhs) {
                        break;
                    }
                    turns->count--;
                }

                if (turns->count < SCAN_ENGINE_MAX_TURNS) {
                    turns->x[turns->count] = x_new;
                    turns->y[turns->count] = y_new;
                    turns->count++;
                }
            }
        }
    }

    if (n >= 2 && turns->count < SCAN_ENGINE_MAX_TURNS) {
        int x_new = n - 1;
        int y_new = samples[n - 1];

        while (turns->count >= 2) {
            int i1 = turns->count - 1;
            int i0 = turns->count - 2;
            long long lhs = (long long)(turns->y[i1] - turns->y[i0]) * (x_new - turns->x[i1]);
            long long rhs = (long long)(y_new - turns->y[i1]) * (turns->x[i1] - turns->x[i0]);
            if (lhs < rhs) {
                break;
            }
            turns->count--;
        }

        turns->x[turns->count] = x_new;
        turns->y[turns->count] = y_new;
        turns->count++;
    }

    if (out_min) {
        *out_min = min_v;
    }

    {
        int last = samples[0];
        int sign = 1;
        memset(out_features, 0, sizeof(*out_features));

        for (int i = 1; i < n; ++i) {
            int cur = samples[i];
            int is_turn = ((cur >= last) != (sign > 0));
            if (is_turn && out_features->count < SCAN_ENGINE_MAX_FEATURES) {
                int k = out_features->count++;
                out_features->from_x[k] = i - 1;
                out_features->to_x[k] = i;
                out_features->from_y[k] = last;
                out_features->to_y[k] = cur;
                out_features->min_value[k] = (cur < last) ? cur : last;
                if (out_features->min_value[k] < min_v) {
                    min_v = out_features->min_value[k];
                }
                sign = -sign;
            }
            last = cur;
        }

        for (int i = 0; i < turns->count && out_features->feature_count < SCAN_ENGINE_MAX_FEATURES; ++i) {
            out_features->features[out_features->feature_count++] = turns->x[i] - min_v;
        }
    }

    return n;
}

static int mod53(int v)
{
    int m = v % SCAN_ENGINE_HASH_MOD;
    if (m < 0) {
        m += SCAN_ENGINE_HASH_MOD;
    }
    return m;
}

int scan_engine_sub_18c2ac(ScanEngineLoopState *state, int seed_delta)
{
    enum {
        FIRST_HASH_PERIOD = 20,
        SECOND_HASH_PERIOD = 15,
    };
    int count;
    int first_hash = 0;
    int first_idx = 0;
    int second_hash = 0;
    int second_idx = 0;

    if (!state) {
        return 0;
    }

    count = state->history_count;
    if (count < 0 || count >= SCAN_ENGINE_HISTORY_CAP) {
        return 0;
    }

    if (count >= 3 && count < SCAN_ENGINE_MAX_TURNS) {
        first_hash = mod53(state->history[count]);
        first_idx = 1;

        if (first_hash != 0) {
            for (int i = 1; i < count; ++i) {
                int term = (i % FIRST_HASH_PERIOD) + 1;
                first_hash = mod53(first_hash + state->history[count - i] * term);
            }
            first_idx = count;
        }
    }

    if (first_hash != state->expected_hash[first_idx]) {
        return 0;
    }

    if (count >= 2 && count < SCAN_ENGINE_MAX_TURNS) {
        second_hash = mod53(state->history[count - 1] + seed_delta);
        second_idx = 1;

        if (second_hash != 0) {
            for (int i = 1; i < count - 1; ++i) {
                int term = (i % SECOND_HASH_PERIOD) + 1;
                second_hash = mod53(second_hash + state->history[count - 1 - i] * term);
            }
            second_idx = count - 1;
        }
    }

    if (second_hash == state->expected_hash[second_idx]
        && (!state->confirm_fn || state->confirm_fn(state->confirm_user, count - 2))) {
        state->quality_score = 11 * state->history_count + 20;
        state->status_code = (state->decoded_count < 3) ? 5 : 1;
        return 1;
    }

    return 0;
}
