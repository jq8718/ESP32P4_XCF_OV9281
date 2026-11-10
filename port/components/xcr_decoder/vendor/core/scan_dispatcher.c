#include "scan_dispatcher.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

typedef struct {
    int x;
    int y;
    int error;
    int dx;
    int dy;
    int step_x;
    int step_y;
    int major_x;
    int pixel_step;
} ScanLineIterCompat;

int fmdclsymbol0049(void *it, int x0, int y0, int x1, int y1, int row_stride);
int AHEAD(void *it);

#define CFG_ENABLED(v) ((v) != -1)
#define RETRY_MASK 0x98u

/*
 * SmartSymbolRedef0001c3 enable map.
 * Names are inferred from decode_key/decode_core symbol configuration and call grouping.
 */
static const ScanEnableBitMap k_enable_map[] = {
    {68,  0xFFFFFFFFu, "Legacy Pair Decoder", "off_68 != -1 -> 000230/000221"},
    {68,  0x000000C0u, "Legacy Pair Decoder", "off_68 bit[6:7] alter primary pass gating"},

    {72,  0xFFFFFFFFu, "UPC/EAN Family", "off_72 != -1 -> 00007c/000079"},
    {72,  0x000000C0u, "UPC/EAN Family", "off_72 bit[6:7] affect strictness gates"},

    {76,  0xFFFFFFFFu, "Code128 / GS1-128", "off_76 != -1 -> 00009f/00009a"},
    {76,  0x00000001u, "Code128 / GS1-128", "bit0 set skips base pass"},
    {76,  0x00000018u, "Code128 / GS1-128", "bits[3:4]==0x8 skips base pass"},
    {76,  0x00000020u, "Code128 / GS1-128", "bit5 enables alternate mode=2 pass"},
    {76,  0x000000C0u, "Code128 / GS1-128", "bits[6:7] special gating"},
    {76,  0x08000000u, "Code128 / GS1-128", "bit27 contributes mode bit"},
    {76,  0x10000000u, "Code128 / GS1-128", "bit28 contributes mode bit"},

    {80,  0xFFFFFFFFu, "Linear Family Group A", "off_80 != -1 -> 0004f4/00008e/0004f3/000089"},
    {84,  0xFFFFFFFFu, "Composite/Linkage Group", "off_84 != -1 -> 000304A/000304/0002f7"},
    {88,  0xFFFFFFFFu, "Linear Family Group B", "off_88 != -1 -> 0000b1/0000ae"},

    {104, 0xFFFFFFFFu, "GS1 DataBar Family", "off_104 != -1 -> 000376/000373/000379 branches"},
    {104, 0x00000018u, "GS1 DataBar Family", "bits[3:4]==0x18 disables 000376/000377"},
    {104, 0x00000003u, "GS1 DataBar Family", "bits[1:0]==0x3 disables 000373/000374"},
    {104, 0x00000004u, "GS1 DataBar Family", "bit2 disables 000379/00037a"},

    {112, 0xFFFFFFFFu, "Linear Family Group C", "off_112 != -1 -> 000399/00039a"},
    {120, 0xFFFFFFFFu, "Linear Family Group D", "off_120 != -1 -> 000504/000502"},
    {124, 0xFFFFFFFFu, "Stacked Group 1", "off_124 != -1 -> 0005ef58..61"},
    {128, 0xFFFFFFFFu, "Stacked Group 2", "off_128 != -1 -> 0005ef54..57"},
    {132, 0xFFFFFFFFu, "Stacked Group 3", "off_132 != -1 -> 0005ff73..76"},
    {136, 0xFFFFFFFFu, "Linear Family Group E", "off_136 != -1 -> 00060156"},
    {144, 0xFFFFFFFFu, "Linear Family Group F", "off_144 != -1 -> 000568/000569"},
    {148, 0xFFFFFFFFu, "Linear Family Group G", "off_148 != -1 -> 0005ff33/0005ff34"},
    {152, 0xFFFFFFFFu, "Linear Family Group H", "off_152 != -1 -> 000601175/000601176"},
    {156, 0xFFFFFFFFu, "Linear Family Group I", "off_156 != -1 -> 0005ff93..96"},
    {164, 0xFFFFFFFFu, "Linear Family Group J", "off_164 != -1 -> 0005ef34..37"},

    {192, 0xFFFFFFFFu, "Composite Assist", "off_192 controls 0006016310B/11B mode"},
    {196, 0xFFFFFFFFu, "Composite Assist", "off_196 enables composite assist branch"},
    {200, 0xFFFFFFFFu, "Code128 Companion", "off_200 enables 000601577F"},
};

static const ScanRetryStage k_retry_order[SCAN_RETRY_MAX_STAGES] = {
    SCAN_RETRY_STAGE_ROTATE_SUM,
    SCAN_RETRY_STAGE_ROTATE_DIF,
    SCAN_RETRY_STAGE_ROTATE2_SUM,
    SCAN_RETRY_STAGE_ROTATE2_DIF,
};

static const int k_dir_tbl[SCAN_DISPATCHER_MAX_DIRS][2] = {
    {1, 0}, {2, 1}, {1, 1}, {1, 2},
    {0, 1}, {-1, 2}, {-1, 1}, {-2, 1},
    {-1, 0}, {-2, -1}, {-1, -1}, {-1, -2},
    {0, -1}, {1, -2}, {1, -1}, {2, -1},
};

static int choose_nearest_dir(int vx, int vy, int prefer_diagonal)
{
    int best_idx = -1;
    long best_dot = LONG_MIN;

    for (int i = 0; i < SCAN_DISPATCHER_MAX_DIRS; ++i) {
        long dot = (long)vx * k_dir_tbl[i][0] + (long)vy * k_dir_tbl[i][1];
        if (prefer_diagonal && (i & 1)) {
            dot += 1;
        }
        if (dot > best_dot) {
            best_dot = dot;
            best_idx = i;
        }
    }

    return best_idx;
}

static int scan_dispatcher_edge_pos_from_index(const ScanDispatchContext *ctx, int idx)
{
    if (!ctx || !ctx->edge_positions || ctx->edge_count <= 0) {
        return idx;
    }
    if (idx < 0) {
        idx = 0;
    } else if (idx >= ctx->edge_count) {
        idx = ctx->edge_count - 1;
    }
    return ctx->edge_positions[idx];
}

size_t scan_dispatcher_enable_map(const ScanEnableBitMap **out_map)
{
    if (out_map) {
        *out_map = k_enable_map;
    }
    return sizeof(k_enable_map) / sizeof(k_enable_map[0]);
}

int scan_dispatcher_run_fmwlsymbol009a(ScanDispatchContext *ctx)
{
    int left;
    int right;
    int process_line;

    if (!ctx) {
        return 0;
    }

    /* fmwlsymbol009a entry writes ctx+20216 = 1. */
    ctx->window_left_index = 1;
    left = scan_dispatcher_edge_pos_from_index(ctx, ctx->window_left_index);
    right = scan_dispatcher_edge_pos_from_index(ctx, ctx->window_right_index);
    process_line = ctx->edge_count;
    ctx->status = SCAN_STATUS_NONE;

    if (ctx->edge_count >= 14 && ctx->edge_positions) {
        int i = 12;
        while (i + 1 < ctx->edge_count && ctx->status == SCAN_STATUS_NONE) {
            int line_idx = i + 1;

            if ((line_idx & 1) && line_idx >= 19) {
                int pprev = ctx->edge_positions[i - 18];
                int p0 = ctx->edge_positions[i - 17];
                int p1 = ctx->edge_positions[i + 1];
                int span = (3 * (p1 - p0)) >> 4;
                if (p0 - pprev > span) {
                    ctx->window_left_index = i - 17;
                }
                left = scan_dispatcher_edge_pos_from_index(ctx, ctx->window_left_index);
                right = p1;
                if ((right - left) > span || line_idx == ctx->edge_count - 1) {
                    ctx->window_right_index = i;
                    if (ctx->window_right_index > ctx->window_left_index + 14 &&
                        ctx->decode_6016277b) {
                        ctx->decode_6016277b(ctx->decode_ctx, line_idx, left, right);
                    }
                }
            }

            if (ctx->enable_305) {
                if ((line_idx & 1) && ctx->decode_fmwlsymbol0062) {
                    ctx->decode_fmwlsymbol0062(ctx->decode_ctx, line_idx, left, right);
                } else if (!(line_idx & 1) && ctx->decode_fmwlsymbol0061) {
                    ctx->decode_fmwlsymbol0061(ctx->decode_ctx, line_idx, left, right);
                }
            }

            if (ctx->enable_304 && ctx->decode_fmwlsymbol0060) {
                ctx->decode_fmwlsymbol0060(ctx->decode_ctx, line_idx, left, right);
            }
            if (ctx->enable_306 && ctx->decode_fmwlsymbol0065) {
                ctx->decode_fmwlsymbol0065(ctx->decode_ctx, line_idx, left, right);
            }

            process_line = i + 2;
            i += 1;
        }
    }

    ctx->process_line_arg = process_line;
    ctx->process_left_arg = left;
    ctx->process_right_arg = right;

    if (ctx->enable_304 && ctx->status != SCAN_STATUS_FATAL && ctx->decode_fmwlsymbol005f) {
        ctx->decode_fmwlsymbol005f(ctx->decode_ctx, process_line, left, right);
    }
    if (ctx->enable_305 && ctx->status != SCAN_STATUS_FATAL && ctx->decode_6016294b) {
        ctx->decode_6016294b(ctx->decode_ctx, process_line, left, right);
    }
    if (ctx->enable_306 && ctx->status != SCAN_STATUS_FATAL && ctx->decode_fmwlsymbol0064) {
        ctx->decode_fmwlsymbol0064(ctx->decode_ctx, process_line, left, right);
    }

    return (int)ctx->status;
}

int scan_dispatcher_result_is_retryable(int status_code)
{
    if (status_code < 0 || status_code > 7) {
        return 0;
    }
    return ((RETRY_MASK >> status_code) & 1u) ? 1 : 0;
}

int scan_dispatcher_build_retry_plan(
    int status_code,
    int max_stages,
    ScanRetryPlan *out_plan)
{
    int i;
    int count;

    if (!out_plan) {
        return 0;
    }

    memset(out_plan, 0, sizeof(*out_plan));
    if (!scan_dispatcher_result_is_retryable(status_code)) {
        return 0;
    }

    if (max_stages <= 0 || max_stages > SCAN_RETRY_MAX_STAGES) {
        count = SCAN_RETRY_MAX_STAGES;
    } else {
        count = max_stages;
    }

    out_plan->count = count;
    for (i = 0; i < count; ++i) {
        out_plan->stages[i] = k_retry_order[i];
    }
    return count;
}

static int call_decoder(
    ScanDecodeFn call,
    void *ctx,
    int family_tag,
    int mode,
    int edge_count,
    int searching)
{
    if (!call) {
        return 0;
    }
    return call(ctx, family_tag | (mode << 8) | (searching << 16), edge_count, 0);
}

int scan_dispatcher_hub_smartsymbol_0001c3(
    const ScanEnableConfig *cfg,
    int edge_count,
    int searching,
    int mode,
    ScanDecodeFn call,
    void *call_ctx)
{
    int half_edges;
    int strict_gate;

    if (!cfg) {
        return 0;
    }

    half_edges = ((edge_count & 1) + edge_count) >> 1;
    strict_gate = ((cfg->off_68 & 0xC0) == 0) ? 1 : 0;

    if (CFG_ENABLED(cfg->off_68) && !(searching && strict_gate)) {
        call_decoder(call, call_ctx, 68, 1, half_edges, searching);
        call_decoder(call, call_ctx, 68, 2, half_edges, searching);
        call_decoder(call, call_ctx, 68, 3, half_edges, searching);
        call_decoder(call, call_ctx, 68, 4, half_edges, searching);
    }

    if (CFG_ENABLED(cfg->off_76)) {
        if ((cfg->off_76 & 0x18) != 0x08) {
            int mode0 = (cfg->off_76 >> 27) & 1;
            call_decoder(call, call_ctx, 76, mode0, half_edges, searching);
            if (cfg->off_200 && !searching) {
                call_decoder(call, call_ctx, 200, mode0, half_edges, searching);
            }
        }
        if ((cfg->off_76 & 0x20) && !searching) {
            int mode2 = ((cfg->off_76 >> 28) & 1) | 2;
            call_decoder(call, call_ctx, 76, mode2, half_edges, searching);
        }
    }

    if (CFG_ENABLED(cfg->off_72)) {
        call_decoder(call, call_ctx, 72, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_112)) {
        call_decoder(call, call_ctx, 112, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_144) && !searching) {
        call_decoder(call, call_ctx, 144, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_80)) {
        call_decoder(call, call_ctx, 80, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_132) && !searching) {
        call_decoder(call, call_ctx, 132, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_128) && !searching) {
        call_decoder(call, call_ctx, 128, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_124) && !searching) {
        call_decoder(call, call_ctx, 124, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_88)) {
        call_decoder(call, call_ctx, 88, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_152) && !searching) {
        call_decoder(call, call_ctx, 152, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_156) && !searching) {
        call_decoder(call, call_ctx, 156, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_164) && !searching) {
        call_decoder(call, call_ctx, 164, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_148) && !searching) {
        call_decoder(call, call_ctx, 148, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_120) && !searching) {
        call_decoder(call, call_ctx, 120, mode, half_edges, searching);
    }
    if (CFG_ENABLED(cfg->off_84)) {
        call_decoder(call, call_ctx, 84, mode, half_edges, searching);
    }

    if (cfg->off_196 && !searching) {
        if (CFG_ENABLED(cfg->off_84) || CFG_ENABLED(cfg->off_72) || CFG_ENABLED(cfg->off_76)) {
            int composite_mode_ok = (cfg->off_192 == 2) ? (mode == 1) : (mode == 0);
            if (composite_mode_ok) {
                call_decoder(call, call_ctx, 196, mode, half_edges, searching);
            }
        }
    }

    if (CFG_ENABLED(cfg->off_104) && !searching) {
        if ((cfg->off_104 & 0x18) != 0x18) {
            call_decoder(call, call_ctx, 104, 0, half_edges, searching);
        }
        if ((cfg->off_104 & 0x03) != 0x03) {
            call_decoder(call, call_ctx, 104, 1, half_edges, searching);
        }
        if ((cfg->off_104 & 0x04) == 0) {
            call_decoder(call, call_ctx, 104, 2, half_edges, searching);
        }
    }

    if (CFG_ENABLED(cfg->off_136) && !searching) {
        call_decoder(call, call_ctx, 136, mode, half_edges, searching);
    }

    return 1;
}

int scan_dispatcher_sub_13927c(
    int axis_forward,
    const int *sample_begin,
    const int *sample_end,
    const ScanRunList *turn_list,
    int baseline_min,
    ScanRunFeatures *out)
{
    const int *base;
    const int *curp;
    const int *pivot;
    const int *extreme;
    int trend;
    int guard;
    int gate;
    int processed;
    int last_ref;
    int edge_count;
    int total_samples;

    if (!sample_begin || !sample_end || !out || sample_end < sample_begin) {
        return 0;
    }

    memset(out, 0, sizeof(*out));
    base = sample_begin;
    curp = sample_begin;
    pivot = sample_begin;
    extreme = sample_begin;
    trend = axis_forward ? 2 : 1;
    gate = *curp + (axis_forward ? 1 : -1);
    guard = axis_forward ? 1 : -1;
    processed = 0;
    last_ref = *curp;
    edge_count = 0;
    total_samples = (int)(sample_end - sample_begin + 1);

    if (out->feature_count < SCAN_DISPATCHER_MAX_TURNS) {
        out->features[out->feature_count++] = 0;
    }

    while (curp <= sample_end) {
        int cur = *curp;
        int cur_idx = (int)(curp - base);
        int ref = last_ref;

        if (turn_list && turn_list->count >= 2) {
            for (int i = 0; i + 1 < turn_list->count; ++i) {
                int x0 = turn_list->values[i];
                int x1 = turn_list->values[i + 1];
                int y0;
                int y1;
                int num;
                int den;
                if (x0 < 0) {
                    x0 = 0;
                } else if (x0 >= total_samples) {
                    x0 = total_samples - 1;
                }
                if (x1 < 0) {
                    x1 = 0;
                } else if (x1 >= total_samples) {
                    x1 = total_samples - 1;
                }
                if (x1 <= x0) {
                    continue;
                }
                if (cur_idx < x0 || cur_idx > x1) {
                    continue;
                }
                y0 = base[x0];
                y1 = base[x1];
                num = (cur_idx - x0) * (y1 - y0);
                den = (x1 - x0);
                ref = y0 + num / den;
                break;
            }
        }
        last_ref = ref;

        if (trend & 1) {
            if (cur > gate) {
                int band = (2 * cur - baseline_min - ref) >> 2;
                if (band < 5) {
                    band = 5;
                }
                guard = cur - band;
                extreme = curp;
            } else if (curp == sample_end || cur < guard) {
                int allow_emit = (!axis_forward) || (edge_count > 0);
                if (allow_emit && out->count < SCAN_DISPATCHER_MAX_TURNS) {
                    const int *edge = pivot;
                    int low = *pivot;
                    int high = *extreme;
                    int target = (low + high) >> 1;
                    if (ref > baseline_min) {
                        int den = 2 * (ref - baseline_min);
                        if (den != 0) {
                            target -= (2 * target - (ref + baseline_min)) * (high - low) / den;
                        }
                    }
                    if (target <= low) {
                        target = low + 1;
                    } else if (target >= high) {
                        target = high - 1;
                    }
                    while (edge < extreme && *edge < target) {
                        ++edge;
                    }
                    if (edge > base) {
                        int edge_idx = (int)(edge - base);
                        int cur_v = *edge;
                        int prev_v = *(edge - 1);
                        int q4 = edge_idx << 4;
                        int den = cur_v - prev_v;
                        if (den != 0) {
                            q4 -= (16 * (cur_v - target)) / den;
                        }
                        if (out->feature_count > 0 && q4 < out->features[out->feature_count - 1]) {
                            q4 = out->features[out->feature_count - 1];
                        }
                        if (out->feature_count < SCAN_DISPATCHER_MAX_TURNS) {
                            out->features[out->feature_count++] = q4;
                        }
                        out->segments[out->count].from_x = (int)(pivot - base);
                        out->segments[out->count].to_x = edge_idx;
                        out->segments[out->count].from_y = *pivot;
                        out->segments[out->count].to_y = cur_v;
                        out->segments[out->count].min_value = (prev_v < cur_v) ? prev_v : cur_v;
                        out->count++;
                        edge_count++;
                    }
                }
                {
                    int band = (ref + baseline_min - 2 * cur) >> 2;
                    if (band < 5) {
                        band = 5;
                    }
                    guard = cur + band;
                }
                pivot = curp;
                trend++;
                gate = cur;
            }
        } else {
            if (cur < gate) {
                int band = (baseline_min - 2 * cur + ref) >> 2;
                if (band < 5) {
                    band = 5;
                }
                guard = cur + band;
                pivot = curp;
                gate = cur;
            } else if (curp == sample_end || cur > guard) {
                int allow_emit = axis_forward || (edge_count > 0);
                if (allow_emit && out->count < SCAN_DISPATCHER_MAX_TURNS) {
                    const int *edge = extreme;
                    int high = *extreme;
                    int low = *pivot;
                    int target = (low + high) >> 1;
                    if (ref > baseline_min) {
                        int den = 2 * (ref - baseline_min);
                        if (den != 0) {
                            target -= (2 * target - (ref + baseline_min)) * (high - low) / den;
                        }
                    }
                    if (target <= low) {
                        target = low + 1;
                    } else if (target >= high) {
                        target = high - 1;
                    }
                    while (edge < pivot && target < *edge) {
                        ++edge;
                    }
                    if (edge > base) {
                        int edge_idx = (int)(edge - base);
                        int cur_v = *edge;
                        int prev_v = *(edge - 1);
                        int q4 = edge_idx << 4;
                        int den = prev_v - cur_v;
                        if (den != 0) {
                            q4 -= (16 * (target - cur_v)) / den;
                        }
                        if (out->feature_count > 0 && q4 < out->features[out->feature_count - 1]) {
                            q4 = out->features[out->feature_count - 1];
                        }
                        if (out->feature_count < SCAN_DISPATCHER_MAX_TURNS) {
                            out->features[out->feature_count++] = q4;
                        }
                        out->segments[out->count].from_x = (int)(extreme - base);
                        out->segments[out->count].to_x = edge_idx;
                        out->segments[out->count].from_y = *extreme;
                        out->segments[out->count].to_y = cur_v;
                        out->segments[out->count].min_value = (prev_v < cur_v) ? prev_v : cur_v;
                        out->count++;
                        edge_count++;
                    }
                }
                {
                    int band = (2 * cur - (ref + baseline_min)) >> 2;
                    if (band < 5) {
                        band = 5;
                    }
                    guard = cur - band;
                }
                extreme = curp;
                trend++;
                gate = cur;
            }
        }

        ++curp;
        ++processed;
    }

    if (out->feature_count < SCAN_DISPATCHER_MAX_TURNS) {
        int terminal_q4 = (processed > 0) ? ((processed - 1) << 4) : 0;
        if (terminal_q4 < out->features[out->feature_count - 1]) {
            terminal_q4 = out->features[out->feature_count - 1];
        }
        out->features[out->feature_count++] = terminal_q4;
    }

    return out->count;
}

int scan_dispatcher_score_direction_chain(
    int center_x,
    int center_y,
    int prefer_diagonal,
    ScanScoreFn scorer,
    void *score_ctx,
    ScanDirectionScoreChain *out)
{
    int i;
    int best_i = -1;
    int best_weighted = INT_MIN;
    int score_sum = 0;
    int delta_sum = 0;
    int max_score = INT_MIN;
    int min_score = INT_MAX;
    int axis_x = 0;
    int axis_y = 0;

    if (!scorer || !out) {
        return 0;
    }

    memset(out, 0, sizeof(*out));

    for (i = 0; i < SCAN_DISPATCHER_MAX_DIRS; ++i) {
        int raw = scorer(score_ctx, center_x, center_y, i);
        int weighted = raw;
        if (prefer_diagonal && (i & 1)) {
            weighted += 64;
        }

        out->raw_scores[i] = raw;
        out->weighted_scores[i] = weighted;
        score_sum += raw;
        if (raw > max_score) {
            max_score = raw;
        }
        if (raw < min_score) {
            min_score = raw;
        }

        if (weighted > best_weighted) {
            best_weighted = weighted;
            best_i = i;
        }
    }

    for (i = 0; i < SCAN_DISPATCHER_MAX_DIRS; ++i) {
        int next = (i + 1) & (SCAN_DISPATCHER_MAX_DIRS - 1);
        int delta = out->raw_scores[i] - out->raw_scores[next];
        if (delta < 0) {
            delta = -delta;
        }
        delta_sum += delta;
    }

    out->best_dir = best_i;
    out->best_weighted_score = best_weighted;
    out->max_score = max_score;
    out->min_score = min_score;
    out->score_sum = score_sum;
    out->score_mean_q4 = (score_sum << 4) / SCAN_DISPATCHER_MAX_DIRS;
    out->abs_neighbor_delta_sum = delta_sum;
    if (score_sum > 0) {
        out->dispersion_q12 = (delta_sum << 12) / score_sum;
    } else {
        out->dispersion_q12 = 0;
    }

    out->refined_dir = best_i;
    if (max_score > 1000 && min_score < (max_score >> 1)) {
        out->refine_enabled = 1;
        for (i = 0; i < SCAN_DISPATCHER_MAX_DIRS / 2; ++i) {
            int delta = out->raw_scores[i] - out->raw_scores[i + 8];
            axis_x += delta * k_dir_tbl[i][0];
            axis_y += delta * k_dir_tbl[i][1];
        }

        if (axis_x != 0 || axis_y != 0) {
            int refined = choose_nearest_dir(axis_x, axis_y, prefer_diagonal);
            if (refined >= 0) {
                out->refined_dir = refined;
            }
        }
    }

    if (axis_x != 0 || axis_y != 0) {
        int axis_abs = abs(axis_x);
        if (abs(axis_y) > axis_abs) {
            axis_abs = abs(axis_y);
        }
        if (axis_abs > 0) {
            out->axis_x_q8 = (axis_x << 8) / axis_abs;
            out->axis_y_q8 = (axis_y << 8) / axis_abs;
        }
    }

    return (best_i >= 0) ? 1 : 0;
}

int scan_dispatcher_choose_direction(
    int center_x,
    int center_y,
    int prefer_diagonal,
    ScanScoreFn scorer,
    void *score_ctx,
    ScanDirectionChoice *out)
{
    ScanDirectionScoreChain chain;
    int dir_idx;

    if (!scorer || !out) {
        return 0;
    }

    if (!scan_dispatcher_score_direction_chain(
            center_x, center_y, prefer_diagonal, scorer, score_ctx, &chain)) {
        return 0;
    }

    dir_idx = chain.refine_enabled ? chain.refined_dir : chain.best_dir;
    if (dir_idx < 0 || dir_idx >= SCAN_DISPATCHER_MAX_DIRS) {
        dir_idx = chain.best_dir;
    }
    if (dir_idx < 0 || dir_idx >= SCAN_DISPATCHER_MAX_DIRS) {
        return 0;
    }

    out->center_x = center_x;
    out->center_y = center_y;
    out->stride_x = k_dir_tbl[dir_idx][0];
    out->stride_y = k_dir_tbl[dir_idx][1];
    out->sample_radius = 32;
    return 1;
}

int scan_dispatcher_build_scanline(
    const ScanDirectionChoice *choice,
    ScanSampleFn sampler,
    void *sample_ctx,
    ScanLineBuffer *out_line)
{
    ScanLineIterCompat it;
    int x0;
    int y0;
    int x1;
    int y1;
    int n;

    if (!choice || !sampler || !out_line) {
        return 0;
    }
    if (choice->sample_radius < 0) {
        return 0;
    }

    x0 = choice->center_x - choice->sample_radius * choice->stride_x;
    y0 = choice->center_y - choice->sample_radius * choice->stride_y;
    x1 = choice->center_x + choice->sample_radius * choice->stride_x;
    y1 = choice->center_y + choice->sample_radius * choice->stride_y;

    n = fmdclsymbol0049(&it, x0, y0, x1, y1, 1);
    if (n <= 0) {
        return 0;
    }

    out_line->count = 0;
    for (int i = 0; i < n && out_line->count < SCAN_DISPATCHER_MAX_SAMPLES; ++i) {
        if (i != 0) {
            (void)AHEAD(&it);
        }
        out_line->samples[out_line->count].x = it.x;
        out_line->samples[out_line->count].y = it.y;
        out_line->samples[out_line->count].value = sampler(sample_ctx, it.x, it.y);
        out_line->count++;
    }

    return (int)out_line->count;
}

int scan_dispatcher_build_scanline_stitched(
    const ScanDirectionChoice *choice,
    ScanSampleFn sampler,
    void *sample_ctx,
    ScanLineBuffer *out_line,
    size_t *out_center_index)
{
    ScanLineIterCompat it_neg;
    ScanLineIterCompat it_pos;
    ScanSample center;
    size_t left_count = 0;
    size_t dst = 0;
    int x_neg;
    int y_neg;
    int x_pos;
    int y_pos;
    int n_neg;
    int n_pos;

    if (!choice || !sampler || !out_line) {
        return 0;
    }
    if (choice->sample_radius < 0) {
        return 0;
    }

    x_neg = choice->center_x - choice->sample_radius * choice->stride_x;
    y_neg = choice->center_y - choice->sample_radius * choice->stride_y;
    x_pos = choice->center_x + choice->sample_radius * choice->stride_x;
    y_pos = choice->center_y + choice->sample_radius * choice->stride_y;

    n_neg = fmdclsymbol0049(&it_neg,
                            choice->center_x,
                            choice->center_y,
                            x_neg,
                            y_neg,
                            1);
    n_pos = fmdclsymbol0049(&it_pos,
                            choice->center_x,
                            choice->center_y,
                            x_pos,
                            y_pos,
                            1);
    if (n_neg <= 0 || n_pos <= 0) {
        return 0;
    }

    center.x = choice->center_x;
    center.y = choice->center_y;
    center.value = sampler(sample_ctx, choice->center_x, choice->center_y);

    out_line->count = 0;
    for (int i = 0; i < n_neg && left_count < SCAN_DISPATCHER_MAX_SAMPLES / 2; ++i) {
        (void)AHEAD(&it_neg);
        out_line->samples[left_count].x = it_neg.x;
        out_line->samples[left_count].y = it_neg.y;
        out_line->samples[left_count].value = sampler(sample_ctx, it_neg.x, it_neg.y);
        left_count++;
    }

    for (size_t i = 0; i < left_count / 2; ++i) {
        ScanSample tmp = out_line->samples[i];
        out_line->samples[i] = out_line->samples[left_count - 1 - i];
        out_line->samples[left_count - 1 - i] = tmp;
    }
    dst = left_count;

    if (out_center_index) {
        *out_center_index = dst;
    }
    if (dst < SCAN_DISPATCHER_MAX_SAMPLES) {
        out_line->samples[dst++] = center;
    }
    for (int i = 0; i < n_pos && i < SCAN_DISPATCHER_MAX_SAMPLES / 2 &&
                        dst < SCAN_DISPATCHER_MAX_SAMPLES; ++i) {
        (void)AHEAD(&it_pos);
        out_line->samples[dst].x = it_pos.x;
        out_line->samples[dst].y = it_pos.y;
        out_line->samples[dst].value = sampler(sample_ctx, it_pos.x, it_pos.y);
        dst++;
    }
    out_line->count = dst;

    return (int)out_line->count;
}
