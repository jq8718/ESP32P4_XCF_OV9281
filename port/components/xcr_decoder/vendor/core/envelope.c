#include "envelope.h"
#include <stdlib.h>
#include <string.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

/*
 * Envelope-adaptive barcode edge detection pipeline.
 * Ported from restored firmware algorithms (RESTORE_PLAN.md §2-6).
 */

/* ════════════════════════════════════════════════════════════════════════
 *  §2: First derivative (in-place)
 *  signal[i] = signal[i+1] - signal[i]
 * ════════════════════════════════════════════════════════════════════════ */
void env_first_derivative(int *signal, int length)
{
    for (int i = 0; i < length - 1; i++)
        signal[i] = signal[i + 1] - signal[i];
}

/* ════════════════════════════════════════════════════════════════════════
 *  §3: DMFI adaptive threshold
 *  Computes first derivative, builds abs-value histogram (65 bins, width 4),
 *  returns 7/8 quantile of non-zero derivatives as threshold.
 * ════════════════════════════════════════════════════════════════════════ */
int env_dmfi_thresh(const int *signal, int length)
{
    if (length < 2) return 8;

    /* Work on a copy for the in-place derivative */
    int *tmp = (int *)malloc((size_t)length * sizeof(int));
    if (!tmp) return 8;
    memcpy(tmp, signal, (size_t)length * sizeof(int));

    env_first_derivative(tmp, length);

    /* Build absolute-value histogram: 65 bins, each width 4 */
    int histogram[65];
    memset(histogram, 0, sizeof(histogram));

    for (int i = 0; i < length - 1; i++) {
        int val = tmp[i] < 0 ? -tmp[i] : tmp[i];
        int bin = val / 4;
        if (bin > 64) bin = 64;
        histogram[bin]++;
    }
    free(tmp);

    int zero_count = histogram[0];
    int target = 7 * (length - 1 - zero_count) / 8;
    int cumsum = 0;
    int bin;
    for (bin = 1; bin <= 63; bin++) {
        cumsum += histogram[bin];
        if (cumsum >= target)
            break;
    }

    int threshold = 4 * bin + 8;
    if (threshold >= 127) threshold = 127;
    if (bin >= 64) threshold = 127;

    return threshold;
}

/* ════════════════════════════════════════════════════════════════════════
 *  §5: Bidirectional exponential smoothing envelope
 *
 *  Two-pass asymmetric IIR filter:
 *  - Upper envelope: tracks peaks (fast follow up, slow decay)
 *  - Lower envelope: tracks valleys (fast follow down, slow rise)
 *  - Reverse pass eliminates directional lag
 *  - Uses <<8 fixed-point for smoothing precision
 * ════════════════════════════════════════════════════════════════════════ */
void env_compute_envelope(const int *signal, int length,
                          int *upper, int *lower,
                          int alpha_upper, int alpha_lower)
{
    if (length <= 0) return;

    /* ===== Forward pass (left → right) ===== */
    int upper_acc = signal[0] << 8;
    int lower_acc = upper_acc;
    upper[0] = upper_acc;
    lower[0] = lower_acc;

    if (length <= 1) {
        upper[0] >>= 8;
        lower[0] >>= 8;
        return;
    }

    for (int i = 1; i < length; i++) {
        int spread = upper_acc - lower_acc;
        int sample_scaled = signal[i] << 8;

        /* Upper: decay from current upper by spread/2^alpha, but never below signal */
        int decayed = upper_acc - (spread >> alpha_upper);
        upper[i] = (decayed > sample_scaled) ? decayed : sample_scaled;

        /* Lower: rise from current lower by spread/2^alpha, but never above signal */
        lower[i] = sample_scaled;
        int risen = lower_acc + (spread >> alpha_lower);
        if (risen < sample_scaled)
            lower[i] = risen;

        upper_acc = upper[i];
        lower_acc = lower[i];
    }

    /* ===== Reverse pass (right → left) ===== */
    upper[length - 1] >>= 8;
    lower[length - 1] >>= 8;

    for (int i = length - 2; i >= 0; i--) {
        int spread = upper_acc - lower_acc;

        /* Reverse-decay upper: use tighter of forward/reverse */
        int decayed = upper_acc - (spread >> alpha_upper);
        if (decayed > upper[i])
            upper[i] = decayed;
        int final_upper = upper[i];

        /* Reverse-rise lower: use tighter of forward/reverse */
        int risen = lower_acc + (spread >> alpha_lower);
        if (risen < lower[i])
            lower[i] = risen;

        lower_acc = lower[i];
        upper_acc = final_upper;

        upper[i] >>= 8;
        lower[i] >>= 8;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  §6: Envelope-adaptive peak-valley edge detector
 *
 *  Alternating state machine that finds bar↔space transitions.
 *  Adaptive trigger thresholds based on signal's normalized position
 *  within the envelope. Asymmetric sensitivity: bar detection is more
 *  sensitive than space detection.
 * ════════════════════════════════════════════════════════════════════════ */
int env_edge_detect(const int *signal, int *output,
                    const int *upper_env, const int *lower_env,
                    int length)
{
    if (length <= 0) return 0;

    output[0] = 0;             /* start marker */
    int count = 1;
    int parity = 1;            /* 1=seeking peak (odd), 0=seeking valley (even) */
    int extreme = signal[0] - 1;
    int threshold = 1;
    int extreme_pos = 0;
    int prev_pos = 0;

    for (int i = 0; i < length; i++) {
        int val = signal[i];
        int env_range = upper_env[i] - lower_env[i];
        int at_end = (i == length - 1);

        if (parity & 1) {
            /* ===== Seeking peak (space/bright top) ===== */
            if (val > extreme) {
                int range = (env_range > 8) ? env_range : 8;
                int norm = ((val - lower_env[i]) << 10) / range;

                int drop;
                if (norm == 1024) {
                    drop = env_range / 3;
                    if (drop < 3) drop = 3;
                } else if (norm >= 923) {
                    drop = env_range / 8;
                    if (drop < 3) drop = 3;
                } else {
                    drop = env_range / 16;
                    if (drop < 3) drop = 3;
                }

                threshold = val - drop;
                extreme = val;
                extreme_pos = i;
            }
            else if (val < threshold || at_end) {
                /* Confirm peak, record edge */
                if (count >= 2) {
                    output[count] = prev_pos;
                    extreme = val;
                    count++;
                } else {
                    extreme = val;
                }

                /* Switch to seeking valley */
                parity++;
                int rise = env_range / 16;
                if (rise < 3) rise = 3;
                threshold = extreme + rise;
            }
        }
        else {
            /* ===== Seeking valley (bar/dark bottom) ===== */
            if (val < extreme) {
                int range = (env_range > 8) ? env_range : 8;
                int norm = ((val - lower_env[i]) << 10) / range;

                if (norm <= 204) {
                    /* Near lower envelope (<20%): extreme sensitivity */
                    threshold = val + 1;
                } else {
                    int rise = env_range / 16;
                    if (rise < 3) rise = 3;
                    threshold = val + rise;
                }
                extreme = val;
            }
            else if (val > threshold || at_end) {
                /* Confirm valley, record edge */
                output[count] = extreme_pos;
                extreme = val;
                count++;
                extreme_pos = i;

                /* Switch to seeking peak */
                parity++;
                int drop = env_range / 16;
                if (drop < 3) drop = 3;
                threshold = extreme - drop;
            }
        }

        prev_pos = extreme_pos;
    }

    return count;
}

/* ════════════════════════════════════════════════════════════════════════
 *  §4: Envelope-normalized strong edge filter
 *
 *  Normalizes signal at each edge position to 0~1024 (10-bit fixed-point)
 *  using the envelope, then keeps only strong edges:
 *  - Bar (dark): normalized <= 204 (~20%)
 *  - Space (bright): normalized > 700 (~68%)
 *  Edges in the uncertainty zone (204~700) are discarded.
 * ════════════════════════════════════════════════════════════════════════ */
int env_edge_filter(const int *signal, const int *edges_in,
                    const int *upper_env, const int *lower_env,
                    int n_edges, int *edges_out)
{
    int count = n_edges + (n_edges & 1);  /* round up to even */
    int out_idx = 0;

    for (int i = 0; i < count - 3; i++) {
        int pos = edges_in[i + 2];  /* skip first 2 edges */
        if (pos < 0) pos = -pos;    /* handle negative positions */

        int range = upper_env[pos] - lower_env[pos];
        if (range <= 8) range = 8;
        int normalized = ((signal[pos] - lower_env[pos]) << 10) / range;

        if ((i + 2) & 1) {
            /* Odd position = expecting bar→space transition */
            if (normalized > 700) {
                edges_out[out_idx++] = pos;
            }
        } else {
            /* Even position = expecting space→bar transition */
            if (normalized <= 204) {
                edges_out[out_idx++] = pos;
            }
        }
    }

    return out_idx;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Full pipeline: row pixels → edge positions
 *
 *  Chains: sample → envelope → edge_detect → envelope_edge_filter
 * ════════════════════════════════════════════════════════════════════════ */

/* Internal buffer size limit */
#define ENV_MAX_LEN 4096

int envelope_decode_scanline(const uint8_t *row, int width,
                             int *edges_out, int max_edges)
{
    if (width <= 10 || width > ENV_MAX_LEN) return 0;

    /* Allocate working buffers:
     * signal[N] + upper_env[N] + lower_env[N] + raw_edges[N] */
    int *buf = (int *)malloc((size_t)width * 4 * sizeof(int));
    if (!buf) return 0;

    int *signal    = buf;
    int *upper_env = buf + width;
    int *lower_env = buf + width * 2;
    int *raw_edges = buf + width * 3;

    /* Step 1: Sample — convert uint8_t row to int array */
    for (int i = 0; i < width; i++)
        signal[i] = row[i];

    /* Step 2: Compute bidirectional exponential smoothing envelope
     * alpha_upper=8 (decay 1/256), alpha_lower=10 (rise 1/1024) */
    env_compute_envelope(signal, width, upper_env, lower_env, 8, 10);

    /* Step 3: Edge detection — peak-valley tracker */
    int n_raw = env_edge_detect(signal, raw_edges, upper_env, lower_env, width);

    if (n_raw <= 9) {
        free(buf);
        return 0;
    }

    /* Step 4: Envelope edge filter — keep only strong edges */
    int n_strong = env_edge_filter(signal, raw_edges, upper_env, lower_env,
                                   n_raw, edges_out);

    /* Clamp to max_edges */
    if (n_strong > max_edges)
        n_strong = max_edges;

    free(buf);
    return n_strong;
}
