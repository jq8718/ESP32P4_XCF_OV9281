#include "aztec.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <stdio.h>

/* ── Constants ─────────────────────────────────────────────────────────── */

#define AZTEC_DEBUG_PRINT 0
#if AZTEC_DEBUG_PRINT
#define AZ_DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define AZ_DBG(...) ((void)0)
#endif

#define AZ_MAX_LAYERS    32
#define AZ_MAX_DATA_CW   1920
#define AZ_MAX_BITS      (AZ_MAX_DATA_CW * 12)

static AztecDecodeDebugStats g_aztec_debug_stats;

typedef struct {
    int full_side_samples_bias;
    int full_side_round_bias;
    int fallback_center_jitter;
    int fallback_scale_jitter;
} AztecFullSamplingTuning;

static const AztecFullSamplingTuning k_aztec_tuning_default = {
    1, /* 20B: side_samples = 2*layer+6 */
    1, /* 20B: slightly outward-biased side interpolation */
    2, /* 20D: finder center jitter widened to [-2, +2] */
    2  /* finder module-scale jitter: [-2, +2] */
};

static AztecFullSamplingTuning g_aztec_tuning = {
    1, 1, 2, 2
};

/* ── GF(2^N) Reed-Solomon ──────────────────────────────────────────────── */

/* GF(16): x^4 + x + 1 = 0x13 */
static int gf16_exp[32];
static int gf16_log[16];
static int gf16_ready;

static void gf16_init(void)
{
    if (gf16_ready) return;
    int v = 1;
    for (int i = 0; i < 15; i++) {
        gf16_exp[i] = v;
        gf16_log[v] = i;
        v <<= 1;
        if (v & 16) v ^= 0x13;
    }
    for (int i = 15; i < 32; i++) gf16_exp[i] = gf16_exp[i - 15];
    gf16_log[0] = -1;
    gf16_ready = 1;
}

static int gf16_mul(int a, int b)
{
    if (a == 0 || b == 0) return 0;
    return gf16_exp[gf16_log[a] + gf16_log[b]];
}

/* GF(64): x^6 + x + 1 = 0x43 */
static int gf64_exp[128];
static int gf64_log[64];
static int gf64_ready;

static void gf64_init(void)
{
    if (gf64_ready) return;
    int v = 1;
    for (int i = 0; i < 63; i++) {
        gf64_exp[i] = v;
        gf64_log[v] = i;
        v <<= 1;
        if (v & 64) v ^= 0x43;
    }
    for (int i = 63; i < 128; i++) gf64_exp[i] = gf64_exp[i - 63];
    gf64_log[0] = -1;
    gf64_ready = 1;
}

/* GF(256): x^8 + x^4 + x^3 + x^2 + 1 = 0x12d */
static int gf256_exp[512];
static int gf256_log[256];
static int gf256_ready;

static void gf256_init(void)
{
    if (gf256_ready) return;
    int v = 1;
    for (int i = 0; i < 255; i++) {
        gf256_exp[i] = v;
        gf256_log[v] = i;
        v <<= 1;
        if (v & 256) v ^= 0x12d;
    }
    for (int i = 255; i < 512; i++) gf256_exp[i] = gf256_exp[i - 255];
    gf256_log[0] = -1;
    gf256_ready = 1;
}

/* GF(4096): x^12 + x^6 + x^4 + x + 1 = 0x1069 */
static int gf4096_exp[8192];
static int gf4096_log[4096];
static int gf4096_ready;

static void gf4096_init(void)
{
    if (gf4096_ready) return;
    int v = 1;
    for (int i = 0; i < 4095; i++) {
        gf4096_exp[i] = v;
        gf4096_log[v] = i;
        v <<= 1;
        if (v & 4096) v ^= 0x1069;
    }
    for (int i = 4095; i < 8192; i++) gf4096_exp[i] = gf4096_exp[i - 4095];
    gf4096_log[0] = -1;
    gf4096_ready = 1;
}

/* ── Generic RS decoder (Berlekamp-Massey + Forney) ────────────────── */

typedef struct {
    int *exp_tbl;
    int *log_tbl;
    int  order;  /* field_size - 1 */
} GField;

static int gf_mul(const GField *gf, int a, int b)
{
    if (a == 0 || b == 0) return 0;
    return gf->exp_tbl[gf->log_tbl[a] + gf->log_tbl[b]];
}

static int gf_inv(const GField *gf, int a)
{
    if (a == 0) return 0;
    return gf->exp_tbl[gf->order - gf->log_tbl[a]];
}

static int rs_decode(const GField *gf, int *cw, int n, int n_ec)
{
    if (n_ec > 256 || n_ec <= 0) return -1;

    int syn[256];
    int has_error = 0;
    for (int i = 0; i < n_ec; i++) {
        int s = 0;
        int alpha_i = gf->exp_tbl[i + 1];
        int pw = 1;
        for (int j = n - 1; j >= 0; j--) {
            s ^= gf_mul(gf, cw[j], pw);
            pw = gf_mul(gf, pw, alpha_i);
        }
        syn[i] = s;
        if (s != 0) has_error = 1;
    }
    if (!has_error) return 0;

    int sigma[256], sp[256];
    memset(sigma, 0, sizeof(sigma));
    memset(sp, 0, sizeof(sp));
    sigma[0] = 1; sp[0] = 1;
    int L = 0, m = 1, b = 1;

    for (int r = 0; r < n_ec; r++) {
        int d = syn[r];
        for (int j = 1; j <= L; j++)
            d ^= gf_mul(gf, sigma[j], syn[r - j]);
        if (d == 0) { m++; continue; }
        int tmp[256];
        memcpy(tmp, sigma, sizeof(tmp));
        int coeff = gf_mul(gf, d, gf_inv(gf, b));
        for (int j = m; j < n_ec + 1 && j < 256; j++)
            sigma[j] ^= gf_mul(gf, coeff, sp[j - m]);
        if (2 * L <= r) {
            L = r + 1 - L;
            memcpy(sp, tmp, sizeof(sp));
            b = d; m = 1;
        } else { m++; }
    }

    /* Reliable RS correction requires the declared error count to be within
     * the classic bound (2*errors <= parity symbols); otherwise the
     * "correction" is a miscorrection of random data and must be rejected. */
    if (2 * L > n_ec) return -1;

    int err_pos[256], ne = 0;
    for (int i = 0; i < n; i++) {
        int v = 0, pw = 1;
        int ai = gf->exp_tbl[gf->order - i];
        for (int j = 0; j <= L; j++) {
            v ^= gf_mul(gf, sigma[j], pw);
            pw = gf_mul(gf, pw, ai);
        }
        if (v == 0) err_pos[ne++] = n - 1 - i;
    }
    if (ne != L) return -1;

    int omega[256];
    memset(omega, 0, sizeof(omega));
    for (int i = 0; i < n_ec; i++) {
        int s = 0;
        for (int j = 0; j <= i && j <= L; j++)
            s ^= gf_mul(gf, sigma[j], syn[i - j]);
        omega[i] = s;
    }

    int sd[256];
    memset(sd, 0, sizeof(sd));
    for (int i = 1; i <= L; i += 2) sd[i - 1] = sigma[i];

    for (int k = 0; k < ne; k++) {
        int pos = err_pos[k];
        int xi = gf->exp_tbl[gf->order - (n - 1 - pos)];
        int ov = 0, pw = 1;
        for (int j = 0; j < n_ec; j++) {
            ov ^= gf_mul(gf, omega[j], pw);
            pw = gf_mul(gf, pw, xi);
        }
        int sv = 0; pw = 1;
        for (int j = 0; j <= (L - 1) / 2; j++) {
            sv ^= gf_mul(gf, sd[j], pw);
            pw = gf_mul(gf, pw, xi);
        }
        if (sv == 0) return -1;
        cw[pos] ^= gf_mul(gf, ov, gf_inv(gf, sv));
    }
    return 0;
}

/* RS for GF(16) mode message */
static int rs_decode_gf16(int *cw, int n, int n_ec)
{
    gf16_init();
    GField gf = { gf16_exp, gf16_log, 15 };
    return rs_decode(&gf, cw, n, n_ec);
}

/* ── Pixel access ──────────────────────────────────────────────────────── */

static inline int is_black(const uint8_t *grid, int w, int h, int x, int y)
{
    if (x < 0 || x >= w || y < 0 || y >= h) return 0;
    return grid[y * w + x] < 128;
}

/* ── Bull's-eye detection ──────────────────────────────────────────────── */

typedef struct {
    int cx, cy;         /* center pixel (center of center module) */
    int compact;        /* 1=compact (5x5), 0=full (7x7) */
    int mod_size_x;     /* module size X (*16 fixed point) */
    int mod_size_y;     /* module size Y (*16 fixed point) */
    int mod_size_x_pos; /* +X module scale (*16 fixed point) */
    int mod_size_x_neg; /* -X module scale (*16 fixed point) */
    int mod_size_y_pos; /* +Y module scale (*16 fixed point) */
    int mod_size_y_neg; /* -Y module scale (*16 fixed point) */
    double fit_error;   /* ring-width consistency error, lower = better finder */
} FinderInfo;

typedef struct {
    int valid;
    int w;
    int h;
    int *storage;
    int *right;
    int *left;
    int *down;
    int *up;
} BullseyeRunCache;

/*
 * Measure a run of same-colored pixels from (x,y) in direction (dx,dy).
 * Returns the run length (not counting the starting pixel).
 */
static int measure_run(const uint8_t *grid, int w, int h,
                       int x, int y, int dx, int dy)
{
    int start_black = is_black(grid, w, h, x, y);
    int len = 0;
    for (;;) {
        x += dx; y += dy;
        if (x < 0 || x >= w || y < 0 || y >= h) break;
        if (is_black(grid, w, h, x, y) != start_black) break;
        len++;
    }
    return len;
}

static BullseyeRunCache bullseye_run_cache_init(const uint8_t *grid, int w, int h)
{
    BullseyeRunCache cache = {0};
    size_t count;
    size_t bytes;

    cache.w = w;
    cache.h = h;
    if (!grid || w <= 0 || h <= 0) return cache;
    if ((size_t)w > SIZE_MAX / (size_t)h) return cache;
    count = (size_t)w * (size_t)h;
    if (count > SIZE_MAX / sizeof(int)) return cache;
    bytes = count * sizeof(int);
    if (bytes > SIZE_MAX / 4u) return cache;

    cache.storage = (int *)malloc(bytes * 4u);
    if (!cache.storage) return cache;
    cache.right = cache.storage;
    cache.left = cache.right + count;
    cache.down = cache.left + count;
    cache.up = cache.down + count;

    for (int y = 0; y < h; y++) {
        size_t row = (size_t)y * (size_t)w;
        for (int x = w - 1; x >= 0; x--) {
            size_t idx = row + (size_t)x;
            cache.right[idx] =
                (x + 1 < w && ((grid[idx + 1] < 128) == (grid[idx] < 128)))
                ? cache.right[idx + 1] + 1 : 0;
        }
        for (int x = 0; x < w; x++) {
            size_t idx = row + (size_t)x;
            cache.left[idx] =
                (x > 0 && ((grid[idx - 1] < 128) == (grid[idx] < 128)))
                ? cache.left[idx - 1] + 1 : 0;
        }
    }

    for (int y = h - 1; y >= 0; y--) {
        size_t row = (size_t)y * (size_t)w;
        for (int x = 0; x < w; x++) {
            size_t idx = row + (size_t)x;
            cache.down[idx] =
                (y + 1 < h && ((grid[idx + (size_t)w] < 128) == (grid[idx] < 128)))
                ? cache.down[idx + (size_t)w] + 1 : 0;
        }
    }
    for (int y = 0; y < h; y++) {
        size_t row = (size_t)y * (size_t)w;
        for (int x = 0; x < w; x++) {
            size_t idx = row + (size_t)x;
            cache.up[idx] =
                (y > 0 && ((grid[idx - (size_t)w] < 128) == (grid[idx] < 128)))
                ? cache.up[idx - (size_t)w] + 1 : 0;
        }
    }
    cache.valid = 1;
    return cache;
}

static void bullseye_run_cache_destroy(BullseyeRunCache *cache)
{
    if (!cache) return;
    free(cache->storage);
    memset(cache, 0, sizeof(*cache));
}

static int measure_run_cached(const uint8_t *grid, int w, int h,
                              int x, int y, int dx, int dy,
                              const BullseyeRunCache *cache)
{
    size_t idx;
    if (!cache || !cache->valid || cache->w != w || cache->h != h ||
        x < 0 || x >= w || y < 0 || y >= h) {
        return measure_run(grid, w, h, x, y, dx, dy);
    }
    idx = (size_t)y * (size_t)w + (size_t)x;
    if (dx == 1 && dy == 0) return cache->right[idx];
    if (dx == -1 && dy == 0) return cache->left[idx];
    if (dx == 0 && dy == 1) return cache->down[idx];
    if (dx == 0 && dy == -1) return cache->up[idx];
    return measure_run(grid, w, h, x, y, dx, dy);
}

/* Refine a black seed to the midpoint of its center module. */
static int seed_refine_center(const uint8_t *grid, int w, int h,
                              int cx, int cy, int *true_cx, int *true_cy,
                              const BullseyeRunCache *cache)
{
    if (!is_black(grid, w, h, cx, cy)) return 0;

    /* Measure center black region in 4 directions */
    int r_right = measure_run_cached(grid, w, h, cx, cy, 1, 0, cache);
    int r_left  = measure_run_cached(grid, w, h, cx, cy, -1, 0, cache);
    int r_down  = measure_run_cached(grid, w, h, cx, cy, 0, 1, cache);
    int r_up    = measure_run_cached(grid, w, h, cx, cy, 0, -1, cache);

    int center_w = r_left + 1 + r_right;
    int center_h_px = r_up + 1 + r_down;
    if (center_w < 1 || center_h_px < 1) return 0;

    /* Refine center to midpoint of the center module */
    *true_cx = cx - r_left + center_w / 2;
    *true_cy = cy - r_up + center_h_px / 2;
    return 1;
}

/* Validate one refined center using the original ring-fit algorithm. */
static int validate_refined_center(const uint8_t *grid, int w, int h,
                                   int true_cx, int true_cy, FinderInfo *info,
                                   const BullseyeRunCache *cache)
{
    if (true_cx < 0 || true_cx >= w || true_cy < 0 || true_cy >= h) return 0;
    if (!is_black(grid, w, h, true_cx, true_cy)) return 0;

    /* Re-measure from refined center to stabilize ring widths. */
    int r_right = measure_run_cached(grid, w, h, true_cx, true_cy, 1, 0, cache);
    int r_left  = measure_run_cached(grid, w, h, true_cx, true_cy, -1, 0, cache);
    int r_down  = measure_run_cached(grid, w, h, true_cx, true_cy, 0, 1, cache);
    int r_up    = measure_run_cached(grid, w, h, true_cx, true_cy, 0, -1, cache);
    int center_w = r_left + 1 + r_right;
    int center_h_px = r_up + 1 + r_down;
    if (center_w < 1 || center_h_px < 1) return 0;

    double msx = (double)center_w;
    double msy = (double)center_h_px;
    if (msx < 1.0 || msy < 1.0) return 0;

    /* Evaluate both full (6 alternating rings) and compact (4 alternating
     * rings) hypotheses -- the ring immediately outside is the mode-message
     * ring and is not part of the alternating bullseye pattern, so the
     * ring count is what lets compact vs. full be told apart. Both may
     * spuriously validate under the necessarily loose per-ring tolerance,
     * so instead of accepting whichever is tried first, keep both
     * candidates and let the caller compare ring-fit quality. */
    FinderInfo cand[2];
    int cand_ok[2] = { 0, 0 };
    for (int try_compact = 0; try_compact <= 1; try_compact++) {
        int n_rings = try_compact ? 4 : 6;
        int ok = 1;
        int total_mods = 2 * n_rings + 1;

        /* Walk outward from the edge of the center black region.
           For each direction, measure alternating ring widths. */
        int ring_widths_r[8], ring_widths_l[8], ring_widths_d[8], ring_widths_u[8];

        /* Right direction: start just past the center black region */
        int pos = true_cx + r_right + 1;
        for (int ring = 1; ring <= n_rings && ok; ring++) {
            int expect_black = ((ring % 2) == 0);
            if (pos >= w) { ok = 0; break; }
            if (is_black(grid, w, h, pos, true_cy) != expect_black) { ok = 0; break; }
            int rlen = measure_run_cached(grid, w, h, pos, true_cy, 1, 0, cache) + 1;
            if (rlen < (int)(msx * 0.25) || rlen > (int)(msx * 3.5 + 1)) { ok = 0; break; }
            ring_widths_r[ring] = rlen;
            pos += rlen;
        }

        /* Left direction */
        pos = true_cx - r_left - 1;
        for (int ring = 1; ring <= n_rings && ok; ring++) {
            int expect_black = ((ring % 2) == 0);
            if (pos < 0) { ok = 0; break; }
            if (is_black(grid, w, h, pos, true_cy) != expect_black) { ok = 0; break; }
            int rlen = measure_run_cached(grid, w, h, pos, true_cy, -1, 0, cache) + 1;
            if (rlen < (int)(msx * 0.25) || rlen > (int)(msx * 3.5 + 1)) { ok = 0; break; }
            ring_widths_l[ring] = rlen;
            pos -= rlen;
        }

        /* Down direction */
        pos = true_cy + r_down + 1;
        for (int ring = 1; ring <= n_rings && ok; ring++) {
            int expect_black = ((ring % 2) == 0);
            if (pos >= h) { ok = 0; break; }
            if (is_black(grid, w, h, true_cx, pos) != expect_black) { ok = 0; break; }
            int rlen = measure_run_cached(grid, w, h, true_cx, pos, 0, 1, cache) + 1;
            if (rlen < (int)(msy * 0.25) || rlen > (int)(msy * 3.5 + 1)) { ok = 0; break; }
            ring_widths_d[ring] = rlen;
            pos += rlen;
        }

        /* Up direction */
        pos = true_cy - r_up - 1;
        for (int ring = 1; ring <= n_rings && ok; ring++) {
            int expect_black = ((ring % 2) == 0);
            if (pos < 0) { ok = 0; break; }
            if (is_black(grid, w, h, true_cx, pos) != expect_black) { ok = 0; break; }
            int rlen = measure_run_cached(grid, w, h, true_cx, pos, 0, -1, cache) + 1;
            if (rlen < (int)(msy * 0.25) || rlen > (int)(msy * 3.5 + 1)) { ok = 0; break; }
            ring_widths_u[ring] = rlen;
            pos -= rlen;
        }

        if (!ok) continue;

        /* A single ring can occasionally fuse with a neighboring artifact
         * (e.g. a data/mode-ring module that happens to share color with
         * the last bullseye ring) and read far wider than the true module
         * size while still passing the loose 0.25x-3.5x sanity check above.
         * Such an outlier skews the aggregate module-size estimate badly
         * enough to misplace samples several modules further out (the mode
         * ring, the data rings). Replace any ring measurement that strays
         * far from the per-ring cross-direction median with that median. */
        for (int ring = 1; ring <= n_rings; ring++) {
            int vals[4] = { ring_widths_r[ring], ring_widths_l[ring],
                            ring_widths_d[ring], ring_widths_u[ring] };
            int sorted[4];
            memcpy(sorted, vals, sizeof(sorted));
            for (int a = 0; a < 4; a++)
                for (int b = a + 1; b < 4; b++)
                    if (sorted[b] < sorted[a]) { int t = sorted[a]; sorted[a] = sorted[b]; sorted[b] = t; }
            double med = (sorted[1] + sorted[2]) / 2.0;
            if (med < 1.0) continue;
            if (ring_widths_r[ring] > med * 1.6 || ring_widths_r[ring] < med * 0.6)
                ring_widths_r[ring] = (int)(med + 0.5);
            if (ring_widths_l[ring] > med * 1.6 || ring_widths_l[ring] < med * 0.6)
                ring_widths_l[ring] = (int)(med + 0.5);
            if (ring_widths_d[ring] > med * 1.6 || ring_widths_d[ring] < med * 0.6)
                ring_widths_d[ring] = (int)(med + 0.5);
            if (ring_widths_u[ring] > med * 1.6 || ring_widths_u[ring] < med * 0.6)
                ring_widths_u[ring] = (int)(med + 0.5);
        }

        /* Compute refined module size from full finder span */
        double full_w = (double)center_w;
        double full_h = (double)center_h_px;
        double half_span_rx = (double)r_right + 0.5;
        double half_span_lx = (double)r_left + 0.5;
        double half_span_dy = (double)r_down + 0.5;
        double half_span_uy = (double)r_up + 0.5;
        for (int ring = 1; ring <= n_rings; ring++) {
            full_w += ring_widths_r[ring] + ring_widths_l[ring];
            full_h += ring_widths_d[ring] + ring_widths_u[ring];
            half_span_rx += ring_widths_r[ring];
            half_span_lx += ring_widths_l[ring];
            half_span_dy += ring_widths_d[ring];
            half_span_uy += ring_widths_u[ring];
        }
        double refined_msx = full_w / total_mods;
        double refined_msy = full_h / total_mods;
        double half_mods = (double)n_rings + 0.5;
        double refined_pos_msx = half_span_rx / half_mods;
        double refined_neg_msx = half_span_lx / half_mods;
        double refined_pos_msy = half_span_dy / half_mods;
        double refined_neg_msy = half_span_uy / half_mods;

        if (refined_msx < 1.0 || refined_msy < 1.0) continue;

        /* Robustify module size estimate: data rings can skew one axis. */
        if (refined_msx > msx * 1.35 || refined_msx < msx * 0.65) {
            refined_msx = msx;
        }
        if (refined_msy > msy * 1.35 || refined_msy < msy * 0.65) {
            refined_msy = msy;
        }
        if (refined_msx > refined_msy * 1.25 || refined_msy > refined_msx * 1.25) {
            double uni = (refined_msx < refined_msy) ? refined_msx : refined_msy;
            refined_msx = uni;
            refined_msy = uni;
        }

        if (refined_msx > 100.0 || refined_msy > 100.0) continue;

        if (refined_pos_msx < refined_msx * 0.70 || refined_pos_msx > refined_msx * 1.35)
            refined_pos_msx = refined_msx;
        if (refined_neg_msx < refined_msx * 0.70 || refined_neg_msx > refined_msx * 1.35)
            refined_neg_msx = refined_msx;
        if (refined_pos_msy < refined_msy * 0.70 || refined_pos_msy > refined_msy * 1.35)
            refined_pos_msy = refined_msy;
        if (refined_neg_msy < refined_msy * 0.70 || refined_neg_msy > refined_msy * 1.35)
            refined_neg_msy = refined_msy;

        /* Ring-width consistency: real bull's-eyes have all rings close to
         * the same width; random data regions that happen to pass the
         * alternating-color test tend to have noisier ring widths. Used to
         * rank multiple finder candidates instead of picking by proximity
         * to the image center alone. */
        double fit_error = 0.0;
        for (int ring = 1; ring <= n_rings; ring++) {
            fit_error += fabs(ring_widths_r[ring] - refined_msx) / refined_msx;
            fit_error += fabs(ring_widths_l[ring] - refined_msx) / refined_msx;
            fit_error += fabs(ring_widths_d[ring] - refined_msy) / refined_msy;
            fit_error += fabs(ring_widths_u[ring] - refined_msy) / refined_msy;
        }
        fit_error /= (double)(4 * n_rings);

        cand[try_compact].cx = true_cx;
        cand[try_compact].cy = true_cy;
        cand[try_compact].compact = try_compact;
        cand[try_compact].mod_size_x = (int)(refined_msx * 16 + 0.5);
        cand[try_compact].mod_size_y = (int)(refined_msy * 16 + 0.5);
        cand[try_compact].mod_size_x_pos = (int)(refined_pos_msx * 16 + 0.5);
        cand[try_compact].mod_size_x_neg = (int)(refined_neg_msx * 16 + 0.5);
        cand[try_compact].mod_size_y_pos = (int)(refined_pos_msy * 16 + 0.5);
        cand[try_compact].mod_size_y_neg = (int)(refined_neg_msy * 16 + 0.5);
        cand[try_compact].fit_error = fit_error;
        cand_ok[try_compact] = 1;
    }

    if (!cand_ok[0] && !cand_ok[1]) return 0;
    int best = cand_ok[0] && (!cand_ok[1] || cand[0].fit_error <= cand[1].fit_error) ? 0 : 1;
    *info = cand[best];
    return 1;
}

static int check_bullseye(const uint8_t *grid, int w, int h,
                          int cx, int cy, FinderInfo *info)
{
    int true_cx;
    int true_cy;

    if (!seed_refine_center(grid, w, h, cx, cy, &true_cx, &true_cy, NULL)) {
        return 0;
    }
    return validate_refined_center(grid, w, h, true_cx, true_cy, info, NULL);
}

static int find_bullseye(const uint8_t *grid, int w, int h, FinderInfo *info)
{
    int best_dist = w * w + h * h;
    double best_fit_error = 1e18;
    int found = 0;
    FinderInfo best;
    BullseyeRunCache run_cache = bullseye_run_cache_init(grid, w, h);
    int mid_x = w / 2, mid_y = h / 2;

    int step = 1;
    size_t center_count = 0;
    uint8_t *center_state = NULL;
    typedef struct {
        size_t center_idx;
        FinderInfo info;
    } SparseCenterInfo;
    SparseCenterInfo *center_info = NULL;
    size_t center_info_count = 0;
    size_t center_info_cap = 0;

    if ((size_t)w <= SIZE_MAX / (size_t)h) {
        center_count = (size_t)w * (size_t)h;
        center_state = (uint8_t *)calloc(center_count, sizeof(*center_state));
    }

    if (w > 200) step = 2;
    if (w > 500) step = 3;

    /* Rank candidates by ring-fit quality first (a real finder has tightly
     * consistent ring widths); only fall back to "closest to image center"
     * as a tie-breaker among similarly-good candidates. */
    for (int y = 3; y < h - 3; y += step) {
        for (int x = 3; x < w - 3; x += step) {
            if (!is_black(grid, w, h, x, y)) continue;

            FinderInfo fi;
            int valid = 0;

            if (center_state) {
                int true_cx;
                int true_cy;
                size_t center_idx;
                size_t i;

                if (!seed_refine_center(
                        grid, w, h, x, y, &true_cx, &true_cy, &run_cache)) {
                    continue;
                }
                center_idx = (size_t)true_cy * (size_t)w + (size_t)true_cx;
                if (center_state[center_idx] == 0) {
                    FinderInfo validated_info;
                    if (validate_refined_center(
                            grid, w, h, true_cx, true_cy, &validated_info,
                            &run_cache)) {
                        int cache_ready = center_info_count < center_info_cap;

                        if (!cache_ready) {
                            size_t new_cap = center_info_cap ? center_info_cap * 2 : 8;
                            SparseCenterInfo *grown = NULL;
                            if (new_cap > center_info_cap &&
                                new_cap <= SIZE_MAX / sizeof(*center_info)) {
                                grown = (SparseCenterInfo *)realloc(
                                    center_info, new_cap * sizeof(*center_info));
                            }
                            if (grown) {
                                center_info = grown;
                                center_info_cap = new_cap;
                                cache_ready = 1;
                            }
                        }
                        if (cache_ready) {
                            center_info[center_info_count].center_idx = center_idx;
                            center_info[center_info_count].info = validated_info;
                            center_info_count++;
                            center_state[center_idx] = 2;
                        }
                        fi = validated_info;
                        valid = 1;
                    } else {
                        center_state[center_idx] = 1;
                    }
                } else if (center_state[center_idx] == 2) {
                    for (i = 0; i < center_info_count; i++) {
                        if (center_info[i].center_idx == center_idx) {
                            fi = center_info[i].info;
                            valid = 1;
                            break;
                        }
                    }
                }
            } else {
                int true_cx;
                int true_cy;
                if (seed_refine_center(
                        grid, w, h, x, y, &true_cx, &true_cy, &run_cache)) {
                    valid = validate_refined_center(
                        grid, w, h, true_cx, true_cy, &fi, &run_cache);
                }
            }

            if (valid) {
                int dx = fi.cx - mid_x;
                int dy = fi.cy - mid_y;
                int dist = dx * dx + dy * dy;
                int better = !found;
                if (found) {
                    if (fi.fit_error < best_fit_error - 0.02) better = 1;
                    else if (fi.fit_error < best_fit_error + 0.02 && dist < best_dist) better = 1;
                }
                if (better) {
                    best = fi;
                    best_dist = dist;
                    best_fit_error = fi.fit_error;
                    found = 1;
                }
            }
        }
    }

    if (found) {
        AZ_DBG("AZTEC_DEBUG: bullseye cx=%d cy=%d compact=%d msx=%d msy=%d fit_err=%.3f\n",
               best.cx, best.cy, best.compact, best.mod_size_x, best.mod_size_y, best.fit_error);
        *info = best;
        free(center_info);
        free(center_state);
        bullseye_run_cache_destroy(&run_cache);
        return 1;
    }
    free(center_info);
    free(center_state);
    bullseye_run_cache_destroy(&run_cache);
    return 0;
}

/* ── Module coordinate mapping ─────────────────────────────────────────── */

static int read_mod(const uint8_t *grid, int w, int h,
                    const FinderInfo *fi, int mod_row, int mod_col)
{
    int scale_x = (mod_col >= 0) ? fi->mod_size_x_pos : fi->mod_size_x_neg;
    int scale_y = (mod_row >= 0) ? fi->mod_size_y_pos : fi->mod_size_y_neg;
    int prod_x = mod_col * ((scale_x > 0) ? scale_x : fi->mod_size_x);
    int prod_y = mod_row * ((scale_y > 0) ? scale_y : fi->mod_size_y);
    int px_x = fi->cx + ((prod_x >= 0) ? ((prod_x + 8) / 16) : -(((-prod_x) + 8) / 16));
    int px_y = fi->cy + ((prod_y >= 0) ? ((prod_y + 8) / 16) : -(((-prod_y) + 8) / 16));
    return is_black(grid, w, h, px_x, px_y);
}

/* ── Reference grid ────────────────────────────────────────────────────── */

static int adjust_for_ref_grid(int offset)
{
    if (offset == 0) return 0;
    int sign = (offset > 0) ? 1 : -1;
    int abs_off = offset * sign;
    int n_cross = (abs_off + 8) / 16;
    return offset + sign * n_cross;
}

typedef struct {
    int ar, ac;
    int br, bc;
} LayerPair;

static int aztec_full_layer_side_samples(int layer)
{
    if (layer < 1) return -1;
    int side_samples = 2 * layer + 5 + g_aztec_tuning.full_side_samples_bias;
    if (side_samples < 4) side_samples = 4;
    return side_samples;
}

static int aztec_side_sample_index(int sample_idx, int side_samples, int side_span)
{
    if (side_samples <= 1) return 0;
    int round_bias = (side_samples - 1) / 2 + g_aztec_tuning.full_side_round_bias;
    if (round_bias < 0) round_bias = 0;
    if (round_bias > (side_samples - 1)) round_bias = side_samples - 1;
    int numer = sample_idx * (side_span - 1) + round_bias;
    return numer / (side_samples - 1);
}

static int build_full_layer_pairs(int layer, LayerPair *pairs, int max_pairs)
{
    int side_samples = aztec_full_layer_side_samples(layer);
    if (side_samples <= 0 || !pairs || max_pairs <= 0) return -1;

    int d_outer = 7 + 2 * layer;
    int d_inner = d_outer - 1;
    int side_start = -(d_outer - 1);
    int side_end = d_outer - 1;
    int side_span = side_end - side_start + 1;
    int n = 0;

    for (int k = 0; k < side_samples; k++) {
        int s = aztec_side_sample_index(k, side_samples, side_span);
        int p = side_start + s;
        int q = side_end - s;
        LayerPair seq[4] = {
            { -d_outer, p, -d_inner, p }, /* top */
            { p, d_outer, p, d_inner },   /* right */
            { d_outer, q, d_inner, q },   /* bottom */
            { q, -d_outer, q, -d_inner }  /* left */
        };
        for (int i = 0; i < 4; i++) {
            if (n >= max_pairs) return n;
            seq[i].ar = adjust_for_ref_grid(seq[i].ar);
            seq[i].ac = adjust_for_ref_grid(seq[i].ac);
            seq[i].br = adjust_for_ref_grid(seq[i].br);
            seq[i].bc = adjust_for_ref_grid(seq[i].bc);
            pairs[n++] = seq[i];
        }
    }

    return n;
}

/* ── Mode message reading ──────────────────────────────────────────────── */

static int read_mode_bits_compact_rot(const uint8_t *grid, int w, int h,
                                      const FinderInfo *fi, int *bits, int *nbits,
                                      int rot)
{
    /* Compact core is 11x11 (center + 4 finder rings + the mode ring).
     * The mode ring sits at Chebyshev distance 5 from the center; each
     * side carries 7 bits sampled at the perpendicular offsets -3..3.
     * rot=0: start top-left, rot=1: start top-right (90° CW), etc. */
    int d = 5;
    int raw[28];
    int n = 0;
    for (int c = -3; c <= 3; c++)
        raw[n++] = read_mod(grid, w, h, fi, -d, c);
    for (int r = -3; r <= 3; r++)
        raw[n++] = read_mod(grid, w, h, fi, r, d);
    for (int c = 3; c >= -3; c--)
        raw[n++] = read_mod(grid, w, h, fi, d, c);
    for (int r = 3; r >= -3; r--)
        raw[n++] = read_mod(grid, w, h, fi, r, -d);
    /* Rotate: each side has 7 bits, rotation shifts by 7*rot */
    int shift = (rot % 4) * 7;
    for (int i = 0; i < 28; i++)
        bits[i] = raw[(i + shift) % 28];
    *nbits = 28;
    return 0;
}

static int read_mode_bits_compact(const uint8_t *grid, int w, int h,
                                  const FinderInfo *fi, int *bits, int *nbits)
{
    return read_mode_bits_compact_rot(grid, w, h, fi, bits, nbits, 0);
}

static int read_mode_bits_full_rot(const uint8_t *grid, int w, int h,
                                   const FinderInfo *fi, int *bits, int *nbits,
                                   int rot)
{
    /* Full core is 15x15 (center + 6 finder rings + the mode ring).
     * The mode ring sits at Chebyshev distance 7 from the center; each
     * side carries 10 bits sampled at offsets -5..-1,1..5 (the central
     * reference-grid module at offset 0 is skipped). */
    int d = 7;
    static const int offs[10] = { -5, -4, -3, -2, -1, 1, 2, 3, 4, 5 };
    int raw[40];
    int n = 0;
    for (int i = 0; i < 10; i++)
        raw[n++] = read_mod(grid, w, h, fi, -d, offs[i]);        /* top: left -> right */
    for (int i = 0; i < 10; i++)
        raw[n++] = read_mod(grid, w, h, fi, offs[i], d);         /* right: top -> bottom */
    for (int i = 9; i >= 0; i--)
        raw[n++] = read_mod(grid, w, h, fi, d, offs[i]);         /* bottom: right -> left */
    for (int i = 9; i >= 0; i--)
        raw[n++] = read_mod(grid, w, h, fi, offs[i], -d);        /* left: bottom -> top */
    /* Each side has 10 bits, rotate by 10*rot */
    int shift = (rot % 4) * 10;
    for (int i = 0; i < 40; i++)
        bits[i] = raw[(i + shift) % 40];
    *nbits = 40;
    return 0;
}

static int read_mode_bits_full(const uint8_t *grid, int w, int h,
                               const FinderInfo *fi, int *bits, int *nbits)
{
    return read_mode_bits_full_rot(grid, w, h, fi, bits, nbits, 0);
}

typedef struct {
    int n_layers;
    int n_data_cw;
    int compact;
    int cw_bits;
    int total_cw;
    int total_bits;
    int ec_cw;
} ModeInfo;

/* ISO/IEC 24778: compact total data-ring bits = (88 + 16*L) * L */
static const int compact_total_bits[] = {0, 104, 240, 408, 608};
static const int compact_cw_bits[] = {0, 6, 6, 8, 8};

static int full_layer_bits(int layers)
{
    /* ISO/IEC 24778: full total data-ring bits = (112 + 16*L) * L */
    return (112 + 16 * layers) * layers;
}

static int decode_mode_compact(const int *bits, ModeInfo *mi)
{
    gf16_init();

    int cw[7];
    for (int i = 0; i < 7; i++) {
        cw[i] = 0;
        for (int b = 0; b < 4; b++)
            cw[i] = (cw[i] << 1) | bits[i * 4 + b];
    }

    AZ_DBG("AZTEC_DEBUG: Mode codewords before RS: ");
    for (int i = 0; i < 7; i++) AZ_DBG("%X ", cw[i]);
    AZ_DBG("\n");

    if (rs_decode_gf16(cw, 7, 5) < 0) {
        AZ_DBG("AZTEC_DEBUG: RS decode GF16 failed for mode message\n");
        return -1;
    }

    AZ_DBG("AZTEC_DEBUG: Mode codewords after RS: ");
    for (int i = 0; i < 7; i++) AZ_DBG("%X ", cw[i]);
    AZ_DBG("\n");

    int val = (cw[0] << 4) | (cw[1] & 0xF);
    int layers = ((val >> 6) & 0x3) + 1;
    int data_cw = (val & 0x3F) + 1;

    mi->compact = 1;
    mi->n_layers = layers;
    mi->n_data_cw = data_cw;
    mi->cw_bits = compact_cw_bits[layers];
    mi->total_bits = compact_total_bits[layers];
    mi->total_cw = mi->total_bits / mi->cw_bits;
    mi->ec_cw = mi->total_cw - data_cw;
    AZ_DBG("AZTEC_DEBUG: compact mode layers=%d data_cw=%d total_cw=%d cw_bits=%d\n",
           layers, data_cw, mi->total_cw, mi->cw_bits);
    if (layers < 1 || layers > 4) return -1;
    if (mi->total_cw <= 0 || mi->total_cw > AZ_MAX_DATA_CW) return -1;
    if (data_cw < 1 || data_cw >= mi->total_cw) return -1;
    return (mi->ec_cw <= 0) ? -1 : 0;
}

static int decode_mode_full(const int *bits, ModeInfo *mi)
{
    gf16_init();

    int cw[10];
    for (int i = 0; i < 10; i++) {
        cw[i] = 0;
        for (int b = 0; b < 4; b++)
            cw[i] = (cw[i] << 1) | bits[i * 4 + b];
    }

    if (rs_decode_gf16(cw, 10, 6) < 0) return -1;

    int val = 0;
    for (int i = 0; i < 4; i++)
        val = (val << 4) | (cw[i] & 0xF);

    int layers = ((val >> 11) & 0x1F) + 1;
    int data_cw = (val & 0x7FF) + 1;

    mi->compact = 0;
    mi->n_layers = layers;
    mi->n_data_cw = data_cw;

    if (layers <= 2)       mi->cw_bits = 6;
    else if (layers <= 8)  mi->cw_bits = 8;
    else if (layers <= 22) mi->cw_bits = 10;
    else                   mi->cw_bits = 12;

    mi->total_bits = full_layer_bits(layers);
    mi->total_cw = mi->total_bits / mi->cw_bits;
    mi->ec_cw = mi->total_cw - data_cw;
    if (layers < 1 || layers > AZ_MAX_LAYERS) return -1;
    if (mi->total_cw <= 0 || mi->total_cw > AZ_MAX_DATA_CW) return -1;
    if (data_cw < 1 || data_cw >= mi->total_cw) return -1;
    return (mi->ec_cw <= 0) ? -1 : 0;
}

/* ── Data layer reading ────────────────────────────────────────────────── */

/*
 * Emit the 8*row_size data bits of one compact data ring, following the
 * ZXing extractBits domino order: left column (top->bottom), bottom row
 * (left->right), right column (bottom->top), top row (right->left); each
 * "domino" is a pair of adjacent bits with the outer bit sampled first.
 */
static void aztec_read_compact_layer_bits(const uint8_t *grid, int w, int h,
                                          const FinderInfo *fi,
                                          int d_outer, int d_inner, int row_size,
                                          int *bits, int *n, int max_bits)
{
    for (int j = 0; j < row_size; j++) {
        int row = -d_outer + j;
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, row, -d_outer);
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, row, -d_inner);
    }
    for (int j = 0; j < row_size; j++) {
        int col = -d_outer + j;
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, d_outer, col);
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, d_inner, col);
    }
    for (int j = 0; j < row_size; j++) {
        int row = d_outer - j;
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, row, d_outer);
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, row, d_inner);
    }
    for (int j = 0; j < row_size; j++) {
        int col = d_outer - j;
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, -d_outer, col);
        if (*n < max_bits) bits[(*n)++] = read_mod(grid, w, h, fi, -d_inner, col);
    }
}

static int read_data_bits(const uint8_t *grid, int w, int h,
                          const FinderInfo *fi, const ModeInfo *mi,
                          int *bits, int max_bits)
{
    int n = 0;

    if (fi->compact) {
        /* Compact symbols: data rings are read from the outermost layer
         * inward (ISO/IEC 24778 / ZXing extractBits order). */
        const int core_half = 5;
        for (int layer = mi->n_layers; layer >= 1; layer--) {
            int d_outer = core_half + 2 * layer;
            int d_inner = d_outer - 1;
            int row_size = 2 * d_outer - 1;
            aztec_read_compact_layer_bits(grid, w, h, fi, d_outer, d_inner, row_size,
                                          bits, &n, max_bits);
        }
        return n;
    }

    /* Full symbols: unchanged reference-grid-aware traversal. */
    for (int layer = 1; layer <= mi->n_layers; layer++) {
        LayerPair pairs[512];
        int n_pairs = build_full_layer_pairs(layer, pairs, 512);
        if (n_pairs <= 0) continue;
        for (int i = 0; i < n_pairs; i++) {
            if (n < max_bits) bits[n++] = read_mod(grid, w, h, fi, pairs[i].ar, pairs[i].ac);
            if (n < max_bits) bits[n++] = read_mod(grid, w, h, fi, pairs[i].br, pairs[i].bc);
        }
    }
    return n;
}

/* ── Bits to codewords ─────────────────────────────────────────────────── */

static int bits_to_codewords_from(const int *bits, int nbits, int start_bit, int cw_bits,
                                  int *cw, int max_cw)
{
    int n = 0;
    if (start_bit < 0) start_bit = 0;
    for (int i = start_bit; i + cw_bits <= nbits && n < max_cw; i += cw_bits) {
        int val = 0;
        for (int b = 0; b < cw_bits; b++)
            val = (val << 1) | bits[i + b];
        cw[n++] = val;
    }
    return n;
}

static int bits_to_codewords(const int *bits, int nbits, int cw_bits,
                             int *cw, int max_cw)
{
    return bits_to_codewords_from(bits, nbits, 0, cw_bits, cw, max_cw);
}

/* ── RS on data codewords ──────────────────────────────────────────────── */

static int rs_correct_data(int *cw, int n_total, int n_data, int cw_bits)
{
    int n_ec = n_total - n_data;
    if (n_ec <= 0) return 0;

    if (cw_bits == 6) {
        gf64_init();
        GField gf = { gf64_exp, gf64_log, 63 };
        return rs_decode(&gf, cw, n_total, n_ec);
    } else if (cw_bits == 8) {
        gf256_init();
        GField gf = { gf256_exp, gf256_log, 255 };
        return rs_decode(&gf, cw, n_total, n_ec);
    } else if (cw_bits == 12) {
        gf4096_init();
        GField gf = { gf4096_exp, gf4096_log, 4095 };
        return rs_decode(&gf, cw, n_total, n_ec);
    }
    /* 10-bit: GF(1024) not implemented, skip EC */
    return 0;
}

/* ── Data decoding (character modes) ───────────────────────────────────── */

enum {
    AZ_MODE_UPPER = 0,
    AZ_MODE_LOWER = 1,
    AZ_MODE_MIXED = 2,
    AZ_MODE_PUNCT = 3,
    AZ_MODE_DIGIT = 4
};

static const char az_upper[32] = {
    0, ' ', 'A', 'B', 'C', 'D', 'E', 'F',
    'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N',
    'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
    'W', 'X', 'Y', 'Z', 0, 0, 0, 0
};

static const char az_lower[32] = {
    0, ' ', 'a', 'b', 'c', 'd', 'e', 'f',
    'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v',
    'w', 'x', 'y', 'z', 0, 0, 0, 0
};

static const char az_mixed[32] = {
    0, '\0', '\1', '\2', '\3', '\4', '\5', '\6',
    '\7', '\10', '\12', '\13', '\14', '\15', '\33', '\34',
    '\35', '\36', '\37', '@', '\\', '^', '_', '`',
    '|', '~', '\177', 0, 0, 0, 0, 0
};

/* ISO/IEC 24778 Table 8 (Punctuation Mode Latch). Values 3-5 emit two
 * characters (". ", ", ", ": "); value 0 is FLG(n) and value 31 is U/L --
 * both are handled specially in decode_data() rather than via this table. */
static const char *az_punct[32] = {
    NULL, "\r", "\r\n", ". ", ", ", ": ", "!", "\"",
    "#", "$", "%", "&", "'", "(", ")", "*",
    "+", ",", "-", ".", "/", ":", ";", "<",
    "=", ">", "?", "[", "]", "{", "}", NULL
};

static const char az_digit[16] = {
    0, ' ', '0', '1', '2', '3', '4', '5',
    '6', '7', '8', '9', ',', '.', 0, 0
};

static int read_bits(const int *bs, int nbits, int *pos, int count)
{
    if (*pos + count > nbits) return -1;
    int val = 0;
    for (int i = 0; i < count; i++)
        val = (val << 1) | bs[(*pos)++];
    return val;
}

static int handle_binary_shift(const int *bs, int nbits, int *pos,
                               char *out, int out_cap, int *out_len)
{
    int blen = read_bits(bs, nbits, pos, 5);
    if (blen < 0) return -1;
    if (blen == 0) {
        int ext = read_bits(bs, nbits, pos, 11);
        if (ext < 0) return -1;
        blen = ext + 31;
    }
    for (int i = 0; i < blen; i++) {
        int byte = read_bits(bs, nbits, pos, 8);
        if (byte < 0) return -1;
        if (*out_len < out_cap - 1) {
            out[(*out_len)++] = (char)byte;
        }
    }
    return 0;
}

static int unstuff_data_codewords(const int *cw, int n_data, int cw_bits,
                                  int *bitstream, int max_bits)
{
    int nbits = 0;
    int mask = (1 << cw_bits) - 1;
    int stuffed_zero = 1;
    int stuffed_one = mask - 1;

    for (int i = 0; i < n_data; i++) {
        int val = cw[i];
        if (val < 0 || val > mask) return -1;
        if (val == 0 || val == mask) return -1;

        if (val == stuffed_zero || val == stuffed_one) {
            int bit = (val == stuffed_one) ? 1 : 0;
            for (int b = 0; b < cw_bits - 1; b++) {
                if (nbits >= max_bits) return -1;
                bitstream[nbits++] = bit;
            }
            continue;
        }

        for (int b = cw_bits - 1; b >= 0; b--) {
            if (nbits >= max_bits) return -1;
            bitstream[nbits++] = (val >> b) & 1;
        }
    }
    return nbits;
}

static int decode_data(const int *cw, int n_data, int cw_bits,
                       char *out, int out_cap)
{
    /* Flatten codewords to bitstream, removing stuff bits. */
    int bitstream[AZ_MAX_BITS];
    int nbits = unstuff_data_codewords(cw, n_data, cw_bits, bitstream, AZ_MAX_BITS);
    if (nbits < 0) return -1;

    int pos = 0;
    int mode = AZ_MODE_UPPER;
    int out_len = 0;
    int shift_mode = -1;

    while (pos < nbits && out_len < out_cap - 1) {
        int cur_mode = (shift_mode >= 0) ? shift_mode : mode;
        shift_mode = -1;

        int bits_needed = (cur_mode == AZ_MODE_DIGIT) ? 4 : 5;
        int val = read_bits(bitstream, nbits, &pos, bits_needed);
        if (val < 0) break;

        switch (cur_mode) {
        case AZ_MODE_UPPER:
            if (val == 0)       shift_mode = AZ_MODE_PUNCT;
            else if (val <= 27) out[out_len++] = az_upper[val];
            else if (val == 28) mode = AZ_MODE_LOWER;
            else if (val == 29) mode = AZ_MODE_MIXED;
            else if (val == 30) mode = AZ_MODE_DIGIT;
            else if (val == 31 &&
                     handle_binary_shift(bitstream, nbits, &pos, out, out_cap, &out_len) < 0) return -1;
            break;

        case AZ_MODE_LOWER:
            if (val == 0)       shift_mode = AZ_MODE_PUNCT;
            else if (val <= 27) out[out_len++] = az_lower[val];
            else if (val == 28) shift_mode = AZ_MODE_UPPER;
            else if (val == 29) mode = AZ_MODE_MIXED;
            else if (val == 30) mode = AZ_MODE_DIGIT;
            else if (val == 31 &&
                     handle_binary_shift(bitstream, nbits, &pos, out, out_cap, &out_len) < 0) return -1;
            break;

        case AZ_MODE_MIXED:
            if (val == 0)       shift_mode = AZ_MODE_PUNCT;
            else if (val <= 27) out[out_len++] = az_mixed[val];
            else if (val == 28) mode = AZ_MODE_PUNCT;
            else if (val == 29) mode = AZ_MODE_UPPER;
            else if (val == 31 &&
                     handle_binary_shift(bitstream, nbits, &pos, out, out_cap, &out_len) < 0) return -1;
            break;

        case AZ_MODE_PUNCT:
            if (val == 0) {
                if (read_bits(bitstream, nbits, &pos, 3) < 0) return -1; /* FLG(n), skip */
            } else if (val == 31) {
                mode = AZ_MODE_UPPER;
            } else {
                for (const char *p = az_punct[val]; p && *p && out_len < out_cap - 1; p++)
                    out[out_len++] = *p;
            }
            break;

        case AZ_MODE_DIGIT:
            if (val == 0)       shift_mode = AZ_MODE_PUNCT;
            else if (val <= 13) out[out_len++] = az_digit[val];
            else if (val == 14) mode = AZ_MODE_UPPER;
            else if (val == 15) shift_mode = AZ_MODE_UPPER;
            break;
        }
    }

    out[out_len] = '\0';
    return out_len;
}

static int aztec_estimate_full_layers_from_geometry(const FinderInfo *fi, int w, int h)
{
    if (!fi || w <= 0 || h <= 0) return -1;
    double ms = ((double)fi->mod_size_x + (double)fi->mod_size_y) / 32.0;
    if (ms < 1.0) return -1;
    int mods_w = (int)lrint((double)w / ms) - 4; /* assume ~2 modules quiet zone on each side */
    int mods_h = (int)lrint((double)h / ms) - 4;
    int mods = (mods_w < mods_h) ? mods_w : mods_h;
    int layers = (int)lrint(((double)mods - 15.0) / 4.0);
    if (layers < 1) layers = 1;
    if (layers > AZ_MAX_LAYERS) layers = AZ_MAX_LAYERS;
    return layers;
}

static int aztec_semantic_score(const char *s, int n,
                                int n_data_cw, int total_cw, int cw_bits)
{
    if (!s || n <= 0) return INT_MIN / 2;

    int printable = 0;
    int upper = 0;
    int lower = 0;
    int digit = 0;
    int spaces = 0;
    int punct = 0;
    int control = 0;
    int vowels = 0;
    int leading_space = (s[0] == ' ');
    int trailing_space = (s[n - 1] == ' ');

    for (int i = 0; i < n; i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch >= 32 && ch <= 126) {
            printable++;
            if (ch >= 'A' && ch <= 'Z') {
                upper++;
                if (ch == 'A' || ch == 'E' || ch == 'I' || ch == 'O' || ch == 'U') vowels++;
            } else if (ch >= 'a' && ch <= 'z') {
                lower++;
                if (ch == 'a' || ch == 'e' || ch == 'i' || ch == 'o' || ch == 'u') vowels++;
            }
            else if (ch >= '0' && ch <= '9') digit++;
            else if (ch == ' ') spaces++;
            else punct++;
        } else if (ch == '\r' || ch == '\n' || ch == '\t') {
            control++;
        } else {
            control += 2;
        }
    }

    int score = 0;
    score += printable * 5;
    score -= control * 30;
    score += upper * 10;
    score += digit * 4;
    score += spaces * 3;
    score += vowels * 8;
    score -= lower * 8;
    score -= punct * 10;

    if (printable * 100 < n * 70) score -= 120;
    if ((upper + digit + spaces) == 0) score -= 80;
    if (leading_space) score -= 80;
    if (trailing_space) score -= 60;

    score -= n * 4;
    if (n > 20) score -= (n - 20) * 12;
    if (n > 32) score -= (n - 32) * 16;

    if (total_cw > 0 && n_data_cw > 0) {
        score -= (n_data_cw * 180) / total_cw;
        score -= n_data_cw * 2;
    }
    if (cw_bits >= 8 && n > 18) {
        score -= (n - 18) * 3;
    }

    return score;
}

static int aztec_payload_quality_score(const char *s, int n, int *out_valid)
{
    if (!s || n <= 0) {
        if (out_valid) *out_valid = 0;
        return INT_MIN / 2;
    }

    int printable = 0;
    int upper = 0;
    int lower = 0;
    int digit = 0;
    int spaces = 0;
    int punct = 0;
    int control = 0;
    int letters = 0;
    int leading_space = (s[0] == ' ');
    int trailing_space = (s[n - 1] == ' ');

    for (int i = 0; i < n; i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch >= 32 && ch <= 126) {
            printable++;
            if (ch >= 'A' && ch <= 'Z') {
                upper++;
                letters++;
            } else if (ch >= 'a' && ch <= 'z') {
                lower++;
                letters++;
            } else if (ch >= '0' && ch <= '9') {
                digit++;
            } else if (ch == ' ') {
                spaces++;
            } else {
                punct++;
            }
        } else {
            control++;
        }
    }

    int valid = 1;
    if (n < 2 || n > 64) valid = 0;
    if (printable < n) valid = 0;
    if (leading_space || trailing_space) valid = 0;
    if (spaces * 100 > n * 45) valid = 0;
    if (punct * 100 > n * 40) valid = 0;

    int score = 0;
    score += upper * 8;
    score += digit * 4;
    score += spaces * 2;
    score -= lower * 3;
    score -= punct * 7;
    score -= control * 60;

    if (letters >= 3 && upper * 100 >= letters * 80) score += 24;
    if (spaces > 0 && spaces * 100 <= n * 20) score += 6;
    if (n > 20) score -= (n - 20) * 9;
    if (n > 36) score -= (n - 36) * 12;
    if (leading_space) score -= 50;
    if (trailing_space) score -= 50;

    /* "Title Case" (every space-separated word starts with an uppercase
     * letter) is a strong signal of genuine human-readable content -- the
     * heavy per-lowercase-letter penalty above otherwise unfairly punishes
     * perfectly legitimate mixed-case text (e.g. "Aztec Test"). Random
     * miscorrected garbage essentially never lands on this pattern. */
    if (letters >= 2) {
        int title_case = 1;
        int start_of_word = 1;
        int words = 0;
        for (int i = 0; i < n && title_case; i++) {
            unsigned char ch = (unsigned char)s[i];
            if (ch == ' ') { start_of_word = 1; continue; }
            if (start_of_word) {
                if (ch >= 'a' && ch <= 'z') { title_case = 0; break; }
                if (ch >= 'A' && ch <= 'Z') words++;
                start_of_word = 0;
            }
        }
        if (title_case && words >= 1) score += 26;
    }

    if (out_valid) *out_valid = valid;
    return score;
}

static int aztec_decode_full_without_mode(const uint8_t *grid, int w, int h,
                                          const FinderInfo *fi,
                                          char *out, int out_cap,
                                          int *out_score,
                                          int *out_layers)
{
    ModeInfo mi;
    int est_layers = aztec_estimate_full_layers_from_geometry(fi, w, h);
    if (est_layers < 1) return 0;

    mi.compact = 0;
    mi.n_layers = est_layers;
    if (est_layers <= 2)       mi.cw_bits = 6;
    else if (est_layers <= 8)  mi.cw_bits = 8;
    else if (est_layers <= 22) mi.cw_bits = 10;
    else                       mi.cw_bits = 12;
    mi.total_bits = full_layer_bits(est_layers);
    if (mi.total_bits <= 0 || mi.total_bits > AZ_MAX_BITS) return 0;
    mi.total_cw = mi.total_bits / mi.cw_bits;
    if (mi.total_cw <= 2 || mi.total_cw > AZ_MAX_DATA_CW) return 0;

    int *data_bits = (int *)malloc(sizeof(int) * (mi.total_bits + 64));
    if (!data_bits) return 0;
    int nbits = read_data_bits(grid, w, h, fi, &mi, data_bits, mi.total_bits + 64);
    int cw[AZ_MAX_DATA_CW];
    int n_cw = bits_to_codewords(data_bits, nbits, mi.cw_bits, cw, AZ_MAX_DATA_CW);
    free(data_bits);
    if (n_cw < mi.total_cw) return 0;

    int best_len = 0;
    int best_score = INT_MIN / 2;
    char best[BARCODE_MAX_LEN];
    best[0] = '\0';

    /* No mode message could be recovered, so n_data (and thus the EC split)
     * is unknown; guarding acceptance behind aztec_payload_quality_score's
     * validity gate -- in addition to the RS decoder's own 2*errors<=ec
     * bound -- keeps this exhaustive search from accepting a plausible-
     * looking miscorrection of random data as if it were a real payload. */
    for (int n_data = 1; n_data < mi.total_cw; n_data++) {
        int work[AZ_MAX_DATA_CW];
        char cand[BARCODE_MAX_LEN];
        memcpy(work, cw, (size_t)mi.total_cw * sizeof(int));
        if (rs_correct_data(work, mi.total_cw, n_data, mi.cw_bits) < 0) continue;
        int len = decode_data(work, n_data, mi.cw_bits, cand, sizeof(cand));
        if (len <= 0) continue;
        int cand_valid = 0;
        aztec_payload_quality_score(cand, len, &cand_valid);
        if (!cand_valid) continue;
        int score = aztec_semantic_score(cand, len, n_data, mi.total_cw, mi.cw_bits);
        if (score > best_score || (score == best_score && len < best_len)) {
            best_len = len;
            best_score = score;
            strncpy(best, cand, sizeof(best) - 1);
            best[sizeof(best) - 1] = '\0';
        }
    }

    if (best_len > 0) {
        int copy_len = (best_len < out_cap - 1) ? best_len : (out_cap - 1);
        memcpy(out, best, (size_t)copy_len);
        out[copy_len] = '\0';
        if (out_score) *out_score = best_score;
        if (out_layers) *out_layers = est_layers;
        return copy_len;
    }
    if (out_score) *out_score = INT_MIN / 2;
    if (out_layers) *out_layers = est_layers;
    return 0;
}

static int aztec_decode_with_finder(const uint8_t *grid, int w, int h,
                                    const FinderInfo *fi,
                                    char *out, int out_cap)
{
    int mode_bits[64];
    int n_mode_bits = 0;
    ModeInfo mi;
    int *data_bits;
    int nbits;
    int cw[AZ_MAX_DATA_CW];
    int n_cw;
    int total_cw;

    if (!fi || fi->mod_size_x < 8 || fi->mod_size_y < 8) return 0;

    if (fi->compact) {
        int mode_ok = 0;
        for (int rot = 0; rot < 4 && !mode_ok; rot++) {
            if (read_mode_bits_compact_rot(grid, w, h, fi, mode_bits, &n_mode_bits, rot) < 0)
                continue;
            if (decode_mode_compact(mode_bits, &mi) == 0)
                mode_ok = 1;
        }
        if (!mode_ok) return 0;
        g_aztec_debug_stats.mode_decode_hits++;
    } else {
        int mode_ok = 0;
        for (int rot = 0; rot < 4 && !mode_ok; rot++) {
            if (read_mode_bits_full_rot(grid, w, h, fi, mode_bits, &n_mode_bits, rot) < 0)
                continue;
            if (decode_mode_full(mode_bits, &mi) == 0)
                mode_ok = 1;
        }
        if (!mode_ok) {
            int fallback_layers = 0;
            int fallback_len = aztec_decode_full_without_mode(grid, w, h, fi, out, out_cap,
                                                              NULL, &fallback_layers);
            if (fallback_len > 0) {
                g_aztec_debug_stats.data_extract_hits++;
                g_aztec_debug_stats.ec_decode_hits++;
            }
            return fallback_len;
        }
        g_aztec_debug_stats.mode_decode_hits++;
    }

    if (mi.total_bits <= 0 || mi.total_bits > AZ_MAX_BITS) return 0;

    data_bits = (int *)malloc(sizeof(int) * (mi.total_bits + 64));
    if (!data_bits) return -1;

    nbits = read_data_bits(grid, w, h, fi, &mi, data_bits, mi.total_bits + 64);
    {
        /* Compact rings are read outer-to-inner and the codeword grouping
         * starts after skipping the leading padding bits (nbits % cw_bits). */
        int skip = (fi->compact && mi.cw_bits > 0) ? (nbits % mi.cw_bits) : 0;
        n_cw = bits_to_codewords_from(data_bits, nbits, skip, mi.cw_bits, cw, AZ_MAX_DATA_CW);
    }
    free(data_bits);
    g_aztec_debug_stats.data_extract_hits++;
    AZ_DBG("AZTEC_DEBUG: nbits=%d n_cw=%d n_data_cw=%d total_cw=%d cw_bits=%d compact=%d\n",
           nbits, n_cw, mi.n_data_cw, mi.total_cw, mi.cw_bits, fi->compact);

    if (n_cw < mi.n_data_cw) return 0;

    total_cw = (n_cw < mi.total_cw) ? n_cw : mi.total_cw;
    if (total_cw < mi.n_data_cw || total_cw > AZ_MAX_DATA_CW) return 0;
#if AZTEC_DEBUG_PRINT
    {
        AZ_DBG("AZTEC_DEBUG: data cw before RS:");
        for (int i = 0; i < total_cw; i++) AZ_DBG(" %x", cw[i]);
        AZ_DBG("\n");
    }
#endif
    if (rs_correct_data(cw, total_cw, mi.n_data_cw, mi.cw_bits) < 0) {
        AZ_DBG("AZTEC_DEBUG: RS correct data failed\n");
        return 0;
    }
    g_aztec_debug_stats.ec_decode_hits++;
#if AZTEC_DEBUG_PRINT
    {
        AZ_DBG("AZTEC_DEBUG: data cw after RS:");
        for (int i = 0; i < mi.n_data_cw; i++) AZ_DBG(" %x", cw[i]);
        AZ_DBG("\n");
    }
#endif
    {
        char primary[BARCODE_MAX_LEN];
        int primary_len = decode_data(cw, mi.n_data_cw, mi.cw_bits, primary, sizeof(primary));
        AZ_DBG("AZTEC_DEBUG: primary_len=%d data=[%.*s]\n", primary_len, primary_len > 0 ? primary_len : 0, primary);
        if (primary_len <= 0) return 0;

        int best_len = primary_len;
        strncpy(out, primary, (size_t)out_cap - 1);
        out[out_cap - 1] = '\0';

        /* The mode message already decoded and passed its own RS check, so
         * the mode-less brute-force fallback (which is far more prone to
         * accepting a miscorrected/wrong-guess payload) is not consulted
         * here anymore -- it only runs when the mode message itself could
         * not be recovered at all (see the !mode_ok branch above). */
        return best_len;
    }
}

int aztec_test_decode_mode_compact_bits(const int *bits, int *layers, int *data_cw,
                                        int *ec_cw, int *cw_bits, int *total_cw)
{
    ModeInfo mi;
    if (!bits) return -1;
    if (decode_mode_compact(bits, &mi) < 0) return -1;
    if (layers) *layers = mi.n_layers;
    if (data_cw) *data_cw = mi.n_data_cw;
    if (ec_cw) *ec_cw = mi.ec_cw;
    if (cw_bits) *cw_bits = mi.cw_bits;
    if (total_cw) *total_cw = mi.total_cw;
    return 0;
}

int aztec_test_decode_mode_full_bits(const int *bits, int *layers, int *data_cw,
                                     int *ec_cw, int *cw_bits, int *total_cw)
{
    ModeInfo mi;
    if (!bits) return -1;
    if (decode_mode_full(bits, &mi) < 0) return -1;
    if (layers) *layers = mi.n_layers;
    if (data_cw) *data_cw = mi.n_data_cw;
    if (ec_cw) *ec_cw = mi.ec_cw;
    if (cw_bits) *cw_bits = mi.cw_bits;
    if (total_cw) *total_cw = mi.total_cw;
    return 0;
}

int aztec_test_decode_data_codewords(const int *cw, int n_data, int cw_bits,
                                     char *out, int out_cap)
{
    if (!cw || !out || out_cap < 2 || n_data <= 0) return -1;
    if (!(cw_bits == 6 || cw_bits == 8 || cw_bits == 10 || cw_bits == 12)) return -1;
    return decode_data(cw, n_data, cw_bits, out, out_cap);
}

int aztec_test_find_bullseye_geometry(const uint8_t *grid, int w, int h,
                                      int *cx, int *cy, int *compact,
                                      int *mod_size_x_pos, int *mod_size_x_neg,
                                      int *mod_size_y_pos, int *mod_size_y_neg)
{
    FinderInfo fi;
    if (!grid || w < 11 || h < 11) return -1;
    if (!find_bullseye(grid, w, h, &fi)) return -1;
    if (cx) *cx = fi.cx;
    if (cy) *cy = fi.cy;
    if (compact) *compact = fi.compact;
    if (mod_size_x_pos) *mod_size_x_pos = fi.mod_size_x_pos;
    if (mod_size_x_neg) *mod_size_x_neg = fi.mod_size_x_neg;
    if (mod_size_y_pos) *mod_size_y_pos = fi.mod_size_y_pos;
    if (mod_size_y_neg) *mod_size_y_neg = fi.mod_size_y_neg;
    return 0;
}

int aztec_test_build_full_layer_pairs(int layer, int *pairs, int max_pairs)
{
    if (layer < 1 || !pairs || max_pairs <= 0) return -1;
    int max_out_pairs = max_pairs / 4;
    if (max_out_pairs <= 0) return -1;

    LayerPair layer_pairs[512];
    int n_pairs = build_full_layer_pairs(layer, layer_pairs, 512);
    if (n_pairs < 0) return -1;
    if (n_pairs > max_out_pairs) n_pairs = max_out_pairs;

    for (int i = 0; i < n_pairs; i++) {
        pairs[i * 4 + 0] = layer_pairs[i].ar;
        pairs[i * 4 + 1] = layer_pairs[i].ac;
        pairs[i * 4 + 2] = layer_pairs[i].br;
        pairs[i * 4 + 3] = layer_pairs[i].bc;
    }
    return n_pairs;
}

static int aztec_try_decode_with_jitter(const uint8_t *grid, int w, int h,
                                        const FinderInfo *seed,
                                        int center_jitter, int scale_jitter,
                                        char *out, int out_cap)
{
    int cj = center_jitter;
    int sj = scale_jitter;
    const int scale_step_fp = 8; /* 1/2 pixel in fixed16 per jitter unit */
    if (!seed) return 0;
    if (cj < 0) cj = 0;
    if (sj < 0) sj = 0;

    for (int dy = -cj; dy <= cj; dy++) {
        for (int dx = -cj; dx <= cj; dx++) {
            for (int sy = -sj; sy <= sj; sy++) {
                for (int sx = -sj; sx <= sj; sx++) {
                    FinderInfo alt = *seed;
                    int len;
                    alt.cx += dx;
                    alt.cy += dy;
                    alt.mod_size_x += sx * scale_step_fp;
                    alt.mod_size_y += sy * scale_step_fp;
                    alt.mod_size_x_pos += sx * scale_step_fp;
                    alt.mod_size_x_neg += sx * scale_step_fp;
                    alt.mod_size_y_pos += sy * scale_step_fp;
                    alt.mod_size_y_neg += sy * scale_step_fp;
                    if (alt.cx < 0 || alt.cx >= w || alt.cy < 0 || alt.cy >= h) continue;
                    if (alt.mod_size_x < 8 || alt.mod_size_y < 8) continue;
                    len = aztec_decode_with_finder(grid, w, h, &alt, out, out_cap);
                    if (len > 0) {
                        AZ_DBG("AZTEC_DEBUG: jitter hit dx=%d dy=%d sx=%d sy=%d compact=%d len=%d data=[%.*s]\n",
                               dx, dy, sx, sy, alt.compact, len, len, out);
                        return len;
                    }
                }
            }
        }
    }
    return 0;
}

/* ── Main decode pipeline ──────────────────────────────────────────────── */

static int aztec_decode_grid_polarity(const uint8_t *grid, int w, int h, char *out, int out_cap)
{
    FinderInfo fi;
    if (!find_bullseye(grid, w, h, &fi)) return 0;
    g_aztec_debug_stats.bullseye_hits++;
    int cj = g_aztec_tuning.fallback_center_jitter;
    int sj = g_aztec_tuning.fallback_scale_jitter;

    int len = aztec_try_decode_with_jitter(grid, w, h, &fi, cj, sj, out, out_cap);
    if (len > 0) {
        g_aztec_debug_stats.payload_decode_hits++;
        return len;
    }
    {
        FinderInfo flip = fi;
        flip.compact = !flip.compact;
        len = aztec_try_decode_with_jitter(grid, w, h, &flip, cj, sj, out, out_cap);
        if (len > 0) {
            g_aztec_debug_stats.payload_decode_hits++;
            return len;
        }
    }

    /* Phase 20C fallback: search additional bull's-eye candidates near image center. */
    {
        int mid_x = w / 2;
        int mid_y = h / 2;
        int max_dim = (w < h) ? w : h;
        int radius = max_dim / 4;
        int attempts = 0;
        int max_attempts = 48;
        if (radius < 12) radius = 12;
        if (radius > 96) radius = 96;

        for (int y = mid_y - radius; y <= mid_y + radius && attempts < max_attempts; y += 2) {
            for (int x = mid_x - radius; x <= mid_x + radius && attempts < max_attempts; x += 2) {
                FinderInfo alt;
                if (x < 3 || y < 3 || x >= w - 3 || y >= h - 3) continue;
                if (!is_black(grid, w, h, x, y)) continue;
                if (!check_bullseye(grid, w, h, x, y, &alt)) continue;
                attempts++;

                len = aztec_try_decode_with_jitter(grid, w, h, &alt, cj, sj, out, out_cap);
                if (len > 0) {
                    g_aztec_debug_stats.payload_decode_hits++;
                    return len;
                }

                /* Also retry with compact/full hypothesis flipped. */
                alt.compact = !alt.compact;
                len = aztec_try_decode_with_jitter(grid, w, h, &alt, cj, sj, out, out_cap);
                if (len > 0) {
                    g_aztec_debug_stats.payload_decode_hits++;
                    return len;
                }
            }
        }
    }
    return 0;
}

int aztec_decode_grid(const uint8_t *grid, int w, int h, char *out, int out_cap)
{
    if (!grid || w < 11 || h < 11 || !out || out_cap < 2) return 0;
    g_aztec_debug_stats.decode_attempts++;

    int len = aztec_decode_grid_polarity(grid, w, h, out, out_cap);
    if (len > 0) return len;

    /* Retry with an inverted grid to support reverse-video symbols (white
     * modules on a black background); is_black()/measure_run() are always
     * polarity-dependent, so the simplest robust fix is to physically
     * invert the pixel buffer and rerun the whole detection pipeline. */
    {
        size_t n_px = (size_t)w * (size_t)h;
        uint8_t *inv = (uint8_t *)malloc(n_px);
        if (!inv) return 0;
        for (size_t i = 0; i < n_px; i++) inv[i] = (uint8_t)(255 - grid[i]);
        AZ_DBG("AZTEC_DEBUG: retrying with inverted polarity\n");
        len = aztec_decode_grid_polarity(inv, w, h, out, out_cap);
        free(inv);
    }
    return len;
}

static int aztec_normalize_ascii_payload(char *s, int len)
{
    if (!s || len <= 0) return len;

    int n = len;
    for (int i = 0; i < len; i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 32 || ch > 126) {
            n = i;
            break;
        }
    }
    while (n > 0 && s[n - 1] == ' ') n--;
    s[n] = '\0';
    return n;
}

static int aztec_is_checkerboard_noise(const uint8_t *grid, int w, int h)
{
    if (!grid || w < 24 || h < 24) return 0;

    int total_pixels = w * h;
    int black = 0;
    for (int i = 0; i < total_pixels; i++) {
        if (grid[i] < 128) black++;
    }
    int black_pct = (black * 100) / total_pixels;
    if (black_pct < 35 || black_pct > 65) return 0;

    int h_pairs = h * (w - 1);
    int v_pairs = (h - 1) * w;
    int h_flip = 0;
    int v_flip = 0;
    for (int y = 0; y < h; y++) {
        const uint8_t *row = grid + y * w;
        for (int x = 0; x + 1 < w; x++) {
            int a = row[x] < 128;
            int b = row[x + 1] < 128;
            if (a != b) h_flip++;
        }
    }
    for (int y = 0; y + 1 < h; y++) {
        const uint8_t *row0 = grid + y * w;
        const uint8_t *row1 = grid + (y + 1) * w;
        for (int x = 0; x < w; x++) {
            int a = row0[x] < 128;
            int b = row1[x] < 128;
            if (a != b) v_flip++;
        }
    }

    int h_flip_pct = (h_pairs > 0) ? (h_flip * 100) / h_pairs : 0;
    int v_flip_pct = (v_pairs > 0) ? (v_flip * 100) / v_pairs : 0;
    if (h_flip_pct < 85 || v_flip_pct < 85) return 0;

    int cells = (w - 1) * (h - 1);
    int checker_cells = 0;
    for (int y = 0; y + 1 < h; y++) {
        const uint8_t *row0 = grid + y * w;
        const uint8_t *row1 = grid + (y + 1) * w;
        for (int x = 0; x + 1 < w; x++) {
            int a = row0[x] < 128;
            int b = row0[x + 1] < 128;
            int c = row1[x] < 128;
            int d = row1[x + 1] < 128;
            if ((a == d) && (b == c) && (a != b)) checker_cells++;
        }
    }
    int checker_pct = (cells > 0) ? (checker_cells * 100) / cells : 0;
    return checker_pct >= 70;
}


int aztec_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !img->pixels || img->width < 11 || img->height < 11)
        return 0;
    if (!result) return 0;

    if (aztec_is_checkerboard_noise(img->pixels, img->width, img->height))
        return 0;

    int len = aztec_decode_grid(img->pixels, img->width, img->height,
                                result->data, BARCODE_MAX_LEN);
    if (len <= 0) return 0;

    len = aztec_normalize_ascii_payload(result->data, len);
    if (len <= 0) return 0;

    {
        int valid = 0;
        int quality = aztec_payload_quality_score(result->data, len, &valid);
        if (!valid || quality < 18) {
            return 0;
        }
    }

    result->length = len;
    result->confidence = 80;
    return 1;
}

void aztec_test_set_full_sampling_tuning(int side_samples_bias, int side_round_bias,
                                         int center_jitter, int scale_jitter)
{
    g_aztec_tuning.full_side_samples_bias = side_samples_bias;
    g_aztec_tuning.full_side_round_bias = side_round_bias;
    g_aztec_tuning.fallback_center_jitter = center_jitter;
    g_aztec_tuning.fallback_scale_jitter = scale_jitter;
}

void aztec_test_get_full_sampling_tuning(int *side_samples_bias, int *side_round_bias,
                                         int *center_jitter, int *scale_jitter)
{
    if (side_samples_bias) *side_samples_bias = g_aztec_tuning.full_side_samples_bias;
    if (side_round_bias) *side_round_bias = g_aztec_tuning.full_side_round_bias;
    if (center_jitter) *center_jitter = g_aztec_tuning.fallback_center_jitter;
    if (scale_jitter) *scale_jitter = g_aztec_tuning.fallback_scale_jitter;
}

void aztec_test_reset_full_sampling_tuning(void)
{
    g_aztec_tuning = k_aztec_tuning_default;
}

void aztec_debug_stats_reset(void)
{
    memset(&g_aztec_debug_stats, 0, sizeof(g_aztec_debug_stats));
}

void aztec_debug_stats_get(AztecDecodeDebugStats *out)
{
    if (!out) {
        return;
    }
    *out = g_aztec_debug_stats;
}
