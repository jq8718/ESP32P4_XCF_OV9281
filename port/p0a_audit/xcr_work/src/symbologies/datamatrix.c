/*
 * DataMatrix decoder restoration.
 *
 * Milestone M1: adaptive edge threshold + tile-spaced scanline edge
 * extraction + row/column chain linking + solid(L)/clock edge
 * classification + L-shape finder assembly.
 *
 * Milestone M2: clock-edge module counting + square(24)/rectangle(6) size
 * determination + perspective grid sampling + finder-border stripping into
 * a data-region mapping matrix.
 *
 * Milestone M3: ECC200 module bit -> codeword placement extraction
 * (ISO/IEC 16022 Annex F), a faithful transcription of the reference's
 * SmartSymbolRedef00014c / sub_5AFF0 placement walk.
 *
 * Milestone M5: data codeword -> text payload decode (ASCII / C40 / Text /
 * X12 / EDIFACT / Base256 / FNC1 / ECI / Macro 05-06), a faithful
 * transcription of the reference's SmartSymbolRedef00013f state machine
 * and its sub_59CB8 triple-value emitter. See Section 15 below.
 *
 * Sections 1-3 are faithfully transcribed from the decompiled reference
 * (docs/XCF_armeabi-v7a/libFsaDecoderCore1.c):
 *   - dmfi_thresh              (263751-263824)  -> dm_thresh()
 *   - SmartSymbolRedef0000f2   (263831-263955)  -> dm_init()
 *   - SmartSymbolRedef000002   (62526-...)      -> dm_grad5()
 *   - sub_16AB20 / sub_16B1E0  (264046 / 264378) -> dm_scan_h()/dm_scan_v()
 *
 * Sections 4-6 (chain linking, solid/clock edge classification, L-shape
 * assembly) are design-layer extrapolations: the reference entry point
 * sub_16E70C is only a weak prototype in the decompiled source and the
 * classification/assembly logic sits in an unread region. This part
 * reuses the engine's own data structures (tile grid, dual-polarity edge
 * tables) rather than inventing an unrelated algorithm, and is documented
 * as such rather than claimed as a literal transcription.
 *
 * Section 7 (size tables) is a faithful transcription verified against the
 * reference:
 *   - dword_19E260/19E2C0/19E320 (7013/7014/7041) -> squares[] blocks/data/ec
 *   - SmartSymbolRedef000138   (56852) -> square lookup by (mapping dim-8)
 *   - unk_1A12C4 index switch (238837-238868) -> rects[] index by mapping
 *     (rows,cols); the table body itself is a weak symbol with no data in
 *     the decompiled dump, so data/ec codeword counts are the standard
 *     ISO/IEC 16022 values (cross-checked: data_cw+ec_cw == total_cw for
 *     every entry, same invariant the square table satisfies).
 *   - size = mapping_dim + 2*regions, regions-per-size switch
 *     (238604-238650) -> squares[]/rects[] regs_h/regs_v grouping
 *     (0..18->1, 24..44->2(step4), 56..96->4(step8), 112..136->6(step12)).
 *
 * Sections 8-11 (module counting, homography grid sampling, finder-border
 * stripping) are design-layer extrapolations of the reference's two
 * complementary approaches (48x48 clock-period cross-correlation in
 * sub_5829C, and the jump-walking counter in sub_148F04 / 0006016220):
 * the FFT-like cost-table period search relies on an engine-global cost
 * table (unk_2069A8) whose data is not present in the decompiled dump, so
 * this restoration instead counts black/white transitions directly along
 * each clock (dashed) edge -- functionally the same "count periods along
 * the clock track" idea as path B, minus the sub-pixel walking state
 * machine. The perspective solve reuses the same general 4-point DLT
 * homography approach already used by qrcode.c's Perspective/
 * compute_perspective (own local copy here, no shared header changes).
 *
 * Milestone M4: Reed-Solomon error correction over GF(256) plus the
 * round-robin block de-interleaving used by DataMatrix's multi-block ECC
 * (ISO/IEC 16022 Annex E), a faithful transcription of the reference's:
 *   - SmartSymbolRedef00014f  (264769) -> gf256_init() (GF(256)/0x12d field
 *     selection; the mode-indexed jump table itself is not reproduced,
 *     only the DataMatrix branch it selects)
 *   - SmartSymbolRedef00014e  (265122) -> rs_decode() (own local copy of
 *     the GF(256) Berlekamp-Massey/Forney decoder already used by
 *     aztec.c, modified to return the corrected-symbol count instead of
 *     a bare 0/-1 status so the confidence formula below can use it)
 *   - SmartSymbolRedef000100  (263447-263623) -> dm_rs_correct(): single-
 *     block path, multi-block round-robin de-interleave + per-block RS +
 *     confidence-as-minimum, and the nd==1558 (144x144) two-convention
 *     retry. The reference reads/writes its flattened codeword array in
 *     reverse order around each RS call (via SmartSymbolRedef00033c
 *     in-place reversal, 263581/263583); rs_decode()'s cw[0]=highest-order
 *     convention already matches a *forward* read of our (M3-produced,
 *     already forward-ordered) codeword array, so the reversal step is
 *     mathematically absorbed and intentionally not reproduced literally.
 */

#include "datamatrix.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <time.h>

#define DM_DEBUG_PRINT 0
#if DM_DEBUG_PRINT
#define DM_DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define DM_DBG(...) ((void)0)
#endif

/* ── Debug statistics ─────────────────────────────────────────────────── */

static DataMatrixDecodeDebugStats g_dm_stats;

/* Internal-only counter (not part of the public debug-stats ABI in
 * datamatrix.h): how many successful decodes required the mirror=1 pass
 * (M6). Handy for local regression checks via DM_DBG; not exposed through
 * datamatrix_debug_stats_get(). */
static uint32_t g_dm_mirror_hits;

static uint64_t dm_now_us(void)
{
    return (uint64_t)clock() * 1000000ULL / (uint64_t)CLOCKS_PER_SEC;
}

void datamatrix_debug_stats_reset(void)
{
    memset(&g_dm_stats, 0, sizeof(g_dm_stats));
    g_dm_mirror_hits = 0;
}

void datamatrix_debug_stats_get(DataMatrixDecodeDebugStats *out)
{
    if (out) {
        *out = g_dm_stats;
    }
}

/* ── Small helpers ────────────────────────────────────────────────────── */

static inline int dm_clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline uint8_t dm_pixel(const Image *img, int x, int y)
{
    x = dm_clampi(x, 0, img->width - 1);
    y = dm_clampi(y, 0, img->height - 1);
    return img->pixels[(size_t)y * img->width + x];
}

/* ── Section 1: dm_thresh() — faithful transcription of dmfi_thresh ─────
 * Adaptive edge-gradient threshold derived from the mid-height row's
 * first difference, bucketed into a 64-bin histogram (bin width 4), with
 * a 7/8 quantile cutoff mapped back via 4*bin+8, clamped to 127.
 */
static int dm_thresh(const Image *img, int32_t *rowbuf)
{
    int W = img->width;
    int hist[64];
    memset(hist, 0, sizeof(hist));

    int y = img->height / 2;
    for (int i = 0; i < W; i++) {
        rowbuf[i] = dm_pixel(img, i, y);
    }

    /* in-place adjacent difference: buf[i] = buf[i] - buf[i+1], W-1 terms */
    for (int i = 0; i + 1 < W; i++) {
        rowbuf[i] = rowbuf[i] - rowbuf[i + 1];
    }

    int hist0 = 0;
    if (W >= 2) {
        for (int i = 0; i + 1 < W; i++) {
            int32_t d = rowbuf[i];
            int mag = d < 0 ? -d : d;
            int bin = mag >> 2;
            if (bin > 63) bin = 63;
            hist[bin]++;
        }
        hist0 = hist[0];
    }

    int target = 7 * (W - hist0) / 8;
    int cum = 0;
    for (int bin = 1; bin <= 63; bin++) {
        cum += hist[bin];
        if (cum > target) {
            int T = 4 * bin + 8;
            return (T >= 127) ? 127 : T;
        }
    }
    return 127;
}

/* ── Section 2: dm_init() — faithful transcription of the module-init
 * tile geometry from SmartSymbolRedef0000f2. The engine-global config
 * knobs that select tile=8/16/32 and the histogram-vs-fixed threshold
 * path do not exist in this restored codebase, so we use the documented
 * defaults (tile=32, base=30, histogram threshold enabled).
 */
typedef struct {
    int W, H;
    int tile;
    int tile_3half;
    int tile_half;
    int tile_dbl;
    int tcols, trows;
    int cx, cy;
    int base_T;
    int step;
    int edge_T;
} DmCtx;

static void dm_init(const Image *img, DmCtx *ctx, int32_t *rowbuf)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->W = img->width;
    ctx->H = img->height;

    int tile = 32; /* unk_27387C == 0 default */
    ctx->tile = tile;
    ctx->tile_3half = 3 * tile / 2;
    ctx->tile_half = tile / 2;
    ctx->tile_dbl = 2 * tile;

    int v5 = 2 * tile;
    int v6 = 2 * tile - 1;
    ctx->tcols = (v6 + ctx->W) / v5;
    ctx->trows = (v6 + ctx->H) / v5;
    ctx->cx = ctx->W / 2;
    ctx->cy = ctx->H / 2;

    ctx->base_T = 30; /* unk_273770==0, unk_27377C==0 default */
    ctx->step = 1;
    ctx->edge_T = dm_thresh(img, rowbuf);
}

/* ── Section 3: 5-tap gradient + peak detection ──────────────────────────
 * Faithful to SmartSymbolRedef000002's kernel [+1,+1,0,-1,-1] and the
 * sub_16AB20/sub_16B1E0 peak rule (v > T && v >= neighbours, strict
 * 3-point local extremum test on each polarity independently).
 * The n5/N block-of-5 sizing in the reference is an assembly-loop
 * unrolling artifact; we compute the gradient over the full valid range
 * for identical numerical results without the tail truncation.
 */
static void dm_grad5(const int32_t *p, int n, int32_t *g /* length n-4 */)
{
    for (int i = 0; i + 4 < n; i++) {
        g[i] = (p[i] + p[i + 1]) - (p[i + 3] + p[i + 4]);
    }
}

#define DM_MAX_EDGES_PER_LINE 512

typedef struct {
    int fixed;               /* y for h-scan rows, x for v-scan columns */
    int xs_pos[DM_MAX_EDGES_PER_LINE];
    int n_pos;
    int xs_neg[DM_MAX_EDGES_PER_LINE];
    int n_neg;
} DmScanLine;

/* Extract positive/negative gradient peaks along one 1-D pixel profile. */
static void dm_find_peaks(const int32_t *g, int n, int edge_T,
                           int base_off, int *out_pos, int *n_pos,
                           int *out_neg, int *n_neg, int cap)
{
    *n_pos = 0;
    *n_neg = 0;
    for (int i = 0; i + 2 < n; i++) {
        int v = g[i + 1];
        if (v > edge_T && v >= g[i] && v >= g[i + 2]) {
            if (*n_pos < cap) out_pos[(*n_pos)++] = base_off + i + 1;
        }
        if (v < -edge_T && v <= g[i] && v <= g[i + 2]) {
            if (*n_neg < cap) out_neg[(*n_neg)++] = base_off + i + 1;
        }
    }
}

/* ── Section 4 (design extrapolation): chain linking ─────────────────────
 * Links same-polarity edge points across consecutive tile-spaced scan
 * lines into straight chains, terminating a chain as soon as one scan
 * line fails to extend it. Chains with >=3 points become candidate line
 * segments (least-squares fit of the varying coordinate against the
 * fixed scan coordinate).
 */
typedef enum { DM_EDGE_UNKNOWN = 0, DM_EDGE_SOLID, DM_EDGE_CLOCK } DmEdgeType;

typedef struct {
    int axis;        /* 0 = from horizontal scan (near-vertical line) */
                      /* 1 = from vertical scan (near-horizontal line) */
    int polarity;     /* +1 / -1 */
    int n;
    double fx0, fv0;  /* first point (fixed, var) */
    double fx1, fv1;  /* last point (fixed, var) */
    double a, b;      /* var = a*fixed + b (least-squares) */
    double px0, py0, px1, py1; /* endpoints in image (x,y) space */
    double dx, dy;    /* unit direction vector */
    double len;
    DmEdgeType type;
    double module_size;
} DmSeg;

typedef struct {
    int active;
    int polarity;
    int n;
    double sum_f, sum_v, sum_fv, sum_ff;
    double first_f, first_v;
    double last_f, last_v;
} DmChain;

#define DM_MAX_CHAINS 256
#define DM_MAX_SEGS 512

static void dm_chain_to_seg(const DmChain *c, int axis, DmSeg *out)
{
    double n = (double)c->n;
    double denom = n * c->sum_ff - c->sum_f * c->sum_f;
    double a, b;
    if (fabs(denom) > 1e-9) {
        a = (n * c->sum_fv - c->sum_f * c->sum_v) / denom;
        b = (c->sum_v - a * c->sum_f) / n;
    } else {
        a = 0.0;
        b = c->sum_v / n;
    }
    out->axis = axis;
    out->polarity = c->polarity;
    out->n = c->n;
    out->fx0 = c->first_f; out->fv0 = c->first_v;
    out->fx1 = c->last_f;  out->fv1 = c->last_v;
    out->a = a; out->b = b;

    if (axis == 0) {
        /* fixed = y, var = x */
        out->px0 = c->first_v; out->py0 = c->first_f;
        out->px1 = c->last_v;  out->py1 = c->last_f;
    } else {
        /* fixed = x, var = y */
        out->px0 = c->first_f; out->py0 = c->first_v;
        out->px1 = c->last_f;  out->py1 = c->last_v;
    }
    double ddx = out->px1 - out->px0;
    double ddy = out->py1 - out->py0;
    double len = sqrt(ddx * ddx + ddy * ddy);
    out->len = len;
    if (len > 1e-6) {
        out->dx = ddx / len;
        out->dy = ddy / len;
    } else {
        out->dx = 0; out->dy = 0;
    }
    out->type = DM_EDGE_UNKNOWN;
    out->module_size = 0.0;
}

/* Links one polarity's edge points across scan lines into DmSeg entries. */
static int dm_link_edges(const DmScanLine *lines, int nlines, int polarity,
                          int axis, int tile_half, DmSeg *out_segs, int max_segs)
{
    DmChain chains[DM_MAX_CHAINS];
    int nchains = 0;
    int nsegs = 0;

    for (int li = 0; li < nlines; li++) {
        const int *xs = (polarity > 0) ? lines[li].xs_pos : lines[li].xs_neg;
        int cnt = (polarity > 0) ? lines[li].n_pos : lines[li].n_neg;
        int fixed = lines[li].fixed;

        int matched_point[DM_MAX_EDGES_PER_LINE];
        memset(matched_point, 0, sizeof(int) * (size_t)cnt);

        for (int ci = 0; ci < nchains; ci++) {
            if (!chains[ci].active) continue;
            double predicted = chains[ci].last_v;
            int best = -1;
            double best_d = 1e18;
            for (int pi = 0; pi < cnt; pi++) {
                if (matched_point[pi]) continue;
                double d = fabs((double)xs[pi] - predicted);
                if (d < best_d) { best_d = d; best = pi; }
            }
            if (best >= 0 && best_d <= (double)tile_half) {
                matched_point[best] = 1;
                DmChain *c = &chains[ci];
                c->n++;
                c->sum_f += fixed;
                c->sum_v += xs[best];
                c->sum_fv += (double)fixed * xs[best];
                c->sum_ff += (double)fixed * fixed;
                c->last_f = fixed;
                c->last_v = xs[best];
            } else {
                /* chain not extended this line: terminate */
                if (chains[ci].n >= 3 && nsegs < max_segs) {
                    dm_chain_to_seg(&chains[ci], axis, &out_segs[nsegs]);
                    nsegs++;
                }
                chains[ci].active = 0;
            }
        }

        /* compact inactive chains out */
        int w = 0;
        for (int r = 0; r < nchains; r++) {
            if (chains[r].active) chains[w++] = chains[r];
        }
        nchains = w;

        for (int pi = 0; pi < cnt; pi++) {
            if (matched_point[pi]) continue;
            if (nchains >= DM_MAX_CHAINS) continue;
            DmChain *c = &chains[nchains++];
            c->active = 1;
            c->polarity = polarity;
            c->n = 1;
            c->sum_f = fixed;
            c->sum_v = xs[pi];
            c->sum_fv = (double)fixed * xs[pi];
            c->sum_ff = (double)fixed * fixed;
            c->first_f = fixed; c->first_v = xs[pi];
            c->last_f = fixed;  c->last_v = xs[pi];
        }
    }

    for (int ci = 0; ci < nchains; ci++) {
        if (chains[ci].n >= 3 && nsegs < max_segs) {
            dm_chain_to_seg(&chains[ci], axis, &out_segs[nsegs]);
            nsegs++;
        }
    }
    return nsegs;
}

/* ── Section 5 (design extrapolation): solid vs clock edge classification
 * Samples along the fitted segment, offset a few pixels into the dark
 * side (chosen from the gradient polarity + scan axis), and inspects the
 * black/white transition pattern.
 */
static void dm_classify_edge(const Image *img, DmSeg *seg, int tile)
{
    int K = (int)(seg->len / ((double)tile / 4.0));
    K = dm_clampi(K, 8, 64);

    /* perpendicular to segment direction */
    double nx = -seg->dy, ny = seg->dx;
    /* pick the sign of the normal that points to the "dark side"
     * implied by scan axis + polarity of the originating gradient peak */
    double target_x = 0.0, target_y = 0.0;
    if (seg->axis == 0) {
        target_x = (seg->polarity > 0) ? 1.0 : -1.0;
    } else {
        target_y = (seg->polarity > 0) ? 1.0 : -1.0;
    }
    if (nx * target_x + ny * target_y < 0) { nx = -nx; ny = -ny; }

    double s = 2.5; /* inward offset in px */
    int vals[64];
    for (int k = 0; k < K; k++) {
        double t = (K > 1) ? (double)k / (double)(K - 1) : 0.0;
        double qx = seg->px0 + (seg->px1 - seg->px0) * t + nx * s;
        double qy = seg->py0 + (seg->py1 - seg->py0) * t + ny * s;
        vals[k] = dm_pixel(img, (int)lround(qx), (int)lround(qy));
    }
    int vmin = vals[0], vmax = vals[0];
    for (int k = 1; k < K; k++) {
        if (vals[k] < vmin) vmin = vals[k];
        if (vals[k] > vmax) vmax = vals[k];
    }
    /* When the sampled strip has (near-)no local contrast (e.g. a purely
     * solid black or white run), the min/max midpoint degenerates to the
     * sample value itself and a strict "<" test would classify every
     * sample as light. Fall back to a fixed mid-gray split in that case. */
    int thresh = (vmax - vmin >= 8) ? (vmin + vmax) / 2 : 128;

    int b[64];
    int dark = 0;
    for (int k = 0; k < K; k++) {
        b[k] = (vals[k] < thresh) ? 1 : 0;
        dark += b[k];
    }
    int flips = 0;
    for (int k = 1; k < K; k++) {
        if (b[k] != b[k - 1]) flips++;
    }

    if (flips <= 2 && dark * 8 >= K * 7) {
        seg->type = DM_EDGE_SOLID;
        return;
    }

    /* Fallback for chains that overran the symbol corner into unrelated
     * content (e.g. caption text below the symbol): the global 7/8-dark
     * test fails, but if the *longest consecutive dark run* still
     * dominates the sample, treat it as solid and trim the segment
     * endpoints down to that run so the contaminated tail is dropped. */
    {
        int best_s = -1, best_e = -1, cur_s = -1;
        for (int k = 0; k <= K; k++) {
            if (k < K && b[k]) {
                if (cur_s < 0) cur_s = k;
            } else if (cur_s >= 0) {
                if (best_s < 0 || (k - cur_s) > (best_e - best_s)) {
                    best_s = cur_s; best_e = k;
                }
                cur_s = -1;
            }
        }
        if (best_s >= 0 && (best_e - best_s) * 10 >= K * 6 && best_e - best_s >= 6) {
            double t0 = (K > 1) ? (double)best_s / (double)(K - 1) : 0.0;
            double t1 = (K > 1) ? (double)(best_e - 1) / (double)(K - 1) : 0.0;
            double nx0 = seg->px0, ny0 = seg->py0, nx1 = seg->px1, ny1 = seg->py1;
            seg->px1 = nx0 + (nx1 - nx0) * t1; seg->py1 = ny0 + (ny1 - ny0) * t1;
            seg->px0 = nx0 + (nx1 - nx0) * t0; seg->py0 = ny0 + (ny1 - ny0) * t0;
            double ddx = seg->px1 - seg->px0, ddy = seg->py1 - seg->py0;
            seg->len = sqrt(ddx * ddx + ddy * ddy);
            if (seg->len > 1e-6) { seg->dx = ddx / seg->len; seg->dy = ddy / seg->len; }
            seg->type = DM_EDGE_SOLID;
            return;
        }
    }

    if (flips >= K / 3 && flips > 0) {
        /* run-length uniformity check */
        int run = 1, max_run = 0, min_run = K;
        int nruns = 0;
        for (int k = 1; k <= K; k++) {
            if (k < K && b[k] == b[k - 1]) {
                run++;
            } else {
                if (run > max_run) max_run = run;
                if (run < min_run) min_run = run;
                nruns++;
                run = 1;
            }
        }
        if (nruns >= 3 && max_run <= 2 * min_run) {
            seg->type = DM_EDGE_CLOCK;
            seg->module_size = 2.0 * seg->len / (double)flips;
            return;
        }
    }
    seg->type = DM_EDGE_UNKNOWN;
}

/* ── Section 6 (design extrapolation): L-shape finder assembly ─────────── */

typedef struct {
    double c0x, c0y; /* L corner (intersection of the two solid edges) */
    double c1x, c1y; /* far end of edge A */
    double c2x, c2y; /* far end of edge B */
    double c3x, c3y; /* opposite corner (parallelogram completion) */
    double score;
} DmFinderCandidate;

/* Intersection of two lines given as point + direction. Returns 0 on
 * (near-)parallel failure. */
static int dm_line_intersect(double p0x, double p0y, double dx0, double dy0,
                              double p1x, double p1y, double dx1, double dy1,
                              double *ix, double *iy)
{
    double denom = dx0 * dy1 - dy0 * dx1;
    if (fabs(denom) < 1e-6) return 0;
    double ex = p1x - p0x, ey = p1y - p0y;
    double t = (ex * dy1 - ey * dx1) / denom;
    *ix = p0x + dx0 * t;
    *iy = p0y + dy0 * t;
    return 1;
}

static double dm_dist(double ax, double ay, double bx, double by)
{
    double dx = ax - bx, dy = ay - by;
    return sqrt(dx * dx + dy * dy);
}

static int dm_assemble_L(const DmSeg *segsA, int nA, const DmSeg *segsB, int nB,
                          int tile, DmFinderCandidate *out, int max_out)
{
    int nout = 0;
    for (int ia = 0; ia < nA; ia++) {
        const DmSeg *A = &segsA[ia];
        if (A->type != DM_EDGE_SOLID) continue;
        for (int ib = 0; ib < nB; ib++) {
            const DmSeg *B = &segsB[ib];
            if (B->type != DM_EDGE_SOLID) continue;

            double dot = fabs(A->dx * B->dx + A->dy * B->dy);
            if (dot > 0.26) continue; /* not near-orthogonal */

            /* proximity: some endpoint pair within 2*tile */
            double d00 = dm_dist(A->px0, A->py0, B->px0, B->py0);
            double d01 = dm_dist(A->px0, A->py0, B->px1, B->py1);
            double d10 = dm_dist(A->px1, A->py1, B->px0, B->py0);
            double d11 = dm_dist(A->px1, A->py1, B->px1, B->py1);
            double dmin = d00;
            int a_near_p1 = 0, b_near_p1 = 0;
            if (d01 < dmin) { dmin = d01; a_near_p1 = 0; b_near_p1 = 1; }
            if (d10 < dmin) { dmin = d10; a_near_p1 = 1; b_near_p1 = 0; }
            if (d11 < dmin) { dmin = d11; a_near_p1 = 1; b_near_p1 = 1; }
            if (dmin > 2.0 * tile) continue;

            double c0x, c0y;
            if (!dm_line_intersect(A->px0, A->py0, A->dx, A->dy,
                                    B->px0, B->py0, B->dx, B->dy, &c0x, &c0y))
                continue;

            double c1x = a_near_p1 ? A->px0 : A->px1;
            double c1y = a_near_p1 ? A->py0 : A->py1;
            double c2x = b_near_p1 ? B->px0 : B->px1;
            double c2y = b_near_p1 ? B->py0 : B->py1;
            double c3x = c1x + c2x - c0x;
            double c3y = c1y + c2y - c0y;

            /* Validity check: the dark side of each solid edge (given by
             * its scan axis + gradient polarity) must face the quad
             * interior. This rejects pairings that picked the inner edge
             * of a solid bar (dark on the outside) instead of the true
             * outer symbol boundary. */
            {
                double cxm = (c0x + c1x + c2x + c3x) / 4.0;
                double cym = (c0y + c1y + c2y + c3y) / 4.0;
                double anx = (A->axis == 0) ? ((A->polarity > 0) ? 1.0 : -1.0) : 0.0;
                double any = (A->axis == 1) ? ((A->polarity > 0) ? 1.0 : -1.0) : 0.0;
                double bnx = (B->axis == 0) ? ((B->polarity > 0) ? 1.0 : -1.0) : 0.0;
                double bny = (B->axis == 1) ? ((B->polarity > 0) ? 1.0 : -1.0) : 0.0;
                double amx = (A->px0 + A->px1) / 2.0, amy = (A->py0 + A->py1) / 2.0;
                double bmx = (B->px0 + B->px1) / 2.0, bmy = (B->py0 + B->py1) / 2.0;
                if (anx * (cxm - amx) + any * (cym - amy) < 0) continue;
                if (bnx * (cxm - bmx) + bny * (cym - bmy) < 0) continue;
            }

            if (nout < max_out) {
                out[nout].c0x = c0x; out[nout].c0y = c0y;
                out[nout].c1x = c1x; out[nout].c1y = c1y;
                out[nout].c2x = c2x; out[nout].c2y = c2y;
                out[nout].c3x = c3x; out[nout].c3y = c3y;
                out[nout].score = A->len + B->len;
                nout++;
            }
        }
    }
    return nout;
}

/* De-duplicate candidates whose L-corner is within one tile of each
 * other, keeping the highest-scoring one. */
static int dm_dedupe_candidates(DmFinderCandidate *cand, int n, int tile)
{
    int kept = 0;
    for (int i = 0; i < n; i++) {
        int dup = -1;
        for (int j = 0; j < kept; j++) {
            if (dm_dist(cand[i].c0x, cand[i].c0y, cand[j].c0x, cand[j].c0y) <= (double)tile) {
                dup = j;
                break;
            }
        }
        if (dup < 0) {
            cand[kept++] = cand[i];
        } else if (cand[i].score > cand[dup].score) {
            cand[dup] = cand[i];
        }
    }
    return kept;
}

/* ── Scan drivers ─────────────────────────────────────────────────────── */

#define DM_MAX_LINES 128

static int dm_scan_h(const Image *img, const DmCtx *ctx, int32_t *rowbuf,
                      int32_t *gradbuf, DmScanLine *lines, int max_lines)
{
    int W = ctx->W, H = ctx->H;
    int tile = ctx->tile;
    int y0 = dm_clampi(ctx->cy - H / 4, 0, H - 1);
    int y1 = dm_clampi(ctx->cy + H / 4, 0, H - 1);
    int nlines = 0;
    for (int y = y0; y <= y1 && nlines < max_lines; y += tile) {
        for (int x = 0; x < W; x++) rowbuf[x] = dm_pixel(img, x, y);
        dm_grad5(rowbuf, W, gradbuf);
        DmScanLine *L = &lines[nlines];
        L->fixed = y;
        dm_find_peaks(gradbuf, W - 4, ctx->edge_T, 0,
                      L->xs_pos, &L->n_pos, L->xs_neg, &L->n_neg,
                      DM_MAX_EDGES_PER_LINE);
        nlines++;
    }
    return nlines;
}

static int dm_scan_v(const Image *img, const DmCtx *ctx, int32_t *colbuf,
                      int32_t *gradbuf, DmScanLine *lines, int max_lines)
{
    int W = ctx->W, H = ctx->H;
    int tile = ctx->tile;
    int x0 = dm_clampi(ctx->cx - W / 4, 0, W - 1);
    int x1 = dm_clampi(ctx->cx + W / 4, 0, W - 1);
    int nlines = 0;
    for (int x = x0; x <= x1 && nlines < max_lines; x += tile) {
        for (int y = 0; y < H; y++) colbuf[y] = dm_pixel(img, x, y);
        dm_grad5(colbuf, H, gradbuf);
        DmScanLine *L = &lines[nlines];
        L->fixed = x;
        dm_find_peaks(gradbuf, H - 4, ctx->edge_T, 0,
                      L->xs_pos, &L->n_pos, L->xs_neg, &L->n_neg,
                      DM_MAX_EDGES_PER_LINE);
        nlines++;
    }
    return nlines;
}

/* ── Top-level finder search ─────────────────────────────────────────────
 * Runs threshold + scan + link + classify + assemble, and returns the
 * de-duplicated list of L-shape finder candidates.
 */
#define DM_MAX_CAND 16

static int dm_find_finders(const Image *img, DmFinderCandidate *out_cand, int max_cand)
{
    int W = img->width, H = img->height;
    if (W < 16 || H < 16) return 0;

    int maxdim = (W > H) ? W : H;
    int32_t *rowbuf = malloc(sizeof(int32_t) * (size_t)maxdim);
    int32_t *gradbuf = malloc(sizeof(int32_t) * (size_t)maxdim);
    DmScanLine *hlines = malloc(sizeof(DmScanLine) * DM_MAX_LINES);
    DmScanLine *vlines = malloc(sizeof(DmScanLine) * DM_MAX_LINES);
    DmSeg *segsH = malloc(sizeof(DmSeg) * DM_MAX_SEGS);
    DmSeg *segsV = malloc(sizeof(DmSeg) * DM_MAX_SEGS);
    int ncand = 0;

    if (!rowbuf || !gradbuf || !hlines || !vlines || !segsH || !segsV) {
        goto cleanup;
    }

    DmCtx ctx;
    dm_init(img, &ctx, rowbuf);

    int nh = dm_scan_h(img, &ctx, rowbuf, gradbuf, hlines, DM_MAX_LINES);
    int nv = dm_scan_v(img, &ctx, rowbuf, gradbuf, vlines, DM_MAX_LINES);

    int nSegH = 0, nSegV = 0;
    nSegH += dm_link_edges(hlines, nh, +1, 0, ctx.tile_half, segsH + nSegH, DM_MAX_SEGS - nSegH);
    nSegH += dm_link_edges(hlines, nh, -1, 0, ctx.tile_half, segsH + nSegH, DM_MAX_SEGS - nSegH);
    nSegV += dm_link_edges(vlines, nv, +1, 1, ctx.tile_half, segsV + nSegV, DM_MAX_SEGS - nSegV);
    nSegV += dm_link_edges(vlines, nv, -1, 1, ctx.tile_half, segsV + nSegV, DM_MAX_SEGS - nSegV);

    for (int i = 0; i < nSegH; i++) dm_classify_edge(img, &segsH[i], ctx.tile);
    for (int i = 0; i < nSegV; i++) dm_classify_edge(img, &segsV[i], ctx.tile);

#if DM_DEBUG_PRINT
    for (int i = 0; i < nSegH; i++) {
        DM_DBG("  segH[%d] n=%d type=%d pol=%d (%.0f,%.0f)-(%.0f,%.0f) len=%.1f\n",
               i, segsH[i].n, segsH[i].type, segsH[i].polarity, segsH[i].px0, segsH[i].py0,
               segsH[i].px1, segsH[i].py1, segsH[i].len);
    }
    for (int i = 0; i < nSegV; i++) {
        DM_DBG("  segV[%d] n=%d type=%d pol=%d (%.0f,%.0f)-(%.0f,%.0f) len=%.1f\n",
               i, segsV[i].n, segsV[i].type, segsV[i].polarity, segsV[i].px0, segsV[i].py0,
               segsV[i].px1, segsV[i].py1, segsV[i].len);
    }
#endif

    DmFinderCandidate raw[DM_MAX_CAND * 4];
    int nraw = dm_assemble_L(segsH, nSegH, segsV, nSegV, ctx.tile,
                              raw, DM_MAX_CAND * 4);
    if (nraw > DM_MAX_CAND * 4) nraw = DM_MAX_CAND * 4;
    nraw = dm_dedupe_candidates(raw, nraw, ctx.tile);

    ncand = (nraw < max_cand) ? nraw : max_cand;
    for (int i = 0; i < ncand; i++) out_cand[i] = raw[i];

#if DM_DEBUG_PRINT
    for (int i = 0; i < ncand; i++) {
        DM_DBG("  cand[%d] C0=(%.0f,%.0f) C1=(%.0f,%.0f) C2=(%.0f,%.0f) C3=(%.0f,%.0f) score=%.1f\n",
               i, out_cand[i].c0x, out_cand[i].c0y, out_cand[i].c1x, out_cand[i].c1y,
               out_cand[i].c2x, out_cand[i].c2y, out_cand[i].c3x, out_cand[i].c3y, out_cand[i].score);
    }
#endif

    DM_DBG("dm: tile=%d edge_T=%d hlines=%d vlines=%d segH=%d segV=%d cand=%d\n",
           ctx.tile, ctx.edge_T, nh, nv, nSegH, nSegV, ncand);

cleanup:
    free(rowbuf);
    free(gradbuf);
    free(hlines);
    free(vlines);
    free(segsH);
    free(segsV);
    return ncand;
}

/* ── Section 7: symbol size tables ───────────────────────────────────────
 * Faithful transcription of dword_19E260 (blocks) / dword_19E2C0 (data cw)
 * / dword_19E320 (ec cw), indexed in the same order as
 * SmartSymbolRedef000138's (mapping_dim-8) case ladder. data_region and
 * regs are derived (and cross-checked) from size = regs*(data_region+2).
 */
typedef struct {
    int size;             /* total symbol side length in modules */
    int regs;              /* regions per side (1/2/4/6) */
    int data_region;       /* per-region data-area side length in modules */
    int total_cw, data_cw, ec_cw, blocks;
} DmSquareSpec;

static const DmSquareSpec k_dm_squares[24] = {
    { 10, 1,  8,    8,    3,    5,  1 },
    { 12, 1, 10,   12,    5,    7,  1 },
    { 14, 1, 12,   18,    8,   10,  1 },
    { 16, 1, 14,   24,   12,   12,  1 },
    { 18, 1, 16,   32,   18,   14,  1 },
    { 20, 1, 18,   40,   22,   18,  1 },
    { 22, 1, 20,   50,   30,   20,  1 },
    { 24, 1, 22,   60,   36,   24,  1 },
    { 26, 1, 24,   72,   44,   28,  1 },
    { 32, 2, 14,   98,   62,   36,  1 },
    { 36, 2, 16,  128,   86,   42,  1 },
    { 40, 2, 18,  162,  114,   48,  1 },
    { 44, 2, 20,  200,  144,   56,  1 },
    { 48, 2, 22,  242,  174,   68,  1 },
    { 52, 2, 24,  288,  204,   84,  2 },
    { 64, 4, 14,  392,  280,  112,  2 },
    { 72, 4, 16,  512,  368,  144,  4 },
    { 80, 4, 18,  648,  456,  192,  4 },
    { 88, 4, 20,  800,  576,  224,  4 },
    { 96, 4, 22,  968,  696,  272,  4 },
    { 104, 4, 24, 1152,  816,  336,  6 },
    { 120, 6, 18, 1458, 1050,  408,  6 },
    { 132, 6, 20, 1800, 1304,  496,  8 },
    { 144, 6, 22, 2178, 1558,  620, 10 },
};

/* Rectangle table: dim_r < dim_c by convention (matches the reference's
 * mapping (rows,cols) index switch at 238837-238868). Codeword counts are
 * the standard ISO/IEC 16022 rectangle values (unk_1A12C4 body is a weak
 * symbol with no data in the decompiled dump). */
typedef struct {
    int dim_r, dim_c;       /* total symbol size (rows x cols), rows<cols */
    int regs_r, regs_c;     /* regions along rows / cols */
    int data_region_r, data_region_c;
    int total_cw, data_cw, ec_cw, blocks;
} DmRectSpec;

static const DmRectSpec k_dm_rects[6] = {
    {  8, 18, 1, 1,  6, 16,  12,  5,  7, 1 },
    {  8, 32, 1, 2,  6, 14,  21, 10, 11, 1 },
    { 12, 26, 1, 1, 10, 24,  30, 16, 14, 1 },
    { 12, 36, 1, 2, 10, 16,  40, 22, 18, 1 },
    { 16, 36, 1, 2, 14, 16,  56, 32, 24, 1 },
    { 16, 48, 1, 2, 14, 22,  77, 49, 28, 1 },
};

typedef struct {
    int is_square;
    int size_rows, size_cols;       /* total symbol size, incl. border */
    int regs_v, regs_h;             /* regions along rows / cols */
    int data_region_r, data_region_c;
    int mapping_rows, mapping_cols; /* = regs*data_region */
    int total_cw, data_cw, ec_cw, blocks;
} DmSizeInfo;

/* Nearest-match lookup: n_rows/n_cols are module counts measured along the
 * two clock edges (see dm_count_modules_along). Accepts a small tolerance
 * to absorb counting noise; picks the globally closest table entry. */
static int dm_size_lookup(int n_rows, int n_cols, DmSizeInfo *out)
{
    int best_err = INT_MAX;
    int best_is_square = 0, best_idx = 0, best_swapped = 0;

    for (int i = 0; i < 24; i++) {
        int err = abs(k_dm_squares[i].size - n_rows) + abs(k_dm_squares[i].size - n_cols);
        if (err < best_err) {
            best_err = err;
            best_is_square = 1;
            best_idx = i;
            best_swapped = 0;
        }
    }
    for (int i = 0; i < 6; i++) {
        int err_direct = abs(k_dm_rects[i].dim_r - n_rows) + abs(k_dm_rects[i].dim_c - n_cols);
        if (err_direct < best_err) {
            best_err = err_direct;
            best_is_square = 0;
            best_idx = i;
            best_swapped = 0;
        }
        int err_swap = abs(k_dm_rects[i].dim_r - n_cols) + abs(k_dm_rects[i].dim_c - n_rows);
        if (err_swap < best_err) {
            best_err = err_swap;
            best_is_square = 0;
            best_idx = i;
            best_swapped = 1;
        }
    }

    if (best_err > 3) return 0; /* no plausible size within tolerance */

    memset(out, 0, sizeof(*out));
    if (best_is_square) {
        const DmSquareSpec *s = &k_dm_squares[best_idx];
        out->is_square = 1;
        out->size_rows = out->size_cols = s->size;
        out->regs_v = out->regs_h = s->regs;
        out->data_region_r = out->data_region_c = s->data_region;
        out->total_cw = s->total_cw; out->data_cw = s->data_cw;
        out->ec_cw = s->ec_cw; out->blocks = s->blocks;
    } else {
        const DmRectSpec *r = &k_dm_rects[best_idx];
        out->is_square = 0;
        if (!best_swapped) {
            out->size_rows = r->dim_r; out->size_cols = r->dim_c;
            out->regs_v = r->regs_r; out->regs_h = r->regs_c;
            out->data_region_r = r->data_region_r; out->data_region_c = r->data_region_c;
        } else {
            out->size_rows = r->dim_c; out->size_cols = r->dim_r;
            out->regs_v = r->regs_c; out->regs_h = r->regs_r;
            out->data_region_r = r->data_region_c; out->data_region_c = r->data_region_r;
        }
        out->total_cw = r->total_cw; out->data_cw = r->data_cw;
        out->ec_cw = r->ec_cw; out->blocks = r->blocks;
    }
    out->mapping_rows = out->regs_v * out->data_region_r;
    out->mapping_cols = out->regs_h * out->data_region_c;
    return 1;
}

/* ── Section 8 (design extrapolation): clock-edge module counting ────────
 * Samples a straight line between two corners of a clock (dashed) edge
 * and counts black/white transitions to estimate the module count along
 * that axis. This stands in for the reference's cost-table period search
 * (sub_5829C / unk_2069A8, data not present in the decompiled dump) with
 * the same underlying idea used by the jump-walking path (sub_148F04):
 * count transitions along the alternating clock track.
 */
#define DM_COUNT_MAX_SAMPLES 600

static int dm_count_modules_along(const Image *img, double ax, double ay,
                                   double bx, double by)
{
    double len = dm_dist(ax, ay, bx, by);
    if (len < 2.0) return 0;

    int ns = (int)lround(len);
    if (ns < 4) ns = 4;
    if (ns > DM_COUNT_MAX_SAMPLES) ns = DM_COUNT_MAX_SAMPLES;

    int vals[DM_COUNT_MAX_SAMPLES + 1];
    for (int i = 0; i <= ns; i++) {
        double t = (double)i / (double)ns;
        double x = ax + (bx - ax) * t;
        double y = ay + (by - ay) * t;
        vals[i] = dm_pixel(img, (int)lround(x), (int)lround(y));
    }
    int vmin = vals[0], vmax = vals[0];
    for (int i = 1; i <= ns; i++) {
        if (vals[i] < vmin) vmin = vals[i];
        if (vals[i] > vmax) vmax = vals[i];
    }
    int thresh = (vmax - vmin >= 8) ? (vmin + vmax) / 2 : 128;

    int transitions = 0;
    int prev = (vals[0] < thresh) ? 1 : 0;
    for (int i = 1; i <= ns; i++) {
        int cur = (vals[i] < thresh) ? 1 : 0;
        if (cur != prev) transitions++;
        prev = cur;
    }
    return transitions + 1;
}

/* ── Section 9: local 4-point homography (own copy of the general DLT
 * solve used by qrcode.c's Perspective/compute_perspective; duplicated
 * here rather than shared since datamatrix.c may not depend on qrcode.c
 * internals). Maps unit-square module space (u,v) in [0,size_cols] x
 * [0,size_rows] to image (x,y).
 */
typedef struct {
    double a[9];
} DmPerspective;

static int dm_solve_linear_8x8(double m[8][9], double out[8])
{
    for (int col = 0; col < 8; col++) {
        int pivot = col;
        double max_abs = fabs(m[col][col]);
        for (int row = col + 1; row < 8; row++) {
            double v = fabs(m[row][col]);
            if (v > max_abs) { max_abs = v; pivot = row; }
        }
        if (max_abs < 1e-12) return 0;
        if (pivot != col) {
            for (int k = col; k <= 8; k++) {
                double tmp = m[col][k]; m[col][k] = m[pivot][k]; m[pivot][k] = tmp;
            }
        }
        double inv = 1.0 / m[col][col];
        for (int k = col; k <= 8; k++) m[col][k] *= inv;
        for (int row = 0; row < 8; row++) {
            if (row == col) continue;
            double factor = m[row][col];
            if (fabs(factor) < 1e-15) continue;
            for (int k = col; k <= 8; k++) m[row][k] -= factor * m[col][k];
        }
    }
    for (int i = 0; i < 8; i++) out[i] = m[i][8];
    return 1;
}

static DmPerspective dm_compute_perspective(
    double su0, double sv0, double du0, double dv0,
    double su1, double sv1, double du1, double dv1,
    double su2, double sv2, double du2, double dv2,
    double su3, double sv3, double du3, double dv3)
{
    DmPerspective t;
    double eq[8][9];
    const double su[4] = { su0, su1, su2, su3 };
    const double sv[4] = { sv0, sv1, sv2, sv3 };
    const double du[4] = { du0, du1, du2, du3 };
    const double dv[4] = { dv0, dv1, dv2, dv3 };
    double sol[8];

    memset(&t, 0, sizeof(t));
    memset(eq, 0, sizeof(eq));
    for (int i = 0; i < 4; i++) {
        int row = i * 2;
        eq[row][0] = su[i]; eq[row][1] = sv[i]; eq[row][2] = 1.0;
        eq[row][6] = -su[i] * du[i]; eq[row][7] = -sv[i] * du[i]; eq[row][8] = du[i];

        eq[row + 1][3] = su[i]; eq[row + 1][4] = sv[i]; eq[row + 1][5] = 1.0;
        eq[row + 1][6] = -su[i] * dv[i]; eq[row + 1][7] = -sv[i] * dv[i]; eq[row + 1][8] = dv[i];
    }
    if (dm_solve_linear_8x8(eq, sol)) {
        for (int i = 0; i < 8; i++) t.a[i] = sol[i];
        t.a[8] = 1.0;
        return t;
    }
    t.a[0] = du1 - du0; t.a[1] = du3 - du0; t.a[2] = du0;
    t.a[3] = dv1 - dv0; t.a[4] = dv3 - dv0; t.a[5] = dv0;
    t.a[6] = 0.0; t.a[7] = 0.0; t.a[8] = 1.0;
    return t;
}

static void dm_perspective_map(const DmPerspective *t, double u, double v,
                                double *x, double *y)
{
    double w = t->a[6] * u + t->a[7] * v + t->a[8];
    if (fabs(w) < 1e-10) w = 1e-10;
    *x = (t->a[0] * u + t->a[1] * v + t->a[2]) / w;
    *y = (t->a[3] * u + t->a[4] * v + t->a[5]) / w;
}

static double dm_bilinear_sample(const Image *img, double x, double y)
{
    int W = img->width, H = img->height;
    if (x < 0.0) x = 0.0;
    if (y < 0.0) y = 0.0;
    if (x > (double)(W - 1)) x = (double)(W - 1);
    if (y > (double)(H - 1)) y = (double)(H - 1);
    int x0 = (int)x, y0 = (int)y;
    int x1 = (x0 + 1 < W) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < H) ? y0 + 1 : y0;
    double fx = x - x0, fy = y - y0;
    double v00 = img->pixels[(size_t)y0 * W + x0];
    double v10 = img->pixels[(size_t)y0 * W + x1];
    double v01 = img->pixels[(size_t)y1 * W + x0];
    double v11 = img->pixels[(size_t)y1 * W + x1];
    double v0 = v00 + (v10 - v00) * fx;
    double v1 = v01 + (v11 - v01) * fx;
    return v0 + (v1 - v0) * fy;
}

/* ── Section 10: perspective grid sampling ────────────────────────────────
 * Module (row,col) center is sampled in half-module coordinates, matching
 * the reference's half-module symbol coordinate convention (00010e /
 * 601496): center_u = col+0.5, center_v = row+0.5 out of [0,size_cols] x
 * [0,size_rows]. row 0 / col 0 sit on the solid (L) border, row/col
 * (size-1) sit on the clock (dashed) border -- standard DataMatrix finder
 * layout. Two-pass grayscale sampling + local-neighborhood threshold,
 * same shape as qrcode.c's sample_grid_grayscale. */
static int dm_sample_grid(const Image *img, const DmPerspective *t,
                           int rows, int cols, uint8_t *grid /* rows*cols */)
{
    int total = rows * cols;
    double *gray = (double *)malloc(sizeof(double) * (size_t)total);
    if (!gray) return 0;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            double u = (double)c + 0.5;
            double v = (double)r + 0.5;
            double x, y;
            dm_perspective_map(t, u, v, &x, &y);
            gray[r * cols + c] = dm_bilinear_sample(img, x, y);
        }
    }
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            double sum = 0.0;
            int cnt = 0;
            for (int dr = -2; dr <= 2; dr++) {
                for (int dc = -2; dc <= 2; dc++) {
                    int nr = r + dr, nc = c + dc;
                    if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                    sum += gray[nr * cols + nc];
                    cnt++;
                }
            }
            double thresh = (cnt > 0) ? (sum / cnt) : 128.0;
            grid[r * cols + c] = (gray[r * cols + c] < thresh) ? 1 : 0;
        }
    }
    free(gray);
    return 1;
}

/* ── Section 11: tie it together for one L-shape candidate ───────────────
 * Uses the candidate's L corner (c0) and the two far corners of its solid
 * edges (c1 along axis A, c2 along axis B; c3 = opposite corner) to: count
 * modules along the two clock edges (c2-c3 parallel to A, c1-c3 parallel
 * to B), look up the symbol size, build the unit-square-to-quad
 * homography and sample the grid, then strip the finder border into a
 * data-region mapping matrix (design layer, structure per the reference's
 * 238694 region loop: each region is (data_region_r+2) x (data_region_c+2)
 * modules with a 1-module solid border at row/col 0 and a 1-module clock
 * border at the far row/col; the interior data_region_r x data_region_c
 * block of every region is concatenated in region-major order).
 *
 * mirror handling (M6, design extrapolation -- the reference's DM core has
 * no internal mirror retry; mirror support lives at the API layer via
 * UndoMirroredImage @ 29215/18477 + mirrorY8bitbmp @ 29814, which flips the
 * whole image before decoding starts. Since this harness has no such flag,
 * datamatrix_decode() below retries every candidate with both chiralities
 * internally.) A mirrored symbol is a diagonal transpose of a normal one
 * about c0, which is exactly "axis A and axis B swap roles": c1 (axis A
 * far corner) and c2 (axis B far corner) trade places while c3 = c1+c2-c0
 * is unchanged. Swapping them once, up front, makes every line below
 * (module counts, size lookup, homography corners) automatically consistent
 * for the mirrored orientation -- no other logic needs to change. */

/* Corner refinement (design extrapolation for the reference's pixel-level
 * boundary walk in sub_148F04, called from the size-decision path at
 * 238329/238380): the L candidate's c1/c2 endpoints come from a
 * tile-quantized scan grid (dm_link_edges) and can undershoot the true
 * symbol corner by up to one tile. Starting from the tile-grid endpoint,
 * step outward 1px at a time along each solid edge's direction, probing
 * two depths into the dark side (2.5px and 5.0px -- a single shallow probe
 * can land exactly on a boundary pixel and stop early on sub-pixel-shifted
 * corners); keep extending while either probe is still dark, tolerating up
 * to a 3px gap, capped at 64px. c3 is recomputed from the refined c1/c2. */
static void dm_refine_corners(const Image *img, DmFinderCandidate *cand)
{
    double ulen = dm_dist(cand->c0x, cand->c0y, cand->c1x, cand->c1y);
    double vlen = dm_dist(cand->c0x, cand->c0y, cand->c2x, cand->c2y);
    if (ulen < 1e-6 || vlen < 1e-6) return;
    double ux = (cand->c1x - cand->c0x) / ulen, uy = (cand->c1y - cand->c0y) / ulen;
    double vx = (cand->c2x - cand->c0x) / vlen, vy = (cand->c2y - cand->c0y) / vlen;

    /* extend c1 along +u, probing slightly inward (toward +v) */
    double best = ulen;
    for (double t = ulen; t < ulen + 64.0; t += 1.0) {
        double px = cand->c0x + ux * t + vx * 2.5;
        double py = cand->c0y + uy * t + vy * 2.5;
        double qx = cand->c0x + ux * t + vx * 5.0;
        double qy = cand->c0y + uy * t + vy * 5.0;
        if (dm_pixel(img, (int)lround(px), (int)lround(py)) < 128 ||
            dm_pixel(img, (int)lround(qx), (int)lround(qy)) < 128) best = t;
        else if (t > best + 3.0) break;
    }
    cand->c1x = cand->c0x + ux * best; cand->c1y = cand->c0y + uy * best;

    /* extend c2 along +v, probing slightly inward (toward +u) */
    best = vlen;
    for (double t = vlen; t < vlen + 64.0; t += 1.0) {
        double px = cand->c0x + vx * t + ux * 2.5;
        double py = cand->c0y + vy * t + uy * 2.5;
        double qx = cand->c0x + vx * t + ux * 5.0;
        double qy = cand->c0y + vy * t + uy * 5.0;
        if (dm_pixel(img, (int)lround(px), (int)lround(py)) < 128 ||
            dm_pixel(img, (int)lround(qx), (int)lround(qy)) < 128) best = t;
        else if (t > best + 3.0) break;
    }
    cand->c2x = cand->c0x + vx * best; cand->c2y = cand->c0y + vy * best;
    cand->c3x = cand->c1x + cand->c2x - cand->c0x;
    cand->c3y = cand->c1y + cand->c2y - cand->c0y;
}

static int dm_analyze_candidate(const Image *img, const DmFinderCandidate *cand_in,
                                 int mirror, DmSizeInfo *out_info, uint8_t **out_mapping)
{
    DmFinderCandidate cand_refined = *cand_in;
    dm_refine_corners(img, &cand_refined);
    cand_in = &cand_refined;
    DmFinderCandidate cand_mirrored;
    if (mirror) {
        cand_mirrored = *cand_in;
        cand_mirrored.c1x = cand_in->c2x; cand_mirrored.c1y = cand_in->c2y;
        cand_mirrored.c2x = cand_in->c1x; cand_mirrored.c2y = cand_in->c1y;
        cand_in = &cand_mirrored;
    }
    const DmFinderCandidate *cand = cand_in;

    /* The clock (right/top) edges run exactly along the outer boundary of
     * the symbol; sampling precisely on that boundary line is prone to
     * rounding half the samples into the background on either side. Inset
     * the two measurement lines inward by a small fixed amount (axis A's
     * unit vector for the right edge, axis B's for the top edge) so the
     * sampled points land reliably inside the border module band. */
    double ulen = dm_dist(cand->c0x, cand->c0y, cand->c1x, cand->c1y);
    double vlen = dm_dist(cand->c0x, cand->c0y, cand->c2x, cand->c2y);
    double ux = (ulen > 1e-6) ? (cand->c1x - cand->c0x) / ulen : 0.0;
    double uy = (ulen > 1e-6) ? (cand->c1y - cand->c0y) / ulen : 0.0;
    double vx = (vlen > 1e-6) ? (cand->c2x - cand->c0x) / vlen : 0.0;
    double vy = (vlen > 1e-6) ? (cand->c2y - cand->c0y) / vlen : 0.0;
    const double inset = 1.5;

    int n_rows = dm_count_modules_along(img,
        cand->c2x - vx * inset, cand->c2y - vy * inset,
        cand->c3x - vx * inset, cand->c3y - vy * inset);
    int n_cols = dm_count_modules_along(img,
        cand->c1x - ux * inset, cand->c1y - uy * inset,
        cand->c3x - ux * inset, cand->c3y - uy * inset);
    if (n_rows < 8 || n_cols < 8) return 0;

    DmSizeInfo info;
    if (!dm_size_lookup(n_rows, n_cols, &info)) return 0;

    /* unit square (u,v) in [0,size_cols]x[0,size_rows] -> image quad:
     * (0,0)=c0, (cols,0)=c2 (B axis), (0,rows)=c1 (A axis), (cols,rows)=c3 */
    DmPerspective t = dm_compute_perspective(
        0.0, 0.0, cand->c0x, cand->c0y,
        (double)info.size_cols, 0.0, cand->c2x, cand->c2y,
        0.0, (double)info.size_rows, cand->c1x, cand->c1y,
        (double)info.size_cols, (double)info.size_rows, cand->c3x, cand->c3y);

    uint8_t *grid = (uint8_t *)malloc((size_t)info.size_rows * (size_t)info.size_cols);
    if (!grid) return 0;
    if (!dm_sample_grid(img, &t, info.size_rows, info.size_cols, grid)) {
        free(grid);
        return 0;
    }

    uint8_t *mapping = (uint8_t *)malloc((size_t)info.mapping_rows * (size_t)info.mapping_cols);
    if (!mapping) {
        free(grid);
        return 0;
    }
    int rh = info.data_region_r + 2;
    int rw = info.data_region_c + 2;
    for (int ry = 0; ry < info.regs_v; ry++) {
        for (int rx = 0; rx < info.regs_h; rx++) {
            int base_r = ry * rh;
            int base_c = rx * rw;
            for (int lr = 0; lr < info.data_region_r; lr++) {
                for (int lc = 0; lc < info.data_region_c; lc++) {
                    int gr = base_r + 1 + lr;
                    int gc = base_c + 1 + lc;
                    int mr = ry * info.data_region_r + lr;
                    int mc = rx * info.data_region_c + lc;
                    mapping[mr * info.mapping_cols + mc] = grid[gr * info.size_cols + gc];
                }
            }
        }
    }
    free(grid);

    DM_DBG("dm: candidate mirror=%d size=%dx%d (measured %dx%d) mapping=%dx%d cw=%d/%d/%d blocks=%d\n",
           mirror, info.size_rows, info.size_cols, n_rows, n_cols,
           info.mapping_rows, info.mapping_cols, info.total_cw, info.data_cw, info.ec_cw, info.blocks);

    *out_info = info;
    *out_mapping = mapping;
    return 1;
}

/* ── Section 12: ECC200 module bit -> codeword placement extraction ──────
 * Faithful transcription of SmartSymbolRedef00014c (59072-59874) and its
 * utah-shape helper sub_5AFF0 (59876-60084), i.e. the ISO/IEC 16022 Annex F
 * bit placement algorithm, applied to the border-stripped data-region
 * matrix built in Section 11.
 *
 * Coordinate convention used here (standard Annex F): row 0 = top/clock
 * edge, row nrow-1 = bottom/solid edge; col 0 = left/solid edge,
 * col ncol-1 = right/clock edge. The Section-11 mapping matrix instead has
 * row 0 nearest the c0-c2 solid edge (row nrow-1 nearest the c1-c3 clock
 * edge) -- the opposite order -- while its column order already matches
 * Annex F (col 0 nearest the c0-c1 solid edge). This mirrors the reference,
 * which stores its module matrix row-flipped relative to the Annex F walk
 * (visited bit uses ROW directly, sampled bit uses nrow-1-ROW, see
 * 59300/59522 in the decompiled source); dm_extract_codewords_from_mapping()
 * below performs the equivalent row flip once, up front, so the placement
 * walk itself can use plain Annex F coordinates.
 */
#define DM_MAX_MAPPING_DIM 132
#define DM_MAX_TOTAL_CW ((DM_MAX_MAPPING_DIM * DM_MAX_MAPPING_DIM) / 8)

typedef struct {
    const uint8_t *bits;    /* nrow*ncol, Annex F coords, 1 = dark module */
    uint8_t *visited;       /* nrow*ncol scratch, 1 = already placed */
    uint8_t *cw;            /* codeword output, cw_cap entries */
    int nrow, ncol;
} DmPlaceCtx;

/* module() wrap rule, verbatim from the reference (e.g. 59286-59296):
 *   row<0:  row += nrow; col += 4 - ((nrow+4) & 7)
 *   col<0:  col += ncol; row += 4 - ((ncol+4) & 7)
 */
static void dm_place_bit(DmPlaceCtx *pc, int row, int col, int idx, uint8_t mask)
{
    if (row < 0) {
        row += pc->nrow;
        col += 4 - ((pc->nrow + 4) & 7);
    }
    if (col < 0) {
        col += pc->ncol;
        row += 4 - ((pc->ncol + 4) & 7);
    }
    if (row < 0 || row >= pc->nrow || col < 0 || col >= pc->ncol) return;
    pc->visited[row * pc->ncol + col] = 1;
    if (pc->bits[row * pc->ncol + col]) pc->cw[idx] |= mask;
}

/* sub_5AFF0: the "utah" 8-module placement shape, bit1(0x80)..bit8(0x01). */
static void dm_place_utah(DmPlaceCtx *pc, int row, int col, int idx)
{
    dm_place_bit(pc, row - 2, col - 2, idx, 0x80);
    dm_place_bit(pc, row - 2, col - 1, idx, 0x40);
    dm_place_bit(pc, row - 1, col - 2, idx, 0x20);
    dm_place_bit(pc, row - 1, col - 1, idx, 0x10);
    dm_place_bit(pc, row - 1, col,     idx, 0x08);
    dm_place_bit(pc, row,     col - 2, idx, 0x04);
    dm_place_bit(pc, row,     col - 1, idx, 0x02);
    dm_place_bit(pc, row,     col,     idx, 0x01);
}

static int dm_is_visited(const DmPlaceCtx *pc, int row, int col)
{
    if (row < 0 || row >= pc->nrow || col < 0 || col >= pc->ncol) return 0;
    return pc->visited[row * pc->ncol + col];
}

/* SmartSymbolRedef00014c main walk: 4 corner special-cases (checked with
 * plain `if`, not `else if`, matching the reference -- they are mutually
 * exclusive per (nrow,ncol) so at most one fires per outer iteration) plus
 * the up/down diagonal utah-shape scans. bits/cw/cw_cap are already in
 * Annex F row order. Returns the number of codewords produced (==
 * nrow*ncol/8 for all but the corner1/corner4 sizes, where nrow*ncol%8==4
 * and a residual 2x2 block in a bottom corner is legitimately left
 * unvisited -- see module-count invariant in the design notes). */
static int dm_extract_codewords(const uint8_t *bits, int nrow, int ncol,
                                 uint8_t *cw, int cw_cap)
{
    if (nrow <= 0 || ncol <= 0 || nrow > DM_MAX_MAPPING_DIM || ncol > DM_MAX_MAPPING_DIM)
        return 0;

    int total = nrow * ncol;
    uint8_t *visited = (uint8_t *)calloc((size_t)total, 1);
    if (!visited) return 0;

    int max_cw = total / 8;
    if (max_cw > cw_cap) max_cw = cw_cap;
    memset(cw, 0, (size_t)max_cw);

    DmPlaceCtx pc = { bits, visited, cw, nrow, ncol };
    int idx = 0, row = 4, col = 0;

    do {
        if (row == nrow && col == 0 && idx < max_cw) {                        /* corner1 */
            dm_place_bit(&pc, nrow - 1, 0,       idx, 0x80);
            dm_place_bit(&pc, nrow - 1, 1,       idx, 0x40);
            dm_place_bit(&pc, nrow - 1, 2,       idx, 0x20);
            dm_place_bit(&pc, 0,        ncol - 2, idx, 0x10);
            dm_place_bit(&pc, 0,        ncol - 1, idx, 0x08);
            dm_place_bit(&pc, 1,        ncol - 1, idx, 0x04);
            dm_place_bit(&pc, 2,        ncol - 1, idx, 0x02);
            dm_place_bit(&pc, 3,        ncol - 1, idx, 0x01);
            idx++;
        }
        if (row == nrow - 2 && col == 0 && (ncol % 4) != 0 && idx < max_cw) {  /* corner2 */
            dm_place_bit(&pc, nrow - 3, 0,       idx, 0x80);
            dm_place_bit(&pc, nrow - 2, 0,       idx, 0x40);
            dm_place_bit(&pc, nrow - 1, 0,       idx, 0x20);
            dm_place_bit(&pc, 0,        ncol - 4, idx, 0x10);
            dm_place_bit(&pc, 0,        ncol - 3, idx, 0x08);
            dm_place_bit(&pc, 0,        ncol - 2, idx, 0x04);
            dm_place_bit(&pc, 0,        ncol - 1, idx, 0x02);
            dm_place_bit(&pc, 1,        ncol - 1, idx, 0x01);
            idx++;
        }
        if (row == nrow - 2 && col == 0 && (ncol % 8) == 4 && idx < max_cw) {  /* corner3 */
            dm_place_bit(&pc, nrow - 3, 0,       idx, 0x80);
            dm_place_bit(&pc, nrow - 2, 0,       idx, 0x40);
            dm_place_bit(&pc, nrow - 1, 0,       idx, 0x20);
            dm_place_bit(&pc, 0,        ncol - 2, idx, 0x10);
            dm_place_bit(&pc, 0,        ncol - 1, idx, 0x08);
            dm_place_bit(&pc, 1,        ncol - 1, idx, 0x04);
            dm_place_bit(&pc, 2,        ncol - 1, idx, 0x02);
            dm_place_bit(&pc, 3,        ncol - 1, idx, 0x01);
            idx++;
        }
        if (row == nrow + 4 && col == 2 && (ncol % 8) == 0 && idx < max_cw) {  /* corner4 */
            dm_place_bit(&pc, nrow - 1, 0,       idx, 0x80);
            dm_place_bit(&pc, nrow - 1, ncol - 1, idx, 0x40);
            dm_place_bit(&pc, 0,        ncol - 3, idx, 0x20);
            dm_place_bit(&pc, 0,        ncol - 2, idx, 0x10);
            dm_place_bit(&pc, 0,        ncol - 1, idx, 0x08);
            dm_place_bit(&pc, 1,        ncol - 3, idx, 0x04);
            dm_place_bit(&pc, 1,        ncol - 2, idx, 0x02);
            dm_place_bit(&pc, 1,        ncol - 1, idx, 0x01);
            idx++;
        }

        do {                                             /* upward diagonal scan */
            if (row < nrow && col >= 0 && !dm_is_visited(&pc, row, col) && idx < max_cw)
                dm_place_utah(&pc, row, col, idx++);
            row -= 2; col += 2;
        } while (row >= 0 && col < ncol);
        row += 1; col += 3;

        do {                                             /* downward diagonal scan */
            if (row >= 0 && col < ncol && !dm_is_visited(&pc, row, col) && idx < max_cw)
                dm_place_utah(&pc, row, col, idx++);
            row += 2; col -= 2;
        } while (row < nrow && col >= 0);
        row += 3; col += 1;
    } while ((row < nrow || col < ncol) && idx < max_cw);

    free(visited);
    return idx;
}

/* Row-flips the Section-11 mapping matrix into Annex F coordinates (see
 * the Section 12 header comment) and runs the placement walk. */
static int dm_extract_codewords_from_mapping(const uint8_t *mapping, int mapping_rows,
                                              int mapping_cols, uint8_t *cw, int cw_cap)
{
    if (mapping_rows <= 0 || mapping_cols <= 0 ||
        mapping_rows > DM_MAX_MAPPING_DIM || mapping_cols > DM_MAX_MAPPING_DIM)
        return 0;

    uint8_t *bits = (uint8_t *)malloc((size_t)mapping_rows * (size_t)mapping_cols);
    if (!bits) return 0;
    for (int r = 0; r < mapping_rows; r++) {
        const uint8_t *src = mapping + (size_t)(mapping_rows - 1 - r) * mapping_cols;
        memcpy(bits + (size_t)r * mapping_cols, src, (size_t)mapping_cols);
    }

    int n = dm_extract_codewords(bits, mapping_rows, mapping_cols, cw, cw_cap);
    free(bits);
    return n;
}

/* ── Section 13: GF(256) Reed-Solomon decoder ─────────────────────────────
 * Own local copy of aztec.c's generic GF(256) Berlekamp-Massey + Forney RS
 * decoder (GF(256): x^8+x^4+x^3+x^2+1 = 0x12d, matching the reference's
 * SmartSymbolRedef00014f mode-0 field selection). Not shared via a header
 * per the interface convention (datamatrix.c must only touch its own
 * file). One behavioural change from aztec.c's copy: this version returns
 * the number of corrected symbols (0 if error-free, L>=1 if corrected) on
 * success instead of a bare 0 status, since dm_rs_correct()'s confidence
 * formula (SmartSymbolRedef000100, 263496/263618) needs the correction
 * count, not just a pass/fail bit.
 */
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

typedef struct {
    int *exp_tbl;
    int *log_tbl;
    int  order; /* field_size - 1 */
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

/* Berlekamp-Massey + Forney RS decoder. cw[0] is the highest-order
 * (first-transmitted) codeword. Returns the number of corrected symbols
 * (>=0) on success, -1 on uncorrectable/detected-but-unlocatable error. */
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
        /* sigma'(x) (the sd[] array) has char-2 nonzero coefficients only
         * at even degrees, but its highest possibly-nonzero degree is
         * L-1 (odd L) -- not (L-1)/2. Evaluating only up to (L-1)/2 drops
         * the top term(s) whenever L>=3, corrupting the Forney magnitude
         * (verified against a synthetic 3-error test vector during M4
         * validation). Loop the full degree range instead; the
         * odd-degree zero entries contribute nothing either way. */
        int sv = 0; pw = 1;
        for (int j = 0; j < L; j++) {
            sv ^= gf_mul(gf, sd[j], pw);
            pw = gf_mul(gf, pw, xi);
        }
        if (sv == 0) return -1;
        cw[pos] ^= gf_mul(gf, ov, gf_inv(gf, sv));
    }
    return L;
}

/* ── Section 14: block de-interleave + RS correction ──────────────────────
 * Faithful transcription of SmartSymbolRedef000100 (263447-263623). `cw`
 * is the M3-produced, forward-ordered flattened codeword array (data
 * codewords in [0,nd), ec codewords in [nd,nd+ne)); nb is the ECC200
 * block count from the size table (Section 7). On success, corrected data
 * codewords are written back into cw[0..nd) in place (ec codewords are
 * left untouched -- downstream payload decode only reads the first nd
 * entries) and *out_confidence receives the reference's integer
 * confidence metric.
 */
#define DM_MAX_BLOCK_CW DM_MAX_TOTAL_CW

static int dm_rs_correct(uint8_t *cw, int nd, int ne, int nb, int *out_confidence)
{
    if (nd < 0 || ne <= 0 || nb <= 0) return 0;
    int total = nd + ne;
    if (total <= 0 || total > DM_MAX_TOTAL_CW) return 0;

    gf256_init();
    GField gf = { gf256_exp, gf256_log, 255 };

    if (nb == 1) {
        /* Single-block path (263578-263623): RS over the whole array,
         * then an all-zero-codeword defense against a spurious "clean"
         * decode of a blank/mis-registered symbol. */
        int buf[DM_MAX_TOTAL_CW];
        for (int i = 0; i < total; i++) buf[i] = cw[i];
        int corr = rs_decode(&gf, buf, total, ne);
        if (corr < 0) return 0;

        int all_zero = 1;
        for (int i = 0; i < total; i++) {
            if (buf[i] != 0) { all_zero = 0; break; }
        }
        if (all_zero) return 0;

        for (int i = 0; i < total; i++) cw[i] = (uint8_t)buf[i];
        *out_confidence = 100 - 200 * corr / (ne + 1);
        return 1;
    }

    /* Multi-block round-robin de-interleave path (263493-263572). ec_pb is
     * the fixed per-block ec codeword count; the data codeword count per
     * block varies (e.g. 8x156 + 2x155 for 144x144) and falls out
     * naturally from the `i += nb` stride below -- no explicit block-size
     * table is needed. */
    int ec_pb = ne / nb;
    if (ec_pb <= 0) return 0;

    int conf = 100;
    int off = 0; /* ec read-offset convention; 144x144 retry only */
    int b = 0;
    while (b < nb) {
        if (off != 0 && b == 8) off = -8;

        int blk[DM_MAX_BLOCK_CW];
        int blen = 0;
        for (int i = b; i < nd; i += nb) blk[blen++] = cw[i];
        int data_len = blen;
        for (int j = b; j < ne; j += nb) {
            int ec_idx = nd + off + j;
            if (ec_idx < nd || ec_idx >= total) { blen = -1; break; }
            blk[blen++] = cw[ec_idx];
        }
        int corr = (blen >= 0) ? rs_decode(&gf, blk, blen, ec_pb) : -1;

        if (corr >= 0) {
            int c = 100 - 200 * corr / (ec_pb + 1);
            if (c < conf) conf = c;
            int k = 0;
            for (int i = b; i < nd; i += nb) cw[i] = (uint8_t)blk[k++];
            (void)data_len;
            b++;
        } else {
            /* Only the 144x144 symbol (nd==1558) gets one retry, and only
             * when block 0 fails on the very first (standard) convention
             * (263547-263556). Any other failure aborts the whole decode. */
            if (b == 0 && off == 0 && nd == 1558) {
                off = 2;
                conf = 100;
                b = 0;
                continue;
            }
            return 0;
        }
    }

    *out_confidence = conf;
    return 1;
}

/* ── Section 15: payload decode (data codewords -> text) ──────────────────
 * Faithful transcription of SmartSymbolRedef00013f (58483-58950, the main
 * ASCII/C40/Text/X12/EDIFACT/Base256 state machine) and sub_59CB8
 * (58952-59066, the C40/Text/X12 triple-value emitter -> dm_emit_c40()).
 * Character tables: the C40/Text base (shift0) tables and the X12 table
 * are directly readable in the dump (off_1A4980/off_1A4990 @ 13376-13377,
 * a0123456789abcd @ 2185); shift1/2/3 table bodies themselves are
 * weak/unlisted symbols, but the FNC1(=27)/UpperShift(=30) special cases
 * hard-coded in sub_59CB8 for shift2 pin those two entries exactly, and
 * the rest follow ISO/IEC 16022 Table 5/6 (cross-checked against those
 * two known entries' positions).
 *
 * Deviation from the reference (intentional, matches this restoration's
 * interface -- BarcodeResult has no field for a scanner-added symbology
 * ID prefix): the reference reserves out[0..2] for an AIM "]dN" prefix
 * before decoding starts (out initialised to 3) and fills it in from
 * a123456R[] at the end (58750-58754). This restoration starts the
 * output buffer at 0 and never writes that prefix; the underlying flags
 * bit computation (FNC1-first/-second -> GS1/AIM detection) is still
 * carried out faithfully in `flags` but isn't surfaced anywhere, matching
 * the target GS1 sample's expected string (no "]d2" prefix).
 */

typedef enum {
    DM_MODE_ASCII = 0,
    DM_MODE_C40,
    DM_MODE_TEXT,
    DM_MODE_X12,
    DM_MODE_EDIFACT,
    DM_MODE_BASE256,
} DmPayloadMode;

/* C40/Text basic (shift0) set: index 0-2 unused (those values latch a
 * pending shift instead of emitting), 3=' ', 4-13='0'-'9',
 * 14-39='A'-'Z' (C40) / 'a'-'z' (Text). off_1A4980/off_1A4990 (13376-77). */
static const char k_dm_c40_basic[40] = {
    0, 0, 0, ' ',
    '0','1','2','3','4','5','6','7','8','9',
    'A','B','C','D','E','F','G','H','I','J','K','L','M',
    'N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
};
static const char k_dm_text_basic[40] = {
    0, 0, 0, ' ',
    '0','1','2','3','4','5','6','7','8','9',
    'a','b','c','d','e','f','g','h','i','j','k','l','m',
    'n','o','p','q','r','s','t','u','v','w','x','y','z',
};

/* Shift2 set, shared by C40 and Text (ISO/IEC 16022 Table 5/6). Values 27
 * (FNC1) and 30 (Upper Shift) are special-cased directly in dm_emit_c40()
 * per sub_59CB8, not looked up here; 28/29/31 are reserved/unused. */
static const char k_dm_shift2[32] = {
    '!','"','#','$','%','&','\'','(',')','*','+',',','-','.','/',
    ':',';','<','=','>','?','@','[','\\',']','^','_',
    0, 0, 0, 0, 0,
};

/* Shift3: C40 is a straight 0x60+v offset (computed inline, no table
 * needed); Text keeps the same punctuation tail but uses uppercase
 * letters for the run that C40 gives lowercase. */
static const char k_dm_text_shift3[32] = {
    '`',
    'A','B','C','D','E','F','G','H','I','J','K','L','M',
    'N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
    '{','|','}','~', 0x7F,
};

/* X12 table (a0123456789abcd, 2185): CR,'*','>',' ','0'-'9','A'-'Z'. */
static const char k_dm_x12[40] = {
    '\r','*','>',' ',
    '0','1','2','3','4','5','6','7','8','9',
    'A','B','C','D','E','F','G','H','I','J','K','L','M',
    'N','O','P','Q','R','S','T','U','V','W','X','Y','Z',
};

typedef struct {
    const uint8_t *cw;
    int count;
    int pos;
    int mode;
    char *out;
    int out_len;
    int out_cap;
    int shift;
    int upsh;
    int macro;
    int rdprog;
    int flags;
    int err;
} DmPayloadState;

static void dm_emit(DmPayloadState *st, int c)
{
    if (st->out_len < st->out_cap) {
        st->out[st->out_len++] = (char)(uint8_t)c;
    }
}

/* Applies a pending Upper Shift (+128, byte wraparound) to the byte just
 * written -- sub_59CB8's "*(byte*)(...) += -128" idiom. */
static void dm_apply_upsh(DmPayloadState *st)
{
    if (st->out_len > 0) {
        int v = (uint8_t)st->out[st->out_len - 1] + 128;
        st->out[st->out_len - 1] = (char)(uint8_t)(v & 0xFF);
    }
}

static char dm_c40text_char(int mode, int shift, int v)
{
    if (v < 0 || v > 39) return 0;
    if (shift == 0) {
        return (mode == DM_MODE_C40) ? k_dm_c40_basic[v] : k_dm_text_basic[v];
    }
    if (shift == 1) {
        return (char)(uint8_t)v; /* 0x00-0x1F control chars, value==code */
    }
    if (v > 31) return 0;
    if (shift == 2) {
        return k_dm_shift2[v];
    }
    /* shift == 3 */
    return (mode == DM_MODE_C40) ? (char)(uint8_t)(0x60 + v) : k_dm_text_shift3[v];
}

/* sub_59CB8 (58952-59066): emits one C40/Text/X12 triple value (0-39). */
static void dm_emit_c40(DmPayloadState *st, int v)
{
    if (st->mode == DM_MODE_X12) {
        dm_emit(st, (v >= 0 && v < 40) ? k_dm_x12[v] : 0);
        return;
    }
    if (st->shift == 0) {
        if (v <= 2) {
            st->shift = v + 1; /* latch pending shift1/2/3 */
        } else {
            dm_emit(st, dm_c40text_char(st->mode, 0, v));
            if (st->upsh) { dm_apply_upsh(st); st->upsh = 0; }
        }
        return;
    }
    if (v == 30 && st->shift == 2) {
        st->upsh = 1;
        st->shift = 0;
        return;
    }
    if (v == 27 && st->shift == 2) {
        /* FNC1 in shift2: position check uses `pos` as it stands *after*
         * this triple's 2 source codewords were already consumed by the
         * caller (matches v11 = *(a1+47560) read inside sub_59CB8). */
        if (st->pos == 2) st->flags |= 2;
        else if (st->pos == 1) st->flags |= 1;
        else dm_emit(st, 0x1D);
    } else {
        dm_emit(st, dm_c40text_char(st->mode, st->shift, v));
    }
    st->shift = 0;
    if (st->upsh) { dm_apply_upsh(st); st->upsh = 0; } /* quirk: applies even after the FNC1 branch */
}

/* Base256 de-randomization (ISO/IEC 16022 Annex B / SmartSymbolRedef000100
 * neighbourhood at 58768-58838): byte = (cw - ((149*p) mod 255) - 1) mod
 * 256, where p is the codeword's 1-based position in the data stream. */
static int dm_base256_unrand(int cw, int p)
{
    int r = (149 * p) % 255;
    int v = cw - r - 1;
    v = ((v % 256) + 256) % 256;
    return v;
}

/* SmartSymbolRedef00013f (58483-58950): the main codeword -> text state
 * machine. On error the reference stops consuming further codewords but
 * keeps whatever was already emitted (LABEL_88/98 both fall through to
 * the same trailer) -- this mirrors that rather than discarding output. */
static int dm_decode_payload(const uint8_t *cw, int count, char *out, int out_cap)
{
    DmPayloadState st;
    memset(&st, 0, sizeof(st));
    st.cw = cw;
    st.count = count;
    st.out = out;
    st.out_cap = out_cap;
    st.mode = DM_MODE_ASCII;

    if (count < 1) goto done;

    while (st.pos < st.count) {
        switch (st.mode) {
        case DM_MODE_ASCII: {
            int c = st.cw[st.pos];
            st.pos += 1;
            if (c == 0) { st.err = 1000; goto done; }
            if (c <= 128) {
                if (st.upsh) { c += 128; st.upsh = 0; }
                dm_emit(&st, c - 1);
                break;
            }
            st.upsh = 0;
            if (c == 129) { goto done; } /* Pad */
            if (c <= 229) {
                int v = c - 130;
                dm_emit(&st, '0' + v / 10);
                dm_emit(&st, '0' + v % 10);
                break;
            }
            switch (c) {
            case 230: st.mode = DM_MODE_C40; break;
            case 231: st.mode = DM_MODE_BASE256; break;
            case 232: /* FNC1 */
                if (st.pos - 1 == 0) st.flags |= 1;
                else if (st.pos - 1 == 1) st.flags |= 2;
                else dm_emit(&st, 0x1D);
                break;
            case 233: /* Structured Append: reference no-ops (quirk 1) */
            case 254: /* reference no-ops (quirk 2) */
                break;
            case 234: /* Reader Programming, first codeword only */
                if (st.pos - 1 == 0) st.rdprog = 1;
                else { st.err = 1234; goto done; }
                break;
            case 235: /* Upper Shift */
                st.upsh = 1;
                break;
            case 236: /* Macro 05 */
            case 237: /* Macro 06 */
                if (st.pos - 1 == 0) {
                    dm_emit(&st, '['); dm_emit(&st, ')'); dm_emit(&st, '>'); dm_emit(&st, 0x1E);
                    dm_emit(&st, '0'); dm_emit(&st, (c == 236) ? '5' : '6'); dm_emit(&st, 0x1D);
                    st.macro = c - 231;
                }
                break;
            case 238: st.mode = DM_MODE_X12; break;
            case 239: st.mode = DM_MODE_TEXT; break;
            case 240: st.mode = DM_MODE_EDIFACT; break;
            case 241: { /* ECI */
                if (st.pos >= st.count) { st.err = 1001; goto done; }
                int c1 = st.cw[st.pos]; st.pos += 1;
                if (c1 == 0) { st.err = 1001; goto done; }
                long eci;
                if (c1 <= 127) {
                    eci = c1 - 1;
                } else if (c1 <= 191) {
                    if (st.pos >= st.count) { st.err = 1001; goto done; }
                    int c2 = st.cw[st.pos]; st.pos += 1;
                    eci = 254L * c1 + c2 - 32386;
                } else if (c1 <= 207) {
                    if (st.pos + 1 >= st.count) { st.err = 1001; goto done; }
                    int c2 = st.cw[st.pos];
                    int c3 = st.cw[st.pos + 1];
                    st.pos += 2;
                    eci = 64516L * c1 + 254L * c2 + c3 - 12370944;
                } else {
                    st.err = 1002; goto done;
                }
                /* unk_2738C4 (ECI escape config) treated as the default
                 * constant 0: '\' is not doubled, ECI is emitted as the
                 * literal "\NNNNNN" (SmartSymbolRedef0000d2 @ 52413). */
                if (eci < 0) eci = 0;
                if (eci > 999999) eci %= 1000000L;
                dm_emit(&st, '\\');
                char digits[7];
                snprintf(digits, sizeof(digits), "%06ld", eci);
                for (int k = 0; k < 6; k++) dm_emit(&st, digits[k]);
                st.flags |= 4;
                break;
            }
            default:
                if (c >= 242 && c <= 253) { st.err = c + 1100; goto done; }
                break; /* unreachable for a valid byte codeword */
            }
            break;
        }

        case DM_MODE_C40:
        case DM_MODE_TEXT:
        case DM_MODE_X12: {
            if (st.pos >= st.count) goto done;
            int c1 = st.cw[st.pos];
            st.pos += 1;
            if (c1 == 254) { /* unlatch */
                st.mode = DM_MODE_ASCII;
                st.shift = 0;
                st.upsh = 0;
                break;
            }
            if (st.pos >= st.count) goto done; /* defensive clamp, quirk 3 */
            int c2 = st.cw[st.pos];
            st.pos += 1;
            int v = c1 * 256 + c2 - 1;
            dm_emit_c40(&st, v / 1600);
            dm_emit_c40(&st, (v % 1600) / 40);
            dm_emit_c40(&st, v % 40);
            if (st.pos + 1 == st.count) {
                if (st.cw[st.pos] == 254) {
                    st.pos += 1;
                    goto done;
                }
                st.mode = DM_MODE_ASCII; /* implicit unlatch, single trailing codeword */
            } else if (st.pos >= st.count) {
                goto done;
            }
            break;
        }

        case DM_MODE_EDIFACT: {
            if (st.pos >= st.count) goto done;
            int c1 = (st.pos < st.count) ? st.cw[st.pos] : 0;
            int c2 = (st.pos + 1 < st.count) ? st.cw[st.pos + 1] : 0; /* defensive, quirk 3 */
            int c3 = (st.pos + 2 < st.count) ? st.cw[st.pos + 2] : 0;
            st.pos += 3;
            uint32_t packed = ((uint32_t)c1 << 16) | ((uint32_t)c2 << 8) | (uint32_t)c3;
            int unlatched = 0;
            for (int k = 0; k < 4; k++) {
                int u = (int)((packed >> (18 - 6 * k)) & 0x3F);
                if (u == 31) { /* unlatch */
                    if (k == 0) st.pos -= 2;
                    else if (k == 1) st.pos -= 1;
                    st.mode = DM_MODE_ASCII;
                    unlatched = 1;
                    break;
                }
                if (u < 0x20) u |= 0x40;
                dm_emit(&st, u);
            }
            if (!unlatched) {
                if (st.pos + 2 >= st.count) {
                    st.mode = DM_MODE_ASCII; /* implicit unlatch, <=2 codewords left */
                } else if (st.pos >= st.count) {
                    goto done;
                }
            }
            break;
        }

        case DM_MODE_BASE256: {
            if (st.pos >= st.count) goto done;
            int p1 = st.pos + 1;
            int d1 = dm_base256_unrand(st.cw[st.pos], p1);
            st.pos += 1;
            int len;
            if (d1 == 0) {
                len = st.count - st.pos;
            } else if (d1 >= 250) {
                if (st.pos >= st.count) goto done;
                int p2 = st.pos + 1;
                int d2 = dm_base256_unrand(st.cw[st.pos], p2);
                st.pos += 1;
                len = 250 * d1 + d2 - 62250;
            } else {
                len = d1;
            }
            if (len < 0) len = 0;
            if (st.pos + len > st.count) len = st.count - st.pos;
            for (int k = 0; k < len; k++) {
                int p3 = st.pos + 1;
                dm_emit(&st, dm_base256_unrand(st.cw[st.pos], p3));
                st.pos += 1;
            }
            st.mode = DM_MODE_ASCII;
            break;
        }

        default:
            goto done;
        }
    }

done:
    if (st.macro >= 1) {
        dm_emit(&st, 0x1E);
        dm_emit(&st, 0x04);
    }
    if (st.out_cap > 0) {
        int term = (st.out_len < st.out_cap) ? st.out_len : st.out_cap - 1;
        st.out[term] = '\0';
    }
    (void)st.rdprog;
    (void)st.err;
    return st.out_len;
}

/* ── Public entry point ──────────────────────────────────────────────── */

static int dm_decode_core(const Image *img, BarcodeResult *result)
{
    uint64_t total_start = dm_now_us();
    datamatrix_debug_stats_reset();
    if (result) {
        memset(result, 0, sizeof(*result));
    }
    if (!img || !img->pixels || img->width <= 0 || img->height <= 0) {
        return -1;
    }

    g_dm_stats.decode_attempts++;

    DmFinderCandidate cand[DM_MAX_CAND];
    uint64_t stage_start = dm_now_us();
    int ncand = dm_find_finders(img, cand, DM_MAX_CAND);
    g_dm_stats.finder_us += dm_now_us() - stage_start;
    g_dm_stats.finder_hits += (uint32_t)ncand;

    /* M6: the reference has no internal mirror retry (mirror handling lives
     * at the API layer, see the Section-11 comment on dm_analyze_candidate);
     * this is a design-layer extension for our harness. For each finder
     * candidate try both chiralities -- mirror=0 (normal) first, then
     * mirror=1 (axis A/B swapped, see dm_analyze_candidate) -- and stop at
     * the first one that survives RS correction and payload decode. Normal
     * symbols always resolve on mirror=0 with zero extra cost; only
     * candidates that fail mirror=0 pay for a second grid sample. */
    for (int i = 0; i < ncand; i++) {
        for (int mirror = 0; mirror <= 1; mirror++) {
            stage_start = dm_now_us();
            DmSizeInfo info;
            uint8_t *mapping = NULL;
            if (!dm_analyze_candidate(img, &cand[i], mirror, &info, &mapping)) {
                g_dm_stats.candidate_us += dm_now_us() - stage_start;
                continue;
            }
            g_dm_stats.candidate_us += dm_now_us() - stage_start;

            g_dm_stats.grid_decode_hits++;

            uint8_t cw[DM_MAX_TOTAL_CW];
            stage_start = dm_now_us();
            int n_cw = dm_extract_codewords_from_mapping(
                mapping, info.mapping_rows, info.mapping_cols, cw, DM_MAX_TOTAL_CW);
            g_dm_stats.codeword_us += dm_now_us() - stage_start;
            DM_DBG("dm: placed %d codewords (expect total_cw=%d, data=%d ec=%d)\n",
                   n_cw, info.total_cw, info.data_cw, info.ec_cw);

            if (n_cw >= info.total_cw && info.total_cw == info.data_cw + info.ec_cw) {
                int confidence = 0;
                stage_start = dm_now_us();
                int corrected = dm_rs_correct(cw, info.data_cw, info.ec_cw, info.blocks, &confidence);
                g_dm_stats.ecc_us += dm_now_us() - stage_start;
                if (corrected) {
                    g_dm_stats.ec_decode_hits++;
                    DM_DBG("dm: RS correction OK, confidence=%d\n", confidence);

                    /* cw[0..data_cw) now holds RS-corrected data codewords;
                     * decode them into text (Section 15). */
                    stage_start = dm_now_us();
                    int declen = dm_decode_payload(cw, info.data_cw, result->data,
                                                    (int)sizeof(result->data) - 1);
                    g_dm_stats.payload_us += dm_now_us() - stage_start;
                    DM_DBG("dm: payload decode -> %d bytes\n", declen);

                    if (declen > 0) {
                        g_dm_stats.payload_decode_hits++;
                        if (mirror) g_dm_mirror_hits++;
                        result->length = declen;
                        result->confidence = confidence;
                        result->symbology = BARCODE_SYM_DATAMATRIX;
                        g_dm_stats.total_us = dm_now_us() - total_start;
                        free(mapping);
                        return declen;
                    }
                } else {
                    DM_DBG("dm: RS correction failed\n");
                }
            }

            free(mapping);
        }
    }

    g_dm_stats.total_us = dm_now_us() - total_start;
    return 0;
}

static int dm_rotate_gray(const Image *src, Image *dst, double angle_degrees)
{
    size_t size = (size_t)src->width * src->height;
    dst->width = src->width;
    dst->height = src->height;
    dst->pixels = malloc(size);
    if (!dst->pixels) return 0;
    double radians = angle_degrees * 3.14159265358979323846 / 180.0;
    double c = cos(radians);
    double s = sin(radians);
    double cx = (src->width - 1) * 0.5;
    double cy = (src->height - 1) * 0.5;
    for (int y = 0; y < dst->height; y++) {
        for (int x = 0; x < dst->width; x++) {
            double dx = x - cx;
            double dy = y - cy;
            double sx = c * dx + s * dy + cx;
            double sy = -s * dx + c * dy + cy;
            int ix = (int)lround(sx);
            int iy = (int)lround(sy);
            dst->pixels[(size_t)y * dst->width + x] =
                (ix >= 0 && ix < src->width && iy >= 0 && iy < src->height) ?
                src->pixels[(size_t)iy * src->width + ix] : 255;
        }
    }
    return 1;
}

int datamatrix_decode(const Image *img, BarcodeResult *result)
{
    static const double deskew_angles[] = {
        0.0, 5.0, -5.0, 10.0, -10.0, 15.0, -15.0,
        20.0, -20.0, 25.0, -25.0, 30.0, -30.0,
        35.0, -35.0, 40.0, -40.0, 45.0, -45.0
    };
    if (!img || !img->pixels || !result) return -1;
    for (size_t attempt = 0; attempt < sizeof(deskew_angles) / sizeof(deskew_angles[0]); attempt++) {
        Image candidate = {0};
        if (deskew_angles[attempt] == 0.0) {
            candidate = *img;
        } else if (!dm_rotate_gray(img, &candidate, deskew_angles[attempt])) {
            continue;
        }
        int decoded = dm_decode_core(&candidate, result);
        if (candidate.pixels != img->pixels) free(candidate.pixels);
        if (decoded > 0) return decoded;
    }
    return 0;
}
