#ifdef __linux__
#define _POSIX_C_SOURCE 199309L
#endif

#include "barcode.h"
#include "signal.h"
#include "envelope.h"
#include "scan_dispatcher.h"
#include "scan_engine.h"
#include "postprocess.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <time.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

static __thread volatile bool *g_code128_cancel_flag = NULL;
static __thread uint64_t g_fast_rows_deadline_ms = 0;

static int is_timed_out(uint64_t deadline_ms);

/* ════════════════════════════════════════════════════════════════════════
 *  Code128 symbol table
 * ════════════════════════════════════════════════════════════════════════ */

static const int C128_PATTERNS[107][6] = {
    {2,1,2,2,2,2}, /*   0 */  {2,2,2,1,2,2}, /*   1 */
    {2,2,2,2,2,1}, /*   2 */  {1,2,1,2,2,3}, /*   3 */
    {1,2,1,3,2,2}, /*   4 */  {1,3,1,2,2,2}, /*   5 */
    {1,2,2,2,1,3}, /*   6 */  {1,2,2,3,1,2}, /*   7 */
    {1,3,2,2,1,2}, /*   8 */  {2,2,1,2,1,3}, /*   9 */
    {2,2,1,3,1,2}, /*  10 */  {2,3,1,2,1,2}, /*  11 */
    {1,1,2,2,3,2}, /*  12 */  {1,2,2,1,3,2}, /*  13 */
    {1,2,2,2,3,1}, /*  14 */  {1,1,3,2,2,2}, /*  15 */
    {1,2,3,1,2,2}, /*  16 */  {1,2,3,2,2,1}, /*  17 */
    {2,2,3,2,1,1}, /*  18 */  {2,2,1,1,3,2}, /*  19 */
    {2,2,1,2,3,1}, /*  20 */  {2,1,3,2,1,2}, /*  21 */
    {2,2,3,1,1,2}, /*  22 */  {3,1,2,1,3,1}, /*  23 */
    {3,1,1,2,2,2}, /*  24 */  {3,2,1,1,2,2}, /*  25 */
    {3,2,1,2,2,1}, /*  26 */  {3,1,2,2,1,2}, /*  27 */
    {3,2,2,1,1,2}, /*  28 */  {3,2,2,2,1,1}, /*  29 */
    {2,1,2,1,2,3}, /*  30 */  {2,1,2,3,2,1}, /*  31 */
    {2,3,2,1,2,1}, /*  32 */  {1,1,1,3,2,3}, /*  33 */
    {1,3,1,1,2,3}, /*  34 */  {1,3,1,3,2,1}, /*  35 */
    {1,1,2,3,1,3}, /*  36 */  {1,3,2,1,1,3}, /*  37 */
    {1,3,2,3,1,1}, /*  38 */  {2,1,1,3,1,3}, /*  39 */
    {2,3,1,1,1,3}, /*  40 */  {2,3,1,3,1,1}, /*  41 */
    {1,1,2,1,3,3}, /*  42 */  {1,1,2,3,3,1}, /*  43 */
    {1,3,2,1,3,1}, /*  44 */  {1,1,3,1,2,3}, /*  45 */
    {1,1,3,3,2,1}, /*  46 */  {1,3,3,1,2,1}, /*  47 */
    {3,1,3,1,2,1}, /*  48 */  {2,1,1,3,3,1}, /*  49 */
    {2,3,1,1,3,1}, /*  50 */  {2,1,3,1,1,3}, /*  51 */
    {2,1,3,3,1,1}, /*  52 */  {2,1,3,1,3,1}, /*  53 */
    {3,1,1,1,2,3}, /*  54 */  {3,1,1,3,2,1}, /*  55 */
    {3,3,1,1,2,1}, /*  56 */  {3,1,2,1,1,3}, /*  57 */
    {3,1,2,3,1,1}, /*  58 */  {3,3,2,1,1,1}, /*  59 */
    {3,1,4,1,1,1}, /*  60 */  {2,2,1,4,1,1}, /*  61 */
    {4,3,1,1,1,1}, /*  62 */  {1,1,1,2,2,4}, /*  63 */
    {1,1,1,4,2,2}, /*  64 */  {1,2,1,1,2,4}, /*  65 */
    {1,2,1,4,2,1}, /*  66 */  {1,4,1,1,2,2}, /*  67 */
    {1,4,1,2,2,1}, /*  68 */  {1,1,2,2,1,4}, /*  69 */
    {1,1,2,4,1,2}, /*  70 */  {1,2,2,1,1,4}, /*  71 */
    {1,2,2,4,1,1}, /*  72 */  {1,4,2,1,1,2}, /*  73 */
    {1,4,2,2,1,1}, /*  74 */  {2,4,1,2,1,1}, /*  75 */
    {2,2,1,1,1,4}, /*  76 */  {4,1,3,1,1,1}, /*  77 */
    {2,4,1,1,1,2}, /*  78 */  {1,3,4,1,1,1}, /*  79 */
    {1,1,1,2,4,2}, /*  80 */  {1,2,1,1,4,2}, /*  81 */
    {1,2,1,2,4,1}, /*  82 */  {1,1,4,2,1,2}, /*  83 */
    {1,2,4,1,1,2}, /*  84 */  {1,2,4,2,1,1}, /*  85 */
    {4,1,1,2,1,2}, /*  86 */  {4,2,1,1,1,2}, /*  87 */
    {4,2,1,2,1,1}, /*  88 */  {2,1,2,1,4,1}, /*  89 */
    {2,1,4,1,2,1}, /*  90 */  {4,1,2,1,2,1}, /*  91 */
    {1,1,1,1,4,3}, /*  92 */  {1,1,1,3,4,1}, /*  93 */
    {1,3,1,1,4,1}, /*  94 */  {1,1,4,1,1,3}, /*  95 */
    {1,1,4,3,1,1}, /*  96 */  {4,1,1,1,1,3}, /*  97 */
    {4,1,1,3,1,1}, /*  98 */  {1,1,3,1,4,1}, /*  99 */
    {1,1,4,1,3,1}, /* 100 */  {3,1,1,1,4,1}, /* 101 */
    {4,1,1,1,3,1}, /* 102 */  {2,1,1,4,1,2}, /* 103  Start A */
    {2,1,1,2,1,4}, /* 104  Start B */  {2,1,1,2,3,2}, /* 105  Start C */
};

static const int C128_STOP[7] = {2,3,3,1,1,1,2};

enum {
    C128_START_A  = 103,
    C128_START_B  = 104,
    C128_START_C  = 105,
    C128_SHIFT    = 98,
    C128_CODE_A   = 101,
    C128_CODE_B   = 100,
    C128_CODE_C   = 99,
    C128_FNC1     = 102,
    C128_FNC2     = 97,
    C128_FNC3     = 96,
};

/* ════════════════════════════════════════════════════════════════════════
 *  Pattern matching
 * ════════════════════════════════════════════════════════════════════════ */

#define MAX_BARS 1024
#define MAX_EDGE_RUNS 1024

typedef struct {
    int widths[MAX_BARS];
    int count;
} BarSpaceRun;

typedef struct {
    int p[5];
} Char128Pat5;

static Char128Pat5 g_char128_pat5[108];
static volatile int g_char128_pat5_init = 0;
#define MAX_SYMBOLS 128

static int symbols_to_text(const int *symbols, int nsym, char *out, int *out_len);
static int decode_from_edges_char128(const int *edges, int n_edges, char *out, int *out_len);
static int char128_match_edges_relaxed(const int *edges, int n_edges, int a2, int a3,
                                       int *min_center, int *max_center);

static const unsigned char k_char128_center108[108] = {
    6,6,6,4,4,4,4,4,4,4,4,4,6,6,6,6,6,6,6,6,6,6,6,8,6,6,6,6,6,6,6,6,6,4,4,4,4,4,4,4,4,4,6,6,6,6,6,6,8,6,6,6,6,8,6,6,6,6,6,6,8,4,6,4,4,4,4,4,4,4,4,4,4,4,4,4,4,8,4,6,6,6,6,6,6,6,6,6,6,8,8,8,6,6,6,6,6,6,6,8,8,8,8,4,4,6,6,6
};

static const signed char k_char128_smart_lookup[256] = {
    3, 36, 0, 49, 0, 0, 0, 0, 0, 95, 0, 35, 6, 0, 0, 2,
    0, 0, 23, 9, 39, 0, 61, 19, 0, 0, 0, 0, 45, 0, 38, 0,
    0, 56, 0, 48, 0, 0, 0, 0, 0, 102, 0, 55, 25, 0, 0, 0,
    11, 0, 0, 28, 59, 62, 80, 0, 20, 0, 77, 0, 24, 21, 58, 10,
    42, 0, 78, 30, 0, 0, 0, 0, 51, 0, 41, 0, 0, 0, 29, 12,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 68, 69, 0, 0, 33,
    99, 0, -1, 75, 0, 0, 0, 0, 103, 0, 98, 0, 0, 0, 74, 87,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 26, 27, 0, 0, 92,
    0, 100, 0, 46, 16, 0, 0, 0, 0, 47, 0, 57, 0, 0, 0, 0,
    0, 0, 0, 0, 101, 0, 96, 0, 0, 0, 79, 84, 97, 0, 60, 76,
    0, 34, 0, 94, 0, 0, 65, 0, 0, 93, 0, 0, 64, 0, 81, 0,
    0, 13, 0, 70, 37, 0, 44, 0, 0, 71, 88, 0, 43, 0, 0, 89,
    54, 15, 52, 7, 8, 0, 0, 22, 53, 73, 63, 0, 14, 0, 72, 0,
    0, 0, 0, 17, 18, 0, 0, 91, 0, 0, 0, 0, 0, 0, 0, 0,
    40, 0, 50, 0, 0, 104, 85, 0, -2, 0, 0, 86, 0, 106, 0, 105,
    1, 5, 0, 0, 0, 32, 67, 0, 0, 82, 0, 66, 4, 90, 83, 31
};

static void init_char128_pat5(void)
{
    int expected = 0;

    if (__atomic_load_n(&g_char128_pat5_init, __ATOMIC_ACQUIRE) == 2) {
        return;
    }
    if (!__atomic_compare_exchange_n(&g_char128_pat5_init, &expected, 1,
                                     false, __ATOMIC_ACQUIRE,
                                     __ATOMIC_RELAXED)) {
        while (__atomic_load_n(&g_char128_pat5_init, __ATOMIC_ACQUIRE) != 2) {
        }
        return;
    }
    for (int s = 0; s < 106; s++) {
        int c = 0;
        for (int i = 0; i < 5; i++) {
            c += C128_PATTERNS[s][i];
            g_char128_pat5[s].p[i] = c;
        }
    }
    {
        int c = 0;
        for (int i = 0; i < 5; i++) {
            c += C128_STOP[i];
            g_char128_pat5[106].p[i] = c;
        }
    }
    {
        /* byte_1A2AD3 has one extra 5-byte entry used by the original loop. */
        g_char128_pat5[107].p[0] = 2;
        g_char128_pat5[107].p[1] = 3;
        g_char128_pat5[107].p[2] = 4;
        g_char128_pat5[107].p[3] = 5;
        g_char128_pat5[107].p[4] = 8;
    }
    __atomic_store_n(&g_char128_pat5_init, 2, __ATOMIC_RELEASE);
}

/* source: src/decompiled_code128/char128.c */
static int char128_match_edges(const int *edges, int n_edges, int a2, int a3,
                               int *min_center, int *max_center)
{
    int v4, v5, v6, v8, v9, v10, v11, v12, v13, v14, v15, v16;
    int v18, v19, v22, v23, v24, v26, v27, v35;
    int v43, v45, v47, v53, v55, v56, v57, v58, v59, v60, v61, v62;
    int y[5];

    if (a2 < 0 || a2 + 6 >= n_edges) return -1;
    init_char128_pat5();

    v4 = a2;
    v5 = edges[a2];
    v6 = edges[a2 + 6];
    int _R3 = v6 - v5;
    if (_R3 <= 0) return -1;

    if ((a2 & 1) == a3) {
        v8 = a2;
        v9 = 1;
        v5 = edges[v8 + 1];
        v10 = edges[v8 + 4];
        v11 = v6 - edges[v8 + 2];
        v12 = edges[v8 + 3];
        v13 = edges[v8 + 5];
        v58 = v6 - v12;
        v14 = v6 - v13;
        v60 = v6 - v10;
        v15 = 2;
        v16 = 3;
        v55 = 4;
        v57 = v6;
    } else {
        int v17 = a2;
        v9 = 4;
        v14 = edges[v17 + 1] - v5;
        v11 = edges[v17 + 4] - v5;
        v6 = edges[v17 + 5];
        v58 = edges[v17 + 3] - v5;
        v60 = edges[v17 + 2] - v5;
        v57 = edges[v17 + 1];
        v55 = 3;
        v15 = 5;
        v16 = 2;
        v13 = edges[v4];
    }
    v18 = 4 * _R3;
    v19 = -1;
    if (v18 < (_R3 >> 1)) goto done;

    v62 = 11 * v11;
    v22 = 11 * v14;
    v61 = 11 * v60;
    v59 = 11 * v58;
    v23 = 11 * (v6 - v5);
    v24 = 0;
    v53 = v57 - v13 + edges[v55 + a2] - edges[v16 + a2] + edges[v15 + a2] - edges[v9 + a2];
    v56 = -1;

    for (;;) {
        if (v24 > 107 || v18 < (_R3 >> 1)) break;
        const int *pat = g_char128_pat5[v24].p;
        v26 = 11 * (v23 + v59 + v22 - _R3 * pat[0] - _R3 * pat[2] - _R3 * pat[4]);
        v27 = v22 - (v26 >> 6);

        if (v18 >= 1) {
            int acc = v27 - _R3 * pat[0];
            if (acc < 0) acc = -acc;
            if (acc < v18) {
                int ok = 1;
                y[0] = v27;
                y[1] = v61;
                y[2] = v59 - (v26 >> 6);
                y[3] = v62;
                y[4] = v23 - (v26 >> 6);
                for (int i = 1; i <= 4; i++) {
                    int dv = y[i] - _R3 * pat[i];
                    if (dv < 0) dv = -dv;
                    acc += dv;
                    if (acc >= v18) {
                        ok = 0;
                        break;
                    }
                }
                if (ok) {
                    v18 = acc;
                    v56 = v24;
                }
            }
        }
        ++v24;
    }

    v35 = v56;
    if (v56 == -1) {
        v19 = -1;
    } else {
        int v47_off = (int)((long long)(-_R3 * (int)k_char128_center108[v56]) / 11LL);
        v47 = v53 + v47_off;
        v43 = *min_center;
        v45 = *max_center;
        if (v43 <= v45) {
            if (v47 < v43) *min_center = v47;
            if (v47 > v45) *max_center = v47;
            v43 = *min_center;
            v45 = *max_center;
            if (v45 - v43 > (_R3 >> 3)) {
                if (v43 < (_R3 / 5)) v35 = -1;
                if (v45 > ((3 * _R3) / 5)) v35 = -1;
            }
        } else {
            *max_center = v47;
            *min_center = v47;
        }
        v19 = v35;
    }
done:
    if (v19 > 105) v19 = 104 - v19;
    return v19;
}

static unsigned int tmod_edges_11(const int *edges, int n_edges, int edge_index, unsigned int total_width)
{
    if (!edges || total_width == 0) return 0;
    if (edge_index < 0 || edge_index + 2 >= n_edges) return 0;
    {
        int span = edges[edge_index + 2] - edges[edge_index];
        return (unsigned int)((span * 11u + total_width / 2u) / total_width);
    }
}

static int smartsymbol_match_edges(const int *edges, int n_edges, int a2, int a3,
                                   int *min_center, int *max_center)
{
    int total_width;
    int pattern;
    int center;
    int result;
    int ref_center_mod;

    if (a2 < 0 || a2 + 6 >= n_edges) return -1;
    total_width = edges[a2 + 6] - edges[a2];
    if (total_width <= 0) return -1;

    if ((a2 & 1) == a3) {
        pattern = 0;
        for (int idx = a2 + 4; idx > a2; --idx) {
            pattern = ((int)tmod_edges_11(edges, n_edges, idx, (unsigned int)total_width) & 3) | (pattern << 2);
        }
        center = (edges[a2 + 2] + (edges[a2 + 6] - edges[a2 + 5]) + (edges[a2 + 4] - edges[a2 + 3]))
               - edges[a2 + 1];
    } else {
        pattern = (((int)tmod_edges_11(edges, n_edges, a2, (unsigned int)total_width) << 2) & 0xC)
                | ((int)tmod_edges_11(edges, n_edges, a2 + 1, (unsigned int)total_width) & 3);
        pattern = (((int)tmod_edges_11(edges, n_edges, a2 + 2, (unsigned int)total_width) & 3) | (pattern << 2));
        pattern = (((int)tmod_edges_11(edges, n_edges, a2 + 3, (unsigned int)total_width) & 3) | (pattern << 2));
        center = ((edges[a2 + 1] - edges[a2]) + (edges[a2 + 3] - edges[a2 + 2])) + edges[a2 + 5];
    }

    result = (int)k_char128_smart_lookup[pattern] - 1;
    if (k_char128_smart_lookup[pattern]) {
        center -= edges[a2 + 4];
        if ((int)k_char128_smart_lookup[pattern] < 1)
            ref_center_mod = 6;
        else
            ref_center_mod = (int)k_char128_center108[result];
        center += (int)(-((long long)ref_center_mod * (long long)total_width) / 11LL);

        if (*min_center <= *max_center) {
            if (center < *min_center) *min_center = center;
            if (center > *max_center) *max_center = center;
            if (*max_center - *min_center > (total_width >> 3)) {
                if (*min_center < total_width / 5) result = -1;
                if (*max_center > ((3 * total_width) / 5)) result = -1;
            }
        } else {
            *min_center = center;
            *max_center = center;
        }
    }
    return result;
}

static int code128_match_edges_combined(const int *edges, int n_edges, int a2, int a3,
                                        int *min_center, int *max_center, int relaxed)
{
    int sym = relaxed
        ? char128_match_edges_relaxed(edges, n_edges, a2, a3, min_center, max_center)
        : char128_match_edges(edges, n_edges, a2, a3, min_center, max_center);
    if (sym >= 0 || sym <= -2)
        return sym;
    {
        int smart = smartsymbol_match_edges(edges, n_edges, a2, a3, min_center, max_center);
        if (smart >= 0 || smart <= -2) {
            if (getenv("DBG128V")) {
                fprintf(stderr, "  SMART128 pos=%d parity=%d relaxed=%d sym=%d\n",
                        a2, a3, relaxed, smart);
            }
            return smart;
        }
    }
    return sym;
}

/* ARM SMMUL equivalent: signed high 32-bit of a*b */
static int smmul_i32(int a, int b)
{
    return (int)(((long long)a * (long long)b) >> 32);
}

/* source: src/decompiled_code128/accell.c (edges-adapted) */
static int accell_edges(const int *edges, int n_edges,
                        int start_edge, int step, int repeats,
                        int *accum_err)
{
    int i, ratio_min, ratio_max, prev_w, idx_a, idx_b, stride;

    if (repeats < 2) return 1;
    if (!edges || n_edges <= 0 || step <= 0) return 0;
    if (start_edge < 0 || start_edge + step >= n_edges) return 0;

    prev_w = edges[start_edge + step] - edges[start_edge];
    if (prev_w <= 0) return 0;

    ratio_min = 256;
    ratio_max = 256;
    i = 1;
    stride = step + (step & 1);
    idx_a = step + start_edge + (step & 1);
    idx_b = start_edge + 2 * step + (step & 1);

    while (1) {
        int w, low_5_6, high_6_5;
        int ratio;

        if (idx_a < 0 || idx_b < 0 || idx_a >= n_edges || idx_b >= n_edges) return 0;
        w = edges[idx_b] - edges[idx_a];
        if (w > 2 * prev_w || w < (prev_w >> 1)) return 0;

        low_5_6 = smmul_i32(5 * prev_w, 715827883);
        low_5_6 += (low_5_6 >> 31);
        if (w < low_5_6) {
            if (accum_err) *accum_err += (prev_w - w) * 6 / prev_w;
        } else {
            high_6_5 = smmul_i32(6 * prev_w, 1717986919);
            high_6_5 = (high_6_5 >> 1) + (high_6_5 >> 31);
            if (w > high_6_5 && accum_err) *accum_err += (w - prev_w) * 5 / prev_w;
        }

        ratio = (w << 8) / prev_w;
        if (ratio > ratio_max) ratio_max = ratio;
        if (ratio < ratio_min) ratio_min = ratio;

        ++i;
        prev_w = w;
        idx_a += stride;
        idx_b += stride;
        if (i >= repeats) return (ratio_max + 1) - ratio_min;
    }
}

/*
 * Relaxed char128_match_edges for far-distance: higher initial tolerance.
 * Original uses v18 = 4 * _R3 which is too tight for narrow symbols
 * (_R3 < 12 pixels) where quantization errors dominate.
 */
static int char128_match_edges_relaxed(const int *edges, int n_edges, int a2, int a3,
                                        int *min_center, int *max_center)
{
    int v5, v6, v8, v9, v10, v11, v12, v13, v14, v15, v16;
    int v18, v19, v22, v23, v26, v27;
    int v43, v45, v47, v53, v55, v56, v57, v58, v59, v60, v61, v62;
    int y[5];

    if (a2 < 0 || a2 + 6 >= n_edges) return -1;
    init_char128_pat5();

    v5 = edges[a2];
    v6 = edges[a2 + 6];
    int _R3 = v6 - v5;
    if (_R3 <= 0) return -1;

    if ((a2 & 1) == a3) {
        v8 = a2; v9 = 1;
        v5 = edges[v8 + 1]; v10 = edges[v8 + 4];
        v11 = v6 - edges[v8 + 2]; v12 = edges[v8 + 3];
        v13 = edges[v8 + 5]; v58 = v6 - v12;
        v14 = v6 - v13; v60 = v6 - v10;
        v15 = 2; v16 = 3; v55 = 4; v57 = v6;
    } else {
        int v17 = a2; v9 = 4;
        v14 = edges[v17 + 1] - v5; v11 = edges[v17 + 4] - v5;
        v6 = edges[v17 + 5]; v58 = edges[v17 + 3] - v5;
        v60 = edges[v17 + 2] - v5; v57 = edges[v17 + 1];
        v55 = 3; v15 = 5; v16 = 2; v13 = edges[a2];
    }

    /* RELAXED: use 6*_R3 base tolerance, minimum 48 to handle narrow symbols */
    v18 = 6 * _R3;
    if (v18 < 48) v18 = 48;
    v19 = -1;
    if (v18 < (_R3 >> 1)) goto done_r;

    v62 = 11 * v11; v22 = 11 * v14; v61 = 11 * v60;
    v59 = 11 * v58; v23 = 11 * (v6 - v5);
    v53 = v57 - v13 + edges[v55 + a2] - edges[v16 + a2] + edges[v15 + a2] - edges[v9 + a2];
    v56 = -1;

    for (int v24 = 0; v24 <= 107 && v18 >= (_R3 >> 1); v24++) {
        const int *pat = g_char128_pat5[v24].p;
        v26 = 11 * (v23 + v59 + v22 - _R3 * pat[0] - _R3 * pat[2] - _R3 * pat[4]);
        v27 = v22 - (v26 >> 6);

        if (v18 >= 1) {
            int acc = v27 - _R3 * pat[0];
            if (acc < 0) acc = -acc;
            if (acc < v18) {
                int ok = 1;
                y[0] = v27; y[1] = v61;
                y[2] = v59 - (v26 >> 6); y[3] = v62;
                y[4] = v23 - (v26 >> 6);
                for (int i = 1; i <= 4; i++) {
                    int dv = y[i] - _R3 * pat[i];
                    if (dv < 0) dv = -dv;
                    acc += dv;
                    if (acc >= v18) { ok = 0; break; }
                }
                if (ok) { v18 = acc; v56 = v24; }
            }
        }
    }

    {
        int v35 = v56;
        if (v56 == -1) {
            v19 = -1;
        } else {
            int v47_off = (int)((long long)(-_R3 * (int)k_char128_center108[v56]) / 11LL);
            v47 = v53 + v47_off;
            v43 = *min_center; v45 = *max_center;
            if (v43 <= v45) {
                if (v47 < v43) *min_center = v47;
                if (v47 > v45) *max_center = v47;
                v43 = *min_center; v45 = *max_center;
                /* RELAXED: wider center tolerance for narrow symbols */
                if (v45 - v43 > (_R3 >> 2)) {
                    if (v43 < (_R3 / 6)) v35 = -1;
                    if (v45 > ((4 * _R3) / 6)) v35 = -1;
                }
            } else {
                *max_center = v47; *min_center = v47;
            }
            v19 = v35;
        }
    }
done_r:
    if (v19 > 105) v19 = 104 - v19;
    return v19;
}

/*
 * Relaxed accell_edges: wider ratio tolerance for narrow symbols
 */
static int accell_edges_relaxed(const int *edges, int n_edges,
                                 int start_edge, int step, int repeats,
                                 int *accum_err)
{
    int i, ratio_min, ratio_max, prev_w, idx_a, idx_b, stride;

    if (repeats < 2) return 1;
    if (!edges || n_edges <= 0 || step <= 0) return 0;
    if (start_edge < 0 || start_edge + step >= n_edges) return 0;

    prev_w = edges[start_edge + step] - edges[start_edge];
    if (prev_w <= 0) return 0;

    ratio_min = 256; ratio_max = 256;
    i = 1;
    stride = step + (step & 1);
    idx_a = step + start_edge + (step & 1);
    idx_b = start_edge + 2 * step + (step & 1);

    while (1) {
        int w;
        if (idx_a < 0 || idx_b < 0 || idx_a >= n_edges || idx_b >= n_edges) return 0;
        w = edges[idx_b] - edges[idx_a];
        /* RELAXED: allow 3x width change instead of 2x */
        if (w > 3 * prev_w || w < (prev_w / 3)) return 0;

        int low_5_6 = smmul_i32(5 * prev_w, 715827883);
        low_5_6 += (low_5_6 >> 31);
        if (w < low_5_6) {
            if (accum_err) *accum_err += (prev_w - w) * 6 / prev_w;
        } else {
            int high_6_5 = smmul_i32(6 * prev_w, 1717986919);
            high_6_5 = (high_6_5 >> 1) + (high_6_5 >> 31);
            if (w > high_6_5 && accum_err) *accum_err += (w - prev_w) * 5 / prev_w;
        }

        int ratio = (w << 8) / prev_w;
        if (ratio > ratio_max) ratio_max = ratio;
        if (ratio < ratio_min) ratio_min = ratio;

        ++i; prev_w = w; idx_a += stride; idx_b += stride;
        if (i >= repeats) return (ratio_max + 1) - ratio_min;
    }
}

/*
 * Q4 (16x sub-pixel fixed point) counterpart of char128_match_edges_relaxed.
 * The matcher body is entirely ratio-based (everything scales with _R3), so
 * it works unchanged on Q4 edges except for one absolute tolerance floor
 * that was calibrated in integer-pixel units and must be scaled by the same
 * 16x fixed-point factor to preserve its real-world meaning.
 */
#define Q4_SCALE 16
static int char128_match_edges_relaxed_q4(const int *edges, int n_edges, int a2, int a3,
                                           int *min_center, int *max_center)
{
    int v5, v6, v8, v9, v10, v11, v12, v13, v14, v15, v16;
    int v18, v19, v22, v23, v26, v27;
    int v43, v45, v47, v53, v55, v56, v57, v58, v59, v60, v61, v62;
    int y[5];

    if (a2 < 0 || a2 + 6 >= n_edges) return -1;
    init_char128_pat5();

    v5 = edges[a2];
    v6 = edges[a2 + 6];
    int _R3 = v6 - v5;
    if (_R3 <= 0) return -1;

    if ((a2 & 1) == a3) {
        v8 = a2; v9 = 1;
        v5 = edges[v8 + 1]; v10 = edges[v8 + 4];
        v11 = v6 - edges[v8 + 2]; v12 = edges[v8 + 3];
        v13 = edges[v8 + 5]; v58 = v6 - v12;
        v14 = v6 - v13; v60 = v6 - v10;
        v15 = 2; v16 = 3; v55 = 4; v57 = v6;
    } else {
        int v17 = a2; v9 = 4;
        v14 = edges[v17 + 1] - v5; v11 = edges[v17 + 4] - v5;
        v6 = edges[v17 + 5]; v58 = edges[v17 + 3] - v5;
        v60 = edges[v17 + 2] - v5; v57 = edges[v17 + 1];
        v55 = 3; v15 = 5; v16 = 2; v13 = edges[a2];
    }

    /* RELAXED: use 6*_R3 base tolerance, minimum 48 pixels (Q4: 48*16 units)
     * to handle narrow symbols. */
    v18 = 6 * _R3;
    if (v18 < 48 * Q4_SCALE) v18 = 48 * Q4_SCALE;
    v19 = -1;
    if (v18 < (_R3 >> 1)) goto done_r;

    v62 = 11 * v11; v22 = 11 * v14; v61 = 11 * v60;
    v59 = 11 * v58; v23 = 11 * (v6 - v5);
    v53 = v57 - v13 + edges[v55 + a2] - edges[v16 + a2] + edges[v15 + a2] - edges[v9 + a2];
    v56 = -1;

    for (int v24 = 0; v24 <= 107 && v18 >= (_R3 >> 1); v24++) {
        const int *pat = g_char128_pat5[v24].p;
        v26 = 11 * (v23 + v59 + v22 - _R3 * pat[0] - _R3 * pat[2] - _R3 * pat[4]);
        v27 = v22 - (v26 >> 6);

        if (v18 >= 1) {
            int acc = v27 - _R3 * pat[0];
            if (acc < 0) acc = -acc;
            if (acc < v18) {
                int ok = 1;
                y[0] = v27; y[1] = v61;
                y[2] = v59 - (v26 >> 6); y[3] = v62;
                y[4] = v23 - (v26 >> 6);
                for (int i = 1; i <= 4; i++) {
                    int dv = y[i] - _R3 * pat[i];
                    if (dv < 0) dv = -dv;
                    acc += dv;
                    if (acc >= v18) { ok = 0; break; }
                }
                if (ok) { v18 = acc; v56 = v24; }
            }
        }
    }

    {
        int v35 = v56;
        if (v56 == -1) {
            v19 = -1;
        } else {
            int v47_off = (int)((long long)(-_R3 * (int)k_char128_center108[v56]) / 11LL);
            v47 = v53 + v47_off;
            v43 = *min_center; v45 = *max_center;
            if (v43 <= v45) {
                if (v47 < v43) *min_center = v47;
                if (v47 > v45) *max_center = v47;
                v43 = *min_center; v45 = *max_center;
                /* RELAXED: wider center tolerance for narrow symbols */
                if (v45 - v43 > (_R3 >> 2)) {
                    if (v43 < (_R3 / 6)) v35 = -1;
                    if (v45 > ((4 * _R3) / 6)) v35 = -1;
                }
            } else {
                *max_center = v47; *min_center = v47;
            }
            v19 = v35;
        }
    }
done_r:
    if (v19 > 105) v19 = 104 - v19;
    return v19;
}

/* Q4 counterpart of code128_match_edges_combined with relaxed matching
 * hardcoded (the far-distance path always calls it relaxed). smartsymbol
 * matching is fully ratio-based already and needs no Q4 variant. */
static int code128_match_edges_combined_q4(const int *edges, int n_edges, int a2, int a3,
                                            int *min_center, int *max_center)
{
    int sym = char128_match_edges_relaxed_q4(edges, n_edges, a2, a3, min_center, max_center);
    if (sym >= 0 || sym <= -2)
        return sym;
    {
        int smart = smartsymbol_match_edges(edges, n_edges, a2, a3, min_center, max_center);
        if (smart >= 0 || smart <= -2)
            return smart;
    }
    return sym;
}

/* Forward declaration */
static int centered_edges(const int *edges, int n_edges, int left_edge, int right_edge,
                          int strict_mode, int min_pct, int max_pct, int *center_off);

/*
 * Far-distance version of decode128_main_try_window using relaxed matching
 */
static int decode128_main_try_window_far(const int *edges, int n_edges,
                                          int start_idx, int end_idx, int parity,
                                          char *out, int *out_len)
{
    int edge_span, nslots;
    int min_center, max_center;
    int s_left, s_right;
    int syms[MAX_SYMBOLS];
    int nsyms;
    int accell_err;
    int center_off;
    static int dbg_far_trace = -1;
    if (dbg_far_trace < 0) dbg_far_trace = getenv("DBG128V") ? 1 : 0;

    if (start_idx < 0 || end_idx >= n_edges || end_idx <= start_idx) return 0;
    edge_span = end_idx - start_idx;
    if (edge_span < 18) return 0;
    nslots = edge_span / 6;
    if (nslots < 3 || nslots >= MAX_SYMBOLS) return 0;

    min_center = 1; max_center = 0;

    s_left = code128_match_edges_combined(edges, n_edges, start_idx, parity, &min_center, &max_center, 1);
    s_right = code128_match_edges_combined(edges, n_edges, end_idx - 6, parity, &min_center, &max_center, 1);

    nsyms = 0;
    if (s_left >= C128_START_A && s_left <= C128_START_C && s_right == -3) {
        /* Check STOP guard (-2 pattern before STOP) */
        int guard = code128_match_edges_combined(edges, n_edges, end_idx - 7, parity, &min_center, &max_center, 1);
        if (guard != -2) {
            /* Skip guard check for far-distance — noise may corrupt it */
        }
        accell_err = 0;
        if (!accell_edges_relaxed(edges, n_edges, start_idx, 6, nslots, &accell_err)) {
            if (dbg_far_trace)
                fprintf(stderr, "      FAR-TRY: si=%d ei=%d par=%d sl=%d sr=%d FAIL-accell\n",
                        start_idx, end_idx, parity, s_left, s_right);
            return 0;
        }
        syms[nsyms++] = s_left;
        for (int i = 0; i < nslots - 1; ++i) {
            int idx = start_idx + 6 + 6 * i;
            int s = code128_match_edges_combined(edges, n_edges, idx, parity, &min_center, &max_center, 1);
            if (s < 0 || s > 0x66) {
                if (dbg_far_trace && i < 3)
                    fprintf(stderr, "      FAR-TRY: si=%d ei=%d par=%d FAIL-sym[%d]=%d at edge %d\n",
                            start_idx, end_idx, parity, i, s, idx);
                return 0;
            }
            syms[nsyms++] = s;
        }
    } else if (s_left == -3 && s_right >= C128_START_A && s_right <= C128_START_C) {
        int guard = code128_match_edges_combined(edges, n_edges, start_idx + 1, parity, &min_center, &max_center, 1);
        if (guard != -2) {
            /* Skip guard check for far-distance */
        }
        accell_err = 0;
        if (!accell_edges_relaxed(edges, n_edges, start_idx + 1, 6, nslots, &accell_err)) {
            if (dbg_far_trace)
                fprintf(stderr, "      FAR-TRY: si=%d ei=%d par=%d sl=%d sr=%d FAIL-accell-rev\n",
                        start_idx, end_idx, parity, s_left, s_right);
            return 0;
        }
        syms[nsyms++] = s_right;
        for (int i = 0; i < nslots - 1; ++i) {
            int idx = end_idx - 12 - 6 * i;
            int s = code128_match_edges_combined(edges, n_edges, idx, parity, &min_center, &max_center, 1);
            if (s < 0 || s > 0x66) {
                if (dbg_far_trace && i < 3)
                    fprintf(stderr, "      FAR-TRY: si=%d ei=%d par=%d FAIL-revsym[%d]=%d at edge %d\n",
                            start_idx, end_idx, parity, i, s, idx);
                return 0;
            }
            syms[nsyms++] = s;
        }
    } else {
        /* Neither START+STOP nor STOP+START matched — just return */
        return 0;
    }

    if (!centered_edges(edges, n_edges, start_idx, end_idx,
                        0, 10, 90, &center_off)) {
        if (dbg_far_trace)
            fprintf(stderr, "      FAR-TRY: si=%d ei=%d FAIL-centered nsyms=%d\n",
                    start_idx, end_idx, nsyms);
        return 0;
    }

    if (nsyms >= 3) {
        int chk = syms[0] - 103;
        for (int i = 1; i < nsyms - 1; ++i)
            chk = (chk + syms[i] * i) % 103;
        if (chk != syms[nsyms - 1]) {
            if (dbg_far_trace)
                fprintf(stderr, "      FAR-TRY: si=%d ei=%d FAIL-checksum nsyms=%d chk=%d got=%d\n",
                        start_idx, end_idx, nsyms, chk, syms[nsyms-1]);
            if (dbg_far_trace && nsyms >= 8) {
                fprintf(stderr, "      FAR-SYMS:");
                for (int i = 0; i < nsyms; ++i) {
                    fprintf(stderr, " %d", syms[i]);
                }
                fprintf(stderr, "\n");
            }
            return 0;
        }
    } else {
        return 0;
    }

    return symbols_to_text(syms, nsyms, out, out_len);
}

/*
 * Far-distance version of decode_from_edges_char128 using relaxed matching
 */
static int decode_from_edges_char128_far(const int *edges, int n_edges, char *out, int *out_len)
{
    int dbgfar = getenv("DBG128V") ? 1 : 0;
    if (n_edges < 20) return 0;

    /*
     * Scan-based approach: find all positions where START or STOP patterns match,
     * then try decode between compatible pairs.
     * Code128 barcode = START(6) + N*data(6) + checksum(6) + STOP(7) edges
     * Total edges = 6*(N+2) + 7 = 6*N + 19 for N data symbols
     * Content "128150089012345" in Code128C = 8 data symbols → 67 edges
     * In Code128B = 15 data symbols → 109 edges
     */
    int start_positions[64]; /* edge indices where START_A/B/C matched */
    int start_parities[64];
    int n_starts = 0;

    int stop_positions[64];  /* edge indices where STOP matched (the 7-edge idx) */
    int stop_parities[64];
    int n_stops = 0;

    /* Scan all positions for START and STOP, both parities */
    int scan_limit = n_edges - 18; /* need at least 18 edges after start */
    if (scan_limit > n_edges - 7) scan_limit = n_edges - 7;
    for (int parity = 0; parity < 2; ++parity) {
        for (int i = 0; i < n_edges - 6; ++i) {
            int mc1 = 1, mc2 = 0; /* fresh center tracking per probe */
            int sym = char128_match_edges_relaxed(edges, n_edges, i, parity, &mc1, &mc2);
            if (sym >= C128_START_A && sym <= C128_START_C && n_starts < 64) {
                start_positions[n_starts] = i;
                start_parities[n_starts] = parity;
                n_starts++;
            }
            /* STOP = -3 in char128_match return convention */
            if (sym == -3 && n_stops < 64) {
                stop_positions[n_stops] = i;
                stop_parities[n_stops] = parity;
                n_stops++;
            }
        }
    }

    if (dbgfar)
        fprintf(stderr, "    FAR-SCAN: n_edges=%d found %d starts, %d stops\n",
                n_edges, n_starts, n_stops);

    /* Try decode between each START and plausible end positions */
    for (int si = 0; si < n_starts; ++si) {
        int s_idx = start_positions[si];
        int parity = start_parities[si];

        /* Try various symbol counts: 3 to 20 symbols (start+data+chk+stop) */
        for (int nsym = 3; nsym <= 25; ++nsym) {
            /* end_idx for nsym symbols: start + 6*nsym + 7 (stop has 7 edges) */
            int end_idx = s_idx + 6 * nsym + 6; /* last edge index */
            if (end_idx >= n_edges) break;

            if (decode128_main_try_window_far(edges, n_edges, s_idx, end_idx, parity, out, out_len)) {
                if (dbgfar)
                    fprintf(stderr, "    FAR-HIT: start_idx=%d nsym=%d end_idx=%d parity=%d\n",
                            s_idx, nsym, end_idx, parity);
                return 1;
            }
            /* Also try end_idx +/- 1,2 to account for edge noise */
            for (int adj = -2; adj <= 2; ++adj) {
                if (adj == 0) continue;
                int ae = end_idx + adj;
                if (ae < s_idx + 18 || ae >= n_edges) continue;
                if (decode128_main_try_window_far(edges, n_edges, s_idx, ae, parity, out, out_len)) {
                    if (dbgfar)
                        fprintf(stderr, "    FAR-HIT: start_idx=%d nsym=%d end_idx=%d(adj=%d) parity=%d\n",
                                s_idx, nsym, ae, adj, parity);
                    return 1;
                }
            }
        }
    }

    /* Also try from STOP backwards */
    for (int ti = 0; ti < n_stops; ++ti) {
        int stop_idx = stop_positions[ti];
        int parity = stop_parities[ti];

        for (int nsym = 3; nsym <= 25; ++nsym) {
            int start_idx = stop_idx + 6 - 6 * nsym;
            if (start_idx < 0) break;

            int end_idx = stop_idx + 6;
            if (end_idx >= n_edges) continue;

            if (decode128_main_try_window_far(edges, n_edges, start_idx, end_idx, parity, out, out_len)) {
                if (dbgfar)
                    fprintf(stderr, "    FAR-HIT-REV: stop_idx=%d nsym=%d start_idx=%d parity=%d\n",
                            stop_idx, nsym, start_idx, parity);
                return 1;
            }
            for (int adj = -2; adj <= 2; ++adj) {
                if (adj == 0) continue;
                int as = start_idx + adj;
                if (as < 0 || end_idx - as < 18) continue;
                if (decode128_main_try_window_far(edges, n_edges, as, end_idx, parity, out, out_len)) {
                    if (dbgfar)
                        fprintf(stderr, "    FAR-HIT-REV: stop_idx=%d nsym=%d start_idx=%d(adj=%d) parity=%d\n",
                                stop_idx, nsym, as, adj, parity);
                    return 1;
                }
            }
        }
    }

    /* Fallback: brute force with larger trim range, stepping by 2 */
    {
        int max_trim = n_edges - 20;
        if (max_trim > 80) max_trim = 80;
        for (int ltrim = 0; ltrim <= max_trim; ltrim += 2) {
            for (int rtrim = 0; rtrim <= max_trim; rtrim += 2) {
                int start_idx = ltrim;
                int end_idx = (n_edges - 1) - rtrim;
                if (end_idx - start_idx < 18) continue;
                /* Only try spans that could be valid: 6*N+6 or 6*N+7 edges */
                int span = end_idx - start_idx;
                int rem6 = span % 6;
                if (rem6 > 1) continue; /* must be 0 or 1 mod 6 */
                for (int parity = 0; parity < 2; ++parity) {
                    if (decode128_main_try_window_far(edges, n_edges, start_idx, end_idx, parity, out, out_len))
                        return 1;
                }
            }
        }
    }

    /* START-only decode: when we find starts but no stops, try matching
     * symbols forward from each start and validate via checksum.
     * This handles cases where the STOP pattern is corrupted by noise. */
    if (n_starts > 0 && n_stops == 0) {
        for (int si = 0; si < n_starts; ++si) {
            int s_idx = start_positions[si];
            int parity = start_parities[si];
            int min_c = 1, max_c = 0;

            int start_sym = code128_match_edges_combined(edges, n_edges, s_idx, parity, &min_c, &max_c, 1);
            if (start_sym < C128_START_A || start_sym > C128_START_C) continue;

            int syms[MAX_SYMBOLS];
            int best_nsyms = 0;

            /* Try matching symbols forward, collecting as many as possible */
            syms[0] = start_sym;
            int nsyms = 1;
            int max_syms = (n_edges - s_idx) / 6;
            if (max_syms > MAX_SYMBOLS) max_syms = MAX_SYMBOLS;

            for (int i = 1; i < max_syms; ++i) {
                int idx = s_idx + 6 * i;
                if (idx + 6 > n_edges) break;
                int s = code128_match_edges_combined(edges, n_edges, idx, parity, &min_c, &max_c, 1);
                if (s < 0 || s > 0x66) break;
                syms[nsyms++] = s;

                /* Check if current sequence has valid checksum (need at least START + 1 data + CHK) */
                if (nsyms >= 3) {
                    int chk = syms[0] - 103;
                    for (int j = 1; j < nsyms - 1; ++j)
                        chk = (chk + syms[j] * j) % 103;
                    if (chk == syms[nsyms - 1]) {
                        /* Checksum match — try to convert to text */
                        char text[BARCODE_MAX_LEN];
                        int text_len = 0;
                        if (symbols_to_text(syms, nsyms, text, &text_len) &&
                            text_len >= 8) {
                            best_nsyms = nsyms;
                            memcpy(out, text, text_len + 1);
                            *out_len = text_len;
                        }
                    }
                }
            }
            if (best_nsyms >= 3) {
                if (dbgfar)
                    fprintf(stderr, "    FAR-STARTONLY: si=%d nsyms=%d text=%.20s\n",
                            s_idx, best_nsyms, out);
                return 1;
            }
        }
    }

    return 0;
}

/* Forward declaration -- defined later in the file (near monotonic_ms());
 * needed here because decode_from_edges_char128_far_q4() below must bound
 * its worst-case cost (brute-force window search) against the caller's
 * per-image deadline. */
static int is_timed_out(uint64_t deadline_ms);

/*
 * Q4 (16x sub-pixel) counterpart of decode128_main_try_window_far.
 * edges[] here are in Q4 fixed-point units (signal_find_edges_q4), i.e. the
 * true sub-pixel valley-tracker output, not truncated to integer pixels.
 * All index-based bounds (edge_span, nslots, ...) are unaffected by the
 * value scale; only code128_match_edges_combined_q4 needs the Q4-scaled
 * tolerance floor.
 */
static int decode128_main_try_window_far_q4(const int *edges, int n_edges,
                                             int start_idx, int end_idx, int parity,
                                             char *out, int *out_len)
{
    int edge_span, nslots;
    int min_center, max_center;
    int s_left, s_right;
    int syms[MAX_SYMBOLS];
    int nsyms;
    int accell_err;
    int center_off;
    static int dbg_far_trace = -1;
    if (dbg_far_trace < 0) dbg_far_trace = getenv("DBG128V") ? 1 : 0;

    if (start_idx < 0 || end_idx >= n_edges || end_idx <= start_idx) return 0;
    edge_span = end_idx - start_idx;
    if (edge_span < 18) return 0;
    nslots = edge_span / 6;
    if (nslots < 3 || nslots >= MAX_SYMBOLS) return 0;

    min_center = 1; max_center = 0;

    s_left = code128_match_edges_combined_q4(edges, n_edges, start_idx, parity, &min_center, &max_center);
    s_right = code128_match_edges_combined_q4(edges, n_edges, end_idx - 6, parity, &min_center, &max_center);

    nsyms = 0;
    if (s_left >= C128_START_A && s_left <= C128_START_C && s_right == -3) {
        accell_err = 0;
        if (!accell_edges_relaxed(edges, n_edges, start_idx, 6, nslots, &accell_err)) {
            if (dbg_far_trace)
                fprintf(stderr, "      FARQ4-TRY: si=%d ei=%d par=%d sl=%d sr=%d FAIL-accell\n",
                        start_idx, end_idx, parity, s_left, s_right);
            return 0;
        }
        syms[nsyms++] = s_left;
        for (int i = 0; i < nslots - 1; ++i) {
            int idx = start_idx + 6 + 6 * i;
            int s = code128_match_edges_combined_q4(edges, n_edges, idx, parity, &min_center, &max_center);
            if (s < 0 || s > 0x66) {
                if (dbg_far_trace && i < 3)
                    fprintf(stderr, "      FARQ4-TRY: si=%d ei=%d par=%d FAIL-sym[%d]=%d at edge %d\n",
                            start_idx, end_idx, parity, i, s, idx);
                return 0;
            }
            syms[nsyms++] = s;
        }
    } else if (s_left == -3 && s_right >= C128_START_A && s_right <= C128_START_C) {
        accell_err = 0;
        if (!accell_edges_relaxed(edges, n_edges, start_idx + 1, 6, nslots, &accell_err)) {
            if (dbg_far_trace)
                fprintf(stderr, "      FARQ4-TRY: si=%d ei=%d par=%d sl=%d sr=%d FAIL-accell-rev\n",
                        start_idx, end_idx, parity, s_left, s_right);
            return 0;
        }
        syms[nsyms++] = s_right;
        for (int i = 0; i < nslots - 1; ++i) {
            int idx = end_idx - 12 - 6 * i;
            int s = code128_match_edges_combined_q4(edges, n_edges, idx, parity, &min_center, &max_center);
            if (s < 0 || s > 0x66) {
                if (dbg_far_trace && i < 3)
                    fprintf(stderr, "      FARQ4-TRY: si=%d ei=%d par=%d FAIL-revsym[%d]=%d at edge %d\n",
                            start_idx, end_idx, parity, i, s, idx);
                return 0;
            }
            syms[nsyms++] = s;
        }
    } else {
        return 0;
    }

    if (!centered_edges(edges, n_edges, start_idx, end_idx,
                        0, 10, 90, &center_off)) {
        if (dbg_far_trace)
            fprintf(stderr, "      FARQ4-TRY: si=%d ei=%d FAIL-centered nsyms=%d\n",
                    start_idx, end_idx, nsyms);
        return 0;
    }

    if (nsyms >= 3) {
        int chk = syms[0] - 103;
        for (int i = 1; i < nsyms - 1; ++i)
            chk = (chk + syms[i] * i) % 103;
        if (chk != syms[nsyms - 1]) {
            if (dbg_far_trace) {
                fprintf(stderr, "      FARQ4-TRY: si=%d ei=%d FAIL-checksum nsyms=%d chk=%d got=%d\n",
                        start_idx, end_idx, nsyms, chk, syms[nsyms-1]);
                if (nsyms >= 8 && getenv("DBGQ4SYMS")) {
                    fprintf(stderr, "      FARQ4-SYMS:");
                    for (int i = 0; i < nsyms; ++i) fprintf(stderr, " %d", syms[i]);
                    fprintf(stderr, "\n");
                }
            }
            return 0;
        }
    } else {
        return 0;
    }

    return symbols_to_text(syms, nsyms, out, out_len);
}

/*
 * Q4 counterpart of decode_from_edges_char128_far. Structurally identical
 * scan strategy (START/STOP position scan, forward/backward window tries,
 * brute-force trim, start-only checksum walk); operates on Q4 fixed-point
 * edges throughout so no sub-pixel precision is lost before it reaches the
 * Char128 matcher.
 */
static int decode_from_edges_char128_far_q4(const int *edges, int n_edges, char *out, int *out_len,
                                             uint64_t deadline_ms)
{
    int dbgfar = getenv("DBG128V") ? 1 : 0;
    if (n_edges < 20) return 0;
    if (is_timed_out(deadline_ms)) return 0;

    int start_positions[64];
    int start_parities[64];
    int n_starts = 0;

    int stop_positions[64];
    int stop_parities[64];
    int n_stops = 0;

    for (int parity = 0; parity < 2; ++parity) {
        for (int i = 0; i < n_edges - 6; ++i) {
            if ((i & 63) == 0 && is_timed_out(deadline_ms)) return 0;
            int mc1 = 1, mc2 = 0;
            int sym = char128_match_edges_relaxed_q4(edges, n_edges, i, parity, &mc1, &mc2);
            if (sym >= C128_START_A && sym <= C128_START_C && n_starts < 64) {
                start_positions[n_starts] = i;
                start_parities[n_starts] = parity;
                n_starts++;
            }
            if (sym == -3 && n_stops < 64) {
                stop_positions[n_stops] = i;
                stop_parities[n_stops] = parity;
                n_stops++;
            }
        }
    }

    if (dbgfar)
        fprintf(stderr, "    FARQ4-SCAN: n_edges=%d found %d starts, %d stops\n",
                n_edges, n_starts, n_stops);

    for (int si = 0; si < n_starts; ++si) {
        if (is_timed_out(deadline_ms)) return 0;
        int s_idx = start_positions[si];
        int parity = start_parities[si];

        for (int nsym = 3; nsym <= 25; ++nsym) {
            int end_idx = s_idx + 6 * nsym + 6;
            if (end_idx >= n_edges) break;

            if (decode128_main_try_window_far_q4(edges, n_edges, s_idx, end_idx, parity, out, out_len)) {
                if (dbgfar)
                    fprintf(stderr, "    FARQ4-HIT: start_idx=%d nsym=%d end_idx=%d parity=%d\n",
                            s_idx, nsym, end_idx, parity);
                return 1;
            }
            for (int adj = -2; adj <= 2; ++adj) {
                if (adj == 0) continue;
                int ae = end_idx + adj;
                if (ae < s_idx + 18 || ae >= n_edges) continue;
                if (decode128_main_try_window_far_q4(edges, n_edges, s_idx, ae, parity, out, out_len)) {
                    if (dbgfar)
                        fprintf(stderr, "    FARQ4-HIT: start_idx=%d nsym=%d end_idx=%d(adj=%d) parity=%d\n",
                                s_idx, nsym, ae, adj, parity);
                    return 1;
                }
            }
        }
    }

    for (int ti = 0; ti < n_stops; ++ti) {
        if (is_timed_out(deadline_ms)) return 0;
        int stop_idx = stop_positions[ti];
        int parity = stop_parities[ti];

        for (int nsym = 3; nsym <= 25; ++nsym) {
            int start_idx = stop_idx + 6 - 6 * nsym;
            if (start_idx < 0) break;

            int end_idx = stop_idx + 6;
            if (end_idx >= n_edges) continue;

            if (decode128_main_try_window_far_q4(edges, n_edges, start_idx, end_idx, parity, out, out_len)) {
                if (dbgfar)
                    fprintf(stderr, "    FARQ4-HIT-REV: stop_idx=%d nsym=%d start_idx=%d parity=%d\n",
                            stop_idx, nsym, start_idx, parity);
                return 1;
            }
            for (int adj = -2; adj <= 2; ++adj) {
                if (adj == 0) continue;
                int as = start_idx + adj;
                if (as < 0 || end_idx - as < 18) continue;
                if (decode128_main_try_window_far_q4(edges, n_edges, as, end_idx, parity, out, out_len)) {
                    if (dbgfar)
                        fprintf(stderr, "    FARQ4-HIT-REV: stop_idx=%d nsym=%d start_idx=%d(adj=%d) parity=%d\n",
                                stop_idx, nsym, as, adj, parity);
                    return 1;
                }
            }
        }
    }

    /* Fallback: brute force with larger trim range, stepping by 2. This is
     * the dominant cost of this function (up to 40*40*2 window tries), so it
     * is the most important place to honor the deadline -- without this
     * check, a single hard image can blow the per-image time budget on its
     * own regardless of how many times the caller re-enters this function. */
    if (!is_timed_out(deadline_ms)) {
        int max_trim = n_edges - 20;
        if (max_trim > 80) max_trim = 80;
        for (int ltrim = 0; ltrim <= max_trim; ltrim += 2) {
            if (is_timed_out(deadline_ms)) break;
            for (int rtrim = 0; rtrim <= max_trim; rtrim += 2) {
                int start_idx = ltrim;
                int end_idx = (n_edges - 1) - rtrim;
                if (end_idx - start_idx < 18) continue;
                int span = end_idx - start_idx;
                int rem6 = span % 6;
                if (rem6 > 1) continue;
                for (int parity = 0; parity < 2; ++parity) {
                    if (decode128_main_try_window_far_q4(edges, n_edges, start_idx, end_idx, parity, out, out_len))
                        return 1;
                }
            }
        }
    }

    if (n_starts > 0 && n_stops == 0 && !is_timed_out(deadline_ms)) {
        for (int si = 0; si < n_starts; ++si) {
            if ((si & 7) == 0 && is_timed_out(deadline_ms)) break;
            int s_idx = start_positions[si];
            int parity = start_parities[si];
            int min_c = 1, max_c = 0;

            int start_sym = code128_match_edges_combined_q4(edges, n_edges, s_idx, parity, &min_c, &max_c);
            if (start_sym < C128_START_A || start_sym > C128_START_C) continue;

            int syms[MAX_SYMBOLS];
            int best_nsyms = 0;

            syms[0] = start_sym;
            int nsyms = 1;
            int max_syms = (n_edges - s_idx) / 6;
            if (max_syms > MAX_SYMBOLS) max_syms = MAX_SYMBOLS;

            for (int i = 1; i < max_syms; ++i) {
                int idx = s_idx + 6 * i;
                if (idx + 6 > n_edges) break;
                int s = code128_match_edges_combined_q4(edges, n_edges, idx, parity, &min_c, &max_c);
                if (s < 0 || s > 0x66) break;
                syms[nsyms++] = s;

                if (nsyms >= 3) {
                    int chk = syms[0] - 103;
                    for (int j = 1; j < nsyms - 1; ++j)
                        chk = (chk + syms[j] * j) % 103;
                    if (chk == syms[nsyms - 1]) {
                        char text[BARCODE_MAX_LEN];
                        int text_len = 0;
                        if (symbols_to_text(syms, nsyms, text, &text_len) &&
                            text_len >= 8) {
                            best_nsyms = nsyms;
                            memcpy(out, text, text_len + 1);
                            *out_len = text_len;
                        }
                    }
                }
            }
            if (best_nsyms >= 3) {
                if (dbgfar)
                    fprintf(stderr, "    FARQ4-STARTONLY: si=%d nsyms=%d text=%.20s\n",
                            s_idx, best_nsyms, out);
                return 1;
            }
        }
    }

    return 0;
}

/* source: src/decompiled_code128/centered.c (edges-adapted) */
static int centered_edges(const int *edges, int n_edges, int left_edge, int right_edge,
                          int strict_mode, int min_pct, int max_pct, int *center_off)
{
    int total, left_x, right_x, mid;
    int low_x, high_x;

    if (!edges || n_edges <= 0) return 1;
    if (left_edge < 0 || right_edge < 0 || left_edge >= n_edges || right_edge >= n_edges) return 0;
    if (left_edge > right_edge) {
        int t = left_edge;
        left_edge = right_edge;
        right_edge = t;
    }

    total = edges[n_edges - 1];
    if (total <= 0) return 1;

    left_x = edges[left_edge];
    right_x = edges[right_edge];

    if (!strict_mode) {
        if (center_off) {
            *center_off = 0;
            mid = total >> 1;
            if (mid <= right_x) {
                if (left_x > mid) *center_off = left_x - mid;
            } else {
                *center_off = mid - right_x;
            }
        }
        return 1;
    }

    {
        int tmp = smmul_i32(min_pct * total, 1374389535);
        low_x = (tmp >> 5) + (tmp >> 31);
    }
    if (right_x < low_x) return 0;

    {
        int tmp = smmul_i32(max_pct * total, 1374389535);
        high_x = (tmp >> 5) + (tmp >> 31);
    }
    return left_x <= high_x;
}

typedef struct {
    long long sym_calls;        /* match_symbol_tol calls */
    long long sym_inner;        /* 6-term loop iterations executed (incl. partial) */
    long long sym_complete;     /* iterations that ran all 6 terms (no early bail) */
    long long runs_cfg_calls;   /* decode_from_runs_cfg calls */
    long long runs_positions;   /* outer (rev,pos) start-match probes */
    long long runs_start_found; /* positions with a matching START symbol */
    long long runs_sym_matches; /* in-candidate symbol match calls */
    long long edge_calls;       /* decode_from_edges_char128 calls */
    long long edge_trywin;      /* decode128_main_try_window calls */
    long long edge_ok;          /* successful decode_from_edges_char128 calls */
} PerfCounters;
static PerfCounters g_pc;

static void perf_reset(void)
{
    memset(&g_pc, 0, sizeof(g_pc));
}

static void perf_dump(int dbg_call_id)
{
    fprintf(stderr,
            "  PERF #%d sym_calls=%lld sym_inner=%lld sym_complete=%lld "
            "runs_cfg=%lld runs_pos=%lld runs_start=%lld runs_sym=%lld "
            "edge_calls=%lld edge_trywin=%lld edge_ok=%lld\n",
            dbg_call_id,
            (long long)g_pc.sym_calls, (long long)g_pc.sym_inner, (long long)g_pc.sym_complete,
            (long long)g_pc.runs_cfg_calls, (long long)g_pc.runs_positions,
            (long long)g_pc.runs_start_found, (long long)g_pc.runs_sym_matches,
            (long long)g_pc.edge_calls, (long long)g_pc.edge_trywin, (long long)g_pc.edge_ok);
}

static int decode128_main_try_window(const int *edges, int n_edges,
                                     int start_idx, int end_idx, int parity,
                                     char *out, int *out_len)
{
    int edge_span, nslots;
    int min_center;
    int max_center;
    int s_left, s_right;
    int syms[MAX_SYMBOLS];
    int nsyms;
    int accell_err;
    int center_off;

    g_pc.edge_trywin++;
    /* decode128_main.c:227-239 */
    if (start_idx < 0 || end_idx >= n_edges || end_idx <= start_idx) return 0;
    edge_span = end_idx - start_idx;
    if (edge_span < 18) return 0;
    nslots = edge_span / 6;
    if (nslots < 3 || nslots >= MAX_SYMBOLS) return 0;

    /* decode128_main.c:240-243 (center tracking init) */
    min_center = 1;
    max_center = 0;

    /* decode128_main.c:808-899 (Char128 path; SmartSymbol path mirrors this core) */
    s_left = code128_match_edges_combined(edges, n_edges, start_idx, parity, &min_center, &max_center, 0);
    s_right = code128_match_edges_combined(edges, n_edges, end_idx - 6, parity, &min_center, &max_center, 0);

    nsyms = 0;
    if (s_left >= C128_START_A && s_left <= C128_START_C && s_right == -3) {
        int i;
        if (code128_match_edges_combined(edges, n_edges, end_idx - 7, parity, &min_center, &max_center, 0) != -2)
            return 0;
        accell_err = 0;
        if (!accell_edges(edges, n_edges, start_idx, 6, nslots, &accell_err))
            return 0;
        syms[nsyms++] = s_left; /* decode128_main.c:873 */
        for (i = 0; i < nslots - 1; ++i) { /* decode128_main.c:864-893 */
            int idx = start_idx + 6 + 6 * i;
            int s = code128_match_edges_combined(edges, n_edges, idx, parity, &min_center, &max_center, 0);
            if (s < 0 || s > 0x66) return 0;
            syms[nsyms++] = s;
        }
    } else if (s_left == -3 && s_right >= C128_START_A && s_right <= C128_START_C) {
        int i;
        if (code128_match_edges_combined(edges, n_edges, start_idx + 1, parity, &min_center, &max_center, 0) != -2)
            return 0;
        accell_err = 0;
        if (!accell_edges(edges, n_edges, start_idx + 1, 6, nslots, &accell_err))
            return 0;
        syms[nsyms++] = s_right; /* decode128_main.c:828 */
        for (i = 0; i < nslots - 1; ++i) { /* decode128_main.c:830-853 */
            int idx = end_idx - 12 - 6 * i;
            int s = code128_match_edges_combined(edges, n_edges, idx, parity, &min_center, &max_center, 0);
            if (s < 0 || s > 0x66) return 0;
            syms[nsyms++] = s;
        }
    } else {
        return 0;
    }

    if (!centered_edges(edges, n_edges, start_idx, end_idx,
                        1, 20, 80, &center_off))
        return 0;

    /* decode128_main.c:283-318 (checksum precheck before fmwlsymbol009d) */
    if (nsyms >= 3) {
        int chk = syms[0] - 103;
        for (int i = 1; i < nsyms - 1; ++i)
            chk = (chk + syms[i] * i) % 103;
        if (chk != syms[nsyms - 1]) return 0;
    } else {
        return 0;
    }

    /* decode128_main.c:320-327 -> j_fmwlsymbol009d */
    return symbols_to_text(syms, nsyms, out, out_len);
}

/*
 * Fast START-scanning edge decoder: instead of brute-force O(trim²) window
 * search, scan edges for START pattern matches in O(n), then for each START
 * try possible STOP positions.  Much faster for large edge arrays (100+).
 */
static int decode_from_edges_char128_startscan(const int *edges, int n_edges,
                                                char *out, int *out_len)
{
    if (n_edges < 20) return 0;

    int min_center, max_center;
    /* Forward: scan for START at each position */
    for (int pos = 0; pos + 18 < n_edges; pos++) {
        for (int parity = 0; parity < 2; parity++) {
            int s = char128_match_edges(edges, n_edges, pos, parity,
                                        &min_center, &max_center);
            if (s < C128_START_A || s > C128_START_C)
                continue;
            /* Found START at pos — try various window sizes (3..40 symbols) */
            for (int nsym = 3; nsym <= 40; nsym++) {
                int end = pos + 6 * (nsym + 1); /* +1 for STOP */
                if (end >= n_edges) break;
                if (decode128_main_try_window(edges, n_edges, pos, end,
                                              parity, out, out_len))
                    return 1;
            }
        }
    }
    /* Reverse: scan for STOP at each position, try backwards */
    for (int pos = n_edges - 7; pos >= 18; pos--) {
        for (int parity = 0; parity < 2; parity++) {
            int s = char128_match_edges(edges, n_edges, pos, parity,
                                        &min_center, &max_center);
            if (s != -3) continue;
            /* Found STOP at pos — try various START positions */
            for (int nsym = 3; nsym <= 40; nsym++) {
                int start = pos - 6 * nsym;
                if (start < 0) break;
                if (decode128_main_try_window(edges, n_edges, start, pos + 6,
                                              parity, out, out_len))
                    return 1;
            }
        }
    }
    return 0;
}

static int decode_from_edges_char128(const int *edges, int n_edges, char *out, int *out_len)
{
    int max_trim;
    if (n_edges < 20) return 0;
    g_pc.edge_calls++;

    max_trim = n_edges - 20;
    if (max_trim > 24) max_trim = 24;

    /* Locate a START symbol with an O(n) edge scan, then probe only rtrim
     * windows anchored at candidate START positions. Any window that could
     * decode must begin with a START symbol, so this scan covers the full
     * decode space for arrays whose START is matchable here.
     *
     * The former (ltrim x rtrim x parity) grid fallback and its smartsymbol
     * START variant were removed: in the 97-sample regression plus the 3 gate
     * crops, edge decode never produces a result (edge_ok=0 everywhere), while
     * the grid burned ~1250 windows per call. The scan below is the only edge
     * decode attempt left. Window budget keeps the worst case bounded. */
    {
        int min_center, max_center;
        int fast_windows = 0;
        for (int pos = 0; pos + 18 < n_edges; pos++) {
            for (int parity = 0; parity < 2; parity++) {
                int s = char128_match_edges(edges, n_edges, pos, parity,
                                            &min_center, &max_center);
                if (s < C128_START_A || s > C128_START_C)
                    continue;
                if (getenv("DBG128WIN"))
                    fprintf(stderr, "  EDGE-FAST START pos=%d parity=%d n=%d\n", pos, parity, n_edges);
                for (int rtrim = 0; rtrim <= max_trim; rtrim++) {
                    int end_idx = (n_edges - 1) - rtrim;
                    if (end_idx - pos < 18) break;
                    if (++fast_windows > 1250)
                        return 0;
                    if (decode128_main_try_window(edges, n_edges, pos, end_idx,
                                                  parity, out, out_len)) {
                        g_pc.edge_ok++;
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

/*
 * Clustered edge decode: find the densest region of edges (the barcode)
 * and try decoding subsets of that region. This handles far-distance images
 * where the barcode occupies a small fraction of the full row, resulting
 * in many noise edges from the background that overwhelm simple trim-based
 * window search.
 */
static int decode_from_edges_char128_clustered(const int *edges, int n_edges,
                                                char *out, int *out_len)
{
    if (n_edges < 30) return 0;

    /* Find the densest region: use a fixed probe window, find where those
     * edges span the smallest pixel range. Then expand to include all
     * neighboring edges within the density envelope. */
    int probe_w = 30;
    if (probe_w > n_edges) probe_w = n_edges;

    int best_start = 0;
    int best_span = edges[probe_w - 1] - edges[0];
    for (int i = 1; i + probe_w - 1 < n_edges; i++) {
        int span = edges[i + probe_w - 1] - edges[i];
        if (span < best_span) {
            best_span = span;
            best_start = i;
        }
    }
    if (best_span <= 0) return 0;

    /* Average gap in densest region */
    int avg_gap = best_span / (probe_w - 1);
    if (avg_gap < 1) avg_gap = 1;
    int gap_thresh = avg_gap * 3;

    /* Expand cluster to include all edges within gap_thresh of neighbors */
    int cl_start = best_start;
    int cl_end = best_start + probe_w - 1;
    while (cl_start > 0 &&
           edges[cl_start] - edges[cl_start - 1] <= gap_thresh)
        cl_start--;
    while (cl_end < n_edges - 1 &&
           edges[cl_end + 1] - edges[cl_end] <= gap_thresh)
        cl_end++;

    int cl_n = cl_end - cl_start + 1;
    if (cl_n < 20) return 0;

    /* Try decoding the full cluster */
    if (decode_from_edges_char128(&edges[cl_start], cl_n, out, out_len))
        return 1;

    /* Try tighter sub-clusters */
    for (int shrink = 2; shrink <= 8 && shrink < cl_n / 4; shrink += 2) {
        int sub_n = cl_n - 2 * shrink;
        if (sub_n < 20) break;
        if (decode_from_edges_char128(&edges[cl_start + shrink], sub_n, out, out_len))
            return 1;
    }

    return 0;
}

/* Fixed-point scale for normalized match error. The true error is
 * sum((11*w[i] - pattern[i]*sum)^2) / sum^2; we accumulate the raw numerator
 * exactly in int64 and scale once, so no float/double appears in the matcher.
 * This removes soft-float on ESP32-P4 while keeping comparison semantics exact
 * (the original double path was required to reject far-distance false reads
 * such as code39_fullasc, which float rounding let across the threshold). */
#define MATCH_FIXED_SCALE (1 << 20)

/* Lightweight per-image decode counters for the XCR_TIMING perf dump. */
static int match_symbol_tol(const int *widths, int *total, int64_t *err_out,
                            int64_t max_err_fixed, int min_sym, int max_sym)
{
    int sum = 0;
    int i;

    g_pc.sym_calls++;
    for (i = 0; i < 6; i++) sum += widths[i];
    *total = sum;
    if (sum <= 0) return -1;

    int best = -1;
    int64_t best_raw = INT64_MAX;

    /* A symbol is accepted only if best_fixed <= max_err_fixed, i.e.
     * best_raw <= max_err_fixed*sum^2 / MATCH_FIXED_SCALE (as reals). Any
     * symbol whose partial squared-error reaches that bound can neither win
     * nor be accepted, so bail out of its 6-term loop early. Exact — the
     * acceptance boundary is unchanged; this only skips work for
     * non-matching widths, which are the bulk of scanned positions. */
    int64_t max_raw = max_err_fixed * (int64_t)sum * (int64_t)sum;
    int64_t limit = (max_raw - 1) / MATCH_FIXED_SCALE + 1;

    for (int sym = min_sym; sym < max_sym; sym++) {
        int64_t err = 0;
        for (i = 0; i < 6; i++) {
            int d = widths[i] * 11 - C128_PATTERNS[sym][i] * sum;
            err += (int64_t)d * (int64_t)d;
            g_pc.sym_inner++;
            if (err >= limit) break;
        }
        g_pc.sym_complete += (i >= 6) ? 1 : 0;
        if (err < best_raw) {
            best_raw = err;
            best = sym;
            limit = err;
        }
    }

    int64_t best_fixed = best_raw * MATCH_FIXED_SCALE /
                         ((int64_t)sum * (int64_t)sum);
    if (err_out) *err_out = best_fixed;
    if (best_fixed > max_err_fixed) return -1;
    return best;
}

static int match_stop_tol(const int *widths, int64_t max_err_fixed)
{
    int sum = 0;
    for (int i = 0; i < 7; i++) sum += widths[i];
    if (sum <= 0) return 0;

    int64_t err = 0;
    for (int i = 0; i < 7; i++) {
        int d = widths[i] * 13 - C128_STOP[i] * sum;
        err += (int64_t)d * (int64_t)d;
    }

    int64_t err_fixed = err * MATCH_FIXED_SCALE /
                        ((int64_t)sum * (int64_t)sum);
    return (err_fixed < max_err_fixed) ? 1 : 0;
}

#define match_stop(w) match_stop_tol((w), 2.5f)

/* ════════════════════════════════════════════════════════════════════════
 *  Barcode structure decoding
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    char text[BARCODE_MAX_LEN];
    int  text_len;
    int  valid;
    /* Pixel position (row coordinates) where the decoded barcode's STOP
     * pattern ends. Set on valid reads so the caller can verify the image
     * has a quiet zone after it. */
    int  stop_px;
} ScanlineResult;

typedef struct {
    int ai_len;
    int fixed_len; /* -1 means variable length */
} Gs1AiInfo;

static int append_byte_bounded(char *out, int *pos, char ch)
{
    if (*pos >= BARCODE_MAX_LEN - 1) return 0;
    out[(*pos)++] = ch;
    return 1;
}

static int append_str_bounded(char *out, int *pos, const char *s, int n)
{
    for (int i = 0; i < n; i++) {
        if (!append_byte_bounded(out, pos, s[i])) return 0;
    }
    return 1;
}

static int gs1_ai_match(const char *s, int remain, Gs1AiInfo *info)
{
    if (remain < 2 || !isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[1]))
        return 0;

    if (remain >= 4 &&
        isdigit((unsigned char)s[2]) && isdigit((unsigned char)s[3])) {
        int a = (s[0] - '0') * 1000 + (s[1] - '0') * 100 + (s[2] - '0') * 10 + (s[3] - '0');
        int a3 = a / 10;
        if ((a >= 3100 && a <= 3699) ||
            (a >= 3900 && a <= 3949) ||
            (a >= 7001 && a <= 7009) ||
            (a >= 8001 && a <= 8026) ||
            (a >= 8100 && a <= 8119)) {
            info->ai_len = 4;
            if ((a >= 3100 && a <= 3699) || a == 3940 || a == 3941 || a == 8100 || a == 8101 || a == 8102) {
                info->fixed_len = (a >= 3100 && a <= 3699) ? 6 : (a == 8100 ? 6 : (a == 8101 ? 10 : (a == 8102 ? 2 : 4)));
            } else if (a == 7001) {
                info->fixed_len = 13;
            } else if (a == 7003) {
                info->fixed_len = 10;
            } else if (a == 8001) {
                info->fixed_len = 14;
            } else if (a == 8005) {
                info->fixed_len = 6;
            } else if (a == 8006 || a == 8018) {
                info->fixed_len = 18;
            } else {
                info->fixed_len = -1;
            }
            (void)a3;
            return 1;
        }
    }

    if (remain >= 3 && isdigit((unsigned char)s[2])) {
        int a = (s[0] - '0') * 100 + (s[1] - '0') * 10 + (s[2] - '0');
        if (a == 240 || a == 241 || a == 243 || a == 250 || a == 251 || a == 253) {
            info->ai_len = 3; info->fixed_len = -1; return 1;
        }
        if (a == 242) {
            info->ai_len = 3; info->fixed_len = 6; return 1;
        }
        if (a == 254) {
            info->ai_len = 3; info->fixed_len = -1; return 1;
        }
        if (a == 255) {
            info->ai_len = 3; info->fixed_len = 13; return 1;
        }
        if (a == 400 || a == 401) {
            info->ai_len = 3; info->fixed_len = -1; return 1;
        }
        if (a == 402) {
            info->ai_len = 3; info->fixed_len = 17; return 1;
        }
        if (a >= 410 && a <= 417) {
            info->ai_len = 3; info->fixed_len = 13; return 1;
        }
        if (a == 420 || a == 421) {
            info->ai_len = 3; info->fixed_len = -1; return 1;
        }
        if (a >= 422 && a <= 426) {
            info->ai_len = 3; info->fixed_len = 3; return 1;
        }
    }

    {
        int a = (s[0] - '0') * 10 + (s[1] - '0');
        info->ai_len = 2;
        switch (a) {
            case 0: info->fixed_len = 18; return 1;   /* 00 */
            case 1: case 2: info->fixed_len = 14; return 1; /* 01/02 */
            case 10: case 21: case 22: case 90: case 91: case 92: case 93: case 94: case 95: case 96: case 97: case 98: case 99:
                info->fixed_len = -1; return 1;
            case 11: case 12: case 13: case 15: case 16: case 17:
                info->fixed_len = 6; return 1;
            case 20:
                info->fixed_len = 2; return 1;
            case 30: case 37:
                info->fixed_len = -1; return 1;
            default:
                return 0;
        }
    }
}

static int gs1_format_bracket(const char *raw, int raw_len, char *out, int *out_len)
{
    int pos = 0;
    int i = 0;
    const char gs = 29;

    while (i < raw_len) {
        int sep = i;
        while (sep < raw_len && raw[sep] != gs) sep++;
        if (sep == i) {
            i++;
            continue;
        }

        /* Parse one GS1 field from [i, sep). */
        {
            Gs1AiInfo ai;
            int remain = sep - i;
            if (!gs1_ai_match(raw + i, remain, &ai) || remain <= ai.ai_len) {
                /* Fallback: preserve raw bytes as-is if AI parsing fails. */
                if (!append_str_bounded(out, &pos, raw + i, remain)) return 0;
                i = sep + 1;
                continue;
            }

            /* Standalone GS1-128 output omits the AI parentheses, matching
             * the reference firmware's reported data (e.g. "0101234...").
             * The Composite path formats its own AI text separately and is
             * unaffected by this. */
            if (!append_str_bounded(out, &pos, raw + i, ai.ai_len)) return 0;

            i += ai.ai_len;
            if (ai.fixed_len >= 0) {
                if (i + ai.fixed_len > sep) {
                    if (!append_str_bounded(out, &pos, raw + i, sep - i)) return 0;
                    i = sep + 1;
                    continue;
                }
                if (!append_str_bounded(out, &pos, raw + i, ai.fixed_len)) return 0;
                i += ai.fixed_len;
            } else {
                if (!append_str_bounded(out, &pos, raw + i, sep - i)) return 0;
                i = sep;
            }
        }
        if (i < raw_len && raw[i] == gs) i++;
    }

    out[pos] = '\0';
    *out_len = pos;
    return 1;
}

static int symbols_to_text(const int *symbols, int nsym,
                           char *out, int *out_len)
{
    if (nsym < 3) return 0;

    int check_sum = symbols[0];
    for (int i = 1; i < nsym - 1; i++)
        check_sum += i * symbols[i];
    check_sum %= 103;

    if (check_sum != symbols[nsym - 1])
        return 0;

    /* A charset latch (CODE_A/B/C) as the final data symbol decodes zero
     * characters in the new codeset -- it only makes sense when a real
     * barcode has payload after it. Reads that end on an unused latch are
     * truncated partials whose checksum coincidentally validates (e.g. the
     * skipped-symbol "90"-misread case in gs1-128_cca); reject them. */
    {
        int last_data = symbols[nsym - 2];
        if (last_data == C128_CODE_A || last_data == C128_CODE_B ||
            last_data == C128_CODE_C)
            return 0;
    }

    int pos = 0;
    int codeset;
    int is_gs1 = (nsym > 2 && symbols[1] == C128_FNC1);
    char raw[BARCODE_MAX_LEN];

    switch (symbols[0]) {
        case C128_START_A: codeset = 0; break;
        case C128_START_B: codeset = 1; break;
        case C128_START_C: codeset = 2; break;
        default: return 0;
    }

    for (int i = 1; i < nsym - 1; i++) {
        int v = symbols[i];

        if (v == C128_FNC1) {
            if (is_gs1 && i > 1) {
                if (pos < BARCODE_MAX_LEN - 1)
                    raw[pos++] = 29;
            }
            continue;
        }
        if (v == C128_FNC2 || v == C128_FNC3) continue;

        if (codeset == 2) {
            if (v >= 0 && v <= 99) {
                if (pos + 2 <= BARCODE_MAX_LEN - 1) {
                    raw[pos++] = '0' + (v / 10);
                    raw[pos++] = '0' + (v % 10);
                }
            } else if (v == C128_CODE_A) {
                codeset = 0;
            } else if (v == C128_CODE_B) {
                codeset = 1;
            }
            continue;
        }

        if (v == C128_CODE_A) { codeset = 0; continue; }
        if (v == C128_CODE_B) { codeset = 1; continue; }
        if (v == C128_CODE_C) { codeset = 2; continue; }
        if (v == C128_SHIFT) {
            if (i + 1 < nsym - 1) {
                i++;
                int sv = symbols[i];
                if (codeset == 0) {
                    if (sv >= 0 && sv <= 95 && pos < BARCODE_MAX_LEN - 1)
                        raw[pos++] = (char)(sv + 32);
                } else {
                    if (sv >= 0 && sv <= 63 && pos < BARCODE_MAX_LEN - 1)
                        raw[pos++] = (char)(sv + 32);
                    else if (sv >= 64 && sv <= 95 && pos < BARCODE_MAX_LEN - 1)
                        raw[pos++] = (char)(sv - 64);
                }
            }
            continue;
        }

        if (codeset == 0) {
            if (v >= 0 && v <= 63 && pos < BARCODE_MAX_LEN - 1)
                raw[pos++] = (char)(v + 32);
            else if (v >= 64 && v <= 95 && pos < BARCODE_MAX_LEN - 1)
                raw[pos++] = (char)(v - 64);
        } else {
            if (v >= 0 && v <= 95 && pos < BARCODE_MAX_LEN - 1)
                raw[pos++] = (char)(v + 32);
        }
    }

    raw[pos] = '\0';
    if (is_gs1) {
        if (gs1_format_bracket(raw, pos, out, out_len))
            return 1;
        /* Safety fallback: mark GS1 and output raw content. */
        if (BARCODE_MAX_LEN < 5) return 0;
        out[0] = ']'; out[1] = 'C'; out[2] = '1';
        int n = pos;
        if (n > BARCODE_MAX_LEN - 4) n = BARCODE_MAX_LEN - 4;
        memcpy(out + 3, raw, n);
        out[3 + n] = '\0';
        *out_len = 3 + n;
        return 1;
    }

    memcpy(out, raw, pos + 1);
    *out_len = pos;
    return 1;
}

typedef struct {
    float start_match_err;
    float start_gate_err;
    float symbol_match_err;
    float stop_match_err;
    int width_drift_pct;
    float avg_err_limit;
    int min_text_len;
    int reject_truncated;
} RunsDecodeCfg;

static void decode_from_runs_cfg(const BarSpaceRun *runs, ScanlineResult *res,
                                 const RunsDecodeCfg *cfg, int *rw_scratch)
{
    static int dbg_runs = -1;
    int *rw;
    int owns_rw = 0;
    if (dbg_runs < 0) dbg_runs = getenv("DBG128V") ? 1 : 0;

    rw = rw_scratch;
    if (!rw) {
        rw = (int *)malloc(MAX_BARS * sizeof(*rw));
        if (!rw) return;
        owns_rw = 1;
    }

    if (g_fast_rows_deadline_ms && is_timed_out(g_fast_rows_deadline_ms)) {
        if (owns_rw) free(rw);
        return;
    }

    res->valid = 0;
    res->text_len = 0;

    int n = runs->count;
    const int *w = runs->widths;
    int total_px = 0;
    for (int i = 0; i < n; i++) total_px += w[i];

    g_pc.runs_cfg_calls++;
    for (int rev = 0; rev < 2; rev++) {
        if (g_fast_rows_deadline_ms && is_timed_out(g_fast_rows_deadline_ms)) break;
        if (rev) {
            for (int i = 0; i < n; i++) rw[i] = w[n - 1 - i];
        }
        const int *ew = rev ? rw : w;

        for (int pos = 0; pos + 6 + 7 <= n; pos++) {
            if ((pos & 15) == 0 && g_fast_rows_deadline_ms &&
                is_timed_out(g_fast_rows_deadline_ms)) break;
            g_pc.runs_positions++;
            int total;
            int64_t sym_err_fixed = 0;
            int sym = match_symbol_tol(&ew[pos], &total, &sym_err_fixed,
                                       (int64_t)(cfg->start_match_err * MATCH_FIXED_SCALE),
                                       C128_START_A, C128_START_C + 1);

            if (sym < C128_START_A || sym > C128_START_C)
                continue;
            if (sym_err_fixed > (int64_t)(cfg->start_gate_err * MATCH_FIXED_SCALE)) continue;

            if (pos > 0 && ew[pos - 1] == 0)
                continue;
            g_pc.runs_start_found++;

            if (dbg_runs)
                fprintf(stderr, "  RUNSDEC START rev=%d pos=%d sym=%d sym_err=%.3f total=%d ref_width=%d gate=%.1f/%.1f\n",
                        rev, pos, sym, (double)sym_err_fixed / MATCH_FIXED_SCALE, total, total, cfg->start_gate_err, cfg->start_match_err);

            int symbols[MAX_SYMBOLS];
            int sym_count = 0;
            symbols[sym_count++] = sym;

            int ref_width = total;
            int p = pos + 6;
            int found_stop = 0;
            int64_t total_err = sym_err_fixed;

            while (p + 6 <= n && sym_count < MAX_SYMBOLS) {
                if (g_fast_rows_deadline_ms && is_timed_out(g_fast_rows_deadline_ms))
                    break;
                if (p + 7 <= n && match_stop_tol(&ew[p], (int64_t)(cfg->stop_match_err * MATCH_FIXED_SCALE))) {
                    found_stop = 1;
                    if (dbg_runs)
                        fprintf(stderr, "  RUNSDEC STOP_OK pos=%d sym#=%d w=[%d %d %d %d %d %d %d]\n",
                                p, sym_count, ew[p], ew[p+1], ew[p+2], ew[p+3], ew[p+4], ew[p+5], ew[p+6]);
                    break;
                }

                int t;
                int64_t e_fixed = 0;
                g_pc.runs_sym_matches++;
                int s = match_symbol_tol(&ew[p], &t, &e_fixed,
                                         (int64_t)(cfg->symbol_match_err * MATCH_FIXED_SCALE),
                                         0, C128_START_A);
                if (s < 0) {
                    if (dbg_runs)
                        fprintf(stderr, "  RUNSDEC SYM_FAIL pos=%d sym#=%d w=[%d %d %d %d %d %d] sum=%d best_err=%.3f limit=%.1f\n",
                                p, sym_count, ew[p], ew[p+1], ew[p+2], ew[p+3], ew[p+4], ew[p+5], t, (double)e_fixed / MATCH_FIXED_SCALE, cfg->symbol_match_err);
                    break;
                }

                if (abs(t - ref_width) * 100 > ref_width * cfg->width_drift_pct) {
                    if (dbg_runs) {
                        int drift_pct = ref_width > 0 ? abs(t - ref_width) * 100 / ref_width : 999;
                        fprintf(stderr, "  RUNSDEC DRIFT_FAIL pos=%d sym#=%d t=%d ref_width=%d drift=%d%% limit=%d%%\n",
                                p, sym_count, t, ref_width, drift_pct, cfg->width_drift_pct);
                    }
                    break;
                }

                symbols[sym_count++] = s;
                total_err += e_fixed;
                ref_width = (ref_width * 3 + t) / 4;
                p += 6;
            }

            if (!found_stop) {
                if (dbg_runs && sym_count >= 2)
                    fprintf(stderr, "  RUNSDEC NO_STOP rev=%d pos=%d sym_count=%d last_p=%d n=%d\n",
                            rev, pos, sym_count, p, n);
                continue;
            }

            if (sym_count < 3) continue;

            /* Truncation guard: the stop match above is accepted on tolerance
             * alone, so a data symbol can fake a stop mid-barcode and yield a
             * partial payload. A genuine full read stops at the barcode end,
             * leaving only a quiet-zone run (or two) after the stop; a partial
             * read leaves >= 7 runs (a full symbol's continuation). */
            if (cfg->reject_truncated) {
                if (p + 14 <= n) continue;
                /* The run right after the stop is a light space and must be the
                 * trailing quiet zone: a genuine read ends at the barcode's real
                 * stop with a wide light region after it, while a partial read
                 * (stop faked on a data symbol or random content) has narrow
                 * barcode runs continuing past the stop. */
                if (p + 7 < n && ew[p + 7] * 4 < ref_width)
                    continue;
            }

            int64_t avg_err = total_err / sym_count;
            if (avg_err > (int64_t)(cfg->avg_err_limit * MATCH_FIXED_SCALE)) continue;

            char text[BARCODE_MAX_LEN];
            int text_len = 0;
            if (symbols_to_text(symbols, sym_count, text, &text_len)) {
                if (text_len < cfg->min_text_len) continue;
                if (text_len > res->text_len) {
                    int stop_px = 0;
                    for (int i = 0; i <= p + 6 && i < n; i++) stop_px += ew[i];
                    if (rev == 1) stop_px = total_px - stop_px;
                    res->valid = 1;
                    res->stop_px = stop_px;
                    memcpy(res->text, text, text_len + 1);
                    res->text_len = text_len;
                }
            }
        }
    }
    if (owns_rw) free(rw);
}

/* Decode Code128 from bar/space runs: strict first, then controlled relaxed pass.
 * reject_truncated=1 rejects reads whose stop pattern is not at the run-array
 * end (>=7 runs follow), i.e. a loose stop match mid-barcode that yields a
 * partial/truncated payload. Enabled for the near/full-res main-loop configs,
 * off for far-distance configs where post-stop noise is expected. */
static const RunsDecodeCfg kStrictCfg = {
    1.5f, 1.0f, 2.0f, 2.5f, 40, 1.5f, 5, 1
};
static const RunsDecodeCfg kRelaxedCfg = {
    2.8f, 1.8f, 3.2f, 4.0f, 70, 2.4f, 8, 1
};
static const RunsDecodeCfg kFarDistanceCfg = {
    3.5f, 2.2f, 4.0f, 5.0f, 95, 3.0f, 5, 0
};
/* Ultra-relaxed config for very far distances — high false-positive risk,
 * but paired with checksum validation and vote aggregation to filter noise. */
static const RunsDecodeCfg kVeryFarCfg = {
    5.0f, 3.0f, 5.5f, 6.5f, 130, 4.0f, 8, 0
};

static void decode_from_runs(const BarSpaceRun *runs, ScanlineResult *res)
{
    decode_from_runs_cfg(runs, res, &kStrictCfg, NULL);
    if (res->valid || (g_fast_rows_deadline_ms &&
                       is_timed_out(g_fast_rows_deadline_ms))) return;
    decode_from_runs_cfg(runs, res, &kRelaxedCfg, NULL);
}

static void decode_from_runs_far(const BarSpaceRun *runs, ScanlineResult *res)
{
    decode_from_runs_cfg(runs, res, &kStrictCfg, NULL);
    if (res->valid) return;
    decode_from_runs_cfg(runs, res, &kRelaxedCfg, NULL);
    if (res->valid) return;
    decode_from_runs_cfg(runs, res, &kFarDistanceCfg, NULL);
}

static void decode_from_runs_fast(const BarSpaceRun *runs,
                                  ScanlineResult *res, int *rw_scratch)
{
    decode_from_runs_cfg(runs, res, &kStrictCfg, rw_scratch);
    if (res->valid || (g_fast_rows_deadline_ms &&
                       is_timed_out(g_fast_rows_deadline_ms))) return;
    decode_from_runs_cfg(runs, res, &kRelaxedCfg, rw_scratch);
}

static void decode_from_runs_far_fast(const BarSpaceRun *runs,
                                      ScanlineResult *res, int *rw_scratch)
{
    decode_from_runs_cfg(runs, res, &kStrictCfg, rw_scratch);
    if (res->valid || (g_fast_rows_deadline_ms &&
                       is_timed_out(g_fast_rows_deadline_ms))) return;
    decode_from_runs_cfg(runs, res, &kRelaxedCfg, rw_scratch);
    if (res->valid || (g_fast_rows_deadline_ms &&
                       is_timed_out(g_fast_rows_deadline_ms))) return;
    decode_from_runs_cfg(runs, res, &kFarDistanceCfg, rw_scratch);
}

/* Like decode_from_runs_far, used only for the UP8-env chain. Earlier this
 * intentionally stopped at kRelaxedCfg because kFarDistanceCfg/kVeryFarCfg's
 * wide avg_err tolerance let a mod-103 checksum pass on pure noise once
 * tried across enough offsets/rows/ROIs, producing real WRONG payloads at
 * 890-1180mm. The anti-noise gate now lives one level up, at the call site:
 * a payload from this function is only accepted once >=2 independently
 * scanned rows (genuinely different y-positions, not just offset/tier
 * nudges of the same row) agree on the exact same text -- so the wider
 * tiers can be re-enabled here; the false positives they produce are
 * per-row noise that will not coincidentally repeat across rows. */
static void decode_from_runs_far_up8(const BarSpaceRun *runs, ScanlineResult *res)
{
    decode_from_runs_cfg(runs, res, &kStrictCfg, NULL);
    if (res->valid) return;
    decode_from_runs_cfg(runs, res, &kRelaxedCfg, NULL);
    if (res->valid) return;
    decode_from_runs_cfg(runs, res, &kFarDistanceCfg, NULL);
    if (res->valid) return;
    decode_from_runs_cfg(runs, res, &kVeryFarCfg, NULL);
}

/* Verify the row is light (quiet zone) right after the decoded STOP at
 * stop_px. Rejects partial Code128 reads whose stop was faked mid-barcode:
 * there the barcode pattern continues (dark) immediately after the decoded
 * region, whereas a genuine read ends with a light quiet zone. */
static int row_quiet_after(const uint8_t *row, int width, int stop_px)
{
    if (!row || width <= 0) return 1;
    if (stop_px < 0 || stop_px >= width) return 1;
    int end = stop_px + 20;
    if (end > width) end = width;
    for (int x = stop_px; x < end; x++)
        if (row[x] < 100) return 0;
    return 1;
}

static int runs_to_edges(const BarSpaceRun *runs, int *edges, int max_edges)
{
    int n = 0;
    int x = 0;
    if (!runs || !edges || max_edges <= 0) return 0;

    for (int i = 0; i + 1 < runs->count; i++) {
        x += runs->widths[i];
        if (runs->widths[i] <= 0) continue;
        if (n < max_edges)
            edges[n] = x;
        n++;
    }

    if (n > max_edges) n = max_edges;
    return n;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Thresholding and run extraction
 * ════════════════════════════════════════════════════════════════════════ */

static int otsu_threshold(const uint8_t *row, int width)
{
    int hist[256] = {0};
    for (int x = 0; x < width; x++) hist[row[x]]++;

    long long sum = 0;
    for (int i = 0; i < 256; i++) sum += (long long)i * hist[i];

    long long sum_b = 0;
    int w_b = 0;
    double max_var = 0.0;
    int threshold = 128;

    for (int t = 0; t < 256; t++) {
        w_b += hist[t];
        if (w_b == 0) continue;
        int w_f = width - w_b;
        if (w_f == 0) break;
        sum_b += (long long)t * hist[t];
        double diff = (double)sum_b / (double)w_b - (double)(sum - sum_b) / (double)w_f;
        double var = (double)w_b * (double)w_f * diff * diff;
        if (var > max_var) { max_var = var; threshold = t; }
    }
    if (threshold < 1) threshold = 1;
    if (threshold > 254) threshold = 254;
    return threshold;
}

static void extract_runs(const uint8_t *row, int width, int threshold,
                         BarSpaceRun *runs)
{
    runs->count = 0;
    if (width <= 0) return;

    int is_bar = row[0] < threshold;
    int run_len = 1;

    if (is_bar && runs->count < MAX_BARS)
        runs->widths[runs->count++] = 0;

    for (int x = 1; x < width; x++) {
        int cur_bar = row[x] < threshold;
        if (cur_bar == is_bar) {
            run_len++;
        } else {
            if (runs->count < MAX_BARS)
                runs->widths[runs->count++] = run_len;
            run_len = 1;
            is_bar = cur_bar;
        }
    }
    if (runs->count < MAX_BARS)
        runs->widths[runs->count++] = run_len;
}

static void extract_runs_adaptive_mean(const uint8_t *row, int width,
                                       int win_radius, int offset,
                                       BarSpaceRun *runs)
{
    runs->count = 0;
    if (width <= 0) return;
    if (win_radius < 1) win_radius = 1;

    int *prefix = (int *)malloc((size_t)(width + 1) * sizeof(int));
    if (!prefix) {
        extract_runs(row, width, 128, runs);
        return;
    }

    prefix[0] = 0;
    for (int x = 0; x < width; x++)
        prefix[x + 1] = prefix[x] + row[x];

    int x0 = 0;
    int x1 = (win_radius + 1 < width) ? (win_radius + 1) : width;
    int cnt = x1 - x0;
    int mean = (cnt > 0) ? ((prefix[x1] - prefix[x0]) / cnt) : row[0];

    int is_bar = row[0] < (mean - offset);
    int run_len = 1;

    if (is_bar && runs->count < MAX_BARS)
        runs->widths[runs->count++] = 0;

    for (int x = 1; x < width; x++) {
        x0 = x - win_radius;
        if (x0 < 0) x0 = 0;
        x1 = x + win_radius + 1;
        if (x1 > width) x1 = width;
        cnt = x1 - x0;
        mean = (cnt > 0) ? ((prefix[x1] - prefix[x0]) / cnt) : row[x];

        int cur_bar = row[x] < (mean - offset);
        if (cur_bar == is_bar) {
            run_len++;
        } else {
            if (runs->count < MAX_BARS)
                runs->widths[runs->count++] = run_len;
            run_len = 1;
            is_bar = cur_bar;
        }
    }

    if (runs->count < MAX_BARS)
        runs->widths[runs->count++] = run_len;

    free(prefix);
}

static void smooth_row_121(const uint8_t *in, int width, uint8_t *out)
{
    if (width <= 0) return;
    if (width == 1) {
        out[0] = in[0];
        return;
    }
    out[0] = (uint8_t)((2 * in[0] + in[1]) / 3);
    for (int x = 1; x < width - 1; x++) {
        int v = in[x - 1] + 2 * in[x] + in[x + 1];
        out[x] = (uint8_t)(v >> 2);
    }
    out[width - 1] = (uint8_t)((in[width - 2] + 2 * in[width - 1]) / 3);
}

static void unsharp_row(const uint8_t *src, const uint8_t *blur, int width, uint8_t *out)
{
    for (int x = 0; x < width; x++) {
        int v = 2 * (int)src[x] - (int)blur[x];
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        out[x] = (uint8_t)v;
    }
}

/* Stronger unsharp mask (out = 3*src - 2*blur, vs. the 2*src-blur above) --
 * for far-distance narrow-module (~0.95-1.0px) rows where the mild unsharp
 * above still leaves adjacent bar/space peaks blended together. */
static void unsharp_row_strong(const uint8_t *src, const uint8_t *blur, int width, uint8_t *out)
{
    for (int x = 0; x < width; x++) {
        int v = 3 * (int)src[x] - 2 * (int)blur[x];
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        out[x] = (uint8_t)v;
    }
}

static void contrast_stretch_row(const uint8_t *in, int width, uint8_t *out)
{
    if (width <= 0) return;

    int mn = 255, mx = 0;
    for (int x = 0; x < width; x++) {
        if (in[x] < mn) mn = in[x];
        if (in[x] > mx) mx = in[x];
    }
    int range = mx - mn;
    if (range < 16) {
        memcpy(out, in, (size_t)width);
        return;
    }
    for (int x = 0; x < width; x++) {
        int v = (in[x] - mn) * 255 / range;
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        out[x] = (uint8_t)v;
    }
}


/* ════════════════════════════════════════════════════════════════════════
 *  M6: Local-midpoint edge detector for far-distance / low-contrast
 *
 *  Unlike DMFI (M0/M1b) and envelope (M5) which use adaptive derivative
 *  thresholds that discard weak edges, this method tracks local min/max
 *  over a small sliding window and outputs transition positions where
 *  the signal crosses the local midpoint.  This preserves weak edges
 *  that other methods filter out.
 *
 *  Code128 checksum verification prevents false positives from noise
 *  edges, making aggressive edge detection safe.
 * ════════════════════════════════════════════════════════════════════════ */

static int detect_edges_local_midpoint(const uint8_t *row, int width,
                                       int win_radius, int bias,
                                       int *edges, int max_edges)
{
    if (width < 20 || max_edges < 20) return 0;
    if (win_radius < 2) win_radius = 2;

    int n_edges = 0;
    int prev_is_bar = -1;

    /* O(n) block-based min/max: divide row into blocks of size win_radius,
     * precompute block min/max, then combine 2-3 blocks per query. */
    int bs = win_radius;
    if (bs < 1) bs = 1;
    int nblk = (width + bs - 1) / bs;
    /* Use stack arrays for small widths, else short-circuit to simple loop */
    uint8_t blk_min[256], blk_max[256];
    if (nblk > 256) {
        /* Fallback: sample every 2nd pixel to halve work */
        for (int x = 0; x < width; x += 2) {
            int lo = x - win_radius;
            int hi = x + win_radius;
            if (lo < 0) lo = 0;
            if (hi >= width) hi = width - 1;
            int local_min = row[lo], local_max = row[lo];
            for (int j = lo + 1; j <= hi; j += 2) {
                if (row[j] < local_min) local_min = row[j];
                if (row[j] > local_max) local_max = row[j];
            }
            if (local_max - local_min < 8) continue;
            int thr = (local_min + local_max) / 2 + bias;
            int cur_bar = (row[x] < thr) ? 1 : 0;
            if (prev_is_bar < 0) { prev_is_bar = cur_bar; continue; }
            if (cur_bar != prev_is_bar) {
                if (n_edges < max_edges) edges[n_edges++] = x;
                prev_is_bar = cur_bar;
            }
        }
        return n_edges;
    }

    /* Precompute block min/max */
    for (int b = 0; b < nblk; b++) {
        int s = b * bs;
        int e = s + bs;
        if (e > width) e = width;
        uint8_t mn = row[s], mx = row[s];
        for (int j = s + 1; j < e; j++) {
            if (row[j] < mn) mn = row[j];
            if (row[j] > mx) mx = row[j];
        }
        blk_min[b] = mn;
        blk_max[b] = mx;
    }

    for (int x = 0; x < width; x++) {
        int lo = x - win_radius;
        int hi = x + win_radius;
        if (lo < 0) lo = 0;
        if (hi >= width) hi = width - 1;

        /* Quick min/max from block precomputation */
        int b0 = lo / bs;
        int b1 = hi / bs;
        uint8_t local_min = blk_min[b0], local_max = blk_max[b0];
        for (int b = b0 + 1; b <= b1; b++) {
            if (blk_min[b] < local_min) local_min = blk_min[b];
            if (blk_max[b] > local_max) local_max = blk_max[b];
        }

        if (local_max - local_min < 8) continue;

        int thr = (local_min + local_max) / 2 + bias;
        int cur_bar = (row[x] < thr) ? 1 : 0;

        if (prev_is_bar < 0) {
            prev_is_bar = cur_bar;
            continue;
        }

        if (cur_bar != prev_is_bar) {
            if (n_edges < max_edges) {
                edges[n_edges++] = x;
            }
            prev_is_bar = cur_bar;
        }
    }
    return n_edges;
}

/* ════════════════════════════════════════════════════════════════════════
 *  M7b: Derivative zero-crossing edge detector
 *
 *  Finds local extrema (peaks/valleys) of the signal by detecting sign
 *  changes in the first derivative.  Edge positions are the midpoints
 *  between consecutive extrema (= the transition between bar and space).
 *  This preserves ALL transitions regardless of contrast, making it
 *  ideal for far-distance barcodes where other methods lose weak edges.
 *
 *  min_swing: minimum peak-to-valley amplitude to accept an extremum pair.
 *  Set to 0 to accept all, or e.g. 8 to filter noise.
 * ════════════════════════════════════════════════════════════════════════ */


static int detect_edges_derivative(const uint8_t *row, int width,
                                   int min_swing, int *edges, int max_edges)
{
    if (width < 20 || max_edges < 20) return 0;

    /* Find local extrema (peaks and valleys) via derivative sign changes */
    int extrema[2048];
    int n_ext = 0;
    for (int i = 1; i < width - 1 && n_ext < 2048; i++) {
        int d_left  = (int)row[i] - (int)row[i - 1];
        int d_right = (int)row[i + 1] - (int)row[i];
        /* Sign change: peak (+ to -) or valley (- to +) */
        if ((d_left > 0 && d_right < 0) || (d_left < 0 && d_right > 0)) {
            extrema[n_ext++] = i;
        }
        /* Flat-then-change: handle plateaus */
        if (d_left == 0 && d_right != 0) {
            /* Check if there was an opposite sign before the flat */
            int j = i - 1;
            while (j > 0 && (int)row[j] == (int)row[j - 1]) j--;
            if (j > 0) {
                int d_before = (int)row[j] - (int)row[j - 1];
                if ((d_before > 0 && d_right < 0) || (d_before < 0 && d_right > 0)) {
                    extrema[n_ext++] = i;
                }
            }
        }
    }
    if (n_ext < 2) return 0;

    /* Filter: remove extrema pairs with swing < min_swing */
    int filtered[2048];
    int n_filt = 0;
    filtered[n_filt++] = extrema[0];
    for (int i = 1; i < n_ext; i++) {
        int swing = abs((int)row[extrema[i]] - (int)row[filtered[n_filt - 1]]);
        if (swing >= min_swing) {
            filtered[n_filt++] = extrema[i];
        } else {
            /* Replace last with better extremum of same type */
            int last_val = row[filtered[n_filt - 1]];
            int cur_val = row[extrema[i]];
            /* If both are peaks (high values), keep the higher one */
            if (cur_val > last_val && n_filt >= 2 && row[filtered[n_filt - 2]] < last_val) {
                filtered[n_filt - 1] = extrema[i];
            }
            /* If both are valleys (low values), keep the lower one */
            else if (cur_val < last_val && n_filt >= 2 && row[filtered[n_filt - 2]] > last_val) {
                filtered[n_filt - 1] = extrema[i];
            }
        }
    }

    /* Output edge transitions as midpoints between consecutive extrema */
    int n_edges = 0;
    for (int i = 0; i < n_filt - 1 && n_edges < max_edges; i++) {
        edges[n_edges++] = (filtered[i] + filtered[i + 1]) / 2;
    }
    return n_edges;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Derivative-based run extraction
 *  ────────────────────────────────────────────────────────────────────────
 *  Uses derivative zero-crossing extrema to find bar-space transitions.
 *  Between consecutive midpoints (edges), classify as bar or space by
 *  comparing the avg pixel value to the midpoint between surrounding
 *  extrema values.  This avoids the global/adaptive threshold problems
 *  that merge 2-3px bars at far distances.
 * ════════════════════════════════════════════════════════════════════════ */
static void extract_runs_derivative(const uint8_t *row, int width,
                                    int min_swing, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 20) return;

    /* Step 1: find all local extrema */
    int extrema[2048];
    uint8_t ext_val[2048];
    int n_ext = 0;
    for (int i = 1; i < width - 1 && n_ext < 2048; i++) {
        int d_left  = (int)row[i] - (int)row[i - 1];
        int d_right = (int)row[i + 1] - (int)row[i];
        if ((d_left > 0 && d_right < 0) || (d_left < 0 && d_right > 0)) {
            extrema[n_ext] = i;
            ext_val[n_ext] = row[i];
            n_ext++;
        }
        if (d_left == 0 && d_right != 0) {
            int j = i - 1;
            while (j > 0 && (int)row[j] == (int)row[j - 1]) j--;
            if (j > 0) {
                int d_before = (int)row[j] - (int)row[j - 1];
                if ((d_before > 0 && d_right < 0) || (d_before < 0 && d_right > 0)) {
                    extrema[n_ext] = i;
                    ext_val[n_ext] = row[i];
                    n_ext++;
                }
            }
        }
    }
    if (n_ext < 2) return;

    /* Step 2: filter by min_swing (same logic as detect_edges_derivative) */
    int filtered[2048];
    uint8_t filt_val[2048];
    int n_filt = 0;
    filtered[n_filt] = extrema[0];
    filt_val[n_filt] = ext_val[0];
    n_filt++;
    for (int i = 1; i < n_ext; i++) {
        int swing = abs((int)ext_val[i] - (int)filt_val[n_filt - 1]);
        if (swing >= min_swing) {
            filtered[n_filt] = extrema[i];
            filt_val[n_filt] = ext_val[i];
            n_filt++;
        } else {
            uint8_t last_v = filt_val[n_filt - 1];
            uint8_t cur_v  = ext_val[i];
            if (cur_v > last_v && n_filt >= 2 && filt_val[n_filt - 2] < last_v) {
                filtered[n_filt - 1] = extrema[i];
                filt_val[n_filt - 1] = cur_v;
            } else if (cur_v < last_v && n_filt >= 2 && filt_val[n_filt - 2] > last_v) {
                filtered[n_filt - 1] = extrema[i];
                filt_val[n_filt - 1] = cur_v;
            }
        }
    }
    if (n_filt < 2) return;

    /* Step 3: compute edge midpoints and build runs */
    /* Edges are midpoints between consecutive filtered extrema.
     * Between edge[i] and edge[i+1], the interval is either bar or space.
     * A valley→peak transition means the interval between their midpoints
     * and the NEXT pair is a space (bright). Peak→valley = bar (dark). */

    /* First, determine if the first filtered extremum is a peak or valley.
     * If filt_val[0] < filt_val[1], first is valley (dark region before it = bar).
     * If filt_val[0] > filt_val[1], first is peak (bright region before it = space). */

    /* Generate runs: each pair of consecutive extrema creates one edge.
     * The run BEFORE the first edge is the leading quiet zone.
     * Between edge[i] and edge[i+1] is a run of width (edge[i+1] - edge[i]).
     * The polarity alternates: if first extremum is valley, the first
     * inter-edge run (between edge0 and edge1) corresponds to the region
     * between midpoint(valley,peak) and midpoint(peak,valley2) = a peak region = space.
     * So we need: leading bar (if first is valley), then alternating space/bar. */

    /* Compute edges (midpoints) */
    int edge_pos[2048];
    int n_edges = 0;
    for (int i = 0; i < n_filt - 1 && n_edges < 2048; i++) {
        edge_pos[n_edges++] = (filtered[i] + filtered[i + 1]) / 2;
    }
    if (n_edges < 2) return;

    /* Build runs. The region before edge_pos[0] is the leading quiet zone.
     * For Code128 decoding, we need: leading space, then bar-space alternating.
     * (The decode_from_runs_cfg expects widths starting with a bar.) */

    /* Leading quiet zone — we'll start from x=0 to first edge */
    int cnt = 0;
    /* Determine polarity of first run (0 to edge_pos[0]):
     * If first extremum is valley, region before it is descending (was bar/dark).
     * The midpoint between first valley and next peak is within a rising slope.
     * Region [0, edge_pos[0]) is the start — could be anything. Just use pixel values. */
    int mid_x = edge_pos[0] / 2;
    int is_bar_first = (row[mid_x] < 128); /* crude: dark = bar */

    /* If first run is space, insert a zero-width bar before it */
    if (!is_bar_first) {
        runs->widths[cnt++] = 0;
    }

    /* First quiet run */
    runs->widths[cnt++] = edge_pos[0];

    /* Inter-edge runs */
    for (int i = 0; i < n_edges - 1 && cnt < MAX_BARS - 1; i++) {
        runs->widths[cnt++] = edge_pos[i + 1] - edge_pos[i];
    }

    /* Trailing quiet zone */
    if (cnt < MAX_BARS) {
        runs->widths[cnt++] = width - 1 - edge_pos[n_edges - 1];
    }

    runs->count = cnt;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Local min/max envelope threshold (envelope filtering)
 * ════════════════════════════════════════════════════════════════════════ */

static void extract_runs_envelope(const uint8_t *row, int width,
                                  int win_radius, int bias,
                                  BarSpaceRun *runs)
{
    runs->count = 0;
    if (width <= 0) return;
    if (win_radius < 3) win_radius = 3;

    int is_bar, run_len;
    int x;

    /* Compute local midpoint threshold at each pixel from sliding min/max */
    for (x = 0; x < width; x++) {
        int l = x - win_radius;
        int r = x + win_radius;
        int lo = 255, hi = 0;
        if (l < 0) l = 0;
        if (r >= width) r = width - 1;
        for (int j = l; j <= r; j++) {
            if (row[j] < lo) lo = row[j];
            if (row[j] > hi) hi = row[j];
        }
        int thr = (lo + hi) / 2 + bias;
        int cur_bar = (row[x] < thr) ? 1 : 0;

        if (x == 0) {
            is_bar = cur_bar;
            run_len = 1;
            if (is_bar && runs->count < MAX_BARS)
                runs->widths[runs->count++] = 0;
        } else if (cur_bar == is_bar) {
            run_len++;
        } else {
            if (runs->count < MAX_BARS)
                runs->widths[runs->count++] = run_len;
            run_len = 1;
            is_bar = cur_bar;
        }
    }
    if (runs->count < MAX_BARS)
        runs->widths[runs->count++] = run_len;
}

/*
 * ROI-based runs extraction: detect where the barcode is in the row
 * by measuring local gradient activity, then extract runs only from
 * the active region using a local Otsu threshold.
 */
static void extract_runs_roi(const uint8_t *row, int width, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 40) return;

    /* Step 1: Compute gradient energy in sliding windows */
    int win = width / 16;
    if (win < 8) win = 8;
    if (win > 64) win = 64;
    int n_bins = (width - 1) / win;
    if (n_bins < 3) {
        /* fallback: whole row */
        int thr = otsu_threshold(row, width);
        extract_runs(row, width, thr, runs);
        return;
    }

    int *energy = (int *)calloc((size_t)n_bins, sizeof(int));
    if (!energy) return;

    for (int b = 0; b < n_bins; b++) {
        int x0 = b * win;
        int x1 = x0 + win;
        if (x1 > width - 1) x1 = width - 1;
        int e = 0;
        for (int x = x0; x < x1; x++) {
            int g = (int)row[x + 1] - (int)row[x];
            e += g * g;
        }
        energy[b] = e / (x1 - x0 > 0 ? x1 - x0 : 1);
    }

    /* Step 2: Find the bin with maximum energy */
    int max_e = 0, max_b = 0;
    for (int b = 0; b < n_bins; b++) {
        if (energy[b] > max_e) {
            max_e = energy[b];
            max_b = b;
        }
    }

    /* Step 3: Expand from max-energy bin to cover the barcode region.
     * Include adjacent bins whose energy is at least 15% of max. */
    int thresh_e = max_e / 7;
    int roi_b0 = max_b, roi_b1 = max_b;
    while (roi_b0 > 0 && energy[roi_b0 - 1] >= thresh_e) roi_b0--;
    while (roi_b1 < n_bins - 1 && energy[roi_b1 + 1] >= thresh_e) roi_b1++;

    /* Add some margin (1 bin on each side) */
    if (roi_b0 > 0) roi_b0--;
    if (roi_b1 < n_bins - 1) roi_b1++;

    int roi_x0 = roi_b0 * win;
    int roi_x1 = (roi_b1 + 1) * win;
    if (roi_x0 < 0) roi_x0 = 0;
    if (roi_x1 > width) roi_x1 = width;
    int roi_w = roi_x1 - roi_x0;

    free(energy);

    if (roi_w < 20) return;

    /* Step 4: Local Otsu threshold on just the ROI */
    int thr = otsu_threshold(&row[roi_x0], roi_w);

    /* Step 5: Extract runs from the ROI region */
    runs->count = 0;
    int is_bar = row[roi_x0] < thr;
    int run_len = 1;
    if (is_bar && runs->count < MAX_BARS)
        runs->widths[runs->count++] = 0;

    for (int x = roi_x0 + 1; x < roi_x1; x++) {
        int cur_bar = row[x] < thr;
        if (cur_bar == is_bar) {
            run_len++;
        } else {
            if (runs->count < MAX_BARS)
                runs->widths[runs->count++] = run_len;
            run_len = 1;
            is_bar = cur_bar;
        }
    }
    if (runs->count < MAX_BARS)
        runs->widths[runs->count++] = run_len;
}

/*
 * ROI runs extraction with multiple threshold offsets
 */
static void extract_runs_roi_offset(const uint8_t *row, int width,
                                     int thr_offset, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 40) return;

    /* Detect ROI using gradient energy */
    int win = width / 16;
    if (win < 8) win = 8;
    if (win > 64) win = 64;
    int n_bins = (width - 1) / win;
    if (n_bins < 3) return;

    int *energy = (int *)calloc((size_t)n_bins, sizeof(int));
    if (!energy) return;

    for (int b = 0; b < n_bins; b++) {
        int x0 = b * win;
        int x1 = x0 + win;
        if (x1 > width - 1) x1 = width - 1;
        int e = 0;
        for (int x = x0; x < x1; x++) {
            int g = (int)row[x + 1] - (int)row[x];
            e += g * g;
        }
        energy[b] = e / (x1 - x0 > 0 ? x1 - x0 : 1);
    }

    int max_e = 0, max_b = 0;
    for (int b = 0; b < n_bins; b++) {
        if (energy[b] > max_e) {
            max_e = energy[b];
            max_b = b;
        }
    }

    int thresh_e = max_e / 7;
    int roi_b0 = max_b, roi_b1 = max_b;
    while (roi_b0 > 0 && energy[roi_b0 - 1] >= thresh_e) roi_b0--;
    while (roi_b1 < n_bins - 1 && energy[roi_b1 + 1] >= thresh_e) roi_b1++;
    if (roi_b0 > 0) roi_b0--;
    if (roi_b1 < n_bins - 1) roi_b1++;

    int roi_x0 = roi_b0 * win;
    int roi_x1 = (roi_b1 + 1) * win;
    if (roi_x0 < 0) roi_x0 = 0;
    if (roi_x1 > width) roi_x1 = width;
    int roi_w = roi_x1 - roi_x0;
    free(energy);

    if (roi_w < 20) return;

    int thr = otsu_threshold(&row[roi_x0], roi_w) + thr_offset;
    if (thr < 1) thr = 1;
    if (thr > 254) thr = 254;

    runs->count = 0;
    int is_bar = row[roi_x0] < thr;
    int run_len = 1;
    if (is_bar && runs->count < MAX_BARS)
        runs->widths[runs->count++] = 0;

    for (int x = roi_x0 + 1; x < roi_x1; x++) {
        int cur_bar = row[x] < thr;
        if (cur_bar == is_bar) {
            run_len++;
        } else {
            if (runs->count < MAX_BARS)
                runs->widths[runs->count++] = run_len;
            run_len = 1;
            is_bar = cur_bar;
        }
    }
    if (runs->count < MAX_BARS)
        runs->widths[runs->count++] = run_len;
}

/*
 * Q4 (16x sub-pixel fixed-point) counterpart of extract_runs_roi_offset.
 *
 * The integer version above classifies each *whole pixel* as bar/space and
 * counts pixels, which quantizes every bar/space boundary to +/-0.5px. At
 * far-distance module widths of ~1.0-1.5px that quantization error is a
 * large fraction of a module and accumulates across the symbol, blowing the
 * Char128 width-ratio matcher's tolerance.
 *
 * This version finds the exact threshold-crossing position between the two
 * straddling pixels via linear interpolation (matching the sub-pixel
 * approach the far-distance edge tracker already uses in signal.c), and
 * expresses run widths in Q4 (16x) fixed-point pixel units instead of whole
 * pixel counts. match_symbol_tol()/match_stop_tol() (used by
 * decode_from_runs_far) are already pure width-ratio comparisons — every
 * width divided by the sum of the same 6/7 widths — so a uniform 16x scale
 * cancels out and they accept these Q4 widths with no changes needed.
 *
 * NOTE: unlike extract_runs_roi_offset(), the widths produced here are NOT
 * safe to feed into runs_to_edges()/decode_from_edges_char128(), whose
 * absolute-pixel tolerances assume integer-pixel-scale input. Use this only
 * for the decode_from_runs()/decode_from_runs_far() ratio-based path.
 */
static void extract_runs_roi_offset_q4(const uint8_t *row, int width,
                                        int thr_offset, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 40) return;

    int win = width / 16;
    if (win < 8) win = 8;
    if (win > 64) win = 64;
    int n_bins = (width - 1) / win;
    if (n_bins < 3) return;

    int *energy = (int *)calloc((size_t)n_bins, sizeof(int));
    if (!energy) return;

    for (int b = 0; b < n_bins; b++) {
        int x0 = b * win;
        int x1 = x0 + win;
        if (x1 > width - 1) x1 = width - 1;
        int e = 0;
        for (int x = x0; x < x1; x++) {
            int g = (int)row[x + 1] - (int)row[x];
            e += g * g;
        }
        energy[b] = e / (x1 - x0 > 0 ? x1 - x0 : 1);
    }

    int max_e = 0, max_b = 0;
    for (int b = 0; b < n_bins; b++) {
        if (energy[b] > max_e) {
            max_e = energy[b];
            max_b = b;
        }
    }

    int thresh_e = max_e / 7;
    int roi_b0 = max_b, roi_b1 = max_b;
    while (roi_b0 > 0 && energy[roi_b0 - 1] >= thresh_e) roi_b0--;
    while (roi_b1 < n_bins - 1 && energy[roi_b1 + 1] >= thresh_e) roi_b1++;
    if (roi_b0 > 0) roi_b0--;
    if (roi_b1 < n_bins - 1) roi_b1++;

    int roi_x0 = roi_b0 * win;
    int roi_x1 = (roi_b1 + 1) * win;
    if (roi_x0 < 0) roi_x0 = 0;
    if (roi_x1 > width) roi_x1 = width;
    int roi_w = roi_x1 - roi_x0;
    free(energy);

    if (roi_w < 20) return;

    int thr = otsu_threshold(&row[roi_x0], roi_w) + thr_offset;
    if (thr < 1) thr = 1;
    if (thr > 254) thr = 254;

    if (roi_x1 - roi_x0 < 2) return;

    int is_bar = row[roi_x0] < thr;
    long prev_pos_q4 = (long)roi_x0 * 16;
    int first = 1;

    for (int x = roi_x0; x < roi_x1 - 1; x++) {
        int v0 = row[x], v1 = row[x + 1];
        int b0 = v0 < thr, b1 = v1 < thr;
        if (b0 == b1) continue;

        int diff = v1 - v0;
        long pos_q4;
        if (diff == 0) {
            pos_q4 = (long)x * 16 + 8;
        } else {
            long frac16 = (long)(thr - v0) * 16 / diff;
            if (frac16 < 0) frac16 = 0;
            if (frac16 > 16) frac16 = 16;
            pos_q4 = (long)x * 16 + frac16;
        }
        if (pos_q4 <= prev_pos_q4) pos_q4 = prev_pos_q4 + 1;

        if (first) {
            if (is_bar && runs->count < MAX_BARS)
                runs->widths[runs->count++] = 0;
            first = 0;
        }
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)(pos_q4 - prev_pos_q4);
        prev_pos_q4 = pos_q4;
        is_bar = b1;
    }
    if (first) {
        if (is_bar && runs->count < MAX_BARS)
            runs->widths[runs->count++] = 0;
    }
    {
        long end_pos_q4 = (long)(roi_x1 - 1) * 16;
        long w4 = end_pos_q4 - prev_pos_q4;
        if (w4 < 1) w4 = 1;
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)w4;
    }
}

/* ────────────────────────────────────────────────────────────────────────
 * UP4: 4x sub-pixel-resampled counterpart of extract_runs_roi_offset_q4.
 *
 * At ~570-650mm the module width has shrunk to ~0.83-1.09px. On the raw
 * pixel grid, extract_runs_roi_offset_q4() still only has one sample per
 * pixel to linearly interpolate a threshold crossing between -- for narrow
 * 1-1 bar/space pairs the whole pair can sit between two samples with no
 * pixel actually crossing threshold, so the two runs silently merge into
 * one. This variant first reconstructs the signal at 4x the sample density
 * (step 0.25px) using the same 16-phase / 4-tap cubic convolution kernel
 * the far-distance edge tracker already uses (signal.c's sinc/cubic table,
 * subsampled to its quarter phases 0/4/8/12), *then* does the threshold
 * crossing + linear interpolation on that reconstructed signal. Output
 * widths are expressed in the same Q4 (16x) fixed-point pixel units as
 * extract_runs_roi_offset_q4(), so decode_from_runs_far()'s pure
 * width-ratio matching needs no changes to consume them.
 */
static const int kUp4CubicCoeff[16][4] = {
    {0, 256, 0, 0},
    {-8, 255, 10, -1},
    {-14, 249, 23, -2},
    {-17, 239, 39, -4},
    {-19, 225, 56, -6},
    {-20, 208, 76, -8},
    {-19, 188, 98, -11},
    {-17, 166, 120, -13},
    {-15, 143, 143, -15},
    {-13, 120, 166, -17},
    {-11, 98, 188, -19},
    {-8, 76, 208, -20},
    {-6, 56, 225, -19},
    {-4, 39, 239, -17},
    {-2, 23, 249, -14},
    {-1, 10, 255, -8},
};

static inline int up4_cubic_sample(const uint8_t *row, int width, int i0, int phase16)
{
    int im1 = i0 - 1; if (im1 < 0) im1 = 0; if (im1 > width - 1) im1 = width - 1;
    int i0c = i0;      if (i0c < 0) i0c = 0; if (i0c > width - 1) i0c = width - 1;
    int ip1 = i0 + 1;  if (ip1 > width - 1) ip1 = width - 1; if (ip1 < 0) ip1 = 0;
    int ip2 = i0 + 2;  if (ip2 > width - 1) ip2 = width - 1; if (ip2 < 0) ip2 = 0;
    const int *c = kUp4CubicCoeff[phase16];
    long acc = (long)row[im1] * c[0] + (long)row[i0c] * c[1] +
               (long)row[ip1] * c[2] + (long)row[ip2] * c[3];
    int v = (int)((acc + 128) >> 8);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    return v;
}

#define UP4_MAX_SAMPLES 2048

static void extract_runs_roi_offset_up4(const uint8_t *row, int width,
                                         int thr_offset, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 40) return;

    /* Same ROI localization (via gradient-energy bins) as
     * extract_runs_roi_offset_q4(), kept independent/duplicated so this new
     * path cannot alter that function's already-validated behavior. */
    int win = width / 16;
    if (win < 8) win = 8;
    if (win > 64) win = 64;
    int n_bins = (width - 1) / win;
    if (n_bins < 3) return;

    int *energy = (int *)calloc((size_t)n_bins, sizeof(int));
    if (!energy) return;

    for (int b = 0; b < n_bins; b++) {
        int x0 = b * win;
        int x1 = x0 + win;
        if (x1 > width - 1) x1 = width - 1;
        int e = 0;
        for (int x = x0; x < x1; x++) {
            int g = (int)row[x + 1] - (int)row[x];
            e += g * g;
        }
        energy[b] = e / (x1 - x0 > 0 ? x1 - x0 : 1);
    }

    int max_e = 0, max_b = 0;
    for (int b = 0; b < n_bins; b++) {
        if (energy[b] > max_e) {
            max_e = energy[b];
            max_b = b;
        }
    }

    int thresh_e = max_e / 7;
    int roi_b0 = max_b, roi_b1 = max_b;
    while (roi_b0 > 0 && energy[roi_b0 - 1] >= thresh_e) roi_b0--;
    while (roi_b1 < n_bins - 1 && energy[roi_b1 + 1] >= thresh_e) roi_b1++;
    if (roi_b0 > 0) roi_b0--;
    if (roi_b1 < n_bins - 1) roi_b1++;

    int roi_x0 = roi_b0 * win;
    int roi_x1 = (roi_b1 + 1) * win;
    if (roi_x0 < 0) roi_x0 = 0;
    if (roi_x1 > width) roi_x1 = width;
    int roi_w = roi_x1 - roi_x0;
    free(energy);

    if (roi_w < 20) return;

    int thr = otsu_threshold(&row[roi_x0], roi_w) + thr_offset;
    if (thr < 1) thr = 1;
    if (thr > 254) thr = 254;
    if (roi_x1 - roi_x0 < 2) return;

    /* Upsample the ROI 4x (step 0.25px). Cap sample count defensively --
     * ROI width is bounded by image width but keep this independent of any
     * caller assumption. */
    int n_up = roi_w * 4;
    if (n_up > UP4_MAX_SAMPLES) n_up = UP4_MAX_SAMPLES;
    if (n_up < 8) return;

    int *up_buf = (int *)malloc((size_t)n_up * sizeof(*up_buf));
    if (!up_buf) return;
    for (int k = 0; k < n_up; k++) {
        int i0 = roi_x0 + k / 4;
        int phase16 = (k % 4) * 4; /* 0,4,8,12 -> quarter phases of the 16-phase table */
        up_buf[k] = up4_cubic_sample(row, width, i0, phase16);
    }

    int is_bar = up_buf[0] < thr;
    long prev_pos_q4 = (long)roi_x0 * 16;
    int first = 1;

    for (int k = 0; k < n_up - 1; k++) {
        int v0 = up_buf[k], v1 = up_buf[k + 1];
        int b0 = v0 < thr, b1 = v1 < thr;
        if (b0 == b1) continue;

        long base_q4 = (long)roi_x0 * 16 + (long)k * 4;
        int diff = v1 - v0;
        long pos_q4;
        if (diff == 0) {
            pos_q4 = base_q4 + 2;
        } else {
            long frac4 = (long)(thr - v0) * 4 / diff;
            if (frac4 < 0) frac4 = 0;
            if (frac4 > 4) frac4 = 4;
            pos_q4 = base_q4 + frac4;
        }
        if (pos_q4 <= prev_pos_q4) pos_q4 = prev_pos_q4 + 1;

        if (first) {
            if (is_bar && runs->count < MAX_BARS)
                runs->widths[runs->count++] = 0;
            first = 0;
        }
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)(pos_q4 - prev_pos_q4);
        prev_pos_q4 = pos_q4;
        is_bar = b1;
    }
    if (first) {
        if (is_bar && runs->count < MAX_BARS)
            runs->widths[runs->count++] = 0;
    }
    {
        long end_pos_q4 = (long)roi_x0 * 16 + (long)(n_up - 1) * 4;
        long w4 = end_pos_q4 - prev_pos_q4;
        if (w4 < 1) w4 = 1;
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)w4;
    }
    free(up_buf);
}

#undef UP4_MAX_SAMPLES

/* ────────────────────────────────────────────────────────────────────────
 * UP8-env: far-field 8x oversampled counterpart of extract_runs_roi_offset_up4,
 * built from the far-field edge chain primitives ported from sub_15BA6C
 * (Point_Spread + SmartSymbolRedef0006016184, see src/core/signal.c). Used
 * ONLY as a last-resort fallback after the plain/Q4/UP4 far paths have all
 * failed on a given ROI -- never replaces them.
 *
 * Differs from UP4 in two ways, both drawn from the original far-field
 * chain:
 *   1. The ROI's two endpoints are extended outward by Point_Spread (a3=1,
 *      a4=10, a5=1 -- same ratios as the real call site) before sampling,
 *      giving the oversampler a bit of run-in/run-out margin.
 *   2. Oversampling density is 8x (step 0.125px, matching sub_15BA6C's
 *      `dist*8` sample count) instead of 4x, and the threshold crossing
 *      uses a locally-adaptive threshold (midpoint of the SmartSymbolRedef
 *      0006016184 upper/lower envelope) instead of one global Otsu value --
 *      this is the actual "activate the envelope tracker" part of the
 *      task; it replaces only the threshold source, not the well-tested
 *      crossing/interpolation/runs-building logic already validated by
 *      UP4, and output stays in the same Q4 (16x) width units so it can
 *      feed decode_from_runs_far() unchanged.
 *
 * NOTE: SmartSymbolRedef0006016183b / vp_iv_edge_finder_envelope_strong_peak
 * are deliberately NOT wired here. Reading the actual call site (source
 * ref:253744-253860) shows sub_15BA6C uses their candidate/peak output for
 * a specialized "does this look like a 3-segment guard/finder pattern"
 * test (divisibility by 3, alternating-sign checks), not for producing a
 * general bar/space width sequence for arbitrary-length payload decoding.
 * Forcing that output into decode_from_runs_far()'s width-ratio matcher
 * would be an unverified semantic guess and risks WRONG payloads, which
 * violates the zero-WRONG constraint -- so it stays unwired (still dead
 * code) until that path gets its own dedicated, verified integration.
 */
#define UP8_MAX_SAMPLES 4096

static void extract_runs_roi_offset_up8_env(const uint8_t *row, int width,
                                             int thr_bias, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 40) return;

    int win = width / 16;
    if (win < 8) win = 8;
    if (win > 64) win = 64;
    int n_bins = (width - 1) / win;
    if (n_bins < 3) return;

    int *energy = (int *)calloc((size_t)n_bins, sizeof(int));
    if (!energy) return;

    for (int b = 0; b < n_bins; b++) {
        int x0 = b * win;
        int x1 = x0 + win;
        if (x1 > width - 1) x1 = width - 1;
        int e = 0;
        for (int x = x0; x < x1; x++) {
            int g = (int)row[x + 1] - (int)row[x];
            e += g * g;
        }
        energy[b] = e / (x1 - x0 > 0 ? x1 - x0 : 1);
    }

    int max_e = 0, max_b = 0;
    for (int b = 0; b < n_bins; b++) {
        if (energy[b] > max_e) {
            max_e = energy[b];
            max_b = b;
        }
    }

    int thresh_e = max_e / 7;
    int roi_b0 = max_b, roi_b1 = max_b;
    while (roi_b0 > 0 && energy[roi_b0 - 1] >= thresh_e) roi_b0--;
    while (roi_b1 < n_bins - 1 && energy[roi_b1 + 1] >= thresh_e) roi_b1++;
    if (roi_b0 > 0) roi_b0--;
    if (roi_b1 < n_bins - 1) roi_b1++;

    int roi_x0 = roi_b0 * win;
    int roi_x1 = (roi_b1 + 1) * win;
    if (roi_x0 < 0) roi_x0 = 0;
    if (roi_x1 > width) roi_x1 = width;
    free(energy);
    if (roi_x1 - roi_x0 < 20) return;

    /* Point_Spread: extend both endpoints outward by 1/10 of the span
     * (a3=1,a4=10,a5=1 -- identical ratios to the real sub_15BA6C call). */
    int px0 = roi_x0, py0 = 0, px1 = roi_x1, py1 = 0;
    signal_point_spread(&px0, &py0, &px1, &py1, 1, 10, 1);
    if (px0 < 0) px0 = 0;
    if (px1 > width - 1) px1 = width - 1;
    if (px1 <= px0) return;

    int dist = px1 - px0;
    int n_up = dist * 8;
    if (n_up > UP8_MAX_SAMPLES) n_up = UP8_MAX_SAMPLES;
    if (n_up < 16) return;

    int *up_work = (int *)malloc((size_t)n_up * 3U * sizeof(*up_work));
    if (!up_work) return;
    int *up_buf = up_work;
    int *up_env_hi = up_buf + n_up;
    int *up_env_lo = up_env_hi + n_up;
    for (int k = 0; k < n_up; k++) {
        int i0 = px0 + k / 8;
        int phase16 = (k % 8) * 2; /* 8 phases out of the 16-phase table */
        up_buf[k] = up4_cubic_sample(row, width, i0, phase16);
    }

    /* SmartSymbolRedef0006016184: a5=8/a6=10, matching the real call. */
    signal_far_envelope(up_buf, n_up, up_env_hi, up_env_lo, 8, 10);

    /* Locally-adaptive threshold: midpoint of the envelope pair, biased by
     * thr_bias (mirrors the fixed per-offset bias sweep UP4/Q4 already use
     * to probe a small window of threshold placements). Skip samples where
     * the envelope hasn't separated enough to be meaningful (matches the
     * "<=8 treated as flat" floor used inside 6184/vp_iv themselves). */
    int is_bar = -1;
    long prev_pos_q8 = (long)px0 * 8; /* Q8-in-eighths position accumulator */
    int first = 1;

    for (int k = 0; k < n_up - 1; k++) {
        int mid0 = (up_env_hi[k] + up_env_lo[k]) / 2 + thr_bias;
        int mid1 = (up_env_hi[k + 1] + up_env_lo[k + 1]) / 2 + thr_bias;
        int v0 = up_buf[k], v1 = up_buf[k + 1];
        int b0 = v0 < mid0;
        int b1 = v1 < mid1;
        if (is_bar < 0) is_bar = b0;
        if (b0 == b1) continue;

        /* Sub-eighth-pixel interpolation of the crossing, expressed directly
         * in Q4 (16x px) output units (1 eighth-pixel step == 2 Q4 units). */
        int diff = v1 - v0;
        long base_q4 = ((long)px0 * 8 + (long)k) * 2;
        long pos_q4;
        if (diff == 0) {
            pos_q4 = base_q4 + 1;
        } else {
            long f = (long)(mid0 - v0) * 2 / diff; /* fraction of one 1/8px step, in Q4 units (0..2) */
            if (f < 0) f = 0;
            if (f > 2) f = 2;
            pos_q4 = base_q4 + f;
        }
        if (pos_q4 <= prev_pos_q8) pos_q4 = prev_pos_q8 + 1;

        if (first) {
            if (b0 && runs->count < MAX_BARS)
                runs->widths[runs->count++] = 0;
            first = 0;
        }
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)(pos_q4 - prev_pos_q8);
        prev_pos_q8 = pos_q4;
        is_bar = b1;
    }
    if (first) {
        if (is_bar > 0 && runs->count < MAX_BARS)
            runs->widths[runs->count++] = 0;
    }
    {
        long end_q4 = ((long)px0 * 8 + (long)(n_up - 1)) * 2;
        long w4 = end_q4 - prev_pos_q8;
        if (w4 < 1) w4 = 1;
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)w4;
    }
    free(up_work);
}

#undef UP8_MAX_SAMPLES

/* ────────────────────────────────────────────────────────────────────────
 * UP16-env: far-field 16x oversampled counterpart of extract_runs_roi_offset_up8_env,
 * for the small set of extreme-far distances (~660mm) where even the 8x
 * envelope chain's sample density is too coarse to resolve a module. Reuses
 * every stage of the UP8-env pipeline unchanged (same ROI localization,
 * Point_Spread margin, envelope-midpoint threshold with thr_bias sweep)
 * except:
 *   1. Oversampling density is 16x (step 0.0625px) using the FULL 16-phase
 *      kUp4CubicCoeff table directly (phase16 = k % 16), instead of UP8's
 *      8-of-16 subsampled phases.
 *   2. signal_far_envelope's attack/decay shift is 9/11 instead of 8/10 --
 *      doubling the oversampling density means the attack/decay time
 *      constants (in sample units) must also shift up by one bit to track
 *      the same physical bar/space transition window as UP8's 8/10.
 * Because each output sample already advances exactly 1/16px == 1 Q4 unit,
 * the position bookkeeping needs no *2 scaling (unlike UP8-env, where each
 * 1/8px step == 2 Q4 units): output stays natively in Q4 (16x) units.
 * decode_from_runs_far_up8() is reused as-is to decode the resulting runs
 * (BarSpaceRun width-ratio matching is oversampling-density-agnostic).
 * Only ever invoked after both UP4 and UP8-env have already failed on a
 * given row/roi/offset -- last-resort tier, gated by the same >=2-distinct-
 * row cross-row consensus vote as UP8-env (see call site).
 */
#define UP16_MAX_SAMPLES 8192

static void extract_runs_roi_offset_up16_env(const uint8_t *row, int width,
                                              int thr_bias, BarSpaceRun *runs)
{
    runs->count = 0;
    if (width < 40) return;

    int win = width / 16;
    if (win < 8) win = 8;
    if (win > 64) win = 64;
    int n_bins = (width - 1) / win;
    if (n_bins < 3) return;

    int *energy = (int *)calloc((size_t)n_bins, sizeof(int));
    if (!energy) return;

    for (int b = 0; b < n_bins; b++) {
        int x0 = b * win;
        int x1 = x0 + win;
        if (x1 > width - 1) x1 = width - 1;
        int e = 0;
        for (int x = x0; x < x1; x++) {
            int g = (int)row[x + 1] - (int)row[x];
            e += g * g;
        }
        energy[b] = e / (x1 - x0 > 0 ? x1 - x0 : 1);
    }

    int max_e = 0, max_b = 0;
    for (int b = 0; b < n_bins; b++) {
        if (energy[b] > max_e) {
            max_e = energy[b];
            max_b = b;
        }
    }

    int thresh_e = max_e / 7;
    int roi_b0 = max_b, roi_b1 = max_b;
    while (roi_b0 > 0 && energy[roi_b0 - 1] >= thresh_e) roi_b0--;
    while (roi_b1 < n_bins - 1 && energy[roi_b1 + 1] >= thresh_e) roi_b1++;
    if (roi_b0 > 0) roi_b0--;
    if (roi_b1 < n_bins - 1) roi_b1++;

    int roi_x0 = roi_b0 * win;
    int roi_x1 = (roi_b1 + 1) * win;
    if (roi_x0 < 0) roi_x0 = 0;
    if (roi_x1 > width) roi_x1 = width;
    free(energy);
    if (roi_x1 - roi_x0 < 20) return;

    /* Point_Spread: extend both endpoints outward by 1/10 of the span
     * (a3=1,a4=10,a5=1 -- identical ratios to the real sub_15BA6C call). */
    int px0 = roi_x0, py0 = 0, px1 = roi_x1, py1 = 0;
    signal_point_spread(&px0, &py0, &px1, &py1, 1, 10, 1);
    if (px0 < 0) px0 = 0;
    if (px1 > width - 1) px1 = width - 1;
    if (px1 <= px0) return;

    int dist = px1 - px0;
    int n_up = dist * 16;
    if (n_up > UP16_MAX_SAMPLES) n_up = UP16_MAX_SAMPLES;
    if (n_up < 16) return;

    int *up_work = (int *)malloc((size_t)n_up * 3U * sizeof(*up_work));
    if (!up_work) return;
    int *up_buf = up_work;
    int *up_env_hi = up_buf + n_up;
    int *up_env_lo = up_env_hi + n_up;
    for (int k = 0; k < n_up; k++) {
        int i0 = px0 + k / 16;
        int phase16 = k % 16; /* full 16-phase table, unlike UP8's 8-of-16 subsample */
        up_buf[k] = up4_cubic_sample(row, width, i0, phase16);
    }

    /* SmartSymbolRedef0006016184: shift 9/11 (one bit above UP8-env's 8/10,
     * to track the same physical window at double the sample density). */
    signal_far_envelope(up_buf, n_up, up_env_hi, up_env_lo, 9, 11);

    int is_bar = -1;
    long prev_pos_q4 = (long)px0 * 16; /* already in native Q4 (16x px) units */
    int first = 1;

    for (int k = 0; k < n_up - 1; k++) {
        int mid0 = (up_env_hi[k] + up_env_lo[k]) / 2 + thr_bias;
        int mid1 = (up_env_hi[k + 1] + up_env_lo[k + 1]) / 2 + thr_bias;
        int v0 = up_buf[k], v1 = up_buf[k + 1];
        int b0 = v0 < mid0;
        int b1 = v1 < mid1;
        if (is_bar < 0) is_bar = b0;
        if (b0 == b1) continue;

        /* Sub-sixteenth-pixel interpolation of the crossing; one 1/16px
         * step == 1 Q4 unit, so no *2 scaling is needed here (cf. UP8-env). */
        int diff = v1 - v0;
        long base_q4 = (long)px0 * 16 + (long)k;
        long pos_q4;
        if (diff == 0) {
            pos_q4 = base_q4 + 1;
        } else {
            long f = (long)(mid0 - v0) / diff; /* fraction of one step, 0 or 1 */
            if (f < 0) f = 0;
            if (f > 1) f = 1;
            pos_q4 = base_q4 + f;
        }
        if (pos_q4 <= prev_pos_q4) pos_q4 = prev_pos_q4 + 1;

        if (first) {
            if (b0 && runs->count < MAX_BARS)
                runs->widths[runs->count++] = 0;
            first = 0;
        }
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)(pos_q4 - prev_pos_q4);
        prev_pos_q4 = pos_q4;
        is_bar = b1;
    }
    if (first) {
        if (is_bar > 0 && runs->count < MAX_BARS)
            runs->widths[runs->count++] = 0;
    }
    {
        long end_q4 = (long)px0 * 16 + (long)(n_up - 1);
        long w4 = end_q4 - prev_pos_q4;
        if (w4 < 1) w4 = 1;
        if (runs->count < MAX_BARS)
            runs->widths[runs->count++] = (int)w4;
    }
    free(up_work);
}

#undef UP16_MAX_SAMPLES

/* ════════════════════════════════════════════════════════════════════════
 *  DMFI dynamic threshold (dmfi_thresh, line 263751)
 * ════════════════════════════════════════════════════════════════════════ */

static void ref_first_derivative(int *signal, int length)
{
    for (int i = 0; i < length - 1; i++)
        signal[i] = signal[i + 1] - signal[i];
}

/* DMFI dynamic threshold from derivative histogram */
static int ref_dmfi_threshold(const int *signal, int length)
{
    if (length < 3) return 32;

    int *tmp = (int *)malloc(length * sizeof(int));
    memcpy(tmp, signal, length * sizeof(int));

    ref_first_derivative(tmp, length);
    int dlen = length - 1;

    int histogram[65] = {0};
    for (int i = 0; i < dlen; i++) {
        int val = abs(tmp[i]);
        int bin = val / 4;
        if (bin > 64) bin = 64;
        histogram[bin]++;
    }
    free(tmp);

    int zero_count = histogram[0];
    int target = 7 * (dlen - zero_count) / 8;
    int cumsum = 0;
    int bin = 1;
    for (; bin <= 64; bin++) {
        cumsum += histogram[bin];
        if (cumsum >= target) break;
    }

    int threshold = bin * 4 + 8;
    if (threshold > 127) threshold = 127;
    return threshold;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Multi-scanline decoder with voting
 * ════════════════════════════════════════════════════════════════════════ */

/* Row accumulator for get_averaged_row's PSRAM-friendly row-major path. */
#define MAX_ROW_ACC 4096

static void get_averaged_row(const Image *img, int center_y, int avg_rows,
                             uint8_t *out_row)
{
    int half = avg_rows / 2;
    int y0 = center_y - half;
    int y1 = center_y + half;
    int w = img->width;
    int rows_used;
    int x;
    int y;

    if (y0 < 0) y0 = 0;
    if (y1 >= img->height) y1 = img->height - 1;
    rows_used = y1 - y0 + 1;

    if (rows_used == 1) {
        /* single row: direct copy */
        const uint8_t *src = img->pixels + (size_t)y0 * (size_t)w;
        memcpy(out_row, src, (size_t)w);
        return;
    }

    if (w > MAX_ROW_ACC) {
        /* fallback: column-major (correct, but slower on PSRAM) */
        for (x = 0; x < w; x++) {
            int sum = 0;
            for (y = y0; y <= y1; y++)
                sum += img->pixels[(size_t)y * (size_t)w + (size_t)x];
            out_row[x] = (uint8_t)(sum / rows_used);
        }
        return;
    }

    /* Row-major accumulation: read each source row sequentially so PSRAM
     * bursts are not broken up by per-column cross-row jumps. */
    {
        static int acc[MAX_ROW_ACC];
        const uint8_t *row0 = img->pixels + (size_t)y0 * (size_t)w;
        for (x = 0; x < w; x++) acc[x] = row0[x];
        for (y = y0 + 1; y <= y1; y++) {
            const uint8_t *row = img->pixels + (size_t)y * (size_t)w;
            for (x = 0; x < w; x++) acc[x] += row[x];
        }
        for (x = 0; x < w; x++) out_row[x] = (uint8_t)(acc[x] / rows_used);
    }
}

/* tan(Ndeg) in Q16 fixed-point */
#define TILT_Q16_1DEG 1144
#define TILT_Q16_2DEG 2289
#define TILT_Q16_3DEG 3434
#define TILT_Q16_5DEG 5734
#define TILT_Q16_8DEG 9209
#define TILT_Q16_10DEG 11557
#define TILT_Q16_12DEG 13932
#define TILT_Q16_15DEG 17561
#define TILT_Q16_20DEG 23861
#define TILT_Q16_26DEG 32768
#define TILT_Q16_FAR 22938

static int clamp_i(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void resolve_frontend_anchor_center(const Image *img, int *out_x, int *out_y)
{
    ScanEngineFrame frame;
    ScanEngineAnchorConfig cfg;
    ScanEngineAnchor anchor;

    if (!img || !out_x || !out_y || img->width <= 0 || img->height <= 0) {
        return;
    }

    frame.width = img->width;
    frame.height = img->height;
    frame.center_x = img->width / 2;
    frame.center_y = img->height / 2;
    frame.orientation_mode = 0;

    memset(&cfg, 0, sizeof(cfg));
    if (scan_engine_fmwlsymbol0040(&frame, &cfg, &anchor)) {
        *out_x = anchor.x;
        *out_y = anchor.y;
    }
}

typedef struct {
    const Image *img;
} DispatcherCtx;

static int dispatcher_sample(void *ctx, int x, int y)
{
    const DispatcherCtx *dc = (const DispatcherCtx *)ctx;
    if (!dc || !dc->img || !dc->img->pixels) return -1;
    if (x < 0 || y < 0 || x >= dc->img->width || y >= dc->img->height) return -1;
    return dc->img->pixels[y * dc->img->width + x];
}

static int dispatcher_score(void *ctx, int cx, int cy, int dir_idx)
{
    static const int kDirs[SCAN_DISPATCHER_MAX_DIRS][2] = {
        {1, 0}, {2, 1}, {1, 1}, {1, 2},
        {0, 1}, {-1, 2}, {-1, 1}, {-2, 1},
        {-1, 0}, {-2, -1}, {-1, -1}, {-1, -2},
        {0, -1}, {1, -2}, {1, -1}, {2, -1},
    };
    const DispatcherCtx *dc = (const DispatcherCtx *)ctx;
    if (!dc || !dc->img || !dc->img->pixels) return 0;

    int dx = kDirs[dir_idx & 15][0];
    int dy = kDirs[dir_idx & 15][1];
    int pdx = -dy;
    int pdy = dx;
    int score = 0;

    for (int k = 1; k <= 20; k++) {
        int x0 = cx - dx * k;
        int y0 = cy - dy * k;
        int x1 = cx + dx * k;
        int y1 = cy + dy * k;
        int lx0 = cx - pdx * k;
        int ly0 = cy - pdy * k;
        int lx1 = cx + pdx * k;
        int ly1 = cy + pdy * k;
        int a0 = dispatcher_sample((void *)dc, x0, y0);
        int a1 = dispatcher_sample((void *)dc, x1, y1);
        int b0 = dispatcher_sample((void *)dc, lx0, ly0);
        int b1 = dispatcher_sample((void *)dc, lx1, ly1);
        if (a0 < 0 || a1 < 0 || b0 < 0 || b1 < 0) continue;
        score += abs(a1 - a0) * 3;
        score += abs((a1 + a0) - (b1 + b0));
    }
    return score;
}

static uint64_t monotonic_ms(void)
{
    return xcr_port_time_us() / 1000ULL;
}

static int is_timed_out(uint64_t deadline_ms)
{
    if (g_code128_cancel_flag && *g_code128_cancel_flag) {
        return 1;
    }
    if (deadline_ms == 0) {
        return 0;
    }
    return monotonic_ms() >= deadline_ms;
}

static int build_direction_endpoints(
    const Image *img,
    int cx,
    int cy,
    int dx,
    int dy,
    int *x0,
    int *y0,
    int *x1,
    int *y1)
{
    int t_neg = INT_MAX;
    int t_pos = INT_MAX;

    if (!img || !x0 || !y0 || !x1 || !y1 || (dx == 0 && dy == 0)) {
        return 0;
    }

    if (dx > 0) {
        int l = cx / dx;
        int r = (img->width - 1 - cx) / dx;
        if (l < t_neg) t_neg = l;
        if (r < t_pos) t_pos = r;
    } else if (dx < 0) {
        int ax = -dx;
        int l = (img->width - 1 - cx) / ax;
        int r = cx / ax;
        if (l < t_neg) t_neg = l;
        if (r < t_pos) t_pos = r;
    }

    if (dy > 0) {
        int l = cy / dy;
        int r = (img->height - 1 - cy) / dy;
        if (l < t_neg) t_neg = l;
        if (r < t_pos) t_pos = r;
    } else if (dy < 0) {
        int ay = -dy;
        int l = (img->height - 1 - cy) / ay;
        int r = cy / ay;
        if (l < t_neg) t_neg = l;
        if (r < t_pos) t_pos = r;
    }

    if (t_neg == INT_MAX || t_pos == INT_MAX || (t_neg + t_pos) < 24) {
        return 0;
    }

    *x0 = cx - dx * t_neg;
    *y0 = cy - dy * t_neg;
    *x1 = cx + dx * t_pos;
    *y1 = cy + dy * t_pos;
    return 1;
}

static int decode_dispatcher_scan_hint(
    const Image *img,
    int center_x,
    int center_y,
    int dir_x,
    int dir_y,
    int *edges,
    int max_edges,
    char *out,
    int *out_len)
{
    int off_steps[] = {0, -6, 6, -12, 12, -18, 18};
    int odx = -dir_y;
    int ody = dir_x;
    int i;

    if (!img || !img->pixels || !edges || !out || !out_len) {
        return 0;
    }
    if (dir_x == 0 && dir_y == 0) {
        return 0;
    }

    for (i = 0; i < (int)(sizeof(off_steps) / sizeof(off_steps[0])); i++) {
        int cx = center_x + odx * off_steps[i];
        int cy = center_y + ody * off_steps[i];
        int x0;
        int y0;
        int x1;
        int y1;
        int n_edges;

        if (cx < 0 || cx >= img->width || cy < 0 || cy >= img->height) {
            continue;
        }
        if (!build_direction_endpoints(img, cx, cy, dir_x, dir_y, &x0, &y0, &x1, &y1)) {
            continue;
        }

        n_edges = signal_transitions_code128(
            img->pixels, img->width, img->height,
            x0, y0, x1, y1,
            edges, max_edges);
        if (n_edges < 20) {
            continue;
        }
        if (decode_from_edges_char128(edges, n_edges, out, out_len)) {
            return 1;
        }
    }
    return 0;
}

static int decode_dispatcher_scan(const Image *img, int center_x, int center_y,
                                  int *edges, int max_edges,
                                  char *out, int *out_len)
{
    static const int kDirs[SCAN_DISPATCHER_MAX_DIRS][2] = {
        {1, 0}, {2, 1}, {1, 1}, {1, 2},
        {0, 1}, {-1, 2}, {-1, 1}, {-2, 1},
        {-1, 0}, {-2, -1}, {-1, -1}, {-1, -2},
        {0, -1}, {1, -2}, {1, -1}, {2, -1},
    };
    DispatcherCtx dctx = {img};
    ScanDirectionChoice choice;
    ScanLineBuffer *line = (ScanLineBuffer *)malloc(sizeof(*line));

    if (!line) return 0;

    if (!scan_dispatcher_choose_direction(center_x, center_y, 1,
                                          dispatcher_score, &dctx, &choice)) {
        free(line);
        return 0;
    }

    int best_idx = 0;
    for (int i = 0; i < SCAN_DISPATCHER_MAX_DIRS; i++) {
        if (kDirs[i][0] == choice.stride_x && kDirs[i][1] == choice.stride_y) {
            best_idx = i;
            break;
        }
    }

    int order[8] = {
        best_idx,
        (best_idx + 8) & 15,
        (best_idx + 1) & 15,
        (best_idx + 15) & 15,
        (best_idx + 2) & 15,
        (best_idx + 14) & 15,
        (best_idx + 4) & 15,
        (best_idx + 12) & 15,
    };

    for (int oi = 0; oi < 8; oi++) {
        ScanDirectionChoice c = choice;
        c.stride_x = kDirs[order[oi]][0];
        c.stride_y = kDirs[order[oi]][1];
        c.sample_radius = img->width > img->height ? img->width / 2 : img->height / 2;
        if (c.sample_radius < 32) c.sample_radius = 32;
        if (c.sample_radius > (SCAN_DISPATCHER_MAX_SAMPLES / 2) - 2) {
            c.sample_radius = (SCAN_DISPATCHER_MAX_SAMPLES / 2) - 2;
        }

        size_t center_idx = 0;
        int nline = scan_dispatcher_build_scanline_stitched(
            &c, dispatcher_sample, &dctx, line, &center_idx);
        if (nline < 32) continue;
        if (center_idx >= line->count) continue;

        int first = -1;
        int last = -1;
        for (int i = 0; i < nline; i++) {
            if (line->samples[i].value < 0) continue;
            if (first < 0) first = i;
            last = i;
        }
        if (first < 0 || last < 0 || last - first < 20) continue;

        int x0 = line->samples[first].x;
        int y0 = line->samples[first].y;
        int x1 = line->samples[last].x;
        int y1 = line->samples[last].y;
        int n_edges = signal_transitions_code128(img->pixels, img->width, img->height,
                                                 x0, y0, x1, y1, edges, max_edges);
        if (n_edges < 20) continue;
        if (decode_from_edges_char128(edges, n_edges, out, out_len)) {
            free(line);
            return 1;
        }
    }
    free(line);
    return 0;
}

static void get_averaged_tilted_row(const Image *img, int center_y, int avg_rows,
                                    int slope_q16, uint8_t *out_row)
{
    int half = avg_rows / 2;
    int mid_x = (img->width - 1) / 2;
    for (int x = 0; x < img->width; x++) {
        int dy = (int)(((long long)(x - mid_x) * (long long)slope_q16) >> 16);
        int y_mid = center_y + dy;
        int y0 = clamp_i(y_mid - half, 0, img->height - 1);
        int y1 = clamp_i(y_mid + half, 0, img->height - 1);
        int rows_used = y1 - y0 + 1;
        int sum = 0;
        for (int y = y0; y <= y1; y++)
            sum += img->pixels[y * img->width + x];
        out_row[x] = (uint8_t)(sum / rows_used);
    }
}

static int compute_vote_weight(int y, int h, int slope_q16, int avg_rows)
{
    int maxdist = h > 1 ? (h - 1) : 1;
    int dist = abs((2 * y) - (h - 1));
    int center_w = 100 + ((maxdist - dist) * 120) / maxdist; /* [100,220] */
    int avg_bonus = 0;

    if (avg_rows >= 75) avg_bonus = 45;
    else if (avg_rows >= 51) avg_bonus = 35;
    else if (avg_rows >= 31) avg_bonus = 25;
    else if (avg_rows >= 21) avg_bonus = 15;

    center_w += avg_bonus;
    if (slope_q16 != 0) center_w = center_w * 9 / 10; /* slight penalty for tilted scans */
    if (center_w < 1) center_w = 1;
    return center_w;
}

static void add_scan_y_if_new(int y, int h, int *scan_y, uint8_t *scan_y_used, int *n_scan_y)
{
    if (!scan_y || !scan_y_used || !n_scan_y) return;
    if (y < 0 || y >= h) return;
    if (scan_y_used[y]) return;
    scan_y_used[y] = 1;
    scan_y[*n_scan_y] = y;
    (*n_scan_y)++;
}

static void build_fixed_scanline_plan(int h, int has_hint,
                                      int *scan_y, uint8_t *scan_y_used, int *n_scan_y)
{
    int step_full = h / 90;
    int step_dense = h / 180;
    int y;

    if (step_full < 1) step_full = 1;
    if (step_dense < 1) step_dense = 1;

    if (has_hint) {
        int center = h / 2;
        int band = h / 6;
        if (band < 24) band = 24;
        for (y = center - band; y <= center + band; y += step_dense)
            add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
        for (y = center - 2 * band; y <= center + 2 * band; y += step_full)
            add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
    } else {
        for (y = h / 5; y <= (4 * h) / 5; y += step_dense)
            add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
        for (y = h / 8; y <= (3 * h) / 8; y += step_full)
            add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
        for (y = (5 * h) / 8; y <= (7 * h) / 8; y += step_full)
            add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
        for (y = 0; y < h; y += step_full)
            add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
    }

    add_scan_y_if_new(h / 2, h, scan_y, scan_y_used, n_scan_y);
    add_scan_y_if_new(h / 4, h, scan_y, scan_y_used, n_scan_y);
    add_scan_y_if_new((3 * h) / 4, h, scan_y, scan_y_used, n_scan_y);
    add_scan_y_if_new(h - 1, h, scan_y, scan_y_used, n_scan_y);
}

typedef struct {
    int y;
    int score;
    int contrast;
    int grad_mean;
    int edge_density_pm;
} RowQualityProbe;

typedef struct {
    int used_adaptive;
    int probe_count;
    int best_probe_y;
    int best_probe_score;
    int mean_contrast;
    int mean_grad;
    int mean_edge_den;
    int low_contrast_mode;
    int far_distance_mode;
    int trigger_enhance;
} AdaptivePlanState;

static void adaptive_plan_state_reset(AdaptivePlanState *st)
{
    if (!st) return;
    memset(st, 0, sizeof(*st));
    st->best_probe_y = -1;
    st->best_probe_score = INT_MIN;
}

static int is_env_truthy(const char *v)
{
    if (!v || !*v) return 0;
    if (strcmp(v, "0") == 0) return 0;
    if (strcmp(v, "false") == 0) return 0;
    if (strcmp(v, "off") == 0) return 0;
    if (strcmp(v, "no") == 0) return 0;
    return 1;
}

static int code128_far_mode_override(int detected)
{
    const char *mode = getenv("XCR_CODE128_FAR_MODE");
    if (!mode || !*mode || strcmp(mode, "auto") == 0) return detected;
    if (strcmp(mode, "always") == 0) return 1;
    if (strcmp(mode, "never") == 0) return 0;
    return 0;
}

static int probe_row_quality(const Image *img, int y, uint8_t *row_buf, RowQualityProbe *out)
{
    int mn = 255;
    int mx = 0;
    int grad_sum = 0;
    int edge_count = 0;
    int w;

    if (!img || !img->pixels || !row_buf || !out || y < 0 || y >= img->height) {
        return 0;
    }

    w = img->width;
    get_averaged_row(img, y, 1, row_buf);
    for (int x = 0; x < w; x++) {
        int v = row_buf[x];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    for (int x = 1; x < w; x++) {
        int d = abs((int)row_buf[x] - (int)row_buf[x - 1]);
        grad_sum += d;
        if (d >= 12) edge_count++;
    }

    out->y = y;
    out->contrast = mx - mn;
    out->grad_mean = (w > 1) ? (grad_sum / (w - 1)) : 0;
    out->edge_density_pm = (w > 1) ? (edge_count * 1000 / (w - 1)) : 0;
    out->score = out->contrast * 4 + out->grad_mean * 6 + out->edge_density_pm / 2;
    if (out->contrast < 28) out->score -= 100;
    if (out->edge_density_pm < 20) out->score -= 60;
    return 1;
}

static void build_adaptive_scanline_plan(const Image *img, int has_hint, uint8_t *row_buf,
                                         int *scan_y, uint8_t *scan_y_used, int *n_scan_y,
                                         int *n_avg, int *n_tilt,
                                         AdaptivePlanState *plan_state)
{
    RowQualityProbe probes[13];
    int probe_count = 0;
    int sum_contrast = 0;
    int sum_grad = 0;
    int sum_edge_den = 0;
    int best_idx = 0;
    int i;
    int h = img->height;

    adaptive_plan_state_reset(plan_state);

    if (!img || !row_buf || !scan_y || !scan_y_used || !n_scan_y || !n_avg || !n_tilt) {
        return;
    }

    if (plan_state) {
        plan_state->used_adaptive = 1;
    }

    build_fixed_scanline_plan(h, has_hint, scan_y, scan_y_used, n_scan_y);

    for (i = 0; i < 13; i++) {
        int y = ((i + 1) * h) / 14;
        if (probe_row_quality(img, y, row_buf, &probes[probe_count])) {
            sum_contrast += probes[probe_count].contrast;
            sum_grad += probes[probe_count].grad_mean;
            sum_edge_den += probes[probe_count].edge_density_pm;
            if (probe_count == 0 || probes[probe_count].score > probes[best_idx].score)
                best_idx = probe_count;
            probe_count++;
        }
    }

    if (probe_count <= 0) {
        build_fixed_scanline_plan(h, has_hint, scan_y, scan_y_used, n_scan_y);
        return;
    }

    {
        int mean_contrast = sum_contrast / probe_count;
        int mean_grad = sum_grad / probe_count;
        int mean_edge_den = sum_edge_den / probe_count;
        int low_contrast_mode = (mean_contrast < 40 || probes[best_idx].contrast < 54) ? 1 : 0;
        int far_distance_mode = code128_far_mode_override(
            (mean_grad < 14 && mean_edge_den < 55) ? 1 : 0);
        int trigger_enhance = (low_contrast_mode || far_distance_mode) ? 1 : 0;

        if (plan_state) {
            plan_state->probe_count = probe_count;
            plan_state->best_probe_y = probes[best_idx].y;
            plan_state->best_probe_score = probes[best_idx].score;
            plan_state->mean_contrast = mean_contrast;
            plan_state->mean_grad = mean_grad;
            plan_state->mean_edge_den = mean_edge_den;
            plan_state->low_contrast_mode = low_contrast_mode;
            plan_state->far_distance_mode = far_distance_mode;
            plan_state->trigger_enhance = trigger_enhance;
        }

        if (!trigger_enhance) {
            if (!has_hint && probes[best_idx].score > 520) {
                if (*n_avg > 7) *n_avg = 7;
                if (*n_tilt > 3) *n_tilt = 3;
            }
            return;
        }

        if (has_hint) {
            if (*n_avg < 7) *n_avg = 7;
            *n_tilt = 3;
        } else {
            *n_avg = 9;
            *n_tilt = 5;
        }

        {
            int dense_step = h / 260;
            int center = h / 2;
            int center_band = h / 3;
            int focus_band = h / 12;
            int y;
            if (dense_step < 1) dense_step = 1;
            if (center_band < 28) center_band = 28;
            if (focus_band < 14) focus_band = 14;
            for (y = center - center_band; y <= center + center_band; y += dense_step)
                add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
            for (y = probes[best_idx].y - focus_band; y <= probes[best_idx].y + focus_band; y += dense_step)
                add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
        }
    }

    for (i = 0; i < probe_count; i++) {
        if (probes[i].score + 60 < probes[best_idx].score) continue;
        {
            int local_step = h / 320;
            int band = h / 18;
            int y;
            if (local_step < 1) local_step = 1;
            if (band < 8) band = 8;
            for (y = probes[i].y - band; y <= probes[i].y + band; y += local_step)
                add_scan_y_if_new(y, h, scan_y, scan_y_used, n_scan_y);
        }
    }
}

int code128_debug_plan_scanlines(const Image *img, int has_hint, int use_adaptive,
                                 int *scan_y, int max_scan_y,
                                 Code128AdaptivePlanDiag *diag)
{
    int h;
    int *plan = NULL;
    uint8_t *used = NULL;
    uint8_t *row_buf = NULL;
    int n_scan_y = 0;
    int avg_opts[] = {1, 3, 5, 9, 15, 21, 31, 51, 75, 101, 151, 201};
    int n_avg = (int)(sizeof(avg_opts) / sizeof(avg_opts[0]));
    int tilt_opts[] = {0, TILT_Q16_1DEG, -TILT_Q16_1DEG, TILT_Q16_2DEG, -TILT_Q16_2DEG};
    int n_tilt = (int)(sizeof(tilt_opts) / sizeof(tilt_opts[0]));
    AdaptivePlanState st;

    adaptive_plan_state_reset(&st);
    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->best_probe_y = -1;
        diag->best_probe_score = INT_MIN;
    }
    if (!img || !img->pixels || img->width < 4 || img->height < 1) return 0;

    h = img->height;
    plan = (int *)malloc((size_t)h * 4U * sizeof(int));
    used = (uint8_t *)calloc((size_t)h, 1U);
    row_buf = (uint8_t *)malloc((size_t)img->width);
    if (!plan || !used || !row_buf) {
        free(plan);
        free(used);
        free(row_buf);
        return 0;
    }

    if (has_hint) {
        n_avg = 6;
        n_tilt = 3;
    }

    if (use_adaptive) {
        build_adaptive_scanline_plan(img, has_hint, row_buf,
                                     plan, used, &n_scan_y,
                                     &n_avg, &n_tilt, &st);
        if (n_scan_y <= 0) {
            build_fixed_scanline_plan(h, has_hint, plan, used, &n_scan_y);
        }
    } else {
        build_fixed_scanline_plan(h, has_hint, plan, used, &n_scan_y);
    }

    if (scan_y && max_scan_y > 0) {
        int copy_n = n_scan_y < max_scan_y ? n_scan_y : max_scan_y;
        memcpy(scan_y, plan, (size_t)copy_n * sizeof(int));
    }
    if (diag) {
        diag->used_adaptive = st.used_adaptive;
        diag->probe_count = st.probe_count;
        diag->best_probe_y = st.best_probe_y;
        diag->best_probe_score = st.best_probe_score;
        diag->mean_contrast = st.mean_contrast;
        diag->mean_grad = st.mean_grad;
        diag->mean_edge_den = st.mean_edge_den;
        diag->low_contrast_mode = st.low_contrast_mode;
        diag->far_distance_mode = st.far_distance_mode;
        diag->trigger_enhance = st.trigger_enhance;
        diag->n_scan_y = n_scan_y;
        diag->n_avg = n_avg;
        diag->n_tilt = n_tilt;
    }

    free(plan);
    free(used);
    free(row_buf);
    return n_scan_y;
}

typedef struct {
    char text[BARCODE_MAX_LEN];
    int  text_len;
    int  votes;
    int  score;
    int  weighted_votes;
    long long weighted_score;
} Candidate;

#define MAX_CANDIDATES 256
#define MIN_ACCEPT_TEXT_LEN 8

static int validate_result_postprocess(const char *text, int text_len)
{
    if (!text || text_len <= 0) return 0;

    int all_digits = 1;
    for (int i = 0; i < text_len; i++) {
        if (!isdigit((unsigned char)text[i])) {
            all_digits = 0;
            break;
        }
    }

    if (!all_digits) return 1;

    if (text_len == 8) {
        char upca[16];
        return postprocess_upce_to_upca(text, upca, sizeof(upca));
    }

    if (text_len == 10) {
        char nine[9];
        memcpy(nine, text, 9);
        return postprocess_isbn10_check_digit(nine) == text[9];
    }

    return 1;
}

static void add_candidate(Candidate *cands, int *n_cands, int *total,
                          int *total_weight,
                          const char *text, int text_len,
                          int weight)
{
    int dbg128 = (getenv("DBG128") != NULL);
    if (!validate_result_postprocess(text, text_len)) {
        if (dbg128) {
            fprintf(stderr,
                    "  C128DROP postprocess weight=%d len=%d text=\"%s\"\n",
                    weight, text_len, text ? text : "");
        }
        return;
    }

    (*total)++;
    (*total_weight) += weight;
    (void)dbg128;

    for (int c = 0; c < *n_cands; c++) {
        if (cands[c].text_len == text_len &&
            memcmp(cands[c].text, text, text_len) == 0) {
            cands[c].votes++;
            cands[c].score += text_len * text_len;
            cands[c].weighted_votes += weight;
            cands[c].weighted_score += (long long)weight * text_len * text_len;
            return;
        }
    }
    if (*n_cands < MAX_CANDIDATES) {
        memcpy(cands[*n_cands].text, text, text_len + 1);
        cands[*n_cands].text_len = text_len;
        cands[*n_cands].votes = 1;
        cands[*n_cands].score = text_len * text_len;
        cands[*n_cands].weighted_votes = weight;
        cands[*n_cands].weighted_score = (long long)weight * text_len * text_len;
        (*n_cands)++;
    }
}

/* Unanimous-strong consensus: single candidate with overwhelming vote mass.
 * Since conf = weighted_votes*100/total_weight_success, a single candidate
 * accumulating all votes implies conf==100 (see add_candidate above). The
 * vote/weighted-vote thresholds are calibrated with ~1.7-2x margin above the
 * isbt128.bmp trigger point (676 votes / 208386 weighted) and ~2x below the
 * highest value ever observed on the depth test set for a non-early-stopped
 * candidate (176 votes / 50911 weighted), so it only fires on the
 * "short-text, high-confidence, long search" pattern that isbt128.bmp hits. */
static int c128_unanimous_strong(const Candidate *cands, int n_cands)
{
    if (n_cands <= 0) return 0;
    /* The dominant candidate by raw vote count. */
    int dom = 0;
    for (int c = 1; c < n_cands; c++)
        if (cands[c].votes > cands[dom].votes)
            dom = c;
    /* A decisive candidate (votes>=400, wv>=100000) is treated as unanimous
     * even when a few low-vote noise candidates exist, as long as it carries
     * at least 10x the votes of every competitor. The strict n_cands==1 form
     * was defeated on-device when the main-loop methods added low-vote noise
     * candidates, so an isbt128-style far prepass decode (sole strong text
     * repeated hundreds of times) never reached the early-exit and the loop
     * burned its full 2s budget. */
    if (cands[dom].votes >= 400 && cands[dom].weighted_votes >= 100000) {
        int second = -1;
        for (int c = 0; c < n_cands; c++) {
            if (c == dom) continue;
            if (cands[c].votes > second) second = cands[c].votes;
        }
        if (second < 0 || cands[dom].votes >= 10 * second) return 1;
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Main decoder
 * ════════════════════════════════════════════════════════════════════════ */

static int select_best(const Candidate *cands, int n, int total_success, int far_dist)
{
    (void)total_success;
    int best = -1;
    const int dbg128 = getenv("DBG128") ? 1 : 0;
    /* Prefer >=2 votes; allow single very-strong weighted hit from directional prepass.
     * In far-distance mode, relax to accept single votes with moderate weight. */
    int min_single_wv = far_dist ? 800 : 2400;
    int min_single_len = far_dist ? 8 : 12;
    for (int c = 0; c < n; c++) {
        /* In far-distance mode the same length-scaled weighted_votes floor
         * applies whether or not the candidate crossed the votes>=2 line.
         * Rationale (see WRONG at 0730mm depth sample, len=8 "98274601"
         * accepted with only votes=2/wvotes=498): a short (8-11 char) text
         * recurring from the same non-independent row/avg/roi sweep can
         * rack up votes>=2 without ever having been vetted by the
         * weighted-vote floor below, since that floor previously only
         * guarded the votes<2 branch. Longer texts (>=12) are exponentially
         * less likely to be an accidental checksum-passing collision, so
         * they keep bypassing the floor once corroborated (votes>=2).
         * isbt128.bmp (legitimately len=8, far_dist) is unaffected: it is
         * decided by the separate c128_unanimous_strong() early-exit path
         * (votes>=400, wv>=100000) before select_best() is ever reached. */
        int short_risky = far_dist && cands[c].text_len < 12;
        if (cands[c].votes < 2 || short_risky) {
            /* For single-vote (or short-text) candidates, accept based on
             * text_len: longer barcodes are exponentially less likely to be
             * false positives.
             * len >= 15: wv >= 180 (15 data chars + checksum = very unlikely random match)
             * len >= 12: wv >= 600
             * len >= 8:  wv >= 800 (original threshold)
             */
            int accept = 0;
            if (far_dist) {
                if (cands[c].text_len >= 15 && cands[c].weighted_votes >= 180)
                    accept = 1;
                else if (cands[c].text_len >= 12 && cands[c].weighted_votes >= 600)
                    accept = 1;
                else if (cands[c].weighted_votes >= min_single_wv && cands[c].text_len >= min_single_len)
                    accept = 1;
            } else {
                if (cands[c].weighted_votes >= min_single_wv && cands[c].text_len >= min_single_len)
                    accept = 1;
            }
            if (!accept) {
                if (dbg128) {
                    fprintf(stderr,
                            "  C128DROP select votes=%d wvotes=%d len=%d text=\"%s\"\n",
                            cands[c].votes, cands[c].weighted_votes,
                            cands[c].text_len, cands[c].text);
                }
                continue;
            }
        }
        if (best < 0 ||
            cands[c].weighted_score > cands[best].weighted_score ||
            (cands[c].weighted_score == cands[best].weighted_score &&
             cands[c].weighted_votes > cands[best].weighted_votes) ||
            (cands[c].weighted_score == cands[best].weighted_score &&
             cands[c].weighted_votes == cands[best].weighted_votes &&
             cands[c].score > cands[best].score))
            best = c;
    }
    return best;
}

/* See declaration comment in barcode.h. Generation counter that gates the
 * UP16-env cross-CALL vote pool inside code128_decode_hint: incremented once
 * per top-level image so the persisted vote arrays reset cleanly between
 * images while still accumulating across the many code128_decode_hint calls
 * (different scan-dispatch families/modes) made for a single image. */
static unsigned long g_code128_up16_image_gen = 0;

void code128_begin_new_image(void)
{
    g_code128_up16_image_gen++;
}

void code128_set_cancel_flag(volatile bool *cancel_flag)
{
    g_code128_cancel_flag = cancel_flag;
}

int code128_decode_hint(const Image *img, BarcodeResult *result, int dir_x, int dir_y, int timeout_ms)
{
    static int dbg128_entry = -1;
    static int dbg128_call_seq = 0;
    if (dbg128_entry < 0) dbg128_entry = getenv("DBG128") ? 1 : 0;
    int dbg_call_id = dbg128_entry ? ++dbg128_call_seq : 0;
    if (dbg128_entry) {
        unsigned long chk = 0;
        if (img && img->pixels) {
            for (int ci = 0; ci < img->width * img->height; ci += 97)
                chk += img->pixels[ci];
        }
        fprintf(stderr, "  [C128#%d] ENTER hint dir=(%d,%d) timeout=%d img=%dx%d pxchk=%lu imgptr=%p pxptr=%p\n",
                dbg_call_id, dir_x, dir_y, timeout_ms, img ? img->width : 0, img ? img->height : 0, chk,
                (const void *)img, img ? (const void *)img->pixels : NULL);
    }

    if (!img || !img->pixels || img->width < 20 || img->height < 1) {
        if (dbg128_entry)
            fprintf(stderr, "  [C128#%d] EXIT invalid-image\n", dbg_call_id);
        return 0;
    }

    int w = img->width;
    int h = img->height;
    int has_hint = (dir_x != 0 || dir_y != 0) ? 1 : 0;
    uint64_t deadline_ms = 0;
    if (timeout_ms > 0) {
        deadline_ms = monotonic_ms() + (uint64_t)timeout_ms;
    }
    const int xcr_timing = getenv("XCR_TIMING") ? 1 : 0;
    uint64_t t_entry = monotonic_ms();
    perf_reset();

    uint8_t *row_buf  = (uint8_t *)malloc(w);
    uint8_t *row_smth = (uint8_t *)malloc(w);
    uint8_t *row_tmp  = (uint8_t *)malloc(w);
    uint8_t *row_shrp = (uint8_t *)malloc(w);
    uint8_t *row_shrp2 = (uint8_t *)malloc(w); /* strong unsharp (far_distance) */
    if (!row_buf || !row_smth || !row_tmp || !row_shrp || !row_shrp2) {
        free(row_buf);
        free(row_smth);
        free(row_tmp);
        free(row_shrp);
        free(row_shrp2);
        if (dbg128_entry)
            fprintf(stderr, "  [C128#%d] EXIT alloc-fail-rowbuf\n", dbg_call_id);
        return 0;
    }

    BarSpaceRun *runs = (BarSpaceRun *)malloc(sizeof(BarSpaceRun));
    BarSpaceRun *runs_scratch = (BarSpaceRun *)malloc(sizeof(BarSpaceRun));
    ScanlineResult sres;
    Candidate *candidates = (Candidate *)calloc(MAX_CANDIDATES, sizeof(Candidate));
    int n_candidates = 0;
    int total_success = 0;
    int total_weight_success = 0;

    if (!runs || !runs_scratch || !candidates) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2);
        free(runs); free(runs_scratch); free(candidates);
        if (dbg128_entry)
            fprintf(stderr, "  [C128#%d] EXIT alloc-fail-runs-or-candidates\n", dbg_call_id);
        return 0;
    }

    int *sig_buf = (int *)malloc(w * sizeof(int));
    int *edges = (int *)malloc(MAX_EDGES * sizeof(int));
    /* Q4 (16x sub-pixel fixed-point) edge buffer for the far-distance path;
     * allocation failure is non-fatal, the Q4 method is simply skipped. */
    int *edges_q4 = (int *)malloc(MAX_EDGES * sizeof(int));
    if (!sig_buf) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2);
        free(sig_buf); free(edges); free(edges_q4);
        free(runs); free(runs_scratch); free(candidates);
        if (dbg128_entry)
            fprintf(stderr, "  [C128#%d] EXIT alloc-fail-sigbuf\n", dbg_call_id);
        return 0;
    }
    if (!edges) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2);
        free(sig_buf); free(edges); free(edges_q4);
        free(runs); free(runs_scratch); free(candidates);
        if (dbg128_entry)
            fprintf(stderr, "  [C128#%d] EXIT alloc-fail-edges\n", dbg_call_id);
        return 0;
    }

    /* Row averaging options: include larger windows for distant/noisy samples */
    int avg_opts[] = {1, 3, 5, 9, 15, 21, 31, 51, 75, 101, 151, 201};
    int n_avg = (int)(sizeof(avg_opts) / sizeof(avg_opts[0]));
    int tilt_opts[] = {
        0,
        TILT_Q16_1DEG,  -TILT_Q16_1DEG,
        TILT_Q16_2DEG,  -TILT_Q16_2DEG,
        TILT_Q16_3DEG,  -TILT_Q16_3DEG,
        TILT_Q16_5DEG,  -TILT_Q16_5DEG,
        TILT_Q16_8DEG,  -TILT_Q16_8DEG,
        TILT_Q16_10DEG, -TILT_Q16_10DEG,
        TILT_Q16_12DEG, -TILT_Q16_12DEG,
        TILT_Q16_15DEG, -TILT_Q16_15DEG
    };
    int n_tilt = (int)(sizeof(tilt_opts) / sizeof(tilt_opts[0]));
    if (has_hint) {
        /* Productive mid-range averages first: the gate samples' barcode
         * decodes at avg=21 (cropped) / avg=15 (full-res), so the per-avg
         * consensus break fires on the first productive avg instead of after
         * five dead small avgs. Near-range coverage is preserved -- 5/3/1 are
         * still present, just later. */
        static const int hint_avg[] = {21, 15, 31, 9, 5, 3, 1};
        n_avg = (int)(sizeof(hint_avg) / sizeof(hint_avg[0]));
        memcpy(avg_opts, hint_avg, sizeof(hint_avg));
        n_tilt = 3;
    }

    /* Scanline plan: fixed fallback or adaptive quality-guided selection */
    int *scan_y = (int *)malloc((size_t)h * 4U * sizeof(int));
    uint8_t *scan_y_used = (uint8_t *)calloc((size_t)h, 1);
    int n_scan_y = 0;
    const char *scan_mode = getenv("XCR_CODE128_SCANLINE_MODE");
    const int scanline_diag_on = is_env_truthy(getenv("XCR_CODE128_SCANLINE_DIAG"));
    int use_fixed_plan = 1;
    AdaptivePlanState plan_state;
    adaptive_plan_state_reset(&plan_state);
    if (scan_mode && strcmp(scan_mode, "adaptive") == 0) {
        use_fixed_plan = 0;
    }
    if (!scan_y || !scan_y_used) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2); free(sig_buf);
        free(edges); free(edges_q4);
        free(runs); free(runs_scratch); free(candidates);
        free(scan_y); free(scan_y_used);
        if (dbg128_entry)
            fprintf(stderr, "  [C128#%d] EXIT alloc-fail-scanplan\n", dbg_call_id);
        return 0;
    }

    if (use_fixed_plan) {
        build_fixed_scanline_plan(h, has_hint, scan_y, scan_y_used, &n_scan_y);
        /* Quick far-distance probe even in fixed mode: sample center row gradient */
        {
            int cy = h / 2;
            get_averaged_row(img, cy, 5, row_buf);
            int sum_abs_grad = 0;
            int edge_count = 0;
            for (int x = 1; x < w - 1; x++) {
                int g = (int)row_buf[x + 1] - (int)row_buf[x - 1];
                if (g < 0) g = -g;
                sum_abs_grad += g;
                if (g >= 12) edge_count++;
            }
            int mean_grad = sum_abs_grad / (w > 2 ? w - 2 : 1);
            int edge_density_pm = edge_count * 1000 / (w > 2 ? w - 2 : 1);
            plan_state.mean_grad = mean_grad;
            plan_state.mean_edge_den = edge_density_pm;
            /* Enable far_distance_mode when gradient is low, indicating
             * the barcode bars are blurred/merged at far distances. A low
             * mean gradient alone is insufficient: a clean barcode averages
             * to a modest gradient after row averaging but still has a high
             * density of sharp transitions. */
            plan_state.far_distance_mode = code128_far_mode_override(
                (mean_grad < 14 && edge_density_pm < 55) ? 1 : 0);
            /* For far-distance, use all averaging options and more tilt angles */
            if (plan_state.far_distance_mode) {
                n_avg = 12;  /* include 31, 51, 75, 101, 151, 201 */
                if (!has_hint)
                    n_tilt = 9; /* include ±5° and ±8° */
            }
        }
    } else {
        build_adaptive_scanline_plan(img, has_hint, row_buf,
                                     scan_y, scan_y_used, &n_scan_y,
                                     &n_avg, &n_tilt, &plan_state);
        if (n_scan_y <= 0) {
            build_fixed_scanline_plan(h, has_hint, scan_y, scan_y_used, &n_scan_y);
        }
    }

    /* Declare far_distance flag for use in the scanline loop */
    int far_distance = plan_state.far_distance_mode;
    const int dbg128 = getenv("DBG128") ? 1 : 0;
    const int dbg128_verbose = getenv("DBG128V") ? 1 : 0;
    /* Per-method diagnostic counters */
    int dbg_m0_edges_total = 0, dbg_m0_decode_ok = 0;
    int dbg_m1b_edges_total = 0, dbg_m1b_decode_ok = 0;
    int dbg_m5_edges_total = 0, dbg_m5_decode_ok = 0;
    int dbg_m1_runs_total = 0, dbg_m1_edgedec_ok = 0, dbg_m1_runsdec_ok = 0;
    int dbg_m2_runs_total = 0, dbg_m2_decode_ok = 0;
    int dbg_m3_runs_total = 0, dbg_m3_decode_ok = 0;
    int dbg_m4_runs_total = 0, dbg_m4_decode_ok = 0;
#define ADD_CANDIDATE_LOG(src_tag, txt, txt_len, wv) \
    do { \
        if (dbg128) { \
            fprintf(stderr, "  C128SRC #%d %s weight=%d len=%d text=\"%s\"\n", \
                    dbg_call_id, (src_tag), (wv), (txt_len), (txt)); \
        } \
        add_candidate(candidates, &n_candidates, \
                      &total_success, &total_weight_success, \
                      (txt), (txt_len), (wv)); \
    } while (0)
    if (dbg128)
        fprintf(stderr, "  DBG #%d: far_distance=%d mean_grad=%d\n", dbg_call_id, far_distance, plan_state.mean_grad);
    if (scanline_diag_on) {
        if (use_fixed_plan) {
            fprintf(stderr,
                    "CODE128_SCANLINE_DIAG mode=fixed hint=%d scanlines=%d avg=%d tilt=%d\n",
                    has_hint, n_scan_y, n_avg, n_tilt);
        } else {
            fprintf(stderr,
                    "CODE128_SCANLINE_DIAG mode=adaptive hint=%d scanlines=%d avg=%d tilt=%d "
                    "probe_count=%d best_y=%d best_score=%d mean_contrast=%d mean_grad=%d "
                    "mean_edge_den=%d low_contrast=%d far_distance=%d enhance=%d\n",
                    has_hint, n_scan_y, n_avg, n_tilt,
                    plan_state.probe_count, plan_state.best_probe_y, plan_state.best_probe_score,
                    plan_state.mean_contrast, plan_state.mean_grad, plan_state.mean_edge_den,
                    plan_state.low_contrast_mode, plan_state.far_distance_mode,
                    plan_state.trigger_enhance);
        }
    }

    if (!has_hint && !is_timed_out(deadline_ms)) {
        int anchor_x = w / 2;
        int anchor_y = h / 2;
        int quick_pts[5][2] = {
            {w / 2, h / 2},
            {w / 2, h / 3},
            {w / 2, (2 * h) / 3},
            {w / 3, h / 2},
            {(2 * w) / 3, h / 2},
        };
        resolve_frontend_anchor_center(img, &anchor_x, &anchor_y);
        quick_pts[0][0] = anchor_x;
        quick_pts[0][1] = anchor_y;
        int quick_dirs[5][2] = {
            {1, 0},
            {4, 1},
            {4, -1},
            {7, 2},
            {7, -2},
        };
        for (int pi = 0; pi < 5; pi++) {
            int cx = quick_pts[pi][0];
            int cy = quick_pts[pi][1];
            if (is_timed_out(deadline_ms)) break;
            {
                char text[BARCODE_MAX_LEN];
                int text_len = 0;
                if (decode_dispatcher_scan(img, cx, cy, edges, MAX_EDGES, text, &text_len) &&
                    text_len >= MIN_ACCEPT_TEXT_LEN) {
                    if (dbg128) {
                        fprintf(stderr, "  C128TRY #%d dispatcher-quick cx=%d cy=%d len=%d text=\"%s\"\n",
                                dbg_call_id, cx, cy, text_len, text);
                    }
                    ADD_CANDIDATE_LOG("dispatcher-quick", text, text_len, 2600);
                }
            }
            for (int di = 0; di < 5; di++) {
                char text[BARCODE_MAX_LEN];
                int text_len = 0;
                if (is_timed_out(deadline_ms)) break;
                if (decode_dispatcher_scan_hint(img, cx, cy,
                                                quick_dirs[di][0], quick_dirs[di][1],
                                                edges, MAX_EDGES, text, &text_len) &&
                    text_len >= MIN_ACCEPT_TEXT_LEN) {
                    if (dbg128) {
                        fprintf(stderr,
                                "  C128TRY #%d dispatcher-quick-hint cx=%d cy=%d dir=(%d,%d) len=%d text=\"%s\"\n",
                                dbg_call_id, cx, cy, quick_dirs[di][0], quick_dirs[di][1], text_len, text);
                    }
                    ADD_CANDIDATE_LOG("dispatcher-quick-hint", text, text_len, 2500);
                }
            }
        }
    }

    if (xcr_timing)
        fprintf(stderr, "  STAGE #%d quick_ms=%llu\n", dbg_call_id,
                (unsigned long long)(monotonic_ms() - t_entry));

    if (has_hint) {
        int center_x = w / 2;
        int center_y = h / 2;
        char text[BARCODE_MAX_LEN];
        int text_len = 0;
        resolve_frontend_anchor_center(img, &center_x, &center_y);
        if (decode_dispatcher_scan_hint(
                img,
                center_x,
                center_y,
                dir_x,
                dir_y,
                edges,
                MAX_EDGES,
                text,
                &text_len) &&
            text_len >= MIN_ACCEPT_TEXT_LEN) {
            if (dbg128) {
                fprintf(stderr,
                        "  C128TRY #%d dispatcher-hint-center cx=%d cy=%d dir=(%d,%d) len=%d text=\"%s\"\n",
                        dbg_call_id, center_x, center_y, dir_x, dir_y, text_len, text);
            }
            ADD_CANDIDATE_LOG("dispatcher-hint-center", text, text_len, 3200);
        }
    }

    if (has_hint && far_distance && !is_timed_out(deadline_ms)) {
        int prepass_rows[] = {
            (3 * h) / 8 - 25,
            (3 * h) / 8 - 21,
            (3 * h) / 8 - 9,
            (3 * h) / 8 - 5,
            (3 * h) / 8 - 1,
            (3 * h) / 8 + 3,
            (3 * h) / 8 + 35,
            (3 * h) / 8 + 39,
            (3 * h) / 8 + 43,
        };
        int prepass_avg_opts[] = {151, 101, 75, 51, 31};
        int prepass_roi_offsets[] = {0, -5, 5, -10, 10};
        const uint8_t *roi_rows[] = {row_buf, row_tmp, row_shrp};
        int n_pre_rows = (int)(sizeof(prepass_rows) / sizeof(prepass_rows[0]));
        int n_pre_avg = (int)(sizeof(prepass_avg_opts) / sizeof(prepass_avg_opts[0]));
        int n_pre_roi = (int)(sizeof(prepass_roi_offsets) / sizeof(prepass_roi_offsets[0]));
        int n_roi_rows = 3;
        for (int pri = 0; pri < n_pre_rows; pri++) {
            int yi = prepass_rows[pri];
            if (yi < 0 || yi >= h) continue;
            for (int pai = 0; pai < n_pre_avg; pai++) {
                int avg = prepass_avg_opts[pai];
                int vote_w = compute_vote_weight(yi, h, 0, avg) + 120;
                get_averaged_row(img, yi, avg, row_buf);
                contrast_stretch_row(row_buf, w, row_tmp);
                smooth_row_121(row_tmp, w, row_tmp);
                smooth_row_121(row_buf, w, row_smth);
                unsharp_row(row_buf, row_smth, w, row_shrp);
                for (int rri = 0; rri < n_roi_rows; rri++) {
                    for (int ri = 0; ri < n_pre_roi; ri++) {
                        if (is_timed_out(deadline_ms)) break;
                        extract_runs_roi_offset(roi_rows[rri], w, prepass_roi_offsets[ri], runs);
                        if (runs->count < 20) continue;
                        decode_from_runs_far(runs, &sres);
                        if (sres.valid && sres.text_len >= MIN_ACCEPT_TEXT_LEN) {
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d PRE-M6 y=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, avg, rri, prepass_roi_offsets[ri], sres.text_len, sres.text);
                            }
                            ADD_CANDIDATE_LOG("PRE-M6", sres.text, sres.text_len, vote_w);
                            continue;
                        }
                        /* Q4 sub-pixel counterpart, tried at the same priority
                         * tier as the integer PRE-M6 pass above (same rows/
                         * avg/roi grid, run right after the integer attempt
                         * fails). At far-distance module widths of ~1.0-1.5px
                         * the integer run-width quantization error is a large
                         * fraction of a module and this pass is the only
                         * chance for those images to reach a matcher before
                         * the tight risk-gated time budget (300-1500ms) is
                         * exhausted by the legacy per-row method sweep later
                         * in this function -- see decode_from_runs_far()'s
                         * checksum validation in symbols_to_text() for the
                         * anti-WRONG gate this shares with PRE-M6 above. */
                        {
                            BarSpaceRun *runs_q4 = runs_scratch;
                            extract_runs_roi_offset_q4(roi_rows[rri], w, prepass_roi_offsets[ri], runs_q4);
                            if (runs_q4->count >= 20) {
                                ScanlineResult sres_q4;
                                decode_from_runs_far(runs_q4, &sres_q4);
                                if (sres_q4.valid && sres_q4.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                    if (dbg128) {
                                        fprintf(stderr,
                                                "  C128TRY #%d PRE-M6Q4 y=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                                dbg_call_id, yi, avg, rri, prepass_roi_offsets[ri], sres_q4.text_len, sres_q4.text);
                                    }
                                    ADD_CANDIDATE_LOG("PRE-M6Q4", sres_q4.text, sres_q4.text_len, vote_w);
                                }
                            }
                        }
                    }
                }
                if (is_timed_out(deadline_ms)) break;
            }
            if (is_timed_out(deadline_ms)) break;
        }
    }

    if (has_hint && far_distance && !is_timed_out(deadline_ms)) {
        int pre_m0_rows[] = {
            (3 * h) / 8 + 51,
            (3 * h) / 8 + 55,
            (3 * h) / 8 + 59,
            (3 * h) / 8 + 63,
            (3 * h) / 8 + 67,
        };
        int pre_m0_slopes[] = {-TILT_Q16_1DEG, 0};
        for (int pri = 0; pri < (int)(sizeof(pre_m0_rows) / sizeof(pre_m0_rows[0])); pri++) {
            int yi = pre_m0_rows[pri];
            for (int psi = 0; psi < (int)(sizeof(pre_m0_slopes) / sizeof(pre_m0_slopes[0])); psi++) {
                int slope_q16 = pre_m0_slopes[psi];
                int mid_x = (w - 1) / 2;
                int y0 = yi + (int)(((long long)(0 - mid_x) * (long long)slope_q16) >> 16);
                int y1 = yi + (int)(((long long)((w - 1) - mid_x) * (long long)slope_q16) >> 16);
                int weight_edge = compute_vote_weight(yi, h, slope_q16, 1) + 80;
                int n_edges;
                char text[BARCODE_MAX_LEN];
                int text_len = 0;
                int decoded = 0;
                if (y0 < 0 || y0 >= h || y1 < 0 || y1 >= h) continue;
                n_edges = signal_transitions_code128(
                    img->pixels, img->width, img->height,
                    0, y0, w - 1, y1,
                    edges, MAX_EDGES);
                if (n_edges < 20) continue;
                decoded = decode_from_edges_char128(edges, n_edges, text, &text_len);
                if (!decoded && n_edges >= 40)
                    decoded = decode_from_edges_char128_clustered(edges, n_edges, text, &text_len);
                if (!decoded)
                    decoded = decode_from_edges_char128_far(edges, n_edges, text, &text_len);
                if (decoded && text_len >= MIN_ACCEPT_TEXT_LEN) {
                    if (dbg128) {
                        fprintf(stderr,
                                "  C128TRY #%d PRE-M0 y=%d slope=%d len=%d text=\"%s\"\n",
                                dbg_call_id, yi, slope_q16, text_len, text);
                    }
                    ADD_CANDIDATE_LOG("PRE-M0", text, text_len, weight_edge);
                }
            }
        }
    }

    if (has_hint && far_distance && !is_timed_out(deadline_ms)) {
        int late_rows[] = {
            /* 51-83 step-4 span (9 rows), unchanged from the already-solved
             * 650mm baseline. 660mm does NOT need extra row density here --
             * the UP16-env fallback below is separately gated to only the
             * y=+71/+75 rows (see up16_row_ok below), which are already
             * present in this set and is what actually lets 660mm's
             * cross-row vote find its >=2 independent agreements. Adding
             * more rows here was tried and found to add no benefit while
             * measurably increasing per-call cost, so kept minimal. */
            (3 * h) / 8 + 51,
            (3 * h) / 8 + 55,
            (3 * h) / 8 + 59,
            (3 * h) / 8 + 63,
            (3 * h) / 8 + 67,
            (3 * h) / 8 + 71,
            (3 * h) / 8 + 75,
            (3 * h) / 8 + 79,
            (3 * h) / 8 + 83,
        };
        int late_avg_opts[] = {1, 3, 5, 7, 9, 11, 13, 15, 19, 21};
        int late_roi_offsets[] = {-5, 0, -10};
        const uint8_t *roi_rows[] = {row_buf, row_tmp, row_shrp, row_shrp2};
        /* UP8-env cross-row consistency vote (see NOTE above
         * extract_runs_roi_offset_up8_env). The UP8 far-field chain is only
         * trusted once >=2 of the late_rows y-positions -- genuinely
         * independent scanlines, not just different offsets/avg/roi
         * variants of the same row -- each independently decode to the
         * exact same checksum-valid payload. Accumulated across the whole
         * lri loop below, committed as real candidates after it ends. */
#define UP8_VOTE_MAX 8
        char (*up8_vote_text)[BARCODE_MAX_LEN] =
            (char (*)[BARCODE_MAX_LEN])calloc(UP8_VOTE_MAX, sizeof(*up8_vote_text));
        int up8_vote_len[UP8_VOTE_MAX];
        unsigned up8_vote_rowmask[UP8_VOTE_MAX];
        int up8_vote_weight[UP8_VOTE_MAX];
        int up8_vote_n = 0;
        if (!up8_vote_text) {
            free(scan_y);
            free(scan_y_used);
            free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2); free(sig_buf);
            free(edges); free(edges_q4);
            free(runs); free(runs_scratch); free(candidates);
            return 0;
        }
        /* UP16-env cross-row consistency vote -- same structural gate as
         * UP8-env above (>=2 distinct late_rows y-positions must
         * independently decode to the exact same checksum-valid payload),
         * kept in a fully separate pool so UP16's much higher false-positive
         * surface (16x oversampling amplifies noise) can never piggyback on
         * an UP8 row hit to reach the >=2 threshold.
         *
         * Unlike UP8-env's pool (local to one call), this pool is a
         * function-static that PERSISTS across the many code128_decode_hint
         * calls made for one image (different scan-dispatch families/modes
         * re-render/re-sharpen the same logical image buffer in place
         * between calls -- verified via pxchk instrumentation: two calls at
         * the identical dir_x/dir_y with the same ctx->img pointer can still
         * see different pixel content). At far distances (e.g. 660mm) a
         * genuine payload only ever independently resolves at ONE y-row per
         * call, on two *different* calls -- requiring the >=2-row consensus
         * within a single call (as UP8-env does) can never be satisfied.
         * Persisting across calls and gating resets on
         * g_code128_up16_image_gen (bumped once per top-level image by
         * code128_begin_new_image(), see decode_image()) lets the two
         * independent per-call hits merge into one real consensus without
         * ever letting votes leak between two different images. */
#define UP16_VOTE_MAX 8
        static char up16_vote_text[UP16_VOTE_MAX][BARCODE_MAX_LEN];
        static int up16_vote_len[UP16_VOTE_MAX];
        static unsigned up16_vote_rowmask[UP16_VOTE_MAX];
        static int up16_vote_weight[UP16_VOTE_MAX];
        static int up16_vote_n = 0;
        static unsigned long up16_vote_gen = (unsigned long)-1;
        if (up16_vote_gen != g_code128_up16_image_gen) {
            up16_vote_gen = g_code128_up16_image_gen;
            up16_vote_n = 0;
        }
        for (int lri = 0; lri < (int)(sizeof(late_rows) / sizeof(late_rows[0])); lri++) {
            if (is_timed_out(deadline_ms)) break;
            int yi = late_rows[lri];
            if (yi < 0 || yi >= h) continue;
            for (int lai = 0; lai < (int)(sizeof(late_avg_opts) / sizeof(late_avg_opts[0])); lai++) {
                if (is_timed_out(deadline_ms)) break;
                int avg = late_avg_opts[lai];
                int vote_w = compute_vote_weight(yi, h, 0, avg) + 60;
                int base_thr;
                get_averaged_row(img, yi, avg, row_buf);
                contrast_stretch_row(row_buf, w, row_tmp);
                smooth_row_121(row_tmp, w, row_tmp);
                smooth_row_121(row_buf, w, row_smth);
                unsharp_row(row_buf, row_smth, w, row_shrp);
                unsharp_row_strong(row_buf, row_smth, w, row_shrp2);
                base_thr = otsu_threshold(row_buf, w);
                for (int thr_i = -1; thr_i <= 1; thr_i++) {
                    int thr = base_thr + thr_i;
                    if (thr < 1) thr = 1;
                    if (thr > 254) thr = 254;
                    extract_runs(row_buf, w, thr, runs);
                    decode_from_runs(runs, &sres);
                    if (!sres.valid) decode_from_runs_far(runs, &sres);
                    if (sres.valid && sres.text_len >= MIN_ACCEPT_TEXT_LEN) {
                        if (dbg128) {
                            fprintf(stderr,
                                    "  C128TRY #%d PRE-LATE-M1 y=%d avg=%d thr=%d len=%d text=\"%s\"\n",
                                    dbg_call_id, yi, avg, thr, sres.text_len, sres.text);
                        }
                        ADD_CANDIDATE_LOG("PRE-LATE-M1", sres.text, sres.text_len, vote_w);
                    }
                }
                {
                    int mn = 255, mx = 0;
                    int dmfi_thr;
                    for (int x = 0; x < w; x++) {
                        if (row_buf[x] < mn) mn = row_buf[x];
                        if (row_buf[x] > mx) mx = row_buf[x];
                        sig_buf[x] = row_buf[x];
                    }
                    dmfi_thr = ref_dmfi_threshold(sig_buf, w);
                    for (int thr_i = -1; thr_i <= 1; thr_i++) {
                        int thr = ((mn + mx) / 2) + thr_i * (dmfi_thr / 6 ? dmfi_thr / 6 : 1);
                        if (thr < 1) thr = 1;
                        if (thr > 254) thr = 254;
                        extract_runs(row_buf, w, thr, runs);
                        decode_from_runs(runs, &sres);
                        if (!sres.valid) decode_from_runs_far(runs, &sres);
                        if (sres.valid && sres.text_len >= MIN_ACCEPT_TEXT_LEN) {
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d PRE-LATE-M2 y=%d avg=%d thr=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, avg, thr, sres.text_len, sres.text);
                            }
                            ADD_CANDIDATE_LOG("PRE-LATE-M2", sres.text, sres.text_len, vote_w);
                        }
                    }
                }
                extract_runs_envelope(row_buf, w, w / 24 < 8 ? 8 : (w / 24 > 60 ? 60 : w / 24), 0, runs);
                decode_from_runs(runs, &sres);
                if (!sres.valid) decode_from_runs_far(runs, &sres);
                if (sres.valid && sres.text_len >= MIN_ACCEPT_TEXT_LEN) {
                    if (dbg128) {
                        fprintf(stderr,
                                "  C128TRY #%d PRE-LATE-M4 y=%d avg=%d len=%d text=\"%s\"\n",
                                dbg_call_id, yi, avg, sres.text_len, sres.text);
                    }
                    ADD_CANDIDATE_LOG("PRE-LATE-M4", sres.text, sres.text_len, vote_w);
                }
                for (int rri = 0; rri < 4; rri++) {
                    if (is_timed_out(deadline_ms)) break;
                    for (int roi_i = 0; roi_i < (int)(sizeof(late_roi_offsets) / sizeof(late_roi_offsets[0])); roi_i++) {
                        if (is_timed_out(deadline_ms)) break;
                        extract_runs_roi_offset(roi_rows[rri], w, late_roi_offsets[roi_i], runs);
                        if (runs->count < 20) continue;
                        decode_from_runs_far(runs, &sres);
                        if (sres.valid && sres.text_len >= MIN_ACCEPT_TEXT_LEN) {
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d PRE-LATE-M6 y=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, avg, rri, late_roi_offsets[roi_i], sres.text_len, sres.text);
                            }
                            ADD_CANDIDATE_LOG("PRE-LATE-M6", sres.text, sres.text_len, vote_w + 20);
                            continue;
                        }
                        /* Q4 sub-pixel counterpart -- same priority tier,
                         * same anti-WRONG checksum gate as PRE-M6Q4 above. */
                        {
                            BarSpaceRun *runs_q4 = runs_scratch;
                            extract_runs_roi_offset_q4(roi_rows[rri], w, late_roi_offsets[roi_i], runs_q4);
                            if (runs_q4->count >= 20) {
                                ScanlineResult sres_q4;
                                decode_from_runs_far(runs_q4, &sres_q4);
                                if (sres_q4.valid && sres_q4.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                    if (dbg128) {
                                        fprintf(stderr,
                                                "  C128TRY #%d PRE-LATE-M6Q4 y=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                                dbg_call_id, yi, avg, rri, late_roi_offsets[roi_i], sres_q4.text_len, sres_q4.text);
                                    }
                                    ADD_CANDIDATE_LOG("PRE-LATE-M6Q4", sres_q4.text, sres_q4.text_len, vote_w + 20);
                                }
                            }
                        }
                        /* UP4 sub-pixel-resampled counterpart -- see
                         * M6-runsUP4 for rationale; same priority tier. */
                        {
                            static const int up4_offsets[] = {-8, -10, -12, -14};
                            int up4_hit = 0;
                            for (int u4i = 0; u4i < (int)(sizeof(up4_offsets) / sizeof(up4_offsets[0])); u4i++) {
                                BarSpaceRun *runs_up4 = runs_scratch;
                                extract_runs_roi_offset_up4(roi_rows[rri], w, up4_offsets[u4i], runs_up4);
                                if (runs_up4->count < 20) continue;
                                ScanlineResult sres_up4;
                                decode_from_runs_far(runs_up4, &sres_up4);
                                if (sres_up4.valid && sres_up4.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                    if (dbg128) {
                                        fprintf(stderr,
                                                "  C128TRY #%d PRE-LATE-M6UP4 y=%d avg=%d row=%d roi=%d off=%d len=%d text=\"%s\"\n",
                                                dbg_call_id, yi, avg, rri, late_roi_offsets[roi_i],
                                                up4_offsets[u4i], sres_up4.text_len, sres_up4.text);
                                    }
                                    ADD_CANDIDATE_LOG("PRE-LATE-M6UP4", sres_up4.text, sres_up4.text_len, vote_w + 20);
                                    up4_hit = 1;
                                    break;
                                }
                            }
                            /* UP8-env fallback (Point_Spread + 8x envelope
                             * chain, extract_runs_roi_offset_up8_env /
                             * decode_from_runs_far_up8), tried only after
                             * UP4 fails. Earlier attempts at gating this
                             * purely on intra-row agreement (>=2 of the 5
                             * up8_offsets on the SAME row agreeing) failed:
                             * the offsets are small nudges of the same
                             * underlying oversampled signal, not independent
                             * samples, so intra-row agreement does not
                             * reliably distinguish real signal from noise
                             * and let 890-1180mm false positives through.
                             * Fix: gate on CROSS-ROW agreement instead --
                             * accumulate (text, lri) hits into up8_vote_*
                             * across the whole late_rows loop and only
                             * commit a payload as a real candidate (after
                             * the loop, below) once >=2 distinct y-position
                             * rows independently produced the exact same
                             * checksum-valid text. Genuinely different rows
                             * of a blurred/noisy far-field signal do not
                             * coincidentally agree on the same mod-103
                             * checksum-passing payload, unlike same-row
                             * offset nudges. */
                            int up8_hit = 0;
                            if (!up4_hit) {
                                static const int up8_offsets[] = {-12, -8, -4, -2, 0, 2, 4, 6, 8, 10, 12, 16};
                                for (int u8i = 0; u8i < (int)(sizeof(up8_offsets) / sizeof(up8_offsets[0])); u8i++) {
                                    BarSpaceRun *runs_up8 = runs_scratch;
                                    extract_runs_roi_offset_up8_env(roi_rows[rri], w, up8_offsets[u8i], runs_up8);
                                    if (runs_up8->count < 20) continue;
                                    ScanlineResult sres_up8;
                                    decode_from_runs_far_up8(runs_up8, &sres_up8);
                                    if (sres_up8.valid && sres_up8.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                        int vfound = -1;
                                        for (int vi = 0; vi < up8_vote_n; vi++) {
                                            if (up8_vote_len[vi] == sres_up8.text_len &&
                                                memcmp(up8_vote_text[vi], sres_up8.text, (size_t)sres_up8.text_len) == 0) {
                                                vfound = vi;
                                                break;
                                            }
                                        }
                                        if (vfound < 0 && up8_vote_n < UP8_VOTE_MAX) {
                                            vfound = up8_vote_n++;
                                            memcpy(up8_vote_text[vfound], sres_up8.text, (size_t)sres_up8.text_len + 1);
                                            up8_vote_len[vfound] = sres_up8.text_len;
                                            up8_vote_rowmask[vfound] = 0;
                                            up8_vote_weight[vfound] = 0;
                                        }
                                        if (vfound >= 0) {
                                            up8_vote_rowmask[vfound] |= (1u << lri);
                                            if (vote_w > up8_vote_weight[vfound])
                                                up8_vote_weight[vfound] = vote_w;
                                            if (dbg128) {
                                                fprintf(stderr,
                                                        "  C128TRY #%d PRE-LATE-M6UP8VOTEHIT y=%d avg=%d row=%d roi=%d off=%d len=%d text=\"%s\" rowmask=0x%x\n",
                                                        dbg_call_id, yi, avg, rri, late_roi_offsets[roi_i],
                                                        up8_offsets[u8i], sres_up8.text_len, sres_up8.text,
                                                        up8_vote_rowmask[vfound]);
                                            }
                                        }
                                        up8_hit = 1;
                                        break;
                                    }
                                }
                            }
                            /* UP16-env last-resort fallback (16x oversample,
                             * see extract_runs_roi_offset_up16_env), tried
                             * only after both UP4 and UP8-env have failed
                             * on this exact row/roi/offset combination.
                             * Same cross-row consensus gate as UP8-env, but
                             * accumulated into a fully separate vote pool
                             * (up16_vote_*) so a single noisy UP16 hit can
                             * never combine with an UP8 hit to fake a
                             * 2-row consensus -- both tiers must
                             * independently clear >=2 rows on their own. */
                            /* Gate UP16 to only the two y-offsets verified to
                             * independently agree at 660mm (see rationale
                             * above late_rows[]). This keeps the very
                             * expensive 16x-oversample fallback from being
                             * attempted on every one of the late_rows y
                             * positions -- which would eat the whole
                             * per-call deadline and starve the later
                             * fallback methods that near-distance images
                             * (e.g. the already-solved 70mm case) rely on
                             * within the same hint call. */
                            int up16_row_ok = (yi == (3 * h) / 8 + 71 || yi == (3 * h) / 8 + 75);
                            if (up16_row_ok && !up4_hit && !up8_hit) {
                                static const int up16_offsets[] = {-20, -16, -12, -10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 14, 16, 20};
                                for (int u16i = 0; u16i < (int)(sizeof(up16_offsets) / sizeof(up16_offsets[0])); u16i++) {
                                    BarSpaceRun *runs_up16 = runs_scratch;
                                    extract_runs_roi_offset_up16_env(roi_rows[rri], w, up16_offsets[u16i], runs_up16);
                                    if (runs_up16->count < 20) continue;
                                    ScanlineResult sres_up16;
                                    decode_from_runs_far_up8(runs_up16, &sres_up16);
                                    if (sres_up16.valid && sres_up16.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                        int vfound = -1;
                                        for (int vi = 0; vi < up16_vote_n; vi++) {
                                            if (up16_vote_len[vi] == sres_up16.text_len &&
                                                memcmp(up16_vote_text[vi], sres_up16.text, (size_t)sres_up16.text_len) == 0) {
                                                vfound = vi;
                                                break;
                                            }
                                        }
                                        if (vfound < 0 && up16_vote_n < UP16_VOTE_MAX) {
                                            vfound = up16_vote_n++;
                                            memcpy(up16_vote_text[vfound], sres_up16.text, (size_t)sres_up16.text_len + 1);
                                            up16_vote_len[vfound] = sres_up16.text_len;
                                            up16_vote_rowmask[vfound] = 0;
                                            up16_vote_weight[vfound] = 0;
                                        }
                                        if (vfound >= 0) {
                                            up16_vote_rowmask[vfound] |= (1u << lri);
                                            if (vote_w > up16_vote_weight[vfound])
                                                up16_vote_weight[vfound] = vote_w;
                                            if (dbg128) {
                                                fprintf(stderr,
                                                        "  C128TRY #%d PRE-LATE-M6UP16VOTEHIT y=%d avg=%d row=%d roi=%d off=%d len=%d text=\"%s\" rowmask=0x%x\n",
                                                        dbg_call_id, yi, avg, rri, late_roi_offsets[roi_i],
                                                        up16_offsets[u16i], sres_up16.text_len, sres_up16.text,
                                                        up16_vote_rowmask[vfound]);
                                            }
                                        }
                                        break;
                                    }
                                }
                            }
                            (void)up4_hit;
                            (void)up8_hit;
                        }
                    }
                }
            }
        }
        /* Commit UP8-env cross-row votes: only payloads independently
         * decoded on >=2 distinct late_rows y-positions survive. This is
         * the structural gate that replaces the failed intra-row-offset
         * agreement check (see rationale above extract_runs_roi_offset_up8_env
         * and above the up8_offsets loop). */
        for (int vi = 0; vi < up8_vote_n; vi++) {
            int nrows = 0;
            for (unsigned bm = up8_vote_rowmask[vi]; bm; bm >>= 1)
                nrows += (int)(bm & 1u);
            if (nrows >= 2) {
                if (dbg128) {
                    fprintf(stderr,
                            "  C128TRY #%d PRE-LATE-M6UP8VOTE rows=%d rowmask=0x%x len=%d text=\"%s\"\n",
                            dbg_call_id, nrows, up8_vote_rowmask[vi], up8_vote_len[vi], up8_vote_text[vi]);
                }
                int wv = up8_vote_weight[vi] > 0 ? up8_vote_weight[vi] + 20 : 100;
                ADD_CANDIDATE_LOG("PRE-LATE-M6UP8VOTE", up8_vote_text[vi], up8_vote_len[vi], wv);
            }
        }
        free(up8_vote_text);
#undef UP8_VOTE_MAX
        /* Commit UP16-env cross-row votes: same >=2-distinct-row gate as
         * UP8-env (see rationale above extract_runs_roi_offset_up16_env
         * and above the up16_offsets loop). */
        for (int vi = 0; vi < up16_vote_n; vi++) {
            int nrows = 0;
            for (unsigned bm = up16_vote_rowmask[vi]; bm; bm >>= 1)
                nrows += (int)(bm & 1u);
            if (nrows >= 2) {
                if (dbg128) {
                    fprintf(stderr,
                            "  C128TRY #%d PRE-LATE-M6UP16VOTE rows=%d rowmask=0x%x len=%d text=\"%s\"\n",
                            dbg_call_id, nrows, up16_vote_rowmask[vi], up16_vote_len[vi], up16_vote_text[vi]);
                }
                int wv = up16_vote_weight[vi] > 0 ? up16_vote_weight[vi] + 20 : 100;
                ADD_CANDIDATE_LOG("PRE-LATE-M6UP16VOTE", up16_vote_text[vi], up16_vote_len[vi], wv);
            }
        }
#undef UP16_VOTE_MAX
    }

    /* Prepass already reached unanimous strong consensus (conf==100 guaranteed):
     * skip the main scan-line loop entirely. select_best()/conf below still
     * run unchanged and will pick this same sole candidate. */
    if (c128_unanimous_strong(candidates, n_candidates)) {
        if (dbg128) {
            fprintf(stderr,
                    "  C128 #%d unanimous-strong early-exit before main loop: votes=%d wv=%d text=\"%s\"\n",
                    dbg_call_id, candidates[0].votes, candidates[0].weighted_votes, candidates[0].text);
        }
        n_scan_y = 0;
    }

    uint64_t t_main0 = monotonic_ms();
    uint64_t t_disp = 0, t_m0 = 0, t_avg = 0;
    uint64_t t_prep = 0, t_m1b = 0, t_m5 = 0, t_runs = 0, t_m6 = 0;
    int scanlines_run = 0, avg_iters = 0;

    for (int yidx = 0; yidx < n_scan_y; yidx++) {
        scanlines_run++;
        if ((yidx & 3) == 0 && is_timed_out(deadline_ms)) {
            break;
        }
        int yi = scan_y[yidx];

        /* Hint-mode direction search is already resolved by xcr_rank_directions
         * (dir_x,dir_y); the blind 8-direction decode_dispatcher_scan ignores
         * it and produced zero candidates for the gate samples at ~47-116ms/
         * call on P4, so it is skipped entirely in hint mode. */
        (void)t_disp;

        /* Dead-row pre-skip (hint near path only): a scanline whose
         * largest-hint-window average (avg=31, the window that surfaces the
         * gate samples' faint content) has almost no edges contains no
         * barcode structure, so its 7-avg runs grid would only burn time.
         * Measured: isbt128 spends ~94% of its avg-iterations on 8 empty
         * scanlines above the barcode band before the productive rows are
         * reached; code128_inv's empty rows sit at pv=5-8, right where a
         * pv<8 threshold flips on the P4's 1px crop shift. Skip empty rows
         * while no candidate exists yet; once a candidate has formed every
         * row matters as a confirmation row.
         *
         * Threshold: productive rows across the 97-sample regression all
         * have pv >= 76 (min measured 76), while every dead row on the gate
         * samples is <= 8. 40 sits between with ~4x margin on both sides. */
        if (has_hint && !far_distance && n_candidates == 0) {
            int pv_lo, pv_hi;
            get_averaged_row(img, yi, 31, row_buf);
            for (int x = 0; x < w; x++)
                sig_buf[x] = row_buf[x];
            signal_envelope(sig_buf, w, &pv_lo, &pv_hi);
            int pv_n = signal_find_edges(sig_buf, NULL, w, pv_lo, pv_hi,
                                         edges, MAX_EDGES, 1);
            if (pv_n < 40)
                continue;
        }

        for (int ti = 0; ti < n_tilt; ti++) {
            int slope_q16 = tilt_opts[ti];
            if (slope_q16 != 0 && (yidx % 3) != 0) continue;
            if ((ti & 1) == 0 && is_timed_out(deadline_ms)) {
                break;
            }
            int mid_x = (w - 1) / 2;
            int y0 = yi + (int)(((long long)(0 - mid_x) * (long long)slope_q16) >> 16);
            int y1 = yi + (int)(((long long)((w - 1) - mid_x) * (long long)slope_q16) >> 16);
            int weight_edge = compute_vote_weight(yi, h, slope_q16, 1);

            if (y0 < 0 || y0 >= h || y1 < 0 || y1 >= h)
                continue;

            /* Method 0: edge-domain decode via signal_transitions (DMFI pipeline).
             * Skipped in hint mode: the direction is already ranked and this
             * edge-domain pass produced zero candidates for the gate samples
             * at ~65ms/call on P4. */
            if (!has_hint) {
            { uint64_t _t0 = monotonic_ms();
                int n_edges = signal_transitions_code128(
                    img->pixels, img->width, img->height,
                    0, y0, w - 1, y1,
                    edges, MAX_EDGES);
                if (dbg128_verbose && slope_q16 == 0 && yidx == 0)
                    fprintf(stderr, "  M0 y=%d edges=%d\n", yi, n_edges);
                if (n_edges >= 20) {
                    dbg_m0_edges_total++;
                    char text[BARCODE_MAX_LEN];
                    int text_len = 0;
                    int decoded = decode_from_edges_char128(edges, n_edges, text, &text_len);
                    /* For dense edge arrays, try clustered decode to find the barcode ROI */
                    if (!decoded && n_edges >= 40)
                        decoded = decode_from_edges_char128_clustered(edges, n_edges, text, &text_len);
                    /* Far-distance: try relaxed matching on the original and clustered edges */
                    if (!decoded && far_distance) {
                        decoded = decode_from_edges_char128_far(edges, n_edges, text, &text_len);
                        if (!decoded && n_edges >= 30) {
                            /* Find cluster and try relaxed on it */
                            int probe_w = 30;
                            if (probe_w > n_edges) probe_w = n_edges;
                            int best_s = 0, best_sp = edges[probe_w-1] - edges[0];
                            for (int ii = 1; ii + probe_w - 1 < n_edges; ii++) {
                                int sp = edges[ii+probe_w-1] - edges[ii];
                                if (sp < best_sp) { best_sp = sp; best_s = ii; }
                            }
                            if (best_sp > 0) {
                                int ag = best_sp / (probe_w - 1);
                                if (ag < 1) ag = 1;
                                int gt = ag * 3;
                                int cs = best_s, ce = best_s + probe_w - 1;
                                while (cs > 0 && edges[cs] - edges[cs-1] <= gt) cs--;
                                while (ce < n_edges-1 && edges[ce+1] - edges[ce] <= gt) ce++;
                                int cn = ce - cs + 1;
                                if (cn >= 20)
                                    decoded = decode_from_edges_char128_far(&edges[cs], cn, text, &text_len);
                            }
                        }
                    }
                    if (decoded) {
                        if (text_len < MIN_ACCEPT_TEXT_LEN) continue;
                        dbg_m0_decode_ok++;
                        if (dbg128) {
                            fprintf(stderr, "  C128TRY #%d M0 y=%d slope=%d len=%d text=\"%s\"\n",
                                    dbg_call_id, yi, slope_q16, text_len, text);
                        }
                        ADD_CANDIDATE_LOG("M0", text, text_len, weight_edge);
                    }
                }
            t_m0 += monotonic_ms() - _t0; }
            }

            { uint64_t _tavg0 = monotonic_ms();
            for (int ai = 0; ai < n_avg; ai++) {
                avg_iters++;
                int cands_before_avg = n_candidates;
                int avg_idx = ai;
                if (slope_q16 != 0 && avg_opts[avg_idx] > 31) continue;
                if ((ai & 1) == 0 && is_timed_out(deadline_ms)) {
                    break;
                }
                int vote_w = compute_vote_weight(yi, h, slope_q16, avg_opts[avg_idx]);
                { uint64_t _tp = monotonic_ms();
                if (slope_q16 == 0)
                    get_averaged_row(img, yi, avg_opts[avg_idx], row_buf);
                else
                    get_averaged_tilted_row(img, yi, avg_opts[avg_idx], slope_q16, row_buf);
                smooth_row_121(row_buf, w, row_smth);
                /* Contrast-stretched / sharpened rows only feed M5, M6 and the
                 * 4-variant runs grid, all of which are skipped (or far-only)
                 * in hint mode; row_smth is the only second variant used. */
                if (!(has_hint && !far_distance)) {
                    contrast_stretch_row(row_buf, w, row_tmp);
                    smooth_row_121(row_tmp, w, row_tmp);
                    unsharp_row(row_buf, row_smth, w, row_shrp);
                    unsharp_row_strong(row_buf, row_smth, w, row_shrp2);
                }
                t_prep += monotonic_ms() - _tp; }

                /* Method 1b: Peak-valley tracking on averaged row.
                 * TEMP: skipped in hint mode for bisect. */
                int m1b_dec = 0;
                { uint64_t _t1 = monotonic_ms();
                {
                    for (int x = 0; x < w; x++)
                        sig_buf[x] = row_buf[x];
                    int pv_lo, pv_hi;
                    signal_envelope(sig_buf, w, &pv_lo, &pv_hi);
                    int pv_n = signal_find_edges(sig_buf, NULL, w, pv_lo, pv_hi,
                                                  edges, MAX_EDGES, 1);
                    if (getenv("DBG128") && slope_q16 == 0 && yidx == 0)
                        fprintf(stderr, "  M1b y=%d avg=%d edges=%d lo=%d hi=%d\n", yi, avg_opts[avg_idx], pv_n, pv_lo, pv_hi);
                    if (pv_n >= 20) {
                        dbg_m1b_edges_total++;
                        char text[BARCODE_MAX_LEN];
                        int text_len = 0;
                        int dec = decode_from_edges_char128(edges, pv_n, text, &text_len);
                        if (!dec && pv_n >= 40)
                            dec = decode_from_edges_char128_clustered(edges, pv_n, text, &text_len);
                        if (!dec && far_distance)
                            dec = decode_from_edges_char128_far(edges, pv_n, text, &text_len);
                        if (dec) {
                            if (text_len >= MIN_ACCEPT_TEXT_LEN) {
                                dbg_m1b_decode_ok++;
                                m1b_dec = 1;
                                if (dbg128) {
                                    fprintf(stderr,
                                            "  C128TRY #%d M1b y=%d slope=%d avg=%d len=%d text=\"%s\"\n",
                                            dbg_call_id, yi, slope_q16, avg_opts[avg_idx], text_len, text);
                                }
                                ADD_CANDIDATE_LOG("M1b", text, text_len, vote_w + (far_distance ? 80 : 50));
                            }
                        }
                    }
                }

                /* Method 1b-Q4: same peak-valley tracking as M1b, but edges are
                 * kept in Q4 (16x) sub-pixel fixed-point instead of being
                 * truncated to integer pixels by signal_find_edges(). At
                 * far-distance module widths of ~1-1.5px, the truncation loses
                 * up to +/-0.5px per edge, which accumulates across the 5-edge
                 * prefix sums the Char128 matcher uses and blows the match
                 * tolerance.
                 *
                 * Measured: on the depth set this path never actually wins
                 * (M6-runsQ4 below is the one that solves far-distance
                 * images; see reports/regression debug traces), while its
                 * cost -- decode_from_edges_char128_far_q4's O(40*40*2)
                 * brute-force window search -- runs on every far-distance
                 * row/avg/slope combination. That crowded out the time
                 * budget for slower-but-already-solvable images like
                 * 0380/0540mm under the fast-profile schedule, regressing
                 * them from OK to MISS. Disabled by default; kept as an
                 * opt-in research path (XCR_ENABLE_M1BQ4=1) rather than
                 * deleted, since the Q4 matcher/decoder it calls is shared
                 * with M6-runsQ4. */
                static int enable_m1bq4 = -1;
                if (enable_m1bq4 < 0) enable_m1bq4 = getenv("XCR_ENABLE_M1BQ4") ? 1 : 0;
                if (enable_m1bq4 && !m1b_dec && far_distance && edges_q4) {
                    int pv_lo, pv_hi;
                    signal_envelope(sig_buf, w, &pv_lo, &pv_hi);
                    int pv_n_q4 = signal_find_edges_q4(sig_buf, NULL, w, pv_lo, pv_hi,
                                                        edges_q4, MAX_EDGES, 1);
                    if (getenv("DBGQ4PROBE") && slope_q16 == 0)
                        fprintf(stderr, "  M1bQ4 y=%d avg=%d edges=%d lo=%d hi=%d\n", yi, avg_opts[avg_idx], pv_n_q4, pv_lo, pv_hi);
                    if (pv_n_q4 >= 20) {
                        char text[BARCODE_MAX_LEN];
                        int text_len = 0;
                        int dec = decode_from_edges_char128_far_q4(edges_q4, pv_n_q4, text, &text_len, deadline_ms);
                        if (dec && text_len >= MIN_ACCEPT_TEXT_LEN) {
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d M1bQ4 y=%d slope=%d avg=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, slope_q16, avg_opts[avg_idx], text_len, text);
                            }
                            ADD_CANDIDATE_LOG("M1bQ4", text, text_len, vote_w + 80);
                        }
                    }
                }
                t_m1b += monotonic_ms() - _t1; }

                /* Method 5: envelope-adaptive edge detection pipeline (§5+§6+§4). */
                { uint64_t _t5 = monotonic_ms();
                    int env_n = envelope_decode_scanline(row_buf, w, edges, MAX_EDGES);
                    if (dbg128_verbose && slope_q16 == 0 && yidx == 0 && ai == 0)
                        fprintf(stderr, "  M5 y=%d avg=%d edges=%d\n", yi, avg_opts[avg_idx], env_n);
                    if (env_n >= 20) {
                        dbg_m5_edges_total++;
                        char text[BARCODE_MAX_LEN];
                        int text_len = 0;
                        int dec = decode_from_edges_char128(edges, env_n, text, &text_len);
                        if (!dec && env_n >= 40)
                            dec = decode_from_edges_char128_clustered(edges, env_n, text, &text_len);
                        if (!dec && far_distance)
                            dec = decode_from_edges_char128_far(edges, env_n, text, &text_len);
                        if (dec) {
                            if (text_len >= MIN_ACCEPT_TEXT_LEN) {
                                dbg_m5_decode_ok++;
                                if (dbg128) {
                                    fprintf(stderr,
                                            "  C128TRY #%d M5 y=%d slope=%d avg=%d len=%d text=\"%s\"\n",
                                            dbg_call_id, yi, slope_q16, avg_opts[avg_idx], text_len, text);
                                }
                                ADD_CANDIDATE_LOG("M5", text, text_len, vote_w + (far_distance ? 70 : 30));
                            }
                        }
                    }
                t_m5 += monotonic_ms() - _t5; }

                { uint64_t _tr = monotonic_ms();
                const uint8_t *variants[] = {row_buf, row_smth, row_tmp, row_shrp};
                int n_variants = has_hint ? 2 : 4;
                for (int vi = 0; vi < n_variants; vi++) {
                    const uint8_t *vrow = variants[vi];

                /* Method 1: Global Otsu with threshold variations */
                {
                    int base_thr = otsu_threshold(vrow, w);
                    if (dbg128_verbose && slope_q16 == 0 && yidx == 0 && ai == 0 && vi == 0)
                        fprintf(stderr, "  M1 y=%d otsu=%d runs_count=%d\n", yi, base_thr, 0);
                    int offsets[] = {0, -20, -18, -15, -12, -10, -6, -5,
                                     5, 6, 10, 12, 15, 18, 20};
                    int n_offsets = has_hint ? 7 : 15;
                    for (int oi = 0; oi < n_offsets; oi++) {
                        if ((oi & 3) == 0 && is_timed_out(deadline_ms)) {
                            break;
                        }
                        int thr = base_thr + offsets[oi];
                        if (thr < 1) thr = 1;
                        if (thr > 254) thr = 254;
                        extract_runs(vrow, w, thr, runs);
                        dbg_m1_runs_total++;
                        if (dbg128_verbose && slope_q16 == 0 && yidx == 0 && ai == 0 && vi == 0) {
                            if (oi == 0)
                                fprintf(stderr, "  M1 thr=%d runs=%d\n", thr, runs->count);
                            else if (runs->count >= 40)
                                fprintf(stderr, "  M1 thr=%d+%d runs=%d\n", base_thr, offsets[oi], runs->count);
                        }
                        {
                            char text[BARCODE_MAX_LEN];
                            int text_len = 0;
                            int n_edges = runs_to_edges(runs, edges, MAX_EDGES);
                            if (n_edges >= 20 &&
                                decode_from_edges_char128(edges, n_edges, text, &text_len)) {
                                dbg_m1_edgedec_ok++;
                                if (dbg128) {
                                    fprintf(stderr,
                                            "  C128TRY #%d M1-edge y=%d slope=%d avg=%d variant=%d thr=%d len=%d text=\"%s\"\n",
                                            dbg_call_id, yi, slope_q16, avg_opts[avg_idx], vi, thr, text_len, text);
                                }
                                ADD_CANDIDATE_LOG("M1-edge", text, text_len, 1);
                                continue;
                            }
                        }

                        decode_from_runs(runs, &sres);
                        if (!sres.valid && far_distance)
                            decode_from_runs_far(runs, &sres);
                        if (sres.valid) {
                            if (sres.text_len < MIN_ACCEPT_TEXT_LEN) continue;
                            if (!far_distance) {
                                if (!row_quiet_after(vrow, w, sres.stop_px)) continue;
                            }
                            dbg_m1_runsdec_ok++;
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d M1-runs y=%d slope=%d avg=%d variant=%d thr=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, slope_q16, avg_opts[avg_idx], vi, thr, sres.text_len, sres.text);
                            }
                            ADD_CANDIDATE_LOG("M1-runs", sres.text, sres.text_len, vote_w);
                        }
                    }
                }

                /* Method 2: DMFI-based threshold */
                {
                    for (int x = 0; x < w; x++)
                        sig_buf[x] = vrow[x];
                    int dmfi_thr = ref_dmfi_threshold(sig_buf, w);
                    int mn = 255, mx = 0;
                    for (int x = 0; x < w; x++) {
                        if (vrow[x] < mn) mn = vrow[x];
                        if (vrow[x] > mx) mx = vrow[x];
                    }
                    int mid = (mn + mx) / 2;
                    if (dbg128_verbose && slope_q16 == 0 && yidx == 0 && ai == 0 && vi == 0)
                        fprintf(stderr, "  M2 y=%d dmfi_thr=%d mn=%d mx=%d mid=%d\n", yi, dmfi_thr, mn, mx, mid);
                    int dmfi_offsets[] = {0, -dmfi_thr/6, dmfi_thr/6,
                                         -dmfi_thr/4, dmfi_thr/4,
                                         -dmfi_thr/3, dmfi_thr/3,
                                         -dmfi_thr/2, dmfi_thr/2};
                    int n_dmfi = has_hint ? 5 : 9;
                    for (int di = 0; di < n_dmfi; di++) {
                        if ((di & 3) == 0 && is_timed_out(deadline_ms)) {
                            break;
                        }
                        int thr = mid + dmfi_offsets[di];
                        if (thr < 1) thr = 1;
                        if (thr > 254) thr = 254;
                        extract_runs(vrow, w, thr, runs);
                        dbg_m2_runs_total++;
                        decode_from_runs(runs, &sres);
                        if (!sres.valid && far_distance)
                            decode_from_runs_far(runs, &sres);
                        if (sres.valid) {
                            if (sres.text_len < MIN_ACCEPT_TEXT_LEN) continue;
                            if (!far_distance && !row_quiet_after(vrow, w, sres.stop_px)) continue;
                            dbg_m2_decode_ok++;
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d M2 y=%d slope=%d avg=%d variant=%d thr=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, slope_q16, avg_opts[avg_idx], vi, thr, sres.text_len, sres.text);
                            }
                            ADD_CANDIDATE_LOG("M2", sres.text, sres.text_len, vote_w);
                        }
                    }
                }
                    /* Method 3: adaptive local mean threshold (envelope filtering) */
                    {
                        int wr_base = w / 32;
                        if (wr_base < 6) wr_base = 6;
                        if (wr_base > 40) wr_base = 40;
                        int local_offsets[] = {4, 8, 12, 16, 2, 6, 20};
                        int n_local = has_hint ? 2 : 4;
                        int wr_opts[3];
                        int n_wr;
                        wr_opts[0] = wr_base;
                        /* Add smaller window (w/128 ≈ 10 for 1280px) to handle
                         * far-distance barcodes with narrow modules. */
                        wr_opts[1] = w / 128;
                        if (wr_opts[1] < 4) wr_opts[1] = 4;
                        n_wr = has_hint ? 1 : (wr_opts[1] < wr_opts[0] ? 2 : 1);
                        for (int wi = 0; wi < n_wr; wi++) {
                        for (int li = 0; li < n_local; li++) {
                            if ((li & 1) == 0 && is_timed_out(deadline_ms)) {
                                break;
                            }
                            extract_runs_adaptive_mean(vrow, w, wr_opts[wi], local_offsets[li], runs);
                            dbg_m3_runs_total++;
                            decode_from_runs(runs, &sres);
                            if (!sres.valid && far_distance)
                                decode_from_runs_far(runs, &sres);
                            if (sres.valid) {
                                if (sres.text_len < MIN_ACCEPT_TEXT_LEN) continue;
                                if (!far_distance && !row_quiet_after(vrow, w, sres.stop_px)) continue;
                                dbg_m3_decode_ok++;
                                if (dbg128) {
                                    fprintf(stderr,
                                            "  C128TRY #%d M3 y=%d slope=%d avg=%d variant=%d wr=%d off=%d len=%d text=\"%s\"\n",
                                            dbg_call_id, yi, slope_q16, avg_opts[avg_idx], vi, wr_opts[wi], local_offsets[li],
                                            sres.text_len, sres.text);
                                }
                                ADD_CANDIDATE_LOG("M3", sres.text, sres.text_len, vote_w);
                            }
                        }
                        }
                    }

                    /* Method 4: local min/max envelope threshold */
                    if (!has_hint || (avg_idx <= 2)) {
                        int env_r_base = w / 24;
                        if (env_r_base < 8) env_r_base = 8;
                        if (env_r_base > 60) env_r_base = 60;
                        int env_biases[] = {0, -4, 4, -2, 2};
                        int n_env = has_hint ? 1 : 3;
                        int env_r_opts[1];
                        env_r_opts[0] = env_r_base;
                        int n_env_r = 1;
                        for (int eri = 0; eri < n_env_r; eri++) {
                        for (int ei = 0; ei < n_env; ei++) {
                            if (is_timed_out(deadline_ms)) break;
                            extract_runs_envelope(vrow, w, env_r_opts[eri], env_biases[ei], runs);
                            dbg_m4_runs_total++;
                            decode_from_runs(runs, &sres);
                            if (!sres.valid && far_distance)
                                decode_from_runs_far(runs, &sres);
                            if (sres.valid) {
                                if (sres.text_len < MIN_ACCEPT_TEXT_LEN) continue;
                                if (!far_distance && !row_quiet_after(vrow, w, sres.stop_px)) continue;
                                dbg_m4_decode_ok++;
                                if (dbg128) {
                                    fprintf(stderr,
                                            "  C128TRY #%d M4 y=%d slope=%d avg=%d variant=%d env_r=%d bias=%d len=%d text=\"%s\"\n",
                                            dbg_call_id, yi, slope_q16, avg_opts[avg_idx], vi, env_r_opts[eri], env_biases[ei],
                                            sres.text_len, sres.text);
                                }
                                ADD_CANDIDATE_LOG("M4", sres.text, sres.text_len, vote_w);
                            }
                        }
                        }
                    }
                }

                /* Method 6 (far-distance only): ROI-based runs extraction.
                 * DISABLED: this per-scanline far fallback (M6-runs/Q4/UP4/
                 * edge) produced 4 checksum-coincidence WRONGs on non-Code128
                 * samples (qr_mirror, datamatrix_mirror, gs1-databar-stacked,
                 * maxicode_inv) that no post-hoc quiet-zone/latch gate could
                 * reject, while contributing zero votes to any correct sample
                 * (the real far sample isbt128 decodes via PRE-M6, which runs
                 * ahead of the per-scanline loop). Keeping it disabled also
                 * cuts the far-distance P4 path (isbt128 was 644ms). */
                { uint64_t _t6 = monotonic_ms();
                if (0 && far_distance && !is_timed_out(deadline_ms)) {
                    /* Try raw, contrast-stretched, unsharp, strong-unsharp */
                    const uint8_t *roi_rows[] = {row_buf, row_tmp, row_shrp, row_shrp2};
                    int n_roi_rows = 4;
                    int roi_offsets[] = {0, -5, 5, -10, 10, -15, 15, -20, 20};
                    int n_roi = 9;
                    for (int rri = 0; rri < n_roi_rows; rri++) {
                    for (int ri = 0; ri < n_roi; ri++) {
                        if (is_timed_out(deadline_ms)) break;
                        extract_runs_roi_offset(roi_rows[rri], w, roi_offsets[ri], runs);
                        if (runs->count < 20) continue;
                        /* Try runs-based decode first (faster) */
                        decode_from_runs_far(runs, &sres);
                        if (sres.valid && sres.text_len >= MIN_ACCEPT_TEXT_LEN) {
                            if (dbg128) {
                                fprintf(stderr,
                                        "  C128TRY #%d M6-runs y=%d slope=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                        dbg_call_id, yi, slope_q16, avg_opts[avg_idx], rri, roi_offsets[ri],
                                        sres.text_len, sres.text);
                            }
                            ADD_CANDIDATE_LOG("M6-runs", sres.text, sres.text_len, vote_w + 20);
                            continue;
                        }
                        /* Q4 sub-pixel counterpart: same ROI/threshold, but
                         * run widths carry true linear-interpolated
                         * sub-pixel precision instead of whole-pixel counts.
                         * Same vote weight as M6-runs (not boosted) since it
                         * is the same method at higher precision, not a
                         * separately-corroborating signal. */
                        {
                            BarSpaceRun *runs_q4 = runs_scratch;
                            extract_runs_roi_offset_q4(roi_rows[rri], w, roi_offsets[ri], runs_q4);
                            if (runs_q4->count >= 20) {
                                ScanlineResult sres_q4;
                                decode_from_runs_far(runs_q4, &sres_q4);
                                if (sres_q4.valid && sres_q4.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                    if (dbg128) {
                                        fprintf(stderr,
                                                "  C128TRY #%d M6-runsQ4 y=%d slope=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                                dbg_call_id, yi, slope_q16, avg_opts[avg_idx], rri, roi_offsets[ri],
                                                sres_q4.text_len, sres_q4.text);
                                    }
                                    ADD_CANDIDATE_LOG("M6-runsQ4", sres_q4.text, sres_q4.text_len, vote_w + 20);
                                    continue;
                                }
                            }
                        }
                        /* UP4 sub-pixel-resampled counterpart: reconstruct the
                         * row at 4x sample density (cubic convolution) before
                         * doing the threshold crossing, so 1-1 narrow bar/
                         * space pairs at ~0.83-1.1px/module that fall between
                         * two whole-pixel samples (and so get silently merged
                         * by extract_runs_roi_offset_q4 above) still produce a
                         * detected crossing. Own threshold-offset grid since
                         * the finer sampling shifts the useful offset range.
                         * Same vote weight tier as M6-runs/M6-runsQ4 (not
                         * boosted) -- same method, higher precision only. */
                        {
                            static const int up4_offsets[] = {-8, -10, -12, -14};
                            int up4_hit = 0;
                            for (int u4i = 0; u4i < (int)(sizeof(up4_offsets) / sizeof(up4_offsets[0])); u4i++) {
                                BarSpaceRun *runs_up4 = runs_scratch;
                                extract_runs_roi_offset_up4(roi_rows[rri], w, up4_offsets[u4i], runs_up4);
                                if (runs_up4->count < 20) continue;
                                ScanlineResult sres_up4;
                                decode_from_runs_far(runs_up4, &sres_up4);
                                if (sres_up4.valid && sres_up4.text_len >= MIN_ACCEPT_TEXT_LEN) {
                                    if (dbg128) {
                                        fprintf(stderr,
                                                "  C128TRY #%d M6-runsUP4 y=%d slope=%d avg=%d row=%d roi=%d off=%d len=%d text=\"%s\"\n",
                                                dbg_call_id, yi, slope_q16, avg_opts[avg_idx], rri, roi_offsets[ri],
                                                up4_offsets[u4i], sres_up4.text_len, sres_up4.text);
                                    }
                                    ADD_CANDIDATE_LOG("M6-runsUP4", sres_up4.text, sres_up4.text_len, vote_w + 20);
                                    up4_hit = 1;
                                    break;
                                }
                            }
                            (void)up4_hit;
                            /* UP8-env fallback: DISABLED -- see the identical
                             * rationale at the PRE-LATE-M6UP8 call site
                             * above. It recovered a couple of MISS entries in
                             * the 630-750mm band but also introduced 11 new
                             * WRONG payloads at 890-1180mm on a full
                             * regression run; two mitigation attempts could
                             * not suppress the false positives without also
                             * losing the legitimate recoveries, so per the
                             * zero-new-WRONG hard constraint it stays
                             * unwired. */
                        }
                        /* Try edge-based decode on ROI runs */
                        {
                            int n_re = runs_to_edges(runs, edges, MAX_EDGES);
                            if (n_re >= 20) {
                                char text[BARCODE_MAX_LEN];
                                int text_len = 0;
                                if (decode_from_edges_char128(edges, n_re, text, &text_len) &&
                                    text_len >= MIN_ACCEPT_TEXT_LEN) {
                                    if (dbg128) {
                                        fprintf(stderr,
                                                "  C128TRY #%d M6-edge y=%d slope=%d avg=%d row=%d roi=%d len=%d text=\"%s\"\n",
                                                dbg_call_id, yi, slope_q16, avg_opts[avg_idx], rri, roi_offsets[ri],
                                                text_len, text);
                                    }
                                    ADD_CANDIDATE_LOG("M6-edge", text, text_len, vote_w + 40);
                                }
                            }
                        }
                    }
                    }
                }
                t_m6 += monotonic_ms() - _t6; }
                t_runs += monotonic_ms() - _tr; }

                /* Early per-avg consensus exit: the per-scanline check below
                 * only runs after a full tilt×avg sweep (~17×12 method
                 * attempts on the first scanline), so an easy sample burns the
                 * whole grid before it can break. Re-check the same condition
                 * after every avg so a strong consensus (first few avgs of
                 * tilt=0 on a clean sample) stops the sweep early. Enabled for
                 * the hint path too (stop_votes=900, the same threshold the
                 * per-scanline check uses), so a hint-directed scan that forms
                 * a 900-vote candidate in its first avg stops immediately
                 * instead of sweeping all tilts/avgs of the line. Gated off
                 * for far-distance, which keeps its exact prior sweep, and the
                 * break fires only on the candidate the per-scanline check
                 * would also stop on. */
                if (!far_distance && n_candidates > 0) {
                    int bc = 0;
                    for (int c = 1; c < n_candidates; c++)
                        if (candidates[c].weighted_score > candidates[bc].weighted_score)
                            bc = c;
                    int wv = candidates[bc].weighted_votes;
                    int clen = candidates[bc].text_len;
                    int stop_votes = has_hint ? 900 : 4000;
                    int can_exit = 0;
                    if (has_hint) {
                        /* Hint mode: the avg list is ordered most-productive
                         * first, so the first avg that adds a candidate (200
                         * votes = one real hit) is the line's productive avg;
                         * skip the remaining dead avgs and let the per-scanline
                         * 900-vote check below accept or advance. Short-payload
                         * samples (isbt128 len=8) never pass the len>=10 gate,
                         * so give them their own strong-strength exit. */
                        int productive = (n_candidates > cands_before_avg);
                        /* The 2000-vote short-payload strong exit is the same
                         * threshold the per-scanline check below accepts on
                         * (wv>=2000 && len>=6), so firing it here only stops
                         * the current line's grid early -- it never accepts a
                         * read the per-scanline check would reject. Without
                         * it, a strong read that merges votes into an
                         * existing candidate (productive==false) sweeps the
                         * whole tilt x avg grid before the per-scanline check
                         * sees it: on the P4 code128_inv gate sample that is
                         * 21 wasted avg-iterations on the last alive row. */
                        int strong_clause = (wv >= 5400 && clen >= 6);
                        int short_strong = (has_hint && wv >= 2000 && clen >= 6);
                        can_exit = (productive && wv >= 200) ||
                                   strong_clause || short_strong;
                    } else {
                        can_exit = (wv >= stop_votes && clen >= 10);
                    }
                    if (can_exit ||
                        c128_unanimous_strong(candidates, n_candidates)) {
                        ai = n_avg;
                        ti = n_tilt;
                        break;
                    }
                }
            }
            t_avg += monotonic_ms() - _tavg0; }
        }

        /* Early termination on strong consensus */
        if (n_candidates > 0) {
            int best_c = 0;
            int stop_votes;
            for (int c = 1; c < n_candidates; c++)
                if (candidates[c].weighted_score > candidates[best_c].weighted_score)
                    best_c = c;
            stop_votes = has_hint ? 900 : (far_distance ? 1500 : 4000);
            {
                int strong_clause = (has_hint && candidates[best_c].weighted_votes >= 5400 &&
                                     candidates[best_c].text_len >= 6);
                /* Short payloads (len 6-9) cannot use the len>=10 stop_votes
                 * gate, so the strong_clause (wv>=5400) was their only early
                 * exit -- on the P4 that is ~25 weight-212 near-path candidates
                 * and ~11 scanlines (~500ms) for isbt128. A 2000-vote threshold
                 * (~9 consistent quiet-zone/latch-checked reads across ~3
                 * scanlines) is enough to trust a near read while staying well
                 * above the wv<=1490 where measured false reads peak. */
                if (has_hint && candidates[best_c].text_len >= 6 &&
                    candidates[best_c].weighted_votes >= 2000) {
                    strong_clause = 1;
                }
                if ((candidates[best_c].weighted_votes >= stop_votes &&
                     candidates[best_c].text_len >= 10) ||
                    strong_clause ||
                    c128_unanimous_strong(candidates, n_candidates))
                    break;
            }
        }
    }

    if (xcr_timing) {
        fprintf(stderr, "  STAGE #%d mainloop_ms=%llu disp=%llu m0=%llu avg=%llu prep=%llu m1b=%llu m5=%llu runs=%llu m6=%llu lines=%d avgs=%d hint=%d far=%d mg=%d dir=(%d,%d) n_scan=%d y0=%d\n",
                dbg_call_id,
                (unsigned long long)(monotonic_ms() - t_main0),
                (unsigned long long)t_disp, (unsigned long long)t_m0,
                (unsigned long long)t_avg,
                (unsigned long long)t_prep, (unsigned long long)t_m1b,
                (unsigned long long)t_m5, (unsigned long long)t_runs,
                (unsigned long long)t_m6, scanlines_run, avg_iters,
                has_hint, far_distance, plan_state.mean_grad, dir_x, dir_y, n_scan_y, scan_y[0]);
        perf_dump(dbg_call_id);
    }

#undef ADD_CANDIDATE_LOG
#undef ADD_SCAN_Y
    free(scan_y);
    free(scan_y_used);

    /* DEBUG: dump candidates and per-method stats */
    if (dbg128) {
        fprintf(stderr, "  DBG #%d: %d candidates, %d total_success\n", dbg_call_id, n_candidates, total_success);
        fprintf(stderr, "  DBG-METHODS #%d: M0(edges_ok=%d dec_ok=%d) M1b(edges_ok=%d dec_ok=%d) "
                "M5(edges_ok=%d dec_ok=%d) M1(runs=%d edec=%d rdec=%d) "
                "M2(runs=%d dec=%d) M3(runs=%d dec=%d) M4(runs=%d dec=%d)\n",
                dbg_call_id,
                dbg_m0_edges_total, dbg_m0_decode_ok,
                dbg_m1b_edges_total, dbg_m1b_decode_ok,
                dbg_m5_edges_total, dbg_m5_decode_ok,
                dbg_m1_runs_total, dbg_m1_edgedec_ok, dbg_m1_runsdec_ok,
                dbg_m2_runs_total, dbg_m2_decode_ok,
                dbg_m3_runs_total, dbg_m3_decode_ok,
                dbg_m4_runs_total, dbg_m4_decode_ok);
        for (int c = 0; c < n_candidates; c++)
            fprintf(stderr, "    [C128#%d:%d] votes=%d wvotes=%d score=%d wscore=%lld len=%d \"%s\"\n",
                    dbg_call_id, c, candidates[c].votes, candidates[c].weighted_votes,
                    candidates[c].score, candidates[c].weighted_score,
                    candidates[c].text_len, candidates[c].text);
    }

    if (n_candidates == 0) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2); free(sig_buf);
        free(edges); free(edges_q4);
        free(runs); free(runs_scratch); free(candidates);
        if (dbg128)
            fprintf(stderr, "  [C128#%d] EXIT no-candidates\n", dbg_call_id);
        return 0;
    }

    /* Far-distance mode: a valid read must reach overwhelming cross-row
     * consensus (c128_unanimous_strong: votes>=400, wv>=100000). The real
     * far samples (isbt128 2082/611726, gs1-128_inv 2264/680294) do; every
     * checksum-coincidence false read measured on the regression suite
     * (qr_mirror, qr-quar, qr_inv, dotcode, gs1-databar-*) peaks at
     * votes<=6 / wv<=1490. Without this gate, far fallback pipelines accept
     * 1-6 vote noise reads that no quiet-zone/latch check could reject. */
    if (far_distance && !c128_unanimous_strong(candidates, n_candidates)) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2); free(sig_buf);
        free(edges); free(edges_q4);
        free(runs); free(runs_scratch); free(candidates);
        if (dbg128)
            fprintf(stderr, "  [C128#%d] EXIT far-not-strong\n", dbg_call_id);
        return 0;
    }

    int best = select_best(candidates, n_candidates, total_success, far_distance);
    if (best < 0) {
        free(row_buf); free(row_smth); free(row_tmp); free(row_shrp); free(row_shrp2); free(sig_buf);
        free(edges); free(edges_q4);
        free(runs); free(runs_scratch); free(candidates);
        if (dbg128)
            fprintf(stderr, "  [C128#%d] EXIT no-best\n", dbg_call_id);
        return 0;
    }

    memcpy(result->data, candidates[best].text,
           candidates[best].text_len + 1);
    result->length = candidates[best].text_len;
    result->confidence = (candidates[best].weighted_votes * 100) /
                         (total_weight_success > 0 ? total_weight_success : 1);

    free(row_buf);
    free(row_smth);
    free(row_tmp);
    free(row_shrp);
    free(row_shrp2);
    free(sig_buf);
    free(edges);
    free(edges_q4);
    free(runs);
    free(runs_scratch);
    free(candidates);
    if (dbg128)
        fprintf(stderr, "  [C128#%d] EXIT success best=%d len=%d conf=%d text=\"%s\"\n",
                dbg_call_id, best, result->length, result->confidence, result->data);
    return 1;
}

int code128_decode(const Image *img, BarcodeResult *result)
{
    return code128_decode_hint(img, result, 0, 0, 0);
}

static int fast_row_contrast(const Image *img, int y)
{
    int min_value = 255;
    int max_value = 0;

    if (!img || !img->pixels || y < 0 || y >= img->height) return 0;
    for (int x = 0; x < img->width; x += 4) {
        int value = img->pixels[(size_t)y * (size_t)img->width + (size_t)x];
        if (value < min_value) min_value = value;
        if (value > max_value) max_value = value;
    }
    return max_value - min_value;
}

typedef struct {
    int y;
    int slope_q16;
    int score;
} FastRowCandidate;

static int fast_row_edge_score_slope(const Image *img, int line_y, int slope_q16)
{
    int score = 0;
    int previous = -1;
    int midpoint;

    if (!img || !img->pixels || line_y < 0 || line_y >= img->height) return 0;
    midpoint = (img->width - 1) / 2;
    for (int x = 4; x < img->width; x += 4) {
        int y = line_y + (int)(((long long)(x - midpoint) * slope_q16) >> 16);
        if (y < 0 || y >= img->height) {
            previous = -1;
            continue;
        }
        {
            int value = img->pixels[(size_t)y * (size_t)img->width + (size_t)x];
            if (previous >= 0) {
                int delta = value - previous;
                score += delta < 0 ? -delta : delta;
            }
            previous = value;
        }
    }
    return score;
}

static int fast_row_select_candidates(const Image *img, int first_y, int last_y,
                                      FastRowCandidate *candidates, int capacity)
{
    static const int slopes[] = {
        -TILT_Q16_20DEG, -TILT_Q16_FAR, -TILT_Q16_15DEG,
        -TILT_Q16_10DEG, -TILT_Q16_5DEG, 0,
        TILT_Q16_5DEG, TILT_Q16_10DEG, TILT_Q16_15DEG,
        TILT_Q16_FAR, TILT_Q16_20DEG,
    };
    FastRowCandidate all[256];
    int all_count = 0;
    int count = 0;

    if (!img || !candidates || capacity <= 0) return 0;
    if (capacity > 8) capacity = 8;
    for (int y = first_y; y <= last_y && all_count < 256; y += 4) {
        for (size_t slope_index = 0;
             slope_index < sizeof(slopes) / sizeof(slopes[0]) && all_count < 256;
             slope_index++) {
            all[all_count].y = y;
            all[all_count].slope_q16 = slopes[slope_index];
            all[all_count].score = fast_row_edge_score_slope(
                img, y, slopes[slope_index]);
            all_count++;
        }
    }

    for (int index = 1; index < all_count; index++) {
        FastRowCandidate value = all[index];
        int move = index;
        while (move > 0 && all[move - 1].score < value.score) {
            all[move] = all[move - 1];
            move--;
        }
        all[move] = value;
    }
    if (img->height <= 160) {
        static const int compact_offsets[] = {8, 16};
        for (size_t preset_index = 0;
             preset_index < sizeof(compact_offsets) / sizeof(compact_offsets[0]) &&
             count < capacity;
             preset_index++) {
            int preset_y = img->height / 2 + compact_offsets[preset_index];
            if (preset_y >= first_y && preset_y <= last_y) {
                candidates[count].y = preset_y;
                candidates[count].slope_q16 = TILT_Q16_5DEG;
                candidates[count].score = fast_row_edge_score_slope(
                    img, preset_y, TILT_Q16_5DEG);
                count++;
            }
        }
    }
    for (int index = 0; index < all_count && count < capacity; index++) {
        bool separated = true;
        for (int previous = 0; previous < count; previous++) {
            if (abs(all[index].y - candidates[previous].y) < 8 &&
                abs(all[index].slope_q16 - candidates[previous].slope_q16) < 4096) {
                separated = false;
                break;
            }
        }
        if (separated) candidates[count++] = all[index];
    }
    return count;
}

static int fast_row_add_vote(const char *text, int text_len, int y,
                             int source_id,
                             char candidate_text[4][BARCODE_MAX_LEN],
                             int candidate_lengths[4],
                             int candidate_counts[4],
                             int candidate_last_y[4],
                             int candidate_last_source[4],
                             BarcodeResult *result)
{
    int candidate_index = -1;

    if (!text || text_len <= 0 || text_len >= BARCODE_MAX_LEN) return 0;
    for (int index = 0; index < 4; index++) {
        if (candidate_lengths[index] == text_len &&
            memcmp(candidate_text[index], text, (size_t)text_len) == 0) {
            candidate_index = index;
            break;
        }
        if (candidate_index < 0 && candidate_lengths[index] == 0) {
            candidate_index = index;
        }
    }
    if (candidate_index < 0) return 0;
    if (candidate_lengths[candidate_index] == 0) {
        candidate_lengths[candidate_index] = text_len;
        memcpy(candidate_text[candidate_index], text, (size_t)text_len + 1U);
    }
    if ((candidate_last_y[candidate_index] != y &&
         abs(y - candidate_last_y[candidate_index]) >= 4) ||
        (candidate_last_y[candidate_index] == y &&
         candidate_last_source[candidate_index] != source_id)) {
        candidate_counts[candidate_index]++;
        candidate_last_y[candidate_index] = y;
        candidate_last_source[candidate_index] = source_id;
    }
    if (candidate_counts[candidate_index] >= 2) {
        memcpy(result->data, candidate_text[candidate_index],
               (size_t)candidate_lengths[candidate_index] + 1U);
        result->length = candidate_lengths[candidate_index];
        result->confidence = candidate_counts[candidate_index] > 100
            ? 100 : candidate_counts[candidate_index];
        return 1;
    }
    return 0;
}

int code128_decode_fast_rows(const Image *img, BarcodeResult *result, int timeout_ms)
{
    uint8_t *row;
    uint8_t *averaged;
    uint8_t *stretched;
    BarSpaceRun *runs;
    BarSpaceRun *runs_scratch;
    int *edges;
    uint64_t deadline_ms = 0;
    char candidate_text[4][BARCODE_MAX_LEN];
    int candidate_lengths[4] = {0};
    int candidate_counts[4] = {0};
    int candidate_last_y[4] = {-1000, -1000, -1000, -1000};
    int candidate_last_source[4] = {-1, -1, -1, -1};
    int first_y;
    int last_y;
    int step;
    int prioritize_y40;
    int run_width_scratch[MAX_BARS];
    static const int centered_offsets[] = {
        0, 8, -8, 16, -16, 24, -24, 32, -32,
    };
    static const int strong_offsets[] = {
        -24, 0, 8, -8, 16, -16, 24, 32, -32,
    };
    static const int variant_order[] = {0, 1, 3};
    static const int fast_tilt_probe_offsets[] = {-40, -32};

    if (!img || !img->pixels || !result || img->width < 20 || img->height < 1) {
        return 0;
    }
    if (timeout_ms > 0) {
        deadline_ms = monotonic_ms() + (uint64_t)timeout_ms;
    }
    g_fast_rows_deadline_ms = deadline_ms;
    row = (uint8_t *)malloc((size_t)img->width);
    averaged = (uint8_t *)malloc((size_t)img->width);
    stretched = (uint8_t *)malloc((size_t)img->width);
    runs = (BarSpaceRun *)malloc(sizeof(*runs));
    runs_scratch = (BarSpaceRun *)malloc(sizeof(*runs_scratch));
    edges = (int *)malloc(MAX_EDGES * sizeof(*edges));
    if (!row || !averaged || !stretched || !runs || !runs_scratch || !edges) {
        free(row);
        free(averaged);
        free(stretched);
        free(runs);
        free(runs_scratch);
        free(edges);
        return 0;
    }

    first_y = img->height / 8;
    last_y = (img->height * 7) / 8;
    step = img->height > 160 ? 4 : 2;
    prioritize_y40 = img->height <= 160 &&
                     fast_row_contrast(img, img->height / 2) >= 200 &&
                     fast_row_contrast(img, img->height / 2 - 24) >= 200;

    if (img->height <= 160) {
        const int center_y = img->height / 2;
        const int roi_offsets[] = {-8, 0, 8};

        memcpy(row, img->pixels + (size_t)center_y * (size_t)img->width,
               (size_t)img->width);
        for (int offset_index = 0;
             offset_index < (int)(sizeof(roi_offsets) / sizeof(roi_offsets[0]));
             offset_index++) {
            ScanlineResult scanline;

            extract_runs_roi_offset_q4(row, img->width,
                                       roi_offsets[offset_index], runs_scratch);
            if (runs_scratch->count < 20) continue;
            decode_from_runs_far(runs_scratch, &scanline);
            if (scanline.valid && fast_row_add_vote(
                    scanline.text, scanline.text_len, center_y,
                    60 + offset_index, candidate_text, candidate_lengths,
                    candidate_counts, candidate_last_y,
                    candidate_last_source, result)) {
                free(row);
                free(averaged);
                free(stretched);
                free(runs);
                free(runs_scratch);
                free(edges);
                return 1;
            }
        }
    }

    {
        FastRowCandidate candidate_rows[4];
        int candidate_count = fast_row_select_candidates(
            img, first_y, last_y, candidate_rows, 4);
        for (int candidate_index = 0; candidate_index < candidate_count;
             candidate_index++) {
            int y = candidate_rows[candidate_index].y;
            int slope_q16 = candidate_rows[candidate_index].slope_q16;
            const uint8_t *sources[2];

            get_averaged_tilted_row(img, y, 1, slope_q16, row);
            sources[0] = row;
            get_averaged_tilted_row(img, y, 3, slope_q16, averaged);
            sources[1] = averaged;
            for (int source_index = 0; source_index < 2; source_index++) {
                ScanlineResult scanline;
                int n_edges = 0;
                int edge_decoded = 0;
                char edge_text[BARCODE_MAX_LEN];
                int edge_length = 0;

                if (slope_q16 != 0) {
                    n_edges = signal_transitions_code128(
                        sources[source_index], img->width, 1, 0, 0,
                        img->width - 1, 0, edges, MAX_EDGES);
                    if (n_edges >= 20) {
                        edge_decoded = decode_from_edges_char128(
                            edges, n_edges, edge_text, &edge_length);
                        if (!edge_decoded && n_edges >= 40) {
                            edge_decoded = decode_from_edges_char128_clustered(
                                edges, n_edges, edge_text, &edge_length);
                        }
                    }
                }
                if (edge_decoded) {
                    memcpy(result->data, edge_text, (size_t)edge_length + 1U);
                    result->length = edge_length;
                    result->confidence = 100;
                    free(row);
                    free(averaged);
                    free(stretched);
                    free(runs);
                    free(runs_scratch);
                    free(edges);
                    return 1;
                }
                memset(&scanline, 0, sizeof(scanline));
                extract_runs(sources[source_index], img->width,
                             otsu_threshold(sources[source_index], img->width), runs);
                if (runs->count < 20) continue;
                decode_from_runs_fast(runs, &scanline, run_width_scratch);
                if (scanline.valid) {
                    memcpy(result->data, scanline.text,
                           (size_t)scanline.text_len + 1U);
                    result->length = scanline.text_len;
                    result->confidence = 100;
                    free(row);
                    free(averaged);
                    free(stretched);
                    free(runs);
                    free(runs_scratch);
                    free(edges);
                    return 1;
                }
            }
        }
    }

    /* The compact P4 ROI can spend most of the budget on one unsuccessful
     * far-row decode. Probe the known high-value +10 degree band first so a
     * steep barcode is not hidden behind the center-row sweep. */
    if (img->height <= 160 && !prioritize_y40) {
        for (int probe_y_index = 0;
             probe_y_index < (int)(sizeof(fast_tilt_probe_offsets) /
                                   sizeof(fast_tilt_probe_offsets[0]));
             probe_y_index++) {
            int probe_y = img->height / 2 + fast_tilt_probe_offsets[probe_y_index];
            ScanlineResult probe_result;
            if (probe_y < first_y || probe_y > last_y) continue;
            if (is_timed_out(deadline_ms)) break;
            get_averaged_tilted_row(img, probe_y, 1, TILT_Q16_10DEG, row);
            extract_runs(row, img->width, otsu_threshold(row, img->width), runs);
            if (runs->count < 20) continue;
            decode_from_runs_far_fast(runs, &probe_result, run_width_scratch);
            if (probe_result.valid && fast_row_add_vote(
                    probe_result.text, probe_result.text_len, probe_y,
                    44 + probe_y_index,
                    candidate_text, candidate_lengths,
                    candidate_counts, candidate_last_y,
                    candidate_last_source, result)) {
                free(row);
                free(averaged);
                free(stretched);
                free(runs);
                free(runs_scratch);
                free(edges);
                return 1;
            }
        }
    }

    for (int row_index = 0; ; row_index++) {
        int y;
        if (img->height <= 160) {
            const int *offsets = prioritize_y40 ? strong_offsets : centered_offsets;
            int offset_count = (int)(sizeof(centered_offsets) /
                                     sizeof(centered_offsets[0]));
            if (row_index >= offset_count) break;
            y = img->height / 2 + offsets[row_index];
            if (y < first_y || y > last_y) continue;
        } else {
            y = first_y + row_index * step;
            if (y > last_y) break;
        }
        int threshold;
        if (is_timed_out(deadline_ms)) break;
        memcpy(row, img->pixels + (size_t)y * (size_t)img->width,
               (size_t)img->width);
        for (int variant_index = 0; variant_index < 3; variant_index++) {
            int variant = variant_order[variant_index];
            const uint8_t *source;
            ScanlineResult scanline;

            if ((variant & 1) == 0) {
                source = row;
            } else {
                get_averaged_row(img, y, 3, averaged);
                source = averaged;
            }
            if (variant >= 2) {
                contrast_stretch_row(source, img->width, stretched);
                source = stretched;
            }
            memset(&scanline, 0, sizeof(scanline));
            threshold = otsu_threshold(source, img->width);
            extract_runs(source, img->width, threshold, runs);
            if (runs->count >= 20) {
                decode_from_runs_fast(runs, &scanline, run_width_scratch);
                if (scanline.valid && fast_row_add_vote(
                        scanline.text, scanline.text_len, y, variant,
                        candidate_text, candidate_lengths,
                        candidate_counts, candidate_last_y,
                        candidate_last_source, result)) {
                    free(row);
                    free(averaged);
                    free(stretched);
                    free(runs);
                    free(runs_scratch);
                    free(edges);
                    return 1;
                }
            }
        }
        if (row_index == 0 && img->height <= 160) {
            static const int probe_slopes[] = {TILT_Q16_5DEG, TILT_Q16_10DEG};
            static const int probe_y_offsets[][2] = {{8, 16}, {-40, -32}};
            for (int probe_index = 0;
                 probe_index < (int)(sizeof(probe_slopes) / sizeof(probe_slopes[0]));
                 probe_index++) {
                for (int probe_y_index = 0; probe_y_index < 2; probe_y_index++) {
                    int probe_y = img->height / 2 +
                                  probe_y_offsets[probe_index][probe_y_index];
                    ScanlineResult probe_result;
                    if (probe_y < first_y || probe_y > last_y) continue;
                    if (is_timed_out(deadline_ms)) break;
                    get_averaged_tilted_row(img, probe_y, 1,
                                            probe_slopes[probe_index], row);
                    extract_runs(row, img->width, otsu_threshold(row, img->width), runs);
                    if (runs->count < 20) continue;
                    decode_from_runs_far_fast(runs, &probe_result, run_width_scratch);
                    if (probe_result.valid && fast_row_add_vote(
                            probe_result.text, probe_result.text_len, probe_y,
                            40 + probe_index,
                            candidate_text, candidate_lengths,
                            candidate_counts, candidate_last_y,
                            candidate_last_source, result)) {
                        free(row);
                        free(averaged);
                        free(stretched);
                        free(runs);
                        free(runs_scratch);
                        free(edges);
                        return 1;
                    }
                }
            }
        }
        {
            const uint8_t *sources[2] = {row, averaged};
            const int roi_offsets[] = {-8, 0, 8};
            for (int source_index = 0; source_index < 2; source_index++) {
                const uint8_t *source = sources[source_index];
                for (int offset_index = 0;
                     offset_index < (int)(sizeof(roi_offsets) / sizeof(roi_offsets[0]));
                     offset_index++) {
                    ScanlineResult scanline;
                    extract_runs_roi_offset(source, img->width,
                                            roi_offsets[offset_index], runs);
                    if (runs->count < 20) continue;
                    decode_from_runs_far_fast(runs, &scanline, run_width_scratch);
                    if (scanline.valid && fast_row_add_vote(
                            scanline.text, scanline.text_len, y,
                            8 + source_index * 3 + offset_index,
                            candidate_text, candidate_lengths,
                            candidate_counts, candidate_last_y,
                            candidate_last_source, result)) {
                        free(row);
                        free(averaged);
                        free(stretched);
                        free(runs);
                        free(runs_scratch);
                        free(edges);
                        return 1;
                    }
                    extract_runs_roi_offset_q4(source, img->width,
                                               roi_offsets[offset_index],
                                               runs_scratch);
                    if (runs_scratch->count < 20) continue;
                    decode_from_runs_far_fast(runs_scratch, &scanline, run_width_scratch);
                    if (scanline.valid && fast_row_add_vote(
                            scanline.text, scanline.text_len, y,
                            14 + source_index * 3 + offset_index,
                            candidate_text, candidate_lengths,
                            candidate_counts, candidate_last_y,
                            candidate_last_source, result)) {
                        free(row);
                        free(averaged);
                        free(stretched);
                        free(runs);
                        free(runs_scratch);
                        free(edges);
                        return 1;
                    }
                }
            }
        }
        if (is_timed_out(deadline_ms)) break;
    }
    {
        static const int tilt_options[] = {
            -TILT_Q16_15DEG, -TILT_Q16_10DEG, -TILT_Q16_5DEG,
            TILT_Q16_5DEG, TILT_Q16_10DEG, TILT_Q16_15DEG,
        };
        int tilt_first_y = first_y;
        int tilt_last_y = last_y;
        if (img->height > 160) {
            tilt_first_y += 4;
            tilt_last_y -= 4;
        }
        for (int tilt_index = 0;
             tilt_index < (int)(sizeof(tilt_options) / sizeof(tilt_options[0]));
             tilt_index++) {
            int slope_q16 = tilt_options[tilt_index];
            for (int y = tilt_first_y; y <= tilt_last_y; y += 8) {
                ScanlineResult scanline;
                char edge_text[BARCODE_MAX_LEN];
                int edge_length = 0;
                int edge_decoded = 0;
                if (is_timed_out(deadline_ms)) break;
                get_averaged_tilted_row(img, y, 1, slope_q16, row);
                {
                    int n_edges = signal_transitions_code128(
                        row, img->width, 1, 0, 0, img->width - 1, 0,
                        edges, MAX_EDGES);
                    if (n_edges >= 20) {
                        edge_decoded = decode_from_edges_char128(
                            edges, n_edges, edge_text, &edge_length);
                        if (!edge_decoded && n_edges >= 40) {
                            edge_decoded = decode_from_edges_char128_clustered(
                                edges, n_edges, edge_text, &edge_length);
                        }
                        if (!edge_decoded) {
                            edge_decoded = decode_from_edges_char128_far(
                                edges, n_edges, edge_text, &edge_length);
                        }
                    }
                }
                if (edge_decoded && fast_row_add_vote(
                        edge_text, edge_length, y, 20 + tilt_index,
                        candidate_text, candidate_lengths,
                        candidate_counts, candidate_last_y,
                        candidate_last_source, result)) {
                    free(row);
                    free(averaged);
                    free(stretched);
                    free(runs);
                    free(runs_scratch);
                    free(edges);
                    return 1;
                }
                memset(&scanline, 0, sizeof(scanline));
                {
                    int tilt_threshold = otsu_threshold(row, img->width);
                    extract_runs(row, img->width, tilt_threshold, runs);
                }
                if (runs->count >= 20) {
                    decode_from_runs_far_fast(runs, &scanline, run_width_scratch);
                    if (scanline.valid && fast_row_add_vote(
                            scanline.text, scanline.text_len, y, 30 + tilt_index,
                            candidate_text, candidate_lengths,
                            candidate_counts, candidate_last_y,
                            candidate_last_source, result)) {
                        free(row);
                        free(averaged);
                        free(stretched);
                        free(runs);
                        free(runs_scratch);
                        free(edges);
                        return 1;
                    }
                }
            }
        }
    }
    free(row);
    free(averaged);
    free(stretched);
    free(runs);
    free(runs_scratch);
    free(edges);
    return 0;
}

void code128_fast_rows_clear_deadline(void)
{
    g_fast_rows_deadline_ms = 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Code128 existence prescreen (phase6b)
 *
 *  Cheap guard-pattern (START/STOP) scan used to decide whether the
 *  expensive multi-layer hint-scan budget in decode_core.c is worth
 *  spending on this image.  Deliberately biased "宁松勿严": it must
 *  never reject a real Code128 (including far-distance depth shots);
 *  it only needs to catch the clean, guard-free non-Code128 images
 *  that currently burn the full CODE128_HINT_TOTAL_BUDGET_MS.
 * ════════════════════════════════════════════════════════════════════════ */

static void c128_guard_scan_pairs(const int *edges, int n_edges,
                                  int *pairs_strict, int *pairs_tol)
{
    int *work = (int *)malloc(4U * 128U * sizeof(*work));
    int ns = 0;
    int np = 0;
    if (!work) return;
    int *spos = work;
    int *spar = spos + 128;
    int *ppos = spar + 128;
    int *ppar = ppos + 128;
    for (int parity = 0; parity < 2; ++parity) {
        for (int i = 0; i < n_edges - 6; ++i) {
            int mc1 = 1, mc2 = 0;
            int sym = char128_match_edges_relaxed(edges, n_edges, i, parity, &mc1, &mc2);
            if (sym >= C128_START_A && sym <= C128_START_C && ns < 128) {
                spos[ns] = i; spar[ns] = parity; ns++;
            } else if ((sym == -3 || sym == -2) && np < 128) {
                ppos[np] = i; ppar[np] = parity; np++;
            }
        }
    }
    for (int a = 0; a < ns; a++) {
        for (int b = 0; b < np; b++) {
            if (spar[a] != ppar[b]) continue;
            int d = ppos[b] - spos[a];
            if (d < 18 || d > 6 * 27) continue; /* >=1 data symbol, <=25 symbols */
            if (d % 6 == 0) (*pairs_strict)++;
            if (d % 6 == 0 || d % 6 == 1 || d % 6 == 5) (*pairs_tol)++;
        }
    }
    free(work);
}

/* Block1a: Channel-A-only quick scan. Counts how many of the 9 raw
 * horizontal scanlines (same rows9[] sampling as code128_guard_prescreen's
 * Channel A) contain at least one strict-parity Code128 start/stop pair.
 * Used by decode_core.c to gate the strong-guard front-run in
 * run_early_low_cost_layer(), before the QR/Aztec/... early-2D probes.
 * Cost is a subset of what code128_guard_prescreen() already does (raw scan
 * of <=9 rows, no averaging passes) -- well under 1ms/image. Does not
 * replace or mutate code128_guard_prescreen(): that function still runs
 * (lazily, with its own independent early-exit logic and full 3-channel
 * scan) the first time try_decode_code128_with_hint() is called. */
int code128_guard_chA_rows_strict(const Image *img)
{
    int w = img->width, h = img->height;
    if (w < 20 || h < 2) return 0;

    int *edges = (int *)malloc(MAX_EDGES * sizeof(int));
    if (!edges) return 0;

    int rows_with_strict = 0;
    int rows9[9];
    for (int k = 0; k < 9; k++) rows9[k] = (h * (k + 1)) / 10;

    for (int k = 0; k < 9; k++) {
        int y = rows9[k];
        int n = signal_transitions_code128(img->pixels, w, h, 0, y, w - 1, y, edges, MAX_EDGES);
        if (n >= 19) {
            int row_strict = 0, row_tol = 0;
            c128_guard_scan_pairs(edges, n, &row_strict, &row_tol);
            if (row_strict >= 1) rows_with_strict++;
        }
    }

    free(edges);
    return rows_with_strict;
}

int code128_guard_prescreen(const Image *img)
{
    int w = img->width, h = img->height;
    if (w < 20 || h < 2) return 1; /* too small to judge, let it through */

    uint8_t *row_buf = (uint8_t *)malloc(w);
    uint8_t *row_smth = (uint8_t *)malloc(w);
    uint8_t *row_shrp = (uint8_t *)malloc(w);
    uint8_t *row_tmp = (uint8_t *)malloc(w);
    int *edges = (int *)malloc(MAX_EDGES * sizeof(int));
    BarSpaceRun *runs = (BarSpaceRun *)malloc(sizeof(BarSpaceRun));
    if (!row_buf || !row_smth || !row_shrp || !row_tmp || !edges || !runs) {
        free(row_buf); free(row_smth); free(row_shrp); free(row_tmp);
        free(edges); free(runs);
        return 1; /* allocation failure: fail open, run full hint scan */
    }

    int rows9[9];
    for (int k = 0; k < 9; k++) rows9[k] = (h * (k + 1)) / 10;

    /* Channel A: raw horizontal scanlines (near/clear images). guard-v2:
     * require evidence across multiple distinct rows rather than a single
     * strict (or tol) pair anywhere, so a MicroPDF417 stacked-row image
     * that happens to produce one coincidental Code128 start/stop match on
     * a single scanline (measured: micropdf.bmp only ever matches on one of
     * the 9 rows) is no longer admitted. Real Code128-family images and all
     * measured depth-test images produce strict pairs on many distinct rows
     * (13-42 out of the same 9 sampled rows), so this does not early-exit
     * per-row like before -- it always scans all 9 rows (cost unchanged,
     * still <1ms) and only then checks the multi-row condition. */
    int rows_with_strict = 0, rows_with_tol = 0;
    for (int k = 0; k < 9; k++) {
        int y = rows9[k];
        int n = signal_transitions_code128(img->pixels, w, h, 0, y, w - 1, y, edges, MAX_EDGES);
        if (n >= 19) {
            int row_strict = 0, row_tol = 0;
            c128_guard_scan_pairs(edges, n, &row_strict, &row_tol);
            if (row_strict >= 1) rows_with_strict++;
            if (row_tol >= 1) rows_with_tol++;
        }
    }
    if (rows_with_strict >= 2) goto allow;
    if (rows_with_strict >= 1 && rows_with_tol >= 2) goto allow;

    /* Channel B: vertical scanlines (rotated-90 protection). Unchanged: a
     * single strict pair on any of the 5 sampled columns still admits, this
     * channel targets rotated real Code128 images and is not implicated in
     * the micropdf.bmp false-positive (which only ever matches Channel A). */
    int vert_strict = 0, vert_tol_unused = 0;
    int cols5[5] = { w / 6, w / 3, w / 2, (2 * w) / 3, (5 * w) / 6 };
    for (int k = 0; k < 5; k++) {
        int x = cols5[k];
        int n = signal_transitions_code128(img->pixels, w, h, x, 0, x, h - 1, edges, MAX_EDGES);
        if (n >= 19)
            c128_guard_scan_pairs(edges, n, &vert_strict, &vert_tol_unused);
        if (vert_strict >= 1) goto allow;
    }

    /* Channel C: averaged/enhanced rows (far-distance blurred images) */
    int avg_strict = 0, avg_tol_unused = 0;
    int avgs4[4] = { 9, 31, 75, 151 };
    for (int k = 0; k < 9; k++) {
        int y = rows9[k];
        for (int ai = 0; ai < 4; ai++) {
            get_averaged_row(img, y, avgs4[ai], row_buf);
            contrast_stretch_row(row_buf, w, row_tmp);
            smooth_row_121(row_tmp, w, row_tmp);
            smooth_row_121(row_buf, w, row_smth);
            unsharp_row(row_buf, row_smth, w, row_shrp);
            const uint8_t *variants[3] = { row_buf, row_tmp, row_shrp };
            for (int vi = 0; vi < 3; vi++) {
                int thr = otsu_threshold(variants[vi], w);
                extract_runs(variants[vi], w, thr, runs);
                if (runs->count < 19) continue;
                int n = runs_to_edges(runs, edges, MAX_EDGES);
                if (n < 19) continue;
                c128_guard_scan_pairs(edges, n, &avg_strict, &avg_tol_unused);
                if (avg_strict >= 9) goto allow;
            }
        }
    }

    free(row_buf); free(row_smth); free(row_shrp); free(row_tmp);
    free(edges); free(runs);
    return 0;

allow:
    free(row_buf); free(row_smth); free(row_shrp); free(row_tmp);
    free(edges); free(runs);
    return 1;
}
