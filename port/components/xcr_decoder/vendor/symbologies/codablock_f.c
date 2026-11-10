/*
 * Codablock F: stacked multi-row Code128 barcode.
 *
 * Self-contained decoder (does not reuse code128.c internals, which are
 * static and only return flattened single-line text). Each row is decoded
 * independently as a Code128 codeword sequence:
 *
 *   [StartA][subset selector][row indicator][7 data codewords][row check][Stop]
 *
 * Subset selector codeword: 98 = subset A, 99 = subset C, 100 = subset B.
 * Row indicator encodes a value v via the subset (A/B or C) mapping:
 *   v <= 31        -> codeword v+64
 *   32 <= v <= 47  -> codeword v-32
 *   v >= 48        -> codeword v-22
 *   (subset C: codeword == v directly)
 * v == totalRows-2 for the first row, v == rowIndex+42 for row rowIndex>=1.
 *
 * The last row's final two data codewords (positions 5,6 of the 7 data
 * codewords) are K1/K2 mod-86 checksums over the assembled payload text,
 * encoded with the same value/subset mapping as the row indicator.
 */

#include "codablock_f.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define XCR_PORT_REMAP_ALLOC 1
#include "xcr_port.h"

/* ── Standard Code128 symbol tables (public ISO/IEC 15417 constants) ─── */

static const int CBF_PATTERNS[106][6] = {
    {2,1,2,2,2,2}, {2,2,2,1,2,2}, {2,2,2,2,2,1}, {1,2,1,2,2,3},
    {1,2,1,3,2,2}, {1,3,1,2,2,2}, {1,2,2,2,1,3}, {1,2,2,3,1,2},
    {1,3,2,2,1,2}, {2,2,1,2,1,3}, {2,2,1,3,1,2}, {2,3,1,2,1,2},
    {1,1,2,2,3,2}, {1,2,2,1,3,2}, {1,2,2,2,3,1}, {1,1,3,2,2,2},
    {1,2,3,1,2,2}, {1,2,3,2,2,1}, {2,2,3,2,1,1}, {2,2,1,1,3,2},
    {2,2,1,2,3,1}, {2,1,3,2,1,2}, {2,2,3,1,1,2}, {3,1,2,1,3,1},
    {3,1,1,2,2,2}, {3,2,1,1,2,2}, {3,2,1,2,2,1}, {3,1,2,2,1,2},
    {3,2,2,1,1,2}, {3,2,2,2,1,1}, {2,1,2,1,2,3}, {2,1,2,3,2,1},
    {2,3,2,1,2,1}, {1,1,1,3,2,3}, {1,3,1,1,2,3}, {1,3,1,3,2,1},
    {1,1,2,3,1,3}, {1,3,2,1,1,3}, {1,3,2,3,1,1}, {2,1,1,3,1,3},
    {2,3,1,1,1,3}, {2,3,1,3,1,1}, {1,1,2,1,3,3}, {1,1,2,3,3,1},
    {1,3,2,1,3,1}, {1,1,3,1,2,3}, {1,1,3,3,2,1}, {1,3,3,1,2,1},
    {3,1,3,1,2,1}, {2,1,1,3,3,1}, {2,3,1,1,3,1}, {2,1,3,1,1,3},
    {2,1,3,3,1,1}, {2,1,3,1,3,1}, {3,1,1,1,2,3}, {3,1,1,3,2,1},
    {3,3,1,1,2,1}, {3,1,2,1,1,3}, {3,1,2,3,1,1}, {3,3,2,1,1,1},
    {3,1,4,1,1,1}, {2,2,1,4,1,1}, {4,3,1,1,1,1}, {1,1,1,2,2,4},
    {1,1,1,4,2,2}, {1,2,1,1,2,4}, {1,2,1,4,2,1}, {1,4,1,1,2,2},
    {1,4,1,2,2,1}, {1,1,2,2,1,4}, {1,1,2,4,1,2}, {1,2,2,1,1,4},
    {1,2,2,4,1,1}, {1,4,2,1,1,2}, {1,4,2,2,1,1}, {2,4,1,2,1,1},
    {2,2,1,1,1,4}, {4,1,3,1,1,1}, {2,4,1,1,1,2}, {1,3,4,1,1,1},
    {1,1,1,2,4,2}, {1,2,1,1,4,2}, {1,2,1,2,4,1}, {1,1,4,2,1,2},
    {1,2,4,1,1,2}, {1,2,4,2,1,1}, {4,1,1,2,1,2}, {4,2,1,1,1,2},
    {4,2,1,2,1,1}, {2,1,2,1,4,1}, {2,1,4,1,2,1}, {4,1,2,1,2,1},
    {1,1,1,1,4,3}, {1,1,1,3,4,1}, {1,3,1,1,4,1}, {1,1,4,1,1,3},
    {1,1,4,3,1,1}, {4,1,1,1,1,3}, {4,1,1,3,1,1}, {1,1,3,1,4,1},
    {1,1,4,1,3,1}, {3,1,1,1,4,1}, {4,1,1,1,3,1}, {2,1,1,4,1,2},
    {2,1,1,2,1,4}, {2,1,1,2,3,2},
};

static const int CBF_STOP[7] = {2,3,3,1,1,1,2};

enum {
    CBF_SHIFT   = 98,
    CBF_CODE_C  = 99,
    CBF_CODE_B  = 100,
    CBF_CODE_A  = 101,
    CBF_FNC1    = 102,
};

enum { CBF_SUBSET_A = 0, CBF_SUBSET_B = 1, CBF_SUBSET_C = 2 };

/* ── Run-length extraction ──────────────────────────────────────────── */

#define CBF_MAX_RUNS 2048

typedef struct {
    int val[CBF_MAX_RUNS]; /* 0 = background, 1 = foreground(bar) */
    int len[CBF_MAX_RUNS];
    int count;
} CBFRuns;

static int extract_runs(const uint8_t *row, int width, int threshold, int invert, CBFRuns *out)
{
    int x;
    int cur_val;
    int run_start;

    out->count = 0;
    if (width < 2) return 0;

    cur_val = ((row[0] < threshold) ? 1 : 0) ^ invert;
    run_start = 0;
    for (x = 1; x < width; x++) {
        int v = ((row[x] < threshold) ? 1 : 0) ^ invert;
        if (v != cur_val) {
            if (out->count >= CBF_MAX_RUNS) return out->count;
            out->val[out->count] = cur_val;
            out->len[out->count] = x - run_start;
            out->count++;
            cur_val = v;
            run_start = x;
        }
    }
    if (out->count < CBF_MAX_RUNS) {
        out->val[out->count] = cur_val;
        out->len[out->count] = width - run_start;
        out->count++;
    }
    return out->count;
}

/* ── Pattern quantization / lookup ──────────────────────────────────── */

static int quantize_group(const int *lens, int n, int modules_total, int *pattern_out)
{
    int sum = 0;
    int i;
    int qsum = 0;

    for (i = 0; i < n; i++) sum += lens[i];
    if (sum <= 0) return 0;
    if (sum * 2 < modules_total) return 0;

    /* Integer-only: q = round(lens[i] * modules_total / sum), identical to the
     * original double math but no emulated soft-float on the P4 (rv32imafc has
     * no double FPU), which dominated the codablock veto cost. */
    for (i = 0; i < n; i++) {
        int q = (2 * lens[i] * modules_total + sum) / (2 * sum);
        if (q < 1) q = 1;
        if (q > 4) q = 4;
        pattern_out[i] = q;
        qsum += q;
    }
    return (qsum == modules_total) ? 1 : 0;
}

static int lookup_codeword(const int *pattern6)
{
    int s;
    for (s = 0; s < 106; s++) {
        if (memcmp(pattern6, CBF_PATTERNS[s], sizeof(int) * 6) == 0) return s;
    }
    return -1;
}

static int match_stop(const int *pattern7)
{
    int i;
    for (i = 0; i < 7; i++) {
        if (pattern7[i] != CBF_STOP[i]) return 0;
    }
    return 1;
}

/* ── Single-row decode attempt ──────────────────────────────────────── */

#define CBF_ROW_DATA_CODEWORDS 7
#define CBF_ROW_TOTAL_CODEWORDS (1 /*selector*/ + 1 /*indicator*/ + CBF_ROW_DATA_CODEWORDS)

typedef struct {
    int codewords[CBF_ROW_TOTAL_CODEWORDS]; /* selector, indicator, data0..6 */
    int checksum;
} CBFRowCandidate;

/* Try to decode one Codablock-F row starting at run index `start` (which
 * must be the first bar of a StartA pattern). Returns 1 on success. */
static int try_decode_row_at(const CBFRuns *runs, int start, CBFRowCandidate *out)
{
    int pattern[7];
    int idx = start;
    int k;
    int expected_checksum;

    if (idx + 6 > runs->count) return 0;
    if (!quantize_group(&runs->len[idx], 6, 11, pattern)) return 0;
    {
        static const int start_a[6] = {2,1,1,4,1,2};
        int i;
        for (i = 0; i < 6; i++) {
            if (pattern[i] != start_a[i]) return 0;
        }
    }
    idx += 6;

    for (k = 0; k < CBF_ROW_TOTAL_CODEWORDS; k++) {
        int sym;
        if (idx + 6 > runs->count) return 0;
        if (!quantize_group(&runs->len[idx], 6, 11, pattern)) return 0;
        sym = lookup_codeword(pattern);
        if (sym < 0 || sym > 102) return 0;
        out->codewords[k] = sym;
        idx += 6;
    }

    /* row check codeword */
    if (idx + 6 > runs->count) return 0;
    if (!quantize_group(&runs->len[idx], 6, 11, pattern)) return 0;
    {
        int chk = lookup_codeword(pattern);
        if (chk < 0 || chk > 102) return 0;
        out->checksum = chk;
    }
    idx += 6;

    /* stop pattern */
    {
        int pattern7[7];
        if (idx + 7 > runs->count) return 0;
        if (!quantize_group(&runs->len[idx], 7, 13, pattern7)) return 0;
        if (!match_stop(pattern7)) return 0;
    }

    /* mod-103 row checksum: start(103) + sum(k * codeword[k]) for k=1..9 */
    expected_checksum = 103;
    for (k = 0; k < CBF_ROW_TOTAL_CODEWORDS; k++) {
        expected_checksum += (k + 1) * out->codewords[k];
    }
    expected_checksum %= 103;
    if (expected_checksum != out->checksum) return 0;

    return 1;
}

static int scan_row_for_codablock(const CBFRuns *runs, CBFRowCandidate *out)
{
    int i;
    for (i = 0; i + 6 <= runs->count; i++) {
        if (runs->val[i] != 1) continue; /* codewords start with a bar */
        if (try_decode_row_at(runs, i, out)) return 1;
    }
    return 0;
}

/* ── Row indicator / K1 / K2 value mapping ──────────────────────────── */

static int subset_from_selector(int codeword)
{
    if (codeword == CBF_SHIFT) return CBF_SUBSET_A; /* selector value 98 = subset A */
    if (codeword == CBF_CODE_C) return CBF_SUBSET_C; /* 99 = subset C */
    if (codeword == CBF_CODE_B) return CBF_SUBSET_B; /* 100 = subset B */
    return -1;
}

static int indicator_encode(int v, int subset)
{
    if (subset == CBF_SUBSET_C) return v;
    if (v <= 31) return v + 64;
    if (v <= 47) return v - 32;
    return v - 22;
}

static int indicator_decode(int codeword, int subset)
{
    int v;
    if (subset == CBF_SUBSET_C) return codeword;
    for (v = 0; v <= 120; v++) {
        if (indicator_encode(v, subset) == codeword) return v;
    }
    return -1;
}

/* ── Codeword -> text state machine (per row) ───────────────────────── */

static int process_codewords(const int *cw, int n, int subset, char *out, int *out_len, int max_len)
{
    int i;
    int shift_active = 0;

    for (i = 0; i < n; i++) {
        int v = cw[i];
        int use_subset = subset;

        if (shift_active) {
            use_subset = (subset == CBF_SUBSET_A) ? CBF_SUBSET_B : CBF_SUBSET_A;
            shift_active = 0;
        }

        if (use_subset == CBF_SUBSET_C) {
            if (v <= 99) {
                if (*out_len + 2 > max_len) return subset;
                out[(*out_len)++] = (char)('0' + v / 10);
                out[(*out_len)++] = (char)('0' + v % 10);
            } else if (v == CBF_CODE_B) {
                subset = CBF_SUBSET_B;
            } else if (v == CBF_CODE_A) {
                subset = CBF_SUBSET_A;
            } /* FNC1 (102): no output */
            continue;
        }

        if (v <= 95) {
            int ascii;
            if (use_subset == CBF_SUBSET_B) {
                ascii = v + 32;
            } else {
                ascii = (v <= 63) ? (v + 32) : (v - 64);
            }
            if (*out_len + 1 > max_len) return subset;
            out[(*out_len)++] = (char)ascii;
        } else if (v == 96 || v == 97) {
            /* FNC3 / FNC2: no output */
        } else if (v == CBF_SHIFT) {
            shift_active = 1;
        } else if (v == CBF_CODE_C) {
            subset = CBF_SUBSET_C;
        } else if (v == CBF_CODE_B) {
            if (subset == CBF_SUBSET_A) subset = CBF_SUBSET_B; /* else FNC4, no-op */
        } else if (v == CBF_CODE_A) {
            if (subset == CBF_SUBSET_B) subset = CBF_SUBSET_A; /* else FNC4, no-op */
        } else if (v == CBF_FNC1) {
            /* no output */
        }
    }
    return subset;
}

/* ── Row aggregation across scanlines ───────────────────────────────── */

#define CBF_MAX_UNIQUE_ROWS 32
#define CBF_MIN_VOTES 3

typedef struct {
    CBFRowCandidate cand;
    int votes;
    double y_sum;
} CBFUniqueRow;

static int candidates_equal(const CBFRowCandidate *a, const CBFRowCandidate *b)
{
    if (a->checksum != b->checksum) return 0;
    return memcmp(a->codewords, b->codewords, sizeof(a->codewords)) == 0;
}

static void register_candidate(CBFUniqueRow *uniq, int *n_uniq, const CBFRowCandidate *cand, int y)
{
    int i;
    for (i = 0; i < *n_uniq; i++) {
        if (candidates_equal(&uniq[i].cand, cand)) {
            uniq[i].votes++;
            uniq[i].y_sum += y;
            return;
        }
    }
    if (*n_uniq >= CBF_MAX_UNIQUE_ROWS) return;
    uniq[*n_uniq].cand = *cand;
    uniq[*n_uniq].votes = 1;
    uniq[*n_uniq].y_sum = y;
    (*n_uniq)++;
}

static int compare_rows_by_y(const void *pa, const void *pb)
{
    const CBFUniqueRow *a = (const CBFUniqueRow *)pa;
    const CBFUniqueRow *b = (const CBFUniqueRow *)pb;
    double ay = a->y_sum / a->votes;
    double by = b->y_sum / b->votes;
    if (ay < by) return -1;
    if (ay > by) return 1;
    return 0;
}

/* ── Top level ───────────────────────────────────────────────────────── */

int codablock_f_decode(const Image *img, BarcodeResult *result)
{
    CBFUniqueRow *uniq = NULL;
    CBFRuns *runs = NULL;
    CBFRowCandidate *cand = NULL;
    int n_uniq = 0;
    int y;
    int i;
    char payload[BARCODE_MAX_LEN];
    int payload_len = 0;
    int n_rows;
    int total_rows_declared;
    int subset;
    int k1_raw, k2_raw, k1_v = -1, k2_v = -1;
    int k1_expected, k2_expected;
    int decoded = 0;

    if (!img || !result || !img->pixels) return 0;
    if (img->width < 40 || img->height < 20) return 0;

    memset(result, 0, sizeof(*result));

    uniq = (CBFUniqueRow *)calloc(CBF_MAX_UNIQUE_ROWS, sizeof(*uniq));
    runs = (CBFRuns *)malloc(sizeof(*runs));
    cand = (CBFRowCandidate *)malloc(sizeof(*cand));
    /* Row cache: gate rows are PSRAM-backed on the P4 (~10x cached-byte cost),
     * so reading each row three times (lo/hi + extract_runs per invert) is
     * wasteful. Copy the row once and run all passes from the cache. */
    uint8_t *row_cache = (uint8_t *)malloc((size_t)img->width);
    if (!uniq || !runs || !cand || !row_cache) {
        free(row_cache);
        goto cleanup;
    }

    for (y = 1; y < img->height - 1; y += 2) {
        const uint8_t *src = img->pixels + (size_t)y * img->width;
        int lo = 255, hi = 0;
        int x;
        int invert;

        memcpy(row_cache, src, (size_t)img->width);
        for (x = 0; x < img->width; x++) {
            if (row_cache[x] < lo) lo = row_cache[x];
            if (row_cache[x] > hi) hi = row_cache[x];
        }
        if (hi - lo < 20) continue;

        for (invert = 0; invert < 2; invert++) {
            int threshold = (lo + hi) / 2;

            if (!extract_runs(row_cache, img->width, threshold, invert, runs)) continue;
            if (scan_row_for_codablock(runs, cand)) {
                register_candidate(uniq, &n_uniq, cand, y);
            }
        }
    }
    free(row_cache);

    /* keep only rows with enough votes */
    n_rows = 0;
    for (i = 0; i < n_uniq; i++) {
        if (uniq[i].votes >= CBF_MIN_VOTES) {
            uniq[n_rows++] = uniq[i];
        }
    }
    if (n_rows < 2) goto cleanup;

    qsort(uniq, n_rows, sizeof(uniq[0]), compare_rows_by_y);

    /* validate row indicator chain */
    subset = subset_from_selector(uniq[0].cand.codewords[0]);
    if (subset < 0) goto cleanup;
    total_rows_declared = indicator_decode(uniq[0].cand.codewords[1], subset);
    if (total_rows_declared < 0 || total_rows_declared + 2 != n_rows) goto cleanup;

    for (i = 1; i < n_rows; i++) {
        int s = subset_from_selector(uniq[i].cand.codewords[0]);
        int v;
        if (s < 0) goto cleanup;
        v = indicator_decode(uniq[i].cand.codewords[1], s);
        if (v < 0 || v != i + 42) goto cleanup;
    }

    /* assemble payload text row by row */
    payload_len = 0;
    k1_raw = k2_raw = -1;
    for (i = 0; i < n_rows; i++) {
        const int *data = &uniq[i].cand.codewords[2]; /* 7 data codewords */
        int row_subset = subset_from_selector(uniq[i].cand.codewords[0]);
        int final_subset;

        if (i == n_rows - 1) {
            final_subset = process_codewords(data, CBF_ROW_DATA_CODEWORDS - 2, row_subset,
                                              payload, &payload_len, (int)sizeof(payload) - 1);
            k1_raw = data[CBF_ROW_DATA_CODEWORDS - 2];
            k2_raw = data[CBF_ROW_DATA_CODEWORDS - 1];
            k1_v = indicator_decode(k1_raw, final_subset);
            k2_v = indicator_decode(k2_raw, final_subset);
        } else {
            process_codewords(data, CBF_ROW_DATA_CODEWORDS, row_subset,
                               payload, &payload_len, (int)sizeof(payload) - 1);
        }
    }
    if (k1_raw < 0 || k1_v < 0 || k2_v < 0) goto cleanup;
    payload[payload_len] = '\0';

    /* K1/K2 mod-86 verification over assembled payload text */
    k1_expected = 0;
    k2_expected = 0;
    for (i = 0; i < payload_len; i++) {
        k1_expected += (i + 1) * (unsigned char)payload[i];
        k2_expected += i * (unsigned char)payload[i];
    }
    k1_expected %= 86;
    k2_expected %= 86;
    if (k1_expected != k1_v || k2_expected != k2_v) goto cleanup;

    if (payload_len <= 0 || payload_len >= BARCODE_MAX_LEN) goto cleanup;

    memcpy(result->data, payload, (size_t)payload_len);
    result->data[payload_len] = '\0';
    result->length = payload_len;
    result->confidence = 99;
    result->symbology = BARCODE_SYM_CODABLOCKF;
    decoded = 1;

cleanup:
    free(cand);
    free(runs);
    free(uniq);
    return decoded;
}
