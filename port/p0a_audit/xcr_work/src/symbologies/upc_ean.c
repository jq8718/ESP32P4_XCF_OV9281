/*
 * UPC/EAN Barcode Decoder
 * Restored from decompiled libFsaDecoderCore1 (ARM)
 *
 * Key source functions:
 *   TMod()      — line 230360 — bar width quantization
 *   CharUPC()   — line 282995 — single character decode
 *   TryAddend() — line 283100 — addendum decode (forward)
 *   sub_1879DC  — line 283300 — full barcode decode (forward)
 *   sub_187E70  — line 283400 — full barcode decode (reverse)
 *
 * Reference tables extracted from binary:
 *   dword_1876B4 — character decode LUT (16 bytes)
 *   dword_1876C4 — width correction table (20 bytes)
 *   byte_1A2E5B  — EAN-13 first digit parity table
 */

#include "upc_ean.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

/* ── Constants ────────────────────────────────────────────────────── */

/* UPC/EAN L-code patterns (standard encoding)
 * Each digit 0-9 has a 7-module pattern of bars and spaces.
 * L-codes (odd parity): used for left half of UPC-A, EAN-13 odd positions
 * G-codes (even parity): L-codes reversed, used for EAN-13 even positions
 * R-codes: complement of L-codes, used for right half
 *
 * Encoding: each pattern is {space, bar, space, bar} widths summing to 7
 */

/* L-code patterns: digit -> {s1, b1, s2, b2} */
static const int L_PATTERNS[10][4] = {
    {3,2,1,1},  /* 0 */
    {2,2,2,1},  /* 1 */
    {2,1,2,2},  /* 2 */
    {1,4,1,1},  /* 3 */
    {1,1,3,2},  /* 4 */
    {1,2,3,1},  /* 5 */
    {1,1,1,4},  /* 6 */
    {1,3,1,2},  /* 7 */
    {1,2,1,3},  /* 8 */
    {3,1,1,2},  /* 9 */
};

/* EAN-13 first digit parity encoding
 * byte_1A2E5B from decompiled source
 * Each value is a 6-bit bitmask: bit i=1 means position i uses G-code (even parity)
 * bit i=0 means L-code (odd parity)
 * Bit 5 = leftmost (position 2), Bit 0 = rightmost (position 7)
 */
static const uint8_t EAN13_PARITY[10] = {
    0x00,  /* 0: LLLLLL = 000000 */
    0x0B,  /* 1: LLGLGG = 001011 */
    0x0D,  /* 2: LLGGLG = 001101 */
    0x0E,  /* 3: LLGGGL = 001110 */
    0x13,  /* 4: LGLLGG = 010011 */
    0x19,  /* 5: LGGLLG = 011001 */
    0x1C,  /* 6: LGGGLL = 011100 */
    0x15,  /* 7: LGLGLG = 010101 */
    0x16,  /* 8: LGLGGL = 010110 */
    0x1A,  /* 9: LGGLGL = 011010 */
};

/* From decompiled: byte_1A2E5B = {7, 11, 13, 14, 19, 25, 28, 21, 22, 26}
 * These are the parity patterns as seen from CharUPC's bit accumulation
 * (accumulated MSB-first as: parity_bit | 2*prev)
 * Our EAN13_PARITY above is the normalized version. */

/* ── Edge Detection ───────────────────────────────────────────────── */

#define MAX_EDGES 2048
#define MAX_SCANLINES 40

typedef struct {
    int positions[MAX_EDGES];
    int count;
} EdgeList;

/*
 * Simple gradient-based edge detection on a scanline.
 * Finds transitions between bars and spaces.
 */
static int detect_edges(const uint8_t *line, int width, EdgeList *edges)
{
    edges->count = 0;
    if (width < 10) return 0;

    /*
     * Hybrid edge detection:
     * 1) For clean/binary images: detect level crossings at midpoint threshold
     * 2) For noisy/analog images: gradient zero-crossing (original method)
     *
     * Determine image type by checking pixel value distribution
     */

    /* Find min/max in the line */
    int vmin = 255, vmax = 0;
    for (int x = 0; x < width; x++) {
        if (line[x] < vmin) vmin = line[x];
        if (line[x] > vmax) vmax = line[x];
    }

    int range = vmax - vmin;
    if (range < 20) return 0; /* no barcode here */

    int mid_thresh = (vmin + vmax) / 2;

    /* Method 1: Simple threshold crossing — works great for clean images */
    int prev_state = (line[0] >= mid_thresh) ? 1 : 0; /* 1=space, 0=bar */
    for (int x = 1; x < width; x++) {
        int state = (line[x] >= mid_thresh) ? 1 : 0;
        if (state != prev_state) {
            if (edges->count < MAX_EDGES) {
                edges->positions[edges->count++] = x;
            }
            prev_state = state;
        }
    }

    return edges->count;
}

/* ── TMod — Bar Width Quantization (restored from line 230360) ──── */

/*
 * TMod(ctx, edge_idx, total_width, modules):
 *   Measures width of 2 elements (bar+space or space+bar)
 *   from edges[edge_idx] to edges[edge_idx+2],
 *   quantized to module units.
 *
 * result = (edges[i+2] - edges[i]) * modules + total_width/2) / total_width
 */
static int tmod(const EdgeList *edges, int idx, int total_width, int modules)
{
    if (total_width <= 0) return 0;
    if (idx < 0 || idx + 2 >= edges->count) return 0;
    int span = edges->positions[idx + 2] - edges->positions[idx];
    return (span * modules + total_width / 2) / total_width;
}

/* ── Element Width Measurement ────────────────────────────────────── */

/* Measure width of single element (bar or space) between two adjacent edges */
static int element_width(const EdgeList *edges, int idx)
{
    if (idx < 0 || idx + 1 >= edges->count) return 0;
    return edges->positions[idx + 1] - edges->positions[idx];
}

/* ── Guard Pattern Detection ──────────────────────────────────────── */

/* Check if 3 elements starting at edge idx match 1:1:1 pattern (start/end guard) */
static int is_guard_111(const EdgeList *edges, int idx)
{
    if (idx + 3 >= edges->count) return 0;
    int w0 = element_width(edges, idx);
    int w1 = element_width(edges, idx + 1);
    int w2 = element_width(edges, idx + 2);
    if (w0 <= 0 || w1 <= 0 || w2 <= 0) return 0;

    int avg = (w0 + w1 + w2) / 3;
    if (avg <= 0) return 0;

    /* Each element should be within ~33% of average (tightened to reject cross-symbology) */
    int tol = avg / 3 + 1;
    return (abs(w0 - avg) <= tol &&
            abs(w1 - avg) <= tol &&
            abs(w2 - avg) <= tol);
}

/* Check if 5 elements match 1:1:1:1:1 (middle guard) */
static int is_guard_11111(const EdgeList *edges, int idx)
{
    if (idx + 5 >= edges->count) return 0;
    int sum = 0;
    int w[5];
    for (int i = 0; i < 5; i++) {
        w[i] = element_width(edges, idx + i);
        if (w[i] <= 0) return 0;
        sum += w[i];
    }
    int avg = sum / 5;
    if (avg <= 0) return 0;
    int tol = avg / 3 + 1;
    for (int i = 0; i < 5; i++) {
        if (abs(w[i] - avg) > tol) return 0;
    }
    return 1;
}

/* ── Character Decoding ───────────────────────────────────────────── */

/*
 * Decode one UPC/EAN character (4 elements = 2 bars + 2 spaces, 7 modules total).
 * Returns: digit (0-9) or -1 on failure.
 * Sets *parity to 0 for L-code (odd), 1 for G-code (even).
 *
 * Restored from CharUPC() logic: measure each element width,
 * find best match against L/G/R patterns.
 */
static int decode_char(const EdgeList *edges, int start_edge, int *parity, int is_right)
{
    if (start_edge + 4 >= edges->count) return -1;

    /* Measure 4 element widths */
    int w[4];
    int total = 0;
    for (int i = 0; i < 4; i++) {
        w[i] = element_width(edges, start_edge + i);
        if (w[i] <= 0) return -1;
        total += w[i];
    }
    if (total <= 0) return -1;

    /* Quantize each element to modules (out of 7) */
    int m[4];
    int msum = 0;
    for (int i = 0; i < 4; i++) {
        m[i] = (w[i] * 7 + total / 2) / total;
        if (m[i] < 1) m[i] = 1;
        msum += m[i];
    }

    /* Adjust to sum to exactly 7 */
    while (msum > 7) {
        int max_idx = 0;
        for (int i = 1; i < 4; i++)
            if (m[i] > m[max_idx]) max_idx = i;
        if (m[max_idx] <= 1) break;
        m[max_idx]--;
        msum--;
    }
    while (msum < 7) {
        int min_idx = 0;
        for (int i = 1; i < 4; i++)
            if (m[i] < m[min_idx]) min_idx = i;
        m[min_idx]++;
        msum++;
    }

    /* Match against patterns */
    int best_digit = -1;
    int best_error = 999;
    int best_parity = 0;

    for (int d = 0; d < 10; d++) {
        /* Try L-code (odd parity) — bar at positions 1,3 */
        int err_l = 0;
        for (int i = 0; i < 4; i++)
            err_l += abs(m[i] - L_PATTERNS[d][i]);

        /* Try R-code — inverted polarity */
        int err_r = 0;
        int r_pat[4] = {
            L_PATTERNS[d][3], L_PATTERNS[d][2],
            L_PATTERNS[d][1], L_PATTERNS[d][0]
        };
        for (int i = 0; i < 4; i++)
            err_r += abs(m[i] - r_pat[i]);

        if (!is_right) {
            /* Left side: L-code or G-code */
            if (err_l < best_error) {
                best_error = err_l;
                best_digit = d;
                best_parity = 0; /* L = odd */
            }
            /* G-code is reversed L-code */
            int err_g = 0;
            int g_pat[4] = {
                L_PATTERNS[d][3], L_PATTERNS[d][2],
                L_PATTERNS[d][1], L_PATTERNS[d][0]
            };
            for (int i = 0; i < 4; i++)
                err_g += abs(m[i] - g_pat[i]);
            if (err_g < best_error) {
                best_error = err_g;
                best_digit = d;
                best_parity = 1; /* G = even */
            }
        } else {
            /* Right side: R-code only (complement of L) */
            /* R-code has same widths as L but starts with space */
            if (err_l < best_error) {
                best_error = err_l;
                best_digit = d;
                best_parity = 0;
            }
        }
    }

    /* Reject if error too large (tightened: max 1 module total error) */
    if (best_error > 1) return -1;

    if (parity) *parity = best_parity;
    return best_digit;
}

/* ── EAN-13 Decode ────────────────────────────────────────────────── */

/*
 * Try to decode an EAN-13 barcode starting at edge position `start`.
 * EAN-13 structure: [start guard 3] [6 chars × 4 edges] [mid guard 5] [6 chars × 4 edges] [end guard 3]
 * Total edges needed: 3 + 24 + 5 + 24 + 3 = 59
 */
static int try_ean13(const EdgeList *edges, int start, char *out, int *out_len)
{
    /* Need at least 59 elements from start */
    if (start + 59 >= edges->count) return 0;

    /* Check start guard (1:1:1) */
    if (!is_guard_111(edges, start)) return 0;

    /* Check middle guard at position start + 3 + 24 = start + 27 */
    int mid = start + 3 + 24;
    if (!is_guard_11111(edges, mid)) return 0;

    /* Check end guard */
    int end_guard = mid + 5 + 24;
    if (!is_guard_111(edges, end_guard)) return 0;

    /* Decode left half (6 characters, starting after start guard) */
    int left_digits[6];
    int left_parity[6];
    int pos = start + 3;
    for (int i = 0; i < 6; i++) {
        int par = 0;
        int d = decode_char(edges, pos, &par, 0);
        if (d < 0) return 0;
        left_digits[i] = d;
        left_parity[i] = par;
        pos += 4;
    }

    /* Decode right half (6 characters, after middle guard) */
    int right_digits[6];
    pos = mid + 5;
    for (int i = 0; i < 6; i++) {
        int d = decode_char(edges, pos, NULL, 1);
        if (d < 0) return 0;
        right_digits[i] = d;
        pos += 4;
    }

    /* Determine first digit from left parity pattern */
    int parity_pattern = 0;
    for (int i = 0; i < 6; i++) {
        parity_pattern = (parity_pattern << 1) | left_parity[i];
    }

    int first_digit = -1;
    for (int d = 0; d < 10; d++) {
        if (EAN13_PARITY[d] == parity_pattern) {
            first_digit = d;
            break;
        }
    }
    if (first_digit < 0) return 0;

    /* Verify check digit */
    int digits[13];
    digits[0] = first_digit;
    for (int i = 0; i < 6; i++) digits[1 + i] = left_digits[i];
    for (int i = 0; i < 6; i++) digits[7 + i] = right_digits[i];

    int checksum = 0;
    for (int i = 0; i < 12; i++) {
        checksum += digits[i] * ((i & 1) ? 3 : 1);
    }
    int check_digit = (10 - (checksum % 10)) % 10;
    if (check_digit != digits[12]) return 0;

    /* Output: UPC-A (first_digit==0) as 12 digits, EAN-13 as 13 digits */
    if (first_digit == 0) {
        for (int i = 1; i < 13; i++) {
            out[i - 1] = '0' + digits[i];
        }
        out[12] = '\0';
        *out_len = 12;
    } else {
        for (int i = 0; i < 13; i++) {
            out[i] = '0' + digits[i];
        }
        out[13] = '\0';
        *out_len = 13;
    }
    return 1;
}

/* ── EAN-8 Decode ─────────────────────────────────────────────────── */

/*
 * EAN-8: [start guard 3] [4 chars × 4] [mid guard 5] [4 chars × 4] [end guard 3]
 * Total: 3 + 16 + 5 + 16 + 3 = 43 elements
 */
static int try_ean8(const EdgeList *edges, int start, char *out, int *out_len)
{
    if (start + 43 >= edges->count) return 0;

    if (!is_guard_111(edges, start)) return 0;

    int mid = start + 3 + 16;
    if (!is_guard_11111(edges, mid)) return 0;

    int end_guard = mid + 5 + 16;
    if (!is_guard_111(edges, end_guard)) return 0;

    /* Decode left 4 (all L-code) */
    int digits[8];
    int pos = start + 3;
    for (int i = 0; i < 4; i++) {
        int d = decode_char(edges, pos, NULL, 0);
        if (d < 0) return 0;
        digits[i] = d;
        pos += 4;
    }

    /* Decode right 4 (all R-code) */
    pos = mid + 5;
    for (int i = 0; i < 4; i++) {
        int d = decode_char(edges, pos, NULL, 1);
        if (d < 0) return 0;
        digits[4 + i] = d;
        pos += 4;
    }

    /* Verify check digit */
    int checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += digits[i] * ((i & 1) ? 1 : 3);
    }
    int check_digit = (10 - (checksum % 10)) % 10;
    if (check_digit != digits[7]) return 0;

    for (int i = 0; i < 8; i++) {
        out[i] = '0' + digits[i];
    }
    out[8] = '\0';
    *out_len = 8;
    return 1;
}

/* ── UPC-E Decode ─────────────────────────────────────────────────── */

/*
 * UPC-E parity patterns (Number System 0).
 * Index = check digit of the expanded UPC-A.
 * Value = 6-bit bitmask: bit i=1 means position i uses G-code (even parity).
 * Bit 5 = leftmost (position 1), Bit 0 = rightmost (position 6).
 */
/* UPC-E parity patterns per GS1 specification (NS=0).
 * Index = check digit. Value = 6-bit bitmask (MSB=pos1).
 * B=even(G)=1, A=odd(L)=0.
 *   0: BBBAAA  1: BBABAA  2: BBAABA  3: BBAAAB
 *   4: BABBAA  5: BAABBA  6: BAAABB  7: BABABA
 *   8: BABAAB  9: BAABAB
 */
static const uint8_t UPCE_PARITY_NS0[10] = {
    0x38,  /* 0: BBBAAA = 111000 */
    0x34,  /* 1: BBABAA = 110100 */
    0x32,  /* 2: BBAABA = 110010 */
    0x31,  /* 3: BBAAAB = 110001 */
    0x2C,  /* 4: BABBAA = 101100 */
    0x26,  /* 5: BAABBA = 100110 */
    0x23,  /* 6: BAAABB = 100011 */
    0x2A,  /* 7: BABABA = 101010 */
    0x29,  /* 8: BABAAB = 101001 */
    0x25,  /* 9: BAABAB = 100101 */
};

/* For NS=1, parity is the complement (swap O/E) */

/* Check if 6 elements starting at idx match 1:1:1:1:1:1 (UPC-E end guard) */
static int is_guard_111111(const EdgeList *edges, int idx)
{
    if (idx + 6 >= edges->count) return 0;
    int sum = 0;
    int w[6];
    for (int i = 0; i < 6; i++) {
        w[i] = element_width(edges, idx + i);
        if (w[i] <= 0) return 0;
        sum += w[i];
    }
    int avg = sum / 6;
    if (avg <= 0) return 0;
    int tol = avg / 3 + 1;
    for (int i = 0; i < 6; i++) {
        if (abs(w[i] - avg) > tol) return 0;
    }
    return 1;
}

/*
 * Expand UPC-E (6 middle digits + number system + check) to 12-digit UPC-A.
 * Input: ns = number system (0 or 1), d[6] = the 6 encoded digits
 * Output: 12-char string (UPC-A without the check digit appended yet — we compute it)
 */
static int upce_expand(int ns, const int d[6], char *out12)
{
    /* Manufacturer + product depends on d[5] (the last encoded digit) */
    int m[5], p[5]; /* manufacturer (5 digits), product (5 digits) */
    memset(m, 0, sizeof(m));
    memset(p, 0, sizeof(p));

    switch (d[5]) {
    case 0: case 1: case 2:
        /* Manuf: d0 d1 d5 0 0, Prod: 0 0 d2 d3 d4 */
        m[0]=d[0]; m[1]=d[1]; m[2]=d[5]; m[3]=0; m[4]=0;
        p[0]=0;    p[1]=0;    p[2]=d[2]; p[3]=d[3]; p[4]=d[4];
        break;
    case 3:
        /* Manuf: d0 d1 d2 0 0, Prod: 0 0 0 d3 d4 */
        m[0]=d[0]; m[1]=d[1]; m[2]=d[2]; m[3]=0; m[4]=0;
        p[0]=0;    p[1]=0;    p[2]=0;    p[3]=d[3]; p[4]=d[4];
        break;
    case 4:
        /* Manuf: d0 d1 d2 d3 0, Prod: 0 0 0 0 d4 */
        m[0]=d[0]; m[1]=d[1]; m[2]=d[2]; m[3]=d[3]; m[4]=0;
        p[0]=0;    p[1]=0;    p[2]=0;    p[3]=0;    p[4]=d[4];
        break;
    default: /* 5-9 */
        /* Manuf: d0 d1 d2 d3 d4, Prod: 0 0 0 0 d5 */
        m[0]=d[0]; m[1]=d[1]; m[2]=d[2]; m[3]=d[3]; m[4]=d[4];
        p[0]=0;    p[1]=0;    p[2]=0;    p[3]=0;    p[4]=d[5];
        break;
    }

    /* UPC-A: NS M0 M1 M2 M3 M4 P0 P1 P2 P3 P4 CHECK */
    int a[12];
    a[0] = ns;
    for (int i = 0; i < 5; i++) a[1+i] = m[i];
    for (int i = 0; i < 5; i++) a[6+i] = p[i];

    /* Compute UPC-A check digit: weights 3,1,3,1,... for positions 0-10 */
    int checksum = 0;
    for (int i = 0; i < 11; i++) {
        checksum += a[i] * ((i & 1) ? 1 : 3);
    }
    a[11] = (10 - (checksum % 10)) % 10;

    for (int i = 0; i < 12; i++) {
        out12[i] = '0' + a[i];
    }
    out12[12] = '\0';
    return a[11]; /* return check digit */
}

/*
 * Try to decode a UPC-E barcode starting at edge position `start`.
 * UPC-E structure: [start guard 3] [6 chars × 4 edges] [end guard 6]
 * Total edges needed: 3 + 24 + 6 = 33
 * Output: expanded 12-digit UPC-A.
 */
static int try_upce(const EdgeList *edges, int start, char *out, int *out_len)
{
    if (start + 33 >= edges->count) return 0;

    /* Check start guard (1:1:1) */
    if (!is_guard_111(edges, start)) return 0;

    /* Check end guard (1:1:1:1:1:1) at start + 3 + 24 = start + 27 */
    int end_guard = start + 3 + 24;
    if (!is_guard_111111(edges, end_guard)) return 0;

    /* Decode 6 characters */
    int digits[6];
    int parity[6];
    int pos = start + 3;
    for (int i = 0; i < 6; i++) {
        int par = 0;
        int d = decode_char(edges, pos, &par, 0);
        if (d < 0) return 0;
        digits[i] = d;
        parity[i] = par;
        pos += 4;
    }

    /* Build parity pattern */
    int parity_pattern = 0;
    for (int i = 0; i < 6; i++) {
        parity_pattern = (parity_pattern << 1) | parity[i];
    }

    /* Match against NS=0 parity table to find check digit */
    int check_digit = -1;
    for (int c = 0; c < 10; c++) {
        if (UPCE_PARITY_NS0[c] == parity_pattern) {
            check_digit = c;
            break;
        }
    }

    if (check_digit < 0) return 0;

    /* Expand UPC-E to full 12-digit UPC-A and verify check digit (NS=0 only) */
    char expanded[13];
    int computed_check = upce_expand(0, digits, expanded);
    if (computed_check != check_digit) return 0;

    /* Output UPC-E 7-digit form: 6 data digits + check digit (matching the
     * reference firmware's reported data, which omits the leading number
     * system digit). */
    for (int i = 0; i < 6; i++) out[i] = '0' + digits[i];
    out[6] = '0' + check_digit;
    out[7] = '\0';
    *out_len = 7;
    return 1;
}

/* ── Multi-Scanline Voting Decoder ────────────────────────────────── */

typedef struct {
    char data[BARCODE_MAX_LEN];
    int  length;
    int  votes;
} Candidate;

#define MAX_CANDIDATES 16

static int add_candidate(Candidate *cands, int *n_cands, const char *data, int len)
{
    for (int i = 0; i < *n_cands; i++) {
        if (cands[i].length == len && memcmp(cands[i].data, data, len) == 0) {
            cands[i].votes++;
            return i;
        }
    }
    if (*n_cands < MAX_CANDIDATES) {
        int idx = *n_cands;
        memcpy(cands[idx].data, data, len);
        cands[idx].data[len] = '\0';
        cands[idx].length = len;
        cands[idx].votes = 1;
        (*n_cands)++;
        return idx;
    }
    return -1;
}

static int finalize_upc_ean_candidate(Candidate *cands, int *n_cands,
                                      const char *decoded, int decoded_len)
{
    if (!decoded || decoded_len <= 0) {
        return -1;
    }

    /*
     * Tail-processing scaffold:
     * keep the current direct-to-vote behavior unchanged for now,
     * but funnel all successful decodes through one helper so
     * supplemental/postprocess stages can be inserted here later.
     */
    return add_candidate(cands, n_cands, decoded, decoded_len);
}

/* ── Public API ───────────────────────────────────────────────────── */

int upc_ean_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !img->pixels || img->width < 60 || img->height < 1) return 0;

    Candidate candidates[MAX_CANDIDATES];
    int n_cands = 0;

    /* Scan multiple horizontal lines */
    int n_lines = img->height < MAX_SCANLINES ? img->height : MAX_SCANLINES;
    int step = img->height / (n_lines + 1);
    if (step < 1) step = 1;

    EdgeList edges;

    for (int li = 0; li < n_lines; li++) {
        int y = step * (li + 1);
        if (y >= img->height) y = img->height - 1;

        const uint8_t *row = img->pixels + y * img->width;
        detect_edges(row, img->width, &edges);


        if (edges.count < 33) continue; /* minimum for UPC-E (33 elements) */

        /* Try to find guard patterns and decode */
        for (int e = 0; e < edges.count - 33; e++) {
            if (!is_guard_111(&edges, e)) continue;

            char decoded[BARCODE_MAX_LEN];
            int decoded_len = 0;

            /* Try EAN-13 and UPC-A (both need 59 elements) */
            if (e + 59 < edges.count) {
                /* Try EAN-13 first (13 digits) — preserves leading zero */
                if (try_ean13(&edges, e, decoded, &decoded_len)) {
                    finalize_upc_ean_candidate(candidates, &n_cands,
                                               decoded, decoded_len);
                    continue;
                }
            }

            /* Try EAN-8 */
            if (try_ean8(&edges, e, decoded, &decoded_len)) {
                finalize_upc_ean_candidate(candidates, &n_cands,
                                           decoded, decoded_len);
                continue;
            }

            /* Try UPC-E (needs 33 elements) */
            if (e + 33 < edges.count) {
                if (try_upce(&edges, e, decoded, &decoded_len)) {
                    finalize_upc_ean_candidate(candidates, &n_cands,
                                               decoded, decoded_len);
                }
            }
        }
    }


    if (n_cands == 0) return 0;

    /* Find candidate with most votes */
    int best = 0;
    for (int i = 1; i < n_cands; i++) {
        if (candidates[i].votes > candidates[best].votes) best = i;
    }

    /* Require at least 5 votes for confidence (tightened to reject cross-symbology false positives) */
    if (candidates[best].votes < 5) return 0;

    memcpy(result->data, candidates[best].data, candidates[best].length + 1);
    result->length = candidates[best].length;
    result->confidence = (candidates[best].votes * 100) / n_lines;
    if (result->confidence > 100) result->confidence = 100;

    return 1;
}
