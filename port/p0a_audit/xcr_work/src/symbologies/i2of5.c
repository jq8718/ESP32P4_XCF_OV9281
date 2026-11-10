#include "i2of5.h"
#include "signal.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_RUNS 4096
#define MAX_SCANLINES 48
#define MAX_CANDIDATES 64

/*
 * I2of5 character encoding: 5 elements per digit, 2 wide + 3 narrow.
 * Index = digit value (0-9).
 * Bit pattern: bit4..bit0 = element0..element4, 1=wide, 0=narrow.
 *   0=NNWWN  1=WNNNW  2=NWNNW  3=WWNNN  4=NNWNW
 *   5=WNWNN  6=NWWNN  7=NNNWW  8=WNNWN  9=NWNWN
 */
static const uint8_t I2OF5_PATTERNS[10] = {
    0x06, /* 0: 00110 = NNWWN */
    0x11, /* 1: 10001 = WNNNW */
    0x09, /* 2: 01001 = NWNNW */
    0x18, /* 3: 11000 = WWNNN */
    0x05, /* 4: 00101 = NNWNW */
    0x14, /* 5: 10100 = WNWNN */
    0x0C, /* 6: 01100 = NWWNN */
    0x03, /* 7: 00011 = NNNWW */
    0x12, /* 8: 10010 = WNNWN */
    0x0A, /* 9: 01010 = NWNWN */
};

typedef struct {
    int len;
    int w[MAX_RUNS];
    uint8_t c[MAX_RUNS]; /* 1=bar (dark), 0=space (light) */
} Runs;

typedef struct {
    char text[BARCODE_MAX_LEN];
    int votes;
    int length;
} Candidate;

/* ── helpers ─────────────────────────────────────────────────────────── */

static int iabs(int v) { return v < 0 ? -v : v; }

static int otsu_threshold(const uint8_t *row, int width)
{
    int hist[256] = {0};
    for (int x = 0; x < width; x++) hist[row[x]]++;

    long long sum = 0;
    for (int i = 0; i < 256; i++) sum += (long long)i * hist[i];

    long long sum_b = 0;
    int w_b = 0;
    double max_var = -1.0;
    int threshold = 127;

    for (int t = 0; t < 256; t++) {
        w_b += hist[t];
        if (w_b == 0) continue;
        int w_f = width - w_b;
        if (w_f == 0) break;
        sum_b += (long long)t * hist[t];
        double m_b = (double)sum_b / w_b;
        double m_f = (double)(sum - sum_b) / w_f;
        double d = m_b - m_f;
        double var = (double)w_b * (double)w_f * d * d;
        if (var > max_var) {
            max_var = var;
            threshold = t;
        }
    }
    /* Clamp: binary images (only 0 and 255) can yield threshold=0,
       making (pixel < 0) always false for uint8_t. */
    if (threshold < 1) threshold = 1;
    if (threshold > 254) threshold = 254;
    return threshold;
}

/* ── run-length extraction ───────────────────────────────────────────── */

static int compress_runs(Runs *runs)
{
    if (!runs || runs->len <= 0) return 0;

    Runs tmp;
    tmp.len = 0;
    for (int i = 0; i < runs->len; i++) {
        if (runs->w[i] <= 0) continue;
        if (tmp.len > 0 && tmp.c[tmp.len - 1] == runs->c[i]) {
            tmp.w[tmp.len - 1] += runs->w[i];
        } else if (tmp.len < MAX_RUNS) {
            tmp.w[tmp.len] = runs->w[i];
            tmp.c[tmp.len] = runs->c[i];
            tmp.len++;
        }
    }

    /* Remove one-pixel glitches between same-color neighbors. */
    for (int i = 1; i + 1 < tmp.len; i++) {
        if (tmp.w[i] <= 1 && tmp.c[i - 1] == tmp.c[i + 1]) {
            tmp.w[i - 1] += tmp.w[i] + tmp.w[i + 1];
            for (int k = i; k + 2 < tmp.len; k++) {
                tmp.w[k] = tmp.w[k + 2];
                tmp.c[k] = tmp.c[k + 2];
            }
            tmp.len -= 2;
            if (--i < 1) i = 1;
        }
    }

    *runs = tmp;
    return runs->len;
}

static int extract_runs(const uint8_t *row, int width, Runs *runs)
{
    runs->len = 0;
    if (width < 16) return 0;

    int thr = otsu_threshold(row, width);
    uint8_t cur = (row[0] < thr) ? 1 : 0;
    int count = 1;

    for (int x = 1; x < width; x++) {
        uint8_t v = (row[x] < thr) ? 1 : 0;
        if (v == cur) {
            count++;
        } else {
            if (runs->len >= MAX_RUNS) return 0;
            runs->w[runs->len] = count;
            runs->c[runs->len] = cur;
            runs->len++;
            cur = v;
            count = 1;
        }
    }
    if (runs->len >= MAX_RUNS) return 0;
    runs->w[runs->len] = count;
    runs->c[runs->len] = cur;
    runs->len++;

    return compress_runs(runs);
}

static int extract_runs_from_edges(const uint8_t *row, int width,
                                   const int *edges, int n_edges,
                                   int thr, Runs *runs)
{
    runs->len = 0;
    if (!edges || n_edges < 4) return 0;

    int prev = 0;
    for (int i = 0; i <= n_edges; i++) {
        int cur = (i == n_edges) ? width : edges[i];
        if (cur < 0) cur = 0;
        if (cur > width) cur = width;
        if (cur <= prev) continue;

        int seg_w = cur - prev;
        int mid = prev + (seg_w >> 1);
        if (mid < 0) mid = 0;
        if (mid >= width) mid = width - 1;
        uint8_t color = (row[mid] < thr) ? 1 : 0;

        if (runs->len > 0 && runs->c[runs->len - 1] == color) {
            runs->w[runs->len - 1] += seg_w;
        } else {
            if (runs->len >= MAX_RUNS) break;
            runs->w[runs->len] = seg_w;
            runs->c[runs->len] = color;
            runs->len++;
        }
        prev = cur;
    }
    return compress_runs(runs);
}

/* ── I2of5 start/stop detection ──────────────────────────────────────── */

/*
 * Classify a run as narrow or wide given the local narrow unit.
 * Returns 0=narrow, 1=wide, -1=ambiguous.
 */
static int classify_nw(int w, int narrow_unit)
{
    if (narrow_unit <= 0) return -1;
    /* Threshold at 1.75x narrow: below = narrow, above = wide */
    int thr = narrow_unit * 7 / 4;
    if (w <= thr) return 0;
    /* Must not be larger than ~4x narrow (sanity) */
    if (w > narrow_unit * 4) return -1;
    return 1;
}

/*
 * Check for I2of5 start pattern at position s in runs.
 * Start pattern: narrow-bar, narrow-space, narrow-bar, narrow-space
 * The first run at s must be a bar (c==1).
 * Returns the estimated narrow unit width, or 0 if not a start.
 */
static int check_start(const Runs *runs, int s)
{
    if (s + 3 >= runs->len) return 0;
    /* Must start with bar */
    if (runs->c[s] != 1) return 0;
    /* Alternating bar-space-bar-space */
    if (runs->c[s + 1] != 0 || runs->c[s + 2] != 1 || runs->c[s + 3] != 0) return 0;

    int w0 = runs->w[s];
    int w1 = runs->w[s + 1];
    int w2 = runs->w[s + 2];
    int w3 = runs->w[s + 3];

    /* All four should be approximately equal (all narrow) */
    int avg = (w0 + w1 + w2 + w3) / 4;
    if (avg <= 0) return 0;

    for (int i = 0; i < 4; i++) {
        int w = runs->w[s + i];
        /* Each element must be within ~60% of average */
        if (iabs(w - avg) * 100 > avg * 60) return 0;
    }

    /* The quiet zone before start should be large */
    if (s > 0 && runs->c[s - 1] == 0) {
        if (runs->w[s - 1] < avg) return 0;
    }

    return avg;
}

/*
 * Check for I2of5 stop pattern at position s in runs.
 * Stop pattern: wide-bar, narrow-space, narrow-bar
 * Returns 1 if valid stop, 0 otherwise.
 */
static int check_stop(const Runs *runs, int s, int narrow_unit)
{
    if (s + 2 >= runs->len) return 0;
    if (runs->c[s] != 1 || runs->c[s + 1] != 0 || runs->c[s + 2] != 1) return 0;

    int wb = runs->w[s];
    int ns = runs->w[s + 1];
    int nb = runs->w[s + 2];

    /* Wide bar: should be 2-4x narrow */
    if (wb < narrow_unit * 3 / 2 || wb > narrow_unit * 4) return 0;
    /* Narrow space and narrow bar */
    if (classify_nw(ns, narrow_unit) != 0) return 0;
    if (classify_nw(nb, narrow_unit) != 0) return 0;

    /* Quiet zone after stop */
    if (s + 3 < runs->len && runs->c[s + 3] == 0) {
        if (runs->w[s + 3] < narrow_unit) return 0;
    }

    return 1;
}

/* ── I2of5 character pair decoding ───────────────────────────────────── */

/*
 * Decode a single digit from 5 element widths.
 * widths[0..4] are the 5 elements.
 * narrow_unit is the estimated narrow element width.
 * Returns digit 0-9, or -1 on failure.
 */
static int decode_digit(const int *widths, int narrow_unit)
{
    /* Classify each element as narrow(0) or wide(1) */
    int pattern = 0;
    int n_wide = 0;
    for (int i = 0; i < 5; i++) {
        int nw = classify_nw(widths[i], narrow_unit);
        if (nw < 0) return -1;
        pattern = (pattern << 1) | nw;
        if (nw) n_wide++;
    }

    /* I2of5 requires exactly 2 wide elements per digit */
    if (n_wide != 2) return -1;

    for (int d = 0; d < 10; d++) {
        if (I2OF5_PATTERNS[d] == pattern) return d;
    }
    return -1;
}

/*
 * Decode a pair of interleaved digits starting at position p in runs.
 * Bars encode digit1, spaces encode digit2.
 * Each pair spans 10 runs (5 bars + 5 spaces interleaved).
 * Returns 1 on success, fills d1 and d2.
 */
static int decode_pair(const Runs *runs, int p, int narrow_unit,
                       int *d1, int *d2)
{
    if (p + 9 >= runs->len) return 0;

    int bar_widths[5];
    int space_widths[5];

    for (int i = 0; i < 5; i++) {
        int bi = p + i * 2;      /* bars at even offsets */
        int si = p + i * 2 + 1;  /* spaces at odd offsets */
        if (runs->c[bi] != 1 || runs->c[si] != 0) return 0;
        bar_widths[i] = runs->w[bi];
        space_widths[i] = runs->w[si];
    }

    *d1 = decode_digit(bar_widths, narrow_unit);
    *d2 = decode_digit(space_widths, narrow_unit);

    if (*d1 < 0 || *d2 < 0) return 0;
    return 1;
}

/* ── I2of5 full scanline decode ──────────────────────────────────────── */

/*
 * Try to decode I2of5 from a run-length sequence.
 * Structure: start(4) + N_pairs*10 + stop(3).
 * The stop pattern (W-N-N) can appear within data, so we try all
 * possible pair counts and verify the stop at the exact expected position.
 */
static int decode_runs(const Runs *runs, char *out, int out_cap)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN];

    for (int s = 0; s + 4 < runs->len; s++) {
        int narrow_unit = check_start(runs, s);
        if (narrow_unit <= 0) continue;

        int data_start = s + 4;
        /* Try each possible number of pairs (1..max) */
        int max_pairs = (runs->len - data_start - 3) / 10;
        if (max_pairs > (BARCODE_MAX_LEN - 1) / 2) max_pairs = (BARCODE_MAX_LEN - 1) / 2;

        for (int np = max_pairs; np >= 2; np--) {
            int stop_pos = data_start + np * 10;
            if (!check_stop(runs, stop_pos, narrow_unit)) continue;

            /* Decode all pairs */
            char tmp[BARCODE_MAX_LEN];
            int n = 0;
            int ok = 1;
            int nu = narrow_unit;

            for (int pair = 0; pair < np; pair++) {
                int p = data_start + pair * 10;
                int d1, d2;
                if (!decode_pair(runs, p, nu, &d1, &d2)) { ok = 0; break; }
                tmp[n++] = '0' + d1;
                tmp[n++] = '0' + d2;

                /* Update narrow unit from decoded pair */
                int sum_n = 0, cnt_n = 0;
                for (int i = 0; i < 10; i++) {
                    int nw = classify_nw(runs->w[p + i], nu);
                    if (nw == 0) { sum_n += runs->w[p + i]; cnt_n++; }
                }
                if (cnt_n >= 3) nu = sum_n / cnt_n;
            }

            if (ok && n > best_len) {
                memcpy(best, tmp, (size_t)n);
                best[n] = '\0';
                best_len = n;
                break; /* longest first, so first success is best */
            }
        }
    }

    if (best_len < 4) return 0;
    if (best_len >= out_cap) best_len = out_cap - 1;
    memcpy(out, best, (size_t)best_len);
    out[best_len] = '\0';
    return best_len;
}

/* ── bidirectional decode ────────────────────────────────────────────── */

static void reverse_runs(const Runs *runs, Runs *rev)
{
    rev->len = runs->len;
    for (int i = 0; i < runs->len; i++) {
        rev->w[i] = runs->w[runs->len - 1 - i];
        rev->c[i] = runs->c[runs->len - 1 - i];
    }
}

static int decode_runs_bidirectional(const Runs *runs, char *out, int out_cap)
{
    char fwd[BARCODE_MAX_LEN];
    int len_fwd = decode_runs(runs, fwd, BARCODE_MAX_LEN);

    Runs rev;
    reverse_runs(runs, &rev);
    char rev_txt[BARCODE_MAX_LEN];
    int len_rev = decode_runs(&rev, rev_txt, BARCODE_MAX_LEN);

    /* If reverse produced result, flip the digits back */
    if (len_rev > 0) {
        char tmp[BARCODE_MAX_LEN];
        for (int i = 0; i < len_rev; i++) tmp[i] = rev_txt[len_rev - 1 - i];
        tmp[len_rev] = '\0';
        memcpy(rev_txt, tmp, (size_t)len_rev + 1);
    }

    /* Prefer the longer result, or forward on tie */
    int use_fwd = 1;
    if (len_fwd <= 0 && len_rev > 0) use_fwd = 0;
    if (len_fwd > 0 && len_rev > len_fwd) use_fwd = 0;

    if (use_fwd && len_fwd > 0) {
        if (len_fwd >= out_cap) len_fwd = out_cap - 1;
        memcpy(out, fwd, (size_t)len_fwd);
        out[len_fwd] = '\0';
        return len_fwd;
    }
    if (!use_fwd && len_rev > 0) {
        if (len_rev >= out_cap) len_rev = out_cap - 1;
        memcpy(out, rev_txt, (size_t)len_rev);
        out[len_rev] = '\0';
        return len_rev;
    }
    return 0;
}

/* ── scanline decoders ───────────────────────────────────────────────── */

static int decode_line_otsu(const uint8_t *row, int width, char *out, int out_cap)
{
    Runs runs;
    if (!extract_runs(row, width, &runs)) return 0;
    return decode_runs_bidirectional(&runs, out, out_cap);
}

static int decode_line_signal(const Image *img, int y, char *out, int out_cap)
{
    int edges[MAX_EDGES];
    int n_edges = signal_transitions(img->pixels, img->width, img->height,
                                     0, y, img->width - 1, y,
                                     edges, MAX_EDGES);
    if (n_edges < 8) return 0;

    const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
    int thr = otsu_threshold(row, img->width);

    Runs runs;
    if (!extract_runs_from_edges(row, img->width, edges, n_edges, thr, &runs)) return 0;

    return decode_runs_bidirectional(&runs, out, out_cap);
}

/* ── candidate voting ────────────────────────────────────────────────── */

static void add_candidate(Candidate cand[MAX_CANDIDATES], int *n_cand,
                          const char *decoded, int len)
{
    for (int k = 0; k < *n_cand; k++) {
        if (strcmp(cand[k].text, decoded) == 0) {
            cand[k].votes++;
            return;
        }
    }
    if (*n_cand < MAX_CANDIDATES) {
        strncpy(cand[*n_cand].text, decoded, BARCODE_MAX_LEN - 1);
        cand[*n_cand].text[BARCODE_MAX_LEN - 1] = '\0';
        cand[*n_cand].votes = 1;
        cand[*n_cand].length = len;
        (*n_cand)++;
    }
}

/* ── main entry point ────────────────────────────────────────────────── */

int i2of5_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !img->pixels || !result) return -1;
    if (img->width < 32 || img->height < 8) return -1;

    Candidate cand[MAX_CANDIDATES];
    int n_cand = 0;
    int decode_hits = 0;

    int scanlines = img->height < MAX_SCANLINES ? img->height : MAX_SCANLINES;
    for (int i = 0; i < scanlines; i++) {
        int y = (i + 1) * img->height / (scanlines + 1);
        const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;

        char decoded[BARCODE_MAX_LEN];
        int len = decode_line_otsu(row, img->width, decoded, BARCODE_MAX_LEN);
        if (len > 0) {
            decode_hits++;
            add_candidate(cand, &n_cand, decoded, len);
        }

        len = decode_line_signal(img, y, decoded, BARCODE_MAX_LEN);
        if (len > 0) {
            decode_hits++;
            add_candidate(cand, &n_cand, decoded, len);
        }
    }

    if (n_cand == 0) return -1;

    /* Pick candidate with most votes */
    int best = 0;
    for (int i = 1; i < n_cand; i++) {
        if (cand[i].votes > cand[best].votes ||
            (cand[i].votes == cand[best].votes && cand[i].length > cand[best].length)) {
            best = i;
        }
    }

    /* Require minimum length (4 digits) and at least 2 votes */
    if (cand[best].length < 4 || cand[best].votes < 2) return -1;

    memset(result, 0, sizeof(*result));
    {
        const char *text = cand[best].text;
        int len = cand[best].length;
        /* Interleaved 2 of 5 always encodes an even number of digits; an
         * odd-length payload is padded with a leading '0' to fill the last
         * pair. The reference firmware's reported data omits that pad
         * digit, so strip a single leading zero here. */
        if (len > 1 && text[0] == '0') {
            text++;
            len--;
        }
        strncpy(result->data, text, BARCODE_MAX_LEN - 1);
        result->data[BARCODE_MAX_LEN - 1] = '\0';
    }
    result->length = (int)strlen(result->data);

    /* Use decode_hits as denominator (not scanlines*2) to reflect
     * actual success rate, matching Codabar's confidence formula. */
    int denom = decode_hits > 0 ? decode_hits : 1;
    int conf = cand[best].votes * 100 / denom;
    if (conf < 10) conf = 10;
    if (conf > 100) conf = 100;
    result->confidence = conf;

    return 1;
}
