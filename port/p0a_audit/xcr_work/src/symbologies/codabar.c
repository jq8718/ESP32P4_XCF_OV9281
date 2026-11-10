#include "codabar.h"
#include "signal.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define MAX_RUNS 4096
#define MAX_SCANLINES 48
#define MAX_CANDIDATES 64

/* Restored table shape used by the legacy decoder (7-element narrow/wide mask). */
static const char CODABAR_ALPHABET[] = "0123456789-$:/.+ABCD";
static const int CODABAR_ENCODING[] = {
    0x003, 0x006, 0x009, 0x060, 0x012, 0x042, 0x021, 0x024, 0x030, 0x048,
    0x00C, 0x018, 0x045, 0x051, 0x054, 0x015, 0x01A, 0x029, 0x00B, 0x00E,
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

static int iabs_i(int v)
{
    return v < 0 ? -v : v;
}

static int is_start_stop(char ch)
{
    return ch == 'A' || ch == 'B' || ch == 'C' || ch == 'D';
}

static int codabar_char_value(char ch)
{
    for (int i = 0; i < 20; i++) {
        if (CODABAR_ALPHABET[i] == ch) return i;
    }
    return -1;
}

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

    /* For binary images (only 0 and 255), Otsu picks t=0 which makes
     * pixel < 0 always false.  Only apply midpoint fallback if the row
     * has exactly 2 distinct values (true binary), to avoid false
     * positives on rows with extreme but non-binary distributions. */
    if (threshold <= 0 || threshold >= 255) {
        int lo = 255, hi = 0;
        int n_distinct = 0;
        int seen[256] = {0};
        for (int x = 0; x < width; x++) {
            if (row[x] < lo) lo = row[x];
            if (row[x] > hi) hi = row[x];
            if (!seen[row[x]]) { seen[row[x]] = 1; n_distinct++; }
        }
        if (n_distinct <= 2) {
            threshold = (lo + hi + 1) / 2;
        }
    }

    return threshold;
}

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
            i -= 1;
            if (i < 1) i = 1;
        }
    }

    *runs = tmp;
    return runs->len;
}

static int extract_runs_threshold(const uint8_t *row, int width, int thr, Runs *runs)
{
    runs->len = 0;
    if (width < 16) return 0;

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

static int extract_runs(const uint8_t *row, int width, Runs *runs)
{
    int thr = otsu_threshold(row, width);
    return extract_runs_threshold(row, width, thr, runs);
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

static int decode_char_at(const Runs *runs, int start,
                          char *out_ch, int *out_total,
                          int *out_narrow_space, int *out_quality)
{
    if (start < 0 || start + 6 >= runs->len) return 0;

    int total = 0;
    for (int i = 0; i < 7; i++) {
        int idx = start + i;
        int w = runs->w[idx];
        if (w <= 0) return 0;
        if (((i & 1) == 0 && runs->c[idx] != 1) || ((i & 1) == 1 && runs->c[idx] != 0)) return 0;
        total += w;
    }

    int best_sym = -1;
    int best_err = 1 << 30;
    int best_sp_narrow = 0;

    for (int s = 0; s < 20; s++) {
        int e = CODABAR_ENCODING[s];
        int bn_sum = 0, bn_cnt = 0, bw_sum = 0, bw_cnt = 0;
        int sn_sum = 0, sn_cnt = 0, sw_sum = 0, sw_cnt = 0;

        for (int i = 0; i < 7; i++) {
            int idx = start + i;
            int wide = (e >> (6 - i)) & 1;
            int w = runs->w[idx];
            if ((i & 1) == 0) {
                if (wide) {
                    bw_sum += w;
                    bw_cnt++;
                } else {
                    bn_sum += w;
                    bn_cnt++;
                }
            } else {
                if (wide) {
                    sw_sum += w;
                    sw_cnt++;
                } else {
                    sn_sum += w;
                    sn_cnt++;
                }
            }
        }

        if (bn_cnt == 0 || sn_cnt == 0 || bw_cnt == 0 || sw_cnt == 0) continue;

        int bn = bn_sum / bn_cnt;
        int bw = bw_sum / bw_cnt;
        int sn = sn_sum / sn_cnt;
        int sw = sw_sum / sw_cnt;

        if (bw * 100 < bn * 145) continue;
        if (sw * 100 < sn * 145) continue;

        int err = 0;
        for (int i = 0; i < 7; i++) {
            int idx = start + i;
            int wide = (e >> (6 - i)) & 1;
            int target;
            if ((i & 1) == 0) {
                target = wide ? bw : bn;
            } else {
                target = wide ? sw : sn;
            }
            err += (iabs_i(runs->w[idx] - target) * 32) / (target + 1);
        }

        if (err < best_err) {
            best_err = err;
            best_sym = s;
            best_sp_narrow = sn;
        }
    }

    if (best_sym < 0 || best_err > 180) return 0;

    *out_ch = CODABAR_ALPHABET[best_sym];
    if (out_total) *out_total = total;
    if (out_narrow_space) *out_narrow_space = best_sp_narrow;
    if (out_quality) *out_quality = best_err;
    return 1;
}

static int codabar_checksum_valid(const char *text, int n)
{
    if (n < 5) return 0;

    int check_pos = n - 2;
    int check_val = codabar_char_value(text[check_pos]);
    if (check_val < 0) return 0;

    /* Optional mod-16 checksum variants used by Codabar/NW-7 devices. */
    int sum_data_only = 0;
    for (int i = 1; i < check_pos; i++) {
        int v = codabar_char_value(text[i]);
        if (v < 0) return 0;
        sum_data_only += v;
    }
    if (((sum_data_only + check_val) & 0x0F) == 0) return 1;

    int sum_with_guards = codabar_char_value(text[0]) + codabar_char_value(text[n - 1]);
    for (int i = 1; i < check_pos; i++) sum_with_guards += codabar_char_value(text[i]);
    if (((sum_with_guards + check_val) & 0x0F) == 0) return 1;

    return 0;
}

static int compute_message_score(const Runs *runs, int start_idx, int end_idx,
                                 const char *text, int n,
                                 int mean_char_quality, int narrow_space)
{
    int score = n * 100 - mean_char_quality;

    if (n >= 3 && codabar_checksum_valid(text, n)) score += 120;

    if (narrow_space > 0) {
        /* codabar_decompiled.c:591-640 / 775-813 / 965-1003
         * Gap and center gating are used before accepting symbol chains.
         * We mirror that by penalizing inconsistent inter-character gaps. */
        for (int p = start_idx + 7; p < end_idx; p += 8) {
            if (p >= 0 && p < runs->len && runs->c[p] == 0) {
                int g = runs->w[p];
                if (g > narrow_space * 4 || g * 3 < narrow_space) score -= 50;
            }
        }
    }

    int left = 0;
    int right = 0;
    if (start_idx > 0 && runs->c[start_idx - 1] == 0) left = runs->w[start_idx - 1];
    if (end_idx + 1 < runs->len && runs->c[end_idx + 1] == 0) right = runs->w[end_idx + 1];
    if (left > 0 && right > 0) {
        int d = iabs_i(left - right);
        if (d > (left + right) / 2) score -= 30;
    }

    return score;
}

static int decode_direction(const Runs *runs, char out[BARCODE_MAX_LEN], int *out_score)
{
    int best_len = 0;
    int best_score = -(1 << 30);
    char best[BARCODE_MAX_LEN];

    for (int s = 0; s + 7 < runs->len; s++) {
        if (runs->c[s] != 1) continue;

        char ch;
        int char_w = 0;
        int narrow_sp = 0;
        int ch_quality = 0;
        if (!decode_char_at(runs, s, &ch, &char_w, &narrow_sp, &ch_quality)) continue;
        if (!is_start_stop(ch)) continue;

        /* codabar_decompiled.c:304-343 / 486-523 / 677-714 / 858-895
         * Start/stop candidates are selected from constrained edge triplets. */
        if (s > 0 && runs->c[s - 1] == 0 && runs->w[s - 1] > 0 && narrow_sp > 0) {
            if (runs->w[s - 1] < narrow_sp / 2) continue;
        }

        char tmp[BARCODE_MAX_LEN];
        int n = 0;
        int sum_quality = ch_quality;
        tmp[n++] = ch;

        int p = s;
        int ok = 0;

        while (n < BARCODE_MAX_LEN - 1) {
            p += 7;
            if (p >= runs->len) break;

            if (runs->c[p] != 0) break;
            int gap = runs->w[p];

            if (narrow_sp > 0 && gap > narrow_sp * 4) break;
            if (narrow_sp > 0 && gap * 3 < narrow_sp) break;
            if (p + 7 >= runs->len) break;

            int next_char_w = 0;
            int next_narrow_sp = 0;
            int next_q = 0;
            if (!decode_char_at(runs, p + 1, &ch, &next_char_w, &next_narrow_sp, &next_q)) break;

            tmp[n++] = ch;
            sum_quality += next_q;
            p += 1;
            if (next_narrow_sp > 0) narrow_sp = next_narrow_sp;

            if (is_start_stop(ch)) {
                ok = (n >= 3);
                break;
            }
        }

        if (!ok) continue;

        int valid = 1;
        for (int i = 1; i + 1 < n; i++) {
            if (is_start_stop(tmp[i])) {
                valid = 0;
                break;
            }
        }
        if (!valid) continue;

        int mean_q = sum_quality / n;
        int msg_score = compute_message_score(runs, s, p, tmp, n, mean_q, narrow_sp);

        if (msg_score > best_score || (msg_score == best_score && n > best_len)) {
            memcpy(best, tmp, (size_t)n);
            best[n] = '\0';
            best_len = n;
            best_score = msg_score;
        }
    }

    if (best_len == 0) return 0;
    memcpy(out, best, (size_t)best_len + 1);
    if (out_score) *out_score = best_score;
    return best_len;
}

static void reverse_runs(const Runs *runs, Runs *rev)
{
    rev->len = runs->len;
    for (int i = 0; i < runs->len; i++) {
        rev->w[i] = runs->w[runs->len - 1 - i];
        rev->c[i] = runs->c[runs->len - 1 - i];
    }
}

static int decode_runs_bidirectional(const Runs *runs, char out[BARCODE_MAX_LEN])
{
    int score_fwd = 0;
    int score_rev = 0;
    char fwd[BARCODE_MAX_LEN];
    char rev_txt[BARCODE_MAX_LEN];

    int len_fwd = decode_direction(runs, fwd, &score_fwd);

    Runs rev;
    reverse_runs(runs, &rev);
    int len_rev = decode_direction(&rev, rev_txt, &score_rev);

    if (len_fwd <= 0 && len_rev <= 0) return 0;

    if (len_rev > 0 && (len_fwd <= 0 || score_rev > score_fwd)) {
        for (int i = 0; i < len_rev; i++) out[i] = rev_txt[len_rev - 1 - i];
        out[len_rev] = '\0';
        return len_rev;
    }

    memcpy(out, fwd, (size_t)len_fwd + 1);
    return len_fwd;
}

static int decode_line_otsu(const uint8_t *row, int width, char out[BARCODE_MAX_LEN])
{
    Runs runs;
    if (!extract_runs(row, width, &runs)) return 0;
    return decode_runs_bidirectional(&runs, out);
}

static int decode_line_signal(const Image *img, int y, char out[BARCODE_MAX_LEN])
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

    return decode_runs_bidirectional(&runs, out);
}

static void add_candidate(Candidate cand[MAX_CANDIDATES], int *n_cand,
                          const char *decoded, int len)
{
    int found = -1;
    for (int k = 0; k < *n_cand; k++) {
        if (strcmp(cand[k].text, decoded) == 0) {
            found = k;
            break;
        }
    }

    if (found >= 0) {
        cand[found].votes++;
    } else if (*n_cand < MAX_CANDIDATES) {
        strcpy(cand[*n_cand].text, decoded);
        cand[*n_cand].votes = 1;
        cand[*n_cand].length = len;
        (*n_cand)++;
    }
}

int codabar_decode(const Image *img, BarcodeResult *result)
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
        int len = decode_line_otsu(row, img->width, decoded);
        if (len > 0) {
            decode_hits++;
            add_candidate(cand, &n_cand, decoded, len);
        }

        /* Additional edge-domain path, aligned with code128 signal chain usage. */
        len = decode_line_signal(img, y, decoded);
        if (len > 0) {
            decode_hits++;
            add_candidate(cand, &n_cand, decoded, len);
        }
    }

    if (n_cand == 0) return -1;

    int best = 0;
    for (int i = 1; i < n_cand; i++) {
        if (cand[i].votes > cand[best].votes ||
            (cand[i].votes == cand[best].votes && cand[i].length > cand[best].length)) {
            best = i;
        }
    }

    /* Reject spurious short matches (start + stop only) that cause
       false positives on non-Codabar images. Accept with at least 1 data character. */
    if (cand[best].length < 3) return -1;

    /* Validate start/stop characters are present and properly positioned */
    int len = cand[best].length;
    if (len < 3) return -1;
    if (!is_start_stop(cand[best].text[0])) return -1;
    if (!is_start_stop(cand[best].text[len - 1])) return -1;

    /* Ensure no start/stop characters in middle */
    for (int i = 1; i < len - 1; i++) {
        if (is_start_stop(cand[best].text[i])) return -1;
    }

    /* VALIDATION: Strengthen Codabar detection to reduce false positives
     * Codabar valid length: typically 5-30 characters (start + data + stop)
     * Valid character set: 0-9, A-D (start/stop), -$:/.+ (special chars) */

    /* Check length bounds */
    if (len < 5 || len > 30) {
        return -1;  /* Reject abnormal lengths */
    }

    /* Validate character set for data portion (between start/stop) */
    for (int i = 1; i < len - 1; i++) {
        char c = cand[best].text[i];
        if (!(c >= '0' && c <= '9') &&  /* digits */
            c != '-' && c != '$' && c != ':' &&
            c != '/' && c != '.' && c != '+') {
            return -1;  /* Invalid character in Codabar */
        }
    }

    /* Require minimum confidence to avoid low-quality matches */
    if (cand[best].votes < 2) {
        return -1;  /* Single-vote matches too unreliable */
    }

    memset(result, 0, sizeof(*result));
    /* Codabar output keeps the start/stop (A-D) characters, matching the
     * reference firmware's reported data (e.g. "A123456A"). */
    strncpy(result->data, cand[best].text, BARCODE_MAX_LEN - 1);
    result->data[BARCODE_MAX_LEN - 1] = '\0';
    result->length = (int)strlen(result->data);

    /* Calculate confidence based on vote ratio
     * Use decode_hits as denominator to be more lenient for valid short barcodes */
    int denom = decode_hits > 0 ? decode_hits : 1;
    int conf = cand[best].votes * 100 / denom;
    if (conf < 10) conf = 10;  /* Ensure minimum confidence for valid decode */
    if (conf > 100) conf = 100;
    result->confidence = conf;

    return 1;
}
