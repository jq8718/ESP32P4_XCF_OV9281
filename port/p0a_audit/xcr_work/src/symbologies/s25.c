/*
 * Standard 2-of-5 decoder.
 */

#include "s25.h"

#include <stdint.h>
#include <string.h>

#define MAX_RUNS 4096
#define MAX_BARS 2048
#define BARCODE_MAX_LEN 256

static const uint8_t S25_PATTERNS[10] = {
    0x06, 0x11, 0x09, 0x18, 0x05, 0x14, 0x0C, 0x03, 0x12, 0x0A,
};

static int otsu_threshold(const uint8_t *row, int width)
{
    int hist[256] = {0};
    long long sum = 0;
    long long sum_b = 0;
    int w_b = 0;
    double max_var = -1.0;
    int threshold = 127;

    for (int x = 0; x < width; x++) hist[row[x]]++;
    for (int i = 0; i < 256; i++) sum += (long long)i * hist[i];
    for (int t = 0; t < 256; t++) {
        w_b += hist[t];
        if (w_b == 0) continue;
        {
            int w_f = width - w_b;
            if (w_f == 0) break;
            sum_b += (long long)t * hist[t];
            {
                double m_b = (double)sum_b / w_b;
                double m_f = (double)(sum - sum_b) / w_f;
                double d = m_b - m_f;
                double var = (double)w_b * (double)w_f * d * d;
                if (var > max_var) {
                    max_var = var;
                    threshold = t;
                }
            }
        }
    }
    if (threshold < 1) threshold = 1;
    if (threshold > 254) threshold = 254;
    return threshold;
}

typedef struct {
    int len;
    int w[MAX_RUNS];
    uint8_t c[MAX_RUNS];
} Runs;

static int extract_runs(const uint8_t *row, int width, Runs *runs)
{
    int thr;
    uint8_t cur;
    int count;

    runs->len = 0;
    if (width < 16) return 0;

    thr = otsu_threshold(row, width);
    cur = (row[0] < thr) ? 1 : 0;
    count = 1;
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

    for (int i = 1; i + 1 < runs->len; i++) {
        if (runs->w[i] <= 1 && runs->c[i - 1] == runs->c[i + 1]) {
            runs->w[i - 1] += runs->w[i] + runs->w[i + 1];
            for (int k = i; k + 2 < runs->len; k++) {
                runs->w[k] = runs->w[k + 2];
                runs->c[k] = runs->c[k + 2];
            }
            runs->len -= 2;
            if (--i < 1) i = 1;
        }
    }
    return runs->len;
}

static int extract_bars(const Runs *runs, int *bar_w, int max_bars)
{
    int nb = 0;
    for (int i = 0; i < runs->len && nb < max_bars; i++) {
        if (runs->c[i] == 1) bar_w[nb++] = runs->w[i];
    }
    return nb;
}

static int estimate_narrow(const int *bars, int n)
{
    int min_w = bars[0];
    if (n < 3) return 0;
    for (int i = 1; i < n; i++) {
        if (bars[i] < min_w) min_w = bars[i];
    }
    return min_w > 0 ? min_w : 1;
}

static int is_wide(int w, int narrow_unit)
{
    return w > narrow_unit * 7 / 4;
}

static int is_narrow(int w, int narrow_unit)
{
    return w <= narrow_unit * 7 / 4;
}

static int decode_digit(const int *widths, int narrow_unit)
{
    int pattern = 0;
    int n_wide = 0;
    for (int i = 0; i < 5; i++) {
        int wide = is_wide(widths[i], narrow_unit);
        pattern = (pattern << 1) | wide;
        if (wide) n_wide++;
    }
    if (n_wide != 2) return -1;
    for (int d = 0; d < 10; d++) {
        if (S25_PATTERNS[d] == pattern) return d;
    }
    return -1;
}

static int decode_industrial_bars(const int *bars, int nb, char *out, int out_cap)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN];
    int nu_global;

    if (nb < 11) return 0;
    nu_global = estimate_narrow(bars, nb);
    if (nu_global <= 0) return 0;

    {
        int max_start = nb / 4;
        if (max_start > 10) max_start = 10;
        for (int s = 0; s < max_start && s + 10 < nb; s++) {
            int nu = nu_global;
            int data_start;
            int remaining;
            int max_digits;

            if (!is_wide(bars[s], nu) || !is_wide(bars[s + 1], nu) || !is_narrow(bars[s + 2], nu)) {
                continue;
            }

            data_start = s + 3;
            remaining = nb - data_start - 3;
            if (remaining < 5) continue;
            max_digits = remaining / 5;
            if (max_digits > BARCODE_MAX_LEN - 1) max_digits = BARCODE_MAX_LEN - 1;

            for (int nd = max_digits; nd >= 1; nd--) {
                int stop_start = data_start + nd * 5;
                char tmp[BARCODE_MAX_LEN];
                int n = 0;
                int ok = 1;
                int local_nu = nu;

                if (stop_start + 2 >= nb) continue;
                if (!is_wide(bars[stop_start], nu) || !is_narrow(bars[stop_start + 1], nu) ||
                    !is_wide(bars[stop_start + 2], nu)) {
                    continue;
                }

                for (int di = 0; di < nd; di++) {
                    int p = data_start + di * 5;
                    int digit = decode_digit(&bars[p], local_nu);
                    if (digit < 0) {
                        ok = 0;
                        break;
                    }
                    tmp[n++] = (char)('0' + digit);
                    {
                        int sum_n = 0;
                        int cnt_n = 0;
                        for (int b = 0; b < 5; b++) {
                            if (is_narrow(bars[p + b], local_nu)) {
                                sum_n += bars[p + b];
                                cnt_n++;
                            }
                        }
                        if (cnt_n >= 2) local_nu = sum_n / cnt_n;
                    }
                }

                if (ok && n > best_len) {
                    memcpy(best, tmp, (size_t)n);
                    best[n] = '\0';
                    best_len = n;
                    break;
                }
            }
        }
    }

    if (best_len < 1 || best_len >= out_cap) return 0;
    memcpy(out, best, (size_t)best_len + 1);
    return best_len;
}

static int decode_iata_bars(const int *bars, int nb, char *out, int out_cap)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN];
    int nu_global;

    if (nb < 9) return 0;
    nu_global = estimate_narrow(bars, nb);
    if (nu_global <= 0) return 0;

    {
        int max_start = nb / 4;
        if (max_start > 10) max_start = 10;
        for (int s = 0; s < max_start && s + 8 < nb; s++) {
            int nu = nu_global;
            int data_start;
            int remaining;
            int max_digits;

            if (!is_narrow(bars[s], nu) || !is_narrow(bars[s + 1], nu)) continue;

            data_start = s + 2;
            remaining = nb - data_start - 2;
            if (remaining < 5) continue;
            max_digits = remaining / 5;
            if (max_digits > BARCODE_MAX_LEN - 1) max_digits = BARCODE_MAX_LEN - 1;

            for (int nd = max_digits; nd >= 1; nd--) {
                int stop_start = data_start + nd * 5;
                char tmp[BARCODE_MAX_LEN];
                int n = 0;
                int ok = 1;
                int local_nu = nu;

                if (stop_start + 1 >= nb) continue;
                if (!is_wide(bars[stop_start], nu) || !is_narrow(bars[stop_start + 1], nu)) continue;

                for (int di = 0; di < nd; di++) {
                    int p = data_start + di * 5;
                    int digit = decode_digit(&bars[p], local_nu);
                    if (digit < 0) {
                        ok = 0;
                        break;
                    }
                    tmp[n++] = (char)('0' + digit);
                    {
                        int sum_n = 0;
                        int cnt_n = 0;
                        for (int b = 0; b < 5; b++) {
                            if (is_narrow(bars[p + b], local_nu)) {
                                sum_n += bars[p + b];
                                cnt_n++;
                            }
                        }
                        if (cnt_n >= 2) local_nu = sum_n / cnt_n;
                    }
                }

                if (ok && n > best_len) {
                    memcpy(best, tmp, (size_t)n);
                    best[n] = '\0';
                    best_len = n;
                    break;
                }
            }
        }
    }

    if (best_len < 1 || best_len >= out_cap) return 0;
    memcpy(out, best, (size_t)best_len + 1);
    return best_len;
}

/*
 * AIM Matrix 2-of-5 start pattern is 6 elements: W,n,N,n,N,n (a wide bar
 * followed by 5 narrow elements alternating space/bar/space/bar/space).
 * `s` must index the leading wide bar.
 */
static int check_matrix_start(const Runs *runs, int s)
{
    int sum;
    int narrow_unit;

    if (s + 5 >= runs->len) return 0;
    if (runs->c[s] != 1) return 0;

    for (int i = 1; i <= 5; i++) {
        int expect_bar = (i % 2 == 0);
        if (expect_bar) {
            if (runs->c[s + i] != 1) return 0;
        } else {
            if (runs->c[s + i] != 0) return 0;
        }
    }

    sum = 0;
    for (int i = 1; i <= 5; i++) sum += runs->w[s + i];
    narrow_unit = sum / 5;
    if (narrow_unit <= 0) return 0;

    for (int i = 1; i <= 5; i++) {
        int diff = runs->w[s + i] - narrow_unit;
        if (diff < 0) diff = -diff;
        if (diff * 100 > narrow_unit * 70) return 0;
    }
    if (runs->w[s] < narrow_unit * 2) return 0;
    if (s > 0 && runs->c[s - 1] == 0 && runs->w[s - 1] < narrow_unit) return 0;
    return narrow_unit;
}

static int check_matrix_stop(const Runs *runs, int s, int narrow_unit)
{
    if (s + 2 >= runs->len) return 0;
    if (runs->c[s] != 1 || runs->c[s + 1] != 0 || runs->c[s + 2] != 1) return 0;
    if (!is_wide(runs->w[s], narrow_unit) || !is_narrow(runs->w[s + 1], narrow_unit) ||
        !is_narrow(runs->w[s + 2], narrow_unit)) {
        return 0;
    }
    if (s + 3 < runs->len && runs->c[s + 3] == 0 && runs->w[s + 3] < narrow_unit) return 0;
    return 1;
}

/*
 * Matrix 2-of-5 character = 5 consecutive elements (bar,space,bar,space,bar),
 * all 5 carrying data (unlike Industrial/IATA 2of5 which only encode in
 * bars). Characters are separated by a single non-data narrow space, so the
 * stride between successive character starts is 6 elements. The stop
 * pattern's leading "n" is exactly that trailing separator, so a flat
 * stride of 6 covers every character including the last one before stop.
 */
static int decode_matrix(const Runs *runs, char *out, int out_cap)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN];
    int max_start_m = runs->len;

    if (max_start_m > 40) max_start_m = 40;
    for (int s = 0; s < max_start_m && s + 5 < runs->len; s++) {
        int narrow_unit = check_matrix_start(runs, s);
        int data_start;
        int max_digits;

        if (narrow_unit <= 0) continue;
        data_start = s + 6;
        max_digits = (runs->len - data_start) / 6;
        if (max_digits > BARCODE_MAX_LEN - 1) max_digits = BARCODE_MAX_LEN - 1;
        if (max_digits < 1) continue;

        for (int nd = max_digits; nd >= 1; nd--) {
            int stop_pos = data_start + nd * 6;
            char tmp[BARCODE_MAX_LEN];
            int n = 0;
            int ok = 1;
            int nu = narrow_unit;

            if (stop_pos >= runs->len) continue;
            if (!check_matrix_stop(runs, stop_pos, narrow_unit)) continue;

            for (int di = 0; di < nd; di++) {
                int p = data_start + di * 6;
                int widths[5];
                int digit;

                if (p + 4 >= runs->len) { ok = 0; break; }
                if (runs->c[p] != 1 || runs->c[p + 1] != 0 || runs->c[p + 2] != 1 ||
                    runs->c[p + 3] != 0 || runs->c[p + 4] != 1) {
                    ok = 0;
                    break;
                }
                for (int b = 0; b < 5; b++) widths[b] = runs->w[p + b];

                digit = decode_digit(widths, nu);
                if (digit < 0) {
                    ok = 0;
                    break;
                }
                tmp[n++] = (char)('0' + digit);

                {
                    int sum_n = 0;
                    int cnt_n = 0;
                    for (int b = 0; b < 5; b++) {
                        if (is_narrow(widths[b], nu)) {
                            sum_n += widths[b];
                            cnt_n++;
                        }
                    }
                    if (cnt_n >= 2) nu = sum_n / cnt_n;
                }
            }

            if (ok && n > best_len) {
                memcpy(best, tmp, (size_t)n);
                best[n] = '\0';
                best_len = n;
                break;
            }
        }
    }

    /* Common encoder convention: left-pad an even-length numeric payload
     * with a leading '0' so the transmitted character count is odd. Strip
     * it back off to recover the logical value. */
    if (best_len > 1 && (best_len % 2) == 1 && best[0] == '0') {
        memmove(best, best + 1, (size_t)best_len);
        best_len--;
    }

    if (best_len < 1 || best_len >= out_cap) return 0;
    memcpy(out, best, (size_t)best_len + 1);
    return best_len;
}

/*
 * HK 2-of-5 (Hong Kong Matrix 2of5 variant) start pattern is 4 narrow
 * elements: N,n,N,n (bar,space,bar,space,all roughly equal, no leading wide
 * bar). Data characters and the W,n,N stop use the same encoding as AIM
 * Matrix 2of5 (see decode_matrix / check_matrix_stop above), just with a
 * shorter, all-narrow start guard.
 */
static int check_hk25_start(const Runs *runs, int s)
{
    int sum;
    int narrow_unit;

    if (s + 3 >= runs->len) return 0;
    if (runs->c[s] != 1 || runs->c[s + 1] != 0 || runs->c[s + 2] != 1 || runs->c[s + 3] != 0) {
        return 0;
    }

    sum = runs->w[s] + runs->w[s + 1] + runs->w[s + 2] + runs->w[s + 3];
    narrow_unit = sum / 4;
    if (narrow_unit <= 0) return 0;

    for (int i = 0; i < 4; i++) {
        int diff = runs->w[s + i] - narrow_unit;
        if (diff < 0) diff = -diff;
        if (diff * 100 > narrow_unit * 70) return 0;
    }
    /* Require a clear quiet zone ahead of the guard so we don't latch onto
     * four incidentally-equal narrow elements in the middle of unrelated
     * data. */
    if (s > 0 && runs->c[s - 1] == 0 && runs->w[s - 1] < narrow_unit * 2) return 0;
    return narrow_unit;
}

static int decode_hk25(const Runs *runs, char *out, int out_cap)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN];
    int max_start_m = runs->len;

    if (max_start_m > 40) max_start_m = 40;
    for (int s = 0; s < max_start_m && s + 3 < runs->len; s++) {
        int narrow_unit = check_hk25_start(runs, s);
        int data_start;
        int max_digits;

        if (narrow_unit <= 0) continue;
        data_start = s + 4;
        max_digits = (runs->len - data_start) / 6;
        if (max_digits > BARCODE_MAX_LEN - 1) max_digits = BARCODE_MAX_LEN - 1;
        if (max_digits < 1) continue;

        for (int nd = max_digits; nd >= 1; nd--) {
            int stop_pos = data_start + nd * 6;
            char tmp[BARCODE_MAX_LEN];
            int n = 0;
            int ok = 1;
            int nu = narrow_unit;

            if (stop_pos >= runs->len) continue;
            if (!check_matrix_stop(runs, stop_pos, narrow_unit)) continue;

            for (int di = 0; di < nd; di++) {
                int p = data_start + di * 6;
                int widths[5];
                int digit;

                if (p + 4 >= runs->len) { ok = 0; break; }
                if (runs->c[p] != 1 || runs->c[p + 1] != 0 || runs->c[p + 2] != 1 ||
                    runs->c[p + 3] != 0 || runs->c[p + 4] != 1) {
                    ok = 0;
                    break;
                }
                for (int b = 0; b < 5; b++) widths[b] = runs->w[p + b];

                digit = decode_digit(widths, nu);
                if (digit < 0) {
                    ok = 0;
                    break;
                }
                tmp[n++] = (char)('0' + digit);

                {
                    int sum_n = 0;
                    int cnt_n = 0;
                    for (int b = 0; b < 5; b++) {
                        if (is_narrow(widths[b], nu)) {
                            sum_n += widths[b];
                            cnt_n++;
                        }
                    }
                    if (cnt_n >= 2) nu = sum_n / cnt_n;
                }
            }

            if (ok && n > best_len) {
                memcpy(best, tmp, (size_t)n);
                best[n] = '\0';
                best_len = n;
                break;
            }
        }
    }

    if (best_len < 1 || best_len >= out_cap) return 0;
    memcpy(out, best, (size_t)best_len + 1);
    return best_len;
}

static void reverse_array(int *arr, int n)
{
    for (int i = 0; i < n / 2; i++) {
        int t = arr[i];
        arr[i] = arr[n - 1 - i];
        arr[n - 1 - i] = t;
    }
}

static void reverse_string(char *s, int len)
{
    for (int i = 0; i < len / 2; i++) {
        char t = s[i];
        s[i] = s[len - 1 - i];
        s[len - 1 - i] = t;
    }
}

static void reverse_runs(const Runs *src, Runs *dst)
{
    dst->len = src->len;
    for (int i = 0; i < src->len; i++) {
        dst->w[i] = src->w[src->len - 1 - i];
        dst->c[i] = src->c[src->len - 1 - i];
    }
}

int s25_decode(const Image *img, BarcodeResult *result)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN] = {0};

    if (!img || !img->pixels || img->width < 32 || img->height < 8 || !result) return 0;
    memset(result, 0, sizeof(*result));

    for (int invert = 0; invert <= 1; invert++) {
        uint8_t inv_row[8192];

        for (int k = -6; k <= 6; k++) {
            int y = img->height / 2 + (k * img->height) / 24;
            const uint8_t *row;
            int w;
            Runs runs;
            int bar_w[MAX_BARS];
            int nb;
            char text[BARCODE_MAX_LEN];
            int n = 0;

            if (y < 0) y = 0;
            if (y >= img->height) y = img->height - 1;

            row = img->pixels + (size_t)y * (size_t)img->width;
            w = img->width < 8192 ? img->width : 8192;

            if (invert) {
                for (int x = 0; x < w; x++) inv_row[x] = (uint8_t)(255 - row[x]);
                row = inv_row;
            }

            if (extract_runs(row, w, &runs) < 10) continue;

            nb = extract_bars(&runs, bar_w, MAX_BARS);
            if (nb >= 9) {
                n = decode_industrial_bars(bar_w, nb, text, BARCODE_MAX_LEN);
                if (n <= 0) {
                    int rev_bars[MAX_BARS];
                    memcpy(rev_bars, bar_w, sizeof(int) * (size_t)nb);
                    reverse_array(rev_bars, nb);
                    n = decode_industrial_bars(rev_bars, nb, text, BARCODE_MAX_LEN);
                    if (n > 0) reverse_string(text, n);
                }
                {
                    char iata_text[BARCODE_MAX_LEN];
                    int iata_n = decode_iata_bars(bar_w, nb, iata_text, BARCODE_MAX_LEN);
                    if (iata_n <= 0) {
                        int rev_bars[MAX_BARS];
                        memcpy(rev_bars, bar_w, sizeof(int) * (size_t)nb);
                        reverse_array(rev_bars, nb);
                        iata_n = decode_iata_bars(rev_bars, nb, iata_text, BARCODE_MAX_LEN);
                        if (iata_n > 0) reverse_string(iata_text, iata_n);
                    }
                    if (iata_n > n) {
                        n = iata_n;
                        memcpy(text, iata_text, (size_t)iata_n + 1);
                    }
                }
            }

            {
                char mat_text[BARCODE_MAX_LEN];
                int mat_n = decode_matrix(&runs, mat_text, BARCODE_MAX_LEN);
                if (mat_n <= 0) {
                    Runs rev;
                    reverse_runs(&runs, &rev);
                    mat_n = decode_matrix(&rev, mat_text, BARCODE_MAX_LEN);
                    if (mat_n > 0) reverse_string(mat_text, mat_n);
                }
                if (mat_n > n) {
                    n = mat_n;
                    memcpy(text, mat_text, (size_t)mat_n + 1);
                }
            }

            {
                char hk_text[BARCODE_MAX_LEN];
                int hk_n = decode_hk25(&runs, hk_text, BARCODE_MAX_LEN);
                if (hk_n <= 0) {
                    Runs rev;
                    reverse_runs(&runs, &rev);
                    hk_n = decode_hk25(&rev, hk_text, BARCODE_MAX_LEN);
                    if (hk_n > 0) reverse_string(hk_text, hk_n);
                }
                if (hk_n > n) {
                    n = hk_n;
                    memcpy(text, hk_text, (size_t)hk_n + 1);
                }
            }

            if (n > best_len) {
                best_len = n;
                memcpy(best, text, (size_t)n + 1);
            }
        }
    }

    if (best_len < 8) return 0;
    memcpy(result->data, best, (size_t)best_len + 1);
    result->length = best_len;
    result->confidence = (best_len >= 8) ? 80 : (50 + best_len * 3);
    if (result->confidence > 95) result->confidence = 95;
    result->symbology = BARCODE_SYM_S25;
    return 1;
}
