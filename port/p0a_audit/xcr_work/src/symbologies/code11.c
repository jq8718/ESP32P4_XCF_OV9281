/*
 * Code 11 decoder.
 */

#include "code11.h"

#include <stdint.h>
#include <string.h>

#define MAX_RUNS 4096

static const uint8_t C11_PATTERNS[12] = {
    0x01, 0x11, 0x09, 0x18, 0x05, 0x14,
    0x0C, 0x02, 0x12, 0x10, 0x04, 0x06,
};
static const char C11_CHARS[] = "0123456789-";

static int otsu_threshold(const uint8_t *row, int width)
{
    int hist[256] = {0};
    long long sum = 0;
    long long sb = 0;
    int wb = 0;
    double mv = -1.0;
    int thr = 127;

    for (int x = 0; x < width; x++) hist[row[x]]++;
    for (int i = 0; i < 256; i++) sum += (long long)i * hist[i];
    for (int t = 0; t < 256; t++) {
        wb += hist[t];
        if (!wb) continue;
        {
            int wf = width - wb;
            if (!wf) break;
            sb += (long long)t * hist[t];
            {
                double d = (double)sb / wb - (double)(sum - sb) / wf;
                double v = (double)wb * (double)wf * d * d;
                if (v > mv) {
                    mv = v;
                    thr = t;
                }
            }
        }
    }
    if (thr < 1) thr = 1;
    if (thr > 254) thr = 254;
    return thr;
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
    int cnt;

    runs->len = 0;
    if (width < 16) return 0;
    thr = otsu_threshold(row, width);
    cur = (row[0] < thr) ? 1 : 0;
    cnt = 1;
    for (int x = 1; x < width; x++) {
        uint8_t v = (row[x] < thr) ? 1 : 0;
        if (v == cur) {
            cnt++;
        } else {
            if (runs->len >= MAX_RUNS) return 0;
            runs->w[runs->len] = cnt;
            runs->c[runs->len] = cur;
            runs->len++;
            cur = v;
            cnt = 1;
        }
    }
    if (runs->len >= MAX_RUNS) return 0;
    runs->w[runs->len] = cnt;
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

static int classify_char(const int *widths, int narrow_unit)
{
    int thr = narrow_unit * 7 / 4;
    int pattern = 0;
    for (int i = 0; i < 5; i++) {
        pattern = (pattern << 1) | (widths[i] > thr ? 1 : 0);
    }
    for (int i = 0; i < 12; i++) {
        if (C11_PATTERNS[i] == pattern) return i;
    }
    return -1;
}

static int estimate_narrow(const int *w, int n)
{
    int min_w = w[0];
    for (int i = 1; i < n; i++) {
        if (w[i] < min_w) min_w = w[i];
    }
    return min_w > 0 ? min_w : 1;
}

static int try_decode_c11(const Runs *runs, int start_run, char *out, int out_cap)
{
    int pos = start_run;
    int remaining = runs->len - pos;
    int sample_end;
    int nu;
    int thr;
    int widths[5];
    char buf[256];
    int n = 0;

    if (runs->c[start_run] != 1) return 0;
    if (remaining < 11) return 0;

    sample_end = pos + 20;
    if (sample_end > runs->len) sample_end = runs->len;
    nu = estimate_narrow(&runs->w[pos], sample_end - pos);
    thr = nu * 7 / 4;

    for (int i = 0; i < 5; i++) widths[i] = runs->w[pos + i];
    if (classify_char(widths, nu) != 11) return 0;
    pos += 5;

    if (pos >= runs->len || runs->c[pos] != 0 || runs->w[pos] > thr) return 0;
    pos++;

    while (pos + 4 < runs->len && n < 250) {
        int ch;
        for (int i = 0; i < 5; i++) widths[i] = runs->w[pos + i];
        ch = classify_char(widths, nu);
        if (ch < 0) return 0;
        if (ch == 11) {
            if (pos + 5 < runs->len && runs->c[pos + 5] == 0 && runs->w[pos + 5] < nu * 2) {
                return 0;
            }
            break;
        }
        if (ch > 10) return 0;
        buf[n++] = C11_CHARS[ch];
        pos += 5;
        if (pos >= runs->len || runs->c[pos] != 0 || runs->w[pos] > thr) return 0;
        {
            int sum_n = 0;
            int cnt_n = 0;
            for (int i = 0; i < 5; i++) {
                if (widths[i] <= thr) {
                    sum_n += widths[i];
                    cnt_n++;
                }
            }
            if (cnt_n >= 2) nu = sum_n / cnt_n;
            thr = nu * 7 / 4;
        }
        pos++;
    }

    if (n < 3 || n >= out_cap) return 0;
    buf[n] = '\0';
    memcpy(out, buf, (size_t)n + 1);
    return n;
}

int code11_decode(const Image *img, BarcodeResult *result)
{
    int best_len = 0;
    char best[256] = {0};

    if (!img || !img->pixels || img->width < 32 || img->height < 8 || !result) return 0;
    memset(result, 0, sizeof(*result));

    for (int invert = 0; invert <= 1; invert++) {
        uint8_t inv_row[8192];

        for (int k = -4; k <= 4; k++) {
            int y = img->height / 2 + (k * img->height) / 16;
            const uint8_t *row;
            int w;
            Runs runs;

            if (y < 0) y = 0;
            if (y >= img->height) y = img->height - 1;

            row = img->pixels + (size_t)y * (size_t)img->width;
            w = img->width < 8192 ? img->width : 8192;

            if (invert) {
                for (int x = 0; x < w; x++) inv_row[x] = (uint8_t)(255 - row[x]);
                row = inv_row;
            }

            if (extract_runs(row, w, &runs) < 11) continue;

            {
                int max_s = runs.len / 3;
                if (max_s > 20) max_s = 20;
                for (int s = 0; s < max_s; s++) {
                    char text[256];
                    int n;
                    if (runs.c[s] != 1) continue;
                    n = try_decode_c11(&runs, s, text, 256);
                    if (n > best_len) {
                        best_len = n;
                        memcpy(best, text, (size_t)n + 1);
                    }
                }
            }
        }
    }

    if (best_len < 6) return 0;
    memcpy(result->data, best, (size_t)best_len + 1);
    result->length = best_len;
    result->confidence = (best_len >= 6) ? 80 : 60;
    result->symbology = BARCODE_SYM_CODE11;
    return 1;
}
