/*
 * MSI (Modified Plessey) barcode decoder.
 *
 * Encoding: each digit = 4 bar-space pairs = 8 elements, MSB first.
 *   Bit 1: wide bar + narrow space (WN)
 *   Bit 0: narrow bar + wide space (NW)
 *
 * Start: wide bar + narrow space (WN pair).
 * Stop:  narrow bar + wide space + narrow bar (NWN).
 */

#include "msi.h"

#include <stdint.h>
#include <string.h>

#define MAX_RUNS 4096

static int otsu_threshold(const uint8_t *row, int width)
{
    int hist[256] = {0};
    long long sum = 0;
    long long sb = 0;
    int wb = 0;
    double mv = -1.0;
    int thr = 127;

    for (int x = 0; x < width; x++) {
        hist[row[x]]++;
    }
    for (int i = 0; i < 256; i++) {
        sum += (long long)i * hist[i];
    }
    for (int t = 0; t < 256; t++) {
        wb += hist[t];
        if (!wb) {
            continue;
        }
        {
            int wf = width - wb;
            if (!wf) {
                break;
            }
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

static int is_wide(int w, int nu)
{
    return w > nu * 7 / 4;
}

static int try_decode_msi(const Runs *runs, int s, char *out, int out_cap)
{
    int pos = s;
    int sample_end;
    int min_w;
    int nu;
    char buf[256];
    int n = 0;

    if (pos + 10 >= runs->len) return 0;
    if (runs->c[pos] != 1) return 0;

    sample_end = pos + 24;
    if (sample_end > runs->len) sample_end = runs->len;
    min_w = runs->w[pos];
    for (int i = pos + 1; i < sample_end; i++) {
        if (runs->w[i] < min_w) min_w = runs->w[i];
    }
    nu = min_w > 0 ? min_w : 1;

    if (!is_wide(runs->w[pos], nu)) return 0;
    if (runs->c[pos + 1] != 0 || is_wide(runs->w[pos + 1], nu)) return 0;
    pos += 2;

    while (pos + 7 < runs->len && n < 250) {
        int digit = 0;
        int ok = 1;

        if (runs->c[pos] == 1 && !is_wide(runs->w[pos], nu) &&
            pos + 2 < runs->len &&
            runs->c[pos + 1] == 0 && is_wide(runs->w[pos + 1], nu) &&
            runs->c[pos + 2] == 1 && !is_wide(runs->w[pos + 2], nu)) {
            if (pos + 10 >= runs->len) break;
        }

        for (int b = 0; b < 4; b++) {
            int bi = pos + b * 2;
            int si = pos + b * 2 + 1;
            int bw;
            int sw;

            if (bi >= runs->len || si >= runs->len) {
                ok = 0;
                break;
            }
            if (runs->c[bi] != 1 || runs->c[si] != 0) {
                ok = 0;
                break;
            }

            bw = is_wide(runs->w[bi], nu);
            sw = is_wide(runs->w[si], nu);
            if (bw && !sw) {
                digit = (digit << 1) | 1;
            } else if (!bw && sw) {
                digit <<= 1;
            } else {
                ok = 0;
                break;
            }
        }
        if (!ok || digit > 9) break;

        buf[n++] = (char)('0' + digit);
        pos += 8;

        {
            int sum_n = 0;
            int cnt_n = 0;
            for (int i = pos - 8; i < pos && i < runs->len; i++) {
                if (!is_wide(runs->w[i], nu)) {
                    sum_n += runs->w[i];
                    cnt_n++;
                }
            }
            if (cnt_n >= 3) nu = sum_n / cnt_n;
        }
    }

    if (pos + 2 < runs->len) {
        if (!(runs->c[pos] == 1 && !is_wide(runs->w[pos], nu) &&
              runs->c[pos + 1] == 0 && is_wide(runs->w[pos + 1], nu) &&
              runs->c[pos + 2] == 1 && !is_wide(runs->w[pos + 2], nu))) {
            return 0;
        }
    } else if (pos < runs->len) {
        if (!(runs->c[pos] == 1 && !is_wide(runs->w[pos], nu))) return 0;
    } else {
        return 0;
    }

    if (n < 4 || n >= out_cap) return 0;
    buf[n] = '\0';
    memcpy(out, buf, (size_t)n + 1);
    return n;
}

int msi_decode(const Image *img, BarcodeResult *result)
{
    int best_len = 0;
    char best[256] = {0};

    if (!img || !img->pixels || img->width < 32 || img->height < 8 || !result) return 0;
    memset(result, 0, sizeof(*result));

    for (int invert = 0; invert <= 1; invert++) {
        uint8_t inv_row[8192];

        for (int k = -6; k <= 6; k++) {
            int y = img->height / 2 + (k * img->height) / 24;
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

            if (extract_runs(row, w, &runs) < 10) continue;

            {
                int max_s = runs.len / 4;
                if (max_s > 15) max_s = 15;
                for (int s = 0; s < max_s; s++) {
                    char text[256];
                    int n = try_decode_msi(&runs, s, text, 256);
                    if (n > best_len) {
                        best_len = n;
                        memcpy(best, text, (size_t)n + 1);
                    }
                }
            }
        }
    }

    if (best_len < 4) return 0;
    memcpy(result->data, best, (size_t)best_len + 1);
    result->length = best_len;
    result->confidence = (best_len >= 6) ? 80 : 60;
    result->symbology = BARCODE_SYM_MSI;
    return 1;
}
