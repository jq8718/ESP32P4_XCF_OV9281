/*
 * Plessey (UK Plessey) barcode decoder.
 */

#include "plessey.h"

#include <stdio.h>
#include <stdlib.h>
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
    return thr < 1 ? 1 : (thr > 254 ? 254 : thr);
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

static int decode_bit(int bar_w, int space_w, int narrow_unit)
{
    (void)narrow_unit;
    if (bar_w > space_w) return 1;
    if (space_w > bar_w) return 0;
    return -1;
}

static const char HEX_CHARS[] = "0123456789ABCDEF";

static int try_decode_plessey(const Runs *runs, int start, char *out, int out_cap)
{
    int min_w = 9999;
    int count = 0;
    int nu;
    int bits[512];
    int bar_w[512];
    int space_w[512];
    int nbits = 0;
    int pos = start;
    int data_start = -1;
    int n = 0;

    for (int i = start; i < runs->len && count < 12; i++) {
        if (runs->w[i] < min_w) min_w = runs->w[i];
        count++;
    }
    if (min_w <= 0 || min_w > 100) return 0;
    nu = min_w;

    while (pos + 1 < runs->len && nbits < 512) {
        int b;
        if (runs->c[pos] != 1) break;
        b = decode_bit(runs->w[pos], runs->w[pos + 1], nu);
        if (b < 0) break;
        bar_w[nbits] = runs->w[pos];
        space_w[nbits] = runs->w[pos + 1];
        bits[nbits++] = b;
        pos += 2;
    }
    if (nbits < 12) return 0;

    for (int off = 0; off + 4 <= nbits; off++) {
        if (bits[off] == 1 && bits[off + 1] == 1 && bits[off + 2] == 0 && bits[off + 3] == 1) {
            data_start = off + 4;
            break;
        }
    }
    if (data_start < 0) return 0;

    int dup_skips = 0;
    for (int i = data_start; i + 3 < nbits && n < out_cap - 1; i += 4) {
        /* Guard against a spurious duplicated run group: occasionally the
         * run extractor emits one extra bar/space nibble whose widths
         * exactly repeat the immediately preceding nibble (an artifact,
         * not real data). Skip such an exact duplicate; the fixed
         * checksum/stop trim below is shortened by the same amount so the
         * true data/checksum boundary doesn't shift. */
        if (i - 4 >= data_start &&
            bar_w[i] == bar_w[i - 4] && bar_w[i + 1] == bar_w[i - 3] &&
            bar_w[i + 2] == bar_w[i - 2] && bar_w[i + 3] == bar_w[i - 1] &&
            space_w[i] == space_w[i - 4] && space_w[i + 1] == space_w[i - 3] &&
            space_w[i + 2] == space_w[i - 2] && space_w[i + 3] == space_w[i - 1]) {
            dup_skips++;
            continue;
        }
        int val = bits[i] | (bits[i + 1] << 1) | (bits[i + 2] << 2) | (bits[i + 3] << 3);
        if (val < 0 || val > 15) break;
        out[n++] = HEX_CHARS[val];
    }
    out[n] = '\0';

    if (getenv("DBGPLESSEY")) {
        fprintf(stderr, "[PLESSEY] start=%d raw_n=%d raw=\"%s\"\n", start, n, out);
        fprintf(stderr, "[PLESSEY] nbits=%d bits=", nbits);
        for (int i = 0; i < nbits; i++) fprintf(stderr, "%d", bits[i]);
        fprintf(stderr, "\n[PLESSEY] widths(bar,space) from pos=%d:", start);
        for (int i = start; i < start + 2 * nbits + 4 && i + 1 < runs->len; i += 2) {
            fprintf(stderr, " (%d,%d)", runs->w[i], runs->w[i + 1]);
        }
        fprintf(stderr, " nu=%d\n", nu);
    }

    if (n >= 6) {
        int trim = 3 - dup_skips;
        if (trim < 0) trim = 0;
        if (trim > n) trim = n;
        n -= trim;
        out[n] = '\0';
    } else if (n >= 4) {
        int trim = 2 - dup_skips;
        if (trim < 0) trim = 0;
        if (trim > n) trim = n;
        n -= trim;
        out[n] = '\0';
    }

    if (getenv("DBGPLESSEY")) {
        fprintf(stderr, "[PLESSEY] start=%d trimmed_n=%d trimmed=\"%s\"\n", start, n, out);
    }
    return n;
}

int plessey_decode(const Image *img, BarcodeResult *result)
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

            if (extract_runs(row, w, &runs) < 16) continue;

            for (int s = 0; s < runs.len / 4 && s < 20; s++) {
                char text[256];
                int n;
                if (runs.c[s] != 1) continue;
                n = try_decode_plessey(&runs, s, text, 256);
                if (n > best_len) {
                    best_len = n;
                    memcpy(best, text, (size_t)n + 1);
                }
            }
        }
    }

    if (best_len < 7) return 0;
    memcpy(result->data, best, (size_t)best_len + 1);
    result->length = best_len;
    result->confidence = 40;
    result->symbology = BARCODE_SYM_PLESSEY;
    return 1;
}
