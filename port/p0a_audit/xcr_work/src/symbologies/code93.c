#include "code93.h"

#include <stdint.h>
#include <string.h>

#define CODE93_MAX_EDGES 512
#define CODE93_STARTSTOP 47

/*
 * Code 93: each character = 3 bars + 3 spaces = 6 elements, 9 modules total.
 * 43 data chars + 4 shift chars + start/stop = 48 patterns.
 * Patterns stored as {bar1, space1, bar2, space2, bar3, space3}.
 */
static const uint8_t k_c93_pat[48][6] = {
    /*  0 */ {1,3,1,1,1,2},  /*  1 */ {1,1,1,2,1,3},
    /*  2 */ {1,1,1,3,1,2},  /*  3 */ {1,1,1,4,1,1},
    /*  4 */ {1,2,1,1,1,3},  /*  5 */ {1,2,1,2,1,2},
    /*  6 */ {1,2,1,3,1,1},  /*  7 */ {1,1,1,1,1,4},
    /*  8 */ {1,3,1,2,1,1},  /*  9 */ {1,4,1,1,1,1},
    /* A  */ {2,1,1,1,1,3},  /* B  */ {2,1,1,2,1,2},
    /* C  */ {2,1,1,3,1,1},  /* D  */ {2,2,1,1,1,2},
    /* E  */ {2,2,1,2,1,1},  /* F  */ {2,3,1,1,1,1},
    /* G  */ {1,1,2,1,1,3},  /* H  */ {1,1,2,2,1,2},
    /* I  */ {1,1,2,3,1,1},  /* J  */ {1,2,2,1,1,2},
    /* K  */ {1,3,2,1,1,1},  /* L  */ {1,1,1,1,2,3},
    /* M  */ {1,1,1,2,2,2},  /* N  */ {1,1,1,3,2,1},
    /* O  */ {1,2,1,1,2,2},  /* P  */ {1,3,1,1,2,1},
    /* Q  */ {2,1,2,1,1,2},  /* R  */ {2,1,2,2,1,1},
    /* S  */ {2,1,1,1,2,2},  /* T  */ {2,1,1,2,2,1},
    /* U  */ {2,2,1,1,2,1},  /* V  */ {2,2,2,1,1,1},
    /* W  */ {1,1,2,1,2,2},  /* X  */ {1,1,2,2,2,1},
    /* Y  */ {1,2,2,1,2,1},  /* Z  */ {1,2,3,1,1,1},
    /* -  */ {3,1,1,1,1,2},  /* .  */ {3,1,1,2,1,1},
    /* SP */ {3,2,1,1,1,1},  /* $  */ {3,1,1,1,2,1},
    /* /  */ {3,1,2,1,1,1},  /* +  */ {2,1,1,1,3,1},
    /* %  */ {1,1,3,2,1,1},
    /* ($) */ {1,2,1,2,2,1},  /* (/) */ {3,1,2,1,1,1},
    /* (+) */ {3,1,1,1,2,1},  /* (%) */ {1,2,2,2,1,1},
    /* *   */ {1,1,1,1,4,1},
};

static const char k_c93_chars[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

/* Decode one 6-element character from edge positions.
 * edges[pos..pos+6] gives 7 edge positions spanning 6 elements.
 * Returns character index 0-47 on success, -1 on failure.
 */
static int decode_char(const int *edges, int pos, int edge_count)
{
    int widths[6];
    int total;
    int best;
    int best_err;

    if (pos + 6 >= edge_count) {
        return -1;
    }

    total = edges[pos + 6] - edges[pos];
    if (total <= 0) {
        return -1;
    }

    /* Compute normalized widths: each element_width * 9 / total_width */
    for (int i = 0; i < 6; i++) {
        int w = edges[pos + i + 1] - edges[pos + i];
        widths[i] = (w * 9 + total / 2) / total;
        if (widths[i] < 1) widths[i] = 1;
    }

    /* Find best matching pattern */
    best = -1;
    best_err = 9999;

    for (int c = 0; c < 48; c++) {
        int err = 0;
        for (int j = 0; j < 6; j++) {
            int d = widths[j] - k_c93_pat[c][j];
            if (d < 0) d = -d;
            err += d;
        }
        if (err < best_err) {
            best_err = err;
            best = c;
        }
    }

    /* Also check alternate patterns found in some Code93 generators */
    if (best_err > 0) {
        static const struct { uint8_t pat[6]; int idx; } k_alt[] = {
            {{1,2,1,1,3,1}, 36}, /* '-' alternate */
            {{1,1,2,1,3,1}, 40}, /* '/' alternate */
            {{1,1,3,1,2,1}, 41}, /* '+' alternate */
        };
        for (int a = 0; a < 3; a++) {
            int err = 0;
            for (int j = 0; j < 6; j++) {
                int d = widths[j] - k_alt[a].pat[j];
                if (d < 0) d = -d;
                err += d;
            }
            if (err < best_err) {
                best_err = err;
                best = k_alt[a].idx;
            }
        }
    }

    /* Allow up to 4 units of error for more tolerance */
    if (best_err > 4) {
        return -1;
    }

    return best;
}

/* Verify C and K check characters.
 * symbols[0..count-1] where last two are C and K checks.
 * Both checks are mandatory; return count-2 on exact pass or -1 on failure.
 */
static int verify_checksums(const int *symbols, int count)
{
    int n;
    int sum;
    int weight;
    int c_computed;
    int c_diff;
    int k_computed;
    int k_diff;

    if (count < 3) {
        return -1;
    }

    n = count - 2;

    /* C check: weight from right, 1..20 cycling */
    sum = 0;
    weight = 1;
    for (int i = n - 1; i >= 0; i--) {
        sum += symbols[i] * weight;
        weight++;
        if (weight > 20) weight = 1;
    }
    c_computed = sum % 47;
    c_diff = c_computed - symbols[n];
    if (c_diff < 0) c_diff = -c_diff;
    if (c_diff > 47 - c_diff) c_diff = 47 - c_diff;

    /* K check: includes C, weight from right, 1..15 cycling */
    sum = 0;
    weight = 1;
    for (int i = n; i >= 0; i--) {
        sum += symbols[i] * weight;
        weight++;
        if (weight > 15) weight = 1;
    }
    k_computed = sum % 47;
    k_diff = k_computed - symbols[n + 1];
    if (k_diff < 0) k_diff = -k_diff;
    if (k_diff > 47 - k_diff) k_diff = 47 - k_diff;

    if (c_diff == 0 && k_diff == 0) {
        return n;
    }

    return -1;
}

static int code93_decode_edges_ex(const int *edges, int edge_count,
                        char *out, int out_cap, int strict)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN] = {0};

    (void)strict;

    if (!edges || edge_count < 14 || !out || out_cap < 2) {
        return 0;
    }

    for (int start = 0; start + 6 < edge_count; start++) {
        int sym;
        int symbols[BARCODE_MAX_LEN];
        int count;
        int pos;
        int payload_len;

        sym = decode_char(edges, start, edge_count);
        if (sym != CODE93_STARTSTOP) {
            continue;
        }

        count = 0;
        pos = start + 6;
        while (pos + 6 < edge_count && count < BARCODE_MAX_LEN - 1) {
            sym = decode_char(edges, pos, edge_count);
            if (sym < 0) {
                break;
            }
            if (sym == CODE93_STARTSTOP) {
                goto found_stop;
            }
            symbols[count++] = sym;
            pos += 6;
        }
        continue;

found_stop:
        if (count < 3) {
            continue;
        }

        payload_len = verify_checksums(symbols, count);

        if (payload_len < 1 || payload_len + 1 > out_cap) {
            continue;
        }

        {
            char tmp[BARCODE_MAX_LEN];
            int ok = 1;
            for (int i = 0; i < payload_len; i++) {
                if (symbols[i] < 0 || symbols[i] >= 43) {
                    ok = 0;
                    break;
                }
                tmp[i] = k_c93_chars[symbols[i]];
            }
            if (!ok) {
                continue;
            }
            tmp[payload_len] = '\0';

            if (payload_len > best_len) {
                best_len = payload_len;
                memcpy(best, tmp, (size_t)payload_len + 1);
            }
        }
    }

    if (best_len <= 0 || best_len + 1 > out_cap) {
        return 0;
    }

    memcpy(out, best, (size_t)best_len + 1);
    return best_len;
}

int code93_decode_edges(const int *edges, int edge_count,
                        char *out, int out_cap)
{
    return code93_decode_edges_ex(edges, edge_count, out, out_cap, 0);
}

/*
 * Simple threshold-based edge detector for a row of pixels.
 * More reliable for Code93 than the firmware's signal_transitions
 * which may split wide elements.
 */
static int simple_edges(const uint8_t *row, int width,
                        int *edges, int max_edges)
{
    int lo = 255;
    int hi = 0;
    int thr;
    int prev;
    int n = 0;
    int first_is_bar;

    /* Find min/max */
    for (int x = 0; x < width; x++) {
        if (row[x] < lo) lo = row[x];
        if (row[x] > hi) hi = row[x];
    }
    if (hi - lo < 30) {
        return 0; /* not enough contrast */
    }
    thr = (lo + hi) / 2;

    /* Find transitions */
    prev = (row[0] < thr) ? 1 : 0; /* 1 = bar (dark) */
    first_is_bar = prev;

    /* Code93 must start with a bar. If first pixel is space, add a phantom edge at 0 */
    if (!first_is_bar && n < max_edges) {
        edges[n++] = 0;
    }

    for (int x = 1; x < width && n < max_edges; x++) {
        int cur = (row[x] < thr) ? 1 : 0;
        if (cur != prev) {
            edges[n++] = x;
            prev = cur;
        }
    }
    return n;
}

static int code93_decode_ex(const Image *img, BarcodeResult *result, int strict)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN] = {0};

    if (!img || !img->pixels || img->width <= 16 || img->height <= 8 || !result) {
        return 0;
    }

    memset(result, 0, sizeof(*result));

    /* Try normal polarity first, then inverted (dark background) */
    for (int invert = 0; invert <= 1 && best_len <= 0; invert++) {
        uint8_t inv_row[8192];

        for (int k = -4; k <= 4; k++) {
            int y = img->height / 2 + (k * img->height) / 18;
            int edges[CODE93_MAX_EDGES];
            int n_edges;
            char text[BARCODE_MAX_LEN];
            int n;
            const uint8_t *row;

            if (y < 1) y = 1;
            if (y >= img->height - 1) y = img->height - 2;

            row = img->pixels + (size_t)y * (size_t)img->width;

            if (invert) {
                int w = img->width < 8192 ? img->width : 8192;
                for (int x = 0; x < w; x++)
                    inv_row[x] = (uint8_t)(255 - row[x]);
                row = inv_row;
            }

            n_edges = simple_edges(row, img->width, edges, CODE93_MAX_EDGES);
            if (n_edges < 14) {
                continue;
            }

            n = code93_decode_edges_ex(edges, n_edges, text, (int)sizeof(text), strict);
            if (n > best_len) {
                best_len = n;
                memcpy(best, text, (size_t)n + 1);
            }
        }
    }

    if (best_len <= 0) {
        return 0;
    }

    /* VALIDATION: Strengthen Code93 detection to reduce false positives
     * Valid length: typically 4-48 characters (min 1 data + 2 checks + start/stop)
     * Valid character set: 0-9, A-Z, -. $/+% (no lowercase, no other specials) */

    /* Check minimum length (must have at least some data beyond checks) */
    if (best_len < 4) {
        return 0;  /* Too short to be valid Code93 */
    }

    /* Validate character set: Code93 only uses uppercase alphanumeric and limited specials */
    for (int i = 0; i < best_len; i++) {
        char c = best[i];
        if (!((c >= '0' && c <= '9') ||   /* digits */
              (c >= 'A' && c <= 'Z') ||   /* uppercase letters */
              c == '-' || c == '.' || c == ' ' ||  /* special chars */
              c == '$' || c == '/' || c == '+' || c == '%')) {
            return 0;  /* Invalid character for Code93 */
        }
    }

    memcpy(result->data, best, (size_t)best_len + 1);
    result->length = best_len;
    result->confidence = (best_len > 12) ? 90 : (60 + best_len * 2);
    if (result->confidence > 100) {
        result->confidence = 100;
    }
    return 1;
}

int code93_decode(const Image *img, BarcodeResult *result)
{
    return code93_decode_ex(img, result, 0);
}

int code93_decode_high_conf(const Image *img, BarcodeResult *result)
{
    return code93_decode_ex(img, result, 1);
}
