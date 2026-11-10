/*
 * AIM USS Telepen decoder.
 *
 * Telepen does not define independent per-character bar/space patterns;
 * instead every 8-bit byte (7 data bits + 1 even-parity bit, transmitted
 * LSB first) is expanded into a stream of bar/space elements according to
 * a small set of bit-block rules (see decode_char_bits() below). Every
 * element is either "narrow" (1 module) or "wide" (3 modules), and every
 * character occupies exactly 16 modules regardless of its bit pattern.
 *
 * Start character: '_' (0x5F, parity byte 0x5F) -> five narrow bar/space
 * pairs followed by one wide bar/space pair.
 * Stop character:  'z' (0x7A, parity byte 0xFA) -> one wide bar/space pair
 * followed by five narrow bar/space pairs.
 *
 * A mandatory trailing checksum character (mod 127 over the 7-bit data
 * values) precedes the stop character.
 *
 * Two payload interpretations are supported:
 *   - Full ASCII: every payload byte is a literal printable ASCII value.
 *   - Compressed Numeric: every payload byte encodes a two-digit decimal
 *     pair via value = 27 + (10*d1 + d2); an even-length result starting
 *     with a padding '0' (added by the encoder to make an odd digit count
 *     even) has that leading '0' stripped.
 */

#include "telepen.h"

#include <stdint.h>
#include <string.h>

#define MAX_RUNS 4096
#define MAX_CHARS 64

typedef struct {
    int len;
    int w[MAX_RUNS];
    uint8_t c[MAX_RUNS]; /* 1 = bar (dark), 0 = space */
} Runs;

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

static int extract_runs(const uint8_t *row, int width, Runs *runs)
{
    int thr;
    uint8_t cur;
    int count;

    runs->len = 0;
    if (width < 32) return 0;

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

    /* Merge away 1px glitch runs, same convention as other 1D decoders. */
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

static inline int is_wide(int w, int narrow_unit)
{
    return w > narrow_unit * 2;
}

/* Locate the start guard: 5 narrow bar/space pairs then 1 wide bar/space
 * pair (12 elements total), starting with a bar. Returns the estimated
 * narrow unit (>0) on match, 0 otherwise. */
static int check_telepen_start(const Runs *runs, int s)
{
    int sum;
    int narrow_unit;

    if (s + 11 >= runs->len) return 0;
    for (int i = 0; i < 12; i++) {
        int expect_bar = (i % 2 == 0);
        if (expect_bar) {
            if (runs->c[s + i] != 1) return 0;
        } else {
            if (runs->c[s + i] != 0) return 0;
        }
    }

    sum = 0;
    for (int i = 0; i < 10; i++) sum += runs->w[s + i];
    narrow_unit = sum / 10;
    if (narrow_unit <= 0) return 0;

    for (int i = 0; i < 10; i++) {
        int diff = runs->w[s + i] - narrow_unit;
        if (diff < 0) diff = -diff;
        if (diff * 100 > narrow_unit * 60) return 0;
    }
    for (int i = 10; i < 12; i++) {
        int expect = narrow_unit * 3;
        int diff = runs->w[s + i] - expect;
        if (diff < 0) diff = -diff;
        if (diff * 100 > expect * 45) return 0;
    }
    /* Require a plausible quiet zone ahead of the guard. */
    if (s > 0 && runs->w[s - 1] < narrow_unit * 2) return 0;
    return narrow_unit;
}

/* Check for the stop guard: 1 wide bar/space pair then 5 narrow bar/space
 * pairs (11 elements needed; the trailing quiet-zone space beyond the
 * final bar is not constrained). */
static int check_telepen_stop(const Runs *runs, int t, int narrow_unit)
{
    if (t + 10 >= runs->len) return 0;
    for (int i = 0; i <= 10; i++) {
        int expect_bar = (i % 2 == 0);
        if (expect_bar) {
            if (runs->c[t + i] != 1) return 0;
        } else {
            if (runs->c[t + i] != 0) return 0;
        }
    }
    {
        int diff0 = runs->w[t] - narrow_unit * 3;
        int diff1 = runs->w[t + 1] - narrow_unit * 3;
        if (diff0 < 0) diff0 = -diff0;
        if (diff1 < 0) diff1 = -diff1;
        if (diff0 * 100 > narrow_unit * 3 * 45) return 0;
        if (diff1 * 100 > narrow_unit * 3 * 45) return 0;
    }
    for (int i = 2; i <= 10; i += 2) {
        int diff = runs->w[t + i] - narrow_unit;
        if (diff < 0) diff = -diff;
        if (diff * 100 > narrow_unit * 60) return 0;
    }
    for (int i = 3; i <= 9; i += 2) {
        int diff = runs->w[t + i] - narrow_unit;
        if (diff < 0) diff = -diff;
        if (diff * 100 > narrow_unit * 60) return 0;
    }
    return 1;
}

/*
 * Decode one 8-bit (7 data + 1 parity) character starting at element
 * index `pos` (must be a bar). Consumes (bar,space) pairs and applies the
 * bit-block automaton:
 *   state OUT, pair (N,N) -> bit 1
 *   state OUT, pair (W,N) -> bits 0,0
 *   state OUT, pair (W,W) -> bits 0,1,0
 *   state OUT, pair (N,W) -> bits 0,1 ; state = IN   (start of 01^n0 block)
 *   state IN,  pair (N,N) -> bit 1                    (middle 1 of block)
 *   state IN,  pair (N,W) -> bits 1,0 ; state = OUT   (end of 01^n0 block)
 * Stops once exactly 8 bits have been collected with state==OUT.
 * Returns number of elements consumed (>0) on success, 0 on failure.
 * *narrow_unit is read for classification and updated on success (running
 * average of the narrow elements seen in this character) to track drift.
 */
static int decode_telepen_char(const Runs *runs, int pos, int *narrow_unit, int *value_out)
{
    int bits[9];
    int nbits = 0;
    int state_in = 0;
    int i = pos;
    int nu = *narrow_unit;
    long sum_n = 0;
    int cnt_n = 0;

    if (nu <= 0) return 0;

    while (nbits < 8) {
        int barw, spacew, bar_wide, space_wide;

        if (i + 1 >= runs->len) return 0;
        if (runs->c[i] != 1 || runs->c[i + 1] != 0) return 0;

        barw = runs->w[i];
        spacew = runs->w[i + 1];
        bar_wide = is_wide(barw, nu);
        space_wide = is_wide(spacew, nu);
        i += 2;

        if (!bar_wide) { sum_n += barw; cnt_n++; }
        if (!space_wide) { sum_n += spacew; cnt_n++; }

        if (!state_in) {
            if (!bar_wide && !space_wide) {
                bits[nbits++] = 1;
            } else if (bar_wide && !space_wide) {
                if (nbits + 2 > 9) return 0;
                bits[nbits++] = 0;
                bits[nbits++] = 0;
            } else if (bar_wide && space_wide) {
                if (nbits + 3 > 9) return 0;
                bits[nbits++] = 0;
                bits[nbits++] = 1;
                bits[nbits++] = 0;
            } else { /* narrow bar, wide space */
                if (nbits + 2 > 9) return 0;
                bits[nbits++] = 0;
                bits[nbits++] = 1;
                state_in = 1;
            }
        } else {
            if (!bar_wide && !space_wide) {
                bits[nbits++] = 1;
            } else if (!bar_wide && space_wide) {
                if (nbits + 2 > 9) return 0;
                bits[nbits++] = 1;
                bits[nbits++] = 0;
                state_in = 0;
            } else {
                return 0; /* invalid element combination inside a block */
            }
        }
        if (nbits > 8) return 0;
    }
    if (nbits != 8 || state_in) return 0;

    {
        int val = 0;
        for (int k = 0; k < 8; k++) val |= (bits[k] << k);
        *value_out = val;
    }
    if (cnt_n >= 2) *narrow_unit = (int)(sum_n / cnt_n);
    return i - pos;
}

static int decode_telepen_runs(const Runs *runs, char *out, int out_cap)
{
    int best_len = 0;
    int max_start = runs->len;

    if (max_start > 80) max_start = 80;

    for (int s = 0; s + 11 < max_start; s++) {
        int narrow_unit = check_telepen_start(runs, s);
        int pos;
        int values[MAX_CHARS];
        int nvals = 0;
        int nu_local;
        int ok = 1;

        if (narrow_unit <= 0) continue;
        nu_local = narrow_unit;
        pos = s + 12;

        for (;;) {
            if (check_telepen_stop(runs, pos, nu_local)) break;
            if (nvals >= MAX_CHARS) { ok = 0; break; }
            {
                int val;
                int consumed = decode_telepen_char(runs, pos, &nu_local, &val);
                if (consumed <= 0) { ok = 0; break; }
                values[nvals++] = val;
                pos += consumed;
            }
        }
        if (!ok || nvals < 2) continue;

        {
            int payload_count = nvals - 1;
            int checksum_val = values[nvals - 1] & 0x7F;
            int sum = 0;
            int all_printable = 1;
            char text[BARCODE_MAX_LEN];
            int tlen = 0;

            for (int k = 0; k < payload_count; k++) sum += values[k] & 0x7F;
            sum %= 127;
            {
                int expected = (127 - sum) % 127;
                if (checksum_val != expected) continue;
            }

            for (int k = 0; k < payload_count; k++) {
                int v = values[k] & 0x7F;
                if (v < 32 || v > 126) { all_printable = 0; break; }
            }

            if (all_printable) {
                for (int k = 0; k < payload_count; k++) {
                    text[tlen++] = (char)(values[k] & 0x7F);
                }
            } else {
                int valid_numeric = 1;
                for (int k = 0; k < payload_count; k++) {
                    int v = values[k] & 0x7F;
                    if (v < 27 || v > 126) { valid_numeric = 0; break; }
                }
                if (!valid_numeric) continue;
                for (int k = 0; k < payload_count; k++) {
                    int v = (values[k] & 0x7F) - 27;
                    int d1 = v / 10;
                    int d2 = v % 10;
                    if (tlen + 2 >= BARCODE_MAX_LEN) { tlen = 0; break; }
                    text[tlen++] = (char)('0' + d1);
                    text[tlen++] = (char)('0' + d2);
                }
                if (tlen >= 2 && (tlen % 2) == 0 && text[0] == '0') {
                    memmove(text, text + 1, (size_t)(tlen - 1));
                    tlen--;
                }
            }

            if (tlen < 1 || tlen >= out_cap) continue;
            if (tlen > best_len) {
                memcpy(out, text, (size_t)tlen);
                out[tlen] = '\0';
                best_len = tlen;
            }
        }
    }
    return best_len;
}

int telepen_decode(const Image *img, BarcodeResult *result)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN] = {0};

    if (!img || !img->pixels || img->width < 32 || img->height < 8 || !result) return 0;
    memset(result, 0, sizeof(*result));

    /* Try both pixel polarities internally (cheap, same trick as s25.c)
     * so a color-inverted symbol still resolves during the fast first
     * dispatch pass instead of waiting for the pipeline's slow
     * multi-second invert/multipass retry. */
    for (int invert = 0; invert <= 1; invert++) {
        for (int k = -6; k <= 6; k++) {
            int y = img->height / 2 + (k * img->height) / 24;
            const uint8_t *row;
            uint8_t inv_row[8192];
            int w;
            Runs runs;
            char text[BARCODE_MAX_LEN];
            int n;

            if (y < 0) y = 0;
            if (y >= img->height) y = img->height - 1;

            row = img->pixels + (size_t)y * (size_t)img->width;
            w = img->width < 8192 ? img->width : 8192;

            if (invert) {
                for (int x = 0; x < w; x++) inv_row[x] = (uint8_t)(255 - row[x]);
                row = inv_row;
            }

            if (extract_runs(row, w, &runs) < 12) continue;

            n = decode_telepen_runs(&runs, text, BARCODE_MAX_LEN);
            if (n > best_len) {
                best_len = n;
                memcpy(best, text, (size_t)n + 1);
            }
        }
    }

    if (best_len < 1) return 0;
    memcpy(result->data, best, (size_t)best_len + 1);
    result->length = best_len;
    result->confidence = (best_len >= 4) ? 85 : (50 + best_len * 5);
    if (result->confidence > 95) result->confidence = 95;
    result->symbology = BARCODE_SYM_TELEPEN;
    return 1;
}
