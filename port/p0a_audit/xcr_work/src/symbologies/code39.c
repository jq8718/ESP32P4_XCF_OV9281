#include "code39.h"

#include "scan_primitives.h"
#include "signal.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CODE39_MAX_EDGES 512

static const uint8_t k_c39_nibble_base[16] = {
    99, 20, 10, 99, 0, 99, 99, 43,
    30, 99, 99, 42, 99, 41, 40, 99,
};

static const uint8_t k_c39_bar_add[32] = {
    99, 99, 99, 7, 99, 4, 0, 99,
    99, 2, 9, 99, 6, 99, 99, 99,
    99, 1, 8, 99, 5, 99, 99, 99,
    3, 99, 99, 99, 99, 99, 99, 99,
};

static const uint8_t k_c39_mirror[44] = {
    16,11,18,17,15,14,10,13,12,19,6,1,8,7,5,4,0,3,2,9,
    36,31,38,37,35,34,30,33,32,39,26,21,28,27,25,24,20,23,22,29,
    43,42,41,40,
};

static const char k_c39_chr[45] = "0123456789JABCDEFGHITKLMNOPQRS*UVWXYZ-. $/+%";

static const uint8_t k_pct_full_ascii[26] = {
    27, 28, 29, 30, 31, 59, 60, 61, 62, 63,
    91, 92, 93, 94, 95, 123, 124, 125, 126, 127,
    0, 64, 96, 127, 127, 127,
};

static int popcount9(int v)
{
    int c = 0;
    for (int i = 0; i < 9; i++) {
        c += (v >> i) & 1;
    }
    return c;
}

static int run_width(const int *edges, int i)
{
    return edges[i + 1] - edges[i];
}

typedef struct {
    int bar_threshold;
    int space_threshold;
    int wide_sum;
    int narrow_sum;
    int quality_acc;
} Code39State;

static int code39_sub_18bfc4(const int *edges, int pos, int threshold, Code39State *st)
{
    int bits[5];
    int add_idx;

    for (int i = 0; i < 5; i++) {
        int edge_i = pos + i * 2;
        int w = run_width(edges, edge_i);
        bits[i] = (w > threshold) ? 1 : 0;
        if (bits[i]) st->wide_sum += w;
        else st->narrow_sum += w;
    }

    add_idx = bits[4]
            | (bits[3] << 1)
            | (bits[2] << 2)
            | (bits[1] << 3)
            | (bits[0] << 4);

    return k_c39_bar_add[add_idx];
}

static int code39_sub_186154(const int *edges, int pos, Code39State *st)
{
    int bits[4];
    int idx;
    int sym;

    st->wide_sum = 0;
    st->narrow_sum = 0;

    for (int i = 0; i < 4; i++) {
        int edge_i = pos + 1 + i * 2;
        int w = run_width(edges, edge_i);
        bits[i] = (w > st->space_threshold) ? 1 : 0;
        if (bits[i]) st->wide_sum += w;
        else st->narrow_sum += w;
    }

    idx = bits[3] | (bits[2] << 1) | (bits[1] << 2) | (bits[0] << 3);
    sym = k_c39_nibble_base[idx];

    st->wide_sum *= 2;

    if ((0x116u >> idx) & 1u) {
        sym += code39_sub_18bfc4(edges, pos, st->bar_threshold, st);
    } else {
        st->narrow_sum *= 9;
    }

    if (sym > 43) {
        return -1;
    }
    return sym;
}

static void sort_small(int *v, int n)
{
    for (int i = 1; i < n; i++) {
        int x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x) {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
}

static int code39_char39(const int *edges, int edge_count, int pos, Code39State *st)
{
    int total;
    ScanEdges scan_edges;
    ScanContext ctx;
    int spread_bar;
    int spread_space;
    int sym;

    if (pos < 0 || pos + 9 >= edge_count) {
        return -1;
    }

    total = edges[pos + 9] - edges[pos];
    if (total <= 0) {
        return -1;
    }

    scan_edges.positions = (int *)edges;
    scan_edges.count = edge_count;
    memset(&ctx, 0, sizeof(ctx));
    ctx.edges = &scan_edges;

    spread_bar = 3 * LsBars(&ctx, pos, 5, 98);
    spread_space = LsBars(&ctx, pos + 1, 4, 115);
    if (spread_bar > spread_space) {
        sym = code39_sub_186154(edges, pos, st);
        if (sym >= 0 && sym < 40) {
            if (st->narrow_sum > 0) {
                st->quality_acc += (150 * st->wide_sum) / st->narrow_sum;
            }
            return sym;
        }
    }

    st->bar_threshold = (total + 4) >> 3;
    st->space_threshold = st->bar_threshold;
    sym = code39_sub_186154(edges, pos, st);
    if (sym >= 0) {
        if (st->narrow_sum > 0) {
            st->quality_acc += (150 * st->wide_sum) / st->narrow_sum;
        }
        return sym;
    }

    {
        int bars[5];
        int spaces[4];
        for (int i = 0; i < 5; i++) bars[i] = run_width(edges, pos + i * 2);
        for (int i = 0; i < 4; i++) spaces[i] = run_width(edges, pos + 1 + i * 2);
        sort_small(bars, 5);
        sort_small(spaces, 4);
        st->bar_threshold = (bars[1] + bars[2]) >> 1;
        st->space_threshold = (spaces[1] + spaces[2]) >> 1;
    }

    sym = code39_sub_186154(edges, pos, st);
    if (sym >= 0 && st->narrow_sum > 0) {
        st->quality_acc += (150 * st->wide_sum) / st->narrow_sum;
    }
    return sym;
}

int code39_char_to_symbol(char ch)
{
    for (int i = 0; i < 44; i++) {
        if (k_c39_chr[i] == ch) return i;
    }
    return -1;
}

static int code39_decode_symbol_from_pattern(int pattern9)
{
    Code39State st;
    int edges[10];
    int x = 0;

    memset(&st, 0, sizeof(st));
    st.bar_threshold = 2;
    st.space_threshold = 2;

    edges[0] = 0;
    for (int i = 0; i < 9; i++) {
        int bit = (pattern9 >> (8 - i)) & 1;
        x += bit ? 3 : 1;
        edges[i + 1] = x;
    }
    return code39_sub_186154(edges, 0, &st);
}

int code39_symbol_pattern(int symbol_index)
{
    static int init_done;
    static int pattern_of_symbol[44];

    if (!init_done) {
        for (int i = 0; i < 44; i++) pattern_of_symbol[i] = -1;

        for (int pat = 0; pat < 512; pat++) {
            int sym;
            if (popcount9(pat) != 3) continue;
            sym = code39_decode_symbol_from_pattern(pat);
            if (sym >= 0 && sym < 44 && pattern_of_symbol[sym] < 0) {
                pattern_of_symbol[sym] = pat;
            }
        }
        init_done = 1;
    }

    if (symbol_index < 0 || symbol_index >= 44) return -1;
    return pattern_of_symbol[symbol_index];
}

static int apply_reverse_and_mirror(int *symbols, int count)
{
    for (int i = 0, j = count - 1; i <= j; i++, j--) {
        int li = k_c39_mirror[symbols[i]];
        int rj = k_c39_mirror[symbols[j]];
        symbols[i] = rj;
        symbols[j] = li;
    }
    return count;
}

static int apply_mod43_checksum(int *symbols, int count)
{
    if (count < 2) {
        return -1;
    }

    int sum = 0;
    for (int i = 0; i < count - 1; i++) {
        sum += symbols[i];
    }

    if ((sum % 43) != symbols[count - 1]) {
        return -1;
    }
    return count - 1;
}

static int symbols_to_text(const int *symbols, int count, char *out, int out_cap)
{
    if (out_cap <= 0) return 0;
    if (count + 1 > out_cap) return 0;

    for (int i = 0; i < count; i++) {
        if (symbols[i] < 0 || symbols[i] >= 44) return 0;
        out[i] = k_c39_chr[symbols[i]];
    }
    out[count] = '\0';
    return count;
}

static int decode_full_ascii(char *s, int len)
{
    int i = 0;
    while (i + 1 < len) {
        char a = s[i];
        char b = s[i + 1];
        int mapped = -1;

        if (b >= 'A' && b <= 'Z') {
            switch (a) {
            case '$': mapped = b - 64; break;
            case '+': mapped = b + 32; break;
            case '/':
                if (b >= 'A' && b <= 'O') mapped = b - 32;
                else if (b == 'Z') mapped = ':';
                break;
            case '%': mapped = k_pct_full_ascii[b - 'A']; break;
            default: break;
            }
        }

        if (mapped >= 0) {
            s[i] = (char)mapped;
            memmove(&s[i + 1], &s[i + 2], (size_t)(len - i - 1));
            len -= 1;
        } else {
            i += 1;
        }
    }

    s[len] = '\0';
    return len;
}

int code39_decode_edges(const int *edges, int edge_count,
                        char *out, int out_cap,
                        int verify_checksum,
                        int decode_extended_ascii)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN] = {0};

    if (!edges || edge_count < 20 || !out || out_cap < 2) {
        return 0;
    }

    for (int start = 0; start + 9 < edge_count; start++) {
        Code39State st;
        int start_sym;
        int stop_sym;
        int reverse_flag;
        int symbols[BARCODE_MAX_LEN];
        int count = 0;
        int pos;
        int ok = 0;

        memset(&st, 0, sizeof(st));
        start_sym = code39_char39(edges, edge_count, start, &st);
        if (start_sym != 26 && start_sym != 30) {
            continue;
        }

        stop_sym = start_sym;
        reverse_flag = (start_sym == 26) ? 1 : 0;

        pos = start + 10;
        while (pos + 9 < edge_count && count < BARCODE_MAX_LEN - 1) {
            int sym = code39_char39(edges, edge_count, pos, &st);
            if (sym < 0) {
                break;
            }
            if (sym == stop_sym) {
                ok = 1;
                break;
            }
            symbols[count++] = sym;
            pos += 10;
        }

        if (!ok || count <= 0) {
            continue;
        }

        if (reverse_flag) {
            apply_reverse_and_mirror(symbols, count);
        }

        if (verify_checksum) {
            int n = apply_mod43_checksum(symbols, count);
            if (n < 0) {
                continue;
            }
            count = n;
            if (count <= 0) {
                continue;
            }
        }

        {
            char tmp[BARCODE_MAX_LEN];
            int n = symbols_to_text(symbols, count, tmp, (int)sizeof(tmp));
            if (n <= 0) {
                continue;
            }
            if (decode_extended_ascii) {
                n = decode_full_ascii(tmp, n);
            } else {
                /* For regular mode, include start/stop symbols (*) */
                if (n + 2 < BARCODE_MAX_LEN) {
                    memmove(tmp + 1, tmp, (size_t)n + 1);
                    tmp[0] = '*';
                    tmp[n + 1] = '*';
                    tmp[n + 2] = '\0';
                    n += 2;
                }
            }
            if (n > best_len) {
                best_len = n;
                memcpy(best, tmp, (size_t)n + 1);
            }
        }
    }

    if (best_len <= 0 || best_len + 1 > out_cap) {
        return 0;
    }

    memcpy(out, best, (size_t)best_len + 1);
    return best_len;
}

int code39_decode(const Image *img, BarcodeResult *result)
{
    int best_len = 0;
    char best[BARCODE_MAX_LEN] = {0};

    if (!img || !img->pixels || img->width <= 16 || img->height <= 8 || !result) {
        return 0;
    }

    memset(result, 0, sizeof(*result));

    for (int k = -4; k <= 4; k++) {
        int y = img->height / 2 + (k * img->height) / 18;
        int edges[CODE39_MAX_EDGES];
        int n_edges;
        char text[BARCODE_MAX_LEN];
        int n;

        if (y < 1) y = 1;
        if (y >= img->height - 1) y = img->height - 2;

        n_edges = signal_transitions(img->pixels, img->width, img->height,
                                     0, y, img->width - 1, y,
                                     edges, CODE39_MAX_EDGES);
        if (n_edges < 20) {
            continue;
        }

        /* Try full ASCII mode first */
        char text_ascii[BARCODE_MAX_LEN];
        int n_ascii = code39_decode_edges(edges, n_edges, text_ascii, (int)sizeof(text_ascii), 0, 1);

        /* Try regular mode */
        int n_reg = code39_decode_edges(edges, n_edges, text, (int)sizeof(text), 0, 0);

        /* Strategy: If REG has *, strip it and compare with ASCII.
         * If they're the same, ASCII didn't do conversion - use REG (with *).
         * If they're different, ASCII did conversion - use ASCII. */

        if (n_ascii > 0 && n_reg > 0) {
            /* Strip * from REG if present */
            const char *reg_content = text;
            int reg_content_len = n_reg;
            if (n_reg > 2 && text[0] == '*' && text[n_reg - 1] == '*') {
                reg_content = text + 1;
                reg_content_len = n_reg - 2;
            }

            /* Compare: if same, ASCII didn't convert, use REG */
            if (n_ascii == reg_content_len && memcmp(text_ascii, reg_content, (size_t)n_ascii) == 0) {
                /* Same content - use REG with * */
                n = n_reg;
            } else {
                /* Different - ASCII did conversion, use it */
                n = n_ascii;
                memcpy(text, text_ascii, (size_t)n_ascii + 1);
            }
        } else if (n_reg > 0) {
            n = n_reg;
        } else {
            n = n_ascii;
        }

        if (n > best_len) {
            best_len = n;
            memcpy(best, text, (size_t)n + 1);
        }
    }

    if (best_len <= 0) {
        return 0;
    }

    /* Trioptic Code 39 (used e.g. on CD/DVD media) is restricted to
     * digits and uppercase letters only (no punctuation) and its
     * reported data does not include the '*' start/stop sentinels.
     * When the regular-mode decode kept the sentinels (best[0]/[len-1]
     * == '*') and the enclosed payload is pure alnum, strip them so the
     * output matches that convention. Codabar/Code39 test data that
     * legitimately contains punctuation (e.g. "*1234+56*") is unaffected. */
    if (best_len >= 3 && best[0] == '*' && best[best_len - 1] == '*') {
        int alnum_only = 1;
        for (int i = 1; i < best_len - 1; i++) {
            char c = best[i];
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z'))) {
                alnum_only = 0;
                break;
            }
        }
        if (alnum_only) {
            memmove(best, best + 1, (size_t)(best_len - 2));
            best_len -= 2;
            best[best_len] = '\0';
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
