/*
 * RSS/GS1 DataBar decoder restoration.
 *
 * Reverse-engineered anchors:
 * - ClearRss()                    line 220954
 * - SmartSymbolRedef0006016295B() line 221033
 * - fmwlsymbol005f()              line 221053 (RSS-14)
 * - fmwlsymbol0061()              line 222931 (RSS Limited)
 * - fmwlsymbol0062()              line 223600 (RSS Expanded)
 *
 * Important behavior from fmwlsymbol005f:
 * - payload becomes "01" + 13 digits + GS1 Mod10 check digit.
 * - final check uses weighted sum (3/1 alternating) equivalent to GTIN Mod10.
 */

#include "rss_gs1.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include <unistd.h>

typedef struct {
    int vote[6];             /* maps +20828..+20848 confidence counters */
    int left_part[6];        /* maps +20788..+20824 decoded partials */
    int right_part[6];       /* maps +20860..+20900 decoded partials */
    int left_ready;          /* +20852 */
    int right_ready;         /* +20892 */
    int next_left_scan;      /* +20856 */
    int next_right_scan;     /* +20896 / +21096 (other variants) */
} RssState;

typedef struct {
    int value;
    int checksum_portion;
    int modules[8];
} RssDataChar;

typedef struct {
    int value;
    double err;
} FinderMatch;

static void clear_rss_state(RssState *s)
{
    /* Mirrors ClearRss() defaults (line 220954). */
    memset(s, 0, sizeof(*s));
    for (int i = 0; i < 6; i++) {
        s->vote[i] = 1;
        s->left_part[i] = -1;
        s->right_part[i] = -1;
    }
}

static void reset_scan_state(RssState *s)
{
    /* Mirrors SmartSymbolRedef0006016295B() relevant reset points. */
    s->next_left_scan = 0;
    s->next_right_scan = 0;
    s->left_ready = 0;
}

static int gs1_mod10_digit(const char *digits, int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++) {
        int d = digits[i] - '0';
        sum += d * ((i & 1) ? 1 : 3);
    }
    return (10 - (sum % 10)) % 10;
}

static void copy_d13(const char *d13, char *out)
{
    /* Output as AI "01" + 14-digit GTIN (13 digits + mod10 check), no separators. */
    out[0] = '0';
    out[1] = '1';
    memcpy(out + 2, d13, 13);
    out[15] = (char)('0' + gs1_mod10_digit(d13, 13));
    out[16] = '\0';
}

static int choose_int(int n, int r)
{
    if (r < 0 || r > n) return 0;
    if (r == 0 || r == n) return 1;
    if (r > n - r) r = n - r;
    int64_t acc = 1;
    for (int i = 1; i <= r; i++) {
        acc = (acc * (n - r + i)) / i;
    }
    return (int)acc;
}

/*
 * Equivalent combinatorial ranker:
 * sub_1387B8() @ libFsaDecoderCore1.c:228243
 */
static int rss_rank_value(const int *widths, int n, int max_width, int no_narrow)
{
    int val = 0;
    int narrow_mask = 0;
    int elements = 0;

    for (int i = 0; i < n; i++) {
        if (widths[i] > max_width || widths[i] < 1) return -1;
        elements += widths[i];
    }

    for (int bar = 0; bar < n - 1; bar++) {
        narrow_mask |= 1 << bar;
        for (int elm = 1; elm < widths[bar]; elm++) {
            int sub_val = choose_int(elements - elm - 1, n - bar - 2);
            if (no_narrow && narrow_mask == 0) {
                int rem = elements - elm - (n - bar - 1);
                if (rem >= n - bar - 1)
                    sub_val -= choose_int(elements - elm - (n - bar), n - bar - 2);
            }
            if (n - bar - 1 > 1) {
                int less_val = 0;
                for (int mxw = elements - elm - (n - bar - 2); mxw > max_width; mxw--) {
                    less_val += choose_int(elements - elm - mxw - 1, n - bar - 3);
                }
                sub_val -= less_val * (n - 1 - bar);
            } else if (elements - elm > max_width) {
                sub_val--;
            }
            val += sub_val;
            narrow_mask &= ~(1 << bar);
        }
        elements -= widths[bar];
    }
    return val;
}

/* Finder patterns used by RSS-14 pair check value (fmwlsymbol0060 path). */
static const int RSS14_FINDER[9][4] = {
    {3, 8, 2, 1}, {3, 5, 5, 1}, {3, 3, 7, 1},
    {3, 1, 9, 1}, {2, 7, 4, 1}, {2, 5, 6, 1},
    {2, 3, 8, 1}, {1, 5, 7, 1}, {1, 3, 9, 1},
};

/*
 * Constants from decompiled tables:
 * - word_1A0DEA / word_1A0DF4 / word_1A0DFE / word_1A0E08 @ ~7961
 * - dword_1311A4 / dword_1311AC / dword_1311B4 / dword_1311BC @ ~3726
 */
static const int OUTSIDE_ODD_WIDEST[5] = {8, 6, 4, 3, 1};
static const int OUTSIDE_EVEN_WIDEST[5] = {1, 3, 5, 6, 8};
static const int OUTSIDE_GSUM[5] = {0, 161, 961, 2015, 2715};
static const int OUTSIDE_T[5] = {1, 10, 34, 70, 126};

static const int INSIDE_ODD_WIDEST[4] = {7, 5, 3, 1};
static const int INSIDE_EVEN_WIDEST[4] = {2, 4, 6, 8};
static const int INSIDE_T[4] = {4, 20, 48, 81};
static const int INSIDE_GSUM[4] = {0, 336, 1036, 1516};

/* fmwlsymbol0060 checksum branches:
 * - line ~221872: store +20804 (for +20788)
 * - line ~222290: store +20812 (for +20796)
 * - line ~221832 / ~222511: store +20808 (for +20792)
 * - line ~221913 / ~222554: store +20816 (for +20800)
 */
static const int RSS14_CHK_W[4][8] = {
    {54, 18, 6, 2, 27, 9, 3, 1},    /* +20804 */
    {58, 72, 24, 8, 29, 36, 12, 4}, /* +20808 */
    {74, 51, 17, 32, 37, 65, 48, 16}, /* +20812 */
    {59, 46, 68, 49, 69, 23, 34, 64}, /* +20816 */
};

/* fmwlsymbol0061 / SmartSymbolRedef0006016294B constants. */
static const int RSS_LIMITED_ODD_WIDEST[7] = {1, 3, 4, 5, 5, 6, 8};
static const int RSS_LIMITED_EVEN_WIDEST[7] = {8, 6, 5, 4, 4, 3, 1};
static const int RSS_LIMITED_GSUM[7] = {1996939, 820064, 1491021, 183064, 1000776, 0, 1979845};
static const int RSS_LIMITED_T[7] = {16632, 6454, 2408, 728, 203, 28, 1};
static const uint8_t RSS_LIMITED_FINDER_MAP[441] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
    42, 43, 255, 44, 255, 255, 255, 255, 255, 255, 45, 255, 255, 255, 255, 46, 255, 255, 255, 255, 255,
    47, 48, 49, 50, 255, 255, 255, 255, 255, 255, 51, 52, 53, 54, 55, 56, 57, 255, 255, 58, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    59, 60, 61, 62, 63, 255, 64, 255, 255, 255, 255, 255, 255, 255, 255, 65, 66, 67, 68, 69, 70,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    71, 72, 73, 74, 75, 76, 77, 78, 255, 255, 79, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 80, 81, 82, 83, 84, 255, 85, 86, 255, 255, 87, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 88, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255
};

static void reverse8(const int in[8], int out[8])
{
    for (int i = 0; i < 8; i++) out[i] = in[7 - i];
}

static int quantize8(const int w[8], int target_modules, int m[8])
{
    int total = 0;
    for (int i = 0; i < 8; i++) {
        if (w[i] <= 0) return 0;
        total += w[i];
    }
    if (total <= 0) return 0;

    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int q = (w[i] * target_modules + total / 2) / total;
        if (q < 1) q = 1;
        if (q > 8) q = 8;
        m[i] = q;
        sum += q;
    }

    while (sum > target_modules) {
        int k = -1;
        for (int i = 0; i < 8; i++) {
            if (m[i] <= 1) continue;
            if (k < 0 || m[i] > m[k]) k = i;
        }
        if (k < 0) break;
        m[k]--;
        sum--;
    }
    while (sum < target_modules) {
        int k = -1;
        for (int i = 0; i < 8; i++) {
            if (k < 0 || m[i] < m[k]) k = i;
        }
        if (k < 0) break;
        m[k]++;
        sum++;
    }
    return sum == target_modules;
}

static int decode_data_char(const int widths[8], int outside, int reverse, RssDataChar *out)
{
    int seq[8];
    if (reverse) reverse8(widths, seq);
    else memcpy(seq, widths, sizeof(seq));

    int target = outside ? 16 : 15;
    int m[8];
    if (!quantize8(seq, target, m)) return 0;

    /* Try both odd/even alignments to tolerate threshold phase drift. */
    for (int phase = 0; phase < 2; phase++) {
        int odd[4], even[4];
        int os = 0, es = 0;
        for (int i = 0; i < 4; i++) {
            odd[i] = m[phase + 2 * i];
            even[i] = m[(1 - phase) + 2 * i];
            os += odd[i];
            es += even[i];
        }

        if (outside) {
            /* fmwlsymbol0060 branch around 221983..222004 */
            if (os < 4 || os > 12 || (os & 1) != 0) continue;
            if (es != 16 - os) continue;
            int grp = (12 - os) >> 1;
            if (grp < 0 || grp >= 5) continue;
            int v_odd = rss_rank_value(odd, 4, OUTSIDE_ODD_WIDEST[grp], 0);
            int v_even = rss_rank_value(even, 4, OUTSIDE_EVEN_WIDEST[grp], 1);
            if (v_odd < 0 || v_even < 0) continue;
            out->value = v_even + OUTSIDE_GSUM[grp] + v_odd * OUTSIDE_T[grp];
        } else {
            /* fmwlsymbol0060 branch around 221792..221799 (inside tables). */
            if (os < 4 || os > 10 || (os & 1) != 0) continue;
            if (es != 15 - os) continue;
            int grp = (10 - os) >> 1;
            if (grp < 0 || grp >= 4) continue;
            int v_odd = rss_rank_value(odd, 4, INSIDE_ODD_WIDEST[grp], 0);
            int v_even = rss_rank_value(even, 4, INSIDE_EVEN_WIDEST[grp], 1);
            if (v_odd < 0 || v_even < 0) continue;
            out->value = v_even + INSIDE_GSUM[grp] + v_odd * INSIDE_T[grp];
        }

        /*
         * Checksum portion used by fmwlsymbol005f() mod-79 gate.
         * Equivalent structure to data-character checksum composition in fmwlsymbol0060().
         */
        int odd_chk = 0;
        int even_chk = 0;
        for (int i = 3; i >= 0; i--) odd_chk = odd_chk * 9 + odd[i];
        for (int i = 3; i >= 0; i--) even_chk = even_chk * 9 + even[i];
        out->checksum_portion = (odd_chk + 3 * even_chk) % 79;
        memcpy(out->modules, m, sizeof(out->modules));
        return 1;
    }
    return 0;
}

static int checksum_weighted(const int modules[8], const int weights[8])
{
    int s = 0;
    for (int i = 0; i < 8; i++) s += modules[i] * weights[i];
    return s % 79;
}

static int checksum_match_mod79(const RssDataChar *a, const RssDataChar *b, const RssDataChar *c, const RssDataChar *d, int target)
{
    /*
     * Single fixed orientation (all four characters re-reversed relative to
     * their decode-time module order) - a real 1/79 hard gate rather than the
     * former 16-combination search that let ~20% of random rows through.
     */
    int ar[8], br[8], cr[8], dr[8];
    reverse8(a->modules, ar);
    reverse8(b->modules, br);
    reverse8(c->modules, cr);
    reverse8(d->modules, dr);

    int chk = 0;
    chk += checksum_weighted(ar, RSS14_CHK_W[0]); /* +20804 */
    chk += checksum_weighted(br, RSS14_CHK_W[1]); /* +20808 */
    chk += checksum_weighted(dr, RSS14_CHK_W[2]); /* +20812 */
    chk += checksum_weighted(cr, RSS14_CHK_W[3]); /* +20816 */
    chk %= 79;
    if (chk > 7) chk++;
    if (chk > 71) chk++;
    return chk == target;
}

static FinderMatch match_finder4(const int w4[4])
{
    FinderMatch best = {-1, 1e9};
    int rev[4] = {w4[3], w4[2], w4[1], w4[0]};
    int sum = w4[0] + w4[1] + w4[2] + w4[3];
    if (sum <= 0 || rev[0] + rev[1] + rev[2] + rev[3] <= 0) return best;
    for (int i = 0; i < 9; i++) {
        int psum = RSS14_FINDER[i][0] + RSS14_FINDER[i][1] + RSS14_FINDER[i][2] + RSS14_FINDER[i][3];
        double scale = (double)sum / (double)psum;
        for (int orient = 0; orient < 2; orient++) {
            const int *src = orient ? rev : w4;
            double err = 0.0;
            for (int k = 0; k < 4; k++) {
                double e = fabs((double)src[k] - scale * (double)RSS14_FINDER[i][k]);
                err += e;
            }
            err /= (double)sum;
            if (err < best.err) {
                best.err = err;
                best.value = i;
            }
        }
    }
    return best;
}

static int finder_ratio_19_24_ok(const int runs[], int n, int idx4)
{
    for (int shift = -1; shift <= 1; shift++) {
        int i = idx4 + shift;
        if (i < 1 || i + 3 >= n) continue;
        int w0 = runs[i - 1];
        int w1 = runs[i];
        int w2 = runs[i + 1];
        int w3 = runs[i + 2];
        int w4 = runs[i + 3];
        int span4_a = w1 + w2 + w3 + w4;
        int span2_a = w2 + w3;
        int span4_b = w0 + w1 + w2 + w3;
        int span2_b = w1 + w2;
        if (19 * span4_a >= 24 * span2_a && 19 * span4_b >= 24 * span2_b) return 1;
    }
    return 0;
}

static int finder_tolerance_25_ok(const int w4[4], int finder_value, double *err_out)
{
    if (finder_value < 0 || finder_value > 8) return 0;
    int psum = RSS14_FINDER[finder_value][0] + RSS14_FINDER[finder_value][1]
             + RSS14_FINDER[finder_value][2] + RSS14_FINDER[finder_value][3];
    if (psum <= 0) return 0;

    int rev[4] = {w4[3], w4[2], w4[1], w4[0]};
    double best = 1e9;
    for (int orient = 0; orient < 2; orient++) {
        const int *src = orient ? rev : w4;
        int sum = src[0] + src[1] + src[2] + src[3];
        if (sum <= 0) continue;
        double scale = (double)sum / (double)psum;
        double err = 0.0;
        int ok = 1;
        for (int i = 0; i < 4; i++) {
            double exp_w = scale * (double)RSS14_FINDER[finder_value][i];
            double delta = fabs((double)src[i] - exp_w);
            if (delta > 0.35 * exp_w) {
                ok = 0;
                break;
            }
            err += delta;
        }
        if (ok) {
            err /= (double)sum;
            if (err < best) best = err;
        }
    }
    if (best >= 1e8) return 0;
    if (err_out) *err_out = best;
    return 1;
}

static int finder_accept(const int runs[], int n, int idx4, const FinderMatch *m, double *guard_err)
{
    if (!m || m->value < 0 || m->err > 0.34) return 0;
    if (idx4 < 0 || idx4 + 3 >= n) return 0;
    if (!finder_ratio_19_24_ok(runs, n, idx4)) return 0;
    int w4[4] = {runs[idx4], runs[idx4 + 1], runs[idx4 + 2], runs[idx4 + 3]};
    if (!finder_tolerance_25_ok(w4, m->value, guard_err)) return 0;
    return 1;
}

static uint64_t rss14_symbol14(const RssDataChar *d0, const RssDataChar *d1, const RssDataChar *d2, const RssDataChar *d3)
{
    uint64_t left_pair = (uint64_t)d0->value * 1597ULL + (uint64_t)d1->value;
    uint64_t right_pair = (uint64_t)d3->value * 1597ULL + (uint64_t)d2->value;
    return left_pair * 4537077ULL + right_pair;
}

static void build_rss14_d13_from_chars(const RssDataChar *d0, const RssDataChar *d1, const RssDataChar *d2, const RssDataChar *d3, char d13[14])
{
    /*
     * fmwlsymbol005f line 221180..221209:
     * D0 = ((d0*1597 + d1) * 4537077) + (d2*1597 + d3)
     * sprintf("%014.0f", D0) then copy bytes v77..v89 (drop the first digit).
     */
    uint64_t symbol14 = rss14_symbol14(d0, d1, d2, d3);
    char s14[32];
    snprintf(s14, sizeof(s14), "%014llu", (unsigned long long)symbol14);
    memcpy(d13, s14 + 1, 13);
    d13[13] = '\0';
}

typedef struct {
    int idx;
    FinderMatch fm;
    double guard_err;
} RssFinderCand;


static int decode_row_rss14(const uint8_t *row, int width, BarcodeResult *result)
{
    int minv = 255, maxv = 0;
    for (int x = 0; x < width; x++) {
        if (row[x] < minv) minv = row[x];
        if (row[x] > maxv) maxv = row[x];
    }
    if (maxv - minv < 8) return 0;
    int th = (minv + maxv) / 2;

    int rw[512], rc[512], nr = 0;
    int cur = row[0] < th ? 1 : 0;
    int cnt = 1;
    for (int x = 1; x < width; x++) {
        int b = row[x] < th ? 1 : 0;
        if (b == cur) {
            cnt++;
        } else {
            if (nr >= 512) return 0;
            rc[nr] = cur;
            rw[nr] = cnt;
            nr++;
            cur = b;
            cnt = 1;
        }
    }
    if (nr >= 512) return 0;
    rc[nr] = cur;
    rw[nr] = cnt;
    nr++;

    /* Drop quiet zones, align to bar first (black=1). */
    int s = 0, e = nr - 1;
    if (nr < 20) return 0;
    while (s <= e && rc[s] == 0) s++;
    while (s <= e && rc[e] == 0) e--;
    if (e - s + 1 < 40) return 0;

    int n = e - s + 1;
    int runs[256];
    if (n > 256) return 0;
    for (int i = 0; i < n; i++) runs[i] = rw[s + i];

    /*
     * Two-stage search: first find all plausible finder positions cheaply
     * (O(n)), then only run the expensive 4-character data decode for
     * finder PAIRS in ascending combined-error order.  This keeps the
     * per-row cost bounded (a handful of finder candidates rather than the
     * full O(n^2) position grid) and makes the genuinely best-matching pair
     * (almost always the true finder pair) get tried first, so the mod-79
     * checksum gate is evaluated against very few spurious candidates.
     */
    RssFinderCand cands[64];
    int ncand = 0;
    for (int idx = 8; idx + 12 < n && ncand < 64; idx++) {
        int f4[4] = {runs[idx], runs[idx + 1], runs[idx + 2], runs[idx + 3]};
        FinderMatch m = match_finder4(f4);
        double guard_err = 0.0;
        if (!finder_accept(runs, n, idx, &m, &guard_err)) continue;
        cands[ncand].idx = idx;
        cands[ncand].fm = m;
        cands[ncand].guard_err = guard_err;
        ncand++;
    }
    if (ncand < 2) return 0;

    typedef struct {
        int li, ri;
        double err;
    } RssPairCand;
    RssPairCand pairs[512];
    int npairs = 0;
    for (int i = 0; i < ncand && npairs < 512; i++) {
        for (int j = i + 1; j < ncand && npairs < 512; j++) {
            /*
             * Canonical RSS-14 layout: between the two finder windows lie
             * exactly the 5 finder runs + inner char (8) + inner char (8),
             * i.e. right finder window always starts 22 runs after the left
             * one.  Enforcing the exact structural distance removes nearly
             * all spurious finder pairs (e.g. MicroPDF417 rows).
             */
            if (cands[j].idx != cands[i].idx + 22) continue;
            pairs[npairs].li = i;
            pairs[npairs].ri = j;
            pairs[npairs].err = cands[i].fm.err + cands[j].fm.err
                               + cands[i].guard_err + cands[j].guard_err;
            npairs++;
        }
    }
    /* Simple insertion sort - npairs is small in practice. */
    for (int i = 1; i < npairs; i++) {
        RssPairCand key = pairs[i];
        int j = i - 1;
        while (j >= 0 && pairs[j].err > key.err) {
            pairs[j + 1] = pairs[j];
            j--;
        }
        pairs[j + 1] = key;
    }

    int max_pairs_tried = npairs < 24 ? npairs : 24;
    for (int p = 0; p < max_pairs_tried; p++) {
        const RssFinderCand *Lc = &cands[pairs[p].li];
        const RssFinderCand *Rc = &cands[pairs[p].ri];
        int lf = Lc->idx, rf = Rc->idx;
        FinderMatch L = Lc->fm, R = Rc->fm;

        /*
         * Canonical run windows (verified against real symbol geometry):
         *   left outer  = lf - 8 .. lf - 1   (before left finder window)
         *   left inner  = lf + 5 .. lf + 12  (after the 5 finder runs)
         *   right inner = rf - 9 .. rf - 2
         *   right outer = rf + 4 .. rf + 11
         */
        int slo = lf - 8;
        int sli = lf + 5;
        int sri = rf - 9;
        int sro = rf + 4;
        if (slo < 0 || sro + 8 > n) continue;

        int l_out[8], l_in[8], r_in[8], r_out[8];
        for (int i = 0; i < 8; i++) {
            l_out[i] = runs[slo + i];
            l_in[i] = runs[sli + i];
            r_in[i] = runs[sri + i];
            r_out[i] = runs[sro + i];
        }
        RssDataChar a, b, c, d;
        if (!decode_data_char(l_out, 1, 0, &a)) continue;
        if (!decode_data_char(l_in, 0, 1, &b)) continue;
        if (!decode_data_char(r_in, 0, 0, &c)) continue;
        if (!decode_data_char(r_out, 1, 1, &d)) continue;

        /*
         * fmwlsymbol005f @ 221108..221126:
         * check = (sum checksum portions) mod 79 with >7 and
         * >71 skip. target = 9 * leftFinder + rightFinder.
         */
        int target = 9 * L.value + R.value;
        if (!checksum_match_mod79(&a, &b, &c, &d, target)) continue;

        char d13[14];
        build_rss14_d13_from_chars(&a, &b, &c, &d, d13);
        if (getenv("RSS_DEBUG")) {
            fprintf(stderr,
                "[rss-linear] n=%d lf=%d rf=%d L=%d R=%d d13=%s\n",
                n, lf, rf, L.value, R.value, d13);
        }
        memset(result, 0, sizeof(*result));
        copy_d13(d13, result->data);
        result->length = 16;
        result->confidence = 88;
        return 1;
    }
    return 0;
}

static int try_rss14_algorithm(const Image *img, BarcodeResult *result)
{
    BarcodeResult cands[16];
    int votes[16];
    int n = 0;
    memset(cands, 0, sizeof(cands));
    memset(votes, 0, sizeof(votes));

    int max_rows = img->height < 192 ? img->height : 192;
    int chosen[192];
    int chosen_n = 0;
    int yc = img->height / 2;

    for (int pass = 0; pass < 2 && chosen_n < max_rows; pass++) {
        for (int y = 0; y < img->height && chosen_n < max_rows; y++) {
            const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
            int minv = 255, maxv = 0;
            for (int x = 0; x < img->width; x++) {
                int v = row[x];
                if (v < minv) minv = v;
                if (v > maxv) maxv = v;
            }
            int contrast = maxv - minv;
            int prefer = abs(y - yc) < img->height / 3;
            if ((pass == 0 && contrast >= 24 && prefer) || (pass == 1 && contrast >= 16)) {
                int seen = 0;
                for (int i = 0; i < chosen_n; i++) {
                    if (chosen[i] == y) {
                        seen = 1;
                        break;
                    }
                }
                if (!seen) chosen[chosen_n++] = y;
            }
        }
    }

    for (int i = 0; i < chosen_n; i++) {
        int y = chosen[i];
        const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
        BarcodeResult tmp;
        memset(&tmp, 0, sizeof(tmp));
        int rr = decode_row_rss14(row, img->width, &tmp);
        if (rr) {
            int k;
            for (k = 0; k < n; k++) {
                if (strcmp(cands[k].data, tmp.data) == 0) {
                    votes[k]++;
                    break;
                }
            }
            if (k == n && n < 16) {
                cands[n] = tmp;
                votes[n] = 1;
                n++;
                k = n - 1;
            }
            /*
             * A handful of rows can occasionally quantize a boundary module
             * width off by one and still pass the mod-79 gate (the gate is
             * 1/79 per candidate, not per row), so require a real majority
             * across many rows rather than stopping at the first two
             * agreeing rows - a genuine linear barcode decodes identically
             * on the large majority of its scan lines.
             */
            if (votes[k] >= 6) break;
        }
    }

    int best = -1, best_vote = 0;
    for (int i = 0; i < n; i++) {
        if (votes[i] > best_vote) {
            best_vote = votes[i];
            best = i;
        }
    }
    if (best >= 0 && best_vote >= 3) {
        *result = cands[best];
        return 1;
    }
    return 0;
}

/*
 * RSS-14 Stacked: decode a single pair (left or right) from a row.
 * Returns 1 if a valid pair is found.
 * out_outer/out_inner: decoded data characters around the finder.
 * out_finder: matched finder pattern index.
 */
typedef struct {
    RssDataChar outer;
    RssDataChar inner;
    int finder;
} Rss14HalfResult;

typedef struct {
    Rss14HalfResult pair;
    int votes;
} Rss14PairVote;

enum { RSS14_POOL_MAX = 48 };

typedef struct {
    Rss14PairVote e[RSS14_POOL_MAX];
    int n;
    int cursor; /* round-robin eviction position */
} Rss14Pool;

static void rss14_half_vote(Rss14Pool *pool, const Rss14HalfResult *cand)
{
    for (int k = 0; k < pool->n; k++) {
        if (pool->e[k].pair.outer.value == cand->outer.value &&
            pool->e[k].pair.inner.value == cand->inner.value &&
            pool->e[k].pair.finder == cand->finder) {
            pool->e[k].votes++;
            return;
        }
    }
    if (pool->n < RSS14_POOL_MAX) {
        pool->e[pool->n].pair = *cand;
        pool->e[pool->n].votes = 1;
        pool->n++;
        return;
    }
    /*
     * Pool full: evict a single-vote entry in round-robin order.  First-fit
     * eviction would thrash a fresh (true) candidate right back out before
     * the next scan row could re-vote it; the rotating cursor guarantees a
     * newly inserted entry survives ~RSS14_POOL_MAX insertions, which is
     * far more than one row can generate, so persistent candidates always
     * reach votes >= 2 and become protected.
     */
    for (int t = 0; t < RSS14_POOL_MAX; t++) {
        int k = (pool->cursor + t) % RSS14_POOL_MAX;
        if (pool->e[k].votes <= 1) {
            pool->e[k].pair = *cand;
            pool->e[k].votes = 1;
            pool->cursor = (k + 1) % RSS14_POOL_MAX;
            return;
        }
    }
}

/*
 * Collect all structurally-canonical half-pair readings from one scan row.
 * Left-pair reading:  outer char at f-8 (fwd), inner char at f+5 (rev).
 * Right-pair reading: inner char at f-9 (fwd), outer char at f+4 (rev).
 * (Same canonical windows as the verified linear RSS-14 layout.)
 */
static void decode_row_rss14_half_multi(
    const uint8_t *row, int width,
    Rss14Pool *left_pool, Rss14Pool *right_pool)
{
    int minv = 255, maxv = 0;
    for (int x = 0; x < width; x++) {
        if (row[x] < minv) minv = row[x];
        if (row[x] > maxv) maxv = row[x];
    }
    if (maxv - minv < 16) return;
    int th = (minv + maxv) / 2;

    int rw[512], rc[512], nr = 0;
    int cur = row[0] < th ? 1 : 0;
    int cnt = 1;
    for (int x = 1; x < width; x++) {
        int b = row[x] < th ? 1 : 0;
        if (b == cur) {
            cnt++;
        } else {
            if (nr >= 512) return;
            rc[nr] = cur;
            rw[nr] = cnt;
            nr++;
            cur = b;
            cnt = 1;
        }
    }
    if (nr >= 512) return;
    rc[nr] = cur;
    rw[nr] = cnt;
    nr++;

    int s = 0, e = nr - 1;
    if (nr < 10) return;
    while (s <= e && rc[s] == 0) s++;
    while (s <= e && rc[e] == 0) e--;
    if (e - s + 1 < 20) return;

    int n = e - s + 1;
    int runs[256];
    if (n > 256) return;
    /*
     * A DataBar Stacked half-row consists of nothing but the half symbol:
     * guard + data char (8) + finder (5) + data char (8) + guard, i.e.
     * ~23..25 runs after quiet-zone trimming.  Rows carrying additional
     * structure (MicroPDF417 rows, text lines, ...) are much longer and are
     * the main source of spurious pairs that survive the mod-79 gate.
     */
    if (n > 32) return;
    for (int i = 0; i < n; i++) runs[i] = rw[s + i];

    /* Scan the run sequence forward and mirrored (stacked bottom rows may
     * be oriented either way relative to the top row). */
    for (int dir = 0; dir < 2; dir++) {
        int seq[256];
        if (dir == 0) {
            memcpy(seq, runs, sizeof(int) * (size_t)n);
        } else {
            for (int i = 0; i < n; i++) seq[i] = runs[n - 1 - i];
        }

        for (int f = 4; f + 4 < n; f++) {
            int f4[4] = {seq[f], seq[f + 1], seq[f + 2], seq[f + 3]};
            FinderMatch fm = match_finder4(f4);
            double guard_err = 0.0;
            if (!finder_accept(seq, n, f, &fm, &guard_err)) continue;

            /* Left-pair interpretation. */
            if (f - 8 >= 0 && f + 13 <= n) {
                int o8[8], i8[8];
                for (int i = 0; i < 8; i++) {
                    o8[i] = seq[f - 8 + i];
                    i8[i] = seq[f + 5 + i];
                }
                Rss14HalfResult hr;
                if (decode_data_char(o8, 1, 0, &hr.outer) &&
                    decode_data_char(i8, 0, 1, &hr.inner)) {
                    hr.finder = fm.value;
                    rss14_half_vote(left_pool, &hr);
                }
            }
            /* Right-pair interpretation. */
            if (f - 9 >= 0 && f + 12 <= n) {
                int i8[8], o8[8];
                for (int i = 0; i < 8; i++) {
                    i8[i] = seq[f - 9 + i];
                    o8[i] = seq[f + 4 + i];
                }
                Rss14HalfResult hr;
                if (decode_data_char(i8, 0, 0, &hr.inner) &&
                    decode_data_char(o8, 1, 1, &hr.outer)) {
                    hr.finder = fm.value;
                    rss14_half_vote(right_pool, &hr);
                }
            }
        }
    }
}

/*
 * RSS-14 Stacked algorithm: scan top half for left pair, bottom half for
 * right pair, combine with mod-79 checksum validation.
 * Uses voting to avoid false positives.
 */
static int try_rss14_stacked_algorithm(const Image *img, BarcodeResult *result)
{
    if (img->height < 40) return 0;

    int mid_y = img->height / 2;

    Rss14Pool top_left, top_right, bot_left, bot_right;
    memset(&top_left, 0, sizeof(top_left));
    memset(&top_right, 0, sizeof(top_right));
    memset(&bot_left, 0, sizeof(bot_left));
    memset(&bot_right, 0, sizeof(bot_right));

    for (int y = 0; y < img->height; y++) {
        const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
        if (y < mid_y) {
            decode_row_rss14_half_multi(row, img->width, &top_left, &top_right);
        } else {
            decode_row_rss14_half_multi(row, img->width, &bot_left, &bot_right);
        }
    }

    if (getenv("RSS_DEBUG")) {
        fprintf(stderr, "[rss-stacked] img=%dx%d pools tl=%d tr=%d bl=%d br=%d mid_y=%d\n",
                img->width, img->height, top_left.n, top_right.n, bot_left.n, bot_right.n, mid_y);
        const Rss14Pool *pp[4] = {&top_left, &top_right, &bot_left, &bot_right};
        const char *pn[4] = {"tl", "tr", "bl", "br"};
        for (int p = 0; p < 4; p++)
            for (int i = 0; i < pp[p]->n; i++)
                if (pp[p]->e[i].votes >= 2)
                    fprintf(stderr, "  %s[%d]: o=%d i=%d f=%d v=%d\n", pn[p], i,
                            pp[p]->e[i].pair.outer.value, pp[p]->e[i].pair.inner.value,
                            pp[p]->e[i].pair.finder, pp[p]->e[i].votes);
    }

    /* A real stacked row decodes identically on several scan lines; junk
     * candidates from text/noise rows rarely repeat.  Combined with the
     * mod-79 cross-pair gate a small fixed threshold is enough. */
    int min_half_votes = 3;

    int best_votes = 0;
    BarcodeResult best_res;
    memset(&best_res, 0, sizeof(best_res));

    /* Combination A: top row holds the left pair, bottom the right pair.
     * Combination B: the reverse (symbol scanned upside-down). */
    for (int combo = 0; combo < 2; combo++) {
        Rss14PairVote *lp = combo == 0 ? top_left.e : bot_left.e;
        int n_lp = combo == 0 ? top_left.n : bot_left.n;
        Rss14PairVote *rp = combo == 0 ? bot_right.e : top_right.e;
        int n_rp = combo == 0 ? bot_right.n : top_right.n;

        for (int t = 0; t < n_lp; t++) {
            if (lp[t].votes < min_half_votes) continue;
            for (int b = 0; b < n_rp; b++) {
                if (rp[b].votes < min_half_votes) continue;
                int total_votes = lp[t].votes + rp[b].votes;
                if (total_votes <= best_votes) continue;

                RssDataChar *a = &lp[t].pair.outer;
                RssDataChar *bdc = &lp[t].pair.inner;
                RssDataChar *c = &rp[b].pair.inner;
                RssDataChar *d = &rp[b].pair.outer;
                int target = 9 * lp[t].pair.finder + rp[b].pair.finder;
                if (!checksum_match_mod79(a, bdc, c, d, target)) continue;

                char d13[14];
                build_rss14_d13_from_chars(a, bdc, c, d, d13);
                memset(&best_res, 0, sizeof(best_res));
                copy_d13(d13, best_res.data);
                best_res.length = 16;
                best_res.confidence = 88;
                best_votes = total_votes;
            }
        }
    }

    if (best_votes >= 2 * min_half_votes) {
        *result = best_res;
        return 1;
    }
    return 0;
}

static int mod89(int v)
{
    int r = v % 89;
    if (r < 0) r += 89;
    return r;
}

static int tmod_edges(const int *edges, int n_edges, int idx, int span, int mod)
{
    if (!edges || idx < 0 || idx + 2 >= n_edges || span <= 0 || mod <= 0) return -1;
    int num = (edges[idx + 2] - edges[idx]) * mod + span / 2;
    return num / span;
}

typedef struct {
    int value;
    int checksum;
    int finder;
} RssLimitedPart;

/*
 * Shared 6-element finder rank + lookup, used identically by both
 * fmwlsymbol0061 branches: d[11]..d[0] hold the 12 telescoped diffs (d[11]
 * being the earliest/most significant), the finder rank windows sample the
 * array with a stride of 2 (sub_1387B8 reads every other stack slot).
 */
static int rss_limited_finder_lookup(const int d[12])
{
    int w6a[6] = {d[11], d[9], d[7], d[5], d[3], d[1]};
    int w6b[6] = {d[10], d[8], d[6], d[4], d[2], d[0]};
    int f1 = rss_rank_value(w6a, 6, 6, 1);
    int f2 = rss_rank_value(w6b, 6, 6, 1);
    if (f1 < 0 || f2 < 0 || f1 > 20 || f2 > 20) return -1;
    int finder = RSS_LIMITED_FINDER_MAP[21 * f1 + f2];
    if (finder > 88) return -1;
    return finder;
}

/*
 * Mirrors fmwlsymbol0061 first branch (decompile @222931, ~223000-223350):
 * finder chain spans a2-2..a2-14 (span=v10, mod=18), data chain spans
 * a2-15..a2-28 (span=v9, mod=26) chained from the finder's last diff.
 */
static int rss_limited_decode_part_20864(const int *edges, int n_edges, int a2, RssLimitedPart *out)
{
    if (!edges || !out || a2 < 28 || a2 >= n_edges) return 0;

    int v9 = edges[a2 - 14] - edges[a2 - 28];
    int v10 = edges[a2] - edges[a2 - 14];
    if (v9 <= 0 || v10 <= 0) return 0;
    if (!(12 * v10 <= 9 * v9 && 9 * v9 <= 14 * v10)) return 0;

    if (tmod_edges(edges, n_edges, a2 - 2, v10, 18) != 2) return 0;

    int t1  = tmod_edges(edges, n_edges, a2 - 3,  v10, 18);
    int d1  = t1 - 1;
    int t2  = tmod_edges(edges, n_edges, a2 - 4,  v10, 18);
    int d2  = t2 - d1;
    int t3  = tmod_edges(edges, n_edges, a2 - 5,  v10, 18);
    int d3  = t3 - d2;
    int t4  = tmod_edges(edges, n_edges, a2 - 6,  v10, 18);
    int d4  = t4 - d3;
    int t5  = tmod_edges(edges, n_edges, a2 - 7,  v10, 18);
    int d5  = t5 - d4;
    int t6  = tmod_edges(edges, n_edges, a2 - 8,  v10, 18);
    int d6  = t6 - d5;
    int t7  = tmod_edges(edges, n_edges, a2 - 9,  v10, 18);
    int d7  = t7 - d6;
    int t8  = tmod_edges(edges, n_edges, a2 - 10, v10, 18);
    int d8  = t8 - d7;
    int t9  = tmod_edges(edges, n_edges, a2 - 11, v10, 18);
    int d9  = t9 - d8;
    int t10 = tmod_edges(edges, n_edges, a2 - 12, v10, 18);
    int d10 = t10 - d9;
    int t11 = tmod_edges(edges, n_edges, a2 - 13, v10, 18);
    int d11 = t11 - d10;
    int t12 = tmod_edges(edges, n_edges, a2 - 14, v10, 18);
    int d12 = t12 - d11;

    if (t12 + t10 + t8 + t6 + t4 + t2 != 16) return 0;
    if (d12 + d10 + d8 + d6 + d4 + d2 != 8) return 0;

    int fd[12] = {d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12};
    int finder = rss_limited_finder_lookup(fd);
    if (finder < 0) return 0;

    int u1  = tmod_edges(edges, n_edges, a2 - 15, v9, 26);
    int D1  = u1 - d12;
    int u2  = tmod_edges(edges, n_edges, a2 - 16, v9, 26);
    int D2  = u2 - D1;
    int u3  = tmod_edges(edges, n_edges, a2 - 17, v9, 26);
    int D3  = u3 - D2;
    int u4  = tmod_edges(edges, n_edges, a2 - 18, v9, 26);
    int D4  = u4 - D3;
    int u5  = tmod_edges(edges, n_edges, a2 - 19, v9, 26);
    int D5  = u5 - D4;
    int u6  = tmod_edges(edges, n_edges, a2 - 20, v9, 26);
    int D6  = u6 - D5;
    int u7  = tmod_edges(edges, n_edges, a2 - 21, v9, 26);
    int D7  = u7 - D6;
    int u8  = tmod_edges(edges, n_edges, a2 - 22, v9, 26);
    int D8  = u8 - D7;
    int u9  = tmod_edges(edges, n_edges, a2 - 23, v9, 26);
    int D9  = u9 - D8;
    int u10 = tmod_edges(edges, n_edges, a2 - 24, v9, 26);
    int D10 = u10 - D9;
    int u11 = tmod_edges(edges, n_edges, a2 - 25, v9, 26);
    int D11 = u11 - D10;
    int u12 = tmod_edges(edges, n_edges, a2 - 26, v9, 26);
    int D12 = u12 - D11;
    int u13 = tmod_edges(edges, n_edges, a2 - 27, v9, 26);
    int D13 = u13 - D12;
    int u14 = tmod_edges(edges, n_edges, a2 - 28, v9, 26);
    int D14 = u14 - D13;

    if (u14 + u12 + u10 + u8 + u6 + u4 + u2 != 26) return 0;
    int sum7 = D14 + D12 + D10 + D8 + D6 + D4 + D2;
    if ((sum7 & 1) == 0) return 0;
    int grp = (sum7 >> 1) - 3;
    if (grp < 0 || grp > 6) return 0;

    int d7a[7] = {D14, D12, D10, D8, D6, D4, D2};
    int d7b[7] = {D13, D11, D9, D7, D5, D3, D1};
    int r1 = rss_rank_value(d7a, 7, RSS_LIMITED_ODD_WIDEST[grp], 1);
    int r2 = rss_rank_value(d7b, 7, RSS_LIMITED_EVEN_WIDEST[grp], 0);
    if (r1 < 0 || r2 < 0) return 0;

    int value = RSS_LIMITED_GSUM[grp] + r2 + RSS_LIMITED_T[grp] * r1;
    int checksum = 66 * D1 + 22 * D2 + 37 * D3 + 42 * D4 + 14 * D5 + 64 * D6
                 + 51 * D7 + 17 * D8 + 65 * D9 + 81 * D10 + 27 * D11 + 9 * D12
                 + 3 * D13 + 1 * D14;

    out->value = value;
    out->checksum = mod89(checksum);
    out->finder = finder;
    return 1;
}

/*
 * Mirrors fmwlsymbol0061 second (else) branch (decompile @223365-223500):
 * finder chain spans a2-16..a2-28 (span=v9, mod=18), data chain spans
 * a2-15..a2-2 (span=v10, mod=26), independent D1 boundary (not chained
 * from the finder diffs, unlike the first branch).
 */
static int rss_limited_decode_part_20860(const int *edges, int n_edges, int a2, RssLimitedPart *out)
{
    if (!edges || !out || a2 < 28 || a2 >= n_edges) return 0;

    int v9 = edges[a2 - 14] - edges[a2 - 28];
    int v10 = edges[a2] - edges[a2 - 14];
    if (v9 <= 0 || v10 <= 0) return 0;
    if (!(12 * v9 <= 9 * v10 && 9 * v10 <= 14 * v9)) return 0;

    if (tmod_edges(edges, n_edges, a2 - 16, v9, 18) != 2) return 0;

    int t1  = tmod_edges(edges, n_edges, a2 - 17, v9, 18);
    int d1  = t1 - 1;
    int t2  = tmod_edges(edges, n_edges, a2 - 18, v9, 18);
    int d2  = t2 - d1;
    int t3  = tmod_edges(edges, n_edges, a2 - 19, v9, 18);
    int d3  = t3 - d2;
    int t4  = tmod_edges(edges, n_edges, a2 - 20, v9, 18);
    int d4  = t4 - d3;
    int t5  = tmod_edges(edges, n_edges, a2 - 21, v9, 18);
    int d5  = t5 - d4;
    int t6  = tmod_edges(edges, n_edges, a2 - 22, v9, 18);
    int d6  = t6 - d5;
    int t7  = tmod_edges(edges, n_edges, a2 - 23, v9, 18);
    int d7  = t7 - d6;
    int t8  = tmod_edges(edges, n_edges, a2 - 24, v9, 18);
    int d8  = t8 - d7;
    int t9  = tmod_edges(edges, n_edges, a2 - 25, v9, 18);
    int d9  = t9 - d8;
    int t10 = tmod_edges(edges, n_edges, a2 - 26, v9, 18);
    int d10 = t10 - d9;
    int t11 = tmod_edges(edges, n_edges, a2 - 27, v9, 18);
    int d11 = t11 - d10;
    int t12 = tmod_edges(edges, n_edges, a2 - 28, v9, 18);
    int d12 = t12 - d11;

    if (t12 + t10 + t8 + t6 + t4 + t2 != 16) return 0;
    if (d12 + d10 + d8 + d6 + d4 + d2 != 8) return 0;

    int fd[12] = {d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12};
    int finder = rss_limited_finder_lookup(fd);
    if (finder < 0) return 0;

    int u1  = tmod_edges(edges, n_edges, a2 - 15, v10, 26);
    int D1  = u1 - 1;
    int u2  = tmod_edges(edges, n_edges, a2 - 14, v10, 26);
    int D2  = u2 - D1;
    int u3  = tmod_edges(edges, n_edges, a2 - 13, v10, 26);
    int D3  = u3 - D2;
    int u4  = tmod_edges(edges, n_edges, a2 - 12, v10, 26);
    int D4  = u4 - D3;
    int u5  = tmod_edges(edges, n_edges, a2 - 11, v10, 26);
    int D5  = u5 - D4;
    int u6  = tmod_edges(edges, n_edges, a2 - 10, v10, 26);
    int D6  = u6 - D5;
    int u7  = tmod_edges(edges, n_edges, a2 - 9,  v10, 26);
    int D7  = u7 - D6;
    int u8  = tmod_edges(edges, n_edges, a2 - 8,  v10, 26);
    int D8  = u8 - D7;
    int u9  = tmod_edges(edges, n_edges, a2 - 7,  v10, 26);
    int D9  = u9 - D8;
    int u10 = tmod_edges(edges, n_edges, a2 - 6,  v10, 26);
    int D10 = u10 - D9;
    int u11 = tmod_edges(edges, n_edges, a2 - 5,  v10, 26);
    int D11 = u11 - D10;
    int u12 = tmod_edges(edges, n_edges, a2 - 4,  v10, 26);
    int D12 = u12 - D11;
    int u13 = tmod_edges(edges, n_edges, a2 - 3,  v10, 26);
    int D13 = u13 - D12;
    int u14 = tmod_edges(edges, n_edges, a2 - 2,  v10, 26);
    int D14 = u14 - D13;

    if (u14 + u12 + u10 + u8 + u6 + u4 + u2 != 26) return 0;
    int sum7 = D13 + D11 + D9 + D7 + D5 + D3 + D1;
    if ((sum7 & 1) == 0) return 0;
    int grp = (sum7 >> 1) - 3;
    if (grp < 0 || grp > 6) return 0;

    int d7a[7] = {D1, D3, D5, D7, D9, D11, D13};
    int d7b[7] = {D2, D4, D6, D8, D10, D12, D14};
    int r1 = rss_rank_value(d7a, 7, RSS_LIMITED_ODD_WIDEST[grp], 1);
    int r2 = rss_rank_value(d7b, 7, RSS_LIMITED_EVEN_WIDEST[grp], 0);
    if (r1 < 0 || r2 < 0) return 0;

    int value = RSS_LIMITED_GSUM[grp] + r2 + RSS_LIMITED_T[grp] * r1;
    int checksum = 20 * D1 + 60 * D2 + 2 * D3 + 6 * D4 + 18 * D5 + 54 * D6
                 + 73 * D7 + 41 * D8 + 34 * D9 + 13 * D10 + 39 * D11 + 28 * D12
                 + 84 * D13 + 74 * D14;

    out->value = value;
    out->checksum = mod89(checksum);
    out->finder = finder;
    return 1;
}

static int try_row_rss_limited(const uint8_t *row, int width, BarcodeResult *result)
{
    int minv = 255, maxv = 0;
    for (int x = 0; x < width; x++) {
        if (row[x] < minv) minv = row[x];
        if (row[x] > maxv) maxv = row[x];
    }
    if (maxv - minv < 8) return 0;
    int th = (minv + maxv) / 2;

    int rw[512], rc[512], nr = 0;
    int cur = row[0] < th ? 1 : 0;
    int cnt = 1;
    for (int x = 1; x < width; x++) {
        int b = row[x] < th ? 1 : 0;
        if (b == cur) {
            cnt++;
        } else {
            if (nr >= 512) return 0;
            rc[nr] = cur;
            rw[nr] = cnt;
            nr++;
            cur = b;
            cnt = 1;
        }
    }
    if (nr >= 512) return 0;
    rc[nr] = cur;
    rw[nr] = cnt;
    nr++;
    if (nr < 40) return 0;

    int n = nr;
    if (n < 30) return 0;
    int runs[320];
    int edges[321];
    if (n > 320) return 0;
    for (int i = 0; i < n; i++) runs[i] = rw[i];
    edges[0] = 0;
    for (int i = 0; i < n; i++) edges[i + 1] = edges[i] + runs[i];

    for (int a2 = 28; a2 + 14 <= n; a2++) {
        RssLimitedPart p64, p60;
        if (!rss_limited_decode_part_20864(edges, n + 1, a2, &p64)) continue;
        if (!rss_limited_decode_part_20860(edges, n + 1, a2 + 14, &p60)) continue;
        if (p64.finder != p60.finder) continue;
        if (mod89(p64.checksum + p60.checksum) != p64.finder) continue;

        /*
         * Left data character (decoded by the part_20864 chain, data runs
         * before the check character) is the high-order factor:
         * value = left * 2013571 + right.
         */
        uint64_t d14 = (uint64_t)p64.value * 2013571ULL + (uint64_t)p60.value;
        /* ISO/IEC 24724: DataBar Limited data range is 0..1999999999999.
         * When the symbol carries a GS1 Composite linkage flag the value
         * 2015133531096 (= 1000776 * 2013571) is added by the encoder. */
        if (d14 >= 2015133531096ULL) d14 -= 2015133531096ULL;
        if (d14 > 1999999999999ULL) continue;
        char gtin14[32];
        snprintf(gtin14, sizeof(gtin14), "%014llu", (unsigned long long)d14);
        if (gtin14[1] != '0' && gtin14[1] != '1') continue;

        char d13[14];
        memcpy(d13, gtin14 + 1, 13);
        d13[13] = '\0';

        memset(result, 0, sizeof(*result));
        copy_d13(d13, result->data);
        result->length = 16;
        result->confidence = 90;
        return 1;
    }
    return 0;
}

static int try_rss_limited_algorithm(const Image *img, BarcodeResult *result)
{
    BarcodeResult cands[8];
    int votes[8];
    int n = 0;
    memset(cands, 0, sizeof(cands));
    memset(votes, 0, sizeof(votes));

    int yc = img->height / 2;
    for (int y = 0; y < img->height; y++) {
        if (abs(y - yc) > img->height / 3) continue;
        const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
        BarcodeResult tmp;
        memset(&tmp, 0, sizeof(tmp));
        if (!try_row_rss_limited(row, img->width, &tmp)) continue;

        int k;
        for (k = 0; k < n; k++) {
            if (strcmp(cands[k].data, tmp.data) == 0) {
                votes[k]++;
                break;
            }
        }
        if (k == n && n < 8) {
            cands[n] = tmp;
            votes[n] = 1;
            n++;
            k = n - 1;
        }
        if (votes[k] >= 2) break;
    }

    int best = -1, best_vote = 0;
    for (int i = 0; i < n; i++) {
        if (votes[i] > best_vote) {
            best_vote = votes[i];
            best = i;
        }
    }
    if (best >= 0 && best_vote >= 2) {
        *result = cands[best];
        return 1;
    }
    return 0;
}

/*
 * RSS Expanded (ISO/IEC 24724, fmwlsymbol0062 @ line 223600) - row frontend.
 *
 * Geometry: finder+data character pairs are found anywhere in the row by
 * four symmetric block variants (A/B/C/D, mirroring the firmware's two
 * "data before finder" / "data after finder" scan directions, each with a
 * plain and a parity-shifted gate-1 anchor). Every accepted hit contributes
 * exactly one 12-bit data-character value. Hits are sorted by their data
 * window's left edge and concatenated MSB-first into a single bitstream;
 * skipping a 5-bit link-flag+header exposes the shared GS1 "general field"
 * (the same numeric/alphanumeric/ISO-646 mode-switching bitstream used by
 * GS1-128), which decodes directly to the human-readable "(AI)value..."
 * text -- the general field's ISO-646 punctuation table already contains
 * literal '(' and ')', so no separate AI-segmentation pass is needed.
 *
 * bit0 (link flag) selects between a standalone symbol (0, handled here)
 * and a symbol linked to a 2D composite component via GS1's separate
 * "compressed numeric" encodation (1, NOT implemented -- honestly rejected
 * below rather than mis-decoded).
 */

typedef struct {
    int data_lo, data_hi; /* run-index window covered by this data character */
    int value;             /* 12-bit combinatorial rank, 0..4095 */
    int w[8];               /* element widths, interleaved bar/space/bar/...,
                              * left-to-right physical order (== ISO/IEC 24724
                              * Table 14's char_widths[i][0..7]); needed for
                              * checksum verification only. */
} RssExpHit;

/* fmwlsymbol0062 combinatorial tables, byte-confirmed in the .so at
 * offset 0x17cf3c (odd/even widest-element bound, group base value,
 * group value-per-odd-rank multiplier). */
static const int RSS_EXP_ODD_WIDEST[5]  = {7, 5, 4, 3, 1};
static const int RSS_EXP_EVEN_WIDEST[5] = {2, 4, 5, 6, 8};
static const int RSS_EXP_GSUM[5]        = {0, 348, 1388, 2948, 3988};
static const int RSS_EXP_T[5]           = {4, 20, 52, 104, 204};

/* dword_1377CC: finder-type (0..4) -> D-chain start offset. Block D reuses
 * the same table (dword_138664 has not been independently byte-confirmed,
 * but empirically matches dword_1377CC at every tested index). Index 0 is
 * finder value 1 (the symbol's leading finder, adjacent to the check
 * character rather than to a regular data character); blocks A/B/C/D never
 * look it up (they require v20 in 1..4), only rss_exp_decode_check_char()
 * does. */
static const int RSS_EXP_OFFSET_TABLE[5] = {1, 3, 3, 3, 2};

/* RSS Expanded 11th ("check") character: a weighted-sum mod-211 checksum of
 * every data character's element widths (ISO/IEC 24724:2011 7.2.6, Table
 * 14/15). Not present anywhere in the current .so byte-search results, so
 * sourced from zint (BSD-licensed, backend/rss.h dbar_exp_checksum_weight /
 * dbar_exp_weight_rows -- the public reference implementation of the same
 * ISO standard) and independently bit-exact validated against the real
 * gs1-databar-expanded.bmp flat sample: the check character decoded straight
 * off the image (via rss_exp_decode_check_char()) equals
 * 211*(symbol_chars-4) + checksum_of(data_hits) with zero ordering fixup
 * needed, plus three negative controls (corrupted bar/space width, wrong
 * weight row) all correctly mismatch.
 *
 * LIMITATION -- honest scope: this is "ISO standard value + single-sample
 * firmware-empirical confirmation," NOT "firmware byte-confirmed" like
 * RSS_EXP_OFFSET_TABLE above. Only one real image (flat) exists to validate
 * against; stacked/expanded-comp remain unsolved (see todo.md backlog) so
 * there is no second independent sample to cross-check the tables against.
 * If a future genuinely-decoded stacked/comp sample disagrees with this
 * checksum, trust the sample over these tables. */
static const unsigned char RSS_EXP_CHK_WEIGHT[23][8] = {
    {   1,   3,   9,  27,  81,  32,  96,  77 },
    {  20,  60, 180, 118, 143,   7,  21,  63 },
    { 189, 145,  13,  39, 117, 140, 209, 205 },
    { 193, 157,  49, 147,  19,  57, 171,  91 },
    {  62, 186, 136, 197, 169,  85,  44, 132 },
    { 185, 133, 188, 142,   4,  12,  36, 108 },
    { 113, 128, 173,  97,  80,  29,  87,  50 },
    { 150,  28,  84,  41, 123, 158,  52, 156 },
    {  46, 138, 203, 187, 139, 206, 196, 166 },
    {  76,  17,  51, 153,  37, 111, 122, 155 },
    {  43, 129, 176, 106, 107, 110, 119, 146 },
    {  16,  48, 144,  10,  30,  90,  59, 177 },
    { 109, 116, 137, 200, 178, 112, 125, 164 },
    {  70, 210, 208, 202, 184, 130, 179, 115 },
    { 134, 191, 151,  31,  93,  68, 204, 190 },
    { 148,  22,  66, 198, 172,  94,  71,   2 },
    {   6,  18,  54, 162,  64, 192, 154,  40 },
    { 120, 149,  25,  75,  14,  42, 126, 167 },
    {  79,  26,  78,  23,  69, 207, 199, 175 },
    { 103,  98,  83,  38, 114, 131, 182, 124 },
    { 161,  61, 183, 127, 170,  88,  53, 159 },
    {  55, 165,  73,   8,  24,  72,   5,  15 },
    {  45, 135, 194, 160,  58, 174, 100,  89 },
};

/* dbar_exp_weight_rows[(data_chars-2)/2][char_index] -> row into
 * RSS_EXP_CHK_WEIGHT above. data_chars must be in [2,20]; the row index is
 * plain floor division and is well-defined for odd data_chars too (see
 * rss_exp_checksum_residue() below). */
static const unsigned char RSS_EXP_WEIGHT_ROWS[10][21] = {
    { 0,  1,  2, 0, 0,  0, 0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0,  5,  6, 3, 4,  0, 0,   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0,  9, 10, 3, 4, 13, 14,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0, 17, 18, 3, 4, 13, 14,  7,  8,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0, 17, 18, 3, 4, 13, 14, 11, 12, 21, 22,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0, 17, 18, 3, 4, 13, 14, 15, 16, 21, 22, 19, 20,  0,  0,  0,  0,  0,  0,  0,  0 },
    { 0,  1,  2, 3, 4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14,  0,  0,  0,  0,  0,  0 },
    { 0,  1,  2, 3, 4,  5,  6,  7,  8,  9, 10, 11, 12, 17, 18, 15, 16,  0,  0,  0,  0 },
    { 0,  1,  2, 3, 4,  5,  6,  7,  8,  9, 10, 11, 12, 17, 18, 19, 20, 21, 22,  0,  0 },
    { 0,  1,  2, 3, 4,  5,  6,  7,  8, 13, 14, 11, 12, 17, 18, 15, 16, 21, 22, 19, 20 },
};

#define RSS_EXP_MAX_RUNS 1024
#define RSS_EXP_MAX_HITS 40

/* Rounded-division telescoping helper shared by classify()/dchain():
 * (edges[idx+2]-edges[idx]) * mod, rounded to nearest, divided by width.
 * Returns -1 for an out-of-range window (all real results are >= 0). */
static int rss_exp_tmod(const int *edges, int nedges, int idx, int width, int mod)
{
    if (idx < 0 || idx + 2 >= nedges || width == 0) return -1;
    long num = (long)(edges[idx + 2] - edges[idx]) * mod;
    return (int)((num + width / 2) / width);
}

/* Classifies a 3-run finder sub-window into one of 5 finder types (0..4,
 * matching v20/v260/v137/v150 in the four scan blocks), or -1 if invalid. */
static int rss_exp_classify(const int *edges, int nedges, int a, int b, int c, int width)
{
    int ta = rss_exp_tmod(edges, nedges, a, width, 15);
    int tb = rss_exp_tmod(edges, nedges, b, width, 15);
    int tc = rss_exp_tmod(edges, nedges, c, width, 15);
    if (ta < 0 || tb < 0 || tc < 0) return -1;
    if (ta + tc != 14) return -1;
    if (tb == 12) return (tc == 5) ? 0 : -1;
    if (tb == 11) {
        if (tc == 6) return 4;
        if (tc == 10) return 5;
        return -1;
    }
    if (tb == 10) {
        if (tc == 5) return 1;
        if (tc == 9) return 3;
        if (tc == 7) return 2;
        return -1;
    }
    return -1;
}

typedef struct { int bar[4]; int space[4]; int grp; } RssExpDChain;

/* Telescopes the 8 data-character run widths into odd/even module counts
 * and the finder-group index; validates the two firmware-derived parity
 * checks (Ts[1]+Ts[3]+Ts[5]+Ts[7]==17, D[8]<=4, grp in [0,4]). */
static int rss_exp_dchain(const int *edges, int nedges, const int call_idxs[8],
                           int width, int offset, RssExpDChain *out)
{
    int Ts[8];
    for (int k = 0; k < 8; k++) {
        int t = rss_exp_tmod(edges, nedges, call_idxs[k], width, 17);
        if (t < 0) return 0;
        Ts[k] = t;
    }
    int D[9] = {0};
    D[1] = Ts[0] - offset;
    for (int k = 2; k <= 8; k++) D[k] = Ts[k - 1] - D[k - 1];
    if ((Ts[1] + Ts[3] + Ts[5] + Ts[7]) != 17) return 0;
    int dsum = D[2] + D[4] + D[6] + D[8];
    if (dsum % 2 != 0 || D[8] > 4) return 0;
    int grp = 6 - (dsum >> 1);
    if (grp < 0 || grp > 4) return 0;
    out->bar[0] = D[8]; out->bar[1] = D[6]; out->bar[2] = D[4]; out->bar[3] = D[2];
    out->space[0] = D[7]; out->space[1] = D[5]; out->space[2] = D[3]; out->space[3] = D[1];
    out->grp = grp;
    return 1;
}

static int rss_exp_decode_char(const int *bar, const int *space, int grp)
{
    for (int i = 0; i < 4; i++) {
        if (bar[i] < 1 || space[i] < 1) return -1;
    }
    int r1 = rss_rank_value(bar, 4, RSS_EXP_ODD_WIDEST[grp], 1);
    if (r1 < 0) return -1;
    int r2 = rss_rank_value(space, 4, RSS_EXP_EVEN_WIDEST[grp], 0);
    if (r2 < 0) return -1;
    return r2 + RSS_EXP_GSUM[grp] + r1 * RSS_EXP_T[grp];
}

static void rss_exp_push_hit(RssExpHit *hits, int *nhits, int max_hits,
                              int data_lo, int data_hi, int value,
                              const int bar[4], const int space[4])
{
    if (*nhits >= max_hits) return;
    hits[*nhits].data_lo = data_lo;
    hits[*nhits].data_hi = data_hi;
    hits[*nhits].value = value;
    for (int k = 0; k < 4; k++) {
        hits[*nhits].w[k * 2] = bar[k];
        hits[*nhits].w[k * 2 + 1] = space[k];
    }
    (*nhits)++;
}

/* Block A: finder=[p-5,p), data=[p-13,p-5) BEFORE finder, table offset. */
static void rss_exp_scan_block_a(const int *edges, int nedges, int n_runs,
                                  RssExpHit *hits, int *nhits, int max_hits)
{
    for (int p = 13; p <= n_runs; p++) {
        int g1 = 19 * (edges[p] - edges[p - 4]);
        int g1r = 24 * (edges[p - 2] - edges[p - 4]);
        if (!(g1 < g1r)) continue;
        int v11 = edges[p - 5] - edges[p - 13];
        int v12 = edges[p] - edges[p - 5];
        if (v11 <= 0 || v12 <= 0) continue;
        int lhs = 31 * v12, rhs = 30 * v11;
        int ok = lhs >= rhs, eq = (lhs == rhs);
        if (lhs <= rhs) { ok = rhs >= 37 * v12; eq = (rhs == 37 * v12); }
        if (!(eq || !ok)) continue;
        int v20 = rss_exp_classify(edges, nedges, p - 5, p - 4, p - 3, v12);
        if (v20 < 1 || v20 > 4) continue;
        int call_idxs[8] = {p - 6, p - 7, p - 8, p - 9, p - 10, p - 11, p - 12, p - 13};
        RssExpDChain dc;
        if (!rss_exp_dchain(edges, nedges, call_idxs, v11, RSS_EXP_OFFSET_TABLE[v20], &dc)) continue;
        int val = rss_exp_decode_char(dc.bar, dc.space, dc.grp);
        if (val < 0) continue;
        rss_exp_push_hit(hits, nhits, max_hits, p - 13, p - 5, val, dc.bar, dc.space);
    }
}

/* Block B: finder=[p-13,p-8), data=[p-8,p) AFTER finder, literal offset=1. */
static void rss_exp_scan_block_b(const int *edges, int nedges, int n_runs,
                                  RssExpHit *hits, int *nhits, int max_hits)
{
    for (int p = 13; p <= n_runs; p++) {
        int g1 = 19 * (edges[p - 8] - edges[p - 12]);
        int g1r = 24 * (edges[p - 10] - edges[p - 12]);
        if (!(g1 < g1r)) continue;
        int v66 = edges[p] - edges[p - 8];
        int v67 = edges[p - 8] - edges[p - 13];
        if (v66 <= 0 || v67 <= 0) continue;
        int lhs = 31 * v67, rhs = 30 * v66;
        int ok = lhs >= rhs, eq = (lhs == rhs);
        if (lhs <= rhs) { ok = rhs >= 37 * v67; eq = (rhs == 37 * v67); }
        if (!(eq || !ok)) continue;
        int v260 = rss_exp_classify(edges, nedges, p - 13, p - 12, p - 11, v67);
        if (v260 < 0) continue;
        int call_idxs[8] = {p - 9, p - 8, p - 7, p - 6, p - 5, p - 4, p - 3, p - 2};
        RssExpDChain dc;
        if (!rss_exp_dchain(edges, nedges, call_idxs, v66, 1, &dc)) continue;
        int val = rss_exp_decode_char(dc.bar, dc.space, dc.grp);
        if (val < 0) continue;
        rss_exp_push_hit(hits, nhits, max_hits, p - 8, p, val, dc.bar, dc.space);
    }
}

/* Block C: same geometry as A, different gate-1 anchor, literal offset=1. */
static void rss_exp_scan_block_c(const int *edges, int nedges, int n_runs,
                                  RssExpHit *hits, int *nhits, int max_hits)
{
    for (int p = 13; p <= n_runs; p++) {
        if (p - 1 > n_runs) continue;
        int g1 = 19 * (edges[p - 1] - edges[p - 5]);
        int g1r = 24 * (edges[p - 1] - edges[p - 3]);
        if (!(g1 < g1r)) continue;
        int v130 = edges[p] - edges[p - 5];
        int v131 = edges[p - 5] - edges[p - 13];
        if (v130 <= 0 || v131 <= 0) continue;
        int lhs = 31 * v130, rhs = 30 * v131;
        int ok = lhs >= rhs, eq = (lhs == rhs);
        if (lhs <= rhs) { ok = rhs >= 37 * v130; eq = (rhs == 37 * v130); }
        if (!(eq || !ok)) continue;
        int v137 = rss_exp_classify(edges, nedges, p - 2, p - 3, p - 4, v130);
        if (v137 < 0) continue;
        int call_idxs[8] = {p - 6, p - 7, p - 8, p - 9, p - 10, p - 11, p - 12, p - 13};
        RssExpDChain dc;
        if (!rss_exp_dchain(edges, nedges, call_idxs, v131, 1, &dc)) continue;
        int val = rss_exp_decode_char(dc.bar, dc.space, dc.grp);
        if (val < 0) continue;
        rss_exp_push_hit(hits, nhits, max_hits, p - 13, p - 5, val, dc.bar, dc.space);
    }
}

/* Block D: same geometry as B, different gate-1 anchor, table offset. */
static void rss_exp_scan_block_d(const int *edges, int nedges, int n_runs,
                                  RssExpHit *hits, int *nhits, int max_hits)
{
    for (int p = 13; p <= n_runs; p++) {
        int g1 = 19 * (edges[p - 9] - edges[p - 13]);
        int g1r = 24 * (edges[p - 9] - edges[p - 11]);
        if (!(g1 < g1r)) continue;
        int v142 = edges[p - 8] - edges[p - 13];
        int v143 = edges[p] - edges[p - 8];
        if (v142 <= 0 || v143 <= 0) continue;
        int lhs = 31 * v142, rhs = 30 * v143;
        int ok = lhs >= rhs, eq = (lhs == rhs);
        if (lhs <= rhs) { ok = rhs >= 37 * v142; eq = (rhs == 37 * v142); }
        if (!(eq || !ok)) continue;
        int v150 = rss_exp_classify(edges, nedges, p - 10, p - 11, p - 12, v142);
        if (v150 < 1 || v150 > 4) continue;
        int call_idxs[8] = {p - 9, p - 8, p - 7, p - 6, p - 5, p - 4, p - 3, p - 2};
        RssExpDChain dc;
        if (!rss_exp_dchain(edges, nedges, call_idxs, v143, RSS_EXP_OFFSET_TABLE[v150], &dc)) continue;
        int val = rss_exp_decode_char(dc.bar, dc.space, dc.grp);
        if (val < 0) continue;
        rss_exp_push_hit(hits, nhits, max_hits, p - 8, p, val, dc.bar, dc.space);
    }
}

static int rss_exp_hit_cmp(const void *a, const void *b)
{
    return ((const RssExpHit *)a)->data_lo - ((const RssExpHit *)b)->data_lo;
}

static int rss_exp_scan_all(const int *runs, int n_runs, RssExpHit *hits, int max_hits)
{
    if (n_runs + 1 > RSS_EXP_MAX_RUNS) return 0;
    int edges[RSS_EXP_MAX_RUNS];
    edges[0] = 0;
    for (int i = 0; i < n_runs; i++) edges[i + 1] = edges[i] + runs[i];
    int nedges = n_runs + 1;

    int nhits = 0;
    rss_exp_scan_block_a(edges, nedges, n_runs, hits, &nhits, max_hits);
    rss_exp_scan_block_b(edges, nedges, n_runs, hits, &nhits, max_hits);
    rss_exp_scan_block_c(edges, nedges, n_runs, hits, &nhits, max_hits);
    rss_exp_scan_block_d(edges, nedges, n_runs, hits, &nhits, max_hits);
    qsort(hits, nhits, sizeof(hits[0]), rss_exp_hit_cmp);
    return nhits;
}

/* Reads nbits (<=32) starting at absolute bit position bp from the hit
 * chain's virtual bitstream (each hit contributes exactly 12 bits,
 * MSB-first); returns -1 if the read runs past the last hit. */
static long rss_exp_peek(const RssExpHit *hits, int nhits, int bp, int nbits)
{
    if (nbits <= 0) return -1;
    long v = 0;
    for (int i = 0; i < nbits; i++) {
        int pos = bp + i;
        int hit_idx = pos / 12, bit_in_hit = pos % 12;
        if (hit_idx < 0 || hit_idx >= nhits) return -1;
        int bit = (hits[hit_idx].value >> (11 - bit_in_hit)) & 1;
        v = (v << 1) | bit;
    }
    return v;
}

/* GS1 general field decoder: the same numeric/alphanumeric/ISO-646
 * mode-switching bitstream scheme used by GS1-128. start_bit is an
 * absolute bit position into the hit chain; *consumed_out is returned as
 * an absolute bit position too, so callers can compute unused-bit padding
 * directly against nhits*12. */
static int rss_exp_decode_general_field(const RssExpHit *hits, int nhits, int start_bit,
                                         int total_bits, char *out, int out_cap, int *consumed_out)
{
    static const char isoiec_puncs[] = "!\"%&'()*+,-./:;<=>?_ ";
    static const char alnum_puncs[] = "*,-./";
    enum { GF_NUMERIC = 1, GF_ALPHANUMERIC = 2, GF_ISOIEC = 3 };
    int i = start_bit, mode = GF_NUMERIC, w = 0;

    while (i < total_bits && w < out_cap - 2) {
        if (mode == GF_NUMERIC) {
            long v4 = rss_exp_peek(hits, nhits, i, 4);
            if (v4 < 0) break;
            if (v4 == 0) { i += 4; mode = GF_ALPHANUMERIC; continue; }
            long v7 = rss_exp_peek(hits, nhits, i, 7);
            if (v7 < 0) break;
            long val = v7 - 8;
            if (val < 0 || val > 120) break;
            int d1 = (int)(val / 11), d2 = (int)(val % 11);
            if (d1 > 10 || d2 > 10) break;
            out[w++] = (d1 == 10) ? '\x1d' : (char)('0' + d1);
            out[w++] = (d2 == 10) ? '\x1d' : (char)('0' + d2);
            i += 7;
        } else if (mode == GF_ALPHANUMERIC) {
            long v3 = rss_exp_peek(hits, nhits, i, 3);
            if (v3 == 0) { i += 3; mode = GF_NUMERIC; continue; }
            long v5 = rss_exp_peek(hits, nhits, i, 5);
            if (v5 < 0) break;
            if (v5 == 4) { i += 5; mode = GF_ISOIEC; continue; }
            if (v5 == 15) { out[w++] = '\x1d'; i += 5; mode = GF_NUMERIC; continue; }
            if (v5 >= 5 && v5 <= 14) { out[w++] = (char)(v5 + 43); i += 5; continue; }
            long v6 = rss_exp_peek(hits, nhits, i, 6);
            if (v6 >= 32 && v6 <= 57) { out[w++] = (char)(v6 + 33); i += 6; continue; }
            if (v6 >= 58 && v6 <= 62) { out[w++] = alnum_puncs[v6 - 58]; i += 6; continue; }
            break;
        } else { /* GF_ISOIEC */
            long v3 = rss_exp_peek(hits, nhits, i, 3);
            if (v3 == 0) { i += 3; mode = GF_NUMERIC; continue; }
            long v5 = rss_exp_peek(hits, nhits, i, 5);
            if (v5 < 0) break;
            if (v5 == 4) { i += 5; mode = GF_ALPHANUMERIC; continue; }
            if (v5 == 15) { out[w++] = '\x1d'; i += 5; mode = GF_NUMERIC; continue; }
            if (v5 >= 5 && v5 <= 14) { out[w++] = (char)(v5 + 43); i += 5; continue; }
            long v7 = rss_exp_peek(hits, nhits, i, 7);
            if (v7 >= 64 && v7 <= 89) { out[w++] = (char)(v7 + 1); i += 7; continue; }
            if (v7 >= 90 && v7 <= 115) { out[w++] = (char)(v7 + 7); i += 7; continue; }
            long v8 = rss_exp_peek(hits, nhits, i, 8);
            if (v8 >= 232 && v8 <= 253) { out[w++] = isoiec_puncs[v8 - 232]; i += 8; continue; }
            break;
        }
    }
    out[w] = '\0';
    if (consumed_out) *consumed_out = i;
    return w;
}

/* Decodes the RSS Expanded check character (ISO/IEC 24724 7.2.6). It sits
 * immediately before the row's first finder, in the same "data BEFORE
 * finder" shape as block A, but keyed off finder value 1 (v20 == 0) -- the
 * one finder classification blocks A/C deliberately skip, since a genuine
 * data character is never found on that side of it (see
 * RSS_EXP_OFFSET_TABLE's comment). p must equal the row's leftmost data
 * hit's data_lo; returns -1 if there isn't room (fewer than 13 runs before
 * it) or any gate fails.
 *
 * NOTE -- this v20==0 / OFFSET_TABLE[0] path is new code added purely to
 * read the check character for verification. It is NOT a firmware data-
 * decoding behavior the production scan blocks (A/B/C/D) exercise or that
 * has been observed decoding real payload data in the .so; it only feeds
 * the checksum gate below. */
static int rss_exp_decode_check_char(const int *edges, int nedges, int p)
{
    if (p < 13 || p >= nedges) return -1;
    int g1 = 19 * (edges[p] - edges[p - 4]);
    int g1r = 24 * (edges[p - 2] - edges[p - 4]);
    if (!(g1 < g1r)) return -1;
    int v11 = edges[p - 5] - edges[p - 13];
    int v12 = edges[p] - edges[p - 5];
    if (v11 <= 0 || v12 <= 0) return -1;
    int lhs = 31 * v12, rhs = 30 * v11;
    int ok = lhs >= rhs, eq = (lhs == rhs);
    if (lhs <= rhs) { ok = rhs >= 37 * v12; eq = (rhs == 37 * v12); }
    if (!(eq || !ok)) return -1;
    int v20 = rss_exp_classify(edges, nedges, p - 5, p - 4, p - 3, v12);
    if (v20 != 0) return -1; /* check character always sits on finder value 1 */
    int call_idxs[8] = {p - 6, p - 7, p - 8, p - 9, p - 10, p - 11, p - 12, p - 13};
    RssExpDChain dc;
    if (!rss_exp_dchain(edges, nedges, call_idxs, v11, RSS_EXP_OFFSET_TABLE[v20], &dc)) return -1;
    return rss_exp_decode_char(dc.bar, dc.space, dc.grp);
}

/* ISO/IEC 24724 7.2.6 weighted-sum mod-211 checksum, computed purely from
 * the data characters' widths (hits[], in the order they occupy the
 * symbol's virtual bitstream -- for a flat row that's already sorted by
 * data_lo == physical left-to-right order matching Table 14's
 * char_widths[i] indexing directly; for a stacked symbol it's the
 * boustrophedon-stitched chain built by rss_exp_stitch_bands(), which is
 * the same ordering the weight-row table expects). data_chars must be in
 * [2,20] for the weight-row table to be defined; returns -1 otherwise.
 *
 * NOTE: data_chars is NOT required to be even. zint's reference encoder
 * (backend/rss.c, the composite check-character helper: data_chars = bp/12
 * with row = dbar_exp_weight_rows[(data_chars-2)/2][i]) uses plain floor
 * division with no parity requirement at all -- an earlier version of this
 * function had an extra "(nhits % 2) != 0 -> reject" gate that was only
 * ever validated against a single even-count flat sample and is not an
 * ISO/zint requirement. Stacked symbols can legitimately split their data
 * characters across rows in odd-sized groups (bit-exact validated against
 * a real stacked sample: nhits=9 -> row_idx=3, computed residue matches the
 * image's own independently-decoded check character). */
static int rss_exp_checksum_residue(const RssExpHit *hits, int nhits)
{
    if (nhits < 2 || nhits > 20) return -1;
    int row_idx = (nhits - 2) / 2;
    int checksum = 0;
    for (int i = 0; i < nhits; i++) {
        int row = RSS_EXP_WEIGHT_ROWS[row_idx][i];
        for (int j = 0; j < 8; j++) checksum += hits[i].w[j] * RSS_EXP_CHK_WEIGHT[row][j];
    }
    return checksum % 211;
}

/* GS1 DataBar Expanded encoding_method=1 "compressed GTIN" field (ISO/IEC
 * 24724:2011 Table 10 method 1 / GS1 General Specifications section on
 * DataBar Expanded compaction): a compact 48-bit field used instead of the
 * generic general-field-only encodation (method 2) whenever the source
 * payload begins with the literal digits "01" (AI (01), GTIN). Bit layout
 * cross-checked against zint's encoder (backend/rss.c
 * dbar_exp_binary_string(), encoding_method == 1 branch -- consulted as an
 * algorithm reference only, not copied):
 *   bit0      link flag (checked by the caller before this is reached)
 *   bit1..3   method field "1XX": bit1==1 selects this method; bits 2-3 are
 *             symbol-character-count parity bits, not needed to decode
 *   bit4..7   digit2: 4-bit leading digit right after the stripped "01"
 *   bit8..47  4 groups of 10 bits, each a 3-decimal-digit (0..999) group
 *             covering the GTIN's next 12 digits
 * The GTIN's own trailing mod-10 check digit is never transmitted -- it is
 * recomputed by the existing gs1_mod10_digit()/copy_d13() helpers (already
 * used by the RSS-14 decoder above for the exact same GTIN check-digit
 * algorithm), from the 13 decoded digits. Returns the absolute bit position
 * where the compressed field ends (always 48 on success) so the caller can
 * resume general-field decoding for any remaining payload right after it;
 * returns -1 if bit1 isn't set (not method 1) or any group is out of range,
 * never a guessed/partial value. */
static int rss_exp_decode_gtin_compressed(const RssExpHit *hits, int nhits, char *out, int *out_len)
{
    long method3 = rss_exp_peek(hits, nhits, 1, 3);
    if (method3 < 4) return -1; /* bit1 (this 3-bit field's MSB) must be 1 */
    long digit2 = rss_exp_peek(hits, nhits, 4, 4);
    if (digit2 < 0 || digit2 > 9) return -1;

    char digits[13];
    digits[0] = (char)('0' + digit2);
    int dp = 1;
    for (int g = 0; g < 4; g++) {
        long grp = rss_exp_peek(hits, nhits, 8 + g * 10, 10);
        if (grp < 0 || grp > 999) return -1;
        digits[dp++] = (char)('0' + (grp / 100) % 10);
        digits[dp++] = (char)('0' + (grp / 10) % 10);
        digits[dp++] = (char)('0' + grp % 10);
    }
    copy_d13(digits, out); /* "01" + 13 digits + GTIN mod10 check digit */
    *out_len = 16;
    return 48;
}

/* Decodes the payload text of an already checksum-validated data-character
 * chain (flat row or boustrophedon-stitched stacked chain) by dispatching
 * on the two structural bits that flat and stacked/standalone and
 * composite-linked symbols all agree on:
 *  - bit0: link flag. 0 = standalone symbol, 1 = linked to a 2D composite
 *    component (GS1 General Specifications composite symbol indicator).
 *    Both values are structurally valid and decoded identically past this
 *    point -- the flag only tells the *caller* whether to also look for a
 *    2D component, it does not change the encodation bit layout at all
 *    (cross-checked against zint's encoder, dbar_exp_binary_string(): the
 *    link-flag bit is written by the caller before the method-specific
 *    field, entirely independent of which encoding_method follows). Callers
 *    that only handle standalone symbols may still reject bit0==1
 *    themselves before calling this.
 *  - bit1: encodation method. 0 = generic general-field-only method
 *    (general field starts at bit 5); 1 = GTIN-compressed method (see
 *    rss_exp_decode_gtin_compressed() above), general field (if any)
 *    resumes right after it.
 * Returns the decoded text length (>0) on success, written to out (out_cap
 * bytes, NUL-terminated); 0 if any structural check fails -- never a
 * partial or guessed string. */
static int rss_exp_decode_payload(const RssExpHit *hits, int nhits, char *out, int out_cap)
{
    int total_bits = nhits * 12;
    long method_bit = rss_exp_peek(hits, nhits, 1, 1);
    char text[BARCODE_MAX_LEN];
    int wlen = 0;

    if (method_bit == 1) {
        char gtin[24];
        int glen = 0;
        int end_bit = rss_exp_decode_gtin_compressed(hits, nhits, gtin, &glen);
        if (end_bit <= 0) return 0;
        char rest[BARCODE_MAX_LEN];
        int consumed = 0;
        int rlen = rss_exp_decode_general_field(hits, nhits, end_bit, total_bits,
                                                  rest, sizeof(rest), &consumed);
        int pad = total_bits - consumed;
        if (rlen < 0 || pad < 0 || pad >= 12 || glen + rlen >= (int)sizeof(text)) return 0;
        memcpy(text, gtin, (size_t)glen);
        memcpy(text + glen, rest, (size_t)rlen + 1);
        wlen = glen + rlen;
    } else if (method_bit == 0) {
        int consumed = 0;
        wlen = rss_exp_decode_general_field(hits, nhits, 5, total_bits, text, sizeof(text), &consumed);
        int pad = total_bits - consumed;
        if (wlen <= 0 || pad < 0 || pad >= 12) return 0;
    } else {
        return 0;
    }

    if (wlen <= 0 || wlen >= out_cap) return 0;
    memcpy(out, text, (size_t)wlen + 1);
    return wlen;
}

/*
 * Decodes one already-binarized row (dark/light run widths) end to end.
 * Hard-reject gates (never fall back to a possibly-wrong string):
 *  - at least 5 data characters. The original gate here required 6, which
 *    was only ever validated against one even-count flat sample -- not an
 *    ISO/zint minimum (the checksum weight-row table above is defined for
 *    any nhits in [2,20], and zint's own encoder imposes no lower bound
 *    beyond that). Real composite-linked linear components (bit-exact
 *    validated against both gs1-databar-expanded-comp.bmp and
 *    gs1-databar-expanded-stacked-comp.bmp: nhits=5, check_val==211*
 *    (symbol_chars-4)+residue, decodes to the exact expected GTIN-compressed
 *    payload) genuinely only carry 5 data characters, so the heuristic is
 *    tightened to the smallest value that still admits them rather than
 *    dropped outright -- the checksum gate below (a real ISO/IEC 24724
 *    mod-211 check, not a heuristic) is what actually guards against
 *    spurious short chains.
 *  - the sorted hit chain must be contiguous: adjacent hits either share an
 *    edge (gap 0) or are separated by exactly one shared 5-run finder
 *    (gap 5) -- any other gap means the chain is discontinuous/spurious
 *  - the chain must explain nearly the whole row (coverage gate)
 *  - the check character (decoded independently from its own widths, just
 *    before the row's first finder) must be decodable and must match the
 *    mod-211 weighted-sum checksum of the data characters plus encode the
 *    right symbol character count -- the strongest gate, a real ISO/IEC
 *    24724 checksum rather than a structural heuristic; a row that leaves
 *    no room for its own check character is rejected outright rather than
 *    let through unverified
 *  - bit0 may be 0 (standalone symbol) or 1 (linked to a 2D composite
 *    component, GS1 General Specifications composite symbol indicator) --
 *    both are structurally valid encodations and are decoded identically by
 *    rss_exp_decode_payload(); the caller/pipeline decides what to do with
 *    a linked-symbol result (e.g. composite_linear_decode() merges it with
 *    a 2D component), this decoder just needs to not mis-decode past the
 *    flag rather than blanket-reject it
 *  - general-field decode must consume all but a small pad of the
 *    available bits (rejects garbage/incomplete decodes)
 */
static int rss_exp_decode_row(const int *runs, int n_runs, char *out, int out_cap)
{
    RssExpHit hits[RSS_EXP_MAX_HITS];
    int nhits = rss_exp_scan_all(runs, n_runs, hits, RSS_EXP_MAX_HITS);
    if (nhits < 5) return 0;

    for (int i = 1; i < nhits; i++) {
        int gap = hits[i].data_lo - hits[i - 1].data_hi;
        if (gap != 0 && gap != 5) return 0;
    }
    if (hits[0].data_lo > 20) return 0;
    if (n_runs - hits[nhits - 1].data_hi > 20) return 0;

    {
        int edges[RSS_EXP_MAX_RUNS];
        edges[0] = 0;
        for (int i = 0; i < n_runs; i++) edges[i + 1] = edges[i] + runs[i];
        int check_val = rss_exp_decode_check_char(edges, n_runs + 1, hits[0].data_lo);
        if (check_val < 0) return 0;
        int residue = rss_exp_checksum_residue(hits, nhits);
        if (residue < 0) return 0;
        int symbol_chars = nhits + 1;
        if (check_val != 211 * (symbol_chars - 4) + residue) return 0;
    }

    long link = rss_exp_peek(hits, nhits, 0, 1);
    if (link != 0 && link != 1) return 0;

    return rss_exp_decode_payload(hits, nhits, out, out_cap);
}

static int rss_exp_extract_runs(const uint8_t *row, int width, int *runs, int max_runs)
{
    int minv = 255, maxv = 0;
    for (int x = 0; x < width; x++) {
        if (row[x] < minv) minv = row[x];
        if (row[x] > maxv) maxv = row[x];
    }
    if (maxv - minv < 8) return 0;
    int th = (minv + maxv) / 2;

    int rw[RSS_EXP_MAX_RUNS], rc[RSS_EXP_MAX_RUNS], nr = 0;
    int cur = row[0] < th ? 1 : 0;
    int cnt = 1;
    for (int x = 1; x < width; x++) {
        int b = row[x] < th ? 1 : 0;
        if (b == cur) {
            cnt++;
        } else {
            if (nr >= RSS_EXP_MAX_RUNS) return 0;
            rc[nr] = cur; rw[nr] = cnt; nr++;
            cur = b; cnt = 1;
        }
    }
    if (nr >= RSS_EXP_MAX_RUNS) return 0;
    rc[nr] = cur; rw[nr] = cnt; nr++;

    int s = 0, e = nr - 1;
    while (s <= e && rc[s] == 0) s++;
    while (s <= e && rc[e] == 0) e--;
    if (e < s || e - s + 1 > max_runs) return 0;
    int n = e - s + 1;
    for (int i = 0; i < n; i++) runs[i] = rw[s + i];
    return n;
}

static int try_rss_expanded_zbar(const Image *img, BarcodeResult *result, size_t min_len, size_t max_len)
{
    char cands[16][BARCODE_MAX_LEN];
    int votes[16];
    int n = 0;

    int max_rows = img->height < 192 ? img->height : 192;
    int chosen[192];
    int chosen_n = 0;
    int yc = img->height / 2;

    for (int pass = 0; pass < 2 && chosen_n < max_rows; pass++) {
        for (int y = 0; y < img->height && chosen_n < max_rows; y++) {
            const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
            int minv = 255, maxv = 0;
            for (int x = 0; x < img->width; x++) {
                int v = row[x];
                if (v < minv) minv = v;
                if (v > maxv) maxv = v;
            }
            int contrast = maxv - minv;
            int prefer = abs(y - yc) < img->height / 3;
            if ((pass == 0 && contrast >= 24 && prefer) || (pass == 1 && contrast >= 16)) {
                int seen = 0;
                for (int i = 0; i < chosen_n; i++) {
                    if (chosen[i] == y) { seen = 1; break; }
                }
                if (!seen) chosen[chosen_n++] = y;
            }
        }
    }

    int runs[RSS_EXP_MAX_RUNS];
    for (int i = 0; i < chosen_n; i++) {
        int y = chosen[i];
        const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
        int nr = rss_exp_extract_runs(row, img->width, runs, RSS_EXP_MAX_RUNS);
        if (nr < 20) continue;

        char text[BARCODE_MAX_LEN];
        int tlen = rss_exp_decode_row(runs, nr, text, sizeof(text));
        if (tlen <= 0) continue;
        if ((size_t)tlen < min_len || (size_t)tlen > max_len) continue;

        int k;
        for (k = 0; k < n; k++) {
            if (strcmp(cands[k], text) == 0) { votes[k]++; break; }
        }
        if (k == n && n < 16) {
            memcpy(cands[n], text, (size_t)tlen + 1);
            votes[n] = 1;
            n++;
            k = n - 1;
        }
        if (k < n && votes[k] >= 3) break;
    }

    int best = -1, best_vote = 0;
    for (int i = 0; i < n; i++) {
        if (votes[i] > best_vote) { best_vote = votes[i]; best = i; }
    }
    if (best >= 0 && best_vote >= 2) {
        size_t L = strlen(cands[best]);
        memcpy(result->data, cands[best], L + 1);
        result->length = (int)L;
        result->confidence = 88;
        return 1;
    }
    return 0;
}

int rss14_stacked_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !result || !img->pixels) return 0;
    if (img->width < 80 || img->height < 20) return 0;

    memset(result, 0, sizeof(*result));

    /* Algorithmic: scan top half for left pair, bottom for right pair. */
    return try_rss14_stacked_algorithm(img, result);
}

int rss_limited_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !result || !img->pixels) return 0;
    if (img->width < 80 || img->height < 20) return 0;
    memset(result, 0, sizeof(*result));
    return try_rss_limited_algorithm(img, result);
}

int rss_expanded_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !result || !img->pixels) return 0;
    if (img->width < 80 || img->height < 20) return 0;
    memset(result, 0, sizeof(*result));
    return try_rss_expanded_zbar(img, result, 16, 20);
}

/*
 * RSS Expanded Stacked (ISO/IEC 24724): output-alignment layer on top of
 * the flat frontend above. A stacked symbol splits its finder+data
 * character chain across several printed rows instead of one, so it needs
 * (1) per-row band detection, (2) boustrophedon row stitching into a single
 * virtual bitstream, and (3) a GS1 DataBar Expanded "encoding_method=1"
 * (GTIN-compressed field) decoder -- the sample this targets encodes a
 * literal "01..." GTIN payload, which per ISO/IEC 24724:2011 Table 10 /
 * zint's dbar_exp_binary_string() uses this compact field instead of the
 * generic general-field-only encodation (method 2) the flat path above
 * assumes. All of this reuses the existing bit-exact primitives
 * (rss_exp_scan_all/the four scan_block variants, rss_exp_decode_check_char,
 * rss_exp_checksum_residue, rss_exp_decode_general_field, gs1_mod10_digit)
 * completely unchanged; only this stitching/dispatch layer is new. Any row
 * or chain that doesn't decode to a fully self-consistent, checksum-valid
 * structure is rejected outright (falls through to the existing flat
 * try_rss_expanded_zbar()), never patched over with a guess.
 */

#define RSS_EXP_MAX_BANDS 12

typedef struct {
    RssExpHit hits[RSS_EXP_MAX_HITS];
    int nhits;
    int edges[RSS_EXP_MAX_RUNS];
    int nedges;
} RssExpBand;

/* One physical printed row of a stacked symbol spans a contiguous run of
 * image scanlines. A scanline "qualifies" as belonging to a row using the
 * same structural criterion the flat decoder already relies on: enough
 * contrast to binarize, at least 20 runs (room for a real finder+data
 * chain), and at least one hit from the unmodified rss_exp_scan_all().
 * Within each contiguous run of qualifying scanlines, the one with the most
 * hits is kept as that row's representative (a more complete decode of the
 * same physical row); the search then resumes after that run, so a genuine
 * flat (single-row) image always collapses to exactly one band. */
static int rss_exp_find_bands(const Image *img, RssExpBand *bands, int max_bands)
{
    int nbands = 0;
    int runs[RSS_EXP_MAX_RUNS];
    int y = 0;
    while (y < img->height && nbands < max_bands) {
        const uint8_t *row = img->pixels + (size_t)y * (size_t)img->width;
        int minv = 255, maxv = 0;
        for (int x = 0; x < img->width; x++) {
            if (row[x] < minv) minv = row[x];
            if (row[x] > maxv) maxv = row[x];
        }
        if (maxv - minv < 24) { y++; continue; }
        int nr = rss_exp_extract_runs(row, img->width, runs, RSS_EXP_MAX_RUNS);
        if (nr < 20) { y++; continue; }
        RssExpHit hits[RSS_EXP_MAX_HITS];
        int nhits = rss_exp_scan_all(runs, nr, hits, RSS_EXP_MAX_HITS);
        if (nhits < 1) { y++; continue; }

        RssExpBand band;
        memcpy(band.hits, hits, sizeof(hits[0]) * (size_t)nhits);
        band.nhits = nhits;
        band.edges[0] = 0;
        for (int i = 0; i < nr; i++) band.edges[i + 1] = band.edges[i] + runs[i];
        band.nedges = nr + 1;

        int yy = y + 1;
        for (; yy < img->height; yy++) {
            const uint8_t *row2 = img->pixels + (size_t)yy * (size_t)img->width;
            int mn = 255, mx = 0;
            for (int x = 0; x < img->width; x++) {
                if (row2[x] < mn) mn = row2[x];
                if (row2[x] > mx) mx = row2[x];
            }
            if (mx - mn < 24) break;
            int nr2 = rss_exp_extract_runs(row2, img->width, runs, RSS_EXP_MAX_RUNS);
            if (nr2 < 20) break;
            RssExpHit hits2[RSS_EXP_MAX_HITS];
            int nhits2 = rss_exp_scan_all(runs, nr2, hits2, RSS_EXP_MAX_HITS);
            if (nhits2 < 1) break;
            if (nhits2 > band.nhits) {
                memcpy(band.hits, hits2, sizeof(hits2[0]) * (size_t)nhits2);
                band.nhits = nhits2;
                band.edges[0] = 0;
                for (int i = 0; i < nr2; i++) band.edges[i + 1] = band.edges[i] + runs[i];
                band.nedges = nr2 + 1;
            }
        }
        bands[nbands++] = band;
        y = yy;
    }
    return nbands;
}

/* Boustrophedon assembly: a stacked symbol's printed rows alternate scan
 * direction top-to-bottom. Row 0 (topmost) contributes its hits in the same
 * left-to-right order the flat decoder already produces; every following
 * odd-indexed row (1, 3, 5, ...) has its hit list reversed before being
 * appended -- list-order reversal only (each hit's own data_lo/data_hi/
 * value/w[] already encode absolute physical widths and are left
 * untouched), not a re-scan of mirrored pixels. Bit-exact validated against
 * the one real stacked sample available (3 rows: normal, reversed,
 * normal). */
static int rss_exp_stitch_bands(const RssExpBand *bands, int nbands, RssExpHit *out, int max_hits)
{
    int n = 0;
    for (int b = 0; b < nbands; b++) {
        int nb = bands[b].nhits;
        if (n + nb > max_hits) return -1;
        if ((b % 2) == 0) {
            memcpy(out + n, bands[b].hits, sizeof(out[0]) * (size_t)nb);
        } else {
            for (int i = 0; i < nb; i++) out[n + i] = bands[b].hits[nb - 1 - i];
        }
        n += nb;
    }
    return n;
}

int rss_expanded_stacked_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !result || !img->pixels) return 0;
    if (img->width < 80 || img->height < 20) return 0;

    memset(result, 0, sizeof(*result));

    RssExpBand bands[RSS_EXP_MAX_BANDS];
    int nbands = rss_exp_find_bands(img, bands, RSS_EXP_MAX_BANDS);
    if (nbands >= 2) {
        RssExpHit hits[RSS_EXP_MAX_HITS];
        int nhits = rss_exp_stitch_bands(bands, nbands, hits, RSS_EXP_MAX_HITS);
        if (nhits >= 6) {
            int check_val = rss_exp_decode_check_char(bands[0].edges, bands[0].nedges,
                                                        bands[0].hits[0].data_lo);
            int residue = rss_exp_checksum_residue(hits, nhits);
            int symbol_chars = nhits + 1;
            long link = rss_exp_peek(hits, nhits, 0, 1);
            if (check_val >= 0 && residue >= 0
                && check_val == 211 * (symbol_chars - 4) + residue
                && (link == 0 || link == 1)) {
                char text[BARCODE_MAX_LEN];
                int wlen = rss_exp_decode_payload(hits, nhits, text, sizeof(text));
                if (wlen > 0 && wlen < (int)sizeof(result->data)) {
                    memcpy(result->data, text, (size_t)wlen + 1);
                    result->length = wlen;
                    result->confidence = 88;
                    return 1;
                }
            }
        }
    }

    return try_rss_expanded_zbar(img, result, 16, BARCODE_MAX_LEN - 1);
}

int rss_gs1_decode(const Image *img, BarcodeResult *result)
{
    if (!img || !result || !img->pixels) return 0;
    if (img->width < 80 || img->height < 20) return 0;

    RssState st;
    clear_rss_state(&st);
    reset_scan_state(&st);

    memset(result, 0, sizeof(*result));
    if (try_rss14_algorithm(img, result)) return 1;
    if (rss_limited_decode(img, result)) return 1;
    if (rss_expanded_stacked_decode(img, result)) return 1;
    if (rss_expanded_decode(img, result)) return 1;

    return 0;
}

int rss_gs1_decode_fast(const Image *img, BarcodeResult *result)
{
    if (!img || !result || !img->pixels) return 0;
    if (img->width < 80 || img->height < 20) return 0;
    return rss_gs1_decode(img, result);
}
