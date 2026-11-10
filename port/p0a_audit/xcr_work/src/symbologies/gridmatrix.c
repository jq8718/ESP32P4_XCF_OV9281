/*
 * GridMatrix (Grid Matrix / 网格矩阵码) decoder -- from-scratch restoration.
 *
 * No decompiled anchor exists for this symbology (0 hits for "GridMatrix" in
 * the reference decompile), so this file is implemented directly against the
 * public AIM Global Document Number AIMD014 Rev. 1.63 standard, cross-checked
 * against zint's open-source ENCODER (backend/gridmtx.c / gridmtx.h, BSD-3,
 * https://github.com/zint/zint) -- zint has no Grid Matrix decoder, so the
 * bit-level structure here was derived by reading the encoder in reverse and
 * validated end-to-end against a zint-generated reference image plus the
 * real test sample (test/001-覆盖率测试/gridmatrix.bmp, expected "12345678").
 * The constant tables in Section 1 are transcribed verbatim from gridmtx.h
 * (itself transcribed from AIMD014 Table 1/9/11/A.1) and are cited inline.
 *
 * Scope: the NUMERAL encoding mode (Section 8) is fully implemented and
 * validated -- it is the only mode present in the test corpus. The other
 * five modes (Chinese/GB2312, Lower, Upper, Mixed, Byte -- Section 9) are
 * implemented structurally, by mirroring zint's encoder bit-assignment
 * logic, but have NOT been validated against any real sample (none exist in
 * the test corpus). Treat them as a best-effort skeleton, not a proven path.
 *
 * Anti-false-positive strategy (important: this decoder must not hijack
 * other 2D symbols, notably an inverted DataMatrix that visually resembles
 * a dark/light checkerboard): every candidate location must pass TWO
 * independent gates before a result is returned --
 *   (1) structural gate: the macromodule frame checkerboard (every 6x6
 *       macromodule's outer ring is either solid-dark or solid-light,
 *       alternating by (x+y)%2 -- see gm_place_macromodule/frame-drawing in
 *       gridmtx.c) must match with very low error over the *entire* grid;
 *   (2) error-correction gate: Reed-Solomon decode must succeed cleanly
 *       (no uncorrectable block) for every interleaved block.
 * Only when both gates pass is a result emitted.
 */

#include "gridmatrix.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ════════════════════════════════════════════════════════════════════════
 * Section 1: constant tables (verbatim transcription of zint's gridmtx.h,
 * itself derived from AIMD014 Table 1, Table 9, Table 11 and Table A.1).
 * ════════════════════════════════════════════════════════════════════════ */

#define GM_MAX_LAYERS 13

/* Table 1 (via Table A.1): max data codewords per (layers, ecc_level).
 * ecc_level is 1..5 (90%,80%,70%,60%,50% recommended-usable fraction);
 * ecc_level 1 is invalid for layers==1 (entry left as 0). */
static const short gm_data_cws[GM_MAX_LAYERS][5] = {
     {    0,   15,   13,  11,   9 },
     {   45,   40,   35,  30,  25 },
     {   89,   79,   69,  59,  49 },
     {  146,  130,  114,  98,  81 },
     {  218,  194,  170, 146, 121 },
     {  305,  271,  237, 203, 169 },
     {  405,  360,  315, 270, 225 },
     {  521,  463,  405, 347, 289 },
     {  650,  578,  506, 434, 361 },
     {  794,  706,  618, 530, 441 },
     {  953,  847,  741, 635, 529 },
     { 1125, 1000,  875, 750, 625 },
     { 1313, 1167, 1021, 875, 729 },
};

/* Table A.1: block layout. n1/b1 = size/count of the "big" blocks, n2=n1-1
 * is the size of the "small" blocks (b2 of them). */
static const char gm_n1[GM_MAX_LAYERS] = {
    18, 50, 98, 81, 121, 113, 113, 116, 121, 126, 118, 125, 122
};
static const char gm_b1[GM_MAX_LAYERS] = {
    1, 1, 1, 2, 2, 2, 2, 3, 2, 7, 5, 10, 6
};
static const char gm_b2[GM_MAX_LAYERS] = {
    0, 0, 0, 0, 0, 1, 2, 2, 4, 0, 4, 0, 6
};

/* Table A.1: E1/B3/E2 -- first b3 blocks (of the b1+b2 total, in block
 * order) use e1 ECC codewords per block, the rest use e2. Indexed
 * [layers-1][ecc_level-1][0..2] = {e1,b3,e2}. */
static const char gm_e1b3e2[GM_MAX_LAYERS][5][3] = {
    { {  0, 0,  0 }, {  3,  1,  0 }, {  5, 1,  0 }, {  7,  1,  0 }, {  9, 1,  0 } },
    { {  5, 1,  0 }, { 10,  1,  0 }, { 15, 1,  0 }, { 20,  1,  0 }, { 25, 1,  0 } },
    { {  9, 1,  0 }, { 19,  1,  0 }, { 29, 1,  0 }, { 39,  1,  0 }, { 49, 1,  0 } },
    { {  8, 2,  0 }, { 16,  2,  0 }, { 24, 2,  0 }, { 32,  2,  0 }, { 41, 1, 40 } },
    { { 12, 2,  0 }, { 24,  2,  0 }, { 36, 2,  0 }, { 48,  2,  0 }, { 61, 1, 60 } },
    { { 11, 3,  0 }, { 23,  1, 22 }, { 34, 2, 33 }, { 45,  3,  0 }, { 57, 1, 56 } },
    { { 12, 1, 11 }, { 23,  2, 22 }, { 34, 3, 33 }, { 45,  4,  0 }, { 57, 1, 56 } },
    { { 12, 2, 11 }, { 23,  5,  0 }, { 35, 3, 34 }, { 47,  1, 46 }, { 58, 4, 57 } },
    { { 12, 6,  0 }, { 24,  6,  0 }, { 36, 6,  0 }, { 48,  6,  0 }, { 61, 1, 60 } },
    { { 13, 4, 12 }, { 26,  1, 25 }, { 38, 5, 37 }, { 51,  2, 50 }, { 63, 7,  0 } },
    { { 12, 6, 11 }, { 24,  4, 23 }, { 36, 2, 35 }, { 47,  9,  0 }, { 59, 7, 58 } },
    { { 13, 5, 12 }, { 25, 10,  0 }, { 38, 5, 37 }, { 50, 10,  0 }, { 63, 5, 62 } },
    { { 13, 1, 12 }, { 25,  3, 24 }, { 37, 5, 36 }, { 49,  7, 48 }, { 61, 9, 60 } },
};

/* Mode-switch codewords/lengths (Table 9). Rows: 0=initial, then
 * CHN,NUM,LWR,UPR,MXD,BYT,EOD; columns: target CHN,NUM,LWR,UPR,MXD,BYT. */
#define GM_CHINESE 1
#define GM_NUMERAL 2
#define GM_LOWER   3
#define GM_UPPER   4
#define GM_MIXED   5
#define GM_BYTE    6
#define GM_EOD     7

static const short gm_mode_switch[8][6] = {
    {    1,    2,    3,    4,    5,    7 },
    {    0, 8161, 8162, 8163, 8164, 8165 },
    { 1019,    0, 1020, 1021, 1022, 1023 },
    {   28,   29,    0,   30,  124,  126 },
    {   28,   29,   30,    0,  124,  126 },
    { 1009, 1010, 1011, 1012,    0, 1015 },
    {    1,    2,    3,    4,    5,    0 },
    { 8160, 1018,   27,   27, 1008,    0 },
};
/* Bit-lengths of the codes above (Table 9). Not stored as a lookup table
 * here: every mode's switch-code width equals that mode's fixed content
 * width (13 for Chinese, 10 for Numeral/Mixed's escaped codes, 4 for the
 * initial/Byte switch, 9 for Byte's count field) except Lower/Upper, whose
 * variable-width (5 vs 7 bit) codes are told apart by the 0b11111 prefix
 * -- both handled directly in the per-mode decoders in Section 9 rather
 * than via a redundant length table. */

/* ════════════════════════════════════════════════════════════════════════
 * Section 2: binarization (own local copy -- global Otsu threshold, not
 * shared with qrcode.c/hanxin.c's block-adaptive scheme per the project's
 * one-file-one-copy convention. The test corpus renders a clean synthetic
 * bimodal image, so a single global threshold is sufficient and cheap).
 * ════════════════════════════════════════════════════════════════════════ */

static int gm_otsu_threshold(const Image *img)
{
    long hist[256] = {0};
    long total = (long)img->width * img->height;
    if (total <= 0) return 128;
    for (long i = 0; i < total; i++) hist[img->pixels[i]]++;

    double sum_all = 0;
    for (int i = 0; i < 256; i++) sum_all += (double)i * hist[i];

    double sum_bg = 0;
    long w_bg = 0;
    double best_var = -1;
    int best_t = 128;
    for (int t = 0; t < 256; t++) {
        w_bg += hist[t];
        if (w_bg == 0) continue;
        long w_fg = total - w_bg;
        if (w_fg == 0) break;
        sum_bg += (double)t * hist[t];
        double mean_bg = sum_bg / (double)w_bg;
        double mean_fg = (sum_all - sum_bg) / (double)w_fg;
        double diff = mean_bg - mean_fg;
        double var_between = (double)w_bg * (double)w_fg * diff * diff;
        if (var_between > best_var) {
            best_var = var_between;
            best_t = t;
        }
    }
    return best_t;
}

/* Nearest-pixel dark sample at a float image coordinate. */
static int gm_is_dark(const Image *img, double x, double y, int threshold)
{
    int ix = (int)(x + 0.5);
    int iy = (int)(y + 0.5);
    if (ix < 0) ix = 0;
    if (iy < 0) iy = 0;
    if (ix >= img->width) ix = img->width - 1;
    if (iy >= img->height) iy = img->height - 1;
    return img->pixels[iy * img->width + ix] < threshold;
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 3: connected-component blob detection (8-connectivity, iterative
 * flood fill) to localize barcode candidate(s) within a larger image that
 * may contain unrelated content (confirmed present in the real test image:
 * label text bands outside the barcode's own tight bounding box). The
 * checkerboard frame pattern's dark macromodules touch corner-to-corner,
 * so under 8-connectivity the whole barcode square forms a single blob.
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    int x0, y0, x1, y1; /* inclusive pixel bbox */
    long area;
} GmBlob;

#define GM_MAX_BLOBS 12

static int gm_find_blobs(const Image *img, int threshold, GmBlob *blobs, int max_blobs)
{
    int w = img->width, h = img->height;
    uint8_t *visited = (uint8_t *)calloc((size_t)w * h, 1);
    int *stack = (int *)malloc(sizeof(int) * (size_t)w * h);
    if (!visited || !stack) { free(visited); free(stack); return 0; }

    int nblobs = 0;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            long idx = (long)y * w + x;
            if (visited[idx]) continue;
            visited[idx] = 1;
            if (!gm_is_dark(img, x, y, threshold)) continue;

            /* Flood-fill this component. */
            int sp = 0;
            stack[sp++] = (int)idx;
            int x0 = x, x1 = x, y0 = y, y1 = y;
            long area = 0;
            while (sp > 0) {
                int cur = stack[--sp];
                int cx = cur % w, cy = cur / w;
                area++;
                if (cx < x0) x0 = cx;
                if (cx > x1) x1 = cx;
                if (cy < y0) y0 = cy;
                if (cy > y1) y1 = cy;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = cx + dx, ny = cy + dy;
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                        long nidx = (long)ny * w + nx;
                        if (visited[nidx]) continue;
                        visited[nidx] = 1;
                        if (!gm_is_dark(img, nx, ny, threshold)) continue;
                        stack[sp++] = (int)nidx;
                    }
                }
            }

            int bw = x1 - x0 + 1, bh = y1 - y0 + 1;
            if (bw < 24 || bh < 24) continue; /* too small to be a grid (min size=18 modules) */
            double aspect = (double)bw / bh;
            if (aspect < 0.8 || aspect > 1.25) continue; /* GridMatrix is always square */
            if (nblobs < max_blobs) {
                blobs[nblobs].x0 = x0; blobs[nblobs].y0 = y0;
                blobs[nblobs].x1 = x1; blobs[nblobs].y1 = y1;
                blobs[nblobs].area = area;
                nblobs++;
            }
        }
    }
    free(visited);
    free(stack);
    return nblobs;
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 4: macromodule ring-index mapping.
 * Faithful transcription of zint's gm_macromodule() (gridmtx.c). This
 * ternary is easy to get subtly wrong (an early Python prototype of this
 * exact function mis-split the nested ternary and silently corrupted one
 * macromodule's index) -- kept as a single expression matching the
 * original exactly, not manually restructured.
 * ════════════════════════════════════════════════════════════════════════ */

static int gm_macromodule(int macromodules_per_dim, int x, int y)
{
    int r = x < y ? (x < macromodules_per_dim - y - 1 ? x : macromodules_per_dim - y - 1)
                  : (y < macromodules_per_dim - x - 1 ? y : macromodules_per_dim - x - 1);
    int m = (((macromodules_per_dim - 1) >> 1) - r) * 2 + 1;
    int b = m * m - 1;
    int idx;

    if (x == r) {
        idx = b - y + r;
    } else if (y == r) {
        idx = b - (m - 1) * 4 + (x - r);
    } else if (x == macromodules_per_dim - r - 1) {
        idx = b - (m - 1) * 3 + (y - r);
    } else {
        idx = b - (m - 1) - (x - r);
    }
    return idx << 1; /* 2 codewords per macromodule */
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 5: grid sampling, frame validation, codeword extraction.
 * ════════════════════════════════════════════════════════════════════════ */

#define GM_MAX_GRID (6 + 12 * GM_MAX_LAYERS) /* 162 */
#define GM_MAX_CW   1313                     /* gm_data_cws[12][0]-scale total */

/* Sample the module grid for a given candidate bbox+layers into a dense
 * dark/light bitmap (1 byte per module, 1=dark). Returns 0 on out-of-range. */
static int gm_sample_grid(const Image *img, int threshold,
                           const GmBlob *blob, int layers,
                           uint8_t grid[GM_MAX_GRID][GM_MAX_GRID])
{
    int size = 6 + 12 * layers;
    double bw = (double)(blob->x1 - blob->x0 + 1);
    double bh = (double)(blob->y1 - blob->y0 + 1);
    double pitch_x = bw / size;
    double pitch_y = bh / size;
    if (pitch_x < 2.0 || pitch_y < 2.0) return 0; /* too small to sample reliably */

    for (int r = 0; r < size; r++) {
        double cy = blob->y0 + (r + 0.5) * pitch_y;
        for (int c = 0; c < size; c++) {
            double cx = blob->x0 + (c + 0.5) * pitch_x;
            grid[r][c] = (uint8_t)gm_is_dark(img, cx, cy, threshold);
        }
    }
    return 1;
}

/* Structural gate: every macromodule's 6x6 border ring must be solid-dark
 * (macromodule (x+y) even) or solid-light ((x+y) odd), matching the
 * frame-drawing loop in zint's zint_gridmatrix(). Returns the fraction of
 * mismatched ring cells (0.0 = perfect match). */
static double gm_frame_error(uint8_t grid[GM_MAX_GRID][GM_MAX_GRID], int layers)
{
    int mpd = 1 + 2 * layers;
    long mismatches = 0, total = 0;
    for (int my = 0; my < mpd; my++) {
        for (int mx = 0; mx < mpd; mx++) {
            int dark_expected = ((mx + my) & 1) == 0;
            int base_r = my * 6, base_c = mx * 6;
            for (int i = 0; i < 6; i++) {
                /* top row, bottom row, left col, right col -- full ring */
                int cells[4][2] = {
                    { base_r,     base_c + i },
                    { base_r + 5, base_c + i },
                    { base_r + i, base_c     },
                    { base_r + i, base_c + 5 },
                };
                for (int k = 0; k < 4; k++) {
                    int v = grid[cells[k][0]][cells[k][1]];
                    if (v != dark_expected) mismatches++;
                    total++;
                }
            }
        }
    }
    return total > 0 ? (double)mismatches / (double)total : 1.0;
}

/* Extract the flat, forward-ordered codeword array (index = macromodule
 * pair index, matching gm_place_data_in_grid's cws[] layout) from the
 * sampled grid. */
static void gm_extract_codewords(uint8_t grid[GM_MAX_GRID][GM_MAX_GRID], int layers,
                                  uint8_t *cws, int total_cws)
{
    int size = 6 + 12 * layers;
    (void)size;
    int mpd = 1 + 2 * layers;
    memset(cws, 0, (size_t)total_cws);

    for (int my = 0; my < mpd; my++) {
        for (int mx = 0; mx < mpd; mx++) {
            int base_r = my * 6, base_c = mx * 6;
            unsigned int cw = 0;
            if (grid[base_r + 1][base_c + 3]) cw |= 0x2000;
            if (grid[base_r + 1][base_c + 4]) cw |= 0x1000;
            for (int r = 1; r < 4; r++) {
                for (int c = 0; c < 4; c++) {
                    if (grid[base_r + 1 + r][base_c + 1 + c]) {
                        cw |= 1u << (15 - r * 4 - c);
                    }
                }
            }
            int macromodule = gm_macromodule(mpd, mx, my);
            if (macromodule >= 0 && macromodule + 1 < total_cws) {
                cws[macromodule]     = (uint8_t)(cw & 0x7F);
                cws[macromodule + 1] = (uint8_t)((cw >> 7) & 0x7F);
            }
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 6: GF(128) Reed-Solomon decoder.
 * Own local copy of datamatrix.c's generic GField/rs_decode Berlekamp-
 * Massey + Forney pattern (that pattern is already parametrized by field
 * table/order, so only the field-init constants change here): primitive
 * polynomial 0x89 (x^7+x^3+1), field order 127, generator index 1 --
 * matching zint's zint_rs_init_gf(&rs,0x89) / zint_rs_init_code(&rs,n,1)
 * used by gm_add_ecc() in gridmtx.c. Not shared via a header per the
 * project's one-file-one-copy convention.
 * ════════════════════════════════════════════════════════════════════════ */

static int gf128_exp[254];
static int gf128_log[128];
static int gf128_ready;

static void gf128_init(void)
{
    if (gf128_ready) return;
    int v = 1;
    for (int i = 0; i < 127; i++) {
        gf128_exp[i] = v;
        gf128_log[v] = i;
        v <<= 1;
        if (v & 128) v ^= 0x89;
    }
    for (int i = 127; i < 254; i++) gf128_exp[i] = gf128_exp[i - 127];
    gf128_log[0] = -1;
    gf128_ready = 1;
}

typedef struct {
    int *exp_tbl;
    int *log_tbl;
    int  order;
} GField;

static int gf_mul(const GField *gf, int a, int b)
{
    if (a == 0 || b == 0) return 0;
    return gf->exp_tbl[gf->log_tbl[a] + gf->log_tbl[b]];
}

static int gf_inv(const GField *gf, int a)
{
    if (a == 0) return 0;
    return gf->exp_tbl[gf->order - gf->log_tbl[a]];
}

/* Berlekamp-Massey + Forney RS decoder. cw[0] is the highest-order
 * (first-transmitted) symbol. Returns the number of corrected symbols
 * (>=0) on success, -1 on uncorrectable/detected-but-unlocatable error. */
static int rs_decode(const GField *gf, int *cw, int n, int n_ec)
{
    if (n_ec > 126 || n_ec <= 0 || n > 126) return -1;

    int syn[126];
    int has_error = 0;
    for (int i = 0; i < n_ec; i++) {
        int s = 0;
        int alpha_i = gf->exp_tbl[i + 1];
        int pw = 1;
        for (int j = n - 1; j >= 0; j--) {
            s ^= gf_mul(gf, cw[j], pw);
            pw = gf_mul(gf, pw, alpha_i);
        }
        syn[i] = s;
        if (s != 0) has_error = 1;
    }
    if (!has_error) return 0;

    int sigma[127], sp[127];
    memset(sigma, 0, sizeof(sigma));
    memset(sp, 0, sizeof(sp));
    sigma[0] = 1; sp[0] = 1;
    int L = 0, m = 1, b = 1;

    for (int r = 0; r < n_ec; r++) {
        int d = syn[r];
        for (int j = 1; j <= L; j++)
            d ^= gf_mul(gf, sigma[j], syn[r - j]);
        if (d == 0) { m++; continue; }
        int tmp[127];
        memcpy(tmp, sigma, sizeof(tmp));
        int coeff = gf_mul(gf, d, gf_inv(gf, b));
        for (int j = m; j < n_ec + 1 && j < 127; j++)
            sigma[j] ^= gf_mul(gf, coeff, sp[j - m]);
        if (2 * L <= r) {
            L = r + 1 - L;
            memcpy(sp, tmp, sizeof(sp));
            b = d; m = 1;
        } else { m++; }
    }

    int err_pos[127], ne = 0;
    for (int i = 0; i < n; i++) {
        int v = 0, pw = 1;
        int ai = gf->exp_tbl[gf->order - i];
        for (int j = 0; j <= L; j++) {
            v ^= gf_mul(gf, sigma[j], pw);
            pw = gf_mul(gf, pw, ai);
        }
        if (v == 0) err_pos[ne++] = n - 1 - i;
    }
    if (ne != L) return -1;

    int omega[127];
    memset(omega, 0, sizeof(omega));
    for (int i = 0; i < n_ec; i++) {
        int s = 0;
        for (int j = 0; j <= i && j <= L; j++)
            s ^= gf_mul(gf, sigma[j], syn[i - j]);
        omega[i] = s;
    }

    int sd[127];
    memset(sd, 0, sizeof(sd));
    for (int i = 1; i <= L; i += 2) sd[i - 1] = sigma[i];

    for (int k = 0; k < ne; k++) {
        int pos = err_pos[k];
        int xi = gf->exp_tbl[gf->order - (n - 1 - pos)];
        int ov = 0, pw = 1;
        for (int j = 0; j < n_ec; j++) {
            ov ^= gf_mul(gf, omega[j], pw);
            pw = gf_mul(gf, pw, xi);
        }
        int sv = 0; pw = 1;
        for (int j = 0; j < L; j++) {
            sv ^= gf_mul(gf, sd[j], pw);
            pw = gf_mul(gf, pw, xi);
        }
        if (sv == 0) return -1;
        cw[pos] ^= gf_mul(gf, ov, gf_inv(gf, sv));
    }
    return L;
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 7: block de-interleave + RS correction.
 * Faithful inverse of gm_add_ecc() (gridmtx.c): the flat codeword array is
 * stored round-robin across num_blocks=b1+b2 blocks (cws[num_blocks*j+i]
 * is the j-th codeword of block i); the first b1 blocks have n1 codewords,
 * the rest have n1-1; of all blocks in order, the first b3 use e1 ECC
 * codewords, the rest use e2. On success, corrected *data* codewords are
 * written back concatenated in block order into `data_out`.
 * ════════════════════════════════════════════════════════════════════════ */

static int gm_rs_correct(const uint8_t *cws, int layers, int ecc_level,
                          uint8_t *data_out, int *out_data_len)
{
    int n1 = gm_n1[layers - 1];
    int b1 = gm_b1[layers - 1];
    int n2 = n1 - 1;
    int b2 = gm_b2[layers - 1];
    int e1 = gm_e1b3e2[layers - 1][ecc_level - 1][0];
    int b3 = gm_e1b3e2[layers - 1][ecc_level - 1][1];
    int e2 = gm_e1b3e2[layers - 1][ecc_level - 1][2];
    int num_blocks = b1 + b2;
    if (num_blocks <= 0 || e1 <= 0) return 0; /* invalid/unused table entry */

    gf128_init();
    GField gf = { gf128_exp, gf128_log, 127 };

    int wp = 0;
    for (int i = 0; i < num_blocks; i++) {
        int block_size = i < b1 ? n1 : n2;
        int ecc_size = i < b3 ? e1 : e2;
        int data_size = block_size - ecc_size;
        if (ecc_size <= 0 || data_size <= 0 || block_size > 126) return 0;

        int block[126];
        for (int j = 0; j < block_size; j++) {
            block[j] = cws[num_blocks * j + i];
        }
        int corr = rs_decode(&gf, block, block_size, ecc_size);
        if (corr < 0) return 0;

        for (int j = 0; j < data_size; j++) {
            if (wp >= GM_MAX_CW) return 0;
            data_out[wp++] = (uint8_t)block[j];
        }
    }
    /* Defensive cross-check: the block layout tables (n1/b1/n2/b2/e1/b3/e2)
     * and the max-data-codewords table (gm_data_cws) are transcribed from
     * two different columns of the same AIMD014 source table (A.1 vs
     * Table 1) -- they must agree on the total data codeword count for
     * every (layers, ecc_level). A mismatch here means a transcription
     * error in one of the tables, not a bad sample; reject rather than
     * risk decoding garbage against inconsistent tables. */
    if (wp != gm_data_cws[layers - 1][ecc_level - 1]) return 0;

    *out_data_len = wp;
    return 1;
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 8: bitstream reader + NUMERAL mode decoder (fully implemented
 * and validated against the real test sample + a zint-generated reference
 * image; the "12345678" bitstream was decoded byte-for-byte by hand before
 * this code was written).
 * ════════════════════════════════════════════════════════════════════════ */

/* Read `n` bits (n<=16) MSB-first starting at bit offset `pos` from a
 * 7-bit-per-codeword data array. Returns -1 if the read would run past
 * total_bits. */
static int gm_get_bits(const uint8_t *data_cws, int total_bits, int pos, int n)
{
    if (pos < 0 || n < 0 || pos + n > total_bits) return -1;
    int v = 0;
    for (int i = 0; i < n; i++) {
        int bitpos = pos + i;
        int cw = data_cws[bitpos / 7];
        int bit = (cw >> (6 - (bitpos % 7))) & 1;
        v = (v << 1) | bit;
    }
    return v;
}

/* AIMD014 6.3.3 non-digit set used inside NUMERAL mode's combo codes. */
static const char gm_numeral_nondigits[] = " +-.,";

static void gm_append_char(char *out, int *outlen, int cap, char c)
{
    if (*outlen < cap - 1) out[(*outlen)++] = c;
}

/* Decodes one NUMERAL-mode run (from just after the mode-switch code that
 * entered NUMERAL, through to the in-band switch-out/EOD code). Leaves
 * *bitpos positioned after the switch-out code; *next_mode receives the
 * target mode (GM_EOD if the message ends here). Returns 0 on success. */
static int gm_decode_numeral(const uint8_t *data_cws, int total_bits, int *bitpos,
                              char *out, int *outlen, int cap, int *next_mode)
{
    int pad = gm_get_bits(data_cws, total_bits, *bitpos, 2);
    if (pad < 0) return -1;
    *bitpos += 2;

    char pending[3] = {0, 0, 0};
    int have_pending = 0;

    for (;;) {
        int g = gm_get_bits(data_cws, total_bits, *bitpos, 10);
        if (g < 0) return -1;
        *bitpos += 10;

        if (g <= 999) {
            if (have_pending) {
                gm_append_char(out, outlen, cap, pending[0]);
                gm_append_char(out, outlen, cap, pending[1]);
                gm_append_char(out, outlen, cap, pending[2]);
            }
            pending[0] = (char)('0' + (g / 100));
            pending[1] = (char)('0' + ((g / 10) % 10));
            pending[2] = (char)('0' + (g % 10));
            have_pending = 1;
        } else if (g >= 1000 && g <= 1015) {
            /* Non-digit combo code: flush the pending triple (always full
             * here, since a combo code cannot itself be the very last
             * token counted by the pad field), then emit the punctuation
             * character (or CR/LF for g==1015) at its recorded position.
             * Best-effort: not exercised by the numeric-only test corpus. */
            if (have_pending) {
                gm_append_char(out, outlen, cap, pending[0]);
                gm_append_char(out, outlen, cap, pending[1]);
                gm_append_char(out, outlen, cap, pending[2]);
                have_pending = 0;
            }
            int rel = g - 1000;
            if (rel == 15) {
                gm_append_char(out, outlen, cap, '\r');
                gm_append_char(out, outlen, cap, '\n');
            } else {
                gm_append_char(out, outlen, cap, gm_numeral_nondigits[rel / 3]);
            }
        } else if (g == gm_mode_switch[GM_EOD][GM_NUMERAL - 1]) {
            if (have_pending) {
                int keep = 3 - pad;
                for (int i = 0; i < keep && i < 3; i++) gm_append_char(out, outlen, cap, pending[i]);
            }
            *next_mode = GM_EOD;
            return 0;
        } else if (g == gm_mode_switch[GM_NUMERAL][GM_CHINESE - 1] ||
                   g == gm_mode_switch[GM_NUMERAL][GM_LOWER - 1] ||
                   g == gm_mode_switch[GM_NUMERAL][GM_UPPER - 1] ||
                   g == gm_mode_switch[GM_NUMERAL][GM_MIXED - 1] ||
                   g == gm_mode_switch[GM_NUMERAL][GM_BYTE - 1]) {
            if (have_pending) {
                int keep = 3 - pad;
                for (int i = 0; i < keep && i < 3; i++) gm_append_char(out, outlen, cap, pending[i]);
            }
            if (g == gm_mode_switch[GM_NUMERAL][GM_CHINESE - 1]) *next_mode = GM_CHINESE;
            else if (g == gm_mode_switch[GM_NUMERAL][GM_LOWER - 1]) *next_mode = GM_LOWER;
            else if (g == gm_mode_switch[GM_NUMERAL][GM_UPPER - 1]) *next_mode = GM_UPPER;
            else if (g == gm_mode_switch[GM_NUMERAL][GM_MIXED - 1]) *next_mode = GM_MIXED;
            else *next_mode = GM_BYTE;
            return 0;
        } else {
            return -1; /* reserved/invalid code */
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 9: other five modes -- best-effort structural implementation
 * mirroring zint's encoder bit assignment logic in reverse. NOT validated
 * against any real sample (none exist in the current test corpus). Byte
 * mode is limited to single-byte output (GB2312 double-byte splitting in
 * Chinese mode is emitted as two raw bytes, not converted to UTF-8).
 * ════════════════════════════════════════════════════════════════════════ */

static int gm_decode_chinese(const uint8_t *data_cws, int total_bits, int *bitpos,
                              char *out, int *outlen, int cap, int *next_mode)
{
    for (;;) {
        int g = gm_get_bits(data_cws, total_bits, *bitpos, 13);
        if (g < 0) return -1;
        *bitpos += 13;

        if (g == gm_mode_switch[GM_EOD][GM_CHINESE - 1]) { *next_mode = GM_EOD; return 0; }
        if (g == gm_mode_switch[GM_CHINESE][GM_NUMERAL - 1]) { *next_mode = GM_NUMERAL; return 0; }
        if (g == gm_mode_switch[GM_CHINESE][GM_LOWER - 1])   { *next_mode = GM_LOWER;   return 0; }
        if (g == gm_mode_switch[GM_CHINESE][GM_UPPER - 1])   { *next_mode = GM_UPPER;   return 0; }
        if (g == gm_mode_switch[GM_CHINESE][GM_MIXED - 1])   { *next_mode = GM_MIXED;   return 0; }
        if (g == gm_mode_switch[GM_CHINESE][GM_BYTE - 1])    { *next_mode = GM_BYTE;    return 0; }

        if (g == 7776) {
            gm_append_char(out, outlen, cap, '\r');
            gm_append_char(out, outlen, cap, '\n');
        } else if (g >= 8033 && g <= 8132) {
            int v = g - 8033;
            gm_append_char(out, outlen, cap, (char)('0' + v / 10));
            gm_append_char(out, outlen, cap, (char)('0' + v % 10));
        } else if (g >= 7777 && g <= 8032) {
            gm_append_char(out, outlen, cap, (char)(g - 7777));
        } else if (g < 7776) {
            /* GB2312 two-byte glyph -- reconstruct raw GB2312 bytes
             * (inverse of the c1/c2 packing in gm_encode's GM_CHINESE
             * case); left as raw bytes rather than converted to UTF-8. */
            int c1, c2;
            if (g < 0x60 * 9) {
                c1 = g / 0x60 + 0xA1;
                c2 = g % 0x60 + 0xA0;
            } else {
                int gg = g - 0x60 * 9;
                c1 = gg / 0x60 + 0xB0;
                c2 = gg % 0x60 + 0xA0;
            }
            gm_append_char(out, outlen, cap, (char)c1);
            gm_append_char(out, outlen, cap, (char)c2);
        } else {
            return -1;
        }
    }
}

static int gm_decode_shift_char(const uint8_t *data_cws, int total_bits, int *bitpos,
                                 char *out, int *outlen, int cap)
{
    int g = gm_get_bits(data_cws, total_bits, *bitpos, 6);
    if (g < 0) return -1;
    *bitpos += 6;
    /* Inverse of gm_add_shift_char's Table 7 control-character mapping. */
    int ch;
    if (g <= 30) ch = g;               /* < ' ' */
    else if (g <= 46) ch = g + 1;       /* < '0' */
    else if (g <= 56) ch = g + 11;      /* < 'A' */
    else if (g <= 79) ch = g + 46;      /* < 'a' */
    else ch = g + 63;
    gm_append_char(out, outlen, cap, (char)ch);
    return 0;
}

static int gm_decode_upper_lower(const uint8_t *data_cws, int total_bits, int *bitpos,
                                  char *out, int *outlen, int cap, int is_upper, int *next_mode)
{
    int self = is_upper ? GM_UPPER : GM_LOWER;
    for (;;) {
        int g = gm_get_bits(data_cws, total_bits, *bitpos, 5);
        if (g < 0) return -1;

        if (g == 31) {
            /* 7-bit escape: {0x7C=MXD, 0x7D=SHIFT, 0x7E=BYT} share the
             * 5-bit prefix 0b11111; distinguish via 2 more bits. */
            int g7 = gm_get_bits(data_cws, total_bits, *bitpos, 7);
            if (g7 < 0) return -1;
            if (g7 == gm_mode_switch[self][GM_MIXED - 1]) { *bitpos += 7; *next_mode = GM_MIXED; return 0; }
            if (g7 == 125) {
                *bitpos += 7;
                if (gm_decode_shift_char(data_cws, total_bits, bitpos, out, outlen, cap) != 0) return -1;
                continue; /* stay in this mode, resume 5-bit reads */
            }
            if (g7 == gm_mode_switch[self][GM_BYTE - 1]) { *bitpos += 7; *next_mode = GM_BYTE; return 0; }
            return -1;
        }
        *bitpos += 5;
        if (g <= 26) {
            gm_append_char(out, outlen, cap, g == 26 ? ' ' : (char)((is_upper ? 'A' : 'a') + g));
        } else if (g == gm_mode_switch[self][GM_CHINESE - 1]) {
            *next_mode = GM_CHINESE; return 0;
        } else if (g == gm_mode_switch[self][GM_NUMERAL - 1]) {
            *next_mode = GM_NUMERAL; return 0;
        } else if (is_upper && g == gm_mode_switch[self][GM_LOWER - 1]) {
            *next_mode = GM_LOWER; return 0;
        } else if (!is_upper && g == gm_mode_switch[self][GM_UPPER - 1]) {
            *next_mode = GM_UPPER; return 0;
        } else {
            return -1;
        }
    }
}

static int gm_decode_mixed(const uint8_t *data_cws, int total_bits, int *bitpos,
                            char *out, int *outlen, int cap, int *next_mode)
{
    for (;;) {
        int g = gm_get_bits(data_cws, total_bits, *bitpos, 6);
        if (g < 0) return -1;

        if (g == 63) {
            /* 10-bit escape: 6-bit prefix 0b111111 + 4 more bits. */
            int g10 = gm_get_bits(data_cws, total_bits, *bitpos, 10);
            if (g10 < 0) return -1;
            *bitpos += 10;
            if (g10 == gm_mode_switch[GM_EOD][GM_MIXED - 1]) { *next_mode = GM_EOD; return 0; }
            if (g10 == 1014) { /* shift indicator */
                if (gm_decode_shift_char(data_cws, total_bits, bitpos, out, outlen, cap) != 0) return -1;
                continue;
            }
            if (g10 == gm_mode_switch[GM_MIXED][GM_CHINESE - 1]) { *next_mode = GM_CHINESE; return 0; }
            if (g10 == gm_mode_switch[GM_MIXED][GM_NUMERAL - 1]) { *next_mode = GM_NUMERAL; return 0; }
            if (g10 == gm_mode_switch[GM_MIXED][GM_LOWER - 1])   { *next_mode = GM_LOWER;   return 0; }
            if (g10 == gm_mode_switch[GM_MIXED][GM_UPPER - 1])   { *next_mode = GM_UPPER;   return 0; }
            if (g10 == gm_mode_switch[GM_MIXED][GM_BYTE - 1])    { *next_mode = GM_BYTE;    return 0; }
            return -1;
        }
        *bitpos += 6;
        char ch;
        if (g <= 9) ch = (char)('0' + g);
        else if (g <= 35) ch = (char)('A' + (g - 10));
        else if (g <= 61) ch = (char)('a' + (g - 36));
        else ch = ' '; /* g == 62 */
        gm_append_char(out, outlen, cap, ch);
    }
}

static int gm_decode_byte(const uint8_t *data_cws, int total_bits, int *bitpos,
                           char *out, int *outlen, int cap, int *next_mode)
{
    for (;;) {
        int count9 = gm_get_bits(data_cws, total_bits, *bitpos, 9);
        if (count9 < 0) return -1;
        *bitpos += 9;
        int nbytes = count9 + 1; /* AIMD014 6.3.7: prefix stores L-1 */
        for (int i = 0; i < nbytes; i++) {
            int b = gm_get_bits(data_cws, total_bits, *bitpos, 8);
            if (b < 0) return -1;
            *bitpos += 8;
            gm_append_char(out, outlen, cap, (char)b);
        }
        /* A block-continuation escape (nested 4-bit code 7 mid-stream) is
         * used by the encoder only for >511-byte blocks; not needed for
         * this test corpus and left unimplemented -- fall through to read
         * the switch-out code that must follow every BYTE block. */
        int g = gm_get_bits(data_cws, total_bits, *bitpos, 4);
        if (g < 0) return -1;
        *bitpos += 4;
        if (g == gm_mode_switch[GM_EOD][GM_BYTE - 1]) { *next_mode = GM_EOD; return 0; }
        if (g == gm_mode_switch[GM_BYTE][GM_CHINESE - 1]) { *next_mode = GM_CHINESE; return 0; }
        if (g == gm_mode_switch[GM_BYTE][GM_NUMERAL - 1]) { *next_mode = GM_NUMERAL; return 0; }
        if (g == gm_mode_switch[GM_BYTE][GM_LOWER - 1])   { *next_mode = GM_LOWER;   return 0; }
        if (g == gm_mode_switch[GM_BYTE][GM_UPPER - 1])   { *next_mode = GM_UPPER;   return 0; }
        if (g == gm_mode_switch[GM_BYTE][GM_MIXED - 1])   { *next_mode = GM_MIXED;   return 0; }
        return -1;
    }
}

/* Top-level bitstream decoder: reads the initial 4-bit mode-switch code,
 * then dispatches to the per-mode content loop until GM_EOD. */
static int gm_decode_bitstream(const uint8_t *data_cws, int num_data_cws, char *out, int cap)
{
    int total_bits = num_data_cws * 7;
    int bitpos = 0;
    int outlen = 0;
    int mode = 0;
    int guard = 0;

    while (mode != GM_EOD) {
        if (++guard > 2000) return -1; /* runaway-stream guard */

        if (mode == 0) {
            int v = gm_get_bits(data_cws, total_bits, bitpos, 4);
            if (v < 0) return -1;
            bitpos += 4;
            if (v == gm_mode_switch[0][GM_CHINESE - 1]) mode = GM_CHINESE;
            else if (v == gm_mode_switch[0][GM_NUMERAL - 1]) mode = GM_NUMERAL;
            else if (v == gm_mode_switch[0][GM_LOWER - 1]) mode = GM_LOWER;
            else if (v == gm_mode_switch[0][GM_UPPER - 1]) mode = GM_UPPER;
            else if (v == gm_mode_switch[0][GM_MIXED - 1]) mode = GM_MIXED;
            else if (v == gm_mode_switch[0][GM_BYTE - 1]) mode = GM_BYTE;
            else return -1;
            continue;
        }

        int next_mode = 0;
        int rc;
        switch (mode) {
            case GM_NUMERAL: rc = gm_decode_numeral(data_cws, total_bits, &bitpos, out, &outlen, cap, &next_mode); break;
            case GM_CHINESE: rc = gm_decode_chinese(data_cws, total_bits, &bitpos, out, &outlen, cap, &next_mode); break;
            case GM_LOWER:   rc = gm_decode_upper_lower(data_cws, total_bits, &bitpos, out, &outlen, cap, 0, &next_mode); break;
            case GM_UPPER:   rc = gm_decode_upper_lower(data_cws, total_bits, &bitpos, out, &outlen, cap, 1, &next_mode); break;
            case GM_MIXED:   rc = gm_decode_mixed(data_cws, total_bits, &bitpos, out, &outlen, cap, &next_mode); break;
            case GM_BYTE:    rc = gm_decode_byte(data_cws, total_bits, &bitpos, out, &outlen, cap, &next_mode); break;
            default: return -1;
        }
        if (rc < 0) return -1;
        mode = next_mode;
    }

    out[outlen < cap ? outlen : cap - 1] = '\0';
    return outlen;
}

/* ════════════════════════════════════════════════════════════════════════
 * Section 10: top-level orchestration.
 * ════════════════════════════════════════════════════════════════════════ */

static int gm_try_decode_at(const Image *img, int threshold, const GmBlob *blob, BarcodeResult *result)
{
    static uint8_t grid[GM_MAX_GRID][GM_MAX_GRID];
    static uint8_t cws[GM_MAX_CW];
    static uint8_t data_cws[GM_MAX_CW];

    for (int layers = 1; layers <= GM_MAX_LAYERS; layers++) {
        if (!gm_sample_grid(img, threshold, blob, layers, grid)) continue;

        /* Gate 1: structural frame checkerboard match. */
        double err = gm_frame_error(grid, layers);
        if (err > 0.03) continue;

        int mpd = 1 + 2 * layers;
        int total_cws = mpd * mpd * 2;
        if (total_cws > GM_MAX_CW) continue;
        gm_extract_codewords(grid, layers, cws, total_cws);

        /* Gate 2: RS decode must succeed cleanly for some ECC level. Try
         * levels from highest (most ECC, most likely to be the one used
         * for small messages -- matches the "12345678" reference, which
         * used layers=1/ecc=5) down to lowest. */
        for (int ecc_level = 5; ecc_level >= 1; ecc_level--) {
            if (layers == 1 && ecc_level == 1) continue; /* invalid combo */
            int data_len = 0;
            if (!gm_rs_correct(cws, layers, ecc_level, data_cws, &data_len)) continue;
            if (data_len <= 0) continue;

            char text[BARCODE_MAX_LEN];
            int n = gm_decode_bitstream(data_cws, data_len, text, (int)sizeof(text));
            if (n <= 0) continue;

            memset(result, 0, sizeof(*result));
            int copy_len = n < BARCODE_MAX_LEN - 1 ? n : BARCODE_MAX_LEN - 1;
            memcpy(result->data, text, (size_t)copy_len);
            result->data[copy_len] = '\0';
            result->length = copy_len;
            result->confidence = 90;
            result->symbology = BARCODE_SYM_GRIDMATRIX;
            return 1;
        }
    }
    return 0;
}

int gridmatrix_decode(const Image *img, BarcodeResult *result)
{
    if (result) memset(result, 0, sizeof(*result));
    if (!img || !img->pixels || img->width < 24 || img->height < 24 || !result) return 0;

    int threshold = gm_otsu_threshold(img);

    static GmBlob blobs[GM_MAX_BLOBS];
    int nblobs = gm_find_blobs(img, threshold, blobs, GM_MAX_BLOBS);

    for (int i = 0; i < nblobs; i++) {
        if (gm_try_decode_at(img, threshold, &blobs[i], result)) return 1;
    }
    return 0;
}
