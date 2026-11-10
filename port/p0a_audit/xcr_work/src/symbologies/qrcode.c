#include "qrcode.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ════════════════════════════════════════════════════════════════════════
 *  QR Code constants
 * ════════════════════════════════════════════════════════════════════════ */

/* Set to 1 to enable QR debug prints (very verbose) */
#define QR_DEBUG_PRINT 0
#if QR_DEBUG_PRINT
#define QR_DBG(...) fprintf(stderr, __VA_ARGS__)
#else
#define QR_DBG(...) ((void)0)
#endif

#define QR_MAX_VERSION   40
#define QR_MAX_MODULES   (4 * QR_MAX_VERSION + 17)  /* 177 for V40 */
#define QR_MAX_CODEWORDS 2956                        /* V40 total */
#define QR_MAX_DATA_BITS (QR_MAX_CODEWORDS * 8)

/* Finder pattern ratio 1:1:3:1:1 */
#define FINDER_RATIO_TOTAL 7

/* ════════════════════════════════════════════════════════════════════════
 *  GF(256) arithmetic for Reed-Solomon
 * ════════════════════════════════════════════════════════════════════════ */

static uint8_t gf_exp[512];
static uint8_t gf_log[256];
static int     gf_init_done;
static QRDecodeDebugStats g_qr_debug_stats;

static void gf_init(void)
{
    if (gf_init_done) return;
    int x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) x ^= 0x11D; /* QR primitive polynomial */
    }
    for (int i = 255; i < 512; i++) {
        gf_exp[i] = gf_exp[i - 255];
    }
    gf_log[0] = 0; /* undefined, but avoids garbage */
    gf_init_done = 1;
}

static uint8_t gf_mul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0) return 0;
    return gf_exp[gf_log[a] + gf_log[b]];
}

static uint8_t gf_div(uint8_t a, uint8_t b)
{
    if (b == 0) return 0;
    if (a == 0) return 0;
    return gf_exp[(gf_log[a] + 255 - gf_log[b]) % 255];
}

/* ════════════════════════════════════════════════════════════════════════
 *  Reed-Solomon error correction
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    uint8_t syndromes[256];
    uint8_t sigma[256];
    uint8_t sigma_old[256];
    uint8_t tmp[256];
    int error_locs[256];
} QrRsWorkspace;

static int rs_correct(uint8_t *data, int total, int ecc_count)
{
    QrRsWorkspace workspace = {0};
    int has_error = 0;

    for (int i = 0; i < ecc_count; i++) {
        uint8_t s = 0;
        for (int j = 0; j < total; j++) {
            s = gf_mul(s, gf_exp[i]) ^ data[j];
        }
        workspace.syndromes[i] = s;
        if (s) has_error = 1;
    }

    if (!has_error) {
        return 1; /* no errors */
    }

    { int ne = 0; for (int i = 0; i < ecc_count; i++) if (workspace.syndromes[i]) ne++;
      QR_DBG("[QRRS] ecc_count=%d total=%d, %d nonzero syndromes\n", ecc_count, total, ne);
    }

    /* Berlekamp-Massey */
    workspace.sigma[0] = 1;
    workspace.sigma_old[0] = 1;
    int L = 0;
    int m = 1;
    uint8_t b = 1;

    for (int n = 0; n < ecc_count; n++) {
        uint8_t delta = workspace.syndromes[n];
        for (int j = 1; j <= L; j++) {
            delta ^= gf_mul(workspace.sigma[j], workspace.syndromes[n - j]);
        }

        if (delta == 0) {
            m++;
        } else if (2 * L <= n) {
            memcpy(workspace.tmp, workspace.sigma, sizeof(workspace.sigma));
            uint8_t factor = gf_div(delta, b);
            for (int j = m; j < ecc_count + 1; j++) {
                workspace.sigma[j] ^= gf_mul(factor, workspace.sigma_old[j - m]);
            }
            memcpy(workspace.sigma_old, workspace.tmp, sizeof(workspace.sigma_old));
            L = n + 1 - L;
            b = delta;
            m = 1;
        } else {
            uint8_t factor = gf_div(delta, b);
            for (int j = m; j < ecc_count + 1; j++) {
                workspace.sigma[j] ^= gf_mul(factor, workspace.sigma_old[j - m]);
            }
            m++;
        }
    }

    if (L > ecc_count / 2) {
        QR_DBG("[QRDEBUG RS] Too many errors: L=%d, ecc_count=%d, max=%d\n",
                L, ecc_count, ecc_count / 2);
        return 0; /* too many errors */
    }

    /* Chien search for error locations */
    int n_errors = 0;

    for (int i = 0; i < total; i++) {
        uint8_t val = 1;
        uint8_t xi = gf_exp[255 - i];
        uint8_t xi_pow = 1;
        for (int j = 1; j <= L; j++) {
            xi_pow = gf_mul(xi_pow, xi);
            val ^= gf_mul(workspace.sigma[j], xi_pow);
        }
        if (val == 0) {
            workspace.error_locs[n_errors++] = total - 1 - i;
        }
    }

    if (n_errors != L) {
        QR_DBG("[QRDEBUG RS] Couldn't find all roots: found %d, expected %d\n", n_errors, L);
        return 0; /* couldn't find all roots */
    }

    /* Forney algorithm for error values */
    for (int k = 0; k < n_errors; k++) {
        int pos = workspace.error_locs[k];
        if (pos < 0 || pos >= total) {
            return 0;
        }

        uint8_t xi = gf_exp[255 - (total - 1 - pos)];
        uint8_t xi_inv = gf_exp[total - 1 - pos];

        /* Evaluate omega(xi_inv) = sum S[j] * sigma[i] where j+i=index */
        uint8_t omega = 0;
        {
            uint8_t xp = 1;
            for (int j = 0; j < ecc_count; j++) {
                uint8_t coeff = 0;
                for (int i = 0; i <= j && i <= L; i++) {
                    coeff ^= gf_mul(workspace.sigma[i], workspace.syndromes[j - i]);
                }
                omega ^= gf_mul(coeff, xp);
                xp = gf_mul(xp, xi_inv);
            }
        }

        /* Evaluate sigma'(xi_inv) */
        uint8_t sigma_prime = 0;
        {
            uint8_t xp = 1;
            for (int j = 1; j <= L; j += 2) {
                sigma_prime ^= gf_mul(workspace.sigma[j], xp);
                xp = gf_mul(xp, gf_mul(xi_inv, xi_inv));
            }
        }

        if (sigma_prime == 0) {
            QR_DBG("[QRDEBUG RS] sigma_prime is zero at pos %d\n", pos);
            return 0;
        }
        data[pos] ^= gf_div(gf_mul(omega, xi), sigma_prime);
    }

    /* Verify corrected */
    for (int i = 0; i < ecc_count; i++) {
        uint8_t s = 0;
        for (int j = 0; j < total; j++) {
            s = gf_mul(s, gf_exp[i]) ^ data[j];
        }
        if (s) {
            QR_DBG("[QRDEBUG RS] Verification failed at syndrome %d\n", i);
            return 0;
        }
    }

    QR_DBG("[QRDEBUG RS] RS correction succeeded!\n");
    return 1;
}

/* ════════════════════════════════════════════════════════════════════════
 *  QR version / format info tables
 * ════════════════════════════════════════════════════════════════════════ */

/* Error correction level: 0=M, 1=L, 2=H, 3=Q  (format info encoding) */
#define EC_L 1
#define EC_M 0
#define EC_Q 3
#define EC_H 2

typedef struct {
    int total_codewords;
    int ec_codewords_per_block;
    int num_blocks_g1;
    int data_cw_per_block_g1;
    int num_blocks_g2;
    int data_cw_per_block_g2;
} ECInfo;

/* EC info table per ISO 18004.  Column order: [M, L, H, Q]
 * matching ec_level_to_col() mapping: 0→M, 1→L, 2→H, 3→Q.
 * {total_cw, ec_per_block, blocks_g1, data_g1, blocks_g2, data_g2} */
static const ECInfo ec_table[40][4] = {
    /* V1  */ {{ 26,10,1,16,0,0},{ 26, 7,1,19,0,0},{ 26,17,1, 9,0,0},{ 26,13,1,13,0,0}},
    /* V2  */ {{ 44,16,1,28,0,0},{ 44,10,1,34,0,0},{ 44,28,1,16,0,0},{ 44,22,1,22,0,0}},
    /* V3  */ {{ 70,26,1,44,0,0},{ 70,15,1,55,0,0},{ 70,22,2,13,0,0},{ 70,18,2,17,0,0}},
    /* V4  */ {{100,18,2,32,0,0},{100,20,1,80,0,0},{100,16,4, 9,0,0},{100,26,2,24,0,0}},
    /* V5  */ {{134,24,2,43,0,0},{134,26,1,108,0,0},{134,22,2,11,2,12},{134,18,2,15,2,16}},
    /* V6  */ {{172,16,4,27,0,0},{172,18,2,68,0,0},{172,28,4,15,0,0},{172,24,4,19,0,0}},
    /* V7  */ {{196,18,4,31,0,0},{196,20,2,78,0,0},{196,26,4,13,1,14},{196,18,2,14,4,18}},
    /* V8  */ {{242,22,2,38,2,39},{242,24,2,97,0,0},{242,26,4,14,2,15},{242,22,4,18,2,19}},
    /* V9  */ {{292,22,3,36,2,37},{292,30,2,116,0,0},{292,24,4,12,4,13},{292,20,4,16,4,17}},
    /* V10 */ {{346,26,4,43,1,44},{346,18,2,68,2,69},{346,28,6,15,2,16},{346,24,6,19,2,20}},
    /* V11 */ {{404,30,1,50,4,51},{404,20,4,81,0,0},{404,24,3,12,8,13},{404,28,4,22,4,23}},
    /* V12 */ {{466,22,6,36,2,37},{466,24,2,92,2,93},{466,28,7,14,4,15},{466,26,4,20,6,21}},
    /* V13 */ {{532,22,8,37,1,38},{532,26,4,107,0,0},{532,22,12,11,4,12},{532,24,8,20,4,21}},
    /* V14 */ {{581,24,4,40,5,41},{581,30,3,115,1,116},{581,24,11,12,5,13},{581,20,11,16,5,17}},
    /* V15 */ {{655,24,5,41,5,42},{655,22,5,87,1,88},{655,24,11,12,7,13},{655,30,5,24,7,25}},
    /* V16 */ {{733,28,7,45,3,46},{733,24,5,98,1,99},{733,30,3,15,13,16},{733,24,15,19,2,20}},
    /* V17 */ {{815,28,10,46,1,47},{815,28,1,107,5,108},{815,28,2,14,17,15},{815,28,1,22,15,23}},
    /* V18 */ {{901,26,9,43,4,44},{901,30,5,120,1,121},{901,28,2,14,19,15},{901,28,17,22,1,23}},
    /* V19 */ {{991,26,3,44,11,45},{991,28,3,113,4,114},{991,26,9,13,16,14},{991,26,17,21,4,22}},
    /* V20 */ {{1085,26,3,41,13,42},{1085,28,3,107,5,108},{1085,28,15,15,10,16},{1085,30,15,24,5,25}},
    /* V21 */ {{1156,26,17,42,0,0},{1156,28,4,116,4,117},{1156,30,19,16,6,17},{1156,28,17,22,6,23}},
    /* V22 */ {{1258,28,17,46,0,0},{1258,28,2,111,7,112},{1258,24,34,13,0,0},{1258,30,7,24,16,25}},
    /* V23 */ {{1364,28,4,47,14,48},{1364,30,4,121,5,122},{1364,30,16,15,14,16},{1364,30,11,24,14,25}},
    /* V24 */ {{1474,28,6,45,14,46},{1474,30,6,117,4,118},{1474,30,30,16,2,17},{1474,30,11,24,16,25}},
    /* V25 */ {{1588,28,8,47,13,48},{1588,26,8,106,4,107},{1588,30,22,15,13,16},{1588,30,7,24,22,25}},
    /* V26 */ {{1706,28,10,46,15,47},{1706,28,19,114,4,115},{1706,28,33,16,4,17},{1706,30,28,22,6,23}},
    /* V27 */ {{1828,28,8,45,21,46},{1828,30,22,122,3,123},{1828,30,12,15,28,16},{1828,30,8,23,26,24}},
    /* V28 */ {{1921,28,11,45,19,46},{1921,30,3,117,10,118},{1921,30,11,15,31,16},{1921,30,4,24,31,25}},
    /* V29 */ {{2051,28,7,45,23,46},{2051,30,21,116,7,117},{2051,30,19,15,26,16},{2051,30,1,23,37,24}},
    /* V30 */ {{2185,28,5,45,25,46},{2185,30,19,118,10,119},{2185,30,23,15,25,16},{2185,30,15,24,25,25}},
    /* V31 */ {{2323,28,13,45,22,46},{2323,30,2,116,29,117},{2323,30,23,15,28,16},{2323,30,42,24,1,25}},
    /* V32 */ {{2465,28,17,45,10,46},{2465,30,10,115,23,116},{2465,30,19,15,35,16},{2465,30,10,24,35,25}},
    /* V33 */ {{2611,28,17,45,14,46},{2611,30,14,115,21,116},{2611,30,11,15,46,16},{2611,30,29,24,19,25}},
    /* V34 */ {{2761,28,13,45,32,46},{2761,30,14,115,23,116},{2761,30,59,16,1,17},{2761,30,44,24,7,25}},
    /* V35 */ {{2876,28,12,47,26,48},{2876,30,12,121,26,122},{2876,30,22,15,41,16},{2876,30,39,24,14,25}},
    /* V36 */ {{3034,28,6,47,34,48},{3034,30,6,121,34,122},{3034,30,2,15,64,16},{3034,30,46,24,10,25}},
    /* V37 */ {{3196,28,29,46,14,47},{3196,30,29,122,14,123},{3196,30,24,15,46,16},{3196,30,49,24,10,25}},
    /* V38 */ {{3362,28,13,46,32,47},{3362,30,13,122,32,123},{3362,30,42,15,32,16},{3362,30,48,24,14,25}},
    /* V39 */ {{3532,28,40,47,7,48},{3532,30,40,117,7,118},{3532,30,10,15,67,16},{3532,30,43,24,22,25}},
    /* V40 */ {{3706,28,18,47,31,48},{3706,30,18,118,31,119},{3706,30,20,15,61,16},{3706,30,34,24,34,25}},
};

/* EC level index: format bits -> table column
   Format info encodes EC as: 00=M, 01=L, 10=H, 11=Q */
static int ec_level_to_col(int ec_bits)
{
    switch (ec_bits) {
    case 0: return 0; /* M */
    case 1: return 1; /* L */
    case 2: return 2; /* H */
    case 3: return 3; /* Q */
    }
    return 0;
}

/* Alignment pattern center positions per version */
static const uint8_t align_pos[40][8] = {
    {0},                           /* V1: none */
    {6,18,0},                      /* V2 */
    {6,22,0},                      /* V3 */
    {6,26,0},                      /* V4 */
    {6,30,0},                      /* V5 */
    {6,34,0},                      /* V6 */
    {6,22,38,0},                   /* V7 */
    {6,24,42,0},                   /* V8 */
    {6,26,46,0},                   /* V9 */
    {6,28,50,0},                   /* V10 */
    {6,30,54,0},                   /* V11 */
    {6,32,58,0},                   /* V12 */
    {6,34,62,0},                   /* V13 */
    {6,26,46,66,0},                /* V14 */
    {6,26,48,70,0},                /* V15 */
    {6,26,50,74,0},                /* V16 */
    {6,30,54,78,0},                /* V17 */
    {6,30,56,82,0},                /* V18 */
    {6,30,58,86,0},                /* V19 */
    {6,34,62,90,0},                /* V20 */
    {6,28,50,72,94,0},             /* V21 */
    {6,26,50,74,98,0},             /* V22 */
    {6,30,54,78,102,0},            /* V23 */
    {6,28,54,80,106,0},            /* V24 */
    {6,32,58,84,110,0},            /* V25 */
    {6,30,58,86,114,0},            /* V26 */
    {6,34,62,90,118,0},            /* V27 */
    {6,26,50,74,98,122,0},         /* V28 */
    {6,30,54,78,102,126,0},        /* V29 */
    {6,26,52,78,104,130,0},        /* V30 */
    {6,30,56,82,108,134,0},        /* V31 */
    {6,34,60,86,112,138,0},        /* V32 */
    {6,30,58,86,114,142,0},        /* V33 */
    {6,34,62,90,118,146,0},        /* V34 */
    {6,30,54,78,102,126,150,0},    /* V35 - note: 8 entries but we only store 7 */
    {6,24,50,76,102,128,154,0},    /* V36 */
    {6,28,54,80,106,132,158,0},    /* V37 */
    {6,32,58,84,110,136,162,0},    /* V38 */
    {6,26,54,82,110,138,166,0},    /* V39 */
    {6,30,58,86,114,142,170,0},    /* V40 */
};

static int align_count(int version)
{
    if (version < 2) return 0;
    return (version / 7) + 2;
}

/* Version info bits for V7-V40 (18-bit values) */
static const uint32_t version_info_bits[34] = {
    0x07C94, 0x085BC, 0x09A99, 0x0A4D3, 0x0BBF6, 0x0C762, 0x0D847, 0x0E60D,
    0x0F928, 0x10B78, 0x1145D, 0x12A17, 0x13532, 0x149A6, 0x15683, 0x168C9,
    0x177EC, 0x18EC4, 0x191E1, 0x1AFAB, 0x1B08E, 0x1CC1A, 0x1D33F, 0x1ED75,
    0x1F250, 0x209D5, 0x216F0, 0x228BA, 0x2379F, 0x24B0B, 0x2542E, 0x26A64,
    0x27541, 0x28C69,
};

/* ════════════════════════════════════════════════════════════════════════
 *  Image binarization (adaptive threshold)
 * ════════════════════════════════════════════════════════════════════════ */

static uint32_t *build_integral_image(const Image *img)
{
    int w = img->width;
    int h = img->height;
    uint32_t *integ = (uint32_t *)calloc((size_t)(w + 1) * (h + 1), sizeof(uint32_t));
    if (!integ) return NULL;

    for (int y = 0; y < h; y++) {
        uint32_t row_sum = 0;
        for (int x = 0; x < w; x++) {
            row_sum += img->pixels[y * w + x];
            integ[(y + 1) * (w + 1) + (x + 1)] = row_sum + integ[y * (w + 1) + (x + 1)];
        }
    }

    return integ;
}

static uint8_t *build_local_mean_map(const Image *img, const uint32_t *integ)
{
    int w = img->width;
    int h = img->height;
    uint8_t *map = (uint8_t *)calloc((size_t)w * h, 1);
    if (!map) return NULL;

    int block = w / 8;
    if (block < 8) block = 8;
    if (block > 64) block = 64;
    int half = block / 2;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int y0 = y - half; if (y0 < 0) y0 = 0;
            int x1 = x + half; if (x1 >= w) x1 = w - 1;
            int y1 = y + half; if (y1 >= h) y1 = h - 1;
            int area = (x1 - x0 + 1) * (y1 - y0 + 1);
            uint32_t sum = integ[(y1 + 1) * (w + 1) + (x1 + 1)]
                         - integ[y0 * (w + 1) + (x1 + 1)]
                         - integ[(y1 + 1) * (w + 1) + x0]
                         + integ[y0 * (w + 1) + x0];
            int mean = (int)(sum / (uint32_t)area);
            map[y * w + x] = (uint8_t)mean;
        }
    }

    return map;
}

static uint8_t *binarize_from_integral(const Image *img, int dark_bias,
                                       const uint32_t *integ)
{
    int w = img->width;
    int h = img->height;
    uint8_t *bin = (uint8_t *)calloc((size_t)w * h, 1);
    if (!bin) return NULL;

    /* Block-based adaptive threshold */
    int block = w / 8;
    if (block < 8) block = 8;
    if (block > 64) block = 64;
    int half = block / 2;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int y0 = y - half; if (y0 < 0) y0 = 0;
            int x1 = x + half; if (x1 >= w) x1 = w - 1;
            int y1 = y + half; if (y1 >= h) y1 = h - 1;
            int area = (x1 - x0 + 1) * (y1 - y0 + 1);
            uint32_t sum = integ[(y1 + 1) * (w + 1) + (x1 + 1)]
                         - integ[y0 * (w + 1) + (x1 + 1)]
                         - integ[(y1 + 1) * (w + 1) + x0]
                         + integ[y0 * (w + 1) + x0];
            int mean = (int)(sum / (uint32_t)area);
            int threshold = mean + dark_bias;
            bin[y * w + x] = (img->pixels[y * w + x] < threshold) ? 1 : 0;
        }
    }

    return bin;
}

static uint8_t *binarize_from_mean_map(const Image *img, int dark_bias,
                                       const uint8_t *map)
{
    int w = img->width;
    int h = img->height;
    uint8_t *bin = (uint8_t *)calloc((size_t)w * h, 1);
    if (!bin) return NULL;

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int p = y * w + x;
            int threshold = (int)map[p] + dark_bias;
            bin[p] = (img->pixels[p] < threshold) ? 1 : 0;
        }
    }

    return bin;
}

static uint8_t *binarize(const Image *img, int dark_bias)
{
    uint32_t *integ = build_integral_image(img);
    uint8_t *bin;

    if (!integ) return NULL;
    bin = binarize_from_integral(img, dark_bias, integ);
    free(integ);
    return bin;
}

static uint8_t *binarize_global(const Image *img)
{
    size_t pixel_count;
    uint8_t min_value = 255;
    uint8_t max_value = 0;
    uint8_t threshold;
    uint8_t *bin;

    if (!img || !img->pixels || img->width <= 0 || img->height <= 0) {
        return NULL;
    }
    pixel_count = (size_t)img->width * (size_t)img->height;
    for (size_t index = 0; index < pixel_count; index++) {
        uint8_t value = img->pixels[index];
        if (value < min_value) min_value = value;
        if (value > max_value) max_value = value;
    }
    threshold = (uint8_t)(((unsigned)min_value + (unsigned)max_value) / 2U);
    bin = (uint8_t *)calloc(pixel_count, 1);
    if (!bin) return NULL;
    for (size_t index = 0; index < pixel_count; index++) {
        bin[index] = img->pixels[index] < threshold ? 1U : 0U;
    }
    return bin;
}

typedef struct {
    double cx, cy;
    double module_size;
} FinderCenter;

#define MAX_FINDERS 64

static double qr_finder_triangle_score(const FinderCenter *a,
                                       const FinderCenter *b,
                                       const FinderCenter *c)
{
    double abx = b->cx - a->cx;
    double aby = b->cy - a->cy;
    double acx = c->cx - a->cx;
    double acy = c->cy - a->cy;
    double bcx = c->cx - b->cx;
    double bcy = c->cy - b->cy;
    double d_ab = abx * abx + aby * aby;
    double d_ac = acx * acx + acy * acy;
    double d_bc = bcx * bcx + bcy * bcy;
    double hyp;
    double leg_a;
    double leg_b;
    double dot;
    double leg_balance;
    double right_quality;
    double pythagorean_quality;
    double axis_quality;
    double shape_quality;
    double area;
    double module_size;
    double axis_modules;
    double axis_error = 1e9;

    if (d_ab <= 1.0 || d_ac <= 1.0 || d_bc <= 1.0) {
        return 0.0;
    }

    /* The longest side is the diagonal between the two outer finders. */
    if (d_ab >= d_ac && d_ab >= d_bc) {
        hyp = d_ab;
        leg_a = d_ac;
        leg_b = d_bc;
        dot = (a->cx - c->cx) * (b->cx - c->cx) +
              (a->cy - c->cy) * (b->cy - c->cy);
    } else if (d_ac >= d_ab && d_ac >= d_bc) {
        hyp = d_ac;
        leg_a = d_ab;
        leg_b = d_bc;
        dot = (a->cx - b->cx) * (c->cx - b->cx) +
              (a->cy - b->cy) * (c->cy - b->cy);
    } else {
        hyp = d_bc;
        leg_a = d_ab;
        leg_b = d_ac;
        dot = (b->cx - a->cx) * (c->cx - a->cx) +
              (b->cy - a->cy) * (c->cy - a->cy);
    }

    if (leg_a <= 0.0 || leg_b <= 0.0 || hyp <= 0.0) {
        return 0.0;
    }
    leg_balance = fmin(leg_a, leg_b) / fmax(leg_a, leg_b);
    right_quality = 1.0 - fmin(1.0,
                               fabs(dot) / (sqrt(leg_a * leg_b) + 1e-9));
    pythagorean_quality = 1.0 - fmin(1.0,
                                     fabs((leg_a + leg_b - hyp) / hyp));
    module_size = (a->module_size + b->module_size + c->module_size) / 3.0;
    axis_modules = 0.5 * (sqrt(leg_a) + sqrt(leg_b)) /
                   (module_size + 1e-9);
    for (int version = 1; version <= 40; version++) {
        double expected_axis = (double)(4 * version + 10);
        double error = fabs(axis_modules - expected_axis);
        if (error < axis_error) axis_error = error;
    }
    axis_quality = 1.0 - fmin(1.0, axis_error / 3.0);
    shape_quality = (leg_balance + right_quality +
                     pythagorean_quality + axis_quality) / 4.0;
    area = fabs(abx * acy - aby * acx);

    /* Keep area as a scale preference, but reject large non-QR triangles. */
    return area * (0.20 + 0.80 * shape_quality);
}

static int find_finder_patterns(const uint8_t *bin, int w, int h,
                                FinderCenter *out, int max_out);
static int try_microqr_decode(const uint8_t *bin, int width, int height,
                              BarcodeResult *result);

static int qrcode_decode_from_binary(const Image *img, const uint8_t *bin,
                                     BarcodeResult *result);
static void flip_horizontal(uint8_t *buf, int w, int h);
static int order_finders(const FinderCenter *fp, int a, int b, int c,
                         int *tl, int *tr, int *bl);
static int qr_reflection_layout_hint(const FinderCenter *a,
                                     const FinderCenter *b,
                                     const FinderCenter *c);
static int qrcode_decode_probe_crop(const Image *img, BarcodeResult *result)
{
    int width;
    int height;
    uint8_t *probe;
    FinderCenter finders[MAX_FINDERS];
    int n_finders;
    int probe_inverted = 0;
    size_t dark_blocks = 0;

    g_qr_debug_stats.probe_calls++;

    if (!img || img->width < 480 || img->height < 480) return 0;
    width = img->width / 3;
    height = img->height / 3;
    probe = (uint8_t *)calloc((size_t)width * height, 1);
    if (!probe) return 0;

    for (int y = 0; y < height; y++) {
        uint8_t *out = probe + (size_t)y * width;
        for (int x = 0; x < width; x++) {
            int source_x = x * 3;
            out[x] = img->pixels[(size_t)(y * 3 + 1) * img->width +
                                 (size_t)(source_x + 1)] < 128 ? 1U : 0U;
            if (out[x]) dark_blocks++;
        }
    }

    if (dark_blocks > ((size_t)width * height) / 2U) {
        probe_inverted = 1;
    }

    n_finders = find_finder_patterns(probe, width, height,
                                     finders, MAX_FINDERS);
    if (n_finders == 0) {
        dark_blocks = 0;
        for (int y = 0; y < height; y++) {
            const uint8_t *row0 = img->pixels + (size_t)(y * 3) * img->width;
            const uint8_t *row1 = row0 + img->width;
            const uint8_t *row2 = row1 + img->width;
            uint8_t *out = probe + (size_t)y * width;
            for (int x = 0; x < width; x++) {
                int source_x = x * 3;
                int dark = (row0[source_x] < 128) +
                           (row0[source_x + 1] < 128) +
                           (row0[source_x + 2] < 128) +
                           (row1[source_x] < 128) +
                           (row1[source_x + 1] < 128) +
                           (row1[source_x + 2] < 128) +
                           (row2[source_x] < 128) +
                           (row2[source_x + 1] < 128) +
                           (row2[source_x + 2] < 128);
                out[x] = dark >= 5 ? 1U : 0U;
                if (out[x]) dark_blocks++;
            }
        }
        if (dark_blocks > ((size_t)width * height) / 2U) {
            probe_inverted = 1;
        }
        n_finders = find_finder_patterns(probe, width, height,
                                         finders, MAX_FINDERS);
    }
    if (n_finders == 0) {
        for (size_t index = 0; index < (size_t)width * height; index++) {
            probe[index] ^= 1U;
        }
        probe_inverted = 1;
        n_finders = find_finder_patterns(probe, width, height,
                                         finders, MAX_FINDERS);
    }
    if ((uint32_t)n_finders > g_qr_debug_stats.probe_max_finders) {
        g_qr_debug_stats.probe_max_finders = (uint32_t)n_finders;
    }
    if (n_finders < 1) {
        free(probe);
        return 0;
    }
    for (int i = 0; i < n_finders; i++) {
        finders[i].cx *= 3.0;
        finders[i].cy *= 3.0;
        finders[i].module_size *= 3.0;
    }

    if (n_finders == 1) {
        int margin = (int)(finders[0].module_size * 14.0 + 0.5);
        int x0 = (int)finders[0].cx - margin;
        int y0 = (int)finders[0].cy - margin;
        int x1 = (int)finders[0].cx + margin + 1;
        int y1 = (int)finders[0].cy + margin + 1;
        if (x0 < 0) x0 = 0;
        if (y0 < 0) y0 = 0;
        if (x1 > img->width) x1 = img->width;
        if (y1 > img->height) y1 = img->height;
        if (x1 - x0 >= 21 && y1 - y0 >= 21 &&
            x1 - x0 <= 512 && y1 - y0 <= 512) {
            int crop_width = x1 - x0;
            int crop_height = y1 - y0;
            uint8_t *pixels = (uint8_t *)calloc(
                (size_t)crop_width * crop_height, 1);
            uint8_t *base_bin = (uint8_t *)calloc(
                (size_t)crop_width * crop_height, 1);
            uint8_t *variant = (uint8_t *)calloc(
                (size_t)crop_width * crop_height, 1);
            uint8_t *variant_pixels = (uint8_t *)calloc(
                (size_t)crop_width * crop_height, 1);
            if (pixels && base_bin && variant && variant_pixels) {
                for (int y = 0; y < crop_height; y++) {
                    const uint8_t *source = img->pixels +
                        (size_t)(y0 + y) * img->width + x0;
                    uint8_t *pixel_row = pixels + (size_t)y * crop_width;
                    uint8_t *bin_row = base_bin + (size_t)y * crop_width;
                    memcpy(pixel_row, source, (size_t)crop_width);
                    for (int x = 0; x < crop_width; x++) {
                        bin_row[x] = pixel_row[x] < 128 ? 1U : 0U;
                    }
                }
                for (int variant_attempt = 0; variant_attempt < 4;
                     variant_attempt++) {
                    int polarity = (probe_inverted + (variant_attempt / 2)) & 1;
                    int mirror = variant_attempt & 1;
                        memcpy(variant, base_bin,
                               (size_t)crop_width * crop_height);
                        memcpy(variant_pixels, pixels,
                               (size_t)crop_width * crop_height);
                        if (polarity) {
                            for (size_t index = 0;
                                 index < (size_t)crop_width * crop_height;
                                 index++) {
                                variant[index] ^= 1U;
                                variant_pixels[index] =
                                    (uint8_t)(255U - variant_pixels[index]);
                            }
                        }
                        if (mirror) {
                            for (int y = 0; y < crop_height; y++) {
                                uint8_t *row = variant + (size_t)y * crop_width;
                                uint8_t *pixel_row = variant_pixels +
                                    (size_t)y * crop_width;
                                for (int left = 0, right = crop_width - 1;
                                     left < right; left++, right--) {
                                    uint8_t swap = row[left];
                                    row[left] = row[right];
                                    row[right] = swap;
                                    swap = pixel_row[left];
                                    pixel_row[left] = pixel_row[right];
                                    pixel_row[right] = swap;
                                }
                            }
                        }
                        if (try_microqr_decode(variant, crop_width,
                                               crop_height, result)) {
                            free(variant);
                            free(variant_pixels);
                            free(base_bin);
                            free(pixels);
                            free(probe);
                            return 1;
                        }
                    }
                }
            free(variant);
            free(variant_pixels);
            free(base_bin);
            free(pixels);
        }
        free(probe);
        return 0;
    }

    if (n_finders < 3) {
        free(probe);
        return 0;
    }

    {
        int selected[3] = {0, 1, 2};
        double best_score = 0.0;
        for (int i = 0; i < n_finders - 2; i++) {
            for (int j = i + 1; j < n_finders - 1; j++) {
                for (int k = j + 1; k < n_finders; k++) {
                    double min_module = finders[i].module_size;
                    double max_module = min_module;
                    double score;
                    if (finders[j].module_size < min_module)
                        min_module = finders[j].module_size;
                    if (finders[k].module_size < min_module)
                        min_module = finders[k].module_size;
                    if (finders[j].module_size > max_module)
                        max_module = finders[j].module_size;
                    if (finders[k].module_size > max_module)
                        max_module = finders[k].module_size;
                    if (min_module <= 0.0 || max_module > min_module * 1.8)
                        continue;
                    score = qr_finder_triangle_score(&finders[i],
                                                     &finders[j],
                                                     &finders[k]);
                    if (score > best_score) {
                        best_score = score;
                        selected[0] = i;
                        selected[1] = j;
                        selected[2] = k;
                    }
                }
            }
        }
        if (best_score > 0.0) {
            int mirror_first = qr_reflection_layout_hint(
                &finders[selected[0]], &finders[selected[1]],
                &finders[selected[2]]);
            double min_x = finders[selected[0]].cx;
            double max_x = min_x;
            double min_y = finders[selected[0]].cy;
            double max_y = min_y;
            double module_size = finders[selected[0]].module_size;
            for (int i = 1; i < 3; i++) {
                FinderCenter *finder = &finders[selected[i]];
                if (finder->cx < min_x) min_x = finder->cx;
                if (finder->cx > max_x) max_x = finder->cx;
                if (finder->cy < min_y) min_y = finder->cy;
                if (finder->cy > max_y) max_y = finder->cy;
                if (finder->module_size > module_size)
                    module_size = finder->module_size;
            }
            int margin = (int)(module_size * 8.0 + 0.5);
            int x0;
            int y0;
            int x1;
            int y1;
            if (margin < 12) margin = 12;
            x0 = (int)(min_x - margin);
            y0 = (int)(min_y - margin);
            x1 = (int)(max_x + margin + 1.0);
            y1 = (int)(max_y + margin + 1.0);
            if (x0 < 0) x0 = 0;
            if (y0 < 0) y0 = 0;
            if (x1 > img->width) x1 = img->width;
            if (y1 > img->height) y1 = img->height;
            if (x1 - x0 >= 21 && y1 - y0 >= 21 &&
                x1 - x0 <= 512 && y1 - y0 <= 512) {
                int crop_width = x1 - x0;
                int crop_height = y1 - y0;
                uint8_t *pixels = (uint8_t *)calloc(
                    (size_t)crop_width * crop_height, 1);
                uint8_t *bin = (uint8_t *)calloc(
                    (size_t)crop_width * crop_height, 1);
                uint8_t *variant_pixels = (uint8_t *)calloc(
                    (size_t)crop_width * crop_height, 1);
                if (pixels && bin && variant_pixels) {
                    Image crop = *img;
                    for (int y = 0; y < crop_height; y++) {
                        const uint8_t *source = img->pixels +
                            (size_t)(y0 + y) * img->width + x0;
                        uint8_t *pixel_row = pixels + (size_t)y * crop_width;
                        uint8_t *bin_row = bin + (size_t)y * crop_width;
                        memcpy(pixel_row, source, (size_t)crop_width);
                        for (int x = 0; x < crop_width; x++) {
                            bin_row[x] = pixel_row[x] < 128 ? 1U : 0U;
                        }
                    }
                    crop.width = crop_width;
                    crop.height = crop_height;
                    for (int variant_attempt = 0; variant_attempt < 4;
                         variant_attempt++) {
                        int polarity = (probe_inverted + (variant_attempt / 2)) & 1;
                        int mirror = mirror_first
                            ? ((variant_attempt & 1) == 0)
                            : (variant_attempt & 1);
                        memcpy(variant_pixels, pixels,
                               (size_t)crop_width * crop_height);
                        for (size_t index = 0;
                             index < (size_t)crop_width * crop_height; index++) {
                            bin[index] = pixels[index] < 128 ? 1U : 0U;
                            if (polarity) {
                                bin[index] ^= 1U;
                                variant_pixels[index] =
                                    (uint8_t)(255U - variant_pixels[index]);
                            }
                        }
                        if (mirror) {
                            for (int y = 0; y < crop_height; y++) {
                                uint8_t *row = bin + (size_t)y * crop_width;
                                uint8_t *pixel_row = variant_pixels +
                                    (size_t)y * crop_width;
                                for (int left = 0, right = crop_width - 1;
                                     left < right; left++, right--) {
                                    uint8_t swap = row[left];
                                    row[left] = row[right];
                                    row[right] = swap;
                                    swap = pixel_row[left];
                                    pixel_row[left] = pixel_row[right];
                                    pixel_row[right] = swap;
                                }
                            }
                        }
                        crop.pixels = variant_pixels;
                        if (qrcode_decode_from_binary(&crop, bin, result)) {
                            free(variant_pixels);
                            free(bin);
                            free(pixels);
                            free(probe);
                            return 1;
                        }
                    }
                }
                free(variant_pixels);
                free(bin);
                free(pixels);
            }
        }
    }
    free(probe);
    return 0;
}

static int qrcode_decode_dense_crop(const Image *img, BarcodeResult *result)
{
    const int sample_step = 4;
    const int sample_width = (img->width + sample_step - 1) / sample_step;
    const int row_threshold = sample_width / 16;
    int *row_scores;
    int run_start = -1;
    int run_end = -1;
    int best_start = -1;
    int best_end = -1;
    int best_length = 0;
    int current_length = 0;

    if (!img || !img->pixels || img->width < 160 || img->height < 160) {
        return 0;
    }
    row_scores = (int *)calloc((size_t)img->height, sizeof(*row_scores));
    if (!row_scores) {
        return 0;
    }
    for (int y = 0; y < img->height; y += sample_step) {
        int score = 0;
        for (int x = 0; x < img->width; x += sample_step) {
            score += img->pixels[(size_t)y * (size_t)img->width + x] < 128;
        }
        row_scores[y] = score;
        if (score >= row_threshold) {
            if (run_start < 0) {
                run_start = y;
            }
            run_end = y;
            current_length = run_end - run_start + sample_step;
        } else if (run_start >= 0) {
            if (current_length > best_length) {
                best_length = current_length;
                best_start = run_start;
                best_end = run_end;
            }
            run_start = -1;
            run_end = -1;
            current_length = 0;
        }
    }
    if (run_start >= 0 && current_length > best_length) {
        best_length = current_length;
        best_start = run_start;
        best_end = run_end;
    }
    free(row_scores);
    if (best_start < 0 || best_length < img->height / 32) {
        return 0;
    }

    {
        int min_x = img->width;
        int max_x = -1;
        int y0 = best_start - sample_step * 3;
        int y1 = best_end + sample_step * 3;
        if (y0 < 0) y0 = 0;
        if (y1 >= img->height) y1 = img->height - 1;
        for (int y = y0; y <= y1; y += sample_step) {
            for (int x = 0; x < img->width; x += sample_step) {
                if (img->pixels[(size_t)y * (size_t)img->width + x] < 128) {
                    if (x < min_x) min_x = x;
                    if (x > max_x) max_x = x;
                }
            }
        }
        if (max_x < min_x) {
            return 0;
        }
        min_x -= sample_step * 3;
        max_x += sample_step * 3;
        if (min_x < 0) min_x = 0;
        if (max_x >= img->width) max_x = img->width - 1;
        y0 -= sample_step * 2;
        y1 += sample_step * 2;
        if (y0 < 0) y0 = 0;
        if (y1 >= img->height) y1 = img->height - 1;
        {
            int crop_width = max_x - min_x + 1;
            int crop_height = y1 - y0 + 1;
            size_t crop_size = (size_t)crop_width * (size_t)crop_height;
            uint8_t *pixels;
            uint8_t *bin;
            uint8_t *variant;
            Image crop;

            if (crop_width < 21 || crop_height < 21 ||
                crop_width > 512 || crop_height > 512) {
                return 0;
            }
            pixels = (uint8_t *)calloc(crop_size, 1);
            bin = (uint8_t *)calloc(crop_size, 1);
            variant = (uint8_t *)calloc(crop_size, 1);
            if (!pixels || !bin || !variant) {
                free(variant);
                free(bin);
                free(pixels);
                return 0;
            }
            for (int y = 0; y < crop_height; y++) {
                const uint8_t *source = img->pixels +
                    (size_t)(y0 + y) * (size_t)img->width + min_x;
                uint8_t *pixel_row = pixels + (size_t)y * (size_t)crop_width;
                memcpy(pixel_row, source, (size_t)crop_width);
                for (int x = 0; x < crop_width; x++) {
                    bin[(size_t)y * (size_t)crop_width + x] =
                        pixel_row[x] < 128 ? 1U : 0U;
                }
            }
            crop = *img;
            crop.width = crop_width;
            crop.height = crop_height;
            for (int attempt = 0; attempt < 4; attempt++) {
                memcpy(variant, pixels, crop_size);
                memcpy(bin, pixels, crop_size);
                for (size_t index = 0; index < crop_size; index++) {
                    bin[index] = variant[index] < 128 ? 1U : 0U;
                    if (attempt >= 2) {
                        bin[index] ^= 1U;
                        variant[index] = (uint8_t)(255U - variant[index]);
                    }
                }
                if (attempt & 1) {
                    flip_horizontal(bin, crop_width, crop_height);
                    flip_horizontal(variant, crop_width, crop_height);
                }
                crop.pixels = variant;
                if (qrcode_decode_from_binary(&crop, bin, result)) {
                    free(variant);
                    free(bin);
                    free(pixels);
                    return 1;
                }
            }
            free(variant);
            free(bin);
            free(pixels);
        }
    }
    return 0;
}

static int qrcode_decode_global(const Image *img, BarcodeResult *result)
{
    g_qr_debug_stats.global_calls++;
    if (qrcode_decode_probe_crop(img, result)) {
        return 1;
    }
    uint8_t *bin = binarize_global(img);
    int decoded = 0;

    if (!bin) return 0;

    {
        FinderCenter finders[MAX_FINDERS];
        int n_finders = find_finder_patterns(bin, img->width, img->height,
                                             finders, MAX_FINDERS);
        if (n_finders >= 3) {
            int selected[3] = {0, 1, 2};
            double best_score = 0.0;
            for (int i = 0; i < n_finders - 2; i++) {
                for (int j = i + 1; j < n_finders - 1; j++) {
                    for (int k = j + 1; k < n_finders; k++) {
                        double min_module = finders[i].module_size;
                        double max_module = min_module;
                        double score;
                        if (finders[j].module_size < min_module)
                            min_module = finders[j].module_size;
                        if (finders[k].module_size < min_module)
                            min_module = finders[k].module_size;
                        if (finders[j].module_size > max_module)
                            max_module = finders[j].module_size;
                        if (finders[k].module_size > max_module)
                            max_module = finders[k].module_size;
                        if (min_module <= 0.0 || max_module > min_module * 1.8)
                            continue;
                        score = qr_finder_triangle_score(&finders[i],
                                                         &finders[j],
                                                         &finders[k]);
                        if (score > best_score) {
                            best_score = score;
                            selected[0] = i;
                            selected[1] = j;
                            selected[2] = k;
                        }
                    }
                }
            }
            if (best_score > 0.0) {
                double min_x = finders[selected[0]].cx;
                double max_x = min_x;
                double min_y = finders[selected[0]].cy;
                double max_y = min_y;
                double module_size = finders[selected[0]].module_size;
                for (int i = 1; i < 3; i++) {
                    FinderCenter *finder = &finders[selected[i]];
                    if (finder->cx < min_x) min_x = finder->cx;
                    if (finder->cx > max_x) max_x = finder->cx;
                    if (finder->cy < min_y) min_y = finder->cy;
                    if (finder->cy > max_y) max_y = finder->cy;
                    if (finder->module_size > module_size) {
                        module_size = finder->module_size;
                    }
                }
                int margin = (int)(module_size * 8.0 + 0.5);
                if (margin < 12) margin = 12;
            int x0 = (int)(min_x - margin);
            int y0 = (int)(min_y - margin);
            int x1 = (int)(max_x + margin + 1.0);
            int y1 = (int)(max_y + margin + 1.0);
            if (x0 < 0) x0 = 0;
            if (y0 < 0) y0 = 0;
            if (x1 > img->width) x1 = img->width;
            if (y1 > img->height) y1 = img->height;
            if (x1 - x0 < 21 || y1 - y0 < 21) {
                x0 = 0;
                y0 = 0;
                x1 = 0;
                y1 = 0;
            }
            if (x1 > x0 && y1 > y0 && x1 - x0 <= 512 && y1 - y0 <= 512) {
                int crop_width = x1 - x0;
                int crop_height = y1 - y0;
                uint8_t *crop_pixels = (uint8_t *)calloc(
                    (size_t)crop_width * crop_height, 1);
                uint8_t *crop_bin = (uint8_t *)calloc(
                    (size_t)crop_width * crop_height, 1);
                if (crop_pixels && crop_bin) {
                    Image crop = *img;
                    for (int y = 0; y < crop_height; y++) {
                        memcpy(crop_pixels + (size_t)y * crop_width,
                               img->pixels + (size_t)(y0 + y) * img->width + x0,
                               (size_t)crop_width);
                        memcpy(crop_bin + (size_t)y * crop_width,
                               bin + (size_t)(y0 + y) * img->width + x0,
                               (size_t)crop_width);
                    }
                    crop.width = crop_width;
                    crop.height = crop_height;
                    crop.pixels = crop_pixels;
                    decoded = qrcode_decode_from_binary(&crop, crop_bin, result);
                    if (!decoded) {
                        for (size_t index = 0;
                             index < (size_t)crop_width * crop_height; index++) {
                            crop_bin[index] ^= 1U;
                        }
                        decoded = qrcode_decode_from_binary(&crop, crop_bin, result);
                    }
                    if (!decoded) {
                        for (size_t index = 0;
                             index < (size_t)crop_width * crop_height; index++) {
                            crop_bin[index] ^= 1U;
                        }
                        flip_horizontal(crop_pixels, crop_width, crop_height);
                        flip_horizontal(crop_bin, crop_width, crop_height);
                        decoded = qrcode_decode_from_binary(&crop, crop_bin, result);
                        if (!decoded) {
                            for (size_t index = 0;
                                 index < (size_t)crop_width * crop_height; index++) {
                                crop_bin[index] ^= 1U;
                            }
                            decoded = qrcode_decode_from_binary(&crop, crop_bin, result);
                        }
                    }
                }
                free(crop_bin);
                free(crop_pixels);
                if (decoded) {
                    free(bin);
                    return 1;
                }
            }
            }
        }
    }

    decoded = qrcode_decode_from_binary(img, bin, result);
    if (!decoded) {
        for (int p = 0; p < img->width * img->height; p++) {
            bin[p] ^= 1U;
        }
        decoded = qrcode_decode_from_binary(img, bin, result);
    }
    free(bin);
    return decoded;
}

static int qrcode_decode_inverse_panel(const Image *img, BarcodeResult *result)
{
    size_t dark_count = 0;
    size_t sampled_pixels;
    const int sample_step = 4;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    int crop_width;
    int crop_height;
    uint8_t *pixels;
    uint8_t *bin;
    Image crop;

    if (!img || !img->pixels || img->width < 320 || img->height < 240) {
        return 0;
    }
    sampled_pixels = ((size_t)img->width + sample_step - 1U) / sample_step;
    sampled_pixels *= ((size_t)img->height + sample_step - 1U) / sample_step;
    min_x = img->width;
    min_y = img->height;
    max_x = -1;
    max_y = -1;
    for (int y = 0; y < img->height; y += sample_step) {
        for (int x = 0; x < img->width; x += sample_step) {
            if (img->pixels[(size_t)y * img->width + x] < 64) {
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
                dark_count++;
            }
        }
    }
    if (max_x < min_x || max_y < min_y || dark_count < sampled_pixels / 8U) {
        return 0;
    }
    crop_width = max_x - min_x + 1;
    crop_height = max_y - min_y + 1;
    min_x -= sample_step;
    min_y -= sample_step;
    max_x += sample_step;
    max_y += sample_step;
    if (min_x < 0) min_x = 0;
    if (min_y < 0) min_y = 0;
    if (max_x >= img->width) max_x = img->width - 1;
    if (max_y >= img->height) max_y = img->height - 1;
    crop_width = max_x - min_x + 1;
    crop_height = max_y - min_y + 1;
    if (crop_width < img->width / 2 || crop_height < img->height / 8 ||
        crop_width > img->width || crop_height > img->height) {
        return 0;
    }
    {
        size_t crop_sample_width = ((size_t)crop_width + sample_step - 1U) /
                                   sample_step;
        size_t crop_sample_height = ((size_t)crop_height + sample_step - 1U) /
                                    sample_step;
        if (dark_count * 100U < crop_sample_width * crop_sample_height * 20U) {
            return 0;
        }
    }

    pixels = (uint8_t *)calloc((size_t)crop_width * crop_height, 1);
    bin = (uint8_t *)calloc((size_t)crop_width * crop_height, 1);
    if (!pixels || !bin) {
        free(bin);
        free(pixels);
        return 0;
    }
    for (int y = 0; y < crop_height; y++) {
        const uint8_t *source = img->pixels +
            (size_t)(min_y + y) * img->width + min_x;
        uint8_t *pixel_row = pixels + (size_t)y * crop_width;
        uint8_t *bin_row = bin + (size_t)y * crop_width;
        for (int x = 0; x < crop_width; x++) {
            pixel_row[x] = (uint8_t)(255U - source[x]);
            bin_row[x] = source[x] > 127 ? 1U : 0U;
        }
    }

    crop = *img;
    crop.width = crop_width;
    crop.height = crop_height;
    crop.pixels = pixels;
    int decoded = try_microqr_decode(bin, crop_width, crop_height, result);
    if (!decoded) {
        decoded = qrcode_decode_from_binary(&crop, bin, result);
    }
    free(bin);
    free(pixels);
    return decoded;
}

static uint8_t *binarize_for_decode(const Image *img, int dark_bias,
                                    const uint8_t *shared_mean_map,
                                    const uint32_t *shared_integ)
{
    if (shared_mean_map) {
        return binarize_from_mean_map(img, dark_bias, shared_mean_map);
    }
    if (shared_integ) {
        return binarize_from_integral(img, dark_bias, shared_integ);
    }
    return binarize(img, dark_bias);
}

/* ════════════════════════════════════════════════════════════════════════
 *  Finder pattern detection
 * ════════════════════════════════════════════════════════════════════════ */

/* Check 1:1:3:1:1 ratio in counts[5] */
static int check_ratio(const int *c)
{
    int total = c[0] + c[1] + c[2] + c[3] + c[4];
    if (total < 7) return 0;
    double unit = (double)total / 7.0;
    double tol = unit * 0.7;

    return fabs(c[0] - unit) < tol &&
           fabs(c[1] - unit) < tol &&
           fabs(c[2] - 3.0 * unit) < tol &&
           fabs(c[3] - unit) < tol &&
           fabs(c[4] - unit) < tol;
}

/* Confirm finder by cross-checking vertically */
static int cross_check_vertical(const uint8_t *bin, int w, int h,
                                int cx, int cy, int est_count,
                                double *refined_cy)
{
    int counts[5] = {0};
    int y = cy;

    /* Count center (state 2) going up */
    while (y >= 0 && bin[y * w + cx]) { counts[2]++; y--; }
    if (y < 0) return 0;
    while (y >= 0 && !bin[y * w + cx]) { counts[1]++; y--; }
    if (y < 0) return 0;
    while (y >= 0 && bin[y * w + cx]) { counts[0]++; y--; }

    /* Count center (state 2) going down */
    y = cy + 1;
    while (y < h && bin[y * w + cx]) { counts[2]++; y++; }
    if (y >= h) return 0;
    while (y < h && !bin[y * w + cx]) { counts[3]++; y++; }
    if (y >= h) return 0;
    while (y < h && bin[y * w + cx]) { counts[4]++; y++; }

    int total = counts[0] + counts[1] + counts[2] + counts[3] + counts[4];
    if (5 * abs(total - est_count) > 2 * est_count) return 0;

    if (!check_ratio(counts)) return 0;

    /* Compute refined Y center: end_y is 1 past the last pixel of state 4.
     * The center of the pattern = end_y - counts[4] - counts[3] - counts[2]/2 */
    if (refined_cy) {
        *refined_cy = (double)(y - counts[4] - counts[3]) - (double)counts[2] / 2.0;
    }
    return 1;
}

/* Cross-check horizontally at precise position */
static int cross_check_horizontal(const uint8_t *bin, int w,
                                  int cx, int cy, int est_count)
{
    int counts[5] = {0};
    int x = cx;

    while (x >= 0 && bin[cy * w + x]) { counts[2]++; x--; }
    if (x < 0) return 0;
    while (x >= 0 && !bin[cy * w + x]) { counts[1]++; x--; }
    if (x < 0) return 0;
    while (x >= 0 && bin[cy * w + x]) { counts[0]++; x--; }

    x = cx + 1;
    while (x < w && bin[cy * w + x]) { counts[2]++; x++; }
    if (x >= w) return 0;
    while (x < w && !bin[cy * w + x]) { counts[3]++; x++; }
    if (x >= w) return 0;
    while (x < w && bin[cy * w + x]) { counts[4]++; x++; }

    int total = counts[0] + counts[1] + counts[2] + counts[3] + counts[4];
    if (5 * abs(total - est_count) > 2 * est_count) return 0;

    return check_ratio(counts);
}

/* Find center x from horizontal counts at row y */
static double center_from_counts(int end_x, const int *c)
{
    return (double)(end_x - c[4] - c[3]) - (double)c[2] / 2.0;
}

static int find_finder_patterns(const uint8_t *bin, int w, int h,
                                FinderCenter *out, int max_out)
{
    int n = 0;

    for (int y = 0; y < h && n < max_out; y++) {
        int counts[5] = {0};
        int state = 0;

        for (int x = 0; x < w; x++) {
            int pixel = bin[y * w + x];
            if (pixel) {
                /* dark */
                if (state == 1 || state == 3) {
                    state++;
                    counts[state] = 1;
                } else {
                    counts[state]++;
                }
            } else {
                /* light */
                if (state == 0 || state == 2) {
                    state++;
                    counts[state] = 1;
                } else if (state == 4) {
                    if (check_ratio(counts)) {
                        int total = counts[0] + counts[1] + counts[2] + counts[3] + counts[4];
                        double cx_d = center_from_counts(x, counts);
                        int cx_i = (int)(cx_d + 0.5);

                        double refined_cy = (double)y;
                        if (cx_i >= 0 && cx_i < w &&
                            cross_check_vertical(bin, w, h, cx_i, y, total, &refined_cy) &&
                            cross_check_horizontal(bin, w, cx_i, y, total)) {

                            double ms = (double)total / 7.0;
                            /* Check for duplicate (merge nearby) */
                            int merged = 0;
                            for (int i = 0; i < n; i++) {
                                double dx = out[i].cx - cx_d;
                                double dy = out[i].cy - refined_cy;
                                if (dx * dx + dy * dy < ms * ms * 4.0) {
                                    out[i].cx = (out[i].cx + cx_d) / 2.0;
                                    out[i].cy = (out[i].cy + refined_cy) / 2.0;
                                    out[i].module_size = (out[i].module_size + ms) / 2.0;
                                    merged = 1;
                                    break;
                                }
                            }
                            if (!merged && n < max_out) {
                                out[n].cx = cx_d;
                                out[n].cy = refined_cy;
                                out[n].module_size = ms;
                                n++;
                            }
                        }
                    }
                    /* Shift counts */
                    counts[0] = counts[2];
                    counts[1] = counts[3];
                    counts[2] = counts[4];
                    counts[3] = 1;
                    counts[4] = 0;
                    state = 3;
                } else {
                    counts[state]++;
                }
            }
        }
    }

    return n;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Order 3 finder patterns into TL, TR, BL
 * ════════════════════════════════════════════════════════════════════════ */

static double dist2(const FinderCenter *a, const FinderCenter *b)
{
    double dx = a->cx - b->cx;
    double dy = a->cy - b->cy;
    return dx * dx + dy * dy;
}

static double cross_product_z(const FinderCenter *o, const FinderCenter *a, const FinderCenter *b)
{
    return (a->cx - o->cx) * (b->cy - o->cy) - (a->cy - o->cy) * (b->cx - o->cx);
}

/* Assigns: tl_idx, tr_idx, bl_idx. Returns 0 on success. */
static int order_finders(const FinderCenter *fp, int a, int b, int c,
                         int *tl, int *tr, int *bl)
{
    double d_ab = dist2(&fp[a], &fp[b]);
    double d_ac = dist2(&fp[a], &fp[c]);
    double d_bc = dist2(&fp[b], &fp[c]);

    int corner, p1, p2;
    if (d_ab > d_ac && d_ab > d_bc) {
        corner = c; p1 = a; p2 = b;
    } else if (d_ac > d_ab && d_ac > d_bc) {
        corner = b; p1 = a; p2 = c;
    } else {
        corner = a; p1 = b; p2 = c;
    }

    /* corner is TL; determine TR vs BL by cross product.
     * In image coordinates y increases downward, so the sign is
     * opposite to the standard math convention: cp > 0 means p1 is
     * to the right of the TL→p2 vector, i.e. p1 = TR. */
    double cp = cross_product_z(&fp[corner], &fp[p1], &fp[p2]);
    if (cp > 0) {
        *tl = corner; *tr = p1; *bl = p2;
    } else {
        *tl = corner; *tr = p2; *bl = p1;
    }
    return 0;
}

static int qr_reflection_layout_hint(const FinderCenter *a,
                                     const FinderCenter *b,
                                     const FinderCenter *c)
{
    const FinderCenter *corner;
    const FinderCenter *other_a;
    const FinderCenter *other_b;
    double d_ab = dist2(a, b);
    double d_ac = dist2(a, c);
    double d_bc = dist2(b, c);
    double tolerance;

    if (d_ab >= d_ac && d_ab >= d_bc) {
        corner = c;
        other_a = a;
        other_b = b;
    } else if (d_ac >= d_ab && d_ac >= d_bc) {
        corner = b;
        other_a = a;
        other_b = c;
    } else {
        corner = a;
        other_a = b;
        other_b = c;
    }

    tolerance = fmax(a->module_size,
                     fmax(b->module_size, c->module_size)) * 2.5;
    if (corner->cx >= other_a->cx && corner->cx >= other_b->cx &&
        fabs(other_a->cy - corner->cy) <= tolerance &&
        other_b->cy > corner->cy + tolerance &&
        fabs(other_b->cx - corner->cx) <= tolerance) {
        return 1;
    }
    if (corner->cx >= other_a->cx && corner->cx >= other_b->cx &&
        fabs(other_b->cy - corner->cy) <= tolerance &&
        other_a->cy > corner->cy + tolerance &&
        fabs(other_a->cx - corner->cx) <= tolerance) {
        return 1;
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Perspective transform (3x3 homography)
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    double a[9]; /* 3x3 row-major */
} Perspective;

typedef struct {
    int version;
    int modules;
    double axis_modules;
    double axis_error;
} QRGeometryEstimate;

typedef struct {
    int use_alignment;
    double align_x;
    double align_y;
    double align_score;
    double br_bias_x;
    double br_bias_y;
} QRPerspectiveCandidate;

typedef struct {
    int ok;
    int version;
    int modules;
    int ec_level;
    int mask_pattern;
    int quality_score;
    int vote_score;
    int sample_pass;
    int candidate_index;
} QRSamplingVoteResult;

typedef struct {
    int out_of_bounds_modules;
    int weak_vote_modules;
    int total_modules;
} QRSampleMetrics;

#define QR_ALIGN_SCORE_WEAK_DEFAULT          18.0
#define QR_ALIGN_SCORE_STRONG_DEFAULT        22.0
#define QR_OOB_REJECT_PERCENT_DEFAULT        18
#define QR_OOB_PENALTY_PERCENT_DEFAULT       7
#define QR_WEAK_VOTE_PENALTY_PERCENT_DEFAULT 12
#define QR_SKEW_BIAS_PENALTY_TH_DEFAULT      0.12

static const QRVoteTuning k_qr_vote_tuning_default = {
    QR_ALIGN_SCORE_WEAK_DEFAULT,
    QR_ALIGN_SCORE_STRONG_DEFAULT,
    QR_OOB_REJECT_PERCENT_DEFAULT,
    QR_OOB_PENALTY_PERCENT_DEFAULT,
    QR_WEAK_VOTE_PENALTY_PERCENT_DEFAULT,
    QR_SKEW_BIAS_PENALTY_TH_DEFAULT
};

static QRVoteTuning g_qr_vote_tuning = {
    QR_ALIGN_SCORE_WEAK_DEFAULT,
    QR_ALIGN_SCORE_STRONG_DEFAULT,
    QR_OOB_REJECT_PERCENT_DEFAULT,
    QR_OOB_PENALTY_PERCENT_DEFAULT,
    QR_WEAK_VOTE_PENALTY_PERCENT_DEFAULT,
    QR_SKEW_BIAS_PENALTY_TH_DEFAULT
};

static int solve_linear_8x8(double m[8][9], double out[8])
{
    for (int col = 0; col < 8; col++) {
        int pivot = col;
        double max_abs = fabs(m[col][col]);
        for (int row = col + 1; row < 8; row++) {
            double v = fabs(m[row][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = row;
            }
        }
        if (max_abs < 1e-12) {
            return 0;
        }
        if (pivot != col) {
            for (int k = col; k <= 8; k++) {
                double tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
            }
        }

        {
            double inv = 1.0 / m[col][col];
            for (int k = col; k <= 8; k++) {
                m[col][k] *= inv;
            }
        }

        for (int row = 0; row < 8; row++) {
            if (row == col) {
                continue;
            }
            {
                double factor = m[row][col];
                if (fabs(factor) < 1e-15) {
                    continue;
                }
                for (int k = col; k <= 8; k++) {
                    m[row][k] -= factor * m[col][k];
                }
            }
        }
    }

    for (int i = 0; i < 8; i++) {
        out[i] = m[i][8];
    }
    return 1;
}

/* Compute transform mapping (u,v) -> (x,y) for 4 point correspondences.
   src: 4 points in module space, dst: 4 points in image space. */
static Perspective compute_perspective(
    double su0, double sv0, double du0, double dv0,
    double su1, double sv1, double du1, double dv1,
    double su2, double sv2, double du2, double dv2,
    double su3, double sv3, double du3, double dv3)
{
    Perspective t;
    double eq[8][9];
    double sol[8];
    const double su[4] = {su0, su1, su2, su3};
    const double sv[4] = {sv0, sv1, sv2, sv3};
    const double du[4] = {du0, du1, du2, du3};
    const double dv[4] = {dv0, dv1, dv2, dv3};

    memset(&t, 0, sizeof(t));
    memset(eq, 0, sizeof(eq));

    for (int i = 0; i < 4; i++) {
        int row = i * 2;
        eq[row][0] = su[i];
        eq[row][1] = sv[i];
        eq[row][2] = 1.0;
        eq[row][6] = -su[i] * du[i];
        eq[row][7] = -sv[i] * du[i];
        eq[row][8] = du[i];

        eq[row + 1][3] = su[i];
        eq[row + 1][4] = sv[i];
        eq[row + 1][5] = 1.0;
        eq[row + 1][6] = -su[i] * dv[i];
        eq[row + 1][7] = -sv[i] * dv[i];
        eq[row + 1][8] = dv[i];
    }

    if (solve_linear_8x8(eq, sol)) {
        t.a[0] = sol[0];
        t.a[1] = sol[1];
        t.a[2] = sol[2];
        t.a[3] = sol[3];
        t.a[4] = sol[4];
        t.a[5] = sol[5];
        t.a[6] = sol[6];
        t.a[7] = sol[7];
        t.a[8] = 1.0;
        return t;
    }

    t.a[0] = du1 - du0; t.a[1] = du3 - du0; t.a[2] = du0;
    t.a[3] = dv1 - dv0; t.a[4] = dv3 - dv0; t.a[5] = dv0;
    t.a[6] = 0.0;       t.a[7] = 0.0;       t.a[8] = 1.0;

    return t;
}

static void perspective_map(const Perspective *t, double u, double v,
                            double *x, double *y)
{
    double w = t->a[6] * u + t->a[7] * v + t->a[8];
    if (fabs(w) < 1e-10) w = 1e-10;
    *x = (t->a[0] * u + t->a[1] * v + t->a[2]) / w;
    *y = (t->a[3] * u + t->a[4] * v + t->a[5]) / w;
}

static QRGeometryEstimate estimate_version_from_finder_geometry(double d_top,
                                                                double d_left,
                                                                double module_size)
{
    QRGeometryEstimate out;
    double axis;

    out.version = 1;
    out.modules = 21;
    out.axis_modules = 0.0;
    out.axis_error = 1e30;

    if (module_size <= 0.0) {
        return out;
    }

    axis = 0.5 * (d_top + d_left) / module_size;
    if (axis < 14.0) {
        axis = 14.0;
    }

    for (int v = 1; v <= 40; v++) {
        double expected_axis = (double)(4 * v + 10);
        double err = fabs(axis - expected_axis);
        if (err < out.axis_error) {
            out.axis_error = err;
            out.version = v;
        }
    }
    out.axis_modules = axis;
    out.modules = 4 * out.version + 17;
    return out;
}

static int qr_get_br_alignment_center_module(int version, double *u, double *v)
{
    int cnt;
    int pos;
    if (version < 2 || version > 40 || !u || !v) {
        return 0;
    }
    cnt = align_count(version);
    if (cnt <= 0 || cnt > 8) {
        return 0;
    }
    pos = align_pos[version - 1][cnt - 1];
    if (pos <= 0) {
        return 0;
    }
    *u = (double)pos + 0.5;
    *v = (double)pos + 0.5;
    return 1;
}

static int qr_alignment_expected_dark(int rr, int cc)
{
    if (rr == 0 && cc == 0) return 1;
    if (rr == -2 || rr == 2 || cc == -2 || cc == 2) return 1;
    return 0;
}

static int qr_sample_bin_at_module(const uint8_t *bin, int w, int h,
                                   const Perspective *t, double u, double v,
                                   int *bit)
{
    double ix, iy;
    int px, py;

    perspective_map(t, u, v, &ix, &iy);
    px = (int)(ix + 0.5);
    py = (int)(iy + 0.5);
    if (px < 0 || px >= w || py < 0 || py >= h) {
        return 0;
    }
    *bit = bin[py * w + px] ? 1 : 0;
    return 1;
}

static int qr_refine_alignment_anchor(const uint8_t *bin, int w, int h,
                                      int version, const Perspective *t,
                                      double *align_x, double *align_y,
                                      double *match_score)
{
    double base_u, base_v;
    double best_u = 0.0, best_v = 0.0;
    int best_score = -999999;

    if (!qr_get_br_alignment_center_module(version, &base_u, &base_v)) {
        return 0;
    }

    for (int du_step = -5; du_step <= 5; du_step++) {
        for (int dv_step = -5; dv_step <= 5; dv_step++) {
            double cu = base_u + (double)du_step * 0.15;
            double cv = base_v + (double)dv_step * 0.15;
            int score = 0;
            int valid = 0;
            for (int rr = -2; rr <= 2; rr++) {
                for (int cc = -2; cc <= 2; cc++) {
                    int bit = 0;
                    int want_dark = qr_alignment_expected_dark(rr, cc);
                    if (!qr_sample_bin_at_module(bin, w, h, t,
                                                 cu + (double)cc,
                                                 cv + (double)rr,
                                                 &bit)) {
                        continue;
                    }
                    valid++;
                    score += (bit == want_dark) ? 2 : -1;
                }
            }
            if (valid < 12) {
                continue;
            }
            if (score > best_score) {
                best_score = score;
                best_u = cu;
                best_v = cv;
            }
        }
    }

    if (best_score < 18) {
        return 0;
    }
    perspective_map(t, best_u, best_v, align_x, align_y);
    if (match_score) {
        *match_score = (double)best_score;
    }
    return 1;
}

static Perspective qr_build_perspective_from_finders_variant(int modules,
                                                             const FinderCenter *tl,
                                                             const FinderCenter *tr,
                                                             const FinderCenter *bl,
                                                             int use_alignment,
                                                             double align_x,
                                                             double align_y,
                                                             double br_bias_x,
                                                             double br_bias_y)
{
    double tl_u = 3.5;
    double tl_v = 3.5;
    double tr_u = (double)modules - 3.5;
    double tr_v = 3.5;
    double bl_u = 3.5;
    double bl_v = (double)modules - 3.5;
    double br_u = (double)modules - 3.5;
    double br_v = (double)modules - 3.5;
    double br_x = tr->cx + bl->cx - tl->cx + br_bias_x;
    double br_y = tr->cy + bl->cy - tl->cy + br_bias_y;

    if (use_alignment && modules >= 25) {
        double align_u = 0.0;
        double align_v = 0.0;
        if (qr_get_br_alignment_center_module((modules - 17) / 4, &align_u, &align_v)) {
            return compute_perspective(
                tl_u, tl_v, tl->cx, tl->cy,
                tr_u, tr_v, tr->cx, tr->cy,
                bl_u, bl_v, bl->cx, bl->cy,
                align_u, align_v, align_x, align_y);
        }
    }

    return compute_perspective(
        tl_u, tl_v, tl->cx, tl->cy,
        tr_u, tr_v, tr->cx, tr->cy,
        br_u, br_v, br_x, br_y,
        bl_u, bl_v, bl->cx, bl->cy);
}

static Perspective qr_build_perspective_from_finders(int modules,
                                                     const FinderCenter *tl,
                                                     const FinderCenter *tr,
                                                     const FinderCenter *bl,
                                                     int use_alignment,
                                                     double align_x,
                                                     double align_y)
{
    return qr_build_perspective_from_finders_variant(modules, tl, tr, bl,
                                                     use_alignment, align_x, align_y,
                                                     0.0, 0.0);
}

static int qr_build_perspective_candidates(const FinderCenter *tl,
                                           const FinderCenter *tr,
                                           const FinderCenter *bl,
                                           int modules,
                                           int alignment_hit,
                                           double align_x,
                                           double align_y,
                                           double align_score,
                                           double module_size,
                                           QRPerspectiveCandidate *out,
                                           int out_cap)
{
    int n = 0;
    double bias_step = module_size * 0.75;

    if (!tl || !tr || !bl || !out || out_cap <= 0) {
        return 0;
    }
    if (modules < 21) {
        modules = 21;
    }
    if (bias_step < 0.6) {
        bias_step = 0.6;
    }

    out[n].use_alignment = 0;
    out[n].align_x = 0.0;
    out[n].align_y = 0.0;
    out[n].align_score = 0.0;
    out[n].br_bias_x = 0.0;
    out[n].br_bias_y = 0.0;
    n++;

    if (n < out_cap) {
        out[n].use_alignment = 0;
        out[n].align_x = 0.0;
        out[n].align_y = 0.0;
        out[n].align_score = 0.0;
        out[n].br_bias_x = bias_step;
        out[n].br_bias_y = bias_step;
        n++;
    }
    if (n < out_cap) {
        out[n].use_alignment = 0;
        out[n].align_x = 0.0;
        out[n].align_y = 0.0;
        out[n].align_score = 0.0;
        out[n].br_bias_x = -bias_step;
        out[n].br_bias_y = -bias_step;
        n++;
    }
    if (alignment_hit && n < out_cap) {
        out[n].use_alignment = 1;
        out[n].align_x = align_x;
        out[n].align_y = align_y;
        out[n].align_score = align_score;
        out[n].br_bias_x = 0.0;
        out[n].br_bias_y = 0.0;
        n++;
    }

    return n;
}

static int qr_score_sampling_quality(const uint8_t *grid, int modules,
                                     int format_used_copy, int format_errors,
                                     int version, int version_used_copy,
                                     int version_errors, int version_agree,
                                     int alignment_used, double align_score,
                                     int sample_out_of_bounds,
                                     int sample_weak_votes,
                                     int sample_total_modules,
                                     double perspective_skew,
                                     double br_bias_magnitude)
{
    int dark = 0;
    int score = 0;
    int total = modules * modules;
    int ratio_penalty;

    if (!grid || modules <= 0 || total <= 0) {
        return -99999;
    }
    for (int i = 0; i < total; i++) {
        if (grid[i]) {
            dark++;
        }
    }
    ratio_penalty = abs(dark * 2 - total) / ((total / 10) + 1);

    score += 120;
    score -= ratio_penalty;
    score += (6 - (format_errors * 2));
    if (format_used_copy == 0) {
        score += 10;
    } else {
        score += 4;
    }

    if (version >= 7) {
        score += (6 - (version_errors * 2));
        if (version_used_copy == 0) {
            score += 8;
        } else {
            score += 3;
        }
        if (version_agree) {
            score += 10;
        }
    } else {
        score += 6;
    }

    if (alignment_used) {
        score += 6;
        score += (int)(align_score * 0.25);
        if (align_score >= g_qr_vote_tuning.align_score_strong) {
            score += 6;
        } else {
            score -= 4;
        }
    } else if (perspective_skew >= g_qr_vote_tuning.skew_bias_penalty_th &&
               br_bias_magnitude > 0.5) {
        score -= 8;
    }

    if (sample_total_modules > 0) {
        int oob_penalty = (sample_out_of_bounds * 28) / sample_total_modules;
        int weak_penalty = (sample_weak_votes * 14) / sample_total_modules;
        score -= oob_penalty;
        score -= weak_penalty;
    }

    return score;
}

static int qr_compose_vote_score(int format_used_copy, int format_errors,
                                 int version, int version_decode_ok,
                                 int version_agree, int version_errors,
                                 int alignment_used, double align_score,
                                 int sample_out_of_bounds,
                                 int sample_weak_votes,
                                 int sample_total_modules,
                                 double perspective_skew,
                                 double br_bias_magnitude)
{
    int score = 1;

    if (format_used_copy == 0 && format_errors <= 1) {
        score += 2;
    } else if (format_errors <= 2) {
        score += 1;
    }

    if (version < 7) {
        score += 1;
    } else if (version_decode_ok) {
        score += version_agree ? 2 : 1;
        if (version_errors >= 2) {
            score--;
        }
    }

    if (alignment_used) {
        if (align_score >= g_qr_vote_tuning.align_score_strong) {
            score += 2;
        } else if (align_score >= g_qr_vote_tuning.align_score_weak) {
            score += 1;
        }
    } else if (perspective_skew >= g_qr_vote_tuning.skew_bias_penalty_th &&
               br_bias_magnitude > 0.5) {
        score--;
    }

    if (sample_total_modules > 0) {
        int oob_pct = (sample_out_of_bounds * 100) / sample_total_modules;
        int weak_pct = (sample_weak_votes * 100) / sample_total_modules;
        if (oob_pct >= g_qr_vote_tuning.oob_penalty_percent) {
            score--;
        }
        if (weak_pct >= g_qr_vote_tuning.weak_vote_penalty_percent) {
            score--;
        }
    }

    if (score < 0) {
        score = 0;
    }
    return score;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Grid sampling
 * ════════════════════════════════════════════════════════════════════════ */

/* Bilinear interpolation on grayscale image at subpixel position */
static double bilinear_sample(const uint8_t *gray, int w, int h, double x, double y)
{
    int x0 = (int)x;
    int y0 = (int)y;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    double fx = x - (double)x0;
    double fy = y - (double)y0;
    if (fx < 0.0) fx = 0.0;
    if (fy < 0.0) fy = 0.0;
    double v00 = gray[y0 * w + x0];
    double v10 = gray[y0 * w + x1];
    double v01 = gray[y1 * w + x0];
    double v11 = gray[y1 * w + x1];
    return v00 * (1.0 - fx) * (1.0 - fy) +
           v10 * fx * (1.0 - fy) +
           v01 * (1.0 - fx) * fy +
           v11 * fx * fy;
}

static int sample_grid(const uint8_t *bin, int w, int h,
                       const Perspective *t, int modules,
                       int vote_cross, double jitter,
                       uint8_t *grid /* modules*modules */,
                       QRSampleMetrics *metrics)
{
    if (metrics) {
        memset(metrics, 0, sizeof(*metrics));
        metrics->total_modules = modules * modules;
    }

    for (int row = 0; row < modules; row++) {
        for (int col = 0; col < modules; col++) {
            if (!vote_cross || jitter <= 0.0) {
                double ix, iy;
                perspective_map(t, (double)col + 0.5, (double)row + 0.5, &ix, &iy);
                int px = (int)(ix + 0.5);
                int py = (int)(iy + 0.5);
                if (px < 0 || px >= w || py < 0 || py >= h) {
                    grid[row * modules + col] = 0;
                    if (metrics) {
                        metrics->out_of_bounds_modules++;
                    }
                } else {
                    grid[row * modules + col] = bin[py * w + px];
                }
            } else {
                static const double k_offsets[5][2] = {
                    {0.0, 0.0},
                    {-1.0, 0.0},
                    {1.0, 0.0},
                    {0.0, -1.0},
                    {0.0, 1.0},
                };
                int ones = 0;
                int valid = 0;
                for (int s = 0; s < 5; s++) {
                    double ix, iy;
                    int px, py;
                    double su = (double)col + 0.5 + k_offsets[s][0] * jitter;
                    double sv = (double)row + 0.5 + k_offsets[s][1] * jitter;
                    perspective_map(t, su, sv, &ix, &iy);
                    px = (int)(ix + 0.5);
                    py = (int)(iy + 0.5);
                    if (px < 0 || px >= w || py < 0 || py >= h) {
                        continue;
                    }
                    valid++;
                    if (bin[py * w + px]) {
                        ones++;
                    }
                }
                grid[row * modules + col] = (ones >= 3) ? 1 : 0;
                if (metrics) {
                    if (valid < 3) {
                        metrics->out_of_bounds_modules++;
                    } else if (ones == 2 || ones == 3) {
                        metrics->weak_vote_modules++;
                    }
                }
            }
        }
    }
    return 1;
}

/* Grayscale-aware grid sampling with bilinear interpolation.
 * Two-pass approach:
 *   Pass 1: sample grayscale value at each module center → gray_vals[]
 *   Pass 2: threshold using mean of 5×5 module neighborhood
 * This avoids binarization threshold errors and nearest-neighbor rounding. */
static int sample_grid_grayscale(const uint8_t *gray, int w, int h,
                                 const Perspective *t, int modules,
                                 uint8_t *grid, QRSampleMetrics *metrics)
{
    int total = modules * modules;
    double *gray_vals = (double *)calloc((size_t)total, sizeof(double));
    if (!gray_vals) return 0;

    if (metrics) {
        memset(metrics, 0, sizeof(*metrics));
        metrics->total_modules = total;
    }

    /* Pass 1: collect bilinear-interpolated grayscale at each module center */
    int oob = 0;
    for (int row = 0; row < modules; row++) {
        for (int col = 0; col < modules; col++) {
            double ix, iy;
            perspective_map(t, (double)col + 0.5, (double)row + 0.5, &ix, &iy);
            if (ix < 0.0 || ix >= (double)(w - 1) ||
                iy < 0.0 || iy >= (double)(h - 1)) {
                gray_vals[row * modules + col] = -1.0;
                oob++;
            } else {
                gray_vals[row * modules + col] = bilinear_sample(gray, w, h, ix, iy);
            }
        }
    }
    if (metrics) metrics->out_of_bounds_modules = oob;

    /* Pass 2: threshold each module using local 5×5 neighborhood mean */
    for (int row = 0; row < modules; row++) {
        for (int col = 0; col < modules; col++) {
            double val = gray_vals[row * modules + col];
            if (val < 0.0) {
                grid[row * modules + col] = 0;
                continue;
            }
            /* Compute mean of 5×5 neighborhood (or smaller at edges) */
            double sum = 0.0;
            int cnt = 0;
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    int nr = row + dy, nc = col + dx;
                    if (nr < 0 || nr >= modules || nc < 0 || nc >= modules) continue;
                    double nv = gray_vals[nr * modules + nc];
                    if (nv < 0.0) continue;
                    sum += nv;
                    cnt++;
                }
            }
            double threshold = (cnt > 0) ? (sum / (double)cnt) : 128.0;
            grid[row * modules + col] = (val < threshold) ? 1 : 0;
            if (metrics) {
                double margin = fabs(val - threshold);
                if (margin < 5.0) {
                    metrics->weak_vote_modules++;
                }
            }
        }
    }

    free(gray_vals);
    return 1;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Format info decoding
 * ════════════════════════════════════════════════════════════════════════ */

static const uint32_t FORMAT_INFO_MASK = 0x5412;

static const uint32_t format_info_decode_table[32] = {
    0x0000, 0x0537, 0x0A6E, 0x0F59, 0x11EB, 0x14DC, 0x1B85, 0x1EB2,
    0x23D6, 0x26E1, 0x29B8, 0x2C8F, 0x323D, 0x370A, 0x3853, 0x3D64,
    0x429B, 0x47AC, 0x48F5, 0x4DC2, 0x5370, 0x5647, 0x591E, 0x5C29,
    0x614D, 0x647A, 0x6B23, 0x6E14, 0x70A6, 0x7591, 0x7AC8, 0x7FFF,
};

typedef struct {
    int ok;
    int decoded;
    int errors;
    int used_unmask_fallback;
} QRFormatCandidate;

typedef struct {
    int ok;
    int version;
    int errors;
} QRVersionCandidate;

static int qr_popcount_u32(uint32_t x)
{
    int n = 0;
    while (x) {
        n++;
        x &= x - 1;
    }
    return n;
}

static QRFormatCandidate qr_decode_format_candidate(uint32_t bits)
{
    QRFormatCandidate out;
    out.ok = 0;
    out.decoded = 0;
    out.errors = 15;
    out.used_unmask_fallback = 0;

    /* Try unmasked first (standard QR codes) */
    uint32_t unmasked_bits = bits ^ FORMAT_INFO_MASK;
    QR_DBG("[QRDEBUG FMT] Input bits=0x%04x, unmasked=0x%04x\n", bits, unmasked_bits);
    for (int i = 0; i < 32; i++) {
        int errors = qr_popcount_u32(unmasked_bits ^ format_info_decode_table[i]);
        if (errors < out.errors) {
            out.errors = errors;
            out.decoded = i;
        }
    }
    QR_DBG("[QRDEBUG FMT] Best match: decoded=%d (ec=%d, mask=%d), errors=%d\n",
            out.decoded, out.decoded >> 3, out.decoded & 7, out.errors);
    if (out.errors <= 3) {
        out.ok = 1;
        return out;
    }

    /* Fallback: try raw bits (for non-standard or already-unmasked input) */
    out.errors = 15;
    for (int i = 0; i < 32; i++) {
        int errors = qr_popcount_u32(bits ^ format_info_decode_table[i]);
        if (errors < out.errors) {
            out.errors = errors;
            out.decoded = i;
        }
    }
    if (out.errors <= 3) {
        out.ok = 1;
        out.used_unmask_fallback = 1;
    }
    return out;
}

static int qr_choose_format_result(uint32_t bits_primary, uint32_t bits_secondary,
                                   int *ec_level, int *mask_pattern,
                                   int *used_copy, int *bit_errors)
{
    QRFormatCandidate a = qr_decode_format_candidate(bits_primary);
    QRFormatCandidate b = qr_decode_format_candidate(bits_secondary);
    QRFormatCandidate best;
    int source = 0;

    if (!a.ok && !b.ok) return 0;

    if (a.ok && b.ok) {
        if (a.decoded == b.decoded) {
            best = (a.errors <= b.errors) ? a : b;
            source = 0;
        } else if (a.errors < b.errors) {
            best = a;
            source = 1;
        } else if (b.errors < a.errors) {
            best = b;
            source = 2;
        } else if (a.used_unmask_fallback != b.used_unmask_fallback) {
            best = a.used_unmask_fallback ? b : a;
            source = a.used_unmask_fallback ? 2 : 1;
        } else {
            best = a;
            source = 1;
        }
    } else if (a.ok) {
        best = a;
        source = 1;
    } else {
        best = b;
        source = 2;
    }

    if (ec_level) *ec_level = best.decoded >> 3;
    if (mask_pattern) *mask_pattern = best.decoded & 7;
    if (used_copy) *used_copy = source;
    if (bit_errors) *bit_errors = best.errors;
    return 1;
}

static int decode_format_info_ex(const uint8_t *grid, int modules,
                                 int *ec_level, int *mask_pattern,
                                 int *used_copy, int *bit_errors)
{
    /* Read format info bits from two locations */
    uint32_t bits1 = 0, bits2 = 0;

    /* Location 1: around top-left finder */
    /* bits 0-7 along row 8 (cols 0-7, skipping col 6) */
    int bit_index = 0;
    for (int x = 0; x <= 8; x++) {
        if (x == 6) continue;
        if (grid[8 * modules + x]) bits1 |= (1u << bit_index);
        bit_index++;
    }
    /* bits 8-14 along col 8 (rows 7 down to 0, skipping row 6) */
    for (int y = 7; y >= 0; y--) {
        if (y == 6) continue;
        if (grid[y * modules + 8]) bits1 |= (1u << bit_index);
        bit_index++;
    }

    /* Location 2: around TR and BL finders */
    bit_index = 0;
    for (int y = modules - 1; y >= modules - 7; y--) {
        if (grid[y * modules + 8]) bits2 |= (1u << bit_index);
        bit_index++;
    }
    for (int x = modules - 8; x < modules; x++) {
        if (grid[8 * modules + x]) bits2 |= (1u << bit_index);
        bit_index++;
    }

    QR_DBG("[QRDEBUG] Format bits: loc1=0x%04x, loc2=0x%04x\n", bits1, bits2);
    return qr_choose_format_result(bits1, bits2, ec_level, mask_pattern, used_copy, bit_errors);
}

/* ════════════════════════════════════════════════════════════════════════
 *  Version info decoding (V7+)
 * ════════════════════════════════════════════════════════════════════════ */

static QRVersionCandidate qr_decode_version_candidate(uint32_t bits)
{
    QRVersionCandidate out;
    out.ok = 0;
    out.version = -1;
    out.errors = 18;

    for (int v = 7; v <= 40; v++) {
        int errors = qr_popcount_u32(bits ^ version_info_bits[v - 7]);
        if (errors < out.errors) {
            out.errors = errors;
            out.version = v;
        }
    }
    if (out.errors <= 3) {
        out.ok = 1;
    }
    return out;
}

static int qr_choose_version_result(uint32_t bits_primary, uint32_t bits_secondary,
                                    int estimated_version, int *version_out,
                                    int *used_copy, int *bit_errors, int *agree)
{
    QRVersionCandidate a = qr_decode_version_candidate(bits_primary);
    QRVersionCandidate b = qr_decode_version_candidate(bits_secondary);
    QRVersionCandidate best;
    int source = 0;
    int is_agree = 0;

    if (!a.ok && !b.ok) return 0;

    if (a.ok && b.ok) {
        if (a.version == b.version) {
            best = (a.errors <= b.errors) ? a : b;
            is_agree = 1;
            source = 0;
        } else if (a.errors < b.errors) {
            best = a;
            source = 1;
        } else if (b.errors < a.errors) {
            best = b;
            source = 2;
        } else {
            int da = abs(a.version - estimated_version);
            int db = abs(b.version - estimated_version);
            if (db < da) {
                best = b;
                source = 2;
            } else {
                best = a;
                source = 1;
            }
        }
    } else if (a.ok) {
        best = a;
        source = 1;
    } else {
        best = b;
        source = 2;
    }

    if (version_out) *version_out = best.version;
    if (used_copy) *used_copy = source;
    if (bit_errors) *bit_errors = best.errors;
    if (agree) *agree = is_agree;
    return 1;
}

static int decode_version_info_ex(const uint8_t *grid, int modules,
                                  int *used_copy, int *bit_errors, int *agree)
{
    /* Read from bottom-left area (6x3 block near BL finder) */
    uint32_t bits1 = 0;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            if (grid[(modules - 11 + j) * modules + i])
                bits1 |= (1u << (i * 3 + j));
        }
    }

    /* Read from top-right area (3x6 block near TR finder) */
    uint32_t bits2 = 0;
    for (int j = 0; j < 6; j++) {
        for (int i = 0; i < 3; i++) {
            if (grid[j * modules + (modules - 11 + i)])
                bits2 |= (1u << (j * 3 + i));
        }
    }

    {
        int best_version = -1;
        int estimated_version = (modules - 17) / 4;
        if (!qr_choose_version_result(bits1, bits2, estimated_version, &best_version,
                                      used_copy, bit_errors, agree)) {
            return -1;
        }
        return best_version;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Mask application
 * ════════════════════════════════════════════════════════════════════════ */

static int mask_bit(int mask, int row, int col)
{
    switch (mask) {
    case 0: return ((row + col) % 2) == 0;
    case 1: return (row % 2) == 0;
    case 2: return (col % 3) == 0;
    case 3: return ((row + col) % 3) == 0;
    case 4: return ((row / 2 + col / 3) % 2) == 0;
    case 5: return ((row * col) % 2 + (row * col) % 3) == 0;
    case 6: return (((row * col) % 2 + (row * col) % 3) % 2) == 0;
    case 7: return (((row + col) % 2 + (row * col) % 3) % 2) == 0;
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Function pattern marking (which modules are NOT data)
 * ════════════════════════════════════════════════════════════════════════ */

static void mark_function_patterns(uint8_t *func, int modules, int version)
{
    /* Finder patterns + separators */
    for (int r = 0; r < 9; r++) {
        for (int c = 0; c < 9; c++) {
            if (r < modules && c < modules) func[r * modules + c] = 1; /* TL */
        }
    }
    for (int r = 0; r < 9; r++) {
        for (int c = modules - 8; c < modules; c++) {
            if (r < modules && c >= 0) func[r * modules + c] = 1; /* TR */
        }
    }
    for (int r = modules - 8; r < modules; r++) {
        for (int c = 0; c < 9; c++) {
            if (r >= 0 && c < modules) func[r * modules + c] = 1; /* BL */
        }
    }

    /* Alignment patterns */
    if (version >= 2) {
        int cnt = align_count(version);
        for (int i = 0; i < cnt; i++) {
            for (int j = 0; j < cnt; j++) {
                int ay = align_pos[version - 1][i];
                int ax = align_pos[version - 1][j];
                if (ay == 0 || ax == 0) continue;
                /* Skip if overlapping finder */
                if (ay <= 8 && ax <= 8) continue;
                if (ay <= 8 && ax >= modules - 8) continue;
                if (ay >= modules - 8 && ax <= 8) continue;
                for (int dr = -2; dr <= 2; dr++) {
                    for (int dc = -2; dc <= 2; dc++) {
                        int rr = ay + dr, cc = ax + dc;
                        if (rr >= 0 && rr < modules && cc >= 0 && cc < modules)
                            func[rr * modules + cc] = 1;
                    }
                }
            }
        }
    }

    /* Timing patterns */
    for (int i = 8; i < modules - 8; i++) {
        func[6 * modules + i] = 1;
        func[i * modules + 6] = 1;
    }

    /* Dark module */
    func[(4 * version + 9) * modules + 8] = 1;

    /* Version info areas (V7+) */
    if (version >= 7) {
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 3; j++) {
                func[(modules - 11 + j) * modules + i] = 1;
                func[i * modules + (modules - 11 + j)] = 1;
            }
        }
    }

    /* Format info areas (already covered by finder regions, but be explicit) */
    for (int i = 0; i < 9; i++) {
        if (i < modules) func[8 * modules + i] = 1;
        if (i < modules) func[i * modules + 8] = 1;
    }
    for (int i = modules - 8; i < modules; i++) {
        func[8 * modules + i] = 1;
    }
    for (int i = modules - 7; i < modules; i++) {
        func[i * modules + 8] = 1;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Read codewords from grid
 * ════════════════════════════════════════════════════════════════════════ */

static int read_codewords(const uint8_t *grid, const uint8_t *func,
                          int modules, int mask,
                          uint8_t *codewords, int max_cw)
{
    int n_cw = 0;
    int bit_idx = 0;
    uint8_t cur_byte = 0;

    int col = modules - 1;
    int going_up = 1;

    while (col > 0) {
        if (col == 6) col--; /* skip timing column */

        for (int cnt = 0; cnt < modules; cnt++) {
            int row = going_up ? (modules - 1 - cnt) : cnt;

            for (int dx = 0; dx <= 1; dx++) {
                int c = col - dx;
                if (c < 0 || c >= modules || row < 0 || row >= modules) continue;
                if (func[row * modules + c]) continue;

                int bit = grid[row * modules + c];
                if (mask_bit(mask, row, c)) bit ^= 1;

                cur_byte = (uint8_t)((cur_byte << 1) | (bit & 1));
                bit_idx++;

                if (bit_idx == 8) {
                    if (n_cw >= max_cw) return n_cw;
                    codewords[n_cw++] = cur_byte;
                    cur_byte = 0;
                    bit_idx = 0;
                }
            }
        }

        going_up = !going_up;
        col -= 2;
    }

    return n_cw;
}

/* ════════════════════════════════════════════════════════════════════════
 *  De-interleave and error-correct codewords
 * ════════════════════════════════════════════════════════════════════════ */

static int deinterleave_and_correct(uint8_t *codewords, int total_cw,
                                    int version, int ec_level,
                                    uint8_t *data_out, int *data_len)
{
    if (version < 1 || version > 40) return 0;

    int col = ec_level_to_col(ec_level);
    const ECInfo *info = &ec_table[version - 1][col];

    int total_blocks = info->num_blocks_g1 + info->num_blocks_g2;
    if (total_blocks <= 0 || total_blocks > 100) return 0;

    int ec_per_block = info->ec_codewords_per_block;
    int data_cw_g1 = info->data_cw_per_block_g1;
    int data_cw_g2 = info->num_blocks_g2 > 0 ? info->data_cw_per_block_g2 : data_cw_g1;

    /* De-interleave data codewords */
    uint8_t *blocks = (uint8_t *)calloc(100U * 256U, 1);
    int *block_data_len = (int *)calloc(100U, sizeof(*block_data_len));
    int *block_total_len = (int *)calloc(100U, sizeof(*block_total_len));
    if (!blocks || !block_data_len || !block_total_len) {
        free(blocks);
        free(block_data_len);
        free(block_total_len);
        return 0;
    }

    for (int i = 0; i < info->num_blocks_g1; i++) {
        block_data_len[i] = data_cw_g1;
        block_total_len[i] = data_cw_g1 + ec_per_block;
    }
    for (int i = 0; i < info->num_blocks_g2; i++) {
        int idx = info->num_blocks_g1 + i;
        block_data_len[idx] = data_cw_g2;
        block_total_len[idx] = data_cw_g2 + ec_per_block;
    }

    /* De-interleave: data codewords come first (interleaved), then EC codewords */
    int max_data_per_block = (data_cw_g2 > data_cw_g1) ? data_cw_g2 : data_cw_g1;
    int pos = 0;

    for (int i = 0; i < max_data_per_block; i++) {
        for (int b = 0; b < total_blocks; b++) {
            if (i < block_data_len[b]) {
                if (pos < total_cw)
                    blocks[b * 256 + i] = codewords[pos++];
            }
        }
    }

    for (int i = 0; i < ec_per_block; i++) {
        for (int b = 0; b < total_blocks; b++) {
            if (pos < total_cw)
                blocks[b * 256 + block_data_len[b] + i] = codewords[pos++];
        }
    }

    /* Error correct each block */
    int out_pos = 0;
    for (int b = 0; b < total_blocks; b++) {
        if (!rs_correct(&blocks[b * 256], block_total_len[b], ec_per_block)) {
            free(blocks);
            free(block_data_len);
            free(block_total_len);
            return 0;
        }
        for (int i = 0; i < block_data_len[b]; i++) {
            data_out[out_pos++] = blocks[b * 256 + i];
        }
    }

    *data_len = out_pos;
    free(blocks);
    free(block_data_len);
    free(block_total_len);
    return 1;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Data stream decoding
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct {
    const uint8_t *data;
    int total_bits;
    int pos;
} BitStream;

static int bs_read(BitStream *bs, int n)
{
    if (n <= 0 || bs->pos + n > bs->total_bits) return -1;
    int val = 0;
    for (int i = 0; i < n; i++) {
        int byte_idx = (bs->pos + i) / 8;
        int bit_idx = 7 - ((bs->pos + i) % 8);
        val = (val << 1) | ((bs->data[byte_idx] >> bit_idx) & 1);
    }
    bs->pos += n;
    return val;
}

static int char_count_bits(int mode, int version)
{
    /* Number of bits in character count indicator */
    if (version <= 9) {
        switch (mode) {
        case 1: return 10; /* numeric */
        case 2: return 9;  /* alphanumeric */
        case 4: return 8;  /* byte */
        case 8: return 8;  /* kanji */
        }
    } else if (version <= 26) {
        switch (mode) {
        case 1: return 12;
        case 2: return 11;
        case 4: return 16;
        case 8: return 10;
        }
    } else {
        switch (mode) {
        case 1: return 14;
        case 2: return 13;
        case 4: return 16;
        case 8: return 12;
        }
    }
    return 8;
}

static const char ALPHANUMERIC_TABLE[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:";

static int decode_data_stream(const uint8_t *data, int data_len, int version,
                              char *out, int out_cap)
{
    BitStream bs;
    bs.data = data;
    bs.total_bits = data_len * 8;
    bs.pos = 0;

    int out_pos = 0;

    while (bs.pos + 4 <= bs.total_bits) {
        int mode = bs_read(&bs, 4);
        if (mode < 0) break;

        /* Modes that have no character count field */
        if (mode == 0) break; /* terminator */
        if (mode == 5) continue; /* FNC1 first position — no payload */
        if (mode == 9) {
            /* FNC1 second position — 8-bit application indicator */
            bs_read(&bs, 8);
            continue;
        }
        if (mode == 7) {
            /* ECI — read indicator and continue */
            int eci = bs_read(&bs, 8);
            (void)eci;
            continue;
        }

        int cc_bits = char_count_bits(mode, version);
        int count = bs_read(&bs, cc_bits);
        if (count < 0) break;

        if (mode == 1) {
            /* Numeric */
            int remaining = count;
            while (remaining >= 3) {
                int triple = bs_read(&bs, 10);
                if (triple < 0) return 0;
                if (out_pos + 3 >= out_cap) return 0;
                out[out_pos++] = (char)('0' + triple / 100);
                out[out_pos++] = (char)('0' + (triple / 10) % 10);
                out[out_pos++] = (char)('0' + triple % 10);
                remaining -= 3;
            }
            if (remaining == 2) {
                int pair = bs_read(&bs, 7);
                if (pair < 0) return 0;
                if (out_pos + 2 >= out_cap) return 0;
                out[out_pos++] = (char)('0' + pair / 10);
                out[out_pos++] = (char)('0' + pair % 10);
            } else if (remaining == 1) {
                int single = bs_read(&bs, 4);
                if (single < 0) return 0;
                if (out_pos + 1 >= out_cap) return 0;
                out[out_pos++] = (char)('0' + single);
            }
        } else if (mode == 2) {
            /* Alphanumeric */
            int remaining = count;
            while (remaining >= 2) {
                int pair = bs_read(&bs, 11);
                if (pair < 0) return 0;
                int c1 = pair / 45;
                int c2 = pair % 45;
                if (c1 < 0 || c1 > 44 || c2 < 0 || c2 > 44) return 0;
                if (out_pos + 2 >= out_cap) return 0;
                out[out_pos++] = ALPHANUMERIC_TABLE[c1];
                out[out_pos++] = ALPHANUMERIC_TABLE[c2];
                remaining -= 2;
            }
            if (remaining == 1) {
                int single = bs_read(&bs, 6);
                if (single < 0 || single > 44) return 0;
                if (out_pos + 1 >= out_cap) return 0;
                out[out_pos++] = ALPHANUMERIC_TABLE[single];
            }
        } else if (mode == 4) {
            /* Byte */
            for (int i = 0; i < count; i++) {
                int b = bs_read(&bs, 8);
                if (b < 0) return 0;
                if (out_pos + 1 >= out_cap) return 0;
                out[out_pos++] = (char)b;
            }
        } else if (mode == 8) {
            /* Kanji - store as raw bytes (Shift-JIS encoded) */
            for (int i = 0; i < count; i++) {
                int val = bs_read(&bs, 13);
                if (val < 0) return 0;
                int byte_val;
                if (val + 0x8140 <= 0x9FFC) {
                    byte_val = val + 0x8140;
                } else {
                    byte_val = val + 0xC140;
                }
                int hi = (byte_val >> 8) & 0xFF;
                int lo = byte_val & 0xFF;
                if (out_pos + 2 >= out_cap) return 0;
                out[out_pos++] = (char)hi;
                out[out_pos++] = (char)lo;
            }
        } else {
            break; /* unknown mode */
        }
    }

    if (out_pos < out_cap) out[out_pos] = '\0';
    return out_pos;
}

/* Flip binary image horizontally (in-place) for mirror QR support */
static void flip_horizontal(uint8_t *buf, int w, int h)
{
    for (int y = 0; y < h; y++) {
        uint8_t *row = buf + y * w;
        for (int x = 0; x < w / 2; x++) {
            uint8_t tmp = row[x];
            row[x] = row[w - 1 - x];
            row[w - 1 - x] = tmp;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  MicroQR decoder (M2-M4, byte/alphanumeric/numeric modes)
 * ════════════════════════════════════════════════════════════════════════ */

/* MicroQR format info: 15 bits, BCH(15,5), mask 0x4445 */
#define MQR_FORMAT_MASK 0x4445

/* MicroQR version table: {side, total_cw, data_cw, ec_cw} per (version, ec_level)
 * M1 omitted (needs GF(16)). M2-M4 use GF(256). */
typedef struct { int side; int total_cw; int data_cw; int ec_cw; } MQRVersionInfo;

/* symbol_number (from format info bits 2..4): maps to version+ec
 * 0=M1,  1=M2-L,  2=M2-M,  3=M3-L,  4=M3-M,  5=M4-L,  6=M4-M,  7=M4-Q */
static const MQRVersionInfo mqr_versions[] = {
    /* 0: M1 */    {11,  5,  3,  2},
    /* 1: M2-L */  {13, 10,  5,  5},
    /* 2: M2-M */  {13, 10,  4,  6},
    /* 3: M3-L */  {15, 17, 11,  6},
    /* 4: M3-M */  {15, 17,  9,  8},
    /* 5: M4-L */  {17, 24, 16,  8},
    /* 6: M4-M */  {17, 24, 14, 10},
    /* 7: M4-Q */  {17, 24, 10, 14},
};

/* Character count indicator bits for MicroQR modes */
static int mqr_cc_bits(int symbol_num, int mode)
{
    /* mode: 1=numeric, 2=alphanum, 4=byte, 8=kanji */
    /* Rows: symbol_num 1..7 (M2-L through M4-Q) */
    static const int table[7][4] = {
        /* num,alph,byte,kanji  (ISO 18004 Table 3) */
        {4, 3, 0, 0},  /* sym 1: M2-L */
        {4, 3, 0, 0},  /* sym 2: M2-M */
        {5, 4, 4, 3},  /* sym 3: M3-L */
        {5, 4, 4, 3},  /* sym 4: M3-M */
        {6, 5, 5, 4},  /* sym 5: M4-L */
        {6, 5, 5, 4},  /* sym 6: M4-M */
        {6, 5, 5, 4},  /* sym 7: M4-Q */
    };
    if (symbol_num < 1 || symbol_num > 7) return 0;
    int col;
    switch (mode) {
    case 1: col = 0; break; /* numeric */
    case 2: col = 1; break; /* alphanumeric */
    case 4: col = 2; break; /* byte */
    case 8: col = 3; break; /* kanji */
    default: return 0;
    }
    return table[symbol_num - 1][col];
}

/* Mode indicator length in MicroQR: version-dependent
 * M1=0, M2=1, M3=2, M4=3 bits */
static int mqr_mode_bits(int symbol_num)
{
    if (symbol_num <= 0) return 0;
    if (symbol_num <= 2) return 1; /* M2 */
    if (symbol_num <= 4) return 2; /* M3 */
    return 3;                       /* M4 */
}

/* Sample a single module from the binary image given grid origin and module size */
static int mqr_sample(const uint8_t *bin, int w, int h,
                      double ox, double oy, double ms, int col, int row)
{
    int px = (int)(ox + (col + 0.5) * ms + 0.5);
    int py = (int)(oy + (row + 0.5) * ms + 0.5);
    if (px < 0 || px >= w || py < 0 || py >= h) return 0;
    return bin[py * w + px];
}

/* Count timing transitions along a line to estimate module count */
static int mqr_count_timing(const uint8_t *bin, int w, int h,
                            double ox, double oy, double ms, int is_horizontal)
{
    /* Timing runs from module 8 to the edge.
     * Try versions M1(11), M2(13), M3(15), M4(17) and pick best match. */
    static const int sides[] = {11, 13, 15, 17};
    int best_side = 0;
    int best_score = -1;

    for (int si = 0; si < 4; si++) {
        int side = sides[si];
        int score = 0;
        /* Check timing: alternating dark/light starting dark at module 8 */
        for (int i = 8; i < side; i++) {
            int expected = (i % 2 == 0) ? 1 : 0; /* even=dark, odd=light */
            int val;
            if (is_horizontal)
                val = mqr_sample(bin, w, h, ox, oy, ms, i, 0);
            else
                val = mqr_sample(bin, w, h, ox, oy, ms, 0, i);
            if (val == expected) score++;
        }
        if (score > best_score) {
            best_score = score;
            best_side = side;
        }
    }
    return best_side;
}

/* Read MicroQR format information (15 bits) */
static int mqr_read_format(const uint8_t *bin, int w, int h,
                           double ox, double oy, double ms, int side,
                           int *symbol_num, int *mask_pat)
{
    (void)side;
    /* Format info positions around the finder pattern.
     * 15 bits read from: row 8, cols 1-8 then col 8, rows 7 down to 1.
     * Bit 0 (MSB of format) at (8,1), ..., bit 7 at (8,8),
     * bit 8 at (7,8), ..., bit 14 at (1,8) */
    int format_bits = 0;
    /* First 8 bits: row 8, columns 1 through 8 */
    for (int i = 0; i < 8; i++) {
        int val = mqr_sample(bin, w, h, ox, oy, ms, 1 + i, 8);
        format_bits = (format_bits << 1) | (val ? 1 : 0);
    }
    /* Next 7 bits: column 8, rows 7 down through 1 */
    for (int i = 0; i < 7; i++) {
        int val = mqr_sample(bin, w, h, ox, oy, ms, 8, 7 - i);
        format_bits = (format_bits << 1) | (val ? 1 : 0);
    }

    format_bits ^= MQR_FORMAT_MASK;

    /* Try all 32 valid format info values and pick closest Hamming distance */
    /* The 5 data bits encode: bits[0..2]=symbol_number, bits[3..4]=data_mask */
    int best_dist = 16;
    int best_data = -1;

    for (int data5 = 0; data5 < 32; data5++) {
        /* BCH(15,5) encode: generator = x^10 + x^8 + x^5 + x^4 + x^2 + x + 1 = 0x537 */
        int codeword = data5 << 10;
        for (int i = 4; i >= 0; i--) {
            if (codeword & (1 << (i + 10)))
                codeword ^= (0x537 << i);
        }
        codeword = (data5 << 10) | codeword;

        /* Hamming distance */
        int diff = (format_bits ^ codeword) & 0x7FFF;
        int dist = 0;
        while (diff) { dist += diff & 1; diff >>= 1; }

        if (dist < best_dist) {
            best_dist = dist;
            best_data = data5;
        }
    }

    if (best_dist > 3 || best_data < 0) return 0;

    *symbol_num = (best_data >> 2) & 0x07;
    *mask_pat = best_data & 0x03;

    QR_DBG("[MQR] format: symbol_num=%d mask=%d (hamming=%d)\n",
           *symbol_num, *mask_pat, best_dist);
    return 1;
}

/* Apply MicroQR data mask (4 patterns, 0-3) */
static void mqr_apply_mask(uint8_t *grid, int side, int mask)
{
    for (int r = 0; r < side; r++) {
        for (int c = 0; c < side; c++) {
            /* Skip function patterns: finder(0-8,0-8), timing(r0 c>=8, c0 r>=8), format */
            if (r <= 8 && c <= 8) continue;  /* finder + separator + format area */
            if (r == 0 && c >= 8) continue;   /* horizontal timing */
            if (c == 0 && r >= 8) continue;   /* vertical timing */

            int flip = 0;
            switch (mask) {
            case 0: flip = (r % 2 == 0); break;
            case 1: flip = ((r / 2 + c / 3) % 2 == 0); break;
            case 2: flip = (((r * c) % 2 + (r * c) % 3) % 2 == 0); break;
            case 3: flip = (((r + c) % 2 + (r * c) % 3) % 2 == 0); break;
            }
            if (flip) grid[r * side + c] ^= 1;
        }
    }
}

/* Check if module is a data module (not function pattern) */
static int mqr_is_data(int r, int c, int side)
{
    (void)side;
    if (r <= 8 && c <= 8) return 0;   /* finder + separator + format */
    if (r == 0 && c >= 8) return 0;   /* horizontal timing */
    if (c == 0 && r >= 8) return 0;   /* vertical timing */
    return 1;
}

/* Extract data codewords from MicroQR grid in zigzag order.
 * MicroQR uses a 2-column-wide upward/downward zigzag like QR,
 * but starts from bottom-right and skips column 0 (timing). */
static int mqr_extract_codewords(const uint8_t *grid, int side,
                                 uint8_t *codewords, int max_cw)
{
    int bit_idx = 0;
    int cw_idx = 0;
    uint8_t cur_byte = 0;

    /* Process 2-column bands from right to left, skipping col 0 (timing) */
    for (int right = side - 1; right >= 1; right -= 2) {
        int left = right - 1;
        if (left < 1) left = 1; /* should not happen for valid MicroQR */

        /* Determine direction: rightmost band goes up, then alternates */
        int band_index = (side - 1 - right) / 2;
        int going_up = (band_index % 2 == 0);

        for (int i = 0; i < side; i++) {
            int r = going_up ? (side - 1 - i) : i;
            /* Right column first, then left */
            for (int dx = 0; dx <= 1; dx++) {
                int c = (dx == 0) ? right : left;
                if (c < 1) continue;
                if (!mqr_is_data(r, c, side)) continue;

                cur_byte = (cur_byte << 1) | (grid[r * side + c] & 1);
                bit_idx++;
                if (bit_idx == 8) {
                    if (cw_idx >= max_cw) return cw_idx;
                    codewords[cw_idx++] = cur_byte;
                    cur_byte = 0;
                    bit_idx = 0;
                }
            }
        }
    }
    /* Flush partial byte */
    if (bit_idx > 0 && cw_idx < max_cw) {
        codewords[cw_idx++] = cur_byte << (8 - bit_idx);
    }
    return cw_idx;
}

/* Decode MicroQR data stream (different mode indicator sizes than QR) */
static int mqr_decode_data(const uint8_t *data, int data_len,
                           int symbol_num, char *out, int out_cap)
{
    BitStream bs;
    bs.data = data;
    bs.total_bits = data_len * 8;
    bs.pos = 0;
    int out_pos = 0;
    int mb = mqr_mode_bits(symbol_num);

    while (bs.pos + mb <= bs.total_bits) {
        int mode_raw = (mb > 0) ? bs_read(&bs, mb) : 0;
        if (mode_raw < 0) break;

        /* Map MicroQR mode indicator to standard mode number.
         * M2: 0=num, 1=alph (1 bit)
         * M3: 00=num, 01=alph, 10=byte, 11=kanji (2 bits)
         * M4: 000=num, 001=alph, 010=byte, 011=kanji, 000=term... (3 bits) */
        int mode;
        if (symbol_num <= 2) {
            /* M2: 1-bit mode */
            if (mode_raw == 0) mode = 1;       /* numeric */
            else mode = 2;                      /* alphanumeric */
        } else if (symbol_num <= 4) {
            /* M3: 2-bit mode */
            if (mode_raw == 0) mode = 1;
            else if (mode_raw == 1) mode = 2;
            else if (mode_raw == 2) mode = 4;  /* byte */
            else mode = 8;                      /* kanji */
        } else {
            /* M4: 3-bit mode */
            if (mode_raw == 0) {
                /* Check if remaining bits are all zero (terminator) */
                if (bs.pos >= bs.total_bits) break;
                mode = 1; /* numeric */
            }
            else if (mode_raw == 1) mode = 2;
            else if (mode_raw == 2) mode = 4;
            else if (mode_raw == 3) mode = 8;
            else break; /* terminator or unknown */
        }

        int cc = mqr_cc_bits(symbol_num, mode);
        if (cc == 0) break; /* mode not supported for this version */
        int count = bs_read(&bs, cc);
        if (count < 0 || count == 0) break;

        if (mode == 4) {
            /* Byte mode */
            for (int i = 0; i < count; i++) {
                int b = bs_read(&bs, 8);
                if (b < 0) return 0;
                if (out_pos + 1 >= out_cap) return 0;
                out[out_pos++] = (char)b;
            }
        } else if (mode == 1) {
            /* Numeric */
            int rem = count;
            while (rem >= 3) {
                int t = bs_read(&bs, 10);
                if (t < 0) return 0;
                if (out_pos + 3 >= out_cap) return 0;
                out[out_pos++] = (char)('0' + t / 100);
                out[out_pos++] = (char)('0' + (t / 10) % 10);
                out[out_pos++] = (char)('0' + t % 10);
                rem -= 3;
            }
            if (rem == 2) {
                int p = bs_read(&bs, 7);
                if (p < 0) return 0;
                if (out_pos + 2 >= out_cap) return 0;
                out[out_pos++] = (char)('0' + p / 10);
                out[out_pos++] = (char)('0' + p % 10);
            } else if (rem == 1) {
                int s = bs_read(&bs, 4);
                if (s < 0) return 0;
                if (out_pos + 1 >= out_cap) return 0;
                out[out_pos++] = (char)('0' + s);
            }
        } else if (mode == 2) {
            /* Alphanumeric */
            int rem = count;
            while (rem >= 2) {
                int p = bs_read(&bs, 11);
                if (p < 0) return 0;
                if (out_pos + 2 >= out_cap) return 0;
                out[out_pos++] = ALPHANUMERIC_TABLE[p / 45];
                out[out_pos++] = ALPHANUMERIC_TABLE[p % 45];
                rem -= 2;
            }
            if (rem == 1) {
                int s = bs_read(&bs, 6);
                if (s < 0 || s > 44) return 0;
                if (out_pos + 1 >= out_cap) return 0;
                out[out_pos++] = ALPHANUMERIC_TABLE[s];
            }
        } else if (mode == 8) {
            /* Kanji */
            for (int i = 0; i < count; i++) {
                int val = bs_read(&bs, 13);
                if (val < 0) return 0;
                int bv = (val + 0x8140 <= 0x9FFC) ? val + 0x8140 : val + 0xC140;
                if (out_pos + 2 >= out_cap) return 0;
                out[out_pos++] = (char)((bv >> 8) & 0xFF);
                out[out_pos++] = (char)(bv & 0xFF);
            }
        } else {
            break;
        }
    }

    if (out_pos < out_cap) out[out_pos] = '\0';
    return out_pos;
}

static int try_microqr_decode(const uint8_t *bin, int width, int height,
                              BarcodeResult *result)
{
    /* Step 1: Find finder patterns — need exactly 1 for MicroQR */
    FinderCenter finders[MAX_FINDERS];
    int n_finders = find_finder_patterns(bin, width, height, finders, MAX_FINDERS);
    if (n_finders != 1) return 0;

    FinderCenter *fp = &finders[0];
    double ms = fp->module_size;
    if (ms < 1.0) return 0;

    /* The finder center is at module (3.5, 3.5) in the MicroQR grid.
     * Origin (top-left corner of module 0,0) = center - 3.5 * ms */
    double ox = fp->cx - 3.5 * ms;
    double oy = fp->cy - 3.5 * ms;

    /* Step 2: Determine grid size from timing patterns */
    int side_h = mqr_count_timing(bin, width, height, ox, oy, ms, 1);
    int side_v = mqr_count_timing(bin, width, height, ox, oy, ms, 0);
    int side = (side_h + side_v + 1) / 2; /* average, round */

    /* Snap to valid MicroQR sizes */
    if (side <= 12) side = 11;
    else if (side <= 14) side = 13;
    else if (side <= 16) side = 15;
    else side = 17;

    QR_DBG("[MQR] side=%d (h=%d v=%d) ms=%.2f\n", side, side_h, side_v, ms);

    /* Step 3: Read format info */
    int symbol_num = 0, mask_pat = 0;
    if (!mqr_read_format(bin, width, height, ox, oy, ms, side, &symbol_num, &mask_pat))
        return 0;

    /* Skip M1 (symbol_num=0): needs GF(16) */
    if (symbol_num == 0) return 0;
    if (symbol_num > 7) return 0;

    const MQRVersionInfo *vi = &mqr_versions[symbol_num];

    /* Verify side matches version */
    if (side != vi->side) {
        QR_DBG("[MQR] side mismatch: timing says %d, format says %d\n", side, vi->side);
        /* Trust format info */
        side = vi->side;
    }

    /* Step 4: Sample grid */
    uint8_t grid[17 * 17];
    memset(grid, 0, sizeof(grid));
    for (int r = 0; r < side; r++) {
        for (int c = 0; c < side; c++) {
            grid[r * side + c] = mqr_sample(bin, width, height, ox, oy, ms, c, r) ? 1 : 0;
        }
    }

    /* Try all 4 masks — the format-info mask may be correct or we may need
     * to brute-force due to grid sampling imprecision. */
    gf_init();
    for (int try_mask = 0; try_mask < 4; try_mask++) {
        /* Work on a copy of the grid */
        uint8_t mgrid[17 * 17];
        memcpy(mgrid, grid, (size_t)(side * side));

        mqr_apply_mask(mgrid, side, try_mask);

        uint8_t codewords[32];
        memset(codewords, 0, sizeof(codewords));
        int n_cw = mqr_extract_codewords(mgrid, side, codewords, vi->total_cw);
        if (n_cw < vi->total_cw) {
            for (int i = n_cw; i < vi->total_cw; i++) codewords[i] = 0;
        }

        int rs_ok = rs_correct(codewords, vi->total_cw, vi->ec_cw);
        QR_DBG("[MQR] mask=%d rs=%s cw[0..3]=%02X %02X %02X %02X\n",
               try_mask, rs_ok?"ok":"FAIL", codewords[0], codewords[1], codewords[2], codewords[3]);
        if (!rs_ok) continue;

        char decoded[BARCODE_MAX_LEN];
        int dec_len = mqr_decode_data(codewords, vi->data_cw, symbol_num,
                                      decoded, BARCODE_MAX_LEN);
        if (dec_len <= 0) continue;

        /* Reject non-printable results */
        int ok = 1;
        for (int i = 0; i < dec_len; i++) {
            unsigned char ch = (unsigned char)decoded[i];
            if (ch < 32 || ch > 126) { ok = 0; break; }
        }
        if (!ok) continue;

        /* Accept this result */
        memcpy(result->data, decoded, dec_len);
        result->data[dec_len] = '\0';
        result->length = dec_len;
        result->symbology = BARCODE_SYM_QR;
        result->confidence = 80;
        return 1;
    }

    return 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Main QR decode pipeline
 * ════════════════════════════════════════════════════════════════════════ */

static int qrcode_decode_from_binary(const Image *img, const uint8_t *bin, BarcodeResult *result)
{
    /* Step 1: Find finder patterns */
    FinderCenter finders[MAX_FINDERS];
    int n_finders = find_finder_patterns(bin, img->width, img->height,
                                         finders, MAX_FINDERS);

    if (n_finders < 3) {
        QR_DBG("[QRDEBUG] Found only %d finders (need 3+)\n", n_finders);
        return 0;
    }
    QR_DBG("[QRDEBUG] Found %d finder patterns\n", n_finders);
    for (int finder_index = 0; finder_index < n_finders; finder_index++) {
        QR_DBG("[QRDEBUG] finder[%d]=%.1f,%.1f ms=%.2f\n", finder_index,
               finders[finder_index].cx, finders[finder_index].cy,
               finders[finder_index].module_size);
    }
    g_qr_debug_stats.finder_stage_hits++;

    if (n_finders > 3) {
        int best_i = -1;
        int best_j = -1;
        int best_k = -1;
        double best_score = 0.0;
        for (int i = 0; i < n_finders - 2; i++) {
            for (int j = i + 1; j < n_finders - 1; j++) {
                for (int k = j + 1; k < n_finders; k++) {
                    double min_module = finders[i].module_size;
                    double max_module = min_module;
                    double score;
                    if (finders[j].module_size < min_module)
                        min_module = finders[j].module_size;
                    if (finders[k].module_size < min_module)
                        min_module = finders[k].module_size;
                    if (finders[j].module_size > max_module)
                        max_module = finders[j].module_size;
                    if (finders[k].module_size > max_module)
                        max_module = finders[k].module_size;
                    if (min_module <= 0.0 || max_module > min_module * 1.8)
                        continue;
                    score = qr_finder_triangle_score(&finders[i],
                                                     &finders[j],
                                                     &finders[k]);
                    if (score > best_score) {
                        best_score = score;
                        best_i = i;
                        best_j = j;
                        best_k = k;
                    }
                }
            }
        }
        if (best_i >= 0) {
            QR_DBG("[QRDEBUG] selected=%d,%d,%d score=%.1f\n", best_i,
                   best_j, best_k, best_score);
            FinderCenter ordered[MAX_FINDERS];
            int next = 3;
            ordered[0] = finders[best_i];
            ordered[1] = finders[best_j];
            ordered[2] = finders[best_k];
            for (int i = 0; i < n_finders; i++) {
                if (i != best_i && i != best_j && i != best_k) {
                    ordered[next++] = finders[i];
                }
            }
            memcpy(finders, ordered, (size_t)n_finders * sizeof(finders[0]));
            if (qr_reflection_layout_hint(&finders[0], &finders[1],
                                          &finders[2])) {
                FinderCenter swap = finders[1];
                finders[1] = finders[2];
                finders[2] = swap;
            }
        }
    }

    /* Step 2: Try all triples of finders */
    int decoded = 0;
    for (int i = 0; i < n_finders - 2 && !decoded; i++) {
        for (int j = i + 1; j < n_finders - 1 && !decoded; j++) {
            for (int k = j + 1; k < n_finders && !decoded; k++) {
                int tl, tr, bl;
                order_finders(finders, i, j, k, &tl, &tr, &bl);

                double ms = (finders[tl].module_size +
                             finders[tr].module_size +
                             finders[bl].module_size) / 3.0;
                if (ms < 1.0) continue;

                /* Estimate version from finder geometry and align module count. */
                double d_top = sqrt(dist2(&finders[tl], &finders[tr]));
                double d_left = sqrt(dist2(&finders[tl], &finders[bl]));
                double perspective_skew = fabs(d_top - d_left) / (fmax(d_top, d_left) + 1e-6);
                QRGeometryEstimate geom = estimate_version_from_finder_geometry(d_top, d_left, ms);
                double align_x = 0.0;
                double align_y = 0.0;
                double align_score = 0.0;
                int alignment_hit = 0;
                QRPerspectiveCandidate candidates[4];
                int candidate_count;
                uint8_t *best_grid = NULL;
                QRSamplingVoteResult best_vote = {0};
                int have_best_vote = 0;
                int version = geom.version;
                int modules = geom.modules;

                if (version < 1 || version > 40) continue;

                Perspective t_probe = qr_build_perspective_from_finders(
                    modules, &finders[tl], &finders[tr], &finders[bl], 0, 0.0, 0.0);

                if (version >= 2 &&
                    qr_refine_alignment_anchor(bin, img->width, img->height, version,
                                               &t_probe, &align_x, &align_y, &align_score)) {
                    alignment_hit = 1;
                }

                candidate_count = qr_build_perspective_candidates(
                    &finders[tl], &finders[tr], &finders[bl],
                    modules, alignment_hit, align_x, align_y, align_score, ms,
                    candidates, (int)(sizeof(candidates) / sizeof(candidates[0])));
                if (candidate_count <= 0) {
                    continue;
                }
                g_qr_debug_stats.perspective_candidates += (uint32_t)candidate_count;

                for (int ci = 0; ci < candidate_count; ci++) {
                    for (int sample_pass = 0; sample_pass < 3; sample_pass++) {
                        int vote_cross = (sample_pass == 1);
                        double jitter = (sample_pass == 1) ? 0.30 : 0.0;
                        int use_grayscale = (sample_pass == 2);
                        int attempt_version = geom.version;
                        int attempt_modules = geom.modules;
                        int ec_level = -1;
                        int mask_pattern = -1;
                        int format_used_copy = -1;
                        int format_bit_errors = 15;
                        int version_used_copy = -1;
                        int version_bit_errors = 18;
                        int version_agree = 0;
                        int version_decode_ok = 0;
                        int quality_score;
                        int vote_score;
                        double br_bias_magnitude;
                        Perspective t_cur;
                        uint8_t *grid;
                        QRSampleMetrics sample_metrics;

                        if (vote_cross) {
                            g_qr_debug_stats.voted_sampling_attempts++;
                        }

                        t_cur = qr_build_perspective_from_finders_variant(
                            attempt_modules, &finders[tl], &finders[tr], &finders[bl],
                            candidates[ci].use_alignment,
                            candidates[ci].align_x, candidates[ci].align_y,
                            candidates[ci].br_bias_x, candidates[ci].br_bias_y);
                        grid = (uint8_t *)calloc((size_t)attempt_modules * attempt_modules, 1);
                        if (!grid) {
                            continue;
                        }
                        g_qr_debug_stats.grid_sampling_attempts++;
                        if (use_grayscale) {
                            sample_grid_grayscale(img->pixels, img->width, img->height,
                                                  &t_cur, attempt_modules, grid, &sample_metrics);
                        } else {
                            sample_grid(bin, img->width, img->height, &t_cur,
                                        attempt_modules, vote_cross, jitter, grid,
                                        &sample_metrics);
                        }

                        if (sample_metrics.total_modules > 0) {
                            int oob_pct = (sample_metrics.out_of_bounds_modules * 100) /
                                          sample_metrics.total_modules;
                            if (oob_pct >= g_qr_vote_tuning.oob_reject_percent) {
                                free(grid);
                                continue;
                            }
                        }

                        if (!decode_format_info_ex(grid, attempt_modules, &ec_level, &mask_pattern,
                                                   &format_used_copy, &format_bit_errors)) {
                            QR_DBG("[QRDEBUG] Format decode failed for version %d\n", attempt_version);
                            free(grid);
                            continue;
                        }
                        QR_DBG("[QRDEBUG] Format decoded: ec_level=%d, mask=%d, errors=%d\n",
                                ec_level, mask_pattern, format_bit_errors);
                        g_qr_debug_stats.format_decode_hits++;

                        if (attempt_version >= 7) {
                            int vi = decode_version_info_ex(grid, attempt_modules,
                                                            &version_used_copy, &version_bit_errors,
                                                            &version_agree);
                            if (vi > 0 && vi != attempt_version) {
                                free(grid);
                                attempt_version = vi;
                                attempt_modules = attempt_version * 4 + 17;
                                t_cur = qr_build_perspective_from_finders_variant(
                                    attempt_modules, &finders[tl], &finders[tr], &finders[bl],
                                    candidates[ci].use_alignment,
                                    candidates[ci].align_x, candidates[ci].align_y,
                                    candidates[ci].br_bias_x, candidates[ci].br_bias_y);
                                grid = (uint8_t *)calloc((size_t)attempt_modules * attempt_modules, 1);
                                if (!grid) {
                                    continue;
                                }
                                g_qr_debug_stats.grid_sampling_attempts++;
                                if (use_grayscale) {
                                    sample_grid_grayscale(img->pixels, img->width, img->height,
                                                          &t_cur, attempt_modules, grid, &sample_metrics);
                                } else {
                                    sample_grid(bin, img->width, img->height, &t_cur,
                                                attempt_modules, vote_cross, jitter, grid,
                                                &sample_metrics);
                                }

                                if (sample_metrics.total_modules > 0) {
                                    int oob_pct = (sample_metrics.out_of_bounds_modules * 100) /
                                                  sample_metrics.total_modules;
                                    if (oob_pct >= g_qr_vote_tuning.oob_reject_percent) {
                                        free(grid);
                                        continue;
                                    }
                                }

                                if (!decode_format_info_ex(grid, attempt_modules, &ec_level, &mask_pattern,
                                                           &format_used_copy, &format_bit_errors)) {
                                    free(grid);
                                    continue;
                                }
                                g_qr_debug_stats.format_decode_hits++;

                                vi = decode_version_info_ex(grid, attempt_modules,
                                                            &version_used_copy, &version_bit_errors,
                                                            &version_agree);
                            }
                            if (vi > 0) {
                                version_decode_ok = 1;
                            }
                        }

                        br_bias_magnitude =
                            (fabs(candidates[ci].br_bias_x) + fabs(candidates[ci].br_bias_y)) /
                            (ms + 1e-6);

                        quality_score = qr_score_sampling_quality(
                            grid, attempt_modules,
                            format_used_copy, format_bit_errors,
                            attempt_version, version_used_copy,
                            version_bit_errors, version_agree,
                            candidates[ci].use_alignment, candidates[ci].align_score,
                            sample_metrics.out_of_bounds_modules,
                            sample_metrics.weak_vote_modules,
                            sample_metrics.total_modules,
                            perspective_skew,
                            br_bias_magnitude);

                        vote_score = qr_compose_vote_score(
                            format_used_copy, format_bit_errors,
                            attempt_version, version_decode_ok,
                            version_agree, version_bit_errors,
                            candidates[ci].use_alignment, candidates[ci].align_score,
                            sample_metrics.out_of_bounds_modules,
                            sample_metrics.weak_vote_modules,
                            sample_metrics.total_modules,
                            perspective_skew,
                            br_bias_magnitude);

                        /* Try EC decode eagerly on each grid with reasonable format info.
                         * Only on sample_pass==0 to limit cost for non-QR images.
                         * Try decoded mask/EC first, then fall back to all combos. */
                        if (format_bit_errors <= 3 && sample_pass == 0) {
                            uint8_t *func_imm = (uint8_t *)calloc((size_t)attempt_modules * attempt_modules, 1);
                            if (func_imm) {
                                mark_function_patterns(func_imm, attempt_modules, attempt_version);
                                uint8_t *data_imm = (uint8_t *)calloc(QR_MAX_CODEWORDS, 1);
                                uint8_t *tcw = (uint8_t *)calloc(QR_MAX_CODEWORDS, 1);
                                int data_len_imm = 0;
                                int ec_ok_imm = 0;

                                if (data_imm && tcw) {
                                    /* Fast path: try decoded mask/EC first */
                                    if (ec_level >= 0 && mask_pattern >= 0) {
                                    int tncw = read_codewords(grid, func_imm, attempt_modules, mask_pattern,
                                                               tcw, QR_MAX_CODEWORDS);
                                    data_len_imm = 0;
                                    if (deinterleave_and_correct(tcw, tncw, attempt_version, ec_level,
                                                                  data_imm, &data_len_imm)) {
                                        ec_ok_imm = 1;
                                    }
                                    }

                                    /* Full search: try all mask/EC combinations */
                                    for (int tm = 0; tm < 8 && !ec_ok_imm; tm++) {
                                        int tncw = read_codewords(grid, func_imm, attempt_modules, tm,
                                                                   tcw, QR_MAX_CODEWORDS);
                                        for (int te = 0; te < 4 && !ec_ok_imm; te++) {
                                            if (tm == mask_pattern && te == ec_level)
                                                continue; /* already tried */
                                            data_len_imm = 0;
                                            if (deinterleave_and_correct(tcw, tncw, attempt_version, te,
                                                                          data_imm, &data_len_imm)) {
                                                ec_ok_imm = 1;
                                            }
                                        }
                                    }
                                }

                                if (ec_ok_imm) {
                                    /* Decode data stream */
                                    char text_imm[BARCODE_MAX_LEN];
                                    int text_len_imm = decode_data_stream(data_imm, data_len_imm, attempt_version,
                                                                           text_imm, BARCODE_MAX_LEN - 1);
                                    if (text_len_imm > 0) {
                                        text_imm[text_len_imm] = '\0';
                                        memcpy(result->data, text_imm, (size_t)text_len_imm + 1);
                                        result->length = text_len_imm;
                                        result->symbology = BARCODE_SYM_QR;
                                        result->confidence = 80;
                                        free(tcw);
                                        free(data_imm);
                                        free(func_imm);
                                        free(grid);
                                        decoded = 1;
                                        break; /* Exit sample_pass loop */
                                    }
                                }
                                free(tcw);
                                free(data_imm);
                                free(func_imm);
                            }
                        }

                        if (!have_best_vote ||
                            vote_score > best_vote.vote_score ||
                            (vote_score == best_vote.vote_score &&
                             quality_score > best_vote.quality_score)) {
                            if (best_grid) {
                                free(best_grid);
                            }
                            best_grid = grid;
                            best_vote.ok = 1;
                            best_vote.version = attempt_version;
                            best_vote.modules = attempt_modules;
                            best_vote.ec_level = ec_level;
                            best_vote.mask_pattern = mask_pattern;
                            best_vote.quality_score = quality_score;
                            best_vote.vote_score = vote_score;
                            best_vote.sample_pass = sample_pass;
                            best_vote.candidate_index = ci;
                            have_best_vote = 1;
                        } else {
                            free(grid);
                        }
                    }
                    if (decoded) break; /* Exit candidate loop */
                }

                if (decoded) break; /* skip best-grid EC attempt if already decoded */

                if (!have_best_vote || !best_grid) {
                    continue;
                }
                g_qr_debug_stats.perspective_vote_wins++;

                {
                    uint8_t *grid = best_grid;
                    int version = best_vote.version;
                    int modules = best_vote.modules;
                    int ec_level = best_vote.ec_level;
                    int mask_pattern = best_vote.mask_pattern;

                    /* Mark function patterns */
                    uint8_t *func = (uint8_t *)calloc((size_t)modules * modules, 1);
                    if (!func) {
                        free(grid);
                        continue;
                    }
                    mark_function_patterns(func, modules, version);

                    /* Read codewords */
                    uint8_t *codewords = (uint8_t *)calloc(QR_MAX_CODEWORDS, 1);
                    uint8_t *data = (uint8_t *)calloc(QR_MAX_CODEWORDS, 1);
                    uint8_t *try_cw = (uint8_t *)calloc(QR_MAX_CODEWORDS, 1);
                    if (!codewords || !data || !try_cw) {
                        free(codewords);
                        free(data);
                        free(try_cw);
                        free(func);
                        free(grid);
                        continue;
                    }
                    int n_cw = read_codewords(grid, func, modules, mask_pattern,
                                              codewords, QR_MAX_CODEWORDS);

                    /* De-interleave + error correct.
                     * Try decoded format first; if it fails, try all EC level
                     * and mask combinations since format info may be misread. */
                    int data_len = 0;
                    int ec_ok = 0;
                    int final_ec = ec_level;
                    int final_mask = mask_pattern;

                    /* First try the decoded format */
                    QR_DBG("[QRDEBUG] Attempting EC decode: version=%d, ec=%d, mask=%d, cw=%d\n",
                            version, ec_level, mask_pattern, n_cw);
                    if (deinterleave_and_correct(codewords, n_cw, version, ec_level,
                                                  data, &data_len)) {
                        ec_ok = 1;
                    }

                    /* If primary format failed, try all mask/ec combinations */
                    if (!ec_ok) {
                        for (int try_mask = 0; try_mask < 8 && !ec_ok; try_mask++) {
                            int try_ncw = read_codewords(grid, func, modules, try_mask,
                                                          try_cw, QR_MAX_CODEWORDS);
                            for (int try_ec = 0; try_ec < 4 && !ec_ok; try_ec++) {
                                if (try_mask == mask_pattern && try_ec == ec_level)
                                    continue; /* already tried */
                                data_len = 0;
                                if (deinterleave_and_correct(try_cw, try_ncw, version, try_ec,
                                                              data, &data_len)) {
                                    final_ec = try_ec;
                                    final_mask = try_mask;
                                    ec_ok = 1;
                                    QR_DBG("[QRDEBUG] EC succeeded with fallback: ec=%d, mask=%d\n",
                                            try_ec, try_mask);
                                }
                            }
                        }
                    }

                    /* grid and func are no longer needed after codeword reading */
                    free(func);
                    free(grid);

                    if (!ec_ok) {
                        free(codewords);
                        free(data);
                        free(try_cw);
                        QR_DBG("[QRDEBUG] EC decode/correction failed (all combos)\n");
                        continue;
                    }
                    QR_DBG("[QRDEBUG] EC decode succeeded, data_len=%d\n", data_len);
                    g_qr_debug_stats.ec_decode_hits++;

                    /* Decode data stream */
                    char text[BARCODE_MAX_LEN];
                    int text_len = decode_data_stream(data, data_len, version,
                                                      text, BARCODE_MAX_LEN - 1);
                    free(codewords);
                    free(data);
                    free(try_cw);
                    if (text_len <= 0) continue;

                    text[text_len] = '\0';
                    memcpy(result->data, text, (size_t)text_len + 1);
                    result->length = text_len;
                    result->confidence = 80;
                    g_qr_debug_stats.payload_decode_hits++;
                    decoded = 1;
                }
            }
        }
    }

    return decoded;
}

int qrcode_decode(const Image *img, BarcodeResult *result)
{
    static const int k_dark_biases[] = {0, -8, -2, 4, -15, 10};
    if (!img || !img->pixels || img->width < 21 || img->height < 21 || !result) {
        return 0;
    }

    QR_DBG("[QRDEBUG] qrcode_decode called for %dx%d image\n", img->width, img->height);
    memset(result, 0, sizeof(*result));
    gf_init();

    if (qrcode_decode_inverse_panel(img, result)) {
        return 1;
    }

    /* High-contrast QR images normally need no adaptive threshold map. */
    if (qrcode_decode_global(img, result)) {
        return 1;
    }

    {
        size_t pixel_count = (size_t)img->width * (size_t)img->height;
        uint8_t *mirrored_pixels = (uint8_t *)calloc(pixel_count, 1);
        if (mirrored_pixels) {
            for (int y = 0; y < img->height; y++) {
                const uint8_t *source = img->pixels + (size_t)y * img->width;
                uint8_t *target = mirrored_pixels + (size_t)y * img->width;
                for (int left = 0, right = img->width - 1;
                     left < img->width; left++, right--) {
                    target[left] = source[right];
                }
            }
            Image mirrored = *img;
            mirrored.pixels = mirrored_pixels;
            int mirrored_decoded = qrcode_decode_global(&mirrored, result);
            if (mirrored_decoded) {
                free(mirrored_pixels);
                return 1;
            }
            free(mirrored_pixels);
        }
    }

    uint8_t *shared_mean_map = NULL;
    uint32_t *shared_integ = build_integral_image(img);
    if (shared_integ) {
        shared_mean_map = build_local_mean_map(img, shared_integ);
    }

    for (int i = 0; i < (int)(sizeof(k_dark_biases) / sizeof(k_dark_biases[0])); i++) {
        uint8_t *bin = binarize_for_decode(
            img, k_dark_biases[i], shared_mean_map, shared_integ);
        int decoded = 0;
        g_qr_debug_stats.binarize_attempts++;
        if (!bin) {
            continue;
        }
        QR_DBG("[QRDEBUG] Trying dark_bias=%d\n", k_dark_biases[i]);
        decoded = qrcode_decode_from_binary(img, bin, result);
        free(bin);
        if (decoded) {
            QR_DBG("[QRDEBUG] SUCCESS! Decoded: %s\n", result->data);
            free(shared_mean_map);
            free(shared_integ);
            return 1;
        }
    }

    /* Mirror retry: flip horizontally and try a few dark biases.
     * Only attempt if image is roughly square (QR codes are square). */
    {
        double aspect = (double)img->width / (double)img->height;
        if (aspect > 0.5 && aspect < 2.0) {
            static const int k_mirror_biases[] = {0, -8, -15};
            for (int i = 0; i < (int)(sizeof(k_mirror_biases) / sizeof(k_mirror_biases[0])); i++) {
                uint8_t *bin = binarize_for_decode(
                    img, k_mirror_biases[i], shared_mean_map, shared_integ);
                if (!bin) continue;
                flip_horizontal(bin, img->width, img->height);
                QR_DBG("[QRDEBUG] Mirror retry dark_bias=%d\n", k_mirror_biases[i]);
                int decoded = qrcode_decode_from_binary(img, bin, result);
                free(bin);
                if (decoded) {
                    QR_DBG("[QRDEBUG] SUCCESS (mirror)! Decoded: %s\n", result->data);
                    free(shared_mean_map);
                    free(shared_integ);
                    return 1;
                }
            }
        }
    }

    /* Standard QR failed. Try MicroQR (single finder pattern). */
    for (int i = 0; i < (int)(sizeof(k_dark_biases) / sizeof(k_dark_biases[0])); i++) {
        uint8_t *bin = binarize_for_decode(
            img, k_dark_biases[i], shared_mean_map, shared_integ);
        if (!bin) continue;
        if (try_microqr_decode(bin, img->width, img->height, result)) {
            free(bin);
            result->symbology = BARCODE_SYM_QR;
            free(shared_mean_map);
            free(shared_integ);
            return 1;
        }
        /* Also try inverted */
        for (int p = 0; p < img->width * img->height; p++) bin[p] ^= 1;
        if (try_microqr_decode(bin, img->width, img->height, result)) {
            free(bin);
            result->symbology = BARCODE_SYM_QR;
            free(shared_mean_map);
            free(shared_integ);
            return 1;
        }
        free(bin);
    }
    /* Also try mirror */
    {
        uint8_t *bin = binarize_for_decode(img, -8, shared_mean_map, shared_integ);
        if (bin) {
            flip_horizontal(bin, img->width, img->height);
            if (try_microqr_decode(bin, img->width, img->height, result)) {
                free(bin);
                result->symbology = BARCODE_SYM_QR;
                free(shared_mean_map);
                free(shared_integ);
                return 1;
            }
            free(bin);
        }
    }

    QR_DBG("[QRDEBUG] All attempts failed\n");
    free(shared_mean_map);
    free(shared_integ);
    return 0;
}

int qrcode_test_decode_format_words(uint16_t bits_primary, uint16_t bits_secondary,
                                    int *ec_level, int *mask_pattern,
                                    int *used_copy, int *bit_errors)
{
    return qr_choose_format_result(bits_primary, bits_secondary, ec_level, mask_pattern,
                                   used_copy, bit_errors);
}

int qrcode_test_decode_version_words(uint32_t bits_primary, uint32_t bits_secondary,
                                     int estimated_version, int *version_out,
                                     int *used_copy, int *bit_errors, int *agree)
{
    return qr_choose_version_result(bits_primary, bits_secondary, estimated_version, version_out,
                                    used_copy, bit_errors, agree);
}

int qrcode_test_estimate_version_geometry(double d_top, double d_left, double module_size)
{
    QRGeometryEstimate geom = estimate_version_from_finder_geometry(d_top, d_left, module_size);
    return geom.version;
}

int qrcode_test_refine_alignment_anchor(const uint8_t *bin, int w, int h, int version,
                                        double tl_x, double tl_y,
                                        double tr_x, double tr_y,
                                        double bl_x, double bl_y,
                                        double *align_x, double *align_y,
                                        double *match_score)
{
    int modules;
    FinderCenter tl, tr, bl;
    Perspective t;

    if (!bin || w <= 0 || h <= 0 || version < 2 || version > 40) {
        return 0;
    }
    modules = 4 * version + 17;
    tl.cx = tl_x; tl.cy = tl_y; tl.module_size = 0.0;
    tr.cx = tr_x; tr.cy = tr_y; tr.module_size = 0.0;
    bl.cx = bl_x; bl.cy = bl_y; bl.module_size = 0.0;

    t = qr_build_perspective_from_finders(modules, &tl, &tr, &bl, 0, 0.0, 0.0);
    return qr_refine_alignment_anchor(bin, w, h, version, &t, align_x, align_y, match_score);
}

int qrcode_test_compose_vote_score(int format_used_copy, int format_errors,
                                   int version, int version_decode_ok,
                                   int version_agree, int version_errors,
                                   int alignment_used, double align_score,
                                   int sample_out_of_bounds,
                                   int sample_weak_votes,
                                   int sample_total_modules,
                                   double perspective_skew,
                                   double br_bias_magnitude)
{
    return qr_compose_vote_score(format_used_copy, format_errors,
                                 version, version_decode_ok,
                                 version_agree, version_errors,
                                 alignment_used, align_score,
                                 sample_out_of_bounds, sample_weak_votes,
                                 sample_total_modules,
                                 perspective_skew, br_bias_magnitude);
}

int qrcode_test_score_sampling_quality(const uint8_t *grid, int modules,
                                       int format_used_copy, int format_errors,
                                       int version, int version_used_copy,
                                       int version_errors, int version_agree,
                                       int alignment_used, double align_score,
                                       int sample_out_of_bounds,
                                       int sample_weak_votes,
                                       int sample_total_modules,
                                       double perspective_skew,
                                       double br_bias_magnitude)
{
    return qr_score_sampling_quality(grid, modules,
                                     format_used_copy, format_errors,
                                     version, version_used_copy,
                                     version_errors, version_agree,
                                     alignment_used, align_score,
                                     sample_out_of_bounds, sample_weak_votes,
                                     sample_total_modules,
                                     perspective_skew, br_bias_magnitude);
}

void qrcode_test_get_vote_tuning(QRVoteTuning *out)
{
    if (!out) {
        return;
    }
    *out = g_qr_vote_tuning;
}

void qrcode_test_set_vote_tuning(const QRVoteTuning *in)
{
    if (!in) {
        return;
    }
    g_qr_vote_tuning = *in;
    if (g_qr_vote_tuning.align_score_weak < 0.0) {
        g_qr_vote_tuning.align_score_weak = 0.0;
    }
    if (g_qr_vote_tuning.align_score_strong < g_qr_vote_tuning.align_score_weak) {
        g_qr_vote_tuning.align_score_strong = g_qr_vote_tuning.align_score_weak;
    }
    if (g_qr_vote_tuning.oob_reject_percent < 1) {
        g_qr_vote_tuning.oob_reject_percent = 1;
    }
    if (g_qr_vote_tuning.oob_reject_percent > 100) {
        g_qr_vote_tuning.oob_reject_percent = 100;
    }
    if (g_qr_vote_tuning.oob_penalty_percent < 0) {
        g_qr_vote_tuning.oob_penalty_percent = 0;
    }
    if (g_qr_vote_tuning.oob_penalty_percent > 100) {
        g_qr_vote_tuning.oob_penalty_percent = 100;
    }
    if (g_qr_vote_tuning.weak_vote_penalty_percent < 0) {
        g_qr_vote_tuning.weak_vote_penalty_percent = 0;
    }
    if (g_qr_vote_tuning.weak_vote_penalty_percent > 100) {
        g_qr_vote_tuning.weak_vote_penalty_percent = 100;
    }
    if (g_qr_vote_tuning.skew_bias_penalty_th < 0.0) {
        g_qr_vote_tuning.skew_bias_penalty_th = 0.0;
    }
}

void qrcode_test_reset_vote_tuning(void)
{
    g_qr_vote_tuning = k_qr_vote_tuning_default;
}

void qrcode_debug_stats_reset(void)
{
    memset(&g_qr_debug_stats, 0, sizeof(g_qr_debug_stats));
}

void qrcode_debug_stats_get(QRDecodeDebugStats *out)
{
    if (!out) {
        return;
    }
    *out = g_qr_debug_stats;
}
