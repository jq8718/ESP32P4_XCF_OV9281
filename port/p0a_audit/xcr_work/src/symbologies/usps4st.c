/*
 * USPS Intelligent Mail Barcode (IMb / USPS4CB) decoder.
 *
 * Public standard USPS-B-3200 ("Intelligent Mail Barcode 4-State
 * Specification"). 65 equal-width, equal-pitch bars, each one of four
 * states relative to a shared baseline (tracker):
 *   Full(F)      = ascender + descender present  (11)
 *   Ascender(A)  = ascender only                 (10)
 *   Descender(D) = descender only                (01)
 *   Tracker(T)   = neither                        (00)
 *
 * Decode pipeline (inverse of the USPS encoder):
 *   1. scan for the tracker row -> locate 65 bar centers (geometry gate)
 *   2. classify each bar's vertical extension -> 65 x (asc,desc) bits
 *   3. bar-to-codeword de-shuffle via AppxD_IV -> 10 x 13-bit "characters"
 *   4. characters -> codewords via 5-of-13 / 2-of-13 tables (+ FCS bits)
 *   5. codewords -> 102-bit big integer -> 11-bit FCS (CRC-11) + payload
 *   6. FCS (CRC-11) must verify -- this is the anti-false-positive gate
 *   7. payload -> 20-digit tracking code + 0/5/9/11-digit routing code
 *
 * Table data (AppxD_IV, the 5-of-13/2-of-13 generation rule and the
 * CRC-11 routine) are reproductions of the public USPS specification, as
 * also published in the (public, non-decompiled) zint barcode library's
 * imail.c. This file was independently written against that spec.
 */

#include "usps4st.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define USPS4ST_BARS      65
#define USPS4ST_DARK_THR  128

/* ---- 1. bar-to-codeword shuffle table (public spec, Appendix D-IV) --- */
/* Character i (0..9), bit j (0..12) lives at bar_map[AppxD_IV[13*i+j]-1].
 * bar_map[0..64]  = descender bit of bar k
 * bar_map[65..129]= ascender  bit of bar k
 */
static const unsigned char AppxD_IV[130] = {
 67,6,78,16,86,95,34,40,45,113,117,121,62,87,18,104,41,76,57,119,
 115,72,97,2,127,26,105,35,122,52,114,7,24,82,68,63,94,44,77,112,
 70,100,39,30,107,15,125,85,10,65,54,88,20,106,46,66,8,116,29,61,
 99,80,90,37,123,51,25,84,129,56,4,109,96,28,36,47,11,71,33,102,
 21,9,17,49,124,79,64,91,42,69,53,60,14,1,27,103,126,75,89,50,
 120,19,32,110,92,111,130,59,31,12,81,43,55,5,74,22,101,128,58,118,
 48,108,38,98,93,23,83,13,73,3
};

/* ---- 2. 5-of-13 / 2-of-13 codeword tables (generated, self-checked) -- */
static uint16_t g_T5[1287];
static uint16_t g_T2[78];
static int16_t  g_REV[8192];
static int      g_tables_init;
static int      g_tables_ok;

static int popcount13(int x)
{
    return __builtin_popcount((unsigned)x & 0x1FFFu);
}

static int rev13(int x)
{
    int r = 0;
    int k;
    for (k = 0; k < 13; k++) {
        if (x & (1 << k)) r |= 1 << (12 - k);
    }
    return r;
}

static void init_nof13(uint16_t *tab, int n, int len)
{
    int lo = 0;
    int hi = len - 1;
    int c;
    for (c = 0; c < 8192; c++) {
        int r;
        if (popcount13(c) != n) continue;
        r = rev13(c);
        if (r < c) continue;
        if (c == r) {
            tab[hi--] = (uint16_t)c;
        } else {
            tab[lo++] = (uint16_t)c;
            tab[lo++] = (uint16_t)r;
        }
    }
}

static void usps4st_init_tables(void)
{
    int i;
    if (g_tables_init) return;
    g_tables_init = 1;

    init_nof13(g_T5, 5, 1287);
    init_nof13(g_T2, 2, 78);

    for (i = 0; i < 8192; i++) g_REV[i] = -1;
    for (i = 0; i < 1287; i++) g_REV[g_T5[i]] = (int16_t)i;
    for (i = 0; i < 78; i++) g_REV[g_T2[i]] = (int16_t)(1287 + i);

    /* Self-check against known-good table values (guards against a typo
     * in the generation rule silently producing a wrong-but-plausible
     * table; if this ever fails, refuse to decode rather than risk
     * misclassifying an unrelated symbology). */
    g_tables_ok =
        g_T5[0] == 0x001F && g_T5[1] == 0x1F00 && g_T5[2] == 0x002F && g_T5[3] == 0x1E80 &&
        g_T5[1284] == 0x0358 && g_T5[1285] == 0x02E8 && g_T5[1286] == 0x01F0 &&
        g_T2[0] == 0x0003 && g_T2[1] == 0x1800 && g_T2[2] == 0x0005 && g_T2[3] == 0x1400 &&
        g_T2[74] == 0x0404 && g_T2[75] == 0x0208 && g_T2[76] == 0x0110 && g_T2[77] == 0x00A0;
}

/* ---- 3. CRC-11 FCS (USPS-B-3200, polynomial 0xF35, init 0x7FF) ------- */
static uint16_t crc11(const uint8_t byte_array[13])
{
    const uint16_t gen = 0xF35;
    uint16_t fcs = 0x7FF;
    uint16_t data;
    int bit;
    int i;

    data = (uint16_t)(byte_array[0] << 5);
    for (bit = 2; bit < 8; bit++) {
        if ((fcs ^ data) & 0x400) fcs = (uint16_t)(((fcs << 1) ^ gen) & 0x7FF);
        else fcs = (uint16_t)((fcs << 1) & 0x7FF);
        data = (uint16_t)((data << 1) & 0xFFFF);
    }
    for (i = 1; i < 13; i++) {
        data = (uint16_t)(byte_array[i] << 3);
        for (bit = 0; bit < 8; bit++) {
            if ((fcs ^ data) & 0x400) fcs = (uint16_t)(((fcs << 1) ^ gen) & 0x7FF);
            else fcs = (uint16_t)((fcs << 1) & 0x7FF);
            data = (uint16_t)((data << 1) & 0xFFFF);
        }
    }
    return fcs;
}

static void to_bytes13(unsigned __int128 v, uint8_t out[13])
{
    int i;
    for (i = 0; i < 13; i++) {
        int shift = 8 * (12 - i);
        out[i] = (uint8_t)((v >> shift) & 0xFF);
    }
}

/* ---- 4. scan primitive: locate 65 bar centers on the tracker row ----- */
typedef struct { int a, b; } Run;

static int cmp_int(const void *pa, const void *pb)
{
    int a = *(const int *)pa;
    int b = *(const int *)pb;
    return (a > b) - (a < b);
}

static int cmp_double(const void *pa, const void *pb)
{
    double a = *(const double *)pa;
    double b = *(const double *)pb;
    return (a > b) - (a < b);
}

static int dark_runs_row(const Image *img, int y, Run *runs, int max_runs)
{
    int w = img->width;
    const uint8_t *row = img->pixels + (size_t)y * (size_t)w;
    int x = 0;
    int n = 0;
    while (x < w) {
        if (row[x] < USPS4ST_DARK_THR) {
            int xs = x;
            while (x < w && row[x] < USPS4ST_DARK_THR) x++;
            if (n >= max_runs) return -1;
            runs[n].a = xs;
            runs[n].b = x - 1;
            n++;
        } else {
            x++;
        }
    }
    return n;
}

/* Find a row containing exactly 65 equal-width, equal-pitch dark runs
 * (tolerating outlier-width runs from e.g. multipass full-image invert
 * leaving large dark borders). Returns 1 and fills *out_y / centers[]
 * on success. */
static int find_tracker_row(const Image *img, int *out_y, double centers[USPS4ST_BARS])
{
    int w = img->width;
    int h = img->height;
    int max_runs = w / 2 + 8;
    Run *runs = (Run *)malloc(sizeof(Run) * (size_t)max_runs);
    int *widths = (int *)malloc(sizeof(int) * (size_t)max_runs);
    int *wsort = (int *)malloc(sizeof(int) * (size_t)max_runs);
    double *cs = (double *)malloc(sizeof(double) * (size_t)max_runs);
    double *gsort = (double *)malloc(sizeof(double) * (size_t)max_runs);
    int found = 0;
    int y;

    if (!runs || !widths || !wsort || !cs || !gsort) {
        free(runs); free(widths); free(wsort); free(cs); free(gsort);
        return 0;
    }

    for (y = 0; y < h && !found; y += 3) {
        int n = dark_runs_row(img, y, runs, max_runs);
        int i;
        int mw;
        int ncand;
        double lo_w;
        double hi_w;
        int ngaps;
        double mg;
        int besti, bestj, i2;

        if (n < USPS4ST_BARS) continue;

        for (i = 0; i < n; i++) widths[i] = runs[i].b - runs[i].a + 1;
        memcpy(wsort, widths, sizeof(int) * (size_t)n);
        qsort(wsort, (size_t)n, sizeof(int), cmp_int);
        mw = wsort[n / 2];
        if (mw <= 0) continue;

        lo_w = mw * 0.4; if (lo_w < 1.0) lo_w = 1.0;
        hi_w = mw * 2.5;
        ncand = 0;
        for (i = 0; i < n; i++) {
            if ((double)widths[i] >= lo_w && (double)widths[i] <= hi_w) {
                cs[ncand++] = (runs[i].a + runs[i].b) / 2.0;
            }
        }
        if (ncand < USPS4ST_BARS) continue;

        ngaps = ncand - 1;
        if (ngaps <= 0) continue;
        for (i = 0; i < ngaps; i++) gsort[i] = cs[i + 1] - cs[i];
        qsort(gsort, (size_t)ngaps, sizeof(double), cmp_double);
        mg = gsort[ngaps / 2];
        if (mg <= mw) continue;

        besti = 0; bestj = -1;
        i2 = 0;
        while (i2 < ncand) {
            int j2 = i2;
            while (j2 + 1 < ncand && fabs(cs[j2 + 1] - cs[j2] - mg) <= mg * 0.35) j2++;
            if (j2 - i2 > bestj - besti) { besti = i2; bestj = j2; }
            i2 = j2 + 1;
        }
        if (bestj - besti + 1 == USPS4ST_BARS) {
            for (i = 0; i < USPS4ST_BARS; i++) centers[i] = cs[besti + i];
            *out_y = y;
            found = 1;
        }
    }

    free(runs); free(widths); free(wsort); free(cs); free(gsort);
    return found;
}

static int pixel_dark(const Image *img, int x, int y)
{
    return img->pixels[(size_t)y * (size_t)img->width + (size_t)x] < USPS4ST_DARK_THR;
}

static int all_centers_dark(const Image *img, const int xs[USPS4ST_BARS], int y)
{
    int i;
    int cnt = 0;
    if (y < 0 || y >= img->height) return 0;
    for (i = 0; i < USPS4ST_BARS; i++) {
        if (pixel_dark(img, xs[i], y)) cnt++;
    }
    return cnt >= (USPS4ST_BARS - 1);
}

/* ---- 5. bar-height classification -> ascender/descender bit vectors -- */
static int extract_states(const Image *img, int asc[USPS4ST_BARS], int desc[USPS4ST_BARS])
{
    double centers[USPS4ST_BARS];
    int xs[USPS4ST_BARS];
    int ymid;
    int yT0, yT1, Ht, ext;
    int i;

    if (!find_tracker_row(img, &ymid, centers)) return 0;
    for (i = 0; i < USPS4ST_BARS; i++) {
        int xc = (int)(centers[i] + 0.5);
        if (xc < 0) xc = 0;
        if (xc >= img->width) xc = img->width - 1;
        xs[i] = xc;
    }

    yT0 = ymid;
    yT1 = ymid;
    while (yT0 - 1 >= 0 && all_centers_dark(img, xs, yT0 - 1)) yT0--;
    while (yT1 + 1 < img->height && all_centers_dark(img, xs, yT1 + 1)) yT1++;

    Ht = yT1 - yT0 + 1;
    ext = (int)(0.45 * Ht);
    if (ext < 3) ext = 3;

    for (i = 0; i < USPS4ST_BARS; i++) {
        int xc = xs[i];
        int yt = yT0;
        int yb = yT1;
        while (yt - 1 >= 0 && pixel_dark(img, xc, yt - 1)) yt--;
        while (yb + 1 < img->height && pixel_dark(img, xc, yb + 1)) yb++;
        asc[i]  = ((yT0 - yt) >= ext) ? 1 : 0;
        desc[i] = ((yb - yT1) >= ext) ? 1 : 0;
    }
    return 1;
}

/* ---- 6/7/8: de-shuffle -> codewords -> FCS -> payload digits --------- */
static int try_decode_bits(const int asc[USPS4ST_BARS], const int desc[USPS4ST_BARS],
                            char *out, int *outlen)
{
    unsigned char bar_map[130];
    int chars[10];
    int cw[10];
    int fcs = 0;
    int i, j;
    unsigned __int128 v;
    uint8_t bytes13[13];
    uint8_t digs[20];
    unsigned __int128 routing;
    int pos;

    if (!g_tables_ok) return 0;

    for (i = 0; i < USPS4ST_BARS; i++) {
        bar_map[i]      = (unsigned char)desc[i];
        bar_map[i + 65] = (unsigned char)asc[i];
    }

    for (i = 0; i < 10; i++) {
        chars[i] = 0;
        for (j = 0; j < 13; j++) {
            chars[i] |= (int)bar_map[AppxD_IV[13 * i + j] - 1] << j;
        }
    }

    for (i = 0; i < 10; i++) {
        int pc = popcount13(chars[i]);
        int c;
        int r;
        if (pc == 5 || pc == 2) {
            c = chars[i];
        } else if (pc == 8 || pc == 11) {
            fcs |= 1 << i;
            c = 0x1FFF - chars[i];
        } else {
            return 0;
        }
        r = g_REV[c];
        if (r < 0) return 0;
        cw[i] = r;
    }

    if (cw[0] >= 659) {
        fcs |= 1 << 10;
        cw[0] -= 659;
    }
    if (cw[0] > 658) return 0;
    if ((cw[9] & 1) || cw[9] > 1270) return 0;
    cw[9] /= 2;
    for (j = 1; j <= 8; j++) {
        if (cw[j] > 1364) return 0;
    }

    v = (unsigned __int128)cw[0];
    for (j = 1; j <= 8; j++) v = v * 1365 + (unsigned __int128)cw[j];
    v = v * 636 + (unsigned __int128)cw[9];
    if (v >= (((unsigned __int128)1) << 102)) return 0;

    to_bytes13(v, bytes13);
    if (crc11(bytes13) != (uint16_t)fcs) return 0;

    for (i = 19; i >= 2; i--) {
        digs[i] = (uint8_t)(v % 10);
        v /= 10;
    }
    digs[1] = (uint8_t)(v % 5);
    v /= 5;
    digs[0] = (uint8_t)(v % 10);
    v /= 10;
    routing = v;

    pos = 0;
    for (i = 0; i < 20; i++) out[pos++] = (char)('0' + digs[i]);

    if (routing == 0) {
        /* no routing code */
    } else if (routing <= 100000) {
        unsigned long long z = (unsigned long long)(routing - 1);
        pos += snprintf(out + pos, 6, "%05llu", z);
    } else if (routing <= 1000100000ULL) {
        unsigned long long z = (unsigned long long)(routing - 100001ULL);
        pos += snprintf(out + pos, 10, "%09llu", z);
    } else if (routing <= 101000100000ULL) {
        unsigned long long z = (unsigned long long)(routing - 1000100001ULL);
        pos += snprintf(out + pos, 12, "%011llu", z);
    } else {
        return 0;
    }

    out[pos] = '\0';
    *outlen = pos;
    return 1;
}

int usps4st_decode(const Image *img, BarcodeResult *result)
{
    int asc[USPS4ST_BARS];
    int desc[USPS4ST_BARS];
    int asc_r[USPS4ST_BARS];
    int desc_r[USPS4ST_BARS];
    char out[BARCODE_MAX_LEN];
    int outlen = 0;
    int i;

    if (!result) return 0;
    memset(result, 0, sizeof(*result));
    if (!img || !img->pixels || img->width < USPS4ST_BARS || img->height < 9) return 0;

    usps4st_init_tables();
    if (!g_tables_ok) return 0;

    if (!extract_states(img, asc, desc)) return 0;

    if (try_decode_bits(asc, desc, out, &outlen)) {
        result->symbology = BARCODE_SYM_USPS4ST;
        result->length = outlen;
        memcpy(result->data, out, (size_t)outlen + 1);
        result->confidence = 100;
        return 1;
    }

    /* 180-degree rotation fallback: reverse bar order, swap asc/desc. */
    for (i = 0; i < USPS4ST_BARS; i++) {
        asc_r[i]  = desc[USPS4ST_BARS - 1 - i];
        desc_r[i] = asc[USPS4ST_BARS - 1 - i];
    }
    if (try_decode_bits(asc_r, desc_r, out, &outlen)) {
        result->symbology = BARCODE_SYM_USPS4ST;
        result->length = outlen;
        memcpy(result->data, out, (size_t)outlen + 1);
        result->confidence = 100;
        return 1;
    }

    return 0;
}
