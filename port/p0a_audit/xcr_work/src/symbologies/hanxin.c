/*
 * HanXin (汉信码) decoder — GB/T 21049 restoration.
 *
 * Structure/tables ported from the public zint reference implementation
 * (backend/hanxin.c, hanxin.h — BSD-3-Clause, Robin Stuart / Zoe Stuart).
 * RS(GF256, primitive 0x163) and RS(GF16, primitive 0x13) decoders follow
 * the same Berlekamp-Massey/Forney structure already used elsewhere in this
 * codebase (see aztec.c rs_decode) but are kept self-contained here.
 */

#include "hanxin.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ════════════════════════════════════════════════════════════════════════
 *  Capacity tables (zint hanxin.h: Table B1 / Annex A / Table D1)
 * ════════════════════════════════════════════════════════════════════════ */

static const unsigned short hx_total_codewords[84] = {
      25,   37,   50,   54,   69,   84,  100,  117,  136,  155,
     161,  181,  203,  225,  249,  273,  299,  325,  353,  381,
     411,  422,  453,  485,  518,  552,  587,  623,  660,  698,
     737,  754,  794,  836,  878,  922,  966, 1011, 1058, 1105,
    1126, 1175, 1224, 1275, 1327, 1380, 1434, 1489, 1513, 1569,
    1628, 1686, 1745, 1805, 1867, 1929, 1992, 2021, 2086, 2151,
    2218, 2286, 2355, 2425, 2496, 2528, 2600, 2673, 2749, 2824,
    2900, 2977, 3056, 3135, 3171, 3252, 3334, 3416, 3500, 3585,
    3671, 3758, 3798, 3886
};

/* Value 'k' from Annex A */
static const unsigned char hx_module_k[84] = {
     0,  0,  0, 14, 16, 16, 17, 18, 19, 20,
    14, 15, 16, 16, 17, 17, 18, 19, 20, 20,
    21, 16, 17, 17, 18, 18, 19, 19, 20, 20,
    21, 17, 17, 18, 18, 19, 19, 19, 20, 20,
    17, 17, 18, 18, 18, 19, 19, 19, 17, 17,
    18, 18, 18, 18, 19, 19, 19, 17, 17, 18,
    18, 18, 18, 19, 19, 17, 17, 17, 18, 18,
    18, 18, 19, 19, 17, 17, 17, 18, 18, 18,
    18, 18, 17, 17
};

/* Value 'r' from Annex A */
static const unsigned char hx_module_r[84] = {
     0,  0,  0, 15, 15, 17, 18, 19, 20, 21,
    15, 15, 15, 17, 17, 19, 19, 19, 19, 21,
    21, 17, 16, 18, 17, 19, 18, 20, 19, 21,
    20, 17, 19, 17, 19, 17, 19, 21, 19, 21,
    18, 20, 17, 19, 21, 18, 20, 22, 17, 19,
    15, 17, 19, 21, 17, 19, 21, 18, 20, 15,
    17, 19, 21, 16, 18, 17, 19, 21, 15, 17,
    19, 21, 15, 17, 18, 20, 22, 15, 17, 19,
    21, 23, 17, 19
};

/* Value 'm' from Annex A */
static const unsigned char hx_module_m[84] = {
    0,  0,  0,  1,  1,  1,  1,  1,  1,  1,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  3,  3,
    3,  4,  4,  4,  4,  4,  4,  4,  4,  4,
    5,  5,  5,  5,  5,  5,  5,  5,  6,  6,
    6,  6,  6,  6,  6,  6,  6,  7,  7,  7,
    7,  7,  7,  7,  7,  8,  8,  8,  8,  8,
    8,  8,  8,  8,  9,  9,  9,  9,  9,  9,
    9,  9, 10, 10
};

/* Error correction block sizes, Table D1: per version(1..84) x ecc(1..4),
 * 3 groups of {#blocks, k(data cw), 2t(ecc cw)}; unused groups are all 0. */
static const unsigned char hx_table_d1[84 * 4 * 9] = {
     1,   21,   4,   0,    0,   0,   0,    0,   0,  /* version 1 */
     1,   17,   8,   0,    0,   0,   0,    0,   0,
     1,   13,  12,   0,    0,   0,   0,    0,   0,
     1,    9,  16,   0,    0,   0,   0,    0,   0,
     1,   31,   6,   0,    0,   0,   0,    0,   0,  /* version 2 */
     1,   25,  12,   0,    0,   0,   0,    0,   0,
     1,   19,  18,   0,    0,   0,   0,    0,   0,
     1,   15,  22,   0,    0,   0,   0,    0,   0,
     1,   42,   8,   0,    0,   0,   0,    0,   0,  /* version 3 */
     1,   34,  16,   0,    0,   0,   0,    0,   0,
     1,   26,  24,   0,    0,   0,   0,    0,   0,
     1,   20,  30,   0,    0,   0,   0,    0,   0,
     1,   46,   8,   0,    0,   0,   0,    0,   0,  /* version 4 */
     1,   38,  16,   0,    0,   0,   0,    0,   0,
     1,   30,  24,   0,    0,   0,   0,    0,   0,
     1,   22,  32,   0,    0,   0,   0,    0,   0,
     1,   57,  12,   0,    0,   0,   0,    0,   0,  /* version 5 */
     1,   49,  20,   0,    0,   0,   0,    0,   0,
     1,   37,  32,   0,    0,   0,   0,    0,   0,
     1,   14,  20,   1,   13,  22,   0,    0,   0,
     1,   70,  14,   0,    0,   0,   0,    0,   0,  /* version 6 */
     1,   58,  26,   0,    0,   0,   0,    0,   0,
     1,   24,  20,   1,   22,  18,   0,    0,   0,
     1,   16,  24,   1,   18,  26,   0,    0,   0,
     1,   84,  16,   0,    0,   0,   0,    0,   0,  /* version 7 */
     1,   70,  30,   0,    0,   0,   0,    0,   0,
     1,   26,  22,   1,   28,  24,   0,    0,   0,
     2,   14,  20,   1,   12,  20,   0,    0,   0,
     1,   99,  18,   0,    0,   0,   0,    0,   0,  /* version 8 */
     1,   40,  18,   1,   41,  18,   0,    0,   0,
     1,   31,  26,   1,   32,  28,   0,    0,   0,
     2,   16,  24,   1,   15,  22,   0,    0,   0,
     1,  114,  22,   0,    0,   0,   0,    0,   0,  /* version 9 */
     2,   48,  20,   0,    0,   0,   0,    0,   0,
     2,   24,  20,   1,   26,  22,   0,    0,   0,
     2,   18,  28,   1,   18,  26,   0,    0,   0,
     1,  131,  24,   0,    0,   0,   0,    0,   0,  /* version 10 */
     1,   52,  22,   1,   57,  24,   0,    0,   0,
     2,   27,  24,   1,   29,  24,   0,    0,   0,
     2,   21,  32,   1,   19,  30,   0,    0,   0,
     1,  135,  26,   0,    0,   0,   0,    0,   0,  /* version 11 */
     1,   56,  24,   1,   57,  24,   0,    0,   0,
     2,   28,  24,   1,   31,  26,   0,    0,   0,
     2,   22,  32,   1,   21,  32,   0,    0,   0,
     1,  153,  28,   0,    0,   0,   0,    0,   0,  /* version 12 */
     1,   62,  26,   1,   65,  28,   0,    0,   0,
     2,   32,  28,   1,   33,  28,   0,    0,   0,
     3,   17,  26,   1,   22,  30,   0,    0,   0,
     1,   86,  16,   1,   85,  16,   0,    0,   0,  /* version 13 */
     1,   71,  30,   1,   72,  30,   0,    0,   0,
     2,   37,  32,   1,   35,  30,   0,    0,   0,
     3,   20,  30,   1,   21,  32,   0,    0,   0,
     1,   94,  18,   1,   95,  18,   0,    0,   0,  /* version 14 */
     2,   51,  22,   1,   55,  24,   0,    0,   0,
     3,   30,  26,   1,   31,  26,   0,    0,   0,
     4,   18,  28,   1,   17,  24,   0,    0,   0,
     1,  104,  20,   1,  105,  20,   0,    0,   0,  /* version 15 */
     2,   57,  24,   1,   61,  26,   0,    0,   0,
     3,   33,  28,   1,   36,  30,   0,    0,   0,
     4,   20,  30,   1,   19,  30,   0,    0,   0,
     1,  115,  22,   1,  114,  22,   0,    0,   0,  /* version 16 */
     2,   65,  28,   1,   61,  26,   0,    0,   0,
     3,   38,  32,   1,   33,  30,   0,    0,   0,
     5,   19,  28,   1,   14,  24,   0,    0,   0,
     1,  126,  24,   1,  125,  24,   0,    0,   0,  /* version 17 */
     2,   70,  30,   1,   69,  30,   0,    0,   0,
     4,   33,  28,   1,   29,  26,   0,    0,   0,
     5,   20,  30,   1,   19,  30,   0,    0,   0,
     1,  136,  26,   1,  137,  26,   0,    0,   0,  /* version 18 */
     3,   56,  24,   1,   59,  26,   0,    0,   0,
     5,   35,  30,   0,    0,   0,   0,    0,   0,
     6,   18,  28,   1,   21,  28,   0,    0,   0,
     1,  148,  28,   1,  149,  28,   0,    0,   0,  /* version 19 */
     3,   61,  26,   1,   64,  28,   0,    0,   0,
     7,   24,  20,   1,   23,  22,   0,    0,   0,
     6,   20,  30,   1,   21,  32,   0,    0,   0,
     3,  107,  20,   0,    0,   0,   0,    0,   0,  /* version 20 */
     3,   65,  28,   1,   72,  30,   0,    0,   0,
     7,   26,  22,   1,   23,  22,   0,    0,   0,
     7,   19,  28,   1,   20,  32,   0,    0,   0,
     3,  115,  22,   0,    0,   0,   0,    0,   0,  /* version 21 */
     4,   56,  24,   1,   63,  28,   0,    0,   0,
     7,   28,  24,   1,   25,  22,   0,    0,   0,
     8,   18,  28,   1,   21,  22,   0,    0,   0,
     2,  116,  22,   1,  122,  24,   0,    0,   0,  /* version 22 */
     4,   56,  24,   1,   72,  30,   0,    0,   0,
     7,   28,  24,   1,   32,  26,   0,    0,   0,
     8,   18,  28,   1,   24,  30,   0,    0,   0,
     3,  127,  24,   0,    0,   0,   0,    0,   0,  /* version 23 */
     5,   51,  22,   1,   62,  26,   0,    0,   0,
     7,   30,  26,   1,   35,  26,   0,    0,   0,
     8,   20,  30,   1,   21,  32,   0,    0,   0,
     2,  135,  26,   1,  137,  26,   0,    0,   0,  /* version 24 */
     5,   56,  24,   1,   59,  26,   0,    0,   0,
     7,   33,  28,   1,   30,  28,   0,    0,   0,
    11,   16,  24,   1,   19,  26,   0,    0,   0,
     3,  105,  20,   1,  121,  22,   0,    0,   0,  /* version 25 */
     5,   61,  26,   1,   57,  26,   0,    0,   0,
     9,   28,  24,   1,   28,  22,   0,    0,   0,
    10,   19,  28,   1,   18,  30,   0,    0,   0,
     2,  157,  30,   1,  150,  28,   0,    0,   0,  /* version 26 */
     5,   65,  28,   1,   61,  26,   0,    0,   0,
     8,   33,  28,   1,   34,  30,   0,    0,   0,
    10,   19,  28,   2,   15,  26,   0,    0,   0,
     3,  126,  24,   1,  115,  22,   0,    0,   0,  /* version 27 */
     7,   51,  22,   1,   54,  22,   0,    0,   0,
     8,   35,  30,   1,   37,  30,   0,    0,   0,
    15,   15,  22,   1,   10,  22,   0,    0,   0,
     4,  105,  20,   1,  103,  20,   0,    0,   0,  /* version 28 */
     7,   56,  24,   1,   45,  18,   0,    0,   0,
    10,   31,  26,   1,   27,  26,   0,    0,   0,
    10,   17,  26,   3,   20,  28,   1,   21,  28,
     3,  139,  26,   1,  137,  28,   0,    0,   0,  /* version 29 */
     6,   66,  28,   1,   66,  30,   0,    0,   0,
     9,   36,  30,   1,   34,  32,   0,    0,   0,
    13,   19,  28,   1,   17,  32,   0,    0,   0,
     6,   84,  16,   1,   82,  16,   0,    0,   0,  /* version 30 */
     6,   70,  30,   1,   68,  30,   0,    0,   0,
     7,   35,  30,   3,   33,  28,   1,   32,  28,
    13,   20,  30,   1,   20,  28,   0,    0,   0,
     5,  105,  20,   1,   94,  18,   0,    0,   0,  /* version 31 */
     6,   74,  32,   1,   71,  30,   0,    0,   0,
    11,   33,  28,   1,   34,  32,   0,    0,   0,
    13,   19,  28,   3,   16,  26,   0,    0,   0,
     4,  127,  24,   1,  126,  24,   0,    0,   0,  /* version 32 */
     7,   66,  28,   1,   66,  30,   0,    0,   0,
    12,   30,  24,   1,   24,  28,   1,   24,  30,
    15,   19,  28,   1,   17,  32,   0,    0,   0,
     7,   84,  16,   1,   78,  16,   0,    0,   0,  /* version 33 */
     7,   70,  30,   1,   66,  28,   0,    0,   0,
    12,   33,  28,   1,   32,  30,   0,    0,   0,
    14,   21,  32,   1,   24,  28,   0,    0,   0,
     5,  117,  22,   1,  117,  24,   0,    0,   0,  /* version 34 */
     8,   66,  28,   1,   58,  26,   0,    0,   0,
    11,   38,  32,   1,   34,  32,   0,    0,   0,
    15,   20,  30,   2,   17,  26,   0,    0,   0,
     4,  148,  28,   1,  146,  28,   0,    0,   0,  /* version 35 */
     8,   68,  30,   1,   70,  24,   0,    0,   0,
    10,   36,  32,   3,   38,  28,   0,    0,   0,
    16,   19,  28,   3,   16,  26,   0,    0,   0,
     4,  126,  24,   2,  135,  26,   0,    0,   0,  /* version 36 */
     8,   70,  28,   2,   43,  26,   0,    0,   0,
    13,   32,  28,   2,   41,  30,   0,    0,   0,
    17,   19,  28,   3,   15,  26,   0,    0,   0,
     5,  136,  26,   1,  132,  24,   0,    0,   0,  /* version 37 */
     5,   67,  30,   4,   68,  28,   1,   69,  28,
    14,   35,  30,   1,   32,  24,   0,    0,   0,
    18,   18,  26,   3,   16,  28,   1,   14,  28,
     3,  142,  26,   3,  141,  28,   0,    0,   0,  /* version 38 */
     8,   70,  30,   1,   73,  32,   1,   74,  32,
    12,   34,  30,   3,   34,  26,   1,   35,  28,
    18,   21,  32,   1,   27,  30,   0,    0,   0,
     5,  116,  22,   2,  103,  20,   1,  102,  20,  /* version 39 */
     9,   74,  32,   1,   74,  30,   0,    0,   0,
    14,   34,  28,   2,   32,  32,   1,   32,  30,
    19,   21,  32,   1,   25,  26,   0,    0,   0,
     7,  116,  22,   1,  117,  22,   0,    0,   0,  /* version 40 */
    11,   65,  28,   1,   58,  24,   0,    0,   0,
    15,   38,  32,   1,   27,  28,   0,    0,   0,
    20,   20,  30,   1,   20,  32,   1,   21,  32,
     6,  136,  26,   1,  130,  24,   0,    0,   0,  /* version 41 */
    11,   66,  28,   1,   62,  30,   0,    0,   0,
    14,   34,  28,   3,   34,  32,   1,   30,  30,
    18,   20,  30,   3,   20,  28,   2,   15,  26,
     5,  105,  20,   2,  115,  22,   2,  116,  22,  /* version 42 */
    10,   75,  32,   1,   73,  32,   0,    0,   0,
    16,   38,  32,   1,   27,  28,   0,    0,   0,
    22,   19,  28,   2,   16,  30,   1,   19,  30,
     6,  147,  28,   1,  146,  28,   0,    0,   0,  /* version 43 */
    11,   66,  28,   2,   65,  30,   0,    0,   0,
    18,   33,  28,   2,   33,  30,   0,    0,   0,
    22,   21,  32,   1,   28,  30,   0,    0,   0,
     6,  116,  22,   3,  125,  24,   0,    0,   0,  /* version 44 */
    11,   75,  32,   1,   68,  30,   0,    0,   0,
    13,   35,  28,   6,   34,  32,   1,   30,  30,
    23,   21,  32,   1,   26,  30,   0,    0,   0,
     7,  105,  20,   4,   95,  18,   0,    0,   0,  /* version 45 */
    12,   67,  28,   1,   63,  30,   1,   62,  32,
    21,   31,  26,   2,   33,  32,   0,    0,   0,
    23,   21,  32,   2,   24,  30,   0,    0,   0,
    10,  116,  22,   0,    0,   0,   0,    0,   0,  /* version 46 */
    12,   74,  32,   1,   78,  30,   0,    0,   0,
    18,   37,  32,   1,   39,  30,   1,   41,  28,
    25,   21,  32,   1,   27,  28,   0,    0,   0,
     5,  126,  24,   4,  115,  22,   1,  114,  22,  /* version 47 */
    12,   67,  28,   2,   66,  32,   1,   68,  30,
    21,   35,  30,   1,   39,  30,   0,    0,   0,
    26,   21,  32,   1,   28,  28,   0,    0,   0,
     9,  126,  24,   1,  117,  22,   0,    0,   0,  /* version 48 */
    13,   75,  32,   1,   68,  30,   0,    0,   0,
    20,   35,  30,   3,   35,  28,   0,    0,   0,
    27,   21,  32,   1,   28,  30,   0,    0,   0,
     9,  126,  24,   1,  137,  26,   0,    0,   0,  /* version 49 */
    13,   71,  30,   2,   68,  32,   0,    0,   0,
    20,   37,  32,   1,   39,  28,   1,   38,  28,
    24,   20,  32,   5,   25,  28,   0,    0,   0,
     8,  147,  28,   1,  141,  28,   0,    0,   0,  /* version 50 */
    10,   73,  32,   4,   74,  30,   1,   73,  30,
    16,   36,  32,   6,   39,  30,   1,   37,  30,
    27,   21,  32,   3,   20,  26,   0,    0,   0,
     9,  137,  26,   1,  135,  26,   0,    0,   0,  /* version 51 */
    12,   70,  30,   4,   75,  32,   0,    0,   0,
    24,   35,  30,   1,   40,  28,   0,    0,   0,
    23,   20,  32,   8,   24,  30,   0,    0,   0,
    14,   95,  18,   1,   86,  18,   0,    0,   0,  /* version 52 */
    13,   73,  32,   3,   77,  30,   0,    0,   0,
    24,   35,  30,   2,   35,  28,   0,    0,   0,
    26,   21,  32,   5,   21,  30,   1,   23,  30,
     9,  147,  28,   1,  142,  28,   0,    0,   0,  /* version 53 */
    10,   73,  30,   6,   70,  32,   1,   71,  32,
    25,   35,  30,   2,   34,  26,   0,    0,   0,
    29,   21,  32,   4,   22,  30,   0,    0,   0,
    11,  126,  24,   1,  131,  24,   0,    0,   0,  /* version 54 */
    16,   74,  32,   1,   79,  30,   0,    0,   0,
    25,   38,  32,   1,   25,  30,   0,    0,   0,
    33,   21,  32,   1,   28,  28,   0,    0,   0,
    14,  105,  20,   1,   99,  18,   0,    0,   0,  /* version 55 */
    19,   65,  28,   1,   72,  28,   0,    0,   0,
    24,   37,  32,   2,   40,  30,   1,   41,  30,
    31,   21,  32,   4,   24,  32,   0,    0,   0,
    10,  147,  28,   1,  151,  28,   0,    0,   0,  /* version 56 */
    15,   71,  30,   3,   71,  32,   1,   73,  32,
    24,   37,  32,   3,   38,  30,   1,   39,  30,
    36,   19,  30,   3,   29,  26,   0,    0,   0,
    15,  105,  20,   1,   99,  18,   0,    0,   0,  /* version 57 */
    19,   70,  30,   1,   64,  28,   0,    0,   0,
    27,   38,  32,   2,   25,  26,   0,    0,   0,
    38,   20,  30,   2,   18,  28,   0,    0,   0,
    14,  105,  20,   1,  113,  22,   1,  114,  22,  /* version 58 */
    17,   67,  30,   3,   92,  32,   0,    0,   0,
    30,   35,  30,   1,   41,  30,   0,    0,   0,
    36,   21,  32,   1,   26,  30,   1,   27,  30,
    11,  146,  28,   1,  146,  26,   0,    0,   0,  /* version 59 */
    20,   70,  30,   1,   60,  26,   0,    0,   0,
    29,   38,  32,   1,   24,  32,   0,    0,   0,
    40,   20,  30,   2,   17,  26,   0,    0,   0,
     3,  137,  26,   1,  136,  26,  10,  126,  24,  /* version 60 */
    22,   65,  28,   1,   75,  30,   0,    0,   0,
    30,   37,  32,   1,   51,  30,   0,    0,   0,
    42,   20,  30,   1,   21,  30,   0,    0,   0,
    12,  126,  24,   2,  118,  22,   1,  116,  22,  /* version 61 */
    19,   74,  32,   1,   74,  30,   1,   72,  28,
    30,   38,  32,   2,   29,  30,   0,    0,   0,
    39,   20,  32,   2,   37,  26,   1,   38,  26,
    12,  126,  24,   3,  136,  26,   0,    0,   0,  /* version 62 */
    21,   70,  30,   2,   65,  28,   0,    0,   0,
    34,   35,  30,   1,   44,  32,   0,    0,   0,
    42,   20,  30,   2,   19,  28,   2,   18,  28,
    12,  126,  24,   3,  117,  22,   1,  116,  22,  /* version 63 */
    25,   61,  26,   2,   62,  28,   0,    0,   0,
    34,   35,  30,   1,   40,  32,   1,   41,  32,
    45,   20,  30,   1,   20,  32,   1,   21,  32,
    15,  105,  20,   2,  115,  22,   2,  116,  22,  /* version 64 */
    25,   65,  28,   1,   72,  28,   0,    0,   0,
    18,   35,  30,  17,   37,  32,   1,   50,  32,
    42,   20,  30,   6,   19,  28,   1,   15,  28,
    19,  105,  20,   1,  101,  20,   0,    0,   0,  /* version 65 */
    33,   51,  22,   1,   65,  22,   0,    0,   0,
    40,   33,  28,   1,   28,  28,   0,    0,   0,
    49,   20,  30,   1,   18,  28,   0,    0,   0,
    18,  105,  20,   2,  117,  22,   0,    0,   0,  /* version 66 */
    26,   65,  28,   1,   80,  30,   0,    0,   0,
    35,   35,  30,   3,   35,  28,   1,   36,  28,
    52,   18,  28,   2,   38,  30,   0,    0,   0,
    26,   84,  16,   0,    0,   0,   0,    0,   0,  /* version 67 */
    26,   70,  30,   0,    0,   0,   0,    0,   0,
    45,   31,  26,   1,    9,  26,   0,    0,   0,
    52,   20,  30,   0,    0,   0,   0,    0,   0,
    16,  126,  24,   1,  114,  22,   1,  115,  22,  /* version 68 */
    23,   70,  30,   3,   65,  28,   1,   66,  28,
    40,   35,  30,   1,   43,  30,   0,    0,   0,
    46,   20,  30,   7,   19,  28,   1,   16,  28,
    19,  116,  22,   1,  105,  22,   0,    0,   0,  /* version 69 */
    20,   70,  30,   7,   66,  28,   1,   63,  28,
    40,   35,  30,   1,   42,  32,   1,   43,  32,
    54,   20,  30,   1,   19,  30,   0,    0,   0,
    17,  126,  24,   2,  115,  22,   0,    0,   0,  /* version 70 */
    24,   70,  30,   4,   74,  32,   0,    0,   0,
    48,   31,  26,   2,   18,  26,   0,    0,   0,
    54,   19,  28,   6,   15,  26,   1,   14,  26,
    29,   84,  16,   0,    0,   0,   0,    0,   0,  /* version 71 */
    29,   70,  30,   0,    0,   0,   0,    0,   0,
     6,   34,  30,   3,   36,  30,  38,   33,  28,
    58,   20,  30,   0,    0,   0,   0,    0,   0,
    16,  147,  28,   1,  149,  28,   0,    0,   0,  /* version 72 */
    31,   66,  28,   1,   37,  26,   0,    0,   0,
    48,   33,  28,   1,   23,  26,   0,    0,   0,
    53,   20,  30,   6,   19,  28,   1,   17,  28,
    20,  115,  22,   2,  134,  24,   0,    0,   0,  /* version 73 */
    29,   66,  28,   2,   56,  26,   2,   57,  26,
    45,   36,  30,   2,   15,  28,   0,    0,   0,
    59,   20,  30,   2,   21,  32,   0,    0,   0,
    17,  147,  28,   1,  134,  26,   0,    0,   0,  /* version 74 */
    26,   70,  30,   5,   75,  32,   0,    0,   0,
    47,   35,  30,   1,   48,  32,   0,    0,   0,
    64,   18,  28,   2,   33,  30,   1,   35,  30,
    22,  115,  22,   1,  133,  24,   0,    0,   0,  /* version 75 */
    33,   65,  28,   1,   74,  28,   0,    0,   0,
    43,   36,  30,   5,   27,  28,   1,   30,  28,
    57,   20,  30,   5,   21,  32,   1,   24,  32,
    18,  136,  26,   2,  142,  26,   0,    0,   0,  /* version 76 */
    33,   66,  28,   2,   49,  26,   0,    0,   0,
    48,   35,  30,   2,   38,  28,   0,    0,   0,
    64,   20,  30,   1,   20,  32,   0,    0,   0,
    19,  126,  24,   2,  135,  26,   1,  136,  26,  /* version 77 */
    32,   66,  28,   2,   55,  26,   2,   56,  26,
    49,   36,  30,   2,   18,  32,   0,    0,   0,
    65,   18,  28,   5,   27,  30,   1,   29,  30,
    20,  137,  26,   1,  130,  26,   0,    0,   0,  /* version 78 */
    30,   75,  32,   2,   71,  32,   0,    0,   0,
    46,   35,  30,   6,   39,  32,   0,    0,   0,
     3,   12,  30,  70,   19,  28,   0,    0,   0,
    20,  147,  28,   0,    0,   0,   0,    0,   0,  /* version 79 */
    35,   70,  30,   0,    0,   0,   0,    0,   0,
    49,   35,  30,   5,   35,  28,   0,    0,   0,
    70,   20,  30,   0,    0,   0,   0,    0,   0,
    21,  136,  26,   1,  155,  28,   0,    0,   0,  /* version 80 */
    34,   70,  30,   1,   64,  28,   1,   65,  28,
    54,   35,  30,   1,   45,  30,   0,    0,   0,
    68,   20,  30,   3,   18,  28,   1,   19,  28,
    19,  126,  24,   5,  115,  22,   1,  114,  22,  /* version 81 */
    33,   70,  30,   3,   65,  28,   1,   64,  28,
    52,   35,  30,   3,   41,  32,   1,   40,  32,
    67,   20,  30,   5,   21,  32,   1,   24,  32,
     2,  150,  28,  21,  136,  26,   0,    0,   0,  /* version 82 */
    32,   70,  30,   6,   65,  28,   0,    0,   0,
    52,   38,  32,   2,   27,  32,   0,    0,   0,
    73,   20,  30,   2,   22,  32,   0,    0,   0,
    21,  126,  24,   4,  136,  26,   0,    0,   0,  /* version 83 */
    30,   74,  32,   6,   73,  30,   0,    0,   0,
    54,   35,  30,   4,   40,  32,   0,    0,   0,
    75,   20,  30,   1,   20,  28,   0,    0,   0,
    30,  105,  20,   1,  114,  22,   0,    0,   0,  /* version 84 */
     3,   45,  22,  55,   47,  20,   0,    0,   0,
     2,   26,  26,  62,   33,  28,   0,    0,   0,
    79,   18,  28,   4,   33,  30,   0,    0,   0
};

/* ════════════════════════════════════════════════════════════════════════
 *  GF(256)/0x163 (data RS) and GF(16)/0x13 (structural-info RS)
 * ════════════════════════════════════════════════════════════════════════ */

static int hx_gf256_exp[512];
static int hx_gf256_log[256];
static int hx_gf256_ready;

static void hx_gf256_init(void)
{
    if (hx_gf256_ready) return;
    int v = 1;
    for (int i = 0; i < 255; i++) {
        hx_gf256_exp[i] = v;
        hx_gf256_log[v] = i;
        v <<= 1;
        if (v & 0x100) v ^= 0x163;
    }
    for (int i = 255; i < 512; i++) hx_gf256_exp[i] = hx_gf256_exp[i - 255];
    hx_gf256_log[0] = -1;
    hx_gf256_ready = 1;
}

static int hx_gf16_exp[32];
static int hx_gf16_log[16];
static int hx_gf16_ready;

static void hx_gf16_init(void)
{
    if (hx_gf16_ready) return;
    int v = 1;
    for (int i = 0; i < 15; i++) {
        hx_gf16_exp[i] = v;
        hx_gf16_log[v] = i;
        v <<= 1;
        if (v & 0x10) v ^= 0x13;
    }
    for (int i = 15; i < 32; i++) hx_gf16_exp[i] = hx_gf16_exp[i - 15];
    hx_gf16_log[0] = -1;
    hx_gf16_ready = 1;
}

typedef struct {
    const int *exp_tbl;
    const int *log_tbl;
    int order; /* field_size - 1 */
} HxGField;

static int hx_gf_mul(const HxGField *gf, int a, int b)
{
    if (a == 0 || b == 0) return 0;
    return gf->exp_tbl[gf->log_tbl[a] + gf->log_tbl[b]];
}

static int hx_gf_inv(const HxGField *gf, int a)
{
    if (a == 0) return 0;
    return gf->exp_tbl[gf->order - gf->log_tbl[a]];
}

/* Same Berlekamp-Massey/Chien/Forney structure as aztec.c's rs_decode;
 * roots alpha^1..alpha^n_ec. Returns 0 on success (cw corrected in place),
 * -1 if the block cannot be reliably corrected (miscorrection guard). */
static int hx_rs_decode(const HxGField *gf, int *cw, int n, int n_ec)
{
    if (n_ec > 256 || n_ec <= 0) return -1;

    int syn[256];
    int has_error = 0;
    for (int i = 0; i < n_ec; i++) {
        int s = 0;
        int alpha_i = gf->exp_tbl[i + 1];
        int pw = 1;
        for (int j = n - 1; j >= 0; j--) {
            s ^= hx_gf_mul(gf, cw[j], pw);
            pw = hx_gf_mul(gf, pw, alpha_i);
        }
        syn[i] = s;
        if (s != 0) has_error = 1;
    }
    if (!has_error) return 0;

    int sigma[256], sp[256];
    memset(sigma, 0, sizeof(sigma));
    memset(sp, 0, sizeof(sp));
    sigma[0] = 1; sp[0] = 1;
    int L = 0, m = 1, b = 1;

    for (int r = 0; r < n_ec; r++) {
        int d = syn[r];
        for (int j = 1; j <= L; j++)
            d ^= hx_gf_mul(gf, sigma[j], syn[r - j]);
        if (d == 0) { m++; continue; }
        int tmp[256];
        memcpy(tmp, sigma, sizeof(tmp));
        int coeff = hx_gf_mul(gf, d, hx_gf_inv(gf, b));
        for (int j = m; j < n_ec + 1 && j < 256; j++)
            sigma[j] ^= hx_gf_mul(gf, coeff, sp[j - m]);
        if (2 * L <= r) {
            L = r + 1 - L;
            memcpy(sp, tmp, sizeof(sp));
            b = d; m = 1;
        } else { m++; }
    }

    if (2 * L > n_ec) return -1;

    int err_pos[256], ne = 0;
    for (int i = 0; i < n; i++) {
        int v = 0, pw = 1;
        int ai = gf->exp_tbl[gf->order - i];
        for (int j = 0; j <= L; j++) {
            v ^= hx_gf_mul(gf, sigma[j], pw);
            pw = hx_gf_mul(gf, pw, ai);
        }
        if (v == 0) err_pos[ne++] = n - 1 - i;
    }
    if (ne != L) return -1;

    int omega[256];
    memset(omega, 0, sizeof(omega));
    for (int i = 0; i < n_ec; i++) {
        int s = 0;
        for (int j = 0; j <= i && j <= L; j++)
            s ^= hx_gf_mul(gf, sigma[j], syn[i - j]);
        omega[i] = s;
    }

    int sd[256];
    memset(sd, 0, sizeof(sd));
    for (int i = 1; i <= L; i += 2) sd[i - 1] = sigma[i];

    for (int k = 0; k < ne; k++) {
        int pos = err_pos[k];
        int xi = gf->exp_tbl[gf->order - (n - 1 - pos)];
        int ov = 0, pw = 1;
        for (int j = 0; j < n_ec; j++) {
            ov ^= hx_gf_mul(gf, omega[j], pw);
            pw = hx_gf_mul(gf, pw, xi);
        }
        int sv = 0; pw = 1;
        for (int j = 0; j <= (L - 1) / 2; j++) {
            sv ^= hx_gf_mul(gf, sd[j], pw);
            pw = hx_gf_mul(gf, pw, xi);
        }
        if (sv == 0) return -1;
        cw[pos] ^= hx_gf_mul(gf, ov, hx_gf_inv(gf, sv));
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Function (non-data) module map
 * ════════════════════════════════════════════════════════════════════════ */

#define HX_MAX_N 189

static void hx_safe_mark(uint8_t *F, int N, int x, int y)
{
    if (x >= 0 && x < N && y >= 0 && y < N && F[y * N + x] == 0) F[y * N + x] = 1;
}

static void hx_plot_alignment(uint8_t *F, int N, int x, int y, int w, int h)
{
    hx_safe_mark(F, N, x, y);
    hx_safe_mark(F, N, x - 1, y + 1);
    for (int i = 1; i <= w; i++) {
        hx_safe_mark(F, N, x - i, y);
        hx_safe_mark(F, N, x - i - 1, y + 1);
    }
    for (int i = 1; i < h; i++) {
        hx_safe_mark(F, N, x, y + i);
        hx_safe_mark(F, N, x - 1, y + i + 1);
    }
}

static void hx_plot_assist(uint8_t *F, int N, int x, int y)
{
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            hx_safe_mark(F, N, x + dx, y + dy);
}

/* F must be zeroed (N*N bytes) by the caller. version is 1-based. */
static void hx_function_map(uint8_t *F, int N, int version)
{
    /* 4 corner finder patterns, 7x7 each. */
    static const int corners[4][2] = { {0, 0}, {0, -7}, {-7, 0}, {-7, -7} };
    for (int c = 0; c < 4; c++) {
        int r0 = corners[c][0] < 0 ? N + corners[c][0] : corners[c][0];
        int c0 = corners[c][1] < 0 ? N + corners[c][1] : corners[c][1];
        for (int r = 0; r < 7; r++)
            for (int cc = 0; cc < 7; cc++)
                F[(r0 + r) * N + (c0 + cc)] = 1;
    }

    /* Separator band (white, width 1) around each finder's inward edges. */
    for (int i = 0; i < 8; i++) {
        int pts[8][2] = {
            {7, i}, {i, 7}, {7, N - 1 - i}, {N - 1 - i, 7},
            {i, N - 8}, {N - 8, i}, {N - 8, N - 1 - i}, {N - 1 - i, N - 8}
        };
        for (int p = 0; p < 8; p++) F[pts[p][0] * N + pts[p][1]] = 1;
    }

    /* Structural information region (34 + 34 modules). */
    for (int i = 0; i <= 8; i++) {
        int pts[8][2] = {
            {8, i}, {N - 9, N - 1 - i}, {8 - i, 8}, {N - 9 + i, N - 9},
            {i, N - 9}, {N - 1 - i, 8}, {8, N - 9 + i}, {N - 9, 8 - i}
        };
        for (int p = 0; p < 8; p++) F[pts[p][0] * N + pts[p][1]] = 1;
    }

    if (version > 3) {
        int K = hx_module_k[version - 1];
        int R = hx_module_r[version - 1];
        int M = hx_module_m[version - 1];

        int y = 0, mod_y = 0;
        while (y < N) {
            int mh = (mod_y < M) ? K : R - 1;
            if ((mod_y & 1) == 0) {
                if ((M & 1) == 1) hx_plot_assist(F, N, 0, y);
            } else {
                if ((M & 1) == 0) hx_plot_assist(F, N, 0, y);
                hx_plot_assist(F, N, N - 1, y);
            }
            mod_y++; y += mh;
        }

        int x = N - 1, mod_x = 0;
        while (x >= 0) {
            int mw = (mod_x < M) ? K : R - 1;
            if ((mod_x & 1) == 0) {
                if ((M & 1) == 1) hx_plot_assist(F, N, x, N - 1);
            } else {
                if ((M & 1) == 0) hx_plot_assist(F, N, x, N - 1);
                hx_plot_assist(F, N, x, 0);
            }
            mod_x++; x -= mw;
        }

        int colsw = 1;
        y = 0; mod_y = 0;
        while (y < N) {
            int mh = (mod_y < M) ? K : R - 1;
            int rowsw;
            if (colsw == 1) { rowsw = 1; colsw = 0; }
            else { rowsw = 0; colsw = 1; }
            x = N - 1; mod_x = 0;
            while (x >= 0) {
                int mw = (mod_x < M) ? K : R - 1;
                if (rowsw == 1) {
                    if (!(y == 0 && x == N - 1)) hx_plot_alignment(F, N, x, y, mw, mh);
                    rowsw = 0;
                } else {
                    rowsw = 1;
                }
                mod_x++; x -= mw;
            }
            mod_y++; y += mh;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Data mask
 * ════════════════════════════════════════════════════════════════════════ */

static int hx_mask_bit(int mask_no, int row, int col)
{
    int i = row + 1, j = col + 1;
    switch (mask_no) {
    case 0: return 0;
    case 1: return (((i + j) & 1) == 0) ? 1 : 0;
    case 2: return ((((i + j) % 3) + (j % 3)) & 1) == 0 ? 1 : 0;
    default: return (((i % j) + (j % i) + (i % 3) + (j % 3)) & 1) == 0 ? 1 : 0;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Structural information (version / ecc / mask), RS(7,3) over GF(16)
 * ════════════════════════════════════════════════════════════════════════ */

static int hx_decode_si(const uint8_t *g, int N, int *out_version, int *out_ecc, int *out_mask)
{
    hx_gf16_init();
    HxGField gf16 = { hx_gf16_exp, hx_gf16_log, 15 };

    for (int pass = 0; pass < 2; pass++) {
        int cells[34][2];
        int idx = 0;
        if (pass == 0) {
            for (int i = 0; i <= 8; i++) { cells[idx][0] = 8; cells[idx][1] = i; idx++; }
            for (int i = 1; i <= 8; i++) { cells[idx][0] = 8 - i; cells[idx][1] = 8; idx++; }
            for (int i = 0; i <= 8; i++) { cells[idx][0] = i; cells[idx][1] = N - 9; idx++; }
            for (int i = 1; i <= 8; i++) { cells[idx][0] = 8; cells[idx][1] = N - 9 + i; idx++; }
        } else {
            for (int i = 0; i <= 8; i++) { cells[idx][0] = N - 9; cells[idx][1] = N - 1 - i; idx++; }
            for (int i = 1; i <= 8; i++) { cells[idx][0] = N - 9 + i; cells[idx][1] = N - 9; idx++; }
            for (int i = 0; i <= 8; i++) { cells[idx][0] = N - 1 - i; cells[idx][1] = 8; idx++; }
            for (int i = 1; i <= 8; i++) { cells[idx][0] = N - 9; cells[idx][1] = 8 - i; idx++; }
        }

        int nib[7];
        for (int k = 0; k < 7; k++) {
            int v = 0;
            for (int b = 0; b < 4; b++) {
                int r = cells[k * 4 + b][0];
                int c = cells[k * 4 + b][1];
                v = (v << 1) | (g[r * N + c] & 1);
            }
            nib[k] = v;
        }
        if (hx_rs_decode(&gf16, nib, 7, 4) != 0) continue;

        int val = (nib[0] << 4) | nib[1];
        int version = val - 20;
        int ecc = ((nib[2] >> 2) & 3) + 1;
        int mask = nib[2] & 3;
        if (version >= 1 && version <= 84) {
            *out_version = version;
            *out_ecc = ecc;
            *out_mask = mask;
            return 1;
        }
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Payload text parsing (numeric / text / byte modes)
 * ════════════════════════════════════════════════════════════════════════ */

static char hx_text_table2[62];
static int hx_text_table2_ready;

static void hx_text_table2_init(void)
{
    if (hx_text_table2_ready) return;
    int idx = 0;
    for (int c = 0; c < 28; c++) hx_text_table2[idx++] = (char)c;
    for (int c = 32; c < 48; c++) hx_text_table2[idx++] = (char)c;
    for (int c = 58; c < 65; c++) hx_text_table2[idx++] = (char)c;
    for (int c = 91; c < 97; c++) hx_text_table2[idx++] = (char)c;
    for (int c = 123; c < 128; c++) hx_text_table2[idx++] = (char)c;
    hx_text_table2_ready = 1;
}

static const char hx_text_table1[63] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

typedef struct {
    const uint8_t *data;
    int nbits;
    int pos;
} HxBitReader;

static int hx_br_read(HxBitReader *br, int n)
{
    if (br->pos + n > br->nbits) return -1;
    int v = 0;
    for (int k = 0; k < n; k++) {
        int bitidx = br->pos + k;
        int byte = br->data[bitidx / 8];
        int bit = (byte >> (7 - (bitidx % 8))) & 1;
        v = (v << 1) | bit;
    }
    br->pos += n;
    return v;
}

static int hx_parse_stream(const uint8_t *data, int ndata, char *out, int out_cap)
{
    hx_text_table2_init();
    HxBitReader br = { data, ndata * 8, 0 };
    int len = 0;

    while (br.pos + 4 <= br.nbits) {
        int mode = hx_br_read(&br, 4);
        if (mode == 0) break;

        if (mode == 1) {
            /* numeric */
            int vals[700];
            int nv = 0;
            int nlast = 0;
            for (;;) {
                int v = hx_br_read(&br, 10);
                if (v < 0) return 0;
                if (v >= 1021) { nlast = v - 1020; break; }
                if (nv >= (int)(sizeof(vals) / sizeof(vals[0]))) return 0;
                vals[nv++] = v;
            }
            if (nv == 0) return 0;
            for (int k = 0; k < nv; k++) {
                char s[4];
                s[0] = (char)('0' + (vals[k] / 100) % 10);
                s[1] = (char)('0' + (vals[k] / 10) % 10);
                s[2] = (char)('0' + vals[k] % 10);
                int start = (k == nv - 1) ? (3 - nlast) : 0;
                if (start < 0) start = 0;
                for (int p = start; p < 3; p++) {
                    if (len >= out_cap - 1) return 0;
                    out[len++] = s[p];
                }
            }
        } else if (mode == 2) {
            /* text */
            int sub = 1;
            for (;;) {
                int v = hx_br_read(&br, 6);
                if (v < 0) return 0;
                if (v == 63) break;
                if (v == 62) { sub = 3 - sub; continue; }
                if (v >= 62) return 0;
                char c = (sub == 1) ? hx_text_table1[v] : hx_text_table2[v];
                if (len >= out_cap - 1) return 0;
                out[len++] = c;
            }
        } else if (mode == 3) {
            /* byte */
            int cnt = hx_br_read(&br, 13);
            if (cnt < 0) return 0;
            if (br.pos + 8 * cnt > br.nbits) return 0;
            for (int k = 0; k < cnt; k++) {
                int b = hx_br_read(&br, 8);
                if (len >= out_cap - 1) return 0;
                out[len++] = (char)b;
            }
        } else {
            /* region1/2, GB18030 4-byte, ECI: not supported (reject) */
            return 0;
        }
    }

    if (len == 0) return 0;
    out[len] = '\0';
    return len;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Full matrix decode: N x N binary grid (1 = dark) -> text
 * ════════════════════════════════════════════════════════════════════════ */

static int hx_decode_matrix(const uint8_t *g, int N, char *out, int out_cap)
{
    if (N < 23 || N > HX_MAX_N || ((N - 23) & 1)) return 0;
    int version = (N - 21) / 2;
    if (version < 1 || version > 84) return 0;

    int si_version, ecc, mask;
    if (!hx_decode_si(g, N, &si_version, &ecc, &mask)) {
        if (getenv("HX_DEBUG")) fprintf(stderr, "[hx]   decode_matrix N=%d version=%d: SI decode failed\n", N, version);
        return 0;
    }
    if (getenv("HX_DEBUG")) fprintf(stderr, "[hx]   decode_matrix N=%d version=%d si_version=%d ecc=%d mask=%d\n", N, version, si_version, ecc, mask);
    if (si_version != version) return 0;
    if (ecc < 1 || ecc > 4) return 0;

    if (getenv("HX_DUMP_GRID")) {
        for (int r = 0; r < N; r++) {
            for (int c = 0; c < N; c++) fputc(g[r * N + c] ? '1' : '0', stderr);
            fputc('\n', stderr);
        }
        fputc('\n', stderr);
    }

    static uint8_t F[HX_MAX_N * HX_MAX_N];
    memset(F, 0, (size_t)N * (size_t)N);
    hx_function_map(F, N, version);

    int total = hx_total_codewords[version - 1];
    int need_bits = total * 8;

    static uint8_t bits[HX_MAX_N * HX_MAX_N];
    int nbits = 0;
    for (int r = 0; r < N && nbits < need_bits; r++) {
        for (int c = 0; c < N && nbits < need_bits; c++) {
            if (F[r * N + c] == 0) {
                int b = (g[r * N + c] & 1) ^ hx_mask_bit(mask, r, c);
                bits[nbits++] = (uint8_t)b;
            }
        }
    }
    if (nbits < need_bits) return 0;

    static uint8_t pf[4096];
    for (int k = 0; k < total; k++) {
        int v = 0;
        for (int b = 0; b < 8; b++) v = (v << 1) | bits[k * 8 + b];
        pf[k] = (uint8_t)v;
    }

    /* Inverse "picket fence" interleave, step 13 (ISO 20830 5.8.2). */
    static uint8_t full[4096];
    {
        int k = 0;
        for (int start = 0; start < 13; start++) {
            for (int i = start; i < total; i += 13) full[i] = pf[k++];
        }
    }

    hx_gf256_init();
    HxGField gf256 = { hx_gf256_exp, hx_gf256_log, 255 };

    const unsigned char *row = &hx_table_d1[((version - 1) * 4 + (ecc - 1)) * 9];
    static uint8_t data[4096];
    int ndata = 0;
    int posn = 0;
    for (int grp = 0; grp < 3; grp++) {
        int nb = row[grp * 3 + 0];
        int k = row[grp * 3 + 1];
        int twoT = row[grp * 3 + 2];
        if (nb <= 0) continue;
        for (int b = 0; b < nb; b++) {
            int blen = k + twoT;
            if (posn + blen > total) return 0;
            int blk[256];
            for (int i = 0; i < blen; i++) blk[i] = full[posn + i];
            posn += blen;
            if (hx_rs_decode(&gf256, blk, blen, twoT) != 0) {
                if (getenv("HX_DEBUG")) fprintf(stderr, "[hx]     block RS fail grp=%d b=%d blen=%d twoT=%d\n", grp, b, blen, twoT);
                return 0;
            }
            if (ndata + k > (int)sizeof(data)) return 0;
            for (int i = 0; i < k; i++) data[ndata++] = (uint8_t)blk[i];
        }
    }
    if (ndata == 0) return 0;

    if (getenv("HX_DEBUG")) fprintf(stderr, "[hx]     data RS ok ndata=%d\n", ndata);
    int r = hx_parse_stream(data, ndata, out, out_cap);
    if (getenv("HX_DEBUG")) fprintf(stderr, "[hx]     parse_stream -> %d\n", r);
    return r;
}

/* ════════════════════════════════════════════════════════════════════════
 *  Finder detection: locate the 4 corner position-detection patterns
 * ════════════════════════════════════════════════════════════════════════ */

static uint8_t *hx_binarize(const Image *img, int dark_bias)
{
    int w = img->width;
    int h = img->height;
    uint8_t *bin = (uint8_t *)calloc((size_t)w * h, 1);
    if (!bin) return NULL;

    int block = w / 8;
    if (block < 8) block = 8;
    if (block > 64) block = 64;
    int half = block / 2;

    uint32_t *integ = (uint32_t *)calloc((size_t)(w + 1) * (h + 1), sizeof(uint32_t));
    if (!integ) { free(bin); return NULL; }

    for (int y = 0; y < h; y++) {
        uint32_t row_sum = 0;
        for (int x = 0; x < w; x++) {
            row_sum += img->pixels[y * w + x];
            integ[(y + 1) * (w + 1) + (x + 1)] = row_sum + integ[y * (w + 1) + (x + 1)];
        }
    }

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

    free(integ);
    return bin;
}

typedef struct { double cx, cy, module_size; int count; } HxCand;

/* Check 1:1:1:1:3 (orientation 0) or 3:1:1:1:1 (orientation 1) ratio. */
static int hx_check_ratio(const int *c, double *unit_out, int *orientation)
{
    int total = c[0] + c[1] + c[2] + c[3] + c[4];
    if (total < 7) return 0;
    double unit = (double)total / 7.0;
    double tol = unit * 0.6 + 1.0;

    if (fabs(c[0] - unit) < tol && fabs(c[1] - unit) < tol && fabs(c[2] - unit) < tol &&
        fabs(c[3] - unit) < tol && fabs(c[4] - 3.0 * unit) < tol) {
        if (unit_out) *unit_out = unit;
        if (orientation) *orientation = 0;
        return 1;
    }
    if (fabs(c[0] - 3.0 * unit) < tol && fabs(c[1] - unit) < tol && fabs(c[2] - unit) < tol &&
        fabs(c[3] - unit) < tol && fabs(c[4] - unit) < tol) {
        if (unit_out) *unit_out = unit;
        if (orientation) *orientation = 1;
        return 1;
    }
    return 0;
}

static double hx_center_from_counts(int end, const int *c, int orientation)
{
    int total = c[0] + c[1] + c[2] + c[3] + c[4];
    if (orientation == 0) {
        return (double)end - (double)c[4] / 2.0;
    }
    double start = (double)(end - total);
    return start + (double)c[0] / 2.0;
}

#define HX_MAX_CAND 512

/* Merge (cx,cy,ms) into an existing nearby candidate using a proper
 * incremental mean (order-independent), or append a new one. Circular
 * tolerance -- used for one-shot merges of already-good estimates. */
static int hx_add_candidate(HxCand *out, int *n, int max_out, double cx, double cy, double ms)
{
    for (int i = 0; i < *n; i++) {
        double dx = out[i].cx - cx;
        double dy = out[i].cy - cy;
        double r = out[i].module_size + ms;
        if (dx * dx + dy * dy < r * r) {
            int cnt = out[i].count + 1;
            out[i].cx += (cx - out[i].cx) / cnt;
            out[i].cy += (cy - out[i].cy) / cnt;
            out[i].module_size += (ms - out[i].module_size) / cnt;
            out[i].count = cnt;
            return 1;
        }
    }
    if (*n < max_out) {
        out[*n].cx = cx; out[*n].cy = cy; out[*n].module_size = ms; out[*n].count = 1;
        (*n)++;
        return 1;
    }
    return 0;
}

/* Same as hx_add_candidate but with an anisotropic tolerance: tight across
 * the scan direction (the finder center's coordinate along the scan axis is
 * very stable from one scanline to the next) and loose along the scan
 * direction itself (a single corner's solid 3x3 block spans several
 * scanlines, e.g. up to ~3 modules for a row scan). This prevents a chain
 * of same-axis merges from drifting the averaged center toward unrelated
 * hits that happen to fall within a naive circular radius. */
static int hx_add_candidate_scan(HxCand *out, int *n, int max_out, double cx, double cy, double ms, int is_row_scan)
{
    double tight = ms * 0.6 + 1.5;
    double loose = ms * 4.0 + 4.0;
    double dx_tol = is_row_scan ? tight : loose;
    double dy_tol = is_row_scan ? loose : tight;

    for (int i = 0; i < *n; i++) {
        double dx = fabs(out[i].cx - cx);
        double dy = fabs(out[i].cy - cy);
        if (dx <= dx_tol && dy <= dy_tol) {
            int cnt = out[i].count + 1;
            out[i].cx += (cx - out[i].cx) / cnt;
            out[i].cy += (cy - out[i].cy) / cnt;
            out[i].module_size += (ms - out[i].module_size) / cnt;
            out[i].count = cnt;
            return 1;
        }
    }
    if (*n < max_out) {
        out[*n].cx = cx; out[*n].cy = cy; out[*n].module_size = ms; out[*n].count = 1;
        (*n)++;
        return 1;
    }
    return 0;
}

/* Scan every row for the 1:1:1:1:3 finder ratio (either orientation). */
static int hx_scan_rows(const uint8_t *bin, int w, int h, HxCand *out, int max_out)
{
    int n = 0;
    for (int y = 0; y < h; y++) {
        int counts[5] = {0};
        int state = 0;
        for (int x = 0; x < w; x++) {
            int pixel = bin[y * w + x];
            if (pixel) {
                if (state == 1 || state == 3) { state++; counts[state] = 1; }
                else counts[state]++;
            } else {
                if (state == 0 || state == 2) { state++; counts[state] = 1; }
                else if (state == 4) {
                    double unit; int orient;
                    if (hx_check_ratio(counts, &unit, &orient)) {
                        double cx = hx_center_from_counts(x, counts, orient);
                        if (getenv("HX_DEBUG3") && cx > 580 && cx < 605 && y > 240 && y < 320)
                            fprintf(stderr, "[hx] rawrow y=%d cx=%.1f unit=%.2f orient=%d counts=%d,%d,%d,%d,%d\n",
                                    y, cx, unit, orient, counts[0], counts[1], counts[2], counts[3], counts[4]);
                        hx_add_candidate_scan(out, &n, max_out, cx, (double)y, unit, 1);
                    }
                    counts[0] = counts[2]; counts[1] = counts[3]; counts[2] = counts[4];
                    counts[3] = 1; counts[4] = 0;
                    state = 3;
                } else {
                    counts[state]++;
                }
            }
        }
    }
    return n;
}

static int hx_scan_cols(const uint8_t *bin, int w, int h, HxCand *out, int max_out)
{
    int n = 0;
    for (int x = 0; x < w; x++) {
        int counts[5] = {0};
        int state = 0;
        for (int y = 0; y < h; y++) {
            int pixel = bin[y * w + x];
            if (pixel) {
                if (state == 1 || state == 3) { state++; counts[state] = 1; }
                else counts[state]++;
            } else {
                if (state == 0 || state == 2) { state++; counts[state] = 1; }
                else if (state == 4) {
                    double unit; int orient;
                    if (hx_check_ratio(counts, &unit, &orient)) {
                        double cy = hx_center_from_counts(y, counts, orient);
                        hx_add_candidate_scan(out, &n, max_out, (double)x, cy, unit, 0);
                    }
                    counts[0] = counts[2]; counts[1] = counts[3]; counts[2] = counts[4];
                    counts[3] = 1; counts[4] = 0;
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
 *  4-point perspective sampling (same approach as qrcode.c / datamatrix.c)
 * ════════════════════════════════════════════════════════════════════════ */

typedef struct { double a[9]; } HxPerspective;

static int hx_solve_linear_8x8(double m[8][9], double out[8])
{
    for (int col = 0; col < 8; col++) {
        int pivot = col;
        double max_abs = fabs(m[col][col]);
        for (int r = col + 1; r < 8; r++) {
            double v = fabs(m[r][col]);
            if (v > max_abs) { max_abs = v; pivot = r; }
        }
        if (max_abs < 1e-12) return 0;
        if (pivot != col) {
            for (int k = col; k <= 8; k++) {
                double tmp = m[col][k]; m[col][k] = m[pivot][k]; m[pivot][k] = tmp;
            }
        }
        double inv = 1.0 / m[col][col];
        for (int k = col; k <= 8; k++) m[col][k] *= inv;
        for (int r = 0; r < 8; r++) {
            if (r == col) continue;
            double factor = m[r][col];
            if (fabs(factor) < 1e-15) continue;
            for (int k = col; k <= 8; k++) m[r][k] -= factor * m[col][k];
        }
    }
    for (int i = 0; i < 8; i++) out[i] = m[i][8];
    return 1;
}

static HxPerspective hx_compute_perspective(
    double su0, double sv0, double du0, double dv0,
    double su1, double sv1, double du1, double dv1,
    double su2, double sv2, double du2, double dv2,
    double su3, double sv3, double du3, double dv3)
{
    HxPerspective t;
    double eq[8][9];
    double sol[8];
    const double su[4] = { su0, su1, su2, su3 };
    const double sv[4] = { sv0, sv1, sv2, sv3 };
    const double du[4] = { du0, du1, du2, du3 };
    const double dv[4] = { dv0, dv1, dv2, dv3 };

    memset(&t, 0, sizeof(t));
    memset(eq, 0, sizeof(eq));

    for (int i = 0; i < 4; i++) {
        int r = i * 2;
        eq[r][0] = su[i]; eq[r][1] = sv[i]; eq[r][2] = 1.0;
        eq[r][6] = -su[i] * du[i]; eq[r][7] = -sv[i] * du[i]; eq[r][8] = du[i];

        eq[r + 1][3] = su[i]; eq[r + 1][4] = sv[i]; eq[r + 1][5] = 1.0;
        eq[r + 1][6] = -su[i] * dv[i]; eq[r + 1][7] = -sv[i] * dv[i]; eq[r + 1][8] = dv[i];
    }

    if (hx_solve_linear_8x8(eq, sol)) {
        for (int i = 0; i < 8; i++) t.a[i] = sol[i];
        t.a[8] = 1.0;
        return t;
    }

    t.a[0] = du1 - du0; t.a[1] = du3 - du0; t.a[2] = du0;
    t.a[3] = dv1 - dv0; t.a[4] = dv3 - dv0; t.a[5] = dv0;
    t.a[6] = 0.0; t.a[7] = 0.0; t.a[8] = 1.0;
    return t;
}

static void hx_perspective_map(const HxPerspective *t, double u, double v, double *x, double *y)
{
    double w = t->a[6] * u + t->a[7] * v + t->a[8];
    if (fabs(w) < 1e-10) w = 1e-10;
    *x = (t->a[0] * u + t->a[1] * v + t->a[2]) / w;
    *y = (t->a[3] * u + t->a[4] * v + t->a[5]) / w;
}

/* Sample one module as a 3x3 majority vote around the mapped pixel (rather
 * than a single nearest pixel): a lone boundary pixel that binarized the
 * "wrong" way (anti-aliasing/blur near a module edge) is outvoted by its
 * neighbors. Returns 0/1, or -1 if the mapped point falls outside the
 * image. */
static int hx_sample_bit(const uint8_t *bin, int w, int h, const HxPerspective *t, double u, double v)
{
    double x, y;
    hx_perspective_map(t, u, v, &x, &y);
    int px = (int)(x + 0.5), py = (int)(y + 0.5);
    if (px < 0 || px >= w || py < 0 || py >= h) return -1;
    int ones = 0;
    for (int dy = -1; dy <= 1; dy++) {
        int sy = py + dy;
        if (sy < 0) sy = 0; else if (sy >= h) sy = h - 1;
        for (int dx = -1; dx <= 1; dx++) {
            int sx = px + dx;
            if (sx < 0) sx = 0; else if (sx >= w) sx = w - 1;
            ones += bin[sy * w + sx];
        }
    }
    return (ones >= 5) ? 1 : 0;
}

/* Known unrotated finder-pattern templates (1 = dark), row-major 7x7, from
 * GB/T 21049. PA sits at the TL corner; PB (unrotated) is reused at both TR
 * and BL; PC sits at BR. */
static const uint8_t HX_TPL_PA[49] = {
    1,1,1,1,1,1,1,
    1,0,0,0,0,0,0,
    1,0,1,1,1,1,1,
    1,0,1,0,0,0,0,
    1,0,1,0,1,1,1,
    1,0,1,0,1,1,1,
    1,0,1,0,1,1,1,
};
static const uint8_t HX_TPL_PB[49] = {
    1,1,1,1,1,1,1,
    0,0,0,0,0,0,1,
    1,1,1,1,1,0,1,
    0,0,0,0,1,0,1,
    1,1,1,0,1,0,1,
    1,1,1,0,1,0,1,
    1,1,1,0,1,0,1,
};
static const uint8_t HX_TPL_PC[49] = {
    1,1,1,0,1,0,1,
    1,1,1,0,1,0,1,
    1,1,1,0,1,0,1,
    0,0,0,0,1,0,1,
    1,1,1,1,1,0,1,
    0,0,0,0,0,0,1,
    1,1,1,1,1,1,1,
};

static int hx_template_score(const uint8_t *bin, int w, int h, const HxPerspective *t,
                             int r0, int c0, const uint8_t *tpl)
{
    int mism = 0;
    for (int r = 0; r < 7; r++) {
        for (int c = 0; c < 7; c++) {
            int b = hx_sample_bit(bin, w, h, t, (double)(c0 + c) + 0.5, (double)(r0 + r) + 0.5);
            if (b < 0) return 999;
            if (b != tpl[r * 7 + c]) mism++;
        }
    }
    return mism;
}

/* Sub-pixel refine the 4 finder-center anchors by a small coordinate-descent
 * search that maximizes agreement between the resampled 7x7 corner boxes and
 * their known unrotated templates (PA/PB/PB/PC). Assumes the coarse tl/tr/
 * bl/br -> corner assignment is the symbol's true (unrotated) orientation;
 * when that assumption is wrong (rotated/mirrored capture) the template
 * scores simply won't improve much and the anchors are left near their
 * original candidate positions -- harmless, since the existing 8-way
 * transform retry downstream still recovers those cases and every result is
 * still gated by the SI/data RS checks. Cheap: a handful of small local
 * searches, no full-grid resampling. */
static void hx_refine_corners(const uint8_t *bin, int w, int h, int N,
                              double *tlx, double *tly, double *trx, double *tr_y,
                              double *blx, double *bly, double *brx, double *bry)
{
    double *px[4] = { tlx, trx, blx, brx };
    double *py[4] = { tly, tr_y, bly, bry };
    static const int box_r0[4] = { 0, 0, -1, -1 };  /* bl/br filled below (N dependent) */
    static const int box_c0[4] = { 0, -1, 0, -1 };
    const uint8_t *tpl[4] = { HX_TPL_PA, HX_TPL_PB, HX_TPL_PB, HX_TPL_PC };
    int r0[4], c0[4];
    for (int k = 0; k < 4; k++) {
        r0[k] = (box_r0[k] < 0) ? N - 7 : box_r0[k];
        c0[k] = (box_c0[k] < 0) ? N - 7 : box_c0[k];
    }

    for (int pass = 0; pass < 2; pass++) {
        for (int k = 0; k < 4; k++) {
            double save_x = *px[k], save_y = *py[k];
            double best_dx = 0.0, best_dy = 0.0;
            int best_score = 999;
            for (double dy = -2.0; dy <= 2.001; dy += 1.0) {
                for (double dx = -2.0; dx <= 2.001; dx += 1.0) {
                    *px[k] = save_x + dx;
                    *py[k] = save_y + dy;
                    HxPerspective t = hx_compute_perspective(
                        5.5, 5.5, *tlx, *tly,
                        (double)N - 5.5, 5.5, *trx, *tr_y,
                        1.5, (double)N - 1.5, *blx, *bly,
                        (double)N - 5.5, (double)N - 5.5, *brx, *bry);
                    int score = hx_template_score(bin, w, h, &t, r0[k], c0[k], tpl[k]);
                    if (score < best_score) { best_score = score; best_dx = dx; best_dy = dy; }
                }
            }
            *px[k] = save_x + best_dx;
            *py[k] = save_y + best_dy;
        }
    }
}

/* Apply one of the 8 orientation transforms (4 rotations x mirror) to a
 * source N x N grid, writing into dst (also N x N). */
static void hx_transform_grid(const uint8_t *src, uint8_t *dst, int N, int variant)
{
    int mirror = variant & 4;
    int rot = variant & 3;
    for (int r = 0; r < N; r++) {
        for (int c = 0; c < N; c++) {
            int sr = r, sc = c;
            if (mirror) sc = N - 1 - c;
            int fr, fc;
            switch (rot) {
            case 0: fr = sr; fc = sc; break;
            case 1: fr = sc; fc = N - 1 - sr; break;
            case 2: fr = N - 1 - sr; fc = N - 1 - sc; break;
            default: fr = N - 1 - sc; fc = sr; break;
            }
            dst[fr * N + fc] = src[r * N + c];
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 *  Top level
 * ════════════════════════════════════════════════════════════════════════ */

static int hx_try_decode_at(const uint8_t *bin, int w, int h,
                            const HxCand *tl, const HxCand *tr,
                            const HxCand *bl, const HxCand *br,
                            char *out, int out_cap)
{
    /* Only the top (TL-TR) and right (TR-BR) edges connect two finder
     * centers that both sit at the symmetric (k, k) / (N-1-k, k) style
     * offset from their own corner. The bottom-left finder's solid 3x3
     * block is *not* symmetric with the others (GB/T 21049 places an
     * unrotated "PB" pattern at both TR and BL, so at BL the block ends
     * up in the far corner: module (N-2, 1) rather than (N-6, 5)) -- so
     * any side length touching BL is skewed and must not be used to
     * estimate the module size / N. */
    double dx_top = tr->cx - tl->cx, dy_top = tr->cy - tl->cy;
    double dx_right = br->cx - tr->cx, dy_right = br->cy - tr->cy;
    double side_top = sqrt(dx_top * dx_top + dy_top * dy_top);
    double side_right = sqrt(dx_right * dx_right + dy_right * dy_right);
    double side = (side_top + side_right) / 2.0;

    double m_est = (tl->module_size + tr->module_size + br->module_size) / 3.0;
    if (m_est < 0.3) return 0;

    int n_est = (int)(side / m_est + 0.5) + 11;

    static uint8_t grid[HX_MAX_N * HX_MAX_N];
    static uint8_t xform[HX_MAX_N * HX_MAX_N];

    /* Sub-pixel refine the 4 anchors once, against the module geometry of
     * the best-guess size, before sweeping N. This absorbs the last ~1px of
     * finder-center noise that otherwise survives averaging and pushes a
     * couple of data bits past the RS(25,21) 2-byte correction budget on
     * noisier captures (see hanxin_inv.bmp) -- harmless no-op when the
     * coarse candidates are already accurate or the assumed orientation is
     * wrong (see hx_refine_corners), since every result is still gated by
     * the SI/data RS checks below. */
    double rtlx = tl->cx, rtly = tl->cy, rtrx = tr->cx, rtry = tr->cy;
    double rblx = bl->cx, rbly = bl->cy, rbrx = br->cx, rbry = br->cy;
    {
        int nref = n_est;
        if (nref < 23) nref = 23;
        if (nref > HX_MAX_N) nref = HX_MAX_N;
        hx_refine_corners(bin, w, h, nref, &rtlx, &rtly, &rtrx, &rtry, &rblx, &rbly, &rbrx, &rbry);
    }

    for (int dn = -4; dn <= 4; dn += 2) {
        int N = n_est + dn;
        if (N < 23 || N > HX_MAX_N) continue;
        if ((N - 23) & 1) continue;

        /* BL source module coordinate: (col 1.5, row N-1.5) -- see comment
         * above. This assumes the geometrically bottom-left detected
         * candidate really is the symbol's un-rotated bottom-left corner;
         * the 8-way rotation/mirror retry below still recovers rotated or
         * mirrored symbols because hx_decode_matrix's structural-info gate
         * rejects any mis-sampled grid before it can be misread. */
        HxPerspective t = hx_compute_perspective(
            5.5, 5.5, rtlx, rtly,
            (double)N - 5.5, 5.5, rtrx, rtry,
            1.5, (double)N - 1.5, rblx, rbly,
            (double)N - 5.5, (double)N - 5.5, rbrx, rbry);

        int ok = 1;
        for (int r = 0; r < N && ok; r++) {
            for (int c = 0; c < N; c++) {
                int b = hx_sample_bit(bin, w, h, &t, (double)c + 0.5, (double)r + 0.5);
                if (b < 0) { ok = 0; break; }
                grid[r * N + c] = (uint8_t)b;
            }
        }
        if (!ok) continue;

        for (int variant = 0; variant < 8; variant++) {
            hx_transform_grid(grid, xform, N, variant);
            if (hx_decode_matrix(xform, N, out, out_cap)) return 1;
        }
    }
    return 0;
}

int hanxin_decode(const Image *img, BarcodeResult *result)
{
    if (result) memset(result, 0, sizeof(*result));
    if (!img || !img->pixels || img->width < 23 || img->height < 23) return 0;

    static const int biases[] = { 0, -12, 12 };
    for (size_t bi = 0; bi < sizeof(biases) / sizeof(biases[0]); bi++) {
        uint8_t *bin = hx_binarize(img, biases[bi]);
        if (!bin) continue;

        HxCand rows[HX_MAX_CAND];
        HxCand cols[HX_MAX_CAND];
        int nrows = hx_scan_rows(bin, img->width, img->height, rows, HX_MAX_CAND);
        int ncols = hx_scan_cols(bin, img->width, img->height, cols, HX_MAX_CAND);

        if (getenv("HX_DEBUG2")) {
            for (int i = 0; i < nrows; i++)
                if (rows[i].cy > 350 && rows[i].cy < 440 && rows[i].cx > 540 && rows[i].cx < 650)
                    fprintf(stderr, "[hx] row-cand[%d] = (%.1f,%.1f) ms=%.2f cnt=%d\n", i, rows[i].cx, rows[i].cy, rows[i].module_size, rows[i].count);
            for (int i = 0; i < ncols; i++)
                if (cols[i].cy > 350 && cols[i].cy < 440 && cols[i].cx > 540 && cols[i].cx < 650)
                    fprintf(stderr, "[hx] col-cand[%d] = (%.1f,%.1f) ms=%.2f cnt=%d\n", i, cols[i].cx, cols[i].cy, cols[i].module_size, cols[i].count);
        }

        /* Intersect row/col candidates: confirmed 2D finder centers. */
        HxCand cand[HX_MAX_CAND];
        int ncand = 0;
        for (int i = 0; i < nrows && ncand < HX_MAX_CAND; i++) {
            for (int j = 0; j < ncols; j++) {
                double tol = (rows[i].module_size + cols[j].module_size) * 0.75 + 1.5;
                double dx = rows[i].cx - cols[j].cx;
                double dy = rows[i].cy - cols[j].cy;
                if (dx * dx + dy * dy < tol * tol) {
                    double cx = (rows[i].cx + cols[j].cx) / 2.0;
                    double cy = (rows[i].cy + cols[j].cy) / 2.0;
                    double ms = (rows[i].module_size + cols[j].module_size) / 2.0;
                    hx_add_candidate(cand, &ncand, HX_MAX_CAND, cx, cy, ms);
                    break;
                }
            }
        }

        if (getenv("HX_DEBUG")) {
            fprintf(stderr, "[hx] bias=%d nrows=%d ncols=%d ncand=%d\n", biases[bi], nrows, ncols, ncand);
            for (int i = 0; i < ncand; i++)
                fprintf(stderr, "[hx]   cand[%d] = (%.1f,%.1f) ms=%.2f\n", i, cand[i].cx, cand[i].cy, cand[i].module_size);
        }
        if (ncand < 4) { free(bin); continue; }

        /* Try successive clusters of similar module size, largest first. */
        int used[HX_MAX_CAND];
        memset(used, 0, sizeof(used));

        for (int attempt = 0; attempt < ncand; attempt++) {
            if (used[attempt]) continue;
            double ref_ms = cand[attempt].module_size;

            int idxs[HX_MAX_CAND];
            int nidx = 0;
            for (int i = 0; i < ncand; i++) {
                if (fabs(cand[i].module_size - ref_ms) < ref_ms * 0.18 + 0.3) {
                    idxs[nidx++] = i;
                }
            }
            if (nidx < 4) continue;

            /* Pick 4 extreme corner points by position. */
            int i_tl = idxs[0], i_br = idxs[0], i_tr = idxs[0], i_bl = idxs[0];
            for (int k = 1; k < nidx; k++) {
                int i = idxs[k];
                if (cand[i].cx + cand[i].cy < cand[i_tl].cx + cand[i_tl].cy) i_tl = i;
                if (cand[i].cx + cand[i].cy > cand[i_br].cx + cand[i_br].cy) i_br = i;
                if (cand[i].cx - cand[i].cy > cand[i_tr].cx - cand[i_tr].cy) i_tr = i;
                if (cand[i].cx - cand[i].cy < cand[i_bl].cx - cand[i_bl].cy) i_bl = i;
            }
            for (int k = 0; k < nidx; k++) used[idxs[k]] = 1;

            if (i_tl == i_br || i_tl == i_tr || i_tl == i_bl ||
                i_br == i_tr || i_br == i_bl || i_tr == i_bl) {
                continue;
            }

            /* Sanity-check that the 4 picked points actually form a
             * roughly square/consistent quad (comparable side lengths and
             * diagonals) before spending time on a full perspective+decode
             * attempt. This rejects quads assembled from unrelated noise
             * that merely happened to share a module-size bucket. */
            {
                double dxt = cand[i_tr].cx - cand[i_tl].cx, dyt = cand[i_tr].cy - cand[i_tl].cy;
                double dxb = cand[i_br].cx - cand[i_bl].cx, dyb = cand[i_br].cy - cand[i_bl].cy;
                double dxl = cand[i_bl].cx - cand[i_tl].cx, dyl = cand[i_bl].cy - cand[i_tl].cy;
                double dxr = cand[i_br].cx - cand[i_tr].cx, dyr = cand[i_br].cy - cand[i_tr].cy;
                double dxd1 = cand[i_br].cx - cand[i_tl].cx, dyd1 = cand[i_br].cy - cand[i_tl].cy;
                double dxd2 = cand[i_bl].cx - cand[i_tr].cx, dyd2 = cand[i_bl].cy - cand[i_tr].cy;
                double st = sqrt(dxt * dxt + dyt * dyt);
                double sb = sqrt(dxb * dxb + dyb * dyb);
                double sl = sqrt(dxl * dxl + dyl * dyl);
                double sr = sqrt(dxr * dxr + dyr * dyr);
                double d1 = sqrt(dxd1 * dxd1 + dyd1 * dyd1);
                double d2 = sqrt(dxd2 * dxd2 + dyd2 * dyd2);
                double smax = st, smin = st;
                if (sb > smax) smax = sb; if (sb < smin) smin = sb;
                if (sl > smax) smax = sl; if (sl < smin) smin = sl;
                if (sr > smax) smax = sr; if (sr < smin) smin = sr;
                if (smin < 1.0) continue;
                if (smax / smin > 1.6) continue;
                double dmax = d1 > d2 ? d1 : d2, dmin = d1 > d2 ? d2 : d1;
                if (dmin < 1.0 || dmax / dmin > 1.6) continue;
            }

            if (getenv("HX_DEBUG")) {
                fprintf(stderr, "[hx] attempt=%d nidx=%d tl=(%.1f,%.1f) tr=(%.1f,%.1f) bl=(%.1f,%.1f) br=(%.1f,%.1f) ms=%.2f\n",
                        attempt, nidx, cand[i_tl].cx, cand[i_tl].cy, cand[i_tr].cx, cand[i_tr].cy,
                        cand[i_bl].cx, cand[i_bl].cy, cand[i_br].cx, cand[i_br].cy, ref_ms);
            }

            char out[BARCODE_MAX_LEN];
            if (hx_try_decode_at(bin, img->width, img->height,
                                 &cand[i_tl], &cand[i_tr], &cand[i_bl], &cand[i_br],
                                 out, sizeof(out))) {
                free(bin);
                if (result) {
                    size_t len = strlen(out);
                    if (len >= sizeof(result->data)) len = sizeof(result->data) - 1;
                    memcpy(result->data, out, len);
                    result->data[len] = '\0';
                    result->length = (int)len;
                    result->symbology = BARCODE_SYM_HANXIN;
                    result->confidence = 95;
                }
                return 1;
            }
        }

        free(bin);
    }

    return 0;
}
