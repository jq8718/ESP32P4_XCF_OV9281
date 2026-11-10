#ifndef CODABLOCK_F_H
#define CODABLOCK_F_H

#include "barcode.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Codablock F decoder: multi-row stacked Code128 symbol.
 * Each row is a self-contained Code128 codeword sequence:
 *   [StartA][subset selector][row indicator][7 data codewords][row check][Stop]
 * Rows are tied together by the row indicator/subset scheme and a pair of
 * K1/K2 mod-86 checksum codewords carried in the last row.
 *
 * Returns >0 on success (1 result written to *result), 0 if not found.
 */
int codablock_f_decode(const Image *img, BarcodeResult *result);

#ifdef __cplusplus
}
#endif

#endif /* CODABLOCK_F_H */
