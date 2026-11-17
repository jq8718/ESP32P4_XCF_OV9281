#ifndef DATAMATRIX_H
#define DATAMATRIX_H

#include "barcode.h"

typedef struct {
    uint32_t decode_attempts;
    uint32_t finder_hits;
    uint32_t grid_decode_hits;
    uint32_t ec_decode_hits;
    uint32_t payload_decode_hits;
    uint64_t finder_us;
    uint64_t candidate_us;
    uint64_t codeword_us;
    uint64_t ecc_us;
    uint64_t payload_us;
    uint64_t total_us;
} DataMatrixDecodeDebugStats;

/*
 * Decode a DataMatrix barcode from a grayscale image.
 * Returns >0 on success (decoded length),
 *          0 if no barcode found,
 *         <0 on error.
 */
int datamatrix_decode(const Image *img, BarcodeResult *result);
int datamatrix_decode_aligned(const Image *img, BarcodeResult *result);
/* Known 10x10 module grid: 1=dark, row 0 and column 0 are solid borders.
 * Validates borders and uses the same ECC200 placement/RS/payload engine.
 * The caller supplies image geometry after a successful decode. */
int datamatrix_decode_grid10(const uint8_t modules[100], BarcodeResult *result);

void datamatrix_debug_stats_reset(void);
void datamatrix_debug_stats_get(DataMatrixDecodeDebugStats *out);

#endif /* DATAMATRIX_H */
