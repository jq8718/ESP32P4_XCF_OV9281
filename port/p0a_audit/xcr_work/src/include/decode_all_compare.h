#ifndef DECODE_ALL_COMPARE_H
#define DECODE_ALL_COMPARE_H

#include "barcode.h"

typedef struct {
    int raw_match;
    int normalized_match;
    int format_only_mismatch;
    char expected_normalized[BARCODE_MAX_LEN];
    char actual_normalized[BARCODE_MAX_LEN];
} DecodeAllCompareResult;

void decode_all_compare_strings(const char *symbology,
                                const char *expected,
                                const char *actual,
                                DecodeAllCompareResult *out);

#endif /* DECODE_ALL_COMPARE_H */
