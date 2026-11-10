#ifndef VALIDATION_H
#define VALIDATION_H

#include "barcode.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Post-decode result validation — ported from sub_2DB6C analysis.
 *
 * Conservative checks only: rejects results that are CERTAINLY wrong.
 * Does not reject ambiguous or borderline cases.
 */

/* Validate a single decoded result.
 * Returns 1 if result passes validation, 0 if rejected. */
int validate_result(const BarcodeResult *result);

/* Validate and filter an array of results in-place.
 * Removes invalid results by compacting the array.
 * Returns the new count after filtering. */
int validate_results(BarcodeResult *results, int count);

#ifdef __cplusplus
}
#endif

#endif /* VALIDATION_H */
