#ifndef TELEPEN_H
#define TELEPEN_H

#include "barcode.h"

/*
 * AIM USS Telepen decoder (Full ASCII + Compressed Numeric modes).
 * Returns >0 on success and fills `result` (result->symbology set to
 * BARCODE_SYM_TELEPEN). Returns 0 if no valid Telepen symbol is found.
 */
int telepen_decode(const Image *img, BarcodeResult *result);

#endif
