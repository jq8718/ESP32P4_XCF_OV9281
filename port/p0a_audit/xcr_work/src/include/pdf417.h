#ifndef PDF417_H
#define PDF417_H

#include "barcode.h"

int pdf417_decode(const Image *img, BarcodeResult *result);

/* MicroPDF417 (ISO/IEC 24728) and GS1 Composite CC-A component decode.
 * Handles both a standalone MicroPDF417 symbol and, when invoked on a GS1
 * Composite image, the CC-A 2D component stacked above the linear symbol. */
int micropdf417_decode(const Image *img, BarcodeResult *result);

#endif /* PDF417_H */
