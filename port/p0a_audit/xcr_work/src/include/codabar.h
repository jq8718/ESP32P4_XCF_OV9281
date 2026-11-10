#ifndef CODABAR_H
#define CODABAR_H

#include "barcode.h"

/* Restored Codabar (NW-7) decoder entry point. */
int codabar_decode(const Image *img, BarcodeResult *result);

#endif /* CODABAR_H */
