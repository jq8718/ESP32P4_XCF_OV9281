#ifndef I2OF5_H
#define I2OF5_H

#include "barcode.h"

/* Interleaved 2 of 5 (ITF) decoder entry point. */
int i2of5_decode(const Image *img, BarcodeResult *result);

#endif /* I2OF5_H */
