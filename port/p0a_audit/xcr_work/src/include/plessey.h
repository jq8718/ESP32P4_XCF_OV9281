#ifndef PLESSEY_H
#define PLESSEY_H

#include "barcode.h"

int plessey_decode(const Image *img, BarcodeResult *result);

#endif /* PLESSEY_H */
