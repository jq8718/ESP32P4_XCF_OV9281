#ifndef KOREAPOST_H
#define KOREAPOST_H

#include "barcode.h"

int koreapost_decode(const Image *img, BarcodeResult *result);

#endif /* KOREAPOST_H */
