#ifndef USPS4ST_H
#define USPS4ST_H

#include "barcode.h"

/*
 * USPS Intelligent Mail Barcode (IMb / USPS4CB, 4-state postal) decoder.
 * Public standard USPS-B-3200. Decodes 65 four-state bars into a 31 (or
 * fewer, depending on routing code length) digit tracking+routing string.
 */
int usps4st_decode(const Image *img, BarcodeResult *result);

#endif /* USPS4ST_H */
