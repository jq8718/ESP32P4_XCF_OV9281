#ifndef UPC_EAN_H
#define UPC_EAN_H

#include "barcode.h"

/*
 * UPC/EAN barcode decoder — restored from decompiled libFsaDecoderCore1.
 *
 * Supports: EAN-13, EAN-8, UPC-A, UPC-E
 * Returns >0 on success (number of results), 0 if no barcode found.
 */
int upc_ean_decode(const Image *img, BarcodeResult *result);

#endif /* UPC_EAN_H */
