#ifndef RSS_GS1_H
#define RSS_GS1_H

#include "barcode.h"

/*
 * RSS/GS1 DataBar decoders.
 *
 * Restored workflow anchors from decompiled source:
 * - ClearRss()                      @ libFsaDecoderCore1.c:220954
 * - SmartSymbolRedef0006016295B()   @ libFsaDecoderCore1.c:221033
 * - fmwlsymbol005f()                @ libFsaDecoderCore1.c:221053 (RSS-14)
 * - fmwlsymbol0061()                @ libFsaDecoderCore1.c:222931 (RSS Limited)
 * - fmwlsymbol0062()                @ libFsaDecoderCore1.c:223600 (RSS Expanded)
 */
int rss_gs1_decode(const Image *img, BarcodeResult *result);
int rss_gs1_decode_fast(const Image *img, BarcodeResult *result);
int rss14_stacked_decode(const Image *img, BarcodeResult *result);
int rss_limited_decode(const Image *img, BarcodeResult *result);
int rss_expanded_decode(const Image *img, BarcodeResult *result);
int rss_expanded_stacked_decode(const Image *img, BarcodeResult *result);

#endif /* RSS_GS1_H */
