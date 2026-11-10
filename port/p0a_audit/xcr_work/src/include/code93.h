#ifndef CODE93_H
#define CODE93_H

#include "barcode.h"

/* Decode Code93 directly from transition edges (x positions of alternating runs). */
int code93_decode_edges(const int *edges, int edge_count,
                        char *out, int out_cap);

/* Decode Code93 from grayscale image. */
int code93_decode(const Image *img, BarcodeResult *result);

/* Strict Code93 decode: requires both C and K checksums to pass (no
 * checksum-bypass fallback). Zero false positives observed across the full
 * coverage + depth test corpora; safe to run in the early low-cost layer. */
int code93_decode_high_conf(const Image *img, BarcodeResult *result);

#endif /* CODE93_H */
