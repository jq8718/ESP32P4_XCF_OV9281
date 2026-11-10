#ifndef CODE39_H
#define CODE39_H

#include "barcode.h"

/* Decode Code39 directly from transition edges (x positions of alternating runs). */
int code39_decode_edges(const int *edges, int edge_count,
                        char *out, int out_cap,
                        int verify_checksum,
                        int decode_extended_ascii);

/* Decode Code39 from grayscale image. */
int code39_decode(const Image *img, BarcodeResult *result);

/* Helpers exposed for synthetic tests. */
int code39_char_to_symbol(char ch);
int code39_symbol_pattern(int symbol_index);

#endif /* CODE39_H */
