#ifndef SIG_UTIL_H
#define SIG_UTIL_H

#include "barcode.h"
#include <stdint.h>
#include <stddef.h>

/* FNV-1a 64-bit hash (standard offset basis). */
uint64_t sig_fnv1a64(const uint8_t *p, size_t n);

/*
 * Compact image signature from center 7-row scan band.
 * Binarizes each row at midpoint threshold, packs bits, hashes with FNV-1a.
 * Appends width+height for disambiguation.
 */
uint64_t sig_image_signature(const Image *img);

/*
 * Full-image signature: FNV-1a over all pixels + width + height.
 * More expensive but distinguishes images that share the same center band.
 */
uint64_t sig_image_signature_full(const Image *img);

#endif /* SIG_UTIL_H */
