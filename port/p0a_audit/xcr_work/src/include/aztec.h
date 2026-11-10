#ifndef AZTEC_H
#define AZTEC_H

#include "barcode.h"

typedef struct {
    uint32_t decode_attempts;
    uint32_t bullseye_hits;
    uint32_t mode_decode_hits;
    uint32_t data_extract_hits;
    uint32_t ec_decode_hits;
    uint32_t payload_decode_hits;
} AztecDecodeDebugStats;

/*
 * Decode an Aztec barcode from a grayscale image.
 * Returns >0 on success (decoded length),
 *          0 if no barcode found,
 *         <0 on error.
 */
int aztec_decode(const Image *img, BarcodeResult *result);

/*
 * Decode from raw pixel grid (for testing without BMP).
 * grid: row-major uint8_t array, 0=black, 255=white.
 * w, h: grid dimensions.
 */
int aztec_decode_grid(const uint8_t *grid, int w, int h, char *out, int out_cap);
void aztec_debug_stats_reset(void);
void aztec_debug_stats_get(AztecDecodeDebugStats *out);

/* Test hooks for mode/data boundary validation. */
int aztec_test_decode_mode_compact_bits(const int *bits, int *layers, int *data_cw,
                                        int *ec_cw, int *cw_bits, int *total_cw);
int aztec_test_decode_mode_full_bits(const int *bits, int *layers, int *data_cw,
                                     int *ec_cw, int *cw_bits, int *total_cw);
int aztec_test_decode_data_codewords(const int *cw, int n_data, int cw_bits,
                                     char *out, int out_cap);
int aztec_test_find_bullseye_geometry(const uint8_t *grid, int w, int h,
                                      int *cx, int *cy, int *compact,
                                      int *mod_size_x_pos, int *mod_size_x_neg,
                                      int *mod_size_y_pos, int *mod_size_y_neg);
int aztec_test_build_full_layer_pairs(int layer, int *pairs, int max_pairs);
void aztec_test_set_full_sampling_tuning(int side_samples_bias, int side_round_bias,
                                         int center_jitter, int scale_jitter);
void aztec_test_get_full_sampling_tuning(int *side_samples_bias, int *side_round_bias,
                                         int *center_jitter, int *scale_jitter);
void aztec_test_reset_full_sampling_tuning(void);

#endif /* AZTEC_H */
