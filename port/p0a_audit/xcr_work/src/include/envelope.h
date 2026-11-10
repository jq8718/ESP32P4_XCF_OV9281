#ifndef ENVELOPE_H
#define ENVELOPE_H

#include <stdint.h>

/*
 * Envelope-adaptive barcode edge detection pipeline.
 * Ported from restored firmware algorithms (RESTORE_PLAN.md §2-6).
 *
 * Pipeline: sample → envelope → edge_detect → envelope_edge_filter → edges
 */

/* §2: In-place first derivative: signal[i] = signal[i+1] - signal[i] */
void env_first_derivative(int *signal, int length);

/* §3: DMFI adaptive threshold from derivative histogram */
int env_dmfi_thresh(const int *signal, int length);

/* §5: Bidirectional exponential smoothing envelope (IIR) */
void env_compute_envelope(const int *signal, int length,
                          int *upper, int *lower,
                          int alpha_upper, int alpha_lower);

/* §6: Envelope-adaptive peak-valley edge detector */
int env_edge_detect(const int *signal, int *output,
                    const int *upper_env, const int *lower_env,
                    int length);

/* §4: Envelope-normalized strong edge filter */
int env_edge_filter(const int *signal, const int *edges_in,
                    const int *upper_env, const int *lower_env,
                    int n_edges, int *edges_out);

/* Full pipeline: row pixels → edge positions */
int envelope_decode_scanline(const uint8_t *row, int width,
                             int *edges_out, int max_edges);

#endif /* ENVELOPE_H */
