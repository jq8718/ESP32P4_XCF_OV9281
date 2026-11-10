#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdint.h>

#define MAX_SIGNAL_LEN 4096
#define MAX_EDGES 512

int signal_sample(const uint8_t *image, int width, int height,
                  int x0, int y0, int x1, int y1,
                  int *signal, int max_len);

void signal_envelope(const int *signal, int len,
                     int *thresh_low, int *thresh_high);

int signal_find_edges(const int *signal, int *work_buf, int len,
                      int thresh_low, int thresh_high,
                      int *edges, int max_edges, int flags);

/* Like signal_find_edges but returns edges in Q4 fixed-point (16x sub-pixel precision). */
int signal_find_edges_q4(const int *signal, int *work_buf, int len,
                         int thresh_low, int thresh_high,
                         int *edges_q4, int max_edges, int flags);

int signal_sinc_max(int a1, int a2);
int signal_sinc_min(int a1, int a2);

/* Far-field edge chain primitives (sub_15BA6C), exposed for use as an
 * additional fallback source when the existing far-distance Code128 paths
 * fail on a given ROI. See src/core/signal.c for provenance/semantics. */
void signal_point_spread(int *x0, int *y0, int *x1, int *y1,
                          int num_a3, unsigned int den_a4, int num_a5);
void signal_far_envelope(const int *raw, int n, int *upper_env, int *lower_env,
                          int attack_shift, int decay_shift);

int signal_transitions(const uint8_t *image, int width, int height,
                       int x0, int y0, int x1, int y1,
                       int *edges, int max_edges);
/* Code128-only deep conditioning chain (sub_2AE40/2AE7C/2AEB8 style). */
int signal_transitions_code128(const uint8_t *image, int width, int height,
                               int x0, int y0, int x1, int y1,
                               int *edges, int max_edges);

#endif
