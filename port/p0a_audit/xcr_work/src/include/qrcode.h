#ifndef QRCODE_H
#define QRCODE_H

#include "barcode.h"

typedef struct {
    uint32_t binarize_attempts;
    uint32_t perspective_candidates;
    uint32_t perspective_vote_wins;
    uint32_t voted_sampling_attempts;
    uint32_t finder_stage_hits;
    uint32_t grid_sampling_attempts;
    uint32_t format_decode_hits;
    uint32_t ec_decode_hits;
    uint32_t payload_decode_hits;
    uint32_t probe_calls;
    uint32_t probe_max_finders;
    uint32_t global_calls;
} QRDecodeDebugStats;

typedef struct {
    double align_score_weak;
    double align_score_strong;
    int oob_reject_percent;
    int oob_penalty_percent;
    int weak_vote_penalty_percent;
    double skew_bias_penalty_th;
} QRVoteTuning;

/*
 * Decode a QR code from a grayscale image.
 * Returns 1 on success, 0 if no QR code found.
 */
int qrcode_decode(const Image *img, BarcodeResult *result);
void qrcode_debug_stats_reset(void);
void qrcode_debug_stats_get(QRDecodeDebugStats *out);

/* Test hooks for format/version correction path alignment. */
int qrcode_test_decode_format_words(uint16_t bits_primary, uint16_t bits_secondary,
                                    int *ec_level, int *mask_pattern,
                                    int *used_copy, int *bit_errors);
int qrcode_test_decode_version_words(uint32_t bits_primary, uint32_t bits_secondary,
                                     int estimated_version, int *version_out,
                                     int *used_copy, int *bit_errors, int *agree);
int qrcode_test_estimate_version_geometry(double d_top, double d_left, double module_size);
int qrcode_test_refine_alignment_anchor(const uint8_t *bin, int w, int h, int version,
                                        double tl_x, double tl_y,
                                        double tr_x, double tr_y,
                                        double bl_x, double bl_y,
                                        double *align_x, double *align_y,
                                        double *match_score);
int qrcode_test_compose_vote_score(int format_used_copy, int format_errors,
                                   int version, int version_decode_ok,
                                   int version_agree, int version_errors,
                                   int alignment_used, double align_score,
                                   int sample_out_of_bounds,
                                   int sample_weak_votes,
                                   int sample_total_modules,
                                   double perspective_skew,
                                   double br_bias_magnitude);
int qrcode_test_score_sampling_quality(const uint8_t *grid, int modules,
                                       int format_used_copy, int format_errors,
                                       int version, int version_used_copy,
                                       int version_errors, int version_agree,
                                       int alignment_used, double align_score,
                                       int sample_out_of_bounds,
                                       int sample_weak_votes,
                                       int sample_total_modules,
                                       double perspective_skew,
                                       double br_bias_magnitude);
void qrcode_test_get_vote_tuning(QRVoteTuning *out);
void qrcode_test_set_vote_tuning(const QRVoteTuning *in);
void qrcode_test_reset_vote_tuning(void);

#endif /* QRCODE_H */
