#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stddef.h>

/* ── Image ──────────────────────────────────────────────────────────── */

typedef struct {
    int      width;
    int      height;
    uint8_t *pixels;   /* row-major, 1 byte per pixel (grayscale) */
} Image;

/* Load an 8-bit or 24-bit uncompressed BMP.  Returns 0 on success. */
int  image_load_bmp(const char *path, Image *img);
void image_free(Image *img);

/* Polarity inversion support */
int  image_is_inverted(const Image *img);
void image_invert(Image *img);

/* ── Code128 decoder ────────────────────────────────────────────────── */

#define BARCODE_MAX_LEN 256

typedef struct {
    char  data[BARCODE_MAX_LEN];
    int   length;
    int   confidence;   /* 0-100, higher = more scanlines agreed */
    int   symbology;    /* BarcodeSymbology */
} BarcodeResult;

typedef enum {
    BARCODE_SYM_UNKNOWN = 0,
    BARCODE_SYM_UPC_EAN,
    BARCODE_SYM_CODE128,
    BARCODE_SYM_CODE39,
    BARCODE_SYM_CODABAR,
    BARCODE_SYM_COMPOSITE,
    BARCODE_SYM_RSS_GS1,
    BARCODE_SYM_QR,
    BARCODE_SYM_AZTEC,
BARCODE_SYM_I2OF5,
BARCODE_SYM_CODE93,
BARCODE_SYM_DATAMATRIX,
BARCODE_SYM_GRIDMATRIX,
BARCODE_SYM_HANXIN,
BARCODE_SYM_KOREAPOST,
BARCODE_SYM_MAXICODE,
BARCODE_SYM_PDF417,
BARCODE_SYM_S25,
BARCODE_SYM_CODE11,
BARCODE_SYM_MSI,
BARCODE_SYM_TELEPEN,
BARCODE_SYM_PLESSEY,
BARCODE_SYM_CODABLOCKF,
BARCODE_SYM_USPS4ST,
} BarcodeSymbology;

/*
 * Decode a Code128 barcode from a grayscale image.
 * Returns >0  on success (number of results),
 *          0  if no barcode found.
 */
int code128_decode(const Image *img, BarcodeResult *result);
int code128_decode_hint(const Image *img, BarcodeResult *result, int dir_x, int dir_y, int timeout_ms);

/*
 * Must be called once per INPUT FILE, before the first decode_image() call
 * for that file (i.e. in the CLI/harness loop over BMP files, not inside
 * decode_image() itself) -- callers such as main.c and test_decode_all.c
 * invoke decode_image() up to twice per file (original polarity, then an
 * inverted-polarity retry if the first attempt found nothing); both count
 * as the same logical image. Bumps an internal generation counter that
 * invalidates the UP16-env far-field cross-CALL vote pool (see up16_vote_*
 * statics inside code128_decode_hint): that pool is intentionally persisted
 * *across* multiple code128_decode_hint invocations -- including across the
 * original/inverted-retry decode_image() pair -- for the same logical image
 * (different scan-dispatch families/modes/polarity re-render the same
 * physical barcode, so a genuine far-field payload can independently
 * surface on different calls at different y-rows or polarities) but must
 * never leak between two different input files.
 */
void code128_begin_new_image(void);

/*
 * Cheap Code128 start/stop guard-pattern existence prescreen.
 * Returns 1 if the image plausibly contains Code128 guard structure
 * (or the check is inconclusive -> fail-open), 0 if no guard pairs
 * were found on any scan channel (safe to downgrade the hint budget).
 * Deliberately biased to over-admit; never intended to reject real
 * Code128/depth images.
 */
int code128_guard_prescreen(const Image *img);

/*
 * Block1a: cheap Channel-A-only quick scan. Returns the count (0-9) of raw
 * horizontal scanlines containing at least one strict-parity guard pair.
 * Used only to gate the strong-guard front-run in decode_core.c; does not
 * replace code128_guard_prescreen() above.
 */
int code128_guard_chA_rows_strict(const Image *img);

typedef struct {
    int used_adaptive;
    int probe_count;
    int best_probe_y;
    int best_probe_score;
    int mean_contrast;
    int mean_grad;
    int mean_edge_den;
    int low_contrast_mode;
    int far_distance_mode;
    int trigger_enhance;
    int n_scan_y;
    int n_avg;
    int n_tilt;
} Code128AdaptivePlanDiag;

int code128_debug_plan_scanlines(const Image *img, int has_hint, int use_adaptive,
                                 int *scan_y, int max_scan_y,
                                 Code128AdaptivePlanDiag *diag);

#endif /* BARCODE_H */
