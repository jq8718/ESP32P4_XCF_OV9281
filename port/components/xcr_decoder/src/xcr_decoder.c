#include "xcr_decoder.h"
#include "xcr_dmtx.h"
#include "xcr_port.h"
#include "xcr_qr_geometry.h"

#include "barcode.h"
#include "codablock_f.h"
#include "datamatrix.h"
#include "qrcode.h"
#include "upc_ean.h"
int xcr_datamatrix_marked_decode(const Image *image, BarcodeResult *result, const XcrDeadline *deadline);
int xcr_datamatrix_marked_decode_ex(const Image *image, BarcodeResult *result, const XcrDeadline *deadline, int defer_geometry);
int xcr_datamatrix_marked_decode_prepared(const Image *image, BarcodeResult *result,
    const XcrDeadline *deadline, int defer_geometry, const XcrDmtxPrepared *prepared);

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if CONFIG_XCR_DATAMATRIX_LIBDMTX
int xcr_dmtx_decode(const Image *image, BarcodeResult *result, const XcrDeadline *deadline);
#endif

int xcr_quirc_decode(const Image *image, BarcodeResult *result,
                     XcrQrTrace *trace);
int xcr_quirc_decode_fast(const Image *image, BarcodeResult *result,
                         XcrQrTrace *trace);
int xcr_quirc_decode_bounded(const Image *image, BarcodeResult *result,
                            XcrQrTrace *trace, const XcrDeadline *deadline, int fast_search);

#define XCR_DEFAULT_DECODER_TIMEOUT_MS 2000U
#define XCR_DEFAULT_IMAGE_BUDGET_MS 10000U
#define XCR_DEFAULT_DIRECTION_ATTEMPTS 2U
#define XCR_MAX_DIRECTION_ATTEMPTS 16

struct XcrDecoder {
    XcrDecoderConfig config;
    XcrDecoderStats stats;
    volatile bool cancel_requested;
    bool busy;
    uint8_t *scratch_pixels;
    size_t scratch_capacity;
    XcrQrTrace qr_trace;
    const XcrDmtxPrepared *prepared;
};

static const int k_direction_vectors[XCR_MAX_DIRECTION_ATTEMPTS][2] = {
    {1, 0}, {2, 1}, {1, 1}, {1, 2},
    {0, 1}, {-1, 2}, {-1, 1}, {-2, 1},
    {-1, 0}, {-2, -1}, {-1, -1}, {-1, -2},
    {0, -1}, {1, -2}, {1, -1}, {2, -1},
};

static int xcr_abs_int(int value)
{
    return value < 0 ? -value : value;
}

static bool xcr_all_digits(const BarcodeResult *result)
{
    int index;

    if (!result || result->length <= 0) {
        return false;
    }
    for (index = 0; index < result->length; index++) {
        if (result->data[index] < '0' || result->data[index] > '9') {
            return false;
        }
    }
    return true;
}

static bool xcr_retail_mod10_valid(const BarcodeResult *result)
{
    int sum = 0;
    int weight = 3;
    int index;
    int expected_check;

    if (!xcr_all_digits(result) ||
        (result->length != 8 && result->length != 12 && result->length != 13)) {
        return false;
    }
    for (index = result->length - 2; index >= 0; index--) {
        sum += (result->data[index] - '0') * weight;
        weight = weight == 3 ? 1 : 3;
    }
    expected_check = (10 - (sum % 10)) % 10;
    return expected_check == result->data[result->length - 1] - '0';
}

static bool xcr_code128_candidate_allowed(XcrDecoder *decoder,
                                          const Image *image,
                                          const BarcodeResult *result)
{
    if (decoder->config.reject_ambiguous_retail_numeric &&
        xcr_all_digits(result) &&
        (result->length == 8 || result->length == 12 || result->length == 13) &&
        !xcr_retail_mod10_valid(result)) {
        return false;
    }
    if (decoder->config.cross_symbology_veto) {
        BarcodeResult codablock_result;
        uint64_t t_cb0 = xcr_port_time_us();
        memset(&codablock_result, 0, sizeof(codablock_result));
        int cb_len = 0;
        if (codablock_f_decode(image, &codablock_result) > 0) {
            cb_len = codablock_result.length;
        }
        if (getenv("XCR_TIMING"))
            fprintf(stderr, "  XCR t_codablock_ms=%llu len=%d cand_len=%d\n",
                    (unsigned long long)((xcr_port_time_us() - t_cb0) / 1000),
                    cb_len, result->length);
        if (cb_len > result->length) {
            return false;
        }
    }
    return true;
}

static uint64_t xcr_hash_result(int symbology, const char *data, int length)
{
    uint64_t hash = 1469598103934665603ULL;
    int index;

    hash ^= (uint64_t)(uint32_t)symbology;
    hash *= 1099511628211ULL;
    for (index = 0; index < length; index++) {
        hash ^= (uint8_t)data[index];
        hash *= 1099511628211ULL;
    }
    return hash;
}

static bool xcr_ensure_scratch(XcrDecoder *decoder, size_t required_size)
{
    uint8_t *new_pixels;

    if (required_size <= decoder->scratch_capacity) {
        return true;
    }
    new_pixels = (uint8_t *)xcr_alloc(required_size, XCR_MEM_LARGE);
    if (!new_pixels) {
        return false;
    }
    xcr_free(decoder->scratch_pixels);
    decoder->scratch_pixels = new_pixels;
    decoder->scratch_capacity = required_size;
    return true;
}

static bool xcr_prepare_image(XcrDecoder *decoder,
                              const XcrGrayImage *input,
                              Image *output,
                              bool *uses_scratch)
{
    int stride;
    int roi_x;
    int roi_y;
    int roi_width;
    int roi_height;
    size_t pixel_count;
    int row;

    if (!decoder || !input || !output || !uses_scratch || !input->pixels ||
        input->width <= 0 || input->height <= 0) {
        return false;
    }
    stride = input->stride > 0 ? input->stride : input->width;
    roi_x = input->roi_width > 0 ? input->roi_x : 0;
    roi_y = input->roi_height > 0 ? input->roi_y : 0;
    roi_width = input->roi_width > 0 ? input->roi_width : input->width;
    roi_height = input->roi_height > 0 ? input->roi_height : input->height;

    if (stride < input->width || roi_x < 0 || roi_y < 0 || roi_width <= 0 ||
        roi_height <= 0 || roi_x > input->width - roi_width ||
        roi_y > input->height - roi_height) {
        return false;
    }

    output->width = roi_width;
    output->height = roi_height;
    if (roi_x == 0 && roi_y == 0 && roi_width == input->width &&
        roi_height == input->height && stride == input->width) {
        output->pixels = (uint8_t *)input->pixels;
        *uses_scratch = false;
        return true;
    }

    if ((size_t)roi_width > SIZE_MAX / (size_t)roi_height) {
        return false;
    }
    pixel_count = (size_t)roi_width * (size_t)roi_height;
    if (!xcr_ensure_scratch(decoder, pixel_count)) {
        return false;
    }
    for (row = 0; row < roi_height; row++) {
        const uint8_t *source_row = input->pixels +
            (size_t)(roi_y + row) * (size_t)stride + (size_t)roi_x;
        memcpy(decoder->scratch_pixels + (size_t)row * (size_t)roi_width,
               source_row,
               (size_t)roi_width);
    }
    output->pixels = decoder->scratch_pixels;
    *uses_scratch = true;
    return true;
}

static bool xcr_image_looks_inverted(const Image *image)
{
    int start_x;
    int end_x;
    int start_y;
    int end_y;
    int dark_count = 0;
    int light_count = 0;
    int x;
    int y;
    int total;

    if (!image || !image->pixels || image->width <= 0 || image->height <= 0) {
        return false;
    }
    start_x = image->width / 4;
    end_x = image->width * 3 / 4;
    start_y = image->height / 4;
    end_y = image->height * 3 / 4;
    for (y = start_y; y < end_y; y += 4) {
        for (x = start_x; x < end_x; x += 4) {
            if (image->pixels[(size_t)y * (size_t)image->width + (size_t)x] < 128) {
                dark_count++;
            } else {
                light_count++;
            }
        }
    }
    total = dark_count + light_count;
    return total > 0 && (dark_count * 100) / total > 25;
}

static void xcr_invert_image(Image *image)
{
    size_t pixel_count;
    size_t index;

    pixel_count = (size_t)image->width * (size_t)image->height;
    for (index = 0; index < pixel_count; index++) {
        image->pixels[index] = (uint8_t)(255U - image->pixels[index]);
    }
}

static bool xcr_image_has_variation(const Image *image)
{
    uint8_t first_pixel;
    size_t pixel_count;
    size_t index;

    if (!image || !image->pixels || image->width <= 0 || image->height <= 0) {
        return false;
    }
    pixel_count = (size_t)image->width * (size_t)image->height;
    first_pixel = image->pixels[0];
    for (index = 1; index < pixel_count; index++) {
        if (image->pixels[index] != first_pixel) {
            return true;
        }
    }
    return false;
}

static int xcr_direction_score(const Image *image, int direction_x, int direction_y)
{
    int center_x = image->width / 2;
    int center_y = image->height / 2;
    int perpendicular_x = -direction_y;
    int perpendicular_y = direction_x;
    int score = 0;
    int step;

    for (step = 1; step <= 20; step++) {
        int x0 = center_x - direction_x * step;
        int y0 = center_y - direction_y * step;
        int x1 = center_x + direction_x * step;
        int y1 = center_y + direction_y * step;
        int px0 = center_x - perpendicular_x * step;
        int py0 = center_y - perpendicular_y * step;
        int px1 = center_x + perpendicular_x * step;
        int py1 = center_y + perpendicular_y * step;
        int along0;
        int along1;
        int across0;
        int across1;

        if (x0 < 0 || y0 < 0 || x1 < 0 || y1 < 0 || px0 < 0 || py0 < 0 ||
            px1 < 0 || py1 < 0 || x0 >= image->width || x1 >= image->width ||
            px0 >= image->width || px1 >= image->width || y0 >= image->height ||
            y1 >= image->height || py0 >= image->height || py1 >= image->height) {
            continue;
        }
        along0 = image->pixels[(size_t)y0 * (size_t)image->width + (size_t)x0];
        along1 = image->pixels[(size_t)y1 * (size_t)image->width + (size_t)x1];
        across0 = image->pixels[(size_t)py0 * (size_t)image->width + (size_t)px0];
        across1 = image->pixels[(size_t)py1 * (size_t)image->width + (size_t)px1];
        score += xcr_abs_int(along1 - along0) * 3;
        score += xcr_abs_int((along1 + along0) - (across1 + across0));
    }
    return score;
}

static int xcr_rank_directions(const Image *image, int *ranked_indices, int capacity)
{
    int scores[XCR_MAX_DIRECTION_ATTEMPTS];
    bool used[XCR_MAX_DIRECTION_ATTEMPTS] = {false};
    int direction_index;
    int output_index;

    for (direction_index = 0; direction_index < XCR_MAX_DIRECTION_ATTEMPTS;
         direction_index++) {
        scores[direction_index] = xcr_direction_score(
            image,
            k_direction_vectors[direction_index][0],
            k_direction_vectors[direction_index][1]);
    }
    for (output_index = 0; output_index < capacity; output_index++) {
        int best_index = -1;
        int best_score = INT_MIN;
        for (direction_index = 0; direction_index < XCR_MAX_DIRECTION_ATTEMPTS;
             direction_index++) {
            if (!used[direction_index] && scores[direction_index] > best_score) {
                best_score = scores[direction_index];
                best_index = direction_index;
            }
        }
        if (best_index < 0) {
            break;
        }
        used[best_index] = true;
        ranked_indices[output_index] = best_index;
    }
    return output_index;
}

static int xcr_remaining_ms(const XcrDeadline *deadline, uint32_t attempt_limit_ms)
{
    uint64_t now_us;
    uint64_t remaining_us;
    uint64_t remaining_ms;

    if (!deadline || deadline->deadline_us == 0) {
        return (int)attempt_limit_ms;
    }
    now_us = xcr_port_time_us();
    if (now_us >= deadline->deadline_us) {
        return 0;
    }
    remaining_us = deadline->deadline_us - now_us;
    remaining_ms = (remaining_us + 999ULL) / 1000ULL;
    if (remaining_ms > attempt_limit_ms) {
        remaining_ms = attempt_limit_ms;
    }
    return (int)remaining_ms;
}

static int xcr_decode_once(XcrDecoder *decoder,
                           const Image *image,
                           const XcrDeadline *deadline,
                           BarcodeResult *result)
{
    int ranked_indices[XCR_MAX_DIRECTION_ATTEMPTS];
    int ranked_count;
    int attempt_count;
    int attempt_index;
    int has_guard;

    if (decoder->config.enable_upc_ean && !xcr_deadline_expired(deadline)) {
        uint64_t t_uc0 = xcr_port_time_us();
        memset(result, 0, sizeof(*result));
        if (upc_ean_decode(image, result) > 0 && result->length > 0) {
            result->symbology = BARCODE_SYM_UPC_EAN;
            return 1;
        }
        if (getenv("XCR_TIMING"))
            fprintf(stderr, "  XCR t_upc_ms=%llu\n",
                    (unsigned long long)((xcr_port_time_us() - t_uc0) / 1000));
    }
    if (!decoder->config.enable_code128 || xcr_deadline_expired(deadline)) {
        return 0;
    }

    attempt_count = decoder->config.code128_direction_attempts;
    if (attempt_count < 1) {
        attempt_count = 1;
    }
    if (attempt_count > XCR_MAX_DIRECTION_ATTEMPTS) {
        attempt_count = XCR_MAX_DIRECTION_ATTEMPTS;
    }
    if (attempt_count == 1) {
        has_guard = true;
    } else {
        uint64_t t_guard0 = xcr_port_time_us();
        has_guard = code128_guard_prescreen(image);
        if (getenv("XCR_TIMING"))
            fprintf(stderr, "  XCR t_guard_ms=%llu has_guard=%d\n",
                    (unsigned long long)((xcr_port_time_us() - t_guard0) / 1000), has_guard);
        if (!has_guard) {
            attempt_count = 1;
        }
    }
    {
        uint64_t t_rank0 = xcr_port_time_us();
        ranked_count = xcr_rank_directions(image, ranked_indices, attempt_count);
        if (getenv("XCR_TIMING"))
            fprintf(stderr, "  XCR t_rank_ms=%llu ranked=%d attempts=%d\n",
                    (unsigned long long)((xcr_port_time_us() - t_rank0) / 1000),
                    ranked_count, attempt_count);
    }
    for (attempt_index = 0; attempt_index < ranked_count; attempt_index++) {
        int direction_index = ranked_indices[attempt_index];
        int timeout_ms = xcr_remaining_ms(deadline, decoder->config.decoder_timeout_ms);

        if (timeout_ms <= 0 || xcr_deadline_expired(deadline)) {
            break;
        }
        memset(result, 0, sizeof(*result));
        {
            uint64_t t_at0 = xcr_port_time_us();
            int dec = code128_decode_hint(image,
                                          result,
                                          k_direction_vectors[direction_index][0],
                                          k_direction_vectors[direction_index][1],
                                          timeout_ms) > 0 &&
                      result->length > 0 &&
                      xcr_code128_candidate_allowed(decoder, image, result);
            if (getenv("XCR_TIMING"))
                fprintf(stderr, "  XCR t_attempt%d_ms=%llu dec=%d dir=(%d,%d) tmo=%d\n",
                        attempt_index,
                        (unsigned long long)((xcr_port_time_us() - t_at0) / 1000),
                        dec,
                        k_direction_vectors[direction_index][0],
                        k_direction_vectors[direction_index][1],
                        timeout_ms);
            if (dec) {
                result->symbology = BARCODE_SYM_CODE128;
                return 1;
            }
        }
    }
    return 0;
}

static int xcr_decode_upc_ean_once(const XcrDecoder *decoder,
                                   const Image *image,
                                   const XcrDeadline *deadline,
                                   BarcodeResult *result)
{
    if (!decoder->config.enable_upc_ean || xcr_deadline_expired(deadline)) {
        return 0;
    }
    memset(result, 0, sizeof(*result));
    if (upc_ean_decode(image, result) > 0 && result->length > 0) {
        result->symbology = BARCODE_SYM_UPC_EAN;
        return 1;
    }
    return 0;
}

static int xcr_decode_qr_once(XcrDecoder *decoder,
                              const Image *image,
                              const XcrDeadline *deadline,
                              BarcodeResult *result)
{
    if (!decoder->config.enable_qr || xcr_deadline_expired(deadline) ||
        !image || image->width < 21 || image->height < 21 ||
        image->width > image->height * 2 || image->height > image->width * 2) {
        return 0;
    }
    memset(result, 0, sizeof(*result));
    int found = xcr_quirc_decode_bounded(image, result, &decoder->qr_trace,
                                        deadline, decoder->config.qr_fast_search);
    if (found > 0 &&
        result->length > 0) {
        result->symbology = BARCODE_SYM_QR;
        return 1;
    }
    if (decoder->config.qr_quirc_only) {
        return 0;
    }
    if (qrcode_decode(image, result) > 0 && result->length > 0) {
        result->symbology = BARCODE_SYM_QR;
        return 1;
    }
    if (decoder->config.polarity_retry) {
        XcrDecoder *mutable_decoder = (XcrDecoder *)decoder;
        size_t pixel_count = (size_t)image->width * (size_t)image->height;
        Image inverted = *image;
        if (xcr_ensure_scratch(mutable_decoder, pixel_count)) {
            memcpy(mutable_decoder->scratch_pixels, image->pixels, pixel_count);
            for (size_t index = 0; index < pixel_count; index++) {
                mutable_decoder->scratch_pixels[index] =
                    (uint8_t)(255U - mutable_decoder->scratch_pixels[index]);
            }
            inverted.pixels = mutable_decoder->scratch_pixels;
            memset(result, 0, sizeof(*result));
            if (qrcode_decode(&inverted, result) > 0 && result->length > 0) {
                result->symbology = BARCODE_SYM_QR;
                return 1;
            }
        }
    }
    return 0;
}

static int xcr_decode_datamatrix_once(XcrDecoder *decoder,
                                     const Image *image,
                                     const XcrDeadline *deadline,
                                     BarcodeResult *result)
{
    if (!decoder->config.enable_datamatrix || xcr_deadline_expired(deadline) ||
        !image || image->width < 10 || image->height < 10) {
        return 0;
    }
    memset(result, 0, sizeof(*result));
#if CONFIG_XCR_DATAMATRIX_LIBDMTX
    int decoded = xcr_dmtx_decode(image, result, deadline);
#else
    int decoded = xcr_datamatrix_marked_decode_prepared(image, result, deadline,
        decoder->config.datamatrix_defer_geometry, decoder->prepared);
#endif
    if (decoded > 0 && result->length > 0) {
        result->symbology = BARCODE_SYM_DATAMATRIX;
        return 1;
    }
    return 0;
}

static int xcr_decode_fast_rows_once(XcrDecoder *decoder,
                                     const Image *image,
                                     const XcrDeadline *deadline,
                                     BarcodeResult *result)
{
    int timeout_ms;

    if (!decoder->config.enable_code128 || xcr_deadline_expired(deadline)) {
        return 0;
    }
    timeout_ms = xcr_remaining_ms(deadline, decoder->config.decoder_timeout_ms);
    if (decoder->config.code128_fast_rows_only && timeout_ms > 180 &&
        image->height > 160) {
        timeout_ms = 180;
    }
    if (timeout_ms <= 0) {
        return 0;
    }
    memset(result, 0, sizeof(*result));
    int fast_decoded = code128_decode_fast_rows(image, result, timeout_ms);
    code128_fast_rows_clear_deadline();
    if (fast_decoded > 0 &&
        result->length > 0 &&
        xcr_code128_candidate_allowed(decoder, image, result)) {
        result->symbology = BARCODE_SYM_CODE128;
        return 1;
    }
    return 0;
}

static void xcr_copy_result(const XcrGrayImage *image,
                            const BarcodeResult *source,
                            XcrBarcodeResult *destination)
{
    int roi_x = image->roi_width > 0 ? image->roi_x : 0;
    int roi_y = image->roi_height > 0 ? image->roi_y : 0;
    int roi_width = image->roi_width > 0 ? image->roi_width : image->width;
    int roi_height = image->roi_height > 0 ? image->roi_height : image->height;
    int length = source->length;

    if (length < 0) {
        length = 0;
    }
    if (length >= XCR_BARCODE_MAX_LEN) {
        length = XCR_BARCODE_MAX_LEN - 1;
    }
    memset(destination, 0, sizeof(*destination));
    memcpy(destination->data, source->data, (size_t)length);
    destination->data[length] = '\0';
    destination->length = length;
    destination->confidence = source->confidence;
    destination->geometry_quality = source->geometry_quality;
    destination->grid_rows = source->grid_rows;
    destination->grid_cols = source->grid_cols;
    destination->geometry_seed_valid = source->geometry_seed_valid;
    if (source->geometry_seed_valid) {
        for (int point = 0; point < 4; point++) {
            destination->geometry_seed[2 * point] = source->geometry_seed[2 * point] + roi_x;
            destination->geometry_seed[2 * point + 1] = source->geometry_seed[2 * point + 1] + roi_y;
        }
    }
    destination->geometry_edge_mask = source->geometry_edge_mask;
    destination->geometry_support = source->geometry_support;
    destination->geometry_residual = source->geometry_residual;
    destination->subpixel_valid = source->subpixel_valid && source->geometry_valid &&
        source->geometry_quality == XCR_GEOMETRY_REFINED;
    if (destination->subpixel_valid) {
        for (int point = 0; point < 4; point++) {
            destination->subpixel_corners[2 * point] = source->subpixel_corners[2 * point] + roi_x;
            destination->subpixel_corners[2 * point + 1] = source->subpixel_corners[2 * point + 1] + roi_y;
        }
    }
    destination->symbology = source->symbology == BARCODE_SYM_DATAMATRIX
        ? XCR_SYM_DATAMATRIX : source->symbology;
    destination->coordinate_space = source->geometry_valid
        ? XCR_COORDINATE_SENSOR : XCR_COORDINATE_DECODE_ROI;
    if (source->geometry_valid) {
        for (int point = 0; point < 4; point++) {
            destination->corners[2 * point] = source->corners[2 * point] + roi_x;
            destination->corners[2 * point + 1] = source->corners[2 * point + 1] + roi_y;
        }
        destination->geometry_valid = 1;
    }
    destination->roi_x = roi_x;
    destination->roi_y = roi_y;
    destination->roi_width = roi_width;
    destination->roi_height = roi_height;
    destination->scan_y = -1;
    destination->sequence = image->sequence;
    destination->capture_generation = image->capture_generation;
    destination->timestamp_us = image->timestamp_us;
    destination->validation_flags = XCR_VALIDATION_CHECKSUM |
                                    XCR_VALIDATION_FORMAT |
                                    XCR_VALIDATION_LENGTH;
    destination->dedup_key = xcr_hash_result(
        destination->symbology,
        destination->data,
        destination->length);
}

void xcr_decoder_config_default(XcrDecoderConfig *config)
{
    if (!config) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->enable_code128 = true;
    config->enable_upc_ean = true;
    config->enable_qr = true;
    config->enable_datamatrix = true;
#if defined(CONFIG_XCR_ENABLE_DATAMATRIX) && !CONFIG_XCR_ENABLE_DATAMATRIX
    config->enable_datamatrix = false;
#endif
#if defined(CONFIG_XCR_DATAMATRIX_ONLY) && CONFIG_XCR_DATAMATRIX_ONLY
    config->enable_code128 = false;
    config->enable_upc_ean = false;
    config->enable_qr = false;
#endif
    config->polarity_retry = true;
    config->cross_symbology_veto = true;
    config->reject_ambiguous_retail_numeric = true;
    config->decoder_timeout_ms = XCR_DEFAULT_DECODER_TIMEOUT_MS;
    config->image_budget_ms = XCR_DEFAULT_IMAGE_BUDGET_MS;
    config->code128_direction_attempts = XCR_DEFAULT_DIRECTION_ATTEMPTS;
    config->code128_fast_rows = false;
    config->code128_fast_rows_only = false;
    config->qr_quirc_only = false;
    config->qr_fast_search = false;
}

XcrDecoder *xcr_decoder_create(const XcrDecoderConfig *config)
{
    XcrDecoderConfig effective_config;
    XcrDecoder *decoder;

    if (config) {
        effective_config = *config;
    } else {
        xcr_decoder_config_default(&effective_config);
    }
    if (effective_config.decoder_timeout_ms == 0) {
        effective_config.decoder_timeout_ms = XCR_DEFAULT_DECODER_TIMEOUT_MS;
    }
    decoder = (XcrDecoder *)xcr_calloc(1, sizeof(*decoder), XCR_MEM_FAST);
    if (!decoder) {
        return NULL;
    }
    decoder->config = effective_config;
    return decoder;
}

void xcr_decoder_destroy(XcrDecoder *decoder)
{
    if (!decoder) {
        return;
    }
    code128_set_cancel_flag(NULL);
    xcr_free(decoder->scratch_pixels);
    xcr_free(decoder);
}

XcrStatus xcr_decoder_decode(XcrDecoder *decoder,
                             const XcrGrayImage *image,
                             XcrBarcodeResult *results,
                             int max_results,
                             int *out_result_count)
{
    XcrDeadline deadline;
    Image vendor_image;
    BarcodeResult vendor_result;
    bool uses_scratch = false;
    bool decoded = false;
    uint64_t started_us;
    uint64_t elapsed_us;
    XcrStatus status;

    if (out_result_count) {
        *out_result_count = 0;
    }
    if (!decoder || !image || !results || max_results <= 0 || !out_result_count) {
        return XCR_INVALID_ARGUMENT;
    }
    if (decoder->busy) {
        return XCR_INTERNAL_ERROR;
    }
    decoder->busy = true;
    xcr_memory_oom_clear();
    started_us = xcr_port_time_us();
    deadline.cancel_flag = &decoder->cancel_requested;
    deadline.deadline_us = decoder->config.image_budget_ms > 0
        ? started_us + (uint64_t)decoder->config.image_budget_ms * 1000ULL
        : 0;
    if (image->deadline_us && (!deadline.deadline_us || image->deadline_us < deadline.deadline_us)) {
        deadline.deadline_us = image->deadline_us;
    }
    decoder->stats.decode_calls++;
    memset(&decoder->qr_trace, 0, sizeof(decoder->qr_trace));
    memset(&vendor_image, 0, sizeof(vendor_image));
    memset(&vendor_result, 0, sizeof(vendor_result));

    if (!xcr_prepare_image(decoder, image, &vendor_image, &uses_scratch)) {
        status = XCR_OOM;
        decoder->stats.oom_calls++;
        goto finish;
    }

    if ((decoder->config.enable_qr && !decoder->config.enable_code128 &&
         !decoder->config.enable_upc_ean) ||
        xcr_image_has_variation(&vendor_image)) {
        code128_begin_new_image();
        code128_set_cancel_flag(&decoder->cancel_requested);

        if (decoder->config.enable_qr && !xcr_deadline_expired(&deadline)) {
            decoded = xcr_decode_qr_once(decoder, &vendor_image,
                                         &deadline, &vendor_result) > 0;
        }

        if (!decoded && decoder->config.enable_datamatrix &&
            !xcr_deadline_expired(&deadline)) {
            decoded = xcr_decode_datamatrix_once(decoder, &vendor_image,
                                                 &deadline, &vendor_result) > 0;
        }

        if (!decoded && decoder->config.code128_fast_rows &&
            !xcr_deadline_expired(&deadline)) {
            decoded = xcr_decode_fast_rows_once(decoder, &vendor_image,
                                                &deadline, &vendor_result) > 0;
        }

        if (!decoded && decoder->config.enable_qr &&
            (decoder->config.enable_code128 || decoder->config.enable_upc_ean) &&
            decoder->config.code128_fast_rows_only &&
            !xcr_deadline_expired(&deadline)) {
            decoded = xcr_decode_qr_once(decoder, &vendor_image,
                                         &deadline, &vendor_result) > 0;
        }

        if (!decoded && decoder->config.code128_fast_rows_only &&
            decoder->config.enable_upc_ean &&
            !xcr_deadline_expired(&deadline)) {
            decoded = xcr_decode_upc_ean_once(decoder, &vendor_image,
                                              &deadline, &vendor_result) > 0;
        }

        if (!decoded && decoder->config.code128_fast_rows_only &&
            (decoder->config.enable_code128 || decoder->config.enable_upc_ean) &&
            decoder->config.polarity_retry &&
            xcr_image_looks_inverted(&vendor_image) &&
            !xcr_deadline_expired(&deadline)) {
            size_t pixel_count = (size_t)vendor_image.width * (size_t)vendor_image.height;
            if (!uses_scratch) {
                if (!xcr_ensure_scratch(decoder, pixel_count)) {
                    status = XCR_OOM;
                    decoder->stats.oom_calls++;
                    goto finish;
                }
                memcpy(decoder->scratch_pixels, vendor_image.pixels, pixel_count);
                vendor_image.pixels = decoder->scratch_pixels;
                uses_scratch = true;
            }
            xcr_invert_image(&vendor_image);
            decoded = xcr_decode_fast_rows_once(decoder, &vendor_image,
                                                &deadline, &vendor_result) > 0;
            if (!decoded) {
                decoded = xcr_decode_upc_ean_once(decoder, &vendor_image,
                                                  &deadline, &vendor_result) > 0;
            }
        }

        if (!decoded && !decoder->config.code128_fast_rows_only &&
            (decoder->config.enable_code128 || decoder->config.enable_upc_ean) &&
            decoder->config.polarity_retry &&
            xcr_image_looks_inverted(&vendor_image) && !xcr_deadline_expired(&deadline)) {
            /* Inverted barcode (dark background): try the inverted polarity
             * first with a short budget. A real inverted image decodes in one
             * fast pass, whereas decoding normal polarity first burns the full
             * per-attempt timeout on a scan that can never match an inverted
             * pattern (code128_inv was 4400ms → ~240ms). On a false positive,
             * restore the original polarity and fall back to normal with the
             * full budget. */
            size_t pixel_count = (size_t)vendor_image.width * (size_t)vendor_image.height;
            if (!uses_scratch) {
                if (!xcr_ensure_scratch(decoder, pixel_count)) {
                    status = XCR_OOM;
                    decoder->stats.oom_calls++;
                    goto finish;
                }
                memcpy(decoder->scratch_pixels, vendor_image.pixels, pixel_count);
                vendor_image.pixels = decoder->scratch_pixels;
                uses_scratch = true;
            }
            xcr_invert_image(&vendor_image);
            {
                XcrDeadline inv_deadline = deadline;
                uint64_t now_us = xcr_port_time_us();
                uint64_t cap_us = 400ULL * 1000ULL;
                if (inv_deadline.deadline_us == 0 ||
                    inv_deadline.deadline_us > now_us + cap_us) {
                    inv_deadline.deadline_us = now_us + cap_us;
                }
                decoded = xcr_decode_once(decoder, &vendor_image, &inv_deadline, &vendor_result) > 0;
            }
            if (!decoded && !xcr_deadline_expired(&deadline)) {
                xcr_invert_image(&vendor_image);
                decoded = xcr_decode_once(decoder, &vendor_image, &deadline, &vendor_result) > 0;
            }
        } else if (!decoded && !decoder->config.code128_fast_rows_only &&
                   (decoder->config.enable_code128 || decoder->config.enable_upc_ean)) {
            decoded = xcr_decode_once(decoder, &vendor_image, &deadline, &vendor_result) > 0;
        }
    }

    if (image->deadline_us && xcr_port_time_us() >= image->deadline_us) {
        decoded = 0;
    }
    if (decoded) {
        xcr_copy_result(image, &vendor_result, &results[0]);
        if (vendor_result.symbology == BARCODE_SYM_QR && decoder->qr_trace.geometry_valid) {
            for (int point = 0; point < 4; point++) {
                results[0].corners[2 * point] = decoder->qr_trace.corners[2 * point] + results[0].roi_x;
                results[0].corners[2 * point + 1] = decoder->qr_trace.corners[2 * point + 1] + results[0].roi_y;
            }
            uint64_t geometry_start = xcr_port_time_us();
            XcrDeadline geometry_deadline = deadline;
            if (!geometry_deadline.deadline_us || geometry_deadline.deadline_us > geometry_start + 10000) {
                geometry_deadline.deadline_us = geometry_start + 10000;
            }
            results[0].geometry_valid = xcr_qr_refine_corners(image->pixels, image->width,
                                                              image->height, image->stride > 0 ? image->stride : image->width,
                                                              results[0].corners, &geometry_deadline);
            decoder->qr_trace.geometry_refine_us = (uint32_t)(xcr_port_time_us() - geometry_start);
            decoder->qr_trace.geometry_refined = results[0].geometry_valid;
            decoder->qr_trace.geometry_valid = results[0].geometry_valid;
            if (results[0].geometry_valid) {
                for (int point = 0; point < 4; point++) {
                    decoder->qr_trace.corners[point * 2] = results[0].corners[point * 2] - results[0].roi_x;
                    decoder->qr_trace.corners[point * 2 + 1] = results[0].corners[point * 2 + 1] - results[0].roi_y;
                }
            }
            results[0].coordinate_space = XCR_COORDINATE_SENSOR;
        }
        *out_result_count = 1;
    }
    if (image->deadline_us && xcr_port_time_us() >= image->deadline_us) {
        decoded = false;
        *out_result_count = 0;
    }
    if (decoder->cancel_requested) {
        status = XCR_CANCELLED;
        decoder->stats.cancelled_calls++;
    } else if (xcr_memory_oom_seen()) {
        status = XCR_OOM;
        decoder->stats.oom_calls++;
    } else if (xcr_deadline_expired(&deadline)) {
        status = decoded ? XCR_TIMEOUT_WITH_VALID_RESULT : XCR_TIMEOUT_NO_RESULT;
        decoder->stats.timeout_calls++;
    } else if (decoded) {
        status = XCR_OK;
        decoder->stats.successful_calls++;
    } else {
        status = XCR_NO_RESULT;
        decoder->stats.no_result_calls++;
    }

finish:
    code128_set_cancel_flag(NULL);
    if (*out_result_count > 0) {
        results[0].status = status;
    }
    elapsed_us = xcr_port_time_us() - started_us;
    decoder->stats.total_decode_us += elapsed_us;
    if (elapsed_us > decoder->stats.max_decode_us) {
        decoder->stats.max_decode_us = elapsed_us;
    }
    decoder->busy = false;
    return status;
}

XcrStatus xcr_decoder_decode_prepared(XcrDecoder *decoder, const XcrGrayImage *image,
    const XcrDmtxPrepared *prepared, XcrBarcodeResult *results, int max_results,
    int *out_result_count)
{
    if (!decoder || decoder->busy || !image || !prepared ||
        prepared->pixels != image->pixels || prepared->sequence != image->sequence ||
        prepared->width != image->width || prepared->height != image->height ||
        image->roi_width > 0 || image->roi_height > 0 ||
        (image->stride && image->stride != image->width)) {
        if (out_result_count) *out_result_count = 0;
        return XCR_INVALID_ARGUMENT;
    }
    decoder->prepared = prepared;
    XcrStatus status = xcr_decoder_decode(decoder, image, results, max_results, out_result_count);
    decoder->prepared = NULL;
    return status;
}

void xcr_decoder_cancel(XcrDecoder *decoder)
{
    if (decoder) {
        decoder->cancel_requested = true;
    }
}

void xcr_decoder_clear_cancel(XcrDecoder *decoder)
{
    if (decoder) {
        decoder->cancel_requested = false;
    }
}

void xcr_decoder_get_stats(const XcrDecoder *decoder, XcrDecoderStats *out)
{
    if (decoder && out) {
        *out = decoder->stats;
    }
}

void xcr_decoder_get_qr_trace(const XcrDecoder *decoder, XcrQrTrace *out)
{
    if (!out) {
        return;
    }
    memset(out, 0, sizeof(*out));
    if (decoder) {
        *out = decoder->qr_trace;
    }
}

const char *xcr_qr_stage_name(XcrQrStage stage)
{
    switch (stage) {
    case XCR_QR_STAGE_CENTER_8X:
        return "center8";
    case XCR_QR_STAGE_FULL_8X:
        return "full8";
    case XCR_QR_STAGE_FULL_4X:
        return "full4";
    case XCR_QR_STAGE_FULL_2X:
        return "full2";
    case XCR_QR_STAGE_FULL_4X_ADAPTIVE:
        return "full4_adaptive";
    case XCR_QR_STAGE_CANDIDATE_4X:
        return "candidate4";
    case XCR_QR_STAGE_CANDIDATE_2X:
        return "candidate2";
    default:
        return "none";
    }
}

const char *xcr_status_name(XcrStatus status)
{
    static const char *names[] = {
        "OK",
        "NO_RESULT",
        "TIMEOUT_NO_RESULT",
        "TIMEOUT_WITH_VALID_RESULT",
        "CANCELLED",
        "OOM",
        "INVALID_ARGUMENT",
        "INTERNAL_ERROR",
    };

    if (status < XCR_OK || status > XCR_INTERNAL_ERROR) {
        return "UNKNOWN";
    }
    return names[status];
}
