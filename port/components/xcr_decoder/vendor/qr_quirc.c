#include "barcode.h"
#include "quirc.h"
#include "quirc_internal.h"
#include "xcr_decoder.h"
#include "xcr_port.h"

#include <stdlib.h>
#include <math.h>
#include <string.h>

#define XCR_QR_MAX_CANDIDATES 4

static _Thread_local const XcrDeadline *s_qr_deadline;

static int xcr_qr_abort(void *context)
{
    return xcr_deadline_expired((const XcrDeadline *)context);
}

static int xcr_qr_expired(void)
{
    return s_qr_deadline && xcr_qr_abort((void *)s_qr_deadline);
}

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} xcr_qr_candidate_box_t;

static int xcr_abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int xcr_max_int(int left, int right)
{
    return left > right ? left : right;
}

static int xcr_min_int(int left, int right)
{
    return left < right ? left : right;
}

static void xcr_record_qr_timing(XcrQrTrace *trace,
                                 XcrQrStage stage,
                                 int part,
                                 uint64_t elapsed_us)
{
    uint32_t *target = NULL;

    if (!trace) {
        return;
    }
    switch (stage) {
    case XCR_QR_STAGE_FULL_4X:
    case XCR_QR_STAGE_FULL_2X:
    case XCR_QR_STAGE_FULL_4X_ADAPTIVE:
        target = part == 0 ? &trace->fallback_sample_us :
                part == 1 ? &trace->fallback_identify_us :
                part == 2 ? &trace->fallback_decode_us : &trace->fallback_refine_us;
        break;
    case XCR_QR_STAGE_FULL_8X:
        target = part == 0 ? &trace->full8_sample_us :
                part == 1 ? &trace->full8_identify_us :
                part == 2 ? &trace->full8_decode_us : &trace->full8_refine_us;
        break;
    case XCR_QR_STAGE_CANDIDATE_4X:
    case XCR_QR_STAGE_CANDIDATE_2X:
        target = part == 0 ? &trace->candidate_sample_us :
                part == 1 ? &trace->candidate_identify_us :
                part == 2 ? &trace->candidate_decode_us : &trace->candidate_refine_us;
        break;
    default:
        return;
    }
    if (elapsed_us > UINT32_MAX - *target) {
        *target = UINT32_MAX;
    } else {
        *target += (uint32_t)elapsed_us;
    }
}

static void xcr_add_candidate_box(xcr_qr_candidate_box_t *boxes,
                                  int *box_count,
                                  int left,
                                  int top,
                                  int right,
                                  int bottom)
{
    int width;
    int height;
    int center_x;
    int center_y;

    if (!boxes || !box_count || *box_count >= XCR_QR_MAX_CANDIDATES ||
        right < left || bottom < top) {
        return;
    }
    width = right - left + 1;
    height = bottom - top + 1;
    center_x = (left + right) / 2;
    center_y = (top + bottom) / 2;

    for (int index = 0; index < *box_count; index++) {
        int old_width = boxes[index].right - boxes[index].left + 1;
        int old_height = boxes[index].bottom - boxes[index].top + 1;
        int center_tolerance_x = xcr_max_int(24, xcr_min_int(width, old_width) / 3);
        int center_tolerance_y = xcr_max_int(24, xcr_min_int(height, old_height) / 3);

        if (xcr_abs_int(center_x - (boxes[index].left + boxes[index].right) / 2) <=
                center_tolerance_x &&
            xcr_abs_int(center_y - (boxes[index].top + boxes[index].bottom) / 2) <=
                center_tolerance_y) {
            boxes[index].left = xcr_min_int(boxes[index].left, left);
            boxes[index].top = xcr_min_int(boxes[index].top, top);
            boxes[index].right = xcr_max_int(boxes[index].right, right);
            boxes[index].bottom = xcr_max_int(boxes[index].bottom, bottom);
            return;
        }
    }

    boxes[*box_count].left = left;
    boxes[*box_count].top = top;
    boxes[*box_count].right = right;
    boxes[*box_count].bottom = bottom;
    (*box_count)++;
}

static void xcr_fill_quirc_buffer(struct quirc *decoder,
                                  const Image *image,
                                  int origin_x,
                                  int origin_y,
                                  int region_width,
                                  int region_height,
                                  int sample_scale,
                                  int mirror,
                                  int anti_alias,
                                  int *sampled_width,
                                  int *sampled_height)
{
    uint8_t *buffer;
    int width;
    int height;

    width = (region_width + sample_scale - 1) / sample_scale;
    height = (region_height + sample_scale - 1) / sample_scale;
    buffer = quirc_begin(decoder, &width, &height);
    if (!buffer) {
        *sampled_width = 0;
        *sampled_height = 0;
        return;
    }

    if (!mirror && anti_alias == 1 && sample_scale >= 4 &&
        region_width % sample_scale == 0 && region_height % sample_scale == 0) {
        int offset = sample_scale / 4;
        int separation = sample_scale - 2 * offset;
        for (int row = 0; row < height; row++) {
            const uint8_t *upper = image->pixels +
                (size_t)(origin_y + row * sample_scale + offset) * image->width + origin_x + offset;
            const uint8_t *lower = upper + (size_t)separation * image->width;
            uint8_t *destination = buffer + (size_t)row * width;
            for (int column = 0; column < width; column++) {
                unsigned sum = upper[0] + upper[separation] + lower[0] + lower[separation];
                destination[column] = (uint8_t)(sum >> 2);
                upper += sample_scale;
                lower += sample_scale;
            }
        }
        *sampled_width = width;
        *sampled_height = height;
        return;
    }
    if (!mirror && anti_alias == 2 && sample_scale == 4 &&
        region_width % 4 == 0 && region_height % 4 == 0) {
        for (int row = 0; row < height; row++) {
            const uint8_t *first = image->pixels + (size_t)(origin_y + row * 4) * image->width + origin_x;
            const uint8_t *second = first + image->width;
            const uint8_t *third = second + image->width;
            const uint8_t *fourth = third + image->width;
            uint8_t *destination = buffer + (size_t)row * width;
            for (int column = 0; column < width; column++) {
                unsigned sum = first[0] + first[1] + first[2] + first[3] +
                               second[0] + second[1] + second[2] + second[3] +
                               third[0] + third[1] + third[2] + third[3] +
                               fourth[0] + fourth[1] + fourth[2] + fourth[3];
                destination[column] = (uint8_t)((sum + 8) >> 4);
                first += 4;
                second += 4;
                third += 4;
                fourth += 4;
            }
        }
        *sampled_width = width;
        *sampled_height = height;
        return;
    }
    for (int y = 0; y < height; y++) {
        int base_y = y * sample_scale;
        for (int x = 0; x < width; x++) {
            int base_x = x * sample_scale;
            if (anti_alias == 2) {
                unsigned int sum = 0;
                unsigned int count = 0;
                for (int sample_y = base_y; sample_y < xcr_min_int(base_y + sample_scale, region_height); sample_y++) {
                    for (int sample_x = base_x; sample_x < xcr_min_int(base_x + sample_scale, region_width); sample_x++) {
                        int source_x = mirror ? region_width - 1 - sample_x : sample_x;
                        sum += image->pixels[(size_t)(origin_y + sample_y) * image->width + origin_x + source_x];
                        count++;
                    }
                }
                buffer[(size_t)y * width + x] = (uint8_t)((sum + count / 2) / count);
            } else if (!anti_alias) {
                int source_x = xcr_min_int(base_x, region_width - 1);
                const uint8_t *source = image->pixels +
                    (size_t)(origin_y + xcr_min_int(base_y, region_height - 1)) *
                    (size_t)image->width + origin_x;
                if (mirror) {
                    source_x = region_width - 1 - source_x;
                }
                buffer[(size_t)y * (size_t)width + (size_t)x] = source[source_x];
            } else {
                int offset = sample_scale / 4;
                int x0 = xcr_min_int(base_x + offset, region_width - 1);
                int x1 = xcr_min_int(base_x + sample_scale - offset, region_width - 1);
                int y0 = xcr_min_int(base_y + offset, region_height - 1);
                int y1 = xcr_min_int(base_y + sample_scale - offset, region_height - 1);
                int source_x0 = mirror ? region_width - 1 - x0 : x0;
                int source_x1 = mirror ? region_width - 1 - x1 : x1;
                size_t row0 = (size_t)(origin_y + y0) * (size_t)image->width + origin_x;
                size_t row1 = (size_t)(origin_y + y1) * (size_t)image->width + origin_x;
                unsigned int sum = image->pixels[row0 + source_x0];
                sum += image->pixels[row0 + source_x1];
                sum += image->pixels[row1 + source_x0];
                sum += image->pixels[row1 + source_x1];
                buffer[(size_t)y * (size_t)width + (size_t)x] = (uint8_t)(sum / 4U);
            }
        }
    }
    *sampled_width = width;
    *sampled_height = height;
}

static int xcr_adaptive_threshold(struct quirc *decoder, int radius, uint32_t bias)
{
    int width;
    int height;
    uint8_t *pixels = quirc_begin(decoder, &width, &height);
    if (!pixels || width <= 0 || height <= 0 || radius < 0) {
        return 0;
    }
    int history_rows = xcr_min_int(height, radius + 1);
    size_t columns_bytes = (size_t)width * sizeof(uint32_t);
    uint32_t *columns = calloc(1, columns_bytes * 2 + (size_t)width * history_rows);
    if (!columns) {
        return 0;
    }
    uint32_t *window_widths = columns + width;
    uint8_t *history = (uint8_t *)(window_widths + width);
    for (int column = 0; column < width; column++) {
        window_widths[column] = (uint32_t)(xcr_min_int(width, column + radius + 1) -
                                          xcr_max_int(0, column - radius));
    }
    for (int row = 0; row < history_rows; row++) {
        const uint8_t *source = pixels + (size_t)row * width;
        for (int column = 0; column < width; column++) {
            columns[column] += source[column];
        }
    }
    for (int row = 0; row < height; row++) {
        int top = xcr_max_int(0, row - radius);
        int bottom = xcr_min_int(height, row + radius + 1);
        uint8_t *destination = pixels + (size_t)row * width;
        memcpy(history + (size_t)(row % history_rows) * width, destination, width);
        uint32_t sum = 0;
        for (int column = 0; column < xcr_min_int(width, radius + 1); column++) {
            sum += columns[column];
        }
        for (int column = 0; column < width; column++) {
            uint32_t area = (uint32_t)(bottom - top) * window_widths[column];
            destination[column] = ((uint32_t)destination[column] + bias) * area < sum ? 0 : 255;
            if (column >= radius) sum -= columns[column - radius];
            if (column + radius + 1 < width) sum += columns[column + radius + 1];
        }
        if (row >= radius) {
            const uint8_t *leaving = history + (size_t)((row - radius) % history_rows) * width;
            for (int column = 0; column < width; column++) {
                columns[column] -= leaving[column];
            }
        }
        if (row + radius + 1 < height) {
            const uint8_t *entering = pixels + (size_t)(row + radius + 1) * width;
            for (int column = 0; column < width; column++) {
                columns[column] += entering[column];
            }
        }
    }
    free(columns);
    return 1;
}

static int xcr_qr_grid_complete(const struct quirc_code *code, int width, int height)
{
    if (code->size < 21 || code->size > QUIRC_MAX_GRID_SIZE) {
        return 0;
    }
    for (int point = 0; point < 4; point++) {
        if (code->corners[point].x < -1 || code->corners[point].y < -1 ||
            code->corners[point].x > width + 1 || code->corners[point].y > height + 1) {
            return 0;
        }
    }
    for (int quadrant = 0; quadrant < 4; quadrant++) {
        int left = quadrant & 1 ? code->size / 2 : 0;
        int right = quadrant & 1 ? code->size : code->size / 2;
        int top = quadrant & 2 ? code->size / 2 : 0;
        int bottom = quadrant & 2 ? code->size : code->size / 2;
        unsigned black = 0;
        for (int row = top; row < bottom; row++) {
            for (int column = left; column < right; column++) {
                int offset = row * code->size + column;
                black += (code->cell_bitmap[offset >> 3] >> (offset & 7)) & 1;
            }
        }
        if (black * 12U < (unsigned)((right - left) * (bottom - top))) {
            return 0;
        }
    }
    return 1;
}

static int xcr_finish_quirc(struct quirc *decoder,
                            BarcodeResult *result,
                            int origin_x,
                            int origin_y,
                            int region_width,
                            int region_height,
                            int sample_scale,
                            int mirror,
                            xcr_qr_candidate_box_t *boxes,
                            int *box_count,
                            XcrQrTrace *trace,
                            XcrQrStage stage,
                            int allow_refine,
                            int sample_offset)
{
    struct quirc_code *code;
    struct quirc_data *data;
    int code_count;

    uint64_t identify_start_us = xcr_port_time_us();
    quirc_end_fast(decoder);
    if (trace && decoder->num_capstones > trace->finder_count) {
        trace->finder_count = (uint8_t)decoder->num_capstones;
    }
    code_count = quirc_count(decoder);
    xcr_record_qr_timing(trace, stage, 1,
                         xcr_port_time_us() - identify_start_us);
    if (code_count <= 0 || xcr_qr_expired()) {
        return 0;
    }

    code = malloc(sizeof(*code));
    data = malloc(sizeof(*data));
    if (!code || !data) {
        free(data);
        free(code);
        return 0;
    }

    uint64_t decode_start_us = xcr_port_time_us();
    uint64_t refinement_us = 0;
    for (int index = 0; index < code_count; index++) {
        if (xcr_qr_expired()) break;
        quirc_decode_error_t error;
        int transposed = 0;
        int min_x = region_width;
        int min_y = region_height;
        int max_x = 0;
        int max_y = 0;
        int complete = 0;

        for (int attempt = 0; attempt < (allow_refine ? 2 : 1); attempt++) {
            if (xcr_qr_expired()) break;
            if (attempt != 0) {
                uint64_t refine_start_us = xcr_port_time_us();
                quirc_refine(decoder, index);
                uint64_t elapsed_us = xcr_port_time_us() - refine_start_us;
                refinement_us += elapsed_us;
                xcr_record_qr_timing(trace, stage, 3, elapsed_us);
            }
            quirc_extract(decoder, index, code);
            complete = xcr_qr_grid_complete(code, decoder->w, decoder->h);
            if (!complete) {
                break;
            }
            transposed = 0;
            error = quirc_decode(code, data);
            if (error == QUIRC_ERROR_DATA_ECC) {
                quirc_flip(code);
                transposed = 1;
                error = quirc_decode(code, data);
            }
            if (error == QUIRC_SUCCESS && data->payload_len > 0) {
                break;
            }
        }
        if (xcr_qr_expired()) break;
        if (!complete) {
            if (trace && trace->incomplete_count < UINT8_MAX) {
                trace->incomplete_count++;
            }
            continue;
        }
        for (int point = 0; point < 4; point++) {
            int mapped_x = mirror
                ? origin_x + region_width - 1 - code->corners[point].x * sample_scale
                : origin_x + code->corners[point].x * sample_scale;
            int mapped_y = origin_y + code->corners[point].y * sample_scale;
            min_x = xcr_min_int(min_x, mapped_x);
            min_y = xcr_min_int(min_y, mapped_y);
            max_x = xcr_max_int(max_x, mapped_x);
            max_y = xcr_max_int(max_y, mapped_y);
        }
        if (boxes && box_count) {
            xcr_add_candidate_box(boxes, box_count, min_x, min_y, max_x, max_y);
        }

        if (error != QUIRC_SUCCESS || data->payload_len <= 0) {
            continue;
        }

        int length = data->payload_len;
        if (length >= BARCODE_MAX_LEN) {
            length = BARCODE_MAX_LEN - 1;
        }
        memcpy(result->data, data->payload, (size_t)length);
        result->data[length] = '\0';
        result->length = length;
        result->confidence = 90;
        result->symbology = BARCODE_SYM_QR;
        if (trace) {
            trace->stage = stage;
            trace->mirror = (uint8_t)(mirror != 0);
            trace->sample_scale = (uint8_t)sample_scale;
            trace->grid_size = (uint8_t)code->size;
            trace->transposed = (uint8_t)transposed;
            trace->geometry_valid = 1;
            for (int point = 0; point < 4; point++) {
                int source_point = transposed ? (4 - point) % 4 : point;
                const quirc_float_t *transform = decoder->grids[index].c;
                float grid_x = source_point == 1 || source_point == 2 ? code->size : 0;
                float grid_y = source_point >= 2 ? code->size : 0;
                float denominator = transform[6] * grid_x + transform[7] * grid_y + 1;
                if (!isfinite(denominator) || fabsf(denominator) < 0.000001f) {
                    trace->geometry_valid = 0;
                    break;
                }
                float sampled_x = (transform[0] * grid_x + transform[1] * grid_y + transform[2]) / denominator * sample_scale + sample_offset;
                float sampled_y = (transform[3] * grid_x + transform[4] * grid_y + transform[5]) / denominator * sample_scale + sample_offset;
                if (!isfinite(sampled_x) || !isfinite(sampled_y) || fabsf(sampled_x) > region_width * 2 || fabsf(sampled_y) > region_height * 2) {
                    trace->geometry_valid = 0;
                    break;
                }
                trace->corners[2 * point] = (int)lrintf(origin_x + (mirror ? region_width - 1 - sampled_x : sampled_x));
                trace->corners[2 * point + 1] = (int)lrintf(origin_y + sampled_y);
            }
        }
        xcr_record_qr_timing(trace, stage, 2,
                             xcr_port_time_us() - decode_start_us - refinement_us);
        free(data);
        free(code);
        return 1;
    }

    xcr_record_qr_timing(trace, stage, 2,
                         xcr_port_time_us() - decode_start_us - refinement_us);
    free(data);
    free(code);
    return 0;
}

static int xcr_scan_region(const Image *image,
                           BarcodeResult *result,
                           int origin_x,
                           int origin_y,
                           int region_width,
                           int region_height,
                           int sample_scale,
                           xcr_qr_candidate_box_t *boxes,
                           int *box_count,
                           int try_mirror,
                           XcrQrTrace *trace,
                           XcrQrStage stage,
                           int allow_refine)
{
    struct quirc *decoder;
    int sampled_width;
    int sampled_height;

    if (!image || !image->pixels || region_width < 21 || region_height < 21 ||
        sample_scale < 1 || origin_x < 0 || origin_y < 0 ||
        origin_x > image->width - region_width ||
        origin_y > image->height - region_height) {
        return 0;
    }

    if (xcr_qr_expired()) return 0;
    decoder = quirc_new();
    if (!decoder) {
        return 0;
    }
    decoder->should_abort = xcr_qr_abort;
    decoder->abort_context = (void *)s_qr_deadline;
    sampled_width = (region_width + sample_scale - 1) / sample_scale;
    sampled_height = (region_height + sample_scale - 1) / sample_scale;
    if (quirc_resize(decoder, sampled_width, sampled_height) < 0) {
        quirc_destroy(decoder);
        return 0;
    }

    uint64_t sample_start_us = xcr_port_time_us();
    xcr_fill_quirc_buffer(decoder, image, origin_x, origin_y,
                          region_width, region_height, sample_scale, 0,
                          stage == XCR_QR_STAGE_FULL_4X_ADAPTIVE ? (allow_refine ? 2 : 1) :
                          stage == XCR_QR_STAGE_FULL_8X || stage == XCR_QR_STAGE_FULL_4X,
                          &sampled_width, &sampled_height);
    if (stage == XCR_QR_STAGE_FULL_4X_ADAPTIVE &&
        !xcr_adaptive_threshold(decoder, allow_refine ? 20 : 12, allow_refine ? 8 : 3)) {
        quirc_destroy(decoder);
        return 0;
    }
    xcr_record_qr_timing(trace, stage, 0,
                         xcr_port_time_us() - sample_start_us);
    if (sampled_width <= 0 || sampled_height <= 0) {
        quirc_destroy(decoder);
        return 0;
    }
    if (xcr_finish_quirc(decoder, result, origin_x, origin_y,
                         region_width, region_height, sample_scale, 0,
                         boxes, box_count, trace, stage, allow_refine,
                         stage == XCR_QR_STAGE_FULL_8X || stage == XCR_QR_STAGE_FULL_4X ||
                         stage == XCR_QR_STAGE_FULL_4X_ADAPTIVE ? sample_scale / 2 : 0)) {
        quirc_destroy(decoder);
        return 1;
    }

    if (!xcr_qr_expired() && stage == XCR_QR_STAGE_FULL_4X_ADAPTIVE && allow_refine) {
        sample_start_us = xcr_port_time_us();
        xcr_fill_quirc_buffer(decoder, image, origin_x, origin_y,
                              region_width, region_height, sample_scale, 0, 1,
                              &sampled_width, &sampled_height);
        int prepared = xcr_adaptive_threshold(decoder, 12, 3);
        xcr_record_qr_timing(trace, stage, 0, xcr_port_time_us() - sample_start_us);
        if (prepared && xcr_finish_quirc(decoder, result, origin_x, origin_y,
                                         region_width, region_height, sample_scale, 0,
                                         boxes, box_count, trace, stage, allow_refine, sample_scale / 2)) {
            quirc_destroy(decoder);
            return 1;
        }
    }
    if (try_mirror && !xcr_qr_expired()) {
        sample_start_us = xcr_port_time_us();
        xcr_fill_quirc_buffer(decoder, image, origin_x, origin_y,
                              region_width, region_height, sample_scale, 1, 0,
                              &sampled_width, &sampled_height);
        xcr_record_qr_timing(trace, stage, 0, xcr_port_time_us() - sample_start_us);
        if (sampled_width > 0 && sampled_height > 0 &&
            xcr_finish_quirc(decoder, result, origin_x, origin_y,
                             region_width, region_height, sample_scale, 1,
                             boxes, box_count, trace, stage, allow_refine, 0)) {
            quirc_destroy(decoder);
            return 1;
        }
    }

    quirc_destroy(decoder);
    return 0;
}

static int xcr_scan_candidate(const Image *image,
                              BarcodeResult *result,
                              const xcr_qr_candidate_box_t *candidate,
                              int first_scale,
                              XcrQrTrace *trace,
                              int allow_refine,
                              int prefer_small)
{
    int left;
    int top;
    int right;
    int bottom;
    int width;
    int height;
    int margin_x;
    int margin_y;

    if (!candidate) {
        return 0;
    }
    width = candidate->right - candidate->left + 1;
    height = candidate->bottom - candidate->top + 1;
    int prefer_two = prefer_small && first_scale >= 4 && width <= 256 && height <= 256;
    margin_x = xcr_max_int(12, width / 8);
    margin_y = xcr_max_int(12, height / 8);
    left = xcr_max_int(0, candidate->left - margin_x);
    top = xcr_max_int(0, candidate->top - margin_y);
    right = xcr_min_int(image->width - 1, candidate->right + margin_x);
    bottom = xcr_min_int(image->height - 1, candidate->bottom + margin_y);
    width = right - left + 1;
    height = bottom - top + 1;
    if (width < 21 || height < 21) {
        return 0;
    }

    /* The 4x pass is the normal refinement. The 2x pass preserves detail
     * for small, rotated, or low-contrast codes when the first refinement
     * still leaves too few pixels per QR module. */
    if (prefer_two && xcr_scan_region(image, result, left, top, width, height, 2,
                        NULL, NULL, 1, trace, XCR_QR_STAGE_CANDIDATE_2X, allow_refine)) {
        return 1;
    }
    if (first_scale >= 4 && xcr_scan_region(image, result, left, top, width, height, 4,
                        NULL, NULL, 1, trace, XCR_QR_STAGE_CANDIDATE_4X, allow_refine)) {
        return 1;
    }
    if (prefer_two) return 0;
    return xcr_scan_region(image, result, left, top, width, height, 2,
                           NULL, NULL, 1, trace, XCR_QR_STAGE_CANDIDATE_2X, allow_refine);
}

static int xcr_quirc_decode_mode(const Image *image, BarcodeResult *result,
                                XcrQrTrace *trace, int fast_search)
{
    XcrQrTrace local_trace;
    if (!trace) {
        trace = &local_trace;
    }
    xcr_qr_candidate_box_t candidates[XCR_QR_MAX_CANDIDATES] = {0};
    int candidate_count = 0;
    if (!image || !image->pixels || !result || image->width < 21 ||
        image->height < 21) {
        return 0;
    }
    if (trace) {
        memset(trace, 0, sizeof(*trace));
    }

    /* One full-frame coarse pass covers the center and the edges while
     * keeping a single candidate list for the refinement stages. */
    if (xcr_scan_region(image, result, 0, 0, image->width, image->height, 8,
                        candidates, &candidate_count, 0, trace,
                        XCR_QR_STAGE_FULL_8X, 0)) {
        if (trace) {
            trace->candidate_count = (uint8_t)candidate_count;
        }
        return 1;
    }
    if (fast_search && candidate_count == 0) {
        unsigned coarse_incomplete = trace->incomplete_count;
        int found = xcr_scan_region(image, result, 0, 0, image->width, image->height, 4,
                                    candidates, &candidate_count, 0, trace,
                                    XCR_QR_STAGE_FULL_4X, 0);
        if (trace) {
            trace->candidate_count = (uint8_t)candidate_count;
        }
        if (!found && candidate_count == 0 && coarse_incomplete > 0 &&
            trace->incomplete_count > coarse_incomplete) {
            return 0;
        }
        if (!found && candidate_count == 0) {
            found = xcr_scan_region(image, result, 0, 0, image->width, image->height, 4,
                                    candidates, &candidate_count, 0, trace,
                                    XCR_QR_STAGE_FULL_4X_ADAPTIVE, trace->finder_count > 0);
            trace->candidate_count = (uint8_t)candidate_count;
        }
        if (found || (candidate_count == 0 && trace->finder_count < 3)) {
            return found;
        }
        if (candidate_count == 0) {
            return xcr_scan_region(image, result, 0, 0, image->width, image->height, 2,
                                    candidates, &candidate_count, 0, trace,
                                    XCR_QR_STAGE_FULL_2X, 1);
        }
    }
    if (candidate_count > 0) {
        if (trace) {
            trace->candidate_count = (uint8_t)candidate_count;
        }
        for (int index = 0; index < candidate_count; index++) {
            if (xcr_scan_candidate(image, result, &candidates[index], 4, trace, 0, fast_search)) {
                return 1;
            }
        }
        if (xcr_scan_region(image, result, 0, 0, image->width, image->height, 4,
                            NULL, NULL, 0, trace, XCR_QR_STAGE_FULL_4X, 0)) {
            if (trace) {
                trace->candidate_count = (uint8_t)candidate_count;
            }
            return 1;
        }
        candidate_count = 0;
        int coarse_found = xcr_scan_region(image, result, 0, 0, image->width, image->height, 8,
                                            candidates, &candidate_count, 0, trace,
                                            XCR_QR_STAGE_FULL_8X, 1);
        if (trace) {
            trace->candidate_count = (uint8_t)candidate_count;
        }
        if (coarse_found) {
            return 1;
        }
    }
    if (trace) {
        trace->candidate_count = (uint8_t)candidate_count;
    }

    for (int index = 0; index < candidate_count; index++) {
        if (xcr_scan_candidate(image, result, &candidates[index], 4, trace, 1, fast_search)) {
            return 1;
        }
    }
    candidate_count = 0;
    int found = xcr_scan_region(image, result, 0, 0, image->width, image->height, 4,
                                candidates, &candidate_count, 0, trace,
                                XCR_QR_STAGE_FULL_4X, 1);
    if (trace) {
        trace->candidate_count = (uint8_t)candidate_count;
    }
    if (found) {
        return 1;
    }
    for (int index = 0; index < candidate_count; index++) {
        if (xcr_scan_candidate(image, result, &candidates[index], 2, trace, 1, fast_search)) {
            return 1;
        }
    }
    candidate_count = 0;
    found = xcr_scan_region(image, result, 0, 0, image->width, image->height, 4,
                            candidates, &candidate_count, 0, trace,
                            XCR_QR_STAGE_FULL_4X_ADAPTIVE, 1);
    if (found) {
        if (trace) {
            trace->candidate_count = (uint8_t)candidate_count;
        }
        return 1;
    }
    candidate_count = 0;
    found = xcr_scan_region(image, result, 0, 0, image->width, image->height, 2,
                            candidates, &candidate_count, 0, trace, XCR_QR_STAGE_FULL_2X, 1);
    if (trace) {
        trace->candidate_count = (uint8_t)candidate_count;
    }
    return found;
}

int xcr_quirc_decode(const Image *image, BarcodeResult *result, XcrQrTrace *trace)
{
    return xcr_quirc_decode_mode(image, result, trace, 0);
}

int xcr_quirc_decode_fast(const Image *image, BarcodeResult *result, XcrQrTrace *trace)
{
    return xcr_quirc_decode_mode(image, result, trace, 1);
}

int xcr_quirc_decode_bounded(const Image *image, BarcodeResult *result,
                           XcrQrTrace *trace, const XcrDeadline *deadline, int fast_search)
{
    const XcrDeadline *previous = s_qr_deadline;
    s_qr_deadline = deadline;
    int found = xcr_quirc_decode_mode(image, result, trace, fast_search);
    if (xcr_qr_expired()) found = 0;
    s_qr_deadline = previous;
    return found;
}
