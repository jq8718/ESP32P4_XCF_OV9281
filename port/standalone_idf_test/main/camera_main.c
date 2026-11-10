#include "p4scan_camera_capture.h"
#include "p4scan_board_power.h"
#include "p4scan_feedback.h"
#include "xcr_decoder.h"
#include "scan_log_buffer.h"
#include "scan_usb.h"
#include "agv_pipeline.h"

#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>

#define TAG "p4scan_live"
#define SCAN_BUTTON_DEBOUNCE_MS 10
#define SCAN_BUTTON_RELEASE_MS 5
#define SCAN_BUTTON_POLL_MS 1
#define SCAN_ROI_WIDTH P4SCAN_CAMERA_DEFAULT_WIDTH
#define SCAN_ROI_HEIGHT 128
#define SCAN_ROI_ROW_STEP 16
#define SCAN_ROI_CANDIDATE_COUNT 2
#define SCAN_CARDINAL_ANGLE_COUNT 2
#define SCAN_MAX_ROI_CANDIDATES (SCAN_ROI_CANDIDATE_COUNT * SCAN_CARDINAL_ANGLE_COUNT)
#define SCAN_TIMEOUT_MS 3000
#define SCAN_DECODER_ATTEMPT_TIMEOUT_MS 1200
#define SCAN_QR_ONLY 1
#define SCAN_ORIENTATION_STEP 64
#define SCAN_ORIENTATION_COLUMN_STEP 8
#define SCAN_ORIENTATION_MARGIN_PERCENT 2
#define SCAN_ANGLE_CANDIDATE_COUNT 4
#define SCAN_DUMP_HEADER_SIZE 4096
#define SCAN_DUMP_FRAME_OFFSET SCAN_DUMP_HEADER_SIZE
#define SCAN_DUMP_ROI_OFFSET (SCAN_DUMP_FRAME_OFFSET + (SCAN_ROI_WIDTH * P4SCAN_CAMERA_DEFAULT_HEIGHT))
#define SCAN_DUMP_REGION_SIZE (SCAN_DUMP_ROI_OFFSET + (SCAN_ROI_WIDTH * SCAN_ROI_HEIGHT) + 4096)
#define SCAN_SAMPLE_COUNT 10
#define SCAN_SAMPLE_FRAME_SIZE (P4SCAN_CAMERA_DEFAULT_WIDTH * P4SCAN_CAMERA_DEFAULT_HEIGHT)

#ifndef CONFIG_XCR_DATAMATRIX_ONLY
#define CONFIG_XCR_DATAMATRIX_ONLY 0
#endif

#if defined(CONFIG_XCR_DATAMATRIX_ONLY) && CONFIG_XCR_DATAMATRIX_ONLY
#define SCAN_DECODER_MODE_NAME "DataMatrix"
#define SCAN_DECODER_CONFIG_NAME "DataMatrix-only"
#else
#define SCAN_DECODER_MODE_NAME "QR"
#define SCAN_DECODER_CONFIG_NAME "QR-only"
#endif

#if !SCAN_QR_ONLY
static const int s_scan_orientation_slopes[] = {
    -65536, -55013, -49152, -37837, -32768, -26412,
    -22938, -17561, -11585, -5730, -4096, -2048, -1024, 0,
    1024, 2048, 4096, 5730, 11585, 17561, 22938, 26412,
    32768, 37837, 49152, 55013, 65536,
};

static const int s_scan_fallback_angles[SCAN_ANGLE_CANDIDATE_COUNT]
    __attribute__((unused)) = {
    90, 270, 45, -45,
};

static const int s_scan_cardinal_angles[SCAN_CARDINAL_ANGLE_COUNT]
    __attribute__((unused)) = {
    0, 180,
};

typedef struct {
    int angle_deg;
    int y;
    int slope_q16;
    int line_y;
    int orientation_source_y;
    bool orientation_ready;
    uint32_t score;
} scan_roi_candidate_t;

typedef struct {
    const p4scan_camera_frame_t *frame;
    scan_roi_candidate_t candidate;
    uint8_t *roi;
    XcrDecoder *decoder;
    SemaphoreHandle_t done;
    bool reverse_before_decode;
    XcrBarcodeResult result;
    int result_count;
    XcrStatus status;
} scan_parallel_worker_t;
#endif

static p4scan_camera_t s_camera;
static SemaphoreHandle_t s_scan_trigger;
static volatile bool s_scan_busy;
static volatile int s_scan_button_level = 1;
static volatile int64_t s_scan_button_down_us;

static void scan_clear_pending_triggers(void)
{
    int dropped = 0;

    while (xSemaphoreTake(s_scan_trigger, 0) == pdTRUE) {
        dropped++;
    }
    if (dropped > 0) {
        ESP_LOGI(TAG, "cleared %d button trigger(s) after scan", dropped);
    }
}

#if CONFIG_P4SCAN_DUMP_FRAME_TO_FLASH
static uint32_t s_scan_sample_frames;
static bool s_scan_sample_ready;
static bool s_scan_sample_failed;

static bool scan_sample_write(const esp_partition_t *partition,
                              uint32_t offset, const uint8_t *data, size_t size)
{
    uint8_t *chunk;

    if (!partition || !data || size == 0 || offset > partition->size ||
        size > partition->size - offset) {
        return false;
    }
    chunk = heap_caps_malloc(4096, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!chunk) {
        return false;
    }
    for (size_t written = 0; written < size; written += 4096) {
        size_t length = size - written;
        if (length > 4096) {
            length = 4096;
        }
        memcpy(chunk, data + written, length);
        if (esp_partition_write(partition, offset + written, chunk, length) != ESP_OK) {
            heap_caps_free(chunk);
            return false;
        }
    }
    heap_caps_free(chunk);
    return true;
}

static void scan_sample_capture_to_flash(const p4scan_camera_frame_t *frame)
{
    const esp_partition_t *partition;
    uint32_t offset;

    if (s_scan_sample_frames >= SCAN_SAMPLE_COUNT || s_scan_sample_failed ||
        !frame || !frame->pixels || frame->width != P4SCAN_CAMERA_DEFAULT_WIDTH ||
        frame->height != P4SCAN_CAMERA_DEFAULT_HEIGHT ||
        frame->stride != P4SCAN_CAMERA_DEFAULT_WIDTH) {
        return;
    }
    partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                         ESP_PARTITION_SUBTYPE_ANY, "samples");
    if (!partition || (size_t)SCAN_SAMPLE_COUNT * SCAN_SAMPLE_FRAME_SIZE > partition->size) {
        ESP_LOGE(TAG, "sample partition unavailable or too small");
        s_scan_sample_failed = true;
        s_scan_sample_frames = SCAN_SAMPLE_COUNT;
        return;
    }
    if (!s_scan_sample_ready) {
        if (esp_partition_erase_range(partition, 0,
                                      SCAN_SAMPLE_COUNT * SCAN_SAMPLE_FRAME_SIZE) != ESP_OK) {
            ESP_LOGE(TAG, "sample partition erase failed");
            s_scan_sample_failed = true;
            s_scan_sample_frames = SCAN_SAMPLE_COUNT;
            return;
        }
        s_scan_sample_ready = true;
    }
    offset = s_scan_sample_frames * SCAN_SAMPLE_FRAME_SIZE;
    if (!scan_sample_write(partition, offset, frame->pixels, SCAN_SAMPLE_FRAME_SIZE)) {
        ESP_LOGE(TAG, "sample frame write failed: index=%lu",
                 (unsigned long)s_scan_sample_frames);
        s_scan_sample_failed = true;
        s_scan_sample_frames = SCAN_SAMPLE_COUNT;
        return;
    }
    s_scan_sample_frames++;
    ESP_LOGW(TAG, "camera sample saved: index=%lu/%d sequence=%lu flash_offset=0x%lx",
             (unsigned long)s_scan_sample_frames, SCAN_SAMPLE_COUNT,
             (unsigned long)frame->sequence, (unsigned long)offset);
}

static uint32_t scan_sample_frames_written(void)
{
    return s_scan_sample_frames;
}
#endif

#if !SCAN_QR_ONLY
static bool scan_decode_attempt_fits_deadline(int64_t deadline)
{
    return esp_timer_get_time() < deadline;
}

#if CONFIG_P4SCAN_DUMP_FRAME_TO_FLASH
typedef struct {
    char magic[4];
    uint32_t version;
    uint32_t header_size;
    uint32_t frame_width;
    uint32_t frame_height;
    uint32_t frame_stride;
    uint32_t frame_offset;
    uint32_t frame_size;
    uint32_t roi_width;
    uint32_t roi_height;
    uint32_t roi_stride;
    uint32_t roi_offset;
    uint32_t roi_size;
    int32_t roi_y;
    int32_t roi_slope_q16;
    int32_t roi_line_y;
    uint32_t sequence;
    uint64_t timestamp_us;
} scan_dump_header_t;

static bool scan_dump_write(const esp_partition_t *partition,
                            uint32_t offset, const uint8_t *data, size_t size)
{
    uint8_t *chunk = NULL;

    if (!partition || !data || size == 0 || offset > partition->size ||
        size > partition->size - offset) {
        return false;
    }
    chunk = heap_caps_malloc(4096, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!chunk) {
        return false;
    }
    for (size_t written = 0; written < size; written += 4096) {
        size_t length = size - written;
        if (length > 4096) {
            length = 4096;
        }
        memcpy(chunk, data + written, length);
        if (esp_partition_write(partition, offset + written, chunk, length) != ESP_OK) {
            heap_caps_free(chunk);
            return false;
        }
    }
    heap_caps_free(chunk);
    return true;
}

static void scan_dump_capture_to_flash(const p4scan_camera_frame_t *frame,
                                       const scan_roi_candidate_t *candidate,
                                       const uint8_t *roi_pixels)
{
    static bool dumped;
    const esp_partition_t *partition;
    scan_dump_header_t header = {0};
    size_t frame_size;
    size_t roi_size;

    if (dumped || !frame || !frame->pixels || !candidate || !roi_pixels) {
        return;
    }
    frame_size = (size_t)frame->width * (size_t)frame->height;
    roi_size = (size_t)SCAN_ROI_WIDTH * SCAN_ROI_HEIGHT;
    partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                         ESP_PARTITION_SUBTYPE_ANY, "samples");
    if (!partition || SCAN_DUMP_REGION_SIZE > partition->size ||
        frame_size > (size_t)SCAN_DUMP_ROI_OFFSET - SCAN_DUMP_FRAME_OFFSET ||
        roi_size > partition->size - SCAN_DUMP_ROI_OFFSET) {
        ESP_LOGE(TAG, "capture dump partition unavailable or too small");
        return;
    }
    if (esp_partition_erase_range(partition, 0, SCAN_DUMP_REGION_SIZE) != ESP_OK) {
        ESP_LOGE(TAG, "capture dump erase failed");
        return;
    }
    if (!scan_dump_write(partition, SCAN_DUMP_FRAME_OFFSET, frame->pixels, frame_size) ||
        !scan_dump_write(partition, SCAN_DUMP_ROI_OFFSET, roi_pixels, roi_size)) {
        ESP_LOGE(TAG, "capture dump write failed");
        return;
    }
    memcpy(header.magic, "XCFD", sizeof(header.magic));
    header.version = 1;
    header.header_size = sizeof(header);
    header.frame_width = frame->width;
    header.frame_height = frame->height;
    header.frame_stride = frame->stride;
    header.frame_offset = SCAN_DUMP_FRAME_OFFSET;
    header.frame_size = (uint32_t)frame_size;
    header.roi_width = SCAN_ROI_WIDTH;
    header.roi_height = SCAN_ROI_HEIGHT;
    header.roi_stride = SCAN_ROI_WIDTH;
    header.roi_offset = SCAN_DUMP_ROI_OFFSET;
    header.roi_size = (uint32_t)roi_size;
    header.roi_y = candidate->y;
    header.roi_slope_q16 = candidate->slope_q16;
    header.roi_line_y = candidate->line_y;
    header.sequence = frame->sequence;
    header.timestamp_us = frame->timestamp_us;
    if (!scan_dump_write(partition, 0, (const uint8_t *)&header, sizeof(header))) {
        ESP_LOGE(TAG, "capture dump header write failed");
        return;
    }
    dumped = true;
    ESP_LOGW(TAG, "capture dump ready: partition=0x%lx header=0x%lx frame=0x%lx/%lu roi=0x%lx/%lu y=%d slope=%d",
             (unsigned long)partition->address,
             (unsigned long)sizeof(header),
             (unsigned long)header.frame_offset,
             (unsigned long)header.frame_size,
             (unsigned long)header.roi_offset,
             (unsigned long)header.roi_size,
             (int)header.roi_y, (int)header.roi_slope_q16);
}
#endif

static int scan_orientation_score(const p4scan_camera_frame_t *frame,
                                  int line_y, int slope_q16)
{
    uint32_t sum = 0;
    int samples = 0;
    int previous = -1;
    int mid_x;

    if (!frame || !frame->pixels || line_y < 0 ||
        line_y >= (int)frame->height) {
        return 0;
    }
    mid_x = ((int)frame->width - 1) / 2;
    for (uint32_t column = 4; column < frame->width;
         column += SCAN_ORIENTATION_COLUMN_STEP) {
        int y = line_y + (int)(((int)column - mid_x) *
                               (long long)slope_q16 >> 16);
        if (y < 0 || y >= (int)frame->height) {
            previous = -1;
            continue;
        }
        {
            int value = frame->pixels[(size_t)y * frame->stride + column];
            if (previous >= 0) {
                sum += (uint32_t)(value > previous ? value - previous :
                                  previous - value);
                samples++;
            }
            previous = value;
        }
    }
    if (samples < (int)frame->width / (SCAN_ORIENTATION_COLUMN_STEP * 2)) {
        return 0;
    }
    return (int)((sum * 1000U) / (uint32_t)samples);
}

static bool scan_find_orientation(const p4scan_camera_frame_t *frame,
                                  int roi_y,
                                  int *out_slope_q16,
                                  int *out_line_y)
{
    int best_slope = 0;
    int best_line_y = roi_y + SCAN_ROI_HEIGHT / 2;
    int best_score = 0;
    int horizontal_score = 0;

    if (!frame || !frame->pixels || !out_slope_q16 || !out_line_y) {
        return false;
    }

    for (size_t slope_index = 0;
         slope_index < sizeof(s_scan_orientation_slopes) / sizeof(s_scan_orientation_slopes[0]);
         slope_index++) {
        int slope_q16 = s_scan_orientation_slopes[slope_index];
        int slope_best_score = 0;
        int slope_best_line_y = best_line_y;
        int first_line_y = roi_y - SCAN_ROI_HEIGHT / 2;
        int last_line_y = roi_y + SCAN_ROI_HEIGHT + SCAN_ROI_HEIGHT / 2;

        if (first_line_y < 0) {
            first_line_y = 0;
        }
        if (last_line_y > (int)frame->height - 1) {
            last_line_y = (int)frame->height - 1;
        }
        for (int line_y = first_line_y; line_y <= last_line_y;
             line_y += SCAN_ORIENTATION_STEP) {
            int score = scan_orientation_score(frame, line_y, slope_q16);
            if (score > slope_best_score) {
                slope_best_score = score;
                slope_best_line_y = line_y;
            }
        }
        if (slope_q16 == 0) {
            horizontal_score = slope_best_score;
        }
        if (slope_best_score > best_score) {
            best_score = slope_best_score;
            best_slope = slope_q16;
            best_line_y = slope_best_line_y;
        }
    }

    if (best_slope != 0) {
        int first_line_y = best_line_y - SCAN_ORIENTATION_STEP;
        int last_line_y = best_line_y + SCAN_ORIENTATION_STEP;

        if (first_line_y < 0) {
            first_line_y = 0;
        }
        if (last_line_y >= (int)frame->height) {
            last_line_y = (int)frame->height - 1;
        }
        for (int line_y = first_line_y; line_y <= last_line_y; line_y += 8) {
            int score = scan_orientation_score(frame, line_y, best_slope);
            if (score > best_score) {
                best_score = score;
                best_line_y = line_y;
            }
        }
    }

    if (best_slope == 0 || horizontal_score <= 0 ||
        best_score < horizontal_score + horizontal_score * SCAN_ORIENTATION_MARGIN_PERCENT / 100) {
        *out_slope_q16 = 0;
        *out_line_y = best_line_y;
        return false;
    }
    *out_slope_q16 = best_slope;
    *out_line_y = best_line_y;
    return true;
}

static void scan_prepare_candidate_orientation(const p4scan_camera_frame_t *frame,
                                               scan_roi_candidate_t *candidate)
{
    int slope_q16 = 0;
    int line_y = candidate ? candidate->line_y : 0;

    if (!frame || !candidate || candidate->orientation_ready) {
        return;
    }
    scan_find_orientation(frame, candidate->orientation_source_y,
                          &slope_q16, &line_y);
    candidate->slope_q16 = slope_q16;
    candidate->line_y = line_y;
    if (candidate->angle_deg == 180) {
        candidate->line_y = (int)frame->height - 1 - line_y;
    }
    candidate->orientation_ready = true;
}

static void scan_build_oriented_roi(const p4scan_camera_frame_t *frame,
                                     const scan_roi_candidate_t *candidate,
                                     uint8_t *output)
{
    int mid_x;
    int mid_output_y = SCAN_ROI_HEIGHT / 2;

    if (!frame || !frame->pixels || !candidate || !output) {
        return;
    }
    mid_x = ((int)frame->width - 1) / 2;
    for (int output_y = 0; output_y < SCAN_ROI_HEIGHT; output_y++) {
        uint8_t *destination = output + (size_t)output_y * SCAN_ROI_WIDTH;
        int offset_y = output_y - mid_output_y;
        for (int column = 0; column < SCAN_ROI_WIDTH; column++) {
            int source_y = candidate->line_y + offset_y +
                (int)(((long long)(column - mid_x) * candidate->slope_q16) >> 16);
            if (column >= (int)frame->width || source_y < 0 ||
                source_y >= (int)frame->height) {
                destination[column] = 255;
            } else {
                destination[column] = frame->pixels[(size_t)source_y * frame->stride + column];
            }
        }
    }
}

static void scan_build_angle_oriented_roi(const p4scan_camera_frame_t *frame,
                                          const scan_roi_candidate_t *candidate,
                                          uint8_t *output)
{
    if (!frame || !candidate || !output) {
        return;
    }
    if (candidate->angle_deg != 180) {
        scan_build_oriented_roi(frame, candidate, output);
        return;
    }
    int mid_x = ((int)frame->width - 1) / 2;
    int mid_output_y = SCAN_ROI_HEIGHT / 2;
    for (int output_y = 0; output_y < SCAN_ROI_HEIGHT; output_y++) {
        uint8_t *destination = output + (size_t)output_y * SCAN_ROI_WIDTH;
        int offset_y = output_y - mid_output_y;
        for (int column = 0; column < SCAN_ROI_WIDTH; column++) {
            int transformed_y = candidate->line_y + offset_y +
                (int)(((long long)(column - mid_x) * candidate->slope_q16) >> 16);
            int source_y = (int)frame->height - 1 - transformed_y;
            int source_x = (int)frame->width - 1 - column;
            if (source_x < 0 || source_x >= (int)frame->width ||
                source_y < 0 || source_y >= (int)frame->height) {
                destination[column] = 255;
            } else {
                destination[column] = frame->pixels[(size_t)source_y * frame->stride +
                                                    (size_t)source_x];
            }
        }
    }
}

static void scan_rotate_angle(const p4scan_camera_frame_t *frame,
                              uint8_t *output, int angle_deg)
{
    int cosine_q16;
    int sine_q16;
    int center_x_q16 = (int)frame->width * 32768;
    int center_y_q16 = (int)frame->height * 32768;

    switch (angle_deg) {
    case 90:
        for (uint32_t row = 0; row < frame->height; row++) {
            for (uint32_t column = 0; column < frame->width; column++) {
                output[(size_t)row * frame->width + column] =
                    frame->pixels[(size_t)column * frame->stride +
                                  (frame->width - 1U - row)];
            }
        }
        return;
    case 270:
        for (uint32_t row = 0; row < frame->height; row++) {
            for (uint32_t column = 0; column < frame->width; column++) {
                output[(size_t)row * frame->width + column] =
                    frame->pixels[(size_t)(frame->height - 1U - column) *
                                  frame->stride + row];
            }
        }
        return;
    case 180:
        for (uint32_t row = 0; row < frame->height; row++) {
            const uint8_t *source = frame->pixels +
                (size_t)(frame->height - 1U - row) * frame->stride;
            uint8_t *destination = output + (size_t)row * frame->width;
            for (uint32_t column = 0; column < frame->width; column++) {
                destination[column] = source[frame->width - 1U - column];
            }
        }
        return;
    case -60:
        cosine_q16 = 32768;
        sine_q16 = -56756;
        break;
    case -45:
        cosine_q16 = 46341;
        sine_q16 = -46341;
        break;
    case 45:
        cosine_q16 = 46341;
        sine_q16 = 46341;
        break;
    case 60:
        cosine_q16 = 32768;
        sine_q16 = 56756;
        break;
    default:
        return;
    }

    for (uint32_t row = 0; row < frame->height; row++) {
        for (uint32_t column = 0; column < frame->width; column++) {
            int dx = (int)column - (int)(frame->width / 2);
            int dy = (int)row - (int)(frame->height / 2);
            int64_t source_x_q16 = (int64_t)cosine_q16 * dx -
                (int64_t)sine_q16 * dy + center_x_q16;
            int64_t source_y_q16 = (int64_t)sine_q16 * dx +
                (int64_t)cosine_q16 * dy + center_y_q16;
            int source_x = (int)(source_x_q16 >> 16);
            int source_y = (int)(source_y_q16 >> 16);
            uint32_t fx;
            uint32_t fy;
            uint32_t top;
            uint32_t bottom;

            if (source_x < 0 || source_x + 1 >= (int)frame->width ||
                source_y < 0 || source_y + 1 >= (int)frame->height) {
                output[(size_t)row * frame->width + column] = 255;
                continue;
            }
            fx = (uint32_t)(source_x_q16 & 0xffff);
            fy = (uint32_t)(source_y_q16 & 0xffff);
            top = ((uint32_t)frame->pixels[(size_t)source_y * frame->stride +
                                            (size_t)source_x] * (65536U - fx) +
                   (uint32_t)frame->pixels[(size_t)source_y * frame->stride +
                                            (size_t)source_x + 1] * fx) >> 16;
            bottom = ((uint32_t)frame->pixels[(size_t)(source_y + 1) *
                                               frame->stride + (size_t)source_x] *
                      (65536U - fx) +
                      (uint32_t)frame->pixels[(size_t)(source_y + 1) *
                                               frame->stride + (size_t)source_x + 1] *
                      fx) >> 16;
            output[(size_t)row * frame->width + column] =
                (uint8_t)((top * (65536U - fy) + bottom * fy) >> 16);
        }
    }
}

static bool scan_enhance_roi(const uint8_t *input, uint8_t *output,
                             size_t pixel_count)
{
    uint32_t histogram[256] = {0};
    size_t index;
    uint32_t low_target;
    uint32_t high_target;
    uint32_t cumulative;
    int low = 0;
    int high = 255;
    int range;

    if (!input || !output || pixel_count == 0) {
        return false;
    }
    for (index = 0; index < pixel_count; index++) {
        histogram[input[index]]++;
    }
    low_target = (uint32_t)(pixel_count / 100U);
    high_target = (uint32_t)(pixel_count - pixel_count / 100U);
    if (low_target == 0) {
        low_target = 1;
    }
    cumulative = 0;
    for (index = 0; index < 256; index++) {
        cumulative += histogram[index];
        if (cumulative >= low_target) {
            low = (int)index;
            break;
        }
    }
    cumulative = 0;
    for (index = 0; index < 256; index++) {
        cumulative += histogram[index];
        if (cumulative >= high_target) {
            high = (int)index;
            break;
        }
    }
    range = high - low;
    if (range < 16 || (high >= 200 && range >= 128)) {
        return false;
    }
    for (index = 0; index < pixel_count; index++) {
        int value = ((int)input[index] - low) * 255 / range;
        if (value < 0) {
            value = 0;
        } else if (value > 255) {
            value = 255;
        }
        output[index] = (uint8_t)value;
    }
    return true;
}

static void scan_enhance_2d(const uint8_t *input, uint8_t *output,
                            uint16_t *horizontal_mean, int width, int height,
                            int radius, int gain)
{
    for (int row = 0; row < height; row++) {
        const uint8_t *source = input + (size_t)row * (size_t)width;
        uint16_t *mean_row = horizontal_mean + (size_t)row * (size_t)width;
        int left = 0;
        int right = radius < width ? radius : width - 1;
        uint32_t sum = 0;
        for (int column = 0; column <= right; column++) {
            sum += source[column];
        }
        for (int column = 0; column < width; column++) {
            mean_row[column] = (uint16_t)(sum / (uint32_t)(right - left + 1));
            if (column == width - 1) {
                break;
            }
            if (column - radius >= 0) {
                sum -= source[column - radius];
            }
            if (column + radius + 1 < width) {
                sum += source[column + radius + 1];
            }
            left = column + 1 - radius;
            if (left < 0) {
                left = 0;
            }
            right = column + 1 + radius;
            if (right >= width) {
                right = width - 1;
            }
        }
    }

    for (int column = 0; column < width; column++) {
        int top = 0;
        int bottom = radius < height ? radius : height - 1;
        uint32_t sum = 0;
        for (int row = top; row <= bottom; row++) {
            sum += horizontal_mean[(size_t)row * (size_t)width +
                                   (size_t)column];
        }
        for (int row = 0; row < height; row++) {
            const uint8_t *source = input + (size_t)row * (size_t)width;
            uint8_t *destination = output + (size_t)row * (size_t)width;
            int mean = (int)(sum / (uint32_t)(bottom - top + 1));
            int value = 128 + ((int)source[column] - mean) * gain;
            if (value < 0) {
                value = 0;
            } else if (value > 255) {
                value = 255;
            }
            destination[column] = (uint8_t)value;
            if (row == height - 1) {
                break;
            }
            if (row - radius >= 0) {
                sum -= horizontal_mean[(size_t)(row - radius) *
                                       (size_t)width + (size_t)column];
            }
            if (row + radius + 1 < height) {
                sum += horizontal_mean[(size_t)(row + radius + 1) *
                                       (size_t)width + (size_t)column];
            }
            top = row + 1 - radius;
            if (top < 0) {
                top = 0;
            }
            bottom = row + 1 + radius;
            if (bottom >= height) {
                bottom = height - 1;
            }
        }
    }
}

static void scan_copy_horizontal_roi(const p4scan_camera_frame_t *frame,
                                      int roi_y, uint8_t *output)
{
    if (!frame || !frame->pixels || !output || roi_y < 0 ||
        roi_y + SCAN_ROI_HEIGHT > (int)frame->height) {
        return;
    }
    for (int row = 0; row < SCAN_ROI_HEIGHT; row++) {
        memcpy(output + (size_t)row * SCAN_ROI_WIDTH,
               frame->pixels + (size_t)(roi_y + row) * frame->stride,
               SCAN_ROI_WIDTH);
    }
}

static void scan_copy_angle_roi(const p4scan_camera_frame_t *frame,
                                const scan_roi_candidate_t *candidate,
                                uint8_t *output)
{
    if (!frame || !candidate || !output) {
        return;
    }
    if (candidate->angle_deg != 180) {
        scan_copy_horizontal_roi(frame, candidate->y, output);
        return;
    }
    for (int row = 0; row < SCAN_ROI_HEIGHT; row++) {
        int source_row = (int)frame->height - 1 - candidate->y - row;
        const uint8_t *source = frame->pixels + (size_t)source_row * frame->stride;
        uint8_t *destination = output + (size_t)row * SCAN_ROI_WIDTH;
        for (int column = 0; column < SCAN_ROI_WIDTH; column++) {
            destination[column] = source[frame->width - 1 - column];
        }
    }
}

static void scan_reverse_roi(uint8_t *pixels, size_t pixel_count)
{
    if (!pixels || pixel_count < 2) {
        return;
    }
    for (size_t left = 0, right = pixel_count - 1; left < right;
         left++, right--) {
        uint8_t value = pixels[left];
        pixels[left] = pixels[right];
        pixels[right] = value;
    }
}

static void scan_parallel_decode_worker(void *arg)
{
    scan_parallel_worker_t *worker = arg;
    XcrGrayImage image;

    memset(&image, 0, sizeof(image));
    image.width = SCAN_ROI_WIDTH;
    image.height = SCAN_ROI_HEIGHT;
    image.stride = SCAN_ROI_WIDTH;
    image.pixels = worker->roi;
    image.sequence = worker->frame->sequence;
    image.capture_generation = 1;
    image.timestamp_us = worker->frame->timestamp_us;
    worker->result_count = 0;
    worker->status = XCR_NO_RESULT;
    scan_build_angle_oriented_roi(worker->frame, &worker->candidate,
                                  worker->roi);
    if (worker->reverse_before_decode) {
        scan_reverse_roi(worker->roi,
                         (size_t)SCAN_ROI_WIDTH * SCAN_ROI_HEIGHT);
    }
    worker->status = xcr_decoder_decode(worker->decoder, &image,
                                        &worker->result, 1,
                                        &worker->result_count);
    xSemaphoreGive(worker->done);
    vTaskDelete(NULL);
}

static int scan_candidate_compare(const void *left_value, const void *right_value)
{
    const scan_roi_candidate_t *left = left_value;
    const scan_roi_candidate_t *right = right_value;

    if (left->score < right->score) {
        return 1;
    }
    if (left->score > right->score) {
        return -1;
    }
    if (left->angle_deg == 0 && right->angle_deg != 0) {
        return -1;
    }
    if (left->angle_deg != 0 && right->angle_deg == 0) {
        return 1;
    }
    return 0;
}

static int scan_select_rois(const p4scan_camera_frame_t *frame,
                            scan_roi_candidate_t *candidates,
                            int candidate_capacity)
{
    uint32_t row_scores[P4SCAN_CAMERA_DEFAULT_HEIGHT] = {0};
    int candidate_count = 0;

    if (!frame || !frame->pixels || !candidates || candidate_capacity <= 0 ||
        frame->width < 8 || frame->height < SCAN_ROI_HEIGHT ||
        frame->stride < frame->width) {
        return 0;
    }

    for (uint32_t row = 0; row < frame->height; row++) {
        const uint8_t *pixels = frame->pixels + (size_t)row * frame->stride;
        uint32_t score = 0;
        for (uint32_t column = 4; column < frame->width; column += 4) {
            uint8_t current = pixels[column];
            uint8_t previous = pixels[column - 4];
            score += current > previous ? current - previous : previous - current;
        }
        row_scores[row] = score;
    }

    for (int candidate_index = 0; candidate_index < candidate_capacity;
         candidate_index++) {
        int best_y = -1;
        uint32_t best_score = 0;
        for (int y = 0; y <= (int)frame->height - SCAN_ROI_HEIGHT;
             y += SCAN_ROI_ROW_STEP) {
            bool overlaps = false;
            uint32_t score = 0;
            for (int previous = 0; previous < candidate_count; previous++) {
                if (y < candidates[previous].y + SCAN_ROI_HEIGHT &&
                    y + SCAN_ROI_HEIGHT > candidates[previous].y) {
                    overlaps = true;
                    break;
                }
            }
            if (overlaps) {
                continue;
            }
            for (int row = y; row < y + SCAN_ROI_HEIGHT; row++) {
                score += row_scores[row];
            }
            if (best_y < 0 || score > best_score) {
                best_y = y;
                best_score = score;
            }
        }
        if (best_y < 0) {
            break;
        }
        candidates[candidate_count].y = best_y;
        candidates[candidate_count].slope_q16 = 0;
        candidates[candidate_count].line_y = best_y + SCAN_ROI_HEIGHT / 2;
        candidates[candidate_count].angle_deg = 0;
        candidates[candidate_count].orientation_source_y = best_y;
        candidates[candidate_count].orientation_ready = false;
        candidates[candidate_count].score = best_score;
        candidate_count++;
    }
    return candidate_count;
}
#endif

static void scan_button_task(void *arg)
{
    (void)arg;
    const gpio_num_t button_gpio = (gpio_num_t)CONFIG_P4SCAN_TRIGGER_GPIO;
    gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << CONFIG_P4SCAN_TRIGGER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "scan trigger GPIO%d setup failed: %s",
                 (int)button_gpio, esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    int last_raw_level = gpio_get_level(button_gpio);
    int stable_level = 1;
    int64_t changed_at = esp_timer_get_time();
    ESP_LOGI(TAG, "scan trigger button ready: GPIO%d active-low",
             (int)button_gpio);

    for (;;) {
        int raw_level = gpio_get_level(button_gpio);
        int64_t now = esp_timer_get_time();
        if (raw_level != last_raw_level) {
            last_raw_level = raw_level;
            changed_at = now;
        } else if (raw_level != stable_level &&
                   (now - changed_at) >= (raw_level == 0 ? SCAN_BUTTON_DEBOUNCE_MS : SCAN_BUTTON_RELEASE_MS) * 1000LL) {
            stable_level = raw_level;
            s_scan_button_level = stable_level;
            if (stable_level == 0) {
                int64_t button_down_us = esp_timer_get_time();
                if (s_scan_busy) {
                    ESP_LOGD(TAG, "button down ignored: scan already in progress");
                } else {
                    s_scan_button_down_us = button_down_us;
                    if (xSemaphoreGive(s_scan_trigger) != pdTRUE) {
                        s_scan_button_down_us = 0;
                        ESP_LOGD(TAG, "button down ignored: request already pending");
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SCAN_BUTTON_POLL_MS));
    }
}

static void camera_decode_task(void *arg)
{
    (void)arg;
    XcrDecoderConfig qr_decoder_config;
    XcrDecoder *qr_decoder;
    uint32_t frames = 0;
    uint32_t decoded = 0;

    xcr_decoder_config_default(&qr_decoder_config);
    qr_decoder_config.enable_code128 = false;
    qr_decoder_config.enable_upc_ean = false;
#if CONFIG_XCR_DATAMATRIX_ONLY
    qr_decoder_config.enable_qr = false;
    qr_decoder_config.enable_datamatrix = true;
#else
    qr_decoder_config.enable_qr = true;
    qr_decoder_config.enable_datamatrix = false;
#endif
    qr_decoder_config.polarity_retry = true;
    qr_decoder_config.qr_quirc_only = true;
#if CONFIG_P4SCAN_QR_FAST_SEARCH
    qr_decoder_config.qr_fast_search = true;
#endif
    qr_decoder_config.decoder_timeout_ms = SCAN_DECODER_ATTEMPT_TIMEOUT_MS;
    qr_decoder_config.image_budget_ms = SCAN_DECODER_ATTEMPT_TIMEOUT_MS;
    qr_decoder = xcr_decoder_create(&qr_decoder_config);
    if (!qr_decoder) {
        ESP_LOGW(TAG, "%s full-frame decoder unavailable",
                 SCAN_DECODER_MODE_NAME);
    }
    ESP_LOGI(TAG, "decoder configured: %s full-frame attempt_budget=%dms scan_timeout=%dms",
             SCAN_DECODER_CONFIG_NAME,
             SCAN_DECODER_ATTEMPT_TIMEOUT_MS, SCAN_TIMEOUT_MS);

    for (;;) {
        p4scan_camera_frame_t frame;
        XcrGrayImage image;
        XcrBarcodeResult result;
        int result_count = 0;
        int64_t button_down_us;
        int64_t trigger_accept_us;
        int64_t capture_begin_start;
        int64_t capture_begin_done;
        int64_t capture_start;
        int64_t capture_done;
        int64_t light_off_start;
        int64_t light_off_done;
        int64_t image_stats_start;
        int64_t image_stats_done;
        int64_t qr_start = 0;
        int64_t qr_done = 0;
        int64_t decode_start;
        int64_t decode_done;
        int64_t decode_deadline;
        int64_t release_start;
        int64_t release_done;
        uint8_t min_value = 255;
        uint8_t max_value = 0;
        uint64_t sum = 0;
        size_t sample_count = 0;
        esp_err_t ret;
        XcrStatus status = XCR_NO_RESULT;
        XcrQrTrace qr_trace;

        bool held_repeat = false;
#if !CONFIG_P4SCAN_CONTINUOUS_SCAN
        if (xSemaphoreTake(s_scan_trigger, pdMS_TO_TICKS(SCAN_BUTTON_DEBOUNCE_MS)) != pdTRUE) {
            if (s_scan_button_level != 0 ||
                gpio_get_level((gpio_num_t)CONFIG_P4SCAN_TRIGGER_GPIO) != 0) {
                continue;
            }
            held_repeat = true;
        }
#else
        vTaskDelay(1);
#endif
        trigger_accept_us = esp_timer_get_time();
        button_down_us = held_repeat ? trigger_accept_us : s_scan_button_down_us;
        s_scan_button_down_us = 0;
        if (button_down_us <= 0) {
            button_down_us = trigger_accept_us;
        }
        s_scan_busy = true;
        scan_log_begin();
        if (held_repeat) {
            ESP_LOGI(TAG, "button held: starting next scan; timing reset t=0");
        }
        ESP_LOGI(TAG, "scan trigger accepted: button_to_accept=%.3fms",
                 (double)(trigger_accept_us - button_down_us) / 1000.0);
        p4scan_feedback_set_state(P4SCAN_FEEDBACK_SCANNING);
        capture_begin_start = esp_timer_get_time();
        ret = p4scan_camera_begin_capture(&s_camera);
        capture_begin_done = esp_timer_get_time();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "capture start failed: %s", esp_err_to_name(ret));
            ESP_LOGE(TAG, "timing failed stage=capture_begin button_to_fail=%.3fms capture_begin=%.3fms",
                     (double)(capture_begin_done - button_down_us) / 1000.0,
                     (double)(capture_begin_done - capture_begin_start) / 1000.0);
            p4scan_feedback_set_state(P4SCAN_FEEDBACK_ERROR);
            scan_clear_pending_triggers();
            s_scan_busy = false;
            p4scan_camera_end_capture(&s_camera);
            scan_log_flush();
            continue;
        }

        capture_start = esp_timer_get_time();
        ret = p4scan_camera_acquire(&s_camera, &frame, 1000);
        capture_done = esp_timer_get_time();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "frame acquire failed: %s", esp_err_to_name(ret));
            light_off_start = esp_timer_get_time();
            p4scan_camera_end_capture(&s_camera);
            light_off_done = esp_timer_get_time();
            ESP_LOGE(TAG, "timing failed stage=frame_acquire button_to_fail=%.3fms capture_begin=%.3fms frame_acquire=%.3fms light_off=%.3fms",
                     (double)(light_off_done - button_down_us) / 1000.0,
                     (double)(capture_begin_done - capture_begin_start) / 1000.0,
                     (double)(capture_done - capture_start) / 1000.0,
                     (double)(light_off_done - light_off_start) / 1000.0);
            p4scan_feedback_set_state(P4SCAN_FEEDBACK_ERROR);
            scan_clear_pending_triggers();
            s_scan_busy = false;
            scan_log_flush();
            continue;
        }
        light_off_start = esp_timer_get_time();
        p4scan_camera_snapshot_exposure(&s_camera);
        ret = p4scan_camera_end_capture(&s_camera);
        light_off_done = esp_timer_get_time();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "capture light shutdown failed: %s", esp_err_to_name(ret));
        }
#if CONFIG_P4SCAN_DUMP_FRAME_TO_FLASH
        scan_sample_capture_to_flash(&frame);
#endif
        image_stats_start = esp_timer_get_time();
        {
            for (uint32_t row = 0; row < frame.height; row += 16) {
                const uint8_t *row_pixels = frame.pixels + (size_t)row * frame.stride;
                for (uint32_t column = 0; column < frame.width; column += 16) {
                    uint8_t value = row_pixels[column];
                    if (value < min_value) {
                        min_value = value;
                    }
                    if (value > max_value) {
                        max_value = value;
                    }
                    sum += value;
                    sample_count++;
                }
            }
            image_stats_done = esp_timer_get_time();
        }
        decode_start = esp_timer_get_time();
#if CONFIG_P4SCAN_CONTINUOUS_SCAN
        if (sample_count > 0) {
            p4scan_camera_adjust_search_exposure(&s_camera, (uint32_t)(sum / sample_count));
        }
#endif
        decode_deadline = decode_start + SCAN_TIMEOUT_MS * 1000LL;
        if (qr_decoder && !CONFIG_XCR_DATAMATRIX_ONLY) {
            ESP_LOGD(TAG, "qr_begin: full-frame %lux%lu",
                     (unsigned long)frame.width, (unsigned long)frame.height);
            memset(&image, 0, sizeof(image));
            image.width = (int)frame.width;
            image.height = (int)frame.height;
            image.stride = (int)frame.stride;
            image.pixels = frame.pixels;
            image.sequence = frame.sequence;
            image.capture_generation = 1;
            image.timestamp_us = frame.timestamp_us;
            memset(&result, 0, sizeof(result));
            result_count = 0;
            qr_start = esp_timer_get_time();
            status = xcr_decoder_decode(qr_decoder, &image, &result, 1,
                                        &result_count);
            qr_done = esp_timer_get_time();
            xcr_decoder_get_qr_trace(qr_decoder, &qr_trace);
        }
#if !SCAN_QR_ONLY
        if (result_count == 0) {
        roi_select_start = esp_timer_get_time();
        roi_count = 0;
        {
            scan_roi_candidate_t selected[SCAN_ROI_CANDIDATE_COUNT];
            int selected_count = scan_select_rois(&frame, selected,
                                                  SCAN_ROI_CANDIDATE_COUNT);

            /* A 180-degree transform preserves horizontal edge scores and slope. */
            for (int angle_pass = 0;
                 angle_pass < SCAN_CARDINAL_ANGLE_COUNT; angle_pass++) {
                int angle = s_scan_cardinal_angles[angle_pass];
                for (int selected_index = 0;
                     selected_index < selected_count &&
                     roi_count < SCAN_MAX_ROI_CANDIDATES;
                     selected_index++) {
                    roi_candidates[roi_count] = selected[selected_index];
                    if (angle == 180) {
                        roi_candidates[roi_count].y =
                            (int)frame.height - SCAN_ROI_HEIGHT -
                            selected[selected_index].y;
                        roi_candidates[roi_count].line_y =
                            (int)frame.height - 1 -
                            selected[selected_index].line_y;
                    }
                    roi_candidates[roi_count].angle_deg = angle;
                    roi_count++;
                }
            }
        }
        qsort(roi_candidates, (size_t)roi_count, sizeof(roi_candidates[0]),
              scan_candidate_compare);
        roi_select_done = esp_timer_get_time();
        ESP_LOGI(TAG, "timing roi_select=%.3fms candidates=%d button_to_roi=%.3fms",
                 (double)(roi_select_done - roi_select_start) / 1000.0,
                 roi_count,
                 (double)(roi_select_done - button_down_us) / 1000.0);
        if (roi_count <= 0) {
            ESP_LOGW(TAG, "no valid scan ROI for frame=%lu",
                     (unsigned long)frames + 1);
            ret = p4scan_camera_release(&s_camera, &frame);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "frame release failed: %s", esp_err_to_name(ret));
            }
            scan_clear_pending_triggers();
            s_scan_busy = false;
            p4scan_feedback_set_state(P4SCAN_FEEDBACK_FAIL);
            scan_log_flush();
            continue;
        }
        roi_decode_start = esp_timer_get_time();
        if (roi_count >= 1 && parallel_decoders[0] && parallel_decoders[1] &&
            parallel_rois[0] && parallel_rois[1] &&
            roi_candidates[0].angle_deg == 0) {
            SemaphoreHandle_t parallel_done = xSemaphoreCreateCounting(2, 0);
            scan_parallel_worker_t workers[2] = {0};
            scan_roi_candidate_t parallel_candidates[2];
            int parallel_started = 0;
            int parallel_completed = 0;

            if (parallel_done) {
                parallel_candidates[0] = roi_candidates[0];
                parallel_candidates[1] = roi_candidates[0];
                parallel_candidates[1].angle_deg = 180;
                parallel_candidates[1].y = (int)frame.height - SCAN_ROI_HEIGHT -
                                            roi_candidates[0].y;
                scan_prepare_candidate_orientation(&frame, &parallel_candidates[0]);
                parallel_candidates[1].slope_q16 =
                    parallel_candidates[0].slope_q16;
                parallel_candidates[1].line_y = (int)frame.height - 1 -
                                                parallel_candidates[0].line_y;
                parallel_candidates[1].orientation_ready = true;
                for (int worker_index = 0; worker_index < 2; worker_index++) {
                    workers[worker_index].frame = &frame;
                    workers[worker_index].candidate = parallel_candidates[worker_index];
                    workers[worker_index].roi = parallel_rois[worker_index];
                    workers[worker_index].decoder = parallel_decoders[worker_index];
                    workers[worker_index].done = parallel_done;
                    workers[worker_index].reverse_before_decode = false;
                    BaseType_t task_ret = xTaskCreatePinnedToCore(
                        scan_parallel_decode_worker, worker_index == 0 ?
                        "p4scan_dec0" : "p4scan_dec180", 8192,
                        &workers[worker_index], 8, NULL,
                        worker_index == 0 ? 1 : 0);
                    if (task_ret == pdPASS) {
                        parallel_started++;
                    } else {
                        ESP_LOGW(TAG, "parallel worker %d creation failed",
                                 worker_index);
                    }
                }
                while (parallel_completed < parallel_started) {
                    xSemaphoreTake(parallel_done, portMAX_DELAY);
                    parallel_completed++;
                    for (int worker_index = 0; worker_index < parallel_started;
                         worker_index++) {
                        if (workers[worker_index].result_count > 0 &&
                            result_count == 0) {
                            result = workers[worker_index].result;
                            result_count = workers[worker_index].result_count;
                            status = workers[worker_index].status;
                            decoded_roi_y = workers[worker_index].candidate.y;
                            for (int other_index = 0;
                                 other_index < parallel_started; other_index++) {
                                if (other_index != worker_index) {
                                    xcr_decoder_cancel(workers[other_index].decoder);
                                }
                            }
                        }
                    }
                }
                for (int worker_index = 0; worker_index < parallel_started;
                     worker_index++) {
                    ESP_LOGI(TAG, "parallel ROI direction=%s status=%s results=%d",
                             worker_index == 0 ? "angle0" : "angle180",
                             xcr_status_name(workers[worker_index].status),
                             workers[worker_index].result_count);
                }
                if (parallel_started == 2 && result_count > 0) {
                    roi_begin = 1;
                }
                vSemaphoreDelete(parallel_done);
            }
        }
        for (int roi_index = 0;
             roi_index < roi_count && result_count == 0;
             roi_index++) {
            if (roi_index < roi_begin) {
                continue;
            }
            memset(&image, 0, sizeof(image));
            image.width = SCAN_ROI_WIDTH;
            image.height = SCAN_ROI_HEIGHT;
            image.stride = SCAN_ROI_WIDTH;
            image.pixels = oriented_roi;
            image.roi_x = 0;
            image.roi_y = 0;
            image.roi_width = 0;
            image.roi_height = 0;
            image.sequence = frame.sequence;
            image.capture_generation = 1;
             image.timestamp_us = frame.timestamp_us;
            if (!oriented_roi) {
                ESP_LOGW(TAG, "scan ROI buffer unavailable");
                break;
            }
            scan_prepare_candidate_orientation(&frame, &roi_candidates[roi_index]);
            if (prepared_angle != roi_candidates[roi_index].angle_deg) {
                candidate_frame = frame;
                if (roi_candidates[roi_index].angle_deg != 0 &&
                    roi_candidates[roi_index].angle_deg != 180) {
                    scan_rotate_angle(&frame, rotated_frame,
                                      roi_candidates[roi_index].angle_deg);
                    candidate_frame.pixels = rotated_frame;
                    candidate_frame.stride = candidate_frame.width;
                }
                prepared_angle = roi_candidates[roi_index].angle_deg;
                ESP_LOGI(TAG, "scan orientation prepared angle=%d",
                         prepared_angle);
            }
            if (roi_candidates[roi_index].slope_q16 != 0) {
                scan_build_angle_oriented_roi(&candidate_frame,
                                              &roi_candidates[roi_index], oriented_roi);
            } else {
                scan_copy_angle_roi(&candidate_frame,
                                    &roi_candidates[roi_index], oriented_roi);
            }
#if CONFIG_P4SCAN_DUMP_FRAME_TO_FLASH
            if (roi_index == 0) {
                scan_dump_capture_to_flash(&candidate_frame,
                                            &roi_candidates[roi_index], oriented_roi);
                decode_start = esp_timer_get_time();
                decode_deadline = decode_start + SCAN_TIMEOUT_MS * 1000LL;
            }
#endif
            memset(&result, 0, sizeof(result));
            result_count = 0;
            ESP_LOGI(TAG, "scan ROI attempt=%d/%d angle=%d y=%d slope_q16=%d edge_score=%lu variant=%s",
                     roi_index + 1, roi_count, roi_candidates[roi_index].angle_deg,
                     roi_candidates[roi_index].y,
                     roi_candidates[roi_index].slope_q16,
                     (unsigned long)roi_candidates[roi_index].score,
                     roi_candidates[roi_index].slope_q16 != 0 ? "oriented" : "raw");
            if (!scan_decode_attempt_fits_deadline(decode_deadline)) {
                status = XCR_TIMEOUT_NO_RESULT;
                break;
            }
            status = xcr_decoder_decode(decoder, &image, &result, 1, &result_count);
            if (result_count == 0 && oriented_roi &&
                scan_decode_attempt_fits_deadline(decode_deadline)) {
                scan_reverse_roi(oriented_roi,
                                 (size_t)SCAN_ROI_WIDTH * SCAN_ROI_HEIGHT);
                image.pixels = oriented_roi;
                ESP_LOGI(TAG, "scan reversed ROI attempt=%d/%d angle=%d variant=%s",
                         roi_index + 1, roi_count,
                         roi_candidates[roi_index].angle_deg,
                         roi_candidates[roi_index].slope_q16 != 0 ?
                         "oriented" : "raw");
                memset(&result, 0, sizeof(result));
                result_count = 0;
                status = xcr_decoder_decode(decoder, &image, &result, 1,
                                            &result_count);
            }
            if (result_count == 0 && roi_candidates[roi_index].slope_q16 != 0 &&
                oriented_roi && scan_decode_attempt_fits_deadline(decode_deadline)) {
                scan_copy_angle_roi(&candidate_frame,
                                    &roi_candidates[roi_index], oriented_roi);
                image.pixels = oriented_roi;
                ESP_LOGI(TAG, "scan horizontal fallback angle=%d y=%d",
                         roi_candidates[roi_index].angle_deg,
                         roi_candidates[roi_index].y);
                memset(&result, 0, sizeof(result));
                result_count = 0;
                status = xcr_decoder_decode(decoder, &image, &result, 1,
                                            &result_count);
            }
            if (result_count == 0 && enhanced_roi &&
                scan_decode_attempt_fits_deadline(decode_deadline)) {
                /* Rebuild the corrected ROI because the reverse and horizontal
                 * fallback attempts reuse the same working buffer. */
                if (roi_candidates[roi_index].slope_q16 != 0) {
                    scan_build_angle_oriented_roi(&candidate_frame,
                                                  &roi_candidates[roi_index],
                                                  oriented_roi);
                } else {
                    scan_copy_angle_roi(&candidate_frame,
                                        &roi_candidates[roi_index],
                                        oriented_roi);
                }
                if (scan_enhance_roi(oriented_roi, enhanced_roi,
                                     (size_t)SCAN_ROI_WIDTH * SCAN_ROI_HEIGHT)) {
                    image.pixels = enhanced_roi;
                    ESP_LOGI(TAG, "scan ROI variant=global-enhanced angle=%d y=%d slope_q16=%d",
                             roi_candidates[roi_index].angle_deg,
                             roi_candidates[roi_index].y,
                             roi_candidates[roi_index].slope_q16);
                    memset(&result, 0, sizeof(result));
                    result_count = 0;
                    status = xcr_decoder_decode(decoder, &image, &result, 1,
                                                &result_count);
                }
            }
            if (result_count == 0 && enhanced_2d_roi && horizontal_mean &&
                roi_candidates[roi_index].slope_q16 == 0 &&
                scan_decode_attempt_fits_deadline(decode_deadline)) {
                scan_enhance_2d(oriented_roi, enhanced_2d_roi, horizontal_mean,
                                SCAN_ROI_WIDTH, SCAN_ROI_HEIGHT, 16, 4);
                image.pixels = enhanced_2d_roi;
                ESP_LOGI(TAG, "scan ROI variant=local-2d y=%d",
                         roi_candidates[roi_index].y);
                memset(&result, 0, sizeof(result));
                result_count = 0;
                status = xcr_decoder_decode(decoder, &image, &result, 1,
                                            &result_count);
            }
            if (result_count > 0) {
                decoded_roi_y = roi_candidates[roi_index].y;
                break;
            }
        }

        if (result_count == 0 && rotated_frame && oriented_roi &&
            scan_decode_attempt_fits_deadline(decode_deadline)) {
                    p4scan_camera_frame_t rotated = frame;

            rotated.pixels = rotated_frame;
            rotated.stride = rotated.width;
            for (int angle_index = 0;
                 angle_index < SCAN_ANGLE_CANDIDATE_COUNT && result_count == 0 &&
                 scan_decode_attempt_fits_deadline(decode_deadline);
                 angle_index++) {
                scan_rotate_angle(&frame, rotated_frame,
                                  s_scan_fallback_angles[angle_index]);
                fallback_roi_count = scan_select_rois(
                    &rotated, fallback_candidates, SCAN_ROI_CANDIDATE_COUNT);
                ESP_LOGI(TAG, "scan rotated fallback angle=%d candidates=%d",
                         s_scan_fallback_angles[angle_index], fallback_roi_count);
                for (int roi_index = 0;
                     roi_index < fallback_roi_count && result_count == 0;
                     roi_index++) {
                    if (!scan_decode_attempt_fits_deadline(decode_deadline)) {
                        break;
                    }
                    image.pixels = oriented_roi;
                    memset(&result, 0, sizeof(result));
                    result_count = 0;
                    scan_prepare_candidate_orientation(&rotated,
                                                       &fallback_candidates[roi_index]);
                    if (fallback_candidates[roi_index].slope_q16 != 0) {
                        scan_build_oriented_roi(&rotated,
                                                &fallback_candidates[roi_index],
                                                oriented_roi);
                        ESP_LOGI(TAG,
                                 "scan rotated oriented ROI attempt=%d/%d angle=%d y=%d slope_q16=%d line_y=%d",
                                 roi_index + 1, fallback_roi_count,
                                 s_scan_fallback_angles[angle_index],
                                 fallback_candidates[roi_index].y,
                                 fallback_candidates[roi_index].slope_q16,
                                 fallback_candidates[roi_index].line_y);
                    } else {
                        scan_copy_horizontal_roi(&rotated,
                                                 fallback_candidates[roi_index].y,
                                                 oriented_roi);
                        ESP_LOGI(TAG,
                                 "scan rotated ROI attempt=%d/%d angle=%d y=%d variant=raw",
                                 roi_index + 1, fallback_roi_count,
                                 s_scan_fallback_angles[angle_index],
                                 fallback_candidates[roi_index].y);
                    }
                    if (!scan_decode_attempt_fits_deadline(decode_deadline)) {
                        status = XCR_TIMEOUT_NO_RESULT;
                        break;
                    }
                    status = xcr_decoder_decode(decoder, &image, &result, 1,
                                                &result_count);
                }
            }
        }
        roi_decode_done = esp_timer_get_time();
        ESP_LOGI(TAG, "timing roi_and_fallback_decode=%.3fms",
                 (double)(roi_decode_done - roi_decode_start) / 1000.0);
        }
#endif
        if (result_count == 0 && esp_timer_get_time() >= decode_deadline) {
            status = XCR_TIMEOUT_NO_RESULT;
            ESP_LOGW(TAG, "scan decode timeout after %dms", SCAN_TIMEOUT_MS);
        }
        decode_done = esp_timer_get_time();
        if (result_count > 0 &&
            (result.symbology == XCR_SYM_QR || result.symbology == XCR_SYM_DATAMATRIX)) {
            int64_t usb_started = esp_timer_get_time();
            bool queued = scan_usb_submit(&result);
            ESP_LOGI(TAG, "usb_result queued=%d capture_us=%llu corners=%d,%d %d,%d %d,%d %d,%d enqueue_us=%lld",
                     queued, (unsigned long long)result.timestamp_us,
                     result.corners[0], result.corners[1], result.corners[2], result.corners[3],
                     result.corners[4], result.corners[5], result.corners[6], result.corners[7],
                     (long long)(esp_timer_get_time() - usb_started));
        }
        scan_usb_preview(&frame, result_count > 0 &&
                         (result.symbology == XCR_SYM_QR ||
                          result.symbology == XCR_SYM_DATAMATRIX) ? &result : NULL,
                         decode_done - decode_start);
        release_start = esp_timer_get_time();
        ret = p4scan_camera_release(&s_camera, &frame);
        release_done = esp_timer_get_time();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "frame release failed: %s", esp_err_to_name(ret));
        }
        frames++;
        if (result_count > 0) {
            decoded++;
            p4scan_camera_remember_exposure(&s_camera);
        }
#if CONFIG_P4SCAN_CONTINUOUS_SCAN
        p4scan_feedback_set_state(result_count > 0 ? P4SCAN_FEEDBACK_SUCCESS : P4SCAN_FEEDBACK_SCANNING);
#else
        p4scan_feedback_set_state(result_count > 0 ? P4SCAN_FEEDBACK_SUCCESS : P4SCAN_FEEDBACK_FAIL);
#endif
        int64_t feedback_queued_us = esp_timer_get_time();
        int64_t report_start_us = esp_timer_get_time();
        scan_log_end();
        ESP_LOGI(TAG,
                 "frame=%lu capture=%.3fms decode=%.3fms release=%.3fms button_total=%.3fms status=%s results=%d",
                 (unsigned long)frames,
                 (double)(capture_done - capture_start) / 1000.0,
                 (double)(decode_done - decode_start) / 1000.0,
                 (double)(release_done - release_start) / 1000.0,
                 (double)(release_done - button_down_us) / 1000.0,
                 xcr_status_name(status), result_count);
        if (result_count > 0) {
            ESP_LOGI(TAG, "decoded symbology=%d length=%d payload=%s",
                     result.symbology, result.length, result.data);
        }
        scan_log_flush();
        ESP_LOGI(TAG, "timing capture_begin=%.3fms frame_acquire=%.3fms light_off=%.3fms button_to_frame=%.3fms",
                 (double)(capture_begin_done - capture_begin_start) / 1000.0,
                 (double)(capture_done - capture_start) / 1000.0,
                 (double)(light_off_done - light_off_start) / 1000.0,
                 (double)(capture_done - button_down_us) / 1000.0);
        ESP_LOGI(TAG, "frame stats seq=%lu min=%u max=%u mean=%.2f samples=%lu",
                 (unsigned long)frame.sequence, (unsigned)min_value, (unsigned)max_value,
                 sample_count ? (double)sum / (double)sample_count : 0.0,
                 (unsigned long)sample_count);
        ESP_LOGI(TAG, "timing image_stats=%.3fms button_to_image_stats=%.3fms",
                 (double)(image_stats_done - image_stats_start) / 1000.0,
                 (double)(image_stats_done - button_down_us) / 1000.0);
        if (qr_decoder && !CONFIG_XCR_DATAMATRIX_ONLY) {
            ESP_LOGI(TAG, "timing qr_full_frame=%.3fms result_count=%d",
                     (double)(qr_done - qr_start) / 1000.0, result_count);
            ESP_LOGI(TAG, "qr_geometry refined=%u time_us=%lu", qr_trace.geometry_refined,
                     (unsigned long)qr_trace.geometry_refine_us);
            ESP_LOGI(TAG, "qr_path=%s mirror=%u candidates=%u finders=%u incomplete=%u",
                     xcr_qr_stage_name(qr_trace.stage),
                     (unsigned)qr_trace.mirror, (unsigned)qr_trace.candidate_count,
                     (unsigned)qr_trace.finder_count, (unsigned)qr_trace.incomplete_count);
            ESP_LOGI(TAG, "qr_parts_us: full8_sample=%lu full8_identify=%lu full8_decode=%lu candidate_sample=%lu candidate_identify=%lu candidate_decode=%lu",
                     (unsigned long)qr_trace.full8_sample_us,
                     (unsigned long)qr_trace.full8_identify_us,
                     (unsigned long)qr_trace.full8_decode_us,
                     (unsigned long)qr_trace.candidate_sample_us,
                     (unsigned long)qr_trace.candidate_identify_us,
                     (unsigned long)qr_trace.candidate_decode_us);
            ESP_LOGI(TAG, "qr_refine_us: full8=%lu candidate=%lu fallback=%lu",
                     (unsigned long)qr_trace.full8_refine_us,
                     (unsigned long)qr_trace.candidate_refine_us,
                     (unsigned long)qr_trace.fallback_refine_us);
            if (qr_trace.fallback_sample_us || qr_trace.fallback_identify_us) {
                ESP_LOGI(TAG, "qr_fallback_us: sample=%lu identify=%lu decode=%lu",
                         (unsigned long)qr_trace.fallback_sample_us,
                         (unsigned long)qr_trace.fallback_identify_us,
                         (unsigned long)qr_trace.fallback_decode_us);
            }
        } else if (qr_decoder) {
            ESP_LOGI(TAG, "timing datamatrix_full_frame=%.3fms result_count=%d",
                     (double)(qr_done - qr_start) / 1000.0, result_count);
        }
        int64_t report_done_us = esp_timer_get_time();
        ESP_LOGI(TAG, "timing button_to_feedback_queued=%.3fms report_log=%.3fms button_to_report_done=%.3fms",
                 (double)(feedback_queued_us - button_down_us) / 1000.0,
                 (double)(report_done_us - report_start_us) / 1000.0,
                 (double)(report_done_us - button_down_us) / 1000.0);
        scan_clear_pending_triggers();
        s_scan_busy = false;
#if CONFIG_P4SCAN_DUMP_FRAME_TO_FLASH
        if (scan_sample_frames_written() < SCAN_SAMPLE_COUNT) {
            ESP_ERROR_CHECK(xSemaphoreGive(s_scan_trigger) == pdTRUE ? ESP_OK : ESP_FAIL);
        }
#endif
        if ((frames % 20) == 0) {
            ESP_LOGI(TAG, "live summary: frames=%lu decoded=%lu free8=%u free_psram=%u",
                     (unsigned long)frames, (unsigned long)decoded,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        }
    }
    xcr_decoder_destroy(qr_decoder);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(scan_log_init());
    p4scan_camera_config_t config;
    p4scan_camera_config_default(&config);
    ESP_LOGI(TAG, "starting OV9281 live XCF pipeline");
    ESP_ERROR_CHECK(p4scan_feedback_init());
    ESP_LOGI(TAG, "init step: feedback ready");
    ESP_ERROR_CHECK(scan_usb_init());
    ESP_LOGI(TAG, "init step: USB ready");
    ESP_LOGI(TAG, "init step: camera init begin");
    ESP_ERROR_CHECK(p4scan_camera_init(&s_camera, &config));
    ESP_LOGI(TAG, "init step: camera init done");
    ESP_ERROR_CHECK(p4scan_camera_start(&s_camera));
    ESP_LOGI(TAG, "init step: camera stream started");
    esp_err_t first_frame = p4scan_camera_wait_for_frame(&s_camera, 3000);
    if (first_frame != ESP_OK) {
        p4scan_ov9281_log_sensor_state(s_camera.i2c_bus);
        ESP_LOGE(TAG, "OV9281 CSI has no frame: %s; check MIPI lane wiring, XCLK and reset",
                 esp_err_to_name(first_frame));
        p4scan_feedback_set_state(P4SCAN_FEEDBACK_ERROR);
        return;
    }
    ESP_LOGI(TAG, "init step: first frame ready");
#if CONFIG_P4SCAN_LIGHT_DIAGNOSTIC && !CONFIG_P4SCAN_REALTIME_PIPELINE
    ESP_ERROR_CHECK(p4scan_ov9281_run_light_diagnostic(s_camera.i2c_bus));
#endif
#if CONFIG_P4SCAN_REALTIME_PIPELINE
    ESP_ERROR_CHECK(agv_pipeline_start(&s_camera));
    return;
#endif
    s_scan_trigger = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(s_scan_trigger ? ESP_OK : ESP_ERR_NO_MEM);
    BaseType_t button_task_ret = xTaskCreatePinnedToCore(scan_button_task, "p4scan_button",
                                                         2048, NULL, 9, NULL, 0);
    ESP_ERROR_CHECK(button_task_ret == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    BaseType_t task_ret = xTaskCreatePinnedToCore(camera_decode_task, "p4scan_decode",
                                                  20480, NULL, 8, NULL, 1);
    ESP_ERROR_CHECK(task_ret == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
#if CONFIG_P4SCAN_SOFTWARE_TRIGGER_ON_BOOT
    ESP_LOGW(TAG, "software diagnostic trigger: issuing one scan request");
    ESP_ERROR_CHECK(xSemaphoreGive(s_scan_trigger) == pdTRUE ? ESP_OK : ESP_FAIL);
#endif
}
