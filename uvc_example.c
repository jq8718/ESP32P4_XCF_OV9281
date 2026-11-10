/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"
#include "example_video_common.h"
#include "quirc.h"

#if CONFIG_FORMAT_MJPEG_CAM1
#define ENCODE_DEV_PATH     ESP_VIDEO_JPEG_DEVICE_NAME
#define UVC_OUTPUT_FORMAT   V4L2_PIX_FMT_JPEG
#elif CONFIG_FORMAT_H264_CAM1
#if CONFIG_EXAMPLE_H264_MAX_QP <= CONFIG_EXAMPLE_H264_MIN_QP
#error "CONFIG_EXAMPLE_H264_MAX_QP should larger than CONFIG_EXAMPLE_H264_MIN_QP"
#endif

#define ENCODE_DEV_PATH     ESP_VIDEO_H264_DEVICE_NAME
#define UVC_OUTPUT_FORMAT   V4L2_PIX_FMT_H264
#endif

#define BUFFER_COUNT        2
#define QR_SCAN_LOG_INTERVAL 5
#define QR_ADAPTIVE_WINDOW 21
#define QR_ADAPTIVE_C 4
#define QR_OVERLAY_MAX_LEN 128
#define QR_SCAN_BUFFER_COUNT 1
#define QR_SCAN_TASK_STACK  (16 * 1024)
#define CAMERA_EXPOSURE_US  2500

typedef struct {
    uint8_t *frame;
    int64_t captured_at_us;
} qr_scan_item_t;

typedef struct uvc {
    int cap_fd;
    uint32_t format;
    uint8_t *cap_buffer[BUFFER_COUNT];

    int m2m_fd;
    uint8_t *m2m_cap_buffer;

    int width;
    int height;
    int capture_stride;
    uint32_t capture_format;
    uint32_t qr_scan_count;
    struct quirc *qr_decoder;
    struct quirc_data *qr_data;
    uint8_t *qr_scan_buffer[QR_SCAN_BUFFER_COUNT];
    QueueHandle_t qr_free_queue;
    QueueHandle_t qr_ready_queue;
    TaskHandle_t qr_task;
    SemaphoreHandle_t qr_text_mutex;
    char qr_text[QR_OVERLAY_MAX_LEN + 1];
    int qr_overlay_width;
    uint32_t qr_scan_fps_tenths;
    uint32_t qr_frame_to_detect_tenths_ms;
    uint32_t qr_detect_to_decode_tenths_ms;
    bool qr_detect_latency_valid;
    bool qr_decode_latency_valid;

    uvc_fb_t fb;
} uvc_t;

static const char *TAG = "example";

static void qr_scanner_stop(uvc_t *uvc)
{
    if (uvc->qr_task) {
        qr_scan_item_t stop = {0};
        if (xQueueSend(uvc->qr_ready_queue, &stop, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to stop QR scanner task");
            return;
        }
        while (uvc->qr_task) {
            vTaskDelay(1);
        }
    }

    if (uvc->qr_decoder) {
        quirc_destroy(uvc->qr_decoder);
        uvc->qr_decoder = NULL;
    }
    free(uvc->qr_data);
    uvc->qr_data = NULL;
    for (int i = 0; i < QR_SCAN_BUFFER_COUNT; i++) {
        free(uvc->qr_scan_buffer[i]);
        uvc->qr_scan_buffer[i] = NULL;
    }
    if (uvc->qr_free_queue) {
        vQueueDelete(uvc->qr_free_queue);
        uvc->qr_free_queue = NULL;
    }
    if (uvc->qr_ready_queue) {
        vQueueDelete(uvc->qr_ready_queue);
        uvc->qr_ready_queue = NULL;
    }
}

static void qr_adaptive_threshold(const uint8_t *image, uint8_t *output, int width, int height)
{
    const int radius = QR_ADAPTIVE_WINDOW / 2;
    const uint32_t window_area = QR_ADAPTIVE_WINDOW * QR_ADAPTIVE_WINDOW;
    uint16_t vertical_sum[width];

    for (int x = 0; x < width; x++) {
        uint16_t sum = 0;
        for (int offset = -radius; offset <= radius; offset++) {
            int y = MAX(0, MIN(offset, height - 1));
            sum += image[y * width + x];
        }
        vertical_sum[x] = sum;
    }

    for (int y = 0; y < height; y++) {
        uint32_t horizontal_sum = vertical_sum[0] * (radius + 1);
        for (int offset = 1; offset <= radius; offset++) {
            horizontal_sum += vertical_sum[offset];
        }

        for (int x = 0; x < width; x++) {
            int threshold = horizontal_sum / window_area - QR_ADAPTIVE_C;
            output[y * width + x] = image[y * width + x] > threshold ? 255 : 0;
            int left = MAX(0, x - radius);
            int right = MIN(x + radius + 1, width - 1);
            horizontal_sum -= vertical_sum[left];
            horizontal_sum += vertical_sum[right];
        }

        if (y + 1 < height) {
            int remove_y = MAX(0, y - radius);
            int add_y = MIN(y + radius + 1, height - 1);
            for (int x = 0; x < width; x++) {
                vertical_sum[x] += image[add_y * width + x] - image[remove_y * width + x];
            }
        }
    }
}

static uint32_t qr_elapsed_tenths_ms(int64_t start_us, int64_t end_us)
{
    if (end_us <= start_us) {
        return 0;
    }

    uint64_t elapsed = ((uint64_t)(end_us - start_us) + 50) / 100;
    return elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
}

static void qr_scanner_task(void *arg)
{
    uvc_t *uvc = arg;
    qr_scan_item_t item;
    int64_t previous_scan_completed_us = 0;

    while (xQueueReceive(uvc->qr_ready_queue, &item, portMAX_DELAY) == pdTRUE) {
        if (!item.frame) {
            uvc->qr_task = NULL;
            vTaskDelete(NULL);
            return;
        }

        uint8_t *image = quirc_begin(uvc->qr_decoder, NULL, NULL);
        qr_adaptive_threshold(item.frame, image, uvc->width, uvc->height);
        quirc_end(uvc->qr_decoder);

        const int count = quirc_count(uvc->qr_decoder);
        const int64_t detected_at_us = count > 0 ? esp_timer_get_time() : 0;
        int decode_errors = 0;
        quirc_decode_error_t last_decode_error = QUIRC_SUCCESS;
        bool decoded = false;
        uint32_t frame_to_detect_tenths_ms = 0;
        uint32_t detect_to_decode_tenths_ms = 0;
        char decoded_text[QR_OVERLAY_MAX_LEN + 1] = {0};
        if (count > 0 && detected_at_us >= item.captured_at_us) {
            frame_to_detect_tenths_ms = qr_elapsed_tenths_ms(item.captured_at_us, detected_at_us);
        }
        for (int i = 0; i < count; i++) {
            struct quirc_code code;
            quirc_extract(uvc->qr_decoder, i, &code);
            quirc_decode_error_t decode_result = quirc_decode(&code, uvc->qr_data);
            if (decode_result != QUIRC_SUCCESS) {
                decode_errors++;
                last_decode_error = decode_result;
                continue;
            }
            if (uvc->qr_data->payload_len <= 0) {
                continue;
            }

            size_t len = MIN((size_t)uvc->qr_data->payload_len, sizeof(uvc->qr_text) - 1);
            memcpy(decoded_text, uvc->qr_data->payload, len);
            detect_to_decode_tenths_ms = qr_elapsed_tenths_ms(detected_at_us, esp_timer_get_time());
            decoded = true;
            break;
        }

        const int64_t scan_completed_us = esp_timer_get_time();
        xSemaphoreTake(uvc->qr_text_mutex, portMAX_DELAY);
        if (previous_scan_completed_us > 0 && scan_completed_us > previous_scan_completed_us) {
            uint64_t fps_tenths = (10000000ULL + (scan_completed_us - previous_scan_completed_us) / 2) /
                                  (scan_completed_us - previous_scan_completed_us);
            uvc->qr_scan_fps_tenths = fps_tenths > UINT32_MAX ? UINT32_MAX : (uint32_t)fps_tenths;
        }
        if (count > 0) {
            uvc->qr_frame_to_detect_tenths_ms = frame_to_detect_tenths_ms;
            uvc->qr_detect_latency_valid = true;
        }
        bool text_changed = false;
        if (decoded) {
            uvc->qr_detect_to_decode_tenths_ms = detect_to_decode_tenths_ms;
            uvc->qr_decode_latency_valid = true;
            text_changed = strcmp(uvc->qr_text, decoded_text) != 0;
            if (text_changed) {
                memcpy(uvc->qr_text, decoded_text, strlen(decoded_text) + 1);
            }
        }
        xSemaphoreGive(uvc->qr_text_mutex);
        if (decoded && text_changed) {
            ESP_LOGI(TAG, "QR decoded: %s", decoded_text);
        }
        previous_scan_completed_us = scan_completed_us;

        if (++uvc->qr_scan_count % QR_SCAN_LOG_INTERVAL == 0 && !decoded) {
            ESP_LOGI(TAG, "QR scan: candidates=%d, decode_errors=%d, last_error=%s",
                     count, decode_errors, decode_errors ? quirc_strerror(last_decode_error) : "none");
        }
        xQueueSend(uvc->qr_free_queue, &item.frame, portMAX_DELAY);
        vTaskDelay(1);
    }
}

static esp_err_t qr_scanner_start(uvc_t *uvc, int width, int height)
{
    const int scan_width = width;
    const int scan_height = height;
    const size_t image_size = (size_t)scan_width * scan_height;
    uvc->qr_scan_count = 0;
    uvc->qr_scan_fps_tenths = 0;
    uvc->qr_frame_to_detect_tenths_ms = 0;
    uvc->qr_detect_to_decode_tenths_ms = 0;
    uvc->qr_detect_latency_valid = false;
    uvc->qr_decode_latency_valid = false;

    uvc->qr_decoder = quirc_new();
    uvc->qr_data = malloc(sizeof(*uvc->qr_data));
    uvc->qr_free_queue = xQueueCreate(QR_SCAN_BUFFER_COUNT, sizeof(uint8_t *));
    uvc->qr_ready_queue = xQueueCreate(1, sizeof(qr_scan_item_t));
    if (!uvc->qr_decoder || !uvc->qr_data || !uvc->qr_free_queue || !uvc->qr_ready_queue) {
        ESP_LOGE(TAG, "Failed to allocate QR scanner resources");
        qr_scanner_stop(uvc);
        return ESP_ERR_NO_MEM;
    }
    if (quirc_resize(uvc->qr_decoder, scan_width, scan_height) < 0) {
        ESP_LOGE(TAG, "Failed to allocate QR decoder image buffer");
        qr_scanner_stop(uvc);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "QR scanner ready: full %dx%d frame, no crop or resize", width, height);

    for (int i = 0; i < QR_SCAN_BUFFER_COUNT; i++) {
        uvc->qr_scan_buffer[i] = malloc(image_size);
        if (!uvc->qr_scan_buffer[i] ||
            xQueueSend(uvc->qr_free_queue, &uvc->qr_scan_buffer[i], 0) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to allocate QR scan frame buffer");
            qr_scanner_stop(uvc);
            return ESP_ERR_NO_MEM;
        }
    }

    if (xTaskCreatePinnedToCore(qr_scanner_task, "qr_scanner", QR_SCAN_TASK_STACK, uvc, 1,
                                &uvc->qr_task, 1) != pdPASS) {
        uvc->qr_task = NULL;
        ESP_LOGE(TAG, "Failed to create QR scanner task");
        qr_scanner_stop(uvc);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void qr_scan_submit(uvc_t *uvc, const uint8_t *frame, int64_t captured_at_us)
{
    uint8_t *scan_buffer;
    if (xQueueReceive(uvc->qr_free_queue, &scan_buffer, 0) != pdTRUE) {
        return;
    }

    const int scan_width = uvc->width;
    const int scan_height = uvc->height;
    for (int y = 0; y < scan_height; y++) {
        const uint8_t *src = frame + y * uvc->capture_stride;
        uint8_t *dst = scan_buffer + y * scan_width;
        if (uvc->capture_format == V4L2_PIX_FMT_GREY) {
            memcpy(dst, src, scan_width);
        } else {
            for (int x = 0; x < scan_width; x++) {
                uint16_t pixel = src[x * 2] | (uint16_t)src[x * 2 + 1] << 8;
                uint8_t green = (pixel >> 5) & 0x3f;
                dst[x] = (green << 2) | (green >> 4);
            }
        }
    }

    qr_scan_item_t item = {
        .frame = scan_buffer,
        .captured_at_us = captured_at_us,
    };
    if (xQueueSend(uvc->qr_ready_queue, &item, 0) != pdTRUE) {
        xQueueSend(uvc->qr_free_queue, &scan_buffer, 0);
    }
}

static void glyph_columns(char c, uint8_t glyph[5])
{
    memset(glyph, 0, 5);
#define GLYPH(ch, a, b, c, d, e) case ch: glyph[0] = a; glyph[1] = b; glyph[2] = c; glyph[3] = d; glyph[4] = e; break
    switch (c) {
        GLYPH('0', 0x3e, 0x51, 0x49, 0x45, 0x3e);
        GLYPH('1', 0x00, 0x42, 0x7f, 0x40, 0x00);
        GLYPH('2', 0x42, 0x61, 0x51, 0x49, 0x46);
        GLYPH('3', 0x21, 0x41, 0x45, 0x4b, 0x31);
        GLYPH('4', 0x18, 0x14, 0x12, 0x7f, 0x10);
        GLYPH('5', 0x27, 0x45, 0x45, 0x45, 0x39);
        GLYPH('6', 0x3c, 0x4a, 0x49, 0x49, 0x30);
        GLYPH('7', 0x01, 0x71, 0x09, 0x05, 0x03);
        GLYPH('8', 0x36, 0x49, 0x49, 0x49, 0x36);
        GLYPH('9', 0x06, 0x49, 0x49, 0x29, 0x1e);
        GLYPH('A', 0x7e, 0x11, 0x11, 0x11, 0x7e);
        GLYPH('B', 0x7f, 0x49, 0x49, 0x49, 0x36);
        GLYPH('C', 0x3e, 0x41, 0x41, 0x41, 0x22);
        GLYPH('D', 0x7f, 0x41, 0x41, 0x22, 0x1c);
        GLYPH('E', 0x7f, 0x49, 0x49, 0x49, 0x41);
        GLYPH('F', 0x7f, 0x09, 0x09, 0x09, 0x01);
        GLYPH('G', 0x3e, 0x41, 0x49, 0x49, 0x7a);
        GLYPH('H', 0x7f, 0x08, 0x08, 0x08, 0x7f);
        GLYPH('I', 0x00, 0x41, 0x7f, 0x41, 0x00);
        GLYPH('J', 0x20, 0x40, 0x41, 0x3f, 0x01);
        GLYPH('K', 0x7f, 0x08, 0x14, 0x22, 0x41);
        GLYPH('L', 0x7f, 0x40, 0x40, 0x40, 0x40);
        GLYPH('M', 0x7f, 0x02, 0x0c, 0x02, 0x7f);
        GLYPH('N', 0x7f, 0x04, 0x08, 0x10, 0x7f);
        GLYPH('O', 0x3e, 0x41, 0x41, 0x41, 0x3e);
        GLYPH('P', 0x7f, 0x09, 0x09, 0x09, 0x06);
        GLYPH('Q', 0x3e, 0x41, 0x51, 0x21, 0x5e);
        GLYPH('R', 0x7f, 0x09, 0x19, 0x29, 0x46);
        GLYPH('S', 0x46, 0x49, 0x49, 0x49, 0x31);
        GLYPH('T', 0x01, 0x01, 0x7f, 0x01, 0x01);
        GLYPH('U', 0x3f, 0x40, 0x40, 0x40, 0x3f);
        GLYPH('V', 0x1f, 0x20, 0x40, 0x20, 0x1f);
        GLYPH('W', 0x3f, 0x40, 0x38, 0x40, 0x3f);
        GLYPH('X', 0x63, 0x14, 0x08, 0x14, 0x63);
        GLYPH('Y', 0x07, 0x08, 0x70, 0x08, 0x07);
        GLYPH('Z', 0x61, 0x51, 0x49, 0x45, 0x43);
        GLYPH('a', 0x20, 0x54, 0x54, 0x54, 0x78);
        GLYPH('b', 0x7f, 0x48, 0x44, 0x44, 0x38);
        GLYPH('c', 0x38, 0x44, 0x44, 0x44, 0x20);
        GLYPH('d', 0x38, 0x44, 0x44, 0x48, 0x7f);
        GLYPH('e', 0x38, 0x54, 0x54, 0x54, 0x18);
        GLYPH('f', 0x08, 0x7e, 0x09, 0x01, 0x02);
        GLYPH('g', 0x0c, 0x52, 0x52, 0x52, 0x3e);
        GLYPH('h', 0x7f, 0x08, 0x04, 0x04, 0x78);
        GLYPH('i', 0x00, 0x44, 0x7d, 0x40, 0x00);
        GLYPH('j', 0x20, 0x40, 0x44, 0x3d, 0x00);
        GLYPH('k', 0x7f, 0x10, 0x28, 0x44, 0x00);
        GLYPH('l', 0x00, 0x41, 0x7f, 0x40, 0x00);
        GLYPH('m', 0x7c, 0x04, 0x18, 0x04, 0x78);
        GLYPH('n', 0x7c, 0x08, 0x04, 0x04, 0x78);
        GLYPH('o', 0x38, 0x44, 0x44, 0x44, 0x38);
        GLYPH('p', 0x7c, 0x14, 0x14, 0x14, 0x08);
        GLYPH('q', 0x08, 0x14, 0x14, 0x18, 0x7c);
        GLYPH('r', 0x7c, 0x08, 0x04, 0x04, 0x08);
        GLYPH('s', 0x48, 0x54, 0x54, 0x54, 0x20);
        GLYPH('t', 0x04, 0x3f, 0x44, 0x40, 0x20);
        GLYPH('u', 0x3c, 0x40, 0x40, 0x20, 0x7c);
        GLYPH('v', 0x1c, 0x20, 0x40, 0x20, 0x1c);
        GLYPH('w', 0x3c, 0x40, 0x30, 0x40, 0x3c);
        GLYPH('x', 0x44, 0x28, 0x10, 0x28, 0x44);
        GLYPH('y', 0x0c, 0x50, 0x50, 0x50, 0x3c);
        GLYPH('z', 0x44, 0x64, 0x54, 0x4c, 0x44);
        GLYPH('.', 0x00, 0x60, 0x60, 0x00, 0x00);
        GLYPH(',', 0x00, 0x50, 0x30, 0x00, 0x00);
        GLYPH(':', 0x00, 0x36, 0x36, 0x00, 0x00);
        GLYPH(';', 0x00, 0x56, 0x36, 0x00, 0x00);
        GLYPH('-', 0x08, 0x08, 0x08, 0x08, 0x08);
        GLYPH('_', 0x40, 0x40, 0x40, 0x40, 0x40);
        GLYPH('/', 0x20, 0x10, 0x08, 0x04, 0x02);
        GLYPH('\\', 0x02, 0x04, 0x08, 0x10, 0x20);
        GLYPH('?', 0x02, 0x01, 0x51, 0x09, 0x06);
        GLYPH('=', 0x14, 0x14, 0x14, 0x14, 0x14);
        GLYPH('+', 0x08, 0x08, 0x3e, 0x08, 0x08);
        GLYPH('&', 0x36, 0x49, 0x55, 0x22, 0x50);
        GLYPH('%', 0x23, 0x13, 0x08, 0x64, 0x62);
        GLYPH('#', 0x14, 0x7f, 0x14, 0x7f, 0x14);
        GLYPH('@', 0x32, 0x49, 0x79, 0x41, 0x3e);
        default:
            break;
    }
#undef GLYPH
}

static void draw_overlay_pixel(uint8_t *frame, int x, int y, int stride, uint32_t format, bool white)
{
    const size_t offset = (size_t)y * stride + x * (format == V4L2_PIX_FMT_RGB565 ? 2 : 1);
    if (format == V4L2_PIX_FMT_RGB565) {
        frame[offset] = white ? 0xff : 0x00;
        frame[offset + 1] = white ? 0xff : 0x00;
    } else {
        frame[offset] = white ? 0xff : 0x00;
    }
}

static void draw_qr_text(uint8_t *frame, int width, int height, int stride, uint32_t format,
                         const char *text, int *previous_width)
{
    const int scale = 2;
    const int margin = 8;
    const int max_chars = (width - 2 * margin) / (6 * scale);
    size_t len = strlen(text);
    if (!len || max_chars <= 0) {
        return;
    }
    const bool truncated = len > (size_t)max_chars;
    if (truncated) {
        len = max_chars;
    }

    const int text_width = (int)len * 6 * scale;
    const int text_height = 7 * scale;
    const int clear_width = MAX(text_width, *previous_width);
    const int left = width - clear_width - margin;
    const int text_left = width - text_width - margin;
    const int top = height - text_height - margin;
    for (int y = top - 4; y < height; y++) {
        for (int x = left - 4; x < width - margin + 4; x++) {
            draw_overlay_pixel(frame, x, y, stride, format, false);
        }
    }
    *previous_width = text_width;

    for (size_t i = 0; i < len; i++) {
        uint8_t glyph[5];
        char c = truncated && i >= len - MIN((size_t)len, 3) ? '.' : text[i];
        if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7e) {
            c = '?';
        }
        glyph_columns(c, glyph);
        for (int x = 0; x < 5; x++) {
            for (int y = 0; y < 7; y++) {
                if (!(glyph[x] & (1U << y))) {
                    continue;
                }
                for (int dy = 0; dy < scale; dy++) {
                    for (int dx = 0; dx < scale; dx++) {
                        draw_overlay_pixel(frame, text_left + (int)i * 6 * scale + x * scale + dx,
                                           top + y * scale + dy, stride, format, true);
                    }
                }
            }
        }
    }
}

static void draw_status_text(uint8_t *frame, int width, int height, int stride, uint32_t format,
                             const char *text, int left, int top, int box_width)
{
    const int scale = 2;
    const int text_height = 7 * scale;
    const int max_chars = (box_width - 8) / (6 * scale);
    size_t len = MIN(strlen(text), (size_t)max_chars);
    if (left < 4 || top < 4 || left + box_width > width || top + text_height + 4 > height) {
        return;
    }

    for (int y = top - 4; y < top + text_height + 4; y++) {
        for (int x = left; x < left + box_width; x++) {
            draw_overlay_pixel(frame, x, y, stride, format, false);
        }
    }

    for (size_t i = 0; i < len; i++) {
        uint8_t glyph[5];
        char c = text[i];
        if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7e) {
            c = '?';
        }
        glyph_columns(c, glyph);
        for (int x = 0; x < 5; x++) {
            for (int y = 0; y < 7; y++) {
                if (!(glyph[x] & (1U << y))) {
                    continue;
                }
                for (int dy = 0; dy < scale; dy++) {
                    for (int dx = 0; dx < scale; dx++) {
                        draw_overlay_pixel(frame, left + 4 + (int)i * 6 * scale + x * scale + dx,
                                           top + y * scale + dy, stride, format, true);
                    }
                }
            }
        }
    }
}

static void print_video_device_info(const struct v4l2_capability *capability)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", (uint16_t)(capability->version >> 16),
             (uint8_t)(capability->version >> 8),
             (uint8_t)capability->version);
    ESP_LOGI(TAG, "driver:  %s", capability->driver);
    ESP_LOGI(TAG, "card:    %s", capability->card);
    ESP_LOGI(TAG, "bus:     %s", capability->bus_info);
    ESP_LOGI(TAG, "capabilities:");
    if (capability->capabilities & V4L2_CAP_VIDEO_CAPTURE) {
        ESP_LOGI(TAG, "\tVIDEO_CAPTURE");
    }
    if (capability->capabilities & V4L2_CAP_READWRITE) {
        ESP_LOGI(TAG, "\tREADWRITE");
    }
    if (capability->capabilities & V4L2_CAP_ASYNCIO) {
        ESP_LOGI(TAG, "\tASYNCIO");
    }
    if (capability->capabilities & V4L2_CAP_STREAMING) {
        ESP_LOGI(TAG, "\tSTREAMING");
    }
    if (capability->capabilities & V4L2_CAP_META_OUTPUT) {
        ESP_LOGI(TAG, "\tMETA_OUTPUT");
    }
    if (capability->capabilities & V4L2_CAP_DEVICE_CAPS) {
        ESP_LOGI(TAG, "device capabilities:");
        if (capability->device_caps & V4L2_CAP_VIDEO_CAPTURE) {
            ESP_LOGI(TAG, "\tVIDEO_CAPTURE");
        }
        if (capability->device_caps & V4L2_CAP_READWRITE) {
            ESP_LOGI(TAG, "\tREADWRITE");
        }
        if (capability->device_caps & V4L2_CAP_ASYNCIO) {
            ESP_LOGI(TAG, "\tASYNCIO");
        }
        if (capability->device_caps & V4L2_CAP_STREAMING) {
            ESP_LOGI(TAG, "\tSTREAMING");
        }
        if (capability->device_caps & V4L2_CAP_META_OUTPUT) {
            ESP_LOGI(TAG, "\tMETA_OUTPUT");
        }
    }
}

static esp_err_t init_capture_video(uvc_t *uvc)
{
    int fd;
    struct v4l2_capability capability;

    fd = open(EXAMPLE_CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open camera device %s: %s", EXAMPLE_CAM_DEV_PATH, strerror(errno));
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(ioctl(fd, VIDIOC_QUERYCAP, &capability));
    print_video_device_info(&capability);

    uvc->cap_fd = fd;

    return 0;
}

static esp_err_t init_codec_video(uvc_t *uvc)
{
    int fd;
    const char *devpath = ENCODE_DEV_PATH;
    struct v4l2_capability capability;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control control[1];

    fd = open(devpath, O_RDONLY);
    assert(fd >= 0);

    ESP_ERROR_CHECK(ioctl(fd, VIDIOC_QUERYCAP, &capability));
    print_video_device_info(&capability);

#if CONFIG_FORMAT_MJPEG_CAM1
    controls.ctrl_class = V4L2_CID_JPEG_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_JPEG_COMPRESSION_QUALITY;
    control[0].value    = CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set JPEG compression quality");
    }
#elif CONFIG_FORMAT_H264_CAM1
    controls.ctrl_class = V4L2_CID_CODEC_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD;
    control[0].value    = CONFIG_EXAMPLE_H264_I_PERIOD;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 intra frame period");
    }

    controls.ctrl_class = V4L2_CID_CODEC_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_MPEG_VIDEO_BITRATE;
    control[0].value    = CONFIG_EXAMPLE_H264_BITRATE;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 bitrate");
    }

    controls.ctrl_class = V4L2_CID_CODEC_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_MPEG_VIDEO_H264_MIN_QP;
    control[0].value    = CONFIG_EXAMPLE_H264_MIN_QP;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 minimum quality");
    }

    controls.ctrl_class = V4L2_CID_CODEC_CLASS;
    controls.count      = 1;
    controls.controls   = control;
    control[0].id       = V4L2_CID_MPEG_VIDEO_H264_MAX_QP;
    control[0].value    = CONFIG_EXAMPLE_H264_MAX_QP;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGW(TAG, "failed to set H.264 maximum quality");
    }
#endif

    uvc->format = UVC_OUTPUT_FORMAT;
    uvc->m2m_fd = fd;

    return 0;
}

static esp_err_t video_start_cb(uvc_format_t uvc_format, int width, int height, int rate, void *cb_ctx)
{
    int type;
    struct v4l2_buffer buf;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    uvc_t *uvc = (uvc_t *)cb_ctx;
    uint32_t capture_fmt = 0;

    ESP_LOGD(TAG, "UVC start");

    if (uvc->format == V4L2_PIX_FMT_JPEG) {
        int fmt_index = 0;
        const uint32_t jpeg_input_formats[] = {
            V4L2_PIX_FMT_GREY,
            V4L2_PIX_FMT_RGB565,
            V4L2_PIX_FMT_UYVY,
            V4L2_PIX_FMT_RGB24
        };
        int jpeg_input_formats_num = sizeof(jpeg_input_formats) / sizeof(jpeg_input_formats[0]);

        while (!capture_fmt) {
            struct v4l2_fmtdesc fmtdesc = {
                .index = fmt_index++,
                .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            };

            if (ioctl(uvc->cap_fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
                break;
            }

            for (int i = 0; i < jpeg_input_formats_num; i++) {
                if (jpeg_input_formats[i] == fmtdesc.pixelformat) {
                    capture_fmt = jpeg_input_formats[i];
                    break;
                }
            }
        }

        if (!capture_fmt) {
            ESP_LOGI(TAG, "The camera sensor output pixel format is not supported by JPEG");
            return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        capture_fmt = V4L2_PIX_FMT_YUV420;
    }

    /* Configure camera interface capture stream */

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_S_FMT, &format));
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_G_FMT, &format));

    struct v4l2_ext_control exposure_control = {
        .id = V4L2_CID_EXPOSURE_ABSOLUTE,
        .value = (CAMERA_EXPOSURE_US + 50) / 100,
    };
    struct v4l2_ext_controls exposure_controls = {
        .ctrl_class = V4L2_CTRL_CLASS_CAMERA,
        .count = 1,
        .controls = &exposure_control,
    };
    if (ioctl(uvc->cap_fd, VIDIOC_S_EXT_CTRLS, &exposure_controls) != 0) {
        ESP_LOGE(TAG, "Failed to set camera exposure to %d us: %s",
                 CAMERA_EXPOSURE_US, strerror(errno));
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Camera exposure set to %d us", CAMERA_EXPOSURE_US);

    uvc->width = width;
    uvc->height = height;
    uvc->capture_format = capture_fmt;
    if (format.fmt.pix.bytesperline) {
        uvc->capture_stride = format.fmt.pix.bytesperline;
    } else {
        switch (capture_fmt) {
        case V4L2_PIX_FMT_RGB565:
        case V4L2_PIX_FMT_UYVY:
            uvc->capture_stride = width * 2;
            break;
        case V4L2_PIX_FMT_RGB24:
            uvc->capture_stride = width * 3;
            break;
        default:
            uvc->capture_stride = width;
            break;
        }
    }
    ESP_LOGI(TAG, "UVC stream start: %dx%d, capture format 0x%08" PRIx32 ", stride %d",
             width, height, capture_fmt, uvc->capture_stride);
    if (capture_fmt == V4L2_PIX_FMT_GREY || capture_fmt == V4L2_PIX_FMT_RGB565) {
        ESP_ERROR_CHECK(qr_scanner_start(uvc, width, height));
    } else {
        ESP_LOGW(TAG, "QR scanning and overlay are unsupported for capture format 0x%08" PRIx32, capture_fmt);
    }

    memset(&req, 0, sizeof(req));
    req.count  = BUFFER_COUNT;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_REQBUFS, &req));

    for (int i = 0; i < BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_ERROR_CHECK (ioctl(uvc->cap_fd, VIDIOC_QUERYBUF, &buf));

        uvc->cap_buffer[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                             MAP_SHARED, uvc->cap_fd, buf.m.offset);
        assert(uvc->cap_buffer[i]);

        ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &buf));
    }

    /* Configure codec output stream */

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = capture_fmt;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = 1;
    req.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_USERPTR;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_REQBUFS, &req));

    /* Configure codec capture stream */

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = uvc->format;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_S_FMT, &format));

    memset(&req, 0, sizeof(req));
    req.count  = 1;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_REQBUFS, &req));

    memset(&buf, 0, sizeof(buf));
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = 0;
    ESP_ERROR_CHECK (ioctl(uvc->m2m_fd, VIDIOC_QUERYBUF, &buf));

    uvc->m2m_cap_buffer = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                          MAP_SHARED, uvc->m2m_fd, buf.m.offset);
    assert(uvc->m2m_cap_buffer);

    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &buf));

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_STREAMON, &type));
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_STREAMON, &type));

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_STREAMON, &type));

    return ESP_OK;
}

static void video_stop_cb(void *cb_ctx)
{
    int type;
    uvc_t *uvc = (uvc_t *)cb_ctx;

    ESP_LOGD(TAG, "UVC stop");

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(uvc->cap_fd, VIDIOC_STREAMOFF, &type);

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ioctl(uvc->m2m_fd, VIDIOC_STREAMOFF, &type);
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(uvc->m2m_fd, VIDIOC_STREAMOFF, &type);
    qr_scanner_stop(uvc);
}

static uvc_fb_t *video_fb_get_cb(void *cb_ctx)
{
    int64_t us;
    uvc_t *uvc = (uvc_t *)cb_ctx;
    struct v4l2_format format;
    struct v4l2_buffer cap_buf;
    struct v4l2_buffer m2m_out_buf;
    struct v4l2_buffer m2m_cap_buf;

    ESP_LOGD(TAG, "UVC get");

    memset(&cap_buf, 0, sizeof(cap_buf));
    cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_DQBUF, &cap_buf));

    uint8_t *capture_frame = uvc->cap_buffer[cap_buf.index];
    const int64_t captured_at_us = esp_timer_get_time();
    if (uvc->qr_task) {
        qr_scan_submit(uvc, capture_frame, captured_at_us);
    }
    char qr_text[sizeof(uvc->qr_text)];
    uint32_t scan_fps_tenths;
    uint32_t frame_to_detect_tenths_ms;
    uint32_t detect_to_decode_tenths_ms;
    bool detect_latency_valid;
    bool decode_latency_valid;
    xSemaphoreTake(uvc->qr_text_mutex, portMAX_DELAY);
    memcpy(qr_text, uvc->qr_text, sizeof(qr_text));
    scan_fps_tenths = uvc->qr_scan_fps_tenths;
    frame_to_detect_tenths_ms = uvc->qr_frame_to_detect_tenths_ms;
    detect_to_decode_tenths_ms = uvc->qr_detect_to_decode_tenths_ms;
    detect_latency_valid = uvc->qr_detect_latency_valid;
    decode_latency_valid = uvc->qr_decode_latency_valid;
    xSemaphoreGive(uvc->qr_text_mutex);
    if (uvc->capture_format == V4L2_PIX_FMT_GREY || uvc->capture_format == V4L2_PIX_FMT_RGB565) {
        char scan_fps_text[32];
        char frame_to_detect_text[40];
        char detect_to_decode_text[40];
        snprintf(scan_fps_text, sizeof(scan_fps_text), "QR FPS: %" PRIu32 ".%" PRIu32,
                 scan_fps_tenths / 10, scan_fps_tenths % 10);
        if (detect_latency_valid) {
            snprintf(frame_to_detect_text, sizeof(frame_to_detect_text),
                     "FRAME->QR: %" PRIu32 ".%" PRIu32 " ms",
                     frame_to_detect_tenths_ms / 10, frame_to_detect_tenths_ms % 10);
        } else {
            snprintf(frame_to_detect_text, sizeof(frame_to_detect_text), "FRAME->QR: --.- ms");
        }
        if (decode_latency_valid) {
            snprintf(detect_to_decode_text, sizeof(detect_to_decode_text),
                     "QR->TEXT: %" PRIu32 ".%" PRIu32 " ms",
                     detect_to_decode_tenths_ms / 10, detect_to_decode_tenths_ms % 10);
        } else {
            snprintf(detect_to_decode_text, sizeof(detect_to_decode_text), "QR->TEXT: --.- ms");
        }
        draw_status_text(capture_frame, uvc->width, uvc->height, uvc->capture_stride,
                         uvc->capture_format, scan_fps_text, 8, 8, 180);
        draw_status_text(capture_frame, uvc->width, uvc->height, uvc->capture_stride,
                         uvc->capture_format, frame_to_detect_text, uvc->width - 328, 8, 320);
        draw_status_text(capture_frame, uvc->width, uvc->height, uvc->capture_stride,
                         uvc->capture_format, detect_to_decode_text, uvc->width - 328, 28, 320);
        draw_qr_text(capture_frame, uvc->width, uvc->height, uvc->capture_stride, uvc->capture_format, qr_text,
                     &uvc->qr_overlay_width);
    }

    memset(&m2m_out_buf, 0, sizeof(m2m_out_buf));
    m2m_out_buf.index  = 0;
    m2m_out_buf.type   = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    m2m_out_buf.memory = V4L2_MEMORY_USERPTR;
    m2m_out_buf.m.userptr = (unsigned long)capture_frame;
    m2m_out_buf.length = cap_buf.bytesused;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &m2m_out_buf));

    memset(&m2m_cap_buf, 0, sizeof(m2m_cap_buf));
    m2m_cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_DQBUF, &m2m_cap_buf));

    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_DQBUF, &m2m_out_buf));
    ESP_ERROR_CHECK(ioctl(uvc->cap_fd, VIDIOC_QBUF, &cap_buf));

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_G_FMT, &format));

    uvc->fb.buf = uvc->m2m_cap_buffer;
    uvc->fb.len = m2m_cap_buf.bytesused;
    uvc->fb.width = format.fmt.pix.width;
    uvc->fb.height = format.fmt.pix.height;
    uvc->fb.format = format.fmt.pix.pixelformat == V4L2_PIX_FMT_JPEG ? UVC_FORMAT_JPEG : UVC_FORMAT_H264;

    us = esp_timer_get_time();
    uvc->fb.timestamp.tv_sec = us / 1000000UL;;
    uvc->fb.timestamp.tv_usec = us % 1000000UL;

    return &uvc->fb;
}

static void video_fb_return_cb(uvc_fb_t *fb, void *cb_ctx)
{
    struct v4l2_buffer m2m_cap_buf;
    uvc_t *uvc = (uvc_t *)cb_ctx;

    ESP_LOGD(TAG, "UVC return");

    m2m_cap_buf.index  = 0;
    m2m_cap_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m2m_cap_buf.memory = V4L2_MEMORY_MMAP;
    ESP_ERROR_CHECK(ioctl(uvc->m2m_fd, VIDIOC_QBUF, &m2m_cap_buf));
}

static esp_err_t init_uvc(uvc_t *uvc)
{
    int index = 0;
    uvc->qr_text_mutex = xSemaphoreCreateMutex();
    if (!uvc->qr_text_mutex) {
        ESP_LOGE(TAG, "Failed to create QR text mutex");
        return ESP_ERR_NO_MEM;
    }
    memcpy(uvc->qr_text, "NOT RECOGNIZED", sizeof("NOT RECOGNIZED"));

    uvc_device_config_t config = {
        .start_cb     = video_start_cb,
        .fb_get_cb    = video_fb_get_cb,
        .fb_return_cb = video_fb_return_cb,
        .stop_cb      = video_stop_cb,
        .cb_ctx       = (void *)uvc,
    };

    config.uvc_buffer_size = UVC_FRAMES_INFO[index][0].width * UVC_FRAMES_INFO[index][0].height;
    config.uvc_buffer = malloc(config.uvc_buffer_size);
    assert(config.uvc_buffer);

    ESP_LOGI(TAG, "Format List");
    ESP_LOGI(TAG, "\tFormat(1) = %s", uvc->format == V4L2_PIX_FMT_JPEG ? "MJPEG" : "H.264");
    ESP_LOGI(TAG, "Frame List");
    ESP_LOGI(TAG, "\tFrame(1) = %d * %d @%dfps", UVC_FRAMES_INFO[index][0].width, UVC_FRAMES_INFO[index][0].height, UVC_FRAMES_INFO[index][0].rate);

    ESP_ERROR_CHECK(uvc_device_config(index, &config));
    ESP_ERROR_CHECK(uvc_device_init());

    return ESP_OK;
}

void app_main(void)
{
    uvc_t *uvc = calloc(1, sizeof(uvc_t));
    assert(uvc);

    gpio_config_t camera_power_config = {
        .pin_bit_mask = 1ULL << 0,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&camera_power_config));
    ESP_ERROR_CHECK(gpio_set_level(0, 1));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(example_video_init());
    if (init_capture_video(uvc) != ESP_OK) {
        ESP_LOGE(TAG, "Camera capture is unavailable; check the sensor connection and SCCB address");
        free(uvc);
        return;
    }
    ESP_ERROR_CHECK(init_codec_video(uvc));
    ESP_ERROR_CHECK(init_uvc(uvc));
}
