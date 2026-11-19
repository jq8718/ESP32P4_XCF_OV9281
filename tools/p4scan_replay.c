/* Test-only application selected by P4SCAN_REPLAY_BUNDLE. No camera,
 * outputs, or persisted configuration are initialized by this app. */
#include "driver/jpeg_decode.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "xcr_dmtx.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

extern const uint8_t replay_start[] asm("_binary_p4scan_replay_bin_start");
extern const uint8_t replay_end[] asm("_binary_p4scan_replay_bin_end");

enum { WIDTH = 1280, HEIGHT = 720, GRAY_BYTES = WIDTH * HEIGHT };

#if CONFIG_IDF_TARGET_ESP32P4
void xcr_dm_vertical_pie(const uint8_t *, const uint8_t *, const uint8_t *, uint8_t *, uint8_t *, int);
void xcr_dm_classify_pie(const uint8_t *, const uint8_t *, const uint8_t *, uint8_t *, int, const uint8_t *);
void xcr_dm_sample2_pie(const uint8_t *, const uint8_t *, const uint8_t *, uint8_t *, uint8_t *, uint8_t *, int);
void xcr_dm_sample_row2_pie(const uint8_t *, uint8_t *, int);

static void verify_vector_threshold(void)
{
    _Alignas(16) uint8_t rows[3][128], lo[128], hi[128], mask[96], constants[32], sampled[3][64];
    uint32_t random = 0x12345678;
    for (int test = 0; test < 32; test++) {
        for (int y = 0; y < 3; y++) {
            for (int x = 0; x < 128; x++) {
                random = random * 1664525U + 1013904223U;
                rows[y][x] = test < 16 ? (random >> 24) : 100 + (random >> 28);
            }
        }
        xcr_dm_sample2_pie(rows[0], rows[1], rows[2], sampled[0], sampled[1], sampled[2], 64);
        for (int y = 0; y < 3; y++)
            for (int x = 0; x < 64; x++)
                if (sampled[y][x] != rows[y][2*x]) abort();
        xcr_dm_sample_row2_pie(rows[1], sampled[0], 64);
        for (int x = 0; x < 64; x++)
            if (sampled[0][x] != rows[1][2*x]) abort();
        xcr_dm_vertical_pie(rows[0], rows[1], rows[2], lo, hi, 128);
        memset(constants, 8, 16);
        memset(constants + 16, test & 1 ? 6 : 0, 16);
        xcr_dm_classify_pie(lo, hi, rows[1] + 16, mask, 96, constants);
        for (int x = 16; x < 112; x++) {
            int l = 255, h = 0;
            for (int y = 0; y < 3; y++)
                for (int d = -5; d <= 5; d += 5) {
                    int v = rows[y][x + d];
                    if (v < l) l = v;
                    if (v > h) h = v;
                }
            uint8_t expected = h - l >= 9 && 2 * rows[1][x] + constants[16] < l + h ? 255 : 0;
            if (mask[x - 16] != expected) {
                printf("P4_VERIFY_FAIL,%d,%d,%d,%d\n", test, x, expected, mask[x - 16]);
                abort();
            }
        }
    }
    puts("P4_VERIFY_OK");
}
#endif

static uint32_t load_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void replay_task(void *unused)
{
    (void)unused;
    jpeg_decoder_handle_t jpeg = NULL;
    jpeg_decode_engine_cfg_t engine = {.timeout_ms = 100};
    jpeg_decode_cfg_t format = {.output_format = JPEG_DECODE_OUT_FORMAT_GRAY};
    jpeg_decode_memory_alloc_cfg_t in_cfg = {.buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER};
    jpeg_decode_memory_alloc_cfg_t out_cfg = {.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER};
    size_t input_capacity = 0, gray_capacity = 0;
    uint8_t *input = jpeg_alloc_decoder_mem(256 * 1024, &in_cfg, &input_capacity);
    uint8_t *gray = jpeg_alloc_decoder_mem(GRAY_BYTES, &out_cfg, &gray_capacity);
    ESP_ERROR_CHECK(jpeg_new_decoder_engine(&engine, &jpeg));
    ESP_ERROR_CHECK(input && gray ? ESP_OK : ESP_ERR_NO_MEM);
    size_t remaining = replay_end - replay_start;
    const uint8_t *cursor = replay_start;
    if (remaining < 8 || memcmp(cursor, "P4RP", 4)) abort();
    uint32_t count = load_u32(cursor + 4);
    cursor += 8;
    remaining -= 8;
    XcrDecoderConfig config;
    xcr_decoder_config_default(&config);
    config.enable_code128 = config.enable_upc_ean = config.enable_qr = false;
    config.enable_datamatrix = config.datamatrix_defer_geometry = true;
    config.image_budget_ms = config.decoder_timeout_ms = 100;
    XcrDecoder *decoder = xcr_decoder_create(&config);
    if (!decoder) abort();
    vTaskDelay(pdMS_TO_TICKS(1800)); /* Serial host can reopen after reset. */
#if CONFIG_IDF_TARGET_ESP32P4
    verify_vector_threshold();
#endif
    printf("P4_REPLAY_BEGIN,%" PRIu32 "\n", count);
    const uint8_t **offsets = malloc(count * sizeof(*offsets));
    if (!offsets) abort();
    for (uint32_t i = 0; i < count; i++) {
        if (remaining < 8) abort();
        uint32_t bytes = load_u32(cursor + 4);
        if (bytes > remaining - 8) abort();
        offsets[i] = cursor;
        cursor += 8 + bytes;
        remaining -= 8 + bytes;
    }
    for (int pass = 0; pass < 2; pass++) {
    printf("P4_REPLAY_MODE,%s\n", pass ? "independent_reverse" : "independent_forward");
    for (uint32_t i = 0; i < count; i++) {
        cursor = offsets[pass ? count - 1 - i : i];
        remaining = replay_end - cursor;
        if (remaining < 8) abort();
        uint32_t id = load_u32(cursor), bytes = load_u32(cursor + 4);
        cursor += 8;
        remaining -= 8;
        if (!bytes || bytes > remaining || bytes > input_capacity) abort();
        memcpy(input, cursor, bytes);
        cursor += bytes;
        remaining -= bytes;
        jpeg_decode_picture_info_t info;
        ESP_ERROR_CHECK(jpeg_decoder_get_info(input, bytes, &info));
        if (info.width != WIDTH || info.height != HEIGHT || info.sample_method != JPEG_DOWN_SAMPLING_GRAY) abort();
        uint32_t output_bytes;
        ESP_ERROR_CHECK(jpeg_decoder_process(jpeg, &format, input, bytes, gray, gray_capacity, &output_bytes));
        if (output_bytes < GRAY_BYTES) abort();
        /* Measure the actual Gray8 scanner and refinement, without JPEG IO. */
        uint64_t start = esp_timer_get_time();
        XcrGrayImage image = {.width = WIDTH, .height = HEIGHT, .stride = WIDTH,
            .pixels = gray, .sequence = i + 1, .capture_generation = 1,
            .timestamp_us = start, .deadline_us = start + 100000};
        XcrDmtxPrepared prepared = {0};
        XcrBarcodeResult result = {0};
        int results = 0;
        if (xcr_dmtx_prepare_front(&image, &prepared) && xcr_dmtx_prepare_finish(&image, &prepared))
            xcr_decoder_decode_prepared(decoder, &image, &prepared, &result, 1, &results);
        xcr_dmtx_prepare_discard(&prepared);
        if (results > 0) xcr_datamatrix_refine(&image, &result);
        uint64_t elapsed = esp_timer_get_time() - start;
        printf("P4_REPLAY,%" PRIu32 ",%d,%d,%" PRIu64 ",%.*s\n",
               id, results, result.geometry_quality, elapsed, result.length, result.data);
        XcrDmtxStats stats;
        xcr_dmtx_stats_get(&stats);
        printf("P4_PROFILE,%" PRIu32 ",%u,%d,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%d,%d,%u\n",
               id, stats.marker_sample_step, stats.marker_roi_valid,
               stats.prescreen_us, stats.setup_us, stats.decode_us,
               elapsed - prepared.front_us, stats.body_us, stats.body_threshold_us,
               (int)(10 * stats.body_candidate_px + .5f), stats.body_binarizer, stats.body_gray_attempts);
        vTaskDelay(1);
    }
    }
    puts("P4_REPLAY_END");
    free(offsets);
    xcr_decoder_destroy(decoder);
    jpeg_del_decoder_engine(jpeg);
    free(gray);
    free(input);
    vTaskDelete(NULL);
}

void app_main(void)
{
    if (xTaskCreatePinnedToCore(replay_task, "replay", 28672, NULL, 8, NULL, 1) != pdPASS) abort();
}
