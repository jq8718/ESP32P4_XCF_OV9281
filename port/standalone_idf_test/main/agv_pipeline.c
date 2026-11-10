#include "agv_pipeline.h"
#include "latest_frame.h"
#include "scan_usb.h"
#include "datamatrix.h"
#include "xcr_dmtx.h"
#include "p4scan_feedback.h"
#include "p4scan_board_power.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#if CONFIG_P4SCAN_REALTIME_PIPELINE
#define FRAME_WIDTH P4SCAN_CAMERA_DEFAULT_WIDTH
#define FRAME_HEIGHT P4SCAN_CAMERA_DEFAULT_HEIGHT
#define TAG "p4scan_agv"

typedef struct {
    const uint8_t *pixels;
    p4scan_camera_frame_t frame;
    uint64_t begin_us;
    uint32_t prepare_us;
    uint32_t acquire_us;
    uint32_t light_us;
    uint32_t capture_period_us;
    uint32_t return_us;
    uint32_t return_wait_us;
    uint64_t released_us;
    XcrBarcodeResult result;
    XcrQrTrace trace;
    XcrDmtxStats marker_stats;
    DataMatrixDecodeDebugStats dm_stats;
    uint64_t decode_start_us;
    uint64_t core_done_us;
    int result_count;
    XcrStatus status;
} agv_slot_t;

static agv_slot_t s_slots[3];
static latest_frame_pool_t s_pool;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_consumer;
static XcrDecoder *s_decoder;
static uint64_t s_light_ready_us;

static void light_button_task(void *argument)
{
    p4scan_camera_t *camera = argument;
    bool stable_pressed = false;
    bool raw_pressed = false;
    bool illumination = true;
    bool applied_aiming = false;
    bool applied_illumination = false;
    int64_t changed_us = esp_timer_get_time();
    for (;;) {
        int64_t now_us = esp_timer_get_time();
        bool pressed = gpio_get_level(CONFIG_P4SCAN_TRIGGER_GPIO) == 0;
        if (pressed != raw_pressed) {
            raw_pressed = pressed;
            changed_us = now_us;
        }
        if (raw_pressed != stable_pressed &&
            now_us - changed_us >= (raw_pressed ? 10000 : 5000)) {
            stable_pressed = raw_pressed;
            if (stable_pressed) illumination = !illumination;
        }
        if (applied_aiming != stable_pressed || applied_illumination != illumination) {
            esp_err_t error = p4scan_ov9281_set_lights(camera->i2c_bus, stable_pressed, illumination);
            if (error == ESP_OK) {
                if (applied_illumination != illumination) {
                    portENTER_CRITICAL(&s_lock);
                    s_light_ready_us = esp_timer_get_time() + 5000 + 1000000 / camera->frame_rate;
                    portEXIT_CRITICAL(&s_lock);
                }
                applied_aiming = stable_pressed;
                applied_illumination = illumination;
                ESP_LOGI(TAG, "button lights: illumination=%d aiming=%d auto_off=disabled", illumination, stable_pressed);
            } else {
                ESP_LOGW(TAG, "button light update failed: %s", esp_err_to_name(error));
            }
        }
        vTaskDelay(1);
    }
}

static void capture_task(void *argument)
{
    p4scan_camera_t *camera = argument;
    uint64_t previous_capture_us = 0;
    uint64_t capture_cycle_us = 0;
    uint64_t last_frame_warning_us = 0;
    uint32_t prepare_total_us = 0, acquire_total_us = 0, dequeue_total_us = 0;
    uint32_t returned_total_us = 0, return_wait_max_us = 0;
    for (;;) {
        uint32_t return_us = 0, return_wait_us = 0;
        for (int index = 0; index < 3; index++) {
            portENTER_CRITICAL(&s_lock);
            bool reusable = s_pool.states[index] == FRAME_FREE;
            portEXIT_CRITICAL(&s_lock);
            if (reusable && s_slots[index].frame.pixels) {
                uint64_t returning_us = esp_timer_get_time();
                if (s_slots[index].released_us && returning_us - s_slots[index].released_us > return_wait_us)
                    return_wait_us = returning_us - s_slots[index].released_us;
                ESP_ERROR_CHECK(p4scan_camera_release(camera, &s_slots[index].frame));
                return_us += esp_timer_get_time() - returning_us;
                s_slots[index].frame.pixels = NULL;
            }
        }
        uint64_t begin_us = esp_timer_get_time();
        if (!capture_cycle_us) capture_cycle_us = begin_us;
        returned_total_us += return_us;
        if (return_wait_us > return_wait_max_us) return_wait_max_us = return_wait_us;
        esp_err_t error = p4scan_camera_begin_capture(camera);
        uint64_t prepared_us = esp_timer_get_time();
        p4scan_camera_frame_t frame = {0};
        if (error == ESP_OK) error = p4scan_camera_acquire(camera, &frame, 1);
        uint64_t acquired_us = esp_timer_get_time();
        prepare_total_us += prepared_us - begin_us;
        acquire_total_us += acquired_us - prepared_us;
        dequeue_total_us += frame.dequeue_us;
        esp_err_t light_error = p4scan_camera_end_capture(camera);
        uint64_t light_done_us = esp_timer_get_time();
        if (error != ESP_OK || light_error != ESP_OK) {
            if (frame.pixels) p4scan_camera_release(camera, &frame);
            if (error == ESP_ERR_TIMEOUT) {
                if (acquired_us - capture_cycle_us >= 1000000 && acquired_us - last_frame_warning_us >= 1000000) {
                    ESP_LOGW(TAG, "no camera frame for %llu ms", (unsigned long long)((acquired_us - capture_cycle_us) / 1000));
                    last_frame_warning_us = acquired_us;
                }
                continue;
            }
            ESP_LOGW(TAG, "capture failed: frame=%s light=%s", esp_err_to_name(error), esp_err_to_name(light_error));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        portENTER_CRITICAL(&s_lock);
        uint64_t light_ready_us = s_light_ready_us;
        portEXIT_CRITICAL(&s_lock);
        if (frame.timestamp_us < light_ready_us) {
            ESP_ERROR_CHECK(p4scan_camera_release(camera, &frame));
            continue;
        }
        portENTER_CRITICAL(&s_lock);
        int slot_index = frame.buffer_index;
        if (s_pool.states[slot_index] == FRAME_FREE) s_pool.states[slot_index] = FRAME_WRITING;
        else slot_index = -1;
        portEXIT_CRITICAL(&s_lock);
        if (slot_index < 0 || frame.width != FRAME_WIDTH || frame.height != FRAME_HEIGHT || frame.stride != FRAME_WIDTH) {
            if (slot_index >= 0) {
                portENTER_CRITICAL(&s_lock);
                latest_frame_release(&s_pool, slot_index);
                portEXIT_CRITICAL(&s_lock);
            }
            p4scan_camera_release(camera, &frame);
            vTaskDelay(1);
            continue;
        }
        agv_slot_t *slot = &s_slots[slot_index];
        slot->pixels = frame.pixels;
        slot->frame = frame;
        slot->released_us = 0;
        slot->frame.dequeue_us = dequeue_total_us;
        slot->return_us = returned_total_us;
        slot->return_wait_us = return_wait_max_us;
        slot->capture_period_us = previous_capture_us ? acquired_us - previous_capture_us : 0;
        previous_capture_us = acquired_us;
        /* Preview and CDC snapshots are generated only after this frame has
         * been decoded; the ISP pipeline already owns auto exposure. */
        slot->begin_us = capture_cycle_us;
        slot->prepare_us = prepare_total_us;
        slot->acquire_us = acquire_total_us;
        slot->light_us = light_done_us - acquired_us;
        capture_cycle_us = 0;
        prepare_total_us = acquire_total_us = dequeue_total_us = 0;
        returned_total_us = return_wait_max_us = 0;
        portENTER_CRITICAL(&s_lock);
        latest_frame_publish(&s_pool, slot_index);
        portEXIT_CRITICAL(&s_lock);
        xTaskNotifyGive(s_consumer);
        vTaskDelay(1);
    }
}

static void finish_decoded_slot(int slot_index)
{
    static uint32_t frames, successes, stale;
    portENTER_CRITICAL(&s_lock);
    uint32_t replaced = s_pool.replaced;
    portEXIT_CRITICAL(&s_lock);
    agv_slot_t *slot = &s_slots[slot_index];
    uint64_t deadline_us = slot->frame.timestamp_us + CONFIG_P4SCAN_FRAME_MAX_AGE_MS * 1000ULL;
    XcrGrayImage image = {
        .pixels = slot->pixels, .width = FRAME_WIDTH, .height = FRAME_HEIGHT, .stride = FRAME_WIDTH,
        .sequence = slot->frame.sequence, .timestamp_us = slot->frame.timestamp_us,
        .capture_generation = 1, .deadline_us = deadline_us,
    };
    uint64_t start_us = slot->decode_start_us;
    XcrBarcodeResult result = slot->result;
    XcrQrTrace trace = slot->trace;
    int count = slot->result_count;
    XcrStatus status = slot->status;
    uint64_t geometry_start = esp_timer_get_time();
    if (count > 0 && result.symbology == XCR_SYM_DATAMATRIX &&
        frame_is_fresh(image.timestamp_us, geometry_start, deadline_us)) {
        slot->marker_stats.geometry_refined = xcr_datamatrix_refine(&image, &result);
        slot->marker_stats.geometry_us = esp_timer_get_time() - geometry_start;
        slot->marker_stats.total_us += slot->marker_stats.geometry_us;
    }
    uint64_t done_us = esp_timer_get_time();
    bool fresh = frame_is_fresh(image.timestamp_us, done_us, deadline_us);
    if (!fresh) { count = 0; stale++; status = XCR_TIMEOUT_NO_RESULT; }
    bool submitted = count > 0 && scan_usb_submit_measured(&result, deadline_us, done_us - start_us);
    frames++;
    if (count > 0) successes++;
    scan_usb_preview(&slot->frame, count > 0 ? &result : NULL, done_us - start_us);
    agv_slot_t completed = *slot;
    portENTER_CRITICAL(&s_lock);
    slot->released_us = esp_timer_get_time();
    latest_frame_release(&s_pool, slot_index);
    portEXIT_CRITICAL(&s_lock);
    slot = &completed;
    p4scan_feedback_set_state(count > 0 ? P4SCAN_FEEDBACK_SUCCESS : P4SCAN_FEEDBACK_FAIL);
    if (frames % 50 == 0) {
    ESP_LOGI(TAG, "seq=%lu prepare=%lu acquire=%lu dq=%lu cache=%lu light_off=%lu queue_age=%llu decode=%llu age=%llu total=%llu status=%s queued=%d replaced=%lu stale=%lu ok=%lu/%lu (us)",
             (unsigned long)image.sequence, (unsigned long)slot->prepare_us,
             (unsigned long)slot->acquire_us, (unsigned long)slot->frame.dequeue_us,
             (unsigned long)slot->frame.cache_sync_us, (unsigned long)slot->light_us,
             (unsigned long long)(start_us - image.timestamp_us),
             (unsigned long long)(done_us - start_us), (unsigned long long)(done_us - image.timestamp_us),
             (unsigned long long)(done_us - slot->begin_us),
             xcr_status_name(status), submitted, (unsigned long)replaced, (unsigned long)stale,
             (unsigned long)successes, (unsigned long)frames);
#if CONFIG_XCR_DATAMATRIX_LIBDMTX && CONFIG_XCR_DATAMATRIX_ONLY
    {
        XcrDmtxStats dm_stats = slot->marker_stats;
        ESP_LOGI(TAG, "libdmtx prescreen=%llu setup=%llu find=%llu decode=%llu total=%llu scale=%u marker=%d hint_hits=%u candidates=%u messages=%u geometry_fallbacks=%u stack_free=%u payload=%s (us)",
                 dm_stats.prescreen_us,
                 dm_stats.setup_us, dm_stats.find_us, dm_stats.decode_us, dm_stats.total_us,
                 dm_stats.scale, dm_stats.marker_roi_valid, dm_stats.hint_hits, dm_stats.candidates, dm_stats.decoded_messages, dm_stats.geometry_fallbacks,
                 (unsigned)uxTaskGetStackHighWaterMark(NULL), count ? result.data : "");
    }
#else
    if (CONFIG_XCR_DATAMATRIX_ONLY || (result.symbology == XCR_SYM_DATAMATRIX && count > 0)) {
        XcrDmtxStats marker_stats = slot->marker_stats;
        ESP_LOGI(TAG, "dm_marker found=%d angle=%.1f detect=%llu normalize=%llu core=%llu geometry=%llu refined=%u total=%llu qr_rejected=%u passes=%u norm_first=%llu norm_retry=%llu (us)",
                 marker_stats.marker_roi_valid, marker_stats.angle_degrees,
                 marker_stats.prescreen_us, marker_stats.setup_us, marker_stats.decode_us,
                 marker_stats.geometry_us, marker_stats.geometry_refined, marker_stats.total_us, marker_stats.qr_rejected,
                 marker_stats.normalize_attempts, marker_stats.normalize_pass_us[0], marker_stats.normalize_pass_us[1]);
        DataMatrixDecodeDebugStats dm_stats = slot->dm_stats;
        ESP_LOGI(TAG, "datamatrix finder=%llu candidate=%llu codeword=%llu ecc=%llu payload=%llu total=%llu finders=%lu payload=%s",
                 (unsigned long long)dm_stats.finder_us,
                 (unsigned long long)dm_stats.candidate_us,
                 (unsigned long long)dm_stats.codeword_us,
                 (unsigned long long)dm_stats.ecc_us,
                 (unsigned long long)dm_stats.payload_us,
                 (unsigned long long)dm_stats.total_us,
                 (unsigned long)dm_stats.finder_hits, result.data);
    } else {
        ESP_LOGI(TAG, "qr sample=%lu identify=%lu refine=%lu ecc=%lu stage=%s finders=%u incomplete=%u geometry=%u geometry_us=%lu payload=%s",
                 (unsigned long)(trace.full8_sample_us + trace.candidate_sample_us + trace.fallback_sample_us),
                 (unsigned long)(trace.full8_identify_us + trace.candidate_identify_us + trace.fallback_identify_us),
                 (unsigned long)(trace.full8_refine_us + trace.candidate_refine_us + trace.fallback_refine_us),
                 (unsigned long)(trace.full8_decode_us + trace.candidate_decode_us + trace.fallback_decode_us),
                 xcr_qr_stage_name(trace.stage), trace.finder_count, trace.incomplete_count,
                 trace.geometry_refined, (unsigned long)trace.geometry_refine_us, count ? result.data : "");
    }
 #endif
    ESP_LOGI(TAG, "geometry seq=%lu quality=%d mask=%u support=%u residual=%.3f core1=%llu gap=%llu refine=%llu",
             (unsigned long)image.sequence, result.geometry_quality, result.geometry_edge_mask,
             result.geometry_support, result.geometry_residual,
             (unsigned long long)(slot->core_done_us - start_us),
             (unsigned long long)(geometry_start - slot->core_done_us),
             (unsigned long long)slot->marker_stats.geometry_us);
    ESP_LOGI(TAG, "flow seq=%lu period=%lu dq=%lu cache=%lu qbuf=%lu return_wait=%lu post=%llu stack_free=%u (us)",
             (unsigned long)image.sequence, (unsigned long)slot->capture_period_us,
             (unsigned long)slot->frame.dequeue_us, (unsigned long)slot->frame.cache_sync_us,
             (unsigned long)slot->return_us, (unsigned long)slot->return_wait_us,
             (unsigned long long)(esp_timer_get_time() - done_us),
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    }
}


static void decode_task(void *argument)
{
    (void)argument;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        portENTER_CRITICAL(&s_lock);
        int slot_index = latest_frame_take(&s_pool);
        portEXIT_CRITICAL(&s_lock);
        if (slot_index < 0) continue;
        agv_slot_t *slot = &s_slots[slot_index];
        uint64_t deadline_us = slot->frame.timestamp_us + CONFIG_P4SCAN_FRAME_MAX_AGE_MS * 1000ULL;
        XcrGrayImage image = {
            .pixels = slot->pixels, .width = FRAME_WIDTH, .height = FRAME_HEIGHT, .stride = FRAME_WIDTH,
            .sequence = slot->frame.sequence, .timestamp_us = slot->frame.timestamp_us,
            .capture_generation = 1, .deadline_us = deadline_us,
        };
        slot->decode_start_us = esp_timer_get_time();
        memset(&slot->result, 0, sizeof(slot->result));
        memset(&slot->trace, 0, sizeof(slot->trace));
        memset(&slot->marker_stats, 0, sizeof(slot->marker_stats));
        memset(&slot->dm_stats, 0, sizeof(slot->dm_stats));
        slot->result_count = 0;
        slot->status = XCR_TIMEOUT_NO_RESULT;
        if (frame_is_fresh(image.timestamp_us, slot->decode_start_us, deadline_us)) {
            slot->status = xcr_decoder_decode(s_decoder, &image, &slot->result, 1, &slot->result_count);
            xcr_decoder_get_qr_trace(s_decoder, &slot->trace);
            xcr_dmtx_stats_get(&slot->marker_stats);
            datamatrix_debug_stats_get(&slot->dm_stats);
        }
        slot->core_done_us = esp_timer_get_time();
        finish_decoded_slot(slot_index);
    }
}

esp_err_t agv_pipeline_start(p4scan_camera_t *camera)
{
    gpio_config_t button_config = {
        .pin_bit_mask = 1ULL << CONFIG_P4SCAN_TRIGGER_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&button_config);
    if (error != ESP_OK) return error;
    error = p4scan_ov9281_set_lights(camera->i2c_bus, false, false);
    if (error != ESP_OK) return error;
    camera->manual_lights = true;
    camera->pooled_frames = true;
    XcrDecoderConfig config;
    xcr_decoder_config_default(&config);
    config.enable_code128 = false;
    config.enable_upc_ean = false;
#if CONFIG_XCR_DATAMATRIX_ONLY
    config.enable_qr = false;
    config.enable_datamatrix = true;
#else
    config.enable_qr = true;
    config.enable_datamatrix = false;
#endif
    config.datamatrix_defer_geometry = true;
    config.qr_quirc_only = true;
    config.qr_fast_search = true;
    config.polarity_retry = false;
    config.image_budget_ms = CONFIG_P4SCAN_FRAME_MAX_AGE_MS;
    s_decoder = xcr_decoder_create(&config);
    if (!s_decoder) return ESP_ERR_NO_MEM;
    esp_log_level_set("p4scan_camera", ESP_LOG_WARN);
    esp_log_level_set("p4scan_power", ESP_LOG_WARN);
    esp_log_level_set("ov9281", ESP_LOG_WARN);
    ESP_LOGI(TAG, "capture=core0 decode+refine=core1 pending=latest-only max_age=%dms free_psram=%u",
             CONFIG_P4SCAN_FRAME_MAX_AGE_MS, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if (xTaskCreatePinnedToCore(decode_task, "agv_decode", 28672, NULL, 8, &s_consumer, 1) != pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(capture_task, "agv_capture", 6144, camera, 8, NULL, 0) != pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(light_button_task, "agv_button", 3072, camera, 9, NULL, 0) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "button: illumination toggles on press; aiming follows hold; debounce=10/5ms; continuous capture");
    return ESP_OK;
}
#endif
