#include "scan_usb.h"
#include "xcr_dmtx.h"
#include "latest_frame.h"
#include <stdatomic.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "driver/jpeg_encode.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tusb.h"
#include "usb_device_uvc.h"

#define USB_LINE_CAPACITY (4 * XCR_BARCODE_MAX_LEN + 220)
#define PREVIEW_WIDTH P4SCAN_CAMERA_DEFAULT_WIDTH
#define PREVIEW_HEIGHT P4SCAN_CAMERA_DEFAULT_HEIGHT
#define PREVIEW_SIZE (PREVIEW_WIDTH * PREVIEW_HEIGHT)
#define JPEG_CAPACITY (PREVIEW_SIZE * 2)

typedef struct {
    uint16_t length;
    uint8_t bytes[USB_LINE_CAPACITY];
    int64_t queued_us;
    uint64_t capture_us;
    uint64_t deadline_us;
    uint32_t sequence;
    uint32_t decode_us;
    int geometry_quality;
    unsigned geometry_edge_mask;
    unsigned geometry_support;
    unsigned geometry_residual_milli;
    bool datamatrix;
} scan_packet_t;

static QueueHandle_t s_packets;
static QueueHandle_t s_timings;
static QueueHandle_t s_profiles;
static atomic_bool s_profile_enabled;
typedef struct {
    uint32_t sequence;
    uint64_t capture_us;
    uint32_t decode_us;
    bool success;
} scan_timing_t;
static atomic_uint s_dropped;
static SemaphoreHandle_t s_preview_free, s_preview_ready, s_jpeg_lock;
static uint8_t *s_preview;
static size_t s_preview_size, s_jpeg_size;
static uint64_t s_preview_timestamp;
static uvc_fb_t s_jpeg;
static jpeg_encoder_handle_t s_encoder;
static atomic_bool s_preview_enabled;
static atomic_bool s_debug_enabled, s_snapshot_requested;
static atomic_bool s_timing_enabled;
static atomic_bool s_geometry_enabled;
static atomic_bool s_subpixel_enabled;
static atomic_bool s_encode_requested;
static atomic_uint s_debug_until_tick;
static atomic_int s_raw_request;
static atomic_bool s_raw_ready;
static bool s_preview_raw;

static void handle_cdc_command(const uint8_t *bytes, size_t received)
{
    static char command[32];
    static size_t used;
    static bool overflow;

    for (size_t index = 0; index < received; index++) {
        if (bytes[index] == '\r' || bytes[index] == '\n') {
            command[used] = 0;
            ESP_LOGI("p4scan_usb", "CDC command=%s", overflow ? "<overflow>" : command);
            if (!overflow && strcmp(command, "@SNAP") == 0) {
                atomic_store(&s_debug_until_tick, xTaskGetTickCount() + pdMS_TO_TICKS(3000));
                atomic_store(&s_debug_enabled, true);
                atomic_store(&s_encode_requested, true);
            } else if (!overflow && strcmp(command, "@RAWFAIL") == 0) {
                atomic_store(&s_raw_request, 2);
            } else if (!overflow && strcmp(command, "@RAWCOARSE") == 0) {
                atomic_store(&s_raw_request, 3);
            } else if (!overflow && strcmp(command, "@RAW") == 0) {
                atomic_store(&s_raw_request, 1);
            } else if (!overflow && strcmp(command, "@TIMINGON") == 0) {
                atomic_store(&s_timing_enabled, true);
            } else if (!overflow && strcmp(command, "@PROFILEON") == 0) {
                atomic_store(&s_profile_enabled, true);
            } else if (!overflow && strcmp(command, "@PROFILEOFF") == 0) {
                atomic_store(&s_profile_enabled, false);
            } else if (!overflow && strcmp(command, "@GEOMETRYON") == 0) {
                atomic_store(&s_geometry_enabled, true);
            } else if (!overflow && strcmp(command, "@GEOMETRYOFF") == 0) {
                atomic_store(&s_geometry_enabled, false);
            } else if (!overflow && strcmp(command, "@SUBPIXELON") == 0) {
                atomic_store(&s_subpixel_enabled, true);
            } else if (!overflow && strcmp(command, "@SUBPIXELOFF") == 0) {
                atomic_store(&s_subpixel_enabled, false);
            } else if (!overflow && strcmp(command, "@TIMINGOFF") == 0) {
                atomic_store(&s_timing_enabled, false);
            } else if (!overflow && strcmp(command, "@DEBUGOFF") == 0) {
                atomic_store(&s_debug_enabled, false);
                atomic_store(&s_snapshot_requested, false);
                atomic_store(&s_encode_requested, false);
                atomic_store(&s_raw_request, 0);
            }
            used = 0;
            overflow = false;
        } else if (used < sizeof(command) - 1) {
            command[used++] = (char)bytes[index];
        } else {
            overflow = true;
        }
    }
}

typedef struct {
    uint32_t sequence;
    uint64_t timestamp_us;
    uint32_t decode_us;
    bool valid;
    XcrBarcodeResult result;
} snapshot_metadata_t;

static snapshot_metadata_t s_preview_metadata, s_jpeg_metadata;

static bool format_corners(char *text, size_t capacity, const XcrBarcodeResult *result)
{
    bool precise = atomic_load(&s_subpixel_enabled) && result->subpixel_valid &&
                   result->geometry_quality == XCR_GEOMETRY_REFINED;
    size_t used = 0;
    for (int index = 0; index < 8; index++) {
        int written;
        if (precise) {
            float coordinate = result->subpixel_corners[index];
            if (!isfinite(coordinate) || coordinate < 0 ||
                coordinate >= (index & 1 ? PREVIEW_HEIGHT : PREVIEW_WIDTH)) return false;
            int milli = (int)lroundf(coordinate * 1000);
            written = snprintf(text + used, capacity - used, "%s%d.%03d",
                               index ? "," : "", milli / 1000, milli % 1000);
        } else {
            written = snprintf(text + used, capacity - used, "%s%d", index ? "," : "", result->corners[index]);
        }
        if (written < 0 || (size_t)written >= capacity - used) return false;
        used += written;
    }
    return true;
}

static bool debug_active(void)
{
    return atomic_load(&s_debug_enabled) &&
           (int32_t)(atomic_load(&s_debug_until_tick) - xTaskGetTickCount()) > 0;
}

static bool debug_write(const void *bytes, size_t length, int64_t deadline)
{
    while (tud_cdc_ready() && tud_cdc_write_available() < length && esp_timer_get_time() < deadline) {
        tud_cdc_write_flush();
        vTaskDelay(1);
    }
    if (!tud_cdc_ready() || esp_timer_get_time() >= deadline) return false;
    bool complete = tud_cdc_write(bytes, length) == length;
    tud_cdc_write_flush();
    return complete;
}

static bool send_result(scan_packet_t *packet)
{
    bool geometry = atomic_load(&s_geometry_enabled) && packet->datamatrix;
    if (!geometry && packet->datamatrix && packet->geometry_quality != 2) return false;
    uint64_t deadline = packet->deadline_us ? packet->deadline_us : packet->queued_us + 100000;
    size_t reserved = packet->length + (packet->deadline_us ? 120 : 0);
    while (tud_cdc_ready() && tud_cdc_write_available() < reserved && (uint64_t)esp_timer_get_time() < deadline) {
        tud_cdc_write_flush();
        vTaskDelay(1);
    }
    uint64_t now = esp_timer_get_time();
    if (!tud_cdc_ready() || now >= deadline) return false;
    if (packet->deadline_us) {
        if (!frame_is_fresh(packet->capture_us, now, deadline)) return false;
        int appended = snprintf((char *)packet->bytes + packet->length - 1,
                                sizeof(packet->bytes) - packet->length + 1, ",%lu,%llu,%llu\r",
                                (unsigned long)packet->sequence, (unsigned long long)packet->capture_us,
                                (unsigned long long)(now - packet->capture_us));
        if (appended < 0 || (size_t)appended >= sizeof(packet->bytes) - packet->length + 1) return false;
        packet->length += appended - 1;
        if (atomic_load(&s_timing_enabled) || geometry) {
            appended = snprintf((char *)packet->bytes + packet->length - 1,
                                sizeof(packet->bytes) - packet->length + 1, ",%lu\r",
                                (unsigned long)packet->decode_us);
            if (appended < 0 || (size_t)appended >= sizeof(packet->bytes) - packet->length + 1) return false;
            packet->length += appended - 1;
        }
        if (geometry) {
            appended = snprintf((char *)packet->bytes + packet->length - 1,
                                sizeof(packet->bytes) - packet->length + 1, ",%d,%u,%u,%u\r",
                                packet->geometry_quality, packet->geometry_edge_mask,
                                packet->geometry_support, packet->geometry_residual_milli);
            if (appended < 0 || (size_t)appended >= sizeof(packet->bytes) - packet->length + 1) return false;
            packet->length += appended - 1;
        }
    }
    if ((uint64_t)esp_timer_get_time() >= deadline) return false;
    bool complete = tud_cdc_write(packet->bytes, packet->length) == packet->length;
    tud_cdc_write_flush();
    return complete;
}

static void send_pending_results(void)
{
    scan_packet_t packet;
    while (xQueueReceive(s_packets, &packet, 0) == pdTRUE) {
        if (!send_result(&packet)) {
            atomic_fetch_add(&s_dropped, 1);
        }
    }
}

static void send_raw_snapshot(void)
{
    int64_t deadline = esp_timer_get_time() + 5000000;
    uint32_t crc = esp_rom_crc32_le(0, s_preview, PREVIEW_SIZE);
    char line[800];
    int count = snprintf(line, sizeof(line), "@RAWFRAME,%lu,%llu,%u,%u,%d,%lu,%u,%08lX\r",
                         (unsigned long)s_preview_metadata.sequence,
                         (unsigned long long)s_preview_metadata.timestamp_us,
                         PREVIEW_WIDTH, PREVIEW_HEIGHT,
                         s_preview_metadata.valid, (unsigned long)s_preview_metadata.decode_us,
                         PREVIEW_SIZE, (unsigned long)crc);
    bool complete = debug_write(line, count, deadline);
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (size_t offset = 0; complete && offset < PREVIEW_SIZE; offset += 384) {
        size_t chunk = PREVIEW_SIZE - offset < 384 ? PREVIEW_SIZE - offset : 384;
        size_t used = 9;
        memcpy(line, "@RAWDATA,", used);
        for (size_t index = 0; index < chunk; index += 3) {
            uint32_t value = (uint32_t)s_preview[offset + index] << 16;
            if (index + 1 < chunk) value |= (uint32_t)s_preview[offset + index + 1] << 8;
            if (index + 2 < chunk) value |= s_preview[offset + index + 2];
            line[used++] = alphabet[(value >> 18) & 63];
            line[used++] = alphabet[(value >> 12) & 63];
            line[used++] = index + 1 < chunk ? alphabet[(value >> 6) & 63] : '=';
            line[used++] = index + 2 < chunk ? alphabet[value & 63] : '=';
        }
        line[used++] = '\r';
        complete = debug_write(line, used, deadline);
        send_pending_results();
    }
    if (complete) {
        count = snprintf(line, sizeof(line), "@RAWEND,%lu\r", (unsigned long)s_preview_metadata.sequence);
        debug_write(line, count, deadline);
    }
    xSemaphoreGive(s_preview_free);
}

static void send_snapshot(void)
{
    static uint8_t *copy;
    if (!copy) copy = heap_caps_malloc(JPEG_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int64_t deadline = esp_timer_get_time() + 2000000;
    if (!copy || xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        const char reply[] = "@WAIT,busy\r";
        debug_write(reply, sizeof(reply) - 1, deadline);
        return;
    }
    size_t length = s_jpeg.len;
    snapshot_metadata_t metadata = s_jpeg_metadata;
    if (length) memcpy(copy, s_jpeg.buf, length);
    xSemaphoreGive(s_jpeg_lock);
    if (!length) {
        const char reply[] = "@WAIT,no_frame\r";
        debug_write(reply, sizeof(reply) - 1, deadline);
        return;
    }
    uint32_t crc = esp_rom_crc32_le(0, copy, length);
    char payload_hex[XCR_BARCODE_MAX_LEN * 2 + 1];
    static const char hex[] = "0123456789ABCDEF";
    int payload_length = metadata.valid ? metadata.result.length : 0;
    for (int index = 0; index < payload_length; index++) {
        uint8_t value = (uint8_t)metadata.result.data[index];
        payload_hex[2 * index] = hex[value >> 4];
        payload_hex[2 * index + 1] = hex[value & 15];
    }
    payload_hex[2 * payload_length] = 0;
    char line[800];
    char coordinates[128];
    if (!format_corners(coordinates, sizeof(coordinates), &metadata.result)) return;
    int count = snprintf(line, sizeof(line), "@FRAME,1,%lu,%llu,%u,%u,%d,%lu,%s,%s,%u,%08lX\r",
                         (unsigned long)metadata.sequence, (unsigned long long)metadata.timestamp_us,
                         PREVIEW_WIDTH, PREVIEW_HEIGHT,
                         metadata.valid, (unsigned long)metadata.decode_us,
                         coordinates,
                         payload_hex, (unsigned)length, (unsigned long)crc);
    if (count > 0 && (size_t)count < sizeof(line) && atomic_load(&s_geometry_enabled) &&
        metadata.result.symbology == XCR_SYM_DATAMATRIX) {
        line[7] = '2';
        int appended = snprintf(line + count - 1, sizeof(line) - count + 1, ",%d,%u,%u,%u\r",
                                metadata.result.geometry_quality, metadata.result.geometry_edge_mask,
                                metadata.result.geometry_support, (unsigned)(metadata.result.geometry_residual * 1000));
        if (appended < 0 || (size_t)appended >= sizeof(line) - count + 1) return;
        count += appended - 1;
    }
    if (count < 0 || (size_t)count >= sizeof(line) || !debug_write(line, count, deadline)) return;
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (size_t offset = 0; offset < length; offset += 384) {
        size_t chunk = length - offset < 384 ? length - offset : 384;
        size_t used = 6;
        memcpy(line, "@DATA,", used);
        for (size_t index = 0; index < chunk; index += 3) {
            uint32_t value = (uint32_t)copy[offset + index] << 16;
            if (index + 1 < chunk) value |= (uint32_t)copy[offset + index + 1] << 8;
            if (index + 2 < chunk) value |= copy[offset + index + 2];
            line[used++] = alphabet[(value >> 18) & 63];
            line[used++] = alphabet[(value >> 12) & 63];
            line[used++] = index + 1 < chunk ? alphabet[(value >> 6) & 63] : '=';
            line[used++] = index + 2 < chunk ? alphabet[value & 63] : '=';
        }
        line[used++] = '\r';
        if (!debug_write(line, used, deadline)) return;
        send_pending_results();
    }
    count = snprintf(line, sizeof(line), "@END,%lu\r", (unsigned long)metadata.sequence);
    debug_write(line, count, deadline);
}

bool scan_usb_submit_measured(const XcrBarcodeResult *result, uint64_t deadline_us, uint32_t decode_us)
{
    if (!s_packets || !result || !result->geometry_valid ||
        result->length <= 0 || result->length >= XCR_BARCODE_MAX_LEN) return false;
    scan_packet_t packet = {0};
    packet.queued_us = esp_timer_get_time();
    packet.capture_us = result->timestamp_us;
    packet.sequence = result->sequence;
    packet.deadline_us = deadline_us;
    packet.decode_us = decode_us;
    packet.datamatrix = result->symbology == XCR_SYM_DATAMATRIX;
    packet.geometry_quality = result->geometry_quality;
    packet.geometry_edge_mask = result->geometry_edge_mask;
    packet.geometry_support = result->geometry_support;
    packet.geometry_residual_milli = (unsigned)(result->geometry_residual * 1000);
    if (deadline_us && !frame_is_fresh(packet.capture_us, packet.queued_us, deadline_us)) return false;
    bool quoted = memchr(result->data, ',', result->length) || memchr(result->data, '"', result->length);
    if (quoted) packet.bytes[packet.length++] = '"';
    const char hex[] = "0123456789ABCDEF";
    for (int index = 0; index < result->length; index++) {
        uint8_t value = (uint8_t)result->data[index];
        if (value < 32 || value == 127) {
            packet.bytes[packet.length++] = '\\';
            packet.bytes[packet.length++] = 'x';
            packet.bytes[packet.length++] = hex[value >> 4];
            packet.bytes[packet.length++] = hex[value & 15];
        } else {
            if (value == '"' || value == '\\') packet.bytes[packet.length++] = value;
            packet.bytes[packet.length++] = value;
        }
    }
    if (quoted) packet.bytes[packet.length++] = '"';
    char coordinates[128];
    if (!format_corners(coordinates, sizeof(coordinates), result)) return false;
    int appended = snprintf((char *)packet.bytes + packet.length, sizeof(packet.bytes) - packet.length,
                            ",%s\r", coordinates);
    if (appended < 0 || (size_t)appended >= sizeof(packet.bytes) - packet.length) return false;
    packet.length += appended;
    if (xQueueOverwrite(s_packets, &packet) == pdTRUE) return true;
    atomic_fetch_add(&s_dropped, 1);
    return false;
}

bool scan_usb_submit_timed(const XcrBarcodeResult *result, uint64_t deadline_us)
{
    return scan_usb_submit_measured(result, deadline_us, 0);
}

bool scan_usb_submit(const XcrBarcodeResult *result)
{
    return scan_usb_submit_timed(result, 0);
}

void scan_usb_profile(const scan_profile_t *profile)
{
    if (s_profiles && atomic_load(&s_profile_enabled)) xQueueOverwrite(s_profiles, profile);
}

static void cdc_task(void *argument)
{
    (void)argument;
    scan_packet_t packet;
    bool was_ready = false;
    uint64_t cpu_sample_us = 0;
    uint64_t profile_sent_us = 0;
    uint32_t idle0_previous = 0, idle1_previous = 0;
    for (;;) {
        uint8_t command_bytes[64];
        bool ready = tud_cdc_ready();
        if (ready != was_ready) {
            if (!ready) {
                atomic_store(&s_profile_enabled, false);
                atomic_store(&s_timing_enabled, false);
                atomic_store(&s_geometry_enabled, false);
            }
            ESP_LOGI("p4scan_usb", "CDC ready=%d dtr=%d speed=%s", ready, tud_cdc_connected(),
                     tud_speed_get() == TUSB_SPEED_HIGH ? "HS" : "FS");
            was_ready = ready;
        }
        uint32_t available = tud_cdc_n_available(0);
        while (available > 0) {
            uint32_t length = tud_cdc_n_read(0, command_bytes,
                                              available < sizeof(command_bytes) ? available : sizeof(command_bytes));
            if (!length) break;
            handle_cdc_command(command_bytes, length);
            available = tud_cdc_n_available(0);
        }
        if (atomic_exchange(&s_raw_ready, false)) send_raw_snapshot();
        if (atomic_exchange(&s_snapshot_requested, false) && ready) send_snapshot();
        scan_timing_t timing;
        if (xQueueReceive(s_timings, &timing, 0) == pdTRUE && atomic_load(&s_timing_enabled) && ready) {
            char line[96];
            int length = snprintf(line, sizeof(line), "@TIMING,%lu,%llu,%lu,%d\r",
                                  (unsigned long)timing.sequence, (unsigned long long)timing.capture_us,
                                  (unsigned long)timing.decode_us, timing.success);
            if (length > 0 && length < sizeof(line) && tud_cdc_write_available() >= length) {
                tud_cdc_write(line, length);
                tud_cdc_write_flush();
            }
        }
        if (ready && atomic_load(&s_profile_enabled) &&
            (uint64_t)esp_timer_get_time() - profile_sent_us >= 1000000) {
            scan_profile_t profile;
            if (xQueueReceive(s_profiles, &profile, 0) == pdTRUE) {
                char line[160];
                int length = snprintf(line, sizeof(line), "@PROFILE,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u\r",
                    (unsigned long)profile.sequence, (unsigned long)profile.frontend_us,
                    (unsigned long)profile.wait_us, (unsigned long)profile.normalize_us,
                    (unsigned long)profile.core_us, (unsigned long)profile.refine_us,
                    profile.sample_step, profile.attempts);
                if (length > 0 && length < sizeof(line) && tud_cdc_write_available() >= length) {
                    tud_cdc_write(line, length);
                    tud_cdc_write_flush();
                    profile_sent_us = esp_timer_get_time();
                }
            }
        }
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        uint64_t cpu_now_us = esp_timer_get_time();
        if (!cpu_sample_us) {
            cpu_sample_us = cpu_now_us;
            idle0_previous = ulTaskGetIdleRunTimeCounterForCore(0);
            idle1_previous = ulTaskGetIdleRunTimeCounterForCore(1);
        } else if (cpu_now_us - cpu_sample_us >= 1000000) {
            uint32_t idle0_now = ulTaskGetIdleRunTimeCounterForCore(0);
            uint32_t idle1_now = ulTaskGetIdleRunTimeCounterForCore(1);
            uint64_t elapsed = cpu_now_us - cpu_sample_us;
            uint32_t idle0_delta = idle0_now - idle0_previous;
            uint32_t idle1_delta = idle1_now - idle1_previous;
            unsigned cpu0 = idle0_delta < elapsed ? 1000U - (unsigned)((uint64_t)idle0_delta * 1000 / elapsed) : 0;
            unsigned cpu1 = idle1_delta < elapsed ? 1000U - (unsigned)((uint64_t)idle1_delta * 1000 / elapsed) : 0;
            cpu_sample_us = cpu_now_us;
            idle0_previous = idle0_now;
            idle1_previous = idle1_now;
#if CONFIG_P4SCAN_PIPELINE_DIAGNOSTICS
            ESP_LOGI("p4scan_cpu", "interval_ms=%u core0_permille=%u core1_permille=%u",
                     (unsigned)(elapsed / 1000), cpu0, cpu1);
#endif
            if (ready) {
                char line[48];
                int length = snprintf(line, sizeof(line), "@CPU,%u,%u,%u\r",
                                      (unsigned)(elapsed / 1000), cpu0, cpu1);
                if (length > 0 && length < sizeof(line) && tud_cdc_write_available() >= length) {
                    tud_cdc_write(line, length);
                    tud_cdc_write_flush();
                }
            }
        }
#endif
        if (xQueueReceive(s_packets, &packet, pdMS_TO_TICKS(2) ? pdMS_TO_TICKS(2) : 1) != pdTRUE) continue;
        if (!send_result(&packet)) {
            atomic_fetch_add(&s_dropped, 1);
        }
    }
}

void tud_cdc_rx_cb(uint8_t interface)
{
    uint8_t bytes[64];
    uint32_t received;
    while ((received = tud_cdc_n_read(interface, bytes, sizeof(bytes))) > 0) {
        handle_cdc_command(bytes, received);
    }
}

static esp_err_t preview_start(uvc_format_t format, int width, int height, int rate, void *context)
{
    (void)rate;
    (void)context;
    if (format != UVC_FORMAT_JPEG || width != PREVIEW_WIDTH || height != PREVIEW_HEIGHT) return ESP_ERR_NOT_SUPPORTED;
    atomic_store(&s_preview_enabled, true);
    return ESP_OK;
}

static void preview_stop(void *context)
{
    (void)context;
    atomic_store(&s_preview_enabled, false);
}

static uvc_fb_t *preview_get(void *context)
{
    (void)context;
    if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(10)) != pdTRUE) return NULL;
    if (s_jpeg.len) return &s_jpeg;
    xSemaphoreGive(s_jpeg_lock);
    return NULL;
}

static void preview_return(uvc_fb_t *frame, void *context)
{
    (void)frame;
    (void)context;
    xSemaphoreGive(s_jpeg_lock);
}

static void queue_preview(const p4scan_camera_frame_t *frame, const XcrBarcodeResult *result, uint32_t decode_us)
{
    if (s_timings && atomic_load(&s_timing_enabled)) {
        scan_timing_t timing = {frame->sequence, frame->timestamp_us, decode_us, result != NULL};
        xQueueOverwrite(s_timings, &timing);
    }
    int raw_request = atomic_load(&s_raw_request);
    bool raw = raw_request == 1 || (raw_request == 2 && !result) ||
               (raw_request == 3 && result && result->geometry_quality != XCR_GEOMETRY_REFINED);
    if ((!raw && !atomic_load(&s_preview_enabled) && !(debug_active() && atomic_load(&s_encode_requested))) ||
        frame->width != PREVIEW_WIDTH || frame->height != PREVIEW_HEIGHT ||
        frame->stride != PREVIEW_WIDTH || xSemaphoreTake(s_preview_free, 0) != pdTRUE) return;
    s_preview_raw = raw;
    if (raw) atomic_store(&s_raw_request, 0);
    atomic_store(&s_encode_requested, false);
    memcpy(s_preview, frame->pixels, PREVIEW_SIZE);
    s_preview_timestamp = frame->timestamp_us;
    memset(&s_preview_metadata, 0, sizeof(s_preview_metadata));
    s_preview_metadata.sequence = frame->sequence;
    s_preview_metadata.timestamp_us = frame->timestamp_us;
    s_preview_metadata.decode_us = decode_us;
    if (result && result->geometry_valid && result->length > 0 && result->length < XCR_BARCODE_MAX_LEN) {
        s_preview_metadata.result = *result;
        s_preview_metadata.valid = true;
    }
    xSemaphoreGive(s_preview_ready);
}

void scan_usb_preview(const p4scan_camera_frame_t *frame, const XcrBarcodeResult *result, uint32_t decode_us)
{
    queue_preview(frame, result, decode_us);
}

static void preview_task(void *argument)
{
    (void)argument;
    jpeg_encode_cfg_t config = {
        .height = PREVIEW_HEIGHT, .width = PREVIEW_WIDTH, .src_type = JPEG_ENC_SRC_GRAY,
        .sub_sample = JPEG_DOWN_SAMPLING_GRAY, .image_quality = 75,
    };
    TickType_t next_video_tick = 0;
    for (;;) {
        xSemaphoreTake(s_preview_ready, portMAX_DELAY);
        if (s_preview_raw) {
            atomic_store(&s_raw_ready, true);
            continue;
        }
        bool video = atomic_load(&s_preview_enabled);
        if (video) {
            TickType_t now = xTaskGetTickCount();
            if (next_video_tick && (int32_t)(next_video_tick - now) > 0)
                vTaskDelay(next_video_tick - now);
            next_video_tick = xTaskGetTickCount() + pdMS_TO_TICKS(1000 / CONFIG_UVC_CAM1_FRAMERATE);
        } else {
            next_video_tick = 0;
        }
        config.src_type = video ? JPEG_ENC_SRC_RGB888 : JPEG_ENC_SRC_GRAY;
        config.sub_sample = video ? JPEG_DOWN_SAMPLING_YUV420 : JPEG_DOWN_SAMPLING_GRAY;
        if (video) {
            /* IDF 5.4 JPEG's Gray8 source shifts horizontal coordinates on
             * this target. Expand backwards so UVC and CSI coordinates match. */
            uint32_t *words = (uint32_t *)s_preview;
            for (size_t block = PREVIEW_SIZE / 4; block > 0; block--) {
                uint32_t gray = words[block - 1];
                uint32_t g0 = gray & 0xffU, g1 = (gray >> 8) & 0xffU;
                uint32_t g2 = (gray >> 16) & 0xffU, g3 = gray >> 24;
                size_t output = (block - 1) * 3;
                words[output] = g0 * 0x010101U | (g1 << 24);
                words[output + 1] = g1 * 0x0101U | g2 * 0x01010000U;
                words[output + 2] = g2 | g3 * 0x01010100U;
            }
        }
        xSemaphoreTake(s_jpeg_lock, portMAX_DELAY);
        uint32_t length = 0;
        if (jpeg_encoder_process(s_encoder, &config, s_preview, s_preview_size,
                                 s_jpeg.buf, s_jpeg_size, &length) == ESP_OK) {
            s_jpeg.len = length;
            s_jpeg.timestamp.tv_sec = s_preview_timestamp / 1000000ULL;
            s_jpeg.timestamp.tv_usec = s_preview_timestamp % 1000000ULL;
            s_jpeg_metadata = s_preview_metadata;
            if (debug_active()) atomic_store(&s_snapshot_requested, true);
        }
        xSemaphoreGive(s_jpeg_lock);
        xSemaphoreGive(s_preview_free);
    }
}

esp_err_t scan_usb_init(void)
{
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    extern const char *string_desc_arr[];
    static char serial[24];
    snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X-OV9281", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    string_desc_arr[3] = serial;
    s_packets = xQueueCreate(1, sizeof(scan_packet_t));
    s_profiles = xQueueCreate(1, sizeof(scan_profile_t));
    if (!s_profiles) return ESP_ERR_NO_MEM;
    s_timings = xQueueCreate(1, sizeof(scan_timing_t));
    if (!s_timings) return ESP_ERR_NO_MEM;
    s_preview_free = xSemaphoreCreateBinary();
    s_preview_ready = xSemaphoreCreateBinary();
    s_jpeg_lock = xSemaphoreCreateMutex();
    jpeg_encode_memory_alloc_cfg_t memory = {.buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER};
    s_preview = jpeg_alloc_encoder_mem(PREVIEW_SIZE * 3, &memory, &s_preview_size);
    memory.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
    s_jpeg.buf = jpeg_alloc_encoder_mem(JPEG_CAPACITY, &memory, &s_jpeg_size);
    uint8_t *transfer = heap_caps_malloc(JPEG_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_packets || !s_preview_free || !s_preview_ready || !s_jpeg_lock || !s_preview || !s_jpeg.buf || !transfer) return ESP_ERR_NO_MEM;
    ESP_LOGI("p4scan_usb", "init memory ready preview=%u jpeg=%u transfer=%u",
             (unsigned)s_preview_size, (unsigned)s_jpeg_size, JPEG_CAPACITY);
    s_jpeg.width = PREVIEW_WIDTH;
    s_jpeg.height = PREVIEW_HEIGHT;
    s_jpeg.format = UVC_FORMAT_JPEG;
    jpeg_encode_engine_cfg_t engine = {.timeout_ms = 100};
    ESP_ERROR_CHECK(jpeg_new_encoder_engine(&engine, &s_encoder));
    ESP_LOGI("p4scan_usb", "init JPEG encoder ready");
    xSemaphoreGive(s_preview_free);
    uvc_device_config_t config = {
        .uvc_buffer = transfer, .uvc_buffer_size = JPEG_CAPACITY, .start_cb = preview_start,
        .stop_cb = preview_stop, .fb_get_cb = preview_get, .fb_return_cb = preview_return,
    };
    ESP_ERROR_CHECK(uvc_device_config(0, &config));
    ESP_LOGI("p4scan_usb", "init UVC config ready");
    ESP_ERROR_CHECK(uvc_device_init());
    ESP_LOGI("p4scan_usb", "init UVC device ready");
    /* In the frame pipeline, keep JPEG formatting/CDC on the backend core:
     * core0's full-frame marker search has much less spare time. TinyUSB's
     * endpoint task stays on its configured core with higher priority. */
#if CONFIG_P4SCAN_FRAME_STAGE_PIPELINE
    const BaseType_t cdc_core = 1;
#else
    const BaseType_t cdc_core = 0;
#endif
    if (xTaskCreatePinnedToCore(cdc_task, "scan_cdc", 8192, NULL, 7, NULL, cdc_core) != pdPASS ||
        xTaskCreatePinnedToCore(preview_task, "scan_preview", 4096, NULL, 7, NULL, 1) != pdPASS) return ESP_ERR_NO_MEM;
    ESP_LOGI("p4scan_usb", "init USB worker tasks ready");
    ESP_LOGI("p4scan_usb", "HS UVC+CDC ready serial=%s protocol=CSV/CR", serial);
    return ESP_OK;
}
