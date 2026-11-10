#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define P4SCAN_CAMERA_DEFAULT_WIDTH  1280
#define P4SCAN_CAMERA_DEFAULT_HEIGHT 720
#define P4SCAN_CAMERA_DEFAULT_FPS    50
#define P4SCAN_CAMERA_BUFFER_COUNT   3

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t frame_rate;
    uint32_t i2c_frequency;
} p4scan_camera_config_t;

typedef struct {
    uint8_t *address;
    size_t length;
} p4scan_camera_buffer_t;

typedef struct {
    int fd;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t pixel_format;
    uint32_t frame_rate;
    p4scan_camera_buffer_t capture[P4SCAN_CAMERA_BUFFER_COUNT];
    int held_buffer_index;
    bool prepared_frame_valid;
    uint32_t prepared_buffer_index;
    uint32_t prepared_bytesused;
    uint64_t prepared_timestamp_us;
    uint32_t frame_sequence;
    bool video_initialized;
    bool streaming;
    bool manual_lights;
    bool pooled_frames;
    uint32_t held_buffer_mask;
    bool lights_enabled;
    bool lights_expired;
    i2c_master_bus_handle_t i2c_bus;
    int successful_exposure;
    int search_exposure;
    int snapshot_exposure;
    bool snapshot_valid;
    int64_t ae_next_adjust_us;
} p4scan_camera_t;

typedef struct {
    const uint8_t *pixels;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t sequence;
    uint64_t timestamp_us;
    int buffer_index;
    uint32_t dequeue_us;
    uint32_t cache_sync_us;
} p4scan_camera_frame_t;

void p4scan_camera_config_default(p4scan_camera_config_t *config);
esp_err_t p4scan_camera_init(p4scan_camera_t *camera,
                              const p4scan_camera_config_t *config);
esp_err_t p4scan_camera_start(p4scan_camera_t *camera);
esp_err_t p4scan_camera_wait_for_frame(p4scan_camera_t *camera, uint32_t timeout_ms);
esp_err_t p4scan_camera_begin_capture(p4scan_camera_t *camera);
esp_err_t p4scan_camera_acquire(p4scan_camera_t *camera,
                                p4scan_camera_frame_t *frame,
                                uint32_t timeout_ms);
esp_err_t p4scan_camera_release(p4scan_camera_t *camera,
                                const p4scan_camera_frame_t *frame);
esp_err_t p4scan_camera_end_capture(p4scan_camera_t *camera);
void p4scan_camera_snapshot_exposure(p4scan_camera_t *camera);
void p4scan_camera_remember_exposure(p4scan_camera_t *camera);
void p4scan_camera_adjust_search_exposure(p4scan_camera_t *camera, uint32_t mean_brightness);
esp_err_t p4scan_camera_stop(p4scan_camera_t *camera);
esp_err_t p4scan_camera_deinit(p4scan_camera_t *camera);

#ifdef __cplusplus
}
#endif
