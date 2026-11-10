#pragma once
#include "esp_err.h"
#include "p4scan_camera_capture.h"
#include "xcr_decoder.h"

esp_err_t scan_usb_init(void);
bool scan_usb_submit(const XcrBarcodeResult *result);
bool scan_usb_submit_timed(const XcrBarcodeResult *result, uint64_t deadline_us);
bool scan_usb_submit_measured(const XcrBarcodeResult *result, uint64_t deadline_us, uint32_t decode_us);
void scan_usb_preview(const p4scan_camera_frame_t *frame, const XcrBarcodeResult *result, uint32_t decode_us);
typedef struct {
    uint32_t sequence, frontend_us, wait_us, normalize_us, core_us, refine_us;
    unsigned sample_step, attempts;
} scan_profile_t;
void scan_usb_profile(const scan_profile_t *profile);

