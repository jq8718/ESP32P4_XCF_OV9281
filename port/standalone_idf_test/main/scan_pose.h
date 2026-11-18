#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "xcr_decoder.h"

typedef struct {
    uint32_t width;
    uint32_t height;
    float fx, fy, cx, cy;
    float k1, k2, p1, p2, k3;
} scan_camera_calibration_t;

typedef struct {
    float yaw_y_deg;
    float pitch_x_deg;
    float roll_z_deg;
    float range_m;
    float optical_depth_m;
    float distance_to_plane_m;
    float tx_m;
    float along_track_m; /* Code relative to camera along the fixed aisle axis. */
    float compute_us;
    float reprojection_px; /* Internal diagnostic; UART CSV format is unchanged. */
} scan_pose_result_t;

void scan_pose_init(void);
void scan_pose_get_calibration(scan_camera_calibration_t *calibration);
esp_err_t scan_pose_set_calibration(const scan_camera_calibration_t *calibration);
bool scan_pose_estimate(const XcrBarcodeResult *result, scan_pose_result_t *pose);
