#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    P4SCAN_FEEDBACK_IDLE = 0,
    P4SCAN_FEEDBACK_SCANNING,
    P4SCAN_FEEDBACK_SUCCESS,
    P4SCAN_FEEDBACK_FAIL,
    P4SCAN_FEEDBACK_ERROR,
} p4scan_feedback_state_t;

esp_err_t p4scan_feedback_init(void);
esp_err_t p4scan_feedback_set_state(p4scan_feedback_state_t state);

#ifdef __cplusplus
}
#endif
