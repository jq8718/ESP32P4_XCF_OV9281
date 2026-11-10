#pragma once

#include "esp_err.h"

esp_err_t scan_log_init(void);
void scan_log_begin(void);
void scan_log_end(void);
void scan_log_flush(void);
