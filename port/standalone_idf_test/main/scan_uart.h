#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "xcr_decoder.h"

esp_err_t scan_uart_init(void);
bool scan_uart_submit(const XcrBarcodeResult *result, uint64_t deadline_us);
