#include "scan_log_buffer.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define SCAN_LOG_CAPACITY 32768

static char *s_buffer;
static size_t s_length;
static unsigned s_dropped;
static bool s_active;
static vprintf_like_t s_output;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static int scan_log_write(const char *format, va_list arguments)
{
    char line[768];
    portENTER_CRITICAL(&s_lock);
    bool active = s_active;
    portEXIT_CRITICAL(&s_lock);
    if (!active) {
        return s_output(format, arguments);
    }
    va_list copy;
    va_copy(copy, arguments);
    int length = vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    portENTER_CRITICAL(&s_lock);
    active = s_active;
    if (active && length >= 0) {
        if ((size_t)length < sizeof(line) && (size_t)length <= SCAN_LOG_CAPACITY - s_length) {
            memcpy(s_buffer + s_length, line, (size_t)length);
            s_length += (size_t)length;
        } else {
            s_dropped++;
        }
    }
    portEXIT_CRITICAL(&s_lock);
    return active ? length : s_output(format, arguments);
}

static void scan_log_output(const char *format, ...)
{
    va_list arguments;
    va_start(arguments, format);
    s_output(format, arguments);
    va_end(arguments);
}

esp_err_t scan_log_init(void)
{
    if (s_buffer) {
        return ESP_OK;
    }
    s_buffer = heap_caps_malloc(SCAN_LOG_CAPACITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_buffer) {
        return ESP_ERR_NO_MEM;
    }
    s_output = vprintf;
    s_output = esp_log_set_vprintf(scan_log_write);
    return ESP_OK;
}

void scan_log_begin(void)
{
    portENTER_CRITICAL(&s_lock);
    s_length = 0;
    s_dropped = 0;
    s_active = true;
    portEXIT_CRITICAL(&s_lock);
}

void scan_log_end(void)
{
    portENTER_CRITICAL(&s_lock);
    s_active = false;
    portEXIT_CRITICAL(&s_lock);
}

void scan_log_flush(void)
{
    scan_log_end();
    if (s_length) {
        scan_log_output("%.*s", (int)s_length, s_buffer);
    }
    if (s_dropped) {
        ESP_LOGW("scan_log", "deferred log overflow: dropped=%u", s_dropped);
    }
    s_length = 0;
    s_dropped = 0;
}
