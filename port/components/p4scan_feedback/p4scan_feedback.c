#include "p4scan_feedback.h"

#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"

#include <stdbool.h>
#include <stdint.h>

#define TAG "p4scan_feedback"
#define P4SCAN_FEEDBACK_QUEUE_LENGTH 1
#define P4SCAN_FEEDBACK_SUCCESS_BLINKS 2
#define P4SCAN_FEEDBACK_SUCCESS_ON_MS 120
#define P4SCAN_FEEDBACK_SUCCESS_OFF_MS 120
#define P4SCAN_FEEDBACK_FAIL_BLINKS 2
#define P4SCAN_FEEDBACK_FAIL_ON_MS 120
#define P4SCAN_FEEDBACK_FAIL_OFF_MS 120
#define P4SCAN_FEEDBACK_ERROR_BLINKS 4
#define P4SCAN_FEEDBACK_ERROR_ON_MS 100
#define P4SCAN_FEEDBACK_ERROR_OFF_MS 100

static led_strip_handle_t s_strip;
static QueueHandle_t s_state_queue;
static bool s_initialized;

static uint8_t scale_color(uint8_t value)
{
    return (uint16_t)value * CONFIG_P4SCAN_STATUS_LED_BRIGHTNESS / 255;
}

static void set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!s_strip) {
        return;
    }
    for (int index = 0; index < CONFIG_P4SCAN_STATUS_LED_COUNT; index++) {
        led_strip_set_pixel(s_strip, index, scale_color(red),
                            scale_color(green), scale_color(blue));
    }
    led_strip_refresh(s_strip);
}

static void set_off(void)
{
    if (s_strip) {
        led_strip_clear(s_strip);
    }
}

static void render_state(p4scan_feedback_state_t state)
{
    switch (state) {
    case P4SCAN_FEEDBACK_SCANNING:
        set_color(0, 0, 255);
        break;
    case P4SCAN_FEEDBACK_SUCCESS:
        for (int blink = 0; blink < P4SCAN_FEEDBACK_SUCCESS_BLINKS; blink++) {
            set_color(0, 255, 0);
            vTaskDelay(pdMS_TO_TICKS(P4SCAN_FEEDBACK_SUCCESS_ON_MS));
            set_off();
            vTaskDelay(pdMS_TO_TICKS(P4SCAN_FEEDBACK_SUCCESS_OFF_MS));
        }
        break;
    case P4SCAN_FEEDBACK_FAIL:
        for (int blink = 0; blink < P4SCAN_FEEDBACK_FAIL_BLINKS; blink++) {
            set_color(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(P4SCAN_FEEDBACK_FAIL_ON_MS));
            set_off();
            vTaskDelay(pdMS_TO_TICKS(P4SCAN_FEEDBACK_FAIL_OFF_MS));
        }
        break;
    case P4SCAN_FEEDBACK_ERROR:
        for (int blink = 0; blink < P4SCAN_FEEDBACK_ERROR_BLINKS; blink++) {
            set_color(255, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(P4SCAN_FEEDBACK_ERROR_ON_MS));
            set_off();
            vTaskDelay(pdMS_TO_TICKS(P4SCAN_FEEDBACK_ERROR_OFF_MS));
        }
        break;
    case P4SCAN_FEEDBACK_IDLE:
    default:
        set_off();
        break;
    }
}

static void feedback_task(void *arg)
{
    (void)arg;
    p4scan_feedback_state_t state;

    for (;;) {
        if (xQueueReceive(s_state_queue, &state, portMAX_DELAY) == pdTRUE) {
            render_state(state);
        }
    }
}

esp_err_t p4scan_feedback_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_P4SCAN_STATUS_LED_GPIO,
        .max_leds = CONFIG_P4SCAN_STATUS_LED_COUNT,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (ret != ESP_OK) {
        return ret;
    }

    s_state_queue = xQueueCreate(P4SCAN_FEEDBACK_QUEUE_LENGTH,
                                 sizeof(p4scan_feedback_state_t));
    if (!s_state_queue) {
        s_strip = NULL;
        return ESP_ERR_NO_MEM;
    }
    set_off();
    BaseType_t task_ret = xTaskCreatePinnedToCore(feedback_task, "p4scan_led",
                                                  3072, NULL, 1, NULL, 0);
    if (task_ret != pdPASS) {
        vQueueDelete(s_state_queue);
        s_state_queue = NULL;
        s_strip = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "WS2812 status LED ready: GPIO%d count=%d brightness=%d",
             CONFIG_P4SCAN_STATUS_LED_GPIO, CONFIG_P4SCAN_STATUS_LED_COUNT,
             CONFIG_P4SCAN_STATUS_LED_BRIGHTNESS);
    return ESP_OK;
}

esp_err_t p4scan_feedback_set_state(p4scan_feedback_state_t state)
{
    if (!s_initialized || !s_state_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGD(TAG, "state=%d", (int)state);
    return xQueueOverwrite(s_state_queue, &state) == pdPASS ? ESP_OK : ESP_FAIL;
}
