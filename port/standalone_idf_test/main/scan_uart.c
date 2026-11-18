#include "scan_uart.h"
#include "scan_pose.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "p4scan_camera_capture.h"

#define SCAN_UART_PORT UART_NUM_1
#define SCAN_UART_TX GPIO_NUM_41
#define SCAN_UART_RX GPIO_NUM_42
#define SCAN_UART_BAUD 921600
#define SCAN_UART_LINE_CAPACITY (4 * XCR_BARCODE_MAX_LEN + 256)

typedef struct {
    uint16_t length;
    char line[SCAN_UART_LINE_CAPACITY];
    uint64_t deadline_us;
} scan_uart_packet_t;

static const char *TAG = "p4scan_uart";
static QueueHandle_t s_packets;
static SemaphoreHandle_t s_tx_lock;

static void uart_send_line(const char *line, size_t length)
{
    if (!line || !length || !s_tx_lock) return;
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "TX busy; response dropped");
        return;
    }
    int written = uart_write_bytes(SCAN_UART_PORT, line, length);
    if (written != (int)length) ESP_LOGW(TAG, "TX write short: %d/%u", written, (unsigned)length);
    xSemaphoreGive(s_tx_lock);
}

static void scan_uart_task(void *argument)
{
    (void)argument;
    scan_uart_packet_t packet;
    for (;;) {
        if (xQueueReceive(s_packets, &packet, portMAX_DELAY) != pdTRUE) continue;
        if (packet.deadline_us && (uint64_t)esp_timer_get_time() >= packet.deadline_us) continue;
        /* TX has its own driver ring buffer; decoding never waits for the wire. */
        uart_send_line(packet.line, packet.length);
    }
}

static void uart_calibration_reply(bool ok, const char *reason)
{
    char line[256];
    int length;
    if (!ok) {
        length = snprintf(line, sizeof(line), "@CAL,ERR,%s\r\n", reason ? reason : "invalid");
    } else {
        scan_camera_calibration_t c;
        scan_pose_get_calibration(&c);
        length = snprintf(line, sizeof(line),
            "@CAL,DATA,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,%.7f\r\n",
            (unsigned long)c.width, (unsigned long)c.height, c.fx, c.fy, c.cx, c.cy,
            c.k1, c.k2, c.p1, c.p2, c.k3);
    }
    if (length > 0 && length < (int)sizeof(line)) uart_send_line(line, (size_t)length);
}

static void handle_calibration_command(char *line)
{
    char *fields[16] = {0};
    int count = 0;
    char *save = NULL;
    for (char *field = strtok_r(line, ",", &save); field && count < 16;
         field = strtok_r(NULL, ",", &save)) fields[count++] = field;
    if (count == 2 && !strcmp(fields[0], "@CAL") && !strcmp(fields[1], "GET")) {
        uart_calibration_reply(true, NULL);
        return;
    }
    if (count != 13 || strcmp(fields[0], "@CAL") || strcmp(fields[1], "SET")) {
        uart_calibration_reply(false, "format");
        return;
    }
    float values[11];
    for (int i = 0; i < 11; ++i) {
        char *end = NULL;
        values[i] = strtof(fields[i + 2], &end);
        if (end == fields[i + 2] || *end || !isfinite(values[i])) {
            uart_calibration_reply(false, "number");
            return;
        }
    }
    if (values[0] != 1280.0f || values[1] != 720.0f) {
        uart_calibration_reply(false, "resolution");
        return;
    }
    scan_camera_calibration_t c = {
        .width = (uint32_t)values[0], .height = (uint32_t)values[1],
        .fx = values[2], .fy = values[3], .cx = values[4], .cy = values[5],
        .k1 = values[6], .k2 = values[7], .p1 = values[8], .p2 = values[9], .k3 = values[10],
    };
    esp_err_t err = scan_pose_set_calibration(&c);
    if (err != ESP_OK) {
        uart_calibration_reply(false, err == ESP_ERR_INVALID_ARG ? "range" : "flash");
        return;
    }
    uart_calibration_reply(true, NULL);
}

static void scan_uart_rx_task(void *argument)
{
    (void)argument;
    uint8_t input[64];
    char line[256];
    size_t used = 0;
    for (;;) {
        int count = uart_read_bytes(SCAN_UART_PORT, input, sizeof(input), pdMS_TO_TICKS(100));
        for (int i = 0; i < count; ++i) {
            char ch = (char)input[i];
            if (ch == '\r' || ch == '\n') {
                if (used) {
                    line[used] = '\0';
                    if (!strncmp(line, "@CAL,", 5)) handle_calibration_command(line);
                    else uart_calibration_reply(false, "command");
                    used = 0;
                }
            } else if (used + 1 < sizeof(line)) {
                line[used++] = ch;
            } else {
                used = 0;
                uart_calibration_reply(false, "length");
            }
        }
    }
}

esp_err_t scan_uart_init(void)
{
    scan_pose_init();
    uart_config_t config = {
        .baud_rate = SCAN_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(SCAN_UART_PORT, &config);
    if (err != ESP_OK) return err;
    err = uart_set_pin(SCAN_UART_PORT, SCAN_UART_TX, SCAN_UART_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;
    err = uart_driver_install(SCAN_UART_PORT, 256, 2048, 0, NULL, 0);
    if (err != ESP_OK) return err;
    s_tx_lock = xSemaphoreCreateMutex();
    if (!s_tx_lock) return ESP_ERR_NO_MEM;
    s_packets = xQueueCreate(1, sizeof(scan_uart_packet_t));
    if (!s_packets) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(scan_uart_task, "scan_uart", 4096, NULL, 7, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(scan_uart_rx_task, "scan_uart_rx", 4096, NULL, 5, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "UART1 TX=GPIO41 RX=GPIO42 921600 8N1; commands: @CAL,GET / @CAL,SET,...");
    return ESP_OK;
}

bool scan_uart_submit(const XcrBarcodeResult *result, uint64_t deadline_us)
{
    if (!s_packets || !result || !result->geometry_valid ||
        (result->symbology == XCR_SYM_DATAMATRIX && result->geometry_quality != 2) ||
        result->length <= 0 || result->length >= XCR_BARCODE_MAX_LEN) return false;
    if (deadline_us && (uint64_t)esp_timer_get_time() >= deadline_us) return false;

    scan_uart_packet_t packet = {.deadline_us = deadline_us};
    bool quoted = memchr(result->data, ',', result->length) ||
                  memchr(result->data, '"', result->length);
    if (quoted) packet.line[packet.length++] = '"';
    static const char hex[] = "0123456789ABCDEF";
    for (int index = 0; index < result->length; ++index) {
        uint8_t value = (uint8_t)result->data[index];
        if (value < 32 || value == 127) {
            packet.line[packet.length++] = '\\';
            packet.line[packet.length++] = 'x';
            packet.line[packet.length++] = hex[value >> 4];
            packet.line[packet.length++] = hex[value & 15];
        } else {
            if (value == '"' || value == '\\') packet.line[packet.length++] = value;
            packet.line[packet.length++] = value;
        }
    }
    if (quoted) packet.line[packet.length++] = '"';

    bool precise = result->subpixel_valid && result->geometry_quality == 2;
    for (int index = 0; index < 8; ++index) {
        int coordinate;
        if (precise) {
            float value = result->subpixel_corners[index];
            if (!isfinite(value) || value < 0 ||
                value >= (index & 1 ? P4SCAN_CAMERA_DEFAULT_HEIGHT : P4SCAN_CAMERA_DEFAULT_WIDTH))
                return false;
            coordinate = (int)lroundf(value * 10.0f);
        } else {
            coordinate = result->corners[index] * 10;
        }
        int written = snprintf(packet.line + packet.length, sizeof(packet.line) - packet.length,
                               ",%d", coordinate);
        if (written < 0 || (size_t)written >= sizeof(packet.line) - packet.length) return false;
        packet.length += written;
    }
    int written = snprintf(packet.line + packet.length, sizeof(packet.line) - packet.length,
                           ",%lu", (unsigned long)result->sequence);
    if (written < 0 || (size_t)written >= sizeof(packet.line) - packet.length) return false;
    packet.length += written;

    scan_pose_result_t pose;
    if (scan_pose_estimate(result, &pose)) {
        written = snprintf(packet.line + packet.length, sizeof(packet.line) - packet.length,
            ",%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
            pose.yaw_y_deg, pose.pitch_x_deg, pose.roll_z_deg, pose.range_m,
            pose.optical_depth_m, pose.distance_to_plane_m, pose.tx_m,
            pose.along_track_m);
    } else {
        written = snprintf(packet.line + packet.length, sizeof(packet.line) - packet.length,
                           ",,,,,,,,\r\n");
    }
    if (written < 0 || (size_t)written >= sizeof(packet.line) - packet.length) return false;
    packet.length += written;
    return xQueueOverwrite(s_packets, &packet) == pdTRUE;
}
