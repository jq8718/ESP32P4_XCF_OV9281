#include "scan_uart.h"
#include "scan_pose.h"
#include "scan_uart_numeric.h"
#include <inttypes.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
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
#define SCAN_UART_SEQUENCE_MODULUS 1000u
#define SCAN_UART_LINE_CAPACITY (4 * XCR_BARCODE_MAX_LEN + 256)

typedef struct {
    uint16_t length;
    char line[SCAN_UART_LINE_CAPACITY];
    uint64_t deadline_us;
    unsigned generation;
} scan_uart_packet_t;

typedef enum { OUTPUT_COMPACT, OUTPUT_FULL } output_mode_t;
static atomic_int s_output_mode = OUTPUT_COMPACT;
static atomic_uint s_output_generation;

static const char *TAG = "p4scan_uart";
static QueueHandle_t s_packets;
static SemaphoreHandle_t s_tx_lock;

static void uart_send_line(const char *line, size_t length, const unsigned *generation)
{
    if (!line || !length || !s_tx_lock) return;
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "TX busy; response dropped");
        return;
    }
    if (generation && *generation != atomic_load(&s_output_generation)) {
        xSemaphoreGive(s_tx_lock);
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
        uart_send_line(packet.line, packet.length, &packet.generation);
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
    if (length > 0 && length < (int)sizeof(line)) uart_send_line(line, (size_t)length, NULL);
}

/* CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xffff, no reflection/xorout. */
static uint16_t scan_crc16(const char *bytes, size_t length)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)(uint8_t)bytes[i] << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}

static void handle_output_command(const char *command)
{
    int mode = -1;
    if (!strcmp(command, "@OUT,COMPACT")) mode = OUTPUT_COMPACT;
    else if (!strcmp(command, "@OUT,FULL")) mode = OUTPUT_FULL;
    else if (strcmp(command, "@OUT,GET")) {
        const char error[] = "@OUT,ERR,command\r\n";
        uart_send_line(error, sizeof(error)-1, NULL);
        return;
    }
    if (mode >= 0) {
        atomic_store(&s_output_mode, mode);
        atomic_fetch_add(&s_output_generation, 1);
    }
    mode = atomic_load(&s_output_mode);
    const char *name = mode == OUTPUT_FULL ? "FULL" : "COMPACT";
    char reply[40];
    int length = snprintf(reply, sizeof(reply), "@OUT,DATA,%s\r\n", name);
    uart_send_line(reply, (size_t)length, NULL);
}

static void handle_calibration_command(char *line)
{
    size_t length = strlen(line);
    if (!length || strstr(line, ",,") || line[length-1] == ',') {
        uart_calibration_reply(false, "format");
        return;
    }
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
    bool overflow = false;
    for (;;) {
        int count = uart_read_bytes(SCAN_UART_PORT, input, sizeof(input), pdMS_TO_TICKS(100));
        for (int i = 0; i < count; ++i) {
            char ch = (char)input[i];
            if (ch == '\r' || ch == '\n') {
                if (!overflow && used) {
                    line[used] = '\0';
                    if (!strncmp(line, "@CAL,", 5)) handle_calibration_command(line);
                    else if (!strncmp(line, "@OUT,", 5)) handle_output_command(line);
                    else uart_calibration_reply(false, "command");
                }
                used = 0;
                overflow = false;
            } else if (!overflow && used + 1 < sizeof(line)) {
                line[used++] = ch;
            } else if (!overflow) {
                used = 0;
                overflow = true;
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
    ESP_LOGI(TAG, "UART1 TX=GPIO41 RX=GPIO42 921600 8N1; integer CRC16 COMPACT; @OUT,FULL/COMPACT/GET; @CAL,GET/SET");
    return ESP_OK;
}

bool scan_uart_submit(const XcrBarcodeResult *result, uint64_t deadline_us)
{
    if (!s_packets || !result || !result->geometry_valid ||
        (result->symbology == XCR_SYM_DATAMATRIX && result->geometry_quality != 2) ||
        result->length <= 0 || result->length >= XCR_BARCODE_MAX_LEN) return false;
    if (deadline_us && (uint64_t)esp_timer_get_time() >= deadline_us) return false;
    for (int i = 0; i < result->length; ++i) {
        if (result->data[i] < '0' || result->data[i] > '9') return false;
    }

    scan_uart_packet_t packet = {.deadline_us = deadline_us,
                                .generation = atomic_load(&s_output_generation)};
    int mode = atomic_load(&s_output_mode);
    scan_pose_result_t pose = {0};
    bool valid = scan_pose_estimate(result, &pose);
    unsigned sequence = result->sequence % SCAN_UART_SEQUENCE_MODULUS;
    int length = snprintf(packet.line, sizeof(packet.line), "@SCAN,6,%c,%u,%d,%.*s",
                          mode == OUTPUT_FULL ? 'F' : 'C',
                          sequence, valid,
                          (int)result->length, (const char *)result->data);
    if (length < 0 || (size_t)length >= sizeof(packet.line)) return false;
    packet.length = length;
    if (mode == OUTPUT_FULL) {
        for (int i = 0; i < 8; ++i) {
            float value = result->subpixel_valid ? result->subpixel_corners[i] : result->corners[i];
            if (!isfinite(value) || value < 0 ||
                value >= (i & 1 ? P4SCAN_CAMERA_DEFAULT_HEIGHT : P4SCAN_CAMERA_DEFAULT_WIDTH)) return false;
            length = snprintf(packet.line+packet.length, sizeof(packet.line)-packet.length,
                              ",%ld", lroundf(value*10));
            if (length < 0 || (size_t)length >= sizeof(packet.line)-packet.length) return false;
            packet.length += length;
        }
    }
    float values[8] = {pose.yaw_y_deg, pose.pitch_x_deg, pose.roll_z_deg, pose.range_m,
                       pose.optical_depth_m, pose.distance_to_plane_m, pose.tx_m, pose.along_track_m};
    for (int i = 0; i < 8; ++i) {
        if (mode == OUTPUT_COMPACT && i != 0 && i != 5 && i != 7) continue;
        /* Millidegrees for angles; millimetres for distances. */
        int32_t scaled;
        if (!scan_uart_scaled_i32(valid ? values[i] : 0, 1000.0f, &scaled)) return false;
        length = snprintf(packet.line+packet.length, sizeof(packet.line)-packet.length,
                          ",%" PRId32, scaled);
        if (length < 0 || (size_t)length >= sizeof(packet.line)-packet.length) return false;
        packet.length += length;
    }
    uint16_t crc = scan_crc16(packet.line, packet.length);
    length = snprintf(packet.line+packet.length, sizeof(packet.line)-packet.length, "*%04X\r\n", crc);
    if (length < 0 || (size_t)length >= sizeof(packet.line)-packet.length) return false;
    packet.length += length;
    return xQueueOverwrite(s_packets, &packet) == pdTRUE;
}
