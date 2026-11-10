#include "p4scan_board_power.h"
#include "esp_log.h"

#define OV9281_SCCB_ADDR_7BIT 0x60
static const char *TAG = "p4scan_power";

/* This OV9281 board is enabled by GPIO0 in p4scan_camera_init().
 * It has no PCA9538A or software-controlled aiming/illumination outputs. */
esp_err_t p4scan_ov9281_set_lights(i2c_master_bus_handle_t bus, bool aiming, bool illumination)
{
    (void)bus; (void)aiming; (void)illumination;
    return ESP_OK;
}
esp_err_t p4scan_ov9281_run_light_diagnostic(i2c_master_bus_handle_t bus)
{
    (void)bus;
    ESP_LOGW(TAG, "External light control is not fitted on this OV9281 board");
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t p4scan_ov9281_power_on(i2c_master_bus_handle_t bus)
{
    (void)bus;
    ESP_LOGI(TAG, "OV9281 enabled by GPIO0; no I2C power expander");
    return ESP_OK;
}
esp_err_t p4scan_ov9281_power_off(i2c_master_bus_handle_t bus)
{
    (void)bus;
    return ESP_OK;
}

static esp_err_t p4scan_read_a16v8(i2c_master_dev_handle_t dev_handle, uint16_t reg, uint8_t *value)
{
    uint8_t address[] = {(uint8_t)(reg >> 8), (uint8_t)reg};
    return i2c_master_transmit_receive(dev_handle, address, sizeof(address), value, 1, 100);
}

void p4scan_ov9281_log_sensor_state(i2c_master_bus_handle_t bus_handle)
{
    static const uint16_t regs[] = {0x0100, 0x3018, 0x3662, 0x380e, 0x380f, 0x4800, 0x4837};
    i2c_master_dev_handle_t dev_handle = NULL;
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OV9281_SCCB_ADDR_7BIT,
        .scl_speed_hz = 100000,
    };
    if (!bus_handle || i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "OV9281 state: cannot open SCCB device");
        return;
    }
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
        uint8_t value = 0;
        esp_err_t error = p4scan_read_a16v8(dev_handle, regs[i], &value);
        if (error == ESP_OK) ESP_LOGI(TAG, "OV9281 reg %04x=%02x", regs[i], value);
        else ESP_LOGW(TAG, "OV9281 reg %04x read failed: %s", regs[i], esp_err_to_name(error));
    }
    i2c_master_bus_rm_device(dev_handle);
}

