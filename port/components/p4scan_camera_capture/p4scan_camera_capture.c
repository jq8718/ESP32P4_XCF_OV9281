#include "p4scan_camera_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "esp_cache.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/mipi_csi_host_struct.h"
#include "soc/mipi_csi_bridge_struct.h"
#include "soc/isp_struct.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_video_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"
#include "linux/v4l2-controls.h"
#include "p4scan_board_power.h"

#define TAG "p4scan_camera"
#define CACHE_LINE_SIZE 64
#define I2C_SDA_GPIO 7
#define I2C_SCL_GPIO 8
#define CAMERA_PIXEL_FORMAT V4L2_PIX_FMT_SBGGR8
#define P4SCAN_INITIAL_EXPOSURE 681
#define P4SCAN_CAMERA_TLINE_NS 10989U
#if CONFIG_P4SCAN_OV9281_FIXED_GAIN > 0
#define P4SCAN_INITIAL_GAIN_INDEX ((CONFIG_P4SCAN_OV9281_FIXED_GAIN - 1) * 64)
#else
#define P4SCAN_INITIAL_GAIN_INDEX 0
#endif
#define P4SCAN_LIGHT_SETTLE_DELAY_MS 5
#define P4SCAN_LIGHT_SETTLE_FRAMES 2

static esp_err_t cache_sync_aligned(void *address, size_t length, uint32_t flags)
{
    uintptr_t start = (uintptr_t)address & ~(uintptr_t)(CACHE_LINE_SIZE - 1);
    uintptr_t end = ((uintptr_t)address + length + CACHE_LINE_SIZE - 1) &
                    ~(uintptr_t)(CACHE_LINE_SIZE - 1);
    return esp_cache_msync((void *)start, end - start, flags);
}

static esp_err_t p4scan_camera_enable_lights(p4scan_camera_t *camera)
{
    esp_err_t ret = p4scan_ov9281_set_lights(camera->i2c_bus, true, true);
    if (ret != ESP_OK) {
        return ret;
    }

    camera->lights_enabled = true;
    camera->lights_expired = false;
    return ret;
}

static esp_err_t p4scan_camera_read_control(p4scan_camera_t *camera, uint32_t id, uint32_t control_class, int *value)
{
    struct v4l2_ext_control control = {.id = id};
    struct v4l2_ext_controls controls = {.ctrl_class = control_class, .count = 1, .controls = &control};
    if (ioctl(camera->fd, VIDIOC_G_EXT_CTRLS, &controls) != 0) {
        return ESP_FAIL;
    }
    *value = control.value;
    return ESP_OK;
}

void p4scan_camera_snapshot_exposure(p4scan_camera_t *camera)
{
    int gain = -1;
    camera->snapshot_valid =
        p4scan_camera_read_control(camera, V4L2_CID_EXPOSURE, V4L2_CID_CAMERA_CLASS,
                                  &camera->snapshot_exposure) == ESP_OK &&
        p4scan_camera_read_control(camera, V4L2_CID_GAIN, V4L2_CID_USER_CLASS, &gain) == ESP_OK &&
        camera->snapshot_exposure > 0 && camera->snapshot_exposure <= 65535 && gain == P4SCAN_INITIAL_GAIN_INDEX;
    ESP_LOGI(TAG, "capture exposure snapshot=%d gain_index=%d valid=%d",
             camera->snapshot_exposure, gain, camera->snapshot_valid);
}

void p4scan_camera_remember_exposure(p4scan_camera_t *camera)
{
    if (camera->snapshot_valid) {
        camera->successful_exposure = camera->snapshot_exposure;
        ESP_LOGI(TAG, "successful exposure cached=%d gain_index=%d", camera->successful_exposure, P4SCAN_INITIAL_GAIN_INDEX);
    }
}

void p4scan_camera_adjust_search_exposure(p4scan_camera_t *camera, uint32_t mean_brightness)
{
#if CONFIG_P4SCAN_OV9281_AE_MAX_EXPOSURE_US > 0
    int64_t now = esp_timer_get_time();
    if (mean_brightness > 255 || now < camera->ae_next_adjust_us) return;
    camera->ae_next_adjust_us = now + 80000;
    if (mean_brightness >= 60 && mean_brightness <= 70) return;
    int current_exposure = 0, current_gain = 0;
    if (p4scan_camera_read_control(camera, V4L2_CID_EXPOSURE, V4L2_CID_CAMERA_CLASS, &current_exposure) != ESP_OK ||
        p4scan_camera_read_control(camera, V4L2_CID_GAIN, V4L2_CID_USER_CLASS, &current_gain) != ESP_OK ||
        current_exposure < 1 || current_gain < 0) return;
    uint32_t current_product = (uint32_t)current_exposure * (current_gain + 64U);
    uint32_t target_product = current_product * 65U / (mean_brightness ? mean_brightness : 1U);
    if (target_product < current_product * 3U / 4U) target_product = current_product * 3U / 4U;
    if (target_product > current_product * 5U / 4U) target_product = current_product * 5U / 4U;
    uint32_t maximum_exposure = CONFIG_P4SCAN_OV9281_AE_MAX_EXPOSURE_US * 1000U / P4SCAN_CAMERA_TLINE_NS;
    if (!maximum_exposure) maximum_exposure = 1;
    uint32_t next_exposure = target_product / 64U;
    if (next_exposure < 1) next_exposure = 1;
    if (next_exposure > maximum_exposure) next_exposure = maximum_exposure;
    uint32_t next_gain = (target_product + next_exposure / 2U) / next_exposure;
    if (next_gain < 64U) next_gain = 64U;
    if (next_gain > CONFIG_P4SCAN_OV9281_AE_MAX_GAIN * 64U) next_gain = CONFIG_P4SCAN_OV9281_AE_MAX_GAIN * 64U;
    if (current_exposure == next_exposure && current_gain == next_gain - 64U) return;
    struct v4l2_ext_control ae_control = {.id = V4L2_CID_EXPOSURE, .value = next_exposure};
    struct v4l2_ext_controls ae_controls = {.ctrl_class = V4L2_CID_CAMERA_CLASS, .count = 1, .controls = &ae_control};
    if (current_exposure != next_exposure && ioctl(camera->fd, VIDIOC_S_EXT_CTRLS, &ae_controls) != 0) {
        ESP_LOGW("p4scan_ae", "exposure update failed: errno=%d", errno);
        return;
    }
    ae_control.id = V4L2_CID_GAIN;
    ae_control.value = next_gain - 64U;
    ae_controls.ctrl_class = V4L2_CID_USER_CLASS;
    if (current_gain != next_gain - 64U && ioctl(camera->fd, VIDIOC_S_EXT_CTRLS, &ae_controls) != 0) {
        ESP_LOGW("p4scan_ae", "gain update failed: errno=%d", errno);
        return;
    }
    ESP_LOGI("p4scan_ae", "mean=%lu target=65 exposure=%lu nominal_us=%lu gain_raw=%lu/64 cap_us=%u cap_gain=%u",
             (unsigned long)mean_brightness, (unsigned long)next_exposure,
             (unsigned long)(next_exposure * P4SCAN_CAMERA_TLINE_NS / 1000U), (unsigned long)next_gain,
             CONFIG_P4SCAN_OV9281_AE_MAX_EXPOSURE_US, CONFIG_P4SCAN_OV9281_AE_MAX_GAIN);
    return;
#endif
    if (!camera->snapshot_valid || mean_brightness > 255) {
        return;
    }
    uint32_t exposure = (uint32_t)camera->snapshot_exposure;
    uint32_t requested = exposure;
    if (mean_brightness < 55 || mean_brightness > 80) {
        requested = exposure * 65U / (mean_brightness ? mean_brightness : 1U);
        if (requested < exposure / 2U) requested = exposure / 2U;
        if (requested > exposure * 2U) requested = exposure * 2U;
    }
    if (requested < 16U) requested = 16U;
    if (requested > 1795U) requested = 1795U;
    camera->search_exposure = (int)requested;
    ESP_LOGI(TAG, "continuous exposure mean=%lu captured=%lu next=%lu",
             (unsigned long)mean_brightness, (unsigned long)exposure, (unsigned long)requested);
}

static esp_err_t p4scan_camera_set_capture_defaults(p4scan_camera_t *camera)
{
    /* Continuous scan retains the reference sensor exposure/gain settings. */
    if (camera->manual_lights) {
        return ESP_OK;
    }
#if CONFIG_P4SCAN_OV9281_AE_MAX_EXPOSURE_US > 0
    return ESP_OK;
#endif
    struct v4l2_ext_control control = {0};
    struct v4l2_ext_controls controls = {0};

    control.id = V4L2_CID_EXPOSURE;
    int requested_exposure = camera->successful_exposure > 0 ? camera->successful_exposure : P4SCAN_INITIAL_EXPOSURE;
    if (camera->search_exposure > 0) {
        requested_exposure = camera->search_exposure;
    }
#if CONFIG_P4SCAN_OV9281_FIXED_EXPOSURE_US > 0
    requested_exposure = CONFIG_P4SCAN_OV9281_FIXED_EXPOSURE_US * 1000U / P4SCAN_CAMERA_TLINE_NS;
    if (requested_exposure < 1) requested_exposure = 1;
#endif
    control.value = requested_exposure;
    controls.ctrl_class = V4L2_CID_CAMERA_CLASS;
    controls.count = 1;
    controls.controls = &control;
    int actual_exposure = 0;
    ESP_RETURN_ON_ERROR(p4scan_camera_read_control(camera, V4L2_CID_EXPOSURE,
                         V4L2_CID_CAMERA_CLASS, &actual_exposure), TAG, "exposure read failed");
    for (int attempt = 0; attempt < 16 && actual_exposure != requested_exposure; attempt++) {
        control.value = requested_exposure;
        if (ioctl(camera->fd, VIDIOC_S_EXT_CTRLS, &controls) != 0 ||
            p4scan_camera_read_control(camera, V4L2_CID_EXPOSURE, V4L2_CID_CAMERA_CLASS, &actual_exposure) != ESP_OK) {
            return ESP_FAIL;
        }
        if (actual_exposure == requested_exposure) {
            break;
        }
    }
    ESP_RETURN_ON_FALSE(actual_exposure == requested_exposure, ESP_FAIL, TAG, "exposure seed restore incomplete");

    memset(&control, 0, sizeof(control));
    memset(&controls, 0, sizeof(controls));
    control.id = V4L2_CID_GAIN;
    control.value = P4SCAN_INITIAL_GAIN_INDEX;
    controls.ctrl_class = V4L2_CID_USER_CLASS;
    controls.count = 1;
    controls.controls = &control;
    int actual_gain = -1;
    ESP_RETURN_ON_ERROR(p4scan_camera_read_control(camera, V4L2_CID_GAIN,
                         V4L2_CID_USER_CLASS, &actual_gain), TAG, "gain read failed");
    if (actual_gain != P4SCAN_INITIAL_GAIN_INDEX && ioctl(camera->fd, VIDIOC_S_EXT_CTRLS, &controls) != 0) {
        ESP_LOGE(TAG, "failed to set capture gain index=%d, errno=%d",
                 P4SCAN_INITIAL_GAIN_INDEX, errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "capture seed applied before light: exposure=%d source=%s gain_index=%d; ISP AE remains enabled",
             actual_exposure, camera->search_exposure > 0 ? "continuous" :
             camera->successful_exposure > 0 ? "last_success" : "default", P4SCAN_INITIAL_GAIN_INDEX);
    return ESP_OK;
}

static void close_capture_buffers(p4scan_camera_t *camera)
{
    for (int index = 0; index < P4SCAN_CAMERA_BUFFER_COUNT; index++) {
        if (camera->capture[index].address) {
            munmap(camera->capture[index].address, camera->capture[index].length);
            camera->capture[index].address = NULL;
            camera->capture[index].length = 0;
        }
    }
}

void p4scan_camera_config_default(p4scan_camera_config_t *config)
{
    if (!config) {
        return;
    }
    config->width = P4SCAN_CAMERA_DEFAULT_WIDTH;
    config->height = P4SCAN_CAMERA_DEFAULT_HEIGHT;
    config->frame_rate = P4SCAN_CAMERA_DEFAULT_FPS;
    config->i2c_frequency = 100000;
}

esp_err_t p4scan_camera_init(p4scan_camera_t *camera,
                             const p4scan_camera_config_t *config)
{
    p4scan_camera_config_t effective;
    i2c_master_bus_config_t bus_config;
    esp_video_init_csi_config_t csi_config;
    esp_video_init_config_t video_config;
    esp_err_t ret;

    ESP_RETURN_ON_FALSE(camera, ESP_ERR_INVALID_ARG, TAG, "camera is NULL");
    if (config) {
        effective = *config;
    } else {
        p4scan_camera_config_default(&effective);
    }
    ESP_RETURN_ON_FALSE(effective.width == P4SCAN_CAMERA_DEFAULT_WIDTH &&
                        effective.height == P4SCAN_CAMERA_DEFAULT_HEIGHT,
                        ESP_ERR_NOT_SUPPORTED, TAG,
                        "OV9281 mode must be 1280x720");
    ESP_RETURN_ON_FALSE(effective.frame_rate == P4SCAN_CAMERA_DEFAULT_FPS && effective.i2c_frequency > 0,
                        ESP_ERR_INVALID_ARG, TAG, "invalid camera configuration");

    memset(camera, 0, sizeof(*camera));
    camera->fd = -1;
    camera->held_buffer_index = -1;
    camera->width = effective.width;
    camera->height = effective.height;
    camera->frame_rate = effective.frame_rate;

    /* The OV9281 UVC reference enables the camera board on GPIO0. */
    gpio_config_t camera_enable = {
        .pin_bit_mask = 1ULL << 0,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_GOTO_ON_ERROR(gpio_config(&camera_enable), fail, TAG, "camera enable GPIO configuration failed");
    ESP_GOTO_ON_ERROR(gpio_set_level(0, 1), fail, TAG, "camera enable GPIO failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    bus_config = (i2c_master_bus_config_t) {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_GOTO_ON_ERROR(i2c_new_master_bus(&bus_config, &camera->i2c_bus), fail,
                      TAG, "failed to initialize I2C0");
    ESP_GOTO_ON_ERROR(p4scan_ov9281_power_on(camera->i2c_bus), fail,
                      TAG, "failed to power OV9281");

    csi_config = (esp_video_init_csi_config_t) {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = camera->i2c_bus,
            .freq = effective.i2c_frequency,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        .dont_init_ldo = false,
    };
    video_config = (esp_video_init_config_t) {
        .csi = &csi_config,
    };
    ESP_GOTO_ON_ERROR(esp_video_init(&video_config), fail,
                      TAG, "failed to initialize CSI video");
    camera->video_initialized = true;
    ESP_LOGI(TAG, "OV9281 video initialized: %ux%u RAW8 @ %lu fps",
             (unsigned)camera->width, (unsigned)camera->height,
             (unsigned long)camera->frame_rate);
    return ESP_OK;

fail:
    if (camera->video_initialized) {
        esp_video_deinit();
        camera->video_initialized = false;
    }
    if (camera->i2c_bus) {
        p4scan_ov9281_power_off(camera->i2c_bus);
        i2c_del_master_bus(camera->i2c_bus);
        camera->i2c_bus = NULL;
    }
    return ret;
}

esp_err_t p4scan_camera_start(p4scan_camera_t *camera)
{
    struct v4l2_format format;
    struct v4l2_requestbuffers request;
    struct v4l2_buffer buffer;
    esp_err_t ret = ESP_FAIL;

    ESP_RETURN_ON_FALSE(camera && camera->video_initialized, ESP_ERR_INVALID_STATE,
                        TAG, "camera video is not initialized");
    if (camera->streaming) {
        return ESP_OK;
    }

    camera->fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    ESP_RETURN_ON_FALSE(camera->fd >= 0, ESP_FAIL, TAG,
                        "open %s failed, errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = camera->width;
    format.fmt.pix.height = camera->height;
    format.fmt.pix.pixelformat = CAMERA_PIXEL_FORMAT;
    if (ioctl(camera->fd, VIDIOC_S_FMT, &format) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed, errno=%d", errno);
        ret = ESP_FAIL;
        goto fail;
    }
    if (format.fmt.pix.pixelformat != CAMERA_PIXEL_FORMAT ||
        format.fmt.pix.width != camera->width ||
        format.fmt.pix.height != camera->height) {
        ESP_LOGE(TAG, "unexpected format: %ux%u fourcc=0x%08lx",
                 (unsigned)format.fmt.pix.width, (unsigned)format.fmt.pix.height,
                 (unsigned long)format.fmt.pix.pixelformat);
        ret = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }
    camera->stride = format.fmt.pix.bytesperline ? format.fmt.pix.bytesperline : camera->width;
    camera->pixel_format = format.fmt.pix.pixelformat;
    ESP_LOGI(TAG, "capture format: %ux%u RAW8 stride=%lu sizeimage=%lu",
             (unsigned)camera->width, (unsigned)camera->height,
             (unsigned long)camera->stride,
             (unsigned long)format.fmt.pix.sizeimage);

    memset(&request, 0, sizeof(request));
    request.count = P4SCAN_CAMERA_BUFFER_COUNT;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (ioctl(camera->fd, VIDIOC_REQBUFS, &request) != 0 ||
        request.count < P4SCAN_CAMERA_BUFFER_COUNT) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed/count=%u errno=%d",
                 (unsigned)request.count, errno);
        ret = ESP_FAIL;
        goto fail;
    }

    for (int index = 0; index < P4SCAN_CAMERA_BUFFER_COUNT; index++) {
        memset(&buffer, 0, sizeof(buffer));
        buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        buffer.index = index;
        if (ioctl(camera->fd, VIDIOC_QUERYBUF, &buffer) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF[%d] failed, errno=%d", index, errno);
            ret = ESP_FAIL;
            goto fail;
        }
        camera->capture[index].address = mmap(NULL, buffer.length,
                                               PROT_READ | PROT_WRITE, MAP_SHARED,
                                               camera->fd, buffer.m.offset);
        if (camera->capture[index].address == MAP_FAILED) {
            camera->capture[index].address = NULL;
            ESP_LOGE(TAG, "mmap[%d] failed, errno=%d", index, errno);
            ret = ESP_ERR_NO_MEM;
            goto fail;
        }
        camera->capture[index].length = buffer.length;
        memset(camera->capture[index].address, 0, buffer.length);
        ESP_GOTO_ON_ERROR(cache_sync_aligned(camera->capture[index].address,
                                             buffer.length,
                                             ESP_CACHE_MSYNC_FLAG_DIR_C2M),
                          fail, TAG, "initial cache sync failed");
        if (ioctl(camera->fd, VIDIOC_QBUF, &buffer) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed, errno=%d", index, errno);
            ret = ESP_FAIL;
            goto fail;
        }
    }

    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(camera->fd, VIDIOC_STREAMON, &type) != 0) {
            ESP_LOGE(TAG, "VIDIOC_STREAMON failed, errno=%d", errno);
            ret = ESP_FAIL;
            goto fail;
        }
    }
    camera->streaming = true;
    camera->lights_enabled = false;
    camera->lights_expired = true;
    camera->frame_sequence = 0;
    ESP_LOGI(TAG, "CSI stream started with %d MMAP buffers", P4SCAN_CAMERA_BUFFER_COUNT);
    return ESP_OK;

fail:
    if (camera->lights_enabled) {
        p4scan_ov9281_set_lights(camera->i2c_bus, false, false);
        camera->lights_enabled = false;
    }
    if (camera->fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(camera->fd, VIDIOC_STREAMOFF, &type);
        memset(&request, 0, sizeof(request));
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;
        ioctl(camera->fd, VIDIOC_REQBUFS, &request);
        close_capture_buffers(camera);
        close(camera->fd);
        camera->fd = -1;
    }
    return ret;
}

static esp_err_t wait_or_prepare_frame(p4scan_camera_t *camera, uint32_t timeout_ms,
                                       int64_t retain_after)
{
    struct v4l2_buffer buffer;
    struct timeval dequeue_timeout;

    ESP_RETURN_ON_FALSE(camera && camera->streaming, ESP_ERR_INVALID_STATE,
                        TAG, "camera is not streaming");
    ESP_RETURN_ON_FALSE(camera->held_buffer_index < 0, ESP_ERR_INVALID_STATE,
                        TAG, "previous frame has not been released");
    ESP_RETURN_ON_FALSE(!camera->prepared_frame_valid, ESP_ERR_INVALID_STATE,
                        TAG, "prepared frame has not been consumed");
    dequeue_timeout.tv_sec = timeout_ms / 1000;
    dequeue_timeout.tv_usec = (timeout_ms % 1000) * 1000;
    if (ioctl(camera->fd, VIDIOC_S_DQBUF_TIMEOUT, &dequeue_timeout) != 0) {
        ESP_LOGE(TAG, "frame signal wait timeout setup failed, errno=%d", errno);
        return ESP_FAIL;
    }

    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (ioctl(camera->fd, VIDIOC_DQBUF, &buffer) != 0) {
        ESP_LOGW(TAG, "frame signal wait failed, errno=%d", errno);
        ESP_LOGW(TAG, "CSI PHY lanes=%lu rx=%08lx stop=%08lx irq=%08lx phy_fatal=%08lx pkt=%08lx phy=%08lx",
                 (unsigned long)MIPI_CSI_HOST.n_lanes.val,
                 (unsigned long)MIPI_CSI_HOST.phy_rx.val,
                 (unsigned long)MIPI_CSI_HOST.phy_stopstate.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_main.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_phy_fatal.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_pkt_fatal.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_phy.val);
        ESP_LOGW(TAG, "CSI frame_seq=%08lx frame_crc=%08lx payload_crc=%08lx data_id=%08lx",
                 (unsigned long)MIPI_CSI_HOST.int_st_seq_frame_fatal.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_crc_frame_fatal.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_pld_crc_fatal.val,
                 (unsigned long)MIPI_CSI_HOST.int_st_data_id.val);
        ESP_LOGW(TAG, "CSI bridge frame=%08lx irq=%08lx buf=%08lx ISP frame=%08lx ctrl=%08lx",
                 (unsigned long)MIPI_CSI_BRIDGE.frame_cfg.val,
                 (unsigned long)MIPI_CSI_BRIDGE.int_raw.val,
                 (unsigned long)MIPI_CSI_BRIDGE.buf_flow_ctl.val,
                 (unsigned long)ISP.frame_cfg.val, (unsigned long)ISP.cntl.val);
        return errno == EAGAIN ? ESP_ERR_TIMEOUT : ESP_FAIL;
    }
    if (buffer.index >= P4SCAN_CAMERA_BUFFER_COUNT ||
        !camera->capture[buffer.index].address) {
        ESP_LOGE(TAG, "frame signal wait returned invalid buffer index=%u",
                 (unsigned)buffer.index);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (retain_after > 0 && esp_timer_get_time() >= retain_after) {
        camera->prepared_buffer_index = buffer.index;
        camera->prepared_bytesused = buffer.bytesused;
        camera->prepared_timestamp_us = (uint64_t)buffer.timestamp.tv_sec * 1000000ULL + buffer.timestamp.tv_usec;
        camera->prepared_frame_valid = true;
        return ESP_OK;
    }

    struct v4l2_buffer release_buffer = {0};
    release_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    release_buffer.memory = V4L2_MEMORY_MMAP;
    release_buffer.index = buffer.index;
    if (ioctl(camera->fd, VIDIOC_QBUF, &release_buffer) != 0) {
        ESP_LOGE(TAG, "frame signal wait QBUF[%u] failed, errno=%d",
                 (unsigned)buffer.index, errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "frame signal confirmed: sequence=%u buffer=%u bytes=%u",
             (unsigned)buffer.sequence, (unsigned)buffer.index,
             (unsigned)buffer.bytesused);
    return ESP_OK;
}

esp_err_t p4scan_camera_wait_for_frame(p4scan_camera_t *camera, uint32_t timeout_ms)
{
    return wait_or_prepare_frame(camera, timeout_ms, 0);
}

esp_err_t p4scan_camera_begin_capture(p4scan_camera_t *camera)
{
    ESP_RETURN_ON_FALSE(camera && camera->streaming, ESP_ERR_INVALID_STATE,
                        TAG, "camera is not streaming");
    if (camera->manual_lights) return p4scan_camera_set_capture_defaults(camera);
    if (camera->lights_enabled) {
        return ESP_OK;
    }

    int64_t frame_check_start = esp_timer_get_time();
    ESP_RETURN_ON_ERROR(p4scan_camera_wait_for_frame(camera, 1000), TAG,
                        "frame signal is not available before light enable");
    int64_t defaults_start = esp_timer_get_time();
    ESP_RETURN_ON_ERROR(p4scan_camera_set_capture_defaults(camera), TAG,
                        "failed to apply capture exposure/gain defaults");
    int64_t light_start = esp_timer_get_time();
    ESP_RETURN_ON_ERROR(p4scan_camera_enable_lights(camera), TAG,
                        "failed to enable capture lights");
    int64_t light_ready = esp_timer_get_time();
    int64_t output_ready = light_ready + P4SCAN_LIGHT_SETTLE_DELAY_MS * 1000LL;
    while (esp_timer_get_time() < output_ready) {
        vTaskDelay(1);
    }
    int64_t settle_deadline = output_ready + P4SCAN_LIGHT_SETTLE_FRAMES * 1000000LL / camera->frame_rate;
    int discarded = 0;
    do {
        ESP_RETURN_ON_ERROR(wait_or_prepare_frame(camera, 1000,
                            discarded >= P4SCAN_LIGHT_SETTLE_FRAMES - 1 ? settle_deadline : 0), TAG,
                            "failed to discard light-settle frame");
        if (!camera->prepared_frame_valid) {
            discarded++;
        }
    } while (!camera->prepared_frame_valid);
    ESP_LOGI(TAG, "capture prepare_us: frame_check=%lld exposure_seed=%lld light_on=%lld settle=%lld discarded=%d retained=1",
             (long long)(defaults_start - frame_check_start), (long long)(light_start - defaults_start),
             (long long)(light_ready - light_start), (long long)(esp_timer_get_time() - light_ready), discarded);
    return ESP_OK;
}

esp_err_t p4scan_camera_acquire(p4scan_camera_t *camera,
                                p4scan_camera_frame_t *frame,
                                uint32_t timeout_ms)
{
    struct v4l2_buffer buffer;
    struct timeval dequeue_timeout;
    size_t required;
    uint8_t *source;
    uint32_t source_stride;
    uint64_t dequeue_start_us;
    uint64_t dequeue_done_us;
    esp_err_t ret;

    ESP_RETURN_ON_FALSE(camera && frame && camera->streaming, ESP_ERR_INVALID_STATE,
                        TAG, "camera is not streaming");
    ESP_RETURN_ON_FALSE(camera->pooled_frames || camera->held_buffer_index < 0, ESP_ERR_INVALID_STATE, TAG,
                        "previous frame has not been released");
    ESP_RETURN_ON_FALSE(camera->manual_lights || camera->lights_enabled, ESP_ERR_INVALID_STATE, TAG,
                        "capture lights are not enabled");
    dequeue_timeout.tv_sec = timeout_ms / 1000;
    dequeue_timeout.tv_usec = (timeout_ms % 1000) * 1000;
    if (ioctl(camera->fd, VIDIOC_S_DQBUF_TIMEOUT, &dequeue_timeout) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_DQBUF_TIMEOUT failed, errno=%d", errno);
        return ESP_FAIL;
    }

    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    dequeue_start_us = esp_timer_get_time();
    if (camera->prepared_frame_valid) {
        buffer.index = camera->prepared_buffer_index;
        buffer.bytesused = camera->prepared_bytesused;
        buffer.timestamp.tv_sec = camera->prepared_timestamp_us / 1000000ULL;
        buffer.timestamp.tv_usec = camera->prepared_timestamp_us % 1000000ULL;
        camera->prepared_frame_valid = false;
    } else if (ioctl(camera->fd, VIDIOC_DQBUF, &buffer) != 0) {
        frame->dequeue_us = esp_timer_get_time() - dequeue_start_us;
        if (errno == EAGAIN || errno == ETIMEDOUT) {
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGE(TAG, "VIDIOC_DQBUF failed, errno=%d", errno);
        return ESP_FAIL;
    }
    dequeue_done_us = esp_timer_get_time();
    if (buffer.index >= P4SCAN_CAMERA_BUFFER_COUNT ||
        !camera->capture[buffer.index].address) {
        ESP_LOGE(TAG, "invalid DQBUF index=%u", (unsigned)buffer.index);
        return ESP_ERR_INVALID_RESPONSE;
    }
    camera->held_buffer_index = (int)buffer.index;
    source = camera->capture[buffer.index].address;
    uint64_t sync_start_us = esp_timer_get_time();
    ret = cache_sync_aligned(source, camera->capture[buffer.index].length,
                             ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    uint64_t sync_done_us = esp_timer_get_time();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "frame cache sync failed: %s", esp_err_to_name(ret));
        struct v4l2_buffer release_buffer = {0};
        release_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        release_buffer.memory = V4L2_MEMORY_MMAP;
        release_buffer.index = buffer.index;
        ioctl(camera->fd, VIDIOC_QBUF, &release_buffer);
        camera->held_buffer_index = -1;
        return ret;
    }

    source_stride = camera->stride;
    required = (size_t)source_stride * camera->height;
    if (buffer.bytesused < required) {
        ESP_LOGE(TAG, "short frame: bytesused=%lu required=%lu index=%u",
                 (unsigned long)buffer.bytesused, (unsigned long)required,
                 (unsigned)buffer.index);
        struct v4l2_buffer release_buffer = {0};
        release_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        release_buffer.memory = V4L2_MEMORY_MMAP;
        release_buffer.index = buffer.index;
        ioctl(camera->fd, VIDIOC_QBUF, &release_buffer);
        camera->held_buffer_index = -1;
        return ESP_ERR_INVALID_SIZE;
    }
    camera->frame_sequence++;
    camera->held_buffer_mask |= 1U << buffer.index;
    frame->pixels = source;
    frame->width = camera->width;
    frame->height = camera->height;
    frame->stride = source_stride;
    frame->sequence = camera->frame_sequence;
    frame->timestamp_us = (uint64_t)buffer.timestamp.tv_sec * 1000000ULL + buffer.timestamp.tv_usec;
    frame->buffer_index = camera->held_buffer_index;
    frame->dequeue_us = dequeue_done_us - dequeue_start_us;
    frame->cache_sync_us = sync_done_us - sync_start_us;
    return ESP_OK;
}

esp_err_t p4scan_camera_release(p4scan_camera_t *camera,
                                const p4scan_camera_frame_t *frame)
{
    struct v4l2_buffer buffer;

    ESP_RETURN_ON_FALSE(camera && frame && camera->streaming, ESP_ERR_INVALID_STATE,
                        TAG, "camera is not streaming");
    ESP_RETURN_ON_FALSE(frame->buffer_index >= 0 && frame->buffer_index < P4SCAN_CAMERA_BUFFER_COUNT &&
                        (camera->held_buffer_mask & (1U << frame->buffer_index)) &&
                        (camera->pooled_frames || camera->held_buffer_index == frame->buffer_index),
                        ESP_ERR_INVALID_ARG, TAG, "frame ownership mismatch");

    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = (unsigned)frame->buffer_index;
    if (ioctl(camera->fd, VIDIOC_QBUF, &buffer) != 0) {
        ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed, errno=%d",
                 frame->buffer_index, errno);
        return ESP_FAIL;
    }
    camera->held_buffer_index = -1;
    camera->held_buffer_mask &= ~(1U << frame->buffer_index);
    return ESP_OK;
}

esp_err_t p4scan_camera_end_capture(p4scan_camera_t *camera)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(camera && camera->streaming, ESP_ERR_INVALID_STATE,
                        TAG, "camera is not streaming");
    if (camera->manual_lights) return ESP_OK;
    if (camera->lights_enabled) {
        ret = p4scan_ov9281_set_lights(camera->i2c_bus, false, false);
        camera->lights_enabled = false;
    }
    camera->lights_expired = true;
    if (camera->prepared_frame_valid) {
        struct v4l2_buffer pending = {0};
        pending.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        pending.memory = V4L2_MEMORY_MMAP;
        pending.index = camera->prepared_buffer_index;
        if (ioctl(camera->fd, VIDIOC_QBUF, &pending) != 0) {
            return ESP_FAIL;
        }
        camera->prepared_frame_valid = false;
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "capture lights disabled after button-triggered frame");
    }
    return ret;
}

esp_err_t p4scan_camera_stop(p4scan_camera_t *camera)
{
    struct v4l2_requestbuffers request;

    ESP_RETURN_ON_FALSE(camera, ESP_ERR_INVALID_ARG, TAG, "camera is NULL");
    if (!camera->streaming) {
        return ESP_OK;
    }
    if (camera->held_buffer_index >= 0) {
        ESP_LOGW(TAG, "stopping camera with held buffer=%d", camera->held_buffer_index);
        camera->held_buffer_index = -1;
    }
    if (camera->manual_lights || camera->lights_enabled) {
        p4scan_ov9281_set_lights(camera->i2c_bus, false, false);
        camera->lights_enabled = false;
    }
    camera->lights_expired = true;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(camera->fd, VIDIOC_STREAMOFF, &type);
    memset(&request, 0, sizeof(request));
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    ioctl(camera->fd, VIDIOC_REQBUFS, &request);
    close_capture_buffers(camera);
    close(camera->fd);
    camera->fd = -1;
    camera->streaming = false;
    camera->prepared_frame_valid = false;
    ESP_LOGI(TAG, "CSI stream stopped");
    return ESP_OK;
}

esp_err_t p4scan_camera_deinit(p4scan_camera_t *camera)
{
    ESP_RETURN_ON_FALSE(camera, ESP_ERR_INVALID_ARG, TAG, "camera is NULL");
    p4scan_camera_stop(camera);
    if (camera->video_initialized) {
        esp_video_deinit();
        camera->video_initialized = false;
    }
    if (camera->i2c_bus) {
        p4scan_ov9281_power_off(camera->i2c_bus);
        i2c_del_master_bus(camera->i2c_bus);
        camera->i2c_bus = NULL;
    }
    return ESP_OK;
}

