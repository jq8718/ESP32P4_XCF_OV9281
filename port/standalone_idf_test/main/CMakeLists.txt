idf_component_register(
    SRCS "camera_main.c" "scan_log_buffer.c" "scan_usb.c" "agv_pipeline.c"
    PRIV_INCLUDE_DIRS "../../components/xcr_decoder/vendor/include"
                      "../../components/xcr_decoder/vendor/quirc"
    REQUIRES p4scan_camera_capture p4scan_board_power p4scan_feedback
             xcr_decoder esp_timer esp_psram esp_partition esp_driver_gpio
             usb_device_uvc espressif__tinyusb esp_driver_jpeg)
