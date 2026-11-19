if(P4SCAN_REPLAY_BUNDLE)
    set(replay_bundle "${CMAKE_CURRENT_BINARY_DIR}/p4scan_replay.bin")
    configure_file("${P4SCAN_REPLAY_BUNDLE}" "${replay_bundle}" COPYONLY)
    idf_component_register(
        SRCS "../../../tools/p4scan_replay.c"
        PRIV_INCLUDE_DIRS "../../components/xcr_decoder/vendor/include"
        REQUIRES xcr_decoder esp_driver_jpeg esp_timer
        EMBED_FILES "${replay_bundle}")
    return()
endif()

idf_component_register(
    SRCS "scan_uart.c" "scan_pose.c" "camera_main.c" "scan_log_buffer.c" "scan_usb.c" "agv_pipeline.c"
    PRIV_INCLUDE_DIRS "../../components/xcr_decoder/vendor/include"
                      "../../components/xcr_decoder/vendor/quirc"
    REQUIRES p4scan_camera_capture p4scan_board_power p4scan_feedback
             xcr_decoder esp_timer esp_psram esp_partition esp_driver_gpio esp_driver_uart nvs_flash
             usb_device_uvc espressif__tinyusb esp_driver_jpeg)
