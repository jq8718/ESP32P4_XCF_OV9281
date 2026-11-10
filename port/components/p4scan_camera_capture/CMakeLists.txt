idf_component_register(
    SRCS "p4scan_camera_capture.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_video esp_driver_i2c esp_driver_gpio esp_mm esp_timer xcr_decoder p4scan_board_power
)
