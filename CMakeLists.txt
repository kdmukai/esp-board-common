set(SRC_FILES
    "src/board_init.c"
    "src/board_i2c.c"
    "src/board_backlight.c"
    "src/board_pmic.cpp"
    "src/board_display_axs15231b.c"
    "src/board_display_st7796.c"
    "src/board_display_st7789.c"
    "src/board_display_st7701.c"
    "src/board_touch_axs15231b.c"
    "src/board_touch_ft6336.c"
    "src/board_touch_cst816d.c"
    "src/board_touch_gt911.c"
    "src/board_camera.c"
    "src/board_camera_csi.c"
    "src/board_pipeline_camera_csi.c"
    "src/board_pipeline_camera_dvp.c"
    "src/board_pipeline_display_lvgl.c"
    "src/board_pipeline.c"
    "src/board_sdcard.c"
    "src/board_audio.c"
    "src/overlay_text.c"
    "src/overlay_rotated.c"
    "src/scan_coordinator.c"
)

# All dependencies listed unconditionally where possible. The #if BOARD_HAS_*
# guards in C source compile out unused peripheral code when the board config
# doesn't define them. The components just need to be present for include path
# resolution.
#
# Camera components are target-conditional: DVP (esp32-camera) for S3,
# CSI (esp_driver_cam + esp_cam_sensor) for P4.
set(REQUIRED_COMPONENTS
    "esp_lcd" "driver" "esp_adc" "espressif__esp_lvgl_port" "lvgl"
    "espressif__esp_lcd_axs15231b" "espressif__esp_io_expander_tca9554"
    "espressif__esp_lcd_st7796"
    "espressif__esp_lcd_touch_ft5x06" "espressif__esp_lcd_touch_cst816s"
    "espressif__esp_lcd_touch_gt911"
    "XPowersLib"
    "fatfs" "nvs_flash"
    "espressif__esp_codec_dev"
    "waveshare__pcf85063a" "waveshare__qmi8658"
    "esp-camera-pipeline" "cam_pipeline_qr"
)

# MIPI-DSI display (ESP32-P4): ST7701 + LDO regulator
if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND REQUIRED_COMPONENTS "espressif__esp_lcd_st7701")
endif()

# DVP camera (ESP32-S3 and compatible targets)
if(NOT IDF_TARGET STREQUAL "esp32p4")
    list(APPEND REQUIRED_COMPONENTS "espressif__esp32-camera")
endif()

# CSI camera (ESP32-P4): esp_video V4L2 layer handles CSI + ISP internally
if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND REQUIRED_COMPONENTS "esp_driver_cam" "espressif__esp_cam_sensor" "esp_driver_isp" "esp_video")
endif()

idf_component_register(
    SRCS ${SRC_FILES}
    INCLUDE_DIRS "src"
    REQUIRES ${REQUIRED_COMPONENTS}
)

# Board config header: selected at build time via BOARD variable.
# Consumer project sets BOARD_CONFIG_DIR to the directory containing board_config.h.
if(DEFINED BOARD_CONFIG_DIR AND EXISTS "${BOARD_CONFIG_DIR}")
    target_include_directories(${COMPONENT_LIB} PUBLIC "${BOARD_CONFIG_DIR}")
elseif(DEFINED BOARD AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}")
    target_include_directories(${COMPONENT_LIB} PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}"
    )
else()
    message(WARNING "board_common: No board config directory found. "
                    "Set BOARD_CONFIG_DIR or BOARD CMake variable.")
endif()
