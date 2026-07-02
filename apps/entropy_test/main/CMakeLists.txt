# Image-entropy consumer test app — camera pipeline + entropy consumer.
set(EXTRA_REQS "")
if(NOT IDF_TARGET STREQUAL "esp32p4")
    list(APPEND EXTRA_REQS "espressif__esp32-camera")
endif()

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES "esp-board-common" "board_log_flash" "espressif__esp_lvgl_port" "esp-camera-pipeline" "cam_pipeline_entropy" "esp_lcd" "mbedtls" ${EXTRA_REQS}
)
