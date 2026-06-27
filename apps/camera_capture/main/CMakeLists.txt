# camera_capture — tap-to-photo, saves RGB565 frames to SD card.
# Reuses esp-board-common (board init, SD card) and esp-camera-pipeline (frame engine).
#
# capture_ui_dsi.c is always compiled; its body is gated on the DSI display
# driver via #if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701 so it becomes an
# empty TU on SPI boards. main.c emits a #error on unsupported display paths.

idf_component_register(
    SRCS "main.c" "capture_service.c" "capture_ui_dsi.c"
    INCLUDE_DIRS "."
    REQUIRES "esp-board-common" "espressif__esp_lvgl_port" "esp-camera-pipeline" "esp_lcd" "fatfs"
)
