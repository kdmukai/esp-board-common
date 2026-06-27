/**
 * camera_capture — tap-to-photo, saves RGB565 frames to SD card.
 *
 * Minimal camera app. Live preview from the pipeline is displayed; tapping
 * the screen captures the current frame and writes it to /sdcard as raw
 * RGB565 (see capture_service for file format). Intended as a data
 * collection tool — e.g. for building a k_quirc corpus — not a QR decoder.
 *
 * Current board support: DSI path only (waveshare_p4_lcd43). SPI dummy-draw
 * boards get a compile-time #error until capture_ui_spi.c is written.
 */
#include <inttypes.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"
#include "board_config.h"
#include "board_pipeline.h"
#include "board_i2c.h"
#include "board_backlight.h"
#include "board_sdcard.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_cam_pipeline.h"

#include "capture_service.h"
#include "capture_ui.h"

static const char *TAG = "camera_capture";

#if !BOARD_HAS_CAMERA
#error "camera_capture requires a board with camera support (BOARD_HAS_CAMERA=1)"
#endif

#if !BOARD_HAS_SDCARD
#error "camera_capture requires a board with an SD card slot (BOARD_HAS_SDCARD=1)"
#endif

#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
#error "camera_capture currently supports only the DSI (ST7701) display path. \
SPI dummy-draw boards need capture_ui_spi.c — see apps/camera_capture/main/capture_ui.h."
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "starting");

    lv_display_t *disp;
    lv_indev_t *touch;
    board_app_config_t app_cfg = { .landscape = BOARD_LANDSCAPE };
    board_init(&app_cfg, &disp, &touch);

    lv_obj_t *screen = NULL;
    if (lvgl_port_lock(0)) {
        screen = lv_screen_active();
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
        board_set_render_interval_ms(10);
        lvgl_port_unlock();
    }

    cam_pipeline_config_t pipeline_cfg = board_pipeline_default_config(
        screen, board_i2c_get_handle());

    /* Square crop: use shorter logical dimension for both axes.
     * Same convention as qr_decoder — keeps captured samples representative
     * of what the decoder sees. */
    uint32_t square = (BOARD_DISP_H_RES < BOARD_DISP_V_RES)
                          ? BOARD_DISP_H_RES : BOARD_DISP_V_RES;
    pipeline_cfg.display_width = square;
    pipeline_cfg.display_height = square;

    cam_pipeline_handle_t pipeline = cam_pipeline_create(&pipeline_cfg);
    if (!pipeline) {
        ESP_LOGE(TAG, "pipeline create failed");
        board_backlight_set(100);
        board_run();
        return;
    }

    /* SD card — fail-soft. A missing card logs, captures fail gracefully. */
    board_sdcard_init();

    if (capture_service_init(pipeline, square, square) != ESP_OK) {
        ESP_LOGE(TAG, "capture_service init failed");
    }

    if (capture_ui_init(pipeline) != ESP_OK) {
        ESP_LOGE(TAG, "capture_ui init failed");
    }

    board_backlight_set(100);

    ESP_LOGI(TAG, "running (%" PRIu32 "x%" PRIu32 " square)", square, square);
    board_run();
}
