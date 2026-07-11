/**
 * Camera pipeline display driver using LVGL.
 *
 * Two rendering paths:
 * - Image widget mode (MIPI-DSI): pushes frames via lv_image, supports overlays.
 * - Dummy-draw mode (SPI): bypasses LVGL rendering, writes directly to panel
 *   via esp_lcd_panel_draw_bitmap() in DMA-friendly stripes.
 *   Fixes tearing on single-buffered SPI panels (ST7796, etc.).
 */
#pragma once

#include <stdbool.h>
#include "cam_pipeline_display_driver.h"

/**
 * Overlay callback — called per frame before panel push.
 * May modify the frame buffer in place to composite text, icons, etc.
 * Runs in the pipeline's camera task context — keep it fast.
 */
typedef void (*pipeline_overlay_cb_t)(uint8_t *frame_buf, uint32_t width,
                                      uint32_t height, void *user_ctx);

/**
 * Optional config passed as display_config in cam_pipeline_config_t.
 * If NULL, defaults to image widget mode (no dummy draw).
 */
typedef struct {
    bool use_dummy_draw;    /**< true = bypass LVGL, direct stripe blit to panel */
    bool byte_swap;         /**< true = swap RGB565 bytes (SPI panels only) */
    bool keep_lvgl_running; /**< partition mode (dummy-draw only): do NOT stop LVGL.
                                 The camera direct-blits its centered square under the
                                 LVGL port lock while LVGL keeps rendering the gutter
                                 chrome (with live touch); a reserved-rect flush guard
                                 fences LVGL off the square. SPI panels only. */
    pipeline_overlay_cb_t overlay_cb;   /**< per-frame overlay compositing callback */
    void *overlay_cb_ctx;               /**< user context passed to overlay_cb */
} board_pipeline_lvgl_display_config_t;

extern const cam_pipeline_display_driver_t board_pipeline_lvgl_display_driver;
