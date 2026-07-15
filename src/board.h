/**
 * Board abstraction interface.
 *
 * Each board is defined by a board_config.h selected at compile time via the
 * BOARD CMake variable.  The generic board_init() in board_init.c dispatches
 * on the config defines to initialise the correct drivers.
 */
#pragma once

#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

/* ── Driver selection enums ── */
#define DISPLAY_ST7796      1
#define DISPLAY_ST7789      2
#define DISPLAY_AXS15231B   3
#define DISPLAY_ST7701      4

#define TOUCH_FT6336        1
#define TOUCH_CST816D       2
#define TOUCH_AXS15231B     3
#define TOUCH_GT911         4

#define PMIC_AXP2101        1

#define CAMERA_DVP          1
#define CAMERA_CSI          2

/* ── Resolution (set in board_init.c from board_config.h) ── */
#ifdef __cplusplus
extern "C" {
#endif

extern const int LCD_H_RES_VAL;
extern const int LCD_V_RES_VAL;

/* ── Compile-time orientation (from Kconfig CONFIG_BOARD_LANDSCAPE) ── */
#ifdef CONFIG_BOARD_LANDSCAPE
#define BOARD_LANDSCAPE  1
#else
#define BOARD_LANDSCAPE  0
#endif

/* Logical display dimensions — what LVGL and application code should use.
 * In landscape mode these are the physical dimensions swapped. */
#if BOARD_LANDSCAPE
#define BOARD_DISP_H_RES  BOARD_LCD_V_RES
#define BOARD_DISP_V_RES  BOARD_LCD_H_RES
#else
#define BOARD_DISP_H_RES  BOARD_LCD_H_RES
#define BOARD_DISP_V_RES  BOARD_LCD_V_RES
#endif

/* ── Application-level config (not hardware — passed by the consuming project) ── */
typedef struct {
    bool landscape;     /* true = 90° CW rotation in flush + touch transform */
} board_app_config_t;

/* ── Board interface ── */

/**
 * Initialise all board hardware and the LVGL display/touch port.
 * Returns 0 on success.
 *
 * @param app_cfg   Application config (orientation, etc.). NULL = defaults (portrait).
 * @param disp      Output: LVGL display handle
 * @param touch_indev Output: LVGL touch input device handle
 */
int board_init(const board_app_config_t *app_cfg,
               lv_display_t **disp, lv_indev_t **touch_indev);

/**
 * Board main loop (never returns).
 * ESP32: idle loop with vTaskDelay.
 * Desktop: SDL event pump.
 */
void board_run(void);

/**
 * Set the maximum LVGL render interval.
 *
 * Creates an internal LVGL timer that forces lv_timer_handler() to
 * return within the specified interval, overriding the default
 * BOARD_LVGL_MAX_SLEEP_MS.
 *
 * Call with a small value (e.g. 10) for real-time camera/animation,
 * or 0 to revert to the default idle behavior.
 *
 * Must hold the LVGL port lock when calling.
 *
 * @param interval_ms  Desired render interval in ms, or 0 to disable.
 */
void board_set_render_interval_ms(uint32_t interval_ms);

/**
 * Reserve a rectangle that LVGL must NOT flush into (partition mode).
 *
 * While a rect is set, the standard-SPI display's LV_EVENT_INVALIDATE_AREA
 * guard clips LVGL invalidations out of this rectangle so a concurrent writer
 * (the camera preview direct-blitting the centered square) owns it exclusively.
 * Coordinates are in LVGL logical pixels. The rect is assumed to be a
 * full-height, horizontally-centered band with chrome living in the left/right
 * gutters (the P4-35 camera-preview layout).
 *
 * Belt-and-suspenders: discipline (keeping chrome widgets in the gutters) is
 * the primary guarantee; this only prevents flicker from stray invalidations.
 * No-op on boards whose display path never registers the guard (ST7701/DSI,
 * RASET). Safe to call from any task while holding the LVGL port lock.
 */
void board_display_set_reserved_rect(int32_t x, int32_t y, int32_t w, int32_t h);

/** Clear the reserved rectangle set by board_display_set_reserved_rect(). */
void board_display_clear_reserved_rect(void);

/* ── Single-writer partition compositing (ST7796 camera preview) ──────────────
 * During a camera session on a partition-mode board, LVGL's flush is redirected
 * into an off-screen shadow framebuffer (it issues NO SPI transactions) and the
 * camera consumer becomes the sole SPI writer: it blits its square AND the gutter
 * columns (from the shadow FB) itself. This removes the two-writer bus collision.
 * begin/end bracket the session; blit_gutters() runs each camera frame. All three
 * are no-ops / absent on boards that don't define BOARD_CAMERA_PARTITION_MODE.
 * MUST be called from the camera consumer task (the sole writer). Declared
 * unconditionally (definitions + call sites are gated on BOARD_CAMERA_PARTITION_MODE
 * so only that board links them). */
esp_err_t board_display_partition_begin(int32_t disp_w, int32_t disp_h,
                                        int32_t sq_x, int32_t sq_y,
                                        int32_t sq_w, int32_t sq_h);
void board_display_partition_end(void);
void board_display_partition_blit_gutters(void);
/* Synchronous panel blit (draw_bitmap + wait for DMA completion) — lets the camera
 * consumer reuse its source buffer immediately. Ends are exclusive, like
 * esp_lcd_panel_draw_bitmap. No-op stub on non-partition boards. */
void board_display_partition_blit(int32_t x_start, int32_t y_start,
                                  int32_t x_end, int32_t y_end, const void *buf);

/* ── Portrait scan display mode (ST7701 MIPI-DSI 4.3 only) ────────────────────
 * The 4.3 renders its landscape UI by CPU-rotating every LVGL frame 90° to the
 * native-portrait panel — the per-frame rotate pegs core 0 and blocks a 2nd
 * decoder (Phase 0 probe proved a 2nd decoder ~doubles throughput). These bracket
 * a QR-scan session: enter reconfigures the LVGL display to native portrait with
 * NO rotation (RENDER_MODE_PARTIAL, direct flush) so the camera can sub-region
 * blit its centered square and LVGL renders only the letterbox — freeing core 0.
 * In-place reconfig (not recreate) so screens + touch indev survive; touch is
 * swapped to portrait and restored on exit. Idempotent. MUST be called with the
 * scan's LVGL screen active. No-op stub on non-ST7701 boards (they partition or
 * are already portrait). Every scan-teardown path must call exit before the next
 * landscape render. */
void board_display_enter_portrait_scan(void);
void board_display_exit_portrait_scan(void);

/* Single-writer panel blit for portrait scan mode: serializes the camera's
 * centered-square blit against LVGL's letterbox flush through the panel's one
 * DMA2D draw path (mutex + wait-own-completion) — both writers share the DPI
 * draw_sem, so a concurrent draw_bitmap trips ESP_ERR_INVALID_STATE. Ends
 * exclusive, like esp_lcd_panel_draw_bitmap. No-op stub on non-ST7701 boards. */
void board_display_portrait_scan_blit(int32_t x1, int32_t y1,
                                      int32_t x2, int32_t y2, const void *buf);

/** Get LCD panel handle (for diagnostic/testing that bypasses LVGL). */
esp_lcd_panel_handle_t board_get_panel_handle(void);

/** Get touch handle (for direct driver access — e.g. reading strength data
 *  that LVGL discards). Returns NULL if touch init failed. */
esp_lcd_touch_handle_t board_get_touch_handle(void);

#ifdef __cplusplus
}
#endif

/* No app_main(), no game_main() — each project defines its own entry point */
