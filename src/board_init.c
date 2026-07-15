/**
 * Generic board initialisation.
 *
 * Reads board_config.h (selected at build time via BOARD CMake variable)
 * and dispatches to the correct driver init functions.
 *
 * Supports landscape orientation via board_app_config_t.landscape flag.
 * On RASET-bug boards (AXS15231B), landscape uses a per-pixel 90° CW
 * rotation flush callback; portrait uses memcpy + byte swap.
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

#include "board.h"
#include "board_config.h"

#include "board_i2c.h"
#include "board_backlight.h"
#if BOARD_HAS_CAMERA
#include "board_pipeline.h"
#include "board_pipeline_display_lvgl.h"
#endif

#if BOARD_HAS_PMIC
#include "board_pmic.h"
#endif

#if BOARD_DISPLAY_DRIVER == DISPLAY_AXS15231B
#include "board_display_axs15231b.h"
#include "esp_lcd_axs15231b.h"
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7796
#include "board_display_st7796.h"
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7789
#include "board_display_st7789.h"
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
#include "board_display_st7701.h"
#endif

#if BOARD_TOUCH_DRIVER == TOUCH_AXS15231B
#include "board_touch_axs15231b.h"
#elif BOARD_TOUCH_DRIVER == TOUCH_FT6336
#include "board_touch_ft6336.h"
#elif BOARD_TOUCH_DRIVER == TOUCH_CST816D
#include "board_touch_cst816d.h"
#elif BOARD_TOUCH_DRIVER == TOUCH_GT911
#include "board_touch_gt911.h"
#endif

#if BOARD_HAS_IO_EXPANDER
#include "esp_io_expander_tca9554.h"
#endif

#include "esp_lvgl_port.h"
#include "esp_lcd_touch.h"
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
#include "esp_lcd_mipi_dsi.h"
#include "esp_timer.h"
#endif
#include "lvgl.h"

static const char *TAG = "board";

/* ── Resolution globals ── */
const int LCD_H_RES_VAL = BOARD_LCD_H_RES;
const int LCD_V_RES_VAL = BOARD_LCD_V_RES;

/* ── Hardware handles ── */
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_touch_handle_t touch_handle = NULL;

/* Accessor for diagnostic/testing (bypasses LVGL) */
esp_lcd_panel_handle_t board_get_panel_handle(void) { return panel_handle; }
esp_lcd_touch_handle_t board_get_touch_handle(void) { return touch_handle; }

/* ── Partition-mode flush guard ──────────────────────────────────────────────
 * Fences LVGL out of a reserved rectangle (the camera-preview square) so a
 * direct-blit writer can own it while LVGL keeps rendering the surrounding
 * gutters. See board_display_set_reserved_rect() in board.h. Registered only on
 * the standard-SPI display path (the partition-capable panels). ── */

/* The standard-SPI display path (partial-update, MADCTL) — the only one that
 * partitions the panel and registers the invalidate-area guard below. */
#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701 && !BOARD_DISPLAY_QUIRK_RASET_BUG
#define BOARD_DISPLAY_STD_SPI 1
#else
#define BOARD_DISPLAY_STD_SPI 0
#endif

static lv_area_t s_reserved_rect;
static bool      s_reserved_active = false;

#if BOARD_DISPLAY_STD_SPI
static void invalidate_area_guard_cb(lv_event_t *e)
{
    if (!s_reserved_active) return;
    lv_area_t *a = (lv_area_t *)lv_event_get_param(e);
    if (!a) return;

    /* No overlap with the reserved band → leave the invalidation untouched. */
    if (a->x2 < s_reserved_rect.x1 || a->x1 > s_reserved_rect.x2 ||
        a->y2 < s_reserved_rect.y1 || a->y1 > s_reserved_rect.y2) {
        return;
    }

    /* Overlaps the camera band. The band is full-height and horizontally
     * centered, so legitimate chrome lives in the left (x < x1) or right
     * (x > x2) gutter; clip the invalidation to the gutter slab it extends
     * into. If it is wholly inside the band (a stray over-square invalidation),
     * collapse it to a 1px no-op — the camera repaints that pixel next frame. */
    if (a->x1 < s_reserved_rect.x1) {
        a->x2 = s_reserved_rect.x1 - 1;   /* keep the left-gutter slab */
    } else if (a->x2 > s_reserved_rect.x2) {
        a->x1 = s_reserved_rect.x2 + 1;   /* keep the right-gutter slab */
    } else {
        a->x2 = a->x1;                    /* fully inside → 1px no-op */
        a->y2 = a->y1;
    }
}

#if BOARD_CAMERA_PARTITION_MODE
/* ── Partition-mode custom flush (single-writer camera compositing) ───────────
 * We take over LVGL's flush for this board so a camera session can redirect
 * LVGL's gutter rendering OFF the SPI bus and into a shadow framebuffer — making
 * the camera the SOLE SPI writer and structurally eliminating the two-writer bus
 * collision that froze the ST7796 (LVGL flush + camera direct-blit racing on one
 * panel IO). esp_lvgl_port's on_color_trans_done stays registered and calls
 * lv_disp_flush_ready when the DMA lands, so here we only swap + blit.
 *
 *   - normal (no camera): identical to esp_lvgl_port's flush — swap + draw_bitmap.
 *   - camera active (s_cam_flush_redirect): copy the rendered gutter pixels into
 *     the shadow FB (byte-swapped, panel-ready) + mark the gutter dirty; the
 *     camera consumer blits them. LVGL issues ZERO SPI transactions. (stage 2) */
#define GUTTER_BAND_LINES 40
static volatile bool s_cam_flush_redirect = false;
static uint16_t *s_shadow_fb  = NULL;   /* full landscape frame, panel-ready (byte-swapped) */
static uint16_t *s_gutter_dma = NULL;   /* internal-DMA band buffer for gutter blits */
static int32_t   s_shadow_w = 0, s_shadow_h = 0;   /* shadow FB dims (LVGL/panel space) */
static int32_t   s_sq_x1 = 0, s_sq_x2 = -1;        /* camera-square column span; gutters flank it */
static int32_t   s_gutter_w = 0;                   /* wider of the two gutters (DMA band width) */
static SemaphoreHandle_t s_shadow_mutex = NULL;    /* guards shadow FB: LVGL write vs camera read */
static lv_display_t *s_std_disp = NULL;
static SemaphoreHandle_t s_blit_sem = NULL;        /* signalled per draw_bitmap DMA completion */

/* Our own on_color_trans_done for the ST7796 panel IO (replaces esp_lvgl_port's).
 * Every draw_bitmap DMA completion: (1) signal s_blit_sem so a camera-path blit can
 * wait before reusing its DMA buffer (trans_queue_depth=10 → draw_bitmap is async);
 * (2) still call lv_display_flush_ready so normal LVGL rendering works unchanged. */
static bool std_spi_blit_done_cb(esp_lcd_panel_io_handle_t io,
                                 esp_lcd_panel_io_event_data_t *ed, void *ctx)
{
    (void)io; (void)ed; (void)ctx;
    BaseType_t woken = pdFALSE;
    if (s_blit_sem) xSemaphoreGiveFromISR(s_blit_sem, &woken);
    /* Only ack a REAL LVGL panel flush. During a camera session LVGL's flush is
     * redirected (it calls flush_ready itself, synchronously) and the camera does
     * ~22 blits/frame — acking those here would be spurious flush_ready calls that
     * corrupt LVGL's draw-buffer accounting, so the first full-screen render after
     * the session over-queues the SPI bus → queue full. Gate on !redirect. */
    if (!s_cam_flush_redirect && s_std_disp) lv_display_flush_ready(s_std_disp);
    return woken == pdTRUE;
}

/* Synchronous panel blit for the camera path: draw_bitmap + WAIT for its DMA to
 * finish, so the caller may safely reuse the source buffer immediately. Only the
 * camera consumer (sole SPI writer) uses this during a session. */
void board_display_partition_blit(int32_t x_start, int32_t y_start,
                                  int32_t x_end, int32_t y_end, const void *buf)
{
    if (s_blit_sem) xSemaphoreTake(s_blit_sem, 0);   /* drop any stale completion */
    esp_err_t bret = esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, buf);
    if (bret != ESP_OK) {
        ESP_LOGE(TAG, "partition blit failed: %s %d,%d..%d,%d",
                 esp_err_to_name(bret), (int)x_start, (int)y_start, (int)x_end, (int)y_end);
    }
    /* Wait for THIS blit's DMA to finish before the caller reuses `buf`. */
    if (s_blit_sem) xSemaphoreTake(s_blit_sem, pdMS_TO_TICKS(100));
}

static void std_spi_partition_flush_cb(lv_display_t *disp,
                                       const lv_area_t *area, uint8_t *px_map)
{
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;

    if (s_cam_flush_redirect && s_shadow_fb) {
        /* Camera session: copy the rendered gutter pixels into the shadow FB
         * (byte-swapped → panel-ready) so the camera consumer blits them. LVGL
         * issues ZERO SPI transactions here → the camera is the sole SPI writer. */
        const uint16_t *src = (const uint16_t *)px_map;
        if (s_shadow_mutex) xSemaphoreTake(s_shadow_mutex, portMAX_DELAY);
        for (int32_t r = 0; r < h; r++) {
            int32_t sy = area->y1 + r;
            if (sy < 0 || sy >= s_shadow_h) continue;
            uint16_t *drow = &s_shadow_fb[(size_t)sy * s_shadow_w];
            const uint16_t *srow = &src[(size_t)r * w];
            for (int32_t c = 0; c < w; c++) {
                int32_t sx = area->x1 + c;
                if (sx < 0 || sx >= s_shadow_w) continue;
                uint16_t p = srow[c];
                drow[sx] = (uint16_t)((p >> 8) | (p << 8));
            }
        }
        if (s_shadow_mutex) xSemaphoreGive(s_shadow_mutex);
        lv_display_flush_ready(disp);
        return;
    }

    lv_draw_sw_rgb565_swap(px_map, (uint32_t)w * (uint32_t)h);
    esp_err_t dret = esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1,
                                               area->x2 + 1, area->y2 + 1, px_map);
    if (dret != ESP_OK) {
        ESP_LOGE(TAG, "flush failed: %s area=%d,%d..%d,%d",
                 esp_err_to_name(dret), (int)area->x1, (int)area->y1,
                 (int)area->x2, (int)area->y2);
    }
    /* flush_ready is signalled by esp_lvgl_port's on_color_trans_done callback. */
}

esp_err_t board_display_partition_begin(int32_t disp_w, int32_t disp_h,
                                        int32_t sq_x, int32_t sq_y,
                                        int32_t sq_w, int32_t sq_h)
{
    s_shadow_w = disp_w;  s_shadow_h = disp_h;
    s_sq_x1 = sq_x;       s_sq_x2 = sq_x + sq_w - 1;
    if (!s_shadow_mutex) s_shadow_mutex = xSemaphoreCreateMutex();
    if (!s_shadow_fb) {
        s_shadow_fb = heap_caps_calloc(1, (size_t)disp_w * disp_h * 2, MALLOC_CAP_SPIRAM);
        if (!s_shadow_fb) {
            ESP_LOGE(TAG, "shadow FB alloc failed (%d B; SPIRAM free=%u)",
                     (int)((size_t)disp_w * disp_h * 2),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
            return ESP_ERR_NO_MEM;
        }
    }
    /* Gutter DMA band buffer sized to the GUTTER width (not the full panel) so it
     * fits the squeezed internal heap during a camera session. */
    int32_t gw_l = sq_x, gw_r = disp_w - (sq_x + sq_w);
    s_gutter_w = (gw_l > gw_r) ? gw_l : gw_r;
    if (s_gutter_w < 1) s_gutter_w = 1;
    if (!s_gutter_dma) {
        /* 64-byte aligned so the gutter blit is zero-copy too: an unaligned DMA
         * source makes the SPI master bounce-allocate per blit, churning the
         * DMA-capable heap (see the stripe-cap note in the pipeline). */
        s_gutter_dma = heap_caps_aligned_alloc(64,
                                        (size_t)s_gutter_w * GUTTER_BAND_LINES * 2,
                                        MALLOC_CAP_DMA);
        if (!s_gutter_dma) {
            ESP_LOGE(TAG, "gutter DMA alloc failed (%d B; INTERNAL free=%u largest=%u)",
                     (int)((size_t)s_gutter_w * GUTTER_BAND_LINES * 2),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
            heap_caps_free(s_shadow_fb); s_shadow_fb = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_blit_sem) xSemaphoreTake(s_blit_sem, 0);   /* start clean (drop stale completion) */
    /* Clip LVGL to the gutters and redirect its flush to the shadow FB. The shadow
     * FB is zeroed (black), so the first gutter blit paints black until LVGL renders
     * the chrome; the camera paints the square on its first frame. */
    board_display_set_reserved_rect(sq_x, sq_y, sq_w, sq_h);
    s_cam_flush_redirect = true;
    /* Kick LVGL to render the current overlay screen into the shadow FB. */
    if (s_std_disp && lvgl_port_lock(100)) {
        lv_obj_invalidate(lv_screen_active());
        lvgl_port_unlock();
    }
    ESP_LOGI(TAG, "Partition single-writer begin: %"PRId32"x%"PRId32
             " square cols %"PRId32"..%"PRId32, disp_w, disp_h, s_sq_x1, s_sq_x2);
    return ESP_OK;
}

void board_display_partition_end(void)
{
    /* Stop redirecting first, so LVGL's flush returns to the panel path. Do NOT
     * trigger an LVGL repaint here: the camera consumer may still be finishing
     * its last synchronous blit, and issuing an LVGL panel flush now would race
     * it (two writers again → the teardown SPI error). The app repaints on its
     * next screen, and lvgl_display_deinit's container delete repaints too — both
     * after the consumer has stopped. Take the shadow mutex so no flush is
     * mid-copy into the buffers we free. */
    s_cam_flush_redirect = false;
    board_display_clear_reserved_rect();
    if (s_shadow_mutex) xSemaphoreTake(s_shadow_mutex, portMAX_DELAY);
    if (s_shadow_fb)  { heap_caps_free(s_shadow_fb);  s_shadow_fb = NULL; }
    if (s_gutter_dma) { heap_caps_free(s_gutter_dma); s_gutter_dma = NULL; }
    if (s_shadow_mutex) xSemaphoreGive(s_shadow_mutex);
    ESP_LOGI(TAG, "Partition single-writer end");
}

/* Blit both gutter columns from the shadow FB to the panel. MUST be called only
 * from the camera consumer task (the sole SPI writer), never concurrently with
 * the square blit. */
void board_display_partition_blit_gutters(void)
{
    if (!s_cam_flush_redirect || !s_shadow_fb || !s_gutter_dma) return;
    int32_t spans[2][2] = { { 0, s_sq_x1 - 1 }, { s_sq_x2 + 1, s_shadow_w - 1 } };
    for (int g = 0; g < 2; g++) {
        int32_t gx1 = spans[g][0], gx2 = spans[g][1];
        int32_t gw = gx2 - gx1 + 1;
        if (gw <= 0) continue;
        for (int32_t y = 0; y < s_shadow_h; y += GUTTER_BAND_LINES) {
            int32_t bh = (y + GUTTER_BAND_LINES > s_shadow_h) ? s_shadow_h - y : GUTTER_BAND_LINES;
            if (s_shadow_mutex) xSemaphoreTake(s_shadow_mutex, portMAX_DELAY);
            for (int32_t r = 0; r < bh; r++) {
                memcpy(&s_gutter_dma[(size_t)r * gw],
                       &s_shadow_fb[(size_t)(y + r) * s_shadow_w + gx1],
                       (size_t)gw * 2);
            }
            if (s_shadow_mutex) xSemaphoreGive(s_shadow_mutex);
            /* Synchronous: wait for the DMA before the next band reuses
             * s_gutter_dma (draw_bitmap is async, trans_queue_depth=10). */
            board_display_partition_blit(gx1, y, gx2 + 1, y + bh, s_gutter_dma);
        }
    }
}
#endif /* BOARD_CAMERA_PARTITION_MODE */
#endif /* BOARD_DISPLAY_STD_SPI */

void board_display_set_reserved_rect(int32_t x, int32_t y, int32_t w, int32_t h)
{
    if (w <= 0 || h <= 0) {
        s_reserved_active = false;
        return;
    }
    s_reserved_rect.x1 = x;
    s_reserved_rect.y1 = y;
    s_reserved_rect.x2 = x + w - 1;
    s_reserved_rect.y2 = y + h - 1;
    s_reserved_active = true;
}

void board_display_clear_reserved_rect(void)
{
    s_reserved_active = false;
}

#if !BOARD_CAMERA_PARTITION_MODE
/* No-op stubs for boards that don't do single-writer partition compositing.
 * (The real implementations live in the BOARD_CAMERA_PARTITION_MODE block below.)
 * These keep the shared display-driver call sites unconditional. */
esp_err_t board_display_partition_begin(int32_t disp_w, int32_t disp_h,
                                        int32_t sq_x, int32_t sq_y,
                                        int32_t sq_w, int32_t sq_h)
{
    (void)disp_w; (void)disp_h; (void)sq_x; (void)sq_y; (void)sq_w; (void)sq_h;
    return ESP_OK;
}
void board_display_partition_end(void) {}
void board_display_partition_blit_gutters(void) {}
void board_display_partition_blit(int32_t x_start, int32_t y_start,
                                  int32_t x_end, int32_t y_end, const void *buf)
{ (void)x_start; (void)y_start; (void)x_end; (void)y_end; (void)buf; }
#endif /* !BOARD_CAMERA_PARTITION_MODE */

#if BOARD_HAS_IO_EXPANDER
static esp_io_expander_handle_t expander_handle = NULL;
#endif

/* ── AXS15231B RASET workaround (custom flush + DMA bounce) ── */
#if BOARD_DISPLAY_QUIRK_RASET_BUG

static SemaphoreHandle_t flush_done_sem = NULL;
static uint8_t *swap_buf[2] = {NULL, NULL};

static bool flush_ready_cb(esp_lcd_panel_io_handle_t panel_io,
                           esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(flush_done_sem, &woken);
    return (woken == pdTRUE);
}

/**
 * Portrait flush callback: memcpy + byte swap, banded DMA.
 * Used when landscape == false on RASET-bug boards.
 */
static void portrait_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (!lv_display_flush_is_last(disp)) {
        lv_display_flush_ready(disp);
        return;
    }

    const int bpp = 2;  /* RGB565 */
    int buf_idx = 0;

    for (int y = 0; y < BOARD_LCD_V_RES; y += BOARD_LINES_PER_BAND) {
        int band_h = (y + BOARD_LINES_PER_BAND > BOARD_LCD_V_RES)
                     ? BOARD_LCD_V_RES - y : BOARD_LINES_PER_BAND;
        int band_bytes = BOARD_LCD_H_RES * band_h * bpp;
        uint8_t *src = px_map + (y * BOARD_LCD_H_RES * bpp);
        uint8_t *dst = swap_buf[buf_idx];

        memcpy(dst, src, band_bytes);
        lv_draw_sw_rgb565_swap(dst, BOARD_LCD_H_RES * band_h);

        if (y > 0) {
            xSemaphoreTake(flush_done_sem, portMAX_DELAY);
        }

        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_LCD_H_RES, y + band_h, dst);
        buf_idx ^= 1;
    }

    xSemaphoreTake(flush_done_sem, portMAX_DELAY);
    lv_display_flush_ready(disp);
}

/**
 * Landscape flush callback: 90° CW rotation + byte swap, banded DMA.
 * Used when landscape == true on RASET-bug boards.
 *
 * LVGL renders in landscape (V_RES x H_RES) into a SPIRAM framebuffer.
 * This callback rotates pixels 90° CW to portrait and sends 320-wide
 * portrait bands to the panel.
 *
 * Landscape LVGL dimensions: hres=LCD_V_RES (480), vres=LCD_H_RES (320)
 * Panel physical dimensions: 320 wide x 480 tall
 */
static void landscape_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    if (!lv_display_flush_is_last(disp)) {
        lv_display_flush_ready(disp);
        return;
    }

    /* Landscape framebuffer: DISP_H_RES=LCD_V_RES wide, DISP_V_RES=LCD_H_RES tall */
    const int DISP_H_RES = BOARD_LCD_V_RES;  /* 480 */
    const int DISP_V_RES = BOARD_LCD_H_RES;  /* 320 */

    uint16_t *fb = (uint16_t *)px_map;
    int buf_idx = 0;

    for (int py = 0; py < BOARD_LCD_V_RES; py += BOARD_LINES_PER_BAND) {
        int band_h = (py + BOARD_LINES_PER_BAND > BOARD_LCD_V_RES)
                     ? BOARD_LCD_V_RES - py : BOARD_LINES_PER_BAND;
        uint16_t *dst = (uint16_t *)swap_buf[buf_idx];

        /* 90° CW rotation: panel(px, py) ← framebuffer(py, DISP_V_RES-1-px)
         * Loop order: px outer, by inner — sequential fb reads for cache efficiency */
        for (int px = 0; px < BOARD_LCD_H_RES; px++) {
            int fb_y = DISP_V_RES - 1 - px;
            int fb_row_offset = fb_y * DISP_H_RES + py;
            for (int by = 0; by < band_h; by++) {
                uint16_t pixel = fb[fb_row_offset + by];
                dst[by * BOARD_LCD_H_RES + px] = (pixel >> 8) | (pixel << 8);
            }
        }

        if (py > 0) {
            xSemaphoreTake(flush_done_sem, portMAX_DELAY);
        }

        esp_lcd_panel_draw_bitmap(panel_handle, 0, py, BOARD_LCD_H_RES, py + band_h, dst);
        buf_idx ^= 1;
    }

    xSemaphoreTake(flush_done_sem, portMAX_DELAY);
    lv_display_flush_ready(disp);
}

#endif /* BOARD_DISPLAY_QUIRK_RASET_BUG */

/* ── ST7701 MIPI-DSI landscape flush ──
 * LVGL renders in landscape (V_RES × H_RES). This callback CPU-rotates
 * the full frame 90° CCW into a double-buffered PSRAM output buffer,
 * then submits to the panel in portrait coordinates synced to vsync.
 *
 * esp_lvgl_port's sw_rotate is incompatible with the full_refresh required
 * by DPI panels, so we handle rotation ourselves.
 *
 * Originally implemented in commit 490f027; lost during the adapter
 * migration (b73fb77 → 83a1a38). Restored here. */
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701

static uint16_t *st7701_rot_buf[2] = {NULL, NULL};
static int st7701_rot_idx = 0;
static SemaphoreHandle_t dpi_flush_sem = NULL;

/* Deferred flush: the 90° rotation + vsync-wait + panel blit run on this task,
 * OFF the LVGL lock. esp_lvgl_port wraps lv_timer_handler (which calls the
 * flush_cb) in the recursive LVGL mutex, so doing the ~30ms rotation + ~16ms
 * vsync wait inline held the lock ~45ms longer per frame — long enough that the
 * camera's non-blocking try-lock frame push (board_pipeline_display_lvgl.c)
 * lost the race ~25% of the time and the preview stalled at ~8fps.  Handing the
 * work to this task lets lv_timer_handler return as soon as the render finishes,
 * dropping the per-frame lock hold from ~(render + rot + vsync) to ~render — i.e.
 * below the camera's ~56ms produce cadence, so pushes stop getting skipped.
 *
 * This REQUIRES LV_DISPLAY_RENDER_MODE_FULL (set in lvgl_port_setup).  In DIRECT
 * mode LVGL's refr_sync_areas() waits for the pending flush BEFORE the render
 * (and also does a per-frame inter-buffer sync copy), both under the lock, which
 * would re-serialize and cancel the benefit.  FULL mode's only cross-frame wait
 * is in draw_buf_flush() AFTER the render, so the render of frame N overlaps the
 * flush task rotating frame N-1. */
static SemaphoreHandle_t st7701_flush_start_sem = NULL;  /* flush_cb  -> task     */
static SemaphoreHandle_t st7701_flush_done_sem  = NULL;  /* task -> flush_wait_cb */
static uint16_t *st7701_flush_fb = NULL;                 /* buffer handed to task */

/* ── Portrait scan mode (Phase 1) state ──
 * The normal UI renders landscape (800×480) + CPU-rotates every frame to the
 * native-portrait panel. A QR scan switches the SAME display in place to native
 * portrait (480×800) with NO rotation so the camera can sub-region blit and core
 * 0 is freed. These are captured from lvgl_port_setup so enter/exit can reconfig
 * the live display and restore it. */
static lv_display_t     *s_st7701_disp = NULL;
static void             *s_st7701_draw_buf[2] = {NULL, NULL};
static size_t            s_st7701_draw_buf_sz = 0;
static bool              s_portrait_scan_active = false;
/* Blit-completion signal for the portrait direct flush: use_dma2d=true makes DPI
 * draw_bitmap from an external (LVGL draw) buffer async, so the flush must wait
 * for the DMA2D copy before releasing the buffer back to LVGL. Given from the
 * panel's on_color_trans_done (registered alongside on_refresh_done in setup). */
static SemaphoreHandle_t s_dsi_blit_done = NULL;
/* Serializes the two portrait-scan writers (camera square + LVGL letterbox flush)
 * through the panel's one DMA2D draw path. */
static SemaphoreHandle_t s_dsi_blit_mutex = NULL;

static bool dpi_color_trans_done_cb(esp_lcd_panel_handle_t panel,
                                    esp_lcd_dpi_panel_event_data_t *edata,
                                    void *user_ctx)
{
    (void)panel; (void)edata; (void)user_ctx;
    BaseType_t woken = pdFALSE;
    if (s_dsi_blit_done) xSemaphoreGiveFromISR(s_dsi_blit_done, &woken);
    return (woken == pdTRUE);
}

/* Single-writer gate: draw_bitmap + wait for the (async, use_dma2d) copy, under a
 * mutex so the camera square and the LVGL letterbox flush never issue overlapping
 * draw_bitmaps (which would trip the DPI driver's draw_sem). */
void board_display_portrait_scan_blit(int32_t x1, int32_t y1,
                                      int32_t x2, int32_t y2, const void *buf)
{
    if (!s_dsi_blit_mutex) return;
    xSemaphoreTake(s_dsi_blit_mutex, portMAX_DELAY);
    if (s_dsi_blit_done) xSemaphoreTake(s_dsi_blit_done, 0);            /* drain stale */
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2, y2, buf);
    if (s_dsi_blit_done) xSemaphoreTake(s_dsi_blit_done, pdMS_TO_TICKS(100));
    xSemaphoreGive(s_dsi_blit_mutex);
}

/* Portrait reserved-rect guard: clips LVGL invalidations OUT of the centered
 * camera square (full-width band, so clip on Y — keep the top/bottom letterbox),
 * the DSI analog of the SPI gutter guard. Inactive until enter_portrait sets a
 * rect. Belt-and-suspenders: keeping chrome in the letterbox is the primary
 * guarantee; this stops a stray full-screen invalidation from repainting the
 * square over the live camera. */
static void dsi_portrait_invalidate_guard_cb(lv_event_t *e)
{
    if (!s_reserved_active) return;
    lv_area_t *a = (lv_area_t *)lv_event_get_param(e);
    if (!a) return;

    if (a->x2 < s_reserved_rect.x1 || a->x1 > s_reserved_rect.x2 ||
        a->y2 < s_reserved_rect.y1 || a->y1 > s_reserved_rect.y2) {
        return;  /* no overlap with the reserved square */
    }
    if (a->y1 < s_reserved_rect.y1) {
        a->y2 = s_reserved_rect.y1 - 1;   /* keep top-letterbox slab */
    } else if (a->y2 > s_reserved_rect.y2) {
        a->y1 = s_reserved_rect.y2 + 1;   /* keep bottom-letterbox slab */
    } else {
        a->x2 = a->x1;                    /* fully inside → 1px no-op */
        a->y2 = a->y1;
    }
}

static bool dpi_vsync_ready_cb(esp_lcd_panel_handle_t panel,
                               esp_lcd_dpi_panel_event_data_t *edata,
                               void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(dpi_flush_sem, &woken);
    return (woken == pdTRUE);
}

/**
 * Rotate the just-rendered landscape frame 90° CCW into the portrait output
 * buffer, then blit it to the panel synced to vsync.  Runs on the flush task,
 * NOT under the LVGL lock.
 *
 * DPI panels don't support partial updates or hardware rotation (MADCTL), so we
 * rotate on the CPU.  No byte swap needed for MIPI-DSI (unlike SPI panels).
 * The output is double-buffered (st7701_rot_buf[0/1]): we rotate into the back
 * buffer while the panel scans the front.
 *
 * Tearing prevention: draw_bitmap triggers a DMA2D copy into the panel's frame
 * buffer.  We wait for the on_refresh_done (vsync) callback so the copy lands in
 * the vertical blanking interval, not mid-scan.  The drain take(0) discards any
 * stale vsync that fired during rotation.
 */
static void st7701_rotate_and_blit(uint16_t *fb)
{
    const int DISP_W = BOARD_LCD_V_RES;  /* landscape width, e.g. 800 */
    uint16_t *out = st7701_rot_buf[st7701_rot_idx];

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    int64_t t0 = esp_timer_get_time();
#endif

    /* 90° CCW rotation: portrait(px, py) = landscape(DISP_W-1-py, px) */
    for (int px = 0; px < BOARD_LCD_H_RES; px++) {
        for (int py = 0; py < BOARD_LCD_V_RES; py++) {
            int fb_x = DISP_W - 1 - py;
            out[py * BOARD_LCD_H_RES + px] = fb[px * DISP_W + fb_x];
        }
    }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    int64_t t1 = esp_timer_get_time();
    static int64_t disp_rot_sum = 0;
    static int64_t disp_rot_max = 0;
    static int disp_rot_count = 0;
    static int64_t disp_rot_last_log = 0;
    int64_t disp_rot_dur = t1 - t0;
    disp_rot_sum += disp_rot_dur;
    if (disp_rot_dur > disp_rot_max) disp_rot_max = disp_rot_dur;
    disp_rot_count++;
    if (t1 - disp_rot_last_log > 2000000) {  /* every 2s */
        /* %d (not %lld): nano-printf (CONFIG_LIBC_NEWLIB_NANO_FORMAT) has no
         * 64-bit conversion; us values fit in int. */
        ESP_LOGI(TAG, "DISP CPU: avg=%d us  max=%d us  n=%d",
                 (int)(disp_rot_sum / disp_rot_count),
                 (int)disp_rot_max, disp_rot_count);
        disp_rot_sum = 0;
        disp_rot_max = 0;
        disp_rot_count = 0;
        disp_rot_last_log = t1;
    }
#endif

    /* Wait for vsync, then submit draw_bitmap so the DMA copy runs during the
     * vertical blanking interval — not mid-scan. */
    xSemaphoreTake(dpi_flush_sem, 0);             /* drain stale vsync */
    xSemaphoreTake(dpi_flush_sem, portMAX_DELAY); /* wait for vsync start */
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0,
                              BOARD_LCD_H_RES, BOARD_LCD_V_RES, out);
    /* Wait for the DMA2D trans to COMPLETE (not merely be issued) before returning.
     * draw_bitmap is async (use_dma2d): it takes the panel's draw_sem and releases
     * it in on_color_trans_done (which also feeds s_dsi_blit_done). Returning here
     * without waiting left draw_sem held, so (a) the double-buffer swap below could
     * reuse st7701_rot_buf[idx] while its trans was still reading it, and (b) the
     * first portrait blit on a scan-mode switch tripped the DPI "previous draw
     * operation is not finished" check on re-entry. The flush task is idle-paced by
     * st7701_flush_start_sem, so this ~few-ms wait overlaps the next LVGL render and
     * doesn't slow the UI. s_dsi_blit_done is empty here (the prior rotate consumed
     * its token), so this waits for exactly this draw's completion. */
    if (s_dsi_blit_done) {
        xSemaphoreTake(s_dsi_blit_done, pdMS_TO_TICKS(100));
    }
    st7701_rot_idx ^= 1;                          /* swap output buffers */
}

/* Flush worker: blocks until flush_cb hands off a rendered buffer, rotates +
 * blits it (off the LVGL lock), then signals completion for flush_wait_cb. */
static void st7701_flush_task(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(st7701_flush_start_sem, portMAX_DELAY);
        st7701_rotate_and_blit(st7701_flush_fb);
        xSemaphoreGive(st7701_flush_done_sem);
    }
}

/**
 * Landscape flush callback for MIPI-DSI (ST7701) displays.
 *
 * Hands the just-rendered buffer to the flush task and returns immediately —
 * WITHOUT lv_display_flush_ready().  LVGL keeps disp->flushing set; the next
 * draw_buf_flush() blocks in st7701_flush_wait_cb() until the task signals it.
 * This is what moves the rotation + vsync wait off the LVGL lock.
 */
static void st7701_landscape_flush_cb(lv_display_t *disp,
                                      const lv_area_t *area, uint8_t *px_map)
{
    /* FULL mode sends one complete frame per flush; this guard only matters if
     * the render mode is ever changed back to a partial/direct variant. */
    if (!lv_display_flush_is_last(disp)) {
        lv_display_flush_ready(disp);
        return;
    }

    st7701_flush_fb = (uint16_t *)px_map;
    xSemaphoreGive(st7701_flush_start_sem);  /* wake the flush task; do NOT flush_ready */
}

/**
 * flush_wait callback — LVGL calls this (from wait_for_flushing) when it needs
 * the previous flush to complete before starting the next one.  Yields on a
 * semaphore instead of the default busy-spin (while(disp->flushing)); LVGL
 * clears disp->flushing after we return.  Paired 1:1 with flush_start (each
 * frame gives done_sem once, each following frame takes it once).
 */
static void st7701_flush_wait_cb(lv_display_t *disp)
{
    (void)disp;
    xSemaphoreTake(st7701_flush_done_sem, portMAX_DELAY);
}

/* ── Portrait scan flush ──
 * Native-portrait, NO rotation: LVGL renders (PARTIAL) directly in panel
 * coordinates, so each dirty area blits straight to the panel. draw_bitmap from
 * an external buffer is async (DMA2D), so wait for the copy (drain-before,
 * wait-after on s_dsi_blit_done) before flush_ready, or LVGL reuses px_map
 * mid-copy and the next draw_bitmap trips the driver's draw_sem (INVALID_STATE).
 * Runs under the LVGL lock (esp_lvgl_port wraps lv_timer_handler); the ~2-4 ms
 * DMA2D copy of a letterbox strip is short enough not to starve the camera's
 * try-lock push. */
static void st7701_portrait_flush_cb(lv_display_t *disp,
                                     const lv_area_t *area, uint8_t *px_map)
{
    /* Through the single-writer gate so a concurrent camera-square blit can't
     * collide on the panel's DMA2D draw path. */
    board_display_portrait_scan_blit(area->x1, area->y1,
                                     area->x2 + 1, area->y2 + 1, px_map);
    lv_display_flush_ready(disp);
}

/* Drain the deferred-rotate handshake so a stale token can't desync the 1:1
 * flush_cb→flush_wait pairing across a mode switch. Absorbs one in-flight rotate
 * (short wait), then clears both sems + the stale vsync. Caller holds the LVGL
 * lock, so no NEW landscape flush can start while we drain. */
static void st7701_quiesce_flush(void)
{
    xSemaphoreTake(st7701_flush_done_sem, pdMS_TO_TICKS(60)); /* in-flight rotate */
    while (xSemaphoreTake(st7701_flush_start_sem, 0) == pdTRUE) { }
    while (xSemaphoreTake(st7701_flush_done_sem, 0) == pdTRUE) { }
    xSemaphoreTake(dpi_flush_sem, 0);
}

void board_display_enter_portrait_scan(void)
{
    if (!s_st7701_disp || s_portrait_scan_active) return;
    lvgl_port_lock(0);

    /* Settle the deferred rotate, then force disp->flushing clear so portrait's
     * NULL flush_wait_cb doesn't busy-spin on a landscape flush that never
     * completes. */
    st7701_quiesce_flush();
    /* st7701_rotate_and_blit now waits for its DMA2D trans-done before signalling
     * st7701_flush_done_sem, so the quiesce above already guarantees the panel's
     * draw_sem is free -- no separate trans-drain needed here for the first portrait
     * blit (fixes the DPI "previous draw not finished" on scan re-entry at the
     * source). */
    lv_display_flush_ready(s_st7701_disp);

    lv_display_set_resolution(s_st7701_disp, BOARD_LCD_H_RES, BOARD_LCD_V_RES);
    lv_display_set_buffers(s_st7701_disp, s_st7701_draw_buf[0],
                           s_st7701_draw_buf[1], s_st7701_draw_buf_sz,
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(s_st7701_disp, st7701_portrait_flush_cb);
    lv_display_set_flush_wait_cb(s_st7701_disp, NULL);

    /* Touch → native portrait (GT911 is configured landscape: swap_xy=1,
     * mirror_y=1). Undo to the panel's native orientation. */
    if (touch_handle) {
        touch_handle->config.flags.swap_xy  = 0;
        touch_handle->config.flags.mirror_x = 0;
        touch_handle->config.flags.mirror_y = 0;
    }

    /* Fence LVGL out of the centered 480×480 camera square (rows 160–639); it
     * renders only the top/bottom letterbox. The camera owns the square. */
    board_display_set_reserved_rect(0, (BOARD_LCD_V_RES - BOARD_LCD_H_RES) / 2,
                                    BOARD_LCD_H_RES, BOARD_LCD_H_RES);

    s_portrait_scan_active = true;
    lv_obj_invalidate(lv_display_get_screen_active(s_st7701_disp));
    lvgl_port_unlock();
    ESP_LOGI(TAG, "portrait scan mode: ENTER (480x800, no rotation)");
}

void board_display_exit_portrait_scan(void)
{
    if (!s_st7701_disp || !s_portrait_scan_active) return;
    lvgl_port_lock(0);

    lv_display_flush_ready(s_st7701_disp);  /* portrait flush is synchronous; belt+braces */

    lv_display_set_resolution(s_st7701_disp, BOARD_LCD_V_RES, BOARD_LCD_H_RES);
    lv_display_set_buffers(s_st7701_disp, s_st7701_draw_buf[0],
                           s_st7701_draw_buf[1], s_st7701_draw_buf_sz,
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(s_st7701_disp, st7701_landscape_flush_cb);
    lv_display_set_flush_wait_cb(s_st7701_disp, st7701_flush_wait_cb);

    /* Drain + clear flushing so the first landscape flush self-primes (flushing
     * clear → LVGL skips flush_wait_cb on the first flush, which gives done_sem
     * for the second flush's wait). */
    st7701_quiesce_flush();
    lv_display_flush_ready(s_st7701_disp);

    if (touch_handle) {
        touch_handle->config.flags.swap_xy  = 1;
        touch_handle->config.flags.mirror_x = 0;
        touch_handle->config.flags.mirror_y = 1;
    }

    board_display_clear_reserved_rect();

    s_portrait_scan_active = false;
    lv_obj_invalidate(lv_display_get_screen_active(s_st7701_disp));
    lvgl_port_unlock();
    ESP_LOGI(TAG, "portrait scan mode: EXIT (800x480, rotation restored)");
}

#endif /* BOARD_DISPLAY_DRIVER == DISPLAY_ST7701 */

#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
/* Portrait scan mode is an ST7701/DSI-only optimization (other boards partition
 * via reserved-rect or are already portrait). No-op stubs so callers link. */
void board_display_enter_portrait_scan(void) {}
void board_display_exit_portrait_scan(void) {}
void board_display_portrait_scan_blit(int32_t x1, int32_t y1,
                                      int32_t x2, int32_t y2, const void *buf)
{ (void)x1; (void)y1; (void)x2; (void)y2; (void)buf; }
#endif

/* ── IO Expander ── */
#if BOARD_HAS_IO_EXPANDER
static void io_expander_init(i2c_master_bus_handle_t bus)
{
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(bus, BOARD_IO_EXPANDER_ADDR, &expander_handle));

    ESP_ERROR_CHECK(esp_io_expander_set_dir(expander_handle, BOARD_IO_EXPANDER_RST_PIN, IO_EXPANDER_OUTPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, BOARD_IO_EXPANDER_RST_PIN, 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander_handle, BOARD_IO_EXPANDER_RST_PIN, 1));
    vTaskDelay(pdMS_TO_TICKS(200));
}
#endif

/* ── LVGL port setup ── */
static void lvgl_port_setup(const board_app_config_t *app_cfg,
                            lv_display_t **disp_out, lv_indev_t **touch_out)
{
    bool landscape = app_cfg && app_cfg->landscape;

#if BOARD_DISPLAY_QUIRK_RASET_BUG
    flush_done_sem = xSemaphoreCreateBinary();
    swap_buf[0] = heap_caps_malloc(BOARD_LCD_H_RES * BOARD_LINES_PER_BAND * 2, MALLOC_CAP_DMA);
    swap_buf[1] = heap_caps_malloc(BOARD_LCD_H_RES * BOARD_LINES_PER_BAND * 2, MALLOC_CAP_DMA);
    assert(swap_buf[0] != NULL && swap_buf[1] != NULL);
#endif

    /* Initialize esp_lvgl_port (creates LVGL internals + handler task) */
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority    = BOARD_LVGL_TASK_PRIORITY;
    port_cfg.task_stack       = BOARD_LVGL_TASK_STACK;
    port_cfg.task_affinity    = BOARD_LVGL_TASK_AFFINITY;
    port_cfg.timer_period_ms  = BOARD_LVGL_TIMER_PERIOD_MS;
    port_cfg.task_max_sleep_ms = BOARD_LVGL_MAX_SLEEP_MS;
    port_cfg.task_stack_caps  = MALLOC_CAP_SPIRAM;
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    /* Determine LVGL display dimensions based on orientation.
     * ST7701 (DSI): portrait uses direct_mode with DPI framebuffers;
     *   landscape uses swapped dimensions with custom CPU-rotation flush.
     * RASET / standard SPI: LVGL sees rotated dimensions; the flush
     *   callback or MADCTL handles the physical transformation. */
    int lvgl_hres, lvgl_vres;
    if (landscape) {
        lvgl_hres = BOARD_LCD_V_RES;
        lvgl_vres = BOARD_LCD_H_RES;
    } else {
        lvgl_hres = BOARD_LCD_H_RES;
        lvgl_vres = BOARD_LCD_V_RES;
    }

    /* ── Register display ── */
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    if (landscape) {
        /* MIPI-DSI landscape: bypass esp_lvgl_port display registration.
         *
         * esp_lvgl_port's DSI path (lvgl_port_add_disp_dsi) only supports
         * direct_mode with avoid_tearing=true, which uses the DPI panel's
         * own portrait framebuffers. We need landscape SPIRAM buffers with
         * a custom rotation flush, so we create the LVGL display directly.
         *
         * This matches the approach from commit 490f027, adapted for the
         * current esp_lvgl_port API where the library's internal flush
         * expects trans_sem (only created with avoid_tearing=true). */
        size_t draw_buf_sz = (uint32_t)lvgl_hres * lvgl_vres * sizeof(lv_color16_t);
        size_t rot_buf_sz  = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t);
        void *buf1 = heap_caps_malloc(draw_buf_sz, MALLOC_CAP_SPIRAM);
        void *buf2 = heap_caps_malloc(draw_buf_sz, MALLOC_CAP_SPIRAM);
        st7701_rot_buf[0] = heap_caps_malloc(rot_buf_sz, MALLOC_CAP_SPIRAM);
        st7701_rot_buf[1] = heap_caps_malloc(rot_buf_sz, MALLOC_CAP_SPIRAM);
        assert(buf1 && buf2 && st7701_rot_buf[0] && st7701_rot_buf[1]);

        /* Retain for the portrait-scan mode switch (reuses these same buffers in
         * PARTIAL mode; draw_buf_sz is identical either orientation). */
        s_st7701_draw_buf[0] = buf1;
        s_st7701_draw_buf[1] = buf2;
        s_st7701_draw_buf_sz = draw_buf_sz;

        dpi_flush_sem = xSemaphoreCreateBinary();
        s_dsi_blit_done = xSemaphoreCreateBinary();
        s_dsi_blit_mutex = xSemaphoreCreateMutex();

        /* Deferred-flush plumbing — created BEFORE the display so the flush task
         * is ready before esp_lvgl_port can call the flush_cb. */
        st7701_flush_start_sem = xSemaphoreCreateBinary();
        st7701_flush_done_sem  = xSemaphoreCreateBinary();
        assert(st7701_flush_start_sem && st7701_flush_done_sem);
        BaseType_t flush_core = (BOARD_ST7701_FLUSH_TASK_AFFINITY < 0)
                                ? tskNO_AFFINITY
                                : (BaseType_t)BOARD_ST7701_FLUSH_TASK_AFFINITY;
        xTaskCreatePinnedToCore(st7701_flush_task, "st7701_flush",
                                BOARD_ST7701_FLUSH_TASK_STACK, NULL,
                                BOARD_ST7701_FLUSH_TASK_PRIORITY, NULL,
                                flush_core);

        lvgl_port_lock(0);
        lv_display_t *disp = lv_display_create(lvgl_hres, lvgl_vres);
        /* FULL (not DIRECT): keeps LVGL's only cross-frame wait_for_flushing in
         * draw_buf_flush AFTER the render, so the render overlaps the deferred
         * flush task.  DIRECT's refr_sync_areas would wait BEFORE the render
         * (under the lock) and negate the win.  See st7701_flush_task comment. */
        lv_display_set_buffers(disp, buf1, buf2, draw_buf_sz,
                               LV_DISPLAY_RENDER_MODE_FULL);
        lv_display_set_flush_cb(disp, st7701_landscape_flush_cb);
        lv_display_set_flush_wait_cb(disp, st7701_flush_wait_cb);
        lvgl_port_unlock();

        /* Register BOTH callbacks in one call (the driver copies both fields, so
         * a later single-field re-register would NULL the other): on_refresh_done
         * gates the landscape rotate on vsync; on_color_trans_done signals the
         * portrait direct flush's DMA2D copy completion. */
        esp_lcd_dpi_panel_event_callbacks_t dpi_cbs = {
            .on_refresh_done     = dpi_vsync_ready_cb,
            .on_color_trans_done = dpi_color_trans_done_cb,
        };
        esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &dpi_cbs, disp);

        /* Portrait reserved-rect guard (inactive until enter_portrait sets a
         * rect): fences LVGL out of the centered camera square so the letterbox
         * flush never repaints over the live preview. */
        lvgl_port_lock(0);
        lv_display_add_event_cb(disp, dsi_portrait_invalidate_guard_cb,
                                LV_EVENT_INVALIDATE_AREA, NULL);
        lvgl_port_unlock();

        s_st7701_disp = disp;
        *disp_out = disp;
    } else {
        /* MIPI-DSI portrait: direct mode with DPI hardware framebuffers.
         * avoid_tearing uses the panel's triple-buffered framebuffers. */
        lvgl_port_display_cfg_t disp_cfg = {
            .panel_handle = panel_handle,
            .hres         = lvgl_hres,
            .vres         = lvgl_vres,
            .buffer_size  = lvgl_hres * 50 * sizeof(lv_color16_t),
            .flags = {
                .direct_mode = 1,
            },
        };
        const lvgl_port_display_dsi_cfg_t dsi_cfg = {
            .flags = { .avoid_tearing = 1 },
        };
        *disp_out = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);
    }

#elif BOARD_DISPLAY_QUIRK_RASET_BUG
    /* RASET boards: full-frame direct mode with custom flush callback.
     * Don't pass io_handle — we register our own IO callback below. */
    lvgl_port_display_cfg_t disp_cfg = {
        .panel_handle  = panel_handle,
        .hres          = lvgl_hres,
        .vres          = lvgl_vres,
        .buffer_size   = (uint32_t)lvgl_hres * lvgl_vres * sizeof(lv_color16_t),
        .double_buffer = true,
        .flags = {
            .buff_spiram  = 1,
            .direct_mode  = 1,
        },
    };
    *disp_out = lvgl_port_add_disp(&disp_cfg);

#else
    /* Standard SPI boards (ST7796, ST7789): partial updates with small
     * INTERNAL DMA draw buffers (not PSRAM).
     *
     * The GPSPI DMA cannot read PSRAM directly (on the P4 only AXI GDMA
     * reaches external memory; GPSPI is served by AHB GDMA). With a PSRAM
     * draw buffer, spi_master bounces EVERY flush through a freshly-malloc'd
     * internal buffer of the full transfer size (~150 KB for a half-screen
     * flush) — and errors out when that allocation fails. Observed on the
     * P4 LCD 3.5: the very first LVGL flush died with `spi transmit (queue)
     * color failed` and the panel never received a pixel. Small internal
     * double buffers keep every flush DMA-direct — same approach as the
     * camera preview stripe buffer in board_pipeline_display_lvgl.c. */
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = io_handle,
        .panel_handle  = panel_handle,
        .hres          = lvgl_hres,
        .vres          = lvgl_vres,
        .buffer_size   = (uint32_t)lvgl_hres * (lvgl_vres / 8) * sizeof(lv_color16_t),
        .double_buffer = true,
        .flags = {
            .buff_dma    = 1,
            .swap_bytes  = 1,
        },
    };
    *disp_out = lvgl_port_add_disp(&disp_cfg);

    /* Partition-mode flush guard: lets the camera preview reserve its centered
     * square so LVGL renders only the gutters beside it (board_display_*_rect).
     * Inactive until a rect is set, so this is a no-op for normal screens and
     * for boards that never enter partition mode. */
    lvgl_port_lock(0);
    lv_display_add_event_cb(*disp_out, invalidate_area_guard_cb,
                            LV_EVENT_INVALIDATE_AREA, NULL);
#if BOARD_CAMERA_PARTITION_MODE
    /* Own the flush so a camera session can redirect LVGL off the SPI bus
     * (single-writer compositing — see std_spi_partition_flush_cb). Take over the
     * panel-IO completion callback too, so camera-path blits can wait for DMA
     * completion (std_spi_blit_done_cb still signals LVGL flush_ready). */
    s_std_disp = *disp_out;
    if (!s_blit_sem) s_blit_sem = xSemaphoreCreateBinary();
    lv_display_set_flush_cb(*disp_out, std_spi_partition_flush_cb);
    esp_lcd_panel_io_callbacks_t part_io_cbs = { .on_color_trans_done = std_spi_blit_done_cb };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &part_io_cbs, NULL);
#endif
    lvgl_port_unlock();
#endif
    assert(*disp_out != NULL);

    /* ── Panel MADCTL for standard SPI boards ── */
#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701 && !BOARD_DISPLAY_QUIRK_RASET_BUG
    /* Landscape mirror axes are per-board: the swap_xy+mirror pair selects one
     * of the two 180°-apart landscape orientations, and which one is "up"
     * depends on the panel's native scan direction. Boards where the default
     * renders upside down override these in board_config.h (both axes must be
     * toggled together to stay a pure rotation). */
#ifndef BOARD_LANDSCAPE_MIRROR_X
#define BOARD_LANDSCAPE_MIRROR_X 1
#endif
#ifndef BOARD_LANDSCAPE_MIRROR_Y
#define BOARD_LANDSCAPE_MIRROR_Y 1
#endif
    if (landscape) {
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, BOARD_LANDSCAPE_MIRROR_X, BOARD_LANDSCAPE_MIRROR_Y);
    } else {
#if defined(BOARD_DISPLAY_MIRROR_X) && BOARD_DISPLAY_MIRROR_X
        esp_lcd_panel_mirror(panel_handle, true, false);
#endif
    }
#endif

    /* ── Custom flush callback overrides ── */
#if BOARD_DISPLAY_QUIRK_RASET_BUG
    lvgl_port_lock(0);
    lv_display_set_flush_cb(*disp_out,
                            landscape ? landscape_flush_cb : portrait_flush_cb);
    lvgl_port_unlock();
    const esp_lcd_panel_io_callbacks_t io_cbs = {
        .on_color_trans_done = flush_ready_cb,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &io_cbs, *disp_out);
#endif

    /* Touch input (skip if touch init failed) */
    if (touch_handle) {
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp   = *disp_out,
            .handle = touch_handle,
        };
        *touch_out = lvgl_port_add_touch(&touch_cfg);
    } else {
        ESP_LOGW(TAG, "No touch controller — skipping LVGL touch registration");
        if (touch_out) *touch_out = NULL;
    }
}

/* ── Phase-1 CP1 portrait-scan self-test (throwaway; autonomous device check) ──
 * With no human to drive the UI to a scan, this task toggles portrait mode on the
 * live app screen so a webcam + serial can confirm the display-mode switch works
 * before the camera/reserved-rect/overlay layers (CP2+) are built. Set to 0 to
 * disable. ST7701 only. */
#define CP1_PORTRAIT_TEST 0

#if CP1_PORTRAIT_TEST && (BOARD_DISPLAY_DRIVER == DISPLAY_ST7701) && BOARD_HAS_CAMERA
/* Portrait-scan self-test (throwaway; autonomous device validation): LVGL chrome
 * ONLY in the top/bottom letterbox + a real camera preview direct-blitted into
 * the centered 480×480 square through the single-writer gate. Validates the whole
 * portrait composite (reserved-rect + gate + real camera path) WITHOUT the app's
 * scan-navigation or a QR — the webcam sees the live preview. */
#define CP2_SQ  480                              /* square side           */
#define CP2_SY  ((BOARD_LCD_V_RES - CP2_SQ) / 2) /* square top row (=160) */

static void cp2_add_letterbox_bg(lv_obj_t *scr, int y, uint32_t color)
{
    lv_obj_t *bg = lv_obj_create(scr);
    lv_obj_remove_style_all(bg);
    lv_obj_set_size(bg, CP2_SQ, CP2_SY);   /* 480 × 160 letterbox strip */
    lv_obj_set_pos(bg, 0, y);
    lv_obj_set_style_bg_color(bg, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(bg, LV_OPA_COVER, 0);
}

static lv_obj_t *cp2_make_letterbox_screen(void)
{
    /* Portrait 480×800 showcase: real LVGL widgets ONLY in the top/bottom
     * letterbox (never over the camera square 160..639), mimicking the scan
     * overlay. Each letterbox has its own opaque bg obj that self-invalidates
     * (so it paints even though the screen-load full invalidation is reserved-
     * rect-clipped to one side). */
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    cp2_add_letterbox_bg(scr, 0, 0x101418);                 /* top    */
    cp2_add_letterbox_bg(scr, CP2_SY + CP2_SQ, 0x101418);   /* bottom (y=640) */

    /* Top letterbox: title bar + status text. */
    lv_obj_t *bar = lv_obj_create(scr);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, 440, 96);
    lv_obj_set_pos(bar, 20, 32);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0x1E66FF), 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(bar, 14, 0);
    lv_obj_t *title = lv_label_create(bar);
    lv_label_set_text(title, "SCANNING QR");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_center(title);

    /* Bottom letterbox: progress bar (track + 58% indicator, plain objs so no
     * LV_USE_BAR dependency) + percent + back chevron. */
    lv_obj_t *track = lv_obj_create(scr);
    lv_obj_remove_style_all(track);
    lv_obj_set_size(track, 380, 36);
    lv_obj_set_pos(track, 50, 684);
    lv_obj_set_style_bg_color(track, lv_color_hex(0x303840), 0);
    lv_obj_set_style_bg_opa(track, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(track, 8, 0);
    lv_obj_t *ind = lv_obj_create(scr);
    lv_obj_remove_style_all(ind);
    lv_obj_set_size(ind, 380 * 58 / 100, 36);
    lv_obj_set_pos(ind, 50, 684);
    lv_obj_set_style_bg_color(ind, lv_color_hex(0x00E676), 0);
    lv_obj_set_style_bg_opa(ind, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ind, 8, 0);

    lv_obj_t *pct = lv_label_create(scr);
    lv_label_set_text(pct, "58%");
    lv_obj_set_style_text_color(pct, lv_color_white(), 0);
    lv_obj_set_pos(pct, 210, 732);

    lv_obj_t *back = lv_label_create(scr);
    lv_label_set_text(back, LV_SYMBOL_LEFT "  BACK");
    lv_obj_set_style_text_color(back, lv_color_hex(0xFFC107), 0);
    lv_obj_set_pos(back, 30, 730);
    return scr;
}

/* CP3: REAL camera preview in portrait — start an actual cam_pipeline whose
 * frames direct-blit into the square (portrait_direct) while LVGL renders the
 * letterbox chrome. Validates the real camera→portrait-square path (orientation,
 * offsets, the gate under real frame rate, DSI underrun with a live PSRAM reader)
 * WITHOUT the app's scan-navigation or a QR — the webcam sees the live preview. */
static void cp2_camera_portrait_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(12000));  /* let the app settle at Home */
    ESP_LOGI(TAG, "CP3 real-camera portrait preview: START");

    lvgl_port_lock(0);
    lv_obj_t *prev = lv_screen_active();
    lvgl_port_unlock();

    board_display_enter_portrait_scan();

    lvgl_port_lock(0);
    lv_obj_t *scr = cp2_make_letterbox_screen();
    lv_screen_load(scr);
    lvgl_port_unlock();

    /* One-time black-square clear: the FB holds the stale rotated landscape frame
     * until the first camera frame (~500 ms warmup). */
    uint16_t *black = heap_caps_aligned_alloc(128, CP2_SQ * CP2_SQ * 2,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (black) {
        memset(black, 0, (size_t)CP2_SQ * CP2_SQ * 2);
        board_display_portrait_scan_blit(0, CP2_SY, CP2_SQ, CP2_SY + CP2_SQ, black);
        heap_caps_free(black);
    }

    /* Real camera pipeline: 480×480 square, rotation 0 (native portrait, no
     * rotate), frames blitted straight into the square through the gate. */
    cam_pipeline_config_t pcfg =
        board_pipeline_default_config(scr, board_i2c_get_handle());
    pcfg.display_width  = CP2_SQ;
    pcfg.display_height = CP2_SQ;
    pcfg.rotation       = 0;
    board_pipeline_lvgl_display_config_t *dc =
        (board_pipeline_lvgl_display_config_t *)pcfg.display_config;
    dc->portrait_direct = true;
    dc->portrait_x = 0;
    dc->portrait_y = CP2_SY;

    cam_pipeline_handle_t pipe = cam_pipeline_create(&pcfg);
    if (!pipe) {
        ESP_LOGE(TAG, "CP3: camera pipeline create FAILED");
    } else {
        ESP_LOGI(TAG, "CP3: LIVE camera in portrait square + LVGL letterbox (35s)");
        vTaskDelay(pdMS_TO_TICKS(35000));
        cam_pipeline_destroy(pipe);
    }

    lvgl_port_lock(0);
    lv_screen_load(prev);
    lv_obj_delete(scr);
    lvgl_port_unlock();
    board_display_exit_portrait_scan();
    ESP_LOGI(TAG, "CP3: DONE (landscape restored, parking)");
    vTaskSuspend(NULL);
}
#endif

/* ── Board interface implementation ── */

int board_init(const board_app_config_t *app_cfg,
               lv_display_t **disp, lv_indev_t **touch_indev)
{
    ESP_LOGI(TAG, "Initializing %s...", BOARD_NAME);

    bool landscape = app_cfg && app_cfg->landscape;

    /* Step 0: Radio co-processor hold-in-reset (air gap).
     * Boards with an external radio co-processor (e.g. the ESP32-C6 behind
     * SDIO on the P4 "WIFI6" boards) define BOARD_RADIO_COPROC_RESET_PIN in
     * board_config.h. The firmware never talks to that chip; driving its
     * reset line low here — the earliest point of board bring-up — and
     * LEAVING it low holds the co-processor in reset for the whole session:
     * it executes no code and radiates nothing. This is the same line
     * esp-hosted would pulse to reset its slave (idles high; low = reset
     * asserted). The pull-down keeps the line biased low even if the pin
     * config is later reset. */
#ifdef BOARD_RADIO_COPROC_RESET_PIN
    gpio_config_t coproc_rst_cfg = {
        .pin_bit_mask = 1ULL << BOARD_RADIO_COPROC_RESET_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&coproc_rst_cfg);
    gpio_set_level(BOARD_RADIO_COPROC_RESET_PIN, 0);
    ESP_LOGI(TAG, "Radio co-processor held in reset (GPIO%d low)",
             (int)BOARD_RADIO_COPROC_RESET_PIN);
#endif

    /* Step 1: I2C bus */
    i2c_master_bus_handle_t i2c_bus = board_i2c_init(
        BOARD_PIN_I2C_SDA, BOARD_PIN_I2C_SCL, BOARD_I2C_PORT);

    /* Step 2: IO expander (if present — resets display hardware) */
#if BOARD_HAS_IO_EXPANDER
    io_expander_init(i2c_bus);
#endif

    /* Step 3: Display — must be initialized BEFORE PMIC.
     * The PMIC init changes voltage rails; doing it between the IO expander
     * reset and the SPI panel init can put the display in a bad state.
     * Matches Waveshare demo order: IO expander → Display → PMIC. */
#if BOARD_DISPLAY_DRIVER == DISPLAY_AXS15231B
    board_display_axs15231b_init(&io_handle, &panel_handle,
                                  BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(lv_color16_t));
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7796
    board_display_st7796_init(&io_handle, &panel_handle,
                               BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(lv_color16_t));
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7789
    board_display_st7789_init(&io_handle, &panel_handle,
                               BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(lv_color16_t));
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    board_display_st7701_init(&io_handle, &panel_handle);
#endif

    /* Step 4: PMIC (if present — after display is fully initialized) */
#if BOARD_HAS_PMIC
    board_pmic_init(i2c_bus);
#endif

    /* Step 5: Touch */
#ifndef BOARD_TOUCH_X_MAX
#define BOARD_TOUCH_X_MAX BOARD_LCD_H_RES
#endif
#ifndef BOARD_TOUCH_Y_MAX
#define BOARD_TOUCH_Y_MAX BOARD_LCD_V_RES
#endif

    /* Touch coordinate max and flags depend on orientation */
    uint16_t touch_x_max = BOARD_TOUCH_X_MAX;
    uint16_t touch_y_max = BOARD_TOUCH_Y_MAX;

#if BOARD_TOUCH_DRIVER == TOUCH_AXS15231B
    if (landscape) {
        /* Touch IC reports in physical portrait (320x480).
         * swap_xy + mirror_x transforms to landscape for LVGL.
         * We init with portrait coords; the driver flags handle the transform. */
        touch_handle = board_touch_axs15231b_init(i2c_bus, touch_x_max, touch_y_max);
        /* Apply landscape transform to the touch handle */
        touch_handle->config.flags.swap_xy = 1;
        touch_handle->config.flags.mirror_x = 1;
    } else {
        touch_handle = board_touch_axs15231b_init(i2c_bus, touch_x_max, touch_y_max);
    }
#elif BOARD_TOUCH_DRIVER == TOUCH_FT6336
    touch_handle = board_touch_ft6336_init(i2c_bus, touch_x_max, touch_y_max);
    if (landscape) {
        /* Values from Waveshare demo 90° rotation config */
        touch_handle->config.flags.swap_xy = 1;
        touch_handle->config.flags.mirror_x = 0;
        touch_handle->config.flags.mirror_y = 1;
    }
#elif BOARD_TOUCH_DRIVER == TOUCH_CST816D
    touch_handle = board_touch_cst816d_init(i2c_bus, touch_x_max, touch_y_max);
#elif BOARD_TOUCH_DRIVER == TOUCH_GT911
    touch_handle = board_touch_gt911_init(i2c_bus, touch_x_max, touch_y_max);
    if (landscape) {
        touch_handle->config.flags.swap_xy = 1;
        touch_handle->config.flags.mirror_x = 0;
        touch_handle->config.flags.mirror_y = 1;
    }
#endif

    /* Step 6: Backlight — init PWM but keep off (duty=0).
     * Caller turns it on after rendering the first frame to avoid
     * a flash of LVGL's default white background. */
    board_backlight_init(BOARD_PIN_LCD_BL);

    /* Step 7: LVGL port */
    lvgl_port_setup(app_cfg, disp, touch_indev);

    ESP_LOGI(TAG, "Board initialized (landscape=%d).", landscape);

#if CP1_PORTRAIT_TEST && (BOARD_DISPLAY_DRIVER == DISPLAY_ST7701) && BOARD_HAS_CAMERA
    xTaskCreatePinnedToCore(cp2_camera_portrait_task, "cp3_cam", 8192, NULL,
                            2, NULL, tskNO_AFFINITY);
#endif
    return 0;
}

void board_run(void)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ── Dynamic render interval ── */

static lv_timer_t *render_kick_timer = NULL;

static void render_kick_cb(lv_timer_t *timer)
{
    /* No-op — the timer's existence keeps lv_timer_handler() returning quickly */
    (void)timer;
}

void board_set_render_interval_ms(uint32_t interval_ms)
{
    if (interval_ms > 0) {
        if (render_kick_timer) {
            lv_timer_set_period(render_kick_timer, interval_ms);
        } else {
            render_kick_timer = lv_timer_create(render_kick_cb, interval_ms, NULL);
        }
    } else {
        if (render_kick_timer) {
            lv_timer_delete(render_kick_timer);
            render_kick_timer = NULL;
        }
    }
}
