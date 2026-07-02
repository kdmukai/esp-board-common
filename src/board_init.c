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

#include "board.h"
#include "board_config.h"

#include "board_i2c.h"
#include "board_backlight.h"

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

#endif /* BOARD_DISPLAY_DRIVER == DISPLAY_ST7701 */

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

        dpi_flush_sem = xSemaphoreCreateBinary();

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

        esp_lcd_dpi_panel_event_callbacks_t dpi_cbs = {
            .on_refresh_done = dpi_vsync_ready_cb,
        };
        esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &dpi_cbs, disp);

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
    /* Standard SPI boards (ST7796, ST7789): partial updates with PSRAM. */
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = io_handle,
        .panel_handle  = panel_handle,
        .hres          = lvgl_hres,
        .vres          = lvgl_vres,
        .buffer_size   = (uint32_t)lvgl_hres * (lvgl_vres / 2) * sizeof(lv_color16_t),
        .double_buffer = true,
        .flags = {
            .buff_spiram = 1,
            .swap_bytes  = 1,
        },
    };
    *disp_out = lvgl_port_add_disp(&disp_cfg);
#endif
    assert(*disp_out != NULL);

    /* ── Panel MADCTL for standard SPI boards ── */
#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701 && !BOARD_DISPLAY_QUIRK_RASET_BUG
    if (landscape) {
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
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

/* ── Board interface implementation ── */

int board_init(const board_app_config_t *app_cfg,
               lv_display_t **disp, lv_indev_t **touch_indev)
{
    ESP_LOGI(TAG, "Initializing %s...", BOARD_NAME);

    bool landscape = app_cfg && app_cfg->landscape;

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
