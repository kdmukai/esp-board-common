/**
 * Camera pipeline display driver — LVGL integration.
 *
 * Two modes:
 *
 * 1. Image widget (default / MIPI-DSI): Pushes frames via an lv_image widget.
 *    Supports LVGL overlay widgets on top of the live feed.
 *    Relies on LVGL rendering cycle — can tear on SPI panels.
 *
 * 2. Dummy-draw (SPI panels): Bypasses LVGL rendering entirely.
 *    Camera frames are byte-swapped and sent to the panel in DMA-friendly
 *    stripes via esp_lcd_panel_draw_bitmap(). Eliminates tearing
 *    on single-buffered SPI panels (ST7796, ST7789).
 *    LVGL overlays are NOT rendered while dummy-draw is active.
 */
#include "board_pipeline_display_lvgl.h"
#include "overlay_util.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_panel_ops.h"
#include "board.h"
#include "lvgl.h"

static const char *TAG = "pipeline_disp_lvgl";

/* ── Phase-0 THROUGHPUT PROBE (throwaway; remove before merge) ─────────────
 * Free core 0 for a 2nd decoder by throttling the DSI preview. On the 4.3 the
 * image-widget push invalidates the whole screen every frame → a full
 * RENDER_MODE_FULL render + a ~30ms CPU 90° rotate-flush, which pegs core 0.
 * Pushing only every Nth frame cuts that cost ~N× while keeping a laggy-but-
 * usable preview so the operator can still AIM (a blind scan would corrupt the
 * throughput measurement). 1 = disabled (every frame). The cpu_load: probe in
 * cam_pipeline_qr.c reports how free core 0 actually got. */
#define CAM_PROBE_PREVIEW_DIVISOR 1

#define DMA_STRIPE_LINES_DEFAULT  120
#define DMA_STRIPE_LINES_MIN      10
#define DMA_BUF_ALIGN             64   /* safe for SPI DMA on all targets */

/* GPSPI sends each esp_lcd_panel_draw_bitmap in chunks capped at the bus's max
 * single transaction = MIN(max_transfer_sz, SPI_MS_DATA_BITLEN/8). On the ESP32-P4
 * and ESP32-S3 that length register is 18 bits → 0x3FFFF/8 = 32767 bytes, an ODD
 * value. A stripe larger than this is split by esp_lcd at byte offset 32767, which
 * is NOT cache-line aligned, so the SPI master (setup_priv_desc) allocates a fresh
 * ~32 KB bounce buffer from the DMA-capable internal heap on EVERY blit. Over a
 * long camera scan the DMA pool fragments below 32 KB, that alloc fails with
 * ESP_ERR_NO_MEM, and the panel freezes. Capping each stripe to a single
 * sub-limit chunk (and row_bytes = width*2 is a multiple of 64 for the 320-wide
 * camera square, so the transfer length stays 64-aligned) keeps the blit
 * zero-copy: no per-blit bounce, no DMA-pool churn/fragmentation, and it avoids
 * the bounce memcpy entirely. */
#define SPI_MAX_SINGLE_XFER_BYTES 32767u

typedef struct {
    /* Common */
    lv_obj_t *container;
    lv_obj_t *img_widget;
    lv_image_dsc_t img_dsc;
    uint8_t *cam_buf;
    uint32_t width;
    uint32_t height;

    /* Dummy-draw mode */
    bool dummy_draw;
    bool keep_lvgl_running;   /* partition mode: LVGL keeps rendering the gutters */
    bool byte_swap;
    lv_display_t *disp;
    uint8_t *dma_buf;
    uint32_t dma_stripe_lines;
    uint32_t panel_width;
    uint32_t panel_height;
    uint32_t x_offset;      /* horizontal centering offset on panel */
    uint32_t y_offset;      /* vertical centering offset on panel */

    /* Per-frame overlay compositing */
    pipeline_overlay_cb_t overlay_cb;
    void *overlay_cb_ctx;

    /* Portrait-scan direct blit (ST7701/DSI) */
    bool    portrait_direct;
    int32_t portrait_x;
    int32_t portrait_y;
} lvgl_display_ctx_t;

/* ── Dummy-draw push: striped DMA blit bypassing LVGL ── */

static bool push_frame_dummy_draw(lvgl_display_ctx_t *ctx,
                                  const uint8_t *rgb565_buf,
                                  uint32_t width, uint32_t height)
{
    esp_lcd_panel_handle_t panel = board_get_panel_handle();

    if (!ctx->byte_swap) {
        /* MIPI-DSI / parallel panels: blit the pipeline buffer directly.
         * No byte-swap needed, no intermediate DMA buffer, single call.
         * For DPI panels, draw_bitmap does a DMA2D copy to the panel's
         * framebuffer — source can be any PSRAM-aligned buffer. */
        uint32_t x = ctx->x_offset;
        uint32_t y = ctx->y_offset;
        esp_err_t ret = esp_lcd_panel_draw_bitmap(
            panel, x, y, x + width, y + height, rgb565_buf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(ret));
            return false;
        }
        return true;
    }

    /* SPI panels: byte-swap + striped DMA through internal-RAM buffer */
    const size_t row_bytes = width * 2;
    const uint8_t *src_fb = rgb565_buf;
    uint32_t row = 0;
    uint32_t x = ctx->x_offset;
    bool ok = true;

    /* Partition mode is now single-writer: LVGL's flush is redirected into a
     * shadow FB (board_init.c) so it touches no SPI, and THIS task is the only
     * SPI writer. No LVGL port lock is taken — there is nothing to serialize
     * against on the bus. (Legacy dummy-draw with LVGL stopped is also
     * sole-writer.) */
    while (row < height) {
        uint32_t block = height - row;
        if (block > ctx->dma_stripe_lines)
            block = ctx->dma_stripe_lines;

        const uint16_t *src = (const uint16_t *)(src_fb + (size_t)row * row_bytes);
        copy_swap_u16((uint16_t *)ctx->dma_buf, src, (size_t)width * block);

        uint32_t y = ctx->y_offset + row;
        if (ctx->keep_lvgl_running) {
            /* Partition mode: synchronous blit (wait for DMA) so the next stripe
             * can reuse ctx->dma_buf — draw_bitmap is async (queue depth 10). */
            board_display_partition_blit(x, y, x + width, y + block, ctx->dma_buf);
        } else {
            esp_err_t ret = esp_lcd_panel_draw_bitmap(
                panel, x, y, x + width, y + block, ctx->dma_buf);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(ret));
                ok = false;
                break;
            }
        }
        row += block;
    }

    /* Blit the live gutter chrome LVGL rendered into the shadow FB (same task =
     * sole writer). Runs every frame; cheap (two 80px columns, banded DMA).
     * No-op stub on non-partition boards; guarded by keep_lvgl_running anyway. */
    if (ctx->keep_lvgl_running) {
        board_display_partition_blit_gutters();
    }
    return ok;
}

/* ── Image widget push: standard LVGL rendering ── */

static bool push_frame_image_widget(lvgl_display_ctx_t *ctx,
                                    const uint8_t *rgb565_buf,
                                    uint32_t width, uint32_t height)
{
    /* Non-blocking try-lock — skip frame if LVGL is busy */
    if (!lvgl_port_lock(1)) {
        return false;
    }

    memcpy(ctx->cam_buf, rgb565_buf, width * height * 2);
    lv_obj_invalidate(ctx->container);

    lvgl_port_unlock();

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    /* Measure interval between consecutive successful pushes */
    static int64_t last_push = 0;
    static int64_t interval_sum = 0;
    static int64_t interval_max = 0;
    static int interval_count = 0;
    static int64_t interval_last_log = 0;
    int64_t now = esp_timer_get_time();
    if (last_push > 0) {
        int64_t interval = now - last_push;
        interval_sum += interval;
        if (interval > interval_max) interval_max = interval;
        interval_count++;
        if (now - interval_last_log > 2000000) {
            /* %d (not %lld): nano-printf has no 64-bit conversion; us fit in int. */
            ESP_LOGI(TAG, "DISP INTERVAL: avg=%d us  max=%d us  n=%d",
                     (int)(interval_sum / interval_count),
                     (int)interval_max, interval_count);
            interval_sum = 0;
            interval_max = 0;
            interval_count = 0;
            interval_last_log = now;
        }
    }
    last_push = now;
#endif

    return true;
}

/* ── Portrait-scan direct blit (ST7701/DSI): camera square → panel ── */

static bool push_frame_portrait_direct(lvgl_display_ctx_t *ctx,
                                       const uint8_t *rgb565_buf,
                                       uint32_t width, uint32_t height)
{
    /* Blit the camera square straight into the reserved portrait region through
     * the single-writer gate (serializes vs LVGL's letterbox flush on the shared
     * DMA2D path). Native portrait ⇒ no rotation, no LVGL image widget. */
    board_display_portrait_scan_blit(ctx->portrait_x, ctx->portrait_y,
                                     ctx->portrait_x + (int32_t)width,
                                     ctx->portrait_y + (int32_t)height,
                                     rgb565_buf);
    return true;
}

/* ── Driver interface ── */

static void *lvgl_display_init(void *parent, uint32_t width, uint32_t height,
                               const void *driver_config)
{
    const board_pipeline_lvgl_display_config_t *cfg =
        (const board_pipeline_lvgl_display_config_t *)driver_config;
    bool use_dummy_draw = cfg && cfg->use_dummy_draw;

    lvgl_display_ctx_t *ctx = calloc(1, sizeof(lvgl_display_ctx_t));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate display context");
        return NULL;
    }

    ctx->width = width;
    ctx->height = height;
    ctx->dummy_draw = use_dummy_draw;
    ctx->keep_lvgl_running = cfg ? cfg->keep_lvgl_running : false;
    ctx->byte_swap = cfg ? cfg->byte_swap : false;
    ctx->overlay_cb = cfg ? cfg->overlay_cb : NULL;
    ctx->overlay_cb_ctx = cfg ? cfg->overlay_cb_ctx : NULL;
    ctx->portrait_direct = cfg ? cfg->portrait_direct : false;
    ctx->portrait_x = cfg ? cfg->portrait_x : 0;
    ctx->portrait_y = cfg ? cfg->portrait_y : 0;

    /* Portrait direct-blit: no LVGL widget, no DMA/cam buffer — frames go straight
     * to the panel square via the single-writer gate. The display must already be
     * in portrait-scan mode (board_display_enter_portrait_scan) and the letterbox
     * chrome is owned by the caller's LVGL screen. */
    if (ctx->portrait_direct) {
        ESP_LOGI(TAG, "LVGL display driver: portrait direct-blit (%"PRIu32"x%"PRIu32
                 " at +%"PRId32",+%"PRId32")", width, height,
                 ctx->portrait_x, ctx->portrait_y);
        return ctx;
    }

    size_t buf_size = width * height * 2; /* RGB565 */

    /* Allocate PSRAM buffer (image widget mode needs it always;
     * dummy-draw mode doesn't, but it's cheap insurance for mode switches) */
    if (!use_dummy_draw) {
        ctx->cam_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!ctx->cam_buf) {
            ESP_LOGE(TAG, "Failed to allocate %zu byte cam_buf in PSRAM", buf_size);
            free(ctx);
            return NULL;
        }
        memset(ctx->cam_buf, 0, buf_size);
    }

    /* Create LVGL widgets — must hold LVGL lock */
    if (!lvgl_port_lock(1000)) {
        ESP_LOGE(TAG, "Failed to acquire LVGL lock for init");
        heap_caps_free(ctx->cam_buf);
        free(ctx);
        return NULL;
    }

    lv_obj_t *par = parent ? (lv_obj_t *)parent : lv_screen_active();

    /* Get the display handle from the parent widget */
    ctx->disp = lv_obj_get_display(par);

    /* Container fills the parent — overlay widgets are children of this */
    ctx->container = lv_obj_create(par);
    lv_obj_remove_style_all(ctx->container);
    lv_obj_set_size(ctx->container, width, height);
    lv_obj_center(ctx->container);

    if (!use_dummy_draw) {
        /* Image widget for LVGL rendering path */
        ctx->img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
        ctx->img_dsc.header.w = width;
        ctx->img_dsc.header.h = height;
        ctx->img_dsc.data_size = buf_size;
        ctx->img_dsc.data = ctx->cam_buf;

        ctx->img_widget = lv_image_create(ctx->container);
        lv_obj_set_size(ctx->img_widget, width, height);
        lv_obj_center(ctx->img_widget);
        lv_image_set_src(ctx->img_widget, &ctx->img_dsc);
    }

    lvgl_port_unlock();

    /* Dummy-draw setup: allocate DMA buffer (SPI only) and enable mode */
    if (use_dummy_draw) {
        if (ctx->byte_swap) {
            /* SPI panels need an internal-RAM DMA buffer for byte-swap + striped blit */
            ctx->dma_stripe_lines = DMA_STRIPE_LINES_DEFAULT;
            /* Cap so each stripe fits in ONE aligned SPI transaction — otherwise
             * esp_lcd splits at the odd 32767-byte limit and the SPI master
             * bounce-allocates per blit, fragmenting the DMA heap → freeze. */
            uint32_t max_single_chunk_lines = SPI_MAX_SINGLE_XFER_BYTES / (width * 2);
            if (max_single_chunk_lines < 1) max_single_chunk_lines = 1;
            if (ctx->dma_stripe_lines > max_single_chunk_lines)
                ctx->dma_stripe_lines = max_single_chunk_lines;
            while (ctx->dma_stripe_lines >= DMA_STRIPE_LINES_MIN) {
                size_t stripe_size = width * ctx->dma_stripe_lines * 2;
                ctx->dma_buf = heap_caps_aligned_alloc(
                    DMA_BUF_ALIGN, stripe_size,
                    MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
                if (ctx->dma_buf) break;
                ctx->dma_stripe_lines /= 2;
            }
            if (!ctx->dma_buf) {
                ESP_LOGE(TAG, "Failed to allocate DMA stripe buffer");
                ctx->dummy_draw = false;
                ctx->cam_buf = heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!ctx->cam_buf) {
                    ESP_LOGE(TAG, "Fallback cam_buf alloc also failed");
                    free(ctx);
                    return NULL;
                }
                memset(ctx->cam_buf, 0, buf_size);
            } else {
                ESP_LOGI(TAG, "DMA stripe buffer: %"PRIu32" lines (%zu bytes, internal RAM)",
                         ctx->dma_stripe_lines, (size_t)(width * ctx->dma_stripe_lines * 2));
            }
        }

        if (ctx->dummy_draw) {

            /* Calculate centering offsets */
            ctx->panel_width = lv_display_get_horizontal_resolution(ctx->disp);
            ctx->panel_height = lv_display_get_vertical_resolution(ctx->disp);
            ctx->x_offset = (ctx->panel_width > width)
                          ? (ctx->panel_width - width) / 2 : 0;
            ctx->y_offset = (ctx->panel_height > height)
                          ? (ctx->panel_height - height) / 2 : 0;

            /* Declared before the partition goto so the jump never skips an
             * initializer (kept ESP_OK / unused on the partition path). */
            esp_err_t ret = ESP_OK;

            if (ctx->keep_lvgl_running) {
                /* ── Partition mode (single-writer compositing) ──────────────
                 * LVGL keeps running and owns the gutters (touch stays live),
                 * but its flush is redirected into a shadow framebuffer — it
                 * issues NO SPI transactions. The camera consumer is the SOLE
                 * SPI writer: it blits the square AND the gutter columns (from
                 * the shadow FB). That is what structurally removes the
                 * two-writer ST7796 bus collision. begin() reserves the square,
                 * allocates the shadow FB + gutter DMA band, and paints the
                 * panel black once. (No-op stub on non-partition boards.) */
                if (board_display_partition_begin(
                        ctx->panel_width, ctx->panel_height,
                        ctx->x_offset, ctx->y_offset, width, height) != ESP_OK) {
                    ESP_LOGE(TAG, "partition begin failed — camera preview degraded");
                }
                goto dummy_draw_done;
            }

            /* Enable dummy-draw mode — stop LVGL rendering so camera
             * frames can be sent directly to the panel. */
            ret = lvgl_port_stop();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to stop LVGL port: %s", esp_err_to_name(ret));
                ctx->dummy_draw = false;
            } else {
                /* Clear entire display to black before camera frames arrive.
                 * DPI panels have multiple framebuffers (typically 3).  Each
                 * clear pass writes to one framebuffer; the panel cycles to
                 * the next on vsync.  Repeat enough times to clear them all. */
                uint32_t clear_lines = ctx->dma_stripe_lines ? ctx->dma_stripe_lines : 50;
                size_t clear_buf_size = ctx->panel_width * clear_lines * 2;
                size_t dma_buf_size = width * ctx->dma_stripe_lines * 2;
                uint8_t *clear_buf = NULL;
                bool free_clear_buf = false;
                if (ctx->dma_buf && clear_buf_size <= dma_buf_size) {
                    /* DMA buffer is large enough for panel-wide clear */
                    clear_buf = ctx->dma_buf;
                    memset(clear_buf, 0, clear_buf_size);
                } else {
                    /* DMA buffer too small or absent — use temporary PSRAM buffer */
                    clear_buf = heap_caps_calloc(1, clear_buf_size,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                    free_clear_buf = true;
                }
                if (clear_buf) {
                    esp_lcd_panel_handle_t panel = board_get_panel_handle();
                    for (int pass = 0; pass < 3; pass++) {
                        for (uint32_t row = 0; row < ctx->panel_height; row += clear_lines) {
                            uint32_t block = ctx->panel_height - row;
                            if (block > clear_lines) block = clear_lines;
                            esp_lcd_panel_draw_bitmap(panel, 0, row,
                                ctx->panel_width, row + block, clear_buf);
                        }
                        if (pass < 2) vTaskDelay(pdMS_TO_TICKS(20));
                    }
                }
                if (free_clear_buf) heap_caps_free(clear_buf);
                ESP_LOGI(TAG, "Display cleared to black (x_offset=%"PRIu32", y_offset=%"PRIu32")",
                         ctx->x_offset, ctx->y_offset);
            }
        dummy_draw_done: ;
        }
    }

    ESP_LOGI(TAG, "LVGL display driver initialized (%"PRIu32"x%"PRIu32", %s)",
             width, height, ctx->dummy_draw ? "dummy-draw" : "image-widget");
    return ctx;
}

static bool lvgl_display_push_frame(void *handle, const uint8_t *rgb565_buf,
                                    uint32_t width, uint32_t height)
{
    lvgl_display_ctx_t *ctx = (lvgl_display_ctx_t *)handle;
    if (!ctx) return false;

    if (ctx->portrait_direct) {
        /* No overlay_cb here: portrait chrome is separate LVGL letterbox, not
         * composited onto the camera frame. */
        return push_frame_portrait_direct(ctx, rgb565_buf, width, height);
    }

#if CAM_PROBE_PREVIEW_DIVISOR > 1
    /* PROBE: throttle the preview to free core 0 for a 2nd decoder. Skip the
     * overlay+render+rotate on all but every Nth frame, but return true so the
     * pipeline keeps front_consumed set and frames keep flowing to the decoders.
     * The DSI peripheral still scans out the (now stale) framebuffer, so the
     * underrun watch stays valid. */
    static uint32_t s_probe_push_count = 0;
    if ((s_probe_push_count++ % CAM_PROBE_PREVIEW_DIVISOR) != 0) {
        return true;
    }
#endif

    /* Overlay compositing — modifies frame buffer in place before display.
     * Cast away const: the pipeline buffer is writable, const is advisory. */
    if (ctx->overlay_cb) {
        ctx->overlay_cb((uint8_t *)rgb565_buf, width, height, ctx->overlay_cb_ctx);
    }

    if (ctx->dummy_draw) {
        return push_frame_dummy_draw(ctx, rgb565_buf, width, height);
    } else {
        return push_frame_image_widget(ctx, rgb565_buf, width, height);
    }
}

static void lvgl_display_deinit(void *handle)
{
    lvgl_display_ctx_t *ctx = (lvgl_display_ctx_t *)handle;
    if (!ctx) return;

    /* Partition mode never stopped LVGL — end the single-writer session: drop
     * the flush redirect + reserved rect, free the shadow FB, and repaint the
     * screen (LVGL owns the panel again). Legacy dummy-draw resumes the LVGL
     * port it stopped. */
    if (ctx->dummy_draw) {
        if (ctx->keep_lvgl_running) {
            board_display_partition_end();  /* no-op stub on non-partition boards */
        } else {
            lvgl_port_resume();
        }
    }

    if (lvgl_port_lock(1000)) {
        if (ctx->container) {
            lv_obj_delete(ctx->container);
        }
        lvgl_port_unlock();
    }

    if (ctx->dma_buf) {
        heap_caps_free(ctx->dma_buf);
    }
    if (ctx->cam_buf) {
        heap_caps_free(ctx->cam_buf);
    }
    free(ctx);

    ESP_LOGI(TAG, "LVGL display driver deinitialized");
}

static void *lvgl_display_get_overlay_parent(void *handle)
{
    lvgl_display_ctx_t *ctx = (lvgl_display_ctx_t *)handle;
    return ctx ? ctx->container : NULL;
}

const cam_pipeline_display_driver_t board_pipeline_lvgl_display_driver = {
    .init               = lvgl_display_init,
    .push_frame         = lvgl_display_push_frame,
    .deinit             = lvgl_display_deinit,
    .get_overlay_parent = lvgl_display_get_overlay_parent,
};
