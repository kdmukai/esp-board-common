/**
 * QR Decoder — continuous QR code scanner using the camera pipeline.
 *
 * Displays live camera feed with decoded QR text overlaid on screen.
 * Text appears on decode and fades out after 2 seconds. New decodes
 * replace the previous text and reset the timer.
 */
#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "board.h"
#include "board_config.h"
#include "board_pipeline.h"
#include "board_i2c.h"
#include "board_backlight.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_cam_pipeline.h"
#include "cam_pipeline_qr.h"
#include "board_log_flash.h"

/* SPI dummy-draw: direct text overlay (LVGL is stopped) */
#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
#include "overlay_text.h"
#include "board_pipeline_display_lvgl.h"
#include "freertos/timers.h"
static overlay_text_t *s_overlay = NULL;
#endif

static const char *TAG = "qr_decoder";

#if !BOARD_HAS_CAMERA
#error "This app requires a board with camera support (BOARD_HAS_CAMERA=1)"
#endif

/* ── FPS stats display ── */

#ifdef CONFIG_CAM_PIPELINE_DEBUG
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
static lv_obj_t *fps_label = NULL;
#endif
static cam_pipeline_handle_t s_pipeline = NULL;
static cam_pipeline_qr_handle_t s_qr = NULL;  /* for reliability stats in HUD */
static uint32_t s_square = 0;                 /* active decode square (HUD) */

/* Exponential moving average smoothing */
#define EMA_ALPHA_SLOW  0.3f   /* ~3 sec settle for stable rates */
#define EMA_ALPHA_FAST  0.7f   /* ~1 sec settle for detection rate */
static float ema_cam = 0, ema_disp = 0, ema_scan = 0, ema_det = 0;
static bool ema_initialized = false;

/* Detection counter — incremented from on_qr_decoded callback */
static volatile uint32_t s_detect_count = 0;
static uint32_t s_detect_count_prev = 0;
static int64_t s_detect_time_prev = 0;

#endif /* CONFIG_CAM_PIPELINE_DEBUG */


#ifdef CONFIG_CAM_PIPELINE_DEBUG

/* Shared FPS stats update — called from platform-specific timer callback */
static void update_fps_stats(void)
{
    if (!s_pipeline) return;

    cam_pipeline_debug_stats_t stats;
    if (cam_pipeline_get_debug_stats(s_pipeline, &stats) != ESP_OK) return;

    /* Compute detection rate from our own counter */
    int64_t now = esp_timer_get_time();
    float det_fps = 0;
    if (s_detect_time_prev > 0) {
        float elapsed = (now - s_detect_time_prev) / 1000000.0f;
        if (elapsed > 0) {
            uint32_t count = s_detect_count;
            det_fps = (count - s_detect_count_prev) / elapsed;
            s_detect_count_prev = count;
        }
    }
    s_detect_time_prev = now;

    if (!ema_initialized) {
        ema_cam = stats.camera_fps;
        ema_disp = stats.display_fps;
        ema_scan = stats.consumer_fps;
        ema_det = det_fps;
        ema_initialized = true;
    } else {
        ema_cam  = EMA_ALPHA_SLOW * stats.camera_fps   + (1 - EMA_ALPHA_SLOW) * ema_cam;
        ema_disp = EMA_ALPHA_SLOW * stats.display_fps   + (1 - EMA_ALPHA_SLOW) * ema_disp;
        ema_scan = EMA_ALPHA_SLOW * stats.consumer_fps  + (1 - EMA_ALPHA_SLOW) * ema_scan;
        ema_det  = EMA_ALPHA_FAST * det_fps             + (1 - EMA_ALPHA_FAST) * ema_det;
    }

    char buf[128];
#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
#if BOARD_LANDSCAPE
    /* One stat per line — fits the narrow 80px landscape gap strip */
    snprintf(buf, sizeof(buf), "cam: %.0f\ndisp: %.0f\nscan: %.0f\ndet: %.0f",
             ema_cam, ema_disp, ema_scan, ema_det);
#else
    snprintf(buf, sizeof(buf), "cam: %.0f  disp: %.0f\nscan: %.0f  det: %.0f",
             ema_cam, ema_disp, ema_scan, ema_det);
#endif
    overlay_text_set_fps(s_overlay, buf);
#else
    /* DSI: benchmark HUD — square + pipeline fps + QR decode reliability.
     * The id%/ok% pair is the resolution-sweep headline (see cam_pipeline_qr):
     * high id% + low ok% = square locates the QR but can't resolve its modules. */
    cam_pipeline_qr_debug_stats_t qr_stats;
    if (cam_pipeline_qr_get_debug_stats(s_qr, &qr_stats)) {
        /* px/module of the last decode is a live distance readout — lets the
         * tester step distance to specific px/module values during a sweep. */
        snprintf(buf, sizeof(buf),
                 "sq%lu  cam%.0f disp%.0f dec%.0f\n"
                 "id%.0f%% ok%.0f%%  det%.1f/s  %.1fpx/m",
                 (unsigned long)s_square, ema_cam, ema_disp,
                 qr_stats.decode_fps, qr_stats.identify_pct,
                 qr_stats.decode_pct, qr_stats.detections_per_sec,
                 qr_stats.last_px_per_module);
    } else {
        snprintf(buf, sizeof(buf),
                 "sq%lu  cam%.0f disp%.0f\nscan%.0f det%.0f",
                 (unsigned long)s_square, ema_cam, ema_disp, ema_scan, ema_det);
    }
    if (fps_label) {
        lv_label_set_text(fps_label, buf);
    }
#endif
}

#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
/* SPI dummy-draw: LVGL is stopped, use FreeRTOS software timer */
static void fps_freertos_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    update_fps_stats();
}
#else
/* DSI: LVGL is running, use LVGL timer */
static void fps_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    update_fps_stats();
}
#endif

#endif /* CONFIG_CAM_PIPELINE_DEBUG */

/* ── QR result display ── */

#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
static lv_obj_t *qr_label = NULL;
#endif

/* Monotonic decode count for the compact on-screen flash (always available,
 * independent of the debug-stats build). */
static volatile uint32_t s_decode_seq = 0;

#define QR_DISPLAY_TIMEOUT_MS  2000

/* Auto-hide timer + callback only for DSI paths (SPI overlay handles internally) */
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
static lv_timer_t *fade_timer = NULL;

/**
 * LVGL timer callback: hide QR text after timeout.
 * Runs in LVGL task context (lock already held by adapter).
 */
static void fade_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (qr_label) {
        lv_obj_add_flag(qr_label, LV_OBJ_FLAG_HIDDEN);
    }
    lv_timer_delete(fade_timer);
    fade_timer = NULL;
}
#endif /* DISPLAY_ST7701 */

/**
 * Per-frame QR callback — called from QR decode task (Core 1).
 * This raw decoder reacts only to successful decodes (its prior on_decoded
 * behavior); MISS/NOTHING frames are ignored. Updates the overlay with the
 * decoded text.
 */
static void on_qr_frame(cam_pipeline_qr_outcome_t outcome,
                        const uint8_t *payload, size_t len,
                        const k_quirc_data_t *metadata, void *user_ctx)
{
    (void)user_ctx;

    if (outcome != CAM_QR_DECODED) {
        return;
    }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    __atomic_add_fetch(&s_detect_count, 1, __ATOMIC_RELAXED);
#endif

    char text[512];

    if (metadata->data_type == K_QUIRC_DATA_TYPE_BYTE) {
        /* Binary payload — display as hex */
        size_t max_bytes = (sizeof(text) - 1) / 3; /* "XX " per byte */
        size_t n = (len < max_bytes) ? len : max_bytes;
        for (size_t i = 0; i < n; i++) {
            snprintf(text + i * 3, 4, "%02X ", payload[i]);
        }
        if (n > 0) text[n * 3 - 1] = '\0'; /* trim trailing space */
        ESP_LOGI(TAG, "QR decoded (%zu bytes, binary): %s", len, text);
    } else {
        /* Text payload — display as-is */
        size_t copy_len = (len < sizeof(text) - 1) ? len : sizeof(text) - 1;
        memcpy(text, payload, copy_len);
        text[copy_len] = '\0';
        ESP_LOGI(TAG, "QR decoded (%zu bytes): %s", len, text);
    }

    /* Compact decode flash: a clean pulse + count + size, not a wall of
     * payload (the full payload is in the serial log above). Lets the tester
     * eyeball the decode rate against the QR animation. */
    uint32_t seq = __atomic_add_fetch(&s_decode_seq, 1, __ATOMIC_RELAXED);
    char flash[48];
    snprintf(flash, sizeof(flash), LV_SYMBOL_OK " #%lu  %lu B",
             (unsigned long)seq, (unsigned long)len);

#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
    /* SPI dummy-draw: direct overlay (no LVGL lock needed) */
    overlay_text_set_qr(s_overlay, flash, QR_DISPLAY_TIMEOUT_MS);
#else
    if (lvgl_port_lock(50)) {
        if (qr_label) {
            lv_label_set_text(qr_label, flash);
            lv_obj_clear_flag(qr_label, LV_OBJ_FLAG_HIDDEN);
        }

        /* Reset or create the auto-hide timer */
        if (fade_timer) {
            lv_timer_reset(fade_timer);
        } else {
            fade_timer = lv_timer_create(fade_timer_cb,
                                         QR_DISPLAY_TIMEOUT_MS, NULL);
            lv_timer_set_repeat_count(fade_timer, 1);
        }
        lvgl_port_unlock();
    }
#endif /* DISPLAY_ST7701 */
}

/* ── Delayed log dump task ── */

void log_dump_task(void *param)
{
    (void)param;
    vTaskDelay(pdMS_TO_TICKS(8000));
    board_log_flash_dump();
    vTaskDelete(NULL);
}

/* ── Main ── */

void app_main(void)
{
    /* Flash log — must be first so it captures all subsequent output */
    board_log_flash_init();

    ESP_LOGI(TAG, "QR decoder starting");

    /* Initialize board hardware.
     * DSI landscape: board_init handles rotation via custom flush callback.
     * SPI landscape: panel stays portrait, text overlays rendered rotated. */
    lv_display_t *disp;
    lv_indev_t *touch;
    board_app_config_t app_cfg = { .landscape = BOARD_LANDSCAPE };
    board_init(&app_cfg, &disp, &touch);

    /* Build pipeline config from board defines */
    lv_obj_t *screen = NULL;
    if (lvgl_port_lock(0)) {
        screen = lv_screen_active();
        board_set_render_interval_ms(10);
        lvgl_port_unlock();
    }

    cam_pipeline_config_t pipeline_cfg = board_pipeline_default_config(
        screen, board_i2c_get_handle());

    /* Square crop: use shorter logical dimension for both axes.
     * Camera fill+crop produces a square frame — no wasted pixels for
     * the QR consumer, and the LVGL display driver centers it.
     *
     * Phase-1 spike: CONFIG_QR_DECODE_SQUARE overrides the side length so the
     * sensor-mode x square sweep can be driven per-build from sdkconfig. 0 =
     * auto (shorter display dimension, the previous behavior). */
    uint32_t square = (BOARD_DISP_H_RES < BOARD_DISP_V_RES)
                          ? BOARD_DISP_H_RES : BOARD_DISP_V_RES;
#if CONFIG_QR_DECODE_SQUARE > 0
    square = CONFIG_QR_DECODE_SQUARE;
#endif
    ESP_LOGI(TAG, "QR decode square: %u px", (unsigned)square);
#ifdef CONFIG_CAM_PIPELINE_DEBUG
    s_square = square;
#endif
    pipeline_cfg.display_width = square;
    pipeline_cfg.display_height = square;

#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
    /* SPI dummy-draw: create direct text overlay (portrait gap areas or
     * landscape rotated overlays — see overlay_text.c for both paths). */
    {
        overlay_text_config_t ov_cfg = {
            .panel_width    = BOARD_LCD_H_RES,
            .panel_height   = BOARD_LCD_V_RES,
            .camera_height  = square,
            .byte_swap      = true,   /* SPI panels need byte-swap */
            .landscape      = BOARD_LANDSCAPE,
            .panel_handle   = board_get_panel_handle(),
#if BOARD_LANDSCAPE
            .font           = &lv_font_montserrat_14,
#else
            .font           = &lv_font_montserrat_24,
#endif
        };
        s_overlay = overlay_text_create(&ov_cfg);
        if (s_overlay) {
            /* Register callback for QR auto-hide timing (does not modify frame) */
            board_pipeline_lvgl_display_config_t *disp_cfg =
                (board_pipeline_lvgl_display_config_t *)pipeline_cfg.display_config;
            disp_cfg->overlay_cb = overlay_text_cb;
            disp_cfg->overlay_cb_ctx = s_overlay;
            ESP_LOGI(TAG, "Gap-area text overlay enabled for SPI dummy-draw");
        }
    }
#endif

    /* Create the pipeline — starts camera streaming + display */
    cam_pipeline_handle_t pipeline = cam_pipeline_create(&pipeline_cfg);
    if (!pipeline) {
        ESP_LOGE(TAG, "Pipeline creation failed");
        board_backlight_set(100);
        board_run();
        return;
    }

    /* Black background behind the centered square */
    if (lvgl_port_lock(0)) {
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);

#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
        /* ── SPI dummy-draw: text in panel gap areas via overlay_text ──
         * LVGL is stopped (dummy-draw mode) so no widgets or timers work.
         * See docs/knowledge/text-overlay-architecture.md for why each path
         * uses a different approach. */

#else
        /* ── DSI: standard LVGL label widgets (portrait and landscape) ──
         * Landscape rotation is handled by the flush callback in board_init,
         * so LVGL widgets render in logical coordinates for both orientations. */

        qr_label = lv_label_create(screen);
        lv_obj_set_width(qr_label, BOARD_DISP_H_RES - 20);
        lv_label_set_long_mode(qr_label, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(qr_label, lv_color_hex(0x00FF00), 0);
        lv_obj_set_style_text_font(qr_label, &lv_font_montserrat_24, 0);
        /* Backing so the HUD stays readable over a frame-filling QR */
        lv_obj_set_style_bg_color(qr_label, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(qr_label, LV_OPA_70, 0);
        lv_obj_set_style_pad_all(qr_label, 8, 0);
        lv_obj_align(qr_label, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_label_set_text(qr_label, "");
        lv_obj_add_flag(qr_label, LV_OBJ_FLAG_HIDDEN);

#ifdef CONFIG_CAM_PIPELINE_DEBUG
        fps_label = lv_label_create(screen);
        lv_obj_set_width(fps_label, BOARD_DISP_H_RES);
        lv_obj_set_style_text_color(fps_label, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_align(fps_label, LV_TEXT_ALIGN_CENTER, 0);
        /* Backing so the HUD stays readable over a frame-filling QR */
        lv_obj_set_style_bg_color(fps_label, lv_color_hex(0x000000), 0);
        lv_obj_set_style_bg_opa(fps_label, LV_OPA_70, 0);
        lv_obj_set_style_pad_ver(fps_label, 8, 0);
        lv_obj_align(fps_label, LV_ALIGN_TOP_MID, 0, 0);
        lv_label_set_text(fps_label, "---");
#endif

#endif /* overlay path selection */

        lvgl_port_unlock();
    }

#ifdef CONFIG_CAM_PIPELINE_DEBUG
    /* Start FPS stats polling timer */
    s_pipeline = pipeline;
#if BOARD_DISPLAY_DRIVER != DISPLAY_ST7701
    /* SPI: LVGL is stopped — use FreeRTOS software timer */
    {
        TimerHandle_t fps_tmr = xTimerCreate("fps", pdMS_TO_TICKS(1000),
                                              pdTRUE, NULL, fps_freertos_timer_cb);
        if (fps_tmr) xTimerStart(fps_tmr, 0);
    }
#else
    /* DSI: LVGL is running — use LVGL timer */
    if (lvgl_port_lock(0)) {
        lv_timer_create(fps_timer_cb, 1000, NULL);
        lvgl_port_unlock();
    }
#endif
#endif

    /* Start QR decode consumer */
    cam_pipeline_qr_config_t qr_cfg = {
        .pipeline     = pipeline,
        .frame_width  = square,
        .frame_height = square,
        .on_frame     = on_qr_frame,
        .user_ctx     = NULL,
    };
    cam_pipeline_qr_handle_t qr = cam_pipeline_qr_create(&qr_cfg);
    if (!qr) {
        ESP_LOGE(TAG, "QR consumer creation failed");
    }
#ifdef CONFIG_CAM_PIPELINE_DEBUG
    s_qr = qr;
#endif

    board_backlight_set(100);

    ESP_LOGI(TAG, "QR decoder running (%"PRIu32"x%"PRIu32" square)", square, square);

    /* Delayed log dump — waits for USB serial to reconnect, then dumps
     * the complete boot log from flash so nothing is missed.
     * Must use internal-RAM stack since board_log_flash_dump reads flash. */
    {
        StaticTask_t *dump_tcb = heap_caps_malloc(sizeof(StaticTask_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        StackType_t *dump_stack = heap_caps_malloc(4096 * sizeof(StackType_t),
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (dump_tcb && dump_stack) {
            xTaskCreateStatic(log_dump_task, "log_dump", 4096, NULL, 1,
                              dump_stack, dump_tcb);
        }
    }

    board_run();
}
