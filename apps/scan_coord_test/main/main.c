/**
 * scan_coord_test — on-device validation of scan_coordinator (esp-board-common).
 *
 * Stands up the camera preview pipeline (like qr_decoder), then drives a
 * scan_coordinator with an INJECTED payload-dedup classifier and a serial-log
 * presenter. No SeedSigner / overlay dependency: this exercises the generic
 * coordinator seam — NEW/REPEAT/MISS/COMPLETE, percent, the state-change dedup,
 * and on_complete — before the builder wires the real overlay presenter on top.
 *
 * By hand:
 *   - aim at a QR            -> PRESENT NEW  + percent tick
 *   - hold it steady         -> PRESENT REPEAT once, then silence (dedup)
 *   - a located-but-unread QR-> PRESENT MISS
 *   - QR leaves view         -> PRESENT NONE
 *   - show N distinct codes  -> 100% then COMPLETE
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "board.h"
#include "board_config.h"
#include "board_pipeline.h"
#include "board_i2c.h"
#include "board_backlight.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

#include "esp_cam_pipeline.h"
#include "scan_coordinator.h"
#include "board_log_flash.h"

static const char *TAG = "scan_coord_test";

#if !BOARD_HAS_CAMERA
#error "This app requires a board with camera support (BOARD_HAS_CAMERA=1)"
#endif

/* Synthetic completion target: N distinct payloads == 100%. Small so COMPLETE
 * is easy to reach by hand with a few different QR codes. */
#define COMPLETE_TARGET_PARTS 4

/* ── Injected classifier: payload-dedup + synthetic percent ──
 * Runs in the decode-task context. v1 stand-in for DecodeQR: a payload that
 * differs from last time is a "new part"; the same payload is "already seen".
 * Real reassembly (fountain codes, formats) arrives later with Python DecodeQR.
 *
 * Identify a payload by length + an FNV-1a hash over the FULL bytes, so dedup is
 * correct at any size: UR/BBQr parts run ~600-2000+ bytes, so a fixed compare
 * buffer would truncate and misclassify true repeats as new. */
static uint32_t s_last_hash = 0;
static size_t   s_last_len  = 0;
static bool     s_have_last = false;
static int      s_distinct  = 0;

static uint32_t fnv1a(const uint8_t *data, size_t len)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h = (h ^ data[i]) * 16777619u;
    }
    return h;
}

static scan_classify_result_t test_classify(void *ctx, const uint8_t *payload,
                                            size_t len, int *out_percent)
{
    (void)ctx;
    uint32_t hash = fnv1a(payload, len);
    bool same = s_have_last && (len == s_last_len) && (hash == s_last_hash);
    if (!same) {
        s_last_hash = hash;
        s_last_len = len;
        s_have_last = true;
        if (s_distinct < COMPLETE_TARGET_PARTS) {
            s_distinct++;
        }
    }

    int pct = s_distinct * 100 / COMPLETE_TARGET_PARTS;
    if (pct > 100) {
        pct = 100;
    }
    *out_percent = pct;

    if (same) {
        return SCAN_CLASSIFY_REPEAT;
    }
    return (s_distinct >= COMPLETE_TARGET_PARTS) ? SCAN_CLASSIFY_COMPLETE
                                                 : SCAN_CLASSIFY_NEW;
}

/* ── Injected presenter: serial log on each state CHANGE (coordinator dedups) ── */
static const char *status_name(scan_frame_status_t s)
{
    switch (s) {
    case SCAN_FRAME_NEW:    return "NEW   ";
    case SCAN_FRAME_REPEAT: return "REPEAT";
    case SCAN_FRAME_MISS:   return "MISS  ";
    case SCAN_FRAME_NONE:
    default:                return "NONE  ";
    }
}

static void test_present(void *ctx, int percent, scan_frame_status_t status)
{
    (void)ctx;
    ESP_LOGI(TAG, "PRESENT  %s  %3d%%", status_name(status), percent);
}

static void test_complete(void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "COMPLETE — %d distinct parts assembled", COMPLETE_TARGET_PARTS);
}

/* ── Delayed flash-log dump (waits for USB serial reconnect) ── */
static void log_dump_task(void *param)
{
    (void)param;
    vTaskDelay(pdMS_TO_TICKS(8000));
    board_log_flash_dump();
    vTaskDelete(NULL);
}

void app_main(void)
{
    board_log_flash_init();
    ESP_LOGI(TAG, "scan_coord_test starting (target=%d parts)",
             COMPLETE_TARGET_PARTS);

    lv_display_t *disp;
    lv_indev_t *touch;
    board_app_config_t app_cfg = { .landscape = BOARD_LANDSCAPE };
    board_init(&app_cfg, &disp, &touch);

    lv_obj_t *screen = NULL;
    if (lvgl_port_lock(0)) {
        screen = lv_screen_active();
        board_set_render_interval_ms(10);
        lvgl_port_unlock();
    }

    cam_pipeline_config_t pipeline_cfg =
        board_pipeline_default_config(screen, board_i2c_get_handle());

    /* Square crop = shorter logical dimension (480 on the P4 LCD 4.3). */
    uint32_t square = (BOARD_DISP_H_RES < BOARD_DISP_V_RES)
                          ? BOARD_DISP_H_RES : BOARD_DISP_V_RES;
    pipeline_cfg.display_width = square;
    pipeline_cfg.display_height = square;
    ESP_LOGI(TAG, "decode square: %u px", (unsigned)square);

    cam_pipeline_handle_t pipeline = cam_pipeline_create(&pipeline_cfg);
    if (!pipeline) {
        ESP_LOGE(TAG, "Pipeline creation failed");
        board_backlight_set(100);
        board_run();
        return;
    }

    /* Black background behind the centered preview square. */
    if (lvgl_port_lock(0)) {
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
        lvgl_port_unlock();
    }

    /* The coordinator attaches a QR consumer to the running pipeline and routes
     * per-frame outcomes through our injected classifier -> dedup -> presenter. */
    scan_coordinator_config_t coord_cfg = {
        .pipeline     = pipeline,
        .frame_width  = square,
        .frame_height = square,
        .classify     = test_classify,
        .classify_ctx = NULL,
        .present      = test_present,
        .present_ctx  = NULL,
        .on_complete  = test_complete,
        .complete_ctx = NULL,
    };
    scan_coordinator_t *coord = scan_coordinator_create(&coord_cfg);
    if (!coord) {
        ESP_LOGE(TAG, "scan_coordinator creation failed");
    }

    board_backlight_set(100);
    ESP_LOGI(TAG, "scan_coord_test running (%ux%u square)",
             (unsigned)square, (unsigned)square);

    /* Delayed flash-log dump — internal-RAM stack (board_log_flash_dump reads
     * flash, so the task stack must not live in PSRAM). */
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
