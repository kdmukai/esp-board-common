/**
 * scan_coord_test — on-device validation of scan_coordinator (esp-board-common).
 *
 * Stands up the camera preview pipeline (like qr_decoder), then drives a
 * scan_coordinator via its POLL model: a consumer task drains the NEW(payload)
 * ring, reads the coalesced status cell, and reports domain results back through
 * a serial-log presenter. This mirrors the production MicroPython-task loop with
 * no SeedSigner / overlay dependency, exercising the generic seam — NEW ring,
 * REPEAT/MISS/NONE status, the sustained-MISS counter, present() dedup, and
 * report_complete() — before the builder wires the real overlay presenter on top.
 *
 * By hand:
 *   - aim at a QR            -> NEW part logged + PRESENT NEW + percent tick
 *   - hold it steady         -> PRESENT REPEAT once, then silence (dedup)
 *   - a located-but-unread QR-> sustained-MISS count climbs (dot ignores it)
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

/* Coordinator handle, shared with the consumer task. */
static scan_coordinator_t *s_coord = NULL;

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

/* ── Consumer task: the poll loop (mirrors the production MicroPython-task loop) ──
 * The coordinator already did transport dedup, so each NEW ring payload is a
 * fresh (bytes-differ) part. v1 "DecodeQR" stand-in: count distinct parts ->
 * percent -> COMPLETE. The status cell drives the dot for non-decode frames; the
 * MISS counter is a sustained-miss signal (logged, dot ignores it). */
static void consumer_task(void *param)
{
    (void)param;
    int distinct = 0;
    uint32_t last_cmiss = 0;
    uint32_t last_dropped = 0;
    bool sustained_warned = false;
    /* Illustrative consumer-side policy only (the real threshold is Python's in the
     * eventual integration). Device data: a dense descriptor scanned too-far/out-of-
     * focus produced runs up to ~32 before the user corrected, so 60 clears a still-
     * recoverable scan while still flagging a genuinely stuck one. See
     * docs/camera-pipeline-phase2-poll-contract.md §7. */
    const uint32_t SUSTAINED_MISS_THRESHOLD = 60;

    while (true) {
        /* 1) Drain the precious NEW ring fully. */
        scan_new_event_t ev;
        while (scan_coordinator_poll_new(s_coord, &ev)) {
            if (distinct < COMPLETE_TARGET_PARTS) {
                distinct++;
            }
            int pct = distinct * 100 / COMPLETE_TARGET_PARTS;
            if (pct > 100) {
                pct = 100;
            }
            ESP_LOGI(TAG, "NEW part: %u bytes (distinct=%d/%d)",
                     (unsigned)ev.len, distinct, COMPLETE_TARGET_PARTS);
            if (distinct >= COMPLETE_TARGET_PARTS) {
                scan_coordinator_report(s_coord, SCAN_FRAME_NEW, 100);
                scan_coordinator_report_complete(s_coord);
            } else {
                scan_coordinator_report(s_coord, SCAN_FRAME_NEW, pct);
            }
        }

        /* 2) Read coalesced status; drive the dot for non-decode frames. */
        scan_status_t st;
        scan_coordinator_read_status(s_coord, &st);
        int pct = distinct * 100 / COMPLETE_TARGET_PARTS;
        if (pct > 100) {
            pct = 100;
        }
        if (st.latest == SCAN_FRAME_REPEAT) {
            scan_coordinator_report(s_coord, SCAN_FRAME_REPEAT, pct);
        } else if (st.latest == SCAN_FRAME_NONE) {
            scan_coordinator_report(s_coord, SCAN_FRAME_NONE, pct);
        }
        /* NEW is handled via the ring above; MISS is ignored for the dot. */

        /* 3) Sustained-MISS detection (consumer policy on the consecutive counter,
         *    which the coordinator resets on any decode). Warn once per run when it
         *    crosses the threshold; clear the latch when a decode resets the run. */
        if (st.consecutive_misses != last_cmiss) {
            if (st.consecutive_misses > last_cmiss) {
                ESP_LOGI(TAG, "MISS run=%u", (unsigned)st.consecutive_misses);
                if (!sustained_warned &&
                    st.consecutive_misses >= SUSTAINED_MISS_THRESHOLD) {
                    ESP_LOGW(TAG, "SUSTAINED MISS (run>=%u, no decode) — "
                             "found-but-unreadable? (damaged / bad transcription)",
                             (unsigned)SUSTAINED_MISS_THRESHOLD);
                    sustained_warned = true;
                }
            } else {
                /* Run reset by a decode -> rearm the warning. */
                sustained_warned = false;
            }
            last_cmiss = st.consecutive_misses;
        }
        if (st.dropped_new != last_dropped) {
            ESP_LOGW(TAG, "NEW ring overflow: %u parts dropped (total=%u)",
                     (unsigned)(st.dropped_new - last_dropped), (unsigned)st.dropped_new);
            last_dropped = st.dropped_new;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
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
     * per-frame outcomes into the NEW ring + status cell. A consumer task (below)
     * polls it; present()/on_complete() fire from there, on the consumer task. */
    scan_coordinator_config_t coord_cfg = {
        .pipeline       = pipeline,
        .frame_width    = square,
        .frame_height   = square,
        .present        = test_present,
        .present_ctx    = NULL,
        .on_complete    = test_complete,
        .complete_ctx   = NULL,
        .new_ring_depth = 0,  /* default */
    };
    s_coord = scan_coordinator_create(&coord_cfg);
    if (!s_coord) {
        ESP_LOGE(TAG, "scan_coordinator creation failed");
    } else {
        xTaskCreate(consumer_task, "scan_consumer", 4096, NULL, 4, NULL);
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
