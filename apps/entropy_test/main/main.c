/**
 * entropy_test — on-device validation of cam_pipeline_entropy (esp-camera-pipeline).
 *
 * Stands up the camera preview pipeline (like scan_coord_test), then attaches the
 * image-entropy consumer instead of a QR consumer / scan_coordinator. The consumer
 * chains SHA-256 across live preview frames. A cycle task periodically arms a
 * capture (freezing the on-screen frame), reads the frozen result, and — to
 * validate the full contract in C — computes final = SHA256(chain || frame)
 * exactly as the host Python will, logs it, holds the frozen frame, then resumes
 * (reshoot). No QR decoding is engaged.
 *
 * On serial:
 *   - live preview runs with NO QR decoding; "chained N frames" ticks up
 *   - every ~6s: "CAPTURED frames=N", chain hex, final hex; frame freezes ~4s
 *   - then resumes; frames_chained keeps climbing
 */
#include <inttypes.h>
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
#include "cam_pipeline_entropy.h"
#include "board_log_flash.h"

#include "mbedtls/sha256.h"

static const char *TAG = "entropy_test";

#if !BOARD_HAS_CAMERA
#error "This app requires a board with camera support (BOARD_HAS_CAMERA=1)"
#endif

static cam_pipeline_entropy_handle_t s_entropy;

static void hex32(const uint8_t *d, char *out /* buffer >= 65 bytes */)
{
    static const char H[] = "0123456789abcdef";
    for (int i = 0; i < 32; i++) {
        out[i * 2]     = H[(d[i] >> 4) & 0xF];
        out[i * 2 + 1] = H[d[i] & 0xF];
    }
    out[64] = '\0';
}

/* Progress: log every 30th chained frame to keep serial readable. */
static void on_frame(uint32_t frames_chained, void *ctx)
{
    (void)ctx;
    if (frames_chained % 30 == 0) {
        ESP_LOGI(TAG, "chained %" PRIu32 " frames", frames_chained);
    }
}

/* Capture / review / reshoot cycle — exercises the full consumer contract and
 * logs the host-equivalent final hash as a stable fingerprint. */
static void capture_cycle_task(void *param)
{
    (void)param;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(6000)); /* accumulate preview frames */

        ESP_LOGI(TAG, "arming capture...");
        cam_pipeline_entropy_capture(s_entropy);

        const uint8_t *chain = NULL, *frame = NULL;
        size_t chain_len = 0, frame_len = 0;
        uint32_t n = 0;
        int tries = 0;
        while (!cam_pipeline_entropy_get_result(s_entropy, &chain, &chain_len,
                                                &frame, &frame_len, &n) &&
               tries < 100) {
            vTaskDelay(pdMS_TO_TICKS(5));
            tries++;
        }
        if (!chain || !frame) {
            ESP_LOGW(TAG, "capture result timed out");
            cam_pipeline_entropy_resume(s_entropy);
            continue;
        }

        /* final = SHA256(chain || final_image) — exactly what the host does. */
        uint8_t final[32];
        mbedtls_sha256_context ctx;
        mbedtls_sha256_init(&ctx);
        mbedtls_sha256_starts(&ctx, 0);
        mbedtls_sha256_update(&ctx, chain, chain_len);
        mbedtls_sha256_update(&ctx, frame, frame_len);
        mbedtls_sha256_finish(&ctx, final);
        mbedtls_sha256_free(&ctx);

        char chex[65], fhex[65];
        hex32(chain, chex);
        hex32(final, fhex);
        ESP_LOGI(TAG, "CAPTURED frames=%" PRIu32 " frame_len=%u",
                 n, (unsigned)frame_len);
        ESP_LOGI(TAG, "  chain = %s", chex);
        ESP_LOGI(TAG, "  final = SHA256(chain||frame) = %s", fhex);

        vTaskDelay(pdMS_TO_TICKS(4000)); /* hold frozen frame on screen */
        ESP_LOGI(TAG, "resuming (reshoot)...");
        cam_pipeline_entropy_resume(s_entropy);
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
    ESP_LOGI(TAG, "entropy_test starting");

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
    ESP_LOGI(TAG, "preview square: %u px", (unsigned)square);

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

    /* Attach the image-entropy consumer to the running pipeline. No QR consumer,
     * no scan_coordinator. seed_hash = NULL (entropy is camera-only); the host
     * may pass a uniqueness seed (e.g. hash of device uptime) in production. */
    cam_pipeline_entropy_config_t ent_cfg = {
        .pipeline     = pipeline,
        .frame_width  = square,
        .frame_height = square,
        .seed_hash    = NULL,
        .on_frame     = on_frame,
        .user_ctx     = NULL,
    };
    s_entropy = cam_pipeline_entropy_create(&ent_cfg);
    if (!s_entropy) {
        ESP_LOGE(TAG, "entropy consumer creation failed");
    }

    board_backlight_set(100);
    ESP_LOGI(TAG, "entropy_test running (%ux%u square)",
             (unsigned)square, (unsigned)square);

    if (s_entropy) {
        xTaskCreate(capture_cycle_task, "cap_cycle", 8192, NULL, 3, NULL);
    }

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
