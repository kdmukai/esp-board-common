/**
 * capture_service — snapshot + save primitives for camera_capture.
 *
 * Layer 1 of the app: platform- and UI-agnostic. Owns one PSRAM snapshot
 * buffer and the monotonically-increasing filename counter (persisted via
 * directory scan at startup). Call sites:
 *
 *   capture_service_init(pipeline, w, h)    once after pipeline + SD mount
 *   capture_service_snapshot()              on tap → locks pipeline frame,
 *                                           copies into internal buffer,
 *                                           releases. Buffer is stable
 *                                           until the next snapshot call.
 *   capture_service_save()                  writes the current snapshot
 *                                           to /sdcard/img_NNNN_WxH.rgb565
 *                                           using the next counter value.
 *
 * Snapshot and save are intentionally split so a review UI can capture
 * on tap but only save on user confirmation (discard = no save call).
 */
#pragma once

#include "esp_cam_pipeline.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t capture_service_init(cam_pipeline_handle_t pipeline,
                               uint32_t width, uint32_t height);

/* Lock current pipeline frame, copy into the internal PSRAM snapshot buffer,
 * release the pipeline frame. Fast (~1–2 ms PSRAM↔PSRAM). Returns ESP_OK
 * on success; ESP_FAIL if no frame is currently available. */
esp_err_t capture_service_snapshot(void);

/* Synchronously write the current snapshot buffer to the next sequential
 * filename. Returns ESP_OK on success; ESP_ERR_INVALID_STATE if no valid
 * snapshot exists; ESP_FAIL on SD I/O error. Blocks ~50–200 ms depending
 * on SD speed. */
esp_err_t capture_service_save(void);

/* Accessors for the current snapshot buffer (for review-mode display).
 * Returns NULL / 0 if no valid snapshot exists. */
const uint8_t *capture_service_snapshot_buffer(void);
uint32_t capture_service_width(void);
uint32_t capture_service_height(void);

#ifdef __cplusplus
}
#endif
