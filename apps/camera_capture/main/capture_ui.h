/**
 * capture_ui — UI state machine + platform bindings for camera_capture.
 *
 * Layer 2+3: wires tap events and flash/freeze/review widgets to the
 * platform-agnostic capture_service (Layer 1). Different implementations
 * live in capture_ui_{dsi,spi}.c; the one whose BOARD_DISPLAY_DRIVER matches
 * the current board contains the active code, others become empty TUs.
 *
 * State machine:
 *
 *   LIVE ──tap──► CAPTURING (flash shown, snapshot taken)
 *                     │
 *                     ├─review disabled─► save ──► LIVE
 *                     │
 *                     └─review enabled──► REVIEWING (frozen frame + buttons)
 *                                            │
 *                                            ├─Save──► save ──► LIVE
 *                                            └─Discard────────► LIVE
 */
#pragma once

#include "esp_cam_pipeline.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 0: Phase 0 (flash → save → live, unconditional).
 * 1: Phase 1 (flash → review → save/discard → live). */
#ifndef CAPTURE_REVIEW_ENABLED
#define CAPTURE_REVIEW_ENABLED 0
#endif

/* Minimum visible duration of the shutter flash, milliseconds. */
#define CAPTURE_FLASH_DURATION_MS 150

/* Initialize UI bindings for the current display path. Must be called after
 * capture_service_init and after the pipeline has been created.
 * On DSI boards: attaches LVGL widgets to cam_pipeline_get_overlay_parent(). */
esp_err_t capture_ui_init(cam_pipeline_handle_t pipeline);

#ifdef __cplusplus
}
#endif
