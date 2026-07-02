/*
 * scan_coordinator -- glue between the camera QR engine and a UI.
 *
 * Coordinates three components and owns the scan lifecycle, but classifies
 * nothing and renders nothing itself:
 *
 *   engine (cam_pipeline_qr)  -> per-frame outcome (NOTHING / MISS / DECODED)
 *     -> classify (INJECTED)  -> domain status (NEW / REPEAT / COMPLETE) + percent
 *     -> state-change dedup    (here -- only push when the overlay state changes)
 *     -> present (INJECTED)    -> UI-agnostic render of (percent, status)
 *     -> on_complete (INJECTED, terminal, once)
 *
 * Lives in esp-board-common (above the generic engine, on altitude grounds) but
 * has NO board-specific or renderer-specific dependency: it takes an existing
 * pipeline handle and the consumer supplies the classifier + presenter. The
 * neutral vocabulary below mirrors Python DecodeQRStatus so the eventual
 * MicroPython/DecodeQR consumer drops in without changing this layer.
 *
 * Named "coordinator" (not "session"): it coordinates components and owns
 * lifecycle rather than managing episode state. Distinct from the builder's
 * camera_manager (lifecycle + ring buffer + qr_poll).
 */

#pragma once

#include "cam_pipeline_qr.h"
#include "esp_cam_pipeline.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Neutral scan-domain status of the most-recent frame (mirrors Python
 * ScanScreen FRAME__* / DecodeQRStatus). Renderer-agnostic -- NOT the overlay's
 * visual enum. A presenter maps this to its own representation (overlay dot
 * color, serial text, ...). COMPLETE is intentionally NOT here: completion is a
 * terminal control-flow event routed to on_complete, not a frame status.
 */
typedef enum {
    SCAN_FRAME_NONE = 0,  /* no recent decode (engine NOTHING)         */
    SCAN_FRAME_NEW,       /* new part      (Python PART_COMPLETE)       */
    SCAN_FRAME_REPEAT,    /* already-seen  (Python PART_EXISTING)       */
    SCAN_FRAME_MISS,      /* located, not decoded (engine MISS)         */
} scan_frame_status_t;

/*
 * What the injected classifier decides for a DECODED payload (the decode-side of
 * DecodeQRStatus). The coordinator routes NEW/REPEAT to present() and COMPLETE
 * to on_complete().
 */
typedef enum {
    SCAN_CLASSIFY_NEW = 0,   /* new part                                  */
    SCAN_CLASSIFY_REPEAT,    /* already-seen part                         */
    SCAN_CLASSIFY_COMPLETE,  /* last part -- payload fully assembled       */
} scan_classify_result_t;

/*
 * Classifier (INJECTED by the consumer). Called once per DECODED frame with the
 * raw payload; returns the domain classification and writes the current progress
 * percent (0..100) via *out_percent. The coordinator never classifies.
 *   v1 test     : payload-dedup + synthetic percent.
 *   production  : thin shim over DecodeQR.add_data() + get_percent_complete().
 * Runs in the decode-task context -- must return quickly.
 */
typedef scan_classify_result_t (*scan_classify_fn)(void *ctx,
                                                   const uint8_t *payload,
                                                   size_t len, int *out_percent);

/*
 * Presenter (INJECTED). Called ONLY on a state change -- the coordinator dedups
 * on (status, percent). UI-agnostic: maps (percent, status) to an overlay
 * dot/bar, serial text, a text-LCD, etc. Runs in the decode-task context.
 */
typedef void (*scan_present_fn)(void *ctx, int percent,
                                scan_frame_status_t status);

/*
 * Completion (INJECTED, optional -- may be NULL). Fired once, terminal, when the
 * classifier returns COMPLETE. The consumer tears down and retrieves the
 * assembled result from its OWN classifier object (mirrors ScanView reading
 * decoder.get_psbt() after is_complete). Runs in the decode-task context.
 */
typedef void (*scan_complete_fn)(void *ctx);

typedef struct {
    /* Existing pipeline (camera preview already running). The coordinator
     * attaches a QR consumer to it; it does NOT create or own the pipeline. */
    cam_pipeline_handle_t pipeline;
    uint32_t frame_width;   /* == decode square */
    uint32_t frame_height;

    scan_classify_fn classify;
    void            *classify_ctx;
    scan_present_fn  present;
    void            *present_ctx;
    scan_complete_fn on_complete;   /* optional */
    void            *complete_ctx;
} scan_coordinator_config_t;

typedef struct scan_coordinator scan_coordinator_t;

/*
 * Create: attaches a QR consumer to config->pipeline and begins dispatching
 * per-frame outcomes through classify -> dedup -> present (+ on_complete).
 * `classify` and `present` are required; `pipeline` must already exist.
 * Returns NULL on failure.
 */
scan_coordinator_t *scan_coordinator_create(const scan_coordinator_config_t *config);

/* Stop the QR consumer and free the coordinator. Does NOT touch the pipeline. */
void scan_coordinator_destroy(scan_coordinator_t *coord);

/*
 * The underlying QR consumer handle, for the debug HUD (px/module, id%/ok%).
 * Returns NULL if coord is NULL.
 */
cam_pipeline_qr_handle_t scan_coordinator_qr_handle(scan_coordinator_t *coord);

#ifdef __cplusplus
}
#endif
