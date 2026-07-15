/*
 * scan_coordinator -- decoupled glue between the camera QR engine and a UI.
 *
 * The consumer's SOLE API to the QR scan pipeline. Owns the scan lifecycle but
 * classifies nothing and renders nothing itself. Decode and consumer run on
 * SEPARATE tasks, joined by buffers -- never a synchronous cross-task call:
 *
 *   decode task:   engine outcome -> transport-dedup -> enqueue to NEW ring
 *                                                    \-> coalesce status cell
 *   consumer task: poll_new()      -> classify (INJECTED domain, e.g. DecodeQR)
 *                  read_status()   -> drive the dot / sustained-miss policy
 *                  report()        -> dedup (status,percent) -> present() (INJECTED)
 *                  report_complete()-> on_complete() (INJECTED, terminal, once)
 *
 * Why poll, not the old synchronous in-task classifier: camera rate, decode rate
 * and UI refresh are independent; coupling them onto the decode task stalls the
 * fastest on the slowest, and a Python classifier (DecodeQR) cannot run on the
 * decode task without blocking it per frame. See
 * docs/camera-pipeline-phase2-poll-contract.md (builder repo).
 *
 * Two-layer dedup:
 *   - TRANSPORT (here, decode task): a DECODED payload byte-identical to the one
 *     just forwarded becomes a REPEAT in the status cell -- the bytes are NOT
 *     re-copied across the ring. Consecutive-only (single last-payload compare),
 *     NOT set-membership.
 *   - DOMAIN (consumer): "already-assembled part" (PART_EXISTING) needs decoder
 *     state and lives in the consumer.
 *
 * Lives in esp-board-common (above the generic engine, below any UI). No
 * board-specific or renderer-specific dependency: present()/on_complete() are
 * injected and fire on the consumer's task, so they may touch a VM or the LVGL
 * lock safely. Neutral vocabulary mirrors Python DecodeQRStatus so the
 * MicroPython/DecodeQR consumer drops in without changing this layer.
 */

#pragma once

#include "cam_pipeline_qr.h"
#include "esp_cam_pipeline.h"
#include <k_quirc.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Neutral scan-domain status of a frame (mirrors Python ScanScreen FRAME__* /
 * DecodeQRStatus). Renderer-agnostic. COMPLETE is intentionally NOT here:
 * completion is a terminal control event routed through report_complete().
 */
typedef enum {
    SCAN_FRAME_NONE = 0,  /* no recent decode (engine NOTHING)          */
    SCAN_FRAME_NEW,       /* new part      (Python PART_COMPLETE)       */
    SCAN_FRAME_REPEAT,    /* already-seen  (Python PART_EXISTING)       */
    SCAN_FRAME_MISS,      /* located, not decoded (engine MISS)         */
} scan_frame_status_t;

/*
 * Presenter (INJECTED, required). Called by scan_coordinator_report() on the
 * CONSUMER's task, only when (status, percent) changes (the coordinator dedups).
 * UI-agnostic: maps (percent, status) to an overlay dot/bar, serial text, etc.
 */
typedef void (*scan_present_fn)(void *ctx, int percent, scan_frame_status_t status);

/*
 * Completion (INJECTED, optional -- may be NULL). Fired once by
 * scan_coordinator_report_complete() when the consumer's decoder reports a fully
 * assembled result. Mirrors ScanView reading decoder.get_psbt() after is_complete.
 */
typedef void (*scan_complete_fn)(void *ctx);

typedef struct {
    /* Existing pipeline (camera preview already running). The coordinator
     * attaches a QR consumer to it; it does NOT create or own the pipeline. */
    cam_pipeline_handle_t pipeline;
    uint32_t frame_width;   /* == decode square */
    uint32_t frame_height;

    scan_present_fn  present;       /* required */
    void            *present_ctx;
    scan_complete_fn on_complete;   /* optional */
    void            *complete_ctx;

    size_t new_ring_depth;  /* NEW(payload) ring slots; 0 => default */

    /* Parallel decode tasks (1 or 2; 0 => 1, clamped to 2). 2 runs a second
     * decoder on a second core for higher per-frame catch reliability -- worth it
     * only where that core is otherwise free (the portrait DSI scan). */
    uint8_t num_decoders;
} scan_coordinator_config_t;

typedef struct scan_coordinator scan_coordinator_t;

/*
 * One NEW(payload) event drained from the ring -- the unique accumulation data.
 * `payload` points into a coordinator-owned buffer valid ONLY until the next
 * scan_coordinator_poll_new() (copy it out promptly, e.g. into a Python bytes).
 */
typedef struct {
    const uint8_t *payload;
    size_t         len;
} scan_new_event_t;

/*
 * Coalesced frame status + counters (latest-wins snapshot). REPEAT/MISS/NOTHING
 * are status, not data, so they never drop a part: only the latest matters.
 *
 * consecutive_misses is a "found-but-unreadable" detector: a run of consecutive
 * MISS frames, reset by ANY non-MISS outcome. A DECODED frame (NEW or REPEAT)
 * resets it because a decode proves the code is readable; NOTHING resets it because
 * "no QR located" ends the current attempt (a reposition, or moving to another code
 * / a second copy, must re-accumulate before we'd call it unreadable). The reset
 * MUST live here, not in the consumer: a valid QR held after it decoded streams
 * REPEATs (no more NEWs) with stray MISS, and the consumer's coarse poll can't see
 * every REPEAT to reset on; only the coordinator sees every frame.
 *
 * A scanner truly vacillating NONE<->MISS keeps the run low -- and that's correct:
 * the user is at the edge of detection (focus and/or damage), where staying silent
 * is the honest answer ("we can't reliably tell what we're seeing"). The consumer
 * just thresholds the value (e.g. >= 10).
 *
 * `corners` is reserved for a future on-screen detection box (the latest MISS
 * location); unpopulated until the engine surfaces corners on the MISS path
 * (has_corners stays false for now).
 */
typedef struct {
    scan_frame_status_t latest;            /* outcome of the most recent frame       */
    uint32_t            consecutive_misses;/* run of MISS frames; any non-MISS resets it */
    uint32_t            dropped_new;        /* monotonic NEW parts dropped on overflow */
    bool                has_corners;        /* latest-MISS corners present (engine, later) */
    k_quirc_point_t     corners[4];         /* latest MISS location, when has_corners  */
} scan_status_t;

/*
 * Create: attaches a QR consumer to config->pipeline and begins dispatching
 * per-frame outcomes into the NEW ring + status cell. `pipeline` and `present`
 * are required. Returns NULL on failure.
 */
scan_coordinator_t *scan_coordinator_create(const scan_coordinator_config_t *config);

/* Stop the QR consumer and free the coordinator. Does NOT touch the pipeline. */
void scan_coordinator_destroy(scan_coordinator_t *coord);

/*
 * CONSUMER task: drain one NEW payload. Returns false if the ring is empty.
 * Drain to empty each loop. `out->payload` valid until the next poll_new().
 */
bool scan_coordinator_poll_new(scan_coordinator_t *coord, scan_new_event_t *out);

/* CONSUMER task: snapshot the coalesced status + counters. */
void scan_coordinator_read_status(scan_coordinator_t *coord, scan_status_t *out);

/*
 * CONSUMER task: push the domain result -> dedup on (status,percent) -> present().
 * The single path to present(); percent is owned by the consumer (its decoder).
 */
void scan_coordinator_report(scan_coordinator_t *coord, scan_frame_status_t status,
                             int percent);

/* CONSUMER task: terminal completion -> on_complete() once. Idempotent. */
void scan_coordinator_report_complete(scan_coordinator_t *coord);

/*
 * The underlying QR consumer handle, for the debug HUD (px/module, id%/ok%).
 * Returns NULL if coord is NULL.
 */
cam_pipeline_qr_handle_t scan_coordinator_qr_handle(scan_coordinator_t *coord);

#ifdef __cplusplus
}
#endif
