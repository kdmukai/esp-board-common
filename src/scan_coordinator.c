/*
 * scan_coordinator -- see scan_coordinator.h.
 *
 * Receives the engine's per-frame outcome, runs the injected classifier on
 * DECODED frames, dedups on the resulting (status, percent) tuple, and pushes
 * changes to the injected presenter. COMPLETE routes to on_complete (once).
 *
 * The state-change dedup MUST live here, not in the engine: NEW vs REPEAT is a
 * classifier decision invisible to the engine (a held-static QR is the same
 * DECODED outcome every frame), so engine-level suppression would hide the
 * NEW->REPEAT (green->gray) transition.
 */

#include "scan_coordinator.h"
#include <esp_log.h>
#include <inttypes.h>
#include <stdlib.h>

static const char *TAG = "scan_coordinator";

struct scan_coordinator {
    cam_pipeline_qr_handle_t qr;

    scan_classify_fn classify;
    void            *classify_ctx;
    scan_present_fn  present;
    void            *present_ctx;
    scan_complete_fn on_complete;
    void            *complete_ctx;

    /* Last pushed overlay state, for dedup. Initialized to an impossible value
     * so the first frame always pushes. Touched only on the decode task. */
    scan_frame_status_t last_status;
    int                 last_percent;
    bool                completed;
};

/* Push to the presenter only when the (status, percent) tuple changes. */
static void emit(struct scan_coordinator *c, scan_frame_status_t status,
                 int percent) {
    if (status == c->last_status && percent == c->last_percent) {
        return;
    }
    c->last_status = status;
    c->last_percent = percent;
    c->present(c->present_ctx, percent, status);
}

/* Engine per-frame outcome -> classify -> dedup -> present / on_complete. */
static void on_frame(cam_pipeline_qr_outcome_t outcome, const uint8_t *payload,
                     size_t len, const k_quirc_data_t *meta, void *ctx) {
    (void)meta;
    struct scan_coordinator *c = (struct scan_coordinator *)ctx;

    /* Completion is terminal: once assembled, ignore further frames until the
     * consumer tears us down. */
    if (c->completed) {
        return;
    }

    switch (outcome) {
    case CAM_QR_DECODED: {
        int percent = c->last_percent;
        scan_classify_result_t r =
            c->classify(c->classify_ctx, payload, len, &percent);

        if (r == SCAN_CLASSIFY_COMPLETE) {
            /* Final progress push (typically 100% as a NEW part), then the
             * terminal signal. */
            emit(c, SCAN_FRAME_NEW, percent);
            c->completed = true;
            if (c->on_complete) {
                c->on_complete(c->complete_ctx);
            }
            return;
        }

        emit(c, r == SCAN_CLASSIFY_NEW ? SCAN_FRAME_NEW : SCAN_FRAME_REPEAT,
             percent);
        break;
    }
    case CAM_QR_MISS:
        /* Located but not decoded: progress unchanged, dot -> MISS. */
        emit(c, SCAN_FRAME_MISS, c->last_percent);
        break;
    case CAM_QR_NOTHING:
    default:
        /* QR left view (or never there): progress unchanged, dot hidden. */
        emit(c, SCAN_FRAME_NONE, c->last_percent);
        break;
    }
}

scan_coordinator_t *scan_coordinator_create(const scan_coordinator_config_t *config) {
    if (!config || !config->pipeline || !config->classify || !config->present) {
        ESP_LOGE(TAG, "Invalid config: pipeline, classify, present required");
        return NULL;
    }

    struct scan_coordinator *c = calloc(1, sizeof(*c));
    if (!c) {
        ESP_LOGE(TAG, "alloc failed");
        return NULL;
    }

    c->classify = config->classify;
    c->classify_ctx = config->classify_ctx;
    c->present = config->present;
    c->present_ctx = config->present_ctx;
    c->on_complete = config->on_complete;
    c->complete_ctx = config->complete_ctx;

    /* Sentinel: an out-of-range status so the first real frame always emits
     * (regardless of percent, which starts at a clean 0 for display). */
    c->last_status = (scan_frame_status_t)-1;
    c->last_percent = 0;

    cam_pipeline_qr_config_t qr_cfg = {
        .pipeline = config->pipeline,
        .frame_width = config->frame_width,
        .frame_height = config->frame_height,
        .on_frame = on_frame,
        .user_ctx = c,
    };
    c->qr = cam_pipeline_qr_create(&qr_cfg);
    if (!c->qr) {
        ESP_LOGE(TAG, "QR consumer creation failed");
        free(c);
        return NULL;
    }

    ESP_LOGI(TAG, "scan_coordinator started (%" PRIu32 "x%" PRIu32 ")",
             config->frame_width, config->frame_height);
    return c;
}

void scan_coordinator_destroy(scan_coordinator_t *coord) {
    if (!coord) {
        return;
    }
    if (coord->qr) {
        cam_pipeline_qr_destroy(coord->qr);
    }
    free(coord);
}

cam_pipeline_qr_handle_t scan_coordinator_qr_handle(scan_coordinator_t *coord) {
    return coord ? coord->qr : NULL;
}
