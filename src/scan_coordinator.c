/*
 * scan_coordinator -- see scan_coordinator.h.
 *
 * Decoupled poll model. The decode task only enqueues: it runs transport dedup
 * (byte-identical-to-last -> REPEAT, no re-copy) and pushes genuinely new
 * payloads into the NEW ring, coalescing REPEAT/MISS/NOTHING into a status cell.
 * The consumer task drains the ring (poll_new), reads the status cell, and pushes
 * domain results back through report() -> present(). Nothing classifies or
 * presents on the decode task.
 *
 * Memory: the NEW ring, the transport last-forwarded buffer, and the poll output
 * buffer are PSRAM (each up to K_QUIRC_MAX_PAYLOAD; ~25 KB total at the default
 * depth) -- too large for scarce internal RAM, and the per-frame memcpy is cheap
 * at the decode rate. A FreeRTOS mutex guards the ring + status cell + last_fwd
 * (single producer = decode task, single consumer; brief holds).
 */

#include "scan_coordinator.h"
#include <esp_log.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"

static const char *TAG = "scan_coordinator";

#define SCAN_NEW_RING_DEFAULT_DEPTH 8

/* One NEW ring slot: a copied payload (decode task's source buffer is reused). */
typedef struct {
    uint8_t bytes[K_QUIRC_MAX_PAYLOAD];
    size_t  len;
} new_slot_t;

struct scan_coordinator {
    cam_pipeline_qr_handle_t qr;

    scan_present_fn  present;
    void            *present_ctx;
    scan_complete_fn on_complete;
    void            *complete_ctx;

    SemaphoreHandle_t lock;     /* guards ring + status cell + last_fwd */

    /* NEW(payload) ring -- single producer (decode task), single consumer.
     * Drop-OLDEST on overflow: NEW is the precious stream, so a drop is counted
     * (dropped_new), never silent. PSRAM-backed. */
    new_slot_t *ring;
    size_t      ring_depth;
    size_t      ring_head;      /* next write */
    size_t      ring_tail;      /* next read  */
    size_t      ring_count;

    /* Transport dedup: last forwarded payload (decode task). PSRAM-backed. */
    uint8_t *last_fwd;
    size_t   last_fwd_len;
    bool     have_last_fwd;

    /* Coalesced status cell. */
    scan_frame_status_t latest;
    uint32_t            consecutive_misses;  /* run of MISS; any non-MISS resets it */
    uint32_t            dropped_new;
    bool                has_corners;
    k_quirc_point_t     corners[4];

    /* poll_new() output holding buffer (consumer task; valid until next poll). PSRAM. */
    uint8_t *poll_buf;

    /* present() dedup -- touched only on the consumer task, so no lock needed. */
    scan_frame_status_t last_present_status;
    int                 last_present_percent;
    bool                completed;
};

/* ---- decode task ---- */

/* Engine per-frame outcome -> transport dedup -> NEW ring / status cell. */
static void on_frame(cam_pipeline_qr_outcome_t outcome, const uint8_t *payload,
                     size_t len, const k_quirc_data_t *meta, void *ctx) {
    (void)meta;
    struct scan_coordinator *c = (struct scan_coordinator *)ctx;

    xSemaphoreTake(c->lock, portMAX_DELAY);
    switch (outcome) {
    case CAM_QR_DECODED: {
        if (len > K_QUIRC_MAX_PAYLOAD) {
            len = K_QUIRC_MAX_PAYLOAD;  /* defensive clamp */
        }
        bool same = c->have_last_fwd && len == c->last_fwd_len &&
                    (len == 0 || memcmp(payload, c->last_fwd, len) == 0);
        if (same) {
            /* Transport REPEAT: same bytes as last frame -- no re-copy. A decode
             * (even a repeat) proves readability, so it resets the miss run. */
            c->latest = SCAN_FRAME_REPEAT;
            c->consecutive_misses = 0;
        } else {
            /* New bytes -> push into the NEW ring (drop-oldest on overflow). */
            if (c->ring_count == c->ring_depth) {
                c->ring_tail = (c->ring_tail + 1) % c->ring_depth;
                c->ring_count--;
                c->dropped_new++;
            }
            new_slot_t *slot = &c->ring[c->ring_head];
            memcpy(slot->bytes, payload, len);
            slot->len = len;
            c->ring_head = (c->ring_head + 1) % c->ring_depth;
            c->ring_count++;

            memcpy(c->last_fwd, payload, len);
            c->last_fwd_len = len;
            c->have_last_fwd = true;
            c->latest = SCAN_FRAME_NEW;
            c->consecutive_misses = 0;  /* progress proves readability */
        }
        break;
    }
    case CAM_QR_MISS:
        /* Located but not decoded. The miss-run climbs; it resets only on a
         * decode (above), so a held-but-unreadable QR accumulates -> the consumer
         * can warn ("found but can't read it"). Corners not surfaced yet (tier-3). */
        c->latest = SCAN_FRAME_MISS;
        c->consecutive_misses++;
        break;
    case CAM_QR_NOTHING:
    default:
        /* No QR located at all -> the current attempt is over. Reset the miss run
         * so a fresh attempt (repositioning, or moving to another code / a second
         * copy) must re-accumulate before we'd call it unreadable. "No QR here"
         * (NONE) vs "a QR here I can't read" (MISS run) is itself useful feedback. */
        c->latest = SCAN_FRAME_NONE;
        c->consecutive_misses = 0;
        break;
    }
    xSemaphoreGive(c->lock);
}

/* ---- consumer task ---- */

bool scan_coordinator_poll_new(scan_coordinator_t *c, scan_new_event_t *out) {
    if (!c || !out) {
        return false;
    }
    bool got = false;
    xSemaphoreTake(c->lock, portMAX_DELAY);
    if (c->ring_count > 0) {
        new_slot_t *slot = &c->ring[c->ring_tail];
        memcpy(c->poll_buf, slot->bytes, slot->len);
        out->payload = c->poll_buf;
        out->len = slot->len;
        c->ring_tail = (c->ring_tail + 1) % c->ring_depth;
        c->ring_count--;
        got = true;
    }
    xSemaphoreGive(c->lock);
    return got;
}

void scan_coordinator_read_status(scan_coordinator_t *c, scan_status_t *out) {
    if (!c || !out) {
        return;
    }
    xSemaphoreTake(c->lock, portMAX_DELAY);
    out->latest = c->latest;
    out->consecutive_misses = c->consecutive_misses;
    out->dropped_new = c->dropped_new;
    out->has_corners = c->has_corners;
    memcpy(out->corners, c->corners, sizeof(out->corners));
    xSemaphoreGive(c->lock);
}

void scan_coordinator_report(scan_coordinator_t *c, scan_frame_status_t status,
                             int percent) {
    if (!c || c->completed) {
        return;
    }
    if (status == c->last_present_status && percent == c->last_present_percent) {
        return;  /* dedup on (status, percent) */
    }
    c->last_present_status = status;
    c->last_present_percent = percent;
    if (c->present) {
        c->present(c->present_ctx, percent, status);
    }
}

void scan_coordinator_report_complete(scan_coordinator_t *c) {
    if (!c || c->completed) {
        return;
    }
    c->completed = true;
    if (c->on_complete) {
        c->on_complete(c->complete_ctx);
    }
}

/* ---- lifecycle ---- */

scan_coordinator_t *scan_coordinator_create(const scan_coordinator_config_t *config) {
    if (!config || !config->pipeline || !config->present) {
        ESP_LOGE(TAG, "Invalid config: pipeline, present required");
        return NULL;
    }

    struct scan_coordinator *c = calloc(1, sizeof(*c));
    if (!c) {
        ESP_LOGE(TAG, "alloc failed");
        return NULL;
    }

    c->present = config->present;
    c->present_ctx = config->present_ctx;
    c->on_complete = config->on_complete;
    c->complete_ctx = config->complete_ctx;
    c->ring_depth = config->new_ring_depth ? config->new_ring_depth
                                           : SCAN_NEW_RING_DEFAULT_DEPTH;

    c->lock = xSemaphoreCreateMutex();
    c->ring = heap_caps_malloc(c->ring_depth * sizeof(new_slot_t),
                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    c->last_fwd = heap_caps_malloc(K_QUIRC_MAX_PAYLOAD,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    c->poll_buf = heap_caps_malloc(K_QUIRC_MAX_PAYLOAD,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!c->lock || !c->ring || !c->last_fwd || !c->poll_buf) {
        ESP_LOGE(TAG, "buffer/mutex alloc failed (ring=%u slots, %u B/slot)",
                 (unsigned)c->ring_depth, (unsigned)sizeof(new_slot_t));
        scan_coordinator_destroy(c);  /* qr is still NULL -- safe */
        return NULL;
    }

    c->latest = SCAN_FRAME_NONE;
    c->last_present_status = (scan_frame_status_t)-1;  /* force first present */
    c->last_present_percent = -1;

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
        scan_coordinator_destroy(c);
        return NULL;
    }

    ESP_LOGI(TAG, "scan_coordinator started (%" PRIu32 "x%" PRIu32 ", NEW ring=%u, PSRAM)",
             config->frame_width, config->frame_height, (unsigned)c->ring_depth);
    return c;
}

void scan_coordinator_destroy(scan_coordinator_t *coord) {
    if (!coord) {
        return;
    }
    /* Stop the decode task FIRST so on_frame can't fire during teardown. */
    if (coord->qr) {
        cam_pipeline_qr_destroy(coord->qr);
    }
    if (coord->lock) {
        vSemaphoreDelete(coord->lock);
    }
    heap_caps_free(coord->ring);
    heap_caps_free(coord->last_fwd);
    heap_caps_free(coord->poll_buf);
    free(coord);
}

cam_pipeline_qr_handle_t scan_coordinator_qr_handle(scan_coordinator_t *coord) {
    return coord ? coord->qr : NULL;
}
