/**
 * Camera pipeline CSI driver for ESP32-P4.
 *
 * Uses the esp_video V4L2 abstraction for MIPI-CSI capture. The ISP
 * pipeline (RAW8 → RGB565) runs internally via
 * CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER.
 *
 * init()  — inits esp_video ONCE per boot, opens V4L2 device, allocates
 *           buffers, queues them
 * start() — begins streaming and spawns a capture task
 * stop()  — stops streaming and joins the capture task
 * deinit()— frees buffers and closes the device; leaves esp_video registered
 *           (no public esp_video_deinit exists — see s_esp_video_inited)
 */
#include "board.h"
#include "board_config.h"

#if BOARD_HAS_CAMERA && BOARD_CAMERA_INTERFACE == CAMERA_CSI

#include "board_pipeline_camera_csi.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "pipeline_cam_csi";

/* esp_video registers board-global singletons (ISP/CSI video devices, the ISP
 * pipeline-controller task, the MIPI LDO channel) and exposes NO public
 * esp_video_deinit() — the ISP controller task handle isn't even retained, so
 * it cannot be stopped. Registering twice fails ("video name=ISP id=20 has
 * been registered"), which is why an in-boot pipeline stop/start used to force
 * a machine.reset(). Init it exactly once per boot and leave it registered;
 * subsequent scan cycles just re-open the V4L2 device + re-REQBUFS/STREAMON. */
static bool s_esp_video_inited = false;

#define CSI_NUM_BUFS      3
#define CSI_TASK_STACK     (16 * 1024)
/* Flattened to 1 (== lvgl + MicroPython VM tasks) for FIFO-fair LVGL-lock access;
 * see BOARD_LVGL_TASK_PRIORITY note. Dropped frames are acceptable here. */
#define CSI_TASK_PRIORITY  1

/* AE target: 2-235 range per esp_video ISP pipeline.
 * Higher = brighter exposure. 128 = neutral. Kern defaults to 80.
 * Mixed use case: paper QR codes under ambient light + display QR codes.
 * Slightly below neutral to avoid blowing out backlit displays. */
#define CSI_AE_TARGET_DEFAULT  100

typedef struct {
    int video_fd;
    uint8_t *frame_bufs[CSI_NUM_BUFS];
    size_t frame_buf_size;
    uint16_t frame_w;
    uint16_t frame_h;
    uint8_t ae_target;
    cam_pipeline_frame_cb_t frame_cb;
    void *user_ctx;
    TaskHandle_t task_handle;
    SemaphoreHandle_t task_done_sem;  /* task gives it on exit; stop() joins on it */
    volatile bool running;
} csi_driver_ctx_t;

/* Forward declaration */
static esp_err_t csi_set_ae_target(void *handle, uint32_t level);

/* ── Capture task ── */

static void csi_capture_task(void *param)
{
    csi_driver_ctx_t *ctx = (csi_driver_ctx_t *)param;

    ESP_LOGI(TAG, "Capture task started (%dx%d)", ctx->frame_w, ctx->frame_h);

    while (ctx->running) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_USERPTR,
        };

        if (ioctl(ctx->video_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (ctx->running) {
                ESP_LOGE(TAG, "VIDIOC_DQBUF failed: errno=%d", errno);
            }
            break;
        }

        int idx = buf.index;
        ctx->frame_cb(ctx->frame_bufs[idx], ctx->frame_w, ctx->frame_h,
                       ctx->user_ctx);

        /* Return buffer to V4L2 queue */
        struct v4l2_buffer qbuf = {
            .type    = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory  = V4L2_MEMORY_USERPTR,
            .index   = idx,
            .m.userptr = (unsigned long)ctx->frame_bufs[idx],
            .length  = ctx->frame_buf_size,
        };
        ioctl(ctx->video_fd, VIDIOC_QBUF, &qbuf);
    }

    ESP_LOGI(TAG, "Capture task exiting");
    /* Signal csi_stop that we've left the loop and are self-deleting. csi_stop
     * joins on this BEFORE calling STREAMOFF, so we always exit here off a real
     * frame (DQBUF returns while streaming is still on) instead of being left
     * blocked in DQBUF -- see the note in csi_stop. */
    if (ctx->task_done_sem) {
        xSemaphoreGive(ctx->task_done_sem);
    }
    vTaskDelete(NULL);
}

/* ── Driver vtable implementation ── */

static void *csi_init(const void *platform_config)
{
    const board_pipeline_csi_config_t *cfg =
        (const board_pipeline_csi_config_t *)platform_config;

    csi_driver_ctx_t *ctx = calloc(1, sizeof(csi_driver_ctx_t));
    if (!ctx) return NULL;
    ctx->video_fd = -1;
    ctx->ae_target = cfg->ae_target; /* 0 = use ISP default */

    /* Initialize esp_video with CSI config.
     * If an I2C bus handle is provided, reuse it for SCCB.
     * Otherwise, let esp_video init its own SCCB bus from pin config. */
    esp_video_init_csi_config_t csi_config[1];
    memset(csi_config, 0, sizeof(csi_config));
    csi_config[0].reset_pin = -1;
    csi_config[0].pwdn_pin = -1;

    if (cfg->i2c_bus) {
        csi_config[0].sccb_config.init_sccb = false;
        csi_config[0].sccb_config.i2c_handle = cfg->i2c_bus;
        csi_config[0].sccb_config.freq = 400000;
    } else {
        csi_config[0].sccb_config.init_sccb = true;
        csi_config[0].sccb_config.i2c_config.port = cfg->sccb_i2c_port;
        csi_config[0].sccb_config.i2c_config.sda_pin = cfg->sccb_sda_pin;
        csi_config[0].sccb_config.i2c_config.scl_pin = cfg->sccb_scl_pin;
        csi_config[0].sccb_config.freq = 100000;
    }
    esp_video_init_config_t cam_config = { .csi = csi_config };

    /* Init once per boot; never deinit (see s_esp_video_inited note above).
     * A second esp_video_init() would fail registering the already-present
     * ISP/CSI devices, so skip it on re-arm — the devices persist and the
     * fresh open()/REQBUFS below re-attaches this driver instance to them. */
    if (!s_esp_video_inited) {
        esp_err_t err = esp_video_init(&cam_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(err));
            free(ctx);
            return NULL;
        }
        s_esp_video_inited = true;
    }

    /* Open V4L2 device */
    ctx->video_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (ctx->video_fd < 0) {
        ESP_LOGE(TAG, "Failed to open %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        free(ctx);
        return NULL;
    }

    /* Query format (ISP delivers RGB565) */
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &fmt) < 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed: errno=%d", errno);
        goto fail;
    }
    ctx->frame_w = fmt.fmt.pix.width;
    ctx->frame_h = fmt.fmt.pix.height;
    ctx->frame_buf_size = ctx->frame_w * ctx->frame_h * 2; /* RGB565 */

    ESP_LOGI(TAG, "Camera format: %dx%d (pixfmt=0x%08"PRIx32", %zu bytes/frame)",
             ctx->frame_w, ctx->frame_h, fmt.fmt.pix.pixelformat,
             ctx->frame_buf_size);

    /* Allocate frame buffers in PSRAM (cache-aligned for DMA) */
    const size_t cache_line = 128;
    for (int i = 0; i < CSI_NUM_BUFS; i++) {
        ctx->frame_bufs[i] = heap_caps_aligned_calloc(
            cache_line, 1, ctx->frame_buf_size, MALLOC_CAP_SPIRAM);
        if (!ctx->frame_bufs[i]) {
            ESP_LOGE(TAG, "Failed to allocate frame buffer %d", i);
            goto fail;
        }
    }

    /* Request V4L2 buffers (USERPTR mode) */
    struct v4l2_requestbuffers req = {
        .count  = CSI_NUM_BUFS,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_USERPTR,
    };
    if (ioctl(ctx->video_fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        goto fail;
    }

    /* Queue all buffers */
    for (int i = 0; i < CSI_NUM_BUFS; i++) {
        struct v4l2_buffer buf = {
            .type    = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory  = V4L2_MEMORY_USERPTR,
            .index   = i,
            .m.userptr = (unsigned long)ctx->frame_bufs[i],
            .length  = ctx->frame_buf_size,
        };
        if (ioctl(ctx->video_fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed", i);
            goto fail;
        }
    }

    ESP_LOGI(TAG, "CSI driver initialized (%dx%d, %d buffers)",
             ctx->frame_w, ctx->frame_h, CSI_NUM_BUFS);
    return ctx;

fail:
    for (int i = 0; i < CSI_NUM_BUFS; i++) {
        if (ctx->frame_bufs[i]) heap_caps_free(ctx->frame_bufs[i]);
    }
    if (ctx->video_fd >= 0) close(ctx->video_fd);
    free(ctx);
    return NULL;
}

static esp_err_t csi_start(void *handle, cam_pipeline_frame_cb_t frame_cb,
                           void *user_ctx, int core_id)
{
    csi_driver_ctx_t *ctx = (csi_driver_ctx_t *)handle;
    ctx->frame_cb = frame_cb;
    ctx->user_ctx = user_ctx;
    ctx->running = true;

    /* Start V4L2 streaming */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx->video_fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed: errno=%d", errno);
        return ESP_FAIL;
    }

    /* Done-semaphore so csi_stop can join the capture task before STREAMOFF
     * (see csi_stop). Created before the task so the task can always give it. */
    ctx->task_done_sem = xSemaphoreCreateBinary();
    if (!ctx->task_done_sem) {
        ESP_LOGE(TAG, "Failed to create capture task done semaphore");
        ctx->running = false;
        ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type);
        return ESP_FAIL;
    }

    /* Spawn capture task */
    BaseType_t ret = xTaskCreatePinnedToCore(
        csi_capture_task, "csi_cap", CSI_TASK_STACK, ctx,
        CSI_TASK_PRIORITY, &ctx->task_handle, core_id);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create capture task");
        vSemaphoreDelete(ctx->task_done_sem);
        ctx->task_done_sem = NULL;
        ctx->running = false;
        ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type);
        return ESP_FAIL;
    }

    /* Apply initial AE target if configured (0 = use ISP default) */
    if (ctx->ae_target > 0) {
        csi_set_ae_target(ctx, ctx->ae_target);
    }

    ESP_LOGI(TAG, "Streaming started (capture task on core %d)", core_id);
    return ESP_OK;
}

static esp_err_t csi_stop(void *handle)
{
    csi_driver_ctx_t *ctx = (csi_driver_ctx_t *)handle;

    /* Join the capture task BEFORE stopping the stream.
     *
     * The task blocks in VIDIOC_DQBUF, which esp_video waits on with a hardcoded
     * portMAX_DELAY and NEVER signals on stop -- VIDIOC_STREAMOFF actually
     * *deletes* the semaphore DQBUF is waiting on. So a task still parked in
     * DQBUF when we STREAMOFF is stranded forever, and it can't even be safely
     * vTaskDelete'd afterward (its event-list item points into the freed
     * semaphore, so removing it would corrupt the heap). Every scan cycle used to
     * orphan one such task, leaking its CSI_TASK_STACK (~16 KB internal RAM) until
     * a later cycle could no longer allocate a task stack and start() failed.
     *
     * Instead, clear `running` and wait for the task to exit on its own while the
     * stream is STILL ON. Frames keep arriving, so DQBUF returns within ~one frame
     * period, the loop sees running==false, gives task_done_sem, and self-deletes
     * cleanly. Only then do we STREAMOFF -- no task is parked in DQBUF, so esp_video
     * deleting the semaphore is harmless. */
    ctx->running = false;

    if (ctx->task_handle) {
        if (ctx->task_done_sem &&
            xSemaphoreTake(ctx->task_done_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
            /* Only reached if frames stopped arriving (camera stall) so DQBUF
             * never returned. Rare degraded case -- leave the task rather than
             * risk a heap-corrupting delete; it is not the per-cycle leak. */
            ESP_LOGW(TAG, "Capture task did not exit before STREAMOFF");
        }
        ctx->task_handle = NULL;
    }

    /* Stop V4L2 streaming (safe now: no task is blocked in DQBUF). */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type);

    if (ctx->task_done_sem) {
        vSemaphoreDelete(ctx->task_done_sem);
        ctx->task_done_sem = NULL;
    }

    ESP_LOGI(TAG, "Streaming stopped");
    return ESP_OK;
}

static void csi_deinit(void *handle)
{
    csi_driver_ctx_t *ctx = (csi_driver_ctx_t *)handle;
    if (!ctx) return;

    /* Release only this instance's V4L2 session (fd + buffers). The esp_video
     * device registrations stay up by design (see s_esp_video_inited) so the
     * next cam_pipeline_create() can re-open the device without a reboot. */
    for (int i = 0; i < CSI_NUM_BUFS; i++) {
        if (ctx->frame_bufs[i]) heap_caps_free(ctx->frame_bufs[i]);
    }
    if (ctx->video_fd >= 0) close(ctx->video_fd);
    free(ctx);

    ESP_LOGI(TAG, "CSI driver deinitialized (esp_video kept registered)");
}

static esp_err_t csi_get_resolution(void *handle, uint32_t *width,
                                    uint32_t *height)
{
    csi_driver_ctx_t *ctx = (csi_driver_ctx_t *)handle;
    if (!ctx) return ESP_ERR_INVALID_STATE;
    *width = ctx->frame_w;
    *height = ctx->frame_h;
    return ESP_OK;
}

static esp_err_t csi_set_ae_target(void *handle, uint32_t level)
{
    csi_driver_ctx_t *ctx = (csi_driver_ctx_t *)handle;
    if (!ctx || ctx->video_fd < 0) return ESP_ERR_INVALID_STATE;

    struct v4l2_ext_control control = {
        .id = V4L2_CID_EXPOSURE,
        .value = (int32_t)level,
    };
    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &control,
    };
    if (ioctl(ctx->video_fd, VIDIOC_S_EXT_CTRLS, &controls) < 0) {
        ESP_LOGW(TAG, "Set AE target %"PRIu32" failed: errno=%d", level, errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "AE target set to %"PRIu32, level);
    return ESP_OK;
}

const cam_pipeline_camera_driver_t board_pipeline_csi_driver = {
    .init           = csi_init,
    .start          = csi_start,
    .stop           = csi_stop,
    .deinit         = csi_deinit,
    .get_resolution = csi_get_resolution,
    .set_ae_target  = csi_set_ae_target,
    .set_focus      = NULL,
    .has_focus_motor = NULL,
};

#endif /* BOARD_HAS_CAMERA && CAMERA_CSI */
