#include "capture_service.h"

#include <dirent.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "capture_service";

static cam_pipeline_handle_t s_pipeline;
static uint32_t s_width, s_height;
static size_t s_size;
static uint8_t *s_snapshot_buf;
static bool s_snapshot_valid;

static int s_next_index = 1;
static SemaphoreHandle_t s_index_mtx;

static int scan_next_index(void)
{
    DIR *d = opendir("/sdcard");
    if (!d) {
        ESP_LOGW(TAG, "/sdcard not accessible — starting index at 1");
        return 1;
    }
    int max = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        int n;
        if (sscanf(e->d_name, "img_%d_", &n) == 1 && n > max) {
            max = n;
        }
    }
    closedir(d);
    return max + 1;
}

esp_err_t capture_service_init(cam_pipeline_handle_t pipeline,
                               uint32_t width, uint32_t height)
{
    if (!pipeline || width == 0 || height == 0) return ESP_ERR_INVALID_ARG;

    s_pipeline = pipeline;
    s_width = width;
    s_height = height;
    s_size = (size_t)width * height * 2;

    s_snapshot_buf = heap_caps_malloc(s_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_snapshot_buf) {
        ESP_LOGE(TAG, "Failed to allocate %zu-byte snapshot buffer in PSRAM", s_size);
        return ESP_ERR_NO_MEM;
    }

    s_index_mtx = xSemaphoreCreateMutex();
    if (!s_index_mtx) {
        heap_caps_free(s_snapshot_buf);
        s_snapshot_buf = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_next_index = scan_next_index();
    ESP_LOGI(TAG, "init ok — %" PRIu32 "x%" PRIu32 " (%zu bytes), starting index %d",
             s_width, s_height, s_size, s_next_index);
    return ESP_OK;
}

esp_err_t capture_service_snapshot(void)
{
    if (!s_snapshot_buf) return ESP_ERR_INVALID_STATE;

    const uint8_t *buf = cam_pipeline_lock_frame(s_pipeline);
    if (!buf) {
        ESP_LOGW(TAG, "snapshot: no frame available");
        s_snapshot_valid = false;
        return ESP_FAIL;
    }
    memcpy(s_snapshot_buf, buf, s_size);
    cam_pipeline_release_frame(s_pipeline);
    s_snapshot_valid = true;
    return ESP_OK;
}

esp_err_t capture_service_save(void)
{
    if (!s_snapshot_valid) return ESP_ERR_INVALID_STATE;

    int index;
    xSemaphoreTake(s_index_mtx, portMAX_DELAY);
    index = s_next_index;
    xSemaphoreGive(s_index_mtx);

    char path[64];
    snprintf(path, sizeof(path), "/sdcard/img_%04d_%lux%lu.rgb565",
             index, (unsigned long)s_width, (unsigned long)s_height);

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "fopen(%s) failed", path);
        return ESP_FAIL;
    }

    size_t written = fwrite(s_snapshot_buf, 1, s_size, f);
    int close_err = fclose(f);

    if (written != s_size || close_err != 0) {
        ESP_LOGE(TAG, "write failed for %s (wrote %zu/%zu, close=%d)",
                 path, written, s_size, close_err);
        return ESP_FAIL;
    }

    xSemaphoreTake(s_index_mtx, portMAX_DELAY);
    s_next_index = index + 1;
    xSemaphoreGive(s_index_mtx);

    ESP_LOGI(TAG, "saved %s (%zu bytes)", path, written);
    return ESP_OK;
}

const uint8_t *capture_service_snapshot_buffer(void)
{
    return s_snapshot_valid ? s_snapshot_buf : NULL;
}

uint32_t capture_service_width(void)  { return s_width; }
uint32_t capture_service_height(void) { return s_height; }
