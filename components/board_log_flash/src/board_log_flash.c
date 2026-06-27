/**
 * Flash-based boot log buffer with crash resilience.
 *
 * Three layers of log capture:
 *
 * 1. FLASH LOG (live + survives power cycle):
 *    vprintf hook → internal-SRAM ring buffer → flush task → flash partition.
 *    Call board_log_flash_dump() at any time to read back the current session.
 *
 * 2. NOINIT RING BUFFER (survives soft reset / panic / watchdog):
 *    Same ring buffer is in a .noinit section. After a soft reset,
 *    board_log_flash_init() checks esp_reset_reason() and dumps it.
 *    Lightweight — no flash writes needed for crash recovery.
 *
 * 3. CORE DUMP (deep forensics):
 *    Ring buffer tagged with COREDUMP_DRAM_ATTR so core dump captures
 *    both the crash backtrace AND the log text. Extract via
 *    idf.py coredump-debug or esp_core_dump_get_summary().
 *
 * CRITICAL: The vprintf hook NEVER touches flash directly. Flash writes
 * require SPI cache disabled, which makes PSRAM inaccessible. The flush
 * task runs on an internal-RAM stack allocated explicitly.
 */

#include "board_log_flash.h"

#ifdef CONFIG_BOARD_LOG_TO_FLASH

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "log_flash";

#define LOG_HEADER_SIZE     4       /* uint32_t total_written */
#define RING_BUF_SIZE       8192    /* ring buffer size */
#define FLUSH_INTERVAL_MS   500     /* flush task period */
#define FLUSH_BUF_SIZE      2048    /* per-flush transfer chunk */

#define NOINIT_MAGIC        0x4C4F4721  /* "LOG!" — validates noinit data */

/* ── Ring buffer ──
 * In internal SRAM (.noinit section) so it survives soft resets.
 * Tagged with COREDUMP_DRAM_ATTR so core dump captures it.
 * Both attributes: noinit keeps data across reset, coredump includes it. */
typedef struct {
    uint32_t magic;                     /* NOINIT_MAGIC if valid */
    size_t head;                        /* write position */
    size_t tail;                        /* read position (for flash flush) */
    uint8_t buf[RING_BUF_SIZE];
} log_ring_t;

/* __NOINIT_ATTR: survives soft resets (panic/watchdog).
 * Core dump captures all internal DRAM when CONFIG_ESP_COREDUMP_CAPTURE_DRAM=y,
 * so this buffer is included in crash dumps regardless of section. */
static __NOINIT_ATTR log_ring_t s_ring;

static const esp_partition_t *s_part = NULL;
static size_t s_flash_written = 0;
static size_t s_flash_capacity = 0;
static vprintf_like_t s_orig_vprintf = NULL;
static bool s_initialized = false;
static volatile bool s_serial_paused = false;  /* suppresses live serial during dump */

/* Previous boot's log from PSRAM (read from flash on init) */
static uint8_t *s_prev_log = NULL;
static size_t s_prev_log_size = 0;

/* ── Ring buffer helpers (lock-free SPSC) ── */

static inline size_t ring_used(void)
{
    size_t h = s_ring.head;
    size_t t = s_ring.tail;
    return (h >= t) ? (h - t) : (RING_BUF_SIZE - t + h);
}

static inline size_t ring_free(void)
{
    return RING_BUF_SIZE - 1 - ring_used();
}

static void ring_write(const char *data, size_t len)
{
    size_t h = s_ring.head;
    for (size_t i = 0; i < len; i++) {
        s_ring.buf[h] = (uint8_t)data[i];
        h = (h + 1) % RING_BUF_SIZE;
    }
    s_ring.head = h;
}

static size_t ring_read_for_flush(uint8_t *dst, size_t max_len)
{
    size_t t = s_ring.tail;
    size_t h = s_ring.head;
    size_t avail = (h >= t) ? (h - t) : (RING_BUF_SIZE - t + h);
    size_t n = (avail < max_len) ? avail : max_len;

    for (size_t i = 0; i < n; i++) {
        dst[i] = s_ring.buf[t];
        t = (t + 1) % RING_BUF_SIZE;
    }
    s_ring.tail = t;
    return n;
}

/* ── Flash I/O (only called from flush task with internal-RAM stack) ── */

static esp_err_t write_to_flash(const uint8_t *data, size_t len)
{
    size_t space = s_flash_capacity - s_flash_written;
    size_t to_write = (len < space) ? len : space;
    if (to_write == 0) return ESP_OK;

    esp_err_t err = esp_partition_write(s_part,
        LOG_HEADER_SIZE + s_flash_written, data, to_write);
    if (err != ESP_OK) return err;

    s_flash_written += to_write;

    uint32_t total = (uint32_t)s_flash_written;
    err = esp_partition_write(s_part, 0, &total, sizeof(total));
    return err;
}

/* ── Flush task (internal-RAM stack, safe for flash ops) ── */

static void flush_task(void *param)
{
    (void)param;
    uint8_t *buf = heap_caps_malloc(FLUSH_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate flush buffer");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(FLUSH_INTERVAL_MS));

        size_t n = ring_read_for_flush(buf, FLUSH_BUF_SIZE);
        if (n > 0) {
            write_to_flash(buf, n);
        }
    }
}

/* ── Log hook (runs in ANY task context — never touches flash) ── */

static int log_to_flash_vprintf(const char *fmt, va_list args)
{
    /* Always write to ring buffer (flash capture), even when serial is paused */
    if (s_initialized) {
        char line[256];
        va_list args_copy;
        va_copy(args_copy, args);
        int len = vsnprintf(line, sizeof(line), fmt, args_copy);
        va_end(args_copy);

        if (len > 0) {
            if (len >= (int)sizeof(line)) len = sizeof(line) - 1;
            size_t avail = ring_free();
            size_t to_write = ((size_t)len < avail) ? (size_t)len : avail;
            if (to_write > 0) {
                ring_write(line, to_write);
            }
        }
    }

    /* Serial output — suppressed during dump to prevent interleaving */
    if (s_serial_paused) return 0;
    return s_orig_vprintf(fmt, args);
}

/* ── Noinit dump helper ── */

static void dump_noinit_log(void)
{
    esp_reset_reason_t reason = esp_reset_reason();
    const char *reason_str;
    switch (reason) {
        case ESP_RST_PANIC:   reason_str = "panic"; break;
        case ESP_RST_INT_WDT: reason_str = "interrupt watchdog"; break;
        case ESP_RST_TASK_WDT: reason_str = "task watchdog"; break;
        case ESP_RST_WDT:     reason_str = "other watchdog"; break;
        case ESP_RST_BROWNOUT: reason_str = "brownout"; break;
        default:              reason_str = NULL; break;
    }

    if (s_ring.magic != NOINIT_MAGIC) {
        ESP_LOGI(TAG, "No noinit log (first boot or power cycle)");
        return;
    }

    if (!reason_str) {
        ESP_LOGI(TAG, "Clean reset (reason=%d) — noinit log discarded", (int)reason);
        return;
    }

    /* Valid noinit data from a crash — dump it */
    size_t avail = ring_used();
    if (avail == 0 || avail > RING_BUF_SIZE) {
        ESP_LOGI(TAG, "Noinit ring buffer empty or corrupt");
        return;
    }

    ESP_LOGW(TAG, "=== Crash log from previous boot (reset: %s, %zu bytes) ===",
             reason_str, avail);

    uint8_t tmp[256];
    size_t pos = s_ring.tail;
    size_t remaining = avail;
    while (remaining > 0) {
        size_t n = (remaining < sizeof(tmp) - 1) ? remaining : sizeof(tmp) - 1;
        for (size_t i = 0; i < n; i++) {
            tmp[i] = s_ring.buf[pos];
            pos = (pos + 1) % RING_BUF_SIZE;
        }
        tmp[n] = '\0';
        fputs((char *)tmp, stdout);
        remaining -= n;
    }

    ESP_LOGW(TAG, "=== End crash log ===");
}

/* ── Public API ── */

esp_err_t board_log_flash_init(void)
{
    /* Check for noinit crash log from previous boot BEFORE re-initializing */
    dump_noinit_log();

    /* Find the log partition */
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      0x42, "log_store");
    if (!s_part) {
        ESP_LOGW(TAG, "No 'log_store' partition found — flash logging disabled");
        /* Still set up the noinit ring buffer for crash capture */
        s_ring.magic = NOINIT_MAGIC;
        s_ring.head = 0;
        s_ring.tail = 0;
        s_orig_vprintf = esp_log_set_vprintf(log_to_flash_vprintf);
        s_initialized = true;
        return ESP_ERR_NOT_FOUND;
    }

    s_flash_capacity = s_part->size - LOG_HEADER_SIZE;
    ESP_LOGI(TAG, "Flash log partition: %zuKB at 0x%"PRIx32,
             s_part->size / 1024, s_part->address);

    /* Save previous boot's flash log to PSRAM */
    uint32_t prev_total = 0;
    esp_partition_read(s_part, 0, &prev_total, sizeof(prev_total));

    if (prev_total > 0 && prev_total <= s_flash_capacity && prev_total != 0xFFFFFFFF) {
        s_prev_log = heap_caps_malloc(prev_total, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_prev_log) {
            esp_partition_read(s_part, LOG_HEADER_SIZE, s_prev_log, prev_total);
            s_prev_log_size = prev_total;
            ESP_LOGI(TAG, "Previous boot flash log: %"PRIu32" bytes available", prev_total);
        }
    } else {
        ESP_LOGI(TAG, "No previous boot flash log found");
    }

    /* Erase partition for this boot's log */
    esp_err_t err = esp_partition_erase_range(s_part, 0, s_part->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase log partition: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t zero = 0;
    esp_partition_write(s_part, 0, &zero, sizeof(zero));

    /* Initialize ring buffer for this boot */
    s_ring.magic = NOINIT_MAGIC;
    s_ring.head = 0;
    s_ring.tail = 0;
    s_flash_written = 0;

    /* Hook ESP_LOG output */
    s_orig_vprintf = esp_log_set_vprintf(log_to_flash_vprintf);

    s_initialized = true;
    ESP_LOGI(TAG, "Flash log active (%zuKB capacity)", s_flash_capacity / 1024);

    /* Start flush task with internal-RAM stack */
    StaticTask_t *task_buf = heap_caps_malloc(sizeof(StaticTask_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    StackType_t *stack_buf = heap_caps_malloc(4096 * sizeof(StackType_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (task_buf && stack_buf) {
        xTaskCreateStatic(flush_task, "log_flush", 4096, NULL, 1,
                          stack_buf, task_buf);
    } else {
        ESP_LOGE(TAG, "Failed to allocate internal-RAM stack for flush task");
    }

    return ESP_OK;
}

void board_log_flash_dump(void)
{
    if (!s_initialized || !s_part) {
        printf("[log_flash] Not initialized or no flash partition\n");
        return;
    }

    /* Pause live serial output to prevent interleaving.
     * Ring buffer still captures everything — no log data is lost. */
    s_serial_paused = true;

    /* Wait for flush task to drain the ring buffer to flash */
    vTaskDelay(pdMS_TO_TICKS(FLUSH_INTERVAL_MS + 100));

    printf("\n=== Current boot log (%zu bytes) ===\n", s_flash_written);
    char chunk[256];
    size_t offset = LOG_HEADER_SIZE;
    size_t remaining = s_flash_written;
    while (remaining > 0) {
        size_t n = remaining < sizeof(chunk) - 1 ? remaining : sizeof(chunk) - 1;
        esp_partition_read(s_part, offset, chunk, n);
        chunk[n] = '\0';
        fputs(chunk, stdout);
        offset += n;
        remaining -= n;
    }
    printf("=== End current boot log ===\n");

    /* Resume live serial output */
    s_serial_paused = false;
}

void board_log_flash_flush(void)
{
    if (s_initialized) {
        vTaskDelay(pdMS_TO_TICKS(FLUSH_INTERVAL_MS + 100));
    }
}

void board_log_flash_dump_previous(void)
{
    if (!s_prev_log || s_prev_log_size == 0) {
        printf("[log_flash] No previous boot flash log available\n");
        return;
    }

    printf("\n=== Previous boot flash log (%zu bytes) ===\n", s_prev_log_size);
    size_t offset = 0;
    while (offset < s_prev_log_size) {
        size_t n = s_prev_log_size - offset;
        if (n > 255) n = 255;
        char tmp[256];
        memcpy(tmp, s_prev_log + offset, n);
        tmp[n] = '\0';
        fputs(tmp, stdout);
        offset += n;
    }
    printf("\n=== End previous boot flash log ===\n");

    heap_caps_free(s_prev_log);
    s_prev_log = NULL;
    s_prev_log_size = 0;
}

#else /* !CONFIG_BOARD_LOG_TO_FLASH */

esp_err_t board_log_flash_init(void) { return ESP_OK; }
void board_log_flash_dump_previous(void) {}
void board_log_flash_dump(void) {}
void board_log_flash_flush(void) {}

#endif
