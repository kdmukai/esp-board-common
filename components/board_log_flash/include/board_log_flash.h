/**
 * Flash-based boot log buffer.
 *
 * Captures all ESP_LOGx output to a flash partition, surviving reboots
 * and hard resets. Each log line is flushed to flash immediately.
 *
 * Usage:
 *   1. Add a "log_store" partition to your partition table (data, 0x42, 64KB+)
 *   2. Call board_log_flash_init() at the very start of app_main()
 *   3. After USB delay, call board_log_flash_dump_previous() for prior boot's log
 *   4. All subsequent ESP_LOGx output is tee'd to both serial and flash
 *   5. Call board_log_flash_dump() at any time to dump current session's log
 *   6. Enable with CONFIG_BOARD_LOG_TO_FLASH=y in sdkconfig
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the flash log buffer.
 *
 * - Saves the previous boot's log (call board_log_flash_dump_previous() later)
 * - Erases the partition for the new boot's log
 * - Hooks esp_log to tee output to flash
 *
 * Call this as early as possible in app_main().
 * No-op if CONFIG_BOARD_LOG_TO_FLASH is not enabled.
 */
esp_err_t board_log_flash_init(void);

/**
 * Dump the previous boot's log to serial.
 * Call after USB serial has had time to reconnect (e.g. after a boot delay).
 * Must be called after board_log_flash_init().
 */
void board_log_flash_dump_previous(void);

/**
 * Dump the current boot's log buffer to serial.
 * Flushes RAM to flash first, then reads back and prints.
 * Can be called at any time.
 */
void board_log_flash_dump(void);

/**
 * Flush any buffered data to flash.
 * Called automatically on crash via esp_register_shutdown_handler,
 * but can also be called manually.
 */
void board_log_flash_flush(void);

#ifdef __cplusplus
}
#endif
