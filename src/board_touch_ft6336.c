#include "board.h"
#include "board_config.h"

#if BOARD_TOUCH_DRIVER == TOUCH_FT6336

#include "board_touch_ft6336.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "touch_ft6336";

/* FT5x06 touch threshold registers (THGROUP..THDIFF, 0x80-0x85) */
#define FT5x06_ID_G_THGROUP             (0x80)
#define FT5x06_TH_REG_COUNT             6

/* FT5x06 scan rate / power mode registers */
#define FT5x06_ID_G_CTRL                (0x86)
#define FT5x06_ID_G_PERIODACTIVE        (0x88)
#define FT5x06_ID_G_PERIODMONITOR       (0x89)

/* Default RST/INT to GPIO_NUM_NC if board config doesn't define them.
 * Boards with an IO expander (e.g. S3 3.5") reset the touch controller
 * externally and don't need these pins. Boards without an IO expander
 * (e.g. P4 3.5") must define them for the hardware reset below. */
#ifndef BOARD_PIN_TOUCH_RST
#define BOARD_PIN_TOUCH_RST  GPIO_NUM_NC
#endif
#ifndef BOARD_PIN_TOUCH_INT
#define BOARD_PIN_TOUCH_INT  GPIO_NUM_NC
#endif

/**
 * Hardware reset for boards with a dedicated touch RST GPIO.
 * Boards with an IO expander skip this (RST pin will be GPIO_NUM_NC).
 *
 * Note: uses runtime check, not #if, because GPIO_NUM_* are enum values
 * that the C preprocessor cannot evaluate.
 */
static void ft6336_hw_reset(void)
{
    if (BOARD_PIN_TOUCH_RST == GPIO_NUM_NC) return;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOARD_PIN_TOUCH_RST,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Reset sequence: hold RST low for 10ms, then release */
    gpio_set_level(BOARD_PIN_TOUCH_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BOARD_PIN_TOUCH_RST, 1);

    /* FT6336 needs ~200ms after reset before I2C is ready */
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "FT6336 hardware reset complete (RST=GPIO%d)", BOARD_PIN_TOUCH_RST);
}

esp_lcd_touch_handle_t board_touch_ft6336_init(i2c_master_bus_handle_t bus,
                                                uint16_t x_max, uint16_t y_max)
{
    ft6336_hw_reset();

    esp_lcd_panel_io_handle_t touch_io_handle;
    esp_lcd_panel_io_i2c_config_t touch_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    touch_io_config.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(bus, &touch_io_config, &touch_io_handle));

    /* Save the controller's pre-init touch thresholds before the Espressif
     * driver overwrites them with its FT5x06 values; restored after init below. */
    uint8_t saved_th[FT5x06_TH_REG_COUNT];
    esp_err_t th_err = esp_lcd_panel_io_rx_param(touch_io_handle, FT5x06_ID_G_THGROUP,
                                                 saved_th, sizeof(saved_th));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = x_max,
        .y_max = y_max,
        .rst_gpio_num = GPIO_NUM_NC,   /* Already reset above; don't let driver reset again */
        .int_gpio_num = BOARD_PIN_TOUCH_INT,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    esp_lcd_touch_handle_t touch_handle = NULL;
    esp_err_t err = esp_lcd_touch_new_i2c_ft5x06(touch_io_handle, &tp_cfg, &touch_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FT6336 touch init failed (0x%x) — continuing without touch", err);
        return NULL;
    }

    /* Restore the saved touch thresholds.
     *
     * The driver's init writes FT5x06 thresholds (THGROUP=70, THDIFF=20, ...)
     * over the FT6336's own panel tuning (THGROUP=25, THDIFF=160 on the P4
     * 3.5"). With the driver values the P4 3.5" controller drops a moving
     * finger after ~30ms, so a swipe reaches LVGL as a short press at its
     * start point and selects the button under it instead of scrolling. */
    if (th_err != ESP_OK) {
        ESP_LOGW(TAG, "Threshold read failed (0x%x); keeping driver values", th_err);
    } else {
        for (int i = 0; i < FT5x06_TH_REG_COUNT && th_err == ESP_OK; i++) {
            th_err = esp_lcd_panel_io_tx_param(touch_handle->io, FT5x06_ID_G_THGROUP + i,
                                               &saved_th[i], 1);
        }
        if (th_err == ESP_OK) {
            ESP_LOGI(TAG, "Thresholds restored: THGROUP=%u THDIFF=%u",
                     saved_th[0], saved_th[FT5x06_TH_REG_COUNT - 1]);
        } else {
            ESP_LOGW(TAG, "Threshold restore failed (0x%x); some driver values remain",
                     th_err);
        }
    }

    /* Override Espressif driver defaults for better swipe detection.
     *
     * The driver sets PERIODACTIVE=12 (~60Hz) and PERIODMONITOR=40 (25Hz)
     * with auto-switch to monitor mode after 2 seconds idle. At 25Hz the
     * simplified monitor-mode algorithm misses fast swipes entirely.
     *
     * Fix: disable auto-switch to monitor mode and increase the active
     * scan rate to ~100Hz (PERIODACTIVE=6). */
    esp_lcd_panel_io_tx_param(touch_handle->io, FT5x06_ID_G_CTRL,
                              (uint8_t[]){0x00}, 1);
    esp_lcd_panel_io_tx_param(touch_handle->io, FT5x06_ID_G_PERIODACTIVE,
                              (uint8_t[]){6}, 1);
    esp_lcd_panel_io_tx_param(touch_handle->io, FT5x06_ID_G_PERIODMONITOR,
                              (uint8_t[]){6}, 1);
    ESP_LOGI(TAG, "Overrides applied: CTRL=0 (no auto-monitor), PERIODACTIVE=6 (~100Hz)");

    return touch_handle;
}

#endif /* BOARD_TOUCH_DRIVER == TOUCH_FT6336 */
