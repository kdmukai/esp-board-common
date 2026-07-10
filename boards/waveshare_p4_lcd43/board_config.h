/**
 * Board configuration: Waveshare ESP32-P4 WiFi6 Touch LCD 4.3
 *
 * Display:  ST7701 MIPI-DSI 2-lane (480x800, portrait native)
 * Touch:    GT911 I2C
 * Camera:   OV5647 MIPI-CSI 2-lane
 * SoC:      ESP32-P4 (dual-core RISC-V 400MHz, 32MB PSRAM, 32MB flash)
 *
 * Pin assignments from waveshareteam/ESP32-P4-WIFI6-Touch-LCD-4.3 BSP.
 */
#pragma once

#include "driver/gpio.h"

#define BOARD_NAME              "Waveshare ESP32-P4 WiFi6 Touch LCD 4.3"

/* ── Display (MIPI-DSI, ST7701) ── */
#define BOARD_DISPLAY_DRIVER    DISPLAY_ST7701
#define BOARD_LCD_H_RES         480
#define BOARD_LCD_V_RES         800
#define BOARD_PIN_LCD_RST       GPIO_NUM_27
#define BOARD_PIN_LCD_BL        GPIO_NUM_26

/* ── Display quirks ── */
#define BOARD_DISPLAY_QSPI              0
#define BOARD_DISPLAY_QUIRK_RASET_BUG   0
#define BOARD_DISPLAY_DIRECT_MODE       0
#define BOARD_DISPLAY_INVERT_COLOR      0

/* ── MIPI-DSI configuration ── */
#define BOARD_MIPI_DSI_LANE_NUM             2
#define BOARD_MIPI_DSI_LANE_BITRATE_MBPS    500
#define BOARD_MIPI_DSI_PHY_LDO_CHAN         3
#define BOARD_MIPI_DSI_PHY_LDO_MV           2500
#define BOARD_MIPI_DPI_CLK_MHZ              30
#define BOARD_MIPI_DPI_NUM_FBS              3
/* DPI video timing (from Waveshare BSP) */
#define BOARD_MIPI_DPI_HBP                  42
#define BOARD_MIPI_DPI_HSYNC                12
#define BOARD_MIPI_DPI_HFP                  42
#define BOARD_MIPI_DPI_VBP                  2
#define BOARD_MIPI_DPI_VSYNC                8
#define BOARD_MIPI_DPI_VFP                  60

/* ── Backlight ── */
#define BOARD_BACKLIGHT_INVERTED    1   /* Higher duty = dimmer on this board */

/* ── IO Expander ── */
#define BOARD_HAS_IO_EXPANDER   0

/* ── Touch (GT911) ── */
#define BOARD_TOUCH_DRIVER      TOUCH_GT911
#define BOARD_PIN_TOUCH_RST     GPIO_NUM_23
#define BOARD_PIN_TOUCH_INT     GPIO_NUM_NC

/* ── I2C ── */
#define BOARD_PIN_I2C_SDA       GPIO_NUM_7
#define BOARD_PIN_I2C_SCL       GPIO_NUM_8
#define BOARD_I2C_PORT          0

/* ── PMIC ── */
#define BOARD_HAS_PMIC          0

/* ── LVGL port tuning ── */
/* Flattened to 1 (== MicroPython VM task) so LVGL-lock access is FIFO-fair and the
 * prio-1 VM/consumer doesn't starve at overlay-create / present(). A/B-confirmed not
 * to affect preview fps (prio-5 firmware measured the same). See board_common note. */
#define BOARD_LVGL_TASK_PRIORITY    1
#define BOARD_LVGL_TASK_STACK       (1024 * 16)
#define BOARD_LVGL_TASK_AFFINITY    -1  /* No core affinity */
#define BOARD_LVGL_MAX_SLEEP_MS     500
#define BOARD_LVGL_TIMER_PERIOD_MS  5

/* ── ST7701 deferred-flush task ──
 * The landscape flush does a ~30ms CPU rotation + vsync wait + panel blit. It
 * runs on this task so that work happens OFF the LVGL lock (see board_init.c).
 * Prio 1 == the flatten baseline: it must NOT preempt the lvgl render (which
 * holds the lock), and it does not take the LVGL mutex itself, so equal prio is
 * safe for mutex fairness. Core/priority are tuning knobs — if the flush task
 * gets starved (wait_for_flushing stalls under the lock), try raising the
 * priority or pinning to core 1. */
#define BOARD_ST7701_FLUSH_TASK_PRIORITY   1
#define BOARD_ST7701_FLUSH_TASK_STACK      4096
#define BOARD_ST7701_FLUSH_TASK_AFFINITY   -1  /* tskNO_AFFINITY */

/* ── Camera (MIPI-CSI, OV5647) ── */
#ifndef BOARD_HAS_CAMERA
#define BOARD_HAS_CAMERA            1
#endif
#define BOARD_CAMERA_INTERFACE      CAMERA_CSI
#define BOARD_PIN_CAM_SCCB_SDA      GPIO_NUM_7
#define BOARD_PIN_CAM_SCCB_SCL      GPIO_NUM_8
#define BOARD_CAM_SCCB_I2C_PORT     0   /* Shares main I2C bus */

/* ── SD Card (4-bit SDMMC) ── */
#ifndef BOARD_HAS_SDCARD
#define BOARD_HAS_SDCARD            1
#endif
#define BOARD_SD_WIDTH              4
#define BOARD_PIN_SD_CLK            GPIO_NUM_43
#define BOARD_PIN_SD_CMD            GPIO_NUM_44
#define BOARD_PIN_SD_D0             GPIO_NUM_39
#define BOARD_PIN_SD_D1             GPIO_NUM_40
#define BOARD_PIN_SD_D2             GPIO_NUM_41
#define BOARD_PIN_SD_D3             GPIO_NUM_42

/* ── Audio (ES8311 + ES7210) ── */
#ifndef BOARD_HAS_AUDIO
#define BOARD_HAS_AUDIO             1
#endif
#define BOARD_PIN_I2S_MCK           GPIO_NUM_13
#define BOARD_PIN_I2S_BCK           GPIO_NUM_12
#define BOARD_PIN_I2S_LRCK          GPIO_NUM_10
#define BOARD_PIN_I2S_DOUT          GPIO_NUM_9
#define BOARD_PIN_I2S_DIN           GPIO_NUM_11
#define BOARD_PIN_PA                GPIO_NUM_53

/* ── RTC / IMU ── */
#ifndef BOARD_HAS_RTC
#define BOARD_HAS_RTC               0
#endif
#ifndef BOARD_HAS_IMU
#define BOARD_HAS_IMU               0
#endif

/* ── Radio co-processor (ESP32-C6 "WIFI6", SDIO slave) ── */
/* The C6 follows the esp-hosted P4 reference wiring: its reset line idles
 * high (chip runs its factory hosted-slave firmware) and GPIO54 is the
 * esp-hosted default SDIO reset-slave pin (low = held in reset). SeedSigner
 * never uses the radio, so board_init() drives this low at boot and leaves
 * it low — the C6 executes no code for the entire session (air gap). */
#define BOARD_RADIO_COPROC_RESET_PIN GPIO_NUM_54
