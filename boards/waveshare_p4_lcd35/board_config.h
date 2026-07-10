/**
 * Board configuration: Waveshare ESP32-P4 WiFi6 Touch LCD 3.5
 *
 * Display:  ST7796 SPI (320x480)
 * Touch:    FT6336 I2C (FT5x06 family)
 * PMIC:     AXP2101
 * Camera:   OV5647 MIPI-CSI
 * SoC:      ESP32-P4 (dual-core RISC-V 400MHz, 32MB PSRAM)
 *
 * Pin assignments from Waveshare BSP (esp32_p4_wifi6_touch_lcd_35.h).
 */
#pragma once

#include "driver/gpio.h"

#define BOARD_NAME              "Waveshare ESP32-P4 WiFi6 Touch LCD 3.5"

/* ── Display ── */
#define BOARD_DISPLAY_DRIVER    DISPLAY_ST7796
#define BOARD_LCD_H_RES         320
#define BOARD_LCD_V_RES         480
#define BOARD_PIN_LCD_SCLK      GPIO_NUM_21
#define BOARD_PIN_LCD_MOSI      GPIO_NUM_20
#define BOARD_PIN_LCD_CS        GPIO_NUM_23
#define BOARD_PIN_LCD_DC        GPIO_NUM_26
#define BOARD_PIN_LCD_RST       GPIO_NUM_27
#define BOARD_PIN_LCD_BL        GPIO_NUM_28
#define BOARD_SPI_HOST          SPI2_HOST
#define BOARD_LCD_PIXEL_CLOCK   (80 * 1000 * 1000)

/* ── Display quirks ── */
#define BOARD_DISPLAY_QSPI              0
#define BOARD_DISPLAY_QUIRK_RASET_BUG   0
#define BOARD_DISPLAY_DIRECT_MODE       0
#define BOARD_DISPLAY_INVERT_COLOR      0
#define BOARD_DISPLAY_MIRROR_X          1   /* Panel is mirrored horizontally by default */
/* Landscape orientation: the generic default (swap_xy + mirror(1,1) in
 * board_init.c) is the canonical one — it matches the FT6336 landscape touch
 * transform. There is no "right way up" in hardware; if it reads upside down
 * on a stand, flip the device. (A 180° display flip via the
 * BOARD_LANDSCAPE_MIRROR_X/Y overrides was tried 2026-07-10 and made touch
 * disagree with the panel — don't flip one without the other.) */

/* ── IO Expander ── */
#define BOARD_HAS_IO_EXPANDER   0

/* ── Touch ── */
#define BOARD_TOUCH_DRIVER      TOUCH_FT6336
#define BOARD_PIN_TOUCH_RST     GPIO_NUM_29
#define BOARD_PIN_TOUCH_INT     GPIO_NUM_50

/* ── I2C ── */
#define BOARD_PIN_I2C_SDA       GPIO_NUM_7
#define BOARD_PIN_I2C_SCL       GPIO_NUM_8
#define BOARD_I2C_PORT          0

/* ── PMIC ── */
#define BOARD_HAS_PMIC          1
#define BOARD_PMIC_DRIVER       PMIC_AXP2101

/* ── LVGL port tuning ── */
/* Flattened to 1 (== MicroPython VM task) so LVGL-lock access is FIFO-fair and the
 * prio-1 VM/consumer doesn't starve at overlay-create / present(). A/B-confirmed not
 * to affect preview fps on the P4 LCD 4.3 (prio-5 firmware measured the same);
 * ported here with the same rationale. See board_common note. */
#define BOARD_LVGL_TASK_PRIORITY    1
#define BOARD_LVGL_TASK_STACK       (1024 * 16)
#define BOARD_LVGL_TASK_AFFINITY    -1  /* No core affinity */
#define BOARD_LVGL_MAX_SLEEP_MS     500
#define BOARD_LVGL_TIMER_PERIOD_MS  5

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

/* ── Audio (ES8311) ── */
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
/* Verified from the board schematic (ESP32-P4-WIFI6-Touch-LCD-3.5-schematic.pdf):
 * P4 GPIO54 → R54 (0R) → C6_CHIP_PU — same esp-hosted P4 reference wiring as
 * the LCD 4.3 board. The C6's reset line idles high (chip runs its factory
 * hosted-slave firmware); driving it low holds the C6 in reset. SeedSigner
 * never uses the radio, so board_init() drives this low at boot and leaves
 * it low — the C6 executes no code for the entire session (air gap). */
#define BOARD_RADIO_COPROC_RESET_PIN GPIO_NUM_54
