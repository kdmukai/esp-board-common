/**
 * Board-level camera pipeline configuration glue.
 *
 * Reads board_config.h defines and populates a cam_pipeline_config_t
 * with the correct camera driver, display driver, and hardware config.
 */
#include "board.h"
#include "board_config.h"

#if BOARD_HAS_CAMERA

#include "board_pipeline.h"
#include "board_pipeline_display_lvgl.h"

#ifndef BOARD_CAMERA_ROTATION
#define BOARD_CAMERA_ROTATION 0
#endif

/* Partition mode: keep LVGL running during the camera preview so live chrome
 * renders in the gutters beside the direct-blit square (SPI/dummy-draw boards
 * only). Opt-in per board — the S3 dummy-draw boards stay on the proven
 * LVGL-stopped path until validated. */
#ifndef BOARD_CAMERA_PARTITION_MODE
#define BOARD_CAMERA_PARTITION_MODE 0
#endif

#if BOARD_CAMERA_INTERFACE == CAMERA_DVP
#include "board_pipeline_camera_dvp.h"
#include "driver/ledc.h"
static board_pipeline_dvp_config_t s_dvp_config;
#elif BOARD_CAMERA_INTERFACE == CAMERA_CSI
#include "board_pipeline_camera_csi.h"
static board_pipeline_csi_config_t s_csi_config;
#endif

static board_pipeline_lvgl_display_config_t s_lvgl_display_config;

cam_pipeline_config_t board_pipeline_default_config(void *display_parent,
                                                    void *i2c_bus)
{
    /* SPI panels: dummy-draw with byte-swap to fix tearing.
     * MIPI-DSI (DPI) panels: image widget path for LVGL overlay support
     * (QR text, FPS stats).  DPI panels don't tear, so dummy-draw isn't
     * needed, and the image widget path gives proper background rendering. */
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    s_lvgl_display_config.use_dummy_draw = false;
    s_lvgl_display_config.byte_swap = false;
    s_lvgl_display_config.keep_lvgl_running = false;
#else
    s_lvgl_display_config.use_dummy_draw = true;
    s_lvgl_display_config.byte_swap = true;
    s_lvgl_display_config.keep_lvgl_running = BOARD_CAMERA_PARTITION_MODE;
#endif

    /* Camera pre-rotation into the single PPA SRM pass (crop+scale+rotate).
     * DSI landscape (ST7701): the display flush rotates the whole LVGL canvas
     *   90° CCW to the portrait panel, so pre-rotate the camera 90° CW to match.
     * SPI/MADCTL landscape boards: the rotation is a fixed camera-to-panel
     *   MOUNT offset, supplied per board as BOARD_CAMERA_ROTATION (PPA angle is
     *   counter-clockwise; e.g. 270 == a 90° CW correction). */
#if BOARD_LANDSCAPE && BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    int cam_rotation = (BOARD_CAMERA_ROTATION + 270) % 360;
#else
    int cam_rotation = BOARD_CAMERA_ROTATION;
#endif

    cam_pipeline_config_t config = {
        .display_width  = BOARD_DISP_H_RES,
        .display_height = BOARD_DISP_V_RES,
        .rotation       = cam_rotation,
        .display_driver = &board_pipeline_lvgl_display_driver,
        .display_config = &s_lvgl_display_config,
        .display_parent = display_parent,
    };

#if BOARD_CAMERA_INTERFACE == CAMERA_DVP
    (void)i2c_bus; /* DVP uses SCCB I2C port directly */
    s_dvp_config = (board_pipeline_dvp_config_t){
        .pin_d0        = BOARD_PIN_CAM_Y2,
        .pin_d1        = BOARD_PIN_CAM_Y3,
        .pin_d2        = BOARD_PIN_CAM_Y4,
        .pin_d3        = BOARD_PIN_CAM_Y5,
        .pin_d4        = BOARD_PIN_CAM_Y6,
        .pin_d5        = BOARD_PIN_CAM_Y7,
        .pin_d6        = BOARD_PIN_CAM_Y8,
        .pin_d7        = BOARD_PIN_CAM_Y9,
        .pin_xclk      = BOARD_PIN_CAM_XCLK,
        .pin_pclk      = BOARD_PIN_CAM_PCLK,
        .pin_vsync     = BOARD_PIN_CAM_VSYNC,
        .pin_href      = BOARD_PIN_CAM_HREF,
        .pin_pwdn      = BOARD_PIN_CAM_PWDN,
        .pin_reset     = BOARD_PIN_CAM_RESET,
        .xclk_freq_hz  = BOARD_CAM_XCLK_FREQ,
        .ledc_timer    = BOARD_CAM_LEDC_TIMER,
        .ledc_channel  = BOARD_CAM_LEDC_CHANNEL,
        .sccb_i2c_port = BOARD_I2C_PORT,
        .frame_width   = 640, /* VGA — pipeline crops to display size */
        .frame_height  = 480,
    };
    config.camera_driver = &board_pipeline_dvp_driver;
    config.camera_config = &s_dvp_config;

#elif BOARD_CAMERA_INTERFACE == CAMERA_CSI
    s_csi_config = (board_pipeline_csi_config_t){
        .i2c_bus = (i2c_master_bus_handle_t)i2c_bus,
        .ae_target = CONFIG_BOARD_CSI_AE_TARGET, /* 0 = ISP default */
    };
    config.camera_driver = &board_pipeline_csi_driver;
    config.camera_config = &s_csi_config;
#endif

    return config;
}

#endif /* BOARD_HAS_CAMERA */
