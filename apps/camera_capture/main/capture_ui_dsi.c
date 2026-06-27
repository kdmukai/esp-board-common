/**
 * capture_ui_dsi — LVGL bindings for the DSI (MIPI-DSI, image-widget) path.
 *
 * Entire TU is gated on BOARD_DISPLAY_DRIVER == DISPLAY_ST7701 so the file
 * becomes empty on non-DSI boards. main.c emits a #error when neither
 * capture_ui_dsi nor (future) capture_ui_spi handles the current board.
 */
#include "board.h"
#include "board_config.h"

#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701

#include "capture_ui.h"
#include "capture_service.h"

#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

static const char *TAG = "capture_ui_dsi";

typedef enum {
    STATE_LIVE,
    STATE_CAPTURING,
    STATE_REVIEWING,
} ui_state_t;

static ui_state_t s_state = STATE_LIVE;

static lv_obj_t *s_tap_btn;
static lv_obj_t *s_flash;
#if CAPTURE_REVIEW_ENABLED
static lv_obj_t *s_review_img;
static lv_obj_t *s_save_btn;
static lv_obj_t *s_discard_btn;
static lv_image_dsc_t s_review_dsc;
#else
static lv_obj_t *s_review_img  = NULL;
static lv_obj_t *s_save_btn    = NULL;
static lv_obj_t *s_discard_btn = NULL;
#endif

static void set_state(ui_state_t next);
static void flash_timer_cb(lv_timer_t *t);

static void tap_event_cb(lv_event_t *e)
{
    if (s_state != STATE_LIVE) return;
    ESP_LOGI(TAG, "tap → capture");
    set_state(STATE_CAPTURING);

    /* Snapshot is fast (~1–2 ms PSRAM memcpy); safe to run on LVGL thread. */
    esp_err_t err = capture_service_snapshot();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "snapshot failed (%s); returning to live",
                 esp_err_to_name(err));
        set_state(STATE_LIVE);
        return;
    }

    /* Hold flash visible for the min duration, then transition. */
    lv_timer_t *t = lv_timer_create(flash_timer_cb, CAPTURE_FLASH_DURATION_MS, NULL);
    lv_timer_set_repeat_count(t, 1);
}

static void flash_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (s_state != STATE_CAPTURING) return;

#if CAPTURE_REVIEW_ENABLED
    /* Set up review image descriptor pointing at the snapshot buffer. */
    const uint8_t *buf = capture_service_snapshot_buffer();
    if (!buf) {
        ESP_LOGW(TAG, "no snapshot buffer; skipping review");
        set_state(STATE_LIVE);
        return;
    }
    uint32_t w = capture_service_width();
    uint32_t h = capture_service_height();
    s_review_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    s_review_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    s_review_dsc.header.w = w;
    s_review_dsc.header.h = h;
    s_review_dsc.data_size = (size_t)w * h * 2;
    s_review_dsc.data = buf;
    lv_image_set_src(s_review_img, &s_review_dsc);

    set_state(STATE_REVIEWING);
#else
    esp_err_t err = capture_service_save();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    }
    set_state(STATE_LIVE);
#endif
}

#if CAPTURE_REVIEW_ENABLED
static void save_event_cb(lv_event_t *e)
{
    if (s_state != STATE_REVIEWING) return;
    esp_err_t err = capture_service_save();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "save failed: %s", esp_err_to_name(err));
    }
    set_state(STATE_LIVE);
}

static void discard_event_cb(lv_event_t *e)
{
    if (s_state != STATE_REVIEWING) return;
    ESP_LOGI(TAG, "discard");
    set_state(STATE_LIVE);
}
#endif

static void show_hide(lv_obj_t *obj, bool show)
{
    if (!obj) return;
    if (show) lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else      lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static void set_state(ui_state_t next)
{
    s_state = next;
    switch (next) {
    case STATE_LIVE:
        show_hide(s_tap_btn,     true);
        show_hide(s_flash,       false);
        show_hide(s_review_img,  false);
        show_hide(s_save_btn,    false);
        show_hide(s_discard_btn, false);
        break;
    case STATE_CAPTURING:
        show_hide(s_tap_btn,     false);
        show_hide(s_flash,       true);
        show_hide(s_review_img,  false);
        show_hide(s_save_btn,    false);
        show_hide(s_discard_btn, false);
        break;
    case STATE_REVIEWING:
        show_hide(s_tap_btn,     false);
        show_hide(s_flash,       false);
        show_hide(s_review_img,  true);
        show_hide(s_save_btn,    true);
        show_hide(s_discard_btn, true);
        break;
    }
}

esp_err_t capture_ui_init(cam_pipeline_handle_t pipeline)
{
    lv_obj_t *parent = (lv_obj_t *)cam_pipeline_get_overlay_parent(pipeline);
    if (!parent) {
        ESP_LOGE(TAG, "no overlay parent from pipeline");
        return ESP_ERR_INVALID_STATE;
    }

    if (!lvgl_port_lock(0)) {
        ESP_LOGE(TAG, "lvgl_port_lock failed");
        return ESP_FAIL;
    }

    /* Full-sized transparent tap target on top of the camera image.
     * Plain lv_obj (not a button) so we can strip styling cleanly. */
    s_tap_btn = lv_obj_create(parent);
    lv_obj_remove_style_all(s_tap_btn);
    lv_obj_set_size(s_tap_btn, lv_pct(100), lv_pct(100));
    lv_obj_center(s_tap_btn);
    lv_obj_add_flag(s_tap_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_tap_btn, tap_event_cb, LV_EVENT_CLICKED, NULL);

    /* White full-area flash overlay, initially hidden. */
    s_flash = lv_obj_create(parent);
    lv_obj_remove_style_all(s_flash);
    lv_obj_set_size(s_flash, lv_pct(100), lv_pct(100));
    lv_obj_center(s_flash);
    lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_flash, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_flash, LV_OBJ_FLAG_HIDDEN);

#if CAPTURE_REVIEW_ENABLED
    /* Frozen snapshot image — shown over the live feed during review. */
    s_review_img = lv_image_create(parent);
    lv_obj_set_size(s_review_img, lv_pct(100), lv_pct(100));
    lv_obj_center(s_review_img);
    lv_obj_add_flag(s_review_img, LV_OBJ_FLAG_HIDDEN);

    /* Save / Discard buttons along the bottom of the review area. */
    s_save_btn = lv_btn_create(parent);
    lv_obj_set_size(s_save_btn, lv_pct(45), 60);
    lv_obj_align(s_save_btn, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_add_event_cb(s_save_btn, save_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_save_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *save_lbl = lv_label_create(s_save_btn);
    lv_label_set_text(save_lbl, "Save");
    lv_obj_set_style_text_font(save_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(save_lbl);

    s_discard_btn = lv_btn_create(parent);
    lv_obj_set_size(s_discard_btn, lv_pct(45), 60);
    lv_obj_align(s_discard_btn, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_add_event_cb(s_discard_btn, discard_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_flag(s_discard_btn, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t *discard_lbl = lv_label_create(s_discard_btn);
    lv_label_set_text(discard_lbl, "Discard");
    lv_obj_set_style_text_font(discard_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(discard_lbl);
#endif

    lvgl_port_unlock();

    ESP_LOGI(TAG, "UI ready (review=%d, flash=%dms)",
             CAPTURE_REVIEW_ENABLED, CAPTURE_FLASH_DURATION_MS);
    return ESP_OK;
}

#endif /* BOARD_DISPLAY_DRIVER == DISPLAY_ST7701 */
