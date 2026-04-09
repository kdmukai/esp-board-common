# LVGL Widget Feasibility by Display Path

Interactive LVGL widgets (sliders, progress bars, gauges) are needed for transient UI like camera focus adjustment. Each display path has different constraints on whether native LVGL widgets can render and accept touch input.

## Summary

| Path | Native LVGL widgets? | Approach |
|------|---------------------|----------|
| DSI portrait (P4 LCD 4.3) | **Yes** | Widgets work natively, no tricks needed |
| DSI landscape (P4 LCD 4.3) | **Yes** | LVGL runs in landscape coordinates; flush callback rotates to portrait panel |
| SPI portrait (P4 LCD 3.5) | **Possible** | Resume LVGL temporarily, accept tearing/frame drops |
| SPI landscape (P4 LCD 3.5) | **Possible** | Same as SPI portrait — MADCTL handles rotation |

## SPI Dummy-Draw (P4 LCD 3.5)

### Why widgets don't work in dummy-draw mode

Dummy-draw calls `lvgl_port_stop()` which halts LVGL's entire timer/task infrastructure. No widget rendering, no invalidation, no timers. It's all-or-nothing. This exists because SPI displays tear when LVGL's flush callback and camera frame pushes contend for the SPI bus.

### Approach: temporarily resume LVGL

Call `lvgl_port_resume()` to re-enable LVGL when a widget (e.g., focus slider) is needed. The camera feed switches to the `push_frame_image_widget` path: camera task copies to `cam_buf`, calls `lv_obj_invalidate()`, and LVGL renders the image widget + slider together as one frame.

**What to expect:**

- **Frame drops** — `push_frame_image_widget` uses a non-blocking try-lock (`lvgl_port_lock(1)`). When LVGL is busy rendering, camera frames are silently dropped. Effective fps drops below the normal ~10fps.
- **Some tearing** — LVGL uses half-screen draw buffers (320 x 240) on this board, so a full-screen dirty region takes 2 flush operations. Between flushes, LVGL's render pipeline runs, creating a window for the panel scan to catch the boundary. The camera image is the main victim; the slider would only tear if it straddles the chunk boundary.
- **Stutter** — Camera task and LVGL task ping-pong on the port lock, adding latency jitter.

**Why this is acceptable:** Focus adjustment is a transient interaction lasting a few seconds. The user is actively watching the camera feed change — some stutter and occasional tear lines are tolerable. Once the slider is dismissed, re-enter dummy-draw mode for clean 10fps.

**Alternative considered:** Rendering a slider-like widget manually into gap buffers (the way `overlay_text.c` draws text) would keep dummy-draw active but is significant effort for a simple rectangle + handle.

### LVGL display config for reference

From `board_init.c`, the ST7796 SPI board uses:
- Half-screen draw buffer (`lvgl_hres * (lvgl_vres / 2)`)
- Double-buffered, PSRAM, `swap_bytes`
- Partial updates (no `direct_mode`)
- No custom flush callback

## DSI Landscape (P4 LCD 4.3)

### How it works (commit e6e58ce)

LVGL runs in landscape coordinates (800×480). A custom flush callback
CPU-rotates the entire frame 90° CCW into a PSRAM buffer, then submits
to the portrait panel synced to vsync. All LVGL widgets — labels,
sliders, images — render normally in landscape coordinates. No manual
rotation or portrait-coordinate tricks needed.

The LVGL display is created directly via `lv_display_create()` +
`lv_display_set_buffers()`, bypassing `esp_lvgl_port`'s DSI path.

### Why esp_lvgl_port is bypassed for landscape

`lvgl_port_add_disp_dsi()` only supports `direct_mode` +
`avoid_tearing=true`, which uses the DPI panel's portrait framebuffers.
The library's internal flush for DSI + direct_mode unconditionally calls
`xSemaphoreTake(trans_sem)` (line 717), but `trans_sem` is only created
with `avoid_tearing=true` (line 345). Any other combination crashes.
See `esp_lvgl_port` source `src/lvgl9/esp_lvgl_port_disp.c`.

Portrait mode still uses `lvgl_port_add_disp_dsi()` with
`avoid_tearing=true` unchanged.

### Camera rotation compensation

The flush rotates the entire LVGL canvas 90° CCW, including the camera
image widget. The camera PPA is pre-rotated 90° CW to compensate:
`(BOARD_CAMERA_ROTATION + 270) % 360`. Net result: camera orientation
matches portrait mode.

## DSI Portrait (P4 LCD 4.3)

No constraints. LVGL is running, coordinates match the display, touch events map correctly. Native widgets just work. This is the simplest path.

## What dummy-draw mode cannot support

Regardless of approach, while dummy-draw is active:
- No LVGL widgets of any kind (rendering halted)
- No LVGL timers (use FreeRTOS timers instead)
- No LVGL animations
- Text rendering uses direct LVGL font API (`lv_font_get_glyph_dsc()` + `font->get_glyph_bitmap()`) into raw RGB565 buffers, bypassing the LVGL draw pipeline entirely

## Proposed: SPI mode-switch support in the pipeline layer

> **Status: speculative.** This section captures how runtime dummy-draw ↔ LVGL switching could work if we decide to go this direction. Not committed to implementing it — may not be needed depending on how the app UX evolves.

### What already exists

- `ctx->dummy_draw` flag gates push path selection (`push_frame_dummy_draw` vs `push_frame_image_widget`)
- `lvgl_display_deinit()` already calls `lvgl_port_resume()` when tearing down dummy-draw mode
- `lvgl_port_stop()` / `lvgl_port_resume()` are the global LVGL lifecycle controls

### What's missing for runtime switching

1. **`cam_buf` not allocated in dummy-draw mode** — `push_frame_image_widget` needs it. Would need upfront allocation or lazy alloc on first switch.
2. **`img_widget` not created in dummy-draw mode** — needs to be created/destroyed on switch.
3. **No mode-switch API** — would need something like `lvgl_display_set_dummy_draw(handle, bool)` that toggles the flag, manages LVGL lifecycle, and creates/destroys the image widget.

### How it would interact with the overlay component

The overlay SPI backend's direct gap-area writes (`esp_lcd_panel_draw_bitmap()`) conflict with LVGL's flush when LVGL is running. A mode-switch notification would tell the overlay backend to stop/start direct rendering:

```
App wants slider → pipeline resumes LVGL → overlay stops gap writes
User dismisses slider → pipeline stops LVGL → overlay re-renders gap buffers
```

The overlay component wouldn't own the mode switch — just react to it. The landscape compositing callback (`overlay_cb`) would continue to work in either mode since it modifies the frame buffer before the push path runs.
