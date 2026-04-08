# DSI Landscape Rendering Architecture (ST7701, ESP32-P4)

The Waveshare ESP32-P4 WiFi6 Touch LCD 4.3 has a 480×800 portrait ST7701
MIPI-DSI panel mounted sideways for landscape use. Getting LVGL to render
800×480 landscape content onto this physically portrait panel is non-trivial
because DPI panels have no hardware rotation capability and esp_lvgl_port's
built-in rotation flags are incompatible with the DSI rendering path.

This document captures the full architecture so we stop re-deriving it.

## The Panel

ST7701 is a MIPI-DSI DPI (video-mode) panel. It has no internal framebuffer
(no GRAM) and no MADCTL register. It displays whatever pixel stream arrives
in its fixed native scan order: 480 pixels wide, 800 lines tall, top to
bottom. There is no way to tell the panel to scan in a different direction.

SPI panels (ST7796, ST7789) have MADCTL and can swap_xy/mirror in hardware
for free landscape rotation. ST7701 cannot.

## Two Rendering Paths

The firmware has two distinct display modes that never run simultaneously:

### 1. LVGL Screen Mode (seedsigner-c-modules screens)

LVGL renders UI screens (main menu, button lists, settings). The LVGL handler
task calls `lv_timer_handler()`, which renders dirty areas into draw buffers
and calls the flush callback to push pixels to the panel.

### 2. Camera Pipeline Mode

The camera pipeline (`esp-camera-pipeline`) captures frames, PPA-rotates them
for the physical panel orientation, and pushes them to the panel via an LVGL
image widget on a dedicated camera screen. Overlays (FPS stats, QR decode
text) are pre-rotated and composited as LVGL image widgets on the same screen.

The camera pipeline operates on its own LVGL screen. `pipeline_create()`
loads the camera screen; `pipeline_destroy()` tears it down and returns
control to LVGL screen mode. See `lvgl-null-active-screen-crash.md` for
why a blank screen must be loaded before deleting the camera screen.

## esp_lvgl_port v2.7.2 DSI Display Modes

`lvgl_port_add_disp_dsi()` supports two buffer strategies selected by the
`avoid_tearing` flag. They are **mutually exclusive architectures**, not
flags that can be combined freely.

### avoid_tearing = true (current portrait mode)

```
Panel hardware framebuffers (from esp_lcd_dpi_panel_get_frame_buffer)
    ↓
LVGL renders directly into panel framebuffers (RENDER_MODE_DIRECT)
    ↓
Flush callback: wait for vsync semaphore, call draw_bitmap
    ↓
DPI hardware scans the framebuffer continuously
```

- Buffers: panel's own DPI framebuffers (2 or 3), not LVGL-allocated
- `disp_ctx->draw_buffs[]` is NOT populated — stays NULL
- LVGL dimensions must match physical panel: 480×800
- `lv_display_set_buffers()` called with `RENDER_MODE_DIRECT`
- `direct_mode` flag is set in disp_ctx
- Vsync callback: `on_refresh_done` fires when panel finishes scanning

### avoid_tearing = false (required for landscape)

```
LVGL-allocated draw buffers (partial or full, from heap_caps_aligned_alloc)
    ↓
LVGL renders into draw buffers (RENDER_MODE_PARTIAL or RENDER_MODE_FULL)
    ↓
Flush callback: rotate buffer, call draw_bitmap with physical coordinates
    ↓
DPI hardware DMA-copies from draw_bitmap argument into panel framebuffer
```

- Buffers: LVGL-allocated, can be partial (e.g. 800×50 bands)
- `disp_ctx->draw_buffs[0]` and `[1]` are populated
- LVGL dimensions can differ from physical panel (e.g. 800×480 landscape)
- `draw_bitmap` uses explicit coordinates, not full-frame
- Vsync callback: `on_color_trans_done` fires when DMA copy completes

## Why sw_rotate Cannot Be Combined With avoid_tearing

esp_lvgl_port's `sw_rotate` flag (with or without PPA) operates in the flush
callback: it takes the rendered buffer, rotates it, then submits via
`draw_bitmap`. This requires:

1. **Rotatable buffers** — LVGL-allocated buffers that contain landscape pixels.
   With `avoid_tearing`, the buffers are the panel's own framebuffers containing
   portrait pixels at physical dimensions. PPA would rotate them, but the output
   goes into a separate PPA output buffer while the flush path still references
   the panel framebuffer.

2. **Coordinate remapping in draw_bitmap** — After rotation, `draw_bitmap` must
   be called with physical portrait coordinates. But the `avoid_tearing` +
   `direct_mode` flush path (line 711-715 of esp_lvgl_port_disp.c) calls
   `draw_bitmap(0, 0, lv_disp_get_hor_res, lv_disp_get_ver_res, color_map)`,
   using LVGL's logical dimensions. If LVGL is 800×480 landscape, it submits
   an 800×480 rectangle to a 480×800 panel — wrong dimensions.

3. **Buffer size match** — `esp_lcd_dpi_panel_get_frame_buffer` returns buffers
   sized for physical dimensions (480×800). If LVGL is configured as 800×480,
   `buffer_size = hres * vres` doesn't match the panel framebuffer size.

The code paths in `lvgl_port_add_disp_priv()` confirm this: the `avoid_tearing`
branch and the `direct_mode`/`full_refresh` branch handle buffer setup
completely differently. They cannot be combined.

## The Landscape Architecture

Landscape on ST7701 requires:

1. **`avoid_tearing = false`** — opt out of panel-owned framebuffers
2. **LVGL configured as 800×480** — `hres = BOARD_LCD_V_RES, vres = BOARD_LCD_H_RES`
3. **Custom flush callback** — replaces esp_lvgl_port's default flush
4. **Own vsync management** — register `on_refresh_done` callback on the DPI panel
5. **Rotation in the flush callback** — rotate 800×480 → 480×800 before `draw_bitmap`

### Flush callback flow

```
LVGL renders 800×480 landscape content into draw buffers
    ↓
Custom flush callback receives the buffer
    ↓
Rotate 90° CCW into a separate PSRAM rotation buffer
    ↓
Drain stale vsync semaphore (take with timeout=0)
    ↓
Wait for vsync (take with portMAX_DELAY)
    ↓
esp_lcd_panel_draw_bitmap(panel, 0, 0, 480, 800, rotated_buffer)
    ↓
Swap double-buffered rotation buffers
    ↓
lv_display_flush_ready()
```

### Rotation options

**CPU loop** (original implementation, commit 490f027):
- Pixel-by-pixel nested loop: `out[py * 480 + px] = fb[px * 800 + (799 - py)]`
- 384,000 pixels at 2 bytes each = 768KB rotated per frame
- Works but slow for large displays

**PPA hardware DMA** (preferred for P4):
- `ppa_do_scale_rotate_mirror()` with `PPA_SRM_ROTATION_ANGLE_90`
- Hardware DMA rotation, near-instant for partial bands
- Requires 128-byte aligned output buffer and buffer_size
- esp_lvgl_port's `LVGL_PORT_ENABLE_PPA` Kconfig creates a PPA client
- Can rotate partial bands (e.g. 800×50) — no minimum size constraint for RGB565

### Buffer allocation

- **Draw buffers**: partial-size, PSRAM, double-buffered (e.g. 800×50×2 = 80KB each)
- **Rotation buffer**: at least as large as draw buffer, 128-byte aligned if using PPA
- **No panel framebuffer management** — `draw_bitmap` handles DMA copy to panel

### Tear prevention

Without `avoid_tearing`, the DPI panel continuously scans its internal
framebuffers. `draw_bitmap` triggers a DMA2D copy from the provided buffer
into the panel's internal buffer. If this copy overlaps with the panel's
scan position, tearing occurs.

Mitigation: wait for the `on_refresh_done` vsync callback before calling
`draw_bitmap`, so the DMA copy runs during the vertical blanking interval.
Drain any stale vsync first (the panel may have fired one while LVGL was
rendering).

## How the Camera Pipeline Differs

The camera pipeline doesn't use LVGL's rendering pipeline for the camera feed:

1. Camera captures frames → PPA rotates to physical portrait orientation
2. Rotated frame is displayed via an LVGL image widget (simple buffer swap)
3. LVGL composites the image widget + overlay widgets in portrait coordinates
4. Standard `avoid_tearing` flush handles display (portrait direct_mode)

Camera overlays (FPS, QR text) are **individually pre-rotated** using the
`overlay_rotated.c` module because LVGL is rendering in portrait mode during
camera pipeline operation. Each overlay:
1. Renders text into a hidden LVGL canvas in landscape coordinates
2. Finds tight bounding box of non-empty pixels
3. CPU-rotates only the cropped sub-rectangle 90° CCW
4. Presents as an `lv_image` widget positioned in landscape coordinates

These CPU rotations operate on small buffers (e.g. 130×60 crop = ~31KB) where
PPA setup overhead would rival CPU time. PPA's advantage is on full-frame
rotation (768KB+) where DMA bandwidth dominates.

## Why the Two Paths Don't Share Rotation Code

| Aspect | LVGL Screen Landscape Flush | Camera Pipeline Overlays |
|--------|----------------------------|--------------------------|
| What's rotated | Entire LVGL framebuffer (800×480) | Small text regions (~130×60) |
| When | Every flush (continuous) | On text change (sporadic) |
| Rotation method | PPA hardware DMA (preferred) | CPU loop (adequate for small buffers) |
| Buffer source | LVGL draw buffer | Hidden LVGL canvas |
| Output destination | `draw_bitmap` → panel | `lv_image` widget → LVGL composites |
| Active during | LVGL screen mode | Camera pipeline mode |

The two modes never run simultaneously. `pipeline_destroy()` tears down the
camera screen and its overlays before LVGL screens take over, and vice versa.

## History

- **490f027**: Original landscape flush (CPU rotation, custom vsync, worked)
- **b73fb77**: Migrated to `esp_lvgl_adapter` — landscape flush rewritten
- **83a1a38**: Migrated back to `esp_lvgl_port v2.7.2` — landscape flush
  dropped ("not yet implemented"), portrait `direct_mode + avoid_tearing`
  used instead. Research doc `docs/lvgl-display-rotation.md` committed in
  this revision but later deleted from tree (still accessible via git show).
- **Current**: Portrait rendering only. Landscape flush needs to be
  re-implemented following the architecture above.

## Key Invariants

1. `avoid_tearing` and `sw_rotate` are incompatible on DSI panels in
   esp_lvgl_port v2.7.2. Do not attempt to combine them.
2. Landscape flush requires `avoid_tearing = false` and a custom flush
   callback that rotates to portrait before `draw_bitmap`.
3. The camera pipeline manages its own rotation (PPA for frames, CPU for
   overlays) independent of the LVGL display rotation path.
4. Never delete the LVGL active screen without loading a replacement first
   (see `lvgl-null-active-screen-crash.md`).
