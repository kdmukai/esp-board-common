# DSI Landscape Simplification Plan

## The Problem

The QR decoder app has three completely different overlay rendering paths
depending on display type and orientation. The DSI landscape path (ST7701,
P4 LCD 4.3) is by far the most complex: it renders overlays as hidden LVGL
canvases, finds tight bounding boxes, CPU-rotates cropped sub-rectangles
90° CCW, and presents them as `lv_image` widgets in portrait coordinates.

This complexity exists because LVGL currently runs in portrait mode on the
DSI panel (480×800). The camera pipeline's PPA rotation corrects for the
physical mounting angle of the camera sensor, which is independent of
display orientation. But LVGL widgets and text still need to be
individually pre-rotated to appear correctly in landscape.

## The Insight

If LVGL runs in landscape (800×480) with a rotation flush callback, the
camera pipeline on DSI boards no longer needs *any* special overlay
handling. Standard LVGL widgets render in landscape coordinates. The flush
callback rotates everything — camera image widget, labels, any future UI —
to portrait for the panel. One rotation point replaces N+1.

This only applies to DSI boards. SPI boards have a fundamentally different
constraint (shared SPI bus forces dummy-draw mode where LVGL is stopped),
and their overlay path must remain as-is.

## What Gets Eliminated

### Code removed from QR decoder app (`apps/qr_decoder/main/main.c`)

All `#if BOARD_LANDSCAPE && BOARD_DISPLAY_DRIVER == DISPLAY_ST7701` blocks:
- `rotated_overlay_t` declarations for FPS and QR overlays (~lines 63-88)
- Landscape FPS stats formatting + `rotated_overlay_update()` calls (~lines 127-134)
- Landscape QR decode callback path with `rotated_overlay_update/show` (~lines 240-245)
- Landscape overlay initialization: `rotated_overlay_init()` for QR (ARGB8888, 800×80)
  and FPS (RGB565, 130×130) with gap calculations (~lines 362-397)

The DSI landscape path **collapses into the DSI portrait path**. Both use
plain `lv_label` widgets. The only compile-time split remaining is DSI vs
SPI (which is irreducible).

### Modules that become dead code for DSI landscape

- **`overlay_rotated.h/c`** — The entire pre-render → bbox-crop → CPU-rotate →
  lv_image pipeline. Still needed by `camera_manager.c` in the micropython-builder
  repo during the transition, but once the landscape flush is in board_init.c,
  camera_manager's overlay code can be simplified the same way.

### Memory recovered

The DSI landscape rotated overlays currently allocate ~1.1 MB of PSRAM:
- FPS: 130×130 RGB565 canvas + rotated buffer
- QR: 800×80 ARGB8888 canvas + 800×80 rotated buffer

With plain LVGL labels, the overhead is negligible — just LVGL's internal
widget memory.

## What Changes in board_common

### 1. board_init.c — Landscape flush callback (the core change)

Restore the custom flush callback for `BOARD_DISPLAY_DRIVER == DISPLAY_ST7701`
when `landscape == true`. This was working in commit `490f027` but lost
during the adapter migration (`b73fb77` → `83a1a38`).

**Reference implementation:** `git show 490f027 -- src/board_init.c`

**Architecture:**
```
LVGL renders 800×480 into a full-frame SPIRAM buffer (full_refresh)
    ↓
Custom flush callback receives the complete frame
    ↓
CPU rotates entire frame 90° CCW into double-buffered PSRAM output
    ↓
Wait for vsync, then esp_lcd_panel_draw_bitmap with portrait coordinates
    ↓
Swap rotation buffers, lv_display_flush_ready()
```

**Why full-frame CPU rotation (not PPA partial bands):** The `490f027`
implementation used a straightforward CPU pixel loop over the entire
800×480 frame. This was tested and working. A PPA-accelerated partial-band
approach could reduce memory and improve throughput but has never been
implemented or validated — see "Future: PPA optimization" below.

**Specific changes to `lvgl_port_setup()`:**

a. The `#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701` block currently
   hard-codes portrait dimensions and `direct_mode = 1`. For landscape,
   swap dimensions and use a full-frame SPIRAM buffer:
   ```c
   #if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
       int lvgl_hres, lvgl_vres;
       if (landscape) {
           lvgl_hres = BOARD_LCD_V_RES;   // 800
           lvgl_vres = BOARD_LCD_H_RES;   // 480
       } else {
           lvgl_hres = BOARD_LCD_H_RES;   // 480
           lvgl_vres = BOARD_LCD_V_RES;   // 800
       }
   ```
   Portrait keeps `direct_mode = 1` with `avoid_tearing = true` (the
   existing path). Landscape uses:
   ```c
   .buffer_size = lvgl_hres * lvgl_vres * sizeof(lv_color16_t),  // full frame
   .double_buffer = true,
   .flags = { .buff_spiram = 1 },
   ```

b. For DSI landscape, use `avoid_tearing = false`. The custom flush
   manages its own vsync timing via a DPI panel `on_refresh_done`
   callback + semaphore. The library's `avoid_tearing` path would
   conflict with this.

c. After `lvgl_port_add_disp_dsi()`, set up the landscape flush:
   - Allocate two full-frame rotation output buffers in PSRAM
     (`BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t)` each,
     ~768KB per buffer, ~1.5MB total)
   - Create a binary semaphore for vsync synchronization
   - Register `on_refresh_done` vsync callback on the DPI panel
   - Override flush callback via `lv_display_set_flush_cb()`

d. The flush callback (`st7701_landscape_flush_cb`):
   - If not `lv_display_flush_is_last()`, just call `flush_ready` and
     return. With `full_refresh`, LVGL sends the complete frame in one
     flush call, so this guard is a safety check — not band skipping.
   - CPU-rotate the full 800×480 landscape frame 90° CCW into the back
     rotation buffer. The pixel mapping:
     `out[py * H_RES + px] = fb[px * V_RES + (V_RES - 1 - py)]`
   - Drain any stale vsync semaphore, then wait for vsync
   - `esp_lcd_panel_draw_bitmap()` with portrait dimensions (H_RES × V_RES)
   - Swap double-buffer index, call `flush_ready`

**Rotation buffer memory:** Two full-frame buffers at 480×800×2 = 768KB
each, ~1.5MB total PSRAM. This replaces the ~1.1MB used by the rotated
overlay canvases, so net PSRAM increase is ~400KB. However, the
`avoid_tearing = true` portrait path uses DPI triple-buffered framebuffers
managed by esp_lvgl_port — the landscape path's rotation buffers may
partially offset that allocation depending on how the library handles it.

### 2. board_pipeline.c — Remove DSI landscape special case

The current code forces physical portrait dimensions for DSI landscape:
```c
#if BOARD_LANDSCAPE && BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    .display_width  = BOARD_LCD_H_RES,      // 480
    .display_height = BOARD_LCD_V_RES,      // 800
```

Once LVGL runs in landscape, this special case is removed. DSI landscape
uses `BOARD_DISP_H_RES` × `BOARD_DISP_V_RES` (800×480) like every other
board. The pipeline creates a 480×480 square camera display within the
800×480 landscape canvas — same as SPI landscape boards.

The camera PPA rotation (`BOARD_CAMERA_ROTATION`) does NOT change. It
corrects for the physical mounting angle of the sensor on the PCB. On a
live camera feed viewed on the same device, the board's physical rotation
moves both camera and display together — no software compensation is
needed for display orientation changes.

### 3. QR decoder app — Collapse DSI landscape into DSI portrait path

Replace the three-way overlay conditional with a two-way split: DSI vs SPI.

**Before (3 paths):**
```
#if BOARD_LANDSCAPE && BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    // Path A: rotated_overlay_t canvases (complex)
#elif BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    // Path B: plain LVGL labels
#else
    // Path C: overlay_text direct rendering (SPI)
#endif
```

**After (2 paths):**
```
#if BOARD_DISPLAY_DRIVER == DISPLAY_ST7701
    // Paths A+B merged: plain LVGL labels (landscape or portrait)
#else
    // Path C: overlay_text direct rendering (SPI, unchanged)
#endif
```

The merged DSI path uses `lv_label` widgets positioned with
`BOARD_DISP_H_RES` / `BOARD_DISP_V_RES` macros, which resolve correctly
for both portrait and landscape (they swap via the `BOARD_LANDSCAPE`
macro in `board.h`).

The `board_init()` call currently has a deliberate override that forces
`.landscape = false` for DSI even when `BOARD_LANDSCAPE=1` (main.c
lines 293-295). This was necessary because LVGL ran portrait and
overlays handled rotation individually. With the flush callback in
place, this override is removed: `.landscape = BOARD_LANDSCAPE` lets
the board config drive the orientation, and board_init handles rotation
at the display level.

### 4. camera_manager.c (micropython-builder) — Same simplification later

The `camera_manager` module in `seedsigner-micropython-builder` has the
same overlay complexity. Once the landscape flush is proven in the QR
decoder app, apply the identical simplification: replace
`rotated_overlay_t` with plain LVGL labels, remove the canvas/rotate
machinery. This is a follow-on task, not part of this plan.

## Implementation Order

### Phase 1: Landscape flush in board_init.c

1. Restore `st7701_landscape_flush_cb` and supporting code (double-buffered
   rotation buffers, vsync semaphore, DPI event callback) to `board_init.c`,
   adapted from commit `490f027` to the current `esp_lvgl_port` API
2. Update `lvgl_port_setup()` to select landscape dimensions, full-frame
   SPIRAM buffer, and `avoid_tearing = false` when `landscape && ST7701`
3. Build and flash the **touch_test** app with `LANDSCAPE=1` to verify
   basic LVGL rendering in landscape without camera pipeline complications
4. Verify touch coordinates map correctly — the GT911 landscape flags
   already exist in `board_init.c` (lines 409-413), so this likely works
   but needs confirmation

### Phase 2: QR decoder app simplification

5. Update `board_pipeline.c` — remove DSI landscape dimension special case
6. Update `apps/qr_decoder/main/main.c` — collapse DSI overlay paths,
   remove the `.landscape = false` DSI override (lines 293-295)
7. Build and flash the QR decoder app with `CONFIG_BOARD_LANDSCAPE=y`
8. Verify: camera feed displays correctly in landscape, QR decode labels
   appear as readable horizontal text, FPS stats display correctly

### Phase 3: Cleanup

9. Once the QR decoder app works, evaluate whether `overlay_rotated.c/h`
    can be removed from the build for DSI boards or if
    `camera_manager.c` in the micropython-builder still depends on it
10. Update `text-overlay-architecture.md` and
    `lvgl-widget-feasibility-by-display-path.md` — the DSI landscape
    sections are now obsolete
11. Update `dsi-landscape-rendering-architecture.md` with any findings
    from the implementation

## Risks and Unknowns

**CPU rotation throughput.** The original `490f027` implementation used
a CPU pixel loop over 800×480×2 = 768KB per frame. This was working at
the time, but the exact fps was not recorded. If the CPU rotation
becomes a bottleneck (especially at higher target frame rates), the PPA
partial-band approach described below is the upgrade path.

**Touch coordinate mapping.** The GT911 touch controller reports
coordinates in physical portrait space. The landscape `swap_xy` /
`mirror_y` flags already exist in `board_init.c` (lines 409-413) and
should handle this, but needs verification on hardware. Check whether
esp_lvgl_port's touch registration applies these flags correctly when
LVGL dimensions are swapped.

**esp_lvgl_port API differences since 490f027.** The original was
written against an earlier version of `esp_lvgl_port`. The
`lvgl_port_add_disp_dsi()` call, `lvgl_port_display_dsi_cfg_t` struct,
and buffer management may have changed. Compare the original code
against the current API before implementing.

## Future: PPA Full-Frame Rotation (next step)

The immediate optimization is replacing the CPU pixel loop with a single
`ppa_do_scale_rotate_mirror()` call — same full-frame architecture, just
hardware-accelerated rotation. See `docs/ppa-full-frame-rotation-plan.md`
for the implementation plan.

## Future: PPA Partial-Band Optimization (later)

If memory reduction is needed, the flush can be restructured to use
the P4's PPA with partial draw bands instead of full-frame buffers:

```
LVGL renders 800×480 into partial draw buffers (e.g. 800×50 bands)
    ↓
Custom flush callback receives each band
    ↓
PPA rotates band 90° CCW → portrait-oriented output buffer
    ↓
esp_lcd_panel_draw_bitmap with portrait coordinates
    ↓
lv_display_flush_ready()
```

This would reduce memory (~80KB rotation buffer vs ~1.5MB) and offload
rotation from CPU to dedicated hardware. Key considerations:

- **PPA buffer requirements:** Output buffer must be 128-byte aligned
  with 128-byte aligned size. Use
  `heap_caps_aligned_alloc(128, size, MALLOC_CAP_SPIRAM)`.
  Buffer must be at least 50×800×2 = 80KB (RGB565, rounded up).
- **Band-level tearing:** Multiple `draw_bitmap` calls per frame means
  partial updates during the active scan region. Band-level tearing is
  less visible than full-frame tearing but may require batching: render
  all bands, then submit in one burst during vsync blanking.
- **flush_is_last handling changes:** With partial bands, every band
  must be rotated and submitted — the `flush_is_last()` early-return
  used in full-frame mode would discard all but the last band.
- **PPA performance:** Research suggests <1ms per 800×50 band. At ~10
  bands per frame and 15fps, ~10ms of PPA time per frame — should be
  acceptable.

## SPI Path — Unchanged

The SPI overlay path (`overlay_text.c/h`) remains exactly as-is. The
constraints that drive its complexity — shared SPI bus, dummy-draw mode,
direct panel writes — are hardware limitations that no driver change can
eliminate. The conditional split between DSI and SPI is irreducible.
