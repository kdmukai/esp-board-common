# PPA Full-Frame Rotation Upgrade Plan

## Outcome (2026-04-09)

**Implemented and tested — not viable.** PPA rotation works correctly but
the single PPA SRM engine is shared with the camera pipeline's
crop+scale+rotate. Both use `PPA_TRANS_MODE_BLOCKING`, serializing on the
hardware. Display rotation averaged 84ms (vs 23ms for CPU rotation) because
it waited behind the camera's ~55ms PPA operations. Display FPS dropped
from 7-8fps to 2fps. Reverted to CPU rotation.

The PPA hardware itself is fast (first frame completed in 15ms before the
camera started). The problem is purely contention. PPA display rotation
would be viable for apps without camera PPA usage (e.g., static LVGL
screens).

Full experiment results in `docs/knowledge/p4-lcd43-landscape-pipeline-optimization.md`.

## Background

The DSI landscape flush callback in `board_init.c` currently uses a CPU
nested pixel loop to rotate the full 800×480 LVGL frame 90° CCW into a
480×800 portrait buffer. This works (implemented in this session, based
on the proven approach from commit `490f027`) but ties up a CPU core
during rotation.

The ESP32-P4 has a PPA (Pixel Processing Accelerator) with an SRM
(Scale-Rotate-Mirror) engine that can do the same rotation in hardware
via DMA, freeing the CPU. The camera pipeline already uses PPA for
960×960 scale+rotate operations at ~15fps, so 800×480 rotation is well
within its capability.

## What Changes

Replace the CPU pixel loop in `st7701_landscape_flush_cb` with a single
`ppa_do_scale_rotate_mirror()` call. Everything else stays the same:
full-frame draw buffer, double-buffered rotation output, vsync timing,
`flush_is_last()` guard.

### Current CPU rotation (in `board_init.c`)
```c
for (int px = 0; px < BOARD_LCD_H_RES; px++) {
    for (int py = 0; py < BOARD_LCD_V_RES; py++) {
        int fb_x = DISP_W - 1 - py;
        out[py * BOARD_LCD_H_RES + px] = fb[px * DISP_W + fb_x];
    }
}
```

### Replacement PPA rotation
```c
ppa_srm_oper_config_t srm = {
    .in.buffer      = fb,
    .in.pic_w       = BOARD_LCD_V_RES,      // 800 (landscape width)
    .in.pic_h       = BOARD_LCD_H_RES,      // 480 (landscape height)
    .in.block_w     = BOARD_LCD_V_RES,      // full width
    .in.block_h     = BOARD_LCD_H_RES,      // full height
    .in.block_offset_x = 0,
    .in.block_offset_y = 0,
    .in.srm_cm      = PPA_SRM_COLOR_MODE_RGB565,
    .out.buffer     = out,
    .out.buffer_size = rot_buf_aligned_sz,
    .out.pic_w      = BOARD_LCD_H_RES,      // 480 (portrait width)
    .out.pic_h      = BOARD_LCD_V_RES,      // 800 (portrait height)
    .out.block_offset_x = 0,
    .out.block_offset_y = 0,
    .out.srm_cm     = PPA_SRM_COLOR_MODE_RGB565,
    .rotation_angle = PPA_SRM_ROTATION_ANGLE_90,  // 90° CCW
    .scale_x        = 1.0,
    .scale_y        = 1.0,
    .mode           = PPA_TRANS_MODE_BLOCKING,
};
ppa_do_scale_rotate_mirror(st7701_ppa_client, &srm);
```

## PPA API Findings

### No maximum dimension limit
The PPA driver (`ppa_srm.c`) validates that blocks fit within pictures
and output fits in the buffer, but imposes **no maximum width/height**.
The hardware processes in 18×18 pixel blocks internally (transparent to
the caller via DMA2D). The camera pipeline already processes 960×960
frames — 800×480 is smaller.

### Buffer alignment requirements
- **Output buffer address**: must be aligned to cache line size (128
  bytes for PSRAM on P4)
- **Output buffer_size**: must be aligned to cache line size
- **Input buffer**: no alignment required (can be anywhere: RAM, flash,
  PSRAM)

Current code uses `heap_caps_malloc()` for rotation buffers. Must change
to `heap_caps_aligned_alloc(128, ...)` for the output buffers. The LVGL
draw buffers (PPA input) don't need alignment changes.

### Rotation angles (counterclockwise)
- `PPA_SRM_ROTATION_ANGLE_90` — 90° CCW (matches current CPU rotation)
- `PPA_SRM_ROTATION_ANGLE_270` — 270° CCW (= 90° CW)

The current CPU loop implements 90° CCW:
`portrait(px, py) = landscape(DISP_W-1-py, px)`. Use
`PPA_SRM_ROTATION_ANGLE_90`. **Verify empirically** — if the image
appears rotated the wrong way, switch to `_ANGLE_270`.

### Color mode
`PPA_SRM_COLOR_MODE_RGB565` — matches the LVGL and panel pixel format.
No byte-swap or RGB-swap needed for DSI (unlike SPI panels).

### Blocking mode
`PPA_TRANS_MODE_BLOCKING` is simplest — the function returns when the
DMA transfer completes. Non-blocking mode could allow overlapping
rotation with other work but adds callback complexity that isn't
needed here.

## Implementation Steps

### 1. Add PPA client lifecycle

At init time (in the landscape setup block of `lvgl_port_setup()`):
```c
#include "driver/ppa.h"

static ppa_client_handle_t st7701_ppa_client = NULL;

// In the landscape setup:
ppa_client_config_t ppa_cfg = { .oper_type = PPA_OPERATION_SRM };
ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &st7701_ppa_client));
```

The PPA client is shared with the camera pipeline's PPA client — they
use separate client handles but the same hardware engine, which
serializes operations automatically.

### 2. Change rotation buffer allocation

Replace `heap_caps_malloc` with 128-byte aligned allocation:
```c
size_t rot_buf_sz = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t);
size_t rot_buf_aligned_sz = (rot_buf_sz + 127) & ~127;  // round up to 128
st7701_rot_buf[0] = heap_caps_aligned_alloc(128, rot_buf_aligned_sz, MALLOC_CAP_SPIRAM);
st7701_rot_buf[1] = heap_caps_aligned_alloc(128, rot_buf_aligned_sz, MALLOC_CAP_SPIRAM);
```

Store `rot_buf_aligned_sz` in a static for use in the PPA config's
`out.buffer_size` field.

### 3. Replace CPU loop with PPA call

In `st7701_landscape_flush_cb`, replace the nested loop with the PPA
config struct + `ppa_do_scale_rotate_mirror()` call shown above.

### 4. Verify rotation direction

Flash and check visually. If the image is rotated the wrong way,
switch between `PPA_SRM_ROTATION_ANGLE_90` and `_ANGLE_270`.

### 5. Measure performance

Add `ESP_LOGI` with `esp_timer_get_time()` around the PPA call to
measure rotation time. Compare against the CPU loop. Expected: PPA
should be significantly faster (hardware DMA vs CPU pixel loop over
768KB).

## Risks

**PPA contention with camera pipeline.** The PPA SRM engine is shared.
The camera pipeline uses it for every frame (scale+rotate camera input).
The flush callback also uses it for every frame (rotate LVGL output).
Both use `PPA_TRANS_MODE_BLOCKING`, so they serialize. If both try to
use PPA simultaneously, one blocks until the other finishes. This should
be fine — they run on different tasks and the serialization adds at most
one PPA operation latency. But measure fps to confirm no regression.

**LVGL draw buffer alignment.** The PPA input buffer is the LVGL draw
buffer, which was allocated by LVGL (via esp_lvgl_port or our manual
`lv_display_set_buffers`). The PPA driver documentation says input
buffers don't need alignment, but `esp_cache_msync` is called on the
input window. Verify the LVGL draw buffers are in PSRAM (they should be
— we allocated with `MALLOC_CAP_SPIRAM`).

## What This Does NOT Change

- Architecture: still full-frame, double-buffered, vsync-synced
- Memory: same ~1.5MB rotation buffers (plus alignment padding)
- Flush callback logic: same `flush_is_last()` guard, same vsync wait
- SPI path: completely unaffected
- Partial-band approach: remains a future option for memory reduction
