# P4 LCD 4.3 Landscape Pipeline Optimization

Experiments run 2026-04-09 on Waveshare ESP32-P4 WiFi6 Touch LCD 4.3 with
QR decoder app in landscape mode. Camera: OV5647 MIPI-CSI. Display: ST7701
MIPI-DSI 480x800 (portrait native, landscape via software rotation).

## System Architecture

The QR decoder pipeline has three concurrent stages sharing two CPU cores
and a single PPA SRM engine:

- **Camera capture** (core 0, priority 5): Captures 1280x960 frames, runs
  PPA crop+scale+rotate (960x960 → 480x480, ~55-60ms per frame)
- **LVGL display** (floating or pinned, priority 5): Renders camera preview
  via image widget, rotates full 800x480 frame 90° CCW via CPU pixel loop
  (~22ms), syncs to DPI vsync
- **QR decode** (core 1, priority 5): Grayscale conversion + quirc detection
  on locked front buffer

All three compete for PSRAM bandwidth. Camera PPA and display CPU rotation
both do large PSRAM DMA/read-write operations.

## Experiment 1: PPA Display Rotation

**Change**: Replace CPU pixel loop in `st7701_landscape_flush_cb` with
`ppa_do_scale_rotate_mirror()` for the 800x480 → 480x800 rotation.

**Result**: PPA rotation works correctly (landscape text renders horizontally,
confirming orientation). But severe contention with camera PPA:

| Metric | CPU rotation | PPA rotation |
|---|---|---|
| Display rotation avg | 23ms | 84ms |
| Display FPS | 7.3-8.4 | 2.0 |
| Camera PPA avg | 61ms | 54ms |
| QR decode fps | 8.4-8.8 | 10.4 |

**Why**: The ESP32-P4 has a single PPA SRM engine. Both camera and display
use `PPA_TRANS_MODE_BLOCKING`, so they serialize. The display rotation waits
behind the camera's PPA operation (~55ms), inflating its total time to ~84ms.
The first frame (before camera started) completed in 15ms, confirming the
PPA hardware itself is fast.

**Verdict**: PPA display rotation is not viable when the camera pipeline also
uses PPA. CPU rotation avoids contention entirely.

**Interaction discovered**: Camera PPA was slightly faster with display PPA
(54ms vs 61ms with CPU rotation) because PPA serialization prevents
simultaneous PSRAM DMA, while CPU rotation runs concurrently and competes
for PSRAM bandwidth.

## Experiment 2: LVGL Core Affinity

**Change**: Pin LVGL task to core 0 (with camera), keeping core 1 dedicated
to QR decode.

**Result** (at same priority 5):

| Metric | LVGL floating (-1) | LVGL pinned core 0 |
|---|---|---|
| Display FPS | 7.3-8.4 | 5.0-8.4 |
| QR decode fps | 8.4-8.8 | 9.1-9.8 |
| QR det/s | 7.3-7.9 | 8.2-9.8 |
| gray time | 23ms | 21-22ms |

**Why**: With LVGL floating, it occasionally runs on core 1 and competes
with QR decode for CPU time. Pinning it to core 0 gives core 1 exclusively
to QR decode, improving decode throughput. Display FPS is more variable
because LVGL now always time-slices with camera on core 0.

## Experiment 3: LVGL Priority

**Change**: Bump LVGL from priority 5 to 6 (above camera at 5), still
pinned to core 0.

**Result**:

| Metric | Priority 5 (core 0) | Priority 6 (core 0) |
|---|---|---|
| Display FPS | 5.0-8.4 | 2.4-4.0 |
| QR decode fps | 9.1-9.8 | 12.4-13.4 |
| QR det/s | 8.2-9.8 | 10.4-11.9 |
| Camera FPS | 15-16 | 13-14 |
| Camera PPA avg | 60ms | 65ms |
| gray time | 21-22ms | 15-16ms |

**Why display dropped**: Higher LVGL priority preempts camera more
aggressively. Camera produces fewer frames (13fps vs 15fps). `push_frame`
uses a 1ms non-blocking `lvgl_port_lock` — with LVGL spending more time in
its render-flush cycle (holding the lock), the camera's push attempts fail
more often.

**Why QR improved dramatically**: Camera dropping from 15fps to 13fps means
2 fewer PPA DMA operations per second (~5MB/s less PSRAM traffic). QR
decode on core 1 is PSRAM-bandwidth-bound (gray conversion reads 460KB,
quirc works in PSRAM). Less PSRAM contention → faster QR processing. Gray
conversion dropped from 21ms to 15ms (30% faster).

**Key insight**: QR decode performance is limited by PSRAM bandwidth, not
CPU time on core 1. Reducing PSRAM traffic from camera PPA operations
directly improves QR throughput.

## Experiment 4: Demand-Driven Frame Skip

**Change**: Skip camera PPA processing when no consumer has used the
previous frame. Tested three variants:

### Skip-at-most-1 (LVGL pinned core 0)
Skip if `!front_consumed && !skipped_last`. Process on the next frame
regardless.

| Metric | No skip | Skip-1 |
|---|---|---|
| Camera FPS | 15 | 30 |
| PPA ops/2s | 30 | 30 |
| Skipped/2s | 0 | 30 |
| Display FPS | 7.3-8.4 | 7.9-8.4 |
| Display skip % | 43-55% | 22-23% |
| QR decode fps | 8.4-8.8 | 7.9-10.1 |

**Why limited impact**: The skip-1 limit still processed ~15fps (same as
no-skip). Camera sensor jumped to 30fps because skipped callbacks return
instantly, but PPA work per second was unchanged.

### Unbounded skip, QR consumer only (LVGL pinned core 0)
Skip whenever `!front_consumed`, no limit. Only QR consumer's `lock_frame`
sets `front_consumed = true`.

| Metric | No skip | Unbounded (QR only) |
|---|---|---|
| PPA ops/2s | 30 | 14-18 |
| Skipped/2s | 0 | 50-52 |
| Display FPS | 7.3-8.4 | 5.4-7.5 |
| Display skip % | 43-55% | 3-10% |
| QR decode fps | 8.4-8.8 | 11.4-12.5 |
| CAM PPA avg | 61ms | 52-56ms |
| gray time | 23ms | 16-17ms |

**Why display dropped**: Only QR consumption triggered new frame processing.
Display demand was invisible to the skip logic. Camera processed ~8fps to
match QR demand, but display needed ~7fps too and was starved.

### Unbounded skip, both consumers (LVGL pinned core 0)
Added `front_consumed = true` when `push_frame` succeeds (display accepted
the frame), in addition to QR consumer's `lock_frame`.

| Metric | QR only | QR + display |
|---|---|---|
| PPA ops/2s | 14-18 | 17-18 |
| Display FPS | 5.4-7.5 | 6.4-6.9 |
| Display skip % | 3-10% | 4-8% |
| QR decode fps | 11.4-12.5 | 10.8-11.4 |

Display recovered partially. Both consumers now drive demand.

### Unbounded skip, both consumers, LVGL floating
Reverted LVGL affinity to -1 (float between cores).

| Metric | Pinned core 0 | Floating |
|---|---|---|
| Display FPS | 6.4-6.9 | 6.4-8.5 |
| QR decode fps | 10.8-11.4 | 9.8-11.9 |
| Display skip % | 4-8% | 0-6% |
| DISP INTERVAL avg | — | 144-156ms |

Display improved (LVGL can use core 1 when QR is between frames). QR
slightly more variable but still well above baseline.

**Display idle time**: With 144-156ms average interval between successful
pushes and ~22ms render+flush, the display waits ~122-134ms for each new
frame (idle ~85% of each cycle).

### Skip-up-to-2, both consumers, LVGL floating
Skip if `!front_consumed && skip_count < 2`. Process on the 3rd frame
regardless.

| Metric | Unbounded | Skip-up-to-2 |
|---|---|---|
| PPA ops/2s | 17-18 | 20-22 |
| Skipped/2s | 50-52 | 33-44 |
| Display FPS | 6.4-8.5 | 5.9-7.9 |
| QR decode fps | 9.8-11.9 | 10.0-13.0 |
| DISP INTERVAL avg | 144-156ms | 128-187ms |

Middle ground: more PPA work than unbounded, less than no-skip. Display
and QR both in acceptable ranges.

### Skip-at-most-1, both consumers, LVGL floating (final configuration)
Skip if `!front_consumed && skip_count < 1`. Process every other frame
at minimum. Both display push_frame and QR lock_frame set front_consumed.

| Metric | No skip baseline | Skip-at-most-1 |
|---|---|---|
| Camera FPS | 15 | 28-30 |
| PPA ops/2s | 30 | 29-30 |
| Skipped/2s | 0 | 27-31 |
| Display FPS | 7.3-8.4 | 7.4-12.3 |
| Display skip % | 43-55% | 7-26% |
| DISP INTERVAL avg | — | 83-134ms |
| QR decode fps | 8.4-8.8 | 9.2-11.9 |
| gray time | 23ms | 16-20ms |
| quirc time | 68-70ms | 64-77ms |

Best overall balance. Display FPS improved to 7.4-12.3 (best across all
experiments). QR decode fps at 9.2-11.9 (above baseline). Camera sensor
runs at native ~30fps with half the frames skipped. PPA processing count
is similar to no-skip (~15fps processed), but the demand-driven skip
ensures frames are processed when consumers need them, not on a fixed
cadence.

**Why this works better than no-skip at the same PPA processing rate**:
Without skip, the camera processes every frame at ~15fps (bottlenecked by
PPA time). With skip-at-most-1, the camera sensor runs at 30fps but only
processes ~15fps — the difference is *which* frames get processed. The
demand signal from `front_consumed` ensures processing aligns with consumer
readiness, reducing display skip % from 43-55% to 7-26%. More processed
frames arrive when the display is actually ready to accept them.

**Data reliability note**: QR det/s varied significantly between runs
(0.5-5.9) depending on camera positioning and lighting. The decode fps
(how fast the decoder processes frames) is the reliable throughput metric;
det/s depends on scene content and is not comparable across runs without
controlled conditions.

## Triple Buffer and Consumer Paths

The triple buffer (back, front, locked) exists so three operations can
proceed concurrently without blocking:

- **Camera** writes to back via PPA DMA (~55ms)
- **QR decoder** holds locked for gray conversion + quirc (~80ms)
- **Front** is the latest completed frame, available for the next QR lock

Without three buffers, the camera would block whenever QR is still
processing — no free buffer to write into.

The display and QR decoder consume frames through different mechanisms:

1. Camera PPA processes frame into back buffer
2. `push_frame` does a **memcpy** of back buffer into LVGL's own `cam_buf`
   (non-blocking, 1ms LVGL lock timeout). The display gets a snapshot and
   does not hold a triple-buffer slot.
3. Back buffer promoted to front
4. QR consumer locks front buffer via `cam_pipeline_lock_frame`, holding
   it for the duration of decode (~80ms). This is the only consumer that
   occupies a triple-buffer slot.

The `front_consumed` flag is set by either consumer — display (`push_frame`
success) or QR (`lock_frame`) — signaling that the camera should process
the next frame. Both consumers can use data from the same processed frame
(display via memcpy, QR via lock), but they don't compete for buffer slots.

## Summary of Interactions

| Factor | Display FPS | QR decode | Camera PPA |
|---|---|---|---|
| PPA display rotation | Severe drop (contention) | Improved (serialized PSRAM) | Faster avg (serialized) |
| LVGL pinned core 0 | Slightly worse | Better (core 1 free) | Same |
| LVGL priority 6 | Much worse | Much better (less PSRAM traffic) | Slower (preempted) |
| Frame skip (unbounded) | Worse (starved) | Better (less PSRAM traffic) | Faster avg |
| Both consumers drive demand | Recovered | Slightly lower | Similar |
| LVGL floating + skip-1 | **Best balance** | Above baseline | Similar rate, better timing |

## Key Findings

**1. PSRAM bandwidth is the dominant bottleneck, not CPU time.** Camera PPA
DMA operations (~2.3MB per frame) compete with QR decode's PSRAM reads
(460KB gray conversion) and the CPU rotation's PSRAM read/write (768KB
each way). Reducing unnecessary PPA operations directly reduces PSRAM
contention for all other consumers.

**2. PPA display rotation is not viable alongside camera PPA.** The single
SRM engine serializes both operations, adding ~55ms of wait time per
display frame. CPU rotation at 22ms is significantly faster despite using
CPU cycles, because it runs concurrently with PPA rather than queuing
behind it.

**3. Core affinity is a tradeoff, not a pure win.** Pinning LVGL to core 0
helps QR decode (core 1 is dedicated) but hurts display (LVGL competes
with camera for core 0 time). Floating affinity lets LVGL opportunistically
use whichever core is free, giving better overall balance.

**4. Task priority has outsized effects via PSRAM bandwidth.** Raising LVGL
priority from 5 to 6 slowed the camera by only 2fps, but that translated
to ~5MB/s less PSRAM DMA traffic, which improved QR gray conversion by 30%.
The indirect effect (bandwidth) dwarfed the direct effect (CPU preemption).

**5. Frame skip effectiveness depends on the limit.** Unbounded skip starves
the display (processes only ~8fps, matching QR demand alone). Skip-at-most-1
processes ~15fps (same as no-skip) but aligns processing with consumer
readiness, dramatically improving display skip rate (43-55% → 7-26%).
The skip limit should match the gap between camera rate and consumer demand.

**6. The display path does not hold triple-buffer slots.** `push_frame`
does a memcpy into LVGL's own buffer and returns. Only the QR consumer
locks a triple-buffer slot for extended processing (~80ms). With frame
skip reducing unnecessary PPA work, the triple buffer's concurrency
benefit is diminished — the camera is already skipping most frames during
QR's hold time.

## Final Configuration

- CPU rotation (not PPA) for landscape display flush
- LVGL floating affinity (-1), priority 5
- Camera capture on core 0, priority 5
- QR decode on core 1, priority 5
- Demand-driven frame skip (at most 1 consecutive skip)
- Both display and QR consumer drive the `front_consumed` demand signal

## Portrait Mode Expectations

Portrait eliminates the ~22ms CPU rotation entirely — LVGL flushes
directly to the DPI panel in native 480x800 orientation. This removes
a major source of PSRAM bandwidth consumption (768KB read + 768KB write
per flush), which should improve both display FPS and QR decode throughput.
The camera PPA still runs for crop+scale+rotate, but with less overall
PSRAM contention.
