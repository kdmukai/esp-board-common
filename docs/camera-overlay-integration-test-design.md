# Camera Preview Overlay — Integration Test Design

Status: **design converged; building the generic seam in this repo.** The
`camera_preview_overlay` component has landed in `seedsigner-lvgl-screens`
(`f6fee6a`, branch `feat/camera-preview-overlay`). The work splits across two repos by
dependency seam (see "Repo split" below): the generic `scan_coordinator` + engine callback
build here; the SeedSigner-specific overlay presenter integrates in the
micropython-builder. This doc records the decisions for both halves.

## Goal

Render and validate the declarative camera-preview overlay over the live QR-decode
preview on the P4 LCD 4.3 (ST7701/DSI, the `sweep_480` target), exercising every state
the overlay spec defines — progress bar fill, percent label, green (new) dot, gray
(repeated) dot, hidden (miss/none) dot, the scanning toggle, and the terminal
completion transition — and measuring the translucent status bar's per-frame re-blend
cost against the existing fps / px-m HUD.

This is a **rendering + integration** test. It does NOT decode animated QR for real
(no fountain-code reassembly) — that capability lands later with Python `DecodeQR`.

## Repo split — session here, SeedSigner overlay integration in the builder

The overlay is **not** a standalone widget — it's part of the `seedsigner` component in
`seedsigner-lvgl-screens`, which `REQUIRES lvgl + nlohmann_json` and drags the whole
SeedSigner screens runtime (fonts, logos, `gui_constants`/profile init, `input_profile`,
`back_button`). Pulling that into `esp-board-common` would invert layering (the generic
board layer would depend on the SeedSigner UI) and duplicate the screens init the
**micropython-builder already does**. So the work splits along the dependency seam:

- **`esp-board-common` (generic, no SeedSigner dep) — this session:** `scan_coordinator` +
  the engine callback + the `qr_decoder` refactor. Validated on device with a
  **text/serial presenter** that prints the neutral status (`NEW`/`REPEAT`/`MISS`/
  `COMPLETE` + percent) — a fast bare-metal loop, zero SeedSigner coupling.
- **`seedsigner-micropython-builder` (SeedSigner-specific, separate/later session):**
  consumes `board_common` (for `scan_coordinator`) + `esp-camera-pipeline` + its existing
  `deps/seedsigner-lvgl-screens`; wires `camera_preview_overlay` as the **presenter**
  over the live preview. The screen integration + the Path-A per-frame re-blend / fps
  measurement happen here, where the screens context already stands up.

The UI-agnostic presenter design is exactly what makes this split clean: **the session
doesn't move — only the presenter's home does.** The neutral-status →
`camera_overlay_frame_status_t` mapping lives in the builder's presenter.

`qr_decoder` stays the lean reference (refactored onto `scan_coordinator`, its existing
text/flash acting as a trivial presenter). Rejected: a `qr_overlay_test` app in
`esp-board-common` pulling lvgl-screens (inverts layering, duplicates screens init).
Rejected: a feature branch (work disappears).

## Three layers — each fact originates where it is known

```
camera ─▶ esp-camera-pipeline (ENGINE: dataflow, hardware-agnostic)
            ├─▶ display sink  ─▶ injected display_driver (esp-board-common) ─▶ panel
            └─▶ QR consumer   ─▶ per-frame outcome callback
                                   │
                                   ▼
                 scan_coordinator (esp-board-common: DOMAIN orchestration)
                   classify (injected) · state-change dedup · percent · on_complete
                                   │
                                   ▼
                 presenter (injected by the app: UI)
                   qr_overlay_test → camera_preview_overlay (LVGL)
                   qr_decoder      → decode-flash text
                   (future)        → text-LCD renderer, etc.
```

- **Engine** knows pixels / PPA / ring buffers / quirc. Knows decode-stage facts only.
- **Session** knows scan-domain concepts (new/repeat/complete, progress, scanning).
  Lives above the engine on **altitude** grounds (it speaks one consumer's domain
  vocabulary; the engine is deliberately app-agnostic and separately reusable, with its
  own CI). Mirrors production: `DecodeQR` + `ScanView` live in the seedsigner app, not
  in the camera driver.
- **Presenter** is UI; the session is UI-agnostic so any renderer (LVGL overlay,
  text-LCD, Pi Zero) consumes the same neutral scan state.

## Engine change — `esp-camera-pipeline` / `cam_pipeline_qr`

Replace the success-only `on_decoded` with a **unified per-frame outcome callback**
(breaking change — acceptable, pre-release). Fired **once per processed frame**:

```c
typedef enum {
    CAM_QR_FRAME_NOTHING = 0,  // no code located
    CAM_QR_FRAME_MISS,         // located (corners valid) but decode failed
    CAM_QR_FRAME_DECODED,      // decoded — payload/meta valid
} cam_qr_frame_outcome_t;

// payload/meta valid iff DECODED.
typedef void (*cam_qr_frame_cb_t)(cam_qr_frame_outcome_t outcome,
                                  const uint8_t *payload, size_t len,
                                  const k_quirc_data_t *meta, void *user_ctx);
```

- Per-frame rule (applied after the decode loop): `DECODED` if any code decoded
  (deliver the first decoded code's payload — the loop may break on first success);
  else `MISS` if ≥1 code was located; else `NOTHING`. Multiple QRs in one frame is not
  a scanning scenario; a count/array can extend this later without breaking the path.
- The identify/miss detection currently exists only under `CONFIG_CAM_PIPELINE_QR_DEBUG`
  (a `QRMISS` log + `missed_codes` counter). **Lift it out of the debug guard** — it is
  nearly free (`k_quirc_count()` is always computed; the corners-on-failure fix lets a
  located-but-undecoded code be measured).
- Fires on **every** frame, including `NOTHING`, so a consumer can drop the indicator
  when the QR leaves view. The engine does NOT dedup (see below).

## Coordinator — `esp-board-common/src/scan_coordinator.{c,h}`

Named `scan_coordinator` (not "session" — it coordinates engine → classifier →
presenter and owns lifecycle, rather than managing episode state; "session" over-indexed
on state). Distinct from the builder's `camera_manager` (lifecycle + ring buffer +
`qr_poll`) and seedsigner's MVC `Controller`.


- Takes an **injected** `cam_pipeline_config_t` (built by the app via
  `board_pipeline_default_config`), so the session itself is board-agnostic and lives
  here only for altitude/sharing reasons.
- Owns: pipeline + QR-consumer lifecycle, the per-frame dispatch, debug-stats exposure
  for the HUD, and the state-change dedup.
- Injected by the app:
  - **classify** — `scan_classify_fn(payload, len) -> {NEW, REPEAT, COMPLETE}`, with an
    out `percent`. Mirrors Python `DecodeQR.add_data` (`PART_COMPLETE`/`PART_EXISTING`/
    `COMPLETE`) + `get_percent_complete`.
  - **present** — `scan_present_fn(percent, scan_frame_status_t)`. UI-agnostic. (The
    decoded content is not shown during scanning; the assembled result arrives via
    on_complete, so present needs no payload.)
  - **on_complete** — `scan_complete_fn()`. Fired once, terminal. The app reacts with
    teardown + handoff; it pulls the assembled result from its **own** classifier object
    (mirrors `ScanView` calling `decoder.get_psbt()` etc. after `is_complete`).

### Neutral domain vocabulary (no renderer dependency)

```c
typedef enum {                 // mirrors Python DecodeQRStatus; NOT the overlay enum
    SCAN_FRAME_NONE = 0,       // no recent decode
    SCAN_FRAME_NEW,            // new part      (Python PART_COMPLETE)
    SCAN_FRAME_REPEAT,         // already-seen  (Python PART_EXISTING)
    SCAN_FRAME_MISS,           // located, not decoded (from engine MISS)
} scan_frame_status_t;
```

- `present` receives the four states above. `COMPLETE` is NOT a dot state — it routes to
  `on_complete` (a terminal control-flow event, not a render).
- `NEW/REPEAT/COMPLETE` come from the classifier (domain); `MISS/NONE` come straight
  from the engine outcome. So `scan_frame_status_t` defines its own vocabulary and does
  not depend on the overlay's `camera_overlay_frame_status_t`.

### State-change dedup — in the session, after classification (correctness, not just perf)

The engine fires every frame; the session pushes to the presenter only when the overlay
state tuple `(frame_status, percent)` **changes**. This MUST live here, not in the
engine: NEW→REPEAT is a real state change (green dot → gray dot) but is invisible at the
engine level (same `DECODED` outcome both frames). The new-vs-repeat decision is a
classifier (domain) decision; engine-level dedup would collapse it and hide the
green→gray transition. Distinct NEW parts each change `percent` → each pushes; repeats
collapse to one push then silence; `NOTHING` pushes once when the QR leaves view.

## Presenter mapping (per renderer; the only place a renderer's enum appears)

`qr_overlay_test` maps the neutral status to the overlay's visual enum:
`NEW→ADDED` (green), `REPEAT→REPEATED` (gray), `MISS/NONE→NONE` (hidden), and calls
`camera_preview_overlay_set_progress(percent, dot)` / `set_scanning`. A future text-LCD
renderer maps the same neutral status to text. **The overlay's input contract needs no
change** — keep its imperative setters; the mapping seam already buys renderer
independence. (Reconsider a declarative `update(spec)` only if a 3rd/4th renderer's
setter duplication actually bites.)

## v1 test classifier (in `qr_overlay_test` only)

Payload-dedup: `strcmp` against the previous payload → new payload = `NEW` + advance a
synthetic percent (`distinct_seen / expected_parts`, where `expected_parts` comes from a
known test-GIF part count); same payload = `REPEAT`, no advance; reaching the target =
`COMPLETE`. A continuous animated stream naturally exercises both dots + the bar + the
completion transition. No real reassembly — this is a rendering check, not a decoder.

## Production trajectory (the session is the invariant)

| Phase | Classifier injected | Session code |
|---|---|---|
| v1 (this test) | payload-dedup + synthetic percent | — |
| Phase 2 POC | shim over `camera_manager` `qr_poll()` | unchanged |
| Production | Python `DecodeQR.add_data` / `get_percent_complete` | unchanged |

Phase 2's `camera_manager` (in the builder) reuses the same lifecycle core, swapping the
push callback for a non-blocking ring buffer + `qr_poll` (MicroPython thread constraint).

## Open items

- **Builder-side overlay presenter (separate/later session).** Wire
  `camera_preview_overlay` over the live preview in the micropython-builder. The builder
  already consumes `deps/seedsigner-lvgl-screens` (submodule) via
  `MICROPY_EXTRA_COMPONENT_DIRS=…/components`, so the dependency path is solved there;
  what's new is composing camera preview + `scan_coordinator` + the overlay presenter, plus
  the seedsigner screens init (`set_display`/profile/fonts). Pairs with Phase 2.
- The engine change touches `esp-camera-pipeline` (its public consumer contract) — small
  but deliberate; it is the one repo with CI, so update tests alongside. Note the engine
  lives here as a submodule and is also vendored in the builder's
  `deps/esp-camera-pipeline` — keep both in sync.
