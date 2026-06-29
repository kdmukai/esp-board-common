# Camera focus: procedure & strategy (Waveshare ESP32-P4 OV5647)

Focus turned out to be the **single biggest decode lever** for QR scanning on
this board — bigger than output-square size or exposure. The bundled camera ships
fixed at ~1 ft; QR scanning wants ~7 in. Manually refocusing took the animated-UR
decode rate from ~2–3/s to ~5/s. This documents the hardware, the refocus
procedure, and the calibration strategy so it can be repeated — and eventually
turned into human-facing instructions.

> Candidate to promote into the cross-project hardware KB
> (`hardware-kb/waveshare/esp32-p4-touch-lcd-43/`) since it's board/camera
> knowledge reusable beyond this app.

## The camera hardware
- The "with camera" option for the ESP32-P4-WIFI6-Touch-LCD-4.3 is a **small
  flex-PCB OV5647 module** (Raspberry-Pi-Camera-v1 style: sensor + onboard
  crystal/regulators), with a **threaded, notched lens barrel**.
- The lens is **manually focusable** (rotate the barrel) but **factory
  thread-locked** at ~1 ft — what looks like "glued" is the thread-lock.
- **No VCM / focus motor.** `CONFIG_CAMERA_OV5647_ISP_AF_ENABLE` /
  `V4L2_CID_FOCUS_ABSOLUTE` are moot here — **focus cannot be set in software.**
  It is a one-time *physical* adjustment.

## Refocus procedure
- **Direction:** **counterclockwise (viewed from the front) = focus closer.** The
  lens extends away from the sensor (standard right-hand thread). It's only a
  *fraction* of a turn from ~1 ft to ~7 in. The first stiffness is the
  thread-lock breaking, not the end of travel.
- **Grip (this matters):** the notches are tiny.
  - **Fingernails** in the notches work for a coarse turn.
  - For control: a **blob of hot glue on a stick/dowel** pressed into the notches
    (let set, twist, pop off after), or a **3D-printed focus tool** that seats in
    the notches, or a pencil **eraser**/soft rubber for friction.
  - **Do NOT use tweezers** — they point-load and skate off, gouging the plastic
    housing (learned the hard way).
- **Stop conditions:** don't keep unscrewing — the lens can come fully out (loses
  thread register, invites dust). If CCW makes a *close* target blurrier, reverse
  (rare reverse-threaded lenses exist).

## Live-focus method (use the instrumentation as a sharpness meter)
1. Flash a debug build (largest square = most sensitive readout) and run the
   preview with the target at the intended working distance.
2. **Stabilize exposure first** (bright) so the image isn't hunting while you
   judge sharpness — AE oscillation makes focus hard to read.
3. Turn the lens slowly while watching the on-device HUD: peak the **`px/m`
   readout** and get the **decode flash firing steadily**. That's the sharp point.
   Lock it there. (px/m + decode flash beat eyeballing.)

## Calibration strategy (why one fixed focus is enough)
- A fixed lens has **one focus distance plus a depth of field** — a *band* of
  acceptably-sharp distances, not a point. Combined with the low decode floor
  (~2–3 px/module empirically), that's a usable working *window*.
- **You don't refocus per QR.** With fixed focus, the *user* varies distance to
  the sweet spot; a bigger/denser QR just fills more/less of the frame.
- **Sparrow renders every QR at a fixed physical size**, so one fixed focus covers
  the whole corpus; only module *density* varies px/module (denser = lower).
  **Calibrate focus so the densest in-spec QR clears the floor** at the working
  distance — everything sparser is then automatic.
- **~3.5 px/module is empirically enough** for ~5 fps animated decode (with bright
  exposure + contrast stretch); no need to chase 5 px/module.
- **Real-world caveat:** non-Sparrow sources (a phone screen, paper) aren't
  size-normalized, so production scanning reintroduces distance variation — the
  depth of field (and the user moving to the sweet spot) must cover it. A
  motorized-AF camera variant would remove this constraint, at a hardware cost.

## Future work
- **Human-facing guide:** a step-by-step user procedure (which way to turn, how
  far, what tool, how to tell it's focused) — this doc is the engineering basis.
- **Focus test targets:** a preview pattern that makes focus *visually obvious* on
  the live image — e.g. a fine **Siemens star**, concentric rings, or a
  high-frequency grid — easier to gauge than a QR. Optionally an on-screen
  **focus-assist** (live sharpness metric / focus peaking) so a user can dial it
  in without decode feedback.

  **A focus target MUST be rendered to Sparrow's on-screen scale**, or it trains
  the wrong focus. Sparrow renders a QR as large as fits within **580×580 px** at
  **integer pixels-per-module**, including a **2-module quiet zone each side**.
  For an *M*-module QR: `ppm = floor(580 / (M + 4))`, `rendered = ppm × (M + 4)`.
  Worked values for our corpus:

  | QR | modules | ppm (screen px/module) | rendered px |
  |---|---|---|---|
  | 2-of-3 (v15) | 77 | 7 | 567 |
  | UR default (v16) | 81 | 6 | 510 |
  | UR large (v17) | 85 | 6 | 534 |
  | BBQr-low (v18) | 89 | 6 | 558 |
  | 3-of-5 (v19) | 93 | 5 | 485 |
  | BBQr-max (v27) | 125 | **4** | 516 |

  Physical size is near-constant (~485–567 px) but **module pitch varies 4–7
  screen-px/module**, and the **densest in-spec QR (BBQr-max) is the finest at 4
  px/module** — the hardest to resolve. So a focus target should:
  1. Be generated within the **same 580×580 / integer-ppm / 2-module-border**
     discipline so it opens on a laptop at the same pixel size as real QRs.
  2. Carry critical detail at the **module pitch of the reference QR** — ideally
     the **densest in-spec** one (lowest ppm) so that "sharp on the target"
     guarantees the easier, higher-ppm QRs. e.g. a grid / Siemens-star whose
     finest line-pairs equal that ppm.
- **Pinned calibration value:** none possible in software (no VCM); the
  calibration *is* the physical lens position, so a **3D-printed focus knob** (see
  the separate model project) for repeatable, fine manual adjustment is the
  practical tooling.
