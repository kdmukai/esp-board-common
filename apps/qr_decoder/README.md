# QR Decoder

Continuous QR code scanner using the esp-camera-pipeline. Displays a live camera feed with decoded QR data overlaid on screen.

## What it does

- Live camera viewfinder with PPA hardware scaling (ESP32-P4) or software center-crop (ESP32-S3)
- Square-cropped display (uses the shorter display dimension for both axes)
- Continuous QR detection at ~16-22 fps via k_quirc decoder
- Decoded text appears as a green overlay and fades after 2 seconds
- FPS stats overlay (camera, display, scan, detection rates)

## Boards

| Board | Display path | Camera fps | Notes |
|---|---|---|---|
| waveshare_p4_lcd43 | LVGL image widget | ~17 | LVGL overlays (QR text + FPS stats) |
| waveshare_p4_lcd35 | dummy_draw (SPI) | ~10 | Tear-free, no LVGL overlays |
| waveshare_s3_lcd35b | LVGL image widget | untested | |
| waveshare_s3_lcd2 | LVGL image widget | untested | |

## Build and flash

Requires Docker. See `docs/dev-workflow.md` in the repo root.

```bash
# Clean build for a specific board
make clean
make docker-build BOARD=waveshare_p4_lcd43

# Flash (from build/ directory)
cd build
docker run --rm \
  --device=/dev/ttyACM0 \
  --group-add $(getent group dialout | cut -d: -f3) \
  -v $(pwd):/workspace -w /workspace \
  ghcr.io/kdmukai-bot/seedsigner-micropython-builder-base:latest \
  bash -lc 'source /opt/toolchains/esp-idf/export.sh >/dev/null 2>&1 && \
    esptool.py --chip auto -p /dev/ttyACM0 -b 460800 \
    --before default_reset --after hard_reset write_flash @flash_args'
```

**Always `make clean` before switching to a different board.**

## Architecture

```
OV5647 camera (CSI/DVP)
    |
    v
esp-camera-pipeline (triple-buffered)
    |-- PPA crop + scale (P4) or software center-crop (S3)
    |-- display driver: dummy_draw (SPI) or LVGL image widget (MIPI-DSI)
    |-- QR consumer: RGB565 -> grayscale -> k_quirc decode
    v
LVGL overlays (decoded text + FPS stats)
```

## Configuration

Key settings in `sdkconfig.defaults`:

- `CONFIG_K_QUIRC_MAX_VERSION=15` — supports QR codes up to version 15 (77x77 modules)
- `CONFIG_K_QUIRC_ADAPTIVE_THRESHOLD=y` — better detection in varied lighting
- `CONFIG_CAM_PIPELINE_DEBUG=y` — enables FPS stats overlay
