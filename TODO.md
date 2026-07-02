# TODO

## CI / GitHub Actions
- **No GitHub Actions CI in this repo yet.** Add a workflow to catch build
  regressions:
  - build the apps (`qr_decoder`, `touch_test`, `camera_capture`) via the Docker
    ESP-IDF image across the supported boards,
  - consider a `clang-format` gate for the C sources (the `k_quirc` submodule
    already has one).
