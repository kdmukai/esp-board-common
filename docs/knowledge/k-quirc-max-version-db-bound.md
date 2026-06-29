# k_quirc: CONFIG_K_QUIRC_MAX_VERSION must not exceed the version DB (25)

## Symptom
Heap corruption (`assert failed: block_next ... !block_is_last(block)`, TLSF)
on the **first successful QR decode**, crashing the QR decode task (Core 1). The
assert fires inside an unrelated `malloc`/`free` — the actual corruption happened
earlier, during decode.

## Root cause
`CONFIG_K_QUIRC_MAX_VERSION` was set to **27**, but k_quirc's version database
`quirc_version_db` (in `k_quirc_version.c`) is only populated **through version
25**. The array is declared `const struct quirc_version_info
quirc_version_db[QUIRC_MAX_VERSION + 1]`, so raising the constant to 27 grows the
array to 28 entries and **zero-fills indices 26 and 27** (C zero-init for missing
initializers).

The decode path then becomes internally inconsistent:
- `k_quirc_identify` accepts any grid with `version <= QUIRC_MAX_VERSION`
  (`k_quirc_identify.c`, the `version > QUIRC_MAX_VERSION` reject), so v26/v27
  grids pass.
- `quirc_extract_internal` bounds the cell bitmap with
  `max_grid_size = QUIRC_MAX_VERSION*4+17`, which also widens to accept them.
- `codestream_ecc` then reads `quirc_version_db[26|27]` = all-zero
  (`data_bytes=0`, `ecc={bs:0,dw:0,ns:0}`). The block-count / ECC-offset math
  (`k_quirc_decode.c` `codestream_ecc`) goes wrong and the RS/data reads corrupt
  the heap-allocated `struct quirc_code` / `struct quirc_data`.

A real capture of a v15–v19 QR only needs to be *mismeasured* as v26/v27
(plausible with a skewed/blurry frame at high resolution) to hit a zeroed entry.

## Rule
**`CONFIG_K_QUIRC_MAX_VERSION` ≤ 25** (the DB's populated range). 25 covers QR up
to 117 modules — i.e. 2-of-3 (v15), UR animated baseline (v17/85-mod), 3-of-5
(v19), BBQr-low (v18/89-mod).

## To support larger QR (e.g. BBQr-max, v27 / 125 modules)
Bumping the constant is **not** sufficient. The version DB must first be
**extended** with real `quirc_version_info` entries for versions 26..N (upstream
quirc has this data through v40: `data_bytes`, alignment-pattern positions,
RS-block parameters per ECC level). Only after the DB is complete to version N
can `CONFIG_K_QUIRC_MAX_VERSION` safely be raised to N.

## History
Introduced 2026-06-27 while raising the cap from the old v15 (a leftover from the
"<70 modules" QR assumption) to accommodate the real Sparrow corpus. Corrected to
25. See `docs/qr-scanning-performance-requirements.md`.
