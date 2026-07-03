# CSI capture task orphaned by STREAMOFF (esp_video teardown gotcha)

**Symptom:** repeated camera start/stop in one boot leaks internal RAM until a
later cycle can no longer allocate a task stack and `start()` fails
(`scan_coordinator create failed`). Deterministic "N cycles pass, then every
cycle fails."

## Root cause
`esp_video`'s V4L2 `VIDIOC_DQBUF` blocks on a hardcoded `portMAX_DELAY`
(`esp_video_ioctl.c`, no timeout, ignores `O_NONBLOCK`), and `VIDIOC_STREAMOFF`
**deletes** the semaphore DQBUF waits on rather than signaling it
(`esp_video.c`, `vSemaphoreDelete(stream->ready_sem)`).

So a capture task parked in `DQBUF` when we `STREAMOFF` is stranded forever: it
never wakes to re-check its `running` flag, so it never reaches
`vTaskDelete(NULL)`. And it can't be reclaimed after the fact either —
force-`vTaskDelete`ing it would touch its event-list item, which now points into
the freed semaphore (heap corruption). Every stop orphaned one task, leaking its
`CSI_TASK_STACK` (~16 KB internal) per cycle.

## Fix pattern: join the capture task BEFORE STREAMOFF
Clear `running` and wait on a done-semaphore **while streaming is still on**.
Frames keep arriving, so `DQBUF` returns within ~one frame period, the loop sees
`running == false`, gives the sem, and self-deletes cleanly. Only then
`STREAMOFF` (no task is parked in `DQBUF`, so deleting the sem is harmless).
See `csi_stop()` in `src/board_pipeline_camera_csi.c`.

The general rule: **never STREAMOFF a stream that a task may be blocked reading**
with this esp_video version — drain/join the reader first.

## Meta-lesson (why the first diagnosis was wrong)
The leak was initially blamed on the QR decode-task teardown (a non-idiomatic
`vTaskSuspend(NULL)` + external `vTaskDelete`). That was a *hypothesis*, never
heap-confirmed. A per-cycle `heap_caps_get_free_size(MALLOC_CAP_INTERNAL)` (or
`esp32.idf_heap_info` from MicroPython) showed the real leak was ~16.7 KB/cycle —
the CSI capture task's 16 KB stack, not the 32 KB QR stack. **Confirm an
internal-RAM leak's size against a heap-diff before attributing it to a specific
allocation;** the "3-pass-then-fail" signature points at exhaustion, not at which
allocation is leaking.
