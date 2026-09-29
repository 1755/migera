---
title: CPU<->GPU data flow - uploads, readback, profiling
description: Establishes the correct wgpu/Bevy paths for per-frame uploads, always-async GPU readback (double-buffered staging), and trustworthy per-pass GPU timing via timestamp queries as wired in src/hybrid. Read before adding any readback or when measuring GPU pass time.
type: reference
status: current
tags:
  - gpu-compute
  - bevy
  - performance
  - tooling
  - debugging
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/pass.rs
  - examples/gallery.rs
sources:
  - https://docs.rs/wgpu/latest/wgpu/struct.Buffer.html
  - https://toji.dev/webgpu-best-practices/buffer-uploads.html
  - gfx-rs wgpu discussion 1438
  - commit c473052
aliases:
  - map_async
  - staging buffer
  - timestamp query
  - RenderDiagnosticsPlugin
  - GPU profiling
---

# CPU<->GPU data flow - uploads, readback, profiling

Sources: wgpu Buffer docs (https://docs.rs/wgpu/latest/wgpu/struct.Buffer.html),
toji.dev WebGPU buffer-upload best practices (https://toji.dev/webgpu-best-practices/buffer-uploads.html),
gfx-rs wgpu discussion #1438 (maintainer guidance on per-frame updates), Sitepoint
WebGPU concurrency guide, Bevy's own examples (`compute_shader_game_of_life`,
GPU-readback example).

## Uploads (CPU -> GPU)

| Method | When to use |
|---|---|
| `Queue::write_buffer` / Bevy `RenderQueue::write_buffer` | **Default.** Internally managed staging belt; no blocking; correct for per-frame updates. |
| `Buffer::mapped_at_creation` + write + unmap | One-time initialization; minimal copies. |
| `wgpu::util::StagingBelt` | Squeezing last upload perf with explicit control (portable). |
| Persistent mapped buffers | Native-only feature territory; rarely worth it in wgpu. |

Rule of thumb from the wgpu maintainers: start with `write_buffer`; only move to
`StagingBelt` if profiling shows upload cost. In Bevy, `RawBufferVec<T>` /
`UninitBufferVec<T>` / `UniformBuffer<T>` already wrap the right calls - use them
instead of raw buffers (this project already does for its primitive records).

## Downloads (GPU -> CPU) - always asynchronous

Storage output needs usage `STORAGE | COPY_SRC`; the staging target needs
`MAP_READ | COPY_DST` (these flag sets cannot be merged into one buffer).

```
compute writes result buffer
encoder.copy_buffer_to_buffer(result -> staging)
queue.submit(...)                       // never block here
staging.slice(..).map_async(Read, cb)   // callback fires on device poll when safe
// in callback (or later): get_mapped_range -> read -> unmap
```

Production patterns:

- **Double-buffered readback**: read frame N-1's staging while frame N renders.
  Never `await` a map on the submit path mid-frame.
- Callbacks must be short (set a flag/channel send); they run during
  `device.poll`/render-app polling on native.
- A mapped buffer cannot be referenced by GPU commands; forgetting `unmap` before reuse
  is a validation error.
- Bevy convenience: `bevy::render::renderer::util::DownloadBuffer::read_buffer`
  wraps copy+map_async for one-shot reads (used by Bevy's own GPU-readback example).
  For per-frame data, roll the double-buffered pattern yourself.

If you find yourself wanting synchronous mid-frame results (CPU decides next dispatch),
redesign: do the decision on the GPU (indirect dispatch) or defer the decision one
frame. Serializing is the classic performance killer.

## Timestamp-query profiling (trustworthy GPU timings)

CPU timers around submit measure hand-off, not execution. The only true per-pass GPU
time:

1. Device feature `TIMESTAMP_QUERY`.
2. QuerySet(type=Timestamp) + two slots per pass (begin/end); pass descriptors take
   `timestamp_writes`. (Bevy's `RenderDiagnosticsPlugin` wraps exactly this.)
3. Resolve into a `QUERY_RESOLVE|COPY_SRC` buffer, copy into a separate
   `MAP_READ|COPY_DST` buffer (usages are mutually exclusive), read **a frame late**
   via the async pattern above.
4. Multiply tick deltas by `Queue::get_timestamp_period` for nanoseconds.

**Actually enabled in this project** (`examples/gallery.rs`, `src/hybrid/pass.rs`) —
confirmed working on this dev machine's Vulkan/RADV backend. Concretely:
- `RenderDiagnosticsPlugin` isn't auto-added by `DefaultPlugins` (Bevy only adds it
  under the `tracing-tracy` Cargo feature, not enabled here) — add it explicitly
  alongside `FrameTimeDiagnosticsPlugin`. No device-feature request needed of your
  own: Bevy's default `WgpuSettings::Functionality` priority already requests every
  feature the adapter advertises, `TIMESTAMP_QUERY` included where the backend
  (Vulkan/DX12 only — Metal/WebGPU/WebGL2 get CPU-only timing) supports it.
- In a render-world system with a `RenderContext` param, call
  `ctx.diagnostic_recorder()` (`Option<Res<DiagnosticsRecorder>>`), `.as_deref()` it
  once, then wrap each `ComputePass`/`RenderPass`/`TrackedRenderPass` region with
  `diagnostics.pass_span(&mut pass, "name")` / `span.end(&mut pass)` — a `None`
  recorder makes every call a no-op, so call sites never need `#[cfg]` guards. See
  `src/hybrid/pass.rs`'s `hybrid_pass` for a real example (one span per pass —
  `"hybrid_trace"` around the trace dispatch, `"hybrid_blit"` around the blit draw, plus
  `"hybrid_ddgi"`, `"hybrid_temporal"`, `"hybrid_denoise"`, `"hybrid_dof"` and others
  as of 2026-09-28).
- Results land in the *same* `DiagnosticsStore` `FrameTimeDiagnosticsPlugin` already
  populates, at path `render/<name>/elapsed_gpu` (and `elapsed_cpu`, always present
  even without timestamp-query support) — read with
  `DiagnosticPath::from_components(["render", "hybrid_trace", "elapsed_gpu"])` +
  `.get(&path).and_then(|d| d.smoothed())`. Treat the GPU path as `Option` — it's
  simply absent on backends without timestamp-query support, not zero.

Watch: timestamps quantized/noisy on some backends - average over ~100 frames before
acting; BigUint64 subtraction before float narrowing.

## Watchdogs & queue health

- One dispatch that runs seconds trips driver TDR/device-lost. Budget per-dispatch
  work; split huge passes across frames if needed.
- If submission outpaces retirement, latency and memory balloon. Gate on completion
  callbacks (or accept Bevy's internal pipelining) rather than queueing unboundedly.

## Decision table (from production budgeting practice)

Frame over budget? Split first by *submit-vs-pass time* (CPU-bound vs GPU-bound), then
by *compute-delta vs render-delta*. Pull only the lever the measurement indicates:
CPU-bound -> fewer/coarser uploads & submissions; compute-bound -> occupancy, register
pressure, group size; render-bound -> overdraw, LOD, bandwidth.

## Related
- [Compute shaders in Bevy 0.19](./bevy-integration.md) — prerequisite: where the passes being fed and profiled here are scheduled.
- [Compute shader performance](./performance-best-practices.md) — deeper: what to change once timestamps show a compute-bound pass.
- [GPU buffer types and allocation strategies](../bevy-rendering/resources-and-assets/gpu-buffers-and-allocation.md) — deeper: Bevy's `RawBufferVec`/`UniformBuffer` wrappers recommended above.
- [Trace-pass bottleneck is not march steps](../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — example: a finding made with the `hybrid_trace` `elapsed_gpu` timestamps described here.
- [Where AABB work belongs: CPU or compute shader](../aabb-acceleration/cpu-vs-gpu-placement.md) — applies: the "never read back mid-frame" rule decides where culling/intersection work runs.
