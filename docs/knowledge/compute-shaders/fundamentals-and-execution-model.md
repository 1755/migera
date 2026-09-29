---
title: Compute shader fundamentals & execution model
description: Explains the invocation/subgroup/workgroup/dispatch hierarchy, hardware limits to query rather than hardcode, WGSL compute builtins, workgroup shared memory and barriers, and indirect dispatch. Read before writing or sizing any WGSL compute kernel.
type: concept
status: current
tags:
  - gpu-compute
  - wgsl
  - performance
updated: 2026-08-23
sources:
  - https://docs.vulkan.org/tutorial/latest/Advanced_Vulkan_Compute/02_Compute_Architecture/
  - https://docs.vulkan.org/guide/latest/compute_shaders.html
  - wgpu docs
aliases:
  - workgroup
  - subgroup
  - warp
  - wavefront
  - LDS
  - var<workgroup>
  - indirect dispatch
---

# Compute shader fundamentals & execution model

Sources: Vulkan docs compute architecture chapter
(https://docs.vulkan.org/tutorial/latest/Advanced_Vulkan_Compute/02_Compute_Architecture/,
https://docs.vulkan.org/guide/latest/compute_shaders.html), wgpu docs.

## The hierarchy

```
dispatch grid (global)          - dispatch_workgroups(x,y,z)
  +-- workgroup (local)         - @workgroup_size(x,y,z); the unit YOU define
       +-- subgroups            - warps (NVIDIA 32) / wavefronts (AMD GCN 64, RDNA 32)
            +-- invocations     - one thread; @builtin(global_invocation_id)
```

- **Invocation**: one thread/lane of work.
- **Subgroup** (wave/warp): invocations executed in lockstep on one SIMD. This is the
  granularity at which divergence hurts (see performance doc).
- **Workgroup**: the smallest unit *you* control. **A workgroup never splits across
  compute units (CU/SM)** - all its invocations execute on the same hardware block.
  This is why members can share Local Data Share (LDS / `var<workgroup>`) memory, and
  why oversized workgroups wreck occupancy or fail to schedule at all.
- **Dispatch**: repeats the workgroup over a global 1D/2D/3D grid. Per-axis count limit
  `maxComputeWorkGroupCount` (often >=65535; newer hardware effectively unlimited).

## Hardware limits (query, don't hardcode)

| Limit | Typical value |
|---|---|
| `maxComputeWorkGroupSize` per axis | 1024 |
| `maxComputeWorkGroupInvocations` (x*y*z product) | 1024 desktop; as low as 128 mobile |
| `maxComputeSharedMemorySize` | 32 KiB |
| subgroup size | 4-128 depending on vendor/architecture |

Mobile GPUs (Mali/Adreno) vary most. Query `wgpu::Limits` at runtime rather than
assuming desktop values.

## WGSL builtins & entry point shape

```wgsl
@compute @workgroup_size(8, 8, 1)
fn main(
    @builtin(global_invocation_id) gid: vec3<u32>,   // unique across whole dispatch
    @builtin(local_invocation_id)  lid: vec3<u32>,   // within workgroup
    @builtin(local_invocation_index) li: u32,        // flattened index in workgroup
    @builtin(workgroup_id)         wid: vec3<u32>,   // which workgroup am I
    @builtin(num_workgroups)       nw:  vec3<u32>,
) {
    if (gid.x >= width || gid.y >= height) { return; }   // bounds check REQUIRED when
    // ...                                              // dispatch rounds up
}
```

Dispatch counts are computed by **rounding up**: `(width + 7) / 8` for an 8-wide
workgroup. Never assume texture/buffer dimensions divide evenly.

## Shared memory (`var<workgroup>`, LDS)

"Essentially the L1 cache you can control." Uses:

- Loading common data once per workgroup instead of once per thread.
- Staging neighbor data for stencil/convolution patterns.
- Reductions (sum/min/max via tree reduction + barrier).

Rules:

- Cross-thread access needs `workgroupBarrier()` after writes before reads. Weak
  (non-coherent) stores additionally need atomics, or exactly-one-writer-per-location
  between barriers.
- LDS is allocated for the *whole workgroup* up front and released only when the last
  invocation finishes - heavy LDS use caps how many workgroups share a CU (occupancy).
- A barrier costs nothing if the whole workgroup fits in one wave (the compiler removes
  it); it costs real synchronization once the group spans waves.

## Indirect dispatch

`dispatch_workgroups_indirect` reads dispatch counts from a GPU buffer, letting compute
itself decide how much work exists (e.g. a compaction pass writes the count). Essential
for GPU-driven pipelines; avoids CPU round-trip to size the next dispatch.

## Compute vs render passes

A compute pass has no fixed-function rasterization: no blending, no viewport, no
interpolation, no automatic framebuffer compression. What it gains: arbitrary buffer
read/write (scatter), shared memory, per-thread control flow independent of pixel
coverage, and multiple logically-distinct passes in one encoder without intermediate
render targets. Arm's guidance: prefer fragment shading for simple image processing;
reach for compute when structure demands it (see performance doc).

## Related
- [Compute shader performance: best practices & anti-patterns](./performance-best-practices.md) — deeper: occupancy, divergence and workgroup-size rules that follow from this model.
- [Compute shaders in Bevy 0.19](./bevy-integration.md) — applies: how a kernel written here gets a pipeline, bind groups and a place in the schedule.
- [CPU<->GPU data flow](./cpu-gpu-data-flow.md) — deeper: feeding and reading back buffers without stalling.
- [Raymarching via compute](./raymarching-via-compute.md) — applies: this model applied to a sphere tracer.
