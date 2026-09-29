---
title: Compute Shaders — Knowledge Base
description: GPU compute for migera - the execution model, performance rules, CPU<->GPU data flow and profiling, wiring compute passes into Bevy 0.19, and what compute buys a sphere tracer. Read before designing any GPU-driven pass, choosing workgroup sizes, or adding readback.
type: index
status: current
tags:
  - gpu-compute
  - wgsl
  - bevy
  - performance
updated: 2026-09-28
---

# Compute Shaders — Knowledge Base

Research-grounded notes on GPU compute: how the hardware executes it, how to get
efficiency out of it, how to move data between CPU and GPU without stalling, and how to
wire compute passes into Bevy 0.19's render-graph-as-systems schedule (the old
`Node`-based graph is superseded). The original motivation was moving migera's
fullscreen-fragment marcher (`assets/shaders/raymarch.wgsl`) into compute; the
`src/hybrid` rewrite now traces in compute (`hybrid_trace.wgsl`), and the GPU physics
passes (`physics_*.wgsl`) use the same model.

## Start here

Read [fundamentals](./fundamentals-and-execution-model.md) →
[performance](./performance-best-practices.md) →
[Bevy integration](./bevy-integration.md). Open
[data flow](./cpu-gpu-data-flow.md) before any readback or timing work.

## Key facts

1. **A workgroup never splits across compute units** — sizing, shared memory and barrier cost all follow from it ([fundamentals](./fundamentals-and-execution-model.md)).
2. **GPUs hide latency with occupancy**, capped by registers, LDS and group size; register spills are a silent slowdown ([performance](./performance-best-practices.md)).
3. **Compute is not inherently faster than fragment shading for pixel work** — naive compute tiling measured ~2x slower; wins come from structure ([raymarching via compute](./raymarching-via-compute.md)).
4. **Single-wave (64-thread) groups suit sphere tracing** (Claybook finding) ([performance](./performance-best-practices.md)).
5. **Never block the CPU on GPU results mid-frame** — readback is async, double-buffered, a frame late ([data flow](./cpu-gpu-data-flow.md)).
6. **Per-pass GPU time comes from timestamp queries** via `RenderDiagnosticsPlugin` + `pass_span`, as wired in `src/hybrid/pass.rs` ([data flow](./cpu-gpu-data-flow.md)).
7. **In Bevy 0.19, compute passes are render-world systems** ordered around `camera_driver` or in `Core3d` sets, not graph `Node`s ([Bevy integration](./bevy-integration.md)).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Compute shader fundamentals & execution model](./fundamentals-and-execution-model.md) | Invocation/subgroup/workgroup/dispatch hierarchy, limits to query, WGSL builtins, shared memory, barriers, indirect dispatch | before writing a WGSL compute kernel |
| [Compute shader performance: best practices & anti-patterns](./performance-best-practices.md) | Occupancy, coalescing, divergence, workgroup-size rules, anti-patterns | a compute pass is slow, or before picking workgroup sizes |
| [CPU<->GPU data flow - uploads, readback, profiling](./cpu-gpu-data-flow.md) | Upload paths, async double-buffered readback, timestamp-query profiling as wired in `src/hybrid` | before adding readback, or when measuring GPU pass time |
| [Compute shaders in Bevy 0.19](./bevy-integration.md) | Schedule placement, pipelines, bind groups, storage textures, displaying compute output | wiring a new compute pass into Bevy |
| [Raymarching via compute](./raymarching-via-compute.md) | What compute buys a sphere tracer; 3-phase plan and which phases `src/hybrid` built | before restructuring the hybrid trace dispatch |

## See also

- [The render graph is gone: render-graph-as-systems](../bevy-rendering/architecture/render-graph-as-systems.md) — the Bevy schedule model compute passes live in.
- [Hybrid renderer architecture](../hybrid-architecture/INDEX.md) — how `src/hybrid` actually structures its compute passes.
- [Where AABB work belongs: CPU or compute shader](../aabb-acceleration/cpu-vs-gpu-placement.md) — placement rules for intersection/culling work.
- [Performance and Production (SDF)](../sdf-3d/performance-and-production/INDEX.md) — SDF-specific cost model and production engines.
