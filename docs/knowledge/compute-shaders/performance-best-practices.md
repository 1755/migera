---
title: "Compute shader performance: best practices & anti-patterns"
description: Establishes that compute throughput comes from latency hiding (occupancy capped by registers, LDS, group size), coalescing and low divergence; gives workgroup-size rules (single-wave groups for sphere tracing) and an anti-pattern list. Read when a compute pass is slow or before picking workgroup sizes.
type: research
status: current
tags:
  - gpu-compute
  - performance
  - raymarching
  - state-of-the-art
updated: 2026-08-23
sources:
  - https://developer.nvidia.com/blog/advanced-api-performance-shaders/
  - https://developer.nvidia.com/blog/optimizing-compute-shaders-for-l2-locality-using-thread-group-id-swizzling/
  - https://gpuopen.com/learn/occupancy-explained/
  - https://gpuopen.com/learn/optimizing-gpu-occupancy-resource-usage-large-thread-groups/
  - https://computergraphics.stackexchange.com/questions/9956/
  - http://evasion.imag.fr/~Fabrice.Neyret/Etudiants/rapports/rapportM1-2018_Danh-Huynh.pdf
aliases:
  - occupancy
  - register spilling
  - VGPR
  - wave divergence
  - thread-group swizzling
  - memory coalescing
---

# Compute shader performance: best practices & anti-patterns

Sources: NVIDIA Advanced API Performance - Shaders
(https://developer.nvidia.com/blog/advanced-api-performance-shaders/), NVIDIA thread-group
swizzling (https://developer.nvidia.com/blog/optimizing-compute-shaders-for-l2-locality-using-thread-group-id-swizzling/),
AMD GPUOpen "Occupancy explained" + "Optimizing GPU occupancy and resource usage with
large thread groups" (https://gpuopen.com/learn/occupancy-explained/,
https://gpuopen.com/learn/optimizing-gpu-occupancy-resource-usage-large-thread-groups/),
Vulkan docs occupancy chapter, Arm GPU Best Practices, OpenGL Programming Guide ch.12,
and the deferred-shading compute-vs-fragment study
(https://computergraphics.stackexchange.com/questions/9956/).

## The mental model: latency hiding, not raw speed

GPUs are memory-latency-bound: a VRAM fetch stalls a wave for hundreds of cycles. A CU
survives this by switching between many resident waves ("occupancy"). Occupancy is capped by:

1. **Registers (VGPRs)** - e.g. 128 regs/thread leaves room for far fewer threads than
   32 regs/thread. Over the limit, the compiler **spills registers to memory** - silent,
   brutal slowdown.
2. **LDS** - allocated per workgroup; 32 KiB/group on a 64 KiB CU = max 2 groups.
3. **Thread slots & barriers** - large multi-wave workgroups hold resources until their
   *last* wave finishes; waves that finish early cannot be replaced until the whole
   group retires.

Higher theoretical occupancy is NOT automatically faster (cache effects can make fewer,
fatter waves win). The real rule: profile, and ensure you're not spilling.

## Workgroup size selection

- Multiples of the native wave size; non-multiples waste lanes (internal fragmentation).
- NVIDIA prefers multiples of 32, AMD GCN of 64 (RDNA also 32), Intel variable.
- No LDS needed: **64-256 threads** is the sweet spot; AMD recommends 256 as default.
- Fullscreen 2D passes: 8x8 or 16x16 are proven starting points (NVIDIA guidance).
- **Sphere tracing / wildly-varying loop counts: single-wave (64-thread) groups win** -
  AMD's own Claybook finding. Resources free as soon as the wave finishes; the compiler
  elides all memory barriers within one wave; no lockstep stragglers holding the group.
  Directly applicable to migera's marcher.
- If most invocations early-out, prefer larger groups (fewer, cheaper launches).

## Memory

- **Coalescing**: adjacent lanes must touch adjacent addresses. For image writes, pack
  threads so a quad spans contiguous cache lines (x-major for linear tiling).
- Load shared/reused data through LDS once per group rather than per thread.
- Prefer `texture*()` over `imageLoad()` for read-only texture data where available
  (uses texture unit; load/store units stay free for real work).
- **Thread-group ID swizzling** for fullscreen passes with wide data footprints:
  remap group IDs (horizontal tiling N=16 or Morton) so concurrently-running groups
  share L2 locality. Measured +47% on Battlefield V's denoiser (L2 hit 63%->86%).

## Divergence - the raymarcher's core problem

A wave executes in lockstep; any lane still marching forces all lanes to wait. Measured
on ray-marchers (Grenoble M1 study,
http://evasion.imag.fr/~Fabrice.Neyret/Etudiants/rapports/rapportM1-2018_Danh-Huynh.pdf):
~80% of intersection tests are spent on ~20% of pixels (silhouettes/grazing angles), and
one slow pixel delays its whole wave *and block*. Consequences & mitigations:

- Minimize worst-case step count, not average.
- Keep per-step cost uniform (branchless SDF evaluation beats data-dependent branches
  when the branch would diverge anyway).
- Consider decoupling rays from pixels (ray queues / packet tracing) so long rays don't
  pin short-ray waves; persistent-thread tile queues beat naive tiling (a controlled
  study measured a tile-queue compute shader at 1.5x fragment-shader speed).
- Sort/bucket work by expected cost when cheaply possible (e.g. by depth tile).

## Anti-pattern list (things to avoid)

- Blocking waits between dispatches in a frame; serial CPU readback per dispatch
  (serial-await anti-pattern) - starves the GPU (~60% idle measured in WebGPU studies).
- Register-spilling "fat" shaders; giant local arrays indexed dynamically.
- Oversized workgroups (1024) without LDS justification.
- Barriers inside divergent control flow (UB/hangs).
- Processing a fragment pass's output in compute mid-frame (backward dependency bubble);
  prefer keeping a producer-consumer chain in one stage type per hop.
- Hardcoded workgroup sizes / limits instead of querying device limits.
- Assuming compute will beat fragment for plain per-pixel shading: a controlled study
  found naive compute tiling ~2x slower than fragment due to pixel-ordering heuristics;
  only structured approaches (tile queues, shared-memory reuse) surpassed it.
- Unbounded single dispatches near driver watchdog timeouts (device-lost risk).

## Related
- [Compute shader fundamentals & execution model](./fundamentals-and-execution-model.md) — prerequisite: waves, workgroups and LDS as used above.
- [Raymarching via compute](./raymarching-via-compute.md) — applies: the divergence and single-wave-group advice turned into a phased plan for migera's marcher.
- [Production case studies: Dreams and Claybook](../sdf-3d/performance-and-production/production-case-studies.md) — deeper: the Claybook source of the single-wave-group recommendation.
- [Performance characteristics of SDF rendering](../sdf-3d/performance-and-production/performance-characteristics.md) — deeper: where SDF march cost goes, complementing the GPU-side view here.
- [DDGI any-hit occlusion](../hybrid-architecture/gi-and-lighting/ddgi-any-hit-occlusion.md) — example: a correct early-out that bought zero time, attributed to wavefront divergence.
- [Trace-pass bottleneck is not march steps](../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — example: cutting average step counts did not move `hybrid_trace` time; measure before optimizing.
- [CPU<->GPU data flow](./cpu-gpu-data-flow.md) — deeper: timestamp profiling to confirm which lever to pull.
