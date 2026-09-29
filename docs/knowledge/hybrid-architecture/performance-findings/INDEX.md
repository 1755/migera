---
title: Hybrid renderer performance findings
description: Measured performance results for src/hybrid, mostly null results — march step count is not the trace-pass bottleneck, render scale already covers half-res reflections, and a skybox bake would not help because scene misses are nearly free. Read before any optimization of the trace pass.
type: index
status: current
tags:
  - performance
  - hybrid-renderer
  - raymarching
updated: 2026-09-28
---

# Hybrid renderer performance findings

Measured on the dev machine's integrated AMD Renoir GPU with the `--stress N`
scenes and `gpu trace` timestamps. The pattern across all three: an optimization
that is standard practice elsewhere, whose premise did not hold here. The real
dominant cost (DDGI's per-probe occlusion rays, ~48–52% of the frame at
`--stress 10000`) is in [DDGI any-hit occlusion](../gi-and-lighting/ddgi-any-hit-occlusion.md).

| Note | What it establishes | Read when |
|---|---|---|
| [Trace-pass bottleneck is not march steps](./trace-pass-bottleneck-is-not-march-steps.md) | Cutting `MAX_MARCH_STEPS` 128 → 32 left `gpu trace` at ~137–143 ms; step-count optimizations cannot help. | Before optimizing march step count. |
| [Render scale subsumes half-res reflections](./render-scale-subsumes-half-res-reflections.md) | `RenderScaleConfig` already gives reflect/transmit passes ~3.2–3.5x at scale 0.5; no separate tier built. | Before adding a per-pass resolution knob. |
| [Skybox / far-object bake not worth it](./skybox-far-object-bake-not-worth-it.md) | Scene misses cost ~8–9 ms vs ~133–150 ms for hits; a far-object bake risks the miss-is-black leak class. | Before proposing a skybox, impostor or far-field cache for speed. |

## See also
- [DDGI any-hit occlusion](../gi-and-lighting/ddgi-any-hit-occlusion.md) — where the trace-pass time actually goes.
- [Analytical SDF gradients plan](../plans/analytical-sdf-gradients.md) — another measured null result: analytic normals.
- [Performance best practices](../../compute-shaders/performance-best-practices.md) — general compute-shader performance guidance.
