---
title: "Ray tracing Gaussians: an alternative to rasterization"
description: Describes 3DGRT (as of 2025-2026), which wraps each Gaussian in an icosahedral proxy under a hardware-RT BVH to get per-ray sorting, shadows, reflections, non-pinhole cameras and mixed mesh+splat tracing, at a cost still above rasterization. Read before needing secondary rays on splat content.
type: research
status: current
tags:
  - 3dgs
  - ray-tracing
  - spatial-acceleration
  - performance
  - state-of-the-art
updated: 2026-08-15
sources:
  - Moenne-Loccoz et al., "3D Gaussian Ray Tracing", SIGGRAPH Asia 2024
  - "GRay: Ray Tracing 3D Gaussians Near the Speed of Splats"
aliases:
  - 3DGRT
  - OptiX
  - icosahedral proxy
---

# Ray tracing Gaussians: an alternative to rasterization

## Why rasterization alone isn't always enough

The [tile-based rasterizer](./tile-based-rasterizer.md) is fast and well-suited to
straightforward primary-view rendering, but rasterization pipelines fundamentally handle
certain effects poorly or not at all: physically correct shadows, reflections and
refraction, and non-pinhole camera models (fisheye lenses, orthographic projections with
specific distortions, or other non-standard camera geometries) generally require
workarounds when built on top of a rasterization pipeline rather than falling out
naturally the way they do in a ray-tracing pipeline, where each ray can be traced
independently with arbitrary origin/direction and can spawn secondary rays for
reflection/shadow queries.

## 3D Gaussian Ray Tracing (3DGRT)

3D Gaussian Ray Tracing addresses this by tracing rays directly against a Gaussian-splat
scene using **hardware-accelerated ray tracing** (via NVIDIA OptiX), rather than
rasterizing. Since dedicated ray-tracing hardware (RT cores) is built around
triangle/BVH intersection — not native Gaussian primitives — 3DGRT approximates each
Gaussian with a small **icosahedral mesh proxy** and builds a scene-level BVH over these
proxies, letting hardware ray-triangle intersection do the heavy lifting while the actual
per-hit shading/compositing still uses the true Gaussian density function.

This unlocks capabilities rasterization-based 3DGS structurally cannot provide without
significant additional engineering:

- **Per-ray Gaussian sorting** — rather than a shared per-tile sort (see
  [tile-based-rasterizer](./tile-based-rasterizer.md)), each ray can determine its own
  correct depth ordering of intersected Gaussians independently, which matters for
  effects requiring genuinely per-ray-accurate compositing.
- **Arbitrary camera models** — fisheye, orthographic, or other non-pinhole projections
  are natural for ray tracing (just change how rays are generated) but require special-
  cased handling in a rasterization pipeline built around a standard perspective
  projection matrix.
- **Integration with mesh-based path tracing** — because the underlying acceleration
  structure is a BVH (the same data structure conventional triangle path tracers use),
  Gaussian-splat scenes and traditional mesh geometry can be traced together in a single
  unified pipeline, enabling mixed real-mesh-plus-splat scenes with consistent shadows
  and reflections across both representations — a capability directly relevant to game-
  engine and VFX integration scenarios (see
  [game-engine-integration](../state-of-the-art/game-engine-integration.md)).

## The cost tradeoff

Ray tracing Gaussians is **not free** relative to rasterization — the fundamental cost
driver is that sorting *all* Gaussians intersecting *every individual camera ray* is
substantially more expensive than the rasterizer's shared per-tile sort, since there's no
equivalent amortization across neighboring pixels the way tile-sharing provides. Ray-
traced 3DGS implementations are consistently reported as slower than their rasterization-
based counterparts for equivalent scenes, though recent work (e.g. "GRay: Ray Tracing 3D
Gaussians Near the Speed of Splats") specifically targets closing this performance gap —
indicating this remains an active area of optimization rather than a solved, closed
problem.

## When ray tracing is worth the cost

The practical decision is a straightforward cost/capability tradeoff:

| Need | Approach |
|---|---|
| Fastest possible primary-view rendering, standard pinhole camera, no secondary rays needed | Rasterization (the tile-based rasterizer) |
| Physically-correct shadows/reflections, non-pinhole cameras, or mixed mesh+splat scenes with consistent lighting | Ray tracing (3DGRT and descendants), accepting a real performance cost |
| Real-time interactive viewing on constrained hardware | Rasterization — ray tracing's overhead is generally too costly for this use case as of 2025-2026 |
| Offline/near-real-time rendering where visual correctness matters more than absolute frame rate | Ray tracing becomes increasingly attractive as the performance gap narrows |

## When to dive in

- Building a rendering pipeline that needs shadows, reflections, or non-pinhole cameras
  for Gaussian-splat content → 3DGRT-family ray tracing is the current answer;
  rasterization-only approaches will require significant custom engineering to
  approximate these effects, if it's possible at all.
- Integrating splat content into a mixed mesh+splat scene requiring consistent lighting
  across both representations → the BVH-based unification 3DGRT provides is directly
  relevant; see
  [game-engine-integration](../state-of-the-art/game-engine-integration.md) for current
  production tooling status.
- Prioritizing raw frame rate for pure novel-view-synthesis playback with no secondary
  lighting effects needed → rasterization remains the faster, simpler, more mature
  choice; don't reach for ray tracing by default.

## Related

- [The tile-based rasterizer](./tile-based-rasterizer.md) — contrast: the faster shared per-tile sort this replaces.
- [Game engine integration and production status](../state-of-the-art/game-engine-integration.md) — applies: mixed mesh+splat production tooling.
- [Hierarchical volume structures](../../hierarchical-volumes/INDEX.md) — prerequisite: BVH construction and traversal behind the proxy acceleration structure.
