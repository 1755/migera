---
title: Performance characteristics of SDF rendering
description: Establishes the raymarching cost model (steps × per-step evaluation, or memory bandwidth for grids), practical iteration budgets, Claybook's storage numbers, and where SDF rendering wins or loses against polygons. Read when estimating whether an SDF approach fits a budget or diagnosing a slow raymarcher.
type: concept
status: current
tags:
  - sdf
  - raymarching
  - performance
updated: 2026-08-15
aliases:
  - raymarch cost model
  - iteration count
  - memory bandwidth
---

# Performance characteristics of SDF rendering

## The fundamental cost model of raymarching

Unlike rasterization (roughly: one pipeline pass per triangle, cost proportional to
triangle count and screen coverage) or hardware ray tracing against a BVH (roughly:
`O(log n)` per ray via tree traversal), sphere tracing's cost per pixel is proportional to
**the number of raymarch steps needed to converge**, which is itself a function of:

- Distance from camera to the nearest surface along that ray (more empty space → more
  steps, though large steps in open space are cheap since each step covers more ground).
- How "flat"/predictable the SDF is locally (near-flat surfaces converge fast; grazing
  angles, thin features, and highly warped/displaced regions converge slowly — see
  [raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md)).
- The cost of a *single* SDF evaluation, which for procedural scenes scales with
  expression tree complexity (number of combined primitives and operators) — every step
  re-evaluates the *entire* combined scene function, not just nearby geometry, unless an
  acceleration structure prunes irrelevant primitives first.

This last point is the most important practical performance lesson: **a naive procedural
scene SDF evaluates every primitive at every step for every pixel**, so cost grows at
minimum linearly (often worse, depending on operator cost) with scene complexity — very
different from rasterization, where geometry outside a pixel's coverage costs nothing for
that pixel. This is why pure procedural raymarching scales poorly to large, detailed
scenes without additional acceleration (spatial partitioning of the primitive list itself,
or converting to a sampled grid representation — see
[sdf-representations](../fundamentals/sdf-representations.md)).

## Memory bandwidth for grid-based SDFs

For sampled-grid SDFs, the dominant cost shifts from arithmetic to **memory bandwidth**:
each raymarch step is a 3D texture fetch (trilinear-interpolated, so effectively 8 texel
reads combined by hardware filtering) rather than an expression evaluation. This is why
grid resolution and storage format directly trade against performance:

- **Storage scales as `resolution^3`** for a dense grid — Claybook's production World SDF
  (a real, shipped example) used a `1024 x 1024 x 512` volume at 8-bit signed precision
  with 5 mip levels, totaling 586 MB — a concrete illustration of how quickly dense-grid
  storage grows at production-quality resolution, and why mipmapping (allowing coarser
  mip levels to be sampled for distant/low-detail regions) and sparse structures (see
  [sparse-and-hierarchical-structures](./sparse-and-hierarchical-structures.md)) are not
  optional extras but load-bearing parts of any production grid-SDF system.
- **Precision**: 8-bit signed formats are common for production because full 32-bit float
  precision is rarely needed for a distance value that's primarily used to decide "is it
  safe to take a step of roughly this size" rather than for exact measurement — a clear
  precision-vs-storage tradeoff most systems resolve in favor of lower precision plus
  mipmapping over higher precision at a single resolution.

## Iteration count budgets in practice

Real-time raymarchers (60 fps target) typically budget somewhere in the range of tens to
a few hundred maximum steps per ray, tuned against scene depth complexity and acceptable
worst-case (grazing-angle, thin-feature) convergence failure rate — there is no universal
correct number; it is scene- and hardware-dependent, and the artifact/performance
tradeoff discussed in
[raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md) is
exactly the lever being tuned when adjusting this budget.

## Where SDF rendering wins vs. loses against polygon rendering

**Wins**:
- Effectively infinite geometric detail/smoothness at any zoom level for procedural
  content, with zero LOD popping and zero polygon budget management, since the surface
  is defined analytically rather than tessellated.
- Booleans (union/intersection/subtraction) and smooth blending are numerically trivial
  and always well-defined — see
  [combination-operators](../primitives-and-operators/combination-operators.md) — versus
  the well-known fragility of boolean mesh operations (self-intersections, non-manifold
  results).
- Domain repetition gives genuinely free (not "cheap," actually O(1)) infinite procedural
  instancing — see
  [domain-operations](../primitives-and-operators/domain-operations.md) — with no
  per-instance draw or storage cost.
- Physics against an SDF elegantly avoids tunneling (thin fast-moving objects passing
  through thin walls), since negative interior distances give a genuine notion of
  "how deep inside" rather than triangles being an infinitely thin shell with no interior
  concept at all — this was Sebastian Aaltonen's specific stated reason for choosing SDF
  physics over mesh-based physics in Claybook.
- Soft shadows and ambient occlusion reuse the same raymarching machinery already needed
  for visibility, at low incremental cost — see
  [soft-shadows-and-ao](../rendering/soft-shadows-and-ao.md) — versus the dedicated
  shadow-mapping and screen-space-AO passes a rasterization pipeline needs layered on top.

**Loses**:
- Per-pixel cost scales with scene complexity in a way rasterization's screen-space
  culling avoids; complex detailed scenes need either grid-baking (losing procedural
  editability and infinite resolution) or careful spatial acceleration of the primitive
  evaluation itself.
- No direct hardware acceleration path as of 2025-2026 — dedicated ray-tracing hardware
  (RT cores) is built around triangle/BVH intersection, not sphere tracing against
  arbitrary scalar functions, so SDF raymarching runs on general-purpose GPU compute
  rather than fixed-function hardware; see
  [hybrid-and-grid-rendering](../rendering/hybrid-and-grid-rendering.md).
- Thin/grazing geometry is a persistent weak point (slow convergence, potential missed
  intersections) in a way rasterization simply doesn't share, since rasterization tests
  triangle coverage directly rather than iteratively converging toward a surface.
- Sharp, precise mechanical/CAD-style edges are harder to represent and extract faithfully
  than with explicit mesh geometry — see the Marching Cubes vs. Dual Contouring tradeoff
  in [sdf-to-mesh-extraction](../mesh-conversion/sdf-to-mesh-extraction.md).

## When to dive in

- Deciding whether SDF rendering is appropriate for a project → weigh the wins/losses
  above against your specific content (organic/procedural-heavy content and physics-heavy
  simulation favor SDFs; large detailed static scenes with hard architectural edges favor
  traditional mesh pipelines, possibly with SDFs used only for indirect lighting per
  [hybrid-and-grid-rendering](../rendering/hybrid-and-grid-rendering.md)).
- A raymarcher is too slow → first check whether cost is dominated by iteration count
  (arithmetic-bound, procedural scene) or memory bandwidth (grid-sampling-bound) — the fix
  differs entirely depending on which.
- Planning storage budget for a grid-based SDF → use Claybook's numbers above as a
  reference point, and prioritize mipmapping and sparse structures over brute-force
  resolution increases.
- Deciding whether to raymarch an SDF directly every frame or bake it into a different
  representation (points, splats, mesh) once and render that repeatedly → this page's
  cost model (per-step cost × step count × pixel count, paid every frame for direct
  raymarching) is the foundation of that decision; see
  [bake-vs-direct-raymarch-efficiency](../../sdf-3dgs-bevy-integration/live-editing/bake-vs-direct-raymarch-efficiency.md)
  for the full amortization argument applied specifically to baking a CSG-composed SDF
  into a 3D Gaussian Splat cloud, including where the literature does and doesn't
  support a quantitative answer.

## Related
- [Sphere tracing](../rendering/sphere-tracing.md) — prerequisite: where the step count comes from.
- [Sparse and hierarchical structures](./sparse-and-hierarchical-structures.md) — deeper: cutting grid memory and empty-space steps.
- [Bake vs. direct raymarch efficiency](../../sdf-3dgs-bevy-integration/live-editing/bake-vs-direct-raymarch-efficiency.md) — contrast: this cost model argued toward baking into splats (archived; migera chose raymarching).
- [Trace-pass bottleneck is not march steps](../../hybrid-architecture/performance-findings/trace-pass-bottleneck-is-not-march-steps.md) — contrast: in migera's `src/hybrid`, step count was measured not to be the bottleneck.
- [Skybox/far-object bake not worth it](../../hybrid-architecture/performance-findings/skybox-far-object-bake-not-worth-it.md) — example: a scene-miss ray already costs ~O(1) in migera.
