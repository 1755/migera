---
title: 3D Signed Distance Fields
description: Domain map for SDF theory and practice — fundamentals, primitives and operators, sphere-tracing rendering (incl. migera's shadow findings), materials without UVs, mesh↔SDF conversion, production case studies, 2025-2026 research. Read when working on src/sdf, src/raymarch or SDF effects/worldgen.
type: index
status: current
tags:
  - sdf
  - raymarching
  - primitives
  - mesh-conversion
updated: 2026-09-28
---

# 3D Signed Distance Fields

Signed distance fields represent geometry as a function ("how far to the nearest
surface, signed by inside/outside") instead of stored vertices. That one idea explains
why SDFs are easy to combine (booleans are `min`/`max`), easy to render without
acceleration hardware (sphere tracing), and hard to use for large or sharp-edged
detailed scenes without extra machinery. This tree covers theory, the practical toolkit,
and production/research state, written from web research cross-checked against Inigo
Quilez's articles, shipped-game talks and 2024-2026 papers.

**Role in migera (2026-09-28):** characters moved to Bevy's PBR pipeline; SDFs remain
for effects and world generation (`src/sdf`, `src/raymarch`, `src/hybrid`). See
[migera pivots to Bevy PBR for characters](../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md).

## Start here

New to SDFs → [fundamentals](./fundamentals/INDEX.md) first; the Eikonal equation and
the exact-vs-bound distinction explain nearly every caveat elsewhere. Otherwise jump to
the topic matching your task.

## Key facts

1. An SDF is a function, not stored geometry; `|∇f| = 1` (Eikonal) is what makes its value a genuine distance — see [what-is-an-sdf](./fundamentals/what-is-an-sdf.md).
2. Booleans are trivial — `min` = union, `max` = intersection — so SDF CSG avoids mesh-CSG's non-manifold failures — see [combination-operators](./primitives-and-operators/combination-operators.md).
3. Sphere tracing is safe as long as the field never overestimates; scaling, blending and warping only give bounds, handled by step damping — see [exact-vs-bound-sdfs](./fundamentals/exact-vs-bound-sdfs.md).
4. Production rarely raymarches a whole detailed scene: Lumen uses SDFs for indirect rays, Dreams converts SDFs to points, Claybook raymarches directly — see [hybrid-and-grid-rendering](./rendering/hybrid-and-grid-rendering.md).
5. Mesh-to-SDF sign determination, not distance, is the hard part; generalized winding number is the robust default — see [sign-determination-methods](./mesh-conversion/sign-determination-methods.md).
6. Every 2D texture on an SDF goes through triplanar/biplanar projection, while the PBR BRDF itself is unchanged — see [materials-and-texturing](./materials-and-texturing/INDEX.md).
7. A baked mesh SDF is only a bound; bake resolution and step damping are load-bearing, and sub-voxel features vanish — see [glitch-free-baked-sdfs](./mesh-conversion/glitch-free-baked-sdfs.md).
8. Soft-shadow parameters must be tied to object scale, not scene extent — migera's `k=12` produced false darkening at `--stress` scale until re-swept to `k=2` — see [soft-shadows-and-ao](./rendering/soft-shadows-and-ao.md).

## Topics

| Note | What it establishes | Read when |
|---|---|---|
| [Fundamentals](./fundamentals/INDEX.md) | Eikonal equation, gradient = normal, exact vs. bound fields, the four representation families, narrow-band fields. | Always first; revisit when a note says "bound" or "narrow-band" unexplained. |
| [Primitives and Operators](./primitives-and-operators/INDEX.md) | The ~28-shape primitive set, boolean/smooth CSG, modifiers, domain repetition. | Building or extending procedural SDF content or `src/sdf/primitives.rs`. |
| [Rendering](./rendering/INDEX.md) | Sphere tracing, normals, soft shadows/AO (with migera's `src/hybrid` findings), artifact map, grid/hybrid architectures. | Implementing or debugging a raymarcher. |
| [Materials and Texturing](./materials-and-texturing/INDEX.md) | Triplanar/biplanar and noise texturing, unchanged PBR, smin-weighted material blending, authoring tradeoffs. | Texturing or shading any SDF surface. |
| [Mesh Conversion](./mesh-conversion/INDEX.md) | Mesh → SDF distance and sign, glTF import, GPU baking, bake QA, a baked leaf in migera's raymarcher, SDF → mesh extraction. | Baking mesh assets or extracting meshes. |
| [Performance and Production](./performance-and-production/INDEX.md) | Cost model, sparse/hierarchical structures, Dreams vs. Claybook. | Estimating feasibility or designing an SDF engine architecture. |
| [State of the Art](./state-of-the-art/INDEX.md) | Neural SDFs, Lumen, modeling tools, 2024-2026 research snapshot. | Judging how settled a technique is. |

## See also

- [Hybrid architecture](../hybrid-architecture/INDEX.md) — migera's own SDF renderer (`src/hybrid`): design decisions, GI bugs, performance findings.
- [Raymarching via compute](../compute-shaders/raymarching-via-compute.md) — running sphere tracing in WGSL compute.
- [Hierarchical volumes](../hierarchical-volumes/INDEX.md) — BVHs and grids for accelerating SDF queries.
- [Analytic intersections](../analytic-intersections/INDEX.md) — closed-form ray hits as an alternative tier to marching.
- [SDF + 3DGS + Bevy integration](../sdf-3dgs-bevy-integration/INDEX.md) — archived record of migera's former SDF-to-splat renderer (removed by 2026-09-06).

## Archived / superseded

| Note | What it establishes | Read when |
|---|---|---|
| [Converting GLB scenes to SDF BSN scenes](./glb-to-bsn-conversion.md) | Dropped plan to fit the Khorinis GLB with SDF primitives; its Phase 1 primitives landed (43017bb), the GLBs and `tools/glb_to_bsn` are gone. | Only for history, or before reviving mesh → primitive fitting. |
