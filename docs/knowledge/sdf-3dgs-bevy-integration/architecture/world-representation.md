---
title: SDF as world description, 3DGS as world rendering
description: Records migera's former top-level design, SDF as the authored source of truth (authoring, collision) baked into 3DGS splats for rendering, modelled on Dreams, and why direct SDF raymarching was rejected then. Archived; migera later chose raymarching. Read before reconsidering an SDF-to-splat architecture.
type: design
status: archived
tags:
  - sdf
  - 3dgs
  - bevy
  - raymarching
  - integration
updated: 2026-08-15
sources:
  - commit eac219e (demo built)
  - commit 22d3b91 (splat pipeline removed)
  - commit 684490c (3DGS dropped from src/hybrid)
aliases:
  - source of truth
  - world representation
  - Dreams architecture
---

# SDF as world description, 3DGS as world rendering

> **Archived (2026-09-28):** migera reversed this design. The splat pipeline was removed in commit 22d3b91 (2026-08-16) and 3DGS was dropped from `src/hybrid` in 684490c (2026-09-06); the SDF is now sphere-traced directly by `src/hybrid`, and the "why not raymarch directly" argument below is the one that lost.

## The core architectural split

This system separates two concerns that are conflated in most single-representation
renderers: **what the world *is*** and **what gets drawn to the screen**. An SDF is used
for the former, a baked 3DGS splat cloud for the latter, connected by an offline/async
bake step rather than either representation trying to serve both roles.

This is not a novel invention — it is the direct architectural pattern already
identified and analyzed in both source knowledge bases independently:

- The SDF knowledge base documents this exact split as the **Dreams architecture**:
  "Operationally Transformed CSG trees... evaluated on-the-fly to high resolution signed
  distance fields, from which we generate dense multi-resolution point clouds" for
  rendering (see
  [production-case-studies](../../sdf-3d/performance-and-production/production-case-studies.md)).
  Dreams chose points; this design chooses Gaussian splats — a strictly richer point
  representation (each "point" carries anisotropic shape, opacity, and view-dependent
  color rather than being a bare position+color sample), for a direct, mechanical
  reason: [hybrid-and-grid-rendering](../../sdf-3d/rendering/hybrid-and-grid-rendering.md)
  already frames "convert SDF to a different representation because it renders faster"
  as a general, validated pattern; 3DGS is simply the most capable currently-available
  target representation for that conversion, given its
  [tile-based rasterizer](../../3dgs/rendering-and-rasterization/tile-based-rasterizer.md)
  achieves real-time rates that pure SDF raymarching cannot guarantee for arbitrarily
  complex scenes (see
  [performance-characteristics](../../sdf-3d/performance-and-production/performance-characteristics.md)
  — procedural SDF cost scales with expression-tree complexity per raymarch step, with no
  hardware-accelerated fast path).
- The 3DGS knowledge base independently documents the corresponding weakness on its own
  side: **3DGS is not a geometry representation, and standard training is per-scene,
  non-generalizable, and dependent on photographic capture** (see
  [nerf-comparison](../../3dgs/fundamentals/nerf-comparison.md) and
  [sfm-initialization](../../3dgs/optimization-and-training/sfm-initialization.md)). An
  SDF description sidesteps both problems: it's a compact, parametric, editable,
  procedurally-generatable world description with no capture step required at all (see
  [sdf-3dgs-baking](../baking-pipeline/sdf-to-splat-baking.md) for how this specifically
  changes the bake pipeline vs. standard 3DGS training).

## Why this division of labor, concretely

| Concern | Representation | Why |
|---|---|---|
| Authoring / world generation (procedural terrain, structures, CSG-composed props) | SDF | Trivial, always-well-defined boolean/smooth-blend composition (see [combination-operators](../../sdf-3d/primitives-and-operators/combination-operators.md)); infinite-resolution, LOD-free; exact analytic surface position *and* normal everywhere via the gradient (see [what-is-an-sdf](../../sdf-3d/fundamentals/what-is-an-sdf.md)) |
| Runtime collision / physics | SDF (or a mesh extracted from it) | SDF's negative-interior distances give a genuine penetration-depth notion current mesh-based physics engines can't cheaply derive (documented rationale for Claybook's choice — see [production-case-studies](../../sdf-3d/performance-and-production/production-case-studies.md)); the same [production maturity gap](../../3dgs/state-of-the-art/game-engine-integration.md) noted for 3DGS ("traditional meshes remain preferred" for physics) makes SDF/mesh, not splats, the correct choice here regardless |
| Real-time visual rendering | 3DGS (baked from the SDF) | Tile-based rasterization achieves real-time frame rates independent of scene *geometric* complexity in the way procedural SDF raymarching cannot guarantee (see [performance-characteristics](../../sdf-3d/performance-and-production/performance-characteristics.md)); Gaussians naturally represent soft/organic detail (foliage, fine surface noise) that a raymarched or meshed SDF renders less cheaply per-pixel |
| Bevy engine integration (culling, LOD, compositing with meshes/UI/post-process) | Bevy's native `Camera`/`ViewTarget`/`PhaseItem` machinery, applied to the baked splats | This is the whole point of the design — see [bevy-pipeline-integration](./bevy-pipeline-integration.md) |

## The data flow

```
  Author time / world-gen time                    Runtime (every frame, or on invalidation)
┌──────────────────────────┐                    ┌─────────────────────────────────────┐
│  SDF scene description   │                    │  Bevy render world                   │
│  (procedural CSG tree,   │   bake (§ baking-   │  ┌─────────────────────────────┐    │
│  primitives+operators,   │──  pipeline docs)──▶│  │ SplatCloud RenderAsset       │    │
│  optionally baked to a   │                    │  │ (position, cov, opacity, SH) │    │
│  sparse voxel grid for   │                    │  └──────────────┬────────────────┘    │
│  large/streamed worlds)  │                    │                 │                     │
└──────────────────────────┘                    │                 ▼                     │
         │ (also feeds)                         │  Custom PhaseItem in Core3dSystems::  │
         ▼                                       │  MainPass, alongside Opaque3d/        │
┌──────────────────────────┐                    │  Transparent3d — see                  │
│  Physics/collision:       │                    │  render-integration docs              │
│  SDF queried directly, or │                    └─────────────────────────────────────┘
│  meshed via Marching      │
│  Cubes for engines that   │
│  need triangles           │
└──────────────────────────┘
```

The SDF scene description is the **single source of truth** the world is authored
against — procedurally generated, hand-modeled with the primitive/operator toolkit (see
[primitives-and-operators](../../sdf-3d/primitives-and-operators/INDEX.md)), or a mix
(hand-placed primitives plus a baked/scanned sparse-grid region, per
[sdf-representations](../../sdf-3d/fundamentals/sdf-representations.md)). It is *not*
raymarched directly for the final rendered frame in this design (contrast with
Claybook's direct-raymarch architecture, also documented in
[production-case-studies](../../sdf-3d/performance-and-production/production-case-studies.md)
— that remains a legitimate alternative design for projects that don't need Bevy's
rasterization-pipeline ecosystem, but is explicitly not the path this document takes,
since the goal here is native integration with Bevy's *existing* screen-space-effect
stack, which assumes a rasterized `ViewTarget`).

## Why not raymarch the SDF directly and skip 3DGS entirely?

This is the natural alternative design, and worth stating explicitly why it's rejected
here: a full-screen SDF raymarch pass could be inserted into `Core3dSystems::MainPass`
directly (as a `FullscreenMaterial`, see
[passes-and-fullscreen-effects](../../bevy-rendering/core-pipeline/passes-and-fullscreen-effects.md)),
producing correct results and *also* composing with post-processing via the same
`ViewTarget` mechanism this design uses for splats. Three reasons this is not the chosen
path:

1. **Depth compositing with ordinary Bevy meshes is far more natural for rasterized
   splats than for a raymarched fullscreen pass.** A tile-rasterized splat cloud writes
   real per-pixel depth into the same depth buffer `main_opaque_pass_3d`/
   `main_transparent_pass_3d` use, so ordinary Bevy `Mesh3d` entities (UI-adjacent props,
   physics-driven objects, characters) interleave correctly via standard depth testing
   with no special-casing. A raymarched fullscreen pass produces one flat color+depth
   result per pixel from a single full-screen invocation, which is much more awkward to
   correctly depth-composite against arbitrarily-ordered opaque/transparent mesh phases
   without effectively re-deriving a phase-like system from scratch inside the raymarch
   shader itself.
2. **Per-pixel raymarch cost scales with scene description complexity** (see
   [performance-characteristics](../../sdf-3d/performance-and-production/performance-characteristics.md))
   in a way the pre-baked splat representation does not — baking amortizes that cost
   once, off the critical per-frame path, exactly analogous to why Unreal's Lumen bakes
   per-mesh distance fields offline rather than raytracing raw scene geometry every frame
   (see [unreal-lumen-distance-fields](../../sdf-3d/state-of-the-art/unreal-lumen-distance-fields.md)).
3. **The explicit, primitive-based nature of 3DGS is directly reusable by Bevy's existing
   render-phase/batching/LOD machinery** (binned phases, `RetainedViewEntity`-keyed
   caching, GPU-driven batching infrastructure — see
   [render-phases-and-batching](../../bevy-rendering/architecture/render-phases-and-batching.md))
   in a way a monolithic fullscreen raymarch shader is not — there's no per-primitive
   granularity for Bevy's culling/specialization/change-list systems to operate on inside
   a single fullscreen pass.

Both designs are legitimate; this document develops the splat-baking path specifically
because it is the one that maximizes reuse of Bevy's *existing* rendering
infrastructure — which is the explicit goal stated for this integration. For the
separate question of whether baking is actually *more efficient* than direct raymarching
(not just better-integrated with Bevy) — including where that argument is well-supported
by precedent and cost-model reasoning versus where it remains an unmeasured, project-specific
question — see
[bake-vs-direct-raymarch-efficiency](../live-editing/bake-vs-direct-raymarch-efficiency.md).

## When to dive in

- Deciding whether this architecture fits a project → the table above is the concrete
  decision matrix; if your project doesn't need Bevy's mesh/physics/UI ecosystem
  alongside photorealistic rendering, a simpler single-representation design (pure SDF
  raymarch, or pure captured 3DGS) may be less engineering effort.
- Understanding how the bake step actually works →
  [sdf-to-splat-baking](../baking-pipeline/sdf-to-splat-baking.md).
- Understanding how baked splats become a native Bevy render phase →
  [bevy-pipeline-integration](./bevy-pipeline-integration.md).

## Related

- [Native integration with Bevy's render pipeline](./bevy-pipeline-integration.md) — deeper: how the baked splats were wired into `Core3d`.
- [Baking a Gaussian splat cloud from an SDF](../baking-pipeline/sdf-to-splat-baking.md) — deeper: the bake step in the data flow.
- [Is baking to splats actually more efficient than raymarching the SDF directly?](../live-editing/bake-vs-direct-raymarch-efficiency.md) — contrast: the efficiency case, which was never measured.
- [Production case studies: Dreams and Claybook](../../sdf-3d/performance-and-production/production-case-studies.md) — prerequisite: the SDF-authoring precedents this design copied.
- [Self-shading vs. G-buffer-writer decision](../../hybrid-architecture/self-shading-vs-gbuffer-decision.md) — superseded-by: the raymarched renderer migera built instead.
- [What is 3D Gaussian Splatting?](../../3dgs/fundamentals/what-is-3dgs.md) — prerequisite: the render target representation.
