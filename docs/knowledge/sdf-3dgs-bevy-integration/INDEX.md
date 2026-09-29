---
title: "SDF + 3D Gaussian Splatting + Bevy: Native Integration Architecture"
description: Archived record of migera's former design for baking an authored SDF into 3DGS splats rendered as native Bevy Core3d passes, built 2026-08 and removed by 2026-09-06, plus never-built live-editing research. Read before reconsidering splats, or for its still-valid Bevy custom-pass and compositing lessons.
type: index
status: current
tags:
  - sdf
  - 3dgs
  - bevy
  - integration
  - baking
updated: 2026-09-28
---

# SDF + 3D Gaussian Splatting + Bevy: Native Integration Architecture

A synthesis across [3D Signed Distance Fields](../sdf-3d/INDEX.md),
[3D Gaussian Splatting](../3dgs/INDEX.md) and [Bevy Rendering](../bevy-rendering/INDEX.md):
use an SDF as the authored description of the world, bake it into Gaussian splats (a
forward sampling problem, not the inverse problem photographic 3DGS training solves),
and render those splats as a native `Core3dSystems::MainPass` citizen so Bevy's built-in
post-processing works on them unchanged.

**Status (2026-09-28): the whole subtree is archived.** The design was built as a demo
(commit eac219e, 2026-08-15: CPU Poisson-disk bake, two-pipeline opaque/translucent Bevy
integration, chunk streaming in 1fe5e40), deleted in 22d3b91 (2026-08-16) in favour of
the SDF raymarcher, re-added as a splat far tier inside `src/hybrid` (8ace6ac,
2026-08-28), and removed for good in 684490c (2026-09-06). migera now sphere-traces the
SDF directly in `src/hybrid` and renders characters with Bevy PBR. The
[live-editing](./live-editing/INDEX.md) topic was never built at all. Treat every note
here as history: re-verify any claim before reusing it.

## Key facts

Still-valid lessons from the built implementation:

1. Opaque splats need a real depth write and test; Weighted Sum Rendering's depth-test-without-write needs a bias with no angle-independent value — [Compositing: WSR vs. depth-tested opaque rendering](./render-integration/sort-free-compositing.md)
2. Translucent custom geometry can reuse Bevy's `Transparent3d` phase unmodified — [Native integration with Bevy's render pipeline](./architecture/bevy-pipeline-integration.md)
3. A custom opaque pass must be ordered `.after(main_opaque_pass_3d)` **and** `.before(main_transparent_pass_3d)`, or translucent content gets painted over — [Native integration with Bevy's render pipeline](./architecture/bevy-pipeline-integration.md)
4. Writing the shared `ViewTarget` makes tonemapping/bloom/FXAA work for free; TAA/SSAO/DOF need a prepass contribution — [Screen-space effect compatibility](./render-integration/screen-space-effect-compatibility.md)
5. SDF surface sampling traps: jump within the tangent plane, verify seed normals are non-degenerate, round hard edges in the SDF rather than patching the sampler — [Baking a Gaussian splat cloud from an SDF](./baking-pipeline/sdf-to-splat-baking.md)
6. Bake vs. raymarch was argued from precedent (Dreams) but never measured; migera chose raymarching — [Bake vs. direct raymarch efficiency](./live-editing/bake-vs-direct-raymarch-efficiency.md)

## Topics

| Topic | What it establishes | Read when |
|---|---|---|
| [Architecture](./architecture/INDEX.md) | SDF as source of truth and splats as render target; the Bevy component/asset/phase/pass scaffolding | Reconsidering an SDF-to-splat design, or adding a custom geometry pass to Core3d |
| [Baking Pipeline](./baking-pipeline/INDEX.md) | The four-step forward SDF-to-splat bake, a GPU-compute variant, and chunked streaming/LOD/invalidation | Sampling points or splats off an SDF, or designing chunked SDF streaming |
| [Render Integration](./render-integration/INDEX.md) | Opaque depth-written vs. translucent `Transparent3d` compositing, and which Bevy post effects need a prepass | Compositing soft primitives in Bevy, or checking post-process support for a custom pass |
| [Live Editing](./live-editing/INDEX.md) | Never-built research on live re-bake, deformation vs. re-bake, SDF-driven physics and lighting, and bake-vs-raymarch efficiency | Planning SDF sculpting/terraforming, SDF collision, or reopening bake vs. raymarch |

## See also

- [Hybrid renderer architecture](../hybrid-architecture/INDEX.md) — what migera built instead: a self-shading SDF raymarcher.
- [Production case studies: Dreams and Claybook](../sdf-3d/performance-and-production/production-case-studies.md) — the SDF-authoring precedents this design copied.
