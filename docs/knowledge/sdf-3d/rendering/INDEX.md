---
title: Rendering
description: How pixels come out of an SDF — the sphere-tracing loop, normal estimation, soft shadows and AO (including migera's src/hybrid shadow findings), a symptom→fix artifact map, and grid/hybrid production architectures. Read when implementing or debugging an SDF raymarcher.
type: index
status: current
tags:
  - sdf
  - raymarching
  - shadows
  - troubleshooting
updated: 2026-09-28
---

# Rendering

The sphere-tracing algorithm itself, how to shade what it finds, how to debug it
when it goes wrong, and how production systems actually deploy SDF rendering
(which, as of 2025-2026, is usually *not* pure full-scene raymarching).

## Start here

Minimum viable renderer: [sphere-tracing](./sphere-tracing.md) →
[normal-estimation](./normal-estimation.md). Something looks wrong →
[raymarching-artifacts-and-fixes](./raymarching-artifacts-and-fixes.md) first.

## Key facts

- Fixed `MAX_STEPS` makes grazing/thin geometry miss; that's a convergence issue, not an SDF bug — see [sphere-tracing](./sphere-tracing.md).
- Soft-shadow banding is removed by Aaltonen's closest-point interpolation, not by more steps — see [soft-shadows-and-ao](./soft-shadows-and-ao.md).
- In migera's `src/hybrid`, the shadow margin uses a fixed object-scale `PENUMBRA_REACH` (never scene extent), `k = 2.0` from a sweep, and padding on every BVH node — see [soft-shadows-and-ao](./soft-shadows-and-ao.md#porting-to-srchybrid-three-more-bugs-the-hybrid_legacy-port-didnt-warn-about).
- Per-object marches must use the object's own AABB slab, never a merged global interval list — see [raymarching-artifacts-and-fixes](./raymarching-artifacts-and-fixes.md).
- Production engines mostly use SDFs for secondary rays (Lumen) or authoring (Dreams), not primary visibility — see [hybrid-and-grid-rendering](./hybrid-and-grid-rendering.md).

## Notes

| Note | What it establishes | Read when |
|---|---|---|
| [Sphere tracing](./sphere-tracing.md) | The adaptive step-by-SDF loop, why it's safe, non-guaranteed convergence under a step cap, epsilon scaling, overrelaxation. | Writing or tuning a raymarch loop. |
| [Estimating surface normals from an SDF](./normal-estimation.md) | Gradient = normal via the 4-tap tetrahedron difference; offset-h tradeoff; edge instability; analytic gradients. | Shading a hit, or chasing normal noise on sharp geometry. |
| [Soft shadows and ambient occlusion from an SDF](./soft-shadows-and-ao.md) | IQ `k`-penumbra → Aaltonen fix → interior penumbras; 5-tap AO; migera's 1/t-decay, slab-truncation, margin, `k` and BVH-padding findings. | Before touching `trace_shadow`, or when shadows band, look boxy, or falsely darken at scale. |
| [Common raymarching artifacts and their causes](./raymarching-artifacts-and-fixes.md) | Symptom → cause → fix for punch-through, floating, banding, thin-geometry misses, acne, normal noise, and three BVH-march bugs. | First stop for any visual raymarching bug. |
| [Rendering grid-based and hybrid SDF representations](./hybrid-and-grid-rendering.md) | Trilinear grid cost, empty-space skipping, Lumen-style and Dreams-style hybrids, RT hardware limits. | Choosing an architecture for a large/complex SDF scene. |

## See also

- [Raymarching via compute](../../compute-shaders/raymarching-via-compute.md) — the WGSL compute implementation side.
- [Hybrid architecture](../../hybrid-architecture/INDEX.md) — migera's own renderer design, GI and performance findings.
- [Hierarchical volumes](../../hierarchical-volumes/INDEX.md) — BVH traversal and padded queries used by shadow rays.
