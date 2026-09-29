---
title: Unreal Engine's Mesh Distance Fields and Lumen
description: Describes the most widely deployed production SDF system — offline-baked per-mesh distance fields, a scene-wide clipmap Global Distance Field, and Lumen's Detail vs. Global software ray tracing tiers used for indirect light, not primary visibility. Read when designing a hybrid SDF architecture or SDF-based GI.
type: reference
status: current
tags:
  - sdf
  - global-illumination
  - prior-art
  - baking
  - lod
updated: 2026-08-15
aliases:
  - Lumen
  - Mesh Distance Field
  - Global Distance Field
  - software ray tracing
  - Unreal Engine 5
---

# Unreal Engine's Mesh Distance Fields and Lumen

Unreal Engine's Lumen global illumination system is, as of Unreal Engine 5, the most
widely deployed production use of SDFs in a mainstream commercial game engine — and its
architecture is a clear real-world illustration of the
[hybrid rendering pattern](../rendering/hybrid-and-grid-rendering.md) (SDFs for indirect
rays, not primary visibility) discussed generally elsewhere in this knowledge base.

## Mesh Distance Fields: per-object baked SDFs

Each static mesh in an Unreal scene can have a **Mesh Distance Field** generated for it:
a sampled-grid SDF (see [sdf-representations](../fundamentals/sdf-representations.md))
stored in a volume texture, covering that mesh's local bounding volume. Critically, this
generation is done **offline**, using triangle raytracing against the source mesh to
compute distances (i.e. the [exact-point-to-mesh-distance](../mesh-conversion/exact-point-to-mesh-distance.md)
+ [sign-determination](../mesh-conversion/sign-determination-methods.md) pipeline covered
in this knowledge base's mesh-conversion topic) — Mesh Distance Field generation cannot
be done at runtime, which is a direct practical consequence of that baking cost.

## Global Distance Field: composited clipmap

At runtime, Unreal composites all relevant per-object Mesh Distance Fields into a single
**Global Distance Field** — an abstract, scene-wide volumetric representation
implemented as a **clipmap** (see
[sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md)
for the general clipmap concept): higher resolution near the camera, progressively
coarser farther away. The Global Distance Field is deliberately a "bare-bones geometrical
representation with minimal per-object detail" — it is not meant to reproduce fine mesh
detail, only enough large-scale geometric structure for cheap, scene-wide occlusion and
lighting-bounce queries.

## Two-tier software ray tracing

Lumen's default ray-tracing mode (Software Ray Tracing, as opposed to hardware RT-core-
based tracing) offers two distinct tracing strategies against these distance fields, a
direct quality/cost dial:

- **Detail Tracing** — traces against individual meshes' own Mesh Distance Fields for
  higher-fidelity results (closer objects, where the coarse Global Distance Field's lack
  of per-object detail would be visually noticeable), at higher per-ray cost.
- **Global Tracing** — traces only against the composited Global Distance Field for
  the fastest possible tracing at reduced geometric fidelity, used for distant/less-
  visually-critical rays where the coarser representation is visually sufficient.

Rays are dynamically routed between these two tiers depending on distance/importance —
the architectural embodiment of the general principle from
[hybrid-and-grid-rendering](../rendering/hybrid-and-grid-rendering.md) that indirect
lighting rays don't need the same geometric fidelity as primary camera rays.

## Why this matters as the SOTA reference point

Lumen is significant not because it does something novel at the algorithmic level (per-
mesh distance fields, clipmap compositing, and software ray tracing against grid SDFs are
all techniques covered individually elsewhere in this knowledge base) but because it
demonstrates these techniques **integrated and shipping at massive scale** across a huge
range of commercial games, hardware targets, and content types — arguably the strongest
existing evidence that SDF-based techniques, deployed with the right architectural
discipline (hybrid, not full-scene raymarching-only), are production-ready for
mainstream AAA game development rather than a niche/demoscene technique.

## Relationship to hardware ray tracing

Lumen also supports a hardware-ray-tracing mode (tracing against triangle BVH via RT
cores rather than software sphere-tracing against SDFs), offered as a higher-fidelity
alternative on hardware that supports it. The existence of both modes side-by-side in one
shipping system is itself informative: it confirms that as of 2025-2026, SDF-based
software ray tracing and hardware triangle ray tracing are treated as complementary,
hardware-availability-dependent alternatives rather than one having fully superseded the
other — see the hardware-acceleration discussion in
[hybrid-and-grid-rendering](../rendering/hybrid-and-grid-rendering.md).

## When to dive in

- Evaluating whether to enable/tune Lumen's Software Ray Tracing settings in a real
  project → understanding Detail vs. Global Tracing's cost/quality tradeoff (this
  document) is the prerequisite before touching the relevant engine settings.
- Designing a custom hybrid rendering architecture and looking for a proven reference
  design → Lumen's "per-object baked SDF + scene-wide clipmap + tiered tracing" pattern
  is the most battle-tested template currently available.
- Wondering whether SDF techniques are "real" production technology or mostly a research/
  demoscene curiosity → Lumen's scale of deployment is the strongest available counter-
  argument to that skepticism.

## Related
- [Rendering grid-based and hybrid SDF representations](../rendering/hybrid-and-grid-rendering.md) — prerequisite: the general hybrid pattern Lumen instantiates.
- [Sparse and hierarchical structures](../performance-and-production/sparse-and-hierarchical-structures.md) — prerequisite: the clipmap concept.
- [Efficient grid baking](../mesh-conversion/efficient-grid-baking.md) — deeper: per-mesh bake resolution and storage.
- [DDGI any-hit occlusion](../../hybrid-architecture/gi-and-lighting/ddgi-any-hit-occlusion.md) — contrast: migera's probe-based GI and its measured cost.
- [Production case studies](../performance-and-production/production-case-studies.md) — contrast: Dreams and Claybook, the other two production patterns.
