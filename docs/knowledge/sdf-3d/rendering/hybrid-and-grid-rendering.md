---
title: Rendering grid-based and hybrid SDF representations
description: Covers trilinear grid SDFs, empty-space skipping, and the two dominant production hybrids — SDFs only for secondary rays with rasterized primary visibility (Lumen), and SDFs for authoring converted to points (Dreams) — plus why RT hardware doesn't help sphere tracing. Read before choosing a large-scene SDF design.
type: concept
status: current
tags:
  - sdf
  - raymarching
  - ray-tracing
  - spatial-acceleration
  - prior-art
  - performance
updated: 2026-08-15
aliases:
  - grid SDF rendering
  - hybrid rendering
  - global distance field
---

# Rendering grid-based and hybrid SDF representations

The [sphere-tracing](./sphere-tracing.md) algorithm applies identically whether the SDF
being evaluated is procedural or a sampled grid — but grid-based and production hybrid
systems add distinct machinery worth understanding separately from the pure-procedural
shader-art case.

## Trilinear interpolation and its cost

Evaluating a sampled-grid SDF (see
[sdf-representations](../fundamentals/sdf-representations.md)) at an arbitrary point
requires trilinear interpolation between the 8 surrounding grid samples. This is a fixed,
small, O(1) cost per query regardless of scene complexity — the key advantage over
procedural evaluation for complex geometry, where an equivalently detailed procedural
expression tree could require many more arithmetic operations per query.

The interpolation does slightly soften the effective Lipschitz guarantee near grid cell
boundaries and can blur genuinely sharp features finer than the grid spacing — this is
the resolution-vs-storage tradeoff inherent to any sampled representation (see
[performance-characteristics](../performance-and-production/performance-characteristics.md)).

## Empty-space skipping with hierarchical structures

A dense grid alone doesn't help a raymarcher skip large empty regions efficiently beyond
what the local SDF value already provides — but for very large scenes, an additional
coarse acceleration structure (sparse octree, clipmap, voxel hashing — see
[sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md))
lets rays jump past entire empty regions in one step rather than relying purely on
local SDF magnitude, which becomes especially important once the field is
[narrow-band/truncated](../fundamentals/narrow-band-and-truncated-sdfs.md) and can no
longer report large distances honestly far from the surface.

## Hybrid: SDF for ray tracing bounces, mesh/points/rasterization for primary visibility

A major class of production systems — Unreal's Lumen being the most widely deployed
example — does **not** raymarch SDFs for primary (camera-visible) rendering at all.
Instead:

- **Primary visibility** is handled by conventional rasterization (or hardware ray
  tracing against triangle geometry).
- **Secondary effects** (global illumination bounce lighting, reflections, soft shadows,
  ambient occlusion) trace rays against a much cheaper, lower-detail SDF representation
  of the scene, since these effects don't need pixel-perfect primary-visibility geometric
  accuracy — an approximate, fast-to-trace SDF proxy is visually sufficient for indirect
  lighting.

See [unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md)
for the concrete architecture (per-mesh baked distance fields composited into a scene-wide
clipmap Global Distance Field, with two tracing modes trading detail against speed). This
pattern — expensive/exact geometry for what the eye scrutinizes directly, cheap/
approximate SDF geometry for what only needs to look *plausible* in bounce lighting — is
arguably the dominant production use of SDFs in current AAA game engines, more so than
full-scene SDF raymarching.

## Hybrid: SDF for authoring/physics, non-SDF for final rendering

A second hybrid pattern, distinct from Lumen's "SDF for indirect rays only" approach: use
SDFs as the *authoring and simulation* representation throughout, but convert to an
entirely different representation specifically for the final rasterized image. Media
Molecule's *Dreams* is the reference example — CSG trees over SDFs are evaluated into
dense point clouds, which are what actually gets rendered (splatted), not raymarched SDF
data directly. See
[production-case-studies](../performance-and-production/production-case-studies.md) for
the full architecture and its rationale.

## Hardware acceleration trends

As of 2025-2026, dedicated hardware ray-tracing units (RT cores and equivalents) are
built and optimized around triangle/BVH intersection, not SDF sphere tracing — so pure
SDF raymarching generally cannot directly leverage this hardware path today. Current
research and production practice both lean toward hybrid approaches that convert SDF
scenes into triangle-BVH-compatible representations (or, per Lumen, only use SDFs for the
rays that specifically benefit from the tradeoff) rather than pursuing hardware-
accelerated sphere tracing as a first-class hardware feature — see
[state-of-the-art](../state-of-the-art/INDEX.md) for where research is actively pushing
on this boundary.

## When to dive in

- Building a scene too complex for pure procedural evaluation to stay real-time → move to
  a sampled grid representation, and read
  [mesh-to-sdf-conversion](../mesh-conversion/INDEX.md) for how to bake it.
- Deciding between full-scene SDF raymarching and a hybrid rasterization+SDF-for-bounces
  architecture → the hybrid approach is what current production engines actually ship;
  read [unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md)
  before committing to pure raymarching for a large, detailed scene.
- Considering SDFs as an authoring/physics tool with a different final render path → read
  [production-case-studies](../performance-and-production/production-case-studies.md) for
  the Dreams/Claybook architectures.

## Related
- [Unreal Lumen distance fields](../state-of-the-art/unreal-lumen-distance-fields.md) — deeper: the SDF-for-secondary-rays architecture in detail.
- [Production case studies](../performance-and-production/production-case-studies.md) — example: Dreams and Claybook.
- [Sparse and hierarchical structures](../performance-and-production/sparse-and-hierarchical-structures.md) — deeper: empty-space skipping structures.
- [migera pivots to Bevy PBR for characters](../../hybrid-architecture/migera-pivot-to-bevy-pbr-for-characters.md) — applies: migera's own move to rasterized characters with SDF kept for effects/worldgen.
- [Self-shading vs. G-buffer decision](../../hybrid-architecture/self-shading-vs-gbuffer-decision.md) — applies: how migera's `src/hybrid` composes SDF output with Bevy's pipeline.
- [Hierarchical grids and trees](../../hierarchical-volumes/hierarchical-grids-and-trees.md) — deeper: coarse occupancy hierarchies for skipping.
