---
title: "Efficient baking: GPU algorithms, resolution, and storage"
description: Covers GPU mesh-to-grid-SDF baking (brute force, binned/BVH, jump flooding), choosing resolution and bounds (Epic's 50³ average / 128³ max), narrow-band baking, Unreal's clipmap two-tier architecture, fp16/BC4/BC5/SVDAG compression, and storing gradients. Read when building a baker or budgeting baked-SDF memory.
type: research
status: current
tags:
  - sdf
  - baking
  - gpu-compute
  - compression
  - lod
  - performance
updated: 2026-08-16
aliases:
  - mesh-to-SDF baking
  - jump flooding
  - clipmap
  - SVDAG
---

# Efficient baking: GPU algorithms, resolution, and storage

Contents: [GPU pipelines](#gpu-compute-shader-baking-pipelines) ·
[Resolution and bounds](#choosing-grid-resolution-and-bounds) · [Narrow band](#narrow-band-baking) ·
[Clipmaps](#multi-resolution--clipmap-architecture-unreals-global-distance-field) ·
[Compression](#compression-and-storage) ·
[Gradients](#precomputing-gradientsnormals-alongside-distance) · [Related](#related)

This document covers the performance-engineering side of baking a triangle mesh into a
sampled-grid SDF — GPU algorithms and storage tradeoffs. For the unsigned-distance and
sign-determination math itself, see
[exact-point-to-mesh-distance](./exact-point-to-mesh-distance.md) and
[sign-determination-methods](./sign-determination-methods.md); for glTF-specific
preprocessing, see [gltf-import-pipeline](./gltf-import-pipeline.md).

## GPU compute-shader baking pipelines

**Brute-force triangle raytracing is the actual production baseline**, not just a naive
strawman: Epic's own offline Mesh Distance Field baker casts rays in many directions per
voxel to find the nearest surface, determining sign via a backface-hit-ratio heuristic
(>50% backface hits into the mesh means "inside"). This runs offline at asset-build time,
explicitly too slow for runtime use on arbitrary meshes
([Wright, "Dynamic Occlusion with Signed Distance Fields," SIGGRAPH 2015](https://advances.realtimerendering.com/s2015/DynamicOcclusionWithSignedDistanceFields.pdf),
slides 19, 22).

**Two-stage pipelines (coarse triangle binning + parallel local search)** are the norm
for anything closer to real-time: bin triangles into a uniform grid or BVH first, then
dispatch one compute thread per output voxel to search only nearby bins/subtrees,
expanding the search radius until a triangle is found — the standard pattern behind
tools like `mesh_to_sdf`/`cumesh2sdf`.

**Jump flooding algorithm (JFA), generalized to 3D, is a proven real-time technique, not
just an academic curiosity.** JFA splats seed distances near the surface into voxels,
then flood-propagates nearest-seed information across the grid in `O(log n)` passes with
halving offsets — the same algorithm family as GPU Voronoi diagrams, extended
straightforwardly from 2D textures to 3D volumes (more texture reads per pass, same
structure). **Unity's `com.unity.demoteam.mesh-to-sdf`** (used for animated-character
VFX/hair collision) is a concrete production implementation offering either jump flood or
linear flood fill, benchmarked at 0.22ms for a 32³ volume / 5,000-triangle mesh on an
RTX 3090 — its README explicitly notes the technique "sacrifices robustness and the
ability to handle large meshes" for speed, recommending 5-8k triangles in a 16³-64³
volume and falling back to a low-res proxy beyond that
([GitHub](https://github.com/Unity-Technologies/com.unity.demoteam.mesh-to-sdf)).
**RTSDF** (GRAPP 2022) is the most rigorous documented hybrid: a fast JFA pass produces a
coarse approximate SDF, which then acts as a "ray mask" deciding *where* to spend
expensive, accurate triangle ray-tracing (only within a threshold distance of the
surface, `d=0.1` found optimal) — explicitly combining "the speed of jump flooding with
the precision of ray tracing," reporting voxelization 0.93ms + jump flooding 2.09ms + ray
tracing 0.55-8.36ms depending on ray count
([arXiv:2210.06160](https://ar5iv.labs.arxiv.org/html/2210.06160)).

JFA's accuracy caveat, stated concretely by RTSDF: it only propagates distances to
whatever seed points were splatted, so voxelization/splatting quality directly limits
accuracy — thin/disconnected surfaces can produce "hollow surface representations with
disconnected regions," which RTSDF corrects with a small uniform positive bias (0.01
units) that deliberately thickens all surfaces before storing them (see
[glitch-free-baked-sdfs](./glitch-free-baked-sdfs.md) for more on this class of
thin-feature-loss artifact).

## Choosing grid resolution and bounds

Unreal's production numbers are the most concrete available reference point:
**50³ voxels is "enough for the average mesh"** (240KB at fp16), with a project-wide
"Distance Field Voxel Density" setting deriving voxel size from each mesh's own bounding-
box scale (not a fixed absolute resolution) rather than one global number, and a hard cap
of **128×128×128 / 8MB per mesh**
([Wright, SIGGRAPH 2015](https://advances.realtimerendering.com/s2015/DynamicOcclusionWithSignedDistanceFields.pdf),
slides 19, 24-25;
[Unreal Engine docs](https://dev.epicgames.com/documentation/unreal-engine/mesh-distance-fields-properties-in-unreal-engine)).
Corners visibly round off as resolution drops, and — critically — **a thin feature
narrower than roughly one voxel loses its interior negative region entirely**, which
breaks root-finding for surface reconstruction; this is the Nyquist-style sampling
argument developed further in [glitch-free-baked-sdfs](./glitch-free-baked-sdfs.md#1-thin-feature-loss).
Unity's SDF Bake Tool exposes the simpler single-scalar version of the same principle: a
"Maximal Resolution" defined as voxel count along the bounding box's longest side
([docs](https://docs.unity3d.com/Packages/com.unity.visualeffectgraph@17.1/manual/sdf-bake-tool-api.html)).

**Automatic feature-adaptive resolution for the baked dense grid itself is not something
either Unreal or Unity does in production** — neither engine's documentation describes
automatically-finer-near-thin-features grids. Instead, both solve "avoid wasting memory
in open volume" architecturally, via the two-tier clipmap system below, rather than via
per-voxel adaptive density within a single bake.

## Narrow-band baking

Restricting per-voxel work to a maximum distance from the surface is standard and
effective: one GPU narrow-band implementation found **5 cells sufficient to generate a
field with no gaps**, at 0.071-0.234s SDF-generation time depending on cell size/band
width for meshes from 52K to 2.8M triangles
([al-ro, "Fast Narrow Band SDF Generation on GPUs"](https://al-ro.github.io/projects/sdf/)).

**Handling the region outside the band correctly is what prevents glitches** (see
[glitch-free-baked-sdfs](./glitch-free-baked-sdfs.md) for the on-screen symptoms of
getting this wrong). Epic's documented fix: the baked volume is padded with a
guaranteed-empty border, and any sample falling outside the valid volume gets **clamped
to the nearest point inside the volume**, with the true (unclamped) distance from the
query point to that clamped sample point added on top — producing a correct, if
approximate, lower-bound distance rather than an undefined or zero value ("Samples
outside the valid area are clamped. Composite distance gives approximation" —
[SIGGRAPH 2015 talk](https://advances.realtimerendering.com/s2015/DynamicOcclusionWithSignedDistanceFields.pdf),
slide 20).

## Multi-resolution / clipmap architecture (Unreal's Global Distance Field)

The best-documented production reference for "many baked fields, one scene, no
re-baking regardless of view distance" is Unreal's two-tier system:

1. **Per-mesh (object-space) field** — baked once offline per unique static-mesh asset
   at the resolution described above, reused across every instance via a transform (the
   primary memory-saving trick, since the SDF itself is stored once per asset, not per
   placement).
2. **Global Distance Field** — a "cache of the per-object Mesh Distance Fields...
   composited into a few volume textures centered around the camera, called clipmaps"
   ([Unreal Engine docs](https://dev.epicgames.com/documentation/en-us/unreal-engine/mesh-distance-fields-in-unreal-engine)).
   Epic's 2015 implementation used 4 clipmaps at 128³ each, scrolled with camera
   movement; new slices are composited from object SDFs only as revealed by motion or
   dirtied by moved objects — "average cost of maintaining is close to 0," worst case
   ~7ms on teleports (slide 51). In current Lumen, the combination rule is explicit:
   **"Lumen traces against each mesh's distance field for the first two meters for
   accuracy, and the merged Global Distance Field for the rest of each ray"**
   ([Lumen Technical Details](https://dev.epicgames.com/documentation/unreal-engine/lumen-technical-details-in-unreal-engine)).

This decouples "detail of a single mesh" (fixed, offline, one-time cost) from "coverage
of a large dynamic world" (small, incremental, distance-graduated runtime cost) — nearer
clipmap slices update more often than farther ones. See
[sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md)'s
own clipmap section for the general structure this specializes.

## Compression and storage

**fp16 (half-float) is the concrete production precedent** — Epic's per-mesh bake uses
fp16, giving ~240KB for a 50³ mesh and ~300MB for a 512³ whole-level atlas
([SIGGRAPH 2015 talk](https://advances.realtimerendering.com/s2015/DynamicOcclusionWithSignedDistanceFields.pdf),
slides 19, 26). Plain 8-bit storage is used elsewhere but has a documented SDF-specific
quality problem: values quantized to an 8-bit unsigned integer "cannot completely store
distance field values as 32-bit floating-point numbers, causing serious damage to the
quality of the signed distance field" — a known limitation motivating higher-precision or
multi-channel encodings for anything beyond the coarsest use case.

**BC4/BC5 block compression** applies directly to single/dual-channel distance data: BC4
stores one channel per 4×4 texel block as two endpoints plus per-texel 3-bit
interpolation selectors (8 bytes/block, 2 bits/texel effective), with an SNORM (signed)
variant that's a natural fit for signed distance values — GPU hardware decodes to
better-than-8-bit precision internally, so a higher-precision source can still compress
usefully into it
([intro to BCn compression](https://acefanatic02.github.io/posts/intro_bcn_part1/)).

**2024-2026 research on more aggressive compression** (worth knowing about, not yet
production-proven): **Wavelet Latent Diffusion (WaLa, Nov 2024)** compresses a 256³ SDF
grid into a 12³×4 latent — a **2427x compression ratio** — via wavelet-domain encoding,
aimed at generative modeling but demonstrating extreme baked-SDF compression is
achievable with acceptable detail loss
([arXiv:2411.08017](https://arxiv.org/html/2411.08017)). **"Transform-Aware Sparse Voxel
Directed Acyclic Graphs"** (ACM TOG/I3D, May 2025) extends SVDAG compression (see
[sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md))
by matching subtrees under geometric transforms (rotation/mirroring), not just exact
duplicates, plus a new pointer-encoding scheme
([DL.ACM](https://dl.acm.org/doi/10.1145/3728301)).

## Precomputing gradients/normals alongside distance

**Production precedent leans toward not storing gradients** — Epic's baked mesh SDF
stores distance only; even for particle-collision queries needing a surface normal, "the
gradient is computed on the fly" via finite-difference sampling at query time, not
precomputed. This matches the "cheap, no extra storage" tradeoff most raymarchers already
make for procedural primitives (see
[normal-estimation](../rendering/normal-estimation.md)).

The alternative is real and documented, with an explicit accuracy/memory tradeoff:
**Gradient-SDF** (CVPR 2022) stores a full gradient vector alongside distance at every
voxel, demonstrating this is "significantly more accurate than a gradient obtained via
standard finite-difference sampling" at query time — at the cost of roughly doubling-to-
quadrupling per-voxel memory (a 3-component vector alongside the scalar), which the paper
notes pushes implementations toward sparse voxel storage to control the added cost
([arXiv:2111.13652](https://arxiv.org/abs/2111.13652)). For a real-time raymarcher
already paying for several finite-difference taps per shaded pixel (see
[normal-estimation](../rendering/normal-estimation.md)), the cheap/no-storage default is
the reasonable starting point; only reach for stored gradients if finite-difference
normal quality specifically proves inadequate near thin/high-curvature baked features
(see [glitch-free-baked-sdfs](./glitch-free-baked-sdfs.md#5-aliasing-and-staircasing-on-baked-curved-surfaces)).

## When to dive in

- Building a first mesh-to-grid-SDF baker → start with brute-force per-voxel triangle
  search for correctness, then move to binned/BVH-accelerated search once baking speed
  matters; reach for JFA only once real-time (not just "fast offline") baking is a
  requirement.
- Choosing a bake resolution → start from Epic's 50³-"average"/128³-max reference points,
  scaled to your mesh's own bounding-box size, not a single fixed number across all
  assets.
- Memory budget is tight for many baked assets → narrow-band baking (5-voxel band is a
  reasonable starting point) plus fp16 storage; only reach for BC4/BC5 or wavelet/SVDAG
  compression once the simpler options are proven insufficient.
- Designing a scene with many baked objects at varying distance from camera → the
  clipmap/two-tier pattern (per-object field + coarse scene-wide composite) is the
  proven architecture, not per-object mip chains alone.

## Related
- [Exact point-to-mesh distance](./exact-point-to-mesh-distance.md) — prerequisite: the distance query being accelerated.
- [Narrow-band and truncated SDFs](../fundamentals/narrow-band-and-truncated-sdfs.md) — prerequisite: why a narrow band suffices.
- [Glitch-free baked SDFs](./glitch-free-baked-sdfs.md) — deeper: validating what this baker produces.
- [Unreal Lumen distance fields](../state-of-the-art/unreal-lumen-distance-fields.md) — example: the clipmap architecture in production.
- [SDF + 3DGS + Bevy integration](../../sdf-3dgs-bevy-integration/INDEX.md) — contrast: migera's former (archived) bake of SDFs into splats rather than grids.
- [Compute shader performance best practices](../../compute-shaders/performance-best-practices.md) — applies: dispatch/workgroup rules for a GPU baker.
