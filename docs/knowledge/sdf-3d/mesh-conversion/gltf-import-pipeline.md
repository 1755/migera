---
title: "Importing glTF/GLB models: format specifics and preprocessing"
description: Covers glTF-specific concerns for mesh-to-SDF baking — node/primitive structure and transform composition, winding flips from negative scale, skinning/morph handling, real-world mesh defects and repair, Rust crates (gltf, mesh_to_sdf), decimation, and bake-at-import caching. Read before writing a glTF→SDF importer.
type: research
status: current
tags:
  - sdf
  - assets
  - mesh-conversion
  - baking
  - correctness
updated: 2026-08-16
aliases:
  - GLB import
  - mesh_to_sdf crate
  - gltf-rs
  - mesh repair
---

# Importing glTF/GLB models: format specifics and preprocessing

Contents: [glTF structure](#gltf-structure-relevant-to-baking) ·
[Mesh quality problems](#real-world-mesh-quality-problems) · [Rust tooling](#rust-tooling) ·
[Simplification](#mesh-simplification-before-baking) ·
[Bake at import](#asset-pipeline-pattern-bake-at-import-time-cache-the-result) · [Related](#related)

This document covers what's specific to **glTF as a format** and to real-world "found"
mesh assets — the practical layer above the generic point-to-mesh-distance/sign-
determination algorithms already covered in
[exact-point-to-mesh-distance](./exact-point-to-mesh-distance.md) and
[sign-determination-methods](./sign-determination-methods.md).

## glTF structure relevant to baking

A glTF `mesh` is an array of `primitive` objects (one per material, typically) —
**a logical model is usually many primitives across many nodes**, not one clean triangle
list. Each primitive's `attributes` map to `accessor`-typed views into binary
`bufferView`s; only `POSITION` (and optionally `indices`) matters for baking. If
`indices` is absent, vertices are drawn in accessor order; if `mode` isn't `TRIANGLES`
(glTF also allows strips/fans/points/lines), a preprocessor must triangulate or reject
that primitive first. Concatenate every primitive's triangles across every mesh/node you
want to bake into one triangle soup — the SDF/winding computation doesn't care about
draw-call boundaries, only geometry.
([glTF 2.0 Spec](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html))

**Node transforms must be composed, not skipped.** Each node has either a `matrix` or
separate TRS (translation/rotation-quaternion/scale) properties. To bake a static SDF,
walk the scene graph from each root, composing each node's local transform with its
parent's accumulated world matrix, and apply that world matrix to every vertex position
before it enters the triangle soup — reading only a mesh's raw local-space accessor data
(skipping this composition) is a common bug with multi-node assets from Blender/Maya,
where raw vertex data looks wrong until the node transform is applied. The Rust `gltf`
crate's `Node::transform()` returns a `Transform` enum with `.matrix()`/`.decomposed()`
accessors built for exactly this composition.
([gltf crate docs](https://docs.rs/gltf/latest/gltf/enum.Transform.html))

**Skinning and morph targets: bake to one static pose, then discard.** Since a grid SDF
bake is inherently static, evaluate the mesh in one fixed pose (bind/rest pose is the
common default for static-prop-style assets) and bake the *resulting deformed vertex
positions* — this mirrors how DCC tools export a "static pose" glTF (e.g. Blender's
exporter's "Rest Position" toggle). If an asset is unposed/rest-pose already, simply
ignoring `skin`/`weights`/morph `targets` and reading raw `POSITION` is correct; posed or
animated assets need explicit CPU skinning/morph-blend evaluation before baking, which no
current Rust SDF tool automates.

**Coordinate system and winding — the real gotcha.** glTF is right-handed, +Y up, +Z
toward the viewer, units in meters
([glTF 2.0 Spec, "Coordinate System and Units"](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html)).
If your engine uses a different up-axis/handedness, apply one explicit basis conversion
at import (commonly folded into the root transform), not per-triangle at bake time. Watch
for **negative/mirrored scale on any node** — it flips the determinant sign of that
node's accumulated world matrix, which flips triangle winding/normal orientation for
everything under it. A bake that doesn't detect and correct for this per-node produces
inside-out sign regions exactly where a mirrored node's geometry sits — a real,
underdocumented failure mode distinct from mesh-quality problems (see below), tracked as
an ongoing coordinate-system ambiguity by Khronos itself
([glTF Issue #2352](https://github.com/KhronosGroup/glTF/issues/2352),
[Issue #566](https://github.com/KhronosGroup/glTF/issues/566)).

## Real-world mesh quality problems

Downloaded/"found" glTF assets (marketplace exports, AI-generated meshes, CAD
conversions, game-ripped models) routinely violate the single-closed-manifold assumption
naive mesh-to-SDF algorithms make:

- **Non-manifold geometry** — edges shared by more than two faces, or "bowtie" vertices
  connecting otherwise-disjoint patches.
- **Non-watertight meshes** — holes, gaps, and (specific to multi-part glTF scenes)
  separate primitives/nodes that are visually adjacent but topologically disconnected
  (e.g. a character's separate head/body/clothing meshes don't share a seam).
- **Inconsistent triangle winding** — mixed CW/CCW faces, common after mesh merges or
  mirrored-instance nodes (see the negative-scale note above).
- **Duplicate and degenerate (near-zero-area) triangles** — common "in GLB files exported
  from game engines after mesh merging or LOD generation"
  ([Tripo3D: Zero-Area Triangles and Degenerates](https://www.tripo3d.ai/blog/explore/smart-mesh-fixing-zero-area-triangles-and-degenerates)).
- **Self-intersections and multi-part union intent** — parts meant to read as one solid
  (bolts + plate, hard-surface CSG-authored props) stored as independently-bounded,
  non-topologically-joined primitives.

This is precisely the scenario
[sign-determination-methods](./sign-determination-methods.md) already recommends
**generalized winding number (GWN)** for — GWN treats every triangle as continuously
contributing a signed solid angle, producing a smooth scalar that degrades gracefully
(rather than failing catastrophically) in the presence of holes, self-intersections, and
non-manifold edges, which is exactly the defect profile of real-world glTF assets. The
2013 Jacobson/Kavan/Sorkine-Hornung paper introducing this received the SIGGRAPH
Test-of-Time Award in 2024, underscoring its continued centrality
([project page](https://igl.ethz.ch/projects/winding-number/)). The 2018 follow-up "Fast
Winding Numbers for Soups and Clouds" (Barill, Dickson, Schmidt, Levin, Jacobson) adds a
tree-accelerated approximation specifically because the original scales poorly on
**disconnected** geometry — exactly the multi-primitive/multi-part glTF case — and is
what production-grade libraries (libigl, Houdini's public `WindingNumber` code) actually
implement
([DL.ACM](https://dl.acm.org/doi/abs/10.1145/3197517.3201337),
[project page](https://www.dgp.toronto.edu/projects/fast-winding-numbers/)).

Two complementary repair strategies, both real and current:
1. **Repair to watertight first**, then bake with a cheaper sign method. **ManifoldPlus**
   (Huang, Zhou, Guibas, 2020) takes arbitrary non-manifold triangle-soup input and
   produces a valid closed manifold mesh, used as a preprocessing step ahead of
   downstream SDF pipelines
   ([GitHub](https://github.com/hjwdzh/ManifoldPlus)). **`elalish/manifold`** is the
   actively-developed general-purpose alternative, with a native Rust port
   (`manifold-rust`) as of 2024-2025, "always returns a valid, watertight, oriented
   2-manifold with no precision crashes on degenerate input"
   ([GitHub](https://github.com/elalish/manifold)) — useful if you specifically need to
   union multiple glTF parts into one solid before baking.
2. **Bake directly against the dirty mesh using GWN**, skipping repair entirely — the
   approach the Rust `mesh_to_sdf` crate exposes as an alternative sign mode (below).

## Rust tooling

- **`gltf-rs/gltf`** — the canonical Rust glTF 2.0 parser (MSRV 1.61+), with a `utils`
  feature exposing typed reader convenience methods (`read_positions()`,
  `read_indices()`) and `Node::transform()`'s `.matrix()`/`.decomposed()` for correct
  world-transform composition (see above). Supports a substantial list of Khronos
  extensions. ([GitHub](https://github.com/gltf-rs/gltf))
- **`Azkellas/mesh_to_sdf`** — the most actively maintained Rust-native mesh-to-SDF crate.
  Two sign modes: **Raycast** (default, robust but requires a watertight mesh) and
  **Normal** ("uses the normals of the triangles to estimate the sign... works for
  non-watertight meshes but might leak negative distances outside the mesh" — its own
  docs are explicit about this tradeoff). Notably, it does **not** implement GWN — for
  arbitrarily dirty input, pairing it with a repair pass (above) or writing a custom GWN
  sign pass is still on the integrator. Acceleration via BVH/R-tree; `generate_grid_sdf`
  is the relevant entry point for a dense-volume bake; optional `serde` support for
  saving/loading the baked grid. Ships a `mesh_to_sdf_client` example that loads a
  glTF/GLB directly and visualizes its baked SDF — the closest existing end-to-end
  reference for this exact pipeline, though it doesn't itself handle node-hierarchy
  transform composition, skinning, or repair.
  ([GitHub](https://github.com/Azkellas/mesh_to_sdf))
- No dedicated Bevy plugin for "glTF → baked SDF grid" exists as of this research —
  `bevy_sdf_klown`, `ALICE-SDF`, and `rust-gpu-sdf` are all procedural-composition-
  oriented (some can bake a *procedural* SDF to a mesh/VDB, the opposite direction) rather
  than mesh-import-oriented. Assembling this pipeline currently means combining `gltf`
  (parsing + transform composition) with `mesh_to_sdf` or a custom voxelizer, with
  repair/GWN robustness layered in by hand for untrusted input.

## Mesh simplification before baking

Since a baked grid can't represent detail finer than its voxel size regardless of source
triangle density, decimating a high-poly source mesh *before* baking is standard practice
— every triangle beyond what the target grid resolution can resolve is pure wasted
preprocessing cost (BVH build, per-voxel nearest-triangle queries, GWN tree cost).

- **`zeux/meshoptimizer`**'s `meshopt_simplify` (topology-preserving, quadric-error-metric
  edge collapse — Garland & Heckbert) is the standard tool, safer ahead of a
  watertightness-sensitive bake since it won't introduce new holes. Its more aggressive
  `meshopt_simplifySloppy` variant *can* merge topologically-disjoint-but-spatially-close
  features — defensible specifically because GWN-based sign determination tolerates
  topological sloppiness that a stricter pipeline couldn't.
  ([GitHub](https://github.com/zeux/meshoptimizer))
- **`donmccurdy/glTF-Transform`** (JS CLI, run as an offline asset-prep step, not at
  runtime) packages exactly the glTF-specific cleanup a pre-bake pass needs: `weld`
  (merge near-duplicate vertices — directly fixes "separate parts not sharing a seam"),
  `dedup`, `join` (merge compatible primitives), `simplify` (meshoptimizer-backed), and
  `prune`. Documented pipeline order: `dedup → join → weld → simplify → prune`.
  ([CLI docs](https://gltf-transform.dev/cli))

## Asset-pipeline pattern: bake at import time, cache the result

Unreal's Mesh Distance Field generation is the reference architecture for "bake once,
cache, reuse": generation is explicitly **offline only** — "cannot be done at runtime" —
happening as part of the Static Mesh build/import process, with per-asset resolution
control and a hard cap ("the maximum size volume texture any single mesh can have is 8
megabytes with a resolution of 128×128×128")
([Unreal Engine docs](https://dev.epicgames.com/documentation/en-us/unreal-engine/mesh-distance-fields-in-unreal-engine)).
The baked field is stored once per unique mesh asset and reused across every instance via
a transform — see [hybrid-baked-and-procedural-scenes](./hybrid-baked-and-procedural-scenes.md)
for how that per-instance transform is applied at query time rather than by re-baking.

The general pattern worth replicating for any glTF import pipeline: cache the baked grid
keyed on **source-asset content hash + bake settings** (resolution, up-axis convention,
decimation ratio, sign method), invalidate only when those inputs change, and never
re-bake at runtime. For on-disk storage, options span from OpenVDB (sparse, standard for
narrow-band fields, cross-tool interop with Houdini/Blender) to a dense 3D texture (Unity
VFX Graph bakes `RHalf Texture3D`s normalized to the bounding box) to a bespoke
serde-serialized grid (`mesh_to_sdf`'s own approach) — see
[efficient-grid-baking](./efficient-grid-baking.md) for the resolution/compression
tradeoffs behind that choice.

## When to dive in

- Writing a glTF-to-SDF import pipeline in Rust from scratch → start with `gltf-rs/gltf`
  for parsing + transform composition, then `mesh_to_sdf` (or a custom baker, see
  [efficient-grid-baking](./efficient-grid-baking.md)) for the actual grid generation.
- Baking assets of unknown/untrusted quality (downloaded, AI-generated, scanned) → assume
  they need repair or GWN-based sign determination; don't default to ray-casting sign
  determination, which silently fails on the defects listed above.
- Seeing inside-out geometry in only part of an imported scene, not the whole thing →
  check for negative/mirrored-scale nodes first (see the winding-flip note above) before
  suspecting a sign-determination algorithm bug.
- Import is slow or memory-heavy for a single asset → decimate before baking
  (meshoptimizer/glTF-Transform), rather than reaching for a coarser bake resolution
  first — triangle count beyond what the target grid can resolve is pure waste.

## Related
- [Sign determination](./sign-determination-methods.md) — prerequisite: GWN for the defective meshes listed here.
- [Efficient grid baking](./efficient-grid-baking.md) — deeper: the bake step after import.
- [Converting GLB scenes to SDF BSN scenes](../glb-to-bsn-conversion.md) — contrast: the (dropped) primitive-fitting alternative to baking.
- [Parse the asset, don't transcribe it](../../engineering-practice/testing/parse-the-asset-dont-transcribe-it.md) — same-trap: migera parses `puppet_base.gltf` in tests instead of hand-copying numbers.
