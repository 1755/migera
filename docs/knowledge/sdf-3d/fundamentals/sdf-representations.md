---
title: The four ways an SDF can be stored/evaluated
description: Classifies every SDF technique into procedural/analytic, sampled grid (dense or sparse), hybrid (converted to points/mesh for rendering) and neural, with storage, per-query cost and editability tradeoffs plus a choose-by-need table. Read when choosing how to represent SDF content.
type: concept
status: current
tags:
  - sdf
  - baking
  - mesh-conversion
  - performance
updated: 2026-08-15
aliases:
  - SDF representation families
  - voxel SDF
  - procedural SDF
---

# The four ways an SDF can be stored/evaluated

Every 3D SDF technique falls into one of four representation families. Understanding
which family a given tool or engine uses explains most of its performance and quality
tradeoffs at a glance.

## 1. Procedural (closed-form / analytic)

The SDF is literal code: a function that, given a point, evaluates a closed-form formula
(possibly a tree combining many primitives via CSG operators). See
[primitives-and-operators](../primitives-and-operators/INDEX.md).

- **Storage**: effectively zero — just the expression tree/shader code.
- **Resolution**: infinite/exact at any zoom level (for exact primitives).
- **Cost**: proportional to expression complexity per query; scenes with many combined
  primitives get expensive to evaluate per-pixel, per-step.
- **Editability**: excellent — parametric, easy to animate (parameters are just
  variables), easy to combine.
- **Typical use**: demoscene/shader art, procedural modeling tools (see
  [state-of-the-art](../state-of-the-art/sdf-modeling-tools.md)), small-to-medium hand-
  authored scenes, individual object representations in hybrid engines.

## 2. Sampled grid (dense or sparse voxel volume)

The SDF is precomputed at regular grid points and stored in a 3D texture/array; values
between grid points are reconstructed by trilinear (or higher-order) interpolation.

- **Storage**: scales with `resolution^3` for a dense grid — this is the dominant cost
  driver. A modest `256^3` volume at 8-bit precision is 16 MB; doubling resolution in
  each axis multiplies storage by 8x.
- **Resolution**: fixed by the grid; interpolation smooths between samples but cannot
  recover detail finer than the grid spacing without artifacts (aliasing/blockiness).
- **Cost**: O(1) per query (one texture fetch + interpolation) regardless of scene
  complexity — the key advantage over procedural fields for complex geometry.
- **Sparse variants**: since most of 3D space in a typical scene is either deep interior
  or far exterior (where exact value barely matters — see
  [narrow-band-sdfs](./narrow-band-and-truncated-sdfs.md)), sparse structures (octrees,
  voxel hashing, clipmaps, DAGs — see
  [sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md))
  store detail only near the surface and dramatically cut memory versus a dense grid.
- **Typical use**: baked mesh distance fields (Unreal's Mesh Distance Fields/Global
  Distance Field), voxel terrain/destructible worlds (Claybook), large static scenes
  where per-object procedural evaluation would be too slow.

## 3. Point-based / hybrid (SDF converted to a different render representation)

The SDF is used as an intermediate authoring/simulation representation, but is converted
— once, or per-frame — into another representation for actual rendering: a point cloud, a
mesh (via marching cubes/dual contouring, see
[sdf-to-mesh-extraction](../mesh-conversion/sdf-to-mesh-extraction.md)), or a set of
Gaussian splats.

- **Storage/cost**: depends entirely on the target representation, not the SDF itself.
- **Typical use**: Media Molecule's *Dreams* is the canonical example — geometry is
  authored and physically simulated as CSG trees over SDFs, then tessellated into dense
  multi-resolution point clouds for actual GPU rendering (no triangle rasterization of
  the SDF itself). See
  [production-case-studies](../performance-and-production/production-case-studies.md).
  This pattern separates "SDF as an authoring/simulation-friendly representation" from
  "fastest thing to actually rasterize," picking the best representation for each job
  rather than forcing raymarching to do both.

## 4. Neural / learned (implicit neural representation)

The SDF is the output of a trained neural network (typically an MLP) mapping 3D
coordinates to a distance value, optionally conditioned on a latent code to represent a
family of shapes from one network (DeepSDF) or fit to a single scene (SIREN, NeuS-style
approaches).

- **Storage**: network weights — can be far more compact than a dense grid for complex,
  detailed geometry, and can represent continuous, unbounded-resolution shapes.
- **Resolution**: theoretically continuous/infinite, but practically limited by network
  capacity — fine high-frequency detail requires either large networks, positional
  encoding, or periodic activations (SIREN) to avoid the "spectral bias" of naive MLPs
  toward low-frequency functions.
- **Cost**: a forward pass through the network per query — historically far more
  expensive per-query than a grid lookup, though 2024-2026 research increasingly targets
  real-time neural SDF evaluation (see
  [neural-and-learned-sdfs](../state-of-the-art/neural-and-learned-sdfs.md)).
- **Eikonal enforcement**: usually only a soft training loss term, not a hard guarantee —
  so gradients away from the surface can be unreliable, which matters if the network's
  output is used for raymarching step sizes rather than just surface classification.
- **Typical use**: shape generation/completion, 3D reconstruction from images/point
  clouds (often alongside NeRF-style radiance fields, e.g. NeuS), compact asset storage
  for ML pipelines.

## How to choose

| Need | Representation |
|------|-----------------|
| Hand-authored, animatable, exact-at-any-zoom shapes | Procedural |
| Complex static geometry (baked from a mesh), O(1) query cost | Sampled grid (dense or sparse) |
| SDF as an authoring/physics tool, but need fast triangle/point rendering | Hybrid (convert to mesh/points) |
| Learned generative shapes, reconstruction from scans/images, compact storage of complex detail | Neural |

Most production systems combine more than one: e.g. Unreal generates per-mesh sampled-
grid SDFs offline, then composites them procedurally at runtime into a sparse Global
Distance Field clipmap for scene-wide software ray tracing (see
[unreal-lumen-distance-fields](../state-of-the-art/unreal-lumen-distance-fields.md)).

## When to dive in

- Choosing a representation for a new project → match against the table above, and read
  the linked topic for the chosen representation before implementing.
- Storage/memory blowing up with a voxel SDF → read
  [narrow-band-and-truncated-sdfs](./narrow-band-and-truncated-sdfs.md) and
  [sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md)
  before assuming dense grids are the only option.
- Considering neural SDFs for a production real-time renderer → read
  [neural-and-learned-sdfs](../state-of-the-art/neural-and-learned-sdfs.md) first; the
  per-query cost and Eikonal-guarantee tradeoffs are significant and evolving fast.

## Related
- [What is a signed distance field?](./what-is-an-sdf.md) — prerequisite: the definition all four families share.
- [Narrow-band and truncated SDFs](./narrow-band-and-truncated-sdfs.md) — deeper: the truncated special case of the grid family.
- [Primitive shapes](../primitives-and-operators/primitive-shapes.md) — example: the procedural family, which migera's `src/sdf` uses.
- [Production case studies](../performance-and-production/production-case-studies.md) — example: Dreams (hybrid) and Claybook (grid) in shipped games.
- [Neural and learned SDFs](../state-of-the-art/neural-and-learned-sdfs.md) — deeper: the neural family in detail.
