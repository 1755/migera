---
title: "Sign determination: is a point inside or outside the mesh?"
description: Compares four inside/outside methods for mesh-to-SDF — ray-cast parity, angle-weighted pseudonormals, generalized winding number (robust default for bad meshes), depth rasterization — plus the heat method. Read when baking untrusted meshes or when a bake comes out inside-out.
type: concept
status: current
tags:
  - sdf
  - mesh-conversion
  - baking
  - correctness
updated: 2026-08-15
aliases:
  - generalized winding number
  - GWN
  - pseudonormal
  - watertight mesh
---

# Sign determination: is a point inside or outside the mesh?

Computing the *unsigned* distance to a mesh (see
[exact-point-to-mesh-distance](./exact-point-to-mesh-distance.md)) is only half the
problem — an SDF also needs to know which side of the surface a query point is on. This
turns out to be the harder and more failure-prone half in practice, because real-world
meshes are frequently **not** the clean, watertight, consistently-oriented manifolds the
simplest sign methods assume.

## Method 1: ray casting / parity counting

Cast a ray from the query point in an arbitrary direction and count how many times it
crosses the mesh surface. An odd number of crossings means the point is inside; even
means outside (the classic point-in-polygon test generalized to 3D).

- **Pro**: simple, intuitive, cheap per query given a spatial acceleration structure for
  ray-triangle intersection.
- **Con**: fails on meshes with holes (non-watertight geometry), and is sensitive to
  degenerate ray-edge/ray-vertex intersections requiring careful numerical handling.
  Results can also depend on the chosen ray direction near problematic mesh regions,
  motivating multi-ray voting (casting several rays and taking a majority vote) as a
  common robustness improvement.

## Method 2: angle-weighted pseudonormals

At each vertex and edge of the mesh, a "pseudonormal" is precomputed as a weighted
average of adjacent face normals (weighted by the subtended angle at that vertex/edge).
For a query point, find the closest point on the mesh (vertex, edge, or face interior)
and compare the direction from that closest point to the query point against the
appropriate pseudonormal (face normal if closest point is face-interior, the precomputed
pseudonormal if closest point is a vertex or edge) — a positive dot product means outside,
negative means inside.

- **Pro**: gives a *consistent* sign for the exact nearest-point query already being
  computed for the unsigned distance, without a separate ray-casting step; behaves well
  even very close to sharp features.
- **Con**: still assumes a manifold, consistently-oriented mesh to produce meaningful
  pseudonormals in the first place; degrades on non-manifold or inconsistently-wound
  input.

## Method 3: generalized winding number (GWN)

The generalized winding number for an arbitrary (not-necessarily-watertight, not-
necessarily-consistently-oriented) triangle mesh is defined as the sum of signed solid
angles subtended by each triangle as seen from the query point. This produces a smooth
scalar function whose value is close to `1` for points solidly inside a coherent mesh
region and close to `0` for points solidly outside — even when the mesh has holes,
self-intersections, non-manifold edges, or locally inconsistent winding.

- **Pro**: by a wide margin the most robust method against imperfect, "in the wild" mesh
  data — this robustness is the specific problem GWN was designed to solve, and it has
  become the standard choice in modern mesh-to-SDF tooling and in boolean-operation
  algorithms built on similar principles (e.g. "Boolean Operations using Generalized
  Winding Numbers").
- **Con**: more expensive to evaluate per query than a plain ray cast or pseudonormal
  lookup (summing solid angles over all triangles, though this is also accelerable with
  spatial hierarchies analogous to the distance BVH/octree). Recent work (e.g. the
  "Antipodal Method") specifically targets faster, more robust GWN evaluation, indicating
  this remains an active area of algorithmic improvement rather than a fully solved
  problem.

## Method 4: depth-based (rasterization) sign determination

A GPU-friendly alternative: rasterize the mesh from several directions and use depth-
buffer parity/comparison to classify points as inside or outside relative to what's been
rasterized. Faster and more GPU-native than ray casting or GWN summation, but — like
ray casting — does not handle meshes with holes correctly, since a hole in the geometry
means no depth information exists to classify points behind it.

## Choosing a method

| Mesh quality | Recommended method |
|---|---|
| Clean, watertight, consistently-oriented (e.g. CAD export, careful game-asset authoring) | Ray casting or pseudonormals — cheaper, and correctness assumptions hold |
| Uncertain quality, scanned/reconstructed, known holes or self-intersections | Generalized winding number — the robustness is worth the extra cost |
| GPU-driven bulk baking pipeline, quality-tolerant | Depth-based rasterization sign, if mesh is known watertight |

## The heat method: a related but distinct alternative

A separate technique (the "heat method for generalized signed distance") computes both
distance and sign together via a diffusion-based (heat-equation) approach, offering
robustness comparable to GWN but producing a genuine distance function directly rather
than a separate sign-classification step layered on top of an unsigned distance query.
This represents a different algorithmic family from all four methods above (which all
separate "find nearest point/unsigned distance" from "determine sign") and is an active
area in geometry-processing research.

## When to dive in

- Baking an SDF from meshes of known-good quality (clean, watertight, single source) →
  ray casting or pseudonormals are sufficient and cheaper; don't reach for GWN by default.
- Baking SDFs from arbitrary/untrusted/scanned mesh input at scale (e.g. a general-
  purpose asset pipeline) → use generalized winding number as the default; the robustness
  is worth the extra cost, and debugging sign errors from a fragile method on bad input
  meshes is usually more expensive than the GWN computation itself.
- Seeing inverted normals or "inside-out" SDF results after baking → check mesh winding
  consistency and watertightness first; this is the single most common root cause, and
  switching to GWN (rather than debugging the mesh) is often the pragmatic fix.

## Related
- [Exact point-to-mesh distance](./exact-point-to-mesh-distance.md) — prerequisite: the unsigned half.
- [Glitch-free baked SDFs](./glitch-free-baked-sdfs.md) — applies: what a sign flip looks like when raymarched.
- [glTF import pipeline](./gltf-import-pipeline.md) — applies: real-world glTF mesh defects that break naive sign methods.
