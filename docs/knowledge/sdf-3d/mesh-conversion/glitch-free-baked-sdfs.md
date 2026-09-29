---
title: "Producing glitch-free baked SDFs: artifacts, causes, fixes"
description: Catalogs baked-grid SDF failure modes — sub-voxel feature loss, trilinear interpolation breaking the Lipschitz bound (fix: step damping), three sign-flip appearances, chunk seams, staircasing — and four QA checks, gradient magnitude first. Read before shipping a baked SDF or when one pits or renders inside-out.
type: guide
status: current
tags:
  - sdf
  - baking
  - raymarching
  - troubleshooting
  - verification
  - correctness
updated: 2026-08-16
aliases:
  - bake artifacts
  - sign flip
  - thin feature loss
  - gradient magnitude check
---

# Producing glitch-free baked SDFs: artifacts, causes, fixes

Contents: [1 Thin features](#1-thin-feature-loss) ·
[2 Interpolation safety](#2-interpolation-artifacts-and-sphere-tracing-safety) ·
[3 Sign flips](#3-sign-flip-artifacts-what-they-actually-look-like-raymarched) ·
[4 Chunk seams](#4-seams-at-chunktile-boundaries) ·
[5 Staircasing](#5-aliasing-and-staircasing-on-baked-curved-surfaces) ·
[6 Validation/QA](#6-validationqa-before-shipping-a-bake) · [Related](#related)

A baked, sampled-grid SDF is only *safe* to sphere-trace if it behaves like the exact
analytic fields [exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md) describes —
this document catalogs the specific ways a mesh-to-grid bake can violate that safety, what
each failure actually looks like on screen (not just in the abstract), and how to detect
a bad bake before shipping it.

## 1. Thin-feature loss

A grid SDF only records information at lattice points. A feature narrower than roughly
one voxel has no sample point landing "inside" it — every surrounding voxel corner
samples exterior, so trilinear reconstruction never crosses zero there, and the feature
vanishes entirely. This is a direct Nyquist-style sampling argument: reconstructing a
spatial feature of width `w` without aliasing it away requires a sampling interval
(voxel size) on the order of `w/2` or smaller.

Production tooling generally does **not** automatically detect or warn about this —
Unreal's own Mesh Distance Field docs show a documented before/after ("Resolution is too
low, important features are lost") but the guidance is purely manual look-and-adjust, not
a preflight check. **RTSDF** gives concrete numbers demonstrating the problem doesn't
fully go away with "more resolution," only asymptotically improves at rapidly increasing
cost: at 64³, "jump flooding is unable to capture some surfaces of thin objects,"
producing visibly "disjointed and incomplete" shadows; 128³ was the minimum the authors
found viable; 256³-512³ "reduces hole size but doesn't eliminate them," with 500³ costing
70ms/frame — thin-feature loss is a genuine resolution-vs-cost tradeoff, not a bug with a
clean fix ([arXiv:2210.06160](https://ar5iv.labs.arxiv.org/html/2210.06160)). RTSDF's own
practical mitigation: add a fixed positive bias (0.01 units) that uniformly thickens
every surface before storing it — blunt, but it reliably prevents thin geometry from
disappearing, at the cost of slightly bloating everything.

## 2. Interpolation artifacts and sphere-tracing safety

Trilinear interpolation between grid samples reconstructs a piecewise-polynomial
*approximation*, not the exact field — and this reconstruction is **not guaranteed to
preserve the Eikonal/unit-gradient property** (`|∇f|=1`) that makes a raw field value
safe to step by. A 2026 paper states this precisely: "generic polynomial interpolation
methods produce outputs that violate the most basic mathematical properties of SDFs...
eikonality ensures that sphere tracing algorithms do not overshoot," showing concretely
that naive trilinear/tricubic interpolation can produce values that "contradict the
inputs, in the form of intersecting opposite-sign spheres (violating eikonality)"
([Chen, Mupparaju, Batty, Sellán, Stein, "Greed for the Spheres," arXiv:2605.01919](https://arxiv.org/html/2605.01919v1)).

This is the **exact same class of problem** already documented in
[exact-vs-bound-sdfs](../fundamentals/exact-vs-bound-sdfs.md) for smooth-min/smooth-max
CSG blending — a construction that's a safe bound almost everywhere can have local
regions (here: near voxel boundaries and around thin/high-curvature features) where the
reconstruction locally overestimates true distance, violating the Lipschitz-1 guarantee
sphere tracing depends on. A ray stepping by the raw interpolated value in such a region
can step *through* thin geometry or a sharp corner it should have detected — punch-
through in the worst case, pitting/banding in milder cases (directly analogous to the
smin-blend-region pitting this project's own raymarcher has already had to fix — see
[raymarching-artifacts-and-fixes](../rendering/raymarching-artifacts-and-fixes.md)).

**The standard, cheap fix is the same "scale your step" principle already used for
non-exact CSG blends**: multiply the interpolated value by a conservative damping factor
(commonly 0.8-0.95) before stepping, trading extra march iterations for guaranteed
safety against local Lipschitz violations in the interpolated field. This requires no
change to the baking process, only to the marching loop.

Two more rigorous (and more expensive) research-grade alternatives exist, not yet
standard production practice: **exact ray/trilinear-patch intersection** — solving
directly for where a ray crosses the trilinear surface within each voxel instead of
marching through it, sidestepping overstepping entirely at the cost of a specialized
intersection routine rather than a drop-in step-loop change
([Hansson Söderlund, Evans, Akenine-Möller, "Ray Tracing of Signed Distance Function
Grids," JCGT 2022](https://research.nvidia.com/publication/2022-09_ray-tracing-signed-distance-function-grids)) —
and **consistency-preserving interpolation**, constructing interpolated values
provably consistent with all input samples so no downstream step-damping is needed at
all (the "Greed for the Spheres" paper above, May 2026, bleeding-edge academic work).
**For most projects, the cheap step-damping fix — already proven in this project's own
smooth-CSG handling — is the right default**; reach for the exact-intersection approach
only if profiling shows damping's extra iterations are unacceptable.

## 3. Sign-flip artifacts: what they actually look like raymarched

[sign-determination-methods](./sign-determination-methods.md) covers the algorithms and
their mesh-side failure conditions; this section covers the on-screen symptom, since
recognizing one is the fastest way to diagnose it.

- **Global sign inversion** (interior/exterior swapped everywhere): a ray from outside
  sees the field reporting large positive "safe" distances even while inside solid
  geometry — the object becomes effectively invisible, the ray marches straight through.
  A camera placed inside an accidentally-inverted mesh (e.g. a room) sees the interior
  surfaces behave like exterior surfaces of a solid — the world reads as turned
  inside-out, the camera embedded in solid matter. This is loud and unmistakable, not
  subtle.
- **Localized sign errors near sharp/non-manifold features**: patchier and easier to
  misdiagnose. Occurs when the closest vertex to a query point is shared by triangles
  facing different directions, particularly at sharp edges — the wrong triangle gets
  picked as "closest," giving the wrong sign right there. Raymarched, this shows as
  small notches carved right at an edge (a flipped pocket reading as "outside"), or as
  small isolated floating blobs a few voxels wide detached from the main surface (a
  flipped pocket reading as "inside" nearby). The distinguishing tell versus normal-
  estimation noise (which also concentrates near sharp/high-curvature regions, see
  [normal-estimation](../rendering/normal-estimation.md)): sign-flip artifacts are
  **static** — they don't flicker or shimmer as the camera moves, because they're baked
  geometry-level holes/floaters, not a per-frame shading computation.
- **"Leaking" through non-watertight seams**: a hole or gap in the source mesh makes
  ray-parity-based sign determination miscount crossings near it. On screen: the object
  looks like a solid, normal surface from most angles, but grazing rays or reflections
  passing near the leak punch through into what should be solid interior — a visible
  seam-shaped dark crack or an unexpected view "inside" the object exactly where the
  source mesh had its hole. This is the single most common root cause of "inside-out"
  bake results in practice, and generalized winding number is the standard fix precisely
  because it evaluates a smooth, globally-integrated function that a single local hole
  can't corrupt (see [sign-determination-methods](./sign-determination-methods.md) and
  [gltf-import-pipeline](./gltf-import-pipeline.md#real-world-mesh-quality-problems)).

## 4. Seams at chunk/tile boundaries

When a large SDF is baked as independent chunks (for streaming/memory reasons) rather
than one monolithic volume, each chunk's interpolation and gradient stencils only have
access to *its own* corner samples. Without shared context across the boundary, either
side clamps/extrapolates incorrectly right at the seam — producing a visible crack,
z-fighting doubled geometry, or a lighting discontinuity exactly at tile grid lines. A
sphere tracer crossing such a boundary can also see a discontinuous jump in reported
distance — the same class of Lipschitz-safety risk as §2, but localized to the seam
rather than the whole volume.

The standard fix is **padding chunks with an overlap border ("ghost cells"/"halo")
sampled consistently with what the neighboring chunk would produce at the same world-
space positions** — each chunk should be baked against the full source mesh (or geometry
extending some margin past its own bounds), not independently approximated at its edge.
The overlap width needed is tied directly to the interpolation/gradient stencil's own
reach: at minimum 1 voxel for trilinear continuity, more for a wider central-difference
normal stencil.

This project's own `Node::Repeat`/`repeat_xz` domain-repetition operator achieves a
related but distinct guarantee — seamless infinite tiling — by remapping world-space
coordinates into one canonical cell rather than baking multiple independent tiles (see
the raymarch shader's own module doc). A chunked mesh-SDF bake is the harder, general
case of the same seamlessness problem without that single-canonical-cell trick, since
chunk content generally isn't identical from tile to tile the way procedural repetition
is.

## 5. Aliasing and staircasing on baked curved surfaces

Every primitive an exact analytic raymarcher evaluates directly (sphere, box, torus,
cylinder — see [primitives-and-operators](../primitives-and-operators/INDEX.md)) is
infinitely sharp and resolution-independent. A baked mesh SDF trades that away: curvature
is only ever known at discrete lattice points, reconstructed by (typically) trilinear
interpolation — a piecewise-linear-per-axis approximation of what's usually a smoothly
curving true surface. This produces the same "corners round off as resolution drops"
effect noted in [efficient-grid-baking](./efficient-grid-baking.md#choosing-grid-resolution-and-bounds) —
the opposite visual direction from staircasing, but the identical root cause
(insufficient sample density relative to true curvature/sharpness).

**Mitigations**: bicubic/tricubic interpolation trades a wider sample stencil for a
smoother, higher-order-continuous reconstruction — but note "Greed for the Spheres"
explicitly lists tricubic interpolation among the methods that are *smoother without
being safe* (§2) — visual smoothness and sphere-tracing safety are separate properties,
improving one doesn't guarantee the other. Adaptive/octree resolution (see
[sparse-and-hierarchical-structures](../performance-and-production/sparse-and-hierarchical-structures.md))
subdivides finer only where curvature demands it, avoiding uniform fine-resolution cost
everywhere. If extracting a mesh back out of a baked SDF, dual contouring (and Dual
Marching Cubes specifically) preserve sharp features that plain marching cubes cannot,
since marching cubes constrains output vertices to lie on grid cell edges while dual
contouring can place a vertex anywhere within a cell — see
[sdf-to-mesh-extraction](./sdf-to-mesh-extraction.md).

**Normal/shading quality degrades faster than raw distance accuracy as resolution
drops** — because normals are a *derivative* of the field, differentiation amplifies the
interpolation/quantization noise already present in the underlying distance values.
Finite-difference normals computed from a coarse grid "are not continuous across voxels
... and can change directions abruptly, resulting in images with abrupt changes in
lighting on surfaces that should appear smooth." The JCGT ray-tracing-grids paper (§2)
treats "continuous normals across voxels" as a distinct contribution worth solving
*separately* from exact position/intersection accuracy — evidence that shading
continuity is a harder problem than raw geometric accuracy at a given resolution, not
something that's automatically fixed once position accuracy is adequate.

## 6. Validation/QA before shipping a bake

- **Gradient-magnitude checking (`|∇f| ≈ 1`)** — the single most direct, fully
  automatable correctness check. Numerically differentiate the baked grid at many sample
  points (the same finite-difference technique used for shading normals, see
  [normal-estimation](../rendering/normal-estimation.md)) and flag regions where `|∇f|`
  deviates substantially from 1: much greater than 1 means a Lipschitz-bound violation
  (directly dangerous for sphere tracing, per §2); much less than 1 typically indicates
  over-smoothed/flattened regions from excessive interpolation or bias-padding. Runs as a
  batch postprocess over the whole volume, independent of any camera view — strictly more
  complete than eyeballing a render.
- **Watertightness/manifoldness pre-checks on the source mesh, before baking at all** —
  catching a sign-determination failure (§3) at its actual root cause is cheaper than
  debugging the baked-volume symptom later. Tools: Open3D's `is_watertight()`/
  `is_self_intersecting()`, or Axom's Quest component. Gate the pipeline on this check:
  route known-good watertight meshes to the cheaper ray-casting/pseudonormal sign method,
  fall back to GWN automatically for anything that fails — see
  [sign-determination-methods](./sign-determination-methods.md)'s own decision table.
- **Visual diffing against the source mesh's rendered silhouette** — render the source
  mesh directly (rasterized ground truth) and the baked SDF via the actual production
  raymarcher, from the same fixed camera angles, and diff. Advantage over gradient
  checking: exercises the entire production pipeline end to end (bake + step-safety +
  normal estimation together), catching things a pure numerical field check might miss —
  e.g. a sign error that's locally Eikonal-consistent but geometrically wrong.
- **Round-trip testing** — re-extract an explicit mesh from the baked SDF (marching
  cubes/dual contouring) and compare against the original source mesh via Hausdorff
  distance ("the largest gap between the original and the reconstructed geometry") or
  Chamfer distance/normal consistency for a fuller picture. The most expensive of the
  four techniques (a full extraction pass) but the most direct ground-truth comparison —
  suited to a slower offline/CI gate rather than a fast interactive check.

## When to dive in

- A baked object is missing thin parts (blades, wires, thin walls) → this is §1, a
  resolution-vs-cost tradeoff, not a bug; either raise resolution for that asset
  specifically or apply a small uniform surface-thickening bias at bake time.
- Seeing pitting, banding, or occasional punch-through on a baked (not purely procedural)
  surface → check whether step-damping is applied to grid-sampled distances (§2) before
  assuming it's a bake-quality bug.
- An imported mesh renders inside-out, has floating fragments, or shows a crack/seam
  where solid geometry should be → match the symptom against §3's three sign-failure
  patterns to identify which one you're looking at before changing the sign algorithm.
- Streaming/chunked baking shows visible seams at tile boundaries → check chunk overlap
  padding (§4) before suspecting the sign or distance algorithm itself.
- Setting up a bake pipeline that needs to run unattended (CI, batch import) → implement
  at least the gradient-magnitude check (§6) as an automated gate; it's the cheapest and
  most complete of the four validation techniques.

## Related
- [Exact vs. bound distance fields](../fundamentals/exact-vs-bound-sdfs.md) — prerequisite: the safety contract a bake must keep.
- [Sign determination](./sign-determination-methods.md) — deeper: the algorithms behind §3's sign flips.
- [Raymarching artifacts and fixes](../rendering/raymarching-artifacts-and-fixes.md) — contrast: the same symptoms from procedural fields.
- [Integrating a baked mesh SDF into this project's raymarcher](./hybrid-baked-and-procedural-scenes.md) — applies: why migera's smin seams against a baked leaf need extra care.
- [Domain operations](../primitives-and-operators/domain-operations.md) — contrast: seamless procedural repetition vs. §4's chunk seams.
