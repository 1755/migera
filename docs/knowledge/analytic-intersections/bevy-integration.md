---
title: Bevy 0.19.1 primitives & integration (source-verified)
description: Records that bevy_math 0.19.1's shape primitives (Sphere, Cuboid, Capsule3d, Torus...) carry no ray intersectors, only the bounding layer's slab/sphere tests, notes picking backends, and keeps the (built, then removed) integration path for exact intersectors. Read before adding exact picking or collision queries.
type: reference
status: current
tags:
  - bevy
  - primitives
  - ray-tracing
  - integration
updated: 2026-09-28
verified: 2026-08-23
sources:
  - bevy_math 0.19.1 src/primitives/dim3.rs
  - bevy_math 0.19.1 src/bounding/raycast3d.rs
  - commit dcf8a70
aliases:
  - bevy_math primitives
  - Ray3d
  - bevy_picking
---

# Bevy 0.19.1 primitives & integration (source-verified)

## What ships in `bevy_math` 0.19.1

`bevy_math/src/primitives/dim3.rs` (verified structs):

- `Sphere { half_size }` (radius as half_size.x), `Cuboid { half_size }`,
  `Cylinder { half_height, radius }`, `Capsule3d { half_height, radius }`,
  `Cone { half_height, radius }` (apex up), `Torus { minor_radius, torus_radius }`,
  plus `Plane3d`, `InfinitePlane3d`, `Segment3d`, and 2D counterparts.
- These are *shape descriptions* for gizmos/meshes/colliders - they do NOT carry
  intersector implementations. There is no built-in exact ray-quadric solver in Bevy;
  the slab test lives only in the bounding layer.

Ray support:

- `Ray3d { origin, direction: Dir3 }`; `RayCast3d` adds precomputed `direction_recip`
  + max distance with `aabb_intersection_at` / `sphere_intersection_at`
  (see aabb-acceleration knowledge). Pattern to imitate for our own intersectors:
  precompute per-ray constants once, store on the ray struct.
- `bevy_math::bounding::{Aabb3d, BoundingSphere}` for volume pairing.

## Picking backends (context)

`bevy_picking`'s mesh backend raycasts triangles; there is no analytic-primitive picking
backend shipped. If UI-picking of SDF entities is ever needed, our exact intersectors
would slot in as a custom picking backend - same math, CPU-side, few rays/frame.

## Integration path for migera

Historical: steps 1-3 were built in the legacy hybrid renderer and removed in commit
dcf8a70 (2026-08-30); see [the production finding](./production-performance-finding.md).
Step 4 (CPU picking/editor queries) is the remaining plausible use.

1. **WGSL first**: port iq's minimals for OUR leaf set (sphere, rounded box, torus,
   capsule, rounded cylinder->cylinder+capsule composition, ellipsoid, hex prism,
   box frame) into a shared wgsl library next to material.wgsl, returning vec2(tN,tF).
   Records already carry params + isometry; add an `exact_ok` flag per leaf.
2. **Rust mirror for tests**: implement the same formulas over `bevy_math` primitives
   (`Sphere`, `Cuboid`, ...) so golden tests can compare WGSL semantics vs Rust exactly
   (same convention as flatten.rs's eval_stack mirror).
3. **Hybrid marcher** (the actual payoff): per subtree, if all leaves exact AND no
   smooth operators -> interval CSG path (single-hit state machine); blended subtrees
   keep sphere tracing inside their bounds. Dispatch choice per ray segment, not per
   frame.
4. **Where it runs**: GPU for rendering rays; CPU via RayCast3d-style helpers only for
   editor/picking queries (few rays).

## Versioning note

Struct fields above verified against bevy_math-0.19.1 sources in the cargo registry;
re-check on Bevy upgrades (primitives module has churned historically).

## Related
- [AABB primitives in Bevy 0.19.1](../aabb-acceleration/bevy-aabb-primitives.md) — deeper: `RayCast3d`'s slab test, the only ray/shape test Bevy ships.
- [Exact ray-primitive intersectors - catalog with costs](./primitive-intersection-catalog.md) — applies: the formulas step 1 would port.
- [CSG on exact intersections](./csg-intervals.md) — applies: the interval-CSG path step 3 describes.
- [The analytic tier cost 5-6x SDF marching and was removed](./production-performance-finding.md) — superseded-by: why steps 1-3 no longer run in the renderer.
