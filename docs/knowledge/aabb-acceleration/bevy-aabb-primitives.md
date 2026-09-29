---
title: AABB primitives in Bevy 0.19.1 (source-verified)
description: Distinguishes bevy_math's Aabb3d/RayCast3d (math layer, branchless slab test) from bevy_camera's Aabb ECS component used by frustum culling, notes Bevy's GPU occlusion-culling precedent, and records the six-scalar-f32 WGSL layout convention. Read before using Bevy bounds types or shipping AABBs to WGSL.
type: reference
status: current
tags:
  - bevy
  - bounding-volumes
  - culling
  - wgsl
updated: 2026-09-28
verified: 2026-09-28
code:
  - src/hybrid/bvh.rs
  - src/hybrid/scene.rs
sources:
  - bevy_math 0.19.1 src/bounding/bounded3d/mod.rs
  - bevy_math 0.19.1 src/bounding/raycast3d.rs
  - bevy_camera 0.19.1 src/primitives.rs
  - bevy_render 0.19.1 src/occlusion_culling/
aliases:
  - Aabb3d
  - RayCast3d
  - aabb_intersection_at
  - NoFrustumCulling
---

# AABB primitives in Bevy 0.19.1 (source-verified)

Two distinct types serve two distinct layers; don't mix them up.

## 1. `bevy_math::bounding::Aabb3d` - the math primitive

`crates/bevy_math/src/bounding/bounded3d/mod.rs`

```rust
pub struct Aabb3d { pub min: Vec3A, pub max: Vec3A }
```

- `Aabb3d::new(center, half_size)`, `from_point_cloud(isometry, points)` (exact fit of
  transformed points), plus the generic `BoundingVolume` trait
  (`translate(Vec3A)`, `rotate(Quat)`, `merged`, `grow`, `scale`...).
- Rotation support exists but produces conservative boxes; prefer constructing in the
  rotated frame via `from_point_cloud` when exactness matters.
- Companion volume types: `BoundingSphere` (cheap rejection first, slab test second is
  a classic two-stage pattern).

## 2. `bevy_math::bounding::RayCast3d` - rays with precomputed reciprocals

`bounding/raycast3d.rs`

```rust
pub struct RayCast3d { origin: Vec3A, direction: Dir3A, direction_recip: Vec3A, max: f32 }
impl RayCast3d {
    pub fn new(origin, direction: impl Into<Dir3A>, max: f32) -> Self;
    pub fn from_ray(ray: Ray3d, max: f32) -> Self;
    pub fn aabb_intersection_at(&self, aabb: &Aabb3d) -> Option<f32>;   // hit t
    pub fn sphere_intersection_at(&self, s: &BoundingSphere) -> Option<f32>;
}
```

`aabb_intersection_at` **is** the branchless slab method, implemented with SIMD select:

- `positive = direction.signum().cmpgt(ZERO)` picks near/far corners per axis;
- `(near - origin) * direction_recip` componentwise; NaN-tolerance delegated to
  `Vec3A` min/max ("when one argument is NaN, the other is used");
- `tmin = tmin.max_element().max(0.)`, `tmax = tmax.min_element().min(self.max)`;
  returns `Some(t_enter)` iff `tmin <= tmax`.

Lessons for our own WGSL port: precompute reciprocals per ray; seed the window with the
caller's `[0, max]`; use sign-select instead of per-axis branches.

## 3. `bevy_camera::primitives::Aabb` - the ECS/render component

`bevy_camera/src/primitives.rs`

```rust
#[derive(Component, ...)]
pub struct Aabb { pub center: Vec3A, pub half_extents: Vec3A }
impl Aabb {
    pub fn from_min_max(minimum: Vec3, maximum: Vec3) -> Self;
    pub fn enclosing(points) -> Option<Self>;
    pub fn relative_radius(&self, plane_normal, world_from_local) -> f32; // plane culling
}
```

- Auto-computed for meshes by `CalculateBounds` (visibility system) and consumed by
  Bevy's frustum culling every frame; `NoFrustumCulling` opts an entity out. It is
  *not* auto-updated when mesh vertex data changes.
- This project's SDF entities are invisible to that pipeline (no `Mesh3d`) - our own
  culling must compute its own bounds (`src/hybrid/scene.rs` computes a per-object
  `world_aabb`, which `src/hybrid/bvh.rs` unions up its tree).
- Conversion: `Aabb3d::new(aabb.center, aabb.half_extents)` and back via
  `Aabb::from_min_max(box.min, box.max)`.

## 4. GPU-side precedent: `bevy_render` occlusion culling

`bevy_render/src/occlusion_culling/` (+ `mesh_preprocess_types.wgsl`): Bevy itself runs
**AABB tests in compute shaders** - object bounding boxes are tested against a
hierarchical-Z pyramid built from last frame's depth, cascaded over multiple passes,
with results read back to drive indirect draws. Proof-by-precedent that per-object AABB
work belongs on GPU at scale, and of the readback-driven-indirect pattern.

## WGSL struct-layout convention

This repo stores GPU records as flat scalar fields (`f32`/`u32` only) because
`encase`'s derive pads `vec3<f32>` fields to 16 bytes while `bytemuck::Pod` requires
gap-free Rust layout (see `PrimitiveRecordCpu` doc comment). AABBs therefore ship as
`six scalar f32s` (min_x..max_z), exactly like the existing `center_*`/`param_*`
fields - never as nested vec3s. `src/hybrid/bvh.rs`'s `BvhNodeGpu` follows the same
convention (verified 2026-09-28).

## Related
- [Ray–AABB intersection: the slab method](./ray-aabb-slab-test.md) — deeper: the algorithm `aabb_intersection_at` implements, and its WGSL port.
- [Bevy 0.19.1 primitives & frustum culling](../hierarchical-volumes/bevy-frustum-culling.md) — deeper: `Frustum::intersects_obb` and the visibility pipeline that consumes the `Aabb` component.
- [Where AABB work belongs: CPU or compute shader](./cpu-vs-gpu-placement.md) — applies: when to use `RayCast3d` on the CPU vs a WGSL slab test.
- [Bevy primitives & integration (analytic)](../analytic-intersections/bevy-integration.md) — contrast: Bevy's shape primitives, which carry no intersectors.
