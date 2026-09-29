---
title: Bevy 0.19.1 primitives & frustum culling (source-verified)
description: Documents Bevy 0.19.1's Aabb component, Frustum::intersects_obb (relative_radius plane test), the Visibility/ViewVisibility/CheckVisibility pipeline, NoFrustumCulling, and the GPU Hi-Z occlusion-culling module as a template for custom culling. Read before touching visibility/culling or gating custom passes by view.
type: reference
status: current
tags:
  - bevy
  - culling
  - bounding-volumes
  - render-pipeline
updated: 2026-08-23
verified: 2026-08-23
sources:
  - bevy_camera 0.19.1 src/primitives.rs
  - bevy_camera 0.19.1 src/visibility/mod.rs
  - bevy_render 0.19.1 src/occlusion_culling/
aliases:
  - frustum culling
  - ViewVisibility
  - CheckVisibility
  - intersects_obb
  - Hi-Z occlusion culling
---

# Bevy 0.19.1 primitives & frustum culling (source-verified)

Locations verified in the local registry: `bevy_camera/src/primitives.rs`,
`bevy_camera/src/visibility/mod.rs`, `bevy_render/src/occlusion_culling/`.

## AABB component

```rust
// bevy_camera::primitives
pub struct Aabb { pub center: Vec3A, pub half_extents: Vec3A }
impl Aabb {
    pub fn from_min_max(minimum: Vec3, maximum: Vec3) -> Self;
    pub fn enclosing(points) -> Option<Self>;
    pub fn relative_radius(&self, p_normal: &Vec3A, world_from_local: &Mat3A) -> f32;
}
```

- Auto-computed by the visibility system for meshed entities; NOT auto-updated when
  vertex data changes; `NoFrustumCulling` opts out. Our SDF entities carry no meshes,
  so we compute/insert our own `Aabb`s if we want Bevy's machinery to see them.

## Frustum tests

```rust
pub struct Frustum(pub ViewFrustum);   // 6 half-spaces (+ optional far)
impl Frustum {
    pub fn intersects_obb(&self, aabb: &Aabb, world_from_local: &Affine3A) -> bool;
    pub fn intersects_obb_identity(&self, aabb: &Aabb) -> bool;
    // also sphere variants via BoundingSphere
}
```

`intersects_obb` is the classic **p-vertex/n-vertex plane test**: for each half-space,
`dot(normal_d, center) + relative_radius <= 0` => outside. `relative_radius` is the
Ritter bounding-sphere-of-box trick: `|half_extents . |normal||` - one dot per plane,
no corner transforms. Cheap enough to run per hierarchy node on CPU *and* trivially
portable to WGSL for GPU-side frustum gating of level cells.

## Visibility pipeline

- Components: `Visibility` (authored), `InheritedVisibility`, `ViewVisibility`
  (`#[require(...)]` wired automatically).
- `CheckVisibility` system computes `ViewVisibility` from `Aabb` + each camera's
  `Frustum`; results flow to render-world `RenderVisibleEntities` per phase.
- For custom hierarchies we can either (a) feed per-cell entities through this
  pipeline (heavy), or (b) treat our occupancy pass as the culler and emit work lists
  consumed by our own passes (recommended - matches the indirect-dispatch design).

## Occlusion culling precedent

`bevy_render::occlusion_culling` + `mesh_preprocess_types.wgsl`: GPU-side object-AABB
tests against a hierarchical-Z pyramid built from previous-frame depth, cascaded in
multiple passes, read back asynchronously to drive indirect draws. This is Bevy's own
in-tree example of exactly the CPU-builds-bounds / GPU-tests / async-readback /
indirect-draw loop our occupancy pass should follow.

## Integration points summary

| Need | Bevy primitive |
|---|---|
| Per-entity bounds | insert `Aabb` manually (no Mesh3d needed) |
| Frustum math | `bevy_camera::primitives::{Frustum, Aabb}` |
| Opt-out of auto culling | `NoFrustumCulling` |
| Custom-phase visible set | `RenderVisibleEntities` / custom phase item queues |
| GPU occlusion pattern | copy `occlusion_culling` module's Hi-Z + cascaded preprocess |

## Related
- [AABB primitives in Bevy 0.19.1](../aabb-acceleration/bevy-aabb-primitives.md) — deeper: the math-layer `Aabb3d`/`RayCast3d` types next to the `Aabb` component used here.
- [Occupancy first-pass design](./occupancy-first-pass-design.md) — applies: the (archived) design that planned frustum-first descent and Hi-Z per this note.
- [Camera and the view system](../bevy-rendering/scene-and-views/camera-and-view-system.md) — prerequisite: where each camera's `Frustum` comes from.
- [Where AABB work belongs: CPU or compute shader](../aabb-acceleration/cpu-vs-gpu-placement.md) — applies: CPU frustum culling vs GPU occlusion culling placement.
