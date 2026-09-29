//! ECS-scene-to-flat-object-list collection: the first real feature in the
//! fresh-start `hybrid` renderer, and deliberately scoped narrow — a flat
//! list of top-level shapes (one per entity, no CSG-tree splitting, no
//! GPU-record encoding) with a correctly computed world-space AABB per
//! object. That's the whole job of this step: prove the AABB/BVH
//! acceleration structure is correct before any SDF marching exists to
//! consume it.
//!
//! Fresh implementation — does not import or reuse anything from
//! `hybrid_legacy`. It leans on `crate::prim::Aabb::transformed()` (a
//! shared, renderer-agnostic utility, not `hybrid_legacy`-owned) for the
//! rotation-correct world-space bound: local AABB computed once per shape
//! kind, then the 8 local corners are rotated and translated and the
//! extremes taken — a naive "translate the local AABB by the transform's
//! position" would silently under-bound any rotated box.

use bevy::prelude::*;

use crate::hybrid::material::Material;
use crate::hybrid::motion::PreviousShapeTransform;
use crate::prim::Aabb;
use crate::sdf::assembly::SdfSceneRoot;
use crate::sdf::components::Shape;

/// One top-level shape collected from the scene: its computed world-space
/// AABB, ready for BVH construction.
#[derive(Clone, Copy, Debug)]
pub struct HybridObject {
    pub entity: Entity,
    pub world_aabb: Aabb,
}

/// Everything the GPU upload path needs for one object, beyond what
/// `HybridObject` already carries for BVH construction: its shape/material
/// for marching + flat-color reporting, and its world transform
/// (translation + rotation only — this renderer's shapes don't support
/// non-uniform scale, matching `world_aabb`'s own scale-discarding
/// convention and `cpu_ref::TraceObject`'s shape exactly, since this is the
/// main-world-queryable counterpart that later gets turned into
/// `TraceObject`/GPU records). Kept as a separate collection function
/// rather than folding into `HybridObject` itself: `HybridObject` is
/// consumed by `Bvh::build`/`update`, which only ever look at
/// `entity`/`world_aabb` — adding shape/material fields there would carry
/// dead weight through every BVH call site for no reason.
#[derive(Clone, Debug)]
pub struct HybridObjectData {
    pub entity: Entity,
    pub shape: Shape,
    pub translation: Vec3,
    pub rotation: Quat,
    pub material: Material,
    /// Last frame's translation/rotation, for temporal-accumulation
    /// reprojection (`extract::object_gpu_from`) — `None` for a
    /// freshly-spawned entity `motion::update_previous_shape_transforms`
    /// hasn't snapshotted yet, in which case the caller treats "no prior
    /// frame" as "no motion" (falls back to this frame's own transform).
    pub previous: Option<PreviousShapeTransform>,
}

/// This shape's bounds in its own local space (centered on its origin, no
/// rotation/translation applied yet) — mirrors `crate::prim::local_aabb`'s
/// per-kind coverage, written fresh against `sdf::components::Shape`
/// rather than `GpuPrimitive` since this step has no GPU record encoding
/// yet to share that type with.
pub fn local_aabb(shape: &Shape) -> Aabb {
    match *shape {
        Shape::Sphere { radius } => Aabb::from_center_half(Vec3::ZERO, Vec3::splat(radius)),
        Shape::RoundedBox { half_extents, .. } => Aabb::from_center_half(Vec3::ZERO, half_extents),
        Shape::RoundedCylinder { radius, half_height, .. } => {
            Aabb::from_center_half(Vec3::ZERO, Vec3::new(radius, half_height, radius))
        }
        Shape::Capsule { a, b, radius } => Aabb {
            min: a.min(b) - Vec3::splat(radius),
            max: a.max(b) + Vec3::splat(radius),
        },
        Shape::RoundedCone { a, b, r0, r1 } => {
            let r = Vec3::splat(r0.max(r1));
            Aabb { min: a.min(b) - r, max: a.max(b) + r }
        }
        Shape::Ellipsoid { radii } => Aabb::from_center_half(Vec3::ZERO, radii),
        Shape::BoxFrame { half_extents, .. } => Aabb::from_center_half(Vec3::ZERO, half_extents),
        Shape::HexPrism { radius, half_height } => {
            // Conservative: |x| <= r, |z| <= 2r/sqrt(3), |y| <= half_height.
            Aabb::from_center_half(Vec3::ZERO, Vec3::new(radius, half_height, 2.0 * radius / 3f32.sqrt()))
        }
    }
}

/// This shape's world-space AABB under `transform`: local bounds rotated
/// and translated via all 8 corners (`Aabb::transformed`), not the local
/// AABB merely translated by `transform.translation` — that shortcut
/// under-bounds any shape whose `transform.rotation` isn't identity.
pub fn world_aabb(shape: &Shape, transform: &GlobalTransform) -> Aabb {
    local_aabb(shape).transformed(transform.translation(), transform.rotation())
}

/// Walks every `SdfSceneRoot`'s shape entities and collects their
/// world-space AABBs. No CSG grouping yet (each shape is its own
/// `HybridObject`) — that's a later step once real objects need to be
/// composed from multiple primitives.
pub fn collect(
    roots: &Query<Entity, With<SdfSceneRoot>>,
    shapes: &Query<(Entity, &Shape, &GlobalTransform, Option<&ChildOf>)>,
) -> Vec<HybridObject> {
    let mut objects = Vec::new();
    for (entity, shape, transform, child_of) in shapes {
        let under_root = child_of.is_some_and(|c| roots.contains(c.parent())) || roots.contains(entity);
        if !under_root {
            continue;
        }
        objects.push(HybridObject { entity, world_aabb: world_aabb(shape, transform) });
    }
    objects
}

/// Same walk as `collect`, but gathers the shape/material/transform data the
/// GPU upload path needs instead of just the AABB. Kept as a separate query
/// over the same `SdfSceneRoot`-rooted shape entities rather than merged
/// into `collect` itself, per this module's own established pattern of one
/// narrow function per concern (see `local_aabb`/`world_aabb`/`collect`).
/// Missing `Material` defaults to flat black (`Material::new(Vec3::ZERO,
/// 0.0, 0.0)`) — every shape this renderer spawns today carries an explicit
/// `Material`, but a future shape without one shouldn't panic the whole
/// extraction.
#[allow(clippy::type_complexity)]
pub fn collect_data(
    roots: &Query<Entity, With<SdfSceneRoot>>,
    shapes: &Query<(Entity, &Shape, &GlobalTransform, Option<&Material>, Option<&ChildOf>, Option<&PreviousShapeTransform>)>,
) -> Vec<HybridObjectData> {
    let mut objects = Vec::new();
    for (entity, shape, transform, material, child_of, previous) in shapes {
        let under_root = child_of.is_some_and(|c| roots.contains(c.parent())) || roots.contains(entity);
        if !under_root {
            continue;
        }
        objects.push(HybridObjectData {
            entity,
            shape: *shape,
            translation: transform.translation(),
            rotation: transform.rotation(),
            material: material.copied().unwrap_or(Material::new(Vec3::ZERO, 0.0, 0.0)),
            previous: previous.copied(),
        });
    }
    objects
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_4;

    use bevy::math::EulerRot;

    use super::*;

    /// Identity rotation: world AABB must equal the local AABB translated
    /// by the transform's position — the trivial case, but worth pinning
    /// so a regression here is caught immediately.
    #[test]
    fn axis_aligned_box_world_aabb_matches_translated_local_aabb() {
        let shape = Shape::RoundedBox { half_extents: Vec3::new(1.0, 0.5, 2.0), corner_radius: 0.0 };
        let transform = GlobalTransform::from(Transform::from_xyz(3.0, 4.0, 5.0));
        let aabb = world_aabb(&shape, &transform);
        assert!((aabb.min - Vec3::new(2.0, 3.5, 3.0)).length() < 1e-5, "min = {:?}", aabb.min);
        assert!((aabb.max - Vec3::new(4.0, 4.5, 7.0)).length() < 1e-5, "max = {:?}", aabb.max);
    }

    /// A cube rotated 45 degrees about Y: its true world AABB half-extent
    /// on X/Z grows to `half * sqrt(2)` (the diagonal swept into an axis),
    /// while Y is untouched (rotation axis). A naive "translate the local
    /// AABB, ignore rotation" implementation would report the unrotated
    /// half-extent (1.0) here instead of the correct ~1.41 — this is
    /// exactly the bug `Aabb::transformed`'s 8-corner method exists to
    /// avoid, pinned so this module can never regress into it silently.
    #[test]
    fn cube_rotated_45_degrees_about_y_has_diagonal_world_aabb() {
        let shape = Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 };
        let transform =
            GlobalTransform::from(Transform::from_rotation(Quat::from_rotation_y(FRAC_PI_4)));
        let aabb = world_aabb(&shape, &transform);
        let expected_half_xz = std::f32::consts::SQRT_2;
        assert!((aabb.max.x - expected_half_xz).abs() < 1e-4, "max.x = {}", aabb.max.x);
        assert!((aabb.max.z - expected_half_xz).abs() < 1e-4, "max.z = {}", aabb.max.z);
        assert!((aabb.max.y - 1.0).abs() < 1e-5, "max.y = {} (Y is the rotation axis, untouched)", aabb.max.y);
        assert!(
            aabb.max.x > 1.0 + 1e-3,
            "rotated AABB must be strictly larger than the unrotated half-extent \
             (a naive translate-only implementation would wrongly report 1.0 here)"
        );
    }

    /// A rotated cube's world AABB must always fully contain the cube's
    /// actual rotated corners — the defining correctness property this
    /// whole module exists to guarantee, checked at an arbitrary rotation
    /// (not just the analytically-convenient 45-degree case above).
    #[test]
    fn world_aabb_contains_all_rotated_corners_at_an_arbitrary_rotation() {
        let half = Vec3::new(1.0, 0.5, 2.0);
        let shape = Shape::RoundedBox { half_extents: half, corner_radius: 0.0 };
        let translation = Vec3::new(-3.0, 2.0, 7.0);
        let rotation = Quat::from_euler(EulerRot::YXZ, 0.7, 0.3, 1.1);
        let transform = GlobalTransform::from(Transform { translation, rotation, ..default() });
        // Corners are recomputed via the same rotate-then-translate math
        // `Aabb::transformed` itself uses, so a corner can legitimately
        // land exactly on the computed boundary — expand by a tiny
        // epsilon so float rounding at the boundary isn't mistaken for a
        // real under-bounding bug.
        let aabb = world_aabb(&shape, &transform).expanded(1e-4);
        for sx in [-1.0f32, 1.0] {
            for sy in [-1.0f32, 1.0] {
                for sz in [-1.0f32, 1.0] {
                    let local_corner = half * Vec3::new(sx, sy, sz);
                    let world_corner = rotation * local_corner + translation;
                    assert!(
                        aabb.contains(world_corner),
                        "world AABB {aabb:?} does not contain rotated corner {world_corner:?}"
                    );
                }
            }
        }
    }
}
