//! Physics-vs-static-scene collision queries: a thin wrapper around
//! `hybrid::cpu_ref::local_distance`/`local_normal` — the same per-`Shape`
//! SDF distance/gradient evaluator the renderer already uses to march and
//! shade every frame. Reusing it directly (rather than a second,
//! physics-private distance implementation) is the entire point of building
//! physics natively on the SDF: static-world collision against any
//! primitive this renderer supports is already correct and tested, with
//! zero new geometry code.
//!
//! `PhysicsShape` (not `Shape`) is deliberately the query surface here: a
//! static collider that happened to be a buggy `Shape::RoundedCone` should
//! be a compile error at the call site, not a runtime `unimplemented!()`
//! panic surfacing three call frames deep inside the renderer's own
//! reference module. `query_static_point` converts back to `Shape` only at
//! the last moment, immediately before calling into `cpu_ref`.

use bevy::prelude::*;

use super::components::PhysicsShape;
use crate::hybrid::cpu_ref;

/// Signed distance and world-space surface normal at `p_world`, against one
/// static shape at the given world-space `translation`/`rotation`.
///
/// World<->local convention mirrors `cpu_ref::march_object` exactly (the
/// same convention the renderer's own trace/shade path uses): local space
/// is `inv_rotation * (p_world - translation)`, and a local-space normal is
/// rotated back to world space by the (non-inverse) `rotation`.
///
/// Negative distance means `p_world` is inside the shape (penetrating); the
/// normal always points outward, away from the shape's surface, regardless
/// of which side `p_world` is on — `local_normal`'s central-difference
/// gradient already has this property, since distance increases outward.
pub fn query_static_point(shape: &PhysicsShape, translation: Vec3, rotation: Quat, p_world: Vec3) -> (f32, Vec3) {
    let shape: crate::sdf::components::Shape = (*shape).into();
    let inv_rotation = rotation.inverse();
    let p_local = inv_rotation * (p_world - translation);
    let distance = cpu_ref::local_distance(&shape, p_local);
    let normal_local = cpu_ref::local_normal(&shape, p_local);
    let normal_world = rotation * normal_local;
    (distance, normal_world)
}

/// One static collider in the scene, as physics needs to see it: shape +
/// world-space pose. A separate, physics-owned type (not a direct ECS
/// query result) so `query_nearest_static` is plain, testable data-in/
/// data-out logic — the ECS collection step lives in whatever `Update`-
/// schedule system builds a `Vec<StaticCollider>` each frame (Stage 2's
/// solver wiring), not in this module.
#[derive(Clone, Copy, Debug)]
pub struct StaticCollider {
    pub shape: PhysicsShape,
    pub translation: Vec3,
    pub rotation: Quat,
}

/// Queries every static collider and returns the one with the smallest
/// (most-penetrating, or nearest-if-none-penetrate) signed distance to
/// `p_world` — i.e. the single contact a simple point-sample body (Stage
/// 2's sphere) should resolve against this frame. Linear scan: acceptable
/// at v1 scale per the staged plan (the existing BVH is available as a
/// later broad-phase prune if profiling ever shows this is a bottleneck;
/// not added speculatively here).
///
/// Returns `None` only if `colliders` is empty.
pub fn query_nearest_static(colliders: &[StaticCollider], p_world: Vec3) -> Option<(f32, Vec3)> {
    colliders
        .iter()
        .map(|c| query_static_point(&c.shape, c.translation, c.rotation, p_world))
        .min_by(|(a, _), (b, _)| a.total_cmp(b))
}

#[cfg(test)]
mod tests {
    use bevy::math::EulerRot;

    use super::*;

    #[test]
    fn a_point_outside_a_sphere_reports_positive_distance_and_outward_normal() {
        let shape = PhysicsShape::Sphere { radius: 1.0 };
        let (distance, normal) = query_static_point(&shape, Vec3::ZERO, Quat::IDENTITY, Vec3::new(3.0, 0.0, 0.0));
        assert!((distance - 2.0).abs() < 1e-3);
        assert!((normal - Vec3::X).length() < 1e-3);
    }

    #[test]
    fn a_point_inside_a_sphere_reports_negative_distance() {
        let shape = PhysicsShape::Sphere { radius: 1.0 };
        let (distance, _) = query_static_point(&shape, Vec3::ZERO, Quat::IDENTITY, Vec3::new(0.5, 0.0, 0.0));
        assert!(distance < 0.0);
    }

    #[test]
    fn a_translated_and_rotated_sphere_reports_correct_distance_regardless_of_pose() {
        // A sphere's distance field is rotation-invariant about its own
        // center, so an arbitrary rotation must not change the result —
        // this is exactly the case that would silently break if the
        // world<->local convention's rotation direction were flipped.
        let shape = PhysicsShape::Sphere { radius: 1.0 };
        let translation = Vec3::new(5.0, 2.0, -3.0);
        let rotation = Quat::from_euler(EulerRot::XYZ, 0.4, 0.8, 1.2);
        let p_world = translation + Vec3::new(3.0, 0.0, 0.0);
        let (distance, normal) = query_static_point(&shape, translation, rotation, p_world);
        assert!((distance - 2.0).abs() < 1e-3);
        assert!((normal - Vec3::X).length() < 1e-3);
    }

    #[test]
    fn a_point_above_a_flat_rounded_box_face_reports_the_box_half_extent_offset() {
        let shape = PhysicsShape::RoundedBox { half_extents: Vec3::new(2.0, 0.5, 2.0), corner_radius: 0.0 };
        // Directly above the box's flat top face, well away from any edge.
        let p_world = Vec3::new(0.0, 2.5, 0.0);
        let (distance, normal) = query_static_point(&shape, Vec3::ZERO, Quat::IDENTITY, p_world);
        assert!((distance - 2.0).abs() < 1e-3);
        assert!((normal - Vec3::Y).length() < 1e-2);
    }

    #[test]
    fn a_point_near_a_rounded_boxs_rounded_corner_resolves_to_the_correct_radial_normal() {
        // This is exactly where SDF collision earns its keep over
        // shape-specific special-casing: a naive "distance to the nearest
        // face plane" collision check gets a corner/edge region wrong,
        // but the SDF's own gradient resolves it correctly by construction
        // since `local_distance`/`local_normal` never special-case
        // faces vs. edges vs. corners.
        let half_extents = Vec3::new(1.0, 1.0, 1.0);
        let corner_radius = 0.3;
        let shape = PhysicsShape::RoundedBox { half_extents, corner_radius };
        // A point diagonally outward from the box's +X+Y+Z rounded corner,
        // along the corner's own outward diagonal direction.
        let corner_center = half_extents - Vec3::splat(corner_radius);
        let diagonal = corner_center.normalize();
        let p_world = corner_center + diagonal * (corner_radius + 1.0);
        let (distance, normal) = query_static_point(&shape, Vec3::ZERO, Quat::IDENTITY, p_world);
        assert!((distance - 1.0).abs() < 1e-2, "expected distance ~1.0, got {distance}");
        assert!((normal - diagonal).length() < 1e-2, "expected normal ~{diagonal:?}, got {normal:?}");
    }

    #[test]
    fn query_nearest_static_picks_the_closer_of_two_colliders() {
        let colliders = [
            StaticCollider { shape: PhysicsShape::Sphere { radius: 1.0 }, translation: Vec3::new(-10.0, 0.0, 0.0), rotation: Quat::IDENTITY },
            StaticCollider { shape: PhysicsShape::Sphere { radius: 1.0 }, translation: Vec3::ZERO, rotation: Quat::IDENTITY },
        ];
        let (distance, _) = query_nearest_static(&colliders, Vec3::new(2.0, 0.0, 0.0)).unwrap();
        assert!((distance - 1.0).abs() < 1e-3);
    }

    #[test]
    fn query_nearest_static_returns_none_for_an_empty_collider_list() {
        assert!(query_nearest_static(&[], Vec3::ZERO).is_none());
    }
}
