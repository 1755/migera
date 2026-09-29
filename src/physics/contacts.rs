//! Rigid-vs-rigid contact generation, Claybook-style: each body's own
//! sample points (`physics::sample_points`) are transformed to world space
//! and queried against the OTHER body's SDF for depth and normal, done
//! BOTH directions (A's samples vs. B's field, and B's samples vs. A's
//! field) — catching contacts either body's own sampling alone might miss
//! (e.g. a small sharp corner poking into a large flat face may only be
//! well-sampled from the corner's own side).
//!
//! Each contact is one `Contact` struct with a world-space point, a
//! world-space normal (pointing from B toward A, i.e. the direction A
//! should be pushed to resolve penetration), and a depth. Body indices
//! (`u32`, not `Entity`) match the flat indexing scheme the GPU physics
//! state buffer will use (Stage 3's GPU port).
//!
//! `BodyKind::Kinematic` support (see `physics::components::BodyKind`)
//! needed ZERO changes in this file: `generate_contacts`/`BodySnapshot`
//! only ever see a shape + a world-space pose, with no concept of body
//! classification at all — a kinematic body's snapshot looks exactly like
//! a dynamic or static one to this module. `solve_world.rs`'s new
//! dynamic-vs-kinematic loop calls this same function unchanged; the
//! kinematic-vs-kinematic and kinematic-vs-static pairs are simply never
//! generated at all (that decision lives in `solve_world.rs`'s
//! `generate_all_contacts`, not here), so this module never even sees
//! that those pairs are being skipped.

use bevy::prelude::*;

use super::components::PhysicsShape;
use super::sample_points::sample_points_local;
use crate::hybrid::cpu_ref;
use crate::sdf::components::Shape;

/// One rigid body's pose, as contact generation needs to see it — a
/// snapshot, not a live ECS reference, so this module's functions are
/// plain data-in/data-out and testable without spinning up a Bevy `App`.
#[derive(Clone, Copy, Debug)]
pub struct BodySnapshot {
    pub shape: PhysicsShape,
    pub translation: Vec3,
    pub rotation: Quat,
}

/// A single contact point between two bodies, in world space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    pub body_a: u32,
    pub body_b: u32,
    pub point_world: Vec3,
    /// Points from B toward A — i.e. the direction body A should move to
    /// separate from body B.
    pub normal_world: Vec3,
    /// Positive penetration depth (how far the sample point is inside the
    /// other body's surface). Contacts with non-positive depth are never
    /// returned — see `generate_contacts`'s own filtering.
    pub depth: f32,
}

fn world_to_local(p_world: Vec3, translation: Vec3, rotation: Quat) -> Vec3 {
    rotation.inverse() * (p_world - translation)
}

fn query_body_point(shape: &PhysicsShape, translation: Vec3, rotation: Quat, p_world: Vec3) -> (f32, Vec3) {
    let full_shape: Shape = (*shape).into();
    let p_local = world_to_local(p_world, translation, rotation);
    let distance = cpu_ref::local_distance(&full_shape, p_local);
    let normal_local = cpu_ref::local_normal(&full_shape, p_local);
    (distance, rotation * normal_local)
}

/// True penetration depth between the two actual surfaces at a sample
/// point, not just the raw other-body query. `sample_points_local`
/// returns points ON the sampling shape's own surface for every shape kind
/// (including `Sphere`, since a fix to its earlier center-only sampling —
/// see that module's own doc comment for the bug this caused), so
/// `own_local_distance` is ~0 for essentially every sample in practice
/// and this is a near no-op. Kept anyway (rather than assumed away) as a
/// defensive correctness net: it costs one extra `local_distance` call per
/// sample point and makes the depth calculation correct even if a future
/// shape's sample set ever returns a near-surface-but-not-exact point
/// (e.g. floating-point rounding in a closed-form surface formula), same
/// reasoning `physics::solve_static`'s own `distance - contact_radius`
/// convention uses for its own single-point sphere-probe approximation.
fn own_local_distance(shape: &PhysicsShape, local_point: Vec3) -> f32 {
    let full_shape: Shape = (*shape).into();
    cpu_ref::local_distance(&full_shape, local_point)
}

/// Generates every penetrating contact between `a` and `b`: `a`'s own
/// sample points queried against `b`'s field (normal points from B toward
/// A, i.e. `b`'s own outward normal at that point — pushing A away from
/// B), then `b`'s sample points queried against `a`'s field (normal
/// flipped, since `a`'s own outward normal there points from A toward B,
/// the opposite convention). Only contacts with positive depth
/// (true separation < 0, i.e. actually penetrating) are returned.
pub fn generate_contacts(body_a: u32, a: &BodySnapshot, body_b: u32, b: &BodySnapshot) -> Vec<Contact> {
    let mut contacts = Vec::new();

    for &local_point in &sample_points_local(&a.shape) {
        let p_world = a.rotation * local_point + a.translation;
        let (raw_distance, normal_from_b) = query_body_point(&b.shape, b.translation, b.rotation, p_world);
        let separation = raw_distance + own_local_distance(&a.shape, local_point);
        if separation < 0.0 {
            contacts.push(Contact {
                body_a,
                body_b,
                point_world: p_world,
                normal_world: normal_from_b,
                depth: -separation,
            });
        }
    }

    for &local_point in &sample_points_local(&b.shape) {
        let p_world = b.rotation * local_point + b.translation;
        let (raw_distance, normal_from_a) = query_body_point(&a.shape, a.translation, a.rotation, p_world);
        let separation = raw_distance + own_local_distance(&b.shape, local_point);
        if separation < 0.0 {
            contacts.push(Contact {
                body_a,
                body_b,
                point_world: p_world,
                // `normal_from_a` is A's own outward normal at this point
                // (pointing from A toward B, since the sample point
                // belongs to B and is penetrating INTO A) — the
                // `Contact::normal_world` convention is "points from B
                // toward A," so this must be negated.
                normal_world: -normal_from_a,
                depth: -separation,
            });
        }
    }

    contacts
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere_at(translation: Vec3, radius: f32) -> BodySnapshot {
        BodySnapshot { shape: PhysicsShape::Sphere { radius }, translation, rotation: Quat::IDENTITY }
    }

    #[test]
    fn two_separated_spheres_produce_no_contacts() {
        let a = sphere_at(Vec3::ZERO, 1.0);
        let b = sphere_at(Vec3::new(5.0, 0.0, 0.0), 1.0);
        assert!(generate_contacts(0, &a, 1, &b).is_empty());
    }

    #[test]
    fn two_overlapping_spheres_produce_a_contact_with_correct_normal_and_depth() {
        // Centers 1.5 apart, both radius 1.0 -> maximum overlap depth 0.5,
        // approached (not hit exactly) by whichever discrete sample point
        // lands closest to the axis between the two centers. Since each
        // sphere now samples a fixed FINITE set of surface points (not
        // just its center -- see sample_points.rs's own doc comment for
        // why a center-only sample can't detect nearby curvature, and
        // SPHERE_SAMPLE_COUNT's own doc comment for why exact coverage of
        // every possible contact direction isn't achievable with any
        // finite fixed sample set), the deepest reported contact is
        // necessarily a slight underestimate of the true analytic depth —
        // a real, inherent discretization gap, not a bug to eliminate via
        // more precision here. A 5% relative tolerance reflects that
        // reality rather than asserting exactness a finite sample count
        // structurally cannot provide.
        let a = sphere_at(Vec3::ZERO, 1.0);
        let b = sphere_at(Vec3::new(1.5, 0.0, 0.0), 1.0);
        let contacts = generate_contacts(0, &a, 1, &b);
        assert!(!contacts.is_empty());
        let deepest = contacts.iter().max_by(|x, y| x.depth.total_cmp(&y.depth)).unwrap();
        assert!((deepest.depth - 0.5).abs() < 0.05, "expected deepest contact depth ~0.5 (within the sample set's inherent discretization gap), got {}", deepest.depth);
        // Normal points from B toward A: B is at +X of A, so the
        // separating direction for A is -X. Same discretization-gap
        // reasoning as the depth assertion above: the sample point
        // closest to the true axis is offset from it by some angle
        // dependent on the fixed sample lattice's spacing, so the deepest
        // contact's normal is close to, not exactly, (-1,0,0).
        assert!((deepest.normal_world - Vec3::NEG_X).length() < 0.2, "expected deepest contact normal close to (-1,0,0) (within the sample set's inherent discretization gap), got {:?}", deepest.normal_world);
    }

    #[test]
    fn contact_generation_is_symmetric_in_depth_regardless_of_argument_order() {
        let a = sphere_at(Vec3::ZERO, 1.0);
        let b = sphere_at(Vec3::new(1.5, 0.0, 0.0), 1.0);
        let ab = generate_contacts(0, &a, 1, &b);
        let ba = generate_contacts(1, &b, 0, &a);
        assert!(!ab.is_empty());
        assert!(!ba.is_empty());
        let ab_depth: f32 = ab.iter().map(|c| c.depth).sum::<f32>() / ab.len() as f32;
        let ba_depth: f32 = ba.iter().map(|c| c.depth).sum::<f32>() / ba.len() as f32;
        assert!((ab_depth - ba_depth).abs() < 1e-2, "expected symmetric average depth, got {ab_depth} vs {ba_depth}");
    }

    #[test]
    fn a_box_resting_flat_on_another_box_produces_a_multi_point_manifold() {
        // Two boxes, B directly below A, overlapping slightly along Y —
        // this is exactly the "one distance query isn't enough" case:
        // a single closest-point query gives one contact, but stable
        // resting needs multiple (all 4 bottom corners of A touching B).
        let a = BodySnapshot {
            shape: PhysicsShape::RoundedBox { half_extents: Vec3::new(1.0, 1.0, 1.0), corner_radius: 0.0 },
            translation: Vec3::new(0.0, 1.9, 0.0),
            rotation: Quat::IDENTITY,
        };
        let b = BodySnapshot {
            shape: PhysicsShape::RoundedBox { half_extents: Vec3::new(2.0, 1.0, 2.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        let contacts = generate_contacts(0, &a, 1, &b);
        // A's 4 bottom corners (y = 1.9 - 1.0 = 0.9) all penetrate B's top
        // face (y = 1.0) by 0.1 -- expect at least 4 contact points, not 1.
        assert!(contacts.len() >= 4, "expected a multi-point manifold, got {} contacts", contacts.len());
        for c in &contacts {
            assert!((c.depth - 0.1).abs() < 1e-2, "expected depth ~0.1, got {}", c.depth);
        }
    }

    #[test]
    fn a_small_corner_poking_into_a_large_flat_face_is_still_caught() {
        // The exact case the module doc comment flags: a small body's own
        // sampling alone might miss a large flat body's surface if the
        // small body's samples don't happen to land inside it, but the
        // LARGE body would need its own samples densely covering a huge
        // area to catch a small poking corner from its own side either.
        // Here, checking the small body's own corner samples against the
        // large flat body's field is what actually catches it -- confirms
        // the asymmetric small-vs-large case works via the small body's
        // own sample direction.
        let small = BodySnapshot {
            shape: PhysicsShape::RoundedBox { half_extents: Vec3::splat(0.1), corner_radius: 0.0 },
            translation: Vec3::new(0.0, 0.95, 0.0),
            rotation: Quat::IDENTITY,
        };
        let large_floor = BodySnapshot {
            shape: PhysicsShape::RoundedBox { half_extents: Vec3::new(20.0, 1.0, 20.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        let contacts = generate_contacts(0, &small, 1, &large_floor);
        assert!(!contacts.is_empty(), "expected the small body's own corner samples to catch penetration into the large floor");
    }
}
