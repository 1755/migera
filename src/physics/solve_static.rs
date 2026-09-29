//! Single-contact positional correction against the static scene — the
//! first real solver, deliberately not yet the full XPBD substep loop
//! (Stage 3, once rigid-vs-rigid contacts exist too and substeps genuinely
//! pay for themselves against multiple simultaneous constraints). One
//! iteration, one contact point per body (a body's own center — Stage 3's
//! Claybook-style multi-sample-point contact generation is what enables
//! stable multi-point manifolds; a single point is sufficient to prove
//! "falls under gravity, rests on the floor" correctness first).

use bevy::prelude::*;

use super::collision_static::{StaticCollider, query_nearest_static};
use super::components::{Inertia, PhysicsShape, RigidBody};

/// Point-source gravity: every body accelerates toward `center` at a
/// constant `magnitude`, regardless of distance — not inverse-square
/// Newtonian falloff. Deliberately a single model, not "flat gravity" and
/// "point gravity" as two separate variants: a constant-magnitude pull
/// toward a point far below a scene's floor is visually indistinguishable
/// from uniform downward gravity at the scale any of this project's scenes
/// operate at (the direction barely changes across a few units of lateral
/// spread when the center is thousands of units away), while a point
/// close enough to matter directly gives planetary/small-body gravity
/// (bodies fall toward a sphere's center from any side) for free, with no
/// separate code path. A resource (not a per-body override component) —
/// v1 has exactly one global gravity source, matching every other physics
/// stage's "add the override mechanism only once something actually needs
/// it" discipline.
#[derive(Resource, Clone, Copy, Debug, PartialEq)]
pub struct PhysicsGravity {
    pub center: Vec3,
    pub magnitude: f32,
}

impl PhysicsGravity {
    /// Acceleration vector at `position`: constant `magnitude`, always
    /// pointing from `position` toward `center`. Returns zero (not NaN)
    /// if a body's position exactly coincides with `center` — an
    /// unreachable case in practice (nothing should ever be co-located
    /// with the gravity source), but `normalize_or_zero` costs nothing
    /// extra and turns a would-be NaN into an inert no-op instead of
    /// silently poisoning the body's velocity the same way the dt=0 bug
    /// this project already fixed once did.
    pub fn acceleration_at(&self, position: Vec3) -> Vec3 {
        (self.center - position).normalize_or_zero() * self.magnitude
    }
}

impl Default for PhysicsGravity {
    /// A center far below the origin along -Y approximates today's old
    /// uniform downward gravity: at any position within a typical scene's
    /// lateral extent (a few tens of units), the direction toward a
    /// center 10,000 units straight down is within a fraction of a degree
    /// of straight down everywhere, an imperceptible curvature error.
    fn default() -> Self {
        Self { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 }
    }
}

/// Integrates gravity into velocity, then resolves penetration against the
/// nearest static collider by correcting position along the contact normal
/// and zeroing the inward-normal component of velocity (restitution 0 — a
/// body coming to rest, not bouncing). A body with `Inertia::STATIC`
/// (`inverse_mass == 0.0`) is immovable by construction: every correction
/// below is scaled by `inverse_mass`, so it's naturally a no-op without a
/// separate "is this static" branch.
///
/// `contact_radius` is the moving body's own bounding radius around
/// `position` (its center) — a body isn't a zero-radius point, so the
/// resting condition is "clearance from the SDF surface equals the body's
/// own radius," not "the center sits exactly on the surface." Treating the
/// query point as a sphere-traced probe of this radius is the correct
/// general SDF-collision technique regardless of the body's actual shape:
/// Stage 3's Claybook-style per-shape sample points replace this single
/// center+radius approximation with a proper multi-point manifold; a
/// sphere is the exact case where a single center+radius probe already
/// IS the correct answer, not an approximation.
///
/// Pure function (no ECS) so the solver step itself is unit-testable
/// without spinning up a Bevy `App` — the calling `Update`-schedule system
/// (`solve_static_collisions`) is the thin ECS-facing wrapper.
pub fn solve_body_static(
    body: &mut RigidBody,
    inertia: &Inertia,
    position: &mut Vec3,
    contact_radius: f32,
    colliders: &[StaticCollider],
    gravity: Vec3,
    dt: f32,
) {
    if inertia.inverse_mass == 0.0 || dt <= 0.0 {
        // `dt <= 0.0` guards against a real, confirmed bug: Bevy's `Time`
        // reports `delta_secs() == 0.0` on the very first `Update` tick
        // (no elapsed wall-clock yet — the same fact
        // `physics::integrate`'s own regression test pins). Dividing the
        // position-delta-derived velocity below by a zero `dt` produces
        // `Inf`/`NaN`, which then poisons this body's `Transform` on the
        // very next frame, which poisons the hybrid renderer's persistent
        // BVH (`bvh::update_persistent_bvh` unions every object's AABB
        // into its ancestors, and NaN propagates unpredictably through
        // `min`/`max`) — observed as the ENTIRE static scene rendering
        // fully black (every shadow ray reporting a false hit against the
        // NaN-poisoned tree), not just the affected body, and not
        // recovering on later frames since the BVH refits in place rather
        // than rebuilding from scratch.
        return;
    }

    // XPBD convention (Müller et al., "Detailed Rigid Body Simulation with
    // Extended Position Based Dynamics"): integrate to a PREDICTED
    // position, correct the position directly against the constraint, then
    // derive velocity from the actual position delta — never patch velocity
    // separately from the position correction. An earlier version of this
    // function corrected position but then re-derived velocity by
    // projecting out the PRE-correction velocity's inward-normal
    // component; that's inconsistent with the position correction actually
    // applied (the correction removes exactly the normal-direction
    // displacement, but the ad hoc velocity projection used a normal
    // sampled a fraction of a step earlier/later than the one the position
    // correction used). Near a curved region where the normal itself
    // changes direction from step to step, that inconsistency injected a
    // small spurious tangential velocity every frame — individually
    // negligible, but compounding over hundreds of frames into a runaway
    // slide off the surface (caught by this function's own settling tests
    // going unstable at a rounded-box corner during Stage 2 development).
    let predicted = *position + body.linear_velocity * dt + gravity * dt * dt;

    let corrected = match query_nearest_static(colliders, predicted) {
        Some((distance, normal)) if distance - contact_radius < 0.0 => predicted - normal * (distance - contact_radius),
        _ => predicted,
    };

    body.linear_velocity = (corrected - *position) / dt;
    *position = corrected;
}

/// A dynamic body's own bounding radius around its center, for the
/// center+radius sphere-probe approximation `solve_body_static` uses (see
/// its own doc comment). Exact for `Sphere` (the only shape this stage's
/// gallery/test scenes actually drop onto the floor); a conservative
/// bounding-sphere radius for every other shape, since a single point
/// probe is only ever an approximation for non-spherical shapes — Stage
/// 3's Claybook-style multi-sample-point contact generation replaces this
/// entirely rather than refining it further.
pub fn bounding_radius(shape: &PhysicsShape) -> f32 {
    match *shape {
        PhysicsShape::Sphere { radius } => radius,
        PhysicsShape::RoundedBox { half_extents, .. } => half_extents.length(),
        PhysicsShape::RoundedCylinder { radius, half_height, .. } => (Vec3::new(radius, half_height, radius)).length(),
        PhysicsShape::Capsule { a, b, radius } => (a - b).length() * 0.5 + radius,
        PhysicsShape::Ellipsoid { radii } => radii.max_element(),
        PhysicsShape::BoxFrame { half_extents, .. } => half_extents.length(),
        PhysicsShape::HexPrism { radius, half_height } => (Vec3::new(radius, half_height, radius)).length(),
    }
}

/// `Update`-schedule system: collects every static collider once per
/// frame — a `PhysicsShape` entity with no `RigidBody` component, since a
/// purely static shape has no need for velocity state at all — then
/// resolves every dynamic `RigidBody` entity (which must also carry
/// `PhysicsShape`, for `bounding_radius`) against them. Registered
/// `.before(bvh::update_persistent_bvh)` by `PhysicsPlugin`, same ordering
/// constraint as `physics_integrate_placeholder`.
#[allow(clippy::type_complexity)]
pub fn solve_static_collisions(
    time: Res<Time>,
    gravity: Res<PhysicsGravity>,
    statics: Query<(&PhysicsShape, &GlobalTransform), Without<RigidBody>>,
    mut dynamics: Query<(&mut RigidBody, &Inertia, &PhysicsShape, &mut Transform)>,
) {
    let colliders: Vec<StaticCollider> = statics
        .iter()
        .map(|(shape, transform)| StaticCollider {
            shape: *shape,
            translation: transform.translation(),
            rotation: transform.rotation(),
        })
        .collect();

    let dt = time.delta_secs();
    for (mut body, inertia, shape, mut transform) in &mut dynamics {
        let mut position = transform.translation;
        let radius = bounding_radius(shape);
        let g = gravity.acceleration_at(position);
        solve_body_static(&mut body, inertia, &mut position, radius, &colliders, g, dt);
        transform.translation = position;
    }
}

#[cfg(test)]
mod tests {
    use super::super::components::PhysicsShape;
    use super::*;

    fn floor() -> StaticCollider {
        StaticCollider {
            shape: PhysicsShape::RoundedBox { half_extents: Vec3::new(10.0, 0.5, 10.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        }
    }

    #[test]
    fn point_gravity_always_points_from_the_body_toward_the_center() {
        let gravity = PhysicsGravity { center: Vec3::ZERO, magnitude: 9.81 };
        let a = gravity.acceleration_at(Vec3::new(10.0, 0.0, 0.0));
        assert!((a - Vec3::new(-9.81, 0.0, 0.0)).length() < 1e-4, "expected pull toward center along -X, got {a:?}");

        let b = gravity.acceleration_at(Vec3::new(0.0, 0.0, -10.0));
        assert!((b - Vec3::new(0.0, 0.0, 9.81)).length() < 1e-4, "expected pull toward center along +Z, got {b:?}");
    }

    #[test]
    fn point_gravity_magnitude_does_not_change_with_distance() {
        // Confirms the deliberate "constant magnitude, not inverse-square"
        // choice this module's own doc comment describes -- a body twice
        // as far from the center must still feel the same magnitude pull,
        // not a weaker one.
        let gravity = PhysicsGravity { center: Vec3::ZERO, magnitude: 9.81 };
        let near = gravity.acceleration_at(Vec3::new(5.0, 0.0, 0.0)).length();
        let far = gravity.acceleration_at(Vec3::new(500.0, 0.0, 0.0)).length();
        assert!((near - far).abs() < 1e-4, "expected equal magnitude regardless of distance, got near={near} far={far}");
    }

    #[test]
    fn point_gravity_at_the_center_itself_is_zero_not_nan() {
        let gravity = PhysicsGravity { center: Vec3::new(3.0, 4.0, 5.0), magnitude: 9.81 };
        let a = gravity.acceleration_at(Vec3::new(3.0, 4.0, 5.0));
        assert_eq!(a, Vec3::ZERO);
        assert!(a.is_finite());
    }

    #[test]
    fn default_gravity_approximates_flat_downward_gravity_across_a_typical_scene_width() {
        // The whole justification for using one point-source model instead
        // of a separate flat-gravity variant: two bodies far apart
        // laterally (a realistic scene width) must still feel gravity
        // pointing almost exactly straight down, not visibly converging
        // toward each other.
        let gravity = PhysicsGravity::default();
        let left = gravity.acceleration_at(Vec3::new(-20.0, 0.0, 0.0)).normalize();
        let right = gravity.acceleration_at(Vec3::new(20.0, 0.0, 0.0)).normalize();
        let straight_down = Vec3::NEG_Y;
        assert!(left.dot(straight_down) > 0.999, "expected left position's gravity direction to be nearly straight down, got {left:?}");
        assert!(right.dot(straight_down) > 0.999, "expected right position's gravity direction to be nearly straight down, got {right:?}");
    }

    #[test]
    fn a_static_body_never_moves_regardless_of_gravity() {
        let mut body = RigidBody::default();
        let inertia = Inertia::STATIC;
        let mut position = Vec3::new(0.0, 5.0, 0.0);
        solve_body_static(&mut body, &inertia, &mut position, 0.0, &[floor()], Vec3::new(0.0, -9.81, 0.0), 1.0 / 60.0);
        assert_eq!(position, Vec3::new(0.0, 5.0, 0.0));
        assert_eq!(body.linear_velocity, Vec3::ZERO);
    }

    #[test]
    fn a_zero_delta_time_step_never_produces_nan_or_infinite_velocity() {
        // Regression test for a real bug caught during this stage's own
        // development: Bevy's `Time::delta_secs()` is exactly 0.0 on the
        // very first `Update` tick (no elapsed wall-clock yet — the same
        // fact `physics::integrate`'s own test pins). The XPBD-style
        // `velocity = (corrected - position) / dt` derivation divides by
        // that zero `dt`, producing NaN/Inf, which then poisons this
        // body's `Transform` next frame and, through it, the hybrid
        // renderer's persistent BVH — observed as the entire static scene
        // rendering fully black, not just the affected body, since NaN
        // propagates unpredictably through the BVH's min/max unioning and
        // the tree refits in place rather than rebuilding from scratch.
        let mut body = RigidBody::default();
        let inertia = Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ONE };
        let mut position = Vec3::new(0.0, 5.0, 0.0);
        solve_body_static(&mut body, &inertia, &mut position, 0.75, &[floor()], Vec3::new(0.0, -9.81, 0.0), 0.0);
        assert_eq!(position, Vec3::new(0.0, 5.0, 0.0), "a zero-dt step must be a complete no-op on position");
        assert!(body.linear_velocity.is_finite(), "expected finite velocity after a zero-dt step, got {:?}", body.linear_velocity);
        assert_eq!(body.linear_velocity, Vec3::ZERO);
    }

    #[test]
    fn a_falling_sphere_comes_to_rest_on_the_floor_surface() {
        let radius = 0.75;
        let inertia = Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ONE };
        let mut body = RigidBody::default();
        let mut position = Vec3::new(0.0, 5.0, 0.0);
        let colliders = [floor()];
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 60.0;

        // The solver corrects the sphere's CENTER so its clearance
        // (distance-to-surface minus its own radius) settles at >= 0.
        // Simulate enough steps for it to settle, then check the resting
        // clearance stays within a small tolerance across further steps
        // (no indefinite sinking or jittering).
        for _ in 0..600 {
            solve_body_static(&mut body, &inertia, &mut position, radius, &colliders, gravity, dt);
        }

        let (distance, _) = query_nearest_static(&colliders, position).unwrap();
        // A resting sphere's center-to-floor distance settles at
        // approximately its own `radius` (clearance ~= 0), confirming it
        // neither sank through nor is floating away from the surface.
        assert!((distance - radius).abs() < 0.05, "expected resting distance ~{radius}, got {distance}");

        // Must have stopped moving (settled), not still oscillating.
        assert!(body.linear_velocity.length() < 0.05, "expected near-zero velocity at rest, got {:?}", body.linear_velocity);
    }

    #[test]
    fn a_sphere_dropped_near_a_rounded_boxs_corner_rests_against_the_correct_surface() {
        // The exact case collision_static's own tests target: resolving
        // against a rounded corner/edge region, not just a flat face —
        // here verified end-to-end through the solver, not just the raw
        // distance/gradient query.
        let half_extents = Vec3::new(1.0, 1.0, 1.0);
        let corner_radius = 0.3;
        let block = StaticCollider {
            shape: PhysicsShape::RoundedBox { half_extents, corner_radius },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        let corner_center = half_extents - Vec3::splat(corner_radius);
        let diagonal = corner_center.normalize();

        let radius = 0.4;
        let inertia = Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ONE };
        let mut body = RigidBody::default();
        // Start well outside, along the corner's own outward diagonal, so
        // gravity alone won't naturally carry it toward the corner region
        // — instead give it an initial velocity aimed at the corner to
        // guarantee the contact this test wants to exercise.
        let mut position = corner_center + diagonal * 3.0;
        body.linear_velocity = -diagonal * 2.0;

        let colliders = [block];
        // Zero gravity deliberately: a convex rounded corner has no local
        // minimum for a frictionless point contact (nothing arrests
        // sideways sliding, same as a marble balanced on a dome), so under
        // continuous gravity a body landing exactly on the corner's own
        // diagonal apex will eventually roll off, however good the
        // solver's per-contact math is — see
        // `a_sphere_resting_on_a_flat_face_near_a_corner_stays_settled_indefinitely`
        // below for the long-run stability regression this fact demands.
        // Here we only want to confirm ONE thing in isolation: given an
        // incoming velocity toward the corner, does the solver resolve the
        // very next contact against the corner's own correct
        // distance/normal (not a flat-face approximation)? Zero gravity
        // means no continued driving force once that single contact is
        // resolved, so the body simply stops there instead of sliding on.
        let gravity = Vec3::ZERO;
        let dt = 1.0 / 60.0;
        for _ in 0..300 {
            solve_body_static(&mut body, &inertia, &mut position, radius, &colliders, gravity, dt);
        }

        let (distance, normal) = query_nearest_static(&colliders, position).unwrap();
        assert!((distance - radius).abs() < 0.05, "expected resting distance ~{radius}, got {distance}");
        assert!((normal - diagonal).length() < 0.05, "expected the corner's own outward normal, got {normal:?}");
    }

    #[test]
    fn a_sphere_resting_on_a_flat_face_near_a_corner_stays_settled_indefinitely() {
        // Regression test for a real bug caught during this stage's own
        // development: an earlier version of `solve_body_static` corrected
        // position against the constraint but then separately zeroed the
        // PRE-correction velocity's inward-normal component — inconsistent
        // with the position correction actually applied. Near a rounded
        // corner, where the contact normal changes direction from step to
        // step, that inconsistency injected a small spurious tangential
        // velocity every frame that individually looked negligible but
        // compounded over hundreds of frames into the sphere sliding all
        // the way across the curved region and off the floor's edge —
        // caught only by simulating far longer than the original
        // 600-frame settling test above did (which stops right as the
        // slide is only just beginning to accelerate).
        let half_extents = Vec3::new(6.0, 0.5, 6.0);
        let corner_radius = 0.4;
        let floor = StaticCollider {
            shape: PhysicsShape::RoundedBox { half_extents, corner_radius },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
        };
        let radius = 0.75;
        let inertia = Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ONE };
        let mut body = RigidBody::default();
        // Dropped above the flat top face, a little inward from the
        // corner — close enough that it grazes the rounded region while
        // falling, far enough from the corner's own apex that it settles
        // on the flat face rather than balancing on an unstable curve.
        let corner_edge = half_extents.x - corner_radius;
        let mut position = Vec3::new(corner_edge - 0.3, 4.0, corner_edge - 0.3);
        let colliders = [floor];
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 60.0;

        for _ in 0..600 {
            solve_body_static(&mut body, &inertia, &mut position, radius, &colliders, gravity, dt);
        }
        let settled_position = position;
        assert!(body.linear_velocity.length() < 1e-4, "expected the body to have fully settled by frame 600, velocity {:?}", body.linear_velocity);

        // The real regression check: keep simulating for a long time
        // afterward. A resting body must stay exactly put — any nonzero
        // drift here is the slow-compounding tangential-velocity leak this
        // test exists to catch.
        for _ in 0..600 {
            solve_body_static(&mut body, &inertia, &mut position, radius, &colliders, gravity, dt);
        }
        assert!(
            (position - settled_position).length() < 1e-4,
            "expected the settled body to stay exactly at rest, drifted from {settled_position:?} to {position:?}"
        );
        assert!(body.linear_velocity.length() < 1e-4, "expected velocity to remain near zero at rest, got {:?}", body.linear_velocity);
    }
}
