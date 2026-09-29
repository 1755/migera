//! The real per-frame physics step: gravity integration, contact
//! generation (dynamic-vs-dynamic AND dynamic-vs-static, both through
//! `physics::contacts::generate_contacts` — a static collider is modeled
//! as an ordinary body with `Inertia::STATIC`, i.e. zero inverse mass, so
//! `solve_rigid::solve_substep_jacobi` already treats it correctly with no
//! special-casing), and the XPBD substep solve loop.
//!
//! Supersedes `solve_static::solve_static_collisions`'s single-contact,
//! static-only path now that dynamic-vs-dynamic contacts exist too: a
//! stacked pyramid of boxes needs floor contacts and box-vs-box contacts
//! satisfied simultaneously, which only one shared Jacobi accumulator can
//! do correctly (two independent single-contact solves would fight each
//! other, each undoing the other's correction). `solve_static.rs` itself
//! is kept as a still-tested, narrower reference (single dynamic body vs.
//! static-only world) — see its own module doc comment.
//!
//! Broad-phase: dynamic-vs-dynamic candidate pairs come from
//! `broadphase::SpatialHash` (a uniform spatial hash, replacing an
//! earlier plain O(n²) all-pairs loop once the stage-3 BVH-under-motion
//! profiling milestone confirmed THAT loop — not BVH refit — was the real
//! bottleneck at scale: 2000 dynamic bodies became too slow for real
//! time). Dynamic-vs-static stays a small, separate O(dynamic × static)
//! loop — never the bottleneck the profiling found, and folding statics
//! into the same spatial hash would force it to span a huge floor's
//! extent in tiny cells sized for a small dynamic body, per
//! `broadphase`'s own doc comment on why statics are excluded from it
//! entirely. `hybrid::bvh::Bvh` itself remains unused for physics broad-
//! phase (entity-indexed, tightly coupled to the renderer's own object
//! list/extraction lifecycle, not a drop-in fit for physics' own body
//! indexing).

use bevy::prelude::*;

use super::broadphase::SpatialHash;
use super::components::{BodyKind, Inertia, PhysicsShape, RigidBody};
use super::contacts::{BodySnapshot, Contact, generate_contacts};
use super::solve_rigid::{BodyState, SubstepStart, solve_substep_jacobi};
use super::solve_static::PhysicsGravity;

/// How many XPBD substeps each `Update` tick runs — Müller et al.'s own
/// finding is that many cheap substeps with one iteration each beat one
/// big step with many iterations; this is squarely in their paper's
/// suggested range (4-10) as a starting point, not yet tuned against a
/// measured stability/cost tradeoff (that tuning is real, expected future
/// work per this stage's own risk classification — solver stability
/// tuning is flagged as "genuinely hard, high risk," not a one-shot pick).
pub(crate) const SUBSTEPS: u32 = 8;

/// `Update`-schedule system: collects every body (static colliders have no
/// `RigidBody`, so they're `Inertia::STATIC` with zero velocity by
/// construction — see `body_state_for_static` below), generates all
/// dynamic-vs-dynamic, dynamic-vs-kinematic, and dynamic-vs-static contacts
/// each substep (contacts must be regenerated per substep, not just once
/// per frame, since bodies move during the solve — confirmed necessary by
/// `solve_rigid`'s own multi-substep convergence test), and runs
/// `SUBSTEPS` XPBD substeps before writing the result back to `Transform`.
/// Registered `.before(bvh::update_persistent_bvh)` by `PhysicsPlugin`,
/// same ordering constraint every physics stage's authoritative-transform
/// write must respect.
///
/// Kinematic bodies (`BodyKind::Kinematic`) carry a real `RigidBody`
/// velocity (unlike statics, whose velocity is hardcoded to zero below)
/// but `Inertia::STATIC`-shaped mass, and their `Transform` is driven
/// externally (animation/script) — this system reads their pose/velocity
/// every frame exactly like a dynamic body, but never writes their
/// `Transform`/`RigidBody` back (see the write-back loop's own comment).
/// This is why kinematics still satisfy the same `dynamics` query shape
/// dynamic bodies do (both have `RigidBody`+`Inertia`+`PhysicsShape`+
/// `Transform`) rather than needing a separate query.
#[allow(clippy::type_complexity)]
pub fn solve_world(
    time: Res<Time>,
    gravity: Res<PhysicsGravity>,
    statics: Query<(&PhysicsShape, &GlobalTransform), Without<RigidBody>>,
    mut dynamics: Query<(&mut RigidBody, &Inertia, Option<&BodyKind>, &PhysicsShape, &mut Transform)>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    let substep_dt = dt / SUBSTEPS as f32;

    let static_shapes: Vec<(PhysicsShape, Vec3, Quat)> =
        statics.iter().map(|(shape, transform)| (*shape, transform.translation(), transform.rotation())).collect();
    let static_count = static_shapes.len();

    // Partition the single `dynamics` query into dynamic vs. kinematic
    // rows up front, in the same pass that will build `states`/`shapes`
    // below -- no extra query traversal beyond what was already paid.
    // `Option<&BodyKind>` + `unwrap_or_default()` (defaulting to
    // `Dynamic`) keeps every pre-existing spawn site (which never set
    // `BodyKind` at all) behaviorally unchanged.
    let mut dynamic_rows = Vec::new();
    let mut kinematic_rows = Vec::new();
    for (rigid_body, inertia, kind, shape, transform) in dynamics.iter_mut() {
        match kind.copied().unwrap_or_default() {
            BodyKind::Kinematic => kinematic_rows.push((rigid_body, inertia, shape, transform)),
            // A `BodyKind::Static` value should never actually appear
            // here in practice (a true static collider has no `RigidBody`
            // at all, so it never matches this query to begin with) --
            // treated the same as `Dynamic` defensively rather than
            // silently dropping the body, since dropping it would leave
            // it permanently un-simulated with no error signal.
            BodyKind::Dynamic | BodyKind::Static => dynamic_rows.push((rigid_body, inertia, shape, transform)),
        }
    }
    let dynamic_count = dynamic_rows.len();
    let kinematic_count = kinematic_rows.len();

    // Body index layout: dynamics first (indices 0..dynamic_count), then
    // kinematics (indices dynamic_count..dynamic_count+kinematic_count),
    // then statics (indices dynamic_count+kinematic_count..) — a fixed,
    // deterministic ordering so `generate_contacts`'s `body_a`/`body_b`
    // indices stay valid across the whole solve. Kinematics sit BETWEEN
    // dynamics and statics (not appended after statics) so dynamic-vs-
    // kinematic contact generation can reuse the exact same "small index
    // range past dynamic_count" shape dynamic-vs-static already uses,
    // just with a different upper bound.
    let mut states: Vec<BodyState> = Vec::with_capacity(dynamic_count + kinematic_count + static_count);
    let mut shapes: Vec<PhysicsShape> = Vec::with_capacity(dynamic_count + kinematic_count + static_count);

    for (rigid_body, inertia, shape, transform) in &dynamic_rows {
        states.push(BodyState::from_components(rigid_body, inertia, transform.translation, transform.rotation));
        shapes.push(**shape);
    }
    for (rigid_body, inertia, shape, transform) in &kinematic_rows {
        // Same constructor dynamics use -- a kinematic body's REAL,
        // externally-set velocity must flow into `BodyState` (unlike a
        // static's hardcoded-zero velocity below), so
        // `resolve_contact_velocities`'s existing, unmodified relative-
        // velocity math correctly computes "dynamic body's velocity minus
        // the moving platform's velocity." `inertia` is expected to carry
        // `inverse_mass == 0.0` (typically `Inertia::STATIC`) so the same
        // zero-inverse-mass gates that already make statics immovable
        // make this body immovable by corrections too, with no new code
        // in `solve_rigid.rs`.
        states.push(BodyState::from_components(rigid_body, inertia, transform.translation, transform.rotation));
        shapes.push(**shape);
    }
    for &(shape, translation, rotation) in &static_shapes {
        states.push(BodyState {
            position: translation,
            rotation,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inverse_mass: 0.0,
            inverse_inertia_local: Vec3::ZERO,
        });
        shapes.push(shape);
    }

    if states.is_empty() {
        return;
    }

    for _ in 0..SUBSTEPS {
        // Snapshot each body's full state BEFORE this substep's own
        // gravity/rotation prediction runs — the true XPBD substep-start
        // reference `solve_substep_jacobi` needs to derive a correct
        // final velocity as `pre_integration_velocity + correction / dt`
        // (see `SubstepStart`'s own doc comment for why this must be
        // tracked directly rather than derived from a position delta:
        // that approach is algebraically equivalent but numerically
        // disastrous at realistic world-scale positions, losing almost
        // all of gravity's own per-substep delta to floating-point
        // cancellation against a large position value — confirmed via an
        // isolated trace reproducing a real test failure, a 7.5% error
        // from pure precision loss, not a logic bug). A real,
        // regression-tested bug also lived in an even earlier version:
        // deriving velocity as `existing_velocity += correction / dt`
        // against the CURRENT (already gravity-advanced) velocity double-
        // counts the fast inbound velocity that caused a hard landing's
        // deep penetration in the first place, producing a physically
        // wrong bounce (confirmed via a standalone trace: -6.9 m/s in
        // became +10+ m/s out in one substep) instead of the correction
        // naturally absorbing it.
        let substep_start: Vec<SubstepStart> = states
            .iter()
            .map(|s| SubstepStart {
                position: s.position,
                rotation: s.rotation,
                linear_velocity: s.linear_velocity,
                angular_velocity: s.angular_velocity,
            })
            .collect();

        // Gravity integration: only dynamic bodies (nonzero inverse mass)
        // accelerate; statics stay exactly put by construction. Direction
        // is recomputed from each body's OWN current position every
        // substep (not just once per frame) since a point-source gravity
        // field's direction genuinely changes as a body moves relative to
        // the center — most visible for bodies orbiting or resting on a
        // small planetary body, negligible (by design, see
        // `PhysicsGravity::default`'s own doc comment) for a distant
        // center approximating flat gravity.
        for state in &mut states {
            if state.inverse_mass > 0.0 {
                let g = gravity.acceleration_at(state.position);
                state.linear_velocity += g * substep_dt;
                state.position += state.linear_velocity * substep_dt;
            }
            // Predict rotation from the body's current angular velocity —
            // the rotational analogue of the linear prediction above, and
            // the step that was MISSING entirely before this fix: without
            // it, angular_velocity accumulated corrections every substep
            // with nothing ever consuming it to actually advance
            // `rotation`, so it grew unboundedly instead of settling
            // (confirmed via a standalone trace: angular_velocity raced
            // to large magnitudes while rotation barely moved, the exact
            // signature of a correction feeding a value nothing drains).
            // Gated on inverse_inertia (not inverse_mass) being nonzero on
            // at least one axis, matching linear prediction's own
            // inverse_mass gate — a body with zero inverse inertia must
            // never rotate at all, predicted or otherwise.
            if state.inverse_inertia_local != Vec3::ZERO {
                let half_dt_omega = state.angular_velocity * (0.5 * substep_dt);
                let delta_q = Quat::from_xyzw(half_dt_omega.x, half_dt_omega.y, half_dt_omega.z, 0.0);
                let derivative = delta_q * state.rotation;
                let predicted = Vec4::from(state.rotation) + Vec4::from(derivative);
                state.rotation = Quat::from_vec4(predicted).normalize();
            }
        }

        let contacts = generate_all_contacts(&states, &shapes, dynamic_count, kinematic_count);
        solve_substep_jacobi(&mut states, &contacts, substep_dt, &substep_start);
    }

    // Kinematics are read-only outputs of the solver: they impart velocity
    // to dynamics they contact, but their own `Transform`/`RigidBody` are
    // never written here (their `Transform` is driven externally, and
    // their `RigidBody` velocity is maintained by whatever drives that
    // Transform, not by this solver) -- so only `0..dynamic_count` of
    // `states` is ever written back, and only `dynamic_rows` is iterated,
    // not `kinematic_rows`.
    for (i, (mut rigid_body, _inertia, _shape, mut transform)) in dynamic_rows.into_iter().enumerate() {
        rigid_body.linear_velocity = states[i].linear_velocity;
        rigid_body.angular_velocity = states[i].angular_velocity;
        transform.translation = states[i].position;
        transform.rotation = states[i].rotation;
    }
}

/// Dynamic-vs-dynamic contacts via `broadphase::SpatialHash` (indices
/// `0..dynamic_count`), dynamic-vs-kinematic contacts via a small, direct
/// `dynamic_count * kinematic_count` loop (indices
/// `dynamic_count..dynamic_count+kinematic_count`), and dynamic-vs-static
/// contacts via a small, direct `dynamic_count * static_count` loop
/// (indices `dynamic_count+kinematic_count..`) — none of these three loops
/// are the bottleneck the profiling milestone found, and folding
/// kinematics or statics into the dynamic-sized spatial hash would force
/// it to span a large collider's extent in cells sized for a small
/// dynamic body — see this module's own doc comment (statics) and Stage
/// 3.5's own plan section (kinematics: the same big-mover/small-cell
/// sizing mismatch argument applies equally to a large moving platform).
///
/// Deliberately NO kinematic-vs-kinematic or kinematic-vs-static loop:
/// both pairs have `inverse_mass == 0.0` on both sides, so
/// `solve_substep_jacobi`'s own `if total_w <= 0.0 { continue; }`
/// early-out already makes any such contact a guaranteed no-op —
/// generating them would be pure wasted sample-point-query work, the same
/// reasoning that already makes static-vs-static contacts unnecessary
/// (see `two_overlapping_static_bodies_never_produce_a_contact` below).
///
/// A pair found twice by the spatial hash (its own documented duplicate-
/// candidate behavior near aliased buckets) simply produces the same
/// contact set twice, which `solve_substep_jacobi`'s per-body averaging
/// already tolerates the same way multiple sample points do — no dedup
/// needed for correctness, only for avoiding redundant work, which isn't
/// this stage's concern yet.
fn generate_all_contacts(states: &[BodyState], shapes: &[PhysicsShape], dynamic_count: usize, kinematic_count: usize) -> Vec<Contact> {
    let mut contacts = Vec::new();

    if dynamic_count > 0 {
        let dynamic_positions: Vec<Vec3> = states[..dynamic_count].iter().map(|s| s.position).collect();
        // Cell size ~2x the largest dynamic body's own bounding radius —
        // Müller's own sizing convention, guaranteeing the 27-cell
        // neighborhood search finds every truly-overlapping pair.
        let max_radius = shapes[..dynamic_count]
            .iter()
            .map(super::solve_static::bounding_radius)
            .fold(0.0f32, f32::max)
            .max(0.01);
        let hash = SpatialHash::build(&dynamic_positions, max_radius * 2.0);

        for i in 0..dynamic_count {
            for candidate in hash.query_candidates(&dynamic_positions, i) {
                let j = candidate as usize;
                if j <= i {
                    // Each unordered pair is generated once, from the
                    // lower index's own query -- `j > i` here means this
                    // is the first (and only intended) time this pair is
                    // processed from this side; skipping `j <= i` avoids
                    // generating the same pair's contacts twice from BOTH
                    // bodies' own neighbor queries (the hash reports
                    // candidates symmetrically).
                    continue;
                }
                let snap_a = BodySnapshot { shape: shapes[i], translation: states[i].position, rotation: states[i].rotation };
                let snap_b = BodySnapshot { shape: shapes[j], translation: states[j].position, rotation: states[j].rotation };
                contacts.extend(generate_contacts(i as u32, &snap_a, j as u32, &snap_b));
            }
        }
    }

    for i in 0..dynamic_count {
        for j in dynamic_count..dynamic_count + kinematic_count {
            let snap_a = BodySnapshot { shape: shapes[i], translation: states[i].position, rotation: states[i].rotation };
            let snap_b = BodySnapshot { shape: shapes[j], translation: states[j].position, rotation: states[j].rotation };
            contacts.extend(generate_contacts(i as u32, &snap_a, j as u32, &snap_b));
        }
    }

    for i in 0..dynamic_count {
        for j in dynamic_count + kinematic_count..states.len() {
            let snap_a = BodySnapshot { shape: shapes[i], translation: states[i].position, rotation: states[i].rotation };
            let snap_b = BodySnapshot { shape: shapes[j], translation: states[j].position, rotation: states[j].rotation };
            contacts.extend(generate_contacts(i as u32, &snap_a, j as u32, &snap_b));
        }
    }

    contacts
}

#[cfg(test)]
mod tests {
    use super::super::components::PhysicsShape;
    use super::*;

    fn body(position: Vec3, inverse_mass: f32) -> BodyState {
        BodyState {
            position,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inverse_mass,
            inverse_inertia_local: Vec3::ZERO,
        }
    }

    #[test]
    fn two_overlapping_static_bodies_never_produce_a_contact() {
        // Two overlapping static bodies (dynamic_count = 0) should never
        // produce a contact through this function, even though
        // generate_contacts itself would happily report the overlap --
        // the pure-static index range is never iterated by either the
        // spatial-hash loop (built only from the dynamic sub-slice) or
        // the dynamic-vs-static loop (whose outer range is
        // `0..dynamic_count`, empty here), not a correctness requirement
        // in itself but worth pinning so a future change doesn't silently
        // reintroduce wasted static-vs-static sample-point queries.
        let states = vec![body(Vec3::ZERO, 0.0), body(Vec3::new(0.5, 0.0, 0.0), 0.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        assert!(generate_all_contacts(&states, &shapes, 0, 0).is_empty());
    }

    #[test]
    fn dynamic_vs_static_overlap_is_still_caught() {
        let states = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(0.5, 0.0, 0.0), 0.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        assert!(!generate_all_contacts(&states, &shapes, 1, 0).is_empty());
    }

    #[test]
    fn dynamic_vs_dynamic_overlap_via_the_spatial_hash_is_still_caught() {
        let states = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(0.5, 0.0, 0.0), 1.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        assert!(!generate_all_contacts(&states, &shapes, 2, 0).is_empty());
    }

    #[test]
    fn each_dynamic_pair_is_only_processed_once_not_twice() {
        // The i/j > i guard in the spatial-hash loop must not cause a
        // pair to be silently dropped -- confirms both "found" and "found
        // exactly once" (via depth: doubling a contact's correction would
        // silently break `solve_substep_jacobi`'s per-body averaging math
        // even though "not empty" alone wouldn't catch it).
        let states = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.5, 0.0, 0.0), 1.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        let contacts = generate_all_contacts(&states, &shapes, 2, 0);
        // Compare against calling `contacts::generate_contacts` directly
        // ONCE for this pair -- that's the ground truth for "one pair,
        // processed exactly once." `generate_all_contacts`'s own outer
        // loop calling `generate_contacts` on the same pair a SECOND time
        // (e.g. once from body 0's neighbor query and again from body 1's,
        // since the hash reports candidates symmetrically) would double
        // this count.
        let snap_a = BodySnapshot { shape: shapes[0], translation: states[0].position, rotation: states[0].rotation };
        let snap_b = BodySnapshot { shape: shapes[1], translation: states[1].position, rotation: states[1].rotation };
        let expected = generate_contacts(0, &snap_a, 1, &snap_b);
        assert_eq!(contacts.len(), expected.len(), "expected the pair to be processed exactly once (matching one direct generate_contacts call), got {} contacts vs {} expected", contacts.len(), expected.len());
    }

    #[test]
    fn a_tilted_box_settling_through_the_real_solve_world_system_has_bounded_angular_velocity() {
        // Regression test for a real bug caught during this stage's own
        // development: `solve_world`'s own per-substep loop integrated
        // LINEAR velocity into position every substep, but had no
        // equivalent step integrating ANGULAR velocity into rotation --
        // so `solve_substep_jacobi`'s angular corrections kept
        // accumulating into `angular_velocity` every substep (since the
        // underlying tilt they were correcting for never actually
        // resolved, nothing ever consumed the correction), producing
        // unboundedly growing angular velocity and, transitively, bodies
        // flying off into space once that runaway spin fed back into
        // linear motion through contacts. Fixed by adding rotation
        // prediction (`rotation += 0.5 * dt * angular_velocity * rotation`,
        // the rotational analogue of the linear `position += dt *
        // velocity`) to the SAME per-substep loop that already predicts
        // linear position -- confirmed here through the real ECS system
        // `solve_world`, not just `solve_substep_jacobi` in isolation
        // (which doesn't exercise the loop where the bug actually lived).
        use bevy::time::TimeUpdateStrategy;

        use super::super::inertia::box_inertia;
        use super::super::integrate::PhysicsPlugin;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(PhysicsPlugin);
        app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });

        let half_extents = Vec3::splat(0.5);
        let inverse_inertia = 1.0 / box_inertia(1.0, half_extents);

        app.world_mut().spawn((
            PhysicsShape::RoundedBox { half_extents: Vec3::new(10.0, 0.5, 10.0), corner_radius: 0.0 },
            Transform::from_xyz(0.0, -0.5, 0.0),
        ));
        let entity = app
            .world_mut()
            .spawn((
                PhysicsShape::RoundedBox { half_extents, corner_radius: 0.0 },
                Transform { translation: Vec3::new(0.0, 0.55, 0.0), rotation: Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, 0.1), ..default() },
                RigidBody::default(),
                Inertia { inverse_mass: 1.0, inverse_tensor_diag: inverse_inertia },
            ))
            .id();

        app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));
        for _ in 0..300 {
            app.update();
        }

        let rigid_body = app.world().get::<RigidBody>(entity).unwrap();
        let transform = app.world().get::<Transform>(entity).unwrap();
        assert!(
            rigid_body.angular_velocity.length() < 1.0,
            "expected bounded angular velocity once the box settles, got {:?} -- unbounded growth is the exact regression this test guards against",
            rigid_body.angular_velocity
        );
        assert!(transform.translation.is_finite(), "expected a finite resting position, got {:?}", transform.translation);
        assert!(transform.translation.length() < 5.0, "expected the box to stay near the floor, not fly off, got {:?}", transform.translation);
    }

    #[test]
    fn a_body_launched_at_orbital_velocity_completes_a_stable_orbit() {
        // A body launched tangent to a point-source gravity center at the
        // circular-orbit speed for its altitude (`v = sqrt(g * r)`,
        // standard orbital mechanics for a uniform-magnitude central
        // field) must trace out a roughly circular path and return close
        // to its own starting position after one period -- fully resolved
        // by the same `solve_world` gravity integration + XPBD substep
        // loop every other body in this module goes through, no scripted
        // trajectory. No static collider exists in this scene at all
        // (deliberately -- an orbit test has nothing to rest on; the
        // whole point is staying aloft under gravity alone), so this also
        // exercises the zero-contact free-flight path over many frames
        // continuously, unlike the settling tests above which quickly
        // reach a contact-dominated steady state.
        use bevy::time::TimeUpdateStrategy;

        use super::super::integrate::PhysicsPlugin;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(PhysicsPlugin);

        let magnitude = 9.81;
        let radius = 10.0;
        let gravity = PhysicsGravity { center: Vec3::ZERO, magnitude };
        app.insert_resource(gravity);

        // Circular-orbit speed at this altitude, tangent to the radius
        // vector -- point-source gravity here has CONSTANT magnitude
        // regardless of distance (`PhysicsGravity::acceleration_at`'s own
        // doc comment), so the standard `v_circular = sqrt(g * r)`
        // derivation (centripetal acceleration `v^2 / r` must equal `g`)
        // applies directly, no inverse-square correction needed.
        let orbital_speed = (magnitude * radius).sqrt();
        let start_position = Vec3::new(radius, 0.0, 0.0);
        let orbital_velocity = Vec3::new(0.0, 0.0, orbital_speed);

        let entity = app
            .world_mut()
            .spawn((
                PhysicsShape::Sphere { radius: 0.3 },
                Transform::from_translation(start_position),
                RigidBody { linear_velocity: orbital_velocity, angular_velocity: Vec3::ZERO },
                Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ZERO },
            ))
            .id();

        // Orbital period `T = 2*pi*sqrt(r / g)` (from `v = sqrt(g*r)` and
        // `T = 2*pi*r / v`) -- run for slightly more than one full period
        // so the body has a chance to complete the loop and start back
        // toward its own starting point.
        let period = std::f32::consts::TAU * (radius / magnitude).sqrt();
        let dt = 1.0 / 120.0;
        let steps = (period / dt * 1.05) as u32;

        app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(dt as f64)));

        let mut max_radius_deviation: f32 = 0.0;
        let mut min_distance_to_start_after_half_period: f32 = f32::MAX;
        let half_period_step = steps / 2;
        for step in 0..steps {
            app.update();
            let transform = app.world().get::<Transform>(entity).unwrap();
            assert!(transform.translation.is_finite(), "expected a finite orbit position at step {step}, got {:?}", transform.translation);
            let current_radius = transform.translation.length();
            max_radius_deviation = max_radius_deviation.max((current_radius - radius).abs());
            if step >= half_period_step {
                min_distance_to_start_after_half_period = min_distance_to_start_after_half_period.min((transform.translation - start_position).length());
            }
        }

        // A stable orbit stays close to its starting radius throughout --
        // this is the actual "didn't spiral into the center or fly off to
        // infinity" check. XPBD's own discretization error (finite
        // substeps approximating continuous circular motion) means this
        // won't be bit-exact, so the tolerance is generous (30% of the
        // orbital radius) while still being far tighter than "didn't
        // explode" alone would catch.
        assert!(
            max_radius_deviation < radius * 0.3,
            "expected the orbit to stay close to its starting radius {radius} throughout, max deviation was {max_radius_deviation}"
        );
        // And it must actually come back around near its starting point
        // within the run -- proof this is a closed orbit, not just "the
        // radius happened to stay bounded while drifting off in angle."
        assert!(
            min_distance_to_start_after_half_period < radius * 0.5,
            "expected the orbit to return near its starting position {start_position:?} after roughly one period, closest approach was {min_distance_to_start_after_half_period}"
        );
    }

    #[test]
    fn a_dynamic_body_resting_on_a_stationary_kinematic_platform_behaves_like_resting_on_a_static_one() {
        // A `BodyKind::Kinematic` platform at zero velocity must be
        // functionally indistinguishable from a true static floor for a
        // resting dynamic body -- same scene built two ways (static floor
        // vs. stationary kinematic floor), same near-identical resting
        // state after the same number of frames.
        use bevy::time::TimeUpdateStrategy;

        use super::super::integrate::PhysicsPlugin;

        fn settle(platform_is_kinematic: bool) -> Vec3 {
            let mut app = App::new();
            app.add_plugins(MinimalPlugins);
            app.add_plugins(TransformPlugin);
            app.add_plugins(PhysicsPlugin);
            app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });

            let mut platform = app.world_mut().spawn((
                PhysicsShape::RoundedBox { half_extents: Vec3::new(10.0, 0.5, 10.0), corner_radius: 0.0 },
                Transform::from_xyz(0.0, -0.5, 0.0),
            ));
            if platform_is_kinematic {
                platform.insert((RigidBody::default(), Inertia::STATIC, BodyKind::Kinematic));
            }

            let entity = app
                .world_mut()
                .spawn((
                    PhysicsShape::Sphere { radius: 0.5 },
                    Transform::from_xyz(0.0, 2.0, 0.0),
                    RigidBody::default(),
                    Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ZERO },
                ))
                .id();

            app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));
            for _ in 0..300 {
                app.update();
            }
            app.world().get::<Transform>(entity).unwrap().translation
        }

        let resting_on_static = settle(false);
        let resting_on_kinematic = settle(true);
        assert!(
            (resting_on_static - resting_on_kinematic).length() < 0.01,
            "expected resting on a stationary kinematic platform to match resting on a static one within 1cm, got static={resting_on_static:?} kinematic={resting_on_kinematic:?}"
        );
    }

    #[test]
    fn a_moving_kinematic_platform_pushes_a_resting_dynamic_body_upward_with_it() {
        // The single most important new-behavior test for kinematic
        // bodies: a platform rising at a constant velocity must carry a
        // resting dynamic body up with it. The platform's own `Transform`
        // and `RigidBody.linear_velocity` are driven manually every frame
        // here, standing in for what a real animation/script system would
        // do -- `solve_world` itself never derives kinematic velocity from
        // Transform deltas (see `RigidBody`'s own doc comment for why).
        //
        // Uses a flat-bottomed box (multi-sample-point contact, like the
        // existing settling tests above) rather than a sphere for the
        // riding body -- a sphere's single-point-of-contact geometry was
        // found, via this test's own development, to intermittently
        // separate-then-resnap against a slowly rising platform (contact
        // depth crossing the detection threshold once per few frames
        // produces a brief free-fall + snap-correction cycle, a real but
        // separate sample-point/contact-generation sensitivity, not a
        // kinematic-support bug -- a box's flat multi-point contact stays
        // continuously engaged instead).
        use bevy::time::TimeUpdateStrategy;

        use super::super::integrate::PhysicsPlugin;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(PhysicsPlugin);
        app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });

        // Deliberately modest: at higher platform speeds, the once-per-
        // frame (not once-per-substep) sampling of a kinematic body's
        // Transform/velocity means a full frame's worth of platform
        // displacement is presented to the solver as an instantaneous
        // jump at the start of the frame, and XPBD's `correction / dt`
        // velocity derivation (`solve_rigid.rs`) amplifies a correction
        // sized for a full frame by dividing it by a single substep's
        // much smaller `dt` -- confirmed via this test's own development
        // (a 1.0 m/s platform produced a resonant launch-and-refall cycle
        // instead of smooth tracking). This is a real, pre-existing
        // property of sampling kinematic input once per frame rather than
        // once per substep (a limitation of THIS solver's granularity,
        // not specific to kinematics -- any contact whose depth jumps by
        // a large amount between one substep and the next would see the
        // same amplification), not a bug introduced by kinematic body
        // support itself -- tracked as a known limitation rather than
        // silently worked around.
        let platform_lift_speed = 0.3;
        let platform = app
            .world_mut()
            .spawn((
                PhysicsShape::RoundedBox { half_extents: Vec3::new(10.0, 0.5, 10.0), corner_radius: 0.0 },
                Transform::from_xyz(0.0, -0.5, 0.0),
                RigidBody { linear_velocity: Vec3::new(0.0, platform_lift_speed, 0.0), angular_velocity: Vec3::ZERO },
                Inertia::STATIC,
                BodyKind::Kinematic,
            ))
            .id();

        let half_extents = Vec3::splat(0.5);
        let inverse_inertia = 1.0 / super::super::inertia::box_inertia(1.0, half_extents);
        let dynamic_entity = app
            .world_mut()
            .spawn((
                PhysicsShape::RoundedBox { half_extents, corner_radius: 0.0 },
                Transform::from_xyz(0.0, 2.0, 0.0),
                RigidBody::default(),
                Inertia { inverse_mass: 1.0, inverse_tensor_diag: inverse_inertia },
            ))
            .id();

        let dt = 1.0 / 60.0;
        app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(dt as f64)));
        // Let the box settle onto the (still stationary at this point)
        // platform first, exactly like the resting-parity test above.
        for _ in 0..300 {
            app.update();
        }
        let resting_translation = app.world().get::<Transform>(dynamic_entity).unwrap().translation;

        // Now drive the platform upward for a further fixed duration,
        // manually advancing its Transform + RigidBody velocity every
        // frame the way an external animation system would.
        let lift_frames = 60;
        for _ in 0..lift_frames {
            let mut platform_transform = app.world_mut().get_mut::<Transform>(platform).unwrap();
            platform_transform.translation.y += platform_lift_speed * dt;
            app.update();
        }

        let platform_translation = app.world().get::<Transform>(platform).unwrap().translation;
        let final_translation = app.world().get::<Transform>(dynamic_entity).unwrap().translation;
        let expected_rise = platform_lift_speed * dt * lift_frames as f32;

        assert!(
            (final_translation.y - resting_translation.y - expected_rise).abs() < 0.2,
            "expected the resting body's Y to track the platform's rise of {expected_rise} within a bounded tolerance, started at {}, ended at {}, platform ended at {}",
            resting_translation.y,
            final_translation.y,
            platform_translation.y
        );
        assert!(final_translation.is_finite(), "expected a finite position while riding the platform, got {final_translation:?}");
    }

    #[test]
    fn a_kinematic_bodys_own_transform_is_never_mutated_by_solve_world_regardless_of_contacts() {
        // Bit-exact before/after despite an active, penetrating contact
        // against a dynamic body, run through the real `solve_world`
        // system -- the single most important regression guard for this
        // whole feature, per the plan's own emphasis.
        use bevy::time::TimeUpdateStrategy;

        use super::super::integrate::PhysicsPlugin;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(PhysicsPlugin);
        app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });

        let kinematic_transform = Transform::from_xyz(0.0, 0.0, 0.0);
        let kinematic_entity = app
            .world_mut()
            .spawn((
                PhysicsShape::Sphere { radius: 1.0 },
                kinematic_transform,
                RigidBody::default(),
                Inertia::STATIC,
                BodyKind::Kinematic,
            ))
            .id();

        // Spawn the dynamic body already deeply overlapping the kinematic
        // body, guaranteeing an active penetrating contact from frame one.
        app.world_mut().spawn((
            PhysicsShape::Sphere { radius: 1.0 },
            Transform::from_xyz(0.5, 0.0, 0.0),
            RigidBody::default(),
            Inertia { inverse_mass: 1.0, inverse_tensor_diag: Vec3::ZERO },
        ));

        app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));
        for _ in 0..60 {
            app.update();
        }

        let kinematic_rigid_body = app.world().get::<RigidBody>(kinematic_entity).unwrap();
        let kinematic_transform_after = app.world().get::<Transform>(kinematic_entity).unwrap();
        assert_eq!(kinematic_transform_after.translation, kinematic_transform.translation, "expected a kinematic body's Transform translation to never be mutated by solve_world");
        assert_eq!(kinematic_transform_after.rotation, kinematic_transform.rotation, "expected a kinematic body's Transform rotation to never be mutated by solve_world");
        assert_eq!(kinematic_rigid_body.linear_velocity, Vec3::ZERO, "expected a kinematic body's RigidBody velocity to never be mutated by solve_world (it stays whatever the external driver last set)");
    }

    #[test]
    fn two_overlapping_kinematic_bodies_never_produce_a_contact() {
        // Mirrors two_overlapping_static_bodies_never_produce_a_contact:
        // dynamic_count = 0, kinematic_count = 2 -- the kinematic-only
        // index range must never be iterated by any of the three loops.
        let states = vec![body(Vec3::ZERO, 0.0), body(Vec3::new(0.5, 0.0, 0.0), 0.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        assert!(generate_all_contacts(&states, &shapes, 0, 2).is_empty());
    }

    #[test]
    fn dynamic_vs_kinematic_overlap_is_still_caught() {
        let states = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(0.5, 0.0, 0.0), 0.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        assert!(!generate_all_contacts(&states, &shapes, 1, 1).is_empty());
    }

    #[test]
    fn a_kinematic_body_never_produces_a_contact_against_a_static_body() {
        // dynamic_count = 0, kinematic_count = 1 -- the remaining index is
        // static. Neither the (empty) spatial-hash loop, the (empty)
        // dynamic-vs-kinematic loop, nor the (empty, since its outer range
        // is 0..dynamic_count) dynamic-vs-static loop ever compares the
        // kinematic body against the static one.
        let states = vec![body(Vec3::ZERO, 0.0), body(Vec3::new(0.5, 0.0, 0.0), 0.0)];
        let shapes = vec![PhysicsShape::Sphere { radius: 1.0 }, PhysicsShape::Sphere { radius: 1.0 }];
        assert!(generate_all_contacts(&states, &shapes, 0, 1).is_empty());
    }

    #[test]
    fn gravity_never_accelerates_a_kinematic_body() {
        // The kinematic analogue of integrate::tests::a_stationary_body_never_moves
        // -- across many frames with no external Transform/RigidBody
        // writes at all, a kinematic body must stay exactly where it
        // started despite PhysicsGravity being active in the scene.
        use bevy::time::TimeUpdateStrategy;

        use super::super::integrate::PhysicsPlugin;

        let mut app = App::new();
        app.add_plugins(MinimalPlugins);
        app.add_plugins(TransformPlugin);
        app.add_plugins(PhysicsPlugin);
        app.insert_resource(PhysicsGravity { center: Vec3::new(0.0, -10_000.0, 0.0), magnitude: 9.81 });

        let start = Transform::from_xyz(0.0, 5.0, 0.0);
        let entity = app.world_mut().spawn((PhysicsShape::Sphere { radius: 0.5 }, start, RigidBody::default(), Inertia::STATIC, BodyKind::Kinematic)).id();

        app.insert_resource(TimeUpdateStrategy::ManualDuration(std::time::Duration::from_secs_f64(1.0 / 60.0)));
        for _ in 0..300 {
            app.update();
        }

        let rigid_body = app.world().get::<RigidBody>(entity).unwrap();
        let transform = app.world().get::<Transform>(entity).unwrap();
        assert_eq!(transform.translation, start.translation, "expected gravity to never move a kinematic body");
        assert_eq!(rigid_body.linear_velocity, Vec3::ZERO, "expected gravity to never accelerate a kinematic body's velocity");
    }
}
