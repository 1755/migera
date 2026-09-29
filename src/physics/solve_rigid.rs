//! Rigid-vs-rigid XPBD substep solver, Jacobi-style: many small substeps
//! with one solver iteration each (Müller, Macklin et al., "Detailed Rigid
//! Body Simulation with Extended Position Based Dynamics," SCA/CGF 2020 —
//! substeps beat iterating one big step, and this substep structure is
//! inherently GPU-friendly: each substep is a cheap, uniform, parallel
//! kernel). Every contact independently computes a positional AND angular
//! correction and SCATTERS both into a per-body accumulator (Macklin,
//! "Unified Particle Physics for Real-Time Applications," SIGGRAPH — the
//! same Jacobi+substep pattern this reference models on the GPU's own
//! eventual `atomicAdd`-then-divide two-pass shape, chosen over graph-
//! coloring Gauss-Seidel because it needs no coloring/graph structure at
//! all, making it the only approach implementable correctly on a first
//! pass by a small team per this stage's own SOTA research); a second pass
//! divides by contact count and applies the averaged correction to both
//! position/linear-velocity and rotation/angular-velocity.
//!
//! This CPU reference deliberately mirrors the GPU algorithm's STRUCTURE
//! (accumulate-then-divide, not a serial per-contact loop that mutates
//! body state immediately) — per `src/hybrid`'s own CPU-ref-first
//! doctrine, "a correct reference" is insufficient if it's not the SAME
//! algorithm the WGSL port will implement; a serial Gauss-Seidel-style
//! reference would converge differently and give the WGSL port nothing
//! meaningful to test against.
//!
//! Angular contact response (Müller et al. 2020, §3.3-3.5): a contact at
//! world point `p` with normal `n` and depth `d` generates a torque on
//! each body proportional to the lever arm `r = p - center_of_mass`. The
//! GENERALIZED inverse mass for a positional constraint on a rotating body
//! is `w = inverse_mass + (r × n)ᵀ · inverse_inertia_world · (r × n)` —
//! not just `inverse_mass` alone — since pushing a point far from the
//! center of mass rotates the body more easily than pushing through the
//! center. The rotation correction itself is the standard XPBD quaternion
//! increment `Δq = 0.5 · [inverse_inertia_world · (r × (λn)), 0] · q`,
//! applied as `q_new = normalize(q + Δq)` (a linearized small-angle
//! update, valid for the small per-substep rotations XPBD produces with
//! enough substeps — the same "many small steps" assumption the whole
//! substep scheme already relies on for the linear case).
//!
//! `BodyKind::Kinematic` support (see `physics::components::BodyKind`)
//! needed ZERO changes in this file. `solve_substep_jacobi`'s position/
//! rotation correction already excludes any body from ever receiving an
//! accumulator contribution whenever `inverse_mass == 0.0`/
//! `inverse_inertia_local == Vec3::ZERO` — exactly the same gate that
//! already makes statics immovable, and a kinematic body is built with
//! `Inertia::STATIC` specifically so it trips that same gate.
//! `resolve_contact_velocities`'s relative-velocity read also needed no
//! change: it already reads `bodies[ia].linear_velocity`/
//! `angular_velocity` directly off the `BodyState` slice with no
//! hardcoded-zero special case for zero-inverse-mass bodies — a static
//! only ever ends up with zero velocity because `solve_world.rs`
//! *constructs* its `BodyState` that way. Feed this file a kinematic
//! body's real, externally-set velocity instead (`solve_world.rs` does,
//! via `BodyState::from_components`) and this file's existing,
//! unmodified relative-velocity math already computes "dynamic body's
//! velocity minus the moving platform's velocity" correctly, while the
//! `inv_mass_a > 0.0`/`inv_mass_b > 0.0` gates still ensure 100% of any
//! impulse lands only on the dynamic side.

use bevy::prelude::*;

use super::components::{Inertia, RigidBody};
use super::contacts::Contact;

/// Per-substep linear velocity damping factor (applied as `v *= 1.0 -
/// LINEAR_DAMPING * dt` after every solve) — standard XPBD practice to
/// bleed off small numerical oscillation, not a physically-motivated drag
/// force. Kept tiny (barely perceptible over a real settling timescale)
/// since its job is dissipating solver noise, not simulating air
/// resistance.
pub(crate) const LINEAR_DAMPING: f32 = 0.02;

/// Hard ceiling on angular velocity magnitude (radians/second), applied
/// after every substep's correction+damping — a direct clamp on the
/// STATE itself, not just the per-substep correction that produces it
/// (see `MAX_ANGULAR_CORRECTION`'s own doc comment for why a correction-
/// level clamp alone proved insufficient: once `angular_velocity` is
/// already large from an earlier substep, `solve_world`'s own rotation
/// PREDICTION step — `rotation += 0.5 * dt * angular_velocity * rotation`
/// — swings the body's predicted orientation far enough that a
/// previously-uninvolved corner can swing deep into the floor, producing
/// a large but geometrically "real" depth that then justifies an even
/// bigger correction next substep; damping/clamping the correction alone
/// never breaks that feedback loop once velocity itself is already large,
/// so the state must be clamped directly). 20 rad/s (~3.2 revolutions/
/// second) is generous for any plausible resting-object rotation while
/// still ruling out the run away values (~10+ rad/s) the regression test
/// below was built to catch.
pub(crate) const MAX_ANGULAR_VELOCITY: f32 = 20.0;

/// Hard ceiling on linear velocity magnitude (meters/second), applied
/// after every substep's correction+damping, mirroring
/// `MAX_ANGULAR_VELOCITY`'s own reasoning exactly: a direct clamp on the
/// STATE itself, not just the per-substep correction that produces it.
/// `MAX_LINEAR_CORRECTION` alone (a clamp on `correction` BEFORE dividing
/// by `dt` to derive velocity, see `solve_substep_jacobi`'s own comment on
/// that clamp) does not bound the DERIVED velocity — dividing even a
/// correctly-clamped correction by a small `dt` still produces an
/// arbitrarily large velocity, and the smaller `dt` is (more substeps, or
/// a real-time-driven `Time::delta_secs()` producing an unusually small
/// per-substep timestep), the worse this gets. A real, regression-tested
/// bug lived exactly here: a resting dynamic body atop a kinematic
/// platform, penetrated by a moderate ~0.1-0.15m depth (built up externally
/// while the platform's own driving system kept moving it during a window
/// where physics dispatch itself hadn't started yet — a real, separate,
/// documented condition, see `physics_stability.rs`'s own kinematic-
/// platform milestone), resolved that single penetration into an
/// instantaneous ~50-60 m/s upward launch once physics resumed — the
/// SAME `correction / dt` amplification `MAX_ANGULAR_VELOCITY`'s own doc
/// comment already names for the angular case, just never closed off for
/// the linear one. 50 m/s (~180 km/h) is generous for any plausible
/// resting/settling/orbiting scene this engine's existing tests exercise
/// (the fastest legitimate velocity on record, the orbital-velocity test,
/// stays under 10 m/s) while still ruling out a runaway launch. Picked at
/// 15 m/s rather than a larger value after this exact bug's own
/// reproduction showed a 50 m/s clamp still visibly launched a resting
/// body tens of units into the air before falling back (physically
/// "correct" under the clamp, but still an obviously-wrong one-substep
/// snap far outside anything this engine's own settling/stacking/soak
/// milestones ever produce) -- 15 m/s comfortably covers every legitimate
/// velocity any existing test reaches while keeping a pathological single-
/// substep correction from producing a visible launch.
pub(crate) const MAX_LINEAR_VELOCITY: f32 = 15.0;

/// Per-substep angular velocity damping factor, same role as
/// `LINEAR_DAMPING` but larger — a real, regression-tested instability
/// was found (not just a theoretical concern) where a zero-compliance,
/// undamped Jacobi solver let a single ordinary contact correction (depth
/// ~0.01) inject an angular velocity of ~1.9 rad/s, which then overshot
/// into the opposite penetration on the very next substep, compounding
/// into a runaway spin over a few dozen substeps of flickering contact
/// (confirmed via a standalone trace before this constant existed).
/// Angular corrections need more damping than linear ones because the
/// same absolute positional depth produces a much larger angular velocity
/// once divided by a lever arm and a substep `dt` — the same mechanism
/// this module's own doc comment already flags for `MAX_ANGULAR_CORRECTION`.
pub(crate) const ANGULAR_DAMPING: f32 = 0.1;

/// Hard ceiling on any single substep's angular correction magnitude
/// (radians) — a safety valve against exactly the resonant blowup
/// `ANGULAR_DAMPING` is also meant to prevent, kept as a SEPARATE
/// mechanism because damping alone doesn't stop a single substep's
/// correction from already being enormous before damping ever gets a
/// chance to act on it (damping scales down velocity AFTER the correction
/// is computed; this clamp bounds the correction itself). 0.1 rad
/// (~5.7°) per substep is generous for legitimate fast rotation while
/// still ruling out the multi-radian single-substep spikes the
/// regression test below was built to catch.
pub(crate) const MAX_ANGULAR_CORRECTION: f32 = 0.1;

/// Hard ceiling on any single substep's LINEAR correction magnitude
/// (world units) — the linear analogue of `MAX_ANGULAR_CORRECTION`, added
/// specifically so the eventual GPU port's fixed-point `atomic<i32>`
/// scatter buffer (WGSL has no native `atomic<f32>`) has a documented,
/// provable overflow-safety bound: a correction's implied fixed-point
/// value is `round(correction * FIXED_POINT_SCALE)`, and without an upper
/// bound on `correction` itself, a single pathological deep-penetration
/// contact could produce a value large enough to overflow `i32`'s
/// ±2.1B range once scaled. Clamping HERE, on the CPU reference, before
/// the GPU port exists, is deliberate: this project's doctrine treats the
/// CPU reference as the algorithm's source of truth, so both backends
/// must clamp identically rather than the GPU silently diverging from an
/// unclamped CPU reference under extreme penetration. 2.0 world units is
/// generous for any legitimate single-substep correction (this engine's
/// existing test scenes never exceed a fraction of that) while still
/// ruling out the runaway-depth pathological case a bad contact (e.g. a
/// fast tunneling body) could otherwise produce. Bounding `correction`
/// alone does NOT bound the velocity `solve_substep_jacobi` derives from
/// it (`correction / dt` — see `MAX_LINEAR_VELOCITY`'s own doc comment for
/// the real, regression-tested case this gap produced).
pub(crate) const MAX_LINEAR_CORRECTION: f32 = 2.0;

/// Hard ceiling on any single contact's velocity-round impulse magnitude
/// (world units/second) — `resolve_contact_velocities`'s own analogue of
/// `MAX_LINEAR_CORRECTION`, added for the identical reason: without a
/// bound, a single pathological contact (e.g. a fast-moving body whose
/// normal_speed is large before this substep's contact catches it) could
/// scatter a value large enough to overflow the GPU port's fixed-point
/// `atomic<i32>` accumulator once scaled by `FIXED_POINT_SCALE`. Set to
/// the SAME value as `MAX_LINEAR_CORRECTION` deliberately — both
/// accumulators share the same `FIXED_POINT_SCALE` on the GPU side (one
/// scale, not two independently-tuned ones, keeps the overflow-safety
/// analysis in one place), and `FIXED_POINT_SCALE`'s own worst-case
/// bound (256 contacts, 4x margin) was computed against exactly this
/// magnitude — raising this constant without also revisiting
/// `FIXED_POINT_SCALE` would silently invalidate that margin. 2.0 m/s per
/// contact, before dividing by count, is generous for any legitimate
/// single-substep velocity correction at this engine's substep rate
/// (gravity alone contributes ~0.02 m/s per substep at 480Hz) while still
/// ruling out the pathological-speed case a bad contact could produce.
pub(crate) const MAX_VELOCITY_IMPULSE: f32 = 2.0;

/// One rigid body's mutable simulation state during a substep — position,
/// orientation, velocities, and inverse mass/inertia (needed here, not
/// just in `Inertia`, since the solver operates on this flat struct
/// without a live ECS query — see `solve_world`'s own body-state
/// construction).
#[derive(Clone, Copy, Debug)]
pub struct BodyState {
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub inverse_mass: f32,
    /// Body-LOCAL diagonal inverse inertia tensor (as stored in
    /// `Inertia`) — rotated into world space per-contact via
    /// `world_inverse_inertia`, not stored pre-rotated here, since the
    /// body's own rotation changes every substep and re-deriving world
    /// space from the current rotation is cheaper than keeping a stale
    /// world-space tensor in sync.
    pub inverse_inertia_local: Vec3,
}

impl BodyState {
    pub fn from_components(rigid_body: &RigidBody, inertia: &Inertia, position: Vec3, rotation: Quat) -> Self {
        Self {
            position,
            rotation,
            linear_velocity: rigid_body.linear_velocity,
            angular_velocity: rigid_body.angular_velocity,
            inverse_mass: inertia.inverse_mass,
            inverse_inertia_local: inertia.inverse_tensor_diag,
        }
    }

    /// The body-local diagonal inverse inertia tensor, rotated into world
    /// space: `R · diag(I⁻¹) · Rᵀ`, computed via the equivalent
    /// per-component scaling `R · (diag(I⁻¹) · (Rᵀ · v))` when applied to
    /// a vector `v`, avoiding ever materializing the full 3x3 matrix
    /// (unnecessary — this solver only ever needs `tensor · vector`
    /// products, never the tensor itself as a standalone value).
    fn apply_world_inverse_inertia(&self, v: Vec3) -> Vec3 {
        let local_v = self.rotation.inverse() * v;
        let scaled = local_v * self.inverse_inertia_local;
        self.rotation * scaled
    }
}

/// Per-body scattered correction accumulator — the CPU analogue of the
/// GPU's per-body `atomicAdd` targets (`sum`/`angular_sum`: `vec3<f32>`,
/// `count: u32`, divided in a second pass). A plain `Vec<Accumulator>`
/// indexed by body index here; the CPU reference sums serially since
/// there's no real concurrency to race over, but the TWO-PASS shape
/// (accumulate fully, THEN divide-and-apply) is preserved exactly, which
/// is the property that actually matters for matching the GPU algorithm.
#[derive(Clone, Copy, Debug, Default)]
struct Accumulator {
    sum: Vec3,
    angular_sum: Vec3,
    count: u32,
}

/// A body's full kinematic state from BEFORE the caller's own
/// gravity/velocity integration ran this substep — i.e. the true start of
/// the substep, prior to `solve_world`'s "predict" step. Passed alongside
/// the already-predicted `bodies` slice so `solve_substep_jacobi` can
/// derive each body's final velocity as `pre_integration_velocity +
/// correction / dt` (Müller et al. 2020's own velocity-update convention)
/// instead of reading it back out of a position delta.
///
/// An earlier version derived velocity from `(final_position -
/// substep_start_position) / dt` directly. That is algebraically identical
/// but numerically disastrous at realistic world-scale positions: a body
/// at `position.y ≈ 100.0` moving by gravity's own per-substep delta
/// (`~1e-4` at 480 Hz) loses almost all of that delta's precision when
/// subtracted from two `f32` values of magnitude 100 (only ~7 significant
/// decimal digits total) — confirmed via an isolated trace that
/// reproduced a real test failure (expected `-0.1635`, derived
/// `-0.1758`, a 7.5% error from pure cancellation, not a logic bug).
/// Tracking velocity as its own quantity through the substep — rather
/// than reconstructing it from a subtraction of two large numbers —
/// avoids ever performing that lossy subtraction.
#[derive(Clone, Copy, Debug)]
pub struct SubstepStart {
    pub position: Vec3,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
}

/// One XPBD substep over `bodies` and `contacts`: `bodies[i].position` on
/// entry is the PREDICTED position — the caller (`solve_world`) has
/// already integrated gravity into both velocity and position before
/// calling this function, exactly matching XPBD's "predict, then correct"
/// structure (Müller et al.). `substep_start[i]` is each body's own
/// full state from BEFORE that prediction ran (see `SubstepStart`'s own
/// doc comment for why this must be tracked directly rather than derived
/// from a position delta). Each contact independently computes a
/// positional correction (split between the two bodies by their relative
/// inverse mass) and scatters it into each body's accumulator; a second
/// pass divides by contact count, applies the averaged correction to
/// position/rotation, then derives velocity as `substep_start`'s
/// PRE-INTEGRATION velocity plus the correction's own implied velocity
/// contribution (`correction / dt`) — correctly absorbing hard impacts
/// (the correction fully accounts for the penetration this substep
/// produced, so adding it to the pre-integration velocity, not the
/// already-gravity-advanced current velocity, avoids double-counting the
/// inbound speed that caused the penetration in the first place) while
/// still preserving free-flight velocity when a body has no contacts this
/// substep (`acc.count == 0`, where the body's velocity is simply left as
/// whatever the caller's own gravity integration already produced).
pub fn solve_substep_jacobi(bodies: &mut [BodyState], contacts: &[Contact], dt: f32, substep_start: &[SubstepStart]) {
    if dt <= 0.0 {
        // Same guard, same reason as `solve_static::solve_body_static`:
        // Bevy's `Time::delta_secs()` is exactly 0.0 on the very first
        // frame, and deriving a velocity delta divided by a zero dt
        // produces NaN/Inf that poisons every later frame.
        return;
    }

    let mut accumulators = vec![Accumulator::default(); bodies.len()];

    for contact in contacts {
        let (ia, ib) = (contact.body_a as usize, contact.body_b as usize);
        let (inv_mass_a, inv_mass_b) = (bodies[ia].inverse_mass, bodies[ib].inverse_mass);

        // Lever arms from each body's own center of mass to the shared
        // world-space contact point — zero for a point sample sitting
        // exactly at a body's center (e.g. Sphere's degenerate case
        // before the multi-sample-point fix; harmless here since a zero
        // lever arm just means zero torque contribution, not a division
        // issue), nonzero for every real contact point.
        let ra = contact.point_world - bodies[ia].position;
        let rb = contact.point_world - bodies[ib].position;
        let n = contact.normal_world;

        // Generalized inverse mass per body (Müller et al. 2020, eq. 2-3):
        // linear inverse mass PLUS the angular contribution from pushing
        // at a lever arm rather than through the center of mass. A body
        // with infinite mass/inertia (inverse values of exactly 0.0)
        // contributes exactly 0.0 here regardless of lever arm, so static
        // bodies remain immovable with no special-casing.
        let angular_a = ra.cross(n);
        let angular_b = rb.cross(n);
        let w_a = inv_mass_a + angular_a.dot(bodies[ia].apply_world_inverse_inertia(angular_a));
        let w_b = inv_mass_b + angular_b.dot(bodies[ib].apply_world_inverse_inertia(angular_b));
        let total_w = w_a + w_b;
        if total_w <= 0.0 {
            // Both bodies infinite-mass (static-vs-static) — no correction
            // is meaningful; skip rather than divide by zero.
            continue;
        }

        // XPBD's scalar Lagrange multiplier for a single positional
        // constraint (Müller et al. 2020, eq. 4, zero-compliance case):
        // splits the depth-sized correction by each body's own share of
        // the combined generalized inverse mass, exactly generalizing the
        // pre-angular "split by inverse mass" rule to include rotation.
        let lambda = contact.depth / total_w;
        let impulse = n * lambda;

        if inv_mass_a > 0.0 {
            // Per-CONTACT clamp, before scattering into the accumulator —
            // a real gap the per-body-average clamp below doesn't close
            // on its own: the GPU port's atomicAdd scatters this raw
            // value BEFORE any division/averaging happens, so a single
            // pathological contact's own unclamped contribution could
            // already overflow the fixed-point accumulator even though
            // the CPU's final post-average result would have been
            // clamped. Clamping HERE too (in addition to the existing
            // per-body clamp after averaging) closes that gap on both
            // backends identically.
            accumulators[ia].sum += (impulse * inv_mass_a).clamp_length(0.0, MAX_LINEAR_CORRECTION);
            accumulators[ia].angular_sum += bodies[ia].apply_world_inverse_inertia(ra.cross(impulse)).clamp_length(0.0, MAX_ANGULAR_CORRECTION);
            accumulators[ia].count += 1;
        }
        if inv_mass_b > 0.0 {
            accumulators[ib].sum -= (impulse * inv_mass_b).clamp_length(0.0, MAX_LINEAR_CORRECTION);
            accumulators[ib].angular_sum -= bodies[ib].apply_world_inverse_inertia(rb.cross(impulse)).clamp_length(0.0, MAX_ANGULAR_CORRECTION);
            accumulators[ib].count += 1;
        }
    }

    for (i, body) in bodies.iter_mut().enumerate() {
        let acc = accumulators[i];

        // Correction application only happens with an active contact
        // (`position`/`rotation` already hold the caller's free-flight
        // prediction when `acc.count == 0`, and adding a zero correction
        // would be a no-op anyway) — but damping/clamping below must run
        // UNCONDITIONALLY, every substep, contact or not. A real,
        // regression-tested bug lived exactly in getting this wrong: an
        // earlier version put damping/clamping inside this same
        // early-`continue`, so a body that picked up a large angular
        // velocity right before losing its last contact kept that
        // velocity FOREVER — nothing ever damped or clamped it again,
        // since it never touched anything again to re-enter this branch,
        // producing an unrecoverable runaway spin that flung the body far
        // from the floor with no way back.
        if acc.count > 0 {
            let mut correction = acc.sum / acc.count as f32;
            let mut angular_correction = acc.angular_sum / acc.count as f32;

            // Hard ceiling BEFORE this correction ever reaches position or
            // velocity — see MAX_LINEAR_CORRECTION's own doc comment for
            // why this exists (GPU fixed-point atomic overflow safety),
            // mirroring the angular clamp immediately below.
            let linear_magnitude = correction.length();
            if linear_magnitude > MAX_LINEAR_CORRECTION {
                correction *= MAX_LINEAR_CORRECTION / linear_magnitude;
            }

            // Hard ceiling BEFORE this correction ever reaches
            // angular_velocity or rotation — see MAX_ANGULAR_CORRECTION's
            // own doc comment for why this is a separate mechanism from
            // damping, not a redundant one.
            let angular_magnitude = angular_correction.length();
            if angular_magnitude > MAX_ANGULAR_CORRECTION {
                angular_correction *= MAX_ANGULAR_CORRECTION / angular_magnitude;
            }

            body.position += correction;

            // XPBD's linearized quaternion update for a small per-substep
            // rotation (Müller et al. 2020, eq. 8-9): treat the angular
            // correction as an infinitesimal rotation vector, apply it as
            // a quaternion derivative `q_new = q + 0.5 * [correction, 0] *
            // q`, then renormalize — valid because substeps keep each
            // individual rotation small, the same "many small steps"
            // assumption the linear solve already leans on.
            let delta_q = Quat::from_xyzw(angular_correction.x, angular_correction.y, angular_correction.z, 0.0);
            let derivative = delta_q * body.rotation;
            let updated = Vec4::from(body.rotation) + 0.5 * Vec4::from(derivative);
            body.rotation = Quat::from_vec4(updated).normalize();

            // Velocity comes from the PRE-INTEGRATION state
            // (`substep_start`) plus this correction's own implied
            // velocity contribution — see `SubstepStart`'s own doc
            // comment for why this must be tracked directly rather than
            // derived from a position delta (catastrophic cancellation at
            // realistic world-scale positions), and why it's added to the
            // pre-integration velocity rather than the current
            // (already-gravity-advanced) velocity (avoiding double-
            // counting the inbound speed that caused the penetration).
            let start = substep_start[i];
            body.linear_velocity = start.linear_velocity + correction / dt;
            body.angular_velocity = start.angular_velocity + angular_correction / dt;
        }
        // else: no contact touched this body this substep, so
        // `position`/`rotation` already hold exactly the caller's
        // free-flight prediction and its velocity is exactly whatever the
        // caller's own gravity integration already produced -- leave it
        // as-is rather than recomputing it.

        // Damping and the hard velocity ceiling apply every substep,
        // contact or not — see this loop's own comment above for why
        // that unconditional application is the actual fix, not just a
        // stylistic preference.
        body.linear_velocity *= (1.0 - LINEAR_DAMPING * dt).max(0.0);
        body.angular_velocity *= (1.0 - ANGULAR_DAMPING * dt).max(0.0);
        let linear_velocity_magnitude = body.linear_velocity.length();
        if linear_velocity_magnitude > MAX_LINEAR_VELOCITY {
            body.linear_velocity *= MAX_LINEAR_VELOCITY / linear_velocity_magnitude;
        }
        let angular_velocity_magnitude = body.angular_velocity.length();
        if angular_velocity_magnitude > MAX_ANGULAR_VELOCITY {
            body.angular_velocity *= MAX_ANGULAR_VELOCITY / angular_velocity_magnitude;
        }
    }

    resolve_contact_velocities(bodies, contacts);
}

/// Velocity-level contact resolution, restitution 0 — a SEPARATE pass from
/// the positional correction above, run after it. A real, regression-
/// tested bug motivated this: the positional pass alone converts a large
/// one-shot position correction (e.g. from a body that free-fell for many
/// substeps before finally touching down) into an equally large IMPLIED
/// velocity via `correction / dt` — but that implied velocity only
/// describes how far the position moved, it never explicitly consumes the
/// body's own pre-existing inbound velocity. Without this pass, a body
/// arriving at a contact with velocity -6.9 m/s came out the other side at
/// +10.0 m/s in a SINGLE substep — a physically wrong bounce for a
/// restitution-0 contact, confirmed via a standalone trace. This pass
/// mirrors `physics::solve_static::solve_body_static`'s own single-contact
/// convention ("remove the inward-normal component of velocity") but
/// generalizes it to the POINT velocity (linear + angular contribution,
/// `v + ω × r`) rather than just the center-of-mass linear velocity, since
/// a rotating body's contact point can be moving quite differently from
/// its center.
fn resolve_contact_velocities(bodies: &mut [BodyState], contacts: &[Contact]) {
    let mut linear_accumulators = vec![Accumulator::default(); bodies.len()];
    let mut angular_accumulators = vec![Accumulator::default(); bodies.len()];

    for contact in contacts {
        let (ia, ib) = (contact.body_a as usize, contact.body_b as usize);
        let n = contact.normal_world;
        let ra = contact.point_world - bodies[ia].position;
        let rb = contact.point_world - bodies[ib].position;

        let point_velocity_a = bodies[ia].linear_velocity + bodies[ia].angular_velocity.cross(ra);
        let point_velocity_b = bodies[ib].linear_velocity + bodies[ib].angular_velocity.cross(rb);
        let relative_velocity = point_velocity_a - point_velocity_b;
        let normal_speed = relative_velocity.dot(n);
        if normal_speed >= 0.0 {
            // Already separating (or exactly at rest along the normal) —
            // nothing to resolve; a restitution-0 contact only ever
            // removes INWARD (approaching) relative velocity, never adds
            // outward velocity (that would be a bounce, restitution > 0).
            continue;
        }

        let (inv_mass_a, inv_mass_b) = (bodies[ia].inverse_mass, bodies[ib].inverse_mass);
        let angular_a = ra.cross(n);
        let angular_b = rb.cross(n);
        let w_a = inv_mass_a + angular_a.dot(bodies[ia].apply_world_inverse_inertia(angular_a));
        let w_b = inv_mass_b + angular_b.dot(bodies[ib].apply_world_inverse_inertia(angular_b));
        let total_w = w_a + w_b;
        if total_w <= 0.0 {
            continue;
        }

        // Impulse magnitude that brings the relative normal velocity
        // exactly to zero (restitution 0) — the standard velocity-level
        // impulse formula, generalizing `solve_static`'s own "zero the
        // inward component" rule to two bodies sharing the correction by
        // their relative generalized inverse mass.
        let impulse = n * (-normal_speed / total_w);

        if inv_mass_a > 0.0 {
            // Per-CONTACT clamp, before scattering — same overflow-safety
            // gap and fix as solve_substep_jacobi's own contact loop
            // above (see that clamp's own inline comment for why the
            // per-body-average clamp below isn't enough on its own).
            linear_accumulators[ia].sum += (impulse * inv_mass_a).clamp_length(0.0, MAX_VELOCITY_IMPULSE);
            linear_accumulators[ia].count += 1;
            angular_accumulators[ia].sum += bodies[ia].apply_world_inverse_inertia(ra.cross(impulse)).clamp_length(0.0, MAX_VELOCITY_IMPULSE);
            angular_accumulators[ia].count += 1;
        }
        if inv_mass_b > 0.0 {
            linear_accumulators[ib].sum -= (impulse * inv_mass_b).clamp_length(0.0, MAX_VELOCITY_IMPULSE);
            linear_accumulators[ib].count += 1;
            angular_accumulators[ib].sum -= bodies[ib].apply_world_inverse_inertia(rb.cross(impulse)).clamp_length(0.0, MAX_VELOCITY_IMPULSE);
            angular_accumulators[ib].count += 1;
        }
    }

    for (i, body) in bodies.iter_mut().enumerate() {
        if linear_accumulators[i].count > 0 {
            let mut correction = linear_accumulators[i].sum / linear_accumulators[i].count as f32;
            // Hard ceiling BEFORE this correction reaches velocity — see
            // MAX_VELOCITY_IMPULSE's own doc comment for why this exists
            // (GPU fixed-point atomic overflow safety), mirroring
            // solve_substep_jacobi's own MAX_LINEAR_CORRECTION clamp.
            let magnitude = correction.length();
            if magnitude > MAX_VELOCITY_IMPULSE {
                correction *= MAX_VELOCITY_IMPULSE / magnitude;
            }
            body.linear_velocity += correction;
        }
        if angular_accumulators[i].count > 0 {
            let mut correction = angular_accumulators[i].sum / angular_accumulators[i].count as f32;
            let magnitude = correction.length();
            if magnitude > MAX_VELOCITY_IMPULSE {
                correction *= MAX_VELOCITY_IMPULSE / magnitude;
            }
            body.angular_velocity += correction;
        }
    }
}

#[cfg(test)]
mod tests {
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

    /// Like `body`, but with non-zero inverse inertia — for tests that
    /// specifically need angular response (most existing tests use
    /// `inverse_inertia_local: ZERO`, which correctly means "cannot
    /// rotate," matching their own pre-angular-response expectations
    /// about pure linear motion).
    fn rotatable_body(position: Vec3, inverse_mass: f32, inverse_inertia: Vec3) -> BodyState {
        BodyState {
            position,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inverse_mass,
            inverse_inertia_local: inverse_inertia,
        }
    }

    /// Snapshot every body's full pre-integration state, exactly what a
    /// real caller (`solve_world`) captures right before its own gravity/
    /// rotation prediction runs each substep -- see `SubstepStart`'s own
    /// doc comment for why this must be the body's full kinematic state,
    /// not just position/rotation.
    fn snapshot(bodies: &[BodyState]) -> Vec<SubstepStart> {
        bodies
            .iter()
            .map(|b| SubstepStart {
                position: b.position,
                rotation: b.rotation,
                linear_velocity: b.linear_velocity,
                angular_velocity: b.angular_velocity,
            })
            .collect()
    }

    #[test]
    fn a_zero_delta_time_step_never_produces_nan_or_infinite_velocity() {
        let mut bodies = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.0, 0.0, 0.0), 1.0)];
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::ZERO, normal_world: Vec3::X, depth: 0.5 }];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 0.0, &substep_start);
        for b in &bodies {
            assert!(b.linear_velocity.is_finite(), "expected finite velocity, got {:?}", b.linear_velocity);
        }
    }

    #[test]
    fn a_pathologically_deep_penetration_has_its_correction_clamped_to_max_linear_correction() {
        // A contact reporting an enormous depth (e.g. a fast body that
        // tunneled deep into another before this substep's contact
        // generation caught it) must not produce an equally enormous
        // positional correction -- see MAX_LINEAR_CORRECTION's own doc
        // comment for why this matters beyond ordinary stability: the
        // eventual GPU port's fixed-point atomic scatter buffer needs a
        // provable upper bound on any single correction to guarantee it
        // can't overflow i32 once scaled, and this clamp is that bound on
        // the CPU reference both backends must match.
        let mut bodies = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.0, 0.0, 0.0), 0.0)];
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 1000.0 }];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        let displacement = (bodies[0].position - Vec3::ZERO).length();
        assert!(
            (displacement - MAX_LINEAR_CORRECTION).abs() < 1e-4,
            "expected the correction to be clamped to MAX_LINEAR_CORRECTION ({MAX_LINEAR_CORRECTION}), got a displacement of {displacement}"
        );
    }

    #[test]
    fn a_correction_divided_by_a_tiny_substep_dt_has_its_derived_velocity_clamped_to_max_linear_velocity() {
        // The real, regression-tested gap MAX_LINEAR_VELOCITY closes:
        // MAX_LINEAR_CORRECTION bounds `correction` itself, but
        // `body.linear_velocity = start.linear_velocity + correction / dt`
        // derives velocity by DIVIDING that bounded correction by `dt` --
        // at a small enough substep dt, even a moderate, legitimately-
        // sized correction (nowhere near MAX_LINEAR_CORRECTION's own
        // clamp) produces an enormous derived velocity. Reproduces the
        // real scenario this was found in: a moderate ~0.1m penetration
        // (not a pathological 1000-unit one) resolved at a small substep
        // dt, exactly like a resting body on a kinematic platform that
        // penetrated while GPU physics dispatch itself hadn't started yet
        // (see physics_stability.rs's own kinematic-platform milestone).
        let mut bodies = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.0, 0.0, 0.0), 0.0)];
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 0.1 }];
        let substep_start = snapshot(&bodies);
        let tiny_dt = 0.0002; // matches a real 8-substep frame at a very high framerate
        solve_substep_jacobi(&mut bodies, &contacts, tiny_dt, &substep_start);
        let velocity_magnitude = bodies[0].linear_velocity.length();
        assert!(
            velocity_magnitude <= MAX_LINEAR_VELOCITY + 1e-3,
            "expected the derived velocity to be clamped to MAX_LINEAR_VELOCITY ({MAX_LINEAR_VELOCITY}), got {velocity_magnitude} (a correction of only 0.1, far under MAX_LINEAR_CORRECTION, divided by a tiny dt of {tiny_dt} would otherwise produce {})",
            0.1 / tiny_dt
        );
    }

    #[test]
    fn a_pathologically_fast_approach_has_its_velocity_impulse_clamped_to_max_velocity_impulse() {
        // A body approaching a static wall at an enormous speed (e.g. one
        // that skipped several substeps' worth of contact resolution due
        // to a transient bug elsewhere, or a deliberately extreme test of
        // the solver's own safety margins) must not receive an equally
        // enormous velocity-round impulse -- see MAX_VELOCITY_IMPULSE's
        // own doc comment for why this matters: the same GPU fixed-point
        // overflow-safety reasoning MAX_LINEAR_CORRECTION already
        // established for the position round applies here too.
        let mut bodies = vec![
            BodyState { position: Vec3::ZERO, rotation: Quat::IDENTITY, linear_velocity: Vec3::new(1_000.0, 0.0, 0.0), angular_velocity: Vec3::ZERO, inverse_mass: 1.0, inverse_inertia_local: Vec3::ZERO },
            body(Vec3::new(1.0, 0.0, 0.0), 0.0),
        ];
        // A contact right at the boundary, normal pointing back toward A
        // (i.e. A is moving INTO B along +X, so the normal opposing that
        // approach points -X) -- depth doesn't matter for this test, only
        // the velocity-round response to the huge approach speed.
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 0.01 }];
        resolve_contact_velocities(&mut bodies, &contacts);
        let velocity_change = (bodies[0].linear_velocity - Vec3::new(1_000.0, 0.0, 0.0)).length();
        assert!(
            (velocity_change - MAX_VELOCITY_IMPULSE).abs() < 1e-3,
            "expected the velocity impulse to be clamped to MAX_VELOCITY_IMPULSE ({MAX_VELOCITY_IMPULSE}), got a velocity change of {velocity_change}"
        );
    }

    #[test]
    fn a_single_contact_between_two_equal_mass_bodies_splits_the_correction_evenly() {
        let mut bodies = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.0, 0.0, 0.0), 1.0)];
        // Normal points from B toward A (i.e. -X, since B is at +X of A)
        // -- push A further in -X, B further in +X.
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        // Equal inverse mass -> equal 50/50 split of the 0.4 correction.
        assert!((bodies[0].position.x - (0.0 - 0.2)).abs() < 1e-4, "body A should move -0.2 in X, got {:?}", bodies[0].position);
        assert!((bodies[1].position.x - (1.0 + 0.2)).abs() < 1e-4, "body B should move +0.2 in X, got {:?}", bodies[1].position);
    }

    #[test]
    fn a_static_body_never_moves_even_when_it_absorbs_a_contact() {
        let mut bodies = vec![body(Vec3::ZERO, 0.0), body(Vec3::new(1.0, 0.0, 0.0), 1.0)];
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
        let original_a = bodies[0].position;
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        assert_eq!(bodies[0].position, original_a, "an infinite-mass body must never move");
        // All correction goes to B since A absorbs none of it.
        assert!((bodies[1].position.x - 1.4).abs() < 1e-4, "expected B to absorb the full correction, got {:?}", bodies[1].position);
    }

    #[test]
    fn two_static_bodies_in_contact_produce_no_motion_and_no_division_by_zero() {
        let mut bodies = vec![body(Vec3::ZERO, 0.0), body(Vec3::new(1.0, 0.0, 0.0), 0.0)];
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.0, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        assert_eq!(bodies[0].position, Vec3::ZERO);
        assert_eq!(bodies[1].position, Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn multiple_contacts_on_one_body_are_averaged_not_summed() {
        // Body 0 has TWO contacts pushing it in the same direction; the
        // Jacobi average (not a naive sum) must not double-apply the
        // correction -- this is the exact property that distinguishes
        // "Jacobi with per-body averaging" from "just sum every
        // correction," and it's what makes this approach safe to
        // parallelize without a graph-coloring pass.
        let mut bodies = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.0, 0.0, 0.0), 0.0), body(Vec3::new(-1.0, 0.0, 0.0), 0.0)];
        let contacts = vec![
            Contact { body_a: 0, body_b: 1, point_world: Vec3::ZERO, normal_world: Vec3::NEG_X, depth: 0.2 },
            Contact { body_a: 0, body_b: 2, point_world: Vec3::ZERO, normal_world: Vec3::NEG_X, depth: 0.6 },
        ];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        // Average of the two corrections (both full-weight since the
        // other bodies are static): (-0.2 + -0.6) / 2 = -0.4.
        assert!((bodies[0].position.x - (-0.4)).abs() < 1e-4, "expected the averaged correction -0.4, got {:?}", bodies[0].position);
    }

    #[test]
    fn a_body_with_no_contacts_stays_exactly_where_it_was() {
        let mut bodies = vec![body(Vec3::new(3.0, 4.0, 5.0), 1.0)];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &[], 1.0 / 60.0, &substep_start);
        assert_eq!(bodies[0].position, Vec3::new(3.0, 4.0, 5.0));
        assert_eq!(bodies[0].linear_velocity, Vec3::ZERO);
    }

    #[test]
    fn a_body_with_no_contacts_keeps_its_existing_velocity_unchanged() {
        // Regression test for a real bug caught during this stage's own
        // development: an earlier version derived velocity as
        // `(corrected - position) / dt` unconditionally, which silently
        // RESET velocity to zero whenever a body had no contacts this
        // substep (corrected == predicted == position, so the delta is
        // exactly zero) -- discarding whatever free-flight velocity the
        // CALLER had already integrated (e.g. gravity) before calling
        // this function. The prior test above didn't catch this because
        // it started from zero velocity, so "reset to zero" and
        // "preserved" looked identical. A body already falling under
        // gravity with no contacts must keep accelerating frame over
        // frame, not have its velocity zeroed every single substep.
        let mut bodies = vec![BodyState {
            position: Vec3::new(0.0, 99.9, 0.0),
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::new(0.0, -5.0, 0.0),
            angular_velocity: Vec3::ZERO,
            inverse_mass: 1.0,
            inverse_inertia_local: Vec3::ZERO,
        }];
        // Mirror solve_world's own contract: `substep_start` is the
        // position BEFORE this substep's own velocity integration, not
        // the already-integrated position passed into this function --
        // capture the snapshot first, THEN advance position by the
        // existing velocity, exactly like the real caller does.
        let dt = 1.0 / 60.0;
        let substep_start = snapshot(&bodies);
        let v = bodies[0].linear_velocity;
        bodies[0].position += v * dt;
        solve_substep_jacobi(&mut bodies, &[], dt, &substep_start);
        // "Preserved" now means "unchanged but for the small, deliberate
        // per-substep damping every body receives regardless of contact
        // state" (see LINEAR_DAMPING's own doc comment) -- not bit-exact
        // equality, which would have been the old (buggy) zero-contact
        // behavior's actual symptom in disguise if reintroduced.
        assert!((bodies[0].linear_velocity - Vec3::new(0.0, -5.0, 0.0)).length() < 0.01, "expected velocity ~preserved (small damping aside) when no contacts touch this body, got {:?}", bodies[0].linear_velocity);
        assert_eq!(bodies[0].position, Vec3::new(0.0, 99.9 - 5.0 * dt, 0.0), "position should have advanced by the caller's own pre-integration, unchanged by this function since there's no correction to apply");
    }

    #[test]
    fn gravity_compounds_across_repeated_substeps_with_no_contacts() {
        // End-to-end free-fall check mirroring solve_world's own
        // integrate-then-solve order: gravity integration happens OUTSIDE
        // this function (the caller's job), so this test drives that
        // exact sequence itself and confirms velocity actually compounds
        // substep over substep rather than being reset each time -- the
        // literal symptom this bug produced (a sphere falling ~substep-
        // count times slower than real gravity).
        let mut bodies = vec![body(Vec3::new(0.0, 100.0, 0.0), 1.0)];
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 60.0;
        let substeps = 8;
        let substep_dt = dt / substeps as f32;

        for _ in 0..substeps {
            let substep_start = snapshot(&bodies);
            bodies[0].linear_velocity += gravity * substep_dt;
            let v = bodies[0].linear_velocity;
            bodies[0].position += v * substep_dt;
            solve_substep_jacobi(&mut bodies, &[], substep_dt, &substep_start);
        }

        // After one full frame (1/60s) of free fall, velocity should be
        // very close to g * dt, not a small fraction of it.
        let expected_velocity_y = -9.81 * dt;
        assert!(
            (bodies[0].linear_velocity.y - expected_velocity_y).abs() < 1e-3,
            "expected velocity.y ~{expected_velocity_y} after one frame of free fall, got {}",
            bodies[0].linear_velocity.y
        );
    }

    #[test]
    fn repeated_substeps_on_two_overlapping_bodies_converge_toward_zero_penetration() {
        // A more realistic multi-substep scenario: regenerate contacts
        // each substep (as the real integration loop will) and confirm
        // the correction converges rather than oscillating or diverging.
        use super::super::contacts::{BodySnapshot, generate_contacts};
        use super::super::components::PhysicsShape;

        let mut bodies = vec![body(Vec3::ZERO, 1.0), body(Vec3::new(1.2, 0.0, 0.0), 1.0)];
        let shape = PhysicsShape::Sphere { radius: 1.0 };
        for _ in 0..200 {
            let snap_a = BodySnapshot { shape, translation: bodies[0].position, rotation: Quat::IDENTITY };
            let snap_b = BodySnapshot { shape, translation: bodies[1].position, rotation: Quat::IDENTITY };
            let contacts = generate_contacts(0, &snap_a, 1, &snap_b);
            let substep_start = snapshot(&bodies);
            solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        }
        let separation = (bodies[1].position - bodies[0].position).length();
        // With multiple surface samples per sphere (not a single center
        // point -- see sample_points.rs's own doc comment for why),
        // Jacobi-averaging several off-axis contacts per substep converges
        // more slowly than the old single-contact case did, but must
        // still converge toward the two spheres just touching (separation
        // 2.0), not stall at a smaller separation or diverge.
        assert!((separation - 2.0).abs() < 5e-2, "expected the two radius-1.0 spheres to converge to approximately touching (separation 2.0), got {separation}");
    }

    #[test]
    fn an_off_center_contact_induces_rotation_when_inertia_is_finite() {
        // The core new behavior this stage adds: a contact whose point is
        // offset from the body's center of mass must produce torque, not
        // just a linear push -- confirmed by checking angular_velocity
        // becomes nonzero (the body starts rotating) when the body CAN
        // rotate (nonzero inverse inertia).
        let mut bodies = vec![
            rotatable_body(Vec3::ZERO, 1.0, Vec3::splat(1.0)),
            rotatable_body(Vec3::new(1.0, 0.0, 0.0), 0.0, Vec3::ZERO),
        ];
        // Contact point offset in +Y from body A's center -- a push along
        // -X applied above the center of mass must induce rotation.
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.5, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        assert!(bodies[0].angular_velocity.length() > 1e-4, "expected an off-center contact to induce rotation, got angular_velocity {:?}", bodies[0].angular_velocity);
    }

    #[test]
    fn a_body_with_zero_inverse_inertia_never_rotates_even_from_an_off_center_contact() {
        // The pre-fix behavior, still correct for a body that's
        // deliberately non-rotating (inverse_inertia_local == ZERO is a
        // legitimate modeling choice, e.g. a kinematic-orientation body,
        // not just "hasn't been fixed yet") -- confirms the generalized
        // inverse mass correctly excludes the angular term when inertia
        // is infinite, matching Inertia::STATIC's own "zero means
        // immovable in that regard" convention.
        let mut bodies = vec![
            body(Vec3::ZERO, 1.0), // body() uses inverse_inertia_local: ZERO
            rotatable_body(Vec3::new(1.0, 0.0, 0.0), 0.0, Vec3::ZERO),
        ];
        let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.5, 0.0), normal_world: Vec3::NEG_X, depth: 0.4 }];
        let substep_start = snapshot(&bodies);
        solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
        assert_eq!(bodies[0].angular_velocity, Vec3::ZERO, "a body with zero inverse inertia must never start rotating");
        // It must still receive the full linear correction -- this isn't
        // "the body is inert," only "it cannot rotate."
        assert!(bodies[0].position.x < 0.0, "expected the body to still move linearly, got {:?}", bodies[0].position);
    }

    #[test]
    fn rotation_stays_normalized_after_many_angular_corrections() {
        // The linearized quaternion update (`q + 0.5*delta*q`, then
        // renormalize) accumulates floating-point drift over many
        // substeps -- confirms `rotation.length()` stays a valid unit
        // quaternion (within tolerance) after a long run, not just after
        // one substep.
        let mut bodies = vec![
            rotatable_body(Vec3::ZERO, 1.0, Vec3::splat(1.0)),
            rotatable_body(Vec3::new(1.0, 0.0, 0.0), 0.0, Vec3::ZERO),
        ];
        for _ in 0..500 {
            let contacts = vec![Contact { body_a: 0, body_b: 1, point_world: Vec3::new(0.5, 0.3, 0.0), normal_world: Vec3::NEG_X, depth: 0.1 }];
            let substep_start = snapshot(&bodies);
            solve_substep_jacobi(&mut bodies, &contacts, 1.0 / 60.0, &substep_start);
            // Re-separate the bodies each iteration so the loop keeps
            // generating fresh angular corrections instead of converging
            // to rest after the first few substeps.
            bodies[0].position = Vec3::ZERO;
        }
        assert!((bodies[0].rotation.length() - 1.0).abs() < 1e-3, "expected rotation to remain a unit quaternion, length was {}", bodies[0].rotation.length());
    }

    #[test]
    fn a_tilted_box_resting_on_a_flat_floor_rotates_toward_lying_flat() {
        // End-to-end angular-settling check: a box dropped with a
        // deliberate initial tilt, resting on a large flat static floor,
        // should progressively rotate toward an axis-aligned resting
        // orientation as contact torque corrects it -- the literal
        // behavior gap a user-reported issue flagged (bodies frozen at
        // their spawn rotation forever, regardless of contacts).
        use super::super::components::PhysicsShape;
        use super::super::contacts::{BodySnapshot, generate_contacts};
        use super::super::inertia::box_inertia;

        let half_extents = Vec3::splat(0.5);
        let mass = 1.0;
        let inverse_inertia = 1.0 / box_inertia(mass, half_extents);

        let mut bodies = vec![
            rotatable_body(Vec3::new(0.0, 0.5, 0.0), 1.0, inverse_inertia),
            rotatable_body(Vec3::new(0.0, -0.5, 0.0), 0.0, Vec3::ZERO),
        ];
        // A deliberate small initial tilt about Z -- one bottom corner
        // starts lower than the other, so contact torque has real work
        // to do to bring it flat.
        bodies[0].rotation = Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, 0.3);

        let box_shape = PhysicsShape::RoundedBox { half_extents, corner_radius: 0.0 };
        let floor_shape = PhysicsShape::RoundedBox { half_extents: Vec3::new(10.0, 0.5, 10.0), corner_radius: 0.0 };

        let initial_tilt = bodies[0].rotation.to_euler(EulerRot::XYZ).2.abs();
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 60.0;
        for _ in 0..600 {
            // Mirror solve_world's own integrate-then-solve order: gravity
            // keeps pressing the box down each substep, sustaining real
            // contact pressure so torque has ongoing work to do -- without
            // this, the box only briefly touches the floor once and then
            // has nothing left correcting it, which isn't how the real
            // per-frame loop behaves.
            let substep_start = snapshot(&bodies);
            bodies[0].linear_velocity += gravity * dt;
            let v = bodies[0].linear_velocity;
            bodies[0].position += v * dt;

            let snap_a = BodySnapshot { shape: box_shape, translation: bodies[0].position, rotation: bodies[0].rotation };
            let snap_b = BodySnapshot { shape: floor_shape, translation: bodies[1].position, rotation: bodies[1].rotation };
            let contacts = generate_contacts(0, &snap_a, 1, &snap_b);
            solve_substep_jacobi(&mut bodies, &contacts, dt, &substep_start);
        }
        let final_tilt = bodies[0].rotation.to_euler(EulerRot::XYZ).2.abs();
        assert!(final_tilt < initial_tilt * 0.5, "expected the box to rotate toward flat (tilt decreasing from {initial_tilt} toward 0), ended at {final_tilt}");
    }
}
