//! Inertialization — transitions that preserve velocity, replacing the
//! crossfade.
//!
//! # The problem with crossfading
//!
//! The conventional way to switch animations is to blend the old and new
//! poses over a fixed window. That blend averages two *positions*, which
//! means it also averages two *velocities* — so a limb travelling fast in
//! one direction and fast in another momentarily travels slowly in some
//! third direction. The result is the characteristic floaty, weightless
//! interruption: the character visibly loses momentum at the exact moment
//! a responsive game most wants to keep it.
//!
//! This project's existing `PoseTransition` is a smoothstep blend of the
//! same family. It is position-continuous but **not velocity-continuous**:
//! interrupt a transition mid-flight and the bone's speed jumps.
//!
//! # What inertialization does instead
//!
//! Switch the source data to the new animation *immediately* — no blend
//! window, no two-pose evaluation — and separately remember the **offset**
//! between where the character was and where the new animation says it
//! should be, along with the velocity it had at that instant. Then decay
//! that offset to zero.
//!
//! Because the offset starts out carrying the old motion's velocity and
//! decays smoothly to nothing, the visible result leaves the old pose at
//! exactly the speed it was already moving and arrives at the new one
//! without ever averaging the two. Momentum is preserved through the
//! interruption. It is also cheaper: only one animation is ever sampled.
//!
//! # Two decay shapes
//!
//! - [`decay_exponential`] — the spring-derived form
//!   `f(t) = e^{-yt}(x + (v + xy)t)`. Never exactly reaches zero, which is
//!   fine for an offset that is already imperceptible after a few
//!   half-lives. Cheapest, and matches the [`spring`](super::spring) module
//!   so a rig tuned in one vocabulary behaves predictably in the other.
//! - [`InertializeCubic`] — Daniel Holden's cubic form, which reaches
//!   *exactly* zero at a known time. Use it when a hard deadline matters:
//!   the foot-locking stage needs the lock offset provably gone before the
//!   next contact, not merely small.
//!
//! Both are implemented for scalars, [`Vec3`], and [`Quat`]. The quaternion
//! variants operate on the **scaled-angle-axis** of the rotation
//! difference, per [`super::quat_ext`] — a flat tangent space where an
//! offset adds and decays like any vector — and every one of them
//! neighbourhoods before differencing.

use bevy::math::{Quat, Vec3};

use super::quat_ext::{from_scaled_angle_axis, neighborhood, to_scaled_angle_axis};

/// `ln(2)`, relating a half-life to an exponential decay rate.
const LN_2: f32 = std::f32::consts::LN_2;

/// The decay rate `y` for which `exp(-y * halflife) == 0.5`.
///
/// Guarded against a zero or negative half-life, which would otherwise
/// produce an infinite rate and NaN the offset.
#[inline]
pub fn halflife_to_decay_rate(halflife: f32) -> f32 {
    LN_2 / halflife.max(1.0e-5)
}

/// Evaluates `f(t) = e^{-yt} * (x + (v + x*y) * t)` — the offset remaining
/// at time `t`, and its rate of change.
///
/// This is the critically damped spring's own solution with a target of
/// zero, which is exactly what an offset needs: it starts at `x` moving at
/// `v`, and decays to nothing without overshooting through zero.
///
/// Returns `(offset, offset_velocity)`.
#[inline]
pub fn decay_exponential(x: f32, v: f32, halflife: f32, t: f32) -> (f32, f32) {
    let y = halflife_to_decay_rate(halflife);
    let j1 = v + x * y;
    let eyt = (-y * t).exp();

    (eyt * (x + j1 * t), eyt * (v - j1 * y * t))
}

/// [`decay_exponential`] for a [`Vec3`], componentwise.
#[inline]
pub fn decay_exponential_vec3(x: Vec3, v: Vec3, halflife: f32, t: f32) -> (Vec3, Vec3) {
    let y = halflife_to_decay_rate(halflife);
    let j1 = v + x * y;
    let eyt = (-y * t).exp();

    (eyt * (x + j1 * t), eyt * (v - j1 * y * t))
}

/// A decaying offset between where something *was* and where its source
/// animation now says it should be.
///
/// The invariant that makes this useful: `offset` always starts out
/// carrying the velocity the old motion had, so applying it keeps momentum
/// continuous across an interruption. See the module doc.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Inertializer {
    /// Remaining offset, in whatever space the caller is working in.
    pub offset: Vec3,
    /// Rate of change of the offset.
    pub velocity: Vec3,
}

impl Inertializer {
    /// No offset — the output equals the source animation exactly.
    pub const ZERO: Self = Self { offset: Vec3::ZERO, velocity: Vec3::ZERO };

    /// Begins a transition.
    ///
    /// `current`/`current_velocity` describe where the character visibly is
    /// *right now* (source animation plus any offset still decaying);
    /// `target`/`target_velocity` describe what the new source animation
    /// says at this same instant.
    ///
    /// Call this at the moment of the switch, then [`Self::advance`] every
    /// frame after.
    ///
    /// Note this composes correctly with an interruption: because `current`
    /// already includes the previous, still-decaying offset, transitioning
    /// again mid-decay neither double-counts nor discards it.
    #[inline]
    pub fn begin(
        current: Vec3,
        current_velocity: Vec3,
        target: Vec3,
        target_velocity: Vec3,
    ) -> Self {
        Self { offset: current - target, velocity: current_velocity - target_velocity }
    }

    /// Decays the offset by `dt`.
    #[inline]
    pub fn advance(&mut self, halflife: f32, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let (offset, velocity) =
            decay_exponential_vec3(self.offset, self.velocity, halflife, dt);
        self.offset = offset;
        self.velocity = velocity;
    }

    /// Adds the remaining offset onto a source-animation value.
    #[inline]
    pub fn apply(&self, source: Vec3) -> Vec3 {
        source + self.offset
    }

    /// Whether the offset has decayed below a perceptible threshold.
    ///
    /// The exponential form never reaches exactly zero, so callers that want
    /// to stop doing work (or to assert a transition finished) need a
    /// tolerance rather than an equality test.
    #[inline]
    pub fn is_settled(&self, tolerance: f32) -> bool {
        self.offset.length_squared() <= tolerance * tolerance
    }
}

/// An [`Inertializer`] for rotations.
///
/// Stores its offset in **scaled-angle-axis** form (axis scaled by the full
/// angle in radians), the flat tangent space where an angular offset decays
/// like a vector. Every difference taken here is neighbourhooded first, so
/// a transition never sends a joint the long way round.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RotationInertializer {
    /// Remaining rotational offset, scaled-angle-axis.
    pub offset: Vec3,
    /// Angular velocity of the offset, rad/s.
    pub velocity: Vec3,
}

impl RotationInertializer {
    /// No offset — the output equals the source animation exactly.
    pub const ZERO: Self = Self { offset: Vec3::ZERO, velocity: Vec3::ZERO };

    /// Begins a rotational transition. See [`Inertializer::begin`].
    ///
    /// The offset is measured as the rotation taking `target` to `current`,
    /// so that composing it back onto the target reproduces the current
    /// orientation exactly at `t = 0`.
    #[inline]
    pub fn begin(
        current: Quat,
        current_velocity: Vec3,
        target: Quat,
        target_velocity: Vec3,
    ) -> Self {
        let current = neighborhood(target, current);

        Self {
            offset: to_scaled_angle_axis(target.inverse() * current),
            velocity: current_velocity - target_velocity,
        }
    }

    /// Decays the offset by `dt`.
    #[inline]
    pub fn advance(&mut self, halflife: f32, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let (offset, velocity) =
            decay_exponential_vec3(self.offset, self.velocity, halflife, dt);
        self.offset = offset;
        self.velocity = velocity;
    }

    /// Composes the remaining offset onto a source-animation rotation.
    ///
    /// Post-multiplied, matching how [`Self::begin`] measured it, so the
    /// offset is expressed in the target's own local frame.
    #[inline]
    pub fn apply(&self, source: Quat) -> Quat {
        source * from_scaled_angle_axis(self.offset)
    }

    /// Whether the offset has decayed below `tolerance` radians.
    #[inline]
    pub fn is_settled(&self, tolerance_radians: f32) -> bool {
        self.offset.length_squared() <= tolerance_radians * tolerance_radians
    }
}

/// Holden's cubic inertialization: an offset that reaches **exactly** zero
/// at a known time, rather than merely approaching it.
///
/// Use this where a deadline is load-bearing. The foot-locking stage is the
/// motivating case: a lock offset that is still 1 mm off when the next
/// contact begins reintroduces exactly the sliding the lock exists to
/// remove, so "small" is not good enough — it has to be *gone*.
///
/// The blend is the standard cubic Hermite basis over `t / blend_time`,
/// which is why both position and velocity land on zero together.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct InertializeCubic {
    /// Offset at the moment the transition began.
    pub offset: Vec3,
    /// Offset velocity at the moment the transition began.
    pub velocity: Vec3,
    /// Seconds elapsed since the transition began.
    pub elapsed: f32,
}

impl InertializeCubic {
    /// No offset.
    pub const ZERO: Self =
        Self { offset: Vec3::ZERO, velocity: Vec3::ZERO, elapsed: 0.0 };

    /// Begins a transition, capturing the current offset and velocity.
    ///
    /// Unlike the exponential form, this captures once and then evaluates a
    /// closed-form curve against `elapsed` — so the remaining offset is a
    /// pure function of time, and the deadline cannot drift.
    #[inline]
    pub fn begin(
        current: Vec3,
        current_velocity: Vec3,
        target: Vec3,
        target_velocity: Vec3,
    ) -> Self {
        Self {
            offset: current - target,
            velocity: current_velocity - target_velocity,
            elapsed: 0.0,
        }
    }

    /// Advances the clock by `dt`.
    #[inline]
    pub fn advance(&mut self, dt: f32) {
        if dt > 0.0 {
            self.elapsed += dt;
        }
    }

    /// The offset remaining now, and its rate of change.
    ///
    /// Exactly `(ZERO, ZERO)` once `elapsed >= blend_time`.
    #[inline]
    pub fn evaluate(&self, blend_time: f32) -> (Vec3, Vec3) {
        let blend_time = blend_time.max(1.0e-5);

        if self.elapsed >= blend_time {
            return (Vec3::ZERO, Vec3::ZERO);
        }

        let t = (self.elapsed / blend_time).clamp(0.0, 1.0);
        let t2 = t * t;
        let t3 = t2 * t;

        // Cubic Hermite basis: w0/w1 carry the initial offset and velocity
        // to zero; w2/w3 are their derivatives.
        let w0 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let w1 = (t3 - 2.0 * t2 + t) * blend_time;
        let w2 = (6.0 * t2 - 6.0 * t) / blend_time;
        let w3 = 3.0 * t2 - 4.0 * t + 1.0;

        (self.offset * w0 + self.velocity * w1, self.offset * w2 + self.velocity * w3)
    }

    /// Whether the blend window has fully elapsed.
    #[inline]
    pub fn is_finished(&self, blend_time: f32) -> bool {
        self.elapsed >= blend_time.max(1.0e-5)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    /// See `quat_ext`'s own note: `Quat::angle_between` is `2*acos(|dot|)`
    /// and amplifies f32 rounding to a ~9.8e-4 rad floor near identity, so
    /// exactness is measured with `1 - |dot|` instead.
    fn rotation_mismatch(a: Quat, b: Quat) -> f32 {
        1.0 - a.dot(b).abs()
    }

    const EXACT: f32 = 1.0e-6;

    #[test]
    fn an_exponential_offset_starts_at_its_captured_value() {
        let (offset, velocity) = decay_exponential(5.0, -2.0, 0.1, 0.0);
        assert_eq!(offset, 5.0, "at t=0 the offset must be exactly what was captured");
        assert_eq!(velocity, -2.0, "and so must its velocity");
    }

    #[test]
    fn an_exponential_offset_decays_monotonically_toward_zero() {
        // Monotonicity matters: an offset that grew, even briefly, would
        // read as the character lurching away before settling.
        let mut previous = f32::INFINITY;
        for step in 0..200 {
            let t = step as f32 * 0.01;
            let (offset, _) = decay_exponential(1.0, 0.0, 0.1, t);
            assert!(
                offset.abs() <= previous + 1.0e-6,
                "the offset must never grow: {offset} at t={t} exceeded {previous}",
            );
            previous = offset.abs();
        }
        assert!(previous < 1.0e-4, "and must be negligible after 2s, got {previous}");
    }

    #[test]
    fn an_exponential_offset_halves_its_envelope_in_one_halflife() {
        // Pins the authoring contract, matching `spring`'s own equivalent
        // test: with zero initial velocity the remaining fraction at
        // t = halflife is e^{-ln2}(1 + ln2) = 0.5*(1 + ln2).
        let (offset, _) = decay_exponential(1.0, 0.0, 0.25, 0.25);
        let expected = 0.5 * (1.0 + LN_2);

        assert!(
            (offset - expected).abs() < 1.0e-4,
            "after one half-life the remaining offset should be {expected}, got {offset}",
        );
    }

    #[test]
    fn a_zero_halflife_does_not_produce_nan() {
        // A RON file with a missing or zeroed field must clamp to "very
        // fast", never NaN the rig.
        let (offset, velocity) = decay_exponential(1.0, 1.0, 0.0, 1.0 / 60.0);
        assert!(offset.is_finite(), "a zero halflife must not produce offset {offset}");
        assert!(velocity.is_finite(), "a zero halflife must not produce velocity {velocity}");
    }

    #[test]
    fn beginning_a_transition_reproduces_the_current_value_exactly() {
        // The no-jump guarantee: at the instant of the switch, the visible
        // output must not move at all.
        let current = Vec3::new(1.0, 2.0, 3.0);
        let target = Vec3::new(-4.0, 0.5, 2.0);

        let inertializer = Inertializer::begin(current, Vec3::ZERO, target, Vec3::ZERO);

        assert!(
            (inertializer.apply(target) - current).length() < 1.0e-6,
            "source + offset must reproduce the pre-switch value exactly, got {:?}",
            inertializer.apply(target),
        );
    }

    #[test]
    fn a_transition_converges_to_the_source_animation() {
        let current = Vec3::new(1.0, 2.0, 3.0);
        let target = Vec3::new(-4.0, 0.5, 2.0);

        let mut inertializer = Inertializer::begin(current, Vec3::ZERO, target, Vec3::ZERO);
        for _ in 0..120 {
            inertializer.advance(0.1, 1.0 / 60.0);
        }

        assert!(
            (inertializer.apply(target) - target).length() < 1.0e-3,
            "after 2s the offset should be gone, leaving the source animation, got {:?}",
            inertializer.apply(target),
        );
        assert!(inertializer.is_settled(1.0e-3), "and should report itself settled");
    }

    #[test]
    fn an_interrupted_transition_preserves_position_exactly() {
        // The no-jump half of the guarantee, asserted on its own because it
        // is exact: `begin` measures the offset from wherever the value
        // currently is, so the visible output cannot move at the instant of
        // the switch no matter how far apart the two animations are.
        let dt = 1.0 / 120.0;
        let halflife = 0.15;

        let first_target = Vec3::new(10.0, 0.0, 0.0);
        let mut inertializer =
            Inertializer::begin(Vec3::ZERO, Vec3::ZERO, first_target, Vec3::ZERO);
        for _ in 0..30 {
            inertializer.advance(halflife, dt);
        }
        let before = inertializer.apply(first_target);

        let second_target = Vec3::new(-5.0, 7.0, 0.0);
        let interrupted =
            Inertializer::begin(before, Vec3::ZERO, second_target, Vec3::ZERO);

        assert!(
            (interrupted.apply(second_target) - before).length() < 1.0e-5,
            "switching to a completely different target must not move the value at all: \
             {before:?} -> {:?}",
            interrupted.apply(second_target),
        );
    }

    #[test]
    fn an_interrupted_transition_carries_momentum_where_a_crossfade_destroys_it() {
        // THE headline property. Stated as a direct A/B against the
        // technique being replaced, because the absolute numbers are not
        // self-evidently good or bad — what matters is the comparison.
        //
        // Note what inertialization does NOT promise: it does not freeze
        // velocity across the switch. The new offset still decays, and a
        // decaying offset has velocity of its own, so some change is
        // inherent (about `1 - e^{-y*dt}` per step, ~3.8% here). What it
        // promises is that the OLD motion's velocity is carried into the
        // new transition rather than averaged away.
        let dt = 1.0 / 120.0;
        let halflife = 0.15;

        let first_target = Vec3::new(10.0, 0.0, 0.0);
        let second_target = Vec3::new(-5.0, 7.0, 0.0);

        // Fly toward the first target, then read off position and velocity.
        let mut inertializer =
            Inertializer::begin(Vec3::ZERO, Vec3::ZERO, first_target, Vec3::ZERO);
        let mut previous = inertializer.apply(first_target);
        let mut velocity_before = Vec3::ZERO;
        for _ in 0..30 {
            inertializer.advance(halflife, dt);
            let now = inertializer.apply(first_target);
            velocity_before = (now - previous) / dt;
            previous = now;
        }

        assert!(
            velocity_before.length() > 1.0,
            "test setup: the value should be moving briskly before the interruption, got {}",
            velocity_before.length(),
        );

        // (a) Inertialize: hand the measured velocity to the new transition.
        let mut inertialized =
            Inertializer::begin(previous, velocity_before, second_target, Vec3::ZERO);
        inertialized.advance(halflife, dt);
        let inertialized_velocity =
            (inertialized.apply(second_target) - previous) / dt;

        // (b) Crossfade: blend the old held pose toward the new one over a
        // window, which is what the smoothstep `PoseTransition` does. The
        // first step of any such blend moves by the blend weight alone and
        // knows nothing about how fast the value was already travelling.
        let blend_window = 0.2;
        let weight = dt / blend_window;
        let crossfaded = previous.lerp(second_target, weight);
        let crossfade_velocity = (crossfaded - previous) / dt;

        // How well does each preserve the direction the value was moving?
        let direction = velocity_before.normalize();
        let inertialized_retained = inertialized_velocity.dot(direction);
        let crossfade_retained = crossfade_velocity.dot(direction);

        assert!(
            inertialized_retained > 0.9 * velocity_before.length(),
            "inertialization should carry almost all of the prior speed into the new \
             transition, but retained only {inertialized_retained} of {}",
            velocity_before.length(),
        );
        assert!(
            crossfade_retained < 0.0,
            "test premise: a crossfade ignores prior velocity entirely — here it should \
             even reverse, having been handed a target in the opposite direction, but \
             retained {crossfade_retained}",
        );
        assert!(
            inertialized_retained > crossfade_retained + velocity_before.length(),
            "inertialization must preserve dramatically more momentum than a crossfade: \
             retained {inertialized_retained} vs {crossfade_retained} (prior speed {})",
            velocity_before.length(),
        );
    }

    #[test]
    fn velocity_continuity_across_an_interruption_scales_with_the_decay_rate() {
        // Quantifies the "some change is inherent" caveat above, and pins
        // it: the discontinuity must stay proportional to how much the
        // offset decays in one step, NOT to how far apart the two
        // animations are. A bug that forgot to carry `current_velocity`
        // into `begin` would fail this, because the error would then scale
        // with the prior speed instead.
        let dt = 1.0 / 120.0;

        for halflife in [0.05, 0.1, 0.2, 0.4] {
            let first_target = Vec3::new(10.0, 0.0, 0.0);

            let mut inertializer =
                Inertializer::begin(Vec3::ZERO, Vec3::ZERO, first_target, Vec3::ZERO);
            let mut previous = inertializer.apply(first_target);
            let mut velocity_before = Vec3::ZERO;
            for _ in 0..20 {
                inertializer.advance(halflife, dt);
                let now = inertializer.apply(first_target);
                velocity_before = (now - previous) / dt;
                previous = now;
            }

            // Switch to a target the value is ALREADY at, isolating the
            // velocity term: with no positional offset to decay, any
            // discontinuity is purely the carried velocity decaying.
            let mut switched =
                Inertializer::begin(previous, velocity_before, previous, Vec3::ZERO);
            switched.advance(halflife, dt);
            let velocity_after = (switched.apply(previous) - previous) / dt;

            let decay_per_step = 1.0 - (-halflife_to_decay_rate(halflife) * dt).exp();
            let change = (velocity_after - velocity_before).length();

            assert!(
                change <= velocity_before.length() * decay_per_step * 2.0 + 1.0e-3,
                "at halflife {halflife} the velocity change ({change}) should stay within \
                 the inherent per-step decay ({:.4} of {}), not scale with anything else",
                decay_per_step,
                velocity_before.length(),
            );
        }
    }

    #[test]
    fn a_rotation_transition_reproduces_the_current_orientation_exactly() {
        let current = Quat::from_axis_angle(Vec3::Y, 1.1);
        let target = Quat::from_axis_angle(Vec3::X, -0.4);

        let inertializer =
            RotationInertializer::begin(current, Vec3::ZERO, target, Vec3::ZERO);

        assert!(
            rotation_mismatch(inertializer.apply(target), current) < EXACT,
            "source * offset must reproduce the pre-switch orientation exactly",
        );
    }

    #[test]
    fn a_rotation_transition_converges_to_the_source_animation() {
        let current = Quat::from_axis_angle(Vec3::Y, 1.1);
        let target = Quat::from_axis_angle(Vec3::X, -0.4);

        let mut inertializer =
            RotationInertializer::begin(current, Vec3::ZERO, target, Vec3::ZERO);
        for _ in 0..120 {
            inertializer.advance(0.1, 1.0 / 60.0);
        }

        assert!(
            rotation_mismatch(inertializer.apply(target), target) < 1.0e-5,
            "after 2s the rotational offset should be gone",
        );
    }

    #[test]
    fn a_rotation_transition_between_a_quaternion_and_its_negation_has_no_offset() {
        // Neighbourhooding, stated as the property the rig depends on: `q`
        // and `-q` are the same orientation, so switching between them must
        // produce no motion at all. Fails if the guard in `begin` is removed.
        let q = Quat::from_axis_angle(Vec3::Y, 0.9);

        let inertializer = RotationInertializer::begin(q, Vec3::ZERO, -q, Vec3::ZERO);

        assert!(
            inertializer.offset.length() < 1.0e-4,
            "q and -q name the same rotation, so the captured offset must be zero, got {:?}",
            inertializer.offset,
        );
    }

    #[test]
    fn a_rotation_transition_never_takes_the_long_way_round() {
        // A 170-degree switch should decay through ~170 degrees, not ~190.
        let current = Quat::from_axis_angle(Vec3::Y, 170.0_f32.to_radians());
        let target = Quat::IDENTITY;

        let inertializer =
            RotationInertializer::begin(current, Vec3::ZERO, target, Vec3::ZERO);

        let magnitude = inertializer.offset.length();
        assert!(
            magnitude <= std::f32::consts::PI + 1.0e-3,
            "the captured offset must be the short way round (<= pi), got {magnitude} rad",
        );
        assert!(
            (magnitude - 170.0_f32.to_radians()).abs() < 1.0e-3,
            "expected a 170-degree offset, got {} degrees",
            magnitude.to_degrees(),
        );
    }

    #[test]
    fn a_cubic_offset_starts_at_its_captured_value() {
        let cubic = InertializeCubic::begin(
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::ZERO,
            Vec3::ZERO,
        );
        let (offset, velocity) = cubic.evaluate(0.5);

        assert!((offset - Vec3::new(3.0, 0.0, 0.0)).length() < 1.0e-6);
        assert!((velocity - Vec3::new(1.0, 0.0, 0.0)).length() < 1.0e-6);
    }

    #[test]
    fn a_cubic_offset_reaches_exactly_zero_at_the_blend_deadline() {
        // The whole reason the cubic form exists alongside the exponential
        // one. Foot locking needs the offset GONE by the deadline, not small.
        let blend_time = 0.25;
        let mut cubic = InertializeCubic::begin(
            Vec3::new(3.0, -2.0, 1.0),
            Vec3::new(5.0, 0.0, -1.0),
            Vec3::ZERO,
            Vec3::ZERO,
        );

        cubic.advance(blend_time);

        let (offset, velocity) = cubic.evaluate(blend_time);
        assert_eq!(offset, Vec3::ZERO, "the cubic offset must be exactly zero at the deadline");
        assert_eq!(velocity, Vec3::ZERO, "and so must its velocity");
        assert!(cubic.is_finished(blend_time));
    }

    #[test]
    fn a_cubic_offset_stays_exactly_zero_past_its_deadline() {
        let blend_time = 0.25;
        let mut cubic =
            InertializeCubic::begin(Vec3::splat(4.0), Vec3::splat(9.0), Vec3::ZERO, Vec3::ZERO);

        cubic.advance(blend_time * 10.0);

        assert_eq!(cubic.evaluate(blend_time), (Vec3::ZERO, Vec3::ZERO));
    }

    #[test]
    fn a_cubic_offset_decays_smoothly_with_no_step_changes() {
        // Guards C1 continuity across the whole window: a jump in the
        // decay would read as a visible pop mid-transition.
        let blend_time = 0.4;
        let dt = 1.0 / 240.0;

        let mut cubic = InertializeCubic::begin(
            Vec3::new(2.0, 0.0, 0.0),
            Vec3::new(-3.0, 0.0, 0.0),
            Vec3::ZERO,
            Vec3::ZERO,
        );

        let mut previous_offset = cubic.evaluate(blend_time).0;
        let mut previous_step = f32::INFINITY;

        while !cubic.is_finished(blend_time) {
            cubic.advance(dt);
            let offset = cubic.evaluate(blend_time).0;
            let step = (offset - previous_offset).length();

            if previous_step.is_finite() {
                assert!(
                    (step - previous_step).abs() < 0.02,
                    "the per-frame change should vary smoothly, but jumped from \
                     {previous_step} to {step}",
                );
            }

            previous_step = step;
            previous_offset = offset;
        }
    }

    #[test]
    fn a_zero_blend_time_does_not_divide_by_zero() {
        let cubic =
            InertializeCubic::begin(Vec3::splat(1.0), Vec3::splat(1.0), Vec3::ZERO, Vec3::ZERO);
        let (offset, velocity) = cubic.evaluate(0.0);

        assert!(offset.is_finite(), "a zero blend time must not produce {offset:?}");
        assert!(velocity.is_finite(), "a zero blend time must not produce {velocity:?}");
    }

    #[test]
    fn advancing_by_a_non_positive_timestep_is_a_no_op() {
        let mut inertializer =
            Inertializer::begin(Vec3::splat(1.0), Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        let before = inertializer;
        inertializer.advance(0.1, 0.0);
        inertializer.advance(0.1, -1.0);
        assert_eq!(inertializer, before);

        let mut cubic =
            InertializeCubic::begin(Vec3::splat(1.0), Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        let before = cubic;
        cubic.advance(0.0);
        cubic.advance(-1.0);
        assert_eq!(cubic, before);
    }

    #[test]
    fn a_rotation_offset_composes_in_the_targets_own_local_frame() {
        // Pins the post-multiply convention: `begin` measures the offset in
        // the target's frame, so `apply` must post-multiply. Swapping the
        // order here silently mirrors every transition through the parent
        // frame, which is hard to spot on a symmetric pose.
        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);
        let local_offset = Quat::from_axis_angle(Vec3::X, 0.3);
        let current = target * local_offset;

        let inertializer =
            RotationInertializer::begin(current, Vec3::ZERO, target, Vec3::ZERO);

        assert!(
            inertializer.offset.normalize().dot(Vec3::X) > 0.999,
            "the offset should be about X in the target's own frame, got {:?}",
            inertializer.offset,
        );
    }
}
