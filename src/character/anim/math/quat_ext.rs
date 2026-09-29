//! Quaternion primitives the rotation-space animation stack is built on:
//! the exponential/logarithm maps (`from_scaled_angle_axis`/
//! `to_scaled_angle_axis`) that let a quaternion be sprung in a flat tangent
//! space, plus the two guards — [`neighborhood`] and [`canonical`] — that
//! keep every quaternion difference on the short way round.
//!
//! # Why the tangent space matters
//!
//! A quaternion cannot be integrated like a vector: adding an angular
//! velocity to `(x, y, z, w)` componentwise leaves the 4D unit hypersphere
//! immediately. The standard fix, used by every stage here, is to do the
//! *dynamics* in the tangent space at the current orientation — a plain
//! `Vec3` of "axis scaled by angle", which adds and scales like any other
//! vector — and map back to a quaternion only at the end. [`quat_log`]
//! goes to that space, [`quat_exp`] comes back.
//!
//! # Why neighborhooding matters (this is the bug class, not a detail)
//!
//! `q` and `-q` name the *same* 3D rotation but sit at opposite poles of the
//! 4D hypersphere. Any code that takes a difference between two quaternions
//! without first putting them in the same hemisphere will, roughly half the
//! time, compute the complementary rotation — the bone swings the 359° way
//! round instead of the 1° way. The failure is silent in the data (both
//! quaternions are perfectly valid, unit length, and name the intended
//! orientation) and spectacular on screen.
//!
//! This module's answer is to make the guard a *named, tested function*
//! rather than an inline `if dot < 0.0` that each call site has to remember.
//! [`neighborhood`] is called before every difference taken anywhere in
//! `character::anim`, and `neighborhood_flips_a_quaternion_that_took_the_long_way_round`
//! fails loudly if it is ever removed.

use bevy::math::{Quat, Vec3};

/// Below this half-angle the small-angle series is both cheaper and *more*
/// accurate than the trigonometric form, whose `sin(t)/t` factor loses
/// precision as `t -> 0`.
const SMALL_ANGLE: f32 = 1.0e-4;

/// Exponential map: a rotation vector (axis scaled by **half** the angle)
/// back to a quaternion.
///
/// Prefer [`from_scaled_angle_axis`] at call sites — it takes a full angle,
/// which is what every other API in this crate speaks. This function exists
/// because the halving belongs to the quaternion algebra, not to the caller.
#[inline]
pub fn quat_exp(v: Vec3) -> Quat {
    let half_angle = v.length();

    if half_angle < SMALL_ANGLE {
        // sin(t)/t -> 1 and cos(t) -> 1 as t -> 0; normalizing absorbs the
        // second-order error rather than letting it accumulate.
        Quat::from_xyzw(v.x, v.y, v.z, 1.0).normalize()
    } else {
        let c = half_angle.cos();
        let s = half_angle.sin() / half_angle;
        Quat::from_xyzw(s * v.x, s * v.y, s * v.z, c)
    }
}

/// Logarithm map: a quaternion to a rotation vector (axis scaled by **half**
/// the angle). Inverse of [`quat_exp`].
///
/// The input is canonicalized first, so the result always describes the
/// shorter of the two equivalent rotations.
#[inline]
pub fn quat_log(q: Quat) -> Vec3 {
    let q = canonical(q);
    let axis_length = Vec3::new(q.x, q.y, q.z).length();

    if axis_length < SMALL_ANGLE {
        Vec3::new(q.x, q.y, q.z)
    } else {
        // atan2 rather than acos(w): it stays accurate as the rotation
        // approaches pi, where acos's derivative blows up.
        let half_angle = axis_length.atan2(q.w);
        Vec3::new(q.x, q.y, q.z) * (half_angle / axis_length)
    }
}

/// A rotation vector — axis scaled by the **full** angle in radians — to a
/// quaternion. This is the form the DHO integrates and the PD controller
/// emits, because an angular velocity in rad/s scales and adds in exactly
/// this space.
#[inline]
pub fn from_scaled_angle_axis(v: Vec3) -> Quat {
    quat_exp(v * 0.5)
}

/// A quaternion to a rotation vector (axis scaled by the **full** angle in
/// radians). Inverse of [`from_scaled_angle_axis`].
#[inline]
pub fn to_scaled_angle_axis(q: Quat) -> Vec3 {
    quat_log(q) * 2.0
}

/// Returns `b`, negated if needed, so that it lies in the same hemisphere as
/// `a` — i.e. so `a.dot(result) >= 0`.
///
/// **Call this before every quaternion difference.** See the module doc for
/// why. The returned quaternion names the identical 3D rotation as `b`; only
/// the path taken to reach it from `a` changes, and that path is what a
/// spring, a slerp, or a PD error term actually integrates along.
#[inline]
pub fn neighborhood(a: Quat, b: Quat) -> Quat {
    if a.dot(b) < 0.0 { -b } else { b }
}

/// Returns `q` mapped into the `w >= 0` hemisphere.
///
/// Use this when there is no reference orientation to be neighbourly *to* —
/// typically when measuring a rotation's own magnitude, where the caller
/// wants the shorter of the two equivalent readings.
#[inline]
pub fn canonical(q: Quat) -> Quat {
    if q.w < 0.0 { -q } else { q }
}

/// The rotation taking `from` to `to`, expressed as a rotation vector (axis
/// scaled by the full angle, radians), measured in `from`'s own frame.
///
/// Neighbourhooded and canonicalized, so the magnitude is always the short
/// way round and never exceeds `pi`. This is the error term the DHO and the
/// ragdoll PD controller both spring toward zero.
#[inline]
pub fn rotation_delta(from: Quat, to: Quat) -> Vec3 {
    let to = neighborhood(from, to);
    to_scaled_angle_axis(from.inverse() * to)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    /// How far apart two rotations are, measured **without** `acos`.
    ///
    /// `Quat::angle_between` is `2 * acos(|dot|)`, and `acos` has an infinite
    /// derivative at 1 — so near identity the ~1.2e-7 rounding error in an
    /// f32 `dot` is amplified to a floor of ~9.8e-4 rad of *apparent* angle.
    /// Asserting a tighter bound than that with `angle_between` measures the
    /// measurement, not the code (verified: two bit-identical quaternions
    /// report 9.76e-4 rad between them).
    ///
    /// `1 - |dot|` has no such amplification, so it is what these tests use
    /// to pin down exactness.
    fn rotation_mismatch(a: Quat, b: Quat) -> f32 {
        1.0 - a.dot(b).abs()
    }

    /// The tightest mismatch worth asserting in `f32`.
    ///
    /// A perfect round trip still lands one unit in the last place away, so
    /// the observed floor for bit-identical-looking quaternions is
    /// `f32::EPSILON` (~1.19e-7). This sits a few ULP above that. It is
    /// still an extremely tight bound: since `1 - |dot| ~ angle^2 / 8`, a
    /// mismatch of `1e-6` corresponds to about **0.003 rad (0.16 degrees)**,
    /// far finer than any animation tolerance in this project.
    const EXACT: f32 = 1.0e-6;

    /// Deterministic pseudo-random unit quaternions. Seeded explicitly rather
    /// than using `fastrand`'s global state so a failure always reproduces —
    /// a hard requirement for a module whose bugs are this subtle.
    fn seeded_quats(count: usize) -> Vec<Quat> {
        let mut rng = fastrand::Rng::with_seed(0x5EED_1234);
        (0..count)
            .map(|_| {
                let axis = Vec3::new(
                    rng.f32() * 2.0 - 1.0,
                    rng.f32() * 2.0 - 1.0,
                    rng.f32() * 2.0 - 1.0,
                );
                let axis = if axis.length() < 1.0e-3 { Vec3::X } else { axis.normalize() };
                // Span nearly the full range, including rotations close to pi
                // where the log map is most delicate.
                Quat::from_axis_angle(axis, rng.f32() * (PI * 1.98) - PI * 0.99)
            })
            .collect()
    }

    #[test]
    fn exp_and_log_round_trip_for_every_seeded_quaternion() {
        for q in seeded_quats(1000) {
            let round_tripped = from_scaled_angle_axis(to_scaled_angle_axis(q));

            let mismatch = rotation_mismatch(q, round_tripped);
            assert!(
                mismatch < EXACT,
                "exp(log(q)) should reproduce q exactly, but mismatched by {mismatch} \
                 (q = {q:?}, round-tripped = {round_tripped:?})",
            );
        }
    }

    #[test]
    fn exp_and_log_round_trip_through_the_small_angle_branch() {
        // Straddle SMALL_ANGLE so both branches, and the seam between them,
        // are covered — a discontinuity here would show up as a stutter at
        // the exact moment a spring settles.
        for exponent in 0..12 {
            let angle = 1.0e-7 * 10.0_f32.powi(exponent);
            let q = Quat::from_axis_angle(Vec3::Y, angle);
            let round_tripped = from_scaled_angle_axis(to_scaled_angle_axis(q));

            let mismatch = rotation_mismatch(q, round_tripped);
            assert!(
                mismatch < EXACT,
                "a {angle}-rad rotation should survive an exp/log round trip, \
                 but mismatched by {mismatch}",
            );
        }
    }

    #[test]
    fn the_identity_rotation_maps_to_a_zero_vector() {
        let v = to_scaled_angle_axis(Quat::IDENTITY);
        assert!(
            v.length() < 1.0e-6,
            "the identity rotation must have zero magnitude in tangent space, got {v:?}",
        );
    }

    #[test]
    fn a_scaled_angle_axis_vector_carries_the_full_angle_not_the_half_angle() {
        // Guards the factor-of-two seam between quat_exp (half-angle) and
        // from_scaled_angle_axis (full angle) — an easy place to drop a 0.5
        // and get a rig that rotates at half or double speed.
        let v = to_scaled_angle_axis(Quat::from_axis_angle(Vec3::Z, FRAC_PI_2));

        assert!(
            (v.length() - FRAC_PI_2).abs() < 1.0e-5,
            "a 90-degree rotation should have magnitude {FRAC_PI_2} in scaled-angle-axis \
             space, got {} — check the half-angle conversion",
            v.length(),
        );
        assert!(
            v.normalize().dot(Vec3::Z) > 0.999,
            "the scaled-angle-axis vector should point along the rotation axis, got {v:?}",
        );
    }

    #[test]
    fn neighborhood_flips_a_quaternion_that_took_the_long_way_round() {
        // THE regression test for the module's headline bug class. If
        // `neighborhood` is ever reduced to a no-op, this fails.
        let a = Quat::from_axis_angle(Vec3::Y, 0.1);
        let b = -a;

        assert!(a.dot(b) < 0.0, "test setup: -a must start in the opposite hemisphere");

        let neighbored = neighborhood(a, b);
        assert!(
            a.dot(neighbored) >= 0.0,
            "neighborhood must return a quaternion in the same hemisphere as the reference",
        );
        assert!(
            a.angle_between(neighbored) < 1.0e-5,
            "a and -a name the SAME rotation, so after neighborhooding the angle between \
             them must be ~0, got {} rad",
            a.angle_between(neighbored),
        );
    }

    #[test]
    fn neighborhood_leaves_an_already_nearby_quaternion_untouched() {
        let a = Quat::from_axis_angle(Vec3::Y, 0.1);
        let b = Quat::from_axis_angle(Vec3::Y, 0.2);

        assert_eq!(
            neighborhood(a, b),
            b,
            "a quaternion already in the reference hemisphere must be returned unchanged",
        );
    }

    #[test]
    fn rotation_delta_between_a_quaternion_and_its_own_negation_is_zero() {
        // The practical consequence of neighborhooding, stated as the
        // property the rest of the stack actually relies on: springing
        // toward `-q` when already at `q` must produce NO motion.
        for q in seeded_quats(200) {
            let delta = rotation_delta(q, -q);
            assert!(
                delta.length() < 1.0e-4,
                "q and -q are the same rotation, so the delta between them must be zero, \
                 got magnitude {} (q = {q:?})",
                delta.length(),
            );
        }
    }

    #[test]
    fn rotation_delta_never_exceeds_pi() {
        // If this ever fails, some bone is being asked to take the long way
        // round, which is exactly what neighborhooding exists to prevent.
        for q in seeded_quats(500) {
            for r in seeded_quats(20) {
                let magnitude = rotation_delta(q, r).length();
                assert!(
                    magnitude <= PI + 1.0e-3,
                    "the short-way-round delta can never exceed pi, got {magnitude} rad",
                );
            }
        }
    }

    #[test]
    fn rotation_delta_recovers_a_known_rotation_in_the_source_frame() {
        let from = Quat::from_axis_angle(Vec3::Y, 0.7);
        let applied = Quat::from_axis_angle(Vec3::X, 0.3);
        // `to` is `from` followed by `applied`, expressed in from's own frame.
        let to = from * applied;

        let delta = rotation_delta(from, to);

        assert!(
            (delta.length() - 0.3).abs() < 1.0e-5,
            "expected a 0.3-rad delta, got {}",
            delta.length(),
        );
        assert!(
            delta.normalize().dot(Vec3::X) > 0.999,
            "the delta should be about the X axis in the source frame, got {delta:?}",
        );
    }

    #[test]
    fn rotation_delta_composed_back_onto_the_source_reproduces_the_target() {
        // The round trip the DHO relies on every frame: measure a delta,
        // integrate it, compose it back.
        for (i, q) in seeded_quats(200).into_iter().enumerate() {
            let target = seeded_quats(200)[(i + 71) % 200];

            let reconstructed = q * from_scaled_angle_axis(rotation_delta(q, target));

            let mismatch = rotation_mismatch(target, reconstructed);
            assert!(
                mismatch < EXACT,
                "composing a measured delta back onto its source must reproduce the target, \
                 but mismatched by {mismatch}",
            );
        }
    }

    #[test]
    fn canonical_maps_into_the_positive_w_hemisphere_without_changing_the_rotation() {
        for q in seeded_quats(200) {
            let c = canonical(-q);
            assert!(c.w >= 0.0, "canonical must return w >= 0, got {}", c.w);
            assert!(
                rotation_mismatch(q, c) < EXACT,
                "canonicalizing must not change which rotation is named",
            );
        }
    }
}
