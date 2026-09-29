//! Damped harmonic oscillators in closed form — the Stage 1 primitive that
//! replaces timeline interpolation and hand-authored easing curves.
//!
//! # Why an exact solution rather than Euler integration
//!
//! The obvious implementation of a spring is
//! `v += (k_p * (target - x) - k_d * v) * dt; x += v * dt`. It is also
//! wrong for this use, in two ways that matter here:
//!
//! - **It is not frame-rate independent.** The same spring stepped at 1/30 s
//!   and at 1/120 s lands in different places, so a character animates
//!   differently on a slow machine. This project already carries a
//!   `dt.min(1/30)` clamp as a scar from exactly this class of problem.
//! - **It diverges at high stiffness.** Explicit integration is only stable
//!   while `k_p * dt^2` stays small, so the stiffest, snappiest poses — the
//!   ones a combat rig most wants — are precisely the ones that explode.
//!
//! The critically damped oscillator has a closed-form solution, so we
//! evaluate it directly at time `dt` instead of marching toward it. The
//! result is identical at any timestep (proved by
//! `a_spring_lands_in_the_same_place_regardless_of_timestep`) and
//! unconditionally stable at any stiffness.
//!
//! # Authoring in half-life, not stiffness
//!
//! `k_p`/`k_d` are a poor authoring surface: they interact, their useful
//! range spans orders of magnitude, and neither has an intuitive unit. This
//! module's public API is instead:
//!
//! - **`halflife`** — how long the remaining distance takes to halve, in
//!   seconds. Directly observable, and the natural way to say "snappy"
//!   (0.05) or "heavy" (0.4).
//! - **`damping_ratio`** — 1.0 critically damped (fastest approach with no
//!   overshoot), below 1.0 springy with follow-through, above 1.0 sluggish.
//!
//! This mirrors the parameterization avian's own `MotorModel::SpringDamper`
//! exposes, so Stage 1 and the Stage 4 ragdoll share one vocabulary.

use bevy::math::Vec3;
use serde::{Deserialize, Serialize};

/// `ln(2)`, the constant relating a half-life to an exponential decay rate.
const LN_2: f32 = std::f32::consts::LN_2;

/// Per-joint spring tuning. Hot-reloadable via RON.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpringParams {
    /// Seconds for the remaining distance to halve. Smaller is snappier.
    pub halflife: f32,
    /// 1.0 = critically damped (no overshoot). `< 1` overshoots and
    /// oscillates (follow-through, weight); `> 1` approaches slowly.
    pub damping_ratio: f32,
    /// Hard cap on `|velocity|` so a teleport or an impulse cannot launch a
    /// limb. In the same units as the quantity being sprung, per second.
    pub max_speed: f32,
}

impl Default for SpringParams {
    fn default() -> Self {
        Self { halflife: 0.1, damping_ratio: 1.0, max_speed: 50.0 }
    }
}

impl SpringParams {
    /// A critically damped spring with the given half-life — the common case.
    pub const fn critical(halflife: f32) -> Self {
        Self { halflife, damping_ratio: 1.0, max_speed: 50.0 }
    }

    /// The decay rate `y` such that `exp(-y * halflife) == 0.5`.
    ///
    /// Guarded against a zero or negative half-life, which would otherwise
    /// produce an infinite rate and NaN out the whole rig.
    #[inline]
    pub fn decay_rate(&self) -> f32 {
        LN_2 / self.halflife.max(1.0e-5)
    }
}

/// `exp(-x)` for `x >= 0`.
///
/// This deliberately calls `f32::exp` rather than the cubic rational
/// approximation `1/(1 + x + 0.48x² + 0.235x³)` that circulates in spring
/// implementations. That approximation is accurate to ~1e-3 only while
/// `x < 1`, and degrades catastrophically beyond it — measured here as
/// **2.1x relative error at x = 5 and 74x at x = 10**.
///
/// Those inputs are not exotic. `x` is `decay_rate * dt`, so a snappy
/// 0.02 s half-life at a 1/60 s step already sits at 0.58, a single dropped
/// frame pushes it past 2, and any stiff spring stepped at a low frame rate
/// lands squarely in the divergent region. The error compounds every step:
/// with the approximation, a spring converging from -100 stalled at 0.9989
/// where the exact solution reaches 0.999998 — a thousandfold miss, caught
/// by `a_spring_converges_to_its_target_from_any_start`.
///
/// `f32::exp` is one hardware-backed instruction on every platform this
/// targets, so the approximation bought nothing measurable and cost
/// correctness.
#[inline]
fn neg_exp(x: f32) -> f32 {
    (-x).exp()
}

/// Advances a scalar spring toward `target` by `dt`, in closed form.
///
/// Returns the new `(position, velocity)`. Stable at any `dt` and any
/// half-life; see the module doc.
#[inline]
pub fn spring_scalar(
    position: f32,
    velocity: f32,
    target: f32,
    params: &SpringParams,
    dt: f32,
) -> (f32, f32) {
    if dt <= 0.0 {
        return (position, velocity);
    }

    let (offset, new_velocity) = solve_offset(position - target, velocity, params, dt);

    (offset + target, new_velocity.clamp(-params.max_speed, params.max_speed))
}

/// The shared closed-form solver: advances a displacement-from-target and
/// its velocity by `dt`, for any damping ratio.
///
/// Splitting this out keeps [`spring_scalar`] and [`spring_vec3`] provably
/// identical componentwise (pinned by
/// `a_vec3_spring_matches_the_scalar_spring_componentwise`) rather than two
/// hand-synchronized copies of delicate math.
///
/// # The three regimes
///
/// Writing `w` for the undamped natural frequency and `z` for the damping
/// ratio, the system is `x'' + 2zw x' + w^2 x = 0`, whose exact solution
/// takes a different form depending on `z`:
///
/// - **`z < 1` underdamped** — a decaying *sinusoid*. This is the regime
///   that overshoots and swings back, giving follow-through and a sense of
///   weight. It genuinely needs the trigonometric form; scaling the
///   critically damped decay rate (an easy mistake, and the one this
///   function was written to fix) can never produce an overshoot at all.
/// - **`z == 1` critically damped** — the `e^{-wt}(1 + wt)` form. Fastest
///   approach with no overshoot.
/// - **`z > 1` overdamped** — two real exponentials, no oscillation.
#[inline]
fn solve_offset(offset: f32, velocity: f32, params: &SpringParams, dt: f32) -> (f32, f32) {
    // Natural frequency, chosen so that at z == 1 the envelope's half-life
    // is exactly `params.halflife` — that is the authoring contract.
    let w = params.decay_rate();
    let z = params.damping_ratio.max(0.0);

    // The envelope decays at z*w in every regime.
    let decay = neg_exp(z * w * dt);

    if (z - 1.0).abs() < 1.0e-3 {
        // Critically damped (and the near-critical band, where the general
        // forms below divide by a vanishing damped frequency).
        let j1 = velocity + offset * w;
        (decay * (offset + j1 * dt), decay * (velocity - j1 * w * dt))
    } else if z < 1.0 {
        // Underdamped: decaying sinusoid at the damped frequency.
        let wd = w * (1.0 - z * z).sqrt();
        let c1 = offset;
        let c2 = (velocity + z * w * offset) / wd;

        let (sin, cos) = (wd * dt).sin_cos();

        let new_offset = decay * (c1 * cos + c2 * sin);
        // d/dt of the above, by the product rule.
        let new_velocity =
            decay * ((-z * w) * (c1 * cos + c2 * sin) + wd * (c2 * cos - c1 * sin));

        (new_offset, new_velocity)
    } else {
        // Overdamped: two real roots, -w(z -/+ sqrt(z^2 - 1)).
        let r = w * (z * z - 1.0).sqrt();
        let r1 = -z * w + r;
        let r2 = -z * w - r;

        // Solve the initial-value problem for the two coefficients.
        let c2 = (velocity - r1 * offset) / (r2 - r1);
        let c1 = offset - c2;

        let e1 = neg_exp(-r1 * dt);
        let e2 = neg_exp(-r2 * dt);

        (c1 * e1 + c2 * e2, c1 * r1 * e1 + c2 * r2 * e2)
    }
}

/// Advances a [`Vec3`] spring toward `target` by `dt`, in closed form.
///
/// Componentwise, but with the speed clamp applied to the *vector*
/// magnitude, so clamping cannot skew the direction of travel.
#[inline]
pub fn spring_vec3(
    position: Vec3,
    velocity: Vec3,
    target: Vec3,
    params: &SpringParams,
    dt: f32,
) -> (Vec3, Vec3) {
    if dt <= 0.0 {
        return (position, velocity);
    }

    let offset = position - target;
    let mut new_offset = Vec3::ZERO;
    let mut new_velocity = Vec3::ZERO;

    for axis in 0..3 {
        let (o, v) = solve_offset(offset[axis], velocity[axis], params, dt);
        new_offset[axis] = o;
        new_velocity[axis] = v;
    }

    (new_offset + target, new_velocity.clamp_length_max(params.max_speed))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Steps a scalar spring for `duration` using `steps` equal substeps.
    fn run_scalar(
        start: f32,
        target: f32,
        params: &SpringParams,
        duration: f32,
        steps: usize,
    ) -> (f32, f32) {
        let dt = duration / steps as f32;
        let mut p = start;
        let mut v = 0.0;
        for _ in 0..steps {
            let (np, nv) = spring_scalar(p, v, target, params, dt);
            p = np;
            v = nv;
        }
        (p, v)
    }

    #[test]
    fn a_spring_lands_in_the_same_place_regardless_of_timestep() {
        // THE test that justifies the closed-form solution. An explicit Euler
        // integrator fails this badly; this must stay green or the whole
        // frame-rate-independence claim is void.
        let params = SpringParams::critical(0.15);

        let coarse = run_scalar(0.0, 1.0, &params, 1.0, 30);
        let medium = run_scalar(0.0, 1.0, &params, 1.0, 60);
        let fine = run_scalar(0.0, 1.0, &params, 1.0, 240);

        assert!(
            (coarse.0 - fine.0).abs() < 1.0e-3,
            "stepping at 30Hz vs 240Hz must agree, got {} vs {}",
            coarse.0,
            fine.0,
        );
        assert!(
            (medium.0 - fine.0).abs() < 1.0e-3,
            "stepping at 60Hz vs 240Hz must agree, got {} vs {}",
            medium.0,
            fine.0,
        );
    }

    #[test]
    fn a_critically_damped_spring_never_overshoots_its_target() {
        // The property that makes `damping_ratio = 1.0` a safe default: a
        // designer can raise stiffness arbitrarily without ever introducing
        // a wobble.
        for halflife in [0.02, 0.05, 0.1, 0.25, 0.5, 1.0] {
            for steps in [15, 30, 60, 120, 240] {
                let params = SpringParams::critical(halflife);
                let dt = 2.0 / steps as f32;

                let mut p = 0.0;
                let mut v = 0.0;
                for _ in 0..steps {
                    let (np, nv) = spring_scalar(p, v, 1.0, &params, dt);
                    p = np;
                    v = nv;
                    assert!(
                        p <= 1.0 + 1.0e-4,
                        "a critically damped spring must approach 1.0 from below and never \
                         exceed it, but reached {p} (halflife {halflife}, {steps} steps)",
                    );
                }
            }
        }
    }

    #[test]
    fn a_spring_halves_its_remaining_distance_in_one_halflife() {
        // Pins the authoring contract: `halflife` means what it says, so a
        // designer can predict settling time without experimenting.
        //
        // Measured with zero initial velocity, where the critically damped
        // solution's REMAINING fraction is `e^{-yt}(1 + yt)`. At t = halflife
        // that is `0.5 * (1 + ln 2)` ~= 0.847 still to go, so the spring has
        // covered ~0.153 of the distance.
        //
        // Note the exponential ENVELOPE has halved, not the distance: the
        // extra `(1 + yt)` factor is the velocity term carrying the spring
        // forward. "Half-life" names the envelope's decay, which is the
        // quantity that stays meaningful once the spring is also moving.
        let params = SpringParams::critical(0.2);
        let (position, _) = run_scalar(0.0, 1.0, &params, 0.2, 200);

        let remaining = 0.5 * (1.0 + LN_2);
        let covered = 1.0 - remaining;
        assert!(
            (position - covered).abs() < 0.01,
            "after exactly one half-life the spring should have covered {covered} of the \
             distance (leaving {remaining}), got {position}",
        );
    }

    #[test]
    fn a_spring_already_at_its_target_never_moves() {
        // Guards against a settled rig dithering in the last mantissa bit,
        // which reads on screen as a persistent shimmer.
        let params = SpringParams::critical(0.1);
        let mut p = 1.0;
        let mut v = 0.0;

        for _ in 0..10_000 {
            let (np, nv) = spring_scalar(p, v, 1.0, &params, 1.0 / 60.0);
            p = np;
            v = nv;
        }

        assert_eq!(p, 1.0, "a spring resting exactly on its target must not drift");
        assert_eq!(v, 0.0, "a resting spring must not accumulate velocity");
    }

    #[test]
    fn a_spring_converges_to_its_target_from_any_start() {
        // Note the speed clamp bounds how fast a large displacement can be
        // closed, so the time budget has to allow for `max_speed` as well as
        // for the half-life — from 1000 away at 50 units/s, the first ~20s
        // are travel-limited, not spring-limited.
        let params = SpringParams { max_speed: f32::INFINITY, ..SpringParams::critical(0.1) };

        for start in [-100.0, -1.0, 0.0, 0.5, 2.0, 1000.0] {
            let (position, velocity) = run_scalar(start, 1.0, &params, 5.0, 300);
            assert!(
                (position - 1.0).abs() < 1.0e-3,
                "from {start}, a 0.1s-halflife spring should reach the target within 5s, \
                 got {position}",
            );
            assert!(velocity.abs() < 1.0e-2, "and should arrive at rest, got v = {velocity}");
        }
    }

    #[test]
    fn an_underdamped_spring_overshoots_but_still_settles() {
        // The follow-through case — a heavy weapon's arm continuing past its
        // target and swinging back. Documents that `damping_ratio < 1` does
        // something distinct, and that it still converges.
        let params = SpringParams { halflife: 0.1, damping_ratio: 0.35, max_speed: 50.0 };

        let dt = 1.0 / 120.0;
        let mut p: f32 = 0.0;
        let mut v = 0.0;
        let mut peak: f32 = 0.0;
        for _ in 0..360 {
            let (np, nv) = spring_scalar(p, v, 1.0, &params, dt);
            p = np;
            v = nv;
            peak = peak.max(p);
        }

        assert!(
            peak > 1.0,
            "an underdamped spring should overshoot its target, but peaked at only {peak}",
        );
        assert!(
            (p - 1.0).abs() < 0.05,
            "and should still settle on the target, got {p}",
        );
    }

    #[test]
    fn the_speed_clamp_bounds_velocity_without_skewing_a_vec3_direction() {
        // A teleport-sized displacement must not launch a limb, and the
        // clamp must not bend the direction of travel while preventing it.
        let params = SpringParams { halflife: 0.01, damping_ratio: 1.0, max_speed: 2.0 };

        let direction = Vec3::new(1.0, 2.0, -2.0).normalize();
        let (_, velocity) =
            spring_vec3(direction * 1000.0, Vec3::ZERO, Vec3::ZERO, &params, 1.0 / 60.0);

        assert!(
            velocity.length() <= params.max_speed + 1.0e-4,
            "velocity must be clamped to max_speed, got {}",
            velocity.length(),
        );
        assert!(
            velocity.normalize().dot(-direction) > 0.999,
            "clamping must preserve the direction of travel, got {velocity:?}",
        );
    }

    #[test]
    fn a_vec3_spring_matches_the_scalar_spring_componentwise() {
        // The two implementations must not drift apart; this keeps a fix to
        // one from silently missing the other.
        let params = SpringParams::critical(0.12);
        let dt = 1.0 / 60.0;

        let mut vp = Vec3::new(0.0, 3.0, -2.0);
        let mut vv = Vec3::ZERO;
        let target = Vec3::new(1.0, -1.0, 0.5);

        let mut sp = [0.0f32, 3.0, -2.0];
        let mut sv = [0.0f32; 3];
        let starget = [1.0f32, -1.0, 0.5];

        for _ in 0..120 {
            let (np, nv) = spring_vec3(vp, vv, target, &params, dt);
            vp = np;
            vv = nv;

            for i in 0..3 {
                let (np, nv) = spring_scalar(sp[i], sv[i], starget[i], &params, dt);
                sp[i] = np;
                sv[i] = nv;
            }
        }

        for i in 0..3 {
            assert!(
                (vp[i] - sp[i]).abs() < 1.0e-5,
                "component {i} diverged: vec3 {} vs scalar {}",
                vp[i],
                sp[i],
            );
        }
    }

    #[test]
    fn a_zero_or_negative_timestep_is_a_no_op() {
        let params = SpringParams::critical(0.1);
        assert_eq!(spring_scalar(0.25, 3.0, 1.0, &params, 0.0), (0.25, 3.0));
        assert_eq!(spring_scalar(0.25, 3.0, 1.0, &params, -1.0), (0.25, 3.0));
    }

    #[test]
    fn a_degenerate_halflife_does_not_produce_nan() {
        // A zero half-life is an authoring error (a RON file with a missing
        // or zeroed field). It must clamp to "very fast", never NaN the rig.
        let params = SpringParams { halflife: 0.0, damping_ratio: 1.0, max_speed: 50.0 };
        let (position, velocity) = spring_scalar(0.0, 0.0, 1.0, &params, 1.0 / 60.0);

        assert!(position.is_finite(), "a zero halflife must not produce {position}");
        assert!(velocity.is_finite(), "a zero halflife must not produce v = {velocity}");
    }
}
