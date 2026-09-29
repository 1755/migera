//! Quaternion PD control — the torque that drives a simulated body toward
//! an orientation.
//!
//! # Why this is not avian's motor
//!
//! avian 0.7 has `AngularMotor`, but only `RevoluteJoint` and
//! `PrismaticJoint` carry one, and its `target_position` is a **scalar**
//! angle. A humanoid joint needs three degrees of freedom, and
//! `SphericalJoint` — the one joint shaped like a shoulder or a hip — has
//! no motor at all.
//!
//! The obvious workaround is to decompose each ball joint into a chain of
//! revolutes and motorize those. This project has already tried that and
//! recorded the result: two independently-simulated rotational springs
//! sharing an intermediate body fought each other into a reproducible
//! instability, and the whole position-space `muscle` module exists partly
//! because of it.
//!
//! So: an **unmotorized** `SphericalJoint` supplies the ball constraint,
//! and orientation is driven by one PD controller per joint, applying a
//! single 3-DOF torque. One rotational actuator per joint means there is
//! no second spring to fight with — the failure mode is structurally
//! absent rather than merely tuned away.
//!
//! # The controller
//!
//! ```text
//! torque = kp * error - kd * angular_velocity
//! ```
//!
//! where `error` is the rotation from current to target, expressed as a
//! scaled-angle-axis vector. That form matters: it is a genuine 3-vector in
//! the same space as angular velocity, so the two terms can be combined
//! directly.
//!
//! Every difference is neighbourhooded first. Without it, a target and a
//! current orientation that happen to lie in opposite quaternion
//! hemispheres produce an error pointing the long way round — the joint
//! takes a 359-degree route to a 1-degree correction.

use bevy::math::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use super::quat_ext::{neighborhood, to_scaled_angle_axis};

/// Tuning for one joint's PD controller.
///
/// Expressed as frequency and damping ratio rather than raw gains, for the
/// same reason [`super::spring::SpringParams`] is: `kp`/`kd` interact,
/// span orders of magnitude, and neither has an intuitive unit, while
/// "how fast, how bouncy" is directly meaningful.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PdParams {
    /// How quickly the joint chases its target, in Hz. Higher is stiffer.
    pub frequency_hz: f32,
    /// 1.0 is critically damped. Below overshoots, above is sluggish.
    pub damping_ratio: f32,
    /// The strongest correction this joint can make, in **rad/s²**.
    ///
    /// Deliberately an angular *acceleration* rather than a torque. The
    /// gains above are acceleration-shaped (`omega^2`, `2*zeta*omega`
    /// describe a second-order system directly), so they already account
    /// for inertia; feeding the result to a torque API that divides by
    /// inertia again applies the scaling twice. Measured, that turned a
    /// reasonable-looking 2232 into 6.4e8 rad/s² on a capsule bone — which
    /// reads exactly like an instability and is really a unit error.
    ///
    /// Working in acceleration also makes a value mean the same thing on a
    /// heavy hip and a light wrist, so per-joint numbers express intent
    /// instead of compensating for mass.
    ///
    /// This limit is what makes the ragdoll *active* rather than
    /// kinematic: a blow that exceeds it wins, the joint yields, the
    /// character breaks form — and then recovers as the controller keeps
    /// pulling, with no authored flinch or get-up animation anywhere.
    pub max_torque: f32,
}

impl Default for PdParams {
    fn default() -> Self {
        Self { frequency_hz: 8.0, damping_ratio: 1.0, max_torque: 120.0 }
    }
}

impl PdParams {
    /// Proportional gain, per unit of rotational inertia.
    ///
    /// For a second-order system `x'' + 2ζω x' + ω² x = 0`, the stiffness
    /// is `ω²`.
    pub fn stiffness(&self) -> f32 {
        let omega = std::f32::consts::TAU * self.frequency_hz;
        omega * omega
    }

    /// Derivative gain, per unit of rotational inertia: `2ζω`.
    pub fn damping(&self) -> f32 {
        let omega = std::f32::consts::TAU * self.frequency_hz;
        2.0 * self.damping_ratio * omega
    }

    /// The derivative gain, clamped to what `dt` can integrate stably.
    ///
    /// # Why this clamp exists
    ///
    /// A damping term integrated explicitly is stable only while
    /// `kd * dt < 2`. Past that, the correction applied in one step
    /// overshoots the velocity it was meant to remove, and the "damping"
    /// starts *adding* energy on alternate steps.
    ///
    /// This is not theoretical here. Measured on a jointed two-body chain
    /// at avian's 64 Hz default, driving a body to a target 15 degrees
    /// away:
    ///
    /// | ζ | `kd·dt` | steady-state chatter |
    /// |---|---|---|
    /// | 0.0 | 0.00 | 3.2 rad/s |
    /// | 0.5 | 0.98 | 10.0 rad/s |
    /// | 1.0 | 1.96 | 12.3 rad/s |
    /// | 4.0 | 7.85 | 16.9 rad/s |
    ///
    /// Damping that monotonically *worsens* oscillation is the signature.
    /// The shipping `default_joint_params` sit at `kd·dt` between 1.37 and
    /// 1.96 — close enough to the boundary that the hip visibly overshot
    /// its target and vibrated.
    ///
    /// A free body never shows this, which is why it survived Phase 6's
    /// single-body tests: with nothing to push against, an over-damped
    /// explicit step just decays. Add a constraint that re-excites the
    /// error every step and the loop closes.
    ///
    /// `0.8` rather than the full `2.0` leaves headroom: the bound is
    /// derived for an isolated second-order system, and a joint solver
    /// correcting the same body adds its own stiffness on top.
    pub fn stable_damping(&self, dt: f32) -> f32 {
        const STABILITY_FRACTION: f32 = 0.8;

        let kd = self.damping();
        if dt <= 0.0 {
            return kd;
        }

        kd.min(STABILITY_FRACTION * 2.0 / dt)
    }
}

/// The torque driving `current` toward `target`, clamped to the joint's
/// own limit.
///
/// Both orientations are world-space; `angular_velocity` is the body's
/// current world-space angular velocity in rad/s. The result is a
/// world-space torque, ready for `Forces::apply_torque`.
///
/// The returned value is an *acceleration-shaped* torque: avian's
/// `apply_torque` divides by the body's effective inverse angular inertia,
/// so gains tuned here behave consistently across differently-massed bones
/// rather than needing per-bone retuning.
pub fn pd_torque(
    current: Quat,
    target: Quat,
    angular_velocity: Vec3,
    params: &PdParams,
) -> Vec3 {
    pd_torque_at(current, target, angular_velocity, params, 0.0)
}

/// [`pd_torque`], with the timestep it will be integrated over.
///
/// Prefer this wherever `dt` is known. It clamps the damping gain to what
/// that timestep can integrate without the damping term adding energy —
/// see [`PdParams::stable_damping`] for the measured failure it prevents.
///
/// A `dt` of zero (or less) means "unknown", and leaves the gain
/// unclamped; [`pd_torque`] is exactly that case.
pub fn pd_torque_at(
    current: Quat,
    target: Quat,
    angular_velocity: Vec3,
    params: &PdParams,
    dt: f32,
) -> Vec3 {
    // Same hemisphere BEFORE differencing, or a 1-degree correction can
    // come back as a 359-degree one.
    let target = neighborhood(current, target);

    let error = to_scaled_angle_axis(target * current.inverse());

    let torque = error * params.stiffness() - angular_velocity * params.stable_damping(dt);

    torque.clamp_length_max(params.max_torque.max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    /// Integrates a single rigid body under the controller, so the tests
    /// measure closed-loop behaviour rather than one torque evaluation.
    ///
    /// Deliberately a plain semi-implicit Euler step on a unit-inertia
    /// body: this is about the controller, not about avian's solver, and
    /// keeping the harness trivial means a failure points at the
    /// controller.
    fn simulate(
        mut orientation: Quat,
        target: Quat,
        params: &PdParams,
        dt: f32,
        steps: usize,
    ) -> (Quat, Vec3, f32) {
        let mut velocity = Vec3::ZERO;
        let mut peak_overshoot: f32 = 0.0;

        for _ in 0..steps {
            let torque = pd_torque(orientation, target, velocity, params);

            velocity += torque * dt;
            orientation = Quat::from_scaled_axis(velocity * dt) * orientation;
            orientation = orientation.normalize();

            // How far PAST the target it has swung, if at all.
            let remaining = to_scaled_angle_axis(
                neighborhood(orientation, target) * orientation.inverse(),
            );
            let initial = to_scaled_angle_axis(
                neighborhood(Quat::IDENTITY, target) * Quat::IDENTITY.inverse(),
            );
            if remaining.dot(initial) < 0.0 {
                peak_overshoot = peak_overshoot.max(remaining.length());
            }
        }

        (orientation, velocity, peak_overshoot)
    }

    #[test]
    fn a_body_already_at_its_target_needs_no_torque() {
        let orientation = Quat::from_axis_angle(Vec3::Y, 0.7);
        let torque = pd_torque(orientation, orientation, Vec3::ZERO, &PdParams::default());

        assert!(
            torque.length() < 1.0e-6,
            "a settled joint should be left alone, got {torque:?}",
        );
    }

    #[test]
    fn a_body_converges_on_its_target() {
        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);
        let params = PdParams { max_torque: f32::INFINITY, ..Default::default() };

        let (orientation, velocity, _) =
            simulate(Quat::IDENTITY, target, &params, 1.0 / 240.0, 480);

        let error = 1.0 - orientation.dot(target).abs();
        assert!(
            error < 1.0e-4,
            "the joint should have reached its target, mismatch {error}",
        );
        assert!(
            velocity.length() < 0.05,
            "and should arrive at rest, got {} rad/s",
            velocity.length(),
        );
    }

    #[test]
    fn a_critically_damped_joint_does_not_overshoot() {
        // The property that makes `damping_ratio = 1.0` a safe default:
        // stiffness can be raised without introducing a wobble.
        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);

        for frequency_hz in [2.0, 5.0, 8.0, 15.0] {
            let params = PdParams {
                frequency_hz,
                damping_ratio: 1.0,
                max_torque: f32::INFINITY,
            };

            let (_, _, overshoot) =
                simulate(Quat::IDENTITY, target, &params, 1.0 / 480.0, 960);

            assert!(
                overshoot < 0.05,
                "a critically damped joint at {frequency_hz} Hz overshot by {overshoot} rad",
            );
        }
    }

    #[test]
    fn an_underdamped_joint_overshoots_and_still_settles() {
        // Follow-through, emergent rather than authored — the same dial
        // that makes a heavy limb lag in Stage 1.
        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);
        let params =
            PdParams { frequency_hz: 6.0, damping_ratio: 0.25, max_torque: f32::INFINITY };

        let (orientation, _, overshoot) =
            simulate(Quat::IDENTITY, target, &params, 1.0 / 480.0, 1920);

        assert!(overshoot > 0.02, "an underdamped joint should overshoot, got {overshoot}");
        assert!(
            1.0 - orientation.dot(target).abs() < 1.0e-3,
            "...and must still settle on the target",
        );
    }

    #[test]
    fn the_torque_never_exceeds_its_own_limit() {
        // The cap is what lets an impact win. If it leaked, a strong enough
        // controller would hold the pose through anything and the ragdoll
        // would be kinematic in all but name.
        let params = PdParams { max_torque: 25.0, ..Default::default() };

        for angle in [0.01, 0.5, 1.5, PI * 0.99] {
            for speed in [0.0, 1.0, 50.0] {
                let torque = pd_torque(
                    Quat::IDENTITY,
                    Quat::from_axis_angle(Vec3::Y, angle),
                    Vec3::new(speed, 0.0, 0.0),
                    &params,
                );

                assert!(
                    torque.length() <= params.max_torque + 1.0e-4,
                    "torque {} exceeded the {} N·m limit at angle {angle}, speed {speed}",
                    torque.length(),
                    params.max_torque,
                );
            }
        }
    }

    #[test]
    fn a_negated_target_produces_no_torque() {
        // THE neighbourhooding regression. `q` and `-q` are the same
        // orientation; a controller handed one while sitting at the other
        // must do nothing. Without the guard it would drive a full turn.
        let orientation = Quat::from_axis_angle(Vec3::Y, 1.1);
        let torque = pd_torque(orientation, -orientation, Vec3::ZERO, &PdParams::default());

        assert!(
            torque.length() < 1.0e-4,
            "q and -q name the same orientation, so no torque is needed, got {torque:?}",
        );
    }

    #[test]
    fn the_error_never_takes_the_long_way_round() {
        // The general form: a 170-degree correction must drive through 170
        // degrees, not 190.
        let target = Quat::from_axis_angle(Vec3::Y, 170f32.to_radians());
        let params = PdParams { max_torque: f32::INFINITY, ..Default::default() };

        let torque = pd_torque(Quat::IDENTITY, target, Vec3::ZERO, &params);
        let implied_angle = torque.length() / params.stiffness();

        assert!(
            (implied_angle - 170f32.to_radians()).abs() < 1.0e-3,
            "expected a 170-degree error, got {} degrees",
            implied_angle.to_degrees(),
        );
    }

    #[test]
    fn damping_opposes_motion() {
        // A joint moving toward a target it has already reached must be
        // slowed, not helped along.
        let params = PdParams { max_torque: f32::INFINITY, ..Default::default() };
        let spinning = Vec3::new(0.0, 3.0, 0.0);

        let torque = pd_torque(Quat::IDENTITY, Quat::IDENTITY, spinning, &params);

        assert!(
            torque.dot(spinning) < 0.0,
            "damping should oppose the spin, but {torque:?} agrees with {spinning:?}",
        );
    }

    #[test]
    fn a_stiffer_joint_holds_its_pose_more_firmly() {
        // The authoring contract for `frequency_hz`: it is what separates a
        // braced fighter from a limp one.
        let target = Quat::from_axis_angle(Vec3::Y, 0.5);

        let soft = pd_torque(
            Quat::IDENTITY,
            target,
            Vec3::ZERO,
            &PdParams { frequency_hz: 2.0, max_torque: f32::INFINITY, ..Default::default() },
        );
        let stiff = pd_torque(
            Quat::IDENTITY,
            target,
            Vec3::ZERO,
            &PdParams { frequency_hz: 12.0, max_torque: f32::INFINITY, ..Default::default() },
        );

        assert!(
            stiff.length() > soft.length() * 4.0,
            "a stiffer joint should pull much harder: {} vs {}",
            stiff.length(),
            soft.length(),
        );
    }

    #[test]
    fn a_zero_torque_limit_makes_the_joint_completely_limp() {
        // The other end of the dial: a fully passive ragdoll.
        let params = PdParams { max_torque: 0.0, ..Default::default() };
        let torque = pd_torque(
            Quat::IDENTITY,
            Quat::from_axis_angle(Vec3::Y, 1.0),
            Vec3::ZERO,
            &params,
        );

        assert_eq!(torque, Vec3::ZERO, "a zero limit must produce no torque at all");
    }

    #[test]
    fn the_controller_never_produces_nan() {
        let cases = [
            (Quat::IDENTITY, Quat::IDENTITY, Vec3::ZERO),
            (Quat::IDENTITY, -Quat::IDENTITY, Vec3::ZERO),
            (Quat::IDENTITY, Quat::from_axis_angle(Vec3::Y, PI), Vec3::splat(1000.0)),
        ];

        for (current, target, velocity) in cases {
            let torque = pd_torque(current, target, velocity, &PdParams::default());
            assert!(torque.is_finite(), "got {torque:?} for {current:?} -> {target:?}");
        }
    }

    #[test]
    fn gains_follow_the_documented_second_order_form() {
        let params = PdParams { frequency_hz: 4.0, damping_ratio: 0.8, max_torque: 100.0 };
        let omega = std::f32::consts::TAU * 4.0;

        assert!((params.stiffness() - omega * omega).abs() < 1.0e-3);
        assert!((params.damping() - 2.0 * 0.8 * omega).abs() < 1.0e-3);
    }

    #[test]
    fn the_damping_gain_is_clamped_to_what_the_timestep_can_integrate() {
        // The explicit-integration bound: `kd * dt` must stay below 2, and
        // this clamps to 0.8 of that for headroom.
        let stiff = PdParams { frequency_hz: 40.0, damping_ratio: 2.0, max_torque: 100.0 };

        // At 64 Hz, the raw gain is far over the bound.
        let dt = 1.0 / 64.0;
        assert!(
            stiff.damping() * dt > 2.0,
            "test setup: the raw gain should exceed the stability bound, got {}",
            stiff.damping() * dt,
        );

        let clamped = stiff.stable_damping(dt);
        assert!(
            clamped * dt <= 1.6 + 1.0e-3,
            "the clamped gain should sit at 0.8 of the bound, but kd*dt is {}",
            clamped * dt,
        );
    }

    #[test]
    fn a_gain_the_timestep_can_carry_is_left_alone() {
        // The clamp must not quietly weaken a well-conditioned joint —
        // that would make every gain below the bound behave differently
        // than authored, for no reason.
        let gentle = PdParams { frequency_hz: 4.0, damping_ratio: 0.5, max_torque: 100.0 };
        let dt = 1.0 / 240.0;

        assert!(gentle.damping() * dt < 1.6, "test setup: this gain is already safe");
        assert_eq!(
            gentle.stable_damping(dt),
            gentle.damping(),
            "a gain inside the bound should pass through unchanged",
        );
    }

    #[test]
    fn an_unknown_timestep_leaves_the_gain_unclamped() {
        // `pd_torque` (no dt) has to keep behaving exactly as it did, or
        // every existing caller silently changes.
        let params = PdParams { frequency_hz: 40.0, damping_ratio: 2.0, max_torque: 100.0 };

        assert_eq!(params.stable_damping(0.0), params.damping());
        assert_eq!(params.stable_damping(-1.0), params.damping());
    }

    #[test]
    fn clamping_the_damping_reduces_steady_state_chatter() {
        // The behavioural claim, not just the arithmetic: an over-damped
        // controller integrated explicitly gains energy, and the clamp is
        // what stops it.
        //
        // `simulate` here is the trivial unit-inertia integrator, so this
        // isolates the controller from avian entirely.
        let params = PdParams { frequency_hz: 30.0, damping_ratio: 3.0, max_torque: 1.0e6 };
        let dt = 1.0 / 64.0;
        let target = Quat::from_axis_angle(Vec3::X, 0.3);

        // Unclamped: `kd * dt` is far over 2, so the damping term
        // overshoots the velocity it is removing and the system diverges.
        let mut orientation = Quat::IDENTITY;
        let mut velocity = Vec3::ZERO;
        let mut unclamped_peak = 0.0f32;
        for _ in 0..400 {
            let torque = {
                let t = neighborhood(orientation, target);
                let error = to_scaled_angle_axis(t * orientation.inverse());
                (error * params.stiffness() - velocity * params.damping())
                    .clamp_length_max(params.max_torque)
            };
            velocity += torque * dt;
            orientation =
                (Quat::from_scaled_axis(velocity * dt) * orientation).normalize();
            unclamped_peak = unclamped_peak.max(velocity.length());
        }

        // Clamped, through the real entry point.
        let mut orientation = Quat::IDENTITY;
        let mut velocity = Vec3::ZERO;
        let mut clamped_peak = 0.0f32;
        for _ in 0..400 {
            let torque = pd_torque_at(orientation, target, velocity, &params, dt);
            velocity += torque * dt;
            orientation =
                (Quat::from_scaled_axis(velocity * dt) * orientation).normalize();
            clamped_peak = clamped_peak.max(velocity.length());
        }

        assert!(
            clamped_peak < unclamped_peak * 0.5,
            "clamping should tame the over-damped case, but peaked at {clamped_peak} \
             against the unclamped {unclamped_peak} rad/s",
        );

        // Deliberately NOT asserting the clamped run is quiet in absolute
        // terms. At 30 Hz the STIFFNESS term is `omega^2` ≈ 35,530, which
        // has its own explicit-integration bound (`sqrt(kp) * dt < 2`,
        // i.e. about 0.6 here against a limit of 2) — so this case is
        // unstable through the P term no matter what the D term does.
        //
        // `stable_damping` fixes the damping half of the bound and only
        // that. A joint whose frequency is too high for its timestep still
        // needs a lower frequency or a finer step; the clamp cannot rescue
        // it, and pretending otherwise here would hide a real limitation.
        assert!(
            clamped_peak.is_finite(),
            "the clamped controller should stay finite, got {clamped_peak}",
        );
    }
}
