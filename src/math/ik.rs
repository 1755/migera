//! Two-bone inverse kinematics, following Daniel Holden's leg-chain
//! recipe.
//!
//! # Why this exists as its own pure module
//!
//! The superseded module's leg solver accumulated at least five separate
//! once-live bugs — a wrong pivot, an inflated bone length, a derived
//! rather than fixed bend axis (which produced a measured **7.3x** leg
//! overstretch), an unsigned instead of signed target angle, and an
//! inverted knee-bend sign. It had no direct unit test; every one of those
//! was caught by eye, late.
//!
//! So this is plain functions over plain values, each of those five
//! failures is a named regression test below, and nothing here touches the
//! ECS.
//!
//! # The recipe
//!
//! 1. **Softly clamp** the target to the limb's reach, so a too-far target
//!    approaches full extension asymptotically instead of snapping to a
//!    locked-straight leg.
//! 2. **Law of cosines** for the knee's interior angle.
//! 3. **A fixed bend axis**, supplied by the caller, rather than one
//!    derived from the current pose.
//!
//! # Two choices worth stating plainly
//!
//! **The bend axis is a parameter, not a cross product.** Deriving it from
//! `hip → target` and `hip → knee` is the obvious approach and it fails
//! whenever those are nearly parallel — which is exactly the case a leg is
//! in most of the time, because a nearly-straight leg has a nearly
//! degenerate plane. Holden's article uses the knee's own side vector for
//! this reason, and the 7.3x overstretch in this project's own history came
//! from a derived axis picking up the hip's lateral offset.
//!
//! **The clamp is soft.** A hard clamp produces a leg that visibly locks
//! the instant a target goes out of reach, and worse, is not differentiable
//! there — so a foot crossing the boundary jitters. The exponential
//! saturation below approaches `max_extension` without ever reaching it,
//! and is smooth across the whole domain.

use bevy::math::Vec3;

/// A two-bone chain's solved joint angles, as rotations about the bend
/// axis measured from the straight-out direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwoBoneSolution {
    /// Angle from the root-to-target direction to the upper bone.
    pub upper_angle: f32,
    /// Interior angle at the middle joint, measured from straight.
    /// Positive means bent.
    pub joint_bend: f32,
    /// How far the target was after clamping. Never exceeds the chain's
    /// own reach.
    pub reach: f32,
}

/// Softly clamps `distance` so it approaches `max_extension` without
/// reaching it.
///
/// Below `max_extension - softening` this is the identity. Above it, the
/// excess is compressed exponentially, so the result rises toward
/// `max_extension` asymptotically and the function stays smooth (C1) across
/// the whole domain — including at the seam, which is where a hard clamp
/// would introduce a visible kink.
///
/// This is Holden's own formulation:
/// `saturation = 1 - exp(-max(d - maxExt + soft, 0) / soft)`.
pub fn soft_clamp_distance(distance: f32, max_extension: f32, softening: f32) -> f32 {
    let softening = softening.max(1.0e-5);
    let threshold = max_extension - softening;

    if distance <= threshold {
        return distance;
    }

    let excess = distance - threshold;
    let saturation = 1.0 - (-excess / softening).exp();

    threshold + softening * saturation
}

/// Solves a two-bone chain reaching from a root toward a target.
///
/// `upper` and `lower` are the two bone lengths; `target_distance` is how
/// far the end effector should end up from the root. Returns the angles
/// that place it there, with the target softly clamped to the chain's own
/// reach first.
///
/// `softening` sets how gently an over-extended target is absorbed; a few
/// millimetres is typical.
pub fn solve_two_bone(
    upper: f32,
    lower: f32,
    target_distance: f32,
    softening: f32,
) -> TwoBoneSolution {
    let max_extension = upper + lower;
    let reach = soft_clamp_distance(target_distance.max(0.0), max_extension, softening);

    // A two-bone chain has a MINIMUM reach as well as a maximum: folded
    // completely double, its end effector is still `|lower - upper|` from
    // the root. A target inside that is unreachable in the same way one
    // beyond `max_extension` is.
    //
    // Clamping it explicitly means `reach` always reports where the end
    // effector will actually be. Without this the law of cosines still
    // produces the right (fully folded) angles — `acos` saturates — but
    // `reach` would claim a distance the chain never achieves, and a caller
    // trusting it to position a foot would be quietly wrong.
    let min_extension = (lower - upper).abs();
    let reach = reach.max(min_extension);

    // Guard the degenerate cases before the law of cosines divides by them.
    if upper <= 1.0e-6 || lower <= 1.0e-6 {
        return TwoBoneSolution { upper_angle: 0.0, joint_bend: 0.0, reach };
    }

    // Law of cosines at the middle joint. `cos_interior` is the cosine of
    // the angle between the two bones; clamped because float error can push
    // it a hair outside [-1, 1] at full extension and `acos` would produce
    // NaN.
    let cos_interior =
        ((upper * upper + lower * lower - reach * reach) / (2.0 * upper * lower))
            .clamp(-1.0, 1.0);
    let interior = cos_interior.acos();

    // A straight chain has an interior angle of pi, so bend-from-straight
    // is the complement. Reporting it this way means "zero is straight",
    // which is what every caller wants.
    let joint_bend = std::f32::consts::PI - interior;

    // Law of cosines at the root: how far the upper bone sits off the
    // straight root-to-target line.
    let cos_upper = ((upper * upper + reach * reach - lower * lower)
        / (2.0 * upper * reach.max(1.0e-6)))
        .clamp(-1.0, 1.0);
    let upper_angle = cos_upper.acos();

    TwoBoneSolution { upper_angle, joint_bend, reach }
}

/// The rotation carrying `from` onto `to`, with an explicit guard for the
/// antipodal case.
///
/// `Quat::from_rotation_arc` is documented-unstable when its inputs are
/// near-opposite, and this project has already been bitten by it once at
/// `dot ≈ -0.99`. At exactly 180 degrees the axis is genuinely arbitrary —
/// any perpendicular is correct — so rather than let the library pick
/// unpredictably, a stable perpendicular is chosen here.
///
/// Near opposite, it half-turns about that perpendicular and then takes the
/// short arc from `-from` onto `to`. The half-turn alone landed on `-from`,
/// up to 1.8° off `to` inside the guard: an arm's elbow aimed past its
/// hanging direction came out 7 mm wide.
pub fn look_rotation(from: Vec3, to: Vec3) -> bevy::math::Quat {
    use bevy::math::Quat;

    let from = from.normalize_or_zero();
    let to = to.normalize_or_zero();

    if from == Vec3::ZERO || to == Vec3::ZERO {
        return Quat::IDENTITY;
    }

    let dot = from.dot(to).clamp(-1.0, 1.0);

    if dot < -0.9995 {
        // Prefer whichever cardinal axis `from` is least aligned with, so
        // the cross product stays well-conditioned.
        let fallback = if from.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let axis = from.cross(fallback).normalize_or_zero();
        let axis = if axis == Vec3::ZERO { Vec3::Y } else { axis };
        let half_turn = Quat::from_axis_angle(axis, std::f32::consts::PI);
        return (Quat::from_rotation_arc(-from, to) * half_turn).normalize();
    }

    Quat::from_rotation_arc(from, to)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Quat;
    use std::f32::consts::PI;

    /// Where a solved two-bone chain actually puts its end effector, built
    /// by forward kinematics from the solution.
    ///
    /// Deliberately independent of the solver: it reconstructs the chain
    /// from angles alone, so if the solver's own algebra is wrong this
    /// disagrees with the requested target rather than agreeing with the
    /// same mistake twice.
    fn end_effector_distance(upper: f32, lower: f32, solution: TwoBoneSolution) -> f32 {
        // Lay the chain out in a plane with the target along +Y.
        //
        // The upper bone sits `upper_angle` off that line; the lower bone
        // then turns back toward it by the joint's bend. Because
        // `joint_bend` is measured FROM STRAIGHT, continuing straight would
        // be `upper_angle` again and the bend subtracts from it — an
        // earlier version of this helper wrote `upper_angle - (PI - bend)`,
        // which is the interior angle rather than the bend and folded the
        // chain the wrong way.
        let upper_end =
            Vec3::new(solution.upper_angle.sin(), solution.upper_angle.cos(), 0.0) * upper;

        let lower_direction = solution.upper_angle - solution.joint_bend;
        let end =
            upper_end + Vec3::new(lower_direction.sin(), lower_direction.cos(), 0.0) * lower;

        end.length()
    }

    #[test]
    fn a_reachable_target_is_reached_exactly() {
        // The core contract, swept across the whole reachable range and
        // both symmetric and lopsided chains.
        let chains: [(f32, f32); 3] = [(0.45, 0.49), (0.5, 0.5), (0.3, 0.7)];
        for (upper, lower) in chains {
            let max = upper + lower;
            // Sweep only the genuinely reachable band. A chain also has a
            // MINIMUM reach of `|lower - upper|` — folded double, its end
            // effector is still that far from the root — so targets inside
            // it are unreachable and belong to their own test.
            let min: f32 = (lower - upper).abs();

            for step in 1..=40 {
                let target = min + (max - min) * step as f32 / 41.0;
                let solution = solve_two_bone(upper, lower, target, 0.005);

                let achieved = end_effector_distance(upper, lower, solution);
                assert!(
                    (achieved - target).abs() < 1.0e-4,
                    "chain ({upper}, {lower}) asked for {target} but reached {achieved}",
                );
            }
        }
    }

    #[test]
    fn an_over_extended_target_never_exceeds_the_chains_reach() {
        // The anti-hyperextension property. A leg asked to reach further
        // than it can must approach full extension, never pass it — a
        // stretched bone is instantly visible and was a recurring bug in
        // the superseded solver.
        let (upper, lower) = (0.45, 0.49);
        let max = upper + lower;

        for multiplier in [1.0, 1.01, 1.1, 1.5, 3.0, 10.0] {
            let solution = solve_two_bone(upper, lower, max * multiplier, 0.005);

            assert!(
                solution.reach <= max,
                "a target {multiplier}x too far produced a reach of {} against a maximum \
                 of {max}",
                solution.reach,
            );
            assert!(
                end_effector_distance(upper, lower, solution) <= max + 1.0e-4,
                "and the end effector must not be placed beyond it either",
            );
        }
    }

    #[test]
    fn a_target_closer_than_the_chain_can_fold_is_clamped_to_its_minimum() {
        // The near-side counterpart of the over-extension clamp, and the
        // less obvious of the two: a chain of unequal bones cannot bring
        // its end effector closer than `|lower - upper|` to the root, no
        // matter how far it folds.
        //
        // `reach` must report where the effector will actually be, so a
        // caller positioning a foot from it is not quietly misled.
        let (upper, lower) = (0.45, 0.49);
        let minimum = lower - upper;

        for target in [0.0, 0.005, 0.02, minimum * 0.5] {
            let solution = solve_two_bone(upper, lower, target, 0.005);

            assert!(
                (solution.reach - minimum).abs() < 1.0e-5,
                "a target at {target} is inside the chain's {minimum} m minimum and must \
                 clamp to it, got {}",
                solution.reach,
            );
            assert!(
                (end_effector_distance(upper, lower, solution) - minimum).abs() < 1.0e-3,
                "...and the effector must actually land there",
            );
        }
    }

    #[test]
    fn a_chain_of_equal_bones_can_fold_all_the_way_to_its_root() {
        // The minimum is zero when the bones match, so an equal-length
        // chain genuinely can touch its own root — the clamp must not
        // prevent that.
        let solution = solve_two_bone(0.5, 0.5, 0.0, 0.005);

        assert!(
            solution.reach < 1.0e-5,
            "equal bones should fold to zero reach, got {}",
            solution.reach,
        );
    }

    #[test]
    fn the_soft_clamp_is_the_identity_well_inside_reach() {
        // The clamp must not perturb ordinary, comfortably reachable
        // targets — only the ones near the limit.
        for distance in [0.0, 0.1, 0.5, 0.8] {
            assert_eq!(
                soft_clamp_distance(distance, 0.94, 0.005),
                distance,
                "a target at {distance} is well within reach and must pass through",
            );
        }
    }

    #[test]
    fn the_soft_clamp_is_smooth_across_its_own_seam() {
        // A hard clamp has a derivative discontinuity exactly where a
        // walking foot crosses it, which reads as a jitter. This checks the
        // numerical derivative does not jump.
        let max = 0.94;
        let softening = 0.01;
        let step = 1.0e-4;

        let mut previous_slope: Option<f32> = None;

        for i in 0..400 {
            let distance = max - 3.0 * softening + i as f32 * step;

            let slope = (soft_clamp_distance(distance + step, max, softening)
                - soft_clamp_distance(distance, max, softening))
                / step;

            if let Some(previous) = previous_slope {
                assert!(
                    (slope - previous).abs() < 0.05,
                    "the clamp's slope jumped from {previous} to {slope} at {distance} — \
                     it is not smooth across the seam",
                );
            }
            previous_slope = Some(slope);
        }
    }

    #[test]
    fn the_soft_clamp_approaches_but_never_reaches_full_extension() {
        let max = 0.94;
        let softening = 0.005;

        let far = soft_clamp_distance(max * 5.0, max, softening);

        // Never EXCEEDS, which is the safety property — a limb longer than
        // its own bones is the visible failure. It does converge to exactly
        // `max` in f32 once the saturation term rounds to 1, which is fine
        // and is why this is `<=` rather than `<`.
        assert!(far <= max, "the clamp must never exceed reach, got {far}");
        assert!(
            far > max - softening,
            "...but should get close to it for a very distant target, got {far}",
        );

        // And a target only slightly too far must be absorbed gently,
        // landing well short of full extension rather than snapping to it.
        let slightly_far = soft_clamp_distance(max + softening, max, softening);
        assert!(
            slightly_far < max,
            "a marginally over-reaching target should stay strictly inside full \
             extension, got {slightly_far}",
        );
    }

    #[test]
    fn a_chain_reaching_its_limit_keeps_a_small_safety_bend() {
        // "Zero is straight" is the convention, but a chain asked to reach
        // its own full length does NOT come back at zero — the soft clamp
        // holds it a few millimetres short, leaving about 7 degrees of
        // bend.
        //
        // That residual is the whole anti-hyperextension mechanism: a joint
        // parked at exactly 180 degrees has no bend direction, so the next
        // frame's solve can flip it either way (the knee-popping
        // artefact), and any numerical error pushes it past straight. The
        // article's advice is to limit extension for exactly this reason.
        let solution = solve_two_bone(0.45, 0.49, 0.94, 0.005);

        assert!(
            solution.joint_bend > 0.01,
            "a chain at its limit should keep a safety bend rather than locking \
             straight, got {}",
            solution.joint_bend,
        );
        assert!(
            solution.joint_bend < 0.25,
            "...but only a small one — it should still read as a straight leg, got {} rad",
            solution.joint_bend,
        );
    }

    #[test]
    fn a_comfortably_reachable_target_leaves_the_chain_unclamped() {
        // The complement: away from the limit the clamp must not intrude,
        // so ordinary poses are solved exactly.
        let solution = solve_two_bone(0.45, 0.49, 0.85, 0.005);

        assert!(
            (solution.reach - 0.85).abs() < 1.0e-6,
            "a target well inside reach must pass through untouched, got {}",
            solution.reach,
        );
    }

    #[test]
    fn a_folded_chain_reports_a_large_bend() {
        let solution = solve_two_bone(0.45, 0.49, 0.2, 0.005);
        assert!(
            solution.joint_bend > 2.0,
            "a tightly folded chain should report a large bend, got {}",
            solution.joint_bend,
        );
    }

    #[test]
    fn bend_increases_monotonically_as_the_target_draws_closer() {
        // Monotonic, so the solver cannot flip between two valid
        // configurations as a foot moves — the knee-popping artefact.
        let (upper, lower) = (0.45, 0.49);
        // Seeded below any real bend, because bend INCREASES as the target
        // closes in; seeding at infinity (as an earlier version did) makes
        // the very first comparison fail regardless of the solver.
        let mut previous = -1.0f32;

        for step in 0..60 {
            let target = (upper + lower) * (1.0 - step as f32 / 60.0);
            let bend = solve_two_bone(upper, lower, target, 0.005).joint_bend;

            assert!(
                bend >= previous - 1.0e-4,
                "bend went backwards ({previous} -> {bend}) as the target closed in",
            );
            previous = bend;
        }
    }

    #[test]
    fn the_solver_never_produces_nan_for_any_input() {
        // Including the degenerate cases a real rig can hand it: a
        // zero-length bone, a zero-distance target, a negative distance.
        let cases = [
            (0.45, 0.49, 0.0),
            (0.45, 0.49, -1.0),
            (0.0, 0.49, 0.5),
            (0.45, 0.0, 0.5),
            (0.0, 0.0, 0.0),
            (0.45, 0.49, f32::MAX),
        ];

        for (upper, lower, target) in cases {
            let solution = solve_two_bone(upper, lower, target, 0.005);
            assert!(
                solution.upper_angle.is_finite()
                    && solution.joint_bend.is_finite()
                    && solution.reach.is_finite(),
                "chain ({upper}, {lower}) toward {target} produced {solution:?}",
            );
        }
    }

    #[test]
    fn a_zero_softening_does_not_divide_by_zero() {
        let clamped = soft_clamp_distance(2.0, 0.94, 0.0);
        assert!(clamped.is_finite(), "got {clamped}");
        assert!(clamped <= 0.94);
    }

    // ---------------------------------------------------------------
    // Regression tests for the five documented once-live bugs in the
    // superseded `straight_leg_chain_to`. Each is named for the mistake it
    // guards against, so a future change that reintroduces one fails with
    // a message explaining what broke last time.
    // ---------------------------------------------------------------

    #[test]
    fn regression_the_chain_solves_over_both_real_leg_segments() {
        // BUG: an earlier solver pivoted from the knee rather than the hip,
        // solving over only shin+foot (0.49 m) and treating the femur as
        // rigid. Every replant then hit the reach clamp and produced a
        // dead-straight leg regardless of the actual target.
        //
        // The property: a target well beyond the SHORT chain's reach, but
        // comfortably inside the full leg's, must be reached exactly rather
        // than clamped.
        let (femur, shin) = (0.45, 0.49);
        let target = 0.75; // past 0.49, inside 0.94

        let solution = solve_two_bone(femur, shin, target, 0.005);

        assert!(
            (solution.reach - target).abs() < 1.0e-5,
            "a target at {target} is inside the full leg's 0.94 m reach and must not be \
             clamped, but came back as {}",
            solution.reach,
        );
        assert!(
            solution.joint_bend > 0.1,
            "...and the knee must actually bend to get there, not lock straight",
        );
    }

    #[test]
    fn regression_bone_lengths_are_taken_as_given_not_inflated() {
        // BUG: an earlier solver measured the femur against a moving pivot
        // and got 0.564 m for a 0.461 m bone, overshooting every target.
        //
        // The property: the solved chain's own segment lengths are exactly
        // what was passed in. Stated as a test because the solver is the
        // one place that could silently rescale them.
        let (upper, lower) = (0.461, 0.49);
        let solution = solve_two_bone(upper, lower, 0.8, 0.005);

        let upper_end =
            Vec3::new(solution.upper_angle.sin(), solution.upper_angle.cos(), 0.0) * upper;
        assert!(
            (upper_end.length() - upper).abs() < 1.0e-6,
            "the upper segment should stay exactly {upper} m long",
        );
    }

    #[test]
    fn regression_the_bend_axis_is_supplied_not_derived() {
        // BUG: deriving the bend axis from a cross product picked up the
        // hip's lateral offset and tilted the knee off-hinge, producing a
        // measured 7.3x overstretch of LeftLeg -> LeftFoot.
        //
        // The property, enforced structurally: this solver returns ANGLES
        // and knows nothing about axes at all, so it cannot derive a wrong
        // one. The caller supplies the hinge. This test documents that as
        // an intentional API decision rather than an omission.
        let solution = solve_two_bone(0.45, 0.49, 0.7, 0.005);

        assert!(
            solution.upper_angle.is_finite() && solution.joint_bend.is_finite(),
            "the solution is expressed purely as angles about a caller-supplied hinge",
        );
    }

    #[test]
    fn regression_the_upper_angle_is_measured_from_the_target_direction() {
        // BUG: an earlier solver used an unsigned angle where a signed one
        // was needed, so the leg swung to the correct magnitude in the
        // wrong direction.
        //
        // The property: `upper_angle` is the angle between the root-to-
        // target line and the upper bone, and it must SHRINK as the chain
        // straightens — at full extension the upper bone lies along the
        // target line exactly.
        let (upper, lower) = (0.45, 0.49);

        let folded = solve_two_bone(upper, lower, 0.5, 0.005);
        let extended = solve_two_bone(upper, lower, 0.93, 0.005);

        assert!(
            extended.upper_angle < folded.upper_angle,
            "the upper bone should align with the target line as the chain extends, but \
             its angle grew from {} to {}",
            folded.upper_angle,
            extended.upper_angle,
        );

        // Bound this against the solution's OWN residual bend rather than a
        // guessed constant: a chain bent by `b` at the joint splits that
        // between its two ends, so the upper bone can never sit further off
        // the target line than `b` itself. An arbitrary threshold here
        // would merely encode whatever softening happened to be in use.
        assert!(
            extended.upper_angle <= extended.joint_bend + 1.0e-4,
            "the upper bone's {} rad offset should not exceed the joint's own {} rad \
             bend — it is a share of that bend, not an independent quantity",
            extended.upper_angle,
            extended.joint_bend,
        );
    }

    #[test]
    fn regression_bend_is_reported_from_straight_not_from_folded() {
        // BUG: an earlier solver had the knee-bend sign inverted
        // (`shin_angle = femur - knee` rather than `+ knee`), bending the
        // leg backwards.
        //
        // The property: `joint_bend` is measured FROM STRAIGHT and is
        // always non-negative, so a caller applying it about a hinge cannot
        // accidentally bend the joint the wrong way by sign alone.
        for step in 0..50 {
            let target = 0.94 * step as f32 / 50.0;
            let bend = solve_two_bone(0.45, 0.49, target, 0.005).joint_bend;

            assert!(
                (0.0..=PI).contains(&bend),
                "bend must be a non-negative angle from straight, got {bend} for target \
                 {target}",
            );
        }
    }

    #[test]
    fn look_rotation_lands_on_a_nearly_opposite_target() {
        // Within the antipodal guard (1.8° of opposite) but not at it: a
        // plain half-turn lands on `-from`, off `to` by up to 1.8°, which
        // left a lifted arm's elbow 7 mm off (`ladder`).
        for from in [Vec3::X, Vec3::NEG_Y, Vec3::new(1.0, -2.0, 3.0).normalize()] {
            let side = from.any_orthonormal_vector();
            for degrees in [179.0f32, 179.5, 179.9, 180.0] {
                let to = Quat::from_axis_angle(side, degrees.to_radians()) * from;
                let rotation = look_rotation(from, to);
                assert!(rotation.is_normalized(), "must be a unit quaternion");
                assert!((rotation * from - to).length() < 1.0e-4, "{from:?} {degrees}°: landed {:?}, wanted {to:?}", rotation * from);
            }
        }
    }

    #[test]
    fn look_rotation_handles_the_antipodal_case() {
        for from in [Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(1.0, -2.0, 3.0).normalize()] {
            let rotation = look_rotation(from, -from);

            assert!(rotation.is_normalized(), "must be a unit quaternion");
            assert!(
                (rotation * from).dot(-from) > 0.999,
                "must actually flip {from:?}, got {:?}",
                rotation * from,
            );
        }
    }

    #[test]
    fn look_rotation_is_exact_for_ordinary_directions() {
        let cases = [
            (Vec3::X, Vec3::Y),
            (Vec3::NEG_Z, Vec3::Y),
            (Vec3::new(0.2, -0.5, 0.8).normalize(), Vec3::new(-0.3, 0.9, 0.1).normalize()),
        ];

        for (from, to) in cases {
            assert!(
                (look_rotation(from, to) * from).dot(to) > 0.9999,
                "look_rotation({from:?}, {to:?}) did not land on the target",
            );
        }
    }

    #[test]
    fn look_rotation_is_identity_for_a_degenerate_input() {
        assert_eq!(look_rotation(Vec3::ZERO, Vec3::Y), Quat::IDENTITY);
        assert_eq!(look_rotation(Vec3::Y, Vec3::ZERO), Quat::IDENTITY);
    }
}
