//! Lowering the hips so a leg can reach ground it otherwise cannot.
//!
//! # The problem this solves
//!
//! Leg IK can only place a foot inside the leg's own reach. When a foot's
//! target is further than that — standing with one foot in a dip, or on a
//! slope where the downhill foot has further to go — the soft extension clamp
//! absorbs the difference and the foot simply stops short, hanging above the
//! surface with the leg locked straight.
//!
//! Dropping the hips moves both hip sockets down, which brings a distant
//! target back inside reach. It is the standard fix, and the article this
//! module follows treats it as the second half of leg IK rather than an
//! extra.
//!
//! # "Avoid the dinosaur"
//!
//! The article's own warning, and the reason this is a clamped correction
//! rather than a solve. Lowering the hips until every foot reaches perfectly
//! produces a character permanently crouched — knees bent, hips sunk, walking
//! like a dinosaur — because the correction is driven by the single worst
//! foot and never recovers.
//!
//! So [`PelvisConfig::max_drop`] caps it, and the cap is deliberately tight.
//! Past a few centimetres the right answer is to accept a foot that does not
//! quite reach: a small unreachable gap is far less visible than a whole-body
//! crouch, and the article says as much — prefer a little sliding over
//! deforming the pose.
//!
//! # Why it is measured, not solved
//!
//! The deficit is computed directly: how far the target is beyond the leg's
//! reach, per foot, taking the worst. There is no iteration, because dropping
//! the hips by exactly the deficit brings that foot exactly into reach —
//! moving the socket down by `d` reduces the socket-to-target distance by at
//! most `d`, and by exactly `d` when the target is straight below.
//!
//! The "at most" is why this is still approximate for a target far out to the
//! side, and why [`solve_pelvis_drop`] is happy to be a single pass: the
//! residual is bounded by the cap anyway.

use bevy::math::Vec3;

use super::legik::LegChain;
use super::rig::{forward_kinematics_on, LocalPose, RigGeometry};

/// How far the hips may be lowered, and how quickly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PelvisConfig {
    /// The furthest the hips will ever drop, metres.
    ///
    /// The "avoid the dinosaur" cap. Small on purpose — see the module doc.
    pub max_drop: f32,
    /// How much of the shortfall to take out of the hips at all, 0 to 1.
    ///
    /// Below 1 the legs are left to strain for the remainder, which keeps the
    /// character standing tall at the cost of a foot that does not quite
    /// reach. That trade is usually the right one.
    pub strength: f32,
    /// Ignore a shortfall smaller than this, metres.
    ///
    /// A deadband, NOT a reduction of the leg's usable length — and it has to
    /// be, because **this rig carries a permanent shortfall even on flat
    /// ground**.
    ///
    /// The legs are authored at exactly critical extension: femur 0.42 plus
    /// shin 0.07 is 0.49, and standing on level ground the hip socket sits
    /// 0.49 above an ankle target that the foot's own thickness puts at
    /// y = -0.01, so the chain is asked to span 0.5008. That 0.0108 m
    /// shortfall is a property of the skeleton, present on every surface, and
    /// a correction that responds to it lowers the hips permanently — the
    /// dinosaur, arrived at from the other direction.
    ///
    /// So the deadband sits above that baseline, and only a shortfall the
    /// GROUND introduces moves the pelvis. Smaller ones are left to the leg's
    /// own soft extension clamp, which is what that clamp is for.
    pub reach_margin: f32,
}

impl Default for PelvisConfig {
    fn default() -> Self {
        Self {
            // 6 cm. Enough to rescue a foot on a moderate slope or a shallow
            // step; not enough to read as a crouch.
            max_drop: 0.06,
            strength: 1.0,
            // 2 cm: comfortably above the rig's own 0.0108 m baseline
            // shortfall (see the field's doc), and well below the depth of
            // ground feature worth crouching for.
            reach_margin: 0.02,
        }
    }
}

/// How far the hips should drop for both feet to reach their targets.
///
/// Returns a non-negative distance, already clamped by
/// [`PelvisConfig::max_drop`]. Zero means every target is comfortably in
/// reach and the hips should stay where the animation put them.
///
/// `targets` are the world-space toe targets the leg IK is about to solve
/// for, and `pose` is the animated pose before any of that has happened.
pub fn solve_pelvis_drop(
    pose: &LocalPose,
    rig: &RigGeometry,
    chains: [(LegChain, Vec3); 2],
    config: &PelvisConfig,
) -> f32 {
    let positions = forward_kinematics_on(pose, rig);

    let mut worst = 0.0f32;

    for (chain, toe_target) in chains {
        // The chain the two-bone solver actually works over: socket to ankle,
        // via the knee. The foot beyond the ankle is carried rigidly, so the
        // ankle is what has to be placed.
        //
        // Read from the rig rather than the posed positions, so the reach is
        // the leg's real fixed length and not whatever the current pose
        // happens to span.
        //
        // Note which offsets these are: a bone's offset is measured from its
        // PARENT, so the femur is `offsets[shin]` (socket to knee) and the
        // shin is `offsets[ankle]` (knee to ankle). `offsets[thigh]` is the
        // hips-to-socket step and is not part of the chain at all. Getting
        // this off by one is the LeftUpLeg-is-the-knee trap this rig's naming
        // sets, and it silently halves the reach.
        let femur = rig.offsets[chain.shin].length();
        let shin = rig.offsets[chain.ankle].length();
        let reach = femur + shin;

        // The ankle's target, derived from the toe's exactly as the leg solve
        // derives it — so this measures the same distance the solver will.
        let ankle_to_toe = positions[chain.toe] - positions[chain.ankle];
        let ankle_target = toe_target - ankle_to_toe;

        let needed = (ankle_target - positions[chain.socket]).length();

        worst = worst.max(needed - reach);
    }

    // The deadband gates the correction without scaling it: a dip deep enough
    // to clear the threshold still gets the full drop it needs, not the
    // remainder after subtracting the margin. The margin decides WHETHER to
    // move the pelvis, never HOW FAR.
    if worst <= config.reach_margin.max(0.0) {
        return 0.0;
    }

    (worst * config.strength.clamp(0.0, 1.0)).min(config.max_drop.max(0.0))
}

/// Lowers a pose's root by `drop` metres.
///
/// Separate from [`solve_pelvis_drop`] so the decision and the edit can be
/// tested apart, and so a caller can smooth the drop over time before
/// applying it — a hip height that snaps between frames is its own artefact.
pub fn apply_pelvis_drop(pose: &mut LocalPose, drop: f32) {
    pose.root_translation.y -= drop;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::stance;
    use crate::character::skeleton::Bone;

    fn standing() -> LocalPose {
        stance(&LocalPose::REST)
    }

    fn toe_targets(pose: &LocalPose, rig: &RigGeometry) -> [(LegChain, Vec3); 2] {
        let positions = forward_kinematics_on(pose, rig);
        [
            (LegChain::LEFT, positions[LegChain::LEFT.toe]),
            (LegChain::RIGHT, positions[LegChain::RIGHT.toe]),
        ]
    }

    #[test]
    fn a_reachable_stance_needs_no_drop_at_all() {
        // The control, and the common case: standing normally, the hips must
        // not move. A correction that fires when nothing is wrong would
        // crouch the character permanently.
        let pose = standing();
        let rig = RigGeometry::default();

        let drop = solve_pelvis_drop(&pose, &rig, toe_targets(&pose, &rig), &PelvisConfig::default());

        assert_eq!(drop, 0.0, "a comfortable stance should not lower the hips");
    }

    #[test]
    fn a_foot_beyond_reach_lowers_the_hips() {
        let pose = standing();
        let rig = RigGeometry::default();

        let mut targets = toe_targets(&pose, &rig);
        // Drop the left target 4 cm — a dip the leg cannot quite reach into.
        targets[0].1.y -= 0.04;

        let drop = solve_pelvis_drop(&pose, &rig, targets, &PelvisConfig::default());

        assert!(drop > 0.0, "an out-of-reach foot should lower the hips");
        assert!(
            drop <= PelvisConfig::default().max_drop,
            "...but never past the cap, got {drop}",
        );
    }

    #[test]
    fn the_drop_tracks_how_far_out_of_reach_the_foot_is() {
        let pose = standing();
        let rig = RigGeometry::default();
        let config = PelvisConfig::default();

        let mut previous = -1.0f32;
        for depth in [0.0, 0.01, 0.02, 0.03] {
            let mut targets = toe_targets(&pose, &rig);
            targets[0].1.y -= depth;

            let drop = solve_pelvis_drop(&pose, &rig, targets, &config);
            assert!(
                drop >= previous,
                "a deeper target should not need LESS drop: {previous} -> {drop}",
            );
            previous = drop;
        }
    }

    #[test]
    fn the_drop_is_capped_however_unreachable_the_target() {
        // THE "avoid the dinosaur" property. A target a metre down must not
        // fold the character into a crouch; it should give up and leave the
        // foot short.
        let pose = standing();
        let rig = RigGeometry::default();

        let mut targets = toe_targets(&pose, &rig);
        targets[0].1.y -= 1.0;

        let drop = solve_pelvis_drop(&pose, &rig, targets, &PelvisConfig::default());

        assert!(
            (drop - PelvisConfig::default().max_drop).abs() < 1.0e-6,
            "an absurd target should clamp to exactly the cap, got {drop}",
        );
    }

    #[test]
    fn the_worst_foot_drives_the_drop() {
        // Both feet share one pelvis, so the one in more trouble decides.
        let pose = standing();
        let rig = RigGeometry::default();
        let config = PelvisConfig::default();

        let mut one = toe_targets(&pose, &rig);
        one[0].1.y -= 0.03;

        let mut both = toe_targets(&pose, &rig);
        both[0].1.y -= 0.03;
        both[1].1.y -= 0.01;

        assert_eq!(
            solve_pelvis_drop(&pose, &rig, one, &config),
            solve_pelvis_drop(&pose, &rig, both, &config),
            "a second, less troubled foot should not change the result",
        );
    }

    #[test]
    fn strength_scales_the_correction() {
        let pose = standing();
        let rig = RigGeometry::default();

        let mut targets = toe_targets(&pose, &rig);
        targets[0].1.y -= 0.03;

        let full = solve_pelvis_drop(
            &pose,
            &rig,
            targets,
            &PelvisConfig { strength: 1.0, ..Default::default() },
        );
        let half = solve_pelvis_drop(
            &pose,
            &rig,
            targets,
            &PelvisConfig { strength: 0.5, ..Default::default() },
        );

        assert!(
            (half - full * 0.5).abs() < 1.0e-6,
            "half strength should halve the drop: {full} vs {half}",
        );
    }

    #[test]
    fn a_zero_strength_never_moves_the_hips() {
        let pose = standing();
        let rig = RigGeometry::default();

        let mut targets = toe_targets(&pose, &rig);
        targets[0].1.y -= 0.5;

        assert_eq!(
            solve_pelvis_drop(
                &pose,
                &rig,
                targets,
                &PelvisConfig { strength: 0.0, ..Default::default() },
            ),
            0.0,
        );
    }

    #[test]
    fn the_reach_margin_is_a_deadband_not_a_shorter_leg() {
        // The distinction this field got wrong first time round. As a
        // deadband, a margin LARGER than the shortfall suppresses the drop
        // entirely, while a small one lets it through unscaled — the drop is
        // never inflated by the margin, which is what subtracting it from the
        // leg's length would do.
        let pose = standing();
        let rig = RigGeometry::default();

        let mut targets = toe_targets(&pose, &rig);
        targets[0].1.y -= 0.02;

        let permissive = solve_pelvis_drop(
            &pose,
            &rig,
            targets,
            &PelvisConfig { reach_margin: 0.0, ..Default::default() },
        );
        let suppressed = solve_pelvis_drop(
            &pose,
            &rig,
            targets,
            &PelvisConfig { reach_margin: 0.5, ..Default::default() },
        );

        assert!(permissive > 0.0, "a real shortfall should drop the hips");
        assert_eq!(
            suppressed, 0.0,
            "a margin wider than the shortfall should suppress it entirely",
        );
    }

    #[test]
    fn an_ordinary_stance_sits_just_inside_full_extension() {
        // Documents WHY the margin is a deadband. The authored stance stands
        // at 99.8% of the leg's length, so any margin implemented as "treat
        // the leg as shorter" reports a shortfall on a comfortable stance.
        //
        // Pinned because it is a property of the authored pose, and a future
        // stance with real knee bend would make the deadband unnecessary —
        // better to find out from a failing test than from a crouching
        // character.
        let pose = standing();
        let rig = RigGeometry::default();
        let positions = forward_kinematics_on(&pose, &rig);

        let chain = LegChain::LEFT;
        let reach =
            rig.offsets[chain.shin].length() + rig.offsets[chain.ankle].length();
        let spanned = (positions[chain.ankle] - positions[chain.socket]).length();

        assert!(
            spanned > reach * 0.99,
            "the stance spans {spanned} of a {reach} m leg — if it now has real knee \
             bend, the reach_margin deadband may no longer be needed",
        );
        assert!(spanned <= reach, "and must never exceed it");
    }

    #[test]
    fn applying_a_drop_lowers_every_bone_by_exactly_that_much() {
        let rig = RigGeometry::default();
        let base = standing();
        let before = forward_kinematics_on(&base, &rig);

        let mut dropped = base;
        apply_pelvis_drop(&mut dropped, 0.05);
        let after = forward_kinematics_on(&dropped, &rig);

        for &bone in Bone::ALL.iter() {
            let shift = after[bone] - before[bone];
            assert!(
                (shift - Vec3::new(0.0, -0.05, 0.0)).length() < 1.0e-6,
                "{} shifted by {shift:?} rather than 5 cm down",
                bone.name(),
            );
        }
    }

    #[test]
    fn dropping_the_hips_actually_brings_a_target_into_reach() {
        // The end-to-end property, and the one that would catch a sign error:
        // after applying the drop, the foot that was short must no longer be.
        let rig = RigGeometry::default();
        let base = standing();

        let mut targets = toe_targets(&base, &rig);
        targets[0].1.y -= 0.04;

        let config = PelvisConfig::default();
        let drop = solve_pelvis_drop(&base, &rig, targets, &config);
        assert!(drop > 0.0, "test setup: this target should need a drop");

        let mut dropped = base;
        apply_pelvis_drop(&mut dropped, drop);

        // Measured the same way the solver measures it.
        let after = solve_pelvis_drop(&dropped, &rig, targets, &config);
        assert!(
            after < drop * 0.2,
            "after dropping {drop} m the remaining shortfall should be nearly gone, \
             but is still {after} m",
        );
    }

    #[test]
    fn a_negative_or_absurd_config_is_safe() {
        let pose = standing();
        let rig = RigGeometry::default();

        let mut targets = toe_targets(&pose, &rig);
        targets[0].1.y -= 0.2;

        for config in [
            PelvisConfig { max_drop: -1.0, ..Default::default() },
            PelvisConfig { strength: -1.0, ..Default::default() },
            PelvisConfig { strength: 5.0, ..Default::default() },
            PelvisConfig { reach_margin: -1.0, ..Default::default() },
        ] {
            let drop = solve_pelvis_drop(&pose, &rig, targets, &config);
            assert!(drop.is_finite() && drop >= 0.0, "got {drop} for {config:?}");
        }
    }
}
