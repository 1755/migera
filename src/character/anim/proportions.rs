//! Body proportions: a rig's segments rescaled to fractions of stature.
//!
//! Winter §4.0.1 (Fig. 4.1, Drillis & Contini 1966) gives every segment as a
//! fraction of standing height `H`. [`winter_factors`] works out, for a rig,
//! how much each segment's child offset must scale to reach those fractions,
//! and how far the hips must rise so the feet still stand where they stood.
//!
//! Applying a factor to a live skinned rig takes two edits (see the KB note
//! "Lengthen a segment by moving its joint and scaling only its
//! skinning"): the child joint's translation scales, and the segment is
//! skinned through a helper joint scaled along it. This module only works
//! out the factors, on [`RigGeometry`], so they can be tested without a
//! mesh.

use bevy::math::{Quat, Vec3};

use super::rig::{forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// Fractions of stature `H`, Winter Fig. 4.1.
pub mod fraction {
    /// Hip joint to knee: 0.530 − 0.285.
    pub const THIGH: f32 = 0.245;
    /// Knee to ankle: 0.285 − 0.039.
    pub const SHANK: f32 = 0.246;
    /// Shoulder joint to elbow.
    pub const UPPER_ARM: f32 = 0.186;
    /// Elbow to wrist.
    pub const FOREARM: f32 = 0.146;
    /// Hip joint height to shoulder joint height: 0.818 − 0.530.
    pub const TRUNK: f32 = 0.288;
    /// Shoulder to shoulder, as the stick figure draws it. Not applied:
    /// with the arm lengths it gives an arm span of 1.14 H against a real
    /// ~1.0 H, so it is a breadth across the deltoids, not joint spacing.
    pub const SHOULDER_WIDTH: f32 = 0.259;
    /// Hip breadth. Not applied: 0.34 m at 1.8 m is the bitrochanteric
    /// (outer) breadth; hip joint centres sit ~0.17 m apart. Set as their
    /// spacing it stood the legs half again too wide.
    pub const HIP_WIDTH: f32 = 0.191;
    /// Hip joint to ankle: the leg as Winter measures it, 0.530 − 0.039.
    pub const LEG: f32 = THIGH + SHANK;
}

/// What [`winter_factors`] works out for a rig.
#[derive(Debug, Clone, PartialEq)]
pub struct Rescale {
    /// The factor each bone's offset from its parent scales by (1 for most).
    pub factors: BoneSet<f32>,
    /// How far the hips rise, metres, world up, so the ankles stay where
    /// they were: the legs' change in height.
    pub hips_rise: f32,
    /// The stature the fractions were taken of, metres.
    pub stature: f32,
}

/// The rig's stature as its legs imply it: hip joint to ankle is
/// [`fraction::LEG`] of `H`.
pub fn stature_from_legs(rig: &RigGeometry) -> f32 {
    let at = forward_kinematics_on(&LocalPose::REST, rig);
    let leg = |hip: Bone, ankle: Bone| at[hip].y - at[ankle].y;
    0.5 * (leg(Bone::LeftUpLeg, Bone::LeftFoot) + leg(Bone::RightUpLeg, Bone::RightFoot)) / fraction::LEG
}

/// The factors that give `rig` Winter's proportions at `stature` metres
/// (or, with `None`, at the stature its legs imply: the legs keep their
/// length, the rest is proportioned to it).
pub fn winter_factors(rig: &RigGeometry, stature: Option<f32>) -> Rescale {
    let h = stature.unwrap_or_else(|| stature_from_legs(rig));
    let mut factors = BoneSet::splat(1.0f32);
    let fk = |factors: &BoneSet<f32>| forward_kinematics_on(&LocalPose::REST, &scaled(rig, factors));
    // Widths keep the rig's own: Winter's are body breadths (see
    // `fraction::HIP_WIDTH`), and the rig's joint spacing is anatomy it
    // was modelled with.

    // The trunk: the spine's offsets together, solved for the hip-to-
    // shoulder height (the shoulders hang from the top of it).
    let trunk_height = |factors: &BoneSet<f32>| {
        let at = fk(factors);
        0.5 * (at[Bone::LeftArm].y + at[Bone::RightArm].y) - 0.5 * (at[Bone::LeftUpLeg].y + at[Bone::RightUpLeg].y)
    };
    // The spine only: scaling the collarbones would change the width.
    let spine = [Bone::Spine, Bone::Spine1, Bone::Spine2];
    let (mut low, mut high) = (0.25f32, 4.0f32);
    for _ in 0..40 {
        let middle = 0.5 * (low + high);
        let mut trial = factors;
        for bone in spine {
            trial[bone] *= middle;
        }
        if trunk_height(&trial) < fraction::TRUNK * h { low = middle } else { high = middle }
    }
    let trunk = 0.5 * (low + high);
    for bone in spine {
        factors[bone] *= trunk;
    }

    // The limbs: each segment its own length, the child joint's offset.
    let at = fk(&factors);
    for (from, to, share) in [
        (Bone::LeftUpLeg, Bone::LeftLeg, fraction::THIGH),
        (Bone::RightUpLeg, Bone::RightLeg, fraction::THIGH),
        (Bone::LeftLeg, Bone::LeftFoot, fraction::SHANK),
        (Bone::RightLeg, Bone::RightFoot, fraction::SHANK),
        (Bone::LeftArm, Bone::LeftForeArm, fraction::UPPER_ARM),
        (Bone::RightArm, Bone::RightForeArm, fraction::UPPER_ARM),
        (Bone::LeftForeArm, Bone::LeftHand, fraction::FOREARM),
        (Bone::RightForeArm, Bone::RightHand, fraction::FOREARM),
    ] {
        factors[to] = share * h / (at[to] - at[from]).length();
    }

    // The hips rise by what the ankles dropped, so the feet stay down.
    let before = forward_kinematics_on(&LocalPose::REST, rig);
    let after = fk(&factors);
    let ankle = |at: &BoneSet<Vec3>| 0.5 * (at[Bone::LeftFoot].y + at[Bone::RightFoot].y);
    Rescale { factors, hips_rise: ankle(&before) - ankle(&after), stature: h }
}

/// `rig` with each bone's offset scaled by `factors`.
pub fn scaled(rig: &RigGeometry, factors: &BoneSet<f32>) -> RigGeometry {
    let mut rig = rig.clone();
    for &bone in Bone::ALL.iter() {
        if bone != Bone::Hips {
            rig.offsets[bone] *= factors[bone];
        }
    }
    rig
}

/// `rig` rescaled as `rescale` says, the hips raised too.
pub fn rescaled(rig: &RigGeometry, rescale: &Rescale) -> RigGeometry {
    let mut rig = scaled(rig, &rescale.factors);
    rig.offsets[Bone::Hips] += rig.root_rotation.inverse() * Vec3::Y * rescale.hips_rise;
    rig
}

/// The turn that carries `+Y` onto `direction`: a helper joint scaling the
/// skin along a segment that does not run along its bone's own `+Y`.
pub fn along(direction: Vec3) -> Quat {
    Quat::from_rotation_arc(Vec3::Y, direction.normalize_or(Vec3::Y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig::puppet_base_as_rendered;

    #[test]
    fn a_rescaled_rig_stands_in_winters_proportions() {
        // Every segment Winter gives, within 1% of its fraction of the
        // stature, on the real rig; and the ankles where they stood.
        let rig = puppet_base_as_rendered();
        for stature in [None, Some(1.6), Some(1.9)] {
            let rescale = winter_factors(&rig, stature);
            let h = rescale.stature;
            let at = forward_kinematics_on(&LocalPose::REST, &rescaled(&rig, &rescale));
            let check = |name: &str, got: f32, share: f32| {
                assert!(
                    (got / (share * h) - 1.0).abs() < 0.01,
                    "{stature:?}: {name} is {got:.3} m, Winter's {:.3}",
                    share * h
                );
            };
            check("thigh", (at[Bone::LeftLeg] - at[Bone::LeftUpLeg]).length(), fraction::THIGH);
            check("shank", (at[Bone::RightFoot] - at[Bone::RightLeg]).length(), fraction::SHANK);
            check("upper arm", (at[Bone::LeftForeArm] - at[Bone::LeftArm]).length(), fraction::UPPER_ARM);
            check("forearm", (at[Bone::RightHand] - at[Bone::RightForeArm]).length(), fraction::FOREARM);
            let before = forward_kinematics_on(&LocalPose::REST, &rig);
            for (a, b) in [(Bone::LeftUpLeg, Bone::RightUpLeg), (Bone::LeftArm, Bone::RightArm)] {
                let width = |at: &BoneSet<Vec3>| (at[a] - at[b]).length();
                assert!((width(&at) - width(&before)).abs() < 1.0e-4, "{stature:?}: {a:?} to {b:?} changed width");
            }
            let trunk = 0.5 * (at[Bone::LeftArm].y + at[Bone::RightArm].y) - 0.5 * (at[Bone::LeftUpLeg].y + at[Bone::RightUpLeg].y);
            check("trunk", trunk, fraction::TRUNK);
            for ankle in [Bone::LeftFoot, Bone::RightFoot] {
                assert!(
                    (at[ankle].y - before[ankle].y).abs() < 1.0e-4,
                    "{stature:?}: the {ankle:?} moved {:.4} m up",
                    at[ankle].y - before[ankle].y
                );
            }
        }
    }

    #[test]
    fn at_its_own_leg_stature_the_legs_keep_their_length() {
        let rig = puppet_base_as_rendered();
        let rescale = winter_factors(&rig, None);
        let leg = |rig: &RigGeometry| {
            let at = forward_kinematics_on(&LocalPose::REST, rig);
            at[Bone::LeftUpLeg].y - at[Bone::LeftFoot].y
        };
        assert!((leg(&rescaled(&rig, &rescale)) - leg(&rig)).abs() < 0.005);
    }

    #[test]
    fn along_carries_up_onto_the_segment() {
        for direction in [Vec3::X, Vec3::NEG_Y, Vec3::new(0.3, 0.9, -0.2)] {
            assert!((along(direction) * Vec3::Y).distance(direction.normalize()) < 1.0e-5);
        }
    }
}
