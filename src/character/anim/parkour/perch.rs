//! Perches: step 16 of the parkour steps beyond the first ten. Crouched on
//! a post or a narrow top: a deep squat (the get-up's, `getup::squat`), the
//! feet brought together, the forearms over the knees, the hands hanging
//! in front, the centre of mass over the feet. Eased down into from
//! standing on the top (where a precision jump lands it) and back up, the
//! legs blended by their feet so they stay on it.
//!
//! No perch data: Assassin's Creed's crouched perch by eye.

use bevy::math::{Quat, Vec3};

use crate::character::anim::anthropometry::centre_of_mass;
use crate::character::anim::armik::{solve_arm_toward_from, ArmChain};
use crate::character::anim::getup;
use crate::character::anim::rig::{delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::sitting::legs_by_their_feet;
use crate::character::anim::stance::narrow_feet;
use crate::character::skeleton::Bone;

/// Crouching down into a perch, and up out of it, seconds.
pub const PERCH_EASE: f32 = 0.8;
/// The feet this far apart in a perch, metres (between the ankles).
pub const FEET_APART: f32 = 0.14;
/// Each wrist this far ahead of its knee and under it, metres: the forearm
/// laid over the knee, the hand hanging in front.
const WRIST_AHEAD: f32 = 0.12;
const WRIST_UNDER: f32 = 0.06;
/// The centre of mass over the feet's middle (heel to ball) to within
/// this, metres, by turning the trunk.
const COM_TOLERANCE: f32 = 0.005;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const KNEES: [Bone; 2] = [Bone::LeftLeg, Bone::RightLeg];
const SIGN: [f32; 2] = [1.0, -1.0];

/// The perch, on `rig`, placed so its feet's middle is where `stood`'s is
/// (on the floor at `y = 0`, the character's frame).
pub fn perch_pose(stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
    let forward = rig.forward();
    let mut pose = getup::squat(rig);
    narrow_feet(&mut pose, rig, FEET_APART);
    // The forearms over the knees.
    for side in 0..2 {
        let at = forward_kinematics_on(&pose, rig);
        let wrist = at[KNEES[side]] + forward * WRIST_AHEAD - Vec3::Y * WRIST_UNDER;
        let pole = (rig.left() * SIGN[side] - Vec3::Y * 0.5).normalize();
        solve_arm_toward_from(&mut pose, &at, ARMS[side], wrist, pole, rig);
    }
    // The centre of mass back over the feet's middle (the arms forward over
    // the knees carried it ahead): the trunk turned back at the waist.
    let feet_middle = |pose: &LocalPose| {
        let at = forward_kinematics_on(pose, rig);
        let ends = [Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase].map(|bone| at[bone].dot(forward));
        ends.iter().sum::<f32>() / 4.0
    };
    let off = |pose: &LocalPose| {
        let at = forward_kinematics_on(pose, rig);
        (at[Bone::Hips] + centre_of_mass(pose, rig)).dot(forward) - feet_middle(pose)
    };
    let lean = |pose: &LocalPose, angle: f32| {
        let mut turned = *pose;
        turned.rotations[Bone::Spine] = delta_after_world_turn(&turned, rig, Bone::Spine, Quat::from_axis_angle(rig.left(), angle));
        turned
    };
    let (mut low, mut high) = (-0.8f32, 0.8f32);
    let start = pose;
    for _ in 0..30 {
        let mid = 0.5 * (low + high);
        let o = off(&lean(&start, mid));
        if o.abs() < COM_TOLERANCE {
            break;
        }
        // A positive turn about the left leans forward, carrying the COM
        // ahead.
        if o > 0.0 {
            high = mid;
        } else {
            low = mid;
        }
    }
    pose = lean(&start, 0.5 * (low + high));
    // Where standing's feet are.
    let (at, standing) = (forward_kinematics_on(&pose, rig), forward_kinematics_on(stood, rig));
    let middle = |at: &crate::character::anim::rig::BoneSet<Vec3>| (at[Bone::LeftFoot] + at[Bone::RightFoot]) * 0.5;
    pose.root_translation += (middle(&standing) - middle(&at)).with_y(0.0);
    pose
}

/// `pose` crouched toward `perched` ([`perch_pose`]) by `weight` (0-1),
/// the legs blended by their feet so they stay on the top.
pub fn perch(pose: &mut LocalPose, perched: &LocalPose, rig: &RigGeometry, weight: f32) {
    if weight <= 0.0 {
        return;
    }
    let from = *pose;
    let blended = crate::character::anim::clip::blend(&from, perched, weight);
    *pose = legs_by_their_feet(blended, &from, perched, weight, rig);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gait::smoothstep;
    use crate::character::anim::rig::BoneSet;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// The perch: the feet together on the floor (on a 0.3 m top), the
    /// centre of mass over them, nothing under the floor, the forearms
    /// over the knees, the hips low; crouched into from standing and back,
    /// the feet stay on the floor and nothing jumps.
    #[test]
    fn a_perch_crouches_over_its_feet() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let perched = perch_pose(&stood, &rig);
        let at = forward_kinematics_on(&perched, &rig);
        let standing = forward_kinematics_on(&stood, &rig);
        let apart = (at[Bone::LeftFoot] - at[Bone::RightFoot]).with_y(0.0).length();
        assert!((apart - FEET_APART).abs() < 0.01, "the feet {apart:.3} m apart");
        // On a 0.3 m square top under the standing feet's middle.
        let middle = (standing[Bone::LeftFoot] + standing[Bone::RightFoot]) * 0.5;
        for bone in [Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase] {
            let off = (at[bone] - middle).with_y(0.0);
            assert!(off.x.abs() < 0.15 && off.z.abs() < 0.15, "{bone:?} off a 0.3 m top: {off:?}");
        }
        let com = at[Bone::Hips] + centre_of_mass(&perched, &rig);
        let feet = [Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase].map(|bone| at[bone].dot(forward));
        let (back, front) = (feet.iter().copied().fold(f32::MAX, f32::min), feet.iter().copied().fold(f32::MIN, f32::max));
        assert!((back..=front).contains(&com.dot(forward)), "the COM {:.3} not over the feet {back:.3}-{front:.3}", com.dot(forward));
        assert!(Bone::ALL.iter().all(|&bone| at[bone].y > -0.02), "a joint under the floor");
        assert!(at[Bone::Hips].y < 0.6 * standing[Bone::Hips].y, "the hips {:.2} up, not crouched", at[Bone::Hips].y);
        for side in 0..2 {
            let over = at[ARMS[side].wrist] - at[KNEES[side]];
            assert!(over.dot(forward) > 0.0 && over.length() < 0.25, "side {side}: the wrist {over:?} from the knee");
        }
        // Down and up.
        let (mut frames, mut lowest): (Vec<BoneSet<Vec3>>, f32) = (Vec::new(), f32::MAX);
        let n = (2.0 * PERCH_EASE * 60.0) as usize;
        for k in 0..=n {
            let u = k as f32 / n as f32;
            let w = smoothstep(if u < 0.5 { 2.0 * u } else { 2.0 - 2.0 * u });
            let mut pose = stood;
            perch(&mut pose, &perched, &rig, w);
            let now = forward_kinematics_on(&pose, &rig);
            lowest = lowest.min([Bone::LeftFoot, Bone::RightFoot].iter().map(|&b| now[b].y).fold(f32::MAX, f32::min));
            frames.push(now);
        }
        let kink = frames.windows(3).map(|w| Bone::ALL.iter().map(|&b| (w[2][b] - 2.0 * w[1][b] + w[0][b]).length()).fold(0.0, f32::max)).fold(0.0, f32::max);
        let ankle = standing[Bone::LeftFoot].y;
        assert!(lowest > ankle - 0.01, "an ankle {lowest:.3} m up, standing's {ankle:.3}");
        assert!(kink < 0.01, "a joint's step changed {kink:.4} m in a frame");
    }
}
