//! Leaning with a run's acceleration: step 11 of the parkour steps beyond
//! the first ten (running agility). A body whose speed or way changes leans
//! by its acceleration, `tan θ = a/g`: into a turn by `v²/r = v·ω`, forward
//! gathering speed (a sprint's start), back shedding it.
//!
//! Into a turn the whole body rolls, about its outer ankle, the feet kept
//! where the gait put them: the outer leg keeps its reach, the inner one
//! bends. Forward and back only the trunk pitches, over the hips:
//! pitched whole over a steady run's stance, the trailing foot at toe-off
//! is out of reach (at 11.5° the hips must sink 9 cm, or the foot comes
//! 7 cm short and slides).

use bevy::math::{Quat, Vec2, Vec3};

use crate::character::anim::jump::GRAVITY;
use crate::character::anim::math::spring::{spring_scalar, SpringParams};
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// The most a body rolls into a turn, and pitches forward or back, radians
/// (about 31° and 26°).
pub const MOST_ROLL: f32 = 0.55;
pub const MOST_PITCH: f32 = 0.45;
/// The lean follows what its acceleration asks with this half-life,
/// seconds: a turn starts at the facing's full rate, and taken at once its
/// roll moved the hips 5 cm in a frame; at 0.08 s, a joint's step changed
/// 1.3 cm in a frame.
pub const LEAN_HALFLIFE: f32 = 0.12;

const FEET: [Bone; 2] = [Bone::LeftFoot, Bone::RightFoot];

/// The lean asked by an acceleration `forward` (gathering speed +) and
/// `toward_left` (a turn's, toward the body's left +), m/s²: `x` the trunk's
/// pitch forward, `y` the body's roll toward its left, radians.
pub fn lean_for(forward: f32, toward_left: f32) -> Vec2 {
    Vec2::new((forward / GRAVITY).atan().clamp(-MOST_PITCH, MOST_PITCH), (toward_left / GRAVITY).atan().clamp(-MOST_ROLL, MOST_ROLL))
}

/// The acceleration toward the left of a body going `speed` m/s and
/// turning `turn_rate` rad/s about `+Y` (a positive turn is to its left,
/// whichever way the rig faces).
pub fn turning(speed: f32, turn_rate: f32) -> f32 {
    speed * turn_rate
}

/// A walker's lean as it goes, sprung toward what its acceleration asks.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Lean {
    /// The pitch forward and roll toward the left now, radians.
    pub now: Vec2,
    velocity: Vec2,
}

impl Lean {
    /// Moves on `dt` seconds toward `wanted` ([`lean_for`]).
    pub fn advance(&mut self, wanted: Vec2, dt: f32) {
        let params = SpringParams::critical(LEAN_HALFLIFE);
        let (x, vx) = spring_scalar(self.now.x, self.velocity.x, wanted.x, &params, dt);
        let (y, vy) = spring_scalar(self.now.y, self.velocity.y, wanted.y, &params, dt);
        (self.now, self.velocity) = (Vec2::new(x, y), Vec2::new(vx, vy));
    }

    /// Back upright at once (a move taking over the pose).
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Leans `pose` by `lean` ([`Lean::now`]): rolled whole toward its left by
/// `lean.y` about its outer ankle, each foot kept where it is and as it is;
/// the trunk pitched forward over the hips by `lean.x`.
pub fn lean(pose: &mut LocalPose, rig: &RigGeometry, lean: Vec2) {
    if lean.y.abs() > 1.0e-6 {
        let at = forward_kinematics_on(pose, rig);
        let worlds = accumulate_world_rotations(pose, rig);
        let attitudes = FEET.map(|foot| worlds[foot]);
        let ankles = FEET.map(|foot| at[foot]);
        let left = rig.left();
        let hips = at[Bone::Hips];
        // About a line through the outer ankle, away from the turn (the
        // right leaning left), so its leg keeps its length exactly: about the
        // ground under it, a straight outer leg came 1 mm short.
        let outer = ankles[if lean.y > 0.0 { 1 } else { 0 }];
        let pivot = hips + left * (outer - hips).dot(left) + Vec3::Y * (outer.y - hips.y);
        let turn = Quat::from_rotation_arc(Vec3::Y, Vec3::Y * lean.y.cos() + left * lean.y.sin());
        let moved = pivot + turn * (hips - pivot);
        pose.root_translation += moved - hips;
        pose.rotations[Bone::Hips] = delta_after_world_turn(pose, rig, Bone::Hips, turn);
        for (side, foot) in FEET.into_iter().enumerate() {
            place_ankle(pose, rig, foot, ankles[side] - moved);
            let now = accumulate_world_rotations(pose, rig)[foot];
            pose.rotations[foot] = delta_after_world_turn(pose, rig, foot, attitudes[side] * now.inverse());
        }
    }
    if lean.x.abs() > 1.0e-6 {
        // About the rig's left, a positive turn leans it forward.
        pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, Quat::from_axis_angle(rig.left(), lean.x));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gait::{leg_length_of, walk_pose_on, GaitParams};
    use crate::character::anim::poses;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// A run's poses through a stride at `speed`.
    fn run_poses(speed: f32, stood: &LocalPose, rig: &RigGeometry) -> Vec<LocalPose> {
        let params = GaitParams::running_for(speed, leg_length_of(rig));
        (0..40).map(|i| walk_pose_on(i as f32 / 40.0, &params, stood, rig)).collect()
    }

    /// The angle of `axis` (a direction) in the plane across `normal`, from
    /// `+Y`, signed toward `toward`.
    fn tilt(axis: Vec3, normal: Vec3, toward: Vec3) -> f32 {
        let flat = axis - normal * axis.dot(normal);
        flat.dot(toward).atan2(flat.dot(Vec3::Y))
    }

    /// Rolled into a turn through a whole stride of a run: the trunk tilts
    /// toward the turn by exactly the lean asked (within 1°), the hips move
    /// toward it, and each foot stays where it was (1 mm) and as it was.
    #[test]
    fn a_turn_rolls_the_whole_body_over_the_outer_foot() {
        let (stood, rig) = real_stood();
        let (forward, left) = (rig.forward(), rig.left());
        for speed in [3.0, 4.5] {
            for roll in [0.2, -0.35, MOST_ROLL] {
                for pose in run_poses(speed, &stood, &rig) {
                    let (before, mut leant) = (forward_kinematics_on(&pose, &rig), pose);
                    lean(&mut leant, &rig, Vec2::new(0.0, roll));
                    let after = forward_kinematics_on(&leant, &rig);
                    let trunk = |at: &crate::character::anim::rig::BoneSet<Vec3>| (at[Bone::Neck] - at[Bone::Hips]).normalize();
                    let tilted = tilt(trunk(&after), forward, left) - tilt(trunk(&before), forward, left);
                    assert!((tilted - roll).abs() < 1.0_f32.to_radians(), "{speed} m/s, roll {roll}: the trunk tilted {tilted}");
                    let toward = (after[Bone::Hips] - before[Bone::Hips]).dot(left) * roll.signum();
                    assert!(toward > 0.1 * roll.abs(), "{speed} m/s, roll {roll}: the hips moved {toward} m toward the turn");
                    let (b, a) = (accumulate_world_rotations(&pose, &rig), accumulate_world_rotations(&leant, &rig));
                    for foot in FEET {
                        let off = (after[foot] - before[foot]).length();
                        assert!(off < 1.0e-3, "{speed} m/s, roll {roll}: {foot:?} moved {off} m");
                        assert!(1.0 - b[foot].dot(a[foot]).abs() < 1.0e-5, "{speed} m/s, roll {roll}: {foot:?} turned");
                    }
                }
            }
        }
    }

    /// Gathering or shedding speed, the trunk pitches by `atan(a/g)` (within
    /// 1°) and nothing below the hips moves.
    #[test]
    fn gathering_speed_pitches_the_trunk_by_its_acceleration() {
        let (stood, rig) = real_stood();
        let (forward, left) = (rig.forward(), rig.left());
        for a in [2.0, -3.0, 4.0] {
            let wanted = lean_for(a, 0.0);
            assert!((wanted.x - (a / GRAVITY).atan()).abs() < 1.0e-6);
            for pose in run_poses(4.0, &stood, &rig) {
                let (before, mut leant) = (forward_kinematics_on(&pose, &rig), pose);
                lean(&mut leant, &rig, wanted);
                let after = forward_kinematics_on(&leant, &rig);
                let trunk = |at: &crate::character::anim::rig::BoneSet<Vec3>| (at[Bone::Neck] - at[Bone::Spine]).normalize();
                let pitched = tilt(trunk(&after), left, forward) - tilt(trunk(&before), left, forward);
                assert!((pitched - wanted.x).abs() < 1.0_f32.to_radians(), "{a} m/s²: the trunk pitched {pitched}, asked {}", wanted.x);
                for bone in [Bone::Hips, Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase] {
                    assert!((after[bone] - before[bone]).length() < 1.0e-6, "{a} m/s²: {bone:?} moved");
                }
            }
        }
    }

    /// A turn begun at once (a run at 4 m/s starting to turn at 1.5 rad/s)
    /// and ended: the lean rises to the turn's without overshooting, and no
    /// joint's step changes over 1 cm in a frame for it.
    #[test]
    fn a_lean_eases_into_and_out_of_a_turn() {
        let (stood, rig) = real_stood();
        let dt = 1.0 / 60.0;
        let wanted = lean_for(0.0, turning(4.0, 1.5));
        let mut now = Lean::default();
        let mut most = 0.0_f32;
        let mut frames = Vec::new();
        let poses = run_poses(4.0, &stood, &rig);
        for frame in 0..120 {
            now.advance(if frame < 60 { wanted } else { Vec2::ZERO }, dt);
            most = most.max(now.now.y);
            // The lean alone, on a still pose: what it adds to a frame's step.
            let mut pose = poses[0];
            lean(&mut pose, &rig, now.now);
            frames.push(forward_kinematics_on(&pose, &rig));
            if frame == 59 {
                assert!((now.now.y - wanted.y).abs() < 0.03 * wanted.y, "the lean reached only {} of {}", now.now.y, wanted.y);
            }
        }
        assert!(most <= wanted.y + 1.0e-5, "the lean overshot to {most} of {}", wanted.y);
        assert!(now.now.y.abs() < 0.03 * wanted.y, "the lean is still {} after the turn", now.now.y);
        let kink = frames.windows(3).map(|w| Bone::ALL.iter().map(|&b| (w[2][b] - 2.0 * w[1][b] + w[0][b]).length()).fold(0.0, f32::max)).fold(0.0, f32::max);
        eprintln!("lean {:.3} of {:.3}, kink {kink:.4}", most, wanted.y);
        assert!(kink < 0.01, "a joint's step changed {kink} m in a frame");
    }
}
