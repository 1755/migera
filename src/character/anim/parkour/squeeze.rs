//! Squeezing sideways through a narrow gap: step 10 of the parkour design,
//! fifth part. Asked to squeeze along a passage ([`Squeeze`], `from` to
//! `to`), the walker walks to its mouth and turns square to it, then side-
//! shuffles along it, the arms held flat at its sides ([`flatten`]) and the
//! head turned to its way, and walks on once through.
//!
//! People turn their shoulders into a gap narrower than about 1.3 shoulder
//! widths (Warren and Whang 1987, [`TURN_SIDEWAYS`]); a sideways shuffle
//! while squeezing has no data, so its pace is a careful side-step's.

use bevy::math::{Quat, Vec3};

use crate::character::anim::armik::ArmChain;
use crate::character::anim::rig::{delta_after_world_turn, LocalPose, RigGeometry};

/// A gap narrower than this many shoulder widths is squeezed through
/// sideways.
pub const TURN_SIDEWAYS: f32 = 1.3;
/// The shuffle's pace along it, m/s.
pub const SQUEEZE_SPEED: f32 = 0.35;
/// Through once this near its far end (along it), metres.
const THROUGH: f32 = 0.08;
/// The arms held in flat, radians toward the body from hanging, and back
/// behind it.
const ARMS_IN: f32 = 0.12;
const ARMS_BACK: f32 = 0.15;

/// A passage to squeeze along, its line from `from` to `to` (level).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Squeeze {
    pub from: Vec3,
    pub to: Vec3,
}

impl Squeeze {
    /// Along it, from `from` to `to`: level, unit.
    pub fn along(&self) -> Vec3 {
        (self.to - self.from).with_y(0.0).normalize_or(Vec3::NEG_Z)
    }

    /// Whether a gap `width` across is squeezed through by a body whose
    /// shoulders are `shoulders` across.
    pub fn is_tight(width: f32, shoulders: f32) -> bool {
        width < TURN_SIDEWAYS * shoulders
    }

    /// The way to face through it: square across it, the side nearer
    /// `forward` (level).
    pub fn facing_for(&self, forward: Vec3) -> Vec3 {
        let across = self.along().cross(Vec3::Y);
        if across.dot(forward) >= 0.0 { across } else { -across }
    }

    /// Which way to shuffle facing `facing` (level, unit) on `rig` turned to
    /// it: `+1` toward its left, `-1` toward its right.
    pub fn side(&self, facing: Vec3, rig: &RigGeometry) -> f32 {
        let turn = Quat::from_rotation_arc(rig.forward(), facing);
        if (turn * rig.left()).dot(self.along()) >= 0.0 { 1.0 } else { -1.0 }
    }

    /// Whether a root at `root` is through: within [`THROUGH`] of its far
    /// end, along it, or past.
    pub fn is_through(&self, root: Vec3) -> bool {
        (root - self.to).with_y(0.0).dot(self.along()) >= -THROUGH
    }
}

/// Squeezed, `weight` (0-1) in: the arms held flat at the sides, in toward
/// the body and a little behind it.
pub fn flatten(pose: &mut LocalPose, rig: &RigGeometry, weight: f32) {
    if weight <= 0.0 {
        return;
    }
    let (forward, left) = (rig.forward(), rig.left());
    for (chain, sign) in [(ArmChain::LEFT, 1.0f32), (ArmChain::RIGHT, -1.0)] {
        // In: hanging (`-Y`) toward the other side; back: toward behind.
        let inward = Vec3::NEG_Y.cross(-left * sign).normalize_or(forward);
        let backward = Vec3::NEG_Y.cross(-forward).normalize_or(left);
        let turn = Quat::from_axis_angle(backward, ARMS_BACK * weight) * Quat::from_axis_angle(inward, ARMS_IN * weight);
        pose.rotations[chain.shoulder] = delta_after_world_turn(pose, rig, chain.shoulder, turn);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::forward_kinematics_on;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
    use crate::character::skeleton::Bone;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// Square across the passage, facing the side nearer its way, it
    /// shuffles toward the far end; through once there; a gap under 1.3
    /// shoulder widths is tight.
    #[test]
    fn a_passage_is_faced_across_and_shuffled_along() {
        let (_, rig) = real_stood();
        let squeeze = Squeeze { from: Vec3::new(0.0, 0.0, -1.0), to: Vec3::new(3.0, 0.0, -1.0) };
        for forward in [Vec3::NEG_Z, Vec3::Z, Vec3::new(0.3, 0.0, -1.0).normalize()] {
            let facing = squeeze.facing_for(forward);
            assert!(facing.dot(squeeze.along()).abs() < 1.0e-6 && facing.dot(forward) > 0.0, "facing {facing} for {forward}");
            let side = squeeze.side(facing, &rig);
            let turn = Quat::from_rotation_arc(rig.forward(), facing);
            assert!((turn * rig.left() * side).dot(squeeze.along()) > 0.99, "shuffling the wrong way facing {facing}");
        }
        assert!(!squeeze.is_through(Vec3::new(2.5, 0.0, -1.0)) && squeeze.is_through(Vec3::new(2.95, 0.0, -1.0)));
        assert!(Squeeze::is_tight(0.5, 0.45) && !Squeeze::is_tight(0.7, 0.45));
    }

    /// Flattened, the hands are in at the hips' sides, no farther out than
    /// the shoulders, and behind where they hang.
    #[test]
    fn squeezed_the_arms_are_held_flat() {
        let (stood, rig) = real_stood();
        let (left, forward) = (rig.left(), rig.forward());
        let mut pose = stood;
        flatten(&mut pose, &rig, 1.0);
        let (now, then) = (forward_kinematics_on(&pose, &rig), forward_kinematics_on(&stood, &rig));
        for (chain, sign) in [(ArmChain::LEFT, 1.0f32), (ArmChain::RIGHT, -1.0)] {
            let out = |at: &crate::character::anim::rig::BoneSet<Vec3>| sign * (at[chain.wrist] - at[Bone::Hips]).dot(left);
            assert!(out(&now) < out(&then), "the {sign} hand not in");
            assert!(out(&now) <= sign * (now[chain.shoulder] - now[Bone::Hips]).dot(left) + 0.02, "the {sign} hand out past its shoulder");
            assert!((now[chain.wrist] - then[chain.wrist]).dot(forward) < -0.03, "the {sign} hand not back");
        }
    }
}
