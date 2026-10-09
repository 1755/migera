//! Teetering at an edge: step 10 of the parkour design, second part.
//! Brought to a stand with its toes at a drop, the walker windmills its
//! arms and rocks its trunk, then settles ([`teeter`]): a layer on the
//! standing pose, the feet kept planted.
//!
//! No data: the arms circle twice, the trunk rocks at about a sway's rate,
//! by eye.

use bevy::math::{Quat, Vec3};

use crate::character::anim::armik::ArmChain;
use crate::character::anim::gait::smoothstep;
use crate::character::anim::rig::{delta_after_world_turn, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// How long a teeter lasts, seconds.
pub const TEETER_TIME: f32 = 1.6;
/// Standing, the ground is looked at this far ahead of the root, metres,
/// and a teeter taken if it is this much lower than where it stands.
pub const TEETER_LOOK: f32 = 0.3;
pub const TEETER_DROP: f32 = 0.5;
/// The arms' circles in a teeter; how far apart from the body they go
/// round, radians.
const CIRCLES: f32 = 2.0;
const ARMS_APART: f32 = 0.35;
/// The trunk's rock back and forth, radians, and its rate, Hz.
const ROCK: f32 = 0.25;
const ROCK_RATE: f32 = 1.25;

/// Whether a body standing at `root` facing `forward` (level, unit) has its
/// toes at a drop: the ground `ground` finds [`TEETER_LOOK`] ahead more than
/// [`TEETER_DROP`] lower than under it (or none).
pub fn at_edge(root: Vec3, forward: Vec3, ground: &dyn Fn(Vec3) -> Option<f32>) -> bool {
    let here = ground(root).unwrap_or(root.y);
    match ground((root + forward * TEETER_LOOK).with_y(here)) {
        Some(ahead) => ahead < here - TEETER_DROP,
        None => true,
    }
}

/// Teetering `t` seconds in ([`TEETER_TIME`] long): both arms circling up
/// in front and down behind, a little out from the body, [`CIRCLES`] times
/// (eased in and out, hanging at either end); the trunk rocking back and
/// forth, faded in and out. The pose untouched at either end.
pub fn teeter(pose: &mut LocalPose, rig: &RigGeometry, t: f32) {
    let s = (t / TEETER_TIME).clamp(0.0, 1.0);
    let envelope = (std::f32::consts::PI * s).sin();
    if s <= 0.0 || s >= 1.0 {
        return;
    }
    let (forward, left) = (rig.forward(), rig.left());
    // About the rig's left, positive tips the up toward the forward.
    let pitch = Vec3::Y.cross(forward).normalize_or(left);
    let rock = ROCK * envelope * (std::f32::consts::TAU * ROCK_RATE * t).sin();
    pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, Quat::from_axis_angle(pitch, rock));
    // Hanging (`-Y`) turned toward the forward first: up in front.
    let circle = Quat::from_axis_angle(Vec3::NEG_Y.cross(forward).normalize_or(left), std::f32::consts::TAU * CIRCLES * smoothstep(s));
    for (chain, sign) in [(ArmChain::LEFT, 1.0f32), (ArmChain::RIGHT, -1.0)] {
        let apart = Quat::from_axis_angle(Vec3::NEG_Y.cross(left * sign).normalize_or(forward), ARMS_APART * envelope);
        pose.rotations[chain.shoulder] = delta_after_world_turn(pose, rig, chain.shoulder, circle * apart);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::{forward_kinematics_on, BoneSet};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// A drop of more than half a metre a toe's length ahead is an edge;
    /// a step down is not, nor ground ahead as high.
    #[test]
    fn an_edge_is_a_drop_just_ahead() {
        let top = |at: Vec3| Some(if at.z > -1.0 { 2.0 } else { 0.0 });
        let forward = Vec3::NEG_Z;
        assert!(at_edge(Vec3::new(0.0, 2.0, -0.8), forward, &top), "0.2 m from a 2 m drop");
        assert!(!at_edge(Vec3::new(0.0, 2.0, -0.5), forward, &top), "0.5 m from it");
        assert!(!at_edge(Vec3::new(0.0, 2.0, -0.8), Vec3::Z, &top), "its back to it");
        let step = |at: Vec3| Some(if at.z > -1.0 { 0.3 } else { 0.0 });
        assert!(!at_edge(Vec3::new(0.0, 0.3, -0.8), forward, &step), "a step down");
    }

    /// A teeter leaves the standing pose as it was at either end, goes
    /// round with the hands over the head, and no joint goes faster than
    /// 14 m/s about the hips nor jumps.
    #[test]
    fn a_teeter_windmills_the_arms_and_settles() {
        let (stood, rig) = real_stood();
        let dt = 1.0 / 60.0;
        let at = |t: f32| {
            let mut pose = stood;
            teeter(&mut pose, &rig, t);
            forward_kinematics_on(&pose, &rig)
        };
        let still = forward_kinematics_on(&stood, &rig);
        for t in [0.0, TEETER_TIME] {
            let then = at(t);
            let off = Bone::ALL.iter().map(|&bone| (then[bone] - still[bone]).length()).fold(0.0, f32::max);
            assert!(off < 1.0e-6, "at {t} s, a joint {off:.6} m off standing");
        }
        let (mut fastest, mut highest, mut kink) = (0.0f32, f32::MIN, 0.0f32);
        let mut frames: Vec<BoneSet<Vec3>> = Vec::new();
        let mut t = 0.0;
        while t <= TEETER_TIME + dt {
            let now = at(t);
            if let Some(last) = frames.last() {
                fastest = fastest.max(Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (last[bone] - last[Bone::Hips])).length() / dt).fold(0.0, f32::max));
            }
            if frames.len() >= 2 {
                let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
                kink = kink.max(Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(0.0, f32::max));
            }
            highest = highest.max(now[ArmChain::LEFT.wrist].y.min(now[ArmChain::RIGHT.wrist].y));
            frames.push(now);
            t += dt;
        }
        eprintln!("fastest {fastest:.2} m/s, kink {kink:.4} m, hands up to {highest:.2} (head {:.2})", still[Bone::Head].y);
        assert!(highest > still[Bone::Head].y, "the hands only up to {highest:.2} m, the head at {:.2}", still[Bone::Head].y);
        assert!(fastest < 14.0, "a joint at {fastest:.1} m/s about the hips");
        assert!(kink < 0.05, "a joint's step changed {kink:.4} m in a frame");
    }
}
