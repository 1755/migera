//! A turning jump from a stand: step 12 of the parkour steps beyond the
//! first ten (more jumps). A standing jump straight up, handed to a fall as
//! it leaves the floor, which turns the body round about `+Y` in the air
//! (`Falling::spin_round`, as a leap back does) and lands it facing the new
//! way.

use bevy::math::Vec3;

use super::Falling;
use crate::character::anim::jump::{Jump, JumpAsk};
use crate::character::anim::rig::{LocalPose, RigGeometry};

/// A turning jump rises this far, metres of the centre of mass: enough
/// flight (about 0.55 s) to turn half round.
pub const SPIN_HEIGHT: f32 = 0.4;
/// It turns over this long at most, seconds (within the flight).
const SPIN_SECONDS: f32 = 0.5;
/// It is handed to the fall this long after it leaves the floor, seconds:
/// handed over as it left, the knee still straightening fast, a knee's
/// step changed 9.9 cm in a frame (the take-off's own 5).
const HAND_OVER: f32 = 0.1;

/// The standing jump of a turning jump.
pub fn spin_jump(stood: &LocalPose, rig: &RigGeometry) -> Jump {
    Jump::plan(JumpAsk::up(SPIN_HEIGHT), stood, rig)
}

/// Whether a turning jump is handed to its fall now ([`HAND_OVER`] into
/// its flight).
pub fn hands_over(jump: &Jump) -> bool {
    jump.airborne() && jump.elapsed() >= jump.ends(crate::character::anim::jump::JumpPhase::Push) + HAND_OVER
}

/// The fall a turning jump is handed ([`hands_over`]): from the walker's
/// root `root` turned `yaw`, to the ground `ground` high, turning `turn`
/// radians in the air.
#[allow(clippy::too_many_arguments)]
pub fn spin_fall(jump: &Jump, root: Vec3, yaw: f32, ground: f32, drop: f32, turn: f32, stood: &LocalPose, rig: &RigGeometry) -> Falling {
    let mut falling = Falling::from_jump(jump, root, yaw, ground, drop, stood, rig);
    falling.spin_round(turn, SPIN_SECONDS, rig);
    falling
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::{forward_kinematics_on, BoneSet};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
    use crate::character::skeleton::Bone;
    use bevy::math::Quat;

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// Turning a half turn either way, and a quarter: it lands facing the
    /// way turned (within 0.01 rad), on the floor near where it left, every
    /// pose finite, no joint whipping round faster than 12 m/s about the
    /// hips nor its step changing more than the take-off's own.
    #[test]
    fn a_standing_jump_turns_round_in_the_air() {
        let (stood, rig) = real_stood();
        for turn in [std::f32::consts::PI, -std::f32::consts::PI, std::f32::consts::FRAC_PI_2] {
            let mut jump = spin_jump(&stood, &rig);
            let world = |pose: &LocalPose, root: Vec3, yaw: f32| {
                let at = forward_kinematics_on(pose, &rig);
                BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
            };
            let forward = rig.forward();
            let mut frames = Vec::new();
            while !hands_over(&jump) {
                jump.advance(DT);
                frames.push(world(&jump.pose(&stood, &rig), forward * jump.travelled(), 0.0));
            }
            let own = frames.windows(3).map(|w| Bone::ALL.iter().map(|&b| (w[2][b] - 2.0 * w[1][b] + w[0][b]).length()).fold(0.0, f32::max)).fold(0.0, f32::max);
            let mut falling = spin_fall(&jump, forward * jump.travelled(), 0.0, 0.0, 0.0, turn, &stood, &rig);
            let (mut kink, mut fastest) = (0.0f32, 0.0f32);
            while !falling.is_done() {
                falling.advance(DT);
                let pose = falling.pose(&rig);
                assert!(Bone::ALL.iter().all(|&b| pose.rotations[b].is_finite()), "turn {turn}: NaN");
                let now = world(&pose, falling.root(), falling.facing());
                let n = frames.len();
                kink = Bone::ALL.iter().map(|&b| (now[b] - 2.0 * frames[n - 1][b] + frames[n - 2][b]).length()).fold(kink, f32::max);
                fastest = Bone::ALL.iter().map(|&b| ((now[b] - now[Bone::Hips]) - (frames[n - 1][b] - frames[n - 1][Bone::Hips])).length() / DT).fold(fastest, f32::max);
                frames.push(now);
            }
            let turned = crate::character::anim::facing::shortest_angle(falling.facing() - turn);
            eprintln!("turn {turn:.2}: turned off {turned:.4}, kink {kink:.4} (take-off's own {own:.4}), fastest {fastest:.1}, root {:?}", falling.root());
            assert!(turned.abs() < 0.01, "turn {turn}: landed facing {} off", turned);
            assert!(falling.root().y.abs() < 1.0e-3 && falling.root().with_y(0.0).length() < 0.3, "turn {turn}: landed at {:?}", falling.root());
            assert!(kink <= own + 0.01, "turn {turn}: a step changed {kink:.4} m, the take-off's own {own:.4}");
            assert!(fastest < 12.0, "turn {turn}: a joint at {fastest:.1} m/s about the hips");
        }
    }
}
