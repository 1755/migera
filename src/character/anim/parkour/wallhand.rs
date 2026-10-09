//! A hand on a nearby wall: step 11 of the parkour steps beyond the first
//! ten (running agility), fifth part. Beside a wall within an arm's reach
//! of a shoulder, that side's hand rests on it, palm flat, a little ahead of
//! the shoulder and under it; walking along close, it rests there as the
//! body passes, brushing along. Eased in and out by how near the wall is.
//!
//! No data: the reach and the hand's place by eye.

use bevy::math::{Quat, Vec3};

use crate::character::anim::armik::{frame_turn, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::rig::{accumulate_bind_rotations, forward_kinematics_on, LocalPose, RigGeometry};

/// A wall this far from a shoulder at most, metres (level), its hand on
/// it, fully by this far; nearer than the least, none (the arm folded
/// against it). Fully on 0.1 m nearer, the wrist moved 4.5 cm for 5 mm of
/// the wall's nearing.
pub const REACH: f32 = 0.62;
const FULL: f32 = 0.45;
const NEAREST: f32 = 0.18;
/// The walker eases its hand on and off a wall over this long, seconds.
pub const EASE: f32 = 0.3;
/// The wrist this far ahead of the shoulder, under it, and off the face,
/// metres.
const AHEAD: f32 = 0.12;
const UNDER: f32 = 0.08;
const OFF_FACE: f32 = 0.03;
/// The wall must stand at least this far under the shoulder, metres, to
/// be leant on (a low wall is not).
const WALL_UP_TO: f32 = 0.2;
/// The probe out from a shoulder, metres a step.
const STEP: f32 = 0.02;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const SIGN: [f32; 2] = [1.0, -1.0];

/// A wall beside a body: on which side (0 left, 1 right), how far from
/// that shoulder (level), and its face's way out (level, unit, the world).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Beside {
    pub side: usize,
    pub distance: f32,
    pub out: Vec3,
}

impl Beside {
    /// How far the hand is on it, 0-1.
    pub fn weight(&self) -> f32 {
        if self.distance < NEAREST {
            return 0.0;
        }
        smoothstep(((REACH - self.distance) / (REACH - FULL)).clamp(0.0, 1.0))
    }
}

/// The nearer wall beside a body posed `pose` at `root` turned `yaw`,
/// probed straight out from each shoulder at the height a hand would rest
/// on it: `blocks(point, low)` whether something solid stands at `point`
/// higher than `low` (the walker's ground probe).
pub fn beside(pose: &LocalPose, root: Vec3, yaw: f32, rig: &RigGeometry, blocks: &dyn Fn(Vec3, f32) -> bool) -> Option<Beside> {
    let turn = Quat::from_rotation_y(yaw);
    let at = forward_kinematics_on(pose, rig);
    (0..2)
        .filter_map(|side| {
            let shoulder = root + turn * at[ARMS[side].shoulder];
            let way = (turn * rig.left() * SIGN[side]).with_y(0.0).normalize_or_zero();
            let low = shoulder.y - WALL_UP_TO;
            let hit = (1..=(REACH / STEP).ceil() as usize).map(|k| k as f32 * STEP).find(|&d| blocks(shoulder + way * d, low))?;
            // Narrowed to its face (found a step late, the wrist sat 2 cm
            // nearer the face than asked).
            let (mut clear, mut inside) = (hit - STEP, hit);
            for _ in 0..10 {
                let mid = 0.5 * (clear + inside);
                if blocks(shoulder + way * mid, low) {
                    inside = mid;
                } else {
                    clear = mid;
                }
            }
            Some(Beside { side, distance: inside, out: -way })
        })
        .min_by(|a, b| a.distance.total_cmp(&b.distance))
}

/// `pose` (at `root` turned `yaw`) with the hand on `wall`'s side resting
/// on its face, by `weight` (0-1): the wrist ahead of the shoulder and
/// under it, just off the face, the palm flat on it, the fingers up and a
/// little ahead.
pub fn rest_hand(pose: &mut LocalPose, root: Vec3, yaw: f32, rig: &RigGeometry, wall: &Beside, weight: f32) {
    if weight <= 0.0 {
        return;
    }
    let turn = Quat::from_rotation_y(yaw);
    let back = turn.inverse();
    let chain = ARMS[wall.side];
    let at = forward_kinematics_on(pose, rig);
    let shoulder = root + turn * at[chain.shoulder];
    let forward = (turn * rig.forward()).with_y(0.0).normalize_or_zero();
    let on_face = shoulder - wall.out * (wall.distance - OFF_FACE) + forward * AHEAD - Vec3::Y * UNDER;
    let hanging = root + turn * at[chain.wrist];
    let target = back * (hanging.lerp(on_face, weight) - root);
    // The elbow out and down, away from the wall's face and the body.
    let pole = (back * (Vec3::NEG_Y - forward * 0.3)).normalize();
    let (elbow, wrist) = solve_arm_toward_from(pose, &at, chain, target, pole, rig);
    // The palm flat on the face, the fingers up and a little ahead.
    let rest = forward_kinematics_on(&LocalPose::REST, rig);
    let along = (rest[chain.wrist] - rest[chain.elbow]).normalize_or(Vec3::NEG_Y);
    let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
    let press = frame_turn(along, palm, back * (Vec3::Y + forward * 0.4).normalize(), back * -wall.out);
    let bind = accumulate_bind_rotations(rig)[chain.wrist];
    turn_hand(pose, rig, chain, bind, press, weight, (wrist - elbow).normalize_or_zero());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// A tall wall 0.3-0.55 m to either side of a standing body: found on
    /// that side; the hand rests on its face (the wrist just off it, no
    /// joint through it), palm toward it; a wall farther than an arm, or
    /// one under the shoulders, is not leant on; the hand eases in by
    /// distance with no jump.
    #[test]
    fn a_hand_rests_on_a_wall_beside_it() {
        let (stood, rig) = real_stood();
        let left = rig.left();
        let shoulder_y = forward_kinematics_on(&stood, &rig)[ArmChain::LEFT.shoulder].y;
        for side in [0usize, 1] {
            for distance in [0.3f32, 0.45, 0.55] {
                // The wall's face this far out from the shoulder's line.
                let shoulder_out = forward_kinematics_on(&stood, &rig)[ARMS[side].shoulder].dot(left) * SIGN[side];
                let face = shoulder_out + distance;
                let way = left * SIGN[side];
                let wall_height = 2.5;
                let blocks = |p: Vec3, low: f32| p.dot(way) >= face && wall_height > low;
                let found = beside(&stood, Vec3::ZERO, 0.0, &rig, &blocks).expect("a wall beside");
                assert_eq!(found.side, side);
                assert!((found.distance - distance).abs() <= 1.0e-3, "found it {} m out, not {distance}", found.distance);
                let mut pose = stood;
                rest_hand(&mut pose, Vec3::ZERO, 0.0, &rig, &found, found.weight());
                let at = forward_kinematics_on(&pose, &rig);
                let wrist = at[ARMS[side].wrist];
                let off = face - wrist.dot(way);
                if found.weight() >= 1.0 {
                    assert!((off - OFF_FACE).abs() < 0.01, "side {side}, {distance} m: the wrist {off:.3} m off the face");
                }
                for bone in crate::character::skeleton::Bone::ALL {
                    assert!(at[bone].dot(way) <= face + 1.0e-3, "side {side}, {distance} m: {bone:?} into the wall");
                }
                if found.weight() >= 1.0 {
                    assert!(wrist.y < shoulder_y && wrist.y > shoulder_y - 0.3, "side {side}: the wrist at {:.2}", wrist.y);
                }
            }
        }
        // Out of reach, or a low wall.
        let way = left;
        let far = |p: Vec3, low: f32| p.dot(way) >= 0.9 && 2.5 > low;
        assert!(beside(&stood, Vec3::ZERO, 0.0, &rig, &far).is_none(), "leant on a wall out of reach");
        let low_wall = |p: Vec3, low: f32| p.dot(way) >= 0.4 && 0.8 > low;
        assert!(beside(&stood, Vec3::ZERO, 0.0, &rig, &low_wall).is_none(), "leant on a waist-high wall");
        // Eased in: the wrist moves smoothly as the wall comes nearer.
        let mut last: Option<Vec3> = None;
        let mut most = 0.0f32;
        for k in 0..=60 {
            let distance = REACH + 0.05 - k as f32 * 0.005;
            let shoulder_out = forward_kinematics_on(&stood, &rig)[ArmChain::LEFT.shoulder].dot(left);
            let face = shoulder_out + distance;
            let blocks = |p: Vec3, low: f32| p.dot(left) >= face && 2.5 > low;
            let mut pose = stood;
            if let Some(found) = beside(&stood, Vec3::ZERO, 0.0, &rig, &blocks) {
                rest_hand(&mut pose, Vec3::ZERO, 0.0, &rig, &found, found.weight());
            }
            let wrist = forward_kinematics_on(&pose, &rig)[ArmChain::LEFT.wrist];
            if let Some(last) = last {
                most = most.max((wrist - last).length());
            }
            last = Some(wrist);
        }
        assert!(most < 0.03, "the wrist jumped {most:.3} m for 5 mm of the wall's nearing");
    }
}
