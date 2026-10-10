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
/// The probe out from a shoulder, metres a step; the two that find the
/// face's way out this far either side of it, radians. A face is beside
/// the body only turned toward it at least this much (the cosine, 60°).
const STEP: f32 = 0.02;
const SPREAD: f32 = 0.2;
const BESIDE: f32 = 0.5;

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
            // How far out along `way` the face is, within `reach`.
            let face_along = |way: Vec3, reach: f32| {
                let hit = (1..=(reach / STEP).ceil() as usize).map(|k| k as f32 * STEP).find(|&d| blocks(shoulder + way * d, low))?;
                // Narrowed to its face (found a step late, the wrist sat
                // 2 cm nearer the face than asked).
                let (mut clear, mut inside) = (hit - STEP, hit);
                for _ in 0..10 {
                    let mid = 0.5 * (clear + inside);
                    if blocks(shoulder + way * mid, low) {
                        inside = mid;
                    } else {
                        clear = mid;
                    }
                }
                Some(inside)
            };
            let hit = face_along(way, REACH)?;
            // The face's own way out, from two probes either side, reaching
            // farther (the one turned from it meets it more slanting): a
            // wall met slanting (walking up to it on a curve) taken as
            // square to the probe put the wrist 11 cm into it.
            let aside = [-SPREAD, SPREAD].map(|angle| {
                let way = Quat::from_rotation_y(angle) * way;
                face_along(way, 2.0 * REACH).map(|d| shoulder + way * d)
            });
            let (out, distance) = match aside {
                [Some(a), Some(b)] => {
                    let along = (b - a).with_y(0.0).normalize_or_zero();
                    let out = Vec3::Y.cross(along).normalize_or(-way);
                    let out = if out.dot(way) > 0.0 { -out } else { out };
                    (out, (shoulder - (shoulder + way * hit)).with_y(0.0).dot(out))
                }
                _ => (-way, hit),
            };
            // A face turned toward the front or the back is ahead or
            // behind, not beside.
            (-out.dot(way) >= BESIDE).then_some(Beside { side, distance, out })
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
    // Ahead along the face (straight ahead, a face met slanting took the
    // wrist on into it).
    let ahead = (forward - wall.out * forward.dot(wall.out)).normalize_or_zero();
    let on_face = shoulder - wall.out * (wall.distance - OFF_FACE) + ahead * AHEAD - Vec3::Y * UNDER;
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

/// `pose` (its frame in the world at `root` turned `turn`) with any hand
/// that ends inside a wall swung out about its shoulder, the whole arm as
/// it is, as little as brings the wrist [`OFF_FACE`] off the face:
/// `blocks(point, low)` as for [`beside`]. A wall ahead is not leant on, and
/// a swinging hand went 8 cm into one as the body turned to face it.
pub fn keep_hands_off(pose: &mut LocalPose, rig: &RigGeometry, root: Vec3, turn: Quat, blocks: &dyn Fn(Vec3, f32) -> bool) {
    let back = turn.inverse();
    for chain in ARMS {
        // Along the arm's own chain (`frame_from` from the root, its first
        // bone not on it): the whole skeleton each frame cost a walker more
        // than the check.
        let along_chain = |bone| crate::character::anim::rig::frame_from(pose, rig, crate::character::skeleton::Bone::Head, bone).0 + pose.root_translation;
        let (shoulder, wrist) = (root + turn * along_chain(chain.shoulder), root + turn * along_chain(chain.wrist));
        let low = wrist.y;
        let level = (wrist - shoulder).with_y(0.0);
        if level.length() < 1.0e-3 {
            continue;
        }
        let way = level.normalize();
        // Held at the face's clearance from as it comes within it (swung
        // out only once in, it went from in to off the face, 3 cm, in a
        // step): looked for a little past the wrist.
        let probe = 3.0 * OFF_FACE;
        if !blocks(wrist + way * probe, low) || blocks(shoulder.with_y(wrist.y), low) {
            continue;
        }
        // The face between them, level with the wrist, and its way out.
        let base = shoulder.with_y(wrist.y);
        let face_along = |way: Vec3, reach: f32| {
            let (mut clear, mut inside) = (0.0, reach);
            if !blocks(base + way * reach, low) {
                return None;
            }
            for _ in 0..12 {
                let mid = 0.5 * (clear + inside);
                if blocks(base + way * mid, low) {
                    inside = mid;
                } else {
                    clear = mid;
                }
            }
            Some(base + way * inside)
        };
        let Some(hit) = face_along(way, level.length() + probe) else { continue };
        let aside = [-SPREAD, SPREAD].map(|angle| face_along(Quat::from_rotation_y(angle) * way, 2.0 * level.length() + 0.3));
        let out = match aside {
            [Some(a), Some(b)] => {
                let out = Vec3::Y.cross((b - a).with_y(0.0).normalize_or_zero()).normalize_or(-way);
                if out.dot(way) > 0.0 { -out } else { out }
            }
            _ => -way,
        };
        if (wrist - hit).dot(out) >= OFF_FACE {
            continue;
        }
        // Swung toward the way out, in the plane of the arm and it: the
        // wrist's distance out goes as `length·cos(angle − toward)`.
        let arm = wrist - shoulder;
        let length = arm.length();
        let along = arm / length;
        let across = (out - along * along.dot(out)).normalize_or_zero();
        if across == Vec3::ZERO {
            continue;
        }
        let toward = across.dot(out).atan2(along.dot(out));
        let wanted = ((hit - shoulder).dot(out) + OFF_FACE) / length;
        if wanted.abs() > 1.0 {
            continue;
        }
        let angle = (toward - wanted.acos()).max(0.0);
        let swing = Quat::from_axis_angle(along.cross(across).normalize(), angle);
        pose.rotations[chain.shoulder] = crate::character::anim::rig::delta_after_world_turn(pose, rig, chain.shoulder, back * swing * turn);
    }
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

    /// A wall brought in on a standing body's hanging hand, square to the
    /// side, slanting 45° ahead, and ahead: the hand untouched while clear
    /// of the face; once in, swung out to just off it, no joint into the
    /// wall; no jump as the wall comes in 5 mm a step.
    #[test]
    fn a_hand_is_kept_out_of_a_wall() {
        let (stood, rig) = real_stood();
        let (left, forward) = (rig.left(), rig.forward());
        let at = forward_kinematics_on(&stood, &rig);
        let wrist = at[ArmChain::LEFT.wrist];
        for degrees in [0.0f32, 45.0, 80.0] {
            let angle = degrees.to_radians();
            let into = (left * angle.cos() + forward * angle.sin()).normalize();
            let mut last: Option<Vec3> = None;
            let mut jump = 0.0f32;
            for k in 0..=40 {
                // The face from 10 cm past the wrist to 10 cm short of it,
                // no nearer the shoulder than 5 cm (a shoulder in the wall
                // has no way out to swing its arm).
                let face = wrist.dot(into) + 0.1 - k as f32 * 0.005;
                if face < at[ArmChain::LEFT.shoulder].dot(into) + 0.05 {
                    break;
                }
                let blocks = |p: Vec3, low: f32| p.dot(into) >= face && 2.5 > low;
                let mut pose = stood;
                keep_hands_off(&mut pose, &rig, Vec3::ZERO, Quat::IDENTITY, &blocks);
                let now = forward_kinematics_on(&pose, &rig);
                let off = face - now[ArmChain::LEFT.wrist].dot(into);
                if face - wrist.dot(into) >= OFF_FACE + 1.0e-3 {
                    assert!((now[ArmChain::LEFT.wrist] - wrist).length() < 1.0e-6, "{degrees}°: a hand clear of the face moved");
                } else {
                    assert!((off - OFF_FACE).abs() < 2.0e-3, "{degrees}°, the face {:.3} in: the wrist {off:.3} m off it", wrist.dot(into) - face);
                }
                for bone in [ArmChain::LEFT.wrist, ArmChain::LEFT.elbow] {
                    assert!(now[bone].dot(into) <= face, "{degrees}°: {bone:?} {:.3} m into the wall", now[bone].dot(into) - face);
                }
                if let Some(last) = last {
                    jump = jump.max((now[ArmChain::LEFT.wrist] - last).length());
                }
                last = Some(now[ArmChain::LEFT.wrist]);
            }
            assert!(jump < 0.01, "{degrees}°: the wrist jumped {jump:.4} m for 5 mm of the wall");
        }
    }

    /// A wall met slanting, turned 20-45° toward the front from beside
    /// (walking up to a wall on a curve), 0.35 m square from the shoulder:
    /// found square, the wrist just off its face, nothing into it; turned
    /// 70°, 0.15 m off, it is ahead and not leant on. Taken square to the
    /// sideways probe, the wrist went 11 cm in.
    #[test]
    fn a_hand_rests_on_a_wall_met_slanting() {
        let (stood, rig) = real_stood();
        let (left, forward) = (rig.left(), rig.forward());
        let shoulder = forward_kinematics_on(&stood, &rig)[ArmChain::LEFT.shoulder];
        for degrees in [20.0f32, 35.0, 45.0, 70.0] {
            let angle = degrees.to_radians();
            // Into the wall, and its face this far square from the
            // shoulder.
            let into = (left * angle.cos() + forward * angle.sin()).normalize();
            let square = if degrees > 60.0 { 0.15 } else { 0.35 };
            let face = shoulder.with_y(0.0).dot(into) + square;
            let blocks = |p: Vec3, low: f32| p.dot(into) >= face && 2.5 > low;
            let found = beside(&stood, Vec3::ZERO, 0.0, &rig, &blocks);
            if degrees > 60.0 {
                assert!(found.is_none(), "{degrees}°: a wall ahead leant on");
                continue;
            }
            let found = found.expect("a wall beside");
            assert!((found.distance - square).abs() < 0.01 && (found.out + into).length() < 0.01, "{degrees}°: found {found:?}");
            let mut pose = stood;
            rest_hand(&mut pose, Vec3::ZERO, 0.0, &rig, &found, found.weight());
            let at = forward_kinematics_on(&pose, &rig);
            let off = face - at[ArmChain::LEFT.wrist].dot(into);
            assert!((off - OFF_FACE).abs() < 0.01, "{degrees}°: the wrist {off:.3} m off the face");
            for bone in crate::character::skeleton::Bone::ALL {
                assert!(at[bone].dot(into) <= face + 1.0e-3, "{degrees}°: {bone:?} {:.3} m into the wall", at[bone].dot(into) - face);
            }
        }
    }
}
