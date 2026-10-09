//! A precision jump: step 11 of the parkour steps beyond the first ten
//! (running agility), fourth part. From a stand, a jump onto a small top (a
//! post, a beam's end) planned so its centre of mass comes down over it
//! (`Jump::onto`), handed to a fall at its top so the fall lands it there,
//! its hips at rest over the middle (`Falling::from_jump`,
//! `Falling::land_at`); stood on it, it balances with its arms out (the
//! beam's balance).

use bevy::math::{Quat, Vec3};

use super::Falling;
use crate::character::anim::jump::Jump;
use crate::character::anim::rig::{LocalPose, RigGeometry};

/// A standing jump from `root`, facing `yaw`, onto the top whose middle is
/// `top` (the world): `None` out of reach (`Jump::onto`), or not ahead of
/// its facing within [`FACING`].
pub fn jump_onto(top: Vec3, root: Vec3, yaw: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Jump> {
    let forward = Quat::from_rotation_y(yaw) * rig.forward();
    let off = (top - root).with_y(0.0);
    if off.length() < 1.0e-3 || forward.angle_between(off) > FACING {
        return None;
    }
    Jump::onto(off.length(), top.y - root.y, stood, rig)
}

/// A top is jumped onto only within this of the facing, radians.
pub const FACING: f32 = 0.05;

/// Whether a jump onto a top is handed to its fall now: in the air and
/// past its top, coming down. Handed over as it left the floor, the fall
/// landed higher than it left, its landing had no depth to brake over and
/// the legs went NaN.
pub fn hands_over(jump: &Jump) -> bool {
    let t = jump.elapsed();
    jump.airborne() && jump.com_height_at(t + 1.0e-3) <= jump.com_height_at(t)
}

/// The fall a jump onto `top` (its middle, the world) is handed at its
/// top ([`hands_over`]): from the walker's root `root` turned `yaw`,
/// landing on the top's height with its hips come to rest over its middle
/// (the plan's few millimetres of miss taken up over the rest of the
/// flight).
pub fn fall_onto(jump: &Jump, top: Vec3, root: Vec3, yaw: f32, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Falling {
    let mut falling = Falling::from_jump(jump, root, yaw, top.y, drop, stood, rig);
    falling.land_at(top, rig);
    falling
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::anthropometry::centre_of_mass;
    use crate::character::anim::jump::JumpPhase;
    use crate::character::anim::rig::{forward_kinematics_on, BoneSet};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
    use crate::character::skeleton::Bone;

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// From a stand, posts 0.4 m square (their tops 0-0.4 m up, their
    /// middles 0.7-1.8 m ahead, as far as a standing jump reaches at each
    /// height): the jump plans, and the fall it is handed as it leaves
    /// lands both feet on the top (heel to ball), the centre of mass over
    /// its middle once stood, nothing into the post; farther, not planned.
    #[test]
    fn a_post_is_jumped_onto_and_stood_on() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let half = 0.2;
        let mut faults = Vec::new();
        assert!(jump_onto(forward * 1.8 + Vec3::Y * 0.4, Vec3::ZERO, 0.0, &stood, &rig).is_none(), "jumped 1.8 m onto a 0.4 m post");
        for (ahead, rise) in [(1.0f32, 0.0f32), (1.4, 0.0), (1.8, 0.0), (0.8, 0.2), (1.2, 0.2), (1.6, 0.2), (0.7, 0.4)] {
            {
                let name = format!("{ahead} m ahead, {rise} m up");
                let middle = forward * ahead + Vec3::Y * rise;
                let on_post = |p: Vec3| {
                    let off = p - middle;
                    off.x.abs() <= half && off.z.abs() <= half
                };
                let ground = |p: Vec3| Some(if on_post(p) { rise } else { 0.0 });
                let Some(mut jump) = jump_onto(middle, Vec3::ZERO, 0.0, &stood, &rig) else {
                    faults.push(format!("{name}: not planned"));
                    continue;
                };
                let world = |pose: &LocalPose, root: Vec3| {
                    let at = forward_kinematics_on(pose, &rig);
                    BoneSet::from_fn(|bone| root + at[bone])
                };
                // The jump's own step change over its whole length, to
                // compare.
                let mut alone = jump.clone();
                let mut own = Vec::new();
                while !alone.is_done() {
                    alone.advance(DT);
                    own.push(world(&alone.pose(&stood, &rig), forward * alone.travelled()));
                }
                let own_kink = own.windows(3).map(|w| Bone::ALL.iter().map(|&b| (w[2][b] - 2.0 * w[1][b] + w[0][b]).length()).fold(0.0, f32::max)).fold(0.0, f32::max);
                let mut frames = Vec::new();
                while !hands_over(&jump) {
                    jump.advance(DT);
                    frames.push(world(&jump.pose(&stood, &rig), forward * jump.travelled()));
                }
                let handed = frames.len();
                let mut falling = fall_onto(&jump, middle, forward * jump.travelled(), 0.0, 0.0, &stood, &rig);
                let _ = ground;
                let (mut into, mut kink, mut worst) = (0.0f32, 0.0f32, (0usize, Bone::Hips));
                let mut deepest = None;
                // At a 30 fps step too (posed at it live, the legs went NaN
                // as it landed).
                // Every frame finite, at a 30 fps step too (handed over as it
                // left the floor, the legs went NaN: its fall landed higher
                // than it left).
                let mut coarse = falling.clone();
                while !coarse.is_done() {
                    coarse.advance(2.0 * DT);
                    let pose = coarse.pose(&rig);
                    if Bone::ALL.iter().any(|&bone| !pose.rotations[bone].is_finite()) || !coarse.root().is_finite() {
                        faults.push(format!("{name}: NaN at 30 fps, {:?}", coarse.phase()));
                        break;
                    }
                }
                while !falling.is_done() {
                    falling.advance(DT);
                    let now = world(&falling.pose(&rig), falling.root());
                    // The toes no more than 2 cm in (as the slide under a
                    // slab's: the foot IK keeps them out live), the rest none.
                    for bone in Bone::ALL {
                        let p = now[bone];
                        let allowed = if matches!(bone, Bone::LeftToeBase | Bone::RightToeBase) { 0.02 } else { 0.0 };
                        if on_post(p) && rise - p.y - allowed > into {
                            into = rise - p.y - allowed;
                            deepest = Some((bone, falling.phase()));
                        }
                    }
                    if frames.len() >= 2 {
                        let (a, b) = (&frames[frames.len() - 2], &frames[frames.len() - 1]);
                        for bone in Bone::ALL {
                            let k = (now[bone] - 2.0 * b[bone] + a[bone]).length();
                            if k > kink {
                                (kink, worst) = (k, (frames.len() - handed, bone));
                            }
                        }
                    }
                    frames.push(now);
                }
                eprintln!("  kink {kink:.4} {:?} {} frames after the hand-off; the jump's own {own_kink:.4}; deepest {deepest:?}", worst.1, worst.0);
                // The worst is the take-off's own (the hand-off is at it).
                if kink > own_kink + 0.01 {
                    faults.push(format!("{name}: a step changed {kink:.4} m in a frame, the jump's own {own_kink:.4}"));
                }
                // Facing yaw 0: the pose's axes are the world's.
                let end = falling.pose(&rig);
                let at = world(&end, falling.root());
                let com_world = at[Bone::Hips] + centre_of_mass(&end, &rig);
                let feet_on = [Bone::LeftFoot, Bone::RightFoot].iter().all(|&ankle| {
                    let p = at[ankle];
                    [-0.05f32, 0.12].iter().all(|&along| on_post(p + forward * along)) && (falling.ground() - rise).abs() < 1.0e-3
                });
                eprintln!("{name}: distance {:.2}, height {:.2}, into {into:.4}, kink {kink:.4}, feet on {feet_on}, com off {:.3}", jump.distance(), jump.com_height_at(jump.ends(JumpPhase::Flight)), (com_world - middle).with_y(0.0).length());
                if !feet_on || into > 0.005 || !on_post(com_world.with_y(middle.y)) {
                    faults.push(format!("{name}: feet on {feet_on}, into {into:.4}, com {:?}", com_world - middle));
                }
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
