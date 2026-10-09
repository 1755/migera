//! A leap of faith: step 12 of the parkour steps beyond the first ten
//! (more jumps). From a high top, a swan dive into a soft pile below (hay):
//! it springs off the edge, its centre of mass flying ballistic into the
//! pile, the arms spread and the legs straight; the body pitches forward
//! into the dive, holds it, then turns on over in a half front flip to land
//! on its back on the pile, sinks into it, and after a moment rises out of
//! it through the get-up's keys (sitting up, squatting, standing) on the
//! floor under it.
//!
//! No data: the shapes and times by eye on Assassin's Creed's.

use bevy::math::{Quat, Vec3};

use crate::character::anim::anthropometry::centre_of_mass;
use crate::character::anim::armik::{solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::getup::{self, Lying};
use crate::character::anim::jump::GRAVITY;
use crate::character::anim::rig::{delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::sitting::legs_by_their_feet;
use crate::character::skeleton::Bone;

/// A soft pile to land in: the middle of its top, its radius and its height
/// over the floor under it, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Haystack {
    pub top: Vec3,
    pub radius: f32,
    pub height: f32,
}

impl Haystack {
    /// The floor under it, metres up.
    pub fn floor(&self) -> f32 {
        self.top.y - self.height
    }
}

/// A top this far over the pile's at least, metres.
pub const LEAST_DROP: f32 = 2.5;
/// Springing off, seconds; leaving this fast up, m/s, and no faster than
/// this across.
const TAKEOFF: f32 = 0.35;
const UP: f32 = 1.2;
const MOST_ACROSS: f32 = 4.0;
/// Pitched forward this far leaving, radians; into the dive (face down) by
/// this share of the flight; on its back (on over a half flip) landing.
const LEAN_OFF: f32 = 0.3;
const DIVE: f32 = std::f32::consts::FRAC_PI_2;
const DIVE_AT: f32 = 0.55;
const ON_BACK: f32 = 3.0 * std::f32::consts::FRAC_PI_2;
/// Landing, the centre of mass this far over the pile's top, metres;
/// sunk, this far over the floor under it; sinking no quicker than this,
/// seconds; hidden in it this long.
const LIE_ON: f32 = 0.15;
const SUNK: f32 = 0.2;
const SINK_LEAST: f32 = 0.25;
const HIDDEN: f32 = 0.6;
/// Rising out: into sitting up, the squat, then standing, seconds.
const STAND: f32 = 0.9;
/// The arms spread this far up from level, radians, and this far forward,
/// as a share of the reach.
const ARMS_UP: f32 = 0.2;
const ARMS_AHEAD: f32 = 0.25;
/// The toes pointed this far, radians.
const TOES_POINTED: f32 = 0.6;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const SIGN: [f32; 2] = [1.0, -1.0];

/// A leap of faith under way.
#[derive(Debug, Clone)]
pub struct LeapOfFaith {
    stood: LocalPose,
    rig: RigGeometry,
    yaw: f32,
    hay: Haystack,
    swan: LocalPose,
    /// The centre of mass as it begins, as it leaves and its velocity; the
    /// flight, seconds; sinking, seconds.
    from: Vec3,
    leave: Vec3,
    velocity: Vec3,
    flight: f32,
    sink: f32,
    /// Rising out: each key and how long into it.
    keys: Vec<(LocalPose, f32)>,
    t: f32,
}

/// The swan's shape: the arms spread wide and a little forward and up, the
/// elbows straight, the legs straight, the toes pointed.
fn swan_pose(stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
    let mut pose = *stood;
    for bone in [Bone::LeftUpLeg, Bone::LeftLeg, Bone::RightUpLeg, Bone::RightLeg] {
        pose.rotations[bone] = Quat::IDENTITY;
    }
    for foot in [Bone::LeftFoot, Bone::RightFoot] {
        pose.rotations[foot] = delta_after_world_turn(&pose, rig, foot, Quat::from_axis_angle(rig.left(), TOES_POINTED));
    }
    for side in 0..2 {
        let at = forward_kinematics_on(&pose, rig);
        let chain = ARMS[side];
        let reach = 0.97 * ((at[chain.elbow] - at[chain.shoulder]).length() + (at[chain.wrist] - at[chain.elbow]).length());
        let way = (rig.left() * SIGN[side] + Vec3::Y * ARMS_UP + rig.forward() * ARMS_AHEAD).normalize();
        let pole = (-rig.forward() - Vec3::Y * 0.5).normalize();
        solve_arm_toward_from(&mut pose, &at, chain, at[chain.shoulder] + way * reach, pole, rig);
    }
    pose
}

/// `pose`'s centre of mass in its frame.
fn com(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
    forward_kinematics_on(pose, rig)[Bone::Hips] + centre_of_mass(pose, rig)
}

/// `pose` pitched whole by `pitch` about the rig's left (forward +), about
/// its hips.
fn pitched(pose: &LocalPose, rig: &RigGeometry, pitch: f32) -> LocalPose {
    let mut turned = *pose;
    turned.rotations[Bone::Hips] = delta_after_world_turn(&turned, rig, Bone::Hips, Quat::from_axis_angle(rig.left(), pitch));
    turned
}

/// The cubic from `a` at rate `va` to `b` at rate `vb` over `seconds`, at
/// `s` (0-1): where, and how fast.
fn hermite(a: Vec3, b: Vec3, va: Vec3, vb: Vec3, seconds: f32, s: f32) -> Vec3 {
    let (s2, s3) = (s * s, s * s * s);
    a * (2.0 * s3 - 3.0 * s2 + 1.0) + va * seconds * (s3 - 2.0 * s2 + s) + b * (-2.0 * s3 + 3.0 * s2) + vb * seconds * (s3 - s2)
}

impl LeapOfFaith {
    /// A leap of faith from standing at `root` facing `yaw` (on a top's
    /// edge) into `hay`: `None` if the pile is not [`LEAST_DROP`] below,
    /// not ahead, or out of a spring's reach.
    pub fn plan(root: Vec3, yaw: f32, hay: Haystack, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        if root.y - hay.top.y < LEAST_DROP {
            return None;
        }
        let turn = Quat::from_rotation_y(yaw);
        let forward = turn * rig.forward();
        if (hay.top - root).with_y(0.0).dot(forward) <= 0.0 {
            return None;
        }
        let from = root + turn * com(stood, rig);
        let leave = from + forward * 0.15 + Vec3::Y * 0.05;
        let landing = hay.top + Vec3::Y * LIE_ON;
        // Down from leaving at `UP` to the landing height.
        let fall = leave.y - landing.y;
        let flight = (UP + (UP * UP + 2.0 * GRAVITY * fall).sqrt()) / GRAVITY;
        let across = (landing - leave).with_y(0.0) / flight;
        if across.length() > MOST_ACROSS {
            return None;
        }
        let velocity = across + Vec3::Y * UP;
        let land_speed = (UP - GRAVITY * flight).abs();
        let sink_depth = landing.y - (hay.floor() + SUNK);
        let sink = (2.0 * sink_depth / land_speed.max(1.0e-3)).max(SINK_LEAST);
        let rise = getup::keys(Lying::FaceUp, rig);
        let keys = rise.iter().map(|key| (key.pose, key.seconds)).chain(std::iter::once((*stood, STAND))).collect();
        Some(Self { stood: *stood, rig: rig.clone(), yaw, hay, swan: swan_pose(stood, rig), from, leave, velocity, flight, sink, keys, t: 0.0 })
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt).min(self.end());
    }

    /// When it is in the pile, seconds: landing, sunk, risen.
    fn landed(&self) -> f32 {
        TAKEOFF + self.flight
    }
    fn sunk(&self) -> f32 {
        self.landed() + self.sink
    }
    fn rises(&self) -> f32 {
        self.sunk() + HIDDEN
    }

    /// Its whole length, seconds.
    pub fn end(&self) -> f32 {
        self.rises() + self.keys.iter().map(|(_, seconds)| seconds).sum::<f32>()
    }

    /// Whether it has risen out of the pile, standing.
    pub fn is_done(&self) -> bool {
        self.t >= self.end()
    }

    /// Whether it is in the air.
    pub fn airborne(&self) -> bool {
        (TAKEOFF..self.landed()).contains(&self.t)
    }

    /// The centre of mass and the pose (the walker's frame), `t` in, until
    /// it rises out.
    fn com_and_pose(&self, t: f32) -> (Vec3, LocalPose) {
        let rig = &self.rig;
        if t < TAKEOFF {
            let s = t / TAKEOFF;
            let at = hermite(self.from, self.leave, Vec3::ZERO, self.velocity, TAKEOFF, s);
            let w = smoothstep(s);
            let blended = crate::character::anim::clip::blend(&self.stood, &self.swan, w);
            return (at, pitched(&blended, rig, LEAN_OFF * w));
        }
        if t < self.landed() {
            let tau = t - TAKEOFF;
            let at = self.leave + self.velocity * tau - Vec3::Y * (0.5 * GRAVITY * tau * tau);
            let u = tau / self.flight;
            let pitch = if u < DIVE_AT {
                LEAN_OFF + (DIVE - LEAN_OFF) * smoothstep(u / DIVE_AT)
            } else {
                DIVE + (ON_BACK - DIVE) * smoothstep((u - DIVE_AT) / (1.0 - DIVE_AT))
            };
            return (at, pitched(&self.swan, rig, pitch));
        }
        let landing = self.leave + self.velocity * self.flight - Vec3::Y * (0.5 * GRAVITY * self.flight * self.flight);
        let land_velocity = self.velocity - Vec3::Y * (GRAVITY * self.flight);
        let sunk = landing.with_y(self.hay.floor() + SUNK) + land_velocity.with_y(0.0) * (0.5 * self.sink);
        let s = ((t - self.landed()) / self.sink).clamp(0.0, 1.0);
        (hermite(landing, sunk, land_velocity, Vec3::ZERO, self.sink, s), pitched(&self.swan, rig, ON_BACK))
    }

    /// The walker's root and pose now.
    fn root_and_pose(&self) -> (Vec3, LocalPose) {
        let turn = Quat::from_rotation_y(self.yaw);
        if self.t < self.rises() {
            let (at, pose) = self.com_and_pose(self.t);
            return (at - turn * com(&pose, &self.rig), pose);
        }
        // Rising out, on the floor under it, the root where it lies: from
        // lying sunk through the keys, the legs blended by their feet.
        let (lying_root, lying) = {
            let (at, pose) = self.com_and_pose(self.rises());
            (at - turn * com(&pose, &self.rig), pose)
        };
        let root = lying_root.with_y(self.hay.floor());
        let mut from = lying;
        from.root_translation += turn.inverse() * (lying_root - root);
        let mut t = self.t - self.rises();
        for (key, seconds) in &self.keys {
            if t <= *seconds {
                let w = smoothstep((t / seconds).clamp(0.0, 1.0));
                let blended = crate::character::anim::clip::blend(&from, key, w);
                return (root, legs_by_their_feet(blended, &from, key, w, &self.rig));
            }
            t -= seconds;
            from = *key;
        }
        (root, self.stood)
    }

    /// The pose now, on the rig it was planned on.
    pub fn pose(&self) -> LocalPose {
        self.root_and_pose().1
    }

    /// The walker's root now.
    pub fn root(&self) -> Vec3 {
        self.root_and_pose().0
    }

    /// The walker's facing: as it left.
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// Every joint in the world now.
    pub fn joints(&self) -> BoneSet<Vec3> {
        let (root, pose) = self.root_and_pose();
        let turn = Quat::from_rotation_y(self.yaw);
        let at = forward_kinematics_on(&pose, &self.rig);
        BoneSet::from_fn(|bone| root + turn * at[bone])
    }

    /// [`Self::pose`], each bone led ahead of its spring (`jump::lead_of`).
    pub fn pose_led(&self, springs: &BoneSet<crate::character::anim::math::SpringParams>) -> LocalPose {
        let mut pose = self.pose();
        let mut posed: Vec<(f32, LocalPose)> = Vec::with_capacity(3);
        for bone in Bone::ALL {
            let lead = crate::character::anim::jump::lead_of(&springs[bone]);
            if lead <= 1.0e-4 {
                continue;
            }
            pose.rotations[bone] = match posed.iter().find(|(at, _)| (at - lead).abs() < 1.0e-4) {
                Some((_, ahead)) => ahead.rotations[bone],
                None => {
                    let mut later = self.clone();
                    later.advance(lead);
                    let ahead = later.pose();
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// From a 6 m and a 10 m top into a 1 m pile 2-3 m out: the centre of
    /// mass flies ballistic into the pile's middle, the body lands on its
    /// back, sinks into the pile, nothing through the floor under it, every
    /// pose finite, and it rises out standing on that floor; a pile too
    /// near below, or behind, is not leapt into.
    #[test]
    fn a_leap_of_faith_lands_on_its_back_in_the_hay() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        for (top, out) in [(6.0f32, 2.5f32), (10.0, 3.0), (6.0, 1.5)] {
            let name = format!("{top} m top, the pile {out} m out");
            let hay = Haystack { top: forward * out + Vec3::Y * 1.0, radius: 1.2, height: 1.0 };
            let mut faith = LeapOfFaith::plan(Vec3::Y * top, 0.0, hay, &stood, &rig).unwrap_or_else(|| panic!("{name}: not planned"));
            let (mut coms, mut lowest, mut kink, mut fastest) = (Vec::new(), f32::MAX, 0.0f32, 0.0f32);
            let mut frames: Vec<BoneSet<Vec3>> = Vec::new();
            let mut on_back = None;
            while !faith.is_done() {
                faith.advance(DT);
                let pose = faith.pose();
                assert!(Bone::ALL.iter().all(|&b| pose.rotations[b].is_finite()), "{name}: NaN");
                let now = faith.joints();
                lowest = Bone::ALL.iter().map(|&b| now[b].y).fold(lowest, f32::min);
                if faith.airborne() {
                    coms.push(faith.root() + com(&pose, &rig));
                }
                if on_back.is_none() && faith.t >= faith.landed() {
                    // The chest's front: the rig's forward carried by the
                    // upper spine.
                    let chest = crate::character::anim::rig::accumulate_world_rotations(&pose, &rig)[Bone::Spine2]
                        * crate::character::anim::rig::accumulate_world_rotations(&LocalPose::REST, &rig)[Bone::Spine2].inverse()
                        * forward;
                    let middle = (now[Bone::Hips] - hay.top).with_y(0.0).length();
                    on_back = Some((chest.y, middle));
                }
                if faith.t >= faith.sunk() && faith.t < faith.rises() {
                    assert!(now[Bone::Head].y < hay.top.y && now[Bone::Hips].y < hay.top.y, "{name}: not sunk in, the head {:.2}", now[Bone::Head].y);
                }
                if frames.len() >= 2 && (faith.t < faith.landed() - DT || faith.t > faith.landed() + faith.sink + DT) {
                    let (a, b) = (&frames[frames.len() - 2], &frames[frames.len() - 1]);
                    kink = Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(kink, f32::max);
                    fastest = Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (b[bone] - b[Bone::Hips])).length() / DT).fold(fastest, f32::max);
                }
                frames.push(now);
            }
            let ballistic = coms.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT) + Vec3::Y * GRAVITY).length()).fold(0.0, f32::max);
            let (chest_up, off_middle) = on_back.expect("landed");
            eprintln!("{name}: ballistic off {ballistic:.3} m/s², chest up {chest_up:.2}, {off_middle:.2} m off the middle, lowest {lowest:.3}, kink {kink:.4}, fastest {fastest:.1}");
            assert!(ballistic < 0.05, "{name}: the flight off ballistic by {ballistic:.3} m/s²");
            assert!(chest_up > 0.8, "{name}: landed with the chest {chest_up:.2} up");
            assert!(off_middle < hay.radius, "{name}: landed {off_middle:.2} m off the pile's middle");
            assert!(lowest > hay.floor() - 0.02, "{name}: a joint {lowest:.3} m up, under the floor");
            assert!(kink < 0.05 && fastest < 14.0, "{name}: a step changed {kink:.4} m, a joint at {fastest:.1} m/s");
            let end = faith.pose();
            assert!(Bone::ALL.iter().all(|&b| 1.0 - end.rotations[b].dot(stood.rotations[b]).abs() < 1.0e-4), "{name}: not standing at the end");
            assert!((faith.root().y - hay.floor()).abs() < 1.0e-4, "{name}: risen onto {:.2}", faith.root().y);
        }
        let hay = Haystack { top: forward * 2.0 + Vec3::Y * 1.0, radius: 1.2, height: 1.0 };
        assert!(LeapOfFaith::plan(Vec3::Y * 2.5, 0.0, hay, &stood, &rig).is_none(), "leapt 1.5 m into hay");
        assert!(LeapOfFaith::plan(Vec3::Y * 6.0, std::f32::consts::PI, hay, &stood, &rig).is_none(), "leapt into hay behind");
    }
}
