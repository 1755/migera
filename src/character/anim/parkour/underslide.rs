//! Sliding under a low obstacle from a run: step 10 of the parkour design,
//! third part. Running at a slab overhead too low to run under (its
//! underside [`LOWEST`]-[`HIGHEST`] up), the body drops into a slide on its
//! seat ([`DROP`]): the trunk leant back, the lead leg out ahead, the other
//! bent with its knee up, the trailing hand down on the floor beside the
//! hips, the other arm forward. It slides on, braked by friction
//! ([`FRICTION`] of gravity), under the slab and past it, and rises to
//! standing as it slows ([`RISE`]).
//!
//! No slide data; the friction is a dry slide's, about a shoe's on
//! concrete, the rest by eye.

use bevy::math::{Quat, Vec3};

use super::Ledge;
use crate::character::anim::armik::{shoulder_lift, solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::GRAVITY;
use crate::character::anim::rig::{forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// The slab's underside, metres above the floor: lower than this, too low
/// to slide under; higher, run under it.
pub const LOWEST: f32 = 0.8;
pub const HIGHEST: f32 = 1.5;
/// The slide's braking, a share of gravity: cloth on a hard floor. At 0.45
/// a 4 m/s slide carried 1.65 m, so it began 0.65 m before a 0.5 m slab and
/// the hands were under it before the body was down.
pub const FRICTION: f32 = 0.3;
/// Dropping into the slide, seconds; the hips' height in it, metres; the
/// trunk leant back this far, radians.
/// The hips' 0.7 m down take free fall's 0.38 s; at 0.3, with the legs
/// forward in 0.18, the hips came down at 9 g and a foot's step changed
/// 3.4 cm in a frame.
const DROP: f32 = 0.42;
/// In the drop, the legs go to the slide's shape over this long, seconds,
/// and the hips start down this late.
const LEGS_DROP: f32 = 0.3;
const HIPS_LAG: f32 = 0.1;
const SLIDE_HIPS: f32 = 0.22;
const LEAN_BACK: f32 = 0.9;
/// It rises once slowed to this, m/s, over this long, seconds.
const RISE_SPEED: f32 = 1.2;
const RISE: f32 = 0.6;
/// The hips this far past the slab's far side before it rises, metres.
pub const PAST: f32 = 0.5;
/// The lead foot this far ahead of the hips, the other this far ahead and
/// out to its side, metres; the ankles this high over the floor.
const LEAD_AHEAD: f32 = 0.72;
const TRAIL_AHEAD: f32 = 0.45;
const TRAIL_OUT: f32 = 0.12;
const ANKLE_HIGH: f32 = 0.09;
/// Between the shapes no ankle lower than this over the floor nor toe
/// lower than it, metres (under a planted running foot's own, so the run's
/// pose is left as it is); the feet lifted off it softly over this, metres;
/// the lift in over this long from the start, seconds (in over the whole
/// drop, too weak at 0.2 s, an ankle came to 3 cm).
const ANKLE_FLOOR: f32 = 0.05;
const TOE_FLOOR: f32 = 0.0;
const FLOOR_SOFT: f32 = 0.03;
const LIFT_IN: f32 = 0.1;
/// The trailing hand on the floor this far out to its side and back from
/// the hips, metres, the wrist this high; the other arm swung forward this
/// far, radians, its elbow bent this much.
const HAND_OUT: f32 = 0.3;
const HAND_BACK: f32 = 0.12;
const WRIST_HIGH: f32 = 0.06;
const LEAD_ARM: (f32, f32) = (0.7, 0.6);
/// Each ankle no farther from its hip than this share of the leg (the
/// hips still high early in the drop, the lead foot's place on the floor
/// was out of reach, the leg straightened, and its knee flipped 6.8 cm in
/// a frame as it came back in).
const LEG_REACH: f32 = 0.95;
/// Clear of the slab by at least this much, metres (the head's top this
/// far over its joint).
const CLEARANCE: f32 = 0.03;
const HEAD_TOP: f32 = 0.12;

const LEGS: [Bone; 2] = [Bone::LeftFoot, Bone::RightFoot];
const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
const SIGN: [f32; 2] = [1.0, -1.0];

/// A slide under a slab, from a run.
#[derive(Debug, Clone)]
pub struct UnderSlide {
    /// The run's pose and the hips' world place as it began; the standing
    /// pose it rises to.
    from: LocalPose,
    hips_from: Vec3,
    stood: LocalPose,
    rig: RigGeometry,
    yaw: f32,
    forward: Vec3,
    /// The floor's height, the speed it began at, m/s; the lead leg (0
    /// left, 1 right).
    floor: f32,
    speed: f32,
    lead: usize,
    /// When it starts rising, and its end, seconds; how far it is.
    rises: f32,
    t: f32,
}

impl UnderSlide {
    /// How far before a slab `depth` deep a slide from `speed` begins, its
    /// face that far ahead of the root: the slide carries the hips
    /// [`PAST`] beyond its far side before it rises. `None` if the speed
    /// is too slow to carry it there.
    pub fn start_distance(speed: f32, depth: f32) -> Option<f32> {
        let braking = FRICTION * GRAVITY;
        let slid = (speed * speed - RISE_SPEED * RISE_SPEED) / (2.0 * braking);
        let before = slid - depth - PAST;
        (before > 0.0).then_some(before)
    }

    /// A slide under `slab` from a run at `speed` (m/s), the root at `root`
    /// turned `yaw` (its rig facing `forward` at none) and posed `from`,
    /// off the foot `planted` (the other leg leads): `None` if the slab is
    /// too low or too high to slide under, not ahead, or anything of the
    /// body would touch it on the way.
    #[allow(clippy::too_many_arguments)]
    pub fn plan(slab: &Ledge, root: Vec3, yaw: f32, speed: f32, planted: usize, from: &LocalPose, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let turn = Quat::from_rotation_y(yaw);
        let forward = (turn * rig.forward()).with_y(0.0).normalize_or_zero();
        let underside = slab.height() - slab.wall_below.max(super::geometry::BAR_DEPTH) - root.y;
        if !(LOWEST..=HIGHEST).contains(&underside) || forward.dot(-slab.out) < 0.9 {
            return None;
        }
        let braking = FRICTION * GRAVITY;
        let slide = Self {
            from: *from,
            hips_from: root + turn * forward_kinematics_on(from, rig)[Bone::Hips],
            stood: *stood,
            rig: rig.clone(),
            yaw,
            forward,
            floor: root.y,
            speed,
            lead: 1 - planted,
            rises: ((speed - RISE_SPEED) / braking).max(DROP),
            t: 0.0,
        };
        // Clear of the slab all the way.
        let slab_at = |p: Vec3| {
            let back = -slab.out_of(p);
            let along = (p - slab.a).dot(slab.along());
            (0.0..=slab.depth).contains(&back) && (0.0..=(slab.b - slab.a).length()).contains(&along)
        };
        let lowest = slab.height() - slab.wall_below.max(super::geometry::BAR_DEPTH) - CLEARANCE;
        let mut probe = slide.clone();
        while !probe.is_done() {
            let at = probe.joints();
            let head = at[Bone::Head] + Vec3::Y * HEAD_TOP;
            if Bone::ALL.iter().map(|&bone| at[bone]).chain([head]).any(|p| slab_at(p) && p.y > lowest) {
                return None;
            }
            probe.advance(1.0 / 60.0);
        }
        // Past it before rising: the hips at the rise beyond its far side.
        let mut rising = slide.clone();
        rising.t = rising.rises;
        // The run's hips stand a few centimetres behind its root.
        let hips = rising.hips();
        (-slab.out_of(hips) > slab.depth + PAST - 0.1).then_some(slide)
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt).min(self.end());
    }

    /// Its whole length, seconds.
    pub fn end(&self) -> f32 {
        self.rises + RISE
    }

    /// Whether it has stood up.
    pub fn is_done(&self) -> bool {
        self.t >= self.end()
    }

    /// How far it has gone along its way at `t`, metres, and how fast it
    /// goes, m/s: braked by friction to the rise, then to a stop over it.
    fn travel(&self, t: f32) -> (f32, f32) {
        let braking = FRICTION * GRAVITY;
        if t <= self.rises {
            (self.speed * t - 0.5 * braking * t * t, self.speed - braking * t)
        } else {
            let at_rise = self.speed * self.rises - 0.5 * braking * self.rises * self.rises;
            let v = (self.speed - braking * self.rises).max(0.0);
            let tau = (t - self.rises).min(RISE);
            (at_rise + v * tau - 0.5 * (v / RISE) * tau * tau, v * (1.0 - tau / RISE))
        }
    }

    /// The hips in the world now.
    fn hips(&self) -> Vec3 {
        self.hips_and_pose().0
    }

    /// The slide's own pose, the hips at `hips` (the world).
    fn sliding_pose(&self, hips: Vec3) -> LocalPose {
        let rig = &self.rig;
        let turn = Quat::from_rotation_y(self.yaw);
        let back = turn.inverse();
        let mut pose = crate::character::anim::jump::upper(&self.stood, rig, -LEAN_BACK, (0.0, 0.0));
        let at = forward_kinematics_on(&pose, rig);
        let root = hips - turn * at[Bone::Hips];
        let to_pose = |p: Vec3| back * (p - root) - at[Bone::Hips];
        let left = turn * rig.left();
        let sockets = [Bone::LeftUpLeg, Bone::RightUpLeg];
        let knees = [Bone::LeftLeg, Bone::RightLeg];
        for (side, &ankle) in LEGS.iter().enumerate() {
            let target = if side == self.lead {
                hips + self.forward * LEAD_AHEAD
            } else {
                hips + self.forward * TRAIL_AHEAD + left * (SIGN[side] * TRAIL_OUT)
            };
            let target = to_pose(target.with_y(self.floor + ANKLE_HIGH)) + at[Bone::Hips];
            let leg = (at[knees[side]] - at[sockets[side]]).length() + (at[ankle] - at[knees[side]]).length();
            let off = target - at[sockets[side]];
            let target = at[sockets[side]] + off * (off.length().min(LEG_REACH * leg) / off.length().max(1.0e-6));
            place_ankle(&mut pose, rig, ankle, target - at[Bone::Hips]);
        }
        // The lead arm forward, the trailing hand down on the floor.
        let trail = 1 - self.lead;
        for bone in [ARMS[self.lead].shoulder, ARMS[self.lead].elbow] {
            let angle = if bone == ARMS[self.lead].shoulder { LEAD_ARM.0 } else { LEAD_ARM.1 };
            pose.rotations[bone] = crate::character::anim::rig::delta_after_world_turn(&pose, rig, bone, Quat::from_axis_angle(rig.left(), -angle));
        }
        let wrist = (hips + left * (SIGN[trail] * HAND_OUT) - self.forward * HAND_BACK).with_y(self.floor + WRIST_HIGH);
        let target = back * (wrist - root);
        let at = forward_kinematics_on(&pose, rig);
        let arm = (at[ARMS[trail].elbow] - at[ARMS[trail].shoulder]).length() + (at[ARMS[trail].wrist] - at[ARMS[trail].elbow]).length();
        let lift = shoulder_lift(at[CLAVICLES[trail]], at[ARMS[trail].shoulder], target, 0.85 * arm);
        pose.rotations[CLAVICLES[trail]] = crate::character::anim::rig::delta_after_world_turn(&pose, rig, CLAVICLES[trail], lift);
        let at = forward_kinematics_on(&pose, rig);
        let pole = (rig.left() * SIGN[trail] - rig.forward() * 0.5).normalize();
        solve_arm_toward_from(&mut pose, &at, ARMS[trail], target, pole, rig);
        pose
    }

    /// The pose now, on the rig it was planned on, in the walker's frame at
    /// [`Self::root`] turned [`Self::facing`]: from the run's into the
    /// slide's over [`DROP`], from the slide's to standing over [`RISE`].
    pub fn pose(&self) -> LocalPose {
        self.hips_and_pose().1
    }

    /// The hips in the world and the pose now. Dropping, the hips come
    /// down after the legs go forward; rising, the slide's shape blends to
    /// standing and the hips stand as high as its legs put them, the lower
    /// foot on the floor (risen on a curve of their own, the legs stood up
    /// faster than the hips and the feet went 5 cm into the floor).
    fn hips_and_pose(&self) -> (Vec3, LocalPose) {
        let (along, _) = self.travel(self.t);
        let level = self.hips_from + self.forward * along;
        let blend = |a: &LocalPose, b: &LocalPose, w: f32| {
            let mut pose = *a;
            for bone in Bone::ALL {
                pose.rotations[bone] = a.rotations[bone].slerp(b.rotations[bone], w);
            }
            pose.root_translation = a.root_translation.lerp(b.root_translation, w);
            pose
        };
        let (hips, mut pose, sliding) = if self.t <= self.rises {
            let lowered = smoothstep(((self.t - HIPS_LAG) / (DROP - HIPS_LAG)).clamp(0.0, 1.0));
            let from = self.hips_from.y - self.floor;
            let hips = level.with_y(self.floor + from + (SLIDE_HIPS - from) * lowered);
            let slide = self.sliding_pose(hips);
            let w = smoothstep((self.t / DROP).clamp(0.0, 1.0));
            let mut pose = blend(&self.from, &slide, w);
            // The legs forward first, the hips after: together, the run's
            // legs went through the floor as the hips came down.
            let legs = smoothstep((self.t / LEGS_DROP).clamp(0.0, 1.0));
            for bone in [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase, Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase] {
                pose.rotations[bone] = self.from.rotations[bone].slerp(slide.rotations[bone], legs);
            }
            (hips, pose, smoothstep((self.t / LIFT_IN).clamp(0.0, 1.0)))
        } else {
            let w = smoothstep(((self.t - self.rises) / RISE).clamp(0.0, 1.0));
            let slide = self.sliding_pose(level.with_y(self.floor + SLIDE_HIPS));
            let pose = blend(&slide, &self.stood, w);
            let at = forward_kinematics_on(&pose, &self.rig);
            let standing = forward_kinematics_on(&self.stood, &self.rig);
            let ankle_floor = ANKLE_HIGH + (standing[Bone::LeftFoot].y.min(standing[Bone::RightFoot].y) - ANKLE_HIGH) * w;
            let height = LEGS.iter().map(|&ankle| at[Bone::Hips].y - at[ankle].y).fold(f32::MIN, f32::max) + ankle_floor;
            (level.with_y(self.floor + height), pose, 1.0 - w)
        };
        // No foot under the floor between the shapes (blended from the run's
        // as the hips dropped, the feet went under it).
        let turn = Quat::from_rotation_y(self.yaw);
        let at = forward_kinematics_on(&pose, &self.rig);
        let root = hips - turn * at[Bone::Hips];
        // Raised by what the ankle or (the run's feet pointed) the toe
        // lacks: the foot keeps its attitude, so the toe rises as much.
        // Softly (a softplus of the lack): lifted only once under, the lift
        // began in a frame and a joint's step changed 4.75 cm. Faded in with
        // the drop and out with the rise, so it starts on the run's pose and
        // ends on standing's (lifted from the first frame, the knees jumped
        // 7 cm).
        for (&ankle, toe) in LEGS.iter().zip([Bone::LeftToeBase, Bone::RightToeBase]) {
            let (world, toe) = (root + turn * at[ankle], root + turn * at[toe]);
            let short = (self.floor + ANKLE_FLOOR - world.y).max(self.floor + TOE_FLOOR - toe.y);
            let lift = sliding * FLOOR_SOFT * (short / FLOOR_SOFT).exp().ln_1p();
            if lift > 1.0e-4 {
                let target = turn.inverse() * (world + Vec3::Y * lift - root) - at[Bone::Hips];
                place_ankle(&mut pose, &self.rig, ankle, target);
            }
        }
        (hips, pose)
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the
    /// spring trails a steady motion (`jump::lead_of`): the slide as it will
    /// be that much later. The root is not sprung, so the rendered body is
    /// the slide's now (posed unled, the legs trailed the dropping hips and
    /// the feet went 6 cm into the floor).
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

    /// The walker's root now: the pose's hips on the slide's hips.
    pub fn root(&self) -> Vec3 {
        let (hips, pose) = self.hips_and_pose();
        hips - Quat::from_rotation_y(self.yaw) * forward_kinematics_on(&pose, &self.rig)[Bone::Hips]
    }

    /// The walker's facing: its way.
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// How fast it goes along its way now, m/s.
    pub fn speed(&self) -> f32 {
        self.travel(self.t).1
    }

    /// Every joint in the world now.
    pub fn joints(&self) -> BoneSet<Vec3> {
        let (pose, root, turn) = (self.pose(), self.root(), Quat::from_rotation_y(self.yaw));
        let at = forward_kinematics_on(&pose, &self.rig);
        BoneSet::from_fn(|bone| root + turn * at[bone])
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

    /// A slab overhead, its underside `under` up, `depth` deep, its face
    /// `near` ahead of a root at the origin running along the rig's
    /// forward.
    fn slab(rig: &RigGeometry, near: f32, depth: f32, under: f32) -> Ledge {
        let forward = rig.forward();
        let thick = 0.2;
        Ledge { wall_below: thick, ..Ledge::wall(forward * near, -forward, 3.0, under + thick, depth) }
    }

    /// From runs at 4.5-6 m/s at slabs 0.85-1.1 m up and 0.5-1 m deep, the
    /// slide starts where it should and plans; under it nothing touches the
    /// slab, the pose is continuous from the run and into standing, no joint
    /// whips round, and it stands past the slab. Too low a slab, or too
    /// slow a run to clear a deep one, is not slid under.
    #[test]
    fn a_low_slab_is_slid_under_from_a_run() {
        let (stood, rig) = real_stood();
        let dt = 1.0 / 60.0;
        let mut faults = Vec::new();
        for speed in [4.5f32, 5.0, 6.0] {
            for depth in [0.5f32, 1.0] {
                for under in [0.85f32, 1.1] {
                    let name = format!("{speed} m/s, {depth} m deep, {under} m up");
                    let Some(near) = UnderSlide::start_distance(speed, depth) else {
                        faults.push(format!("{name}: no start distance"));
                        continue;
                    };
                    let slab = slab(&rig, near, depth, under);
                    // The run's pose: standing, a run's lean on it.
                    let from = crate::character::anim::jump::upper(&stood, &rig, 0.15, (0.3, 0.6));
                    let Some(mut slide) = UnderSlide::plan(&slab, Vec3::ZERO, 0.0, speed, 0, &from, &stood, &rig) else {
                        faults.push(format!("{name}: not planned"));
                        continue;
                    };
                    let world = |s: &UnderSlide| s.joints();
                    let start = forward_kinematics_on(&from, &rig);
                    let mut frames = vec![BoneSet::from_fn(|bone| start[bone] - rig.forward() * speed * dt), BoneSet::from_fn(|bone| start[bone])];
                    let (mut fastest, mut kink, mut lowest) = (0.0f32, 0.0f32, f32::MAX);
                    while !slide.is_done() {
                        slide.advance(dt);
                        let now = world(&slide);
                        lowest = lowest.min(now[Bone::LeftFoot].y.min(now[Bone::RightFoot].y)).min(now[Bone::LeftToeBase].y.min(now[Bone::RightToeBase].y) + 0.05);
                        let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
                        fastest = fastest.max(Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (b[bone] - b[Bone::Hips])).length() / dt).fold(0.0, f32::max));
                        kink = kink.max(Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(0.0, f32::max));
                        frames.push(now);
                    }
                    let end = slide.root();
                    let past = -slab.out_of(end) - depth;
                    eprintln!("{name}: start {near:.2} m, fastest {fastest:.1} m/s, kink {kink:.4}, stood {past:.2} m past");
                    if fastest >= 14.0 {
                        faults.push(format!("{name}: a joint at {fastest:.1} m/s"));
                    }
                    // The lead foot meets its place on the floor as the hips
                    // come down, a strike: 2.8 cm.
                    if kink >= 0.03 {
                        faults.push(format!("{name}: a joint's step changed {kink:.4} m"));
                    }
                    if past < PAST - 0.1 {
                        faults.push(format!("{name}: stood only {past:.2} m past the slab"));
                    }
                    // The ankles 5 cm up at least, the toes no more than 2 cm
                    // into the floor (blended from the run, both went under).
                    if lowest < 0.03 {
                        faults.push(format!("{name}: a foot at {lowest:.3} m (an ankle, or a toe less 5 cm)"));
                    }
                }
            }
        }
        // From the run's own poses, any foot.
        for phase in [0.0f32, 0.25, 0.5, 0.75] {
            let run = crate::character::anim::gait::GaitParams::running_on(5.0, &rig);
            let from = crate::character::anim::gait::walk_pose_on(phase, &run, &stood, &rig);
            let near = UnderSlide::start_distance(5.0, 0.8).expect("a start");
            for planted in 0..2 {
                let Some(mut slide) = UnderSlide::plan(&slab(&rig, near, 0.8, 0.9), Vec3::ZERO, 0.0, 5.0, planted, &from, &stood, &rig) else {
                    faults.push(format!("from the run at {phase}, off foot {planted}: not planned"));
                    continue;
                };
                let mut lowest = f32::MAX;
                while !slide.is_done() {
                    slide.advance(dt);
                    let now = slide.joints();
                    lowest = lowest.min(now[Bone::LeftFoot].y.min(now[Bone::RightFoot].y)).min(now[Bone::LeftToeBase].y.min(now[Bone::RightToeBase].y) + 0.05);
                }
                if lowest < 0.03 {
                    faults.push(format!("from the run at {phase}, off foot {planted}: a foot at {lowest:.3} m"));
                }
            }
        }
        let low = slab(&rig, 2.0, 0.5, 0.6);
        let from = stood;
        if UnderSlide::plan(&low, Vec3::ZERO, 0.0, 4.0, 0, &from, &stood, &rig).is_some() {
            faults.push("a 0.6 m slab slid under".into());
        }
        if UnderSlide::start_distance(2.0, 1.5).is_some() {
            faults.push("a 1.5 m deep slab from 2 m/s planned".into());
        }
        assert!(faults.is_empty(), "{} faults:\n{}", faults.len(), faults.join("\n"));
    }
}
