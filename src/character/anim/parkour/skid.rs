//! Reversing from a run: step 11 of the parkour steps beyond the first ten
//! (running agility), second part. A skid stop: the body turns side-on to
//! its way and its feet go out ahead, both sliding, braked by friction
//! ([`FRICTION`] of gravity), the body leant back against the way by as
//! much (`tan θ = μ`); slowed, the feet stick and the hips come forward
//! over them to standing. A plant-and-turn is the same braking with the
//! body turned all the way round while it slides, so it stands facing back
//! ([`Skid::plan`]'s `round`).
//!
//! No skid data: the friction is a shoe sliding on a hard floor, the rest
//! by eye.

use bevy::math::{Quat, Vec3};

use crate::character::anim::anthropometry::centre_of_mass;
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::GRAVITY;
use crate::character::anim::rig::{forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// The slowest run that skids, m/s: slower, its slide is over before the
/// feet are out ahead.
pub const SKID_FROM: f32 = 3.2;
/// The slide's braking, a share of gravity: a shoe sliding on a hard floor.
pub const FRICTION: f32 = 0.5;
/// Going into the skid, seconds: the legs out ahead over this long, the
/// hips lowered from this late.
const ENTRY: f32 = 0.35;
const LEGS_IN: f32 = 0.35;
const HIPS_LAG: f32 = 0.05;
/// The hips this much lower than standing in the slide, metres, the trunk
/// leant back against the way this far, radians: upright, the legs out
/// ahead carry the centre of mass with them, and at 0.12 m down the feet
/// went 0.53-0.63 m out to lean by `atan(μ)`, 4-5 cm out of reach.
const DROP: f32 = 0.28;
const TRUNK_BACK: f32 = 0.3;
/// Each ankle no farther from its hip than this share of the leg.
const LEG_REACH: f32 = 0.95;
/// A plant-and-turn brakes for at least this long, seconds, so it turns no
/// faster than half a turn in it.
const TURN_LEAST: f32 = 0.45;
/// A skid stop turns the body this far side-on to its way, toward the
/// leading leg's other side, radians (a plant-and-turn half a turn).
const SIDE_ON: f32 = 1.2;
/// Slowed, the feet stick and the hips come over them in this long,
/// seconds.
const RISE: f32 = 0.5;
/// The arms out for balance, this share of the beam's.
const ARMS_OUT: f32 = 0.6;
/// Between the shapes no ankle nor toe lower than this under standing's
/// height, metres; lifted softly over this, metres, the lift faded in over
/// the first this long, seconds, and out over the entry's last (the shape's
/// own feet are on the floor; softened over 1 cm, a toe's step changed
/// 3.9 cm in a frame as the lift came on).
const FLOOR_UNDER: f32 = 0.01;
const FLOOR_SOFT: f32 = 0.025;
const LIFT_IN: f32 = 0.05;
const LIFT_OUT: f32 = 0.1;

/// How far the feet slide ahead is solved at this many facings through the
/// turn (each its centre of mass off its hips differently: one distance for
/// all leant a plant-and-turn 0.37-0.42 rad, not 0.46).
const AHEADS: usize = 17;

const FEET: [Bone; 2] = [Bone::LeftFoot, Bone::RightFoot];
const TOES: [Bone; 2] = [Bone::LeftToeBase, Bone::RightToeBase];

/// A skid stop or a plant-and-turn, from a run.
#[derive(Debug, Clone)]
pub struct Skid {
    /// The run's pose and the hips' world place as it began; the standing
    /// pose it ends in.
    from: LocalPose,
    hips_from: Vec3,
    stood: LocalPose,
    rig: RigGeometry,
    /// The way it goes (level, unit) and the facing it began with.
    way: Vec3,
    yaw: f32,
    /// How far it turns, radians (positive to its left).
    turn: f32,
    floor: f32,
    speed: f32,
    /// How far ahead of the hips its feet slide, metres, through its turn
    /// (from facing as it began to as it ends, evenly); the speed they stick
    /// at, m/s; when they stick, seconds.
    aheads: [f32; AHEADS],
    sticks_at: f32,
    sticks: f32,
    /// The feet's world places once stuck.
    stuck: Option<[Vec3; 2]>,
    t: f32,
}

impl Skid {
    /// A skid from a run at `speed` (m/s), the root at `root` turned `yaw`
    /// and posed `from`, its way the rig's forward turned by `yaw`; the
    /// foot `planted` the one down (the body turns away from the other, so
    /// it leads). `round`, it turns half a turn and stands facing back.
    /// `None` too slow to skid.
    #[allow(clippy::too_many_arguments)]
    pub fn plan(root: Vec3, yaw: f32, speed: f32, planted: usize, round: bool, from: &LocalPose, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        if speed < SKID_FROM {
            return None;
        }
        let turn_q = Quat::from_rotation_y(yaw);
        let way = (turn_q * rig.forward()).with_y(0.0).normalize_or_zero();
        // Turned away from the swinging leg: it comes round in front.
        let side = if planted == 0 { -1.0 } else { 1.0 };
        let turn = side * if round { std::f32::consts::PI } else { SIDE_ON };
        let mut skid = Self {
            from: *from,
            hips_from: root + turn_q * forward_kinematics_on(from, rig)[Bone::Hips],
            stood: *stood,
            rig: rig.clone(),
            way,
            yaw,
            turn,
            floor: root.y,
            speed,
            aheads: [0.0; AHEADS],
            sticks_at: 0.0,
            sticks: 0.0,
            stuck: None,
            t: 0.0,
        };
        for i in 0..AHEADS {
            skid.aheads[i] = skid.ahead_for(yaw + turn * i as f32 / (AHEADS - 1) as f32);
        }
        // Stuck at the speed that brings the hips over the feet by the end
        // of the rise, braked steadily to a stop; turned as it will end.
        skid.sticks_at = 2.0 * skid.aheads[AHEADS - 1] / RISE;
        skid.sticks = (speed - skid.sticks_at) / (FRICTION * GRAVITY);
        if skid.sticks < if round { TURN_LEAST } else { ENTRY } {
            return None;
        }
        // Each foot where the shape puts it, at every facing.
        let hips = Vec3::Y * (skid.floor + skid.low());
        let reached = (0..AHEADS).all(|i| {
            let yaw = yaw + turn * i as f32 / (AHEADS - 1) as f32;
            let pose = skid.skid_pose(hips, yaw);
            let at = forward_kinematics_on(&pose, rig);
            let root = hips - Quat::from_rotation_y(yaw) * at[Bone::Hips];
            FEET.iter().all(|&foot| (root + Quat::from_rotation_y(yaw) * at[foot] - skid.foot_target(hips, yaw, skid.aheads[i], foot)).length() < 1.0e-3)
        });
        reached.then_some(skid)
    }

    /// Where `foot` slides, the hips at `hips` facing `yaw`, the feet `ahead`.
    fn foot_target(&self, hips: Vec3, yaw: f32, ahead: f32, foot: Bone) -> Vec3 {
        let standing = forward_kinematics_on(&self.stood, &self.rig);
        let under = Quat::from_rotation_y(yaw) * (standing[foot] - standing[Bone::Hips]);
        (hips + under.with_y(0.0) + self.way * ahead).with_y(self.floor + standing[foot].y)
    }

    /// The hips' height in the slide over the floor, metres.
    fn low(&self) -> f32 {
        forward_kinematics_on(&self.stood, &self.rig)[Bone::Hips].y - DROP
    }

    /// How far ahead of the hips along its way the feet slide facing `yaw`
    /// for the centre of mass to be leant back from them by `atan(μ)`:
    /// solved by a few corrections of what the shape gives.
    fn ahead_for(&self, yaw: f32) -> f32 {
        let hips = Vec3::Y * (self.floor + self.low());
        let turn = Quat::from_rotation_y(yaw);
        let mut ahead = FRICTION * (centre_of_mass(&self.stood, &self.rig).y + self.low());
        for _ in 0..4 {
            let pose = self.shape(hips, yaw, ahead);
            let at = forward_kinematics_on(&pose, &self.rig);
            let root = hips - turn * at[Bone::Hips];
            let feet = root + turn * (at[Bone::LeftFoot] + at[Bone::RightFoot]) * 0.5;
            let com = hips + turn * centre_of_mass(&pose, &self.rig);
            ahead += FRICTION * (com.y - self.floor) - (feet - com).dot(self.way);
        }
        ahead
    }

    /// How far ahead the feet slide facing `yaw` ([`Self::aheads`]).
    fn ahead_at(&self, yaw: f32) -> f32 {
        if self.turn.abs() < 1.0e-6 {
            return self.aheads[0];
        }
        let x = ((yaw - self.yaw) / self.turn).clamp(0.0, 1.0) * (AHEADS - 1) as f32;
        let i = (x.floor() as usize).min(AHEADS - 2);
        self.aheads[i] + (self.aheads[i + 1] - self.aheads[i]) * (x - i as f32)
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        let before = self.t;
        self.t = (self.t + dt).min(self.end());
        if before < self.sticks && self.t >= self.sticks {
            let mut at_stick = self.clone();
            at_stick.t = self.sticks;
            let (hips, pose, yaw) = at_stick.sliding();
            let at = forward_kinematics_on(&pose, &self.rig);
            let (turn, root) = (Quat::from_rotation_y(yaw), hips - Quat::from_rotation_y(yaw) * at[Bone::Hips]);
            self.stuck = Some(FEET.map(|foot| root + turn * at[foot]));
        }
    }

    /// Its whole length, seconds.
    pub fn end(&self) -> f32 {
        self.sticks + RISE
    }

    /// Whether it stands.
    pub fn is_done(&self) -> bool {
        self.t >= self.end()
    }

    /// Whether its feet have stuck.
    pub fn is_stuck(&self) -> bool {
        self.t >= self.sticks
    }

    /// How far along its way the hips are at `t`, metres, and how fast they
    /// go, m/s: braked by friction till the feet stick, then steadily to a
    /// stop over the rise.
    fn travel(&self, t: f32) -> (f32, f32) {
        let braking = FRICTION * GRAVITY;
        let s = t.min(self.sticks);
        let slid = self.speed * s - 0.5 * braking * s * s;
        if t <= self.sticks {
            return (slid, self.speed - braking * t);
        }
        let tau = (t - self.sticks).min(RISE);
        (slid + self.sticks_at * tau - 0.5 * (self.sticks_at / RISE) * tau * tau, self.sticks_at * (1.0 - tau / RISE))
    }

    /// Its facing at `t`: turned side-on (or round) while it slides.
    fn yaw_at(&self, t: f32) -> f32 {
        self.yaw + self.turn * smoothstep((t / self.sticks).clamp(0.0, 1.0))
    }

    /// The skid's own shape, the hips at `hips` (the world), facing `yaw`
    /// ([`Self::shape`] with the feet as far ahead as it leans by).
    fn skid_pose(&self, hips: Vec3, yaw: f32) -> LocalPose {
        self.shape(hips, yaw, self.ahead_at(yaw))
    }

    /// Standing, the arms out, the hips at `hips` (the world) facing `yaw`,
    /// the feet slid out `ahead` of them along its way on the floor.
    fn shape(&self, hips: Vec3, yaw: f32, ahead: f32) -> LocalPose {
        let rig = &self.rig;
        let mut pose = self.stood;
        super::beam::balance(&mut pose, rig, ARMS_OUT, 0.0);
        let turn = Quat::from_rotation_y(yaw);
        let back = turn.inverse() * -self.way;
        let lean = Quat::from_rotation_arc(Vec3::Y, Vec3::Y * TRUNK_BACK.cos() + back * TRUNK_BACK.sin());
        pose.rotations[Bone::Spine] = crate::character::anim::rig::delta_after_world_turn(&pose, rig, Bone::Spine, lean);
        let at = forward_kinematics_on(&pose, rig);
        let root = hips - turn * at[Bone::Hips];
        for (foot, (socket, knee)) in FEET.into_iter().zip([(Bone::LeftUpLeg, Bone::LeftLeg), (Bone::RightUpLeg, Bone::RightLeg)]) {
            let target = turn.inverse() * (self.foot_target(hips, yaw, ahead, foot) - root);
            let leg = (at[knee] - at[socket]).length() + (at[foot] - at[knee]).length();
            let off = target - at[socket];
            let target = at[socket] + off * (off.length().min(LEG_REACH * leg) / off.length().max(1.0e-6));
            place_ankle(&mut pose, rig, foot, target - at[Bone::Hips]);
        }
        pose
    }

    /// The hips, pose and facing while sliding (till the feet stick): from
    /// the run's into the skid's over [`ENTRY`], the legs first, the hips
    /// lowered after.
    fn sliding(&self) -> (Vec3, LocalPose, f32) {
        let (along, _) = self.travel(self.t);
        let level = self.hips_from + self.way * along;
        let low = self.low();
        let lowered = smoothstep(((self.t - HIPS_LAG) / (ENTRY - HIPS_LAG)).clamp(0.0, 1.0));
        let from = self.hips_from.y - self.floor;
        let hips = level.with_y(self.floor + from + (low - from) * lowered);
        let yaw = self.yaw_at(self.t);
        let skid = self.skid_pose(hips, yaw);
        // The run's pose, as it was, turned with the body.
        let w = smoothstep((self.t / ENTRY).clamp(0.0, 1.0));
        let legs = smoothstep((self.t / LEGS_IN).clamp(0.0, 1.0));
        let mut pose = skid;
        for bone in Bone::ALL {
            let share = if is_leg(bone) { legs } else { w };
            pose.rotations[bone] = self.from.rotations[bone].slerp(skid.rotations[bone], share);
        }
        pose.root_translation = self.from.root_translation.lerp(skid.root_translation, w);
        let lifting = smoothstep((self.t / LIFT_IN).clamp(0.0, 1.0)) * (1.0 - smoothstep(((self.t - ENTRY + LIFT_OUT) / LIFT_OUT).clamp(0.0, 1.0)));
        self.keep_off_floor(&mut pose, hips, yaw, lifting);
        (hips, pose, yaw)
    }

    /// Lifts each foot clear of the floor where the blend took it under,
    /// softly, by `weight`.
    fn keep_off_floor(&self, pose: &mut LocalPose, hips: Vec3, yaw: f32, weight: f32) {
        if weight <= 0.0 {
            return;
        }
        let turn = Quat::from_rotation_y(yaw);
        let standing = forward_kinematics_on(&self.stood, &self.rig);
        let at = forward_kinematics_on(pose, &self.rig);
        let root = hips - turn * at[Bone::Hips];
        for (foot, toe) in FEET.into_iter().zip(TOES) {
            let (ankle, tip) = (root + turn * at[foot], root + turn * at[toe]);
            let short = (self.floor + standing[foot].y - FLOOR_UNDER - ankle.y).max(self.floor + standing[toe].y - FLOOR_UNDER - tip.y);
            let lift = weight * FLOOR_SOFT * (short / FLOOR_SOFT).exp().ln_1p();
            if lift > 1.0e-4 {
                place_ankle(pose, &self.rig, foot, turn.inverse() * (ankle + Vec3::Y * lift - root) - at[Bone::Hips]);
            }
        }
    }

    /// The hips, pose and facing now.
    fn hips_pose_yaw(&self) -> (Vec3, LocalPose, f32) {
        let Some(stuck) = self.stuck.filter(|_| self.t > self.sticks) else {
            return self.sliding();
        };
        // Stuck: the hips come forward over the feet, rising to standing,
        // the arms coming in; each foot held where it stuck.
        let w = smoothstep(((self.t - self.sticks) / RISE).clamp(0.0, 1.0));
        let (along, _) = self.travel(self.t);
        let yaw = self.yaw_at(self.t);
        let hips = (self.hips_from + self.way * along).with_y(self.floor + self.low() + DROP * w);
        let skid = self.skid_pose(hips, yaw);
        let mut pose = skid;
        for bone in Bone::ALL {
            pose.rotations[bone] = skid.rotations[bone].slerp(self.stood.rotations[bone], w);
        }
        let turn = Quat::from_rotation_y(yaw);
        let at = forward_kinematics_on(&pose, &self.rig);
        let root = hips - turn * at[Bone::Hips];
        for (side, foot) in FEET.into_iter().enumerate() {
            place_ankle(&mut pose, &self.rig, foot, turn.inverse() * (stuck[side] - root) - at[Bone::Hips]);
        }
        (hips, pose, yaw)
    }

    /// The pose now, on the rig it was planned on, at [`Self::root`] turned
    /// [`Self::facing`].
    pub fn pose(&self) -> LocalPose {
        self.hips_pose_yaw().1
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

    /// The walker's root now: the pose's hips on the skid's hips.
    pub fn root(&self) -> Vec3 {
        let (hips, pose, yaw) = self.hips_pose_yaw();
        hips - Quat::from_rotation_y(yaw) * forward_kinematics_on(&pose, &self.rig)[Bone::Hips]
    }

    /// The walker's facing now.
    pub fn facing(&self) -> f32 {
        self.yaw_at(self.t)
    }

    /// How fast it goes along its way now, m/s.
    pub fn speed(&self) -> f32 {
        self.travel(self.t).1
    }

    /// Every joint in the world now.
    pub fn joints(&self) -> BoneSet<Vec3> {
        let (hips, pose, yaw) = self.hips_pose_yaw();
        let turn = Quat::from_rotation_y(yaw);
        let at = forward_kinematics_on(&pose, &self.rig);
        let root = hips - turn * at[Bone::Hips];
        BoneSet::from_fn(|bone| root + turn * at[bone])
    }

    /// The whole body's centre of mass in the world now.
    pub fn centre_of_mass(&self) -> Vec3 {
        let (hips, pose, yaw) = self.hips_pose_yaw();
        hips + Quat::from_rotation_y(yaw) * centre_of_mass(&pose, &self.rig)
    }
}

fn is_leg(bone: Bone) -> bool {
    matches!(bone, Bone::LeftUpLeg | Bone::LeftLeg | Bone::LeftFoot | Bone::LeftToeBase | Bone::RightUpLeg | Bone::RightLeg | Bone::RightFoot | Bone::RightToeBase)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gait::{leg_length_of, walk_pose_on, GaitParams};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// What a skid measured.
    #[derive(Debug, Default)]
    struct Skidded {
        /// The braking while sliding, its least and most, m/s².
        braking: (f32, f32),
        /// The lean back of the centre of mass from the feet while sliding
        /// past the entry, least and most, radians.
        lean: (f32, f32),
        /// The most a stuck foot moves, a foot goes under the floor, a
        /// joint's step changes in a frame; the fastest joint about the
        /// hips.
        stuck_moved: f32,
        under: f32,
        kink: f32,
        fastest: f32,
    }

    fn run(skid: &mut Skid, from: &LocalPose, rig: &RigGeometry, speed: f32) -> Skidded {
        let dt = 1.0 / 60.0;
        let start = forward_kinematics_on(from, rig);
        let mut frames = vec![BoneSet::from_fn(|bone| start[bone] - rig.forward() * speed * dt), BoneSet::from_fn(|bone| start[bone])];
        let mut m = Skidded { braking: (f32::MAX, f32::MIN), lean: (f32::MAX, f32::MIN), ..Default::default() };
        let mut speeds = vec![skid.speed()];
        let mut stuck_at: Option<[Vec3; 2]> = None;
        while !skid.is_done() {
            skid.advance(dt);
            let now = skid.joints();
            speeds.push(skid.speed());
            if skid.t > ENTRY && !skid.is_stuck() {
                let n = speeds.len();
                let braking = (speeds[n - 2] - speeds[n - 1]) / dt;
                m.braking = (m.braking.0.min(braking), m.braking.1.max(braking));
                let feet = (now[Bone::LeftFoot] + now[Bone::RightFoot]) * 0.5;
                let com = skid.centre_of_mass();
                let lean = ((feet - com).dot(skid.way)).atan2(com.y - skid.floor);
                m.lean = (m.lean.0.min(lean), m.lean.1.max(lean));
            }
            if skid.t > skid.sticks {
                let feet = [now[Bone::LeftFoot], now[Bone::RightFoot]];
                let held = *stuck_at.get_or_insert(feet);
                m.stuck_moved = m.stuck_moved.max((feet[0] - held[0]).length()).max((feet[1] - held[1]).length());
            }
            for toe in TOES {
                m.under = m.under.max(skid.floor - now[toe].y);
            }
            let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
            m.kink = m.kink.max(Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(0.0, f32::max));
            m.fastest = m.fastest.max(Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (b[bone] - b[Bone::Hips])).length() / dt).fold(0.0, f32::max));
            frames.push(now);
        }
        m
    }

    /// From runs at 3.5-6 m/s, off either foot: braked at friction's
    /// share of gravity, leant back by `atan(μ)` (within 2°), the stuck feet
    /// held (1 mm), nothing under the floor, the pose continuous from the
    /// run's into standing, facing side-on; a plant-and-turn faces back.
    #[test]
    fn a_run_skids_to_a_stop_or_round_to_face_back() {
        let (stood, rig) = real_stood();
        let leg = leg_length_of(&rig);
        let mut faults = Vec::new();
        for speed in [3.5f32, 4.5, 6.0] {
            let params = GaitParams::running_for(speed, leg);
            for planted in [0usize, 1] {
                for round in [false, true] {
                    let name = format!("{speed} m/s, off {planted}, {}", if round { "round" } else { "stop" });
                    // The run's pose as that foot comes down.
                    let from = walk_pose_on(0.5 * planted as f32, &params, &stood, &rig);
                    let Some(mut skid) = Skid::plan(Vec3::ZERO, 0.0, speed, planted, round, &from, &stood, &rig) else {
                        faults.push(format!("{name}: not planned"));
                        continue;
                    };
                    let m = run(&mut skid, &from, &rig, speed);
                    eprintln!("{name}: {m:?}");
                    let braking = FRICTION * GRAVITY;
                    if (m.braking.0 - braking).abs() > 0.05 || (m.braking.1 - braking).abs() > 0.05 {
                        faults.push(format!("{name}: braked {:?}", m.braking));
                    }
                    let lean = FRICTION.atan();
                    if (m.lean.0 - lean).abs() > 2.0_f32.to_radians() || (m.lean.1 - lean).abs() > 2.0_f32.to_radians() {
                        faults.push(format!("{name}: leant {:?}, not {lean}", m.lean));
                    }
                    // The swinging foot thrown out ahead mid-entry: a 2.4-3.1
                    // cm change of step (the slide under a slab's strike,
                    // 2.8). Toes no more than 2 cm into the floor, as there.
                    if m.stuck_moved > 1.0e-3 || m.under > 0.02 || m.kink > 0.035 || m.fastest > 8.0 {
                        faults.push(format!("{name}: {m:?}"));
                    }
                    let turned = crate::character::anim::facing::shortest_angle(skid.facing() - skid.yaw);
                    let wanted = if round { std::f32::consts::PI } else { SIDE_ON };
                    if (turned.abs() - wanted).abs() > 1.0e-3 {
                        faults.push(format!("{name}: turned {turned}"));
                    }
                    let end = skid.pose();
                    if let Some(bone) = Bone::ALL.iter().find(|&&bone| 1.0 - end.rotations[bone].dot(stood.rotations[bone]).abs() > 1.0e-4) {
                        let (a, b) = (forward_kinematics_on(&end, &rig), forward_kinematics_on(&stood, &rig));
                        faults.push(format!("{name}: not standing at the end, {bone:?}; feet off {:.4} {:.4}", (a[Bone::LeftFoot] - a[Bone::Hips] - b[Bone::LeftFoot] + b[Bone::Hips]).length(), (a[Bone::RightFoot] - a[Bone::Hips] - b[Bone::RightFoot] + b[Bone::Hips]).length()));
                    }
                }
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
        assert!(Skid::plan(Vec3::ZERO, 0.0, 2.5, 0, false, &stood, &stood, &rig).is_none(), "a slow run skidded");
    }
}
