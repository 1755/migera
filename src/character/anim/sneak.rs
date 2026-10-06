//! Sneaking: the body lowered over bent knees, and the heels kept up
//! ([`Sneak`]).
//!
//! A sneak is asked as two dials: how deep to crouch, 0 (standing) to 1
//! (the deepest sneak, [`DEEPEST`]), and whether to go on the toes. This
//! module poses a crouch from a stand and eases between crouches
//! ([`Crouching`]).
//!
//! A crouch is posed as the jump's countermovement is
//! (`jump::Feet::solved`): the trunk leans and the arms come forward with
//! the depth, and then the pelvis is solved so the pose's real centre of
//! mass (COM) is where the crouch asks: lowered by its depth, and over the
//! feet. Flat, over where it stood; on the toes, the heels risen
//! [`TOES_HEEL`] about the toe tips, over the balls of the feet. The feet
//! stay where they stood.

use super::foot::Sole;
use super::gait::{hermite, leg_length_of};
use super::jump::{self, com_of, Aim, Feet};
use super::rig::{LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// How far the deepest sneak lowers the COM below standing, as a fraction
/// of leg length (thigh plus shank): on `puppet_base` 0.16 m, the knees
/// folded 75° and the shanks leaning 29° forward, past severe crouch gait's
/// 50° (Steele et al. 2010) and short of a jump's countermovement (90-110°).
pub const DEEPEST: f32 = 0.18;

/// How far the heels rise about the toe tips on the toes, radians (20°):
/// the jump's [`jump::HEEL_RISE`] leaving the floor.
pub const TOES_HEEL: f32 = jump::HEEL_RISE;

/// Where a sneak carries each hand (the wrist) from its shoulder, as
/// fractions of the arm's length (upper arm plus forearm): ahead, below, and
/// in toward the middle. Out of the way of the knees, in front of the belly,
/// the elbow bent about 90°. Authored: no recording of a sneak's arms was to
/// hand.
pub const CARRIED_HAND: (f32, f32, f32) = (0.5, 0.42, 0.04);

/// Which way a carried arm's elbow points, (out to its side, back, down):
/// down under the shoulder, a little out and back, the forearm reaching
/// forward near level. Pointed out 0.7 with the hands carried 0.12 in, the
/// forearms ran 36° inward and the hands met over the belly.
pub const CARRIED_ELBOW: (f32, f32, f32) = (0.4, 0.3, 0.85);

/// How much further ahead and higher the right hand is carried than the
/// left, as fractions of the arm's length: the two are never mirror images,
/// whose sameness is the plainest tell of a puppet.
pub const LEAD_HAND: (f32, f32) = (0.06, 0.04);

/// How far a carried hand hangs from its wrist, radians (20°): loose, not
/// held out straight along the forearm.
pub const WRIST_DROP: f32 = 0.35;

/// How far a sneak's hands swing ahead and back walking, metres per m/s,
/// against the legs (each hand with the opposite foot): a little, the hands
/// moving rather than the arms swinging from the shoulder.
pub const HAND_SWING: f32 = 0.05;

/// How far up a swinging hand rises as it comes forward, of its swing.
pub const HAND_RISE: f32 = 0.3;

/// Carries the arms as a sneak does (`weight` 0-1, its depth): each hand
/// placed ahead of and below its shoulder ([`CARRIED_HAND`], [`LEAD_HAND`])
/// and `swings` (left, right) metres further ahead, rising as it comes; the
/// elbow toward [`CARRIED_ELBOW`]; the hand hanging [`WRIST_DROP`].
///
/// Each hand goes from where the pose has it toward its place by `weight`,
/// and its elbow from where it bends now, so at no weight the pose is left
/// as it is, and a sneak's arms come in as its crouch does.
///
/// The crouch's arms were both upper arms swung forward and both elbows
/// bent by one angle, and walking, a walk's swing held back: the two hands
/// side by side, fists straight out of the forearms, swinging fore and aft
/// from the shoulder in one plane. A puppet's.
pub fn carry_arms(pose: &mut LocalPose, rig: &RigGeometry, weight: f32, swings: [f32; 2]) {
    use super::armik::{solve_arm_toward_from, ArmChain};
    use super::rig::{delta_after_world_turn, forward_kinematics_on};
    use bevy::math::{Quat, Vec3};
    let weight = weight.clamp(0.0, 1.0);
    if weight <= 0.0 {
        return;
    }
    let (forward, up) = (rig.forward(), Vec3::Y);
    // One pass for both arms: solving one leaves the other's joints where
    // they were. Re-run for every step, carrying the arms added 26 µs to a
    // crouch posed (`anim_bench --gait crouch`); now 5.
    let at = forward_kinematics_on(pose, rig);
    for (i, chain, side) in [(0usize, ArmChain::LEFT, 1.0f32), (1, ArmChain::RIGHT, -1.0)] {
        let (shoulder, elbow, wrist) = (at[chain.shoulder], at[chain.elbow], at[chain.wrist]);
        let length = (elbow - shoulder).length() + (wrist - elbow).length();
        let out = rig.left() * side;
        let (lead, raise) = if i == 1 { LEAD_HAND } else { (0.0, 0.0) };
        let swing = swings[i];
        let carried = shoulder + forward * ((CARRIED_HAND.0 + lead) * length + swing) - up * ((CARRIED_HAND.1 - raise) * length - HAND_RISE * swing)
            - out * (CARRIED_HAND.2 * length);
        let target = wrist.lerp(carried, weight);
        // The elbow's side now, square to the shoulder-to-wrist line, toward
        // the carried one.
        let line = (wrist - shoulder).normalize_or_zero();
        let now = ((elbow - shoulder) - line * (elbow - shoulder).dot(line)).normalize_or_zero();
        let carried_pole = (out * CARRIED_ELBOW.0 - forward * CARRIED_ELBOW.1 - up * CARRIED_ELBOW.2).normalize();
        let pole = if now == Vec3::ZERO { carried_pole } else { now.lerp(carried_pole, weight) };
        let (placed_elbow, placed_wrist) = solve_arm_toward_from(pose, &at, chain, target, pole, rig);
        // The hand hung from its wrist: turned about the level line square
        // to the forearm, the way that tips it down.
        let forearm = (placed_wrist - placed_elbow).normalize_or_zero();
        let axis = forearm.cross(up).normalize_or_zero();
        if axis != Vec3::ZERO {
            pose.rotations[chain.wrist] = delta_after_world_turn(pose, rig, chain.wrist, Quat::from_axis_angle(axis, -WRIST_DROP * weight));
        }
    }
}

/// Each hand's swing ahead (left, right), metres, walking at `speed` m/s at
/// `cycle`: each with the opposite foot, at its forward peak just after that
/// foot's footfall, as the walk's arms (`gait`'s arm swing).
pub fn hand_swings(cycle: f32, speed: f32) -> [f32; 2] {
    const LAG: f32 = 0.02;
    let amplitude = HAND_SWING * speed.max(0.0);
    // The left hand with the right foot, down at 0.5; the right with the
    // left, down at 0.
    [0.5f32, 0.0].map(|footfall| amplitude * (std::f32::consts::TAU * (cycle - (footfall + LAG - 0.25))).sin())
}

/// The most the COM accelerates going down into a crouch or up out of it,
/// m/s²: a fifth of `g`, a calm, controlled crouch rather than a jump's
/// countermovement (which unloads the floor to `jump::LEAST_LOAD`, 0.57 g).
pub const CROUCH_ACCELERATION: f32 = 2.0;

/// The quickest any change of crouch is made, seconds: rising onto the
/// toes, or a small change of depth.
pub const QUICKEST: f32 = 0.4;

/// The fastest a sneak walks, m/s: a walk asked faster is held to it, and a
/// sneak never runs. Authored: a crouched walk is slow.
pub const FASTEST: f32 = 1.0;

/// How much of a walk's arm swing the deepest sneak holds back: the arms
/// are carried ahead of the body, not swung. In proportion to the depth.
pub const ARMS_HELD: f32 = 0.7;

/// A sneak's walk at `speed` m/s (at most [`FASTEST`]) on `rig`, from the
/// crouch `crouched` (posed by [`Footing::pose`], `depth` 0-1 of the deepest,
/// `toes` 0-1 onto them) against standing in `stood`: Winter's stride,
/// replayed with the thigh and knee as much further flexed as the crouch's
/// ([`super::gait::CrouchAngles`]), and the arms held.
///
/// Its stride is the walk's at the same speed, so every crouch walks its
/// feet along the same path over the floor: not crouched, it is the walk,
/// and a crouch changing while walking blends two of them without moving
/// the feet ([`SneakGait`]).
pub fn sneaking_on(speed: f32, crouched: &LocalPose, stood: &LocalPose, rig: &RigGeometry, depth: f32, toes: f32) -> super::gait::GaitParams {
    use super::gait::{leg_joints, sagittal_angles, CrouchAngles, GaitParams};
    let walk = GaitParams::walking_on(speed.clamp(0.0, FASTEST), rig);
    // Each leg's extra flexion over standing, their mean: the crouch is
    // posed square, so the two agree.
    let extra = |i: usize| {
        let joints = leg_joints([Bone::LeftFoot, Bone::RightFoot][i]);
        let (now, then) = (sagittal_angles(crouched, rig, joints), sagittal_angles(stood, rig, joints));
        (now[0] - then[0], now[2] - then[2])
    };
    let (left, right) = (extra(0), extra(1));
    GaitParams {
        crouch: CrouchAngles {
            thigh: 0.5 * (left.0 + right.0),
            knee: 0.5 * (left.1 + right.1),
            heel: TOES_HEEL * toes.clamp(0.0, 1.0),
        },
        arm_swing: walk.arm_swing * (1.0 - ARMS_HELD * depth.clamp(0.0, 1.0)),
        ..walk
    }
}

impl Footing {
    /// How deep `crouch` is, 0-1 of the deepest.
    pub fn depth_of(&self, crouch: Crouch) -> f32 {
        (crouch.drop / self.deepest).clamp(0.0, 1.0)
    }

    /// The crouched pose `crouch` walks from, and its walk at `speed`.
    pub fn walk(&self, crouch: Crouch, speed: f32, stood: &LocalPose, rig: &RigGeometry) -> (LocalPose, super::gait::GaitParams) {
        let crouched = self.pose(crouch, stood, rig);
        let params = sneaking_on(speed, &crouched, stood, rig, self.depth_of(crouch), crouch.toes.clamp(0.0, 1.0));
        (crouched, params)
    }
}

/// A sneak's walk while its crouch changes ([`Crouching`]): the walk of the
/// crouch it set off from and the one it is going to, at one clock, blended
/// by how far it has gone. Each is cached (`walk::walk_cycle`), so a change
/// builds two cycles at most, where one for each crouch passed through
/// would build one a frame. Still, the one walk.
#[derive(Debug, Clone)]
pub struct SneakGait {
    from: (LocalPose, super::gait::GaitParams),
    to: (LocalPose, super::gait::GaitParams),
    weight: f32,
    /// How deep each crouch is, 0-1, and the speed: the arms are carried
    /// ([`carry_arms`]) by the depth, the hands swinging by the speed.
    depths: (f32, f32),
    speed: f32,
}

impl SneakGait {
    /// The walk `crouching` is in at `speed`, from standing in `stood`.
    pub fn of(crouching: &Crouching, footing: &Footing, speed: f32, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let to = footing.walk(crouching.to, speed, stood, rig);
        let weight = crouching.gone();
        let from = if weight >= 1.0 { to } else { footing.walk(crouching.from, speed, stood, rig) };
        let depths = (footing.depth_of(crouching.from), footing.depth_of(crouching.to));
        Self { from, to, weight, depths, speed }
    }

    /// The pose at `cycle`: the two walks blended, and each leg then put
    /// where the two have it.
    ///
    /// Both walks put a foot at the same place over the floor, so the ankle
    /// from the hips, blended in proportion as the hips' height is, keeps it
    /// there; its world attitude is blended too. The legs' joints blended
    /// instead (a knee from 15° to 75°, the hips lerped) pressed the planted
    /// foot 52 mm into the floor half-way from standing to the deepest.
    pub fn pose(&self, cycle: f32, rig: &RigGeometry) -> LocalPose {
        use super::gait::walk_pose_on;
        use super::rig::{accumulate_world_rotations, delta_after_world_turn, offset_from};
        let to = walk_pose_on(cycle, &self.to.1, &self.to.0, rig);
        let w = self.weight;
        let mut pose = if w >= 1.0 {
            to
        } else {
            let from = walk_pose_on(cycle, &self.from.1, &self.from.0, rig);
            let mut pose = super::clip::blend(&from, &to, w);
            let (turned_from, turned_to) = (accumulate_world_rotations(&from, rig), accumulate_world_rotations(&to, rig));
            for ankle in [Bone::LeftFoot, Bone::RightFoot] {
                let target = offset_from(&from, rig, Bone::Hips, ankle).lerp(offset_from(&to, rig, Bone::Hips, ankle), w);
                super::stance::place_ankle(&mut pose, rig, ankle, target);
                let wanted = turned_from[ankle].slerp(turned_to[ankle], w);
                let now = accumulate_world_rotations(&pose, rig)[ankle];
                pose.rotations[ankle] = delta_after_world_turn(&pose, rig, ankle, wanted * now.inverse());
            }
            pose
        };
        // The arms carried again over the walk's, the hands swinging.
        let depth = self.depths.0 + (self.depths.1 - self.depths.0) * w.min(1.0);
        carry_arms(&mut pose, rig, depth, hand_swings(cycle, self.speed));
        pose
    }

    /// The walk it is going to: its params, and the crouch it walks from.
    pub fn target(&self) -> &(LocalPose, super::gait::GaitParams) {
        &self.to
    }
}

/// A sneak asked for: how deep to crouch, 0 (standing) to 1 ([`DEEPEST`]),
/// and whether on the toes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sneak {
    pub crouch: f32,
    pub on_toes: bool,
}

impl Sneak {
    /// Standing tall, flat-footed: no sneak.
    pub const STANDING: Self = Self { crouch: 0.0, on_toes: false };

    /// Whether it asks for anything but standing.
    pub fn is_sneaking(&self) -> bool {
        self.crouch > 0.0 || self.on_toes
    }

    /// The crouch it asks for: how far the COM is lowered, metres, on a leg
    /// `leg` long, and how far onto the toes, 0-1.
    pub fn crouch_on(&self, leg: f32) -> Crouch {
        Crouch { drop: self.crouch.clamp(0.0, 1.0) * DEEPEST * leg, toes: if self.on_toes { 1.0 } else { 0.0 } }
    }
}

/// A crouch: how far the COM is lowered below standing, metres, and how far
/// onto the toes, 0 (flat) to 1 (the heels risen [`TOES_HEEL`]).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Crouch {
    pub drop: f32,
    pub toes: f32,
}

impl Crouch {
    /// Whether it is standing, flat-footed.
    pub fn is_standing(&self) -> bool {
        self.drop <= 0.0 && self.toes <= 0.0
    }
}

/// What a crouch is posed from: the feet where they stood, worked out once
/// from the standing pose.
#[derive(Debug, Clone, Copy)]
pub struct Footing {
    feet: Feet,
    /// The COM standing, in the standing hips' frame: along the rig's
    /// forward, and up.
    stand: (f32, f32),
    /// The balls of the feet along the rig's forward, their middle.
    balls: f32,
    /// How far the ankles rise with the heels up on the toes, metres.
    rise: f32,
    /// The deepest crouch's drop, metres.
    deepest: f32,
}

impl Footing {
    /// The feet as they stand in `stood`.
    pub fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let feet = Feet::of(stood, rig);
        let com = com_of(stood, stood, rig);
        let forward = rig.forward();
        let balls = [Bone::LeftFoot, Bone::RightFoot].map(|ankle| Sole::of(rig, ankle).points(stood, rig)[1].dot(forward));
        let rise = [0, 1].map(|i| feet.ankle(i, TOES_HEEL).y - feet.ankle(i, 0.0).y);
        Self {
            feet,
            stand: (com.dot(forward), com.y),
            balls: 0.5 * (balls[0] + balls[1]),
            rise: 0.5 * (rise[0] + rise[1]),
            deepest: DEEPEST * leg_length_of(rig),
        }
    }

    /// How far the COM rises going up onto the toes, metres: the ankles'
    /// rise.
    pub fn rise(&self) -> f32 {
        self.rise
    }

    /// Where `crouch` puts the COM, in the standing hips' frame: along the
    /// rig's forward, and up.
    pub fn com(&self, crouch: Crouch) -> (f32, f32) {
        let toes = crouch.toes.clamp(0.0, 1.0);
        (self.stand.0 + toes * (self.balls - self.stand.0), self.stand.1 + toes * self.rise - crouch.drop)
    }

    /// `stood` crouched as `crouch` asks. Standing, `stood` itself.
    pub fn pose(&self, crouch: Crouch, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        if crouch.is_standing() {
            return *stood;
        }
        let depth = (crouch.drop / self.deepest).max(0.0);
        let lean = (jump::LEAN_PER_DEPTH * crouch.drop.max(0.0)).min(jump::MOST_LEAN);
        // The arms carried before the pelvis is solved, so the COM is where
        // the crouch asks with them where they are.
        let mut upper = jump::upper(stood, rig, lean, (0.0, 0.0));
        carry_arms(&mut upper, rig, depth, [0.0; 2]);
        let heel = TOES_HEEL * crouch.toes.clamp(0.0, 1.0);
        let (ahead, height) = self.com(crouch);
        self.feet.solved(&upper, stood, rig, ahead, Aim::Height { height, knee: jump::KNEE_AT_TAKEOFF, heel }, 0.0, heel)
    }
}

/// A crouch under way: eased from where it was to where it is asked, at
/// rest at the end, and no faster than [`CROUCH_ACCELERATION`] allows.
/// Asked another crouch on the way, it goes on from where it is at the rate
/// it is going.
#[derive(Debug, Clone, Copy, Default)]
pub struct Crouching {
    from: Crouch,
    /// Its rate as it set off from `from`, per second.
    rate: Crouch,
    to: Crouch,
    seconds: f32,
    t: f32,
}

impl Crouching {
    /// Where it is now.
    pub fn now(&self) -> Crouch {
        self.at(self.t)
    }

    /// Whether it has stood up, flat-footed, and is still.
    pub fn is_standing(&self) -> bool {
        self.is_still() && self.to.is_standing()
    }

    /// Whether it has got where it was asked.
    pub fn is_still(&self) -> bool {
        self.t >= self.seconds
    }

    /// Where it is going.
    pub fn target(&self) -> Crouch {
        self.to
    }

    /// How far it has gone from where it set off toward where it is going,
    /// 0-1: along the COM's height (the drop, less the toes' rise at a
    /// nominal 7 cm), else the toes alone.
    pub fn gone(&self) -> f32 {
        if self.is_still() {
            return 1.0;
        }
        let along = |c: Crouch| (c.drop - 0.07 * c.toes, c.toes);
        let (from, now, to) = (along(self.from), along(self.now()), along(self.to));
        let fraction = if (to.0 - from.0).abs() > 1.0e-4 { (now.0 - from.0) / (to.0 - from.0) } else { (now.1 - from.1) / (to.1 - from.1) };
        if fraction.is_finite() { fraction.clamp(0.0, 1.0) } else { 1.0 }
    }

    /// Asks it for `to`: from where it is now, at the rate it is going. The
    /// COM rises `rise` metres going up onto the toes ([`Footing::rise`]):
    /// timed by the crouch's depth alone, a deep flat crouch changed to a
    /// shallower one on the toes rose 0.15 m where it had planned 0.08, and
    /// the pelvis accelerated at 3.3 m/s².
    pub fn ask(&mut self, to: Crouch, rise: f32) {
        if to == self.to {
            return;
        }
        let (from, rate) = (self.now(), self.rate_at(self.t));
        // The COM's way down, and its rate.
        let span = (to.drop - rise * to.toes) - (from.drop - rise * from.toes);
        let speed = rate.drop - rise * rate.toes;
        // The cubic's steepest acceleration is at an end: 6·span/T² less
        // 4·rate/T at the start, and 2·rate/T less 6·span/T² at the end.
        let fits = |seconds: f32| {
            let a = |start: f32, end: f32| (6.0 * span - start * seconds).abs().max((6.0 * span - end * seconds).abs()) / (seconds * seconds);
            a(4.0 * speed, 2.0 * speed) <= CROUCH_ACCELERATION
        };
        let mut seconds = QUICKEST.max((6.0 * span.abs() / CROUCH_ACCELERATION).sqrt());
        while !fits(seconds) && seconds < 10.0 {
            seconds *= 1.05;
        }
        *self = Self { from, rate, to, seconds, t: 0.0 };
    }

    /// Moves it on by `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt).min(self.seconds);
    }

    fn at(&self, t: f32) -> Crouch {
        if self.seconds <= 0.0 {
            return self.to;
        }
        let u = t / self.seconds;
        let eased = |from: f32, rate: f32, to: f32| hermite(from, to, rate * self.seconds, 0.0, u);
        Crouch { drop: eased(self.from.drop, self.rate.drop, self.to.drop), toes: eased(self.from.toes, self.rate.toes, self.to.toes) }
    }

    fn rate_at(&self, t: f32) -> Crouch {
        if t >= self.seconds || self.seconds <= 0.0 {
            return Crouch::default();
        }
        let h = 1.0e-3 * self.seconds;
        let (before, after) = (self.at((t - h).max(0.0)), self.at((t + h).min(self.seconds)));
        let span = (t + h).min(self.seconds) - (t - h).max(0.0);
        Crouch { drop: (after.drop - before.drop) / span, toes: (after.toes - before.toes) / span }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::math::Vec3;
    use crate::character::anim::rig::{forward_kinematics_on, Side};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    fn crouches(rig: &RigGeometry) -> Vec<Crouch> {
        let leg = leg_length_of(rig);
        let mut all = Vec::new();
        for on_toes in [false, true] {
            for k in 0..=10 {
                all.push(Sneak { crouch: k as f32 / 10.0, on_toes }.crouch_on(leg));
            }
        }
        all
    }

    /// The knee's flexion, degrees, and the shank's tilt forward from
    /// vertical, degrees, of the left leg.
    fn knee_and_shank(pose: &LocalPose, rig: &RigGeometry) -> (f32, f32) {
        let p = forward_kinematics_on(pose, rig);
        let (thigh, shank) = (p[Bone::LeftLeg] - p[Bone::LeftUpLeg], p[Bone::LeftFoot] - p[Bone::LeftLeg]);
        let flexion = thigh.angle_between(shank).to_degrees();
        let tilt = (-shank).angle_between(Vec3::Y).to_degrees() * (-shank).dot(rig.forward()).signum();
        (flexion, tilt)
    }

    #[test]
    fn a_crouch_carries_its_com_where_asked_over_its_feet() {
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        for crouch in crouches(&rig) {
            let pose = footing.pose(crouch, &stood, &rig);
            let com = com_of(&pose, &stood, &rig);
            let (ahead, height) = footing.com(crouch);
            let miss = Vec3::new(com.dot(rig.forward()) - ahead, com.y - height, 0.0).length();
            assert!(miss < 1.0e-3, "{crouch:?}: the COM is {} mm off where asked", miss * 1e3);
        }
    }

    #[test]
    fn standing_it_is_the_standing_pose() {
        let (stood, rig) = real_stood();
        let pose = Footing::of(&stood, &rig).pose(Crouch::default(), &stood, &rig);
        assert_eq!(pose.root_translation, stood.root_translation);
        for bone in Bone::ALL {
            assert_eq!(pose.rotations[bone], stood.rotations[bone], "{bone:?}");
        }
    }

    #[test]
    fn crouched_the_feet_stay_where_they_stood_and_the_heels_rise_on_the_toes() {
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        let soles = [Bone::LeftFoot, Bone::RightFoot].map(|ankle| Sole::of(&rig, ankle));
        let points = |pose: &LocalPose| [0, 1].map(|i| soles[i].points(pose, &rig).map(|p| p + (pose.root_translation - stood.root_translation)));
        let standing = points(&stood);
        for crouch in crouches(&rig) {
            let now = points(&footing.pose(crouch, &stood, &rig));
            for i in 0..2 {
                let tip = (now[i][2] - standing[i][2]).length();
                assert!(tip < 1.0e-3, "{crouch:?}: foot {i}'s tip moved {} mm", tip * 1e3);
                let length = (standing[i][2] - standing[i][0]).length();
                let heel = now[i][0].y - standing[i][0].y;
                let expected = length * (TOES_HEEL * crouch.toes).sin();
                assert!((heel - expected).abs() < 3.0e-3, "{crouch:?}: foot {i}'s heel {} mm up, not {}", heel * 1e3, expected * 1e3);
            }
        }
    }

    /// The knees fold forward all through, and the deepest sneak folds them
    /// 60-80°: Steele et al. 2010 class crouch gait by the least knee
    /// flexion in stance, mild 20-35°, moderate 35-50°, severe past 50°.
    /// Flat-footed, the shank leans no further forward than an ankle bends
    /// with the body's weight on it (a weight-bearing lunge, about 40°).
    #[test]
    fn a_crouch_folds_its_knees_forward_as_far_as_a_deep_sneak() {
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        for crouch in crouches(&rig) {
            let pose = footing.pose(crouch, &stood, &rig);
            for side in [Side::Left, Side::Right] {
                let fold = rig.knee_fold_direction(&pose, side);
                assert!(fold < 0.0, "{crouch:?}: the {side:?} knee folds backward ({fold})");
            }
            let (_, tilt) = knee_and_shank(&pose, &rig);
            assert!(crouch.toes > 0.0 || tilt < 40.0, "{crouch:?}: the shank leans {tilt}°");
        }
        let deepest = Sneak { crouch: 1.0, on_toes: false }.crouch_on(leg_length_of(&rig));
        let (flexion, tilt) = knee_and_shank(&footing.pose(deepest, &stood, &rig), &rig);
        assert!((60.0..80.0).contains(&flexion), "the deepest sneak folds the knees {flexion}°, the shanks leaning {tilt}°");
    }

    /// A sneak's walk at `speed`, `crouch` deep and `on_toes`: its params,
    /// the crouch it walks from, and the pose at a phase.
    fn sneak_walk(stood: &LocalPose, rig: &RigGeometry, speed: f32, crouch: f32, on_toes: bool) -> (super::super::gait::GaitParams, LocalPose) {
        let footing = Footing::of(stood, rig);
        let asked = Sneak { crouch, on_toes }.crouch_on(leg_length_of(rig));
        let crouched = footing.pose(asked, stood, rig);
        (sneaking_on(speed, &crouched, stood, rig, footing.depth_of(asked), asked.toes), crouched)
    }

    const SNEAKS: [(f32, bool); 4] = [(0.5, false), (1.0, false), (0.5, true), (1.0, true)];
    const SPEEDS: [f32; 3] = [0.4, 0.7, 1.0];

    /// Every phase of a sneak's walk: the knees fold forward, and through
    /// stance the least knee flexion is a crouch gait's (Steele et al.
    /// 2010: past 50° severe), the hips riding lower than the walk's.
    #[test]
    fn a_sneak_walks_on_bent_knees_lower_than_a_walk() {
        use crate::character::anim::gait::{leg_phase, walk_pose_on, GaitParams, SHORT_STEPS};
        let (stood, rig) = real_stood();
        for (crouch, on_toes) in SNEAKS {
            for speed in SPEEDS {
                let (params, crouched) = sneak_walk(&stood, &rig, speed, crouch, on_toes);
                let walk = GaitParams::walking_with_steps(speed, leg_length_of(&rig), SHORT_STEPS);
                let (mut least, mut lower) = (f32::MAX, 0.0f32);
                for i in 0..64 {
                    let phase = i as f32 / 64.0;
                    let pose = walk_pose_on(phase, &params, &crouched, &rig);
                    for side in [Side::Left, Side::Right] {
                        let fold = rig.knee_fold_direction(&pose, side);
                        assert!(fold < 0.0, "{crouch} {on_toes} at {speed} m/s, phase {phase}: the {side:?} knee folds backward");
                    }
                    if leg_phase(phase, params.duty_factor).is_stance() {
                        least = least.min(knee_and_shank(&pose, &rig).0);
                    }
                    lower += (walk_pose_on(phase, &walk, &stood, &rig).root_translation.y - pose.root_translation.y) / 64.0;
                }
                // The crouch's own hips below standing's.
                let drop = stood.root_translation.y - crouched.root_translation.y;
                let floor = if crouch >= 1.0 { 50.0 } else { 30.0 };
                assert!(least > floor, "{crouch} {on_toes} at {speed} m/s: the stance knee straightens to {least}°");
                // Flat, within 0.2 mm measured. On the toes 10-21 mm lower,
                // more the faster: the walk it is measured against rides up
                // over its heel, which a toe walk never comes down on.
                let within = if on_toes { (-0.002, 0.025) } else { (-0.002, 0.002) };
                assert!((within.0..within.1).contains(&(lower - drop)), "{crouch} {on_toes} at {speed} m/s: the hips ride {lower} m below the walk's, the crouch's COM {drop}");
            }
        }
    }

    /// Through single support the planted foot is on the floor, as a walk's
    /// is (`locomotion`'s `a_planted_foot_stays_on_the_ground`: never floating, pressed in
    /// no more than the foot IK lifts out); through swing it clears it.
    /// On the toes, the heel stays up through stance.
    #[test]
    fn a_sneaks_feet_stay_on_the_floor_and_swing_clear_of_it() {
        use crate::character::anim::foot::lowest;
        use crate::character::anim::gait::{leg_phase, walk_pose_on, LegPhase};
        let (stood, rig) = real_stood();
        let sole = Sole::of(&rig, Bone::LeftFoot);
        let ground = lowest(&sole.points(&stood, &rig));
        for (crouch, on_toes) in [(0.0, false)].into_iter().chain(SNEAKS) {
            for speed in SPEEDS {
                let (params, crouched) = sneak_walk(&stood, &rig, speed, crouch, on_toes);
                let (mut floating, mut pressed, mut dipped, mut mid, mut flattest) = (0.0f32, 0.0f32, f32::MAX, f32::MAX, f32::MAX);
                for i in 0..200 {
                    let phase = i as f32 / 200.0;
                    let pose = walk_pose_on(phase, &params, &crouched, &rig);
                    let risen = pose.root_translation.y - stood.root_translation.y;
                    let points = sole.points(&pose, &rig);
                    let height = risen + lowest(&points) - ground;
                    match leg_phase(phase, params.duty_factor) {
                        LegPhase::Stance { progress } => {
                            if (0.25..=0.75).contains(&progress) {
                                floating = floating.max(height);
                                pressed = pressed.max(-height);
                            }
                            if (0.1..=0.9).contains(&progress) {
                                flattest = flattest.min(points[0].y - points[2].y);
                            }
                        }
                        LegPhase::Swing { progress } => {
                            dipped = dipped.min(height);
                            if (0.3..0.7).contains(&progress) {
                                mid = mid.min(height);
                            }
                        }
                    }
                }
                let what = format!("{crouch} {on_toes} at {speed} m/s");
                assert!(floating < 0.002, "{what}: a planted foot floated {} mm", floating * 1e3);
                // The walk's own (the uncrouched case, 14-15 mm): the sneak's
                // feet go as the walk's do under its hips.
                assert!(pressed < 0.016, "{what}: a planted foot sank {} mm", pressed * 1e3);
                assert!(dipped > -0.002, "{what}: a swinging foot dipped {} mm into the floor", -dipped * 1e3);
                assert!(mid > 0.01, "{what}: mid-swing clears the floor by {} mm", mid * 1e3);
                if on_toes {
                    assert!(flattest > 0.04, "{what}: the heel came down to {} mm over the tip", flattest * 1e3);
                }
            }
        }
    }

    /// On the toes the tip is the foot's contact from touchdown to toe-off,
    /// and stays where it landed in the world: its motion under the hips
    /// and the body's travel (`locomotion::root_velocity_of`) cancel, frame
    /// by frame at 60 Hz. Before its whole stance counted toward the thigh
    /// correction (`walk::WalkCycle::refine`), it landed moving 8.9 mm a
    /// frame at 0.8 m/s, the recorded heel's roll carrying its ankle on.
    #[test]
    fn on_the_toes_a_planted_tip_stays_where_it_landed() {
        use crate::character::anim::gait::{leg_phase, walk_pose_on, LegPhase};
        use crate::character::anim::locomotion::{distance_per_cycle, root_velocity_of};
        let (stood, rig) = real_stood();
        let sole = Sole::of(&rig, Bone::LeftFoot);
        for crouch in [0.5, 1.0] {
            for speed in SPEEDS {
                let (params, crouched) = sneak_walk(&stood, &rig, speed, crouch, true);
                let at = |p: f32| walk_pose_on(p, &params, &crouched, &rig);
                let cycles_a_frame = speed / distance_per_cycle(&params, &crouched, &rig) / 60.0;
                let (n, mut worst, mut worst_at) = (600, 0.0f32, 0.0);
                for i in 0..n {
                    let (from, to) = (i as f32 / n as f32, (i + 1) as f32 / n as f32);
                    // Not as it leaves the floor at toe-off, the last 3 %.
                    let LegPhase::Stance { progress } = leg_phase(from, params.duty_factor) else { continue };
                    if progress > 0.97 {
                        continue;
                    }
                    let under = (sole.points(&at(to), &rig)[2] - sole.points(&at(from), &rig)[2]).dot(rig.forward());
                    let body = root_velocity_of(0.5 * (from + to), 1.0, &params, &at, &rig).dot(rig.forward()) * (to - from);
                    let per_frame = (under + body).abs() / (to - from) * cycles_a_frame;
                    if per_frame > worst {
                        (worst, worst_at) = (per_frame, progress);
                    }
                }
                // Measured at most 0.11 mm.
                assert!(worst < 3.0e-4, "{crouch} at {speed} m/s: the planted tip moves {} mm a frame at {worst_at} through stance", worst * 1e3);
            }
        }
    }

    /// Shuffling aside crouched (`shuffle`, posed on the crouch): a foot
    /// down stays on the floor, both down move alike under the body (or one
    /// slides), and the hips ride as far below an upright shuffle's as the
    /// crouch's are below standing.
    #[test]
    fn a_sneak_shuffles_aside_crouched_its_feet_down_on_the_floor() {
        use crate::character::anim::foot::lowest;
        use crate::character::anim::gait::{leg_phase, walk_pose_on};
        use crate::character::anim::rig::offset_from;
        use crate::character::anim::shuffle::shuffling;
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        let soles = [Bone::LeftFoot, Bone::RightFoot].map(|ankle| Sole::of(&rig, ankle));
        let grounds = [0, 1].map(|leg| lowest(&soles[leg].points(&stood, &rig)));
        for (crouch, on_toes) in SNEAKS {
            let crouched = footing.pose(Sneak { crouch, on_toes }.crouch_on(leg_length_of(&rig)), &stood, &rig);
            let drop = stood.root_translation.y - crouched.root_translation.y;
            for (speed, toward, ahead) in [(0.4, 1.0, 0.0), (0.6, -1.0, 0.0), (0.5, 1.0, 0.6)] {
                let params = shuffling(speed, toward, ahead);
                let duty = params.duty_factor;
                let what = format!("{crouch} {on_toes}, {speed} m/s toward {toward} ahead {ahead}");
                let (frames, mut lower, mut previous) = (240, 0.0f32, None::<[Vec3; 2]>);
                for i in 0..frames {
                    let cycle = i as f32 / frames as f32;
                    let pose = walk_pose_on(cycle, &params, &crouched, &rig);
                    lower += (walk_pose_on(cycle, &params, &stood, &rig).root_translation.y - pose.root_translation.y) / frames as f32;
                    let risen = pose.root_translation.y - stood.root_translation.y;
                    let toes = [Bone::LeftToeBase, Bone::RightToeBase].map(|bone| offset_from(&pose, &rig, Bone::Hips, bone));
                    for leg in 0..2 {
                        if leg_phase(cycle + 0.5 * leg as f32, duty).is_stance() {
                            let height = risen + lowest(&soles[leg].points(&pose, &rig)) - grounds[leg];
                            assert!(height.abs() < 2.0e-3, "{what}: at {cycle:.3} foot {leg} down is {:.1} mm off the floor", height * 1e3);
                        }
                    }
                    let both = |at: f32| leg_phase(at, duty).is_stance() && leg_phase(at + 0.5, duty).is_stance();
                    if let Some([was_left, was_right]) = previous
                        && both(cycle)
                        && both(cycle - 1.0 / frames as f32)
                    {
                        let apart = ((toes[0] - was_left) - (toes[1] - was_right)) * Vec3::new(1.0, 0.0, 1.0);
                        assert!(apart.length() < 5.0e-4, "{what}: at {cycle:.3} the feet down moved {:.2} mm apart", apart.length() * 1e3);
                    }
                    previous = Some(toes);
                }
                assert!((lower - drop).abs() < 0.01, "{what}: the hips ride {lower:.3} m below an upright shuffle's, the crouch's {drop:.3}");
            }
        }
    }

    /// The deepest sneak carries each hand ahead of and below its shoulder,
    /// its elbow under the shoulder and out past the hand, bent near a right
    /// angle, the forearm reaching forward and only a little in; and the two
    /// arms are not mirror images. The arms swung forward and the elbows
    /// bent by one angle each had the hands side by side, straight out of
    /// the forearms; carried 0.12 of the arm in with the elbows 0.7 out, the
    /// forearms ran 36° in and the hands met over the belly.
    #[test]
    fn a_sneak_carries_its_hands_ahead_the_elbows_bent_under_and_out() {
        use crate::character::anim::rig::forward_kinematics_on;
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        let (forward, left) = (rig.forward(), rig.left());
        for on_toes in [false, true] {
            let pose = footing.pose(Sneak { crouch: 1.0, on_toes }.crouch_on(leg_length_of(&rig)), &stood, &rig);
            let at = forward_kinematics_on(&pose, &rig);
            let mut ahead = [0.0f32; 2];
            for (i, (shoulder, elbow, wrist), side) in [(0usize, (Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand), 1.0f32), (1, (Bone::RightArm, Bone::RightForeArm, Bone::RightHand), -1.0)] {
                let (s, e, w) = (at[shoulder], at[elbow], at[wrist]);
                let out = left * side;
                let what = format!("on the toes {on_toes}, {shoulder:?}");
                ahead[i] = (w - s).dot(forward);
                assert!(ahead[i] > 0.15 && (s - w).y > 0.1, "{what}: the hand is {:.3} m ahead and {:.3} below the shoulder", ahead[i], (s - w).y);
                assert!((s - e).y > 0.15, "{what}: the elbow is only {:.3} m below the shoulder", (s - e).y);
                assert!((e - w).dot(out) > 0.02, "{what}: the elbow is {:.3} m out past the hand", (e - w).dot(out));
                let bend = 180.0 - (s - e).angle_between(w - e).to_degrees();
                assert!((60.0..110.0).contains(&bend), "{what}: the elbow bends {bend}°");
                let forearm = w - e;
                let inward = (-forearm.dot(out)).atan2(forearm.dot(forward)).to_degrees();
                assert!((0.0..30.0).contains(&inward), "{what}: the forearm runs {inward}° in");
            }
            assert!(ahead[1] - ahead[0] > 0.01, "on the toes {on_toes}: the hands are carried alike, {ahead:?}");
        }
    }

    /// At no weight [`carry_arms`] leaves the pose as it is, and toward no
    /// weight it moves the arms toward nothing: a sneak's arms come in with
    /// its crouch, with no jump as it begins. Steeply, though: the standing
    /// arm is within 0.4 mm of straight, and bringing its wrist in a few
    /// millimetres swings the elbow out by more (measured 0.3 mm at 1e-4,
    /// 2.7 at 1e-3, 25 at 0.02). Solved with the arm IK's usual 5 mm of
    /// softening, inside which the standing arm sits, the elbow jumped 12 mm
    /// at 1e-4 (`armik::solve_arm_toward`).
    #[test]
    fn carried_by_nothing_the_arms_are_left_as_they_are() {
        use crate::character::anim::rig::forward_kinematics_on;
        let (stood, rig) = real_stood();
        let mut untouched = stood;
        carry_arms(&mut untouched, &rig, 0.0, [0.0; 2]);
        assert_eq!(untouched.rotations.0, stood.rotations.0);
        let before = forward_kinematics_on(&stood, &rig);
        for (weight, most) in [(1.0e-4, 1.0e-3), (1.0e-3, 5.0e-3)] {
            let mut barely = stood;
            carry_arms(&mut barely, &rig, weight, [0.0; 2]);
            let after = forward_kinematics_on(&barely, &rig);
            for bone in [Bone::LeftForeArm, Bone::LeftHand, Bone::RightForeArm, Bone::RightHand] {
                let moved = (after[bone] - before[bone]).length();
                assert!(moved < most, "carried by {weight}, the {bone:?} moved {moved} m");
            }
        }
    }

    /// Walking crouched, each hand swings ahead and back against the other
    /// by about [`HAND_SWING`] a m/s, the arms carried, not swung from the
    /// shoulder.
    #[test]
    fn a_sneaks_hands_swing_against_each_other() {
        use crate::character::anim::rig::forward_kinematics_on;
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        let speed = 0.8;
        let mut crouching = Crouching::default();
        crouching.ask(Sneak { crouch: 1.0, on_toes: false }.crouch_on(leg_length_of(&rig)), footing.rise());
        crouching.advance(10.0);
        let gait = SneakGait::of(&crouching, &footing, speed, &stood, &rig);
        let ahead: Vec<[f32; 2]> = (0..64)
            .map(|i| {
                let at = forward_kinematics_on(&gait.pose(i as f32 / 64.0, &rig), &rig);
                [(Bone::LeftArm, Bone::LeftHand), (Bone::RightArm, Bone::RightHand)].map(|(s, w)| (at[w] - at[s]).dot(rig.forward()))
            })
            .collect();
        let mean = |k: usize| ahead.iter().map(|a| a[k]).sum::<f32>() / 64.0;
        let (ml, mr) = (mean(0), mean(1));
        let (mut lr, mut ll, mut rr) = (0.0, 0.0, 0.0);
        for a in &ahead {
            lr += (a[0] - ml) * (a[1] - mr);
            ll += (a[0] - ml).powi(2);
            rr += (a[1] - mr).powi(2);
        }
        let correlation = lr / (ll * rr).sqrt();
        assert!(correlation < -0.8, "the hands swing together, correlation {correlation}");
        for k in 0..2 {
            let range = ahead.iter().map(|a| a[k]).fold(f32::MIN, f32::max) - ahead.iter().map(|a| a[k]).fold(f32::MAX, f32::min);
            let asked = 2.0 * HAND_SWING * speed;
            assert!((0.6 * asked..1.6 * asked).contains(&range), "hand {k} swings {range:.3} m, asked {asked:.3}");
        }
    }

    /// Not crouched, a sneak's walk is the walk.
    #[test]
    fn uncrouched_a_sneak_walks_as_a_walk() {
        use crate::character::anim::gait::GaitParams;
        let (stood, rig) = real_stood();
        let (params, crouched) = sneak_walk(&stood, &rig, 0.8, 0.0, false);
        assert_eq!(params, GaitParams::walking_on(0.8, &rig));
        assert_eq!(crouched.root_translation, stood.root_translation);
    }

    /// Changing its crouch on the move, part way between two walks
    /// ([`SneakGait`]): through single support the planted foot stays on
    /// the floor as the walk's does (pressed in no more than its 16 mm,
    /// never floating). The legs' joints blended instead of their ankles,
    /// it was pressed 52 mm in half-way from standing to the deepest.
    ///
    /// Not its slip: root motion is read off the same blended pose, so the
    /// planted contact holds still by construction.
    #[test]
    fn a_crouch_changing_on_the_move_keeps_the_planted_foot_on_the_floor() {
        use crate::character::anim::foot::lowest;
        use crate::character::anim::gait::{leg_phase, LegPhase};
        let (stood, rig) = real_stood();
        let footing = Footing::of(&stood, &rig);
        let leg = leg_length_of(&rig);
        let sole = Sole::of(&rig, Bone::LeftFoot);
        let ground = lowest(&sole.points(&stood, &rig));
        let speed = 0.8;
        let changes = [
            (Sneak::STANDING, Sneak { crouch: 1.0, on_toes: false }),
            (Sneak { crouch: 1.0, on_toes: false }, Sneak { crouch: 0.6, on_toes: true }),
            (Sneak::STANDING, Sneak { crouch: 1.0, on_toes: true }),
        ];
        for (from, to) in changes {
            for weight in [0.25, 0.5, 0.75] {
                let gait = SneakGait {
                    from: footing.walk(from.crouch_on(leg), speed, &stood, &rig),
                    to: footing.walk(to.crouch_on(leg), speed, &stood, &rig),
                    weight,
                    depths: (from.crouch, to.crouch),
                    speed,
                };
                let duty = gait.to.1.duty_factor;
                let (mut floating, mut pressed) = (0.0f32, 0.0f32);
                for i in 0..200 {
                    let phase = i as f32 / 200.0;
                    let LegPhase::Stance { progress } = leg_phase(phase, duty) else { continue };
                    if !(0.25..=0.75).contains(&progress) {
                        continue;
                    }
                    let pose = gait.pose(phase, &rig);
                    let height = pose.root_translation.y - stood.root_translation.y + lowest(&sole.points(&pose, &rig)) - ground;
                    floating = floating.max(height);
                    pressed = pressed.max(-height);
                }
                let what = format!("{from:?} to {to:?}, {weight} of the way");
                assert!(floating < 0.002, "{what}: the planted foot floats {} mm", floating * 1e3);
                assert!(pressed < 0.016, "{what}: the planted foot is pressed {} mm into the floor", pressed * 1e3);
            }
        }
    }

    /// Down at rest, up at rest, and asked again half-way it turns without
    /// a jump in where it is or how fast it goes, never accelerating harder
    /// than [`CROUCH_ACCELERATION`].
    #[test]
    fn a_crouch_eases_at_rest_and_turns_smoothly_when_asked_again() {
        let (dt, rise) = (1.0 / 240.0, 0.07);
        let mut crouching = Crouching::default();
        // Down deep and flat, then on the toes and half as deep: the COM
        // rises by both. Then, half-way back up, asked down again.
        let mut trace = vec![crouching.now()];
        for (to, turn_at) in [(Crouch { drop: 0.16, toes: 0.0 }, None), (Crouch { drop: 0.08, toes: 1.0 }, Some(0.5)), (Crouch::default(), None)] {
            crouching.ask(to, rise);
            let mut turned = turn_at.is_none();
            while !(turned && crouching.is_still()) {
                crouching.advance(dt);
                if !turned && crouching.now().toes > turn_at.unwrap_or(0.0) {
                    crouching.ask(Crouch { drop: 0.16, toes: 0.0 }, rise);
                    turned = true;
                }
                trace.push(crouching.now());
                assert!(trace.len() < 10_000, "never got there");
            }
        }
        assert!(crouching.is_standing());
        let drops: Vec<f32> = trace.iter().map(|c| c.drop - rise * c.toes).collect();
        let speeds: Vec<f32> = drops.windows(2).map(|w| (w[1] - w[0]) / dt).collect();
        let accelerations: Vec<f32> = speeds.windows(2).map(|w| (w[1] - w[0]) / dt).collect();
        let hardest = accelerations.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
        assert!(hardest <= CROUCH_ACCELERATION * 1.05, "accelerated at {hardest} m/s²");
        assert!(speeds[0].abs() < 0.01 && speeds.last().unwrap().abs() < 0.01, "not at rest at the ends: {} and {} m/s", speeds[0], speeds.last().unwrap());
        // A cubic at rest at its ends goes at most 1.5 times its mean rate.
        let steepest_toes = trace.windows(2).map(|w| (w[1].toes - w[0].toes).abs() / dt).fold(0.0f32, f32::max);
        assert!(steepest_toes <= 1.5 / QUICKEST * 1.01, "onto the toes at {steepest_toes}/s");
        assert!(trace.iter().all(|c| (-0.05..=1.05).contains(&c.toes)), "the toes overshoot");
    }
}
