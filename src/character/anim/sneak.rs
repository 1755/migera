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

/// The arms at the deepest crouch, as (swing, elbow) on top of standing,
/// radians: the upper arms a little forward, the elbows bent, the hands
/// held in front of the thighs out of the way of the knees. Authored; in
/// proportion to the depth above it.
pub const CROUCHED_ARMS: (f32, f32) = (0.3, 0.8);

/// The most the COM accelerates going down into a crouch or up out of it,
/// m/s²: a fifth of `g`, a calm, controlled crouch rather than a jump's
/// countermovement (which unloads the floor to `jump::LEAST_LOAD`, 0.57 g).
pub const CROUCH_ACCELERATION: f32 = 2.0;

/// The quickest any change of crouch is made, seconds: rising onto the
/// toes, or a small change of depth.
pub const QUICKEST: f32 = 0.4;

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
        let upper = jump::upper(stood, rig, lean, (CROUCHED_ARMS.0 * depth, CROUCHED_ARMS.1 * depth));
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
