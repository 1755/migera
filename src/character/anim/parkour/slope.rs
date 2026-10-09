//! Sliding down a slope too steep to walk: step 18 of the parkour steps
//! beyond the first ten (slides and long falls). A roof is slid down low on
//! the feet, a hand trailing on the slope behind; a steep face (a
//! pyramid's) on the feet, the body leant back, the arms out
//! ([`SlopeKind`]).
//!
//! The body slides as a block on its feet: along the slope its speed
//! changes by gravity's share down it less friction and air drag,
//! `g (sin θ − μ cos θ) − k w²` ([`FRICTION`], [`DRAG`]), integrated once
//! when the slide begins, at a fixed step, so it does not depend on the
//! frame rate. The path is the slope's line rounded over its top and, where
//! it runs out onto ground, over its foot ([`FILLET`]): the hips' vertical
//! speed never steps. Run out, the slide brakes on the flat to a stop and
//! stands; at a drop, it goes over the edge as a fall (the walker's), and
//! a jump asked leaps off it.
//!
//! No slide data: the friction is a shoe sliding on tiles, the rest by eye.

use std::sync::Arc;

use bevy::math::{Quat, Vec3};

use super::Ledge;
use crate::character::anim::armik::{solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::ground::{GroundHit, GroundProbe};
use crate::character::anim::jump::GRAVITY;
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// A slope steeper than this (rise over run) is slid down, not walked.
pub const SLIDE_GRADE: f32 = 0.6;
/// The feet's sliding friction, a share of the slope's push into them; air
/// drag, per metre (`k`: the terminal speed down a 40° roof about 5.5 m/s).
pub const FRICTION: f32 = 0.4;
pub const DRAG: f32 = 0.11;
/// The path's rounding over the slope's top and foot, metres either side
/// of the crease.
pub const FILLET: f32 = 0.4;
/// The feet's own path's rounding over the top edge and the foot crease,
/// metres either side: over the top as wide as the hips' (tighter, a foot
/// passing it in the entry changed step 7.8 cm at 4 m/s), at the foot
/// tighter (rounded as wide, a foot rode 12 cm over the slope's foot).
const FOOT_FILLET: (f32, f32) = (0.4, 0.25);
/// The slide's step, seconds, and the longest it is integrated for.
const STEP: f32 = 1.0 / 240.0;
const LONGEST: f32 = 60.0;
/// Going into the slide, seconds: the pose and facing over this long, the
/// legs over this long.
const ENTRY: f32 = 0.35;
const LEGS_IN: f32 = 0.35;
/// Drifting across the slope as it began, the drift dies away over this
/// long, seconds.
const DRIFT: f32 = 0.3;
/// Run out onto the flat, slowed to this, m/s, the feet stick and it rises
/// over this long, seconds.
const STOP: f32 = 0.8;
const RISE: f32 = 0.5;
/// Asked to leap off a drop's edge, the hips pushed up to this speed, m/s
/// (a standing jump's take-off), over at most this long before the edge,
/// seconds, the legs straightening under them (added at the edge at once,
/// the hips' step changed 6 cm in the frame it leapt).
const LEAP_UP: f32 = 3.0;
const LEAP_PUSH: f32 = 0.2;
/// Asked to catch the eave, it brakes over this much of the slope (level
/// metres) to come to its edge this fast, m/s.
const CATCH_BRAKE: f32 = 1.0;
const CATCH_SPEED: f32 = 1.2;
/// Each ankle no farther from its hip than this share of the leg.
const LEG_REACH: f32 = 0.95;
/// Blending in, no ankle nor toe lower than this under where it stands on
/// the surface, metres, lifted softly over this, metres; the lift faded in
/// over the first this long and out over the entry's last this long,
/// seconds (the skid's).
const FLOOR_UNDER: f32 = 0.01;
const FLOOR_SOFT: f32 = 0.025;
const LIFT_IN: f32 = 0.05;
const LIFT_OUT: f32 = 0.1;
/// Where a point stands on a slope from: this far below its surface,
/// metres (a foot reaching for it), as a ledge's top.
const TOP_BELOW: f32 = 0.3;

/// How a slope is slid down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlopeKind {
    /// Low on the feet, the hand on the up-slope side trailing on it.
    Roof,
    /// On the feet, leant back, the arms out (a pyramid's face).
    Face,
}

impl SlopeKind {
    /// The hips this much lower than standing, metres; the trunk leant back
    /// up the slope this far from upright, radians; the arms out this much
    /// (a beam's share).
    fn shape(self) -> (f32, f32, f32) {
        match self {
            SlopeKind::Roof => (0.6, 0.35, 0.5),
            SlopeKind::Face => (0.35, 0.3, 0.8),
        }
    }

    /// The left foot this far ahead of the hips down the slope, the right
    /// this far (behind, negative), level metres: a roof's low stance
    /// astride, a face's feet near together (down a 50° face, a foot 0.3 m
    /// ahead was out of the leg's reach, 8 cm off the surface).
    fn feet(self) -> [f32; 2] {
        match self {
            SlopeKind::Roof => [0.3, -0.15],
            SlopeKind::Face => [0.1, -0.05],
        }
    }
}

/// A plane falling away from its top edge: `width` along that edge, `run`
/// level metres down to its foot edge, `drop` lower there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Slope {
    /// The middle of its top edge (the world).
    pub top: Vec3,
    /// The way it falls away (level, unit).
    pub down: Vec3,
    pub width: f32,
    pub run: f32,
    pub drop: f32,
    pub kind: SlopeKind,
}

impl Slope {
    pub fn new(top: Vec3, down: Vec3, width: f32, run: f32, drop: f32, kind: SlopeKind) -> Self {
        Self { top, down: down.with_y(0.0).normalize_or(Vec3::NEG_Z), width, run: run.max(0.1), drop, kind }
    }

    /// Rise over run.
    pub fn grade(&self) -> f32 {
        self.drop / self.run
    }

    /// Along its top edge (level, unit; to the left facing down it).
    pub fn across(&self) -> Vec3 {
        Vec3::Y.cross(self.down)
    }

    /// `at`'s level place: how far down from the top edge, and how far
    /// along it from its middle, metres.
    pub fn place(&self, at: Vec3) -> (f32, f32) {
        let d = (at - self.top).with_y(0.0);
        (d.dot(self.down), d.dot(self.across()))
    }

    /// Whether `at` is over (or under) it.
    pub fn covers(&self, at: Vec3) -> bool {
        let (along, across) = self.place(at);
        (0.0..=self.run).contains(&along) && across.abs() <= 0.5 * self.width
    }

    /// Its surface's height at `along` down it.
    pub fn height_at(&self, along: f32) -> f32 {
        self.top.y - self.grade() * along.clamp(0.0, self.run)
    }

    /// Its surface under `at`, no further than a foot's reach below it.
    pub fn top_under(&self, at: Vec3) -> Option<f32> {
        let height = self.height_at(self.place(at).0);
        (self.covers(at) && at.y > height - TOP_BELOW).then_some(height)
    }

    /// Its surface's normal (up and down it).
    pub fn normal(&self) -> Vec3 {
        (Vec3::Y + self.down * self.grade()).normalize()
    }

    /// The middle of its foot edge.
    pub fn foot(&self) -> Vec3 {
        self.top + self.down * self.run - Vec3::Y * self.drop
    }

    /// Its foot edge as a ledge to hang from, facing down it, its wall
    /// going `below` down from it (an eave over a house's wall).
    pub fn eave(&self, below: f32) -> Ledge {
        Ledge { wall_below: below, ..Ledge::wall(self.foot() - Vec3::Y * self.foot().y, self.down, self.width, self.foot().y, 0.3) }
    }
}

/// The ground `under` with slopes on it, the highest under a point.
pub struct SlopeGround {
    pub under: Box<dyn GroundProbe>,
    pub slopes: Vec<Slope>,
}

impl GroundProbe for SlopeGround {
    fn sample(&self, at: Vec3) -> Option<GroundHit> {
        let under = self.under.sample(at);
        let on = self.slopes.iter().filter_map(|slope| slope.top_under(at).map(|height| (height, slope.normal()))).reduce(|a, b| if b.0 > a.0 { b } else { a });
        match (on, under) {
            (Some((height, _)), Some(under)) if under.height > height => Some(under),
            (Some((height, normal)), _) => Some(GroundHit { height, normal }),
            (None, under) => under,
        }
    }

    fn blocks(&self, point: Vec3, low: f32, high: f32) -> bool {
        self.under.blocks(point, low, high)
    }
}

/// The slide's path: the height (relative to the top edge's) at `along`
/// down it, and its slope there (down is positive), on a slope `grade`
/// steep `run` long, rounded over the top and, `runs_out`, the foot.
fn profile(along: f32, grade: f32, run: f32, runs_out: bool) -> (f32, f32) {
    let r = FILLET.min(0.25 * run);
    if along <= -r {
        (0.0, 0.0)
    } else if along < r {
        let u = along + r;
        (-grade * u * u / (4.0 * r), grade * u / (2.0 * r))
    } else if !runs_out || along <= run - r {
        (-grade * along, grade)
    } else if along < run + r {
        let u = run + r - along;
        (-grade * run + grade * u * u / (4.0 * r), grade * u / (2.0 * r))
    } else {
        (-grade * run, 0.0)
    }
}

/// A slide down a slope under way.
#[derive(Debug, Clone)]
pub struct SlopeSlide {
    slope: Slope,
    /// Whether the slope runs out onto ground as high as its foot (else it
    /// ends at a drop).
    runs_out: bool,
    rig: Arc<RigGeometry>,
    stood: LocalPose,
    /// The pose, facing and hips (the world) as it began; how far across
    /// the slope it was then and how fast it drifted across.
    from: LocalPose,
    yaw_from: f32,
    hips_from: Vec3,
    across: f32,
    drift: f32,
    /// How far the walk's hips were from the slide's as it began: eased
    /// out over the entry.
    entry_offset: Vec3,
    /// How far down the slope (level metres) and how fast along its path
    /// (m/s), every [`STEP`] from the start: to the foot edge at a drop, to
    /// the stop run out.
    track: Arc<Vec<(f32, f32)>>,
    /// Leaping off the edge: when the push up began, seconds.
    leap_from: Option<f32>,
    t: f32,
}

impl SlopeSlide {
    /// A slide down `slope` from a walker at `root` facing `yaw`, posed
    /// `from`, moving `velocity` (level): `None` unless it is going down
    /// the slope (within 60° of straight down it), near enough its top
    /// edge, and the slope steep enough to slide on. `ground` finds the
    /// ground's height past its foot edge (the world).
    #[allow(clippy::too_many_arguments)]
    pub fn begin(slope: &Slope, root: Vec3, yaw: f32, velocity: Vec3, from: &LocalPose, ground: &dyn Fn(Vec3) -> Option<f32>, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let (along, across) = slope.place(root);
        let downward = velocity.dot(slope.down);
        let r = FILLET.min(0.25 * slope.run);
        if slope.grade() < SLIDE_GRADE || downward < 0.5 * velocity.length() || downward <= 0.0 || !(-r..0.5 * r).contains(&along) || across.abs() > 0.5 * slope.width {
            return None;
        }
        let past = slope.foot() + slope.down * 0.3;
        let runs_out = ground(past.with_y(slope.foot().y + 0.1)).is_some_and(|height| (height - slope.foot().y).abs() < 0.05);
        let turn = Quat::from_rotation_y(yaw);
        let mut slide = Self {
            slope: *slope,
            runs_out,
            rig: Arc::new(rig.clone()),
            stood: *stood,
            from: *from,
            yaw_from: yaw,
            hips_from: root + turn * forward_kinematics_on(from, rig)[Bone::Hips],
            across,
            drift: velocity.dot(slope.across()),
            entry_offset: Vec3::ZERO,
            track: Arc::new(Vec::new()),
            leap_from: None,
            t: 0.0,
        };
        slide.track = Arc::new(slide.integrate(along, downward, false));
        let (start_hips, _) = slide.shape(along, across, yaw, 0.0, 0.0);
        slide.entry_offset = slide.hips_from - start_hips;
        Some(slide)
    }

    /// The same slide braking over its last [`CATCH_BRAKE`] to come to its
    /// edge at [`CATCH_SPEED`], to catch the eave (at a drop only): going
    /// over it at the slide's own 3 m/s, the body was 1 m out from it by
    /// the time it had turned round to face it.
    pub fn braking_to_catch(mut self) -> Self {
        if !self.runs_out {
            let (along, speed) = self.track[0];
            let (_, slope) = self.path(along);
            self.track = Arc::new(self.integrate(along, speed / (1.0 + slope * slope).sqrt(), true));
        }
        self
    }

    /// The path's height and slope at `along` (the world's height).
    fn path(&self, along: f32) -> (f32, f32) {
        let (height, slope) = profile(along, self.slope.grade(), self.slope.run, self.runs_out);
        (self.slope.top.y + height, slope)
    }

    /// The track from `along` at `level` m/s down it: friction coming in
    /// with the slope over its top (walked onto it, it would stop dead on
    /// the flat behind its edge).
    fn integrate(&self, mut along: f32, level: f32, catch: bool) -> Vec<(f32, f32)> {
        let r = FILLET.min(0.25 * self.slope.run);
        let (_, slope) = self.path(along);
        let mut speed = level * (1.0 + slope * slope).sqrt();
        let mut track = vec![(along, speed)];
        let steps = (LONGEST / STEP) as usize;
        // Braking to catch the eave: from `CATCH_BRAKE` short of it, slowed
        // evenly to `CATCH_SPEED` at it, the hands and feet dragging.
        let mut braking: Option<f32> = None;
        for _ in 0..steps {
            let (_, slope) = self.path(along);
            let angle = slope.atan();
            if catch && !self.runs_out && braking.is_none() && along >= self.slope.run - CATCH_BRAKE {
                let length = (self.slope.run - along).max(1.0e-3) / angle.cos();
                braking = Some(((speed * speed - CATCH_SPEED * CATCH_SPEED) / (2.0 * length)).max(0.0));
            }
            let friction = FRICTION * ((along + r) / (2.0 * r)).clamp(0.0, 1.0);
            let accel = match braking {
                Some(braking) => -braking,
                None => GRAVITY * (angle.sin() - friction * angle.cos()) - DRAG * speed * speed,
            };
            speed = (speed + accel * STEP).max(0.0);
            along += speed * angle.cos() * STEP;
            track.push((along, speed));
            if !self.runs_out && along >= self.slope.run {
                break;
            }
            if self.runs_out && along > self.slope.run + r && speed <= STOP {
                break;
            }
        }
        track
    }

    /// How far down and how fast at `t` along the track (past its end, the
    /// rise's braking to a stop).
    fn travel(&self, t: f32) -> (f32, f32) {
        let x = t / STEP;
        let last = self.track.len() - 1;
        let k = (x.floor() as usize).min(last);
        if k >= last {
            let (along, speed) = self.track[last];
            if !self.runs_out {
                return (along, speed);
            }
            let tau = (t - self.slid()).clamp(0.0, RISE);
            return (along + speed * (tau - 0.5 * tau * tau / RISE), speed * (1.0 - tau / RISE));
        }
        let s = x - k as f32;
        let (a, b) = (self.track[k], self.track[k + 1]);
        (a.0 + (b.0 - a.0) * s, a.1 + (b.1 - a.1) * s)
    }

    /// How long it slides, seconds: to the foot edge, or to its stop.
    pub fn slid(&self) -> f32 {
        (self.track.len() - 1) as f32 * STEP
    }

    /// Its whole length: the slide, and run out, the rise.
    pub fn end(&self) -> f32 {
        self.slid() + if self.runs_out { RISE } else { 0.0 }
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt).min(self.end());
    }

    /// How long since it began, seconds.
    pub fn elapsed(&self) -> f32 {
        self.t
    }

    /// Asked to leap off its edge (at a drop): the hips pushed up over the
    /// last [`LEAP_PUSH`] before it, or what is left, to [`LEAP_UP`] at the
    /// edge (the fall going on with it). Once.
    pub fn leap(&mut self) {
        if !self.runs_out && self.leap_from.is_none() {
            self.leap_from = Some(self.t.max(self.slid() - LEAP_PUSH));
        }
    }

    /// Whether it leaps off its edge.
    pub fn leaps(&self) -> bool {
        self.leap_from.is_some()
    }

    /// How far the push up has lifted the hips by `t`, metres: its speed
    /// rising as a smoothstep to [`LEAP_UP`] at the edge, so lifted by its
    /// integral, `V T (u³ − u⁴/2)`.
    fn leap_lift(&self, t: f32) -> f32 {
        let Some(from) = self.leap_from else { return 0.0 };
        let span = (self.slid() - from).max(1.0e-3);
        let u = ((t - from) / span).clamp(0.0, 1.0);
        LEAP_UP * span * (u * u * u - 0.5 * u * u * u * u)
    }

    /// Run out, whether it stands; at a drop, whether it is over the edge.
    pub fn is_done(&self) -> bool {
        self.t >= self.end()
    }

    /// Whether it ends at a drop (going over the edge as a fall).
    pub fn ends_at_drop(&self) -> bool {
        !self.runs_out
    }

    /// The slope it slides down.
    pub fn slope(&self) -> &Slope {
        &self.slope
    }

    /// Its facing at `t`: turned to face down the slope over the entry.
    fn yaw_at(&self, t: f32) -> f32 {
        let forward = self.rig.forward();
        let down = self.slope.down.x.atan2(self.slope.down.z) - forward.x.atan2(forward.z);
        let turn = crate::character::anim::facing::shortest_angle(down - self.yaw_from);
        self.yaw_from + turn * smoothstep((t / ENTRY).clamp(0.0, 1.0))
    }

    /// The walker's facing now.
    pub fn facing(&self) -> f32 {
        self.yaw_at(self.t)
    }

    /// How far across the slope at `t`: the drift it began with dying away.
    fn across_at(&self, t: f32) -> f32 {
        self.across + self.drift * DRIFT * (1.0 - (-t / DRIFT).exp())
    }

    /// The surface's point at `along` down it, `across` along its top edge:
    /// the true surface; past its foot the ground as high, or at a drop the
    /// slope's line on (a foot going over the edge goes on as it went).
    fn surface(&self, along: f32, across: f32) -> Vec3 {
        let height = if along <= 0.0 {
            self.slope.top.y
        } else if along <= self.slope.run || self.runs_out {
            self.slope.height_at(along)
        } else {
            self.slope.top.y - self.slope.grade() * along
        };
        (self.slope.top + self.slope.down * along + self.slope.across() * across).with_y(height)
    }

    /// The feet's path's height at `along` down it: the surface, rounded
    /// over its creases, never under it. On the surface itself a foot
    /// sliding over the top edge dived in a frame (12 cm at 4 m/s). A
    /// rounding under the top's convex edge cuts into it (by `g r/4` at the
    /// edge), so it is lifted by `(g r/4)(1 − (x/r)²)²`, which is at least
    /// that cut everywhere (`(1−u²)² ≥ (1−u)²`) and smooth: the feet skim
    /// up to 4 cm over the slope there. At the foot's concave crease the
    /// rounding is over the surface already.
    fn foot_height(&self, along: f32, across: f32) -> f32 {
        let (grade, run) = (self.slope.grade(), self.slope.run);
        let (top, foot) = (FOOT_FILLET.0.min(0.25 * run), FOOT_FILLET.1.min(0.25 * run));
        let true_height = self.surface(along, across).y;
        if along.abs() < top {
            let (r, u) = (top, along / top);
            let rounded = -grade * (along + r) * (along + r) / (4.0 * r);
            self.slope.top.y + rounded + 0.25 * grade * r * (1.0 - u * u) * (1.0 - u * u)
        } else if self.runs_out && (along - run).abs() < foot {
            let u = run + foot - along;
            self.slope.top.y - grade * run + grade * u * u / (4.0 * foot)
        } else {
            true_height
        }
    }

    /// The angle a foot is soled to at `along`: the feet's path's slope
    /// (soled to the true surface, a toe passing the foot edge turned level
    /// in a frame, 10.6 cm).
    fn angle_under(&self, along: f32, across: f32) -> f32 {
        let e = 1.0e-3;
        ((self.foot_height(along - e, across) - self.foot_height(along + e, across)) / (2.0 * e)).atan()
    }

    /// The hips, the pose and the facing at the hips' place `along` down
    /// and `across`, `rise` of the way back up to standing (run out).
    fn shape(&self, along: f32, across: f32, yaw: f32, rise: f32, lift: f32) -> (Vec3, LocalPose) {
        let rig = &*self.rig;
        let (drop, back, arms) = self.slope.kind.shape();
        let standing = forward_kinematics_on(&self.stood, rig);
        let (path, _) = self.path(along);
        let hips = (self.slope.top + self.slope.down * along + self.slope.across() * across).with_y(path + standing[Bone::Hips].y - drop * (1.0 - rise) + lift);
        let mut pose = self.stood;
        super::beam::balance(&mut pose, rig, arms * (1.0 - rise), 0.0);
        let turn = Quat::from_rotation_y(yaw);
        let up_slope = turn.inverse() * -self.slope.down;
        let lean = back * (1.0 - rise);
        let tilt = Quat::from_rotation_arc(Vec3::Y, Vec3::Y * lean.cos() + up_slope * lean.sin());
        pose.rotations[Bone::Spine] = delta_after_world_turn(&pose, rig, Bone::Spine, tilt);
        let at = forward_kinematics_on(&pose, rig);
        let root = hips - turn * at[Bone::Hips];
        // The feet: the left ahead down the slope, the right behind, each
        // on the surface under it, soled to it.
        let level = accumulate_world_rotations(&self.stood, rig);
        for (side, (foot, (socket, knee))) in [Bone::LeftFoot, Bone::RightFoot].into_iter().zip([(Bone::LeftUpLeg, Bone::LeftLeg), (Bone::RightUpLeg, Bone::RightLeg)]).enumerate() {
            let ahead = self.slope.kind.feet()[side] * (1.0 - rise);
            let stands = turn * (standing[foot] - standing[Bone::Hips]);
            let aside = stands.dot(self.slope.across());
            let under = along + ahead + stands.dot(self.slope.down);
            let angle = self.angle_under(under, across + aside);
            let path = self.surface(under, across + aside).with_y(self.foot_height(under, across + aside));
            let ankle = path + (Vec3::Y * angle.cos() + self.slope.down * angle.sin()) * standing[foot].y;
            let target = turn.inverse() * (ankle - root);
            // No farther than `LEG_REACH`, or standing's own reach (shorter,
            // the risen stand's knee was bent 8 cm off standing's).
            let leg = (at[knee] - at[socket]).length() + (at[foot] - at[knee]).length();
            let reach = (LEG_REACH * leg).max((standing[foot] - standing[socket]).length());
            let off = target - at[socket];
            let target = at[socket] + off * (off.length().min(reach) / off.length().max(1.0e-6));
            place_ankle(&mut pose, rig, foot, target - at[Bone::Hips]);
            let pitch = Quat::from_axis_angle(Vec3::Y.cross(turn.inverse() * self.slope.down).normalize_or(Vec3::X), angle);
            let now = accumulate_world_rotations(&pose, rig)[foot];
            pose.rotations[foot] = delta_after_world_turn(&pose, rig, foot, pitch * level[foot] * now.inverse());
        }
        // A roof's trailing hand: the up-slope side's (the right, the left
        // foot leading), on the slope behind and beside the hips.
        if self.slope.kind == SlopeKind::Roof && rise < 1.0 {
            let at = forward_kinematics_on(&pose, rig);
            let free = pose;
            let chain = ArmChain::RIGHT;
            let side = -(turn * rig.left()).dot(self.slope.across()).signum();
            let place = self.surface(along - 0.45, across + side * 0.3) + self.slope.normal() * 0.05;
            let pole = (up_slope + turn.inverse() * self.slope.across() * side + Vec3::Y * 0.3).normalize();
            solve_arm_toward_from(&mut pose, &at, chain, turn.inverse() * (place - root), pole, rig);
            for bone in [chain.shoulder, chain.elbow, chain.wrist] {
                pose.rotations[bone] = pose.rotations[bone].slerp(free.rotations[bone], rise);
            }
        }
        (hips, pose)
    }

    /// The hips, pose and facing now.
    fn now(&self) -> (Vec3, LocalPose, f32) {
        let t = self.t;
        let (along, _) = self.travel(t);
        let across = self.across_at(t);
        let yaw = self.yaw_at(t);
        let rise = if self.runs_out { smoothstep(((t - self.slid()) / RISE).clamp(0.0, 1.0)) } else { 0.0 };
        // Leaping, the hips pushed up off the feet, the legs straightening.
        let (hips, slid) = self.shape(along, across, yaw, rise, self.leap_lift(t));
        // Into the slide from the walk's pose and hips.
        let w = smoothstep((t / ENTRY).clamp(0.0, 1.0));
        let legs = smoothstep((t / LEGS_IN).clamp(0.0, 1.0));
        let mut pose = slid;
        if w < 1.0 {
            for bone in Bone::ALL {
                let share = if is_leg(bone) { legs } else { w };
                pose.rotations[bone] = self.from.rotations[bone].slerp(slid.rotations[bone], share);
            }
            pose.root_translation = self.from.root_translation.lerp(slid.root_translation, w);
        }
        let hips = hips + self.entry_offset * (1.0 - w);
        // Blending in, each foot kept off the surface, softly: between the
        // walk's legs and the slide's, the trailing toe went 9-27 cm under
        // the top behind the edge.
        let lifting = smoothstep((t / LIFT_IN).clamp(0.0, 1.0)) * (1.0 - smoothstep(((t - ENTRY + LIFT_OUT) / LIFT_OUT).clamp(0.0, 1.0)));
        if lifting > 0.0 {
            self.keep_off_surface(&mut pose, hips, yaw, lifting);
        }
        (hips, pose, yaw)
    }

    /// Lifts each foot clear of the surface where the blend took it under,
    /// softly, by `weight` (the skid's `keep_off_floor`).
    fn keep_off_surface(&self, pose: &mut LocalPose, hips: Vec3, yaw: f32, weight: f32) {
        let rig = &*self.rig;
        let turn = Quat::from_rotation_y(yaw);
        let standing = forward_kinematics_on(&self.stood, rig);
        let at = forward_kinematics_on(pose, rig);
        let root = hips - turn * at[Bone::Hips];
        // Off the feet's path, not the surface: lifted off the top edge's
        // corner itself, a foot dived as it passed it (12.6 cm).
        let under = |point: Vec3| {
            let (along, across) = self.slope.place(point);
            self.foot_height(along, across)
        };
        for (foot, toe) in [(Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightFoot, Bone::RightToeBase)] {
            let (ankle, tip) = (root + turn * at[foot], root + turn * at[toe]);
            let short = (under(ankle) + standing[foot].y - FLOOR_UNDER - ankle.y).max(under(tip) + standing[toe].y - FLOOR_UNDER - tip.y);
            let lift = weight * FLOOR_SOFT * (short / FLOOR_SOFT).exp().ln_1p();
            if lift > 1.0e-4 {
                place_ankle(pose, rig, foot, turn.inverse() * (ankle + Vec3::Y * lift - root) - at[Bone::Hips]);
            }
        }
    }

    /// The pose now, at [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self) -> LocalPose {
        self.now().1
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

    /// The walker's root now: the pose's hips on the slide's hips.
    pub fn root(&self) -> Vec3 {
        let (hips, pose, yaw) = self.now();
        hips - Quat::from_rotation_y(yaw) * forward_kinematics_on(&pose, &self.rig)[Bone::Hips]
    }

    /// Every joint in the world now.
    pub fn joints(&self) -> BoneSet<Vec3> {
        let (hips, pose, yaw) = self.now();
        let turn = Quat::from_rotation_y(yaw);
        let at = forward_kinematics_on(&pose, &self.rig);
        let root = hips - turn * at[Bone::Hips];
        BoneSet::from_fn(|bone| root + turn * at[bone])
    }

    /// The hips' velocity in the world now, m/s.
    pub fn hips_velocity(&self) -> Vec3 {
        let h = 1.0e-3;
        let mut before = self.clone();
        before.t = (self.t - h).max(0.0);
        let span = self.t - before.t;
        if span <= 0.0 {
            let mut after = self.clone();
            after.t = h;
            return (after.now().0 - self.now().0) / h;
        }
        (self.now().0 - before.now().0) / span
    }

    /// The walker's root velocity now: the hips' (what a fall leaving it
    /// goes on with).
    pub fn velocity(&self) -> Vec3 {
        self.hips_velocity()
    }
}

fn is_leg(bone: Bone) -> bool {
    matches!(bone, Bone::LeftUpLeg | Bone::LeftLeg | Bone::LeftFoot | Bone::LeftToeBase | Bone::RightUpLeg | Bone::RightLeg | Bone::RightFoot | Bone::RightToeBase)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gait::{walk_pose_on, GaitParams};
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        (stood, rig)
    }

    /// A slope's surface stands over it from a foot's reach below; its
    /// ground finds it over the ground under it, its normal tilted down it.
    #[test]
    fn a_slopes_surface_is_stood_on_over_it() {
        let slope = Slope::new(Vec3::new(0.0, 4.0, -1.0), Vec3::NEG_Z, 2.0, 3.0, 2.4, SlopeKind::Roof);
        assert_eq!(slope.top_under(Vec3::new(0.5, 3.0, -2.5)), Some(2.8));
        assert_eq!(slope.top_under(Vec3::new(0.5, 3.0, -4.5)), None, "past its foot");
        assert_eq!(slope.top_under(Vec3::new(1.5, 3.0, -2.5)), None, "past its side");
        assert_eq!(slope.top_under(Vec3::new(0.5, 2.0, -2.5)), None, "well below it");
        let ground = SlopeGround { under: Box::new(crate::character::anim::ground::FlatGround::default()), slopes: vec![slope] };
        let hit = ground.sample(Vec3::new(0.0, 3.5, -2.0)).expect("on it");
        assert!((hit.height - 3.2).abs() < 1.0e-5 && hit.normal.z < -0.5, "{hit:?}");
        assert_eq!(ground.sample(Vec3::new(0.0, 3.5, 2.0)).map(|hit| hit.height), Some(0.0));
        let eave = slope.eave(1.6);
        assert!((eave.height() - 1.6).abs() < 1.0e-5 && (eave.out - Vec3::NEG_Z).length() < 1.0e-6);
    }

    /// Walked or run onto a roof and a steep face, it slides down on its
    /// feet: the feet on the surface, nothing through it, a roof's trailing
    /// hand on it; its speed down the straight part as gravity, friction
    /// and drag have it (the closed form, not the integration); every
    /// joint's path smooth. At a roof's edge a fall goes on at the hips'
    /// velocity; run out from a face onto the ground, it brakes to a stop
    /// and stands.
    #[test]
    fn slopes_are_slid_down_on_the_feet() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let ankle = forward_kinematics_on(&stood, &rig)[Bone::LeftFoot].y;
        let mut faults = Vec::new();
        // Off a roof's edge also leaping (pushed up over its last moment)
        // and braking to catch its eave.
        #[derive(Debug, Clone, Copy, PartialEq)]
        enum Off {
            Plain,
            Leap,
            Catch,
        }
        let cases = [
            (SlopeKind::Roof, 0.84f32, 3.0f32, 6.0f32, 1.4f32, Off::Plain),
            (SlopeKind::Roof, 0.84, 3.0, 6.0, 4.0, Off::Plain),
            (SlopeKind::Roof, 0.84, 3.0, 6.0, 4.0, Off::Leap),
            (SlopeKind::Roof, 0.84, 3.0, 6.0, 4.0, Off::Catch),
            (SlopeKind::Face, 1.19, 5.0, 5.95, 1.4, Off::Plain),
            (SlopeKind::Face, 1.19, 5.0, 5.95, 4.0, Off::Plain),
        ];
        for (kind, grade, run, top, speed, off) in cases {
            let name = format!("{kind:?} at {grade} from {speed} m/s, {off:?}");
            let slope = Slope::new(Vec3::new(0.0, top, 0.0) + forward * 0.35, forward, 3.0, run, grade * run, kind);
            let ground = |at: Vec3| slope.top_under(at).or(Some(0.0));
            let params = if speed > 2.0 { GaitParams::running_on(speed, &rig) } else { GaitParams::walking_on(speed, &rig) };
            let from = walk_pose_on(0.0, &params, &stood, &rig);
            let Some(mut slide) = SlopeSlide::begin(&slope, Vec3::new(0.0, top, 0.0), 0.0, forward * speed, &from, &ground, &stood, &rig) else {
                faults.push(format!("{name}: not begun"));
                continue;
            };
            match off {
                Off::Plain => {}
                Off::Leap => slide.leap(),
                Off::Catch => slide = slide.braking_to_catch(),
            }
            // The ground under a point: behind the top edge the top, on the
            // slope its surface, past its foot the ground (none at a drop).
            let drops = slide.ends_at_drop();
            let surface_y = |at: Vec3| {
                let (along, across) = slope.place(at);
                if across.abs() > 0.5 * slope.width {
                    0.0
                } else if along < 0.0 {
                    top
                } else if along <= slope.run {
                    slope.height_at(along)
                } else if drops {
                    f32::MIN
                } else {
                    0.0
                }
            };
            let (mut feet_off, mut hand_off, mut under, mut kink) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
            let mut frames: Vec<BoneSet<Vec3>> = Vec::new();
            let mut worst = (String::new(), String::new());
            let normal = slope.normal();
            // Whole frames only: the last, cut short at the end, is the
            // walker's hand-off (the rest of it given to what follows).
            while slide.t + DT <= slide.end() {
                slide.advance(DT);
                let joints = slide.joints();
                let sliding = slide.t > ENTRY && slide.t < slide.slid();
                let (along, _) = slide.travel(slide.t);
                let on_slope = (FILLET..slope.run - FILLET).contains(&along);
                if sliding && on_slope {
                    for foot in [Bone::LeftFoot, Bone::RightFoot] {
                        let at = joints[foot];
                        // The ankle's height off the surface along its
                        // normal, against standing's: past the feet's path's
                        // rounding over the top edge.
                        if slope.covers(at) && slope.place(at).0 > FOOT_FILLET.0 {
                            let height = (at.y - slope.height_at(slope.place(at).0)) * normal.y;
                            feet_off = feet_off.max((height - ankle).abs());
                        }
                    }
                    if kind == SlopeKind::Roof {
                        let wrist = joints[ArmChain::RIGHT.wrist];
                        hand_off = hand_off.max((wrist.y - slope.height_at(slope.place(wrist).0)) * normal.y - 0.05);
                    }
                }
                for bone in Bone::ALL {
                    let at = joints[bone];
                    if surface_y(at) - at.y > under {
                        under = surface_y(at) - at.y;
                        worst.0 = format!("{bone:?} at {:.3} s", slide.t);
                    }
                }
                if frames.len() >= 2 {
                    let n = frames.len();
                    for bone in Bone::ALL {
                        let step = (joints[bone] - 2.0 * frames[n - 1][bone] + frames[n - 2][bone]).length();
                        if step > kink {
                            kink = step;
                            worst.1 = format!("{bone:?} at {:.3} s", slide.t);
                        }
                    }
                }
                frames.push(joints);
            }
            eprintln!("{name}: most under {}, most changed {}", worst.0, worst.1);
            // The straight part's speed against `w(t) = W tanh(√(ak) t + c)`.
            let angle = grade.atan();
            let a = GRAVITY * (angle.sin() - FRICTION * angle.cos());
            let terminal = (a / DRAG).sqrt();
            let k0 = slide.track.iter().position(|&(along, _)| along > FILLET).expect("onto the straight");
            // (Braking to catch, only up to where it brakes.)
            let straight_to = slope.run - if off == Off::Catch { CATCH_BRAKE } else { FILLET };
            let k1 = slide.track.iter().position(|&(along, _)| along > straight_to).unwrap_or(slide.track.len() - 1);
            let (w0, w1) = (slide.track[k0].1, slide.track[k1].1);
            let seconds = (k1 - k0) as f32 * STEP;
            let closed = terminal * ((a * DRAG).sqrt() * seconds + (w0 / terminal).atanh()).tanh();
            eprintln!("{name}: slid {:.2} s, {w0:.2} -> {w1:.2} m/s (closed form {closed:.2}), feet off {feet_off:.4}, hand off {hand_off:.4}, under {under:.4}, kink {kink:.4}", slide.slid());
            if (w1 - closed).abs() > 0.02 {
                faults.push(format!("{name}: {w1:.3} m/s at the foot, the closed form {closed:.3}"));
            }
            if feet_off > 0.02 || under > 0.01 {
                faults.push(format!("{name}: the feet {feet_off:.4} off the slope, a joint {under:.4} under it"));
            }
            if kind == SlopeKind::Roof && hand_off > 0.06 {
                faults.push(format!("{name}: the hand {hand_off:.4} off the roof"));
            }
            // The gait's own change of step at its speed (posed from its
            // clock, carried at its speed): stopping its swing mid-stride is
            // part of going into the slide.
            let own = {
                let distance = crate::character::anim::locomotion::distance_per_cycle(&params, &stood, &rig);
                let cycles: Vec<(f32, BoneSet<Vec3>)> = (0..120)
                    .map(|k| {
                        let cycle = k as f32 * DT * speed / distance;
                        let at = forward_kinematics_on(&walk_pose_on(cycle.fract(), &params, &stood, &rig), &rig);
                        (cycle, BoneSet::from_fn(|b| at[b] + forward * (cycle * distance)))
                    })
                    .collect();
                // Not across the cycle's wrap (the clock's, not the gait's).
                let mut most = (0.0f32, Bone::Hips, 0.0f32);
                for w in cycles.windows(3).filter(|w| w[0].0.floor() == w[2].0.floor()) {
                    for b in Bone::ALL {
                        let step = (w[2].1[b] - 2.0 * w[1].1[b] + w[0].1[b]).length();
                        if step > most.0 {
                            most = (step, b, w[1].0);
                        }
                    }
                }
                eprintln!("{name}: the gait's most at {:?}, cycle {:.3}, {distance:.3} m a cycle", most.1, most.2);
                most.0
            };
            eprintln!("{name}: the gait's own change of step {own:.4}");
            if kink > own.max(0.02) + 0.01 {
                faults.push(format!("{name}: a step changed {kink:.4}, the gait's own {own:.4}"));
            }
            if slide.ends_at_drop() {
                slide.advance(slide.end() - slide.t);
                let falling = super::super::Falling::off(slide.root(), slide.facing(), slide.velocity(), &slide.pose(), 0.0, 0.0, &stood, &rig);
                let jump = (falling.hips_velocity() - slide.hips_velocity()).length();
                if jump > 0.05 {
                    faults.push(format!("{name}: the hips' velocity changed {jump:.3} going over the edge"));
                }
                // Leaping, going up off the edge, pushed up by `LEAP_UP` on
                // the slide's own; braking to catch, at `CATCH_SPEED`.
                let (velocity, edge_speed) = (slide.hips_velocity(), slide.travel(slide.t).1);
                let down_the_slope = -edge_speed * grade.atan().sin();
                eprintln!("{name}: off the edge at {velocity} ({edge_speed:.2} m/s along it)");
                match off {
                    Off::Leap if (velocity.y - down_the_slope - LEAP_UP).abs() > 0.1 => faults.push(format!("{name}: leapt up at {:.2} m/s over the slide's", velocity.y - down_the_slope)),
                    Off::Catch if (edge_speed - CATCH_SPEED).abs() > 0.1 =>faults.push(format!("{name}: at the edge at {edge_speed:.2} m/s braking to catch")),
                    _ => {}
                }
            } else {
                slide.advance(slide.end() - slide.t);
                let standing = forward_kinematics_on(&stood, &rig);
                let at = forward_kinematics_on(&slide.pose(), &rig);
                let (off, bone) = Bone::ALL.iter().map(|&b| ((at[b] - standing[b]).length(), b)).fold((0.0, Bone::Hips), |a, b| if b.0 > a.0 { b } else { a });
                if off > 0.01 || slide.travel(slide.t).1 > 1.0e-3 {
                    faults.push(format!("{name}: stood {off:.3} from standing ({bone:?}), moving {:.3}", slide.travel(slide.t).1));
                }
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
