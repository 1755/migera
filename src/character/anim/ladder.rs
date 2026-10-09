//! Climbing a ladder: getting on from the floor, up, down, sliding down the
//! rails, and off again.
//!
//! A climb is planned as moves of the four limbs between holds (the floor,
//! a rung, a rail), one step at a time, with the hips on a smooth path the
//! holds set. Every frame the body is posed on it:
//! - the trunk leant toward the ladder;
//! - each foot's ball placed on its rung (or the floor), the leg solved to
//!   it (`stance::place_ankle`);
//! - each hand placed over its rung by arm IK (`armik`), the elbow down and
//!   out, the hand tipped over the rung.
//!
//! The pattern is the four-beat lateral one, with the lateral the most used
//! in free climbing (McIntyre 1983) and among the commonest four-beat
//! patterns on rungs (Jensen and Holland 2020): one limb at a time, each
//! hand going up before its own foot, three limbs always holding. Each foot
//! moves past the other ([`Pattern`]): onto the rung `gap` above it, each
//! step raising the body `gap` rungs. Widely spaced rungs, too far for a
//! foot or a hand to pass the other, are climbed both feet to a rung.
//!
//! Everything is scaled to the ladder: rung spacing chooses the pattern and
//! times the steps (the body rises at [`SPEED_UP`], about 0.4 m/s,
//! Simeonov et al. 2020); the width spaces the hands and feet; the lean
//! tilts the body with the rails.
//!
//! A slide ([`Climb::Slide`]) is the sailor's: the hands take the rails,
//! the feet press their insides to the rails' outsides, and the body slides
//! down at up to [`SLIDE_SPEED`], braking to land on the floor beside the
//! rails, then steps back to where it got on.
//!
//! Positions are in the climb's frame: the pose frame of the walker where it
//! got on, turned to face the ladder square. The walker's root rides the
//! hips ([`Climbing::root`]), so its pose keeps the standing hips' place.

use std::f32::consts::TAU;

use bevy::math::{Quat, Vec2, Vec3};

use super::armik::{shoulder_lift, solve_arm_toward_from, ArmChain};
use super::gait::{hermite, leg_length_of};
use super::obstacles::Footprint;
use super::rig::{accumulate_bind_rotations, accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use super::stance::place_ankle;
use crate::character::skeleton::Bone;

/// A ladder: two rails with rungs between them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ladder {
    /// The floor point between the rails' feet.
    pub base: Vec3,
    /// The side it is climbed from, horizontal: a climber faces the other
    /// way.
    pub toward: Vec3,
    /// How far its top leans away from the climber, radians from vertical:
    /// 0 a fixed ladder, 0.25 a leaning one set at the 4:1 rule (75.5°).
    pub lean: f32,
    /// Between the rails, metres.
    pub width: f32,
    /// From one rung to the next along the rails, metres.
    pub spacing: f32,
    /// From the floor to the first rung along the rails, metres.
    pub first: f32,
    /// How many rungs.
    pub rungs: u32,
    /// A landing it leads onto, its width and depth, metres: a floor level
    /// with its top rung's top, on past it from the rungs, the rails going
    /// on [`Ladder::RAILS_ABOVE`] above it to hold stepping off. `None`, the
    /// ladder just ends, and a climb holds on at its top.
    pub landing: Option<Vec2>,
}

impl Ladder {
    /// A rung's radius, metres.
    pub const RUNG_RADIUS: f32 = 0.016;
    /// How far a ladder's rails go on above a landing, metres (OSHA
    /// 1910.23: 42 inches).
    pub const RAILS_ABOVE: f32 = 1.07;

    /// A fixed ladder 0.42 m wide, its 12 rungs 0.3 m apart from 0.3 m up
    /// (OSHA 1910.23: at most 0.36 m apart, at least 0.4 m wide).
    pub fn standard(base: Vec3, toward: Vec3) -> Self {
        Self { base, toward, lean: 0.0, width: 0.42, spacing: 0.3, first: 0.3, rungs: 12, landing: None }
    }

    /// The landing's floor height, if it leads onto one.
    pub fn landing_height(&self) -> Option<f32> {
        self.landing.map(|_| self.rung(self.top()).y + Self::RUNG_RADIUS)
    }

    /// Where the landing is: its floor's middle, the way from the rungs onto
    /// it (horizontal), and its width and depth; its height is the middle's.
    pub fn landing_area(&self) -> Option<(Vec3, Vec3, Vec2)> {
        let size = self.landing?;
        let height = self.landing_height()?;
        let edge = self.middle_at(height);
        Some((edge - self.out() * (0.5 * size.y), -self.out(), size))
    }

    /// The horizontal way out of it, toward the climber.
    pub fn out(&self) -> Vec3 {
        Vec3::new(self.toward.x, 0.0, self.toward.z).normalize_or(Vec3::Z)
    }

    /// Up its rails.
    pub fn up(&self) -> Vec3 {
        Vec3::Y * self.lean.cos() - self.out() * self.lean.sin()
    }

    /// The climber's left, along its rungs.
    pub fn left(&self) -> Vec3 {
        Vec3::Y.cross(-self.out())
    }

    /// Square to its rails and rungs, toward the climber.
    pub fn normal(&self) -> Vec3 {
        self.up().cross(self.left())
    }

    /// Rung `i`'s middle (0 the lowest).
    pub fn rung(&self, i: i32) -> Vec3 {
        self.base + self.up() * (self.first + i as f32 * self.spacing)
    }

    /// The highest rung's index.
    pub fn top(&self) -> i32 {
        self.rungs.max(1) as i32 - 1
    }

    /// How long its rails are: a rung's spacing past the highest, or
    /// [`Ladder::RAILS_ABOVE`] above a landing.
    pub fn length(&self) -> f32 {
        let past = if self.landing.is_some() { Self::RAILS_ABOVE / self.up().y.max(1.0e-3) } else { self.spacing };
        self.first + self.top() as f32 * self.spacing + past
    }

    /// The highest a hand holds: the top rung, or above a landing the rails
    /// at the heights rungs would be, up to their tops.
    pub fn highest_grip(&self) -> i32 {
        match self.landing {
            // A hand's width below the rails' tops.
            Some(_) => self.top() + ((Self::RAILS_ABOVE - 0.05) / self.spacing).floor() as i32,
            None => self.top(),
        }
    }

    /// The middle of its rungs' line at height `y`.
    pub fn middle_at(&self, y: f32) -> Vec3 {
        let up = self.up();
        self.base + up * ((y - self.base.y) / up.y.max(1.0e-3))
    }

    /// What it stands on, for a walk to go round: its rails' feet.
    pub fn footprint(&self) -> Footprint {
        let out = self.out();
        Footprint { middle: Vec3::new(self.base.x, 0.0, self.base.z), forward: out, size: Vec2::new(self.width + 0.08, 0.08) }
    }

    /// `self` seen from a frame at `origin` turned `turn` from the world's.
    fn in_frame(&self, origin: Vec3, turn: Quat) -> Self {
        let back = turn.inverse();
        Self { base: back * (self.base - origin), toward: back * self.toward, ..*self }
    }
}

/// A ground with ladders' landings on it: each landing's floor where it is
/// (from just below it up), else the ground `under` it. For a walker that
/// steps off onto a landing to stand there (`walker::Walker::climb`).
pub struct LadderGround {
    pub under: Box<dyn super::ground::GroundProbe>,
    pub ladders: Vec<Ladder>,
}

impl super::ground::GroundProbe for LadderGround {
    fn sample(&self, at: Vec3) -> Option<super::ground::GroundHit> {
        for ladder in &self.ladders {
            let Some((middle, onto, size)) = ladder.landing_area() else { continue };
            let off = at - middle;
            let across = onto.cross(Vec3::Y);
            if off.dot(onto).abs() <= 0.5 * size.y && off.dot(across).abs() <= 0.5 * size.x && off.y > -LANDING_BELOW {
                return Some(super::ground::GroundHit { height: middle.y, normal: Vec3::Y });
            }
        }
        self.under.sample(at)
    }

    fn blocks(&self, point: Vec3, low: f32, high: f32) -> bool {
        let landing = self.ladders.iter().filter_map(Ladder::landing_area).any(|(middle, onto, size)| {
            let off = point - middle;
            let across = onto.cross(Vec3::Y);
            off.dot(onto).abs() <= 0.5 * size.y && off.dot(across).abs() <= 0.5 * size.x && middle.y > low
        });
        landing || self.under.blocks(point, low, high)
    }
}

/// How far below a landing a point still stands on it, metres: a foot
/// reaching for it, the root a step down.
const LANDING_BELOW: f32 = 0.3;

/// What a walker is asked to do on a ladder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Climb {
    /// Get on (walking there first) and climb up, to hold on near the top.
    Up,
    /// Climb down and step off onto the floor.
    Down,
    /// Slide down the rails to the floor, hands and feet gripping them, and
    /// step back off. Once on the rails it slides all the way down,
    /// whatever is asked.
    Slide,
}

/// What a limb holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Hold {
    /// A hand hanging as the standing pose has it.
    Free,
    /// A foot on the floor.
    Floor,
    /// A rung, by its index.
    Rung(i32),
    /// Its rail, sliding down it: a hand above its shoulder, a foot pressed
    /// to the rail's outside below the hips.
    Rail,
    /// A foot down on the floor beside its rail, a slide landed.
    Landed,
    /// A foot on the landing at the top, at its edge between the rails,
    /// counted the rung above the top one (its index).
    Top(i32),
    /// A foot on the landing where the walk to the ladder left it.
    Landing(i32),
}

impl Hold {
    /// The rung index a foot is at, the floor one below the lowest, the
    /// landing one above the highest.
    fn index(self) -> i32 {
        match self {
            Hold::Rung(i) | Hold::Top(i) | Hold::Landing(i) => i,
            Hold::Floor | Hold::Landed => -1,
            Hold::Free => i32::MIN / 2,
            // A hand sliding on its rail is above any rung it follows a
            // foot down to.
            Hold::Rail => i32::MAX / 2,
        }
    }
}

/// The limbs, as indexed in [`Climbing::holds`].
const LEFT_HAND: usize = 0;
const LEFT_FOOT: usize = 2;
const fn hand(side: usize) -> usize {
    LEFT_HAND + side
}
const fn foot(side: usize) -> usize {
    LEFT_FOOT + side
}
/// Each side's sign along the climber's left.
const SIGN: [f32; 2] = [1.0, -1.0];
const LEGS: [(Bone, Bone, Bone, Bone); 2] =
    [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase)];
const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];

/// How far a knee joint keeps from a rung's axis and a rail's middle line,
/// metres: a knee's half thickness and the bar's
/// ([`Climbing::bow_for`]).
const KNEE_CLEAR: f32 = 0.055;
/// The hips' bows tried for a step, metres ([`Climbing::bow_for`]), and at
/// how many moments through it the knees are judged: at 10, a swinging
/// knee passed through a rung between two of them.
const BOWS: [f32; 7] = [0.0, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15];
const BOW_SAMPLES: usize = 30;
/// How far up the body rises climbing, m/s: 0.39-0.44 measured on rungs
/// 0.3-0.36 m apart (Simeonov et al. 2020).
pub const SPEED_UP: f32 = 0.42;
/// How fast it comes down, m/s: 9 % slower than up (Simeonov et al. 2020).
pub const SPEED_DOWN: f32 = 0.38;
/// The shortest step, seconds: a foot moved in about half of it.
const SHORTEST_STEP: f32 = 0.6;
/// Sliding down the rails: speeding up and braking at most this, m/s², a
/// grip's friction holding back 60 % of the body's weight; at most this
/// fast, m/s; and landing this fast, m/s, the knees taking the rest.
const SLIDE_ACCELERATION: f32 = 4.0;
const SLIDE_SPEED: f32 = 2.0;
const SLIDE_LANDING: f32 = 1.0;
/// Taking the rails from the rungs, seconds; and stepping back off them,
/// landed.
const GRIP_SECONDS: f32 = 1.2;
const STEP_BACK_SECONDS: f32 = 1.1;
/// Sliding, each hand on its rail this far above its shoulder, of the
/// arm's length; each foot's ball this far below the hips, of the leg's,
/// its inside edge against the rail's outside, the toes turned out by
/// [`RAIL_FOOT_TURN`] radians.
const RAIL_HAND_ABOVE: f32 = 0.1;
const RAIL_FEET_BELOW: f32 = 0.72;
const RAIL_FOOT_TURN: f32 = 0.5;
/// A rail's middle, out from the rungs' ends, metres (it is 4 cm across);
/// and how far beside it a foot's ball sits.
const RAIL_MIDDLE: f32 = 0.02;
const RAIL_FOOT_OUT: f32 = 0.07;
/// How far the knees let the hips sink taking a slide's landing, of the
/// leg's length.
const ABSORB: f32 = 0.11;
/// How high a foot lifts stepping across the floor, metres.
const STEP_LIFT: f32 = 0.06;
/// The most the hips decelerate stopping mid-step, m/s².
const BRAKING: f32 = 2.5;
/// Reaching for the rungs from the floor, and letting go back on it,
/// seconds.
const REACH_SECONDS: f32 = 1.1;
const RELEASE_SECONDS: f32 = 0.9;

/// How far a foot's lift reaches as it leaves its hold, the hip socket to
/// the ankle as a fraction of the leg's length: the lowest foot just short of
/// straight, which sets the hips' height.
const REACH: f32 = 0.94;
/// The hips' distance out from the ladder's rungs, of the leg's length:
/// standing at the spot to get on, sliding, and the farthest climbing.
const HIPS_OUT: f32 = 0.55;
/// Climbing, the hips come in from [`HIPS_OUT`] as far as holds the hands
/// highest ([`Climbing::pattern_and_hips_for`]), but no nearer than this, of
/// the leg's length. Held at `HIPS_OUT`, the hands' rung was the one at the
/// chest (the arms are short for the rig's height).
const HIPS_IN_MOST: f32 = 0.36;
/// Over how many rungs the hips ease back out to [`HIPS_OUT`] at the top
/// ([`Climbing::top_out`]).
const TOP_EASE: f32 = 2.0;
/// How far the trunk leans toward the ladder, radians, on top of its lean.
const LEAN: f32 = 0.0;
/// A foot moves this far up at most, of the leg's length: past it, both
/// feet go to each rung.
const FOOT_RISE_MOST: f32 = 0.85;
/// The foot rise preferred, of the leg's length: what chooses how far a
/// foot passes the other.
const FOOT_RISE: f32 = 0.6;
/// How far above its eyes it looks at the ladder going up, and below them
/// going down, metres. The ladder is a third of a metre in front of the
/// eyes: half a metre below them bowed the head 55°, chin to chest.
const LOOK_UP: f32 = 0.3;
const LOOK_DOWN: f32 = 0.15;
/// The most the trunk leans in to bring a high hand within reach, radians.
const MOST_LEAN: f32 = 0.6;
/// The farthest a hand reaches from its shoulder, of the arm's length.
const ARM_REACH: f32 = 0.95;
/// The hands grip the highest rung whose wrist is at most this far from the
/// (lifted) shoulder, of the arm's length, the trunk at its lean: the arms
/// long, the hands well above the feet. Gripped nearest the shoulder
/// instead, the hands came down to the waist and the trunk folded in over
/// them.
const HAND_STRETCH: f32 = 0.85;
/// A hand farther than this from its shoulder, of the arm's length, lifts
/// the shoulder toward it ([`shoulder_lift`]), as a reach overhead raises
/// the shoulder girdle.
const LIFT_FROM: f32 = 0.85;
/// A hand rung whose wrist is within this of [`HAND_STRETCH`], metres, is
/// within it (`Climbing::stretched`): one the lift brings there is exactly
/// at it, give or take float noise.
const STRETCH_SLACK: f32 = 1.0e-4;
/// Feet passing each other are taken only if their best hips hold the hands
/// no more than this lower than the best of all, metres
/// (`Climbing::pattern_and_hips_for`).
const PASSING_LOWER: f32 = 0.02;
/// Each side's clavicle: turning it swings the shoulder joint about its root.
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
/// How far through its move a hand starts closing on what it reaches for,
/// so its fingers (`hand::CLOSE_RATE`) shut as it arrives.
const CLOSE_AT: f32 = 0.8;
/// How square to its rail a hand holds it: the share of its reach along
/// the rail taken out of the way its fingers point (1 square, 0 along).
const RAIL_SQUARE: f32 = 0.25;
/// How far below its rail's top a hand sliding on it keeps, metres: a
/// hand's width.
const RAIL_HAND_BELOW_TOP: f32 = 0.05;
/// How fast the feet shuffle back to a landing's edge to get on, m/s.
const SHUFFLE_SPEED: f32 = 0.4;
/// Where the hips stand on a landing, metres past the rungs; and with one
/// foot on it, the other on the top rungs, metres out from them: through
/// the rails, over the ladder's top.
const TOP_IN: f32 = 0.1;
/// Where a walk to a landing's spot ends, metres in from it: clear of the
/// edge as it turns round to put its back to the ladder; getting on, the
/// feet shuffle back the rest.
pub const TOP_APPROACH_IN: f32 = 0.3;
const TOP_HIPS_OUT: f32 = 0.12;
/// Where a hand without known fingers closes, metres along it from the
/// wrist: about the knuckles.
const GUESSED_KNUCKLES: f32 = 0.08;
/// Which way a gripping arm's elbow bends: out to its side, down, and back
/// toward the climber. Half as far out as back, a high hand's elbow stuck
/// out sideways at shoulder height.
const ELBOW_POLE: (f32, f32, f32) = (0.0, 0.6, 0.6);
/// The same holding a rail ([`Climbing::elbow_pole`]): out to its side.
const RAIL_ELBOW_POLE: (f32, f32, f32) = (0.8, 0.3, 0.4);
/// How far a gripping hand tips over its bar off the line of its reach,
/// radians: the wrist flexed a little, as round a bar held.
const GRIP_FLEX: f32 = 0.3;
/// How far out a moving hand's rung keeps clear, metres: the fingers off
/// the rungs it passes.
const HAND_CLEAR: f32 = 0.08;
/// How far a moving foot's ball comes out beyond its toe tip, metres.
const TOE_CLEAR: f32 = 0.06;
/// How a step goes one way: when its foot moves and then a hand, and how
/// the hips' rate swings about its mean through it.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Beat {
    /// The shares of the step the foot moves in, and a hand after it.
    foot: (f32, f32),
    hand: (f32, f32),
    /// The hips go at `1 - modulation·sin(2π(t - phase))` times their mean
    /// rate at `t` through the step.
    modulation: f32,
    phase: f32,
}

/// Up: the hips slowest while the foot swings, fastest pushing up on it
/// once down, as the next hand goes up.
const UP: Beat = Beat { foot: (0.0, 0.55), hand: (0.58, 0.92), modulation: 0.6, phase: 0.02 };
/// Down: the hand of the foot that went down last follows it first, the
/// hips slowest; then the foot reaches down as the hips lower onto the
/// other leg, landing as they arrive. With the hand after the foot and the
/// step ending at the hand, the foot came down on its rung before the hips
/// had, 2-9 cm short of it.
const DOWN: Beat = Beat { foot: (0.32, 1.0), hand: (0.0, 0.3), modulation: 0.8, phase: -0.1 };

/// How a ladder is climbed, chosen by its spacing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pattern {
    /// Each foot moves past the other onto the rung this many above it, the
    /// feet as many rungs apart; 0 both feet to each rung.
    pub gap: i32,
    /// How many rungs above its foot's a hand grips.
    pub hand: i32,
}

impl Pattern {
    /// Where a foot at `this` goes, the other at `other`, going `up`.
    fn next(&self, this: i32, other: i32, up: bool) -> i32 {
        let dir = if up { 1 } else { -1 };
        if self.gap > 0 {
            other + dir * self.gap
        } else if (this - other) * dir < 0 {
            other
        } else {
            other + dir
        }
    }
}

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    /// The hips joint, the pose's frame.
    hips: Vec3,
    /// Each hip socket and shoulder, and clavicle's root, from the hips
    /// joint.
    sockets: [Vec3; 2],
    shoulders: [Vec3; 2],
    clavicles: [Vec3; 2],
    /// Each leg's thigh and shank, and arm's upper arm and forearm, end to
    /// end.
    legs: [f32; 2],
    arms: [f32; 2],
    /// Each foot's ball (toe joint), and its ankle from the ball.
    balls: [Vec3; 2],
    ankles: [Vec3; 2],
    /// How far the toe tip is ahead of the ball, and the ball above the
    /// floor.
    toe: f32,
    ball_height: f32,
    /// Each foot's world rotation.
    attitudes: [Quat; 2],
    /// Each hand's bind rotation, and each arm's upper arm and forearm.
    hand_binds: [Quat; 2],
    arm_bones: [(f32, f32); 2],
    /// How each hand holds a bar, in the rest pose's frame (the T-pose's).
    grips: [super::hand::HandGrip; 2],
    /// The head joint, from the hips joint.
    eyes: Vec3,
}

impl Body {
    fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let at = forward_kinematics_on(stood, rig);
        let world = accumulate_world_rotations(stood, rig);
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let binds = accumulate_bind_rotations(rig);
        let hips = at[Bone::Hips];
        let tips = LEGS.map(|(_, _, ankle, _)| super::foot::Sole::of(rig, ankle).points(stood, rig)[2] + hips);
        let balls = LEGS.map(|(_, _, _, toe)| at[toe]);
        Self {
            stood: *stood,
            hips,
            sockets: LEGS.map(|(socket, _, _, _)| at[socket] - hips),
            shoulders: ARMS.map(|arm| at[arm.shoulder] - hips),
            clavicles: CLAVICLES.map(|clavicle| at[clavicle] - hips),
            legs: LEGS.map(|(socket, knee, ankle, _)| (at[knee] - at[socket]).length() + (at[ankle] - at[knee]).length()),
            arms: ARMS.map(|arm| (at[arm.elbow] - at[arm.shoulder]).length() + (at[arm.wrist] - at[arm.elbow]).length()),
            balls,
            ankles: [0, 1].map(|i| at[LEGS[i].2] - balls[i]),
            toe: (tips[0] - balls[0]).length(),
            ball_height: balls[0].y.min(balls[1].y),
            attitudes: LEGS.map(|(_, _, ankle, _)| world[ankle]),
            hand_binds: ARMS.map(|arm| binds[arm.wrist]),
            arm_bones: ARMS.map(|arm| ((at[arm.elbow] - at[arm.shoulder]).length(), (at[arm.wrist] - at[arm.elbow]).length())),
            // Until the fingers are known (`Climbing::set_grips`): the hand
            // on from the forearm, its rest palm down (a T-pose's), the bar
            // across it at the knuckles.
            grips: ARMS.map(|arm| {
                let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
                let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
                super::hand::HandGrip { bar: along * GUESSED_KNUCKLES + palm * (super::hand::GRIP_RADIUS + super::hand::FINGER_HALF_THICKNESS), palm, along }
            }),
            eyes: at[Bone::Head] - hips,
        }
    }
}

/// The hips' way through a step: from `from` to `to` over `duration`
/// seconds, leaving at `starts` and arriving at `ends` m/s; or, `steady`,
/// at a rate that swings `modulation` about the mean through the step.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Path {
    from: Vec3,
    to: Vec3,
    /// When it starts, seconds into the step, and how long it takes.
    start: f32,
    duration: f32,
    starts: Vec3,
    ends: Vec3,
    /// The step's beat, steady: its rate swinging through it.
    steady: Option<Beat>,
    /// Sliding: the way gone at the slide's speeds instead.
    slide: Option<Slide>,
}

/// A slide down the rails, `length` metres: from rest at `accel` m/s² up to
/// `top` m/s, braked at `accel` to `land` m/s as it reaches the floor.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Slide {
    accel: f32,
    top: f32,
    land: f32,
    length: f32,
}

impl Slide {
    /// Down `length` metres from rest, at most [`SLIDE_SPEED`], landing at
    /// [`SLIDE_LANDING`].
    fn down(length: f32) -> Self {
        let (accel, land) = (SLIDE_ACCELERATION, SLIDE_LANDING);
        // As fast as speeding up and braking leaves room for.
        let top = SLIDE_SPEED.min(((2.0 * accel * length + land * land) / 2.0).sqrt());
        // Too short to brake: it lands at the speed it has gathered.
        let land = land.min((2.0 * accel * length).sqrt());
        Self { accel, top: top.max(land), land, length }
    }

    /// How long speeding up, gliding and braking each take, seconds.
    fn phases(&self) -> (f32, f32, f32) {
        let up = self.top / self.accel;
        let brake = (self.top - self.land) / self.accel;
        let gone = 0.5 * self.top * up + 0.5 * (self.top + self.land) * brake;
        (up, ((self.length - gone) / self.top).max(0.0), brake)
    }

    fn duration(&self) -> f32 {
        let (up, glide, brake) = self.phases();
        up + glide + brake
    }

    /// How far down it has gone at `t` seconds.
    fn gone(&self, t: f32) -> f32 {
        let (up, glide, brake) = self.phases();
        let t = t.clamp(0.0, up + glide + brake);
        if t <= up {
            0.5 * self.accel * t * t
        } else if t <= up + glide {
            0.5 * self.top * up + self.top * (t - up)
        } else {
            let b = t - up - glide;
            0.5 * self.top * up + self.top * glide + self.top * b - 0.5 * self.accel * b * b
        }
        .min(self.length)
    }
}

impl Beat {
    /// The share of the way along a steady step at `t` of it.
    fn share(&self, t: f32) -> f32 {
        t + self.modulation * ((TAU * (t - self.phase)).cos() - (TAU * self.phase).cos()) / TAU
    }

    /// The rate a steady step leaves and arrives at, as a multiple of its
    /// mean.
    fn rate(&self) -> f32 {
        1.0 + self.modulation * (TAU * self.phase).sin()
    }
}

impl Path {
    fn at(&self, t: f32) -> Vec3 {
        if let Some(slide) = self.slide {
            return self.from + (self.to - self.from) * (slide.gone(t - self.start) / slide.length.max(1.0e-6));
        }
        let s = ((t - self.start) / self.duration.max(1.0e-6)).clamp(0.0, 1.0);
        match self.steady {
            Some(beat) => self.from + (self.to - self.from) * beat.share(s),
            None => {
                let d = self.duration;
                let axis = |a: fn(Vec3) -> f32| hermite(a(self.from), a(self.to), a(self.starts) * d, a(self.ends) * d, s);
                Vec3::new(axis(|v| v.x), axis(|v| v.y), axis(|v| v.z))
            }
        }
    }

    fn velocity(&self, t: f32) -> Vec3 {
        let h = 1.0e-3;
        (self.at(t + h) - self.at(t - h)) / (2.0 * h)
    }
}

/// What kind of step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// From the floor, the hands reach for their rungs.
    Reach,
    Up,
    Down,
    /// Back on the floor, the hands let go.
    Release,
    /// The hands, then the feet, from the rungs onto the rails.
    Grip,
    /// Down the rails.
    Slide,
    /// The feet on the floor, the knees taking the landing.
    Land,
    /// Off the floor beside the rails, back to where it got on, letting go.
    StepBack,
}

/// One limb going from one hold to another over `window` of a step.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Move {
    limb: usize,
    from: Hold,
    to: Hold,
    window: (f32, f32),
}

/// One step: its limbs' moves, and the hips', the trunk's lean and the
/// body's turn to square up.
#[derive(Debug, Clone, PartialEq)]
struct Step {
    kind: Kind,
    duration: f32,
    moves: Vec<Move>,
    hips: Path,
    /// How far the hips bow out from the ladder through the step, metres at
    /// its middle ([`Climbing::bow_for`]).
    bow: f32,
    lean: (f32, f32),
    turn: (f32, f32),
    sink: (f32, f32),
}

/// A walker on a ladder, from getting on to stepping off.
#[derive(Debug, Clone)]
pub struct Climbing {
    /// The ladder in the climb's frame, and in the world.
    local: Ladder,
    world: Ladder,
    /// The climb's frame: where the root stood getting on, and the facing
    /// that squares up to the ladder.
    origin: Vec3,
    yaw: f32,
    body: Body,
    pub pattern: Pattern,
    /// How far out from the rungs the hips go on the ladder, metres
    /// ([`HIPS_IN_MOST`]).
    hips_out: f32,
    /// The rig it was measured on, to pose steps planned ahead on
    /// ([`Climbing::bow_for`]).
    rig: std::sync::Arc<RigGeometry>,
    /// The bows of the steps planned lately, shared with its copies run
    /// ahead (`pose_led`): planned again by each, a climb cost 744 µs a
    /// frame.
    bows: std::sync::Arc<std::sync::Mutex<Vec<(BowKey, f32)>>>,
    /// What each limb holds as the step began.
    holds: [Hold; 4],
    /// Each foot's floor spot (its ball) and its turn about `+Y`.
    floor: [(Vec3, f32); 2],
    /// Where the root stands on the floor to get on or step off; and on a
    /// landing at the top, between the rails at its edge; and each foot's
    /// ball and turn on the landing where the walk there left it.
    spot: Vec3,
    top_spot: Vec3,
    top_floor: [(Vec3, f32); 2],
    step: Option<Step>,
    elapsed: f32,
    /// The hips standing still: where, at the end of the last step.
    rest: Vec3,
    lean: f32,
    turn: f32,
    /// How far the hips sit below the standing pose's, standing on the
    /// floor: the foot IK's pelvis drop standing ([`Self::new`]'s `drop`),
    /// eased out getting on and in stepping off.
    sink: f32,
    drop: f32,
    /// The foot that moved last: the other goes next when both are level.
    last_foot: usize,
    /// What it was last asked.
    ask: Option<Climb>,
    /// Every limb has let go: back on the floor.
    done: bool,
}

impl Climbing {
    /// Getting on `ladder` from the floor, standing in `stood` (on `rig`) at
    /// `root`, facing `facing` (radians about `+Y`); `square` is the facing
    /// that faces the ladder. `drop` is how far the foot IK has the hips
    /// below `stood`'s standing (`AnimFootIk::pelvis_drop`): the climb
    /// starts and ends there. Left out, the hips popped 7.6 mm as the foot
    /// IK let go of the legs getting on and took them back stepping off.
    #[allow(clippy::too_many_arguments)]
    pub fn new(ladder: &Ladder, root: Vec3, facing: f32, square: f32, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let turn = Quat::from_rotation_y(square);
        let local = ladder.in_frame(root, turn);
        let body = Body::of(stood, rig);
        let leg = leg_length_of(rig);
        let offset = super::facing::shortest_angle(facing - square);
        let twist = Quat::from_rotation_y(offset);
        // Standing on its landing, it gets on from the top.
        let on_top = local.landing_height().is_some_and(|height| height.abs() < 0.3);
        let standing = [0, 1].map(|i| (twist * body.balls[i], offset));
        let feet = if on_top { Hold::Landing(local.top() + 1) } else { Hold::Floor };
        let mut climbing = Self {
            local,
            world: *ladder,
            origin: root,
            yaw: square,
            body: body.clone(),
            pattern: Pattern { gap: 1, hand: 4 },
            hips_out: HIPS_OUT * leg,
            rig: std::sync::Arc::new(rig.clone()),
            bows: Default::default(),
            holds: [Hold::Free, Hold::Free, feet, feet],
            floor: standing,
            spot: Vec3::ZERO,
            top_spot: Vec3::ZERO,
            top_floor: standing,
            step: None,
            elapsed: 0.0,
            rest: twist * body.hips - Vec3::Y * drop,
            lean: 0.0,
            turn: offset,
            sink: drop,
            drop,
            last_foot: 0,
            ask: None,
            done: false,
        };
        climbing.spot = climbing.hips_square(local.base.y + body.hips.y) - body.hips;
        if let Some(height) = local.landing_height() {
            let hips = local.middle_at(height + body.hips.y) - local.out() * TOP_IN;
            climbing.top_spot = Vec3::new(hips.x, height + body.hips.y, hips.z) - body.hips;
        }
        // Got on from the top, a foot comes down on the floor where standing
        // at the spot puts it, square to the ladder.
        if on_top {
            for side in 0..2 {
                climbing.floor[side] = (climbing.spot + body.balls[side], 0.0);
            }
        }
        (climbing.pattern, climbing.hips_out) = climbing.pattern_and_hips_for(leg);
        climbing
    }

    /// Where a walker's root stands on `ladder`'s landing to get on from
    /// the top, facing `square` (radians about `+Y`, its back to the
    /// ladder): its hips [`TOP_IN`] in from the rungs, square to them.
    /// `None` without a landing.
    pub fn top_spot(ladder: &Ladder, square: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Vec3> {
        let height = ladder.landing_height()?;
        let hips = forward_kinematics_on(stood, rig)[Bone::Hips];
        let at = ladder.middle_at(height + hips.y) - ladder.out() * TOP_IN;
        let spot = at - Quat::from_rotation_y(square) * hips;
        Some(Vec3::new(spot.x, height, spot.z))
    }

    /// Where a walker's root stands to get on `ladder`, facing `square`
    /// (radians about `+Y`): its hips [`HIPS_OUT`] from the rungs, square in
    /// front of them.
    pub fn spot(ladder: &Ladder, square: f32, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
        let hips = forward_kinematics_on(stood, rig)[Bone::Hips];
        let at = ladder.middle_at(ladder.base.y + hips.y) + ladder.out() * (HIPS_OUT * leg_length_of(rig));
        let spot = at - Quat::from_rotation_y(square) * hips;
        Vec3::new(spot.x, ladder.base.y, spot.z)
    }

    /// The pattern for this ladder's spacing on a leg `leg` long, and how far
    /// out the hips go, from [`HIPS_OUT`] in to [`HIPS_IN_MOST`]: the one
    /// that holds the hands highest, the farthest out of equals; of those
    /// whose feet pass each other as the spacing has them
    /// ([`Self::gap_for`]) if the best of them holds the hands within
    /// [`PASSING_LOWER`] of that. The knees keep clear however near the hips
    /// come (`bow_for`).
    ///
    /// Chosen by the hands alone, the feet's way flipped with the hips' (the
    /// rungs reach in steps): a ladder whose feet pass climbed both feet to
    /// each rung. Chosen as the farthest out keeping the hands within a share
    /// of the arm below the shoulders, rungs 0.45 m apart were held at the
    /// chest. Passing whenever any distance let it, rungs 0.36 m apart were
    /// climbed so with the hips at their nearest, the hands 4.9 cm lower, the
    /// trunk leant 0.49 rad in and a knee 2 cm from a rung; where the feet
    /// pass on the other ladders, they hold the hands highest at every
    /// distance.
    fn pattern_and_hips_for(&self, leg: f32) -> (Pattern, f32) {
        const TRIED: usize = 20;
        let tried: Vec<(Pattern, f32, f32)> = (0..TRIED)
            .map(|i| {
                let mut trial = self.clone();
                trial.hips_out = leg * (HIPS_OUT + (HIPS_IN_MOST - HIPS_OUT) * i as f32 / (TRIED - 1) as f32);
                let pattern = trial.pattern_for(leg);
                (pattern, trial.hips_out, trial.hand_height(&pattern))
            })
            .collect();
        // Ties to the farthest out: the first of equals, tried from out in.
        let best = |from: &[(Pattern, f32, f32)]| {
            from.iter().copied().fold(None::<(Pattern, f32, f32)>, |best, candidate| match best {
                Some(best) if best.2 >= candidate.2 - 1.0e-4 => Some(best),
                _ => Some(candidate),
            })
        };
        let gap = self.gap_for(leg);
        let keeping: Vec<_> = tried.iter().filter(|(pattern, _, _)| pattern.gap == gap).copied().collect();
        let overall = best(&tried);
        let passing = best(&keeping).filter(|passing| overall.is_none_or(|overall| passing.2 >= overall.2 - PASSING_LOWER));
        passing.or(overall).map_or((self.pattern, self.hips_out), |(pattern, out, _)| (pattern, out))
    }

    /// How many rungs apart the feet go on this ladder's spacing, a leg `leg`
    /// long, reach aside: 0 both feet to each rung.
    fn gap_for(&self, leg: f32) -> i32 {
        let spacing = self.local.spacing.max(0.05);
        if 2.0 * spacing > FOOT_RISE_MOST * leg { 0 } else { ((FOOT_RISE * leg / (2.0 * spacing)).round() as i32).max(1) }
    }

    /// How far above its (unlifted) shoulder a hand climbing by `pattern`
    /// holds its rung through its hold, metres: the middle of where it is as
    /// the feet below it stand on taking it and on letting it go. Judged at
    /// one end, both feet to each rung picked a rung only just in reach.
    fn hand_height(&self, pattern: &Pattern) -> f32 {
        let low = 4;
        let (taken, left, rung) = match pattern.gap {
            0 => ([low; 2], [low + 1; 2], low + 1 + pattern.hand),
            g => ([low, low + g], [low + g, low + 2 * g], low + g + pattern.hand),
        };
        let above = |feet: [i32; 2]| {
            let shoulder = self.hips_at(feet.map(Hold::Rung)) + Quat::from_axis_angle(self.local.left(), LEAN + self.local.lean) * self.body.shoulders[0];
            (self.grip_at(0, rung) - shoulder).dot(self.local.up())
        };
        0.5 * (above(taken) + above(left))
    }

    /// The pattern for this ladder's spacing on a leg `leg` long, the hips
    /// [`Self::hips_out`] from the rungs.
    fn pattern_for(&self, leg: f32) -> Pattern {
        let gap = self.gap_for(leg);
        if gap == 0 {
            return self.together();
        }
        // Climbing either way, a hand is between `hand` and `hand + 2·gap`
        // rungs above the lower foot: the highest just moved, at a step's
        // start, the lowest about to move, at its end, the body a step
        // higher. The highest within `HAND_STRETCH` at the start.
        let g = gap;
        let low = 4;
        let (hips, trunk) = (self.hips_at([Hold::Rung(low), Hold::Rung(low + g)]), Quat::from_axis_angle(self.local.left(), LEAN + self.local.lean));
        let hand = self.stretched(|hand| self.reach_to(0, hips, trunk, self.grip_at(0, low + 2 * g + hand)));
        // Past what leaning in brings within reach (`reaching`), each foot to
        // the rung above the other is too far for the hands: both feet to
        // each rung, as rungs too far apart for a foot. Judged by a fixed
        // share of the arm, the wrist where a real hand's fingers close (on
        // `puppet_base`, farther from the shoulder than an estimated one)
        // put 0.3 m rungs past it.
        let reached = |feet: [i32; 2], rung: i32| self.within_reach(0, rung, self.hips_at(feet.map(Hold::Rung)));
        if !(reached([low, low + g], low + 2 * g + hand) && reached([low + g, low + 2 * g], low + g + hand)) {
            return self.together();
        }
        Pattern { gap, hand }
    }

    /// Both feet to each rung, and each hand: the hands' rung the highest
    /// within `HAND_STRETCH` as the feet stand level.
    fn together(&self) -> Pattern {
        let low = 4;
        let (hips, trunk) = (self.hips_at([Hold::Rung(low); 2]), Quat::from_axis_angle(self.local.left(), LEAN + self.local.lean));
        let hand = self.stretched(|hand| self.reach_to(0, hips, trunk, self.grip_at(0, low + 1 + hand)));
        Pattern { gap: 0, hand }
    }

    /// The highest hand rung (above its foot's) whose wrist, `reach(hand)`
    /// from the (lifted) shoulder, is within [`HAND_STRETCH`] of the arm; the
    /// nearest if none is.
    ///
    /// A rung the shoulder's lift brings within it is lifted to exactly
    /// `LIFT_FROM` of the arm, the same share: compared bare, which side of
    /// it such a rung fell was float noise (±1e-7 m), and a change of 1e-6 to
    /// a turn in the standing pose flipped a ladder's whole pattern. Within
    /// [`STRETCH_SLACK`] counts.
    fn stretched(&self, reach: impl Fn(i32) -> f32) -> i32 {
        let most = HAND_STRETCH * self.body.arms[0] + STRETCH_SLACK;
        (0..24)
            .rev()
            .find(|&hand| reach(hand) <= most)
            .unwrap_or_else(|| (0..24).min_by(|&a, &b| reach(a).total_cmp(&reach(b))).unwrap_or(0))
    }

    /// Where the hips go at height `y`: [`HIPS_OUT`] out from the rungs, in
    /// line with the ladder's middle.
    fn hips_square(&self, y: f32) -> Vec3 {
        let at = self.local.middle_at(y) + self.local.out() * (HIPS_OUT * self.body.legs[0]);
        Vec3::new(at.x, y, at.z)
    }

    /// A foot's ball on `hold`, and its turn about `+Y`, the hips at `hips`.
    fn foot_at(&self, side: usize, hold: Hold, hips: Vec3) -> (Vec3, f32) {
        match hold {
            Hold::Rung(i) => {
                let lane = self.body.sockets[side].x.abs().min(0.5 * self.local.width - 0.06).max(0.0);
                (self.local.rung(i) + self.local.left() * SIGN[side] * lane + Vec3::Y * (Ladder::RUNG_RADIUS + self.body.ball_height), 0.0)
            }
            Hold::Rail => {
                let at = self.local.middle_at(hips.y - RAIL_FEET_BELOW * self.body.legs[side]);
                (at + self.local.left() * SIGN[side] * (0.5 * self.local.width + RAIL_FOOT_OUT), SIGN[side] * RAIL_FOOT_TURN)
            }
            Hold::Landed => self.foot_at(side, Hold::Rail, self.touchdown()),
            Hold::Top(_) => (self.top_spot + self.body.balls[side], 0.0),
            Hold::Landing(_) => self.top_floor[side],
            _ => self.floor[side],
        }
    }

    /// A hand's wrist on rung `i`.
    fn grip_at(&self, side: usize, i: i32) -> Vec3 {
        self.hand_at(side, Hold::Rung(i), self.rest).expect("a rung is held")
    }

    /// How hand `side` holds `hold`, the hips at `hips`: a point on the
    /// bar's axis (the rung's or the rail's middle), the way its fingers
    /// point and its palm faces; `None` hanging.
    ///
    /// A hand closed round a bar can roll round it, so it holds it the way
    /// the arm comes at it: its fingers on from the shoulder toward the bar
    /// (square to the bar), tipped [`GRIP_FLEX`] over it, the palm onto it.
    /// A rung overhead is held palm to the ladder, one at the chest palm
    /// down, rolling round as the body climbs past. Held one fixed way (the
    /// fingers into the ladder, tipped down over the rung), the hand bent
    /// 97-111° off a forearm coming up from below.
    fn grip_frame(&self, side: usize, hold: Hold, hips: Vec3) -> Option<(Vec3, Vec3, Vec3)> {
        let (left, normal) = (self.local.left(), self.local.normal());
        let (bar, axis) = match hold {
            // Past the top rung, the rail going on above a landing, at the
            // height that rung would be.
            Hold::Rung(i) if i > self.local.top() => {
                let at = self.local.middle_at(self.local.rung(i).y);
                (at + left * SIGN[side] * (0.5 * self.local.width + RAIL_MIDDLE), self.local.up())
            }
            Hold::Rung(i) => {
                let lane = (self.body.shoulders[side] - self.body.sockets[side]).x.abs().max(self.body.shoulders[side].x.abs()).min(0.5 * self.local.width - 0.07).max(0.0);
                (self.local.rung(i) + left * SIGN[side] * lane, left)
            }
            // Sliding with the body, below the rails' tops.
            Hold::Rail => {
                let rails_top = self.local.landing_height().map_or(self.local.rung(self.local.top()).y + self.local.spacing, |height| height + Ladder::RAILS_ABOVE);
                let height = (hips.y + self.body.shoulders[side].y + RAIL_HAND_ABOVE * self.body.arms[side]).min(rails_top - RAIL_HAND_BELOW_TOP);
                let at = self.local.middle_at(height);
                (at + left * SIGN[side] * (0.5 * self.local.width + RAIL_MIDDLE), self.local.up())
            }
            _ => return None,
        };
        let trunk = Quat::from_axis_angle(left, LEAN + self.local.lean);
        let mut shoulder = self.lifted(side, hips, trunk, bar);
        let on_rail = self.on_rail(hold);
        let toward = if on_rail { -left * SIGN[side] - normal } else { -normal - Vec3::Y };
        // The frame for fingers coming in along `reach`: the palm onto the
        // bar, square to the reach and the bar, on the ladder's side of the
        // reach (forward of an upward reach, under a forward one; on a rail,
        // toward the ladder's middle); tipped over the bar.
        // A rail across the palm on a slant, the fingers following the
        // forearm part of the way along it: square to it, a hand on a rail
        // beside the hips, its forearm hanging, bent 120-160° off it.
        let square = if on_rail { RAIL_SQUARE } else { 1.0 };
        let frame = |reach: Vec3| {
            let reach = (reach - axis * reach.dot(axis) * square).normalize_or(-normal);
            let across = axis.cross(reach).normalize_or_zero();
            let palm = if across.dot(toward) >= 0.0 { across } else { -across };
            ((reach * GRIP_FLEX.cos() + palm * GRIP_FLEX.sin()).normalize(), (palm * GRIP_FLEX.cos() - reach * GRIP_FLEX.sin()).normalize())
        };
        // Along the forearm, as the arm IK will lay it: its elbow from the
        // shoulder, the wrist and the elbow's pole. From the shoulder toward
        // the bar first, then from the elbow that puts there.
        let pole = self.elbow_pole(side, on_rail);
        let (upper, fore) = self.body.arm_bones[side];
        let mut reach = bar - shoulder;
        for _ in 0..2 {
            let (line, palm) = frame(reach);
            let wrist = bar - self.hand_turn(side, line, palm) * self.body.grips[side].bar;
            shoulder = self.lifted(side, hips, trunk, wrist);
            let to_wrist = wrist - shoulder;
            let distance = to_wrist.length().clamp(1.0e-4, upper + fore);
            let direction = to_wrist.normalize_or(Vec3::NEG_Y);
            let out = (pole - direction * pole.dot(direction)).normalize_or_zero();
            let angle = ((upper * upper + distance * distance - fore * fore) / (2.0 * upper * distance)).clamp(-1.0, 1.0).acos();
            let elbow = shoulder + (direction * angle.cos() + out * angle.sin()) * upper;
            reach = wrist - elbow;
        }
        let (line, palm) = frame(reach);
        Some((bar, line, palm))
    }

    /// Which way arm `side`'s elbow bends holding a rung, or (`on_rail`) a
    /// rail, in the climb's frame. On a rail out to its side: down, the
    /// forearm hung along the rail beside the hips, and a hand cannot close
    /// round a bar running down its palm (the wrist bent 93-101°).
    fn elbow_pole(&self, side: usize, on_rail: bool) -> Vec3 {
        let (out, back) = (self.local.left() * SIGN[side], self.local.normal());
        let (sideways, down, behind) = if on_rail { RAIL_ELBOW_POLE } else { ELBOW_POLE };
        (out * sideways - Vec3::Y * down + back * behind).normalize()
    }

    /// Whether `hold` is a rail: one slid on, or above a landing at a rung's
    /// height.
    fn on_rail(&self, hold: Hold) -> bool {
        hold == Hold::Rail || matches!(hold, Hold::Rung(i) if i > self.local.top())
    }

    /// The world turn that carries hand `side` from its rest to point its
    /// fingers along `line`, its palm facing `palm`.
    fn hand_turn(&self, side: usize, line: Vec3, palm: Vec3) -> Quat {
        let rest = &self.body.grips[side];
        super::armik::frame_turn(rest.along, rest.palm, line, palm)
    }

    /// A hand's wrist on `hold`, the hips at `hips`: where it puts the bar
    /// its fingers close round ([`Body::grips`]) on the rung's or rail's
    /// axis; `None` hanging.
    fn hand_at(&self, side: usize, hold: Hold, hips: Vec3) -> Option<Vec3> {
        let (bar, line, palm) = self.grip_frame(side, hold, hips)?;
        Some(bar - self.hand_turn(side, line, palm) * self.body.grips[side].bar)
    }

    /// The hands' grips, from their fingers (`hand::RelaxedHands::grips`, in
    /// each hand's own frame, on `rig`): each hand placed so its fingers
    /// close round its rung. Without them, an estimate from the forearm.
    pub fn set_grips(&mut self, grips: [Option<super::hand::HandGrip>; 2], rig: &RigGeometry) {
        for (side, grip) in super::hand::bound_grips(grips, rig).into_iter().enumerate() {
            if let Some(grip) = grip {
                self.body.grips[side] = grip;
            }
        }
    }

    /// Where the hips land a slide: as high as puts the feet, on their
    /// rails, on the floor.
    fn touchdown(&self) -> Vec3 {
        self.hips_square(self.local.base.y + self.body.ball_height + RAIL_FEET_BELOW * self.body.legs[0])
    }

    /// Each foot's ankle on `holds`.
    fn ankle_at(&self, side: usize, hold: Hold) -> Vec3 {
        let (ball, turn) = self.foot_at(side, hold, self.rest);
        ball + Quat::from_rotation_y(turn) * self.body.ankles[side]
    }

    /// Where the hips stand with the feet on `feet`: square to the ladder,
    /// as high as lets the lower foot reach its hold at [`REACH`].
    ///
    /// Both feet on a landing, it stands there ([`Self::top_spot`]); one on
    /// it and one on the top rungs, the hips come forward over the ladder's
    /// top ([`TOP_HIPS_OUT`]), no higher than standing on the landing.
    fn hips_at(&self, feet: [Hold; 2]) -> Vec3 {
        let on_top = feet.iter().filter(|foot| matches!(foot, Hold::Top(_))).count();
        if on_top == 2 {
            return self.top_spot + self.body.hips - Vec3::Y * self.drop;
        }
        let (out, highest) = match on_top {
            1 => (TOP_HIPS_OUT, self.top_spot.y + self.body.hips.y - self.drop),
            _ => (self.top_out(feet), f32::MAX),
        };
        let place = |y: f32| {
            let at = self.local.middle_at(y) + self.local.out() * out;
            Vec3::new(at.x, y, at.z)
        };
        let ankles = [0, 1].map(|side| self.ankle_at(side, feet[side]));
        let fits = |y: f32| {
            let hips = place(y);
            y <= highest && (0..2).all(|side| (hips + self.body.sockets[side] - ankles[side]).length() <= REACH * self.body.legs[side])
        };
        let (mut low, mut high) = (ankles[0].y.min(ankles[1].y), ankles[0].y.max(ankles[1].y) + self.body.legs[0] * 1.2);
        if !fits(low) {
            return place(low);
        }
        for _ in 0..30 {
            let middle = 0.5 * (low + high);
            if fits(middle) { low = middle } else { high = middle }
        }
        place(low)
    }

    /// How far out the hips go with the feet on `feet`: [`Self::hips_out`],
    /// easing out to [`HIPS_OUT`] over [`TOP_EASE`] rungs as the feet climb
    /// past where the hands can follow (their rung above the top one held).
    /// Held in close there, the hands came down below the chest, the
    /// forearms hanging onto the top rung and the wrists bent 76° over it.
    fn top_out(&self, feet: [Hold; 2]) -> f32 {
        let rung = |foot: Hold| match foot {
            Hold::Rung(i) => i,
            _ => -1,
        };
        let past = (rung(feet[0]).min(rung(feet[1])) + self.pattern.hand - self.local.highest_grip()) as f32;
        let eased = super::gait::smoothstep((past / TOP_EASE).clamp(0.0, 1.0));
        self.hips_out + (HIPS_OUT * self.body.legs[0] - self.hips_out) * eased
    }

    /// The step from `holds` going `ask`, the last foot moved `last`, the
    /// hips at `hips`; `None` if there is none (holding on, or at the top).
    fn plan(&self, holds: [Hold; 4], last: usize, ask: Option<Climb>) -> Option<(Kind, Vec<Move>, [Hold; 4])> {
        let feet = [holds[foot(0)], holds[foot(1)]];
        let on_floor = feet == [Hold::Floor; 2];
        let on_top = feet.iter().all(|foot| matches!(foot, Hold::Top(_)));
        let on_landing = feet.iter().all(|foot| matches!(foot, Hold::Landing(_)));
        let on_rungs = feet.iter().all(|foot| matches!(foot, Hold::Rung(_)));
        let free = holds[hand(0)] == Hold::Free && holds[hand(1)] == Hold::Free;
        let top = self.local.top();
        let highest = self.local.highest_grip();
        let mut after = holds;
        // On the rails it slides on down, lands and steps back off, whatever
        // is asked.
        if feet == [Hold::Rail; 2] {
            if self.rest.y > self.touchdown().y + 1.0e-3 {
                return Some((Kind::Slide, Vec::new(), holds));
            }
            after[foot(0)] = Hold::Landed;
            after[foot(1)] = Hold::Landed;
            let moves = (0..2).map(|side| Move { limb: foot(side), from: Hold::Rail, to: Hold::Landed, window: (0.0, 0.0) }).collect();
            return Some((Kind::Land, moves, after));
        }
        if feet == [Hold::Landed; 2] {
            after = [Hold::Free, Hold::Free, Hold::Floor, Hold::Floor];
            let moves = vec![
                Move { limb: foot(0), from: Hold::Landed, to: Hold::Floor, window: (0.0, 0.45) },
                Move { limb: foot(1), from: Hold::Landed, to: Hold::Floor, window: (0.35, 0.8) },
                Move { limb: hand(1), from: holds[hand(1)], to: Hold::Free, window: (0.1, 0.6) },
                Move { limb: hand(0), from: holds[hand(0)], to: Hold::Free, window: (0.3, 0.85) },
            ];
            return Some((Kind::StepBack, moves, after));
        }
        match ask? {
            // Off the floor, hands on the rungs: onto the rails, the hands
            // first, then the feet.
            Climb::Slide if !free && on_rungs => {
                after = [Hold::Rail; 4];
                let moves = vec![
                    Move { limb: hand(1), from: holds[hand(1)], to: Hold::Rail, window: (0.0, 0.35) },
                    Move { limb: hand(0), from: holds[hand(0)], to: Hold::Rail, window: (0.15, 0.5) },
                    Move { limb: foot(0), from: feet[0], to: Hold::Rail, window: (0.45, 0.75) },
                    Move { limb: foot(1), from: feet[1], to: Hold::Rail, window: (0.65, 0.95) },
                ];
                Some((Kind::Grip, moves, after))
            }
            // A foot on the floor or the landing still: down as climbing,
            // onto the rungs or off them.
            Climb::Slide if !free || on_landing => self.plan(holds, last, Some(Climb::Down)),
            // From the floor going up, or the landing going down, the hands
            // reach for the rungs (or above a landing the rails) their feet's
            // first steps need, as high as reach allows standing. On the
            // landing, the feet first shuffle back to its edge, between the
            // rails.
            Climb::Up | Climb::Down if free && ((on_floor && ask == Some(Climb::Up)) || (on_landing && ask == Some(Climb::Down))) => {
                let up = on_floor;
                let hips = if up { self.rest } else { self.top_spot + self.body.hips - Vec3::Y * self.drop };
                // On the landing, its back to the ladder, upright: leant in
                // toward the ladder's rungs, the trunk tipped away from its
                // rails, and no grip on them was in reach.
                let trunk = Quat::from_axis_angle(self.local.left(), if up { LEAN + self.local.lean } else { 0.0 });
                // The right foot goes first (`last_foot` starts left), then
                // the left.
                let from = if up { -1 } else { top + 1 };
                let right = self.pattern.next(from, from, up).clamp(-1, top + 1);
                let feet = [self.pattern.next(from, right, up).clamp(-1, top + 1), right];
                let first = [0, 1].map(|side| {
                    let off = |rung: i32| self.reach_to(side, hips, trunk, self.hand_at(side, Hold::Rung(rung), hips).expect("a rung is held"));
                    let mut rung = (feet[side] + self.pattern.hand).min(highest);
                    while rung > 0 && off(rung) > ARM_REACH * self.body.arms[side] {
                        rung -= 1;
                    }
                    rung
                });
                // From the landing, the rails beside it, sliding down them
                // as it steps down until each follows its foot to a rung.
                after[hand(0)] = if up { Hold::Rung(first[0]) } else { Hold::Rail };
                after[hand(1)] = if up { Hold::Rung(first[1]) } else { Hold::Rail };
                if up {
                    return Some((
                        Kind::Reach,
                        vec![
                            Move { limb: hand(1), from: Hold::Free, to: after[hand(1)], window: (0.0, 0.55) },
                            Move { limb: hand(0), from: Hold::Free, to: after[hand(0)], window: (0.35, 0.9) },
                        ],
                        after,
                    ));
                }
                after[foot(0)] = Hold::Top(top + 1);
                after[foot(1)] = Hold::Top(top + 1);
                Some((
                    Kind::Reach,
                    vec![
                        Move { limb: foot(1), from: holds[foot(1)], to: after[foot(1)], window: (0.0, 0.35) },
                        Move { limb: foot(0), from: holds[foot(0)], to: after[foot(0)], window: (0.25, 0.6) },
                        Move { limb: hand(1), from: Hold::Free, to: after[hand(1)], window: (0.45, 0.8) },
                        Move { limb: hand(0), from: Hold::Free, to: after[hand(0)], window: (0.6, 0.95) },
                    ],
                    after,
                ))
            }
            // On the landing at the top, the hands let go.
            Climb::Up if !free && on_top => {
                after[hand(0)] = Hold::Free;
                after[hand(1)] = Hold::Free;
                Some((
                    Kind::Release,
                    vec![
                        Move { limb: hand(1), from: holds[hand(1)], to: Hold::Free, window: (0.0, 0.6) },
                        Move { limb: hand(0), from: holds[hand(0)], to: Hold::Free, window: (0.25, 0.85) },
                    ],
                    after,
                ))
            }
            Climb::Up if !free => {
                let (a, b) = (feet[0].index(), feet[1].index());
                // The lower foot; level, the one that did not move last.
                let side = if a == b { 1 - last } else if a < b { 0 } else { 1 };
                let mut target = self.pattern.next(feet[side].index(), feet[1 - side].index(), true);
                // Onto a landing past the top, else no higher than the hands
                // lead it.
                if self.local.landing.is_some() {
                    target = target.min(top + 1);
                }
                // Above a landing the hands stop at the rails' tops and the
                // feet go on past them, onto it.
                if target > top + i32::from(self.local.landing.is_some()) || (self.local.landing.is_none() && target + self.pattern.hand > highest) {
                    return None;
                }
                after[foot(side)] = self.foot_hold(target);
                let mut moves = vec![Move { limb: foot(side), from: feet[side], to: after[foot(side)], window: UP.foot }];
                // Stepping onto a landing, the hands slide on the rails with
                // the body: held where they were, they were left below and
                // behind it, 17-36 cm out of reach.
                if matches!(after[foot(side)], Hold::Top(_)) {
                    for (limb, window) in [(hand(1), (0.0, 0.3)), (hand(0), (0.15, 0.45))] {
                        if holds[limb] != Hold::Rail {
                            after[limb] = Hold::Rail;
                            moves.push(Move { limb, from: holds[limb], to: Hold::Rail, window });
                        }
                    }
                    return Some((Kind::Up, moves, after));
                }
                // The other foot's hand goes up for its step next, if there
                // is one: at the top, gone up for a step never taken, it was
                // a rung too high to hold coming down.
                let other = 1 - side;
                let next = self.pattern.next(feet[other].index(), target, true);
                let grip = next + self.pattern.hand;
                if grip <= highest && grip > holds[hand(other)].index() {
                    after[hand(other)] = Hold::Rung(grip);
                    moves.push(Move { limb: hand(other), from: holds[hand(other)], to: after[hand(other)], window: UP.hand });
                }
                Some((Kind::Up, moves, after))
            }
            Climb::Down if !free && on_floor => {
                after[hand(0)] = Hold::Free;
                after[hand(1)] = Hold::Free;
                Some((
                    Kind::Release,
                    vec![
                        Move { limb: hand(1), from: holds[hand(1)], to: Hold::Free, window: (0.0, 0.6) },
                        Move { limb: hand(0), from: holds[hand(0)], to: Hold::Free, window: (0.25, 0.85) },
                    ],
                    after,
                ))
            }
            Climb::Down if !free => {
                let (a, b) = (feet[0].index(), feet[1].index());
                // The upper foot; level, the one that did not move last.
                let side = if a == b { 1 - last } else if a > b { 0 } else { 1 };
                let target = self.pattern.next(feet[side].index(), feet[1 - side].index(), false).max(-1);
                after[foot(side)] = self.foot_hold(target);
                let mut moves = Vec::with_capacity(2);
                // The hand of the foot that went down last follows it first,
                // no lower than it reaches from where the body stands then:
                // on the floor, followed all the way, it held a rung 0.4 m
                // below its shoulder, out of reach.
                let hips = self.hips_on(Kind::Down, holds);
                let mut grip = (feet[last].index() + self.pattern.hand).min(highest);
                while grip < holds[hand(last)].index() && !self.within_reach(last, grip, hips) {
                    grip += 1;
                }
                // Down, not up: a hand sliding on its rail beside the hips,
                // stepping down off a landing, went up for the rail's top,
                // over its shoulder behind it.
                let lower = self.hand_at(last, holds[hand(last)], hips).is_none_or(|now| self.grip_at(last, grip).y < now.y);
                if grip < holds[hand(last)].index() && lower {
                    after[hand(last)] = Hold::Rung(grip);
                    moves.push(Move { limb: hand(last), from: holds[hand(last)], to: after[hand(last)], window: DOWN.hand });
                }
                moves.push(Move { limb: foot(side), from: feet[side], to: after[foot(side)], window: DOWN.foot });
                Some((Kind::Down, moves, after))
            }
            _ => None,
        }
    }

    /// What a foot at rung index `i` stands on: the floor below the lowest
    /// rung, a landing above the highest.
    fn foot_hold(&self, i: i32) -> Hold {
        let top = self.local.top();
        if i < 0 {
            Hold::Floor
        } else if i > top && self.local.landing.is_some() {
            Hold::Top(top + 1)
        } else {
            Hold::Rung(i.min(top))
        }
    }

    /// Whether hand `side` reaches rung `i` with the hips at `hips`, the
    /// trunk at its lean on the ladder or leant in as far as it does to
    /// reach ([`Self::reaching`]).
    fn within_reach(&self, side: usize, i: i32, hips: Vec3) -> bool {
        [LEAN + self.local.lean, MOST_LEAN].into_iter().any(|lean| self.reach_to(side, hips, Quat::from_axis_angle(self.local.left(), lean), self.grip_at(side, i)) <= ARM_REACH * self.body.arms[side])
    }

    /// Arm `side`'s shoulder joint, the hips at `hips` and the trunk turned
    /// `trunk` from standing, lifted toward a wrist at `grip`
    /// ([`shoulder_lift`]).
    fn lifted(&self, side: usize, hips: Vec3, trunk: Quat, grip: Vec3) -> Vec3 {
        let (root, shoulder) = (hips + trunk * self.body.clavicles[side], hips + trunk * self.body.shoulders[side]);
        root + shoulder_lift(root, shoulder, grip, LIFT_FROM * self.body.arms[side]) * (shoulder - root)
    }

    /// How far a wrist at `grip` is from arm `side`'s shoulder lifted toward
    /// it ([`Self::lifted`]).
    fn reach_to(&self, side: usize, hips: Vec3, trunk: Quat, grip: Vec3) -> f32 {
        (grip - self.lifted(side, hips, trunk, grip)).length()
    }

    /// Where the hips stand once on `holds`.
    fn hips_on(&self, kind: Kind, holds: [Hold; 4]) -> Vec3 {
        match kind {
            // From the landing, shuffled back to its edge.
            Kind::Reach if matches!(holds[foot(0)], Hold::Top(_)) => self.top_spot + self.body.hips - Vec3::Y * self.drop,
            Kind::Reach => self.rest,
            // Out to where it slides, its knees clear of the rungs: kept as
            // near as the hands climbing wanted, a knee slid down 0.9 cm
            // from a rung's middle.
            Kind::Grip => self.hips_square(self.rest.y),
            Kind::Release if matches!(holds[foot(0)], Hold::Top(_)) => self.top_spot + self.body.hips - Vec3::Y * self.drop,
            Kind::Release | Kind::StepBack => self.spot + self.body.hips - Vec3::Y * self.drop,
            Kind::Slide => self.touchdown(),
            Kind::Land => self.touchdown() - Vec3::Y * (ABSORB * self.body.legs[0]),
            Kind::Up | Kind::Down => self.hips_at([holds[foot(0)], holds[foot(1)]]),
        }
    }

    /// How long a step takes whose hips go `from` to `to`.
    fn duration(kind: Kind, from: Vec3, to: Vec3) -> f32 {
        match kind {
            // From the landing, the shuffle back to its edge too.
            Kind::Reach => REACH_SECONDS + (to - from).length() / SHUFFLE_SPEED,
            Kind::Release => RELEASE_SECONDS,
            Kind::Up => ((to.y - from.y).abs() / SPEED_UP).max(SHORTEST_STEP),
            Kind::Down => ((to.y - from.y).abs() / SPEED_DOWN).max(SHORTEST_STEP),
            Kind::Grip => GRIP_SECONDS,
            Kind::Slide => Slide::down((to - from).length()).duration(),
            // Braked from the landing speed over the knees' give, at rest
            // when they have given it: a cubic over three times the give
            // at that speed.
            Kind::Land => 3.0 * (to - from).length() / SLIDE_LANDING,
            Kind::StepBack => STEP_BACK_SECONDS,
        }
    }

    /// The step starting now going `ask`, the hips leaving at `velocity`.
    fn start(&self, ask: Option<Climb>, velocity: Vec3) -> Option<Step> {
        let (kind, moves, after) = self.plan(self.holds, self.last_foot, ask)?;
        let from = self.rest;
        let to = self.hips_on(kind, after);
        let duration = Self::duration(kind, from, to);
        let mean = (to - from) / duration;
        let beat = if kind == Kind::Down { DOWN } else { UP };
        let rate = beat.rate();
        // Going on at the same rate after, if the next step is like it.
        let last = moves.iter().find(|m| m.limb >= LEFT_FOOT).map_or(self.last_foot, |m| m.limb - LEFT_FOOT);
        let next = self.plan(after, last, ask).filter(|(next, _, _)| *next == kind && matches!(kind, Kind::Up | Kind::Down)).map(|(next, _, later)| {
            let further = self.hips_on(next, later);
            (further - to, Self::duration(next, to, further))
        });
        let goes_on = next.is_some_and(|(delta, time)| (delta - (to - from)).length() < 1.0e-3 && (time - duration).abs() < 1.0e-3);
        let ends = if goes_on && mean.length() > 0.0 { mean * rate } else { Vec3::ZERO };
        let steady = goes_on && (velocity - mean * rate).length() < 1.0e-3 && mean.length() > 0.0;
        // A slide reaches the floor still moving, and the landing takes that
        // speed on.
        let slide = (kind == Kind::Slide).then(|| Slide::down((to - from).length()));
        let ends = slide.map_or(ends, |slide| (to - from).normalize_or_zero() * slide.land);
        let lean_on = LEAN + self.local.lean;
        let (lean, turn, sink) = match kind {
            // From the landing, upright: it leans in once on the rungs.
            Kind::Reach if matches!(after[foot(0)], Hold::Top(_)) => ((self.lean, 0.0), (self.turn, 0.0), (self.sink, 0.0)),
            Kind::Reach => ((self.lean, lean_on), (self.turn, 0.0), (self.sink, 0.0)),
            Kind::Release | Kind::StepBack => ((self.lean, 0.0), (self.turn, 0.0), (self.sink, self.drop)),
            _ => ((self.lean, lean_on), (self.turn, 0.0), (self.sink, self.sink)),
        };
        let mut step = Step {
            kind,
            duration,
            moves,
            hips: Path { from, to, start: 0.0, duration, starts: velocity, ends, steady: steady.then_some(beat), slide },
            bow: 0.0,
            lean,
            turn,
            sink,
        };
        if matches!(kind, Kind::Up | Kind::Down) {
            step.bow = self.bow_for(&step);
        }
        Some(step)
    }

    /// How far `step`'s hips bow out from the ladder, the least of
    /// [`BOWS`] that keeps each knee joint [`KNEE_CLEAR`] from every rung's
    /// axis and each rail's middle line through it (posed at
    /// [`BOW_SAMPLES`] moments); none does, the one that keeps them
    /// farthest.
    ///
    /// The hips close enough for the hands to reach above the shoulders, a
    /// knee came through a rung going down and level with one going up; a
    /// knee turned out to clear the rungs swung into a rail.
    fn bow_for(&self, step: &Step) -> f32 {
        let key = BowKey {
            kind: step.kind,
            holds: self.holds,
            from: (step.hips.from * 1000.0).round().as_ivec3(),
            to: (step.hips.to * 1000.0).round().as_ivec3(),
        };
        if let Some(&(_, bow)) = self.bows.lock().ok().and_then(|bows| bows.iter().find(|(planned, _)| *planned == key).copied()).as_ref() {
            return bow;
        }
        let bow = self.searched_bow(step);
        if let Ok(mut bows) = self.bows.lock() {
            if bows.len() >= 8 {
                bows.remove(0);
            }
            bows.push((key, bow));
        }
        bow
    }

    /// [`Self::bow_for`], searched.
    fn searched_bow(&self, step: &Step) -> f32 {
        let mut trial = self.clone();
        let mut best = (f32::MIN, 0.0);
        for bow in BOWS {
            trial.step = Some(Step { bow, ..step.clone() });
            let clear = (0..=BOW_SAMPLES)
                .map(|k| {
                    trial.elapsed = step.duration * k as f32 / BOW_SAMPLES as f32;
                    trial.knee_clearance()
                })
                .fold(f32::MAX, f32::min);
            if clear >= KNEE_CLEAR {
                return bow;
            }
            if clear > best.0 {
                best = (clear, bow);
            }
        }
        best.1
    }

    /// How near either knee joint is to a rung's axis or a rail's middle line
    /// now, metres.
    fn knee_clearance(&self) -> f32 {
        let now = self.now();
        // The legs alone: the hands' grip frames and the arms' solves have
        // no say in where the knees go.
        let at = forward_kinematics_on(&self.posed(&self.rig, false), &self.rig);
        let ladder = &self.local;
        LEGS.iter()
            .map(|&(_, knee, _, _)| {
                let knee = now.root + Quat::from_rotation_y(now.turn) * at[knee];
                let rungs = (0..=ladder.top())
                    .map(|i| {
                        let off = knee - ladder.rung(i);
                        let across = off.dot(ladder.left()).clamp(-0.5 * ladder.width, 0.5 * ladder.width);
                        (off - ladder.left() * across).length()
                    })
                    .fold(f32::MAX, f32::min);
                let rails = [-1.0, 1.0]
                    .map(|rail| {
                        let off = knee - (ladder.base + ladder.left() * rail * (0.5 * ladder.width + RAIL_MIDDLE));
                        (off - ladder.up() * off.dot(ladder.up())).length()
                    })
                    .into_iter()
                    .fold(f32::MAX, f32::min);
                rungs.min(rails)
            })
            .fold(f32::MAX, f32::min)
    }

    /// Advances it by `dt`, asked `ask` (`None` holds on where it is, once
    /// its step is done).
    pub fn advance(&mut self, ask: Option<Climb>, dt: f32) {
        self.ask = ask;
        if self.done {
            return;
        }
        let mut dt = dt;
        loop {
            if self.step.is_none() {
                self.elapsed = 0.0;
                self.step = self.start(ask, Vec3::ZERO);
                if self.step.is_none() {
                    return;
                }
            }
            let step = self.step.as_ref().expect("a step");
            // Asked to stop or turn back mid-step, the hips come to rest at
            // the step's end, from where they are and how fast. A slide
            // goes on to the floor.
            if step.hips.ends != Vec3::ZERO && matches!(step.kind, Kind::Up | Kind::Down) {
                let after = step.moves.iter().fold(self.holds, |mut holds, m| {
                    holds[m.limb] = m.to;
                    holds
                });
                let last = step.moves.iter().find(|m| m.limb >= LEFT_FOOT).map_or(self.last_foot, |m| m.limb - LEFT_FOOT);
                let next = self.plan(after, last, ask).map(|(kind, _, _)| kind);
                if next != Some(step.kind) {
                    let t = self.elapsed;
                    let (at, velocity) = (step.hips.at(t), step.hips.velocity(t));
                    let to = step.hips.to;
                    // As long as stopping at `BRAKING` takes, if longer than
                    // the step has left: over what was left, the hips braked
                    // at 5.8 m/s².
                    let braking = |time: f32| {
                        let way = to - at;
                        ((way * 6.0 - velocity * 4.0 * time) / (time * time)).length().max(((velocity * 2.0 * time - way * 6.0) / (time * time)).length())
                    };
                    let mut rest = (step.duration - t).max(1.0e-3);
                    while braking(rest) > BRAKING && rest < 3.0 {
                        rest += 0.01;
                    }
                    let step = self.step.as_mut().expect("a step");
                    step.hips = Path { from: at, to, start: t, duration: rest, starts: velocity, ends: Vec3::ZERO, steady: None, slide: None };
                }
            }
            let step = self.step.as_ref().expect("a step");
            // Done once its limbs have moved and its hips arrived.
            let left = step.duration.max(step.hips.start + step.hips.duration) - self.elapsed;
            if dt < left {
                self.elapsed += dt;
                return;
            }
            // The step is done: its holds taken, and the next begun with
            // what is left of the frame, going on at the rate it ended.
            dt -= left;
            let step = self.step.take().expect("a step");
            let velocity = step.hips.ends;
            for m in &step.moves {
                self.holds[m.limb] = m.to;
                if m.limb >= LEFT_FOOT {
                    self.last_foot = m.limb - LEFT_FOOT;
                }
            }
            self.rest = step.hips.to;
            self.lean = step.lean.1;
            self.turn = step.turn.1;
            self.sink = step.sink.1;
            // Off the floor, a foot comes back down where getting off puts it:
            // the standing feet at the spot, square to the ladder.
            for side in 0..2 {
                if self.holds[foot(side)] != Hold::Floor {
                    self.floor[side] = (self.spot + self.body.balls[side], 0.0);
                }
            }
            if matches!(step.kind, Kind::Release | Kind::StepBack) {
                self.done = true;
                return;
            }
            self.elapsed = 0.0;
            self.step = self.start(ask, velocity);
            if self.step.is_none() {
                return;
            }
        }
    }

    /// Whether it has stepped off and let go: standing again.
    pub fn is_done(&self) -> bool {
        self.done
    }

    /// Whether it is getting on, its hands reaching for the ladder from the
    /// floor or a landing, its feet still there.
    pub fn is_getting_on(&self) -> bool {
        self.step.as_ref().is_some_and(|step| step.kind == Kind::Reach)
    }

    /// Whether it is mid-step.
    pub fn is_moving(&self) -> bool {
        self.step.is_some()
    }

    /// How much its hands hold the ladder, 0 (both hanging) to 1: taking
    /// hold and letting go, eased.
    pub fn holding(&self) -> f32 {
        let t = self.now().t;
        (0..2)
            .map(|side| match self.step.as_ref().and_then(|step| step.moves.iter().find(|m| m.limb == hand(side))) {
                Some(Move { from: Hold::Free, window, .. }) => super::gait::smoothstep(window_share(*window, t)),
                Some(Move { to: Hold::Free, window, .. }) => 1.0 - super::gait::smoothstep(window_share(*window, t)),
                _ if self.holds[hand(side)] == Hold::Free => 0.0,
                _ => 1.0,
            })
            .fold(0.0, f32::max)
    }

    /// How closed each hand's fingers are asked to be (left, right), 0
    /// relaxed to 1 round its rung or rail (`hand::RelaxedHands::grip`):
    /// open as soon as it lets go, closed as it arrives; the fingers ease
    /// between (`hand::close_hands`).
    pub fn grips(&self) -> [f32; 2] {
        let t = self.now().t;
        [0, 1].map(|side| match self.step.as_ref().and_then(|step| step.moves.iter().find(|m| m.limb == hand(side))) {
            Some(m) => {
                let s = window_share(m.window, t);
                let held = |hold: Hold| hold != Hold::Free;
                let gripping = (held(m.from) && s <= 0.0) || (held(m.to) && s >= CLOSE_AT);
                if gripping { 1.0 } else { 0.0 }
            }
            None if self.holds[hand(side)] == Hold::Free => 0.0,
            None => 1.0,
        })
    }

    /// What each limb holds as the current step began (left hand, right
    /// hand, left foot, right foot).
    pub fn holds(&self) -> [Hold; 4] {
        self.holds
    }

    /// The ladder it is on, in the world.
    pub fn ladder(&self) -> &Ladder {
        &self.world
    }

    /// The facing it climbs at, radians about `+Y`: square to the ladder,
    /// turning from where it got on.
    pub fn facing(&self) -> f32 {
        self.yaw + self.now().turn
    }

    /// Where the walker's root is in the world now: under the hips as
    /// standing.
    pub fn root(&self) -> Vec3 {
        self.origin + Quat::from_rotation_y(self.yaw) * self.now().root
    }

    /// Where it looks, in the world: the ladder in front of its eyes, above
    /// them going up and below going down ([`LOOK_UP`], [`LOOK_DOWN`]).
    ///
    /// At each next hold instead, close in front of the face, the head
    /// bowed into the raised arm; down at the foot reaching for its rung,
    /// chin to chest.
    pub fn look(&self) -> Vec3 {
        let now = self.now();
        let eyes = now.hips + Quat::from_rotation_y(now.turn) * self.body.eyes;
        let above = match self.step.as_ref().map(|step| step.kind) {
            Some(Kind::Up | Kind::Reach) => LOOK_UP,
            Some(Kind::Down | Kind::Release | Kind::Grip | Kind::Slide | Kind::Land) => -LOOK_DOWN,
            Some(Kind::StepBack) | None => 0.0,
        };
        self.origin + Quat::from_rotation_y(self.yaw) * self.local.middle_at(eyes.y + above)
    }

    /// The state of the body at this moment.
    fn now(&self) -> Now {
        let (hips, lean, turn, sink, t) = match &self.step {
            Some(step) => {
                let s = (self.elapsed / step.duration).clamp(0.0, 1.0);
                let eased = super::gait::smoothstep(s);
                // Out and back, still at both ends.
                let bowed = self.local.out() * (step.bow * (std::f32::consts::PI * s).sin().powi(2));
                (step.hips.at(self.elapsed) + bowed, lerp(step.lean, eased), lerp(step.turn, eased), lerp(step.sink, eased), s)
            }
            None => (self.rest, self.lean, self.turn, self.sink, 0.0),
        };
        // The pose's hips `sink` below the standing pose's, the root under
        // them.
        let root = hips - Quat::from_rotation_y(turn) * self.body.hips + Vec3::Y * sink;
        Now { hips, root, lean, turn, sink, t }
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the spring
    /// trails a steady motion (`jump::lead_of`): the climb as it will be
    /// that much later, going on as last asked. The root is not sprung, so
    /// the rendered body is the climb's now.
    ///
    /// Unled, a held hand and a planted foot crept 2-4 mm a frame as the
    /// root rode up and the sprung limbs trailed the body's speed changes.
    pub fn pose_led(&self, rig: &RigGeometry, springs: &super::rig::BoneSet<super::math::SpringParams>) -> LocalPose {
        let mut pose = self.pose(rig);
        let mut posed: Vec<(f32, LocalPose)> = Vec::with_capacity(3);
        for bone in Bone::ALL {
            let lead = super::jump::lead_of(&springs[bone]);
            if lead <= 1.0e-4 {
                continue;
            }
            pose.rotations[bone] = match posed.iter().find(|(at, _)| (at - lead).abs() < 1.0e-4) {
                Some((_, ahead)) => ahead.rotations[bone],
                None => {
                    let mut later = self.clone();
                    later.advance(self.ask, lead);
                    let ahead = later.pose(rig);
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }

    /// The pose now, on `rig` (the one it was measured on), in the walker's
    /// pose frame at [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self, rig: &RigGeometry) -> LocalPose {
        self.posed(rig, true)
    }

    /// [`Self::pose`], or (`arms` false) only the trunk and legs: the arms
    /// as standing, the trunk at the step's lean.
    fn posed(&self, rig: &RigGeometry, arms: bool) -> LocalPose {
        let now = self.now();
        let turn = Quat::from_rotation_y(now.turn);
        let back = turn.inverse();
        let to_pose = |p: Vec3| back * (p - now.root);
        let step = self.step.as_ref();
        let moving = |limb: usize| step.and_then(|step| step.moves.iter().find(|m| m.limb == limb).map(|m| (*m, window_share(m.window, now.t))));
        // Each hand's wrist on a rung or a rail, or between two; or, taking
        // hold or letting go, the hold and how much it holds it.
        // Each hand's grip frame (`grip_frame`) on what it holds, or moves
        // from and to, worked out once: asked again for the wrist, the lean
        // and the hand's turn, a climb cost twice as much.
        let frames: [Vec<(Hold, Option<GripFrame>)>; 2] = [0, 1].map(|side| {
            let holds = match moving(hand(side)) {
                _ if !arms => vec![],
                Some((m, _)) => vec![m.from, m.to],
                None => vec![self.holds[hand(side)]],
            };
            holds.into_iter().map(|hold| (hold, self.grip_frame(side, hold, now.hips))).collect()
        });
        let frame_of = |side: usize, hold: Hold| frames[side].iter().find(|(held, _)| *held == hold).and_then(|(_, frame)| *frame);
        let on = |side: usize, hold: Hold| frame_of(side, hold).map(|(bar, line, palm)| bar - self.hand_turn(side, line, palm) * self.body.grips[side].bar);
        let grips = [0, 1].map(|side| match moving(hand(side)) {
            Some((m, s)) => match (on(side, m.from), on(side, m.to)) {
                (Some(from), Some(to)) => Some((self.travel(from, to, HAND_CLEAR, s), 1.0)),
                (Some(from), None) => Some((from, 1.0 - super::gait::smoothstep(s))),
                (None, Some(to)) => Some((to, super::gait::smoothstep(s))),
                (None, None) => None,
            },
            None => on(side, self.holds[hand(side)]).map(|at| (at, 1.0)),
        });
        let mut pose = super::jump::upper(&self.body.stood, rig, self.reaching(&now, grips), (0.0, 0.0));
        pose.root_translation.y -= now.sink;
        let hips = self.body.hips - Vec3::Y * now.sink;

        // Each foot's ball on its hold, or on its way; its ankle placed, and
        // the foot turned to its attitude.
        for (side, &(_, _, ankle_bone, _)) in LEGS.iter().enumerate() {
            let (ball, yaw) = match moving(foot(side)) {
                Some((m, s)) => {
                    let (from, from_yaw) = self.foot_at(side, m.from, now.hips);
                    let (to, to_yaw) = self.foot_at(side, m.to, now.hips);
                    let turned = from_yaw + (to_yaw - from_yaw) * super::gait::smoothstep(s);
                    // Across the floor, a step: lifted, not out of the ladder.
                    if m.from == Hold::Landed || matches!(m.from, Hold::Landing(_)) {
                        (from.lerp(to, super::gait::smoothstep(s)) + Vec3::Y * (STEP_LIFT * (std::f32::consts::PI * s).sin()), turned)
                    } else if matches!(m.to, Hold::Top(_)) || matches!(m.from, Hold::Top(_)) {
                        (self.over_top(from, to, s, matches!(m.to, Hold::Top(_))), turned)
                    } else {
                        (self.travel(from, to, self.body.toe + TOE_CLEAR, s), turned)
                    }
                }
                None => self.foot_at(side, self.holds[foot(side)], now.hips),
            };
            let attitude = Quat::from_rotation_y(yaw);
            let ankle = to_pose(ball + attitude * self.body.ankles[side]);
            place_ankle(&mut pose, rig, ankle_bone, ankle - hips);
            let now_attitude = accumulate_world_rotations(&pose, rig)[ankle_bone];
            let wanted = back * attitude * self.body.attitudes[side];
            pose.rotations[ankle_bone] = delta_after_world_turn(&pose, rig, ankle_bone, wanted * now_attitude.inverse());
        }
        if !arms {
            return pose;
        }

        // Each hand on its rung, or on its way, or hanging.
        let at = forward_kinematics_on(&pose, rig);
        let targets = [0, 1].map(|side| {
            let free = at[ARMS[side].wrist];
            let wrist_on = |hold: Hold| on(side, hold).map(to_pose);
            match (grips[side], moving(hand(side))) {
                (Some((on, held)), _) if held >= 1.0 => (to_pose(on), 1.0),
                (_, Some((m, s))) => {
                    let eased = super::gait::smoothstep(s);
                    match (wrist_on(m.from), wrist_on(m.to)) {
                        (None, Some(to)) => (free.lerp(to, eased), eased),
                        (Some(from), None) => (from.lerp(free, eased), 1.0 - eased),
                        _ => (free, 0.0),
                    }
                }
                (_, None) => (free, 0.0),
            }
        });
        // Each shoulder lifted toward a hand reaching far (`shoulder_lift`),
        // by how much the hand holds.
        for side in 0..2 {
            let (target, grip) = targets[side];
            if grip > 0.0 {
                let lift = shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], target, LIFT_FROM * self.body.arms[side]);
                pose.rotations[CLAVICLES[side]] = delta_after_world_turn(&pose, rig, CLAVICLES[side], Quat::IDENTITY.slerp(lift, grip));
            }
        }
        let at = forward_kinematics_on(&pose, rig);
        for side in 0..2 {
            let chain = ARMS[side];
            let (target, grip) = targets[side];
            if grip <= 0.0 {
                continue;
            }
            let (shoulder, elbow) = (at[chain.shoulder], at[chain.elbow]);
            let line = (at[chain.wrist] - shoulder).normalize_or_zero();
            let side_now = ((elbow - shoulder) - line * (elbow - shoulder).dot(line)).normalize_or_zero();
            // Toward the pole of what it holds, turning to what it reaches for
            // over its move. Turned at the step's start, a hand still on its
            // rung, about to go to a rail, had its elbow out to the side and
            // its wrist bent 76° round the rung.
            let pole_of = |hold: Hold| self.elbow_pole(side, self.on_rail(hold));
            let held = back
                * match moving(hand(side)) {
                    Some((m, s)) if m.to != Hold::Free && m.from != Hold::Free => pole_of(m.from).lerp(pole_of(m.to), super::gait::smoothstep(s)).normalize_or(pole_of(m.to)),
                    Some((m, _)) if m.to != Hold::Free => pole_of(m.to),
                    Some((m, _)) => pole_of(m.from),
                    None => pole_of(self.holds[hand(side)]),
                };
            let pole = if side_now == Vec3::ZERO { held } else { side_now.lerp(held, grip) };
            let (elbow_at, wrist_at) = solve_arm_toward_from(&mut pose, &at, chain, target, pole, rig);
            // The hand turned to hold its rung or rail (`grip_frame`): on its
            // way between two, turning from one's hold to the other's.
            let turn_on = |hold: Hold| frame_of(side, hold).map(|(_, line, palm)| self.hand_turn(side, back * line, back * palm));
            let wanted = match moving(hand(side)) {
                Some((m, s)) => match (turn_on(m.from), turn_on(m.to)) {
                    (Some(from), Some(to)) => Some(from.slerp(to, super::gait::smoothstep(s))),
                    (from, to) => from.or(to),
                },
                None => turn_on(self.holds[hand(side)]),
            };
            if let Some(wanted) = wanted {
                self.turn_hand(&mut pose, rig, side, wanted, grip, (wrist_at - elbow_at).normalize_or_zero());
            }
        }
        pose
    }

    /// Turns hand `side` toward the world turn `wanted` from its rest (the
    /// pose's frame), by `grip` (0 not at all): its roll about the forearm's
    /// line (`forearm`) by the forearm, up to [`MOST_FOREARM_TWIST`], the
    /// rest at the wrist. The wrist does not move.
    ///
    /// A hand turned at the wrist alone to face its palm onto a rung twisted
    /// it by the forearm's whole roll.
    fn turn_hand(&self, pose: &mut LocalPose, rig: &RigGeometry, side: usize, wanted: Quat, grip: f32, forearm: Vec3) {
        super::armik::turn_hand(pose, rig, ARMS[side], self.body.hand_binds[side], wanted, grip, forearm);
    }

    /// How far the trunk leans `now`: the step's lean, and further in while
    /// a hand holds a rung (`grips`, the climb's frame) beyond
    /// [`ARM_REACH`] of its shoulder, as far as brings it within it, to at
    /// most [`MOST_LEAN`]. On rungs 0.36 m apart a hand went two rungs up
    /// from beside the chest to overhead, and with the trunk held at its
    /// lean the arm fell 11 cm short.
    ///
    /// Each hand asks its own lean, by how much it holds its rung: one letting
    /// go asks less and less.
    fn reaching(&self, now: &Now, grips: [Option<(Vec3, f32)>; 2]) -> f32 {
        let turn = Quat::from_rotation_y(now.turn);
        let left = self.local.left();
        let asks = |side: usize, grip: Vec3| {
            let short = |lean: f32| self.reach_to(side, now.hips, turn * Quat::from_axis_angle(left, lean), grip) - ARM_REACH * self.body.arms[side];
            let (mut low, mut high) = (now.lean, now.lean.max(MOST_LEAN));
            if short(low) <= 0.0 {
                return low;
            }
            if short(high) > 0.0 {
                return high;
            }
            for _ in 0..20 {
                let middle = 0.5 * (low + high);
                if short(middle) > 0.0 { low = middle } else { high = middle }
            }
            high
        };
        (0..2)
            .filter_map(|side| grips[side].map(|(grip, held)| now.lean + (asks(side, grip) - now.lean) * held))
            .fold(now.lean, f32::max)
    }

    /// A foot's way over the ladder's top at `s` of its move, from `from` to
    /// `to`: `onto` the landing, up above the top rung first and then
    /// across onto it; off it, back across first and then down; lifted
    /// [`STEP_LIFT`] clear of the top rung on the way. Laid straight there,
    /// the toes went through the top rung.
    ///
    /// Out of the ladder first by the toes' length, as from rung to rung,
    /// before rising past the top rung (back in after, coming down): risen
    /// straight up off the rung below it, the sole went through it.
    fn over_top(&self, from: Vec3, to: Vec3, s: f32, onto: bool) -> Vec3 {
        let smooth = |t: f32| super::gait::smoothstep(t.clamp(0.0, 1.0));
        let (rise, across) = if onto { (smooth((s - 0.15) / 0.5), smooth((s - 0.45) / 0.55)) } else { (smooth((s - 0.4) / 0.45), smooth(s / 0.55)) };
        let away = if onto { smooth(s / 0.2) - smooth((s - 0.4) / 0.25) } else { smooth((s - 0.35) / 0.25) - smooth((s - 0.8) / 0.2) };
        let way = to - from;
        let flat = Vec3::new(way.x, 0.0, way.z);
        let clear = self.body.toe + TOE_CLEAR;
        from + flat * across + self.local.normal() * (clear * away) + Vec3::Y * (way.y * rise + STEP_LIFT * (std::f32::consts::PI * s).sin())
    }

    /// A limb's way from `from` to `to` at `s` of its move: out from the
    /// ladder by `clear` (less what it is out already), across, and back in.
    fn travel(&self, from: Vec3, to: Vec3, clear: f32, s: f32) -> Vec3 {
        let normal = self.local.normal();
        let out_of = |p: Vec3| (p - self.local.middle_at(p.y)).dot(normal);
        let clear = (clear - out_of(from).min(out_of(to)).max(0.0)).max(0.0);
        let across = super::gait::smoothstep(((s - 0.2) / 0.6).clamp(0.0, 1.0));
        let away = super::gait::smoothstep((s / 0.3).clamp(0.0, 1.0)) - super::gait::smoothstep(((s - 0.7) / 0.3).clamp(0.0, 1.0));
        from.lerp(to, across) + normal * (clear * away)
    }
}

/// How a hand holds a bar ([`Climbing::grip_frame`]): a point on its axis,
/// the way the fingers point, the way the palm faces.
type GripFrame = (Vec3, Vec3, Vec3);

/// What a step's bow depends on ([`Climbing::bow_for`]): its kind, the
/// holds it starts from, and its hips' way, millimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
struct BowKey {
    kind: Kind,
    holds: [Hold; 4],
    from: bevy::math::IVec3,
    to: bevy::math::IVec3,
}

/// The state of the body at a moment of a climb, in the climb's frame.
#[derive(Debug, Clone, Copy)]
struct Now {
    hips: Vec3,
    root: Vec3,
    lean: f32,
    turn: f32,
    sink: f32,
    /// How far through the step, 0-1.
    t: f32,
}

fn lerp((a, b): (f32, f32), t: f32) -> f32 {
    a + (b - a) * t
}

/// How far through `window` (shares of a step) `t` is, 0-1.
fn window_share((start, end): (f32, f32), t: f32) -> f32 {
    ((t - start) / (end - start).max(1.0e-6)).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    const DT: f32 = 1.0 / 60.0;

    /// A walker standing on its spot before `ladder`, square to it.
    fn on_spot(ladder: &Ladder, stood: &LocalPose, rig: &RigGeometry) -> Climbing {
        let square = super::super::approach::heading_of(-ladder.out()) - super::super::approach::heading_of(rig.forward());
        let spot = Climbing::spot(ladder, square, stood, rig);
        let mut climbing = Climbing::new(ladder, spot, square, square, 0.0, stood, rig);
        climbing.set_grips(crate::character::anim::hand::puppet_grips(), rig);
        climbing
    }

    /// Each frame of `climbing` asked `ask` for `seconds`: its pose in the
    /// world's frame, joint positions.
    fn run(climbing: &mut Climbing, ask: Option<Climb>, seconds: f32, rig: &RigGeometry, mut each: impl FnMut(&Climbing, &Joints, &Extra)) {
        let frames = (seconds / DT).round() as usize;
        let soles = LEGS.map(|(_, _, ankle, _)| crate::character::anim::foot::Sole::of(rig, ankle));
        for _ in 0..frames {
            climbing.advance(ask, DT);
            let pose = climbing.pose(rig);
            let at = forward_kinematics_on(&pose, rig);
            let turn = Quat::from_rotation_y(climbing.facing());
            let root = climbing.root();
            let world = crate::character::anim::rig::BoneSet::from_fn(|bone| root + turn * at[bone]);
            let rotations = accumulate_world_rotations(&pose, rig);
            let extra = Extra {
                // Heel, ball and tip of each sole.
                soles: soles.map(|sole| sole.points(&pose, rig).map(|p| world[Bone::Hips] + turn * p)),
                hands: [0, 1].map(|side| turn * rotations[ARMS[side].wrist] * climbing.body.hand_binds[side].inverse()),
            };
            each(climbing, &world, &extra);
        }
    }

    type Joints = crate::character::anim::rig::BoneSet<Vec3>;

    /// What else a frame shows: each sole's heel, ball and tip, and each
    /// hand's world turn from its rest.
    struct Extra {
        soles: [[Vec3; 3]; 2],
        hands: [Quat; 2],
    }

    /// The ladders tried: standard; rungs close, wide apart and very wide
    /// apart; narrow and wide; leaning.
    fn ladders() -> Vec<(&'static str, Ladder)> {
        let standard = Ladder::standard(Vec3::new(0.3, 0.0, -1.0), Vec3::new(0.2, 0.0, 1.0));
        vec![
            ("standard", standard),
            ("close", Ladder { spacing: 0.22, rungs: 16, first: 0.22, ..standard }),
            ("apart", Ladder { spacing: 0.36, rungs: 10, first: 0.36, ..standard }),
            ("far apart", Ladder { spacing: 0.45, rungs: 8, first: 0.4, ..standard }),
            ("narrow", Ladder { width: 0.32, ..standard }),
            ("wide", Ladder { width: 0.6, ..standard }),
            ("leaning", Ladder { lean: 0.25, ..standard }),
        ]
    }

    /// Ladders leading onto a landing: standard, close and very wide apart
    /// rungs, and leaning.
    fn to_landings() -> Vec<(&'static str, Ladder)> {
        let landing = Some(Vec2::new(2.0, 2.0));
        ladders()
            .into_iter()
            .filter(|(name, _)| matches!(*name, "standard" | "close" | "far apart" | "leaning"))
            .map(|(name, ladder)| (name, Ladder { landing, ..ladder }))
            .collect()
    }

    /// What a whole climb, up and down again, measured.
    #[derive(Debug, Default)]
    struct Measured {
        /// The most a held limb strays from its hold, metres (hands, feet).
        held: [f32; 2],
        /// The least a knee is ahead of its hip-to-ankle line, metres.
        knee_ahead: f32,
        /// The least a knee joint keeps from a rung's middle, and a rail's,
        /// metres.
        knee_clear: f32,
        knee_rail: f32,
        /// The least a moving foot's ball or tip keeps from a rung's middle.
        foot_clear: f32,
        /// The most a held hand's grip strays from its rung's or rail's
        /// axis, metres, and its palm from facing onto it, radians.
        bar_off: f32,
        palm_off: f32,
        /// The most a held hand bends off its forearm's line, radians.
        wrist_bend: f32,
        /// The hips' greatest acceleration, m/s², and speed, m/s.
        hips_acceleration: f32,
        fastest: f32,
        /// A knee joint's greatest acceleration, m/s².
        knee_acceleration: f32,
        /// How fast the hips went up, and down, through the middle of each
        /// way, m/s.
        speeds: [f32; 2],
        /// Whether it got back down and let go, and how far the root ends
        /// from where it got on, metres.
        done: bool,
        returned: f32,
        /// Stepped off onto a landing: how far from its spot there it
        /// stood, metres; and how far its pose was from standing (`1 - |dot|`
        /// of the worst joint).
        topped: Option<f32>,
        top_pose: f32,
        /// Each limb's (hand, foot) largest move up, rungs.
        moves: [i32; 2],
        /// Each frame all four limbs hold rungs: the trunk's lean in toward
        /// the ladder from its rails, radians (the hips to the neck), and
        /// how far its held hands' grips are above their shoulders along
        /// the rails, of the arm.
        trunk_in: Vec<f32>,
        hands_above: Vec<f32>,
        /// The farthest a moving hand's wrist goes through the rungs' plane,
        /// metres.
        hand_through: f32,
        /// The least an upper arm or forearm keeps from its rail's middle
        /// line, metres, its hand on or between rungs.
        arm_rail: f32,
    }

    /// A climb up `ladder` to the top and down again, measured.
    fn climbed(ladder: &Ladder) -> Measured {
        climbed_and(ladder, Climb::Down)
    }

    /// A climb up `ladder` to the top and back to the floor by `down`
    /// (climbing down or sliding), measured.
    fn climbed_and(ladder: &Ladder, down: Climb) -> Measured {
        let (stood, rig) = real_stood();
        let mut climbing = on_spot(ladder, &stood, &rig);
        let start = climbing.root();
        let mut measured = Measured { knee_ahead: f32::MAX, knee_clear: f32::MAX, knee_rail: f32::MAX, foot_clear: f32::MAX, arm_rail: f32::MAX, ..Default::default() };
        let mut hips: Vec<Vec3> = Vec::new();
        let mut knees: Vec<[Vec3; 2]> = Vec::new();
        let world = *ladder;
        let to_world = |c: &Climbing, p: Vec3| c.origin + Quat::from_rotation_y(c.yaw) * p;
        let mut each = |c: &Climbing, at: &Joints, extra: &Extra| {
            let soles = &extra.soles;
            hips.push(at[Bone::Hips]);
            knees.push([at[Bone::LeftLeg], at[Bone::RightLeg]]);
            let step = c.step.as_ref();
            let t = c.now().t;
            // Stepping between rungs, no hand yet on the top one (where the
            // feet climb on under still hands).
            let mid_ladder = step.is_some_and(|s| matches!(s.kind, Kind::Up | Kind::Down))
                && c.holds.iter().enumerate().all(|(limb, hold)| matches!(hold, Hold::Rung(i) if limb >= LEFT_FOOT || *i < world.top()));
            if mid_ladder {
                let trunk = at[Bone::Neck] - at[Bone::Hips];
                let lean_in = (-trunk.dot(world.normal())).atan2(trunk.dot(world.up()));
                measured.trunk_in.push(lean_in);
            }
            for limb in 0..4 {
                let side = limb % 2;
                let m = step.and_then(|s| s.moves.iter().find(|m| m.limb == limb).copied());
                let hold = match m {
                    None => Some(c.holds[limb]),
                    Some(m) if t <= m.window.0 => Some(m.from),
                    Some(m) if t >= m.window.1 => Some(m.to),
                    Some(m) => {
                        if let (Hold::Rung(a), Hold::Rung(b)) = (m.from, m.to) {
                            measured.moves[limb / 2] = measured.moves[limb / 2].max(b - a);
                        }
                        None
                    }
                };
                // Each arm segment's nearest approach to its rail's middle
                // line, sampled along it, the hand on or between rungs, or
                // reaching for them.
                let on_rails = |hold: Hold| hold == Hold::Rail || matches!(hold, Hold::Rung(i) if i > world.top());
                let near_rails = match m {
                    Some(m) => on_rails(m.from) || on_rails(m.to),
                    None => on_rails(c.holds[limb]),
                };
                if limb < LEFT_FOOT && !near_rails && c.holds[limb] != Hold::Free {
                    let rail = world.base + world.left() * SIGN[side] * (0.5 * world.width + RAIL_MIDDLE);
                    let chain = ARMS[side];
                    for (a, b) in [(at[chain.shoulder], at[chain.elbow]), (at[chain.elbow], at[chain.wrist])] {
                        for k in 0..=10 {
                            let off = a.lerp(b, k as f32 / 10.0) - rail;
                            measured.arm_rail = measured.arm_rail.min((off - world.up() * off.dot(world.up())).length());
                        }
                    }
                }
                if limb >= LEFT_FOOT {
                    let (socket, knee, ankle, toe) = LEGS[side];
                    let line = (at[ankle] - at[socket]).normalize();
                    let off = at[knee] - at[socket];
                    let ahead = (off - line * off.dot(line)).dot(Quat::from_rotation_y(c.facing()) * rig.forward());
                    measured.knee_ahead = measured.knee_ahead.min(ahead);
                    for i in 0..ladder.rungs as i32 {
                        let off = at[knee] - world.rung(i);
                        let across = off.dot(world.left()).clamp(-0.5 * world.width, 0.5 * world.width);
                        measured.knee_clear = measured.knee_clear.min((off - world.left() * across).length());
                    }
                    // From each rail's middle line, square to it.
                    for rail in [-1.0, 1.0] {
                        let off = at[knee] - (world.base + world.left() * rail * (0.5 * world.width + RAIL_MIDDLE));
                        measured.knee_rail = measured.knee_rail.min((off - world.up() * off.dot(world.up())).length());
                    }
                    match hold {
                        Some(hold) => {
                            let ball = to_world(c, c.foot_at(side, hold, c.now().hips).0);
                            measured.held[1] = measured.held[1].max((at[toe] - ball).length());
                        }
                        // Moving, every point of its sole clear of every rung.
                        None => {
                            for point in soles[side] {
                                for i in 0..ladder.rungs as i32 {
                                    let off = point - world.rung(i);
                                    let across = off.dot(world.left()).clamp(-0.5 * world.width, 0.5 * world.width);
                                    measured.foot_clear = measured.foot_clear.min((off - world.left() * across).length());
                                }
                            }
                        }
                    }
                } else if hold.is_none() {
                    let wrist = at[ARMS[side].wrist];
                    if wrist.y < world.rung(world.top()).y {
                        measured.hand_through = measured.hand_through.max(-(wrist - world.middle_at(wrist.y)).dot(world.normal()));
                    }
                } else if let Some(hold) = hold
                    && let Some(wrist) = c.hand_at(side, hold, c.now().hips)
                {
                    measured.held[0] = measured.held[0].max((at[ARMS[side].wrist] - to_world(c, wrist)).length());
                    if let (true, Hold::Rung(i)) = (mid_ladder, hold) {
                        measured.hands_above.push((world.rung(i) - at[ARMS[side].shoulder]).dot(world.up()) / c.body.arms[side]);
                    }
                    // Where its fingers close, on the rung's or rail's axis;
                    // its palm onto it.
                    let grip = &c.body.grips[side];
                    let bar = at[ARMS[side].wrist] + extra.hands[side] * grip.bar;
                    let (axis, along) = match hold {
                        Hold::Rung(i) if i <= world.top() => (world.rung(i), world.left()),
                        _ => (world.base + world.left() * SIGN[side] * (0.5 * world.width + RAIL_MIDDLE), world.up()),
                    };
                    let off = bar - axis;
                    measured.bar_off = measured.bar_off.max((off - along * off.dot(along)).length());
                    let (_, _, palm) = c.grip_frame(side, hold, c.now().hips).expect("held");
                    let forearm = (at[ARMS[side].wrist] - at[ARMS[side].elbow]).normalize();
                    let bend = (extra.hands[side] * grip.along).dot(forearm).clamp(-1.0, 1.0).acos();
                    measured.wrist_bend = measured.wrist_bend.max(bend);
                    let facing = (extra.hands[side] * grip.palm).dot(Quat::from_rotation_y(c.yaw) * palm);
                    measured.palm_off = measured.palm_off.max(facing.clamp(-1.0, 1.0).acos());
                }
            }
        };
        let (mut seconds, mut up) = (0.0, 0);
        while seconds < 60.0 {
            run(&mut climbing, Some(Climb::Up), DT, &rig, &mut each);
            seconds += DT;
            up += 1;
            if climbing.is_done() || (!climbing.is_moving() && climbing.holds[hand(0)] != Hold::Free) {
                break;
            }
        }
        // Off onto a landing: on again from there, as the walker gets on,
        // standing where the climb left it.
        if climbing.is_done() {
            let expected = climbing.origin + Quat::from_rotation_y(climbing.yaw) * climbing.top_spot;
            measured.topped = Some((climbing.root() - expected).length());
            let pose = climbing.pose(&rig);
            measured.top_pose = Bone::ALL.iter().map(|&bone| 1.0 - pose.rotation(bone).dot(stood.rotation(bone)).abs()).fold(0.0, f32::max);
            let (root, facing) = (climbing.root(), climbing.facing());
            climbing = Climbing::new(ladder, root, facing, climbing.yaw, 0.0, &stood, &rig);
            climbing.set_grips(crate::character::anim::hand::puppet_grips(), &rig);
        }
        while seconds < 120.0 && !climbing.is_done() {
            run(&mut climbing, Some(down), DT, &rig, &mut each);
            seconds += DT;
        }
        measured.done = climbing.is_done();
        measured.returned = (climbing.root() - start).length();
        measured.hips_acceleration = hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        measured.fastest = hips.windows(2).map(|w| (w[1] - w[0]).length() / DT).fold(0.0, f32::max);
        measured.knee_acceleration = knees.windows(3).flat_map(|w| [0, 1].map(|i| ((w[2][i] - 2.0 * w[1][i] + w[0][i]) / (DT * DT)).length())).fold(0.0, f32::max);
        // The rate over the middle half of each way, m/s.
        let rate = |samples: &[Vec3]| {
            let (a, b) = (samples.len() / 4, 3 * samples.len() / 4);
            (samples[b].y - samples[a].y) / ((b - a) as f32 * DT)
        };
        measured.speeds = [rate(&hips[..up]), -rate(&hips[up..])];
        measured
    }

    #[test]
    fn every_hold_is_held_and_every_limb_clears_the_ladder() {
        for (name, ladder) in ladders() {
            let m = climbed(&ladder);
            assert!(m.done && m.returned < 1.0e-3, "{name}: back on the floor {}, {:.4} m from where it got on", m.done, m.returned);
            // A hand on its lowest rung, standing, is the arm's whole reach.
            assert!(m.held[0] < 5.0e-3 && m.held[1] < 1.0e-3, "{name}: a hand strays {:.4} m from its rung, a foot {:.4}", m.held[0], m.held[1]);
            assert!(m.knee_ahead > 0.01, "{name}: a knee {:.4} m ahead of its leg's line", m.knee_ahead);
            // A knee may come into the ladder between rungs, the hips bowing
            // out for it to pass one (`bow_for`): without the bow, 0.3-4 cm
            // from a rung's middle; turned out instead, into the rails.
            assert!(m.knee_clear > 0.045 && m.knee_rail > 0.055, "{name}: a knee {:.4} m from a rung's middle, {:.4} m from a rail's", m.knee_clear, m.knee_rail);
            assert!(m.knee_acceleration < 260.0, "{name}: a knee accelerated {:.0} m/s²", m.knee_acceleration);
            assert!(m.foot_clear > Ladder::RUNG_RADIUS - 5.0e-4, "{name}: a moving sole {:.4} m from a rung's middle", m.foot_clear);
            // The arms clear of the rails, the elbows down and back: pulled
            // out to the side, a forearm came across a rail to its rung.
            assert!(m.arm_rail > 0.075, "{name}: an arm {:.4} m from a rail's middle", m.arm_rail);
            assert!(m.hand_through < 1.0e-3, "{name}: a moving wrist {:.4} m through the rungs' plane", m.hand_through);
            assert!(m.hips_acceleration < 9.0, "{name}: the hips accelerate {:.2} m/s²", m.hips_acceleration);
        }
    }

    /// Climbing, the trunk is upright along the rails, not folded in over
    /// the hands, and the hands hold rungs above the shoulders (the hips in
    /// for them, the shoulders lifted toward them). Leant in 0.16 rad, the
    /// standard ladder's hands' median rung was 0.1 of the arm below the
    /// shoulder.
    #[test]
    fn it_climbs_upright_its_hands_above_its_shoulders() {
        let median = |v: &[f32]| {
            let mut s = v.to_vec();
            s.sort_by(f32::total_cmp);
            s[s.len() / 2]
        };
        for (name, ladder) in ladders() {
            let m = climbed(&ladder);
            let most = m.trunk_in.iter().copied().fold(f32::MIN, f32::max);
            // Upright: at most a 3° lean in, reaching a rung 0.45 m up.
            assert!(most < 0.06, "{name}: the trunk leant {most:.2} rad in");
            // About 0.4 of the arm above on rungs 0.22-0.3 m apart. Wider,
            // the next rung up is out of reach (0.45 m, past the 0.36 m most
            // standards allow), and a leaning ladder's rungs come no nearer
            // the shoulders than the hips allow: about at the shoulders.
            let least = match name {
                "leaning" | "far apart" => -0.25,
                "apart" => 0.05,
                _ => 0.3,
            };
            assert!(median(&m.hands_above) > least, "{name}: the hands' median rung {:.2} of the arm above the shoulders", median(&m.hands_above));
        }
    }

    /// Posed led ahead of the springs (`pose_led`), as the walker poses it,
    /// no wrist passes through the ladder. Each bone is led by its own
    /// spring's lag: the arms, solved for the trunk ahead, set on a trunk
    /// leaning in fast to reach, put both fists 15-33 cm through it.
    #[test]
    fn led_no_hand_passes_through_the_ladder() {
        let (stood, rig) = real_stood();
        let springs = crate::character::anim::dho::default_springs();
        for (name, ladder) in ladders() {
            let mut c = on_spot(&ladder, &stood, &rig);
            let mut through = 0.0f32;
            for frame in 0..2400 {
                c.advance(Some(if frame < 1200 { Climb::Up } else { Climb::Down }), DT);
                let at = forward_kinematics_on(&c.pose_led(&rig, &springs), &rig);
                let turn = Quat::from_rotation_y(c.facing());
                for side in 0..2 {
                    let wrist = c.root() + turn * at[ARMS[side].wrist];
                    if wrist.y < ladder.rung(ladder.top()).y {
                        through = through.max(-(wrist - ladder.middle_at(wrist.y)).dot(ladder.normal()));
                    }
                }
            }
            assert!(through < 0.01, "{name}: a wrist {through:.3} m through the rungs' plane");
        }
    }

    /// Up a ladder that leads onto a landing, it steps off and stands on
    /// it, as it stands; on again from there (climbing or sliding down), it
    /// gets back to where it first got on. On the way every hold is held,
    /// every limb clears the ladder, and nothing jolts.
    #[test]
    fn a_landing_is_stepped_off_at_the_top_and_got_on_from() {
        for (name, ladder) in to_landings() {
            for down in [Climb::Down, Climb::Slide] {
                let m = climbed_and(&ladder, down);
                assert!(m.topped.is_some_and(|off| off < 1.0e-3) && m.top_pose < 1.0e-5, "{name}: on the landing {:?} m off its spot, {:.6} off standing", m.topped, m.top_pose);
                assert!(m.done && m.returned < 1.0e-3, "{name} {down:?}: back on the floor {}, {:.4} m from where it got on", m.done, m.returned);
                assert!(m.held[0] < 5.0e-3 && m.held[1] < 1.0e-3, "{name} {down:?}: a hand strays {:.4} m, a foot {:.4}", m.held[0], m.held[1]);
                assert!(m.foot_clear > Ladder::RUNG_RADIUS - 5.0e-4, "{name} {down:?}: a moving sole {:.4} m from a rung's middle", m.foot_clear);
                assert!(m.knee_clear > 0.045 && m.knee_rail > 0.05, "{name} {down:?}: a knee {:.4} m from a rung's middle, {:.4} m from a rail's", m.knee_clear, m.knee_rail);
                assert!(m.hips_acceleration < 9.0 && m.knee_acceleration < 260.0, "{name} {down:?}: the hips accelerated {:.2} m/s², a knee {:.0}", m.hips_acceleration, m.knee_acceleration);
                assert!(m.bar_off < 1.0e-3 && m.palm_off < 0.01 && m.wrist_bend < 65f32.to_radians(), "{name} {down:?}: a grip {:.4} m off its bar, a palm {:.3} rad off it, a wrist bent {:.0}°", m.bar_off, m.palm_off, m.wrist_bend.to_degrees());
            }
        }
    }

    /// Slid down from the top, hands and feet keep to the rails, no faster
    /// than [`SLIDE_SPEED`], and it lands and steps back to where it got on.
    #[test]
    fn a_slide_keeps_to_the_rails_and_lands_where_it_got_on() {
        for (name, ladder) in ladders() {
            let m = climbed_and(&ladder, Climb::Slide);
            assert!(m.done && m.returned < 1.0e-3, "{name}: back on the floor {}, {:.4} m from where it got on", m.done, m.returned);
            assert!(m.held[0] < 5.0e-3 && m.held[1] < 1.0e-3, "{name}: a hand strays {:.4} m from its hold, a foot {:.4}", m.held[0], m.held[1]);
            assert!(m.knee_ahead > 0.01 && m.knee_clear > 0.045, "{name}: a knee {:.4} m ahead of its leg's line, {:.4} m from a rung's middle", m.knee_ahead, m.knee_clear);
            assert!(m.fastest < SLIDE_SPEED + 0.05, "{name}: slid at {:.2} m/s", m.fastest);
            assert!(m.speeds[1] > 2.0 * SPEED_DOWN, "{name}: down at {:.2} m/s, slower than twice climbing down", m.speeds[1]);
            // Speeding up at the slide's 4 m/s² and taking its landing at 1 m/s
            // over the knees' 10 cm.
            assert!(m.hips_acceleration < 9.0 && m.knee_acceleration < 260.0, "{name}: the hips accelerated {:.2} m/s², a knee {:.0}", m.hips_acceleration, m.knee_acceleration);
        }
    }

    /// A held hand closes round its rung or rail: the bar its fingers wrap
    /// (`hand::gripped`, `puppet_base`'s own fingers) on the rung's or
    /// rail's axis, its palm onto it, the wrist bent no further than a
    /// wrist goes.
    #[test]
    fn a_hand_closes_round_its_rung_or_rail() {
        for (name, ladder) in ladders() {
            for down in [Climb::Down, Climb::Slide] {
                let m = climbed_and(&ladder, down);
                assert!(m.bar_off < 1.0e-3, "{name} {down:?}: a grip {:.4} m off its bar's axis", m.bar_off);
                assert!(m.palm_off < 0.01, "{name} {down:?}: a palm {:.3} rad off facing its bar", m.palm_off);
                // About 30° climbing, 40-50° on the rails; 61° as a hand
                // lands with the trunk leant in to reach it. Held one fixed
                // way, 97-111°.
                assert!(m.wrist_bend < 65f32.to_radians(), "{name} {down:?}: a wrist bent {:.0}°", m.wrist_bend.to_degrees());
            }
        }
    }

    /// A hand reaching far up lifts its shoulder toward it
    /// (`shoulder_lift`, about the clavicle's root): the reach comes within
    /// [`LIFT_FROM`] of the arm, and the arm then puts the wrist exactly on
    /// it. One already within leaves the shoulder be.
    #[test]
    fn a_far_reach_lifts_the_shoulder_and_the_arm_lands_on_it() {
        let (stood, rig) = real_stood();
        let at = forward_kinematics_on(&stood, &rig);
        let (root, shoulder) = (at[CLAVICLES[0]], at[ARMS[0].shoulder]);
        let arm = (at[ARMS[0].elbow] - shoulder).length() + (at[ARMS[0].wrist] - at[ARMS[0].elbow]).length();
        for (up, forward) in [(0.4, 0.25), (0.4, 0.45), (0.2, 0.4)] {
            let far = shoulder + Vec3::new(0.05, up, forward).normalize() * 0.95 * arm;
            let lift = shoulder_lift(root, shoulder, far, LIFT_FROM * arm);
            assert!(lift.to_axis_angle().1 > 0.05, "{up} {forward}: lifted only {:.3} rad", lift.to_axis_angle().1);
            let mut pose = stood;
            pose.rotations[CLAVICLES[0]] = delta_after_world_turn(&pose, &rig, CLAVICLES[0], lift);
            let lifted = forward_kinematics_on(&pose, &rig);
            let reach = (far - lifted[ARMS[0].shoulder]).length() / arm;
            assert!(reach < LIFT_FROM + 1.0e-3, "{up} {forward}: still {reach:.3} of the arm away");
            solve_arm_toward_from(&mut pose, &lifted, ARMS[0], far, Vec3::new(0.0, -1.0, 0.5), &rig);
            let off = (forward_kinematics_on(&pose, &rig)[ARMS[0].wrist] - far).length();
            assert!(off < 1.0e-4, "{up} {forward}: the wrist {off:.5} m off its reach");
        }
        let near = shoulder + Vec3::new(0.0, 0.2, 0.2).normalize() * 0.6 * arm;
        assert!(shoulder_lift(root, shoulder, near, LIFT_FROM * arm).to_axis_angle().1 < 1.0e-5, "a near reach lifted the shoulder");
    }

    #[test]
    fn it_climbs_at_a_persons_pace() {
        let m = climbed(&ladders()[0].1);
        assert!((m.speeds[0] - SPEED_UP).abs() < 0.03, "up at {:.3} m/s", m.speeds[0]);
        assert!((m.speeds[1] - SPEED_DOWN).abs() < 0.03, "down at {:.3} m/s", m.speeds[1]);
    }

    /// The way a ladder is climbed does not hang on float noise: the standing
    /// pose's turns nudged by a few 1e-6 rad (as normalizing a turn helper's
    /// output did), every ladder keeps its pattern and its hips' distance.
    /// Compared bare against `HAND_STRETCH`, a hand rung the shoulder's lift
    /// brings to exactly that flipped the 0.36 m ladder to passing feet.
    #[test]
    fn a_ladders_pattern_does_not_hang_on_float_noise() {
        let (stood, rig) = real_stood();
        for nudge in [1.0e-6f32, -1.0e-6, 3.0e-6] {
            let mut nudged = stood;
            for (i, bone) in [Bone::Spine, Bone::Spine1, Bone::LeftUpLeg, Bone::RightUpLeg, Bone::LeftArm, Bone::RightArm].into_iter().enumerate() {
                let axis = [Vec3::X, Vec3::Y, Vec3::Z][i % 3];
                nudged.rotations[bone] = Quat::from_axis_angle(axis, nudge) * nudged.rotations[bone];
            }
            for (name, ladder) in ladders() {
                let (was, now) = (on_spot(&ladder, &stood, &rig), on_spot(&ladder, &nudged, &rig));
                eprintln!("{name}: {:?}, hips {:.4} m out", was.pattern, was.hips_out);
                assert_eq!(was.pattern, now.pattern, "{name}, nudged {nudge}: the pattern changed");
                assert!((was.hips_out - now.hips_out).abs() < 1.0e-3, "{name}, nudged {nudge}: the hips {} m out, were {}", now.hips_out, was.hips_out);
            }
        }
    }

    /// Each foot passes the other where a foot and a hand reach two rungs;
    /// past that, both feet go to each rung. Closer rungs are gripped more
    /// rungs above the feet.
    #[test]
    fn the_way_it_climbs_follows_the_rungs_spacing() {
        let (stood, rig) = real_stood();
        let found: Vec<(&str, Pattern, [i32; 2])> = ladders().into_iter().map(|(name, ladder)| (name, on_spot(&ladder, &stood, &rig).pattern, climbed(&ladder).moves)).collect();
        for (name, pattern, moves) in &found {
            let passes = matches!(*name, "standard" | "close" | "narrow" | "wide" | "leaning");
            let (gap, moved) = if passes { (1, 2) } else { (0, 1) };
            assert!(pattern.gap == gap && *moves == [moved; 2], "{name}: {pattern:?}, hands and feet moved {moves:?} rungs");
        }
        let hand = |wanted: &str| found.iter().find(|(name, _, _)| *name == wanted).map(|(_, pattern, _)| pattern.hand).unwrap_or_default();
        assert!(hand("close") > hand("standard"), "gripped {} rungs up 0.22 m apart, {} 0.3 m apart", hand("close"), hand("standard"));
    }

    /// Asked nothing mid-step, it finishes the step, its hips coming to rest,
    /// and holds still; asked down again, it climbs down and off.
    #[test]
    fn asked_nothing_it_holds_on_where_it_is() {
        let (stood, rig) = real_stood();
        let ladder = ladders()[0].1;
        let mut climbing = on_spot(&ladder, &stood, &rig);
        let mut hips = Vec::new();
        run(&mut climbing, Some(Climb::Up), 3.3, &rig, |_, at, _| hips.push(at[Bone::Hips]));
        assert!(climbing.is_moving(), "mid-step");
        run(&mut climbing, None, 2.0, &rig, |_, at, _| hips.push(at[Bone::Hips]));
        assert!(!climbing.is_moving(), "still stepping 2 s later");
        let held = climbing.pose(&rig);
        run(&mut climbing, None, 1.0, &rig, |_, at, _| hips.push(at[Bone::Hips]));
        let drift = Bone::ALL.iter().map(|&bone| 1.0 - held.rotation(bone).dot(climbing.pose(&rig).rotation(bone)).abs()).fold(0.0, f32::max);
        assert!(drift < 1.0e-6, "holding on, a joint turned {drift}");
        let jolt = hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        assert!(jolt < 5.0, "stopping, the hips accelerated {jolt:.2} m/s²");
        run(&mut climbing, Some(Climb::Down), 20.0, &rig, |_, _, _| {});
        assert!(climbing.is_done(), "not back on the floor");
    }

    /// Off again, the pose is the standing one, where it got on: the walker
    /// stands on from it without a jump.
    #[test]
    fn stepped_off_it_stands_as_it_stood() {
        let (stood, rig) = real_stood();
        let ladder = ladders()[0].1;
        let mut climbing = on_spot(&ladder, &stood, &rig);
        let start = climbing.root();
        run(&mut climbing, Some(Climb::Up), 4.0, &rig, |_, _, _| {});
        run(&mut climbing, Some(Climb::Down), 20.0, &rig, |_, _, _| {});
        assert!(climbing.is_done(), "not back on the floor");
        let pose = climbing.pose(&rig);
        let (at, was) = (forward_kinematics_on(&pose, &rig), forward_kinematics_on(&stood, &rig));
        let off = Bone::ALL.iter().map(|&bone| (at[bone] - was[bone]).length()).fold(0.0, f32::max);
        assert!(off < 1.0e-3 && (climbing.root() - start).length() < 1.0e-3, "a joint {off:.4} m off standing, the root {:.4} m off", (climbing.root() - start).length());
    }
}
