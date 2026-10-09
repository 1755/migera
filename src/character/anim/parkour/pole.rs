//! A vertical pole: step 9 of the parkour design, second part. Stood at,
//! it is jumped onto, the hands one above the other and the feet clamping
//! it from either side; then climbed up or down (an inchworm: the arms pull
//! the body up as the legs fold, then the clamped legs stand it up as the
//! hands go up one over the other), slid down, gone round, and let go of.
//!
//! No pole data: the climb's pace is a rope climb's, about 3 s a metre
//! (an unverified summary, `parkour-movement-data`); its shape follows the
//! hang's (the shoulder lift, the hand turn onto a bar).

use bevy::math::{Quat, Vec3};

use super::Falling;
use crate::character::anim::armik::{frame_turn, shoulder_lift, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::hand::{HandGrip, FINGER_HALF_THICKNESS, GRIP_RADIUS};
use crate::character::anim::jump::{lead_of, GRAVITY};
use crate::character::anim::math::SpringParams;
use crate::character::anim::rig::{accumulate_bind_rotations, accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// A pole's radius, metres: a 5 cm pipe.
pub const POLE_RADIUS: f32 = 0.025;
/// The hips this far from the pole's axis, metres, holding it.
pub const HIPS_OFF: f32 = 0.3;
/// The trunk leant toward the pole, radians.
const LEAN: f32 = 0.15;
/// Each wrist's height above the hips, the legs stood up: the lower hand's
/// and the upper's, metres.
const HAND_LOW: f32 = 0.5;
const HAND_HIGH: f32 = 0.7;
/// The ankles below the hips, the legs stood up (the knees a little bent:
/// at 0.85 they reached 0.9 m) and folded up (the knees to the chest),
/// metres.
const LEGS_STOOD: f32 = 0.78;
const LEGS_FOLDED: f32 = 0.43;
/// The arms' pull, metres the hips rise on the hands.
const PULL: f32 = 0.25;
/// The hips' rise in a climbing cycle, metres: the pull and the legs'
/// stand. Each hand goes up as far, held no lower than about 0.15 m over
/// the hips and put no higher than its arm reaches, about 0.78: a 0.65 m
/// range. At 0.78 m, the hand held longest hung straight down at the hips
/// and its elbow swung 3.5 cm off its path in a frame.
pub const STROKE: f32 = PULL + LEGS_STOOD - LEGS_FOLDED;
/// A cycle's length, seconds (0.6 m in 1.8 s, a rope climb's pace), and the
/// share of it the pull takes.
pub const CYCLE: f32 = 1.8;
const PULL_SHARE: f32 = 0.4;
/// In the stand, the lower hand goes up over the upper between these
/// shares of the cycle, then the other: the first arriving with the hips
/// 0.84 of their way up (its place in reach), the second leaving before
/// the hips pass it.
const FIRST_HAND: (f32, f32) = (0.55, 0.85);
const SECOND_HAND: (f32, f32) = (0.85, 1.0);
/// A hand going up comes this far off the pole on its way, metres: toward
/// the body, its shoulder (at 0.06 the elbow swung 2.07 cm off its path
/// in a frame).
const HAND_OFF: f32 = 0.03;
/// Each ankle this far to its side of the pole's axis, and this far behind
/// it, clamping it; each foot turned this far sole in, radians.
const CLAMP_SIDE: f32 = 0.07;
const CLAMP_BEHIND: f32 = 0.03;
const SOLE_IN: f32 = 0.7;
/// Getting on: the hips rise this much over this long, metres and seconds;
/// the hands are on by this share of it, the feet off the floor from this
/// share; the standing pose eased out over the first this long.
const GET_ON_RISE: f32 = 0.35;
const GET_ON: f32 = 0.8;
const HANDS_ON: f32 = 0.6;
const FEET_OFF: f32 = 0.25;
const GET_ON_EASE: f32 = 0.2;
/// Getting on, each hand comes this much nearer its shoulder halfway up,
/// metres.
const SWEEP_BEND: f32 = 0.12;
/// The highest hand keeps this far under the pole's top, metres; climbing
/// down, the feet no nearer the floor than this before it lets go.
const TOP_MARGIN: f32 = 0.1;
const BOTTOM_MARGIN: f32 = 0.25;
/// Sliding: gravity braked by this share, to at most this speed, m/s; the
/// legs this far under the hips and the hands this far over them,
/// metres; eased into over this long; let go this high over the floor.
const SLIDE_BRAKE: f32 = 0.7;
const SLIDE_TOP: f32 = 2.5;
const SLIDE_LEGS: f32 = 0.6;
const SLIDE_HANDS: (f32, f32) = (0.45, 0.65);
const SLIDE_IN: f32 = 0.25;
const SLIDE_OFF: f32 = 0.15;
/// Going round: at this rate, rad/s, eased to and from over this long.
const SPIN_RATE: f32 = 1.5;
const SPIN_EASE: f32 = 0.3;
/// Letting go, pushed off it at this speed, m/s.
const LET_GO_AWAY: f32 = 1.0;

/// Each leg's socket, knee, ankle and toe: left, right.
const LEGS: [(Bone, Bone, Bone, Bone); 2] =
    [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase)];
const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
/// Which way each side lies along the character's left.
const SIGN: [f32; 2] = [1.0, -1.0];
/// Where a hand without known fingers holds, metres along it from the wrist.
const GUESSED_KNUCKLES: f32 = 0.08;

/// A vertical pole standing on the floor at `foot`, `height` tall.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pole {
    pub foot: Vec3,
    pub height: f32,
    pub radius: f32,
}

impl Pole {
    /// A pole of [`POLE_RADIUS`] at `foot`, `height` tall.
    pub fn new(foot: Vec3, height: f32) -> Self {
        Self { foot, height, radius: POLE_RADIUS }
    }

    /// Its axis at height `y` (the world).
    pub fn at(&self, y: f32) -> Vec3 {
        self.foot.with_y(y)
    }

    /// Its top's height.
    pub fn top(&self) -> f32 {
        self.foot.y + self.height
    }
}

/// What a walker is asked to do on its pole.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoleAsk {
    /// Walk to it, get on, and climb up while asked (to its top).
    Up,
    /// Climb down while asked; at the bottom, let go onto the floor.
    Down,
    /// Slide down to the floor, braked, and land.
    Slide,
    /// Go round it while asked, holding on.
    Round(Round),
    /// Let go, pushed off it, and fall.
    LetGo,
}

/// Which way round a pole, seen from above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Round {
    /// Anticlockwise: the body going to its own right.
    Left,
    /// Clockwise.
    Right,
}

/// Where the hands and feet hold and the hips are: heights (the world).
#[derive(Debug, Clone, Copy, PartialEq)]
struct Holds {
    hips: f32,
    feet: f32,
    hands: [f32; 2],
    /// The hand lower on the pole.
    lower: usize,
}

impl Holds {
    /// Stood up on clamped legs, the hands at their places over hips at
    /// `hips`, `lower` the lower hand.
    fn stood(hips: f32, lower: usize) -> Self {
        let mut hands = [0.0; 2];
        hands[lower] = hips + HAND_LOW;
        hands[1 - lower] = hips + HAND_HIGH;
        Self { hips, feet: hips - LEGS_STOOD, hands, lower }
    }

    /// `u` (0-1) through a climbing cycle up from these: the pull (the
    /// hands held, the feet folding up), then the stand (the feet held, the
    /// lower hand over the upper, then the other), and how far off the pole
    /// each hand is on its way.
    fn climbing(&self, u: f32) -> (Self, [f32; 2]) {
        let pulled = smoothstep((u / PULL_SHARE).clamp(0.0, 1.0));
        let stood = smoothstep(((u - PULL_SHARE) / (1.0 - PULL_SHARE)).clamp(0.0, 1.0));
        let hips = self.hips + PULL * pulled + (LEGS_STOOD - LEGS_FOLDED) * stood;
        let feet = self.feet + STROKE * pulled;
        let end = Self::stood(self.hips + STROKE, 1 - self.lower);
        let mut hands = self.hands;
        let mut off = [0.0; 2];
        for (side, (from, to)) in [(self.lower, FIRST_HAND), (1 - self.lower, SECOND_HAND)] {
            let s = ((u - from) / (to - from)).clamp(0.0, 1.0);
            hands[side] = self.hands[side] + (end.hands[side] - self.hands[side]) * smoothstep(s);
            // Off and back on with no speed at either end (`sin`, the hand
            // stopped dead on the pole and an elbow jumped 3.5 cm).
            off[side] = HAND_OFF * (std::f32::consts::PI * s).sin().powi(2);
        }
        let lower = if u >= SECOND_HAND.0 { end.lower } else { self.lower };
        (Self { hips, feet, hands, lower }, off)
    }

    /// The cycle up that ends at these (stood up): its start.
    fn before(&self) -> Self {
        Self::stood(self.hips - STROKE, 1 - self.lower)
    }
}

/// Where it is on the pole.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Jumping on from standing, `t` seconds in.
    GettingOn { t: f32 },
    /// Held, stood up on the clamped legs.
    Holding,
    /// A climbing cycle, up or down, `u` (0-1) through it, from `from`
    /// (stood up at its start going up, at its end going down).
    Climbing { up: bool, u: f32, from: Holds },
    /// Sliding down at `speed`, m/s, `t` seconds in, from `from`.
    Sliding { speed: f32, t: f32, from: Holds },
    /// Let go (pushed off, or onto the floor at the bottom, or the slide's
    /// end): handed to a fall ([`Poling::release`]).
    Released { velocity: Vec3 },
}

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    /// The hips joint, the pose's frame; each arm's length.
    hips: Vec3,
    arms: [f32; 2],
    /// Each shoulder joint's, hand's and ankle's standing place (the pose's
    /// frame), each foot's standing world rotation.
    shoulders: [Vec3; 2],
    hands: [Vec3; 2],
    ankles: [Vec3; 2],
    attitudes: [Quat; 2],
    /// Each hand's bind rotation, and how it holds a bar in its rest frame.
    hand_binds: [Quat; 2],
    grips: [HandGrip; 2],
}

impl Body {
    fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let at = forward_kinematics_on(stood, rig);
        let world = accumulate_world_rotations(stood, rig);
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let binds = accumulate_bind_rotations(rig);
        Self {
            stood: *stood,
            hips: at[Bone::Hips],
            arms: ARMS.map(|arm| (at[arm.elbow] - at[arm.shoulder]).length() + (at[arm.wrist] - at[arm.elbow]).length()),
            shoulders: ARMS.map(|arm| at[arm.shoulder]),
            hands: ARMS.map(|arm| at[arm.wrist]),
            ankles: LEGS.map(|(_, _, ankle, _)| at[ankle]),
            attitudes: LEGS.map(|(_, _, ankle, _)| world[ankle]),
            hand_binds: ARMS.map(|arm| binds[arm.wrist]),
            // Until the fingers are known (`Poling::set_grips`): the hand
            // on from the forearm, its rest palm down.
            grips: ARMS.map(|arm| {
                let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
                let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
                HandGrip { bar: along * GUESSED_KNUCKLES + palm * (GRIP_RADIUS + FINGER_HALF_THICKNESS), palm, along }
            }),
        }
    }
}

/// On a pole ([`Pole`]): getting on, holding, climbing, sliding, going
/// round, letting go.
#[derive(Debug, Clone)]
pub struct Poling {
    pole: Pole,
    body: Body,
    rig: RigGeometry,
    /// The rig's own facing, as a heading.
    ahead: f32,
    /// Round the pole: the hips' bearing from its axis (radians about
    /// `+Y`, as `approach::heading_of` reads a direction), and how fast it
    /// goes round, rad/s.
    round: f32,
    spin: f32,
    /// Getting on: where it stood (its root) and its holds once on.
    stood_at: Vec3,
    holds: Holds,
    phase: Phase,
    /// The ask it moves on ([`Self::advance`]).
    ask: Option<PoleAsk>,
    /// The floor's height under it.
    floor: f32,
}

impl Poling {
    /// Where the root stands to get on `pole`, coming from `from`: the hips
    /// [`HIPS_OFF`] from its axis toward `from`, facing it.
    pub fn spot(pole: &Pole, from: Vec3, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
        let away = (from - pole.foot).with_y(0.0).normalize_or(Vec3::Z);
        let turn = Self::turn_facing(-away, rig);
        let hips = forward_kinematics_on(stood, rig)[Bone::Hips];
        (pole.at(pole.foot.y) + away * HIPS_OFF - turn * hips.with_y(0.0)).with_y(pole.foot.y)
    }

    /// The facing turn that points the rig's forward along `toward`.
    fn turn_facing(toward: Vec3, rig: &RigGeometry) -> Quat {
        Quat::from_rotation_y(crate::character::anim::approach::heading_of(toward) - crate::character::anim::approach::heading_of(rig.forward()))
    }

    /// Getting on `pole` from standing with the root at `root` (on `rig`,
    /// standing `stood`): jumping onto it facing it, the hips rising
    /// [`GET_ON_RISE`] as the hands take it and the feet clamp it.
    pub fn get_on(pole: &Pole, root: Vec3, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let body = Body::of(stood, rig);
        let away = (root - pole.foot).with_y(0.0).normalize_or(Vec3::Z);
        let round = crate::character::anim::approach::heading_of(away);
        let hips = root.y + body.hips.y + GET_ON_RISE;
        Self {
            pole: *pole,
            body,
            rig: rig.clone(),
            ahead: crate::character::anim::approach::heading_of(rig.forward()),
            round,
            spin: 0.0,
            stood_at: root,
            holds: Holds::stood(hips, 0),
            phase: Phase::GettingOn { t: 0.0 },
            ask: None,
            floor: root.y,
        }
    }

    /// Each hand placed so its own fingers close round the pole: their
    /// grips, measured on the hand's rest frame (`hand::grip_of`).
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2]) {
        for (side, grip) in grips.into_iter().enumerate() {
            if let Some(grip) = grip {
                self.body.grips[side] = grip;
            }
        }
    }

    /// Moves it on `dt` seconds, asked `ask`: a climb's cycle under way
    /// finishes; a slide, once begun, goes to the floor.
    pub fn advance(&mut self, ask: Option<PoleAsk>, dt: f32) {
        self.ask = ask;
        self.step(dt);
    }

    fn step(&mut self, dt: f32) {
        // Going round: eased toward its rate while asked and held.
        let wanted = match (self.ask, self.phase) {
            (Some(PoleAsk::Round(Round::Left)), Phase::Holding) => SPIN_RATE,
            (Some(PoleAsk::Round(Round::Right)), Phase::Holding) => -SPIN_RATE,
            _ => 0.0,
        };
        let most = SPIN_RATE / SPIN_EASE * dt;
        self.spin += (wanted - self.spin).clamp(-most, most);
        self.round += self.spin * dt;
        self.phase = match self.phase {
            Phase::GettingOn { t } if t + dt >= GET_ON => Phase::Holding,
            Phase::GettingOn { t } => Phase::GettingOn { t: t + dt },
            Phase::Climbing { up, u, from } => {
                let u = u + dt / CYCLE;
                if u < 1.0 {
                    Phase::Climbing { up, u, from }
                } else {
                    self.holds = if up { from.climbing(1.0).0 } else { from };
                    self.next_phase()
                }
            }
            Phase::Sliding { speed, t, from } => {
                let speed = (speed + GRAVITY * (1.0 - SLIDE_BRAKE) * dt).min(SLIDE_TOP);
                let fall = speed * dt;
                let from = Holds { hips: from.hips - fall, feet: from.feet - fall, hands: from.hands.map(|hand| hand - fall), ..from };
                let sliding = Phase::Sliding { speed, t: t + dt, from };
                self.phase = sliding;
                let now = self.holds_now().0;
                if now.feet - self.floor <= SLIDE_OFF {
                    // Let go where it slid to (left at the holds it slid
                    // from, the hips went back up 3 m in a frame).
                    self.holds = now;
                    Phase::Released { velocity: Vec3::NEG_Y * speed }
                } else {
                    sliding
                }
            }
            phase => phase,
        };
        if self.phase == Phase::Holding && self.spin.abs() < 1.0e-3 {
            self.phase = self.next_phase();
        }
    }

    /// What it does next, held stood up: as asked.
    fn next_phase(&self) -> Phase {
        let holds = self.holds;
        match self.ask {
            Some(PoleAsk::Up) if holds.climbing(1.0).0.hands.iter().fold(f32::MIN, |a, &b| a.max(b)) <= self.pole.top() - TOP_MARGIN => Phase::Climbing { up: true, u: 0.0, from: holds },
            Some(PoleAsk::Down) if holds.before().feet - self.floor >= BOTTOM_MARGIN => Phase::Climbing { up: false, u: 0.0, from: holds.before() },
            // At the bottom, onto the floor.
            Some(PoleAsk::Down) => Phase::Released { velocity: Vec3::ZERO },
            Some(PoleAsk::Slide) => Phase::Sliding { speed: 0.0, t: 0.0, from: holds },
            Some(PoleAsk::LetGo) => Phase::Released { velocity: self.away() * LET_GO_AWAY },
            _ => Phase::Holding,
        }
    }

    /// From the pole's axis out to the hips, level.
    fn away(&self) -> Vec3 {
        Quat::from_rotation_y(self.round) * Vec3::NEG_Z
    }

    /// The facing turn now: toward the pole, the bearing out to the hips
    /// turned round (along the bearing itself, it faced away from the pole
    /// and reached behind its back for it, which only a lean back hid).
    fn turn(&self) -> Quat {
        Quat::from_rotation_y(self.facing())
    }

    /// The holds now, and how far off the pole each hand is.
    fn holds_now(&self) -> (Holds, [f32; 2]) {
        match self.phase {
            Phase::Climbing { up, u, from } => from.climbing(if up { u } else { 1.0 - u }),
            Phase::Sliding { t, from, .. } => {
                let s = smoothstep((t / SLIDE_IN).clamp(0.0, 1.0));
                let mut hands = from.hands;
                hands[from.lower] = from.hands[from.lower] + (from.hips + SLIDE_HANDS.0 - from.hands[from.lower]) * s;
                hands[1 - from.lower] = from.hands[1 - from.lower] + (from.hips + SLIDE_HANDS.1 - from.hands[1 - from.lower]) * s;
                let feet = from.feet + (from.hips - SLIDE_LEGS - from.feet) * s;
                (Holds { feet, hands, ..from }, [0.0; 2])
            }
            _ => (self.holds, [0.0; 2]),
        }
    }

    /// The hips in the world now.
    fn hips(&self) -> Vec3 {
        let (holds, _) = self.holds_now();
        let on = self.pole.at(holds.hips) + self.away() * HIPS_OFF;
        match self.phase {
            Phase::GettingOn { t } => {
                let s = smoothstep((t / GET_ON).clamp(0.0, 1.0));
                let from = self.stood_at + self.turn() * self.body.hips;
                from.lerp(on, s)
            }
            _ => on,
        }
    }

    /// The pose now, on `rig` (the one it was measured on), in the
    /// walker's pose frame at [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self, rig: &RigGeometry) -> LocalPose {
        let (holds, off) = self.holds_now();
        let hips = self.hips();
        let turn = self.turn();
        let root = hips - turn * self.body.hips;
        let back = turn.inverse();
        let toward = -self.away();
        let left = turn * rig.left();
        let getting_on = match self.phase {
            Phase::GettingOn { t } => Some(t),
            _ => None,
        };
        let lean = LEAN * getting_on.map_or(1.0, |t| smoothstep((t / GET_ON).clamp(0.0, 1.0)));
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, lean, (0.0, 0.0));
        // The legs: each ankle clamping the pole from its side, the sole
        // turned in; getting on, from where it stood.
        for (side, &(_, _, ankle_bone, _)) in LEGS.iter().enumerate() {
            let clamp = self.pole.at(holds.feet) + left * (SIGN[side] * CLAMP_SIDE) - toward * CLAMP_BEHIND;
            let stood = self.stood_at + turn * self.body.ankles[side];
            let (target, gone) = match getting_on {
                // The feet no farther from the hips than clamped: the hips
                // rising ahead of them, the leg went past straight (0.9 m
                // of 0.87) and its knee flipped 4.7 cm in a frame.
                Some(t) => {
                    let s = smoothstep(((t / GET_ON - FEET_OFF) / (1.0 - FEET_OFF)).clamp(0.0, 1.0));
                    let target = stood.lerp(clamp, s);
                    let most = LEGS_STOOD.hypot(HIPS_OFF);
                    let off = target - hips;
                    (if off.length() > most { hips + off * (most / off.length()) } else { target }, s)
                }
                None => (clamp, 1.0),
            };
            place_ankle(&mut pose, rig, ankle_bone, back * (target - root) - self.body.hips);
            let standing = turn * self.body.attitudes[side];
            let clamping = Quat::from_axis_angle(toward, -SIGN[side] * SOLE_IN) * standing;
            let attitude = standing.slerp(clamping, gone);
            let now = accumulate_world_rotations(&pose, rig)[ankle_bone];
            pose.rotations[ankle_bone] = delta_after_world_turn(&pose, rig, ankle_bone, (back * attitude) * now.inverse());
        }
        // The arms: each hand round the pole at its height, the fingers
        // round it toward the other side, the palm toward its axis.
        let mut wrists = [Vec3::ZERO; 2];
        let mut turns = [Quat::IDENTITY; 2];
        for side in 0..2 {
            let grip = &self.body.grips[side];
            let line = -left * SIGN[side];
            let hand = frame_turn(grip.along, grip.palm, line, toward);
            let on = self.pole.at(holds.hands[side]) - hand * grip.bar - toward * off[side];
            wrists[side] = match getting_on {
                // Raised forward round the shoulder to the pole: straight
                // there, the hand's path passed 5 cm from the shoulder, the
                // arm folded shut and the elbow flipped 17 cm in a frame.
                Some(t) => {
                    let s = smoothstep((t / (GET_ON * HANDS_ON)).clamp(0.0, 1.0));
                    let shoulder = self.body.shoulders[side];
                    let (from, to) = (self.body.hands[side] - shoulder, back * (on - root) - shoulder);
                    let swept = Quat::IDENTITY.slerp(Quat::from_rotation_arc(from.normalize(), to.normalize()), s) * from.normalize();
                    // The elbow bent on the way: swept at the arm's length,
                    // near straight, the elbow's swivel swung 4 cm a frame.
                    let reach = from.length() + (to.length() - from.length()) * s - SWEEP_BEND * (std::f32::consts::PI * s).sin();
                    root + turn * (shoulder + swept * reach)
                }
                None => on,
            };
            turns[side] = hand;
        }
        let weight = getting_on.map_or(1.0, |t| smoothstep((t / (GET_ON * HANDS_ON)).clamp(0.0, 1.0)));
        self.arms_to(&mut pose, rig, root, back, wrists, turns, weight);
        // Getting on, the standing pose eased out.
        if let Some(t) = getting_on {
            let s = smoothstep((t / GET_ON_EASE).clamp(0.0, 1.0));
            for bone in Bone::ALL {
                pose.rotations[bone] = self.body.stood.rotations[bone].slerp(pose.rotations[bone], s);
            }
        }
        pose
    }

    /// Each arm to its wrist at `wrists` (the world), the hand turned to
    /// `turns` by `weight`: the shoulder lifted toward it, the elbow out to
    /// the side and down.
    #[allow(clippy::too_many_arguments)]
    fn arms_to(&self, pose: &mut LocalPose, rig: &RigGeometry, root: Vec3, back: Quat, wrists: [Vec3; 2], turns: [Quat; 2], weight: f32) {
        let to_pose = |p: Vec3| back * (p - root);
        let targets = wrists.map(to_pose);
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            let lift = shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], targets[side], 0.85 * self.body.arms[side]);
            pose.rotations[CLAVICLES[side]] = delta_after_world_turn(pose, rig, CLAVICLES[side], lift);
        }
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            let chain = ARMS[side];
            let pole = (rig.left() * (SIGN[side] * 0.7) - Vec3::Y * 0.5 - rig.forward() * 0.2).normalize();
            let (elbow, wrist) = solve_arm_toward_from(pose, &at, chain, targets[side], pole, rig);
            turn_hand(pose, rig, chain, self.body.hand_binds[side], back * turns[side], weight, (wrist - elbow).normalize_or_zero());
        }
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the
    /// spring trails a steady motion (`jump::lead_of`).
    pub fn pose_led(&self, rig: &RigGeometry, springs: &BoneSet<SpringParams>) -> LocalPose {
        let mut pose = self.pose(rig);
        let mut posed: Vec<(f32, LocalPose)> = Vec::with_capacity(3);
        for bone in Bone::ALL {
            let lead = lead_of(&springs[bone]);
            if lead <= 1.0e-4 {
                continue;
            }
            pose.rotations[bone] = match posed.iter().find(|(at, _)| (at - lead).abs() < 1.0e-4) {
                Some((_, ahead)) => ahead.rotations[bone],
                None => {
                    let mut later = self.clone();
                    later.step(lead);
                    let ahead = later.pose(rig);
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }

    /// Where the walker's root is now: under the hips as the standing pose
    /// has them.
    pub fn root(&self) -> Vec3 {
        self.hips() - self.turn() * self.body.hips
    }

    /// The walker's facing now: toward the pole.
    pub fn facing(&self) -> f32 {
        self.round + std::f32::consts::PI - self.ahead
    }

    /// Where it looks: the pole above the upper hand.
    pub fn look(&self) -> Vec3 {
        let (holds, _) = self.holds_now();
        self.pole.at(holds.hands[0].max(holds.hands[1]) + 0.3)
    }

    /// How closed each hand is round the pole (1 holding, 0 open), as the
    /// hands' fingers want it.
    pub fn grips(&self) -> [f32; 2] {
        let (_, off) = self.holds_now();
        let closing = match self.phase {
            Phase::GettingOn { t } => smoothstep(((t / (GET_ON * HANDS_ON) - 0.8) / 0.2).clamp(0.0, 1.0)),
            Phase::Released { .. } => 0.0,
            _ => 1.0,
        };
        off.map(|off| closing * (1.0 - off / HAND_OFF).clamp(0.0, 1.0))
    }

    /// Each wrist's place on the pole now (the world), and whether it is
    /// there (not on its way up, nor getting on).
    pub fn wrists(&self) -> [(Vec3, bool); 2] {
        let (holds, off) = self.holds_now();
        let toward = -self.away();
        let left = self.turn() * self.rig.left();
        [0, 1].map(|side| {
            let grip = &self.body.grips[side];
            let hand = frame_turn(grip.along, grip.palm, -left * SIGN[side], toward);
            (self.pole.at(holds.hands[side]) - hand * grip.bar, off[side] == 0.0 && !matches!(self.phase, Phase::GettingOn { .. }))
        })
    }

    /// The feet's clamp's height on the pole now.
    pub fn feet_height(&self) -> f32 {
        self.holds_now().0.feet
    }

    /// Whether it holds the pole stood up, between moves.
    pub fn is_holding(&self) -> bool {
        self.phase == Phase::Holding
    }

    /// Whether it is climbing, up (`Some(true)`) or down.
    pub fn climbing(&self) -> Option<bool> {
        match self.phase {
            Phase::Climbing { up, .. } => Some(up),
            _ => None,
        }
    }

    /// Whether it is sliding down.
    pub fn is_sliding(&self) -> bool {
        matches!(self.phase, Phase::Sliding { .. })
    }

    /// Whether it has let go ([`Self::release`]).
    pub fn is_released(&self) -> bool {
        matches!(self.phase, Phase::Released { .. })
    }

    /// The pole it is on.
    pub fn pole(&self) -> &Pole {
        &self.pole
    }

    /// Let go: the fall from the pose now, to the ground `ground` finds
    /// under the root (`parkour::fall`), at the velocity it let go with.
    pub fn release(&self, ground: &dyn Fn(Vec3) -> Option<f32>, stood: &LocalPose, rig: &RigGeometry) -> Falling {
        let velocity = match self.phase {
            Phase::Released { velocity } => velocity,
            _ => Vec3::ZERO,
        };
        let root = self.root();
        let floor = ground(root).unwrap_or(self.floor);
        Falling::off(root, self.facing(), velocity, &self.pose(rig), floor, 0.0, stood, rig)
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

    /// What a time on a pole measured.
    #[derive(Debug, Default)]
    struct Poled {
        /// The most a held hand strays from its place on the pole; a foot's
        /// clamp from its; the deepest a joint goes into the pole.
        hand_off: f32,
        foot_off: f32,
        into: f32,
        /// The fastest joint about the hips, and when.
        fastest: f32,
        fastest_at: f32,
        /// The most a joint's step changes in a frame.
        kink: f32,
        kink_at: (f32, Option<Bone>),
    }

    fn world(pose: &LocalPose, root: Vec3, yaw: f32, rig: &RigGeometry) -> BoneSet<Vec3> {
        let at = forward_kinematics_on(pose, rig);
        BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
    }

    /// Runs `poling` asked `ask` for `seconds` (or until released),
    /// measuring.
    fn run(poling: &mut Poling, ask: Option<PoleAsk>, seconds: f32, m: &mut Poled, frames: &mut Vec<BoneSet<Vec3>>, t: &mut f32) {
        let (_, rig) = real_stood();
        let end = *t + seconds;
        while *t < end && !poling.is_released() {
            poling.advance(ask, DT);
            *t += DT;
            let now = world(&poling.pose(&rig), poling.root(), poling.facing(), &rig);
            if let Some(last) = frames.last() {
                let speed = Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (last[bone] - last[Bone::Hips])).length() / DT).fold(0.0, f32::max);
                if speed > m.fastest {
                    (m.fastest, m.fastest_at) = (speed, *t);
                }
            }
            if frames.len() >= 2 {
                let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
                let (kink, bone) = Bone::ALL.iter().map(|&bone| ((now[bone] - 2.0 * b[bone] + a[bone]).length(), bone)).fold((0.0, Bone::Hips), |a, b| if b.0 > a.0 { b } else { a });
                if kink > m.kink {
                    (m.kink, m.kink_at) = (kink, (*t, Some(bone)));
                }
            }
            for (side, (wrist, held)) in poling.wrists().into_iter().enumerate() {
                if held {
                    m.hand_off = m.hand_off.max((now[ARMS[side].wrist] - wrist).length());
                }
            }
            if !poling.is_sliding() && !matches!(poling.phase, Phase::GettingOn { .. }) {
                let left = poling.turn() * rig.left();
                for (side, &(_, _, ankle, _)) in LEGS.iter().enumerate() {
                    let clamp = poling.pole.at(poling.feet_height()) + left * (SIGN[side] * CLAMP_SIDE) + poling.away() * CLAMP_BEHIND;
                    m.foot_off = m.foot_off.max((now[ankle] - clamp).length());
                }
            }
            // Into the pole: any joint nearer its axis than its radius, and
            // the shins' and thighs' middles.
            let axis = |p: Vec3| (p - poling.pole.at(p.y)).with_y(0.0).length();
            let mut points: Vec<Vec3> = Bone::ALL.iter().map(|&bone| now[bone]).collect();
            for &(socket, knee, ankle, _) in &LEGS {
                points.push(0.5 * (now[socket] + now[knee]));
                points.push(0.5 * (now[knee] + now[ankle]));
            }
            for p in points.into_iter().filter(|p| p.y < poling.pole.top()) {
                m.into = m.into.max(poling.pole.radius - axis(p));
            }
            frames.push(now);
        }
    }

    /// Stood 0.5 m in front of a 5 m pole, it gets on and climbs up three
    /// cycles (2.3 m), holds, climbs back down, then slides to the floor
    /// and lands: the hands on the pole when holding, the feet clamping it,
    /// nothing into it, no joint whipping round, the pose continuous; the
    /// slide touching down no faster than its top speed.
    #[test]
    fn a_pole_is_climbed_up_and_down_and_slid_down() {
        let (stood, rig) = real_stood();
        for heading in [0.0f32, 1.3, -2.4] {
            let pole = Pole::new(Vec3::new(0.3, 0.0, -0.8), 5.0);
            let from = pole.foot + Quat::from_rotation_y(heading) * Vec3::Z * 2.0;
            let root = Poling::spot(&pole, from, &stood, &rig);
            let mut poling = Poling::get_on(&pole, root, &stood, &rig);
            let (mut m, mut frames, mut t) = (Poled::default(), vec![world(&stood, root, poling.facing(), &rig)], 0.0);
            run(&mut poling, None, 1.5, &mut m, &mut frames, &mut t);
            assert!(poling.is_holding(), "{heading}: not holding after getting on");
            let start = poling.holds.hips;
            run(&mut poling, Some(PoleAsk::Up), 3.0 * CYCLE - 0.5 * DT, &mut m, &mut frames, &mut t);
            run(&mut poling, None, 0.5, &mut m, &mut frames, &mut t);
            let climbed = poling.holds.hips - start;
            assert!((climbed - 3.0 * STROKE).abs() < 1.0e-3, "{heading}: climbed {climbed:.3} m, not three strokes");
            run(&mut poling, Some(PoleAsk::Down), 3.0 * CYCLE - 0.5 * DT, &mut m, &mut frames, &mut t);
            run(&mut poling, None, 0.5, &mut m, &mut frames, &mut t);
            assert!((poling.holds.hips - start).abs() < 1.0e-3, "{heading}: back down {:.3} m off where it got on", poling.holds.hips - start);
            run(&mut poling, Some(PoleAsk::Up), 3.0 * CYCLE, &mut m, &mut frames, &mut t);
            run(&mut poling, Some(PoleAsk::Slide), 10.0, &mut m, &mut frames, &mut t);
            assert!(poling.is_released(), "{heading}: never reached the floor");
            let Phase::Released { velocity } = poling.phase else { unreachable!() };
            assert!(velocity.length() <= SLIDE_TOP + 1.0e-3, "{heading}: touched down at {:.2} m/s", velocity.length());
            let mut falling = poling.release(&|_| Some(0.0), &stood, &rig);
            let held = *frames.last().unwrap();
            falling.advance(DT);
            let first = world(&falling.pose(&rig), falling.root(), falling.facing(), &rig);
            let kink = Bone::ALL.iter().map(|&bone| (first[bone] - 2.0 * held[bone] + frames[frames.len() - 2][bone]).length()).fold(0.0, f32::max);
            for _ in 0..(3.0 / DT) as usize {
                falling.advance(DT);
            }
            eprintln!("{heading}: {m:?}, landing kink {kink:.4}");
            assert!(falling.is_done(), "{heading}: never landed");
            assert!(m.hand_off < 1.0e-3, "{heading}: a held hand {:.4} m off the pole", m.hand_off);
            assert!(m.foot_off < 0.01, "{heading}: a foot {:.4} m off its clamp", m.foot_off);
            assert!(m.into < 1.0e-3, "{heading}: a joint {:.4} m into the pole", m.into);
            assert!(m.fastest < 14.0, "{heading}: a joint at {:.1} m/s about the hips at {:.2} s", m.fastest, m.fastest_at);
            assert!(m.kink < 0.02 && kink < 0.02, "{heading}: a joint's step changed {:.4} m in a frame ({kink:.4} letting go)", m.kink);
        }
    }

    /// Held on a pole, it goes round it a half turn either way, the hands
    /// on it; at the top it climbs no higher; let go, it falls clear and
    /// lands.
    #[test]
    fn a_pole_is_gone_round_climbed_to_its_top_and_let_go_of() {
        let (stood, rig) = real_stood();
        let pole = Pole::new(Vec3::new(0.0, 0.0, -0.8), 3.2);
        let root = Poling::spot(&pole, Vec3::ZERO, &stood, &rig);
        let mut poling = Poling::get_on(&pole, root, &stood, &rig);
        let (mut m, mut frames, mut t) = (Poled::default(), vec![world(&stood, root, poling.facing(), &rig)], 0.0);
        run(&mut poling, None, 1.5, &mut m, &mut frames, &mut t);
        let before = poling.round;
        // Eased in and out over the same time, a half turn's time at its
        // rate goes round a half turn.
        run(&mut poling, Some(PoleAsk::Round(Round::Left)), std::f32::consts::PI / SPIN_RATE, &mut m, &mut frames, &mut t);
        run(&mut poling, None, 1.0, &mut m, &mut frames, &mut t);
        let turned = poling.round - before;
        assert!((turned - std::f32::consts::PI).abs() < 0.2, "went round {turned:.2} rad, not a half turn");
        run(&mut poling, Some(PoleAsk::Round(Round::Right)), std::f32::consts::PI / SPIN_RATE, &mut m, &mut frames, &mut t);
        run(&mut poling, None, 1.0, &mut m, &mut frames, &mut t);
        assert!((poling.round - before).abs() < 0.2, "back round to {:.2} rad off", poling.round - before);
        run(&mut poling, Some(PoleAsk::Up), 10.0, &mut m, &mut frames, &mut t);
        let top = poling.holds.hands[0].max(poling.holds.hands[1]);
        assert!(top <= pole.top() - TOP_MARGIN + 1.0e-4 && top > pole.top() - TOP_MARGIN - STROKE, "the upper hand at {top:.2} m on a {:.1} m pole", pole.top());
        run(&mut poling, Some(PoleAsk::LetGo), 0.1, &mut m, &mut frames, &mut t);
        assert!(poling.is_released(), "did not let go");
        let mut falling = poling.release(&|_| Some(0.0), &stood, &rig);
        for _ in 0..(4.0 / DT) as usize {
            falling.advance(DT);
        }
        eprintln!("{m:?}");
        assert!(falling.is_done(), "never landed");
        let clear = (falling.root() - pole.foot).with_y(0.0).length();
        assert!(clear > HIPS_OFF, "landed {clear:.2} m from the pole");
        assert!(m.hand_off < 1.0e-3, "a held hand {:.4} m off the pole", m.hand_off);
        assert!(m.into < 1.0e-3, "a joint {:.4} m into the pole", m.into);
        assert!(m.fastest < 14.0, "a joint at {:.1} m/s about the hips at {:.2} s", m.fastest, m.fastest_at);
        assert!(m.kink < 0.02, "a joint's step changed {:.4} m in a frame", m.kink);
    }
}
