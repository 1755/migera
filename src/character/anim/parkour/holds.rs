//! Free climbing on holds: step 13 of the parkour design (the steps beyond
//! the first ten). A wall of hand- and footholds ([`HoldWall`], the holds
//! given as values, as a `Ledge` is) is climbed limb by limb
//! ([`FreeClimb`]): a move is one limb to a new hold, three held while it
//! moves, in a four-beat order for the way asked (up, down, aside or
//! diagonally), the hips following the holds. A hand hold out of reach
//! above is jumped for (a dyno: a sink, a drive, a flight to both hands on
//! it, the feet finding holds again after). With no foothold in reach the
//! feet hang free. At the bottom it steps off onto the floor; at the top it
//! takes the wall's lip into a hang (`Hanging::caught`), to climb up from.
//!
//! Climbing studies bound the pace: speed climbers make 2.5-2.8 hand moves
//! a second; here a hand moves in 0.55 s and a foot in 0.45, a recreational
//! climber's (no measured recreational pace: by eye).

use bevy::math::{Quat, Vec2, Vec3};

use super::{Falling, Hanging, Ledge};
use crate::character::anim::armik::{frame_turn, shoulder_lift, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::hand::{hook_lip, HandGrip, FINGER_HALF_THICKNESS, GRIP_RADIUS};
use crate::character::anim::jump::{lead_of, GRAVITY};
use crate::character::anim::math::SpringParams;
use crate::character::anim::rig::{accumulate_bind_rotations, accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// What a hold takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoldKind {
    /// An edge: a hand hooks over it, a foot stands on it.
    Edge,
    /// A jug: a deep hold, as an edge (a hand's or a foot's).
    Jug,
    /// A foothold only: too small for a hand.
    Foot,
}

impl HoldKind {
    fn hand(self) -> bool {
        self != HoldKind::Foot
    }
}

/// A hold on a wall: its top's middle (where a hand hooks, a foot's ball
/// stands), and what it takes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hold {
    pub at: Vec3,
    pub kind: HoldKind,
}

/// A flat wall of holds facing `out` (level, unit), the face through
/// `face`; its top a ledge to climb out onto, if it has one.
#[derive(Debug, Clone, PartialEq)]
pub struct HoldWall {
    pub holds: Vec<Hold>,
    pub out: Vec3,
    pub face: Vec3,
    pub top: Option<Ledge>,
}

impl HoldWall {
    /// A wall facing `out` through `face` with `holds`, no top.
    pub fn new(face: Vec3, out: Vec3, holds: Vec<Hold>) -> Self {
        Self { holds, out: out.with_y(0.0).normalize_or(Vec3::Z), face, top: None }
    }

    /// A grid of edges `columns` × `rows`, `across` and `up` apart, the
    /// lowest row `bottom` up and the grid's middle at `face` along the
    /// face, every other row shifted half `across` aside (a wall to test
    /// on).
    pub fn grid(face: Vec3, out: Vec3, columns: usize, rows: usize, across: f32, up: f32, bottom: f32) -> Self {
        let out = out.with_y(0.0).normalize_or(Vec3::Z);
        let along = out.cross(Vec3::Y);
        let mut holds = Vec::new();
        for row in 0..rows {
            for column in 0..columns {
                // Every other row a little aside, as a real wall's are.
                let shift = if row % 2 == 0 { 0.0 } else { 0.5 * across };
                let u = (column as f32 - 0.5 * (columns as f32 - 1.0)) * across + shift;
                holds.push(Hold { at: face.with_y(0.0) + along * u + Vec3::Y * (bottom + row as f32 * up), kind: HoldKind::Edge });
            }
        }
        Self::new(face, out, holds)
    }

    /// Along the face, level: the left of a body facing it.
    pub fn along(&self) -> Vec3 {
        self.out.cross(Vec3::Y)
    }

    /// `point`'s place on the face: along it from `face`, up (the world's
    /// height), and out in front of it.
    pub fn place(&self, point: Vec3) -> (Vec2, f32) {
        let off = point - self.face;
        (Vec2::new(off.dot(self.along()), point.y), off.dot(self.out))
    }
}

/// The way asked, on the face: `x` along it (toward a body's left facing
/// it), `y` up.
pub type ClimbWay = Vec2;

/// The hips this far out from the face, metres, feet on holds; feet free.
const HIPS_OUT: f32 = 0.32;
const HIPS_OUT_FREE: f32 = 0.26;
/// The trunk leant toward the wall, radians.
const LEAN: f32 = 0.15;
/// The hands over the hips, metres: the most and least (the arms straight
/// and bent); hanging free, this much.
/// Hanging free 1.0 m under the hands, the arms were straight and an elbow
/// moving off it flipped 22 cm in a frame.
const HANDS_ABOVE: (f32, f32) = (0.65, 0.95);
const HANG_BELOW: f32 = 0.9;
/// An arm's wrist kept softly within this share of its length from its
/// shoulder, from the reach a held hand keeps to (straight, its elbow has no
/// way to bend and flips).
const ARM_SOFT: (f32, f32) = (0.95, 0.99);
/// A hand come down to its shoulder (the body risen past its hold) has its
/// elbow pulled down: from this far above the shoulder, metres, fully over
/// the second this far below that. Out sideways, a hand held at the
/// shoulder stuck its elbow straight out level with it.
const LOCK_OFF: (f32, f32) = (0.2, 0.3);
/// The feet under the hips, metres, braced.
const FEET_BELOW: f32 = 0.75;
/// A limb reaches this share of its length at most; a foot's hold no nearer
/// its socket than this share (folded up).
const ARM_REACH: f32 = 0.95;
const LEG_REACH: f32 = 0.95;
const LEG_FOLD: f32 = 0.4;
/// A move's least progress the way asked, metres; and the progress a hand
/// and a foot each look for.
const LEAST_PROGRESS: f32 = 0.1;
const HAND_STEP: f32 = 0.4;
const FOOT_STEP: f32 = 0.35;
/// A hand's and a foot's move, seconds; each comes this far off the wall on
/// its way, metres.
const HAND_MOVE: f32 = 0.55;
const FOOT_MOVE: f32 = 0.45;
const HAND_ARC: f32 = 0.08;
const FOOT_ARC: f32 = 0.06;
/// A wrist this far out from its hold, the ankle this far out and up from
/// the ball on its hold, metres; the foot pitched toes down this much,
/// radians.
const ANKLE_OUT: f32 = 0.05;
const TOES_DOWN: f32 = 0.35;
/// The feet keep this far below the lower hand, metres; each knee joint
/// this far out from the face.
const FEET_UNDER_HANDS: f32 = 0.5;
const KNEE_CLEAR: f32 = 0.06;
/// The knee's turn out starts this far either side of its clearance and
/// goes this share of the way to sideways at most (over a band of 12 cm,
/// a knee swung 30 cm out in a quarter second).
const KNEE_SOFT: f32 = 0.12;
const KNEE_MOST_OUT: f32 = 0.6;
/// A foothold at least this far under the hips, metres (higher, the knee
/// folds into the wall).
const FOOT_UNDER_HIPS: f32 = 0.5;
/// A foot looks for a hold this far to its side of the hips' middle, a
/// hand this far.
const FOOT_ASIDE: f32 = 0.15;
const HAND_ASIDE: f32 = 0.2;
/// Let go in a dyno, the feet leave their holds over this share of its
/// flight.
const FEET_LEAVE: f32 = 0.7;
/// A dyno: a hand hold this much over the higher hand at most, and this
/// far aside of the hands' middle, metres (measured from the shoulders'
/// reach instead, a gap of 1.1 m was never jumped); the sink and the
/// drive, seconds; the hips sunk this much, metres.
const DYNO_ABOVE: f32 = 1.3;
const DYNO_ASIDE: f32 = 0.5;
/// In a dyno's flight, each hand this much nearer its shoulder halfway (at
/// 0.12 the bent elbow came forward 1.5 cm into the wall; turning its pole
/// back from the wall instead, nearly along an arm reaching for it, flipped
/// elbows); going with the body by this share of the flight.
const DYNO_BEND: f32 = 0.12;
const HANDS_FOLLOW: f32 = 0.4;
/// Flying, each hand this far off the wall halfway, metres.
const FLYING_OFF: f32 = 0.12;
const SINK: f32 = 0.25;
const DRIVE: f32 = 0.2;
const SUNK: f32 = 0.12;
/// Both hands caught on one hold, this far either side of it, metres.
const MATCHED: f32 = 0.09;
/// Getting on from standing, seconds; the standing pose eased out over the
/// first this long.
const GET_ON: f32 = 1.0;
const GET_ON_EASE: f32 = 0.2;
/// The root this far out from the face to get on, metres.
pub const GET_ON_OFF: f32 = 0.4;
/// At the bottom, the lower foot's hold no higher than this asked down, it
/// steps off onto the floor, metres.
const STEP_OFF: f32 = 0.45;
/// At the top, both hands on the lip asked up (this near its height), it
/// takes the lip; the lip's holds this far apart along it; it blends into
/// the hang over this long, seconds (taken at once, a joint moved 25 cm in
/// a frame: the hang's own arms and legs).
const TOP_REACH: f32 = 0.01;
const LIP_HOLDS: f32 = 0.1;
const TOP_BLEND: f32 = 0.3;
/// Where a hand without known fingers hooks, metres along it.
const GUESSED_KNUCKLES: f32 = 0.08;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
const LEGS: [(Bone, Bone, Bone, Bone); 2] =
    [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase)];
const SIGN: [f32; 2] = [1.0, -1.0];
/// Limbs: left hand, right hand, left foot, right foot.
const LH: usize = 0;
const RH: usize = 1;
const LF: usize = 2;
const RF: usize = 3;

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    hips: Vec3,
    /// Each shoulder and hip socket from the hips joint, the leaned trunk's
    /// pose frame; each arm's and leg's length.
    shoulders: [Vec3; 2],
    sockets: [Vec3; 2],
    arms: [f32; 2],
    legs: [f32; 2],
    /// Each hand's and ankle's standing place, the pose's frame; each ankle
    /// from its ball; each foot's standing world rotation.
    hands: [Vec3; 2],
    ankles: [Vec3; 2],
    balls: [Vec3; 2],
    attitudes: [Quat; 2],
    hand_binds: [Quat; 2],
    grips: [HandGrip; 2],
}

impl Body {
    fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let at = forward_kinematics_on(stood, rig);
        let leaned = forward_kinematics_on(&crate::character::anim::jump::upper(stood, rig, LEAN, (0.0, 0.0)), rig);
        let world = accumulate_world_rotations(stood, rig);
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let binds = accumulate_bind_rotations(rig);
        let hips = at[Bone::Hips];
        Self {
            stood: *stood,
            hips,
            shoulders: ARMS.map(|arm| leaned[arm.shoulder] - leaned[Bone::Hips]),
            sockets: LEGS.map(|(socket, _, _, _)| at[socket] - hips),
            arms: ARMS.map(|arm| (at[arm.elbow] - at[arm.shoulder]).length() + (at[arm.wrist] - at[arm.elbow]).length()),
            legs: LEGS.map(|(socket, knee, ankle, _)| (at[knee] - at[socket]).length() + (at[ankle] - at[knee]).length()),
            hands: ARMS.map(|arm| at[arm.wrist]),
            ankles: LEGS.map(|(_, _, ankle, _)| at[ankle]),
            balls: LEGS.map(|(_, _, ankle, toe)| at[ankle] - at[toe]),
            attitudes: LEGS.map(|(_, _, ankle, _)| world[ankle]),
            hand_binds: ARMS.map(|arm| binds[arm.wrist]),
            grips: ARMS.map(|arm| {
                let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
                let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
                HandGrip { bar: along * GUESSED_KNUCKLES + palm * (GRIP_RADIUS + FINGER_HALF_THICKNESS), palm, along }
            }),
        }
    }
}

/// What a climber is doing.
#[derive(Debug, Clone, PartialEq)]
enum Doing {
    /// Getting on from standing, `t` seconds in.
    GettingOn { t: f32 },
    /// Held on its holds.
    Holding,
    /// Moving `limb` from where it was to hold `to` (`None`: let free, or
    /// a free foot hung under the hips), `t` seconds in; the hips from and
    /// to.
    Moving { limb: usize, from: Vec3, to: Option<usize>, t: f32, seconds: f32, hips: (Vec3, Vec3) },
    /// A dyno at hold `to`, `t` seconds in.
    Dyno { to: usize, t: f32, plan: Box<DynoPlan> },
    /// Let go onto the floor; taking the top's lip into a hang (blending
    /// into it, [`Topping`]), and taken.
    SteppedOff,
    ToppingOut,
    ToppedOut,
}

/// Taking the top's lip: the hang caught from the climb's pose, moving on,
/// blended into from that pose and root over [`TOP_BLEND`].
#[derive(Debug, Clone)]
struct Topping {
    t: f32,
    hang: Hanging,
    from: LocalPose,
    from_root: Vec3,
}

/// A dyno's path: the hips sunk, driven up to the release, flown to the
/// catch; where the hands catch.
#[derive(Debug, Clone, PartialEq)]
struct DynoPlan {
    start: Vec3,
    sunk: Vec3,
    release: Vec3,
    velocity: Vec3,
    catch: Vec3,
    flight: f32,
    hands: [Vec3; 2],
    /// Where the hands were as it let go.
    let_go: [Vec3; 2],
}

impl DynoPlan {
    fn length(&self) -> f32 {
        SINK + DRIVE + self.flight
    }

    /// The hips `t` seconds in.
    fn hips(&self, t: f32) -> Vec3 {
        let hermite = |a: Vec3, va: Vec3, b: Vec3, vb: Vec3, s: f32, length: f32| {
            let (s2, s3) = (s * s, s * s * s);
            a * (2.0 * s3 - 3.0 * s2 + 1.0) + va * length * (s3 - 2.0 * s2 + s) + b * (-2.0 * s3 + 3.0 * s2) + vb * length * (s3 - s2)
        };
        if t < SINK {
            hermite(self.start, Vec3::ZERO, self.sunk, Vec3::ZERO, t / SINK, SINK)
        } else if t < SINK + DRIVE {
            hermite(self.sunk, Vec3::ZERO, self.release, self.velocity, (t - SINK) / DRIVE, DRIVE)
        } else {
            let tau = (t - SINK - DRIVE).min(self.flight);
            self.release + self.velocity * tau - Vec3::Y * (0.5 * GRAVITY * tau * tau)
        }
    }
}

/// Free climbing a wall of holds.
#[derive(Debug, Clone)]
pub struct FreeClimb {
    wall: HoldWall,
    body: Body,
    rig: RigGeometry,
    /// The facing turn (toward the wall), and its yaw.
    turn: Quat,
    yaw: f32,
    /// Each limb's hold (left hand, right hand, left foot, right foot); a
    /// foot's may be none (hanging free).
    limbs: [Option<usize>; 4],
    /// The hips now, the world.
    hips: Vec3,
    doing: Doing,
    /// The way asked, and where in its four-beat order it is.
    way: Option<ClimbWay>,
    next: usize,
    /// Getting on: where it stood.
    stood_at: Vec3,
    floor: f32,
    /// Each hand's own grip, if its fingers are known (for the hang it
    /// tops out into); taking the top's lip.
    grips: [Option<HandGrip>; 2],
    topping: Option<Box<Topping>>,
}

impl FreeClimb {
    /// Getting on `wall` from standing with the root at `root` in front of
    /// it (on `rig`, standing `stood`): the hands to the two hand holds
    /// nearest over its shoulders within reach, one each side, the feet to
    /// the footholds that put the hips where both reach. `None` if no two
    /// such hand holds.
    pub fn get_on(wall: &HoldWall, root: Vec3, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let body = Body::of(stood, rig);
        // The top's lip a row of hand holds, so the hands climb onto it
        // before it is taken into a hang (taken from holds 0.3 m under it,
        // the hands jumped 32 cm onto it).
        let mut wall = wall.clone();
        if let Some(top) = wall.top {
            let length = (top.b - top.a).length();
            let count = (length / LIP_HOLDS).floor() as usize;
            for k in 1..count {
                wall.holds.push(Hold { at: top.a + top.along() * (k as f32 * LIP_HOLDS), kind: HoldKind::Edge });
            }
        }
        let wall = &wall;
        let yaw = crate::character::anim::approach::heading_of(-wall.out) - crate::character::anim::approach::heading_of(rig.forward());
        let turn = Quat::from_rotation_y(yaw);
        let mut climb = Self {
            wall: wall.clone(),
            body,
            rig: rig.clone(),
            turn,
            yaw,
            limbs: [None; 4],
            hips: root + turn * forward_kinematics_on(stood, rig)[Bone::Hips],
            doing: Doing::GettingOn { t: 0.0 },
            way: None,
            next: 0,
            stood_at: root,
            floor: root.y,
            grips: [None; 2],
            topping: None,
        };
        // The hands: over the shoulders, the hips as standing raised a little
        // and brought to where they hang off the face.
        let (_, out) = wall.place(climb.hips);
        let lifted = climb.hips + wall.out * (HIPS_OUT - out) + Vec3::Y * 0.2;
        let hand_for = |side: usize, taken: Option<usize>| {
            let shoulder = lifted + climb.turn * climb.body.shoulders[side];
            wall.holds
                .iter()
                .enumerate()
                .filter(|(i, hold)| hold.kind.hand() && Some(*i) != taken && hold.at.y > shoulder.y && (climb.wrist_for(side, hold.at) - shoulder).length() <= ARM_REACH * climb.body.arms[side])
                .min_by(|(_, a), (_, b)| (a.at - shoulder).length().total_cmp(&(b.at - shoulder).length()))
                .map(|(i, _)| i)
        };
        let left = hand_for(0, None)?;
        let right = hand_for(1, Some(left))?;
        let (left, right) = if wall.place(wall.holds[left].at).0.x >= wall.place(wall.holds[right].at).0.x { (left, right) } else { (right, left) };
        climb.limbs[LH] = Some(left);
        climb.limbs[RH] = Some(right);
        climb.limbs[LF] = None;
        climb.limbs[RF] = None;
        // The feet: the best footholds for the hips the hands put.
        for foot in [LF, RF] {
            climb.limbs[foot] = climb.foothold_for(foot, &climb.limbs);
        }
        climb.hips = climb.hips_for(&climb.limbs);
        Some(climb)
    }

    /// Where the root stands to get on `wall` coming from `from`: in front
    /// of it [`GET_ON_OFF`] out, level with `from` along it.
    pub fn spot(wall: &HoldWall, from: Vec3) -> Vec3 {
        let (place, _) = wall.place(from);
        (wall.face + wall.along() * place.x + wall.out * GET_ON_OFF).with_y(from.y)
    }

    /// Each hand placed so its own fingers hook its hold.
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2]) {
        self.grips = grips;
        for (side, grip) in grips.into_iter().enumerate() {
            if let Some(grip) = grip {
                self.body.grips[side] = grip;
            }
        }
    }

    /// The hand's world turn hooked over a hold: the palm against the wall,
    /// the fingers up over it.
    fn hook_turn(&self, side: usize) -> Quat {
        let grip = &self.body.grips[side];
        frame_turn(grip.along, grip.palm, Vec3::Y, -self.wall.out)
    }

    /// The wrist for hand `side` hooked on a hold at `at`.
    fn wrist_for(&self, side: usize, at: Vec3) -> Vec3 {
        at - self.hook_turn(side) * hook_lip(&self.body.grips[side])
    }

    /// The ankle for foot `side`, its ball on a hold at `at`, and its
    /// world rotation (facing the wall, toes down a little).
    fn ankle_for(&self, side: usize, at: Vec3) -> (Vec3, Quat) {
        let axis = self.turn * self.rig.left();
        let attitude = Quat::from_axis_angle(axis, TOES_DOWN) * self.turn * self.body.attitudes[side];
        let ball_to_ankle = attitude * (self.turn * self.body.attitudes[side]).inverse() * (self.turn * self.body.balls[side]);
        (at + self.wall.out * ANKLE_OUT * 0.0 + ball_to_ankle + self.wall.out * ANKLE_OUT, attitude)
    }

    /// Where the hips go held by `limbs`: along the face between the held
    /// limbs, up between what the hands and feet allow, out from the face.
    fn hips_for(&self, limbs: &[Option<usize>; 4]) -> Vec3 {
        let at = |limb: usize| limbs[limb].map(|i| self.wall.holds[i].at);
        let hands: Vec<Vec3> = [LH, RH].iter().filter_map(|&l| at(l)).collect();
        let feet: Vec<Vec3> = [LF, RF].iter().filter_map(|&l| at(l)).collect();
        let mean = |points: &[Vec3]| points.iter().copied().sum::<Vec3>() / points.len().max(1) as f32;
        let held: Vec<Vec3> = hands.iter().chain(feet.iter()).copied().collect();
        let (along, _) = self.wall.place(mean(&held));
        let hand_y = mean(&hands).y;
        let (height, out) = if feet.is_empty() {
            (hand_y - HANG_BELOW, HIPS_OUT_FREE)
        } else {
            ((mean(&feet).y + FEET_BELOW).clamp(hand_y - HANDS_ABOVE.1, hand_y - HANDS_ABOVE.0), HIPS_OUT)
        };
        self.wall.face.with_y(0.0) + self.wall.along() * along.x + self.wall.out * out + Vec3::Y * height
    }

    /// Whether hand `side` reaches a hold at `at` from hips at `hips`.
    fn hand_reaches(&self, side: usize, at: Vec3, hips: Vec3) -> bool {
        let shoulder = hips + self.turn * self.body.shoulders[side];
        (self.wrist_for(side, at) - shoulder).length() <= ARM_REACH * self.body.arms[side]
    }

    /// Whether foot `side` reaches a hold at `at` from hips at `hips`: not
    /// too far, nor too high under them.
    fn foot_reaches(&self, side: usize, at: Vec3, hips: Vec3) -> bool {
        let socket = hips + self.turn * self.body.sockets[side];
        let off = (self.ankle_for(side, at).0 - socket).length() / self.body.legs[side];
        (LEG_FOLD..=LEG_REACH).contains(&off) && hips.y - at.y >= FOOT_UNDER_HIPS
    }

    /// The best foothold for `foot` with the other limbs on `limbs`: under
    /// the hands, its side of the other foot, nearest where a foot hangs
    /// [`FEET_BELOW`] under the hips; `None` with none in reach.
    fn foothold_for(&self, foot: usize, limbs: &[Option<usize>; 4]) -> Option<usize> {
        let side = foot - LF;
        let lowest_hand = [LH, RH].iter().filter_map(|&l| limbs[l]).map(|i| self.wall.holds[i].at.y).fold(f32::MAX, f32::min);
        let other = limbs[LF + 1 - side].map(|i| self.wall.place(self.wall.holds[i].at).0.x);
        let mut trial = *limbs;
        trial[foot] = None;
        let hips_free = self.hips_for(&trial);
        let wanted = hips_free.y - FEET_BELOW + 0.25;
        // Under its own hip (with nothing else to keep it its side, the left
        // foot took a hold right of the hips and the right one went 0.75 m
        // out).
        let under_hip = self.wall.place(hips_free).0.x + SIGN[side] * FOOT_ASIDE;
        self.wall
            .holds
            .iter()
            .enumerate()
            .filter(|(i, hold)| !limbs.contains(&Some(*i)) && hold.at.y <= lowest_hand - FEET_UNDER_HANDS && hold.at.y > self.floor + 0.1)
            .filter(|(_, hold)| other.is_none_or(|o| SIGN[side] * (self.wall.place(hold.at).0.x - o) >= -0.05))
            .filter(|(i, _)| {
                let mut with = *limbs;
                with[foot] = Some(*i);
                let hips = self.hips_for(&with);
                self.foot_reaches(side, self.wall.holds[*i].at, hips) && [LH, RH].iter().all(|&h| with[h].is_none_or(|j| self.hand_reaches(h, self.wall.holds[j].at, hips)))
            })
            .min_by(|(_, a), (_, b)| {
                let cost = |hold: &Hold| (hold.at.y - wanted).abs() + (self.wall.place(hold.at).0.x - under_hip).abs();
                cost(a).total_cmp(&cost(b))
            })
            .map(|(i, _)| i)
    }

    /// The four-beat order for `way`: up, a hand then the other side's
    /// foot; down, the feet first; aside, the leading side's hand then
    /// foot.
    fn order(way: ClimbWay) -> [usize; 4] {
        if way.y.abs() >= way.x.abs() {
            if way.y >= 0.0 { [RH, LF, LH, RF] } else { [LF, LH, RF, RH] }
        } else if way.x >= 0.0 {
            [LH, LF, RH, RF]
        } else {
            [RH, RF, LH, LF]
        }
    }

    /// The best hold for `limb` the way `way` from `limbs`: progressing at
    /// least [`LEAST_PROGRESS`], nearest a step's progress, little across
    /// the way, on its own side of its pair, every limb still in reach of
    /// the hips it leaves.
    fn hold_for(&self, limb: usize, way: ClimbWay, limbs: &[Option<usize>; 4]) -> Option<usize> {
        let hand = limb < LF;
        let side = if hand { limb } else { limb - LF };
        let pair = if hand { 1 - limb } else { LF + 1 - side };
        let from = limbs[limb].map(|i| self.wall.holds[i].at).unwrap_or(self.hips_for(limbs) - Vec3::Y * if hand { -0.8 } else { FEET_BELOW });
        let (from_place, _) = self.wall.place(from);
        let pair_x = limbs[pair].map(|i| self.wall.place(self.wall.holds[i].at).0.x);
        let way = way.normalize_or_zero();
        let step = if hand { HAND_STEP } else { FOOT_STEP };
        self.wall
            .holds
            .iter()
            .enumerate()
            .filter(|(i, hold)| !limbs.contains(&Some(*i)) && (!hand || hold.kind.hand()) && hold.at.y > self.floor + 0.1)
            .filter_map(|(i, hold)| {
                let (place, _) = self.wall.place(hold.at);
                let moved = place - from_place;
                let progress = moved.dot(way);
                let across = moved.perp_dot(way).abs();
                if progress < LEAST_PROGRESS || pair_x.is_some_and(|x| SIGN[side] * (place.x - x) < -0.05) {
                    return None;
                }
                let mut with = *limbs;
                with[limb] = Some(i);
                // The feet under the hands.
                let lowest_hand = [LH, RH].iter().filter_map(|&l| with[l]).map(|j| self.wall.holds[j].at.y).fold(f32::MAX, f32::min);
                if [LF, RF].iter().filter_map(|&l| with[l]).any(|j| self.wall.holds[j].at.y > lowest_hand - FEET_UNDER_HANDS) {
                    return None;
                }
                let hips = self.hips_for(&with);
                let reached = [LH, RH].iter().all(|&h| with[h].is_none_or(|j| self.hand_reaches(h, self.wall.holds[j].at, hips)))
                    && [LF, RF].iter().all(|&f| with[f].is_none_or(|j| self.foot_reaches(f - LF, self.wall.holds[j].at, hips)));
                // Each limb its own side of the hips: a foot under its hip,
                // a hand over its shoulder.
                let own = self.wall.place(hips).0.x + SIGN[side] * if hand { HAND_ASIDE } else { FOOT_ASIDE };
                reached.then_some((i, (progress - step).abs() + 0.5 * across + 0.5 * (place.x - own).abs()))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }

    /// A dyno up toward `way` (mostly up): the best hand hold above the
    /// hands, no more than [`DYNO_ABOVE`] over the higher (none in a move's
    /// reach, or it would be climbed to).
    fn dyno_for(&self, way: ClimbWay) -> Option<usize> {
        if way.y < 0.7 * way.length() {
            return None;
        }
        let hands = [LH, RH].map(|l| self.limbs[l].map(|i| self.wall.holds[i].at));
        let top = hands.iter().flatten().map(|at| at.y).fold(f32::MIN, f32::max);
        let middle = hands.iter().flatten().copied().sum::<Vec3>() / 2.0;
        self.wall
            .holds
            .iter()
            .enumerate()
            .filter(|(i, hold)| !self.limbs.contains(&Some(*i)) && hold.kind.hand() && hold.at.y > top + LEAST_PROGRESS && hold.at.y <= top + DYNO_ABOVE)
            .filter(|(_, hold)| (self.wall.place(hold.at).0.x - self.wall.place(middle).0.x).abs() <= DYNO_ASIDE)
            .min_by(|(_, a), (_, b)| (self.wall.place(a.at).0.x - self.wall.place(middle).0.x).abs().total_cmp(&(self.wall.place(b.at).0.x - self.wall.place(middle).0.x).abs()))
            .map(|(i, _)| i)
    }

    /// Plans a dyno to hold `to`: sunk toward the wall, driven up, flown to
    /// the catch at the top of its flight, both hands on the hold.
    fn plan_dyno(&self, to: usize) -> DynoPlan {
        let hold = self.wall.holds[to].at;
        let along = self.wall.along();
        let hands = [hold + along * MATCHED, hold - along * MATCHED];
        let mut caught = self.limbs;
        caught[LF] = None;
        caught[RF] = None;
        caught[LH] = Some(to);
        caught[RH] = Some(to);
        let catch = self.hips_for(&caught);
        let start = self.hips;
        // Sunk no lower than the hands still reach (hanging straight-armed,
        // a sink of 12 cm left a hand 7.4 cm off its hold).
        let lowest_hand = [LH, RH].iter().filter_map(|&l| self.limbs[l]).map(|i| self.wall.holds[i].at.y).fold(f32::MAX, f32::min);
        let sunk = start - Vec3::Y * SUNK.min((start.y - (lowest_hand - HANDS_ABOVE.1)).max(0.0));
        // Released a third of the way up, the rest flown.
        let release = sunk + (catch - sunk) * 0.35;
        let rise = (catch.y - release.y).max(0.05);
        let up = (2.0 * GRAVITY * rise).sqrt();
        let flight = up / GRAVITY;
        let velocity = (catch - release).with_y(0.0) / flight + Vec3::Y * up;
        // Where each hand is, matched ones aside of their hold (from the
        // hold's middle, a matched hand jumped 9 cm as it let go).
        let let_go = [LH, RH].map(|l| self.limb_at(l));
        DynoPlan { start, sunk, release, velocity, catch, flight, hands, let_go }
    }

    /// Moves it on `dt` seconds, asked to climb `way` (or held).
    pub fn advance(&mut self, way: Option<ClimbWay>, dt: f32) {
        if self.way.map(Self::order) != way.map(Self::order) {
            self.next = 0;
        }
        self.way = way;
        let mut dt = dt;
        // A move ending mid-frame, the next starts with the rest of it.
        for _ in 0..3 {
            dt = self.step(dt);
            if dt <= 0.0 {
                break;
            }
        }
    }

    /// Moves it on up to `dt` seconds; what is left over when a move ends.
    fn step(&mut self, dt: f32) -> f32 {
        match &mut self.doing {
            Doing::GettingOn { t } => {
                *t += dt;
                if *t >= GET_ON {
                    let left = *t - GET_ON;
                    self.doing = Doing::Holding;
                    return left;
                }
                0.0
            }
            Doing::Moving { limb, to, t, seconds, hips, .. } => {
                *t += dt;
                let (limb, to, seconds, hips) = (*limb, *to, *seconds, *hips);
                self.hips = hips.0.lerp(hips.1, smoothstep((*t / seconds).clamp(0.0, 1.0)));
                if *t >= seconds {
                    let left = *t - seconds;
                    self.limbs[limb] = to;
                    self.hips = hips.1;
                    self.doing = Doing::Holding;
                    return left;
                }
                0.0
            }
            Doing::Dyno { to, t, plan } => {
                *t += dt;
                let (to, length) = (*to, plan.length());
                self.hips = plan.hips(*t);
                if *t >= length {
                    let left = *t - length;
                    self.limbs[LH] = Some(to);
                    self.limbs[RH] = Some(to);
                    self.limbs[LF] = None;
                    self.limbs[RF] = None;
                    self.hips = plan.catch;
                    self.doing = Doing::Holding;
                    return left;
                }
                0.0
            }
            Doing::ToppingOut => {
                if let Some(topping) = self.topping.as_mut() {
                    topping.t += dt;
                    topping.hang.advance(dt);
                    if topping.t >= TOP_BLEND {
                        self.doing = Doing::ToppedOut;
                    }
                }
                0.0
            }
            Doing::SteppedOff | Doing::ToppedOut => 0.0,
            Doing::Holding => {
                // Free feet find holds again before anything else.
                for foot in [LF, RF] {
                    if self.limbs[foot].is_none()
                        && let Some(hold) = self.foothold_for(foot, &self.limbs)
                    {
                        self.start_move(foot, Some(hold));
                        return dt;
                    }
                }
                let Some(way) = self.way else { return 0.0 };
                // At the bottom, asked down, it steps off.
                let lowest_foot = [LF, RF].iter().filter_map(|&l| self.limbs[l]).map(|i| self.wall.holds[i].at.y).fold(f32::MAX, f32::min);
                if way.y < -0.5 * way.length() && lowest_foot - self.floor <= STEP_OFF {
                    self.doing = Doing::SteppedOff;
                    return 0.0;
                }
                // At the top, asked up, it takes the lip.
                if let Some(top) = self.wall.top
                    && way.y > 0.5 * way.length()
                    && [LH, RH].iter().all(|&l| self.limbs[l].is_some_and(|i| top.height() - self.wall.holds[i].at.y <= TOP_REACH))
                {
                    let (pose, root) = (self.pose(), self.root());
                    let square = Hanging::square(&top, self.rig.forward());
                    let hang = Hanging::caught(&top, &[], self.hips, Vec3::ZERO, &pose, root, square, 0.0, self.grips, &self.body.stood, &self.rig);
                    self.topping = Some(Box::new(Topping { t: 0.0, hang, from: pose, from_root: root }));
                    self.doing = Doing::ToppingOut;
                    return 0.0;
                }
                let order = Self::order(way);
                for k in 0..4 {
                    let limb = order[(self.next + k) % 4];
                    if let Some(hold) = self.hold_for(limb, way, &self.limbs) {
                        self.next = (self.next + k + 1) % 4;
                        self.start_move(limb, Some(hold));
                        return dt;
                    }
                }
                // A dyno driven off the feet: with them free, none (made
                // hanging, the arms flipped their elbows as they let go).
                if self.limbs[LF].is_some()
                    && self.limbs[RF].is_some()
                    && let Some(to) = self.dyno_for(way)
                {
                    let plan = Box::new(self.plan_dyno(to));
                    self.doing = Doing::Dyno { to, t: 0.0, plan };
                    return dt;
                }
                0.0
            }
        }
    }

    /// Starts moving `limb` to hold `to`.
    fn start_move(&mut self, limb: usize, to: Option<usize>) {
        let from = self.limb_at(limb);
        let mut with = self.limbs;
        with[limb] = to;
        let seconds = if limb < LF { HAND_MOVE } else { FOOT_MOVE };
        self.doing = Doing::Moving { limb, from, to, t: 0.0, seconds, hips: (self.hips, self.hips_for(&with)) };
    }

    /// Where limb `limb` is now (its hold, or where a free foot hangs); both
    /// hands on one hold, [`MATCHED`] either side of it.
    fn limb_at(&self, limb: usize) -> Vec3 {
        match self.limbs[limb] {
            Some(i) if limb < LF && self.limbs[1 - limb] == Some(i) => self.wall.holds[i].at + self.wall.along() * (SIGN[limb] * MATCHED),
            Some(i) => self.wall.holds[i].at,
            None => self.hips - Vec3::Y * FEET_BELOW + self.turn * self.rig.left() * (SIGN[limb % 2] * 0.12),
        }
    }

    /// Each limb's target now (the world): a hand's hold or its way to one,
    /// a foot's ball on its hold or hanging; and whether it is held.
    fn targets(&self) -> [(Vec3, bool); 4] {
        let mut targets = [0, 1, 2, 3].map(|limb| (self.limb_at(limb), self.limbs[limb].is_some()));
        match &self.doing {
            Doing::Moving { limb, from, to, t, seconds, .. } => {
                let s = (t / seconds).clamp(0.0, 1.0);
                let end = to.map_or(self.hips - Vec3::Y * FEET_BELOW, |i| self.wall.holds[i].at);
                let arc = if *limb < LF { HAND_ARC } else { FOOT_ARC };
                targets[*limb] = (from.lerp(end, smoothstep(s)) + self.wall.out * (arc * (std::f32::consts::PI * s).sin().powi(2)), false);
            }
            Doing::Dyno { t, plan, .. } => {
                let released = *t >= SINK + DRIVE;
                for (side, limb) in [LH, RH].into_iter().enumerate() {
                    if released {
                        // Swept round the shoulder, the elbow bent on the
                        // way (straight there, the arm passed through
                        // straight and its elbow flipped 39 cm in a frame).
                        let flown = ((t - SINK - DRIVE) / plan.flight).clamp(0.0, 1.0);
                        let s = smoothstep(flown);
                        let shoulder = |hips: Vec3| hips + self.turn * self.body.shoulders[side];
                        let from = plan.let_go[side] - shoulder(plan.release);
                        let to = plan.hands[side] - shoulder(plan.catch);
                        let swept = Quat::IDENTITY.slerp(Quat::from_rotation_arc(from.normalize(), to.normalize()), s) * from.normalize();
                        let reach = from.length() + (to.length() - from.length()) * s - DYNO_BEND * (std::f32::consts::PI * s).sin();
                        // From where it let go, still, into going with the
                        // body (with it at once, the hand went from still to
                        // the body's 3.3 m/s in a frame, an elbow 12 cm off
                        // its path).
                        let with_body = smoothstep((flown / HANDS_FOLLOW).clamp(0.0, 1.0));
                        let at = plan.let_go[side].lerp(shoulder(self.hips) + swept * reach, with_body);
                        // Off the wall until the catch: an arc round the
                        // shoulder between two holds on it bulges through it
                        // (2.3 cm), and the elbow behind the hand came 2.8 cm
                        // into it.
                        let (_, out) = self.wall.place(at);
                        let off = FLYING_OFF * (std::f32::consts::PI * s).sin();
                        targets[limb] = (at + self.wall.out * (off - out).max(0.0), false);
                    }
                }
                if released {
                    // Off their holds over a moment (at once, a foot moved
                    // 91 cm in a frame).
                    // Within the flight however short (a 0.2 s one caught
                    // them nine tenths of the way, and they jumped 10 cm).
                    let s = smoothstep(((t - SINK - DRIVE) / (FEET_LEAVE * plan.flight)).clamp(0.0, 1.0));
                    for limb in [LF, RF] {
                        let hanging = self.hips - Vec3::Y * FEET_BELOW + self.turn * self.rig.left() * (SIGN[limb % 2] * 0.12);
                        targets[limb] = (targets[limb].0.lerp(hanging, s), false);
                    }
                }
            }
            _ => {}
        }
        targets
    }

    /// The hips in the world now.
    fn hips_now(&self) -> Vec3 {
        match self.doing {
            Doing::GettingOn { t } => {
                let from = self.stood_at + self.turn * self.body.hips;
                from.lerp(self.hips, smoothstep((t / GET_ON).clamp(0.0, 1.0)))
            }
            _ => self.hips,
        }
    }

    /// The pose now, on the rig it was made on, in the walker's frame at
    /// [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self) -> LocalPose {
        // Taking the top's lip: from the climb's last pose into the hang's.
        if let Some(topping) = self.topping.as_ref() {
            let w = smoothstep((topping.t / TOP_BLEND).clamp(0.0, 1.0));
            let hang = topping.hang.pose(&self.rig);
            let mut pose = topping.from;
            for bone in Bone::ALL {
                pose.rotations[bone] = topping.from.rotations[bone].slerp(hang.rotations[bone], w);
            }
            pose.root_translation = topping.from.root_translation.lerp(hang.root_translation, w);
            return pose;
        }
        self.climbing_pose()
    }

    /// The climb's own pose ([`Self::pose`] but taking the lip).
    fn climbing_pose(&self) -> LocalPose {
        let rig = &self.rig;
        let hips = self.hips_now();
        let root = hips - self.turn * self.body.hips;
        let back = self.turn.inverse();
        let getting_on = match self.doing {
            Doing::GettingOn { t } => Some(smoothstep((t / GET_ON).clamp(0.0, 1.0))),
            _ => None,
        };
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, LEAN * getting_on.unwrap_or(1.0), (0.0, 0.0));
        let targets = self.targets();
        // The legs: each ankle to its ball's place; getting on, from where
        // it stood, no farther from its hip than its reach.
        for (side, &(_, _, ankle_bone, _)) in LEGS.iter().enumerate() {
            let (ball, _) = targets[LF + side];
            let (mut ankle, mut attitude) = self.ankle_for(side, ball);
            if let Some(w) = getting_on {
                let stood = self.stood_at + self.turn * self.body.ankles[side];
                ankle = stood.lerp(ankle, w);
                attitude = (self.turn * self.body.attitudes[side]).slerp(attitude, w);
            }
            let socket = hips + self.turn * self.body.sockets[side];
            let off = ankle - socket;
            let most = LEG_REACH * self.body.legs[side];
            if off.length() > most {
                ankle = socket + off * (most / off.length());
            }
            place_ankle(&mut pose, rig, ankle_bone, back * (ankle - root) - self.body.hips);
            // A knee near the wall turned out to its side, more the nearer
            // (a high step folded it forward 10 cm into the wall). Smoothly
            // by its clearance: turned as little as cleared it, a knee unable
            // to clear jumped to the full turn, 70 cm in a frame.
            let bones = [LEGS[side].0, LEGS[side].1, ankle_bone];
            let knee_out = self.wall.place(root + self.turn * forward_kinematics_on(&pose, rig)[bones[1]]).1;
            let turn_out = KNEE_MOST_OUT * smoothstep(((KNEE_CLEAR + KNEE_SOFT - knee_out) / (2.0 * KNEE_SOFT)).clamp(0.0, 1.0));
            if turn_out > 0.0 {
                crate::character::anim::stance::knee_toward(&mut pose, rig, bones, rig.left() * SIGN[side], turn_out);
            }
            let now = accumulate_world_rotations(&pose, rig)[ankle_bone];
            pose.rotations[ankle_bone] = delta_after_world_turn(&pose, rig, ankle_bone, (back * attitude) * now.inverse());
        }
        // The arms: each hand hooked over its hold; getting on, raised
        // forward round the shoulder to it.
        let at = forward_kinematics_on(&pose, rig);
        let wrists = [0, 1].map(|side| {
            let hooked = self.wrist_for(side, targets[side].0);
            match getting_on {
                Some(w) => {
                    let shoulder = at[ARMS[side].shoulder];
                    let (from, to) = (self.body.hands[side] - shoulder, back * (hooked - root) - shoulder);
                    let swept = Quat::IDENTITY.slerp(Quat::from_rotation_arc(from.normalize(), to.normalize()), w) * from.normalize();
                    let reach = from.length() + (to.length() - from.length()) * w - 0.12 * (std::f32::consts::PI * w).sin();
                    root + self.turn * (shoulder + swept * reach)
                }
                None => hooked,
            }
        });
        let targets_pose = wrists.map(|w| back * (w - root));
        for side in 0..2 {
            let lift = shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], targets_pose[side], 0.85 * self.body.arms[side]);
            pose.rotations[CLAVICLES[side]] = delta_after_world_turn(&pose, rig, CLAVICLES[side], lift);
        }
        let at = forward_kinematics_on(&pose, rig);
        let targets_pose = [0, 1].map(|side| {
            let (shoulder, arm) = (at[ARMS[side].shoulder], self.body.arms[side]);
            let off = targets_pose[side] - shoulder;
            let (knee, most) = (ARM_SOFT.0 * arm, ARM_SOFT.1 * arm);
            let length = off.length();
            if length <= knee {
                return targets_pose[side];
            }
            let soft = knee + (most - knee) * ((length - knee) / (most - knee)).tanh();
            shoulder + off * (soft / length)
        });
        for side in 0..2 {
            // The elbow out and back from the wall, a little down, as a
            // hang's: the hands kept over the shoulders (a hand moving up
            // past its shoulder flipped its elbow 29 cm in a frame). Mostly
            // down, an arm reaching up to the wall had its elbow's pole
            // turned toward the wall and the elbow went 5 cm into it. A hand
            // come down to its shoulder, pulled down ([`LOCK_OFF`]).
            let low = smoothstep(((at[ARMS[side].shoulder].y + LOCK_OFF.0 - targets_pose[side].y) / LOCK_OFF.1).clamp(0.0, 1.0));
            let pole = (rig.left() * (SIGN[side] * 0.7) - rig.forward() * 0.5 - Vec3::Y * (0.2 + 0.8 * low)).normalize();
            let (elbow, wrist) = solve_arm_toward_from(&mut pose, &at, ARMS[side], targets_pose[side], pole, rig);
            turn_hand(&mut pose, rig, ARMS[side], self.body.hand_binds[side], back * self.hook_turn(side), getting_on.unwrap_or(1.0), (wrist - elbow).normalize_or_zero());
        }
        if let Some(w) = getting_on {
            let s = smoothstep((w * GET_ON / GET_ON_EASE).clamp(0.0, 1.0));
            for bone in Bone::ALL {
                pose.rotations[bone] = self.body.stood.rotations[bone].slerp(pose.rotations[bone], s);
            }
        }
        // In a dyno's flight, the arms turn from how they held at the
        // release to how they hold at the catch (solved toward hands flying
        // on a path, the arm passed through straight and an elbow flipped
        // 50 cm in a frame).
        if let Doing::Dyno { to, t, plan } = &self.doing
            && *t >= SINK + DRIVE
        {
            let s = smoothstep(((t - SINK - DRIVE) / plan.flight).clamp(0.0, 1.0));
            let held_at = |hips: Vec3, limbs: [Option<usize>; 4]| {
                let mut held = self.clone();
                (held.doing, held.hips, held.limbs) = (Doing::Holding, hips, limbs);
                held.pose()
            };
            let mut caught = self.limbs;
            (caught[LH], caught[RH], caught[LF], caught[RF]) = (Some(*to), Some(*to), None, None);
            let (released, caught) = (held_at(plan.release, self.limbs), held_at(plan.catch, caught));
            for (chain, clavicle) in ARMS.iter().zip(CLAVICLES) {
                for bone in [clavicle, chain.shoulder, chain.elbow, chain.wrist] {
                    pose.rotations[bone] = released.rotations[bone].slerp(caught.rotations[bone], s);
                }
            }
            // Each hand kept off the wall, the arm turned out about its
            // shoulder: none at either end (the body rising under turning
            // arms carried a hand 5.6 cm into the wall).
            let clear = FLYING_OFF * (std::f32::consts::PI * s).sin();
            for chain in ARMS {
                let at = forward_kinematics_on(&pose, rig);
                let (shoulder, wrist) = (root + self.turn * at[chain.shoulder], root + self.turn * at[chain.wrist]);
                let short = clear - self.wall.place(wrist).1;
                let arm = wrist - shoulder;
                let axis = arm.cross(self.wall.out).normalize_or_zero();
                if short > 0.0 && axis != Vec3::ZERO {
                    // About `arm x out`, a positive turn takes the wrist out.
                    let turn = Quat::from_axis_angle(back * axis, short / arm.length().max(0.1));
                    pose.rotations[chain.shoulder] = delta_after_world_turn(&pose, rig, chain.shoulder, turn);
                }
            }
        }
        pose
    }

    /// [`Self::pose`], each bone led ahead of its spring (`jump::lead_of`).
    pub fn pose_led(&self, springs: &BoneSet<SpringParams>) -> LocalPose {
        let mut pose = self.pose();
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
                    let ahead = later.pose();
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }

    /// The walker's root now: under the hips as standing has them.
    pub fn root(&self) -> Vec3 {
        if let Some(topping) = self.topping.as_ref() {
            let w = smoothstep((topping.t / TOP_BLEND).clamp(0.0, 1.0));
            return topping.from_root.lerp(topping.hang.root(), w);
        }
        self.hips_now() - self.turn * self.body.hips
    }

    /// The walker's facing: toward the wall.
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// How closed each hand is (1 on its hold, 0 moving).
    pub fn grips(&self) -> [f32; 2] {
        let targets = self.targets();
        [targets[LH].1 as i32 as f32, targets[RH].1 as i32 as f32]
    }

    /// Each limb's hold now (left hand, right hand, left foot, right foot).
    pub fn holds(&self) -> [Option<usize>; 4] {
        self.limbs
    }

    /// Whether it is mid-move (a limb on its way, or a dyno).
    pub fn is_moving(&self) -> bool {
        matches!(self.doing, Doing::Moving { .. } | Doing::Dyno { .. })
    }

    /// Whether it is in a dyno's flight (no hand held).
    pub fn is_flying(&self) -> bool {
        matches!(&self.doing, Doing::Dyno { t, .. } if *t >= SINK + DRIVE)
    }

    /// Whether it has stepped off at the bottom, or taken the top's lip.
    pub fn stepped_off(&self) -> bool {
        self.doing == Doing::SteppedOff
    }

    pub fn topped_out(&self) -> bool {
        self.doing == Doing::ToppedOut
    }

    /// The wall it climbs.
    pub fn wall(&self) -> &HoldWall {
        &self.wall
    }

    /// Each wrist's and ball's place for a limb on its hold (the world):
    /// where a held limb is meant to be.
    pub fn held_places(&self) -> [Option<Vec3>; 4] {
        let targets = self.targets();
        [0, 1, 2, 3].map(|limb| {
            let (at, held) = targets[limb];
            held.then(|| if limb < LF { self.wrist_for(limb, at) } else { self.ankle_for(limb - LF, at).0 })
        })
    }

    /// Stepped off: the fall from the pose now onto the floor.
    pub fn step_off(&self, stood: &LocalPose) -> Falling {
        Falling::off(self.root(), self.yaw, Vec3::ZERO, &self.pose(), self.floor, 0.0, stood, &self.rig)
    }

    /// Topped out: the hang on the top's lip it has blended into, to go on
    /// from (`others` about it, for shimmying and leaps).
    pub fn top_out(&self, others: &[Ledge]) -> Option<Hanging> {
        let mut hang = self.topping.as_ref()?.hang.clone();
        hang.set_others(others);
        Some(hang)
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

    /// What a climb measured.
    #[derive(Debug, Default)]
    struct Climbed {
        /// The most a held limb strays from its place (wrist, ankle).
        hand_off: f32,
        foot_off: f32,
        /// The deepest a joint goes into the wall; the fastest about the
        /// hips; the most a joint's step changes in a frame.
        into: f32,
        fastest: f32,
        kink: f32,
        /// The fewest limbs held outside a dyno's flight; the longest a
        /// dyno's flight holds none, seconds.
        fewest: usize,
        flight: f32,
        moves: usize,
        dynos: usize,
    }

    fn world(climb: &FreeClimb) -> BoneSet<Vec3> {
        let (pose, root, turn) = (climb.pose(), climb.root(), Quat::from_rotation_y(climb.facing()));
        let at = forward_kinematics_on(&pose, &climb.rig);
        BoneSet::from_fn(|bone| root + turn * at[bone])
    }

    /// Climbs `climb` asked `way` for `seconds`, measuring.
    fn run(climb: &mut FreeClimb, way: Option<ClimbWay>, seconds: f32, m: &mut Climbed, frames: &mut Vec<BoneSet<Vec3>>) {
        let mut t = 0.0;
        let mut flying = 0.0;
        while t < seconds && !climb.stepped_off() && !climb.topped_out() {
            let was_moving = climb.is_moving();
            climb.advance(way, DT);
            t += DT;
            if climb.is_moving() && !was_moving {
                m.moves += 1;
            }
            if climb.is_flying() {
                if flying == 0.0 {
                    m.dynos += 1;
                }
                flying += DT;
                m.flight = m.flight.max(flying);
            } else {
                flying = 0.0;
            }
            let now = world(climb);
            let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
            m.fastest = m.fastest.max(Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (b[bone] - b[Bone::Hips])).length() / DT).fold(0.0, f32::max));
            m.kink = m.kink.max(Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(0.0, f32::max));
            let places = climb.held_places();
            if !matches!(climb.doing, Doing::GettingOn { .. }) {
                // A hand moving, the other three held.
                if matches!(climb.doing, Doing::Moving { limb, .. } if limb < LF) {
                    m.fewest = m.fewest.min(places.iter().filter(|p| p.is_some()).count());
                }
                for (side, chain) in ARMS.iter().enumerate() {
                    if let Some(wrist) = places[side] {
                        m.hand_off = m.hand_off.max((now[chain.wrist] - wrist).length());
                    }
                }
                for (side, &(_, _, ankle, _)) in LEGS.iter().enumerate() {
                    if let Some(at) = places[LF + side] {
                        m.foot_off = m.foot_off.max((now[ankle] - at).length());
                    }
                }
            }
            for bone in Bone::ALL {
                m.into = m.into.max(-climb.wall.place(now[bone]).1);
            }
            frames.push(now);
        }
    }

    /// A 6 m wall of edges 0.4 m across and 0.3 m up: got on from standing,
    /// climbed up 2 m, aside each way, diagonally and down: held hands on
    /// their holds, feet on theirs, three limbs held while one moves,
    /// nothing into the wall, no joint whipping round nor jumping, and it
    /// gets where it was asked.
    #[test]
    fn a_wall_of_holds_is_climbed_limb_by_limb_every_way() {
        let (stood, rig) = real_stood();
        let wall = HoldWall::grid(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 9, 18, 0.4, 0.3, 0.35);
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        let mut m = Climbed { fewest: 4, ..Default::default() };
        let mut frames = vec![world(&climb), world(&climb)];
        run(&mut climb, None, 1.5, &mut m, &mut frames);
        let start = climb.hips;
        run(&mut climb, Some(Vec2::Y), 9.0, &mut m, &mut frames);
        let up = climb.hips.y - start.y;
        let mid = climb.hips;
        run(&mut climb, Some(Vec2::X), 6.0, &mut m, &mut frames);
        let left = wall.place(climb.hips).0.x - wall.place(mid).0.x;
        let mid = climb.hips;
        run(&mut climb, Some(-Vec2::X), 6.0, &mut m, &mut frames);
        let right = wall.place(mid).0.x - wall.place(climb.hips).0.x;
        let mid = climb.hips;
        run(&mut climb, Some(Vec2::new(1.0, 1.0)), 5.0, &mut m, &mut frames);
        let diagonal = climb.hips - mid;
        let mid = climb.hips;
        run(&mut climb, Some(-Vec2::Y), 9.0, &mut m, &mut frames);
        let down = mid.y - climb.hips.y;
        eprintln!("{m:?}; up {up:.2}, left {left:.2}, right {right:.2}, diagonal {diagonal:.2}, down {down:.2}");
        assert!(up > 1.2, "climbed up only {up:.2} m in 9 s");
        assert!(left > 0.8 && right > 0.8, "aside only {left:.2} and {right:.2} m in 6 s");
        assert!(diagonal.y > 0.4 && wall.place(diagonal + wall.face).0.x > 0.3, "diagonally only {diagonal:.2}");
        assert!(down > 1.2, "climbed down only {down:.2} m in 9 s");
        assert!(m.hand_off < 1.0e-3, "a held hand {:.4} m off its hold", m.hand_off);
        assert!(m.foot_off < 0.01, "a held foot {:.4} m off its hold", m.foot_off);
        assert!(m.fewest >= 3, "only {} limbs held", m.fewest);
        assert!(m.into < 0.005, "a joint {:.4} m into the wall", m.into);
        assert!(m.fastest < 14.0, "a joint at {:.1} m/s about the hips", m.fastest);
        assert!(m.kink < 0.03, "a joint's step changed {:.4} m in a frame", m.kink);
    }

    /// A gap of 1.1 m in the hand holds above, footholds going on through
    /// it: jumped (a dyno), caught with both hands, the feet finding holds
    /// after; never no hand held for under 0.15 s nor over 0.4.
    #[test]
    fn a_hold_out_of_reach_is_jumped_for() {
        let (stood, rig) = real_stood();
        let mut wall = HoldWall::grid(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 7, 9, 0.4, 0.3, 0.35);
        let top = wall.holds.iter().map(|h| h.at.y).fold(f32::MIN, f32::max);
        // A row 1.1 m over the top row, and more above it; footholds in the
        // gap.
        for row in 0..4 {
            for column in -2..=2 {
                wall.holds.push(Hold { at: Vec3::new(column as f32 * 0.4, top + 1.1 + row as f32 * 0.3, -0.6), kind: HoldKind::Edge });
            }
        }
        for row in 1..4 {
            for column in -2..=2 {
                wall.holds.push(Hold { at: Vec3::new(column as f32 * 0.4 + 0.2, top + row as f32 * 0.3, -0.6), kind: HoldKind::Foot });
            }
        }
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        let mut m = Climbed { fewest: 4, ..Default::default() };
        let mut frames = vec![world(&climb), world(&climb)];
        run(&mut climb, Some(Vec2::Y), 14.0, &mut m, &mut frames);
        eprintln!("{m:?}, hips at {:.2}", climb.hips.y);
        assert!(m.dynos >= 1, "no dyno");
        assert!(climb.hips.y > top + 0.3, "stuck under the gap, the hips at {:.2}", climb.hips.y);
        assert!((0.15..0.4).contains(&m.flight), "no hand held for {:.2} s", m.flight);
        assert!(m.hand_off < 1.0e-3 && m.into < 0.005 && m.fastest < 14.0, "{m:?}");
    }

    /// Climbed down to the floor it steps off and lands; up to the top's lip
    /// it takes it into a hang, the pose continuous.
    #[test]
    fn it_steps_off_at_the_bottom_and_tops_out_into_a_hang() {
        let (stood, rig) = real_stood();
        let mut wall = HoldWall::grid(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 7, 8, 0.4, 0.3, 0.35);
        let top = wall.holds.iter().map(|h| h.at.y).fold(f32::MIN, f32::max) + 0.3;
        wall.top = Some(Ledge::wall(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 4.0, top, 1.0));
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        let mut m = Climbed { fewest: 4, ..Default::default() };
        let mut frames = vec![world(&climb), world(&climb)];
        run(&mut climb, Some(-Vec2::Y), 6.0, &mut m, &mut frames);
        assert!(climb.stepped_off(), "never stepped off");
        let mut falling = climb.step_off(&stood);
        for _ in 0..(3.0 / DT) as usize {
            falling.advance(DT);
        }
        assert!(falling.is_done(), "never landed");
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        let mut frames = vec![world(&climb), world(&climb)];
        run(&mut climb, Some(Vec2::Y), 20.0, &mut m, &mut frames);
        assert!(climb.topped_out(), "never topped out, the hands at {:?}", climb.limbs);
        let held = *frames.last().unwrap();
        let mut hanging = climb.top_out(&[]).expect("a hang");
        hanging.advance(DT);
        let at = forward_kinematics_on(&hanging.pose(&rig), &rig);
        let first = BoneSet::from_fn(|bone| hanging.root() + Quat::from_rotation_y(hanging.facing()) * at[bone]);
        let jump = Bone::ALL.iter().map(|&bone| (first[bone] - held[bone]).length()).fold(0.0, f32::max);
        eprintln!("{m:?}, topping out a joint moved {jump:.3}");
        assert!(jump < 0.05, "taking the lip, a joint moved {jump:.3} m in a frame");
    }
}
