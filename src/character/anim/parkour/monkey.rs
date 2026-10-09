//! Monkey bars: step 17 of the parkour steps beyond the first ten
//! (swinging on fixtures). A line of bars overhead, crossed hand over hand:
//! jumped up to from under the first, both hands on it; then each hand in
//! turn swung on past the other to the bar after it, the body swinging
//! under the holding hand and twisting toward the leading one; at the last
//! bar the hands match, the swing comes to rest, and it lets go and lands.
//!
//! No monkey-bar data: the pace is a hand move every 0.9 s (a free hang's
//! pendulum is about 2.4 s, and a brachiating step about a third of it),
//! set by eye.

use bevy::math::{Quat, Vec3};

use super::Falling;
use crate::character::anim::armik::{frame_turn, shoulder_lift, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::{hermite, smoothstep};
use crate::character::anim::hand::{HandGrip, FINGER_HALF_THICKNESS, GRIP_RADIUS};
use crate::character::anim::jump::lead_of;
use crate::character::anim::math::SpringParams;
use crate::character::anim::rig::{accumulate_bind_rotations, accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// A bar's radius, metres.
pub const BAR_RADIUS: f32 = 0.02;

/// A hand move's time, seconds, and the share of it at its start both
/// hands hold.
pub const STEP: f32 = 0.9;
const BOTH: f32 = 0.25;
/// How much slower than its mean the body goes with both hands holding
/// (and faster under the holding hand), of the mean: a pendulum's swing.
const SWAY: f32 = 0.5;
/// How far the hands reach from their (lifted) shoulders, of the arm, and
/// the farthest an arm is solved to.
const REACH: f32 = 0.94;
const ARM_REACH: f32 = 0.98;
/// How far the body twists toward the leading hand as it catches, radians.
const TWIST: f32 = 0.35;
/// How far a hand dips under the bars on its way, metres.
const HAND_DIP: f32 = 0.08;
/// The legs hang this share of their length along the body's line, piked
/// ahead this far, radians.
const LEG: f32 = 0.97;
const PIKE: f32 = 0.15;
/// Getting on: how long, seconds, by when (of it) the hands are on the
/// first bar, and how long the standing pose eases out over. The hands
/// swing from hanging to overhead: by 0.48 s, a wrist's step changed
/// 2.2 cm a frame round the shoulder.
const GET_ON: f32 = 0.8;
const HANDS_ON: f32 = 0.75;
const GET_ON_EASE: f32 = 0.2;
/// Swept up round the shoulder, how far a hand's way bends in, metres.
const SWEEP_BEND: f32 = 0.12;
/// At the last bar, how long the hands' match and the swing's rest take,
/// seconds; and how long it hangs still before letting go.
const SETTLE: f32 = STEP;
const HOLD: f32 = 0.4;
/// How a hand's grip opens off its bar and closes on the next: the first
/// and last share of its move. Let go at once, the point the body hangs
/// from jumped and a toe's step changed 18 cm in a frame.
const OPENING: f32 = 0.2;
const CLOSING: f32 = 0.2;

/// A quintic ease, 0 to 1 with no speed nor acceleration at either end:
/// a hand's hold eased by a cubic stepped a toe 2.3 cm in a frame as it
/// finished opening.
fn ease(u: f32) -> f32 {
    let u = u.clamp(0.0, 1.0);
    u * u * u * (u * (6.0 * u - 15.0) + 10.0)
}

const LEGS: [(Bone, Bone, Bone, Bone); 2] =
    [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase)];
const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
const SIGN: [f32; 2] = [1.0, -1.0];
const GUESSED_KNUCKLES: f32 = 0.08;

/// A line of monkey bars: the first bar's middle (on its axis, the world),
/// the way along the line (level), how far apart the bars are, metres, and
/// how many.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MonkeyBars {
    pub first: Vec3,
    pub way: Vec3,
    pub spacing: f32,
    pub bars: usize,
}

impl MonkeyBars {
    /// `bars` bars `spacing` apart from `first` along `way`.
    pub fn new(first: Vec3, way: Vec3, spacing: f32, bars: usize) -> Self {
        Self { first, way: Vec3::new(way.x, 0.0, way.z).normalize_or(Vec3::NEG_Z), spacing, bars: bars.max(2) }
    }

    /// Bar `k`'s middle (`k` may be fractional, along the line).
    pub fn bar(&self, k: f32) -> Vec3 {
        self.first + self.way * (self.spacing * k)
    }

    /// The bars' axis, to the left looking along the line.
    pub fn across(&self) -> Vec3 {
        Vec3::Y.cross(self.way).normalize()
    }

    /// The last bar's index.
    pub fn last(&self) -> usize {
        self.bars - 1
    }
}

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    hips: Vec3,
    arms: [f32; 2],
    legs: [f32; 2],
    shoulders: [Vec3; 2],
    sockets: [Vec3; 2],
    hands: [Vec3; 2],
    ankles: [Vec3; 2],
    attitudes: [Quat; 2],
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
            legs: LEGS.map(|(socket, knee, ankle, _)| (at[knee] - at[socket]).length() + (at[ankle] - at[knee]).length()),
            shoulders: ARMS.map(|arm| at[arm.shoulder]),
            sockets: LEGS.map(|(socket, _, _, _)| at[socket]),
            hands: ARMS.map(|arm| at[arm.wrist]),
            ankles: LEGS.map(|(_, _, ankle, _)| at[ankle]),
            attitudes: LEGS.map(|(_, _, ankle, _)| world[ankle]),
            hand_binds: ARMS.map(|arm| binds[arm.wrist]),
            grips: ARMS.map(|arm| {
                let along = (rest[arm.wrist] - rest[arm.elbow]).normalize_or(Vec3::NEG_Y);
                let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
                HandGrip { bar: along * GUESSED_KNUCKLES + palm * (GRIP_RADIUS + FINGER_HALF_THICKNESS), palm, along }
            }),
        }
    }

    /// How far the hips hang under the bars on one hand, metres: the
    /// shoulders over the hips, the arm reaching [`REACH`] of its length,
    /// the wrist under the bar's axis.
    fn hang(&self) -> f32 {
        let shoulders = 0.5 * (self.shoulders[0] + self.shoulders[1]) - self.hips;
        shoulders.y + REACH * 0.5 * (self.arms[0] + self.arms[1]) + self.grips[0].bar.length()
    }

    /// Half the shoulders' width, metres.
    fn half_width(&self) -> f32 {
        0.5 * (self.shoulders[0] - self.shoulders[1]).length()
    }
}

/// Where it is on the bars.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Jumping up to the first bar, `t` seconds in.
    GettingOn { t: f32 },
    /// Crossing, `t` seconds in (hand moves, the last hands matching, the
    /// swing come to rest), and hanging still after.
    Crossing { t: f32 },
    /// Let go at the far end: handed to a fall ([`Crossing::release`]).
    Released,
}

/// Crossing monkey bars ([`MonkeyBars`]).
#[derive(Debug, Clone)]
pub struct Crossing {
    bars: MonkeyBars,
    body: Body,
    /// The facing turn (along the line) and its heading for the walker.
    turn: Quat,
    yaw: f32,
    /// Where it stood to get on (its root).
    stood_at: Vec3,
    phase: Phase,
}

/// One hand's move: from bar `from` to bar `to`, over `start..end` seconds
/// into the crossing.
#[derive(Debug, Clone, Copy)]
struct Move {
    side: usize,
    from: usize,
    to: usize,
    start: f32,
    end: f32,
}

impl Crossing {
    /// Where the root stands on a floor `floor` high to get on `bars`: the
    /// hips under the first bar, facing along the line.
    pub fn spot(bars: &MonkeyBars, floor: f32, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
        let turn = Self::facing_turn(bars, rig);
        let hips = forward_kinematics_on(stood, rig)[Bone::Hips];
        (bars.first - turn * hips).with_y(floor)
    }

    /// The heading a walker faces along `bars`.
    pub fn heading(bars: &MonkeyBars, rig: &RigGeometry) -> f32 {
        crate::character::anim::approach::heading_of(bars.way) - crate::character::anim::approach::heading_of(rig.forward())
    }

    fn facing_turn(bars: &MonkeyBars, rig: &RigGeometry) -> Quat {
        Quat::from_rotation_y(crate::character::anim::approach::heading_of(bars.way) - crate::character::anim::approach::heading_of(rig.forward()))
    }

    /// Getting on `bars` from standing with the root at `root`.
    pub fn get_on(bars: &MonkeyBars, root: Vec3, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let turn = Self::facing_turn(bars, rig);
        Self {
            bars: *bars,
            body: Body::of(stood, rig),
            turn,
            yaw: Self::heading(bars, rig),
            stood_at: root,
            phase: Phase::GettingOn { t: 0.0 },
        }
    }

    /// Each hand placed so its own fingers close round a bar: their grips
    /// as the rig's fingers make them (in each hand's own frame, on `rig`).
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2], rig: &RigGeometry) {
        for (side, grip) in crate::character::anim::hand::bound_grips(grips, rig).into_iter().enumerate() {
            if let Some(grip) = grip {
                self.body.grips[side] = grip;
            }
        }
    }

    /// The hand moves across: the right hand to bar 1, then each hand in
    /// turn two bars on, until one reaches the last; then the other matches
    /// it there.
    fn moves(&self) -> Vec<Move> {
        let last = self.bars.last();
        let mut moves = Vec::new();
        let mut at = [0usize; 2];
        let mut side = 1;
        let mut start = 0.0;
        loop {
            let to = (at[1 - side] + 1).min(last);
            if to <= at[side] && at[side] == last {
                break;
            }
            moves.push(Move { side, from: at[side], to, start, end: start + STEP });
            at[side] = to;
            start += STEP;
            if at == [last, last] {
                break;
            }
            side = 1 - side;
        }
        moves
    }

    /// How long the crossing takes, to the swing come to rest under the
    /// last bar, seconds.
    fn crossing_time(&self) -> f32 {
        self.moves().last().map_or(0.0, |last| last.end) + SETTLE
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.phase = match self.phase {
            Phase::GettingOn { t } if t + dt >= GET_ON => Phase::Crossing { t: t + dt - GET_ON },
            Phase::GettingOn { t } => Phase::GettingOn { t: t + dt },
            Phase::Crossing { t } if t + dt >= self.crossing_time() + HOLD => Phase::Released,
            Phase::Crossing { t } => Phase::Crossing { t: t + dt },
            Phase::Released => Phase::Released,
        };
    }

    /// How much of the body each hand carries `t` seconds into the
    /// crossing (0-1): a moving hand's share eased out to none half way
    /// through its move (the hips then under the other hand's bar) and back
    /// by its end. By its grip alone (eased in 0.14 s), the point the body
    /// hangs from swung the legs' line 0.2 rad and stepped a foot 2.6 cm in
    /// a frame.
    fn carried_at(&self, t: f32) -> [f32; 2] {
        let mut carried = [1.0; 2];
        for m in self.moves() {
            if t > m.start && t < m.end {
                let half = 0.5 * (m.end - m.start);
                let u = (t - m.start) / half;
                carried[m.side] = if u < 1.0 { 1.0 - ease(u) } else { ease(u - 1.0) };
            }
        }
        carried
    }

    /// Where each hand is `t` seconds into the crossing: its bar (as a
    /// fractional place along the line), how far it dips under the bars,
    /// and how closed it is (0 open, 1 holding).
    fn hands_at(&self, t: f32) -> [(f32, f32, f32); 2] {
        let mut hands = [(0.0, 0.0, 1.0); 2];
        for m in self.moves() {
            if t >= m.end {
                hands[m.side] = (m.to as f32, 0.0, 1.0);
            } else if t > m.start {
                let going = m.start + BOTH * STEP;
                let u = ((t - going) / (m.end - going)).clamp(0.0, 1.0);
                let s = smoothstep(u);
                let place = m.from as f32 + (m.to as f32 - m.from as f32) * s;
                let opened = ease(u / OPENING);
                let closed = 1.0 - opened + ease((u - (1.0 - CLOSING)) / CLOSING);
                hands[m.side] = (place, HAND_DIP * (std::f32::consts::PI * u).sin().powi(2), closed.clamp(0.0, 1.0));
            }
        }
        hands
    }

    /// The hips' place along the line `t` seconds into the crossing (in
    /// bars): from under the first, through the middle between the hands
    /// at each move's end (slow there, fast under the holding hand), to
    /// under the last; and how far each move's end is past the last's.
    fn along_at(&self, t: f32) -> f32 {
        let moves = self.moves();
        let mean = 1.0 / STEP;
        let between = mean * (1.0 - SWAY);
        // The knots: under the first at rest; each move's end between the
        // hands; under the last at rest.
        let mut knots = vec![(0.0f32, 0.0f32, 0.0f32)];
        let mut at = [0usize; 2];
        for m in &moves {
            at[m.side] = m.to;
            let middle = 0.5 * (at[0] + at[1]) as f32;
            knots.push((m.end, middle, if at[0] == at[1] { 0.0 } else { between }));
        }
        let end = knots.last().map_or(0.0, |k| k.0) + SETTLE;
        knots.push((end, self.bars.last() as f32, 0.0));
        // A matched last pair ends at rest already: the settle holds.
        let t = t.clamp(0.0, end);
        let i = knots.windows(2).position(|w| t <= w[1].0).unwrap_or(knots.len() - 2);
        let (a, b) = (knots[i], knots[i + 1]);
        let span = (b.0 - a.0).max(1.0e-6);
        hermite(a.1, b.1, a.2 * span, b.2 * span, ((t - a.0) / span).clamp(0.0, 1.0))
    }

    /// How far the body is twisted toward the leading hand `t` seconds into
    /// the crossing, radians about `+Y` (positive turns the left shoulder
    /// back): most as each hand catches, through none between.
    fn twist_at(&self, t: f32) -> f32 {
        let moves = self.moves();
        let last = self.bars.last();
        // Each catch's twist, the leading hand's shoulder forward; none
        // with the hands matched.
        let mut knots = vec![(0.0f32, 0.0f32)];
        let mut at = [0usize; 2];
        for m in &moves {
            at[m.side] = m.to;
            let twist = if at[0] == at[1] { 0.0 } else if at[0] > at[1] { -TWIST } else { TWIST };
            knots.push((m.end, twist));
        }
        let _ = last;
        let t = t.max(0.0);
        let i = knots.windows(2).position(|w| t <= w[1].0);
        match i {
            Some(i) => {
                let (a, b) = (knots[i], knots[i + 1]);
                a.1 + (b.1 - a.1) * smoothstep(((t - a.0) / (b.0 - a.0).max(1.0e-6)).clamp(0.0, 1.0))
            }
            None => knots.last().map_or(0.0, |k| k.1),
        }
    }

    /// Each hand's grip point on its bar (the world) for a place along the
    /// line `place` (in bars), dipped `dip` under the bars.
    fn grip_point(&self, side: usize, place: f32, dip: f32) -> Vec3 {
        self.bars.bar(place) + self.bars.across() * (SIGN[side] * self.body.half_width()) - Vec3::Y * dip
    }

    /// A hand round its grip point `point`, its forearm coming from `from`
    /// (the world): its turn (the fingers on along that line, over the
    /// bar; the palm forward) and its wrist (the world).
    fn hand_at(&self, side: usize, point: Vec3, from: Vec3) -> (Quat, Vec3) {
        let grip = &self.body.grips[side];
        let along = (point - from).normalize_or(Vec3::Y);
        let forward = self.bars.way;
        let palm = (forward - along * forward.dot(along)).normalize_or(forward);
        let hand = frame_turn(grip.along, grip.palm, along, palm);
        (hand, point - hand * grip.bar)
    }

    /// The hips in the world now, and the point the body hangs from.
    fn hips(&self) -> (Vec3, Vec3) {
        let hang = self.body.hang();
        match self.phase {
            // Up as the hands reach the bar: rising over the whole of
            // getting on, the hips were 10 cm under the hang as the hands
            // met the bar, the arms clamped near straight, and an elbow
            // swung 2 cm in a frame.
            Phase::GettingOn { t } => {
                let s = ease(t / (GET_ON * HANDS_ON));
                let from = self.stood_at + self.turn * self.body.hips;
                let on = self.bars.bar(0.0) - Vec3::Y * hang;
                (from.lerp(on, s), self.bars.bar(0.0))
            }
            _ => {
                let t = self.crossing_t();
                let hands = self.hands_at(t);
                // Hung from the hands by how much each carries.
                let weights = self.carried_at(t).map(|w| w.max(1.0e-3));
                let pivot = (self.bars.bar(hands[0].0) * weights[0] + self.bars.bar(hands[1].0) * weights[1]) / (weights[0] + weights[1]);
                let along = self.along_at(t);
                let level = self.bars.bar(along).with_y(0.0) + Vec3::Y * pivot.y;
                // The hands apart along the line, both holding: the
                // shoulders higher, so each arm reaches.
                let apart = (hands[0].0 - hands[1].0).abs() * self.bars.spacing * weights[0].min(weights[1]);
                let offset = (0.5 * apart - self.body.half_width() * self.twist_at(t).abs().sin()).max(0.0);
                let reach = REACH * 0.5 * (self.body.arms[0] + self.body.arms[1]);
                let rise = reach - (reach * reach - offset * offset).max(0.0).sqrt();
                let d = (level - pivot.with_y(level.y)).length();
                let length = hang - rise;
                (level - Vec3::Y * (length * length - d * d).max(0.0).sqrt(), pivot)
            }
        }
    }

    /// Seconds into the crossing (0 getting on).
    fn crossing_t(&self) -> f32 {
        match self.phase {
            Phase::Crossing { t } => t,
            Phase::Released => self.crossing_time() + HOLD,
            Phase::GettingOn { .. } => 0.0,
        }
    }

    /// The pose now, on `rig`, in the walker's pose frame at
    /// [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self, rig: &RigGeometry) -> LocalPose {
        self.posed(rig).0
    }

    /// [`Self::pose`], and where it put each wrist (the world).
    fn posed(&self, rig: &RigGeometry) -> (LocalPose, [Vec3; 2]) {
        let (hips, pivot) = self.hips();
        let turn = self.turn;
        let root = hips - turn * self.body.hips;
        let back = turn.inverse();
        let forward = self.bars.way;
        let getting_on = match self.phase {
            Phase::GettingOn { t } => Some(t),
            _ => None,
        };
        let t = self.crossing_t();
        // The trunk turned whole toward the point it hangs from, twisted
        // toward the leading hand.
        let to = pivot - hips;
        let lean = to.dot(forward).atan2(to.y) * getting_on.map_or(1.0, |t| smoothstep((t / GET_ON).clamp(0.0, 1.0)));
        let twist = if getting_on.is_some() { 0.0 } else { self.twist_at(t) };
        let mut pose = self.body.stood;
        let left = rig.left();
        pose.rotations[Bone::Hips] = delta_after_world_turn(&pose, rig, Bone::Hips, Quat::from_axis_angle(left, lean) * Quat::from_rotation_y(twist));
        pose.rotations[Bone::Neck] = delta_after_world_turn(&pose, rig, Bone::Neck, Quat::from_axis_angle(left, -0.6 * lean));
        // The legs along the body's line from where it hangs, piked ahead.
        let down = (-to).normalize_or(Vec3::NEG_Y);
        let ahead = (forward - down * forward.dot(down)).normalize_or(forward);
        let legs = down * PIKE.cos() + ahead * PIKE.sin();
        for (side, &(_, _, ankle_bone, _)) in LEGS.iter().enumerate() {
            let socket = hips + turn * (self.body.sockets[side] - self.body.hips);
            let hanging = socket + legs * (LEG * self.body.legs[side]);
            let target = match getting_on {
                Some(t) => {
                    let s = smoothstep((t / GET_ON).clamp(0.0, 1.0));
                    let stood = self.stood_at + turn * self.body.ankles[side];
                    let target = stood.lerp(hanging, s);
                    let most = LEG * self.body.legs[side];
                    let off = target - socket;
                    if off.length() > most { socket + off * (most / off.length()) } else { target }
                }
                None => hanging,
            };
            place_ankle(&mut pose, rig, ankle_bone, back * (target - root) - self.body.hips);
            // The feet hanging, pointed a little along the legs.
            let standing = turn * self.body.attitudes[side];
            let now = accumulate_world_rotations(&pose, rig)[ankle_bone];
            pose.rotations[ankle_bone] = delta_after_world_turn(&pose, rig, ankle_bone, (back * standing) * now.inverse());
        }
        // The arms: each hand round its bar (or on its way), the fingers on
        // along its forearm over the bar, the palm forward. The forearm's
        // line first from the shoulder as standing carries it, then from
        // the elbow the arm solved to that puts there: from the shoulder
        // alone, the elbow out and back bent the wrist 0.68 rad.
        let hands = self.hands_at(t);
        let points = [0, 1].map(|side| self.grip_point(side, hands[side].0, hands[side].1));
        let mut froms = [0, 1].map(|side| hips + turn * (self.body.shoulders[side] - self.body.hips));
        let unarmed = pose;
        let mut wrists = [Vec3::ZERO; 2];
        for pass in 0..2 {
            pose = unarmed;
            let turns;
            (wrists, turns) = self.hands_toward(&points, &froms, getting_on, root, rig);
            let weight = getting_on.map_or(1.0, |t| ease(t / (GET_ON * HANDS_ON)));
            self.arms_to(&mut pose, rig, root, back, wrists, turns, weight);
            if pass == 0 {
                let at = forward_kinematics_on(&pose, rig);
                froms = [0, 1].map(|side| root + turn * at[ARMS[side].elbow]);
            }
        }
        if let Some(t) = getting_on {
            let s = ease(t / GET_ON_EASE);
            for bone in Bone::ALL {
                pose.rotations[bone] = self.body.stood.rotations[bone].slerp(pose.rotations[bone], s);
            }
        }
        (pose, wrists)
    }

    /// Each wrist's target and hand turn (the world), its hand round
    /// `points` from forearms coming from `froms`; getting on, swept up to
    /// it.
    fn hands_toward(&self, points: &[Vec3; 2], froms: &[Vec3; 2], getting_on: Option<f32>, root: Vec3, rig: &RigGeometry) -> ([Vec3; 2], [Quat; 2]) {
        let (turn, back) = (self.turn, self.turn.inverse());
        let mut wrists = [Vec3::ZERO; 2];
        let mut turns = [Quat::IDENTITY; 2];
        for side in 0..2 {
            let (hand, on) = self.hand_at(side, points[side], froms[side]);
            wrists[side] = match getting_on {
                // Swept up through the front about the shoulder, then the
                // small turn onto its hold: from hanging to overhead is near
                // half a turn, whose shortest arc has no steady axis (an
                // elbow stepped 2.7 cm in a frame).
                Some(t) => {
                    let s = smoothstep((t / (GET_ON * HANDS_ON)).clamp(0.0, 1.0));
                    let from_shoulder = self.body.shoulders[side];
                    let (from, to) = (self.body.hands[side] - from_shoulder, back * (on - root) - from_shoulder);
                    let (f, g) = (from.normalize(), to.normalize());
                    let axis = f.cross(rig.forward()).normalize_or(rig.left());
                    let level = (g - axis * g.dot(axis)).normalize_or(g);
                    let theta = axis.dot(f.cross(level)).atan2(f.dot(level)).rem_euclid(std::f32::consts::TAU);
                    let fix = Quat::from_rotation_arc(Quat::from_axis_angle(axis, theta) * f, g);
                    let swept = Quat::IDENTITY.slerp(fix, s) * (Quat::from_axis_angle(axis, theta * s) * f);
                    let reach = from.length() + (to.length() - from.length()) * s - SWEEP_BEND * (std::f32::consts::PI * s).sin();
                    root + turn * (from_shoulder + swept * reach)
                }
                None => on,
            };
            turns[side] = hand;
        }
        (wrists, turns)
    }

    /// Each arm to its wrist (the world), the hand turned to `turns` by
    /// `weight`: the shoulder lifted toward it, the elbow out and back.
    #[allow(clippy::too_many_arguments)]
    fn arms_to(&self, pose: &mut LocalPose, rig: &RigGeometry, root: Vec3, back: Quat, wrists: [Vec3; 2], turns: [Quat; 2], weight: f32) {
        let targets = wrists.map(|p| back * (p - root));
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            // Eased in as the hands go up (getting on): the lift's limit
            // changes from reaching forward to up, and as the swept hands
            // passed between them a shoulder stepped 2.3 cm in a frame.
            let lift = Quat::IDENTITY.slerp(shoulder_lift(at[CLAVICLES[side]], at[ARMS[side].shoulder], targets[side], 0.85 * self.body.arms[side]), weight);
            pose.rotations[CLAVICLES[side]] = delta_after_world_turn(pose, rig, CLAVICLES[side], lift);
        }
        let at = forward_kinematics_on(pose, rig);
        for side in 0..2 {
            let chain = ARMS[side];
            let pole = (rig.left() * (SIGN[side] * 0.7) - rig.forward() * 0.5).normalize();
            let off = targets[side] - at[chain.shoulder];
            let most = ARM_REACH * self.body.arms[side];
            let target = if off.length() > most { at[chain.shoulder] + off * (most / off.length()) } else { targets[side] };
            let (elbow, wrist) = solve_arm_toward_from(pose, &at, chain, target, pole, rig);
            turn_hand(pose, rig, chain, self.body.hand_binds[side], back * turns[side], weight, (wrist - elbow).normalize_or_zero());
        }
    }

    /// [`Self::pose`], each bone led ahead of its spring.
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
                    later.advance(lead);
                    let ahead = later.pose(rig);
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }

    /// Where the walker's root is now.
    pub fn root(&self) -> Vec3 {
        self.hips().0 - self.turn * self.body.hips
    }

    /// The walker's facing now: along the line.
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// Where it looks: the bar ahead of the leading hand.
    pub fn look(&self) -> Vec3 {
        let hands = self.hands_at(self.crossing_t());
        self.bars.bar(hands[0].0.max(hands[1].0) + 1.0)
    }

    /// How closed each hand is round its bar (1 holding, 0 open).
    pub fn grips(&self) -> [f32; 2] {
        match self.phase {
            Phase::GettingOn { t } => [smoothstep(((t / (GET_ON * HANDS_ON) - 0.8) / 0.2).clamp(0.0, 1.0)); 2],
            Phase::Released => [0.0; 2],
            Phase::Crossing { t } => self.hands_at(t).map(|(_, _, closed)| closed),
        }
    }

    /// Each wrist's place on its bar now (the world), and whether it holds
    /// it there, posed on `rig`.
    pub fn wrists(&self, rig: &RigGeometry) -> [(Vec3, bool); 2] {
        let hands = self.hands_at(self.crossing_t());
        let (_, wrists) = self.posed(rig);
        [0, 1].map(|side| {
            let (_, dip, closed) = hands[side];
            (wrists[side], closed >= 1.0 && dip == 0.0 && matches!(self.phase, Phase::Crossing { .. }))
        })
    }

    /// Whether it has let go at the far end.
    pub fn is_released(&self) -> bool {
        self.phase == Phase::Released
    }

    /// The bars it is on.
    pub fn bars(&self) -> &MonkeyBars {
        &self.bars
    }

    /// Let go: the fall from the pose now to the ground `ground` finds
    /// under the root, from rest.
    pub fn release(&self, ground: &dyn Fn(Vec3) -> Option<f32>, stood: &LocalPose, rig: &RigGeometry) -> Falling {
        let root = self.root();
        let floor = ground(root).unwrap_or(self.stood_at.y);
        Falling::off(root, self.facing(), Vec3::ZERO, &self.pose(rig), floor, 0.0, stood, rig)
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

    use crate::character::anim::hand::wrist_bend;

    fn world(pose: &LocalPose, root: Vec3, yaw: f32, rig: &RigGeometry) -> BoneSet<Vec3> {
        let at = forward_kinematics_on(pose, rig);
        BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
    }

    /// Lines of 5 and 8 bars, 0.35 and 0.45 m apart, 2.3 m up, along -Z and
    /// along +X, with the rig's own fingers' grips: got on and crossed to
    /// the last bar, every hand that holds within 1 mm of its place on its
    /// bar, its fingers within 0.5 rad of its forearm's line; nothing into a bar; no joint over
    /// 6 m/s about the hips nor its step changing over 2 cm in a frame;
    /// across in the time its moves take; let go under the last bar, it
    /// lands there.
    #[test]
    fn monkey_bars_are_crossed_hand_over_hand() {
        let (stood, rig) = real_stood();
        let mut faults = Vec::new();
        for (bars, spacing, way) in [(5usize, 0.4f32, Vec3::NEG_Z), (8, 0.35, Vec3::X), (5, 0.45, Vec3::NEG_Z)] {
            let line = MonkeyBars::new(Vec3::new(0.3, 2.3, -0.5), way, spacing, bars);
            let name = format!("{bars} bars {spacing} m apart along {way:?}");
            let spot = Crossing::spot(&line, 0.0, &stood, &rig);
            let mut crossing = Crossing::get_on(&line, spot, &stood, &rig);
            // The rig's own fingers' grips, as the walker gives them.
            let grips = crate::character::anim::hand::puppet_grips();
            crossing.set_grips(grips, &rig);
            let (mut bend, mut flex) = (0.0f32, (0.0f32, 0.0f32));
            let (mut hand_off, mut into, mut fastest, mut kink) = (0.0f32, 0.0f32, 0.0f32, (0.0f32, 0.0f32, Bone::Hips));
            let mut frames: Vec<BoneSet<Vec3>> = Vec::new();
            let mut t = 0.0;
            while !crossing.is_released() && t < 60.0 {
                crossing.advance(DT);
                t += DT;
                let pose = crossing.pose(&rig);
                if !Bone::ALL.iter().all(|&b| pose.rotations[b].is_finite()) {
                    faults.push(format!("{name}: NaN at {t:.2}"));
                    break;
                }
                let now = world(&pose, crossing.root(), crossing.facing(), &rig);
                let bends = wrist_bend(&pose, &rig, &grips);
                for (side, (wrist, held)) in crossing.wrists(&rig).into_iter().enumerate() {
                    if held {
                        hand_off = hand_off.max((now[ARMS[side].wrist] - wrist).length());
                        bend = bend.max(bends[side].0);
                        flex = (flex.0.min(bends[side].1), flex.1.max(bends[side].1));
                    }
                }
                // Into a bar: any joint but the hands' within the bar's
                // radius of its axis, along its length.
                for k in 0..bars {
                    let bar = line.bar(k as f32);
                    for &bone in Bone::ALL.iter().filter(|b| !matches!(b, Bone::LeftHand | Bone::RightHand | Bone::LeftForeArm | Bone::RightForeArm)) {
                        let d = now[bone] - bar;
                        let off = (d - line.across() * d.dot(line.across())).length();
                        if d.dot(line.across()).abs() < 0.6 {
                            into = into.max(BAR_RADIUS + 0.03 - off);
                        }
                    }
                }
                if let Some(last) = frames.last() {
                    fastest = Bone::ALL.iter().map(|&b| ((now[b] - now[Bone::Hips]) - (last[b] - last[Bone::Hips])).length() / DT).fold(fastest, f32::max);
                }
                if frames.len() >= 2 {
                    let (a, b) = (&frames[frames.len() - 2], &frames[frames.len() - 1]);
                    for &bone in Bone::ALL.iter() {
                        let step = (now[bone] - 2.0 * b[bone] + a[bone]).length();
                        if step > kink.0 {
                            kink = (step, t, bone);
                        }
                    }
                }
                frames.push(now);
            }
            let took = t - GET_ON;
            // A move to each bar after the first, and the hands' match.
            let expected = bars as f32 * STEP + SETTLE + HOLD;
            let mut falling = crossing.release(&|_| Some(0.0), &stood, &rig);
            while !falling.is_done() {
                falling.advance(DT);
            }
            let landed = falling.root();
            let under = line.bar(line.last() as f32);
            if bend > 0.5 || flex.0 < -0.9 || flex.1 > 1.3 {
                faults.push(format!("{name}: a held hand bent {bend:.2} rad sideways, flexed {:.2}..{:.2}", flex.0, flex.1));
            }
            eprintln!(
                "{name}: wrist bend {bend:.3}, flexed {:.2}..{:.2}, hand off {hand_off:.5}, into {into:.4}, fastest {fastest:.2}, kink {:.4} at {:.2} ({:?}), took {took:.2} (expected {expected:.2}), landed {landed:?} under {under:?}",
                flex.0, flex.1, kink.0, kink.1, kink.2
            );
            if hand_off > 1.0e-3 {
                faults.push(format!("{name}: a held hand {hand_off:.4} m off its bar"));
            }
            if into > 0.0 {
                faults.push(format!("{name}: a joint {into:.4} m into a bar"));
            }
            if fastest > 6.0 {
                faults.push(format!("{name}: a joint at {fastest:.2} m/s about the hips"));
            }
            if kink.0 > 0.02 {
                faults.push(format!("{name}: a step changed {:.4} m at {:.2} ({:?})", kink.0, kink.1, kink.2));
            }
            if landed.y.abs() > 1.0e-3 || (landed - under).with_y(0.0).length() > 0.5 {
                faults.push(format!("{name}: landed at {landed:?}, the last bar at {under:?}"));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
