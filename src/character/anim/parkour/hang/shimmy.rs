//! Shimmying along a ledge, hand over hand, and round its corners: step 3
//! of the parkour design.
//!
//! Each step, the lead hand (on the side it goes) lets go, comes up off the
//! lip and hooks on again a stride along; the grip's middle moves along
//! after it, and the hips follow it on the hang's sideways spring, lagging
//! a little as a hanging body does; then the trail hand follows to its
//! lane, a shoulder's width from the lead. The hands never cross, and one
//! always holds. Braced, each foot steps along the wall in turn; free, the
//! legs hang from the hips and swing as they lag.
//!
//! At a corner (another ledge meeting this one's end at its height,
//! [`Ledge::joined`](super::Ledge::joined)) the lead hand goes round onto
//! the next ledge, the body turns with the face (outward round a block's
//! corner, inward into a wall's), and the trail hand follows. The body is
//! carried by the one rigid motion from hanging on the first face to
//! hanging on the next: in the plane, a rotation about its fixed point.
//! Turned about the corner itself, a body at an inside corner would go into
//! the side wall.
//!
//! There is no data for a hanging traverse (`parkour-movement-data`): speed
//! climbers' 2.5-2.8 hand moves a second is only an upper bound. The timing
//! is set by eye.

use bevy::math::{Mat2, Quat, Vec2, Vec3};

use super::{Hanging, Ledge};
use crate::character::anim::gait::smoothstep;

/// Which way to shimmy: the character's own left or right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shimmy {
    Left,
    Right,
}

/// How far along each step goes, metres, and how long it takes, seconds:
/// 0.25 m/s. A step round a corner takes longer.
const STRIDE: f32 = 0.25;
const CYCLE: f32 = 1.0;
/// Round an outside corner the body swings about 0.9 m: in 1.6 s, at
/// 4.4 m/s².
const CORNER_CYCLE: f32 = 2.4;
/// When, as shares of a step, the lead hand moves, the grip's middle moves
/// along, the trail hand moves; and, braced, the lead foot and the trail
/// foot.
const LEAD_HAND: (f32, f32) = (0.0, 0.35);
const GRIP: (f32, f32) = (0.0, 0.75);
const TRAIL_HAND: (f32, f32) = (0.45, 0.8);
const LEAD_FOOT: (f32, f32) = (0.25, 0.55);
const TRAIL_FOOT: (f32, f32) = (0.55, 0.9);
/// The lead hand, the trail hand and the body round an outside corner,
/// and an inside one. Round an inside corner the trail hand goes while the
/// body has turned only about a third: going as round an outside one, with
/// the body turned over half, it was 7 cm out of reach on the face it left
/// (hanging free). Round an outside corner, so early, the lead hand was
/// 1.2 cm out of reach on the face it went onto.
const OUTSIDE_TIMING: [(f32, f32); 3] = [(0.0, 0.35), (0.45, 0.8), (0.0, 0.75)];
const INSIDE_TIMING: [(f32, f32); 3] = [(0.0, 0.35), (0.4, 0.72), (0.0, 0.82)];
/// A moving hand comes up off the lip this far, and out from the face,
/// metres, at the middle of its move; a moving foot out from the wall.
const HAND_LIFT: f32 = 0.05;
const HAND_OUT: f32 = 0.03;
const FOOT_OUT: f32 = 0.05;
/// Round a corner, a hand goes this far out from both faces, metres.
const CORNER_HAND_OUT: f32 = 0.1;
/// Round a corner, braced feet come off the wall over the first window
/// (shares of the step) and plant again on the next face over the second.
const CORNER_FEET_OFF: ((f32, f32), (f32, f32)) = ((0.0, 0.2), (0.65, 0.9));
/// A hand stays this far from the edge's end, metres; a step shorter than
/// this is not taken.
const END_MARGIN: f32 = 0.08;
const SHORTEST: f32 = 0.05;
/// A hand comes this near a corner, on either ledge, metres: an outside
/// corner, and an inside one, its elbow clear of the side wall, yet near
/// enough for the lead hand to reach across onto it (at 0.45 m, 0.73 m
/// across: 9.6 cm out of reach hanging free).
const OUTSIDE_CORNER: f32 = 0.12;
const INSIDE_CORNER: f32 = 0.3;
/// Before a corner the hands come this near each other, metres.
const MATCHED: f32 = 0.12;
/// Shimmying, the hips come this much nearer the grip than hanging,
/// metres, over this long, seconds.
const PULL: f32 = 0.06;
const PULL_TIME: f32 = 0.3;
/// The first step starts once the hips are within this of pulled up,
/// metres.
const PULLED_WITHIN: f32 = 0.01;
/// A moving hand opens over this share of its move, and closes over the
/// same at its end.
const OPENING: f32 = 0.25;

/// Eased 0-1 across `(from, to)`.
fn across(s: f32, (from, to): (f32, f32)) -> f32 {
    smoothstep(((s - from) / (to - from)).clamp(0.0, 1.0))
}

/// Whether two ledges are the same edge, either way round.
fn same(a: &Ledge, b: &Ledge) -> bool {
    ((a.a - b.a).length() < 1.0e-3 && (a.b - b.b).length() < 1.0e-3) || ((a.a - b.b).length() < 1.0e-3 && (a.b - b.a).length() < 1.0e-3)
}

/// Round a corner: onto which ledge, how far the face turns about `+Y`,
/// and the rigid motion carrying the body there.
#[derive(Debug, Clone)]
struct Corner {
    next: Ledge,
    turn: f32,
    /// The corner, and its motion's fixed point.
    at: Vec3,
    pivot: Vec3,
    /// The facing, its turn and the face's axes as it began.
    from_yaw: f32,
    from_turn: Quat,
    from_out: Vec3,
    from_along: Vec3,
    /// Into a wall's corner, not round a block's.
    inside: bool,
}

impl Corner {
    /// When the lead hand, the trail hand and the body move.
    fn timing(&self) -> [(f32, f32); 3] {
        if self.inside { INSIDE_TIMING } else { OUTSIDE_TIMING }
    }
}

/// A step along the ledge under way.
#[derive(Debug, Clone)]
pub(super) struct Step {
    /// Seconds into it, and how long it takes.
    t: f32,
    cycle: f32,
    /// Along the edge the way it goes (unit, the world).
    way: Vec3,
    /// The hand and foot on the side it goes.
    lead: usize,
    /// Each lip point, foot's ball and the grip's middle as it began, and
    /// each lip point where it lands.
    from_lips: [Vec3; 2],
    from_balls: [Vec3; 2],
    from_grip: Vec3,
    to_lips: [Vec3; 2],
    corner: Option<Corner>,
}

impl Step {
    /// The share of the step done.
    fn share(&self) -> f32 {
        (self.t / self.cycle).clamp(0.0, 1.0)
    }

    /// When hand `side` moves, as shares of the step.
    fn hand_window(&self, side: usize) -> (f32, f32) {
        let lead = side == self.lead;
        match self.corner.as_ref() {
            Some(corner) => corner.timing()[if lead { 0 } else { 1 }],
            None if lead => LEAD_HAND,
            None => TRAIL_HAND,
        }
    }

    /// When the grip's middle (and with it the body) moves, as shares of
    /// the step.
    fn grip_window(&self) -> (f32, f32) {
        self.corner.as_ref().map_or(GRIP, |corner| corner.timing()[2])
    }

    fn foot_window(&self, side: usize) -> (f32, f32) {
        if side == self.lead { LEAD_FOOT } else { TRAIL_FOOT }
    }

    /// How far the body has turned round the corner, radians.
    fn turned(&self) -> f32 {
        self.corner.as_ref().map_or(0.0, |corner| corner.turn * across(self.share(), corner.timing()[2]))
    }

    /// Hand `side`'s progress through its move, 0-1.
    fn hand(&self, side: usize) -> f32 {
        across(self.share(), self.hand_window(side))
    }

    /// Whether hand `side` stays where it is this step (the lead, as the
    /// trail hand comes up to it before a corner).
    fn stays(&self, side: usize) -> bool {
        (self.to_lips[side] - self.from_lips[side]).length() < 1.0e-4
    }

    /// How far the grip's middle goes along, straight.
    fn moved(&self) -> Vec3 {
        0.5 * (self.to_lips[0] + self.to_lips[1] - self.from_lips[0] - self.from_lips[1])
    }
}

impl Hanging {
    /// Asks it to shimmy (each frame it should, `None` to stop): it starts a
    /// step when hanging, not climbing up, and finishes the step under way
    /// once asked to stop. At the edge's end it goes round a corner onto
    /// another of [`Self::set_others`] meeting it there, or stops.
    pub fn shimmy(&mut self, ask: Option<Shimmy>) {
        self.shimmy_ask = ask;
    }

    /// The other ledges it may shimmy onto round a corner.
    pub fn set_others(&mut self, ledges: &[Ledge]) {
        self.others = ledges.iter().filter(|other| !same(other, &self.ledge)).copied().collect();
    }

    /// Whether it is shimmying, a step under way.
    pub fn is_shimmying(&self) -> bool {
        self.step.is_some()
    }

    /// The face it hangs on now, out of it and along its edge: turning as
    /// it goes round a corner.
    pub(super) fn face(&self) -> (Vec3, Vec3) {
        match self.step.as_ref().and_then(|step| step.corner.as_ref().map(|corner| (step, corner))) {
            Some((step, corner)) => {
                let turn = Quat::from_rotation_y(step.turned());
                (turn * corner.from_out, turn * corner.from_along)
            }
            None => (self.ledge.out, self.ledge.along()),
        }
    }

    /// Out of the face hand `side` hooks onto: going round a corner, turning
    /// from the one face's to the next as it moves.
    pub(super) fn hand_out(&self, side: usize) -> Vec3 {
        match self.step.as_ref().and_then(|step| step.corner.as_ref().map(|corner| (step, corner))) {
            Some((step, corner)) => Quat::from_rotation_y(corner.turn * step.hand(side)) * corner.from_out,
            None => self.ledge.out,
        }
    }

    /// The step asked for: along the edge if there is room, else round a
    /// corner onto another ledge meeting its end.
    fn next_step(&self) -> Option<Step> {
        let ask = self.shimmy_ask?;
        if self.up_asked || !self.is_hanging() {
            return None;
        }
        let left = self.turn * self.rig.left();
        let (lead, toward) = match ask {
            Shimmy::Left => (0, left),
            Shimmy::Right => (1, -left),
        };
        let along = self.ledge.along();
        let way = if along.dot(toward) >= 0.0 { along } else { -along };
        let end = usize::from(way.dot(along) > 0.0);
        let corner = self.ledge.joined(end, &self.others).filter(|(_, turn)| turn.abs() > 0.5);
        let near = |next: &Ledge| if next.along().dot(self.ledge.out) < 0.0 { OUTSIDE_CORNER } else { INSIDE_CORNER };
        let margin = corner.as_ref().map_or(END_MARGIN, |(next, _)| near(next));
        let end_at = if end == 1 { self.ledge.b } else { self.ledge.a };
        let room = (end_at - self.lips[lead]).dot(way) - margin;
        let length = STRIDE.min(room);
        let trail = 1 - lead;
        let gap = (self.lips[lead] - self.lips[trail]).dot(way);
        // The lead a stride on, the trail hand a gap behind it: the hands'
        // own width apart, or matched up before a corner.
        let along_to = |lead_to: Vec3, gap: f32| {
            let mut to_lips = [Vec3::ZERO; 2];
            to_lips[lead] = lead_to;
            to_lips[trail] = lead_to - way * gap;
            to_lips
        };
        let step = |to_lips: [Vec3; 2], cycle: f32, corner: Option<Corner>| Step {
            t: 0.0,
            cycle,
            way,
            lead,
            from_lips: self.lips,
            from_balls: self.wall_balls,
            from_grip: self.grip,
            to_lips,
            corner,
        };
        if length >= SHORTEST {
            return Some(step(along_to(self.lips[lead] + way * length, self.spread), CYCLE, None));
        }
        let (next, turn) = corner?;
        // Hanging already nearer an inside corner than that, its shoulder
        // against the side wall, it has no room to turn: it stops.
        if room < -SHORTEST {
            return None;
        }
        // At the corner, the trail hand comes up to the lead first: a hand
        // left a shoulder's width back on this face was 17 cm out of reach
        // as the body turned onto the next.
        if gap > MATCHED + SHORTEST {
            return Some(step(along_to(self.lips[lead], MATCHED), CYCLE, None));
        }
        // Round the corner: each hand onto the next ledge, as near the corner
        // and as near each other as they are here, the lead on ahead (put
        // nearer the corner, the hands crossed); the body carried rigidly
        // from hanging on this face to hanging on that.
        let reach = near(&next);
        let mut to_lips = [Vec3::ZERO; 2];
        to_lips[lead] = next.a + next.along() * (reach + gap);
        to_lips[trail] = next.a + next.along() * reach;
        let to_grip = 0.5 * (to_lips[0] + to_lips[1]);
        let rotation = Quat::from_rotation_y(turn);
        let (rx, rz) = (rotation * Vec3::X, rotation * Vec3::Z);
        let fixed = Mat2::from_cols(Vec2::new(1.0 - rx.x, -rx.z), Vec2::new(-rz.x, 1.0 - rz.z));
        let moved = to_grip - rotation * self.grip;
        let pivot = fixed.inverse() * Vec2::new(moved.x, moved.z);
        let corner = Corner {
            next,
            turn,
            at: next.a,
            pivot: Vec3::new(pivot.x, self.grip.y, pivot.y),
            from_yaw: self.yaw,
            from_turn: self.turn,
            from_out: self.ledge.out,
            from_along: self.ledge.along(),
            inside: reach == INSIDE_CORNER,
        };
        Some(step(to_lips, CORNER_CYCLE, Some(corner)))
    }

    /// Moves the step under way on `dt`, starting the next if still asked:
    /// the grip's middle along (the hips' sideways offset from it kept, so
    /// they follow on the spring), or the body round a corner; each hand's
    /// and foot's hold where it lands.
    pub(super) fn advance_shimmy(&mut self, dt: f32) {
        // Pulled up a little while shimmying, both arms slack enough to
        // reach a lip off to the side: at the hang's length a held wrist was
        // 1.9 cm short of its hook. A step starts once pulled up (starting at
        // once, the first lead hand was 3.3 mm short in the air).
        let wanted = self.step.is_some() || self.next_step().is_some();
        let toward = if wanted { PULL } else { 0.0 };
        self.pulled += (toward - self.pulled).clamp(-PULL * dt / PULL_TIME, PULL * dt / PULL_TIME);
        if self.step.is_none() && self.pulled >= PULL && self.swing.r <= self.rest.0 - PULL + PULLED_WITHIN {
            self.step = self.next_step();
        }
        let Some(step) = self.step.as_mut() else { return };
        step.t += dt;
        let s = step.share();
        match step.corner.as_ref() {
            Some(corner) => {
                let turned = Quat::from_rotation_y(step.turned());
                self.yaw = corner.from_yaw + step.turned();
                self.turn = turned * corner.from_turn;
                self.grip = corner.pivot + turned * (step.from_grip - corner.pivot);
            }
            None => {
                let grip = step.from_grip + step.moved() * across(s, step.grip_window());
                self.swing.along -= (grip - self.grip).dot(self.ledge.along());
                self.grip = grip;
            }
        }
        for side in 0..2 {
            if s >= step.hand_window(side).1 {
                self.lips[side] = step.to_lips[side];
            }
            if self.braced && step.corner.is_none() && s >= step.foot_window(side).1 {
                self.wall_balls[side] = step.from_balls[side] + step.moved();
            }
        }
        if step.t >= step.cycle {
            let over = step.t - step.cycle;
            if let Some(corner) = step.corner.clone() {
                self.turned_corner(&corner);
            }
            self.step = None;
            self.step = self.next_step();
            if let Some(next) = self.step.as_mut() {
                next.t = over;
            }
        }
    }

    /// Round the corner: hanging on the next ledge, the last one among the
    /// others, the body's sideways offset and the feet's holds taken onto
    /// the new face.
    fn turned_corner(&mut self, corner: &Corner) {
        let rotation = Quat::from_rotation_y(corner.turn);
        self.yaw = corner.from_yaw + corner.turn;
        self.turn = rotation * corner.from_turn;
        let from_grip = self.step.as_ref().map_or(self.grip, |step| step.from_grip);
        self.grip = corner.pivot + rotation * (from_grip - corner.pivot);
        self.wall_balls = self.wall_balls.map(|ball| corner.pivot + rotation * (ball - corner.pivot));
        // Along the new edge the other way round, the offset turns sign.
        if (rotation * corner.from_along).dot(corner.next.along()) < 0.0 {
            self.swing.along = -self.swing.along;
            self.swing.dalong = -self.swing.dalong;
        }
        let was = self.ledge;
        self.ledge = corner.next;
        self.others.retain(|other| !same(other, &corner.next));
        self.others.push(was);
    }

    /// Where hand `side`'s lip point is now: on the lip, or coming up off it
    /// and along to the next (round a corner, out round it).
    pub(super) fn lip_now(&self, side: usize) -> Vec3 {
        let Some(step) = self.step.as_ref() else { return self.lips[side] };
        let u = step.hand(side);
        if u <= 0.0 || u >= 1.0 || step.stays(side) {
            return self.lips[side];
        }
        let (from, to) = (step.from_lips[side], step.to_lips[side]);
        let hump = (std::f32::consts::PI * u).sin();
        match step.corner.as_ref() {
            Some(corner) => {
                let round = corner.at + (corner.from_out + corner.next.out).normalize_or(corner.from_out) * CORNER_HAND_OUT;
                let r = 1.0 - u;
                from * (r * r) + round * (2.0 * r * u) + to * (u * u) + Vec3::Y * (HAND_LIFT * hump)
            }
            None => from.lerp(to, u) + Vec3::Y * (HAND_LIFT * hump) + self.ledge.out * (HAND_OUT * hump),
        }
    }

    /// Whether hand `side` is off the lip, moving.
    pub fn hand_moving(&self, side: usize) -> bool {
        self.step.as_ref().is_some_and(|step| {
            let (from, to) = step.hand_window(side);
            !step.stays(side) && (from..to).contains(&step.share())
        })
    }

    /// Where foot `side`'s ball is now, braced: on the wall, stepping along
    /// it, or carried round a corner; raised as far as the body is pulled
    /// up (held where it was, a foot was 2.1 cm short of its hold).
    pub(super) fn ball_now(&self, side: usize) -> Vec3 {
        self.ball_along(side) + Vec3::Y * self.pulled
    }

    fn ball_along(&self, side: usize) -> Vec3 {
        let Some(step) = self.step.as_ref() else { return self.wall_balls[side] };
        if let Some(corner) = step.corner.as_ref() {
            let turned = Quat::from_rotation_y(step.turned());
            return corner.pivot + turned * (step.from_balls[side] - corner.pivot);
        }
        let u = across(step.share(), step.foot_window(side));
        if u <= 0.0 || u >= 1.0 {
            return self.wall_balls[side];
        }
        step.from_balls[side] + step.moved() * u + self.ledge.out * (FOOT_OUT * (std::f32::consts::PI * u).sin())
    }

    /// How far braced feet are off the wall, hanging (0-1): going round a
    /// corner they come off it and swing round, planting again on the next
    /// face. Carried round on the wall, a foot's toes went 6.5 cm into the
    /// block by the corner.
    pub(super) fn feet_off_wall(&self) -> f32 {
        self.step.as_ref().filter(|step| step.corner.is_some()).map_or(0.0, |step| {
            let s = step.share();
            across(s, CORNER_FEET_OFF.0) * (1.0 - across(s, CORNER_FEET_OFF.1))
        })
    }

    /// How far each elbow is tucked back toward the hips (0-1): round an
    /// inside corner, clear of the side wall. Pointing out to the side as
    /// hanging, the lead forearm went 1.15 cm into it as the hand crossed.
    pub(super) fn elbows_tucked(&self) -> [f32; 2] {
        let tucked = self.step.as_ref().filter(|step| step.corner.as_ref().is_some_and(|corner| corner.inside)).map_or(0.0, |step| {
            let s = step.share();
            across(s, (0.0, 0.15)) * (1.0 - across(s, (0.85, 1.0)))
        });
        [tucked; 2]
    }

    /// Whether foot `side` is off its hold, stepping or carried round.
    pub fn foot_moving(&self, side: usize) -> bool {
        self.step.as_ref().is_some_and(|step| {
            let (from, to) = if step.corner.is_some() { step.grip_window() } else { step.foot_window(side) };
            (from..to).contains(&step.share())
        })
    }

    /// How closed hand `side` is shimmying: opening as it lets go, closing
    /// as it hooks on again; `None` if it is not moving.
    pub(super) fn shimmy_grip(&self, side: usize) -> Option<f32> {
        let step = self.step.as_ref().filter(|step| !step.stays(side))?;
        let (from, to) = step.hand_window(side);
        let u = (step.share() - from) / (to - from);
        (0.0..1.0).contains(&u).then(|| 1.0 - smoothstep((u.min(1.0 - u) / OPENING).clamp(0.0, 1.0)))
    }

    /// Where it looks shimmying: along the lip the way it goes, round a
    /// corner along the next.
    pub(super) fn shimmy_look(&self) -> Option<Vec3> {
        self.step.as_ref().map(|step| match step.corner.as_ref() {
            Some(corner) => corner.at + corner.next.along() * 0.6,
            None => self.grip + step.way * 0.6,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::{grabbing, real_stood, wall};
    use super::super::{ARMS, LEGS};
    use super::*;
    use crate::character::anim::rig::{forward_kinematics_on, BoneSet};
    use crate::character::skeleton::Bone;

    const DT: f32 = 1.0 / 60.0;

    /// What a shimmy measured, frame by frame.
    #[derive(Debug, Default)]
    struct Measured {
        /// The most a held hand's wrist strays from its hook, metres, and
        /// where: the hand, seconds, the share of its step (-1 between
        /// steps), whether round a corner.
        held: f32,
        worst_held: (usize, f32, f32, bool),
        /// Frames with no hand on the lip.
        none_held: usize,
        /// The most a moving hand's wrist strays from its way, metres, and
        /// the highest its lip point comes above the lip.
        moving_off: f32,
        lifted: f32,
        /// Where the worst of it was: the hand, the seconds, the share of its
        /// step, the step's lead hand.
        worst_moving: (usize, f32, f32, usize),
        /// The least gap between the hands, the left hand on the character's
        /// left, metres (negative: crossed).
        least_gap: f32,
        /// The deepest any joint goes into a block under a ledge, metres,
        /// which and when.
        into_block: f32,
        deepest: Option<(Bone, f32)>,
        /// Braced, the most a planted foot's ball strays from its hold.
        feet_off: f32,
        /// The hips' greatest acceleration, m/s², and how far they went
        /// along the first edge, metres; their speed at the end.
        acceleration: f32,
        travelled: f32,
        settled_speed: f32,
        /// Each lip point's distance from the edge's nearer end, the least.
        end_room: f32,
    }

    /// How deep `p` is inside the block under `ledge` (from its face back
    /// its depth, along its edge, down its wall): the least way out; 0
    /// outside it.
    fn inside(ledge: &Ledge, p: Vec3) -> f32 {
        let (into, down) = (-ledge.out_of(p), ledge.height() - p.y);
        let along = (p - ledge.a).dot(ledge.along());
        let length = (ledge.b - ledge.a).length();
        if into > 0.0 && into < ledge.depth && down > 0.0 && down < ledge.wall_below && along > 0.0 && along < length {
            // Out through any side: through the far one too (a toe on a
            // block's front read a whole block deep from its back face).
            into.min(ledge.depth - into).min(down).min(along).min(length - along)
        } else {
            0.0
        }
    }

    fn shimmied(ledge: &Ledge, others: &[Ledge], ask: Shimmy, seconds: f32, then: f32) -> (Measured, Hanging) {
        let (_, rig) = real_stood();
        let mut hanging = grabbing(ledge).expect("in reach");
        hanging.set_others(others);
        hanging.advance(if hanging.is_braced() { 3.0 } else { 10.0 });
        let mut m = Measured { least_gap: f32::MAX, end_room: f32::MAX, ..Default::default() };
        let mut hips = Vec::new();
        let start = hanging.hang_hips();
        let blocks: Vec<Ledge> = std::iter::once(*ledge).chain(others.iter().copied()).collect();
        let frames = ((seconds + then) / DT).round() as usize;
        for frame in 0..frames {
            hanging.shimmy((frame as f32 * DT < seconds).then_some(ask));
            hanging.advance(DT);
            let pose = hanging.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let turn = Quat::from_rotation_y(hanging.facing());
            let world = BoneSet::from_fn(|bone| hanging.root() + turn * at[bone]);
            hips.push(world[Bone::Hips]);
            let wrists = hanging.wrists();
            let mut holding = 0;
            for side in 0..2 {
                let off = (world[ARMS[side].wrist] - wrists[side]).length();
                if hanging.hand_moving(side) {
                    if off > m.moving_off {
                        let step = hanging.step.as_ref().expect("stepping");
                        m.worst_moving = (side, frame as f32 * DT, step.share(), step.lead);
                    }
                    m.moving_off = m.moving_off.max(off);
                    // Off the lip by its lift at the middle of its move.
                    m.lifted = m.lifted.max(hanging.lip_now(side).y - ledge.height());
                } else {
                    holding += 1;
                    if off > m.held {
                        m.worst_held = (side, frame as f32 * DT, hanging.step.as_ref().map_or(-1.0, |step| step.share()), hanging.step.as_ref().is_some_and(|step| step.corner.is_some()));
                    }
                    m.held = m.held.max(off);
                    let lip = hanging.lip_now(side);
                    let now = hanging.ledge();
                    let along = (lip - now.a).dot(now.along());
                    m.end_room = m.end_room.min(along.min((now.b - now.a).length() - along));
                }
            }
            if holding == 0 {
                m.none_held += 1;
            }
            m.least_gap = m.least_gap.min((hanging.lip_now(0) - hanging.lip_now(1)).dot(turn * rig.left()));
            if hanging.is_braced() && hanging.since > 0.6 && hanging.feet_off_wall() <= 0.0 {
                for side in 0..2 {
                    if !hanging.foot_moving(side) {
                        // The ball on its hold: off the face alone, a toe
                        // 20 cm from its hold, dragged along the face,
                        // passed.
                        m.feet_off = m.feet_off.max((world[LEGS[side].3] - hanging.ball_now(side)).length());
                    }
                }
            }
            for bone in Bone::ALL {
                for block in &blocks {
                    let depth = inside(block, world[bone]);
                    if depth > m.into_block {
                        (m.into_block, m.deepest) = (depth, Some((bone, frame as f32 * DT)));
                    }
                }
            }
        }
        m.acceleration = hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        m.travelled = (hanging.hang_hips() - start).dot(ledge.along()).abs();
        m.settled_speed = hips.windows(2).last().map_or(0.0, |w| (w[1] - w[0]).length() / DT);
        (m, hanging)
    }

    /// Asked to shimmy left or right, braced and free, it goes hand over
    /// hand along the lip: one hand always on it and held within a
    /// millimetre, the hands never crossing, nothing through the wall, the
    /// braced feet on the face between their steps, the hips moving
    /// smoothly; and stopped, it comes to rest.
    #[test]
    fn it_shimmies_along_the_lip_hand_over_hand() {
        for (below, name) in [(2.15, "braced"), (0.15, "free")] {
            for ask in [Shimmy::Left, Shimmy::Right] {
                let ledge = wall(2.15, below);
                let (m, _) = shimmied(&ledge, &[], ask, 3.0, 4.0);
                let name = format!("{name} {ask:?}");
                assert_eq!(m.none_held, 0, "{name}: {} frames with no hand on the lip", m.none_held);
                assert!(m.held < 1.0e-3, "{name}: a held wrist {:.4} m off its hook", m.held);
                // In the air too: with the body following only from a tenth
                // of the step, the lead arm reached its stride at full
                // stretch, 4.3 mm short.
                assert!(m.moving_off < 1.0e-3, "{name}: a moving wrist {:.4} m off its way (hand, s, share, lead: {:?})", m.moving_off, m.worst_moving);
                assert!(m.lifted > 0.8 * HAND_LIFT, "{name}: a moving hand only {:.3} m off the lip", m.lifted);
                assert!(m.least_gap > 0.2, "{name}: the hands {:.3} m apart at the least", m.least_gap);
                assert!(m.into_block < 1.0e-3, "{name}: {:?} {:.4} m into the wall", m.deepest, m.into_block);
                assert!(m.feet_off < 0.01, "{name}: a planted foot {:.4} m off its hold", m.feet_off);
                assert!(m.acceleration < 3.0, "{name}: the hips accelerated {:.2} m/s²", m.acceleration);
                assert!((m.travelled - 3.0 * STRIDE).abs() < 0.03, "{name}: went {:.3} m along, three strides", m.travelled);
                assert!(m.settled_speed < 0.01, "{name}: still moving at {:.3} m/s", m.settled_speed);
            }
        }
    }

    /// Shimmying on to the edge's end, it stops with its hands short of it.
    #[test]
    fn it_stops_at_the_edges_end() {
        let ledge = Ledge::wall(Vec3::new(0.0, 0.0, -0.5), Vec3::Z, 1.6, 2.15, 1.0);
        let (m, _) = shimmied(&ledge, &[], Shimmy::Right, 6.0, 2.0);
        assert!(m.end_room >= END_MARGIN - 1.0e-3, "a hand {:.3} m from the end", m.end_room);
        assert_eq!(m.none_held, 0);
    }

    /// Shimmying on round a block's corner (outward) and into a wall's
    /// (inward), braced and free, it turns onto the next ledge and goes on
    /// along it: a hand always held, the hands never crossing, nothing in
    /// either block, and at rest hanging square to the new face.
    #[test]
    fn it_shimmies_round_outside_and_inside_corners() {
        let height = 2.15;
        for below in [height, 0.15] {
            let with_wall = |ledge: Ledge| Ledge { wall_below: below, ..ledge };
            let block = Ledge::block(Vec3::new(-0.2, 0.0, -0.5), Vec3::Z, 1.6, 1.0, height).map(with_wall);
            // The inside corner far enough on to come up to it (grabbed with
            // the lead hand 0.19 m from it, the shoulder was against the
            // side wall).
            let front = with_wall(Ledge::wall(Vec3::new(-0.25, 0.0, -0.5), Vec3::Z, 2.3, height, 1.0));
            let side = with_wall(Ledge::wall(Vec3::new(0.9, 0.0, 0.0), Vec3::NEG_X, 1.0, height, 0.5));
            for (name, first, others, turn) in [("outside", block[0], block.to_vec(), std::f32::consts::FRAC_PI_2), ("inside", front, vec![side], -std::f32::consts::FRAC_PI_2)] {
                let name = format!("{name}, {below} m of wall");
                let (_, rig) = real_stood();
                let facing = grabbing(&first).expect("in reach").facing();
                let (m, hanging) = shimmied(&first, &others, Shimmy::Right, 6.0, 4.0);
                assert_eq!(m.none_held, 0, "{name}: {} frames with no hand on the lip", m.none_held);
                assert!(m.held < 1.0e-3, "{name}: a held wrist {:.4} m off its hook ({:?})", m.held, m.worst_held);
                assert!(m.moving_off < 1.0e-3, "{name}: a moving wrist {:.4} m off its way ({:?})", m.moving_off, m.worst_moving);
                assert!(m.feet_off < 0.01, "{name}: a planted foot {:.4} m off its hold", m.feet_off);
                assert!(m.least_gap > 0.8 * MATCHED, "{name}: the hands {:.3} m apart at the least", m.least_gap);
                assert!(m.into_block < 1.0e-3, "{name}: {:?} {:.4} m into a block", m.deepest, m.into_block);
                assert!(m.acceleration < 3.0, "{name}: the hips accelerated {:.2} m/s²", m.acceleration);
                assert!(m.settled_speed < 0.01, "{name}: still moving at {:.3} m/s", m.settled_speed);
                let now = hanging.ledge();
                assert!(now.out.dot(first.out).abs() < 1.0e-3, "{name}: not round the corner, on a face out {:?}", now.out);
                let turned = (hanging.facing() - facing + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
                assert!((turned - turn).abs() < 1.0e-3, "{name}: turned {turned:.3} rad");
                // Square to the new face: the rig's forward into it.
                let forward = Quat::from_rotation_y(hanging.facing()) * rig.forward();
                assert!(forward.dot(-now.out) > 0.999, "{name}: facing {forward:?} on a face out {:?}", now.out);
            }
        }
    }
}
