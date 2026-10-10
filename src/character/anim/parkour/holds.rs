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
    /// Overhanging from this height, leant out this far from upright,
    /// radians, up to a flat roof's quarter turn ([`Self::leaning`],
    /// [`Self::bent`]).
    pub lean: Option<(f32, f32)>,
}

impl HoldWall {
    /// A wall facing `out` through `face` with `holds`, no top.
    pub fn new(face: Vec3, out: Vec3, holds: Vec<Hold>) -> Self {
        Self { holds, out: out.with_y(0.0).normalize_or(Vec3::Z), face, top: None, lean: None }
    }

    /// The same wall overhanging from `from` high up, its face leant out
    /// `angle` from upright (under a quarter turn): each hold above that
    /// height, and its top, carried straight out with it (so the steeper,
    /// the farther apart up the face).
    pub fn leaning(mut self, from: f32, angle: f32) -> Self {
        self.lean = Some((from, angle));
        let shift = |y: f32| self.out * (y - from).max(0.0) * angle.tan();
        for hold in &mut self.holds {
            hold.at += shift(hold.at.y);
        }
        if let Some(top) = self.top.as_mut() {
            let by = shift(top.height());
            (top.a, top.b) = (top.a + by, top.b + by);
        }
        self
    }

    /// The same wall bent out at `from` high up, its face above leant out
    /// `angle` from upright, up to a flat roof's quarter turn: each hold
    /// above that height, and its top, folded over with it, as far up the
    /// face as it was (so its holds are as far apart as they were). Bent
    /// past [`BENT_FREE`], nothing under the top's lip to brace the feet on
    /// (a hang from it is free).
    pub fn bent(mut self, from: f32, angle: f32) -> Self {
        self.lean = Some((from, angle));
        let fold = |p: Vec3| if p.y > from { p.with_y(from) + (Vec3::Y * angle.cos() + self.out * angle.sin()) * (p.y - from) } else { p };
        for hold in &mut self.holds {
            hold.at = fold(hold.at);
        }
        if let Some(top) = self.top.as_mut() {
            (top.a, top.b) = (fold(top.a), fold(top.b));
            if angle > BENT_FREE {
                top.wall_below = 0.0;
            }
        }
        self
    }

    /// How far up the face `point` is: its height where the face is
    /// upright, and on along the leaning face above (so a climb measured on
    /// it is an upright one turned); off the face, as the nearer of the two
    /// faces' nearest points to it is, blended over [`NEARER_BAND`] about
    /// where they are as near (taken from one or the other, a foot hanging
    /// free turned with the face that was nearer and changed its step
    /// 17 cm).
    pub fn rise(&self, point: Vec3) -> f32 {
        let Some((from, angle)) = self.lean else { return point.y };
        let (level, height) = (self.level_out(point), point.y - from);
        // Up the leaning face from the crease, and off it.
        let (up, off) = (height * angle.cos() + level * angle.sin(), level * angle.cos() - height * angle.sin());
        let upright = Vec2::new(level, height.max(0.0)).length();
        let leaning = Vec2::new(off, up.min(0.0)).length();
        let w = smoothstep(((upright - leaning) / NEARER_BAND + 0.5).clamp(0.0, 1.0));
        point.y.min(from) + (from + up.max(0.0) - point.y.min(from)) * w
    }

    /// The point on the face `along` it and `rise` up it ([`Self::rise`]).
    pub fn on_face(&self, along: f32, rise: f32) -> Vec3 {
        let base = self.face.with_y(0.0) + self.along() * along;
        match self.lean {
            Some((from, angle)) if rise > from => base + Vec3::Y * from + (Vec3::Y * angle.cos() + self.out * angle.sin()) * (rise - from),
            _ => base + Vec3::Y * rise,
        }
    }

    /// How far out of the upright face `point` is, level, metres.
    fn level_out(&self, point: Vec3) -> f32 {
        (point - self.face).dot(self.out)
    }

    /// How far the face leans out at `point`'s place up it, radians: eased
    /// in over a band about the crease, so a hand going over it turns its
    /// palm smoothly; and only as near the face as [`FACE_NEAR`] (a foot
    /// hanging free out under a leaning face turned with it, and swinging
    /// changed its step 12-17 cm).
    fn lean_at(&self, point: Vec3) -> f32 {
        self.lean.map_or(0.0, |(from, angle)| {
            let near = 1.0 - smoothstep(((self.place(point).1 - FACE_NEAR.0) / (FACE_NEAR.1 - FACE_NEAR.0)).clamp(0.0, 1.0));
            angle * smoothstep(((self.rise(point) - from) / CREASE_BAND + 0.5).clamp(0.0, 1.0)) * near
        })
    }

    /// The face's normal and its way up it at `point`'s place up it (eased
    /// over the crease, [`Self::lean_at`]).
    fn frame_at(&self, point: Vec3) -> (Vec3, Vec3) {
        let lean = self.lean_at(point);
        if lean == 0.0 {
            return (self.out, Vec3::Y);
        }
        (self.out * lean.cos() - Vec3::Y * lean.sin(), Vec3::Y * lean.cos() + self.out * lean.sin())
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

    /// Any wall, its holds grown from its roughness (step 14): a patch
    /// `width` wide and `height` high facing `out`, its foot's middle at
    /// `face`, its top a ledge to climb out onto. Its holds are a jittered
    /// grid, one to a cell, each cell's place and kind drawn from a hash of
    /// `seed` and the cell (so the same wall always has the same holds,
    /// whoever climbs it, and none is stored but the wall's own numbers).
    /// Rougher (`roughness` 0-1), the cells are smaller (from a 0.4 × 0.3 m
    /// climbing wall's to 0.65 of it) and more of the holds take a hand
    /// (from 70 % to 95 %, the rest footholds only). From 1.25 of the
    /// climbing wall's and 60 %, walls of 0.6 had gaps that took a dyno.
    pub fn rough(face: Vec3, out: Vec3, width: f32, height: f32, roughness: f32, seed: u32) -> Self {
        let out = out.with_y(0.0).normalize_or(Vec3::Z);
        let along = out.cross(Vec3::Y);
        let roughness = roughness.clamp(0.0, 1.0);
        let scale = 1.0 - 0.35 * roughness;
        let (cell_x, cell_y) = (0.4 * scale, 0.3 * scale);
        let (columns, rows) = ((width / cell_x).floor().max(1.0) as i32, ((height - ROUGH_BOTTOM - ROUGH_TOP) / cell_y).floor().max(1.0) as i32);
        let hands = 0.7 + 0.25 * roughness;
        let mut holds = Vec::with_capacity((columns * rows) as usize);
        for row in 0..rows {
            for column in 0..columns {
                let draw = |k: u32| cell_hash(seed, column, row, k);
                let u = (column as f32 + 0.5 + ROUGH_JITTER * (2.0 * draw(0) - 1.0)) * cell_x - 0.5 * columns as f32 * cell_x;
                let v = ROUGH_BOTTOM + (row as f32 + 0.5 + ROUGH_JITTER * (2.0 * draw(1) - 1.0)) * cell_y;
                let kind = match draw(2) {
                    x if x >= hands => HoldKind::Foot,
                    x if x < 0.15 * hands => HoldKind::Jug,
                    _ => HoldKind::Edge,
                };
                holds.push(Hold { at: face.with_y(0.0) + along * u + Vec3::Y * (face.y + v), kind });
            }
        }
        Self { top: Some(Ledge::wall(face, out, width, face.y + height, 1.0)), ..Self::new(face, out, holds) }
    }

    /// Along the face, level: the left of a body facing it.
    pub fn along(&self) -> Vec3 {
        self.out.cross(Vec3::Y)
    }

    /// `point`'s place on the face: along it from `face`, up it
    /// ([`Self::rise`]), and out in front of it (overhanging, from the
    /// nearer of the upright face and the leaning one: the space in front of
    /// the two is where both are in front, so a point's distance out is the
    /// less).
    pub fn place(&self, point: Vec3) -> (Vec2, f32) {
        let off = point - self.face;
        let level = off.dot(self.out);
        let out = match self.lean {
            Some((from, angle)) => level.min(level * angle.cos() - (point.y - from) * angle.sin()),
            None => level,
        };
        (Vec2::new(off.dot(self.along()), self.rise(point)), out)
    }

    /// The way out of the face nearer `point` ([`Self::place`]'s): the
    /// upright one's, or the leaning one's, blended over [`NEARER_BAND`]
    /// about where the two are as near (taken from one or the other, a
    /// hand turned off the face by it changed its step 16 cm).
    pub fn normal_nearest(&self, point: Vec3) -> Vec3 {
        let Some((from, angle)) = self.lean else { return self.out };
        let level = self.level_out(point);
        let leaning = level * angle.cos() - (point.y - from) * angle.sin();
        let w = smoothstep(((level - leaning) / NEARER_BAND + 0.5).clamp(0.0, 1.0));
        self.out.lerp(self.out * angle.cos() - Vec3::Y * angle.sin(), w).normalize()
    }
}

/// A rough wall's holds start this far over its foot and stop this far
/// under its top, metres; each this share of its cell off the cell's middle
/// at most (two holds no nearer than 0.2 of a cell).
const ROUGH_BOTTOM: f32 = 0.3;
const ROUGH_TOP: f32 = 0.15;
const ROUGH_JITTER: f32 = 0.4;
/// An overhang's lean is eased in over this band about its crease, metres
/// of height (a hand's palm and a foot's sole turning with the face).
const CREASE_BAND: f32 = 0.2;
/// [`HoldWall::normal_nearest`] blends the two faces' over this much
/// difference in how near they are, metres.
const NEARER_BAND: f32 = 0.1;
/// A wall bent out past this, radians, leaves its top's lip with nothing
/// under it the feet reach ([`HoldWall::bent`]).
const BENT_FREE: f32 = std::f32::consts::FRAC_PI_4;
/// A point turns with an overhang's face ([`HoldWall::lean_at`]) within
/// the first of these of it, metres, and not at all beyond the second.
const FACE_NEAR: (f32, f32) = (0.15, 0.45);

/// A number in 0-1 drawn for a rough wall's cell `column`, `row` (the
/// `k`th), from `seed`: SplitMix64's finaliser over the four packed
/// together. Integer only, so the same on every machine.
fn cell_hash(seed: u32, column: i32, row: i32, k: u32) -> f32 {
    let mut x = (seed as u64) ^ ((column as u32 as u64) << 20) ^ ((row as u32 as u64) << 40) ^ ((k as u64) << 60);
    x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    x ^= x >> 31;
    (x >> 40) as f32 / (1u64 << 24) as f32
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
/// A hand's fingers turn with its forearm over a hold no farther than this
/// off straight up, radians (a sidepull): kept straight up, a forearm
/// reaching in from the side bent its wrist 1.7 rad.
const HOOK_TILT: f32 = 1.4;
const HOOK_INTO: f32 = 0.9;
/// Overhanging, this far: the body swung out under both hands on one jug,
/// a forearm lay along the leaning face, aside and down it, and held to
/// [`HOOK_TILT`] a wrist bent 0.63 rad sideways (turned back up the face
/// instead, 1.16).
const HOOK_TILT_LEANING: f32 = 2.2;
/// The arms are solved this many times, each hand's fingers turned toward
/// the forearm the last solve left (twice, the fingers were still settling:
/// most of a held wrist's sideways bend, up to 0.60 rad on a 40° face, was
/// that, 0.21 once more).
const HOOK_PASSES: usize = 3;
/// A forearm's sideways part under this share of it turns the fingers
/// aside only as far as it goes, so they pass through straight up as it
/// crosses over.
const HOOK_SOFT: f32 = 0.5;
/// Overhanging, a forearm's part along the face under this share of it is
/// eased toward the face's way up (from 0.4, a forearm a fifth along the
/// face swung its fingers from one side to the other, a wrist 5 cm).
const HOOK_FLAT: f32 = 0.7;
/// A move is not made if it takes an arm (moving or held) nearer pointing
/// against its elbow's pole than this (the cosine, about 8°): there the
/// elbow's way round the arm is undefined, and it flipped 34-44 cm in a
/// frame on a rough wall. No pole that depends on the arm's way alone is
/// defined everywhere (a hairy ball), so the climb keeps out of where it is
/// not. From this (about 18°) a move is chosen the less the nearer it
/// takes an arm (kept out of all of that, a climb stuck where only such a
/// move went on; chosen less from 25°, the grid's diagonal went aside).
/// Judged on an estimate ([`FreeClimb::against_pole`]): checked on the
/// pose itself at points along each move tried, no climb in the tests
/// changed by a bit, and climbing cost 5.3 times as much.
const ELBOW_CLEAR: f32 = 0.99;
const ELBOW_AVOID: f32 = 0.95;
/// Overhanging, kept out of this and chosen less from this: the face leans
/// over the arm, and an elbow pointing nowhere in particular (a held arm at
/// 0.96) went 2 cm into it. Swivelled out of the face instead about the
/// arm's line, there is no way round that does not flip somewhere: toward
/// the face's normal an elbow pointing straight at it flipped 30-50 cm,
/// toward its own side one reaching aside went on in.
const ELBOW_CLEAR_LEANING: f32 = 0.975;
const ELBOW_AVOID_LEANING: f32 = 0.92;
/// An arm already past the limit may go this much nearer still on a move.
/// A hand's move whose arm passes nearer its pole than the first takes
/// longer, up to this much longer again at the limit (the estimate reads a
/// little under the pose: 0.920 for an arm that reached 0.938).
const ELBOW_STAY: f32 = 0.005;
const ELBOW_SLOW_FROM: f32 = 0.85;
const ELBOW_SLOW: f32 = 1.0;
/// The moving limb's path and the hips are tried at this many points along
/// a move.
const CLEAR_SAMPLES: usize = 8;
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
/// Flying, each hand this far off the wall halfway, metres; turned out to
/// it eased in over this either side of where it starts.
const FLYING_OFF: f32 = 0.12;
const FLYING_EASE: f32 = 0.06;
/// A dyno's flight is sampled this many times for how far its arms bow
/// out ([`DynoArms::bow`]), and bowed this much farther.
const BOW_SAMPLES: usize = 10;
const BOW_MARGIN: f32 = 1.15;
/// A dyno lets go this share of the way from the sink to the catch at most;
/// under an overhang it is caught at most this far in from hanging at rest,
/// radians about the hold.
const RELEASE_AT: f32 = 0.35;
/// A dyno's flight lasts this long at least, seconds, and crosses no faster
/// than this, m/s.
const FLIGHT_LEAST: f32 = 0.2;
const FLIGHT_ACROSS: f32 = 2.5;
const CATCH_IN: f32 = 0.5;
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
/// The feet cut loose ([`Swing`]): the body swings under the hands as a
/// rod hung from one end, as long as from the hands to the feet, damped
/// this much of critically (a climber's core stops it: one swing out and
/// back keeps 28 % of it). The hands hold still for this many swings (a
/// whole one, out and back, is the period), the feet are brought back onto
/// holds after this many (by when it is down to 15 %), the swing is let
/// fade over its last this long, seconds, and is gone after this many.
const SWING_DAMPING: f32 = 0.2;
/// The body's own turn with the swing eased in over this long, seconds.
const SWING_TURN_IN: f32 = 0.3;
/// A move turns the body no faster than this on average, rad/s (the feet
/// brought back onto a 52° face, turning it 0.7 rad in a foot's 0.45 s,
/// changed a leg's step 6.5 cm).
const TILT_RATE: f32 = 0.8;
const SWING_HANDS: f32 = 0.5;
const SWING_FEET: f32 = 1.5;
const SWING_FADE: f32 = 0.5;
const SWING_END: f32 = 3.0;

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
    /// The head and the chest from the hips joint, the leaned trunk's pose
    /// frame.
    upper: [Vec3; 2],
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
            upper: [Bone::Head, Bone::Spine2].map(|bone| leaned[bone] - leaned[Bone::Hips]),
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
    /// a free foot hung under the hips), `t` seconds in; the hips and the
    /// body's tilt from and to.
    Moving { limb: usize, from: Vec3, to: Option<usize>, t: f32, seconds: f32, hips: (Vec3, Vec3), tilts: (f32, f32) },
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
    /// The body's tilt at the start and at the catch (hanging, none; under
    /// an overhang, caught in from it, as its line from the hold leans).
    tilt: f32,
    catch_tilt: f32,
    /// The arms in its flight ([`DynoArms`]).
    arms: Option<Box<DynoArms>>,
}

/// A dyno's arms in its flight: how they hold at the release and at the
/// catch (posed once, as it is planned).
#[derive(Debug, Clone, PartialEq)]
struct DynoArms {
    released: LocalPose,
    caught: LocalPose,
    /// How far each arm is turned out from the wall halfway through the
    /// flight, metres at the wrist out of the face, eased in and out as
    /// `sin(πs)`: as far as its path from the release's pose to the
    /// catch's, sampled, comes nearer the face than [`FLYING_OFF`] would
    /// have it. (Kept off only where it came too near, the hand came at a
    /// 52° face's underside at 2.4 m/s and was stopped in a frame or two,
    /// its step changing 8.9 cm.) Unbowed (`raw`), the path as sampled.
    bow: [f32; 2],
    raw: bool,
}

/// A dyno's hands and feet leave their holds over the drive's last this
/// long, seconds, carried up with their shoulders and hips, so they let go
/// going with the body: held to their holds to the release, the body passed
/// them at 3.4 m/s, the arms folding at 15-23 rad/s, and a forearm's step
/// changed 6-10 cm in the frame they let go (carried on turning so into the
/// flight, they overshot by over a radian; turning the elbow's pull down
/// smoothly through it, 6 cm).
const CARRY: f32 = 0.15;

impl DynoPlan {
    fn length(&self) -> f32 {
        SINK + DRIVE + self.flight
    }

    /// The body's tilt `t` seconds in: kept while the feet are on through
    /// the sink and the drive, let go in the flight (let go from the start,
    /// a held foot was pulled 22 cm off its hold under a 52° lean).
    fn tilt(&self, t: f32) -> f32 {
        self.tilt + (self.catch_tilt - self.tilt) * smoothstep(((t - SINK - DRIVE) / self.flight).clamp(0.0, 1.0))
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

/// The feet cut loose under an overhang: the body swinging under the
/// hands, a damped pendulum about the hands' middle, the wall's along its
/// axis, let go by a dyno's catch in from its rest. What it adds to the
/// hips and the body's tilt is its own: the climb goes on under it, and it
/// dies away.
#[derive(Debug, Clone, PartialEq)]
struct Swing {
    t: f32,
    axis: Vec3,
    /// The hips hanging at rest from the pivot.
    rest: Vec3,
    /// Where it starts from the rest, radians, and how fast it swings
    /// there, rad/s (positive: the hips out from the wall); the body's tilt
    /// there.
    angle: f32,
    spin: f32,
    tilt: f32,
    /// The undamped and the damped angular frequencies, rad/s.
    omega: f32,
    damped: f32,
    /// Fading out from this time over this long.
    fade: (f32, f32),
}

impl Swing {
    /// A swing's period, seconds.
    fn period(&self) -> f32 {
        std::f32::consts::TAU / self.damped
    }

    /// The angle from the rest now, radians.
    fn angle_now(&self) -> f32 {
        let decay = SWING_DAMPING * self.omega;
        let (sin, cos) = (self.damped * self.t).sin_cos();
        (-decay * self.t).exp() * (self.angle * cos + (self.spin + decay * self.angle) / self.damped * sin)
    }

    /// What it adds to the hips now, and to the body's tilt (fading).
    fn now(&self) -> (Vec3, f32) {
        let left = 1.0 - smoothstep(((self.t - self.fade.0) / self.fade.1).clamp(0.0, 1.0));
        let angle = self.angle_now() * left;
        // A body turned about the axis tips its top out by the opposite;
        // the body's own turn eased in from how it was caught (at once,
        // under a 52° lean the feet changed step 10 cm).
        let turning = smoothstep((self.t / SWING_TURN_IN).clamp(0.0, 1.0));
        (Quat::from_axis_angle(self.axis, angle) * self.rest - self.rest, self.tilt * (1.0 - turning) - angle * turning)
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
    /// Getting on: where it stood, and the pose it stood in (the standing
    /// pose unless told, [`Self::set_start`]).
    stood_at: Vec3,
    start: Option<Box<LocalPose>>,
    floor: f32,
    /// Each hand's own grip, if its fingers are known (for the hang it
    /// tops out into); taking the top's lip.
    grips: [Option<HandGrip>; 2],
    topping: Option<Box<Topping>>,
    /// How far the body tilts its top out from upright, radians (on an
    /// overhang, with the face between the feet and the hands; hanging
    /// free, none), as the hips are without the swing.
    tilt: f32,
    /// The feet cut loose, swinging.
    swing: Option<Box<Swing>>,
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
            start: None,
            floor: root.y,
            grips: [None; 2],
            topping: None,
            tilt: 0.0,
            swing: None,
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
                .filter(|(i, hold)| hold.kind.hand() && Some(*i) != taken && hold.at.y > shoulder.y && (climb.wrist_for(side, hold.at, Vec3::Y) - shoulder).length() <= ARM_REACH * climb.body.arms[side])
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
        climb.tilt = climb.tilt_for(&climb.limbs);
        Some(climb)
    }

    /// Where the root stands to get on `wall` coming from `from`: in front
    /// of it [`GET_ON_OFF`] out, level with `from` along it.
    pub fn spot(wall: &HoldWall, from: Vec3) -> Vec3 {
        let (place, _) = wall.place(from);
        (wall.face + wall.along() * place.x + wall.out * GET_ON_OFF).with_y(from.y)
    }

    /// Getting on from `pose` (the walker's as it was, on the same rig and
    /// in its frame): eased out of it, each hand swept to its hold from
    /// where it was. From the standing pose, a hand resting on the wall
    /// beside the body dropped 40 cm to the side and rose again, at
    /// 2.8 m/s.
    pub fn set_start(&mut self, pose: &LocalPose) {
        self.start = Some(Box::new(*pose));
    }

    /// The pose getting on starts from.
    fn start_pose(&self) -> &LocalPose {
        self.start.as_deref().unwrap_or(&self.body.stood)
    }

    /// Each hand placed so its own fingers hook its hold: their grips as
    /// the rig's fingers make them, in each hand's own frame (kept so, for
    /// a hang it hands over to, which turns them itself).
    pub fn set_grips(&mut self, grips: [Option<HandGrip>; 2]) {
        self.grips = grips;
        for (side, grip) in crate::character::anim::hand::bound_grips(grips, &self.rig).into_iter().enumerate() {
            if let Some(grip) = grip {
                self.body.grips[side] = grip;
            }
        }
    }

    /// The hand's world turn hooked over a hold on a face out along
    /// `normal`: the palm against the face, the fingers along `along` (in
    /// the face's plane) over it.
    fn hook_turn_on(&self, side: usize, along: Vec3, normal: Vec3) -> Quat {
        let grip = &self.body.grips[side];
        frame_turn(grip.along, grip.palm, along, -normal)
    }

    /// The face limb `limb` turns with at `at`: how far it leans, its
    /// normal and its way up. Moving to a hold, turned from its old hold's
    /// to its new one's over the move (by where it is, a foot crossing a
    /// roof's crease turned a quarter turn over 20 cm and changed its step
    /// 16 cm).
    fn limb_face(&self, limb: usize, at: Vec3) -> (f32, Vec3, Vec3) {
        let face = |at: Vec3| {
            let (normal, up) = self.wall.frame_at(at);
            (self.wall.lean_at(at), normal, up)
        };
        match &self.doing {
            Doing::Moving { limb: moving, from, to: Some(to), t, seconds, .. } if *moving == limb && self.wall.lean.is_some() => {
                let s = smoothstep((t / seconds).clamp(0.0, 1.0));
                let ((lean0, normal0, up0), (lean1, normal1, up1)) = (face(*from), face(self.wall.holds[*to].at));
                (lean0 + (lean1 - lean0) * s, normal0.lerp(normal1, s).normalize(), up0.lerp(up1, s).normalize())
            }
            _ => face(at),
        }
    }

    /// Which way a hand's fingers go over its hold with its forearm along
    /// `forearm` (the world): on along the forearm in the wall's plane, but
    /// no farther than [`HOOK_TILT`] off straight up (a sidepull's); and
    /// tipped into the wall as far as the forearm comes in toward it, up to
    /// [`HOOK_INTO`], the heel of the hand off the face (laid flat on it,
    /// a forearm coming in bent the wrist back 1.05 rad). (On an overhang,
    /// all of it in the face's own plane at the hold, out along `normal`,
    /// up it `up`.)
    fn hook_along_on(&self, forearm: Vec3, normal: Vec3, up: Vec3) -> Vec3 {
        let on = forearm - normal * forearm.dot(normal);
        // Overhanging, a forearm pointing nearly into the face has no way
        // along it: eased toward the face's way up as it comes to (at a 40°
        // crease, a held wrist hopped 9.5 cm round its hold).
        let flat = smoothstep((1.0 - on.length() / (HOOK_FLAT * forearm.length()).max(1.0e-6)).clamp(0.0, 1.0));
        let on = if self.wall.lean.is_some() { on + up * (HOOK_FLAT * forearm.length() * flat) } else { on };
        let across = on - up * on.dot(up);
        let most = if self.wall.lean.is_some() { HOOK_TILT_LEANING } else { HOOK_TILT };
        let tilt = across.length().atan2(on.dot(up)).clamp(0.0, most);
        // Sideways as far as the forearm goes sideways, not by its sign
        // alone: a forearm pointing down, passing straight down, swung the
        // fingers from one side to the other in a frame, and the held wrist
        // round its hold 7.6 cm (a rough wall).
        let aside = across / across.length().max(HOOK_SOFT * on.length()).max(1.0e-6);
        let level = (up * tilt.cos() + aside * tilt.sin()).normalize_or(up);
        let into = (-forearm.dot(normal)).atan2(on.length()).clamp(0.0, HOOK_INTO);
        level * into.cos() - normal * into.sin()
    }

    /// The wrist for hand `side` hooked on a hold at `at`, its fingers
    /// along `along` ([`Self::hook_along_on`]).
    fn wrist_for(&self, side: usize, at: Vec3, along: Vec3) -> Vec3 {
        self.wrist_on(side, at, along, self.wall.frame_at(at).0)
    }

    /// [`Self::wrist_for`] on a face out along `normal`.
    fn wrist_on(&self, side: usize, at: Vec3, along: Vec3, normal: Vec3) -> Vec3 {
        at - self.hook_turn_on(side, along, normal) * hook_lip(&self.body.grips[side])
    }

    /// The ankle for foot `side`, its ball on a hold at `at`, and its
    /// world rotation (facing the wall, toes down a little; on an overhang,
    /// turned with the face).
    fn ankle_for(&self, side: usize, at: Vec3) -> (Vec3, Quat) {
        self.ankle_leaning(side, at, self.wall.lean_at(at), self.wall.frame_at(at).0)
    }

    /// [`Self::ankle_for`] turned with a face leaning `lean`, out of it
    /// along `normal`.
    fn ankle_leaning(&self, side: usize, at: Vec3, lean: f32, normal: Vec3) -> (Vec3, Quat) {
        let axis = self.turn * self.rig.left();
        let face = if lean == 0.0 { Quat::IDENTITY } else { Quat::from_axis_angle(self.wall.along(), -lean) };
        let attitude = face * Quat::from_axis_angle(axis, TOES_DOWN) * self.turn * self.body.attitudes[side];
        let ball_to_ankle = attitude * (self.turn * self.body.attitudes[side]).inverse() * (self.turn * self.body.balls[side]);
        (at + ball_to_ankle + normal * ANKLE_OUT, attitude)
    }

    /// How far a dyno's feet have left their holds `t` seconds in, 0-1
    /// (from when they are carried, [`CARRY`]).
    fn feet_left(plan: &DynoPlan, t: f32) -> f32 {
        smoothstep(((t - (SINK + DRIVE - CARRY)) / (CARRY + FEET_LEAVE * plan.flight)).clamp(0.0, 1.0))
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
        // Overhanging, braced: up the face as an upright wall is climbed,
        // off it along the tilted body (so on the leaning face, all of it is
        // the upright climb turned).
        if !feet.is_empty() && self.wall.lean.is_some() {
            let rise = |points: &[Vec3]| points.iter().map(|&p| self.wall.rise(p)).sum::<f32>() / points.len() as f32;
            let (hands, feet) = (rise(&hands), rise(&feet));
            let rise = (feet + FEET_BELOW).clamp(hands - HANDS_ABOVE.1, hands - HANDS_ABOVE.0);
            let tilt = self.tilt_for(limbs);
            let hips = self.wall.on_face(along.x, rise) + self.tilted(self.wall.out, tilt) * HIPS_OUT;
            return self.clear_of_the_face(hips, tilt, HIPS_OUT);
        }
        // Hanging free, under the hands.
        let (height, out) = if feet.is_empty() {
            (hand_y - HANG_BELOW, HIPS_OUT_FREE + self.wall.level_out(mean(&hands)))
        } else {
            ((mean(&feet).y + FEET_BELOW).clamp(hand_y - HANDS_ABOVE.1, hand_y - HANDS_ABOVE.0), HIPS_OUT)
        };
        self.wall.face.with_y(0.0) + self.wall.along() * along.x + self.wall.out * out + Vec3::Y * height
    }

    /// `hips` (the body tilted `tilt`) moved out from the wall as far as
    /// keeps the head, the chest and the shoulders as far off the face as
    /// they are off a flat face with the hips `off` it: nowhere on a flat
    /// face, upright or leaning; at an overhang's crease, the face above
    /// leaning out over the body. Placed off the upright face there, the
    /// head went 8 cm into a 52° lean. Moved between the two faces' ways
    /// out, which takes it off either alike (straight out, it went nowhere
    /// off a roof).
    fn clear_of_the_face(&self, hips: Vec3, tilt: f32, off: f32) -> Vec3 {
        let Some((_, lean)) = self.wall.lean else { return hips };
        let points = self.body.upper.iter().chain(self.body.shoulders.iter());
        let short = points
            .map(|&offset| {
                let world = self.turn * offset;
                // On a flat face the body is tilted with: off by `off` and
                // its own offset across the body.
                let flat = off + world.dot(self.wall.out);
                flat - self.wall.place(hips + self.tilted(world, tilt)).1
            })
            .fold(0.0, f32::max);
        let between = self.wall.out * (0.5 * lean).cos() - Vec3::Y * (0.5 * lean).sin();
        hips + between * (short / (0.5 * lean).cos())
    }

    /// How far the body tilts its top out held by `limbs`: as the face
    /// between the feet and the hands leans (none upright, and none hanging
    /// free).
    fn tilt_for(&self, limbs: &[Option<usize>; 4]) -> f32 {
        let middle = |of: [usize; 2]| {
            let held: Vec<Vec3> = of.iter().filter_map(|&l| limbs[l]).map(|i| self.wall.holds[i].at).collect();
            (!held.is_empty()).then(|| held.iter().copied().sum::<Vec3>() / held.len() as f32)
        };
        match (middle([LH, RH]), middle([LF, RF])) {
            (Some(hands), Some(feet)) if self.wall.lean.is_some() => (self.wall.level_out(hands) - self.wall.level_out(feet)).atan2((hands.y - feet.y).max(0.0)),
            _ => 0.0,
        }
    }

    /// `v`, a vector of the upright body (the world), with the body tilted
    /// its top out `tilt` about its hips.
    fn tilted(&self, v: Vec3, tilt: f32) -> Vec3 {
        if tilt == 0.0 { v } else { Quat::from_axis_angle(self.wall.along(), -tilt) * v }
    }

    /// Shoulder `side` with the hips at `hips`, the body tilted `tilt`.
    fn shoulder(&self, side: usize, hips: Vec3, tilt: f32) -> Vec3 {
        hips + self.tilted(self.turn * self.body.shoulders[side], tilt)
    }

    /// Whether hand `side` reaches a hold at `at` from hips at `hips`, the
    /// body tilted `tilt`.
    fn hand_reaches(&self, side: usize, at: Vec3, hips: Vec3, tilt: f32) -> bool {
        let shoulder = self.shoulder(side, hips, tilt);
        (self.wrist_for(side, at, self.wall.frame_at(at).1) - shoulder).length() <= ARM_REACH * self.body.arms[side]
    }

    /// How nearly hand `side` on a hold at `at`, the hips at `hips`, points
    /// its arm against its elbow's pole: the cosine between the arm and the
    /// pole's reverse (the pole as the pose makes it, the clavicle's lift
    /// aside).
    fn against_pole(&self, side: usize, at: Vec3, hips: Vec3, tilt: f32) -> f32 {
        let shoulder = self.shoulder(side, hips, tilt);
        let wrist = self.wrist_for(side, at, self.wall.frame_at(at).1);
        let reach = (wrist - shoulder).normalize_or_zero();
        let up = self.tilted(Vec3::Y, tilt);
        let low = smoothstep(((LOCK_OFF.0 - (wrist - shoulder).dot(up)) / LOCK_OFF.1).clamp(0.0, 1.0));
        let pole = self.tilted((self.turn * (self.rig.left() * (SIGN[side] * 0.7) - self.rig.forward() * 0.5) - Vec3::Y * (0.2 + 0.8 * low)).normalize(), tilt);
        -reach.dot(pole)
    }

    /// How near pointing against its elbow's pole an arm may be taken, and
    /// from where that is chosen less (the cosines; overhanging, tighter).
    /// An arm already past it (both hands caught on one hold) may stay as
    /// far: held to the limit, a climber caught past it under a 52° lean had
    /// no move left at all.
    fn elbow_limits(&self) -> (f32, f32) {
        let (clear, avoid) = if self.wall.lean.is_some() { (ELBOW_CLEAR_LEANING, ELBOW_AVOID_LEANING) } else { (ELBOW_CLEAR, ELBOW_AVOID) };
        let now = [LH, RH].iter().filter(|&&hand| self.limbs[hand].is_some()).map(|&hand| self.against_pole(hand, self.limb_at(hand), self.hips, self.tilt)).fold(-1.0, f32::max);
        if now + ELBOW_STAY > clear { (now + ELBOW_STAY, avoid) } else { (clear, avoid) }
    }

    /// A limb `s` of the way through a move from `from` to `to`, coming
    /// `arc` off the face on its way (out of the face where it is: straight
    /// out, a hand moving along a roof went along it).
    fn on_the_way(&self, from: Vec3, to: Vec3, s: f32, arc: f32) -> Vec3 {
        let at = from.lerp(to, smoothstep(s));
        at + self.wall.frame_at(at).0 * (arc * (std::f32::consts::PI * s).sin().powi(2))
    }

    /// How nearly moving `limb` to hold `to` from `limbs` takes either arm
    /// against its pole ([`Self::against_pole`]), at worst: the moving hand
    /// along its path, a held one as the hips move under it; tried at
    /// [`CLEAR_SAMPLES`] points past where it is now.
    fn worst_against_pole(&self, limb: usize, to: usize, limbs: &[Option<usize>; 4]) -> f32 {
        let mut with = *limbs;
        with[limb] = Some(to);
        let (start, end) = (self.hips, self.hips_for(&with));
        let tilts = (self.tilt, self.tilt_for(&with));
        let from = self.limb_at(limb);
        let goal = self.wall.holds[to].at;
        (1..=CLEAR_SAMPLES)
            .flat_map(|k| {
                let s = k as f32 / CLEAR_SAMPLES as f32;
                let hips = start.lerp(end, smoothstep(s));
                let tilt = tilts.0 + (tilts.1 - tilts.0) * smoothstep(s);
                [LH, RH].into_iter().filter_map(move |hand| {
                    let at = if hand == limb {
                        Some(self.on_the_way(from, goal, s, HAND_ARC))
                    } else {
                        limbs[hand].map(|_| self.limb_at(hand))
                    };
                    at.map(|at| self.against_pole(hand, at, hips, tilt))
                })
            })
            .fold(-1.0, f32::max)
    }

    /// Whether foot `side` reaches a hold at `at` from hips at `hips`, the
    /// body tilted `tilt`: not too far, nor too high under them.
    fn foot_reaches(&self, side: usize, at: Vec3, hips: Vec3, tilt: f32) -> bool {
        let socket = hips + self.tilted(self.turn * self.body.sockets[side], tilt);
        let off = (self.ankle_for(side, at).0 - socket).length() / self.body.legs[side];
        // Under them along the tilted body.
        (LEG_FOLD..=LEG_REACH).contains(&off) && (hips - at).dot(self.tilted(Vec3::Y, tilt)) >= FOOT_UNDER_HIPS
    }

    /// The best foothold for `foot` with the other limbs on `limbs`: under
    /// the hands, its side of the other foot, nearest where a foot hangs
    /// [`FEET_BELOW`] under the hips; `None` with none in reach.
    fn foothold_for(&self, foot: usize, limbs: &[Option<usize>; 4]) -> Option<usize> {
        let side = foot - LF;
        // Heights up the face ([`HoldWall::rise`]).
        let rise = |at: Vec3| self.wall.rise(at);
        let lowest_hand = [LH, RH].iter().filter_map(|&l| limbs[l]).map(|i| rise(self.wall.holds[i].at)).fold(f32::MAX, f32::min);
        let other = limbs[LF + 1 - side].map(|i| self.wall.place(self.wall.holds[i].at).0.x);
        let mut trial = *limbs;
        trial[foot] = None;
        let hips_free = self.hips_for(&trial);
        let wanted = rise(hips_free) - FEET_BELOW + 0.25;
        // Under its own hip (with nothing else to keep it its side, the left
        // foot took a hold right of the hips and the right one went 0.75 m
        // out).
        let under_hip = self.wall.place(hips_free).0.x + SIGN[side] * FOOT_ASIDE;
        self.wall
            .holds
            .iter()
            .enumerate()
            .filter(|(i, hold)| !limbs.contains(&Some(*i)) && rise(hold.at) <= lowest_hand - FEET_UNDER_HANDS && hold.at.y > self.floor + 0.1)
            .filter(|(_, hold)| other.is_none_or(|o| SIGN[side] * (self.wall.place(hold.at).0.x - o) >= -0.05))
            .filter(|(i, _)| {
                let mut with = *limbs;
                with[foot] = Some(*i);
                let (hips, tilt) = (self.hips_for(&with), self.tilt_for(&with));
                self.foot_reaches(side, self.wall.holds[*i].at, hips, tilt)
                    && [LH, RH].iter().all(|&h| with[h].is_none_or(|j| self.hand_reaches(h, self.wall.holds[j].at, hips, tilt)))
                    && self.worst_against_pole(foot, *i, limbs) < self.elbow_limits().0
            })
            .min_by(|(_, a), (_, b)| {
                let cost = |hold: &Hold| (rise(hold.at) - wanted).abs() + (self.wall.place(hold.at).0.x - under_hip).abs();
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
        self
            .wall
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
                // The feet under the hands (up the face).
                let rise = |j: usize| self.wall.rise(self.wall.holds[j].at);
                let lowest_hand = [LH, RH].iter().filter_map(|&l| with[l]).map(rise).fold(f32::MAX, f32::min);
                if [LF, RF].iter().filter_map(|&l| with[l]).any(|j| rise(j) > lowest_hand - FEET_UNDER_HANDS) {
                    return None;
                }
                let (hips, tilt) = (self.hips_for(&with), self.tilt_for(&with));
                let reached = [LH, RH].iter().all(|&h| with[h].is_none_or(|j| self.hand_reaches(h, self.wall.holds[j].at, hips, tilt)))
                    && [LF, RF].iter().all(|&f| with[f].is_none_or(|j| self.foot_reaches(f - LF, self.wall.holds[j].at, hips, tilt)));
                if !reached {
                    return None;
                }
                // An arm kept off pointing against its elbow's pole (for
                // holds in reach only: for every hold, climbing cost 2.8
                // times as much).
                let against = self.worst_against_pole(limb, i, limbs);
                // Each limb its own side of the hips: a foot under its hip,
                // a hand over its shoulder.
                let own = self.wall.place(hips).0.x + SIGN[side] * if hand { HAND_ASIDE } else { FOOT_ASIDE };
                let (clear, avoid) = self.elbow_limits();
                let avoid = ((against - avoid) / (clear - avoid)).max(0.0);
                (against < clear).then_some((i, (progress - step).abs() + 0.5 * across + 0.5 * (place.x - own).abs() + avoid))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(i, _)| i)
    }

    /// A dyno up toward `way` (mostly up): the nearest hand hold above the
    /// hands, up and aside, no more than [`DYNO_ABOVE`] over the higher, up
    /// the face (none in a move's reach, or it would be climbed to), that
    /// the feet can get back onto holds from, hanging on it; with none,
    /// the nearest.
    fn dyno_for(&self, way: ClimbWay) -> Option<usize> {
        if way.y < 0.7 * way.length() {
            return None;
        }
        let hands = [LH, RH].map(|l| self.limbs[l].map(|i| self.wall.holds[i].at));
        let top = hands.iter().flatten().map(|&at| self.wall.rise(at)).fold(f32::MIN, f32::max);
        let middle = hands.iter().flatten().copied().sum::<Vec3>() / 2.0;
        let mut near: Vec<(usize, f32)> = self
            .wall
            .holds
            .iter()
            .enumerate()
            .filter(|(i, hold)| !self.limbs.contains(&Some(*i)) && hold.kind.hand() && (top + LEAST_PROGRESS..=top + DYNO_ABOVE).contains(&self.wall.rise(hold.at)))
            .filter(|(_, hold)| (self.wall.place(hold.at).0.x - self.wall.place(middle).0.x).abs() <= DYNO_ASIDE)
            .map(|(i, hold)| (i, self.wall.rise(hold.at) - top + (self.wall.place(hold.at).0.x - self.wall.place(middle).0.x).abs()))
            .collect();
        near.sort_by(|a, b| a.1.total_cmp(&b.1));
        // Hanging from it after, the feet find a hold again (by aside
        // alone, under a 52° lean one jumped a row too far, 1 m, and hung
        // with no foothold low enough; to the nearest, it hung where the
        // only one low enough was across the gap below, out of reach).
        let feet_back = |to: usize| {
            let mut hung = self.clone();
            (hung.limbs[LH], hung.limbs[RH], hung.limbs[LF], hung.limbs[RF]) = (Some(to), Some(to), None, None);
            (hung.hips, hung.tilt, hung.swing) = (hung.hips_for(&hung.limbs), 0.0, None);
            [LF, RF].iter().any(|&foot| hung.foothold_for(foot, &hung.limbs).is_some())
        };
        near.iter().find(|&&(i, _)| feet_back(i)).or(near.first()).map(|&(i, _)| i)
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
        // Under an overhang, caught on the hang's circle about the hold as
        // far in as the body comes up from (no farther than `CATCH_IN`), to
        // swing out from there: caught at the hang's rest, plumb under the
        // hold, the flight left the wall at 4 m/s under a 52° lean.
        let (catch, catch_tilt) = if self.wall.lean.is_some() {
            let axis = self.wall.along();
            let rest = catch - hold;
            let rest_in = rest - axis * rest.dot(axis);
            let from = sunk - hold;
            let from_in = from - axis * from.dot(axis);
            let angle = axis.dot(rest_in.cross(from_in)).atan2(rest_in.dot(from_in)).clamp(-CATCH_IN, 0.0);
            // Turned about the hold with the body: its top tips the other
            // way.
            (hold + Quat::from_axis_angle(axis, angle) * rest, -angle)
        } else {
            (catch, 0.0)
        };
        // Released a third of the way up, the rest flown; no farther than
        // the feet still reach from their holds (a third of the way to a
        // catch out under an overhang stretched the legs past their length,
        // a held foot 22 cm off its hold).
        let reached = |hips: Vec3| {
            [LF, RF].iter().all(|&foot| {
                self.limbs[foot].is_none_or(|i| {
                    let side = foot - LF;
                    let socket = hips + self.tilted(self.turn * self.body.sockets[side], self.tilt);
                    (self.ankle_for(side, self.wall.holds[i].at).0 - socket).length() <= LEG_REACH * self.body.legs[side]
                })
            })
        };
        let mut share = RELEASE_AT;
        if !reached(sunk + (catch - sunk) * share) {
            let (mut low, mut high) = (0.0, share);
            for _ in 0..12 {
                let mid = 0.5 * (low + high);
                if reached(sunk + (catch - sunk) * mid) { low = mid } else { high = mid }
            }
            share = low;
        }
        let release = sunk + (catch - sunk) * share;
        // Flown to the catch at the top of its flight, if it is above; no
        // shorter than [`FLIGHT_LEAST`] nor faster across than
        // [`FLIGHT_ACROSS`], falling onto it if it must (along a roof the
        // catch hangs below the release, and flown up to it, a 0.1 s flight
        // crossed a metre at 10 m/s and swung the body up through the roof).
        let rise = catch.y - release.y;
        let across = (catch - release).with_y(0.0);
        let flight = (2.0 * rise.max(0.0) / GRAVITY).sqrt().max(FLIGHT_LEAST).max(across.length() / FLIGHT_ACROSS);
        let velocity = across / flight + Vec3::Y * ((rise + 0.5 * GRAVITY * flight * flight) / flight);
        // Where each hand is, matched ones aside of their hold (from the
        // hold's middle, a matched hand jumped 9 cm as it let go).
        let let_go = [LH, RH].map(|l| self.limb_at(l));
        let mut plan = DynoPlan { start, sunk, release, velocity, catch, flight, hands, let_go, tilt: self.tilt, catch_tilt, arms: None };
        plan.arms = Some(Box::new(self.dyno_arms(to, &plan)));
        plan
    }

    /// The arms in a dyno's flight to hold `to` on `plan` ([`DynoArms`]).
    fn dyno_arms(&self, to: usize, plan: &DynoPlan) -> DynoArms {
        // Held at the catch.
        let mut held = self.clone();
        (held.doing, held.hips, held.tilt, held.swing) = (Doing::Holding, plan.catch, plan.catch_tilt, None);
        (held.limbs[LH], held.limbs[RH], held.limbs[LF], held.limbs[RF]) = (Some(to), Some(to), None, None);
        let caught = held.pose();
        // Posed `t` seconds into the dyno (with `arms`, if flying).
        let posed_at = |t: f32, arms: Option<DynoArms>| {
            let mut plan = plan.clone();
            plan.arms = arms.map(Box::new);
            let mut at = self.clone();
            (at.hips, at.tilt, at.swing) = (plan.hips(t), plan.tilt(t), None);
            at.doing = Doing::Dyno { to, t, plan: Box::new(plan) };
            (at.climbing_posed().0, at.hips)
        };
        // As the drive leaves them (the hands carried up with it).
        let let_go = SINK + DRIVE;
        let released = posed_at(let_go - 1.0e-4, None).0;
        // How far out each arm is bowed: sampled along the flight unbowed.
        let mut arms = DynoArms { released, caught, bow: [0.0; 2], raw: true };
        let mut bow = [0.0f32; 2];
        for k in 1..BOW_SAMPLES {
            let s = k as f32 / BOW_SAMPLES as f32;
            let (pose, hips) = posed_at(let_go + plan.flight * s, Some(arms.clone()));
            let root = hips - self.turn * self.body.hips;
            let at = forward_kinematics_on(&pose, &self.rig);
            // Its middle only: by its ends a hand is at its hold.
            let eased = smoothstep(s);
            let bowing = (std::f32::consts::PI * eased).sin();
            if bowing < 0.5 {
                continue;
            }
            for (side, chain) in ARMS.iter().enumerate() {
                let lacks = FLYING_OFF * bowing - self.wall.place(root + self.turn * at[chain.wrist]).1;
                bow[side] = bow[side].max(lacks / bowing);
            }
        }
        arms.bow = bow.map(|b| b * BOW_MARGIN);
        arms.raw = false;
        arms
    }

    /// Moves it on `dt` seconds, asked to climb `way` (or held).
    pub fn advance(&mut self, way: Option<ClimbWay>, dt: f32) {
        if self.way.map(Self::order) != way.map(Self::order) {
            self.next = 0;
        }
        self.way = way;
        self.tick(dt);
        let mut dt = dt;
        // A move ending mid-frame, the next starts with the rest of it.
        for _ in 0..3 {
            dt = self.step(dt);
            if dt <= 0.0 {
                break;
            }
        }
    }

    /// Moves the swing on `dt` seconds, gone once faded.
    fn tick(&mut self, dt: f32) {
        if let Some(swing) = self.swing.as_mut() {
            swing.t += dt;
            if swing.t >= swing.fade.0 + swing.fade.1 {
                self.swing = None;
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
            Doing::Moving { limb, to, t, seconds, hips, tilts, .. } => {
                *t += dt;
                let (limb, to, seconds, hips, tilts) = (*limb, *to, *seconds, *hips, *tilts);
                let s = smoothstep((*t / seconds).clamp(0.0, 1.0));
                self.hips = hips.0.lerp(hips.1, s);
                self.tilt = tilts.0 + (tilts.1 - tilts.0) * s;
                if *t >= seconds {
                    let left = *t - seconds;
                    self.limbs[limb] = to;
                    self.hips = hips.1;
                    self.tilt = tilts.1;
                    self.doing = Doing::Holding;
                    return left;
                }
                0.0
            }
            Doing::Dyno { to, t, plan } => {
                *t += dt;
                let (to, length) = (*to, plan.length());
                self.hips = plan.hips(*t);
                self.tilt = plan.tilt(*t);
                if *t >= length {
                    let left = *t - length;
                    self.limbs[LH] = Some(to);
                    self.limbs[RH] = Some(to);
                    self.limbs[LF] = None;
                    self.limbs[RF] = None;
                    self.hips = plan.catch;
                    self.tilt = plan.catch_tilt;
                    // Caught under an overhang, in from hanging: it swings
                    // out under the hold, the feet cut loose.
                    if self.wall.lean.is_some() {
                        let velocity = plan.velocity - Vec3::Y * (GRAVITY * plan.flight);
                        self.swing_from(velocity);
                    }
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
                // Cut loose, how many swings in.
                let swings = self.swing.as_ref().map(|swing| swing.t / swing.period());
                // Free feet find holds again before anything else (cut
                // loose, once the swing has died down, the swing fading out
                // over the foot's move).
                if swings.is_none_or(|n| n >= SWING_FEET) {
                    for foot in [LF, RF] {
                        if self.limbs[foot].is_none()
                            && let Some(hold) = self.foothold_for(foot, &self.limbs)
                        {
                            self.start_move(foot, Some(hold));
                            return dt;
                        }
                    }
                }
                if swings.is_some_and(|n| n < SWING_HANDS) {
                    return 0.0;
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
                    && [LH, RH].iter().all(|&l| self.limbs[l].is_some_and(|i| self.wall.rise(top.a) - self.wall.rise(self.wall.holds[i].at) <= TOP_REACH))
                {
                    let (pose, root) = (self.pose(), self.root());
                    let square = Hanging::square(&top, self.rig.forward());
                    let hang = Hanging::caught(&top, &[], self.hips_now(), Vec3::ZERO, &pose, root, square, 0.0, self.grips, &self.body.stood, &self.rig);
                    self.topping = Some(Box::new(Topping { t: 0.0, hang, from: pose, from_root: root }));
                    self.doing = Doing::ToppingOut;
                    return 0.0;
                }
                let order = Self::order(way);
                for k in 0..4 {
                    let limb = order[(self.next + k) % 4];
                    if limb >= LF && self.limbs[limb].is_none() && swings.is_some_and(|n| n < SWING_FEET) {
                        continue;
                    }
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

    /// Starts moving `limb` to hold `to`. A free foot to a hold, swinging:
    /// the swing fades out over its move.
    /// A move turning the body far (feet brought back onto a steep
    /// overhang) takes as long as turning it at [`TILT_RATE`] does.
    fn start_move(&mut self, limb: usize, to: Option<usize>) {
        let from = self.limb_at(limb);
        let mut with = self.limbs;
        with[limb] = to;
        let tilts = (self.tilt, self.tilt_for(&with));
        let turned = (tilts.1 - self.tilt_now()).abs();
        // Overhanging, a hand whose arm passes near pointing against its
        // elbow's pole moves slower, its elbow sweeping round (at full pace,
        // at 0.94 under a 40° lean an elbow's step changed 6.6 cm; on an
        // upright grid, slowed too, the diagonal climb went the wrong way).
        let slow = match to {
            Some(to) if limb < LF && self.wall.lean.is_some() => {
                let clear = self.elbow_limits().0;
                1.0 + ELBOW_SLOW * smoothstep(((self.worst_against_pole(limb, to, &self.limbs) - ELBOW_SLOW_FROM) / (clear - ELBOW_SLOW_FROM)).clamp(0.0, 1.0))
            }
            _ => 1.0,
        };
        let seconds = ((if limb < LF { HAND_MOVE } else { FOOT_MOVE }) * slow).max(turned / TILT_RATE);
        if limb >= LF
            && to.is_some()
            && let Some(swing) = self.swing.as_mut()
            && swing.t + seconds < swing.fade.0 + swing.fade.1
        {
            swing.fade = (swing.t, seconds);
        }
        self.doing = Doing::Moving { limb, from, to, t: 0.0, seconds, hips: (self.hips, self.hips_for(&with)), tilts };
    }

    /// Caught hanging under an overhang, the hips (on the hang's circle
    /// about the hands, tilted as their line leans) moving at `velocity`:
    /// they swing on under the hands from there ([`Swing`]), about the
    /// hands' middle; the hips as they hang at rest beneath them.
    fn swing_from(&mut self, velocity: Vec3) {
        let pivot = 0.5 * (self.limb_at(LH) + self.limb_at(RH));
        let (from, rest) = (self.hips - pivot, self.hips_for(&self.limbs) - pivot);
        let axis = self.wall.along();
        let (rest_in, from_in) = (rest - axis * rest.dot(axis), from - axis * from.dot(axis));
        let angle = axis.dot(rest_in.cross(from_in)).atan2(rest_in.dot(from_in));
        // Turning about the pivot: the hips' speed across their arm.
        let spin = axis.dot(from_in.cross(velocity)) / from_in.length_squared().max(1.0e-4);
        // A rod from the hands to the feet, hung from one end.
        let omega = (1.5 * GRAVITY / (rest.length() + FEET_BELOW)).sqrt();
        let damped = omega * (1.0 - SWING_DAMPING * SWING_DAMPING).sqrt();
        let end = SWING_END * std::f32::consts::TAU / damped;
        self.swing = Some(Box::new(Swing { t: 0.0, axis, rest, angle, spin, tilt: self.tilt, omega, damped, fade: (end - SWING_FADE, SWING_FADE) }));
        self.hips = pivot + rest;
        self.tilt = 0.0;
    }

    /// Where limb `limb` is now (its hold, or where a free foot hangs); both
    /// hands on one hold, [`MATCHED`] either side of it.
    fn limb_at(&self, limb: usize) -> Vec3 {
        match self.limbs[limb] {
            Some(i) if limb < LF && self.limbs[1 - limb] == Some(i) => self.wall.holds[i].at + self.wall.along() * (SIGN[limb] * MATCHED),
            Some(i) => self.wall.holds[i].at,
            None => self.hanging_foot(limb % 2),
        }
    }

    /// Where foot `side`'s ball hangs free: under the hips, with the body.
    fn hanging_foot(&self, side: usize) -> Vec3 {
        let (swung, tilt) = self.swing.as_ref().map_or((Vec3::ZERO, 0.0), |swing| swing.now());
        self.hips + swung + self.tilted(-Vec3::Y * FEET_BELOW + self.turn * self.rig.left() * (SIGN[side] * 0.12), self.tilt + tilt)
    }

    /// Each limb's target now (the world): a hand's hold or its way to one,
    /// a foot's ball on its hold or hanging; and whether it is held.
    fn targets(&self) -> [(Vec3, bool); 4] {
        let mut targets = [0, 1, 2, 3].map(|limb| (self.limb_at(limb), self.limbs[limb].is_some()));
        match &self.doing {
            Doing::Moving { limb, from, to, t, seconds, .. } => {
                let s = (t / seconds).clamp(0.0, 1.0);
                let end = to.map_or_else(|| self.hanging_foot(limb % 2), |i| self.wall.holds[i].at);
                let arc = if *limb < LF { HAND_ARC } else { FOOT_ARC };
                targets[*limb] = (self.on_the_way(*from, end, s, arc), false);
                // A hand leaving a hold it matched on: the other slides to
                // the hold's middle as it goes (at once, it jumped 9 cm).
                if *limb < LF
                    && let Some(i) = self.limbs[*limb]
                    && self.limbs[1 - *limb] == Some(i)
                {
                    let other = 1 - *limb;
                    targets[other].0 = self.wall.holds[i].at + self.wall.along() * (SIGN[other] * MATCHED * (1.0 - smoothstep(s)));
                }
            }
            Doing::Dyno { t, plan, .. } => {
                let released = *t >= SINK + DRIVE;
                // Leaving their holds carried with the shoulders and the
                // hips ([`CARRY`]), no nearer the face; the feet carried on
                // so as they leave.
                let from = SINK + DRIVE - CARRY;
                if *t > from {
                    let w = smoothstep(((t - from) / CARRY).min(1.0));
                    let (hips, tilt) = (plan.hips(*t), plan.tilt(*t));
                    let socket = |side: usize, hips: Vec3, tilt: f32| hips + self.tilted(self.turn * self.body.sockets[side], tilt);
                    for (limb, target) in targets.iter_mut().enumerate() {
                        let moved = if limb < LF {
                            if released {
                                continue;
                            }
                            self.shoulder(limb, hips, tilt) - self.shoulder(limb, plan.hips(from), plan.tilt(from))
                        } else {
                            socket(limb - LF, hips, tilt) - socket(limb - LF, plan.hips(from), plan.tilt(from))
                        };
                        let (held, at) = (target.0, target.0 + moved * w);
                        // A hand no nearer the face (a foot goes with the
                        // hips, off it: held off a leaning face as it came
                        // near, one changed its step 10 cm).
                        let short = if limb < LF { self.wall.place(held).1 - self.wall.place(at).1 } else { 0.0 };
                        *target = (at + self.wall.normal_nearest(at) * short.max(0.0), false);
                    }
                }
                for (side, limb) in [LH, RH].into_iter().enumerate() {
                    if released {
                        // Swept round the shoulder, the elbow bent on the
                        // way (straight there, the arm passed through
                        // straight and its elbow flipped 39 cm in a frame).
                        let flown = ((t - SINK - DRIVE) / plan.flight).clamp(0.0, 1.0);
                        let s = smoothstep(flown);
                        let shoulder = |hips: Vec3, tilt: f32| self.shoulder(side, hips, tilt);
                        let from = plan.let_go[side] - shoulder(plan.release, plan.tilt(SINK + DRIVE));
                        let to = plan.hands[side] - shoulder(plan.catch, plan.catch_tilt);
                        let swept = Quat::IDENTITY.slerp(Quat::from_rotation_arc(from.normalize(), to.normalize()), s) * from.normalize();
                        let reach = from.length() + (to.length() - from.length()) * s - DYNO_BEND * (std::f32::consts::PI * s).sin();
                        // From where it let go, still, into going with the
                        // body (with it at once, the hand went from still to
                        // the body's 3.3 m/s in a frame, an elbow 12 cm off
                        // its path).
                        let with_body = smoothstep((flown / HANDS_FOLLOW).clamp(0.0, 1.0));
                        let at = plan.let_go[side].lerp(shoulder(self.hips, self.tilt) + swept * reach, with_body);
                        // Off the wall until the catch: an arc round the
                        // shoulder between two holds on it bulges through it
                        // (2.3 cm), and the elbow behind the hand came 2.8 cm
                        // into it.
                        let (_, out) = self.wall.place(at);
                        let off = FLYING_OFF * (std::f32::consts::PI * s).sin();
                        targets[limb] = (at + self.wall.normal_nearest(at) * (off - out).max(0.0), false);
                    }
                }
                if *t > from {
                    // Off their holds over a moment (at once, a foot moved
                    // 91 cm in a frame), from when they are carried.
                    // Within the flight however short (a 0.2 s one caught
                    // them nine tenths of the way, and they jumped 10 cm;
                    // left from the release, a knee changed its step 7 cm
                    // in a 0.14 s flight).
                    let s = Self::feet_left(plan, *t);
                    for limb in [LF, RF] {
                        targets[limb] = (targets[limb].0.lerp(self.hanging_foot(limb % 2), s), false);
                    }
                }
            }
            _ => {}
        }
        targets
    }

    /// The hips in the world now.
    fn hips_now(&self) -> Vec3 {
        let swung = self.swing.as_ref().map_or(Vec3::ZERO, |swing| swing.now().0);
        match self.doing {
            Doing::GettingOn { t } => {
                let from = self.stood_at + self.turn * self.body.hips;
                from.lerp(self.hips, smoothstep((t / GET_ON).clamp(0.0, 1.0)))
            }
            _ => self.hips + swung,
        }
    }

    /// The body's tilt now, its top out, radians.
    fn tilt_now(&self) -> f32 {
        let swung = self.swing.as_ref().map_or(0.0, |swing| swing.now().1);
        match self.doing {
            Doing::GettingOn { t } => self.tilt * smoothstep((t / GET_ON).clamp(0.0, 1.0)),
            _ => self.tilt + swung,
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
        self.climbing_posed().0
    }

    /// The climb's own pose ([`Self::pose`] but taking the lip), and which
    /// way each hand's fingers go over its hold ([`Self::hook_along_on`]).
    fn climbing_posed(&self) -> (LocalPose, [Vec3; 2]) {
        let rig = &self.rig;
        let hips = self.hips_now();
        let root = hips - self.turn * self.body.hips;
        let back = self.turn.inverse();
        let getting_on = match self.doing {
            Doing::GettingOn { t } => Some(smoothstep((t / GET_ON).clamp(0.0, 1.0))),
            _ => None,
        };
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, LEAN * getting_on.unwrap_or(1.0), (0.0, 0.0));
        // Tilted with an overhang or a swing, the whole body about its hips.
        let tilt = self.tilt_now();
        let tilting = if tilt != 0.0 { Quat::from_axis_angle(back * self.wall.along(), -tilt) } else { Quat::IDENTITY };
        if tilt != 0.0 {
            pose.rotations[Bone::Hips] = delta_after_world_turn(&pose, rig, Bone::Hips, tilting);
        }
        let targets = self.targets();
        // The legs: each ankle to its ball's place; getting on, from where
        // it stood, no farther from its hip than its reach.
        for (side, &(_, _, ankle_bone, _)) in LEGS.iter().enumerate() {
            let (ball, _) = targets[LF + side];
            // Leaving its hold in a dyno, turned from as on it to as hanging
            // (by where it is, a foot carried up past the crease turned
            // with the face in a few frames, its step changing 10 cm).
            let (mut ankle, mut attitude) = match (&self.doing, self.limbs[LF + side]) {
                (Doing::Dyno { t, plan, .. }, Some(hold)) if *t > SINK + DRIVE - CARRY => {
                    let (hold, hanging) = (self.wall.holds[hold].at, self.hanging_foot(side));
                    let s = Self::feet_left(plan, *t);
                    let lean = self.wall.lean_at(hold) + (self.wall.lean_at(hanging) - self.wall.lean_at(hold)) * s;
                    self.ankle_leaning(side, ball, lean, self.wall.frame_at(hold).0.lerp(self.wall.frame_at(hanging).0, s).normalize())
                }
                _ => {
                    let (lean, normal, _) = self.limb_face(LF + side, ball);
                    self.ankle_leaning(side, ball, lean, normal)
                }
            };
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
        // The arms: each hand hooked over its hold, its fingers on along
        // its forearm ([`Self::hook_along_on`]); getting on, raised forward
        // round the shoulder to it. First hooked fingers up, then turned
        // toward the forearm the arm solved to: fingers up, an arm reaching
        // in from the side bent its wrist 1.7 rad sideways.
        let unarmed = pose;
        let started = getting_on.map(|_| forward_kinematics_on(self.start_pose(), rig));
        let faces = [0, 1].map(|side| self.limb_face(side, targets[side].0));
        let mut alongs = faces.map(|(_, _, up)| up);
        for pass in 0..HOOK_PASSES {
        pose = unarmed;
        let at = forward_kinematics_on(&pose, rig);
        let wrists = [0, 1].map(|side| {
            let hooked = self.wrist_on(side, targets[side].0, alongs[side], faces[side].1);
            match getting_on {
                Some(w) => {
                    let shoulder = at[ARMS[side].shoulder];
                    let started = started.as_ref().map_or(self.body.hands[side], |at| at[ARMS[side].wrist]);
                    let (from, to) = (started - shoulder, back * (hooked - root) - shoulder);
                    let swept = Quat::IDENTITY.slerp(Quat::from_rotation_arc(from.normalize(), to.normalize()), w) * from.normalize();
                    let reach = from.length() + (to.length() - from.length()) * w - 0.12 * (std::f32::consts::PI * w).sin();
                    // No nearer the face than the hooked wrist ends (round
                    // the shoulder to a hold out to the side, the sweep went
                    // 2.8 cm into the wall on a rough wall).
                    let at = root + self.turn * (shoulder + swept * reach);
                    let short = self.wall.place(hooked).1 - self.wall.place(at).1;
                    at + self.wall.normal_nearest(at) * short.max(0.0)
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
        let mut next = alongs;
        for side in 0..2 {
            // The elbow out and back from the wall, a little down, as a
            // hang's: the hands kept over the shoulders (a hand moving up
            // past its shoulder flipped its elbow 29 cm in a frame). Mostly
            // down, an arm reaching up to the wall had its elbow's pole
            // turned toward the wall and the elbow went 5 cm into it. A hand
            // come down to its shoulder, pulled down ([`LOCK_OFF`]).
            // Tilted with the body (upright, the overhang flipped elbows
            // 70 cm in a frame).
            let low = smoothstep(((LOCK_OFF.0 - (targets_pose[side] - at[ARMS[side].shoulder]).dot(tilting * Vec3::Y)) / LOCK_OFF.1).clamp(0.0, 1.0));
            let pole = tilting * (rig.left() * (SIGN[side] * 0.7) - rig.forward() * 0.5 - Vec3::Y * (0.2 + 0.8 * low)).normalize();
            let (elbow, wrist) = solve_arm_toward_from(&mut pose, &at, ARMS[side], targets_pose[side], pole, rig);
            let (_, normal, up) = faces[side];
            turn_hand(&mut pose, rig, ARMS[side], self.body.hand_binds[side], back * self.hook_turn_on(side, alongs[side], normal), getting_on.unwrap_or(1.0), (wrist - elbow).normalize_or_zero());
            if pass + 1 < HOOK_PASSES {
                next[side] = self.hook_along_on(self.turn * (wrist - elbow), normal, up);
            }
        }
        alongs = next;
        }
        if let Some(w) = getting_on {
            let s = smoothstep((w * GET_ON / GET_ON_EASE).clamp(0.0, 1.0));
            let start = self.start_pose();
            for bone in Bone::ALL {
                pose.rotations[bone] = start.rotations[bone].slerp(pose.rotations[bone], s);
            }
            pose.root_translation = start.root_translation.lerp(pose.root_translation, s);
        }
        // In a dyno's flight, the arms turn from how they held at the
        // release to how they hold at the catch (solved toward hands flying
        // on a path, the arm passed through straight and an elbow flipped
        // 50 cm in a frame).
        if let Doing::Dyno { to, t, plan } = &self.doing
            && *t >= SINK + DRIVE
        {
            let flown = (t - SINK - DRIVE).min(plan.flight);
            let s = smoothstep(flown / plan.flight);
            let planned;
            let arms = match plan.arms.as_deref() {
                Some(arms) => arms,
                None => {
                    planned = self.dyno_arms(*to, plan);
                    &planned
                }
            };
            for (chain, clavicle) in ARMS.iter().zip(CLAVICLES) {
                for bone in [clavicle, chain.shoulder, chain.elbow, chain.wrist] {
                    pose.rotations[bone] = arms.released.rotations[bone].slerp(arms.caught.rotations[bone], s);
                }
            }
            // Each hand kept off the wall, the arm turned out about its
            // shoulder: none at either end (the body rising under turning
            // arms carried a hand 5.6 cm into the wall).
            let clear = FLYING_OFF * (std::f32::consts::PI * s).sin();
            for (side, chain) in ARMS.iter().enumerate() {
                if arms.raw {
                    break;
                }
                let at = forward_kinematics_on(&pose, rig);
                let wrist = root + self.turn * at[chain.wrist];
                // Bowed out as far as the flight's path needs
                // ([`DynoArms::bow`]), smoothly; anything left eased in over
                // a band about where it starts, so it starts smoothly (from
                // at once, a hand's step changed 5.7 cm as it came within
                // its clearance; a hand coming at the face at 2.4 m/s, held
                // off it exactly, changed its step 8.9 cm).
                let bowed = arms.bow[side] * (std::f32::consts::PI * s).sin();
                let short = clear - self.wall.place(wrist).1 - bowed;
                let short = if short.abs() < FLYING_EASE { (short + FLYING_EASE).powi(2) / (4.0 * FLYING_EASE) } else { short.max(0.0) };
                let by = bowed + short;
                if by <= 0.0 {
                    continue;
                }
                // Out of the nearer face (out level, from a leaning face's
                // underside, a hand cleared only cos 52° of what it lacked
                // and stayed 9 mm in), the elbow bent the way it is bent
                // (turned out about the shoulder instead, an arm reaching up
                // into the face hardly moved its hand off it).
                let target = back * (wrist + self.wall.normal_nearest(wrist) * by - root);
                let (shoulder, elbow) = (at[chain.shoulder], at[chain.elbow]);
                let line = (target - shoulder).normalize_or_zero();
                let bend = (elbow - shoulder) - line * (elbow - shoulder).dot(line);
                if bend.length() > 1.0e-3 {
                    solve_arm_toward_from(&mut pose, &at, *chain, target, bend.normalize(), rig);
                }
            }
        }
        (pose, alongs)
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
                    later.tick(lead);
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
        let alongs = if self.topping.is_some() { [Vec3::Y; 2] } else { self.climbing_posed().1 };
        [0, 1, 2, 3].map(|limb| {
            let (at, held) = targets[limb];
            held.then(|| if limb < LF { self.wrist_for(limb, at, alongs[limb]) } else { self.ankle_for(limb - LF, at).0 })
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
        /// hips; the most a joint's step changes in a frame (a dyno's too).
        into: f32,
        fastest: f32,
        kink: f32,
        /// The fewest limbs held outside a dyno's flight; the longest a
        /// dyno's flight holds none, seconds.
        fewest: usize,
        flight: f32,
        moves: usize,
        dynos: usize,
        /// The most a held hand bends sideways at the wrist, and how far its
        /// wrist flexes, least and most (negative bent back), radians (the
        /// rig's own fingers' grips, `hand::wrist_bend`).
        bend: f32,
        flex: (f32, f32),
        /// How often the feet cut loose and swung; the most the body tilted
        /// its top out, radians.
        swings: usize,
        tilt: f32,
    }

    fn world(climb: &FreeClimb) -> BoneSet<Vec3> {
        let (pose, root, turn) = (climb.pose(), climb.root(), Quat::from_rotation_y(climb.facing()));
        let at = forward_kinematics_on(&pose, &climb.rig);
        BoneSet::from_fn(|bone| root + turn * at[bone])
    }

    /// Climbs `climb` asked `way` for `seconds`, measuring.
    fn run(climb: &mut FreeClimb, way: Option<ClimbWay>, seconds: f32, m: &mut Climbed, frames: &mut Vec<BoneSet<Vec3>>) {
        run_until(climb, way, seconds, m, frames, &|_| false);
    }

    /// [`run`], stopping early once `stop` holds.
    fn run_until(climb: &mut FreeClimb, way: Option<ClimbWay>, seconds: f32, m: &mut Climbed, frames: &mut Vec<BoneSet<Vec3>>, stop: &dyn Fn(&FreeClimb) -> bool) {
        let mut t = 0.0;
        let mut flying = 0.0;
        while t < seconds && !climb.stepped_off() && !climb.topped_out() && !stop(climb) {
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
            if climb.swing.as_ref().is_some_and(|swing| swing.t < 0.5 * DT) {
                m.swings += 1;
            }
            m.tilt = m.tilt.max(climb.tilt_now());
            // Taking the lip, the climb hands over to a hang, the feet off
            // their holds (`it_steps_off_at_the_bottom_and_tops_out_into_a_
            // hang` bounds that hand-over).
            if climb.topping.is_some() {
                frames.push(now);
                continue;
            }
            let (a, b) = (frames[frames.len() - 2], frames[frames.len() - 1]);
            m.fastest = m.fastest.max(Bone::ALL.iter().map(|&bone| ((now[bone] - now[Bone::Hips]) - (b[bone] - b[Bone::Hips])).length() / DT).fold(0.0, f32::max));
            let kink = Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(0.0, f32::max);
            m.kink = m.kink.max(kink);
            let places = climb.held_places();
            if !matches!(climb.doing, Doing::GettingOn { .. }) {
                // A hand moving, the other three held.
                if matches!(climb.doing, Doing::Moving { limb, .. } if limb < LF) {
                    m.fewest = m.fewest.min(places.iter().filter(|p| p.is_some()).count());
                }
                let bends = crate::character::anim::hand::wrist_bend(&climb.pose(), &climb.rig, &crate::character::anim::hand::puppet_grips());
                for (side, chain) in ARMS.iter().enumerate() {
                    if let Some(wrist) = places[side] {
                        m.hand_off = m.hand_off.max((now[chain.wrist] - wrist).length());
                        m.bend = m.bend.max(bends[side].0);
                        m.flex = (m.flex.0.min(bends[side].1), m.flex.1.max(bends[side].1));
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
        climb.set_grips(crate::character::anim::hand::puppet_grips());
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
        // A held hand's fingers on along its forearm: kept straight up, a
        // forearm reaching in from the side bent a wrist 1.7 rad.
        assert!(m.bend < 0.6 && m.flex.0 > -0.9 && m.flex.1 < 1.3, "a held hand bent {:.2} rad sideways, flexed {:.2}..{:.2}", m.bend, m.flex.0, m.flex.1);
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
    /// after; never no hand held for under 0.15 s nor over 0.4; no step
    /// changing more than 4.5 cm through it (holding the hands and feet to
    /// their holds to the release, 8.5 cm).
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
        assert!(m.kink < 0.045, "a joint's step changed {:.4} m in a frame", m.kink);
    }

    /// Got on from a walker standing with a hand resting on a wall beside
    /// it: the first frame is the pose it stood in, and the hand goes from
    /// where it rested to its hold, never back down to the standing pose's
    /// on the way. Eased out of the standing pose instead, it jumped there
    /// in the first frame (52 cm).
    #[test]
    fn getting_on_starts_from_the_pose_it_stood_in() {
        let (stood, rig) = real_stood();
        let wall = HoldWall::grid(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 9, 18, 0.4, 0.3, 0.35);
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        climb.set_grips(crate::character::anim::hand::puppet_grips());
        let turn = Quat::from_rotation_y(climb.facing());
        let mut start = stood;
        let beside = crate::character::anim::parkour::wallhand::Beside { side: 0, distance: 0.4, out: -(turn * rig.left()) };
        crate::character::anim::parkour::wallhand::rest_hand(&mut start, Vec3::ZERO, climb.facing(), &rig, &beside, 1.0);
        climb.set_start(&start);
        let at = forward_kinematics_on(&start, &rig);
        let was = BoneSet::from_fn(|bone| turn * at[bone]);
        let hand = Bone::LeftHand;
        let standing = turn * forward_kinematics_on(&stood, &rig)[hand];
        let mut frames = vec![was];
        while matches!(climb.doing, Doing::GettingOn { .. }) {
            climb.advance(None, DT);
            frames.push(world(&climb));
        }
        let first = Bone::ALL.iter().map(|&bone| (frames[1][bone] - was[bone]).length()).fold(0.0, f32::max);
        // The most a joint's step changes in a frame (standing still before).
        frames.insert(0, was);
        let kink = frames.windows(3).map(|w| Bone::ALL.iter().map(|&bone| (w[2][bone] - 2.0 * w[1][bone] + w[0][bone]).length()).fold(0.0, f32::max)).fold(0.0, f32::max);
        // The hand's nearest to where the standing pose has it, against where
        // it started from.
        let nearest = frames.iter().map(|f| (f[hand] - standing).length()).fold(f32::MAX, f32::min);
        let rested = (was[hand] - standing).length();
        eprintln!("first frame {first:.4}, a step changing {kink:.4} at most, hand nearest the standing pose's {nearest:.3} (rested {rested:.3} off it)");
        assert!(rested > 0.3, "the hand rested only {rested:.3} from where standing has it");
        assert!(first < 0.01, "a joint moved {first:.3} m in the first frame");
        assert!(nearest > 0.5 * rested, "the hand went back to within {nearest:.3} of the standing pose's");
        assert!(kink < 0.02, "a joint's step changed {kink:.3} m in a frame");
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

    /// A wall upright to 2.3 m and overhanging above it, leant out 0.45 rad
    /// (26°), its holds 0.3 m apart but for a gap of `gap` over the crease,
    /// its top a ledge.
    fn overhang(gap: f32) -> HoldWall {
        overhang_at(gap, 0.45)
    }

    fn overhang_at(gap: f32, lean: f32) -> HoldWall {
        upright_then(gap).leaning(2.3, lean)
    }

    /// The same wall bent out at 2.3 m `lean` from upright, up to a flat
    /// roof, its holds as far apart up the face as they were.
    fn bent_at(gap: f32, lean: f32) -> HoldWall {
        upright_then(gap).bent(2.3, lean)
    }

    /// An upright wall of holds 0.3 m apart but for a gap of `gap` 0.15 m
    /// under 2.3 m, its top a ledge.
    fn upright_then(gap: f32) -> HoldWall {
        let face = Vec3::new(0.0, 0.0, -0.6);
        let mut wall = HoldWall::grid(face, Vec3::Z, 9, 7, 0.4, 0.3, 0.35);
        let top_row = wall.holds.iter().map(|h| h.at.y).fold(f32::MIN, f32::max);
        let upper = HoldWall::grid(face, Vec3::Z, 9, 6, 0.4, 0.3, top_row + gap);
        wall.holds.extend(upper.holds);
        let top = top_row + gap + 6.0 * 0.3;
        wall.top = Some(Ledge::wall(face, Vec3::Z, 4.0, top, 1.0));
        wall
    }

    /// Overhangs leant 26° to 52° over gaps of 0.3-0.6 m in their holds at
    /// the crease: each climbed to the top (by dynos and swings where it
    /// must), held limbs on their holds, nothing into either face, no step
    /// changing more than 4.5 cm (a dyno's too), a held wrist bent no more
    /// than 0.6 rad sideways (the upright grid's bound). Before, a 0.6 m gap
    /// ended hanging with no foothold, 40° pulled a held foot 15 cm off, 52°
    /// put the head 3.7 cm into the face, a dyno changed a step 10 cm, and a
    /// wrist bent 0.70 rad.
    #[test]
    fn steep_overhangs_and_wide_gaps_are_climbed() {
        let (stood, rig) = real_stood();
        let mut faults = Vec::new();
        for (gap, lean) in [(0.6f32, 0.45f32), (0.3, 0.7), (0.6, 0.7), (0.3, 0.9), (0.6, 0.9)] {
            let name = format!("{gap} m gap, {:.0}°", lean.to_degrees());
            let wall = overhang_at(gap, lean);
            let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
            climb.set_grips(crate::character::anim::hand::puppet_grips());
            let mut m = Climbed { fewest: 4, ..Default::default() };
            let mut frames = vec![world(&climb), world(&climb)];
            run(&mut climb, Some(Vec2::Y), 40.0, &mut m, &mut frames);
            eprintln!("{name}: topped out {}, {m:?}", climb.topped_out());
            if !climb.topped_out() {
                faults.push(format!("{name}: stuck, the hips at {:.2}", climb.hips.y));
            }
            if m.hand_off > 1.0e-3 || m.foot_off > 0.01 || m.into > 0.005 || m.kink > 0.045 || m.bend > 0.6 {
                faults.push(format!("{name}: {m:?}"));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }

    /// Steeper still, the wall bent over at 2.3 m (its holds as far apart up
    /// the face as below), 65°, 78° and a flat roof, over a gap of 0.3 m at
    /// the crease: each climbed out to its top's lip and taken into a hang,
    /// held as the overhangs are (held limbs on their holds, nothing into
    /// either face, no step changing more than 4.5 cm, a held wrist bent no
    /// more than 0.6 rad); under the roof, the body flat under it with the
    /// feet on its holds. Before, measured by height rather than up the
    /// face, a roof's holds were all as high and none was progress up it.
    /// (Over a 0.6 m gap the roof is climbed too, but near its lip a held
    /// elbow goes 1.7 cm into it and a moving one flips: see the note.)
    #[test]
    fn steeper_overhangs_and_roofs_are_climbed() {
        let (stood, rig) = real_stood();
        let mut faults = Vec::new();
        let roof = std::f32::consts::FRAC_PI_2;
        for (gap, lean) in [(0.3f32, 65.0f32.to_radians()), (0.3, 78.0f32.to_radians()), (0.3, roof)] {
            let name = format!("{gap} m gap, {:.0}°", lean.to_degrees());
            let wall = bent_at(gap, lean);
            let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
            climb.set_grips(crate::character::anim::hand::puppet_grips());
            let mut m = Climbed { fewest: 4, ..Default::default() };
            let mut frames = vec![world(&climb), world(&climb)];
            run(&mut climb, Some(Vec2::Y), 40.0, &mut m, &mut frames);
            eprintln!("{name}: topped out {}, {m:?}", climb.topped_out());
            if !climb.topped_out() {
                faults.push(format!("{name}: stuck, the hips at {:.2}", climb.hips));
            }
            if m.hand_off > 1.0e-3 || m.foot_off > 0.01 || m.into > 0.005 || m.kink > 0.045 || m.bend > 0.6 {
                faults.push(format!("{name}: {m:?}"));
            }
            if lean == roof && (m.tilt - roof).abs() > 0.01 {
                faults.push(format!("{name}: the body tilted {:.3} at most, not flat under the roof", m.tilt));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }

    /// An overhang leant 26° with holds all the way: climbed to the top
    /// with the feet on, the body tilted with the face (all of it on the
    /// leaning face, the upright climb turned): held limbs on their holds,
    /// nothing into either face, no joint whipping round, no step changing
    /// more than the upright grid's own.
    #[test]
    fn an_overhang_is_climbed_with_the_feet_on_its_holds() {
        let (stood, rig) = real_stood();
        let wall = overhang(0.3);
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        climb.set_grips(crate::character::anim::hand::puppet_grips());
        let mut m = Climbed { fewest: 4, ..Default::default() };
        let mut frames = vec![world(&climb), world(&climb)];
        run(&mut climb, Some(Vec2::Y), 40.0, &mut m, &mut frames);
        eprintln!("{m:?}");
        assert!(climb.topped_out(), "stuck, the hips at {:.2}", climb.hips.y);
        assert!(m.dynos == 0 && m.swings == 0, "{} dynos, {} swings", m.dynos, m.swings);
        assert!((m.tilt - 0.45).abs() < 0.01, "the body tilted {:.3}, not with the face", m.tilt);
        assert!(m.hand_off < 1.0e-3 && m.foot_off < 0.01, "a held hand {:.4}, a foot {:.4} off its hold", m.hand_off, m.foot_off);
        assert!(m.into < 0.005, "a joint {:.4} m into the wall", m.into);
        assert!(m.fastest < 14.0 && m.kink < 0.03, "{m:?}");
    }

    /// Under the same overhang with a gap of 0.45 m in its holds over the
    /// crease: jumped across, caught hanging in from plumb under the hold,
    /// the feet cut loose and the body swings out under the hands. Held
    /// still, the swing is a damped pendulum about the hands: the feet's
    /// line from them half a period between turning points, as a rod as
    /// long as hands to toes hung from one end swings (measured on the posed
    /// skeleton, not the swing's own numbers), each turn about half the
    /// last. Then the feet are brought back up onto the face's holds and it
    /// climbs on to the top; nothing goes into either face.
    #[test]
    fn under_an_overhang_the_feet_cut_loose_swing_and_come_back() {
        let (stood, rig) = real_stood();
        let wall = overhang(0.45);
        let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig).expect("got on");
        climb.set_grips(crate::character::anim::hand::puppet_grips());
        let mut m = Climbed { fewest: 4, ..Default::default() };
        let mut frames = vec![world(&climb), world(&climb)];
        run_until(&mut climb, Some(Vec2::Y), 30.0, &mut m, &mut frames, &|climb| climb.swing.is_some());
        assert!(climb.swing.is_some() && climb.limbs[LF].is_none() && climb.limbs[RF].is_none(), "never cut loose: {:?}", climb.limbs);
        // Held still: the feet's line from the hands, out from straight
        // down about the wall's along, until a foot takes a hold.
        let mut swung: Vec<(f32, f32, f32)> = Vec::new();
        let mut t = 0.0;
        while climb.limbs[LF].is_none() && climb.limbs[RF].is_none() && !climb.is_moving() && t < 6.0 {
            run(&mut climb, None, DT, &mut m, &mut frames);
            t += DT;
            let now = frames[frames.len() - 1];
            let hands = 0.5 * (now[Bone::LeftHand] + now[Bone::RightHand]);
            let toes = 0.5 * (now[Bone::LeftToeBase] + now[Bone::RightToeBase]);
            let line = toes - hands;
            swung.push((t, line.dot(wall.out).atan2(-line.y), line.length()));
        }
        let turns: Vec<(f32, f32)> = swung.windows(3).filter(|w| (w[1].1 - w[0].1) * (w[2].1 - w[1].1) < 0.0).map(|w| (w[1].0, w[1].1)).collect();
        assert!(turns.len() >= 3, "only {} turning points in {t:.2} s", turns.len());
        let length = swung.iter().map(|s| s.2).sum::<f32>() / swung.len() as f32;
        let half = std::f32::consts::PI / ((1.5 * GRAVITY / length).sqrt() * (1.0 - SWING_DAMPING * SWING_DAMPING).sqrt());
        let halves = [turns[1].0 - turns[0].0, turns[2].0 - turns[1].0];
        let decay = (turns[2].1 - turns[1].1) / (turns[0].1 - turns[1].1);
        eprintln!("turns {turns:?}, hands to toes {length:.3} m: half periods {halves:?}, a rod's {half:.3}; each turn {decay:.3} of the last");
        assert!((turns[0].1 - turns[1].1).abs() > 0.2, "swung only {:.3} rad", (turns[0].1 - turns[1].1).abs());
        assert!(halves.iter().all(|h| (h / half - 1.0).abs() < 0.1), "half periods {halves:?}, not a rod's {half:.3}");
        assert!((0.4..0.65).contains(&decay), "each turn {decay:.3} of the last");
        // On up: the feet back on holds, to the top.
        run_until(&mut climb, Some(Vec2::Y), 30.0, &mut m, &mut frames, &|climb| climb.limbs[LF].is_some() && climb.limbs[RF].is_some());
        let feet_back = climb.limbs[LF].is_some() && climb.limbs[RF].is_some();
        run(&mut climb, Some(Vec2::Y), 30.0, &mut m, &mut frames);
        eprintln!("{m:?}");
        assert!(feet_back, "the feet never came back onto holds");
        assert!(climb.topped_out(), "stuck, the hips at {:.2}", climb.hips.y);
        assert!(m.hand_off < 1.0e-3 && m.foot_off < 0.01, "a held hand {:.4}, a foot {:.4} off its hold", m.hand_off, m.foot_off);
        assert!(m.into < 0.005, "a joint {:.4} m into the wall", m.into);
        assert!(m.fastest < 14.0 && m.kink < 0.045, "{m:?}");
    }

    /// A rough wall grows the same holds from the same seed, and others
    /// from another; all on the patch, none nearer another than a fifth of
    /// a cell; the rougher, the more holds and the more of them for hands.
    #[test]
    fn a_rough_wall_grows_the_same_holds_from_the_same_seed() {
        let face = Vec3::new(1.0, 0.0, -0.6);
        let wall = |roughness: f32, seed: u32| HoldWall::rough(face, Vec3::Z, 3.0, 6.0, roughness, seed);
        assert_eq!(wall(0.5, 7), wall(0.5, 7), "the same seed grew other holds");
        assert_ne!(wall(0.5, 7).holds, wall(0.5, 8).holds, "another seed grew the same holds");
        for roughness in [0.0f32, 0.5, 1.0] {
            let w = wall(roughness, 7);
            let cell = 0.3 * (1.0 - 0.35 * roughness);
            for (i, hold) in w.holds.iter().enumerate() {
                let (at, out) = w.place(hold.at);
                assert!(at.x.abs() <= 1.5 && (0.0..=6.0).contains(&at.y) && out.abs() < 1.0e-5, "a hold off the patch at {at}, {out} out");
                let nearest = w.holds.iter().enumerate().filter(|&(j, _)| j != i).map(|(_, other)| (other.at - hold.at).length()).fold(f32::MAX, f32::min);
                assert!(nearest >= 0.2 * cell - 1.0e-4, "two holds {nearest:.3} apart at roughness {roughness}");
            }
        }
        let count = |roughness: f32| {
            let w = wall(roughness, 7);
            (w.holds.len(), w.holds.iter().filter(|h| h.kind.hand()).count())
        };
        let ((smooth, smooth_hands), (rough, rough_hands)) = (count(0.0), count(1.0));
        assert!(rough > 2 * smooth && rough_hands * smooth > smooth_hands * rough, "{smooth} holds ({smooth_hands} for hands) smooth, {rough} ({rough_hands}) rough");
    }

    /// Rough walls, 5 m, several seeds: got on from standing and climbed up
    /// to the top into a hang; twice, the same climb (deterministic). Rough
    /// enough to climb hold to hold (0.8 and up), as on a climbing wall:
    /// held limbs on their holds, nothing in the wall, no joint whipping
    /// round, no dyno, no step changing more than a moving arm's elbow
    /// sweeping round over a hold placed anyhow (1.8-4.1 cm, over several
    /// frames, where the grid's regular holds keep it under 3). Sparser
    /// (0.3, 0.6), it still gets up, the hands within a centimetre of their
    /// holds, by dynos (whose own feet and arms are bounded by none of
    /// these: see the note).
    #[test]
    fn rough_walls_are_climbed_to_the_top() {
        let (stood, rig) = real_stood();
        let mut faults = Vec::new();
        for roughness in [0.3f32, 0.6, 0.8, 1.0] {
            let sparse = roughness < 0.7;
            for seed in 1..=2u32 {
                let name = format!("roughness {roughness}, seed {seed}");
                let wall = HoldWall::rough(Vec3::new(0.0, 0.0, -0.6), Vec3::Z, 2.4, 5.0, roughness, seed);
                let climbed = |m: &mut Climbed| {
                    let mut climb = FreeClimb::get_on(&wall, Vec3::ZERO, &stood, &rig)?;
                    let mut frames = vec![world(&climb), world(&climb)];
                    run(&mut climb, Some(Vec2::Y), 40.0, m, &mut frames);
                    Some((climb.topped_out(), climb.hips))
                };
                let mut m = Climbed { fewest: 4, ..Default::default() };
                // A sparse wall may have no two hand holds over the
                // shoulders to get on by.
                let Some((topped, hips)) = climbed(&mut m) else {
                    if !sparse {
                        faults.push(format!("{name}: never got on"));
                    }
                    continue;
                };
                // Climbed again (once a roughness: a climb is slow to run).
                let again = if seed == 1 { climbed(&mut Climbed { fewest: 4, ..Default::default() }).map(|(_, hips)| hips) } else { Some(hips) };
                eprintln!("{name}: {} holds, topped out {topped}, hips at {:.2}, {m:?}", wall.holds.len(), hips.y);
                if !topped {
                    faults.push(format!("{name}: stuck, the hips at {:.2}", hips.y));
                }
                if again != Some(hips) {
                    faults.push(format!("{name}: climbed again, ended at {again:?}, not {hips}"));
                }
                if m.hand_off > if sparse { 0.01 } else { 1.0e-3 } {
                    faults.push(format!("{name}: a held hand {:.4} off its hold", m.hand_off));
                }
                if !sparse && (m.dynos > 0 || m.foot_off > 0.01 || m.into > 0.005 || m.fastest > 14.0 || m.kink > 0.05) {
                    faults.push(format!("{name}: {m:?}"));
                }
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }
}
