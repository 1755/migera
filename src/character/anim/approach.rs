//! Walking up to a chair and turning round to sit on it.
//!
//! A person sits on a chair by standing in front of it with their back to
//! it, so the walk has to end on a spot ([`stand_spot`]) facing away from the
//! chair. Healthy adults turn the last half circle in ~2.5 steps and ~1.5 s
//! (median, Robinson et al. 2018, the timed 180° turn test), not by
//! pivoting on the spot.
//!
//! So the walk ends on a small turning circle beside that spot: straight
//! to where a line from the walker touches the circle, then round it at a
//! slow walk until facing away from the chair. Of the two circles (turning
//! left or right) the shorter way is taken: a Dubins path, the circle's
//! radius set by the slow walk's speed and a person's turning rate.
//!
//! # Landing the stop on the spot
//!
//! A walk cannot stop anywhere: told to stop, it walks on to a footfall and
//! ends with a last step (`transition`), so where it can stand comes in
//! half strides, 0.39 m apart at the turning pace. Stopped at the nearest,
//! it stood up to 0.29 m off the spot. A person closing on a target
//! adjusts their steps over the last few to land on it (Lee, Lishman and
//! Thomson 1982, long jumpers' run-ups). So over its last
//! [`FIT_WITHIN`] the walk paces itself ([`fitted_pace`]): the speed whose
//! stride puts a footfall where a stop from it ends on the spot. Whatever
//! is left, the seat makes up (`sitting::Seat`).
//!
//! # The chair in the way
//!
//! The turn ends [`TURN_AHEAD`] in front of the spot: a circle ending on
//! it dips a radius behind it, and the body passed 15 cm past the seat's
//! front edge. A straight that would cross the chair goes round it, corner
//! by corner ([`Chair::standard`] gives its footprint).
//!
//! Positions are on the floor plane (`y` ignored). Headings are the walking
//! direction's yaw ([`heading_of`]), not the facing's: the two differ by
//! the rig's own forward, which the caller converts at the boundary.

use std::f32::consts::{PI, TAU};

use bevy::math::{Vec2, Vec3};

use super::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use super::sitting;
use crate::character::skeleton::Bone;

/// A chair to sit on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Chair {
    /// The floor point under where the seated hips go: the seat's middle,
    /// a little in front of the backrest.
    pub seat: Vec3,
    /// The way a person sitting on it faces, horizontal.
    pub forward: Vec3,
    /// Its seat's height, metres.
    pub height: f32,
    /// What it stands on, to walk round: its width and depth (along
    /// [`Chair::forward`]), backrest included, metres.
    pub size: Vec2,
    /// How far in front of [`Chair::seat`] the middle of that is, metres.
    pub ahead: f32,
}

impl Chair {
    /// A standard chair (`sitting::CHAIR_HEIGHT`): a 0.46 m seat, 0.44 m
    /// deep, reaching 0.30 m in front of where the seated hips go, a backrest
    /// 0.04 m thick behind them.
    pub fn standard(seat: Vec3, forward: Vec3) -> Self {
        Self { seat, forward: forward.normalize_or_zero(), height: sitting::CHAIR_HEIGHT, size: Vec2::new(0.46, 0.48), ahead: 0.06 }
    }

    /// Where `at` is in the chair's frame: (across, along its forward)
    /// from the middle of what it stands on.
    fn local(&self, at: Vec3) -> Vec2 {
        let forward = Vec3::new(self.forward.x, 0.0, self.forward.z).normalize_or_zero();
        let left = Vec3::Y.cross(forward);
        let off = at - (self.seat + forward * self.ahead);
        Vec2::new(off.dot(left), off.dot(forward))
    }

    /// The world point at (across, along) in the chair's frame.
    fn world(&self, local: Vec2) -> Vec3 {
        let forward = Vec3::new(self.forward.x, 0.0, self.forward.z).normalize_or_zero();
        let left = Vec3::Y.cross(forward);
        let middle = self.seat + forward * self.ahead;
        Vec3::new(middle.x, 0.0, middle.z) + left * local.x + forward * local.y
    }

    /// Where along the walk from `from` to `to` (0 to 1) it first comes
    /// within `margin` of the chair, if it does.
    fn met(&self, from: Vec3, to: Vec3, margin: f32) -> Option<f32> {
        let half = self.size * 0.5 + Vec2::splat(margin);
        let (a, b) = (self.local(from), self.local(to));
        let d = b - a;
        // The slabs, each axis's entry and exit.
        let (mut enter, mut leave) = (0.0_f32, 1.0_f32);
        for axis in 0..2 {
            let (start, step, bound) = (a[axis], d[axis], half[axis]);
            if step.abs() < 1.0e-9 {
                if start.abs() > bound {
                    return None;
                }
                continue;
            }
            let (t0, t1) = ((-bound - start) / step, (bound - start) / step);
            enter = enter.max(t0.min(t1));
            leave = leave.min(t0.max(t1));
        }
        (enter <= leave).then_some(enter)
    }
}

/// How far the body's middle keeps from the chair walking round it,
/// metres: a hip's half width and a little.
const CLEAR_OF_CHAIR: f32 = 0.2;
/// The corners it walks round by are this far out from the chair.
const ROUND_BY: f32 = 0.45;
/// A way that comes within [`CLEAR_OF_CHAIR`] of the chair only over its
/// last this many metres ends beside it, as the turn onto the spot does:
/// clear.
const ENDS_BESIDE: f32 = 0.2;

/// Where to walk first so as not to walk through `chair` on the way from
/// `at` to `to`: the corner, well out from it, that makes the shortest way
/// round, of those not yet `passed` (a bit each, by index). `None` when the
/// way is clear. Asked again every frame, it goes corner to corner.
fn round(chair: &Chair, at: Vec3, to: Vec3, passed: u8) -> Option<(usize, Vec3)> {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let (at, to) = (flat(at), flat(to));
    // Already within the margin, it keeps clear of the chair itself: given
    // up on there as "beside it", closing on a corner switched the routing
    // off, and the walk went 12 cm into the seat.
    let margin = if chair.met(at, at, CLEAR_OF_CHAIR).is_some() { 0.02 } else { CLEAR_OF_CHAIR };
    let blocked = |from: Vec3, to: Vec3, beside: f32| {
        let length = from.distance(to);
        chair.met(from, to, margin).is_some_and(|t| (1.0 - t) * length > beside)
    };
    if !blocked(at, to, ENDS_BESIDE) {
        return None;
    }
    let half = chair.size * 0.5 + Vec2::splat(ROUND_BY);
    [Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, 1.0), Vec2::new(-1.0, -1.0)]
        .into_iter()
        .map(|corner| chair.world(corner * half))
        .enumerate()
        // Not a corner already reached: skipping only the one it was at, a
        // slowed walk orbited a corner, the next one and this one taking
        // turns as the nearer.
        .filter(|&(i, corner)| passed & (1 << i) == 0 && !blocked(at, corner, 0.0))
        .min_by(|(_, a), (_, b)| (a.distance(at) + a.distance(to)).total_cmp(&(b.distance(at) + b.distance(to))))
}

/// Within this of a corner, metres, it counts as reached.
const AT_CORNER: f32 = 0.3;

/// The turning circle's radius, metres: the slow walk at 2 rad/s, half a
/// circle in ~1.6 s (Robinson: 1.5 s).
///
/// A circle ending on the spot facing away from the chair dips a radius
/// behind it just before, and the chair's front edge is only ~0.10 m
/// behind the spot: ending on the spot, the body passed 15 cm behind the
/// seat's front edge, at its corner ([`TURN_AHEAD`] keeps that to ~5 cm,
/// beside it). Smaller circles the walk could not follow: the body trails
/// its heading by about a step, and at 0.12 m and 0.18 m it stood 18-22 cm
/// off its spot, past what the seat makes up.
pub const TURN_RADIUS: f32 = 0.25;
/// The walk round the turning circle, m/s, before it is paced.
pub const TURN_SPEED: f32 = 0.5;
/// The turn ends this far in front of the spot, metres, so its dip stays
/// beside the chair; the seat makes it up (`sitting::Seat::back`).
pub const TURN_AHEAD: f32 = 0.10;
/// The walk to the turning circle when its owner asks no speed, m/s.
pub const APPROACH_SPEED: f32 = 1.0;
/// Paced over the last this many metres of the path: the turn and a step
/// or two before it. Over 1.5 m the slow pace was a 2.5 s creep.
pub const FIT_WITHIN: f32 = PI * TURN_RADIUS + 0.6;
/// The paces it may take, m/s.
const PACES: (f32, f32) = (0.25, 0.8);
/// A stride grows as speed to this power (`gait::GaitParams::walking_at`:
/// cadence as `speed^0.35`, stride as `speed^0.65`).
const STRIDE_POWER: f32 = 0.65;
/// How far the body travels through a stop's last step, from its footfall
/// to standing, as a fraction of the stride: the double support, then the
/// gait fading out over the other foot's swing (`transition`). Measured
/// live: 0.245 m of a 0.772 m stride.
const LAST_STEP: f32 = 0.32;
/// Footfalls this close, as a fraction of the stride, start the last step
/// at once (`transition::near_footfall`'s window at 60 Hz).
const FOOTFALL_WINDOW: f32 = 0.05;
/// Close enough to the spot and its heading to sit without walking:
/// metres and radians.
const THERE: f32 = 0.06;
const THERE_HEADING: f32 = 0.35;
/// How hard the walk round the circle steers back onto it, radians of
/// heading per radius off it: a fixed 2 rad/m, tuned on a 0.25 m circle,
/// let the walk cut inside the 0.12 m one and stop 7 cm short.
const BACK_ONTO_CIRCLE: f32 = 0.5 / TURN_RADIUS;
/// Onto the circle this far, metres, before the straight's end.
const ONTO_ARC: f32 = TURN_RADIUS * 0.4;
/// A path whose straight is shorter than this, metres, starting this close
/// to the spot, leaves no room to turn onto the line.
const LEAVE_WITHIN: f32 = 0.6;
/// A turn sharper than this, radians, is walked at [`TURN_SPEED`].
const SHARP_TURN: f32 = 0.8;
/// Paced again once the stop is predicted this far off, metres: each new
/// pace costs the gait a stride measurement and the walk a step's measure
/// of what it covers, and the seat makes up a few centimetres
/// (`sitting::Seat::back`).
const REPACE_OFF: f32 = 0.03;

/// The gait, as far as the approach needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gait {
    /// The gait clock, a footfall each half cycle.
    pub cycle: f32,
    /// How far a cycle travels at `speed`, metres.
    pub stride: f32,
    /// The speed the legs step at, m/s; zero standing.
    pub speed: f32,
    /// Fully at rest.
    pub stopped: bool,
}

/// How far a walk told to stop now travels before it stands, metres: on to
/// the next footfall, then the last step.
pub fn stop_distance(cycle: f32, stride: f32) -> f32 {
    let half = cycle.rem_euclid(0.5);
    let to_footfall = if half < FOOTFALL_WINDOW { -half } else { 0.5 - half };
    ((to_footfall + LAST_STEP) * stride).max(0.0)
}

/// The stops a walk can still make, as stride fractions: on from `cycle` to
/// each coming footfall, then the last step.
fn stops_from(cycle: f32) -> impl Iterator<Item = f32> {
    let half = cycle.rem_euclid(0.5);
    (0..8).map(move |k| 0.5 - half + 0.5 * k as f32 + LAST_STEP)
}

/// The stride a walk at `speed` takes, from one at `known_speed` taking
/// `known`.
fn stride_at(speed: f32, known: f32, known_speed: f32) -> f32 {
    known * (speed / known_speed).powf(STRIDE_POWER)
}

/// The pace that puts a stop's end `left` metres on: of the coming
/// footfalls, the one whose stride is nearest the turning pace's, its
/// stride turned into a speed.
pub fn fitted_pace(left: f32, gait: &Gait) -> f32 {
    if gait.speed <= 0.0 || gait.stride <= 0.0 {
        return TURN_SPEED;
    }
    let nominal = stride_at(TURN_SPEED, gait.stride, gait.speed);
    let ratio = stops_from(gait.cycle)
        .map(|fraction| left / fraction / nominal)
        .min_by(|a, b| a.ln().abs().total_cmp(&b.ln().abs()))
        .unwrap_or(1.0);
    (TURN_SPEED * ratio.powf(1.0 / STRIDE_POWER)).clamp(PACES.0, PACES.1)
}

/// How far off the nearest stop the walk can still make ends from `left`
/// metres on, at the stride it takes now.
fn stop_off(left: f32, gait: &Gait) -> f32 {
    stops_from(gait.cycle).map(|fraction| (left - fraction * gait.stride).abs()).fold(f32::INFINITY, f32::min)
}

/// The yaw of a horizontal direction about `+Y`, zero along `-Z`, the
/// convention of `facing::Facing::yaw`.
pub fn heading_of(direction: Vec3) -> f32 {
    (-direction.x).atan2(-direction.z)
}

/// The horizontal direction a heading points along.
pub fn direction_of(heading: f32) -> Vec3 {
    Vec3::new(-heading.sin(), 0.0, -heading.cos())
}

/// Where the character's root stands, facing [`Chair::forward`], to sit on
/// `chair` without moving its feet: the seated hips over the seat
/// (`sitting::seat_offset`). `stood` is the standing pose, on `rig`, whose
/// frame `to_world` turns into the world's once facing the chair's way.
pub fn stand_spot(chair: &Chair, rig: &RigGeometry, stood: &LocalPose, to_world: bevy::math::Quat) -> Vec3 {
    let (offset, _) = sitting::seat_offset(rig, stood, chair.height);
    let hips = forward_kinematics_on(stood, rig)[Bone::Hips];
    let seated = hips + rig.forward() * offset.x + rig.left() * offset.y;
    let spot = chair.seat - to_world * Vec3::new(seated.x, 0.0, seated.z);
    Vec3::new(spot.x, 0.0, spot.z)
}

/// The way round the turning circle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// Turning left: heading increasing.
    Left,
    Right,
}

impl Side {
    fn sign(self) -> f32 {
        match self {
            Side::Left => 1.0,
            Side::Right => -1.0,
        }
    }
}

/// A path to a spot: straight to the turning circle, then round it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Path {
    pub side: Side,
    /// The turning circle's centre.
    pub centre: Vec3,
    /// Where the straight line meets it.
    pub tangent: Vec3,
    /// The straight part's length and the arc's, metres.
    pub straight: f32,
    pub arc: f32,
}

impl Path {
    pub fn length(&self) -> f32 {
        self.straight + self.arc
    }
}

/// The circle a walk turning `side` round ends on, at `spot` heading
/// `heading`.
fn centre_of(spot: Vec3, heading: f32, side: Side) -> Vec3 {
    // Left of the heading turning left (the heading turns toward it).
    spot + direction_of(heading + side.sign() * PI * 0.5) * TURN_RADIUS
}

/// The angle swept turning `side` from radial direction `from` to `to`,
/// in `[0, 2π)`.
fn swept(from: Vec3, to: Vec3, side: Side) -> f32 {
    (side.sign() * (heading_of(to) - heading_of(from))).rem_euclid(TAU)
}

/// The path from `at` to `spot`, arriving heading `heading`, turning
/// `side` round the last circle; `None` from inside that circle.
pub fn path_turning(at: Vec3, spot: Vec3, heading: f32, side: Side) -> Option<Path> {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let (at, spot) = (flat(at), flat(spot));
    let centre = centre_of(spot, heading, side);
    let out = at - centre;
    let distance = out.length();
    if distance < TURN_RADIUS {
        return None;
    }
    // The tangent point whose circling direction leads on from the line.
    let angle = (TURN_RADIUS / distance).acos();
    let towards = heading_of(out);
    let tangent = [towards + angle, towards - angle]
        .into_iter()
        .map(|radial| centre + direction_of(radial) * TURN_RADIUS)
        .find(|&point| {
            let radial = (point - centre) / TURN_RADIUS;
            let circling = direction_of(heading_of(radial) + side.sign() * PI * 0.5);
            (point - at).dot(circling) >= -1.0e-4
        })?;
    let arc = swept(tangent - centre, spot - centre, side) * TURN_RADIUS;
    Some(Path { side, centre, tangent, straight: tangent.distance(at), arc })
}

/// The shorter of the two paths.
pub fn path(at: Vec3, spot: Vec3, heading: f32) -> Option<Path> {
    [Side::Left, Side::Right]
        .into_iter()
        .filter_map(|side| path_turning(at, spot, heading, side))
        .min_by(|a, b| a.length().total_cmp(&b.length()))
}

/// What the walk does this frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Order {
    /// Walk at `speed`, turning toward `heading` at most `rate` rad/s.
    Walk { speed: f32, heading: f32, rate: f32 },
    /// Stop, still turning toward `heading`.
    Stop { heading: f32, rate: f32 },
    /// On the spot facing the right way: sit.
    Arrived,
}

/// Where a walk to a chair is.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Approach {
    /// Not started.
    #[default]
    Idle,
    /// Walking the path round a circle turning `side`; `on_arc` once round
    /// the circle; `pace` once paced, and the gait clock when; `around`
    /// while going round the chair, with the corners passed.
    Walking { side: Side, on_arc: bool, pace: Option<(f32, f32)>, around: Option<u8> },
    /// Too close to the spot to turn onto it: walking away first.
    Leaving,
    /// Told to stop, the last step under way, still following the path
    /// round `side` if on one.
    Stopping { side: Option<Side>, on_arc: bool },
    Arrived,
}

/// Following the path round `side` from `at`: how far is left along it,
/// the heading to walk, and whether round the circle yet. `None` inside
/// the circle before reaching it.
fn follow(at: Vec3, spot: Vec3, heading: f32, side: Side, on_arc: bool) -> Option<(f32, f32, bool)> {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let centre = centre_of(spot, heading, side);
    let path = path_turning(at, spot, heading, side);
    let on_arc = on_arc || path.is_some_and(|path| path.straight < ONTO_ARC);
    if !on_arc {
        let path = path?;
        return Some((path.length(), heading_of(path.tangent - flat(at)), false));
    }
    // Round the circle, steered back onto it if off.
    let radial = flat(at) - centre;
    let along = heading_of(radial) + side.sign() * PI * 0.5;
    let wide = radial.length() - TURN_RADIUS;
    let toward = along + side.sign() * (wide * BACK_ONTO_CIRCLE).clamp(-0.6, 0.6);
    // At the radius it is walking: wide of the circle, it travels further
    // for the same turn, and measured at the circle's radius its stop fell
    // 2-4 cm short.
    let swept = swept(radial, spot - centre, side);
    // Just past the spot reads as a whole circle to go.
    let left = if swept > TAU - 0.4 { 0.0 } else { swept * radial.length().max(TURN_RADIUS * 0.5) };
    Some((left, toward, true))
}

impl Approach {
    /// Advances the walk from `at`, walking heading `walking`, to `spot`,
    /// arriving heading `heading`, at `speed` until it paces itself, round
    /// `chair` if it is in the way.
    #[allow(clippy::too_many_arguments)]
    pub fn advance(&mut self, at: Vec3, walking: f32, gait: &Gait, spot: Vec3, heading: f32, speed: f32, chair: Option<&Chair>) -> Order {
        let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let off = flat(spot - at).length();
        let facing_off = super::facing::shortest_angle(heading - walking).abs();
        let turning = |pace: f32| 1.5 * pace.max(TURN_SPEED) / TURN_RADIUS;
        // Turning sharply, it slows to the turning pace, as a person does: at
        // 1 m/s the 2 rad/s turn is a 0.5 m loop.
        let slowed = |speed: f32, toward: f32| {
            if super::facing::shortest_angle(toward - walking).abs() > SHARP_TURN { TURN_SPEED.min(speed) } else { speed }
        };
        if let Approach::Idle = self {
            *self = if off < THERE && facing_off < THERE_HEADING {
                Approach::Stopping { side: None, on_arc: false }
            } else {
                match path(at, spot, heading) {
                    // Too close to the spot to turn onto it, within both its
                    // circles or with too short a straight to turn onto the
                    // line: away from the chair first, and back slowly
                    // (`SHARP_TURN`).
                    Some(path) if path.straight >= LEAVE_WITHIN || off >= 2.0 * LEAVE_WITHIN => {
                        Approach::Walking { side: path.side, on_arc: false, pace: None, around: None }
                    }
                    _ => Approach::Leaving,
                }
            };
        }
        match *self {
            Approach::Idle => unreachable!(),
            Approach::Leaving => {
                // To a point well in front of the spot, then back.
                let away = spot + direction_of(heading) * (2.0 * LEAVE_WITHIN);
                if flat(away - at).length() < 0.2 {
                    *self = Approach::Idle;
                }
                Order::Walk { speed: TURN_SPEED, heading: heading_of(flat(away - at)), rate: 2.0 }
            }
            Approach::Walking { side, on_arc, pace, around } => {
                let Some((left, toward, on_arc)) = follow(at, spot, heading, side, on_arc) else {
                    // Walked inside the circle on the straight: start over.
                    *self = Approach::Idle;
                    return Order::Stop { heading, rate: super::facing::Facing::default().turn_rate };
                };
                // A straight through the chair goes round it first, never
                // stopping on the way; round, the path is planned afresh
                // (the shorter circle from beside the chair is not the one
                // from behind it).
                // Round it toward whichever circle is the shorter way from
                // where it now is: held to the one chosen behind the chair,
                // whose tangent lay on the chair's side, it went back for
                // a corner behind.
                let passed = around.unwrap_or(0);
                let best = if around.is_some() { path(at, spot, heading) } else { path_turning(at, spot, heading, side) };
                let corner = if on_arc {
                    None
                } else {
                    best.and_then(|path| chair.and_then(|chair| round(chair, at, path.tangent, passed)).map(|corner| (path.side, corner)))
                };
                if let Some((side, (index, corner))) = corner {
                    let reached = if flat(corner - at).length() < AT_CORNER { 1 << index } else { 0 };
                    *self = Approach::Walking { side, on_arc, pace: None, around: Some(passed | reached) };
                    let toward = heading_of(flat(corner - at));
                    return Order::Walk { speed: slowed(speed, toward), heading: toward, rate: 2.0 };
                }
                if around.is_some() {
                    *self = Approach::Idle;
                    return self.advance(at, walking, gait, spot, heading, speed, chair);
                }
                // Told to stop when that stop ends nearer the spot than the
                // next one could; paced, that is on it.
                // Only once paced: from a full-speed stride (1.4 m at 1.2 m/s)
                // it stopped 1.3 m out, before it had slowed at all.
                // And walking at that pace: a stop decided on a stride from
                // another speed (0.72 m/s, paced at 0.39) ended mid-turn,
                // 25 cm short, beside the chair.
                let at_pace = pace.is_some_and(|(pace, _)| (gait.speed - pace).abs() < 0.02);
                if at_pace && left - stop_distance(gait.cycle, gait.stride) <= 0.25 * gait.stride {
                    *self = Approach::Stopping { side: Some(side), on_arc };
                    return Order::Stop { heading: toward, rate: turning(gait.speed) };
                }
                let pace = if left > FIT_WITHIN {
                    None
                } else {
                    // Paced once, then again only if the stop drifts off,
                    // and at most once a step: each new speed costs the gait
                    // a stride measurement and spoils the step being
                    // measured, and repaced every frame on those the pace
                    // hopped 0.39-0.66 m/s.
                    match pace {
                        Some((pace, since)) if (gait.cycle - since).rem_euclid(1.0) < 0.5 || stop_off(left, gait) < REPACE_OFF => Some((pace, since)),
                        _ => Some((fitted_pace(left, gait), gait.cycle)),
                    }
                };
                *self = Approach::Walking { side, on_arc, pace, around: None };
                let speed = pace.map(|(pace, _)| pace).unwrap_or(if on_arc { TURN_SPEED } else { slowed(speed, toward) });
                Order::Walk { speed, heading: toward, rate: if on_arc { turning(speed) } else { 2.0 } }
            }
            Approach::Stopping { side, on_arc } => {
                // Along the path while there is any and the last step is
                // under way, then the arrival's heading: a stop short of
                // the path's end otherwise never turns the rest of the way.
                let followed = side.filter(|_| !gait.stopped).and_then(|side| follow(at, spot, heading, side, on_arc).map(|f| (side, f)));
                let toward = match followed {
                    Some((side, (left, toward, on_arc))) if left > 0.03 => {
                        *self = Approach::Stopping { side: Some(side), on_arc };
                        toward
                    }
                    _ => {
                        *self = Approach::Stopping { side: None, on_arc };
                        heading
                    }
                };
                if gait.stopped && facing_off < 0.02 {
                    *self = Approach::Arrived;
                    Order::Arrived
                } else {
                    Order::Stop { heading: toward, rate: turning(gait.speed) }
                }
            }
            Approach::Arrived => Order::Arrived,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::facing::shortest_angle;

    /// The stride the model walk takes at the turning pace (the live one's).
    const STRIDE: f32 = 0.772;

    /// A walk modelled as the real one stops: told to stop, it walks on to
    /// a footfall (one within [`FOOTFALL_WINDOW`] counts) and then
    /// [`LAST_STEP`] of a stride; its stride follows its speed.
    struct Walk {
        at: Vec3,
        facing: f32,
        cycle: f32,
        speed: f32,
        /// Told to stop: the stride fraction still to travel once known.
        stopping: Option<Option<f32>>,
        walked: f32,
    }

    impl Walk {
        fn stride(&self) -> f32 {
            stride_at(self.speed.max(0.05), STRIDE, TURN_SPEED)
        }

        fn gait(&self) -> Gait {
            let stopped = matches!(self.stopping, Some(Some(left)) if left <= 0.0);
            Gait { cycle: self.cycle, stride: self.stride(), speed: if stopped { 0.0 } else { self.speed }, stopped }
        }

        fn step(&mut self, order: Order, dt: f32) {
            let (heading, rate) = match order {
                Order::Walk { speed, heading, rate } => {
                    self.speed = speed;
                    self.stopping = None;
                    (heading, rate)
                }
                Order::Stop { heading, rate } => (heading, rate),
                Order::Arrived => return,
            };
            self.facing += shortest_angle(heading - self.facing).clamp(-rate * dt, rate * dt);
            let advance = self.speed / self.stride() * dt;
            let mut moved = advance;
            if let Order::Stop { .. } = order {
                let stopping = self.stopping.get_or_insert(None);
                match stopping {
                    None => {
                        let half = self.cycle.rem_euclid(0.5);
                        if half < FOOTFALL_WINDOW {
                            *stopping = Some(LAST_STEP - half);
                        } else if half + advance >= 0.5 {
                            *stopping = Some(LAST_STEP - (half + advance - 0.5));
                        }
                    }
                    Some(left) => {
                        moved = advance.min(*left).max(0.0);
                        *left -= advance;
                    }
                }
            }
            self.cycle = (self.cycle + moved).rem_euclid(1.0);
            let distance = moved * self.stride();
            self.at += direction_of(self.facing) * distance;
            self.walked += distance;
        }
    }

    /// Walks to `spot` from `from`, heading `heading`, arriving heading
    /// `arrive`, round `chair`: where it ended, facing, how far it walked,
    /// and every point it walked through.
    fn walk(from: Vec3, heading: f32, spot: Vec3, arrive: f32, cycle: f32, chair: Option<&Chair>) -> (Vec3, f32, f32, Vec<Vec3>) {
        let dt = 1.0 / 60.0;
        let mut walk = Walk { at: from, facing: heading, cycle, speed: 0.0, stopping: None, walked: 0.0 };
        let mut approach = Approach::default();
        let mut track = Vec::new();
        for _ in 0..60 * 60 {
            let order = approach.advance(walk.at, walk.facing, &walk.gait(), spot, arrive, 1.2, chair);
            if order == Order::Arrived {
                return (walk.at, walk.facing, walk.walked, track);
            }
            walk.step(order, dt);
            track.push(walk.at);
        }
        panic!("never arrived from {from} heading {heading}");
    }

    /// The chair whose turn ends at (0, 0, -3), sat on facing +Z: its seat
    /// 0.4 m behind the spot, as the rig's upright sit puts it, the spot
    /// [`TURN_AHEAD`] behind where the turn ends.
    fn chair_behind_the_spot() -> Chair {
        Chair::standard(Vec3::new(0.0, 0.0, -3.4 - TURN_AHEAD), Vec3::Z)
    }

    #[test]
    fn the_turn_onto_the_spot_passes_at_most_7_cm_past_the_seat_front() {
        // Every way of arriving dips a turning radius behind where the turn
        // ends; ending on the spot, the body's middle passed 15 cm past the
        // seat's front edge, by its corner. Ended TURN_AHEAD in front, at
        // most 7 cm. (A smaller circle the walk cannot follow; a pivot
        // swings the feet as far.)
        let chair = chair_behind_the_spot();
        let spot = Vec3::new(0.0, 0.0, -3.0);
        for (from, heading) in [(Vec3::ZERO, 0.0), (Vec3::new(2.0, 0.0, -3.0), PI * 0.5), (Vec3::new(-2.0, 0.0, -2.5), -PI * 0.5)] {
            let (_, _, _, track) = walk(from, heading, spot, PI, 0.2, Some(&chair));
            let half = chair.size * 0.5;
            for local in track.iter().map(|&point| chair.local(point)) {
                let behind_front = half.y - local.y;
                if local.x.abs() < half.x && behind_front > 0.0 {
                    assert!(behind_front < 0.07, "from {from}: {:.0} mm past the seat's front edge", behind_front * 1e3);
                }
            }
        }
    }

    #[test]
    fn the_gallerys_chairs_are_reached_from_where_its_character_starts() {
        // The gallery's character starts at the origin; its chairs (`--chair`)
        // at -1.5,-1.5 facing the camera and at 0,-1.5 with its back to the
        // character. Live, the first walked 12 cm into the seat's corner and
        // the second never sat down. The spot 0.4 m in front of the seat,
        // the turn ending TURN_AHEAD further, as the walker has it.
        for (seat, forward) in [(Vec3::new(-1.5, 0.0, -1.5), Vec3::Z), (Vec3::new(0.0, 0.0, -1.5), Vec3::NEG_Z), (Vec3::new(1.5, 0.0, -2.0), Vec3::NEG_X)] {
            let chair = Chair::standard(seat, forward);
            let turn_end = seat + forward * (0.4 + TURN_AHEAD);
            for heading in [0.0, PI, 0.7] {
                let (at, _, _, track) = walk(Vec3::ZERO, heading, turn_end, heading_of(forward), 0.3, Some(&chair));
                let off = Vec3::new(at.x - turn_end.x, 0.0, at.z - turn_end.z).length();
                assert!(off < 0.06, "chair at {seat} from heading {heading}: ended {off:.3} m off");
                let half = chair.size * 0.5;
                for local in track.iter().map(|&point| chair.local(point)) {
                    let inside = half - local.abs();
                    assert!(inside.x < 0.0 || inside.y < 0.07, "chair at {seat} from heading {heading}: walked into it at {local}");
                }
            }
        }
    }

    #[test]
    fn a_walk_from_behind_the_chair_goes_round_it() {
        // Straight to the turning circle it walked through the chair. From
        // behind, and from behind and to either side, it now keeps its
        // middle clear of it (CLEAR_OF_CHAIR, less a little for steering)
        // until it is beside the spot, and still arrives.
        let chair = chair_behind_the_spot();
        let spot = Vec3::new(0.0, 0.0, -3.0);
        for (from, heading) in [(Vec3::new(0.0, 0.0, -6.0), 0.0), (Vec3::new(1.0, 0.0, -5.5), PI), (Vec3::new(-0.8, 0.0, -4.5), 0.5)] {
            let (at, _, _, track) = walk(from, heading, spot, PI, 0.1, Some(&chair));
            let off = Vec3::new(at.x - spot.x, 0.0, at.z - spot.z).length();
            // Less straight is left to pace on once round the chair; still
            // well within what the seat makes up (15 cm back, 8 across).
            assert!(off < 0.06, "from {from}: ended {off:.3} m off the spot");
            let closest = track
                .iter()
                .filter(|point| point.distance(spot) > 0.6)
                .map(|&point| {
                    let local = chair.local(point);
                    let outside = (local.abs() - chair.size * 0.5).max(Vec2::ZERO);
                    if local.abs().cmplt(chair.size * 0.5).all() { -1.0 } else { outside.length() }
                })
                .fold(f32::INFINITY, f32::min);
            assert!(closest > CLEAR_OF_CHAIR - 0.06, "from {from}: came within {closest:.2} m of the chair");
            // And through the turn, never into it: past its front edge only
            // brushing its corner (see
            // `the_turn_onto_the_spot_passes_at_most_7_cm_past_the_seat_front`).
            for point in &track {
                let local = chair.local(*point);
                let inside = chair.size * 0.5 - local.abs();
                assert!(inside.x < 0.03 || inside.y < 0.07, "from {from}: walked into the chair at {point}");
            }
        }
    }

    #[test]
    fn the_path_ends_on_the_spot_heading_the_way_asked() {
        let spot = Vec3::new(1.0, 0.0, -2.0);
        for (from, heading) in [(Vec3::ZERO, 0.0), (Vec3::new(4.0, 0.0, 1.0), 2.0), (Vec3::new(-3.0, 0.0, -5.0), -2.5)] {
            let path = path(from, spot, heading).unwrap();
            // The circle passes through the spot, and its tangent there is
            // the arrival heading.
            assert!((path.centre.distance(spot) - TURN_RADIUS).abs() < 1.0e-5);
            let radial = (spot - path.centre) / TURN_RADIUS;
            let circling = direction_of(heading_of(radial) + path.side.sign() * PI * 0.5);
            assert!(circling.dot(direction_of(heading)) > 0.9999, "arrives heading {heading}: {circling}");
            // The straight touches the circle: perpendicular to its radius.
            assert!((path.tangent - from).dot(path.tangent - path.centre).abs() < 1.0e-4);
        }
    }

    #[test]
    fn a_walk_to_a_chair_turns_round_and_its_stop_lands_on_the_spot() {
        // A chair 3 m ahead, facing the walker: it must walk up and turn
        // about half a circle. And from beside, behind, close, and too
        // close to turn onto the spot; each from several points in the
        // stride, since where a stop can land depends on it.
        let spot = Vec3::new(0.0, 0.0, -3.0);
        let arrive = PI;
        // The detour each may take beyond the straight line, metres: too
        // close, it walks out 1.2 m and comes back round.
        for (from, heading, detour) in [
            (Vec3::ZERO, 0.0, 1.5),
            (Vec3::new(2.0, 0.0, -3.0), PI * 0.5, 0.5),
            (Vec3::new(0.0, 0.0, -6.0), 0.0, 2.5),
            (Vec3::new(0.3, 0.0, -2.0), 0.0, 1.5),
            (Vec3::new(0.1, 0.0, -3.2), 0.0, 6.5),
        ] {
            for cycle in [0.0, 0.13, 0.27, 0.41] {
                // No chair to walk round: the stop alone (round it, see
                // `a_walk_from_behind_the_chair_goes_round_it`).
                let (at, facing, walked, _) = walk(from, heading, spot, arrive, cycle, None);
                let off = Vec3::new(at.x - spot.x, 0.0, at.z - spot.z).length();
                // Re-paced only past REPACE_OFF: the seat makes up the rest.
                assert!(off < REPACE_OFF + 0.01, "from {from} at cycle {cycle}: ended {off:.3} m off the spot");
                assert!(shortest_angle(facing - arrive).abs() < 0.03, "from {from}: facing {facing}");
                let straight_line = from.distance(spot);
                assert!(walked < straight_line + detour, "from {from}: walked {walked:.2} m for {straight_line:.2}");
            }
        }
    }

    #[test]
    fn stopped_at_the_nearest_footfall_alone_misses_by_up_to_a_quarter_stride() {
        // Why it paces itself: unpaced, a stop can only end in half-stride
        // steps, the nearest up to a quarter stride from any spot.
        let worst = (0..100)
            .map(|i| {
                let left = 1.0 + i as f32 * 0.01;
                stop_off(left, &Gait { cycle: 0.1, stride: STRIDE, speed: TURN_SPEED, stopped: false })
            })
            .fold(0.0, f32::max);
        assert!(worst > 0.18, "unpaced worst {worst:.3} m");
        // Paced for the same, it lands within a centimetre.
        for i in 0..100 {
            let left = 1.0 + i as f32 * 0.01;
            let gait = Gait { cycle: 0.1, stride: STRIDE, speed: TURN_SPEED, stopped: false };
            let pace = fitted_pace(left, &gait);
            let paced = Gait { stride: stride_at(pace, STRIDE, TURN_SPEED), speed: pace, ..gait };
            assert!(stop_off(left, &paced) < 0.01, "left {left}: paced {pace:.2} m/s still {:.3} m off", stop_off(left, &paced));
        }
    }

    #[test]
    fn turning_round_takes_about_as_long_as_a_person() {
        // From walking straight at the chair: a little over a half circle,
        // the straight meeting the circle from 3 m off.
        let path = path(Vec3::ZERO, Vec3::new(0.0, 0.0, -3.0), PI).unwrap();
        let seconds = path.arc / TURN_SPEED;
        assert!((PI * TURN_RADIUS..PI * TURN_RADIUS + 0.06).contains(&path.arc), "arc {:.3}", path.arc);
        assert!((1.2..2.2).contains(&seconds), "{seconds:.2} s round (Robinson: 1.5 s median)");
    }

    #[test]
    fn already_there_sits_without_walking() {
        let mut approach = Approach::default();
        let standing = Gait { cycle: 0.0, stride: 0.0, speed: 0.0, stopped: true };
        let spot = Vec3::new(0.02, 0.0, 0.0);
        assert!(matches!(approach.advance(Vec3::ZERO, 0.1, &standing, spot, 0.0, 1.0, None), Order::Stop { heading: 0.0, .. }));
        assert_eq!(approach.advance(Vec3::ZERO, 0.0, &standing, spot, 0.0, 1.0, None), Order::Arrived);
    }
}
