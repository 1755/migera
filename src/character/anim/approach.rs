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

use super::gait;
use super::obstacles::Footprint;
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

    /// What it stands on: for the feet to keep clear of
    /// (`obstacles::AnimObstacles`) and the walk to go round.
    pub fn footprint(&self) -> Footprint {
        let forward = Vec3::new(self.forward.x, 0.0, self.forward.z).normalize_or_zero();
        let middle = self.seat + forward * self.ahead;
        Footprint { middle: Vec3::new(middle.x, 0.0, middle.z), forward, size: self.size }
    }
}

/// How far the body's middle keeps from an obstacle walking round it,
/// metres: a hip's half width and a little.
const CLEAR_OF_CHAIR: f32 = 0.2;
/// The corners it walks round by are this far out from an obstacle.
const ROUND_BY: f32 = 0.45;
/// A way that comes within [`CLEAR_OF_CHAIR`] of an obstacle only over its
/// last this many metres ends beside it, as the turn onto the spot does:
/// clear.
const ENDS_BESIDE: f32 = 0.2;
/// Within this of a corner, metres, it counts as reached.
const AT_CORNER: f32 = 0.3;
/// A new way round is taken over the one being walked only when shorter by
/// this, metres: re-planned every frame without it, two near-equal ways
/// took turns, and a walk orbited a corner.
const ROUTE_HYSTERESIS: f32 = 0.3;

/// How far from where the turn ends an entry is, metres: room to turn for
/// the spot there and a straight onto the turning circle long enough to
/// pace the stop on. At 0.9 m the stop fell 9 cm off.
const ENTRY: f32 = 1.2;
/// How far the final approach from an entry keeps from obstacles other than
/// its own chair, metres: a turn's dip from beside a table's chair grazed
/// the next chair's corner.
const ENTRY_CLEAR: f32 = 0.1;

/// The final approach from `from` to `spot` arriving heading `heading`, as
/// points every ~5 cm: the straight onto the turning circle, then round it.
fn final_approach(from: Vec3, spot: Vec3, heading: f32) -> Vec<Vec3> {
    let Some(path) = path(from, spot, heading) else { return vec![from, spot] };
    let mut points: Vec<Vec3> = (0..=(path.straight / 0.05) as usize).map(|i| from.lerp(path.tangent, (i as f32 * 0.05 / path.straight.max(1.0e-6)).min(1.0))).collect();
    let start = heading_of(path.tangent - path.centre);
    let swept = path.arc / TURN_RADIUS;
    let steps = (path.arc / 0.05).ceil().max(1.0) as usize;
    points.extend((0..=steps).map(|i| path.centre + direction_of(start + path.side.sign() * swept * i as f32 / steps as f32) * TURN_RADIUS));
    points
}
/// Within this of the entry, metres, it turns for the spot.
const AT_ENTRY: f32 = 0.25;

/// Where a walk to `spot` (where the turn ends, arriving heading `heading`)
/// comes from: [`ENTRY`] in front of it, or to its left or right, the
/// nearest to `at` that is clear of every obstacle with a clear way into
/// the spot. A person comes at a chair from in front in the open, and
/// along the gap from one side at a table. Always from in front, past a
/// table's corner, the walk went through the table; back the way it came,
/// it came the same way again, too close again.
pub fn entry(obstacles: &[Footprint], at: Vec3, spot: Vec3, heading: f32) -> Vec3 {
    let forward = direction_of(heading);
    let left = Vec3::Y.cross(forward);
    let candidates = [spot + forward * ENTRY, spot + left * ENTRY, spot - left * ENTRY];
    candidates
        .into_iter()
        .filter(|&point| entry_fits(obstacles, point, spot, heading))
        .min_by(|a, b| a.distance(at).total_cmp(&b.distance(at)))
        .unwrap_or(candidates[0])
}

/// Whether `point` will do as an entry for `spot`: clear of every obstacle,
/// a clear way into the spot, and the whole final approach, its turn
/// included, clear of all but its own chair (whose front the turn passes by
/// design).
fn entry_fits(obstacles: &[Footprint], point: Vec3, spot: Vec3, heading: f32) -> bool {
    obstacles.iter().all(|o| o.met(point, point, CLEAR_OF_CHAIR).is_none())
        && route(obstacles, !obstacles.is_empty(), point, spot, None) == Route::Clear
        && final_approach(point, spot, heading).iter().all(|&p| obstacles.iter().skip(1).all(|o| o.met(p, p, ENTRY_CLEAR).is_none()))
}

/// Where a walk round obstacles goes next.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Route {
    /// Straight there.
    Clear,
    /// To this corner first.
    Via(Vec3),
    /// No way round from here.
    Stuck,
}

/// Where to walk next so as not to walk through any of `obstacles` on the
/// way from `at` to `to`. The first obstacle, if `own`, is the chair the
/// walk ends beside: a way into `to` keeps out of it alone, not its margin
/// (`to` is within it, by design).
///
/// A visibility graph: each obstacle's corners [`ROUND_BY`] out from it
/// (those not within [`CLEAR_OF_CHAIR`] of another), linked where a
/// straight walk keeps clear of every obstacle, searched from `to`. Round a
/// table with its chairs pulled out, walked corner by corner of one box, a
/// corner could lie inside the next. The corner being walked to (`held`)
/// is kept unless another way is [`ROUTE_HYSTERESIS`] shorter.
pub fn route(obstacles: &[Footprint], own: bool, at: Vec3, to: Vec3, held: Option<Vec3>) -> Route {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let (at, to) = (flat(at), flat(to));
    // Already within an obstacle's margin, it keeps clear of the obstacle
    // itself: given up on there as "beside it", closing on a corner
    // switched the routing off, and the walk went 12 cm into the seat.
    let margins: Vec<f32> = obstacles.iter().map(|o| if o.met(at, at, CLEAR_OF_CHAIR).is_some() { 0.02 } else { CLEAR_OF_CHAIR }).collect();
    // `into`: a way into the goal, which may end beside an obstacle and
    // reaches its own chair.
    let blocked = |from: Vec3, to: Vec3, into: bool| {
        let length = from.distance(to);
        obstacles.iter().zip(&margins).enumerate().any(|(i, (o, &margin))| {
            if into && own && i == 0 {
                return o.met(from, to, 0.02).is_some();
            }
            let beside = if into { ENDS_BESIDE } else { 0.0 };
            o.met(from, to, margin).is_some_and(|t| (1.0 - t) * length > beside)
        })
    };
    if !blocked(at, to, true) {
        return Route::Clear;
    }
    // The goal first, then the corners clear of every obstacle.
    let mut nodes = vec![to];
    nodes.extend(
        obstacles
            .iter()
            .flat_map(|o| o.corners(ROUND_BY))
            .filter(|&corner| obstacles.iter().all(|o| o.met(corner, corner, CLEAR_OF_CHAIR).is_none())),
    );
    // Shortest distances to the goal (Dijkstra; a few dozen nodes).
    let n = nodes.len();
    let mut distance = vec![f32::INFINITY; n];
    let mut done = vec![false; n];
    distance[0] = 0.0;
    for _ in 0..n {
        let Some(u) = (0..n).filter(|&i| !done[i] && distance[i].is_finite()).min_by(|&a, &b| distance[a].total_cmp(&distance[b])) else { break };
        done[u] = true;
        for v in 0..n {
            if done[v] {
                continue;
            }
            // Into the goal it may end beside an obstacle (the spot is beside
            // its chair).
            let through = distance[u] + nodes[u].distance(nodes[v]);
            if through < distance[v] && !blocked(nodes[v], nodes[u], u == 0) {
                distance[v] = through;
            }
        }
    }
    // The first corner: the visible one with the shortest way on.
    let way = |i: usize| at.distance(nodes[i]) + distance[i];
    let Some(best) = (1..n).filter(|&i| distance[i].is_finite() && !blocked(at, nodes[i], false)).min_by(|&a, &b| way(a).total_cmp(&way(b))) else {
        return Route::Stuck;
    };
    if let Some(held) = held
        && let Some(kept) = (1..n).find(|&i| nodes[i].distance(held) < 1.0e-3)
        && at.distance(held) > AT_CORNER
        && distance[kept].is_finite()
        && !blocked(at, held, false)
        && way(kept) < way(best) + ROUTE_HYSTERESIS
    {
        return Route::Via(held);
    }
    Route::Via(nodes[best])
}

/// The turning circle's radius, metres: the slow walk at 2.2 rad/s, half a
/// circle in 1.4 s (Robinson: 1.5 s).
///
/// A circle ending on the spot facing away from the chair dips a radius
/// behind it just before, toward the seat's front corner. At 0.25 m the
/// body's middle came within 5 cm of the chair; at 0.18 m, 12 cm. The walk
/// follows the smaller circle only in short steps (`gait::SHORT_STEPS`):
/// in its own 0.39 m ones the body trailed its heading by about a step, and
/// at 0.12 m and 0.18 m it stood 18-22 cm off its spot.
pub const TURN_RADIUS: f32 = 0.18;
/// The walk round the turning circle, m/s, before it is paced: half a
/// circle in 1.4 s.
pub const TURN_SPEED: f32 = 0.4;
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
/// Within this of an obstacle, metres, it walks at [`TURN_SPEED`] too.
const SLOW_NEAR: f32 = 0.5;
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
    /// The speeds between which the stride grows with speed
    /// (`gait::stride_speeds`).
    pub stride_speeds: (f32, f32),
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
/// `known`: it grows as `speed^0.65` between `speeds`, and is held outside
/// them (`gait::stride_speeds`).
pub fn stride_at(speed: f32, known: f32, known_speed: f32, speeds: (f32, f32)) -> f32 {
    let held = |speed: f32| speed.clamp(speeds.0, speeds.1);
    known * (held(speed) / held(known_speed)).powf(gait::STRIDE_GROWTH)
}

/// The pace that puts a stop's end `left` metres on: of the coming
/// footfalls, the one whose stride is nearest the turning pace's among
/// those a pace can give, its stride turned into a speed; with none, the
/// one that lands nearest.
///
/// Below `Gait::stride_speeds` a slower pace only steps slower: assuming
/// it shortened the stride, a walk paced to 0.30 m/s stopped 20 cm past
/// its spot.
pub fn fitted_pace(left: f32, gait: &Gait) -> f32 {
    if gait.speed <= 0.0 || gait.stride <= 0.0 {
        return TURN_SPEED;
    }
    let stride = |speed: f32| stride_at(speed, gait.stride, gait.speed, gait.stride_speeds);
    let (shortest, longest, nominal) = (stride(PACES.0), stride(PACES.1), stride(TURN_SPEED));
    let wanted = || stops_from(gait.cycle).map(|fraction| (fraction, left / fraction));
    let wanted = wanted()
        .filter(|&(_, wanted)| (shortest..=longest).contains(&wanted))
        .min_by(|a, b| (a.1 / nominal).ln().abs().total_cmp(&(b.1 / nominal).ln().abs()))
        .or_else(|| {
            let miss = |(fraction, wanted): (f32, f32)| miss_cost(left - fraction * wanted.clamp(shortest, longest));
            wanted().min_by(|&a, &b| miss(a).total_cmp(&miss(b)))
        })
        .map_or(nominal, |(_, wanted)| wanted.clamp(shortest, longest));
    // The pace taking it; for the shortest, the pace it stops shortening
    // at.
    let held = gait.speed.clamp(gait.stride_speeds.0, gait.stride_speeds.1);
    (held * (wanted / gait.stride).powf(1.0 / gait::STRIDE_GROWTH)).clamp(PACES.0, PACES.1)
}

/// How much of what the seat makes up (`sitting::Seat`) a stop ending
/// `short` metres of the path's end takes (negative: past it); over 1 it
/// sits off the seat's middle.
///
/// The path ends round the turning circle heading away from the chair, the
/// turn [`TURN_AHEAD`] in front of the spot:
/// - past it, the stop stands further out, where the seat's 15 cm back
///   take up only 5;
/// - short of it, back round the circle: nearer the chair (25 cm) and to
///   the side (8 cm).
///
/// Where the stride no longer shortens (`Gait::stride_speeds`) no pace may
/// land a stop on the end; the nearer by distance alone may be 30 cm short
/// and 16 cm to the side, against 4 cm past that the seat takes up.
fn miss_cost(short: f32) -> f32 {
    if short < 0.0 {
        return -short / (sitting::SEAT_BACK_RANGE - TURN_AHEAD);
    }
    let round = (short / TURN_RADIUS).min(PI);
    let back = TURN_RADIUS * if round < PI * 0.5 { round.sin() } else { 1.0 };
    let across = TURN_RADIUS * (1.0 - round.cos());
    // Further short than the half circle, on the straight before it: still
    // worse the further (held there, two stops a step apart cost the same,
    // and it stopped a metre out).
    let before = (short - PI * TURN_RADIUS).max(0.0);
    (back / (sitting::SEAT_BACK_RANGE + TURN_AHEAD)).max(across / sitting::SEAT_ACROSS_RANGE) + before / (sitting::SEAT_BACK_RANGE + TURN_AHEAD)
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
    /// while going round obstacles, with the corner it is walking to;
    /// `committed` from an entry, its way checked clear there.
    Walking { side: Side, on_arc: bool, pace: Option<(f32, f32)>, around: Option<Vec3>, committed: bool },
    /// Walking to an `entry` ([`entry`]) first: too close to the spot to
    /// turn onto it, or with something in the way of the turn; `around`
    /// the corner it is walking to, going round obstacles.
    Leaving { entry: Vec3, around: Option<Vec3> },
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
    /// `obstacles` where they are in the way: the first, if any, is the
    /// chair it walks to, whose margin the spot lies within.
    #[allow(clippy::too_many_arguments)]
    pub fn advance(&mut self, at: Vec3, walking: f32, gait: &Gait, spot: Vec3, heading: f32, speed: f32, obstacles: &[Footprint]) -> Order {
        let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let off = flat(spot - at).length();
        let facing_off = super::facing::shortest_angle(heading - walking).abs();
        let turning = |pace: f32| 1.5 * pace.max(TURN_SPEED) / TURN_RADIUS;
        // Turning sharply, it slows to the turning pace, as a person does: at
        // 1 m/s the 2 rad/s turn is a 0.5 m loop.
        // And near furniture: at 1.2 m/s the turn onto a line past a chair's
        // corner swung 0.6 m wide and came within 8 cm of it.
        let near = obstacles.iter().any(|o| o.met(at, at, SLOW_NEAR).is_some());
        let slowed = |speed: f32, toward: f32| {
            if near || super::facing::shortest_angle(toward - walking).abs() > SHARP_TURN { TURN_SPEED.min(speed) } else { speed }
        };
        if let Approach::Idle = self {
            *self = if off < THERE && facing_off < THERE_HEADING {
                Approach::Stopping { side: None, on_arc: false }
            } else {
                // Straight onto the turn when there is room for it and
                // nothing in the way; else by an entry first (`entry`).
                match path(at, spot, heading) {
                    Some(path)
                        if (path.straight >= LEAVE_WITHIN || off >= 2.0 * LEAVE_WITHIN)
                            && route(obstacles, !obstacles.is_empty(), at, path.tangent, None) == Route::Clear =>
                    {
                        Approach::Walking { side: path.side, on_arc: false, pace: None, around: None, committed: false }
                    }
                    _ => Approach::Leaving { entry: entry(obstacles, at, spot, heading), around: None },
                }
            };
        }
        match *self {
            Approach::Idle => unreachable!(),
            Approach::Leaving { entry: chosen, around } => {
                // The entry chosen again if no longer fit: the obstacles are
                // found as the walk nears them, and one chosen before the far
                // chairs were known lay 15 cm from one, unreachable.
                let entry = if entry_fits(obstacles, chosen, spot, heading) { chosen } else { self::entry(obstacles, at, spot, heading) };
                // To the entry, round anything in the way; there, onto the
                // turn whatever the check above says (from the entry it was
                // chosen to be clear).
                if flat(entry - at).length() < AT_ENTRY
                    && let Some(path) = path(at, spot, heading)
                {
                    *self = Approach::Walking { side: path.side, on_arc: false, pace: None, around: None, committed: true };
                    return self.advance(at, walking, gait, spot, heading, speed, obstacles);
                }
                // No way round: wait where it is, never straight on (it
                // walked through a table so).
                let to = match route(obstacles, false, at, entry, around.filter(|_| entry == chosen)) {
                    Route::Via(corner) => corner,
                    Route::Clear => entry,
                    Route::Stuck => {
                        *self = Approach::Leaving { entry, around: None };
                        return Order::Stop { heading: walking, rate: 0.0 };
                    }
                };
                *self = Approach::Leaving { entry, around: (to != entry).then_some(to) };
                let toward = heading_of(flat(to - at));
                Order::Walk { speed: slowed(speed, toward), heading: toward, rate: 2.0 }
            }
            Approach::Walking { side, on_arc, pace, around, committed } => {
                let Some((left, toward, on_arc)) = follow(at, spot, heading, side, on_arc) else {
                    // Walked inside the circle on the straight: start over.
                    *self = Approach::Idle;
                    return Order::Stop { heading, rate: super::facing::Facing::default().turn_rate };
                };
                // A straight through an obstacle goes round it first
                // (`route`), never stopping on the way; round, the path is
                // planned afresh (the shorter circle from beside a chair is
                // not the one from behind it). Going round, toward whichever
                // circle is the shorter way from where it now is: held to the
                // one chosen behind the chair, whose tangent lay on the
                // chair's side, it went back for a corner behind.
                // From an entry, committed: its whole final approach was
                // checked clear there (`entry`); re-routed toward the tangent
                // from on it, a walk at a table found no way and stood.
                let best = if around.is_some() { path(at, spot, heading) } else { path_turning(at, spot, heading, side) };
                let routed = if on_arc || committed { None } else { best.map(|path| (path.side, route(obstacles, !obstacles.is_empty(), at, path.tangent, around))) };
                match routed {
                    Some((side, Route::Via(corner))) => {
                        *self = Approach::Walking { side, on_arc, pace: None, around: Some(corner), committed };
                        let toward = heading_of(flat(corner - at));
                        return Order::Walk { speed: slowed(speed, toward), heading: toward, rate: 2.0 };
                    }
                    // No way round from here: on to the corner it was going
                    // to, else by an entry. Walked straight on instead, it went
                    // through its chair to a spot it could not reach.
                    Some((_, Route::Stuck)) => {
                        if let Some(corner) = around {
                            let toward = heading_of(flat(corner - at));
                            return Order::Walk { speed: slowed(speed, toward), heading: toward, rate: 2.0 };
                        }
                        *self = Approach::Leaving { entry: entry(obstacles, at, spot, heading), around: None };
                        return self.advance(at, walking, gait, spot, heading, speed, obstacles);
                    }
                    _ => {}
                }
                if around.is_some() {
                    *self = Approach::Idle;
                    return self.advance(at, walking, gait, spot, heading, speed, obstacles);
                }
                // Told to stop when that stop ends nearer the spot than the
                // next one could; paced, that is on it.
                // Only once paced: from a full-speed stride (1.4 m at 1.2 m/s)
                // it stopped 1.3 m out, before it had slowed at all.
                // And walking at that pace: a stop decided on a stride from
                // another speed (0.72 m/s, paced at 0.39) ended mid-turn,
                // 25 cm short, beside the chair.
                // Of this stop and the next, half a stride on, the better
                // (`miss_cost`): past the end costs more.
                let at_pace = pace.is_some_and(|(pace, _)| (gait.speed - pace).abs() < 0.02);
                let short = left - stop_distance(gait.cycle, gait.stride);
                if at_pace && miss_cost(short) <= miss_cost(short - 0.5 * gait.stride) {
                    *self = Approach::Stopping { side: Some(side), on_arc };
                    return Order::Stop { heading: toward, rate: turning(gait.speed) };
                }
                // Paced only once lined up with the path: paced to 0.8 m/s
                // while turning onto a short straight from an entry, it swung
                // wide and met the circle 1.3 rad off its heading.
                let lined_up = super::facing::shortest_angle(toward - walking).abs() <= SHARP_TURN;
                let pace = if left > FIT_WITHIN || !lined_up {
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
                *self = Approach::Walking { side, on_arc, pace, around: None, committed };
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

    /// The stride the model walk takes at 0.54 m/s (the live one's, measured
    /// at the turning pace when its stride was held from there down).
    const STRIDE: f32 = 0.772;
    const STRIDE_SPEED: f32 = 0.54;

    /// Where the model walk's stride grows with speed: `puppet_base`'s, its
    /// legs 0.888 m, placing itself in short steps (0.25-2.1 m/s).
    fn stride_speeds() -> (f32, f32) {
        gait::stride_speeds_with_steps(0.888, gait::SHORT_STEPS)
    }

    /// The model walk's stride at `speed`.
    fn stride_of(speed: f32) -> f32 {
        stride_at(speed, STRIDE, STRIDE_SPEED, stride_speeds())
    }

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
            stride_of(self.speed.max(0.05))
        }

        fn gait(&self) -> Gait {
            let stopped = matches!(self.stopping, Some(Some(left)) if left <= 0.0);
            Gait { cycle: self.cycle, stride: self.stride(), speed: if stopped { 0.0 } else { self.speed }, stopped, stride_speeds: stride_speeds() }
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
    /// `arrive`, round `obstacles`: where it ended, facing, how far it
    /// walked, and every point it walked through.
    fn walk(from: Vec3, heading: f32, spot: Vec3, arrive: f32, cycle: f32, obstacles: &[Footprint]) -> (Vec3, f32, f32, Vec<Vec3>) {
        let dt = 1.0 / 60.0;
        let mut walk = Walk { at: from, facing: heading, cycle, speed: 0.0, stopping: None, walked: 0.0 };
        let mut approach = Approach::default();
        let mut track = Vec::new();
        for _ in 0..60 * 60 {
            let order = approach.advance(walk.at, walk.facing, &walk.gait(), spot, arrive, 1.2, obstacles);
            if order == Order::Arrived {
                return (walk.at, walk.facing, walk.walked, track);
            }
            walk.step(order, dt);
            track.push(walk.at);
        }
        panic!("never arrived from {from} heading {heading}");
    }

    /// Whether the seat makes up a walk ending at `at` for a turn ending at
    /// `turn_end` heading `arrive` (`sitting::Seat`, its back and across
    /// ranges round the spot [`TURN_AHEAD`] behind it), with 2 cm to spare:
    /// how far it ended past the spot and to its side.
    fn seat_makes_up(at: Vec3, turn_end: Vec3, arrive: f32) -> Result<(), String> {
        let along = direction_of(arrive);
        let off = Vec3::new(at.x - turn_end.x, 0.0, at.z - turn_end.z) + along * TURN_AHEAD;
        let (past, across) = (off.dot(along), off.dot(Vec3::Y.cross(along)));
        if past.abs() < sitting::SEAT_BACK_RANGE - 0.02 && across.abs() < sitting::SEAT_ACROSS_RANGE - 0.02 {
            Ok(())
        } else {
            Err(format!("ended {past:+.3} m past the spot, {across:+.3} m to its side"))
        }
    }

    /// How far in front of the seated hips the walker's `stand_spot` puts
    /// the spot on `puppet_base`, metres (measured live: 0.40 from the hips'
    /// own offset, and the standing hips 0.11 behind the root).
    const SPOT_IN_FRONT: f32 = 0.51;

    /// The chair whose turn ends at (0, 0, -3), sat on facing +Z: its seat
    /// [`SPOT_IN_FRONT`] behind the spot, the spot [`TURN_AHEAD`] behind
    /// where the turn ends.
    fn chair_behind_the_spot() -> Chair {
        Chair::standard(Vec3::new(0.0, 0.0, -3.0 - SPOT_IN_FRONT - TURN_AHEAD), Vec3::Z)
    }

    #[test]
    fn the_turn_onto_the_spot_keeps_the_body_11_cm_clear_of_the_chair() {
        // Every way of arriving dips a turning radius behind where the turn
        // ends, toward the seat's front corner. On a 0.25 m circle the
        // body's middle came within 5 cm of the chair; on 0.18 m, in short
        // steps, 12 cm. (A pivot instead swings the feet 30 cm out.)
        let chair = chair_behind_the_spot().footprint();
        let spot = Vec3::new(0.0, 0.0, -3.0);
        for (from, heading) in [(Vec3::ZERO, 0.0), (Vec3::new(2.0, 0.0, -3.0), PI * 0.5), (Vec3::new(-2.0, 0.0, -2.5), -PI * 0.5)] {
            let (_, _, _, track) = walk(from, heading, spot, PI, 0.2, &[chair]);
            let clear = clearance(&track, &chair);
            assert!(clear > 0.11, "from {from}: the body's middle came within {clear:.3} m of the chair");
        }
    }

    /// How near a walk's track comes to a footprint, metres: negative inside.
    fn clearance(track: &[Vec3], footprint: &Footprint) -> f32 {
        let half = footprint.size * 0.5;
        track
            .iter()
            .map(|&point| {
                let over = footprint.local(point).abs() - half;
                if over.cmplt(Vec2::ZERO).all() { over.max_element() } else { over.max(Vec2::ZERO).length() }
            })
            .fold(f32::INFINITY, f32::min)
    }

    #[test]
    fn a_chairs_footprint_covers_its_seat_and_backrest() {
        // Turned to face +X: its front edge 0.30 in front of where the hips
        // go, its back 0.18 behind (the backrest), 0.46 across.
        use crate::character::anim::obstacles::{FootObstacles, Footprints};
        let chair = Chair::standard(Vec3::new(1.0, 0.0, 2.0), Vec3::X);
        let footprint = Footprints(vec![chair.footprint()]);
        let inside = |point: Vec3| footprint.clear(point, point, 0.0) != Vec3::ZERO;
        for (point, expect) in [
            (Vec3::new(1.29, 0.0, 2.0), true),
            (Vec3::new(1.31, 0.0, 2.0), false),
            (Vec3::new(0.83, 0.0, 2.0), true),
            (Vec3::new(0.81, 0.0, 2.0), false),
            (Vec3::new(1.0, 0.0, 2.22), true),
            (Vec3::new(1.0, 0.0, 2.24), false),
        ] {
            assert_eq!(inside(point), expect, "{point}");
        }
    }

    #[test]
    fn the_gallerys_chairs_are_reached_from_where_its_character_starts() {
        // The gallery's character starts at the origin; its chairs (`--chair`)
        // at -1.5,-1.5 facing the camera and at 0,-1.5 with its back to the
        // character. Live, the first walked 12 cm into the seat's corner and
        // the second never sat down. The spot SPOT_IN_FRONT of the seat,
        // the turn ending TURN_AHEAD further, as the walker has it.
        for (seat, forward) in [(Vec3::new(-1.5, 0.0, -1.5), Vec3::Z), (Vec3::new(0.0, 0.0, -1.5), Vec3::NEG_Z), (Vec3::new(1.5, 0.0, -2.0), Vec3::NEG_X)] {
            let chair = Chair::standard(seat, forward).footprint();
            let turn_end = seat + forward * (SPOT_IN_FRONT + TURN_AHEAD);
            for heading in [0.0, PI, 0.7] {
                let (at, _, _, track) = walk(Vec3::ZERO, heading, turn_end, heading_of(forward), 0.3, &[chair]);
                if let Err(off) = seat_makes_up(at, turn_end, heading_of(forward)) {
                    panic!("chair at {seat} from heading {heading}: {off}");
                }
                let clear = clearance(&track, &chair);
                assert!(clear > 0.1, "chair at {seat} from heading {heading}: came within {clear:.3} m of it");
            }
        }
    }

    #[test]
    fn a_walk_to_a_seat_at_a_table_goes_round_the_table_and_the_other_chairs() {
        // The playground's dining set: a 1.2 by 0.8 m table and four chairs
        // pulled out 0.75 m, facing it. Routed round its own chair only, the
        // walk went through the table. From each side of the room, to each
        // chair: never into the table or another chair, 10 cm clear of its
        // own, and where the seat makes up the rest.
        let table = Footprint { middle: Vec3::ZERO, forward: Vec3::Z, size: Vec2::new(1.2, 0.8) };
        let chairs: Vec<Chair> = [(-0.3, 1.0), (0.3, 1.0), (-0.3, -1.0), (0.3, -1.0)]
            .into_iter()
            .map(|(x, side): (f32, f32)| {
                let middle = Vec3::new(x, 0.0, side * 1.15);
                let forward = Vec3::new(0.0, 0.0, -side);
                Chair::standard(middle - forward * 0.08, forward)
            })
            .collect();
        for (index, chair) in chairs.iter().enumerate() {
            // Its own chair first, as the walker passes them.
            let mut obstacles = vec![chair.footprint(), table];
            obstacles.extend(chairs.iter().enumerate().filter(|(i, _)| *i != index).map(|(_, c)| c.footprint()));
            let turn_end = chair.seat + chair.forward * (SPOT_IN_FRONT + TURN_AHEAD);
            for (from, heading) in [(Vec3::new(0.0, 0.0, 4.0), 0.0), (Vec3::new(0.0, 0.0, -4.0), PI), (Vec3::new(4.0, 0.0, 0.5), 1.5), (Vec3::new(-4.0, 0.0, -0.5), -1.5)] {
                let (at, _, _, track) = walk(from, heading, turn_end, heading_of(chair.forward), 0.2, &obstacles);
                if let Err(off) = seat_makes_up(at, turn_end, heading_of(chair.forward)) {
                    panic!("chair {index} from {from}: {off}");
                }
                for (i, obstacle) in obstacles.iter().enumerate() {
                    let clear = clearance(&track, obstacle);
                    let needed = if i == 0 { 0.1 } else { 0.0 };
                    assert!(clear > needed, "chair {index} from {from}: came within {clear:.3} m of obstacle {i}");
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
        let chair = chair_behind_the_spot().footprint();
        let spot = Vec3::new(0.0, 0.0, -3.0);
        for (from, heading) in [(Vec3::new(0.0, 0.0, -6.0), 0.0), (Vec3::new(1.0, 0.0, -5.5), PI), (Vec3::new(-0.8, 0.0, -4.5), 0.5)] {
            let (at, _, _, track) = walk(from, heading, spot, PI, 0.1, &[chair]);
            // Less straight is left to pace on once round the chair; still
            // within what the seat makes up (15 cm back, 8 across).
            if let Err(off) = seat_makes_up(at, spot, PI) {
                panic!("from {from}: {off}");
            }
            let closest = track
                .iter()
                .filter(|point| point.distance(spot) > 0.6)
                .map(|&point| {
                    let local = chair.local(point);
                    let outside = (local.abs() - chair.size * 0.5).max(Vec2::ZERO);
                    if local.abs().cmplt(chair.size * 0.5).all() { -1.0 } else { outside.length() }
                })
                .fold(f32::INFINITY, f32::min);
            // Less than CLEAR_OF_CHAIR: a straight may end beside the chair
            // (ENDS_BESIDE), and the walk trails its heading through a turn.
            assert!(closest > CLEAR_OF_CHAIR - 0.1, "from {from}: came within {closest:.2} m of the chair");
            // And through the turn, 10 cm clear (see
            // `the_turn_onto_the_spot_keeps_the_body_11_cm_clear_of_the_chair`).
            let clear = clearance(&track, &chair);
            assert!(clear > 0.1, "from {from}: came within {clear:.3} m of the chair");
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
                let (at, facing, walked, _) = walk(from, heading, spot, arrive, cycle, &[]);
                // Not always on it: from where the stride no longer
                // shortens, the stop lands short where no pace fits.
                if let Err(off) = seat_makes_up(at, spot, arrive) {
                    panic!("from {from} at cycle {cycle}: {off}");
                }
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
                stop_off(left, &Gait { cycle: 0.1, stride: stride_of(TURN_SPEED), speed: TURN_SPEED, stopped: false, stride_speeds: stride_speeds() })
            })
            .fold(0.0, f32::max);
        assert!(worst > 0.24 * stride_of(TURN_SPEED), "unpaced worst {worst:.3} m");
        // Paced for the same, it lands within a centimetre: in short steps
        // (`gait::SHORT_STEPS`). With the walk's own, held below 0.54 m/s,
        // 9 of 100 landed up to 8.5 cm short.
        for i in 0..100 {
            let left = 1.0 + i as f32 * 0.01;
            let gait = Gait { cycle: 0.1, stride: stride_of(TURN_SPEED), speed: TURN_SPEED, stopped: false, stride_speeds: stride_speeds() };
            let pace = fitted_pace(left, &gait);
            let paced = Gait { stride: stride_of(pace), speed: pace, ..gait };
            assert!(stop_off(left, &paced) < 0.01, "left {left}: paced {pace:.2} m/s still {:.3} m off", stop_off(left, &paced));
        }
    }

    #[test]
    fn a_stop_is_weighed_by_how_much_of_the_seats_range_it_takes() {
        // The point a stop `short` of the turn's end stands on, round the
        // circle from the end of a path arriving heading +Z at the origin:
        // how far past the spot (TURN_AHEAD behind) and to its side.
        let path_end = Vec3::ZERO;
        let centre = centre_of(path_end, PI, Side::Left);
        let stands = |short: f32| -> (f32, f32) {
            let point = if short < 0.0 {
                path_end + Vec3::Z * -short
            } else {
                let radial = bevy::math::Quat::from_rotation_y(-short / TURN_RADIUS) * (path_end - centre);
                centre + radial
            };
            (point.z + TURN_AHEAD, point.x)
        };
        let seat_used = |short: f32| {
            let (past, across) = stands(short);
            (past.abs() / sitting::SEAT_BACK_RANGE).max(across.abs() / sitting::SEAT_ACROSS_RANGE)
        };
        // Past and short by a few centimetres: past takes more.
        for off in [0.03, 0.06, 0.09] {
            assert!(miss_cost(-off) > miss_cost(off), "{off}");
            assert!(seat_used(-off) > seat_used(off), "{off}");
        }
        // 30 cm short is 16 cm to the side: worse than 4 cm past, which the
        // seat takes. By distance alone the short one is nearer.
        assert!(miss_cost(0.3) > miss_cost(-0.04));
        assert!(seat_used(0.3) > 1.0 && seat_used(-0.04) < 1.0);
        // And both agree where the seat stops taking it up.
        for short in [-0.06, -0.04, 0.05, 0.08, 0.15] {
            assert_eq!(miss_cost(short) > 1.0, seat_used(short) > 1.0, "{short}");
        }
        // Ever worse the further short, past the half circle too: held
        // there, two stops a step apart cost the same, and the walk stopped
        // a metre out.
        for i in 0..200 {
            let short = i as f32 * 0.01;
            assert!(miss_cost(short + 0.01) > miss_cost(short), "{short}");
        }
    }

    #[test]
    fn a_pace_below_where_the_stride_stops_shortening_does_not_shorten_it() {
        // Live, paced to 0.30 m/s on a stride measured at 0.61, the walk
        // stopped 20 cm past its spot: the gait's stride is held below
        // 0.54 m/s (`gait::stride_speeds`), and only its cadence drops.
        // Placing itself in short steps, below 0.25 m/s.
        let amplitude = |params: gait::GaitParams| match params.curves {
            gait::LegCurves::Measured { amplitude } => amplitude,
            _ => unreachable!(),
        };
        for (speeds, walk) in [
            (gait::stride_speeds(0.888), &(|speed| gait::GaitParams::walking_for(speed, 0.888)) as &dyn Fn(f32) -> gait::GaitParams),
            (stride_speeds(), &|speed| gait::GaitParams::walking_with_steps(speed, 0.888, gait::SHORT_STEPS)),
        ] {
            let slowest = speeds.0;
            assert!((stride_at(slowest * 0.6, 0.833, 0.61, speeds) - stride_at(slowest, 0.833, 0.61, speeds)).abs() < 1.0e-6);
            // As the gait itself takes it, on the rig the model's legs are.
            assert_eq!(amplitude(walk(slowest * 0.6)), amplitude(walk(slowest)));
            assert!(amplitude(walk(slowest + 0.05)) > amplitude(walk(slowest)));
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
        let standing = Gait { cycle: 0.0, stride: 0.0, speed: 0.0, stopped: true, stride_speeds: stride_speeds() };
        let spot = Vec3::new(0.02, 0.0, 0.0);
        assert!(matches!(approach.advance(Vec3::ZERO, 0.1, &standing, spot, 0.0, 1.0, &[]), Order::Stop { heading: 0.0, .. }));
        assert_eq!(approach.advance(Vec3::ZERO, 0.0, &standing, spot, 0.0, 1.0, &[]), Order::Arrived);
    }
}
