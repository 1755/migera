//! Running up a wall: step 8 of the parkour design
//! (`docs/knowledge/character-animation/parkour/`), first part.
//!
//! Running at a wall, it takes off from one foot (a running leap,
//! [`Jump::from_run`]), plants the other on the face about a metre up, and
//! drives up off it for about a third of a second; then it flies on as a
//! fall ([`Falling`]) aimed at the wall's lip, which it catches if its hands
//! reach it, or lands at the wall's foot.
//!
//! Measured (`parkour-movement-data`): a wall climb comes in at 4.7 m/s, its
//! last ground step 1.17 m out, its first wall step 1.0 m up (Croft et al.
//! 2019). One foot's run up (Lawson 2015, not peer reviewed): the COM leaves
//! the floor rising 2.93 m/s, meets the wall 0.06 s later, leaves it 0.37 s
//! after that, 0.84 m higher, rising 1.55 m/s.
//!
//! - **Take-off**: the leap's own, its lead (free) foot brought onto its
//!   hold on the face as it meets it.
//! - **On the wall**: the hips on a Hermite curve from how the leap brought
//!   them to how they leave, the foot held on the face, the take-off leg's
//!   knee driven up, the arms swung up toward the lip.
//! - **Off it**: a fall from there at the hips' velocity, aimed at the lip
//!   and held off the wall.

use bevy::math::{Quat, Vec3};

use super::geometry::Ledge;
use super::Falling;
use crate::character::anim::armik::{solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::{Jump, JumpAsk, JumpPhase, RunStart, ARMS_UP, GRAVITY};
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::{knee_toward, place_ankle};
use crate::character::skeleton::Bone;

/// Leaving the floor, the COM rises this fast, m/s (Lawson: 2.93).
const UP_SPEED: f32 = 2.9;
/// The take-off asked to land this far, toe to toe, metres: short, so it
/// brakes all a leap can.
const BRAKED_TO: f32 = 0.3;
/// From leaving the floor to the foot on the wall, seconds (Lawson: 0.06;
/// so short, at 4.5 m/s the lead foot came from the run's swing onto its
/// hold at 14 m/s about the hips).
const TO_WALL: f32 = 0.1;
/// The foot on the wall, seconds (Lawson: about 0.37).
const ON_WALL: f32 = 0.37;
/// The foot's ball on the face this high over the floor, metres (Croft:
/// the first wall step 1.0 m up).
const FOOT_UP: f32 = 1.0;
/// The foot pitched toes-up on the face, radians (as a braced hang's).
const TOES_UP: f32 = 1.1;
/// The lead foot comes onto its hold over this long before it meets the
/// wall, seconds (in 0.2 s, from a take-off 0.25 m far at 4.5 m/s, a toe
/// went 15 m/s about the hips).
const ONTO_WALL: f32 = 0.3;
/// The hips this far out from the face as the foot meets it, metres: where
/// the walker takes off from; and as they leave it, metres.
const HIPS_MEETING: f32 = 0.75;
const HIPS_LEAVING: f32 = 0.42;
/// The hips rise at most this much on the wall, metres (Lawson: the COM
/// 0.84); as far as the leg on the face reaches at this share of its
/// length.
const RISE: f32 = 0.84;
const LEAVE_LEG: f32 = 0.96;
/// The hold no farther from the socket than this share of the leg as the
/// foot meets it.
const MEETING_LEG: f32 = 0.98;
/// Leaving, pushed out from the face this fast, m/s (none: the fall's hold
/// off the wall keeps it out; pushed out 0.3 m/s, the hips drifted out of
/// the hands' reach of the lip); rising at least, and at most, this fast,
/// m/s (one foot's run up leaves at 1.55, Lawson; a hard one higher).
const PUSH_OUT: f32 = 0.0;
const LEAST_UP: f32 = 1.0;
const MOST_UP: f32 = 2.6;
/// The shoulders top out this far under the lip, metres (a leap's catch,
/// `hang::leap`).
const UNDER_LIP: f32 = 0.08;
/// The shoulders topping out this much lower still reach the lip, metres:
/// the hands reach 0.9 of the arm (0.45 m) from them, the hips 0.42 out
/// (measured: a 2.6 m wall caught, 2.7 m not).
const REACH_SLACK: f32 = 0.1;
/// The pose goes from the leap's to the wall's over this long, seconds.
const BLEND: f32 = 0.1;
/// On the wall the trunk leans this far toward it as the foot meets it,
/// upright leaving, radians; the arms swing up to this many times the
/// jump's arms up (nearly overhead): leant in 0.25 rad, the arms swung
/// forward-up 2.6 rad, a hand went 13-17 cm into the wall.
const LEAN: f32 = 0.0;
const ARMS: f32 = 1.45;
const ELBOWS_MID: f32 = 2.0;
/// The foot on the wall, its knee points up, this much out from the face
/// and (both knees) this much out to its own side for each unit up: up and
/// a little out from the face, at 4.5 m/s it went 2.7 cm in as the hips
/// came nearest.
const KNEE_OUT: f32 = 0.3;
const KNEE_ASIDE: f32 = 0.5;
/// The take-off leg's knee drives up: its ankle this far ahead of and
/// under the hips, metres, by this share of the time on the wall.
const DRIVE: (f32, f32) = (0.15, 0.45);
const DRIVE_BY: f32 = 0.7;
/// The driven ankle kept at least this far out from the face, metres; and
/// the wrists under the lip (a fall's `HANDS_OFF_WALL`).
const DRIVE_OFF: f32 = 0.25;
const HANDS_OFF: f32 = 0.1;
/// An elbow nearer the face than this plus the ease, metres, is turned
/// along it, wholly by this near.
const ELBOW_OFF: f32 = 0.05;
const ELBOW_EASE: f32 = 0.15;
/// Leaving, the foot on the wall peels off it this fast, m/s.
const PEEL_OFF: f32 = 1.0;
/// The most the run meets the wall off square, radians.
pub const MOST_SLANT: f32 = 0.3;
/// The hips brake on the wall at most this hard, m/s² (3 g: a foot planted
/// on a wall at a run).
const MOST_BRAKE: f32 = 3.0 * GRAVITY;

/// A kick off a wall toward a lip (a tic-tac, [`WallRun::kick`]): met up to
/// this far off square, radians; the foot on the wall at least and at most
/// this long, seconds (no tic-tac data: a running jump's take-off contact
/// to a run up's 0.37 s), tried in steps this long; where it leaves found
/// in this many passes.
pub const MOST_KICK_SLANT: f32 = 1.1;
const KICK_ON_WALL: (f32, f32) = (0.15, 0.35);
const KICK_STEP: f32 = 0.01;
/// A kick's foot higher on the face than a run up's, its ball this high,
/// metres (about the hips' height: at a run up's 1.0 m the leg, nearly
/// level, lifted the hips 0.24 m while the flight wanted 0.39, braking
/// them at 5.8 g).
const KICK_FOOT_UP: f32 = 1.2;
/// A wall to kick off reaches the floor within this, metres, and stands
/// this far over the foot's hold at least, metres.
const FLOOR_SLACK: f32 = 0.05;
const ABOVE_HOLD: f32 = 0.3;
/// Kicked from the air, the foot's ball meets the face this far above the
/// hips, metres (a running kick's 1.2 m hold is about the hips' height as
/// they meet the wall), the foot brought onto it over this long, seconds
/// (at once, a toe went 43-58 m/s in a frame; over 0.15 s, the knee
/// 14-15 m/s).
const AIR_FOOT: f32 = 0.0;
pub const AIR_ONTO: f32 = 0.2;
/// Kicking across to another wall, the flight rises this fast, m/s, and
/// meets that wall as it tops out. That wall faces the one kicked within
/// this much of square, radians, this far across, metres (tested across
/// 1.8-2.2 m), and no lower than this under the lip chained to, metres.
const ACROSS_UP: f32 = 3.0;
const ACROSS_SQUARE: f32 = 0.3;
const ACROSS_GAP: (f32, f32) = (1.6, 2.4);
const ACROSS_BELOW_LIP: f32 = 0.5;
/// A kick's hips meet the wall this far out from it, metres: nearer than a
/// run up's, its foot's hold higher (met 0.75 m out, taken off 0.2 m
/// farther, the leg on the wall could not lift the hips; 0.4 m, could not
/// reach the hold).
const KICK_MEETING: f32 = 0.6;
/// A kick's hips met at most this far from [`KICK_MEETING`] out, metres
/// (0.4 m along a run 0.6 rad off square nearer, 0.33 m nearer it, a kick
/// planned to a 2.5 m lip missed it).
const KICK_MEETING_SLACK: f32 = 0.2;
/// A kick is taken off up to this much nearer and farther than its best
/// ([`WallRun::kick_takeoff`]), metres, along the run: within it, measured,
/// every kick plans and catches its lip (0.2 m farther at 4.5 m/s, the hips
/// braked on the wall at 3.1 g).
pub const KICK_TAKEOFF: (f32, f32) = (0.2, 0.1);
const KICK_PASSES: usize = 4;
/// The kick's flight to the lip goes across at most this fast, m/s, up at
/// most this fast, m/s, out from the wall kicked at least this fast, m/s,
/// and rises at least this much, metres (as a leap's from a hang plans its
/// catch, `hang::leap`).
const KICK_FASTEST: f32 = 4.0;
/// A kick's flight to its lip lasts this long at least, seconds, falling
/// into the catch past its top if the lip is not far above: timed to its
/// top alone, a lip 0.1 m above and 0.55 m across (kicked from the air up
/// a shaft) was crossed at 3.9 m/s and the hips braked at 6 g reversing.
const KICK_LEAST_FLIGHT: f32 = 0.3;
const KICK_MOST_UP: f32 = 3.5;
const KICK_LEAST_OUT: f32 = 0.5;
const KICK_LEAST_RISE: f32 = 0.1;
/// The catch planned with the hips this far out from the lip's face,
/// metres (a braced hang's, `hang::leap`).
const CATCH_OUT: f32 = 0.42;
/// Kicked off toward a lip, it turns to face it over this long in the air,
/// seconds; caught this far from its ends at least, metres (a leap's).
const KICK_SPIN: f32 = 0.4;
const END_MARGIN: f32 = 0.35;
/// The catch planned on the lip nearest a point this far out from where
/// the hips leave the wall kicked, metres: nearest where they leave, in a
/// corner the flight went along the wall, not off it.
const KICK_AWAY: f32 = 0.5;

const LEGS: [(Bone, Bone, Bone); 2] = [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot)];

/// Eased 0-1 over `span` from `from`.
fn ease(t: f32, from: f32, span: f32) -> f32 {
    smoothstep(((t - from) / span.max(1.0e-6)).clamp(0.0, 1.0))
}

/// A cubic Hermite from `a` (velocity `va`) to `b` (velocity `vb`) over
/// `span` seconds, at `t`: position and velocity.
fn hermite(a: Vec3, va: Vec3, b: Vec3, vb: Vec3, span: f32, t: f32) -> (Vec3, Vec3) {
    let s = (t / span).clamp(0.0, 1.0);
    let (s2, s3) = (s * s, s * s * s);
    let at = a * (2.0 * s3 - 3.0 * s2 + 1.0) + va * (span * (s3 - 2.0 * s2 + s)) + b * (3.0 * s2 - 2.0 * s3) + vb * (span * (s3 - s2));
    let rate = (a * (6.0 * s2 - 6.0 * s) + va * (span * (3.0 * s2 - 4.0 * s + 1.0)) + b * (6.0 * s - 6.0 * s2) + vb * (span * (3.0 * s2 - 2.0 * s))) / span;
    (at, rate)
}

/// The take-off from a run up a wall: a leap rising [`UP_SPEED`], asked to
/// land short ([`BRAKED_TO`]), so its plant leg brakes it as hard as a leap
/// does (`jump::leap::MOST_LOSS_PER_UP`). The wall met at the run's speed,
/// at 4.5 m/s the hips braked at 2.8 g on it; Lawson's run up meets the
/// wall at 2.35 m/s.
fn take_off(start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> Jump {
    Jump::from_run(JumpAsk::running(UP_SPEED * UP_SPEED / (2.0 * GRAVITY), BRAKED_TO), start, stood, rig)
}

/// A run up a wall, as planned at take-off.
#[derive(Debug, Clone)]
pub struct WallRun {
    /// The take-off leap (its frame: the pose's at `origin`, turned `turn`).
    jump: Jump,
    origin: Vec3,
    yaw: f32,
    turn: Quat,
    /// The wall (its lip and face), the foot IK's drop.
    wall: Ledge,
    drop: f32,
    stood: LocalPose,
    rig: std::sync::Arc<RigGeometry>,
    /// The leg whose foot goes on the wall (the lead).
    lead: usize,
    /// When the foot meets the wall and when it leaves, seconds into the
    /// jump; seconds into it now.
    contact: f32,
    leave: f32,
    t: f32,
    /// The leap's pose as the foot meets the wall; the hips then and their
    /// velocity, and leaving (the world).
    meeting: LocalPose,
    hips: (Vec3, Vec3),
    leaving: (Vec3, Vec3),
    /// The foot's ankle on the wall and its world rotation there.
    hold: (Vec3, Quat),
    /// The standing hips (the pose's frame).
    body_hips: Vec3,
    /// How long the foot is on the wall, seconds.
    on_wall: f32,
    /// Kicking off toward a lip or another wall ([`Self::kick`]): which,
    /// and how far it turns in the air to face it, radians.
    target: Option<KickTarget>,
    spin: f32,
    /// Kicked off from the air ([`Self::kick_from_air`]): no take-off; the
    /// fall it came from, flying on until the foot meets the wall.
    from_air: bool,
    approach: Option<Box<Falling>>,
}

/// What a kick off a wall flies to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KickTarget {
    /// A lip, caught as a leap from a hang catches it.
    Lip(Ledge),
    /// Another wall's face (a ledge's), met high to kick off it in turn
    /// (chaining, [`WallRun::kick_from_air`]).
    Wall(Ledge),
}

impl KickTarget {
    fn ledge(&self) -> Ledge {
        match *self {
            KickTarget::Lip(ledge) | KickTarget::Wall(ledge) => ledge,
        }
    }
}

/// How the foot meets the wall, the rest planned from it alike: after a
/// running take-off ([`take_off`]), or from the air.
struct Contact {
    jump: Jump,
    origin: Vec3,
    yaw: f32,
    contact: f32,
    meeting: LocalPose,
    hips: Vec3,
    velocity: Vec3,
    lead: usize,
    /// The foot's ball on the face this high, the world.
    ball_y: f32,
    from_air: bool,
}

impl WallRun {
    /// A run up `wall` (its lip ledge, its face reaching the floor) by a
    /// walker running at `start.speed`, its root at `origin` turned `yaw`
    /// as the foot of `start.leg` came down, its hips `drop` below
    /// standing: `None` if it runs at the wall more than [`MOST_SLANT`] off
    /// square, or the take-off is so far from or near the wall that its
    /// hips would not meet it right.
    #[allow(clippy::too_many_arguments)]
    pub fn plan(wall: &Ledge, origin: Vec3, yaw: f32, start: RunStart, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        Self::plan_on(wall, None, origin, yaw, start, drop, stood, rig)
    }

    /// A kick off `wall` toward `target`'s lip (a tic-tac): run at the wall
    /// up to [`MOST_KICK_SLANT`] off square, plant a foot on it as a run up
    /// does, and push off up and across, turning in the air to catch the
    /// lip as a leap from a hang does. `None` as [`Self::plan`], or if the
    /// lip would need the push into the wall, or more than
    /// [`KICK_MOST_UP`] up.
    #[allow(clippy::too_many_arguments)]
    pub fn kick(wall: &Ledge, target: &Ledge, origin: Vec3, yaw: f32, start: RunStart, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        Self::plan_on(wall, Some(KickTarget::Lip(*target)), origin, yaw, start, drop, stood, rig)
    }

    /// A kick off `wall` across to `other`'s face, met high as the flight
    /// tops out, to kick off that in turn ([`Self::kick_from_air`]): wall to
    /// wall up a gap too high for one kick. `None` as [`Self::kick`].
    #[allow(clippy::too_many_arguments)]
    pub fn kick_across(wall: &Ledge, other: &Ledge, origin: Vec3, yaw: f32, start: RunStart, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        Self::plan_on(wall, Some(KickTarget::Wall(*other)), origin, yaw, start, drop, stood, rig)
    }

    /// A kick off `wall` from the air, toward `target`: falling (or flying
    /// across from another kick) as `falling` is now, it flies on
    /// [`AIR_ONTO`] while the foot nearer the wall comes onto it, as high as
    /// the hips, and the kick goes on as from a run. `None` unless by then
    /// it faces the wall within [`MOST_KICK_SLANT`], its hips within
    /// [`KICK_MEETING_SLACK`] of [`KICK_MEETING`] out of it, or as
    /// [`Self::kick`].
    pub fn kick_from_air(wall: &Ledge, target: KickTarget, falling: &Falling, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let mut ahead = falling.clone();
        ahead.advance(AIR_ONTO);
        if !ahead.airborne() {
            return None;
        }
        let (root, yaw, pose) = (ahead.root(), ahead.facing(), ahead.pose(rig));
        let turn = Quat::from_rotation_y(yaw);
        if -(turn * rig.forward()).dot(wall.out) < MOST_KICK_SLANT.cos() {
            return None;
        }
        let hips = root + turn * forward_kinematics_on(&pose, rig)[Bone::Hips];
        if (wall.out_of(hips) - KICK_MEETING).abs() > KICK_MEETING_SLACK {
            return None;
        }
        let contact = Contact {
            jump: Jump::plan(JumpAsk::up(0.02), stood, rig),
            origin: root,
            yaw,
            contact: AIR_ONTO,
            meeting: pose,
            hips,
            velocity: ahead.hips_velocity(),
            lead: 1 - Self::kick_leg(wall, yaw, stood, rig),
            ball_y: hips.y + AIR_FOOT,
            from_air: true,
        };
        let mut run = Self::finish(wall, Some(target), contact, drop, stood, rig)?;
        run.approach = Some(Box::new(falling.clone()));
        Some(run)
    }

    /// Kicked from the air, the fall it flies on as `t` seconds into it.
    fn approach_at(&self, t: f32) -> Option<Falling> {
        self.approach.as_deref().map(|falling| {
            let mut now = falling.clone();
            now.advance(t);
            now
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn plan_on(wall: &Ledge, target: Option<KickTarget>, origin: Vec3, yaw: f32, start: RunStart, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let turn = Quat::from_rotation_y(yaw);
        let forward = turn * rig.forward();
        let most_slant = if target.is_some() { MOST_KICK_SLANT } else { MOST_SLANT };
        if -forward.dot(wall.out) < most_slant.cos() {
            return None;
        }
        let jump = take_off(start, stood, rig);
        let contact = jump.ends(JumpPhase::Push) + TO_WALL;
        if contact >= jump.ends(JumpPhase::Flight) {
            return None;
        }
        let hips_at = |t: f32| origin + turn * (forward_kinematics_on(&jump.pose_at(t, stood, rig), rig)[Bone::Hips] + rig.forward() * jump.travelled_at(t));
        let meeting = jump.pose_at(contact, stood, rig);
        let hips = hips_at(contact);
        let velocity = (hips_at(contact + 1.0e-3) - hips_at(contact - 1.0e-3)) / 2.0e-3;
        // Met where the hips are near enough the wall, not into it.
        let out = wall.out_of(hips);
        let (meeting_out, slack) = if target.is_some() { (KICK_MEETING, KICK_MEETING_SLACK) } else { (HIPS_MEETING, MEETING_SLACK) };
        if (out - meeting_out).abs() > slack {
            return None;
        }
        // Kicking, the foot nearer the wall goes on it (off the other, it
        // crossed in front: off the right foot at 0.6 rad, a lip 2.5 m
        // high wanted 3.57 m/s up).
        if target.is_some() && start.leg != Self::kick_leg(wall, yaw, stood, rig) {
            return None;
        }
        let foot_up = if target.is_some() { KICK_FOOT_UP } else { FOOT_UP };
        let contact = Contact { jump, origin, yaw, contact, meeting, hips, velocity, lead: 1 - start.leg, ball_y: origin.y + foot_up, from_air: false };
        Self::finish(wall, target, contact, drop, stood, rig)
    }

    /// The rest of a plan from how the foot meets the wall: its hold, the
    /// hips on the wall, how they leave and the flight after.
    fn finish(wall: &Ledge, target: Option<KickTarget>, contact: Contact, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let Contact { jump, origin, yaw, contact, meeting, hips, velocity, lead, ball_y, from_air } = contact;
        let turn = Quat::from_rotation_y(yaw);
        let forward = turn * rig.forward();
        let meets = forward_kinematics_on(&meeting, rig);
        let meets_world = |bone: Bone| origin + turn * (meets[bone] + rig.forward() * jump.travelled_at(contact));
        let out = wall.out_of(hips);
        let stood_at = forward_kinematics_on(stood, rig);
        let stood_world = accumulate_world_rotations(stood, rig);
        // The foot's hold: its ball on the face under the lead socket, toes
        // up.
        let (socket, _, ankle) = LEGS[lead];
        let toe = crate::character::anim::foot::foot_bones(ankle).1;
        let along = wall.along();
        let lateral = (meets_world(socket) - wall.a).dot(along);
        let ball = wall.a + along * lateral + Vec3::Y * (ball_y - wall.a.y);
        let standing = turn * stood_world[ankle];
        let attitude = Quat::from_axis_angle((-wall.out).cross(Vec3::Y).normalize_or(along), TOES_UP) * standing;
        let ankle_at = ball + attitude * standing.inverse() * (turn * (stood_at[ankle] - stood_at[toe]));
        // The hold within the leg's reach as the foot meets it: taken off
        // 0.25 m far, the hips met the wall a metre out, the foot 1.7 cm
        // short of its hold.
        // The leg's length socket to ankle, straight in the rest pose: summed
        // by its segments, the synthetic rig's (shifted a joint: a 7 cm stub
        // for a shin) came out too short to reach any hold.
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let leg = (rest[ankle] - rest[socket]).length();
        if (meets_world(socket) - ankle_at).length() > MEETING_LEG * leg {
            return None;
        }
        // Leaving: the hips risen, nearer the wall, pushed out; rising as
        // fast as brings the hands to the lip near the top of the flight.
        let shoulders = 0.5 * (stood_at[Bone::LeftArm] + stood_at[Bone::RightArm]) - stood_at[Bone::Hips];
        // As high as the leg on its hold reaches (`LEAVE_LEG` of it), the
        // hips this far above the socket: rising Lawson's 0.84 m, the foot
        // was dragged off the face, the leg out of reach (the COM's rise is
        // helped by the ankle's push off the toes, which this pose has not).
        let socket_below = (stood_at[Bone::Hips] - stood_at[socket]).y;
        let reach = LEAVE_LEG * leg;
        // The highest the hips go, the socket `across` from the ankle on its
        // hold seen from above; or seen from above where `at` is.
        let highest = |across: f32| ankle_at.y + (reach * reach - across * across).max(0.0).sqrt() + socket_below;
        let socket_off = turn * (stood_at[socket] - stood_at[Bone::Hips]);
        let highest_at = |at: Vec3| highest((at + socket_off - ankle_at).with_y(0.0).length());
        // A run up's taken as its hips' way out from the face, not the
        // socket's from the ankle (its catch measured so).
        let leave_at = hips + wall.out * (HIPS_LEAVING - out) + Vec3::Y * (highest(HIPS_LEAVING) - hips.y).min(RISE);
        let (leaving, spin, on_wall) = match target {
            // The shoulders topping out just under the lip, as a leap's
            // catch plans (`hang::leap`): planned an arm's reach under it,
            // the hands never came within reach of it on the way down.
            None => {
                let apex = wall.height() - UNDER_LIP - shoulders.y;
                let up = (2.0 * GRAVITY * (apex - leave_at.y).max(0.0)).sqrt().clamp(LEAST_UP, MOST_UP);
                ((leave_at, wall.out * PUSH_OUT + Vec3::Y * up), 0.0, ON_WALL)
            }
            // Kicking: caught as a leap from a hang catches, the shoulders
            // just under the lip, the hips out from its face; at the top of
            // the flight, or falling past it if crossing takes longer.
            Some(target) => {
                let flight = |leave_at: Vec3| match target {
                    KickTarget::Lip(target) => {
                        let lip = target.nearest(leave_at + wall.out * KICK_AWAY, END_MARGIN);
                        let catch_height = lip.y - UNDER_LIP - shoulders.y;
                        let catch = (lip + target.out * CATCH_OUT).with_y(catch_height);
                        let rise = (catch_height - leave_at.y).max(KICK_LEAST_RISE);
                        let to_top = (2.0 * rise / GRAVITY).sqrt();
                        let seconds = to_top.max((catch - leave_at).with_y(0.0).length() / KICK_FASTEST).max(KICK_LEAST_FLIGHT);
                        let up = (catch_height - leave_at.y) / seconds + 0.5 * GRAVITY * seconds;
                        (catch - leave_at).with_y(0.0) / seconds + Vec3::Y * up
                    }
                    // Across to another wall, met as the flight tops out
                    // (`ACROSS_UP` up), the hips as far out of its face as a
                    // kick meets a wall; later if crossing takes longer.
                    KickTarget::Wall(other) => {
                        let face = other.nearest(leave_at + wall.out * KICK_AWAY, END_MARGIN) + other.out * KICK_MEETING;
                        let across = (face - leave_at).with_y(0.0);
                        let seconds = (ACROSS_UP / GRAVITY).max(across.length() / KICK_FASTEST);
                        across / seconds + Vec3::Y * ACROSS_UP
                    }
                };
                // Leaving where a steady push from how the hips met the
                // wall to how they leave it brings them (left 0.3 m up
                // and carried half the way along, ends disagreeing with
                // their velocities, the hips braked at 6.4 g): on the wall
                // as long as that takes to bring them as high as the leg
                // reaches, no nearer the face than a run up leaves it; the
                // flight planned again from each.
                let leave_after = |on_wall: f32| {
                    let mut leave_at = leave_at;
                    for _ in 0..KICK_PASSES {
                        leave_at = hips + (velocity + flight(leave_at)) * (0.5 * on_wall);
                        leave_at += wall.out * (wall.out_of(leave_at).max(HIPS_LEAVING) - wall.out_of(leave_at));
                    }
                    leave_at
                };
                // Under a lip, low enough to rise into its catch: kicked from
                // the air at a lip not far above, the longest push the leg
                // allowed left 0.67 m up, over the catch, the flight falling
                // into it and the hips braked at 7 g.
                let under_catch = |at: Vec3| match target {
                    KickTarget::Lip(lip) => at.y <= lip.nearest(at + wall.out * KICK_AWAY, END_MARGIN).y - UNDER_LIP - shoulders.y - KICK_LEAST_RISE,
                    KickTarget::Wall(_) => true,
                };
                // The longest on the wall the leg reaches through.
                let steps = ((KICK_ON_WALL.1 - KICK_ON_WALL.0) / KICK_STEP).round() as usize;
                let (on_wall, leave_at) = (0..=steps)
                    .map(|k| KICK_ON_WALL.1 - KICK_STEP * k as f32)
                    .map(|on_wall| (on_wall, leave_after(on_wall)))
                    .find(|(_, at)| at.y <= highest_at(*at) && under_catch(*at))
                    .unwrap_or_else(|| {
                        let at = leave_after(KICK_ON_WALL.0);
                        (KICK_ON_WALL.0, at.with_y(at.y.min(highest_at(at))))
                    });
                let leaving = flight(leave_at);
                // Off the wall kicked, not into it; not more than a kick's
                // push up.
                if leaving.y > KICK_MOST_UP || leaving.dot(wall.out) < KICK_LEAST_OUT {
                    return None;
                }
                let heading = |d: Vec3| d.x.atan2(d.z);
                let spin = (heading(-target.ledge().out) - heading(forward) + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI;
                ((leave_at, leaving), spin, on_wall)
            }
        };
        // Room on the wall to brake the run within `MOST_BRAKE`: taken off
        // 0.25 m nearer at 4.5 m/s, the hips braked at 3.5 g.
        let braking = (0..=40).map(|k| {
            let t = on_wall * k as f32 / 40.0;
            let (_, a) = hermite(hips, velocity, leaving.0, leaving.1, on_wall, t);
            let (_, b) = hermite(hips, velocity, leaving.0, leaving.1, on_wall, (t + 1.0e-3).min(on_wall));
            ((b - a) / 1.0e-3).length()
        });
        let most = braking.fold(0.0, f32::max);
        if most > MOST_BRAKE {
            return None;
        }
        Some(Self {
            jump,
            origin,
            yaw,
            turn,
            wall: *wall,
            drop,
            stood: *stood,
            rig: std::sync::Arc::new(rig.clone()),
            lead,
            contact,
            leave: contact + on_wall,
            t: 0.0,
            meeting,
            hips: (hips, velocity),
            leaving,
            hold: (ankle_at, attitude),
            body_hips: stood_at[Bone::Hips],
            on_wall,
            target,
            spin,
            from_air,
            approach: None,
        })
    }

    /// How far before a wall's face (along the way of running) a run's
    /// foot best comes down to run up it from: its hips [`HIPS_MEETING`]
    /// out as the other foot meets the face, run at `slant` off square.
    pub fn takeoff(start: RunStart, slant: f32, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        Self::takeoff_meeting(start, slant, HIPS_MEETING, stood, rig)
    }

    /// As [`Self::takeoff`], to kick off it: the hips [`KICK_MEETING`] out.
    pub fn kick_takeoff(start: RunStart, slant: f32, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        Self::takeoff_meeting(start, slant, KICK_MEETING, stood, rig)
    }

    fn takeoff_meeting(start: RunStart, slant: f32, meeting: f32, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        let jump = take_off(start, stood, rig);
        let contact = jump.ends(JumpPhase::Push) + TO_WALL;
        let hips = forward_kinematics_on(&jump.pose_at(contact, stood, rig), rig)[Bone::Hips];
        hips.dot(rig.forward()) + jump.travelled_at(contact) + meeting / slant.cos().max(0.2)
    }

    /// The wall among `ledges` a run from `origin` along `forward` (the
    /// world) would kick off toward `target`: the nearest ahead met within
    /// [`MOST_KICK_SLANT`] of square, its face reaching the floor and above
    /// the foot's hold; and how the run meets it.
    pub fn kick_off(ledges: &[Ledge], target: &Ledge, origin: Vec3, forward: Vec3) -> Option<(Ledge, super::vault::Obstacle)> {
        let ahead = |own: bool| {
            ledges
                .iter()
                .chain(own.then_some(target))
                .filter(|ledge| (own || *ledge != target) && ledge.height() - ledge.wall_below <= origin.y + FLOOR_SLACK && ledge.height() > origin.y + KICK_FOOT_UP + ABOVE_HOLD)
                .filter_map(|ledge| super::vault::Obstacle::ahead(ledge, origin, forward, MOST_KICK_SLANT).map(|face| (*ledge, face)))
                .min_by(|(_, a), (_, b)| a.near.total_cmp(&b.near))
        };
        // Up a shaft the wall to kick first is the lip's own (to kick across
        // from, [`Self::across_from`]); only if no other is ahead.
        ahead(false).or_else(|| ahead(true))
    }

    /// The wall among `ledges` facing `wall` across a gap a kick crosses
    /// ([`ACROSS_GAP`]), tall enough to kick off from the air toward
    /// `target`: the nearest.
    pub fn across_from(ledges: &[Ledge], wall: &Ledge, target: &Ledge) -> Option<Ledge> {
        ledges
            .iter()
            .filter(|other| *other != wall && other.out.dot(wall.out) < -ACROSS_SQUARE.cos() && other.height() >= target.height() - ACROSS_BELOW_LIP)
            .map(|other| (*other, wall.out_of(other.nearest(wall.a, 0.0))))
            .filter(|&(_, gap)| (ACROSS_GAP.0..=ACROSS_GAP.1).contains(&gap))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(other, _)| other)
    }

    /// The leg a kick off `wall` takes off from, turned `yaw`: the one
    /// farther from it, the nearer foot going on the wall.
    pub fn kick_leg(wall: &Ledge, yaw: f32, stood: &LocalPose, rig: &RigGeometry) -> usize {
        let at = forward_kinematics_on(stood, rig);
        let out = |leg: usize| (Quat::from_rotation_y(yaw) * at[LEGS[leg].0]).dot(wall.out);
        if out(0) > out(1) { 0 } else { 1 }
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t += dt;
        if self.t < self.contact {
            self.jump.advance(dt.min(self.contact - self.jump.elapsed()).max(0.0));
        }
    }

    /// Seconds into it.
    pub fn elapsed(&self) -> f32 {
        self.t
    }

    /// Whether its foot has left the wall: it flies on as a fall
    /// ([`Self::release`]).
    pub fn is_released(&self) -> bool {
        self.t >= self.leave
    }

    /// Whether its take-off foot is on the floor.
    pub fn feet_down(&self) -> [bool; 2] {
        if !self.from_air && self.t < self.jump.ends(JumpPhase::Push) { [0, 1].map(|leg| leg != self.lead) } else { [false; 2] }
    }

    /// The facing (radians about `+Y`): kicked from the air, the fall's
    /// until the foot meets the wall.
    pub fn facing(&self) -> f32 {
        match self.approach_at(self.t).filter(|_| self.t < self.contact) {
            Some(falling) => falling.facing(),
            None => self.yaw,
        }
    }

    /// The wall it runs up.
    pub fn wall(&self) -> &Ledge {
        &self.wall
    }

    /// The hips (the world) and their velocity on the wall at `t`.
    fn hips_on_wall(&self, t: f32) -> (Vec3, Vec3) {
        let ((a, va), (b, vb)) = (self.hips, self.leaving);
        hermite(a, va, b, vb, self.on_wall, t - self.contact)
    }

    /// The root now (the world).
    pub fn root(&self) -> Vec3 {
        self.root_at(self.t)
    }

    fn root_at(&self, t: f32) -> Vec3 {
        if t < self.contact {
            if let Some(falling) = self.approach_at(t) {
                return falling.root();
            }
            return self.origin + self.turn * (self.rig.forward() * self.jump.travelled_at(t));
        }
        self.hips_on_wall(t).0 - self.turn * self.body_hips
    }

    /// The pose now, on the rig it was planned on, in the walker's frame at
    /// [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self) -> LocalPose {
        self.pose_at(self.t)
    }

    fn pose_at(&self, t: f32) -> LocalPose {
        let rig = &*self.rig;
        let back = self.turn.inverse();
        if t < self.contact {
            // The leap's (kicked from the air, the fall's), its lead foot
            // brought onto its hold: kicked from the air with the foot on
            // the wall at once, a toe went 43-58 m/s in a frame.
            let approach = self.approach_at(t);
            let (mut pose, turn) = match approach.as_ref() {
                Some(falling) => (falling.pose(rig), Quat::from_rotation_y(falling.facing())),
                None => (self.jump.pose_at(t, &self.stood, rig), self.turn),
            };
            // From no earlier than the take-off: a foot already part-way to
            // the hold at the first frame went 14 m/s.
            let from = (self.contact - ONTO_WALL).max(0.0);
            let w = ease(t, from, self.contact - from);
            if w > 0.0 {
                let root = self.root_at(t);
                let at = forward_kinematics_on(&pose, rig);
                let (socket, knee, ankle) = LEGS[self.lead];
                let hold = turn.inverse() * (self.hold.0 - root);
                place_ankle(&mut pose, rig, ankle, at[ankle].lerp(hold, w) - at[Bone::Hips]);
                // As on the wall: aimed ahead before and out after, the knee
                // turned round at 16 m/s as the foot met the face.
                knee_toward(&mut pose, rig, [socket, knee, ankle], self.wall_knee(), w);
                self.turn_foot_in(&mut pose, ankle, self.hold.1, w, turn);
            }
            return pose;
        }
        // Swung as long as a run up's time on the wall: a kick's own 0.16 s
        // swung the hands at 15 m/s; leaving, the fall coasts it on.
        let span = self.on_wall.max(ON_WALL);
        let s = (t - self.contact) / span;
        // The trunk upright from leaning in, the arms swung up, from the
        // leap's pose as the foot met the wall.
        // The elbows bent through the swing's middle, the hands coming up by
        // the body: swung up straight, mid-way they pointed at the wall and
        // went 7-12 cm into it.
        let (swing, elbow) = ARMS_UP;
        let s = s.clamp(0.0, 1.0);
        let arms = smoothstep(s) * ARMS;
        let bent = ELBOWS_MID * (std::f32::consts::PI * s).sin();
        let shaped = crate::character::anim::jump::upper(&self.stood, rig, LEAN * (1.0 - s), (swing * arms, elbow * arms + bent));
        let w = ease(t, self.contact, BLEND);
        let mut pose = shaped;
        for bone in Bone::ALL {
            pose.rotations[bone] = self.meeting.rotations[bone].slerp(shaped.rotations[bone], w);
        }
        pose.root_translation = self.stood.root_translation;
        let root = self.root_at(t);
        let hips = forward_kinematics_on(&pose, rig)[Bone::Hips];
        // The foot held on the wall, its knee up and a little out from it
        // (pointed toward it, it went 2.7 cm in).
        let (socket, knee, ankle) = LEGS[self.lead];
        place_ankle(&mut pose, rig, ankle, back * (self.hold.0 - root) - hips);
        knee_toward(&mut pose, rig, [socket, knee, ankle], self.wall_knee(), 1.0);
        self.turn_foot(&mut pose, ankle, self.hold.1, 1.0);
        // The take-off leg's knee driven up, from where the leap had it.
        let trail = 1 - self.lead;
        let (socket, knee, ankle) = LEGS[trail];
        let met = self.met_ankle(trail);
        let driven = self.hips_on_wall(t).0 + self.turn * (rig.forward() * DRIVE.0 - Vec3::Y * DRIVE.1);
        // Kept out from the face: taken off nearer, the hips came nearer it
        // and the driven knee went 2.7 cm in.
        let driven = driven + self.wall.out * (DRIVE_OFF - self.wall.out_of(driven)).max(0.0);
        let d = ease(t, self.contact, DRIVE_BY * span);
        place_ankle(&mut pose, rig, ankle, back * (met.lerp(driven, d) - root) - hips);
        // Up and out to its side: ahead, it went 5 mm into the wall.
        let aside = rig.left() * if trail == 0 { 1.0 } else { -1.0 };
        knee_toward(&mut pose, rig, [socket, knee, ankle], Vec3::Y + aside * KNEE_ASIDE, d);
        // The hands kept off the face under the lip, as a fall keeps them
        // (taken off near, a hand swinging up went 2 cm in); and the elbows,
        // turned along the face as they come near it (kicking, taken off
        // near, the arm on the wall's side swung its elbow 2-8 cm in).
        let at = forward_kinematics_on(&pose, rig);
        let out = back * self.wall.out;
        for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
            let (wrist, elbow) = (root + self.turn * at[chain.wrist], root + self.turn * at[chain.elbow]);
            let short = HANDS_OFF - self.wall.out_of(wrist);
            let turned = smoothstep(((ELBOW_OFF + ELBOW_EASE - self.wall.out_of(elbow)) / ELBOW_EASE).clamp(0.0, 1.0));
            if (short > 0.0 || turned > 0.0) && wrist.y < self.wall.height() {
                let pole = (at[chain.elbow] - 0.5 * (at[chain.shoulder] + at[chain.wrist])).normalize_or(-rig.forward());
                let pole = (pole - out * pole.dot(out).min(0.0) * turned).normalize_or(pole);
                solve_arm_toward_from(&mut pose, &at, chain, back * (wrist + self.wall.out * short.max(0.0) - root), pole, rig);
            }
        }
        pose
    }

    /// Which way the foot on the wall points its knee (the pose's frame):
    /// up, a little out from the face and out to its side.
    fn wall_knee(&self) -> Vec3 {
        let rig = &*self.rig;
        let aside = rig.left() * if self.lead == 0 { 1.0 } else { -1.0 };
        Vec3::Y - rig.forward() * KNEE_OUT + aside * KNEE_ASIDE
    }

    /// Where leg `leg`'s ankle was as the foot met the wall (the world).
    fn met_ankle(&self, leg: usize) -> Vec3 {
        let at = forward_kinematics_on(&self.meeting, &self.rig);
        self.origin + self.turn * (at[LEGS[leg].2] + self.rig.forward() * self.jump.travelled_at(self.contact))
    }

    /// Turns `ankle`'s foot toward world rotation `attitude`, by `weight`.
    fn turn_foot(&self, pose: &mut LocalPose, ankle: Bone, attitude: Quat, weight: f32) {
        self.turn_foot_in(pose, ankle, attitude, weight, self.turn);
    }

    /// [`Self::turn_foot`], the pose's frame turned `turn` in the world.
    fn turn_foot_in(&self, pose: &mut LocalPose, ankle: Bone, attitude: Quat, weight: f32, turn: Quat) {
        let rig = &*self.rig;
        let now = accumulate_world_rotations(pose, rig)[ankle];
        let wanted = turn.inverse() * attitude;
        let turn = Quat::IDENTITY.slerp(wanted * now.inverse(), weight);
        pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, turn);
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the
    /// spring trails a steady motion (`jump::lead_of`), as the other moves
    /// pose: the run up as it will be that much later.
    pub fn pose_led(&self, springs: &crate::character::anim::rig::BoneSet<crate::character::anim::math::SpringParams>) -> LocalPose {
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
                    // Not past letting go: the fall poses itself from there.
                    let ahead = self.pose_at((self.t + lead).min(self.leave));
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }

    /// Whether its hands can reach the wall's lip from the top of its
    /// flight.
    pub fn reaches_lip(&self) -> bool {
        let rig = &*self.rig;
        let at = forward_kinematics_on(&self.stood, rig);
        let shoulders = 0.5 * (at[Bone::LeftArm] + at[Bone::RightArm]) - at[Bone::Hips];
        // A kick is planned to reach its lip, or not taken.
        if self.target.is_some() {
            return true;
        }
        let (at, velocity) = self.leaving;
        let apex = at.y + velocity.y * velocity.y / (2.0 * GRAVITY);
        // The shoulders up to just under the lip, as planned.
        apex + shoulders.y >= self.wall.height() - UNDER_LIP - REACH_SLACK
    }

    /// The lip it reaches for: the wall's own, or kicking, the one kicked
    /// toward.
    pub fn lip(&self) -> Option<Ledge> {
        match self.target {
            None => Some(self.wall),
            Some(KickTarget::Lip(lip)) => Some(lip),
            Some(KickTarget::Wall(_)) => None,
        }
    }

    /// Kicking across ([`Self::kick_across`]), the wall it flies to.
    pub fn across(&self) -> Option<Ledge> {
        match self.target {
            Some(KickTarget::Wall(other)) => Some(other),
            _ => None,
        }
    }

    /// Let go of the wall: the fall from here at the hips' velocity, aimed
    /// at the lip, held off the wall, landing on the ground `ground` finds.
    pub fn release(&self, ground: &dyn Fn(Vec3) -> Option<f32>) -> Falling {
        let rig = &*self.rig;
        let t = self.t.max(self.leave);
        let pose = self.pose_at(t);
        let root = self.root_at(t);
        let below = ground(root).unwrap_or(self.origin.y);
        let mut falling = Falling::off(root, self.yaw, self.leaving.1, &pose, below, self.drop, &self.stood, rig);
        // The legs and the trunk and arms as they move leaving.
        const READ: f32 = 0.01;
        let ankles = |t: f32| {
            let at = forward_kinematics_on(&self.pose_at(t), rig);
            LEGS.map(|(_, _, ankle)| self.turn * (at[ankle] - at[Bone::Hips]))
        };
        let (now, then) = (ankles(t), ankles(t - READ));
        // The foot on the wall peeling off it: coasting down the face as it
        // was held, its toes levelling went 1.4 cm into it.
        falling.coast_legs([0, 1].map(|side| (now[side] - then[side]) / READ + if side == self.lead { self.wall.out * PEEL_OFF } else { Vec3::ZERO }));
        let before = self.pose_at(t - READ);
        let mut ahead = pose;
        let on = 1.0 + Falling::COAST_AHEAD / READ;
        for bone in Bone::ALL {
            ahead.rotations[bone] = before.rotations[bone].slerp(pose.rotations[bone], on);
        }
        falling.coast_upper(ahead);
        // Kicking, turned in the air to face the lip kicked toward.
        if self.spin.abs() > 1.0e-3 {
            falling.spin_round(self.spin, KICK_SPIN, rig);
        }
        falling.land_on(ground);
        // Aimed at its lip; or across, held off the wall flown to.
        match (self.lip(), self.across()) {
            (Some(lip), _) => {
                falling.aim_at(lip);
                falling.against(&[lip], rig);
            }
            (None, Some(other)) => falling.against(&[other], rig),
            (None, None) => {}
        }
        falling
    }
}

/// How far the hips may be from [`HIPS_MEETING`] out as the foot meets the
/// wall, metres: the walker's take-off is a running step's spread off its
/// best.
const MEETING_SLACK: f32 = 0.35;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::BoneSet;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// What a run up a wall measured, frame by frame.
    #[derive(Debug, Default)]
    struct RanUp {
        /// The most the foot strays from its hold on the wall, metres.
        hold_off: f32,
        /// The deepest any joint goes into the wall (under its lip), metres,
        /// which and when.
        into: f32,
        deepest: Option<(Bone, f32)>,
        /// The fastest any joint moves, m/s, which and when.
        fastest: f32,
        fastest_bone: Option<(Bone, f32)>,
        /// The leap's own fastest before the wall, m/s.
        leap_fastest: f32,
        /// The hips' greatest acceleration on the wall, m/s².
        on_wall: f32,
        /// Caught the lip, and then the wrists' height under it; or landed.
        caught: bool,
        landed: bool,
        /// Whether the plan said the hands reach the lip.
        reaches: bool,
    }

    fn ran_up(height: f32, speed: f32, leg: usize, off: f32) -> Option<RanUp> {
        let (stood, rig) = real_stood();
        let start = RunStart { leg, speed };
        let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 4.0, height, 1.0);
        let yaw = super::super::Hanging::square(&wall, rig.forward());
        let forward = Quat::from_rotation_y(yaw) * rig.forward();
        let origin = wall.a.with_y(0.0) - forward * (WallRun::takeoff(start, 0.0, &stood, &rig) + off);
        let origin = Vec3::new(0.0, 0.0, origin.z);
        let mut run = WallRun::plan(&wall, origin, yaw, start, 0.0, &stood, &rig)?;
        let mut m = RanUp { reaches: run.reaches_lip(), ..Default::default() };
        let world = |pose: &LocalPose, root: Vec3, yaw: f32| {
            let at = forward_kinematics_on(pose, &rig);
            BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
        };
        let mut last: Option<BoneSet<Vec3>> = None;
        let mut hips = Vec::new();
        let mut t = 0.0;
        let measure = |now: BoneSet<Vec3>, t: f32, m: &mut RanUp, last: &mut Option<BoneSet<Vec3>>| {
            for bone in Bone::ALL {
                let p = now[bone];
                let into = -wall.out_of(p);
                if into > m.into && p.y < wall.height() && into < 1.0 {
                    (m.into, m.deepest) = (into, Some((bone, t)));
                }
                if let Some(before) = last.as_ref() {
                    // About the hips: over the ground a running foot already
                    // goes twice the run's speed.
                    let speed = ((p - now[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT;
                    if speed > m.fastest {
                        (m.fastest, m.fastest_bone) = (speed, Some((bone, t)));
                    }
                }
            }
            *last = Some(now);
        };
        let mut leap_last: Option<BoneSet<Vec3>> = None;
        while !run.is_released() {
            run.advance(DT);
            t += DT;
            let now = world(&run.pose(), run.root(), run.facing());
            if run.elapsed() >= run.contact && !run.is_released() {
                m.hold_off = m.hold_off.max((now[LEGS[run.lead].2] - run.hold.0).length());
                hips.push(now[Bone::Hips]);
            }
            // The leap's own, before the wall.
            if run.elapsed() < run.contact {
                let leap = world(&run.jump.pose_at(run.elapsed(), &stood, &rig), run.root(), run.facing());
                if let Some(before) = leap_last.as_ref() {
                    for bone in Bone::ALL {
                        m.leap_fastest = m.leap_fastest.max(((leap[bone] - leap[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT);
                    }
                }
                leap_last = Some(leap);
            }
            measure(now, t, &mut m, &mut last);
        }
        let mut falling = run.release(&|_| Some(0.0));
        loop {
            if falling.airborne() && falling.catches(&[wall], &rig).is_some() {
                m.caught = true;
                break;
            }
            if falling.is_done() {
                m.landed = true;
                break;
            }
            falling.advance(DT);
            t += DT;
            let now = world(&falling.pose(&rig), falling.root(), falling.facing());
            measure(now, t, &mut m, &mut last);
        }
        m.on_wall = hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        Some(m)
    }

    /// Running at 3.5 and 4.5 m/s at walls 2.3-3.4 m high, off either
    /// foot, from its best take-off and a quarter metre either side: the
    /// foot held on the face, nothing into the wall, no joint whipping round
    /// the hips, the hips braked on the wall within 3 g; it catches the lip
    /// up to 2.6 m, as planned, and higher lands at the wall's foot.
    #[test]
    fn it_runs_up_a_wall_and_catches_its_lip_or_lands() {
        for height in [2.3, 2.5, 2.6, 2.7, 3.4] {
            for speed in [3.5, 4.5] {
                for (leg, off) in [0, 1].into_iter().flat_map(|leg| [-0.25, -0.1, 0.0, 0.1, 0.25].map(|off| (leg, off))) {
                    let name = format!("{height} m wall, {speed} m/s, off the {} {off:+} m from its best", if leg == 0 { "left" } else { "right" });
                    // Farther than a tenth of a metre off its best, refused
                    // if there is no room to brake on the wall, or the hold
                    // is out of the leg's reach.
                    let Some(m) = ran_up(height, speed, leg, off) else {
                        assert!(off.abs() > 0.15, "{name}: not run up");
                        continue;
                    };
                    assert!(m.hold_off < 1.0e-3, "{name}: the foot {:.4} m off its hold", m.hold_off);
                    // The fall's own keep-off leaves a toe 1.6 mm in.
                    assert!(m.into < 2.0e-3, "{name}: {:?} {:.4} m into the wall", m.deepest, m.into);
                    // 14 m/s, by eye (a sprinter's swing foot goes about 10
                    // about the hips; pops were 16-42), or the leap's own.
                    assert!(m.fastest < 14.0_f32.max(m.leap_fastest + 0.01), "{name}: {:?} at {:.2} m/s about the hips, the leap's own {:.2}", m.fastest_bone, m.fastest, m.leap_fastest);
                    assert!(m.on_wall < 3.0 * GRAVITY, "{name}: the hips braked at {:.1} m/s² on the wall", m.on_wall);
                    assert_eq!(m.caught, m.reaches, "{name}: caught {}, planned to reach {}", m.caught, m.reaches);
                    assert_eq!(m.caught, height <= 2.6, "{name}: caught {}", m.caught);
                    assert!(m.caught || m.landed, "{name}: neither caught nor landed");
                }
            }
        }
    }

    /// How deep `p` is inside a wall's block (its face's extent along, up
    /// to its top, in from its face as deep as its top): 0 outside.
    fn inside_wall(wall: &Ledge, p: Vec3) -> f32 {
        let (into, along) = (-wall.out_of(p), (p - wall.a).dot(wall.along()));
        if into > 0.0 && into < wall.depth && p.y < wall.height() && (0.0..=(wall.b - wall.a).length()).contains(&along) { into.min(wall.height() - p.y) } else { 0.0 }
    }

    /// What a kick measured.
    #[derive(Debug, Default)]
    struct Kicked {
        hold_off: f32,
        into: f32,
        deepest: Option<(Bone, f32)>,
        fastest: f32,
        fastest_bone: Option<(Bone, f32)>,
        leap_fastest: f32,
        on_wall: f32,
        caught: bool,
    }

    /// Running at `slant` off square at a tall wall, kicking off it toward
    /// a lip `height` high on a wall at right angles to it, off `leg` (or
    /// the one a kick takes off from), taken off at its best.
    fn kicked(height: f32, slant: f32, speed: f32, leg: Option<usize>) -> Option<Kicked> {
        kicked_from(height, slant, speed, leg, 0.0)
    }

    /// [`kicked`], taken off `off` metres farther along the run than its
    /// best.
    fn kicked_from(height: f32, slant: f32, speed: f32, leg: Option<usize>, off: f32) -> Option<Kicked> {
        let (stood, rig) = real_stood();
        let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 8.0, 4.0, 1.0);
        let target = Ledge::wall(Vec3::new(-2.2, 0.0, 0.5), Vec3::X, 3.0, height, 1.0);
        let yaw = super::super::Hanging::square(&wall, rig.forward()) + slant;
        let start = RunStart { leg: leg.unwrap_or_else(|| WallRun::kick_leg(&wall, yaw, &stood, &rig)), speed };
        let forward = Quat::from_rotation_y(yaw) * rig.forward();
        // Where the run meets the face: the hips meeting it 1.8 m from the
        // other wall's face (a kick from farther out flies too long to
        // rise to its lip within a kick's push up).
        let hit = Vec3::new(-2.2 + 1.8 - KICK_MEETING * slant.tan(), 0.0, -1.0);
        let origin = hit - forward * (WallRun::kick_takeoff(start, slant, &stood, &rig) + off);
        let mut run = WallRun::kick(&wall, &target, origin, yaw, start, 0.0, &stood, &rig)?;
        let mut m = Kicked::default();
        let world = |pose: &LocalPose, root: Vec3, yaw: f32| {
            let at = forward_kinematics_on(pose, &rig);
            BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
        };
        let (mut last, mut leap_last): (Option<BoneSet<Vec3>>, Option<BoneSet<Vec3>>) = (None, None);
        let mut hips = Vec::new();
        let mut t = 0.0;
        let measure = |now: BoneSet<Vec3>, t: f32, m: &mut Kicked, last: &mut Option<BoneSet<Vec3>>| {
            for bone in Bone::ALL {
                let into = inside_wall(&wall, now[bone]).max(inside_wall(&target, now[bone]));
                if into > m.into {
                    (m.into, m.deepest) = (into, Some((bone, t)));
                }
                if let Some(before) = last.as_ref() {
                    let speed = ((now[bone] - now[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT;
                    if speed > m.fastest {
                        (m.fastest, m.fastest_bone) = (speed, Some((bone, t)));
                    }
                }
            }
            *last = Some(now);
        };
        while !run.is_released() {
            run.advance(DT);
            t += DT;
            let now = world(&run.pose(), run.root(), run.facing());
            if run.elapsed() >= run.contact && !run.is_released() {
                m.hold_off = m.hold_off.max((now[LEGS[run.lead].2] - run.hold.0).length());
                hips.push(now[Bone::Hips]);
            }
            if run.elapsed() < run.contact {
                let leap = world(&run.jump.pose_at(run.elapsed(), &stood, &rig), run.root(), run.facing());
                if let Some(before) = leap_last.as_ref() {
                    for bone in Bone::ALL {
                        m.leap_fastest = m.leap_fastest.max(((leap[bone] - leap[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT);
                    }
                }
                leap_last = Some(leap);
            }
            measure(now, t, &mut m, &mut last);
        }
        let mut falling = run.release(&|_| Some(0.0));
        loop {
            if falling.airborne() && falling.catches(&[target], &rig).is_some() {
                m.caught = true;
                break;
            }
            if falling.is_done() {
                break;
            }
            falling.advance(DT);
            t += DT;
            let now = world(&falling.pose(&rig), falling.root(), falling.facing());
            measure(now, t, &mut m, &mut last);
        }
        m.on_wall = hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max);
        Some(m)
    }

    /// Running at 3.5 and 4.5 m/s at a tall wall 0.6-0.9 rad off square,
    /// taken off within [`KICK_TAKEOFF`] of its best, it kicks off it and
    /// catches a lip 2.3-2.7 m high on a wall at right angles (out of a
    /// standing jump's reach): the foot held on the wall, nothing into
    /// either wall, no joint whipping round, the hips braked on the wall
    /// within 3 g. Taken off 0.2 m farther, if it kicks at all, as well.
    #[test]
    fn it_kicks_off_a_wall_and_catches_a_lip_round_the_corner() {
        let mut faults = Vec::new();
        for height in [2.3, 2.5, 2.7] {
            for (slant, speed, off) in [0.6, 0.75, 0.9].into_iter().flat_map(|slant| [3.5, 4.5].into_iter().flat_map(move |speed| [-0.2, -0.1, 0.0, 0.1, 0.2].map(|off| (slant, speed, off)))) {
                let name = format!("a {height} m lip, {slant} rad off square, {speed} m/s, {off} m off the best");
                // Within the walker's window kicked; past it, maybe not.
                let Some(m) = kicked_from(height, slant, speed, None, off) else {
                    if (-KICK_TAKEOFF.0..=KICK_TAKEOFF.1).contains(&off) {
                        faults.push(format!("{name}: not kicked"));
                    }
                    continue;
                };
                eprintln!("{name}: {m:?}");
                if m.hold_off >= 1.0e-3 {
                    faults.push(format!("{name}: the foot strayed {:.4} m off its hold", m.hold_off));
                }
                if m.into >= 2.0e-3 {
                    faults.push(format!("{name}: {:?} went {:.4} m into a wall", m.deepest, m.into));
                }
                if m.fastest >= 14.0f32.max(m.leap_fastest + 0.01) {
                    faults.push(format!("{name}: {:?} went {:.1} m/s about the hips", m.fastest_bone, m.fastest));
                }
                if m.on_wall >= 3.0 * GRAVITY {
                    faults.push(format!("{name}: the hips braked at {:.1} m/s² on the wall", m.on_wall));
                }
                if !m.caught {
                    faults.push(format!("{name}: the lip was not caught"));
                }
            }
        }
        assert!(faults.is_empty(), "{} faults:\n{}", faults.len(), faults.join("\n"));
    }

    /// What a chain of kicks measured, and whether it caught its lip.
    #[derive(Debug, Default)]
    struct Chained {
        hold_off: f32,
        into: f32,
        deepest: Option<(Bone, f32)>,
        fastest: f32,
        fastest_bone: Option<(Bone, f32)>,
        on_wall: f32,
        kicks: usize,
        caught: bool,
    }

    /// Up a shaft `width` wide, its near wall's lip `height` high (out of
    /// one kick's reach), the far wall 4 m: running at the near wall
    /// `slant` off square, kicked across to the far wall, off that from the
    /// air back to the near wall's lip.
    fn chained(width: f32, height: f32, slant: f32, speed: f32) -> Option<Chained> {
        let (stood, rig) = real_stood();
        let near = Ledge::wall(Vec3::new(0.0, 0.0, -0.5 * width), Vec3::Z, 8.0, height, 1.0);
        let far = Ledge::wall(Vec3::new(0.0, 0.0, 0.5 * width), Vec3::NEG_Z, 8.0, 4.0, 1.0);
        let yaw = super::super::Hanging::square(&near, rig.forward()) + slant;
        let forward = Quat::from_rotation_y(yaw) * rig.forward();
        let start = RunStart { leg: WallRun::kick_leg(&near, yaw, &stood, &rig), speed };
        let hit = Vec3::new(0.0, 0.0, -0.5 * width);
        let origin = hit - forward * WallRun::kick_takeoff(start, slant, &stood, &rig);
        let mut run = WallRun::kick_across(&near, &far, origin, yaw, start, 0.0, &stood, &rig)?;
        let mut m = Chained { kicks: 1, ..Default::default() };
        let inside = |p: Vec3| inside_wall(&near, p).max(inside_wall(&far, p));
        let mut last: Option<BoneSet<Vec3>> = None;
        let mut t = 0.0;
        let mut measure = |now: BoneSet<Vec3>, t: f32, m: &mut Chained| {
            for bone in Bone::ALL {
                if inside(now[bone]) > m.into {
                    (m.into, m.deepest) = (inside(now[bone]), Some((bone, t)));
                }
                if let Some(before) = last.as_ref() {
                    let speed = ((now[bone] - now[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT;
                    if speed > m.fastest {
                        (m.fastest, m.fastest_bone) = (speed, Some((bone, t)));
                    }
                }
            }
            last = Some(now);
        };
        let world = |pose: &LocalPose, root: Vec3, yaw: f32| {
            let at = forward_kinematics_on(pose, &rig);
            BoneSet::from_fn(|bone| root + Quat::from_rotation_y(yaw) * at[bone])
        };
        for leg in 0..2 {
            let mut hips = Vec::new();
            while !run.is_released() {
                run.advance(DT);
                t += DT;
                let now = world(&run.pose(), run.root(), run.facing());
                if run.elapsed() >= run.contact && !run.is_released() {
                    m.hold_off = m.hold_off.max((now[LEGS[run.lead].2] - run.hold.0).length());
                    hips.push(now[Bone::Hips]);
                }
                measure(now, t, &mut m);
            }
            m.on_wall = m.on_wall.max(hips.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (DT * DT)).length()).fold(0.0, f32::max));
            let mut falling = run.release(&|_| Some(0.0));
            loop {
                if leg == 1 && falling.airborne() && falling.catches(&[near], &rig).is_some() {
                    m.caught = true;
                    return Some(m);
                }
                if falling.is_done() {
                    return Some(m);
                }
                // Across, met by the far wall: kicked off it back to the lip.
                if leg == 0
                    && falling.airborne()
                    && let Some(next) = WallRun::kick_from_air(&far, KickTarget::Lip(near), &falling, 0.0, &stood, &rig)
                {
                    run = next;
                    m.kicks += 1;
                    break;
                }
                falling.advance(DT);
                t += DT;
                let now = world(&falling.pose(&rig), falling.root(), falling.facing());
                measure(now, t, &mut m);
            }
        }
        Some(m)
    }

    /// Up shafts 1.8-2.2 m wide whose near lip is 3.0-3.4 m high (out of a
    /// run up's 2.6 m, and of one kick's), running at 3.5-4.5 m/s 0.9-1.0
    /// rad off square at the near wall (taken off within the shaft), it
    /// kicks across to the far wall and off that back to the lip, catching
    /// it: each foot held on its wall, nothing into either, no joint
    /// whipping round, braked within 3 g.
    #[test]
    fn it_kicks_wall_to_wall_up_a_shaft_and_catches_the_lip() {
        let mut faults = Vec::new();
        for width in [1.8, 2.0, 2.2] {
            for height in [3.0, 3.2, 3.4] {
                for (slant, speed) in [(0.9, 3.5), (0.95, 4.0), (1.0, 4.5)] {
                    let name = format!("a {width} m shaft, a {height} m lip, {slant} rad off square, {speed} m/s");
                    let Some(m) = chained(width, height, slant, speed) else {
                        faults.push(format!("{name}: not kicked across"));
                        continue;
                    };
                    eprintln!("{name}: {m:?}");
                    if m.kicks < 2 {
                        faults.push(format!("{name}: never kicked off the far wall"));
                        continue;
                    }
                    if m.hold_off >= 1.0e-3 {
                        faults.push(format!("{name}: a foot strayed {:.4} m off its hold", m.hold_off));
                    }
                    if m.into >= 2.0e-3 {
                        faults.push(format!("{name}: {:?} went {:.4} m into a wall", m.deepest, m.into));
                    }
                    if m.fastest >= 14.0 {
                        faults.push(format!("{name}: {:?} went {:.1} m/s about the hips", m.fastest_bone, m.fastest));
                    }
                    if m.on_wall >= 3.0 * GRAVITY {
                        faults.push(format!("{name}: the hips braked at {:.1} m/s² on a wall", m.on_wall));
                    }
                    if !m.caught {
                        faults.push(format!("{name}: the lip was not caught"));
                    }
                }
            }
        }
        assert!(faults.is_empty(), "{} faults:\n{}", faults.len(), faults.join("\n"));
    }

    /// A kick off the foot farther from the wall (the nearer leg crossing in
    /// front), at a wall met more than [`MOST_KICK_SLANT`] off square, or
    /// toward a lip too high for a kick's push is not taken.
    #[test]
    fn a_kick_off_the_wrong_foot_too_askew_or_toward_too_high_a_lip_is_not_taken() {
        let (stood, rig) = real_stood();
        let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 8.0, 4.0, 1.0);
        let yaw = super::super::Hanging::square(&wall, rig.forward()) + 0.75;
        let crossed = 1 - WallRun::kick_leg(&wall, yaw, &stood, &rig);
        assert!(kicked(2.5, 0.75, 4.0, Some(crossed)).is_none(), "off the foot farther from the wall");
        assert!(kicked(2.5, MOST_KICK_SLANT + 0.1, 4.0, None).is_none(), "met {} rad off square", MOST_KICK_SLANT + 0.1);
        assert!(kicked(3.2, 0.75, 4.0, None).is_none(), "toward a 3.2 m lip");
        assert!(kicked(2.5, 0.75, 4.0, None).is_some(), "but the same kick toward a 2.5 m lip is taken");
    }

    /// Running at a wall too far off square, or taking off so far from or
    /// near it that the foot would not meet it right, it does not run up.
    #[test]
    fn a_wall_met_askew_or_from_the_wrong_spot_is_not_run_up() {
        let (stood, rig) = real_stood();
        let start = RunStart { leg: 0, speed: 4.0 };
        let wall = Ledge::wall(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 4.0, 2.5, 1.0);
        let yaw = super::super::Hanging::square(&wall, rig.forward());
        let best = WallRun::takeoff(start, 0.0, &stood, &rig);
        let from = |off: f32| Vec3::new(0.0, 0.0, wall.a.z + best + off);
        assert!(WallRun::plan(&wall, from(0.0), yaw, start, 0.0, &stood, &rig).is_some(), "not run up from its best");
        assert!(WallRun::plan(&wall, from(0.0), yaw + 0.5, start, 0.0, &stood, &rig).is_none(), "run up 0.5 rad off square");
        assert!(WallRun::plan(&wall, from(1.0), yaw, start, 0.0, &stood, &rig).is_none(), "run up from 1 m too far");
        assert!(WallRun::plan(&wall, from(-0.6), yaw, start, 0.0, &stood, &rig).is_none(), "run up from 0.6 m too near");
    }
}
