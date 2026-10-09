//! Vaulting a low obstacle from a run: the second part of step 7 of the
//! parkour design (`docs/knowledge/character-animation/parkour/`).
//!
//! A speed vault is a running leap ([`Jump::from_run`]) over the obstacle,
//! landing on the other foot and running on. Its flight is the leap's, the
//! centre of mass (COM) on the same parabola, but the body is reshaped over
//! the obstacle:
//!
//! - **The hips roll** about the line of running, the legs tucked (thighs
//!   up ahead, knees bent) and swung out to the lead leg's side over the
//!   top; the trunk rolls back most of the way, leaning toward the other
//!   side.
//! - **The other hand plants on the top** as the hips cross its near face,
//!   for about a fifth of a second, the arm taking the lean.
//!
//! The reshaped body is moved as a whole to keep the COM where the leap
//! has it, so the flight stays ballistic. There are no speed-vault timings
//! (`parkour-movement-data`): the clearance is the hurdlers' (the COM
//! 0.23-0.39 m over a 1.07 m hurdle, Mansour et al. 2024), the hand's
//! contact a little longer than a gymnast's on the vaulting table
//! (0.12-0.22 s), the shapes set by eye.

use bevy::math::{Quat, Vec3};

use crate::character::anim::armik::{frame_turn, shoulder_lift, solve_arm_toward_from, turn_hand, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::{Jump, JumpAsk, JumpPhase, RunStart};
use crate::character::anim::rig::{accumulate_bind_rotations, accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::{knee_toward, place_ankle};
use crate::character::skeleton::Bone;

/// The COM's highest over the top, metres: hurdlers clear a hurdle with
/// it 0.23-0.39 m over (Mansour et al. 2024); the legs tucked under it
/// need the more. At 0.32 the take-off leg's toe went 3.8 cm into a 0.9 m
/// wall.
const COM_OVER: f32 = 0.38;
/// The COM rises at least this far from leaving, metres: a vault springs
/// up (about a 0.4 s flight). Over a 0.6 m rail, rising to just its top's
/// clearance left a 0.22 s flight, too short to roll the hips, plant the
/// hand and roll back.
const LEAST_RISE: f32 = 0.2;
/// Taken off away from its best, the COM rises up to this much more,
/// metres.
const MORE_RISE: f32 = 0.3;
/// A planned vault is let through if nothing goes deeper into the obstacle
/// than this, metres.
const CLEAR_TOLERANCE: f32 = 5.0e-4;
/// The take-off toe at least this far from the near face, metres (a
/// steeplechaser leaves 1.34 m from the barrier at 5 m/s, Slawinski et al.
/// 2019).
const LEAST_TAKEOFF: f32 = 1.0;
/// The landing foot's toe this far past the far face, metres (a
/// steeplechaser lands 1.17 m past the barrier, Slawinski et al. 2019).
const LAND_PAST: f32 = 0.9;
/// How far the hips roll the legs out to the side over the top, radians,
/// and how far the trunk stays rolled (the rest given back at the spine).
const ROLL: f32 = 1.15;
const TRUNK_ROLL: f32 = 0.55;
/// The furthest the trunk leans over onto the hand, radians: past the
/// hips' roll, the shoulders down near the hips' height (the rig's arms
/// are 17 % short; rolled with the hips, a shoulder was 0.53 m over a
/// 0.6 m rail, the arm 0.495 m).
const MOST_TRUNK: f32 = 1.45;
/// Each ankle tucked, in the hips' rolled frame from its socket: this far
/// ahead and down, metres (thighs up ahead, knees bent).
const TUCK: (f32, f32) = (0.3, 0.15);
/// A tucked ankle is raised (toward its socket's height) as far as puts it
/// this far over the top, metres: tucked higher for every obstacle, a
/// take-off leg's knee swung through at 14.9 m/s over a 0.75 m rail; as
/// low, the lead toe went 2.4 cm into a 0.9 m wall.
const ANKLE_OVER: f32 = 0.15;
const ANKLE_UNDER_SOCKET: f32 = -0.05;
/// A tucked knee points up and this much ahead for each unit up.
const KNEE_AHEAD: f32 = 0.4;
/// The hand is on the top while its shoulder passes this far either side
/// of it along the way, metres: 0.12-0.17 s at 4-3 m/s (gymnasts 0.12-0.22 s
/// on the vaulting table), at most [`CONTACT`]. Held 0.2 s from the moment
/// the COM crossed the near face, at 3 m/s the shoulder went 0.3 m past it
/// and out of the arm's reach. Reaching for it and lifting off it, seconds.
const SWEEP: f32 = 0.25;
const CONTACT: f32 = 0.2;
/// The hand on the top, the body moved to the leap's COM and the arm solved
/// onto the plant this many times over (each moves the other a little).
const HAND_PASSES: usize = 4;
/// The shortest hand contact a vault plants for, seconds.
const LEAST_CONTACT: f32 = 0.08;
const REACH: f32 = 0.12;
/// Lifting off, seconds: planted, the hand goes back about the COM at the
/// run's speed, and let go in 0.12 s from 4 m/s it swung on at 11 m/s.
const LIFT: f32 = 0.2;
/// The wrist planted this far over the top, metres: the palm's thickness
/// under it.
const WRIST_OVER: f32 = 0.03;
/// The hand plants this far in from the near face, metres, and from the
/// far face at most.
const PLANT_IN: f32 = 0.1;
/// The hips roll over this long from take-off, seconds (at most
/// [`ROLL_SHARE`] of the flight), and back over this long before
/// touchdown, from the hand's lifting off at the earliest. Rolled in 0.1 s,
/// the legs carried round swung a toe at 16.8 m/s about the COM; rolled
/// back over 0.2 s, a 0.4 s flight left no time for the hand.
const ROLL_IN: f32 = 0.2;
const ROLL_SHARE: f32 = 0.45;
const ROLL_OUT: f32 = 0.12;
/// The lead (free) leg starts tucking this long before take-off, seconds;
/// each tucks over this long (at most half the time left), and is let down
/// over this long before touchdown.
const TUCK_EARLY: f32 = 0.1;
const TUCK_IN: f32 = 0.15;
/// The take-off leg, coming from behind, tucks over this long, seconds,
/// and is let down to the run's swing over as long: in 0.15 s its knee
/// swung through at 15.8 m/s about the COM, the leap's own fastest 9.2.
const TRAIL_TUCK_IN: f32 = 0.3;
const TUCK_OUT: f32 = 0.2;
/// The take-off leg stays tucked until the leap's own swing has carried
/// its ankle this far past the far face, metres (the toes reach 0.16 ahead
/// of it).
const TRAIL_PAST: f32 = 0.35;
/// A speed vault's reach: obstacles this low to this high over the floor,
/// metres (lower, stepped or hopped over; higher than the hips, a mantle),
/// at most this deep, from a run at least this fast, m/s. Over a 0.6 m
/// rail the least rise put the hips 0.6 m over it, the hand out of reach
/// even leant all the way over.
pub const LOWEST: f32 = 0.75;
pub const HIGHEST_OVER_HIPS: f32 = 0.1;
pub const DEEPEST: f32 = 0.7;
pub const SLOWEST: f32 = 2.5;
/// The most a run meets an obstacle's face off square, radians: a speed
/// vault's, and a lazy vault's (taken from an angle, up to 34°: at 40° a
/// 0.75 m rail could not be vaulted from 3 m/s).
pub const MOST_SLANT: f32 = 0.5;
pub const LAZY_MOST_SLANT: f32 = 0.6;
/// A lazy vault rolls the hips this far, radians (a speed vault's
/// [`ROLL`]), the body more upright; its lead leg tucks to this far ahead
/// of and under its socket, metres, nearly straight; and the take-off leg
/// starts tucking this long after take-off, seconds.
const LAZY_ROLL: f32 = 0.6;
const LAZY_LEAD_TUCK: (f32, f32) = (0.5, 0.0);
const LAZY_TRAIL_LATE: f32 = 0.1;
/// A vault is taken from the foot that comes down this far either side of
/// its best take-off (`Jump::vault_takeoff`) at most, metres; nearer still,
/// too late.
pub const TAKEOFF_SLACK: f32 = 0.35;

/// The hand on the top no farther from its lifted shoulder than this share
/// of the arm.
const MOST_REACH: f32 = 0.97;

const ARMS: [ArmChain; 2] = [ArmChain::LEFT, ArmChain::RIGHT];
const CLAVICLES: [Bone; 2] = [Bone::LeftShoulder, Bone::RightShoulder];
const KNEES: [Bone; 2] = [Bone::LeftLeg, Bone::RightLeg];
const LEGS: [(Bone, Bone); 2] = [(Bone::LeftUpLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightFoot)];
/// Which way each side lies along the character's left.
const SIGN: [f32; 2] = [1.0, -1.0];

/// The obstacle, in a jump's frame (the pose's frame where the jump began,
/// the floor at `y = 0`): the line of running meets its near face this far
/// along the rig's forward and crosses it in this far (along the run), its
/// top this high; its face met `slant` off square (radians about `+Y`, the
/// way into it turned from the way of running).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obstacle {
    pub near: f32,
    pub depth: f32,
    pub top: f32,
    pub slant: f32,
}

/// A hop's reach (step 11 of the steps beyond the first ten, running
/// agility): obstacles at most this high and deep, metres, hopped in a
/// run's stride. Its COM rises at least this far, metres, and at most this
/// much more to clear; its landing toe this far past the far face, metres.
/// Over a 0.55 m rail the COM rose 0.25 m and the run went on 1.3 m/s
/// slower (or from 3 m/s at its best take-off, no hop cleared it).
pub const HOP_HIGHEST: f32 = 0.45;
pub const HOP_DEEPEST: f32 = 0.6;
const HOP_LEAST_RISE: f32 = 0.05;
const HOP_MORE_RISE: f32 = 0.5;
const HOP_PAST: f32 = 0.9;
/// A hop's take-off toe at least this far from the near face, metres: at
/// 0.5 and 0.7, from 3 m/s no hop cleared a 0.45 m rail. Its landing toe
/// [`HOP_PAST`] past the far face: at 0.65, the lead foot came over the far
/// edge a few frames before touchdown and its lift had to drop in them.
const HOP_LEAST_TAKEOFF: f32 = 1.0;
/// Hopping, each ankle kept this far over the top and each toe this far,
/// metres, while either is within this far of the obstacle along the way;
/// the lift eased in and out over this long, seconds (a running leap's own
/// legs trailed into anything over a kerb, even rising 0.55 m; lifted by
/// how near the foot was, a knee rose 0.3 m in a tenth of a second and its
/// step changed 6 cm in a frame).
const HOP_ANKLE_OVER: f32 = 0.14;
const HOP_TOE_OVER: f32 = 0.05;
const HOP_MARGIN: f32 = 0.1;
const HOP_EASE: f32 = 0.12;
/// The body comes down about this share of a foot's lift, the legs drawn up
/// carrying the centre of mass up (they are about a third of the mass, and
/// a leg's centre rises about half its foot's lift): each foot lifted so
/// much more.
const LIFT_SINKS: f32 = 0.16;
/// A foot is down while its ankle is no higher than this over standing's,
/// metres, and moves no faster than this, m/s.
const HOP_DOWN: f32 = 0.05;
const HOP_STILL: f32 = 0.3;

/// Which vault: a speed vault (the legs together round one side), a lazy
/// vault (from an angle, the lead leg over first, nearly straight, the
/// other after; the body more upright), or a hop (a small obstacle passed
/// in the run's stride: its leap, lengthened and raised as little as clears
/// it, its flight as it is).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VaultKind {
    #[default]
    Speed,
    Lazy,
    Hop,
}

impl VaultKind {
    /// The most its run may meet an obstacle's face off square, radians.
    pub fn most_slant(self) -> f32 {
        match self {
            Self::Speed | Self::Hop => MOST_SLANT,
            Self::Lazy => LAZY_MOST_SLANT,
        }
    }

    /// The foot it takes off from at an obstacle met `slant` off square, if
    /// it matters: a lazy vault's legs go over toward the far end, so the
    /// lead leg (the other) must be on that side.
    pub fn takeoff_leg(self, slant: f32) -> Option<usize> {
        match self {
            Self::Lazy if slant > 0.0 => Some(0),
            Self::Lazy if slant < 0.0 => Some(1),
            _ => None,
        }
    }

    /// How far its hips roll over the top, radians.
    fn roll(self) -> f32 {
        match self {
            Self::Speed => ROLL,
            Self::Lazy => LAZY_ROLL,
            Self::Hop => 0.0,
        }
    }
}

impl Obstacle {
    /// A face met square: `near` ahead, `depth` deep, `top` high.
    pub const fn square(near: f32, depth: f32, top: f32) -> Self {
        Self { near, depth, top, slant: 0.0 }
    }

    /// `ledge` (its face and top, `depth` deep) as an obstacle ahead of a
    /// run from `origin` along `forward` (the world): its near face where
    /// the line of running meets it, its depth along that line, its top
    /// over `origin`'s floor, how far off square. `None` if the run meets
    /// the face more than `most_slant` off square, beyond its ends, or not
    /// ahead.
    pub fn ahead(ledge: &super::Ledge, origin: Vec3, forward: Vec3, most_slant: f32) -> Option<Self> {
        let forward = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
        let square = -forward.dot(ledge.out);
        if square < most_slant.cos() {
            return None;
        }
        let near = (ledge.a - origin).dot(ledge.out) / -square;
        let at = origin + forward * near;
        let along = (at - ledge.a).dot(ledge.along());
        if near <= 0.0 || !(0.0..=(ledge.b - ledge.a).length()).contains(&along) {
            return None;
        }
        let into = -ledge.out;
        let slant = forward.cross(into).y.atan2(forward.dot(into));
        Some(Self { near, depth: ledge.depth / square, top: ledge.height() - origin.y, slant })
    }

    /// How deep a point (the jump's frame, along the rig's `forward`) is
    /// inside it: the least way out, 0 outside.
    pub fn inside(&self, p: Vec3, forward: Vec3) -> f32 {
        let into_way = Quat::from_rotation_y(self.slant) * forward;
        let into = (p - forward * self.near).dot(into_way);
        let thick = self.depth * self.slant.cos();
        let (out_far, down) = (thick - into, self.top - p.y);
        if into > 0.0 && out_far > 0.0 && down > 0.0 && p.y > 0.0 { into.min(out_far).min(down) } else { 0.0 }
    }
}

/// A vault's reshaping of its leap's flight.
#[derive(Debug, Clone, Copy)]
pub struct Vaulting {
    obstacle: Obstacle,
    /// The way the legs swing out (the lead leg's side), along the rig's
    /// left; the hand that plants (the other side's).
    side: f32,
    hand: usize,
    /// The flight, seconds into the jump: from take-off to touchdown; and
    /// when the leap's own swing has carried the take-off leg past the
    /// obstacle ([`TRAIL_PAST`]).
    flight: (f32, f32),
    trail_until: f32,
    /// When the hand is on the top, and where (the jump's frame).
    contact: (f32, f32),
    plant: Vec3,
    /// The hand's turn flat on the top, from its rest (as `turn_hand` takes
    /// it), and its bind.
    press: Quat,
    bind: Quat,
    /// How far the trunk stays rolled toward the hand, radians (at least
    /// [`TRUNK_ROLL`]).
    trunk: f32,
    /// Each foot's world rotation standing, the pose's frame.
    feet: [Quat; 2],
    kind: VaultKind,
}

impl Jump {
    /// A vault (`kind`) over `obstacle` (the jump's frame) from a run at
    /// `start.speed`, the foot of `start.leg` just down: a running leap
    /// whose COM tops out [`COM_OVER`] above the top, landing past it and
    /// running on, its flight reshaped ([`Vaulting`]). `None` if the
    /// obstacle is out of a vault's reach ([`LOWEST`],
    /// [`HIGHEST_OVER_HIPS`], [`DEEPEST`], [`VaultKind::most_slant`]), the
    /// run is slower than [`SLOWEST`], or nothing planned clears it.
    pub fn vault(obstacle: Obstacle, start: RunStart, kind: VaultKind, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        if kind == VaultKind::Hop {
            return Self::hop(obstacle, start, stood, rig);
        }
        let hips = forward_kinematics_on(stood, rig)[Bone::Hips].y;
        if obstacle.top < LOWEST || obstacle.top > hips + HIGHEST_OVER_HIPS || obstacle.depth > DEEPEST || start.speed < SLOWEST || obstacle.slant.abs() > kind.most_slant() {
            return None;
        }
        let forward = rig.forward();
        let toe = |jump: &Jump| {
            let at = forward_kinematics_on(&jump.pose_at_unshaped(0.0, stood, rig), rig);
            at[crate::character::anim::foot::foot_bones(LEGS[start.leg].1).1].dot(forward)
        };
        // The COM's rise for its top over the obstacle, from where it leaves:
        // planned once to find that, then again.
        let first = Jump::from_run(JumpAsk::running(0.3, 0.0), start, stood, rig);
        let distance = obstacle.near - toe(&first) + obstacle.depth + LAND_PAST;
        // Taken off nearer or farther than its best, a little higher, for the
        // longer flight: at the clearance's rise alone, only 0.3-1.0 m of
        // take-offs (less than a running step) could be vaulted.
        (0..=(MORE_RISE / 0.05).round() as usize).find_map(|k| {
            let mut jump = first.clone();
            for _ in 0..2 {
                let leaves = planned_com(&jump, jump.ends(JumpPhase::Push), stood, rig).y;
                let height = (obstacle.top + COM_OVER - leaves).max(LEAST_RISE) + 0.05 * k as f32;
                jump = Jump::from_run(JumpAsk::running(height, distance), start, stood, rig);
            }
            let vaulting = Vaulting::plan(&jump, obstacle, start, kind, stood, rig)?;
            jump.set_vault(vaulting);
            jump.clears(stood, rig).then_some(jump)
        })
    }

    /// Whether, posed through its flight and on until the take-off leg is
    /// past the obstacle, nothing of the body goes into it: taken off
    /// nearer than its best, a lead toe went 3.4 cm into the near face.
    fn clears(&self, stood: &LocalPose, rig: &RigGeometry) -> bool {
        let Some(vaulting) = self.vaulting() else { return true };
        let forward = rig.forward();
        let (from, to) = (vaulting.flight.0 - TUCK_EARLY, vaulting.trail_until.max(vaulting.flight.1) + 0.1);
        (0..=((to - from) * 120.0).ceil() as usize).all(|k| {
            let t = from + k as f32 / 120.0;
            let at = forward_kinematics_on(&self.pose_at(t, stood, rig), rig);
            let travelled = forward * self.travelled_at(t);
            Bone::ALL.iter().all(|&bone| vaulting.obstacle.inside(at[bone] + travelled, forward) < CLEAR_TOLERANCE)
        })
    }

    /// A hop over a small `obstacle` (the jump's frame) in a run's stride at
    /// `start.speed`, the foot of `start.leg` just down: a running leap,
    /// its flight as it is, landing [`HOP_PAST`] past it and running on, its
    /// COM rising as little as clears it. `None` if the obstacle is out of a
    /// hop's reach ([`HOP_HIGHEST`], [`HOP_DEEPEST`]), the run is slower
    /// than [`SLOWEST`], or nothing planned clears it.
    pub fn hop(obstacle: Obstacle, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        if obstacle.top > HOP_HIGHEST || obstacle.depth > HOP_DEEPEST || start.speed < SLOWEST || obstacle.slant.abs() > MOST_SLANT {
            return None;
        }
        let forward = rig.forward();
        let first = Jump::from_run(JumpAsk::running(HOP_LEAST_RISE, 0.0), start, stood, rig);
        let toe = forward_kinematics_on(&first.pose_at_unshaped(0.0, stood, rig), rig)[crate::character::anim::foot::foot_bones(LEGS[start.leg].1).1].dot(forward);
        let distance = obstacle.near - toe + obstacle.depth + HOP_PAST;
        (0..=(HOP_MORE_RISE / 0.05).round() as usize).find_map(|k| {
            let mut jump = Jump::from_run(JumpAsk::running(HOP_LEAST_RISE + 0.05 * k as f32, distance), start, stood, rig);
            jump.set_hop(HopLift::plan(&jump, obstacle, stood, rig)?);
            jump.clears_plainly(obstacle, stood, rig).then_some(jump)
        })
    }

    /// A standing jump as `ask`ed over a low `obstacle` in its way (the
    /// jump's frame), its knees drawn up over it (step 12, a tuck): the
    /// feet lifted over it as a hop's, the centre of mass's path unchanged.
    /// `None` if a foot would be over it on the floor, or it is not cleared.
    pub fn tuck_over(ask: JumpAsk, obstacle: Obstacle, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let mut jump = Jump::plan(ask, stood, rig);
        jump.set_hop(HopLift::plan(&jump, obstacle, stood, rig)?);
        jump.clears_plainly(obstacle, stood, rig).then_some(jump)
    }

    /// Whether, posed from take-off until it runs on, nothing of the body
    /// goes into `obstacle`.
    fn clears_plainly(&self, obstacle: Obstacle, stood: &LocalPose, rig: &RigGeometry) -> bool {
        let forward = rig.forward();
        let end = self.duration();
        (0..=(end * 120.0).ceil() as usize).all(|k| {
            let t = k as f32 / 120.0;
            let at = forward_kinematics_on(&self.pose_at(t, stood, rig), rig);
            let travelled = forward * self.travelled_at(t);
            Bone::ALL.iter().all(|&bone| obstacle.inside(at[bone] + travelled, forward) < CLEAR_TOLERANCE)
        })
    }

    /// How far ahead of where a run's foot comes down a small obstacle's
    /// near face is best for a hop from it: the COM's top over its middle,
    /// the take-off toe [`HOP_LEAST_TAKEOFF`] from its face at least.
    pub fn hop_takeoff(depth: f32, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        let jump = Jump::from_run(JumpAsk::running(HOP_LEAST_RISE + 0.1, 0.0), start, stood, rig);
        let push = jump.ends(JumpPhase::Push);
        let flight = jump.ends(JumpPhase::Flight);
        let apex = (push..=flight).step_by_samples(80).max_by(|a, b| jump.com_height_at(*a).total_cmp(&jump.com_height_at(*b))).unwrap_or(push);
        let toe = forward_kinematics_on(&jump.pose_at_unshaped(0.0, stood, rig), rig)[crate::character::anim::foot::foot_bones(LEGS[start.leg].1).1].dot(rig.forward());
        (planned_com(&jump, apex, stood, rig).dot(rig.forward()) - 0.5 * depth).max(toe + HOP_LEAST_TAKEOFF)
    }

    /// How far ahead of where a run's foot comes down (the root there, the
    /// jump's frame) an obstacle's near face is best for a vault from it:
    /// the COM's top over the obstacle's middle.
    pub fn vault_takeoff(depth: f32, top: f32, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        let mut jump = Jump::from_run(JumpAsk::running(0.3, 0.0), start, stood, rig);
        let leaves = planned_com(&jump, jump.ends(JumpPhase::Push), stood, rig).y;
        jump = Jump::from_run(JumpAsk::running((top + COM_OVER - leaves).max(LEAST_RISE), 0.0), start, stood, rig);
        let push = jump.ends(JumpPhase::Push);
        let flight = jump.ends(JumpPhase::Flight);
        let apex = (push..=flight).step_by_samples(80).max_by(|a, b| jump.com_height_at(*a).total_cmp(&jump.com_height_at(*b))).unwrap_or(push);
        // And no nearer the take-off toe than `LEAST_TAKEOFF`: the COM's top
        // over the middle had it leave 0.8 m short, the lead toe driving up
        // into the near face (4 cm).
        let toe = forward_kinematics_on(&jump.pose_at_unshaped(0.0, stood, rig), rig)[crate::character::anim::foot::foot_bones(LEGS[start.leg].1).1].dot(rig.forward());
        (planned_com(&jump, apex, stood, rig).dot(rig.forward()) - 0.5 * depth).max(toe + LEAST_TAKEOFF)
    }
}

/// How a hop lifts each foot over its obstacle: for each leg (left,
/// right), the lift's start, from when its foot comes within
/// [`HOP_MARGIN`] of the obstacle to when it has passed, its end, and how
/// high, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HopLift {
    legs: [Option<HopLeg>; 2],
}

/// One leg's lift ([`HopLift`]), seconds into the jump.
#[derive(Debug, Clone, Copy, PartialEq)]
struct HopLeg {
    rise: f32,
    from: f32,
    to: f32,
    fall: f32,
    height: f32,
}

/// One foot at one moment of a hop's bare leap ([`HopLift::plan`]).
#[derive(Debug, Clone, Copy)]
struct HopFoot {
    /// Within [`HOP_MARGIN`] of the obstacle along the way; down (low and
    /// still); how far it must rise to clear the top, metres.
    over: bool,
    down: bool,
    need: f32,
}

impl HopLift {
    /// The lift `jump`'s bare leap needs to carry each foot over
    /// `obstacle`, eased in and out over [`HOP_EASE`] while the foot is off
    /// the floor (a foot down is low, [`HOP_DOWN`], and still,
    /// [`HOP_STILL`]): `None` if a foot is over it while down.
    fn plan(jump: &Jump, obstacle: Obstacle, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let forward = rig.forward();
        let standing = forward_kinematics_on(stood, rig);
        let end = jump.duration();
        const RATE: f32 = 240.0;
        let ankles: Vec<[(Vec3, Vec3); 2]> = (0..=(end * RATE).ceil() as usize)
            .map(|k| {
                let t = k as f32 / RATE;
                let at = forward_kinematics_on(&jump.pose_at_unshaped(t, stood, rig), rig);
                let travelled = forward * jump.travelled_at(t);
                LEGS.map(|(_, ankle)| (at[ankle] + travelled, at[crate::character::anim::foot::foot_bones(ankle).1] + travelled))
            })
            .collect();
        // Still as well as low: the leap's landing foot skims the floor
        // before it lands.
        let samples: Vec<[HopFoot; 2]> = (0..ankles.len())
            .map(|k| {
                [0, 1].map(|side| {
                    let (p, tip) = ankles[k][side];
                    let moving = (ankles[(k + 1).min(ankles.len() - 1)][side].0 - ankles[k.saturating_sub(1)][side].0).length() * RATE / 2.0;
                    let over = |q: Vec3| (obstacle.near - HOP_MARGIN..=obstacle.near + obstacle.depth + HOP_MARGIN).contains(&q.dot(forward));
                    HopFoot {
                        over: over(p) || over(tip),
                        down: p.y < standing[LEGS[side].1].y + HOP_DOWN && moving < HOP_STILL,
                        need: (obstacle.top + HOP_ANKLE_OVER - p.y).max(obstacle.top + HOP_TOE_OVER - tip.y),
                    }
                })
            })
            .collect();
        let time = |i: usize| i as f32 / RATE;
        let mut legs = [None; 2];
        for (side, leg) in legs.iter_mut().enumerate() {
            let over: Vec<usize> = (0..samples.len()).filter(|&i| samples[i][side].over).collect();
            let (Some(&first), Some(&last)) = (over.first(), over.last()) else { continue };
            if over.iter().any(|&i| samples[i][side].down) {
                return None;
            }
            let height = over.iter().map(|&i| samples[i][side].need).fold(0.0, f32::max);
            if height <= 0.0 {
                continue;
            }
            // Eased in no earlier than it left the floor, out no later than
            // it meets it.
            let left = (0..first).rev().find(|&i| samples[i][side].down).map_or(0.0, time);
            let meets = (last..samples.len()).find(|&i| samples[i][side].down).map_or(end, time);
            let (from, to) = (time(first), time(last));
            *leg = Some(HopLeg { rise: (from - HOP_EASE).max(left), from, to, fall: (to + HOP_EASE).min(meets), height });
        }
        Some(Self { legs })
    }

    /// How far leg `side`'s foot is lifted `t` seconds in, metres.
    fn at(&self, side: usize, t: f32) -> f32 {
        let Some(leg) = self.legs[side] else { return 0.0 };
        let up = smoothstep(((t - leg.rise) / (leg.from - leg.rise).max(1.0e-3)).clamp(0.0, 1.0));
        let down = smoothstep(((t - leg.to) / (leg.fall - leg.to).max(1.0e-3)).clamp(0.0, 1.0));
        leg.height * up * (1.0 - down)
    }
}

/// `pose` (a hop's, `t` seconds into its jump) with each foot lifted over
/// its obstacle as `lift` has it. The knee bends to it, up ahead, as a
/// hurdler's.
pub fn lift_over(pose: &LocalPose, lift: &HopLift, t: f32, rig: &RigGeometry) -> LocalPose {
    let mut lifted = *pose;
    let at = forward_kinematics_on(pose, rig);
    let mut moved = false;
    for (side, (_, ankle)) in LEGS.into_iter().enumerate() {
        let up = lift.at(side, t);
        if up > 1.0e-5 {
            // Each lift by as much more as the body comes down for it below.
            place_ankle(&mut lifted, rig, ankle, at[ankle] + Vec3::Y * up / (1.0 - LIFT_SINKS) - at[Bone::Hips]);
            moved = true;
        }
    }
    // The centre of mass where the flight has it: the body moved down by
    // as much as the legs drawn up carried it up (the tuck's own path
    // unchanged, step 12).
    if moved {
        lifted.root_translation -= com(&lifted, rig) - com(pose, rig);
    }
    lifted
}

/// The COM a jump plans `t` seconds in, in the jump's frame (the pose's,
/// the floor at `y = 0`). The plan's is in the standing hips' frame.
fn planned_com(jump: &Jump, t: f32, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
    forward_kinematics_on(stood, rig)[Bone::Hips] + rig.forward() * jump.com_ahead_at(t) + Vec3::Y * jump.com_height_at(t)
}

/// Eased 0-1 over `span` seconds (or metres) from `from`.
fn ease(t: f32, from: f32, span: f32) -> f32 {
    smoothstep(((t - from) / span.max(1.0e-6)).clamp(0.0, 1.0))
}

/// A pose's whole-body COM, in its frame.
fn com(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
    pose.root_translation + crate::character::anim::anthropometry::centre_of_mass(pose, rig)
}

/// Evenly spaced samples of a range, both ends included.
trait Samples {
    fn step_by_samples(self, n: usize) -> impl Iterator<Item = f32>;
}

impl Samples for std::ops::RangeInclusive<f32> {
    fn step_by_samples(self, n: usize) -> impl Iterator<Item = f32> {
        let (a, b) = (*self.start(), *self.end());
        (0..=n).map(move |k| a + (b - a) * k as f32 / n as f32)
    }
}

impl Vaulting {
    fn plan(jump: &Jump, obstacle: Obstacle, start: RunStart, kind: VaultKind, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let forward = rig.forward();
        let flight = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
        // When the COM crosses each face.
        let crossing = |plane: f32| {
            let (mut t, end) = flight;
            while t < end && planned_com(jump, t, stood, rig).dot(forward) < plane {
                t += 1.0 / 480.0;
            }
            t
        };
        // The hand on the top centred on the COM over its middle.
        let middle = crossing(obstacle.near + 0.5 * obstacle.depth);
        let contact = (2.0 * SWEEP / jump.speed().max(0.1)).min(CONTACT);
        // While the hips are rolled all the way: from take-off over a 0.6 m
        // rail, the shoulder had not come down within the arm's reach.
        let rolled = (flight.0 + ROLL_IN.min(ROLL_SHARE * (flight.1 - flight.0)), flight.1 - ROLL_OUT);
        let contact = ((middle - 0.5 * contact).max(rolled.0), (middle + 0.5 * contact).min(rolled.1));
        if contact.1 - contact.0 < LEAST_CONTACT {
            return None;
        }
        let lead = 1 - start.leg;
        let hand = start.leg;
        // A lazy vault's legs go over toward the obstacle's far end: met at a
        // slant whose near end is on the lead leg's side, it is not taken off
        // this foot (`Obstacle::inside`: a turn toward the left brings the
        // face nearer on the left). Off that foot, 0.4 rad off square, the
        // lead leg met the face first.
        if kind == VaultKind::Lazy && SIGN[lead] * obstacle.slant > 0.0 {
            return None;
        }
        let past = obstacle.near + obstacle.depth + TRAIL_PAST;
        let ankle = LEGS[hand].1;
        let mut trail_until = flight.1;
        let last = jump.ends(JumpPhase::Recover);
        while trail_until < last && forward_kinematics_on(&jump.pose_at_unshaped(trail_until, stood, rig), rig)[ankle].dot(forward) + jump.travelled_at(trail_until) < past {
            trail_until += 1.0 / 240.0;
        }
        let mut vaulting = Self {
            obstacle,
            side: SIGN[lead],
            hand,
            flight,
            trail_until,
            contact,
            plant: Vec3::ZERO,
            press: Quat::IDENTITY,
            bind: accumulate_bind_rotations(rig)[ARMS[hand].wrist],
            trunk: TRUNK_ROLL.min(kind.roll()),
            feet: LEGS.map(|(_, ankle)| accumulate_world_rotations(stood, rig)[ankle]),
            kind,
        };
        // Palm flat on the top, the fingers ahead and a little out.
        let rest = forward_kinematics_on(&LocalPose::REST, rig);
        let chain = ARMS[hand];
        let along = (rest[chain.wrist] - rest[chain.elbow]).normalize_or(Vec3::NEG_Y);
        let palm = (Vec3::NEG_Y - along * along.dot(Vec3::NEG_Y)).normalize_or(Vec3::NEG_Z);
        let out = rig.left() * SIGN[hand];
        vaulting.press = frame_turn(along, palm, (forward + out * 0.4).normalize(), Vec3::NEG_Y);
        let arm = (rest[chain.elbow] - rest[chain.shoulder]).length() + (rest[chain.wrist] - rest[chain.elbow]).length();
        let clavicle = CLAVICLES[hand];
        // The trunk leant no further toward the hand than lets it reach the
        // top: at the rolled trunk's 0.55 rad, a 0.6 m rail was 9 cm out of
        // the arm's reach.
        let mut trunk = TRUNK_ROLL.min(kind.roll());
        while trunk <= MOST_TRUNK + 1.0e-4 {
            vaulting.trunk = trunk;
            // The hand plants under its shoulder midway through the contact,
            // the body reshaped but not reaching: on the top, in from its
            // faces.
            let middle = 0.5 * (contact.0 + contact.1);
            let at = forward_kinematics_on(&vaulting.body_at(jump, middle, stood, rig), rig);
            let shoulder = at[chain.shoulder] + forward * jump.travelled_at(middle);
            let ahead = shoulder.dot(forward).clamp(obstacle.near + PLANT_IN, obstacle.near + (obstacle.depth - PLANT_IN).max(PLANT_IN));
            let across = shoulder - forward * shoulder.dot(forward) - Vec3::Y * shoulder.y;
            vaulting.plant = across + forward * ahead + Vec3::Y * (obstacle.top + WRIST_OVER);
            // Within the arm's reach from its lifted shoulder all through
            // the contact.
            let reaches = (0..=10).all(|k| {
                let t = contact.0 + (contact.1 - contact.0) * k as f32 / 10.0;
                let at = forward_kinematics_on(&vaulting.body_at(jump, t, stood, rig), rig);
                let plant = vaulting.plant - forward * jump.travelled_at(t);
                let lift = shoulder_lift(at[clavicle], at[chain.shoulder], plant, 0.85 * arm);
                let lifted = at[clavicle] + lift * (at[chain.shoulder] - at[clavicle]);
                (plant - lifted).length() <= MOST_REACH * arm
            });
            if reaches {
                return Some(vaulting);
            }
            trunk += 0.05;
        }
        None
    }

    /// The body reshaped `t` seconds into `jump`, its COM the leap's, the
    /// hand not yet on the top.
    fn body_at(&self, jump: &Jump, t: f32, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let leap = jump.pose_at_unshaped(t, stood, rig);
        let mut pose = leap;
        self.reshape_body(&mut pose, &leap, t, rig);
        pose.root_translation += com(&leap, rig) - com(&pose, rig);
        pose
    }

    /// How far the hips are rolled `t` seconds into the jump, 0-1: from
    /// take-off, and back by touchdown once the hand is off. Rolled at once
    /// as it left a 0.6 m rail (a 0.22 s flight), a toe moved 1 m in a frame.
    fn rolling(&self, t: f32) -> f32 {
        let (from, to) = self.flight;
        if t <= from || t >= to {
            return 0.0;
        }
        let back = (to - ROLL_OUT).max(self.contact.1).min(to - 1.0e-3);
        ease(t, from, ROLL_IN.min(ROLL_SHARE * (to - from))).min(1.0 - ease(t, back, to - back))
    }

    /// How far leg `leg` is tucked `t` seconds into the jump, 0-1: the lead
    /// (free) leg from late in the push, let down to land by touchdown; the
    /// take-off leg from take-off, until the leap's own swing has brought it
    /// past the far face. Let down by touchdown, behind the obstacle in the
    /// leap's landing stance, it swung back down through it (12 cm).
    fn tucking(&self, t: f32, leg: usize) -> f32 {
        let (tucking_in, letting_down) = self.tuck_parts(t, leg);
        tucking_in.min(letting_down)
    }

    /// How far leg `leg` has tucked in, and how far it is not yet let down,
    /// `t` seconds into the jump, each 0-1 ([`Self::tucking`] is the less).
    fn tuck_parts(&self, t: f32, leg: usize) -> (f32, f32) {
        let (from, to) = self.flight;
        // A lazy vault's take-off leg follows the lead leg over, later.
        let late = if self.kind == VaultKind::Lazy { LAZY_TRAIL_LATE } else { 0.0 };
        let (start, end) = if leg == self.hand { (from + late, to.max(self.trail_until)) } else { (from - TUCK_EARLY, to) };
        if t <= start || t >= end {
            return (0.0, 0.0);
        }
        // At most half the flight to tuck (the take-off leg less: at 0.55 of
        // it its toe was not up over a 0.9 m wall in time, 1.6 cm in).
        // (A lazy vault's take-off leg, less carried round by the hips' roll
        // and starting late, more: at 0.45, 14.8 m/s.)
        let trail_share = if self.kind == VaultKind::Lazy { 0.6 } else { 0.45 };
        let (tuck_in, share, tuck_out) = if leg == self.hand { (TRAIL_TUCK_IN, trail_share, TRAIL_TUCK_IN) } else { (TUCK_IN, 0.5, TUCK_OUT) };
        let down = (end - tuck_out).max(start + 1.0e-3);
        // A share of the flight from take-off at least: of what was left once
        // a lazy vault's take-off leg started late, its knee whipped through
        // at 17.6 m/s.
        (ease(t, start, tuck_in.min(share * (to - start.min(from)))), 1.0 - ease(t, down, end - down))
    }

    /// Whether leg `leg` is on the floor `t` seconds into the jump: the
    /// take-off leg until it leaves, the lead leg once it lands.
    fn down(&self, t: f32, leg: usize) -> bool {
        if leg == self.hand { t <= self.flight.0 } else { t >= self.flight.1 }
    }

    /// How much the hand holds the top `t` seconds into the jump, 0-1:
    /// reaching for it, on it, lifting off it.
    fn holding(&self, t: f32) -> f32 {
        let (on, off) = self.contact;
        if t < on {
            smoothstep(((t - (on - REACH)) / REACH).clamp(0.0, 1.0))
        } else if t <= off {
            1.0
        } else {
            1.0 - smoothstep(((t - off) / LIFT).clamp(0.0, 1.0))
        }
    }

    /// The hips rolled, the trunk rolled back, the legs tucked out to the
    /// side, by how fully it is reshaped at `t`; `leap` is the leap's own
    /// pose there.
    fn reshape_body(&self, pose: &mut LocalPose, leap: &LocalPose, t: f32, rig: &RigGeometry) {
        let forward = rig.forward();
        let out = rig.left() * self.side;
        // A turn carrying `-Y` toward `out`.
        let axis = Vec3::NEG_Y.cross(out).normalize_or(forward);
        let w = self.rolling(t);
        let most = self.kind.roll();
        let roll = Quat::from_axis_angle(axis, most * w);
        let before = forward_kinematics_on(leap, rig);
        if w > 0.0 {
            pose.rotations[Bone::Hips] = delta_after_world_turn(pose, rig, Bone::Hips, roll);
            pose.rotations[Bone::Spine] = delta_after_world_turn(pose, rig, Bone::Spine, Quat::from_axis_angle(axis, -(most - self.trunk) * w));
        }
        // Each foot off the floor tucked in the rolled hips' frame, from
        // where the leap has it.
        let at = forward_kinematics_on(pose, rig);
        let hips = at[Bone::Hips];
        for (leg, (socket, ankle)) in LEGS.into_iter().enumerate() {
            let tuck = self.tucking(t, leg);
            if self.down(t, leg) || (tuck <= 0.0 && w <= 0.0) {
                continue;
            }
            // From where the rolled hips carry the leap's own ankle: from the
            // leap's ankle itself, the take-off leg hung back under the
            // rolled hips and its knee went 10 cm into a 0.9 m wall.
            let carried = hips + roll * (before[ankle] - before[Bone::Hips]);
            // A lazy vault's lead leg goes over nearly straight, ahead.
            let (ahead, under) = if self.kind == VaultKind::Lazy && leg != self.hand { LAZY_LEAD_TUCK } else { TUCK };
            let tucked = at[socket] + roll * (forward * ahead - Vec3::Y * under);
            let high = (self.obstacle.top + ANKLE_OVER).min(at[socket].y - ANKLE_UNDER_SOCKET);
            let tucked = Vec3::new(tucked.x, tucked.y.max(high), tucked.z);
            // The take-off leg tucking in, up first, then on: straight there,
            // the foot came forward as fast as it rose and reached a 0.9 m
            // wall's face low (3 cm in). Let down straight: the way it came,
            // it went back down through the obstacle (8 cm). (The lead foot,
            // ahead already, tucks straight: up first, its toe reached the
            // face, 2.4 cm in.)
            let (tucking_in, letting_down) = self.tuck_parts(t, leg);
            let target = if leg == self.hand && tucking_in < letting_down {
                let s = tucking_in;
                let raised = carried + Vec3::Y * (tucked.y - carried.y).max(0.0);
                carried * ((1.0 - s) * (1.0 - s)) + raised * (2.0 * s * (1.0 - s)) + tucked * (s * s)
            } else {
                carried.lerp(tucked, tuck)
            };
            place_ankle(pose, rig, ankle, target - hips);
            // The knee up, a little ahead, in the rolled hips' frame: kept in
            // its hinge, the take-off leg's knee swung across under the body
            // at 12 m/s as its ankle went over to the far side; pointed as
            // far ahead as up, it reached a 0.9 m wall's face before it was
            // over the top (2 cm in).
            knee_toward(pose, rig, [socket, KNEES[leg], ankle], roll * (forward * KNEE_AHEAD + Vec3::Y), tuck);
            // The foot level in the rolled frame, as standing: pointed as it
            // left, the take-off leg's toe hung 1.6 cm into a 0.9 m wall, and
            // the lead foot's toes 10 cm under its ankle 2.2 cm in.
            let now = accumulate_world_rotations(pose, rig)[ankle];
            let turn = Quat::IDENTITY.slerp((roll * self.feet[leg]) * now.inverse(), tuck);
            pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, turn);
        }
    }

    /// The leap's pose `leap` at `t` reshaped: the body, then moved whole
    /// to keep the leap's COM, then the hand onto the top. `travelled` is
    /// how far the jump's root has gone forward there.
    pub(crate) fn reshape(&self, leap: &LocalPose, t: f32, travelled: f32, rig: &RigGeometry) -> LocalPose {
        let mut pose = *leap;
        let holding = self.holding(t);
        if self.rolling(t) <= 0.0 && self.tucking(t, 0) <= 0.0 && self.tucking(t, 1) <= 0.0 && holding <= 0.0 {
            return pose;
        }
        self.reshape_body(&mut pose, leap, t, rig);
        // The hand off the top, one move of the whole body puts its COM on
        // the leap's: posed six times over every frame, a vault cost three
        // times a leap.
        if holding <= 0.0 {
            pose.root_translation += com(leap, rig) - com(&pose, rig);
            return pose;
        }
        // The COM is the leap's: the whole body moved by the reshaping's
        // miss, the arm solved afresh onto the plant each time from where
        // the body has the wrist, a few times over (the arm's reach moves the
        // COM a little). Lerped from the wrist as last solved, the target
        // crept on each time; turned on from the last solve, the clavicle
        // lifted again each time.
        let body = pose;
        let free = forward_kinematics_on(&body, rig)[ARMS[self.hand].wrist] - body.root_translation;
        for _ in 0..HAND_PASSES {
            let shift = com(leap, rig) - com(&pose, rig);
            let root = pose.root_translation + shift;
            pose = body;
            pose.root_translation = root;
            self.hand_on(&mut pose, root + free, travelled, holding, rig);
        }
        pose
    }

    /// The hand reaching from `from` (the wrist as the body has it) for its
    /// plant, on it, or lifting off it, by `holding`.
    fn hand_on(&self, pose: &mut LocalPose, from: Vec3, travelled: f32, holding: f32, rig: &RigGeometry) {
        let forward = rig.forward();
        let chain = ARMS[self.hand];
        let at = forward_kinematics_on(pose, rig);
        let plant = self.plant - forward * travelled;
        let target = from.lerp(plant, holding);
        let clavicle = if self.hand == 0 { Bone::LeftShoulder } else { Bone::RightShoulder };
        let arm = (at[chain.elbow] - at[chain.shoulder]).length() + (at[chain.wrist] - at[chain.elbow]).length();
        let lift = shoulder_lift(at[clavicle], at[chain.shoulder], target, 0.85 * arm);
        pose.rotations[clavicle] = delta_after_world_turn(pose, rig, clavicle, Quat::IDENTITY.slerp(lift, holding));
        let at = forward_kinematics_on(pose, rig);
        // The elbow back and out.
        let pole = (-forward + rig.left() * (SIGN[self.hand] * 0.5)).normalize();
        let (elbow, wrist) = solve_arm_toward_from(pose, &at, chain, target, pole, rig);
        turn_hand(pose, rig, chain, self.bind, self.press, holding, (wrist - elbow).normalize_or_zero());
    }

    /// Where the hand plants, the jump's frame; and when it is on the top.
    pub fn plant(&self) -> (Vec3, (f32, f32)) {
        (self.plant, self.contact)
    }

    /// The obstacle vaulted, the jump's frame.
    pub fn obstacle(&self) -> Obstacle {
        self.obstacle
    }
}

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

    /// What a vault measured, frame by frame.
    #[derive(Debug, Default)]
    struct Vaulted {
        /// The deepest any joint goes into the obstacle, metres, which and
        /// when.
        into: f32,
        deepest: Option<(Bone, f32)>,
        /// The most the planting hand's wrist strays from its plant while on
        /// it, metres.
        hand_off: f32,
        /// The most the COM strays from the leap's in the air, metres.
        com_off: f32,
        /// The fastest any joint moves relative to the COM, m/s, which and
        /// when; and the same for the leap unreshaped.
        fastest: f32,
        fastest_bone: Option<(Bone, f32)>,
        leap_fastest: f32,
        /// The run's speed going on, m/s.
        resumed: f32,
    }

    /// A vault (`kind`) over an obstacle `top` high, `thick` through, its
    /// face met `slant` off square, from a run at `speed`, off `leg`, `off`
    /// past its best take-off.
    fn vaulted(kind: VaultKind, top: f32, thick: f32, slant: f32, speed: f32, leg: usize, off: f32) -> Option<Vaulted> {
        let (stood, rig) = real_stood();
        let start = RunStart { leg, speed };
        let depth = thick / slant.cos();
        let near = Jump::vault_takeoff(depth, top, start, &stood, &rig) + off;
        let obstacle = Obstacle { near, depth, top, slant };
        let mut jump = Jump::vault(obstacle, start, kind, &stood, &rig)?;
        let forward = rig.forward();
        let com = |pose: &LocalPose| pose.root_translation + crate::character::anim::anthropometry::centre_of_mass(pose, &rig);
        // The jump's travel, read from a clone (the closure cannot borrow
        // the jump being advanced).
        let planned = jump.clone();
        let jump_travel = |t: f32| planned.travelled_at(t);
        let world = |pose: &LocalPose, t: f32| {
            let at = forward_kinematics_on(pose, &rig);
            BoneSet::from_fn(|bone| at[bone] + forward * jump_travel(t))
        };
        let mut m = Vaulted::default();
        let (plant, contact) = planned.vaulting().expect("vaulting").plant();
        // The last frame's joints and the hips' velocity, overall and leaping.
        type Frame = Option<(BoneSet<Vec3>, Vec3)>;
        let (mut last, mut last_leap): (Frame, Frame) = (None, None);
        while !jump.is_done() && jump.resumes().is_some_and(|_| jump.elapsed() < jump.ends(JumpPhase::Flight) + 0.3) {
            jump.advance(DT);
            let t = jump.elapsed();
            let pose = jump.pose(&stood, &rig);
            let leap = jump.pose_at_unshaped(t, &stood, &rig);
            let now = world(&pose, t);
            for bone in Bone::ALL {
                let depth = obstacle.inside(now[bone], forward);
                if depth > m.into {
                    (m.into, m.deepest) = (depth, Some((bone, t)));
                }
            }
            if (contact.0..=contact.1).contains(&t) {
                m.hand_off = m.hand_off.max((now[ARMS[planned.vaulting().unwrap().hand].wrist] - plant).length());
            }
            if jump.phase() == JumpPhase::Flight {
                m.com_off = m.com_off.max((com(&pose) - com(&leap)).length());
            }
            let c = com(&pose) + forward * jump_travel(t);
            let leap_now = world(&leap, t);
            let leap_c = com(&leap) + forward * jump_travel(t);
            if let (Some((before, before_c)), Some((leap_before, leap_before_c))) = (last, last_leap) {
                for bone in Bone::ALL {
                    let speed = ((now[bone] - c) - (before[bone] - before_c)).length() / DT;
                    if speed > m.fastest {
                        (m.fastest, m.fastest_bone) = (speed, Some((bone, t)));
                    }
                    m.leap_fastest = m.leap_fastest.max(((leap_now[bone] - leap_c) - (leap_before[bone] - leap_before_c)).length() / DT);
                }
            }
            last = Some((now, c));
            last_leap = Some((leap_now, leap_c));
        }
        m.resumed = planned.resumes().expect("runs on").speed;
        Some(m)
    }

    /// From a run at 3-4 m/s, over obstacles 0.75-1 m high and 0.25-0.5 m
    /// deep, taking off from either foot, from its best take-off to 0.4 m
    /// farther (the walker's last steps aim at the best): nothing goes
    /// into the obstacle, the hand is held on its plant, the COM flies the
    /// leap's parabola, no joint whips round, and it runs on little slower.
    #[test]
    fn it_vaults_a_low_obstacle_from_a_run_and_runs_on() {
        for (top, depth) in [(0.75, 0.25), (0.9, 0.3), (1.0, 0.5)] {
            for speed in [3.0, 4.0] {
                for (leg, off) in [0, 1].into_iter().flat_map(|leg| [0.0, 0.2, 0.4].map(|off| (leg, off))) {
                    let name = format!("{top} m high, {depth} m deep, {speed} m/s, off the {} {off:+} m from its best", if leg == 0 { "left" } else { "right" });
                    let m = vaulted(VaultKind::Speed, top, depth, 0.0, speed, leg, off).unwrap_or_else(|| panic!("{name}: not vaulted"));
                    clean(&name, &m, speed);
                }
            }
        }
    }

    /// From a run at 3-4 m/s at a 0.3 m thick obstacle 0.75-1 m high, met
    /// square or 0.3-0.6 rad off it either way, off either foot (off the
    /// one whose lead leg is on the near end's side, refused), 0.1 m past its
    /// best take-off (where the walker aims; it plans from there to 0.1-0.5 m
    /// past, the lead leg reaching ahead into the face nearer): a lazy vault
    /// is as clean as a speed vault.
    #[test]
    fn it_lazy_vaults_a_low_obstacle_from_an_angle() {
        for top in [0.75, 0.9, 1.0] {
            for (speed, slant) in [3.0, 4.0].into_iter().flat_map(|speed| [0.0, 0.3, -0.3, 0.6, -0.6].map(|slant| (speed, slant))) {
                for (leg, off) in [0, 1].into_iter().flat_map(|leg| [0.1].map(|off| (leg, off))) {
                    let name = format!("lazy, {top} m high, {slant:+} rad off square, {speed} m/s, off the {} {off:+} m from its best", if leg == 0 { "left" } else { "right" });
                    let vaulted = vaulted(VaultKind::Lazy, top, 0.3, slant, speed, leg, off);
                    // Off the foot whose lead leg is on the near end's side,
                    // refused.
                    if SIGN[1 - leg] * slant > 0.0 {
                        assert!(vaulted.is_none(), "{name}: vaulted toward the near end");
                        continue;
                    }
                    clean(&name, &vaulted.unwrap_or_else(|| panic!("{name}: not vaulted")), speed);
                }
            }
        }
    }

    /// Nothing into the obstacle, the hand held on its plant, the COM on
    /// the leap's parabola, no joint whipping round, running on.
    fn clean(name: &str, m: &Vaulted, speed: f32) {        // No joint faster about the COM than this, m/s: a sprinter's swing
        // foot goes about 10 m/s about the COM at full speed; the pops the
        // reshaping had (a shape switched on in a frame) were 21-61. No
        // vault data: the legs' whip round the side is set by eye.
        const MOST_JOINT_SPEED: f32 = 14.0;
        assert!(m.into < 1.0e-3, "{name}: {:?} {:.4} m into the obstacle", m.deepest, m.into);
        assert!(m.hand_off < 1.0e-3, "{name}: the hand {:.4} m off its plant", m.hand_off);
        assert!(m.com_off < 1.0e-3, "{name}: the COM {:.4} m off the leap's", m.com_off);
        // (Or the leap's own, past it once running on: 14.3 m/s at 4 m/s.)
        assert!(m.fastest < MOST_JOINT_SPEED.max(m.leap_fastest + 0.01), "{name}: {:?} at {:.2} m/s about the COM, the leap's fastest {:.2}", m.fastest_bone, m.fastest, m.leap_fastest);
        // Runs on no more than this much slower, m/s: a leap gives up speed
        // for its rise, and a lazy vault taken late rises more (from 4 m/s
        // it went on at 2.77). Traceurs lose about 0.2 (a known gap).
        assert!(m.resumed > speed - 1.25, "{name}: runs on at {:.2} m/s", m.resumed);
    }

    /// From runs at 3-4.5 m/s at obstacles 0.25-0.45 m high and 0.2-0.4 m
    /// deep, off either foot, from its best take-off to 0.4 m past it: a
    /// hop clears it (nothing into it), its COM rising little, and runs on
    /// little slower; too high or deep, it is not hopped.
    #[test]
    fn a_small_obstacle_is_hopped_in_stride() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let mut faults = Vec::new();
        for (top, depth) in [(0.25, 0.2), (0.35, 0.3), (0.45, 0.4)] {
            for speed in [3.0, 4.5] {
                for (leg, off) in [0, 1].into_iter().flat_map(|leg| [0.0, 0.2, 0.4].map(|off| (leg, off))) {
                    let name = format!("{top} m high, {depth} m deep, {speed} m/s, off {leg} {off:+} m");
                    let start = RunStart { leg, speed };
                    let obstacle = Obstacle::square(Jump::hop_takeoff(depth, start, &stood, &rig) + off, depth, top);
                    let Some(mut jump) = Jump::vault(obstacle, start, VaultKind::Hop, &stood, &rig) else {
                        faults.push(format!("{name}: not hopped"));
                        continue;
                    };
                    let (mut into, mut highest, mut fastest, mut kink) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
                    let com_at = |jump: &Jump| jump.com_height_at(jump.elapsed());
                    let leaves = jump.com_height_at(jump.ends(JumpPhase::Push));
                    let mut frames: Vec<(BoneSet<Vec3>, Vec3)> = Vec::new();
                    let (mut bares, mut bare_kink): (Vec<BoneSet<Vec3>>, f32) = (Vec::new(), 0.0);
                    while !jump.is_done() && jump.resumes().is_some_and(|_| jump.elapsed() < jump.ends(JumpPhase::Flight) + 0.3) {
                        jump.advance(DT);
                        let pose = jump.pose(&stood, &rig);
                        let at = forward_kinematics_on(&pose, &rig);
                        let travelled = forward * jump.travelled_at(jump.elapsed());
                        let now = BoneSet::from_fn(|bone| at[bone] + travelled);
                        let c = pose.root_translation + crate::character::anim::anthropometry::centre_of_mass(&pose, &rig) + travelled;
                        into = Bone::ALL.iter().map(|&bone| obstacle.inside(now[bone], forward)).fold(into, f32::max);
                        highest = highest.max(com_at(&jump));
                        if let Some((before, before_c)) = frames.last() {
                            fastest = Bone::ALL.iter().map(|&bone| ((now[bone] - c) - (before[bone] - *before_c)).length() / DT).fold(fastest, f32::max);
                        }
                        if frames.len() >= 2 {
                            let (a, b) = (&frames[frames.len() - 2].0, &frames[frames.len() - 1].0);
                            kink = Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(kink, f32::max);
                        }
                        frames.push((now, c));
                        let bare = forward_kinematics_on(&jump.pose_at_unshaped(jump.elapsed(), &stood, &rig), &rig);
                        bares.push(BoneSet::from_fn(|bone| bare[bone] + travelled));
                        if bares.len() >= 3 {
                            let n = bares.len();
                            bare_kink = Bone::ALL.iter().map(|&bone| (bares[n - 1][bone] - 2.0 * bares[n - 2][bone] + bares[n - 3][bone]).length()).fold(bare_kink, f32::max);
                        }
                    }
                    let resumed = jump.resumes().map_or(0.0, |r| r.speed);
                    eprintln!("{name}: into {into:.4}, rose {:.2}, runs on at {resumed:.2}, fastest {fastest:.1}, kink {kink:.4} (the leap's own {bare_kink:.4})", highest - leaves);
                    // The run's own fastest about the COM is 14.3 m/s at 4 m/s
                    // (`clean`). The leap's own step changes 11-13 cm in a
                    // frame as a planted toe leaves or meets the floor: the
                    // lift adds no more than a centimetre to it.
                    if into > 1.0e-3 || resumed < speed - 0.8 || fastest > 14.0 || kink > bare_kink + 0.01 {
                        faults.push(format!("{name}: into {into:.4}, runs on at {resumed:.2}, fastest {fastest:.1}, kink {kink:.4}"));
                    }
                }
            }
        }
        // And from as far past its best as the walker takes one (a step and
        // the slack).
        let start = RunStart { leg: 0, speed: 4.0 };
        for off in 0..13 {
            let near = Jump::hop_takeoff(0.3, start, &stood, &rig) + 0.1 * off as f32;
            if Jump::vault(Obstacle::square(near, 0.3, 0.35), start, VaultKind::Hop, &stood, &rig).is_none() {
                faults.push(format!("0.35 m high, 4 m/s, {:.1} m past its best: not hopped", 0.1 * off as f32));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
        let hop = |top: f32, depth: f32| Jump::vault(Obstacle::square(1.5, depth, top), RunStart { leg: 0, speed: 4.0 }, VaultKind::Hop, &stood, &rig).is_some();
        assert!(!hop(0.6, 0.3), "hopped a 0.6 m wall");
        assert!(!hop(0.3, 1.0), "hopped a 1 m deep block");
    }

    /// Standing jumps (0.35 m up, 1.4 m on; 0.45 m up, 1.8 m on) over
    /// obstacles 0.55-0.65 m and 0.65-0.75 m high (the lowest each does not
    /// clear by itself), 0.2-0.3 m deep, in their middle: untucked
    /// a leg goes into each; tucked, nothing does, the centre of mass keeps
    /// the jump's path, and no joint's step changes over 1 cm more in a
    /// frame than the jump's own.
    #[test]
    fn a_standing_jump_tucks_its_knees_over_an_obstacle() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        let mut faults = Vec::new();
        for (ask, low) in [(JumpAsk::forward(0.35, 1.4), 0.55f32), (JumpAsk::forward(0.45, 1.8), 0.65)] {
            let bare = Jump::plan(ask, &stood, &rig);
            for (top, depth) in [(low, 0.2f32), (low + 0.1, 0.2), (low, 0.3)] {
                let name = format!("{} m up, {} m on, over {top} m high, {depth} m deep", ask.height, ask.distance);
                let obstacle = Obstacle::square(0.5 * bare.distance() - 0.5 * depth + 0.1, depth, top);
                // How far into it, the most any joint's step changes in a
                // frame, the fastest joint (m/s) about the COM, and the COM's
                // greatest distance from the jump's.
                let run = |jump: &Jump| {
                    let (mut into, mut kink, mut fastest, mut com_off) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
                    let mut frames: Vec<(BoneSet<Vec3>, Vec3)> = Vec::new();
                    for k in 0..(jump.duration() / DT) as usize {
                        let t = k as f32 * DT;
                        let pose = jump.pose_at(t, &stood, &rig);
                        let at = forward_kinematics_on(&pose, &rig);
                        let travelled = forward * jump.travelled_at(t);
                        let now = BoneSet::from_fn(|bone| at[bone] + travelled);
                        let c = com(&pose, &rig) + travelled;
                        com_off = com_off.max((com(&pose, &rig) - com(&jump.pose_at_unshaped(t, &stood, &rig), &rig)).length());
                        into = Bone::ALL.iter().map(|&bone| obstacle.inside(now[bone], forward)).fold(into, f32::max);
                        if let Some((before, before_c)) = frames.last() {
                            fastest = Bone::ALL.iter().map(|&bone| ((now[bone] - c) - (before[bone] - *before_c)).length() / DT).fold(fastest, f32::max);
                        }
                        if frames.len() >= 2 {
                            let (a, b) = (&frames[frames.len() - 2].0, &frames[frames.len() - 1].0);
                            kink = Bone::ALL.iter().map(|&bone| (now[bone] - 2.0 * b[bone] + a[bone]).length()).fold(kink, f32::max);
                        }
                        frames.push((now, c));
                    }
                    (into, kink, fastest, com_off)
                };
                let (bare_into, bare_kink, bare_fastest, _) = run(&bare);
                let Some(tucked) = Jump::tuck_over(ask, obstacle, &stood, &rig) else {
                    faults.push(format!("{name}: not tucked over"));
                    continue;
                };
                let (into, kink, fastest, com_off) = run(&tucked);
                eprintln!("{name}: untucked into {bare_into:.3}; tucked into {into:.4}, kink {kink:.4} (its own {bare_kink:.4}), fastest {fastest:.1} ({bare_fastest:.1}), COM off {com_off:.5}");
                if bare_into < 0.01 || into > 1.0e-3 || kink > bare_kink + 0.01 || com_off > 1.0e-3 {
                    faults.push(format!("{name}: untucked into {bare_into:.3}; tucked into {into:.4}, kink {kink:.4} (its own {bare_kink:.4}), COM off {com_off:.4}"));
                }
            }
        }
        // And not only in its middle: a 0.55 m post anywhere 0.35-0.9 m
        // ahead of where it stands (nearer, a foot is over it on the
        // floor; farther, landing).
        for k in 0..=11 {
            let near = 0.35 + 0.05 * k as f32;
            if Jump::tuck_over(JumpAsk::forward(0.35, 1.4), Obstacle::square(near, 0.2, 0.55), &stood, &rig).is_none() {
                faults.push(format!("a 0.55 m post {near:.2} m ahead: not tucked over"));
            }
        }
        assert!(faults.is_empty(), "{faults:#?}");
    }

    /// Too low (a leap clears it), higher than the hips (a mantle), too
    /// deep, or from too slow a run, it is not vaulted.
    #[test]
    fn an_obstacle_out_of_a_vaults_reach_is_not_vaulted() {
        let (stood, rig) = real_stood();
        let start = RunStart { leg: 0, speed: 3.5 };
        let vault = |top: f32, depth: f32, start: RunStart| Jump::vault(Obstacle::square(1.0, depth, top), start, VaultKind::Speed, &stood, &rig).is_some();
        assert!(!vault(0.3, 0.3, start), "vaulted a 0.3 m kerb");
        assert!(!vault(1.2, 0.3, start), "vaulted a 1.2 m wall");
        assert!(!vault(0.9, 1.0, start), "vaulted a 1 m deep block");
        assert!(!vault(0.9, 0.3, RunStart { leg: 0, speed: 1.5 }), "vaulted from a walk");
        // Met too far off square for its kind.
        let slanted = |kind: VaultKind, slant: f32| Jump::vault(Obstacle { slant, ..Obstacle::square(1.0, 0.3, 0.9) }, start, kind, &stood, &rig).is_some();
        assert!(!slanted(VaultKind::Speed, 0.7), "speed vaulted 0.7 rad off square");
        assert!(!slanted(VaultKind::Lazy, 1.0), "lazy vaulted 1 rad off square");
    }
}
