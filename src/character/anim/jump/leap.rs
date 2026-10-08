//! Jumping from a run: a take-off from one foot, the flight, and a landing
//! on the other foot that runs on, or on both feet that stops.
//!
//! The run's own measured stance is replayed for the take-off leg, its foot
//! where the run planted it, while the centre of mass (COM) is moved from
//! where the run had it at contact to the take-off's speed up: it dips
//! deeper and drives up harder than a running step. The free leg drives its
//! thigh up and the arms swing harder (a running leap, Panoutsakopoulos et
//! al. 2010). The stance covers the ground a running step's does, the leg
//! sweeping the same angle, but takes longer by what the plant leg brakes:
//! a leap leaves a little slower than it ran ([`LOSS_PER_UP`]).
//!
//! Through the air the COM flies on a parabola, the body turning from the
//! take-off's shape into the run's own at the next contact; the landing
//! foot comes to rest over its spot before it lands. The landing stance
//! replays the run's stance on that leg as the take-off's did, absorbing
//! the fall, and hands back to the run at its toe-off: the clock set there,
//! the run going on at the speed it landed with.
//!
//! Landing on both feet, the take-off brakes harder on the plant leg (a
//! jump stop, [`LANDING_SPEED`]), the flight brings both feet to rest over
//! their spots, and the landing and standing up are the standing jump's,
//! braking what speed is left over the landed feet within a shoe's grip
//! ([`LANDING_BRAKE`]).

use bevy::math::{Quat, Vec3};

use super::{com_of, hips_of, Jump, JumpAsk, JumpPhase, GRAVITY, LANDING_KNEE, LEGS};
use crate::character::anim::gait::{self, hermite, smoothstep, walk_pose_on, GaitParams};
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, offset_from, LocalPose, RigGeometry};
use crate::character::anim::stance::{facing_sign, place_ankle, KNEE_AXIS};
use crate::character::skeleton::Bone;

/// How much forward speed a leap gives up for its speed up, m/s per m/s:
/// a recreational leap from 5 m/s leaving 2.4 m/s up loses about 0.8, a
/// hop from 3.5 m/s at 1.2 m/s up about 0.3; long jumpers lose 1.5-1.8 for
/// 3.2-3.5 up (Linthorne 2008). The plant leg, ahead of the body, brakes it.
pub const LOSS_PER_UP: f32 = 0.3;

/// The most it gives up to land short of where the run would carry it,
/// m/s per m/s up: braking harder on the plant leg.
pub const MOST_LOSS_PER_UP: f32 = 0.6;

/// The free leg's thigh and knee at a full leap's take-off, radians: the
/// thigh driven up toward the horizontal, the knee folded about 90°
/// (Panoutsakopoulos et al. 2010; coaching descriptions).
pub const DRIVE_THIGH: f32 = 1.3;
pub const DRIVE_KNEE: f32 = 1.6;

/// The speed up, m/s, from which the free leg drives fully; below it, in
/// proportion. A hop over something while jogging hardly lifts the knee.
pub const FULL_DRIVE_UP: f32 = 2.4;

/// How much harder the arms swing at a full leap's take-off, of the run's
/// own swing.
pub const ARM_DRIVE: f32 = 1.0;

/// The fastest a jump from a run that lands on both feet leaves forward,
/// m/s: a jump stop, braked on the plant leg (which, ahead of the body,
/// brakes easily: from 4 m/s down to 2 over 0.3 s is 6 m/s² against the
/// take-off's 2-3 body weights). Left at the run's speed, the two-foot
/// landing stopped 3.6 m/s in 0.1 s, braking at 3.5 g on the floor.
pub const LANDING_SPEED: f32 = 2.0;

/// How hard a two-foot landing brakes the COM forward, m/s²: 0.6 g, inside
/// what a shoe's grip holds under the landing's 1-2 body weights. The COM
/// touches down as far behind where it stands landed as that takes.
pub const LANDING_BRAKE: f32 = 6.0;

/// Where a jump from a run begins: the leg whose foot has just come down,
/// which takes off, 0 the left and 1 the right, and the run's speed, m/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunStart {
    pub leg: usize,
    pub speed: f32,
}

/// How finely a stance's carry is worked out ([`Stance::carried`]).
const CARRY_SAMPLES: usize = 32;

/// One stance on one leg: the run's stance at `params` replayed from
/// `contact`, its foot held where it is down, over `seconds`: the hips
/// going forward from `from.0` to `to.0` and the COM up from `from.1` to
/// `to.1` (with their rates per second).
///
/// Forward it is the hips that are planned, as the run moves them (at its
/// speed, with the root). The run's legs move its COM in the body by up to
/// 3 mm in a frame (at the toe-off, -0.1 then 2.9 then 0.4 mm): planned on
/// the COM, the hips took that kick instead, 3 mm in a frame two frames
/// before handing back.
#[derive(Debug, Clone, Copy)]
struct Stance {
    leg: usize,
    params: GaitParams,
    /// The cycle at the leg's contact, and the stance's share of a cycle.
    contact: f32,
    duty: f32,
    /// How far the body has gone over the foot, through the stance (in
    /// `CARRY_SAMPLES` steps): what keeps the sole's points on the floor
    /// still as the run's foot rolls from heel to toe. Carried at the run's
    /// speed instead, the heel slid 18 mm after it struck and the tip 17 mm
    /// before it left.
    carry: [Vec3; CARRY_SAMPLES + 1],
    /// Where the planted ankle is moved from where the run's pose has it.
    anchor: Vec3,
    seconds: f32,
    from: (f32, f32),
    to: (f32, f32),
    from_rate: (f32, f32),
    to_rate: (f32, f32),
    /// How far ahead of the hips the COM is at either end: what the root's
    /// travel adds to the hips', so it meets the flight's COM.
    offsets: (f32, f32),
    /// How far the free leg drives up and the arms swing harder, by the end.
    drive: f32,
}

impl Stance {
    /// The hips forward and the COM up `s` seconds in.
    fn hips_at(&self, s: f32) -> (f32, f32) {
        let u = (s / self.seconds).clamp(0.0, 1.0);
        let ahead = hermite(self.from.0, self.to.0, self.from_rate.0 * self.seconds, self.to_rate.0 * self.seconds, u);
        (ahead, pushed_height(self.from.1, self.from_rate.1, self.to.1, self.to_rate.1, self.seconds, u))
    }

    /// The way the root travels forward, and the COM up, `s` seconds in:
    /// the hips' way, plus the COM's offset ahead of them eased from one
    /// end's to the other's.
    fn com_at(&self, s: f32) -> (f32, f32) {
        let u = (s / self.seconds).clamp(0.0, 1.0);
        let (hips, up) = self.hips_at(s);
        (hips + self.offsets.0 + (self.offsets.1 - self.offsets.0) * smoothstep(u), up)
    }

    /// The run's pose `q` into the stance, at its own root, the free leg
    /// driven and the arms swung as far as `q` has them.
    fn shaped(&self, q: f32, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let drive = self.drive * smoothstep(q);
        let mut params = self.params;
        params.arm_swing *= 1.0 + ARM_DRIVE * drive;
        let mut pose = walk_pose_on(self.contact + q * self.duty, &params, stood, rig);
        if drive > 0.0 {
            drive_free_leg(&mut pose, rig, 1 - self.leg, drive);
        }
        pose
    }

    /// Where the planted ankle is `q` into the stance, in the jump's frame,
    /// given the run's pose there: the run's own, carried as far as the body
    /// has gone over the foot.
    fn ankle(&self, q: f32, pose: &LocalPose, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
        ankle_in_frame(pose, stood, rig, self.leg) + self.carried(q) + self.anchor
    }

    /// How far the body has gone over the foot `q` into the stance.
    fn carried(&self, q: f32) -> Vec3 {
        let at = q.clamp(0.0, 1.0) * CARRY_SAMPLES as f32;
        let i = (at as usize).min(CARRY_SAMPLES - 1);
        self.carry[i].lerp(self.carry[i + 1], at - i as f32)
    }

    /// Works out [`Self::carry`]: step by step through the stance, the body
    /// moved by as much as the sole's points on the floor moved back under
    /// it (those within 3 mm of the lowest; all of them on a flat foot).
    fn carries(&mut self, stood: &LocalPose, rig: &RigGeometry) {
        let sole = super::super::foot::Sole::of(rig, LEGS[self.leg].2);
        let points = |q: f32| {
            let pose = walk_pose_on(self.contact + q * self.duty, &self.params, stood, rig);
            sole.points(&pose, rig).map(|p| p + hips_of(&pose, stood))
        };
        let mut was = points(0.0);
        self.carry[0] = Vec3::ZERO;
        for i in 1..=CARRY_SAMPLES {
            let now = points(i as f32 / CARRY_SAMPLES as f32);
            let low = was.iter().map(|p| p.y).fold(f32::MAX, f32::min);
            let down: Vec<usize> = (0..3).filter(|&k| was[k].y < low + 0.003).collect();
            let back = down.iter().map(|&k| was[k] - now[k]).sum::<Vec3>() / down.len() as f32;
            self.carry[i] = self.carry[i - 1] + Vec3::new(back.x, 0.0, back.z);
            was = now;
        }
    }

    fn pose_at(&self, s: f32, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let q = (s / self.seconds).clamp(0.0, 1.0);
        let shaped = self.shaped(q, stood, rig);
        let ankle = self.ankle(q, &shaped, stood, rig);
        let attitude = accumulate_world_rotations(&shaped, rig)[LEGS[self.leg].2];
        // The other foot kept off the floor as the run keeps it swinging.
        let clear = match gait::leg_phase(self.contact + q * self.duty + 0.5, self.duty) {
            gait::LegPhase::Swing { progress } => crate::character::anim::run::swing_clearance(progress),
            gait::LegPhase::Stance { .. } => 0.0,
        };
        on_one_leg(&shaped, stood, rig, self.leg, ankle, attitude, self.hips_at(s), clear)
    }
}

/// Pushes off a wall holding a running leap up (running along a wall): each
/// one `start` seconds after take-off, `span` long, adding `gain` m/s up.
/// Each push's force rises and falls as sin² over its span, from none to
/// none, so the flight's acceleration has no step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lift {
    pub pushes: [(f32, f32, f32); 2],
}

impl Lift {
    /// How much higher the pushes have put the COM `s` seconds after
    /// take-off than the flight alone would, metres, and how much faster
    /// up, m/s.
    pub fn at(&self, s: f32) -> (f32, f32) {
        let tau = std::f32::consts::TAU;
        self.pushes.iter().filter(|&&(start, span, _)| s > start && span > 0.0).fold((0.0, 0.0), |(height, rate), &(start, span, gain)| {
            // Peak acceleration 2·gain/span: the pulse's mean is half.
            let most = 2.0 * gain / span;
            let u = ((s - start) / span).min(1.0);
            let v = most * span * (0.5 * u - (tau * u).sin() / (2.0 * tau));
            let p = most * span * span * (0.25 * u * u + ((tau * u).cos() - 1.0) / (2.0 * tau * tau));
            (height + p + v * (s - start - span).max(0.0), rate + v)
        })
    }
}

/// The flight: the COM thrown from `from` at `speed` forward and `up`, the
/// body turning from `takeoff`'s shape into `landing`'s (both placed in the
/// jump's frame), each landing leg's ankle (`lands`) brought from where it
/// left to its spot; held up by `lift`, if any.
#[derive(Debug, Clone, Copy)]
struct Flight {
    seconds: f32,
    from: (f32, f32),
    speed: f32,
    up: f32,
    takeoff: LocalPose,
    landing: LocalPose,
    lands: [bool; 2],
    lift: Option<Lift>,
}

impl Flight {
    /// How fast the hips go forward through the air at its start (`end`
    /// false) or its end, m/s: the COM's speed less how fast the COM moves
    /// ahead of them, the legs swinging under it. What the stances either
    /// side meet.
    fn hips_rate(&self, jump: &Jump, end: bool, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        const STEP: f32 = 1.0e-3;
        let forward = rig.forward();
        let ahead = |s: f32| {
            let pose = self.pose_at(jump, s, stood, rig);
            (com_of(&pose, stood, rig) - hips_of(&pose, stood)).dot(forward)
        };
        let (a, b) = if end { (self.seconds - STEP, self.seconds) } else { (0.0, STEP) };
        self.speed - (ahead(b) - ahead(a)) / STEP
    }

    fn com_at(&self, s: f32) -> (f32, f32) {
        let s = s.clamp(0.0, self.seconds);
        let lifted = self.lift.map_or(0.0, |lift| lift.at(s).0);
        (self.from.0 + self.speed * s, self.from.1 + self.up * s - 0.5 * GRAVITY * s * s + lifted)
    }

    /// How long a flight from `from` thrown `up`, held up by `lift`, takes
    /// to come down `drop` below where it left, seconds; and how fast it
    /// comes down then, m/s.
    fn falls(up: f32, drop: f32, lift: Option<Lift>) -> (f32, f32) {
        let ballistic = (up + (up * up + 2.0 * GRAVITY * drop).max(0.0).sqrt()) / GRAVITY;
        let Some(lift) = lift else {
            return (ballistic, GRAVITY * ballistic - up);
        };
        // Held up, it comes down later: past the ballistic time it is still
        // above, and it falls below eventually.
        let below = |s: f32| up * s - 0.5 * GRAVITY * s * s + lift.at(s).0 + drop;
        let (mut early, mut late) = (ballistic, ballistic + 0.5);
        while below(late) > 0.0 {
            late += 0.5;
        }
        for _ in 0..40 {
            let mid = 0.5 * (early + late);
            if below(mid) > 0.0 { early = mid } else { late = mid }
        }
        let seconds = 0.5 * (early + late);
        (seconds, GRAVITY * seconds - up - lift.at(seconds).1)
    }

    fn pose_at(&self, jump: &Jump, s: f32, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let u = (s / self.seconds).clamp(0.0, 1.0);
        let blend = smoothstep(u);
        let mut shape = self.takeoff;
        for bone in Bone::ALL {
            shape.rotations[bone] = self.takeoff.rotations[bone].slerp(self.landing.rotations[bone], blend);
        }
        shape.root_translation = self.takeoff.root_translation.lerp(self.landing.root_translation, blend);
        // The landing foot's way across the floor held in the world, from
        // where it was as the other left to its spot, at rest at both ends:
        // carried with the body instead, it came down moving with it, and
        // the run's next stance planted a skidding foot. Its height under
        // the hips from the two shapes; raised where the leg cannot reach
        // that far down (see the standing jump's flight).
        let across = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let world = |pose: &LocalPose, leg: usize| accumulate_world_rotations(pose, rig)[LEGS[leg].2];
        let feet = [0, 1].map(|leg| {
            let (left, lands) = (ankle_in_frame(&self.takeoff, stood, rig, leg), ankle_in_frame(&self.landing, stood, rig, leg));
            let below = |pose: &LocalPose, ankle: Vec3| ankle.y - hips_of(pose, stood).y;
            let (from, to) = (below(&self.takeoff, left), below(&self.landing, lands));
            (across(left.lerp(lands, blend)), from + (to - from) * blend, world(&self.takeoff, leg).slerp(world(&self.landing, leg), blend))
        });
        let (ahead, height) = self.com_at(s);
        let forward = rig.forward();
        let mut pose = shape;
        for _ in 0..8 {
            let mut trial = shape;
            trial.root_translation = pose.root_translation;
            let hips = hips_of(&trial, stood);
            for (leg, &(way, below, attitude)) in feet.iter().enumerate() {
                if !self.lands[leg] {
                    continue;
                }
                let socket = hips + offset_from(&trial, rig, Bone::Hips, LEGS[leg].0);
                let mut target = way + Vec3::Y * (hips.y + below);
                let (reach, out) = (jump.feet.reach(leg, LANDING_KNEE), target - socket);
                let level = across(out).length_squared();
                if out.length_squared() > reach * reach && level < reach * reach {
                    target = socket + across(out) - Vec3::Y * (reach * reach - level).sqrt();
                }
                set_ankle(&mut trial, stood, rig, leg, target, attitude);
            }
            let com = com_of(&trial, stood, rig);
            let (miss_ahead, miss_up) = (ahead - com.dot(forward), height - com.y);
            pose = trial;
            if miss_ahead.hypot(miss_up) < 1.0e-5 {
                break;
            }
            pose.root_translation += forward * (miss_ahead / 0.85) + Vec3::Y * miss_up;
        }
        pose
    }
}

/// The run's part of a jump from it: the take-off and the flight, and
/// running on, the landing stance. Landing on both feet, the landing and
/// standing up are the standing jump's own.
#[derive(Debug, Clone, Copy)]
pub(super) struct FromRun {
    takeoff: Stance,
    flight: Flight,
    landing: Option<(Stance, Resume)>,
}

/// Where the run picks up once a leap from it is done.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Resume {
    /// The run's clock at the landing foot's toe-off, and how fast it goes
    /// at the speed landed with, cycles a second.
    pub cycle: f32,
    pub rate: f32,
    /// The run's speed, m/s: the root's, as the walker moves a run.
    pub speed: f32,
    /// How far the root moves to meet the run's pose there, in the pose's
    /// frame.
    pub handover: Vec3,
}

impl FromRun {
    /// Whether `t` seconds in is this part's to plan and pose: all of it
    /// running on; landing on both feet, until touchdown.
    pub fn owns(&self, jump: &Jump, t: f32) -> bool {
        self.landing.is_some() || matches!(jump.phase_at(t), JumpPhase::Down | JumpPhase::Push | JumpPhase::Flight)
    }

    /// The COM (ahead, up) `t` seconds in, in the jump's frame, where it
    /// [`Self::owns`] it.
    pub fn com_at(&self, jump: &Jump, t: f32) -> (f32, f32) {
        let [_, push, flight, ..] = jump.ends;
        match (jump.phase_at(t), &self.landing) {
            (JumpPhase::Down | JumpPhase::Push, _) => self.takeoff.com_at(t),
            (JumpPhase::Land | JumpPhase::Recover, Some((landing, _))) => landing.com_at(t - flight),
            _ => self.flight.com_at(t - push),
        }
    }

    /// The pose `t` seconds in, in the jump's frame (before its travel is
    /// taken off the root), where it [`Self::owns`] it.
    pub fn pose_at(&self, jump: &Jump, t: f32, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let [_, push, flight, ..] = jump.ends;
        match (jump.phase_at(t), &self.landing) {
            (JumpPhase::Down | JumpPhase::Push, _) => self.takeoff.pose_at(t, stood, rig),
            (JumpPhase::Land | JumpPhase::Recover, Some((landing, _))) => landing.pose_at(t - flight, stood, rig),
            _ => self.flight.pose_at(jump, t - push, stood, rig),
        }
    }

    /// On one foot `t` seconds in, where the plan has the hips forward, in
    /// the jump's frame (there the hips are planned forward, not the COM).
    #[cfg(test)]
    pub fn hips_ahead_at(&self, jump: &Jump, t: f32) -> Option<f32> {
        let flight = jump.ends[2];
        match (jump.phase_at(t), &self.landing) {
            (JumpPhase::Down | JumpPhase::Push, _) => Some(self.takeoff.hips_at(t).0),
            (JumpPhase::Land | JumpPhase::Recover, Some((landing, _))) => Some(landing.hips_at(t - flight).0),
            _ => None,
        }
    }

    /// Which feet are down `t` seconds in: the take-off foot, none, then
    /// the landing foot, or both.
    pub fn feet_down(&self, jump: &Jump, t: f32) -> [bool; 2] {
        match (jump.phase_at(t), &self.landing) {
            (JumpPhase::Down | JumpPhase::Push, _) => [0, 1].map(|i| i == self.takeoff.leg),
            (JumpPhase::Flight, _) => [false; 2],
            (_, Some((landing, _))) => [0, 1].map(|i| i == landing.leg),
            (_, None) => [true; 2],
        }
    }
}

impl Jump {
    /// A jump as `ask`ed from a run at `start.speed`, the foot of
    /// `start.leg` having just come down (the run's clock at its contact):
    /// it takes off from that foot, lands on the other and runs on.
    pub fn from_run(ask: JumpAsk, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let height = ask.height.clamp(0.02, super::HIGHEST);
        let up = (2.0 * GRAVITY * height).sqrt();
        let forward = rig.forward();
        let mut jump = Self::plan(JumpAsk::up(height), stood, rig);
        let leg_length = gait::leg_length_of(rig);
        let run_at = |speed: f32| {
            let params = GaitParams::running_on(speed, rig);
            let reference = crate::character::anim::run::reference_speed(speed, leg_length);
            let cycle = crate::character::anim::run::distance_per_cycle(stood, rig, reference) / speed.max(0.1);
            (params, cycle)
        };
        let (before, cycle) = run_at(start.speed);
        let contact = 0.5 * start.leg as f32;
        let lead = 1 - start.leg;
        let com = |pose: &LocalPose| {
            let c = com_of(pose, stood, rig);
            (c.dot(forward), c.y)
        };
        // How fast the run's COM goes forward and up at cycle `p`, m/s: as
        // the run moves the root (at its speed, as the walker does) and the
        // COM moves in the body. Read off a different speed than the run
        // goes at, the hand-back stepped the pelvis 11 mm in a frame.
        let rate_at = |params: &GaitParams, speed: f32, cycle: f32, p: f32| {
            const STEP: f32 = 0.002;
            let (now, next) = (com(&walk_pose_on(p, params, stood, rig)), com(&walk_pose_on(p + STEP, params, stood, rig)));
            ((next.0 - now.0) / (STEP * cycle) + speed, (next.1 - now.1) / (STEP * cycle))
        };

        // The take-off: the run's stance, its COM leaving at the speed up
        // asked and a little slower forward.
        let drive = (up / FULL_DRIVE_UP).min(1.0);
        let mut takeoff = Stance {
            leg: start.leg,
            params: before,
            contact,
            duty: before.duty_factor,
            carry: [Vec3::ZERO; CARRY_SAMPLES + 1],
            anchor: Vec3::ZERO,
            seconds: 0.0,
            from: (0.0, 0.0),
            to: (0.0, 0.0),
            // The hips going on as the run moves them, the COM rising as the
            // run's does.
            from_rate: (start.speed, rate_at(&before, start.speed, cycle, contact).1),
            to_rate: (0.0, up),
            offsets: (0.0, 0.0),
            drive,
        };
        takeoff.carries(stood, rig);
        let hips = |pose: &LocalPose| hips_of(pose, stood).dot(forward);
        let first = takeoff.shaped(0.0, stood, rig);
        takeoff.from = (hips(&first), com(&first).1);
        // Leaving: the run's toe-off with the free leg driven, the body
        // carried over the foot.
        let carried = takeoff.carried(1.0);
        let mut leaving = takeoff.shaped(1.0, stood, rig);
        leaving.root_translation += carried;
        // And risen as far as the plant leg reaches, all but straight: a
        // leap leaves with the body high over its foot. Left at the run's
        // toe-off height, 27 mm over its contact's, the COM had to gain
        // 2.8 m/s up rising almost none, and pulled on the floor (-0.4
        // body weights) dipping first.
        let socket = hips_of(&leaving, stood) + offset_from(&leaving, rig, Bone::Hips, LEGS[start.leg].0);
        let ankle = ankle_in_frame(&leaving, stood, rig, start.leg);
        let reach = jump.feet.reach(start.leg, super::KNEE_AT_TAKEOFF);
        let level = Vec3::new(ankle.x - socket.x, 0.0, ankle.z - socket.z).length_squared();
        leaving.root_translation.y += (ankle.y + (reach * reach - level).max(0.0).sqrt() - socket.y).max(0.0);
        let left = com(&leaving);
        takeoff.to = (hips(&leaving), left.1);
        takeoff.offsets = (com(&first).0 - takeoff.from.0, left.0 - takeoff.to.0);

        let mut speed = (start.speed - LOSS_PER_UP * up).max(0.5 * start.speed);
        let toe = |pose: &LocalPose, leg: usize| {
            let bone = super::super::foot::foot_bones(LEGS[leg].2).1;
            (hips_of(pose, stood) + offset_from(pose, rig, Bone::Hips, bone)).dot(forward)
        };
        let took_off_at = toe(&takeoff.shaped(0.0, stood, rig), start.leg);
        // Asked a distance, toe to toe: the speed that lands it there,
        // between flying on at the run's and braking hard.
        let aimed = |speed: f32, reached: f32, seconds: f32| {
            if ask.distance > 0.0 { (speed + (ask.distance - reached) / seconds).clamp(start.speed - MOST_LOSS_PER_UP * up, start.speed) } else { speed }
        };
        if !ask.keep_running {
            return jump.lands_on_both_feet(takeoff, left, speed, up, stood, rig, |speed, reached, seconds| aimed(speed, reached - took_off_at, seconds));
        }
        let mut flight = None;
        let mut landing_stance = None;
        let mut leaving_rate = speed;
        for _ in 0..3 {
            // The stance sweeps the leg through the run's angle, braking
            // from the run's speed to the leap's: the hips leave as fast as
            // they go on through the air.
            takeoff.to_rate.0 = leaving_rate;
            takeoff.seconds = 2.0 * (takeoff.to.0 - takeoff.from.0) / (takeoff.from_rate.0 + leaving_rate);
            let thrown = takeoff.pose_at(takeoff.seconds, stood, rig);

            // The landing: the run's contact on the other leg, at the speed
            // it lands with.
            let (after, after_cycle) = run_at(speed);
            let lands_at = contact + 0.5;
            let mut landing = walk_pose_on(lands_at, &after, stood, rig);
            let touch = com(&landing);
            let drop = left.1 - touch.1;
            let (seconds, falling) = Flight::falls(up, drop, ask.lift);
            let ahead = left.0 + speed * seconds;
            landing.root_translation += forward * (ahead - touch.0);
            let landed = Flight { seconds, from: left, speed, up, takeoff: thrown, landing, lands: [0, 1].map(|i| i == lead), lift: ask.lift };
            leaving_rate = landed.hips_rate(&jump, false, stood, rig);

            // The landing stance: the run's on the landing leg, its COM
            // falling in at the flight's speed down and leaving as the run's
            // toe-off does.
            let duty = after.duty_factor;
            let mut stance = Stance {
                leg: lead,
                params: after,
                contact: lands_at,
                duty,
                carry: [Vec3::ZERO; CARRY_SAMPLES + 1],
                anchor: Vec3::ZERO,
                seconds: 0.0,
                // The hips coming in as they flew, the COM falling at the
                // flight's speed down.
                from: (hips(&landing), touch.1),
                to: (0.0, 0.0),
                from_rate: (landed.hips_rate(&jump, true, stood, rig), -falling),
                // Leaving as the run does from its toe-off, where it picks
                // up: the hips with the root, at its speed.
                to_rate: (speed, rate_at(&after, speed, after_cycle, lands_at + duty - 0.002).1),
                offsets: (0.0, 0.0),
                drive: 0.0,
            };
            stance.carries(stood, rig);
            let over = stance.carried(1.0).dot(forward);
            let first = walk_pose_on(lands_at, &after, stood, rig);
            stance.anchor = ankle_in_frame(&landing, stood, rig, lead) - ankle_in_frame(&first, stood, rig, lead);
            let last = walk_pose_on(lands_at + duty, &after, stood, rig);
            stance.to = (stance.from.0 + over, com(&last).1);
            stance.offsets = (ahead - stance.from.0, com(&last).0 - hips(&last));
            // Over the foot from the speed it landed with to the run's.
            stance.seconds = 2.0 * over / (stance.from_rate.0 + speed);
            flight = Some(landed);
            landing_stance = Some(stance);
            if ask.distance <= 0.0 {
                break;
            }
            speed = aimed(speed, toe(&landed.landing, lead) - took_off_at, seconds);
        }
        let (flight, landing) = (flight.expect("planned"), landing_stance.expect("planned"));
        jump.distance = toe(&flight.landing, lead) - took_off_at;
        jump.speed = flight.speed;
        jump.up = up;
        let mut end = 0.0;
        for (slot, span) in [0.0, takeoff.seconds, flight.seconds, landing.seconds, 0.0].into_iter().enumerate() {
            end += span;
            jump.ends[slot] = end;
        }
        let resume_cycle = landing.contact + landing.duty;
        let (after, after_cycle) = run_at(flight.speed);
        let resume = Resume { cycle: resume_cycle, rate: 1.0 / after_cycle, speed: flight.speed, handover: Vec3::ZERO };
        jump.run = Some(Box::new(FromRun { takeoff, flight, landing: Some((landing, resume)) }));
        // What the run's pose at its toe-off has the root at, against where
        // the jump leaves it: moved to meet it as the run picks up.
        let resumed = walk_pose_on(resume_cycle, &after, stood, rig);
        let left = jump.pose_at(jump.duration(), stood, rig);
        let handover = left.root_translation - resumed.root_translation;
        if let Some((_, resume)) = jump.run.as_mut().and_then(|run| run.landing.as_mut()) {
            resume.handover = Vec3::new(handover.x, 0.0, handover.z);
        }
        jump
    }

    /// The rest of a jump from a run that lands on both feet and stops,
    /// taking off as `takeoff` plans at about `speed` forward and `up`: the
    /// standing jump's forefoot touchdown, landing and standing up, the
    /// landing braking the run's speed over the landed feet. `aimed` gives
    /// the next pass's speed from this one's, how far ahead its toes land
    /// and its flight's seconds.
    #[allow(clippy::too_many_arguments)]
    fn lands_on_both_feet(
        mut self,
        mut takeoff: Stance,
        left: (f32, f32),
        mut speed: f32,
        up: f32,
        stood: &LocalPose,
        rig: &RigGeometry,
        aimed: impl Fn(f32, f32, f32) -> f32,
    ) -> Self {
        use super::{Aim, ARMS_LANDING, DEEPEST_LANDING, FULL_ARMS, LANDING_DECELERATION, LANDING_HEEL, LEAN_PER_DEPTH, LEAN_PER_SPEED, MOST_LEAN, QUICKEST_DOWN, RECOVERY_ACCELERATION};
        let forward = rig.forward();
        let com = |pose: &LocalPose| {
            let c = com_of(pose, stood, rig);
            (c.dot(forward), c.y)
        };
        let toes = |pose: &LocalPose| {
            LEGS.iter().map(|&(_, _, ankle)| (hips_of(pose, stood) + offset_from(pose, rig, Bone::Hips, super::super::foot::foot_bones(ankle).1)).dot(forward)).sum::<f32>() * 0.5
        };
        let height = up * up / (2.0 * GRAVITY);
        let stand = self.stand;
        let mut planned = None;
        let mut leaving_rate = None;
        speed = speed.min(LANDING_SPEED);
        for pass in 0..4 {
            takeoff.to_rate.0 = leaving_rate.unwrap_or(speed);
            takeoff.seconds = 2.0 * (takeoff.to.0 - takeoff.from.0) / (takeoff.from_rate.0 + takeoff.to_rate.0);
            let thrown = takeoff.pose_at(takeoff.seconds, stood, rig);
            (self.speed, self.pace) = (speed, speed);
            self.arms = ((height + speed * speed / (2.0 * GRAVITY)) / FULL_ARMS).min(1.0).powi(2);
            // Touching down on the forefoot as a standing jump does, its COM
            // as far behind where it stands landed as the landing brakes it
            // over at `LANDING_BRAKE`; the feet as far ahead as the flight
            // carries it.
            let behind = speed * speed / (2.0 * LANDING_BRAKE);
            let mut meeting = thrown;
            let mut seconds = 0.0;
            // Settled, as `Jump::plan`'s: the landing poses the lean of the
            // height it touched down at, and the legs, solved to just reach,
            // come up short of a lean a few millimetres apart.
            for _ in 0..5 {
                let drop = left.1 - self.touchdown;
                seconds = (up + (up * up + 2.0 * GRAVITY * drop).max(0.0).sqrt()) / GRAVITY;
                let ahead = left.0 + speed * seconds;
                self.distance = ahead + behind - stand.0;
                let lean = (LEAN_PER_DEPTH * (stand.1 - self.touchdown).max(0.0) + LEAN_PER_SPEED * speed).min(MOST_LEAN);
                let shaped = self.upper(stood, rig, lean, self.arm_shape(ARMS_LANDING));
                meeting = self.solved(&shaped, stood, rig, ahead, Aim::Reach { heel: LANDING_HEEL, knee: LANDING_KNEE }, self.distance);
                self.touchdown = com(&meeting).1;
            }
            // The landing and standing up, as `Jump::plan`'s.
            self.down = GRAVITY * seconds - up;
            let deepest = DEEPEST_LANDING - (stand.1 - self.touchdown).max(0.0);
            let absorb = (self.down * self.down / (2.0 * LANDING_DECELERATION)).min(deepest);
            self.lowest = self.touchdown - absorb;
            let land_time = 2.0 * absorb / self.down;
            let recover_time = QUICKEST_DOWN.max((6.0 * (stand.1 - self.lowest) / RECOVERY_ACCELERATION).sqrt());
            let mut end = 0.0;
            for (slot, span) in [0.0, takeoff.seconds, seconds, land_time, recover_time].into_iter().enumerate() {
                end += span;
                self.ends[slot] = end;
            }
            let flight = Flight { seconds, from: left, speed, up, takeoff: thrown, landing: meeting, lands: [true; 2], lift: None };
            // The hips leave as fast as they go on through the air.
            leaving_rate = Some(flight.hips_rate(&self, false, stood, rig));
            planned = Some(flight);
            if pass < 3 {
                speed = aimed(speed, toes(&meeting), seconds).min(LANDING_SPEED);
            }
        }
        let flight = planned.expect("planned");
        // The last pass's take-off, leaving at its flight's rate.
        let pushing = takeoff.seconds;
        takeoff.to_rate.0 = leaving_rate.unwrap_or(speed);
        takeoff.seconds = 2.0 * (takeoff.to.0 - takeoff.from.0) / (takeoff.from_rate.0 + takeoff.to_rate.0);
        for end in self.ends.iter_mut().skip(1) {
            *end += takeoff.seconds - pushing;
        }
        // The flight's way forward meets the touchdown shape exactly, and the
        // landing brakes it to rest over the feet in its own time.
        self.touched = com(&flight.landing).0;
        self.braking = 2.0 * (stand.0 + self.distance - self.touched).max(1.0e-3) / speed.max(1.0e-3);
        self.up = up;
        self.run = Some(Box::new(FromRun { takeoff, flight, landing: None }));
        // Stood up over the landed feet: where the standing pose has the
        // root, against where the jump leaves it.
        let left = self.pose_at(self.duration(), stood, rig);
        let settle = left.root_translation - stood.root_translation;
        self.settle = Vec3::new(settle.x, 0.0, settle.z);
        self
    }

    /// From a run, which feet are down now; `None` for a standing jump.
    pub fn run_feet_down(&self) -> Option<[bool; 2]> {
        self.run.as_ref().map(|run| run.feet_down(self, self.t))
    }

    /// From a run, landing on the other foot: where the run picks up once
    /// it is done.
    pub fn resumes(&self) -> Option<Resume> {
        self.run.as_ref().and_then(|run| run.landing.map(|(_, resume)| resume))
    }
}

/// The COM's height `u` (0-1) through a stance of `seconds` on one foot,
/// from `from` rising at `from_rate` to `to` rising at `to_rate` (m, m/s).
///
/// Shaped by the floor's push, not the path: the push is nothing as the foot
/// touches and as it leaves (the COM falling at g, as in the flights either
/// side) and never pulls, between them a bump `u^p(1-u)` — late in a
/// take-off, which gains much speed for little rise, and early in a
/// landing (the same bump reversed in time). Its size and `p` meet both
/// ends exactly. A cubic Hermite through the same ends has a straight-line
/// acceleration: a leap leaving at 2.4 m/s from 27 mm over its contact
/// asked the floor to pull (-0.4 body weights) as the foot came down; a
/// quintic held at g at both ends still pulled (-0.5) and dipped 11 cm.
pub(super) fn pushed_height(from: f32, from_rate: f32, to: f32, to_rate: f32, seconds: f32, u: f32) -> f32 {
    let t = seconds;
    // The push's whole impulse (per second, over the stance), and how early
    // it comes: the share of it whose effect on the rise is as if it all
    // came at the start, 0.5 for a push symmetric in time.
    let whole = (to_rate - from_rate) / t + GRAVITY;
    let early = ((to - from - from_rate * t) / (t * t) + 0.5 * GRAVITY) / whole;
    if !(whole > 0.0 && (0.05..0.95).contains(&early)) {
        return hermite(from, to, from_rate * t, to_rate * t, u);
    }
    // ∫∫ of u^p(1-u) from 0, and the bump's size for the whole impulse.
    let twice = |p: f32, u: f32| u.powf(p + 2.0) / ((p + 1.0) * (p + 2.0)) - u.powf(p + 3.0) / ((p + 2.0) * (p + 3.0));
    let size = |p: f32| whole * (p + 1.0) * (p + 2.0);
    if early <= 0.5 {
        // Late: from the start.
        let p = 2.0 / early - 3.0;
        from + from_rate * t * u + t * t * (-0.5 * GRAVITY * u * u + size(p) * twice(p, u))
    } else {
        // Early: the same, from the end back.
        let p = 2.0 / (1.0 - early) - 3.0;
        let back = 1.0 - u;
        to - to_rate * t * back + t * t * (-0.5 * GRAVITY * back * back + size(p) * twice(p, back))
    }
}

/// `leg`'s ankle in the jump's frame (the standing hips').
fn ankle_in_frame(pose: &LocalPose, stood: &LocalPose, rig: &RigGeometry, leg: usize) -> Vec3 {
    hips_of(pose, stood) + offset_from(pose, rig, Bone::Hips, LEGS[leg].2)
}

/// Puts `leg`'s ankle at `ankle` (the jump's frame), its foot turned to
/// `attitude` in the world.
fn set_ankle(pose: &mut LocalPose, stood: &LocalPose, rig: &RigGeometry, leg: usize, ankle: Vec3, attitude: Quat) {
    let bone = LEGS[leg].2;
    place_ankle(pose, rig, bone, ankle - hips_of(pose, stood));
    let now = accumulate_world_rotations(pose, rig)[bone];
    pose.rotations[bone] = delta_after_world_turn(pose, rig, bone, attitude * now.inverse());
}

/// `shaped` standing on `leg`, its ankle at `ankle` and its foot at
/// `attitude`, the pelvis moved to `hips.0` forward and so the COM is at
/// `hips.1` up.
///
/// The other foot is kept `clear` off the floor at least: lowered with the
/// pelvis below where the run has it, a swinging foot scraped 3.8 mm into
/// the floor through a jump stop's longer take-off.
#[allow(clippy::too_many_arguments)]
fn on_one_leg(shaped: &LocalPose, stood: &LocalPose, rig: &RigGeometry, leg: usize, ankle: Vec3, attitude: Quat, hips: (f32, f32), clear: f32) -> LocalPose {
    let forward = rig.forward();
    let free = 1 - leg;
    let sole = super::super::foot::Sole::of(rig, LEGS[free].2);
    let floor = sole.points(stood, rig).iter().map(|p| p.y).fold(f32::MAX, f32::min);
    let mut shift = Vec3::ZERO;
    let mut pose = *shaped;
    // The COM moves with the pelvis less what the planted leg keeps back.
    for _ in 0..8 {
        let mut trial = *shaped;
        trial.root_translation += shift;
        set_ankle(&mut trial, stood, rig, leg, ankle, attitude);
        let low = sole.points(&trial, rig).iter().map(|p| p.y + hips_of(&trial, stood).y).fold(f32::MAX, f32::min);
        if low < floor + clear {
            let lifted = ankle_in_frame(&trial, stood, rig, free) + Vec3::Y * (floor + clear - low);
            let held = accumulate_world_rotations(&trial, rig)[LEGS[free].2];
            set_ankle(&mut trial, stood, rig, free, lifted, held);
        }
        let ahead = hips.0 - hips_of(&trial, stood).dot(forward);
        let up = hips.1 - com_of(&trial, stood, rig).y;
        pose = trial;
        if ahead.hypot(up) < 1.0e-5 {
            break;
        }
        shift += forward * ahead + Vec3::Y * (up / 0.85);
    }
    pose
}

/// Drives `leg`'s thigh up and folds its knee, `drive` of the way to a full
/// leap's ([`DRIVE_THIGH`], [`DRIVE_KNEE`]).
fn drive_free_leg(pose: &mut LocalPose, rig: &RigGeometry, leg: usize, drive: f32) {
    let (hip, knee, ankle) = LEGS[leg];
    let facing = facing_sign(rig);
    let [thigh, _, knee_angle, _] = gait::sagittal_angles(pose, rig, gait::leg_joints(ankle));
    let turn = |angle: f32| Quat::from_axis_angle(KNEE_AXIS, facing * angle);
    gait::compose(pose, hip, turn(drive * (DRIVE_THIGH - thigh)));
    gait::compose(pose, knee, turn(-(drive * (DRIVE_KNEE - knee_angle))));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::foot::Sole;
    use crate::character::anim::rig::forward_kinematics_on;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// Leaps checked: a hop over something while running at 3 m/s, and a
    /// leap at 4 m/s.
    const LEAPS: [(f32, JumpAsk); 2] = [(3.0, JumpAsk::running(0.08, 0.0)), (4.0, JumpAsk::running(0.3, 0.0))];

    /// The hips' way forward frame by frame at 60 Hz, as the walker moves
    /// the body: the run up to the take-off foot's contact (`before`
    /// seconds into the frame), the jump, the hand-back within the frame the
    /// landing foot leaves in, and the run after; and which frame handed
    /// back.
    fn hips_through(ask: JumpAsk, speed: f32, before: f32) -> (Vec<f32>, usize) {
        let (stood, rig) = real_stood();
        let f = rig.forward();
        let dt = 1.0 / 60.0;
        let mut jump = Jump::from_run(ask, RunStart { leg: 0, speed }, &stood, &rig);
        let mut body = 0.0;
        // Two frames of the run before, the last `before` seconds short of
        // the take-off foot's contact (cycle 0).
        let running = GaitParams::running_on(speed, &rig);
        let reference = crate::character::anim::run::reference_speed(speed, gait::leg_length_of(&rig));
        let rate = speed / crate::character::anim::run::distance_per_cycle(&stood, &rig, reference);
        let mut hips: Vec<f32> = [dt + before, before]
            .iter()
            .map(|&ago| -speed * (ago - before) + walk_pose_on(-ago * rate, &running, &stood, &rig).root_translation.dot(f))
            .collect();
        jump.advance(dt - before);
        body += jump.travelled() + before * speed;
        hips.push(body + jump.pose_at(jump.elapsed(), &stood, &rig).root_translation.dot(f));
        let resume = jump.resumes().unwrap();
        let after = GaitParams::running_on(resume.speed, &rig);
        let pose_at = |q: f32| walk_pose_on(q, &after, &stood, &rig);
        let (mut cycle, mut handed) = (None, 0);
        for _ in 0..80 {
            match cycle {
                None => {
                    let past = jump.elapsed() + dt - jump.duration();
                    if past >= 0.0 {
                        let rest = jump.travelled_at(jump.duration()) - jump.travelled();
                        let c = resume.cycle + past * resume.rate;
                        body += resume.handover.dot(f) + rest + resume.speed * past;
                        hips.push(body + pose_at(c).root_translation.dot(f));
                        cycle = Some(c);
                        handed = hips.len() - 1;
                    } else {
                        let was = jump.travelled();
                        jump.advance(dt);
                        body += jump.travelled() - was;
                        hips.push(body + jump.pose_at(jump.elapsed(), &stood, &rig).root_translation.dot(f));
                    }
                }
                Some(c) => {
                    let c = c + resume.rate * dt;
                    body += resume.speed * dt;
                    hips.push(body + pose_at(c).root_translation.dot(f));
                    cycle = Some(c);
                }
            }
        }
        (hips, handed)
    }

    /// Taken from the run and handed back within the frames the feet come
    /// down and leave in, the hips go on as they came: each step at either
    /// end within 1 mm of the ones beside it (0.33 at worst), wherever in
    /// its frame the jump began. Ended with a frame posed at the toe-off
    /// instead, the hips moved 13 mm of 57 that frame live; read at the
    /// run's contact velocity rather than its speed, the hop's hand-back
    /// stepped 11 mm; with the COM planned forward on the landing foot, the
    /// run's legs kicked the hips 3 mm two frames before it.
    #[test]
    fn taken_from_the_run_and_handed_back_the_hips_go_on_as_they_came() {
        for (speed, ask) in LEAPS.into_iter().chain([(4.0, JumpAsk::running(0.1, 0.0))]) {
            for before in [0.002, 0.009, 0.015] {
                let (hips, at) = hips_through(ask, speed, before);
                assert!(at > 0, "{ask:?} never handed back");
                let step = |i: usize| hips[i] - hips[i - 1];
                // 2: the step into the jump's first frame against the run's
                // last; then the frames either side of handing back, and
                // the ones before it on the landing foot.
                for i in [2, at - 2, at - 1, at, at + 1] {
                    let jolt = (step(i) - step(i - 1)).abs();
                    assert!(jolt < 0.001, "{ask:?} at {speed} m/s, {before} s in: the hips stepped {:.1} then {:.1} mm", step(i - 1) * 1e3, step(i) * 1e3);
                }
            }
        }
    }

    /// How far the pose `t` seconds in is off its plan: on one foot the hips
    /// forward and the COM up, else the COM both ways.
    fn off_plan(jump: &Jump, t: f32, stood: &LocalPose, rig: &RigGeometry) -> f32 {
        let forward = rig.forward();
        let pose = jump.pose_at(t, stood, rig);
        let shift = forward * jump.travelled_at(t);
        let com = com_of(&pose, stood, rig) + shift;
        let ahead = match jump.run.as_ref().and_then(|run| run.hips_ahead_at(jump, t)) {
            Some(planned) => (hips_of(&pose, stood) + shift).dot(forward) - planned,
            None => com.dot(forward) - jump.com_ahead_at(t),
        };
        ahead.hypot(com.y - jump.com_height_at(t))
    }

    /// Each foot's heel, ball and tip `t` seconds in, in the frame the leap
    /// began in (the pose's frame travels with the character).
    fn soles_at(jump: &Jump, t: f32, stood: &LocalPose, rig: &RigGeometry) -> [[Vec3; 3]; 2] {
        let pose = jump.pose_at(t, stood, rig);
        let shift = hips_of(&pose, stood) + rig.forward() * jump.travelled_at(t);
        LEGS.map(|(_, _, ankle)| Sole::of(rig, ankle).points(&pose, rig).map(|p| p + shift))
    }

    /// The leap starts on the run's own pose at the take-off foot's contact
    /// and ends on its own pose at the landing foot's toe-off (the root
    /// moved by the hand-back), so the run hands over and back with nothing
    /// changing.
    #[test]
    fn a_leap_starts_and_ends_on_the_runs_own_pose() {
        let (stood, rig) = real_stood();
        for (speed, ask) in LEAPS {
            let jump = Jump::from_run(ask, RunStart { leg: 0, speed }, &stood, &rig);
            let resume = jump.resumes().unwrap();
            let ends = [
                (jump.pose_at(0.0, &stood, &rig), walk_pose_on(0.0, &GaitParams::running_on(speed, &rig), &stood, &rig), Vec3::ZERO),
                (
                    jump.pose_at(jump.duration(), &stood, &rig),
                    walk_pose_on(resume.cycle, &GaitParams::running_on(resume.speed, &rig), &stood, &rig),
                    resume.handover,
                ),
            ];
            for (end, (jumped, ran, moved)) in ends.iter().enumerate() {
                assert!((jumped.root_translation - moved - ran.root_translation).length() < 1.0e-3, "{ask:?}: end {end}'s root is off the run's");
                for bone in Bone::ALL {
                    assert!(1.0 - jumped.rotations[bone].dot(ran.rotations[bone]).abs() < 1.0e-4, "{ask:?}: end {end}'s {bone:?} is off the run's");
                }
            }
        }
    }

    /// The pose's COM keeps to the plan; in the air it falls at g; on a
    /// foot the floor never pulls, nor pushes past 6 body weights (a leap's
    /// take-off 5.4, running long jumps 4-10).
    #[test]
    fn a_leap_flies_at_g_and_the_floor_never_pulls() {
        let (stood, rig) = real_stood();
        let forward = rig.forward();
        for (speed, ask) in LEAPS {
            let jump = Jump::from_run(ask, RunStart { leg: 0, speed }, &stood, &rig);
            let com = |t: f32| com_of(&jump.pose_at(t, &stood, &rig), &stood, &rig) + forward * jump.travelled_at(t);
            let h = 1.0e-2;
            let mut t = h;
            while t < jump.duration() - h {
                let at = com(t);
                let off = off_plan(&jump, t, &stood, &rig);
                assert!(off < 1.0e-3, "{ask:?} at {t:.3} s the body is {:.1} mm off its plan", off * 1e3);
                let phase = jump.phase_at(t);
                if jump.phase_at(t - h) == phase && jump.phase_at(t + h) == phase {
                    let y = |t: f32| jump.com_height_at(t);
                    let load = 1.0 + (y(t + h) - 2.0 * y(t) + y(t - h)) / (h * h) / GRAVITY;
                    if phase == JumpPhase::Flight {
                        let fall = (com(t + h).y - 2.0 * at.y + com(t - h).y) / (h * h);
                        assert!((fall + GRAVITY).abs() < 0.1, "{ask:?} at {t:.3} s in the air the COM accelerates {fall}");
                    } else {
                        assert!((-0.02..6.0).contains(&load), "{ask:?} at {t:.3} s ({phase:?}) the floor carries {load} body weights");
                    }
                }
                t += h;
            }
            // And it really leaves: as high as asked, at 2.3-3.3 m/s along
            // the way it ran less what the plant leg braked.
            let apex = jump.com_height_at(jump.ends(JumpPhase::Push) + jump.up / GRAVITY) - jump.com_height_at(jump.ends(JumpPhase::Push));
            assert!((apex - ask.height).abs() < 2.0e-3, "{ask:?}: rose {apex} m");
            assert!((jump.speed() - (speed - LOSS_PER_UP * jump.up)).abs() < 1.0e-3, "{ask:?}: flew at {} m/s", jump.speed());
        }
    }

    /// A foot that is down stays where it came down (its points on the
    /// floor moving under 1.5 mm per 1/240 s as it rolls, against 18 mm
    /// carried at the run's speed), nothing goes through the floor (the
    /// run's own sole sits 1.8 mm into it), and both knees fold forward
    /// throughout.
    #[test]
    fn a_leaps_feet_stay_where_they_are_down_and_its_knees_fold_forward() {
        use crate::character::anim::rig::Side;
        let (stood, rig) = real_stood();
        for (speed, ask) in LEAPS {
            let jump = Jump::from_run(ask, RunStart { leg: 0, speed }, &stood, &rig);
            let run = jump.run.as_ref().unwrap();
            let floor = soles_at(&jump, 0.0, &stood, &rig).iter().flatten().map(|p| p.y).fold(f32::MAX, f32::min);
            let dt = 1.0 / 240.0;
            let mut last = soles_at(&jump, 0.0, &stood, &rig);
            let mut t = dt;
            while t <= jump.duration() {
                let feet = soles_at(&jump, t, &stood, &rig);
                let down = run.feet_down(&jump, t);
                for i in 0..2 {
                    for k in 0..3 {
                        assert!(feet[i][k].y > floor - 3.0e-3, "{ask:?} at {t:.3} s foot {i} is {:.1} mm in the floor", (floor - feet[i][k].y) * 1e3);
                        if down[i] && feet[i][k].y < floor + 0.003 && last[i][k].y < floor + 0.003 {
                            let moved = feet[i][k] - last[i][k];
                            let slid = Vec3::new(moved.x, 0.0, moved.z).length();
                            assert!(slid < 1.5e-3, "{ask:?} at {t:.3} s foot {i}'s point {k} slid {:.2} mm", slid * 1e3);
                        }
                    }
                }
                let pose = jump.pose_at(t, &stood, &rig);
                for side in [Side::Left, Side::Right] {
                    let fold = rig.knee_fold_direction(&pose, side);
                    assert!(fold < 0.0, "{ask:?} at {t:.3} s the {side:?} knee folds backward ({fold})");
                }
                last = feet;
                t += dt;
            }
        }
    }

    /// The free leg drives its thigh up leaving the floor, by as much as the
    /// leap's speed up asks: near level for a full leap, hardly for a hop.
    #[test]
    fn a_leaps_free_thigh_drives_up_as_hard_as_the_leap() {
        let (stood, rig) = real_stood();
        let thigh = |speed: f32, ask: JumpAsk| {
            let jump = Jump::from_run(ask, RunStart { leg: 0, speed }, &stood, &rig);
            let pose = jump.pose_at(jump.ends(JumpPhase::Push), &stood, &rig);
            let p = forward_kinematics_on(&pose, &rig);
            let along = p[Bone::RightLeg] - p[Bone::RightUpLeg];
            along.dot(rig.forward()).atan2(-along.y)
        };
        let (leap, hop) = (thigh(4.0, JumpAsk::running(0.3, 0.0)), thigh(3.0, JumpAsk::running(0.08, 0.0)));
        assert!(leap > 1.0, "a leap's free thigh is {:.0}° from vertical", leap.to_degrees());
        assert!(hop < leap - 0.3, "a hop's free thigh is {:.0}°, a leap's {:.0}°", hop.to_degrees(), leap.to_degrees());
    }

    /// Running jumps that land on both feet and stop.
    const STOPS: [(f32, JumpAsk); 2] = [(3.0, JumpAsk::forward(0.1, 0.0)), (4.0, JumpAsk::forward(0.3, 0.0))];

    /// Landing on both feet: it starts on the run's pose at the contact,
    /// keeps its COM on the plan with the floor never pulling (nor past 6
    /// body weights), holds the take-off foot and then both landed feet
    /// still where they are down, folds its knees forward and no deeper
    /// than 125°, carries the knee across touchdown without a jump, and
    /// ends standing over the landed feet (the root where the standing pose
    /// has it, less the settle the walker gives).
    #[test]
    fn a_running_jump_lands_on_both_feet_and_stands() {
        use crate::character::anim::rig::Side;
        let (stood, rig) = real_stood();
        for (speed, ask) in STOPS {
            let jump = Jump::from_run(ask, RunStart { leg: 0, speed }, &stood, &rig);
            assert!(jump.resumes().is_none(), "{ask:?} runs on");
            let start = walk_pose_on(0.0, &GaitParams::running_on(speed, &rig), &stood, &rig);
            let first = jump.pose_at(0.0, &stood, &rig);
            assert!((first.root_translation - start.root_translation).length() < 1.0e-3, "{ask:?} starts off the run's root");
            let last = jump.pose_at(jump.duration(), &stood, &rig);
            assert!((last.root_translation - jump.settle() - stood.root_translation).length() < 1.0e-3, "{ask:?} ends off the standing root");
            for bone in Bone::ALL {
                assert!(1.0 - first.rotations[bone].dot(start.rotations[bone]).abs() < 1.0e-4, "{ask:?}: {bone:?} starts off the run's");
                assert!(1.0 - last.rotations[bone].dot(stood.rotations[bone]).abs() < 1.0e-4, "{ask:?}: {bone:?} ends off standing");
            }
            let floor = soles_at(&jump, 0.0, &stood, &rig).iter().flatten().map(|p| p.y).fold(f32::MAX, f32::min);
            let landed = soles_at(&jump, jump.duration(), &stood, &rig);
            let flex = |t: f32| {
                let p = forward_kinematics_on(&jump.pose_at(t, &stood, &rig), &rig);
                180.0 - (p[Bone::LeftUpLeg] - p[Bone::LeftLeg]).angle_between(p[Bone::LeftFoot] - p[Bone::LeftLeg]).to_degrees()
            };
            let dt = 1.0 / 240.0;
            let mut last_feet = soles_at(&jump, 0.0, &stood, &rig);
            let mut t = dt;
            while t <= jump.duration() {
                let pose = jump.pose_at(t, &stood, &rig);
                let off = off_plan(&jump, t, &stood, &rig);
                assert!(off < 1.0e-3, "{ask:?} at {t:.3} s the body is {:.1} mm off its plan", off * 1e3);
                let phase = jump.phase_at(t);
                if phase != JumpPhase::Flight && jump.phase_at(t - dt) == phase && jump.phase_at(t + dt) == phase {
                    let y = |t: f32| jump.com_height_at(t);
                    let load = 1.0 + (y(t + dt) - 2.0 * y(t) + y(t - dt)) / (dt * dt) / GRAVITY;
                    assert!((-0.02..6.0).contains(&load), "{ask:?} at {t:.3} s ({phase:?}) the floor carries {load} body weights");
                    // Within a shoe's grip, landing: braked at 3.5 g across
                    // before `LANDING_SPEED`.
                    if matches!(phase, JumpPhase::Land | JumpPhase::Recover) {
                        let x = |t: f32| jump.com_ahead_at(t);
                        let across = (x(t + 0.01) - 2.0 * x(t) + x(t - 0.01)) / 1.0e-4;
                        assert!(across.abs() <= 0.8 * load * GRAVITY + 0.3, "{ask:?} at {t:.3} s ({phase:?}) the floor brakes {across} m/s² under {load} body weights");
                    }
                }
                let feet = soles_at(&jump, t, &stood, &rig);
                let down = jump.run.as_ref().unwrap().feet_down(&jump, t);
                for i in 0..2 {
                    for k in 0..3 {
                        assert!(feet[i][k].y > floor - 3.0e-3, "{ask:?} at {t:.3} s foot {i} is {:.1} mm in the floor", (floor - feet[i][k].y) * 1e3);
                        if down[i] && feet[i][k].y < floor + 0.003 && last_feet[i][k].y < floor + 0.003 {
                            let moved = feet[i][k] - last_feet[i][k];
                            let slid = Vec3::new(moved.x, 0.0, moved.z).length();
                            assert!(slid < 1.5e-3, "{ask:?} at {t:.3} s ({phase:?}) foot {i}'s point {k} slid {:.2} mm", slid * 1e3);
                        }
                    }
                    // Come to rest over their spots as they touch, not
                    // skidding onto them: the last 1/240 s in the air, under
                    // 2 mm across (carried with the body, 11 mm). The take-
                    // off foot swings 1.65 m from behind in 0.35 s, and is
                    // still moving 3.4 m/s 50 ms out.
                    if phase == JumpPhase::Flight && t + dt > jump.ends(JumpPhase::Flight) {
                        let moved = feet[i][2] - last_feet[i][2];
                        let across = Vec3::new(moved.x, 0.0, moved.z).length();
                        assert!(across < 2.0e-3, "{ask:?} at {t:.3} s foot {i} comes down moving {:.2} mm across", across * 1e3);
                    }
                    if matches!(phase, JumpPhase::Land | JumpPhase::Recover) {
                        let off = Vec3::new(feet[i][2].x - landed[i][2].x, 0.0, feet[i][2].z - landed[i][2].z).length();
                        assert!(off < 1.0e-3, "{ask:?} at {t:.3} s ({phase:?}) foot {i}'s tip is {:.1} mm off its spot", off * 1e3);
                    }
                }
                for side in [Side::Left, Side::Right] {
                    let fold = rig.knee_fold_direction(&pose, side);
                    assert!(fold < 0.0, "{ask:?} at {t:.3} s ({phase:?}) the {side:?} knee folds backward ({fold})");
                }
                assert!(flex(t) < 125.0, "{ask:?} at {t:.3} s the knee folds to {}°", flex(t));
                last_feet = feet;
                t += dt;
            }
            let touchdown = jump.ends(JumpPhase::Flight);
            let (meeting, landing) = (flex(touchdown - 1.0e-4), flex(touchdown + 1.0e-4));
            assert!((meeting - landing).abs() < 6.0, "{ask:?}: the knee meets the floor at {meeting}° and lands at {landing}°");
        }
    }

    /// Asked a distance toe to toe, it lands there if braking harder on the
    /// plant leg can get it there, never farther than the run carries it.
    #[test]
    fn a_leap_lands_as_far_as_asked_within_what_it_can_brake() {
        let (stood, rig) = real_stood();
        let natural = Jump::from_run(JumpAsk::running(0.2, 0.0), RunStart { leg: 0, speed: 4.0 }, &stood, &rig).distance();
        let shorter = Jump::from_run(JumpAsk::running(0.2, natural - 0.2), RunStart { leg: 0, speed: 4.0 }, &stood, &rig);
        assert!((shorter.distance() - (natural - 0.2)).abs() < 0.02, "asked {} m, landed {} m", natural - 0.2, shorter.distance());
        let farther = Jump::from_run(JumpAsk::running(0.2, natural + 1.0), RunStart { leg: 0, speed: 4.0 }, &stood, &rig);
        assert!(farther.speed() <= 4.0 + 1.0e-4 && farther.distance() < natural + 0.6, "asked {} m: flew at {} m/s, {} m", natural + 1.0, farther.speed(), farther.distance());
    }
}
