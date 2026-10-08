//! Running along a wall: step 8 of the parkour design
//! (`docs/knowledge/character-animation/parkour/`), third part.
//!
//! Running beside a wall, close to it and along it, it takes off from the
//! foot farther from it (a running leap, [`Jump::from_run`]) and steps
//! twice on the wall's face: the near foot, then the far one. Each foot is
//! held where it met the face while the body passes over it, and the
//! wall's pushes hold the flight up ([`Lift`]). Then it lands on the near
//! foot and runs on, as a leap does.
//!
//! No measured data (`parkour-movement-data`): the wall's pushes are
//! friction against the push into it, so they hold the body up a little,
//! not climb it. Two steps of 0.16 s each add 1.3 m/s up.
//!
//! - **Take-off**: the leap's, asked [`RISE`] up.
//! - **On the wall**: the body leant off the wall about the line of running
//!   (the hips rolled, the trunk back toward upright), each stepping foot
//!   on its hold, its sole to the face, its knee up and ahead; the COM the
//!   leap's, held up by the pushes.
//! - **Off it**: the lean let go by touchdown, the leap's own landing.

use bevy::math::{Quat, Vec3};

use super::geometry::Ledge;
use crate::character::anim::anthropometry::centre_of_mass;
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::{Jump, JumpAsk, JumpPhase, Lift, RunStart};
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::anim::stance::{knee_toward, place_ankle};
use crate::character::skeleton::Bone;

/// The run meets the wall at most this far off parallel, radians.
pub const MOST_SLANT: f32 = 0.3;
/// The hips this near the face and this far, at most, as it takes off,
/// metres.
pub const HIPS_OFF: (f32, f32) = (0.4, 0.75);
/// The leap asked this high, metres.
const RISE: f32 = 0.3;
/// The first step on the wall this long after take-off, seconds; each step
/// this long, seconds, the second this long after the first leaves.
const TO_WALL: f32 = 0.12;
const STEP_ON: f32 = 0.16;
const GAP: f32 = 0.06;
/// Each step's push up, m/s.
const GAINS: [f32; 2] = [0.7, 0.6];
/// Each step's ball on the face this far under the hips midway through it,
/// metres.
const DROP: [f32; 2] = [0.5, 0.45];
/// The foot's toes pitched up along the face, radians.
const TOES_UP: f32 = 0.3;
/// The hips rolled off the wall this far, radians, the trunk back this much
/// of it; rolled in over this long from take-off, back over this long by
/// touchdown, seconds.
const LEAN: f32 = 0.45;
const TRUNK_BACK: f32 = 0.5;
const ROLL_IN: f32 = 0.12;
const ROLL_OUT: f32 = 0.15;
/// The near foot comes onto its hold over this long, seconds, from no
/// earlier than the jump's start (the far foot from take-off on); each foot
/// comes off its hold over this long. In 0.12 s on and 0.15 s off, the near
/// toe went 14.5-17 m/s; in 0.25 s on, started in the take-off's stance on
/// the leap's own swing, still 14.5-17 m/s at 4-4.5 m/s.
const ONTO: f32 = 0.4;
const OFF: [f32; 2] = [0.25, 0.2];
/// A stepping knee kept this far off the face, metres, and its ball out of
/// it, by moving the ankle out in up to this many passes.
const KNEE_OFF: f32 = 0.02;
const KEEP_OFF_PASSES: usize = 4;
/// A stepping knee points ahead and this much up for each unit ahead:
/// pointed up, square to a leg reaching down to the face it leant toward
/// the face, within 2 cm of it; pointed up and out, nearly along the leg,
/// it swung round at 35 m/s.
const KNEE_UP: f32 = 0.3;
/// The holds this far in from the wall's ends at least, metres, and under
/// its top.
const END_MARGIN: f32 = 0.3;
const UNDER_TOP: f32 = 0.3;
/// The body posed onto its COM and its feet this many times over.
const PASSES: usize = 3;
/// A foot this far from its hold through its step is out of reach, metres.
const REACHED: f32 = 1.0e-3;

const LEGS: [(Bone, Bone, Bone); 2] = [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot)];

/// Eased 0-1 over `span` from `from`.
fn ease(t: f32, from: f32, span: f32) -> f32 {
    smoothstep(((t - from) / span.max(1.0e-6)).clamp(0.0, 1.0))
}

/// A pose's whole-body COM, in its frame.
fn com(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
    pose.root_translation + centre_of_mass(pose, rig)
}

/// One step on the wall: which leg, when (seconds into the jump, on and
/// off), its ankle on the hold (the jump's frame) and its world rotation
/// there.
#[derive(Debug, Clone, Copy)]
struct Step {
    leg: usize,
    on: f32,
    off: f32,
    ankle: Vec3,
    attitude: Quat,
    /// How far the jump has travelled as the foot meets the face, metres.
    travelled: f32,
}

/// A run along a wall's reshaping of its leap: the lean off the wall and the
/// steps on it.
#[derive(Debug, Clone)]
pub struct AlongWall {
    /// Out of the wall's face (the jump's frame, horizontal); a point `p`
    /// is `p·out - face` out of it.
    out: Vec3,
    face: f32,
    /// The flight, seconds into the jump: take-off to touchdown.
    flight: (f32, f32),
    steps: [Step; 2],
}

impl Jump {
    /// A run along `wall` (its face) by a walker running at `start.speed`,
    /// its root at `origin` turned `yaw` as the foot of `start.leg` came
    /// down: a running leap held up by two steps on the face, landing on the
    /// near foot and running on. `None` if the run meets the wall more than
    /// [`MOST_SLANT`] off parallel, takes off from the near foot, its hips
    /// not within [`HIPS_OFF`] of the face, the holds past the wall's ends
    /// or top, or out of the legs' reach.
    pub fn along_wall(wall: &Ledge, origin: Vec3, yaw: f32, start: RunStart, stood: &LocalPose, rig: &RigGeometry) -> Option<Self> {
        let back = Quat::from_rotation_y(yaw).inverse();
        let forward = rig.forward();
        // The wall in the jump's frame: a point's way out of the face is
        // `p·out - face`.
        let (out, along) = (back * wall.out, back * wall.along());
        let face = (wall.a - origin).dot(wall.out);
        let corner = back * (wall.a - origin);
        let out_of = |p: Vec3| p.dot(out) - face;
        if forward.dot(out).abs() > MOST_SLANT.sin() || start.leg != AlongWall::takeoff_leg(wall, yaw, rig) {
            return None;
        }
        let near = 1 - start.leg;
        let first = TO_WALL;
        let second = TO_WALL + STEP_ON + GAP;
        let lift = Lift { pushes: [(first, STEP_ON, GAINS[0]), (second, STEP_ON, GAINS[1])] };
        let mut jump = Jump::from_run(JumpAsk { lift: Some(lift), ..JumpAsk::running(RISE, 0.0) }, start, stood, rig);
        let push = jump.ends(JumpPhase::Push);
        let hips_at = |jump: &Jump, t: f32| forward_kinematics_on(&jump.pose_at_unshaped(t, stood, rig), rig)[Bone::Hips] + forward * jump.travelled_at(t);
        let off = out_of(hips_at(&jump, push));
        if !(HIPS_OFF.0..=HIPS_OFF.1).contains(&off) {
            return None;
        }
        let stood_at = forward_kinematics_on(stood, rig);
        let standing = accumulate_world_rotations(stood, rig);
        // The sole turned to the face, the toes pitched up along it.
        let to_face = Quat::from_rotation_arc(Vec3::NEG_Y, -out);
        let pitch = Quat::from_axis_angle(forward.cross(Vec3::Y).normalize_or(out), TOES_UP);
        let length = (wall.b - wall.a).length();
        let top = wall.height() - origin.y;
        let steps = [(near, first), (start.leg, second)].map(|(leg, at)| {
            let (on, off) = (push + at, push + at + STEP_ON);
            // The ankle under the hips midway through, so the leg sweeps
            // as far behind as ahead; the ball on the face (with the ball
            // under the hips, the ankle 0.13 m behind it, the foot came off
            // its hold leaving at 4 m/s).
            let ankle = LEGS[leg].2;
            let toe = crate::character::anim::foot::foot_bones(ankle).1;
            let attitude = pitch * to_face * standing[ankle];
            let from_ball = attitude * standing[ankle].inverse() * (stood_at[ankle] - stood_at[toe]);
            let hips = hips_at(&jump, 0.5 * (on + off));
            let ankle_at = hips - out * (out_of(hips) - from_ball.dot(out)) - Vec3::Y * DROP[(leg != near) as usize];
            (Step { leg, on, off, ankle: ankle_at, attitude, travelled: jump.travelled_at(on) }, ankle_at - from_ball)
        });
        // On the face, in from its ends and under its top.
        if steps.iter().any(|&(_, ball)| !(END_MARGIN..=length - END_MARGIN).contains(&(ball - corner).dot(along)) || ball.y > top - UNDER_TOP || ball.y < 0.0) {
            return None;
        }
        jump.set_along(AlongWall { out, face, flight: (push, jump.ends(JumpPhase::Flight)), steps: steps.map(|(step, _)| step) });
        // Each foot on its hold through its step: placed short of a hold out
        // of reach, it is not.
        let reached = steps.iter().all(|&(step, _)| {
            (0..=8).all(|k| {
                let t = step.on + (step.off - step.on) * k as f32 / 8.0;
                let at = forward_kinematics_on(&jump.pose_at(t, stood, rig), rig);
                (at[LEGS[step.leg].2] + forward * jump.travelled_at(t) - step.ankle).length() < REACHED
            })
        });
        reached.then_some(jump)
    }
}

impl AlongWall {
    /// The leg a run along `wall`, turned `yaw`, takes off from: the one
    /// farther from it, the near foot stepping on it first.
    pub fn takeoff_leg(wall: &Ledge, yaw: f32, rig: &RigGeometry) -> usize {
        let toward = Quat::from_rotation_y(yaw).inverse() * -wall.out;
        if rig.left().dot(toward) > 0.0 { 1 } else { 0 }
    }

    /// How far the body is leant off the wall `t` seconds into the jump,
    /// 0-1: in from take-off, out by touchdown once the last foot is off.
    fn leaning(&self, t: f32) -> f32 {
        let (from, to) = self.flight;
        if t <= from || t >= to {
            return 0.0;
        }
        let back = (to - ROLL_OUT).max(self.steps[1].off).min(to - 1.0e-3);
        ease(t, from, ROLL_IN).min(1.0 - ease(t, back, to - back))
    }

    /// How far step `k`'s foot is on its hold `t` seconds into the jump,
    /// 0-1: coming onto it, on it, off it. The take-off foot comes from
    /// take-off on, from behind the body: brought on over `ONTO` like the
    /// near foot, its toe went 18-22 m/s.
    fn stepping(&self, t: f32, k: usize) -> f32 {
        let step = &self.steps[k];
        let from = if k == 1 { self.flight.0 } else { (step.on - ONTO).max(0.0) };
        ease(t, from, step.on - from).min(1.0 - ease(t, step.off, OFF[k]))
    }

    /// The leap's pose `leap`, `t` seconds in, `travelled` along, reshaped:
    /// leant off the wall, the stepping feet on their holds, its COM the
    /// leap's.
    pub(crate) fn reshape(&self, leap: &LocalPose, t: f32, travelled: f32, rig: &RigGeometry) -> LocalPose {
        let lean = self.leaning(t);
        let steps = [0, 1].map(|k| self.stepping(t, k));
        if lean <= 0.0 && steps.iter().all(|&w| w <= 0.0) {
            return *leap;
        }
        let forward = rig.forward();
        let mut pose = *leap;
        if lean > 0.0 {
            // A turn carrying `+Y` out from the wall: the head away, the
            // feet toward it.
            let axis = Vec3::Y.cross(self.out).normalize_or(forward);
            pose.rotations[Bone::Hips] = delta_after_world_turn(&pose, rig, Bone::Hips, Quat::from_axis_angle(axis, LEAN * lean));
            pose.rotations[Bone::Spine] = delta_after_world_turn(&pose, rig, Bone::Spine, Quat::from_axis_angle(axis, -TRUNK_BACK * LEAN * lean));
        }
        let wanted = com(leap, rig);
        for _ in 0..PASSES {
            pose.root_translation += wanted - com(&pose, rig);
            for (k, &w) in steps.iter().enumerate() {
                if w > 0.0 {
                    self.place(&mut pose, &self.steps[k], w, t, travelled, rig);
                }
            }
        }
        pose
    }

    /// Step `step`'s foot brought `w` of the way onto its hold, `t` seconds
    /// into the jump; its ball kept out of the face (turning sole to it on
    /// the way, the far ball went 6.7 mm in).
    fn place(&self, pose: &mut LocalPose, step: &Step, w: f32, t: f32, travelled: f32, rig: &RigGeometry) {
        let forward = rig.forward();
        let (socket, knee, ankle) = LEGS[step.leg];
        let toe = crate::character::anim::foot::foot_bones(ankle).1;
        let at = forward_kinematics_on(pose, rig);
        // Coming on, toward where the hold will be about the body as the
        // foot meets it, eased into the hold held in the world by then: eased
        // toward the hold itself, running back past the body at the run's
        // speed, the near toe went 18 m/s.
        let held = if t < step.on { step.travelled + (travelled - step.travelled) * w * w } else { travelled };
        let hold = step.ankle - forward * held;
        let mut goal = at[ankle].lerp(hold, w);
        for _ in 0..KEEP_OFF_PASSES {
            place_ankle(pose, rig, ankle, goal - at[Bone::Hips]);
            knee_toward(pose, rig, [socket, knee, ankle], forward + Vec3::Y * KNEE_UP, w);
            let now = accumulate_world_rotations(pose, rig)[ankle];
            let turn = Quat::IDENTITY.slerp(step.attitude * now.inverse(), w);
            pose.rotations[ankle] = delta_after_world_turn(pose, rig, ankle, turn);
            // The ball's way into the face, and the knee's within `KNEE_OFF`
            // of it (leaving its hold at 0.45 m off, the near knee went 4 mm
            // in; turned out from the face instead, the knee swung round at
            // 35 m/s): the ankle moved out as far.
            let placed = forward_kinematics_on(pose, rig);
            let out_of = |bone: Bone| (placed[bone] + forward * travelled).dot(self.out) - self.face;
            let short = (-out_of(toe)).max(KNEE_OFF - out_of(knee));
            if short <= 1.0e-5 {
                break;
            }
            goal += self.out * short;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::jump::GRAVITY;
    use crate::character::anim::rig::BoneSet;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    const DT: f32 = 1.0 / 60.0;

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// A 3 m wall along the run, its face `off` metres to the side of the
    /// root as the foot comes down (`side` 1 on the left, -1 the right),
    /// reaching from 2 m behind to 6 m ahead.
    fn wall_beside(off: f32, side: f32, rig: &RigGeometry) -> Ledge {
        let (forward, left) = (rig.forward(), rig.left());
        Ledge::wall(left * (side * off) + forward * 2.0, left * -side, 8.0, 3.0, 1.0)
    }

    /// What a run along a wall measured.
    #[derive(Debug, Default)]
    struct Ran {
        hold_off: f32,
        into: f32,
        deepest: Option<(Bone, f32)>,
        fastest: f32,
        fastest_bone: Option<(Bone, f32)>,
        /// The leap's own fastest joint, unshaped.
        leap_fastest: f32,
        lowest_hold: f32,
        com_kink: f32,
        runs_on: bool,
    }

    fn ran_along(off: f32, side: f32, speed: f32, leg: Option<usize>) -> Option<Ran> {
        let (stood, rig) = real_stood();
        let wall = wall_beside(off, side, &rig);
        let leg = leg.unwrap_or_else(|| AlongWall::takeoff_leg(&wall, 0.0, &rig));
        let jump = Jump::along_wall(&wall, Vec3::ZERO, 0.0, RunStart { leg, speed }, &stood, &rig)?;
        let along = jump.running_along().expect("along a wall").clone();
        let forward = rig.forward();
        let mut m = Ran { lowest_hold: f32::MAX, runs_on: jump.resumes().is_some(), ..Default::default() };
        let world = |pose: LocalPose, t: f32| {
            let at = forward_kinematics_on(&pose, &rig);
            BoneSet::from_fn(|bone| at[bone] + forward * jump.travelled_at(t))
        };
        let end = jump.duration();
        let (mut last, mut leap_last): (Option<BoneSet<Vec3>>, Option<BoneSet<Vec3>>) = (None, None);
        let mut t = 0.0;
        while t <= end {
            let now = world(jump.pose_at(t, &stood, &rig), t);
            let leap = world(jump.pose_at_unshaped(t, &stood, &rig), t);
            if let Some(before) = leap_last.as_ref() {
                for bone in Bone::ALL {
                    m.leap_fastest = m.leap_fastest.max(((leap[bone] - leap[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT);
                }
            }
            leap_last = Some(leap);
            for step in &along.steps {
                if (step.on..=step.off).contains(&t) {
                    m.hold_off = m.hold_off.max((now[LEGS[step.leg].2] - step.ankle).length());
                }
            }
            for bone in Bone::ALL {
                let p = now[bone];
                let into = -wall.out_of(p);
                if into > m.into && p.y < wall.height() && (0.0..=8.0).contains(&(p - wall.a).dot(wall.along())) {
                    (m.into, m.deepest) = (into, Some((bone, t)));
                }
                if let Some(before) = last.as_ref() {
                    let speed = ((now[bone] - now[Bone::Hips]) - (before[bone] - before[Bone::Hips])).length() / DT;
                    if speed > m.fastest {
                        (m.fastest, m.fastest_bone) = (speed, Some((bone, t)));
                    }
                }
            }
            last = Some(now);
            t += DT;
        }
        for step in &along.steps {
            m.lowest_hold = m.lowest_hold.min(step.ankle.y);
        }
        // The planned COM's acceleration up through the flight: no step
        // from frame to frame bigger than a push's rise over a frame allows.
        let (push, flight) = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
        let y = |t: f32| jump.com_height_at(t);
        let accel = |t: f32| (y(t + DT) - 2.0 * y(t) + y(t - DT)) / (DT * DT);
        let mut t = push + DT;
        while t + 2.0 * DT < flight {
            m.com_kink = m.com_kink.max((accel(t + DT) - accel(t)).abs());
            t += DT;
        }
        Some(m)
    }

    /// Running at 3.5-4.5 m/s beside a wall 0.45-0.7 m off, on either side,
    /// it runs along it: each foot held on its hold, nothing into the wall,
    /// no joint whipping round, the holds up off the floor, the flight
    /// smooth, and it runs on.
    #[test]
    fn it_runs_along_a_wall_and_runs_on() {
        let mut faults = Vec::new();
        for side in [1.0, -1.0] {
            for speed in [3.5, 4.0, 4.5] {
                for off in [0.45, 0.55, 0.65] {
                    let name = format!("{} wall {off} m off, {speed} m/s", if side > 0.0 { "a left" } else { "a right" });
                    let Some(m) = ran_along(off, side, speed, None) else {
                        faults.push(format!("{name}: not run along"));
                        continue;
                    };
                    eprintln!("{name}: {m:?}");
                    if m.hold_off >= 1.0e-3 {
                        faults.push(format!("{name}: a foot strayed {:.4} m off its hold", m.hold_off));
                    }
                    if m.into >= 2.0e-3 {
                        faults.push(format!("{name}: {:?} went {:.4} m into the wall", m.deepest, m.into));
                    }
                    if m.fastest >= 14.0f32.max(m.leap_fastest + 0.01) {
                        faults.push(format!("{name}: {:?} went {:.1} m/s about the hips", m.fastest_bone, m.fastest));
                    }
                    if m.lowest_hold < 0.4 {
                        faults.push(format!("{name}: a hold only {:.2} m up", m.lowest_hold));
                    }
                    if m.com_kink > 0.5 * GRAVITY {
                        faults.push(format!("{name}: the COM's acceleration stepped {:.1} m/s² in a frame", m.com_kink));
                    }
                    if !m.runs_on {
                        faults.push(format!("{name}: it does not run on"));
                    }
                }
            }
        }
        assert!(faults.is_empty(), "{} faults:\n{}", faults.len(), faults.join("\n"));
    }

    /// Off the near foot, from too far off the wall or too near, at a wall
    /// met askew, or one too short to step on, it is not run along.
    #[test]
    fn a_wall_too_far_askew_or_short_or_off_the_wrong_foot_is_not_run_along() {
        let (stood, rig) = real_stood();
        let wall = wall_beside(0.55, 1.0, &rig);
        let leg = AlongWall::takeoff_leg(&wall, 0.0, &rig);
        let run = |wall: &Ledge, leg: usize, yaw: f32| Jump::along_wall(wall, Vec3::ZERO, yaw, RunStart { leg, speed: 4.0 }, &stood, &rig).is_some();
        assert!(run(&wall, leg, 0.0), "the run along the wall itself is taken");
        assert!(!run(&wall, 1 - leg, 0.0), "off the near foot");
        assert!(!run(&wall_beside(1.2, 1.0, &rig), leg, 0.0), "1.2 m off");
        assert!(!run(&wall_beside(0.1, 1.0, &rig), leg, 0.0), "0.1 m off");
        assert!(!run(&wall, leg, MOST_SLANT + 0.1), "met {} rad askew", MOST_SLANT + 0.1);
        let short = Ledge::wall(rig.left() * 0.55 + rig.forward() * 0.5, -rig.left(), 1.0, 3.0, 1.0);
        assert!(!run(&short, leg, 0.0), "a wall 1 m long");
    }
}
