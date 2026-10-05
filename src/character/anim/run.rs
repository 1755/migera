//! The run: measured running strides, played on a specific rig.
//!
//! The data is Fukuchi, Fukuchi & Duarte's public running set —
//! R. K. Fukuchi et al., "A public dataset of running biomechanics and the
//! effects of running speed on lower extremity kinematics and kinetics",
//! *PeerJ* 5:e3298 (2017) — runners on an instrumented treadmill at 2.5, 3.5
//! and 4.5 m/s. `tools/extract_running_strides.py` turns its raw markers and
//! treadmill force into one mean stride per speed:
//! `assets/anim/reference/fukuchi_running_strides.csv`, with the stride
//! timing in `fukuchi_running_summary.csv`. Embedded here, like the walk's
//! reference, so the data cannot drift from what the tests check.
//!
//! # Segment attitudes, as the walk
//!
//! The leg is driven by the thigh's attitude from vertical, the knee's
//! flexion and the foot's pitch from flat — the same zeros the measured walk
//! replays (`walk::WalkCycle::pose_legs`). Thigh and shank are along the
//! bones, from vertical: each marker cluster's change from the runner's own
//! standing trial, added to that bone's standing attitude, which leans back
//! 5.6 and 3.1 degrees (see `tools/extract_running_strides.py`). A hip angle
//! relative to the pelvis would carry the pelvis's 2-9 degree rocking onto
//! the leg.
//!
//! # Speed
//!
//! Between the three recorded speeds every curve, the stance share and the
//! pelvis are interpolated, each stride first retimed so its toe-off falls
//! where the interpolated stance share says; outside them, extrapolated
//! from the nearest two, as far as [`SLOWEST`] and [`FASTEST`]. A rig
//! proportioned unlike the runners runs at the same Froude number,
//! `speed² / (g · leg)` ([`reference_speed`]), as the walk does.
//!
//! # The pelvis
//!
//! Through stance it rides where the planted foot holds it — the leg's own
//! geometry is the spring-mass dip a runner's body makes, lowest at
//! mid-stance — and through flight it follows a cubic from the height and
//! rate the body left the ground with to those it lands with: a ballistic
//! arc, near enough, over the tenth of a second it lasts. The recorded
//! pelvis rocks forward and back twice a stride; the trunk above it leans
//! forward with speed (Thorstensson et al. 1984, [`trunk_lean`]).

use std::hash::{Hash, Hasher};
use std::sync::{Arc, LazyLock, Mutex};

use bevy::math::Quat;

use super::foot::{lowest, Sole};
use super::gait::{self, LegPhase};
use super::reference::Periodic;
use super::rig::{LocalPose, RigGeometry};
use super::stance::KNEE_AXIS;
use crate::character::skeleton::Bone;

/// The mean strides, 100 samples a stride from foot contact, per speed.
const STRIDES: &str = include_str!("../../../assets/anim/reference/fukuchi_running_strides.csv");
/// Per speed: subjects, strides, stride seconds, stance share, leg length.
const SUMMARY: &str = include_str!("../../../assets/anim/reference/fukuchi_running_summary.csv");

/// One recorded running speed's mean stride. Angles in radians, phase 0 at
/// the leg's foot contact.
#[derive(Debug, Clone, PartialEq)]
pub struct RunStride {
    /// The treadmill's speed, m/s.
    pub speed: f32,
    /// The thigh's attitude from vertical, forward (+), from standing.
    pub thigh: Periodic,
    /// Knee flexion (+), from standing.
    pub knee: Periodic,
    /// The foot's pitch, toe up (+), from standing (flat).
    pub foot: Periodic,
    /// The pelvis's anterior tilt (+), from standing.
    pub pelvis_tilt: Periodic,
    /// The pelvis's height about its mean, metres, on the runners' bodies.
    pub pelvis_bob: Periodic,
    /// The share of the stride the foot is down.
    pub duty: f32,
    /// One stride's duration, seconds.
    pub stride_seconds: f32,
    /// The runners' mean thigh plus shank, metres.
    pub leg_length: f32,
}

/// The recorded strides, slowest first.
pub static FUKUCHI: LazyLock<Vec<RunStride>> = LazyLock::new(|| {
    let parse = |line: &str| -> Vec<f32> {
        line.split(',').map(|f| f.trim().parse().unwrap_or_else(|_| panic!("bad field in {line:?}"))).collect()
    };
    let summary: Vec<Vec<f32>> = SUMMARY.lines().skip(1).filter(|l| !l.trim().is_empty()).map(parse).collect();
    let rows: Vec<Vec<f32>> = STRIDES.lines().skip(1).filter(|l| !l.trim().is_empty()).map(parse).collect();
    let radians = std::f32::consts::PI / 180.0;
    let mut strides: Vec<RunStride> = summary
        .iter()
        .map(|s| {
            let column = |c: usize, scale: f32| -> Vec<f32> {
                rows.iter().filter(|r| (r[0] - s[0]).abs() < 1.0e-3).map(|r| r[c] * scale).collect()
            };
            RunStride {
                speed: s[0],
                thigh: Periodic::fit(&column(2, radians)),
                knee: Periodic::fit(&column(3, radians)),
                foot: Periodic::fit(&column(4, radians)),
                pelvis_tilt: Periodic::fit(&column(5, radians)),
                pelvis_bob: Periodic::fit(&column(6, 1.0e-3)),
                duty: s[5],
                stride_seconds: s[3],
                leg_length: s[7],
            }
        })
        .collect();
    strides.sort_by(|a, b| a.speed.total_cmp(&b.speed));
    strides
});

/// The runners' mean leg (thigh plus shank), metres.
pub fn recorded_leg_length() -> f32 {
    let strides = &*FUKUCHI;
    strides.iter().map(|s| s.leg_length).sum::<f32>() / strides.len() as f32
}

/// The slowest recorded-runner speed a run is played at, m/s: half a metre
/// a second below the slowest recording, where a run meets the walk.
pub const SLOWEST: f32 = 2.0;
/// The fastest, m/s: the fastest recording. Faster, the stride holds and
/// only the cadence rises (`gait::LegCurves::Run`'s `pace`). Extrapolated
/// to 6 m/s instead, the legs asked the landing to come down 9 cm to keep
/// the flight ballistic (`FlightPlan`), and past 4.6 m/s it could not.
pub const FASTEST: f32 = 4.5;

/// The speed, m/s, a runner proportioned like the recorded ones runs alike
/// at, for a run at `speed` on legs `leg_length` long (Froude scaling, as
/// `gait::GaitParams::walking_for`), held within [`SLOWEST`]-[`FASTEST`].
pub fn reference_speed(speed: f32, leg_length: f32) -> f32 {
    recorded_speed(speed, leg_length).clamp(SLOWEST, FASTEST)
}

/// [`reference_speed`] before it is held to the recordings' range.
pub fn recorded_speed(speed: f32, leg_length: f32) -> f32 {
    speed * (recorded_leg_length() / leg_length.max(0.1)).sqrt()
}

/// The pair of recorded strides to interpolate between (or extrapolate
/// from) at reference speed `speed`, and how far from the first.
fn bracket(speed: f32) -> (&'static RunStride, &'static RunStride, f32) {
    let strides = &*FUKUCHI;
    let last = strides.len() - 1;
    let i = strides.iter().position(|s| s.speed > speed).unwrap_or(last + 1).clamp(1, last);
    let (a, b) = (&strides[i - 1], &strides[i]);
    (a, b, (speed - a.speed) / (b.speed - a.speed))
}

/// The share of the stride a foot is down at reference speed `speed`.
pub fn duty_at(speed: f32) -> f32 {
    let (a, b, t) = bracket(speed);
    (a.duty + (b.duty - a.duty) * t).clamp(0.2, 0.48)
}

/// A recorded stride's duration at reference speed `speed`, seconds.
pub fn stride_seconds_at(speed: f32) -> f32 {
    let (a, b, t) = bracket(speed);
    a.stride_seconds + (b.stride_seconds - a.stride_seconds) * t
}

/// Retimes a cycle position so a toe-off at `from` falls at `to`: the walk's
/// smooth warp (`gait::reference_phase`), `p + c·sin(2πp)/2π`.
fn retimed(p: f32, from: f32, to: f32) -> f32 {
    use std::f32::consts::TAU;
    let p = p.rem_euclid(1.0);
    let sine = (TAU * from).sin();
    if (to - from).abs() < 1.0e-6 || sine.abs() < 1.0e-3 {
        return p;
    }
    let c = (TAU * (to - from) / sine).clamp(-0.95, 0.95);
    p + c * (TAU * p).sin() / TAU
}

/// The recorded curves at reference speed `speed`, a leg `p` into its
/// cycle: `[thigh, knee, foot, pelvis_tilt]`, radians.
pub fn angles_at(speed: f32, p: f32) -> [f32; 4] {
    let (a, b, t) = bracket(speed);
    let duty = duty_at(speed);
    let of = |s: &RunStride| {
        let q = retimed(p, duty, s.duty);
        [s.thigh.at(q), s.knee.at(q), s.foot.at(q), s.pelvis_tilt.at(q)]
    };
    let (x, y) = (of(a), of(b));
    [0, 1, 2, 3].map(|i| x[i] + (y[i] - x[i]) * t)
}

/// The recorded pelvis's height about its mean at reference speed `speed`,
/// a leg `p` into its cycle, metres on the runners' bodies.
pub fn pelvis_at(speed: f32, p: f32) -> f32 {
    let (a, b, t) = bracket(speed);
    let duty = duty_at(speed);
    let (x, y) = (a.pelvis_bob.at(retimed(p, duty, a.duty)), b.pelvis_bob.at(retimed(p, duty, b.duty)));
    x + (y - x) * t
}

/// How far the recorded pelvis spans, top to bottom, at reference speed
/// `speed`, metres on the runners' bodies: each recording's span,
/// interpolated. (Extrapolated point by point, the shifted curves spanned
/// 13 % more at 2.2 m/s than either recording.)
pub fn recorded_span(speed: f32) -> f32 {
    let (a, b, t) = bracket(speed);
    let span = |s: &RunStride| {
        let heights: Vec<f32> = (0..100).map(|i| s.pelvis_bob.at(i as f32 / 100.0)).collect();
        heights.iter().copied().fold(f32::MIN, f32::max) - heights.iter().copied().fold(f32::MAX, f32::min)
    };
    span(a) + (span(b) - span(a)) * t
}

/// The mean of the recorded pelvis tilt at reference speed `speed`, radians.
fn mean_tilt(speed: f32) -> f32 {
    let (a, b, t) = bracket(speed);
    a.pelvis_tilt.mean() + (b.pelvis_tilt.mean() - a.pelvis_tilt.mean()) * t
}

/// How far forward a runner's trunk leans at reference speed `speed`,
/// radians: Thorstensson, Nilsson, Carlson & Zomlefer (1984, *Acta Physiol
/// Scand* 121:9-22), ten men running 2-6 m/s on a treadmill, the trunk's
/// mean forward inclination rising from about 6 to 13 degrees with speed,
/// taken here as linear over that range.
pub fn trunk_lean(speed: f32) -> f32 {
    let t = ((speed - 2.0) / 4.0).clamp(0.0, 1.0);
    (6.0 + 7.0 * t).to_radians()
}

/// A running step's width, of leg length: Arellano & Kram (2011, *J
/// Biomech* 44:1291-1295), preferred 3.95 % (about 3.6 cm). The walk's
/// is near four times that (`stance::STEP_WIDTH`).
pub const STEP_WIDTH: f32 = 0.0395;

/// How far a running leg's toes bend up at its cycle position `p` (0 at
/// contact), radians (`foot::toe_bend`).
///
/// Recorded feet pitch 44 degrees toe-down by toe-off at 2.5 m/s, the
/// runner's toes bent flat under them. A rig's toes held rigid reached 5 cm
/// below the ball there, and the pelvis rose 65 mm over standing to keep
/// them on the floor, its highest at toe-off where a runner's is mid-flight.
pub fn toe_bend(speed: f32, p: f32) -> f32 {
    super::foot::toe_bend(|q| angles_at(speed, q)[2], p, duty_at(speed))
}

/// Each leg: its phase offset in the cycle, and its hip, knee and ankle.
const LEGS: [(f32, Bone, Bone, Bone); 2] = [
    (0.0, Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
    (0.5, Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
];

/// The run on one rig, from one base pose: what does not depend on speed.
#[derive(Debug, Clone, PartialEq)]
pub struct RunCycle {
    /// Each leg's hip and foot rotations with the feet brought in to the
    /// run's step width.
    narrowed: [[Quat; 2]; 2],
    /// The bind's shank angle from vertical, per leg.
    bind_shank: [f32; 2],
    soles: [Sole; 2],
    /// Each foot's lowest contact under the base pose, hips-relative: the
    /// ground, when standing.
    ground: [f32; 2],
}

/// How high a running foot rises at least just after toe-off, metres, and
/// by how much of the swing it does.
const TOE_LIFT: f32 = 0.03;
const TOE_LIFT_BY: f32 = 0.12;

/// How far a running foot's sole keeps off the floor at `progress` through
/// its swing, at least, metres: off at once after toe-off, rising
/// `x(2 - x)` to [`TOE_LIFT`], and down to the walk's clearance and then
/// to the floor for the landing.
///
/// With no lift asked at the start of swing, the tip stayed on the floor
/// three frames after its lock let go and skimmed 2 cm a frame backward. A
/// smoothstep is flat at the floor too (see `transition::landing_lift`).
/// Kept on the pose (`RunCycle::clear_swing`) and again on the rendered foot
/// (`plugin::AnimFootIk::clear`): the legs' springs trail a foot pitching
/// ~1000 degrees a second at toe-off by ~20 degrees, and the rendered tip
/// stayed down five frames while the pose's was 3 cm up.
pub fn swing_clearance(progress: f32) -> f32 {
    if progress < 0.5 {
        let x = (progress / TOE_LIFT_BY).min(1.0);
        TOE_LIFT * x * (2.0 - x)
    } else {
        super::walk::TOE_CLEARANCE * gait::smoothstep((1.0 - progress) / 0.2)
    }
}

/// How far into stance the pelvis's take-off and landing rates are read,
/// of the cycle.
const RATE_SPAN: f32 = 0.002;

impl RunCycle {
    fn build(base: &LocalPose, rig: &RigGeometry) -> Self {
        let mut narrow = *base;
        super::stance::narrow_feet(&mut narrow, rig, STEP_WIDTH * gait::leg_length_of(rig));
        let narrowed = LEGS.map(|(_, hip, _, ankle)| [narrow.rotations[hip], narrow.rotations[ankle]]);
        let bind_shank = [0, 1].map(|leg| {
            let [thigh, _, knee, _] = gait::sagittal_angles(&LocalPose::REST, rig, gait::leg_joints(LEGS[leg].3));
            thigh - knee
        });
        let soles = [Sole::of(rig, LEGS[0].3), Sole::of(rig, LEGS[1].3)];
        let ground = [0, 1].map(|leg| lowest(&soles[leg].points(base, rig)));
        Self { narrowed, bind_shank, soles, ground }
    }

    /// The legs, pelvis tilt and trunk at `phase` and reference speed
    /// `speed`, composed onto `base`; the pelvis at its standing height.
    fn posed(&self, base: &LocalPose, rig: &RigGeometry, phase: f32, speed: f32, facing: f32, plan: Option<&FlightPlan>) -> LocalPose {
        let mut pose = *base;
        for (leg, &(_, hip, _, ankle)) in LEGS.iter().enumerate() {
            pose.rotations[hip] = self.narrowed[leg][0];
            pose.rotations[ankle] = self.narrowed[leg][1];
        }
        // The pelvis tilts forward as recorded (the left leg's cycle: both
        // legs' curves are the same pelvis), the trunk above it leaning by
        // the mean of the lean asked for less the tilt's own mean, so it
        // rocks with the pelvis about its lean. About the rig's left: a
        // positive turn carries what is above the joint forward.
        let tilt = angles_at(speed, phase)[3];
        let lean = trunk_lean(speed) - mean_tilt(speed);
        pose.rotations[Bone::Hips] =
            super::rig::delta_after_world_turn(&pose, rig, Bone::Hips, Quat::from_axis_angle(rig.left(), tilt));
        pose.rotations[Bone::Spine] =
            super::rig::delta_after_world_turn(&pose, rig, Bone::Spine, Quat::from_axis_angle(rig.left(), lean));

        for (leg, &(shift, hip, knee, ankle)) in LEGS.iter().enumerate() {
            let [thigh, knee_angle, toe_up] = leg_angles(speed, phase + shift);
            // The legs give a little where the flight needs them to
            // (`FlightPlan`), the foot keeping its attitude.
            let [thigh, knee_angle] = match plan {
                Some(plan) => {
                    let [d_thigh, d_knee] = plan.correction(speed, phase + shift);
                    [thigh + d_thigh, knee_angle + d_knee]
                }
                None => [thigh, knee_angle],
            };
            let shank = thigh - knee_angle;
            let wanted = [thigh, knee_angle, toe_up - shank + self.bind_shank[leg]];
            // Measured on the pose as tilted: the thigh's attitude is from
            // vertical, whatever the pelvis does.
            let [thigh_now, _, knee_now, ankle_now] = gait::sagittal_angles(&pose, rig, gait::leg_joints(ankle));
            // See `walk::WalkCycle::pose_legs` for the signs.
            let turn = |angle: f32| Quat::from_axis_angle(KNEE_AXIS, facing * angle);
            gait::compose(&mut pose, hip, turn(wanted[0] - thigh_now));
            gait::compose(&mut pose, knee, turn(-(wanted[1] - knee_now)));
            gait::compose(&mut pose, ankle, turn(wanted[2] - ankle_now));
            // The toes bend up about their joint, as the foot (pointing
            // forward) lifts its toe under a positive turn.
            let toes = super::foot::foot_bones(ankle).1;
            gait::compose(&mut pose, toes, turn(toe_bend(speed, phase + shift)));
        }
        pose
    }

    /// How far the pelvis rises from standing to put `leg`'s foot on the
    /// ground in `pose`.
    fn need(&self, pose: &LocalPose, rig: &RigGeometry, leg: usize) -> f32 {
        self.ground[leg] - lowest(&self.soles[leg].points(pose, rig))
    }

    /// The pelvis's rise at `phase` with `leg` down (stance), from its own
    /// posed legs.
    #[allow(clippy::too_many_arguments)]
    fn stance_rise(&self, base: &LocalPose, rig: &RigGeometry, phase: f32, speed: f32, facing: f32, leg: usize, plan: Option<&FlightPlan>) -> f32 {
        self.need(&self.posed(base, rig, phase, speed, facing, plan), rig, leg)
    }

    /// The running pose at `phase` and reference speed `speed`, its legs
    /// giving where the flight needs them to ([`FlightPlan`]).
    /// `pace` is how much faster than the stride's own speed it runs
    /// (`gait::LegCurves::Run`): the flight is shorter for it.
    pub fn pose(&self, base: &LocalPose, rig: &RigGeometry, phase: f32, speed: f32, pace: f32, facing: f32) -> LocalPose {
        let plan = self.plan(base, rig, speed, pace, facing);
        self.pose_with(base, rig, phase, speed, facing, Some(&plan))
    }

    /// The running pose with the recorded legs as they are: what the flight
    /// plan is worked out from.
    fn pose_with(&self, base: &LocalPose, rig: &RigGeometry, phase: f32, speed: f32, facing: f32, plan: Option<&FlightPlan>) -> LocalPose {
        let phase = phase.rem_euclid(1.0);
        let duty = duty_at(speed);
        let mut pose = self.posed(base, rig, phase, speed, facing, plan);
        let down = (0..2).find(|&leg| gait::leg_phase(phase + LEGS[leg].0, duty).is_stance());
        let rise = match down {
            Some(leg) => self.need(&pose, rig, leg),
            None => {
                // In flight: the leg that left, at its toe-off, and the
                // other, landing half a cycle after the first's contact.
                let half = phase.rem_euclid(0.5);
                let (left_at, lands_at) = (phase - (half - duty), phase + (0.5 - half));
                let off = if phase < 0.5 { 0 } else { 1 };
                let on = 1 - off;
                let at = |p: f32, leg: usize| self.stance_rise(base, rig, p, speed, facing, leg, plan);
                let (from, to) = (at(left_at, off), at(lands_at, on));
                let span = 0.5 - duty;
                // Rates per cycle, read just inside each stance.
                let leaving = (from - at(left_at - RATE_SPAN, off)) / RATE_SPAN;
                let arriving = (at(lands_at + RATE_SPAN, on) - to) / RATE_SPAN;
                let u = ((half - duty) / span).clamp(0.0, 1.0);
                gait::hermite(from, to, leaving * span, arriving * span, u)
            }
        };
        pose.root_translation.y += rise;
        for (leg, &(shift, ..)) in LEGS.iter().enumerate() {
            if let LegPhase::Swing { progress } = gait::leg_phase(phase + shift, duty) {
                self.clear_swing(&mut pose, base, rig, leg, progress, facing);
            }
        }
        pose
    }

    /// Bends a swinging leg's knee as far as it needs to keep its foot
    /// [`super::walk::TOE_CLEARANCE`] off the floor through mid-swing, none
    /// at either end, where the foot is on it.
    ///
    /// The recorded legs and the pelvis they hold disagree by about 4 cm:
    /// replayed on human proportions, the stance leg puts the hips 43-50 mm
    /// lower at contact than at toe-off, where the recording's pelvis
    /// markers are 3-8 mm lower — soft tissue moving under the thigh
    /// clusters, likely, since the pelvis's markers sit on bone. Imposing
    /// the recorded pelvis instead asked 4 % more leg than there is. So the
    /// legs lead, the pelvis drops ~5 cm through flight, and just before the
    /// next contact the trailing foot, pitched 70 degrees toe-down, dipped
    /// up to 26 mm through the floor at the slowest run. A runner's knee
    /// folds fast after toe-off; a few degrees more of it lifts the foot.
    fn clear_swing(&self, pose: &mut LocalPose, base: &LocalPose, rig: &RigGeometry, leg: usize, progress: f32, facing: f32) {
        const MOST: f32 = 0.3;
        let wanted = swing_clearance(progress);
        let risen = pose.root_translation.y - base.root_translation.y;
        let knee = LEGS[leg].2;
        let clearance = |pose: &LocalPose| risen + lowest(&self.soles[leg].points(pose, rig)) - self.ground[leg];
        let bend = |pose: &mut LocalPose, by: f32| gait::compose(pose, knee, Quat::from_axis_angle(KNEE_AXIS, facing * -by));
        for _ in 0..2 {
            let now = clearance(pose);
            // Short of the clearance, the corner at zero rounded over 4 mm
            // as `walk::clear_swinging_feet` does, so a clear foot is left
            // exactly alone.
            let short = {
                const ROUNDING: f32 = 0.004;
                let x = wanted - now;
                if x <= 0.0 {
                    0.0
                } else if x < ROUNDING {
                    x * x / (2.0 * ROUNDING)
                } else {
                    x - ROUNDING * 0.5
                }
            };
            if short <= 1.0e-5 {
                return;
            }
            let mut probe = *pose;
            bend(&mut probe, 0.02);
            let per = (clearance(&probe) - now) / 0.02;
            if per < 0.05 {
                return;
            }
            bend(pose, (short / per).min(MOST));
        }
    }
}

/// The recorded thigh, knee (kept off the reach singularity, as the walk's
/// is) and foot pitch at reference speed `speed`, `p` into a leg's cycle.
fn leg_angles(speed: f32, p: f32) -> [f32; 3] {
    let [thigh, knee, toe_up, _] = angles_at(speed, p);
    [thigh, gait::soft_floor(knee, gait::KNEE_FLOOR, gait::KNEE_FLOOR_SOFTNESS), toe_up]
}

/// How the legs give so the body flies as a body does: between toe-off and
/// the next contact the pelvis follows gravity exactly, leaving the ground
/// rising as the recorded pelvis does and dipping through stance as deep.
///
/// # Why the legs have to give
///
/// The legs replayed alone nearly agree with the recorded pelvis at the two
/// ends of stance (since the thigh and shank are from vertical, see
/// `tools/extract_running_strides.py`), but not quite on its rates: the
/// rig's toes stop bending at [`super::foot::TOE_BEND`], and the pelvis
/// they lift slows to half the recording's rise in the last 1 % of stance;
/// and its stance dip is shallower than the runners' by about a third.
///
/// So each stance leg shortens by `L(q)` (the foot keeping its place and
/// attitude, the pelvis coming down by as much), one smooth curve over the
/// stance, the two legs alike:
///
/// - **At contact** `c`, leaving at rate `σ`: the body keeps falling into
///   the stance at the rate it landed with.
/// - **At mid-stance** ([`DIP_AT`]) a dip `D`, where the knee is bent
///   most and gives the most height for its angle: deep enough that the
///   pelvis spans what the recording's does.
/// - **At toe-off** `e`, coming out at rate `τ`: the leg's push, so the
///   body leaves the ground rising at the recording's rate.
///
/// The toe-off state thrown ballistically lands on the contact's state,
/// height and rate: with the take-off rate chosen, `c` and `σ` follow in
/// closed form; `e` only where even a level take-off would land too high.
/// Alternatives measured (human proportions, Python): the recording's whole
/// pelvis imposed asks 22-26 degrees more knee in late stance; a
/// least-squares shortening weighted by the knee's cost finds the early
/// shape as a one-sample spike, ~7 degrees.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightPlan {
    duty: f32,
    /// Shortening at contact, metres, and its rate leaving it, per cycle.
    contact: f32,
    leaving: f32,
    /// Shortening at mid-stance, metres.
    dip: f32,
    /// Shortening at toe-off, metres, and the rate it comes out at, per
    /// cycle.
    end: f32,
    release: f32,
    /// The rig's thigh and shank, metres.
    thigh: f32,
    shank: f32,
    /// The legs' correction `[thigh, knee]` at toe-off and at contact, and
    /// their rates per cycle, for the swing between.
    toe_off: [f32; 2],
    toe_off_rate: [f32; 2],
    contact_change: [f32; 2],
    contact_rate: [f32; 2],
}

/// Where in stance the plan's dip is deepest, of the stance: about where
/// the recorded pelvis is lowest and the knee most bent.
const DIP_AT: f32 = 0.45;

/// The share of the swing over which a leg's correction eases out after
/// toe-off, and in before contact.
const SWING_EASE: f32 = 0.3;

/// The most a leg is shortened at contact, metres: past the recordings, at
/// 5.5 m/s, the plan asked 91 mm (38 degrees of knee).
const MOST_AT_CONTACT: f32 = 0.02;

/// The most a leg is shortened at mid-stance, metres.
const MOST_DIP: f32 = 0.03;

impl FlightPlan {
    /// How much the leg is shortened `q` cycles into its stance, metres:
    /// Hermite from contact to the dip, and from the dip to toe-off.
    fn shortening(&self, q: f32) -> f32 {
        let mid = DIP_AT * self.duty;
        let short = if q < mid {
            gait::hermite(self.contact, self.dip, self.leaving * mid, 0.0, q / mid)
        } else {
            let span = self.duty - mid;
            gait::hermite(self.dip, self.end, 0.0, -self.release * span, (q - mid) / span)
        };
        short.max(0.0)
    }

    /// The `[thigh, knee]` change that shortens a leg at `thigh` and `knee`
    /// (absolute, radians) by `short` metres, its ankle kept as far ahead
    /// of the hip: a two-link leg on the rig's thigh and shank.
    fn shortened(&self, [thigh, knee]: [f32; 2], short: f32) -> [f32; 2] {
        let (a, b) = (self.thigh, self.shank);
        let ahead = a * thigh.sin() + b * (thigh - knee).sin();
        let down = a * thigh.cos() + b * (thigh - knee).cos() - short;
        let bend = ((ahead * ahead + down * down - a * a - b * b) / (2.0 * a * b)).clamp(-1.0, 1.0).acos();
        let swung = ahead.atan2(down) + (b * bend.sin()).atan2(a + b * bend.cos());
        [swung - thigh, bend - knee]
    }

    /// The change to a leg `p` into its cycle at reference speed `speed`:
    /// through stance, what shortens it as planned; through swing, eased
    /// from toe-off's to contact's.
    fn correction(&self, speed: f32, p: f32) -> [f32; 2] {
        let p = p.rem_euclid(1.0);
        if p < self.duty {
            let [thigh, knee, _] = leg_angles(speed, p);
            self.shortened([thigh, knee], self.shortening(p))
        } else {
            // Eased out of toe-off's over the first part of the swing,
            // leaving at the rate the stance had, and into contact's over
            // the last, arriving still. One curve across the whole swing,
            // given the stance's rates at both ends (a knee folding fast
            // after a landing, ~10 rad a cycle), swung 40 degrees out.
            //
            // Not the push's rate, though: a leg straightening into its
            // toe-off folds at once after it, and carried on, the knee
            // straightened on into the swing and left the foot 4.9 mm off
            // the floor at 2.2 m/s.
            let span = 1.0 - self.duty;
            let u = (p - self.duty) / span;
            let out = (u / SWING_EASE).min(1.0);
            let into = ((u - (1.0 - SWING_EASE)) / SWING_EASE).max(0.0);
            [0, 1].map(|i| {
                let rate = if self.release > 0.0 { 0.0 } else { self.toe_off_rate[i] };
                gait::hermite(self.toe_off[i], 0.0, rate * span * SWING_EASE, 0.0, out)
                    + self.contact_change[i] * gait::smoothstep(into)
            })
        }
    }

    /// The plan for a run at reference speed `speed`, one stride taking
    /// `cycle_seconds`, from the recorded legs' own pelvis heights: `need`
    /// at a phase of a leg's cycle (its foot down) on the uncorrected pose.
    fn solve(speed: f32, cycle_seconds: f32, rig: &RigGeometry, need: impl Fn(f32) -> f32) -> Self {
        // The span the pose reads the pelvis's take-off and landing rates
        // over, so the plan and the flight agree.
        const STEP: f32 = RATE_SPAN;
        let duty = duty_at(speed);
        let (thigh, shank) = (rig.offsets[Bone::LeftLeg].length(), rig.offsets[Bone::LeftFoot].length());
        // Gravity in metres per cycle², and the flight's length in cycles.
        let gravity = 9.81 * cycle_seconds * cycle_seconds;
        let flight = 0.5 - duty;
        let (at_contact, at_toe_off) = (need(0.0), need(duty));
        let (contact_rate, toe_off_rate) = ((need(STEP) - at_contact) / STEP, (at_toe_off - need(duty - STEP)) / STEP);
        // The recorded pelvis, on this rig's leg (Froude: heights, and
        // rates per cycle, scale with the leg): its take-off rate, read over
        // a span its smooth fit resolves, and how far it spans.
        let scale = gait::leg_length_of(rig) / recorded_leg_length();
        const WIDE: f32 = 0.01;
        let wanted_rate = (pelvis_at(speed, duty) - pelvis_at(speed, duty - WIDE)) / WIDE * scale;
        let wanted_span = recorded_span(speed) * scale;

        // Landing height: the toe-off state thrown lands on the contact's,
        // `c = k - v·flight` for a take-off at `v` with nothing taken at
        // toe-off. The recording's take-off rate, as far as `c` stays within
        // 0 and its most; never slower than the legs' own.
        // Nor faster than a dip within its most can push: the curve from
        // the dip to toe-off overshoots past a slope of three times their
        // difference over the span.
        let mid = DIP_AT * duty;
        let k = at_contact - at_toe_off + 0.5 * gravity * flight * flight;
        let take_off = wanted_rate
            .clamp((k - MOST_AT_CONTACT) / flight, k / flight)
            .min(toe_off_rate + 3.0 * MOST_DIP / (duty - mid))
            .max(toe_off_rate);
        let release = take_off - toe_off_rate;
        let mut contact = k - take_off * flight;
        // Landing too high even so: taken at toe-off instead.
        let end = (-contact).max(0.0);
        contact = contact.clamp(0.0, MOST_AT_CONTACT);
        // Landing rate: the take-off rate less gravity's over the flight.
        let leaving = (contact_rate - take_off + gravity * flight).max(0.0);

        // The dip: as deep as makes the pelvis span the recording's, from
        // the flight's apex to the stance's lowest; at least deep enough
        // that the curve from it does not dip under toe-off's on its way.
        let least = end + release * (duty - mid) / 3.0;
        const SAMPLES: usize = 20;
        let raw: Vec<f32> = (0..=SAMPLES).map(|i| need(duty * i as f32 / SAMPLES as f32)).collect();
        let apex = at_toe_off - end + if take_off > 0.0 { take_off * take_off / (2.0 * gravity) } else { 0.0 };
        let span_with = |dip: f32| {
            let shape = Self { dip, ..Self::flat(duty, thigh, shank) };
            let shape = Self { contact, leaving, end, release, ..shape };
            let low = raw.iter().enumerate().map(|(i, r)| r - shape.shortening(duty * i as f32 / SAMPLES as f32)).fold(f32::MAX, f32::min);
            apex - low
        };
        let dip = if span_with(least) >= wanted_span || least >= MOST_DIP {
            least
        } else if span_with(MOST_DIP) <= wanted_span {
            MOST_DIP
        } else {
            let (mut low, mut high) = (least, MOST_DIP);
            for _ in 0..24 {
                let middle = 0.5 * (low + high);
                if span_with(middle) < wanted_span {
                    low = middle;
                } else {
                    high = middle;
                }
            }
            0.5 * (low + high)
        };
        let mut plan = Self { contact, leaving, dip, end, release, ..Self::flat(duty, thigh, shank) };
        let change = |p: f32| plan.correction(speed, p);
        let (end, before_end, start, after_start) = (change(duty - 1.0e-4), change(duty - STEP), change(0.0), change(STEP));
        plan.toe_off = end;
        plan.toe_off_rate = [0, 1].map(|i| (end[i] - before_end[i]) / (STEP - 1.0e-4));
        plan.contact_change = start;
        plan.contact_rate = [0, 1].map(|i| (after_start[i] - start[i]) / STEP);
        plan
    }

    /// A plan that shortens nothing, for a rig's `thigh` and `shank`.
    fn flat(duty: f32, thigh: f32, shank: f32) -> Self {
        Self {
            duty,
            contact: 0.0,
            leaving: 0.0,
            dip: 0.0,
            end: 0.0,
            release: 0.0,
            thigh,
            shank,
            toe_off: [0.0; 2],
            toe_off_rate: [0.0; 2],
            contact_change: [0.0; 2],
            contact_rate: [0.0; 2],
        }
    }
}

impl FlightPlan {
    /// `self` and `other` mixed, `t` of the way to `other`.
    fn blend(&self, other: &Self, t: f32) -> Self {
        let mix = |a: f32, b: f32| a + (b - a) * t;
        let pair = |a: [f32; 2], b: [f32; 2]| [mix(a[0], b[0]), mix(a[1], b[1])];
        Self {
            duty: mix(self.duty, other.duty),
            contact: mix(self.contact, other.contact),
            leaving: mix(self.leaving, other.leaving),
            dip: mix(self.dip, other.dip),
            end: mix(self.end, other.end),
            release: mix(self.release, other.release),
            thigh: mix(self.thigh, other.thigh),
            shank: mix(self.shank, other.shank),
            toe_off: pair(self.toe_off, other.toe_off),
            toe_off_rate: pair(self.toe_off_rate, other.toe_off_rate),
            contact_change: pair(self.contact_change, other.contact_change),
            contact_rate: pair(self.contact_rate, other.contact_rate),
        }
    }
}

/// Refinements of a flight plan on the corrected legs' own heights.
const PLAN_PASSES: usize = 6;

/// The grid flight plans are worked out on, in reference speed (m/s) and
/// in pace, a plan between blended from its neighbours: worked out at every
/// speed a run speeding up passes, one cost 1.5 ms a frame.
const PLAN_SPEED_STEP: f32 = 0.1;
const PLAN_PACE_STEP: f32 = 0.05;

/// Flight plans recently worked out, keyed by cycle, speed and pace.
static PLANS: Mutex<Vec<(u64, u32, u32, FlightPlan)>> = Mutex::new(Vec::new());

/// How many flight plans to keep: the grid over the recordings' speeds,
/// and the paces past them, for a few rigs.
const PLANS_KEPT: usize = 128;

impl RunCycle {
    /// The flight plan at reference speed `speed` and `pace`
    /// ([`FlightPlan`]): blended from those worked out on the grid around
    /// it ([`PLAN_SPEED_STEP`], [`PLAN_PACE_STEP`]).
    fn plan(&self, base: &LocalPose, rig: &RigGeometry, speed: f32, pace: f32, facing: f32) -> FlightPlan {
        let grid = |x: f32, step: f32| {
            let at = x / step;
            let below = if (at - at.round()).abs() < 1.0e-3 { at.round() } else { at.floor() };
            (below * step, (below + 1.0) * step, (at - below).max(0.0))
        };
        let (slower, faster, by_speed) = grid(speed, PLAN_SPEED_STEP);
        let (shorter, longer, by_pace) = grid(pace, PLAN_PACE_STEP);
        let row = |pace: f32| {
            let at = self.plan_at(base, rig, slower, pace, facing);
            if by_speed < 1.0e-3 { at } else { at.blend(&self.plan_at(base, rig, faster, pace, facing), by_speed) }
        };
        let at = row(shorter);
        if by_pace < 1.0e-3 { at } else { at.blend(&row(longer), by_pace) }
    }

    /// The flight plan worked out at reference speed `speed` and `pace`,
    /// once: on the uncorrected pose, a stride's duration from the stride
    /// the run takes (`distance_per_cycle`, which only the horizontal
    /// travel decides, untouched by the plan) at the speed it really goes.
    fn plan_at(&self, base: &LocalPose, rig: &RigGeometry, speed: f32, pace: f32, facing: f32) -> FlightPlan {
        let key = key_of(base, rig);
        if let Ok(plans) = PLANS.lock()
            && let Some((.., plan)) = plans.iter().find(|(k, s, p, _)| *k == key && *s == speed.to_bits() && *p == pace.to_bits())
        {
            return *plan;
        }
        let leg = gait::leg_length_of(rig);
        let real_speed = speed * pace * (leg / recorded_leg_length()).sqrt();
        let cycle_seconds = distance_per_cycle(base, rig, speed) / real_speed.max(0.1);
        let need = |p: f32, plan: Option<&FlightPlan>| self.need(&self.posed(base, rig, p, speed, facing, plan), rig, 0);
        let mut plan = FlightPlan::solve(speed, cycle_seconds, rig, |p| need(p, None));
        // The two-link shortening is the rig's leg only near enough (the
        // feet narrowed, the sole rolling): solved again on the heights the
        // corrected legs really ask, less what the plan meant them to give.
        for _ in 0..PLAN_PASSES {
            let previous = plan;
            plan = FlightPlan::solve(speed, cycle_seconds, rig, |p| need(p, Some(&previous)) + previous.shortening(p));
        }
        if let Ok(mut plans) = PLANS.lock() {
            plans.insert(0, (key, speed.to_bits(), pace.to_bits(), plan));
            plans.truncate(PLANS_KEPT);
        }
        plan
    }
}

/// Recently built cycles, most recent first.
static CYCLES: Mutex<Vec<(u64, Arc<RunCycle>)>> = Mutex::new(Vec::new());

/// How many distinct cycles to keep: a handful of rigs and base poses.
const CACHED: usize = 8;

/// The run cycle for this base pose and rig, built on first use.
pub fn run_cycle(base: &LocalPose, rig: &RigGeometry) -> Arc<RunCycle> {
    let key = key_of(base, rig);
    if let Ok(mut cache) = CYCLES.lock()
        && let Some(found) = cache.iter().position(|(k, _)| *k == key)
    {
        let entry = cache.remove(found);
        let cycle = entry.1.clone();
        cache.insert(0, entry);
        return cycle;
    }
    let cycle = Arc::new(RunCycle::build(base, rig));
    if let Ok(mut cache) = CYCLES.lock() {
        cache.insert(0, (key, cycle.clone()));
        cache.truncate(CACHED);
    }
    cycle
}

/// Everything a cycle depends on, hashed bit for bit.
fn key_of(base: &LocalPose, rig: &RigGeometry) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    let mut feed = |values: &[f32]| values.iter().for_each(|v| v.to_bits().hash(&mut hasher));
    for q in base.rotations.0.iter().chain(rig.bind_rotations.0.iter()) {
        feed(&q.to_array());
    }
    feed(&rig.root_rotation.to_array());
    feed(&base.root_translation.to_array());
    for v in rig.offsets.0.iter().chain(rig.toe_end_offsets.0.iter()) {
        feed(&v.to_array());
    }
    hasher.finish()
}

/// The Froude number, `speed² / (g · leg)`, at which people change from a
/// walk to a run: about 0.5, whatever their legs' length (Kram, Domingo &
/// Ferris 1997, *J Exp Biol* 200:821-826). 2.09 m/s on `puppet_base`'s
/// 0.888 m leg (thigh plus shank).
pub const CHANGEOVER_FROUDE: f32 = 0.5;

/// A run drops back to a walk below this share of the changeover speed, so
/// a speed held at the changeover does not flicker between the two.
pub const BACK_TO_WALK: f32 = 0.9;

/// How fast a walker gathers speed above a walk, and sheds it, m/s².
/// Authored: a jogger's easy pick-up and a firm but unhurried stop (a
/// sprinter's start is several times harder). A stride cannot change from a
/// walk to a fast run in one step, so an asked speed above the walk is
/// eased toward, and coming down, the run slows to a walk before it stops.
pub const ACCELERATION: f32 = 2.0;
pub const DECELERATION: f32 = 3.0;

/// The speed, m/s, at which legs `leg_length` long change from a walk to a
/// run ([`CHANGEOVER_FROUDE`]).
pub fn changeover_speed(leg_length: f32) -> f32 {
    (CHANGEOVER_FROUDE * 9.81 * leg_length.max(0.1)).sqrt()
}

/// The speed to hand the gait this frame, m/s, from the speed `asked` and
/// last frame's `pace`, `running` how far the gait is a run
/// ([`Gaits::running`]).
///
/// Starting, up to `changeover` at once (a start eases in itself); once
/// walking, gathered at [`ACCELERATION`]. Coming down, shed at [`DECELERATION`],
/// held at the changeover's back-to-walk speed until the gait is a walk
/// again, and on down to a slower walk the same way; a stop (asked nothing)
/// is the walk's own at once. Handed the slower walk's speed at once, the
/// walk's pose, which its speed sets, dropped the hips 37 mm in a frame
/// just after a run became a walk. Not past the changeover until the walk
/// is fully in (`walking`): gathering through the start, a walk's first step
/// was taken at 3-4 m/s and a foot slid 44 mm.
///
/// A walk with both feet down (`both_down`) holds its speed. Its stride and
/// stance share follow its speed, so a change moves its feet apart or
/// together, and root motion can keep only one of them where it stands:
/// slowing, the trailing foot rolling onto its toe slid up to 18 mm a frame
/// as it left; live, its toe tip slid 28 mm slowing from 1.2 to 0.6 m/s,
/// and 10-18 mm slowing into a walk after a run.
pub fn paced(asked: f32, pace: f32, running: f32, changeover: f32, walking: bool, both_down: bool, dt: f32) -> f32 {
    if walking && both_down && running <= 0.0 && asked > 0.0 {
        return pace;
    }
    if !walking && asked > changeover && running <= 0.0 {
        return changeover.max(pace.min(asked));
    }
    if asked >= pace {
        if walking {
            // Gathered from the walk's own pace: jumped to the changeover,
            // a walk at 1.2 m/s dropped its hips 23 mm in a frame.
            asked.min(pace + ACCELERATION * dt)
        } else if asked <= changeover && running <= 0.0 {
            asked
        } else {
            asked.min(pace.max(changeover.min(asked)) + ACCELERATION * dt)
        }
    } else if asked <= 0.0 && pace <= changeover && running <= 0.0 {
        asked
    } else if running > 0.0 {
        // Still a run: no slower than where it turns back into a walk,
        // until it has. Shed further, it ran on at 0.2 m/s for 1.4 s: the
        // change waits for a step's stretch, and the clock had all but
        // stopped by the time the next one came round.
        asked.max(pace - DECELERATION * dt).max(changeover * BACK_TO_WALK)
    } else {
        asked.max(pace - DECELERATION * dt)
    }
}

/// Whether a walker walks or runs, and the one step that changes between
/// them.
///
/// The change takes one step, on the walk's single support from the other
/// foot's toe-off to the run's own toe-off: the one stretch where both
/// gaits have the same one foot down, so blending them there never slides a
/// foot (the lesson of `transition`: a blend weight changing with two feet
/// down slips one). Speeding up, people change gait in one transition step
/// (Segers, Aerts, Lenoir & De Clercq 2006, *Gait Posture* 24:247-254);
/// faded over longer, the walk's double support and the run's flight would
/// disagree about which feet are down.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Gaits {
    /// How far the gait is a run: 0 a walk, 1 a run, between through the
    /// change.
    pub running: f32,
    /// A change under way, from this weight (0 or 1).
    change: Option<f32>,
    /// How the two gaits' phases line up for the change asked, worked out
    /// once it is asked ([`Matched`]).
    matched: Option<Matched>,
    /// The change's stretch, in the clock's half-cycle.
    window: (f32, f32),
}

/// Where the walk and the run each have the planted foot at the same place
/// under the hips: a table of the walk's phases into a step (half a cycle)
/// and the run's matching each. A straight line through one match drifted
/// 90 mm apart by the end of the walk's stance, and the toe went at 0.68 of
/// the body's pace there.
///
/// At one clock the two do not: the run is further into its stance, its
/// planted foot ~160 mm further back under the hips at 1.9 m/s. Blended at
/// one clock over the change's 16 frames, the foot moved ~10 mm a frame
/// under the hips that neither gait moves it: the walk's root motion, read
/// off that foot, braked the body from 2.0 to 0.8 m/s (and a run's would
/// have slid the foot). So through a change the gait coming in is posed at
/// the phase matching the gait going out, and the clock moves onto it as
/// the change ends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matched {
    /// The walk's phases into a step, and the run's matching each: where
    /// each has the planted toe as far ahead of the hips. `None`: the two
    /// clocks as one.
    table: Option<[(f32, f32); MATCHED]>,
}

/// How many phases a [`Matched`] table holds, over the walk's stance.
const MATCHED: usize = 13;

impl Matched {
    /// The two clocks as one: the change as it was before matching.
    pub const SAME: Self = Self { table: None };

    /// Through the table, the walk's phase to the run's (`to_run`) or back,
    /// linear between its entries and on past its ends.
    fn map(&self, x: f32, to_run: bool) -> f32 {
        let Some(table) = &self.table else { return x };
        let at = |i: usize| if to_run { table[i] } else { (table[i].1, table[i].0) };
        let i = (0..MATCHED - 2).find(|&i| x < at(i + 1).0).unwrap_or(MATCHED - 2);
        let ((x0, y0), (x1, y1)) = (at(i), at(i + 1));
        y0 + (y1 - y0) * (x - x0) / (x1 - x0).max(1.0e-5)
    }

    /// The run's phase into a step at the walk's `p`.
    fn run_of(&self, p: f32) -> f32 {
        self.map(p, true)
    }

    /// The walk's phase into a step at the run's `q`.
    fn walk_of(&self, q: f32) -> f32 {
        self.map(q, false)
    }

    /// Matches `walk` and `run` (poses at a cycle) on `rig`, whose stance
    /// shares are `walk_duty` and `run_duty`, by the right foot's toe joint
    /// ahead of the hips: sampled over each gait's stance on it (from 0.5),
    /// each walk phase paired with the run phase putting the toe as far
    /// ahead. The toe goes back under the hips all stance in both, so each
    /// is one-to-one.
    pub fn feet(walk: impl Fn(f32) -> LocalPose, run: impl Fn(f32) -> LocalPose, walk_duty: f32, run_duty: f32, rig: &RigGeometry) -> Self {
        let ahead = |pose: LocalPose| super::rig::offset_from(&pose, rig, Bone::Hips, Bone::RightToeBase).dot(rig.forward());
        let sample = |f: &dyn Fn(f32) -> LocalPose, duty: f32| -> [(f32, f32); MATCHED] {
            std::array::from_fn(|i| {
                let p = duty * i as f32 / (MATCHED - 1) as f32;
                (p, ahead(f(0.5 + p)))
            })
        };
        let (walked, ran) = (sample(&walk, walk_duty), sample(&run, run_duty));
        let falling = |s: &[(f32, f32); MATCHED]| s.windows(2).all(|w| w[1].1 < w[0].1);
        if !falling(&walked) || !falling(&ran) {
            return Self::SAME;
        }
        // The run phase at each walk sample, by the run's own samples
        // (linear between them, on past the ends).
        let table = walked.map(|(p, toe)| {
            let i = (0..MATCHED - 2).find(|&i| toe > ran[i + 1].1).unwrap_or(MATCHED - 2);
            let ((q0, t0), (q1, t1)) = (ran[i], ran[i + 1]);
            (p, q0 + (q1 - q0) * (toe - t0) / (t1 - t0))
        });
        Self { table: Some(table) }
    }
}

impl Gaits {
    /// Back to a walk at once: stopping, sitting, shuffling or fallen.
    pub fn walk(&mut self) {
        *self = Self::default();
    }

    /// Whether a change is under way.
    pub fn changing(&self) -> bool {
        self.change.is_some()
    }

    /// Whether a change under way is from the run.
    pub fn from_run(&self) -> bool {
        self.change.is_some_and(|from| from >= 1.0)
    }

    /// The walk's and the run's cycle at the clock's `cycle`: the clock's
    /// own for the gait going out, matched for the one coming in.
    pub fn phases(&self, cycle: f32) -> (f32, f32) {
        let (Some(from), Some(matched)) = (self.change, self.matched) else {
            return (cycle, cycle);
        };
        let half = cycle.rem_euclid(0.5);
        let base = cycle - half;
        if from >= 1.0 { (base + matched.walk_of(half), cycle) } else { (cycle, base + matched.run_of(half)) }
    }

    /// Advances to this frame: legs stepping at `speed`, which change gait
    /// at `changeover` (and back below [`BACK_TO_WALK`] of it), the walk's
    /// stance share `walk_duty` and the run's `run_duty`, the gait clock
    /// at `cycle`, last frame at `previous`. `feet` matches the two gaits'
    /// phases ([`Matched::feet`]), asked once a change is wanted.
    ///
    /// Returns where the clock goes when a change ends: onto the phase of
    /// the gait come in.
    #[allow(clippy::too_many_arguments)]
    pub fn advance(&mut self, speed: f32, changeover: f32, walk_duty: f32, run_duty: f32, cycle: f32, previous: f32, feet: impl FnOnce() -> Matched) -> Option<f32> {
        let goal = if speed >= changeover {
            1.0
        } else if speed <= changeover * BACK_TO_WALK {
            0.0
        } else {
            self.change.map_or(self.running, |from| 1.0 - from)
        };
        let (half, before) = (cycle.rem_euclid(0.5), previous.rem_euclid(0.5));
        if self.change.is_none() {
            if goal == self.running {
                self.matched = None;
                return None;
            }
            // Each step's stretch: from the walk's other foot's toe-off to
            // this foot's toe-off in the run, both in single support, a
            // little short of the walk's next landing and the run's toe-off.
            const SHORT: f32 = 0.01;
            let (from_toe_off, to_toe_off) = (walk_duty - 0.5, run_duty);
            let matched = *self.matched.get_or_insert_with(feet);
            // From the run, not before a third of its stance: root motion
            // reads the planted foot through the change, and a run's sprung
            // leg is still landing behind its target early in stance
            // (started just after the landing, the body braked to 0.3 m/s).
            let window = if self.running <= 0.0 {
                (from_toe_off, (0.5 - SHORT).min(matched.walk_of(to_toe_off - SHORT)))
            } else {
                (matched.run_of(from_toe_off).max(to_toe_off / 3.0), (to_toe_off - SHORT).min(matched.run_of(0.5 - SHORT)))
            };
            // Too short a stretch to match in: as before matching.
            let (matched, window) = if window.1 - window.0 > 0.05 { (matched, window) } else { (Matched::SAME, (from_toe_off, to_toe_off)) };
            if before < window.0 && half >= window.0 {
                self.change = Some(self.running);
                self.matched = Some(matched);
                self.window = window;
            }
        }
        let from = self.change?;
        let (start, end) = self.window;
        let through = ((half - start) / (end - start).max(1.0e-3)).clamp(0.0, 1.0);
        // Through, or past it within a frame (the clock wrapped).
        let done = through >= 1.0 || half < before;
        let to = 1.0 - from;
        if !done {
            self.running = from + (to - from) * gait::smoothstep(through);
            return None;
        }
        // The clock onto the gait come in, at the stretch's end.
        let matched = self.matched.unwrap_or(Matched::SAME);
        let onto = if from >= 1.0 { matched.walk_of(end) } else { matched.run_of(end) };
        self.running = to;
        self.change = None;
        self.matched = None;
        Some(cycle + onto - end)
    }
}

/// The reference speeds, m/s, the stride's travel is measured at for
/// [`distance_per_cycle`], interpolated between.
const DISTANCE_STEP: f32 = 0.25;
const DISTANCES: usize = ((FASTEST - SLOWEST) / DISTANCE_STEP) as usize + 1;

/// Measured stride tables, most recent first, per base pose and rig.
static DISTANCE_TABLES: Mutex<Vec<(u64, [Option<f32>; DISTANCES])>> = Mutex::new(Vec::new());

/// How far a run at reference speed `reference` travels in a cycle on this
/// rig, metres (`locomotion::distance_per_cycle`), measured every
/// [`DISTANCE_STEP`] once and interpolated: speeding up, a walker asks a new
/// speed every frame, and measuring each costs a cycle of poses.
pub fn distance_per_cycle(base: &LocalPose, rig: &RigGeometry, reference: f32) -> f32 {
    let key = key_of(base, rig);
    let x = ((reference.clamp(SLOWEST, FASTEST) - SLOWEST) / DISTANCE_STEP).min((DISTANCES - 1) as f32);
    let i = (x.floor() as usize).min(DISTANCES - 2);
    // On the legs as recorded, without the flight plan: the plan needs the
    // stride's duration, and only the horizontal travel, which the plan
    // leaves alone, decides it (`locomotion::distance_per_cycle`'s mean
    // speed over a cycle).
    let measured = |slot: usize| {
        const SAMPLES: usize = 32;
        let speed = SLOWEST + slot as f32 * DISTANCE_STEP;
        let params = super::gait::GaitParams::running_like_recorded(speed);
        let (cycle, facing) = (run_cycle(base, rig), super::stance::facing_sign(rig));
        let pose_at = |p: f32| cycle.pose_with(base, rig, p, speed, facing, None);
        (0..SAMPLES)
            .map(|i| super::locomotion::root_velocity_of(i as f32 / SAMPLES as f32, 1.0, &params, &pose_at, rig).length())
            .sum::<f32>()
            / SAMPLES as f32
    };
    let mut ends = [0.0; 2];
    if let Ok(mut tables) = DISTANCE_TABLES.lock() {
        let found = match tables.iter().position(|(k, _)| *k == key) {
            Some(found) => found,
            None => {
                tables.insert(0, (key, [None; DISTANCES]));
                tables.truncate(CACHED);
                0
            }
        };
        let table = &mut tables[found].1;
        for (end, slot) in ends.iter_mut().zip([i, i + 1]) {
            *end = *table[slot].get_or_insert_with(|| measured(slot));
        }
    } else {
        ends = [measured(i), measured(i + 1)];
    }
    ends[0] + (ends[1] - ends[0]) * (x - i as f32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::foot::{TOE_BEND, TOE_RELAX};
    use crate::character::anim::gait::{walk_pose_on, GaitParams};
    use crate::character::anim::locomotion::distance_per_cycle;

    fn real_stood() -> (LocalPose, RigGeometry) {
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    /// How high each foot's lowest contact is above the floor in `pose`,
    /// metres.
    fn heights(pose: &LocalPose, stood: &LocalPose, rig: &RigGeometry) -> [f32; 2] {
        let rise = pose.root_translation.y - stood.root_translation.y;
        [0, 1].map(|leg| {
            let sole = Sole::of(rig, LEGS[leg].3);
            rise + lowest(&sole.points(pose, rig)) - lowest(&sole.points(stood, rig))
        })
    }

    #[test]
    fn the_recordings_are_three_speeds_whose_stance_and_stride_shorten() {
        let strides = &*FUKUCHI;
        assert_eq!(strides.iter().map(|s| s.speed).collect::<Vec<_>>(), [2.5, 3.5, 4.5]);
        for pair in strides.windows(2) {
            assert!(pair[1].duty < pair[0].duty && pair[1].stride_seconds < pair[0].stride_seconds, "{pair:?}");
        }
        // Contact 0.29-0.21 s and 161-183 steps a minute: the running
        // literature's figures for these speeds.
        for s in strides.iter() {
            let contact = s.duty * s.stride_seconds;
            assert!((0.2..0.3).contains(&contact), "{} m/s: contact {contact} s", s.speed);
            assert!((0.29..0.42).contains(&s.duty), "{} m/s: duty {}", s.speed, s.duty);
        }
    }

    /// The cadence the pose's own stride gives (`locomotion::distance_per_cycle`
    /// at each speed) against the recordings', at the same Froude number.
    /// Measured on `puppet_base`: 4.3-5.8 % quicker. With the toes held
    /// rigid it was within 1 %, but the toe tip then swept back under the
    /// body and the pelvis rose 65 mm at toe-off; bent, the ball carries the
    /// last of the stance and the stride is that much shorter.
    #[test]
    fn a_run_steps_near_the_recorded_cadence() {
        let (stood, rig) = real_stood();
        let leg = gait::leg_length_of(&rig);
        for speed in [2.5f32, 3.5, 4.5] {
            let params = GaitParams::running_on(speed, &rig);
            let seconds = distance_per_cycle(&params, &stood, &rig) / speed;
            let recorded = stride_seconds_at(reference_speed(speed, leg)) * (leg / recorded_leg_length()).sqrt();
            let off = seconds / recorded - 1.0;
            assert!((-0.08..0.02).contains(&off), "{speed} m/s: a stride takes {seconds} s against the recorded {recorded}");
        }
    }

    /// Between toe-off and the next contact the pelvis falls as a body
    /// does, at g (`FlightPlan`), the legs giving little for it: measured on
    /// `puppet_base`, the flight's acceleration within -10.0..-9.3 m/s² at
    /// every speed, the knee at most 17 degrees off the recording (at
    /// 4.5 m/s; 7.5-10 elsewhere), the thigh 9. With the thigh and shank
    /// from standing rather than from vertical, following the legs alone it
    /// fell 5 cm a flight, its acceleration -14..-5 m/s².
    ///
    /// It leaves the ground rising, 0.22-0.42 m/s, and lands at 0.42-0.75
    /// (the runners' hips: up 0.53, down 0.73 at 3.5 m/s); the bob, lowest
    /// in stance, 71-97 mm against the recorded 78-97 (scaled to the rig's
    /// leg). With the legs' own toe-off and a dip only after landing, it
    /// took off level, landed at 0.8-1.2 m/s and spanned 20 % short.
    #[test]
    fn a_running_body_flies_at_g_and_its_legs_give_little_for_it() {
        let (stood, rig) = real_stood();
        let leg = gait::leg_length_of(&rig);
        let facing = crate::character::anim::stance::facing_sign(&rig);
        for speed in [2.2f32, 2.5, 3.5, 4.5, 5.0, 6.0] {
            let params = GaitParams::running_on(speed, &rig);
            let reference = reference_speed(speed, leg);
            let seconds = super::distance_per_cycle(&stood, &rig, reference) / speed;
            let n = 400;
            let heights: Vec<f32> =
                (0..n).map(|i| walk_pose_on(i as f32 / n as f32, &params, &stood, &rig).root_translation.y).collect();
            let dt = seconds / n as f32;
            let duty = params.duty_factor;
            for i in 0..n {
                let half = (i as f32 / n as f32).rem_euclid(0.5);
                if half > duty + 0.01 && half < 0.49 {
                    let acceleration = (heights[(i + n - 1) % n] - 2.0 * heights[i] + heights[(i + 1) % n]) / (dt * dt);
                    assert!((-10.8..-8.8).contains(&acceleration), "{speed} m/s: the pelvis accelerates {acceleration} m/s² in flight");
                }
            }
            let plan = run_cycle(&stood, &rig).plan(&stood, &rig, reference, recorded_speed(speed, leg) / reference, facing);
            for i in 0..100 {
                let [thigh, knee] = plan.correction(reference, i as f32 / 100.0);
                assert!(knee.abs() < 18f32.to_radians() && thigh.abs() < 10f32.to_radians(), "{speed} m/s: legs moved {thigh} / {knee} rad");
            }
            let span = heights.iter().copied().fold(f32::MIN, f32::max) - heights.iter().copied().fold(f32::MAX, f32::min);
            let (a, b, t) = bracket(reference);
            let range = |s: &RunStride| {
                let bob: Vec<f32> = (0..100).map(|i| s.pelvis_bob.at(i as f32 / 100.0)).collect();
                bob.iter().copied().fold(f32::MIN, f32::max) - bob.iter().copied().fold(f32::MAX, f32::min)
            };
            let recorded = (range(a) + (range(b) - range(a)) * t) * leg / recorded_leg_length();
            assert!((span / recorded - 1.0).abs() < 0.15, "{speed} m/s: the pelvis spans {span} m against the recorded {recorded}");
            // It leaves the ground rising, as a runner's does, and lands
            // no harder than one.
            let (off, on) = ((duty * n as f32).ceil() as usize + 1, (0.5 * n as f32).floor() as usize - 1);
            let (take_off, landing) = ((heights[off + 1] - heights[off]) / dt, (heights[on] - heights[on - 1]) / dt);
            assert!(take_off > 0.15, "{speed} m/s: the body leaves the ground at {take_off} m/s");
            assert!((-0.9..-0.3).contains(&landing), "{speed} m/s: the body lands at {landing} m/s");
            let lowest_at = heights.iter().enumerate().min_by(|a, b| a.1.total_cmp(b.1)).unwrap().0 as f32 / n as f32;
            assert!(gait::leg_phase(lowest_at, duty).is_stance() || gait::leg_phase(lowest_at + 0.5, duty).is_stance());
        }
    }

    /// Through stance the foot is on the floor; between stances neither is:
    /// a run's flight. Through swing no foot goes into the floor (the knee
    /// guard, `RunCycle::clear_swing`, took a dip of 26 mm out at the
    /// slowest run), and it is off the floor from just after toe-off.
    #[test]
    fn a_run_stands_on_one_foot_then_flies_and_never_scuffs() {
        let (stood, rig) = real_stood();
        for speed in [2.2f32, 3.0, 4.5, 6.0] {
            let params = GaitParams::running_on(speed, &rig);
            let mut flight = 0;
            for i in 0..400 {
                let p = i as f32 / 400.0;
                let pose = walk_pose_on(p, &params, &stood, &rig);
                let height = heights(&pose, &stood, &rig);
                let mut down = 0;
                for leg in 0..2 {
                    match gait::leg_phase(p + LEGS[leg].0, params.duty_factor) {
                        LegPhase::Stance { .. } => {
                            down += 1;
                            assert!(height[leg].abs() < 1.0e-3, "{speed} m/s, phase {p}: a foot down {} mm off the floor", height[leg] * 1e3);
                        }
                        LegPhase::Swing { progress } => {
                            assert!(height[leg] > -1.5e-3, "{speed} m/s, phase {p}: a swinging foot {} mm into the floor", height[leg] * 1e3);
                            // Over its last tenth, coming down to land, at
                            // least the landing's own clearance.
                            let least = if progress < 0.9 { 0.005 } else { swing_clearance(progress) };
                            if (0.05..0.97).contains(&progress) {
                                assert!(height[leg] > least, "{speed} m/s, phase {p}: a swinging foot only {} mm up", height[leg] * 1e3);
                            }
                        }
                    }
                }
                assert!(down <= 1, "a run never has both feet down");
                flight += (down == 0) as usize;
            }
            let share = flight as f32 / 400.0;
            assert!((share - (1.0 - 2.0 * params.duty_factor)).abs() < 0.01, "{speed} m/s: in the air {share} of the cycle");
        }
    }

    #[test]
    fn the_two_legs_run_alike_half_a_cycle_apart() {
        let (stood, rig) = real_stood();
        let params = GaitParams::running_on(3.5, &rig);
        for i in 0..50 {
            let p = i as f32 / 50.0;
            let (a, b) = (walk_pose_on(p, &params, &stood, &rig), walk_pose_on(p + 0.5, &params, &stood, &rig));
            let left = gait::sagittal_angles(&a, &rig, gait::leg_joints(Bone::LeftFoot));
            let right = gait::sagittal_angles(&b, &rig, gait::leg_joints(Bone::RightFoot));
            for (l, r) in left.iter().zip(right) {
                assert!((l - r).abs() < 2.0e-3, "phase {p}: left {left:?} against right {right:?}");
            }
        }
    }

    /// Fukuchi's knee: ~42 degrees at mid-stance, 92-119 at the swing's
    /// fold, the fold deeper the faster.
    #[test]
    fn the_knee_folds_further_the_faster_the_run() {
        let (stood, rig) = real_stood();
        let fold = |speed: f32| {
            let params = GaitParams::running_on(speed, &rig);
            (0..100)
                .map(|i| gait::sagittal_angles(&walk_pose_on(i as f32 / 100.0, &params, &stood, &rig), &rig, gait::leg_joints(Bone::LeftFoot))[2])
                .fold(f32::MIN, f32::max)
        };
        let (slow, fast) = (fold(2.5), fold(4.5));
        assert!((1.5..1.75).contains(&slow) && (1.95..2.2).contains(&fast) && fast > slow, "{slow} {fast}");
    }

    /// The trunk leans further forward the faster (Thorstensson 1984), and
    /// the head stays ahead of the hips by it.
    #[test]
    fn a_faster_run_leans_further_forward() {
        let (stood, rig) = real_stood();
        let ahead = |speed: f32| {
            let pose = walk_pose_on(0.2, &GaitParams::running_on(speed, &rig), &stood, &rig);
            crate::character::anim::rig::offset_from(&pose, &rig, Bone::Hips, Bone::Head).dot(rig.forward())
        };
        let standing = crate::character::anim::rig::offset_from(&stood, &rig, Bone::Hips, Bone::Head).dot(rig.forward());
        let (slow, fast) = (ahead(2.2), ahead(5.5));
        assert!(standing < slow && slow < fast, "head ahead of the hips: standing {standing}, slow {slow}, fast {fast}");
    }

    #[test]
    fn the_toes_bend_up_through_push_off_and_straighten_in_swing() {
        for speed in [2.5f32, 3.5, 4.5] {
            let duty = duty_at(speed);
            assert!(toe_bend(speed, 0.0).abs() < 0.02, "a heel strike's toes are straight");
            let push = toe_bend(speed, duty - 0.01);
            assert!((0.45..=TOE_BEND + 1.0e-3).contains(&push), "{speed} m/s: bent {push} at toe-off");
            assert!(toe_bend(speed, duty + TOE_RELAX) < 1.0e-4, "straight again by early swing");
            for i in 0..100 {
                assert!(toe_bend(speed, i as f32 / 100.0) <= TOE_BEND + 1.0e-3);
            }
        }
    }

    #[test]
    fn a_swinging_foot_leaves_the_floor_at_once_and_lands_gently() {
        assert_eq!(swing_clearance(0.0), 0.0);
        assert_eq!(swing_clearance(1.0), 0.0);
        // Rising at once: a tenth of the way to its lift, it is well above a
        // tenth of it (a smoothstep would be at 3 %).
        assert!(swing_clearance(0.1 * TOE_LIFT_BY) > 0.15 * TOE_LIFT);
        assert!((swing_clearance(TOE_LIFT_BY) - TOE_LIFT).abs() < 1.0e-6);
    }

    #[test]
    fn the_pace_jumps_to_a_walk_and_gathers_above_it() {
        let (changeover, dt) = (2.0, 0.1);
        // A walk takes its speed at once.
        assert_eq!(paced(1.5, 0.0, 0.0, changeover, false, false, dt), 1.5);
        // Starting, a run waits at the changeover until the walk is in...
        let mut pace = 0.0;
        for _ in 0..10 {
            pace = paced(4.0, pace, 0.0, changeover, false, false, dt);
            assert_eq!(pace, changeover);
        }
        // ...then gathers at ACCELERATION.
        pace = paced(4.0, pace, 0.0, changeover, true, false, dt);
        assert!((pace - (changeover + ACCELERATION * dt)).abs() < 1.0e-6);
        for _ in 0..100 {
            pace = paced(4.0, pace, 1.0, changeover, true, false, dt);
        }
        assert_eq!(pace, 4.0);
        // Stopping, it slows while the gait is still a run, down to where a
        // run turns back into a walk and no further until it has; walking
        // again, it stops at once.
        let slowed = paced(0.0, 4.0, 1.0, changeover, true, false, dt);
        assert!((slowed - (4.0 - DECELERATION * dt)).abs() < 1.0e-6);
        let mut pace = slowed;
        for _ in 0..100 {
            pace = paced(0.0, pace, 1.0, changeover, true, false, dt);
        }
        assert_eq!(pace, changeover * BACK_TO_WALK);
        assert_eq!(paced(0.0, pace, 0.0, changeover, true, false, dt), 0.0);
        // Walking, a slower walk is shed into and a faster one gathered, not
        // jumped to: a walk's speed sets its pose, and the hips with it.
        let shed = paced(1.0, changeover * BACK_TO_WALK, 0.0, changeover, true, false, dt);
        assert!((shed - (changeover * BACK_TO_WALK - DECELERATION * dt)).abs() < 1.0e-6);
        let gathered = paced(1.8, 1.2, 0.0, changeover, true, false, dt);
        assert!((gathered - (1.2 + ACCELERATION * dt)).abs() < 1.0e-6);
        assert_eq!(paced(4.0, 1.2, 0.0, changeover, true, false, dt), gathered);
        // With both feet down a walk holds its speed either way; a stop
        // still takes the walk's own at once, and a run is not held.
        assert_eq!(paced(1.0, 1.8, 0.0, changeover, true, true, dt), 1.8);
        assert_eq!(paced(1.8, 1.2, 0.0, changeover, true, true, dt), 1.2);
        assert_eq!(paced(0.0, 1.2, 0.0, changeover, true, true, dt), 0.0);
        assert_eq!(paced(0.0, 4.0, 1.0, changeover, true, true, dt), slowed);
    }

    /// A walk asked to slow, its speed eased by `paced` and its root motion
    /// read off its contacts, as the walker does it: on `puppet_base`, from
    /// 1.2 to 0.6 m/s starting at a footfall, the trailing foot moves no
    /// more on the floor than a steady walk's. Eased with both feet down
    /// too, it slid up to 18 mm a frame as it left; live, its toe tip slid
    /// 28 mm, and 10-18 mm slowing into a walk after a run.
    #[test]
    fn a_walk_changes_its_speed_only_with_one_foot_down() {
        use crate::character::anim::foot::Sole;
        use crate::character::anim::locomotion::root_displacement_between;
        use crate::character::anim::rig::forward_kinematics_on;
        use bevy::math::Vec3;
        let (stood, rig) = real_stood();
        let changeover = changeover_speed(gait::leg_length_of(&rig));
        let dt = 1.0 / 60.0;
        let soles = |pose: &LocalPose, body: Vec3| {
            let hips = forward_kinematics_on(pose, &rig)[Bone::Hips];
            [Bone::LeftFoot, Bone::RightFoot].map(|ankle| Sole::of(&rig, ankle).points(pose, &rig).map(|p| p + hips + body))
        };
        let floor = soles(&stood, Vec3::ZERO).iter().flatten().map(|p| p.y).fold(f32::MAX, f32::min);
        // From a right footfall, the left trailing: the worst move in a
        // frame of the trailing foot's contacts within a millimetre of the
        // floor both frames, both feet down, metres; and the pace at the end.
        let walk = |from: f32, asked: f32, hold: bool| {
            let (mut pace, mut cycle, mut body) = (from, 0.5f32, Vec3::ZERO);
            let params = GaitParams::walking_on(pace, &rig);
            let mut previous = (walk_pose_on(cycle, &params, &stood, &rig), cycle);
            let mut worst = 0.0f32;
            for _ in 0..120 {
                let duty = GaitParams::walking_on(pace, &rig).duty_factor;
                let both_down = [0.0, 0.5].map(|shift| gait::leg_phase(cycle + shift, duty).is_stance()) == [true; 2];
                pace = paced(asked, pace, 0.0, changeover, true, hold && both_down, dt);
                let params = GaitParams::walking_on(pace, &rig);
                cycle = (cycle + pace / distance_per_cycle(&params, &stood, &rig) * dt).rem_euclid(1.0);
                let pose = walk_pose_on(cycle, &params, &stood, &rig);
                let before = soles(&previous.0, body);
                let middle = previous.1 + 0.5 * (cycle - previous.1).rem_euclid(1.0);
                body += root_displacement_between(&previous.0, &pose, middle, &params, &rig).unwrap_or(Vec3::ZERO);
                let after = soles(&pose, body);
                let phases = [0, 1].map(|leg| gait::leg_phase(cycle + 0.5 * leg as f32, params.duty_factor));
                let trailing = |leg: usize| {
                    matches!(phases[leg], gait::LegPhase::Stance { progress } if progress > 0.5) && phases[1 - leg].is_stance()
                };
                for leg in (0..2).filter(|&leg| trailing(leg)) {
                    for (b, a) in before[leg].iter().zip(&after[leg]).filter(|(b, a)| b.y < floor + 1.0e-3 && a.y < floor + 1.0e-3) {
                        worst = worst.max(Vec3::new(a.x - b.x, 0.0, a.z - b.z).length());
                    }
                }
                previous = (pose, cycle);
            }
            (worst, pace)
        };
        // Measured: 0.62 mm, as steady; eased with both feet down, the
        // stance share grew under the foot rolling onto its toe, and it
        // slid 1.2, 3.0, 7.9 then 18.3 mm a frame.
        let (steady, _) = walk(1.2, 1.2, true);
        let (held, pace) = walk(1.2, 0.6, true);
        let (eased, _) = walk(1.2, 0.6, false);
        assert!((pace - 0.6).abs() < 1.0e-4, "test setup: slowed only to {pace}");
        assert!(
            held < steady + 5.0e-4 && eased > held + 1.0e-2,
            "slowing, the trailing foot moved {:.2} mm a frame ({:.2} eased with both feet down), steady {:.2}",
            held * 1e3,
            eased * 1e3,
            steady * 1e3
        );
    }

    /// Through a change between walk and run, the gait coming in is posed
    /// where its planted foot matches the one going out's
    /// (`Matched::feet`), and the clock moves onto it at the end: on
    /// `puppet_base` at the back-to-walk speed, the planted toe keeps the
    /// body's pace under the hips over the change within 10 % (0.77-1.29 of
    /// it frame by frame, as the gaits' own steps vary), both ways, and the
    /// pose does not jump as the clock moves. Blended at one clock, it kept
    /// 1.16 of it walk to run; live, run to walk, the walk's root motion,
    /// read off that toe, braked the body from 2.0 to 0.8 m/s.
    #[test]
    fn a_change_between_walk_and_run_keeps_the_planted_foots_pace() {
        let (stood, rig) = real_stood();
        let leg = gait::leg_length_of(&rig);
        let changeover = changeover_speed(leg);
        let speed = changeover * BACK_TO_WALK;
        let (walk, run) = (GaitParams::walking_on(speed, &rig), GaitParams::running_on(speed, &rig));
        let walked = distance_per_cycle(&walk, &stood, &rig);
        let ran = super::distance_per_cycle(&stood, &rig, reference_speed(speed, leg));
        let dt = 1.0 / 60.0;
        let feet = || Matched::feet(|p| walk_pose_on(p, &walk, &stood, &rig), |p| walk_pose_on(p, &run, &stood, &rig), walk.duty_factor, run.duty_factor, &rig);
        // Asked a run from a walk, then a walk from the run.
        for (from, asked) in [(0.0f32, changeover * 1.01), (1.0, speed * 0.99)] {
            let mut gaits = Gaits { running: from, ..Default::default() };
            let toe = |gaits: &Gaits, cycle: f32| {
                let (w, r) = gaits.phases(cycle);
                let pose = crate::character::anim::clip::blend(&walk_pose_on(w, &walk, &stood, &rig), &walk_pose_on(r, &run, &stood, &rig), gaits.running);
                // The foot down through the change: the right, at the clock's
                // second half.
                crate::character::anim::rig::offset_from(&pose, &rig, Bone::Hips, Bone::RightToeBase).dot(rig.forward())
            };
            let (mut cycle, mut previous) = (0.52f32, 0.5f32);
            let mut last = toe(&gaits, cycle);
            let mut changed = false;
            let (mut travelled, mut frames) = (0.0f32, 0);
            for _ in 0..120 {
                let stride = if gaits.running <= 0.0 || (gaits.changing() && !gaits.from_run()) { walked } else { ran };
                let next = cycle + speed / stride * dt;
                let before = gaits.running;
                let moved = gaits.advance(asked, changeover, walk.duty_factor, run.duty_factor, next, cycle, feet);
                previous = cycle;
                cycle = moved.unwrap_or(next);
                let now = toe(&gaits, cycle);
                changed |= gaits.running != before;
                if gaits.running != before && cycle.rem_euclid(1.0) >= 0.5 {
                    let step = (last - now) / (speed * dt);
                    assert!((0.7..1.35).contains(&step), "from {from}: at {cycle:.3} ({}) the toe went {step} of the body's pace", gaits.running);
                    travelled += last - now;
                    frames += 1;
                }
                last = now;
                if changed && !gaits.changing() {
                    break;
                }
            }
            let _ = previous;
            assert!(changed && gaits.running == 1.0 - from, "from {from}: the change never ended ({})", gaits.running);
            let pace = travelled / (speed * dt * frames as f32);
            assert!((pace - 1.0).abs() < 0.1, "from {from}: over the change the toe kept {pace} of the body's pace");
        }
    }

    /// The change between walk and run starts only as a step's stretch
    /// opens (the walk's other foot's toe-off), runs through it, and ends at
    /// the run's toe-off; never in a walk's double support.
    #[test]
    fn walk_and_run_change_over_one_step_of_single_support() {
        let (walk_duty, run_duty, changeover) = (0.6, 0.38, 2.0);
        let (opens, closes) = (walk_duty - 0.5, run_duty);
        let mut gaits = Gaits::default();
        let mut previous = 0.3;
        let mut began = None;
        for i in 1..400 {
            let cycle = 0.3 + i as f32 * 0.005;
            let before = gaits.running;
            gaits.advance(2.5, changeover, walk_duty, run_duty, cycle, previous, || Matched::SAME);
            let half = cycle.rem_euclid(0.5);
            if gaits.running != before {
                assert!((opens..=closes + 0.005).contains(&half), "changed at {half} of a step, outside {opens}-{closes}");
                assert!(gaits.running >= before, "a change one way only");
                // Eased from where its stretch opens, not jumped into
                // part-way: 1/56 of the stretch a frame here.
                assert!(gaits.running - before < 0.06, "jumped {before} -> {}", gaits.running);
                began.get_or_insert(cycle);
            }
            previous = cycle;
            if gaits.running >= 1.0 {
                let began = began.expect("a change");
                assert!(cycle - began < 0.5, "over one step: began {began}, done {cycle}");
                break;
            }
        }
        assert_eq!(gaits.running, 1.0, "running by now");
        // Between the changeover and a tenth below it, it keeps running.
        let mut previous = 0.0;
        for i in 1..400 {
            let cycle = i as f32 * 0.005;
            gaits.advance(changeover * 0.95, changeover, walk_duty, run_duty, cycle, previous, || Matched::SAME);
            previous = cycle;
        }
        assert_eq!(gaits.running, 1.0, "held between the two speeds");
        gaits.walk();
        assert_eq!(gaits, Gaits::default());
    }

    /// Measured at 3.5 m/s on `puppet_base`: the upper arm swings 37
    /// degrees behind to 5 in front, the elbow folds 68-106 degrees, and the
    /// hand rides ahead of the elbow all the way round.
    #[test]
    fn a_running_arm_stays_bent_with_the_hand_ahead_of_the_elbow() {
        let (stood, rig) = real_stood();
        let params = GaitParams::running_on(3.5, &rig);
        let (mut back, mut front) = (0.0f32, 0.0f32);
        for i in 0..32 {
            let pose = walk_pose_on(i as f32 / 32.0, &params, &stood, &rig);
            let at = |b| crate::character::anim::rig::offset_from(&pose, &rig, Bone::Hips, b);
            let (shoulder, elbow, hand) = (at(Bone::LeftArm), at(Bone::LeftForeArm), at(Bone::LeftHand));
            let (upper, fore) = ((elbow - shoulder).normalize(), (hand - elbow).normalize());
            let swing = upper.dot(rig.forward()).atan2(-upper.y).to_degrees();
            (back, front) = (back.min(swing), front.max(swing));
            let flexion = upper.dot(fore).clamp(-1.0, 1.0).acos().to_degrees();
            assert!((60.0..115.0).contains(&flexion), "elbow at {flexion} degrees");
            assert!((hand - elbow).dot(rig.forward()) > 0.1, "the hand fell behind the elbow");
        }
        assert!(back < -25.0 && front > 0.0 && front < 20.0, "the upper arm swings {back} to {front} degrees");
    }

    /// Seen from the front, a running hand comes in across the body as it
    /// swings forward: inside the shoulder at the front of the swing (0.17 m
    /// off the midline against the shoulder's 0.22, on `puppet_base` at
    /// 3.5 m/s; 0.24, outside it, before `GaitParams::arm_inward`), out by
    /// the hip at the back. Both arms alike.
    #[test]
    fn a_running_hand_comes_in_across_the_body_at_the_front_of_its_swing() {
        let (stood, rig) = real_stood();
        let params = GaitParams::running_on(3.5, &rig);
        for (shoulder, hand) in [(Bone::LeftArm, Bone::LeftHand), (Bone::RightArm, Bone::RightHand)] {
            let mut front = (f32::MIN, 0.0);
            let mut back = (f32::MAX, 0.0);
            for i in 0..32 {
                let pose = walk_pose_on(i as f32 / 32.0, &params, &stood, &rig);
                let at = |b| crate::character::anim::rig::offset_from(&pose, &rig, Bone::Hips, b);
                let ahead = (at(hand) - at(shoulder)).dot(rig.forward());
                let across = at(hand).dot(rig.left()).abs() / at(shoulder).dot(rig.left()).abs();
                if ahead > front.0 {
                    front = (ahead, across);
                }
                if ahead < back.0 {
                    back = (ahead, across);
                }
            }
            assert!(front.1 < 0.85, "{shoulder:?}: at the front of the swing the hand is {} of the shoulder's width out", front.1);
            assert!(back.1 > 1.1, "{shoulder:?}: at the back the hand is {} of the shoulder's width out", back.1);
        }
        // A walking arm swings fore and aft.
        assert_eq!(GaitParams::walking_on(1.4, &rig).arm_inward, 0.0);
    }
}
