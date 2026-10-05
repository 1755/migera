//! Jumping from a stand, planned as the body's centre of mass moving under
//! the forces a person can put through the floor.
//!
//! A countermovement jump (McMahon et al. 2018: unweighting, braking,
//! propulsion, flight, landing) is planned once, from the height asked, as
//! the path of the centre of mass (COM) over time:
//!
//! - **Down:** from standing still to the bottom of the countermovement,
//!   still again, no quicker than unloading the floor to [`LEAST_LOAD`] of
//!   body weight allows.
//! - **Push:** from the bottom up to take-off at constant acceleration, so
//!   the body leaves at exactly `√(2·g·h)` upward.
//! - **Flight:** a parabola under `g`. Winter (p. 88): the path is decided
//!   at take-off and nothing the limbs do in the air changes it.
//! - **Land:** from touchdown down to the landing's lowest point at
//!   constant deceleration, [`LANDING_DECELERATION`].
//! - **Recover:** back up to standing, still.
//!
//! Each frame the pose's own shape is set (trunk lean, arm swing, legs),
//! and then the pelvis is solved so that the pose's real centre of mass
//! (`anthropometry::centre_of_mass`) lies on the path. Arms swinging up
//! raise the COM in the body, and the pelvis then rises less for it, as a
//! real jumper's does (Lees et al. 2004).
//!
//! Feet that are down stay where they stood. Their heels rise about the
//! toe tips only when the legs, straight, can no longer reach them, as in
//! the end of a real push.

use bevy::math::{Quat, Vec3};

use super::anthropometry::centre_of_mass;
use super::foot::Sole;
use super::gait::{hermite, smoothstep};
use super::math::SpringParams;
use super::rig::{accumulate_world_rotations, delta_after_world_turn, offset_from, LocalPose, RigGeometry};
use super::stance::place_ankle;
use crate::character::skeleton::Bone;

/// Gravity, m/s².
pub const GRAVITY: f32 = 9.81;

/// Each leg's socket, knee and ankle: left, right.
const LEGS: [(Bone, Bone, Bone); 2] = [
    (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
    (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
];

/// The highest jump asked, metres of COM rise above take-off. Pushing over
/// the countermovement's path at no more than ~2.6 body weights on average
/// gives about this. Recreational adults jump 0.24-0.35 m (McMahon et al.
/// 2017, McInnis & Donahue 2024).
pub const HIGHEST: f32 = 0.6;

/// How deep the countermovement takes the COM, per metre of jump height:
/// self-chosen depths are 0.31-0.36 m for 0.32-0.35 m jumps (McMahon et
/// al. 2017; McHugh et al. 2024).
pub const DEPTH_PER_HEIGHT: f32 = 0.85;

/// The countermovement's depth for the smallest and largest jumps, metres.
pub const DEPTHS: (f32, f32) = (0.12, 0.38);

/// The least the floor is loaded as the body drops into the
/// countermovement, of body weight: 0.43 measured (McHugh et al. 2024).
/// It caps how fast the drop starts.
pub const LEAST_LOAD: f32 = 0.43;

/// The quickest countermovement, seconds: 0.5-0.7 measured to the bottom
/// (McHugh et al. 2024; McMahon et al. 2017).
pub const QUICKEST_DOWN: f32 = 0.5;

/// How hard a landing decelerates the COM, m/s² on average: the floor
/// carries twice the body's weight. A soft landing peaks at 1.6 body
/// weights and a stiff one at 2.6 (Myers et al. 2011, from 40 cm).
pub const LANDING_DECELERATION: f32 = 1.0 * GRAVITY;

/// How far a landing may take the COM below where it touched down, metres.
/// Deeper than the countermovement: soft landings flex the knee to 117°
/// (DeVita & Skelly 1992).
pub const DEEPEST_LANDING: f32 = 0.42;

/// The most the COM accelerates up out of the landing to standing, m/s²:
/// it slows as hard at the top, which unloads the floor as the drop into
/// the countermovement does, to [`LEAST_LOAD`].
pub const RECOVERY_ACCELERATION: f32 = (1.0 - LEAST_LOAD) * GRAVITY;

/// How far the heels have risen about the toe tips at take-off, radians
/// (20°): with the arms up, the COM leaves 0.11 m above standing on
/// `puppet_base`, where McMahon et al. 2017's displacements put it at
/// 0.09-0.11 m. The heels carry most of it: 0.094 m at 0.5 rad on their
/// own, the arms 0.035 m, straight knees almost none. At 0.5 rad the COM
/// left 0.14 m up. A heel rise this small is less plantarflexion than
/// joint-angle studies give at take-off, a tension this keeps on the side
/// of the measured COM.
pub const HEEL_RISE: f32 = 0.35;

/// How far short of straight the knees stay at take-off, radians: "near
/// full extension".
pub const KNEE_AT_TAKEOFF: f32 = 0.12;

/// How far the trunk leans forward per metre the COM is below standing,
/// radians: about 45° at the bottom of a 0.33 m countermovement
/// (Vanrenterghem et al. 2008: held upright, the jump loses 10 %).
pub const LEAN_PER_DEPTH: f32 = 2.3;

/// The most the trunk leans, radians.
pub const MOST_LEAN: f32 = 0.85;

/// How much of the lean the pelvis takes; the spine takes the rest.
pub const PELVIS_SHARE: f32 = 0.4;

/// How far the arms swing back by the bottom of the countermovement and
/// forward-up by take-off, radians from hanging (Lees et al. 2004: back in
/// the descent, forward and up through the push).
/// Up a little over the shoulders' height at take-off.
///
/// Each is (swing, elbow): the upper arm swung forward from hanging and
/// the elbow bent on top of the standing pose's, radians. The swing is
/// driven with the elbows a little bent, the long arm the lever (35° back,
/// 17° up); landing, the arms come forward and bend to guard (46°, then
/// 69°).
pub const ARMS_BACK: (f32, f32) = (-0.8, 0.6);
pub const ARMS_UP: (f32, f32) = (2.0, 0.3);

/// The arms ahead at touchdown, and at the bottom of the landing.
pub const ARMS_LANDING: (f32, f32) = (0.5, 0.8);
pub const ARMS_LANDED: (f32, f32) = (0.8, 1.2);

/// The jump height, metres, from which the arms swing as far as the
/// `ARMS_*` shapes; below it, by the square of the height's share of it
/// (8 % at 0.1 m, a third at 0.2 m). The arm swing is a maximal jump's (it
/// adds 10-13 % to the take-off speed, Ashby & Heegaard 2002); a small hop
/// hardly moves the arms. Scaled in proportion, a 0.1 m hop still swung
/// them 40° forward.
pub const FULL_ARMS: f32 = 0.35;

/// How a landing meets the floor: on the forefoot, the heels risen this
/// far about the toe tips (radians), and the knees this far short of
/// straight (15° at contact, Myers et al. 2011). The heels come down as the
/// body sinks.
pub const LANDING_HEEL: f32 = 0.35;
pub const LANDING_KNEE: f32 = 0.26;

/// How far the feet tuck up under the body mid-flight, per metre of jump
/// height.
pub const TUCK_PER_HEIGHT: f32 = 0.15;

/// The most a bone is led ahead of its spring, seconds: the trunk's and
/// head's soft springs lag 0.4-0.5 s, and led that far they would turn
/// ahead of the jump's own phases.
pub const MOST_LEAD: f32 = 0.1;

/// How far a spring trails a target moving at a steady rate, seconds:
/// `2ζ/ω` for `x'' = −2ζω·x' − ω²·(x − target)`, with `ω` the spring's
/// decay rate (`ln 2 / halflife`). 0.043 s for the legs' 0.015 s
/// half-life, 0.087 s for the arms' 0.03 s; capped at [`MOST_LEAD`].
pub fn lead_of(spring: &SpringParams) -> f32 {
    (2.0 * spring.damping_ratio / spring.decay_rate()).min(MOST_LEAD)
}

/// Where a jump is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JumpPhase {
    Down,
    Push,
    Flight,
    Land,
    Recover,
}

/// The legs of a pose: down on the feet where they stood, the heels risen
/// `heel` about the toe tips, and further if the knees need it to stay
/// `knee` short of straight; or free in the air with each ankle
/// hips-relative and the feet pitched toe-down by an angle.
#[derive(Debug, Clone, Copy)]
enum Legs {
    Down { knee: f32, heel: f32 },
    Free { ankles: [Vec3; 2], pitch: f32 },
}

/// What a pose on its feet is solved for: its COM at `height`, or as high
/// as its legs reach (the shape it leaves or meets the floor in). Either
/// way the heels risen `heel` at least and the knees at least `knee` short
/// of straight.
#[derive(Debug, Clone, Copy)]
enum Aim {
    Height { height: f32, knee: f32, heel: f32 },
    Reach { heel: f32, knee: f32 },
}

/// How far the heels have risen about the toe tips `u` through `phase`, at
/// least, radians. The push straightens hip, then knee, then ankle (the
/// proximal-to-distal order of a jump): the heels rise over its last 60 %.
/// Risen only once the legs were straight, they rose in the push's last
/// two frames, and the sprung legs hitched the pelvis at take-off (357 m/s²
/// live). Landing, they come down over its first 40 %.
fn heel_at(phase: JumpPhase, u: f32) -> f32 {
    match phase {
        JumpPhase::Push => HEEL_RISE * smoothstep((u - 0.4) / 0.6),
        JumpPhase::Land => LANDING_HEEL * (1.0 - smoothstep(u / 0.4)),
        _ => 0.0,
    }
}

/// The knees' least bend on the floor in `phase`: pushing off, all but
/// straight; landing, as they met it.
fn least_knee(phase: JumpPhase) -> f32 {
    match phase {
        JumpPhase::Down | JumpPhase::Push => KNEE_AT_TAKEOFF,
        _ => LANDING_KNEE,
    }
}

/// A jump under way: its plan, and how far into it.
#[derive(Debug, Clone)]
pub struct Jump {
    /// Seconds since it began.
    t: f32,
    /// When each phase ends, seconds from the start: down, push, flight,
    /// land, recover.
    ends: [f32; 5],
    /// The COM at standing, in the standing hips' frame: along the rig's
    /// forward, and up.
    stand: (f32, f32),
    /// The COM's height at the bottom of the countermovement, at take-off,
    /// at touchdown and at the bottom of the landing.
    bottom: f32,
    takeoff: f32,
    touchdown: f32,
    lowest: f32,
    /// The COM's speed up at take-off and down at touchdown, m/s.
    up: f32,
    down: f32,
    /// The legs' shape leaving the floor and meeting it: ankles hips-relative.
    leaving: [Vec3; 2],
    meeting: [Vec3; 2],
    /// The same ankles in the standing hips' frame.
    leaving_at: [Vec3; 2],
    meeting_at: [Vec3; 2],
    tuck: f32,
    /// How much of the full arm swing it takes, 0-1 ([`FULL_ARMS`]).
    arms: f32,
    feet: Feet,
}

/// Where the feet stand, worked out once from the standing pose.
#[derive(Debug, Clone, Copy)]
struct Feet {
    /// Each ankle, and each toe tip, in the standing hips' frame.
    ankles: [Vec3; 2],
    tips: [Vec3; 2],
    /// Each foot's world rotation standing, and the axis its heel rises
    /// about (horizontal, across the foot).
    attitudes: [Quat; 2],
    axes: [Vec3; 2],
    /// Each leg's thigh and shank lengths.
    limbs: [(f32, f32); 2],
}

impl Feet {
    fn of(stood: &LocalPose, rig: &RigGeometry) -> Self {
        let world = accumulate_world_rotations(stood, rig);
        let ankles = LEGS.map(|(_, _, ankle)| offset_from(stood, rig, Bone::Hips, ankle));
        let tips = LEGS.map(|(_, _, ankle)| Sole::of(rig, ankle).points(stood, rig)[2]);
        let axes = [0, 1].map(|i| {
            let along = tips[i] - ankles[i];
            Vec3::Y.cross(Vec3::new(along.x, 0.0, along.z).normalize_or(rig.forward()))
        });
        let limbs = LEGS.map(|(socket, knee, ankle)| {
            let at = |bone| offset_from(stood, rig, Bone::Hips, bone);
            ((at(knee) - at(socket)).length(), (at(ankle) - at(knee)).length())
        });
        Self { ankles, tips, attitudes: LEGS.map(|(_, _, ankle)| world[ankle]), axes, limbs }
    }

    /// How far leg `i` reaches, socket to ankle, with its knee `knee`
    /// radians short of straight.
    fn reach(&self, i: usize, knee: f32) -> f32 {
        let (femur, shin) = self.limbs[i];
        (femur * femur + shin * shin + 2.0 * femur * shin * knee.cos()).sqrt()
    }

    /// Where foot `i`'s ankle is with its heel risen `angle` about its tip.
    fn ankle(&self, i: usize, angle: f32) -> Vec3 {
        self.tips[i] + Quat::from_axis_angle(self.axes[i], angle) * (self.ankles[i] - self.tips[i])
    }
}

/// Where the standing hips' frame puts `pose`'s hips joint: its root's
/// move from standing.
fn hips_of(pose: &LocalPose, stood: &LocalPose) -> Vec3 {
    pose.root_translation - stood.root_translation
}

/// `pose`'s COM in the standing hips' frame.
fn com_of(pose: &LocalPose, stood: &LocalPose, rig: &RigGeometry) -> Vec3 {
    hips_of(pose, stood) + centre_of_mass(pose, rig)
}

impl Jump {
    /// A jump `height` metres high (the COM's rise above take-off), from
    /// standing in `stood`. Clamped to [`HIGHEST`].
    pub fn plan(height: f32, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let height = height.clamp(0.02, HIGHEST);
        let feet = Feet::of(stood, rig);
        let com = com_of(stood, stood, rig);
        let stand = (com.dot(rig.forward()), com.y);
        let depth = (height * DEPTH_PER_HEIGHT).clamp(DEPTHS.0, DEPTHS.1);
        let mut jump = Self {
            t: 0.0,
            ends: [0.0; 5],
            stand,
            bottom: stand.1 - depth,
            takeoff: stand.1,
            touchdown: stand.1,
            lowest: stand.1 - depth,
            up: (2.0 * GRAVITY * height).sqrt(),
            down: 0.0,
            leaving: feet.ankles,
            meeting: feet.ankles,
            leaving_at: feet.ankles,
            meeting_at: feet.ankles,
            tuck: height * TUCK_PER_HEIGHT,
            arms: (height / FULL_ARMS).min(1.0).powi(2),
            feet,
        };

        // Take-off: on the toes, legs all but straight, arms up.
        let leaving = jump.upper(stood, rig, 0.0, jump.arm_shape(ARMS_UP));
        let leaving = jump.solved(&leaving, stood, rig, stand.0, Aim::Reach { heel: HEEL_RISE, knee: KNEE_AT_TAKEOFF });
        jump.takeoff = com_of(&leaving, stood, rig).y;
        jump.leaving = LEGS.map(|(_, _, ankle)| offset_from(&leaving, rig, Bone::Hips, ankle));
        jump.leaving_at = jump.leaving.map(|ankle| ankle + hips_of(&leaving, stood));
        // Touchdown: on the forefoot, knees a little bent, arms ahead; its
        // trunk leaning as its COM's height asks, should that be below
        // standing.
        let mut lean = 0.0;
        for _ in 0..2 {
            let meeting = jump.upper(stood, rig, lean, jump.arm_shape(ARMS_LANDING));
            let meeting = jump.solved(&meeting, stood, rig, stand.0, Aim::Reach { heel: LANDING_HEEL, knee: LANDING_KNEE });
            jump.touchdown = com_of(&meeting, stood, rig).y;
            jump.meeting = LEGS.map(|(_, _, ankle)| offset_from(&meeting, rig, Bone::Hips, ankle));
            jump.meeting_at = jump.meeting.map(|ankle| ankle + hips_of(&meeting, stood));
            lean = (LEAN_PER_DEPTH * (stand.1 - jump.touchdown).max(0.0)).min(MOST_LEAN);
        }

        // Down: Hermite at rest both ends; its steepest acceleration is
        // 6·depth/T² at the start, held to unloading the floor to LEAST_LOAD.
        let down_time = QUICKEST_DOWN.max((6.0 * depth / ((1.0 - LEAST_LOAD) * GRAVITY)).sqrt());
        // Push: from rest at constant acceleration to `up` over the path.
        let path = jump.takeoff - jump.bottom;
        let push_time = 2.0 * path / jump.up;
        // Flight: from take-off height to touchdown height.
        let drop = jump.takeoff - jump.touchdown;
        let flight_time = (jump.up + (jump.up * jump.up + 2.0 * GRAVITY * drop).sqrt()) / GRAVITY;
        jump.down = GRAVITY * flight_time - jump.up;
        // Land: constant deceleration to rest.
        let absorb = (jump.down * jump.down / (2.0 * LANDING_DECELERATION)).min(DEEPEST_LANDING);
        jump.lowest = jump.touchdown - absorb;
        let land_time = 2.0 * absorb / jump.down;
        // Recover: Hermite at rest both ends, steepest at its start.
        let rise = stand.1 - jump.lowest;
        let recover_time = QUICKEST_DOWN.max((6.0 * rise / RECOVERY_ACCELERATION).sqrt());
        let mut end = 0.0;
        for (slot, span) in [down_time, push_time, flight_time, land_time, recover_time].into_iter().enumerate() {
            end += span;
            jump.ends[slot] = end;
        }
        jump
    }

    /// Seconds since it began.
    pub fn elapsed(&self) -> f32 {
        self.t
    }

    /// How long the whole jump takes, seconds.
    pub fn duration(&self) -> f32 {
        self.ends[4]
    }

    /// When `phase` ends, seconds from the start.
    pub fn ends(&self, phase: JumpPhase) -> f32 {
        self.ends[phase as usize]
    }

    /// Moves the jump on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.t = (self.t + dt).min(self.duration());
    }

    /// Whether it has landed and stood up again.
    pub fn is_done(&self) -> bool {
        self.t >= self.duration()
    }

    /// The phase at `t` seconds in.
    pub fn phase_at(&self, t: f32) -> JumpPhase {
        use JumpPhase::*;
        [Down, Push, Flight, Land, Recover].into_iter().zip(self.ends).find(|&(_, end)| t < end).map_or(Recover, |(p, _)| p)
    }

    /// The phase now.
    pub fn phase(&self) -> JumpPhase {
        self.phase_at(self.t)
    }

    /// The COM's planned height `t` seconds in, in the standing hips' frame.
    pub fn com_height_at(&self, t: f32) -> f32 {
        let [down, push, flight, land, recover] = self.ends;
        let (from, span) = match self.phase_at(t) {
            JumpPhase::Down => (0.0, down),
            JumpPhase::Push => (down, push - down),
            JumpPhase::Flight => (push, flight - push),
            JumpPhase::Land => (flight, land - flight),
            JumpPhase::Recover => (land, recover - land),
        };
        let s = (t - from).clamp(0.0, span);
        match self.phase_at(t) {
            JumpPhase::Down => hermite(self.stand.1, self.bottom, 0.0, 0.0, s / span),
            JumpPhase::Push => self.bottom + 0.5 * (self.up / span) * s * s,
            JumpPhase::Flight => self.takeoff + self.up * s - 0.5 * GRAVITY * s * s,
            JumpPhase::Land => self.touchdown - self.down * s + 0.5 * (self.down / span) * s * s,
            JumpPhase::Recover => hermite(self.lowest, self.stand.1, 0.0, 0.0, s / span),
        }
    }

    /// Whether both feet are in the air.
    pub fn airborne(&self) -> bool {
        self.phase() == JumpPhase::Flight
    }

    /// How far into the phase at `t` it is, 0-1.
    fn progress_at(&self, t: f32) -> f32 {
        let phase = self.phase_at(t) as usize;
        let from = if phase == 0 { 0.0 } else { self.ends[phase - 1] };
        ((t - from) / (self.ends[phase] - from)).clamp(0.0, 1.0)
    }

    /// The trunk's lean (radians forward) and the arms' (swing, elbow) at
    /// `t` seconds in.
    fn shape_at(&self, t: f32) -> (f32, (f32, f32)) {
        let below = (self.stand.1 - self.com_height_at(t)).max(0.0);
        let lean = (LEAN_PER_DEPTH * below).min(MOST_LEAN);
        let u = self.progress_at(t);
        // Each pair is (swing, elbow), from one held arm shape to the next.
        let between = |from: (f32, f32), to: (f32, f32), s: f32| (from.0 + (to.0 - from.0) * s, from.1 + (to.1 - from.1) * s);
        let arms = match self.phase_at(t) {
            JumpPhase::Down => between((0.0, 0.0), ARMS_BACK, (below / (self.stand.1 - self.bottom)).min(1.0)),
            JumpPhase::Push => between(ARMS_BACK, ARMS_UP, smoothstep((u * 1.25).min(1.0))),
            JumpPhase::Flight => between(ARMS_UP, ARMS_LANDING, smoothstep(u)),
            JumpPhase::Land => between(ARMS_LANDING, ARMS_LANDED, u),
            JumpPhase::Recover => between(ARMS_LANDED, (0.0, 0.0), smoothstep(u)),
        };
        (lean, self.arm_shape(arms))
    }

    /// An arm shape (swing, elbow) as far as this jump's strength swings
    /// the arms.
    fn arm_shape(&self, (swing, elbow): (f32, f32)) -> (f32, f32) {
        (swing * self.arms, elbow * self.arms)
    }

    /// The pose `t` seconds in, from standing in `stood`.
    pub fn pose_at(&self, t: f32, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let height = self.com_height_at(t);
        let u = self.progress_at(t);
        let (lean, arms) = self.shape_at(t);
        let upper = self.upper(stood, rig, lean, arms);
        if self.phase_at(t) == JumpPhase::Flight {
            let blend = smoothstep(u);
            let tuck = Vec3::Y * self.tuck * (std::f32::consts::PI * u).sin();
            let pitch = HEEL_RISE + (LANDING_HEEL - HEEL_RISE) * blend;
            // Each ankle's height under the hips from its shape, but its
            // way across the floor held in the world, from where it left to
            // where it lands: the pelvis moves under the arms' swing to keep
            // the COM on its path, and the feet carried with it came down
            // moving 2 mm a frame across, 8 mm the frame they landed.
            let across = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
            let ways = [0, 1].map(|i| across(self.leaving_at[i].lerp(self.meeting_at[i], blend)));
            let mut root = upper.root_translation;
            let mut pose = upper;
            for _ in 0..6 {
                let mut trial = upper;
                trial.root_translation = root;
                let hips = hips_of(&trial, stood);
                let ankles = [0, 1].map(|i| {
                    let shape = self.leaving[i].lerp(self.meeting[i], blend) + tuck;
                    ways[i] - across(hips) + Vec3::Y * shape.y
                });
                pose = self.legs(&trial, stood, rig, Legs::Free { ankles, pitch });
                // Nothing holds it: the whole body moves with the COM.
                let com = com_of(&pose, stood, rig);
                let miss = rig.forward() * (self.stand.0 - com.dot(rig.forward())) + Vec3::Y * (height - com.y);
                if miss.length() < 1.0e-6 {
                    break;
                }
                root += miss;
            }
            pose
        } else {
            let phase = self.phase_at(t);
            self.solved(&upper, stood, rig, self.stand.0, Aim::Height { height, knee: least_knee(phase), heel: heel_at(phase, u) })
        }
    }

    /// Where its plan has each foot now, in the pose's frame: its ball
    /// rolling about its tip as the heel rises or comes down (its height
    /// above where it stands flat), the tip where it stood.
    pub fn touchdown(&self, stood: &LocalPose, rig: &RigGeometry) -> super::plugin::Touchdown {
        let pose = self.pose(stood, rig);
        let (at, flat) = (super::rig::forward_kinematics_on(&pose, rig), super::rig::forward_kinematics_on(stood, rig));
        let toe = |bone: Bone| Vec3::new(at[bone].x, at[bone].y - flat[bone].y, at[bone].z);
        super::plugin::Touchdown {
            toes: [toe(Bone::LeftToeBase), toe(Bone::RightToeBase)],
            // The point the tip lock holds (`legik::toe_tip`): the toe's end,
            // not the sole's tip under it, which a pitched foot puts
            // elsewhere across the floor.
            tips: [super::legik::LegChain::LEFT, super::legik::LegChain::RIGHT]
                .map(|chain| super::legik::toe_tip(&pose, chain, rig).map_or(Vec3::ZERO, |(tip, _)| tip)),
        }
    }

    /// The pose now.
    pub fn pose(&self, stood: &LocalPose, rig: &RigGeometry) -> LocalPose {
        self.pose_at(self.t, stood, rig)
    }

    /// [`Self::pose_at`] led ahead of `springs`: each bone's rotation taken
    /// from the plan its spring's lag ahead ([`lead_of`]), the root (not
    /// sprung) from `t`. Sprung, the rendered body is then close to the
    /// plan at `t`.
    ///
    /// Unled, the springs trailed it: the arms reached shoulder height at
    /// take-off where the plan had them 25° above, and the legs, still
    /// straightening, put the rendered COM 30 mm off the parabola in
    /// flight. The trunk and arms are led from their shape alone
    /// ([`Self::shape_at`]); a whole pose, pelvis solved, for every lead cost
    /// 91 µs a character a frame.
    pub fn pose_led(&self, t: f32, stood: &LocalPose, rig: &RigGeometry, springs: &super::rig::BoneSet<SpringParams>) -> LocalPose {
        const LED: [Bone; 13] = [
            Bone::Spine,
            Bone::Spine1,
            Bone::Spine2,
            Bone::Neck,
            Bone::Head,
            Bone::LeftShoulder,
            Bone::LeftArm,
            Bone::LeftForeArm,
            Bone::LeftHand,
            Bone::RightShoulder,
            Bone::RightArm,
            Bone::RightForeArm,
            Bone::RightHand,
        ];
        let mut posed: Vec<(f32, bool, LocalPose)> = Vec::with_capacity(4);
        let mut pose = self.pose_at(t, stood, rig);
        // Not past the phase it is in: there the plan's motion turns at
        // once (a landing), and led across it, the feet came down 8.1 then
        // 16.1 mm off their path the last frames before they landed.
        let end = self.ends[self.phase_at(t) as usize];
        for bone in Bone::ALL {
            let lead = lead_of(&springs[bone]).min(end - t);
            if lead <= 1.0e-4 {
                continue;
            }
            // The trunk and arms from their shape alone; the legs and hips
            // need the pelvis solved there too.
            let whole = !LED.contains(&bone);
            let at = match posed.iter().find(|(l, w, _)| (l - lead).abs() < 1.0e-4 && *w == whole) {
                Some((_, _, ahead)) => ahead.rotations[bone],
                None => {
                    let ahead = if whole {
                        self.pose_at(t + lead, stood, rig)
                    } else {
                        let (lean, arms) = self.shape_at(t + lead);
                        self.upper(stood, rig, lean, arms)
                    };
                    posed.push((lead, whole, ahead));
                    ahead.rotations[bone]
                }
            };
            pose.rotations[bone] = at;
        }
        pose
    }

    /// [`Self::pose_led`] now.
    pub fn pose_now_led(&self, stood: &LocalPose, rig: &RigGeometry, springs: &super::rig::BoneSet<SpringParams>) -> LocalPose {
        self.pose_led(self.t, stood, rig, springs)
    }

    /// `stood` with the trunk leaning `lean` (radians forward) and the arms
    /// swung `arms` (radians forward from hanging).
    fn upper(&self, stood: &LocalPose, rig: &RigGeometry, lean: f32, (swing, elbow): (f32, f32)) -> LocalPose {
        let mut pose = *stood;
        let left = rig.left();
        for (bone, angle) in [(Bone::Hips, lean * PELVIS_SHARE), (Bone::Spine, lean * (1.0 - PELVIS_SHARE)), (Bone::Neck, -0.6 * lean)] {
            pose.rotations[bone] = delta_after_world_turn(&pose, rig, bone, Quat::from_axis_angle(left, angle));
        }
        // The upper arm swung, then the forearm bent forward on it.
        for (bone, angle) in [(Bone::LeftArm, swing), (Bone::RightArm, swing), (Bone::LeftForeArm, elbow), (Bone::RightForeArm, elbow)] {
            pose.rotations[bone] = delta_after_world_turn(&pose, rig, bone, Quat::from_axis_angle(left, -angle));
        }
        pose
    }

    /// `upper` standing on its feet with its COM at `ahead` along the
    /// rig's forward and at `aim`'s height, the knees kept at least `knee`
    /// short of straight.
    fn solved(&self, upper: &LocalPose, stood: &LocalPose, rig: &RigGeometry, ahead: f32, aim: Aim) -> LocalPose {
        let forward = rig.forward();
        let (knee, heel) = match aim {
            Aim::Height { knee, heel, .. } | Aim::Reach { knee, heel } => (knee, heel),
        };
        let mut shift = Vec3::ZERO;
        let mut pose = *upper;
        // The COM moves with the pelvis less what the legs keep back: about
        // 0.85 of it. A few steps settle it.
        for _ in 0..8 {
            let mut trial = *upper;
            trial.root_translation += shift;
            if let Aim::Reach { heel, knee } = aim {
                trial.root_translation.y += self.highest(&trial, stood, rig, heel, knee);
            }
            pose = self.legs(&trial, stood, rig, Legs::Down { knee, heel });
            let com = com_of(&pose, stood, rig);
            let up = match aim {
                Aim::Height { height, .. } => height - com.y,
                Aim::Reach { .. } => 0.0,
            };
            let miss = Vec3::Y * up + forward * (ahead - com.dot(forward));
            if miss.length() < 1.0e-5 {
                break;
            }
            shift += miss / 0.85;
        }
        pose
    }

    /// How far `pose`'s root may rise for its legs to just reach their
    /// ankles with the heels risen `heel` about the toe tips and the knees
    /// `knee` short of straight.
    fn highest(&self, pose: &LocalPose, stood: &LocalPose, rig: &RigGeometry, heel: f32, knee: f32) -> f32 {
        let hips = hips_of(pose, stood);
        (0..2)
            .map(|i| {
                let socket = hips + offset_from(pose, rig, Bone::Hips, LEGS[i].0);
                let ankle = self.feet.ankle(i, heel);
                let across = Vec3::new(ankle.x - socket.x, 0.0, ankle.z - socket.z).length_squared();
                ankle.y + (self.feet.reach(i, knee).powi(2) - across).max(0.0).sqrt() - socket.y
            })
            .fold(f32::MAX, f32::min)
    }

    /// `pose` with its legs set: each ankle placed, and each foot turned to
    /// its attitude.
    fn legs(&self, pose: &LocalPose, stood: &LocalPose, rig: &RigGeometry, legs: Legs) -> LocalPose {
        let mut pose = *pose;
        let hips = hips_of(&pose, stood);
        for (i, &(socket, _, ankle)) in LEGS.iter().enumerate() {
            let (target, pitch) = match legs {
                Legs::Down { knee, heel } => {
                    let at = hips + offset_from(&pose, rig, Bone::Hips, socket);
                    let short = |angle: f32| (self.feet.ankle(i, angle) - at).length() - self.feet.reach(i, knee);
                    // The least heel rise from `heel` that brings the ankle
                    // into reach.
                    let angle = if short(heel) <= 0.0 {
                        heel
                    } else if short(HEEL_RISE) > 0.0 {
                        HEEL_RISE
                    } else {
                        let (mut low, mut high) = (heel, HEEL_RISE);
                        for _ in 0..20 {
                            let middle = 0.5 * (low + high);
                            if short(middle) > 0.0 { low = middle } else { high = middle }
                        }
                        high
                    };
                    (self.feet.ankle(i, angle) - hips, angle)
                }
                Legs::Free { ankles, pitch } => (ankles[i], pitch),
            };
            place_ankle(&mut pose, rig, ankle, target);
            let now = accumulate_world_rotations(&pose, rig)[ankle];
            let wanted = Quat::from_axis_angle(self.feet.axes[i], pitch) * self.feet.attitudes[i];
            pose.rotations[ankle] = delta_after_world_turn(&pose, rig, ankle, wanted * now.inverse());
        }
        pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

    fn real_stood() -> (LocalPose, RigGeometry) {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    const DT: f32 = 1.0 / 240.0;

    /// The COM's height on the plan and on the pose itself, every DT.
    fn sampled(jump: &Jump, stood: &LocalPose, rig: &RigGeometry) -> Vec<(f32, JumpPhase, f32, Vec3)> {
        let steps = (jump.duration() / DT).ceil() as usize;
        (0..=steps)
            .map(|n| {
                let t = (n as f32 * DT).min(jump.duration());
                let pose = jump.pose_at(t, stood, rig);
                (t, jump.phase_at(t), jump.com_height_at(t), com_of(&pose, stood, rig))
            })
            .collect()
    }

    #[test]
    fn the_pose_carries_its_com_on_the_planned_path() {
        let (stood, rig) = real_stood();
        let jump = Jump::plan(0.35, &stood, &rig);
        let forward = rig.forward();
        for (t, phase, planned, com) in sampled(&jump, &stood, &rig) {
            assert!(
                (com.y - planned).abs() < 1.0e-3 && (com.dot(forward) - jump.stand.0).abs() < 1.0e-3,
                "at {t:.3} s ({phase:?}) the COM is at {:.4} up, {:.4} ahead; planned {planned:.4}, {:.4}",
                com.y,
                com.dot(forward),
                jump.stand.0
            );
        }
    }

    #[test]
    fn it_flies_at_g_to_the_height_asked() {
        let (stood, rig) = real_stood();
        for height in [0.1, 0.35, 0.5] {
            let jump = Jump::plan(height, &stood, &rig);
            let apex = sampled(&jump, &stood, &rig).iter().map(|s| s.3.y).fold(f32::MIN, f32::max);
            assert!((apex - jump.takeoff - height).abs() < 2.0e-3, "{height} m asked: rose {}", apex - jump.takeoff);
            // The COM's acceleration over flight, from the pose.
            let (start, end) = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
            let y = |t: f32| com_of(&jump.pose_at(t, &stood, &rig), &stood, &rig).y;
            let mut t = start + 0.02;
            while t < end - 0.02 {
                let h = 0.01;
                let a = (y(t + h) - 2.0 * y(t) + y(t - h)) / (h * h);
                assert!((a + GRAVITY).abs() < 0.3, "{height} m: at {t:.3} s in flight the COM accelerates {a}");
                t += 0.02;
            }
        }
    }

    /// The floor's push, in body weights, is 1 + the COM's acceleration over
    /// g. Down 0.43 at the least (McHugh et al. 2024); the push 1.8 on
    /// average with peaks of 2.2-2.5 measured (McMahon et al. 2017); the
    /// landing 2 on average. Durations: down 0.5-0.8 s, the push 0.30-0.34 s
    /// for a 0.3-0.35 m jump (McHugh 2024; McInnis & Donahue 2024).
    #[test]
    fn the_floor_pushes_as_a_person_can() {
        let (stood, rig) = real_stood();
        let jump = Jump::plan(0.35, &stood, &rig);
        let h = 1.0e-3;
        let y = |t: f32| jump.com_height_at(t);
        let mut t = h;
        let (mut least, mut most_push, mut most_land) = (f32::MAX, 0.0f32, 0.0f32);
        while t < jump.duration() - h {
            let phase = jump.phase_at(t);
            if phase != JumpPhase::Flight && jump.phase_at(t - h) == phase && jump.phase_at(t + h) == phase {
                let load = 1.0 + (y(t + h) - 2.0 * y(t) + y(t - h)) / (h * h) / GRAVITY;
                least = least.min(load);
                match phase {
                    JumpPhase::Push => most_push = most_push.max(load),
                    JumpPhase::Land => most_land = most_land.max(load),
                    _ => {}
                }
            }
            t += h;
        }
        let down = jump.ends(JumpPhase::Down);
        let push = jump.ends(JumpPhase::Push) - down;
        assert!(least > LEAST_LOAD - 0.02, "the floor carried {least} body weights at the least");
        assert!((1.6..2.6).contains(&most_push), "pushed at {most_push} body weights");
        assert!((1.6..2.6).contains(&most_land), "landed at {most_land} body weights");
        assert!((0.5..0.8).contains(&down), "down in {down} s");
        assert!((0.25..0.4).contains(&push), "pushed in {push} s");
    }

    #[test]
    fn the_feet_stay_where_they_stood_and_never_go_through_the_floor() {
        let (stood, rig) = real_stood();
        let jump = Jump::plan(0.35, &stood, &rig);
        let soles = LEGS.map(|(_, _, ankle)| Sole::of(&rig, ankle));
        let points = |pose: &LocalPose| [0, 1].map(|i| soles[i].points(pose, &rig).map(|p| p + hips_of(pose, &stood)));
        let standing = points(&stood);
        let floor = standing.iter().flatten().map(|p| p.y).fold(f32::MAX, f32::min);
        let mut highest_in_flight = 0.0f32;
        for (t, phase, _, _) in sampled(&jump, &stood, &rig) {
            let pose = jump.pose_at(t, &stood, &rig);
            let now = points(&pose);
            for (i, foot) in now.iter().enumerate() {
                for (k, p) in foot.iter().enumerate() {
                    assert!(p.y > floor - 1.0e-3, "at {t:.3} s ({phase:?}) foot {i} point {k} is {} mm under the floor", (floor - p.y) * 1e3);
                }
                if phase != JumpPhase::Flight {
                    // The tip never moves; the heel and ball only rise.
                    let tip = foot[2] - standing[i][2];
                    assert!(tip.length() < 1.0e-3, "at {t:.3} s ({phase:?}) foot {i}'s tip moved {} mm", tip.length() * 1e3);
                } else {
                    highest_in_flight = highest_in_flight.max(foot[2].y - floor);
                }
            }
        }
        assert!(highest_in_flight > 0.3, "the toes rose only {highest_in_flight} m");
        // On the toes as it leaves: the heels risen HEEL_RISE about the tips.
        let leaving = points(&jump.pose_at(jump.ends(JumpPhase::Push) - 1.0e-4, &stood, &rig));
        for (i, foot) in leaving.iter().enumerate() {
            let heel = foot[0].y - standing[i][0].y;
            let expected = (standing[i][2] - standing[i][0]).length() * HEEL_RISE.sin();
            assert!((heel - expected).abs() < 0.01, "foot {i}'s heel left the floor {heel} m up, not {expected}");
        }
    }

    /// The knees fold forward all through (`knee_fold_direction` negative:
    /// an unsigned angle cannot tell a backward knee), 90-110° at the
    /// bottom of the countermovement (80-100° of flexion is the protocols'
    /// norm), all but straight leaving the floor, and no more than 125°
    /// landing (117° in DeVita & Skelly 1992's soft landings).
    #[test]
    fn the_knees_bend_as_a_jumpers_do() {
        use crate::character::anim::rig::{forward_kinematics_on, Side};
        let (stood, rig) = real_stood();
        let jump = Jump::plan(0.35, &stood, &rig);
        let flexion = |t: f32| {
            let p = forward_kinematics_on(&jump.pose_at(t, &stood, &rig), &rig);
            180.0 - (p[Bone::LeftUpLeg] - p[Bone::LeftLeg]).angle_between(p[Bone::LeftFoot] - p[Bone::LeftLeg]).to_degrees()
        };
        for (t, phase, _, _) in sampled(&jump, &stood, &rig) {
            let pose = jump.pose_at(t, &stood, &rig);
            for side in [Side::Left, Side::Right] {
                let fold = rig.knee_fold_direction(&pose, side);
                assert!(fold < 0.0, "at {t:.3} s ({phase:?}) the {side:?} knee folds backward ({fold})");
            }
        }
        let raised = jump.takeoff - jump.stand.1;
        assert!((0.09..0.12).contains(&raised), "the COM leaves {raised} m above standing");
        let bottom = flexion(jump.ends(JumpPhase::Down));
        let leaving = flexion(jump.ends(JumpPhase::Push) - 1.0e-4);
        let landed = flexion(jump.ends(JumpPhase::Land));
        assert!((90.0..110.0).contains(&bottom), "{bottom}° at the bottom");
        assert!(leaving < 10.0, "{leaving}° leaving the floor");
        assert!(landed < 125.0, "{landed}° at the bottom of the landing");
    }

    /// Through the springs at 60 Hz, as the plugin renders it: the worst
    /// gap between the rendered and planned COM, the upper arm's angle at
    /// take-off off the plan's, and the feet's worst horizontal gap from the
    /// plan's over the flight's last frames.
    fn rendered_gaps(led: bool) -> (f32, f32) {
        use crate::character::anim::dho::{default_springs, DhoState};
        use crate::character::anim::rig::forward_kinematics_on;
        let (stood, rig) = real_stood();
        let springs = default_springs();
        let jump = Jump::plan(0.35, &stood, &rig);
        let dt = 1.0 / 60.0;
        let mut dho = DhoState::settled_on(&stood);
        let (mut com_gap, mut arm_gap) = (0.0f32, 0.0f32);
        let takeoff = jump.ends(JumpPhase::Push);
        let mut t = 0.0;
        while t < jump.duration() {
            t += dt;
            let target = if led { jump.pose_led(t, &stood, &rig, &springs) } else { jump.pose_at(t, &stood, &rig) };
            dho.advance(&target, &springs, dt);
            let rendered = dho.pose(target.root_translation);
            let planned = jump.pose_at(t, &stood, &rig);
            if jump.phase_at(t) == JumpPhase::Flight {
                com_gap = com_gap.max((com_of(&rendered, &stood, &rig) - com_of(&planned, &stood, &rig)).length());
            }
            if (t - takeoff).abs() < dt * 0.5 {
                let (r, p) = (forward_kinematics_on(&rendered, &rig), forward_kinematics_on(&planned, &rig));
                let arm = |at: &crate::character::anim::rig::BoneSet<Vec3>| (at[Bone::LeftForeArm] - at[Bone::LeftArm]).normalize();
                arm_gap = arm(&r).angle_between(arm(&p)).to_degrees();
            }
        }
        (com_gap, arm_gap)
    }

    /// Led ahead of the springs (`pose_led`), the body the springs render
    /// keeps to the plan where nothing else holds it: in flight, and the
    /// arms at take-off. (Down, the foot IK re-solves the legs onto the
    /// feet from the unsprung pelvis.) Measured: the COM 6.6 mm off the
    /// parabola (30.4 unled, the legs still straightening), the upper arm
    /// 3.7° off at take-off (16.9).
    #[test]
    fn led_ahead_of_its_springs_the_rendered_body_keeps_to_the_plan() {
        let (com, arm) = rendered_gaps(true);
        let (com_unled, arm_unled) = rendered_gaps(false);
        assert!(com < 0.015 && arm < 6.0, "led: COM {:.1} mm off in flight, arm {arm:.1}° at take-off", com * 1e3);
        assert!(com_unled > 1.5 * com && arm_unled > 2.0 * arm, "unled no worse: COM {com_unled}, arm {arm_unled}");
    }

    /// In flight each foot keeps its way across the floor, from where it
    /// left to where it lands, while the pelvis moves under the arms' swing:
    /// carried with the pelvis, the feet moved up to 3.9 mm a frame across,
    /// and live came down 8 mm off the frame they landed.
    #[test]
    fn in_flight_the_feet_come_straight_down_onto_their_spots() {
        let (stood, rig) = real_stood();
        let jump = Jump::plan(0.35, &stood, &rig);
        let (start, end) = (jump.ends(JumpPhase::Push), jump.ends(JumpPhase::Flight));
        let ankles = |t: f32| {
            let pose = jump.pose_at(t, &stood, &rig);
            LEGS.map(|(_, _, ankle)| offset_from(&pose, &rig, Bone::Hips, ankle) + hips_of(&pose, &stood))
        };
        let dt = 1.0 / 60.0;
        let mut t = start + 1.0e-4;
        while t + dt < end {
            let (now, next) = (ankles(t), ankles(t + dt));
            for i in 0..2 {
                // The way from the take-off's ankle (heel up) to the
                // landing's, smoothstepped: never more than ~1 mm a frame.
                let across = Vec3::new(next[i].x - now[i].x, 0.0, next[i].z - now[i].z).length();
                assert!(across < 1.0e-3, "at {t:.3} s ankle {i} moved {:.2} mm across the floor in a frame", across * 1e3);
            }
            t += dt;
        }
    }

    /// The arms swing as hard as the jump: up to [`FULL_ARMS`] by the
    /// square of its height's share, a small hop hardly moving them.
    #[test]
    fn the_arms_swing_as_hard_as_the_jump() {
        use crate::character::anim::rig::forward_kinematics_on;
        let (stood, rig) = real_stood();
        let upper_arm = |pose: &LocalPose| {
            let at = forward_kinematics_on(pose, &rig);
            (at[Bone::LeftForeArm] - at[Bone::LeftArm]).normalize()
        };
        let hanging = upper_arm(&stood);
        // How far the upper arm has swung from hanging as it leaves the floor.
        let swung = |height: f32| {
            let jump = Jump::plan(height, &stood, &rig);
            upper_arm(&jump.pose_at(jump.ends(JumpPhase::Push) - 1.0e-4, &stood, &rig)).angle_between(hanging)
        };
        let (hop, half, jump, high) = (swung(0.1), swung(0.2), swung(FULL_ARMS), swung(0.5));
        assert!((jump - ARMS_UP.0).abs() < 0.05, "a full jump swings the arms {jump} rad, not {}", ARMS_UP.0);
        let share = |h: f32| (h / FULL_ARMS).powi(2) * ARMS_UP.0;
        assert!((hop - share(0.1)).abs() < 0.05, "a 0.1 m hop swings them {hop} rad, not {}", share(0.1));
        assert!((half - share(0.2)).abs() < 0.05, "a 0.2 m jump swings them {half} rad, not {}", share(0.2));
        assert!((high - jump).abs() < 1.0e-3, "higher than FULL_ARMS they swing no further: {high} against {jump}");
    }

    #[test]
    fn it_starts_and_ends_standing() {
        let (stood, rig) = real_stood();
        let jump = Jump::plan(0.35, &stood, &rig);
        for t in [0.0, jump.duration()] {
            let pose = jump.pose_at(t, &stood, &rig);
            assert!((pose.root_translation - stood.root_translation).length() < 1.0e-3, "root moved at {t}");
            for bone in crate::character::skeleton::Bone::ALL {
                assert!(1.0 - pose.rotations[bone].dot(stood.rotations[bone]).abs() < 1.0e-6, "{bone:?} turned at {t}");
            }
        }
    }
}
