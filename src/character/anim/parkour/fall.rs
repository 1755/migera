//! Falling off an edge and landing from height: step 4 of the parkour
//! design.
//!
//! The body leaves an edge with the velocity it had (walking off a top, or
//! later letting go of a hang); its hips fall under gravity until the feet
//! meet the ground below, the legs reaching from where they were to the
//! landing's shape, knees 25° bent (20-29° at contact, every technique: Dai
//! et al. 2020). Then it lands:
//!
//! - **Down**: the hips go from the touchdown speed to rest at the deepest
//!   point in the measured time (squat landings from 0.9, 1.8 and 2.7 m:
//!   377, 335 and 290 ms) as deep as the measured most knee flexion puts them
//!   on this rig's legs (116°, 126°, 134°), the braking rising to a peak
//!   about 65 ms in. The forward speed is braked over the same time, the
//!   feet planted where the hips come to rest over them; the trunk leans
//!   forward and the arms reach forward as it goes down.
//! - **Up**: back to standing, at the foot IK's drop.
//!
//! From 0.9 m the hips take about 4 body weights (a parkour landing peaks
//! at 2.9-3.2 from 0.75 m, a stiff one at 5.2: Puddle and Maulder 2013).
//! Past [`ROLL_DROP`] it rolls over a shoulder instead; past [`FATAL_DROP`]
//! it does not land (the walker hands the body to the ragdoll).

use bevy::math::{Quat, Vec3};

use super::Ledge;
use crate::character::anim::armik::{solve_arm_toward_from, ArmChain};
use crate::character::anim::gait::smoothstep;
use crate::character::anim::jump::{GRAVITY, RECOVERY_ACCELERATION};
use crate::character::anim::rig::{accumulate_world_rotations, delta_after_world_turn, forward_kinematics_on, BoneSet, LocalPose, RigGeometry};
use crate::character::anim::stance::place_ankle;
use crate::character::skeleton::Bone;

/// Each leg's socket, knee, ankle and toe: left, right.
const LEGS: [(Bone, Bone, Bone, Bone); 2] =
    [(Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase), (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase)];

/// Dropping from these heights, metres, a squat landing takes this long,
/// seconds, and flexes the knees at most this far, degrees (Dai et al.
/// 2020). Keyed by the drop, not the touchdown speed the study gives
/// (3.0, 4.9, 6.3 m/s): free fall from 0.9 m touches down at 4.2 m/s, and
/// those speeds do not scale as the drops' square roots.
const LANDINGS: [(f32, f32, f32); 3] = [(0.9, 0.377, 116.0), (1.8, 0.335, 126.0), (2.7, 0.290, 134.0)];
/// The knees' flexion as the feet touch down, degrees (20-29° measured).
const KNEE_AT_CONTACT: f32 = 25.0;
/// The ground a walker steps down onto without falling, metres: deeper, it
/// falls.
pub const STEP_DOWN: f32 = 0.3;
/// The legs come to the landing's shape over this share of the flight, at
/// least this long, seconds.
const LEGS_REACH: f32 = 0.25;
/// Leaving a jump, the legs coast on their own swing this long, seconds;
/// the trunk's and arms' swing is read off this far ahead in the jump
/// (over 75 ms, a hand's curved swing left 1.1 cm a frame off).
const LEGS_COAST: f32 = 0.15;
/// Turning round in the air, begun this long after leaving, seconds, and
/// done within this share of the flight.
const SPIN_AFTER: f32 = 0.1;
const SPIN_IN_FLIGHT: f32 = 0.8;
/// The flight is searched for a top to land on this finely, seconds.
const LAND_STEP: f32 = 0.01;
/// A foot on a top: from this far behind its ankle to this far ahead (the
/// ball), and this far either side, metres.
const FOOT_HEEL: f32 = 0.05;
const FOOT_BALL: f32 = 0.12;
const FOOT_SIDE: f32 = 0.05;
/// A roll's way along the ground is checked every this far, and this far
/// either side of it (the tucked body's half width), metres.
const ROLL_ROOM_STEP: f32 = 0.1;
const ROLL_HALF_WIDTH: f32 = 0.25;
const AHEAD: f32 = 0.01;
/// The trunk leans forward this much a metre of the landing's depth,
/// radians; flying, this much.
const LEAN_PER_DEPTH: f32 = 1.1;
const FLYING_LEAN: f32 = 0.05;
/// The arms flying (out and up for balance), and at the bottom of the
/// landing (forward): swing and elbow bend, radians (`jump::upper`).
const ARMS_FLYING: (f32, f32) = (0.9, 0.5);
const ARMS_LANDING: (f32, f32) = (0.9, 0.7);
/// Reaching up to catch a ledge falling past, the arms up overhead (swing
/// and elbow bend, radians, `jump::upper`).
const ARMS_REACHING: (f32, f32) = (2.6, 0.15);
/// Landed, the arms go from the flight's to the landing's over this long,
/// seconds.
const ARMS_DOWN: f32 = 0.15;
/// The trunk and arms go from where they were as it left to the flight's
/// over this long, seconds.
const ARMS_FREE: f32 = 0.35;
/// Facing a wall, how far the wrists keep off it, metres: a hand's length,
/// its fingers up the wall.
const HANDS_OFF_WALL: f32 = 0.1;
/// ... over this far under its top, metres; and the knees in the air.
const WALL_EASED: f32 = 0.2;
const KNEE_OFF_WALL: f32 = 0.02;
/// The ankle moved out this many times to bring its knee off a wall: each
/// brings the knee about half the way (twice, a knee driven up on a wall
/// kicked off, spun to face the lip, stayed 1.5 cm in; six, 0.1 mm).
const KNEE_PASSES: usize = 6;
/// Sliding down a wall: the share of gravity the hands and feet brake off
/// (no measured data: landing 0.71 of free fall's speed); the wrists held
/// this far off the face, metres (the palm on it); the feet braced on the
/// face for this share of the flight.
const SLIDE_BRAKE: f32 = 0.5;
const SLIDE_HANDS: f32 = 0.04;
/// Sliding, the hands on the face at most this far above their shoulders,
/// metres, brought down to that over this long from letting go, seconds,
/// and let go of the face over this long once landed, seconds.
const SLIDE_ABOVE: f32 = 0.25;
const SLIDE_HANDS_IN: f32 = 0.35;
const SLIDE_LET_GO: f32 = 0.25;
/// Sliding, the elbows point down, this much out from the face and this
/// much out to their own sides for each unit down.
const SLIDE_ELBOW: (f32, f32) = (0.4, 0.6);
const SLIDE_POLE_IN: f32 = 0.1;
const SLIDE_FEET: f32 = 0.6;
/// Falling into a wall it faces, the hips stop this far out from it,
/// metres (standing facing it, the hips are about this far off it),
/// slowing from this much and as far again further out: the arms taking
/// it, about 2.3 g from 3 m/s (over 0.1 m, a jump falling short at 3 m/s
/// stopped at 7.5 g); landing, the room the landing's shape needs this
/// much more.
const HIPS_OFF_WALL: f32 = 0.25;
const WALL_GIVE: f32 = 0.2;
/// The least the give is cut to, leaving near the wall, metres.
const MIN_WALL_GIVE: f32 = 0.05;
const ROOM_MARGIN: f32 = 0.05;
/// A wall faced within this of where it would come to rest is fallen
/// against, metres: a landing's shape reaches 0.65 m ahead of the hips.
const WALL_NEAR: f32 = 1.0;
/// Landing facing it, the hips' room is planned every this many seconds,
/// held up and averaged over this long either side.
const ROOM_STEP: f32 = 0.01;
const ROOM_WINDOW: f32 = 0.08;
/// The trunk's and arms' bones: let go from a hang, the trunk leant toward
/// the wall would snap upright. And the thighs and shins, whose knee hinge
/// the ankles are placed keeping: a running jump's take-off knee, turned
/// at once to standing's hinge, moved 2.8 cm in the frame it fell.
const FREED_BONES: [Bone; 18] = [
    Bone::LeftUpLeg,
    Bone::LeftLeg,
    Bone::RightUpLeg,
    Bone::RightLeg,
    Bone::Hips,
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
/// Falling past a ledge, the hands catch it once its lip comes within this
/// share of the arm's length of the shoulders' middle, above them.
const CATCH_REACH: f32 = 0.9;
/// The frame's way past the lip is looked along at this many points.
const CATCH_SWEEP: usize = 8;
/// The quickest a landing rises back to standing, seconds.
const QUICKEST_UP: f32 = 0.5;
/// Dropping further than this, metres, it rolls: the guidance is to roll
/// above about standing height (not peer reviewed); a squat landing from
/// 1.8 m loads the hips with 6.5 body weights.
pub const ROLL_DROP: f32 = 1.7;
/// Dropping further than this, metres, it does not land: the body goes to
/// the ragdoll at touchdown (no data; past where the landings measured,
/// 2.7 m, their loads climb steeply).
pub const FATAL_DROP: f32 = 4.0;
/// Moving on fast, it rolls from lower: a squat landing brakes the forward
/// speed over its own time, and harder than this, m/s², it rolls (from
/// 1.0 m at 3 m/s, 8 m/s², the feet planted 0.57 m ahead); but not from
/// lower than this, metres (the lowest drop the landings measured). No
/// data: set by eye.
const MOST_BRAKING: f32 = 6.0;
const ROLL_LOW: f32 = 0.75;
/// Dropping further than this, metres, and not so far it is fatal, it lands
/// hurt: deeper (the knees to this, degrees), down onto its hands, held
/// down this long, seconds, and up this much slower. No data: past the
/// measured 2.7 m, set by eye.
pub const HURT_DROP: f32 = 3.0;
const HURT_KNEE: f32 = 140.0;
const HURT_HOLD: f32 = 0.8;
const HURT_SLOWER: f32 = 2.0;
/// Hurt, the trunk leans this much a metre of depth, radians: far enough
/// over for the hands to reach the ground under the shoulders.
const HURT_LEAN_PER_DEPTH: f32 = 2.6;
/// A hand planted on the ground: its wrist this high, metres.
const WRIST_ON_GROUND: f32 = 0.04;
/// Rolling: the touchdown taken by the squat landing's first part (the feet
/// planted, the knees giving), seconds; then tucking, the turn rising to the
/// roll's, seconds; the turn's easing off as it comes up, seconds; and
/// coming up to standing, seconds. Tucking from the touchdown's straight
/// legs at once, the body stalled on its feet (5.9 m/s down to 0.1 in a
/// frame).
const ROLL_ABSORB: f32 = 0.15;
const ROLL_TUCK: f32 = 0.2;
const ROLL_EASE: f32 = 0.2;
const ROLL_UP: f32 = 0.9;
/// Rolling, at least this fast, m/s, and as fast as this share of the
/// touchdown speed turned forward.
const ROLL_SLOWEST: f32 = 2.0;
const ROLL_FROM_DOWN: f32 = 0.45;
/// The roll's axis tilted from the body's left about its way, radians: over
/// one shoulder to the other hip, not head over heels.
const ROLL_TILT: f32 = 0.45;
/// The tuck: the trunk curled forward, radians; the arms in (swing, elbow);
/// the chin down, radians; each ankle this far forward of and below its
/// socket, metres (the knees up to the chest).
const TUCK_LEAN: f32 = 1.3;
const TUCK_ARMS: (f32, f32) = (1.2, 1.8);
const TUCK_CHIN: f32 = 0.9;
const TUCK_ANKLES: (f32, f32) = (0.12, 0.3);
/// The roll's resting height is sampled this often, seconds, and smoothed
/// over this either side, seconds.
const REST_STEP: f32 = 0.005;
const REST_WINDOW: f32 = 0.06;

/// Where it is in a fall.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FallPhase {
    /// In the air.
    Flight,
    /// Feet down, the hips going down to the bottom.
    Land,
    /// Rolling over a shoulder.
    Roll,
    /// Rising back to standing.
    Recover,
}

/// A roll, as planned: everything about the body's centroid, the anchor
/// it is placed from each frame.
#[derive(Debug, Clone)]
struct Roll {
    /// The way it rolls (horizontal, unit), about which axis (the world),
    /// how fast, m/s, and turning how fast, rad/s.
    way: Vec3,
    axis: Vec3,
    speed: f32,
    spin: f32,
    /// The tucked pose, and how far its farthest joint is from its centroid
    /// (what the roll's speed turns it at).
    tuck: LocalPose,
    #[cfg_attr(not(test), allow(dead_code))]
    radius: f32,
    /// The squat landing it starts as; standing at the end (the pose's
    /// frame).
    squat: Box<Falling>,
    stood: LocalPose,
    /// The centroid as it starts to tuck, and its velocity; tucked and
    /// rolling; done rolling; standing at the end (the world).
    from: Vec3,
    from_velocity: Vec3,
    tucked: Vec3,
    rolled: Vec3,
    end: Vec3,
    /// Where the root stands at the end; and its resting height above the
    /// ground from the end of the squat's give on, every `REST_STEP`.
    end_root: Vec3,
    rests: Vec<f32>,
    /// Tucking, how far the eased height is lifted, every `REST_STEP`.
    lifts: Vec<f32>,
}

/// `samples` (every `REST_STEP` from the end of the squat's give) at `tau`
/// after touchdown, linearly between them; 0 if there are none.
fn sampled(samples: &[f32], tau: f32) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    let at = ((tau - ROLL_ABSORB) / REST_STEP).max(0.0);
    let last = samples.len() - 1;
    let k = (at.floor() as usize).min(last);
    let next = (k + 1).min(last);
    let s = (at - k as f32).clamp(0.0, 1.0);
    samples[k] + (samples[next] - samples[k]) * s
}

impl Roll {
    /// The resting height `tau` after touchdown.
    fn rest_at(&self, tau: f32) -> f32 {
        sampled(&self.rests, tau)
    }

    /// Tucking, the lift on the eased height `tau` after touchdown.
    fn lift_at(&self, tau: f32) -> f32 {
        sampled(&self.lifts, tau)
    }
}

/// The body as the standing pose has it, measured once.
#[derive(Debug, Clone)]
struct Body {
    stood: LocalPose,
    hips: Vec3,
    sockets: [Vec3; 2],
    ankles: [Vec3; 2],
    thigh: f32,
    shin: f32,
}

/// A walker falling off an edge and landing below.
#[derive(Debug, Clone)]
pub struct Falling {
    body: Body,
    yaw: f32,
    turn: Quat,
    /// The foot IK's pelvis drop standing.
    drop: f32,
    /// Seconds since it left; and before the last [`Self::advance`], the
    /// frame a catch is swept over.
    t: f32,
    last_t: f32,
    /// The hips and their velocity as it left; each ankle then (the world).
    from_hips: Vec3,
    velocity: Vec3,
    from_ankles: [Vec3; 2],
    /// The pose as it left, its arms let go from; and whether it reaches up
    /// to catch a ledge falling past ([`Self::reach`]).
    from_pose: LocalPose,
    reaching: bool,
    /// The ground it lands on.
    ground: f32,
    /// When it touches down, the bottom and standing again, seconds.
    ends: [f32; 3],
    /// Down at touchdown, m/s; how deep it goes, metres.
    touch_speed: f32,
    depth: f32,
    /// How the landing brakes: the velocity falls as `(1-s)^n (1+n·s)`.
    braking: f32,
    /// The hips above the ground touching down, and standing again.
    touch_height: f32,
    stand_height: f32,
    /// Each ankle planted on the ground (the world).
    feet: [Vec3; 2],
    /// A wall it faces, its hands kept off ([`Self::facing_wall`]): a point
    /// on its top edge and the face's way out.
    wall: Option<(Vec3, Vec3)>,
    /// How far the hips keep off it, metres, every [`ROOM_STEP`] seconds
    /// from leaving (empty: [`HIPS_OFF_WALL`]).
    wall_rooms: Vec<f32>,
    /// How far out from that the hips start slowing, metres: [`WALL_GIVE`],
    /// or less, leaving nearer (see [`Self::facing_wall`]).
    wall_give: f32,
    /// Each ankle's velocity as it left, relative to the hips (the world):
    /// the legs swinging in a jump ([`Self::from_jump`]).
    ankle_velocities: [Vec3; 2],
    /// Leaving a jump, its pose [`AHEAD`] later: the trunk and arms leave
    /// turning toward it (and on past) as the jump turned them.
    from_ahead: Option<LocalPose>,
    /// Leaving a jump, the hips' own velocity less the centre of mass's,
    /// horizontal ([`Self::drift`]).
    hips_drift: Vec3,
    /// How far the landing is moved to put both feet on a top, horizontal
    /// ([`Self::shifted`]).
    land_shift: Vec3,
    /// Squatting where a roll would go off what it lands on
    /// ([`Self::keep_roll_on`]).
    no_roll: bool,
    /// Turning in the air: how far short of its facing it leaves, radians,
    /// and how long it takes ([`Self::spin_round`]).
    spin: f32,
    spin_time: f32,
    /// The ledge it leaps at ([`Self::aim_at`]).
    target: Option<Ledge>,
    /// Rolling, not squatting; landing hurt; the rig it was measured on.
    roll: Option<Roll>,
    hurt: bool,
    rig: std::sync::Arc<RigGeometry>,
    /// Sliding down a wall ([`Self::slide`]): the share of gravity its hands
    /// and feet brake off; 0 falling free.
    slide: f32,
}

impl Falling {
    /// A fall from where a walker's root is, turned `yaw` (radians about
    /// `+Y`), its hips moving `velocity`, its legs posed `pose`, to the
    /// ground `ground` high below; its hips `drop` below standing (the foot
    /// IK's) as it left, and to be when it stands again.
    #[allow(clippy::too_many_arguments)]
    pub fn off(root: Vec3, yaw: f32, velocity: Vec3, pose: &LocalPose, ground: f32, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let turn = Quat::from_rotation_y(yaw);
        let at = forward_kinematics_on(stood, rig);
        let hips = at[Bone::Hips];
        let body = Body {
            stood: *stood,
            hips,
            sockets: LEGS.map(|(socket, _, _, _)| at[socket] - hips),
            ankles: LEGS.map(|(_, _, ankle, _)| at[ankle]),
            thigh: 0.5 * LEGS.iter().map(|&(socket, knee, _, _)| (at[knee] - at[socket]).length()).sum::<f32>(),
            shin: 0.5 * LEGS.iter().map(|&(_, knee, ankle, _)| (at[ankle] - at[knee]).length()).sum::<f32>(),
        };
        let posed = forward_kinematics_on(pose, rig);
        let from_ankles = LEGS.map(|(_, _, ankle, _)| root + turn * posed[ankle]);
        let from_hips = root + turn * posed[Bone::Hips];
        let mut falling = Self {
            body,
            yaw,
            turn,
            drop,
            t: 0.0,
            last_t: 0.0,
            from_hips,
            velocity,
            from_ankles,
            from_pose: *pose,
            reaching: false,
            ground,
            ends: [0.0; 3],
            touch_speed: 0.0,
            depth: 0.0,
            braking: 1.0,
            touch_height: 0.0,
            stand_height: 0.0,
            feet: [Vec3::ZERO; 2],
            wall: None,
            wall_rooms: Vec::new(),
            wall_give: WALL_GIVE,
            ankle_velocities: [Vec3::ZERO; 2],
            from_ahead: None,
            hips_drift: Vec3::ZERO,
            land_shift: Vec3::ZERO,
            no_roll: false,
            spin: 0.0,
            spin_time: 0.0,
            target: None,
            roll: None,
            hurt: false,
            rig: std::sync::Arc::new(rig.clone()),
            slide: 0.0,
        };
        falling.replan(rig);
        falling
    }

    /// How fast it falls in the air, m/s²: gravity, less what sliding down
    /// a wall brakes off.
    fn gravity(&self) -> f32 {
        GRAVITY * (1.0 - self.slide)
    }

    /// Sliding down the wall it faces ([`Self::against`] first) instead of
    /// falling free: its hands and feet pressed to the face brake
    /// [`SLIDE_BRAKE`] of gravity off, the arms up, the hands on the face,
    /// the feet braced on it until late; landed as from the drop that
    /// speed comes from.
    pub fn slide(&mut self) {
        self.slide = SLIDE_BRAKE;
        self.reaching = true;
        let rig = self.rig.clone();
        self.replan(&rig);
    }

    /// Whether it slides down a wall ([`Self::slide`]).
    pub fn is_sliding(&self) -> bool {
        self.slide > 0.0
    }

    /// Whether it faces a wall it is held off ([`Self::against`]).
    pub fn faces_wall(&self) -> bool {
        self.wall.is_some()
    }

    /// Sliding, how much the hands are held on the face, 0-1: all of it in
    /// the air, let go over [`SLIDE_LET_GO`] once landed.
    fn slide_hold(&self, flight: f32, landed: bool) -> f32 {
        if !self.is_sliding() {
            0.0
        } else if !landed {
            1.0
        } else {
            1.0 - smoothstep(((self.t - flight) / SLIDE_LET_GO).clamp(0.0, 1.0))
        }
    }

    /// How far the legs have come from where they left to the landing's
    /// shape, 0-1, through a flight `flight` long: over all of it; sliding,
    /// over its last part only, the feet braced on the face till then.
    fn legs_down(&self, flight: f32) -> f32 {
        let u = (self.t / flight).clamp(0.0, 1.0);
        if self.is_sliding() { ((u - SLIDE_FEET) / (1.0 - SLIDE_FEET)).clamp(0.0, 1.0) } else { u }
    }

    /// A jump in the air (`jump::Jump`) gone over an edge, falling on to
    /// the ground `ground` high below: its pose now, from the walker's root
    /// `root` turned `yaw`, the hips' velocity the jump's (its root motion
    /// and its pose's together).
    #[allow(clippy::too_many_arguments)]
    pub fn from_jump(jump: &crate::character::anim::jump::Jump, root: Vec3, yaw: f32, ground: f32, drop: f32, stood: &LocalPose, rig: &RigGeometry) -> Self {
        let turn = Quat::from_rotation_y(yaw);
        let t = jump.elapsed();
        // The centre of mass's velocity, the ballistic one: the hips swing
        // about it as the legs tuck (their own, 1.43 m/s forward one frame
        // and 1.04 two later, flung the fall short of a ledge in reach).
        let h = 1.0e-3;
        let com_at = |t: f32| turn * rig.forward() * jump.travelled_at(t) + Vec3::Y * jump.com_height_at(t);
        let velocity = (com_at(t + h) - com_at(t - h)) / (2.0 * h);
        let mut falling = Self::off(root, yaw, velocity, &jump.pose_at(t, stood, rig), ground, drop, stood, rig);
        // The legs' own swing, the hips' aside: from rest, the feet stopped
        // dead (1.7 m/s) as the fall took over.
        let ankles = |t: f32| {
            let at = forward_kinematics_on(&jump.pose_at(t, stood, rig), rig);
            LEGS.map(|(_, _, ankle, _)| turn * (at[ankle] - at[Bone::Hips]))
        };
        // Over the jump's next `AHEAD` too: a take-off leg near straight
        // turns its knee far for a few millimetres of the ankle's miss.
        let (after, now) = (ankles(t + AHEAD), ankles(t));
        falling.ankle_velocities = [0, 1].map(|side| (after[side] - now[side]) / AHEAD);
        falling.from_ahead = Some(jump.pose_at(t + AHEAD, stood, rig));
        // The hips going on at their own speed a moment: a running jump's
        // run ahead of its centre of mass, and the fall's first step was
        // 2.9 cm off the jump's. Read over the jump's next `AHEAD`: just
        // after take-off they slow fast (3.0 m/s at the instant, 2.4 over
        // the next frame), and the instant's overshot by 1 cm a frame.
        let hips_at = |t: f32| com_at(t).with_y(0.0) + turn * forward_kinematics_on(&jump.pose_at(t, stood, rig), rig)[Bone::Hips].with_y(0.0);
        falling.hips_drift = (hips_at(t + AHEAD) - hips_at(t)) / AHEAD - velocity.with_y(0.0);
        let rig = falling.rig.clone();
        falling.replan(&rig);
        falling
    }

    /// Lands on the first top its feet come down onto along its flight, if
    /// higher than the ground it falls to: `ground` finds the ground's height
    /// at a point (the world). A running jump clearing a gap fell on past
    /// the far top it reached, to the floor of the gap.
    pub fn land_on(&mut self, ground: &dyn Fn(Vec3) -> Option<f32>) {
        let rig = self.rig.clone();
        let [flight, ..] = self.ends;
        let below = self.ground;
        // Over a top and above it the step before: come down onto it, not
        // into its side (reaching its footprint already under its top, a
        // jump falling short landed on it through the wall, 16 cm in).
        let mut over = None;
        // The feet are planted ahead of the hips touching down (under the
        // hips, a top whose edge was under the feet was not found).
        // Each foot's ball: a top under any part of either foot is landed on
        // (both feet then put on it, `land_both_feet_on`).
        let touch = self.from_hips + flat_of(self.velocity) * flight + self.drift(flight);
        let ahead = self.feet.map(|foot| flat_of(foot - touch) + self.turn * rig.forward() * FOOT_BALL);
        for i in 1..=(flight / LAND_STEP).ceil() as usize {
            let t = (LAND_STEP * i as f32).min(flight);
            let hips = self.from_hips + self.velocity * t - Vec3::Y * (0.5 * self.gravity() * t * t) + self.drift(t);
            let feet = hips - Vec3::Y * self.touch_height;
            let top = ahead.iter().filter_map(|&ahead| ground(feet + ahead)).filter(|&top| top > self.ground + STEP_DOWN).reduce(f32::max);
            if let Some(top) = top.filter(|&top| feet.y <= top && over == Some(top)) {
                self.land_both_feet_on(top, below, ground, &rig);
                self.keep_roll_on(below, ground, &rig);
                return;
            }
            over = top.filter(|&top| feet.y > top);
        }
        self.keep_roll_on(below, ground, &rig);
    }

    /// Rolling, the roll's way along the ground kept on ground as high as it
    /// lands on, every [`ROLL_ROOM_STEP`] from where it tucks to where it
    /// stands, a body's width either side; else it squats (the feet put on
    /// a top again). The feet put on a top where a squat lands, a roll went
    /// on 1.5-2 m and off its far edge.
    fn keep_roll_on(&mut self, below: f32, ground: &dyn Fn(Vec3) -> Option<f32>, rig: &RigGeometry) {
        let Some(roll) = self.roll.as_ref() else {
            return;
        };
        let level = self.ground;
        let on = |point: Vec3| ground(Vec3::new(point.x, level + 0.1, point.z)).is_some_and(|height| (height - level).abs() < 0.05);
        let across = Vec3::Y.cross(roll.way) * ROLL_HALF_WIDTH;
        let legs = [(roll.from, roll.tucked), (roll.tucked, roll.rolled), (roll.rolled, roll.end_root)];
        let stays = legs.iter().all(|&(a, b)| {
            let steps = ((flat_of(b - a).length() / ROLL_ROOM_STEP).ceil() as usize).max(1);
            (0..=steps).all(|k| {
                let point = a.lerp(b, k as f32 / steps as f32);
                [-1.0, 0.0, 1.0].iter().all(|&side| on(point + across * side))
            })
        });
        if stays {
            return;
        }
        // Squatting: on a top, both feet on it again (or past it).
        self.no_roll = true;
        self.land_shift = Vec3::ZERO;
        if level > below + STEP_DOWN {
            self.land_both_feet_on(level, below, ground, rig);
        } else {
            self.replan(rig);
        }
    }

    /// Landing on the top `top` high, with both feet on it: as it falls, or
    /// moved as little as puts both on it (its edge across or along the
    /// way it falls, between or under the feet); else not on it at all,
    /// falling on to the ground below. Landed as it fell, a foot was left
    /// over the edge, in the air.
    fn land_both_feet_on(&mut self, top: f32, below: f32, ground: &dyn Fn(Vec3) -> Option<f32>, rig: &RigGeometry) {
        self.ground = top;
        self.replan(rig);
        let forward = flat_of(self.velocity).try_normalize().unwrap_or(self.turn * rig.forward());
        let across = Vec3::Y.cross(forward);
        let toward = self.turn * rig.forward();
        let on = |point: Vec3| ground(Vec3::new(point.x, top + 0.1, point.z)).is_some_and(|height| (height - top).abs() < 0.05);
        // Each foot from heel to ball, a foot's width either side.
        let both_on = |falling: &Self| {
            falling.feet.iter().all(|&ankle| {
                [-FOOT_HEEL, FOOT_BALL].iter().all(|&along| [-FOOT_SIDE, FOOT_SIDE].iter().all(|&side| on(ankle + toward * along + Vec3::Y.cross(toward) * side)))
            })
        };
        if both_on(self) {
            return;
        }
        // Moved by a displacement eased in over the flight, not a change of
        // velocity: re-aimed, a running jump's hand-off kinked 1.3 cm.
        let shifts = (1..=15).flat_map(|k| {
            let s = 0.02 * k as f32;
            [forward * s, -forward * s, across * s, -across * s]
        });
        for shift in shifts {
            let mut moved = self.clone();
            moved.land_shift = shift;
            moved.replan(rig);
            if both_on(&moved) {
                *self = moved;
                return;
            }
        }
        // Not on it: falling on past it, moved as little as keeps the feet
        // clear of it all the way down (falling straight on, a foot over a
        // narrow top went down through it).
        self.ground = below;
        self.replan(rig);
        if self.feet_clear(ground, rig) {
            return;
        }
        // Further than onto it: the arms out landing reach past the feet.
        let wider = (1..=30).flat_map(|k| {
            let s = 0.02 * k as f32;
            [forward * s, -forward * s, across * s, -across * s]
        });
        for shift in wider {
            let mut moved = self.clone();
            moved.land_shift = shift;
            moved.replan(rig);
            if moved.feet_clear(ground, rig) {
                *self = moved;
                return;
            }
        }
    }

    /// Whether every joint stays clear of every top (1 cm over it) all the
    /// way down and up again, sampled every 5 ms (the feet alone: moved to
    /// straddle a narrow top, the body came down onto it).
    fn feet_clear(&self, ground: &dyn Fn(Vec3) -> Option<f32>, rig: &RigGeometry) -> bool {
        let mut at = self.clone();
        let end = self.ends[2];
        (0..=(end / 0.005).ceil() as usize).all(|i| {
            at.t = (0.005 * i as f32).min(end);
            let joints = forward_kinematics_on(&at.pose(rig), rig);
            let (root, turn) = (at.root(), at.turn_now());
            // The highest ground at each joint's spot, read from far above
            // (from the joint, a top more than a foot over it was missed).
            Bone::ALL.into_iter().all(|bone| {
                let joint = root + turn * joints[bone];
                ground(Vec3::new(joint.x, joint.y + 100.0, joint.z)).is_none_or(|height| height <= joint.y - 0.01 || height < self.ground + STEP_DOWN)
            })
        })
    }

    /// How far the landing has been moved `t` seconds in
    /// ([`Self::land_shift`]): eased in over the flight from standing still.
    fn shifted(&self, t: f32) -> Vec3 {
        self.land_shift * smoothstep((t / self.ends[0].max(1.0e-3)).clamp(0.0, 1.0))
    }

    /// Plans the landing for the velocity it left with: a squat, a roll or
    /// hurt.
    fn replan(&mut self, rig: &RigGeometry) {
        self.roll = None;
        self.plan();
        // Rolling past standing height, or moving on too fast to brake in a
        // squat; not hurt.
        let drop = self.dropped();
        let braking = flat_of(self.velocity).length() / landing_for(drop).0;
        if !self.hurt && self.wall.is_none() && !self.no_roll && (drop > ROLL_DROP || (drop > ROLL_LOW && braking > MOST_BRAKING)) {
            self.plan_roll(rig);
        }
    }

    /// Leaving with the horizontal velocity that brings its hips to rest
    /// over `spot` (the world; its height not read): on a step under the
    /// feet letting go, or clear past its edge. Squatting only: a roll
    /// goes on past where a squat comes to rest.
    pub fn land_at(&mut self, spot: Vec3, rig: &RigGeometry) {
        // The time to rest depends on the speed only through where the feet
        // plant ahead, and a wall faced holds the rest off it: a few rounds
        // of correcting by the miss settle it.
        let [flight, landed, _] = self.ends;
        let flat = flat_of(spot - self.from_hips) / (flight + 0.5 * (landed - flight)).max(1.0e-3);
        self.velocity = Vec3::new(flat.x, self.velocity.y, flat.z);
        self.replan(rig);
        for _ in 0..8 {
            let [flight, landed, _] = self.ends;
            self.velocity += flat_of(spot - self.rest()) / (flight + 0.5 * (landed - flight)).max(1.0e-3);
            self.replan(rig);
        }
    }

    /// Facing a wall close enough to touch (a point on its top edge, the
    /// face's way out): under its top, the hands kept [`HANDS_OFF_WALL`]
    /// off it and the toes out of it.
    ///
    /// Landed, the hips come back from it as far as the landing's shape
    /// reaches ahead of them (the hands, kept off, aside): stopped at
    /// [`HIPS_OFF_WALL`], a 3 m drop's landing put the head 34 cm into
    /// it. It does not roll.
    pub fn facing_wall(&mut self, point: Vec3, out: Vec3) {
        self.wall = Some((point, out));
        self.wall_rooms.clear();
        let rig = self.rig.clone();
        self.replan(&rig);
        let forward = self.turn * rig.forward();
        // Twice: the room moves the hips against the planted feet, which
        // moves the knees.
        for _ in 0..2 {
            let mut probe = self.clone();
            let [flight, _, end] = self.ends;
            let needs: Vec<f32> = (0..=(end / ROOM_STEP).ceil() as usize)
                .map(|i| {
                    probe.t = (ROOM_STEP * i as f32).min(end);
                    // In the air, the hips' stop (braced feet let go from a
                    // wall are ahead of them on it).
                    if probe.t < flight {
                        return HIPS_OFF_WALL;
                    }
                    let at = forward_kinematics_on(&probe.pose(&rig), &rig);
                    let arms = [ArmChain::LEFT, ArmChain::RIGHT].map(|chain| [chain.elbow, chain.wrist]).concat();
                    let ahead = Bone::ALL
                        .into_iter()
                        .filter(|bone| !arms.contains(bone) && !matches!(bone, Bone::LeftArm | Bone::RightArm))
                        .map(|bone| (self.turn * (at[bone] - at[Bone::Hips])).dot(forward))
                        .fold(0.0, f32::max);
                    HIPS_OFF_WALL.max(ahead + ROOM_MARGIN)
                })
                .scan(0.0f32, |most, need| {
                    *most = most.max(need);
                    Some(*most)
                })
                .collect();
            // Held up to the most over twice a window ahead, then averaged
            // over it either side twice: smooth (a ramp once averaged kinked
            // at 6 g), and never short of the need, which only grows.
            let w = (ROOM_WINDOW / ROOM_STEP).round() as usize;
            let held: Vec<f32> = (0..needs.len()).map(|i| needs[i..(i + 2 * w + 1).min(needs.len())].iter().copied().fold(0.0, f32::max)).collect();
            let average = |values: &[f32]| -> Vec<f32> {
                (0..values.len()).map(|i| {
                    let window = &values[i.saturating_sub(w)..(i + w + 1).min(values.len())];
                    window.iter().sum::<f32>() / window.len() as f32
                }).collect()
            };
            self.wall_rooms = average(&average(&held));
            self.replan(&rig);
        }
        // Leaving nearer than the give (a braced hang's hips, 0.42 m out,
        // against the 0.45 m it starts at in the air), slowing starts where
        // they leave: else a leap aside along a wall in line was pushed off
        // it the frame it let go (4 m/s²).
        let leaving = (self.from_hips - point).dot(out) - self.wall_room_at(0.0);
        self.wall_give = leaving.clamp(MIN_WALL_GIVE, WALL_GIVE);
        self.replan(&rig);
    }

    /// How far the hips keep off the wall `t` seconds in, metres (planned
    /// by [`Self::facing_wall`]).
    fn wall_room_at(&self, t: f32) -> f32 {
        let Some(&last) = self.wall_rooms.last() else {
            return HIPS_OFF_WALL;
        };
        let at = t / ROOM_STEP;
        let i = (at.floor() as usize).min(self.wall_rooms.len() - 1);
        let next = self.wall_rooms.get(i + 1).copied().unwrap_or(last);
        self.wall_rooms[i] + (next - self.wall_rooms[i]) * (at - i as f32).clamp(0.0, 1.0)
    }

    /// The room the hips keep landed.
    fn wall_room(&self) -> f32 {
        self.wall_rooms.last().copied().unwrap_or(HIPS_OFF_WALL)
    }

    /// The wall among `ledges` it would fall into, if any, faced: one it
    /// faces (within 60°), in front of where it leaves, coming to rest
    /// within [`WALL_NEAR`] of its face, its top above the ground it lands
    /// on by more than a step and its face reaching down to it (not a slab
    /// it falls out from under).
    pub fn against(&mut self, ledges: &[Ledge], rig: &RigGeometry) {
        let forward = self.turn * rig.forward();
        let rest = self.rest();
        let wall = ledges.iter().find(|ledge| {
            let along = (rest - ledge.a).dot(ledge.along());
            forward.dot(-ledge.out) > 0.5
                && (0.0..=(ledge.b - ledge.a).length()).contains(&along)
                && ledge.out_of(self.from_hips) > 0.0
                && ledge.out_of(rest) < WALL_NEAR
                && ledge.height() > self.ground + STEP_DOWN
                && ledge.height() - ledge.wall_below <= self.ground + STEP_DOWN
        });
        if let Some(ledge) = wall {
            self.facing_wall(ledge.nearest(rest, 0.0), ledge.out);
        }
    }

    /// Where its hips come to rest landing, the world (a squat's).
    pub fn rest(&self) -> Vec3 {
        let [flight, landed, _] = self.ends;
        let rest = self.off_wall(self.from_hips + flat_of(self.velocity) * (flight + 0.5 * (landed - flight)) + self.drift(flight), self.wall_room());
        Vec3::new(rest.x, self.ground + self.stand_height, rest.z)
    }

    /// The ground it lands on, metres up.
    pub fn ground(&self) -> f32 {
        self.ground
    }

    /// Whether it lands hurt ([`HURT_DROP`]): down onto its hands, held
    /// down, slow up.
    pub fn is_hurt(&self) -> bool {
        self.hurt
    }

    /// Reaching up, falling, to catch a ledge it falls past (or not).
    pub fn reach(&mut self, up: bool) {
        self.reaching = up;
    }

    /// The hips' velocity now (the world).
    pub fn hips_velocity(&self) -> Vec3 {
        let h = 1.0e-3;
        let (before, after) = ((self.t - h).max(0.0), self.t + h);
        (self.hips_at(after) - self.hips_at(before)) / (after - before)
    }

    /// Falling, the ledge among `ledges` it catches now, if any: one it
    /// faces (square to within 45°), its hips out in front of the face and
    /// along it, its lip within the arms' reach of the shoulders and above
    /// them, falling. Reaching up ([`Self::reach`]), the hands meet it.
    ///
    /// Swept over the frame the shoulders pass the lip's height, the body as
    /// it is at times across it. Tested at the frame's end alone, a frame of
    /// a tenth of a second (a hitch) fell past the 0.36 m the lip is in
    /// reach and on to the ground.
    pub fn catches(&self, ledges: &[Ledge], rig: &RigGeometry) -> Option<Ledge> {
        // Rising at the frame's end, it rose all of it (ballistic).
        if !self.airborne() || self.hips_velocity().y >= 0.0 {
            return None;
        }
        // The body `t` seconds in: its shoulders' middle, hips, facing and
        // arm's length; whether it can catch then (falling, not turning).
        struct Then {
            shoulders: Vec3,
            hips: Vec3,
            forward: Vec3,
            arm: f32,
            able: bool,
        }
        let then = |t: f32| {
            let mut at = self.clone();
            at.t = t;
            let joints = forward_kinematics_on(&at.pose(rig), rig);
            let (root, turn) = (at.root(), at.turn_now());
            Then {
                shoulders: root + turn * (0.5 * (joints[ArmChain::LEFT.shoulder] + joints[ArmChain::RIGHT.shoulder])),
                hips: root + turn * joints[Bone::Hips],
                forward: turn * rig.forward(),
                arm: (joints[ArmChain::LEFT.elbow] - joints[ArmChain::LEFT.shoulder]).length() + (joints[ArmChain::LEFT.wrist] - joints[ArmChain::LEFT.elbow]).length(),
                able: at.airborne() && at.hips_velocity().y < 0.0 && !at.is_spinning(),
            }
        };
        let catches = |ledge: &Ledge, body: &Then| {
            body.able
                && body.shoulders.y <= ledge.height()
                && body.forward.dot(-ledge.out) > std::f32::consts::FRAC_1_SQRT_2
                && (0.05..0.8).contains(&ledge.out_of(body.hips))
                && (ledge.nearest(body.shoulders, 0.3) - body.shoulders).length() <= CATCH_REACH * body.arm
        };
        let now = then(self.t);
        // The frame's start, posed only when its shoulders may have been
        // over a lip: no higher than its hips by their distance now (a
        // frame's bend of the spine barely changes it).
        let reach_up = (now.shoulders - now.hips).length() + 0.05;
        let start_hips = self.hips_at(self.last_t).y;
        let mut start: Option<Then> = None;
        // Swept over the frame the shoulders pass the lip, the body as it is
        // at each of `CATCH_SWEEP` times across it: estimated by moving the
        // body back as the hips moved, at 5 fps a leap back passed the lip
        // out of reach (the body turning and reaching meanwhile).
        ledges.iter().copied().find(|ledge| {
            if now.shoulders.y > ledge.height() {
                return false;
            }
            if start_hips + reach_up < ledge.height() || start.get_or_insert_with(|| then(self.last_t)).shoulders.y <= ledge.height() {
                return catches(ledge, &now);
            }
            (1..=CATCH_SWEEP).any(|k| {
                let t = self.last_t + (self.t - self.last_t) * k as f32 / CATCH_SWEEP as f32;
                if k == CATCH_SWEEP { catches(ledge, &now) } else { catches(ledge, &then(t)) }
            })
        })
    }

    /// How far it drops, standing height to standing height, metres.
    pub fn dropped(&self) -> f32 {
        // Sliding, the drop free fall would meet the ground as fast from.
        (self.from_hips.y - self.drop_hips() - self.ground) * (1.0 - self.slide)
    }

    /// Whether it drops too far to land: the body goes to the ragdoll as it
    /// touches down ([`FATAL_DROP`]).
    pub fn is_fatal(&self) -> bool {
        self.dropped() > FATAL_DROP
    }

    /// Whether it rolls landing.
    pub fn rolls(&self) -> bool {
        self.roll.is_some()
    }

    /// Plans the roll: from the touchdown (as the squat's) tucking and
    /// rolling once over a shoulder at its speed, the centroid as high as the
    /// tucked body's farthest joint is from it, then coming up to standing.
    fn plan_roll(&mut self, rig: &RigGeometry) {
        let flight = self.ends[0];
        let flat = Vec3::new(self.velocity.x, 0.0, self.velocity.z);
        let way = flat.try_normalize().unwrap_or(self.turn * rig.forward());
        // Forward about the body's left (as `jump::upper` leans), tilted about
        // the way.
        let lateral = Vec3::Y.cross(way).normalize();
        let lateral = if lateral.dot(self.turn * rig.left()) < 0.0 { -lateral } else { lateral };
        let axis = Quat::from_axis_angle(way, -ROLL_TILT) * lateral;
        let speed = flat.length().max(ROLL_FROM_DOWN * self.touch_speed).max(ROLL_SLOWEST);
        // The tuck.
        let mut tuck = crate::character::anim::jump::upper(&self.body.stood, rig, TUCK_LEAN, TUCK_ARMS);
        let left = rig.left();
        pose_turn(&mut tuck, rig, Bone::Neck, Quat::from_axis_angle(left, TUCK_CHIN));
        for (side, &(_, _, ankle, _)) in LEGS.iter().enumerate() {
            let target = self.body.sockets[side] + rig.forward() * TUCK_ANKLES.0 - Vec3::Y * TUCK_ANKLES.1;
            place_ankle(&mut tuck, rig, ankle, target);
        }
        let centroid = centroid_of(&tuck, rig);
        let radius = forward_kinematics_on(&tuck, rig).iter().map(|(_, p)| (*p - centroid).length()).fold(0.0, f32::max) + 0.02;
        let spin = speed / radius;
        // The squat landing it starts as, its centroid and velocity as it
        // starts to tuck; standing at the end.
        let squat = Box::new(self.clone());
        let centroid_at = |t: f32| {
            let mut at = (*squat).clone();
            at.t = t;
            at.root() + self.turn * centroid_of(&at.pose(rig), rig)
        };
        let start = flight + ROLL_ABSORB;
        let from = centroid_at(start);
        let from_velocity = (centroid_at(start + 1.0e-3) - from) / 1.0e-3;
        let mut stood = self.body.stood;
        stood.root_translation.y -= self.drop;
        for (side, &(_, _, ankle, _)) in LEGS.iter().enumerate() {
            place_ankle(&mut stood, rig, ankle, self.body.ankles[side] - (self.body.hips - Vec3::Y * self.drop));
        }
        // Rolling: tucked by the end of the tucking, the turn rising to its
        // speed over it; once round, easing off as it comes up.
        let tucked_at = ROLL_ABSORB + ROLL_TUCK;
        let rolled_at = (tucked_at + std::f32::consts::TAU / spin - 0.5 * ROLL_TUCK - 0.5 * ROLL_EASE).max(tucked_at);
        let height = self.ground + radius;
        let tucked = Vec3::new(0.0, height, 0.0) + flat_of(from + way * (0.5 * ROLL_TUCK * (flat_of(from_velocity).dot(way) + speed)));
        let rolled = tucked + way * (speed * (rolled_at - tucked_at));
        let end_root = Vec3::new(rolled.x, self.ground, rolled.z) + way * (0.5 * speed * ROLL_UP) - flat_of(self.turn * centroid_of(&stood, rig));
        let end = end_root + self.turn * centroid_of(&stood, rig);
        self.ends = [flight, flight + rolled_at, flight + rolled_at + ROLL_UP];
        self.roll = Some(Roll { way, axis, speed, spin, tuck, radius, squat, stood, from, from_velocity, tucked, rolled, end, end_root, rests: Vec::new(), lifts: Vec::new() });
        self.plan_rest(rig);
    }

    /// How far the hips are above the ankles' line, the knees flexed `flexion`
    /// (radians), the leg upright: its socket-to-ankle length.
    fn leg_at(&self, flexion: f32) -> f32 {
        let (a, b) = (self.body.thigh, self.body.shin);
        (a * a + b * b + 2.0 * a * b * flexion.cos()).sqrt()
    }

    fn plan(&mut self) {
        // The hips above the ground standing (the stood pose's, its root on
        // the ground, less the drop), and touching down: as much higher as
        // the legs are longer at the knees' contact flexion than standing.
        let standing_leg = (0..2).map(|side| (self.body.ankles[side] - (self.body.hips + self.body.sockets[side])).length()).sum::<f32>() * 0.5;
        let contact = self.leg_at(KNEE_AT_CONTACT.to_radians());
        self.stand_height = self.body.hips.y - self.drop;
        // The landing's time and the knees' deepest, from how far it drops,
        // standing height to standing height.
        let drop = self.dropped();
        self.hurt = drop > HURT_DROP && drop <= FATAL_DROP;
        let (land, flexion) = landing_for(drop);
        let flexion = if self.hurt { HURT_KNEE } else { flexion };
        // Moving on, the feet plant ahead of the hips by half the braking
        // distance, the leg leant toward them: its height that much less
        // (taken upright, a planted ankle at 1.4 m/s was 3.5 mm short).
        let ahead = 0.5 * Vec3::new(self.velocity.x, 0.0, self.velocity.z).length() * land;
        self.touch_height = self.body.hips.y + ((contact * contact - ahead * ahead).max(0.0).sqrt() - standing_leg);
        // Flight: down from where it left to the hips at touchdown.
        let fall = self.from_hips.y - (self.ground + self.touch_height);
        let up = self.velocity.y;
        let g = self.gravity();
        let flight = ((up + (up * up + 2.0 * g * fall.max(0.0)).sqrt()) / g).max(1.0e-3);
        self.touch_speed = g * flight - up;
        // The legs upright, the hips go down as far as the legs shorten.
        let deepest = contact - self.leg_at(flexion.to_radians());
        // As deep as the knees go: the velocity falls as `(1-s)^n (1+n·s)`
        // over the landing, `n` making it that deep. Its braking rises from
        // touchdown to a peak at `T/n` (about 65 ms from 0.9 m; 77-80 ms
        // measured from 0.75 m, Puddle and Maulder 2013) and eases into the
        // bottom. A cubic from the touchdown speed to rest could not be
        // shallower than a third of `v·T`: from 0.9 m, the knees went to 136°,
        // not 116°; braking hardest at touchdown peaked there.
        let v = self.touch_speed;
        self.depth = deepest.min(v * land);
        self.braking = (2.0 * v * land / self.depth - 2.0).max(0.0);
        let rise = (self.stand_height + self.ground) - (self.ground + self.touch_height - self.depth);
        let up_time = (6.0 * rise.max(0.0) / RECOVERY_ACCELERATION).sqrt().max(QUICKEST_UP);
        let up_time = if self.hurt { HURT_HOLD + HURT_SLOWER * up_time } else { up_time };
        self.ends = [flight, flight + land, flight + land + up_time];
        // The feet planted under where the hips come to rest: braking the
        // forward speed over the landing goes on half its time's worth.
        let flat = Vec3::new(self.velocity.x, 0.0, self.velocity.z);
        let rest = self.off_wall(self.from_hips + flat * (flight + 0.5 * land) + self.drift(flight), self.wall_room());
        let root = Vec3::new(rest.x, self.ground, rest.z) - self.turn * Vec3::new(self.body.hips.x, 0.0, self.body.hips.z);
        self.feet = self.body.ankles.map(|ankle| root + self.turn * ankle);
    }

    /// The hips above the ground standing where it left, the drop then
    /// taken off: how high it stood.
    fn drop_hips(&self) -> f32 {
        self.body.hips.y - self.drop
    }

    /// Moves it on `dt` seconds.
    pub fn advance(&mut self, dt: f32) {
        self.last_t = self.t;
        self.t = (self.t + dt).min(self.ends[2]);
    }

    /// Where it is.
    pub fn phase(&self) -> FallPhase {
        if self.t < self.ends[0] {
            FallPhase::Flight
        } else if self.t < self.ends[1] {
            if self.roll.is_some() { FallPhase::Roll } else { FallPhase::Land }
        } else {
            FallPhase::Recover
        }
    }

    /// Rolling, touched down: the pose (its frame) and the root now. The
    /// shape mixed from touching down into the tuck and out to standing,
    /// turned about the roll's axis, placed so its centroid is on the
    /// anchor's path.
    fn rolling(&self, roll: &Roll, rig: &RigGeometry) -> (LocalPose, Vec3) {
        let [flight, rolled_at, stood_at] = self.ends;
        let tau = self.t - flight;
        // First the squat landing's own give, the feet planted.
        let squatting = || {
            let mut squat = (*roll.squat).clone();
            squat.t = self.t;
            squat
        };
        if tau < ROLL_ABSORB {
            let squat = squatting();
            return (squat.pose_at_root(squat.hips_root(), rig), squat.hips_root());
        }
        let pose = self.rolled_pose(roll, tau, rig);
        let (rise, rolled, up) = (ROLL_TUCK, rolled_at - flight, stood_at - rolled_at);
        let tucking = tau - ROLL_ABSORB;
        // The anchor: from the squat's down to rolling height, along at the
        // roll's speed, up to standing.
        let anchor = if tucking < rise {
            hermite(roll.from, roll.from_velocity, roll.tucked, roll.way * roll.speed, rise, tucking / rise)
        } else if tau < rolled {
            roll.tucked + roll.way * (roll.speed * (tucking - rise))
        } else {
            hermite(roll.rolled, roll.way * roll.speed, roll.end, Vec3::ZERO, up, ((tau - rolled) / up).clamp(0.0, 1.0))
        };
        // On the ground, at the planned resting height, eased in from the
        // squat's own over the tucking.
        let resting = self.ground + roll.rest_at(tau);
        let height = if tucking < rise { self.eased_height(roll, tau) + roll.lift_at(tau) } else { resting };
        let root = flat_of(anchor - self.turn * centroid_of(&pose, rig)) + Vec3::Y * height;
        (pose, root)
    }

    /// The roll's shape, `tau` after touchdown (past the squat's give): mixed
    /// from the squat into the tuck and out to standing, turned about the
    /// roll's axis. It does not depend on where the body is.
    fn rolled_pose(&self, roll: &Roll, tau: f32, rig: &RigGeometry) -> LocalPose {
        let [flight, rolled_at, stood_at] = self.ends;
        let (rolled, up) = (rolled_at - flight, stood_at - rolled_at);
        let tucking = tau - ROLL_ABSORB;
        let tucked = if tau < rolled {
            smoothstep((tucking / ROLL_TUCK).clamp(0.0, 1.0))
        } else {
            1.0 - smoothstep(((tau - rolled) / (0.5 * up)).clamp(0.0, 1.0))
        };
        let base = if tau < rolled {
            let mut squat = (*roll.squat).clone();
            squat.t = flight + tau;
            squat.pose_at_root(squat.hips_root(), rig)
        } else {
            roll.stood
        };
        let mut pose = mixed(&base, &roll.tuck, tucked);
        // The turn: rising to the roll's over the tucking, steady, easing off
        // to once round as it comes up.
        let (rise, ease, spin) = (ROLL_TUCK, ROLL_EASE, roll.spin);
        let turned = if tucking < rise {
            spin * tucking * tucking / (2.0 * rise)
        } else if tau < rolled {
            spin * (tucking - 0.5 * rise)
        } else {
            let left = (ease - (tau - rolled)).max(0.0);
            std::f32::consts::TAU - spin * left * left / (2.0 * ease)
        };
        pose_turn(&mut pose, rig, Bone::Hips, Quat::from_axis_angle(self.turn.inverse() * roll.axis, turned));
        pose
    }

    /// The roll's resting height above the ground (the root's), sampled
    /// over the roll once its shape is known: at each moment, as high as
    /// rests its lowest joint as low as standing's (so it stands on its spot
    /// at the end), the greatest over a window either side, then averaged
    /// over it. Smooth and never lower than the joints need. Resting on the
    /// lowest joint frame by frame, the hips jerked as it changed (557 m/s²);
    /// on a sphere's height for the tuck, the toes went 29 cm under as it
    /// came up.
    fn plan_rest(&mut self, rig: &RigGeometry) {
        let Some(roll) = self.roll.as_ref() else { return };
        let [flight, _, stood_at] = self.ends;
        let lowest = |pose: &LocalPose| forward_kinematics_on(pose, rig).iter().map(|(_, p)| p.y).fold(f32::MAX, f32::min);
        let standing = lowest(&roll.stood);
        let count = ((stood_at - flight - ROLL_ABSORB) / REST_STEP).ceil() as usize + 1;
        let needed: Vec<f32> = (0..count).map(|k| standing - lowest(&self.rolled_pose(roll, ROLL_ABSORB + k as f32 * REST_STEP, rig))).collect();
        let reach = (REST_WINDOW / REST_STEP).round() as usize;
        let window = |k: usize| k.saturating_sub(reach)..(k + reach + 1).min(count);
        let highest: Vec<f32> = (0..count).map(|k| needed[window(k)].iter().copied().fold(f32::MIN, f32::max)).collect();
        let rests: Vec<f32> = (0..count).map(|k| {
            let span = window(k);
            highest[span.clone()].iter().sum::<f32>() / span.len() as f32
        }).collect();
        if let Some(roll) = self.roll.as_mut() {
            roll.rests = rests;
        }
        // Tucking, the height eases from the squat's own to the resting one,
        // and may pass under what the joints need (a joint went 1 cm under
        // rolling off a 1 m drop at a run; held up to it frame by frame, the
        // hips jerked at 67 m/s²): what it falls short, planned the same way,
        // ramped in from nothing as the tucking starts.
        let Some(roll) = self.roll.as_ref() else { return };
        let tucking = (ROLL_TUCK / REST_STEP).ceil() as usize + 1;
        let short: Vec<f32> = (0..tucking)
            .map(|k| {
                let tau = ROLL_ABSORB + k as f32 * REST_STEP;
                let need = standing - lowest(&self.rolled_pose(roll, tau, rig));
                (need - (self.eased_height(roll, tau) - self.ground)).max(0.0)
            })
            .collect();
        let window = |k: usize| k.saturating_sub(reach)..(k + reach + 1).min(tucking);
        let highest: Vec<f32> = (0..tucking).map(|k| short[window(k)].iter().copied().fold(0.0, f32::max)).collect();
        let lifts: Vec<f32> = (0..tucking)
            .map(|k| {
                let span = window(k);
                // In from nothing as the tucking starts, out to nothing as
                // it ends (where the resting height takes over; left on, the
                // hips jumped there at 125 m/s²).
                let at = k as f32 * REST_STEP;
                let ramp = smoothstep((at / REST_WINDOW).clamp(0.0, 1.0)) * (1.0 - smoothstep(((at - (ROLL_TUCK - REST_WINDOW)) / REST_WINDOW).clamp(0.0, 1.0)));
                ramp * highest[span.clone()].iter().sum::<f32>() / span.len() as f32
            })
            .collect();
        if let Some(roll) = self.roll.as_mut() {
            roll.lifts = lifts;
        }
    }

    /// Tucking, the height eased from the squat's own root to the planned
    /// resting one, `tau` after touchdown.
    fn eased_height(&self, roll: &Roll, tau: f32) -> f32 {
        let mut squat = (*roll.squat).clone();
        squat.t = self.ends[0] + tau;
        let squat = squat.hips_root().y;
        let resting = self.ground + roll.rest_at(tau);
        squat + (resting - squat) * smoothstep(((tau - ROLL_ABSORB) / ROLL_TUCK).clamp(0.0, 1.0))
    }

    /// Whether it stands again.
    pub fn is_done(&self) -> bool {
        self.t >= self.ends[2]
    }

    /// Whether it is in the air.
    pub fn airborne(&self) -> bool {
        self.phase() == FallPhase::Flight
    }

    /// How fast it touches down, m/s.
    pub fn touch_speed(&self) -> f32 {
        self.touch_speed
    }

    /// When it touches down, reaches the bottom and stands, seconds.
    pub fn ends(&self) -> [f32; 3] {
        self.ends
    }

    /// The facing (radians about `+Y`): turning in the air
    /// ([`Self::spin_round`]), from where it left to where it lands.
    pub fn facing(&self) -> f32 {
        if self.spin == 0.0 {
            return self.yaw;
        }
        // Begun a moment after leaving, and eased in from no speed and no
        // acceleration: turning from the instant it left, one knee's step
        // changed 0.9 cm more the frame it let go, the other's less.
        let s = ((self.t - SPIN_AFTER) / (self.spin_time - SPIN_AFTER).max(1.0e-3)).clamp(0.0, 1.0);
        self.yaw - self.spin * (1.0 - s * s * s * (s * (6.0 * s - 15.0) + 10.0))
    }

    /// The facing now as a turn (the body's frame; the landing is planned in
    /// the facing it lands in).
    fn turn_now(&self) -> Quat {
        Quat::from_rotation_y(self.facing())
    }

    /// Turning `spin` radians about `+Y` in the air over `seconds` (within
    /// the flight), from the facing it left with: it lands facing `spin` on
    /// from it, the landing planned so. Pushing back off a wall, it turns
    /// round to land or catch facing away from it.
    pub fn spin_round(&mut self, spin: f32, seconds: f32, rig: &RigGeometry) {
        self.yaw += spin;
        self.turn = Quat::from_rotation_y(self.yaw);
        self.spin = spin;
        self.replan(rig);
        self.spin_time = seconds.min(SPIN_IN_FLIGHT * self.ends[0]);
    }

    /// Whether it is still turning in the air.
    pub fn is_spinning(&self) -> bool {
        self.spin != 0.0 && self.t < self.spin_time
    }

    /// Leaping at a ledge (`hang::leap`): the one it catches, reaching for
    /// it, whatever the walker asks.
    pub fn aim_at(&mut self, ledge: Ledge) {
        self.target = Some(ledge);
        self.reaching = true;
    }

    /// Leaving with the legs moving, each ankle's velocity relative to the
    /// hips (the world): they coast on it, as leaving a jump.
    pub fn coast_legs(&mut self, ankle_velocities: [Vec3; 2]) {
        self.ankle_velocities = ankle_velocities;
    }

    /// Leaving with the trunk and arms turning: the pose they would be in
    /// [`AHEAD`] later, which they leave turning toward (and on past).
    pub fn coast_upper(&mut self, ahead: LocalPose) {
        self.from_ahead = Some(ahead);
    }

    /// How far ahead the pose given to [`Self::coast_upper`] is, seconds.
    pub const COAST_AHEAD: f32 = AHEAD;

    /// The ledge it leaps at, if any ([`Self::aim_at`]).
    pub fn target(&self) -> Option<Ledge> {
        self.target
    }

    /// The hips in the world now.
    pub fn hips(&self) -> Vec3 {
        if let Some(roll) = self.roll.as_ref().filter(|_| self.t >= self.ends[0]) {
            let (pose, root) = self.rolling(roll, &self.rig);
            return root + self.turn * forward_kinematics_on(&pose, &self.rig)[Bone::Hips];
        }
        self.off_wall(self.free_hips(), self.wall_room_at(self.t))
    }

    /// `point` held `room` out from the wall it faces, if any: coming to
    /// it, eased to a stop over [`WALL_GIVE`] (going on into it, a jump
    /// falling short landed with its shoulders 18 cm into the wall).
    fn off_wall(&self, point: Vec3, room: f32) -> Vec3 {
        let Some((top, out)) = self.wall else {
            return point;
        };
        let (x, s) = ((point - top).dot(out) - room, self.wall_give);
        let kept = if x >= s {
            x
        } else if x > -s {
            (x + s) * (x + s) / (4.0 * s)
        } else {
            0.0
        };
        point + out * (kept - x)
    }

    /// How far the hips have drifted `t` seconds in from the centre of
    /// mass's path: going on at their own speed as they left a jump, the
    /// difference slowing evenly to nothing over [`LEGS_COAST`]; and the
    /// landing moved ([`Self::shifted`]).
    fn drift(&self, t: f32) -> Vec3 {
        let coast = t.min(LEGS_COAST);
        self.hips_drift * (coast - 0.5 * coast * coast / LEGS_COAST) + self.shifted(t)
    }

    /// The hips with no wall in the way.
    fn free_hips(&self) -> Vec3 {
        let [flight, landed, _] = self.ends;
        let flat = Vec3::new(self.velocity.x, 0.0, self.velocity.z);
        let t = self.t;
        if t < flight {
            return self.from_hips + self.velocity * t - Vec3::Y * (0.5 * self.gravity() * t * t) + self.drift(t);
        }
        let touch = self.from_hips + flat * flight + self.drift(flight);
        let touch = Vec3::new(touch.x, self.ground + self.touch_height, touch.z);
        let land = landed - flight;
        if t < landed {
            // Down: from the touchdown speed to rest at the depth, braking
            // hardest at first; forward, braked evenly to rest.
            let u = t - flight;
            let (v, n, h) = (self.touch_speed, self.braking, land);
            let r = 1.0 - (u / h).clamp(0.0, 1.0);
            let down = v * h * (2.0 / (n + 2.0) - r.powf(n + 1.0) + n * r.powf(n + 2.0) / (n + 2.0));
            return touch + flat * (u - 0.5 * u * u / h) - Vec3::Y * down;
        }
        let bottom = Vec3::new(touch.x, touch.y - self.depth, touch.z) + flat * (0.5 * land);
        // Hurt, it stays down a while first.
        let held = if self.hurt { HURT_HOLD } else { 0.0 };
        let up = smoothstep(((t - landed - held) / (self.ends[2] - landed - held)).clamp(0.0, 1.0));
        let standing = self.ground + self.stand_height;
        Vec3::new(bottom.x, bottom.y + (standing - bottom.y) * up, bottom.z)
    }

    /// Where the walker's root is now: under the hips as the standing pose
    /// has them, less the foot IK's drop once standing again.
    pub fn root(&self) -> Vec3 {
        let root = match self.roll.as_ref().filter(|_| self.is_landed()) {
            Some(roll) => self.rolling(roll, &self.rig).1,
            None => self.hips_root(),
        };
        // Landed, on the ground, how far under or over it the body is in
        // the pose: riding the hips down, the floor the sprung pose is kept
        // clear of was under the floor, and the feet went through it.
        if self.is_landed() { Vec3::new(root.x, self.ground, root.z) } else { root }
    }

    /// The root riding the hips (under them as standing has them, the drop
    /// added once stood): the roll eases its height from the squat's.
    fn hips_root(&self) -> Vec3 {
        let root = self.hips() - self.turn_now() * self.body.hips;
        if self.is_done() { root + Vec3::Y * self.drop } else { root }
    }

    /// Whether it has touched down (squatting, rolling or hurt).
    pub fn is_landed(&self) -> bool {
        self.t >= self.ends[0]
    }

    /// How deep the hips are below touching down, 0-1 of the landing's
    /// depth.
    fn squat(&self) -> f32 {
        let touch = self.ground + self.touch_height;
        ((touch - self.hips().y) / self.depth.max(1.0e-3)).clamp(0.0, 1.0)
    }

    /// The pose now, on `rig` (the one it was measured on), in the walker's
    /// pose frame at [`Self::root`] turned [`Self::facing`].
    pub fn pose(&self, rig: &RigGeometry) -> LocalPose {
        let (mut pose, root) = match self.roll.as_ref().filter(|_| self.is_landed()) {
            Some(roll) => self.rolling(roll, rig),
            None => (self.pose_at_root(self.hips_root(), rig), self.hips_root()),
        };
        // Landed, from the root on the ground ([`Self::root`]).
        if self.is_landed() {
            pose.root_translation.y += root.y - self.ground;
        }
        pose
    }

    /// [`Self::pose`] (not rolling) from the walker's root at `root`.
    fn pose_at_root(&self, root: Vec3, rig: &RigGeometry) -> LocalPose {
        let hips = self.hips();
        // In the frame it faces now: turning in the air, the leaving pose
        // and the feet's places in the world kept as they are at the start.
        let turn = self.turn_now();
        let back = turn.inverse();
        let [flight, _, _] = self.ends;
        let landed = self.t >= flight;
        let squat = self.squat();
        let (lean, arms) = if landed {
            let depth = self.depth * squat;
            let recovering = self.phase() == FallPhase::Recover;
            let arms = if recovering { 1.0 - smoothstep(((self.t - self.ends[1]) / (self.ends[2] - self.ends[1])).clamp(0.0, 1.0)) } else { 1.0 };
            // Hurt, over onto the hands; not facing a wall, whose room for
            // that pushed the hips back off it while still in the air.
            let per_depth = if self.hurt && self.wall.is_none() { HURT_LEAN_PER_DEPTH } else { LEAN_PER_DEPTH };
            // From the flight's arms over the landing's first moment: reaching
            // up for a lip it missed (a run up a wall too high), the arms
            // swung down to the landing's in a frame, a hand at 40 m/s.
            let into = smoothstep((flight / (LEGS_REACH * flight).max(0.15)).clamp(0.0, 1.0));
            let flying = if self.reaching { ARMS_REACHING } else { ARMS_FLYING };
            let flying = (flying.0 * into, flying.1 * into);
            let down = smoothstep(((self.t - flight) / ARMS_DOWN).clamp(0.0, 1.0));
            let landing = (ARMS_LANDING.0 * arms, ARMS_LANDING.1 * arms);
            (per_depth * depth, (flying.0 + (landing.0 - flying.0) * down, flying.1 + (landing.1 - flying.1) * down))
        } else {
            let into = smoothstep((self.t / (LEGS_REACH * flight).max(0.15)).clamp(0.0, 1.0));
            let flying = if self.reaching { ARMS_REACHING } else { ARMS_FLYING };
            (FLYING_LEAN * into, (flying.0 * into, flying.1 * into))
        };
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, lean, arms);
        // The trunk and arms from where they were as it left (holding a
        // ledge, swinging in a walk), not snapped to the flight's.
        // Leaving a jump, coasting on their own swing as the legs do: from
        // rest, the hands stopped dead (1.7 cm a frame off the jump's).
        let freed = smoothstep((self.t / ARMS_FREE).clamp(0.0, 1.0));
        if freed < 1.0 {
            let coast = self.t.min(LEGS_COAST);
            let swung = (coast - 0.5 * coast * coast / LEGS_COAST) / AHEAD;
            for bone in FREED_BONES {
                let from = match self.from_ahead.as_ref() {
                    Some(ahead) => self.from_pose.rotations[bone].slerp(ahead.rotations[bone], swung),
                    None => self.from_pose.rotations[bone],
                };
                pose.rotations[bone] = from.slerp(pose.rotations[bone], freed);
            }
        }
        // The hips in the pose's frame: down from standing as far as they
        // are below it (the root rides the hips' height above the ground
        // only once standing again).
        let pose_hips = back * (hips - root);
        pose.root_translation += pose_hips - self.body.hips;
        // The legs: flying, from where they left to the landing's shape
        // under the hips at touchdown; landed, the feet planted.
        let touch_hips = self.hips_at(flight);
        for (side, &(_, _, bone, _)) in LEGS.iter().enumerate() {
            let mut ankle = if landed {
                self.feet[side]
            } else {
                // Coasting on their own swing as they left, slowing evenly
                // to a stop over `LEGS_COAST`.
                let coast = self.t.min(LEGS_COAST);
                let from = self.from_ankles[side] - self.from_hips + self.ankle_velocities[side] * (coast - 0.5 * coast * coast / LEGS_COAST);
                let to = self.feet[side] - touch_hips;
                hips + from.lerp(to, smoothstep(self.legs_down(flight)))
            };
            place_ankle(&mut pose, rig, bone, back * (ankle - root) - pose_hips);
            // In the air facing a wall, the foot out as far as its knee
            // would go into it (a running jump's legs reaching ahead for
            // its landing put a knee 5 cm in, the hips held off).
            if let Some((top, out)) = self.wall.filter(|_| !landed) {
                for _ in 0..KNEE_PASSES {
                    let knee = root + turn * forward_kinematics_on(&pose, rig)[LEGS[side].1];
                    let short = KNEE_OFF_WALL - (knee - top).dot(out);
                    if short <= 0.0 || knee.y >= top.y {
                        break;
                    }
                    ankle += out * short;
                    place_ankle(&mut pose, rig, bone, back * (ankle - root) - pose_hips);
                }
            }
            // The feet level, turned with the body: from how they were as it
            // left (set level at once, a braced foot's toes, pitched up on the
            // wall, jumped 17 cm the frame it let go).
            let level = accumulate_world_rotations(&self.body.stood, rig)[bone];
            let left = accumulate_world_rotations(&self.from_pose, rig)[bone];
            // Sliding, braced on the face until the legs come down.
            let levelling = if self.is_sliding() { self.legs_down(flight) } else { (self.t / ARMS_FREE).clamp(0.0, 1.0) };
            let wanted = left.slerp(level, smoothstep(levelling));
            let now = accumulate_world_rotations(&pose, rig)[bone];
            pose.rotations[bone] = delta_after_world_turn(&pose, rig, bone, wanted * now.inverse());
            // Facing a wall, the ankle out as far as its toes would go into
            // it levelling (let go onto a step, coming in to it, 7 mm).
            if let Some((top, out)) = self.wall {
                let toe = root + turn * forward_kinematics_on(&pose, rig)[LEGS[side].3];
                let short = (top - toe).dot(out);
                if short > 0.0 && toe.y < top.y {
                    place_ankle(&mut pose, rig, bone, back * (ankle + out * short - root) - pose_hips);
                    let now = accumulate_world_rotations(&pose, rig)[bone];
                    pose.rotations[bone] = delta_after_world_turn(&pose, rig, bone, wanted * now.inverse());
                }
            }
        }
        // A wall it faces: the hands kept off it, sliding down it (let go
        // onto a step, the hands still up from the hang went 0.2 m into the
        // wall as it landed nearer it). Only under its top; and off it by
        // nothing at first, more as they are let go: held hooked on the
        // lip, kept off it at once, they jumped 4-15 cm the frame it let go.
        // And the face of the ledge leapt at, below its lip (leaping up, the
        // hands reaching for it went 2.6 cm into the wall under it).
        let target_wall = self.target.map(|ledge| (ledge.nearest(hips, 0.0), ledge.out));
        for (top, out) in self.wall.into_iter().chain(target_wall) {
            let at = forward_kinematics_on(&pose, rig);
            for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
                let wrist = root + turn * at[chain.wrist];
                let under = smoothstep(((top.y - wrist.y) / WALL_EASED).clamp(0.0, 1.0));
                let off = (wrist - top).dot(out);
                let short = HANDS_OFF_WALL * under * freed - off;
                let kept = wrist + out * short.max(0.0);
                // Sliding down its own wall, the hands held on the face,
                // eased onto it as they come under its top, no higher than
                // the arm reaches it from its shoulder (from overhead, the
                // shoulders 0.35-0.41 m out, the hands stayed 7-10 cm off),
                // brought down to that over `SLIDE_HANDS_IN` (pulled down as
                // they came under the top, the forearms went 21 m/s); let go
                // over `SLIDE_LET_GO` landed (at once, a hand went 17.7 m/s).
                let hold = if Some((top, out)) == self.wall { self.slide_hold(flight, landed) } else { 0.0 };
                let wanted = if hold > 0.0 {
                    let shoulder = root + turn * at[chain.shoulder];
                    let down = (wrist.y - (shoulder.y + SLIDE_ABOVE)).max(0.0) * smoothstep((self.t / SLIDE_HANDS_IN).clamp(0.0, 1.0));
                    kept.lerp(wrist + out * ((SLIDE_HANDS - off) * under) - Vec3::Y * down, hold)
                } else {
                    kept
                };
                // Only under the top: above it (a hand let go from over the
                // lip), there is no wall, and pushed out onto the face's
                // plane the hands jumped 6 cm the frame it leapt.
                if (wanted - wrist).length() > 1.0e-4 && wrist.y < top.y {
                    // The elbow's own way: turned to another, it jumped
                    // 12 cm as the hands began to be kept off. Sliding, down
                    // and out to its side: its own way, the arm passing
                    // straight on the way from overhead to the face, the
                    // elbow turned round at 16-19 m/s.
                    let own = (at[chain.elbow] - 0.5 * (at[chain.shoulder] + at[chain.wrist])).normalize_or(back * out);
                    let side = if chain.wrist == ArmChain::LEFT.wrist { 1.0 } else { -1.0 };
                    let sliding = (back * (out * SLIDE_ELBOW.0 - Vec3::Y) + rig.left() * (side * SLIDE_ELBOW.1)).normalize_or(own);
                    // Eased in from letting go over `SLIDE_POLE_IN`: at
                    // once, an elbow jumped 4.9 cm the frame it let go; over
                    // 0.35 s, still part its own way, it turned round at
                    // 14.8 m/s.
                    let pole = own.lerp(sliding, hold * smoothstep((self.t / SLIDE_POLE_IN).clamp(0.0, 1.0))).normalize_or(own);
                    solve_arm_toward_from(&mut pose, &at, chain, back * (wanted - root), pole, rig);
                }
            }
        }
        // Hurt, down onto its hands: each planted on the ground under its
        // shoulder as it goes down, held there, lifted as it gets up.
        let planted = self.hands_down();
        if planted > 0.0 {
            let free = pose;
            let at = forward_kinematics_on(&pose, rig);
            for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
                let shoulder = root + turn * at[chain.shoulder];
                let hand = Vec3::new(shoulder.x, self.ground + WRIST_ON_GROUND, shoulder.z);
                let pole = (rig.forward() * -0.5 + Vec3::Y * 0.2).normalize();
                solve_arm_toward_from(&mut pose, &at, chain, back * (hand - root), pole, rig);
                for bone in [chain.shoulder, chain.elbow, chain.wrist] {
                    pose.rotations[bone] = free.rotations[bone].slerp(pose.rotations[bone], planted);
                }
            }
        }
        pose
    }

    /// Hurt, how far the hands are down on the ground (0-1): going down over
    /// the second half of the landing, held, lifting over the first half of
    /// getting up.
    pub fn hands_down(&self) -> f32 {
        if !self.hurt || self.roll.is_some() || self.wall.is_some() {
            return 0.0;
        }
        let [flight, landed, stood] = self.ends;
        let lift = landed + HURT_HOLD;
        let ease = |t: f32, (a, b): (f32, f32)| smoothstep(((t - a) / (b - a)).clamp(0.0, 1.0));
        ease(self.t, (0.5 * (flight + landed), landed)) * (1.0 - ease(self.t, (lift, lift + 0.5 * (stood - lift))))
    }

    /// The hips in the world at `t`.
    fn hips_at(&self, t: f32) -> Vec3 {
        let mut at = self.clone();
        at.t = t;
        at.hips()
    }

    /// Each ankle planted on the ground, the world.
    pub fn feet(&self) -> [Vec3; 2] {
        self.feet
    }

    /// [`Self::pose`], each bone led ahead of its spring by how far the
    /// spring trails a steady motion (`jump::lead_of`).
    pub fn pose_led(&self, rig: &RigGeometry, springs: &BoneSet<crate::character::anim::math::SpringParams>) -> LocalPose {
        let mut pose = self.pose(rig);
        let mut posed: Vec<(f32, LocalPose)> = Vec::with_capacity(3);
        for bone in Bone::ALL {
            let lead = crate::character::anim::jump::lead_of(&springs[bone]);
            if lead <= 1.0e-4 {
                continue;
            }
            pose.rotations[bone] = match posed.iter().find(|(at, _)| (at - lead).abs() < 1.0e-4) {
                Some((_, ahead)) => ahead.rotations[bone],
                None => {
                    let mut later = self.clone();
                    later.advance(lead);
                    let ahead = later.pose(rig);
                    posed.push((lead, ahead));
                    ahead.rotations[bone]
                }
            };
        }
        pose
    }
}

/// The world turn `turn` given to `bone` (and all it carries).
fn pose_turn(pose: &mut LocalPose, rig: &RigGeometry, bone: Bone, turn: Quat) {
    pose.rotations[bone] = delta_after_world_turn(pose, rig, bone, turn);
}

/// The middle of all the pose's joints (the pose's frame).
fn centroid_of(pose: &LocalPose, rig: &RigGeometry) -> Vec3 {
    let at = forward_kinematics_on(pose, rig);
    at.iter().map(|(_, p)| *p).sum::<Vec3>() / Bone::ALL.len() as f32
}

/// `v` on the level.
fn flat_of(v: Vec3) -> Vec3 {
    Vec3::new(v.x, 0.0, v.z)
}

/// `a` and `b` mixed `w` of the way to `b`: each bone's turn, and the root.
fn mixed(a: &LocalPose, b: &LocalPose, w: f32) -> LocalPose {
    let mut pose = *a;
    for bone in Bone::ALL {
        pose.rotations[bone] = a.rotations[bone].slerp(b.rotations[bone], w);
    }
    pose.root_translation = a.root_translation.lerp(b.root_translation, w);
    pose
}

/// Cubic Hermite from `p0` (velocity `v0`) to `p1` (velocity `v1`) over
/// `span` seconds, at `s` (0-1) of it.
fn hermite(p0: Vec3, v0: Vec3, p1: Vec3, v1: Vec3, span: f32, s: f32) -> Vec3 {
    let (s2, s3) = (s * s, s * s * s);
    p0 * (2.0 * s3 - 3.0 * s2 + 1.0) + v0 * (span * (s3 - 2.0 * s2 + s)) + p1 * (3.0 * s2 - 2.0 * s3) + v1 * (span * (s3 - s2))
}

/// A squat landing from a drop of `drop` metres: how long it takes,
/// seconds, and how far the knees flex at most, degrees; between the
/// measured drops, and below them toward a step down's (the knees at
/// contact, half as long).
fn landing_for(drop: f32) -> (f32, f32) {
    let [(h0, t0, k0), ..] = LANDINGS;
    if drop <= h0 {
        let s = (drop / h0).clamp(0.0, 1.0);
        return (t0 * (0.5 + 0.5 * s), KNEE_AT_CONTACT + (k0 - KNEE_AT_CONTACT) * s);
    }
    for pair in LANDINGS.windows(2) {
        let ((ha, ta, ka), (hb, tb, kb)) = (pair[0], pair[1]);
        if drop <= hb {
            let s = (drop - ha) / (hb - ha);
            return (ta + (tb - ta) * s, ka + (kb - ka) * s);
        }
    }
    let (_, t, k) = LANDINGS[2];
    (t, k)
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

    /// What a fall measured, frame by frame.
    #[derive(Debug, Default)]
    struct Measured {
        /// The hips' velocity jump across touchdown, m/s (the posed hips).
        velocity_jump: f32,
        /// The hips' greatest deceleration after touchdown, in body weights.
        peak_load: f32,
        /// How long from touchdown to the bottom, seconds.
        landing: f32,
        /// The knees' most flexion, degrees.
        deepest_knee: f32,
        /// The most a planted ankle strays from its spot, metres.
        feet_off: f32,
        /// The lowest any joint goes below the ground, metres.
        below_ground: f32,
        /// The touchdown: the lower ankle's height above where it plants.
        touch_gap: f32,
        /// Standing at the end: the pose's largest distance from standing.
        from_standing: f32,
    }

    fn fell(height: f32, forward: f32) -> (Falling, Measured) {
        let (stood, rig) = real_stood();
        let yaw = 0.0;
        let turn = Quat::from_rotation_y(yaw);
        let velocity = turn * rig.forward() * forward;
        let mut falling = Falling::off(Vec3::new(0.0, height, 0.0), yaw, velocity, &stood, 0.0, 0.0, &stood, &rig);
        let mut m = Measured::default();
        // The feet just as they touch down: on the ground.
        {
            let mut touching = falling.clone();
            touching.t = falling.ends()[0] - 1.0e-5;
            let at = forward_kinematics_on(&touching.pose(&rig), &rig);
            m.touch_gap = (0..2).map(|side| (touching.root() + turn * at[LEGS[side].2] - falling.feet[side]).length()).fold(0.0, f32::max);
        }
        let mut hips = Vec::new();
        let flight = falling.ends()[0];
        let mut touched = None;
        loop {
            let pose = falling.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let world = BoneSet::from_fn(|bone| falling.root() + turn * at[bone]);
            hips.push((falling.t, world[Bone::Hips]));
            for bone in Bone::ALL {
                m.below_ground = m.below_ground.max(-world[bone].y);
            }
            if falling.t >= flight {
                touched.get_or_insert(hips.len() - 1);
                for side in 0..2 {
                    m.feet_off = m.feet_off.max((world[LEGS[side].2] - falling.feet[side]).length());
                }
            }
            for &(socket, knee, ankle, _) in &LEGS {
                let (thigh, shin) = ((world[knee] - world[socket]).normalize(), (world[ankle] - world[knee]).normalize());
                m.deepest_knee = m.deepest_knee.max(thigh.dot(shin).clamp(-1.0, 1.0).acos().to_degrees());
            }
            if falling.is_done() {
                let standing = forward_kinematics_on(&stood, &rig);
                m.from_standing = Bone::ALL.iter().map(|&bone| (at[bone] - standing[bone]).length()).fold(0.0, f32::max);
                break;
            }
            falling.advance(DT);
        }
        let touched = touched.expect("touched down");
        let velocity = |k: usize| (hips[k + 1].1 - hips[k].1) / DT;
        m.velocity_jump = (velocity(touched) - velocity(touched - 2)).length();
        let bottom = falling.ends()[1];
        m.landing = bottom - flight;
        m.peak_load = hips[touched..]
            .windows(3)
            .filter(|w| w[0].0 < bottom)
            .map(|w| ((w[2].1.y - 2.0 * w[1].1.y + w[0].1.y) / (DT * DT) + GRAVITY) / GRAVITY)
            .fold(0.0, f32::max);
        (falling, m)
    }

    /// Dropping from 1.8 and 2.4 m, past standing height, still or walking
    /// off, it rolls: nothing ever below the ground, the centroid's velocity
    /// continuous at touchdown, the forward speed kept while rolling, once
    /// round, and standing on its spot at the end.
    #[test]
    fn a_high_drop_is_rolled() {
        let (stood, rig) = real_stood();
        let turn = Quat::IDENTITY;
        for height in [1.8, 2.4] {
            for forward in [0.0, 1.4] {
                let name = format!("{height} m, {forward} m/s");
                let mut falling = Falling::off(Vec3::new(0.0, height, 0.0), 0.0, rig.forward() * forward, &stood, 0.0, 0.0, &stood, &rig);
                assert!(falling.rolls() && !falling.is_fatal(), "{name}: not rolling");
                let roll = falling.roll.clone().expect("rolling");
                let (flight, rolled_at) = (falling.ends()[0], falling.ends()[1]);
                let (mut below, mut slowest, mut centroids) = (0.0f32, f32::MAX, Vec::new());
                let mut deepest = (Bone::Hips, 0.0);
                let mut hips = Vec::new();
                let mut last_hips;
                loop {
                    let pose = falling.pose(&rig);
                    let at = forward_kinematics_on(&pose, &rig);
                    let root = falling.root();
                    for (bone, p) in at.iter() {
                        let depth = -(root + turn * *p).y;
                        if depth > below {
                            (below, deepest) = (depth, (bone, falling.t - flight));
                        }
                    }
                    let centroid = root + turn * centroid_of(&pose, &rig);
                    centroids.push((falling.t, centroid));
                    hips.push((falling.t, root + turn * at[Bone::Hips]));
                    last_hips = accumulate_world_rotations(&pose, &rig)[Bone::Hips];
                    if falling.is_done() {
                        break;
                    }
                    falling.advance(DT);
                }
                for w in centroids.windows(2) {
                    let (t, v) = (w[0].0, flat_of(w[1].1 - w[0].1) / DT);
                    if t > flight + ROLL_ABSORB + ROLL_TUCK && t < rolled_at {
                        slowest = slowest.min(v.dot(roll.way));
                    }
                }
                // The hips' velocity across touchdown (the feet stop dead
                // there: the centroid's, legs and all, steps 2.4 m/s), and
                // their greatest acceleration from the tucking on.
                let touched = hips.iter().position(|(t, _)| *t >= flight).expect("touched down");
                let velocity = |k: usize| (hips[k + 1].1 - hips[k].1) / DT;
                let jump = (velocity(touched) - velocity(touched - 2)).length();
                let tucking = flight + ROLL_ABSORB;
                let (mut hardest, mut hardest_at) = (0.0f32, 0.0);
                for (k, w) in hips.windows(3).enumerate() {
                    let a = ((w[2].1 - 2.0 * w[1].1 + w[0].1) / (DT * DT)).length();
                    if w[0].0 >= tucking && a > hardest {
                        (hardest, hardest_at) = (a, hips[k].0 - flight);
                    }
                }
                // Rolling, the hips swing round the tucked centroid at about
                // 6 rad/s (36-46 m/s² measured); resting on the lowest joint
                // frame by frame, they jerked at 557.
                assert!(hardest < 50.0, "{name}: the hips accelerated {hardest:.1} m/s² {hardest_at:.3} s after touchdown");
                let standing = accumulate_world_rotations(&stood, &rig)[Bone::Hips];
                assert!(below < 1.0e-3, "{name}: {:?} {below:.4} m below the ground {:.3} s after touchdown (rolled at {:.3})", deepest.0, deepest.1, rolled_at - flight);
                assert!(jump < 0.5, "{name}: the centroid's velocity jumped {jump:.3} m/s at touchdown");
                assert!(slowest > 0.95 * roll.speed, "{name}: rolling at {slowest:.2} of {:.2} m/s", roll.speed);
                assert!(last_hips.dot(standing).abs() > 0.9999, "{name}: not once round, the pelvis {:.4} off standing", last_hips.angle_between(standing));
                assert!((falling.root() - roll.end_root).length() < 1.0e-3, "{name}: standing {:.4} m off its spot", (falling.root() - roll.end_root).length());
                assert!(roll.radius > 0.2 && roll.radius < 0.8, "{name}: tucked {:.3} m round", roll.radius);
            }
        }
    }

    /// Dropping past [`FATAL_DROP`] it does not land; below it, it does.
    #[test]
    fn a_fatal_drop_is_not_landed() {
        let (stood, rig) = real_stood();
        let off = |height: f32| Falling::off(Vec3::new(0.0, height, 0.0), 0.0, Vec3::ZERO, &stood, 0.0, 0.0, &stood, &rig);
        assert!(off(4.5).is_fatal(), "4.5 m landed");
        assert!(!off(3.5).is_fatal(), "3.5 m not landed");
        assert!(!off(1.2).rolls() && off(2.0).rolls(), "rolls from the wrong height");
    }

    /// Running on fast off a low drop it rolls, as from a high one; walking
    /// it squats; off a drop lower than any landing measured it squats
    /// whatever its speed.
    #[test]
    fn a_fast_run_off_a_low_drop_is_rolled() {
        let (stood, rig) = real_stood();
        let off = |height: f32, speed: f32| Falling::off(Vec3::new(0.0, height, 0.0), 0.0, rig.forward() * speed, &stood, 0.0, 0.0, &stood, &rig);
        assert!(off(1.0, 3.5).rolls(), "1.0 m at 3.5 m/s squatted");
        assert!(!off(1.0, 1.4).rolls(), "1.0 m walking rolled");
        assert!(!off(0.5, 4.0).rolls(), "0.5 m rolled");
        // And rolls well: nothing below the ground, standing at the end.
        let mut falling = off(1.0, 3.5);
        let mut below = 0.0f32;
        loop {
            let at = forward_kinematics_on(&falling.pose(&rig), &rig);
            let root = falling.root();
            below = below.max(at.iter().map(|(_, p)| -(root + *p).y).fold(0.0, f32::max));
            if falling.is_done() {
                break;
            }
            falling.advance(DT);
        }
        assert!(below < 1.0e-3, "a joint {below:.4} m below the ground");
    }

    /// From 3.5 m, past the measured drops and short of fatal, it lands
    /// hurt: down onto its hands (the wrists on the ground under the
    /// shoulders), the knees bent deep, held down, then up slowly to
    /// standing; nothing below the ground.
    #[test]
    fn a_hurt_landing_goes_down_onto_the_hands() {
        let (stood, rig) = real_stood();
        let mut falling = Falling::off(Vec3::new(0.0, 3.5, 0.0), 0.0, Vec3::ZERO, &stood, 0.0, 0.0, &stood, &rig);
        assert!(falling.is_hurt() && !falling.rolls() && !falling.is_fatal(), "3.5 m not landed hurt");
        let (mut below, mut down_for, mut wrists_off, mut deepest_knee) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        let ended;
        loop {
            let pose = falling.pose(&rig);
            let at = forward_kinematics_on(&pose, &rig);
            let root = falling.root();
            below = below.max(at.iter().map(|(_, p)| -(root + *p).y).fold(0.0, f32::max));
            if falling.hands_down() >= 1.0 {
                down_for += DT;
                for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
                    wrists_off = wrists_off.max(((root + at[chain.wrist]).y - WRIST_ON_GROUND).abs());
                }
            }
            for &(socket, knee, ankle, _) in &LEGS {
                let (thigh, shin) = ((at[knee] - at[socket]).normalize(), (at[ankle] - at[knee]).normalize());
                deepest_knee = deepest_knee.max(thigh.dot(shin).clamp(-1.0, 1.0).acos().to_degrees());
            }
            if falling.is_done() {
                ended = Some(at);
                break;
            }
            falling.advance(DT);
        }
        let standing = forward_kinematics_on(&stood, &rig);
        let from_standing = Bone::ALL.iter().map(|&bone| (ended.expect("ended")[bone] - standing[bone]).length()).fold(0.0, f32::max);
        assert!(below < 1.0e-3, "a joint {below:.4} m below the ground");
        assert!(wrists_off < 0.01, "a planted wrist {wrists_off:.4} m off the ground");
        assert!(down_for > 0.75 * HURT_HOLD, "on its hands only {down_for:.2} s");
        assert!(deepest_knee > HURT_KNEE - 12.0, "the knees at most {deepest_knee:.0}°");
        assert!(from_standing < 0.01, "{from_standing:.4} m from standing at the end");
    }

    /// Dropping from 0.9 and 1.6 m (a measured drop, and between them),
    /// still or walking
    /// off, it touches down at free fall's speed, its feet meeting the
    /// ground with the hips' velocity continuous, lands as long and as deep
    /// as measured, the load at the hips bounded, nothing through the
    /// ground, and stands again.
    #[test]
    fn a_drop_is_landed_as_measured() {
        // Peak loads: a squat landing's from 0.9 m about 4 body weights
        // (3.2 for a parkour landing from 0.75 m, 5.2 a stiff one); from
        // 1.8 m, where rolling is the guidance, more.
        for (height, landing, knee, load) in [(0.9, 0.377, 116.0, 4.5), (1.6, 0.344, 123.8, 6.5)] {
            for forward in [0.0, 1.4] {
                // The hips leave from standing on a top `height` up.
                let (falling, m) = fell(height, forward);
                let name = format!("{height} m, {forward} m/s");
                let free_fall = (2.0 * GRAVITY * (falling.from_hips.y - falling.touch_height)).sqrt();
                assert!((falling.touch_speed() - free_fall).abs() < 0.01, "{name}: touched down at {:.2} m/s", falling.touch_speed());
                assert!(m.touch_gap.abs() < 0.01, "{name}: the lower ankle {:.4} m off the ground as it touched down", m.touch_gap);
                assert!(m.velocity_jump < 0.15, "{name}: the hips' velocity jumped {:.3} m/s at touchdown", m.velocity_jump);
                assert!((m.landing - landing).abs() < 0.03, "{name}: landed over {:.3} s", m.landing);
                assert!((m.deepest_knee - knee).abs() < 12.0, "{name}: the knees at most {:.0}°", m.deepest_knee);
                assert!(m.peak_load < load, "{name}: {:.2} body weights at the hips", m.peak_load);
                assert!(m.feet_off < 1.0e-3, "{name}: a planted ankle {:.4} m off its spot", m.feet_off);
                assert!(m.below_ground < 1.0e-3, "{name}: a joint {:.4} m below the ground", m.below_ground);
                assert!(m.from_standing < 0.01, "{name}: {:.4} m from standing at the end", m.from_standing);
            }
        }
    }
}
