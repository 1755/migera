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
/// The trunk and arms go from where they were as it left to the flight's
/// over this long, seconds.
const ARMS_FREE: f32 = 0.35;
/// The trunk's and arms' bones: let go from a hang, the trunk leant toward
/// the wall would snap upright.
const FREED_BONES: [Bone; 14] = [
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
    #[cfg_attr(not(test), allow(dead_code))]
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
    /// Rolling, not squatting; landing hurt; the rig it was measured on.
    roll: Option<Roll>,
    hurt: bool,
    rig: std::sync::Arc<RigGeometry>,
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
            roll: None,
            hurt: false,
            rig: std::sync::Arc::new(rig.clone()),
        };
        falling.plan();
        // Rolling past standing height, or moving on too fast to brake in a
        // squat; not hurt.
        let drop = falling.dropped();
        let braking = flat_of(velocity).length() / landing_for(drop).0;
        if !falling.hurt && (drop > ROLL_DROP || (drop > ROLL_LOW && braking > MOST_BRAKING)) {
            falling.plan_roll(rig);
        }
        falling
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
    /// Swept over the last frame: the shoulders where they passed the lip's
    /// height (the frame's start, if already under it). Tested at the
    /// frame's end alone, a frame of a tenth of a second (a hitch) fell
    /// past the 0.36 m the lip is in reach and on to the ground.
    pub fn catches(&self, ledges: &[Ledge], rig: &RigGeometry) -> Option<Ledge> {
        if !self.airborne() || self.hips_velocity().y >= 0.0 {
            return None;
        }
        let pose = self.pose(rig);
        let at = forward_kinematics_on(&pose, rig);
        let root = self.root();
        let shoulders = root + self.turn * (0.5 * (at[ArmChain::LEFT.shoulder] + at[ArmChain::RIGHT.shoulder]));
        let arm = (at[ArmChain::LEFT.elbow] - at[ArmChain::LEFT.shoulder]).length() + (at[ArmChain::LEFT.wrist] - at[ArmChain::LEFT.elbow]).length();
        let hips = root + self.turn * at[Bone::Hips];
        // The frame's start, the body moved as the hips did.
        let before = shoulders + self.hips_at(self.last_t) - hips;
        let forward = self.turn * rig.forward();
        ledges.iter().copied().find(|ledge| {
            let height = ledge.height();
            if shoulders.y > height {
                return false;
            }
            let passing = if before.y > height { before.lerp(shoulders, (before.y - height) / (before.y - shoulders.y)) } else { before };
            let lip = ledge.nearest(passing, 0.3);
            let out = ledge.out_of(hips);
            forward.dot(-ledge.out) > std::f32::consts::FRAC_1_SQRT_2
                && (0.05..0.8).contains(&out)
                && (lip - passing).length() <= CATCH_REACH * arm
        })
    }

    /// How far it drops, standing height to standing height, metres.
    pub fn dropped(&self) -> f32 {
        self.from_hips.y - self.drop_hips() - self.ground
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
        let drop = self.from_hips.y - self.drop_hips() - self.ground;
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
        let flight = ((up + (up * up + 2.0 * GRAVITY * fall.max(0.0)).sqrt()) / GRAVITY).max(1.0e-3);
        self.touch_speed = GRAVITY * flight - up;
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
        let rest = self.from_hips + flat * (flight + 0.5 * land);
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
            return (squat.pose(rig), squat.root());
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
            squat.pose(rig)
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
        let squat = squat.root().y;
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

    /// The facing (radians about `+Y`).
    pub fn facing(&self) -> f32 {
        self.yaw
    }

    /// The hips in the world now.
    pub fn hips(&self) -> Vec3 {
        if let Some(roll) = self.roll.as_ref().filter(|_| self.t >= self.ends[0]) {
            let (pose, root) = self.rolling(roll, &self.rig);
            return root + self.turn * forward_kinematics_on(&pose, &self.rig)[Bone::Hips];
        }
        let [flight, landed, _] = self.ends;
        let flat = Vec3::new(self.velocity.x, 0.0, self.velocity.z);
        let t = self.t;
        if t < flight {
            return self.from_hips + self.velocity * t - Vec3::Y * (0.5 * GRAVITY * t * t);
        }
        let touch = self.from_hips + flat * flight;
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
        if let Some(roll) = self.roll.as_ref().filter(|_| self.t >= self.ends[0]) {
            return self.rolling(roll, &self.rig).1;
        }
        let root = self.hips() - self.turn * self.body.hips;
        if self.is_done() { root + Vec3::Y * self.drop } else { root }
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
        if let Some(roll) = self.roll.as_ref().filter(|_| self.t >= self.ends[0]) {
            return self.rolling(roll, rig).0;
        }
        let hips = self.hips();
        let root = self.root();
        let back = self.turn.inverse();
        let [flight, _, _] = self.ends;
        let landed = self.t >= flight;
        let squat = self.squat();
        let (lean, arms) = if landed {
            let depth = self.depth * squat;
            let recovering = self.phase() == FallPhase::Recover;
            let arms = if recovering { 1.0 - smoothstep(((self.t - self.ends[1]) / (self.ends[2] - self.ends[1])).clamp(0.0, 1.0)) } else { 1.0 };
            let per_depth = if self.hurt { HURT_LEAN_PER_DEPTH } else { LEAN_PER_DEPTH };
            (per_depth * depth, (ARMS_LANDING.0 * arms, ARMS_LANDING.1 * arms))
        } else {
            let into = smoothstep((self.t / (LEGS_REACH * flight).max(0.15)).clamp(0.0, 1.0));
            let flying = if self.reaching { ARMS_REACHING } else { ARMS_FLYING };
            (FLYING_LEAN * into, (flying.0 * into, flying.1 * into))
        };
        let mut pose = crate::character::anim::jump::upper(&self.body.stood, rig, lean, arms);
        // The trunk and arms from where they were as it left (holding a
        // ledge, swinging in a walk), not snapped to the flight's.
        let freed = smoothstep((self.t / ARMS_FREE).clamp(0.0, 1.0));
        if freed < 1.0 {
            for bone in FREED_BONES {
                pose.rotations[bone] = self.from_pose.rotations[bone].slerp(pose.rotations[bone], freed);
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
            let ankle = if landed {
                self.feet[side]
            } else {
                let from = self.from_ankles[side] - self.from_hips;
                let to = self.feet[side] - touch_hips;
                hips + from.lerp(to, smoothstep((self.t / flight).clamp(0.0, 1.0)))
            };
            place_ankle(&mut pose, rig, bone, back * (ankle - root) - pose_hips);
            // The feet level, turned with the body: from how they were as it
            // left (set level at once, a braced foot's toes, pitched up on the
            // wall, jumped 17 cm the frame it let go).
            let now = accumulate_world_rotations(&pose, rig)[bone];
            let level = accumulate_world_rotations(&self.body.stood, rig)[bone];
            let left = accumulate_world_rotations(&self.from_pose, rig)[bone];
            let wanted = left.slerp(level, smoothstep((self.t / ARMS_FREE).clamp(0.0, 1.0)));
            pose.rotations[bone] = delta_after_world_turn(&pose, rig, bone, wanted * now.inverse());
        }
        // Hurt, down onto its hands: each planted on the ground under its
        // shoulder as it goes down, held there, lifted as it gets up.
        let planted = self.hands_down();
        if planted > 0.0 {
            let free = pose;
            let at = forward_kinematics_on(&pose, rig);
            for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
                let shoulder = root + self.turn * at[chain.shoulder];
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
        if !self.hurt || self.roll.is_some() {
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
