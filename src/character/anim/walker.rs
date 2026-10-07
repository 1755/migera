//! A walking character: the gait driven from a desired speed and heading,
//! moved by its own rendered feet (root motion), balancing against pushes,
//! and, with a ragdoll, falling and getting up.
//!
//! Add [`WalkerPlugin`] and put a [`Walker`] on a bound humanoid
//! (`humanoid::spawn_gltf_humanoid`); [`attach_walkers`] gives it the rest of
//! the stack once its skeleton exists. Steer it by writing [`Walker`]: its
//! `speed`, its [`Steer`], a push. The character's world position and yaw
//! are [`WalkerState`]'s to write, not the caller's: root motion owns them.
//!
//! Moved out of `examples/character_gallery.rs` (2026-10-02), where it had
//! grown as gallery-local systems, so other examples and a game drive a
//! character the same way.

use std::f32::consts::TAU;

use bevy::math::Vec2;
use bevy::prelude::*;

use super::gait::{cycle_of, walk_pose_on, GaitParams};
use super::ground::FlatGround;
use super::humanoid::{FacingCorrection, HumanoidSet};
use super::phase::{GaitPhase, PhaseLayer};
use super::plugin::{AnimArmIk, AnimFootIk, AnimGround, AnimPose, AnimSet, Landing};
use super::rig::{LocalPose, RigGeometry};
use super::stance::{stance_on, stance_on_rig, DEFAULT_KNEE_FLEX};
use super::walk_balance::{self, WalkBalance};
use super::{approach, balance, facing, locomotion, lookat, poses, sitting, transition};
use super::{AnimPhaseLayer, AnimSprings, AnimTarget, AnimTargetAsset, Ragdoll, RagdollSet, FALL_DAMPING, FALL_TONE};
use crate::character::{Bone, HumanoidSkeleton};

/// Drives every [`Walker`].
pub struct WalkerPlugin;

impl Plugin for WalkerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, attach_walkers.after(HumanoidSet::Bind))
            .add_systems(Update, drive_walkers.in_set(AnimSet::Target).in_set(WalkerSet::Drive))
            .add_systems(Update, ride_rendered_feet.after(AnimSet::Spring).before(AnimSet::Ik).in_set(WalkerSet::Ride))
            // With a ragdoll (`AnimRagdollPlugin`): falls, get-ups, and the
            // fallen body carrying the character. Inert without one.
            .add_systems(
                Update,
                (
                    fall_when_uncaught.after(WalkerSet::Drive).before(RagdollSet::Hit),
                    get_up_when_rested.before(RagdollSet::Hit),
                    follow_the_fallen_body.before(WalkerSet::Ride),
                ),
            );
    }
}

/// Where the walker's systems run, so consumers can order around them.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WalkerSet {
    /// The gait, the balance and the heading, in `AnimSet::Target`.
    Drive,
    /// Root motion, after the springs, before the IK.
    Ride,
}

/// How a walker turns.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Steer {
    /// Holds its heading.
    #[default]
    Straight,
    /// Walks a steady circle at this many radians per second (positive
    /// turns left).
    Circle(f32),
    /// Turns toward `yaw` (radians about `+Y`, zero this crate's `-Z`
    /// forward) at most `rate` radians per second, then holds it.
    Toward { yaw: f32, rate: f32 },
}

/// What a walking character is asked to do. Written by its owner.
#[derive(Component, Debug, Clone)]
pub struct Walker {
    /// The named pose it stands in and walks on (`poses::by_name`;
    /// `getup:sit|squat|quadruped|half_kneel|side_sit_left|side_sit_right`
    /// holds one get-up key). Its asset `anim/<name>.pose.ron` replaces it
    /// once loaded, so editing the file updates the running character.
    pub pose: String,
    /// The speed asked for, m/s. Zero stands; the transition walks it in
    /// and out (Winter §11.3.3). A walk, or above `run::changeover_speed`
    /// a run (`run`), the speed eased toward above a walk.
    pub speed: f32,
    pub steer: Steer,
    /// A world point to look at, else ahead.
    pub look_at: Option<Vec3>,
    /// A world point for the left hand to reach for.
    pub reach: Option<Vec3>,
    /// Pushes not yet delivered, m/s along the character's (forward,
    /// left): see [`Walker::push`].
    pub pushes: Vec<Vec2>,
    /// Let a ragdolled walker fall now, as if no step could catch it.
    pub fall_now: bool,
    /// The falling joints' damping, per second.
    pub fall_damping: f32,
    /// How long a fallen walker lies still before getting up, seconds.
    pub getup_delay: f32,
    /// Sit down this way (`sitting`), or stand up again (`None`). A walking
    /// character stops first; a seated one asked to sit another way stands
    /// up first.
    pub sit: Option<sitting::Sitting>,
    /// The seat height of the chair it sits on where it stands, metres.
    pub chair_height: f32,
    /// A chair to sit on, sitting a chair's way: it walks there and turns
    /// round first (`approach`). `None` sits where it stands.
    pub chair: Option<approach::Chair>,
    /// Walk aside at this speed, m/s, positive to its left: the side
    /// shuffle (`shuffle`), only asked neither to walk nor to sit. Turning
    /// the other way, or walking on, it stops first.
    pub aside: f32,
    /// One step aside, metres, positive to its left, from a stand: the
    /// standing balance's side step and close
    /// (`balance::Balance::step_aside`). Taken when read; a walk asked for
    /// meanwhile starts once the feet have closed.
    pub step_aside: f32,
    /// Jump this high, metres of the centre of mass's rise above take-off,
    /// and this far, metres the feet land ahead (`jump`), from a stand.
    /// Taken when read; asked while moving, sitting or already jumping, it
    /// is dropped. A walk asked meanwhile starts once the jump has landed
    /// and stood.
    pub jump: Option<super::jump::JumpAsk>,
    /// Sneak: crouch this deep, 0-1, flat or on the toes (`sneak`). It
    /// crouches standing, walking (no faster than `sneak::FASTEST`, and a
    /// run slows to a walk first) or shuffling aside, and changes crouch on
    /// the move; asked to sit, jump or step aside, it stands up first.
    pub sneak: super::sneak::Sneak,
    /// The ladder to climb (`ladder`).
    pub ladder: Option<super::ladder::Ladder>,
    /// Climb [`Walker::ladder`]: `Up` walks to it, gets on and climbs to its
    /// top (stepping off onto its landing, if it has one), `Down` climbs down
    /// and steps off, `Slide` slides down; `None` holds on where it is, its
    /// step done. Off the ladder, `Up` from the floor and `Down` or `Slide`
    /// from its landing get on. On it, nothing else asked of it (walking,
    /// sitting, jumping, sneaking) is done. A landing must be in the
    /// walker's ground ([`super::ladder::LadderGround`]) for it to stand
    /// there.
    pub climb: Option<super::ladder::Climb>,
    /// The ledge to act on (`parkour`).
    pub ledge: Option<super::parkour::Ledge>,
    /// Asked of [`Walker::ledge`]: `Grab` walks under it, jumps and hangs
    /// from it (`parkour::hang`). Out of a standing jump's reach, the ask is
    /// dropped. Hanging, `ClimbUp` climbs onto its top and stands there (on
    /// the walker's ground, [`super::parkour::LedgeGround`]); the ask is
    /// dropped once taken. Hanging, nothing else asked of it is done.
    pub hang: Option<super::parkour::hang::HangAsk>,
}

impl Default for Walker {
    fn default() -> Self {
        Self {
            pose: "relaxed_stand".into(),
            speed: 0.0,
            steer: Steer::Straight,
            look_at: None,
            reach: None,
            pushes: Vec::new(),
            fall_now: false,
            fall_damping: FALL_DAMPING,
            getup_delay: 1.0,
            sit: None,
            chair_height: sitting::CHAIR_HEIGHT,
            chair: None,
            aside: 0.0,
            step_aside: 0.0,
            jump: None,
            sneak: super::sneak::Sneak::STANDING,
            ladder: None,
            climb: None,
            ledge: None,
            hang: None,
        }
    }
}

/// Whether a walker stands, sits, or is moving between the two.
#[derive(Debug, Clone, Default)]
pub enum Posture {
    #[default]
    Standing,
    /// Going through `keys` from `from`, `elapsed` seconds in, sitting `how`:
    /// down (`down`) ends seated; up ends with a last blend into the
    /// standing pose over `stand_seconds`.
    Moving { from: LocalPose, keys: Vec<sitting::SitKey>, elapsed: f32, how: sitting::Sitting, down: bool, stand_seconds: f32 },
    /// Sitting `how`, in `pose`.
    Seated { how: sitting::Sitting, pose: LocalPose },
}

impl Posture {
    /// Whether it stands (not seated, nor on its way).
    pub fn is_standing(&self) -> bool {
        matches!(self, Posture::Standing)
    }

    /// Whether it sits, or is sitting down or standing up, on a chair.
    pub fn on_chair(&self) -> bool {
        match self {
            Posture::Standing => false,
            Posture::Seated { how, .. } | Posture::Moving { how, .. } => how.on_chair(),
        }
    }

    /// Advances it by `dt` toward what `wanted` asks, and returns the pose
    /// to draw if not standing, with which feet stay planted. `stood` is the
    /// standing pose (where standing up ends); `ready` whether a walker
    /// standing may start sitting (stopped).
    pub fn advance(
        &mut self,
        wanted: Option<sitting::Sitting>,
        chair: sitting::Seat,
        stood: &LocalPose,
        rig: &RigGeometry,
        ready: bool,
        dt: f32,
    ) -> Option<(LocalPose, [bool; 2])> {
        // Start a move if asked to.
        match (&*self, wanted) {
            // The keys refined once, as the move starts (`sitting::refined`).
            (Posture::Standing, Some(how)) if ready => {
                let keys = sitting::refined(stood, sitting::sitting_down(how, rig, stood, chair), rig);
                *self = Posture::Moving { from: *stood, keys, elapsed: 0.0, how, down: true, stand_seconds: 0.0 };
            }
            (Posture::Seated { how, pose }, wanted) if wanted != Some(*how) => {
                // Up, the last move is into the standing pose.
                let mut keys = sitting::standing_up(*how, rig, stood, chair);
                keys.push(sitting::SitKey { pose: *stood, seconds: sitting::stand_seconds(*how) });
                let keys = sitting::refined(pose, keys, rig);
                *self = Posture::Moving { from: *pose, keys, elapsed: 0.0, how: *how, down: false, stand_seconds: 0.0 };
            }
            _ => {}
        }
        match self {
            Posture::Standing => None,
            Posture::Seated { pose, .. } => Some((*pose, feet_down(pose, pose, rig))),
            Posture::Moving { from, keys, elapsed, how, down, .. } => {
                *elapsed += dt;
                let mut left = *elapsed;
                let mut previous = *from;
                for (pose, seconds) in keys.iter().map(|key| (key.pose, key.seconds)) {
                    if left < seconds {
                        let t = (left / seconds).clamp(0.0, 1.0);
                        let eased = t * t * (3.0 - 2.0 * t);
                        let planted = feet_down(&previous, &pose, rig);
                        let mut blended = super::rig::blend_in_world(&previous, &pose, eased, rig);
                        // The root carried so the planted feet stay exactly
                        // where both keys have them: the rotations swing
                        // while the root moves straight, and between two keys
                        // with the feet in one place they slid up to 82 mm
                        // standing up from a chair, and dipped 27 mm into
                        // the floor sitting down.
                        // Each held where the two keys have it, between them
                        // as the blend goes: held where the first key had it,
                        // a foot "planted" within 3 cm ended its segment up to
                        // 3 cm off the next key, and the body jumped there.
                        let at = super::rig::forward_kinematics_on(&blended, rig);
                        let (was, will) = (super::rig::forward_kinematics_on(&previous, rig), super::rig::forward_kinematics_on(&pose, rig));
                        let feet = [Bone::LeftFoot, Bone::RightFoot];
                        let held: Vec<Vec3> = feet
                            .iter()
                            .zip(planted)
                            .filter(|(_, down)| *down)
                            .map(|(&foot, _)| was[foot].lerp(will[foot], eased) - at[foot])
                            .collect();
                        if !held.is_empty() {
                            blended.root_translation += held.iter().sum::<Vec3>() / held.len() as f32;
                        }
                        return Some((sitting::clear_floor(blended, rig), planted));
                    }
                    left -= seconds;
                    previous = pose;
                }
                // Through every key.
                let last = previous;
                *self = if *down { Posture::Seated { how: *how, pose: last } } else { Posture::Standing };
                match self {
                    Posture::Standing => None,
                    _ => Some((last, feet_down(&last, &last, rig))),
                }
            }
        }
    }
}

/// Which feet (left, right) stay planted from `from` to `to`: those on the
/// floor in both that move less than 3 cm between them.
fn feet_down(from: &LocalPose, to: &LocalPose, rig: &RigGeometry) -> [bool; 2] {
    let (a, b) = (super::rig::forward_kinematics_on(from, rig), super::rig::forward_kinematics_on(to, rig));
    [Bone::LeftFoot, Bone::RightFoot].map(|foot| {
        let low = |pose: &LocalPose| super::getup::contact_height(pose, rig, super::getup::Contact::Foot(foot)) < 0.03;
        a[foot].distance(b[foot]) < 0.03 && low(from) && low(to)
    })
}

impl Walker {
    /// Shoves the character, changing its centre of mass's velocity by
    /// `velocity` m/s along its own (forward, left): standing it sways and
    /// steps, walking its next footfalls move.
    pub fn push(&mut self, velocity: Vec2) {
        self.pushes.push(velocity);
    }
}

/// Standing on a ladder's landing this near its spot there, metres, and
/// this near its facing, radians, it gets on where it stands.
const LANDING_NEAR: f32 = 0.6;
const LANDING_NEAR_HEADING: f32 = 0.5;

/// Going to grab a ledge, it walks first to a point this far out in front
/// of its spot, metres (within [`LEDGE_AT_LEAD_IN`] of it counts), and
/// from there straight in.
const LEDGE_LEAD_IN: f32 = 1.2;
const LEDGE_AT_LEAD_IN: f32 = 0.3;

/// How long both feet stay planted after standing up, seconds: several
/// times the legs' 0.015 s spring half-life, for the extension to settle.
const STOOD_HOLD: f32 = 0.3;

/// How far through its stance a gait's foot bears weight for the foot IK's
/// hips (`AnimFootIk::gait_bearing`): past it, the foot is rolling off its
/// toes. A run's foot in its last frame down, pinned behind a body
/// speeding up, dropped the hips 21 mm for a frame.
pub const BEARING: f32 = 0.85;

/// A walker's own state: where it is and faces (root motion's to write),
/// how far into walking it is, and what its last frame rendered.
#[derive(Component)]
pub struct WalkerState {
    /// Root motion. `Drive`: the gait publishes its velocity and turn, and
    /// [`ride_rendered_feet`] moves the body from the pose actually
    /// rendered.
    pub locomotion: locomotion::Locomotion,
    pub facing: facing::Facing,
    pub transition: transition::Transition,
    pub look: lookat::LookAt,
    pub stride: Stride,
    /// Standing, sitting, or on the way between ([`Walker::sit`]).
    pub posture: Posture,
    /// The walk to [`Walker::chair`], and the chair with the spot it walks
    /// to.
    pub approach: approach::Approach,
    pub seat: Option<(approach::Chair, Vec3)>,
    pub walked: Walked,
    /// How far off its spot the walk to the chair stopped: the seat's
    /// [`sitting::Seat::back`] and [`sitting::Seat::across`], held from
    /// sitting down to standing.
    pub sit_offset: Vec2,
    /// Seconds both feet stay planted after standing up ([`STOOD_HOLD`]).
    pub stood_hold: f32,
    /// A step aside under way ([`Walker::step_aside`]), until the feet have
    /// closed.
    pub stepping_aside: bool,
    /// Shuffling aside ([`Walker::aside`]), toward +1 its left or -1 its
    /// right, until the shuffle has stopped.
    pub shuffle: Option<f32>,
    /// The speed the shuffle's stride and width are for, m/s: the last
    /// asked.
    pub shuffle_speed: f32,
    /// How much of the shuffle's way is forward, as its stride is laid
    /// (`shuffle::shuffling`): the last asked, eased.
    pub shuffle_ahead: f32,
    /// How far the body is turned off where it was steered to face, to walk
    /// its way going aside and forward at once, radians (left positive).
    pub strafe: f32,
    /// The speed handed to the gait, m/s: the speed asked, eased above a
    /// walk at `run::ACCELERATION` and `run::DECELERATION`.
    pub pace: f32,
    /// Walking or running (`run`), and the one step that changes between
    /// them.
    pub gaits: super::run::Gaits,
    /// A jump under way ([`Walker::jump`]), until it has landed and stood,
    /// or from a run, landed and run on.
    pub jump: Option<super::jump::Jump>,
    /// A jump asked while running, waiting for the next foot to come down:
    /// that foot takes off.
    pub leap_asked: Option<super::jump::JumpAsk>,
    /// The crouch it is in or on its way to ([`Walker::sneak`]).
    pub crouching: super::sneak::Crouching,
    /// On a ladder ([`Walker::climb`]), from getting on to stepping off.
    pub climbing: Option<super::ladder::Climbing>,
    /// The ladder it walks to, and the spot it gets on from.
    /// Whether that spot is on the ladder's landing, at its top.
    pub ladder_spot: Option<(super::ladder::Ladder, bool, Vec3)>,
    /// Grabbing a ledge and hanging from it ([`Walker::hang`]).
    pub hanging: Option<super::parkour::Hanging>,
    /// The ledge it walks under, the spot it jumps from, and whether it has
    /// come round in front of it to walk straight in (`LEDGE_LEAD_IN`).
    pub ledge_spot: Option<(super::parkour::Ledge, Vec3, bool)>,
    /// The stride the current gait really takes, keyed by its speed,
    /// whether the real rig has bound, whether it shuffles and the crouch
    /// it sneaks in: measuring it costs a cycle of root-motion samples, so
    /// it is redone only when one changes.
    measured: Option<(StrideKey, f32)>,
}

/// What [`WalkerState::measured`]'s stride is keyed by: the speed's bits,
/// whether the real rig has bound, whether it shuffles, and the crouch's
/// drop and toes' bits.
type StrideKey = (u32, bool, bool, [u32; 2]);

impl WalkerState {
    /// A walker standing at `position`, facing `yaw`.
    pub fn at(position: Vec3, yaw: f32) -> Self {
        Self {
            locomotion: locomotion::Locomotion { mode: locomotion::RootMotion::Drive, position, ..Default::default() },
            facing: facing::Facing { yaw, target_yaw: yaw, ..Default::default() },
            transition: Default::default(),
            look: lookat::LookAt::forward(),
            stride: Default::default(),
            posture: Posture::Standing,
            approach: approach::Approach::Idle,
            seat: None,
            walked: Walked::default(),
            sit_offset: Vec2::ZERO,
            stood_hold: 0.0,
            stepping_aside: false,
            shuffle: None,
            shuffle_speed: 0.0,
            shuffle_ahead: 0.0,
            strafe: 0.0,
            pace: 0.0,
            gaits: Default::default(),
            jump: None,
            leap_asked: None,
            crouching: Default::default(),
            climbing: None,
            ladder_spot: None,
            hanging: None,
            ledge_spot: None,
            measured: None,
        }
    }

    /// Whether its hands hold the body off the floor, a move posing it on
    /// its holds: on a ladder, or grabbing or hanging from a ledge.
    pub fn on_holds(&self) -> bool {
        self.climbing.is_some() || self.hanging.is_some()
    }
}

/// The last whole step a walk took at one speed, measured from footfall to
/// footfall: what its next step at that speed will cover. The stride the
/// gait computes is a straight walk's; on the tight circle a walk turns
/// round on before a chair, the body moved ~82 % of it, and a stop timed by
/// it fell 15 cm short. A decayed average over recent frames was no
/// better: the sprung legs lag a change of speed, and just after slowing it
/// read 120 %.
#[derive(Debug, Clone, Copy, Default)]
pub struct Walked {
    /// The distance moved since the last footfall, and at what speed, if
    /// that speed held.
    since: f32,
    speed: f32,
    steady: bool,
    cycle: f32,
    position: Vec3,
    /// The last step measured: its length and speed.
    step: Option<(f32, f32)>,
}

impl Walked {
    /// Adds the frame from the last: the root now at `position`, the clock
    /// at `cycle` (a footfall each half), the legs stepping at `speed`.
    fn update(&mut self, position: Vec3, cycle: f32, speed: f32) {
        let flat = |v: Vec3| Vec2::new(v.x, v.z);
        let moved = flat(position - self.position).length();
        // A jump (a teleport) is not walking.
        if moved < 0.5 {
            self.since += moved;
        }
        if speed != self.speed {
            self.steady = false;
        }
        let footfall = |cycle: f32| (cycle * 2.0).floor();
        if speed <= 0.0 {
            self.step = None;
        } else if footfall(cycle) != footfall(self.cycle) {
            if self.steady {
                self.step = Some((self.since, speed));
            }
            (self.since, self.speed, self.steady) = (0.0, speed, true);
        }
        self.cycle = cycle;
        self.position = position;
    }

    /// The stride a walk at `speed` covers: two of the last steps measured,
    /// grown or shrunk with the speed as the gait's are (only between
    /// `speeds`, `gait::stride_speeds`); else `geometric`.
    fn stride(&self, speed: f32, geometric: f32, speeds: (f32, f32)) -> f32 {
        match self.step {
            Some((length, at)) if at > 0.0 && speed > 0.0 => approach::stride_at(speed, 2.0 * length, at, speeds),
            _ => geometric,
        }
    }
}

/// The gait as a walk to a spot ([`approach`]) needs it, at clock `cycle`:
/// where its stop can land, by the stride its current speed measured last
/// frame.
fn approach_gait(state: &mut WalkerState, cycle: f32, gait_rig: &RigGeometry) -> approach::Gait {
    let walking = state.transition.weight > 0.0;
    let speed_now = if walking { state.transition.stride_speed } else { 0.0 };
    state.walked.update(state.locomotion.position, cycle, speed_now);
    let stride_speeds = super::gait::stride_speeds_with_steps(super::gait::leg_length_of(gait_rig), super::gait::SHORT_STEPS);
    approach::Gait {
        cycle,
        stride: state.walked.stride(speed_now, state.measured.map_or(0.0, |(_, distance)| distance), stride_speeds),
        speed: speed_now,
        stopped: !walking && state.transition.is_at_rest(),
        stride_speeds,
    }
}

/// What [`ride_rendered_feet`] needs from this frame's gait, and the pose
/// it rendered last frame.
#[derive(Debug, Clone, Default)]
pub struct Stride {
    pub params: Option<GaitParams>,
    /// The gait cycle this frame, and last frame.
    pub cycle: f32,
    pub previous_cycle: f32,
    pub previous: Option<LocalPose>,
    /// How much of the gait is playing: 0 standing, 1 walking.
    pub weight: f32,
    /// How far a balance recovery step has just carried the character, in
    /// the pose's frame (`balance::Balance::travelled`): moved like root
    /// motion, then zeroed.
    pub stepped: Vec3,
    /// This frame's whole travel is in `stepped` (a leap handing back to
    /// the run): the gait's root motion is skipped. Read once.
    pub given: bool,
}

/// Walkers whose skeleton has bound but who have no animation stack yet.
type UnattachedWalkers<'w, 's> = Query<
    'w,
    's,
    (Entity, &'static Walker, &'static Transform, Option<&'static FacingCorrection>, Option<&'static AnimGround>),
    (With<HumanoidSkeleton>, Without<WalkerState>),
>;

/// Gives each [`Walker`] whose skeleton has bound the animation stack: the
/// pose (seeded compiled-in, then its asset), the springs, the phase layer,
/// foot and arm IK, ground, the balances and its state. The ground is flat
/// unless the walker already has an [`AnimGround`].
pub fn attach_walkers(mut commands: Commands, asset_server: Res<AssetServer>, walkers: UnattachedWalkers) {
    for (entity, walker, transform, correction, ground) in &walkers {
        // Seeded with the compiled-in pose so the character is never a
        // T-posed mannequin for the frames the asset takes to load.
        let seed = poses::by_name(&walker.pose).unwrap_or_else(|| {
            warn!("walker: unknown pose '{}', falling back to relaxed_stand", walker.pose);
            poses::relaxed_stand()
        });
        let handle = asset_server.load(format!("anim/{}.pose.ron", walker.pose));
        // The heading the root already has, the asset's correction taken off.
        let heading = transform.rotation * correction.map_or(Quat::IDENTITY, |c| c.0).inverse();
        let yaw = heading.to_euler(EulerRot::YXZ).0;
        let mut entity_commands = commands.entity(entity);
        entity_commands.insert((
            AnimTarget::new(seed),
            AnimTargetAsset(handle),
            AnimSprings::default(),
            GaitPhase { speed: walker.speed, ..Default::default() },
            AnimPhaseLayer(if walker.speed > 0.0 { PhaseLayer::locomotion() } else { PhaseLayer::standing_idle() }),
            // Feet planted on whatever is beneath them, walking as well as
            // standing: root motion cancels the stance foot's relative
            // travel, so the locks absorb only the residual.
            AnimFootIk::default(),
            AnimArmIk::default(),
            (balance::Balance::default(), WalkBalance::default()),
            WalkerState::at(transform.translation, yaw),
        ));
        if ground.is_none() {
            entity_commands.insert(AnimGround(Box::new(FlatGround::default())));
        }
    }
}

/// Everything [`drive_walkers`] reads and writes per character.
type WalkingRig = (
    &'static mut Walker,
    &'static mut AnimTarget,
    // Mutable so the speed reaches the leg clock.
    &'static mut GaitPhase,
    &'static mut WalkerState,
    &'static mut AnimArmIk,
    &'static mut AnimFootIk,
    &'static mut Transform,
    // The asset's own facing correction, composed under the heading.
    Option<&'static FacingCorrection>,
    &'static mut AnimPhaseLayer,
    // Winter's standing pendulum: a push sways the body over its feet.
    &'static mut balance::Balance,
    // The same pendulum walking: a push moves the next footfalls.
    &'static mut WalkBalance,
    // Down or getting up, the walker stands.
    Option<&'static Ragdoll>,
    // What it walks round, going to a chair.
    Option<&'static super::obstacles::RouteObstacles>,
    // The springs a jump's pose is led ahead of (`jump::Jump::pose_led`).
    Option<&'static super::plugin::AnimSprings>,
    // Its fingers, closed round a ladder's rungs and rails.
    Option<&'static mut super::hand::RelaxedHands>,
);

/// Drives each walker's gait from its clock, in `AnimSet::Target`, so the
/// phase layer composes on top and the springs smooth the result.
pub fn drive_walkers(time: Res<Time>, mut rigs: Query<WalkingRig>) {
    for (mut walker, mut target, mut phase, mut state, mut arm_ik, mut foot_ik, mut root, correction, mut layer, mut balance, mut walk_balance, ragdoll, route_obstacles, springs, mut hands) in
        &mut rigs
    {
        let state = &mut *state;
        // Composed onto the AUTHORED pose, re-read every frame: composed onto
        // last frame's result, each frame layered another cycle on and the
        // legs wound up without bound.
        let base = poses::by_name(&walker.pose).unwrap_or_else(poses::relaxed_stand);
        // On a ladder, held by its hands, a push is not caught by stepping.
        let mut due_pushes = std::mem::take(&mut walker.pushes);
        if state.on_holds() {
            due_pushes.clear();
        }
        // The rig the gait is posed on: the real one once bound, the
        // synthetic proxy for the first frames. The gait's vertical motion
        // is in fractions of THIS rig's leg.
        let gait_rig = foot_ik.rig.clone().unwrap_or_default();

        // A leap from a run whose landing foot leaves within this frame: the
        // run picks up now, before the frame is posed, as far past that
        // toe-off as the frame goes. Its speed is what it landed with, and
        // the frame's travel is given whole: the jump's to the toe-off, the
        // run's after it, and the root moved to meet the run's pose; the
        // run's root motion is skipped this frame.
        //
        // Ended with a frame posed at the toe-off and the run picked up the
        // next, the frame that ended it moved the pelvis 13 mm of 57 and the
        // next 104.
        let dt = time.delta_secs();
        if let Some((resume, past, rest)) = state.jump.as_ref().and_then(|jump| {
            let past = jump.elapsed() + dt - jump.duration();
            jump.resumes().filter(|_| past >= 0.0).map(|resume| (resume, past, jump.travelled_at(jump.duration()) - jump.travelled()))
        }) {
            let cycle = (resume.cycle + past * resume.rate).rem_euclid(1.0);
            phase.gait = cycle * TAU;
            state.stride.cycle = cycle;
            state.stride.previous_cycle = cycle;
            state.pace = resume.speed;
            state.stride.stepped += resume.handover + gait_rig.forward() * (rest + resume.speed * past);
            state.stride.given = true;
            state.jump = None;
        }

        // Asked to sit on a chair elsewhere: walk to it and turn round first
        // (`approach`), overriding the speed and steering asked for. Its spot
        // is solved once per chair.
        let fallen = ragdoll.is_some_and(Ragdoll::is_falling);
        let (mut wanted_speed, mut steer, mut arrived, mut look_at) = (walker.speed, walker.steer, true, walker.look_at);
        // Walking to a chair it places itself, in short steps when slow.
        let mut placing = false;
        // Asked up a ladder it is not on (`ladder`): it walks to the spot in
        // front of it, facing it, as to a chair, and gets on there once
        // stopped. Its spot is solved once per ladder.
        // Up from the floor; down (or sliding) from a landing it leads onto,
        // getting on there, its back to the ladder, a little in from the
        // edge (`ladder::TOP_APPROACH_IN`): the turn round to face away from
        // the ladder dips toward the edge.
        let on_landing = walker.ladder.and_then(|ladder| ladder.landing_height()).is_some_and(|height| (state.locomotion.position.y - height).abs() < 0.3);
        let ladder_asked = state.climbing.is_none()
            && match walker.climb {
                Some(super::ladder::Climb::Up) => !on_landing,
                Some(_) => on_landing,
                None => false,
            }
            && walker.ladder.is_some()
            && walker.sit.is_none()
            && state.posture.is_standing()
            && !fallen;
        let mut at_ladder = false;
        // Asked to grab a ledge (`parkour::hang`): it walks to the spot under
        // it, facing the wall, and jumps from there once stopped.
        let hang_asked = !state.on_holds()
            && walker.hang == Some(super::parkour::hang::HangAsk::Grab)
            && walker.ledge.is_some()
            && walker.sit.is_none()
            && !ladder_asked
            && state.posture.is_standing()
            && !fallen;
        let mut at_ledge = false;
        match (walker.sit, walker.chair, foot_ik.rig.as_ref()) {
            _ if state.on_holds() => {
                state.approach = approach::Approach::Idle;
                (wanted_speed, steer) = (0.0, Steer::Straight);
            }
            (_, _, Some(rig)) if hang_asked => {
                let ledge = walker.ledge.expect("asked to grab a ledge");
                let ahead = approach::heading_of(rig.forward());
                let square = super::parkour::Hanging::square(&ledge, rig.forward());
                if state.ledge_spot.is_none_or(|(was, _, _)| was != ledge) {
                    let stood = stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig);
                    let spot = super::parkour::Hanging::spot(&ledge, state.locomotion.position, square, &stood, rig);
                    state.ledge_spot = Some((ledge, spot, false));
                    state.approach = approach::Approach::Idle;
                }
                let (_, spot, led_in) = state.ledge_spot.expect("a spot");
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                let obstacles: Vec<_> = route_obstacles.map(|route| route.0.clone()).unwrap_or_default();
                // First out in front of the spot, then straight in to it: the
                // approach (made to end with a chair behind it) came at a spot
                // faced at a wall from beside it, turned in along the wall and
                // put a fingertip 18 cm into it.
                let lead_in = spot + ledge.out * LEDGE_LEAD_IN;
                let to_lead_in = Vec2::new(lead_in.x - state.locomotion.position.x, lead_in.z - state.locomotion.position.z);
                let order = if !led_in && to_lead_in.length() > LEDGE_AT_LEAD_IN {
                    let toward = approach::heading_of(Vec3::new(to_lead_in.x, 0.0, to_lead_in.y));
                    approach::Order::Walk { speed, heading: toward, rate: 2.0 }
                } else {
                    if let Some(found) = state.ledge_spot.as_mut() {
                        found.2 = true;
                    }
                    state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, spot, square + ahead, speed, &obstacles)
                };
                match order {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        (wanted_speed, steer, at_ledge) = (0.0, Steer::Straight, true);
                    }
                }
                if look_at.is_none() {
                    look_at = Some(ledge.nearest(state.locomotion.position, 0.0));
                }
            }
            (_, _, Some(rig)) if ladder_asked => {
                let ladder = walker.ladder.expect("asked up a ladder");
                let ahead = approach::heading_of(rig.forward());
                let square = approach::heading_of(-ladder.out()) - ahead;
                if state.ladder_spot.is_none_or(|(was, top, _)| was != ladder || top != on_landing) {
                    let stood = stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig);
                    let spot = match super::ladder::Climbing::top_spot(&ladder, square, &stood, rig).filter(|_| on_landing) {
                        Some(top) => top - ladder.out() * super::ladder::TOP_APPROACH_IN,
                        None => super::ladder::Climbing::spot(&ladder, square, &stood, rig),
                    };
                    state.ladder_spot = Some((ladder, on_landing, spot));
                    state.approach = approach::Approach::Idle;
                }
                let spot = state.ladder_spot.map_or(Vec3::ZERO, |(_, _, spot)| spot);
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                // Round the ladder and whatever else stands in the way.
                let own = ladder.footprint();
                let mut obstacles = vec![own];
                if let Some(route) = route_obstacles {
                    obstacles.extend(route.0.iter().copied().filter(|piece| !own.contains(piece, 0.05)));
                }
                // Already by the ladder's top on its landing, its back to it
                // (just off it, say): getting on shuffles it back to the edge
                // (`ladder`). Walked there instead, from 0.3 m off it turned
                // a circle that went out over the landing's side and fell.
                let near = Vec2::new(spot.x - state.locomotion.position.x, spot.z - state.locomotion.position.z).length();
                let facing_off = facing::shortest_angle(state.facing.yaw - square).abs();
                let order = if on_landing && near < LANDING_NEAR && facing_off < LANDING_NEAR_HEADING && state.approach == approach::Approach::Idle {
                    approach::Order::Arrived
                } else {
                    state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, spot, square + ahead, speed, &obstacles)
                };
                match order {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        (wanted_speed, steer, at_ladder) = (0.0, Steer::Straight, true);
                    }
                }
                // It looks at the ladder on the way.
                if look_at.is_none() {
                    look_at = Some(ladder.middle_at(state.locomotion.position.y + 1.6));
                }
            }
            (Some(how), Some(chair), Some(rig)) if how.on_chair() && state.posture.is_standing() && !fallen => {
                // Headings are the walking direction's, the rig's own
                // forward turned by the facing.
                let ahead = approach::heading_of(rig.forward());
                let facing_yaw = approach::heading_of(chair.forward) - ahead;
                if state.seat.is_none_or(|(was, _)| was != chair) {
                    let stood = stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig);
                    let spot = approach::stand_spot(&chair, rig, &stood, Quat::from_rotation_y(facing_yaw));
                    state.seat = Some((chair, spot));
                    state.approach = approach::Approach::Idle;
                }
                let spot = state.seat.map_or(Vec3::ZERO, |(_, spot)| spot);
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                // The turn ends a little in front of the spot, clear of the
                // chair; the seat makes that up.
                let turn_to = spot + chair.forward.normalize_or_zero() * approach::TURN_AHEAD;
                // Round its chair and whatever else stands in the way.
                // Its own chair's pieces, in the physics world too, are its
                // chair: the spot is within their margin by design.
                let own = chair.footprint();
                let mut obstacles = vec![own];
                if let Some(route) = route_obstacles {
                    obstacles.extend(route.0.iter().copied().filter(|piece| !own.contains(piece, 0.05)));
                }
                let order = state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, turn_to, facing_yaw + ahead, speed, &obstacles);
                match order {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        // The sitting starts this frame. Its seat goes as
                        // much further back, and across, as the walk
                        // stopped off its spot, so the hips land on the
                        // seat's middle with the feet where they are.
                        let off = state.locomotion.position - spot;
                        let forward = chair.forward.normalize_or_zero();
                        let left = Vec3::Y.cross(forward);
                        state.sit_offset = Vec2::new(
                            off.dot(forward).clamp(-sitting::SEAT_BACK_RANGE, sitting::SEAT_BACK_RANGE),
                            (-off.dot(left)).clamp(-sitting::SEAT_ACROSS_RANGE, sitting::SEAT_ACROSS_RANGE),
                        );
                        info!(
                            "walker: at the chair, {:.0} mm off its spot: seat {:+.0} mm back, {:+.0} mm across",
                            Vec2::new(off.x, off.z).length() * 1e3,
                            state.sit_offset.x * 1e3,
                            state.sit_offset.y * 1e3
                        );
                        steer = Steer::Straight;
                    }
                }
                // It looks at the chair on the way, until it turns round.
                if matches!(state.approach, approach::Approach::Walking { on_arc: false, .. }) && look_at.is_none() {
                    look_at = Some(chair.seat + Vec3::Y * chair.height);
                }
            }
            _ if state.posture.is_standing() => {
                state.approach = approach::Approach::Idle;
                state.seat = None;
            }
            _ => {}
        }

        // The transition first: it decides the speed the legs step at, which
        // through a stop's last step is the walk's, not the zero asked for
        // (Winter §11.3.3). Standing, it is told how the idle carries its
        // weight, so a start stands on the loaded leg.
        let idle_shift = PhaseLayer::standing_idle().sway.map_or(0.0, |sway| sway.weight_shift(phase.elapsed));
        if state.transition.is_at_rest() {
            state.transition.idle_shift = idle_shift;
        }
        let config = transition::TransitionConfig {
            // Half the duty factor: the other leg's mid-swing.
            mid_swing: state.stride.params.map_or(transition::TransitionConfig::default().mid_swing, |p| p.duty_factor * 0.5),
            // A shuffle starts and stops over a whole swing: its quick
            // cadence left a fade from mid-swing 6 frames long.
            whole_swing: state.shuffle.is_some(),
            ..Default::default()
        };
        // A push lands on whichever balance carries the body: the walking
        // one once the walk is fully in (or still catching an earlier push),
        // else the standing one. A hit arrives on the standing balance
        // (`ragdoll_plugin`) and is handed over.
        let walking = (state.transition.weight >= 1.0 && balance.is_settled(1.0e-5)) || !walk_balance.is_settled(1.0e-3);
        for &push in &due_pushes {
            if walking {
                walk_balance.push(push);
            } else {
                balance.push(push);
            }
        }
        if walking && balance.pending_push() != Vec2::ZERO {
            walk_balance.push(balance.take_push());
        }
        // A push from behind speeds the walk up (`WalkBalance::surge`).
        // Fallen or getting up, it asks for no speed, and starts walking
        // again from a stand once up: walking on, the gait's root motion
        // carried the rising body forward 1.7-2.9 m, sliding.
        // Seated, sitting down or asked to, it asks no speed either: it stops
        // first, and stands up before it walks again.
        // Walking to a chair, it walks.
        // Jumping, it asks no speed until it has landed and stood; leaping
        // from a run to run on, the run goes on under it.
        let jumping = state.jump.as_ref().is_some_and(|jump| jump.resumes().is_none());

        // The standing knee bend, from the rig's own measured geometry, so
        // it bends the right way on any rig (baked into a pose file it bent
        // backward on a rig facing the other way).
        let stood = match &foot_ik.rig {
            Some(rig) => stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig),
            None => stance_on(&base, DEFAULT_KNEE_FLEX),
        };
        let stood = match (walker.pose.strip_prefix("getup:"), &foot_ik.rig) {
            (Some(name), Some(rig)) => {
                use super::getup::Key;
                let key = match name {
                    "sit" => Key::Sit,
                    "squat" => Key::Squat,
                    "quadruped" => Key::Quadruped,
                    "side_sit_left" => Key::SideSit { left_down: true },
                    "side_sit_right" => Key::SideSit { left_down: false },
                    _ => Key::HalfKneel,
                };
                key.pose(rig)
            }
            _ => stood,
        };

        // Asked to sneak (`sneak`), it crouches, standing, walking or
        // shuffling aside; walking, the walk eases into the crouch's
        // (`sneak::SneakGait`), and asked to stop sneaking it stands up on the
        // move. Not running: asked to sneak, a run slows to a walk first.
        // Asked to sit, jump, step aside or climb, it stands up first.
        let busy = walker.step_aside != 0.0
            || walker.sit.is_some()
            || walker.jump.is_some()
            || fallen
            || !state.posture.is_standing()
            || state.jump.is_some()
            || state.gaits.running > 0.0
            || ladder_asked
            || hang_asked
            || state.on_holds();
        let sneak = if busy { super::sneak::Sneak::STANDING } else { walker.sneak };
        let mut footing = None;
        if sneak.is_sneaking() || !state.crouching.is_standing() {
            let crouch_on = super::sneak::Footing::of(&stood, &gait_rig);
            state.crouching.ask(sneak.crouch_on(super::gait::leg_length_of(&gait_rig)), crouch_on.rise());
            state.crouching.advance(time.delta_secs());
            footing = Some(crouch_on);
        }
        let crouched = !state.crouching.is_standing();
        // From a stand it sets off once its crouch is still.
        let still = fallen
            || !state.posture.is_standing()
            || (walker.sit.is_some() && arrived)
            || at_ladder
            || at_ledge
            || state.on_holds()
            || jumping
            || (!state.crouching.is_still() && state.transition.is_at_rest());
        // Asked to go aside (`Walker::aside`), with or without forward, and
        // not to sit. Mostly across (45° or more off forward): the side
        // shuffle (`shuffle`), a walk of its own on the same clock, forward
        // on a diagonal. Mostly forward: the walk, the body turned to its way
        // and the head kept on where it faced (`strafe`, below). Changing
        // between them, or the shuffle's side, it stops first.
        let side = if still || walker.sit.is_some() { 0.0 } else { walker.aside };
        let forward = wanted_speed.max(0.0);
        let going = forward.hypot(side);
        let ahead = if going > 0.0 { forward / going } else { 0.0 };
        let toward = (side != 0.0 && ahead <= super::shuffle::SHUFFLE_MOST_AHEAD).then(|| side.signum());
        if state.transition.is_at_rest() {
            state.shuffle = toward;
        }
        let shuffling = state.shuffle.is_some() && state.shuffle == toward;
        let strafe = if side != 0.0 && toward.is_none() && state.shuffle.is_none() { side.atan2(forward) } else { 0.0 };
        // One step aside asked (`Walker::step_aside`): the standing
        // balance's side step and close, from a stand. A walk asked for
        // meanwhile starts once the feet have closed.
        let step_aside = if crouched { 0.0 } else { std::mem::take(&mut walker.step_aside) };
        if step_aside != 0.0 && !still && state.transition.is_at_rest() && state.shuffle.is_none() {
            balance.step_aside(step_aside);
        }
        let closed = balance.swing.is_none() && balance.feet == [Vec2::ZERO; 2] && balance.aside() == 0.0;
        state.stepping_aside = !closed && (state.stepping_aside || balance.aside() != 0.0);
        let asked = if still || state.stepping_aside {
            0.0
        } else if state.shuffle.is_some() {
            if shuffling { going } else { 0.0 }
        } else if toward.is_some() {
            // Walking, asked to shuffle: it stops first.
            0.0
        } else {
            going + walk_balance.surge
        };
        // Sneaking, or asked to, no faster than a sneak walks.
        let asked = if crouched || walker.sneak.is_sneaking() { asked.min(super::sneak::FASTEST) } else { asked };
        // Above a walk the speed is eased toward (`run::ACCELERATION`): up
        // to the changeover at once, as a walk always took its speed, then
        // gathered; coming down, shed until the gait is a walk again, which
        // then stops as a walk does. Fallen, sitting or shuffling, at once.
        let leg = super::gait::leg_length_of(&gait_rig);
        let changeover = super::run::changeover_speed(leg);
        let dt = time.delta_secs();
        // Both feet down by the gait's clock, as last frame left it: a walk
        // holds its speed then (`run::paced`).
        let both_down = foot_ik.gait_swing == Some([false; 2]);
        state.pace = if still || state.shuffle.is_some() || toward.is_some() {
            asked
        } else {
            super::run::paced(asked, state.pace, state.gaits.running, changeover, state.transition.weight >= 1.0, both_down, dt)
        };
        let asked = state.pace;
        let weight_before = state.transition.weight;
        let event = state.transition.advance(asked, cycle_of(&phase), &config, time.delta_secs());
        let speed = state.transition.stride_speed;

        // A walk's stride grows with its speed, scaled to this rig's leg.
        // Placing itself at a chair, its stride shortens further: the
        // approach paces its stop and turns by it (`gait::SHORT_STEPS`).
        // The shuffle's stride and width from the speed asked, held through
        // its stop; set from a stand, eased to a new speed asked on the way
        // (at once, the feet jumped to the new stride and width).
        // Its diagonal likewise.
        if shuffling {
            if weight_before <= 0.0 {
                (state.shuffle_speed, state.shuffle_ahead) = (going, ahead);
            } else {
                let most = super::shuffle::SHUFFLE_REGEAR * time.delta_secs();
                state.shuffle_speed += (going - state.shuffle_speed).clamp(-most, most);
                let most = super::shuffle::SHUFFLE_REAIM * time.delta_secs();
                state.shuffle_ahead += (ahead - state.shuffle_ahead).clamp(-most, most);
            }
        }
        // Crouched, the sneak's walk: the one its crouch is going to, blended
        // from the one it set off from while the crouch changes.
        let sneak_gait = footing
            .filter(|_| crouched && state.shuffle.is_none())
            .map(|footing| super::sneak::SneakGait::of(&state.crouching, &footing, speed, &stood, &gait_rig));
        // What a stand blends with and a shuffle is posed on (the crouch it
        // is in), and what a walk is measured on (the crouch it is going to),
        // else standing. A shuffle is posed afresh each frame, so it takes
        // the crouch as it changes.
        let gait_base = match (footing, sneak_gait.as_ref()) {
            (_, Some(gait)) if state.crouching.is_still() => gait.target().0,
            (Some(footing), _) if crouched => footing.pose(state.crouching.now(), &stood, &gait_rig),
            _ => stood,
        };
        let gait_target = sneak_gait.as_ref().map_or(gait_base, |gait| gait.target().0);
        let walk_params = if let Some(toward) = state.shuffle {
            super::shuffle::shuffling(state.shuffle_speed, toward, state.shuffle_ahead)
        } else if let Some(gait) = sneak_gait.as_ref() {
            gait.target().1
        } else if placing {
            GaitParams::walking_with_steps(speed, leg, super::gait::SHORT_STEPS)
        } else {
            GaitParams::walking_on(speed, &gait_rig)
        };
        let run_params = GaitParams::running_for(speed, leg);

        // A run above the changeover speed, a walk again below a tenth less,
        // changed in one step (`run::Gaits`); only walking fully and not
        // shuffling. Through the change the gait coming in is posed where
        // its planted foot matches the one going out's
        // (`run::Matched::feet`), and the clock moves onto it at the end.
        if still || state.shuffle.is_some() || state.transition.weight < 1.0 || crouched {
            state.gaits.walk();
        } else {
            let previous_cycle = state.stride.cycle;
            let now = cycle_of(&phase);
            let feet = || {
                super::run::Matched::feet(
                    |p| walk_pose_on(p, &walk_params, &stood, &gait_rig),
                    |p| walk_pose_on(p, &run_params, &stood, &gait_rig),
                    walk_params.duty_factor,
                    run_params.duty_factor,
                    &gait_rig,
                )
            };
            if let Some(onto) = state.gaits.advance(speed, changeover, walk_params.duty_factor, run_params.duty_factor, now, previous_cycle, feet) {
                phase.gait = onto.rem_euclid(1.0) * TAU;
                // Root motion reads which feet are down half-way through
                // the frame, from last frame's clock: moved with it.
                state.stride.previous_cycle += onto - now;
            }
        }
        let running = state.gaits.running;
        // What root motion, the foot locks and the transition read: the
        // gait mostly in play, with the stance share between the two; while
        // changing, the stance share of the gait going out, whose the clock
        // still is.
        let params = if running <= 0.0 {
            walk_params
        } else if running >= 1.0 {
            run_params
        } else {
            let duty_factor = if state.gaits.from_run() { run_params.duty_factor } else { walk_params.duty_factor };
            GaitParams { duty_factor, ..if running < 0.5 { walk_params } else { run_params } }
        };

        // The speed reaches the LEG clock as the cadence that makes this
        // gait's stride travel at exactly this speed: set once at spawn, the
        // legs kept the launch rhythm while the body moved at the new speed,
        // and the planted feet slid by the difference.
        // Keyed by the speed the stride is for: a shuffle's, the one asked.
        let stride_for = if state.shuffle.is_some() { state.shuffle_speed } else { speed };
        // And by the crouch a sneak is going to, on whose walk it is measured.
        // Keyed by the crouch it is in, it was measured afresh, a cycle built,
        // every frame of a crouch going down or up.
        let crouch_to = if crouched { state.crouching.target() } else { super::sneak::Crouch::default() };
        let key = (stride_for.to_bits(), foot_ik.rig.is_some(), state.shuffle.is_some(), [crouch_to.drop.to_bits(), crouch_to.toes.to_bits()]);
        let walked = match state.measured {
            Some((measured, distance)) if measured == key => distance,
            _ if running >= 1.0 => 0.0,
            _ => {
                let distance = locomotion::distance_per_cycle(&walk_params, &gait_target, &gait_rig);
                state.measured = Some((key, distance));
                distance
            }
        };
        // A run's stride from its table (`run::distance_per_cycle`); through
        // the change, the stride of the gait going out, whose the clock is:
        // the one coming in, matched, keeps its own pace by it.
        let distance = if running <= 0.0 || (state.gaits.changing() && !state.gaits.from_run()) {
            walked
        } else {
            super::run::distance_per_cycle(&stood, &gait_rig, super::run::reference_speed(speed, leg))
        };
        if speed > 0.0 && distance > 1.0e-4 {
            phase.base_frequency_hz = 0.0;
            phase.speed_coefficient = 1.0 / distance;
        } else {
            let defaults = GaitPhase::default();
            phase.base_frequency_hz = defaults.base_frequency_hz;
            phase.speed_coefficient = defaults.speed_coefficient;
        }
        if phase.speed != speed {
            phase.speed = speed;
        }
        match event {
            // The first step joins the walk at the swinging leg's mid-swing;
            // the gait has no weight yet, so moving its clock moves nothing.
            Some(transition::TransitionEvent::FirstStep { cycle }) => phase.gait = cycle * TAU,
            // Back at rest: the idle's weight shifts start over, standing
            // square a while first.
            Some(transition::TransitionEvent::AtRest) => phase.elapsed = 0.0,
            None => {}
        }
        let cycle = cycle_of(&phase);
        let weight = state.transition.weight;

        // Each layer's oscillators fade with the gait's weight; the idle's
        // sway runs only at rest, eased back in after a stop (switched on
        // at once it ticked the pelvis 6 mm sideways in a frame).
        let mut wanted = PhaseLayer::between(&PhaseLayer::standing_idle(), &PhaseLayer::locomotion(), weight);
        // Sitting, jumping or climbing, no standing weight shift: it breathes.
        // Getting on a ladder, it fades as the hands take hold: stopped at
        // once, standing a while on a landing first, the hips stepped 4.4 mm
        // aside in a frame.
        let getting_on = state.climbing.as_ref().filter(|climbing| climbing.is_getting_on()).map(|climbing| 1.0 - climbing.holding());
        if !state.transition.is_at_rest() || !state.posture.is_standing() || state.jump.is_some() || (state.on_holds() && getting_on.is_none()) {
            wanted.sway = None;
        } else if let Some(sway) = wanted.sway.as_mut() {
            const SETTLE_SECONDS: f32 = 1.5;
            let t = (phase.elapsed / SETTLE_SECONDS).clamp(0.0, 1.0);
            let settled = t * t * (3.0 - 2.0 * t) * getting_on.unwrap_or(1.0);
            sway.lateral *= settled;
            sway.fore_aft *= settled;
        }
        // Shuffling, none of it: a shuffle carries its own pelvis
        // (`shuffle::shuffle_pose`), and the walk's sway re-solved it over
        // the walk's loaded feet, wide apart: a 7 mm dip at every step, 403
        // m/s² headless.
        fade_walk_sway(&mut wanted, if state.shuffle.is_some() { 1.0 } else { running });
        // Holding a ladder, nothing sways the arms or the chest: laid over
        // the hands posed on their rungs, the idle's arm swing and breath
        // moved them off.
        // Grabbing a ledge, from the start: the arms reach for it.
        let holding = state.climbing.as_ref().map(|climbing| climbing.holding()).or(state.hanging.as_ref().map(|_| 1.0));
        if let Some(holding) = holding {
            let free = 1.0 - holding;
            for (bone, oscillator) in &mut wanted.oscillators {
                if matches!(bone, Bone::Spine | Bone::Spine1 | Bone::Spine2 | Bone::LeftShoulder | Bone::RightShoulder | Bone::LeftArm | Bone::RightArm | Bone::LeftForeArm | Bone::RightForeArm | Bone::LeftHand | Bone::RightHand) {
                    oscillator.amplitude *= free;
                }
            }
        }
        if layer.0 != wanted {
            layer.0 = wanted;
        }

        // The pose as a function of phase, ONE definition, used both to pose
        // the character and to derive its root motion, so the two cannot
        // disagree. Blended from `stood`, so the standing knee bend does not
        // pop as a walk begins or ends; crouched, from the crouch.
        let mut prepared = gait_base;
        state.transition.apply_release(&mut prepared, &gait_rig);
        // A stop's last swing is set down onto where it will stand.
        foot_ik.landing = state.transition.landing(&prepared, &gait_rig);
        // A push sways the standing body over its feet and it recovers
        // (Winter's inverted pendulum), stepping if it must.
        if !balance.is_settled(1.0e-5) {
            let support = balance::Support::of(&stood, &gait_rig);
            balance.step(&support, balance::pendulum_k(&stood, &gait_rig), time.delta_secs());
            if let Some(by) = balance.travelled {
                state.stride.stepped += gait_rig.forward() * by.x + gait_rig.left() * by.y;
                balance.rebase();
            }
            if foot_ik.landing.is_none()
                && let Some((left, spot, strength)) = balance.landing_spot(&prepared, &gait_rig)
            {
                foot_ik.landing = Some(Landing { left, spot, strength, place: true });
            }
            balance.apply(&mut prepared, &gait_rig);
        }
        // The feet the balance has down stay locked however its sprung legs
        // lag a stumbling body; a walk's feet are the locks' own call.
        // A run's feet lock as they land (`FootLock::update_gripped`), and
        // leave the floor at once (`run::swing_clearance`).
        // A walk's too, fully walking (the start's and stop's fades lift and
        // set down their own swings), not shuffling.
        foot_ik.grip = running >= 0.5;
        foot_ik.touchdown = None;
        let walking = weight >= 1.0 && state.shuffle.is_none();
        foot_ik.clear = [0.0, 0.5].map(|shift| match super::gait::leg_phase(cycle + shift, params.duty_factor) {
            super::gait::LegPhase::Swing { progress } if foot_ik.grip => super::run::swing_clearance(progress),
            super::gait::LegPhase::Swing { progress } if walking => super::walk::swing_clearance(progress),
            _ => 0.0,
        });
        foot_ik.gait_swing = (weight > 0.0)
            .then(|| [0.0, 0.5].map(|shift| !super::gait::leg_phase(cycle + shift, params.duty_factor).is_stance()));
        foot_ik.gait_bearing = (weight > 0.0).then(|| {
            [0.0, 0.5].map(|shift| {
                matches!(super::gait::leg_phase(cycle + shift, params.duty_factor), super::gait::LegPhase::Stance { progress } if progress < BEARING)
            })
        });
        foot_ik.planted = if weight <= 0.0 && !balance.is_settled(1.0e-5) {
            balance.planted()
        } else if state.shuffle.is_some() && weight > 0.0 {
            // A shuffle's feet down are known from its clock. Left to the
            // locks' speed test, the foot standing through a restart's fade
            // (the blend sinking it 2 cm in the pose) was let go mid-stance,
            // rose 8 mm and crept 2.4 cm.
            // The fade's swinging foot: a stop's last, else a start's first
            // (the one not standing through it).
            let swinging_left = if state.transition.last_swing > 0.0 { state.transition.last_swing_leg > 0.0 } else { state.transition.stance < 0.0 };
            super::shuffle::planted(cycle, params.duty_factor, weight, swinging_left)
        } else if running >= 0.5 {
            // A run's stance foot is down from contact to toe-off: told to
            // the locks, so the lagging sprung leg under a body going 3-6 m/s
            // cannot fool their speed test, and let go at toe-off, so the
            // lock does not hold a foot that has left.
            [0.0, 0.5].map(|shift| super::gait::leg_phase(cycle + shift, params.duty_factor).is_stance())
        } else if weight <= 0.0 && !state.crouching.is_standing() {
            // Crouching or crouched standing, both feet stay where they
            // stood, as a jump's do going down into its countermovement.
            [true; 2]
        } else {
            [false; 2]
        };

        // A push while walking: the walk goes on, its footfalls moved to
        // catch the body; the body moves by the push's offset like root
        // motion.
        if !walk_balance.is_settled(0.0) {
            let on = |v: Vec2| gait_rig.forward() * v.x + gait_rig.left() * v.y;
            if weight > 0.0 {
                let toe = |bone| super::rig::offset_from(&stood, &gait_rig, Bone::Hips, bone);
                let width = (toe(Bone::LeftToeBase) - toe(Bone::RightToeBase)).dot(gait_rig.left()).abs();
                let walk = walk_balance::Stride {
                    seconds: 1.0 / phase.gait_frequency_hz().max(1.0e-3),
                    duty_factor: params.duty_factor,
                    step_length: distance * 0.5,
                    step_width: width,
                    max_step: super::gait::leg_length_of(&gait_rig),
                };
                let k = balance::pendulum_k(&stood, &gait_rig);
                walk_balance.step(&walk, state.stride.cycle, cycle, k, time.delta_secs());
            } else {
                // Stopped before the push was spent: the standing balance
                // takes the body as it is, its feet where they stand.
                let shown = walk_balance.moved();
                state.stride.stepped += on(shown);
                balance.velocity += walk_balance.velocity + walk_balance.pending_push();
                balance.feet = [0, 1].map(|leg| walk_balance.foot_displacement(leg));
                *walk_balance = WalkBalance::default();
            }
            state.stride.stepped += on(walk_balance.moved());
            foot_ik.displaced = [0, 1].map(|leg| on(walk_balance.foot_displacement(leg)));
            walk_balance.settle(1.0e-3);
            if walk_balance.is_settled(0.0) {
                foot_ik.displaced = [Vec3::ZERO; 2];
            }
        }
        // How deep the crouch is, for a crouched shuffle's arms.
        let carried = footing.filter(|_| crouched && state.shuffle.is_some()).map_or(0.0, |footing| footing.depth_of(state.crouching.now()));
        let transition_state = &state.transition;
        let gaits = state.gaits;
        let rendered = |cycle: f32| {
            if weight <= 0.0 {
                prepared
            } else {
                // Walking, running, or the one step changing between them,
                // each gait at its own phase (`run::Gaits::phases`).
                let walking = if let Some(gait) = sneak_gait.as_ref() {
                    gait.pose(cycle, &gait_rig)
                } else if running <= 0.0 {
                    // A walk, or a shuffle on the crouch it is in, its arms
                    // carried as a sneak's: the shuffle's own carry, out from
                    // the body, laid over them held them out wide, the hands
                    // outside the hips.
                    let mut pose = walk_pose_on(cycle, &walk_params, &gait_base, &gait_rig);
                    super::sneak::carry_arms(&mut pose, &gait_rig, carried, [0.0; 2]);
                    pose
                } else if running >= 1.0 {
                    walk_pose_on(cycle, &run_params, &stood, &gait_rig)
                } else {
                    let (walked, ran) = gaits.phases(cycle);
                    let (walk, run) = (walk_pose_on(walked, &walk_params, &stood, &gait_rig), walk_pose_on(ran, &run_params, &stood, &gait_rig));
                    super::clip::blend(&walk, &run, running)
                };
                transition_state.blend(&prepared, &walking, &gait_rig)
            }
        };
        // Clippy misses the second use: root motion reads `&rendered` below.
        #[allow(clippy::redundant_closure_call)]
        {
            target.pose = rendered(cycle);
        }
        // Sitting, sitting down or standing up: the posture's pose instead
        // (`sitting`), its legs as solved on its own contacts, untouched by
        // the leg IK. It starts only once stopped.
        let mut legs_free = false;
        if let Some(rig) = foot_ik.rig.clone() {
            let ready = weight <= 0.0 && state.transition.is_at_rest() && arrived && state.crouching.is_standing() && !state.on_holds();
            // The seat as the walk to it left it (`sit_seat`), else one
            // where it stands.
            let seat = match walker.chair {
                Some(chair) => sitting::Seat { height: chair.height, back: state.sit_offset.x, across: state.sit_offset.y },
                None => sitting::Seat::at(walker.chair_height),
            };
            let rising = matches!(state.posture, Posture::Moving { down: false, .. });
            let advanced = state.posture.advance(walker.sit, seat, &stood, &rig, ready, time.delta_secs());
            // Stood up: both feet stay planted a moment more, until the
            // sprung legs have finished extending. Let go at once, a foot
            // still moving with them was released by its speed and slid to
            // where the standing pose has it: 11-17 mm after a walk that
            // ended turning, whose feet are not set as the stance's.
            if rising && state.posture.is_standing() {
                state.stood_hold = STOOD_HOLD;
            }
            if advanced.is_none() && state.stood_hold > 0.0 {
                state.stood_hold -= time.delta_secs();
                foot_ik.planted = [true; 2];
            }
            if let Some((pose, planted)) = advanced {
                target.pose = pose;
                foot_ik.planted = planted;
                foot_ik.landing = None;
                // With both feet down the locks hold them against the
                // springs' lag (on a chair 21-24 mm without them; a squat's
                // toes 29 mm into the floor) and the legs stay in their
                // planes. Seated on the floor, free: the leg IK keeps each
                // knee in its leg's plane, and a cross-legged knee is not.
                // Kneeling, the feet stand on tucked toes, which the foot IK
                // (built for a flat foot) drove 47 mm into the floor.
                let floor_seated = matches!(state.posture, Posture::Seated { how, .. } if !how.on_chair());
                let kneeling = matches!(state.posture, Posture::Seated { how, .. } | Posture::Moving { how, .. } if how == sitting::Sitting::Floor(sitting::FloorPose::Kneeling));
                legs_free = !(planted[0] && planted[1]) || floor_seated || kneeling;
            }
        }
        // A jump (`jump`), from a stand: its pose instead, the feet told to
        // the foot IK from its plan. Down, the locks hold them whatever the
        // sprung legs do; in flight they are in the air, no hips dropped to
        // reach them and the toe tips free; landing, each locks as it
        // touches.
        let asked_jump = if crouched { None } else { walker.jump.take() };
        let mut let_go = false;
        if let Some(rig) = foot_ik.rig.clone() {
            let standing = weight <= 0.0
                && state.transition.is_at_rest()
                && state.posture.is_standing()
                && !state.on_holds()
                && !at_ladder
                && !at_ledge
                && !state.stepping_aside
                && state.shuffle.is_none()
                && !fallen
                && balance.is_settled(1.0e-5);
            // Running fully, a jump waits for the next foot to come down,
            // and takes off from it (`jump::Jump::from_run`).
            let running = state.gaits.running >= 1.0 && !state.gaits.changing() && weight >= 1.0 && state.shuffle.is_none() && !fallen;
            if let Some(ask) = asked_jump
                && state.jump.is_none()
            {
                if standing {
                    state.jump = Some(super::jump::Jump::plan(ask, &stood, &rig));
                } else if running {
                    state.leap_asked = Some(ask);
                }
            }
            if !running {
                state.leap_asked = None;
            }
            let mut started = false;
            let rate = phase.gait_frequency_hz();
            if let Some(ask) = state.leap_asked
                && state.jump.is_none()
                && rate > 0.0
            {
                // The foot whose contact the clock passed this frame.
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let mut jump = super::jump::Jump::from_run(ask, super::jump::RunStart { leg, speed }, &stood, &rig);
                    jump.advance(since);
                    // Landing on both feet, it stops: the gait is let go
                    // under the jump (at the end of the frame, once the
                    // gait's root motion has been read), so it stands once
                    // landed.
                    let_go = jump.resumes().is_none();
                    // The run carried the body up to the contact; the jump
                    // from there.
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    state.stride.stepped += rig.forward() * (run_part + jump.travelled());
                    state.jump = Some(jump);
                    state.leap_asked = None;
                    started = true;
                }
            }
            let standing_pose = target.pose;
            if let Some(jump) = state.jump.as_mut() {
                // The COM's way forward moves the character, like root
                // motion; the pose keeps it over the root.
                if !started {
                    let before = jump.travelled();
                    jump.advance(time.delta_secs());
                    state.stride.stepped += rig.forward() * (jump.travelled() - before);
                }
                // Each bone led ahead of its spring, so the body rendered
                // is the plan's.
                target.pose = match springs {
                    Some(springs) => jump.pose_now_led(&stood, &rig, &springs.0),
                    None => jump.pose(&stood, &rig),
                };
                // From a run, a foot at a time, and each held where the plan
                // has it all the while it is down; standing, both feet, held
                // as they land.
                let from_run = jump.run_feet_down();
                let down = from_run.unwrap_or([!jump.airborne(); 2]);
                foot_ik.planted = down;
                foot_ik.grip = from_run.is_some() || matches!(jump.phase(), super::jump::JumpPhase::Land | super::jump::JumpPhase::Recover);
                let pinned = if from_run.is_some() { down.contains(&true) } else { foot_ik.grip };
                foot_ik.touchdown = pinned.then(|| jump.touchdown(&stood, &rig));
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = Some(down.map(|down| !down));
                foot_ik.gait_bearing = Some(down);
                foot_ik.landing = None;
                legs_free = false;
                // Standing, done once it has stood; from a run, the run picks
                // up next frame (above).
                // Stood up: the standing pose this frame, the root moved to
                // meet it (`Jump::settle`). Moved with the jump's last pose
                // still shown, the hips went 15 mm on and 16 back.
                // And the feet left locked where they are, no longer pinned
                // where the jump's last pose had them: pinned, they went on
                // the settle's 14 mm.
                if jump.is_done() && jump.resumes().is_none() {
                    state.stride.stepped += jump.settle();
                    target.pose = standing_pose;
                    foot_ik.touchdown = None;
                    state.jump = None;
                    state.stood_hold = STOOD_HOLD;
                }
            }
        }
        // On a ladder (`ladder`): its pose instead, posed on its holds, the
        // root riding its hips and its facing square to the ladder; the
        // legs left as posed, no foot on the floor. Got on once stopped on
        // the spot the walk to it ended on, from wherever that is; stepped
        // off, it stands, both feet planted a moment.
        if let Some(rig) = foot_ik.rig.clone() {
            let ready = at_ladder && weight <= 0.0 && state.transition.is_at_rest() && state.crouching.is_standing() && state.jump.is_none();
            if ready
                && state.climbing.is_none()
                && let Some((ladder, _, _)) = state.ladder_spot
            {
                let square = approach::heading_of(-ladder.out()) - approach::heading_of(rig.forward());
                let mut climbing = super::ladder::Climbing::new(&ladder, state.locomotion.position, state.facing.yaw, square, foot_ik.pelvis_drop, &stood, &rig);
                // Each hand placed so its own fingers close round its rung.
                if let Some(hands) = hands.as_ref() {
                    climbing.set_grips(hands.grips, &rig);
                }
                state.climbing = Some(climbing);
                state.approach = approach::Approach::Idle;
            }
            if let Some(climbing) = state.climbing.as_mut() {
                climbing.advance(walker.climb, dt);
                // Each bone led ahead of its spring, so the body rendered is
                // the climb's.
                target.pose = match springs {
                    Some(springs) => climbing.pose_led(&rig, &springs.0),
                    None => climbing.pose(&rig),
                };
                state.locomotion.position = climbing.root();
                state.facing.yaw = climbing.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if look_at.is_none() {
                    look_at = Some(climbing.look());
                }
                if let Some(hands) = hands.as_mut() {
                    let grips = if climbing.is_done() { [0.0; 2] } else { climbing.grips() };
                    if hands.grip != grips || hands.hook != [false; 2] {
                        hands.grip = grips;
                        hands.hook = [false; 2];
                    }
                }
                // Off again, the idle's weight shifts start over, standing
                // square a while first: on at once, the body went 4 cm
                // aside in a frame.
                if climbing.is_done() {
                    state.climbing = None;
                    state.ladder_spot = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                }
            }
        }
        // Grabbing a ledge (`parkour::hang`): once stopped on the spot under
        // it, it jumps for the lip and hangs from it, posed on its holds,
        // the root riding its hips, the legs as posed. Out of a standing
        // jump's reach, the ask is dropped.
        if let Some(rig) = foot_ik.rig.clone() {
            let ready = at_ledge && weight <= 0.0 && state.transition.is_at_rest() && state.crouching.is_standing() && state.jump.is_none();
            if ready
                && state.hanging.is_none()
                && let Some((ledge, _, _)) = state.ledge_spot
            {
                let square = super::parkour::Hanging::square(&ledge, rig.forward());
                match super::parkour::Hanging::grab(&ledge, state.locomotion.position, square, foot_ik.pelvis_drop, &stood, &rig) {
                    Some(mut hanging) => {
                        // Each hand placed so its own fingers hook over the lip.
                        if let Some(hands) = hands.as_ref() {
                            hanging.set_grips(hands.grips, &stood, &rig);
                        }
                        state.hanging = Some(hanging);
                    }
                    None => walker.hang = None,
                }
                state.approach = approach::Approach::Idle;
                state.ledge_spot = None;
            }
            if let Some(hanging) = state.hanging.as_mut() {
                if walker.hang == Some(super::parkour::hang::HangAsk::ClimbUp) && hanging.climb_up() {
                    walker.hang = None;
                }
                hanging.advance(dt);
                // Each bone led ahead of its spring, so the body rendered is
                // the grab's.
                target.pose = match springs {
                    Some(springs) => hanging.pose_led(&rig, &springs.0),
                    None => hanging.pose(&rig),
                };
                state.locomotion.position = hanging.root();
                state.facing.yaw = hanging.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if look_at.is_none() {
                    look_at = Some(hanging.look());
                }
                if let Some(hands) = hands.as_mut() {
                    let grips = hanging.grips();
                    if hands.grip != grips || hands.hook != [true; 2] {
                        hands.grip = grips;
                        hands.hook = [true; 2];
                    }
                }
                // Climbed up, it stands on the top, as off a ladder.
                if hanging.is_done() {
                    state.hanging = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                }
            }
        }
        if foot_ik.legs_free != legs_free {
            foot_ik.legs_free = legs_free;
        }
        if foot_ik.off_floor != state.on_holds() {
            foot_ik.off_floor = state.on_holds();
        }


        // Turned to its way going aside and forward at once, it looks where
        // it was steered to face.
        if look_at.is_none() && state.strafe != 0.0 {
            let ahead = approach::heading_of(gait_rig.forward());
            look_at = Some(root.translation + approach::direction_of(state.facing.yaw - state.strafe + ahead) * 3.0 + Vec3::Y * 1.6);
        }
        // The look, composed after the gait, independent of it.
        // Retargeted in place, so the look eases from where it is.
        // On a ladder, by the neck and head alone: the spine's share bent the
        // chest, and the hands posed on their rungs went 8-9 cm off them.
        state.look.target = look_at;
        let mut look_config = lookat::LookAtConfig::default();
        if state.on_holds() {
            for (bone, share) in &mut look_config.chain.iter_mut().map(|b| (b.bone, &mut b.share)) {
                *share = match bone {
                    Bone::Neck => 0.4,
                    Bone::Head => 0.6,
                    _ => 0.0,
                };
            }
        }
        if let Some(direction) = state.look.advance(
            lookat::head_position(&target.pose, &RigGeometry::default()) + root.translation,
            state.facing.rotation(),
            &look_config,
            time.delta_secs(),
        ) {
            lookat::apply(&mut target.pose, direction, &look_config, &RigGeometry::default());
        }

        // The reach is solved by the plugin in `AnimSet::Ik`, against the
        // live rig: solved here against the synthetic proxy (mirrored from
        // the real rig) it sent the hand to the wrong side of the body.
        // On a ladder, its hands are on the rungs.
        arm_ik.left = walker.reach.filter(|_| !state.on_holds());

        match steer {
            Steer::Straight => {}
            // A steady circle: always a quarter turn ahead.
            Steer::Circle(rate) if rate != 0.0 => {
                state.facing.target_yaw = facing::shortest_angle(state.facing.yaw + rate.signum() * std::f32::consts::FRAC_PI_2);
                state.facing.turn_rate = rate.abs();
            }
            Steer::Circle(_) => {}
            Steer::Toward { yaw, rate } => {
                state.facing.target_yaw = facing::shortest_angle(yaw);
                state.facing.turn_rate = rate;
            }
        }
        // Going aside and forward at once, mostly forward: the body turned
        // to its way, by the angle off where it was steered to face (the
        // head kept there, below). A fresh target takes the whole turn; a
        // held one the change. Not on a circle.
        let strafe = if matches!(steer, Steer::Circle(_)) { 0.0 } else { strafe };
        match steer {
            Steer::Toward { .. } => state.facing.target_yaw = facing::shortest_angle(state.facing.target_yaw + strafe),
            Steer::Straight => state.facing.target_yaw = facing::shortest_angle(state.facing.target_yaw + strafe - state.strafe),
            Steer::Circle(_) => {}
        }
        state.strafe = strafe;

        let turn = locomotion::advance_turning_with(
            &mut state.locomotion,
            &mut state.facing,
            cycle,
            // The rate `cycle` actually advances at, the leg clock's own: paired
            // with a smoothed cadence instead, the planted foot slid 14% at
            // 1 m/s.
            phase.gait_frequency_hz(),
            &params,
            &rendered,
            &gait_rig,
            time.delta_secs(),
        );
        // The foot locks need the same frame's turn, so a planted foot
        // pivots with the body rather than being dragged by it.
        foot_ik.turn = turn;

        state.stride.params = Some(params);
        state.stride.cycle = cycle;
        state.stride.weight = weight;
        if let_go {
            state.transition = super::transition::Transition::standing();
            state.gaits.walk();
            state.pace = 0.0;
        }

        // The entity turns to match the heading, composed onto the asset's
        // own correction (heading first, then the correction): assigned over
        // it, the correction was wiped and the character walked backward.
        root.rotation = state.facing.rotation() * correction.map_or(Quat::IDENTITY, |c| c.0);
    }
}

/// Fades `layer`'s walk sway and pelvic turn out as the gait is `running`
/// (0 walking, 1 running; a shuffle passes 1). They are a walk's: the pelvis turned about
/// whichever feet a walk's stance timing has loaded. On a run's clock (a
/// third of the stride down, and flights) its pivot jumped between the
/// feet, and with it the root, 15 mm back in three frames once a step; and
/// under a leap from the run, a 9 mm step as it handed back. The run's
/// pelvis tilt is in its own pose.
pub fn fade_walk_sway(layer: &mut PhaseLayer, running: f32) {
    if let Some(walk) = layer.walk_sway.as_mut() {
        walk.gain *= 1.0 - running.clamp(0.0, 1.0);
    }
}

/// Moves each walker by exactly how far its planted feet moved under it in
/// the pose just RENDERED, after the springs, before the IK: integrating the
/// gait's published velocity instead erred by `½·a·dt²` and by the springs'
/// lag, and a planted foot slid 39 mm a stance.
pub fn ride_rendered_feet(time: Res<Time>, mut rigs: Query<(&AnimPose, &mut WalkerState, &mut AnimFootIk, &mut Transform, &AnimGround)>) {
    for (pose, mut state, mut foot_ik, mut root, ground) in &mut rigs {
        let state = &mut *state;
        let now = pose.pose();
        let rig = foot_ik.rig.clone().unwrap_or_default();
        let moved = match (&state.stride.params, &state.stride.previous) {
            // Standing, the idle sways the pelvis over feet that stay put;
            // read as root motion, that sway walked the character.
            _ if state.stride.weight <= 0.0 => Vec3::ZERO,
            // Jumping, its travel is the jump's (`Stride::stepped`); from a
            // run, the run's clock goes on under it, and its velocity would
            // carry the body a second time.
            _ if state.jump.is_some() || std::mem::take(&mut state.stride.given) => Vec3::ZERO,
            // Running, at the run's own speed: its clock is set so a stride
            // covers exactly that, so over a stride the feet do not drift.
            // A runner's body changes speed by a few per cent through a
            // stride, and the run's feet down are told to the locks from the
            // clock, which hold them in the world whatever the pose does.
            //
            // At the gait's contact velocity instead (`locomotion::
            // root_velocity_of`), the body followed the recorded heel, which
            // lands still moving forward (18 mm over the first tenth of
            // stance, carried at the run's speed): it slowed to 2.4 m/s at
            // every contact at a 4 m/s run, the pelvis stepping 45 mm in a
            // frame of 70. From the rendered contacts, worse: each landing
            // foot, its sprung leg still swinging, braked the body from 3.2
            // to 0.3-1.5 m/s, and it ran 15 % slow.
            // Changing between the two, the planted foot it changes on:
            // both gaits have it in the same place (`run::Gaits::phases`).
            _ if state.gaits.running >= 0.5 && !state.gaits.changing() => {
                state.facing.rotation() * rig.forward() * (state.transition.stride_speed * time.delta_secs())
            }
            (Some(params), Some(previous)) => {
                // The cycle half-way through the frame decides which feet
                // are planted.
                let middle = state.stride.previous_cycle + 0.5 * (state.stride.cycle - state.stride.previous_cycle).rem_euclid(1.0);
                locomotion::root_displacement_between(previous, &now, middle, params, &rig)
                    .map(|moved| state.facing.rotation() * moved)
                    // No foot down, a run's flight: the body coasts.
                    .unwrap_or(state.locomotion.root_velocity * time.delta_secs())
            }
            _ => Vec3::ZERO,
        };
        // A balance recovery step carries the character too.
        let moved = moved + state.facing.rotation() * std::mem::take(&mut state.stride.stepped);
        state.locomotion.position += moved;
        // The foot locks keep a planted foot where it is in the WORLD only if
        // they know the body moved over it.
        foot_ik.turn.travel = moved;
        state.stride.previous = Some(now);
        state.stride.previous_cycle = state.stride.cycle;

        // Travel moves the ENTITY, not the pose's root translation (routed
        // through the rig's hips frame, that walked a Z-up rig into the
        // floor).
        let height_before = root.translation.y;
        root.translation = state.locomotion.position;
        // Height from the ground: horizontal travel alone left a character
        // 7.7 m under a hillside after 26 m of a 0.3 grade. Not on a ladder,
        // whose climb carries the root up and down.
        if !state.on_holds()
            && let Some(height) = locomotion::ground_following_height(root.translation, 0.0, ground.0.as_ref())
        {
            root.translation.y = height;
        }
        // The rise is travel too: a lock that knew only the horizontal part
        // carried a planted foot 9 cm up a 0.2 grade every stance.
        foot_ik.turn.travel.y = root.translation.y - height_before;
    }
}

/// A ragdolled walker falls when its balance finds no step that catches it
/// (`Balance::falls`), or when asked ([`Walker::fall_now`]). The balances are
/// reset: the body is the physics' now, and a stumble still being posed
/// underneath would move the character too. So is the gait, to a stand:
/// the body rises from the ground into standing, not into a walk already
/// under way, and walks on with a start once up (`drive_walkers`).
pub fn fall_when_uncaught(mut rigs: Query<(&mut Walker, &mut balance::Balance, &mut WalkBalance, &mut Ragdoll, &mut AnimFootIk, &mut WalkerState)>) {
    for (mut walker, mut balance, mut walk_balance, mut ragdoll, mut foot_ik, mut state) in &mut rigs {
        let asked = std::mem::take(&mut walker.fall_now);
        if ragdoll.is_falling() || !(asked || balance.falls || walk_balance.falls) {
            continue;
        }
        info!(
            "walker: falling ({})",
            if balance.falls {
                format!("a push asked for a {:.2} m step", balance.wanted_step)
            } else if walk_balance.falls {
                format!("a push while walking asked for a {:.2} m step", walk_balance.wanted_step)
            } else {
                "asked".into()
            }
        );
        // The push goes with it, the part not yet delivered too.
        let pushed = balance.velocity + balance.pending_push() + walk_balance.velocity + walk_balance.pending_push();
        let launch = foot_ik.rig.as_ref().map_or(Vec3::ZERO, |rig| state.facing.rotation() * (rig.forward() * pushed.x + rig.left() * pushed.y));
        ragdoll.fall_moving(FALL_TONE, walker.fall_damping, launch);
        *balance = balance::Balance::default();
        *walk_balance = WalkBalance::default();
        foot_ik.displaced = [Vec3::ZERO; 2];
        foot_ik.planted = [false; 2];
        foot_ik.landing = None;
        state.transition = transition::Transition::standing();
    }
}

/// A fallen walker that has come to rest lies for its
/// [`Walker::getup_delay`], then rises through the get-up keys for how it
/// lies (`Ragdoll::get_up`), turned to face the way it gets up.
pub fn get_up_when_rested(mut ragdolls: Query<(&Walker, &mut Ragdoll, &mut WalkerState)>) {
    for (walker, mut ragdoll, mut state) in &mut ragdolls {
        if ragdoll.fall.is_some_and(|fall| fall.at_rest && fall.rise.is_none()) && ragdoll.get_up(walker.getup_delay) {
            info!("walker: at rest, getting up");
        }
        if let Some(rise) = ragdoll.fall.as_mut().and_then(|fall| fall.rise.as_mut())
            && rise.turn_pending
        {
            state.facing.yaw += rise.turn;
            state.facing.target_yaw = state.facing.yaw;
            rise.turn_pending = false;
            info!("walker: lying {:?}, turning {:.0}° to get up", rise.lying, rise.turn.to_degrees());
        }
    }
}

/// While a ragdoll falls it moves the character after its body
/// (`AnimRagdollPlugin`); root motion takes that position up, or it wrote the
/// old one back as the body stopped being followed (every rise slid the
/// character 0.45 m back to where it fell from).
pub fn follow_the_fallen_body(mut rigs: Query<(&Ragdoll, &Transform, &mut WalkerState)>) {
    for (ragdoll, transform, mut state) in &mut rigs {
        if ragdoll.is_falling() {
            state.locomotion.position.x = transform.translation.x;
            state.locomotion.position.z = transform.translation.z;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::stance::stance_on_rig;

    /// The root's worst forward move in a 60 Hz frame against the frame
    /// before, metres, over two strides of a 4 m/s run, the locomotion layer
    /// composed on with its walk sway faded for `running`.
    fn worst_root_jolt(running: f32) -> f32 {
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        let stood = stance_on_rig(&poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let speed = 4.0;
        let params = GaitParams::running_on(speed, &rig);
        let reference = super::super::run::reference_speed(speed, super::super::gait::leg_length_of(&rig));
        let distance = super::super::run::distance_per_cycle(&stood, &rig, reference);
        let mut layer = PhaseLayer::locomotion();
        fade_walk_sway(&mut layer, running);
        let dt = 1.0 / 60.0;
        let ahead: Vec<f32> = (0..80)
            .map(|frame| {
                let phase = GaitPhase { gait: frame as f32 * dt * speed / distance * TAU, speed, base_frequency_hz: 0.0, speed_coefficient: 1.0 / distance, ..Default::default() };
                let mut pose = walk_pose_on(cycle_of(&phase), &params, &stood, &rig);
                layer.apply_on(&phase, &mut pose, &rig);
                pose.root_translation.dot(rig.forward())
            })
            .collect();
        let steps: Vec<f32> = ahead.windows(2).map(|w| w[1] - w[0]).collect();
        steps.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f32::max)
    }

    /// Running, the locomotion layer leaves the root going forward steadily
    /// (the run's body moves at its speed): its walk sway, turning the
    /// pelvis about the feet a walk's stance timing loads, jumped the root
    /// 15 mm back in three frames once a step on a run's clock.
    #[test]
    fn running_the_layer_leaves_the_root_going_steadily() {
        let (running, walking) = (worst_root_jolt(1.0), worst_root_jolt(0.0));
        assert!(running < 1.0e-4, "running, the root's step changes {:.2} mm in a frame", running * 1e3);
        assert!(walking > 3.0e-3, "the walk's sway on a run's clock jolts the root only {:.2} mm", walking * 1e3);
    }
}
