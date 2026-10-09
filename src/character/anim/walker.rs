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
    /// Falling (letting go, or off an edge), reach up and catch a ledge it
    /// falls past: [`Walker::ledge`] or any of [`Walker::ledges`] it faces.
    pub catch: bool,
    /// Other ledges about it, which a hang shimmies onto round a corner
    /// where one meets its ledge's end.
    pub ledges: Vec<super::parkour::Ledge>,
    /// Asked of [`Walker::ledge`]: `Grab` walks under it, jumps and hangs
    /// from it (`parkour::hang`). Out of a standing jump's reach, the ask is
    /// dropped. `ClimbUp` climbs onto its top and stands there (on the
    /// walker's ground, [`super::parkour::LedgeGround`]), grabbing it first
    /// if not hanging yet; the ask is dropped once taken. `Shimmy` goes
    /// along it hand over hand while asked (grabbing it first). Hanging,
    /// nothing else asked of it is done.
    pub hang: Option<super::parkour::hang::HangAsk>,
    /// Beams about it (`parkour::beam`): on one, it walks along its line at
    /// a beam's pace, the feet nearly on the line, the arms out.
    pub beams: Vec<super::parkour::beam::Beam>,
    /// Crawl on hands and knees along its facing while asked
    /// (`parkour::crawl`): it gets down from standing first, and up again
    /// once not asked. Crawling, nothing else asked of it is done.
    pub crawl: bool,
    /// Squeeze sideways along a narrow passage (`parkour::squeeze`): it
    /// walks to its mouth, turns square to it, shuffles along it, and walks
    /// on once through (the ask then dropped).
    pub squeeze: Option<super::parkour::squeeze::Squeeze>,
    /// A wall of holds to free climb (`parkour::holds`).
    pub holds: Option<super::parkour::holds::HoldWall>,
    /// Climb [`Walker::holds`] this way (on the face: `x` toward its left
    /// facing it, `y` up; zero holds still): it walks to the wall and gets
    /// on first. On it, `None` holds still; asked down at the bottom it
    /// steps off, up at the top it takes the lip into a hang and climbs up.
    pub free_climb: Option<bevy::math::Vec2>,
    /// The pole to climb (`parkour::pole`).
    pub pole: Option<super::parkour::Pole>,
    /// Asked of [`Walker::pole`]: `Up` walks to it, gets on and climbs while
    /// asked (to its top); on it, `Down` climbs down (letting go onto the
    /// floor at the bottom), `Slide` slides to the floor, `Round` goes round
    /// it while asked, `LetGo` lets go (reaching to catch a ledge if
    /// [`Walker::catch`]). The asks that end on the floor are dropped once
    /// taken. On it, nothing else asked of it is done.
    pub on_pole: Option<super::parkour::pole::PoleAsk>,
    /// A small top to jump onto from standing (`parkour::precision`), its
    /// middle (the world): it turns to face it, jumps, lands on it and
    /// balances there, its arms out; out of reach, the ask is dropped. Its
    /// top must be among [`Walker::ledges`] for it to stand there.
    pub onto: Option<Vec3>,
    /// Crouch into a perch where it stands (`parkour::perch`), on a post or
    /// a narrow top; up again when not asked. Perched, it rises before it
    /// walks or jumps.
    pub perch: bool,
    /// Look round, the gaze swept slowly side to side (a viewpoint's).
    pub look_round: bool,
    /// A jump from standing that turns this far in the air, radians about
    /// `+Y` (`parkour::spin`; π turns round); the ask is dropped once taken.
    pub spin_jump: Option<f32>,
    /// A leap of faith into this pile from standing on a top's edge
    /// (`parkour::faith`): it turns to face it, dives, lands on its back in
    /// it and rises out; the ask is dropped once taken (or out of reach).
    pub leap_of_faith: Option<super::parkour::faith::Haystack>,
    /// Running fast (`parkour::skid::SKID_FROM`), asked to stop it skids to
    /// a stop, and asked to face back (steered over [`SKID_BACK`] off its
    /// facing) it plants and turns round, standing; otherwise it slows to a
    /// walk first and turns at its facing's rate.
    pub skid: bool,
    /// Running at this springboard (`parkour::springboard`): its last steps
    /// paced to bring a foot down on its end, it leaps off it as much
    /// higher as the board gives, and lands on what it comes down on, or
    /// catches (with [`Self::catch`]); the ask is dropped once taken, or run
    /// past.
    pub springboard: Option<super::parkour::springboard::Springboard>,
    /// Monkey bars to cross (`parkour::monkey`): it walks under the first,
    /// gets on, crosses hand over hand, lets go under the last and lands;
    /// the ask is dropped once let go.
    pub monkey_bars: Option<super::parkour::monkey::MonkeyBars>,
    /// A flagpole to swing round (`parkour::flagpole`): falling by it (off
    /// an edge, or a jump's flight gone over one), it reaches for it,
    /// catches it, swings round and is let go on the way up; the ask is
    /// dropped once let go.
    pub flagpole: Option<super::parkour::flagpole::Flagpole>,
    /// A post at a corner to swing round (`parkour::corner`), and how far
    /// round, radians (positive to the left): running past it, the last
    /// steps paced, it leaps, its near hand on the post, the way it goes
    /// bent round it, and runs on; the ask is dropped once taken, or run
    /// past.
    pub corner: Option<(super::parkour::Pole, f32)>,
    /// Hooks or pots hung up to swing on (`parkour::flagpole`, a hook):
    /// falling by one, it reaches for it, catches it one-handed, swings
    /// forward and lets go; on to the next it falls by.
    pub hooks: Vec<Vec3>,
    /// Moving platforms (`parkour::platform`), written by the app each
    /// frame (share the handle with the character's `PlatformGround`, so
    /// the feet stand on them): standing, walking or jumping on one, it is
    /// carried with it; leaving it, it carries its velocity; falling onto
    /// one, it lands on it where it is.
    pub platforms: super::parkour::platform::Platforms,
    /// Slopes too steep to walk (`parkour::slope`: roofs, steep faces), on
    /// its ground too (`SlopeGround`): walked or run onto one down it, it
    /// slides down it on its feet; run out at its foot, it stands; at a
    /// drop, it goes over the edge falling (catching the eave if asked to
    /// catch and it is among its ledges); asked to jump, it leaps off.
    pub slopes: Vec<super::parkour::slope::Slope>,
}

/// Steered this far off its facing, radians, a skidding walker turns round
/// on the spot ([`Walker::skid`]).
pub const SKID_BACK: f32 = 2.4;

/// Going over a slope's edge to catch its eave, the turn round to face it
/// takes this long, seconds (`parkour::slope`).
const SLOPE_CATCH_TURN: f32 = 0.3;

/// Off a top it jumped onto by this far, metres, it no longer balances on
/// it ([`Walker::onto`]).
const PERCH_LEFT: f32 = 0.35;

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
            catch: false,
            ledges: Vec::new(),
            hang: None,
            beams: Vec::new(),
            crawl: false,
            squeeze: None,
            holds: None,
            free_climb: None,
            pole: None,
            on_pole: None,
            onto: None,
            perch: false,
            look_round: false,
            spin_jump: None,
            leap_of_faith: None,
            skid: false,
            springboard: None,
            monkey_bars: None,
            flagpole: None,
            corner: None,
            hooks: Vec::new(),
            platforms: Default::default(),
            slopes: Vec::new(),
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

/// Running at an obstacle to vault, it aims a foot this far beyond its best
/// take-off, metres (a speed vault plans from the best to 0.4 m beyond it,
/// a lazy vault 34° off square to 0.1 m), and
/// stretches or shortens its pace for it by at most this share, adjusting
/// its last steps as a long jumper does.
const VAULT_AIM: f32 = 0.1;
const VAULT_PACE: f32 = 0.2;
/// A run up a wall is taken off as much as this nearer than its best,
/// metres, at most (it plans to 0.1-0.25 m either side).
const WALL_TAKEOFF_NEAR: f32 = 0.1;
/// Asked to run along a wall, with less of it than this left ahead, metres,
/// the ask is dropped (a run along it covers about 3 m).
const RUN_ALONG_LEFT: f32 = 1.0;

/// Walking in to mantle, it mantles from a foot down within this of its
/// spot, metres (about the reach of the planted foot to its own spot), if
/// it walks faster than this, m/s, and faces within this of square to the
/// wall, radians; else it stops on the spot first.
const MANTLE_FROM_WALK: f32 = 0.5;
const MANTLE_WALKING: f32 = 0.3;
const MANTLE_SQUARE: f32 = 0.2;

/// How long both feet stay planted after standing up, seconds: several
/// times the legs' 0.015 s spring half-life, for the extension to settle.
const STOOD_HOLD: f32 = 0.3;

/// Walking, the body is kept this far from a wall, metres, round the root:
/// less than the 0.25 m the hips stand off a wall faced; looked for this
/// many ways round. It stops for a wall straight ahead this much further
/// off, and as far as it goes in this long at its speed.
pub const BODY_RADIUS: f32 = 0.2;
/// On a beam, it turns back onto its line at this rate, rad/s.
const BEAM_TURN: f32 = 2.0;
/// Asked to slide under a slab, too slow to yet, it gives up nearer than
/// this, metres.
const SLIDE_TOO_NEAR: f32 = 1.5;
/// It walks under anything wholly this far over what it stands on, metres
/// (a bar to swing on, 2.3 m up).
pub const HEADROOM: f32 = 2.0;
const WALL_PROBES: usize = 32;
const WALL_STOP_MARGIN: f32 = 0.15;
const WALL_STOP_TIME: f32 = 0.3;
/// Going round a wall, the turns off the way wanted it tries, radians, and
/// how fast it turns onto one (as the approach turns, rad/s).
pub const DETOUR_STEP: f32 = 0.2618;
const DETOUR_RATE: f32 = 3.0;
/// A turn off the way is a way round only if clear this far, metres.
const DETOUR_CLEAR: f32 = 1.5;
/// Back to its line, it heads for the point on it this far ahead, metres,
/// turned at most this far off its way, radians; on it within this,
/// metres, the detour is over.
const LINE_LOOKAHEAD: f32 = 1.5;
const LINE_MOST_TURN: f32 = std::f32::consts::FRAC_PI_4;
const LINE_ON: f32 = 0.05;
/// ... and facing its way within this, radians: over within half a turn
/// step (7.5°), it walked on straight off its line, 0.38 m in 12 m.
const LINE_FACING: f32 = 0.01;

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
    /// Falling off an edge and landing below (`parkour::fall`); and the
    /// ground found below as the root went over the edge, the fall to start.
    pub falling: Option<super::parkour::Falling>,
    pub fall_to: Option<f32>,
    /// Running up a wall (`parkour::wall`), until it lets go into a fall.
    pub wall_run: Option<super::parkour::wall::WallRun>,
    /// On a pole ([`Walker::on_pole`]), until it lets go into a fall; and
    /// the pole it walks to with the spot it gets on from.
    pub poling: Option<super::parkour::Poling>,
    pub pole_spot: Option<(super::parkour::Pole, Vec3)>,
    /// How far onto a beam's balance it is (0-1, `parkour::beam`), and how
    /// long it has balanced, seconds (the sway's clock).
    pub beam: f32,
    pub beam_time: f32,
    /// Teetering at an edge it stopped at (`parkour::teeter`), seconds in;
    /// and whether it has this stop (once a stop).
    pub teeter: Option<f32>,
    pub teetered: bool,
    /// Sliding under a slab from a run (`parkour::underslide`).
    pub under_slide: Option<super::parkour::underslide::UnderSlide>,
    /// Crawling, from getting down to standing up (`parkour::crawl`).
    pub crawling: Option<super::parkour::crawl::Crawling>,
    /// Squeezing along a passage (`parkour::squeeze`): the way it faces
    /// through it (chosen once, as it is asked), whether it is in it, and
    /// how far its arms are flattened (0-1).
    pub squeeze_facing: Option<Vec3>,
    pub squeezing: bool,
    pub squeezed: f32,
    /// Free climbing a wall of holds (`parkour::holds`), and the spot it
    /// walks to to get on.
    pub free_climbing: Option<super::parkour::holds::FreeClimb>,
    pub holds_spot: Option<Vec3>,
    /// Skidding to a stop or round from a run (`parkour::skid`).
    pub skid: Option<super::parkour::skid::Skid>,
    /// Sliding down a slope too steep to walk (`parkour::slope`).
    pub sloping: Option<super::parkour::slope::SlopeSlide>,
    /// Jumping onto a small top (`parkour::precision`), and the top it
    /// stands on after, balancing.
    pub onto: Option<Vec3>,
    pub perched: Option<Vec3>,
    /// A turning jump's turn, until it is handed to its fall
    /// (`parkour::spin`).
    pub spinning: Option<f32>,
    /// A leap of faith under way (`parkour::faith`).
    pub faith: Option<super::parkour::faith::LeapOfFaith>,
    /// Leaping off a springboard, until handed to its fall
    /// (`parkour::springboard`); the board, as it is bent.
    pub springing: Option<super::parkour::springboard::Springboard>,
    /// Crossing monkey bars (`parkour::monkey`), and the spot it walks to
    /// under the first.
    pub monkey: Option<super::parkour::monkey::Crossing>,
    pub monkey_spot: Option<(super::parkour::monkey::MonkeyBars, Vec3)>,
    /// Swinging round a flagpole, or on a hook (`parkour::flagpole`), and
    /// the hook last let go of (not caught again falling from it).
    pub flagging: Option<super::parkour::flagpole::Swinging>,
    pub last_hook: Option<Vec3>,
    /// Swinging round a corner post in a leap (`parkour::corner`).
    pub cornering: Option<super::parkour::corner::CornerSwing>,
    /// The moving platform (`Walker::platforms`, by index) it is carried
    /// with: stood on, jumped on, or fallen in the frame of.
    pub riding: Option<usize>,
    /// How far that carried it this frame (the world).
    pub carried: Vec3,
    /// Perching (`parkour::perch`): how far crouched into it (0-1, eased),
    /// and its pose on the rig bound (made once).
    pub perch_weight: f32,
    pub perch_pose: Option<LocalPose>,
    /// Looking round, seconds in.
    pub look_round_t: f32,
    /// A hand on a wall beside it (`parkour::wallhand`): how far on (0-1,
    /// eased), and the wall (kept while it eases off).
    pub wall_hand: f32,
    pub wall_beside: Option<super::parkour::wallhand::Beside>,
    /// Running, its lean with its acceleration (`parkour::lean`), and its
    /// facing last frame, for its turn's rate.
    pub lean: super::parkour::lean::Lean,
    pub lean_yaw: f32,
    /// Kicked across to another wall to chain a kick off it: that wall and
    /// the lip to kick to from it.
    pub kick_chain: Option<(super::parkour::Ledge, super::parkour::Ledge)>,
    /// Going round a wall ([`way_round`]).
    pub detour: Option<Detour>,
    /// The ledge it walks under, the spot it jumps from, and whether it has
    /// come round in front of it to walk straight in (`LEDGE_LEAD_IN`).
    pub ledge_spot: Option<(super::parkour::Ledge, Vec3, bool)>,
    /// Running at an obstacle to vault, how much faster or slower than asked
    /// it runs, so a foot comes down at its best take-off (1 otherwise).
    pub vault_pace: f32,
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
            falling: None,
            wall_run: None,
            poling: None,
            pole_spot: None,
            beam: 0.0,
            beam_time: 0.0,
            teeter: None,
            teetered: false,
            under_slide: None,
            crawling: None,
            squeeze_facing: None,
            squeezing: false,
            squeezed: 0.0,
            free_climbing: None,
            holds_spot: None,
            skid: None,
            sloping: None,
            onto: None,
            perched: None,
            spinning: None,
            faith: None,
            springing: None,
            monkey: None,
            monkey_spot: None,
            flagging: None,
            last_hook: None,
            cornering: None,
            riding: None,
            carried: Vec3::ZERO,
            perch_weight: 0.0,
            perch_pose: None,
            look_round_t: 0.0,
            wall_hand: 0.0,
            wall_beside: None,
            lean: Default::default(),
            lean_yaw: yaw,
            kick_chain: None,
            fall_to: None,
            detour: None,
            ledge_spot: None,
            vault_pace: 1.0,
            measured: None,
        }
    }

    /// Whether a move poses the body off the floor, its hands on holds or in
    /// the air: on a ladder, grabbing or hanging from a ledge, running up a
    /// wall, or falling off an edge and landing.
    pub fn on_holds(&self) -> bool {
        self.climbing.is_some() || self.hanging.is_some() || self.falling.is_some() || self.wall_run.is_some() || self.poling.is_some() || self.under_slide.is_some() || self.crawling.is_some() || self.free_climbing.is_some() || self.skid.is_some() || self.faith.is_some() || self.monkey.is_some() || self.flagging.is_some() || self.sloping.is_some()
    }

    /// Leaping off a springboard, the board and how far it is bent under the
    /// take-off foot now, metres.
    pub fn springboard_bent(&self) -> Option<(super::parkour::springboard::Springboard, f32)> {
        let board = self.springing?;
        Some((board, self.jump.as_ref().map_or(0.0, |jump| jump.board_sunk())))
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
    // What it walks round, going to a chair; and what it stands on (below a
    // hang it lets go of, it falls to).
    (Option<&'static super::obstacles::RouteObstacles>, Option<&'static AnimGround>),
    // The springs a jump's pose is led ahead of (`jump::Jump::pose_led`).
    Option<&'static super::plugin::AnimSprings>,
    // Its fingers, closed round a ladder's rungs and rails.
    Option<&'static mut super::hand::RelaxedHands>,
);

/// Drives each walker's gait from its clock, in `AnimSet::Target`, so the
/// phase layer composes on top and the springs smooth the result.
pub fn drive_walkers(time: Res<Time>, mut rigs: Query<WalkingRig>) {
    for (mut walker, mut target, mut phase, mut state, mut arm_ik, mut foot_ik, mut root, correction, mut layer, mut balance, mut walk_balance, ragdoll, (route_obstacles, ground), springs, mut hands) in
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

        // On a moving platform (`parkour::platform`), or in a jump or fall
        // in its frame: carried as far as it moved, its own motion on top.
        // The foot locks are not told (they ride with the body): planted on
        // the platform, they go with it.
        let platforms = walker.platforms.now();
        let carried = match state.riding.and_then(|i| platforms.get(i)) {
            Some(platform) => platform.moved,
            None => {
                state.riding = None;
                Vec3::ZERO
            }
        };
        state.carried = carried;
        if carried != Vec3::ZERO {
            state.locomotion.position += carried;
            if let Some(falling) = state.falling.as_mut() {
                falling.shift(carried);
            }
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
        // A vault is taken running at it, not walked to (below).
        let hang_asked = !state.on_holds()
            && walker.hang.is_some_and(|ask| {
                !matches!(
                    ask,
                    super::parkour::hang::HangAsk::Vault(_)
                        | super::parkour::hang::HangAsk::WallRun
                        | super::parkour::hang::HangAsk::WallKick
                        | super::parkour::hang::HangAsk::RunAlong
                        | super::parkour::hang::HangAsk::SlideUnder
                )
            })
            && walker.ledge.is_some()
            && walker.sit.is_none()
            && !ladder_asked
            && state.posture.is_standing()
            && !fallen;
        let mut at_ledge = false;
        // Asked up a pole (`parkour::pole`): it walks to the spot in front of
        // it, facing it, and gets on once stopped.
        let pole_asked = !state.on_holds()
            && walker.on_pole == Some(super::parkour::pole::PoleAsk::Up)
            && walker.pole.is_some()
            && walker.sit.is_none()
            && state.posture.is_standing()
            && !fallen;
        let mut at_pole = false;
        // Asked across monkey bars (`parkour::monkey`): it walks to the spot
        // under the first, facing along them, and gets on once stopped.
        let monkey_asked = !state.on_holds() && walker.monkey_bars.is_some() && walker.sit.is_none() && state.posture.is_standing() && !fallen;
        let mut at_monkey = false;
        // Asked to squeeze along a passage (`parkour::squeeze`): to its mouth,
        // square to it; then in it, shuffling along (below).
        if walker.squeeze.is_none() {
            (state.squeezing, state.squeeze_facing) = (false, None);
        }
        let squeeze_asked = !state.on_holds() && !state.squeezing && walker.squeeze.is_some() && walker.sit.is_none() && state.posture.is_standing() && !fallen;
        // Asked to free climb (`parkour::holds`): to the spot in front of the
        // wall, facing it; got on once stopped (below).
        let holds_asked = !state.on_holds() && walker.free_climb.is_some() && walker.holds.is_some() && walker.sit.is_none() && state.posture.is_standing() && !fallen;
        let mut at_holds = false;
        match (walker.sit, walker.chair, foot_ik.rig.as_ref()) {
            _ if state.on_holds() => {
                state.approach = approach::Approach::Idle;
                (wanted_speed, steer) = (0.0, Steer::Straight);
            }
            (_, _, Some(rig)) if holds_asked => {
                let wall = walker.holds.as_ref().expect("asked to climb");
                let ahead = approach::heading_of(rig.forward());
                let spot = *state.holds_spot.get_or_insert_with(|| super::parkour::holds::FreeClimb::spot(wall, state.locomotion.position));
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                let obstacles: Vec<_> = route_obstacles.map(|route| route.0.clone()).unwrap_or_default();
                match state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, spot, approach::heading_of(-wall.out), speed, &obstacles) {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        (wanted_speed, steer, at_holds) = (0.0, Steer::Straight, true);
                    }
                }
                if look_at.is_none() {
                    look_at = Some(wall.face + Vec3::Y * 1.8);
                }
            }
            (_, _, Some(rig)) if squeeze_asked => {
                let squeeze = walker.squeeze.expect("asked to squeeze");
                let ahead = approach::heading_of(rig.forward());
                let facing = *state.squeeze_facing.get_or_insert_with(|| squeeze.facing_for(Quat::from_rotation_y(state.facing.yaw) * rig.forward()));
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                let obstacles: Vec<_> = route_obstacles.map(|route| route.0.clone()).unwrap_or_default();
                match state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, squeeze.from, approach::heading_of(facing), speed, &obstacles) {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        (wanted_speed, steer) = (0.0, Steer::Straight);
                        state.squeezing = true;
                        state.approach = approach::Approach::Idle;
                    }
                }
            }
            (_, _, Some(rig)) if state.squeezing => {
                let squeeze = walker.squeeze.expect("squeezing");
                let ahead = approach::heading_of(rig.forward());
                let facing = state.squeeze_facing.unwrap_or_else(|| squeeze.facing_for(rig.forward()));
                (wanted_speed, steer) = (0.0, Steer::Toward { yaw: approach::heading_of(facing) - ahead, rate: 2.0 });
                if look_at.is_none() {
                    look_at = Some(squeeze.to + Vec3::Y * 1.6);
                }
                if squeeze.is_through(state.locomotion.position) {
                    walker.squeeze = None;
                    (state.squeezing, state.squeeze_facing) = (false, None);
                }
            }
            (_, _, Some(rig)) if pole_asked => {
                let pole = walker.pole.expect("asked up a pole");
                let ahead = approach::heading_of(rig.forward());
                if state.pole_spot.is_none_or(|(was, _)| was != pole) {
                    let stood = stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig);
                    state.pole_spot = Some((pole, super::parkour::Poling::spot(&pole, state.locomotion.position, &stood, rig)));
                    state.approach = approach::Approach::Idle;
                }
                let (_, spot) = state.pole_spot.expect("a spot");
                let facing = approach::heading_of((pole.foot - spot).with_y(0.0).normalize_or(rig.forward()));
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                let obstacles: Vec<_> = route_obstacles.map(|route| route.0.clone()).unwrap_or_default();
                match state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, spot, facing, speed, &obstacles) {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        (wanted_speed, steer, at_pole) = (0.0, Steer::Straight, true);
                    }
                }
                if look_at.is_none() {
                    look_at = Some(pole.at(1.8));
                }
            }
            (_, _, Some(rig)) if monkey_asked => {
                use super::parkour::monkey::Crossing;
                let bars = walker.monkey_bars.expect("asked across monkey bars");
                let ahead = approach::heading_of(rig.forward());
                if state.monkey_spot.is_none_or(|(was, _)| was != bars) {
                    let stood = stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig);
                    state.monkey_spot = Some((bars, Crossing::spot(&bars, state.locomotion.position.y, &stood, rig)));
                    state.approach = approach::Approach::Idle;
                }
                let (_, spot) = state.monkey_spot.expect("a spot");
                let facing = approach::heading_of(bars.way);
                let speed = if walker.speed > 0.0 { walker.speed } else { approach::APPROACH_SPEED };
                placing = true;
                let gait = approach_gait(state, cycle_of(&phase), &gait_rig);
                let obstacles: Vec<_> = route_obstacles.map(|route| route.0.clone()).unwrap_or_default();
                match state.approach.advance(state.locomotion.position, state.facing.yaw + ahead, &gait, spot, facing, speed, &obstacles) {
                    approach::Order::Walk { speed, heading, rate } => {
                        (wanted_speed, steer, arrived) = (speed, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Stop { heading, rate } => {
                        (wanted_speed, steer, arrived) = (0.0, Steer::Toward { yaw: heading - ahead, rate }, false);
                    }
                    approach::Order::Arrived => {
                        (wanted_speed, steer, at_monkey) = (0.0, Steer::Straight, true);
                    }
                }
                if look_at.is_none() {
                    look_at = Some(bars.first);
                }
            }
            (_, _, Some(rig)) if hang_asked => {
                let ledge = walker.ledge.expect("asked to grab a ledge");
                let ahead = approach::heading_of(rig.forward());
                let square = super::parkour::Hanging::square(&ledge, rig.forward());
                if state.ledge_spot.is_none_or(|(was, _, _)| was != ledge) {
                    let stood = stance_on_rig(&base, DEFAULT_KNEE_FLEX, rig);
                    // Dropping down, the spot on the top it lowers itself from
                    // (where a climb up from the hang ends), walked to
                    // directly: no lead-in in front of a wall.
                    state.ledge_spot = Some(if walker.hang == Some(super::parkour::hang::HangAsk::DropDown) {
                        let grips = hands.as_ref().map_or([None; 2], |hands| hands.grips);
                        let hung = super::parkour::Hanging::hung(&ledge, &walker.ledges, state.locomotion.position, foot_ik.pelvis_drop, grips, &stood, rig);
                        (ledge, hung.standing_spot(), true)
                    } else if walker.hang == Some(super::parkour::hang::HangAsk::Mantle) {
                        (ledge, super::parkour::Hanging::mantle_spot(&ledge, &walker.ledges, state.locomotion.position, square, &stood, rig), false)
                    } else {
                        (ledge, super::parkour::Hanging::spot(&ledge, &walker.ledges, state.locomotion.position, square, &stood, rig), false)
                    });
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
        // On a beam (`parkour::beam`): its balance eased in, walking along its
        // line (back onto it if off) at no more than a beam's pace.
        let on_beam = if state.on_holds() || !state.posture.is_standing() || placing {
            None
        } else {
            walker.beams.iter().copied().find(|beam| beam.holds(state.locomotion.position))
        };
        // Stood on a small top it jumped onto (`parkour::precision`), the
        // beam's balance too, until it steps off it (judged standing: in the
        // fall onto it, the root still short of it, the balance was given up
        // before it landed).
        if !state.on_holds() && state.perched.is_some_and(|top| (state.locomotion.position - top).with_y(0.0).length() > PERCH_LEFT) {
            state.perched = None;
        }
        let perched = state.perched.is_some() && state.falling.as_ref().is_none_or(|falling| falling.is_landed());
        let toward_beam = if on_beam.is_some() || perched { 1.0 } else { 0.0 };
        let eased = time.delta_secs() / super::parkour::beam::BEAM_EASE;
        state.beam = (state.beam + (toward_beam - state.beam).clamp(-eased, eased)).clamp(0.0, 1.0);
        state.beam_time = if state.beam > 0.0 { state.beam_time + time.delta_secs() } else { 0.0 };
        if let (Some(beam), Some(rig)) = (on_beam, foot_ik.rig.as_ref()) {
            let ahead = approach::heading_of(rig.forward());
            let facing = Quat::from_rotation_y(state.facing.yaw) * rig.forward();
            let along = approach::heading_of(beam.way_for(facing)) - ahead;
            let (yaw, _) = back_to_line(state.locomotion.position, beam.a, along, rig.forward());
            if wanted_speed > 0.0 {
                steer = Steer::Toward { yaw, rate: BEAM_TURN };
            }
            wanted_speed = wanted_speed.min(super::parkour::beam::BEAM_SPEED);
        }
        // Perched (`parkour::perch`), it rises before it walks.
        if state.perch_weight > 0.0 {
            wanted_speed = 0.0;
        }
        // A wall ahead within its stopping distance (`keep_off_walls`): it
        // turns off its way as little as is clear, keeps to that side along
        // the wall, and back onto its way past its end (`way_round`); with
        // no way within a quarter turn, it stops. Stopped only once held at
        // it, it set off again each time it stood clear, and shuffled.
        // Not walking to a spot by a wall (a ledge's, a ladder's): that
        // approach ends nearer than this stops; nor on a circle; nor running
        // at an obstacle to vault it (it turned off its way round the
        // obstacle 2 m short). Running to vault, it runs at its pace for a
        // foot to come down at the best take-off.
        let vaulting = matches!(
            walker.hang,
            Some(
                super::parkour::hang::HangAsk::Vault(_)
                    | super::parkour::hang::HangAsk::WallRun
                    | super::parkour::hang::HangAsk::WallKick
                    | super::parkour::hang::HangAsk::RunAlong
                    | super::parkour::hang::HangAsk::SlideUnder
            )
        ) && walker.ledge.is_some()
            || walker.springboard.is_some()
            || walker.corner.is_some();
        if vaulting {
            wanted_speed *= state.vault_pace;
        } else {
            state.vault_pace = 1.0;
        }
        // Nor in a jump, which keeps its own way: hopping a low obstacle,
        // the ask dropped at take-off, it turned off round it in the air and
        // the hop curved 1.7 m aside.
        if !state.on_holds()
            && !placing
            && !vaulting
            && state.jump.is_none()
            && wanted_speed > 0.0
            && !matches!(steer, Steer::Circle(_))
            && let Some(ground) = ground
        {
            let speed = state.locomotion.root_velocity.length();
            let reach = BODY_RADIUS + WALL_STOP_MARGIN + WALL_STOP_TIME * speed;
            // Steered toward a facing, that is the way wanted, now; else the
            // facing it had when it turned off.
            let at = state.locomotion.position;
            let side = state.detour.map_or(0.0, |detour| detour.side);
            let wanted = match steer {
                Steer::Toward { yaw, .. } => yaw,
                _ => state.detour.map_or(state.facing.yaw, |detour| detour.wanted),
            };
            match way_round(at, wanted, side, reach, gait_rig.forward(), ground.0.as_ref()) {
                // Clear: back to its line (heading for it, if that way is
                // clear too, else along its way), the detour over once on
                // it and facing its way.
                WayRound::Clear => {
                    if let Some(detour) = state.detour {
                        let (back, off) = back_to_line(at, detour.from, wanted, gait_rig.forward());
                        let toward = match way_round(at, back, side, reach, gait_rig.forward(), ground.0.as_ref()) {
                            WayRound::Clear => back,
                            _ => wanted,
                        };
                        steer = Steer::Toward { yaw: toward, rate: DETOUR_RATE };
                        // Over: its way set as its facing to hold (left at
                        // the last heading for its line, a little off its
                        // way, it walked on off its line, 0.13 m in 13 m).
                        if off < LINE_ON && super::facing::shortest_angle(state.facing.yaw - wanted).abs() < LINE_FACING {
                            steer = Steer::Toward { yaw: wanted, rate: DETOUR_RATE };
                            state.detour = None;
                        }
                    }
                }
                WayRound::Turn { yaw, side } => {
                    steer = Steer::Toward { yaw, rate: DETOUR_RATE };
                    let from = state.detour.map_or(at, |detour| detour.from);
                    state.detour = Some(Detour { wanted, side, from });
                }
                WayRound::Blocked => wanted_speed = 0.0,
            }
        } else if placing || state.on_holds() {
            state.detour = None;
        }
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
        // Squeezing, the shuffle along the passage.
        let squeeze_side = match (state.squeezing, walker.squeeze, state.squeeze_facing, foot_ik.rig.as_ref()) {
            (true, Some(squeeze), Some(facing), Some(rig)) => Some(squeeze.side(facing, rig) * super::parkour::squeeze::SQUEEZE_SPEED),
            _ => None,
        };
        let side = if still || walker.sit.is_some() { 0.0 } else { squeeze_side.unwrap_or(walker.aside) };
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
        let pace_before = state.pace;
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
            // On a beam, the feet nearly on its line, the arms held out
            // rather than swung (swung under the lift, one reached forward
            // and the other hung back).
            let walk = GaitParams::walking_on(speed, &gait_rig);
            GaitParams { feet_apart: super::parkour::beam::feet_apart(state.beam), arm_swing: walk.arm_swing * (1.0 - state.beam), ..walk }
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
        // Balancing on a beam: the arms out, swaying against the trunk.
        if state.beam > 0.0 {
            let sway = super::parkour::beam::sway_at(state.beam_time);
            super::parkour::beam::balance(&mut target.pose, &gait_rig, super::gait::smoothstep(state.beam), sway);
        }
        // Squeezing: the arms held flat at the sides (`parkour::squeeze`).
        let flattened = time.delta_secs() / super::parkour::beam::BEAM_EASE;
        state.squeezed = (state.squeezed + if state.squeezing { flattened } else { -flattened }).clamp(0.0, 1.0);
        if state.squeezed > 0.0 {
            super::parkour::squeeze::flatten(&mut target.pose, &gait_rig, super::gait::smoothstep(state.squeezed));
        }
        // Stopped with its toes at a drop: a teeter, once a stop, the arms
        // windmilling, the feet planted (`parkour::teeter`).
        let standing_still = weight <= 0.0 && state.transition.is_at_rest() && state.posture.is_standing() && !state.on_holds() && !crouched && walker.sit.is_none();
        if !standing_still {
            state.teetered = false;
            state.teeter = None;
        } else if !state.teetered
            && let Some(rig) = foot_ik.rig.as_ref()
        {
            let forward = Quat::from_rotation_y(state.facing.yaw) * rig.forward();
            let ground_at = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
            if super::parkour::teeter::at_edge(state.locomotion.position, forward.with_y(0.0).normalize_or(Vec3::NEG_Z), &ground_at) {
                state.teetered = true;
                state.teeter = Some(0.0);
            }
        }
        if let Some(t) = state.teeter {
            super::parkour::teeter::teeter(&mut target.pose, &gait_rig, t);
            let t = t + time.delta_secs();
            state.teeter = (t < super::parkour::teeter::TEETER_TIME).then_some(t);
        }
        // Running, leant with its acceleration (`parkour::lean`): rolled into
        // a turn by its speed times its facing's turn rate (last frame's),
        // the trunk pitched forward gathering speed and back shedding it.
        {
            use super::parkour::lean;
            let dt = time.delta_secs();
            if dt > 0.0 {
                let turn_rate = facing::shortest_angle(state.facing.yaw - state.lean_yaw) / dt;
                let wanted = if weight >= 1.0 && !state.on_holds() && state.jump.is_none() {
                    lean::lean_for((state.pace - pace_before) / dt, lean::turning(speed, turn_rate)) * running
                } else {
                    Vec2::ZERO
                };
                state.lean.advance(wanted, dt);
            }
            state.lean_yaw = state.facing.yaw;
            if state.lean.now.length_squared() > 1.0e-10 {
                lean::lean(&mut target.pose, &gait_rig, state.lean.now);
            }
        }
        // Beside a wall, standing or walking, a hand rests on it
        // (`parkour::wallhand`), eased on and off; not with the arms busy
        // (a beam's balance, a squeeze, a teeter, a reach, crouched).
        {
            use super::parkour::wallhand;
            let free = running <= 0.0
                && !state.on_holds()
                && state.jump.is_none()
                && state.posture.is_standing()
                && !crouched
                && state.beam <= 0.0
                && state.squeezed <= 0.0
                && state.teeter.is_none()
                && walker.reach.is_none();
            let found = ground.filter(|_| free).and_then(|ground| {
                let blocks = |point: Vec3, low: f32| ground.0.blocks(point, low, low + 2.0);
                wallhand::beside(&target.pose, state.locomotion.position, state.facing.yaw, &gait_rig, &blocks)
            });
            let wanted = found.map_or(0.0, |wall| wall.weight());
            let most = time.delta_secs() / wallhand::EASE;
            state.wall_hand = (state.wall_hand + (wanted - state.wall_hand).clamp(-most, most)).clamp(0.0, 1.0);
            // A wall on the other side taken only once the hand is off.
            if found.is_some_and(|wall| state.wall_beside.is_none_or(|last| last.side == wall.side) || state.wall_hand <= 0.0) {
                state.wall_beside = found;
            }
            if state.wall_hand <= 0.0 {
                state.wall_beside = None;
            }
            if let Some(wall) = state.wall_beside {
                wallhand::rest_hand(&mut target.pose, state.locomotion.position, state.facing.yaw, &gait_rig, &wall, super::gait::smoothstep(state.wall_hand));
            }
        }
        // Asked to perch, standing still: crouched down into it, up again
        // when not asked, or asked to walk or jump onto a top
        // (`parkour::perch`).
        {
            use super::parkour::perch;
            let still = weight <= 0.0 && state.transition.is_at_rest() && state.posture.is_standing() && !state.on_holds() && state.jump.is_none() && !crouched;
            let toward = if walker.perch && walker.onto.is_none() && walker.speed <= 0.0 && still { 1.0 } else { 0.0 };
            let most = time.delta_secs() / perch::PERCH_EASE;
            state.perch_weight = (state.perch_weight + (toward - state.perch_weight).clamp(-most, most)).clamp(0.0, 1.0);
            if state.perch_weight > 0.0 {
                if foot_ik.rig.is_some() && state.perch_pose.is_none() {
                    state.perch_pose = Some(perch::perch_pose(&stood, &gait_rig));
                }
                if let Some(perched) = state.perch_pose.as_ref() {
                    perch::perch(&mut target.pose, perched, &gait_rig, super::gait::smoothstep(state.perch_weight));
                }
            }
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
        // Slid off a slope's edge this frame: its fall already moved on by
        // the rest of the frame (`parkour::slope`).
        let mut slid_off = false;
        let mut let_go = false;
        // Run up a wall from this frame's contact (below): posed from the
        // next.
        let mut wall_started = false;
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
                    // A low obstacle in its way: its knees tucked up over
                    // it (`Jump::tuck_over`), if that clears it.
                    use super::parkour::vault::{Obstacle, VaultKind};
                    let ahead = state.facing.rotation() * rig.forward();
                    let reach = ask.distance;
                    let tucked = walker
                        .ledge
                        .iter()
                        .chain(&walker.ledges)
                        .filter_map(|ledge| Obstacle::ahead(ledge, state.locomotion.position, ahead, VaultKind::Hop.most_slant()))
                        .filter(|obstacle| obstacle.top > 0.0 && obstacle.near + obstacle.depth < reach)
                        .min_by(|a, b| a.near.total_cmp(&b.near))
                        .and_then(|obstacle| super::jump::Jump::tuck_over(ask, obstacle, &stood, &rig));
                    state.jump = Some(tucked.unwrap_or_else(|| super::jump::Jump::plan(ask, &stood, &rig)));
                } else if running {
                    state.leap_asked = Some(ask);
                }
            }
            // A leap of faith (`parkour::faith`), standing: turned to face
            // the pile on the spot, then the leap.
            if let Some(hay) = walker.leap_of_faith
                && standing
                && state.jump.is_none()
                && state.faith.is_none()
                && state.perch_weight <= 0.0
            {
                let ahead = approach::heading_of(rig.forward());
                let toward = approach::heading_of((hay.top - state.locomotion.position).with_y(0.0).normalize_or(rig.forward())) - ahead;
                if facing::shortest_angle(toward - state.facing.yaw).abs() > 0.02 {
                    steer = Steer::Toward { yaw: toward, rate: 2.0 };
                } else {
                    state.faith = super::parkour::faith::LeapOfFaith::plan(state.locomotion.position, state.facing.yaw, hay, &stood, &rig);
                    walker.leap_of_faith = None;
                }
            }
            // A turning jump (`parkour::spin`), standing.
            if standing
                && state.jump.is_none()
                && state.perch_weight <= 0.0
                && let Some(turn) = walker.spin_jump.take()
            {
                state.jump = Some(super::parkour::spin::spin_jump(&stood, &rig));
                state.spinning = Some(turn);
            }
            // Asked onto a small top (`parkour::precision`), standing: turned
            // to face it on the spot, then a standing jump onto it.
            if let Some(top) = walker.onto
                && standing
                && state.jump.is_none()
                && state.perch_weight <= 0.0
            {
                use super::parkour::precision::{jump_onto, FACING};
                let ahead = approach::heading_of(rig.forward());
                let toward = approach::heading_of((top - state.locomotion.position).with_y(0.0).normalize_or(rig.forward())) - ahead;
                if facing::shortest_angle(toward - state.facing.yaw).abs() > 0.5 * FACING {
                    steer = Steer::Toward { yaw: toward, rate: 2.0 };
                } else {
                    if let Some(jump) = jump_onto(top, state.locomotion.position, state.facing.yaw, &stood, &rig) {
                        state.jump = Some(jump);
                        state.onto = Some(top);
                    }
                    walker.onto = None;
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
            // Vaulting (`parkour::vault`): running at a low obstacle, at each
            // foot's contact it takes off if the next would come down no
            // nearer its best take-off; come down past that, the ask is
            // dropped.
            if let Some(super::parkour::hang::HangAsk::Vault(kind)) = walker.hang
                && let Some(ledge) = walker.ledge
                && running
                && state.jump.is_none()
                && rate > 0.0
            {
                use super::parkour::vault::{Obstacle, TAKEOFF_SLACK};
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    // Where the root is at the contact, the jump's frame there.
                    let origin = state.locomotion.position + state.facing.rotation() * (state.stride.stepped + rig.forward() * run_part);
                    let start = super::jump::RunStart { leg, speed };
                    if let Some(obstacle) = Obstacle::ahead(&ledge, origin, state.facing.rotation() * rig.forward(), kind.most_slant()) {
                        let best = if kind == super::parkour::vault::VaultKind::Hop {
                            super::jump::Jump::hop_takeoff(obstacle.depth, start, &stood, &rig)
                        } else {
                            super::jump::Jump::vault_takeoff(obstacle.depth, obstacle.top, start, &stood, &rig)
                        };
                        let step = speed / (2.0 * rate);
                        let aim = best + VAULT_AIM;
                        let next = obstacle.near - step;
                        // This foot if its vault plans and the next would be
                        // past the best or no nearer the aim.
                        let vault = (obstacle.near >= best && obstacle.near - best <= step + TAKEOFF_SLACK)
                            .then(|| super::jump::Jump::vault(obstacle, start, kind, &stood, &rig))
                            .flatten()
                            .filter(|_| next < best || (obstacle.near - aim).abs() <= (next - aim).abs());
                        if let Some(mut jump) = vault {
                            jump.advance(since);
                            state.stride.stepped += rig.forward() * (run_part + jump.travelled());
                            state.jump = Some(jump);
                            started = true;
                            walker.hang = None;
                        } else if obstacle.near < best {
                            // Past its take-off: no vault.
                            walker.hang = None;
                        } else {
                            // A whole number of steps on to the aim: the run's
                            // pace stretched or shortened to them.
                            let to_go = obstacle.near - aim;
                            let mut steps = (to_go / step).round().max(1.0);
                            // The foot it must take off from (a lazy vault at
                            // a slant) there: one step more or fewer if not
                            // (the foot `steps` on is this one if even).
                            if let Some(wanted) = kind.takeoff_leg(obstacle.slant) {
                                let lands = if (steps as usize).is_multiple_of(2) { leg } else { 1 - leg };
                                if lands != wanted {
                                    steps = if to_go / step > steps || steps <= 1.0 { steps + 1.0 } else { steps - 1.0 };
                                }
                            }
                            state.vault_pace = (state.vault_pace * to_go / (steps * step)).clamp(1.0 - VAULT_PACE, 1.0 + VAULT_PACE);
                        }
                    }
                }
            }
            // A springboard (`parkour::springboard`): running at it, at each
            // foot's contact it leaps off it if the take-off ankle came down
            // on its end and the next would come down no nearer; otherwise
            // the pace is stretched or shortened to bring one down there.
            // Off its line, or come down past it, the ask is dropped.
            if let Some(board) = walker.springboard
                && running
                && state.jump.is_none()
                && rate > 0.0
            {
                use super::parkour::springboard::{spring_leap, takeoff_ahead, ON_BOARD};
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    let origin = state.locomotion.position + state.facing.rotation() * (state.stride.stepped + rig.forward() * run_part);
                    let way = state.facing.rotation() * rig.forward();
                    let start = super::jump::RunStart { leg, speed };
                    let ahead = takeoff_ahead(&board, start, &stood, &rig);
                    // How far past where this foot came down its end is.
                    let (short, across) = board.off(origin + way * ahead);
                    let to_go = -short;
                    let step = speed / (2.0 * rate);
                    if across.abs() > ON_BOARD || to_go < -ON_BOARD {
                        walker.springboard = None;
                    } else if to_go.abs() <= ON_BOARD && (to_go - step < -ON_BOARD || to_go.abs() <= (to_go - step).abs()) {
                        let mut jump = spring_leap(&board, start, &stood, &rig);
                        jump.advance(since);
                        state.stride.stepped += rig.forward() * (run_part + jump.travelled());
                        state.jump = Some(jump);
                        state.springing = Some(board);
                        started = true;
                        walker.springboard = None;
                    } else {
                        let steps = (to_go / step).round().max(1.0);
                        state.vault_pace = (state.vault_pace * to_go / (steps * step)).clamp(1.0 - VAULT_PACE, 1.0 + VAULT_PACE);
                    }
                }
            }
            // A corner post (`parkour::corner`): running past it, at each
            // foot's contact it leaps round it if the flight would begin
            // with the post beside the hips and the next would not be
            // nearer; otherwise the pace is stretched or shortened to bring
            // a foot down there. Off its side or run past, the ask is
            // dropped.
            if let Some((post, turn)) = walker.corner
                && running
                && state.jump.is_none()
                && rate > 0.0
            {
                use super::parkour::corner::{plan, takeoff_ahead, ABEAM, FARTHEST, NEAREST};
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    let origin = state.locomotion.position + state.facing.rotation() * (state.stride.stepped + rig.forward() * run_part);
                    let (way, left) = (state.facing.rotation() * rig.forward(), state.facing.rotation() * rig.left());
                    let to = (post.foot - origin).with_y(0.0);
                    let (along, aside) = (to.dot(way), to.dot(left));
                    let start = super::jump::RunStart { leg, speed };
                    let leaves = takeoff_ahead(start, turn, aside, &stood, &rig);
                    let to_go = along - leaves;
                    let step = speed / (2.0 * rate);
                    if aside.signum() != turn.signum() || !(NEAREST - 0.1..=FARTHEST + 0.1).contains(&aside.abs()) || to_go < -ABEAM {
                        walker.corner = None;
                    } else if to_go.abs() <= ABEAM && (to_go - step < -ABEAM || to_go.abs() <= (to_go - step).abs()) {
                        if let Some((mut jump, mut swing)) = plan(&post, origin, state.facing.yaw, start, turn, &stood, &rig) {
                            if let Some(hands) = hands.as_ref() {
                                swing.set_grips(hands.grips, &rig);
                            }
                            jump.advance(since);
                            state.stride.stepped += rig.forward() * (run_part + jump.travelled());
                            state.jump = Some(jump);
                            state.cornering = Some(swing);
                            started = true;
                        }
                        walker.corner = None;
                    } else {
                        let steps = (to_go / step).round().max(1.0);
                        state.vault_pace = (state.vault_pace * to_go / (steps * step)).clamp(1.0 - VAULT_PACE, 1.0 + VAULT_PACE);
                    }
                }
            }
            // Sliding under a slab (`parkour::underslide`): off the foot that
            // comes down at its start (the latest that clears it), the pace
            // adjusted for one to.
            if walker.hang == Some(super::parkour::hang::HangAsk::SlideUnder)
                && let Some(slab) = walker.ledge
                && running
                && state.jump.is_none()
                && state.under_slide.is_none()
                && rate > 0.0
            {
                use super::parkour::underslide::UnderSlide;
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down
                    && let Some(obstacle) = super::parkour::vault::Obstacle::ahead(&slab, state.locomotion.position, state.facing.rotation() * rig.forward(), 0.4)
                {
                    let step = speed / (2.0 * rate);
                    match UnderSlide::start_distance(speed, obstacle.depth) {
                        Some(best) if obstacle.near <= best => {
                            if let Some(slide) = UnderSlide::plan(&slab, state.locomotion.position, state.facing.yaw, speed, leg, &target.pose, &stood, &rig) {
                                state.under_slide = Some(slide);
                                state.stride.stepped = Vec3::ZERO;
                                started = true;
                            }
                            walker.hang = None;
                        }
                        Some(best) => {
                            let to_go = obstacle.near - best;
                            let steps = (to_go / step).round().max(1.0);
                            state.vault_pace = (state.vault_pace * to_go / (steps * step)).clamp(1.0 - VAULT_PACE, 1.0 + VAULT_PACE);
                        }
                        // Too slow yet (still speeding up: at its first
                        // footfall a 5 m/s run was at 2.7 and the ask was
                        // dropped, and it ran round the slab): waits, unless
                        // the slab is already too near to start.
                        None if obstacle.near < SLIDE_TOO_NEAR => walker.hang = None,
                        None => {}
                    }
                }
            }
            // Skidding (`parkour::skid`), if it skids: running fast, asked to
            // stop or to face back, off the foot that comes down.
            let asked_back = matches!(walker.steer, Steer::Toward { yaw, .. } if facing::shortest_angle(yaw - state.facing.yaw).abs() > SKID_BACK);
            if walker.skid && running && state.jump.is_none() && state.skid.is_none() && !state.on_holds() && rate > 0.0 && (walker.speed <= 0.0 || asked_back) {
                // Not turned toward the way back before its foot comes down:
                // turned first, it skidded on along the turned way, 1.7 m
                // aside of where it ran.
                if asked_back && speed >= super::parkour::skid::SKID_FROM {
                    steer = Steer::Straight;
                }
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down
                    && let Some(skid) = super::parkour::skid::Skid::plan(state.locomotion.position, state.facing.yaw, speed, leg, asked_back, &target.pose, &stood, &rig)
                {
                    state.skid = Some(skid);
                    state.stride.stepped = Vec3::ZERO;
                    started = true;
                }
            }
            // A slope too steep to walk (`parkour::slope`): walked or run onto
            // it down it, near its top edge, it slides from this frame's pose.
            if !walker.slopes.is_empty() && state.jump.is_none() && state.skid.is_none() && !state.on_holds() {
                let velocity = state.locomotion.root_velocity.with_y(0.0);
                let root = state.locomotion.position + state.facing.rotation() * state.stride.stepped;
                let sample = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
                // Asked to catch, braking to come to a drop's edge slowly
                // enough to catch its eave.
                if let Some(slide) = walker
                    .slopes
                    .iter()
                    .find_map(|slope| super::parkour::slope::SlopeSlide::begin(slope, root, state.facing.yaw, velocity, &target.pose, &sample, &stood, &rig))
                    .map(|slide| if walker.catch { slide.braking_to_catch() } else { slide })
                {
                    state.locomotion.position = root;
                    state.stride.stepped = Vec3::ZERO;
                    state.sloping = Some(slide);
                    started = true;
                }
            }
            // Running along a wall (`parkour::along`): beside it, at the first
            // contact of the foot farther from it whose run along plans, a
            // leap held up by two steps on its face, landing and running on
            // as a vault does; past the wall's far end, the ask is dropped.
            if walker.hang == Some(super::parkour::hang::HangAsk::RunAlong)
                && let Some(ledge) = walker.ledge
                && running
                && state.jump.is_none()
                && rate > 0.0
            {
                use super::parkour::along::AlongWall;
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    let origin = state.locomotion.position + state.facing.rotation() * (state.stride.stepped + rig.forward() * run_part);
                    let forward = state.facing.rotation() * rig.forward();
                    let left_to_run = (ledge.a - origin).dot(forward).max((ledge.b - origin).dot(forward));
                    let along = (leg == AlongWall::takeoff_leg(&ledge, state.facing.yaw, &rig))
                        .then(|| super::jump::Jump::along_wall(&ledge, origin, state.facing.yaw, super::jump::RunStart { leg, speed }, &stood, &rig))
                        .flatten();
                    if let Some(mut jump) = along {
                        jump.advance(since);
                        state.stride.stepped += rig.forward() * (run_part + jump.travelled());
                        state.jump = Some(jump);
                        started = true;
                        walker.hang = None;
                    } else if left_to_run < RUN_ALONG_LEFT {
                        walker.hang = None;
                    }
                }
            }
            // Running up a wall (`parkour::wall`): as a vault is taken, from
            // the foot that comes down nearest its best take-off, the pace
            // adjusted over the last steps; past it, the ask is dropped.
            if walker.hang == Some(super::parkour::hang::HangAsk::WallRun)
                && let Some(ledge) = walker.ledge
                && running
                && state.jump.is_none()
                && state.wall_run.is_none()
                && rate > 0.0
            {
                use super::parkour::wall::{WallRun, MOST_SLANT};
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    let origin = state.locomotion.position + state.facing.rotation() * (state.stride.stepped + rig.forward() * run_part);
                    let start = super::jump::RunStart { leg, speed };
                    if let Some(face) = super::parkour::vault::Obstacle::ahead(&ledge, origin, state.facing.rotation() * rig.forward(), MOST_SLANT) {
                        let best = WallRun::takeoff(start, face.slant, &stood, &rig);
                        let step = speed / (2.0 * rate);
                        let next = face.near - step;
                        let run = (face.near >= best - WALL_TAKEOFF_NEAR)
                            .then(|| WallRun::plan(&ledge, origin, state.facing.yaw, start, foot_ik.pelvis_drop, &stood, &rig))
                            .flatten()
                            .filter(|_| next < best - WALL_TAKEOFF_NEAR || (face.near - best).abs() <= (next - best).abs());
                        if let Some(mut run) = run {
                            run.advance(since);
                            state.stride.stepped = Vec3::ZERO;
                            state.wall_run = Some(run);
                            wall_started = true;
                            let_go = true;
                            walker.hang = None;
                        } else if face.near < best - WALL_TAKEOFF_NEAR {
                            walker.hang = None;
                        } else {
                            let to_go = face.near - best;
                            let steps = (to_go / step).round().max(1.0);
                            state.vault_pace = (state.vault_pace * to_go / (steps * step)).clamp(1.0 - VAULT_PACE, 1.0 + VAULT_PACE);
                        }
                    }
                }
            }
            // Kicking off a wall toward a lip (`parkour::wall`, a tic-tac):
            // the wall the nearest ahead among the others, taken as a run up
            // is, but only off the foot farther from it, the pace adjusted
            // for that foot to come down at the best take-off.
            if walker.hang == Some(super::parkour::hang::HangAsk::WallKick)
                && let Some(target) = walker.ledge
                && running
                && state.jump.is_none()
                && state.wall_run.is_none()
                && rate > 0.0
            {
                use super::parkour::wall::{WallRun, KICK_TAKEOFF};
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                if let Some(leg) = came_down {
                    let since = (cycle - 0.5 * leg as f32).rem_euclid(1.0) / rate;
                    let run_part = (time.delta_secs() - since).max(0.0) * speed;
                    let origin = state.locomotion.position + state.facing.rotation() * (state.stride.stepped + rig.forward() * run_part);
                    let start = super::jump::RunStart { leg, speed };
                    if let Some((wall, face)) = WallRun::kick_off(&walker.ledges, &target, origin, state.facing.rotation() * rig.forward()) {
                        let best = WallRun::kick_takeoff(start, face.slant, &stood, &rig);
                        let step = speed / (2.0 * rate);
                        let wanted = WallRun::kick_leg(&wall, state.facing.yaw, &stood, &rig);
                        // Taken off within the kick's window (`KICK_TAKEOFF`),
                        // aimed at its middle; this foot's next contact is two
                        // steps on.
                        let (nearer, farther) = KICK_TAKEOFF;
                        let within = |near: f32| (best - nearer..=best + farther).contains(&near);
                        let aim = best + 0.5 * (farther - nearer);
                        let next = face.near - 2.0 * step;
                        // Straight to the lip; out of one kick's reach, across
                        // to the wall facing this one, to kick off that in
                        // turn (chaining).
                        let across = WallRun::across_from(&walker.ledges, &wall, &target);
                        let mut chained = None;
                        let kick = (leg == wanted && within(face.near))
                            .then(|| {
                                WallRun::kick(&wall, &target, origin, state.facing.yaw, start, foot_ik.pelvis_drop, &stood, &rig).or_else(|| {
                                    let other = across?;
                                    chained = Some((other, target));
                                    WallRun::kick_across(&wall, &other, origin, state.facing.yaw, start, foot_ik.pelvis_drop, &stood, &rig)
                                })
                            })
                            .flatten()
                            .filter(|_| !within(next) || (face.near - aim).abs() <= (next - aim).abs());
                        if kick.is_some() {
                            state.kick_chain = chained;
                        }
                        if let Some(mut run) = kick {
                            run.advance(since);
                            state.stride.stepped = Vec3::ZERO;
                            state.wall_run = Some(run);
                            wall_started = true;
                            let_go = true;
                            walker.hang = None;
                        } else if face.near < best - nearer {
                            walker.hang = None;
                        } else {
                            // A whole number of steps on to the aim, the foot
                            // landing there the one farther from the wall.
                            let to_go = face.near - aim;
                            let mut steps = (to_go / step).round().max(1.0);
                            let lands = if (steps as usize).is_multiple_of(2) { leg } else { 1 - leg };
                            if lands != wanted {
                                steps = if to_go / step > steps || steps <= 1.0 { steps + 1.0 } else { steps - 1.0 };
                            }
                            state.vault_pace = (state.vault_pace * to_go / (steps * step)).clamp(1.0 - VAULT_PACE, 1.0 + VAULT_PACE);
                        }
                    }
                }
            }
            // Mantling straight from a walk (`Hanging::mantle`): walking in to
            // its spot, square to the wall, the first foot down within
            // `MANTLE_FROM_WALK` of the spot stays, and it mantles from there
            // without stopping, the other foot stepping in.
            if walker.hang == Some(super::parkour::hang::HangAsk::Mantle)
                && state.hanging.is_none()
                && state.jump.is_none()
                && let Some((ledge, spot, true)) = state.ledge_spot
                && !running
                && speed > MANTLE_WALKING
                && rate > 0.0
            {
                let came_down = (0..2).find(|&leg| {
                    let contact = 0.5 * leg as f32;
                    (cycle - contact).rem_euclid(1.0) < (state.stride.cycle - contact).rem_euclid(1.0)
                });
                let square = super::parkour::Hanging::square(&ledge, rig.forward());
                let at = state.locomotion.position + state.facing.rotation() * state.stride.stepped;
                let near = Vec3::new(spot.x - at.x, 0.0, spot.z - at.z).length() <= MANTLE_FROM_WALK;
                let squared = super::facing::shortest_angle(state.facing.yaw - square).abs() < MANTLE_SQUARE;
                if let Some(leg) = came_down
                    && near
                    && squared
                {
                    let from = super::parkour::hang::FromWalk { at, pose: target.pose, velocity: state.facing.rotation() * rig.forward() * speed, planted: leg };
                    let grips = hands.as_ref().map_or([None; 2], |hands| hands.grips);
                    if let Some(hanging) = super::parkour::Hanging::mantle(&ledge, &walker.ledges, spot, square, foot_ik.pelvis_drop, grips, Some(from), &stood, &rig) {
                        state.hanging = Some(hanging);
                        state.approach = approach::Approach::Idle;
                        state.ledge_spot = None;
                        walker.hang = None;
                    }
                }
            }
            let standing_pose = target.pose;
            if let Some(jump) = state.jump.as_mut() {
                // The COM's way forward moves the character, like root
                // motion; the pose keeps it over the root.
                if !started {
                    let (before, was) = (jump.travelled(), jump.elapsed());
                    jump.advance(time.delta_secs());
                    // Round a corner post (`parkour::corner`): the facing
                    // turned through the flight, the way it goes bent round
                    // the post with it.
                    if let Some(swing) = state.cornering {
                        let turned = swing.turned_at(jump, jump.elapsed()) - swing.turned_at(jump, was);
                        state.facing.yaw += turned;
                        state.facing.target_yaw += turned;
                    }
                    state.stride.stepped += rig.forward() * (jump.travelled() - before);
                }
                // Each bone led ahead of its spring, so the body rendered
                // is the plan's.
                target.pose = match springs {
                    Some(springs) => jump.pose_now_led(&stood, &rig, &springs.0),
                    None => jump.pose(&stood, &rig),
                };
                // Its near hand on the post, the body banked toward it.
                if let Some(swing) = state.cornering {
                    let root = state.locomotion.position + state.facing.rotation() * state.stride.stepped;
                    target.pose = swing.hold(&target.pose, jump, root, state.facing.yaw, &rig);
                    if let Some(hands) = hands.as_mut() {
                        let mut grips = hands.grip;
                        grips[swing.side] = swing.holding(jump);
                        if hands.grip != grips {
                            hands.grip = grips;
                        }
                    }
                    if swing.is_done(jump) {
                        state.cornering = None;
                    }
                }
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
                // Off a springboard, the take-off foot sinks with the board
                // under the ground the foot IK holds a foot on: the plan's
                // legs as posed (held, the ankle rose as the plain leap's).
                legs_free = state.springing.is_some();
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
        // Free climbing (`parkour::holds`): got on once stopped in front of
        // the wall; posed on its holds, the root riding its hips; stepped off
        // at the bottom, a fall; topped out, a hang on the lip, climbing up
        // if it was climbing up.
        if let Some(rig) = foot_ik.rig.clone() {
            let ready = at_holds && weight <= 0.0 && state.transition.is_at_rest() && state.crouching.is_standing() && state.jump.is_none();
            if ready
                && state.free_climbing.is_none()
                && let Some(wall) = walker.holds.as_ref()
            {
                match super::parkour::holds::FreeClimb::get_on(wall, state.locomotion.position, &stood, &rig) {
                    Some(mut climb) => {
                        if let Some(hands) = hands.as_ref() {
                            climb.set_grips(hands.grips);
                        }
                        state.free_climbing = Some(climb);
                    }
                    None => walker.free_climb = None,
                }
                state.holds_spot = None;
                state.approach = approach::Approach::Idle;
            }
            if let Some(climb) = state.free_climbing.as_mut() {
                let way = walker.free_climb.filter(|way| way.length_squared() > 1.0e-6);
                climb.advance(way, dt);
                target.pose = match springs {
                    Some(springs) => climb.pose_led(&springs.0),
                    None => climb.pose(),
                };
                state.locomotion.position = climb.root();
                state.facing.yaw = climb.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if let Some(hands) = hands.as_mut() {
                    let grips = climb.grips();
                    if hands.grip != grips || hands.hook != [true; 2] {
                        hands.grip = grips;
                        hands.hook = [true; 2];
                    }
                }
                if climb.stepped_off() {
                    state.falling = Some(climb.step_off(&stood));
                    state.free_climbing = None;
                    walker.free_climb = None;
                } else if climb.topped_out() {
                    let up = way.is_some_and(|way| way.y > 0.0);
                    state.hanging = climb.top_out(&walker.ledges);
                    state.free_climbing = None;
                    walker.free_climb = None;
                    if up {
                        walker.hang = Some(super::parkour::hang::HangAsk::ClimbUp);
                    }
                }
            }
        }
        // On a pole (`parkour::pole`): its pose instead, the root riding its
        // hips, facing it; got on once stopped on the spot in front of it;
        // let go, into a fall.
        if let Some(rig) = foot_ik.rig.clone() {
            let ready = at_pole && weight <= 0.0 && state.transition.is_at_rest() && state.crouching.is_standing() && state.jump.is_none();
            if ready
                && state.poling.is_none()
                && let Some((pole, _)) = state.pole_spot
            {
                let mut poling = super::parkour::Poling::get_on(&pole, state.locomotion.position, &stood, &rig);
                if let Some(hands) = hands.as_ref() {
                    poling.set_grips(hands.grips);
                }
                state.poling = Some(poling);
                state.approach = approach::Approach::Idle;
            }
            if let Some(poling) = state.poling.as_mut() {
                poling.advance(walker.on_pole, dt);
                target.pose = match springs {
                    Some(springs) => poling.pose_led(&rig, &springs.0),
                    None => poling.pose(&rig),
                };
                state.locomotion.position = poling.root();
                state.facing.yaw = poling.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if look_at.is_none() {
                    look_at = Some(poling.look());
                }
                if let Some(hands) = hands.as_mut() {
                    let grips = poling.grips();
                    if hands.grip != grips || hands.hook != [false; 2] {
                        hands.grip = grips;
                        hands.hook = [false; 2];
                    }
                }
                // Let go (pushed off, at the bottom, or slid to the floor):
                // a fall, reaching to catch a ledge if asked.
                if poling.is_released() {
                    let mut falling = poling.release(&|at| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height), &stood, &rig);
                    falling.reach(walker.catch);
                    state.falling = Some(falling);
                    state.poling = None;
                    state.pole_spot = None;
                    walker.on_pole = None;
                }
            }
        }
        // Round a flagpole (`parkour::flagpole`): its pose instead, the root
        // riding its hips; let go, a fall flung on as it swung, landing on
        // the first top along its flight.
        if let Some(rig) = foot_ik.rig.clone()
            && let Some(swinging) = state.flagging.as_mut()
        {
            swinging.advance(dt);
            target.pose = match springs {
                Some(springs) => swinging.pose_led(&rig, &springs.0),
                None => swinging.pose(&rig),
            };
            state.locomotion.position = swinging.root();
            state.facing.yaw = swinging.facing();
            state.facing.target_yaw = state.facing.yaw;
            foot_ik.planted = [false; 2];
            foot_ik.landing = None;
            foot_ik.touchdown = None;
            foot_ik.clear = [0.0; 2];
            foot_ik.gait_swing = None;
            foot_ik.gait_bearing = None;
            legs_free = true;
            if look_at.is_none() {
                look_at = Some(swinging.look());
            }
            if let Some(hands) = hands.as_mut() {
                let grips = swinging.grips();
                if hands.grip != grips || hands.hook != [false; 2] {
                    hands.grip = grips;
                    hands.hook = [false; 2];
                }
            }
            if swinging.is_released() {
                let top = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
                let floor = top(swinging.root().with_y(swinging.root().y + 0.05)).unwrap_or(0.0);
                let mut falling = swinging.release(&top, floor, &stood, &rig);
                let ledges: Vec<super::parkour::Ledge> = walker.ledge.into_iter().chain(walker.ledges.iter().copied()).collect();
                falling.against(&ledges, &rig);
                state.falling = Some(falling);
                // A flagpole's ask is done; hooks stay asked.
                if swinging.holding() == [true; 2] {
                    walker.flagpole = None;
                }
                state.flagging = None;
            }
        }
        // Across monkey bars (`parkour::monkey`): got on once stopped under
        // the first, its pose instead, the root riding its hips; let go at
        // the far end, into a fall.
        if let Some(rig) = foot_ik.rig.clone() {
            let ready = at_monkey && weight <= 0.0 && state.transition.is_at_rest() && state.crouching.is_standing() && state.jump.is_none();
            if ready
                && state.monkey.is_none()
                && let Some((bars, _)) = state.monkey_spot
            {
                let mut crossing = super::parkour::monkey::Crossing::get_on(&bars, state.locomotion.position, &stood, &rig);
                if let Some(hands) = hands.as_ref() {
                    crossing.set_grips(hands.grips, &rig);
                }
                state.monkey = Some(crossing);
                state.approach = approach::Approach::Idle;
            }
            if let Some(crossing) = state.monkey.as_mut() {
                crossing.advance(dt);
                target.pose = match springs {
                    Some(springs) => crossing.pose_led(&rig, &springs.0),
                    None => crossing.pose(&rig),
                };
                state.locomotion.position = crossing.root();
                state.facing.yaw = crossing.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if look_at.is_none() {
                    look_at = Some(crossing.look());
                }
                if let Some(hands) = hands.as_mut() {
                    let grips = crossing.grips();
                    if hands.grip != grips || hands.hook != [false; 2] {
                        hands.grip = grips;
                        hands.hook = [false; 2];
                    }
                }
                if crossing.is_released() {
                    let mut falling = crossing.release(&|at| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height), &stood, &rig);
                    falling.reach(walker.catch);
                    state.falling = Some(falling);
                    state.monkey = None;
                    state.monkey_spot = None;
                    walker.monkey_bars = None;
                }
            }
        }
        // Let go to leap this frame (below), the flight starts from this
        // frame's pose.
        let mut leapt = false;
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
                // Dropping down: the hang below, lowered into from where it
                // stands on the top.
                if walker.hang == Some(super::parkour::hang::HangAsk::DropDown) {
                    let grips = hands.as_ref().map_or([None; 2], |hands| hands.grips);
                    let mut hanging = super::parkour::Hanging::hung(&ledge, &walker.ledges, state.locomotion.position, foot_ik.pelvis_drop, grips, &stood, &rig);
                    hanging.lower_down(state.locomotion.position);
                    state.hanging = Some(hanging);
                    walker.hang = None;
                } else if walker.hang == Some(super::parkour::hang::HangAsk::Mantle) {
                    // Mantling: the hands onto the top from where it stands;
                    // too low, too high or too shallow, the ask is dropped.
                    let grips = hands.as_ref().map_or([None; 2], |hands| hands.grips);
                    state.hanging = super::parkour::Hanging::mantle(&ledge, &walker.ledges, state.locomotion.position, square, foot_ik.pelvis_drop, grips, None, &stood, &rig);
                    walker.hang = None;
                } else {
                match super::parkour::Hanging::grab(&ledge, state.locomotion.position, square, foot_ik.pelvis_drop, &stood, &rig) {
                    Some(mut hanging) => {
                        // Each hand placed so its own fingers hook over the lip.
                        if let Some(hands) = hands.as_ref() {
                            hanging.set_grips(hands.grips, &stood, &rig);
                        }
                        hanging.set_others(&walker.ledges);
                        state.hanging = Some(hanging);
                    }
                    None => walker.hang = None,
                }
                }
                state.approach = approach::Approach::Idle;
                state.ledge_spot = None;
            }
            // Letting go (`parkour::fall`): hanging, not climbing up nor
            // mid-step, it falls from where it hangs to the ground below,
            // pushing off a wall braced, reaching up if asked to catch.
            if matches!(walker.hang, Some(super::parkour::hang::HangAsk::LetGo | super::parkour::hang::HangAsk::SlideDown))
                && let Some(hanging) = state.hanging.as_ref()
                && hanging.is_hanging()
                && !hanging.is_climbing_up()
                && !hanging.is_shimmying()
            {
                // Sliding down the wall, if it can (braced, the wall reaching
                // the ground); else letting go.
                let ground_at = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
                let slid = (walker.hang == Some(super::parkour::hang::HangAsk::SlideDown)).then(|| hanging.slide_down(&ground_at, &stood, &rig)).flatten();
                let mut falling = slid.unwrap_or_else(|| hanging.let_go(&ground_at, &stood, &rig));
                if !falling.is_sliding() {
                    falling.reach(walker.catch);
                }
                state.falling = Some(falling);
                state.hanging = None;
                walker.hang = None;
            }
            if let Some(hanging) = state.hanging.as_mut() {
                // Climbing up is taken once it hangs (asked while it still
                // walks there or jumps, it grabs first); a top with no room
                // to stand drops the ask.
                if walker.hang == Some(super::parkour::hang::HangAsk::ClimbUp) && hanging.is_hanging() {
                    hanging.climb_up();
                    walker.hang = None;
                }
                // A leap is taken once it just hangs (not climbing up nor
                // mid-step); with nothing to leap at, the ask is dropped.
                if let Some(super::parkour::hang::HangAsk::Leap(way)) = walker.hang
                    && hanging.is_hanging()
                    && !hanging.is_climbing_up()
                    && !hanging.is_shimmying()
                {
                    hanging.leap(way, &rig);
                    walker.hang = None;
                }
                // Shimmying while asked; the step under way finishes.
                hanging.shimmy(match walker.hang {
                    Some(super::parkour::hang::HangAsk::Shimmy(way)) => Some(way),
                    _ => None,
                });
                // Swinging free: pumped while asked; a lache pumps until
                // the swing comes round to let go (then flies as a leap
                // does); braced, dropped.
                match walker.hang {
                    Some(super::parkour::hang::HangAsk::Swing) => hanging.pump(true),
                    Some(super::parkour::hang::HangAsk::Lache) if hanging.is_hanging() && hanging.is_braced() => walker.hang = None,
                    Some(super::parkour::hang::HangAsk::Lache) => {
                        if hanging.is_hanging() && !hanging.is_climbing_up() && !hanging.is_shimmying() && hanging.lache(&rig) {
                            walker.hang = None;
                        }
                    }
                    _ => hanging.pump(false),
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
            // Leaping, let go: in the air (`parkour::fall`), aimed at the
            // ledge leapt at.
            if let Some(hanging) = state.hanging.as_ref().filter(|hanging| hanging.is_released()) {
                let ledges: Vec<super::parkour::Ledge> = walker.ledge.into_iter().chain(walker.ledges.iter().copied()).collect();
                let falling = hanging.release(&|at| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height), &ledges, &stood, &rig);
                state.falling = Some(falling);
                state.hanging = None;
                leapt = true;
            }
            // Running up a wall (`parkour::wall`): posed as planned, the root
            // riding its hips; let go of the wall, in the air aimed at its
            // lip.
            if let Some(run) = state.wall_run.as_mut() {
                if !wall_started {
                    run.advance(dt);
                }
                target.pose = match springs {
                    Some(springs) => run.pose_led(&springs.0),
                    None => run.pose(),
                };
                state.locomotion.position = run.root();
                state.facing.yaw = run.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if look_at.is_none() {
                    look_at = Some(run.wall().nearest(run.root(), 0.0));
                }
                if run.is_released() {
                    let falling = run.release(&|at| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height));
                    state.falling = Some(falling);
                    state.wall_run = None;
                    leapt = true;
                }
            }
            // Crawling (`parkour::crawl`): got down once standing still, posed
            // in the walker's frame, the root carried along its facing; stood
            // up again, it walks on.
            let standing_still = weight <= 0.0 && state.transition.is_at_rest() && state.posture.is_standing() && !state.on_holds() && walker.sit.is_none();
            if walker.crawl && standing_still && state.crawling.is_none() {
                state.crawling = Some(super::parkour::crawl::Crawling::new(state.locomotion.position, state.facing.yaw, &stood, &rig));
            }
            if let Some(crawl) = state.crawling.as_mut() {
                crawl.advance(walker.crawl, dt);
                target.pose = crawl.pose();
                state.locomotion.position = crawl.root();
                state.facing.yaw = crawl.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if crawl.is_done() {
                    state.crawling = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                }
            }
            // Sliding under a slab (`parkour::underslide`): posed as planned,
            // the root riding its hips; stood up past it, it walks on.
            if let Some(slide) = state.under_slide.as_mut() {
                slide.advance(dt);
                target.pose = match springs {
                    Some(springs) => slide.pose_led(&springs.0),
                    None => slide.pose(),
                };
                state.locomotion.position = slide.root();
                state.facing.yaw = slide.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if slide.is_done() {
                    state.under_slide = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                }
            }
            // A leap of faith (`parkour::faith`): posed as planned, the root
            // riding its centre of mass; risen out of the pile, it stands.
            if let Some(faith) = state.faith.as_mut() {
                faith.advance(dt);
                target.pose = match springs {
                    Some(springs) => faith.pose_led(&springs.0),
                    None => faith.pose(),
                };
                state.locomotion.position = faith.root();
                state.facing.yaw = faith.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if faith.is_done() {
                    state.faith = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                    let_go = true;
                }
            }
            // Skidding (`parkour::skid`): posed as planned, the root riding
            // its hips; stood, it stands (and runs on from a stand if asked).
            if let Some(skid) = state.skid.as_mut() {
                skid.advance(dt);
                target.pose = match springs {
                    Some(springs) => skid.pose_led(&springs.0),
                    None => skid.pose(),
                };
                state.locomotion.position = skid.root();
                state.facing.yaw = skid.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if skid.is_done() {
                    state.skid = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                    let_go = true;
                }
            }
            // Sliding down a slope (`parkour::slope`): posed as planned, the
            // root riding its hips. Run out, it stands. At a drop it goes
            // over the edge as a fall at the hips' velocity, moved on by the
            // rest of the frame: leaping up off it if a jump was asked while
            // sliding; asked to catch, turning round in the air to face the
            // eave and catch it (as any fall catches a ledge it is given).
            if asked_jump.is_some()
                && let Some(slide) = state.sloping.as_mut()
            {
                slide.leap();
            }
            if let Some(slide) = state.sloping.as_mut() {
                let before = slide.elapsed();
                slide.advance(dt);
                target.pose = match springs {
                    Some(springs) => slide.pose_led(&springs.0),
                    None => slide.pose(),
                };
                state.locomotion.position = slide.root();
                state.facing.yaw = slide.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                if slide.is_done() && slide.ends_at_drop() {
                    let leftover = (before + dt - slide.end()).max(0.0);
                    let root = slide.root();
                    let sample = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
                    let below = sample(root.with_y(slide.slope().foot().y - 0.05)).unwrap_or(0.0);
                    let mut falling = super::parkour::Falling::off(root, slide.facing(), slide.velocity(), &slide.pose(), below, foot_ik.pelvis_drop, &stood, &rig);
                    if walker.catch && !slide.leaps() {
                        falling.spin_round(std::f32::consts::PI, SLOPE_CATCH_TURN, &rig);
                    }
                    falling.land_on(&sample);
                    let ledges: Vec<super::parkour::Ledge> = walker.ledge.into_iter().chain(walker.ledges.iter().copied()).collect();
                    falling.against(&ledges, &rig);
                    falling.advance(leftover);
                    state.falling = Some(falling);
                    state.sloping = None;
                    slid_off = true;
                } else if slide.is_done() {
                    state.sloping = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                    let_go = true;
                }
            }
        }
        // Over an edge (`parkour::fall`): from the gait's pose and the root's
        // velocity it falls to the ground found below and lands, posed off
        // the floor, the root riding its hips; stood again, it walks on.
        if let Some(rig) = foot_ik.rig.clone() {
            // Started from this frame's pose, it moves on from the next.
            let mut started = leapt || slid_off;
            // Walked off an edge: the fall begins where the root was before
            // this frame's travel, so it goes this frame's way at once (begun
            // still, the body stood still a frame: a 5.7 cm change of step).
            let mut walked_off = false;
            // Jumping onto a small top (`parkour::precision`): at the jump's
            // top, a fall landing on the top, the hips at rest over its
            // middle.
            if let Some(top) = state.onto
                && state.jump.as_ref().is_some_and(super::parkour::precision::hands_over)
                && state.falling.is_none()
                && let Some(jump) = state.jump.take()
            {
                let root = state.locomotion.position + state.facing.rotation() * std::mem::take(&mut state.stride.stepped);
                state.falling = Some(super::parkour::precision::fall_onto(&jump, top, root, state.facing.yaw, foot_ik.pelvis_drop, &stood, &rig));
                state.onto = None;
                state.perched = Some(top);
                started = true;
            }
            // A turning jump (`parkour::spin`): a little into its flight, a
            // fall turning it round in the air, landing on the floor it left.
            if let Some(turn) = state.spinning
                && state.jump.as_ref().is_some_and(super::parkour::spin::hands_over)
                && state.falling.is_none()
                && let Some(jump) = state.jump.take()
            {
                let root = state.locomotion.position + state.facing.rotation() * std::mem::take(&mut state.stride.stepped);
                let floor = state.locomotion.position.y;
                state.falling = Some(super::parkour::spin::spin_fall(&jump, root, state.facing.yaw, floor, foot_ik.pelvis_drop, turn, &stood, &rig));
                state.spinning = None;
                started = true;
            }
            // Off a springboard (`parkour::springboard`): a little into its
            // flight, a fall landing on the first top along it (or the floor
            // it left), held off the walls it faces; caught as any fall.
            if state.springing.is_some()
                && state.jump.as_ref().is_some_and(super::parkour::springboard::hands_over)
                && state.falling.is_none()
                && let Some(jump) = state.jump.take()
            {
                let root = state.locomotion.position + state.facing.rotation() * std::mem::take(&mut state.stride.stepped);
                // The ground under it no higher than it left (a top higher
                // is landed on along the flight).
                let floor = state.locomotion.position.y;
                let top = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
                let below = top(root.with_y(floor + 0.05)).map_or(floor, |height| height.min(floor));
                let mut falling = super::parkour::springboard::spring_fall(&jump, root, state.facing.yaw, below, &top, foot_ik.pelvis_drop, &stood, &rig);
                let ledges: Vec<super::parkour::Ledge> = walker.ledge.into_iter().chain(walker.ledges.iter().copied()).collect();
                falling.against(&ledges, &rig);
                state.falling = Some(falling);
                state.springing = None;
                started = true;
            }
            if state.jump.is_none() {
                state.springing = None;
                state.cornering = None;
            }
            if let Some(below) = state.fall_to.take()
                && state.falling.is_none()
                && !fallen
            {
                started = true;
                let mut falling = match state.jump.take() {
                    // A jump in the air goes on falling as it flew, from
                    // where its travel this frame (not yet ridden) puts it.
                    Some(jump) => {
                        let root = state.locomotion.position + state.facing.rotation() * std::mem::take(&mut state.stride.stepped);
                        super::parkour::Falling::from_jump(&jump, root, state.facing.yaw, below, foot_ik.pelvis_drop, &stood, &rig)
                    }
                    None => {
                        walked_off = true;
                        let velocity = Vec3::new(state.locomotion.root_velocity.x, 0.0, state.locomotion.root_velocity.z);
                        super::parkour::Falling::off(state.locomotion.position, state.facing.yaw, velocity, &target.pose, below, foot_ik.pelvis_drop, &stood, &rig)
                    }
                };
                // Onto a top it comes down on along its flight (a running
                // jump across a gap); else against a wall faced, kept off it.
                // With moving platforms, in the frame it lands in: off one,
                // carrying its velocity; onto one, where it will be.
                let sample = |at: Vec3| ground.and_then(|ground| ground.0.sample(at)).map(|hit| hit.height);
                if platforms.is_empty() {
                    falling.land_on(&sample);
                } else {
                    let frame;
                    (falling, frame) = super::parkour::platform::frame_for_fall(falling, state.riding, &platforms, &sample);
                    // Walked off, it goes this frame's way at once: into the
                    // world from a platform, from before this frame's carry
                    // (its own velocity has the platform's in it now: both,
                    // 4.5 cm too far); from the world onto one, with this
                    // frame's carry (its own velocity is relative now:
                    // neither, 5.8 cm short).
                    if walked_off && state.riding != frame {
                        let moved = frame.and_then(|i| platforms.get(i)).map_or(Vec3::ZERO, |platform| platform.moved);
                        falling.shift(moved - state.carried);
                    }
                    state.riding = frame;
                }
                let ledges: Vec<super::parkour::Ledge> = walker.ledge.into_iter().chain(walker.ledges.iter().copied()).collect();
                falling.against(&ledges, &rig);
                state.falling = Some(falling);
            }
            // Chaining kicks (`parkour::wall`): flying across from a kick,
            // met by the wall kicked across to, it kicks off that from the
            // air toward the lip; landed, the chain is given up.
            if let Some((across, lip)) = state.kick_chain
                && let Some(falling) = state.falling.as_ref()
            {
                use super::parkour::wall::{KickTarget, WallRun};
                if !falling.airborne() {
                    state.kick_chain = None;
                } else if let Some(run) = WallRun::kick_from_air(&across, KickTarget::Lip(lip), falling, foot_ik.pelvis_drop, &stood, &rig) {
                    target.pose = run.pose();
                    state.locomotion.position = run.root();
                    state.facing.yaw = run.facing();
                    state.facing.target_yaw = state.facing.yaw;
                    foot_ik.planted = [false; 2];
                    foot_ik.landing = None;
                    foot_ik.touchdown = None;
                    legs_free = true;
                    state.wall_run = Some(run);
                    state.falling = None;
                    state.kick_chain = None;
                }
            }
            if let Some(falling) = state.falling.as_mut() {
                // Asked to swing round a flagpole or on hooks: reaching up
                // for them.
                if walker.flagpole.is_some() || !walker.hooks.is_empty() {
                    falling.reach(true);
                }
                if !falling.airborne() {
                    state.last_hook = None;
                }
                if !started || walked_off {
                    falling.advance(dt);
                }
                target.pose = match springs {
                    Some(springs) => falling.pose_led(&rig, &springs.0),
                    None => falling.pose(&rig),
                };
                state.locomotion.position = falling.root();
                state.facing.yaw = falling.facing();
                state.facing.target_yaw = state.facing.yaw;
                foot_ik.planted = [false; 2];
                foot_ik.landing = None;
                foot_ik.touchdown = None;
                foot_ik.clear = [0.0; 2];
                foot_ik.gait_swing = None;
                foot_ik.gait_bearing = None;
                legs_free = true;
                // Asked to catch: reaching up, the hands catch a ledge it
                // falls past (its ledge, or any about it), hanging from it.
                // A leap's flight catches the ledge leapt at, whatever is asked.
                // Sliding down a wall, the arms are up on its face already.
                let target = falling.target();
                if target.is_none() && !falling.is_sliding() {
                    falling.reach(walker.catch);
                }
                let ledges: Vec<super::parkour::Ledge> = walker.ledge.into_iter().chain(walker.ledges.iter().copied()).collect();
                let catchable: Vec<super::parkour::Ledge> = match target {
                    Some(target) => vec![target],
                    None if walker.catch => ledges.clone(),
                    None => Vec::new(),
                };
                let caught = falling.airborne() && !catchable.is_empty();
                if let Some(ledge) = caught.then(|| falling.catches(&catchable, &rig)).flatten() {
                    let pose = falling.pose(&rig);
                    let grips = hands.as_ref().map_or([None; 2], |hands| hands.grips);
                    let square = super::parkour::Hanging::square(&ledge, rig.forward());
                    let hanging = super::parkour::Hanging::caught(&ledge, &ledges, falling.hips(), falling.hips_velocity(), &pose, falling.root(), square, foot_ik.pelvis_drop, grips, &stood, &rig);
                    state.hanging = Some(hanging);
                    state.falling = None;
                } else if let Some(pole) = walker.pole.filter(|_| walker.catch && falling.airborne())
                    && let Some(mut poling) = super::parkour::Poling::caught(&pole, falling.hips(), falling.hips_velocity(), &falling.pose(&rig), falling.facing(), falling.ground(), &stood, &rig)
                {
                    // Asked to catch, coming by its pole: caught on it
                    // (`parkour::pole`, a jump to a pole).
                    if let Some(hands) = hands.as_ref() {
                        poling.set_grips(hands.grips);
                    }
                    state.poling = Some(poling);
                    state.falling = None;
                } else if let Some(pole) = walker.flagpole.filter(|_| falling.airborne())
                    && let Some(mut swinging) = super::parkour::flagpole::Swinging::caught(&pole, falling.hips(), falling.hips_velocity(), &falling.pose(&rig), falling.facing(), &stood, &rig)
                {
                    // Asked to swing round a flagpole, coming by it: caught
                    // on it (`parkour::flagpole`).
                    if let Some(hands) = hands.as_ref() {
                        swinging.set_grips(hands.grips);
                    }
                    state.flagging = Some(swinging);
                    state.falling = None;
                } else if falling.airborne()
                    && let Some((hook, mut swinging)) = walker
                        .hooks
                        .iter()
                        .filter(|&&hook| state.last_hook != Some(hook))
                        .find_map(|&hook| super::parkour::flagpole::Swinging::caught_hook(hook, falling.hips(), falling.hips_velocity(), &falling.pose(&rig), falling.facing(), &stood, &rig).map(|s| (hook, s)))
                {
                    // Falling by a hook asked to swing on: caught on it
                    // one-handed (`parkour::flagpole`, a hook).
                    if let Some(hands) = hands.as_ref() {
                        swinging.set_grips(hands.grips);
                    }
                    state.last_hook = Some(hook);
                    state.flagging = Some(swinging);
                    state.falling = None;
                } else if falling.is_fatal() && !falling.airborne() && ragdoll.is_some() {
                    // Too far to land: at touchdown the body goes to the
                    // ragdoll, falling on with the velocity it hit with (a
                    // walker without one lands it as it can).
                    walker.fall_now = true;
                    state.falling = None;
                } else if falling.is_done() {
                    state.falling = None;
                    state.stood_hold = STOOD_HOLD;
                    phase.elapsed = 0.0;
                }
            }
        }
        if foot_ik.legs_free != legs_free {
            foot_ik.legs_free = legs_free;
        }
        // Landed from a fall, on the floor again: the sprung pose kept clear
        // of it (left off it, the feet went 0.18 m through landing from 3 m).
        // Off a springboard, its foot sunk under the floor with the board.
        let off_floor = (state.on_holds() && !state.falling.as_ref().is_some_and(|falling| falling.is_landed())) || state.springing.is_some();
        if foot_ik.off_floor != off_floor {
            foot_ik.off_floor = off_floor;
        }


        // Looking round (a viewpoint's): the gaze swept 1.1 rad either side
        // of its facing and back every 7 s, level.
        if walker.look_round && look_at.is_none() {
            state.look_round_t += time.delta_secs();
            let ahead = approach::heading_of(gait_rig.forward());
            let swept = 1.1 * (std::f32::consts::TAU * state.look_round_t / 7.0).sin();
            look_at = Some(root.translation + approach::direction_of(state.facing.yaw + swept + ahead) * 4.0 + Vec3::Y * 1.5);
        } else {
            state.look_round_t = 0.0;
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

/// Going round a wall: the facing it wants, the side it turned off it (+1 or
/// -1), and where it turned off, on the line it goes back to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Detour {
    pub wanted: f32,
    pub side: f32,
    pub from: Vec3,
}

/// Which way to walk with a wall ahead ([`way_round`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WayRound {
    /// The way wanted is clear.
    Clear,
    /// Turned to this facing (radians about `+Y`), to this side of the way
    /// wanted (+1 or -1).
    Turn { yaw: f32, side: f32 },
    /// No way within a quarter turn either side: it stops.
    Blocked,
}

/// The way round a wall for a body at `at` wanting to face `wanted` (its
/// facing, radians about `+Y`, the rig facing `forward` at none), looking
/// `reach` ahead: the way wanted if clear, else the least turn off it, in
/// steps of [`DETOUR_STEP`] up to a quarter turn, that is clear, trying
/// `side` first (so it keeps to the side it chose and does not dither).
/// Walking along the wall, past its end the way wanted is clear again: it
/// goes round. A wall is something solid from a step higher than it stands
/// up to its [`HEADROOM`] (`GroundProbe::blocks`).
pub fn way_round(at: Vec3, wanted: f32, side: f32, reach: f32, forward: Vec3, ground: &dyn super::ground::GroundProbe) -> WayRound {
    let wall = |point: Vec3| ground.blocks(point, at.y + super::parkour::fall::STEP_DOWN, at.y + HEADROOM);
    // The body's width swept along it (a line from the middle cleared a
    // block's corner the body could not, and it stuck there).
    let clear = |yaw: f32, reach: f32| {
        let way = Quat::from_rotation_y(yaw) * forward;
        let across = Vec3::Y.cross(way) * BODY_RADIUS;
        let steps = (reach / 0.25).ceil().max(3.0) as usize;
        (1..=steps).all(|k| [-1.0, 0.0, 1.0].iter().all(|&side| !wall(at + way * (reach * k as f32 / steps as f32) + across * side)))
    };
    if clear(wanted, reach) {
        return WayRound::Clear;
    }
    // A way round leads somewhere: clear for `DETOUR_CLEAR` (a turn clear
    // only to the side wall of a dead end, it went back and forth there).
    let first = if side < 0.0 { -1.0 } else { 1.0 };
    for k in 1..=(std::f32::consts::FRAC_PI_2 / DETOUR_STEP).round() as usize {
        for side in [first, -first] {
            let yaw = wanted + side * DETOUR_STEP * k as f32;
            if clear(yaw, reach.max(DETOUR_CLEAR)) {
                return WayRound::Turn { yaw, side };
            }
        }
    }
    WayRound::Blocked
}

/// Back to its line after going round a wall: the facing that heads for the
/// point on the line through `from` along the facing `wanted` (the rig
/// facing `forward` at none) [`LINE_LOOKAHEAD`] ahead of the body at `at`,
/// turned at most [`LINE_MOST_TURN`] off `wanted`; and how far off the line
/// the body is, metres. Gone round a block, it walked on along a parallel
/// line 1.2 m out.
pub fn back_to_line(at: Vec3, from: Vec3, wanted: f32, forward: Vec3) -> (f32, f32) {
    let way = Quat::from_rotation_y(wanted) * forward;
    let left = Vec3::Y.cross(way);
    let off = (at - from).dot(left);
    (wanted - (off / LINE_LOOKAHEAD).atan().clamp(-LINE_MOST_TURN, LINE_MOST_TURN), off.abs())
}

/// The body standing at `at` (the root, on the ground) moved `moved`, kept
/// out of walls: something solid from a step higher than it stands on up
/// to its [`HEADROOM`] within [`BODY_RADIUS`] of the root. Into one, the
/// move slides along it, or stops; and the wall's way out, if one held it.
pub fn keep_off_walls(at: Vec3, moved: Vec3, ground: &dyn super::ground::GroundProbe) -> (Vec3, Option<Vec3>) {
    let wall = |point: Vec3| ground.blocks(point, at.y + super::parkour::fall::STEP_DOWN, at.y + HEADROOM);
    // How near the nearest wall is standing at `centre` (`BODY_RADIUS` if
    // none within it), and which way: along each way round a wall is in,
    // where it starts. (Counted by the probes that hit, a corner's way out
    // was 22° off, and sliding along it, it stuck.)
    let clearance = |centre: Vec3| -> (f32, Vec3) {
        // Clear of every other probe first, the rest not looked at (a corner
        // can come 0.4 cm in between them; between 8, it came 2.7 cm in): on
        // open floor, the cost.
        let coarse = (0..WALL_PROBES).step_by(2).any(|k| {
            let angle = std::f32::consts::TAU * k as f32 / WALL_PROBES as f32;
            wall(centre + Vec3::new(angle.cos(), 0.0, angle.sin()) * BODY_RADIUS)
        });
        if !coarse {
            return (BODY_RADIUS, Vec3::ZERO);
        }
        (0..WALL_PROBES)
            .map(|k| {
                let angle = std::f32::consts::TAU * k as f32 / WALL_PROBES as f32;
                Vec3::new(angle.cos(), 0.0, angle.sin())
            })
            .filter(|&way| wall(centre + way * BODY_RADIUS))
            .map(|way| {
                let (mut low, mut high) = (0.0, BODY_RADIUS);
                for _ in 0..8 {
                    let middle = 0.5 * (low + high);
                    if wall(centre + way * middle) { high = middle } else { low = middle }
                }
                (high, way)
            })
            .fold((BODY_RADIUS, Vec3::ZERO), |nearest, found| if found.0 < nearest.0 { found } else { nearest })
    };
    // Moved, then pushed back out of any wall it reaches, along the way it
    // found it, a few rounds (a corner found along two ways): it slides
    // along, and stops only square on. (Its move cut to the part along a
    // wall instead, it stuck at a block's corner.)
    let mut to = at + moved;
    let mut out = None;
    for _ in 0..4 {
        let (near, toward) = clearance(to);
        if toward == Vec3::ZERO {
            break;
        }
        to -= toward * (BODY_RADIUS - near + 1.0e-3);
        out = Some(-toward);
    }
    (to - at, out)
}

/// What [`ride_rendered_feet`] moves: the walker's own for its platforms.
type RiddenRig = (&'static AnimPose, &'static mut WalkerState, &'static mut AnimFootIk, &'static mut Transform, &'static AnimGround, Option<&'static Walker>);

/// Moves each walker by exactly how far its planted feet moved under it in
/// the pose just RENDERED, after the springs, before the IK: integrating the
/// gait's published velocity instead erred by `½·a·dt²` and by the springs'
/// lag, and a planted foot slid 39 mm a stance.
pub fn ride_rendered_feet(time: Res<Time>, mut rigs: Query<RiddenRig>) {
    for (pose, mut state, mut foot_ik, mut root, ground, walker) in &mut rigs {
        let state = &mut *state;
        let platforms = walker.map(|walker| walker.platforms.now()).unwrap_or_default();
        let carried = std::mem::take(&mut state.carried);
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
            // Falling, running up a wall or sliding down a slope, where it is
            // is the move's (`parkour::Falling::root`), set each frame: the
            // gait's motion added on was a frame's lag (sliding, its first
            // frame went 4.5 cm too far).
            _ if state.falling.is_some() || state.wall_run.is_some() || state.sloping.is_some() => {
                state.stride.stepped = Vec3::ZERO;
                Vec3::ZERO
            }
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
        // Walking, kept out of walls: run on after landing at one's foot, it
        // went on straight through the block. (On holds or jumping, the
        // move places the body itself.)
        let moved = if state.on_holds() || state.jump.is_some() { moved } else { keep_off_walls(state.locomotion.position, moved, ground.0.as_ref()).0 };
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
            // Over an edge, the ground more than a step below: it falls
            // (`parkour::fall`, started next frame), not snapped down to it;
            // in a jump's flight too (a jump falling short of the far side
            // was snapped down to the floor of the gap).
            // Pushing off the edge, the root over the drop already, the
            // feet still on the top: held up until it leaves (snapped down,
            // a jump from 0.3 m back landed on the floor of the gap).
            let drops = height < height_before - super::parkour::fall::STEP_DOWN;
            // A jump whose own landing is on ground as high (a gap cleared)
            // is held up over the gap to land and run on as it would.
            let clears = state.jump.as_ref().is_some_and(|jump| {
                let left = jump.travelled_at(jump.ends(super::jump::JumpPhase::Flight)) - jump.travelled();
                let lands = root.translation + state.facing.rotation() * rig.forward() * left;
                ground.0.sample(lands.with_y(height_before)).is_some_and(|hit| hit.height >= height_before - super::parkour::fall::STEP_DOWN)
            });
            // Leaping off a springboard, its own fall takes it at the top of
            // its flight (handed one rising toward a higher top, a fall's
            // landing went NaN).
            if drops && state.jump.as_ref().is_some_and(|jump| clears || state.springing.is_some() || (!jump.airborne() && jump.elapsed() < jump.ends(super::jump::JumpPhase::Flight))) {
                root.translation.y = height_before;
                state.locomotion.position.y = height_before;
            } else if drops && state.jump.as_ref().is_none_or(|jump| jump.airborne()) {
                root.translation.y = height_before;
                state.locomotion.position.y = height_before;
                state.fall_to = Some(height);
            } else {
                root.translation.y = height;
            }
        }
        // The rise is travel too: a lock that knew only the horizontal part
        // carried a planted foot 9 cm up a 0.2 grade every stance.
        // Less a platform's own rise, which carries the planted feet too.
        foot_ik.turn.travel.y = root.translation.y - height_before - carried.y;
        // Standing or walking on a moving platform, carried with it from
        // the next frame; on holds, not. (In a jump or a fall, the frame it
        // began in, or was planned to land in.) Walked off its edge, kept
        // till the fall begins next frame, which carries its velocity on:
        // let go here, the fall began in the world, standing still.
        if state.jump.is_none() && state.falling.is_none() && state.fall_to.is_none() {
            state.riding = if state.on_holds() { None } else { platforms.iter().position(|platform| platform.stood_on_by(state.locomotion.position)) };
        }
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

    /// Walking at 1.4 m/s straight at a 2 m block it turns off and goes
    /// round it, back onto its way past it; along a wall too long to see
    /// the end of, it follows it; into a dead end (three walls), it stops.
    /// Never into a wall. (It stopped at the first wall.)
    #[test]
    fn walking_goes_round_a_wall() {
        use crate::character::anim::ground::{FlatGround, GroundProbe};
        use crate::character::anim::parkour::{geometry::LedgeGround, Ledge};
        let forward = Vec3::NEG_Z;
        // A point walker: turned toward its steer at the detour's rate,
        // moved through `keep_off_walls`, wanting to face 0 (-Z).
        let walk = |ledges: Vec<Ledge>, seconds: f32| {
            let ground = LedgeGround::new(Box::new(FlatGround::default()), ledges);
            let (dt, speed) = (1.0 / 60.0, 1.4);
            let (mut at, mut yaw, mut detour, mut deepest, mut stopped) = (Vec3::ZERO, 0.0f32, None::<Detour>, 0.0f32, false);
            for _ in 0..(seconds / dt) as usize {
                let reach = BODY_RADIUS + WALL_STOP_MARGIN + WALL_STOP_TIME * speed;
                let (wanted, side) = (0.0, detour.map_or(0.0, |detour| detour.side));
                let (mut toward, mut go) = (wanted, true);
                match way_round(at, wanted, side, reach, forward, &ground) {
                    WayRound::Clear => {
                        if let Some(line) = detour {
                            let (back, off) = back_to_line(at, line.from, wanted, forward);
                            if way_round(at, back, side, reach, forward, &ground) == WayRound::Clear {
                                toward = back;
                            }
                            if off < LINE_ON && crate::character::anim::facing::shortest_angle(yaw - wanted).abs() < LINE_FACING {
                                detour = None;
                            }
                        }
                    }
                    WayRound::Turn { yaw, side } => {
                        toward = yaw;
                        detour = Some(Detour { wanted, side, from: detour.map_or(at, |detour| detour.from) });
                    }
                    WayRound::Blocked => go = false,
                }
                stopped = !go;
                let turn = crate::character::anim::facing::shortest_angle(toward - yaw);
                yaw += turn.clamp(-DETOUR_RATE * dt, DETOUR_RATE * dt);
                if go {
                    at += keep_off_walls(at, Quat::from_rotation_y(yaw) * forward * (speed * dt), &ground).0;
                }
                // Whether the body's circle, less 1 cm, reaches a block.
                deepest = (0..16)
                    .map(|k| at + Quat::from_rotation_y(k as f32 * std::f32::consts::TAU / 16.0) * Vec3::X * (BODY_RADIUS - 0.01))
                    .filter(|&point| ground.sample(point + Vec3::Y * 100.0).is_some_and(|hit| hit.height > 0.3))
                    .count()
                    .max(deepest as usize) as f32;
            }
            (at, yaw, stopped, deepest)
        };
        // A 2 m block straight ahead, its near face 2 m off.
        let block = Ledge::block(Vec3::new(0.0, 0.0, -2.0), Vec3::Z, 2.0, 2.0, 3.0);
        let (at, yaw, stopped, into) = walk(block.to_vec(), 8.0);
        assert!(into == 0.0, "round the block, the body went into it");
        assert!(at.z < -5.0 && !stopped, "round the block, ended at {at:?}, stopped {stopped}");
        assert!(crate::character::anim::facing::shortest_angle(yaw).abs() < 0.05, "past the block, facing {yaw:.2} off its way");
        assert!(at.x.abs() < LINE_ON, "past the block, {:.3} m off its line (on a parallel line, 1.2 m)", at.x);
        // A wall 40 m wide straight ahead: along it.
        let wall = Ledge::block(Vec3::new(0.0, 0.0, -2.0), Vec3::Z, 40.0, 1.0, 3.0);
        let (at, _, stopped, into) = walk(wall.to_vec(), 6.0);
        assert!(into == 0.0 && !stopped && at.x.abs() > 3.0, "along the wall, ended at {at:?}, stopped {stopped}");
        // A dead end: walls ahead and either side, closed.
        let ahead = Ledge::block(Vec3::new(0.0, 0.0, -2.0), Vec3::Z, 4.0, 1.0, 3.0);
        let left = Ledge::block(Vec3::new(-0.8, 0.0, -0.5), Vec3::X, 3.0, 0.5, 3.0);
        let right = Ledge::block(Vec3::new(0.8, 0.0, -0.5), Vec3::NEG_X, 3.0, 0.5, 3.0);
        let (_, _, stopped, into) = walk([ahead, left, right].concat(), 6.0);
        assert!(into == 0.0 && stopped, "in the dead end: stopped {stopped}");
        // A bar 2.3 m up across the way: straight on under it; at 1.5 m,
        // round it.
        let (at, _, stopped, _) = walk(vec![Ledge::bar(Vec3::new(0.0, 0.0, -2.0), Vec3::Z, 8.0, 2.3)], 6.0);
        assert!(!stopped && at.x.abs() < 1.0e-3 && at.z < -5.0, "under a 2.3 m bar, ended at {at:?}, stopped {stopped}");
        let (at, _, _, _) = walk(vec![Ledge::bar(Vec3::new(0.0, 0.0, -2.0), Vec3::Z, 8.0, 1.5)], 6.0);
        assert!(at.x.abs() > 1.0 || at.z > -2.0, "a 1.5 m bar walked through, ended at {at:?}");
    }

    /// Walking into a 3 m block's face from the floor the body stops
    /// `BODY_RADIUS` off it, held, the face's way out given; diagonally it
    /// slides along it and on past its end; away it goes free; never into
    /// it; and on the top, walking to its
    /// edge (the floor below), nothing holds it. Run on after landing at a
    /// wall's foot, it went straight through the block.
    #[test]
    fn walking_is_kept_out_of_walls() {
        use crate::character::anim::ground::FlatGround;
        use crate::character::anim::parkour::{geometry::LedgeGround, Ledge};
        let block = Ledge::block(Vec3::new(0.0, 0.0, -1.0), Vec3::Z, 2.0, 2.0, 3.0);
        let ground = LedgeGround::new(Box::new(FlatGround::default()), block.to_vec());
        // The face at z = -1, facing +z.
        // The body nearer the block (x -1..1, z -3..-1) than its radius,
        // less 2 cm.
        let inside = |at: Vec3| {
            let (dx, dz) = ((at.x.abs() - 1.0).max(0.0), (at.z - (-1.0)).max(-3.0 - at.z).max(0.0));
            dx.hypot(dz) < BODY_RADIUS - 0.02
        };
        let walk = |from: Vec3, way: Vec3| {
            let mut at = from;
            let mut held = None;
            for _ in 0..200 {
                let (moved, blocked) = keep_off_walls(at, way * 0.02, &ground);
                at += moved;
                held = blocked;
                assert!(!inside(at), "walking {way:?}, the body at {at:?} reached into the block");
            }
            (at, held)
        };
        let (at, held) = walk(Vec3::new(0.0, 0.0, 0.0), Vec3::NEG_Z);
        assert!((at.z - (-1.0 + BODY_RADIUS)).abs() < 0.03, "head on, stopped at z {:.3}", at.z);
        assert!(held.is_some_and(|out| out.dot(Vec3::Z) > 0.9), "head on, held by {held:?}");
        let (at, _) = walk(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, -1.0).normalize());
        assert!(at.x > 1.0 + BODY_RADIUS && at.z < -2.0, "diagonally, ended at {at:?} (not slid along and past its end)");
        let (at, held) = walk(Vec3::new(0.0, 0.0, -0.75), Vec3::Z);
        assert!(at.z > 3.0 && held.is_none(), "away, ended at {at:?}, held {held:?}");
        let (moved, held) = keep_off_walls(Vec3::new(0.0, 3.0, -1.2), Vec3::Z * 0.5, &ground);
        assert!((moved.z - 0.5).abs() < 1.0e-6 && held.is_none(), "on the top to its edge, moved {moved:?}, held {held:?}");
    }
}
