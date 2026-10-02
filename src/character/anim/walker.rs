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
use super::{balance, facing, locomotion, lookat, poses, transition};
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
    /// and out (Winter §11.3.3), a run above [`RUN_ABOVE`].
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
        }
    }
}

impl Walker {
    /// Shoves the character, changing its centre of mass's velocity by
    /// `velocity` m/s along its own (forward, left): standing it sways and
    /// steps, walking its next footfalls move.
    pub fn push(&mut self, velocity: Vec2) {
        self.pushes.push(velocity);
    }
}

/// A run above this speed, m/s, a walk below: where a fast walk's cadence
/// would cost more than a run's flight.
pub const RUN_ABOVE: f32 = 2.2;

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
    /// The stride the current gait really takes, keyed by its speed and
    /// whether the real rig has bound: measuring it costs a cycle of
    /// root-motion samples, so it is redone only when either changes.
    measured: Option<(u32, bool, f32)>,
}

impl WalkerState {
    /// A walker standing at `position`, facing `yaw`.
    pub fn at(position: Vec3, yaw: f32) -> Self {
        Self {
            locomotion: locomotion::Locomotion { mode: locomotion::RootMotion::Drive, position, ..Default::default() },
            facing: facing::Facing { yaw, target_yaw: yaw, ..Default::default() },
            transition: Default::default(),
            look: lookat::LookAt::forward(),
            stride: Default::default(),
            measured: None,
        }
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
);

/// Drives each walker's gait from its clock, in `AnimSet::Target`, so the
/// phase layer composes on top and the springs smooth the result.
pub fn drive_walkers(time: Res<Time>, mut rigs: Query<WalkingRig>) {
    for (mut walker, mut target, mut phase, mut state, mut arm_ik, mut foot_ik, mut root, correction, mut layer, mut balance, mut walk_balance, ragdoll) in
        &mut rigs
    {
        let state = &mut *state;
        // Composed onto the AUTHORED pose, re-read every frame: composed onto
        // last frame's result, each frame layered another cycle on and the
        // legs wound up without bound.
        let base = poses::by_name(&walker.pose).unwrap_or_else(poses::relaxed_stand);
        let due_pushes = std::mem::take(&mut walker.pushes);
        // The rig the gait is posed on: the real one once bound, the
        // synthetic proxy for the first frames. The gait's vertical motion
        // is in fractions of THIS rig's leg.
        let gait_rig = foot_ik.rig.clone().unwrap_or_default();

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
        let asked = if ragdoll.is_some_and(Ragdoll::is_falling) { 0.0 } else { walker.speed + walk_balance.surge };
        let event = state.transition.advance(asked, cycle_of(&phase), &config, time.delta_secs());
        let speed = state.transition.stride_speed;

        // A walk's stride grows with its speed, scaled to this rig's leg.
        let params = if speed >= RUN_ABOVE { GaitParams::running() } else { GaitParams::walking_on(speed, &gait_rig) };

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

        // The speed reaches the LEG clock as the cadence that makes this
        // gait's stride travel at exactly this speed: set once at spawn, the
        // legs kept the launch rhythm while the body moved at the new speed,
        // and the planted feet slid by the difference.
        let key = (speed.to_bits(), foot_ik.rig.is_some());
        let distance = match state.measured {
            Some((speed, bound, distance)) if (speed, bound) == key => distance,
            _ => {
                let distance = locomotion::distance_per_cycle(&params, &stood, &gait_rig);
                state.measured = Some((key.0, key.1, distance));
                distance
            }
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
        if !state.transition.is_at_rest() {
            wanted.sway = None;
        } else if let Some(sway) = wanted.sway.as_mut() {
            const SETTLE_SECONDS: f32 = 1.5;
            let t = (phase.elapsed / SETTLE_SECONDS).clamp(0.0, 1.0);
            let settled = t * t * (3.0 - 2.0 * t);
            sway.lateral *= settled;
            sway.fore_aft *= settled;
        }
        if layer.0 != wanted {
            layer.0 = wanted;
        }

        // The pose as a function of phase, ONE definition, used both to pose
        // the character and to derive its root motion, so the two cannot
        // disagree. Blended from `stood`, so the standing knee bend does not
        // pop as a walk begins or ends.
        let mut prepared = stood;
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
                foot_ik.landing = Some(Landing { left, spot, strength });
            }
            balance.apply(&mut prepared, &gait_rig);
        }
        // The feet the balance has down stay locked however its sprung legs
        // lag a stumbling body; a walk's feet are the locks' own call.
        foot_ik.planted = if weight <= 0.0 && !balance.is_settled(1.0e-5) { balance.planted() } else { [false; 2] };

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
        let transition_state = &state.transition;
        let rendered = |cycle: f32| {
            if weight <= 0.0 {
                prepared
            } else {
                let walking = walk_pose_on(cycle, &params, &stood, &gait_rig);
                transition_state.blend(&prepared, &walking, &gait_rig)
            }
        };
        // Clippy misses the second use: root motion reads `&rendered` below.
        #[allow(clippy::redundant_closure_call)]
        {
            target.pose = rendered(cycle);
        }

        // The look, composed after the gait, independent of it.
        // Retargeted in place, so the look eases from where it is.
        state.look.target = walker.look_at;
        if let Some(direction) = state.look.advance(
            lookat::head_position(&target.pose, &RigGeometry::default()) + root.translation,
            state.facing.rotation(),
            &lookat::LookAtConfig::default(),
            time.delta_secs(),
        ) {
            lookat::apply(&mut target.pose, direction, &lookat::LookAtConfig::default(), &RigGeometry::default());
        }

        // The reach is solved by the plugin in `AnimSet::Ik`, against the
        // live rig: solved here against the synthetic proxy (mirrored from
        // the real rig) it sent the hand to the wrong side of the body.
        arm_ik.left = walker.reach;

        match walker.steer {
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

        // The entity turns to match the heading, composed onto the asset's
        // own correction (heading first, then the correction): assigned over
        // it, the correction was wiped and the character walked backward.
        root.rotation = state.facing.rotation() * correction.map_or(Quat::IDENTITY, |c| c.0);
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
        // 7.7 m under a hillside after 26 m of a 0.3 grade.
        if let Some(height) = locomotion::ground_following_height(root.translation, 0.0, ground.0.as_ref()) {
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
