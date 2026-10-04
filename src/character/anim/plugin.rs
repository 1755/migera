//! The Bevy glue: [`AnimPlugin`], its components, and its system sets.
//!
//! Everything in the rest of `character::anim` is plain functions over plain
//! data, deliberately unaware of Bevy. This module is the only place that
//! knows about schedules, queries, and components, which is what keeps the
//! numerical core testable in microseconds without a `World`.
//!
//! # Using it
//!
//! ```ignore
//! app.add_plugins(AnimPlugin::default());
//!
//! // then, per character:
//! commands.entity(rig).insert((
//!     AnimTarget::settled_on(relaxed_stand()),
//!     AnimSprings::default(),
//! ));
//! ```
//!
//! The plugin never spawns a skeleton, never owns a camera, and never reads
//! CLI arguments. A consumer builds its own [`HumanoidSkeleton`] — whether
//! from this crate's synthetic rig or from a real glTF via
//! `HumanoidSkeleton::for_other_rig` — and the plugin animates whatever it
//! finds. Examples are thin consumers, not the home of the logic.

use bevy::prelude::*;

use super::armik::{solve_arm_on, ArmChain, ArmIkConfig, ArmTarget};
use super::asset::PoseAsset;
use super::dho::{default_springs, DhoState};
use super::footlock::{FootLock, FootLockConfig, Turn};
use super::ground::{FlatGround, GroundProbe};
use super::legik::{solve_leg_grounded, LegChain, LegIkConfig};
use super::pelvis::{apply_pelvis_drop, solve_pelvis_drop, PelvisConfig};
use super::math::spring::SpringParams;
use super::phase::{GaitPhase, PhaseLayer};
use super::rig::{forward_kinematics_on, RigGeometry};
use crate::character::skeleton::Bone;
use super::retarget::write_pose_to_skeleton;
use super::rig::{BoneSet, LocalPose};
use crate::character::skeleton::HumanoidSkeleton;

/// The per-frame stages, exposed so a consumer can order its own systems
/// against them (e.g. driving [`AnimTarget`] before the springs read it, or
/// reading the solved pose after write-back).
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AnimSet {
    /// Resolve what the rig should be aiming at this frame.
    Target,
    /// Layer continuous procedural motion onto that target.
    Phase,
    /// Integrate the per-joint springs toward the result.
    Spring,
    /// Adapt the sprung pose to the ground: foot locking and leg IK.
    Ik,
    /// Write the solved pose onto the skeleton's `Transform`s.
    Write,
}

/// The pose a character is currently being driven toward.
///
/// Set `pose` at any time and the rig springs to it — there is no
/// transition to start, no blend to schedule, and no bad moment at which to
/// interrupt. See [`DhoState::retarget`].
#[derive(Component, Debug, Clone, Copy)]
pub struct AnimTarget {
    /// The goal pose.
    pub pose: LocalPose,
}

impl AnimTarget {
    /// Aims at `pose`. The rig springs toward it from wherever it is.
    pub fn new(pose: LocalPose) -> Self {
        Self { pose }
    }

    /// Aims at `pose` — paired with [`AnimPose::settled_on`] when a
    /// character should *begin* in a pose rather than spring into it.
    pub fn settled_on(pose: LocalPose) -> Self {
        Self { pose }
    }
}

impl Default for AnimTarget {
    fn default() -> Self {
        Self { pose: LocalPose::REST }
    }
}

/// Ground adaptation for one character: foot locking plus leg IK.
///
/// Optional, like the phase layer. A character without it is driven purely
/// by its authored pose, which is what a cutscene or an airborne character
/// wants.
#[derive(Component, Debug, Clone, Default)]
pub struct AnimFootIk {
    /// Thresholds for locking and releasing.
    pub lock: FootLockConfig,
    /// How the two-bone solve behaves.
    pub ik: LegIkConfig,
    /// Left foot state.
    pub left: FootLock,
    /// Right foot state.
    pub right: FootLock,
    /// How far the hips may drop to help a foot reach.
    pub pelvis: PelvisConfig,
    /// How far the body turned this frame.
    ///
    /// Written by whatever owns the character's heading — see
    /// [`super::locomotion::advance_turning`], which returns exactly this.
    /// A planted foot pivots with it instead of being dragged sideways; left
    /// at [`Turn::NONE`] the locks behave as they always have.
    ///
    /// Its `travel` is how far the character entity moved this frame, WORLD
    /// axes, written by whatever moves it. A lock left without it rides
    /// along with the body.
    pub turn: Turn,
    /// The ground-corrected pose, recomputed each frame from the animated
    /// one. Kept here rather than in [`AnimPose::state`] so the correction
    /// never feeds back into the spring — see that field's own note.
    pub corrected: Option<LocalPose>,
    /// How far the hips were actually lowered this frame, metres.
    ///
    /// Exposed for debugging and for a consumer that wants to react to the
    /// character crouching — it is also the honest readout of how hard the
    /// ground adaptation is working.
    pub pelvis_drop: f32,
    /// The rig this character is actually being driven on, as the IK stage
    /// measured it from the live skeleton.
    ///
    /// Published because a caller that poses the gait needs the SAME rig
    /// the solve used, and rebuilding it independently is both wasteful and
    /// a chance for the two to disagree. `None` until the skeleton binds —
    /// a glTF loads asynchronously, so the first frames have nothing to
    /// measure.
    ///
    /// Needed since the gait's vertical amplitudes became fractions of leg
    /// length (see [`super::gait::GaitParams::hip_dip`]): posing on the
    /// synthetic proxy while solving on a real rig scales the body's
    /// vertical motion to the wrong leg.
    pub rig: Option<RigGeometry>,
    /// A foot being set down onto a spot, if any: the last step of a stop
    /// (`transition::Transition::landing`).
    ///
    /// Judged here, on the sprung pose, and not on the target: the legs'
    /// springs lag a fast-swinging foot by ~10 cm, so a target that set the
    /// foot down onto its spot left the RENDERED foot creeping its last
    /// ~2 cm within 3 mm of the floor. The IK holds that toe up by
    /// [`super::transition::landing_lift`] of the rendered foot's own
    /// distance from its spot.
    pub landing: Option<Landing>,
    /// Feet the caller knows are down (left, right): their locks hold
    /// however fast the sprung foot moves ([`FootLock::update_planted`]).
    /// Written each frame by whatever owns contact, such as a standing
    /// balance; `[false; 2]` leaves contact to the locks' own speed test.
    pub planted: [bool; 2],
    /// How far each foot (left, right) goes from where the animation puts
    /// it, metres, the pose's frame, horizontal: where a walking balance
    /// sets a foot down to catch a push (`walk_balance::WalkBalance`).
    /// Added to the animated toe before the lock and the ground see it, so
    /// a displaced foot locks, grounds and releases where it really is.
    pub displaced: [Vec3; 2],
    /// Leave the legs as the pose has them: no foot locks, leg IK or pelvis
    /// drop. For a pose solved on its own contacts (`sitting`): the leg IK
    /// keeps each knee in its leg's plane, so a cross-legged pose's knees,
    /// turned 55° out, came back pointing straight ahead.
    pub legs_free: bool,
}

/// A foot being set down onto a spot. See [`AnimFootIk::landing`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Landing {
    /// Which foot: `true` the rig's left.
    pub left: bool,
    /// Where its toe joint will stand, in the pose's frame.
    pub spot: Vec3,
    /// How much of the lift applies, 0 to 1.
    pub strength: f32,
    /// Also carry the foot across onto its spot, by `strength`, before its
    /// lock sees it: for a step whose spot is planned, not posed (a
    /// balance's, `balance::Balance::landing_spot`). Left to the sprung
    /// foot, a side step's landed 1.9 cm wide, its lock held it there, and
    /// the leg, a centimetre short of straight, left its ankle 18 mm up.
    pub place: bool,
}

/// Where one character's hands are reaching, in world space.
///
/// Optional, like [`AnimFootIk`]. A character without it leaves its arms
/// entirely to the authored pose.
///
/// # Why this lives in the plugin and not in the caller
///
/// The arm solve has to run against the character's REAL rig geometry, which
/// only the plugin has — it reads the live bone transforms to build it (see
/// `solve_foot_ik`'s own `RigGeometry::from_skeleton` call).
///
/// Solving in a caller's system instead means solving against
/// `RigGeometry::default()`, the synthetic T-pose proxy, and the two rigs'
/// arms are not merely different sizes: the proxy's left shoulder sits at
/// x = −0.300 while `puppet_base`'s is at x = +0.212. They are MIRRORED. A
/// world-space target solved on the proxy and retargeted onto the real rig
/// therefore sends the hand to the wrong side of the body — measured, a target
/// at (0.45, 1.15, −0.30) put the left hand at (−0.197, 1.208, +0.147), while
/// the same solve on the real rig is exact to 0.0000 m.
///
/// So a world-space arm target is only meaningful with the real rig in hand,
/// and this component is how a caller expresses one.
///
/// # This works end to end, after three frame bugs stacked on top of each other
///
/// Measured live with a target at world (0.32, 1.15, -0.25): `hand_l` lands at
/// **(0.3200, 1.1500, -0.2500)** and `hand_r` is untouched. Getting there took
/// three independent fixes, recorded because each one masked the next and the
/// visible symptom never resembled the cause.
///
/// 1. **The pose-space convention.** Forward kinematics applied a pose delta in
///    the bone's local frame where the renderer applies it about a world axis.
///    See `rig::accumulate_world_rotations`.
/// 2. **The substitute hips offset**, in this function. `Hips` cannot be read
///    back from the live rig, and the value standing in for it was in the
///    synthetic Y-up frame while the rig's root correction is Z-up — which laid
///    the whole character on its back inside the solver, ankle y = -0.856. The
///    leg IK then reacted *correctly* to that, rotating each toe 113 degrees to
///    rescue a tip it believed was a metre underground, and the only visible
///    sign was feet whose toes pointed at the sky.
/// 3. **The world -> pose rotation**, read from the character entity when the
///    correction it needed sat on a node *below* that entity. See
///    `root_rotation` in this function.
///
/// The misleading part was that (2) and (3) both produce mirror-shaped
/// symptoms, so each looked like the whole story in turn. The loader — the
/// prime suspect for two rounds — turned out to be innocent: its captured bind
/// rotations match the file's bit-for-bit, verified by dumping both.
///
/// # A shape of false progress worth remembering
///
/// Partway through, `hand_l`'s X error read 0.34 mm while the feature was still
/// thoroughly broken. The probe target sat near the character's centreline,
/// where a mirror about X is nearly the identity — the number improved for a
/// reason unrelated to what it appeared to confirm. Screenshots caught what the
/// metric could not.
#[derive(Component, Debug, Clone, Default)]
pub struct AnimArmIk {
    /// Where the left hand should be, in world space. `None` leaves the arm
    /// to the pose.
    pub left: Option<Vec3>,
    /// Where the right hand should be, in world space.
    pub right: Option<Vec3>,
    /// How the two-bone solve behaves.
    pub ik: ArmIkConfig,
    /// How the left hand should be oriented once it arrives.
    ///
    /// Only used when [`ArmIkConfig::aim_hand`] is set.
    pub left_rotation: Option<Quat>,
    /// How the right hand should be oriented once it arrives.
    pub right_rotation: Option<Quat>,
}

/// The ground a character's feet are adapted to.
///
/// Boxed rather than generic so characters in one scene can stand on
/// different things, and so adding this to an entity does not ripple a type
/// parameter through the whole plugin.
#[derive(Component)]
pub struct AnimGround(pub Box<dyn GroundProbe>);

impl Default for AnimGround {
    fn default() -> Self {
        Self(Box::new(FlatGround::default()))
    }
}

/// The continuous procedural motion layered onto a character's target pose.
///
/// Optional: a character without one is driven purely by Stage 1, which is
/// what a cutscene or a precisely-authored pose wants. Pair it with a
/// [`GaitPhase`] — both must be present for the layer to do anything.
#[derive(Component, Debug, Clone, Default)]
pub struct AnimPhaseLayer(pub PhaseLayer);

/// Per-bone spring tuning for one character.
///
/// This is the dial that turns one set of pose data into a heavy brute or a
/// quick duellist without touching the poses themselves.
#[derive(Component, Debug, Clone, Copy)]
pub struct AnimSprings(pub BoneSet<SpringParams>);

impl Default for AnimSprings {
    fn default() -> Self {
        Self(default_springs())
    }
}

impl AnimSprings {
    /// Every bone on the same spring.
    pub fn uniform(params: SpringParams) -> Self {
        Self(BoneSet::splat(params))
    }
}

/// The live spring state, and the pose actually being rendered.
///
/// Inserted automatically for any entity that has an [`AnimTarget`] and a
/// [`HumanoidSkeleton`] but no state yet, so a consumer never has to
/// construct one. Insert it explicitly (via [`AnimPose::settled_on`]) only
/// to start a character already settled in a pose.
#[derive(Component, Debug, Clone, Copy)]
pub struct AnimPose {
    /// Per-joint spring state — the pose the ANIMATION produces, before
    /// any ground adaptation.
    ///
    /// Ground correction is deliberately kept out of this. Writing IK
    /// results back into the spring would make next frame's "animated"
    /// position already corrected, and on sloped ground that compounds: a
    /// raised target moves the foot forward, which samples higher ground,
    /// which raises the target again. Measured on a 0.35 grade, it threw
    /// the legs out horizontally within a few frames.
    pub state: DhoState,
    /// Root translation, carried through to the skeleton's own hip joint.
    pub root_translation: Vec3,
}

impl AnimPose {
    /// A rig already settled in `pose`, with no residual motion — it will
    /// not spring away from it on the first frame.
    pub fn settled_on(pose: &LocalPose) -> Self {
        Self { state: DhoState::settled_on(pose), root_translation: pose.root_translation }
    }

    /// The pose currently being rendered.
    pub fn pose(&self) -> LocalPose {
        self.state.pose(self.root_translation)
    }
}

impl Default for AnimPose {
    fn default() -> Self {
        Self { state: DhoState::AT_REST, root_translation: Vec3::ZERO }
    }
}

/// Drives a character from a hot-reloadable `.pose.ron` asset.
///
/// Attach this alongside (or instead of) setting [`AnimTarget`] by hand.
/// Whenever the file changes on disk, the character springs to the new
/// pose — no restart, and no jump, because the spring simply gets a new
/// target (see [`DhoState::retarget`]).
#[derive(Component, Debug, Clone)]
pub struct AnimTargetAsset(pub Handle<PoseAsset>);

/// Stage 1 of the procedural animation stack: authored poses, driven by
/// per-joint damped harmonic oscillators, written onto a humanoid skeleton.
///
/// Add it once; it animates every entity carrying a [`HumanoidSkeleton`] and
/// an [`AnimTarget`].
///
/// Pose *assets* need [`super::asset::AnimAssetPlugin`] as well; it is kept
/// separate so a consumer building poses in code never pays for the asset
/// machinery.
#[derive(Debug, Default, Clone, Copy)]
pub struct AnimPlugin;

impl Plugin for AnimPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                ensure_anim_pose.in_set(AnimSet::Target),
                // Only runs once pose assets exist, so a consumer that
                // builds poses in code need not add `AnimAssetPlugin`.
                apply_pose_assets
                    .in_set(AnimSet::Target)
                    .run_if(resource_exists::<Assets<PoseAsset>>),
                advance_phase_clocks.in_set(AnimSet::Phase),
                advance_springs.in_set(AnimSet::Spring),
                solve_foot_ik.in_set(AnimSet::Ik),
                write_poses.in_set(AnimSet::Write),
            ),
        )
        .configure_sets(
            Update,
            (
                AnimSet::Target,
                AnimSet::Phase,
                AnimSet::Spring,
                AnimSet::Ik,
                AnimSet::Write,
            )
                .chain(),
        );
    }
}

/// Pushes a loaded (or just-reloaded) pose asset into its character's
/// [`AnimTarget`].
///
/// Reacts to `AssetEvent::{Added, Modified}`, so this is the whole of the
/// hot-reload path: Bevy's `file_watcher` re-parses a changed file, this
/// system notices, and the spring redirects on the next frame.
fn apply_pose_assets(
    mut events: MessageReader<AssetEvent<PoseAsset>>,
    assets: Res<Assets<PoseAsset>>,
    mut rigs: Query<(&AnimTargetAsset, &mut AnimTarget)>,
) {
    for event in events.read() {
        let changed = match event {
            AssetEvent::Added { id } | AssetEvent::Modified { id } => *id,
            _ => continue,
        };

        let Some(asset) = assets.get(changed) else { continue };

        let pose = match asset.to_local_pose() {
            Ok(pose) => pose,
            Err(error) => {
                // A bad edit should report itself and leave the character
                // in its last good pose, not snap it to rest.
                warn!("ignoring an invalid pose asset: {error}");
                continue;
            }
        };

        for (handle, mut target) in &mut rigs {
            if handle.0.id() == changed {
                target.pose = pose;
            }
        }
    }
}

/// Gives any newly-targeted rig its spring state, so consumers only have to
/// insert an [`AnimTarget`].
///
/// A rig that specifies a target on spawn begins **settled on it** rather
/// than springing in from the rest pose — otherwise every character would
/// visibly unfold from a T-pose on its first frame.
/// Rigs that have asked to be animated but have no spring state yet.
type UninitialisedRigs<'w, 's> =
    Query<'w, 's, (Entity, &'static AnimTarget), (With<HumanoidSkeleton>, Without<AnimPose>)>;

fn ensure_anim_pose(mut commands: Commands, rigs: UninitialisedRigs) {
    for (entity, target) in &rigs {
        commands.entity(entity).insert(AnimPose::settled_on(&target.pose));
    }
}

/// Advances each character's phase clocks.
///
/// Split from [`layer_phase_onto_target`] so the clocks tick exactly once
/// per frame regardless of how many things read them.
fn advance_phase_clocks(time: Res<Time>, mut phases: Query<&mut GaitPhase>) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for mut phase in &mut phases {
        phase.advance(dt);
    }
}

/// Everything the spring step reads. The springs and the procedural layer
/// are both optional, so a character can opt into as much or as little of
/// the stack as it wants.
type SpringDrivenRigs<'w, 's> = Query<
    'w,
    's,
    (
        &'static AnimTarget,
        &'static mut AnimPose,
        Option<&'static AnimSprings>,
        Option<&'static GaitPhase>,
        Option<&'static AnimPhaseLayer>,
        // For the real rig, which the phase layer's body sway needs.
        Option<&'static AnimFootIk>,
    ),
>;

/// Integrates every rig's springs toward its target, with any procedural
/// layer composed on top.
///
/// The layer is applied to a **copy** of the target, never to the stored
/// `AnimTarget`. Writing it back would make each frame's sine ride on the
/// previous frame's, compounding a small sway into an ever-growing one —
/// and would corrupt the authored pose a consumer set.
fn advance_springs(time: Res<Time>, mut rigs: SpringDrivenRigs) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    let fallback = AnimSprings::default();

    for (target, mut pose, springs, phase, layer, foot_ik) in &mut rigs {
        let springs = springs.unwrap_or(&fallback);

        let mut goal = target.pose;
        if let (Some(phase), Some(layer)) = (phase, layer) {
            // The body's sway moves the pelvis over the feet, which takes
            // the real rig; before it has bound (or on a rig without foot
            // IK) the layer applies its rotations alone. The synthetic
            // proxy's leg segments are shifted a joint, so swaying on it
            // would turn the wrong bones.
            match foot_ik.and_then(|ik| ik.rig.as_ref()) {
                Some(rig) => layer.0.apply_on(phase, &mut goal, rig),
                None => layer.0.apply(phase, &mut goal),
            }
        }

        pose.state.advance(&goal, &springs.0, dt);
        pose.root_translation = goal.root_translation;
    }
}

/// Everything the IK stage reads and writes per character.
///
/// A named alias rather than an inline tuple: the solve needs the sprung pose,
/// the foot-lock state, the ground, the real skeleton (for rig geometry and
/// live bone positions), the arm targets, and the character's own placement.
type IkRig = (
    &'static AnimPose,
    &'static mut AnimFootIk,
    Option<&'static AnimGround>,
    Option<&'static HumanoidSkeleton>,
    Option<&'static AnimArmIk>,
    Option<&'static GlobalTransform>,
    // The character's own placement, read as this frame's when it has no
    // parent: its `GlobalTransform` is last frame's, and a root motion step
    // or a rise up a slope since would offset every ground sample.
    Option<&'static Transform>,
    Has<ChildOf>,
    // What the feet keep clear of, if anything.
    Option<&'static super::obstacles::AnimObstacles>,
);

/// The geometry of the rig `skeleton` drives, in metres, as the renderer
/// places it: what the gait, foot IK and root motion solve on.
///
/// `local_translation` reads a bone's own rest `Transform.translation`
/// (`None` while it is still loading). The one construction both the live
/// solve and the tests use: a hand-copied twin of it in a test once kept
/// passing while the live rig drifted.
///
/// - **Every bone but the hips**: its local translation, times the
///   armature's scale ([`HumanoidSkeleton::bone_translation_scale`]). A
///   Mixamo rig under Blender's 0.01 node stores centimetres, and the raw
///   values gave `character.glb` a 46 m thigh: it walked at a fraction of
///   the asked speed, sideways, feet sliding ~200 mm.
/// - **The hips**: the rig's own rest offset
///   ([`HumanoidSkeleton::hips_rest_offset`]), never the live transform —
///   `write_pose_to_skeleton` overwrites it every frame, so reading it back
///   would feed the solve its own previous output. In the rig's frame, not
///   this crate's: forward kinematics applies the root's Z-up correction to
///   it, and handing it this crate's Y-up (0, 0.94, 0) once laid the whole
///   character on its back (ankle y = −0.856; the leg IK then rotated each
///   toe 113° to rescue a tip it believed was a metre underground, which
///   rendered as toes pointing at the sky). And at the rig's OWN rest
///   height, not the synthetic 0.94 m: the renderer puts the hips at their
///   real rest plus the root translation, so any other height grounds every
///   foot against the wrong floor — `character.glb` (1.126 m) crouched with
///   its feet 0.186 m in the air.
pub fn live_rig_geometry(skeleton: &HumanoidSkeleton, local_translation: impl Fn(Bone) -> Option<Vec3>) -> RigGeometry {
    let offsets = BoneSet::from_fn(|bone| {
        if bone == Bone::Hips {
            return skeleton.hips_rest_offset();
        }
        local_translation(bone)
            .map(|translation| translation * skeleton.bone_translation_scale())
            .unwrap_or_else(|| bone.t_pose_offset())
    });
    RigGeometry::from_skeleton(skeleton, offsets)
}

/// Plants each character's feet on the ground, and places any reaching hands.
///
/// Runs after the springs, deliberately: IK is a *correction* to the pose
/// the animation produced, so it has to see the final animated result. It
/// writes directly into [`AnimPose`] rather than the target, for the same
/// reason the phase layer does not write back — the correction is recomputed
/// from the ground every frame, and feeding it forward would compound.
fn solve_foot_ik(
    time: Res<Time>,
    mut rigs: Query<IkRig>,
    transforms: Query<&Transform>,
    world_transforms: Query<&GlobalTransform>,
    toe_children: Query<&Children>,
    parents: Query<(), With<ChildOf>>,
) {
    let dt = time.delta_secs();
    let fallback = AnimGround::default();

    for (pose, mut foot_ik, ground, skeleton, arm_ik, root, placed, parented, obstacles) in &mut rigs {
        let ground = ground.unwrap_or(&fallback);
        // Where the pose's frame is in the world, for sampling the ground:
        // its origin at the character's, turned as the character is (what
        // the hips hang from, against how it was bound; not the hips
        // themselves, which carry the walk's pelvic twist). Sampled at pose
        // points instead, a slope ran along the character's own forward
        // whichever way it faced: turned 90° across a 0.2 grade, its feet
        // stood level where one was 4 cm uphill of the other.
        let origin = match (placed, parented) {
            (Some(transform), false) => transform.translation,
            _ => root.map_or(Vec3::ZERO, |global| global.translation()),
        };
        // Hips hung from nothing have no turn to read (a bare rig).
        let turn = skeleton
            .filter(|skeleton| parents.contains(skeleton.entity(Bone::Hips)))
            .and_then(|skeleton| {
                let hips = skeleton.entity(Bone::Hips);
                let world = world_transforms.get(hips).ok()?.rotation();
                let local = transforms.get(hips).ok()?.rotation.normalize();
                Some(((world * local.inverse()) * skeleton.hips_root_rotation().inverse()).normalize())
            })
            .unwrap_or(Quat::IDENTITY);
        let sample_ground = |point: Vec3| {
            ground.0.sample(origin + turn * point).map(|hit| crate::character::anim::ground::GroundHit {
                height: hit.height - origin.y,
                normal: turn.inverse() * hit.normal,
            })
        };
        // The move (the pose's frame, horizontal) that keeps a foot, its toe
        // joint at `toe` pointing `along` with its ankle `ankle_back` behind,
        // clear of the obstacles: asked in the world, as the ground is.
        let frame = turn;
        let keep_clear = |toe: Vec3, along: Vec3, ankle_back: f32| {
            let Some(obstacles) = obstacles else { return Vec3::ZERO };
            let (heel, tip) = super::obstacles::foot_line(toe, along, ankle_back);
            let out = frame.inverse() * obstacles.0.clear(origin + frame * heel, origin + frame * tip, super::obstacles::FOOT_CLEARANCE);
            Vec3::new(out.x, 0.0, out.z)
        };


        // Copied out before the per-foot loop takes a mutable borrow of the
        // lock states, which live in the same component.
        let lock_config = foot_ik.lock;
        let ik_config = foot_ik.ik;
        let turn = foot_ik.turn;
        let landing = foot_ik.landing;

        // The rig this pose is actually driving. Ground height is a
        // property of the world, so solving against it on a proxy rig gives
        // the wrong answer everywhere the two disagree — which is
        // everywhere except flat ground.
        let rig = match skeleton {
            Some(skeleton) => live_rig_geometry(skeleton, |bone| {
                transforms.get(skeleton.entity(bone)).ok().map(|transform| transform.translation)
            }),
            None => RigGeometry::default(),
        };

        // The toe tip, MEASURED off the rig rather than estimated.
        //
        // `RigGeometry::from_skeleton` can only estimate it — a fraction of the
        // ankle-to-toe offset — because `HumanoidSkeleton` has no toe-end bone
        // and deliberately never will (a 23rd bone would invalidate every
        // `[T; 22]`, `Bone::ALL`, and every RON asset for a point that is never
        // rendered). `from_gltf` measures it from the rig's own `ball_leaf_l`,
        // and the two disagree by ~27 degrees on `puppet_base`:
        //
        // ```text
        //   measured   (0, 0.0789,  0.0000)
        //   estimated  (0, 0.0711, -0.0356)
        // ```
        //
        // That matters because `legik::lift_toe_end_out_of_the_ground` rotates
        // the toe to rescue a tip it believes is under the floor. Pointed the
        // wrong way, the estimate reports a penetration that is not there and
        // the toe gets pitched up for nothing — measured live as a tip 0.060 m
        // ABOVE its own joint on flat ground, where every in-crate path keeps
        // it level to within 0.8 mm. That is the "toes point at the sky"
        // symptom.
        //
        // The toe's own CHILD is the tip, on any rig that models one, so this
        // needs no per-rig name table: whatever hangs off the toe joint is by
        // construction the thing the toe points at. A rig with no such child
        // keeps the estimate.
        let rig = match skeleton {
            Some(skeleton) => {
                let mut rig = rig;
                for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
                    let Ok(children) = toe_children.get(skeleton.entity(toe)) else {
                        continue;
                    };
                    let Some(tip) = children.iter().next() else {
                        continue;
                    };
                    let Ok(local) = transforms.get(tip) else { continue };
                    rig = rig.with_toe_end(toe, local.translation * skeleton.bone_translation_scale());
                }
                rig
            }
            None => rig,
        };

        // The character's own placement in the world, for converting
        // world-space arm targets into the frame the pose is solved in.
        //
        // Only the ROTATION is needed: the translation half of the mapping is
        // calibrated from a shoulder's live position instead (see the arm loop),
        // which is exact where reconstructing the hip placement is not.
        //
        // # Read from the live HIPS, not the character entity
        //
        // The character entity is not necessarily the top of the correction
        // chain. `character_gallery` spawns its mesh under a node carrying a
        // 180-degree yaw (`--character-yaw-correction`, default 180) so the
        // model faces the camera, and that node sits BELOW the character entity
        // and ABOVE `pelvis`. Reading the character entity therefore returns
        // identity while every live bone transform carries the yaw, and the
        // world -> pose mapping silently loses it.
        //
        // Measured with the yaw unaccounted for, every bone's x came back
        // negated — `LeftArm` world +0.2106 against pose -0.2104, `LeftUpLeg`
        // +0.1143 against -0.1143, `Head` -0.0140 against +0.0143. That is a
        // 180-degree yaw exactly, and it is what made a left-hand target look
        // like it was driving the right arm: the solve was correct and aimed at
        // a mirrored point.
        //
        // The hips' own world rotation carries every correction between the
        // world and the rig, whatever the asset's nesting happens to be. The
        // rig's own root and hip bind rotations are divided back out because
        // forward kinematics already applies them.
        let root_rotation = match skeleton
            .and_then(|skeleton| world_transforms.get(skeleton.entity(Bone::Hips)).ok())
        {
            Some(hips) => {
                hips.to_scale_rotation_translation().1
                    * (rig.root_rotation * rig.bind_rotations[Bone::Hips]).inverse()
            }
            // No skeleton yet: fall back to the character entity, which is
            // correct whenever nothing sits between it and the hips.
            None => match root {
                Some(global) => global.to_scale_rotation_translation().1,
                None => Quat::IDENTITY,
            },
        };

        let mut solved = pose.pose();
        let pelvis_config = foot_ik.pelvis;

        // Legs posed as authored, untouched (`AnimFootIk::legs_free`); the
        // locks let go, so a stand begins with them afresh.
        if foot_ik.legs_free {
            foot_ik.left = Default::default();
            foot_ik.right = Default::default();
            foot_ik.pelvis_drop = 0.0;
            // The sprung pose lifted clear of the floor: the posed one is
            // (`sitting::clear_floor`), but a foot turning fast near the
            // floor trails it, and kneeling down a toe tip went 47 mm under.
            let under = -super::sitting::lowest_point(&solved, &rig);
            if under > 0.0 {
                solved.root_translation.y += under;
            }
            foot_ik.corrected = Some(solved);
            foot_ik.rig = Some(rig);
            continue;
        }

        // Where the ANIMATION puts each toe, sampled once before any IK
        // runs. Both the ground query and the lock read from this.
        //
        // Sampling from the in-progress solve instead creates a positive
        // feedback loop on any non-flat ground: raising a target moves the
        // foot forward, which samples higher ground, which raises the
        // target again. On a 0.35 grade that diverged within a handful of
        // frames and threw the legs out horizontally — a Left-view
        // screenshot caught it, with every test passing.
        let animated_toes = forward_kinematics_on(&solved, &rig);

        // Pass one: decide where each toe wants to go. No solving yet — the
        // pelvis drop below depends on ALL the targets, and dropping the hips
        // changes every leg's reach, so nothing can be solved until the hip
        // height is settled.
        let mut resolved: [Option<(LegChain, Vec3, crate::character::anim::ground::GroundHit)>;
            2] = [None, None];

        for (slot, (chain, side)) in [
            (LegChain::LEFT, Side::Left),
            (LegChain::RIGHT, Side::Right),
        ]
        .into_iter()
        .enumerate()
        {
            let displaced = foot_ik.displaced[slot];
            let mut animated = animated_toes[chain.toe] + Vec3::new(displaced.x, 0.0, displaced.z);
            if let Some(landing) = landing
                && landing.place
                && landing.left == matches!(side, Side::Left)
            {
                let across = (landing.spot - animated) * landing.strength;
                animated += Vec3::new(across.x, 0.0, across.z);
            }

            let Some(hit) = sample_ground(animated) else {
                // No ground under this foot — over a ledge, say. Leave it
                // following the animation rather than planting it on
                // nothing.
                continue;
            };

            // The toe JOINT is not the contact point. On this rig
            // `LeftToeBase` sits at y = -0.02 in the bind pose — the joint
            // is inside the foot, and the sole is what touches the floor.
            //
            // So "on the ground" means the joint sits at its own offset
            // above the surface, not at the surface itself. Forcing the
            // joint to the surface lifts the whole leg by that offset every
            // frame, which dragged the legs into a visible forward lunge —
            // caught by a Left-view screenshot after every test had passed.
            //
            // Measured from the ANIMATED pose, not the rest pose. The walk
            // articulates the ankle through toe-off and heel-strike, so how
            // far the joint sits above the sole genuinely changes; a fixed
            // rest-pose value pinned every target at `y = +0.0152` while the
            // cycle swung the toe between `-0.038` and `+0.060`, and the leg
            // IK folded the knee to close the gap. See
            // [`toe_contact_offset`] for the measurement and why reading the
            // animated pose cannot feed back on itself.
            let contact_offset = toe_contact_offset(&solved, chain, &rig);
            let surface = hit.height + contact_offset;

            // The body's travel arrives in world axes; the lock works in the
            // pose's, the same rotation the arm targets below go through.
            //
            // And no turn: the pose's frame already turns with the body, so a
            // foot pivoting with it stays where it is there. Rotated by the
            // frame's yaw about the WORLD pivot as well, an anchor held in the
            // pose's frame jumped yaw × the character's distance from the
            // origin: at a wall 7 m out a planted foot flicked 0.55 m each
            // frame of the turn (`a_planted_foot_turning_far_from_the_origin_stays_with_the_body`).
            //
            // Into the pose's frame through what the hips hang from (`frame`),
            // not the live hips, which carry the pose's own pelvic roll: on
            // one leg the balance rolls the pelvis ~4°, and each 0.2 m side
            // step's travel came out 14 mm vertical, lifting every planted
            // foot's anchor 14 mm a step until the feet hovered.
            let turn = Turn { travel: frame.inverse() * turn.travel, yaw_delta: 0.0, pivot: Vec3::ZERO };
            let planted = foot_ik.planted[matches!(side, Side::Right) as usize];
            let lock = match side {
                Side::Left => &mut foot_ik.left,
                _ => &mut foot_ik.right,
            };
            let mut target = lock.update_planted(animated, surface, &lock_config, dt, turn, planted);

            // Kept clear of what the feet must not stand in or swing through
            // (`obstacles`), about the target the leg is solved to: worked out
            // on the walker's pose instead, the sprung leg trailed it 4-7 cm
            // and the clearance had to be 12 cm. A planted foot's lock goes
            // with it: a turn pivots a planted foot about the body, and in
            // front of a chair that carried one into the chair's leg.
            let toe_from_ankle = animated_toes[chain.toe] - animated_toes[chain.ankle];
            let flat = Vec3::new(toe_from_ankle.x, 0.0, toe_from_ankle.z);
            let out = keep_clear(target, flat.normalize_or_zero(), flat.length());
            if out != Vec3::ZERO {
                target += out;
                lock.shift_anchor(out);
            }

            // Never let the sole sink below the surface, even mid-release.
            target.y = target.y.max(surface);

            // A foot being set down clears the floor until it is over its
            // spot, measured on the rendered (sprung) foot.
            if let Some(landing) = landing
                && landing.left == matches!(side, Side::Left)
            {
                let away = Vec3::new(animated.x - landing.spot.x, 0.0, animated.z - landing.spot.z).length();
                target.y = target.y.max(surface + landing.strength * super::transition::landing_lift(away));
            }

            resolved[slot] = Some((chain, target, hit));
        }

        // Pass two: lower the hips if a foot cannot reach, BEFORE any leg is
        // solved. A leg solved against the old hip height would be
        // immediately invalidated by the drop.
        //
        // Only when both feet have targets — with one foot over a ledge there
        // is no shared constraint to satisfy, and dropping the hips for the
        // single grounded foot would crouch the character mid-stride.
        if let [Some((left_chain, left_target, _)), Some((right_chain, right_target, _))] =
            resolved
        {
            let drop = solve_pelvis_drop(
                &solved,
                &rig,
                [(left_chain, left_target), (right_chain, right_target)],
                &pelvis_config,
            );
            apply_pelvis_drop(&mut solved, drop);
            foot_ik.pelvis_drop = drop;
        } else {
            foot_ik.pelvis_drop = 0.0;
        }

        // Pass three: solve each leg against the settled hip height.
        for entry in resolved.into_iter().flatten() {
            let (chain, target, hit) = entry;
            solve_leg_grounded(&mut solved, chain, target, Some(hit), &ik_config, &rig);
        }

        // Pass four: the arms, if this character is reaching for anything.
        //
        // After the legs and the pelvis drop, deliberately: lowering the hips
        // moves both shoulders, so an arm solved first would be aiming from a
        // shoulder position that the drop then invalidates. The legs cannot be
        // affected in turn, because nothing here touches the spine or hips.
        // Each shoulder's live world position, for calibrating the world -> pose
        // mapping below. Read before the arm loop because the loop mutates
        // `solved`, and this must reflect the frame the skeleton is actually in.
        let shoulder_positions: BoneSet<Option<Vec3>> = BoneSet::from_fn(|bone| {
            skeleton.and_then(|skeleton| {
                world_transforms
                    .get(skeleton.entity(bone))
                    .ok()
                    .map(|global| global.translation())
            })
        });

        if let Some(arm_ik) = arm_ik {
            for (chain, target, rotation) in [
                (ArmChain::LEFT, arm_ik.left, arm_ik.left_rotation),
                (ArmChain::RIGHT, arm_ik.right, arm_ik.right_rotation),
            ] {
                let Some(world_target) = target else {
                    continue;
                };

                // Targets are world-space; the solve happens in the FK frame.
                //
                // ROTATION: just the entity's own transform. `rig.root_rotation`
                // is already inside `forward_kinematics_on`'s accumulation, so
                // composing it again here double-applies the Z-up correction —
                // measured, that turned a leg direction of (0, -0.992, -0.126)
                // into (0, -0.126, -0.992), swapping Y and Z outright.
                //
                // ORIGIN: calibrated rather than derived, because the two frames
                // deliberately place the root differently —
                // `write_pose_to_skeleton` puts the hips at
                // `Hips::t_pose_world_position()` while the FK chain uses its own
                // root, which measured as a 0.95 m disagreement in the
                // shoulder's height. The shoulder this solve pivots from has a
                // known position in BOTH frames, so the offset is measured
                // directly and cannot drift out of sync with the writer the way
                // a reconstruction of its hip placement would.
                let rotation_to_pose = root_rotation.inverse();


                let Some(shoulder_world) = shoulder_positions[chain.shoulder] else {
                    // No live transform for this shoulder — the skeleton is
                    // still loading. Leave the arm to the animation rather than
                    // solving against a frame we cannot locate.
                    continue;
                };
                let shoulder_pose = forward_kinematics_on(&solved, &rig)[chain.shoulder];


                // World -> pose: rotate into the pose's axes, then shift so the
                // shoulder coincides.
                let local =
                    rotation_to_pose * (world_target - shoulder_world) + shoulder_pose;

                solve_arm_on(
                    &mut solved,
                    chain,
                    match rotation {
                        Some(hand) => ArmTarget::grip(local, rotation_to_pose * hand),
                        None => ArmTarget::reach(local),
                    },
                    &arm_ik.ik,
                    &rig,
                );
            }
        }

        foot_ik.corrected = Some(solved);
        // Published so a caller posing the gait uses the same rig this
        // solve measured, rather than rebuilding one that can disagree.
        foot_ik.rig = Some(rig);
    }
}

/// Which leg is being solved.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Left,
    Right,
}

/// How far the toe JOINT sits above the lowest point of its own foot, in a
/// given pose.
///
/// This is the quantity a ground target needs: "the joint is on the floor"
/// means the joint sits this far above the surface, because the sole — not
/// the joint — is what touches.
///
/// # Why the pose matters, and why this is not a world height
///
/// This used to be measured in [`LocalPose::REST`] and used directly as a
/// world height, which conflated two different things: the foot's own
/// SHAPE (joint above sole, a fixed property of how the artist bound it)
/// and however high the rest pose happened to hold that foot.
///
/// Those agree only while the foot is at its rest attitude. The walk cycle
/// articulates the ankle through toe-off and heel-strike, so the joint's
/// height above the sole genuinely changes — and pinning the target to the
/// rest value made the animated foot miss it by a wide margin.
///
/// Measured on `puppet_base.gltf` with the old rest-pose value: the target
/// sat at `y = +0.0152` for every frame while the walk cycle swung the toe
/// between `-0.038` and `+0.060`. The leg IK closed that gap by folding
/// the knee, reaching **39 degrees** of anatomical knee angle where a
/// walking human holds 160-175. That is the "grasshopper legs" report.
///
/// # Why this cannot feed back on itself
///
/// It is taken from the ANIMATED pose — the spring's output, before any
/// ground correction — and never from the corrected one. The correction is
/// deliberately kept out of the spring's state (see
/// [`AnimFootIk::corrected`]), so this frame's offset cannot depend on last
/// frame's solve. It is also a difference WITHIN the foot rather than an
/// absolute height, so translating the whole character changes it by zero.
///
/// # The sole, not the joints
///
/// "Lowest point of the foot" is the walk's own sole ([`Sole`]: heel, ball
/// and tip where the bind pose stands them on the floor), carried in the
/// foot's frame. It used to be the lower of the toe joint and the toe tip,
/// which are joints, not sole: on `puppet_base` both sit 15.2 mm above the
/// bind floor, so the offset came out 0 and the IK planted the joint itself
/// on the floor, 15 mm lower than the walk plants the same foot. Standing,
/// the ball rendered at 1.6 mm against the asset's 15.2.
fn toe_contact_offset(pose: &LocalPose, chain: LegChain, rig: &RigGeometry) -> f32 {
    use super::foot::{lowest, Sole};
    use super::rig::offset_from;
    let joint = offset_from(pose, rig, Bone::Hips, chain.toe).y;
    let sole = lowest(&Sole::of(rig, chain.ankle).points(pose, rig));

    // Never negative: a toe joint below its own sole would mean the foot is
    // inverted, and the joint then plants on the surface itself.
    (joint - sole).max(0.0)
}

/// Writes each rig's solved pose onto its skeleton's `Transform`s.
///
/// Runs in `Update`, hence before Bevy's own `PostUpdate` transform
/// propagation — the rig's world positions are derived from these
/// rotations, so they must be in place first.
fn write_poses(
    rigs: Query<(&HumanoidSkeleton, &AnimPose, Option<&AnimFootIk>)>,
    mut transforms: Query<&mut Transform>,
) {
    for (skeleton, pose, foot_ik) in &rigs {
        // Prefer the ground-corrected pose where one exists. It is stored
        // separately from the spring state precisely so it can be rendered
        // without feeding back into next frame's animation.
        let rendered = foot_ik
            .and_then(|ik| ik.corrected)
            .unwrap_or_else(|| pose.pose());

        write_pose_to_skeleton(skeleton, &rendered, &mut transforms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::ground::SlopedGround;
    use crate::character::anim::rig::{forward_kinematics, toe_end_positions};
    use crate::character::skeleton::Bone;
    use std::f32::consts::FRAC_PI_2;

    /// An app with the plugin and one bare rig, ready to step.
    fn app_with_rig(target: LocalPose) -> (App, Entity) {
        let mut app = App::new();
        app.add_plugins(AnimPlugin);
        app.init_resource::<Time>();

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, skeleton) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            world.entity_mut(root).insert((AnimTarget::new(target), AnimSprings::default()));
            let _ = skeleton;
            root
        };

        (app, rig)
    }

    /// Steps the app, advancing `Time` by a fixed frame.
    fn step(app: &mut App, frames: usize) {
        for _ in 0..frames {
            let world = app.world_mut();
            let mut time = world.resource_mut::<Time>();
            time.advance_by(std::time::Duration::from_secs_f32(1.0 / 60.0));
            app.update();
        }
    }

    #[test]
    fn the_plugin_inserts_spring_state_for_a_targeted_rig() {
        // A consumer should only have to insert an AnimTarget.
        let (mut app, rig) = app_with_rig(LocalPose::REST);
        step(&mut app, 1);

        assert!(
            app.world().get::<AnimPose>(rig).is_some(),
            "the plugin must give a targeted rig its own AnimPose",
        );
    }

    #[test]
    fn a_rig_spawned_with_a_pose_starts_settled_in_it() {
        // Otherwise every character visibly unfolds from a T-pose on frame
        // one, which is the sort of thing that is obvious in motion and easy
        // to miss in a still.
        let mut target = LocalPose::REST;
        target.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_2));

        let (mut app, rig) = app_with_rig(target);
        step(&mut app, 1);

        let pose = app.world().get::<AnimPose>(rig).unwrap();
        assert!(
            pose.state.current[Bone::LeftArm]
                .abs_diff_eq(target.rotation(Bone::LeftArm), 1.0e-5),
            "a rig spawned with a pose must begin in it, not spring toward it",
        );
    }

    #[test]
    fn the_solved_pose_reaches_the_skeletons_transforms() {
        // End-to-end: target -> spring -> write-back -> Transform.
        let mut target = LocalPose::REST;
        target.set_rotation(Bone::Head, Quat::from_axis_angle(Vec3::Y, 0.6));

        let (mut app, rig) = app_with_rig(target);
        step(&mut app, 1);

        let skeleton = app.world().get::<HumanoidSkeleton>(rig).unwrap().clone();
        let head = app.world().get::<Transform>(skeleton.entity(Bone::Head)).unwrap();

        let expected = skeleton.rest_rotation(Bone::Head) * target.rotation(Bone::Head);
        assert!(
            head.rotation.abs_diff_eq(expected, 1.0e-4),
            "the Head transform should carry the solved pose, got {:?}",
            head.rotation,
        );
    }

    #[test]
    fn changing_the_target_springs_the_rig_toward_it() {
        let (mut app, rig) = app_with_rig(LocalPose::REST);
        step(&mut app, 1);

        let mut moved = LocalPose::REST;
        moved.set_rotation(Bone::RightArm, Quat::from_axis_angle(Vec3::Z, 1.0));
        app.world_mut().entity_mut(rig).insert(AnimTarget::new(moved));

        step(&mut app, 120);

        let pose = app.world().get::<AnimPose>(rig).unwrap();
        assert!(
            pose.state.current[Bone::RightArm].abs_diff_eq(moved.rotation(Bone::RightArm), 1.0e-4),
            "the rig should have sprung to its new target, got {:?}",
            pose.state.current[Bone::RightArm],
        );
    }

    #[test]
    fn a_rig_without_explicit_springs_still_animates() {
        // AnimSprings is optional; the plugin falls back to sane defaults.
        let mut target = LocalPose::REST;
        target.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.4));

        let mut app = App::new();
        app.add_plugins(AnimPlugin);
        app.init_resource::<Time>();

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, _) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            // Deliberately NO AnimSprings.
            world.entity_mut(root).insert(AnimTarget::new(target));
            root
        };

        step(&mut app, 2);

        let pose = app.world().get::<AnimPose>(rig).unwrap();
        assert!(
            pose.state.current[Bone::Spine].abs_diff_eq(target.rotation(Bone::Spine), 1.0e-5),
            "a rig with no AnimSprings should still be driven by the defaults",
        );
    }

    #[test]
    fn a_pose_asset_drives_the_character_and_reloading_it_redirects_the_spring() {
        // The whole hot-reload contract, end to end: an asset becomes the
        // target, and CHANGING that asset moves the character again without
        // a restart and without a jump.
        use crate::character::anim::asset::{AnimAssetPlugin, AuthoredBone, AuthoredRotation};

        let mut app = App::new();
        app.add_plugins((bevy::asset::AssetPlugin::default(), AnimAssetPlugin, AnimPlugin));
        app.init_resource::<Time>();

        // A pose asset built in memory — the loader's own parsing is
        // covered in `asset`'s tests; this is about the runtime wiring.
        let handle = {
            let mut assets = app.world_mut().resource_mut::<Assets<PoseAsset>>();
            let mut pose = PoseAsset::default();
            pose.bones.insert(
                "LeftArm".into(),
                AuthoredBone {
                    swing: AuthoredRotation { axis: (0.0, 0.0, 1.0), degrees: 60.0 },
                    twist_degrees: 0.0,
                },
            );
            assets.add(pose)
        };

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, _) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            world
                .entity_mut(root)
                .insert((AnimTarget::default(), AnimTargetAsset(handle.clone())));
            root
        };

        // Long enough for the spring to settle: the exponential form only
        // approaches its target, so a tolerance this tight needs several
        // half-lives (1 s leaves ~1.3 degrees on the default arm spring).
        step(&mut app, 180);

        let sixty_degrees = Quat::from_axis_angle(Vec3::Z, 60f32.to_radians());
        let settled = app.world().get::<AnimPose>(rig).unwrap().state.current[Bone::LeftArm];
        assert!(
            settled.abs_diff_eq(sixty_degrees, 1.0e-3),
            "the character should have sprung to the asset's pose, got {settled:?}",
        );

        // Now "edit the file": mutating the asset raises `Modified`, which
        // is exactly what `file_watcher` does after a save on disk.
        {
            let mut assets = app.world_mut().resource_mut::<Assets<PoseAsset>>();
            let mut pose = assets.get_mut(&handle).unwrap();
            pose.bones.get_mut("LeftArm").unwrap().swing.degrees = -30.0;
        }

        step(&mut app, 180);

        let reloaded = app.world().get::<AnimPose>(rig).unwrap().state.current[Bone::LeftArm];
        assert!(
            reloaded.abs_diff_eq(Quat::from_axis_angle(Vec3::Z, -30f32.to_radians()), 1.0e-3),
            "editing the asset should redirect the character, got {reloaded:?}",
        );
    }

    #[test]
    fn an_invalid_pose_asset_leaves_the_character_in_its_last_good_pose() {
        // A typo mid-edit must not snap the character to rest — that would
        // make the hot-reload loop hostile to work in.
        use crate::character::anim::asset::{AnimAssetPlugin, AuthoredBone, AuthoredRotation};

        let mut app = App::new();
        app.add_plugins((bevy::asset::AssetPlugin::default(), AnimAssetPlugin, AnimPlugin));
        app.init_resource::<Time>();

        let handle = {
            let mut assets = app.world_mut().resource_mut::<Assets<PoseAsset>>();
            let mut pose = PoseAsset::default();
            pose.bones.insert(
                "LeftArm".into(),
                AuthoredBone {
                    swing: AuthoredRotation { axis: (0.0, 0.0, 1.0), degrees: 60.0 },
                    twist_degrees: 0.0,
                },
            );
            assets.add(pose)
        };

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, _) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            world
                .entity_mut(root)
                .insert((AnimTarget::default(), AnimTargetAsset(handle.clone())));
            root
        };

        step(&mut app, 60);
        let good = app.world().get::<AnimTarget>(rig).unwrap().pose.rotation(Bone::LeftArm);

        // Introduce a bone name that does not exist on the rig.
        {
            let mut assets = app.world_mut().resource_mut::<Assets<PoseAsset>>();
            let mut pose = assets.get_mut(&handle).unwrap();
            pose.bones.clear();
            pose.bones.insert("LeftArmm".into(), AuthoredBone::default());
        }

        step(&mut app, 10);

        let after = app.world().get::<AnimTarget>(rig).unwrap().pose.rotation(Bone::LeftArm);
        assert_eq!(
            after, good,
            "an invalid edit must be ignored, leaving the last good pose in place",
        );
    }

    /// A bare rig carrying a phase layer and its clock.
    fn app_with_idle_rig() -> (App, Entity) {
        let mut app = App::new();
        app.add_plugins(AnimPlugin);
        app.init_resource::<Time>();

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, _) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            world.entity_mut(root).insert((
                AnimTarget::new(LocalPose::REST),
                AnimSprings::default(),
                GaitPhase::default(),
                AnimPhaseLayer(PhaseLayer::standing_idle()),
            ));
            root
        };

        (app, rig)
    }

    #[test]
    fn a_character_with_a_phase_layer_never_goes_completely_still() {
        // The point of Stage 2: a settled spring would otherwise stop dead,
        // which reads as a mannequin.
        let (mut app, rig) = app_with_idle_rig();

        // Let the spring settle first, so any motion after this is the
        // oscillators' doing and not the initial convergence.
        step(&mut app, 240);

        let mut samples = Vec::new();
        for _ in 0..120 {
            step(&mut app, 1);
            // The neck: the idle's weight shift moves the pelvis now (with a
            // rig to move it over), and the neck rides the same clock.
            samples.push(app.world().get::<AnimPose>(rig).unwrap().state.current[Bone::Neck]);
        }

        let largest_change = samples
            .windows(2)
            .map(|pair| pair[0].angle_between(pair[1]))
            .fold(0.0f32, f32::max);

        assert!(
            largest_change > 1.0e-5,
            "a character with an idle layer should keep moving after settling, but the \
             spine changed by at most {largest_change} rad per frame",
        );
    }

    #[test]
    fn the_phase_layer_never_writes_back_into_the_authored_target() {
        // The compounding bug this design guards against: applying the
        // layer to the stored target would make each frame's sine ride on
        // the previous frame's, growing a small sway without bound.
        let (mut app, rig) = app_with_idle_rig();

        step(&mut app, 600);

        let target = app.world().get::<AnimTarget>(rig).unwrap();
        for &bone in Bone::ALL.iter() {
            assert_eq!(
                target.pose.rotation(bone),
                Quat::IDENTITY,
                "{} drifted in the stored target — the layer is being written back",
                bone.name(),
            );
        }
    }

    #[test]
    fn idle_motion_stays_within_a_believable_amplitude_over_a_long_run() {
        // The observable consequence of the previous test, stated as the
        // property a viewer would actually notice: a subtle idle must not
        // grow into a lurch after a minute.
        let (mut app, rig) = app_with_idle_rig();

        step(&mut app, 240);

        let mut furthest = 0.0f32;
        for _ in 0..3600 {
            step(&mut app, 1);
            let pose = app.world().get::<AnimPose>(rig).unwrap();
            for &bone in Bone::ALL.iter() {
                furthest =
                    furthest.max(pose.state.current[bone].angle_between(Quat::IDENTITY));
            }
        }

        assert!(
            furthest < 0.2,
            "after a minute of idling the largest deviation should still be subtle, but \
             reached {furthest} rad",
        );
    }

    #[test]
    fn a_character_without_a_phase_layer_settles_completely() {
        // Stage 2 must be genuinely optional — a precisely-authored pose
        // has to be able to hold perfectly still.
        let (mut app, rig) = app_with_rig(LocalPose::REST);

        step(&mut app, 300);

        let pose = app.world().get::<AnimPose>(rig).unwrap();
        for &bone in Bone::ALL.iter() {
            assert_eq!(
                pose.state.current[bone],
                Quat::IDENTITY,
                "{} should be perfectly still without a phase layer",
                bone.name(),
            );
        }
    }

    #[test]
    fn walking_faster_makes_the_idle_motion_quicken() {
        // Speed coupling, observed end to end rather than on the clock
        // alone: the rendered motion itself must quicken.
        let measure_activity = |speed: f32| {
            let (mut app, rig) = app_with_idle_rig();
            app.world_mut().entity_mut(rig).insert(GaitPhase { speed, ..Default::default() });

            step(&mut app, 240);

            let mut total = 0.0;
            let mut previous =
                app.world().get::<AnimPose>(rig).unwrap().state.current[Bone::Neck];
            for _ in 0..120 {
                step(&mut app, 1);
                let current =
                    app.world().get::<AnimPose>(rig).unwrap().state.current[Bone::Neck];
                total += previous.angle_between(current);
                previous = current;
            }
            total
        };

        let standing = measure_activity(0.0);
        let walking = measure_activity(1.4);

        assert!(
            walking > standing * 2.0,
            "walking should visibly quicken the procedural motion: {walking} vs \
             {standing} rad of travel over the same window",
        );
    }

    /// A rig standing on `ground`, with foot IK enabled.
    fn app_with_grounded_rig(ground: impl GroundProbe) -> (App, Entity) {
        let mut app = App::new();
        app.add_plugins(AnimPlugin);
        app.init_resource::<Time>();

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, _) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            world.entity_mut(root).insert((
                AnimTarget::new(crate::character::anim::poses::relaxed_stand()),
                AnimSprings::default(),
                AnimFootIk::default(),
                AnimGround(Box::new(ground)),
            ));
            root
        };

        (app, rig)
    }

    /// The RENDERED toe positions — the ground-corrected pose where one
    /// exists, matching what `write_poses` actually puts on screen.
    fn toe_positions(app: &App, rig: Entity) -> (Vec3, Vec3) {
        let pose = app
            .world()
            .get::<AnimFootIk>(rig)
            .and_then(|ik| ik.corrected)
            .unwrap_or_else(|| app.world().get::<AnimPose>(rig).unwrap().pose());

        let positions = forward_kinematics(&pose);
        (positions[LegChain::LEFT.toe], positions[LegChain::RIGHT.toe])
    }

    /// Where each foot's LOWEST point sits — the part that actually touches
    /// the floor.
    ///
    /// The honest reference for "is this foot planted". Predicting the toe
    /// JOINT's height instead needs a contact offset, and taking that offset
    /// from the rest pose while the character stands in `relaxed_stand` is
    /// how these tests used to be written — it was close enough to pass
    /// while the offset was a fixed number, and became a 0.019 m error the
    /// moment the offset started tracking the pose it is actually solving.
    fn sole_heights(app: &App, rig: Entity) -> (f32, f32) {
        let pose = app
            .world()
            .get::<AnimFootIk>(rig)
            .and_then(|ik| ik.corrected)
            .unwrap_or_else(|| app.world().get::<AnimPose>(rig).unwrap().pose());

        // The walk's sole, the same one the IK plants (`toe_contact_offset`).
        use crate::character::anim::foot::{lowest, Sole};
        let geometry = RigGeometry::default();
        let hips = forward_kinematics_on(&pose, &geometry)[Bone::Hips].y;
        let sole = |ankle| hips + lowest(&Sole::of(&geometry, ankle).points(&pose, &geometry));
        (sole(LegChain::LEFT.ankle), sole(LegChain::RIGHT.ankle))
    }

    /// [`app_with_grounded_rig`] on the real rig, `puppet_base`: one entity
    /// per bone at the asset's own local translation, and its skeleton, so
    /// the IK solves on the live rig geometry as the game does. The
    /// synthetic rig's leg joints are shifted by one (its `LeftUpLeg` is the
    /// knee), so a leg folding on it is not a leg folding.
    fn app_with_real_rig(ground: impl GroundProbe) -> (App, Entity, RigGeometry) {
        use crate::character::anim::gltf_rig::{parsed_rig, puppet_base, real_skeleton};
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let mut app = App::new();
        app.add_plugins(AnimPlugin);
        app.init_resource::<Time>();
        let asset = puppet_base();
        let world = app.world_mut();
        let entities: std::collections::HashMap<Bone, Entity> = Bone::ALL
            .iter()
            .map(|&bone| (bone, world.spawn(Transform::from_translation(asset.offsets[bone])).id()))
            .collect();
        let skeleton = real_skeleton(&parsed_rig(), entities);
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &asset);
        let rig = world
            .spawn((
                AnimTarget::new(stood),
                AnimSprings::default(),
                AnimFootIk::default(),
                AnimGround(Box::new(ground)),
                skeleton,
            ))
            .id();
        (app, rig, asset)
    }

    /// Each foot's heel, ball and tip contacts, world heights, on the rig
    /// the IK solved on.
    fn real_soles(app: &App, rig: Entity) -> [[f32; 3]; 2] {
        use crate::character::anim::foot::Sole;
        let ik = app.world().get::<AnimFootIk>(rig).unwrap();
        let (pose, geometry) = (ik.corrected.unwrap(), ik.rig.clone().unwrap());
        let hips = forward_kinematics_on(&pose, &geometry)[Bone::Hips].y;
        [Bone::LeftFoot, Bone::RightFoot].map(|ankle| Sole::of(&geometry, ankle).points(&pose, &geometry).map(|p| hips + p.y))
    }

    #[test]
    fn a_turned_character_stands_on_the_slope_where_its_feet_are() {
        // Turned 90° across a slope, away from the origin: one foot stands
        // uphill of the other. Sampled at pose points, the slope ran along
        // the character's own forward and both feet stood level, 2 cm in
        // and out of the ground.
        use crate::character::anim::foot::Sole;
        use crate::character::anim::gltf_rig::parsed_rig;
        let grade = 0.2;
        let ground = SlopedGround { height: 0.0, grade };
        let (mut app, rig, _) = app_with_real_rig(SlopedGround { height: 0.0, grade });
        app.add_plugins(TransformPlugin);
        let position = Vec3::new(1.5, grade * 3.0, -3.0);
        let turn = Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
        let hips = app.world().get::<HumanoidSkeleton>(rig).unwrap().entity(Bone::Hips);
        let armature =
            app.world_mut().spawn((Transform::from_rotation(parsed_rig().hips_parent_rest_world_rotation), ChildOf(rig))).id();
        app.world_mut().entity_mut(rig).insert(Transform::from_translation(position).with_rotation(turn));
        app.world_mut().entity_mut(hips).insert(ChildOf(armature));
        step(&mut app, 120);

        let ik = app.world().get::<AnimFootIk>(rig).unwrap();
        let (pose, geometry) = (ik.corrected.unwrap(), ik.rig.clone().unwrap());
        let hips = forward_kinematics_on(&pose, &geometry)[Bone::Hips];
        let mut heights = vec![];
        for ankle in [Bone::LeftFoot, Bone::RightFoot] {
            // Each foot's lowest contact, against the ground under it.
            let lowest = Sole::of(&geometry, ankle)
                .points(&pose, &geometry)
                .map(|p| hips + p)
                .into_iter()
                .min_by(|a, b| a.y.total_cmp(&b.y))
                .unwrap();
            let under = ground.sample(position + turn * lowest).unwrap().height - position.y;
            assert!(
                (lowest.y - under).abs() < 0.005,
                "{ankle:?}'s sole is at {:.4}, the ground under it at {under:.4}",
                lowest.y
            );
            heights.push(under);
        }
        assert!((heights[0] - heights[1]).abs() > 0.02, "the feet should stand at different heights: {heights:?}");
    }

    #[test]
    fn a_planted_foot_turning_far_from_the_origin_stays_with_the_body() {
        // The turn handed to the foot locks (`AnimFootIk::turn`) pivots about
        // the character's WORLD position, but the locks hold their anchors in
        // the pose's frame, which already turns with the body. Rotated about
        // a world point metres away, an anchor jumped a frame's yaw × that
        // distance: turning at a wall 7 m out, a planted foot flicked 0.55 m
        // back and forth every frame and the hips with it. A planted foot
        // pivots with the body: in the pose's frame it stays where it is.
        use crate::character::anim::footlock::Turn;
        let (mut app, rig, _) = app_with_real_rig(FlatGround::default());
        let position = Vec3::new(6.0, 0.0, 6.0);
        app.world_mut().entity_mut(rig).insert(Transform::from_translation(position));
        step(&mut app, 120);
        let soles = |app: &App| {
            let ik = app.world().get::<AnimFootIk>(rig).unwrap();
            let (pose, geometry) = (ik.corrected.unwrap(), ik.rig.clone().unwrap());
            let joints = forward_kinematics_on(&pose, &geometry);
            [joints[Bone::LeftFoot], joints[Bone::RightFoot]]
        };
        let (rate, mut yaw, mut worst) = (2.4f32, 0.0f32, 0.0f32);
        let mut before = soles(&app);
        for _ in 0..60 {
            let delta = rate / 60.0;
            yaw += delta;
            app.world_mut().get_mut::<Transform>(rig).unwrap().rotation = Quat::from_rotation_y(yaw);
            app.world_mut().get_mut::<AnimFootIk>(rig).unwrap().turn = Turn { pivot: position, yaw_delta: delta, travel: Vec3::ZERO };
            step(&mut app, 1);
            let now = soles(&app);
            for (a, b) in now.iter().zip(&before) {
                worst = worst.max(a.distance(*b));
            }
            before = now;
        }
        assert!(worst < 0.005, "a planted ankle moved {:.1} mm in one frame of the turn", worst * 1e3);
    }

    #[test]
    fn a_planted_foot_keeps_its_height_as_a_rolled_body_moves_sideways() {
        // The body's travel reaches the locks in the pose's frame. Turned
        // into it through the live hips, it carried their roll: on one leg
        // the balance rolls the pelvis ~4°, each 0.2 m side step's travel
        // came out 14 mm vertical, and the planted feet rose 14 mm a step
        // until they hovered. Through what the hips hang from, it stays
        // level.
        use crate::character::anim::gltf_rig::parsed_rig;
        let (mut app, rig, asset) = app_with_real_rig(FlatGround::default());
        // The live hierarchy: the hips under an armature node, transforms
        // propagated, so the frame is read off the live hips as it is live.
        app.add_plugins(TransformPlugin);
        let hips = app.world().get::<HumanoidSkeleton>(rig).unwrap().entity(Bone::Hips);
        let armature = app.world_mut().spawn((Transform::from_rotation(parsed_rig().hips_parent_rest_world_rotation), ChildOf(rig))).id();
        app.world_mut().entity_mut(rig).insert(Transform::IDENTITY);
        app.world_mut().entity_mut(hips).insert(ChildOf(armature));
        step(&mut app, 120);
        let mut rolled = app.world().get::<AnimTarget>(rig).unwrap().pose;
        rolled.rotations[Bone::Hips] = super::super::rig::delta_after_world_turn(&rolled, &asset, Bone::Hips, Quat::from_axis_angle(asset.forward(), 0.07));
        app.world_mut().get_mut::<AnimTarget>(rig).unwrap().pose = rolled;
        step(&mut app, 60);
        let heights = |app: &App| {
            let ik = app.world().get::<AnimFootIk>(rig).unwrap();
            let joints = forward_kinematics_on(&ik.corrected.unwrap(), &ik.rig.clone().unwrap());
            [joints[Bone::LeftToeBase].y, joints[Bone::RightToeBase].y]
        };
        // 6 cm each way, within the legs' reach of the feet left where they
        // are (live, the balance moves them in the pose too). One way the
        // old anchor sank, and the ground clamp hid it; the other it rose
        // 4.2 mm.
        let mut at = Vec3::ZERO;
        for way in [1.0, -1.0, -1.0, 1.0] {
            let before = heights(&app);
            for _ in 0..20 {
                let travel = Vec3::new(0.003 * way, 0.0, 0.0);
                at += travel;
                app.world_mut().get_mut::<Transform>(rig).unwrap().translation = at;
                let mut ik = app.world_mut().get_mut::<AnimFootIk>(rig).unwrap();
                ik.turn = crate::character::anim::footlock::Turn { pivot: at, yaw_delta: 0.0, travel };
                // Down, as a standing balance says its feet are.
                ik.planted = [true; 2];
                step(&mut app, 1);
            }
            let ik = app.world().get::<AnimFootIk>(rig).unwrap();
            assert!(ik.left.anchor().is_some() && ik.right.anchor().is_some(), "both feet should stay locked");
            for (now, then) in heights(&app).iter().zip(before) {
                assert!((now - then).abs() < 1.0e-3, "a planted toe went {:+.1} mm up as the body moved 6 cm sideways", (now - then) * 1e3);
            }
        }
    }

    #[test]
    fn a_real_foot_stands_whole_on_raised_ground() {
        // The synthetic fixture's heel sank ~13 cm into ground raised 25 cm
        // under a body held still: its "knee" is an ankle stub, which flips
        // 180° and pitches the foot. A real leg just bends its knee — so on
        // the real rig the whole sole, heel to tip, rests on the plane.
        for height in [0.0, 0.1, 0.25] {
            let (mut app, rig, _) = app_with_real_rig(FlatGround { height });
            step(&mut app, 120);
            for (side, [heel, ball, tip]) in ["left", "right"].into_iter().zip(real_soles(&app, rig)) {
                let lowest = heel.min(ball).min(tip);
                assert!(
                    (lowest - height).abs() < 0.005,
                    "on ground at {height} m the {side} sole's lowest contact is at {lowest:.4} \
                     (heel {heel:.4}, ball {ball:.4}, tip {tip:.4})"
                );
                assert!(heel > height - 0.005, "on ground at {height} m the {side} heel sank to {heel:.4}");
            }
        }
    }

    #[test]
    fn a_landing_foot_is_held_up_until_it_is_over_its_spot() {
        // A stop's last swing, judged on the rendered foot: the legs'
        // springs lag a fast swing by ~10 cm, and a foot set down by its
        // TARGET crept its last ~2 cm along the floor.
        use crate::character::anim::transition::landing_lift;
        let toe_after = |landing: Option<Landing>| {
            let (mut app, rig) = app_with_grounded_rig(FlatGround { height: 0.0 });
            step(&mut app, 60);
            app.world_mut().get_mut::<AnimFootIk>(rig).unwrap().landing = landing;
            step(&mut app, 1);
            let pose = app.world().get::<AnimFootIk>(rig).and_then(|ik| ik.corrected).unwrap();
            let animated = app.world().get::<AnimPose>(rig).unwrap().pose();
            (forward_kinematics(&pose)[Bone::LeftToeBase], forward_kinematics(&animated)[Bone::LeftToeBase])
        };
        let (planted, animated) = toe_after(None);

        // 6 cm short of its spot: held up by the lift for 6 cm.
        let spot = animated + Vec3::new(0.0, 0.0, 0.06);
        let (held, _) = toe_after(Some(Landing { left: true, spot, strength: 1.0, place: false }));
        let raised = held.y - planted.y;
        assert!(
            (raised - landing_lift(0.06)).abs() < 1.0e-3,
            "6 cm from its spot the toe rose {:.1} mm, the lift is {:.1}",
            raised * 1e3,
            landing_lift(0.06) * 1e3
        );

        // Over its spot: down.
        let (landed, _) = toe_after(Some(Landing { left: true, spot: animated, strength: 1.0, place: false }));
        assert!((landed.y - planted.y).abs() < 1.0e-4, "over its spot the toe stayed {:.2} mm up", (landed.y - planted.y) * 1e3);
    }

    #[test]
    fn a_placed_landing_carries_the_free_foot_onto_its_spot_and_locks_it_there() {
        // A balance step's spot is planned: the sprung foot, left to itself,
        // landed 1.9 cm wide of it and was locked there. Placed, the foot is
        // carried across by the landing's strength before its lock sees it,
        // and locks on the spot.
        let toe_after = |landing: Option<Landing>, frames: usize| {
            let (mut app, rig) = app_with_grounded_rig(FlatGround { height: 0.0 });
            app.world_mut().get_mut::<AnimFootIk>(rig).unwrap().landing = landing;
            step(&mut app, frames);
            let pose = app.world().get::<AnimFootIk>(rig).and_then(|ik| ik.corrected).unwrap();
            forward_kinematics(&pose)[Bone::LeftToeBase]
        };
        let free = toe_after(None, 30);
        let spot = free + Vec3::new(0.03, 0.0, 0.0);
        for (strength, expect) in [(1.0, spot), (0.5, free + Vec3::new(0.015, 0.0, 0.0))] {
            let placed = toe_after(Some(Landing { left: true, spot, strength, place: true }), 30);
            let off = Vec3::new(placed.x - expect.x, 0.0, placed.z - expect.z).length();
            assert!(off < 1.0e-3, "at strength {strength} the toe is {:.1} mm off", off * 1e3);
        }
        // Not placed: where the animation has it.
        let left = toe_after(Some(Landing { left: true, spot, strength: 1.0, place: false }), 30);
        assert!(Vec3::new(left.x - free.x, 0.0, left.z - free.z).length() < 1.0e-3);
    }

    #[test]
    fn foot_ik_plants_both_toes_on_flat_ground() {
        let (mut app, rig) = app_with_grounded_rig(FlatGround { height: 0.0 });
        step(&mut app, 120);

        // The physical property: the SOLE sits on the floor. Asserting a
        // predicted joint height instead needs a contact offset, which is
        // exactly the quantity under test.
        let (left, right) = sole_heights(&app, rig);

        for (name, sole) in [("left", left), ("right", right)] {
            assert!(
                (sole - 0.0).abs() < 0.01,
                "the {name} sole should rest on the ground (y=0), but sat at y={sole}",
            );
        }
    }

    #[test]
    fn foot_ik_follows_a_raised_ground_plane() {
        // The whole reason `GroundProbe` exists: the superseded module
        // hardcoded y = 0 in three places, so a character could only ever
        // stand at the world origin's height.
        let (mut app, rig) = app_with_grounded_rig(FlatGround { height: 0.25 });
        step(&mut app, 120);

        // The BALL contact, which is what the IK plants. The whole sole is
        // not on the plane here, and that is this SYNTHETIC rig, not the
        // IK: its leg joints are shifted by one (its `LeftUpLeg` is the
        // knee, its "shin" a 0.07 m ankle stub), so folding 25 cm flips the
        // stub 180° and pitches the foot, heel ~13 cm under the plane. A
        // real leg bends its knee: on `puppet_base` the whole sole rests on
        // the plane at every height — see
        // `a_real_foot_stands_whole_on_raised_ground`.
        let (left, right) = {
            use crate::character::anim::foot::Sole;
            let pose = app.world().get::<AnimFootIk>(rig).and_then(|ik| ik.corrected).unwrap();
            let geometry = RigGeometry::default();
            let hips = forward_kinematics_on(&pose, &geometry)[Bone::Hips].y;
            let ball = |ankle| hips + Sole::of(&geometry, ankle).points(&pose, &geometry)[1].y;
            (ball(LegChain::LEFT.ankle), ball(LegChain::RIGHT.ankle))
        };
        let expected = 0.25;

        // 2 cm, not 1.
        //
        // This fixture leaves the body at the world origin and raises the
        // ground 0.25 m under it, so the feet have to come a quarter of a
        // metre UP toward the hips — the leg has to fold that far, and this
        // rig is authored at critical extension with little fold to spare.
        // Measured, the sole settles at 0.232: **1.8 cm short**, and the
        // pelvis drop is already doing what it can.
        //
        // The tolerance is widened to what the rig actually achieves rather
        // than the shortfall being hidden, because the property under test
        // is "the feet follow a ground plane that is not at y = 0" — which
        // they do — and not "the leg can fold 0.25 m", which it cannot. On
        // flat ground, where the fold is within reach, the same measurement
        // lands inside 1 cm.
        for (name, sole) in [("left", left), ("right", right)] {
            assert!(
                (sole - expected).abs() < 0.02,
                "the {name} sole should rest on the raised ground (expected y={expected}), \
                 but sat at y={}",
                sole,
            );
        }
    }

    #[test]
    fn probe_knee_bend_direction_through_the_pipeline() {
        // SIGNED, not magnitude. Every earlier check measured the angle
        // between thigh and shin, which is identical whichever way the knee
        // folds — so a backward-bending knee passed them all.
        use crate::character::anim::gait::{walk_pose_on, GaitParams};
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::forward_kinematics_on;
        use crate::character::anim::stance::stance;

        let rig = gltf_rig::puppet_base();
        let params = GaitParams::default();

        let report = |label: &str, pose: &LocalPose| {
            let k = forward_kinematics_on(pose, &rig);
            let fwd = (k[Bone::LeftToeBase] - k[Bone::LeftFoot]).normalize_or_zero();
            let thigh = k[Bone::LeftLeg] - k[Bone::LeftUpLeg];
            let shin = k[Bone::LeftFoot] - k[Bone::LeftLeg];
            // A human knee: the thigh goes FORWARD from the hip and the
            // shin comes BACK from the knee.
            println!(
                "  {label:28} thigh.fwd={:+.4} shin.fwd={:+.4}  => {}",
                thigh.dot(fwd),
                shin.dot(fwd),
                if thigh.dot(fwd) > 0.0 && shin.dot(fwd) < 0.0 {
                    "human"
                } else if thigh.dot(fwd) < 0.0 && shin.dot(fwd) > 0.0 {
                    "BACKWARD (grasshopper)"
                } else {
                    "straight-ish"
                },
            );
        };

        report("REST", &LocalPose::REST);
        report("stance(REST)", &stance(&LocalPose::REST));
        report("relaxed_stand", &crate::character::anim::poses::relaxed_stand());
        for i in 0..4 {
            let phase = i as f32 / 4.0;
            let pose = walk_pose_on(phase, &params, &stance(&LocalPose::REST), &rig);
            report(&format!("walk {phase:.2} on stance"), &pose);
        }

        // The measurement that actually decides it: which side of the
        // hip-to-ankle line the knee sits on. Positive = forward = human.
        println!("\n  knee offset from the hip-ankle line, along the rig's forward:");
        let signed = |label: &str, pose: &LocalPose| {
            let k = forward_kinematics_on(pose, &rig);
            let fwd = (k[Bone::LeftToeBase] - k[Bone::LeftFoot]).normalize_or_zero();
            let hip = k[Bone::LeftUpLeg];
            let ankle = k[Bone::LeftFoot];
            let knee = k[Bone::LeftLeg];
            let mid = (hip + ankle) * 0.5;
            println!(
                "  {label:28} {:+.4}  ({})",
                (knee - mid).dot(fwd),
                if (knee - mid).dot(fwd) > 0.0 { "human" } else { "BACKWARD" },
            );
        };
        signed("REST", &LocalPose::REST);
        signed("stance(REST)", &stance(&LocalPose::REST));
        for i in 0..4 {
            let phase = i as f32 / 4.0;
            let pose = walk_pose_on(phase, &params, &stance(&LocalPose::REST), &rig);
            signed(&format!("walk {phase:.2} on stance"), &pose);
        }
    }

    #[test]
    fn the_contact_offset_tracks_the_pose_rather_than_the_rest_attitude() {
        // The quantity is "how far the toe joint sits above the sole",
        // which is a property of the foot's ATTITUDE — and the walk
        // articulates the ankle, so it genuinely changes through the cycle.
        //
        // Taking it from `LocalPose::REST` instead pinned every ground
        // target at one height while the animated foot moved through a
        // 98 mm vertical range, and the leg IK folded the knee to close the
        // gap: 39 degrees of anatomical knee angle against a walking
        // human's 160-175. That is the "grasshopper legs" report.
        use crate::character::anim::gait::{walk_pose_on, GaitParams};
        use crate::character::anim::gltf_rig;
        use crate::character::anim::stance::stance;

        let rig = gltf_rig::puppet_base();
        let base = stance(&LocalPose::REST);
        let params = GaitParams::default();

        let rest = toe_contact_offset(&LocalPose::REST, LegChain::LEFT, &rig);

        let mut lowest = f32::MAX;
        let mut highest = f32::MIN;
        for i in 0..32 {
            let phase = i as f32 / 32.0;
            let pose = walk_pose_on(phase, &params, &base, &rig);
            let offset = toe_contact_offset(&pose, LegChain::LEFT, &rig);
            lowest = lowest.min(offset);
            highest = highest.max(offset);
        }

        // It is never negative — the joint cannot sit below its own sole.
        assert!(
            lowest >= 0.0,
            "a contact offset came out negative ({lowest}), which would plant the \
             joint below the sole",
        );

        // And it genuinely varies, which is the whole point: a fixed value
        // cannot describe a foot that rolls through toe-off and
        // heel-strike.
        assert!(
            highest - lowest > 0.01,
            "the contact offset only varied by {} m across the cycle ({lowest} to \
             {highest}) — if it is effectively constant then reading it from the \
             animated pose buys nothing and this test is not measuring what it claims",
            highest - lowest,
        );

        // Measured against the sole (`foot::Sole`), a flat foot is where the
        // toe joint sits lowest above it: rolling onto the heel or the toes
        // only raises it. So the rest attitude sits at the bottom of the
        // cycle's range (15.2 mm, the asset's own joint-above-floor; the
        // walk's flattest foot 15.5), and a fixed rest value would have been
        // wrong by up to ~95 mm the other way. Against the old joints-only
        // "sole" the rest value landed mid-range instead, and the IK planted
        // the joint itself on the floor.
        assert!(
            (rest - 0.0152).abs() < 5.0e-4,
            "a flat foot's toe joint should sit at the asset's 15.2 mm above its sole, got {rest}",
        );
        assert!(
            rest <= lowest + 1.0e-3,
            "the flat rest foot ({rest}) should be the cycle's lowest offset ({lowest} to {highest})",
        );
    }

    #[test]
    fn foot_ik_never_lets_a_toe_sink_through_the_ground() {
        // Ground penetration is among the most visible possible artefacts,
        // and it must hold on every frame, not just once settled.
        let (mut app, rig) = app_with_grounded_rig(FlatGround { height: 0.1 });

        // Measured at the SOLE — the lowest point of the foot — which is
        // what "penetration" actually means. Checking the toe joint instead
        // needs a contact offset, and an offset taken from the wrong pose
        // turns this into an assertion about joint placement rather than a
        // penetration check.
        let floor = 0.1;

        for _ in 0..180 {
            step(&mut app, 1);
            let (left, right) = sole_heights(&app, rig);

            assert!(
                left >= floor - 1.0e-3 && right >= floor - 1.0e-3,
                "a sole sank below the ground (floor {floor}): left y={left}, \
                 right y={right}",
            );
        }
    }

    #[test]
    fn a_planted_foot_is_held_clear_of_an_obstacle_and_the_other_left_alone() {
        // A table leg (4 cm square) put 4 cm in front of where the left toe
        // stands, under the foot: the foot IK moves that foot off it until
        // the toe keeps FOOT_CLEARANCE from it, the planted lock with it, and
        // leaves the right foot where it was.
        use crate::character::anim::obstacles::{AnimObstacles, Footprint, Footprints, FOOT_CLEARANCE};
        let (mut app, rig) = app_with_grounded_rig(FlatGround::default());
        step(&mut app, 120);
        let (left, right) = toe_positions(&app, rig);
        let leg = Footprint { middle: Vec3::new(left.x, 0.0, left.z + 0.06), forward: Vec3::Z, size: Vec2::splat(0.04) };
        app.world_mut().entity_mut(rig).insert(AnimObstacles(Box::new(Footprints(vec![leg]))));
        step(&mut app, 60);
        let (moved_left, moved_right) = toe_positions(&app, rig);
        let local = leg.local(moved_left);
        let gap = (local.abs() - leg.size * 0.5).max(Vec2::ZERO).length();
        assert!(gap > FOOT_CLEARANCE - 0.005, "the left toe is {gap:.3} m from the leg");
        assert!(app.world().get::<AnimFootIk>(rig).unwrap().left.is_locked(), "the left foot let go");
        assert!(moved_right.distance(right) < 1.0e-3, "the right foot moved {:.4} m", moved_right.distance(right));
        // Held there: no creeping once clear.
        step(&mut app, 30);
        assert!(toe_positions(&app, rig).0.distance(moved_left) < 1.0e-3, "the left foot crept");
    }

    #[test]
    fn a_planted_toe_does_not_slide() {
        // THE property of the whole stage. Once locked, a toe's world
        // position must hold still — that is what "not sliding" means.
        let (mut app, rig) = app_with_grounded_rig(FlatGround::default());

        // Let the springs settle and the feet lock.
        step(&mut app, 120);
        let (mut previous_left, mut previous_right) = toe_positions(&app, rig);

        let mut worst = 0.0f32;
        for _ in 0..120 {
            step(&mut app, 1);
            let (left, right) = toe_positions(&app, rig);

            worst = worst
                .max((left - previous_left).length())
                .max((right - previous_right).length());

            previous_left = left;
            previous_right = right;
        }

        assert!(
            worst < 1.0e-3,
            "a planted toe should not move, but travelled {worst} m in a single frame",
        );
    }

    #[test]
    fn foot_ik_lifts_the_feet_onto_a_raised_slope() {
        // A ramp is only interesting where it is actually raised. In
        // `relaxed_stand` both toes sit near z = 0, where a slope through
        // the origin is essentially flat — an earlier version of this test
        // compared against the surface under the SOLVED toe and was really
        // just re-measuring flat ground.
        //
        // Offsetting the ramp puts both feet on a genuinely raised
        // surface, which is the property worth asserting: the character
        // stands ON the ground it is given, wherever that is.
        let grade = 0.4;
        let base = 0.2;
        let (mut app, rig) =
            app_with_grounded_rig(SlopedGround { height: base, grade });
        step(&mut app, 180);

        let (left, right) = toe_positions(&app, rig);

        // Asserted on the CONTACT POINT, not on the toe joint.
        //
        // An earlier version compared the joint's height against
        // `surface + rest_toe_height(..)`, which mixes two different
        // references: `rest_toe_height` now reports the lowest contact point
        // (the virtual tip, 0.01 m below the joint on this rig), while the
        // measurement is of the joint. It also assumes the foot is level, and
        // a foot aligned to a 0.4-grade slope is not — the tilt lifts the
        // joint another ~0.03 m. The old formula put the joint BELOW the
        // surface, which is not a thing a standing foot does.
        //
        // What actually defines "standing on it" is that the foot touches the
        // surface and nothing sinks through.
        for (name, toe) in [("left", left), ("right", right)] {
            let surface = base + grade * -toe.z;

            assert!(
                toe.y > surface - 1.0e-3,
                "the {name} toe joint should sit at or above the sloped surface \
                 ({surface}), but sat at y={}",
                toe.y,
            );
            assert!(
                toe.y < surface + 0.08,
                "...and should rest ON it rather than hovering, but sat {} m above",
                toe.y - surface,
            );
        }
    }

    #[test]
    fn the_world_to_pose_mapping_recovers_a_yaw_correction_above_the_hips() {
        // `solve_foot_ik` derives its world -> pose rotation from the live HIPS
        // rather than the character entity, because a rig can carry correction
        // nodes between the two. `character_gallery` does exactly that: a
        // 180-degree yaw so the model faces the camera.
        //
        // This pins the arithmetic that recovers it. Reading the character
        // entity instead returns identity, the mapping loses the yaw, and every
        // world-space arm target lands mirrored through the character's own
        // centreline — which renders as a left-hand target driving the right
        // arm, and is how this was found.
        use crate::character::anim::gltf_rig;

        let rig = gltf_rig::puppet_base();

        for yaw in [0.0, std::f32::consts::PI, 0.7] {
            let correction = Quat::from_rotation_y(yaw);

            // What the live hips' `GlobalTransform` would read: the correction
            // node, composed with everything forward kinematics itself applies.
            let hips_world = correction * rig.root_rotation * rig.bind_rotations[Bone::Hips];

            // The production expression, verbatim.
            let recovered =
                hips_world * (rig.root_rotation * rig.bind_rotations[Bone::Hips]).inverse();

            let error = 2.0
                * (recovered.inverse() * correction).w.abs().clamp(0.0, 1.0).acos();
            assert!(
                error < 1.0e-4,
                "a {:.1}-degree correction came back as {recovered:?}, {:.3} degrees off",
                yaw.to_degrees(),
                error.to_degrees(),
            );

            // And the consequence, stated on a point rather than a quaternion:
            // mapping a world target into the pose frame has to undo the
            // correction, not ignore it. A `q * q.inverse() * v` round trip
            // would pass no matter what `recovered` held, so this compares
            // against the correction applied by hand.
            let world_point = Vec3::new(0.32, 1.15, -0.25);
            let mapped = recovered.inverse() * world_point;
            let expected = correction.inverse() * world_point;

            assert!(
                (mapped - expected).length() < 1.0e-5,
                "a {:.1}-degree correction mapped {world_point:?} to {mapped:?}, \
                 not {expected:?}",
                yaw.to_degrees(),
            );
        }
    }

    #[test]
    fn on_a_slope_the_feet_tilt_to_match_and_no_toe_tip_sinks_through() {
        // The end-to-end property for both grounding corrections, through the
        // real plugin rather than the solver alone.
        //
        // Before this, `GroundHit::normal` was computed by `SlopedGround`,
        // carried through the whole stack, and read by nothing: a foot on a
        // ramp stayed perfectly level and drove its heel or toe into the
        // surface.
        let grade = 0.4_f32;
        let base = 0.2;
        let (mut app, rig) = app_with_grounded_rig(SlopedGround { height: base, grade });
        step(&mut app, 180);

        let world = app.world();
        let skeleton = world.get::<HumanoidSkeleton>(rig).expect("a skeleton");
        let pose = world
            .get::<AnimFootIk>(rig)
            .expect("foot ik")
            .corrected
            .expect("a corrected pose");

        let offsets = BoneSet::from_fn(|bone| {
            if bone == Bone::Hips {
                return bone.t_pose_offset();
            }
            world
                .get::<Transform>(skeleton.entity(bone))
                .map(|t| t.translation)
                .unwrap_or_else(|| bone.t_pose_offset())
        });
        let geometry = RigGeometry::from_skeleton(skeleton, offsets);

        let (left_tip, right_tip) = toe_end_positions(&pose, &geometry);
        let positions = forward_kinematics_on(&pose, &geometry);

        for (name, tip, toe, ankle) in [
            ("left", left_tip, positions[Bone::LeftToeBase], positions[Bone::LeftFoot]),
            ("right", right_tip, positions[Bone::RightToeBase], positions[Bone::RightFoot]),
        ] {
            // The foot is actually tilted: on a rising slope the tip must sit
            // HIGHER than the toe joint behind it, which a level foot never
            // does (at rest the tip is 0.01 m BELOW the joint).
            assert!(
                tip.y > toe.y,
                "the {name} foot should pitch up to match the rising slope, but its \
                 tip ({}) sits below its toe joint ({})",
                tip.y,
                toe.y,
            );

            let _ = ankle;
        }

        // The tilt alignment adds must be the SLOPE's, not an arbitrary
        // amount — measured against the SAME ground with alignment disabled.
        //
        // Comparing against flat ground instead (as an earlier version did)
        // measures far more than alignment: flat ground also changes where the
        // leg solve puts the ankle, so the difference conflates the two and
        // reported 0.740 rad for a correction that was in fact exactly the
        // intended 0.304. The pose's own authored pitch is deliberately
        // preserved, so only a same-ground A/B isolates what alignment did.
        let (mut unaligned_app, unaligned_rig) =
            app_with_grounded_rig(SlopedGround { height: base, grade });
        unaligned_app
            .world_mut()
            .get_mut::<AnimFootIk>(unaligned_rig)
            .expect("foot ik")
            .ik
            .normal_alignment = 0.0;
        step(&mut unaligned_app, 180);

        let unaligned_world = unaligned_app.world();
        let unaligned_skeleton =
            unaligned_world.get::<HumanoidSkeleton>(unaligned_rig).expect("a skeleton");
        let unaligned_pose = unaligned_world
            .get::<AnimFootIk>(unaligned_rig)
            .expect("foot ik")
            .corrected
            .expect("a corrected pose");

        let unaligned_offsets = BoneSet::from_fn(|bone| {
            if bone == Bone::Hips {
                return bone.t_pose_offset();
            }
            unaligned_world
                .get::<Transform>(unaligned_skeleton.entity(bone))
                .map(|t| t.translation)
                .unwrap_or_else(|| bone.t_pose_offset())
        });
        let unaligned_geometry =
            RigGeometry::from_skeleton(unaligned_skeleton, unaligned_offsets);

        let unaligned_positions = forward_kinematics_on(&unaligned_pose, &unaligned_geometry);
        let (unaligned_tip, _) = toe_end_positions(&unaligned_pose, &unaligned_geometry);

        let unaligned_sole =
            (unaligned_tip - unaligned_positions[Bone::LeftToeBase]).normalize();
        let aligned_sole = (left_tip - positions[Bone::LeftToeBase]).normalize();

        let added_tilt = unaligned_sole.angle_between(aligned_sole);
        let expected = grade.atan() * LegIkConfig::default().normal_alignment;

        assert!(
            (added_tilt - expected).abs() < 0.08,
            "alignment should add the slope's own tilt scaled by the {} blend \
             ({expected} rad), but added {added_tilt} rad",
            LegIkConfig::default().normal_alignment,
        );

        // Nothing sinks through the surface.
        //
        // Measured: without alignment the tip sits 0.015 m UNDER the slope;
        // with it, 0.049 m clear. Clearing is correct here rather than
        // suspicious — on a rising slope the foot pitches toe-up, so the tip
        // lifts while the heel stays down and carries the contact.
        for (name, tip) in [("left", left_tip), ("right", right_tip)] {
            let surface_at_tip = base + grade * -tip.z;
            assert!(
                tip.y > surface_at_tip - 0.005,
                "the {name} toe tip sank {} m below the surface",
                surface_at_tip - tip.y,
            );
        }
    }

    /// Ground that sits lower under one side than the other — a foot in a
    /// dip, which is the case the pelvis adjustment exists for.
    #[derive(Clone, Copy)]
    struct SteppedGround {
        /// Height for `x < 0` (the rig's left).
        left: f32,
        /// Height everywhere else.
        right: f32,
    }

    impl GroundProbe for SteppedGround {
        fn sample(
            &self,
            position: Vec3,
        ) -> Option<crate::character::anim::ground::GroundHit> {
            Some(crate::character::anim::ground::GroundHit::flat(
                if position.x < 0.0 { self.left } else { self.right },
            ))
        }
    }

    #[test]
    fn a_foot_in_a_shallow_dip_lowers_the_hips() {
        // The pelvis adjustment doing its job end to end.
        //
        // A 5 cm dip is within `FootLockConfig::max_contact_height` (0.08 m),
        // so the foot genuinely tracks the lower ground, its target goes past
        // what the leg can reach, and the hips come down to meet it.
        let (mut app, rig) = app_with_grounded_rig(SteppedGround { left: -0.05, right: 0.0 });
        step(&mut app, 180);

        let drop = app.world().get::<AnimFootIk>(rig).expect("foot ik").pelvis_drop;

        assert!(drop > 0.0, "a foot in a 5 cm dip should lower the hips");
        assert!(
            drop <= PelvisConfig::default().max_drop + 1.0e-6,
            "...but never past the {} m dinosaur cap, got {drop}",
            PelvisConfig::default().max_drop,
        );
    }

    #[test]
    fn ground_below_the_contact_height_is_a_ledge_not_a_dip() {
        // The other side of the same boundary, and a documented behaviour
        // worth pinning: a foot only reaches for ground within
        // `max_contact_height`. Past that the surface is a ledge, the foot
        // keeps following the animation, and there is no shortfall for the
        // pelvis to correct.
        //
        // Stated because the naive expectation is the opposite — that a
        // deeper hole needs MORE crouching, not none.
        for depth in [0.12_f32, 0.25, 3.0] {
            let (mut app, rig) =
                app_with_grounded_rig(SteppedGround { left: -depth, right: 0.0 });
            step(&mut app, 180);

            let drop = app.world().get::<AnimFootIk>(rig).expect("foot ik").pelvis_drop;

            assert_eq!(
                drop, 0.0,
                "a {depth} m drop is past the contact height and should read as a \
                 ledge, but produced a {drop} m pelvis drop",
            );
        }
    }

    #[test]
    fn level_ground_never_lowers_the_hips() {
        // The control, and the one that caught a real defect: the rig's legs
        // are authored at exactly critical extension, so a hip socket at
        // y = 0.49 is asked to span 0.5008 to an ankle target the foot's own
        // thickness puts at y = -0.01. That 0.0108 m shortfall is permanent
        // and present on every surface, so a correction without a deadband
        // above it lowers the hips forever — the dinosaur, reached from the
        // other direction.
        let (mut app, rig) = app_with_grounded_rig(FlatGround::default());
        step(&mut app, 180);

        assert_eq!(
            app.world().get::<AnimFootIk>(rig).expect("foot ik").pelvis_drop,
            0.0,
            "flat ground should need no pelvis drop at all",
        );
    }

    #[test]
    fn the_character_keeps_standing_height_over_any_ground() {
        // "Avoid the dinosaur" as an observable property rather than an
        // internal number: whatever the ground does, the hips stay at
        // standing height.
        // Read from the solved pose rather than a `GlobalTransform`: this
        // harness never runs Bevy's transform propagation, so every global
        // transform reads as the identity and a test trusting one would pass
        // or fail for reasons unrelated to the animation.
        let rest_hips = forward_kinematics_on(&LocalPose::REST, &RigGeometry::default())
            [Bone::Hips]
            .y;

        for depth in [0.0_f32, 0.05, 0.12, 3.0] {
            let (mut app, rig) =
                app_with_grounded_rig(SteppedGround { left: -depth, right: 0.0 });
            step(&mut app, 180);

            let pose = app
                .world()
                .get::<AnimFootIk>(rig)
                .expect("foot ik")
                .corrected
                .expect("a corrected pose");

            let hips =
                forward_kinematics_on(&pose, &RigGeometry::default())[Bone::Hips].y;

            assert!(
                hips > rest_hips - PelvisConfig::default().max_drop - 1.0e-4,
                "over a {depth} m dip the hips dropped from {rest_hips} to {hips}, \
                 past the {} m cap — the character is crouching",
                PelvisConfig::default().max_drop,
            );
        }
    }

    #[test]
    fn foot_ik_does_not_diverge_on_a_slope() {
        // THE regression for a real runaway. Sampling the ground from the
        // in-progress solve (rather than the animated pose), or writing the
        // correction back into the spring, creates positive feedback on any
        // non-flat ground: a raised target moves the foot forward, which
        // samples higher ground, which raises the target again.
        //
        // Measured on a 0.35 grade it threw the legs out horizontally
        // within a few frames, with every unit test still passing.
        let (mut app, rig) = app_with_grounded_rig(SlopedGround { height: 0.0, grade: 0.35 });

        for _ in 0..300 {
            step(&mut app, 1);
            let (left, right) = toe_positions(&app, rig);

            for (name, toe) in [("left", left), ("right", right)] {
                assert!(
                    toe.length() < 2.0,
                    "the {name} toe diverged to {toe:?} — the ground sample is feeding \
                     back into its own target",
                );
            }
        }
    }

    #[test]
    fn a_character_without_foot_ik_is_left_unplanted() {
        // Stage 3 must be optional in the same way Stage 2 is — an
        // airborne character or a cutscene wants the authored pose
        // untouched.
        let (mut app, rig) = app_with_rig(crate::character::anim::poses::relaxed_stand());
        step(&mut app, 60);

        assert!(
            app.world().get::<AnimFootIk>(rig).is_none(),
            "no foot IK should be added without being asked for",
        );
    }

    #[test]
    fn a_missing_ground_leaves_the_foot_following_the_animation() {
        // Over a ledge there is nothing to plant on, and forcing a lock
        // there would pin a foot to empty space.
        struct NoGround;
        impl GroundProbe for NoGround {
            fn sample(&self, _: Vec3) -> Option<crate::character::anim::ground::GroundHit> {
                None
            }
        }

        let (mut app, rig) = app_with_grounded_rig(NoGround);
        step(&mut app, 60);

        let foot_ik = app.world().get::<AnimFootIk>(rig).unwrap();
        assert!(
            !foot_ik.left.is_locked() && !foot_ik.right.is_locked(),
            "a foot over empty space must not lock",
        );
    }

    #[test]
    fn an_untargeted_rig_is_left_alone() {
        // The plugin must not animate skeletons that did not ask for it —
        // a scene may hold rigs driven by something else entirely.
        let mut app = App::new();
        app.add_plugins(AnimPlugin);
        app.init_resource::<Time>();

        let rig = {
            let world = app.world_mut();
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, world);
            let (root, _) = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(world);
            root
        };

        step(&mut app, 3);

        assert!(
            app.world().get::<AnimPose>(rig).is_none(),
            "a skeleton with no AnimTarget must not be given spring state",
        );
    }
}
