//! Wiring the active ragdoll into avian.
//!
//! Kept apart from [`super::ragdoll`] — which is plain functions over plain
//! values — so the control law stays testable without a physics world, and
//! apart from [`super::plugin`] so a consumer that wants only the kinematic
//! stack never links a physics dependency it does not use.

use avian3d::prelude::*;
use bevy::prelude::*;

use super::anthropometry;
use super::foot::{Sole, SoleBox};
use super::plugin::{AnimFootIk, AnimPose, AnimSet};
use super::ragdoll::{joint_torque_at, JointLimits, Ragdoll};
use super::math::quat_ext::neighborhood;
use super::rig::{BoneSet, LocalPose, RigGeometry};
use crate::character::skeleton::{Bone, HumanoidSkeleton};

/// The stages the ragdoll adds to the frame.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RagdollSet {
    /// Apply this frame's [`RagdollHit`]s, and advance recovery from
    /// earlier ones.
    Hit,
    /// Publish the kinematic pose as each joint's world-space target.
    PublishTargets,
    /// Read the simulated bodies back and blend them into the pose.
    ReadBack,
}

/// Each joint's world-space target orientation, written by the kinematic
/// stack and read inside the physics schedule.
///
/// Stored on the simulated body rather than looked up through the
/// character, so the torque system is a flat per-body query — which is what
/// lets it run inside `PhysicsSchedule` without touching the rest of the
/// stack.
/// Reflected so it is readable over BRP. That is not incidental: the
/// difference between "the target is wrong" and "the controller cannot
/// reach a correct target" is invisible from outside, and being able to
/// query the live value settled exactly that question once already.
#[derive(Component, Debug, Clone, Copy, Reflect)]
#[reflect(Component)]
pub struct JointTarget {
    /// Which bone this body stands in for.
    pub bone: Bone,
    /// Where it should be, in world space.
    pub target: Quat,
    /// The character this belongs to, so the torque system can read its
    /// tuning.
    pub character: Entity,
}

/// Drives a simulated skeleton toward the kinematic pose.
///
/// Requires `PhysicsPlugins` and [`super::plugin::AnimPlugin`]. Add it only
/// for characters that need physical reactions — a purely kinematic
/// character should not pay for a physics body per bone.
#[derive(Debug, Default, Clone, Copy)]
pub struct AnimRagdollPlugin;

impl Plugin for AnimRagdollPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<JointTarget>();

        add_hit_systems(app);
        app.configure_sets(Update, RagdollSet::Hit.before(RagdollSet::PublishTargets));

        app.add_systems(
            Update,
            publish_joint_targets
                .in_set(RagdollSet::PublishTargets)
                .after(AnimSet::Write),
        );

        // Read the simulation back onto the pose, then re-write the
        // skeleton from it.
        //
        // Ordered AFTER the physics writeback so it sees this frame's
        // solved rotations rather than last frame's, and it re-runs the
        // skeleton write itself because `AnimSet::Write` has already
        // happened by then — without that, the blended pose would sit in
        // `AnimPose` and reach the renderer a frame late.
        app.add_systems(
            PostUpdate,
            (read_back_simulated_pose, write_simulated_pose)
                .chain()
                .in_set(RagdollSet::ReadBack)
                .after(PhysicsSystems::Writeback)
                .before(TransformSystems::Propagate),
        );
    }

    /// The torque system is registered in `finish` rather than `build`.
    ///
    /// Reaching into `PhysicsSchedule` during `build` forces it into
    /// existence before avian has finished its own setup, and avian's
    /// diagnostics registration guards against duplicates with
    /// `is_resource_added` — which only sees same-tick additions. The
    /// result is a physics world missing resources its own systems
    /// require. Registering afterwards leaves avian's ordering intact.
    fn finish(&self, app: &mut App) {
        // Inside the physics schedule, before the solver.
        //
        // `apply_torque` accumulates an acceleration that avian applies
        // over one physics step and then clears. Running this in `Update`
        // instead would apply the same stale torque once per substep —
        // verified against avian's own source, where the accumulator is
        // documented as cleared after each step.
        add_physics_step_systems(app);
    }
}

/// A blow landing on one bone of a ragdolled character.
///
/// Write one with `MessageWriter<RagdollHit>`; [`AnimRagdollPlugin`] does
/// the rest. The struck limb is shoved, goes slack for a moment, and pulls
/// itself back to the animation as its strength returns — with no flinch
/// clip and no state machine.
///
/// # Why a velocity, not an impulse
///
/// The bodies [`spawn_ragdoll`] builds carry avian's default collider
/// density of 1 kg/m³, so a forearm weighs about three grams. An impulse in
/// N·s would mean nothing a caller could tune: a 1 N·s punch would launch
/// that forearm at hundreds of metres per second. The PD controller already
/// sidesteps mass by being acceleration-shaped (see `apply_joint_torques`),
/// and this follows it — `velocity` is the speed change the struck body's
/// centre receives, converted to an impulse with the body's own mass. The
/// same hit then reads the same on any rig, whatever its bodies weigh.
#[derive(Message, Debug, Clone, Copy, PartialEq)]
pub struct RagdollHit {
    /// The character carrying the [`Ragdoll`].
    pub character: Entity,
    /// The bone struck. A bone with no body of its own — a hand, the head —
    /// passes the blow to its nearest simulated ancestor, which is what a
    /// real blow to the head does to the neck.
    pub bone: Bone,
    /// Speed change given to the struck body's centre, in m/s, world space.
    pub velocity: Vec3,
    /// Where it lands, in world space. Off-centre hits also spin the body.
    /// `None` strikes the body's centre.
    pub point: Option<Vec3>,
    /// Fraction of the struck joint's strength knocked out, `0..=1`. See
    /// [`Ragdoll::stun`] for why a hit needs this to be visible at all.
    pub stun: f32,
}

impl RagdollHit {
    /// A hit at the struck body's centre that fully stuns its joint.
    pub fn new(character: Entity, bone: Bone, velocity: Vec3) -> Self {
        Self { character, bone, velocity, point: None, stun: 1.0 }
    }

    /// The same hit, landing at `point` rather than the body's centre.
    pub fn at_point(self, point: Vec3) -> Self {
        Self { point: Some(point), ..self }
    }

    /// The same hit, knocking out `stun` of the joint's strength.
    pub fn with_stun(self, stun: f32) -> Self {
        Self { stun, ..self }
    }
}

/// Registers everything hits need. Shared by the plugin and the headless
/// test harness, so the two cannot drift apart — the harness already
/// hand-registers the read-back, and keeping that in step by hand is the
/// maintenance cost its own comment warns about.
fn add_hit_systems(app: &mut App) {
    app.add_message::<RagdollHit>().add_systems(
        Update,
        (
            apply_ragdoll_hits,
            release_falling_roots,
            rest_fallen_ragdolls,
            rise_fallen_ragdolls,
            recover_from_hits,
            support_own_weight,
        )
            .chain()
            .in_set(RagdollSet::Hit),
    );
}

/// When a fallen ragdoll's bodies may sleep: below 0.1 m/s and 0.3 rad/s
/// (avian's default is 0.15 and 0.15) for avian's `TimeToSleep`.
///
/// A body at rest on the floor is never still in the solver: measured on
/// `puppet_base`, six or seven of its bodies kept spinning at 0.05-0.18
/// rad/s about ever-changing axes, right at the default bound, so it never
/// slept, and that jitter walked the whole body 9 mm/s across the floor.
/// Asleep, it stops (0.0 mm over 3 s).
pub const FALLEN_SLEEP: SleepThreshold = SleepThreshold { linear: 0.1, angular: 0.3 };

/// How slow every body of a fallen ragdoll must stay, and for how long,
/// before it is put to rest: m/s, rad/s, seconds.
///
/// Sleeping on its own (`FALLEN_SLEEP`) is not enough. Some landings rest
/// in a pose the solver never quite holds: live, a hips-up `puppet_base`
/// slid 4 mm/s and a `character.glb` 6-25 mm/s for as long as they lay
/// there, while the same fall, landing differently, slept. The frame's
/// physics-step count varies live, so which one happens varies run to run.
pub const REST_SPEED: f32 = 0.05;
/// See [`REST_SPEED`].
pub const REST_SPIN: f32 = 0.5;
/// See [`REST_SPEED`].
pub const REST_SECONDS: f32 = 1.0;

/// Puts a fallen ragdoll to sleep once all of it has stayed slow
/// ([`REST_SPEED`], [`REST_SPIN`]) for [`REST_SECONDS`], and marks it
/// [`super::ragdoll::Fall::at_rest`]: settled, for whatever hands the
/// body back to animation.
fn rest_fallen_ragdolls(
    mut commands: Commands,
    mut characters: Query<&mut Ragdoll>,
    bodies: Query<(&LinearVelocity, &AngularVelocity, Has<Sleeping>)>,
    time: Res<Time>,
) {
    for mut ragdoll in &mut characters {
        let Some(fall) = ragdoll.fall else { continue };
        if fall.at_rest || fall.root_offset.is_none() {
            continue;
        }
        let states: Vec<_> = ragdoll.bodies.iter().filter_map(|(_, body)| body.and_then(|body| bodies.get(body).ok())).collect();
        let asleep = states.iter().all(|(_, _, sleeping)| *sleeping);
        let slow = states.iter().all(|(linear, angular, sleeping)| {
            *sleeping || (linear.0.length() < REST_SPEED && angular.0.length() < REST_SPIN)
        });
        let still_for = if slow { fall.still_for + time.delta_secs() } else { 0.0 };
        let at_rest = asleep || still_for >= REST_SECONDS;
        if at_rest && !asleep && let Some(hips) = ragdoll.bodies[Bone::Hips] {
            commands.queue(SleepBody(hips));
        }
        if let Some(fall) = ragdoll.fall.as_mut() {
            fall.still_for = still_for;
            fall.at_rest = at_rest;
        }
    }
}

/// Advances every rise ([`Ragdoll::get_up`], H3), and ends it.
///
/// While rising the foot locks are kept free: the character entity followed
/// the body across the floor without them knowing, and locked they would
/// pull the standing feet back to where the fall began.
///
/// A rise ends the frame after its blend reached 1, so the skeleton has been
/// drawn in the animated pose: each body is set onto its drawn bone (at its
/// [`Ragdoll::body_offsets`]), still and awake; the root is pinned again;
/// the fall's joint damping and sleep bounds go; the ragdoll is no longer
/// falling.
#[allow(clippy::too_many_arguments)]
fn rise_fallen_ragdolls(
    mut commands: Commands,
    mut characters: Query<(&mut Ragdoll, &HumanoidSkeleton, Option<&mut AnimFootIk>)>,
    joints: Query<(Entity, &SphericalJoint)>,
    bodies: Query<(&Position, &Rotation, &JointTarget)>,
    transforms: Query<&Transform>,
    live: TransformHelper,
    time: Res<Time>,
) {
    for (mut ragdoll, skeleton, foot_ik) in &mut characters {
        let Some(super::ragdoll::Fall { rise: Some(rise), .. }) = ragdoll.fall else { continue };
        if let Some(mut foot_ik) = foot_ik {
            foot_ik.left = Default::default();
            foot_ik.right = Default::default();
        }
        // First frame: how it lies, which way it will face, and the keys.
        if rise.lying.is_none() {
            let (Some(Ok((hips_at, _, _))), Some(Ok((head_at, _, _))), Some(Ok((_, chest, chest_target)))) = (
                ragdoll.bodies[Bone::Hips].map(|b| bodies.get(b)),
                ragdoll.bodies[Bone::Head].map(|b| bodies.get(b)),
                ragdoll.bodies[Bone::Spine2].map(|b| bodies.get(b)),
            ) else {
                continue;
            };
            let world = rig_geometry(skeleton, &transforms, &live);
            let forward = world.forward();
            // The chest's forward now: the standing pose's, carried by the
            // turn from its target to where its body lies.
            let chest_forward = chest.0 * chest_target.target.inverse() * forward;
            let lying = if chest_forward.y > 0.0 { super::getup::Lying::FaceUp } else { super::getup::Lying::FaceDown };
            // Face down it rises toward its head; face up, sitting up, toward
            // its feet.
            let spine = Vec3::new(head_at.0.x - hips_at.0.x, 0.0, head_at.0.z - hips_at.0.z).normalize_or_zero();
            let heading = if lying == super::getup::Lying::FaceUp { -spine } else { spine };
            let flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
            let turn = if heading == Vec3::ZERO || flat == Vec3::ZERO {
                0.0
            } else {
                flat.cross(heading).y.atan2(flat.dot(heading))
            };
            let rig = super::plugin::live_rig_geometry(skeleton, |bone| {
                transforms.get(skeleton.entity(bone)).ok().map(|transform| transform.translation)
            });
            ragdoll.rise_keys = super::getup::keys(lying, &rig);
            if let Some(fall) = ragdoll.fall.as_mut()
                && let Some(rise) = fall.rise.as_mut()
            {
                rise.lying = Some(lying);
                rise.turn = turn;
                rise.turn_pending = true;
            }
        }
        if rise.elapsed < ragdoll.rise_seconds() + time.delta_secs() {
            if let Some(fall) = ragdoll.fall.as_mut()
                && let Some(rise) = fall.rise.as_mut()
            {
                rise.elapsed += time.delta_secs();
            }
            continue;
        }
        let ours: Vec<Entity> = ragdoll.bodies.iter().filter_map(|(_, body)| *body).collect();
        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let Ok(drawn) = live.compute_global_transform(skeleton.entity(bone)) else { continue };
            let rotation = drawn.rotation();
            let position = drawn.translation() + rotation * ragdoll.body_offsets[bone];
            commands.entity(body).insert((
                Position(position),
                Rotation(rotation),
                Transform::from_translation(position).with_rotation(rotation),
                LinearVelocity::ZERO,
                AngularVelocity::ZERO,
                SleepThreshold::default(),
            ));
            if bone == Bone::Hips {
                commands.entity(body).insert((
                    RigidBody::Kinematic,
                    KinematicRoot { offset: ragdoll.body_offsets[bone], position, rotation },
                ));
            }
            commands.queue(WakeBody(body));
        }
        for (joint, spherical) in &joints {
            if ours.contains(&spherical.body2) {
                commands.entity(joint).remove::<JointDamping>();
            }
        }
        ragdoll.fall = None;
    }
}

/// Lets go of the root of every ragdoll set falling ([`Ragdoll::fall`]):
/// a pinned (kinematic) root becomes dynamic, keeping the velocity it was
/// following the hips with, so the body falls with the momentum the
/// character had.
///
/// Records where the hips body sits in the hips bone's frame
/// ([`super::ragdoll::Fall::root_offset`]), which the display needs to
/// hang the skeleton on the body.
fn release_falling_roots(
    mut commands: Commands,
    mut characters: Query<(&mut Ragdoll, &HumanoidSkeleton)>,
    roots: Query<&KinematicRoot>,
    bodies: Query<(&Position, &Rotation)>,
    mut velocities: Query<&mut LinearVelocity>,
    joints: Query<(Entity, &SphericalJoint)>,
    live: TransformHelper,
) {
    for (mut ragdoll, skeleton) in &mut characters {
        let Some(fall) = ragdoll.fall else { continue };
        if fall.root_offset.is_some() {
            continue;
        }
        let ours: Vec<Entity> = ragdoll.bodies.iter().filter_map(|(_, body)| *body).collect();
        for &body in &ours {
            commands.entity(body).insert(FALLEN_SLEEP);
            if fall.launch != Vec3::ZERO
                && let Ok(mut velocity) = velocities.get_mut(body)
            {
                velocity.0 += fall.launch;
            }
        }
        if fall.damping > 0.0 {
            for (joint, spherical) in &joints {
                if ours.contains(&spherical.body2) {
                    commands.entity(joint).insert(JointDamping { linear: 0.0, angular: fall.damping });
                }
            }
        }
        let Some(body) = ragdoll.bodies[Bone::Hips] else { continue };
        let offset = match roots.get(body) {
            Ok(root) => {
                commands.entity(body).remove::<KinematicRoot>().insert(RigidBody::Dynamic);
                root.offset
            }
            // Never pinned: measured from where the body is now.
            Err(_) => {
                let (Ok((position, rotation)), Ok(hips)) =
                    (bodies.get(body), live.compute_global_transform(skeleton.entity(Bone::Hips)))
                else {
                    continue;
                };
                rotation.0.inverse() * (position.0 - hips.translation())
            }
        };
        if let Some(fall) = ragdoll.fall.as_mut() {
            fall.root_offset = Some(offset);
        }
    }
}

/// Each body carries as much of its own weight as its joint has strength.
///
/// `GravityScale = 1 - effective strength`: a fully driven joint's muscles
/// hold it up completely, a limp one falls under full gravity, and a
/// stunned limb drops for exactly as long as it is stunned.
///
/// # Why the controller cannot do this itself
///
/// The PD output is an angular ACCELERATION applied to one body, which is
/// what makes its tuning rig-independent — and also what makes it blind to
/// load. A spine body weighs a few grams and carries the entire upper body
/// through its joints; an acceleration sized for the body alone cannot
/// hold that up. Measured on the live real rig with limits off: every limb
/// tracked to a few degrees with gravity disabled, and the torso folded
/// over to 178 degrees the moment gravity was on.
///
/// Scaling gravity by strength is exact gravity compensation at full
/// strength, needs no model of the load, and reuses the one number the
/// torque ceiling and the display blend already share.
fn support_own_weight(
    characters: Query<&Ragdoll>,
    mut bodies: Query<(&JointTarget, &mut GravityScale)>,
) {
    for (target, mut gravity) in &mut bodies {
        let Ok(ragdoll) = characters.get(target.character) else { continue };
        let scale = 1.0 - ragdoll.carried_weight(target.bone);
        // Compared first so an unchanged body is not marked changed.
        if gravity.0 != scale {
            gravity.0 = scale;
        }
    }
}

/// Delivers each [`RagdollHit`]: a stun on the joints, a shove on the body.
///
/// Runs in `Update`, outside the physics schedule, and that is correct for
/// an impulse where it would not be for a torque: avian applies an impulse
/// to the velocity immediately rather than accumulating it for the next
/// step, so it lands exactly once however many substeps follow.
fn apply_ragdoll_hits(
    mut hits: MessageReader<RagdollHit>,
    mut characters: Query<(&mut Ragdoll, Option<&mut super::balance::Balance>, Option<&HumanoidSkeleton>)>,
    // `ComputedMass` alongside `Forces` is legal: `Forces` only reads it.
    // (Its own mass accessor is on a private avian trait.)
    mut bodies: Query<(&RigidBody, &ComputedMass, Forces)>,
    transforms: Query<&Transform>,
    live: TransformHelper,
) {
    for hit in hits.read() {
        let Ok((mut ragdoll, balance, skeleton)) = characters.get_mut(hit.character) else { continue };

        // The bone that actually receives the blow. The stun centres here
        // too rather than on `hit.bone`, so the joint that visibly moves is
        // the one fully slackened.
        let Some(struck) = ragdoll
            .bodies[hit.bone]
            .map(|_| hit.bone)
            .or_else(|| nearest_simulated_ancestor(hit.bone, &ragdoll))
        else {
            continue;
        };

        ragdoll.stun(struck, hit.stun);

        let Some(body) = ragdoll.bodies[struck] else { continue };

        // The blow moves the whole body too: the struck body's momentum
        // change, spread over all of it, is a push on the standing balance
        // (`Balance::push`), which absorbs it, steps, or falls. Without
        // this the pinned root held the character up through any blow.
        if let (Some(mut balance), Some(skeleton)) = (balance, skeleton)
            && !ragdoll.is_falling()
        {
            let mass = |body: Entity| bodies.get(body).map_or(0.0, |(_, mass, _)| mass.value());
            let total: f32 = ragdoll.bodies.iter().filter_map(|(_, body)| *body).map(mass).sum();
            if total > 0.0 {
                let moved = hit.velocity * (mass(body) / total);
                let rig = rig_geometry(skeleton, &transforms, &live);
                balance.push(Vec2::new(moved.dot(rig.forward()), moved.dot(rig.left())));
            }
        }
        let Ok((rigid_body, mass, mut forces)) = bodies.get_mut(body) else { continue };

        // Only a dynamic body can be shoved. This is not a formality: a
        // pinned root is KINEMATIC, and avian moves kinematic bodies by
        // their velocity — an impulse there would not be resisted, it would
        // set the whole character drifting forever.
        if !rigid_body.is_dynamic() {
            continue;
        }

        let inverse_mass = mass.inverse();
        if inverse_mass <= 0.0 {
            continue;
        }
        let impulse = hit.velocity / inverse_mass;

        match hit.point {
            Some(point) => forces.apply_linear_impulse_at_point(impulse, point),
            None => forces.apply_linear_impulse(impulse),
        }
    }
}

/// Returns each stunned joint toward its dial.
fn recover_from_hits(mut ragdolls: Query<&mut Ragdoll>, time: Res<Time>) {
    let dt = time.delta_secs();
    for mut ragdoll in &mut ragdolls {
        // Checked through a shared borrow first, so an unstunned ragdoll is
        // not marked changed every frame.
        if ragdoll.is_stunned() {
            ragdoll.recover(dt);
        }
    }
}

/// Converts the kinematic pose into a world-space target per simulated
/// joint.
fn publish_joint_targets(
    characters: Query<(Entity, &Ragdoll, &AnimPose, Option<&AnimFootIk>, &HumanoidSkeleton)>,
    transforms: Query<&Transform>,
    live: TransformHelper,
    mut targets: Query<&mut JointTarget>,
    mut roots: Query<&mut KinematicRoot>,
) {
    for (character, ragdoll, pose, foot_ik, skeleton) in &characters {
        // The pose actually being rendered — ground-corrected where Stage 3
        // is present, so the ragdoll chases the feet the player can see
        // rather than the pre-IK animation.
        let rendered = foot_ik
            .and_then(|ik| ik.corrected)
            .unwrap_or_else(|| pose.pose());

        let rig = rig_geometry(skeleton, &transforms, &live);
        let world = joint_targets(&rendered, &rig);

        // A pinned root follows the hips exactly as they stand this frame.
        if let Some(mut root) = ragdoll.bodies[Bone::Hips].and_then(|body| roots.get_mut(body).ok())
            && let Ok(hips) = live.compute_global_transform(skeleton.entity(Bone::Hips))
        {
            root.rotation = hips.rotation();
            root.position = hips.translation() + root.rotation * root.offset;
        }

        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let Ok(mut target) = targets.get_mut(body) else { continue };

            target.bone = bone;
            target.character = character;
            target.target = world[bone];
        }
    }
}

/// Moves each pinned root toward its [`KinematicRoot`] pose by setting the
/// velocity that arrives there in exactly one physics step. Runs inside
/// `PhysicsSchedule`, where `Time` is the physics step.
///
/// (Tried and reverted: following the frame's measured pace plus a drift
/// correction, to avoid a zero-velocity step when one frame runs two
/// physics steps. Live, it changed nothing measurable — the root's surging
/// speed came from the animated hips themselves.)
fn follow_kinematic_roots(
    mut roots: Query<(&KinematicRoot, &Position, &Rotation, &mut LinearVelocity, &mut AngularVelocity)>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }

    for (root, position, rotation, mut linear, mut angular) in &mut roots {
        linear.0 = (root.position - position.0) / dt;

        // Shortest way round: q and -q are one orientation.
        let delta = root.rotation * rotation.0.inverse();
        let delta = if delta.w < 0.0 { -delta } else { delta };
        angular.0 = delta.to_scaled_axis() / dt;
    }
}

/// Registers the systems that run inside `PhysicsSchedule`. Shared by the
/// plugin and the headless test harness, so the two cannot drift apart.
fn add_physics_step_systems(app: &mut App) {
    app.get_schedule_mut(PhysicsSchedule)
        .expect("AnimRagdollPlugin requires PhysicsPlugins")
        .add_systems(
            (follow_kinematic_roots, apply_joint_torques)
                .in_set(PhysicsStepSystems::First)
                // avian's interpolation bookkeeping also writes velocities
                // in this set, which Bevy flags as an ambiguity. It is
                // benign: that system only snapshots the previous velocity
                // for rendering, while these set the velocity or the
                // acceleration the solver is about to integrate — neither
                // reads what the other writes.
                //
                // Declared rather than ordered because the system in
                // question is private to avian and cannot be named.
                .ambiguous_with_all(),
        );
}

/// Applies each joint's PD torque. Runs inside `PhysicsSchedule`.
fn apply_joint_torques(
    characters: Query<&Ragdoll>,
    mut bodies: Query<(&JointTarget, Forces)>,
    time: Res<Time>,
) {
    // The step this torque will actually be integrated over. Passed to the
    // controller so it can clamp its damping gain to what that step can
    // carry — see `PdParams::stable_damping` for the measured vibration
    // this prevents on a jointed body.
    //
    // Inside `PhysicsSchedule`, `Time` is avian's own physics clock, so
    // this is the physics step rather than the render frame.
    let dt = time.delta_secs();

    for (target, mut forces) in &mut bodies {
        let Ok(ragdoll) = characters.get(target.character) else { continue };

        // Read the orientation and spin through `Forces` rather than
        // querying `Rotation`/`AngularVelocity` alongside it: `Forces`
        // already borrows `AngularVelocity` mutably, so a second access in
        // the same query is a hard conflict.
        let current = forces.rotation().0;
        let angular_velocity = forces.angular_velocity();

        let torque = joint_torque_at(
            target.bone,
            current,
            target.target,
            angular_velocity,
            ragdoll,
            dt,
        );

        if torque != Vec3::ZERO {
            // `apply_angular_acceleration`, NOT `apply_torque`.
            //
            // The PD gains are acceleration-shaped — `omega^2` and
            // `2*zeta*omega` describe a second-order system directly — so
            // they already account for inertia. `apply_torque` divides by
            // the body's inverse angular inertia as well, applying the
            // scaling twice.
            //
            // Measured: a capsule bone has an inverse angular inertia
            // around 2.9e4 to 2.9e5, so a perfectly reasonable 2232 N·m
            // became 6.4e8 rad/s² and the body span up to 1e7 rad/s within
            // a single step. That reads exactly like the instability this
            // design was built to avoid, and is not one — it is a unit
            // error.
            //
            // Applying acceleration directly also means the gains behave
            // identically on a heavy hip and a light wrist, so the graded
            // defaults in `ragdoll::default_joint_params` express intent
            // rather than compensating for mass.
            forces.apply_angular_acceleration(torque);
        }
    }
}

/// Writes the simulated bodies back onto the rendered skeleton.
///
/// This is what makes the ragdoll *visible*. Without it the physics runs
/// and the PD controller works, but nothing downstream reads the result —
/// the character renders its kinematic pose and the simulation is a very
/// expensive no-op. (It shipped that way at first, with `RagdollSet::ReadBack`
/// declared and empty.)
///
/// # Converting world rotations back into a local pose
///
/// Each body carries its bone's WORLD rotation. The skeleton wants each
/// bone's pose delta, so this walks parent-before-child undoing the
/// accumulation [`joint_targets`] performs — see [`delta_from_world`] for
/// the composition, including the bind-frame conjugation the naive
/// `bind⁻¹ * parent_world⁻¹ * world` misses on any rig whose binds are not
/// identity.
///
/// A bone with no simulated body keeps its animated local rotation, which
/// is why a partial ragdoll (`bodies` sparse by design — leaves are never
/// simulated) renders correctly rather than collapsing those bones to rest.
///
/// # Blending
///
/// Per-joint [`RagdollStrength`] selects between the animated pose and the
/// simulated one — but in the opposite sense to the torque path. There,
/// strength scales how hard the controller *drives*; here it decides how
/// much of the result is *shown*. A fully driven joint (`1.0`) is already
/// tracking the animation, so showing the simulation is nearly a no-op; a
/// limp joint (`0.0`) has physically fallen away from the animation, and
/// showing that is the whole point.
///
/// So the blend is `slerp(animated, simulated, 1 - strength)`: it displays
/// the simulation exactly where the controller has stopped enforcing the
/// animation.
///
/// # Where the result goes
///
/// Into [`Ragdoll::displayed`], never back into `AnimPose`. The animation
/// must stay the animation: see that field for the feedback loop writing
/// it into the spring state created.
///
/// # The animated end is the CORRECTED pose
///
/// The same pose [`publish_joint_targets`] drives the bodies toward: foot
/// IK, the pelvis drop and arm IK included. This used to blend from the
/// raw `AnimPose` instead, so switching the ragdoll on silently switched
/// foot and arm IK off on screen, and the bodies tracked one pose while
/// the renderer drew another (the arm and spine bodies sat ~7 cm off the
/// drawn skeleton once they tracked at all). Where the simulation should
/// win, the blend already lets it — at low strength the corrected pose is
/// simply not shown.
fn read_back_simulated_pose(
    mut characters: Query<(&mut Ragdoll, &AnimPose, Option<&AnimFootIk>, &HumanoidSkeleton)>,
    bodies: Query<&Rotation>,
    transforms: Query<&Transform>,
    live: TransformHelper,
) {
    for (mut ragdoll, pose, foot_ik, skeleton) in &mut characters {
        if !ragdoll.bodies.iter().any(|(_, body)| body.is_some()) {
            continue;
        }

        let rig = rig_geometry(skeleton, &transforms, &live);
        let animated = foot_ik.and_then(|ik| ik.corrected).unwrap_or_else(|| pose.pose());
        let mut displayed = animated;

        // The world rotations the animation would have produced, used both
        // as the parent frame for bones with no body and as the blend's
        // animated end.
        let accumulated_bind = super::rig::accumulate_bind_rotations(&rig);

        // The simulated world rotation of every bone, parent before child:
        // a body's own rotation where one exists, and otherwise the
        // simulated parent carrying this bone's ANIMATED delta. The second
        // half matters now that bodies skip bones: `Spine1` has no body, so
        // `Spine2`'s physical delta has to be measured against where the
        // simulated lower torso actually carried `Spine1`, not against the
        // animation's `Spine1`.
        let mut world = BoneSet::splat(Quat::IDENTITY);
        for &bone in Bone::ALL.iter() {
            let parent_world = match bone.parent() {
                Some(parent) => world[parent],
                None => rig.root_rotation,
            };

            let body_rotation = ragdoll.bodies[bone].and_then(|body| bodies.get(body).ok());
            world[bone] = match body_rotation {
                Some(rotation) => rotation.0,
                None => {
                    let bind = accumulated_bind[bone];
                    let delta = animated.rotations[bone];
                    parent_world * rig.bind_rotations[bone] * (bind.inverse() * delta * bind)
                }
            };

            if body_rotation.is_none() {
                continue;
            }

            // Invert the accumulation to recover a pose delta.
            let simulated_local =
                delta_from_world(bone, parent_world, world[bone], &rig, &accumulated_bind);

            // Neighbourhood BEFORE the slerp, or a joint can take the long
            // way round between two rotations that are already close.
            let animated_local = animated.rotations[bone];
            let simulated_local = neighborhood(animated_local, simulated_local);

            // Effective strength, so a hit's stun is SHOWN as well as felt
            // by the controller — see `Ragdoll::effective_strength`.
            let show_simulation = ragdoll.shown(bone);
            displayed.rotations[bone] = animated_local.slerp(simulated_local, show_simulation).normalize();
        }

        // Rising: from the pose it lies in, through the get-up keys, to the
        // animation. The root's first leg, off the lying body, is
        // `write_simulated_pose`'s.
        //
        // Blended per bone in the WORLD, then turned back into local
        // rotations: each segment takes the shortest way from where it is to
        // where the next key has it. Blended locally, a limb rode its
        // parents' swing as well as its own, and sitting up flung an arm out
        // sideways, palm up, on its way to the floor.
        if let Some((segment, t)) = ragdoll.rise_segment() {
            let keys = &ragdoll.rise_keys;
            let lying = displayed;
            let from = if segment == 0 { lying } else { keys[segment - 1].pose };
            let to = keys.get(segment).map_or(animated, |key| key.pose);
            let (a, b) = (super::rig::accumulate_world_rotations(&from, &rig), super::rig::accumulate_world_rotations(&to, &rig));
            let mut world = BoneSet::splat(Quat::IDENTITY);
            for &bone in Bone::ALL.iter() {
                world[bone] = a[bone].slerp(neighborhood(a[bone], b[bone]), t).normalize();
                let parent_world = bone.parent().map_or(rig.root_rotation, |parent| world[parent]);
                let local = delta_from_world(bone, parent_world, world[bone], &rig, &accumulated_bind);
                displayed.rotations[bone] = neighborhood(from.rotations[bone], local).normalize();
            }
            displayed.root_translation = if segment == 0 {
                to.root_translation
            } else {
                from.root_translation.lerp(to.root_translation, t)
            };
        }

        ragdoll.displayed = Some(displayed);
    }
}

/// Re-writes the skeleton from the blended pose.
///
/// A near-copy of `plugin::write_poses`, deliberately not shared: that one
/// runs in `AnimSet::Write` during `Update`, and this has to run after the
/// physics writeback in `PostUpdate`. Calling the same system twice in two
/// schedules would do the first write's work again for every character,
/// including those with no ragdoll at all.
///
/// Writes [`Ragdoll::displayed`], whose animated end already includes the
/// ground and arm corrections — see [`read_back_simulated_pose`].
///
/// # While falling
///
/// The pose's root translation is the animation's; the body has gone
/// elsewhere. So the hips are placed on the hips body (its centre less
/// [`super::ragdoll::Fall::root_offset`]), and the character entity
/// follows the body across the ground, keeping its height, so whatever
/// follows the character (a camera, a controller) follows the fall. The
/// entity is taken to be top-level: its translation is a world position.
fn write_simulated_pose(
    rigs: Query<(Entity, &HumanoidSkeleton, &Ragdoll)>,
    bodies: Query<(&Position, &Rotation)>,
    parents: Query<&ChildOf>,
    mut transforms: ParamSet<(Query<&mut Transform>, TransformHelper)>,
) {
    for (character, skeleton, ragdoll) in &rigs {
        let Some(displayed) = &ragdoll.displayed else { continue };
        super::retarget::write_pose_to_skeleton(skeleton, displayed, &mut transforms.p0());

        let Some(fall) = ragdoll.fall else { continue };
        let Some(offset) = fall.root_offset else { continue };
        let Some(Ok((position, rotation))) = ragdoll.bodies[Bone::Hips].map(|body| bodies.get(body)) else { continue };
        let lying = position.0 - rotation.0 * offset;
        // Rising, the character stays where it is and the hips blend from
        // the body up to where the animation just put them.
        // Rising, the character stays where it is, and the hips leave the
        // body for the first key's over the first segment; after that the
        // keys' own root (the written pose) places them.
        let rising = ragdoll.rise_segment();
        if rising.is_none()
            && let Ok(mut entity) = transforms.p0().get_mut(character)
        {
            entity.translation.x = lying.x;
            entity.translation.z = lying.z;
        }
        let hips = skeleton.entity(Bone::Hips);
        let Ok(parent) = parents.get(hips).map(ChildOf::parent) else { continue };
        let Ok(parent_world) = transforms.p1().compute_global_transform(parent) else { continue };
        let Ok(written) = transforms.p0().get(hips).map(|transform| parent_world.affine().transform_point3(transform.translation)) else { continue };
        let hips_world = match rising {
            Some((0, t)) => lying.lerp(written, t),
            Some(_) => written,
            None => lying,
        };
        let local = parent_world.affine().inverse().transform_point3(hips_world);
        if let Ok(mut transform) = transforms.p0().get_mut(hips) {
            transform.translation = local;
        }
        // Blending the lying pose's rotations up to standing swings the
        // feet through the floor (a ball 17 cm under it, live): rising, the
        // whole skeleton is lifted so no end joint goes below the ground,
        // the character entity's height.
        if rising.is_some()
            && let Ok(ground) = transforms.p0().get(character).map(|entity| entity.translation.y)
        {
            let lowest = RISE_CLEARANCE_BONES
                .iter()
                .filter_map(|&bone| transforms.p1().compute_global_transform(skeleton.entity(bone)).ok())
                .map(|joint| joint.translation().y)
                .fold(f32::MAX, f32::min);
            let lift = ground - lowest;
            if lift > 0.0 && lift.is_finite() {
                let local = parent_world.affine().inverse().transform_point3(hips_world + Vec3::Y * lift);
                if let Ok(mut transform) = transforms.p0().get_mut(hips) {
                    transform.translation = local;
                }
            }
        }
    }
}

/// The joints a rise keeps above the ground: the body's ends.
const RISE_CLEARANCE_BONES: [Bone; 7] = [
    Bone::LeftToeBase,
    Bone::RightToeBase,
    Bone::LeftFoot,
    Bone::RightFoot,
    Bone::LeftHand,
    Bone::RightHand,
    Bone::Head,
];

/// The world rotation each joint's body is driven toward under `pose`:
/// exactly where the renderer draws that bone.
///
/// Delegates to [`super::rig::accumulate_world_rotations`] rather than
/// composing the chain here. This module used to keep its own copy,
/// `parent * bind * delta`, from before the rig's pose-space fix — a delta
/// applied in the bone's local frame where the renderer conjugates it into
/// the bind frame. On the synthetic rig every bind is identity and the two
/// agree, so every headless test passed; on the real rig the live ragdoll
/// sat 35-178 degrees off its targets on every body. One definition, shared
/// with the renderer's own contract, is what keeps that from recurring.
fn joint_targets(pose: &LocalPose, rig: &RigGeometry) -> BoneSet<Quat> {
    super::rig::accumulate_world_rotations(pose, rig)
}

/// The pose delta that puts `bone` at `world`, given its parent's world
/// rotation — the exact inverse of [`joint_targets`]' composition.
///
/// ```text
///   world = parent_world * bind_local * (B⁻¹ * delta * B)
///   delta = B * ((parent_world * bind_local)⁻¹ * world) * B⁻¹
/// ```
///
/// where `B` is the bone's accumulated bind rotation: a pose delta names a
/// WORLD axis, and `B` is what converts it into the bone's frame.
fn delta_from_world(
    bone: Bone,
    parent_world: Quat,
    world: Quat,
    rig: &RigGeometry,
    accumulated_bind: &BoneSet<Quat>,
) -> Quat {
    let local = (parent_world * rig.bind_rotations[bone]).inverse() * world;
    let bind = accumulated_bind[bone];
    bind * local * bind.inverse()
}

/// Reads a skeleton's real geometry, the same way the IK stage does — but
/// rooted in the WORLD as the character stands this frame.
///
/// # A live root, not the bind-time one
///
/// `RigGeometry::from_skeleton` roots the chain at `hips_root_rotation`,
/// the hips' parent's rotation when the rig was bound. That is right for
/// the kinematic stack, which works in the character's own frame, and
/// wrong here: the bodies live in world space, so the moment the character
/// turned, every published target was off by the turn. It never showed
/// because every check ran on a standing character. The hips' parent's
/// rotation is read fresh through `TransformHelper` instead of from
/// `GlobalTransform`, which would still describe last frame.
fn rig_geometry(
    skeleton: &HumanoidSkeleton,
    transforms: &Query<&Transform>,
    live: &TransformHelper,
) -> RigGeometry {
    let offsets = super::rig::BoneSet::from_fn(|bone| {
        if bone == Bone::Hips {
            return bone.t_pose_offset();
        }
        // In metres: see `HumanoidSkeleton::bone_translation_scale`.
        transforms
            .get(skeleton.entity(bone))
            .map(|transform| transform.translation * skeleton.bone_translation_scale())
            .unwrap_or_else(|_| bone.t_pose_offset())
    });

    let mut rig = RigGeometry::from_skeleton(skeleton, offsets);
    if let Some(root) = live_hips_parent_rotation(skeleton, transforms, live) {
        rig.root_rotation = root;
    }
    rig
}

/// The world rotation of whatever the hips hang from, as of now.
fn live_hips_parent_rotation(
    skeleton: &HumanoidSkeleton,
    transforms: &Query<&Transform>,
    live: &TransformHelper,
) -> Option<Quat> {
    let hips = skeleton.entity(Bone::Hips);
    let world = live.compute_global_transform(hips).ok()?.rotation();
    // Normalized: `local` is what the read-back wrote last frame, and
    // `inverse` assumes a unit quaternion. Unnormalized, any drift in it came
    // back through here as the next frame's root, and the next write, and
    // never went away: a fallen body's hips reached norm 1.03, scaling the
    // skeleton 6% and drawing it 8° off its bodies.
    let local = transforms.get(hips).ok()?.rotation.normalize();
    Some((world * local.inverse()).normalize())
}

/// Drives a pinned root toward the pose the character's hips have now.
///
/// The root is KINEMATIC so the ragdoll cannot fall (see
/// `RagdollSpawnConfig::pin_root`), which also means nothing moved it: a
/// walking character left its whole physics body where it spawned, still
/// perfectly posed. [`publish_joint_targets`] writes where the hips are;
/// [`follow_kinematic_roots`] closes the gap by velocity, so the joints and
/// contacts see real motion rather than a teleport.
#[derive(Component, Debug, Clone, Copy)]
pub struct KinematicRoot {
    /// The body's centre in its bone's own frame. A body sits at its
    /// segment's midpoint, not at the joint (see [`spawn_bone_body`]).
    pub offset: Vec3,
    /// Where the body should be this physics step.
    pub position: Vec3,
    /// How it should be oriented.
    pub rotation: Quat,
}

/// The collision layers ragdolls are given by default: the top 16 bits,
/// well away from avian's default first layer, which the world uses.
///
/// Each ragdoll takes ONE bit and filters out only that bit, so its own
/// bodies never touch (see `spawn_bone_body`) while other characters'
/// bodies and the world still do. It used to be one shared layer that every
/// ragdoll filtered out — which also made any two characters' ragdolls pass
/// straight through each other.
///
/// Bits are handed out round-robin by [`next_ragdoll_layer`]. With more
/// than 16 ragdolls alive at once, two can share a bit, and only that pair
/// passes through each other; a game that needs more distinct layers sets
/// [`RagdollSpawnConfig::collision_layer`] itself.
pub const RAGDOLL_LAYER_POOL: LayerMask = LayerMask(0xFFFF_0000);

/// The next layer bit from [`RAGDOLL_LAYER_POOL`], round-robin.
pub fn next_ragdoll_layer() -> LayerMask {
    use std::sync::atomic::{AtomicU32, Ordering};
    static NEXT: AtomicU32 = AtomicU32::new(0);

    ragdoll_layer(NEXT.fetch_add(1, Ordering::Relaxed))
}

/// The `n`-th ragdoll's layer: the `n`-th set bit of
/// [`RAGDOLL_LAYER_POOL`], wrapping.
pub fn ragdoll_layer(n: u32) -> LayerMask {
    let pool = RAGDOLL_LAYER_POOL.0;
    let mut remaining = pool;
    for _ in 0..n % pool.count_ones() {
        remaining &= remaining - 1;
    }
    LayerMask(remaining & remaining.wrapping_neg())
}

/// Where and how big one bone's simulated body is.
///
/// Grouped rather than passed as five positional arguments because four of
/// them are geometry that has to be mutually consistent — a caller that got
/// `world_rotation` and `segment_direction` from different sources would
/// reintroduce exactly the frame bug this type's own fields document.
#[derive(Debug, Clone, Copy)]
pub struct BoneBodyPlacement {
    /// The body's centre: the midpoint of the segment it spans.
    pub world_position: Vec3,
    /// The bone's own world rotation. See [`spawn_bone_body`].
    pub world_rotation: Quat,
    /// World-space vector from this bone's joint to its child's.
    pub segment_direction: Vec3,
    /// Length of that segment.
    pub length: f32,
    /// Capsule radius.
    pub radius: f32,
    /// The body's mass in kilograms, or `None` to let avian derive one from
    /// the capsule's volume. See [`default_body_masses`] for why the
    /// ragdoll sets it.
    pub mass: Option<f32>,
    /// This character's own collision layer: its bodies belong to it and
    /// ignore it. See [`RAGDOLL_LAYER_POOL`].
    pub collision_layer: LayerMask,
}

/// Spawns a simulated body for one bone.
///
/// Capsule-shaped, sized from the bone's own length, with the collider
/// centred on the segment rather than the joint — a capsule pinned at the
/// joint would leave half the limb uncovered.
///
/// # The body's rotation IS the bone's rotation
///
/// `world_rotation` must be the bone's own world rotation, exactly as
/// [`publish_joint_targets`] computes the PD target. That equality is the
/// whole contract: the controller drives the body's rotation toward the
/// target, so if the body's frame were anything else, "tracking the target"
/// would mean holding a pose the bone does not have.
///
/// This was live-caught getting it wrong. The body used to be spawned
/// oriented along its own *segment* (joint to child), which for a bone
/// whose bind rotation differs from its parent's is not the bone's frame at
/// all — measured at **93.5 degrees** off on the knees, 46 on the ankles,
/// 39-45 on the shoulders, while the spine and arms happened to agree.
/// Every body then spent the simulation being driven toward a frame it
/// could not sit in.
///
/// So the capsule is built from explicit endpoints instead, expressed in
/// the bone's own frame, leaving the body's rotation free to mean exactly
/// one thing.
pub fn spawn_bone_body(
    commands: &mut Commands,
    bone: Bone,
    character: Entity,
    placement: BoneBodyPlacement,
) -> Entity {
    let BoneBodyPlacement {
        world_position,
        world_rotation,
        segment_direction,
        length,
        radius,
        mass,
        collision_layer,
    } = placement;

    // The segment, expressed in the body's own frame. The body sits at the
    // segment's midpoint, so the endpoints are half of it either way.
    let half_segment = (world_rotation.inverse() * segment_direction) * 0.5;

    // A degenerate segment would make `capsule_endpoints` produce a
    // zero-length shape; fall back to the body's own axis so a bad bone
    // cannot poison the physics world.
    let half_segment = if half_segment.length() < 1.0e-5 {
        Vec3::Y * 0.5 * length.max(0.02)
    } else {
        half_segment
    };

    let density = match mass {
        Some(mass) if mass > 0.0 => density_for(mass, radius, half_segment.length() * 2.0),
        _ => 1.0,
    };

    let body = commands
        .spawn((
            RigidBody::Dynamic,
            // Explicit endpoints, so the collider carries the alignment and
            // the body's own rotation does not have to.
            Collider::capsule_endpoints(radius, -half_segment, half_segment),
            ColliderDensity(density),
            // A ragdoll's own bodies never touch each other; everything
            // else still collides. `JointCollisionDisabled` only exempts a
            // joint's own two bodies, and non-adjacent ones still touched:
            // the torso, sized as a torso, reached the neck capsule starting
            // 0.10 m above it. The contact fought the controller from the
            // very first step — opposite spins on neighbouring bodies with
            // every body exactly at its target — and settled into a standoff
            // with the neck 8.8 degrees off and still turning at 3.6 rad/s,
            // limits or no limits. See `RAGDOLL_LAYER_POOL`.
            CollisionLayers::new(collision_layer, !collision_layer),
            Transform::from_translation(world_position).with_rotation(world_rotation),
            JointTarget { bone, target: world_rotation, character },
            // Set every frame from the joint's strength — see
            // `support_own_weight`. Full gravity until then.
            GravityScale(1.0),
            Name::new(format!("ragdoll:{}", bone.name())),
        ))
        .id();

    // A limb's mass is not spread evenly along it the way a uniform capsule's
    // is: a thigh's sits 43% of the way from the hip, a forearm-and-hand's
    // 68% of the way to the wrist. Where Winter's Table 4.1 has a row for
    // the span this body covers, its centre of mass and inertia come from
    // the table instead of from the collider.
    if let (Some(segment), Some(mass)) = (anthropometry::limb_segment(bone), mass)
        && mass > 0.0
    {
        commands.entity(body).insert(limb_mass_properties(segment, mass, half_segment, radius));
    }

    body
}

/// A limb body's mass properties from its Table 4.1 row, for a body sitting
/// at its segment's midpoint with the segment running `-half_segment` (the
/// proximal joint) to `+half_segment`, in the body's own frame.
///
/// - Centre of mass at `com_from_proximal` along the segment.
/// - Inertia about the two transverse axes `m·(ρ·L)²`, from the radius of
///   gyration about the COM.
/// - Inertia about the long axis, which the 2D table does not give, that of
///   a solid cylinder of the capsule's radius, `½·m·r²`.
///
/// `NoAuto*`, so the collider's own uniform-density properties do not add
/// to them.
pub fn limb_mass_properties(
    segment: anthropometry::LimbSegment,
    mass: f32,
    half_segment: Vec3,
    radius: f32,
) -> impl Bundle {
    let length = half_segment.length() * 2.0;
    let along = half_segment.normalize_or(Vec3::Y);
    let transverse = mass * (segment.gyration_about_com * length).powi(2);
    let long_axis = 0.5 * mass * radius * radius;
    (
        Mass(mass),
        CenterOfMass(half_segment * (2.0 * segment.com_from_proximal - 1.0)),
        AngularInertia::new_with_local_frame(
            Vec3::new(transverse, long_axis, transverse),
            Quat::from_rotation_arc(Vec3::Y, along),
        ),
        NoAutoMass,
        NoAutoAngularInertia,
        NoAutoCenterOfMass,
    )
}

/// Builds a complete simulated skeleton for one character.
///
/// Reads the rig's own live `GlobalTransform`s, so it works on a retargeted
/// glTF as well as on the synthetic debug skeleton — the bodies land
/// exactly where the mesh's bones already are, whatever pose it is in.
///
/// # Which bones get a body
///
/// Every bone with a simulated parent and a non-trivial length. Leaves are
/// skipped: a toe or a hand is short enough that its capsule would be
/// mostly radius, and each extra body costs a constraint row for motion
/// nobody can see. [`Ragdoll::bodies`] carries `None` for them, and the
/// torque loop already skips a `None`.
///
/// # Ordering
///
/// Bodies first, joints second, because a joint needs both entities to
/// exist. `Bone::ALL` is parent-before-child, so one pass suffices.
///
/// Returns the [`Ragdoll`] to insert on the character. It is returned
/// rather than inserted so the caller can adjust strength or limits before
/// the first physics step — limits in particular are read when the joints
/// are created and cannot be changed afterwards.
pub fn spawn_ragdoll(
    commands: &mut Commands,
    character: Entity,
    skeleton: &HumanoidSkeleton,
    global_transforms: &Query<&GlobalTransform>,
    config: &RagdollSpawnConfig,
) -> Ragdoll {
    let mut ragdoll = Ragdoll { limits: config.limits, ..Default::default() };

    let layout = &config.layout;
    let collision_layer = config.collision_layer.unwrap_or_else(next_ragdoll_layer);

    // Pass one: a body per bone the layout names, placed at the midpoint of
    // the capsule running from this bone's joint to its tip.
    for &bone in Bone::ALL.iter() {
        let (Ok(joint), Some(tip)) = (
            global_transforms.get(skeleton.entity(bone)),
            body_tip(bone, layout, skeleton, global_transforms),
        ) else {
            continue;
        };

        let length = joint.translation().distance(tip);
        if length < config.minimum_bone_length {
            continue;
        }

        let Some(body_position) = body_world_position(bone, layout, skeleton, global_transforms)
        else {
            continue;
        };

        let body = spawn_bone_body(
            commands,
            bone,
            character,
            BoneBodyPlacement {
                world_position: body_position,
                // The bone's OWN world rotation, matching what
                // `publish_joint_targets` will target. See `spawn_bone_body`.
                world_rotation: joint.rotation(),
                segment_direction: tip - joint.translation(),
                length,
                radius: match bone {
                    // The torso is as wide as it is, whatever its bones'
                    // lengths — see `RagdollSpawnConfig::torso_radius`.
                    Bone::Hips | Bone::Spine | Bone::Spine2 => config.torso_radius,
                    _ => (length * config.radius_fraction).clamp(0.02, config.maximum_radius),
                },
                mass: Some(config.masses[bone]),
                collision_layer,
            },
        );

        // A foot stands on its sole, not on a capsule: a block in the
        // ankle bone's frame, which is this body's frame too, offset from
        // the body's centre (it sits at the ankle-to-ball midpoint).
        let block = match (config.feet, bone) {
            (Some([left, _]), Bone::LeftFoot) => Some(left),
            (Some([_, right]), Bone::RightFoot) => Some(right),
            _ => None,
        };
        if let Some(block) = block {
            commands.entity(body).insert(sole_collider(
                block,
                joint.rotation().inverse() * (body_position - joint.translation()),
            ));
        }

        // The root holds the whole assembly up — see
        // `RagdollSpawnConfig::pin_root`. Kinematic rather than static so a
        // controller can still move it; the solver treats both as infinite
        // mass, which is what stops the ragdoll falling.
        if bone == Bone::Hips && config.pin_root {
            commands.entity(body).insert((
                RigidBody::Kinematic,
                // Where it is now, until the first publish says otherwise.
                KinematicRoot {
                    offset: joint.rotation().inverse() * (body_position - joint.translation()),
                    position: body_position,
                    rotation: joint.rotation(),
                },
            ));
        }

        ragdoll.bodies[bone] = Some(body);
        ragdoll.body_offsets[bone] = joint.rotation().inverse() * (body_position - joint.translation());
    }

    // Pass two: connect each body to its nearest simulated ancestor. Not
    // simply to `bone.parent()` — a skipped bone would otherwise orphan
    // everything beneath it.
    for &bone in Bone::ALL.iter() {
        let Some(body) = ragdoll.bodies[bone] else { continue };
        let Some(parent_bone) = nearest_simulated_ancestor(bone, &ragdoll) else { continue };
        let Some(parent_body) = ragdoll.bodies[parent_bone] else { continue };

        let (Ok(parent_global), Ok(child_global)) = (
            global_transforms.get(skeleton.entity(parent_bone)),
            global_transforms.get(skeleton.entity(bone)),
        ) else {
            continue;
        };

        // The joint sits at this bone's own origin. Both anchors are that
        // same world point expressed in each BODY's local frame — which is
        // not the bone's frame: a body sits at its segment's MIDPOINT, half
        // a bone away from the bone's origin.
        //
        // Expressing the anchors in the bone entities' frames instead was a
        // real, live-caught bug: every joint ended up anchored half a bone
        // off, and the constraint spent the simulation wrenching bodies
        // toward points they should never have been pinned to. Measured at
        // 154-177 degrees of tracking error with gravity disabled, while
        // the published targets were provably correct to 0.0 degrees.
        let world_anchor = child_global.translation();

        // Body centres, recomputed exactly the way pass one placed them.
        let (Some(parent_body_position), Some(child_body_position)) = (
            body_world_position(parent_bone, layout, skeleton, global_transforms),
            body_world_position(bone, layout, skeleton, global_transforms),
        ) else {
            continue;
        };

        // A body's rotation is its bone's rotation (see `spawn_bone_body`),
        // so the world-to-body rotation is the bone's own inverse.
        let parent_anchor =
            parent_global.rotation().inverse() * (world_anchor - parent_body_position);
        let child_anchor =
            child_global.rotation().inverse() * (world_anchor - child_body_position);

        // The child's segment in its own bone frame. A bone is rigid, so
        // this is the same in every pose — which is what lets the limit
        // frame be centred on the bind pose whatever pose the character is
        // spawned in.
        let segment = child_global.rotation().inverse() * (child_body_position - world_anchor);
        let (basis_on_parent, basis_on_child) = joint_bases(
            segment,
            bind_rotation_between(parent_bone, bone, |b| skeleton.rest_rotation(b)),
        );

        connect_bodies(
            commands,
            parent_body,
            body,
            parent_anchor,
            child_anchor,
            basis_on_parent,
            basis_on_child,
            ragdoll.limits[bone],
        );
    }

    ragdoll
}

/// How [`spawn_ragdoll`] sizes the bodies it creates.
#[derive(Debug, Clone, Copy)]
pub struct RagdollSpawnConfig {
    /// Capsule radius as a fraction of the bone's own length. A limb whose
    /// radius scales with its length keeps a consistent look across
    /// differently-proportioned rigs.
    pub radius_fraction: f32,
    /// Upper bound on that radius, so a long bone does not produce an
    /// absurdly fat capsule.
    pub maximum_radius: f32,
    /// Bones shorter than this get no body at all; their children join the
    /// nearest simulated ancestor instead.
    ///
    /// A very short bone becomes a capsule that is nearly all radius, with
    /// almost no inertia or authority of its own, hung between two ball
    /// joints. The synthetic rig's 0.07 m ankle stub tumbled in place at
    /// 13-84 rad/s once jointed neighbours stopped colliding (the overlap
    /// contacts had been damping it), and giving it a sane mass did not
    /// stop it; skipping it left every other body settled at 0.0 degrees.
    /// The default sits between that stub and the shortest real segment
    /// measured on `puppet_base`, its 0.083 m neck.
    pub minimum_bone_length: f32,
    /// The rotation limits each joint is created with.
    ///
    /// Lives here rather than only on [`Ragdoll`] because avian's
    /// constraint owns its limits from the moment the joint entity exists —
    /// changing `Ragdoll::limits` afterwards does nothing. Passing them
    /// through the spawn config is the only point at which they can
    /// actually be chosen.
    pub limits: BoneSet<Option<JointLimits>>,
    /// Whether the root body is pinned in place.
    ///
    /// # Why this exists, and why it defaults to on
    ///
    /// The PD controller drives **rotation only**. Nothing in this module
    /// applies a linear force, so a ragdoll under gravity has nothing
    /// holding it up: every joint can be perfectly oriented while the whole
    /// assembly falls. Live-caught exactly that way — the bodies were
    /// 1.9 km below the floor after a few seconds, each one still correctly
    /// posed relative to its neighbours.
    ///
    /// Note this is invisible to a cohesion test: a ragdoll falling as one
    /// connected body keeps its spread constant, so
    /// `a_full_ragdoll_holds_itself_together_under_gravity` passes happily
    /// while the character is in orbit. It took a screenshot to see it.
    ///
    /// Pinning the root is the simple, correct answer while the ragdoll is
    /// a *reaction* layer on top of a kinematically-positioned character:
    /// the kinematic stack owns where the character is, and the ragdoll
    /// owns how it is bent. A free-floating ragdoll needs a character
    /// controller to own its position instead, which is why this is a
    /// switch rather than a hardcoded choice.
    pub pin_root: bool,
    /// Which bones own a body and where each ends. See
    /// [`default_body_layout`].
    pub layout: BoneSet<Option<BodyEnd>>,
    /// Each body's mass in kilograms. See [`default_body_masses`].
    pub masses: BoneSet<f32>,
    /// Capsule radius of the three torso bodies.
    ///
    /// Limbs scale their radius with their length; a torso does not — it is
    /// as wide as a chest, however the spine happens to be divided. Sized
    /// like a limb, the synthetic rig's 0.10 m upper torso became a 15 kg
    /// pencil with a third of the bending inertia of the head it carried,
    /// and the torso chain settled into a limit cycle: the neck coning 6.5
    /// degrees off its target at 2.5 rad/s, with or without gravity.
    pub torso_radius: f32,
    /// This character's collision layer, or `None` to take the next one
    /// from [`RAGDOLL_LAYER_POOL`]. Set it when a game manages its own
    /// layers or has more than 16 ragdolls alive at once.
    pub collision_layer: Option<LayerMask>,
    /// Flat soles for the feet (left, right), in each ankle bone's own
    /// frame ([`sole_blocks`]), in place of their capsules. A ragdoll that
    /// stands on its own feet needs them: a capsule from ankle to ball has
    /// no heel and no flat underside, and unpinned, the feet skated 0.8 m
    /// in 2.5 s, turning 67°. `None` keeps the capsules, which is enough
    /// for a ragdoll held up by its pinned root.
    pub feet: Option<[SoleBox; 2]>,
}

/// The flat soles [`RagdollSpawnConfig::feet`] wants, for `rig`: the walk's
/// own soles (`foot::Sole::block`), so the physics foot stands where the
/// animated one does.
pub fn sole_blocks(rig: &RigGeometry) -> [SoleBox; 2] {
    [Bone::LeftFoot, Bone::RightFoot].map(|ankle| Sole::of(rig, ankle).block())
}

/// A foot body's collider and friction for its sole `block`, given where the
/// body's centre sits from its ankle joint in the ankle's own frame
/// (`body_from_joint`): the body's frame IS the ankle bone's, so the block
/// goes in as it is, offset by that.
///
/// Friction 1.0, a rubber sole's: avian averages the two surfaces'.
pub fn sole_collider(block: SoleBox, body_from_joint: Vec3) -> impl Bundle {
    (
        Collider::compound(vec![(
            block.center - body_from_joint,
            block.rotation,
            Collider::cuboid(block.size.x, block.size.y, block.size.z),
        )]),
        Friction::new(1.0),
    )
}

impl Default for RagdollSpawnConfig {
    fn default() -> Self {
        Self {
            radius_fraction: 0.22,
            maximum_radius: 0.09,
            minimum_bone_length: 0.075,
            pin_root: true,
            limits: super::ragdoll::default_joint_limits(),
            layout: default_body_layout(),
            masses: default_body_masses(),
            torso_radius: 0.13,
            collision_layer: None,
            feet: None,
        }
    }
}

/// Where one simulated body's capsule ends. The body is owned by the bone
/// whose joint it starts at, and covers every bone between.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BodyEnd {
    /// At a descendant's joint. `Spine` running to `Spine2` is one body
    /// covering `Spine1` as well.
    Joint(Bone),
    /// Straight UP from the owning bone as the rig was bound, carried with
    /// the bone as it rotates, by `fraction` of the hips-to-bone distance.
    /// For a leaf with no joint beyond it: the head, whose top the rig does
    /// not mark.
    ///
    /// Bind-pose vertical rather than the bone's own `+Y`: every rig is
    /// authored upright and looking ahead, so the top of the skull points
    /// straight up in bind whatever axes the artist chose. On `puppet_base`
    /// the two happen to agree to 1.7 degrees; on a rig whose head bone
    /// points elsewhere, only bind vertical is still the head.
    Along { fraction: f32 },
}

/// A bone's world rotation in the bind pose, from its skeleton: the rig's
/// root rotation composed with every rest rotation down to the bone.
fn bind_world_rotation(skeleton: &HumanoidSkeleton, bone: Bone) -> Quat {
    let mut rotation = skeleton.rest_rotation(bone);
    let mut current = bone;
    while let Some(parent) = current.parent() {
        rotation = skeleton.rest_rotation(parent) * rotation;
        current = parent;
    }
    skeleton.hips_root_rotation() * rotation
}

/// The bind-pose world up, in `bone`'s own frame. See [`BodyEnd::Along`].
fn bind_up_in_bone_frame(bind_world: Quat) -> Vec3 {
    bind_world.inverse() * Vec3::Y
}

/// Which bones own a simulated body, and where each body ends.
///
/// # Fewer, chunkier bodies
///
/// Pelvis, a lower and an upper torso, the head, and upper arm, forearm,
/// thigh, shin and foot on each side: 14 bodies, the shape game ragdolls
/// are usually built in. The collarbones, `Spine1` and the neck get none;
/// they keep their animated rotation, carried by the body above them.
///
/// # The head is its own body, not the neck's
///
/// It was a head-and-neck body owned by `Neck`, which meant it rotated
/// with the NECK: in the relaxed stance the neck turns 41 degrees from
/// bind while the head, measured, bows only 29 — so the capsule standing in
/// for the head pointed where the head was not. Owned by `Head` and running
/// up from the skull base (see [`BodyEnd::Along`]), the body IS the head,
/// driven toward the head's own target. Its joint to the upper torso spans
/// the neck too, so its limit is the neck's and head's ranges combined.
///
/// It used to be one body per non-leaf bone, 17 on `puppet_base`, and the
/// short ones were unstable: the 0.083 m neck and 0.106 m lower spine
/// became capsules that were mostly radius, with almost no inertia, hung
/// between heavier neighbours. Measured live over 15 s, the neck spun at up
/// to 1769 rad/s and the lower spine at 880 while the long leg bodies held
/// their targets — the same failure the synthetic rig's 0.07 m ankle stub
/// showed in isolation.
///
/// The head's `fraction` is an anthropometric estimate: the head above the
/// skull base is about 0.35 of the hips-to-skull-base distance.
pub fn default_body_layout() -> BoneSet<Option<BodyEnd>> {
    BoneSet::from_fn(|bone| match bone {
        Bone::Hips => Some(BodyEnd::Joint(Bone::Spine)),
        Bone::Spine => Some(BodyEnd::Joint(Bone::Spine2)),
        Bone::Spine2 => Some(BodyEnd::Joint(Bone::Neck)),
        Bone::Head => Some(BodyEnd::Along { fraction: 0.35 }),
        Bone::LeftArm => Some(BodyEnd::Joint(Bone::LeftForeArm)),
        Bone::LeftForeArm => Some(BodyEnd::Joint(Bone::LeftHand)),
        Bone::RightArm => Some(BodyEnd::Joint(Bone::RightForeArm)),
        Bone::RightForeArm => Some(BodyEnd::Joint(Bone::RightHand)),
        Bone::LeftUpLeg => Some(BodyEnd::Joint(Bone::LeftLeg)),
        Bone::LeftLeg => Some(BodyEnd::Joint(Bone::LeftFoot)),
        Bone::LeftFoot => Some(BodyEnd::Joint(Bone::LeftToeBase)),
        Bone::RightUpLeg => Some(BodyEnd::Joint(Bone::RightLeg)),
        Bone::RightLeg => Some(BodyEnd::Joint(Bone::RightFoot)),
        Bone::RightFoot => Some(BodyEnd::Joint(Bone::RightToeBase)),
        _ => None,
    })
}

/// Each body's mass in kilograms, for a 70 kg person.
///
/// Segment fractions from the standard anthropometric tables (Dempster, as
/// tabulated by Winter): thorax 21.6%, abdomen 13.9%, pelvis 14.2%,
/// head-and-neck 8.1%, upper arm 2.8%, forearm-and-hand 2.2%, thigh 10%,
/// shank 4.65%, foot 1.45%.
///
/// # Why masses have to be set at all
///
/// Left to avian, a body's mass is its capsule volume at 1 kg/m³, which
/// scales with length cubed and ignores what the body carries. With the
/// torso as a few short bodies, the upper torso came out lighter than the
/// arms hung from it at a 0.2 m lever — and one arm spun off to 153 degrees
/// on a rig with gravity switched off.
///
/// Only the RATIOS matter: the controller is acceleration-shaped and a hit
/// is a velocity, so mass decides nothing but how neighbouring bodies share
/// a constraint's correction. That is exactly what these ratios are for.
pub fn default_body_masses() -> BoneSet<f32> {
    use super::anthropometry::fraction::*;
    const TOTAL: f32 = 70.0;
    BoneSet::from_fn(|bone| {
        TOTAL
            * match bone {
                Bone::Hips => PELVIS,
                Bone::Spine => ABDOMEN,
                Bone::Spine2 => THORAX,
                Bone::Head => HEAD_AND_NECK,
                Bone::LeftArm | Bone::RightArm => UPPER_ARM,
                Bone::LeftForeArm | Bone::RightForeArm => FOREARM_AND_HAND,
                Bone::LeftUpLeg | Bone::RightUpLeg => THIGH,
                Bone::LeftLeg | Bone::RightLeg => SHANK,
                Bone::LeftFoot | Bone::RightFoot => FOOT,
                _ => 0.01,
            }
    })
}

/// The collider density that gives a capsule of `radius` around a segment
/// of `length` exactly `mass`.
fn density_for(mass: f32, radius: f32, length: f32) -> f32 {
    use std::f32::consts::PI;
    let volume = PI * radius * radius * length + 4.0 / 3.0 * PI * radius.powi(3);
    if volume <= 0.0 { 1.0 } else { mass / volume }
}

/// The world position where `bone`'s body ends, per `layout`.
fn body_tip(
    bone: Bone,
    layout: &BoneSet<Option<BodyEnd>>,
    skeleton: &HumanoidSkeleton,
    global_transforms: &Query<&GlobalTransform>,
) -> Option<Vec3> {
    let position = |b: Bone| global_transforms.get(skeleton.entity(b)).ok().map(|g| g.translation());

    match layout[bone]? {
        BodyEnd::Joint(end) => position(end),
        BodyEnd::Along { fraction } => {
            let joint = global_transforms.get(skeleton.entity(bone)).ok()?;
            let hips = position(Bone::Hips)?;
            let start = joint.translation();
            let up = joint.rotation() * bind_up_in_bone_frame(bind_world_rotation(skeleton, bone));
            Some(start + up * fraction * start.distance(hips))
        }
    }
}

/// Where a bone's simulated body sits: the midpoint of the capsule running
/// from its own joint to its [`body_tip`].
///
/// Shared by both passes of [`spawn_ragdoll`] deliberately. Pass one places
/// the bodies and pass two anchors the joints between them, and those two
/// have to agree exactly — computing the midpoint twice from one definition
/// is what makes that agreement structural rather than a convention two
/// call sites have to remember.
fn body_world_position(
    bone: Bone,
    layout: &BoneSet<Option<BodyEnd>>,
    skeleton: &HumanoidSkeleton,
    global_transforms: &Query<&GlobalTransform>,
) -> Option<Vec3> {
    let joint = global_transforms.get(skeleton.entity(bone)).ok()?;
    let tip = body_tip(bone, layout, skeleton, global_transforms)?;

    Some(joint.translation().midpoint(tip))
}

/// A joint's relative rotation as the physical joint measures it:
/// `(swing, twist)` in radians — how far the bone tilts off its frame's
/// `+Y`, and its signed roll about itself.
///
/// # avian's limits are not a cone and a twist — unless you ask sideways
///
/// Measured, not read off the names (pinned by
/// `avians_swing_and_twist_limits_are_not_a_cone_and_a_twist`): avian 0.7
/// measures both limits against a REFERENCE axis,
/// `twist_axis.any_orthonormal_vector()`. `swing_limit` bounds how far that
/// reference tilts; `twist_limit` bounds the twist axes' roll about it.
/// With the default `twist_axis = +Y` along the bone, the reference lies
/// ACROSS the bone, and the two limits become two unrelated bend stops,
/// one of which also catches twist. Read as a cone and a twist, the live
/// arm's normal 85-degree bend was clamped at ~68.5 by its 70-degree
/// "twist" range, and the knee's bend fell under its 8-degree twist stop.
///
/// Setting `twist_axis = +X` instead makes the reference
/// `X.any_orthonormal_vector()` = `+Y`, the bone itself: `swing_limit`
/// becomes a true cone around the bone and `twist_limit` a true roll about
/// it (pinned by `a_twist_axis_across_the_bone_makes_avians_limits_a_true_cone_and_twist`).
/// This function mirrors that configuration exactly, so a pose can be
/// checked against the limits the solver will really enforce.
pub fn avian_limit_angles(relative: Quat) -> (f32, f32) {
    let bone = Vec3::Y;
    let moved = relative * bone;
    let swing = bone.angle_between(moved);

    // avian's twist: the twist axes (+X) projected across the bisector of
    // the two bone axes, and the signed angle between them about it.
    let n = (bone + moved).normalize_or_zero();
    if n == Vec3::ZERO {
        return (swing, 0.0);
    }
    let project = |b: Vec3| (b - n.dot(b) * n).normalize_or_zero();
    let (b1, b2) = (project(Vec3::X), project(relative * Vec3::X));
    if b1 == Vec3::ZERO || b2 == Vec3::ZERO {
        return (swing, 0.0);
    }
    let twist = n.dot(b1.cross(b2)).atan2(b1.dot(b2));
    (swing, twist)
}

/// Each body's joint frame, in its own local space, for a joint whose
/// frames coincide in the BIND pose with `+Y` down the child's segment.
///
/// `segment` is the child's segment in its own bone frame;
/// `bind_between` is the bind rotation from the parent bone's frame to the
/// child's (see [`bind_rotation_between`]). A body's frame IS its bone's
/// frame (see [`spawn_bone_body`]), so in the bind pose the child sits at
/// `parent · bind_between`, and choosing the parent's basis as
/// `bind_between · child_basis` makes the two frames coincide exactly
/// there. The joint's swing and twist are then precisely the child's pose
/// delta, which is what makes "the animation stays inside the limits" a
/// checkable property of the pose data.
pub fn joint_bases(segment: Vec3, bind_between: Quat) -> (Quat, Quat) {
    let axis = segment.normalize_or_zero();
    let on_child = if axis == Vec3::ZERO { Quat::IDENTITY } else { Quat::from_rotation_arc(Vec3::Y, axis) };
    (bind_between * on_child, on_child)
}

/// The product of bind rotations from just below `ancestor` down to and
/// including `bone`: the child's bind frame expressed in the ancestor's.
pub fn bind_rotation_between(ancestor: Bone, bone: Bone, rest: impl Fn(Bone) -> Quat) -> Quat {
    let mut between = Quat::IDENTITY;
    let mut current = bone;
    while current != ancestor {
        between = rest(current) * between;
        let Some(parent) = current.parent() else { break };
        current = parent;
    }
    between
}

/// The closest ancestor of `bone` that actually has a body.
///
/// Walking up rather than using `bone.parent()` directly means a skipped
/// bone does not detach the whole chain below it — a hand with no body
/// still leaves the forearm jointed to the arm.
fn nearest_simulated_ancestor(bone: Bone, ragdoll: &Ragdoll) -> Option<Bone> {
    let mut current = bone.parent()?;
    loop {
        if ragdoll.bodies[current].is_some() {
            return Some(current);
        }
        current = current.parent()?;
    }
}

/// Connects two simulated bodies with an unmotorized ball joint.
///
/// Deliberately unmotorized: see [`super::ragdoll`]'s own module doc for
/// why chaining motorized revolutes is the design this project already
/// found unstable.
///
/// `limits` is optional. `None` leaves the joint free to rotate anywhere,
/// which is how this shipped originally — a PD controller driving hard
/// against a constraint is a mild analogue of the two-springs problem, so
/// limits were added deliberately and have their own stability test
/// (`a_joint_driven_hard_into_its_limit_stays_stable`).
///
/// # The limit frame
///
/// `basis_on_parent` and `basis_on_child` are each body's joint frame in
/// its own local space. avian's twist axis is the frames' `+Y`, which
/// passes through the swing cone. [`spawn_ragdoll`] computes them with
/// [`joint_bases`] so both frames coincide in the BIND pose, with `+Y`
/// down the child bone: the cone is centred on the anatomical neutral and
/// the twist runs along the limb, on any rig.
///
/// This used to rely on each body's own local `+Y` being its bone axis.
/// That held while bodies were oriented along their segments, and stopped
/// holding when they were re-oriented into their bones' own frames (see
/// [`spawn_bone_body`]). On `puppet_base`, whose thigh is bound ~164
/// degrees off, the rest pose then sat far outside a 75-degree cone: the
/// solver snapped the legs "legal" on the first step and the live thighs
/// sat 90-121 degrees off their targets, with or without gravity.
#[allow(clippy::too_many_arguments)]
pub fn connect_bodies(
    commands: &mut Commands,
    parent: Entity,
    child: Entity,
    anchor_on_parent: Vec3,
    anchor_on_child: Vec3,
    basis_on_parent: Quat,
    basis_on_child: Quat,
    limits: Option<JointLimits>,
) -> Entity {
    let mut joint = SphericalJoint::new(parent, child)
        .with_local_anchor1(anchor_on_parent)
        .with_local_anchor2(anchor_on_child)
        .with_local_basis1(basis_on_parent)
        .with_local_basis2(basis_on_child);

    if let Some(limits) = limits {
        // ACROSS the bone, not along it: see `avian_limit_angles`. This is
        // what makes avian's two limits a true cone and a true twist.
        joint.twist_axis = Vec3::X;
        joint = joint
            .with_swing_limits(-limits.swing_half_angle, limits.swing_half_angle)
            .with_twist_limits(limits.twist_range.0, limits.twist_range.1);
    }

    // Neighbouring capsules meet at the joint and so always overlap — the
    // spine segments are barely longer than they are thick. avian lets
    // jointed bodies collide unless told otherwise, and did: the contacts
    // pushed every short, fat body off its target, measured on the live
    // real rig at 82-92 degrees on the neck and lower spine with gravity
    // AND limits both off, while the long limbs beside them tracked to a
    // few degrees.
    commands.spawn((joint, JointCollisionDisabled)).id()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::math::pd::PdParams;
    use crate::character::anim::ragdoll::{RagdollStrength, FALL_DAMPING, FALL_TONE};
    use crate::character::anim::rig::BoneSet;
    use core::time::Duration;
    use std::f32::consts::FRAC_PI_2;

    const TIMESTEP: f32 = 1.0 / 64.0;

    /// A headless physics app, following avian's own test harness: no
    /// window, no renderer, and a manually-driven clock so every run is
    /// identical.
    fn physics_app() -> App {
        let mut app = App::new();
        // `AssetPlugin` + `MeshPlugin` are required even headless: avian's
        // collider cache reads `AssetEvent<Mesh>`, and Bevy panics on an
        // unregistered message type rather than skipping the system. This
        // mirrors avian's own test harness.
        app.add_plugins((
            MinimalPlugins,
            bevy::asset::AssetPlugin::default(),
            bevy::mesh::MeshPlugin,
            PhysicsPlugins::default(),
            TransformPlugin,
        ));

        app.insert_resource(Gravity(Vec3::ZERO));
        app.insert_resource(Time::<Fixed>::from_duration(Duration::from_secs_f32(
            TIMESTEP,
        )));
        app.insert_resource(bevy::time::TimeUpdateStrategy::ManualDuration(
            Duration::from_secs_f32(TIMESTEP),
        ));

        // Let avian finish registering its own resources before reaching
        // into its schedule — see `AnimRagdollPlugin::finish`.
        app.finish();

        add_physics_step_systems(&mut app);

        // The read-back half, mirroring `AnimRagdollPlugin::build`.
        //
        // Registered by hand rather than by adding the plugin, because the
        // plugin also wants `AnimSet::Write` from `AnimPlugin` to order
        // against, and these tests deliberately run the ragdoll without the
        // kinematic stack. Keeping the two registrations in step is a real
        // maintenance cost — and the alternative was worse: without this,
        // every read-back test silently passed through a world where the
        // read-back never ran.
        app.add_systems(
            PostUpdate,
            (read_back_simulated_pose, write_simulated_pose)
                .chain()
                .after(PhysicsSystems::Writeback)
                .before(TransformSystems::Propagate),
        );

        // Hits, through the plugin's own registration.
        add_hit_systems(&mut app);

        app
    }

    /// A single free body driven toward `target`.
    fn spawn_driven_body(app: &mut App, target: Quat, params: PdParams) -> (Entity, Entity) {
        let ragdoll = Ragdoll {
            params: super::super::rig::BoneSet::splat(params),
            ..Default::default()
        };

        let character = app.world_mut().spawn(ragdoll).id();

        let body = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::capsule(0.05, 0.3),
                Transform::default(),
                JointTarget { bone: Bone::LeftArm, target, character },
            ))
            .id();

        (character, body)
    }

    fn step(app: &mut App, steps: usize) {
        for _ in 0..steps {
            app.update();
        }
    }

    fn rotation_of(app: &App, body: Entity) -> Quat {
        app.world().get::<Rotation>(body).map(|r| r.0).unwrap_or(Quat::IDENTITY)
    }

    #[test]
    fn a_driven_body_converges_on_its_target() {
        // The headline: a simulated body tracks an orientation through
        // torque alone, with no motor and no kinematic override.
        let mut app = physics_app();
        let target = Quat::from_axis_angle(Vec3::Y, FRAC_PI_2);

        let (_, body) = spawn_driven_body(
            &mut app,
            target,
            PdParams { frequency_hz: 6.0, damping_ratio: 1.0, max_torque: 5000.0 },
        );

        step(&mut app, 200);

        let mismatch = 1.0 - rotation_of(&app, body).dot(target).abs();
        assert!(
            mismatch < 1.0e-3,
            "the body should have reached its target, mismatch {mismatch}",
        );
    }

    #[test]
    fn a_driven_body_stays_stable_over_a_long_run() {
        // The explicit guard against the instability that killed the
        // earlier avian design. Sixty seconds of continuous driving must
        // produce no NaN and no runaway spin.
        let mut app = physics_app();

        let (_, body) = spawn_driven_body(
            &mut app,
            Quat::from_axis_angle(Vec3::Y, FRAC_PI_2),
            PdParams { frequency_hz: 8.0, damping_ratio: 1.0, max_torque: 5000.0 },
        );

        // Skip the first moments: a body starting 90 degrees from its
        // target accelerates hard to close that gap, and the initial
        // transient is legitimately fast. What must not happen is energy
        // GROWING once it has settled.
        let settle = (0.5 / TIMESTEP) as usize;
        for _ in 0..settle {
            app.update();
        }

        let mut worst_after_settling = 0.0f32;

        for _ in 0..(60.0 / TIMESTEP) as usize {
            app.update();

            let rotation = rotation_of(&app, body);
            assert!(rotation.is_finite(), "the rotation went non-finite: {rotation:?}");

            let velocity = app.world().get::<AngularVelocity>(body).unwrap().0;
            assert!(
                velocity.is_finite(),
                "the angular velocity went non-finite: {velocity:?}",
            );

            worst_after_settling = worst_after_settling.max(velocity.length());
        }

        // A settled joint should be essentially still. Anything growing
        // here is the runaway this design exists to avoid.
        assert!(
            worst_after_settling < 1.0,
            "after settling, the body should be nearly at rest, but reached {} rad/s \
             over the next minute",
            worst_after_settling,
        );
    }

    #[test]
    fn a_limp_body_is_not_driven_at_all() {
        // With zero strength the joint is a plain ragdoll joint, so a body
        // given an impulse just keeps going.
        let mut app = physics_app();

        let (character, body) = spawn_driven_body(
            &mut app,
            Quat::from_axis_angle(Vec3::Y, FRAC_PI_2),
            PdParams::default(),
        );

        app.world_mut()
            .get_mut::<Ragdoll>(character)
            .unwrap()
            .set_strength(0.0);

        app.update();
        app.world_mut().get_mut::<AngularVelocity>(body).unwrap().0 = Vec3::new(0.0, 1.0, 0.0);

        let before = app.world().get::<AngularVelocity>(body).unwrap().0;
        step(&mut app, 30);
        let after = app.world().get::<AngularVelocity>(body).unwrap().0;

        assert!(
            after.dot(before) > 0.0 && after.length() > before.length() * 0.5,
            "a limp joint should not brake the body: {before:?} -> {after:?}",
        );
    }

    #[test]
    fn a_torque_limit_lets_an_impulse_win() {
        // THE property that makes this an ACTIVE ragdoll rather than a
        // kinematic rig: a blow beyond the joint's strength moves it,
        // rather than being silently resisted.
        let mut app = physics_app();

        let (_, body) = spawn_driven_body(
            &mut app,
            Quat::IDENTITY,
            // Deliberately weak.
            PdParams { frequency_hz: 6.0, damping_ratio: 1.0, max_torque: 0.05 },
        );

        step(&mut app, 5);

        // A hard shove.
        app.world_mut().get_mut::<AngularVelocity>(body).unwrap().0 = Vec3::new(0.0, 12.0, 0.0);
        step(&mut app, 10);

        let displaced = rotation_of(&app, body).angle_between(Quat::IDENTITY);
        assert!(
            displaced > 0.3,
            "a weak joint should be overwhelmed by a hard impulse, but only moved \
             {displaced} rad",
        );
    }

    #[test]
    fn a_body_recovers_its_pose_after_being_struck() {
        // ...and then comes back, with no authored get-up animation. That
        // recovery is the whole point of keeping the controller running
        // through the impact rather than switching to a ragdoll mode.
        let mut app = physics_app();

        let (_, body) = spawn_driven_body(
            &mut app,
            Quat::IDENTITY,
            PdParams { frequency_hz: 6.0, damping_ratio: 1.0, max_torque: 500.0 },
        );

        step(&mut app, 20);

        // Hard enough to beat a correctly-scaled controller. An earlier
        // version used 8 rad/s, which a 500 rad/s^2 joint simply absorbed —
        // the blow has to actually land for a recovery test to mean
        // anything.
        app.world_mut().get_mut::<AngularVelocity>(body).unwrap().0 = Vec3::new(0.0, 60.0, 0.0);
        step(&mut app, 5);

        let knocked = rotation_of(&app, body).angle_between(Quat::IDENTITY);
        assert!(knocked > 0.1, "test setup: the blow should have moved it, got {knocked}");

        step(&mut app, 300);

        let recovered = rotation_of(&app, body).angle_between(Quat::IDENTITY);
        assert!(
            recovered < 0.05,
            "the joint should have pulled itself back to its target, but is still \
             {recovered} rad away (was {knocked})",
        );
    }

    #[test]
    fn a_stronger_joint_resists_the_same_impulse_better() {
        // The strength dial, observed end to end rather than on the torque
        // alone.
        let displacement_at = |max_torque: f32| {
            let mut app = physics_app();
            let (_, body) = spawn_driven_body(
                &mut app,
                Quat::IDENTITY,
                PdParams { frequency_hz: 6.0, damping_ratio: 1.0, max_torque },
            );

            step(&mut app, 5);
            app.world_mut().get_mut::<AngularVelocity>(body).unwrap().0 =
                Vec3::new(0.0, 6.0, 0.0);
            step(&mut app, 20);

            rotation_of(&app, body).angle_between(Quat::IDENTITY)
        };

        let weak = displacement_at(0.5);
        let strong = displacement_at(200.0);

        assert!(
            strong < weak,
            "a stronger joint should give less ground: {strong} vs {weak} rad",
        );
    }

    #[test]
    fn an_unmotorized_spherical_joint_holds_two_bodies_together() {
        // The ball constraint itself, independent of any torque: the two
        // bodies must stay connected at their shared anchor.
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));

        let anchor = Vec3::new(0.0, 0.0, 0.0);

        let upper = app
            .world_mut()
            .spawn((
                RigidBody::Static,
                Collider::capsule(0.05, 0.3),
                Transform::default(),
            ))
            .id();

        let lower = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::capsule(0.05, 0.3),
                Transform::from_translation(Vec3::new(0.0, -0.3, 0.0)),
            ))
            .id();

        app.world_mut().spawn(
            SphericalJoint::new(upper, lower)
                .with_local_anchor1(Vec3::new(0.0, -0.15, 0.0))
                .with_local_anchor2(Vec3::new(0.0, 0.15, 0.0)),
        );

        step(&mut app, 120);

        let upper_anchor = app.world().get::<Transform>(upper).unwrap().translation
            + Vec3::new(0.0, -0.15, 0.0);
        let lower_transform = *app.world().get::<Transform>(lower).unwrap();
        let lower_anchor =
            lower_transform.translation + lower_transform.rotation * Vec3::new(0.0, 0.15, 0.0);

        let separation = (upper_anchor - lower_anchor).length();
        assert!(
            separation < 0.05,
            "the joint should hold the bodies together, but they drifted {separation} m \
             apart (anchor {anchor:?})",
        );
    }

    /// Builds two bodies joined at a shared anchor, the upper one static,
    /// with the lower one driven by a PD controller toward `target`.
    ///
    /// This is the configuration joint limits actually have to survive: a
    /// controller with real authority pulling against a hard constraint.
    /// A single free body (which every other test here uses) cannot
    /// exercise it, because there is nothing for it to press against.
    ///
    /// The lower body's collider is centred on its segment, so the joint
    /// anchor sits half a bone away from its centre of mass — the real
    /// geometry [`spawn_bone_body`] produces, and the geometry under which
    /// the controller's steady-state behaviour is worth testing.
    fn spawn_limited_chain(
        app: &mut App,
        target: Quat,
        limits: Option<JointLimits>,
        params: PdParams,
    ) -> (Entity, Entity) {
        let character = app.world_mut().spawn_empty().id();

        let upper = app
            .world_mut()
            .spawn((RigidBody::Static, Collider::capsule(0.05, 0.3), Transform::default()))
            .id();

        // The shared anchor, in world space: the bottom of the static
        // upper body.
        const ANCHOR: Vec3 = Vec3::new(0.0, -0.15, 0.0);
        // Where the lower body's centre sits relative to that anchor when
        // it is unrotated.
        const CENTRE_FROM_ANCHOR: Vec3 = Vec3::new(0.0, -0.15, 0.0);

        // Start the body at rest (identity), which the constraint
        // satisfies exactly.
        let start_rotation = Quat::IDENTITY;
        let start_position = ANCHOR + start_rotation * CENTRE_FROM_ANCHOR;

        let lower = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::capsule(0.05, 0.3),
                Transform::from_translation(start_position).with_rotation(start_rotation),
                JointTarget { bone: Bone::LeftArm, target, character },
            ))
            .id();

        let mut joint = SphericalJoint::new(upper, lower)
            .with_local_anchor1(Vec3::new(0.0, -0.15, 0.0))
            .with_local_anchor2(Vec3::new(0.0, 0.15, 0.0));

        if let Some(limits) = limits {
            // The production limit configuration — see `connect_bodies`.
            joint.twist_axis = Vec3::X;
            joint = joint
                .with_swing_limits(-limits.swing_half_angle, limits.swing_half_angle)
                .with_twist_limits(limits.twist_range.0, limits.twist_range.1);
        }

        app.world_mut().spawn(joint);

        let mut ragdoll = Ragdoll::default();
        ragdoll.params = BoneSet::splat(params);
        ragdoll.bodies[Bone::LeftArm] = Some(lower);
        app.world_mut().entity_mut(character).insert(ragdoll);

        (character, lower)
    }

    /// Peak observed rotation *speed* in each one-second window, in rad/s,
    /// measured as the actual change in orientation per step.
    ///
    /// # Why not read `AngularVelocity`
    ///
    /// Because on a jointed body it is not the quantity it appears to be.
    /// Sampled once per `app.update()`, it alternates between roughly
    /// 0.5 and 11 rad/s on consecutive steps while the body's orientation
    /// moves less than one degree — an 11 rad/s spin would cover ten
    /// degrees in that time. The reading is a mid-solve value, not the
    /// body's settled motion, and a test asserting on it measures the
    /// schedule rather than the physics. (The single-body tests above get
    /// away with it only because an unconstrained body has no solver
    /// iteration to be caught in the middle of.)
    ///
    /// Differencing the orientation cannot lie in that way: if the body
    /// is not moving, consecutive rotations are equal, whatever any
    /// intermediate velocity buffer happens to hold.
    ///
    /// Returned per-second rather than as a single maximum because the
    /// distinction that matters for stability is not "how fast" but "is it
    /// growing" — a flat series is a bounded limit cycle, a rising one is
    /// the runaway this design exists to avoid, and a single worst-case
    /// number cannot tell them apart.
    fn per_second_rotation_speed_peaks(app: &mut App, body: Entity, seconds: f32) -> Vec<f32> {
        let steps_per_second = (1.0 / TIMESTEP) as usize;
        let mut peaks = Vec::new();
        let mut current = 0.0f32;
        let mut previous = rotation_of(app, body);

        for step_index in 0..(seconds / TIMESTEP) as usize {
            app.update();

            let rotation = rotation_of(app, body);
            assert!(rotation.is_finite(), "rotation went non-finite: {rotation:?}");

            let velocity = app.world().get::<AngularVelocity>(body).unwrap().0;
            assert!(velocity.is_finite(), "angular velocity went non-finite: {velocity:?}");

            // Orientation change per unit time — the body's real speed.
            current = current.max(previous.angle_between(rotation) / TIMESTEP);
            previous = rotation;

            if step_index % steps_per_second == steps_per_second - 1 {
                peaks.push(current);
                current = 0.0;
            }
        }

        peaks
    }

    #[test]
    fn a_joint_driven_to_a_reachable_target_reaches_it_despite_its_limit() {
        // The ordinary case, and the one that says limits do not break
        // normal driving: a target inside the cone is satisfiable, so the
        // controller must actually get there.
        //
        // # What this asserts, and why not stillness
        //
        // It asserts the joint ARRIVES, not that it goes quiet. A body
        // whose collider is centred on its segment has its centre of mass
        // half a bone from the joint anchor, so every commanded angular
        // acceleration also demands a linear motion the point constraint
        // must cancel — and the controller re-commands it next step.
        //
        // That is a real torque/constraint interaction, not a defect in
        // the control law: it persists with the damping term switched off
        // entirely (3.2 rad/s at ζ=0), and it scales with the anchor
        // offset (0.10 rad/s anchored at the centre of mass, 13 rad/s at
        // 0.15 m). Asserting stillness here would assert something this
        // configuration provably does not do.
        //
        // # Why a joint at its LIMIT is different
        //
        // `a_joint_driven_hard_into_its_limit_stays_bounded` asserts near
        // stillness (0.06 rad/s), and that is not a contradiction: there
        // the CONSTRAINT holds the joint, and a constraint does not
        // re-command itself each step. Mid-range only the controller holds
        // it, and the controller is the thing oscillating.
        //
        // # Scale
        //
        // This fixture drives at 2000 rad/s², 28x the real arm's ceiling,
        // to make the effect measurable. At shipped ceilings it is much
        // smaller — 2.6 rad/s on an arm, 2.2 on a neck, 4.0 on a hip —
        // and a weak joint undershoots rather than oscillating: the arm
        // reaches only 6.8° of a 15° target, which is what running out of
        // authority against the constraint looks like.
        let mut app = physics_app();

        let target = Quat::from_axis_angle(Vec3::X, 15.0_f32.to_radians());
        let (_, body) = spawn_limited_chain(
            &mut app,
            target,
            Some(JointLimits::degrees(30.0, 10.0)),
            PdParams { frequency_hz: 10.0, damping_ratio: 1.0, max_torque: 2000.0 },
        );

        step(&mut app, (3.0 / TIMESTEP) as usize);
        let reached = rotation_of(&app, body);
        let error = reached.angle_between(target).to_degrees();

        assert!(
            error < 10.0,
            "a 15-degree target inside a 30-degree cone should be reached, but the joint \
             settled {error:.2} degrees away from it",
        );
    }

    #[test]
    fn the_shipped_joints_reach_mid_range_targets() {
        // At the ceilings first shipped, the dominant failure here was
        // UNDERSHOOT: the arm reached 6.8 of 15 degrees, out of authority —
        // the same shortfall that later left the walking rig's limbs
        // flailing. This keeps the progress half of what was once a
        // calmness test too.
        //
        // Why not the calmness half: this fixture — one body on an
        // immovable parent — has a documented torque/constraint oscillation
        // that scales with authority (2-4 rad/s at the old ceilings, 6.3 on
        // an arm and 10.4 on a hip once `CEILING_SCALE` raised them 6x; an
        // attempted fix that rotated the body about its joint doubled it).
        // It is a proxy, and it stopped agreeing with the rig it stands in
        // for: the real character at the same ceilings rests within 0.1
        // degrees with ~0 spin, live. The claim is now tested on the whole
        // rig, by `a_ragdoll_settles_quietly_into_a_mid_range_pose`.
        let target = Quat::from_axis_angle(Vec3::X, 15.0_f32.to_radians());

        for bone in [Bone::LeftArm, Bone::Neck, Bone::LeftUpLeg] {
            let params = crate::character::anim::ragdoll::default_joint_params()[bone];

            let mut app = physics_app();
            let (_, body) = spawn_limited_chain(
                &mut app,
                target,
                Some(JointLimits::degrees(30.0, 10.0)),
                params,
            );

            step(&mut app, (3.0 / TIMESTEP) as usize);

            let reached = rotation_of(&app, body).to_axis_angle().1.to_degrees();
            assert!(
                reached > 4.0,
                "{} should move meaningfully toward a 15-degree target, but only \
                 reached {reached:.2} degrees",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_joint_driven_hard_into_its_limit_stays_bounded() {
        // The test Phase 6 deferred, and the reason limits shipped as
        // `None` until now: a PD controller pressed against a hard
        // constraint is a mild analogue of the two-rotational-springs
        // instability that killed this project's earlier avian design.
        //
        // Deliberately adversarial and deliberately *impossible*: the
        // target is 180 degrees away, unreachable through a 30-degree cone,
        // so the controller can never satisfy it and keeps pulling at full
        // authority forever. A real character should not be asked to do
        // this; the point is that even when it is, nothing diverges.
        //
        // # What this asserts
        //
        // GROWTH, not stillness. A joint held against a stop it is still
        // being pushed through does not have to be motionless; what it
        // must not do is gain energy. So the first and last second's peaks
        // must agree to within a few percent — a genuine runaway compounds
        // and would blow that apart within a second or two.
        let mut app = physics_app();

        let (_, body) = spawn_limited_chain(
            &mut app,
            Quat::from_axis_angle(Vec3::X, std::f32::consts::PI),
            Some(JointLimits::degrees(30.0, 10.0)),
            PdParams { frequency_hz: 10.0, damping_ratio: 1.0, max_torque: 2000.0 },
        );

        // Let the body travel to the limit and be caught by it.
        step(&mut app, (1.0 / TIMESTEP) as usize);

        let peaks = per_second_rotation_speed_peaks(&mut app, body, 30.0);

        let first = peaks[0];
        let last = *peaks.last().expect("30 seconds of samples");
        let worst = peaks.iter().copied().fold(0.0f32, f32::max);

        // The real guard: no energy growth. A constraint and a controller
        // feeding each other would show up here immediately.
        assert!(
            last < first * 1.05 + 0.05,
            "energy is growing while the joint is pinned at its limit: first second \
             peaked at {first} rad/s, last second at {last} rad/s; per-second peaks \
             {peaks:?}",
        );

        // And it should be essentially STILL, not merely bounded.
        //
        // This bound was 5.0 while the controller had its `kd * dt`
        // instability and the frame bugs, when a pinned joint genuinely
        // chattered at ~6.5 rad/s. Those are fixed, and the joint now sits
        // at 0.06 rad/s — so the loose bound was documenting a defect that
        // no longer exists, and kept it in the project's notes as an open
        // concern for longer than it was real.
        //
        // Tightened to what the code actually does. A regression in the
        // controller or in the limit handling now fails here instead of
        // hiding under a threshold chosen for a broken build.
        assert!(
            worst < 0.5,
            "a joint at its limit should sit still, but turned at up to {worst} rad/s; \
             per-second peaks {peaks:?}",
        );

        // And it must actually be AT the limit, not somewhere short of it —
        // a joint that never reached its stop would pass every check above
        // by simply not being tested.
        let held = rotation_of(&app, body).to_axis_angle().1.to_degrees();
        assert!(
            (held - 30.0).abs() < 2.0,
            "the joint should be pinned at its 30-degree limit, but sits at {held:.2}",
        );
    }

    #[test]
    fn a_swing_limit_actually_stops_the_joint() {
        // The limit has to DO something — a test that only checks
        // stability would pass just as well if the limits were silently
        // ignored, which is the failure mode worth guarding against.
        //
        // Same unreachable 180-degree target as above; the question is how
        // far the body actually gets.
        let target = Quat::from_axis_angle(Vec3::X, std::f32::consts::PI);
        let params = PdParams { frequency_hz: 10.0, damping_ratio: 1.0, max_torque: 2000.0 };

        let travel_with = |limits: Option<JointLimits>| -> f32 {
            let mut app = physics_app();
            let (_, body) = spawn_limited_chain(&mut app, target, limits, params);
            step(&mut app, (3.0 / TIMESTEP) as usize);
            rotation_of(&app, body).angle_between(Quat::IDENTITY)
        };

        let limited = travel_with(Some(JointLimits::degrees(30.0, 10.0)));
        let free = travel_with(None);

        assert!(
            limited < free * 0.75,
            "a 30-degree cone should visibly restrict travel, but the limited joint \
             reached {:.1} degrees against the free joint's {:.1}",
            limited.to_degrees(),
            free.to_degrees(),
        );

        // And the stop should land near where it was asked to. Generous,
        // because a soft constraint legitimately overshoots under a
        // 2000 rad/s^2 controller and the cone is measured from the
        // joint frame rather than from identity.
        assert!(
            limited < 75.0_f32.to_radians(),
            "a 30-degree cone should not let the joint reach {:.1} degrees",
            limited.to_degrees(),
        );
    }

    /// Spawns a real bone hierarchy and builds a full ragdoll on it, in a
    /// running physics app.
    ///
    /// This is the only test here that exercises `spawn_ragdoll` end to
    /// end — every other one hand-builds one or two bodies, which cannot
    /// catch a mistake in how the rig is traversed.
    fn spawn_full_ragdoll_app() -> (App, Entity, Ragdoll) {
        let mut app = physics_app();
        let (character, ragdoll, _root) = spawn_character_ragdoll(&mut app, Transform::IDENTITY);
        (app, character, ragdoll)
    }

    /// One more character, with its skeleton at `placement` and a full
    /// ragdoll built on it, in an existing physics app. Returns the
    /// character, its ragdoll, and the skeleton's root entity — the one to
    /// move to walk or turn the character.
    fn spawn_character_ragdoll(app: &mut App, placement: Transform) -> (Entity, Ragdoll, Entity) {
        spawn_character_ragdoll_with(app, placement, &RagdollSpawnConfig::default())
    }

    /// [`spawn_character_ragdoll`] with an explicit spawn configuration.
    fn spawn_character_ragdoll_with(
        app: &mut App,
        placement: Transform,
        config: &RagdollSpawnConfig,
    ) -> (Entity, Ragdoll, Entity) {
        // The bone entities, plus a character entity to own the ragdoll.
        let (root, skeleton) = {
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, app.world());
            let built = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                placement,
            );
            queue.apply(app.world_mut());
            built
        };

        // Transform propagation has to run before the spawn reads
        // `GlobalTransform`, or every bone reports the origin.
        app.update();

        let character = app.world_mut().spawn_empty().id();

        let mut queue = bevy::ecs::world::CommandQueue::default();
        let mut state = app.world_mut().query::<&GlobalTransform>();
        let world = app.world();
        let globals = state.query(world);
        let mut commands = Commands::new(&mut queue, world);

        let ragdoll = spawn_ragdoll(&mut commands, character, &skeleton, &globals, config);

        queue.apply(app.world_mut());

        app.world_mut().entity_mut(character).insert((ragdoll.clone(), skeleton));

        (character, ragdoll, root)
    }

    /// Drives a ragdolled character for four seconds with `drive(t)` run
    /// before every frame, and returns the worst tracking error any body
    /// shows after the first second, with the bone it was on.
    fn worst_tracking_while(
        drive: impl Fn(&mut App, Entity, Entity, f32),
    ) -> (f32, &'static str) {
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::ZERO));
        app.add_systems(Update, publish_joint_targets);
        let (character, ragdoll, root) = spawn_character_ragdoll(&mut app, Transform::IDENTITY);
        app.world_mut()
            .entity_mut(character)
            .insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
        step(&mut app, 10);

        let mut worst = (0.0f32, "none");
        for i in 1..=(4.0 / TIMESTEP) as usize {
            let t = i as f32 * TIMESTEP;
            drive(&mut app, character, root, t);
            app.update();
            if t <= 1.0 {
                continue;
            }
            for &bone in Bone::ALL.iter() {
                let Some(body) = ragdoll.bodies[bone] else { continue };
                let target = app.world().get::<JointTarget>(body).unwrap().target;
                let error = rotation_of(&app, body).angle_between(target).to_degrees();
                if error > worst.0 {
                    worst = (error, bone.name());
                }
            }
        }
        worst
    }

    /// A character on the REAL rig (`puppet_base`'s parsed bind pose, as a
    /// parented entity hierarchy) with a full ragdoll, in a physics app.
    /// Returns the character, its ragdoll, the skeleton root to move, and
    /// the rig's geometry.
    fn spawn_real_rig_ragdoll(app: &mut App) -> (Entity, Ragdoll, Entity, RigGeometry) {
        spawn_real_rig_ragdoll_with(app, &RagdollSpawnConfig::default(), Vec3::ZERO)
    }

    /// [`spawn_real_rig_ragdoll`] with a spawn config, and the whole rig
    /// placed at `at` (its bind pose stands on `y = 0` at the origin).
    fn spawn_real_rig_ragdoll_with(
        app: &mut App,
        config: &RagdollSpawnConfig,
        at: Vec3,
    ) -> (Entity, Ragdoll, Entity, RigGeometry) {
        use crate::character::anim::gltf_rig;

        let parsed = gltf_rig::parsed_rig();
        let rig = gltf_rig::puppet_base();

        let root = app
            .world_mut()
            .spawn(
                Transform::from_rotation(parsed.hips_parent_rest_world_rotation)
                    .with_scale(parsed.hips_parent_rest_world_scale)
                    .with_translation(at),
            )
            .id();
        let mut entities = std::collections::HashMap::new();
        for &bone in Bone::ALL.iter() {
            let parent = bone.parent().map_or(root, |parent| entities[&parent]);
            let translation =
                if bone == Bone::Hips { parsed.hips_rest_local_translation } else { rig.offsets[bone] };
            let entity = app
                .world_mut()
                .spawn((
                    Transform::from_translation(translation).with_rotation(rig.bind_rotations[bone]),
                    ChildOf(parent),
                ))
                .id();
            entities.insert(bone, entity);
        }
        let skeleton = gltf_rig::real_skeleton(&parsed, entities);
        app.update();

        let character = app.world_mut().spawn_empty().id();
        let mut queue = bevy::ecs::world::CommandQueue::default();
        let mut state = app.world_mut().query::<&GlobalTransform>();
        let world = app.world();
        let globals = state.query(world);
        let mut commands = Commands::new(&mut queue, world);
        let ragdoll = spawn_ragdoll(&mut commands, character, &skeleton, &globals, config);
        queue.apply(app.world_mut());
        app.world_mut().entity_mut(character).insert((ragdoll.clone(), skeleton));
        (character, ragdoll, root, rig)
    }

    /// One foot body, `puppet_base`'s left, placed as its bind pose stands
    /// 1 cm above a friction-1 floor under gravity, with a sole `block` or
    /// its capsule, left for 2 s: how far it turned and slid.
    fn settle_a_foot(block: Option<SoleBox>) -> (f32, f32) {
        use crate::character::anim::gltf_rig::puppet_base;
        use crate::character::anim::rig::{accumulate_world_rotations, forward_kinematics_on};
        let rig = puppet_base();
        let rest = LocalPose::REST;
        let (world, rotations) = (forward_kinematics_on(&rest, &rig), accumulate_world_rotations(&rest, &rig));
        let (ankle, toe, rotation) = (world[Bone::LeftFoot], world[Bone::LeftToeBase], rotations[Bone::LeftFoot]);
        let centre = (ankle + toe) * 0.5 + Vec3::Y * 0.01;

        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
        let character = app.world_mut().spawn_empty().id();
        let mut queue = bevy::ecs::world::CommandQueue::default();
        let mut commands = Commands::new(&mut queue, app.world());
        let length = ankle.distance(toe);
        let body = spawn_bone_body(
            &mut commands,
            Bone::LeftFoot,
            character,
            BoneBodyPlacement {
                world_position: centre,
                world_rotation: rotation,
                segment_direction: toe - ankle,
                length,
                radius: (length * 0.22).clamp(0.02, 0.09),
                mass: Some(1.0),
                collision_layer: LayerMask(1 << 20),
            },
        );
        if let Some(block) = block {
            commands.entity(body).insert(sole_collider(block, rotation.inverse() * (centre - Vec3::Y * 0.01 - ankle)));
        }
        queue.apply(app.world_mut());
        step(&mut app, 128);
        let now = app.world().get::<Transform>(body).unwrap();
        let turned = (rotation.inverse() * now.rotation).to_axis_angle().1.to_degrees();
        let slid = Vec3::new(now.translation.x - centre.x, 0.0, now.translation.z - centre.z).length();
        (turned, slid)
    }

    #[test]
    fn a_foot_stands_flat_on_its_sole() {
        // The unpinned ragdoll's first spike: its feet, one capsule each from
        // ankle to ball, rolled and skated (0.8 m in 2.5 s, turning 67°).
        // A foot standing on its sole block stays as it was placed.
        let block = sole_blocks(&crate::character::anim::gltf_rig::puppet_base())[0];
        let (turned, slid) = settle_a_foot(Some(block));
        assert!(turned < 2.0, "the foot on its sole turned {turned:.1}°");
        assert!(slid < 0.005, "the foot on its sole slid {:.1} mm", slid * 1e3);
        // Measured: on its sole 0.10° and 0.16 mm; the capsule 26.6° and
        // 25.1 mm.
        let (capsule_turned, _) = settle_a_foot(None);
        assert!(capsule_turned > 10.0, "the capsule foot is the reason: it should roll or tip, turned {capsule_turned:.1}°");
    }

    // SPIKE (plan step 4.1) — an unpinned ragdoll on the real rig, full
    // gravity, full PD strength toward its bind pose, feet on a floor with
    // friction. How long does pose tracking alone keep it standing?
    #[test]
    #[ignore]
    fn probe_unpinned_ragdoll_stands() {
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
        let feet = std::env::var("PROBE_FEET").is_ok().then(|| sole_blocks(&crate::character::anim::gltf_rig::puppet_base()));
        let config = RagdollSpawnConfig { pin_root: false, feet, ..Default::default() };
        let (character, ragdoll, _root, _rig) = spawn_real_rig_ragdoll_with(&mut app, &config, Vec3::Y * 0.05);
        let lowest = |app: &App| {
            ragdoll.bodies.iter().filter_map(|(_, b)| *b).map(|b| app.world().get::<GlobalTransform>(b).unwrap().translation().y).fold(f32::MAX, f32::min)
        };
        if let Ok(hz) = std::env::var("PROBE_HZ").map(|v| v.parse::<f32>().unwrap()) {
            let mut stored = app.world_mut().get_mut::<Ragdoll>(character).unwrap();
            for (_, params) in stored.params.iter_mut() {
                params.frequency_hz = hz;
            }
        }
        let hips = ragdoll.bodies[Bone::Hips].unwrap();
        let at = |app: &App| app.world().get::<GlobalTransform>(hips).unwrap().compute_transform();
        // Full gravity with full muscle: `support_own_weight` (in `Update`,
        // after this frame's physics) zeroes it at full strength, so it is
        // put back before every next step.
        let full_gravity = |app: &mut App| {
            for (_, body) in ragdoll.bodies.iter() {
                if let Some(body) = *body {
                    app.world_mut().get_mut::<GravityScale>(body).unwrap().0 = 1.0;
                }
            }
        };
        let step = |app: &mut App, n: usize| {
            for _ in 0..n {
                full_gravity(app);
                app.update();
            }
        };
        step(&mut app, 1);
        let start = at(&app);
        let foot_start = {
            let t = app.world().get::<GlobalTransform>(ragdoll.bodies[Bone::LeftFoot].unwrap()).unwrap().compute_transform();
            (t.translation, t.rotation)
        };
        println!("start hips {:?}, lowest body centre {:.3}", start.translation, lowest(&app));
        let mut fell = None;
        for frame in 1..=240 {
            step(&mut app, 1);
            let now = at(&app);
            let drop = start.translation.y - now.translation.y;
            let drift = Vec3::new(now.translation.x - start.translation.x, 0.0, now.translation.z - start.translation.z).length();
            let tilt = (start.rotation.inverse() * now.rotation).to_axis_angle().1.to_degrees();
            if frame % 12 == 0 {
                let foot = |bone: Bone| {
                    let t = app.world().get::<GlobalTransform>(ragdoll.bodies[bone].unwrap()).unwrap().compute_transform();
                    (t.translation, t.rotation)
                };
                let (lf, lr) = foot(Bone::LeftFoot);
                println!(
                    "t {:.2}s: hips drop {:+.3} m, drift {:.3} m, tilt {:5.1}° | left foot at {:?}, turned {:5.1}° from spawn",
                    frame as f32 * TIMESTEP, drop, drift, tilt, lf, (foot_start.1.inverse() * lr).to_axis_angle().1.to_degrees()
                );
                let _ = foot_start.0;
            }
            if fell.is_none() && drop > 0.1 {
                fell = Some(frame as f32 * TIMESTEP);
            }
        }
        println!("fell (hips down 10 cm) at {fell:?}");
    }

    #[test]
    fn a_limb_body_carries_winters_mass_distribution() {
        // Table 4.1, on the real rig: a thigh's mass is 10% of the body, its
        // centre 43.3% of the way from hip to knee (not the capsule's
        // midpoint), and its transverse inertia m·(0.323·L)². Read back from
        // what avian actually computed, so a collider still contributing its
        // own uniform-density properties would fail here.
        let mut app = physics_app();
        let (character, ragdoll, _root, _rig) = spawn_real_rig_ragdoll(&mut app);
        step(&mut app, 1);

        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap();
        let joint = |bone| app.world().get::<GlobalTransform>(skeleton.entity(bone)).unwrap().translation();
        let (hip, knee) = (joint(Bone::LeftUpLeg), joint(Bone::LeftLeg));
        let length = hip.distance(knee);

        let body = ragdoll.bodies[Bone::LeftUpLeg].expect("a thigh body");
        let world = app.world();
        let transform = world.get::<GlobalTransform>(body).unwrap().compute_transform();
        let mass = world.get::<ComputedMass>(body).unwrap().value();
        let centre = transform.translation + transform.rotation * world.get::<ComputedCenterOfMass>(body).unwrap().0;
        let (principal, _) = world.get::<ComputedAngularInertia>(body).unwrap().principal_angular_inertia_with_local_frame();

        let expected_mass = default_body_masses()[Bone::LeftUpLeg];
        assert!((mass - expected_mass).abs() < 1.0e-3, "thigh mass {mass} kg, expected {expected_mass}");
        let wanted = hip + (knee - hip) * 0.433;
        assert!(centre.distance(wanted) < 0.005, "thigh COM {:.3} m from 43.3% down the thigh", centre.distance(wanted));
        let transverse = expected_mass * (0.323 * length).powi(2);
        assert!(
            (principal.max_element() - transverse).abs() < transverse * 0.01,
            "transverse inertia {} against Winter's {transverse}",
            principal.max_element(),
        );
    }

    #[test]
    fn on_the_real_rig_a_walking_arm_swings_as_far_as_its_animation() {
        // The one tracking check on the real character's proportions,
        // walking its own gait cycle. A saturated controller in a chain
        // overshoots: at x6 ceilings the upper arm swung 71.7 degrees
        // against a 57.3-degree target swing (what set `CEILING_SCALE` to
        // 12); at x12 it swings 57.4. That was the gait's OLD arm swing,
        // mistimed and axis-skewed; the corrected one is gentler and x6
        // would now do for the arms alone — at x1 the arm is 92 degrees
        // off, so this still catches saturation.
        use crate::character::anim::gait::{walk_pose_on, GaitParams};
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        app.add_systems(Update, publish_joint_targets);
        let (character, ragdoll, root, rig) = spawn_real_rig_ragdoll(&mut app);
        let stood =
            stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        app.world_mut().entity_mut(character).insert(AnimPose::settled_on(&stood));
        let start = *app.world().get::<Transform>(root).unwrap();
        step(&mut app, (3.0 / TIMESTEP) as usize);

        let params = GaitParams::default();
        let arm = ragdoll.bodies[Bone::LeftArm].unwrap();
        let (mut targets, mut bodies) = (Vec::new(), Vec::new());
        for i in 1..=(5.0 / TIMESTEP) as usize {
            let t = i as f32 * TIMESTEP;
            let pose = walk_pose_on(t.fract(), &params, &stood, &rig);
            *app.world_mut().get_mut::<AnimPose>(character).unwrap() = AnimPose::settled_on(&pose);
            app.world_mut().get_mut::<Transform>(root).unwrap().translation =
                start.translation + Vec3::new(0.0, 0.0, t);
            app.update();
            if t > 1.0 {
                targets.push(app.world().get::<JointTarget>(arm).unwrap().target);
                bodies.push(rotation_of(&app, arm));
            }
        }

        let range = |qs: &[Quat]| {
            qs.iter()
                .flat_map(|a| qs.iter().map(move |b| a.angle_between(*b)))
                .fold(0.0f32, f32::max)
                .to_degrees()
        };
        let (asked, swung) = (range(&targets), range(&bodies));
        let worst = targets
            .iter()
            .zip(&bodies)
            .map(|(target, body)| target.angle_between(*body).to_degrees())
            .fold(0.0f32, f32::max);
        assert!(asked > 20.0, "setup: the walk should swing the arm, only {asked:.1} degrees");

        // Per-sample tracking is the claim. A range comparison alone was
        // too blunt once the gait's arm swing was corrected to a human
        // size: the body's range may exceed the target's by up to twice the
        // ordinary lag (measured 44.5 against 34.4 degrees with the worst
        // sample only 7.2 off), which reads as overshoot and is not.
        assert!(
            worst < 10.0,
            "the arm should follow its animation, but was {worst:.1} degrees off at worst",
        );
        // ...and a loose guard against the gross overshoot saturation gives.
        assert!(
            swung < asked * 1.5,
            "the arm swung {swung:.1} degrees against an animation of {asked:.1} — overshoot",
        );
    }

    #[test]
    fn a_ragdoll_settles_quietly_into_a_mid_range_pose() {
        // The rig anyone will use, at the ceilings it ships with: spawned at
        // rest, then asked for the relaxed stance — arms travelling ~70
        // degrees to mid-range targets, the configuration whose isolated
        // single-limb proxy oscillates — under gravity. It must end up on
        // target and quiet.
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        app.add_systems(Update, publish_joint_targets);
        let (character, ragdoll, _root) = spawn_character_ragdoll(&mut app, Transform::IDENTITY);
        app.world_mut()
            .entity_mut(character)
            .insert(AnimPose::settled_on(&crate::character::anim::poses::relaxed_stand()));

        step(&mut app, (3.0 / TIMESTEP) as usize);

        let (mut worst_error, mut worst_spin) = ((0.0f32, "none"), (0.0f32, "none"));
        for _ in 0..(2.0 / TIMESTEP) as usize {
            app.update();
            for &bone in Bone::ALL.iter() {
                let Some(body) = ragdoll.bodies[bone] else { continue };
                let target = app.world().get::<JointTarget>(body).unwrap().target;
                let error = rotation_of(&app, body).angle_between(target).to_degrees();
                let spin = app.world().get::<AngularVelocity>(body).unwrap().0.length();
                if error > worst_error.0 {
                    worst_error = (error, bone.name());
                }
                if spin > worst_spin.0 {
                    worst_spin = (spin, bone.name());
                }
            }
        }

        assert!(
            worst_error.0 < 2.0 && worst_spin.0 < 0.5,
            "the rig should settle on its relaxed stance, but {} is {:.2} degrees off and {} \
             turning at {:.2} rad/s",
            worst_error.1,
            worst_error.0,
            worst_spin.1,
            worst_spin.0,
        );
    }

    #[test]
    fn a_walking_character_does_not_set_its_limbs_ringing() {
        // A steady 1 m/s walk with a still pose: nothing should move but
        // the whole body. Live, the walk left forearms, feet and head
        // flailing 20-176 degrees off while the torso tracked; headless,
        // the head rang 39.8 degrees long after the start — both saturated
        // controllers, see `CEILING_SCALE`.
        let (error, bone) = worst_tracking_while(|app, _, root, t| {
            app.world_mut().get_mut::<Transform>(root).unwrap().translation = Vec3::new(0.0, 0.0, -t);
        });
        assert!(error < 3.0, "walking set {bone} ringing {error:.1} degrees off its target");
    }

    #[test]
    fn the_ragdoll_can_follow_a_brisk_arm_swing() {
        // A 1 Hz, 0.5 rad swing — an ordinary walking arm. A ball joint
        // carries no torque, so the forearm holds its angle against the
        // swinging elbow by its own controller alone; at the authored
        // ceilings it saturated and flailed 80 degrees off.
        let (error, bone) = worst_tracking_while(|app, character, _, t| {
            let mut pose = crate::character::anim::poses::rest();
            let swing = 0.5 * (std::f32::consts::TAU * t).sin();
            pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Z, swing));
            *app.world_mut().get_mut::<AnimPose>(character).unwrap() = AnimPose::settled_on(&pose);
        });
        // 15 rather than a few: a critically damped 8 Hz controller chasing
        // a moving target lags by design — about 8 degrees at this speed.
        assert!(error < 15.0, "an arm swing left {bone} {error:.1} degrees behind its target");
    }

    #[test]
    fn a_pinned_ragdoll_walks_and_turns_with_its_character() {
        // Two bugs no standing test could see. The pinned root is
        // kinematic, and nothing moved it: a walking character left its
        // physics body where it spawned. And the targets were rooted at the
        // BIND-time rotation of the hips' parent, so the moment the
        // character turned, every one was off by the turn.
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::ZERO));
        // Publishing is the plugin's, registered here because the harness
        // runs without the kinematic stack it orders against.
        app.add_systems(Update, publish_joint_targets);

        let (character, ragdoll, root) = spawn_character_ragdoll(&mut app, Transform::IDENTITY);
        app.world_mut()
            .entity_mut(character)
            .insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
        step(&mut app, 10);

        // Walk 1 m along X while turning a quarter turn, over two seconds,
        // then stand for one.
        let steps = (2.0 / TIMESTEP) as usize;
        for i in 1..=steps {
            let t = i as f32 / steps as f32;
            *app.world_mut().get_mut::<Transform>(root).unwrap() =
                Transform::from_xyz(t, 0.0, 0.0).with_rotation(Quat::from_rotation_y(t * FRAC_PI_2));
            app.update();
        }
        // One second after the turn stops, the whole rig must be back on
        // target. At the authored ceilings (`CEILING_SCALE` = 1) the left
        // arm rang for about two seconds here — 13.4 degrees at 1 s, 5.7
        // at 2 s — the same saturation `CEILING_SCALE` documents; it passes
        // from x6 up.
        step(&mut app, (1.0 / TIMESTEP) as usize);

        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap().clone();
        let hips = ragdoll.bodies[Bone::Hips].unwrap();
        let hips_error = app
            .world()
            .get::<Position>(hips)
            .unwrap()
            .0
            .distance(app.world().get::<KinematicRoot>(hips).unwrap().position);
        let hips_travel = app.world().get::<Position>(hips).unwrap().0.x;
        assert!(
            hips_error < 0.01 && hips_travel > 0.9,
            "the pinned root should walk with the character: it is {hips_error:.3} m off its \
             target, having travelled {hips_travel:.3} m of 1 m",
        );

        // Every target is where the renderer draws its bone, in world
        // space — and every body got there.
        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let target = app.world().get::<JointTarget>(body).unwrap().target;
            let drawn = app.world().get::<GlobalTransform>(skeleton.entity(bone)).unwrap().rotation();
            let frame = target.angle_between(drawn).to_degrees();
            let tracking = rotation_of(&app, body).angle_between(target).to_degrees();
            assert!(
                frame < 0.5 && tracking < 2.0,
                "{}: target is {frame:.1} degrees from the drawn bone, body {tracking:.1} from \
                 its target",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_released_ragdoll_falls_with_its_momentum_and_is_drawn_where_it_lies() {
        // H2: a pinned ragdoll carried forward at 1 m/s, then let go. It
        // must keep going (the kinematic root's velocity is the hips'), come
        // down onto the floor and stop there, and the skeleton must be
        // drawn on the bodies, root and all. At twelve substeps: at avian's
        // six a fallen body's resting jitter kept it awake (see the
        // gallery's `SubstepCount`).
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81)).insert_resource(SubstepCount(12));
        app.add_systems(Update, publish_joint_targets);
        app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let config = RagdollSpawnConfig { feet: Some(sole_blocks(&rig)), ..Default::default() };
        let (character, ragdoll, root, _) = spawn_real_rig_ragdoll_with(&mut app, &config, Vec3::Y * 0.01);
        app.world_mut().entity_mut(character).insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
        step(&mut app, 10);

        let hips = ragdoll.bodies[Bone::Hips].unwrap();
        let start = app.world().get::<Position>(hips).unwrap().0;
        // Carried forward (+X) at 1 m/s for a quarter second.
        let carried = (0.25 / TIMESTEP) as usize;
        for _ in 0..carried {
            app.world_mut().get_mut::<Transform>(root).unwrap().translation.x += TIMESTEP;
            app.update();
        }
        let released_at = app.world().get::<Position>(hips).unwrap().0;
        app.world_mut().get_mut::<Ragdoll>(character).unwrap().fall(FALL_TONE, FALL_DAMPING);
        step(&mut app, (0.1 / TIMESTEP) as usize);
        let coasted = app.world().get::<Position>(hips).unwrap().0.x - released_at.x;
        assert!(
            app.world().get::<RigidBody>(hips).unwrap().is_dynamic() && app.world().get::<KinematicRoot>(hips).is_none(),
            "the root should be released"
        );
        // 1 m/s for 0.1 s: most of 10 cm.
        assert!(coasted > 0.06, "the falling body kept only {:.1} cm of the carried 1 m/s over 0.1 s", coasted * 100.0);

        // The written skeleton stays unit-length throughout: the read-back's
        // own output fed back as the next frame's root, and drifted to a
        // norm of 1.03 lying down (scaling the skeleton 6%, drawing it 8°
        // off its bodies).
        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap().clone();
        // Five seconds: it sleeps at 4.2 s (`probe_fall_damping`).
        for _ in 0..(5.0 / TIMESTEP) as usize {
            app.update();
            let norm = app.world().get::<Transform>(skeleton.entity(Bone::Hips)).unwrap().rotation.length();
            assert!((norm - 1.0).abs() < 1.0e-4, "the drawn hips rotation has norm {norm}");
        }
        let bodies: Vec<Entity> = ragdoll.bodies.iter().filter_map(|(_, body)| *body).collect();
        let hips_now = app.world().get::<Position>(hips).unwrap().0;
        assert!(hips_now.is_finite(), "the fall blew up");
        // At rest it sleeps, and stays put: awake, the solver's resting
        // jitter walked it 9 mm/s (`FALLEN_SLEEP`).
        let asleep = bodies.iter().filter(|&&body| app.world().get::<Sleeping>(body).is_some()).count();
        assert_eq!(asleep, bodies.len(), "5 s after the fall, only {asleep} of {} bodies sleep", bodies.len());
        step(&mut app, (3.0 / TIMESTEP) as usize);
        let crept = app.world().get::<Position>(hips).unwrap().0.distance(hips_now);
        assert!(crept < 1.0e-3, "the fallen body crept {:.1} mm in 3 s", crept * 1e3);
        assert!(hips_now.y < 0.35, "the hips should lie low after 5 s of falling, not {:.2} m up (stood at {:.2})", hips_now.y, start.y);
        for &body in &bodies {
            let at = app.world().get::<Position>(body).unwrap().0;
            let speed = app.world().get::<LinearVelocity>(body).unwrap().0.length();
            assert!(at.y > -0.02, "a body sank {:.1} cm into the floor", -at.y * 100.0);
            assert!(speed < 0.2, "a body still moves at {speed:.2} m/s after 5 s");
        }

        // Drawn where it lies: the rendered hips joint on the hips body,
        // and every simulated bone at its body's rotation.
        let offset = app.world().get::<Ragdoll>(character).unwrap().fall.unwrap().root_offset.unwrap();
        let body_rotation = app.world().get::<Rotation>(hips).unwrap().0;
        let drawn = app.world().get::<GlobalTransform>(skeleton.entity(Bone::Hips)).unwrap().translation();
        assert!(
            drawn.distance(hips_now - body_rotation * offset) < 0.01,
            "the drawn hips are {:.1} cm off the body",
            drawn.distance(hips_now - body_rotation * offset) * 100.0
        );
        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let drawn = app.world().get::<GlobalTransform>(skeleton.entity(bone)).unwrap().rotation();
            let error = rotation_of(&app, body).angle_between(drawn).to_degrees();
            assert!(error < 1.0, "{} is drawn {error:.1}° off its body", bone.name());
        }
    }

    #[test]
    fn a_blow_pushes_the_standing_balance_by_its_share_of_the_body() {
        // A hit on the chest moves the whole body by the chest's share of
        // the momentum, in the rig's own forward/left: that push is what
        // lets the balance absorb it, step, or fall. A blow straight up
        // pushes nothing.
        let mut app = physics_app();
        let (character, ragdoll, _root, _) = spawn_real_rig_ragdoll(&mut app);
        app.world_mut().entity_mut(character).insert(crate::character::anim::balance::Balance::default());
        app.world_mut().entity_mut(character).insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
        step(&mut app, 3);
        let mass = |app: &App, body: Entity| app.world().get::<ComputedMass>(body).unwrap().value();
        let total: f32 = ragdoll.bodies.iter().filter_map(|(_, body)| *body).map(|body| mass(&app, body)).sum();
        let share = mass(&app, ragdoll.bodies[Bone::Spine2].unwrap()) / total;
        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap().clone();
        let forward = {
            let mut state = bevy::ecs::system::SystemState::<(Query<&Transform>, TransformHelper)>::new(app.world_mut());
            let (transforms, live) = state.get(app.world()).unwrap();
            rig_geometry(&skeleton, &transforms, &live).forward()
        };
        let pushed = |app: &App| {
            let balance = app.world().get::<crate::character::anim::balance::Balance>(character).unwrap();
            balance.velocity + balance.pending_push()
        };

        app.world_mut().write_message(RagdollHit::new(character, Bone::Spine2, Vec3::Y * 5.0));
        step(&mut app, 1);
        assert!(pushed(&app).length() < 1.0e-5, "a blow straight up pushed {}", pushed(&app));

        app.world_mut().write_message(RagdollHit::new(character, Bone::Spine2, forward * 5.0));
        step(&mut app, 1);
        let push = pushed(&app);
        assert!(
            (push.x - 5.0 * share).abs() < 1.0e-3 && push.y.abs() < 1.0e-3,
            "a 5 m/s blow forward on the chest ({:.0}% of the body) pushed {push}, not ({:.3}, 0)",
            share * 100.0,
            5.0 * share
        );
    }

    #[test]
    fn a_fallen_ragdoll_gets_up_and_is_pinned_again() {
        // H3: fallen and at rest, `get_up` lies still through its delay,
        // blends the skeleton smoothly up to the animation, then sets every
        // body back onto its bone and pins the root; the ragdoll then holds.
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81)).insert_resource(SubstepCount(12));
        app.add_systems(Update, publish_joint_targets);
        app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let config = RagdollSpawnConfig { feet: Some(sole_blocks(&rig)), ..Default::default() };
        let (character, ragdoll, _root, _) = spawn_real_rig_ragdoll_with(&mut app, &config, Vec3::Y * 0.01);
        // The character stands on its ground at its entity's height (the
        // rise keeps the body's ends above it).
        app.world_mut()
            .entity_mut(character)
            .insert((AnimPose::settled_on(&crate::character::anim::poses::rest()), Transform::default()));
        step(&mut app, 10);
        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap().clone();
        let drawn_hips = |app: &App| app.world().get::<GlobalTransform>(skeleton.entity(Bone::Hips)).unwrap().translation();
        let stood = drawn_hips(&app);

        app.world_mut().get_mut::<Ragdoll>(character).unwrap().fall(FALL_TONE, FALL_DAMPING);
        let mut frames = 0;
        while !app.world().get::<Ragdoll>(character).unwrap().fall.unwrap().at_rest {
            app.update();
            frames += 1;
            assert!(frames < (8.0 / TIMESTEP) as usize, "never came to rest");
        }
        assert!(app.world_mut().get_mut::<Ragdoll>(character).unwrap().get_up(0.5), "get_up refused a body at rest");
        let lying = drawn_hips(&app);
        assert!(stood.y - lying.y > 0.4, "it should have been lying: hips {:.2} m, stood {:.2}", lying.y, stood.y);

        // The delay: still lying.
        step(&mut app, (0.45 / TIMESTEP) as usize);
        assert!(drawn_hips(&app).distance(lying) < 1.0e-3, "moved {:.1} mm during the delay", drawn_hips(&app).distance(lying) * 1e3);
        // How it lies has been read, and its keys chosen.
        let stored = app.world().get::<Ragdoll>(character).unwrap().clone();
        let rise = stored.fall.unwrap().rise.unwrap();
        assert!(rise.lying.is_some() && stored.rise_keys.len() == 2, "no get-up keys chosen: {rise:?}");
        // The rise: through its keys, smoothly, no end joint under the floor.
        let mut previous = drawn_hips(&app);
        let mut frames = 0;
        while app.world().get::<Ragdoll>(character).unwrap().is_falling() {
            app.update();
            let now = drawn_hips(&app);
            assert!(now.distance(previous) < 0.03, "the hips jumped {:.1} mm in a frame", now.distance(previous) * 1e3);
            for bone in [Bone::LeftToeBase, Bone::RightToeBase, Bone::LeftHand, Bone::RightHand, Bone::Head] {
                let y = app.world().get::<GlobalTransform>(skeleton.entity(bone)).unwrap().translation().y;
                assert!(y > -0.01, "{} went {:.0} mm under the floor while rising", bone.name(), -y * 1e3);
            }
            previous = now;
            frames += 1;
            assert!(frames < ((stored.rise_seconds() + 0.5) / TIMESTEP) as usize, "the rise never ended");
        }
        assert!(
            frames as f32 * TIMESTEP > stored.rise_seconds() - 0.1,
            "the rise took {:.2} s, its keys {:.2} s",
            frames as f32 * TIMESTEP,
            stored.rise_seconds()
        );
        assert!(drawn_hips(&app).distance(stood) < 0.01, "stood up {:.1} cm from where it stood", drawn_hips(&app).distance(stood) * 100.0);

        // Pinned again, every body on its bone, and it holds.
        let hips = ragdoll.bodies[Bone::Hips].unwrap();
        step(&mut app, 2);
        assert!(app.world().get::<KinematicRoot>(hips).is_some(), "the root is not pinned again");
        for seconds in [0.0, 2.0] {
            step(&mut app, (seconds / TIMESTEP) as usize);
            for &bone in Bone::ALL.iter() {
                let Some(body) = ragdoll.bodies[bone] else { continue };
                let drawn = app.world().get::<GlobalTransform>(skeleton.entity(bone)).unwrap();
                let wanted = drawn.translation() + drawn.rotation() * ragdoll.body_offsets[bone];
                let at = app.world().get::<Position>(body).unwrap().0;
                let turned = rotation_of(&app, body).angle_between(drawn.rotation()).to_degrees();
                assert!(
                    at.distance(wanted) < 0.02 && turned < 3.0,
                    "{}: {:.1} cm and {turned:.1}° off its bone {seconds} s after getting up",
                    bone.name(),
                    at.distance(wanted) * 100.0
                );
            }
        }
        assert!(drawn_hips(&app).distance(stood) < 0.01, "the standing ragdoll drifted {:.1} cm", drawn_hips(&app).distance(stood) * 100.0);
    }

    #[test]
    fn a_fallen_body_that_never_sleeps_by_itself_is_put_to_rest() {
        // Some landings never drop under the sleep bound by themselves and
        // creep for as long as they lie (4-25 mm/s live). With avian's own
        // sleeping made impossible, `rest_fallen_ragdolls` alone must stop
        // it: marked at rest, asleep, and still.
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81)).insert_resource(SubstepCount(12));
        app.add_systems(Update, publish_joint_targets);
        app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let config = RagdollSpawnConfig { feet: Some(sole_blocks(&rig)), ..Default::default() };
        let (character, ragdoll, _root, _) = spawn_real_rig_ragdoll_with(&mut app, &config, Vec3::Y * 0.01);
        app.world_mut().entity_mut(character).insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
        step(&mut app, 10);
        app.world_mut().get_mut::<Ragdoll>(character).unwrap().fall(FALL_TONE, FALL_DAMPING);
        step(&mut app, 2);
        let bodies: Vec<Entity> = ragdoll.bodies.iter().filter_map(|(_, body)| *body).collect();
        for &body in &bodies {
            app.world_mut().entity_mut(body).insert(SleepThreshold { linear: 0.0, angular: 0.0 });
        }
        let mut rested = None;
        for i in 1..=(8.0 / TIMESTEP) as usize {
            app.update();
            if rested.is_none() && app.world().get::<Ragdoll>(character).unwrap().fall.unwrap().at_rest {
                rested = Some(i as f32 * TIMESTEP);
            }
        }
        assert!(rested.is_some(), "the fallen body was never put to rest");
        let asleep = bodies.iter().filter(|&&body| app.world().get::<Sleeping>(body).is_some()).count();
        assert_eq!(asleep, bodies.len(), "put to rest, but only {asleep} of {} bodies sleep", bodies.len());
        let hips = ragdoll.bodies[Bone::Hips].unwrap();
        let before = app.world().get::<Position>(hips).unwrap().0;
        step(&mut app, (2.0 / TIMESTEP) as usize);
        let crept = app.world().get::<Position>(hips).unwrap().0.distance(before);
        assert!(crept < 1.0e-3, "at rest since {rested:?} s, it still crept {:.1} mm in 2 s", crept * 1e3);
    }

    // Fall tone sweep at 12 substeps: peak limb speed after the first
    // 0.3 s, when every body sleeps, and where the hips end up.
    #[test]
    #[ignore]
    fn probe_fall_damping() {
        for damping in [0.0, 1.0, 3.0, 10.0] {
            for carry in [Vec3::X, Vec3::NEG_Z, Vec3::NEG_X] {
                let mut app = physics_app();
                app.insert_resource(Gravity(Vec3::NEG_Y * 9.81)).insert_resource(SubstepCount(12));
                app.add_systems(Update, publish_joint_targets);
                app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
                let rig = crate::character::anim::gltf_rig::puppet_base();
                let config = RagdollSpawnConfig { feet: Some(sole_blocks(&rig)), ..Default::default() };
                let (character, ragdoll, root, _) = spawn_real_rig_ragdoll_with(&mut app, &config, Vec3::Y * 0.01);
                app.world_mut().entity_mut(character).insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
                step(&mut app, 10);
                for _ in 0..(0.25 / TIMESTEP) as usize {
                    app.world_mut().get_mut::<Transform>(root).unwrap().translation += carry * TIMESTEP;
                    app.update();
                }
                app.world_mut().get_mut::<Ragdoll>(character).unwrap().fall(0.0, damping);
                let bodies: Vec<Entity> = ragdoll.bodies.iter().filter_map(|(_, b)| *b).collect();
                let (mut peak, mut slept) = (0.0f32, None);
                for i in 1..=(8.0 / TIMESTEP) as usize {
                    app.update();
                    if i as f32 * TIMESTEP > 0.3 {
                        peak = bodies.iter().map(|&b| app.world().get::<LinearVelocity>(b).unwrap().0.length()).fold(peak, f32::max);
                    }
                    if slept.is_none() && bodies.iter().all(|&b| app.world().get::<Sleeping>(b).is_some()) {
                        slept = Some(i as f32 * TIMESTEP);
                    }
                }
                let hips = app.world().get::<Position>(ragdoll.bodies[Bone::Hips].unwrap()).unwrap().0;
                println!("damping {damping:4.1} carry {carry}: peak limb {peak:.2} m/s, asleep at {slept:?} s, hips at {:.2} m", hips.y);
            }
        }
    }

    // Cost of one ragdoll's physics step at 6 and 12 substeps, standing
    // (pinned) and falling. `cargo test --release -- --ignored --nocapture`.
    #[test]
    #[ignore]
    fn probe_ragdoll_substep_cost() {
        for substeps in [6, 12] {
            let mut app = physics_app();
            app.insert_resource(Gravity(Vec3::NEG_Y * 9.81)).insert_resource(SubstepCount(substeps));
            app.add_systems(Update, publish_joint_targets);
            app.world_mut().spawn((RigidBody::Static, Collider::half_space(Vec3::Y), Friction::new(1.0), Transform::default()));
            let rig = crate::character::anim::gltf_rig::puppet_base();
            let config = RagdollSpawnConfig { feet: Some(sole_blocks(&rig)), ..Default::default() };
            let (character, _ragdoll, _root, _) = spawn_real_rig_ragdoll_with(&mut app, &config, Vec3::Y * 0.01);
            app.world_mut().entity_mut(character).insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
            step(&mut app, 30);
            let time = |app: &mut App, frames: usize| {
                let mut samples: Vec<f64> = (0..frames)
                    .map(|_| {
                        let start = std::time::Instant::now();
                        app.update();
                        start.elapsed().as_secs_f64() * 1e3
                    })
                    .collect();
                samples.sort_by(f64::total_cmp);
                (samples[frames / 2], samples[frames * 99 / 100])
            };
            let standing = time(&mut app, 320);
            app.world_mut().get_mut::<Ragdoll>(character).unwrap().fall(FALL_TONE, FALL_DAMPING);
            let falling = time(&mut app, 128);
            println!("{substeps} substeps: standing p50 {:.3} p99 {:.3} ms, falling p50 {:.3} p99 {:.3} ms", standing.0, standing.1, falling.0, falling.1);
        }
    }

    #[test]
    fn ragdoll_layers_are_single_distinct_bits_from_the_pool() {
        // The pure selection, not the shared counter: tests run in
        // parallel and other ragdoll tests draw from it too.
        let layers: Vec<u32> = (0..16).map(|n| ragdoll_layer(n).0).collect();
        for &layer in &layers {
            assert_eq!(layer.count_ones(), 1, "a ragdoll's layer should be one bit: {layer:#x}");
            assert_eq!(layer & !RAGDOLL_LAYER_POOL.0, 0, "{layer:#x} is outside the pool");
        }
        // Sixteen ragdolls use all sixteen layers, then it wraps.
        let union = layers.iter().fold(0, |acc, layer| acc | layer);
        assert_eq!(union, RAGDOLL_LAYER_POOL.0, "sixteen ragdolls should use sixteen layers");
        assert_eq!(ragdoll_layer(16), ragdoll_layer(0), "the seventeenth should wrap");
    }

    #[test]
    fn two_characters_ragdolls_collide_with_each_other_but_not_themselves() {
        // Each ragdoll filters out only its OWN layer. With one shared
        // layer that every ragdoll filtered out, a character's bodies
        // ignored each other — needed — and every other character's too.
        let mut app = physics_app();
        let (_a, first, _) = spawn_character_ragdoll(&mut app, Transform::IDENTITY);
        // Close enough that the two torsos overlap.
        let (_b, second, _) =
            spawn_character_ragdoll(&mut app, Transform::from_xyz(0.15, 0.0, 0.0));

        let bodies = |ragdoll: &Ragdoll| -> Vec<Entity> {
            Bone::ALL.iter().filter_map(|&bone| ragdoll.bodies[bone]).collect()
        };
        let (mine, theirs) = (bodies(&first), bodies(&second));
        for &body in mine.iter().chain(&theirs) {
            app.world_mut().entity_mut(body).insert(CollidingEntities::default());
        }

        step(&mut app, 3);

        let mut across = 0;
        for &body in &mine {
            let touching = app.world().get::<CollidingEntities>(body).unwrap();
            for other in touching.iter() {
                assert!(
                    !mine.contains(other),
                    "a ragdoll's own bodies should never touch, but two of the first one's did",
                );
                across += usize::from(theirs.contains(other));
            }
        }
        assert!(across > 0, "overlapping characters' ragdolls should collide, but passed through");
    }

    #[test]
    fn a_full_ragdoll_simulates_most_of_the_rig() {
        let (_app, _character, ragdoll) = spawn_full_ragdoll_app();

        let simulated: Vec<&str> = Bone::ALL
            .iter()
            .filter(|&&bone| ragdoll.bodies[bone].is_some())
            .map(|bone| bone.name())
            .collect();

        // The layout's fourteen bodies, minus this synthetic rig's two
        // 0.07 m ankle stubs (`LeftLeg`/`RightLeg`), which fall under
        // `minimum_bone_length`. A floor rather than an exact count keeps
        // this from breaking on a proportions tweak, while still failing
        // hard if the traversal silently stops simulating the rig.
        assert!(
            simulated.len() >= 12,
            "a full ragdoll should simulate most of the rig, got {} bones: {simulated:?}",
            simulated.len(),
        );

        // The load-bearing chain specifically must be there.
        for bone in
            // Not `LeftLeg`: on this synthetic rig it is a 0.07 m ankle stub,
            // deliberately skipped — see `RagdollSpawnConfig::minimum_bone_length`.
            [Bone::Hips, Bone::Spine, Bone::LeftUpLeg, Bone::LeftFoot, Bone::LeftArm]
        {
            assert!(
                ragdoll.bodies[bone].is_some(),
                "{} must be simulated, but has no body",
                bone.name(),
            );
        }

        // Hands and toes are too small to simulate; the neck rides between
        // the upper torso and the head.
        for bone in [Bone::Neck, Bone::LeftHand, Bone::LeftToeBase] {
            assert!(
                ragdoll.bodies[bone].is_none(),
                "{} should ride on its parent's body, not have its own",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_full_ragdoll_holds_itself_together_under_gravity() {
        // The integration proof: a complete simulated skeleton, every
        // joint limited, driven by nothing at all, must fall as one
        // connected body rather than exploding into parts.
        let (mut app, character, ragdoll) = spawn_full_ragdoll_app();

        // Limp, so this tests the CONSTRAINTS alone — no controller
        // holding anything together.
        app.world_mut().entity_mut(character).get_mut::<Ragdoll>().unwrap().set_strength(0.0);
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));

        let bodies: Vec<Entity> = Bone::ALL
            .iter()
            .filter_map(|&bone| ragdoll.bodies[bone])
            .collect();

        let spread_of = |app: &App| -> f32 {
            let positions: Vec<Vec3> = bodies
                .iter()
                .map(|&body| app.world().get::<Transform>(body).unwrap().translation)
                .collect();
            let centre: Vec3 =
                positions.iter().copied().sum::<Vec3>() / positions.len() as f32;
            positions.iter().map(|p| p.distance(centre)).fold(0.0f32, f32::max)
        };

        let initial_spread = spread_of(&app);
        step(&mut app, 180);

        for &body in &bodies {
            let transform = *app.world().get::<Transform>(body).unwrap();
            assert!(
                transform.translation.is_finite() && transform.rotation.is_finite(),
                "a ragdoll body went non-finite: {transform:?}",
            );
        }

        // A connected body falling under gravity keeps roughly its own
        // extent. One that has come apart flies outward without bound —
        // which is what an unsatisfiable set of constraints looks like.
        let final_spread = spread_of(&app);
        assert!(
            final_spread < initial_spread * 2.0,
            "the ragdoll came apart: bodies spread {final_spread:.2} m from their centre \
             against an initial {initial_spread:.2} m",
        );
    }

    #[test]
    fn a_full_ragdoll_does_not_fall_through_the_world() {
        // The test the cohesion check above could not be: a ragdoll falling
        // as ONE connected body keeps its spread constant, so
        // `holds_itself_together` passes perfectly while the character
        // drops out of the world.
        //
        // That is not hypothetical. Before `RagdollSpawnConfig::pin_root`
        // existed, the gallery's ragdoll was measured (via BRP, on the live
        // ECS) at y = -1930 m after a few seconds — every joint correctly
        // oriented, the whole assembly in free fall, because the PD
        // controller drives rotation and nothing was driving position.
        //
        // It took a screenshot to notice. This test is the cheap version.
        let (mut app, _character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));

        let hips = ragdoll.bodies[Bone::Hips].expect("the root must be simulated");
        let start = app.world().get::<Transform>(hips).unwrap().translation;

        step(&mut app, 300);

        let end = app.world().get::<Transform>(hips).unwrap().translation;
        let dropped = start.y - end.y;

        assert!(
            dropped < 0.05,
            "the ragdoll root fell {dropped:.3} m in five seconds — nothing is holding \
             the character up (started at y={:.3}, now y={:.3})",
            start.y,
            end.y,
        );
    }

    #[test]
    fn an_unpinned_ragdoll_is_free_to_fall() {
        // The other half of the switch: `pin_root: false` must genuinely
        // release the root, or the config option is a lie. A consumer with
        // its own character controller needs this.
        let mut app = physics_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));

        let (_root, skeleton) = {
            let mut queue = bevy::ecs::world::CommandQueue::default();
            let mut commands = Commands::new(&mut queue, app.world());
            let built = crate::character::skeleton::tests::spawn_bare_bone_entities(
                &mut commands,
                Transform::IDENTITY,
            );
            queue.apply(app.world_mut());
            built
        };
        app.update();

        let character = app.world_mut().spawn_empty().id();
        let mut queue = bevy::ecs::world::CommandQueue::default();
        let mut state = app.world_mut().query::<&GlobalTransform>();
        let world = app.world();
        let globals = state.query(world);
        let mut commands = Commands::new(&mut queue, world);
        let ragdoll = spawn_ragdoll(
            &mut commands,
            character,
            &skeleton,
            &globals,
            &RagdollSpawnConfig { pin_root: false, ..Default::default() },
        );
        queue.apply(app.world_mut());

        let hips = ragdoll.bodies[Bone::Hips].expect("the root must be simulated");
        app.world_mut().entity_mut(character).insert((ragdoll, skeleton));
        // Limp, so only gravity acts.
        app.world_mut().entity_mut(character).get_mut::<Ragdoll>().unwrap().set_strength(0.0);

        let start = app.world().get::<Transform>(hips).unwrap().translation;
        step(&mut app, 120);
        let end = app.world().get::<Transform>(hips).unwrap().translation;

        assert!(
            start.y - end.y > 0.5,
            "an unpinned ragdoll should fall freely, but its root only moved {:.3} m",
            start.y - end.y,
        );
    }

    /// A full ragdoll with the kinematic pose state attached, so the
    /// read-back has something to blend into and write out.
    ///
    /// The `AnimPose` is deliberately the REST pose, matching the bind-pose
    /// rig `spawn_bare_bone_entities` builds and the orientations the
    /// bodies were therefore spawned at. Seeding it with `relaxed_stand`
    /// instead describes a character whose animation and physics disagree
    /// from frame zero — the arms alone differ by 71 degrees — which makes
    /// "did read-back change the pose?" unanswerable.
    fn spawn_readback_app(strength: f32) -> (App, Entity, Ragdoll) {
        let (mut app, character, ragdoll) = spawn_full_ragdoll_app();

        app.world_mut()
            .entity_mut(character)
            .insert(AnimPose::settled_on(&crate::character::anim::poses::rest()));
        app.world_mut()
            .entity_mut(character)
            .get_mut::<Ragdoll>()
            .unwrap()
            .set_strength(strength);

        (app, character, ragdoll)
    }

    #[test]
    fn a_limp_ragdolls_fall_reaches_the_rendered_skeleton() {
        // The headline property of read-back, and the one that was missing
        // entirely: physics has to be VISIBLE. Before `read_back_simulated_pose`
        // existed the simulation ran correctly and the character rendered
        // its kinematic pose regardless — an expensive no-op.
        //
        // A limp ragdoll under gravity is the clearest case: it physically
        // collapses, so the rendered skeleton must change. If the bones do
        // not move, nothing is being read back.
        let (mut app, character, _ragdoll) = spawn_readback_app(0.0);
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));

        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap().clone();
        let before: Vec<Quat> = Bone::ALL
            .iter()
            .map(|&bone| app.world().get::<Transform>(skeleton.entity(bone)).unwrap().rotation)
            .collect();

        step(&mut app, 120);

        let moved = Bone::ALL
            .iter()
            .enumerate()
            .filter(|(index, bone)| {
                let now =
                    app.world().get::<Transform>(skeleton.entity(**bone)).unwrap().rotation;
                now.angle_between(before[*index]).to_degrees() > 5.0
            })
            .count();

        assert!(
            moved >= 5,
            "a limp ragdoll collapsing under gravity must move the rendered skeleton, but \
             only {moved} bones changed by more than 5 degrees — the simulation is not \
             reaching the pose",
        );
    }

    #[test]
    fn a_fully_driven_ragdoll_leaves_the_animated_pose_alone() {
        // The other end of the dial, and the one that keeps read-back from
        // being destructive: at full strength the controller is already
        // tracking the animation, so showing the simulation must be close
        // to a no-op. A read-back that overwrote the animated pose here
        // would make every ragdoll character visibly worse than a purely
        // kinematic one.
        let (mut app, character, _ragdoll) = spawn_readback_app(1.0);
        app.insert_resource(Gravity(Vec3::ZERO));

        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap().clone();
        let before: Vec<Quat> = Bone::ALL
            .iter()
            .map(|&bone| app.world().get::<Transform>(skeleton.entity(bone)).unwrap().rotation)
            .collect();

        step(&mut app, 120);

        let mut worst = (0.0f32, "none");
        for (index, &bone) in Bone::ALL.iter().enumerate() {
            let now = app.world().get::<Transform>(skeleton.entity(bone)).unwrap().rotation;
            let drift = now.angle_between(before[index]).to_degrees();
            if drift > worst.0 {
                worst = (drift, bone.name());
            }
        }

        assert!(
            worst.0 < 1.0,
            "a fully driven ragdoll should render essentially the animated pose, but {} \
             drifted {:.2} degrees",
            worst.1,
            worst.0,
        );
    }

    #[test]
    fn read_back_preserves_every_bone_length() {
        // The structural invariant the whole rotation-space design rests
        // on, checked on the one path that could break it: read-back
        // computes local rotations from simulated WORLD rotations, and an
        // error there would write a rotation that does not correspond to
        // any real pose. Lengths cannot stretch under pure rotation, so
        // this fails loudly if the inversion is wrong.
        let (mut app, character, _ragdoll) = spawn_readback_app(0.0);
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        step(&mut app, 120);

        let pose = app
            .world()
            .get::<Ragdoll>(character)
            .unwrap()
            .displayed
            .expect("read-back should have produced a displayed pose");
        let positions = crate::character::anim::rig::forward_kinematics(&pose);

        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest = bone.t_pose_offset().length();
            let posed = (positions[bone] - positions[parent]).length();

            assert!(
                (posed - rest).abs() < 1.0e-4,
                "read-back stretched {} to {posed} m against a rest length of {rest} m",
                bone.name(),
            );
        }
    }

    /// A pose on the REAL rig, with deltas about axes each bone's bind
    /// rotation genuinely moves. On the synthetic rig every bind is
    /// identity and any composition order gives the same answer, and on the
    /// real legs a delta about X commutes with an X-bound bind — so a pose
    /// that only bends knees would pass whatever the convention.
    fn real_rig_with_off_axis_pose() -> (World, HumanoidSkeleton, RigGeometry, LocalPose) {
        use crate::character::anim::gltf_rig;

        let mut world = World::new();
        let entities =
            Bone::ALL.iter().map(|&bone| (bone, world.spawn(Transform::IDENTITY).id())).collect();
        let skeleton = gltf_rig::real_skeleton(&gltf_rig::parsed_rig(), entities);
        let rig = RigGeometry::from_skeleton(&skeleton, BoneSet::from_fn(|b| b.t_pose_offset()));

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Spine1, Quat::from_axis_angle(Vec3::Y, 0.25));
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.4));
        pose.set_rotation(Bone::LeftForeArm, Quat::from_axis_angle(Vec3::X, -0.5));
        pose.set_rotation(Bone::RightUpLeg, Quat::from_axis_angle(Vec3::Z, 0.2));
        pose.set_rotation(Bone::LeftLeg, Quat::from_axis_angle(Vec3::Y, 0.3));

        (world, skeleton, rig, pose)
    }

    /// Every bone's world rotation as the RENDERER draws it: accumulated
    /// from what `write_pose_to_skeleton` actually wrote, not from a model
    /// of it — a hand-composed chain only restates its author's convention.
    fn rendered_world_rotations(
        world: &mut World,
        skeleton: &HumanoidSkeleton,
        pose: &LocalPose,
    ) -> BoneSet<Quat> {
        {
            let mut state = world.query::<&mut Transform>();
            let mut query = state.query_mut(world);
            crate::character::anim::retarget::write_pose_to_skeleton(skeleton, pose, &mut query);
        }

        let mut rendered = BoneSet::splat(Quat::IDENTITY);
        for &bone in Bone::ALL.iter() {
            let parent = match bone.parent() {
                Some(parent) => rendered[parent],
                None => skeleton.hips_root_rotation(),
            };
            rendered[bone] =
                parent * world.get::<Transform>(skeleton.entity(bone)).unwrap().rotation;
        }
        rendered
    }

    #[test]
    fn published_targets_are_the_rotations_the_renderer_draws_on_a_real_rig() {
        // The live gallery's ragdoll measured 35-178 degrees off its
        // targets on every body, with gravity and without — the legs at
        // exactly 121.2 degrees both ways, a frame error rather than a
        // sag. The ragdoll kept a private copy of the rotation accumulation
        // from before `rig`'s pose-space fix, and every headless test here
        // runs a synthetic rig whose identity binds make the two agree.
        let (mut world, skeleton, rig, pose) = real_rig_with_off_axis_pose();
        let rendered = rendered_world_rotations(&mut world, &skeleton, &pose);
        let targets = joint_targets(&pose, &rig);

        let mut worst = (0.0f32, "none");
        for &bone in Bone::ALL.iter() {
            let error = targets[bone].angle_between(rendered[bone]).to_degrees();
            if error > worst.0 {
                worst = (error, bone.name());
            }
        }

        assert!(
            worst.0 < 0.01,
            "the ragdoll would drive {} {:.2} degrees away from where the renderer draws it",
            worst.1,
            worst.0,
        );
    }

    /// The nearest ancestor of `bone` that the layout gives a body.
    fn body_ancestor(bone: Bone, layout: &BoneSet<Option<BodyEnd>>) -> Option<Bone> {
        let mut current = bone.parent()?;
        while layout[current].is_none() {
            current = current.parent()?;
        }
        Some(current)
    }

    /// A joint's swing and twist under `pose`, in degrees, measured in the
    /// frames [`joint_bases`] gives the physical joint — between `bone`'s
    /// body and its nearest body ancestor, through any bones between.
    ///
    /// Built from world rotations rather than from `bone`'s own delta, so a
    /// joint that spans a skipped bone (the arm hangs from the upper torso
    /// through the collarbone) counts the skipped bone's motion too. Splits
    /// about `+Y` into swing and twist exactly as avian's constraint reads
    /// it.
    fn swing_and_twist(
        pose: &LocalPose,
        rig: &RigGeometry,
        layout: &BoneSet<Option<BodyEnd>>,
        bone: Bone,
    ) -> Option<(f32, f32)> {
        use crate::character::anim::rig::{accumulate_world_rotations, forward_kinematics_on};

        let parent = body_ancestor(bone, layout)?;

        // The capsule's direction in the bone's own frame, from the rest
        // pose — a bone is rigid, so any pose would give the same answer.
        let segment = match layout[bone]? {
            BodyEnd::Joint(end) => {
                let rest_positions = forward_kinematics_on(&LocalPose::REST, rig);
                let rest_world = accumulate_world_rotations(&LocalPose::REST, rig);
                rest_world[bone].inverse() * (rest_positions[end] - rest_positions[bone])
            }
            BodyEnd::Along { .. } => {
                let bind = accumulate_world_rotations(&LocalPose::REST, rig);
                bind_up_in_bone_frame(bind[bone])
            }
        };
        let (on_parent, on_child) = joint_bases(
            segment,
            bind_rotation_between(parent, bone, |b| rig.bind_rotations[b]),
        );

        let world = accumulate_world_rotations(pose, rig);
        let relative = (world[parent] * on_parent).inverse() * (world[bone] * on_child);

        let (swing, twist) = avian_limit_angles(relative);
        Some((swing.to_degrees(), twist.to_degrees()))
    }

    #[test]
    fn every_pose_the_character_holds_sits_inside_its_joint_limits_on_a_real_rig() {
        // Limits are anatomical stops centred on the bind pose. A shipped
        // pose outside one is a pose the physical joint cannot hold: the
        // controller drives into the stop forever, which is the one
        // configuration this module's design notes call unstable. Measured
        // live, before the limit frame was fixed: shoulders and neck driven
        // against their stops, the arms wandering to 130 degrees off.
        //
        // Pure data, no physics: the joint frame makes swing and twist
        // exactly the pose delta, so this reads the pose alone.
        use crate::character::anim::gait::{walk_pose_on, GaitParams};
        use crate::character::anim::gltf_rig;
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        let rig = gltf_rig::puppet_base();
        let limits = crate::character::anim::ragdoll::default_joint_limits();
        let layout = default_body_layout();

        let mut poses: Vec<(String, LocalPose)> = crate::character::anim::poses::all_named_poses()
            .into_iter()
            .map(|(name, pose)| (name.to_string(), pose))
            .collect();
        // The gait, composed exactly as the gallery composes it.
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        poses.push(("stance".into(), stood));
        for (gait, params) in [("walk", GaitParams::default()), ("run", GaitParams::running())] {
            for step in 0..16 {
                let phase = step as f32 / 16.0;
                poses.push((format!("{gait}@{phase:.3}"), walk_pose_on(phase, &params, &stood, &rig)));
            }
        }

        let mut violations = Vec::new();
        for (name, pose) in &poses {
            for &bone in Bone::ALL.iter() {
                let (Some(limit), Some((swing, twist))) =
                    (limits[bone], swing_and_twist(pose, &rig, &layout, bone))
                else {
                    continue;
                };
                // Against what avian will actually enforce.
                let cone = limit.swing_half_angle.to_degrees();
                let (low, high) =
                    (limit.twist_range.0.to_degrees(), limit.twist_range.1.to_degrees());
                if swing > cone || twist < low || twist > high {
                    violations.push(format!(
                        "{name}: {} swing {swing:.1} (cone {cone:.0}), twist {twist:.1} \
                         (range {low:.0}..{high:.0})",
                        bone.name(),
                    ));
                }
            }
        }

        assert!(
            violations.is_empty(),
            "{} pose/joint combinations fall outside their limits:\n{}",
            violations.len(),
            violations.join("\n"),
        );
    }

    #[test]
    fn read_back_recovers_the_pose_the_renderer_drew_on_a_real_rig() {
        // The other half of the seam: given bodies sitting exactly where the
        // renderer drew each bone, the read-back's inversion must recover
        // the pose that produced them. Checked against the renderer's own
        // output rather than against `joint_targets`, so the inverse is not
        // being compared with the function it inverts.
        let (mut world, skeleton, rig, pose) = real_rig_with_off_axis_pose();
        let rendered = rendered_world_rotations(&mut world, &skeleton, &pose);
        let accumulated_bind = crate::character::anim::rig::accumulate_bind_rotations(&rig);

        for &bone in Bone::ALL.iter() {
            let parent_world = match bone.parent() {
                Some(parent) => rendered[parent],
                None => rig.root_rotation,
            };
            let recovered =
                delta_from_world(bone, parent_world, rendered[bone], &rig, &accumulated_bind);
            let error = recovered.angle_between(pose.rotations[bone]).to_degrees();

            assert!(
                error < 0.01,
                "read-back would show {} {error:.2} degrees away from the pose the renderer \
                 drew it in",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_fully_driven_ragdoll_renders_the_ik_corrected_pose() {
        // Turning the ragdoll on used to switch foot and arm IK off on
        // screen: the read-back blended from the raw animation while the
        // bodies were driven toward the corrected one. At full strength the
        // render must be exactly what the kinematic stack alone would draw.
        let (mut app, character, _ragdoll) = spawn_readback_app(1.0);
        app.insert_resource(Gravity(Vec3::ZERO));

        // An "IK result" that differs visibly from the animated rest pose.
        let mut corrected = crate::character::anim::poses::rest();
        let reach = Quat::from_axis_angle(Vec3::Z, 0.6);
        corrected.set_rotation(Bone::LeftForeArm, reach);
        app.world_mut()
            .entity_mut(character)
            .insert(AnimFootIk { corrected: Some(corrected), ..Default::default() });

        step(&mut app, 30);

        let rendered = rendered_rotation(&app, character, Bone::LeftForeArm);
        let error = rendered.angle_between(reach).to_degrees();
        assert!(
            error < 0.5,
            "the render should show the IK-corrected forearm, but is {error:.1} degrees off it",
        );
    }

    #[test]
    fn read_back_never_writes_the_simulation_into_the_animation() {
        // The feedback loop, stated directly. A limp ragdoll collapses, the
        // render shows it — and the ANIMATION must still be the rest pose
        // it was given, or next frame's display blend and PD targets both
        // start from physics instead of from the animation.
        let (mut app, character, _ragdoll) = spawn_readback_app(0.0);
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        step(&mut app, 120);

        let animated = app.world().get::<AnimPose>(character).unwrap().pose();
        let rest = crate::character::anim::poses::rest();
        for &bone in Bone::ALL.iter() {
            let drift = animated.rotations[bone].angle_between(rest.rotations[bone]).to_degrees();
            assert!(
                drift < 1.0e-3,
                "read-back leaked the simulation into {}'s animated rotation ({drift:.2} \
                 degrees) — see `Ragdoll::displayed`",
                bone.name(),
            );
        }
    }

    /// How far a limited joint lets its child bend or twist when driven hard
    /// toward `target`, with avian's raw `swing`/`twist` limits in degrees.
    fn avian_reach(target: Quat, swing: f32, twist: f32) -> f32 {
        avian_reach_about(target, swing, twist, Vec3::Y)
    }

    /// [`avian_reach`] with an explicit joint `twist_axis`.
    fn avian_reach_about(target: Quat, swing: f32, twist: f32, twist_axis: Vec3) -> f32 {
        let mut app = physics_app();
        let character = app
            .world_mut()
            .spawn(Ragdoll {
                params: BoneSet::splat(PdParams {
                    frequency_hz: 6.0,
                    damping_ratio: 1.0,
                    max_torque: 400.0,
                }),
                ..Default::default()
            })
            .id();
        let parent = app
            .world_mut()
            .spawn((RigidBody::Static, Collider::capsule(0.05, 0.3), Transform::default()))
            .id();
        let child = app
            .world_mut()
            .spawn((
                RigidBody::Dynamic,
                Collider::capsule(0.05, 0.3),
                Transform::from_xyz(0.0, -0.25, 0.0),
                JointTarget { bone: Bone::LeftArm, target, character },
            ))
            .id();
        let mut joint = SphericalJoint::new(parent, child)
            .with_local_anchor1(Vec3::new(0.0, -0.1, 0.0))
            .with_local_anchor2(Vec3::new(0.0, 0.15, 0.0))
            .with_swing_limits(-swing.to_radians(), swing.to_radians())
            .with_twist_limits(-twist.to_radians(), twist.to_radians());
        joint.twist_axis = twist_axis;
        app.world_mut().spawn((joint, JointCollisionDisabled));
        step(&mut app, 300);
        rotation_of(&app, child).angle_between(Quat::IDENTITY).to_degrees()
    }

    #[test]
    fn avians_swing_and_twist_limits_are_not_a_cone_and_a_twist() {
        // Pins avian 0.7's actual limit semantics, MEASURED — the names
        // mislead. There is no cone: `swing_limit` bounds the angle between
        // the two bodies' reference axes (`twist_axis.any_orthonormal_vector()`),
        // which moves under a twist AND under a bend about the other
        // perpendicular axis; `twist_limit` bounds a bend about the
        // reference axis alone. Read as a cone and a twist, the live arm's
        // 85-degree bend was stopped by its 70-degree "twist" range at
        // ~68.5 degrees. If this fails, avian has changed:
        // `avian_limit_ranges` and `avian_limit_angles` must follow it.
        let reference = Vec3::Y.any_orthonormal_vector();
        let other = Vec3::Y.cross(reference).normalize();
        let about = |axis: Vec3| Quat::from_axis_angle(axis, 60f32.to_radians());

        // Unlimited, every probe gets where it was sent.
        for axis in [Vec3::Y, reference, other] {
            assert!(avian_reach(about(axis), 170.0, 170.0) > 55.0, "setup: free joint about {axis}");
        }

        // `swing_limit` stops a twist and a bend about the OTHER axis...
        assert!(avian_reach(about(Vec3::Y), 10.0, 170.0) < 20.0, "swing range should stop a twist");
        assert!(avian_reach(about(other), 10.0, 170.0) < 20.0, "swing range should stop that bend");
        // ...but not a bend about the reference axis, which is the twist
        // range's alone.
        assert!(avian_reach(about(reference), 10.0, 170.0) > 55.0, "swing range should not stop it");
        assert!(avian_reach(about(reference), 170.0, 10.0) < 20.0, "twist range should stop it");
        assert!(avian_reach(about(other), 170.0, 10.0) > 55.0, "twist range should not stop that");
    }

    #[test]
    fn every_limited_ragdoll_joint_uses_the_cone_and_twist_configuration() {
        // The workaround only helps if every joint actually carries it. A
        // joint left on avian's default `twist_axis` silently reverts to the
        // two-bend-stops reading — which clamped the live arm at 68.5
        // degrees and gave the knee's bend an 8-degree stop.
        let (mut app, _character, _ragdoll) = spawn_full_ragdoll_app();
        let mut query = app.world_mut().query::<&SphericalJoint>();
        let joints: Vec<SphericalJoint> = query.iter(app.world()).cloned().collect();

        assert!(joints.len() >= 10, "setup: expected a full rig of joints, got {}", joints.len());
        for joint in &joints {
            assert!(joint.swing_limit.is_some(), "every ragdoll joint should be limited");
            assert_eq!(joint.twist_axis, Vec3::X, "a joint kept avian's default twist axis");
        }
    }

    #[test]
    fn a_twist_axis_across_the_bone_makes_avians_limits_a_true_cone_and_twist() {
        // The workaround `connect_bodies` relies on. With `twist_axis = X`,
        // avian's reference axis is `X.any_orthonormal_vector()` = +Y, the
        // bone itself: `swing_limit` then bounds how far the bone tilts in
        // ANY direction (a cone) and `twist_limit` the roll about it.
        assert_eq!(Vec3::X.any_orthonormal_vector(), Vec3::Y, "the premise, from glam");

        let about = |axis: Vec3| Quat::from_axis_angle(axis, 60f32.to_radians());
        let reach = |target, swing, twist| avian_reach_about(target, swing, twist, Vec3::X);

        // Bends in every direction around the bone are the cone's...
        for axis in [Vec3::X, Vec3::Z, Vec3::new(1.0, 0.0, 1.0).normalize()] {
            assert!(reach(about(axis), 10.0, 170.0) < 20.0, "the cone should stop a bend about {axis}");
            assert!(reach(about(axis), 170.0, 10.0) > 55.0, "the twist should not stop a bend about {axis}");
        }
        // ...and the roll about the bone is the twist's.
        assert!(reach(about(Vec3::Y), 10.0, 170.0) > 55.0, "the cone should not stop a roll");
        assert!(reach(about(Vec3::Y), 170.0, 10.0) < 20.0, "the twist should stop a roll");
    }

    #[test]
    fn a_ragdoll_spawned_at_its_targets_stays_perfectly_still() {
        // Nothing should move a driven ragdoll that starts exactly where it
        // is told to be. Two things did: contacts between non-adjacent
        // bodies (the neck held 8.8 degrees off and still turning at
        // 3.6 rad/s, from the very first step) and, before them, bodies too
        // light for the loads on their joints. Strict on purpose — a
        // standoff shows as residual SPIN at a constant error, which a
        // tolerance on the error alone reads as "settled".
        let (mut app, _character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        step(&mut app, 400);

        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let target = app.world().get::<JointTarget>(body).unwrap().target;
            let error = rotation_of(&app, body).angle_between(target).to_degrees();
            let spin = app.world().get::<AngularVelocity>(body).unwrap().0.length();
            assert!(
                error < 0.1 && spin < 0.01,
                "{} should rest at its target, but is {error:.2} degrees off and turning at \
                 {spin:.3} rad/s",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_fully_driven_ragdoll_holds_its_pose_under_gravity() {
        // The PD is acceleration-shaped and cannot see the load a body
        // carries, so the spine could never hold the upper body up: live,
        // the torso folded to 178 degrees the moment gravity was on while
        // every limb tracked to a few degrees without it.
        // `support_own_weight` is what makes this hold.
        let (mut app, _character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
        step(&mut app, 300);

        let mut worst = (0.0f32, "none");
        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let target = app.world().get::<JointTarget>(body).unwrap().target;
            let error = rotation_of(&app, body).angle_between(target).to_degrees();
            if error > worst.0 {
                worst = (error, bone.name());
            }
        }

        assert!(
            worst.0 < 5.0,
            "a fully driven ragdoll should hold its pose under gravity, but {} sagged {:.1} \
             degrees",
            worst.1,
            worst.0,
        );
    }

    #[test]
    fn a_stunned_limb_falls_under_gravity_and_the_rest_does_not() {
        // The other end of `support_own_weight`: weight support follows
        // strength, so a stunned arm drops while the body holding it up
        // stays put.
        //
        // A/B on identical input — the same scene with and without the
        // stun — so what is measured is the stun's effect and nothing else.
        // The fall is slower than a free pendulum, and legitimately so: the
        // half-stunned forearm still drives its WORLD orientation, which
        // props the arm up through the elbow.
        let drops = |stun: f32| {
            let (mut app, character, ragdoll) = spawn_full_ragdoll_app();
            app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));
            step(&mut app, 10);

            let arm = ragdoll.bodies[Bone::LeftArm].unwrap();
            let spine = ragdoll.bodies[Bone::Spine2].unwrap();
            let (arm_start, spine_start) = (rotation_of(&app, arm), rotation_of(&app, spine));

            // No shove at all: only the stun.
            app.world_mut().write_message(
                RagdollHit::new(character, Bone::LeftArm, Vec3::ZERO).with_stun(stun),
            );
            step(&mut app, 15);

            (
                rotation_of(&app, arm).angle_between(arm_start).to_degrees(),
                rotation_of(&app, spine).angle_between(spine_start).to_degrees(),
            )
        };

        let (stunned_arm, stunned_spine) = drops(1.0);
        let (held_arm, _) = drops(0.0);

        assert!(
            stunned_arm > 5.0 && stunned_arm > 5.0 * held_arm.max(0.1),
            "the stun should drop the arm: {stunned_arm:.2} degrees stunned against \
             {held_arm:.2} unstunned",
        );
        assert!(
            stunned_spine < 2.0,
            "the spine carrying it should hold, but moved {stunned_spine:.2} degrees",
        );
    }

    #[test]
    fn a_full_ragdoll_tracks_its_targets_without_gravity() {
        // The regression test for TWO real, live-caught frame bugs, both of
        // which left the controller driving every body toward a pose it
        // could not hold. Measured at 177 degrees of error with gravity
        // disabled and targets provably correct to 0.0 degrees:
        //
        // 1. `spawn_bone_body` oriented each body along its own SEGMENT
        //    rather than in its bone's frame. For a bone whose bind
        //    rotation differs from its parent's those are not the same
        //    thing — 93 degrees apart on the knees.
        // 2. `connect_bodies` anchored each joint using the BONE entities'
        //    frames, but a body sits at its segment's midpoint, half a bone
        //    away. Every constraint pulled toward the wrong point.
        //
        // Gravity is off deliberately: this asserts the geometry is
        // self-consistent, which is a separate question from whether the
        // torque ceilings can hold a limb up. Mixing the two is what made
        // the original bug hard to see.
        let (mut app, _character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::ZERO));
        step(&mut app, 600);

        let mut worst = (0.0f32, "none");
        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };
            let target = app.world().get::<JointTarget>(body).unwrap().target;
            let error = rotation_of(&app, body).angle_between(target).to_degrees();
            if error > worst.0 {
                worst = (error, bone.name());
            }
        }

        // 30 degrees rather than near-zero: `Spine2` and the shoulders are
        // multi-child joints whose body spans only the continuation chain,
        // so a residual offset there is structural, not a defect. What this
        // catches is the 150-180 degree failure of a frame mismatch.
        assert!(
            worst.0 < 30.0,
            "a ragdoll with no gravity should settle onto its targets, but {} is \
             {:.1} degrees away — check the body and anchor frames in spawn_ragdoll",
            worst.1,
            worst.0,
        );
    }

    #[test]
    fn a_ragdoll_body_is_oriented_in_its_bones_own_frame() {
        // The direct unit-level statement of bug 1 above, so a failure
        // points at the cause rather than at a settled-pose symptom.
        //
        // A body's rotation must equal its bone's world rotation, because
        // that is exactly what `publish_joint_targets` will drive it
        // toward. Orienting it along its own segment instead is what put
        // the knees 93 degrees out.
        let (mut app, _character, ragdoll) = spawn_full_ragdoll_app();

        let mut state = app.world_mut().query::<&GlobalTransform>();
        let world = app.world();
        let globals = state.query(world);
        let skeleton = world
            .iter_entities()
            .find_map(|entity| entity.get::<HumanoidSkeleton>())
            .expect("the character carries a skeleton");

        for &bone in Bone::ALL.iter() {
            let Some(body) = ragdoll.bodies[bone] else { continue };

            let body_rotation = world.get::<Transform>(body).unwrap().rotation;
            let bone_rotation = globals.get(skeleton.entity(bone)).unwrap().rotation();

            let error = body_rotation.angle_between(bone_rotation).to_degrees();
            assert!(
                error < 0.1,
                "{}'s simulated body is {error:.1} degrees off its bone's own frame — a \
                 body's rotation must BE its bone's rotation, since that is what the PD \
                 controller drives it toward",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_full_driven_ragdoll_stays_stable() {
        // The same rig, this time fully driven. Every joint's controller
        // is pulling toward its spawn orientation while every constraint
        // holds — the configuration the whole `kd * dt` investigation was
        // about, now at rig scale.
        let (mut app, _character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::NEG_Y * 9.81));

        let bodies: Vec<Entity> = Bone::ALL
            .iter()
            .filter_map(|&bone| ragdoll.bodies[bone])
            .collect();

        // Past the settling transient.
        step(&mut app, (1.0 / TIMESTEP) as usize);

        let peak_over = |app: &mut App, seconds: f32| -> f32 {
            let mut previous: Vec<Quat> =
                bodies.iter().map(|&b| rotation_of(app, b)).collect();
            let mut worst = 0.0f32;

            for _ in 0..(seconds / TIMESTEP) as usize {
                app.update();
                for (index, &body) in bodies.iter().enumerate() {
                    let rotation = rotation_of(app, body);
                    assert!(rotation.is_finite(), "a driven ragdoll body went non-finite");
                    worst = worst.max(previous[index].angle_between(rotation) / TIMESTEP);
                    previous[index] = rotation;
                }
            }
            worst
        };

        let early = peak_over(&mut app, 5.0);
        let late = peak_over(&mut app, 5.0);

        // The guard is on GROWTH, per the single-joint tests: a driven
        // ragdoll under gravity is never perfectly still, but its motion
        // must not compound.
        assert!(
            late < early * 1.5 + 0.5,
            "a driven ragdoll is gaining energy: peaked at {early:.2} rad/s over the \
             first five seconds and {late:.2} over the next five",
        );
    }

    #[test]
    fn the_head_body_is_vertical_at_rest_and_turns_with_the_head() {
        // The head body runs up from the skull base along bind-pose
        // vertical, carried by the head. So at rest it is exactly vertical
        // on a real rig, whatever that rig's bone axes are — which neither
        // earlier guess managed — and in any pose it tilts exactly as far as
        // the head itself turned.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::{accumulate_world_rotations, forward_kinematics_on};

        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        assert!(rest[Bone::Head].y > rest[Bone::Hips].y + 0.3, "setup: the rig frame is Y-up");

        let bind = accumulate_world_rotations(&LocalPose::REST, &rig);
        let local_up = bind_up_in_bone_frame(bind[Bone::Head]);
        let at_rest = (bind[Bone::Head] * local_up).angle_between(Vec3::Y).to_degrees();
        let own_y = (bind[Bone::Head] * Vec3::Y).angle_between(Vec3::Y).to_degrees();
        assert!(at_rest < 0.01, "at rest the head body should be vertical, tilts {at_rest:.2}");
        eprintln!("puppet_base head: own +Y is {own_y:.1} degrees off vertical at rest");

        // In a pose, it turns with the head and nothing else.
        let pose = crate::character::anim::poses::relaxed_stand();
        let world = accumulate_world_rotations(&pose, &rig);
        let tilt = (world[Bone::Head] * local_up).angle_between(Vec3::Y);
        let head_turn = (world[Bone::Head] * bind[Bone::Head].inverse()).angle_between(Quat::IDENTITY);
        assert!(
            tilt <= head_turn + 1.0e-3,
            "the head body tilts {:.1} degrees but the head only turned {:.1}",
            tilt.to_degrees(),
            head_turn.to_degrees(),
        );
    }

    #[test]
    fn the_default_layout_is_fourteen_chunky_bodies() {
        let layout = default_body_layout();
        let owners: Vec<Bone> =
            Bone::ALL.iter().copied().filter(|&bone| layout[bone].is_some()).collect();

        assert_eq!(owners.len(), 14, "pelvis, two torso, head, 4 arm, 6 leg: {owners:?}");

        // The short bones whose bodies were unstable on the real rig.
        for bone in [Bone::Spine1, Bone::LeftShoulder, Bone::RightShoulder, Bone::Neck] {
            assert!(layout[bone].is_none(), "{} should ride on its parent's body", bone.name());
        }

        // Every body runs DOWN its own chain: the end is a strict
        // descendant, or the capsule would point back through the body.
        for &bone in &owners {
            let Some(BodyEnd::Joint(end)) = layout[bone] else { continue };
            let mut walker = end.parent();
            while walker.is_some_and(|b| b != bone) {
                walker = walker.and_then(Bone::parent);
            }
            assert_eq!(
                walker,
                Some(bone),
                "{}'s body ends at {}, which is not below it",
                bone.name(),
                end.name(),
            );
        }
    }

    #[test]
    fn every_bone_except_the_root_has_a_limit() {
        // Guards the table itself: a bone silently missing a limit is a
        // joint that can rotate through the body, and nothing else here
        // would catch it.
        let limits = crate::character::anim::ragdoll::default_joint_limits();

        for &bone in Bone::ALL.iter() {
            if bone == Bone::Hips {
                assert!(
                    limits[bone].is_none(),
                    "Hips is the root and has no parent joint to limit",
                );
                continue;
            }

            let limit = limits[bone]
                .unwrap_or_else(|| panic!("{} has no joint limit", bone.name()));

            assert!(
                limit.swing_half_angle > 0.0 && limit.swing_half_angle <= std::f32::consts::PI,
                "{}'s swing half-angle {} is not a usable cone",
                bone.name(),
                limit.swing_half_angle,
            );
            assert!(
                limit.twist_range.0 <= limit.twist_range.1,
                "{}'s twist range {:?} is inverted",
                bone.name(),
                limit.twist_range,
            );
        }
    }

    #[test]
    fn partial_strength_gives_partial_resistance() {
        let mut app = physics_app();

        let (character, body) = spawn_driven_body(
            &mut app,
            Quat::IDENTITY,
            PdParams { frequency_hz: 6.0, damping_ratio: 1.0, max_torque: 100.0 },
        );

        app.world_mut()
            .get_mut::<Ragdoll>(character)
            .unwrap()
            .strength = super::super::rig::BoneSet::splat(RagdollStrength(0.02));

        step(&mut app, 5);
        app.world_mut().get_mut::<AngularVelocity>(body).unwrap().0 = Vec3::new(0.0, 6.0, 0.0);
        step(&mut app, 20);

        let displaced = rotation_of(&app, body).angle_between(Quat::IDENTITY);
        assert!(
            displaced > 0.05,
            "a mostly-weakened joint should still give ground, got {displaced} rad",
        );
    }

    /// The rendered local rotation of one bone.
    fn rendered_rotation(app: &App, character: Entity, bone: Bone) -> Quat {
        let skeleton = app.world().get::<HumanoidSkeleton>(character).unwrap();
        app.world().get::<Transform>(skeleton.entity(bone)).unwrap().rotation
    }

    #[test]
    fn a_hit_gives_the_struck_body_the_requested_velocity() {
        // Pins the unit. The bodies weigh grams (default density, 1 kg/m³),
        // so a hit read as an IMPULSE would launch this one at hundreds of
        // times the requested speed.
        let mut app = physics_app();
        let (character, body) = spawn_driven_body(&mut app, Quat::IDENTITY, PdParams::default());
        app.world_mut().get_mut::<Ragdoll>(character).unwrap().bodies[Bone::LeftArm] = Some(body);
        step(&mut app, 2);

        let requested = Vec3::new(1.5, 0.0, -0.5);
        app.world_mut().write_message(RagdollHit::new(character, Bone::LeftArm, requested));
        app.update();

        let velocity = app.world().get::<LinearVelocity>(body).unwrap().0;
        assert!(
            velocity.distance(requested) < 1.0e-3,
            "the struck body should move at {requested:?}, got {velocity:?}",
        );
    }

    #[test]
    fn a_hit_shows_on_the_rendered_skeleton_and_then_recovers() {
        // The whole feature end to end, measured where the player sees it:
        // the rendered bone, not the physics body.
        let (mut app, character, _ragdoll) = spawn_readback_app(1.0);
        app.insert_resource(Gravity(Vec3::ZERO));
        step(&mut app, 30);

        let struck = Bone::LeftForeArm;
        let rest = rendered_rotation(&app, character, struck);
        let far_rest = rendered_rotation(&app, character, Bone::RightLeg);

        // Across the forearm, which lies along the arm in the bind pose.
        app.world_mut().write_message(RagdollHit::new(character, struck, Vec3::new(0.0, 0.0, 3.0)));

        let response = app.world().get::<Ragdoll>(character).unwrap().stun_response;
        let mut peak = 0.0f32;
        let mut far_peak = 0.0f32;
        for _ in 0..((response.hold_secs + 0.2) / TIMESTEP) as usize {
            app.update();
            peak = peak.max(rendered_rotation(&app, character, struck).angle_between(rest));
            far_peak = far_peak
                .max(rendered_rotation(&app, character, Bone::RightLeg).angle_between(far_rest));
        }

        assert!(
            peak.to_degrees() > 20.0,
            "the struck forearm should visibly give way, but only moved {:.1} degrees",
            peak.to_degrees(),
        );
        assert!(
            far_peak.to_degrees() < 1.0,
            "a blow to the left forearm should not move the right knee, which moved {:.2} \
             degrees",
            far_peak.to_degrees(),
        );

        // Full recovery, plus time for the controller to settle.
        step(&mut app, ((response.recover_secs + 2.0) / TIMESTEP) as usize);

        let residual = rendered_rotation(&app, character, struck).angle_between(rest);
        let stun = app.world().get::<Ragdoll>(character).unwrap().stun[struck];
        assert!(
            residual.to_degrees() < 2.0,
            "the forearm should have pulled itself back to the animation, but is still \
             {:.1} degrees off (peaked at {:.1}); its stun is {stun:?}",
            residual.to_degrees(),
            peak.to_degrees(),
        );
        assert!(
            !app.world().get::<Ragdoll>(character).unwrap().is_stunned(),
            "and the stun should have worn off",
        );
    }

    #[test]
    fn without_a_stun_a_hit_moves_the_body_but_not_the_rendered_pose() {
        // Why `RagdollHit::stun` exists. At full strength the read-back
        // shows the animation, so a blow that does not weaken the joint is
        // physically real and visually absent. If this ever starts failing
        // because the render DOES move, the display blend has changed and
        // `stun`'s documentation needs revisiting.
        let (mut app, character, ragdoll) = spawn_readback_app(1.0);
        app.insert_resource(Gravity(Vec3::ZERO));
        step(&mut app, 30);

        let struck = Bone::LeftForeArm;
        let body = ragdoll.bodies[struck].unwrap();
        let rest_render = rendered_rotation(&app, character, struck);
        let rest_body = rotation_of(&app, body);

        app.world_mut().write_message(
            RagdollHit::new(character, struck, Vec3::new(0.0, 0.0, 3.0)).with_stun(0.0),
        );
        step(&mut app, 6);

        let body_moved = rotation_of(&app, body).angle_between(rest_body).to_degrees();
        let render_moved =
            rendered_rotation(&app, character, struck).angle_between(rest_render).to_degrees();

        assert!(body_moved > 2.0, "test setup: the body should be shoved, moved {body_moved:.2}");
        assert!(
            render_moved < 0.5,
            "at full strength and no stun the render should stay animated, moved {render_moved:.2}",
        );
    }

    #[test]
    fn a_hit_to_a_bone_with_no_body_lands_on_its_nearest_one() {
        // A hand is too small for a body; a blow to it is a blow to the
        // forearm.
        let (mut app, character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::ZERO));
        step(&mut app, 5);
        assert!(ragdoll.bodies[Bone::LeftHand].is_none(), "test setup: the hand has no body");

        app.world_mut()
            .write_message(RagdollHit::new(character, Bone::LeftHand, Vec3::new(2.0, 0.0, 0.0)));
        app.update();

        let forearm = ragdoll.bodies[Bone::LeftForeArm].unwrap();
        let speed = app.world().get::<LinearVelocity>(forearm).unwrap().0.length();
        assert!(speed > 0.5, "the forearm should take the blow, but moves at {speed} m/s");

        let stun = app.world().get::<Ragdoll>(character).unwrap().stun[Bone::LeftForeArm];
        assert_eq!(stun.limpness, 1.0, "and be the joint that goes fully slack");
    }

    #[test]
    fn a_hit_never_sets_a_pinned_root_drifting() {
        // The pinned root is KINEMATIC, and avian moves kinematic bodies by
        // their velocity. Shoving it would not be resisted — the whole
        // character would slide away and never stop.
        let (mut app, character, ragdoll) = spawn_full_ragdoll_app();
        app.insert_resource(Gravity(Vec3::ZERO));
        step(&mut app, 5);

        let hips = ragdoll.bodies[Bone::Hips].unwrap();
        let start = app.world().get::<Transform>(hips).unwrap().translation;

        app.world_mut()
            .write_message(RagdollHit::new(character, Bone::Hips, Vec3::new(2.0, 0.0, 0.0)));
        step(&mut app, 120);

        let drift = app.world().get::<Transform>(hips).unwrap().translation.distance(start);
        assert!(drift < 1.0e-4, "a hit set the pinned root drifting {drift:.3} m");
    }
}
