//! Joint drives: a real joint torque between two bodies, the reaction on the
//! parent included, integrated implicitly every physics substep (plan step
//! 4b).
//!
//! # Why not the ragdoll's PD
//!
//! [`super::ragdoll`]'s controller turns each body toward its WORLD target
//! with an angular acceleration on that body alone. Nothing pushes back on
//! the body it hangs from, so it is an invisible hand per body, not a
//! muscle, and it holds the body up only by switching gravity off
//! (`GravityScale = 1 − strength`). Unpinned under real gravity it buckled
//! at 0.5 s, or at 16 Hz held ~1.1 s and then slid its feet apart (see the
//! knowledge note on an unpinned ragdoll).
//!
//! A standing body carries its weight through its joints: a torque `+τ` on
//! the child and `−τ` on the parent, in N·m, sized to the load. That is
//! unstable explicitly integrated at the stiffness standing needs against a
//! light foot. So it is solved as a soft constraint, implicitly, the way
//! avian's own motors and Box2D's soft step do, and inside avian's substep
//! loop: [`drive_impulse`] is stable at any stiffness and step.
//!
//! # The drive
//!
//! For one substep `h`, with `e` the child's rotation error from its target
//! relative to the parent (world scaled-angle-axis), `v` the child's spin
//! relative to the parent, and `K` the two bodies' inverse angular inertias
//! summed (world), the impulse on the child is
//!
//! ```text
//!   P = −(1 + c·K)⁻¹ (h·kp·e + c·v),   c = h·kd + h²·kp
//! ```
//!
//! which is implicit Euler on `kp·e + kd·v` taken at the step's END: the
//! spring sees where the joint will be, so it cannot overshoot by being
//! late. The parent receives `−P`.

use avian3d::dynamics::solver::solver_body::{SolverBody, SolverBodyInertia};
use avian3d::prelude::*;
use bevy::math::{Mat3, Quat, Vec2, Vec3};
use bevy::prelude::*;

use super::math::quat_ext::{neighborhood, to_scaled_angle_axis};
use super::ragdoll::Ragdoll;
use super::ragdoll_plugin::{nearest_simulated_ancestor, JointTarget, KinematicRoot};
use crate::character::skeleton::Bone;

/// A joint drive's stiffness per kilogram of body mass, N·m/rad/kg, for
/// the load-bearing chain (see [`drive_share`]). The drives only correct:
/// the weight is fed forward. Unpinned on `puppet_base`, 10 and 40 stood
/// alike (knees 5-7° off their targets either way, the stiffness the light
/// bodies let through), 10 at the lightest.
pub const DRIVE_STIFFNESS_PER_KG: f32 = 10.0;

/// How far outside the planted soles the capture point may run before a
/// body standing on its own feet falls, metres: past the edge the COP
/// cannot bring it back, and it has no step to take yet.
pub const UNCATCHABLE: f32 = 0.02;

/// How far the feet may stand from their stance's own spacing and still
/// count as together, metres: past it, a recovery step on its own feet
/// is followed by a join.
pub const JOINED: f32 = 0.04;

/// How far past where the capture point will be a recovery step on its
/// own feet lands, metres.
pub const STEP_MARGIN: f32 = 0.05;

/// How far the COM may stand from the nearer foot's sole for the other to
/// join it, metres: that foot holds the body through the join's swing.
pub const JOIN_REACH: f32 = 0.08;

/// A drive's damping per unit stiffness, seconds.
pub const DRIVE_DAMPING_SECONDS: f32 = 0.1;

/// The share of [`DRIVE_STIFFNESS_PER_KG`] each joint gets: the
/// load-bearing chain (legs, spine) all of it, the head and arms less.
pub fn drive_share(bone: Bone) -> f32 {
    match bone {
        Bone::Neck | Bone::Head => 0.3,
        Bone::LeftArm | Bone::RightArm => 0.2,
        Bone::LeftForeArm | Bone::RightForeArm | Bone::LeftHand | Bone::RightHand => 0.1,
        _ => 1.0,
    }
}

/// A joint's strength and speed (plan step 4.2): its muscles' most torque
/// holding still, N·m per kilogram of body mass, about each of the
/// character's axes (`[left, forward, up]`) and each way (`[turning the
/// child positively, negatively]`), and the joint speed at which
/// shortening they give none, rad/s.
///
/// The axes, standing, for a left-side joint (a right one is the mirror:
/// its forward and up columns swap): a positive turn about left swings a
/// hanging segment back (knee flexion, hip, shoulder and elbow extension),
/// tips the trunk and neck forward (flexion) and the foot's toes down
/// (plantarflexion); about forward, swings a hanging limb out (abduction),
/// rolls the foot's sole out (eversion) and the hand back from its palm
/// (wrist extension, the palm facing in); about up, turns a limb's front
/// out (external rotation, forearm supination).
///
/// Maximal voluntary isometric torques of a young man (Harbo et al. 2012,
/// Eur J Appl Physiol 112:267, their regressions at 25 y, 1.80 m, 80 kg)
/// unless noted; not Winter's walking peaks (§7.4.5: knee 0.5 N·m/kg),
/// what a walk uses rather than what a body can give.
/// - Ankle: plantarflexion 1.8 (isokinetic 1.6, isometric 10-30 % higher),
///   dorsiflexion 0.57; eversion 0.42, inversion 0.46 (30 °/s); turning
///   the foot 0.3, estimated.
/// - Knee: flexion 1.5 (isokinetic 1.33), extension 3.5; rotation 0.35
///   either way (Armour 2004, 28 N·m at 60 °/s, 90.5 kg); sideways 0.6,
///   Winter's walking knee abductor moment, mostly passive.
/// - Hip: extension 2.5, flexion 2.2; abduction 1.29, adduction 1.08,
///   external rotation 0.66, internal 0.72 (isometric, 30 young adults).
/// - Trunk: flexion 2.2, extension 3.0 (lumbar, 241 N·m in men); axial
///   rotation 1.25 (~100 N·m in men); lateral bend 1.75, between flexion
///   and rotation as the planes order (extension > flexion > lateral >
///   axial).
/// - Neck and head: flexion 0.40, extension 0.70, lateral bend 0.48,
///   rotation 0.20 (Vasavada et al. 2001: 30, 52, 36, 15 N·m in men).
/// - Shoulder: extension 1.25, flexion 1.0 (estimated from extension ≈
///   1.25 × flexion); abduction 0.84, adduction 1.07; rotation internal
///   0.6, external 0.4 (internal ≈ 1.5 × external; sizes estimated).
/// - Elbow: extension 0.61, flexion 0.67; supination 0.127, pronation
///   0.076 (8.9 and 5.3 N·m, 99 men and women); sideways 0.3, estimated
///   (a hinge's, mostly passive).
/// - Wrist: extension 0.155, flexion 0.33; ulnar deviation 0.13, radial
///   0.15 (9.5 and 11.0 N·m, 10 men).
///
/// The speeds, rad/s, are where Hill's curve at `k` = 0.25 best fits
/// Anderson, Madigan & Nussbaum's (2007) measured torque-velocity of young
/// men: ankle 21 (plantarflexion), knee 24 (extension), hip 19
/// (extension, near-linear there). The elbow's 16.5 is its unloaded
/// flexion speed (Sci Rep 2017); the trunk's, neck's, shoulder's and
/// wrist's are estimates.
pub fn joint_strengths(bone: Bone) -> ([[f32; 2]; 3], f32) {
    // A hinge's sideways bend is held by its bones and ligaments, not its
    // muscles: the ball joint standing in for a knee or an elbow gets a
    // structural limit there, N·m/kg. Capped at the knee's walking
    // abductor moment (0.6, Winter §7.4.5) as if muscle, a side push left
    // the body wandering ±10 mm, never settling.
    const HINGE: f32 = 5.0;
    // [[+left, −left], [+forward, −forward], [+up, −up]], left side.
    let (table, speed): ([[f32; 2]; 3], f32) = match bone {
        Bone::LeftFoot | Bone::RightFoot | Bone::LeftToeBase | Bone::RightToeBase => ([[1.8, 0.57], [0.42, 0.46], [0.3, 0.3]], 21.0), // ESTIMATE: up
        Bone::LeftLeg | Bone::RightLeg => ([[1.5, 3.5], [HINGE, HINGE], [0.35, 0.35]], 24.0),
        Bone::LeftUpLeg | Bone::RightUpLeg => ([[2.5, 2.2], [1.29, 1.08], [0.66, 0.72]], 19.0),
        Bone::Neck | Bone::Head => ([[0.40, 0.70], [0.48, 0.48], [0.20, 0.20]], 15.0), // ESTIMATE: speed
        Bone::LeftArm | Bone::RightArm | Bone::LeftShoulder | Bone::RightShoulder => ([[1.25, 1.0], [0.84, 1.07], [0.4, 0.6]], 20.0), // ESTIMATE: speed, rotation sizes
        Bone::LeftForeArm | Bone::RightForeArm => ([[0.61, 0.67], [HINGE, HINGE], [0.127, 0.076]], 16.5),
        Bone::LeftHand | Bone::RightHand => ([[0.13, 0.15], [0.155, 0.33], [0.1, 0.1]], 20.0), // ESTIMATE: up, speed
        _ => ([[2.2, 3.0], [1.75, 1.75], [1.25, 1.25]], 15.0), // ESTIMATE: speed
    };
    let right = matches!(
        bone,
        Bone::RightFoot
            | Bone::RightToeBase
            | Bone::RightLeg
            | Bone::RightUpLeg
            | Bone::RightArm
            | Bone::RightShoulder
            | Bone::RightForeArm
            | Bone::RightHand
    );
    let mirrored = if right { [table[0], [table[1][1], table[1][0]], [table[2][1], table[2][0]]] } else { table };
    (mirrored, speed)
}

/// A body driven toward its [`JointTarget`] relative to its parent's, by a
/// joint torque between the two. On the child body.
#[derive(Component, Debug, Clone, Copy, PartialEq, Reflect)]
#[reflect(Component)]
pub struct JointDrive {
    /// A torque on the child held for the whole step, N·m, world (the
    /// parent gets its negative): the weight this joint carries, fed
    /// forward so the stiffness only corrects.
    pub feedforward: Vec3,
    /// The parent body (the nearest simulated ancestor's).
    pub parent: Entity,
    /// N·m per radian of error.
    pub stiffness: f32,
    /// N·m per rad/s of relative spin.
    pub damping: f32,
    /// Whether the ground holds the child (a foot flat on the floor): the
    /// drive is then sized on the parent's inertia alone.
    pub child_grounded: bool,
    /// For a planted foot: what the ground can hold the ankle with, the
    /// sole's corners (horizontal `x`, `z`), the joint point, and the
    /// weight on the foot, N. The ankle's torque is held to what keeps
    /// its pressure inside the sole.
    pub sole: Option<([Vec2; 4], Vec3, f32)>,
    /// The most torque the joint's muscles give holding still, N·m, about
    /// each of its axes (`frame`: left, forward, up) and each way (turning
    /// the child positively, negatively) ([`joint_strengths`]): the drive
    /// and what it is fed together never exceed it, scaled by
    /// [`force_velocity`] as the joint moves.
    pub budgets: [[f32; 2]; 3],
    /// The character's left, forward and up as the joint was set up
    /// standing, in the parent body's frame: the axes `budgets` are about
    /// (flexion-extension, abduction-adduction, axial rotation).
    pub frame: Mat3,
    /// The joint's speed at which a shortening muscle gives no force,
    /// rad/s ([`force_velocity`]).
    pub max_speed: f32,
    /// The torque it put on the child in the last substep, N·m, world.
    pub applied: Vec3,
    /// The muscles' twitch time, seconds ([`twitch_seconds`]): what they
    /// are commanded (`commanded`) reaches `feedforward` through a
    /// critically damped lag of this time constant. Zero: at once.
    pub twitch: f32,
    /// The torque the joint is commanded, N·m, world: its load and the
    /// balance (`carry_weight`), before the muscles' lag.
    pub commanded: Vec3,
    /// The balance's part of `commanded` as the muscles give it, N·m, and
    /// how fast it is changing, N·m/s: the lag's state. `feedforward` is
    /// the weight carried now plus this.
    pub activated: Vec3,
    /// See `activated`.
    pub activation_rate: Vec3,
    /// Whether the lag has started from a first command (a new drive takes
    /// it at once rather than sagging while it fills).
    pub primed: bool,
    /// The joint's strength now, `0..=1` (`Ragdoll::effective_strength`:
    /// its dial less a hit's stun), scaling the drive and its command: a
    /// struck limb goes slack and its muscles take it back.
    pub strength: f32,
    /// The child's rotation relative to the parent to drive toward instead
    /// of the animation's (a stepping leg's swing, `carry_weight`).
    pub relative_override: Option<Quat>,
    /// For a swinging joint: the leg beyond it as a lump about the joint,
    /// kg·m², world. Each substep it is driven along its arc with
    /// `I·(−ω²·e − 2ω·ω_rel)` at [`SWING_FREQUENCY`], critically damped.
    pub swing_inertia: Option<Mat3>,
}

/// A joint's muscles' twitch time, seconds: the contraction time of
/// Winter §9.0.5 (Buchthal & Schmalbruch 1970), the time constant of the
/// critically damped response from command to force. Legs: soleus 74 ms,
/// medial gastrocnemius 79. Arms: biceps 52, triceps 44.5. Winter gives no
/// trunk or neck muscle; 60 ms between the two is a choice.
pub fn twitch_seconds(bone: Bone) -> f32 {
    match bone {
        Bone::LeftUpLeg | Bone::RightUpLeg | Bone::LeftLeg | Bone::RightLeg | Bone::LeftFoot | Bone::RightFoot => 0.075,
        Bone::LeftArm | Bone::RightArm | Bone::LeftForeArm | Bone::RightForeArm | Bone::LeftHand | Bone::RightHand => 0.05,
        _ => 0.06,
    }
}

/// Advances a critically damped lag of time constant `seconds` from
/// `(value, rate)` toward `target` by `dt`, solved exactly (stable at any
/// step): the impulse response is Winter's twitch, `F0·(t/T)·e^{−t/T}`.
pub fn activate(value: Vec3, rate: Vec3, target: Vec3, seconds: f32, dt: f32) -> (Vec3, Vec3) {
    if seconds <= 0.0 || dt <= 0.0 {
        return (target, Vec3::ZERO);
    }
    let w = 1.0 / seconds;
    let (error, fade) = (value - target, (-w * dt).exp());
    let carry = rate + error * w;
    (target + (error + carry * dt) * fade, (rate - carry * (w * dt)) * fade)
}

/// How much of its isometric strength a muscle gives at `shortening`, its
/// joint's speed in the direction it pulls over the joint's `max_speed`
/// (negative: lengthening). Shortening, Hill's hyperbola
/// `(1 − v)/(1 + v/k)` with `k` = 0.25 (Winter §9.2.1 gives the form;
/// 0.25 is the wider literature's curvature): zero at `max_speed`.
/// Lengthening, it rises to a plateau of 1.4, Thelen's (2003) young adult
/// within Winter's 1.1-1.8 × Fmax (§9.2.2), leaving zero speed with the
/// hyperbola's slope.
pub fn force_velocity(shortening: f32) -> f32 {
    const CURVATURE: f32 = 0.25;
    const PLATEAU: f32 = 1.4;
    if shortening >= 0.0 {
        ((1.0 - shortening) / (1.0 + shortening / CURVATURE)).max(0.0)
    } else {
        let slope = 1.0 + 1.0 / CURVATURE;
        PLATEAU - (PLATEAU - 1.0) * (-slope / (PLATEAU - 1.0) * -shortening).exp()
    }
}

/// The angular impulse on the child for one substep `h` (the parent gets
/// its negative): see the module doc. `inverse_inertia` is both bodies'
/// world inverse angular inertias summed.
pub fn drive_impulse(error: Vec3, relative_velocity: Vec3, inverse_inertia: Mat3, stiffness: f32, damping: f32, h: f32) -> Vec3 {
    let c = h * damping + h * h * stiffness;
    let a = Mat3::IDENTITY + inverse_inertia * c;
    if a.determinant().abs() < 1.0e-12 {
        return Vec3::ZERO;
    }
    -(a.inverse() * (error * (h * stiffness) + relative_velocity * c))
}

/// The child's rotation error from where `parent_target⁻¹ · child_target`
/// would put it under the parent as it is now: world scaled-angle-axis.
pub fn drive_error(child: Quat, parent: Quat, child_target: Quat, parent_target: Quat) -> Vec3 {
    let wanted = (parent * (parent_target.inverse() * child_target)).normalize();
    let wanted = neighborhood(child, wanted);
    to_scaled_angle_axis(child * wanted.inverse())
}

/// Applies every [`JointDrive`] for one substep. Runs in avian's
/// `SubstepSchedule`, before velocities are integrated, on the solver's own
/// body state: `Rotation` is the step's start there, `delta_rotation` what
/// the substeps have added since.
pub(crate) fn apply_joint_drives(
    mut drives: Query<(Entity, &mut JointDrive, &JointTarget)>,
    targets: Query<&JointTarget>,
    mut bodies: Query<(&mut SolverBody, &SolverBodyInertia, &Rotation)>,
    time: Res<Time>,
) {
    let h = time.delta_secs();
    if h <= 0.0 {
        return;
    }
    for (child, mut drive, child_target) in &mut drives {
        let Ok(parent_target) = targets.get(drive.parent) else { continue };
        let Ok([(mut child_body, child_inertia, child_rotation), (mut parent_body, parent_inertia, parent_rotation)]) =
            bodies.get_many_mut([child, drive.parent])
        else {
            continue;
        };
        let child_now = (child_body.delta_rotation.0 * child_rotation.0).normalize();
        let parent_now = (parent_body.delta_rotation.0 * parent_rotation.0).normalize();
        let error = match drive.relative_override {
            Some(relative) => drive_error(child_now, parent_now, relative, Quat::IDENTITY),
            None => drive_error(child_now, parent_now, child_target.target, parent_target.target),
        };
        let relative = child_body.angular_velocity - parent_body.angular_velocity;
        let (child_inverse, parent_inverse) =
            (child_inertia.effective_inv_angular_inertia().to_mat3(), parent_inertia.effective_inv_angular_inertia().to_mat3());
        let sized_on = if drive.child_grounded { parent_inverse } else { child_inverse + parent_inverse };
        let strength = drive.strength.clamp(0.0, 1.0);
        let mut impulse = drive_impulse(error, relative, sized_on, drive.stiffness * strength, drive.damping * strength, h) + drive.feedforward * h;
        // A swinging joint follows its arc, every substep: explicit, but at
        // ω·h ≈ 0.03 stable whatever the leg's true inertia. Once per
        // physics step instead, it chattered and once flung the body away.
        if let Some(lump) = drive.swing_inertia {
            let w = std::f32::consts::TAU * SWING_FREQUENCY;
            impulse += lump * (-error * (w * w) - relative * (2.0 * w)) * (strength * h);
        }
        if drive.child_grounded
            && let Some((sole, _, weight)) = drive.sole
        {
            impulse = within_sole(impulse / h, &sole, weight) * h;
        }
        // No more than the muscles have, about each anatomical axis and
        // each way, at the speed the joint moves: a joint turning the way
        // its torque pulls is shortening.
        let axes = Mat3::from_quat(parent_now) * drive.frame;
        let (torque, spin) = (axes.transpose() * (impulse / h), axes.transpose() * relative);
        let mut held = Vec3::ZERO;
        for axis in 0..3 {
            let pull = torque[axis];
            let most = drive.budgets[axis][if pull >= 0.0 { 0 } else { 1 }];
            let speed = if drive.max_speed > 0.0 { pull.signum() * spin[axis] / drive.max_speed } else { 0.0 };
            let most = most * force_velocity(speed);
            held[axis] = pull.clamp(-most, most);
        }
        impulse = axes * held * h;
        drive.applied = impulse / h;
        if !drive.child_grounded {
            child_body.angular_velocity += child_inverse * impulse;
        }
        parent_body.angular_velocity -= parent_inverse * impulse;
    }
}

/// Gives a character standing on its own feet (`Ragdoll::carries_itself`)
/// its drives and frees its root; takes the drives away, and the planted
/// feet's dominance with them, when it no longer does. Stopped by a fall,
/// the fall takes the body; stopped by
/// `Ragdoll::stop_standing_on_own_feet`, the root is pinned again where
/// the body stands, and eased to the animation (`KinematicRoot::settle`).
/// Advances `Ragdoll::stand_blend`, the screen's switch between the two.
pub(crate) fn manage_joint_drives(
    mut commands: Commands,
    mut characters: Query<&mut Ragdoll>,
    masses: Query<&ComputedMass>,
    drives: Query<&JointDrive>,
    roots: Query<(), With<KinematicRoot>>,
    poses: Query<(&Position, &Rotation)>,
    time: Res<Time>,
) {
    for mut ragdoll in &mut characters {
        let bodies: Vec<(Bone, Entity)> = ragdoll.bodies.iter().filter_map(|(bone, body)| Some((bone, (*body)?))).collect();
        let step = time.delta_secs() / super::ragdoll::SWITCH_SECONDS;
        // Falling, the screen shows the bodies anyway; a rise ends on the
        // animation, so a body standing on its own again blends in afresh.
        ragdoll.stand_blend = match (ragdoll.carries_itself(), ragdoll.fall.is_some()) {
            (true, _) => (ragdoll.stand_blend + step).min(1.0),
            (false, true) => 0.0,
            (false, false) => (ragdoll.stand_blend - step).max(0.0),
        };
        let carried = bodies.iter().any(|&(_, body)| drives.get(body).is_ok());
        if !ragdoll.carries_itself()
            && carried
            && ragdoll.fall.is_none()
            && let Some(hips) = ragdoll.bodies[Bone::Hips]
            && let Ok((position, rotation)) = poses.get(hips)
        {
            commands.entity(hips).insert((
                RigidBody::Kinematic,
                KinematicRoot {
                    offset: ragdoll.body_offsets[Bone::Hips],
                    position: position.0,
                    rotation: rotation.0,
                    velocity: Vec3::ZERO,
                    settle: Some((position.0, rotation.0, 0.0)),
                },
            ));
        }
        if ragdoll.carries_itself() {
            let total: f32 = bodies.iter().filter_map(|(_, body)| masses.get(*body).ok()).map(|mass| mass.value()).sum();
            // The character's left, forward and up, standing: across its hip
            // joints, level.
            let hip = |bone: Bone| {
                let (position, rotation) = poses.get(ragdoll.bodies[bone]?).ok()?;
                Some(position.0 - rotation.0 * ragdoll.body_offsets[bone])
            };
            let Some(across) = hip(Bone::LeftUpLeg).zip(hip(Bone::RightUpLeg)).map(|(l, r)| Vec3::new(l.x - r.x, 0.0, l.z - r.z).normalize_or_zero())
            else {
                continue;
            };
            let (left, up) = (across, Vec3::Y);
            let forward = left.cross(up);
            for &(bone, body) in &bodies {
                if bone == Bone::Hips && roots.get(body).is_ok() {
                    commands.entity(body).remove::<KinematicRoot>().insert(RigidBody::Dynamic);
                }
                let Some(parent) = nearest_simulated_ancestor(bone, &ragdoll).and_then(|parent| ragdoll.bodies[parent]) else { continue };
                if drives.get(body).is_err() {
                    let stiffness = DRIVE_STIFFNESS_PER_KG * total * drive_share(bone);
                    let (strengths, max_speed) = joint_strengths(bone);
                    let Ok((_, turned)) = poses.get(parent) else { continue };
                    let into = turned.0.inverse();
                    commands.entity(body).insert(JointDrive {
                        feedforward: Vec3::ZERO,
                        parent,
                        stiffness,
                        damping: stiffness * DRIVE_DAMPING_SECONDS,
                        child_grounded: false,
                        sole: None,
                        budgets: strengths.map(|ways| ways.map(|per_kg| per_kg * total)),
                        frame: Mat3::from_cols(into * left, into * forward, into * up),
                        max_speed,
                        applied: Vec3::ZERO,
                        twitch: twitch_seconds(bone),
                        commanded: Vec3::ZERO,
                        activated: Vec3::ZERO,
                        activation_rate: Vec3::ZERO,
                        primed: false,
                        strength: 1.0,
                        relative_override: None,
                        swing_inertia: None,
                    });
                }
            }
        } else {
            for &(_, body) in &bodies {
                if drives.get(body).is_ok() {
                    commands.entity(body).remove::<(JointDrive, Dominance)>();
                }
            }
        }
    }
}

/// Whether `body` touches a static body (the ground).
fn touches_ground(body: Entity, contacts: &ContactGraph, kinds: &Query<&RigidBody>) -> bool {
    contacts.contact_pairs_with(body).any(|pair| {
        let other = if pair.collider1 == body { pair.collider2 } else { pair.collider1 };
        pair.is_touching() && kinds.get(other).is_ok_and(|kind| *kind == RigidBody::Static)
    })
}

/// What [`carry_weight`] reads of each body.
type BodyState = (
    &'static Position,
    &'static Rotation,
    &'static ComputedMass,
    &'static ComputedCenterOfMass,
    &'static LinearVelocity,
    Option<&'static Collider>,
    &'static AngularVelocity,
);

/// How fast a swinging leg follows its arc, Hz (critically damped): the
/// tracking torque fed to its hip and knee, `I·(ω²·e − 2ζω·ω_rel)` on the
/// leg beyond each joint as a lump. Sized on the thigh body alone, the
/// drive moved the foot 4 cm of a 20 cm step in its 0.3 s.
pub const SWING_FREQUENCY: f32 = 4.0;

/// Once per physics step, for every character carrying itself: which feet
/// are planted, and the weight each joint carries (`JointDrive::feedforward`).
///
/// # Planted feet
///
/// A foot touching the ground is planted: [`Dominance`] 1, so the joint
/// above moves the leg and never pushes the foot, and its ankle drive is
/// carried by the ground (`child_grounded`). avian solves its joints after
/// its contacts in every substep, each correction split by inverse mass,
/// so a 1 kg foot under the body took nearly all of it: unpinned, both
/// soles sank 52 mm into the floor and tilted 10° in 9 frames. Pushing the
/// ankle's reaction into a planted foot instead spun it out of the floor.
///
/// # The weight each joint carries
///
/// The drives' correcting stiffness is capped by the two light bodies they
/// are sized on, whatever the gain: the body above an ankle cannot turn
/// with its shin alone. So each joint is fed the gravity moment of the side
/// the ground does not hold, about its joint point:
/// - its child's side (an arm, the head, the trunk above a spine joint, a
///   leg in the air) hangs from it;
/// - on a planted leg, the side above: that leg above the joint, and the
///   leg's share of everything on no planted leg, by where the centre of
///   mass stands between the feet.
///
/// With nothing planted, nothing is fed: a body in the air carries no
/// weight.
///
/// # Balance (plan step 4c)
///
/// Winter's pendulum law, as the standing balance poses it
/// (`balance::Balance`), on the measured centre of mass: the pressure goes
/// to `COP = COM + (COM − rest)·k·ω² + v·2ζω·k`, held inside the planted
/// soles. `rest` is where the COM stood over the feet when both were first
/// planted; `k` the COM's height over the ankles over `g`. Each planted
/// ankle then carries its load as pushed up at its own share of that COP
/// (Winter's ankle moment, `W·(COP − ankle)`), and each leg's share of
/// everything else follows where the COP stands between the feet: the
/// hips' load/unload (§11.2.1).
#[allow(clippy::too_many_arguments)]
pub(crate) fn carry_weight(
    mut characters: Query<&mut Ragdoll>,
    mut drives: Query<&mut JointDrive>,
    bodies: Query<BodyState>,
    kinds: Query<&RigidBody>,
    contacts: Res<ContactGraph>,
    mut commands: Commands,
    gravity: Res<Gravity>,
    time: Res<Time>,
    targets: Query<&JointTarget>,
) {
    const FEET: [Bone; 2] = [Bone::LeftFoot, Bone::RightFoot];
    for mut ragdoll in &mut characters {
        if !ragdoll.carries_itself() {
            ragdoll.stand_rest = None;
            ragdoll.feet_home = None;
            ragdoll.stand_height = None;
            ragdoll.own_step = None;
            ragdoll.own_transfer = None;
            continue;
        }
        let all: Vec<Bone> = Bone::ALL.iter().copied().filter(|&bone| ragdoll.bodies[bone].is_some()).collect();
        // Each body's centre of mass and mass, and its bone's joint point.
        let mut state = super::rig::BoneSet::splat((Vec3::ZERO, 0.0f32, Vec3::ZERO));
        let (mut momentum, mut total) = (Vec3::ZERO, 0.0f32);
        for &bone in &all {
            let Ok((position, rotation, mass, centre, velocity, _, _)) = bodies.get(ragdoll.bodies[bone].unwrap()) else { return };
            state[bone] = (position.0 + rotation.0 * centre.0, mass.value(), position.0 - rotation.0 * ragdoll.body_offsets[bone]);
            momentum += velocity.0 * mass.value();
            total += mass.value();
        }
        let below = |top: Bone| -> Vec<Bone> {
            all.iter()
                .copied()
                .filter(|&bone| std::iter::successors(Some(bone), |b| b.parent()).any(|b| b == top))
                .collect()
        };
        let legs = [below(Bone::LeftUpLeg), below(Bone::RightUpLeg)];
        let feet_bodies = FEET.map(|foot| ragdoll.bodies[foot]);
        let touching = |leg: usize| feet_bodies[leg].is_some_and(|foot| touches_ground(foot, &contacts, &kinds));
        // A step under way: the swinging foot is not planted; it lands once
        // its time is up and it is down.
        if let Some(mut step) = ragdoll.own_step {
            step.elapsed += time.delta_secs();
            ragdoll.own_step = (step.elapsed < step.duration || !touching(step.leg)).then_some(step);
        }
        let swinging = ragdoll.own_step.map(|step| step.leg);
        let planted: Vec<usize> = (0..2).filter(|&leg| Some(leg) != swinging && touching(leg)).collect();
        for (leg, foot) in FEET.iter().enumerate() {
            let Some(body) = ragdoll.bodies[*foot] else { continue };
            let down = planted.contains(&leg);
            if let Ok(mut drive) = drives.get_mut(body)
                && drive.child_grounded != down
            {
                drive.child_grounded = down;
                commands.entity(body).insert(Dominance(down as i8));
            }
        }
        let flat = |v: Vec3| Vec2::new(v.x, v.z);
        let com = all.iter().fold(Vec3::ZERO, |sum, &b| sum + state[b].0 * state[b].1) / total;
        let velocity = momentum / total;
        // Each planted sole's footprint, its block's bottom corners, and
        // the height of the ground under it.
        let soles: Vec<(Vec<Vec2>, f32)> = planted
            .iter()
            .map(|&leg| {
                let Some(Ok((position, rotation, _, _, _, Some(collider), _))) = ragdoll.bodies[FEET[leg]].map(|body| bodies.get(body)) else {
                    return (vec![flat(state[FEET[leg]].2)], state[FEET[leg]].2.y);
                };
                // The block's sole face (its −y face, `SoleBox`), seen from
                // above whatever its tilt: a foot down rolls flat as the body
                // comes over it. Taken as only the corners on the ground, a
                // backward step landing toes first stood on its toe edge, and
                // the capture point over its heel read as uncaught.
                let (mut sole, mut lowest) = (Vec::new(), f32::MAX);
                if let Some(compound) = collider.shape_scaled().as_compound() {
                    for (iso, shape) in compound.shapes() {
                        let Some(cuboid) = shape.as_cuboid() else { continue };
                        let half = Vec3::new(cuboid.half_extents.x, cuboid.half_extents.y, cuboid.half_extents.z);
                        let (offset, turn) = (Vec3::new(iso.translation.x, iso.translation.y, iso.translation.z), iso.rotation);
                        for corner in 0..8 {
                            let sign = |bit: u32| if corner & (1 << bit) == 0 { -1.0 } else { 1.0 };
                            let local = Vec3::new(sign(0) * half.x, sign(1) * half.y, sign(2) * half.z);
                            let world = position.0 + rotation.0 * (offset + turn * local);
                            lowest = lowest.min(world.y);
                            if sign(1) < 0.0 {
                                sole.push(flat(world));
                            }
                        }
                    }
                }
                if sole.is_empty() { (vec![flat(state[FEET[leg]].2)], state[FEET[leg]].2.y) } else { (convex_hull(sole), lowest) }
            })
            .collect();
        // Where the pressure goes: Winter's law on the measured COM, held
        // inside the soles.
        let mut cop = flat(com);
        let mut feet_apart = false;
        if !planted.is_empty() {
            let middle = planted.iter().fold(Vec3::ZERO, |sum, &leg| sum + state[FEET[leg]].2) / planted.len() as f32;
            let apart = state[Bone::RightFoot].2 - state[Bone::LeftFoot].2;
            let apart = Vec3::new(apart.x, 0.0, apart.z);
            if planted.len() == 2 && ragdoll.stand_rest.is_none() {
                ragdoll.stand_rest = Some(Vec3::new(com.x - middle.x, 0.0, com.z - middle.z));
                ragdoll.feet_home = Some(apart);
                let hips = (state[Bone::LeftUpLeg].2.y + state[Bone::RightUpLeg].2.y) * 0.5;
                ragdoll.stand_height = Some(hips - middle.y);
            }
            let rest = flat(middle + ragdoll.stand_rest.unwrap_or(Vec3::new(com.x - middle.x, 0.0, com.z - middle.z)));
            // After a recovery step the feet stand apart: once the foot the
            // body is nearer can hold it, the other joins it at the stance's
            // own width, landing beside it, under the body.
            let displaced = ragdoll.feet_home.is_some_and(|home| home.distance(apart) > JOINED);
            feet_apart = displaced;
            if planted.len() == 2 && displaced && ragdoll.own_step.is_none() {
                let sole_centre = |leg: usize| soles[leg].0.iter().copied().sum::<Vec2>() / soles[leg].0.len().max(1) as f32;
                let onto = *ragdoll.own_transfer.get_or_insert(if sole_centre(0).distance(flat(com)) <= sole_centre(1).distance(flat(com)) { 0 } else { 1 });
                // Not moved over it first: pushed onto it, the COM ran on past
                // the foot's outer edge, asked for a side step outward, and
                // each widened the stance until it ran away.
                let (over, inside) = nearest_in_polygon(flat(com), &soles[onto].0);
                if (inside || over.distance(flat(com)) < JOIN_REACH) && flat(velocity).length() < 0.15 {
                    let home = ragdoll.feet_home.unwrap_or(apart);
                    let joining = 1 - onto;
                    let to = if joining == 1 { state[Bone::LeftFoot].2 + home } else { state[Bone::RightFoot].2 - home };
                    let from = state[FEET[joining]].2;
                    let foot = ragdoll.bodies[FEET[joining]].and_then(|body| bodies.get(body).ok()).map_or(Quat::IDENTITY, |(_, rotation, ..)| rotation.0);
                    ragdoll.own_step = Some(super::ragdoll::OwnStep {
                        leg: joining,
                        from,
                        to: Vec3::new(to.x, from.y, to.z),
                        elapsed: 0.0,
                        duration: super::balance::JOIN_SECONDS,
                        foot,
                    });
                    ragdoll.own_transfer = None;
                }
            } else if !displaced {
                ragdoll.own_transfer = None;
            }
            let ankles = planted.iter().map(|&leg| state[FEET[leg]].2.y).sum::<f32>() / planted.len() as f32;
            let k = ((com.y - ankles) / gravity.0.length().max(1.0e-6)).max(0.0);
            let (w, zeta) = (super::balance::RECOVERY_FREQUENCY, super::balance::RECOVERY_DAMPING);
            let wanted = flat(com) + (flat(com) - rest) * (k * w * w) + flat(velocity) * (2.0 * zeta * w * k);
            let hull = convex_hull(soles.iter().flat_map(|(sole, _)| sole.iter().copied()).collect());
            // Beyond what the feet can catch: the capture point (Hof) out of
            // the soles. It steps where the capture point will be, as the
            // standing balance does (`balance::Balance`); with no step that
            // reaches, it falls, and the fall's own machinery takes the
            // body. Mid-step the capture point is outside the one sole
            // standing, as it must be.
            let capture = flat(com) + flat(velocity) * k.sqrt();
            let (edge, caught) = nearest_in_polygon(capture, &hull);
            if !caught && edge.distance(capture) > UNCATCHABLE && ragdoll.own_step.is_none() {
                let step = (ragdoll.steps_on_own_feet && planted.len() == 2)
                    .then(|| plan_own_step(capture, k, &soles, &state))
                    .flatten();
                match step {
                    Some(mut step) => {
                        // Carried flat as it lifted: left at the animation's
                        // angle to a shin swung forward, it landed toes down
                        // and behind, on a corner.
                        let foot = [Bone::LeftFoot, Bone::RightFoot][step.leg];
                        step.foot = ragdoll.bodies[foot].and_then(|body| bodies.get(body).ok()).map_or(Quat::IDENTITY, |(_, rotation, ..)| rotation.0);
                        ragdoll.own_step = Some(step);
                    }
                    None => {
                        ragdoll.fall(super::ragdoll::FALL_TONE, super::ragdoll::FALL_DAMPING);
                        continue;
                    }
                }
            }
            let (held, inside) = nearest_in_polygon(wanted, &hull);
            let centre = hull.iter().copied().sum::<Vec2>() / hull.len().max(1) as f32;
            cop = if inside { held } else { held + (centre - held).normalize_or_zero() * super::balance::SUPPORT_MARGIN };
        }
        // Each planted leg's share of what no planted leg carries, by where
        // the pressure stands between the feet.
        let upper: Vec<Bone> = all.iter().copied().filter(|b| !planted.iter().any(|&leg| legs[leg].contains(b))).collect();
        let shares = |pressure: Vec2| {
            let mut share = [0.0f32; 2];
            match planted[..] {
                [leg] => share[leg] = 1.0,
                [left, right] => {
                    let centre = |i: usize| soles[i].0.iter().copied().sum::<Vec2>() / soles[i].0.len() as f32;
                    let (l, r) = (centre(0), centre(1));
                    let t = ((pressure - l).dot(r - l) / (r - l).length_squared().max(1.0e-6)).clamp(0.0, 1.0);
                    share[left] = 1.0 - t;
                    share[right] = t;
                }
                _ => {}
            }
            share
        };
        // The body's horizontal acceleration under a pressure (the
        // pendulum: g·(COM − COP)/height), carried by every part as
        // d'Alembert's effective gravity `g − a`.
        let effective_under = |pressure: Vec2| {
            if planted.is_empty() {
                return gravity.0;
            }
            let floor = soles.iter().map(|(_, y)| *y).fold(f32::MAX, f32::min);
            let height = (com.y - floor).max(0.1);
            let lean = flat(com) - pressure;
            gravity.0 - Vec3::new(lean.x, 0.0, lean.y) * (gravity.0.length() / height)
        };
        // What joint `bone` carries with the pressure at `pressure`, and,
        // for a planted ankle, its sole as offsets from the joint and the
        // weight on it.
        let carried = |bone: Bone, pressure: Vec2| -> (Vec3, Option<([Vec2; 4], Vec3, f32)>) {
            let (effective, share) = (effective_under(pressure), shares(pressure));
            let joint = state[bone].2;
            let moment = |set: &[Bone]| set.iter().fold(Vec3::ZERO, |sum, &b| sum + (state[b].0 - joint).cross(effective * state[b].1));
            let subtree = below(bone);
            if planted.is_empty() {
                (Vec3::ZERO, None)
            } else if let Some(leg) = planted.iter().copied().find(|&leg| legs[leg].contains(&bone)) {
                // On a planted leg: the ground pushes on this foot at its
                // share of the pressure (the point of its sole nearest the
                // COP), with its load's weight and the force that moves it.
                // The joint carries that push's moment about itself, less
                // the segments below it.
                let index = planted.iter().position(|&p| p == leg).unwrap();
                let (at, _) = nearest_in_polygon(pressure, &soles[index].0);
                let at = Vec3::new(at.x, soles[index].1, at.y);
                let load = legs[leg].iter().map(|&b| state[b].1).sum::<f32>() + share[leg] * upper.iter().map(|&b| state[b].1).sum::<f32>();
                let sole = (bone == FEET[leg])
                    .then(|| <[Vec2; 4]>::try_from(soles[index].0.iter().map(|&c| c - flat(joint)).collect::<Vec<_>>()).ok())
                    .flatten()
                    .map(|corners| (corners, joint, load * -effective.y));
                ((at - joint).cross(effective * load) - moment(&subtree), sole)
            } else {
                (-moment(&subtree), None)
            }
        };
        // A swinging leg is driven along its arc: the thigh and shin turned
        // to put the ankle on it (`leg_toward`), relative to their parents;
        // the foot keeps the animation's angle to the shin.
        let mut swing: Vec<(Bone, Quat)> = Vec::new();
        if let Some(step) = ragdoll.own_step {
            let [thigh, shin, foot] = [[Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot], [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot]][step.leg];
            let (hip, knee, ankle) = (state[thigh].2, state[shin].2, state[foot].2);
            let left = state[Bone::LeftUpLeg].2 - state[Bone::RightUpLeg].2;
            let forward = Vec3::new(left.x, 0.0, left.z).normalize_or_zero().cross(Vec3::Y);
            let (thigh_way, shin_way) = leg_toward(hip, swing_point(&step), hip.distance(knee), knee.distance(ankle), forward);
            let rotation = |bone: Bone| ragdoll.bodies[bone].and_then(|body| bodies.get(body).ok()).map(|(_, rotation, ..)| rotation.0);
            if let (Some(pelvis), Some(thigh_now), Some(shin_now)) = (rotation(Bone::Hips), rotation(thigh), rotation(shin)) {
                let thigh_world = Quat::from_rotation_arc((knee - hip).normalize_or_zero(), thigh_way) * thigh_now;
                let shin_world = Quat::from_rotation_arc((ankle - knee).normalize_or_zero(), shin_way) * shin_now;
                swing.push((thigh, (pelvis.inverse() * thigh_world).normalize()));
                swing.push((shin, (thigh_world.inverse() * shin_world).normalize()));
                swing.push((foot, (shin_world.inverse() * step.foot).normalize()));
            }
        }
        // With the feet off their stance (a step under way, or taken), the
        // animation's legs no longer fit them: each planted leg is solved to
        // where its foot stands, from the hip, against the animation's
        // upright pelvis, its foot kept as it lies. Left on the animation's
        // angles, the closed chain tilted the trunk 7-23° after a step.
        if ragdoll.own_step.is_some() || feet_apart {
            let pelvis_target = ragdoll.bodies[Bone::Hips].and_then(|body| targets.get(body).ok()).map(|target| target.target);
            let left = state[Bone::LeftUpLeg].2 - state[Bone::RightUpLeg].2;
            let forward = Vec3::new(left.x, 0.0, left.z).normalize_or_zero().cross(Vec3::Y);
            let rotation = |bone: Bone| ragdoll.bodies[bone].and_then(|body| bodies.get(body).ok()).map(|(_, rotation, ..)| rotation.0);
            for &leg in &planted {
                let [thigh, shin, foot] = [[Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot], [Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot]][leg];
                let (hip, knee, ankle) = (state[thigh].2, state[shin].2, state[foot].2);
                let (upper, lower) = (hip.distance(knee), knee.distance(ankle));
                // From the hip held at the stance's height over this ankle,
                // as low as the leg needs to reach it: solved from where the
                // hip is, the leg let the body sink (50 mm in 0.6 s).
                let height = ragdoll.stand_height.unwrap_or(hip.y - ankle.y);
                let across = Vec3::new(hip.x - ankle.x, 0.0, hip.z - ankle.z).length();
                let reach = ((0.98 * (upper + lower)).powi(2) - across * across).max(0.0).sqrt();
                let held = Vec3::new(hip.x, ankle.y + height.min(reach), hip.z);
                let (thigh_way, shin_way) = leg_toward(held, ankle, upper, lower, forward);
                if let (Some(pelvis), Some(thigh_now), Some(shin_now), Some(foot_now)) = (pelvis_target, rotation(thigh), rotation(shin), rotation(foot)) {
                    let thigh_world = Quat::from_rotation_arc((knee - hip).normalize_or_zero(), thigh_way) * thigh_now;
                    let shin_world = Quat::from_rotation_arc((ankle - knee).normalize_or_zero(), shin_way) * shin_now;
                    swing.push((thigh, (pelvis.inverse() * thigh_world).normalize()));
                    swing.push((shin, (thigh_world.inverse() * shin_world).normalize()));
                    swing.push((foot, (shin_world.inverse() * foot_now).normalize()));
                }
            }
        }
        let swinging_bones: Vec<Bone> = ragdoll
            .own_step
            .map(|step| [[Bone::LeftUpLeg, Bone::LeftLeg], [Bone::RightUpLeg, Bone::RightLeg]][step.leg].to_vec())
            .unwrap_or_default();
        // A swinging joint's leg beyond it, as a lump about the joint (point
        // masses, each with a 10 cm radius of its own), for its tracking.
        let lump = |bone: Bone| {
            let joint = state[bone].2;
            below(bone).iter().fold(Mat3::ZERO, |sum, &b| {
                let r = state[b].0 - joint;
                sum + (Mat3::IDENTITY * (r.length_squared() + 0.01) - Mat3::from_cols(r * r.x, r * r.y, r * r.z)) * state[b].1
            })
        };
        for &bone in &all {
            let Ok(mut drive) = drives.get_mut(ragdoll.bodies[bone].unwrap()) else { continue };
            drive.relative_override = swing.iter().find(|(b, _)| *b == bone).map(|(_, relative)| *relative);
            // The weight it carries as the body stands now (the pressure
            // under the COM), and with the balance's pressure.
            let (still, _) = carried(bone, flat(com));
            let (balanced, sole) = carried(bone, cop);
            drive.sole = sole;
            // As strong as the joint is now (a hit's stun). Holding the
            // posture is the muscles' standing tone, at once; the balance's
            // correction comes a twitch time late (step 4.4). Lagged too,
            // the weight of a leaning trunk arrived after it had leant
            // further, and a side push left the body swaying ±10-18 mm
            // without end.
            let strength = ragdoll.effective_strength(bone).0;
            drive.strength = strength;
            drive.commanded = balanced * strength;
            let correction = (balanced - still) * strength;
            if drive.primed {
                let (value, rate) = activate(drive.activated, drive.activation_rate, correction, drive.twitch, time.delta_secs());
                drive.activated = value;
                drive.activation_rate = rate;
            } else {
                drive.activated = correction;
                drive.primed = true;
            }
            drive.feedforward = still * strength + drive.activated;
            // A swinging joint is also driven along its arc, each substep
            // (`apply_joint_drives`): a planned motor command, given at once.
            drive.swing_inertia = (drive.relative_override.is_some() && swinging_bones.contains(&bone)).then(|| lump(bone));
        }
    }
}

/// The recovery step a body standing on both feet takes when its capture
/// point (`capture`, horizontal) has left them, or `None` if no step
/// within [`super::balance::MAX_STEP`] reaches it. `soles` are the left
/// and right soles' footprints; `state` each bone's centre of mass, mass
/// and joint point; `k` the pendulum constant, s².
///
/// The standing balance's rule (`balance::Balance`): through the swing
/// the pressure stands on the other foot at its point nearest the capture
/// point, and the capture point runs away from it as `e^{t/√k}`, so the
/// swinging sole lands where it will be. Sideways, the near leg steps out,
/// quickly ([`super::balance::SIDE_STEP_SECONDS`]); there is no crossover
/// on its own feet (the legs would have to pass). Ahead or back, the foot
/// further from the capture point steps, keeping its side.
fn plan_own_step(capture: Vec2, k: f32, soles: &[(Vec<Vec2>, f32)], state: &super::rig::BoneSet<(Vec3, f32, Vec3)>) -> Option<super::ragdoll::OwnStep> {
    let flat = |v: Vec3| Vec2::new(v.x, v.z);
    let centre = |leg: usize| soles[leg].0.iter().copied().sum::<Vec2>() / soles[leg].0.len().max(1) as f32;
    let left = flat(state[Bone::LeftUpLeg].2 - state[Bone::RightUpLeg].2).normalize_or_zero();
    let forward = Vec2::new(-left.y, left.x);
    let middle = (centre(0) + centre(1)) * 0.5;
    let off = capture - middle;
    let sideways = off.dot(left).abs() > off.dot(forward).abs();
    let leg = if sideways {
        if off.dot(left) > 0.0 { 0 } else { 1 }
    } else if centre(0).distance(capture) >= centre(1).distance(capture) {
        0
    } else {
        1
    };
    let (pressure, _) = nearest_in_polygon(capture, &soles[1 - leg].0);
    let seconds = if sideways { super::balance::SIDE_STEP_SECONDS } else { super::balance::STEP_SECONDS };
    // A little past where the capture point will be (Hof's foot placement),
    // so the stance it lands in can brake: landed exactly there, the body
    // was only just caught and ran on.
    let landing = pressure + (capture - pressure) * (seconds / k.max(1.0e-4).sqrt()).exp();
    let landing = landing + (capture - pressure).normalize_or_zero() * STEP_MARGIN;
    let mut travel = landing - centre(leg);
    if !sideways {
        travel -= left * travel.dot(left);
    }
    if travel.length() > super::balance::MAX_STEP {
        return None;
    }
    let from = state[[Bone::LeftFoot, Bone::RightFoot][leg]].2;
    Some(super::ragdoll::OwnStep { leg, from, to: from + Vec3::new(travel.x, 0.0, travel.y), elapsed: 0.0, duration: seconds, foot: Quat::IDENTITY })
}

/// Where a swinging ankle should be `elapsed` into `step`, world: eased
/// from where it lifted to where it lands, lifted
/// [`super::balance::STEP_LIFT`] at mid-swing; once its time is up,
/// pressed a little into the ground to find it.
pub fn swing_point(step: &super::ragdoll::OwnStep) -> Vec3 {
    let t = (step.elapsed / step.duration).clamp(0.0, 1.0);
    if t >= 1.0 {
        return step.to - Vec3::Y * 0.02;
    }
    let eased = t * t * (3.0 - 2.0 * t);
    step.from.lerp(step.to, eased) + Vec3::Y * super::balance::STEP_LIFT * (std::f32::consts::PI * t).sin()
}

/// The world directions a leg's thigh and shin take for its ankle to reach
/// `target` from the hip joint `hip`, the knee bending toward `forward`
/// (Holden's two-bone recipe, `math::ik::solve_two_bone`, with its soft
/// clamp at full reach).
pub fn leg_toward(hip: Vec3, target: Vec3, upper: f32, lower: f32, forward: Vec3) -> (Vec3, Vec3) {
    let line = target - hip;
    let along = line.normalize_or_zero();
    let solution = super::math::ik::solve_two_bone(upper, lower, line.length(), 0.005);
    let pole = (forward - along * forward.dot(along)).normalize_or_zero();
    let thigh = (along * solution.upper_angle.cos() + pole * solution.upper_angle.sin()).normalize_or_zero();
    let knee = hip + thigh * upper;
    let shin = (hip + along * solution.reach - knee).normalize_or_zero();
    (thigh, shin)
}

/// `torque` on a planted foot (N·m, world) held to what the ground can
/// give it: the pressure it implies, `weight` (N) pushing up at a
/// horizontal offset `d` from the ankle (`τ = d × (0, −W, 0)`), kept inside
/// `sole` (the sole's corners as offsets from the ankle, counter-clockwise).
/// The twist about the vertical is left alone.
pub fn within_sole(torque: Vec3, sole: &[Vec2; 4], weight: f32) -> Vec3 {
    if weight <= 0.0 {
        return Vec3::new(0.0, torque.y, 0.0);
    }
    // τx = dz·W, τz = −dx·W.
    let offset = Vec2::new(-torque.z / weight, torque.x / weight);
    let (held, _) = nearest_in_polygon(offset, sole);
    Vec3::new(held.y * weight, torque.y, -held.x * weight)
}

/// The convex hull of `points` (horizontal, `x` and `z` as a `Vec2`),
/// counter-clockwise, by Andrew's monotone chain.
pub fn convex_hull(mut points: Vec<Vec2>) -> Vec<Vec2> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup_by(|a, b| a.distance_squared(*b) < 1.0e-12);
    if points.len() < 3 {
        return points;
    }
    let cross = |o: Vec2, a: Vec2, b: Vec2| (a - o).perp_dot(b - o);
    let mut hull: Vec<Vec2> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Box<dyn Iterator<Item = &Vec2>> = if pass == 0 { Box::new(points.iter()) } else { Box::new(points.iter().rev()) };
        for &p in ordered {
            while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    hull
}

/// The point of the convex polygon `hull` (counter-clockwise) nearest `p`,
/// and whether `p` was inside it.
pub fn nearest_in_polygon(p: Vec2, hull: &[Vec2]) -> (Vec2, bool) {
    if hull.len() < 3 {
        return (hull.first().copied().unwrap_or(p), false);
    }
    let edges = || hull.iter().zip(hull.iter().cycle().skip(1));
    if edges().all(|(&a, &b)| (b - a).perp_dot(p - a) >= 0.0) {
        return (p, true);
    }
    let nearest = edges()
        .map(|(&a, &b)| {
            let t = ((p - a).dot(b - a) / (b - a).length_squared().max(1.0e-12)).clamp(0.0, 1.0);
            a + (b - a) * t
        })
        .min_by(|x, y| x.distance_squared(p).total_cmp(&y.distance_squared(p)))
        .unwrap_or(p);
    (nearest, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_leg_solved_toward_a_point_reaches_it_with_the_knee_forward() {
        let (upper, lower) = (0.45, 0.42);
        let hip = Vec3::new(0.1, 1.0, 0.0);
        let forward = Vec3::NEG_Z;
        for target in [Vec3::new(0.1, 0.2, 0.0), Vec3::new(0.25, 0.25, -0.3), Vec3::new(-0.05, 0.3, 0.25)] {
            let (thigh, shin) = leg_toward(hip, target, upper, lower, forward);
            let ankle = hip + thigh * upper + shin * lower;
            assert!(ankle.distance(target) < 1.0e-3, "{target}: reached {ankle}");
            let knee = hip + thigh * upper;
            let line = (target - hip).normalize();
            let off = knee - (hip + line * line.dot(knee - hip));
            assert!(off.dot(forward) > 0.0, "{target}: the knee bent back, {off}");
        }
    }

    #[test]
    fn a_muscle_gives_less_shortening_and_more_lengthening() {
        // Hill's hyperbola shortening (k = 0.25), Thelen's 1.4 plateau
        // lengthening, joined smoothly at rest.
        assert_eq!(force_velocity(0.0), 1.0);
        assert!(force_velocity(1.0).abs() < 1.0e-6 && force_velocity(1.5) == 0.0, "none at and past the top speed");
        assert!((force_velocity(0.5) - 0.5 / 3.0).abs() < 1.0e-5, "(1 − v)/(1 + 4v) at v = 0.5: {}", force_velocity(0.5));
        assert!((force_velocity(-10.0) - 1.4).abs() < 1.0e-3, "the eccentric plateau: {}", force_velocity(-10.0));
        let slope = |v: f32| (force_velocity(v + 1.0e-3) - force_velocity(v - 1.0e-3)) / 2.0e-3;
        let (below, above) = (slope(-2.0e-3), slope(2.0e-3));
        assert!((below - above).abs() < 0.2 && (above + 5.0).abs() < 0.2, "slopes {below} and {above} at rest");
        let samples: Vec<f32> = (-50..=50).map(|i| force_velocity(i as f32 / 50.0)).collect();
        assert!(samples.windows(2).all(|w| w[1] <= w[0]), "stronger the faster it lengthens, weaker the faster it shortens");
    }

    #[test]
    fn the_muscles_command_reaches_them_as_a_twitch() {
        // A step through the critically damped lag never overshoots and at
        // t = T has 1 − 2/e of it (the twitch's own shape, Winter §9.0.5).
        let (seconds, dt): (f32, f32) = (0.075, 1.0e-4);
        let (mut value, mut rate) = (Vec3::ZERO, Vec3::ZERO);
        let mut peak = 0.0f32;
        for _ in 0..(seconds / dt).round() as usize {
            (value, rate) = activate(value, rate, Vec3::X, seconds, dt);
            peak = peak.max(value.x);
        }
        assert!((value.x - (1.0 - 2.0 / std::f32::consts::E)).abs() < 1.0e-3, "at T: {}", value.x);
        for _ in 0..(10.0 * seconds / dt) as usize {
            (value, rate) = activate(value, rate, Vec3::X, seconds, dt);
            peak = peak.max(value.x);
        }
        assert!(peak <= 1.0 + 1.0e-5 && (value.x - 1.0).abs() < 1.0e-3, "peak {peak}, end {}", value.x);
        // The same at a whole physics step: exact, not integrated.
        let (big, _) = activate(Vec3::ZERO, Vec3::ZERO, Vec3::X, seconds, seconds);
        assert!((big.x - (1.0 - 2.0 / std::f32::consts::E)).abs() < 1.0e-5, "one step of T: {}", big.x);
    }

    #[test]
    fn a_joints_strength_is_per_axis_and_way_and_mirrored() {
        // Plan 4.2: the budgets, not Winter's walking peaks. A standing
        // knee carries ~0.49 N·m/kg into extension (negative about left);
        // its budget that way must clear it well.
        let (knee, _) = joint_strengths(Bone::LeftLeg);
        assert!(knee[0][1] > 2.0 * 0.49, "a knee extending with {} N·m/kg", knee[0][1]);
        // The ankle pushes the toes down far harder than it lifts them.
        let (ankle, _) = joint_strengths(Bone::LeftFoot);
        assert!(ankle[0][0] > 3.0 * ankle[0][1], "plantarflexion {} against dorsiflexion {}", ankle[0][0], ankle[0][1]);
        for bone in Bone::ALL.iter().copied() {
            let (strengths, speed) = joint_strengths(bone);
            assert!(strengths.iter().flatten().all(|s| *s > 0.0 && s.is_finite()) && speed > 0.0, "{}: {strengths:?}", bone.name());
        }
        // The mirror keeps flexion-extension and swaps the ways about
        // forward and up (abduction on the left is the right's negative).
        for (left, right) in [(Bone::LeftFoot, Bone::RightFoot), (Bone::LeftUpLeg, Bone::RightUpLeg), (Bone::LeftArm, Bone::RightArm), (Bone::LeftHand, Bone::RightHand)] {
            let ((l, _), (r, _)) = (joint_strengths(left), joint_strengths(right));
            assert_eq!(l[0], r[0], "{}", left.name());
            assert_eq!(l[1], [r[1][1], r[1][0]], "{}", left.name());
            assert_eq!(l[2], [r[2][1], r[2][0]], "{}", left.name());
        }
    }

    #[test]
    fn an_ankle_torque_is_held_to_what_the_sole_can_give() {
        // A 700 N foot, its sole 0.06 m behind the ankle to 0.18 ahead (+z)
        // and ±0.05 across (x). Without the hold, a glued foot let the
        // ankle push as if the pressure stood anywhere, and a 1.2 m/s push
        // was "caught".
        let sole = [Vec2::new(-0.05, -0.06), Vec2::new(0.05, -0.06), Vec2::new(0.05, 0.18), Vec2::new(-0.05, 0.18)];
        let weight = 700.0;
        // Pressure 0.1 m ahead: τx = dz·W = 70, inside, untouched.
        let inside = Vec3::new(70.0, 3.0, 0.0);
        assert!((within_sole(inside, &sole, weight) - inside).length() < 1.0e-3);
        // 0.3 m ahead is past the toe: held at 0.18 (τx = 126); the twist
        // is kept.
        let held = within_sole(Vec3::new(210.0, 3.0, 0.0), &sole, weight);
        assert!((held - Vec3::new(126.0, 3.0, 0.0)).length() < 1.0e-3, "{held}");
        // 0.1 m to the side (τz = −dx·W = −70): held at 0.05.
        let held = within_sole(Vec3::new(0.0, 0.0, -70.0), &sole, weight);
        assert!((held - Vec3::new(0.0, 0.0, -35.0)).length() < 1.0e-3, "{held}");
    }

    #[test]
    fn a_polygon_holds_what_is_inside_and_clamps_what_is_not() {
        // Two soles side by side, as boxes' corners, and a stray inside.
        let soles = vec![
            Vec2::new(-0.15, -0.05), Vec2::new(-0.05, -0.05), Vec2::new(-0.15, 0.20), Vec2::new(-0.05, 0.20),
            Vec2::new(0.05, -0.05), Vec2::new(0.15, -0.05), Vec2::new(0.05, 0.20), Vec2::new(0.15, 0.20),
            Vec2::new(0.0, 0.1),
        ];
        let hull = convex_hull(soles);
        assert_eq!(hull.len(), 4, "the two soles' hull is their outer box: {hull:?}");
        let (at, inside) = nearest_in_polygon(Vec2::new(0.0, 0.05), &hull);
        assert!(inside && at == Vec2::new(0.0, 0.05));
        let (at, inside) = nearest_in_polygon(Vec2::new(0.0, 0.5), &hull);
        assert!(!inside && (at - Vec2::new(0.0, 0.2)).length() < 1.0e-6, "{at}");
        let (at, _) = nearest_in_polygon(Vec2::new(0.4, -0.3), &hull);
        assert!((at - Vec2::new(0.15, -0.05)).length() < 1.0e-6, "a corner: {at}");
    }

    /// One body on a fixed parent, driven toward zero error; returns the
    /// angle each step.
    fn settle(stiffness: f32, damping: f32, inertia: f32, h: f32, steps: usize) -> Vec<f32> {
        let inverse = Mat3::from_diagonal(Vec3::splat(1.0 / inertia));
        let (mut angle, mut spin) = (0.5f32, 0.0f32);
        (0..steps)
            .map(|_| {
                let impulse = drive_impulse(Vec3::X * angle, Vec3::X * spin, inverse, stiffness, damping, h);
                spin += impulse.x / inertia;
                angle += spin * h;
                angle
            })
            .collect()
    }

    #[test]
    fn a_drive_is_stable_at_any_stiffness_and_step() {
        // Explicitly, kp/I·h² over 4 diverges. Here a 1 kg·cm² foot on
        // 1e6 N·m/rad at a 64 Hz step (kp/I·h² ≈ 2.4e6) still only settles.
        for (stiffness, damping, inertia, h) in
            [(700.0, 70.0, 0.005, 1.0 / 768.0), (1.0e6, 1.0e4, 1.0e-4, 1.0 / 64.0), (10.0, 0.0, 1.0, 1.0 / 64.0)]
        {
            let angles = settle(stiffness, damping, inertia, h, 2000);
            let worst = angles.iter().fold(0.0f32, |a, b| a.max(b.abs()));
            assert!(worst <= 0.5 + 1.0e-6, "kp {stiffness}, I {inertia}, h {h}: grew to {worst}");
            if damping > 0.0 {
                assert!(angles.last().unwrap().abs() < 1.0e-3, "kp {stiffness}: did not settle, {}", angles.last().unwrap());
            }
        }
    }

    #[test]
    fn a_drive_matches_the_spring_at_small_steps() {
        // At a small step, the implicit drive is the plain damped spring:
        // critically damped (kd = 2√(kp·I)) it never overshoots, and at
        // ωt = 3 it has (1 + ωt)e^{-ωt} = 4e⁻³ ≈ 0.199 of its error left.
        let (stiffness, inertia): (f32, f32) = (100.0, 0.25);
        let omega = (stiffness / inertia).sqrt();
        let damping = 2.0 * (stiffness * inertia).sqrt();
        let h = 1.0e-4;
        let angles = settle(stiffness, damping, inertia, h, (3.0 / omega / h) as usize);
        assert!(angles.iter().all(|a| *a > -1.0e-6), "a critically damped drive overshot");
        let expected = 0.5 * 4.0 * (-3.0f32).exp();
        assert!((angles.last().unwrap() - expected).abs() < 2.0e-3, "after ωt = 3: {} against {expected}", angles.last().unwrap());
    }

    #[test]
    fn the_error_is_measured_relative_to_the_parent() {
        // A parent turned 30° about Y carries the child's target with it:
        // a child turned the same is on target.
        let turn = Quat::from_rotation_y(0.5);
        let error = drive_error(turn, turn, Quat::IDENTITY, Quat::IDENTITY);
        assert!(error.length() < 1.0e-5, "{error}");
        // Unturned, the child is 0.5 rad off, about the turn's axis back.
        let error = drive_error(Quat::IDENTITY, turn, Quat::IDENTITY, Quat::IDENTITY);
        assert!((error - Vec3::NEG_Y * 0.5).length() < 1.0e-5, "{error}");
    }
}
