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
use bevy::math::{Mat3, Quat, Vec3};
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
    drives: Query<(Entity, &JointDrive, &JointTarget)>,
    targets: Query<&JointTarget>,
    mut bodies: Query<(&mut SolverBody, &SolverBodyInertia, &Rotation)>,
    time: Res<Time>,
) {
    let h = time.delta_secs();
    if h <= 0.0 {
        return;
    }
    for (child, drive, child_target) in &drives {
        let Ok(parent_target) = targets.get(drive.parent) else { continue };
        let Ok([(mut child_body, child_inertia, child_rotation), (mut parent_body, parent_inertia, parent_rotation)]) =
            bodies.get_many_mut([child, drive.parent])
        else {
            continue;
        };
        let child_now = (child_body.delta_rotation.0 * child_rotation.0).normalize();
        let parent_now = (parent_body.delta_rotation.0 * parent_rotation.0).normalize();
        let error = drive_error(child_now, parent_now, child_target.target, parent_target.target);
        let relative = child_body.angular_velocity - parent_body.angular_velocity;
        let (child_inverse, parent_inverse) =
            (child_inertia.effective_inv_angular_inertia().to_mat3(), parent_inertia.effective_inv_angular_inertia().to_mat3());
        let sized_on = if drive.child_grounded { parent_inverse } else { child_inverse + parent_inverse };
        let impulse = drive_impulse(error, relative, sized_on, drive.stiffness, drive.damping, h) + drive.feedforward * h;
        if !drive.child_grounded {
            child_body.angular_velocity += child_inverse * impulse;
        }
        parent_body.angular_velocity -= parent_inverse * impulse;
    }
}

/// Gives a character standing on its own feet (`Ragdoll::carries_itself`)
/// its drives and frees its root; takes the drives away, and the planted
/// feet's dominance with them, when it no longer does (a fall).
pub(crate) fn manage_joint_drives(
    mut commands: Commands,
    characters: Query<&Ragdoll>,
    masses: Query<&ComputedMass>,
    drives: Query<&JointDrive>,
    roots: Query<(), With<KinematicRoot>>,
) {
    for ragdoll in &characters {
        let bodies: Vec<(Bone, Entity)> = ragdoll.bodies.iter().filter_map(|(bone, body)| Some((bone, (*body)?))).collect();
        if ragdoll.carries_itself() {
            let total: f32 = bodies.iter().filter_map(|(_, body)| masses.get(*body).ok()).map(|mass| mass.value()).sum();
            for &(bone, body) in &bodies {
                if bone == Bone::Hips && roots.get(body).is_ok() {
                    commands.entity(body).remove::<KinematicRoot>().insert(RigidBody::Dynamic);
                }
                let Some(parent) = nearest_simulated_ancestor(bone, ragdoll).and_then(|parent| ragdoll.bodies[parent]) else { continue };
                if drives.get(body).is_err() {
                    let stiffness = DRIVE_STIFFNESS_PER_KG * total * drive_share(bone);
                    commands.entity(body).insert(JointDrive {
                        feedforward: Vec3::ZERO,
                        parent,
                        stiffness,
                        damping: stiffness * DRIVE_DAMPING_SECONDS,
                        child_grounded: false,
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
pub(crate) fn carry_weight(
    characters: Query<&Ragdoll>,
    mut drives: Query<&mut JointDrive>,
    bodies: Query<(&Position, &Rotation, &ComputedMass, &ComputedCenterOfMass)>,
    kinds: Query<&RigidBody>,
    contacts: Res<ContactGraph>,
    mut commands: Commands,
    gravity: Res<Gravity>,
) {
    const FEET: [Bone; 2] = [Bone::LeftFoot, Bone::RightFoot];
    for ragdoll in &characters {
        if !ragdoll.carries_itself() {
            continue;
        }
        let all: Vec<Bone> = Bone::ALL.iter().copied().filter(|&bone| ragdoll.bodies[bone].is_some()).collect();
        // Each body's centre of mass and mass, and its bone's joint point.
        let mut state = super::rig::BoneSet::splat((Vec3::ZERO, 0.0f32, Vec3::ZERO));
        for &bone in &all {
            let Ok((position, rotation, mass, centre)) = bodies.get(ragdoll.bodies[bone].unwrap()) else { return };
            state[bone] = (position.0 + rotation.0 * centre.0, mass.value(), position.0 - rotation.0 * ragdoll.body_offsets[bone]);
        }
        let below = |top: Bone| -> Vec<Bone> {
            all.iter()
                .copied()
                .filter(|&bone| std::iter::successors(Some(bone), |b| b.parent()).any(|b| b == top))
                .collect()
        };
        let legs = [below(Bone::LeftUpLeg), below(Bone::RightUpLeg)];
        let planted: Vec<usize> = (0..2)
            .filter(|&leg| ragdoll.bodies[FEET[leg]].is_some_and(|foot| touches_ground(foot, &contacts, &kinds)))
            .collect();
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
        // Each planted leg's share of what no planted leg carries.
        let upper: Vec<Bone> = all.iter().copied().filter(|b| !planted.iter().any(|&leg| legs[leg].contains(b))).collect();
        let mut share = [0.0f32; 2];
        match planted[..] {
            [leg] => share[leg] = 1.0,
            [left, right] => {
                let total: f32 = all.iter().map(|&b| state[b].1).sum();
                let com = all.iter().fold(Vec3::ZERO, |sum, &b| sum + state[b].0 * state[b].1) / total;
                let (l, r) = (state[FEET[left]].2, state[FEET[right]].2);
                let across = Vec3::new(r.x - l.x, 0.0, r.z - l.z);
                let t = (Vec3::new(com.x - l.x, 0.0, com.z - l.z).dot(across) / across.length_squared().max(1.0e-6)).clamp(0.0, 1.0);
                share[left] = 1.0 - t;
                share[right] = t;
            }
            _ => {}
        }
        for &bone in &all {
            let Ok(mut drive) = drives.get_mut(ragdoll.bodies[bone].unwrap()) else { continue };
            let joint = state[bone].2;
            let moment = |set: &[Bone], weight: f32| {
                set.iter().fold(Vec3::ZERO, |sum, &b| sum + (state[b].0 - joint).cross(gravity.0 * state[b].1 * weight))
            };
            let subtree = below(bone);
            drive.feedforward = if planted.is_empty() {
                Vec3::ZERO
            } else if let Some(leg) = planted.iter().copied().find(|&leg| legs[leg].contains(&bone)) {
                let above: Vec<Bone> = legs[leg].iter().copied().filter(|b| !subtree.contains(b)).collect();
                moment(&above, 1.0) + moment(&upper, share[leg])
            } else {
                -moment(&subtree, 1.0)
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
