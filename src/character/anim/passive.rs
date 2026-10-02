//! Passive joint tone: what an unconscious body's joints still do.
//!
//! A fallen ragdoll had no pose control (`ragdoll::FALL_TONE` = 0), only a
//! uniform joint damping, and hard anatomical stops. So its limbs swung
//! freely until they struck a stop: a puppet's. Headless, four falls ended
//! with hips on their stops (−29.9° of the 30° extension limit, 116.6° of
//! 120° flexion, abduction 44.4° of 45°), a backward fall's knees locked
//! straight at the −5° stop, and every elbow at its −5° stop.
//!
//! A real body without muscle activity is not that soft. Muscle's parallel
//! elastic element and the joints' connective tissue resist stretch, and
//! the more the further they are stretched (Winter §9.1, the passive
//! force-length curve), with a viscous part (Winter §9.3). Measured as joint
//! moments (Riener & Edrich 1999), the knee's is a few N·m in mid-range and
//! rises steeply toward its ends: with the hip straight, +10 N·m fully
//! extended, −4.5 at 60° of flexion, −16.5 at 130°, zero near 14-43°
//! depending on the hip.
//!
//! So while falling each joint pulls its body, relative to its parent, toward
//! the pose a relaxed body settles in (`getup::relaxed`, the neutral body
//! posture measured in weightlessness), with a stiffness that rises with
//! the stretch:
//!
//! ```text
//!   k(θ) = k₀ · (1 + (θ / θs)²)
//! ```
//!
//! `θ` the angle from that pose, `k₀` the joint's mid-range stiffness, `θs`
//! where it has doubled. It is weaker than gravity in mid-range, so a limb
//! lying on the floor still lies there. It is applied as a joint torque
//! (the parent takes the reaction), solved implicitly every substep
//! (`joint_drive::drive_impulse`), so a stiff end of range cannot go
//! unstable on a light hand.

use avian3d::dynamics::solver::solver_body::{SolverBody, SolverBodyInertia};
use avian3d::prelude::*;
use bevy::prelude::*;

use super::joint_drive::{drive_error, drive_impulse};
use crate::character::skeleton::Bone;

/// A falling body's passive pull toward where its joint relaxes. On the
/// child body; added as a fall begins and removed at the re-pin.
#[derive(Component, Debug, Clone, Copy, PartialEq)]
pub struct PassiveJoint {
    /// The parent body.
    pub parent: Entity,
    /// The child body's rotation relative to the parent's at rest, in the
    /// parent body's frame (`parent⁻¹ · child`).
    pub relaxed: Quat,
    /// Mid-range stiffness, N·m/rad.
    pub stiffness: f32,
    /// The angle from relaxed at which the stiffness has doubled, radians.
    pub stiffens: f32,
    /// N·m·s/rad of relative spin.
    pub damping: f32,
    /// The one axis the pull acts about, in the parent body's frame, if
    /// not all three.
    /// - A joint that is a hinge while falling (a knee, an elbow): its axis.
    ///   The hinge holds the limb's roll and sideways tilt where the fall
    ///   found them; pulled toward the relaxed pose's in all three
    ///   directions, a shin rolled against its hinge at 1.4-2 rad/s and
    ///   the body never came to rest.
    pub hinge: Option<Vec3>,
}

/// A joint's passive mid-range stiffness, N·m/rad, the angle from relaxed
/// at which it has doubled, radians, and its damping, N·m·s/rad, for a
/// 75 kg adult (scaled by body mass, `passive_joint`).
///
/// - Knee: fitted to Riener & Edrich's (4 N·m/rad, doubling 86° out):
///   4.1, 9.5 and 22.7 N·m at 60°, 90° and 130° of flexion against their
///   4.5, 6.2 and 16.5 (`the_knees_passive_moment_is_…`).
/// - Hip: of the same order and stiffer, two-joint muscles crossing it, and
///   nearer its extension stop; damping 1.9-4.6 N·m·s/rad measured, the
///   knee's far less.
/// - Shoulder, elbow, wrist: lighter segments, scaled down with them.
/// - Trunk: the spine's ligaments and discs, stiffer than a limb.
/// - Neck: under what holds the head level against gravity (~4 N·m), so a
///   limp head lolls, but it does not drop to its stop.
/// - Ankle: none (`None`). A light foot lying on the floor, pulled by its
///   ankle, fought the floor's friction: at 3 N·m/rad feet crept and
///   jolted (0.05-0.5 m/s) for 8 s in falls that rested without it, about
///   the flexion axis alone too, and at 1 N·m/rad one landing in three
///   still crept. Its limits and the fall's joint damping hold it, as
///   before.
pub fn passive_gains(bone: Bone) -> Option<(f32, f32, f32)> {
    let degrees = |d: f32| d.to_radians();
    Some(match bone {
        Bone::Spine | Bone::Spine1 | Bone::Spine2 => (30.0, degrees(30.0), 4.0),
        Bone::Neck | Bone::Head => (2.5, degrees(40.0), 0.3),
        Bone::LeftUpLeg | Bone::RightUpLeg => (10.0, degrees(70.0), 3.0),
        Bone::LeftLeg | Bone::RightLeg => (4.0, degrees(86.0), 0.5),
        Bone::LeftFoot | Bone::RightFoot | Bone::LeftToeBase | Bone::RightToeBase => return None,
        Bone::LeftArm | Bone::RightArm => (3.0, degrees(70.0), 0.5),
        Bone::LeftForeArm | Bone::RightForeArm => (2.0, degrees(70.0), 0.2),
        Bone::LeftHand | Bone::RightHand => (0.4, degrees(50.0), 0.05),
        _ => (2.0, degrees(40.0), 0.2),
    })
}

/// The body mass the gains in [`passive_gains`] are for, kg.
pub const REFERENCE_MASS: f32 = 75.0;

/// The passive joint for `bone`'s body hanging from `parent`, the two
/// drawn at rest as `child_relaxed` and `parent_relaxed` (world), on a body
/// of `mass` kg; `None` for a joint without passive tone.
pub fn passive_joint(bone: Bone, parent: Entity, child_relaxed: Quat, parent_relaxed: Quat, mass: f32) -> Option<PassiveJoint> {
    let (stiffness, stiffens, damping) = passive_gains(bone)?;
    let scale = mass / REFERENCE_MASS;
    Some(PassiveJoint {
        parent,
        relaxed: (parent_relaxed.inverse() * child_relaxed).normalize(),
        stiffness: stiffness * scale,
        stiffens,
        damping: damping * scale,
        hinge: None,
    })
}

/// The stiffness at `angle` from relaxed: `k₀ · (1 + (θ / θs)²)`.
pub fn stiffness_at(joint: &PassiveJoint, angle: f32) -> f32 {
    joint.stiffness * (1.0 + (angle / joint.stiffens).powi(2))
}

/// Applies every [`PassiveJoint`] for one substep, on the solver's own body
/// state, as `joint_drive::apply_joint_drives` does.
pub(crate) fn apply_passive_joints(
    joints: Query<(Entity, &PassiveJoint)>,
    mut bodies: Query<(&mut SolverBody, &SolverBodyInertia, &Rotation)>,
    time: Res<Time>,
) {
    let h = time.delta_secs();
    if h <= 0.0 {
        return;
    }
    for (child, joint) in &joints {
        let Ok([(mut child_body, child_inertia, child_rotation), (mut parent_body, parent_inertia, parent_rotation)]) =
            bodies.get_many_mut([child, joint.parent])
        else {
            continue;
        };
        let child_now = (child_body.delta_rotation.0 * child_rotation.0).normalize();
        let parent_now = (parent_body.delta_rotation.0 * parent_rotation.0).normalize();
        let mut error = drive_error(child_now, parent_now, joint.relaxed, Quat::IDENTITY);
        let mut relative = child_body.angular_velocity - parent_body.angular_velocity;
        if let Some(axis) = joint.hinge {
            let axis = (parent_now * axis).normalize();
            (error, relative) = (axis * error.dot(axis), axis * relative.dot(axis));
        }
        let (child_inverse, parent_inverse) =
            (child_inertia.effective_inv_angular_inertia().to_mat3(), parent_inertia.effective_inv_angular_inertia().to_mat3());
        let stiffness = stiffness_at(joint, error.length());
        let impulse = drive_impulse(error, relative, child_inverse + parent_inverse, stiffness, joint.damping, h);
        child_body.angular_velocity += child_inverse * impulse;
        parent_body.angular_velocity -= parent_inverse * impulse;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stiffness_doubles_at_its_angle_and_rises_with_the_stretch() {
        let joint = passive_joint(Bone::LeftLeg, Entity::PLACEHOLDER, Quat::IDENTITY, Quat::IDENTITY, REFERENCE_MASS).unwrap();
        assert!((stiffness_at(&joint, 0.0) - joint.stiffness).abs() < 1.0e-6);
        assert!((stiffness_at(&joint, joint.stiffens) - 2.0 * joint.stiffness).abs() < 1.0e-5);
        assert!(stiffness_at(&joint, 2.0 * joint.stiffens) > stiffness_at(&joint, joint.stiffens));
    }

    #[test]
    fn the_knees_passive_moment_is_of_the_order_riener_and_edrich_measured() {
        // Their knee, hip straight, ankle neutral: −4.5 N·m 46° past its
        // zero (60° of flexion) and −16.5 N·m 116° past it (130°), degrees
        // in the exponents. The model need not fit the curve, only its
        // size: within a factor of two over the range a fall uses.
        let riener = |knee: f32| {
            (1.800 - 0.0352 * knee).exp() - (-3.971 + 0.0495 * knee).exp() + (2.220 - 0.150 * knee).exp() - 4.820
        };
        let joint = passive_joint(Bone::LeftLeg, Entity::PLACEHOLDER, Quat::IDENTITY, Quat::IDENTITY, REFERENCE_MASS).unwrap();
        for knee in [60.0f32, 90.0, 130.0] {
            // From their zero, about 14°.
            let angle = (knee - 14.0).to_radians();
            let ours = stiffness_at(&joint, angle) * angle;
            let theirs = -riener(knee);
            assert!(
                ours > 0.5 * theirs && ours < 2.0 * theirs,
                "at {knee}° of flexion: {ours:.1} N·m against Riener and Edrich's {theirs:.1}",
            );
        }
    }

    #[test]
    fn a_passive_joint_relaxes_relative_to_its_parent_not_the_world() {
        // A turn of the whole pair is no stretch: the pull is between the
        // two bodies (unlike the standing pose's world targets, which a fall
        // at tone 0.15 kept chasing, a forearm moving 0.57 m/s after 3 s).
        let relaxed = Quat::from_rotation_x(0.6);
        let joint = passive_joint(Bone::LeftLeg, Entity::PLACEHOLDER, relaxed, Quat::IDENTITY, REFERENCE_MASS).unwrap();
        let turn = Quat::from_rotation_y(1.3) * Quat::from_rotation_z(0.4);
        let error = drive_error(turn * relaxed, turn, joint.relaxed, Quat::IDENTITY);
        assert!(error.length() < 1.0e-5, "a turned pair at rest read {error}");
    }
}
