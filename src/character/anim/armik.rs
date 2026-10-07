//! Two-bone IK for an arm — reaching, and holding a grip point.
//!
//! The arm is the same mathematical problem as the leg ([`super::legik`]):
//! a fixed pivot, two segments, and an end effector to place. What differs
//! is not the maths but the *purpose*, and that changes two things.
//!
//! # A hand is aimed, a foot is planted
//!
//! A foot's job is to stop moving — [`super::footlock`] pins it and the leg
//! solves around it. A hand's job is usually to point at something: gripping
//! a handle, holding a weapon along its axis, pressing a palm to a wall. So
//! the arm chain solves the wrist position AND optionally rotates the hand to
//! a requested world orientation, which the leg never needs.
//!
//! # The bone names are shifted a joint, again
//!
//! This is the trap that cost a whole debugging session on the legs, and the
//! arm repeats it exactly. On the real rig (`puppet_base.gltf`, measured, not
//! assumed):
//!
//! | bone | offset length | what it actually spans |
//! |---|---|---|
//! | `LeftArm` | 0.2097 m | clavicle to shoulder joint |
//! | `LeftForeArm` | 0.2511 m | **the upper arm** |
//! | `LeftHand` | 0.2436 m | **the forearm** |
//!
//! A bone's offset is the vector from its PARENT to itself, so the segment
//! called "the upper arm" is stored on the bone named `LeftForeArm`. Reading
//! `offsets[LeftArm]` as the upper arm — the obvious misreading — picks up the
//! clavicle-to-shoulder stub instead and shortens the arm by 20%.
//! [`ArmChain`] therefore names its fields by ROLE (`shoulder`, `elbow`,
//! `wrist`) and `the_arm_bone_names_are_shifted_one_joint` pins the measured
//! lengths so a rig change cannot quietly invalidate the mapping.
//!
//! # The elbow axis is supplied, never derived
//!
//! The rest pose is a T-pose with the arm dead straight: 0.4947 m of reach
//! carrying **0.2 mm** of slack, an elbow interior angle of 176.5 degrees.
//! That is the same critical-extension singularity the legs sit at, and it is
//! why a cross-product-derived hinge cannot work — the bend plane is
//! degenerate exactly when the limb is straightest, which is its rest state.
//!
//! The axis was measured rather than reasoned about, because the analogous
//! `KNEE_AXIS` comment shipped inverted and ran an entire walk cycle
//! backwards. Rotating the elbow 45 degrees on the real rig moves the wrist:
//!
//! | axis | wrist displacement | meaning |
//! |---|---|---|
//! | `+X` | 0.0058 m | **the arm's own long axis: twist** |
//! | `+Y` | 0.1864 m | the hinge — swings the hand forward |
//! | `+Z` | 0.1863 m | bends, but sideways |
//!
//! `+X` is the one to note: the upper arm runs along world `(0.9995, 0.0,
//! -0.0303)`, so a delta about `+X` rotates the forearm about itself and moves
//! the wrist essentially nowhere. A solver configured with it silently fails to
//! bend. Pinned by `the_configured_elbow_axis_actually_bends_the_elbow`, which
//! asserts both halves — that the default bends and that `+X` does not.
//!
//! An earlier version of this table said the opposite, naming `+Y` as the dead
//! axis. That measurement was taken through a broken pose-space convention (see
//! [`rig::accumulate_world_rotations`](super::rig::accumulate_world_rotations));
//! it was a real observation of a wrong system. Worth remembering that
//! "measured, not assumed" is necessary and not sufficient — what is measured
//! also has to be correct.
//!
//! # The axis is perpendicularized before use, and the leg's is not
//!
//! `Quat::from_axis_angle(hinge, angle) * direction` yields a vector `angle`
//! away from `direction` **only if the hinge is perpendicular to it**.
//! Otherwise it sweeps a cone and the component of the hinge lying along the
//! target direction tilts the result out of the bend plane.
//!
//! This is easy to miss because the leg solver ignores it and works anyway: a
//! foot target is nearly straight down, already almost perpendicular to its
//! `+X` knee axis. An arm reaches in every direction, including straight out
//! along its own hinge where the raw axis is *parallel* to the target and the
//! construction degenerates completely. Measured on a target 0.27 m from the
//! shoulder, the raw axis left the wrist **0.167 m** from a point well inside
//! reach, and no choice of axis or sign got below 0.084 m. Subtracting the
//! along-target component makes it exact.
//!
//! # The bend sign mirrors between the two arms
//!
//! A single world-axis hinge cannot serve both arms: the left upper arm runs
//! along `+X` and the right along `-X`, so the rotational sense that swings one
//! elbow forward swings the other back. [`ArmChain::bend_sign`] carries it.
//!
//! The wrist cannot detect this — the two-bone geometry lands it correctly
//! either way — so the symptom is only visible at the elbow. Measured with a
//! shared hinge and mirrored targets: elbows at `z = -0.291` and `z = +0.055`,
//! with both wrists exactly mirrored.
//!
//! # Two bugs this module found in shared code
//!
//! Both were in code the legs had been using all along, and both were invisible
//! there for the same reason: a conjugation by a bind rotation is a no-op when
//! the delta's axis is parallel to the bind's, and the leg chain is bound about
//! `X` while every leg delta a walk cycle produces is *also* about `X`.
//!
//! 1. **Forward kinematics applied the delta in the wrong frame** — the local
//!    one rather than the world one the renderer uses. So a solver could hit a
//!    target exactly and the character still render its hand 0.36 m away. See
//!    [`rig::accumulate_world_rotations`](super::rig::accumulate_world_rotations).
//! 2. **`aim_bone` conjugated by the wrong frame**, which only shows up once an
//!    ancestor is posed — which is exactly what the arm solver does when it
//!    aims the forearm right after swinging the shoulder. See
//!    [`legik::world_correction_frame`](super::legik).

use bevy::math::{Quat, Vec3};

use super::legik::aim_bone;
use super::math::ik::solve_two_bone;
use super::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// Which joints make up one arm.
///
/// # Why the shoulder joint is the pivot and not part of the chain
///
/// The same reason `LegChain` starts at the hip socket: `LeftShoulder`
/// (the clavicle) hangs off `Spine2`, which also carries the neck, the head
/// and the other arm. Rotating it to place one hand would swing all of them.
///
/// So the clavicle is left alone and the chain solves the two segments below
/// the shoulder joint. A shrug or a shoulder roll is animation's business,
/// composed before the solve; IK reads whatever the clavicle is doing and
/// works from there.
// No `Eq`: `bend_sign` is an `f32`. It only ever holds ±1.0, but encoding that
// as a type is not worth a newtype here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmChain {
    /// The shoulder joint. A fixed pivot — never rotated by the solve.
    ///
    /// Note the name: this is `Bone::LeftArm`, not `Bone::LeftShoulder`.
    /// `LeftShoulder` is the clavicle.
    pub shoulder: Bone,
    /// Rotating this aims the upper arm. Named `LeftForeArm` on the rig; see
    /// the module doc's table.
    pub elbow: Bone,
    /// The wrist, and the joint the chain solves to place.
    pub wrist: Bone,
    /// Which way this arm's elbow bend is measured, `+1.0` or `-1.0`.
    ///
    /// # Why the hinge axis cannot be one shared value
    ///
    /// [`ArmIkConfig::elbow_axis`] names the hinge in world axes, and the two
    /// arms point opposite ways: the left upper arm runs along world `+X`, the
    /// right along `-X`. A hinge that swings the left elbow forward swings the
    /// right one backward, because "forward" relative to a mirrored bone is the
    /// other rotational sense.
    ///
    /// Measured with a single shared `+Y` hinge and mirrored targets, both arms
    /// put their WRIST in the mirrored place — that much the two-bone geometry
    /// guarantees — while the elbows ended at `z = -0.291` and `z = +0.055`:
    /// one bent forward, one back. Pinned by
    /// `both_arms_solve_mirrored_targets_to_mirrored_results`, which checks the
    /// elbow depth and not only the wrist, precisely because the wrist alone
    /// cannot see this.
    pub bend_sign: f32,
}

impl ArmChain {
    /// The left arm.
    pub const LEFT: Self = Self {
        shoulder: Bone::LeftArm,
        elbow: Bone::LeftForeArm,
        wrist: Bone::LeftHand,
        bend_sign: 1.0,
    };

    /// The right arm.
    pub const RIGHT: Self = Self {
        shoulder: Bone::RightArm,
        elbow: Bone::RightForeArm,
        wrist: Bone::RightHand,
        bend_sign: -1.0,
    };
}

/// How the arm solver should behave.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmIkConfig {
    /// How gently an out-of-reach target is absorbed, metres.
    ///
    /// The tempting choice is a generous zone — an arm straining for something
    /// just out of range reads better than one snapping to full extension, and
    /// unlike a leg it carries no weight. But the softening zone applies to
    /// targets INSIDE reach too, and the rest arm sits only 0.0118 m off full
    /// extension, so a wide zone is active during ordinary reaching.
    ///
    /// Measured: at `0.02` a target exactly at the arm's own current wrist came
    /// back 0.0015 m short — a held grip would creep inward every frame it was
    /// re-solved. At `0.005` and below the no-op is exact. So this matches the
    /// leg's value, and the straining look has to come from somewhere that does
    /// not tax the common case.
    pub softening: f32,
    /// The axis the elbow hinges about, in the rig's own frame.
    ///
    /// Supplied rather than derived, for the reason in the module doc: the
    /// rest arm is straight to within 0.2 mm, so any cross product picks an
    /// unstable axis in the pose the arm spends most of its time near.
    pub elbow_axis: Vec3,
    /// Whether to rotate the hand to [`ArmTarget::hand_rotation`] after
    /// placing the wrist.
    ///
    /// Off by default. A reach only needs the hand to arrive; a grip needs it
    /// oriented, and orientation is the part most likely to look wrong, so it
    /// is opt-in.
    pub aim_hand: bool,
}

impl Default for ArmIkConfig {
    fn default() -> Self {
        Self {
            softening: 0.005,
            // Measured on the real rig. The upper arm runs along world +X
            // (0.9995, 0.0, -0.0303), so +X is the arm's own long axis — a
            // delta about it twists the forearm and moves the wrist 0.0058 m,
            // i.e. nowhere. +Y and +Z both bend it properly (0.186 m each);
            // +Y is the one that swings the hand toward the FRONT of the body,
            // which is the way an elbow goes.
            //
            // This default read `Vec3::X` until the pose-space convention was
            // corrected. Under the old (local-axis) composition the degenerate
            // axis appeared to be +Y, and the module doc said so — a measured
            // observation that was nonetheless wrong, because the thing it
            // measured was broken. See `rig::accumulate_world_rotations`.
            elbow_axis: Vec3::Y,
            aim_hand: false,
        }
    }
}

/// Where an arm should put its hand.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmTarget {
    /// Where the wrist should end up, in world space.
    pub wrist: Vec3,
    /// How the hand should be oriented once it gets there.
    ///
    /// `None` leaves whatever rotation the pose authored, which is what a
    /// plain reach wants. `Some` is for a grip — a handle held along its own
    /// axis, a palm flat against a surface — and needs
    /// [`ArmIkConfig::aim_hand`] on to take effect.
    pub hand_rotation: Option<Quat>,
}

impl ArmTarget {
    /// Reach for a point, leaving the hand's orientation to the pose.
    pub fn reach(wrist: Vec3) -> Self {
        Self { wrist, hand_rotation: None }
    }

    /// Grip a point with the hand held at a specific world orientation.
    pub fn grip(wrist: Vec3, hand_rotation: Quat) -> Self {
        Self { wrist, hand_rotation: Some(hand_rotation) }
    }
}

/// Places one arm's wrist at its target, editing `pose` in place.
///
/// Returns where the wrist actually ended up, which differs from the request
/// when it was out of reach — the caller can compare the two to see how far
/// the arm is straining, and for instance decide to lean the torso in.
pub fn solve_arm(
    pose: &mut LocalPose,
    chain: ArmChain,
    target: ArmTarget,
    config: &ArmIkConfig,
) -> Vec3 {
    solve_arm_on(pose, chain, target, config, &RigGeometry::default())
}

/// [`solve_arm`] on a specific rig.
///
/// The form to use whenever the target comes from the world — a grab point, a
/// weapon grip, a ledge. [`solve_arm`] is this with the synthetic T-pose,
/// which is correct only when the target was derived from that same proxy.
///
/// The distinction is not cosmetic here: the synthetic rig's upper arm is
/// 0.28 m against the real rig's 0.2511 m, so a target at the edge of one
/// rig's reach is comfortably inside or well outside the other's.
pub fn solve_arm_on(
    pose: &mut LocalPose,
    chain: ArmChain,
    target: ArmTarget,
    config: &ArmIkConfig,
    rig: &RigGeometry,
) -> Vec3 {
    let before = forward_kinematics_on(pose, rig);

    let shoulder = before[chain.shoulder];
    let upper = (before[chain.elbow] - before[chain.shoulder]).length();
    let fore = (before[chain.wrist] - before[chain.elbow]).length();

    let to_target = target.wrist - shoulder;
    let distance = to_target.length();

    if distance < 1.0e-6 || upper < 1.0e-6 || fore < 1.0e-6 {
        return before[chain.wrist];
    }

    let solution = solve_two_bone(upper, fore, distance, config.softening);

    let direction = to_target / distance;

    let requested_axis = config.elbow_axis.normalize_or_zero();
    if requested_axis == Vec3::ZERO {
        return before[chain.wrist];
    }

    // The hinge has to be PERPENDICULAR to the line to the target, or
    // `from_axis_angle(hinge, angle) * direction` does not produce a vector at
    // `angle` from `direction` at all — it sweeps a cone about the hinge, and
    // the component of the hinge lying along `direction` tilts the result out
    // of the intended bend plane.
    //
    // Measured on a target 0.27 m off the shoulder at (0.20, -0.15, -0.10):
    // using the raw `+X` axis left the wrist 0.167 m from a target well inside
    // reach, and NO choice of axis or sign got below 0.084 m. Removing the
    // along-target component makes it exact.
    //
    // The leg solver uses its configured axis raw and mostly gets away with it
    // because a foot target is nearly straight down, which is already almost
    // perpendicular to its `+X` knee axis. An arm reaches in every direction,
    // including straight along `+X`, so the arm cannot.
    let along = requested_axis.dot(direction);
    let hinge = (requested_axis - direction * along).normalize_or_zero();

    // The degenerate case: the target lies exactly along the requested hinge,
    // so there is no bend plane to pick. Any perpendicular is as good as
    // another; take a stable one rather than returning a zero vector.
    let hinge = if hinge == Vec3::ZERO {
        let fallback = if direction.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        direction.cross(fallback).normalize_or_zero()
    } else {
        hinge
    };
    if hinge == Vec3::ZERO {
        return before[chain.wrist];
    }

    // The bend direction, per side. Mirrored because the two arms point
    // opposite ways along `X`, so one world-axis hinge cannot bend both elbows
    // the same way relative to their own arm — see `ArmChain::bend_sign`.
    //
    // Stated as a measurement rather than a derivation, because the analogous
    // `KNEE_AXIS` comment shipped inverted and ran a whole walk cycle
    // backwards. Pinned by `the_elbow_bends_forward_not_backward` (direction)
    // and `both_arms_solve_mirrored_targets_to_mirrored_results` (the mirror).
    let bend = chain.bend_sign;

    let upper_direction =
        Quat::from_axis_angle(hinge, bend * solution.upper_angle) * direction;
    let elbow_position = shoulder + upper_direction * upper;

    let fore_direction =
        Quat::from_axis_angle(hinge, bend * (solution.upper_angle - solution.joint_bend))
            * direction;
    let wrist_position = elbow_position + fore_direction * fore;

    // Re-running forward kinematics between the two aims matters: the second
    // is computed from where the elbow actually is after the first moved it,
    // not from a stale snapshot.
    // Aim the upper arm, then the forearm, re-running forward kinematics
    // between them so the second aim sees where the elbow actually ended up
    // rather than a stale snapshot.
    //
    // `aim_bone` rotates its first argument so that its second lands on the
    // target, and it measures the current direction as
    // `positions[child] - positions[bone]` — so the pair given here is
    // (shoulder joint, elbow) and then (elbow, wrist), which is exactly the
    // two segments being solved.
    aim_bone(pose, &before, chain.shoulder, chain.elbow, elbow_position, rig);

    let after_shoulder = forward_kinematics_on(pose, rig);
    aim_bone(pose, &after_shoulder, chain.elbow, chain.wrist, wrist_position, rig);

    if let Some(wanted) = target.hand_rotation.filter(|_| config.aim_hand) {
        set_hand_rotation(pose, chain, wanted, rig);
    }

    forward_kinematics_on(pose, rig)[chain.wrist]
}

/// Places `chain`'s wrist at `wrist` (the pose's frame), its elbow bending
/// toward `pole`: in the plane of the shoulder-to-wrist line and `pole`, on
/// `pole`'s side. The hand keeps its turn on the forearm.
///
/// For a hand carried in front of the body, where the configured hinge of
/// [`solve_arm_on`] (one world axis for every target) cannot say which way
/// round the elbow goes. Returns where the wrist ended up.
pub fn solve_arm_toward(pose: &mut LocalPose, chain: ArmChain, wrist: Vec3, pole: Vec3, rig: &RigGeometry) -> Vec3 {
    let before = forward_kinematics_on(pose, rig);
    solve_arm_toward_from(pose, &before, chain, wrist, pole, rig).1
}

/// [`solve_arm_toward`] from `before`, the pose's joint positions as they
/// stand for this arm (another arm solved since leaves them so); returns
/// where it put the elbow and the wrist.
pub fn solve_arm_toward_from(
    pose: &mut LocalPose,
    before: &super::rig::BoneSet<Vec3>,
    chain: ArmChain,
    wrist: Vec3,
    pole: Vec3,
    rig: &RigGeometry,
) -> (Vec3, Vec3) {
    let shoulder = before[chain.shoulder];
    let upper = (before[chain.elbow] - before[chain.shoulder]).length();
    let fore = (before[chain.wrist] - before[chain.elbow]).length();
    let to_target = wrist - shoulder;
    let distance = to_target.length();
    let unmoved = (before[chain.elbow], before[chain.wrist]);
    if distance < 1.0e-6 || upper < 1.0e-6 || fore < 1.0e-6 {
        return unmoved;
    }
    let direction = to_target / distance;
    // The pole's part square to the line: the side the elbow goes.
    let side = (pole - direction * pole.dot(direction)).normalize_or_zero();
    if side == Vec3::ZERO {
        return unmoved;
    }
    // Next to no softening: a standing arm is within 0.4 mm of straight,
    // inside the usual 5 mm of it, and asked for its own wrist its elbow
    // jumped 12 mm out (`sneak::tests::carried_by_nothing_the_arms_are_left_as_they_are`).
    let solution = solve_two_bone(upper, fore, distance, 1.0e-4);
    let elbow = shoulder + (direction * solution.upper_angle.cos() + side * solution.upper_angle.sin()) * upper;
    let reached = shoulder + direction * solution.reach;
    super::legik::aim_bone(pose, before, chain.shoulder, chain.elbow, elbow, rig);
    let after = forward_kinematics_on(pose, rig);
    super::legik::aim_bone(pose, &after, chain.elbow, chain.wrist, reached, rig);
    (elbow, reached)
}

/// Rotates the hand to a world-space orientation.
///
/// Unlike every other correction here this one is an assignment rather than a
/// delta: a grip specifies the orientation absolutely, so composing onto
/// whatever the pose had would make the result depend on the incoming
/// animation. Solving the wrist position first and orienting second is the
/// right order because placing the wrist rotates the forearm, which changes
/// the frame this rotation has to be expressed in.
fn set_hand_rotation(pose: &mut LocalPose, chain: ArmChain, wanted: Quat, rig: &RigGeometry) {
    // Forward kinematics gives the hand's world rotation as
    //   W = P * d * B,   P = W(parent) * bind_local * B^-1
    // with `B` the accumulated bind and `d` the stored delta (see
    // `legik::world_correction_frame` for the derivation of `P`). Setting
    // `W = wanted` and solving:
    //   d = P^-1 * wanted * B^-1
    //
    // `P` carries the shoulder and elbow rotations the two aims just made,
    // which is exactly why the orientation has to be set AFTER the position and
    // not before.
    let frame = super::legik::world_correction_frame(pose, chain.wrist, rig);
    let bind = super::rig::accumulate_bind_rotations(rig)[chain.wrist];

    pose.set_rotation(chain.wrist, frame.inverse() * wanted * bind.inverse());
}

/// Where one arm's wrist currently is, on a specific rig.
///
/// A convenience for the common "hold the hand where it already is" case —
/// read it once when a grip begins, then feed it back as the target while the
/// body moves underneath.
pub fn wrist_position(pose: &LocalPose, chain: ArmChain, rig: &RigGeometry) -> Vec3 {
    forward_kinematics_on(pose, rig)[chain.wrist]
}

/// The most a clavicle turns lifting its shoulder straight up, radians, in
/// [`shoulder_lift`]; a reach forward lifts it half as far, one down a
/// quarter.
pub(super) const MOST_SHOULDER_LIFT: f32 = 0.4;

/// The most a forearm rolls about its own line turning a hand onto what it
/// holds ([`turn_hand`]), radians.
pub(super) const MOST_FOREARM_TWIST: f32 = 1.4;

/// The turn of a clavicle rooted at `root` that swings its shoulder joint
/// (at `shoulder`) toward `grip`, just far enough to bring it within `within`
/// of it: none when it already is. At most [`MOST_SHOULDER_LIFT`] lifting
/// straight up, half that reaching forward, a quarter down; never past
/// pointing at the grip. Turn `Bone::LeftShoulder` (or the right) by it: its
/// rotation swings the shoulder joint about the clavicle's root.
///
/// The shoulder joint's distance from the grip as the clavicle turns by `θ`
/// toward it is `|d|² + |v|² - 2|d||v|cos(φ - θ)`, `v` the clavicle, `d` the
/// root to the grip, `φ` between them: solved for `within`.
pub(super) fn shoulder_lift(root: Vec3, shoulder: Vec3, grip: Vec3, within: f32) -> Quat {
    let (v, d) = (shoulder - root, grip - root);
    let (lv, ld) = (v.length(), d.length());
    let axis = v.cross(d);
    if lv < 1.0e-6 || ld < 1.0e-6 || axis.length_squared() < 1.0e-12 {
        return Quat::IDENTITY;
    }
    let axis = axis.normalize();
    let phi = (v.dot(d) / (lv * ld)).clamp(-1.0, 1.0).acos();
    let k = (ld * ld + lv * lv - within * within) / (2.0 * ld * lv);
    let theta = if k <= phi.cos() {
        0.0
    } else if k >= 1.0 {
        phi
    } else {
        phi - k.acos()
    };
    // Which way the shoulder sets off: up, forward, down.
    let rise = axis.cross(v).normalize_or_zero().y;
    let most = MOST_SHOULDER_LIFT * (0.5 + 0.5 * rise).clamp(0.25, 1.0);
    Quat::from_axis_angle(axis, theta.clamp(0.0, most.min(phi)))
}

/// The world turn that carries a hand from its rest, its fingers along
/// `rest_along` and its palm facing `rest_palm`, to point its fingers along
/// `along`, its palm facing `palm`.
pub(super) fn frame_turn(rest_along: Vec3, rest_palm: Vec3, along: Vec3, palm: Vec3) -> Quat {
    let basis = |along: Vec3, palm: Vec3| {
        let along = along.normalize_or_zero();
        let palm = (palm - along * palm.dot(along)).normalize_or_zero();
        bevy::math::Mat3::from_cols(along, palm, along.cross(palm))
    };
    Quat::from_mat3(&(basis(along, palm) * basis(rest_along, rest_palm).transpose())).normalize()
}

/// Turns `chain`'s hand toward the world turn `wanted` from its rest (the
/// pose's frame; `bind` the hand's bind rotation), by `weight` (0 not at
/// all): its roll about the forearm's line (`forearm`) by the forearm, up to
/// [`MOST_FOREARM_TWIST`], the rest at the wrist. The wrist does not move.
///
/// A hand turned at the wrist alone to face its palm onto a rung twisted it
/// by the forearm's whole roll.
pub(super) fn turn_hand(pose: &mut LocalPose, rig: &RigGeometry, chain: ArmChain, bind: Quat, wanted: Quat, weight: f32, forearm: Vec3) {
    use super::rig::{accumulate_world_rotations, delta_after_world_turn};
    let carried = |pose: &LocalPose| (accumulate_world_rotations(pose, rig)[chain.wrist] * bind.inverse()).normalize();
    let now = carried(pose);
    let wanted = if now.dot(wanted) < 0.0 { -wanted } else { wanted };
    let target = now.slerp(wanted, weight.clamp(0.0, 1.0)).normalize();
    // The roll about the forearm's line, given to the forearm.
    let turn = (target * now.inverse()).normalize();
    let along = Vec3::new(turn.x, turn.y, turn.z).dot(forearm);
    let twist = Quat::from_xyzw(forearm.x * along, forearm.y * along, forearm.z * along, turn.w).normalize();
    let (axis, angle) = twist.to_axis_angle();
    let angle = if angle > std::f32::consts::PI { angle - std::f32::consts::TAU } else { angle };
    let twist = Quat::from_axis_angle(axis, angle.clamp(-MOST_FOREARM_TWIST, MOST_FOREARM_TWIST));
    pose.rotations[chain.elbow] = delta_after_world_turn(pose, rig, chain.elbow, twist);
    // The rest at the wrist.
    let rest = (target * carried(pose).inverse()).normalize();
    pose.rotations[chain.wrist] = delta_after_world_turn(pose, rig, chain.wrist, rest);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gltf_rig;

    /// A pose with the elbow off full extension, so the solver has a solution
    /// space to work in.
    ///
    /// The rest arm is straight to within 0.2 mm (see the module doc), which
    /// is the same singularity `stance` exists to get the legs out of. Tests
    /// that need a bendable arm start here.
    fn arms_bent() -> LocalPose {
        let mut pose = LocalPose::REST;
        for chain in [ArmChain::LEFT, ArmChain::RIGHT] {
            // About the real bend axis, NOT `+X`. `+X` is the arm's own long
            // axis on this rig, so bending about it moves the wrist 0.0058 m
            // and leaves the arm just as straight as it started — which put
            // every test using this helper straight back into the singularity
            // it exists to escape. See `ArmIkConfig::default`.
            pose.set_rotation(
                chain.elbow,
                Quat::from_axis_angle(ArmIkConfig::default().elbow_axis, -0.5),
            );
        }
        pose
    }

    #[test]
    fn the_arm_bone_names_are_shifted_one_joint() {
        // The trap this module's doc opens with, pinned against the real rig.
        // `offsets[LeftArm]` is NOT the upper arm — it is the clavicle stub —
        // and a future rig edit that changes this must break a test rather
        // than silently shorten every arm by 20%.
        let rig = gltf_rig::puppet_base();

        let clavicle_stub = rig.offsets[Bone::LeftArm].length();
        let upper_arm = rig.offsets[Bone::LeftForeArm].length();
        let forearm = rig.offsets[Bone::LeftHand].length();

        assert!(
            (clavicle_stub - 0.2097).abs() < 1.0e-3,
            "shoulder-joint offset moved: {clavicle_stub}",
        );
        assert!(
            (upper_arm - 0.2511).abs() < 1.0e-3,
            "upper arm (stored on LeftForeArm) moved: {upper_arm}",
        );
        assert!(
            (forearm - 0.2436).abs() < 1.0e-3,
            "forearm (stored on LeftHand) moved: {forearm}",
        );

        // And the thing that makes the misreading dangerous rather than
        // merely confusing: the wrong bone is a materially different length.
        assert!(
            (upper_arm - clavicle_stub).abs() > 0.03,
            "if these were similar the misreading would be harmless, and this \
             test would not be worth having",
        );
    }

    #[test]
    fn the_rest_arm_is_critically_extended() {
        // Why `elbow_axis` must be supplied rather than derived. If a rig
        // change ever gives the arm real slack, this fails and the
        // supplied-axis machinery can be revisited.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);

        let upper = (rest[Bone::LeftForeArm] - rest[Bone::LeftArm]).length();
        let fore = (rest[Bone::LeftHand] - rest[Bone::LeftForeArm]).length();
        let span = (rest[Bone::LeftHand] - rest[Bone::LeftArm]).length();

        let slack = upper + fore - span;
        assert!(
            slack < 0.002,
            "the rest arm is supposed to be critically straight; slack is now \
             {slack} m, which would change the axis-stability argument",
        );
    }

    #[test]
    fn the_configured_elbow_axis_actually_bends_the_elbow() {
        // The `+Y` trap: an axis along the arm's own length rotates the
        // forearm about itself and moves the wrist nowhere, so a solver
        // configured with it fails silently. This asserts the default is not
        // that axis.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);

        let mut pose = LocalPose::REST;
        pose.set_rotation(
            ArmChain::LEFT.elbow,
            Quat::from_axis_angle(ArmIkConfig::default().elbow_axis, 0.785),
        );
        let bent = forward_kinematics_on(&pose, &rig);

        let moved = (bent[Bone::LeftHand] - rest[Bone::LeftHand]).length();
        assert!(
            moved > 0.1,
            "rotating the elbow about the configured axis moved the wrist only \
             {moved} m -- the axis is along the arm, not across it",
        );

        // And the trap itself, so the reason the default is what it is stays
        // visible: the arm's own long axis is +X on this rig, and a delta about
        // it does essentially nothing.
        let mut along = LocalPose::REST;
        along.set_rotation(ArmChain::LEFT.elbow, Quat::from_axis_angle(Vec3::X, 0.785));
        let twisted = forward_kinematics_on(&along, &rig);
        let twist_moved = (twisted[Bone::LeftHand] - rest[Bone::LeftHand]).length();

        assert!(
            twist_moved < 0.05,
            "+X was supposed to be this rig's along-the-arm axis, but it moved \
             the wrist {twist_moved} m -- if the rig changed, re-measure \
             `elbow_axis` rather than assuming it still holds",
        );
    }

    #[test]
    fn the_elbow_bends_forward_not_backward() {
        // The direction nobody asserted on the legs, where an inverted axis
        // comment ran a whole walk cycle backwards. An elbow brings the hand
        // toward the front of the body (-Z), never behind it.
        let rig = gltf_rig::puppet_base();

        // A target the arm can reach only by bending: straight down from the
        // shoulder, well inside full extension.
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let shoulder = rest[Bone::LeftArm];
        let target = shoulder + Vec3::new(0.15, -0.20, 0.0);

        let mut pose = arms_bent();
        solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let solved = forward_kinematics_on(&pose, &rig);
        let elbow_offset = solved[Bone::LeftForeArm] - shoulder;
        let straight_line = (target - shoulder).normalize();

        // The elbow sits off the shoulder-to-target line. Which side it sits
        // on is the whole question: in front (-Z) is an elbow, behind (+Z) is
        // a broken joint.
        let sideways = elbow_offset - straight_line * elbow_offset.dot(straight_line);
        assert!(
            sideways.z < 0.0,
            "the elbow must bend toward the front of the body, but it went to \
             z={:+.4} (offset {:?})",
            sideways.z,
            sideways,
        );
    }

    #[test]
    fn solving_for_the_poses_own_wrist_position_changes_nothing() {
        // IK is a modification, not a replacement. Asking for where the arm
        // already is must be a no-op, or a held grip would drift every frame.
        let rig = gltf_rig::puppet_base();
        let base = arms_bent();
        let target = forward_kinematics_on(&base, &rig)[ArmChain::LEFT.wrist];

        let mut pose = base;
        solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let moved = (forward_kinematics_on(&pose, &rig)[ArmChain::LEFT.wrist] - target).length();
        assert!(
            moved < 1.0e-3,
            "solving for the current position moved the wrist {moved} m",
        );
    }

    #[test]
    fn a_reachable_target_is_hit() {
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let shoulder = rest[Bone::LeftArm];

        // Comfortably inside the 0.4947 m reach, and off-axis so it needs both
        // joints to move.
        let target = shoulder + Vec3::new(0.20, -0.15, -0.10);

        let mut pose = arms_bent();
        let landed = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let error = (landed - target).length();
        assert!(error < 1.0e-3, "wrist landed {error} m from a reachable target");
    }

    #[test]
    fn an_unreachable_target_strains_toward_it_without_exceeding_reach() {
        // The anti-hyperextension property. A target at twice the arm's reach
        // must pull the arm straight toward it and stop, never past it.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let shoulder = rest[Bone::LeftArm];

        let upper = rig.offsets[Bone::LeftForeArm].length();
        let fore = rig.offsets[Bone::LeftHand].length();
        let reach = upper + fore;

        let target = shoulder + Vec3::new(0.0, 0.0, -2.0 * reach);

        let mut pose = arms_bent();
        let landed = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let extension = (landed - shoulder).length();
        assert!(
            extension <= reach + 1.0e-4,
            "the arm extended {extension} m, past its {reach} m reach",
        );

        // And it must genuinely strain -- an arm that just stays put also
        // satisfies the bound above.
        assert!(
            extension > reach * 0.9,
            "the arm should reach nearly all the way out, but only got to \
             {extension} m of {reach} m",
        );

        // Segment lengths survive. A rotation cannot stretch a bone by
        // construction, so this is a guard against a future translation
        // channel rather than a live risk.
        let solved = forward_kinematics_on(&pose, &rig);
        let solved_upper = (solved[Bone::LeftForeArm] - solved[Bone::LeftArm]).length();
        let solved_fore = (solved[Bone::LeftHand] - solved[Bone::LeftForeArm]).length();
        assert!((solved_upper - upper).abs() < 1.0e-4, "upper arm stretched");
        assert!((solved_fore - fore).abs() < 1.0e-4, "forearm stretched");
    }

    #[test]
    fn solving_one_arm_leaves_the_other_alone() {
        // The reason the clavicle is a fixed pivot. If the chain reached up
        // into `Spine2` to aim the upper arm, placing one hand would swing
        // the other arm, the neck and the head.
        let rig = gltf_rig::puppet_base();
        let base = arms_bent();
        let before = forward_kinematics_on(&base, &rig);

        let shoulder = before[Bone::LeftArm];
        let mut pose = base;
        solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(shoulder + Vec3::new(0.10, -0.25, -0.15)),
            &ArmIkConfig::default(),
            &rig,
        );

        let after = forward_kinematics_on(&pose, &rig);
        for bone in [
            Bone::RightArm,
            Bone::RightForeArm,
            Bone::RightHand,
            Bone::Head,
            Bone::Neck,
            Bone::Spine2,
            Bone::Hips,
        ] {
            let moved = (after[bone] - before[bone]).length();
            assert!(
                moved < 1.0e-5,
                "solving the left arm moved {} by {moved} m",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_grip_orients_the_hand_in_world_space() {
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let shoulder = rest[Bone::LeftArm];
        let target = shoulder + Vec3::new(0.18, -0.12, -0.12);

        // An arbitrary, deliberately not-axis-aligned orientation, so a
        // partly-correct frame conversion cannot pass by coincidence.
        let wanted = Quat::from_axis_angle(Vec3::new(0.3, 0.8, 0.5).normalize(), 0.9);

        let mut pose = arms_bent();
        let config = ArmIkConfig { aim_hand: true, ..Default::default() };
        solve_arm_on(&mut pose, ArmChain::LEFT, ArmTarget::grip(target, wanted), &config, &rig);

        let world = super::super::rig::accumulate_world_rotations(&pose, &rig);
        let landed = world[Bone::LeftHand];

        // Quaternions double-cover rotations, so compare the rotation rather
        // than the components.
        let difference = (landed.inverse() * wanted).normalize();
        let angle = 2.0 * difference.w.abs().clamp(0.0, 1.0).acos();
        assert!(
            angle < 1.0e-3,
            "the hand ended {angle} rad from the requested grip orientation",
        );
    }

    #[test]
    fn a_grip_does_not_disturb_the_wrist_position_it_was_solved_to() {
        // Orientation is applied after placement, so it must not undo it.
        // This is why `set_hand_rotation` writes the HAND and not the forearm.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let target = rest[Bone::LeftArm] + Vec3::new(0.18, -0.12, -0.12);
        let wanted = Quat::from_axis_angle(Vec3::new(0.3, 0.8, 0.5).normalize(), 0.9);

        let mut reaching = arms_bent();
        let plain = solve_arm_on(
            &mut reaching,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let mut gripping = arms_bent();
        let gripped = solve_arm_on(
            &mut gripping,
            ArmChain::LEFT,
            ArmTarget::grip(target, wanted),
            &ArmIkConfig { aim_hand: true, ..Default::default() },
            &rig,
        );

        let drift = (gripped - plain).length();
        assert!(drift < 1.0e-4, "orienting the hand moved the wrist {drift} m");
    }

    #[test]
    fn aim_hand_off_leaves_the_authored_hand_rotation_alone() {
        // The config flag has to actually gate. A grip target with the flag
        // off must reach and nothing more.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let target = rest[Bone::LeftArm] + Vec3::new(0.18, -0.12, -0.12);

        let authored = Quat::from_axis_angle(Vec3::Z, 0.4);
        let mut pose = arms_bent();
        pose.set_rotation(Bone::LeftHand, authored);

        solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::grip(target, Quat::from_axis_angle(Vec3::Y, 1.2)),
            &ArmIkConfig::default(),
            &rig,
        );

        let kept = pose.rotation(Bone::LeftHand);
        let difference = (kept.inverse() * authored).normalize();
        let angle = 2.0 * difference.w.abs().clamp(0.0, 1.0).acos();
        assert!(
            angle < 1.0e-5,
            "with aim_hand off the hand rotation should be untouched, but it \
             moved {angle} rad",
        );
    }

    #[test]
    fn both_arms_solve_mirrored_targets_to_mirrored_results() {
        // The side-independence of `elbow_axis`. A single axis value has to
        // work for both arms -- if it needed negating per side, this fails.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);

        let left_target = rest[Bone::LeftArm] + Vec3::new(0.15, -0.20, -0.10);
        let right_target = rest[Bone::RightArm] + Vec3::new(-0.15, -0.20, -0.10);

        let mut pose = arms_bent();
        let left = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(left_target),
            &ArmIkConfig::default(),
            &rig,
        );
        let right = solve_arm_on(
            &mut pose,
            ArmChain::RIGHT,
            ArmTarget::reach(right_target),
            &ArmIkConfig::default(),
            &rig,
        );

        assert!((left.x + right.x).abs() < 1.0e-3, "x should mirror: {left:?} vs {right:?}");
        assert!((left.y - right.y).abs() < 1.0e-3, "y should match: {left:?} vs {right:?}");
        assert!((left.z - right.z).abs() < 1.0e-3, "z should match: {left:?} vs {right:?}");

        // And both elbows bend the same way relative to their own arm.
        let solved = forward_kinematics_on(&pose, &rig);
        let left_elbow = solved[Bone::LeftForeArm];
        let right_elbow = solved[Bone::RightForeArm];
        assert!(
            (left_elbow.z - right_elbow.z).abs() < 1.0e-3,
            "both elbows should bend to the same depth: {} vs {}",
            left_elbow.z,
            right_elbow.z,
        );
    }

    #[test]
    fn a_degenerate_config_is_a_no_op_rather_than_a_nan() {
        // A zero axis cannot define a hinge. Returning the current position
        // is the honest answer; producing NaN would poison the whole pose and
        // every bone downstream of it.
        let rig = gltf_rig::puppet_base();
        let base = arms_bent();
        let expected = forward_kinematics_on(&base, &rig)[ArmChain::LEFT.wrist];

        let mut pose = base;
        let landed = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(Vec3::new(0.3, 1.2, -0.2)),
            &ArmIkConfig { elbow_axis: Vec3::ZERO, ..Default::default() },
            &rig,
        );

        assert!(landed.is_finite(), "degenerate axis produced {landed:?}");
        assert!((landed - expected).length() < 1.0e-6, "it should have been a no-op");
    }

    #[test]
    fn a_target_at_the_shoulder_is_a_no_op_rather_than_a_nan() {
        // Zero distance would divide by zero deriving the aim direction.
        let rig = gltf_rig::puppet_base();
        let base = arms_bent();
        let shoulder = forward_kinematics_on(&base, &rig)[ArmChain::LEFT.shoulder];

        let mut pose = base;
        let landed = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(shoulder),
            &ArmIkConfig::default(),
            &rig,
        );

        assert!(landed.is_finite(), "a target at the shoulder produced {landed:?}");
    }

    #[test]
    fn wrist_position_agrees_with_what_the_solver_reports() {
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let target = rest[Bone::LeftArm] + Vec3::new(0.15, -0.18, -0.08);

        let mut pose = arms_bent();
        let reported = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let read_back = wrist_position(&pose, ArmChain::LEFT, &rig);
        assert!((reported - read_back).length() < 1.0e-6);
    }

    #[test]
    fn aiming_a_bone_with_a_large_bind_rotation_is_exact() {
        // The bug this module surfaced in shared code. `aim_bone` conjugated
        // its world-space delta by the ancestors' accumulated rotation alone,
        // omitting the bone's OWN bind rotation -- but forward kinematics
        // composes `... * bind_rotations[bone] * pose.rotations[bone]`, so the
        // pose rotation sits inside the bind.
        //
        // On the legs the binds are within a few degrees of identity and the
        // omission is invisible, which is why it shipped. `LeftArm` binds at
        // 92.6 degrees: asked to point the elbow straight DOWN, the arm swung
        // UP, landing 0.417 m away on a 0.251 m bone -- worse than not moving.
        //
        // The target here is exactly one bone-length from the shoulder, so a
        // correct aim is achievable by rotation alone and the tolerance can be
        // tight enough to leave no room for a partly-right frame.
        let rig = gltf_rig::puppet_base();
        let base = arms_bent();
        let before = forward_kinematics_on(&base, &rig);

        let shoulder = before[Bone::LeftArm];
        let upper = (before[Bone::LeftForeArm] - shoulder).length();

        // Several directions, because the `+X` case is where the arm already
        // points and would pass under any frame convention at all.
        for direction in [
            Vec3::NEG_Y,
            Vec3::NEG_Z,
            Vec3::new(0.5, -0.5, -0.5).normalize(),
            Vec3::new(-0.3, -0.8, 0.5).normalize(),
        ] {
            let target = shoulder + direction * upper;

            let mut pose = base;
            aim_bone(&mut pose, &before, Bone::LeftArm, Bone::LeftForeArm, target, &rig);

            let after = forward_kinematics_on(&pose, &rig);
            let error = (after[Bone::LeftForeArm] - target).length();
            assert!(
                error < 1.0e-5,
                "aiming toward {direction:?} left the elbow {error} m from an \
                 exactly-reachable target",
            );

            // And the bone it pivots from must not have moved.
            assert!(
                (after[Bone::LeftArm] - shoulder).length() < 1.0e-6,
                "the shoulder itself moved",
            );
        }
    }

    #[test]
    fn this_rigs_binds_are_large_enough_that_the_convention_matters() {
        // Keeps the test above honest. Every frame question in this module is
        // invisible on a rig whose bind rotations are near identity — the whole
        // synthetic rig is like that, which is why a wrong convention survived
        // in the shipped code for months while every synthetic test passed.
        //
        // So: assert that the real rig's arm and foot really are bound far from
        // identity, and would therefore expose a convention error.
        let rig = gltf_rig::puppet_base();
        let bind = super::super::rig::accumulate_bind_rotations(&rig);

        for bone in [Bone::LeftArm, Bone::LeftFoot] {
            let angle = 2.0 * bind[bone].normalize().w.abs().clamp(0.0, 1.0).acos();
            assert!(
                angle > 0.5,
                "{} is bound only {angle} rad from identity, so this rig can no \
                 longer discriminate between world-axis and local-axis deltas \
                 and the tests above have gone vacuous",
                bone.name(),
            );
        }
    }

    #[test]
    fn the_leg_solver_is_more_accurate_after_the_shared_frame_fix() {
        // `aim_bone` is shared with the leg, so fixing it had to be checked
        // against the leg rather than assumed harmless. The real rig's ankle
        // binds at -69.8 degrees, so the bind term is NOT negligible there
        // either -- the leg was simply accurate enough for its own tolerances
        // to pass.
        //
        // This asserts the leg hits a reachable toe target tightly, which is
        // the property the fix must not have broken.
        use crate::character::anim::legik::{solve_leg_on, LegChain, LegIkConfig};
        use crate::character::anim::stance::stance;

        let rig = gltf_rig::puppet_base();
        let base = stance(&LocalPose::REST);
        let before = forward_kinematics_on(&base, &rig);

        for chain in [LegChain::LEFT, LegChain::RIGHT] {
            let target = before[chain.toe] + Vec3::new(0.02, 0.03, -0.04);

            let mut pose = base;
            let landed = solve_leg_on(&mut pose, chain, target, &LegIkConfig::default(), &rig);

            let error = (landed - target).length();
            assert!(
                error < 5.0e-4,
                "the leg landed {error} m from a reachable toe target",
            );
        }
    }

    #[test]
    fn the_elbow_lands_where_the_two_bone_solution_put_it() {
        // The solve computes an elbow position and a wrist position, then aims
        // two bones to realise them. If only the wrist is checked, a chain that
        // reaches the right point with a wrong-looking elbow passes -- and the
        // elbow is the part a viewer reads as "that arm is broken".
        let rig = gltf_rig::puppet_base();
        let before = forward_kinematics_on(&arms_bent(), &rig);
        let shoulder = before[Bone::LeftArm];
        let target = shoulder + Vec3::new(0.20, -0.15, -0.10);

        // Recompute the solution independently, the way the solver does.
        let upper = (before[Bone::LeftForeArm] - shoulder).length();
        let fore = (before[Bone::LeftHand] - before[Bone::LeftForeArm]).length();
        let to_target = target - shoulder;
        let distance = to_target.length();
        let solution = solve_two_bone(upper, fore, distance, ArmIkConfig::default().softening);
        let direction = to_target / distance;

        // Perpendicularized, exactly as the solver does — see its own comment.
        let requested = ArmIkConfig::default().elbow_axis;
        let hinge = (requested - direction * requested.dot(direction)).normalize();

        let expected_elbow =
            shoulder + (Quat::from_axis_angle(hinge, solution.upper_angle) * direction) * upper;

        let mut pose = arms_bent();
        solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let solved = forward_kinematics_on(&pose, &rig);
        let error = (solved[Bone::LeftForeArm] - expected_elbow).length();
        assert!(
            error < 1.0e-4,
            "the elbow ended {error} m from where the two-bone solution placed \
             it ({:?} vs {expected_elbow:?})",
            solved[Bone::LeftForeArm],
        );
    }

    #[test]
    fn the_default_softening_does_not_shrink_a_reachable_target() {
        // The softening zone applies to targets inside reach as well as beyond
        // it, and the rest arm sits only 0.0118 m off full extension — so a
        // generous zone is active during ordinary reaching, not just straining.
        //
        // This is what forced the default down from 0.02 to the leg's 0.005: at
        // 0.02 a target at the arm's own wrist came back 0.0015 m short, which a
        // per-frame grip would compound into a visible inward creep.
        let rig = gltf_rig::puppet_base();
        let base = arms_bent();
        let target = forward_kinematics_on(&base, &rig)[Bone::LeftHand];

        let mut pose = base;
        let landed = solve_arm_on(
            &mut pose,
            ArmChain::LEFT,
            ArmTarget::reach(target),
            &ArmIkConfig::default(),
            &rig,
        );

        let short_by = (landed - target).length();
        assert!(
            short_by < 1.0e-5,
            "the default softening pulled a reachable target {short_by} m short",
        );

        // And the mechanism, so a future default change fails here with a clear
        // reason rather than in a distant drift test.
        let wide = ArmIkConfig { softening: 0.02, ..Default::default() };
        let mut pose = base;
        let with_wide =
            solve_arm_on(&mut pose, ArmChain::LEFT, ArmTarget::reach(target), &wide, &rig);
        // At THIS bend the wide zone barely bites — the target sits 0.47535 m
        // out on 0.49471 m of reach, 0.0194 m of slack against a 0.02 m zone,
        // and the wide solve comes back only 0.000001 m short. So the guard has
        // to be stated somewhere the zone is genuinely active: a nearly-straight
        // arm, which is where the rest pose actually sits (0.2 mm of slack) and
        // therefore where the default's value was decided.
        let _ = with_wide;

        let straight = LocalPose::REST;
        let straight_target = forward_kinematics_on(&straight, &rig)[Bone::LeftHand];

        let mut pose = straight;
        let narrow_landed =
            solve_arm_on(&mut pose, ArmChain::LEFT, ArmTarget::reach(straight_target), &ArmIkConfig::default(), &rig);

        let mut pose = straight;
        let wide_landed =
            solve_arm_on(&mut pose, ArmChain::LEFT, ArmTarget::reach(straight_target), &wide, &rig);

        let narrow_short = (narrow_landed - straight_target).length();
        let wide_short = (wide_landed - straight_target).length();

        assert!(
            wide_short > narrow_short + 1.0e-4,
            "a 0.02 m softening zone is supposed to pull a near-straight arm \
             visibly shorter than the 0.005 m default does, but it came back \
             {wide_short} m against {narrow_short} m -- if that is no longer \
             true the default can be widened again",
        );
    }

    #[test]
    fn solving_is_idempotent() {
        // Running the solver twice on the same target must not drift. A
        // per-frame grip does exactly this, so drift here is a slow slide out
        // of position in the game.
        let rig = gltf_rig::puppet_base();
        let rest = forward_kinematics_on(&LocalPose::REST, &rig);
        let target = rest[Bone::LeftArm] + Vec3::new(0.15, -0.18, -0.08);
        let config = ArmIkConfig::default();

        let mut pose = arms_bent();
        let first = solve_arm_on(&mut pose, ArmChain::LEFT, ArmTarget::reach(target), &config, &rig);
        let second =
            solve_arm_on(&mut pose, ArmChain::LEFT, ArmTarget::reach(target), &config, &rig);

        assert!(
            (first - second).length() < 1.0e-5,
            "re-solving moved the wrist from {first:?} to {second:?}",
        );
    }
}
