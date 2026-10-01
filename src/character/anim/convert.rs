//! Converting position-authored poses into rotation space.
//!
//! The superseded `muscle::pose` module authors a pose as a sparse map of
//! **absolute world-space offsets** from each bone's T-pose position. Those
//! numbers are valuable: the current `relaxed_stand` and `idle_loop` were
//! derived from real Mixamo `idle.glb` reference data (see
//! `tools/dump_animation_pose.py`), replacing earlier hand-guessed values
//! that had repeatedly shipped wrong. Re-authoring them by eye in a new
//! representation would throw that provenance away.
//!
//! So this module converts them instead — same source data, no guessing.
//!
//! # Why the conversion is not simply mechanical
//!
//! Given target world positions for every bone, the local rotation of each
//! bone follows from the direction to its child... except where a bone has
//! **more than one** child. In this rig `Hips` has three (`Spine`,
//! `LeftUpLeg`, `RightUpLeg`) and `Spine2` has three (`Neck`,
//! `LeftShoulder`, `RightShoulder`), and `relaxed_stand` moves all three of
//! `Spine2`'s. Three children want three different parent rotations, and
//! only one can be honoured: the problem is genuinely underdetermined, not
//! merely fiddly.
//!
//! Averaging the candidates (e.g. slerping them) is the tempting answer and
//! a known-bad one — it satisfies *none* of the children exactly and reads
//! as jitter whenever they disagree. The rig already carries the standard
//! alternative in [`Bone::chain_continuation_child`]: one designated child
//! (`Spine` for `Hips`, `Neck` for `Spine2`) determines the shared parent's
//! rotation, and every lateral attachment absorbs its own deviation in its
//! own local rotation instead of fighting for the parent's.
//!
//! Concretely, that means a lateral attachment's own **position** is not
//! preserved. A bone's position is set by its *parent's* rotation; its own
//! rotation moves only its children. So once `Spine2` follows `Neck`, both
//! shoulders sit wherever that puts them, and no shoulder rotation can move
//! them elsewhere. What survives is everything below: the arm chain absorbs
//! the deviation in its own rotations, so hands still land where authored.
//!
//! For `relaxed_stand` this displacement is a few centimetres at the
//! shoulder — small, and invisible once the arm chain re-aims beneath it.
//! It is nonetheless a real difference from the source data, which is why
//! [`conversion_error`] exists and why the Phase 2 acceptance bar is a
//! measured tolerance rather than an assumption.
//!
//! # What a converted pose does and does not preserve
//!
//! Preserved exactly: every bone's **direction** from its parent, hence
//! every child's position, to within the rig's own bone lengths.
//!
//! Not preserved: any authored position that violates a bone length. The
//! position representation can express "hand 20 cm further from the elbow
//! than the forearm is long"; a rotation cannot, and will place the hand at
//! the correct *direction* and the rig's own *distance*. This is a feature
//! — that class of stretch was a recurring bug — but it means a converted
//! pose can differ from its source wherever the source was geometrically
//! impossible. [`conversion_error`] reports exactly where and by how much,
//! so the difference is measured rather than assumed.

use bevy::math::{Quat, Vec3};

use super::rig::{forward_kinematics, BoneSet, LocalPose};
use crate::character::skeleton::Bone;

/// Target world positions for every bone, as the position-space
/// representation understands them.
pub type WorldPositions = BoneSet<Vec3>;

/// Which child determines `bone`'s own rotation, if any.
///
/// For a single-child bone that child is unambiguous. For a multi-child
/// bone it is [`Bone::chain_continuation_child`]; see the module doc for
/// why averaging is the wrong answer. Leaf bones have no child and so no
/// rotation of their own to derive — they inherit their parent's frame.
fn rotation_driving_child(bone: Bone) -> Option<Bone> {
    if let Some(continuation) = bone.chain_continuation_child() {
        return Some(continuation);
    }

    let mut only_child = None;
    for &candidate in Bone::ALL.iter() {
        if candidate.parent() == Some(bone) {
            if only_child.is_some() {
                // More than one child and no continuation rule: the rig's
                // own tables are inconsistent. Better to leave the bone
                // unrotated than to silently pick one arbitrarily.
                return None;
            }
            only_child = Some(candidate);
        }
    }
    only_child
}

/// Converts target world positions into a rotation-space [`LocalPose`],
/// folding in an authored twist per bone.
///
/// # Why twist needs its own input
///
/// A target *position* cannot express roll around a bone's own axis: a
/// forearm pointing the same way with the palm up or palm down has
/// identical joint positions. The position-space representation therefore
/// carries a separate `twists` map, and reading only positions silently
/// discards it — `wave_pose`'s 90-degree forearm and hand roll, for one.
///
/// Rotation space needs no such side channel, because a quaternion already
/// carries roll. This function is the bridge: it folds the authored twist
/// into the same quaternion as the swing, after which the two are
/// indistinguishable and the separate channel disappears.
///
/// `twist_radians` is queried per bone; return `0.0` for bones with no
/// authored roll.
pub fn pose_from_world_positions_with_twist(
    targets: &WorldPositions,
    twist_radians: impl Fn(Bone) -> f32,
) -> LocalPose {
    let mut pose = pose_from_world_positions(targets);

    for &bone in Bone::ALL.iter() {
        let twist = twist_radians(bone);
        if twist == 0.0 {
            continue;
        }

        // Roll happens about the bone's OWN axis — the direction it points
        // its driving child in the rest pose. Composing it after the swing
        // (rather than before) means it turns about the already-aimed
        // direction, which is what "palm up" means physically.
        //
        // This mirrors the superseded module's own rule, which applied
        // twist strictly after the swing solve specifically so it could
        // never fight it. Here the ordering is the only thing that carries
        // over; there is no second simulated channel to fight with.
        let Some(child) = rotation_driving_child(bone) else { continue };
        let axis = child.t_pose_offset().normalize_or_zero();
        if axis == Vec3::ZERO {
            continue;
        }

        let swing = pose.rotation(bone);
        pose.set_rotation(bone, swing * Quat::from_axis_angle(axis, twist));
    }

    pose
}

/// Converts target world positions into a rotation-space [`LocalPose`].
///
/// Walks parent-before-child, so each bone's rotation is derived in the
/// frame its already-converted ancestors establish — the same single-pass
/// structure [`forward_kinematics`] uses, and the reason
/// [`Bone::ALL`]'s ordering is pinned by a test.
///
/// # Chasing directions, not positions
///
/// Each bone is aimed so the **direction** to its driving child matches the
/// authored one. It deliberately does *not* try to place the child at the
/// authored position, because for a lateral attachment that is impossible:
/// its position is fixed by its parent's rotation (see the module doc).
///
/// This distinction is worth a concrete number. The real `relaxed_stand`
/// authors a **13.7 cm** absolute displacement of each shoulder — more than
/// the shoulder bone is long. Nothing a rotation-space rig does can move a
/// shoulder that far, so a position-chasing conversion leaves ~11.5 cm of
/// residual on every bone of both arms (measured). Aiming by direction
/// instead reproduces the visible *shape* of the pose — which is what the
/// authored positions were describing — and lets each chain hang correctly
/// beneath wherever its attachment really sits.
pub fn pose_from_world_positions(targets: &WorldPositions) -> LocalPose {
    pose_from_world_positions_against(targets, &BoneSet::from_fn(|bone| bone.t_pose_world_position()))
}

/// [`pose_from_world_positions`], measured from `rest`: the source rig's
/// own bind pose, as world positions in the targets' frame.
///
/// # Why the source's bind, not this crate's T-pose
///
/// A pose here is a bend away from a rig's bind (the renderer applies it to
/// whatever shape each rig was bound in). Measured from the straight
/// synthetic T-pose, a clip's positions store the source rig's own bind
/// shape as part of the bend. Mixamo's bind spine leans back 0.3, 14.1 and
/// 12.2 degrees in its segments; converted that way, `relaxed_stand` bent
/// the target's spine by that much on top of the target's own bind curve,
/// and puppet_base stood with its chest thrown 23.6 degrees back. Measured
/// from the source's bind, a pose keeps only what the actor did: the idle's
/// spine bends −4.1, +0.8 and +1.6 degrees.
pub fn pose_from_world_positions_against(targets: &WorldPositions, rest: &WorldPositions) -> LocalPose {
    let mut pose = LocalPose::REST;

    // Each bone's accumulated rest-relative rotation, built as we go. We
    // cannot call `accumulate_rest_relative_rotations` because the pose is
    // still being constructed.
    let mut accumulated = BoneSet::splat(Quat::IDENTITY);

    for &bone in Bone::ALL.iter() {
        let Some(child) = rotation_driving_child(bone) else {
            // A leaf, or an ambiguous multi-child bone: no rotation to
            // derive. Its own frame stays whatever its parent established.
            accumulated[bone] = match bone.parent() {
                Some(parent) => accumulated[parent],
                None => Quat::IDENTITY,
            };
            continue;
        };

        // Where the rest pose points this bone at its driving child, and
        // where the authored positions want it to point — both expressed in
        // this bone's own parent frame, so the rotation we derive is local.
        let parent_frame = match bone.parent() {
            Some(parent) => accumulated[parent],
            None => Quat::IDENTITY,
        };

        let rest_direction = (rest[child] - rest[bone]).normalize_or_zero();
        let target_direction =
            (targets[child] - targets[bone]).normalize_or_zero();

        let local_rotation = if rest_direction == Vec3::ZERO
            || target_direction == Vec3::ZERO
        {
            Quat::IDENTITY
        } else {
            // Undo the parent frame so the arc is measured locally.
            let target_in_parent_frame = parent_frame.inverse() * target_direction;
            shortest_arc(rest_direction, target_in_parent_frame)
        };

        pose.set_rotation(bone, local_rotation);
        accumulated[bone] = parent_frame * local_rotation;
    }

    pose.root_translation = targets[Bone::Hips] - rest[Bone::Hips];
    pose
}

/// The shortest rotation taking `from` to `to`, with an explicit guard for
/// the antipodal case.
///
/// `Quat::from_rotation_arc` is documented to be unstable when the inputs
/// are near-opposite, and this project has already been bitten by it once
/// (a solved direction at `dot ~= -0.99` landing in the degenerate branch).
/// At 180 degrees the rotation axis is genuinely arbitrary — any axis
/// perpendicular to `from` is a valid answer — so rather than let the
/// library pick unpredictably we choose a stable perpendicular explicitly.
pub fn shortest_arc(from: Vec3, to: Vec3) -> Quat {
    let dot = from.dot(to).clamp(-1.0, 1.0);

    if dot < -0.9995 {
        // Near-antipodal: pick any axis perpendicular to `from`, preferring
        // whichever cardinal axis it is least aligned with so the cross
        // product stays well-conditioned.
        let fallback_axis = if from.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let axis = from.cross(fallback_axis).normalize_or_zero();
        let axis = if axis == Vec3::ZERO { Vec3::Y } else { axis };
        return Quat::from_axis_angle(axis, std::f32::consts::PI);
    }

    Quat::from_rotation_arc(from, to)
}

/// How far a converted pose lands from the positions it was built from,
/// per bone, in metres.
///
/// Non-zero entries are not necessarily errors: they mark places where the
/// authored positions were not achievable with the rig's real bone lengths,
/// or where a lateral attachment's position is fixed by its parent (see the
/// module doc). Use this to decide whether a conversion is faithful enough
/// to ship or needs hand-authoring, rather than assuming either way.
///
/// For judging whether the *pose* survived, prefer
/// [`direction_error_degrees`] — position error accumulates down a chain,
/// so one displaced attachment makes every bone beneath it look wrong even
/// when the limb's shape is reproduced perfectly.
pub fn conversion_error(targets: &WorldPositions, pose: &LocalPose) -> BoneSet<f32> {
    let achieved = forward_kinematics(pose);
    BoneSet::from_fn(|bone| (achieved[bone] - targets[bone]).length())
}

/// Whether any rotation in the rig can control the direction from `bone`'s
/// parent to `bone`.
///
/// That direction is produced by the parent's rotation. A parent can aim at
/// exactly one child — its [`rotation_driving_child`] — so every *other*
/// child of a multi-child parent sits wherever that aim puts it. Their
/// placement is a consequence, not an authorable quantity.
///
/// In this rig the uncontrollable bones are `LeftShoulder`/`RightShoulder`
/// (siblings of `Neck` under `Spine2`) and `LeftUpLeg`/`RightUpLeg`
/// (siblings of `Spine` under `Hips`).
pub fn direction_is_controllable(bone: Bone) -> bool {
    match bone.parent() {
        None => false,
        Some(parent) => rotation_driving_child(parent) == Some(bone),
    }
}

/// The angle, in degrees, between each bone's authored direction-from-parent
/// and the direction the converted pose actually produces.
///
/// This is the fidelity measure that matters for a pose: it is what a viewer
/// sees as the limb's shape, and unlike position error it does not
/// accumulate down a chain — a bone whose attachment had to move but whose
/// own aim is correct reads as `0`.
///
/// Bones whose direction no rotation can control (see
/// [`direction_is_controllable`]) are reported as `0`, because a non-zero
/// reading there measures the authored data's own inconsistency rather than
/// the conversion's fidelity. `relaxed_stand`, for instance, asks
/// `Spine2 -> RightShoulder` to rotate 31 degrees *and* stretch 5.7% —
/// neither of which `Spine2` can supply while aiming at `Neck`. Use
/// [`conversion_error`] to see what those bones actually cost in metres.
///
/// `Hips` has no parent and is reported as `0`.
pub fn direction_error_degrees(
    targets: &WorldPositions,
    pose: &LocalPose,
) -> BoneSet<f32> {
    let achieved = forward_kinematics(pose);

    BoneSet::from_fn(|bone| {
        if !direction_is_controllable(bone) {
            return 0.0;
        }
        let Some(parent) = bone.parent() else { return 0.0 };

        let wanted = (targets[bone] - targets[parent]).normalize_or_zero();
        let got = (achieved[bone] - achieved[parent]).normalize_or_zero();

        if wanted == Vec3::ZERO || got == Vec3::ZERO {
            return 0.0;
        }
        wanted.dot(got).clamp(-1.0, 1.0).acos().to_degrees()
    })
}

/// `pose`, converted against this crate's straight T-pose, rebased for
/// `bones` onto a source rig bound in `rest`: each of those bones now bends
/// from the source's bind direction to where it pointed, as
/// [`pose_from_world_positions_against`] would have converted it. Every
/// other bone keeps its world orientation (its local rotation absorbs the
/// change), so a hand-finished pose's arms, gaze and legs stay as they were.
///
/// For a pose whose source positions are gone or that was finished by hand
/// since (`relaxed_stand`): the bind-relative fix for its spine.
pub fn rebase_onto_bind(pose: &LocalPose, rest: &WorldPositions, bones: &[Bone]) -> LocalPose {
    let old = super::rig::accumulate_rest_relative_rotations(pose);
    let mut world = old;
    for &bone in bones {
        let Some(child) = rotation_driving_child(bone) else { continue };
        let (straight, bound) = (child.t_pose_offset().normalize_or_zero(), (rest[child] - rest[bone]).normalize_or_zero());
        if straight == Vec3::ZERO || bound == Vec3::ZERO {
            continue;
        }
        // Old: straight → posed. New: bound → the same posed direction.
        world[bone] = old[bone] * shortest_arc(bound, straight);
    }
    let mut rebased = *pose;
    for &bone in Bone::ALL.iter() {
        let parent = bone.parent().map_or(Quat::IDENTITY, |parent| world[parent]);
        rebased.set_rotation(bone, (parent.inverse() * world[bone]).normalize());
    }
    rebased
}

/// [`forward_kinematics`] on a rig bound in `rest` (world positions): each
/// bone's rest offset from its parent carried through the parent's
/// accumulated rotation, as the synthetic table's are.
pub fn forward_kinematics_against(pose: &LocalPose, rest: &WorldPositions) -> WorldPositions {
    let accumulated = super::rig::accumulate_rest_relative_rotations(pose);
    let mut positions = BoneSet::splat(Vec3::ZERO);
    for &bone in Bone::ALL.iter() {
        positions[bone] = match bone.parent() {
            Some(parent) => positions[parent] + accumulated[parent] * (rest[bone] - rest[parent]),
            None => rest[bone] + pose.root_translation,
        };
    }
    positions
}

/// [`direction_error_degrees`] for a pose converted against `rest`
/// ([`pose_from_world_positions_against`]): posed on a rig bound in `rest`,
/// how far each bone points from where `targets` has it.
pub fn direction_error_degrees_against(targets: &WorldPositions, rest: &WorldPositions, pose: &LocalPose) -> BoneSet<f32> {
    let achieved = forward_kinematics_against(pose, rest);
    BoneSet::from_fn(|bone| {
        if !direction_is_controllable(bone) {
            return 0.0;
        }
        let Some(parent) = bone.parent() else { return 0.0 };
        let wanted = (targets[bone] - targets[parent]).normalize_or_zero();
        let got = (achieved[bone] - achieved[parent]).normalize_or_zero();
        if wanted == Vec3::ZERO || got == Vec3::ZERO {
            return 0.0;
        }
        wanted.dot(got).clamp(-1.0, 1.0).acos().to_degrees()
    })
}

/// The largest per-bone direction error, and which bone it belongs to.
pub fn worst_direction_error(
    targets: &WorldPositions,
    pose: &LocalPose,
) -> (Bone, f32) {
    let errors = direction_error_degrees(targets, pose);
    let mut worst = (Bone::Hips, 0.0f32);
    for (bone, &error) in errors.iter() {
        if error > worst.1 {
            worst = (bone, error);
        }
    }
    worst
}

/// The largest per-bone conversion error, and which bone it belongs to.
pub fn worst_conversion_error(
    targets: &WorldPositions,
    pose: &LocalPose,
) -> (Bone, f32) {
    let errors = conversion_error(targets, pose);
    let mut worst = (Bone::Hips, 0.0f32);
    for (bone, &error) in errors.iter() {
        if error > worst.1 {
            worst = (bone, error);
        }
    }
    worst
}

/// Mirrors a pose left-to-right across the sagittal plane.
///
/// Used to check that a pose documented as symmetric really is. The rig's
/// convention is +X to the character's right, so mirroring negates the X
/// axis; for a rotation that means negating the `y` and `z` components
/// (the axis components perpendicular to X flip sign together with the
/// handedness of the rotation).
pub fn mirrored(pose: &LocalPose) -> LocalPose {
    let mut mirrored = LocalPose::REST;

    for &bone in Bone::ALL.iter() {
        let source = mirror_bone(bone);
        let q = pose.rotation(source);
        mirrored.set_rotation(bone, Quat::from_xyzw(q.x, -q.y, -q.z, q.w));
    }

    let root = pose.root_translation;
    mirrored.root_translation = Vec3::new(-root.x, root.y, root.z);
    mirrored
}

/// The left/right counterpart of a bone, or the bone itself if it is
/// central.
///
/// Derived from the bone's own name rather than a hand-written table, so it
/// cannot drift out of step with [`Bone::ALL`].
pub fn mirror_bone(bone: Bone) -> Bone {
    let name = bone.name();
    let mirrored_name = if let Some(rest) = name.strip_prefix("Left") {
        format!("Right{rest}")
    } else if let Some(rest) = name.strip_prefix("Right") {
        format!("Left{rest}")
    } else {
        return bone;
    };

    Bone::from_name(&mirrored_name).unwrap_or(bone)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::math::quat_ext::neighborhood;
    use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    /// The world positions the rest pose itself produces.
    fn rest_positions() -> WorldPositions {
        forward_kinematics(&LocalPose::REST)
    }

    #[test]
    fn converting_the_rest_positions_yields_the_rest_pose() {
        // The identity case: positions that are already exactly the T-pose
        // must convert to all-identity rotations, not to something that
        // merely renders the same.
        let pose = pose_from_world_positions(&rest_positions());

        for &bone in Bone::ALL.iter() {
            assert!(
                pose.rotation(bone).abs_diff_eq(Quat::IDENTITY, 1.0e-5),
                "{} should convert to an identity rotation, got {:?}",
                bone.name(),
                pose.rotation(bone),
            );
        }
        assert!(
            pose.root_translation.length() < 1.0e-6,
            "and the root should not move, got {:?}",
            pose.root_translation,
        );
    }

    #[test]
    fn a_converted_pose_reproduces_the_positions_it_was_built_from() {
        // Round trip through a pose that IS achievable by rotation alone:
        // build target positions with forward kinematics, convert them back,
        // and require the original rotations to reappear.
        let mut original = LocalPose::REST;
        original.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.6));
        original.set_rotation(Bone::LeftForeArm, Quat::from_axis_angle(Vec3::Z, -0.4));
        original.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.2));
        original.set_rotation(Bone::RightUpLeg, Quat::from_axis_angle(Vec3::X, 0.5));

        let targets = forward_kinematics(&original);
        let converted = pose_from_world_positions(&targets);

        let (worst_bone, worst) = worst_conversion_error(&targets, &converted);
        assert!(
            worst < 1.0e-4,
            "a rotation-achievable pose must round-trip exactly, but {} was off by {worst} m",
            worst_bone.name(),
        );
    }

    #[test]
    fn conversion_preserves_every_bone_length_even_from_impossible_targets() {
        // The structural guarantee. Feed deliberately unreachable targets —
        // a hand placed a metre beyond the arm's reach — and the result must
        // still be a valid rig: correct direction, rig's own length. This is
        // precisely the stretch that the position representation allowed and
        // that repeatedly shipped as a bug.
        let mut targets = rest_positions();
        targets[Bone::LeftHand] += Vec3::new(-1.0, 0.0, 0.0);
        targets[Bone::RightFoot] += Vec3::new(0.0, -0.8, 0.0);

        let pose = pose_from_world_positions(&targets);
        let achieved = forward_kinematics(&pose);

        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest_length = bone.t_pose_offset().length();
            let achieved_length = (achieved[bone] - achieved[parent]).length();

            assert!(
                (achieved_length - rest_length).abs() < 1.0e-5,
                "{} must keep its rest length {rest_length} even from an impossible \
                 target, got {achieved_length}",
                bone.name(),
            );
        }
    }

    #[test]
    fn an_unreachable_target_still_points_the_bone_in_the_right_direction() {
        // The other half of the previous test: clamping to the rig's own
        // length must not also throw away the authored DIRECTION.
        let mut targets = rest_positions();
        let reach = Vec3::new(-0.5, -0.5, 0.0);
        targets[Bone::LeftHand] = targets[Bone::LeftForeArm] + reach;

        let pose = pose_from_world_positions(&targets);
        let achieved = forward_kinematics(&pose);

        let wanted = reach.normalize();
        let got = (achieved[Bone::LeftHand] - achieved[Bone::LeftForeArm]).normalize();

        assert!(
            got.dot(wanted) > 0.999,
            "the hand should point toward its unreachable target ({wanted:?}), got {got:?}",
        );
    }

    #[test]
    fn a_multi_child_parents_rotation_follows_its_continuation_child() {
        // Pins the resolution of the genuinely-underdetermined case. Move
        // all three of Spine2's children; Spine2 itself must follow Neck
        // (its `chain_continuation_child`), and the shoulders must absorb
        // their own deviation in their OWN rotations rather than dragging
        // the shared parent toward a compromise none of them wanted.
        // Build the targets by ROTATING each child around Spine2, so every
        // one of them stays at its own real bone length. Authoring a target
        // by translating a child (the obvious thing to write) makes it
        // unreachable by rotation and tests bone-length clamping instead of
        // the rule under test — `Neck` translated 5 cm sits 0.1118 m from
        // Spine2 when the bone is 0.10 m.
        let spine2 = rest_positions()[Bone::Spine2];
        let neck_tilt = Quat::from_axis_angle(Vec3::X, 0.25);
        let left_tilt = Quat::from_axis_angle(Vec3::Z, 0.20);
        let right_tilt = Quat::from_axis_angle(Vec3::Z, 0.35);

        let mut targets = rest_positions();
        let rotate_about_spine2 = |q: Quat, p: Vec3| spine2 + q * (p - spine2);

        targets[Bone::Neck] = rotate_about_spine2(neck_tilt, targets[Bone::Neck]);
        targets[Bone::Head] = rotate_about_spine2(neck_tilt, targets[Bone::Head]);
        targets[Bone::LeftShoulder] =
            rotate_about_spine2(left_tilt, targets[Bone::LeftShoulder]);
        targets[Bone::RightShoulder] =
            rotate_about_spine2(right_tilt, targets[Bone::RightShoulder]);

        let pose = pose_from_world_positions(&targets);
        let achieved = forward_kinematics(&pose);

        // The continuation child is honoured exactly...
        assert!(
            (achieved[Bone::Neck] - targets[Bone::Neck]).length() < 1.0e-4,
            "Neck, as Spine2's continuation child, must reach its target exactly, but \
             landed {} m away",
            (achieved[Bone::Neck] - targets[Bone::Neck]).length(),
        );

        // ...and Spine2 really did follow Neck rather than splitting the
        // difference: its rendered direction is exactly the neck tilt.
        let spine2_direction =
            (achieved[Bone::Neck] - achieved[Bone::Spine2]).normalize();
        let expected_direction = (neck_tilt * Vec3::Y).normalize();
        assert!(
            spine2_direction.dot(expected_direction) > 0.999,
            "Spine2 should point along its continuation child's tilt {expected_direction:?}, \
             got {spine2_direction:?}",
        );

        // The lateral attachments, by contrast, do NOT reach their own
        // targets — and cannot, by construction. A bone's POSITION is set by
        // its parent's rotation; its own rotation only moves its children.
        // So once Spine2 follows Neck, both shoulders' positions are fully
        // determined and no shoulder rotation can change them.
        //
        // This is precisely what "three children, one parent rotation, and
        // only one can be honoured" means in practice. Asserting otherwise
        // (an earlier version of this test did) demands something no
        // rotation-space rig can deliver.
        //
        // What IS preserved is everything below the shoulder: the arm chain
        // absorbs the deviation in its own rotations, which is both what the
        // rule promises and where an animator would author it.
        let shoulder_error =
            (achieved[Bone::LeftShoulder] - targets[Bone::LeftShoulder]).length();
        assert!(
            shoulder_error > 1.0e-3,
            "test premise: the lateral attachment should be displaced by following the \
             continuation child, got {shoulder_error} m",
        );
        assert!(
            shoulder_error < 0.06,
            "but the displacement must stay small enough to be absorbed downstream, got \
             {shoulder_error} m",
        );
    }

    #[test]
    fn an_authored_twist_survives_conversion() {
        // Closes the gap where roll was silently dropped: a target position
        // cannot express it, so it arrives on its own channel and has to be
        // folded into the quaternion explicitly.
        let targets = rest_positions();

        let plain = pose_from_world_positions(&targets);
        let twisted =
            pose_from_world_positions_with_twist(&targets, |bone| {
                if bone == Bone::LeftForeArm { FRAC_PI_2 } else { 0.0 }
            });

        assert!(
            plain.rotation(Bone::LeftForeArm).abs_diff_eq(Quat::IDENTITY, 1.0e-6),
            "without twist this bone should be unrotated",
        );

        let angle = twisted.rotation(Bone::LeftForeArm).angle_between(Quat::IDENTITY);
        assert!(
            (angle - FRAC_PI_2).abs() < 1.0e-4,
            "the authored 90-degree twist should appear in the bone's rotation, got {} \
             degrees",
            angle.to_degrees(),
        );
    }

    #[test]
    fn a_twist_rolls_about_the_bones_own_axis_without_moving_it() {
        // The defining property of roll: it changes orientation but not
        // where the bone points, so no child position moves.
        let targets = rest_positions();

        let twisted = pose_from_world_positions_with_twist(&targets, |bone| {
            if bone == Bone::LeftArm { 1.0 } else { 0.0 }
        });
        let positions = forward_kinematics(&twisted);

        // LeftForeArm lies ALONG LeftArm's own axis, so rolling about that
        // axis must leave it exactly where it was.
        assert!(
            (positions[Bone::LeftForeArm] - rest_positions()[Bone::LeftForeArm]).length()
                < 1.0e-5,
            "a pure roll must not move the bone it points at",
        );

        // But the rotation itself is genuinely non-identity.
        assert!(
            !twisted.rotation(Bone::LeftArm).abs_diff_eq(Quat::IDENTITY, 1.0e-4),
            "...while still being a real rotation",
        );
    }

    #[test]
    fn a_zero_twist_is_exactly_the_untwisted_pose() {
        let targets = rest_positions();

        let plain = pose_from_world_positions(&targets);
        let zero_twist = pose_from_world_positions_with_twist(&targets, |_| 0.0);

        for &bone in Bone::ALL.iter() {
            assert_eq!(
                plain.rotation(bone),
                zero_twist.rotation(bone),
                "{} should be bit-identical with no twist authored",
                bone.name(),
            );
        }
    }

    #[test]
    fn shortest_arc_handles_the_antipodal_case_without_degenerating() {
        // `Quat::from_rotation_arc` is documented-unstable here and this
        // project has already been bitten by it. The result must be a valid
        // unit quaternion that genuinely performs the flip.
        for from in [Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(1.0, 2.0, -3.0).normalize()] {
            let arc = shortest_arc(from, -from);

            assert!(arc.is_normalized(), "the antipodal arc must be a unit quaternion");
            assert!(
                (arc * from).dot(-from) > 0.999,
                "the antipodal arc must actually map {from:?} to its opposite, got {:?}",
                arc * from,
            );
        }
    }

    #[test]
    fn shortest_arc_is_exact_for_ordinary_directions() {
        let cases = [
            (Vec3::X, Vec3::Y),
            (Vec3::Y, Vec3::NEG_Z),
            (Vec3::new(0.3, -0.5, 0.8).normalize(), Vec3::new(-0.2, 0.9, 0.1).normalize()),
        ];

        for (from, to) in cases {
            let arc = shortest_arc(from, to);
            assert!(
                (arc * from).dot(to) > 0.9999,
                "shortest_arc({from:?}, {to:?}) should map from onto to, got {:?}",
                arc * from,
            );
        }
    }

    #[test]
    fn shortest_arc_never_exceeds_half_a_turn() {
        let mut rng = fastrand::Rng::with_seed(0xA11CE);
        for _ in 0..500 {
            let random_unit = |rng: &mut fastrand::Rng| {
                Vec3::new(
                    rng.f32() * 2.0 - 1.0,
                    rng.f32() * 2.0 - 1.0,
                    rng.f32() * 2.0 - 1.0,
                )
                .normalize_or_zero()
            };
            let from = random_unit(&mut rng);
            let to = random_unit(&mut rng);
            if from == Vec3::ZERO || to == Vec3::ZERO {
                continue;
            }

            let angle = shortest_arc(from, to).angle_between(Quat::IDENTITY);
            assert!(
                angle <= PI + 1.0e-3,
                "an arc must take the short way round, got {angle} rad",
            );
        }
    }

    #[test]
    fn mirroring_swaps_left_and_right_bone_names() {
        assert_eq!(mirror_bone(Bone::LeftHand), Bone::RightHand);
        assert_eq!(mirror_bone(Bone::RightToeBase), Bone::LeftToeBase);
        assert_eq!(mirror_bone(Bone::Head), Bone::Head, "central bones map to themselves");
        assert_eq!(mirror_bone(Bone::Hips), Bone::Hips);
    }

    #[test]
    fn mirroring_is_its_own_inverse() {
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, FRAC_PI_4));
        pose.set_rotation(Bone::RightForeArm, Quat::from_axis_angle(Vec3::Z, -0.3));
        pose.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.15));
        pose.root_translation = Vec3::new(0.2, 0.0, -1.0);

        let twice = mirrored(&mirrored(&pose));

        for &bone in Bone::ALL.iter() {
            let original = pose.rotation(bone);
            let round_tripped = neighborhood(original, twice.rotation(bone));
            assert!(
                original.abs_diff_eq(round_tripped, 1.0e-5),
                "mirroring {} twice should return the original, got {round_tripped:?} \
                 for {original:?}",
                bone.name(),
            );
        }
        assert!((twice.root_translation - pose.root_translation).length() < 1.0e-6);
    }

    #[test]
    fn mirroring_reflects_positions_across_the_sagittal_plane() {
        // The property that makes `mirrored` usable as a symmetry check:
        // mirroring the pose must mirror the resulting world positions.
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Z, FRAC_PI_2));
        pose.set_rotation(Bone::Spine1, Quat::from_axis_angle(Vec3::Y, 0.3));

        let positions = forward_kinematics(&pose);
        let mirrored_positions = forward_kinematics(&mirrored(&pose));

        for &bone in Bone::ALL.iter() {
            let expected = {
                let p = positions[mirror_bone(bone)];
                Vec3::new(-p.x, p.y, p.z)
            };
            let actual = mirrored_positions[bone];

            assert!(
                (actual - expected).length() < 1.0e-4,
                "{} should sit at the mirror of {}'s position {expected:?}, got {actual:?}",
                bone.name(),
                mirror_bone(bone).name(),

            );
        }
    }

    #[test]
    fn a_symmetric_pose_is_unchanged_by_mirroring() {
        // Built symmetric by construction, so mirroring must be a no-op.
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Z, 0.4));
        pose.set_rotation(Bone::RightArm, Quat::from_axis_angle(Vec3::Z, -0.4));
        pose.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, 0.2));

        let flipped = mirrored(&pose);

        for &bone in Bone::ALL.iter() {
            let original = pose.rotation(bone);
            let flipped_rotation = neighborhood(original, flipped.rotation(bone));
            assert!(
                original.abs_diff_eq(flipped_rotation, 1.0e-5),
                "a symmetric pose should survive mirroring unchanged, but {} became \
                 {flipped_rotation:?} instead of {original:?}",
                bone.name(),
            );
        }
    }
}

#[cfg(test)]
mod real_pose_conversion {
    use super::*;
    use crate::character::anim::rig::BONE_COUNT;

    /// The position-space `relaxed_stand` targets, frozen.
    ///
    /// These are the exact world positions the superseded
    /// `muscle::pose::relaxed_stand` produced, captured before that module
    /// was deleted. They came from real Mixamo `idle.glb` reference data
    /// via `tools/dump_animation_pose.py`.
    ///
    /// Frozen as a literal rather than read from the live pose, because the
    /// tests below measure a property of **the converter** — how faithfully
    /// it turns positions into rotations — and that needs a fixed,
    /// non-trivial, real-world input to measure against. A synthetic input
    /// would not exercise the multi-child `Spine2` case that the whole
    /// module doc is about, and the rotation-space `poses::relaxed_stand`
    /// cannot serve: it is the converter's *output*, so feeding it back in
    /// would test nothing.
    const RELAXED_STAND_TARGETS: [(Bone, Vec3); BONE_COUNT] = [
        (Bone::Hips, Vec3::new(0.0, 0.94, 0.0)),
        (Bone::Spine, Vec3::new(0.0, 1.11, 0.0)),
        (Bone::Spine1, Vec3::new(-0.0132, 1.2793, 0.013)),
        (Bone::Spine2, Vec3::new(-0.0132, 1.435, 0.0498)),
        (Bone::Neck, Vec3::new(-0.0064, 1.5331, 0.0681)),
        (Bone::Head, Vec3::new(0.0112, 1.7386999, -0.0545)),
        (Bone::LeftShoulder, Vec3::new(-0.0796, 1.6075, 0.0877)),
        (Bone::LeftArm, Vec3::new(-0.20900002, 1.5811, 0.0412)),
        (Bone::LeftForeArm, Vec3::new(-0.24670005, 1.3039, 0.0293)),
        (Bone::LeftHand, Vec3::new(-0.27730006, 1.0461999, 0.0133)),
        (Bone::RightShoulder, Vec3::new(0.0796, 1.6075, 0.0877)),
        (Bone::RightArm, Vec3::new(0.20900002, 1.5811, 0.0412)),
        (Bone::RightForeArm, Vec3::new(0.24670005, 1.3039, 0.0293)),
        (Bone::RightHand, Vec3::new(0.27730006, 1.0461999, 0.0133)),
        (Bone::LeftUpLeg, Vec3::new(-0.1, 0.49, 0.0)),
        (Bone::LeftLeg, Vec3::new(-0.1, 0.07000002, 0.0)),
        (Bone::LeftFoot, Vec3::new(-0.1, 2.2351742e-8, 0.0)),
        (Bone::LeftToeBase, Vec3::new(-0.1, -0.019999977, -0.14)),
        (Bone::RightUpLeg, Vec3::new(0.1, 0.49, 0.0)),
        (Bone::RightLeg, Vec3::new(0.1, 0.07000002, 0.0)),
        (Bone::RightFoot, Vec3::new(0.1, 2.2351742e-8, 0.0)),
        (Bone::RightToeBase, Vec3::new(0.1, -0.019999977, -0.14)),
    ];

    /// The frozen targets as a [`BoneSet`], keyed by bone rather than by
    /// array position so a future reordering of `Bone::ALL` cannot silently
    /// scramble them.
    fn relaxed_stand_targets() -> WorldPositions {
        let mut targets = BoneSet::from_fn(|bone| bone.t_pose_world_position());
        for (bone, position) in RELAXED_STAND_TARGETS {
            targets[bone] = position;
        }
        targets
    }

    /// Guards the fixture itself: every bone must be listed exactly once.
    /// A duplicate or omission would leave a bone silently at its T-pose
    /// and quietly weaken every test below.
    #[test]
    fn the_frozen_fixture_covers_every_bone_exactly_once() {
        for &bone in Bone::ALL.iter() {
            let count =
                RELAXED_STAND_TARGETS.iter().filter(|(listed, _)| *listed == bone).count();
            assert_eq!(count, 1, "{} appears {count} times in the fixture", bone.name());
        }
    }

    /// Measures — rather than assumes — how faithfully the real,
    /// reference-data-derived `relaxed_stand` survives conversion, so the
    /// accept-or-hand-author decision rests on numbers.
    ///
    /// Reports both metrics, because they say different things here:
    /// position error is dominated by the shoulder displacement that
    /// rotation space cannot express (~11.5 cm, see the module doc), while
    /// direction error reports whether the pose's visible shape survived.
    #[test]
    fn relaxed_stand_survives_conversion_with_its_shape_intact() {
        let targets = relaxed_stand_targets();
        let pose = pose_from_world_positions(&targets);

        let positions = conversion_error(&targets, &pose);
        let directions = direction_error_degrees(&targets, &pose);

        eprintln!("\n=== relaxed_stand conversion ===");
        eprintln!("  {:<16} {:>10} {:>12}", "bone", "pos (m)", "dir (deg)");
        let mut rows: Vec<(Bone, f32, f32)> = Bone::ALL
            .iter()
            .map(|&bone| (bone, positions[bone], directions[bone]))
            .filter(|(_, p, d)| *p > 1.0e-5 || *d > 1.0e-3)
            .collect();
        rows.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
        for (bone, position, direction) in &rows {
            eprintln!("  {:<16} {:>10.5} {:>12.3}", bone.name(), position, direction);
        }

        let (worst_bone, worst_degrees) = worst_direction_error(&targets, &pose);
        eprintln!("  worst direction: {} at {:.3} deg\n", worst_bone.name(), worst_degrees);

        // Every bone must point the way it was authored to. This is the
        // pose's shape, and it is what a viewer actually sees.
        assert!(
            worst_degrees < 0.5,
            "the converted pose should reproduce the authored SHAPE, but {} is off by \
             {worst_degrees} degrees",
            worst_bone.name(),
        );
    }

    /// The counterpart to the shape test: documents, as an executable claim,
    /// exactly which positions cannot be represented and by how much — so
    /// the limitation stays measured rather than drifting into folklore.
    #[test]
    fn only_the_shoulder_attachments_lose_their_authored_position() {
        let targets = relaxed_stand_targets();
        let pose = pose_from_world_positions(&targets);
        let errors = conversion_error(&targets, &pose);

        // The spine chain is reproduced essentially exactly: it is a simple
        // chain, so every rotation is fully determined.
        for bone in [Bone::Spine, Bone::Spine1, Bone::Spine2, Bone::Neck, Bone::Head] {
            assert!(
                errors[bone] < 0.002,
                "{} is on the spine chain and should convert almost exactly, got {} m",
                bone.name(),
                errors[bone],
            );
        }

        // The arms inherit the shoulders' unrepresentable displacement. Pin
        // the magnitude so a future change that makes it WORSE is caught,
        // and so the number in the module doc stays honest.
        for bone in [Bone::LeftShoulder, Bone::RightShoulder] {
            assert!(
                (0.05..0.15).contains(&errors[bone]),
                "{}'s position error should sit in the documented ~11.5 cm range \
                 (relaxed_stand authors a 13.7 cm shoulder displacement that rotation \
                 space cannot express), got {} m",
                bone.name(),
                errors[bone],
            );
        }
    }

    /// The legs are untouched by `relaxed_stand`, so they must convert to
    /// exactly the rest pose — a guard against the converter inventing
    /// rotations for bones nobody authored.
    #[test]
    fn untouched_bones_convert_to_the_rest_pose_exactly() {
        let targets = relaxed_stand_targets();
        let pose = pose_from_world_positions(&targets);

        for bone in [
            Bone::LeftUpLeg,
            Bone::LeftLeg,
            Bone::LeftFoot,
            Bone::RightUpLeg,
            Bone::RightLeg,
            Bone::RightFoot,
        ] {
            assert!(
                pose.rotation(bone).abs_diff_eq(Quat::IDENTITY, 1.0e-5),
                "{} is untouched by relaxed_stand and must stay at rest, got {:?}",
                bone.name(),
                pose.rotation(bone),
            );
        }
    }
}
