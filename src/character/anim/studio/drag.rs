//! Direct manipulation: picking a joint in the viewport and dragging it.
//!
//! Sliders are precise and slow. Reaching into the scene and moving the
//! elbow is how this work is actually done, and it is the difference
//! between the studio being a nicer text editor and being an animation
//! tool.
//!
//! As with [`super::edit`], the maths lives here as plain functions over
//! plain values so it can be tested without a window, a camera, or a mouse.
//! The ECS wiring is in [`super::drag_plugin`].

use bevy::math::{Quat, Vec2, Vec3};

use crate::character::anim::rig::{forward_kinematics, LocalPose};
use crate::character::skeleton::Bone;

/// How close to a joint the cursor must be, in pixels, to grab it.
///
/// Generous: joints are drawn small, and a user aiming at "the elbow" is
/// aiming at a region, not a pixel. Too tight and the tool feels broken;
/// too loose and neighbouring joints in a foreshortened limb become
/// impossible to tell apart.
pub const GRAB_RADIUS_PIXELS: f32 = 22.0;

/// A joint the cursor is over, and how far away it was.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointPick {
    /// Which bone's joint was hit.
    pub bone: Bone,
    /// Distance from the cursor, in pixels.
    pub distance_pixels: f32,
    /// Distance from the camera, along the view direction.
    pub depth: f32,
}

/// Finds the joint nearest the cursor.
///
/// `screen_positions` is each bone's joint projected to screen space, with
/// `None` for joints behind the camera or otherwise unprojectable.
///
/// Ties are broken by **depth, nearest first**. On a character viewed from
/// the front, a hand can sit directly over a hip in screen space; picking
/// whichever the iteration order happened to reach first would make the
/// tool feel arbitrary. The one closer to the viewer is the one the user
/// can see, so it is the one they mean.
pub fn pick_joint(
    cursor: Vec2,
    screen_positions: &[(Bone, Option<Vec2>, f32)],
    radius_pixels: f32,
) -> Option<JointPick> {
    let mut best: Option<JointPick> = None;

    for &(bone, position, depth) in screen_positions {
        let Some(position) = position else { continue };
        if depth <= 0.0 {
            continue;
        }

        let distance = position.distance(cursor);
        if distance > radius_pixels {
            continue;
        }

        let candidate = JointPick { bone, distance_pixels: distance, depth };
        best = Some(match best {
            Some(current) if current.depth <= candidate.depth => current,
            _ => candidate,
        });
    }

    best
}

/// A drag in progress.
///
/// Every field is a snapshot taken **when the grab began** and never
/// updated. That is what makes a drag idempotent: the rotation for a given
/// cursor position is computed from the same fixed reference every frame,
/// so holding the mouse still holds the limb still.
///
/// Updating any of these from the live rig instead produces a feedback
/// loop — the frame's correction is computed against a pose that already
/// includes the previous frame's correction, and the limb spins. That was
/// a real, user-reported bug.
#[derive(Debug, Clone, Copy)]
pub struct BoneDrag {
    /// The bone being rotated. Note this is the **parent** of the joint the
    /// user grabbed: grabbing the elbow and pulling bends the upper arm,
    /// because a joint's position is set by its parent's rotation.
    pub bone: Bone,
    /// The joint the user grabbed, whose position they are steering.
    pub handle: Bone,
    /// The grabbed joint's world position when the drag began.
    pub start_world: Vec3,
    /// The pivot (the rotated bone's own joint) when the drag began.
    pub pivot_world: Vec3,
    /// The rotated bone's parent's world rotation when the drag began.
    pub parent_world: Quat,
    /// The bone's local rotation when the drag began.
    pub start_rotation: Quat,
    /// Offset from the cursor to the joint at the moment of the grab.
    ///
    /// Added to the cursor's world position every frame so the joint keeps
    /// its distance from the pointer instead of snapping onto it. Without
    /// it, clicking a joint anywhere inside the grab radius — up to 22
    /// pixels away — instantly jerks the limb by that much before the drag
    /// has even started, which reads as the joint teleporting on click.
    pub grab_offset: Vec3,
}

/// Which bone a grab on `handle` should rotate, if any.
///
/// A joint's world position is determined by its **parent's** rotation —
/// rotating a bone moves its children, not itself. So grabbing the elbow
/// and dragging must rotate the upper arm. Grabbing the root has nothing
/// above it to rotate, and returns `None`.
pub fn bone_for_handle(handle: Bone) -> Option<Bone> {
    handle.parent()
}

/// The rotation that best aims `bone` so its child lands under the cursor.
///
/// Works in the bone's own parent frame: both the current and the desired
/// direction are expressed relative to the joint's parent, and the result
/// is the shortest arc between them composed onto the bone's rest
/// rotation. That keeps the operation a pure rotation — it cannot stretch
/// a bone, whatever the user drags toward, including a point the limb
/// cannot physically reach (it aims at it and stops at its own length).
///
/// `pivot_world` is the dragged joint's parent's position, `target_world`
/// is where the user wants the joint to be.
pub fn aim_rotation(
    pivot_world: Vec3,
    current_world: Vec3,
    target_world: Vec3,
    parent_world_rotation: Quat,
    start_rotation: Quat,
) -> Quat {
    let current = current_world - pivot_world;
    let target = target_world - pivot_world;

    // A degenerate direction means the user has dragged onto the pivot
    // itself; there is no meaningful aim, so hold the pose rather than
    // spinning to an arbitrary axis.
    let (Some(current), Some(target)) =
        (current.try_normalize(), target.try_normalize())
    else {
        return start_rotation;
    };

    // Express both in the parent's frame before differencing: a world-space
    // arc composed onto a local rotation is the classic "right angle, wrong
    // axis" bug this project has hit before.
    let inverse_parent = parent_world_rotation.inverse();
    let local_current = inverse_parent * current;
    let local_target = inverse_parent * target;

    let arc = shortest_arc(local_current, local_target);
    arc * start_rotation
}

/// The shortest rotation taking `from` onto `to`, with an antipodal guard.
///
/// `Quat::from_rotation_arc` is undefined for exactly-opposite directions.
/// A drag *will* produce that — pulling a limb straight through the pivot
/// to the far side is a natural motion — so the degenerate case is chosen
/// explicitly rather than left to whatever the library happens to return.
fn shortest_arc(from: Vec3, to: Vec3) -> Quat {
    const ANTIPODAL: f32 = -0.999_9;

    if from.dot(to) < ANTIPODAL {
        // Any perpendicular axis is a valid half-turn; pick a stable one so
        // the same drag always produces the same result.
        let axis = from.any_orthonormal_vector();
        return Quat::from_axis_angle(axis, std::f32::consts::PI);
    }

    Quat::from_rotation_arc(from, to)
}

/// Where a dragged joint should go, given the cursor.
///
/// The cursor is a ray; the joint is a point. Constraining the joint to the
/// plane through its original position, facing the camera, is the standard
/// resolution: the joint tracks the cursor exactly in the two dimensions
/// the user can see, and keeps its depth in the one they cannot.
///
/// Returns `None` when the ray is parallel to the plane, which cannot
/// happen for a camera-facing plane but is guarded rather than assumed.
pub fn drag_target_on_view_plane(
    ray_origin: Vec3,
    ray_direction: Vec3,
    plane_point: Vec3,
    plane_normal: Vec3,
) -> Option<Vec3> {
    let denominator = ray_direction.dot(plane_normal);
    if denominator.abs() < 1.0e-6 {
        return None;
    }

    let t = (plane_point - ray_origin).dot(plane_normal) / denominator;
    if t <= 0.0 {
        return None;
    }

    Some(ray_origin + ray_direction * t)
}

/// Each bone's world position under `pose`, for picking against.
pub fn joint_world_positions(pose: &LocalPose) -> Vec<(Bone, Vec3)> {
    let positions = forward_kinematics(pose);
    Bone::ALL.iter().map(|&bone| (bone, positions[bone])).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::poses;
    use crate::character::anim::rig::forward_kinematics;

    fn at(bone: Bone, x: f32, y: f32, depth: f32) -> (Bone, Option<Vec2>, f32) {
        (bone, Some(Vec2::new(x, y)), depth)
    }

    #[test]
    fn picking_finds_a_joint_under_the_cursor() {
        let joints = [at(Bone::LeftArm, 100.0, 100.0, 5.0)];
        let pick = pick_joint(Vec2::new(105.0, 103.0), &joints, GRAB_RADIUS_PIXELS);

        assert_eq!(pick.expect("should hit").bone, Bone::LeftArm);
    }

    #[test]
    fn picking_ignores_joints_outside_the_grab_radius() {
        let joints = [at(Bone::LeftArm, 100.0, 100.0, 5.0)];
        let pick = pick_joint(Vec2::new(400.0, 400.0), &joints, GRAB_RADIUS_PIXELS);

        assert!(pick.is_none(), "a distant joint should not be grabbed");
    }

    #[test]
    fn picking_prefers_the_nearer_joint_when_two_overlap() {
        // The case that makes the tool feel arbitrary if handled by
        // iteration order: on a front view a hand can sit directly over a
        // hip. The one the user can SEE is the one they mean.
        let joints = [
            at(Bone::LeftUpLeg, 100.0, 100.0, 9.0),
            at(Bone::LeftHand, 102.0, 101.0, 3.0),
        ];

        let pick = pick_joint(Vec2::new(101.0, 100.0), &joints, GRAB_RADIUS_PIXELS);
        assert_eq!(
            pick.expect("should hit").bone,
            Bone::LeftHand,
            "the joint closer to the camera should win",
        );
    }

    #[test]
    fn picking_prefers_the_nearer_joint_whatever_the_list_order() {
        // Same test, list reversed: if this passes only one way round, the
        // tie-break is not actually happening.
        let joints = [
            at(Bone::LeftHand, 102.0, 101.0, 3.0),
            at(Bone::LeftUpLeg, 100.0, 100.0, 9.0),
        ];

        let pick = pick_joint(Vec2::new(101.0, 100.0), &joints, GRAB_RADIUS_PIXELS);
        assert_eq!(pick.expect("should hit").bone, Bone::LeftHand);
    }

    #[test]
    fn picking_skips_joints_behind_the_camera() {
        // A joint behind the viewer projects to a screen position that can
        // land anywhere, including under the cursor. Depth is what
        // distinguishes it.
        let joints = [(Bone::LeftArm, Some(Vec2::new(100.0, 100.0)), -2.0)];
        let pick = pick_joint(Vec2::new(100.0, 100.0), &joints, GRAB_RADIUS_PIXELS);

        assert!(pick.is_none(), "a joint behind the camera is not grabbable");
    }

    #[test]
    fn picking_skips_unprojectable_joints() {
        let joints = [(Bone::LeftArm, None, 5.0)];
        assert!(pick_joint(Vec2::ZERO, &joints, GRAB_RADIUS_PIXELS).is_none());
    }

    #[test]
    fn grabbing_a_joint_rotates_its_parent() {
        // The rule that makes dragging behave the way a viewer expects:
        // rotating a bone moves its CHILDREN, so steering the elbow means
        // rotating the upper arm.
        assert_eq!(bone_for_handle(Bone::LeftForeArm), Some(Bone::LeftArm));
        assert_eq!(bone_for_handle(Bone::LeftHand), Some(Bone::LeftForeArm));
    }

    #[test]
    fn the_root_has_nothing_to_rotate() {
        assert_eq!(bone_for_handle(Bone::Hips), None);
    }

    #[test]
    fn aiming_points_the_bone_at_the_target() {
        // The core claim: after aiming, the dragged joint lies along the
        // direction of the target.
        let pivot = Vec3::ZERO;
        let current = Vec3::new(0.0, -1.0, 0.0);
        let target = Vec3::new(1.0, 0.0, 0.0);

        let rotation =
            aim_rotation(pivot, current, target, Quat::IDENTITY, Quat::IDENTITY);

        let aimed = rotation * (current - pivot);
        assert!(
            aimed.normalize().dot((target - pivot).normalize()) > 0.999,
            "the bone should point at the target, got {aimed:?}",
        );
    }

    #[test]
    fn aiming_works_in_the_parents_frame_not_the_world() {
        // The "right angle, wrong axis" bug class this project has hit
        // before: a world-space arc composed onto a LOCAL rotation is
        // wrong whenever the parent is itself rotated.
        //
        // With the parent turned 90 degrees about Y, a world-space target
        // must still be reached — which it only is if the arc was
        // expressed in the parent's frame first.
        let parent = Quat::from_axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_2);
        let pivot = Vec3::ZERO;
        let current = Vec3::new(0.0, -1.0, 0.0);
        let target = Vec3::new(0.0, 0.0, 1.0);

        let local = aim_rotation(pivot, current, target, parent, Quat::IDENTITY);

        // Reconstruct the world direction exactly as the rig does:
        // `parent_world * local * offset`, where `offset` is the child's
        // rest offset in the PARENT's frame.
        //
        // An earlier version of this test wrote
        // `parent * local * (parent.inverse() * offset)` — pre-rotating the
        // offset into the parent frame, which the rig never does. That is
        // the buggy convention the code itself used, so the test agreed
        // with the bug and passed while dragging a joint UP rotated it
        // DOWN on the real rig.
        let offset_in_parent_frame = parent.inverse() * (current - pivot);
        let aimed = parent * local * offset_in_parent_frame;

        assert!(
            aimed.normalize().dot((target - pivot).normalize()) > 0.999,
            "aiming under a rotated parent should still reach the target, got {aimed:?}",
        );
    }

    #[test]
    fn aiming_reaches_the_target_through_the_rigs_own_composition() {
        // The test that actually pins the convention, written after the
        // one above turned out to validate the bug rather than the fix.
        //
        // `forward_kinematics` computes a child's world position as
        // `parent_world_rotation * (bind * local) * offset`, with `offset`
        // in the parent's own frame. So the ONLY meaningful claim is: feed
        // `aim_rotation` the same inputs the plugin does, put its answer
        // back through that exact composition, and land on the target.
        //
        // The symptom this catches is specific — a parent carrying a
        // half-turn (which the real mesh's spine and shoulder binds do)
        // inverts one screen axis while leaving the other correct, which
        // reads as "vertical is backwards, horizontal is fine".
        for parent in [
            Quat::IDENTITY,
            Quat::from_axis_angle(Vec3::Y, std::f32::consts::PI),
            Quat::from_axis_angle(Vec3::Z, std::f32::consts::PI),
            Quat::from_axis_angle(Vec3::X, 0.7) * Quat::from_axis_angle(Vec3::Y, 2.3),
        ] {
            let pivot = Vec3::new(0.2, 1.4, 0.0);
            let start_rotation = Quat::from_axis_angle(Vec3::Z, 0.3);

            // The child's offset in the parent's frame, and where that puts
            // it in the world under the starting pose.
            let offset = Vec3::new(0.0, -0.3, 0.0);
            let current = pivot + parent * start_rotation * offset;

            // Somewhere reachable: the same distance, swung 40 degrees.
            let target = pivot
                + Quat::from_axis_angle(Vec3::X, 0.7) * (current - pivot);

            let local = aim_rotation(pivot, current, target, parent, start_rotation);

            // The rig's own composition, verbatim.
            let aimed = pivot + parent * local * offset;

            assert!(
                aimed.distance(target) < 1.0e-4,
                "under parent {parent:?} the aim landed at {aimed:?} instead of \
                 {target:?} — the arc is being composed in the wrong frame",
            );
        }
    }

    #[test]
    fn aiming_at_the_pivot_itself_holds_the_pose() {
        // Dragging the cursor onto the joint's own parent is a degenerate
        // aim. Returning the starting rotation keeps the limb still instead
        // of snapping it to an arbitrary axis.
        let start = Quat::from_axis_angle(Vec3::Z, 0.4);
        let pivot = Vec3::new(1.0, 2.0, 3.0);

        let held = aim_rotation(pivot, pivot, Vec3::new(4.0, 5.0, 6.0), Quat::IDENTITY, start);
        assert_eq!(held, start);

        let held = aim_rotation(pivot, Vec3::new(4.0, 5.0, 6.0), pivot, Quat::IDENTITY, start);
        assert_eq!(held, start);
    }

    #[test]
    fn aiming_straight_backwards_produces_a_valid_half_turn() {
        // Pulling a limb through its own pivot to the far side is a natural
        // drag and an antipodal arc, where `from_rotation_arc` is
        // undefined. The result must still be a usable rotation.
        let pivot = Vec3::ZERO;
        let current = Vec3::new(0.0, -1.0, 0.0);
        let target = Vec3::new(0.0, 1.0, 0.0);

        let rotation =
            aim_rotation(pivot, current, target, Quat::IDENTITY, Quat::IDENTITY);

        assert!(rotation.is_finite(), "an antipodal drag must not produce NaN");
        assert!(
            (rotation.length() - 1.0).abs() < 1.0e-4,
            "the result must be a unit quaternion, got length {}",
            rotation.length(),
        );

        let aimed = rotation * current;
        assert!(
            aimed.normalize().dot(target.normalize()) > 0.999,
            "a half turn should reach the opposite direction, got {aimed:?}",
        );
    }

    #[test]
    fn dragging_never_stretches_a_bone() {
        // The structural invariant, on the drag path. Whatever the user
        // aims at — including a point far beyond the limb's reach — the
        // result is a rotation, and a rotation cannot change a length.
        let mut pose = poses::relaxed_stand();
        let positions = forward_kinematics(&pose);

        let rotation = aim_rotation(
            positions[Bone::LeftArm],
            positions[Bone::LeftForeArm],
            // Deliberately unreachable: ten metres away.
            positions[Bone::LeftForeArm] + Vec3::new(10.0, 4.0, -3.0),
            Quat::IDENTITY,
            pose.rotation(Bone::LeftArm),
        );
        pose.set_rotation(Bone::LeftArm, rotation);

        let dragged = forward_kinematics(&pose);
        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest = bone.t_pose_offset().length();
            let posed = (dragged[bone] - dragged[parent]).length();

            assert!(
                (posed - rest).abs() < 1.0e-5,
                "dragging stretched {} to {posed} m against a rest length of {rest} m",
                bone.name(),
            );
        }
    }

    #[test]
    fn aiming_is_idempotent() {
        // The property that makes a drag stable, and the one whose absence
        // made limbs spin: aiming at the same target twice must give the
        // same answer.
        //
        // The bug was structural rather than in this function — the caller
        // re-read the joint's LIVE position each frame while composing onto
        // the rotation from the grab, so every frame re-applied the
        // accumulated arc on top of itself. Simulated here by feeding the
        // result back in, which is what the caller was effectively doing.
        let pivot = Vec3::ZERO;
        let start = Vec3::new(0.0, -1.0, 0.0);
        let target = Vec3::new(0.7, -0.7, 0.0);

        let once = aim_rotation(pivot, start, target, Quat::IDENTITY, Quat::IDENTITY);
        let twice = aim_rotation(pivot, start, target, Quat::IDENTITY, Quat::IDENTITY);

        assert!(
            once.abs_diff_eq(twice, 1.0e-6),
            "aiming twice at the same target must give the same rotation",
        );

    }

    #[test]
    fn a_drifting_parent_frame_walks_the_bone() {
        // The actual mechanism behind the user-reported spinning, isolated.
        //
        // `parent_world` was read LIVE from the rig every frame. Writing the
        // edited pose re-runs retargeting, which can move the parent — so
        // the next frame expressed the SAME world-space arc in a DIFFERENT
        // frame, producing a different local rotation for an unmoved
        // cursor. The bone walks, frame after frame, while the mouse is
        // held still.
        //
        // (An earlier theory — that re-reading the joint's live POSITION
        // compounds the arc — was tested and disproved: that loop
        // converges. Recorded so the wrong explanation is not rediscovered.)
        let pivot = Vec3::ZERO;
        let start = Vec3::new(0.0, -1.0, 0.0);
        let target = Vec3::new(0.6, -0.8, 0.0);

        let fixed_frame = Quat::IDENTITY;
        let drifted_frame = Quat::from_axis_angle(Vec3::Y, 0.15);

        let with_fixed = aim_rotation(pivot, start, target, fixed_frame, Quat::IDENTITY);
        let with_drifted =
            aim_rotation(pivot, start, target, drifted_frame, Quat::IDENTITY);

        assert!(
            !with_fixed.abs_diff_eq(with_drifted, 1.0e-3),
            "a changed parent frame must change the local rotation — that is why the \
             frame has to be snapshotted at the grab rather than re-read each frame",
        );
    }

    #[test]
    fn repeated_aiming_from_the_grab_snapshot_never_drifts() {
        // Sixty frames of holding the mouse still. With the drag's inputs
        // frozen at the grab, every frame must produce exactly the same
        // rotation — which is what "the limb holds still" means.
        let pivot = Vec3::ZERO;
        let start = Vec3::new(0.0, -1.0, 0.0);
        let target = Vec3::new(0.5, -0.8, 0.3);
        let start_rotation = Quat::from_axis_angle(Vec3::Z, 0.2);

        let first = aim_rotation(pivot, start, target, Quat::IDENTITY, start_rotation);

        for frame in 1..60 {
            let again = aim_rotation(pivot, start, target, Quat::IDENTITY, start_rotation);
            assert!(
                first.abs_diff_eq(again, 1.0e-6),
                "frame {frame} produced a different rotation from frame 0 despite \
                 identical inputs — the drag is not idempotent",
            );
        }
    }

    #[test]
    fn a_grab_offset_keeps_the_joint_off_the_cursor() {
        // The click-jump bug: the grab radius is 22 pixels, so a click
        // rarely lands dead-centre on a joint. Aiming at the raw cursor
        // position snaps the limb by that offset the instant the button
        // goes down, which reads as the joint teleporting on click.
        //
        // Carrying the offset means a click with no mouse MOVEMENT is a
        // no-op, which is what a user expects.
        let pivot = Vec3::ZERO;
        let joint = Vec3::new(0.0, -1.0, 0.0);
        // The cursor landed 8 cm to the side of the joint.
        let cursor_at_grab = joint + Vec3::new(0.08, 0.0, 0.0);
        let grab_offset = joint - cursor_at_grab;

        // Frame one: the cursor has not moved.
        let target = cursor_at_grab + grab_offset;
        let rotation = aim_rotation(pivot, joint, target, Quat::IDENTITY, Quat::IDENTITY);

        assert!(
            rotation.abs_diff_eq(Quat::IDENTITY, 1.0e-6),
            "clicking a joint without moving the mouse must not rotate anything, got \
             {rotation:?}",
        );
    }

    #[test]
    fn without_the_grab_offset_a_click_would_jump() {
        // The counterpart: proves the offset is doing real work rather than
        // being a no-op that happens to look right.
        let pivot = Vec3::ZERO;
        let joint = Vec3::new(0.0, -1.0, 0.0);
        let cursor_at_grab = joint + Vec3::new(0.08, 0.0, 0.0);

        // Aiming at the bare cursor, as the buggy version did.
        let rotation =
            aim_rotation(pivot, joint, cursor_at_grab, Quat::IDENTITY, Quat::IDENTITY);

        assert!(
            !rotation.abs_diff_eq(Quat::IDENTITY, 1.0e-3),
            "test intent: aiming at the raw cursor DOES move the joint, which is the \
             jump the offset exists to prevent",
        );
    }

    #[test]
    fn a_view_plane_ray_hits_where_expected() {
        // Straight down -Z at a plane 5 units away facing the camera.
        let hit = drag_target_on_view_plane(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::new(0.0, 0.0, -5.0),
            Vec3::Z,
        );

        assert_eq!(hit, Some(Vec3::new(0.0, 0.0, -5.0)));
    }

    #[test]
    fn a_ray_parallel_to_the_plane_misses() {
        let hit = drag_target_on_view_plane(
            Vec3::ZERO,
            Vec3::X,
            Vec3::new(0.0, 0.0, -5.0),
            Vec3::Z,
        );

        assert!(hit.is_none(), "a parallel ray has no intersection");
    }

    #[test]
    fn a_plane_behind_the_camera_is_not_hit() {
        // Guards against dragging a joint to a point behind the viewer,
        // which would send the limb somewhere the user cannot see.
        let hit = drag_target_on_view_plane(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::new(0.0, 0.0, 5.0),
            Vec3::Z,
        );

        assert!(hit.is_none());
    }

    #[test]
    fn joint_positions_cover_every_bone() {
        let positions = joint_world_positions(&poses::relaxed_stand());
        assert_eq!(positions.len(), Bone::ALL.len());
    }
}
