//! The pose being edited, and the operations an editor performs on it.
//!
//! Deliberately free of egui: everything here is plain values and plain
//! functions, so the editing *semantics* — what "reset this bone" or "mirror
//! to the other side" actually means — are unit-testable without standing up
//! a UI, a window, or a running app. The panel in [`super::pose_editor`] is
//! then a thin translation of clicks into these calls.
//!
//! This mirrors the split the rest of `anim` already uses: [`super::super::ragdoll`]
//! is plain functions and [`super::super::ragdoll_plugin`] is the ECS wiring,
//! precisely so the control law can be tested without a physics world.

use bevy::math::{Quat, Vec3};

use crate::character::anim::convert::mirror_bone;
use crate::character::anim::rig::LocalPose;
use crate::character::skeleton::Bone;

/// One bone's rotation, in the form a person edits it.
///
/// Axis-and-angle rather than raw `(x, y, z, w)`: nobody can look at
/// `(0.0, 0.0, 0.581, 0.814)` and see "71 degrees about Z", and an editor
/// whose numbers cannot be read is not much better than editing the file by
/// hand. This is the same decomposition [`super::super::asset::AuthoredRotation`]
/// stores, so what the editor shows and what the file says are the same
/// quantity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EditableRotation {
    /// Axis to turn about, in the bone's own rest frame.
    pub axis: Vec3,
    /// How far to turn.
    pub degrees: f32,
}

impl Default for EditableRotation {
    /// No rotation. `+Y` rather than a zero axis so dragging the angle
    /// slider off zero immediately produces a usable rotation instead of a
    /// degenerate one.
    fn default() -> Self {
        Self { axis: Vec3::Y, degrees: 0.0 }
    }
}

impl EditableRotation {
    /// The quaternion this describes.
    ///
    /// A zero or degenerate axis yields identity rather than NaN — an
    /// editor lets a user drag an axis component to zero mid-edit, and that
    /// transient state must not poison the rig.
    pub fn to_quat(self) -> Quat {
        let axis = self.axis.normalize_or_zero();
        if axis == Vec3::ZERO || self.degrees == 0.0 {
            Quat::IDENTITY
        } else {
            Quat::from_axis_angle(axis, self.degrees.to_radians())
        }
    }

    /// Describes `rotation` in editable form.
    ///
    /// Note this is lossy in one specific way: `to_axis_angle` always
    /// returns a non-negative angle, so a rotation authored as `-30 degrees
    /// about +X` comes back as `+30 degrees about -X`. Those are the same
    /// rotation, and [`Self::to_quat`] reproduces it exactly — but the
    /// numbers a user sees can flip sign after a round trip. That is why
    /// the editor keeps its own [`EditableRotation`] per bone rather than
    /// re-deriving one from the pose every frame.
    pub fn from_quat(rotation: Quat) -> Self {
        let (axis, angle) = rotation.to_axis_angle();
        if angle.abs() < 1.0e-6 {
            return Self::default();
        }
        Self { axis, degrees: angle.to_degrees() }
    }
}

/// A pose open in the editor, plus the state an editor needs that a
/// [`LocalPose`] does not carry.
#[derive(Debug, Clone)]
pub struct PoseEdit {
    /// Per-bone rotations in editable form.
    ///
    /// Held alongside the pose rather than derived from it each frame, so a
    /// user's chosen axis survives round-tripping through a quaternion (see
    /// [`EditableRotation::from_quat`]). Dragging an angle to zero and back
    /// keeps the axis the user picked instead of snapping to a canonical
    /// one.
    rotations: Vec<EditableRotation>,
    /// Root displacement, metres.
    pub root_translation: Vec3,
    /// Whether anything has changed since the last save or load.
    ///
    /// The plan calls this out explicitly: an asset hot-reload must not
    /// silently discard in-progress edits, so the studio has to know whether
    /// there are any.
    dirty: bool,
    /// Which file this came from, if any, so "Save" can mean "save back
    /// where it came from" rather than always prompting.
    pub source: Option<String>,
}

impl Default for PoseEdit {
    fn default() -> Self {
        Self::from_pose(&LocalPose::REST)
    }
}

impl PoseEdit {
    /// Opens `pose` for editing.
    pub fn from_pose(pose: &LocalPose) -> Self {
        Self {
            rotations: Bone::ALL
                .iter()
                .map(|&bone| EditableRotation::from_quat(pose.rotation(bone)))
                .collect(),
            root_translation: pose.root_translation,
            dirty: false,
            source: None,
        }
    }

    /// Opens `pose`, remembering which file it came from.
    pub fn from_file(pose: &LocalPose, path: impl Into<String>) -> Self {
        Self { source: Some(path.into()), ..Self::from_pose(pose) }
    }

    /// The pose these edits describe, ready to write onto a rig.
    pub fn to_pose(&self) -> LocalPose {
        let mut pose = LocalPose::REST;
        for (index, &bone) in Bone::ALL.iter().enumerate() {
            pose.set_rotation(bone, self.rotations[index].to_quat());
        }
        pose.root_translation = self.root_translation;
        pose
    }

    /// This bone's rotation as the editor holds it.
    pub fn rotation(&self, bone: Bone) -> EditableRotation {
        self.rotations[bone.index()]
    }

    /// Replaces one bone's rotation, marking the pose dirty.
    ///
    /// Dirty tracking lives here rather than at the call site so it cannot
    /// be forgotten — every mutation goes through one of these methods.
    ///
    /// # Why the comparison has a tolerance
    ///
    /// A UI widget writes back the value it *displays*, which is rounded to
    /// however many digits it shows. Merely rendering a panel therefore
    /// feeds back a value a hair different from the one it was given, and
    /// an exact `!=` would call that an edit — reporting unsaved changes on
    /// a pose nobody touched, every frame, forever. Live-caught doing
    /// exactly that.
    ///
    /// The widgets render six decimals, so a round trip can shift a value
    /// by up to half of the last digit — 5e-7. The tolerance is set just
    /// above that: large enough to absorb display rounding, and far below
    /// anything a person could mean to change (a millionth of a degree).
    pub fn set_rotation(&mut self, bone: Bone, rotation: EditableRotation) {
        const UNCHANGED: f32 = 1.0e-6;

        let existing = self.rotations[bone.index()];
        let same = (existing.degrees - rotation.degrees).abs() < UNCHANGED
            && existing.axis.abs_diff_eq(rotation.axis, UNCHANGED);

        if !same {
            self.rotations[bone.index()] = rotation;
            self.dirty = true;
        }
    }

    /// Returns one bone to rest.
    pub fn reset_bone(&mut self, bone: Bone) {
        self.set_rotation(bone, EditableRotation::default());
    }

    /// Returns every bone to rest.
    pub fn reset_all(&mut self) {
        for &bone in Bone::ALL.iter() {
            self.reset_bone(bone);
        }
    }

    /// Copies one side of the body onto the other.
    ///
    /// Takes the pose's *current* rotations, mirrors them, and keeps only
    /// the half that `from_left` asks for — so "mirror left to right"
    /// leaves every left bone untouched and overwrites every right one.
    /// Central bones (spine, head) are never touched: mirroring them is
    /// either a no-op or a lie, and `relaxed_stand` carries deliberate
    /// central asymmetry from its mocap source that this must not destroy.
    pub fn mirror(&mut self, from_left: bool) {
        let mirrored = crate::character::anim::convert::mirrored(&self.to_pose());

        for &bone in Bone::ALL.iter() {
            let counterpart = mirror_bone(bone);
            if counterpart == bone {
                // A central bone: it is its own mirror.
                continue;
            }

            // Overwrite the side we are mirroring ONTO.
            let is_left = bone.name().starts_with("Left");
            let overwrite = if from_left { !is_left } else { is_left };
            if overwrite {
                self.set_rotation(bone, EditableRotation::from_quat(mirrored.rotation(bone)));
            }
        }
    }

    /// Whether there are unsaved changes.
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Marks the pose saved.
    pub fn mark_clean(&mut self) {
        self.dirty = false;
    }

    /// Marks the pose as carrying unsaved work.
    ///
    /// For changes that replace the whole pose at once rather than going
    /// through [`Self::set_rotation`] — capturing from the live rig, most
    /// obviously. Without this such a change would look saved, and the next
    /// load would discard it silently.
    pub fn set_dirty(&mut self) {
        self.dirty = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::poses;
    use crate::character::anim::rig::forward_kinematics;

    #[test]
    fn an_editable_rotation_round_trips_through_a_quaternion() {
        let original = Quat::from_axis_angle(Vec3::new(0.3, 0.8, -0.5).normalize(), 0.9);
        let editable = EditableRotation::from_quat(original);

        assert!(
            editable.to_quat().abs_diff_eq(original, 1.0e-5),
            "an editable rotation should describe the same rotation it came from, got {:?}",
            editable.to_quat(),
        );
    }

    #[test]
    fn a_degenerate_axis_yields_identity_rather_than_nan() {
        // A user dragging every axis component to zero is a transient the
        // editor must survive — a NaN here would propagate into the rig and
        // stay there.
        let degenerate = EditableRotation { axis: Vec3::ZERO, degrees: 45.0 };
        assert_eq!(degenerate.to_quat(), Quat::IDENTITY);
        assert!(degenerate.to_quat().is_finite());
    }

    #[test]
    fn a_zero_angle_yields_identity_whatever_the_axis() {
        for axis in [Vec3::X, Vec3::Y, Vec3::Z, Vec3::new(1.0, 2.0, 3.0)] {
            let rotation = EditableRotation { axis, degrees: 0.0 };
            assert_eq!(rotation.to_quat(), Quat::IDENTITY, "axis {axis:?}");
        }
    }

    #[test]
    fn an_edited_pose_round_trips_through_the_editor() {
        // Opening a real pose and immediately saving it must not change it.
        // This is what makes the editor safe to open on authored data: a
        // user who opens `relaxed_stand` to look at it, then saves, must get
        // the same file back.
        let original = poses::relaxed_stand();
        let edit = PoseEdit::from_pose(&original);
        let round_tripped = edit.to_pose();

        for &bone in Bone::ALL.iter() {
            assert!(
                original.rotation(bone).abs_diff_eq(round_tripped.rotation(bone), 1.0e-5),
                "{} changed just by being opened in the editor: {:?} -> {:?}",
                bone.name(),
                original.rotation(bone),
                round_tripped.rotation(bone),
            );
        }
    }

    #[test]
    fn opening_a_pose_leaves_it_clean_and_editing_marks_it_dirty() {
        let mut edit = PoseEdit::from_pose(&poses::relaxed_stand());
        assert!(!edit.is_dirty(), "just opening a pose is not an edit");

        // Setting a bone to what it already is is not an edit either —
        // otherwise a slider that merely renders its current value would
        // mark every pose dirty on the first frame.
        let unchanged = edit.rotation(Bone::LeftArm);
        edit.set_rotation(Bone::LeftArm, unchanged);
        assert!(!edit.is_dirty(), "a no-op assignment should not dirty the pose");

        edit.set_rotation(
            Bone::LeftArm,
            EditableRotation { axis: Vec3::Z, degrees: 45.0 },
        );
        assert!(edit.is_dirty(), "changing a bone should dirty the pose");

        edit.mark_clean();
        assert!(!edit.is_dirty());
    }

    #[test]
    fn a_widgets_rounded_write_back_does_not_count_as_an_edit() {
        // The live-caught bug: egui writes back the value it DISPLAYS, and
        // a displayed value is rounded. Merely rendering the panel fed a
        // slightly different number back in, which an exact comparison
        // called an edit — so an untouched pose reported unsaved changes,
        // and worse, the rounded axis was renormalized and re-rounded every
        // frame until `LeftShoulder` had drifted from 25.56 degrees to
        // 71.36.
        let mut edit = PoseEdit::from_pose(&poses::relaxed_stand());
        let original = edit.rotation(Bone::LeftShoulder);

        // What a six-decimal widget would hand back.
        let round_tripped = EditableRotation {
            axis: Vec3::new(
                (original.axis.x * 1.0e6).round() / 1.0e6,
                (original.axis.y * 1.0e6).round() / 1.0e6,
                (original.axis.z * 1.0e6).round() / 1.0e6,
            ),
            degrees: (original.degrees * 1.0e6).round() / 1.0e6,
        };

        edit.set_rotation(Bone::LeftShoulder, round_tripped);

        assert!(
            !edit.is_dirty(),
            "a rounded round trip is not an edit, but the pose was marked dirty",
        );
    }

    #[test]
    fn a_real_edit_still_registers_despite_the_tolerance() {
        // The tolerance must not swallow genuine changes — a slider nudged
        // by a degree is an edit and has to be saved.
        let mut edit = PoseEdit::from_pose(&poses::relaxed_stand());
        let mut nudged = edit.rotation(Bone::LeftShoulder);
        nudged.degrees += 1.0;

        edit.set_rotation(Bone::LeftShoulder, nudged);

        assert!(edit.is_dirty(), "a one-degree change is a real edit");
    }

    #[test]
    fn resetting_a_bone_returns_it_to_rest() {
        let mut edit = PoseEdit::from_pose(&poses::relaxed_stand());
        assert_ne!(edit.rotation(Bone::LeftArm).degrees, 0.0);

        edit.reset_bone(Bone::LeftArm);

        assert_eq!(edit.to_pose().rotation(Bone::LeftArm), Quat::IDENTITY);
        // And only that bone.
        assert_ne!(edit.to_pose().rotation(Bone::RightArm), Quat::IDENTITY);
    }

    #[test]
    fn resetting_everything_yields_the_rest_pose() {
        let mut edit = PoseEdit::from_pose(&poses::wave());
        edit.reset_all();

        for &bone in Bone::ALL.iter() {
            assert_eq!(edit.to_pose().rotation(bone), Quat::IDENTITY, "{}", bone.name());
        }
    }

    #[test]
    fn mirroring_left_to_right_copies_the_pose_across() {
        // `wave` raises only the RIGHT arm, so mirroring right-to-left must
        // produce a raised left arm.
        let mut edit = PoseEdit::from_pose(&poses::wave());
        edit.mirror(false);

        let positions = forward_kinematics(&edit.to_pose());

        assert!(
            positions[Bone::LeftHand].y > positions[Bone::Head].y * 0.85,
            "after mirroring the wave, the LEFT hand should be raised, got y={}",
            positions[Bone::LeftHand].y,
        );
    }

    #[test]
    fn mirroring_leaves_the_source_side_untouched() {
        // The direction has to actually mean something: mirroring right
        // onto left must not disturb the right arm it copied from.
        let original = poses::wave();
        let mut edit = PoseEdit::from_pose(&original);
        edit.mirror(false);

        for bone in [Bone::RightShoulder, Bone::RightArm, Bone::RightForeArm] {
            assert!(
                original.rotation(bone).abs_diff_eq(edit.to_pose().rotation(bone), 1.0e-4),
                "{} is on the source side and must not change when mirroring onto the \
                 other side",
                bone.name(),
            );
        }
    }

    #[test]
    fn mirroring_never_touches_the_central_chain() {
        // `relaxed_stand` carries deliberate spine/neck asymmetry from its
        // real mocap source — see `poses::symmetric_named_poses`. Mirroring
        // a limb must not quietly straighten that out.
        let original = poses::relaxed_stand();
        let mut edit = PoseEdit::from_pose(&original);
        edit.mirror(true);

        for bone in [Bone::Spine, Bone::Spine1, Bone::Spine2, Bone::Neck, Bone::Head] {
            assert!(
                original.rotation(bone).abs_diff_eq(edit.to_pose().rotation(bone), 1.0e-5),
                "{} is a central bone and must survive mirroring unchanged",
                bone.name(),
            );
        }
    }

    #[test]
    fn an_edited_pose_can_never_stretch_a_bone() {
        // The structural invariant the whole rotation-space design rests
        // on, asserted on the editor's own output: whatever a user drags,
        // the result is still a set of local rotations, and a rotation
        // cannot change a bone length.
        let mut edit = PoseEdit::from_pose(&poses::relaxed_stand());

        // Something deliberately extreme.
        edit.set_rotation(
            Bone::LeftArm,
            EditableRotation { axis: Vec3::new(1.0, 1.0, 1.0), degrees: 170.0 },
        );
        edit.set_rotation(Bone::Spine, EditableRotation { axis: Vec3::X, degrees: -95.0 });

        let positions = forward_kinematics(&edit.to_pose());

        for &bone in Bone::ALL.iter() {
            let Some(parent) = bone.parent() else { continue };
            let rest = bone.t_pose_offset().length();
            let posed = (positions[bone] - positions[parent]).length();

            assert!(
                (posed - rest).abs() < 1.0e-5,
                "editing stretched {} to {posed} m against a rest length of {rest} m",
                bone.name(),
            );
        }
    }
}
