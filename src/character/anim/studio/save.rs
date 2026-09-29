//! Writing an edited pose back out as RON, and reading one in.
//!
//! Kept apart from [`super::edit`] because file I/O is the one thing here
//! that can fail for reasons having nothing to do with the pose — a missing
//! directory, a read-only file — and those failures need to reach the user
//! as a message rather than a panic.

use std::path::{Path, PathBuf};

use crate::character::anim::asset::PoseAsset;
use crate::character::anim::rig::LocalPose;

use super::edit::PoseEdit;

/// Why a save or load did not happen.
#[derive(Debug)]
pub enum PoseFileError {
    /// The file could not be read or written.
    Io(std::io::Error),
    /// The file is not valid RON, or does not describe a pose.
    Malformed(String),
    /// The file names a bone this rig does not have.
    UnknownBone(String),
    /// A save was asked for with no path to save to.
    NoPath,
}

impl std::fmt::Display for PoseFileError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "{error}"),
            Self::Malformed(detail) => write!(f, "not a valid pose file: {detail}"),
            Self::UnknownBone(detail) => write!(f, "{detail}"),
            Self::NoPath => write!(f, "no file name — use Save As"),
        }
    }
}

impl std::error::Error for PoseFileError {}

/// Where pose files live, relative to the working directory.
pub const POSE_DIRECTORY: &str = "assets/anim";

/// The file a named pose is stored in.
pub fn pose_path(name: &str) -> PathBuf {
    Path::new(POSE_DIRECTORY).join(format!("{name}.pose.ron"))
}

/// Serializes `edit` as RON, in the same shape the loader reads.
///
/// Round-tripping through [`PoseAsset`] rather than writing the text by
/// hand is deliberate: it means the editor cannot emit a file the runtime
/// loader would reject, because it is producing the very type the loader
/// produces.
pub fn to_ron(edit: &PoseEdit) -> Result<String, PoseFileError> {
    let asset = PoseAsset::from_local_pose(&edit.to_pose());

    ron::ser::to_string_pretty(&asset, ron::ser::PrettyConfig::new().struct_names(false))
        .map_err(|error| PoseFileError::Malformed(error.to_string()))
}

/// Parses a pose file's text.
pub fn from_ron(source: &str) -> Result<LocalPose, PoseFileError> {
    ron::from_str::<PoseAsset>(source)
        .map_err(|error| PoseFileError::Malformed(error.to_string()))?
        .to_local_pose()
        .map_err(|error| PoseFileError::UnknownBone(error.to_string()))
}

/// Writes `edit` to its own source file.
///
/// Creates [`POSE_DIRECTORY`] if it is missing, so a fresh checkout does not
/// fail the first save.
pub fn save(edit: &PoseEdit) -> Result<PathBuf, PoseFileError> {
    let path = edit.source.clone().ok_or(PoseFileError::NoPath)?;
    save_as(edit, &path)
}

/// Writes `edit` to `path`.
pub fn save_as(edit: &PoseEdit, path: &str) -> Result<PathBuf, PoseFileError> {
    let text = to_ron(edit)?;
    let path = PathBuf::from(path);

    if let Some(directory) = path.parent() {
        std::fs::create_dir_all(directory).map_err(PoseFileError::Io)?;
    }

    std::fs::write(&path, text).map_err(PoseFileError::Io)?;
    Ok(path)
}

/// Reads a pose file into an editable pose.
pub fn load(path: &str) -> Result<PoseEdit, PoseFileError> {
    let text = std::fs::read_to_string(path).map_err(PoseFileError::Io)?;
    let pose = from_ron(&text)?;
    Ok(PoseEdit::from_file(&pose, path))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::poses;
    use crate::character::anim::studio::edit::EditableRotation;
    use crate::character::skeleton::Bone;
    use bevy::math::Vec3;

    #[test]
    fn a_pose_survives_a_full_text_round_trip() {
        // The property the whole save path rests on: what the editor writes
        // is what the loader reads. If this drifts, a user's saved work
        // renders differently from what they saw while editing.
        let original = poses::relaxed_stand();
        let edit = PoseEdit::from_pose(&original);

        let text = to_ron(&edit).expect("should serialize");
        let parsed = from_ron(&text).expect("should parse what it just wrote");

        for &bone in Bone::ALL.iter() {
            assert!(
                original.rotation(bone).abs_diff_eq(parsed.rotation(bone), 1.0e-5),
                "{} changed across a save/load round trip: {:?} -> {:?}",
                bone.name(),
                original.rotation(bone),
                parsed.rotation(bone),
            );
        }
    }

    #[test]
    fn an_edit_survives_the_round_trip_too() {
        // Not just authored data — a pose the user actually changed.
        let mut edit = PoseEdit::from_pose(&poses::rest());
        edit.set_rotation(
            Bone::LeftForeArm,
            EditableRotation { axis: Vec3::new(0.2, -0.7, 0.4).normalize(), degrees: 63.5 },
        );

        let expected = edit.to_pose();
        let text = to_ron(&edit).expect("should serialize");
        let parsed = from_ron(&text).expect("should parse");

        assert!(
            expected
                .rotation(Bone::LeftForeArm)
                .abs_diff_eq(parsed.rotation(Bone::LeftForeArm), 1.0e-5),
            "an edited bone should survive saving: {:?} -> {:?}",
            expected.rotation(Bone::LeftForeArm),
            parsed.rotation(Bone::LeftForeArm),
        );
    }

    #[test]
    fn a_saved_file_is_sparse() {
        // Bones at rest are omitted, keeping files readable and diffable.
        // A 22-bone file of mostly identities would make a one-bone change
        // invisible in review.
        let mut edit = PoseEdit::from_pose(&poses::rest());
        edit.set_rotation(Bone::LeftArm, EditableRotation { axis: Vec3::Z, degrees: 30.0 });

        let text = to_ron(&edit).expect("should serialize");

        assert!(text.contains("LeftArm"), "the edited bone must appear");
        assert!(
            !text.contains("RightForeArm"),
            "bones left at rest should be omitted, but the file mentions RightForeArm:\n{text}",
        );
    }

    #[test]
    fn a_malformed_file_is_reported_rather_than_panicking() {
        // The editor must survive being pointed at a broken file — this is
        // exactly the situation hand-editing a pose file produces.
        let error = from_ron("this is not ron at all").expect_err("should fail");
        assert!(matches!(error, PoseFileError::Malformed(_)), "got {error:?}");
    }

    #[test]
    fn a_file_naming_an_unknown_bone_says_so() {
        // The specific, actionable failure: a typo in a bone name would
        // otherwise silently do nothing, leaving the author wondering why
        // their edit had no effect.
        let source = r#"(bones: {"LeftArmm": (swing: (axis: (0.0, 0.0, 1.0), degrees: 30.0))})"#;
        let error = from_ron(source).expect_err("should fail");

        assert!(matches!(error, PoseFileError::UnknownBone(_)), "got {error:?}");
        assert!(
            error.to_string().contains("LeftArmm"),
            "the message should name the offending bone, got: {error}",
        );
    }

    #[test]
    fn saving_without_a_path_is_an_error_not_a_guess() {
        // Better than inventing a filename: a pose that came from nowhere
        // has no obvious place to go, and silently picking one loses work.
        let edit = PoseEdit::from_pose(&poses::rest());
        let error = save(&edit).expect_err("a pose with no source has nowhere to save");
        assert!(matches!(error, PoseFileError::NoPath));
    }

    #[test]
    fn pose_paths_land_in_the_asset_directory() {
        let path = pose_path("relaxed_stand");
        assert_eq!(path, Path::new("assets/anim/relaxed_stand.pose.ron"));
    }

    #[test]
    fn the_dump_tool_writes_files_the_shipped_loader_accepts() {
        // `tools/dump_animation_pose.py --ron` hand-writes RON rather than
        // going through `PoseAsset`, since it runs inside Blender's Python
        // and cannot link this crate. That makes its output format a
        // standing assumption which nothing else would catch drifting —
        // a pose file that Blender emits and the engine rejects fails at
        // the worst possible moment, hours into authoring.
        //
        // This checks a real dump of `assets/models/idle.glb` frame 0,
        // committed alongside the tool, through the loader the runtime
        // actually uses.
        let source = include_str!("../../../../assets/anim/idle_stand.pose.ron");
        let pose = from_ron(source).expect("the loader must accept the dump tool's output");

        // And it must describe a real pose, not parse into an empty one —
        // a file of all-identity rotations would parse fine and mean
        // nothing.
        let posed = Bone::ALL
            .iter()
            .filter(|&&bone| {
                !pose.rotation(bone).abs_diff_eq(bevy::math::Quat::IDENTITY, 1.0e-4)
            })
            .count();

        assert!(
            posed > 10,
            "a dumped idle frame should pose most of the rig, got {posed} bones",
        );
    }

    #[test]
    fn a_dumped_pose_is_in_this_crates_coordinate_space() {
        // Blender is Z-up; this crate is Y-up. A dump that kept Blender's
        // axes would parse cleanly and render a character lying on its
        // face — plausible numbers, wrong rig.
        //
        // Forward kinematics puts the head above the hips if and only if
        // the conversion happened.
        use crate::character::anim::rig::forward_kinematics;

        let source = include_str!("../../../../assets/anim/idle_stand.pose.ron");
        let pose = from_ron(source).expect("should parse");
        let positions = forward_kinematics(&pose);

        assert!(
            positions[Bone::Head].y > positions[Bone::Hips].y + 0.5,
            "a dumped idle should stand upright in Y-up space, but the head sits at \
             y={} against hips at y={} — the Z-up conversion is missing or wrong",
            positions[Bone::Head].y,
            positions[Bone::Hips].y,
        );

        // Both feet below the hips, for the same reason: a Z-up dump would
        // scatter them sideways.
        for foot in [Bone::LeftFoot, Bone::RightFoot] {
            assert!(
                positions[foot].y < positions[Bone::Hips].y,
                "{} should sit below the hips, got y={}",
                foot.name(),
                positions[foot].y,
            );
        }

        // The strong check. "Head above hips" turns out NOT to discriminate
        // — a Y/Z-swapped pose still stands upright, because most of this
        // idle's rotations are small and the rig's own offsets carry the
        // height regardless. Measured, not assumed.
        //
        // What does discriminate is the limbs. An idle hangs its arms down
        // and forward-facing; swapping Y and Z swings them sideways,
        // because that is the axis the swap actually exchanges.
        let mut unconverted = pose;
        for &bone in Bone::ALL.iter() {
            let rotation = pose.rotation(bone);
            unconverted.set_rotation(
                bone,
                bevy::math::Quat::from_xyzw(rotation.x, rotation.z, rotation.y, rotation.w),
            );
        }
        let scrambled = forward_kinematics(&unconverted);

        let hand_drop = positions[Bone::LeftShoulder].y - positions[Bone::LeftHand].y;
        let scrambled_drop =
            scrambled[Bone::LeftShoulder].y - scrambled[Bone::LeftHand].y;

        assert!(
            hand_drop > 0.35,
            "a dumped idle hangs its arms down: the hand should sit well below the \
             shoulder, got {hand_drop:.3} m",
        );
        assert!(
            scrambled_drop < hand_drop * 0.8,
            "test intent: a Y/Z-swapped pose must NOT hang its arms the same way \
             ({scrambled_drop:.3} m against the converted {hand_drop:.3} m), or this \
             test would pass on an unconverted dump",
        );
    }

    #[test]
    fn the_editor_writes_files_the_shipped_loader_accepts() {
        // Guards the editor against the runtime drifting away from it: the
        // saved text must parse through `PoseAsset`, the same type
        // `AnimAssetPlugin` loads at runtime, not merely through something
        // shaped like it.
        let edit = PoseEdit::from_pose(&poses::wave());
        let text = to_ron(&edit).expect("should serialize");

        let asset: PoseAsset =
            ron::from_str(&text).expect("the runtime loader's own type must parse this");
        assert!(
            asset.to_local_pose().is_ok(),
            "the runtime loader must accept every file the editor writes",
        );
    }
}
