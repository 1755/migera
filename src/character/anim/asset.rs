//! Poses and clips as hot-reloadable RON assets.
//!
//! # Why authored data belongs in files
//!
//! Poses started life as hardcoded `Vec3` tables in Rust. That works, but
//! it puts a recompile between every tweak and seeing the result — which
//! for work that is judged by eye (does this idle read as a person or a
//! mannequin?) is the difference between iterating for an hour and
//! iterating for a day.
//!
//! With `file_watcher` already enabled, saving a `.anim.ron` updates a
//! running character immediately. That is the loop the Phase 8 studio is
//! built on, and it works from a text editor today.
//!
//! # The format
//!
//! Sparse by design — a bone you do not mention stays at rest:
//!
//! ```ron
//! (
//!     bones: {
//!         "LeftArm":     (swing: (axis: (0.0, 0.0, 1.0), degrees: 71.0)),
//!         "LeftForeArm": (swing: (axis: (0.0, 0.0, 1.0), degrees: 12.0), twist_degrees: 90.0),
//!     },
//!     root_translation: (0.0, 0.0, 0.0),
//! )
//! ```
//!
//! Rotations are authored as **axis + degrees**, not raw quaternion
//! components. `(x, y, z, w)` is unreadable and unwritable by hand — nobody
//! can look at `(0.0, 0.0, 0.581, 0.814)` and see "71 degrees about Z" — and
//! the point of moving to files is that a person can edit them.
//!
//! Bones are named with the same PascalCase strings as [`Bone::name`], so
//! an asset is checked against the rig rather than against array positions
//! that could silently shift.

use std::collections::HashMap;

use bevy::asset::io::Reader;
use bevy::asset::{Asset, AssetApp, AssetLoader, LoadContext};
use bevy::math::{Quat, Vec3};
use bevy::reflect::TypePath;
use serde::{Deserialize, Serialize};

use super::rig::LocalPose;
use crate::character::skeleton::Bone;

/// A rotation, authored the way a person would write one.
///
/// Defaults to no rotation, so a bone that only twists need not spell out
/// an identity swing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AuthoredRotation {
    /// Axis to turn about, in the bone's own rest frame. Normalized on
    /// load, so it need not be exact in the file.
    pub axis: (f32, f32, f32),
    /// How far to turn, in degrees — the unit the authoring actually
    /// happens in.
    pub degrees: f32,
}

impl Default for AuthoredRotation {
    /// No rotation. The axis is `+Y` rather than zero so a file that sets
    /// only `degrees` still describes a usable rotation.
    fn default() -> Self {
        Self { axis: (0.0, 1.0, 0.0), degrees: 0.0 }
    }
}

impl AuthoredRotation {
    /// The quaternion this describes.
    ///
    /// A zero or degenerate axis yields the identity rotation rather than a
    /// NaN, so a half-finished file cannot poison the rig.
    pub fn to_quat(self) -> Quat {
        let axis = Vec3::new(self.axis.0, self.axis.1, self.axis.2);
        let axis = axis.normalize_or_zero();

        if axis == Vec3::ZERO || self.degrees == 0.0 {
            Quat::IDENTITY
        } else {
            Quat::from_axis_angle(axis, self.degrees.to_radians())
        }
    }

    /// Describes `rotation` in authorable form. Used by the studio's save
    /// path so round-tripping a file does not rewrite it into quaternions.
    pub fn from_quat(rotation: Quat) -> Self {
        let (axis, angle) = rotation.to_axis_angle();
        Self { axis: (axis.x, axis.y, axis.z), degrees: angle.to_degrees() }
    }
}

/// What one bone does in a pose.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AuthoredBone {
    /// Where the bone points, relative to its rest direction.
    ///
    /// Deliberately not an `Option`: RON 0.8 requires an explicit
    /// `Some(...)` wrapper around optional values, which would make every
    /// line of every pose file noisier for no gain. A zero-degree swing
    /// already means "unrotated", so the default carries the same
    /// information with none of the syntax.
    pub swing: AuthoredRotation,
    /// Roll about the bone's own axis, in degrees.
    ///
    /// Kept separate in the *file* even though the runtime folds it into
    /// one quaternion, because the two are conceptually different to an
    /// author: swing is "where does the arm point", twist is "which way is
    /// the palm". Merging them in the file would make both harder to edit.
    pub twist_degrees: f32,
}

/// A single authored pose.
#[derive(Asset, TypePath, Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct PoseAsset {
    /// Per-bone rotations, keyed by [`Bone::name`]. Sparse: an unlisted
    /// bone stays at rest.
    pub bones: HashMap<String, AuthoredBone>,
    /// Root displacement, metres.
    pub root_translation: (f32, f32, f32),
}

/// Why a pose asset could not be turned into a [`LocalPose`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PoseAssetError {
    /// A bone name in the file does not exist on the rig.
    ///
    /// Reported rather than ignored: a typo like `"LeftArmm"` would
    /// otherwise silently do nothing, and the author would be left
    /// wondering why their edit had no effect.
    UnknownBone(String),
}

impl std::fmt::Display for PoseAssetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownBone(name) => write!(
                f,
                "'{name}' is not a bone on this rig — check the spelling against \
                 Bone::name (PascalCase, e.g. \"LeftForeArm\")"
            ),
        }
    }
}

impl std::error::Error for PoseAssetError {}

impl PoseAsset {
    /// Builds the runtime pose this file describes.
    ///
    /// Swing and twist are composed in that order, matching
    /// [`super::convert::pose_from_world_positions_with_twist`]: roll turns
    /// about the already-aimed direction, which is what "palm up" means.
    pub fn to_local_pose(&self) -> Result<LocalPose, PoseAssetError> {
        let mut pose = LocalPose::REST;

        for (name, authored) in &self.bones {
            let bone = Bone::from_name(name)
                .ok_or_else(|| PoseAssetError::UnknownBone(name.clone()))?;

            let swing = authored.swing.to_quat();

            let rotation = if authored.twist_degrees == 0.0 {
                swing
            } else {
                let axis = twist_axis(bone);
                swing * Quat::from_axis_angle(axis, authored.twist_degrees.to_radians())
            };

            pose.set_rotation(bone, rotation);
        }

        pose.root_translation =
            Vec3::new(self.root_translation.0, self.root_translation.1, self.root_translation.2);

        Ok(pose)
    }

    /// Describes `pose` as an authorable asset — the inverse of
    /// [`Self::to_local_pose`], used by the studio's save path.
    ///
    /// Bones at rest are omitted, keeping saved files sparse and readable
    /// rather than listing all 22 bones with identity rotations.
    ///
    /// Note this does not attempt to split a rotation back into swing and
    /// twist: that decomposition is not unique, and guessing would rewrite
    /// an author's deliberate choice. A round-tripped file carries the
    /// whole rotation as `swing`.
    pub fn from_local_pose(pose: &LocalPose) -> Self {
        let mut bones = HashMap::new();

        for &bone in Bone::ALL.iter() {
            let rotation = pose.rotation(bone);
            if rotation.abs_diff_eq(Quat::IDENTITY, 1.0e-6) {
                continue;
            }

            bones.insert(
                bone.name().to_string(),
                AuthoredBone {
                    swing: AuthoredRotation::from_quat(rotation),
                    twist_degrees: 0.0,
                },
            );
        }

        Self {
            bones,
            root_translation: (
                pose.root_translation.x,
                pose.root_translation.y,
                pose.root_translation.z,
            ),
        }
    }
}

/// The axis a bone rolls about: the direction it points its driving child
/// in the rest pose.
fn twist_axis(bone: Bone) -> Vec3 {
    // Mirrors `convert`'s own rule — a multi-child parent is oriented by
    // its continuation child, so that is the axis it rolls about.
    let child = bone.chain_continuation_child().or_else(|| {
        let mut only_child = None;
        for &candidate in Bone::ALL.iter() {
            if candidate.parent() == Some(bone) {
                if only_child.is_some() {
                    return None;
                }
                only_child = Some(candidate);
            }
        }
        only_child
    });

    match child {
        Some(child) => {
            let axis = child.t_pose_offset().normalize_or_zero();
            if axis == Vec3::ZERO { Vec3::Y } else { axis }
        }
        // A leaf bone has no child to point at; roll about its own offset
        // from its parent, which is the only axis it has.
        None => {
            let axis = bone.t_pose_offset().normalize_or_zero();
            if axis == Vec3::ZERO { Vec3::Y } else { axis }
        }
    }
}

/// Loads `.pose.ron` files.
#[derive(Default, TypePath)]
pub struct PoseAssetLoader;

impl AssetLoader for PoseAssetLoader {
    type Asset = PoseAsset;
    type Settings = ();
    type Error = PoseLoadError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _load_context: &mut LoadContext<'_>,
    ) -> Result<PoseAsset, PoseLoadError> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;

        let asset: PoseAsset = ron::de::from_bytes(&bytes)?;

        // Validate eagerly, at load time, so a typo surfaces as a loud
        // asset error rather than a bone that silently never moves.
        asset.to_local_pose()?;

        Ok(asset)
    }

    fn extensions(&self) -> &[&str] {
        &["pose.ron"]
    }
}

/// Everything that can go wrong loading a pose file.
///
/// Hand-written rather than derived: `thiserror` is only an indirect
/// dependency here, and one error enum does not justify making it a direct
/// one.
#[derive(Debug)]
pub enum PoseLoadError {
    /// The file could not be read.
    Io(std::io::Error),
    /// The file is not valid RON, or does not match the pose schema.
    Ron(ron::error::SpannedError),
    /// The file parsed but describes something this rig does not have.
    Invalid(PoseAssetError),
}

impl std::fmt::Display for PoseLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(error) => write!(f, "could not read the pose file: {error}"),
            Self::Ron(error) => write!(f, "could not parse the pose file: {error}"),
            Self::Invalid(error) => {
                write!(f, "the pose file refers to something not on this rig: {error}")
            }
        }
    }
}

impl std::error::Error for PoseLoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io(error) => Some(error),
            Self::Ron(error) => Some(error),
            Self::Invalid(error) => Some(error),
        }
    }
}

impl From<std::io::Error> for PoseLoadError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

impl From<ron::error::SpannedError> for PoseLoadError {
    fn from(error: ron::error::SpannedError) -> Self {
        Self::Ron(error)
    }
}

impl From<PoseAssetError> for PoseLoadError {
    fn from(error: PoseAssetError) -> Self {
        Self::Invalid(error)
    }
}

/// Registers the pose asset type and its loader.
///
/// Separate from `AnimPlugin` so a consumer that builds poses in code (or a
/// headless test) never pays for the asset machinery.
#[derive(Debug, Default, Clone, Copy)]
pub struct AnimAssetPlugin;

impl bevy::app::Plugin for AnimAssetPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_asset::<PoseAsset>().init_asset_loader::<PoseAssetLoader>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::forward_kinematics;
    use std::f32::consts::FRAC_PI_2;

    fn parse(source: &str) -> PoseAsset {
        ron::de::from_str(source).expect("test asset should parse")
    }

    #[test]
    fn a_minimal_pose_file_parses_and_leaves_every_other_bone_at_rest() {
        let asset = parse(
            r#"(
                bones: {
                    "LeftArm": (swing: (axis: (0.0, 0.0, 1.0), degrees: 90.0)),
                },
            )"#,
        );

        let pose = asset.to_local_pose().expect("should convert");

        assert!(
            pose.rotation(Bone::LeftArm)
                .abs_diff_eq(Quat::from_axis_angle(Vec3::Z, FRAC_PI_2), 1.0e-5),
            "the authored 90-degree swing should appear on LeftArm",
        );

        for &bone in Bone::ALL.iter() {
            if bone == Bone::LeftArm {
                continue;
            }
            assert_eq!(
                pose.rotation(bone),
                Quat::IDENTITY,
                "{} was not mentioned and must stay at rest",
                bone.name(),
            );
        }
    }

    #[test]
    fn an_empty_pose_file_is_the_rest_pose() {
        let pose = parse("()").to_local_pose().expect("should convert");

        for &bone in Bone::ALL.iter() {
            assert_eq!(pose.rotation(bone), Quat::IDENTITY);
        }
        assert_eq!(pose.root_translation, Vec3::ZERO);
    }

    #[test]
    fn an_unknown_bone_name_is_reported_rather_than_ignored() {
        // A typo must not silently do nothing — that is the worst possible
        // authoring experience, because the file looks right.
        let asset = parse(
            r#"(bones: { "LeftArmm": (swing: (axis: (0.0, 0.0, 1.0), degrees: 10.0)) })"#,
        );

        match asset.to_local_pose() {
            Err(PoseAssetError::UnknownBone(name)) => assert_eq!(name, "LeftArmm"),
            other => panic!("expected an UnknownBone error, got {other:?}"),
        }
    }

    #[test]
    fn the_error_message_names_the_offending_bone_and_how_to_fix_it() {
        let message = PoseAssetError::UnknownBone("leftarm".into()).to_string();
        assert!(message.contains("leftarm"), "the message should quote the bad name");
        assert!(
            message.contains("PascalCase"),
            "and should say what the right form looks like, got: {message}",
        );
    }

    #[test]
    fn a_twist_is_applied_about_the_bones_own_axis() {
        let pose = parse(
            r#"(bones: { "LeftForeArm": (twist_degrees: 90.0) })"#,
        )
        .to_local_pose()
        .expect("should convert");

        let angle = pose.rotation(Bone::LeftForeArm).angle_between(Quat::IDENTITY);
        assert!(
            (angle - FRAC_PI_2).abs() < 1.0e-4,
            "expected a 90-degree roll, got {} degrees",
            angle.to_degrees(),
        );

        // Roll must not move the bone it points at.
        let positions = forward_kinematics(&pose);
        let rest = forward_kinematics(&LocalPose::REST);
        assert!(
            (positions[Bone::LeftHand] - rest[Bone::LeftHand]).length() < 1.0e-5,
            "a pure roll must leave the child where it was",
        );
    }

    #[test]
    fn swing_and_twist_compose_in_that_order() {
        // Roll turns about the ALREADY-AIMED direction; reversing the order
        // would roll about the rest direction instead, which is a different
        // and wrong pose.
        let asset = parse(
            r#"(bones: {
                "LeftArm": (swing: (axis: (0.0, 0.0, 1.0), degrees: 90.0), twist_degrees: 45.0),
            })"#,
        );

        let pose = asset.to_local_pose().expect("should convert");

        let swing = Quat::from_axis_angle(Vec3::Z, FRAC_PI_2);
        let twist = Quat::from_axis_angle(
            Bone::LeftForeArm.t_pose_offset().normalize(),
            45f32.to_radians(),
        );

        assert!(
            pose.rotation(Bone::LeftArm).abs_diff_eq(swing * twist, 1.0e-5),
            "expected swing * twist, got {:?}",
            pose.rotation(Bone::LeftArm),
        );
    }

    #[test]
    fn a_degenerate_axis_yields_identity_rather_than_nan() {
        // A half-written file must not poison the rig with NaN.
        let pose = parse(
            r#"(bones: { "LeftArm": (swing: (axis: (0.0, 0.0, 0.0), degrees: 90.0)) })"#,
        )
        .to_local_pose()
        .expect("should convert");

        assert_eq!(
            pose.rotation(Bone::LeftArm),
            Quat::IDENTITY,
            "a zero axis should be inert, not NaN",
        );
    }

    #[test]
    fn an_unnormalized_axis_is_normalized_on_load() {
        // Authors should not have to compute unit vectors by hand.
        let pose = parse(
            r#"(bones: { "LeftArm": (swing: (axis: (0.0, 0.0, 7.5), degrees: 90.0)) })"#,
        )
        .to_local_pose()
        .expect("should convert");

        assert!(
            pose.rotation(Bone::LeftArm)
                .abs_diff_eq(Quat::from_axis_angle(Vec3::Z, FRAC_PI_2), 1.0e-5),
            "a long axis should behave exactly like a unit one",
        );
    }

    #[test]
    fn a_pose_round_trips_through_the_asset_format() {
        let mut original = LocalPose::REST;
        original.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Z, 1.24));
        original.set_rotation(Bone::Spine, Quat::from_axis_angle(Vec3::X, -0.4));
        original.root_translation = Vec3::new(0.1, 0.0, -2.0);

        let serialized = ron::ser::to_string(&PoseAsset::from_local_pose(&original))
            .expect("should serialize");
        let restored: PoseAsset = ron::de::from_str(&serialized).expect("should parse");
        let restored = restored.to_local_pose().expect("should convert");

        for &bone in Bone::ALL.iter() {
            let mismatch = 1.0 - original.rotation(bone).dot(restored.rotation(bone)).abs();
            assert!(
                mismatch < 1.0e-6,
                "{} did not survive the round trip (mismatch {mismatch})",
                bone.name(),
            );
        }
        assert!(
            (restored.root_translation - original.root_translation).length() < 1.0e-6,
            "the root translation should survive too",
        );
    }

    #[test]
    fn saving_a_pose_omits_every_bone_left_at_rest() {
        // Keeps authored files sparse and readable instead of listing all
        // 22 bones with identity rotations.
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Head, Quat::from_axis_angle(Vec3::Y, 0.3));

        let asset = PoseAsset::from_local_pose(&pose);

        assert_eq!(asset.bones.len(), 1, "only the moved bone should be written");
        assert!(asset.bones.contains_key("Head"));
    }

    #[test]
    fn every_shipped_pose_file_parses_and_matches_its_compiled_in_pose() {
        // The shipped RON files are what the game actually loads, so they
        // are the fixtures worth testing — not just the in-memory structs.
        //
        // Also pins them against the compiled-in poses they were exported
        // from, so the two cannot drift apart silently: editing a file by
        // hand without re-exporting (or vice versa) fails here rather than
        // showing up as a mysterious visual difference later.
        for (name, expected) in crate::character::anim::poses::all_named_poses() {
            let path = format!("assets/anim/{name}.pose.ron");
            let source = std::fs::read_to_string(&path)
                .unwrap_or_else(|error| panic!("{path} should exist: {error}"));

            let asset: PoseAsset = ron::de::from_str(&source)
                .unwrap_or_else(|error| panic!("{path} should parse: {error}"));
            let loaded = asset
                .to_local_pose()
                .unwrap_or_else(|error| panic!("{path} should be valid: {error}"));

            for &bone in Bone::ALL.iter() {
                let mismatch =
                    1.0 - loaded.rotation(bone).dot(expected.rotation(bone)).abs();
                assert!(
                    mismatch < 1.0e-5,
                    "{path}: {} differs from the compiled-in '{name}' pose (mismatch \
                     {mismatch}) — re-run the export test if the pose changed",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn the_loader_accepts_the_pose_ron_extension() {
        use bevy::asset::AssetLoader;
        assert_eq!(PoseAssetLoader.extensions(), &["pose.ron"]);
    }
}
