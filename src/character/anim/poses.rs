//! The named poses, in rotation space.
//!
//! These are **derived, not re-authored**. Each one traces back to real
//! Mixamo `idle.glb` reference data, dumped by
//! `tools/dump_animation_pose.py` into the superseded position-space
//! `muscle::pose` tables, then converted into rotation space by
//! [`super::convert`]. Re-eyeballing them in a new representation would
//! discard that provenance — and this project's own history is that
//! hand-guessed offsets ship wrong (two poses once landed limbs 59-77% off
//! the rig's real bone lengths, past a green test suite).
//!
//! Conversion fidelity was measured rather than assumed: for
//! `relaxed_stand` every controllable bone direction converted to within
//! **0.03 degrees** (see [`super::convert::direction_error_degrees`]).
//!
//! # Why the data lives in `assets/anim/`, not in this file
//!
//! The conversion used to run at *startup*, reading `muscle::pose` and
//! converting every time the program launched. That made a deleted module
//! load-bearing for a pose's values, and it recomputed a fixed answer on
//! every run.
//!
//! The converted result is now frozen into `assets/anim/*.pose.ron` — the
//! same files the asset loader hot-reloads — and embedded here with
//! [`include_str!`]. One representation, one set of numbers, checked into
//! the repo where they can be read and diffed. Editing a pose file changes
//! both the hot-reload path and these fallbacks, so the two can never
//! silently disagree.
//!
//! The functions below are the compiled-in fallback used when no asset
//! handle is present (and the fixtures the invariant tests run against);
//! [`super::asset`] is the live path.

use super::asset::PoseAsset;
use super::rig::LocalPose;

/// The rig's own bind pose: every bone at its rest orientation.
///
/// Note this is the *rig's* rest pose, not necessarily a T-pose on a
/// retargeted mesh — the whole point of the rig-independent representation
/// is that identity means "however this particular skeleton was bound".
pub fn rest() -> LocalPose {
    LocalPose::REST
}

/// A relaxed standing pose: arms hanging naturally at the sides, spine
/// settled, head level, knees softly bent.
///
/// The upper body came from real Mixamo idle reference data. The legs come
/// from [`super::stance`], because the source pose leaves them dead
/// straight — which is what a bind pose is, and which makes foot IK
/// unsolvable (see that module for the measurements). The knee flex is
/// already baked into the stored file.
///
/// This is the baseline every other standing pose should build on, not
/// [`rest`]: on this rig that is a T-pose, and it reads as a mannequin.
pub fn relaxed_stand() -> LocalPose {
    embedded("relaxed_stand", include_str!("../../../assets/anim/relaxed_stand.pose.ron"))
}

/// A standing pose imported straight from `assets/models/idle.glb`.
///
/// The first pose produced by the full reference pipeline —
/// `tools/dump_animation_pose.py` reads frame 0 of the clip,
/// `examples/import_reference_pose` converts its world positions into
/// rotations. Its provenance is *reproducible*, which `relaxed_stand`'s is
/// not: the source positions are committed beside it as
/// `idle_stand.positions.ron`, so the pose can be regenerated rather than
/// merely trusted.
///
/// It converts at a worst direction error of **0.028 degrees**, and its
/// `LeftArm` lands within 0.005 degrees of `relaxed_stand`'s — which was
/// derived from this same clip through the superseded position-space
/// module. Two independent paths agreeing to that tolerance is the best
/// evidence available that both are right.
///
/// Note the legs are as the clip authored them: straight. See
/// [`super::stance`] for why a standing pose wants a knee bend, and why
/// `relaxed_stand` adds one.
pub fn idle_stand() -> LocalPose {
    embedded("idle_stand", include_str!("../../../assets/anim/idle_stand.pose.ron"))
}

/// A raised right arm, mid-wave.
///
/// Deliberately **asymmetric** — it must never be added to the symmetry
/// invariant test.
pub fn wave() -> LocalPose {
    embedded("wave", include_str!("../../../assets/anim/wave.pose.ron"))
}

/// Parses one of the compiled-in pose files.
///
/// Panics on malformed data, deliberately. These strings are embedded at
/// compile time from files in this repo, so a failure here is a broken
/// checked-in asset, not bad user input — and every caller returns a
/// `LocalPose` that the whole rig depends on. Surfacing it as an `Option`
/// would push a "what do I do with `None`?" decision onto call sites for a
/// condition none of them could sensibly recover from, and the invariant
/// tests below parse all of them on every test run.
fn embedded(name: &str, source: &str) -> LocalPose {
    ron::from_str::<PoseAsset>(source)
        .unwrap_or_else(|error| panic!("assets/anim/{name}.pose.ron is malformed: {error}"))
        .to_local_pose()
        .unwrap_or_else(|error| panic!("assets/anim/{name}.pose.ron names a bone the rig does not have: {error}"))
}

/// Every pose that is expected to be left/right symmetric.
///
/// Manually maintained rather than auto-discovered, deliberately: a pose
/// added here that is not actually symmetric fails loudly, and a pose
/// omitted by accident has to be caught in review. The project's own
/// verification rules call this out explicitly.
///
/// # Why `relaxed_stand` is NOT in this list
///
/// The superseded module documented it as symmetric, and its own symmetry
/// test passed — but that test only compared left/right bone *pairs*, so
/// nothing ever checked the central chain. `relaxed_stand` carries up to
/// **1.3 cm of sideways offset** on `Spine1`/`Spine2`/`Neck`/`Head`,
/// producing a measurable lean (a ~0.039 z-component on `Spine`'s
/// converted rotation).
///
/// That asymmetry is not a defect: the pose was derived from a real Mixamo
/// idle frame, and a real person standing at rest is never perfectly
/// mirrored — the superseded module's own idle research called deliberate
/// left/right asymmetry "the single biggest tell of a robotic idle". The
/// mistake was only in calling the pose symmetric.
///
/// [`wave`] is also correctly absent — it is asymmetric by design.
pub fn symmetric_named_poses() -> Vec<(&'static str, LocalPose)> {
    vec![("rest", rest())]
}

/// Every named pose, symmetric or not.
///
/// Also manually maintained; see [`symmetric_named_poses`].
pub fn all_named_poses() -> Vec<(&'static str, LocalPose)> {
    vec![
        ("rest", rest()),
        ("relaxed_stand", relaxed_stand()),
        ("idle_stand", idle_stand()),
        ("wave", wave()),
    ]
}

/// Looks a pose up by name, for CLI flags and the Phase 8 studio.
pub fn by_name(name: &str) -> Option<LocalPose> {
    all_named_poses()
        .into_iter()
        .find(|(pose_name, _)| *pose_name == name)
        .map(|(_, pose)| pose)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::convert::{mirror_bone, mirrored};
    use crate::character::anim::math::quat_ext::neighborhood;
    use crate::character::anim::rig::forward_kinematics;
    use crate::character::skeleton::Bone;
    use bevy::math::{Quat, Vec3};

    /// Pins `relaxed_stand` against the reference data it was derived from.
    ///
    /// This is the test the Phase 7 deletion made necessary. The pose used
    /// to be *computed* at startup from the superseded position-space
    /// module's tables, so its provenance was enforced by construction: if
    /// the source data said the left hand sits at a particular point, the
    /// pose put it there. Freezing the conversion into a file turned those
    /// rotations into bare literals that nothing checks — exactly the
    /// situation this project's own history warns about, where hand-edited
    /// pose numbers shipped 59-77% wrong past a green suite.
    ///
    /// So the source positions are asserted directly, in world space, at
    /// the tolerance the conversion was originally accepted at. The legs
    /// are excluded: the stored pose deliberately adds the [`super::stance`]
    /// knee flex that the straight-legged source lacks (see
    /// [`relaxed_stand`]), and the shoulders are excluded because rotation
    /// space provably cannot reproduce their authored displacement (see
    /// [`super::convert`], which measures it at ~11.5 cm).
    #[test]
    fn the_imported_idle_agrees_with_the_pose_derived_from_the_same_clip() {
        // The strongest evidence available that the reference pipeline is
        // correct: `relaxed_stand` and `idle_stand` come from the SAME
        // Mixamo clip by completely independent routes — the first through
        // the superseded position-space module, the second through
        // `dump_animation_pose.py` plus `import_reference_pose`.
        //
        // Two independent derivations agreeing is worth far more than
        // either matching a number I chose. It also makes this a live
        // regression test on the dump tool's coordinate conversion, which
        // took three corrections to get right: Z-up to Y-up, then the
        // forward-axis SIGN (a backwards head), then the X mirror (Mixamo
        // puts the character's left on +X, this crate on -X).
        //
        // The spine chain is compared rather than the whole rig: the legs
        // legitimately differ, because `relaxed_stand` adds the `stance`
        // knee bend the source clip does not have.
        let derived = relaxed_stand();
        let imported = idle_stand();

        // 12 degrees, not near-zero. The two are NOT expected to match
        // exactly: `relaxed_stand` came from the superseded module's
        // hand-corrected tables rather than from raw clip data, and its
        // own doc records that it carries a deliberate ~1.3 cm sideways
        // offset on the spine chain from that editing. What this asserts
        // is that they describe the same *posture* — a bound loose enough
        // to allow the known editing, and far tighter than any of the
        // three coordinate bugs produced (those ran to 41 degrees on the
        // neck and mirrored the arms outright).
        //
        // Not the `Neck`: `relaxed_stand`'s was re-solved on 2026-09-29 for
        // a level gaze (the clip's bowed the head 30.5 degrees at the floor),
        // so it now departs from the clip by design. The spine chain still
        // carries the comparison.
        for bone in [Bone::Spine, Bone::Spine1, Bone::Spine2] {
            let a = derived.rotation(bone);
            let b = neighborhood(a, imported.rotation(bone));

            let degrees = a.angle_between(b).to_degrees();
            assert!(
                degrees < 12.0,
                "{} differs by {degrees:.2} degrees between the two derivations of the \
                 same clip frame — one of the two pipelines has drifted",
                bone.name(),
            );
        }

        // The arms are the sensitive case: a mirrored import swaps them,
        // which the spine (nearly symmetric) would not reveal.
        for (bone, counterpart) in
            [(Bone::LeftArm, Bone::RightArm), (Bone::RightArm, Bone::LeftArm)]
        {
            let derived_rotation = derived.rotation(bone);
            let same_side = derived_rotation
                .angle_between(neighborhood(derived_rotation, imported.rotation(bone)))
                .to_degrees();
            let other_side = derived_rotation
                .angle_between(neighborhood(derived_rotation, imported.rotation(counterpart)))
                .to_degrees();

            assert!(
                same_side < other_side,
                "{} matches the imported {} ({other_side:.1} deg) better than its own \
                 ({same_side:.1} deg) — the import is mirrored",
                bone.name(),
                counterpart.name(),
            );
        }
    }

    #[test]
    fn relaxed_stand_still_matches_its_reference_data() {
        // The exact world positions the Mixamo-derived source authored,
        // via `tools/dump_animation_pose.py`. Carried over verbatim when
        // the position-space module was deleted.
        let reference = [
            (Bone::Spine, Vec3::new(0.0, 1.11, 0.0)),
            (Bone::Spine1, Vec3::new(-0.0132, 1.2793, 0.013)),
            (Bone::Spine2, Vec3::new(-0.0132, 1.435, 0.0498)),
            (Bone::Neck, Vec3::new(-0.0064, 1.5331, 0.0681)),
            // The clip's `Head` (0.0112, 1.7387, -0.0545) is deliberately no
            // longer matched: its neck bowed the head 30.5 degrees toward the
            // floor, and the neck angle was solved for a level gaze instead.
            // See `the_relaxed_stand_stands_upright_and_balanced`.
            (Bone::LeftArm, Vec3::new(-0.20900002, 1.5811, 0.0412)),
            (Bone::LeftForeArm, Vec3::new(-0.24670005, 1.3039, 0.0293)),
            (Bone::LeftHand, Vec3::new(-0.27730006, 1.0461999, 0.0133)),
            (Bone::RightArm, Vec3::new(0.20900002, 1.5811, 0.0412)),
            (Bone::RightForeArm, Vec3::new(0.24670005, 1.3039, 0.0293)),
            (Bone::RightHand, Vec3::new(0.27730006, 1.0461999, 0.0133)),
        ];

        let positions = forward_kinematics(&relaxed_stand());

        for (bone, expected) in reference {
            let actual = positions[bone];
            // The arms inherit the shoulders' unrepresentable displacement,
            // so they are checked at the looser bound `convert`'s own tests
            // measured; the spine chain converts essentially exactly.
            let tolerance = if matches!(
                bone,
                Bone::LeftArm
                    | Bone::LeftForeArm
                    | Bone::LeftHand
                    | Bone::RightArm
                    | Bone::RightForeArm
                    | Bone::RightHand
            ) {
                0.15
            } else {
                0.01
            };

            assert!(
                actual.distance(expected) < tolerance,
                "relaxed_stand's {} has drifted from its reference data: authored at \
                 {expected:?}, now at {actual:?} ({:.4} m away, tolerance {tolerance})",
                bone.name(),
                actual.distance(expected),
            );
        }
    }

    // `cargo test --release -- --ignored --nocapture probe_spine_profile`.
    #[test]
    #[ignore]
    fn probe_spine_profile() {
        use crate::character::anim::gltf_rig::puppet_base_as_rendered;
        use crate::character::anim::rig::offset_from;
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = puppet_base_as_rendered();
        let forward = rig.forward();
        let stood = stance_on_rig(&relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let chain = [Bone::Hips, Bone::Spine, Bone::Spine1, Bone::Spine2, Bone::Neck, Bone::Head];
        for (name, pose) in [("bind", LocalPose::REST), ("stand", stood)] {
            let at = |bone| offset_from(&pose, &rig, Bone::Hips, bone);
            let mut line = format!("{name:5}:");
            for pair in chain.windows(2) {
                let v = at(pair[1]) - at(pair[0]);
                line += &format!(" {}→{} {:+.1}° ({:.0} mm)", pair[0].name(), pair[1].name(), v.dot(forward).atan2(v.y).to_degrees(), v.length() * 1e3);
            }
            let sockets = (at(Bone::LeftUpLeg) + at(Bone::RightUpLeg)) * 0.5;
            let v = at(Bone::Spine) - sockets;
            line += &format!(" | sockets→Spine {:+.1}°", v.dot(forward).atan2(v.y).to_degrees());
            println!("{line}");
        }
    }

    #[test]
    fn the_relaxed_stand_stands_upright_and_balanced() {
        // Measured on the rig as RENDERED (`puppet_base_as_rendered`): this
        // pose's rotations are fixed world axes authored facing -Z, and on
        // the unturned fixture they raise the arms overhead.
        //
        // - Balance: in quiet standing the centre of pressure averages under
        //   the centre of mass, about 4 cm ahead of the ankle joints (Winter,
        //   Example 5.1: the ground reaction of a static stance 4 cm
        //   anterior to the ankle). Measured 4.8 cm; the bind, 4.2.
        // - Gaze level: the clip bowed the head 30.5 degrees at the floor.
        // - Trunk upright: the hips-to-shoulders line within a few degrees of
        //   the bind's own.
        // - Arms at the sides: hands below the shoulders and beside the hips.
        use crate::character::anim::anthropometry::centre_of_mass;
        use crate::character::anim::gltf_rig::puppet_base_as_rendered;
        use crate::character::anim::rig::{accumulate_world_rotations, offset_from};
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        let rig = puppet_base_as_rendered();
        let pose = stance_on_rig(&relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let forward = rig.forward();
        let at = |bone| offset_from(&pose, &rig, Bone::Hips, bone);
        let mid = |a, b| (at(a) + at(b)) * 0.5;
        let ankles = mid(Bone::LeftFoot, Bone::RightFoot);

        let com_ahead = (centre_of_mass(&pose, &rig) - ankles).dot(forward);
        assert!((0.02..0.07).contains(&com_ahead), "centre of mass {com_ahead:.3} m ahead of the ankles");

        let gaze = {
            let now = accumulate_world_rotations(&pose, &rig)[Bone::Head];
            let bind = accumulate_world_rotations(&LocalPose::REST, &rig)[Bone::Head];
            let v = (now * bind.inverse()) * forward;
            v.y.atan2(v.dot(forward)).to_degrees()
        };
        assert!(gaze.abs() < 1.0, "the head looks {gaze:+.1} degrees off level");

        let lean = |p: &LocalPose| {
            let at = |bone| offset_from(p, &rig, Bone::Hips, bone);
            let v = (at(Bone::LeftArm) + at(Bone::RightArm) - at(Bone::LeftUpLeg) - at(Bone::RightUpLeg)) * 0.5;
            v.dot(forward).atan2(v.y).to_degrees()
        };
        let trunk = lean(&pose) - lean(&LocalPose::REST);
        assert!(trunk.abs() < 5.0, "the trunk leans {trunk:+.1} degrees off the bind's");

        let shoulders = mid(Bone::LeftArm, Bone::RightArm);
        for hand in [Bone::LeftHand, Bone::RightHand] {
            let h = at(hand);
            assert!(shoulders.y - h.y > 0.4, "{} is only {:.2} m below the shoulders", hand.name(), shoulders.y - h.y);
            assert!((h - mid(Bone::LeftUpLeg, Bone::RightUpLeg)).dot(forward).abs() < 0.08, "{} hangs ahead or behind", hand.name());
        }
    }

    /// Every named pose must still load through the *asset* path.
    ///
    /// The example gets its poses from the asset server, not from these
    /// functions, so a file that no longer parses — or that names a bone
    /// the rig dropped — would break the running program while every
    /// pose-data test here still passed.
    ///
    /// This deliberately does not compare the result against the
    /// compiled-in pose: both sides would run the same
    /// `PoseAsset::to_local_pose`, so any bug in it cancels out and the
    /// comparison proves nothing (verified by inverting that function and
    /// watching the comparison still pass). What is asserted instead is
    /// that loading *succeeds* and yields a pose that is actually posed.
    #[test]
    fn every_named_pose_file_still_loads_through_the_asset_path() {
        use crate::character::anim::asset::PoseAsset;

        for (name, _) in all_named_poses() {
            let path = format!("assets/anim/{name}.pose.ron");
            let source = std::fs::read_to_string(&path)
                .unwrap_or_else(|error| panic!("{path} could not be read: {error}"));

            let loaded = ron::from_str::<PoseAsset>(&source)
                .unwrap_or_else(|error| panic!("{path} is malformed: {error}"))
                .to_local_pose()
                .unwrap_or_else(|error| panic!("{path} names an unknown bone: {error}"));

            // `rest` IS the identity pose, so an all-identity result is
            // correct there and only there. For every other pose it means
            // the file's contents never reached the rig.
            let is_posed = Bone::ALL
                .iter()
                .any(|&bone| !loaded.rotation(bone).abs_diff_eq(Quat::IDENTITY, 1.0e-6));

            assert_eq!(
                is_posed,
                name != "rest",
                "{path} loaded, but its posed-ness is wrong: every bone at rest is correct \
                 only for the rest pose",
            );
        }
    }

    #[test]
    fn every_named_pose_preserves_every_bone_length_exactly() {
        // The structural invariant that replaces the superseded module's
        // own bone-length test. There it needed a 0.02 m tolerance and had
        // caught real 59-77% overstretches; here a rotation CANNOT change a
        // bone length, so the tolerance is float precision and the failure
        // mode is impossible by construction.
        //
        // Kept as a test anyway, because it is exactly the guard that would
        // catch a future translation channel being added carelessly.
        for (name, pose) in all_named_poses() {
            let positions = forward_kinematics(&pose);

            for &bone in Bone::ALL.iter() {
                let Some(parent) = bone.parent() else { continue };

                let rest_length = bone.t_pose_offset().length();
                let posed_length = (positions[bone] - positions[parent]).length();

                assert!(
                    (posed_length - rest_length).abs() < 1.0e-5,
                    "in pose '{name}', {} sits {posed_length} m from {} but its rest \
                     length is {rest_length} m",
                    bone.name(),
                    parent.name(),
                );
            }
        }
    }

    #[test]
    fn every_symmetric_named_pose_mirrors_left_and_right_exactly() {
        // The second mandatory invariant. Mirroring is derived from bone
        // NAMES (see `convert::mirror_bone`) so it cannot drift out of step
        // with `Bone::ALL`.
        for (name, pose) in symmetric_named_poses() {
            let flipped = mirrored(&pose);

            for &bone in Bone::ALL.iter() {
                let original = pose.rotation(bone);
                let mirrored_rotation = neighborhood(original, flipped.rotation(bone));

                assert!(
                    original.abs_diff_eq(mirrored_rotation, 1.0e-4),
                    "pose '{name}' is documented symmetric, but {} does not mirror its \
                     counterpart {}: {original:?} vs {mirrored_rotation:?}",
                    bone.name(),
                    mirror_bone(bone).name(),
                );
            }
        }
    }

    #[test]
    fn the_wave_pose_is_asymmetric_and_must_stay_out_of_the_symmetry_list() {
        // Guards the symmetry list itself: if someone adds `wave` to it,
        // that test would fail confusingly. This says why, up front.
        let wave = wave();
        let flipped = mirrored(&wave);

        let differs = Bone::ALL.iter().any(|&bone| {
            let original = wave.rotation(bone);
            !original.abs_diff_eq(neighborhood(original, flipped.rotation(bone)), 1.0e-3)
        });

        assert!(
            differs,
            "wave() is asymmetric by design (it raises one arm); if it has become \
             symmetric, the symmetry list should be updated deliberately",
        );

        assert!(
            !symmetric_named_poses().iter().any(|(name, _)| *name == "wave"),
            "wave() must not appear in the symmetric-pose list",
        );
    }

    #[test]
    fn relaxed_stand_is_deliberately_slightly_asymmetric() {
        // Pins the finding that led to `relaxed_stand` being removed from
        // the symmetry list: its source is a real mocap idle frame, and a
        // person standing at rest is never perfectly mirrored. Documented
        // here as intent so it cannot be quietly "fixed" into a mannequin.
        //
        // The asymmetry must stay SMALL, though — a lean big enough to read
        // as a deliberate weight shift would be a different pose.
        let pose = relaxed_stand();
        let flipped = mirrored(&pose);

        let mut largest = 0.0f32;
        for &bone in Bone::ALL.iter() {
            let original = pose.rotation(bone);
            let mirrored_rotation = neighborhood(original, flipped.rotation(bone));
            largest = largest.max(original.angle_between(mirrored_rotation));
        }

        assert!(
            largest > 1.0e-3,
            "relaxed_stand should carry the slight asymmetry of its mocap source, but \
             mirrors exactly — if it has been re-authored symmetric, move it back into \
             symmetric_named_poses()",
        );
        assert!(
            largest < 0.2,
            "the asymmetry should be subtle, but the worst bone differs from its mirror \
             by {largest} rad",
        );
    }

    #[test]
    fn every_standing_pose_hangs_both_arms_below_the_shoulders() {
        // A specific, falsifiable claim about the pose's shape — the sort of
        // thing that a bone-length test cannot catch but a viewer notices
        // instantly. Stated as "is the hand below the shoulder", not "does
        // this look better than before".
        //
        // Applied to EVERY standing pose, not just the reference-derived
        // one. A hand-authored pose has no reference data to be pinned
        // against, so these structural claims are the only thing standing
        // between it and a limb somewhere impossible.
        for (name, pose) in standing_poses() {
            let positions = forward_kinematics(&pose);

            for (shoulder, hand) in
                [(Bone::LeftShoulder, Bone::LeftHand), (Bone::RightShoulder, Bone::RightHand)]
            {
                assert!(
                    positions[hand].y < positions[shoulder].y - 0.2,
                    "in '{name}', {} should hang well below {}, but sat at y={} against \
                     a shoulder at y={}",
                    hand.name(),
                    shoulder.name(),
                    positions[hand].y,
                    positions[shoulder].y,
                );
            }
        }
    }

    /// Every pose that depicts a character standing at rest.
    ///
    /// Manually listed for the same reason the other arrays are: a pose
    /// added to the rig and forgotten here silently loses its shape
    /// coverage, and that has to fail in review rather than in a
    /// screenshot months later.
    fn standing_poses() -> Vec<(&'static str, LocalPose)> {
        vec![("relaxed_stand", relaxed_stand()), ("idle_stand", idle_stand())]
    }

    #[test]
    fn no_standing_pose_crosses_a_hand_to_the_opposite_side() {
        // Ported from the superseded module's own highest-value regression:
        // a real, screenshot-caught bug where retargeting swept each arm
        // diagonally across the chest. The geometric definition is what
        // caught it — screenshots alone had passed a still-broken fix.
        for (name, pose) in standing_poses() {
            let positions = forward_kinematics(&pose);

            assert!(
                positions[Bone::LeftHand].x < 0.0,
                "in '{name}', the left hand must stay on the character's own left \
                 (negative X), got x={}",
                positions[Bone::LeftHand].x,
            );
            assert!(
                positions[Bone::RightHand].x > 0.0,
                "in '{name}', the right hand must stay on the character's own right \
                 (positive X), got x={}",
                positions[Bone::RightHand].x,
            );
        }
    }

    #[test]
    fn the_wave_pose_raises_the_right_hand_above_the_head() {
        let positions = forward_kinematics(&wave());

        assert!(
            positions[Bone::RightHand].y > positions[Bone::Head].y * 0.85,
            "a wave should bring the right hand up near head height, got hand y={} \
             against head y={}",
            positions[Bone::RightHand].y,
            positions[Bone::Head].y,
        );
        // `wave_pose` touches ONLY the right arm chain, so the left arm
        // stays wherever the base pose put it — and its base is the T-pose,
        // meaning straight out horizontally, not down. Asserting "down" here
        // would be asserting a pose this data never described.
        assert!(
            (positions[Bone::LeftHand].y - positions[Bone::LeftShoulder].y).abs() < 0.15,
            "the left arm should remain at its T-pose height, got hand y={} against \
             shoulder y={}",
            positions[Bone::LeftHand].y,
            positions[Bone::LeftShoulder].y,
        );
        assert!(
            positions[Bone::LeftHand].x < -0.5,
            "and should still extend out to the character's left, got x={}",
            positions[Bone::LeftHand].x,
        );
    }

    #[test]
    fn the_rest_pose_is_all_identity() {
        for &bone in Bone::ALL.iter() {
            assert_eq!(
                rest().rotation(bone),
                Quat::IDENTITY,
                "{} should be identity in the rest pose",
                bone.name(),
            );
        }
    }

    #[test]
    fn poses_can_be_looked_up_by_name() {
        assert!(by_name("relaxed_stand").is_some());
        assert!(by_name("wave").is_some());
        assert!(by_name("rest").is_some());
        assert!(by_name("nonsense").is_none());
    }

    /// Writes the converted poses out as RON, so `assets/anim/` carries the
    /// same reference-data-derived values the code does.
    ///
    /// A test rather than a build script: it runs on demand
    /// (`cargo test --release --lib export_named_poses_to_ron -- --ignored`),
    /// keeps the conversion in one place, and cannot silently rewrite
    /// authored files during a normal test run — once a pose has been
    /// hand-tuned in the studio, regenerating it would destroy that work.
    #[test]
    #[ignore = "writes to assets/anim/; run deliberately, not on every test run"]
    fn export_named_poses_to_ron() {
        use crate::character::anim::asset::PoseAsset;

        let directory = std::path::Path::new("assets/anim");
        std::fs::create_dir_all(directory).expect("should create assets/anim");

        for (name, pose) in all_named_poses() {
            let asset = PoseAsset::from_local_pose(&pose);
            let serialized = ron::ser::to_string_pretty(
                &asset,
                ron::ser::PrettyConfig::new().struct_names(false),
            )
            .expect("should serialize");

            let path = directory.join(format!("{name}.pose.ron"));
            std::fs::write(&path, serialized).expect("should write the pose file");
            eprintln!("wrote {}", path.display());
        }
    }

    #[test]
    fn report_relaxed_stand_shape() {
        // A reporting probe, not an assertion: prints the authored shape so
        // it can be compared against what the real rig renders.
        let p = forward_kinematics(&relaxed_stand());
        eprintln!("\n=== relaxed_stand, synthetic rig world positions ===");
        for b in [
            Bone::LeftShoulder,
            Bone::LeftArm,
            Bone::LeftForeArm,
            Bone::LeftHand,
            Bone::Head,
        ] {
            eprintln!("  {:<14} {:?}", b.name(), p[b]);
        }
        eprintln!(
            "  hand is {:.3} m below shoulder",
            p[Bone::LeftShoulder].y - p[Bone::LeftHand].y
        );
        eprintln!("=== authored local rotation deltas ===");
        for b in [Bone::LeftShoulder, Bone::LeftArm, Bone::LeftForeArm] {
            let (axis, angle) = relaxed_stand().rotation(b).to_axis_angle();
            eprintln!("  {:<14} axis {axis:?} angle {:.1} deg", b.name(), angle.to_degrees());
        }
    }

    #[test]
    fn every_named_pose_appears_in_the_lookup_table() {
        // Guards the manual lists against drift: anything nameable must be
        // findable.
        for (name, _) in all_named_poses() {
            assert!(by_name(name).is_some(), "'{name}' is listed but not resolvable");
        }
    }
}
