//! Builds a [`RigGeometry`] by parsing a real glTF, for tests.
//!
//! # Why this exists
//!
//! Tests that measure anything about the WORLD — a foot's height, a leg's
//! reach, a bone's orientation — need the rig the game actually ships, and
//! this crate's synthetic T-pose is not it. Two gaps, both of which have
//! produced live bugs that every test passed through:
//!
//! - **Proportions.** The synthetic rig leaves `LeftLeg -> LeftFoot` as a
//!   0.07 m stub where `puppet_base.gltf` has a 0.459 m shin, so 42 degrees
//!   of knee flexion moves the sole 0.050 m there and the ankle 0.329 m on
//!   the real rig. A correct walk renders with a visibly straight leg.
//! - **Bind rotations.** The synthetic rig binds every bone at identity;
//!   the real one binds its foot at **-69.8 degrees**. A foot that sits
//!   flat in the test is pitched in the game.
//!
//! The previous answer was a hand-transcribed helper — constants read out of
//! the glTF once and typed into a test. That closes the gap on the day it is
//! written and drifts silently afterwards, which is the defect this replaces.
//!
//! # Why parsing is cheap here
//!
//! A `.gltf` keeps its node hierarchy in plain JSON and only its mesh data
//! in the companion `.bin`. `puppet_base.gltf` is 31 KB of JSON with 69
//! nodes, every one carrying its rotation and translation — so the whole
//! bind pose is readable without touching Bevy, the asset server, or a
//! GPU. `serde_json` is already in the tree via `bevy_gltf`.
//!
//! # What this does and does not verify
//!
//! It reproduces the **bind pose**: offsets, bind rotations, and the root
//! correction. [`real_skeleton`] turns that into a real
//! [`HumanoidSkeleton`], which is what lets `retarget`'s own tests exercise
//! the retargeting path — `for_other_rig`, `hips_local_translation_for`
//! including its parent-scale division, `delta_in_bone_frame`'s conjugation,
//! and `write_pose_to_skeleton` through a headless `World` with
//! `MinimalPlugins` + `TransformPlugin`.
//!
//! That path was previously described here as needing the live app. It does
//! not: of the five `HumanoidSkeleton` methods it uses, only
//! [`HumanoidSkeleton::entity`] touches the ECS at all, and a spawned entity
//! per bone is cheap. The claim conflated "needs a Bevy `Query`" with "needs
//! the running game".
//!
//! What genuinely still needs the live app is the **glTF loader**: these
//! parse the asset's JSON directly, so they verify the maths against the
//! bind pose the FILE declares, not that `bevy_gltf` reproduces that bind
//! pose when it loads it.

use std::collections::HashMap;

use bevy::math::{Quat, Vec3};

use super::rig::{BoneSet, RigGeometry};
use crate::character::skeleton::Bone;

/// The UE-mannequin naming convention `puppet_base.gltf` follows.
///
/// Deliberately the same table `character_gallery`'s own
/// `UE_MANNEQUIN_BONE_NAMES` uses. Keeping a second copy is a real cost —
/// but the alternative is exporting the example's table into the library,
/// and a name mapping is example-layer policy rather than library
/// behaviour. `the_gallery_and_this_module_agree_about_bone_names` pins
/// them together instead.
const BONE_NAMES: [(Bone, &str); 22] = [
    (Bone::Hips, "pelvis"),
    (Bone::Spine, "spine_01"),
    (Bone::Spine1, "spine_02"),
    (Bone::Spine2, "spine_03"),
    (Bone::Neck, "neck_01"),
    (Bone::Head, "Head"),
    (Bone::LeftShoulder, "clavicle_l"),
    (Bone::LeftArm, "upperarm_l"),
    (Bone::LeftForeArm, "lowerarm_l"),
    (Bone::LeftHand, "hand_l"),
    (Bone::RightShoulder, "clavicle_r"),
    (Bone::RightArm, "upperarm_r"),
    (Bone::RightForeArm, "lowerarm_r"),
    (Bone::RightHand, "hand_r"),
    (Bone::LeftUpLeg, "thigh_l"),
    (Bone::LeftLeg, "calf_l"),
    (Bone::LeftFoot, "foot_l"),
    (Bone::LeftToeBase, "ball_l"),
    (Bone::RightUpLeg, "thigh_r"),
    (Bone::RightLeg, "calf_r"),
    (Bone::RightFoot, "foot_r"),
    (Bone::RightToeBase, "ball_r"),
];

/// One glTF node, reduced to what a bind pose needs.
struct Node {
    name: String,
    translation: Vec3,
    rotation: Quat,
    scale: Vec3,
    children: Vec<usize>,
}

/// `puppet_base.gltf`, embedded at compile time.
///
/// `include_str!` rather than a runtime read so the test does not depend on
/// the working directory, matching how `poses.rs` embeds its own assets.
const PUPPET_BASE: &str = include_str!("../../../assets/models/puppet_base.gltf");

/// The real rig's bind pose, as a [`RigGeometry`].
///
/// Parsed from [`PUPPET_BASE`] every call. That is a few hundred
/// microseconds and keeps the helper free of shared mutable state; if it
/// ever shows up in a profile, cache it behind a `OnceLock`.
pub fn puppet_base() -> RigGeometry {
    from_gltf(PUPPET_BASE).expect("puppet_base.gltf should parse")
}

/// [`puppet_base`] as the character is actually rendered: turned half round
/// about +Y, the asset's facing correction.
///
/// `puppet_base.gltf` faces +Z, this crate's poses are authored facing −Z,
/// and the renderer turns the asset round. Poses authored as fixed
/// world-axis rotations — `relaxed_stand`'s arms and spine, anything
/// imported from a clip — therefore mean different things on the two rigs:
/// forward kinematics on [`puppet_base`] put `relaxed_stand`'s hands a metre
/// ABOVE the hips, where the live character hangs them at its sides. On this
/// rig the same pose lands where the renderer draws it, pinned to the live
/// character over BRP by `the_rendered_rig_matches_the_live_character`.
///
/// Anything written against the rig's own forward (`RigGeometry::forward`,
/// `stance::facing_sign`) — the gait, the feet, the stance — is the same on
/// both. Use this one for a question about a pose's rendered SHAPE.
pub fn puppet_base_as_rendered() -> RigGeometry {
    let mut rig = puppet_base();
    rig.root_rotation = Quat::from_rotation_y(std::f32::consts::PI) * rig.root_rotation;
    rig
}

/// Everything a [`HumanoidSkeleton`] needs about a parsed rig, except the
/// ECS entities.
///
/// Split out from [`real_skeleton`] so a test can vary one field — a
/// non-unit parent scale, say — without rebuilding the parse.
#[derive(Clone)]
pub struct ParsedRig {
    /// Each bone's own local bind rotation.
    pub rest_rotations: HashMap<Bone, Quat>,
    /// Each bone's own rest direction, unit length.
    pub rest_directions: HashMap<Bone, Vec3>,
    /// `Hips`' own local translation within its parent.
    pub hips_rest_local_translation: Vec3,
    /// The accumulated world rotation of everything above `Hips`.
    pub hips_parent_rest_world_rotation: Quat,
    /// The accumulated world scale of everything above `Hips`.
    pub hips_parent_rest_world_scale: Vec3,
}

/// Parses the bind-pose data a real skeleton is built from.
pub fn parsed_rig() -> ParsedRig {
    parse_rig(PUPPET_BASE).expect("puppet_base.gltf should parse")
}

/// [`parsed_rig`], reporting failure instead of panicking.
pub fn parse_rig(source: &str) -> Result<ParsedRig, String> {
    let json: serde_json::Value =
        serde_json::from_str(source).map_err(|e| format!("not valid JSON: {e}"))?;

    let nodes = parse_nodes(&json)?;
    let by_name: HashMap<&str, usize> =
        nodes.iter().enumerate().map(|(i, n)| (n.name.as_str(), i)).collect();

    let mut parent_of: HashMap<usize, usize> = HashMap::new();
    for (index, node) in nodes.iter().enumerate() {
        for &child in &node.children {
            parent_of.insert(child, index);
        }
    }

    let mut rest_rotations = HashMap::new();
    let mut rest_directions = HashMap::new();

    for &(bone, name) in BONE_NAMES.iter() {
        let index = *by_name
            .get(name)
            .ok_or_else(|| format!("{} maps to '{name}', which is not in the file", bone.name()))?;

        rest_rotations.insert(bone, nodes[index].rotation);
        rest_directions.insert(bone, nodes[index].translation.normalize_or_zero());
    }

    let hips = *by_name.get("pelvis").ok_or_else(|| "no 'pelvis' node".to_string())?;

    Ok(ParsedRig {
        rest_rotations,
        rest_directions,
        hips_rest_local_translation: nodes[hips].translation,
        hips_parent_rest_world_rotation: accumulated_rotation_above(&nodes, &parent_of, hips),
        // `puppet_base` has no meaningful non-unit scale — its whole chain
        // is 1.0 — so this is the identity case for that rig. The scale
        // division exists for assets that DO carry one (Blender's
        // FBX-cm-to-glTF-m 0.01 correction node is the documented case), and
        // a test wanting to exercise it should override this field.
        hips_parent_rest_world_scale: accumulated_scale_above(&nodes, &parent_of, hips),
    })
}

/// A [`HumanoidSkeleton`] carrying the real rig's bind pose.
///
/// `entities` supplies the ECS entity per bone — a caller with a real
/// `World` passes its spawned ones; a caller testing only the maths can pass
/// placeholders, since every method except
/// [`HumanoidSkeleton::entity`] reads the bind-pose data rather than the
/// entity map.
pub fn real_skeleton(
    rig: &ParsedRig,
    entities: HashMap<Bone, bevy::prelude::Entity>,
) -> crate::character::skeleton::HumanoidSkeleton {
    crate::character::skeleton::HumanoidSkeleton::for_other_rig(
        entities,
        rig.rest_rotations.clone(),
        rig.rest_directions.clone(),
        rig.hips_rest_local_translation,
        rig.hips_parent_rest_world_rotation,
        rig.hips_parent_rest_world_scale,
        // The ANIMATION's own rest reference, deliberately this crate's
        // synthetic T-pose position rather than the glTF's own scene-space
        // one — a solved root translation is always expressed in the
        // synthetic convention whatever rig is being driven. The gallery
        // passes the same thing; see its own note.
        Bone::Hips.t_pose_world_position(),
    )
}

/// The product of every scale from the scene root down to, but not
/// including, `index`.
fn accumulated_scale_above(
    nodes: &[Node],
    parent_of: &HashMap<usize, usize>,
    index: usize,
) -> Vec3 {
    let mut chain = Vec::new();
    let mut cursor = index;
    while let Some(&parent) = parent_of.get(&cursor) {
        chain.push(parent);
        cursor = parent;
    }

    chain.iter().rev().fold(Vec3::ONE, |acc, &a| acc * nodes[a].scale)
}

/// The real rig's leg LENGTHS on an otherwise-synthetic rig.
///
/// Most tests want exactly this combination: leg segments that are real, so
/// knee flexion is visible (the synthetic rig's 0.07 m stub makes it 6x less
/// so), posed in this crate's own upright convention, so `y` reads as height
/// without unpicking a Z-up correction first.
///
/// Lengths come from the parsed asset rather than transcribed, so they
/// cannot drift. Directions stay synthetic deliberately: a glTF offset is a
/// vector in its own rig's frame, and only its magnitude is portable.
///
/// For anything about ORIENTATION use [`puppet_base`], which is the whole
/// asset including its bind rotations.
pub fn real_leg_lengths() -> RigGeometry {
    let real = puppet_base();
    let mut rig = RigGeometry::default();

    for (up, leg, foot, side) in [
        (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, -1.0),
        (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, 1.0),
    ] {
        // The hip socket keeps its synthetic placement — it is a lateral
        // offset rather than a leg segment, and nothing measures it.
        rig.offsets[up] = Vec3::new(0.11 * side, -0.11, 0.0);
        rig.offsets[leg] = Vec3::new(0.0, -real.offsets[leg].length(), 0.0);
        rig.offsets[foot] = Vec3::new(0.0, -real.offsets[foot].length(), 0.0);
    }

    rig
}

/// [`puppet_base`], but reporting failure instead of panicking.
///
/// Separate so the parser's own error paths are testable without a broken
/// asset checked in.
pub fn from_gltf(source: &str) -> Result<RigGeometry, String> {
    let json: serde_json::Value =
        serde_json::from_str(source).map_err(|e| format!("not valid JSON: {e}"))?;

    let nodes = parse_nodes(&json)?;

    let by_name: HashMap<&str, usize> =
        nodes.iter().enumerate().map(|(i, n)| (n.name.as_str(), i)).collect();

    // Every node's parent, so a bone can be walked back to the scene root.
    let mut parent_of: HashMap<usize, usize> = HashMap::new();
    for (index, node) in nodes.iter().enumerate() {
        for &child in &node.children {
            parent_of.insert(child, index);
        }
    }

    let index_of = |bone: Bone| -> Result<usize, String> {
        let name = BONE_NAMES
            .iter()
            .find(|(b, _)| *b == bone)
            .map(|(_, n)| *n)
            .ok_or_else(|| format!("{} has no glTF name mapping", bone.name()))?;

        by_name
            .get(name)
            .copied()
            .ok_or_else(|| format!("{} maps to '{name}', which is not in the file", bone.name()))
    };

    // Each bone's own local transform IS its offset and bind rotation: a
    // glTF node's translation is relative to its parent, which is exactly
    // what `RigGeometry` wants.
    let mut offsets = BoneSet::splat(Vec3::ZERO);
    let mut bind_rotations = BoneSet::splat(Quat::IDENTITY);

    for &(bone, _) in BONE_NAMES.iter() {
        let index = index_of(bone)?;
        offsets[bone] = nodes[index].translation;
        bind_rotations[bone] = nodes[index].rotation;
    }

    // The root correction: everything above `Hips`, accumulated.
    //
    // This is what turns the rig's own authoring convention (Z-up here)
    // into this crate's (Y-up). Leaving it at identity puts the foot at
    // y = +1.68 — above the hips — because the ~164-degree thigh bind that
    // carries the correction is then uncompensated.
    let hips = index_of(Bone::Hips)?;
    let root_rotation = accumulated_rotation_above(&nodes, &parent_of, hips);

    let mut rig = RigGeometry { offsets, bind_rotations, root_rotation, ..Default::default() };

    // The toe tip, measured rather than estimated: this rig HAS the joint
    // (`ball_leaf_l`), which is the case `RigGeometry::with_toe_end` exists
    // for.
    for (toe, leaf) in
        [(Bone::LeftToeBase, "ball_leaf_l"), (Bone::RightToeBase, "ball_leaf_r")]
    {
        if let Some(&index) = by_name.get(leaf) {
            rig = rig.with_toe_end(toe, nodes[index].translation);
        }
    }

    Ok(rig)
}

/// The product of every rotation from the scene root down to, but not
/// including, `index`.
fn accumulated_rotation_above(
    nodes: &[Node],
    parent_of: &HashMap<usize, usize>,
    index: usize,
) -> Quat {
    // Walk up collecting ancestors, then compose downward — a rotation
    // chain applies parent-first, so the order matters.
    let mut chain = Vec::new();
    let mut cursor = index;
    while let Some(&parent) = parent_of.get(&cursor) {
        chain.push(parent);
        cursor = parent;
    }

    let mut accumulated = Quat::IDENTITY;
    for &ancestor in chain.iter().rev() {
        accumulated *= nodes[ancestor].rotation;
    }

    accumulated
}

/// Reads the `nodes` array.
fn parse_nodes(json: &serde_json::Value) -> Result<Vec<Node>, String> {
    let raw = json
        .get("nodes")
        .and_then(|n| n.as_array())
        .ok_or_else(|| "no `nodes` array".to_string())?;

    raw.iter()
        .map(|node| {
            Ok(Node {
                name: node
                    .get("name")
                    .and_then(|n| n.as_str())
                    .unwrap_or_default()
                    .to_string(),
                translation: read_vec3(node.get("translation")).unwrap_or(Vec3::ZERO),
                rotation: read_quat(node.get("rotation")).unwrap_or(Quat::IDENTITY),
                scale: read_vec3(node.get("scale")).unwrap_or(Vec3::ONE),
                children: node
                    .get("children")
                    .and_then(|c| c.as_array())
                    .map(|c| c.iter().filter_map(|i| i.as_u64().map(|i| i as usize)).collect())
                    .unwrap_or_default(),
            })
        })
        .collect()
}

fn read_vec3(value: Option<&serde_json::Value>) -> Option<Vec3> {
    let array = value?.as_array()?;
    if array.len() != 3 {
        return None;
    }
    Some(Vec3::new(
        array[0].as_f64()? as f32,
        array[1].as_f64()? as f32,
        array[2].as_f64()? as f32,
    ))
}

/// glTF stores a quaternion as `[x, y, z, w]`, which is also
/// [`Quat::from_xyzw`]'s order.
fn read_quat(value: Option<&serde_json::Value>) -> Option<Quat> {
    let array = value?.as_array()?;
    if array.len() != 4 {
        return None;
    }
    Some(Quat::from_xyzw(
        array[0].as_f64()? as f32,
        array[1].as_f64()? as f32,
        array[2].as_f64()? as f32,
        array[3].as_f64()? as f32,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::{forward_kinematics_on, LocalPose};

    #[test]
    fn the_rendered_rig_matches_the_live_character() {
        // Measured over BRP on `character_gallery --anim-speed 0`, standing
        // in `relaxed_stand` with the gallery's knee bend, 2026-09-29:
        // heights above `pelvis` of `upperarm_l`, `lowerarm_l`, `hand_l`.
        // On `puppet_base` without the facing turn, forward kinematics put
        // the same joints at +0.546, +0.795, +1.036: arms raised overhead.
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = puppet_base_as_rendered();
        let pose = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let k = forward_kinematics_on(&pose, &rig);
        for (bone, live) in [(Bone::LeftArm, 0.456), (Bone::LeftForeArm, 0.207), (Bone::LeftHand, -0.034)] {
            let height = k[bone].y - k[Bone::Hips].y;
            assert!((height - live).abs() < 0.005, "{}: {height:.3} against {live} live", bone.name());
        }
    }

    #[test]
    fn the_estimated_toe_tip_is_a_poor_stand_in_for_the_measured_one() {
        // Why `solve_foot_ik` measures the tip off the live rig instead of
        // letting `RigGeometry::from_skeleton` estimate it.
        //
        // [`from_gltf`] MEASURES the tip from the rig's own `ball_leaf_l`.
        // `from_skeleton` can only ESTIMATE it — a fraction of the ankle-to-toe
        // offset — because `HumanoidSkeleton` has no toe-end concept
        // (deliberately: a 23rd bone would invalidate every `[T; 22]`,
        // `Bone::ALL`, and every RON asset, for a point that is never
        // rendered).
        //
        // The estimate is ~27 degrees off on this rig, and the consequence was
        // not subtle. `legik::lift_toe_end_out_of_the_ground` rotates the toe
        // to rescue a tip it believes is below the floor, so an estimate
        // pointing the wrong way reports a penetration that is not there:
        // measured live on flat ground, a tip 0.060 m ABOVE its own joint,
        // where every in-crate path keeps it level to within 0.8 mm. Feet whose
        // toes point at the sky.
        //
        // The plugin now reads the toe's own CHILD, which needs no per-rig name
        // table — whatever hangs off the toe joint is by construction the point
        // the toe runs toward. Live, after that change: tip 0.5 mm BELOW its
        // joint and 0.0789 m in front, matching the measured bone exactly.
        //
        // This test keeps the gap itself pinned, so nobody "simplifies" the
        // plugin back to the estimate on the grounds that it looks equivalent.
        let rig = puppet_base();

        for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
            let measured = rig.toe_end_offset(toe);
            let estimated = rig.bind_rotations[toe].inverse()
                * (rig.offsets[toe] * crate::character::anim::rig::TOE_END_FRACTION);

            let divergence = measured.angle_between(estimated).to_degrees();
            assert!(
                divergence > 10.0,
                "{}'s estimated tip is only {divergence:.1} degrees off the \
                 measured one — if the estimate has become accurate, the \
                 plugin's child lookup could be dropped, but check that on a \
                 second rig before believing it",
                toe.name(),
            );
        }
    }

    #[test]
    fn the_rest_pose_puts_the_toe_tip_level_and_in_front_of_its_joint() {
        // A foot's tip extends FORWARD from the toe joint and sits level with
        // it. Stated because the opposite — a tip above its joint — is what a
        // whole class of frame bug renders as, and it is immediately visible
        // as feet whose toes point at the sky.
        let rig = puppet_base();
        let positions = forward_kinematics_on(&LocalPose::REST, &rig);
        let rotations =
            crate::character::anim::rig::accumulate_world_rotations(&LocalPose::REST, &rig);

        for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
            let joint = positions[toe];
            let tip = joint + rotations[toe] * rig.toe_end_offset(toe);

            assert!(
                (tip.y - joint.y).abs() < 0.005,
                "{}'s tip is {:.4} m above its joint; a rest-pose foot is flat",
                toe.name(),
                tip.y - joint.y,
            );
            assert!(
                tip.z - joint.z > 0.05,
                "{}'s tip is only {:.4} m in front of its joint — toes point \
                 forward, and a tip behind the joint means the foot is reversed",
                toe.name(),
                tip.z - joint.z,
            );
        }
    }

    #[test]
    fn the_plugins_substitute_hips_offset_leaves_the_character_standing_up() {
        // The plugin cannot read `Hips`' live translation — that is the one
        // thing `write_pose_to_skeleton` overwrites every frame, so reading it
        // back would feed the solve its own output. It substitutes a fixed
        // value instead, and THAT value's frame is the trap this pins.
        //
        // `Bone::t_pose_offset` is the synthetic table's, Y-up. The real rig's
        // `root_rotation` is a Z-up correction, and forward kinematics applies
        // it to the hips offset because the root has no parent to inherit one
        // from. Handing it the Y-up value turns (0, 0.94, 0) into (0, 0, 0.94)
        // and lays the character on its back — measured, before the fix: ankle
        // y = -0.856, toe y = -0.927, a metre underground.
        //
        // The leg IK then reacted CORRECTLY to that garbage, rotating each toe
        // 113 degrees to lift a tip it believed was 1.006 m below the floor.
        // The visible symptom was feet whose toes pointed at the sky, which is
        // several steps away from the actual cause.
        let mut world = bevy::prelude::World::new();
        let mut entities = HashMap::new();
        for &bone in Bone::ALL.iter() {
            entities.insert(bone, world.spawn(bevy::prelude::Transform::IDENTITY).id());
        }
        let skeleton = real_skeleton(&parsed_rig(), entities);
        let parsed = puppet_base();

        // Exactly what `solve_foot_ik` builds.
        let rig = crate::character::anim::plugin::live_rig_geometry(&skeleton, |bone| Some(parsed.offsets[bone]));

        let positions = forward_kinematics_on(&LocalPose::REST, &rig);

        assert!(
            positions[Bone::Hips].y > 0.5,
            "the hips are at {:?} — a standing character carries them up in Y, \
             and a large Z with a near-zero Y means the rig is lying down",
            positions[Bone::Hips],
        );
        for foot in [Bone::LeftFoot, Bone::RightFoot] {
            assert!(
                positions[foot].y < positions[Bone::Hips].y,
                "{} is at {:?}, above the hips at {:?}",
                foot.name(),
                positions[foot],
                positions[Bone::Hips],
            );
            assert!(
                positions[foot].y > -0.1,
                "{} is at {:?}, sunk below the floor",
                foot.name(),
                positions[foot],
            );
        }
    }

    #[test]
    fn the_live_rig_geometry_stands_where_the_asset_does_in_metres() {
        // The geometry the gait, foot IK and root motion solve on must put
        // every joint where the renderer does, in metres. It did neither on
        // `character.glb`: its hips took the synthetic 0.94 m (the asset's
        // rest is 1.126 m), so it crouched with its feet 0.186 m in the air;
        // and its bones' centimetre translations under Blender's 0.01 node
        // went in unscaled, a 46 m thigh. `puppet_base` hid both — metres,
        // and hips 9 mm from 0.94.
        //
        // Checked on `puppet_base` and on a centimetre-authored copy of it
        // under a 0.01 node, which must solve on the same geometry.
        use crate::character::anim::plugin::live_rig_geometry;

        let mut world = bevy::prelude::World::new();
        let entities: HashMap<Bone, bevy::prelude::Entity> =
            Bone::ALL.iter().map(|&bone| (bone, world.spawn(bevy::prelude::Transform::IDENTITY).id())).collect();
        let asset = puppet_base();
        let bind = forward_kinematics_on(&LocalPose::REST, &asset);

        let metres = parsed_rig();
        let mut centimetres = metres.clone();
        centimetres.hips_rest_local_translation *= 100.0;
        centimetres.hips_parent_rest_world_scale = Vec3::splat(0.01);

        for (label, parsed, units) in [("metres", metres, 1.0), ("centimetres", centimetres, 100.0)] {
            let skeleton = real_skeleton(&parsed, entities.clone());
            let rig = live_rig_geometry(&skeleton, |bone| Some(asset.offsets[bone] * units));
            let solved = forward_kinematics_on(&LocalPose::REST, &rig);
            for bone in [
                Bone::Hips,
                Bone::LeftUpLeg,
                Bone::LeftLeg,
                Bone::LeftFoot,
                Bone::LeftToeBase,
                Bone::RightFoot,
                Bone::Spine2,
            ] {
                let off = (solved[bone] - bind[bone]).length();
                assert!(
                    off < 1.0e-3,
                    "{label}: {} at {:?}, the asset's bind puts it at {:?} ({:.1} mm off)",
                    bone.name(),
                    solved[bone],
                    bind[bone],
                    off * 1e3
                );
            }
        }
    }

    #[test]
    fn a_stance_keeps_the_soles_where_the_asset_stands_them() {
        // Bending the knees shortens the legs; the stance lowers the hips by
        // the same, so the feet stay at the asset's own bind heights. It
        // used to leave the hips at full height, and on `puppet_base` the
        // feet floated 6.9 mm with no leg left to reach the floor: the foot
        // IK pitched them 4.5° toe-down instead.
        use crate::character::anim::plugin::live_rig_geometry;
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let mut world = bevy::prelude::World::new();
        let entities: HashMap<Bone, bevy::prelude::Entity> =
            Bone::ALL.iter().map(|&bone| (bone, world.spawn(bevy::prelude::Transform::IDENTITY).id())).collect();
        let asset = puppet_base();
        let rig = live_rig_geometry(&real_skeleton(&parsed_rig(), entities), |bone| Some(asset.offsets[bone]));
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let (bind, standing) = (forward_kinematics_on(&LocalPose::REST, &rig), forward_kinematics_on(&stood, &rig));
        assert!(stood.root_translation.y < -0.005, "the hips should come down, moved {}", stood.root_translation.y);
        for bone in [Bone::LeftFoot, Bone::RightFoot, Bone::LeftToeBase, Bone::RightToeBase] {
            let off = standing[bone].y - bind[bone].y;
            assert!(off.abs() < 5.0e-4, "{} stands {:.1} mm off its bind height", bone.name(), off * 1e3);
        }
    }

    #[test]
    fn the_real_rig_parses() {
        let rig = puppet_base();

        // Every bone got a real offset — a zero one would mean a name that
        // silently failed to resolve.
        for &(bone, name) in BONE_NAMES.iter() {
            if bone == Bone::Hips {
                continue; // the root's own offset is its scene position
            }
            assert!(
                rig.offsets[bone].length() > 1.0e-6,
                "{} ('{name}') came back with a zero offset",
                bone.name(),
            );
        }
    }

    #[test]
    fn the_legs_have_a_real_shin() {
        // THE proportion gap. The synthetic rig leaves this segment as a
        // 0.07 m stub; the real one is nearly half a metre, which is why
        // knee flexion is 6x more visible on it.
        let rig = puppet_base();

        let femur = rig.offsets[Bone::LeftLeg].length();
        let shin = rig.offsets[Bone::LeftFoot].length();

        assert!(
            (femur - 0.429).abs() < 0.01,
            "expected a 0.429 m femur, got {femur}",
        );
        assert!((shin - 0.459).abs() < 0.01, "expected a 0.459 m shin, got {shin}");
    }

    #[test]
    fn the_foot_carries_its_real_bind_rotation() {
        // THE orientation gap, and the reason this module exists. The
        // synthetic rig binds every bone at identity; this one binds its
        // foot about 70 degrees off.
        let rig = puppet_base();

        let (axis, angle) = rig.bind_rotations[Bone::LeftFoot].to_axis_angle();
        let signed = if axis.x < 0.0 { -angle } else { angle };

        assert!(
            (signed.to_degrees() - (-69.8)).abs() < 1.0,
            "expected the foot bound at about -69.8 degrees, got {}",
            signed.to_degrees(),
        );
    }

    #[test]
    fn the_root_correction_stands_the_rig_the_right_way_up() {
        // Without it the ~164-degree thigh bind — which IS the Z-up-to-Y-up
        // correction, carried on the first leg bone — flips the chain and
        // puts the foot ABOVE the hips, measured at y = +1.68.
        let rig = puppet_base();
        let positions = forward_kinematics_on(&LocalPose::REST, &rig);

        assert!(
            positions[Bone::LeftFoot].y < positions[Bone::Hips].y,
            "the foot ({}) should sit below the hips ({})",
            positions[Bone::LeftFoot].y,
            positions[Bone::Hips].y,
        );
        assert!(
            positions[Bone::Head].y > positions[Bone::Hips].y,
            "...and the head above them",
        );
    }

    #[test]
    fn the_bind_pose_stands_at_a_human_height() {
        // A whole-rig sanity check: if the parse or the root correction
        // were wrong, the character would not be person-shaped.
        let rig = puppet_base();
        let positions = forward_kinematics_on(&LocalPose::REST, &rig);

        let height = positions[Bone::Head].y - positions[Bone::LeftFoot].y;
        assert!(
            (1.2..2.2).contains(&height),
            "head-to-foot spans {height} m, which is not a human-scale rig",
        );
    }

    #[test]
    fn the_toe_tip_is_read_from_the_rig_rather_than_estimated() {
        // This rig has `ball_leaf_l`, so the tip is measured. The
        // estimate — half the toe's own length — is for rigs that do not.
        let rig = puppet_base();

        let tip = rig.toe_end_offset(Bone::LeftToeBase);
        assert!(
            (tip.length() - 0.0789).abs() < 0.001,
            "expected the measured 0.0789 m tip, got {}",
            tip.length(),
        );
    }

    #[test]
    fn the_gallery_and_this_module_agree_about_bone_names() {
        // Two copies of the same mapping, pinned together. The gallery owns
        // the real one (a name convention is example-layer policy); this
        // guards the copy against drifting from it.
        //
        // Checked by VALUE rather than by importing, because an example's
        // items are not visible to the library's own tests.
        let expected: [(Bone, &str); 22] = [
            (Bone::Hips, "pelvis"),
            (Bone::Spine, "spine_01"),
            (Bone::Spine1, "spine_02"),
            (Bone::Spine2, "spine_03"),
            (Bone::Neck, "neck_01"),
            (Bone::Head, "Head"),
            (Bone::LeftShoulder, "clavicle_l"),
            (Bone::LeftArm, "upperarm_l"),
            (Bone::LeftForeArm, "lowerarm_l"),
            (Bone::LeftHand, "hand_l"),
            (Bone::RightShoulder, "clavicle_r"),
            (Bone::RightArm, "upperarm_r"),
            (Bone::RightForeArm, "lowerarm_r"),
            (Bone::RightHand, "hand_r"),
            (Bone::LeftUpLeg, "thigh_l"),
            (Bone::LeftLeg, "calf_l"),
            (Bone::LeftFoot, "foot_l"),
            (Bone::LeftToeBase, "ball_l"),
            (Bone::RightUpLeg, "thigh_r"),
            (Bone::RightLeg, "calf_r"),
            (Bone::RightFoot, "foot_r"),
            (Bone::RightToeBase, "ball_r"),
        ];

        assert_eq!(BONE_NAMES, expected);
    }

    #[test]
    fn every_bone_in_the_enum_has_a_name_mapping() {
        // A forgotten bone would resolve to nothing and silently take a
        // zero offset.
        for &bone in Bone::ALL.iter() {
            assert!(
                BONE_NAMES.iter().any(|(b, _)| *b == bone),
                "{} has no glTF name",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_missing_bone_is_reported_rather_than_silently_zeroed() {
        // The failure mode that matters: a rig without one of the bones
        // must say so, not hand back a plausible-looking geometry with a
        // hole in it.
        let without_a_foot = r#"{"nodes":[{"name":"pelvis"}]}"#;

        let error = from_gltf(without_a_foot).expect_err("should not parse");
        assert!(
            error.contains("foot_l") || error.contains("spine_01"),
            "expected a named-bone error, got: {error}",
        );
    }

    #[test]
    fn malformed_input_is_reported() {
        assert!(from_gltf("not json at all").is_err());
        assert!(from_gltf(r#"{"meshes":[]}"#).is_err(), "no nodes array");
    }
}
