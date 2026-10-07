//! A skinned glTF humanoid, loaded and bound to this crate's [`Bone`]s.
//!
//! [`spawn_gltf_humanoid`] spawns the asset; [`HumanoidPlugin`] waits for its
//! nodes to appear (a `WorldAssetRoot` instantiates asynchronously, several
//! frames later) and then inserts the [`HumanoidSkeleton`] that the rest of
//! `character::anim` drives. Any number of characters bind independently.
//!
//! Moved out of `examples/character_gallery.rs` (2026-10-02) so every
//! example, and a game, binds a rig the same way.

use std::collections::HashMap;

use bevy::gltf::GltfAssetLabel;
use bevy::mesh::skinning::SkinnedMesh;
use bevy::prelude::*;
use bevy::world_serialization::WorldAssetRoot;

use crate::character::{Bone, BoneMarker, HumanoidSkeleton};

/// Binds every [`GltfHumanoid`] once its nodes have spawned.
pub struct HumanoidPlugin;

impl Plugin for HumanoidPlugin {
    fn build(&self, app: &mut App) {
        // The fingers curl the moment a rig binds, before anything reads it.
        app.add_systems(Update, (bind_gltf_humanoids, super::hand::relax_hands).chain().in_set(HumanoidSet::Bind));
        // Fingers closing round what a hand holds (`hand::RelaxedHands::grip`),
        // once what holds it has asked.
        app.add_systems(Update, super::hand::close_hands.after(super::plugin::AnimSet::Target));
    }
}

/// Where binding runs, so consumers can order after it.
#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum HumanoidSet {
    Bind,
}

/// The root of a glTF humanoid not yet bound, or bound (it then also has a
/// [`HumanoidSkeleton`]).
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct GltfHumanoid;

/// The asset's own facing correction, composed under the heading rather
/// than overwritten by it.
///
/// This crate's convention is `-Z` forward; an asset whose geometry faces
/// `+Z` (as `puppet_base.gltf` does) needs a 180° yaw. Root motion writes
/// the root's rotation every frame, so the correction must be remembered:
/// assigned over, it was wiped the first frame the walk ran, and the mesh
/// walked backward (measured, `character_gallery`, before this was a
/// component).
#[derive(Component, Debug, Clone, Copy)]
pub struct FacingCorrection(pub Quat);

/// Rescale a humanoid's segments to Winter's fractions of `stature` (or of
/// its own height, `None`) as it binds (`proportions::winter_factors`).
#[derive(Component, Debug, Clone, Copy, Default)]
pub struct HumanoidProportions {
    pub stature: Option<f32>,
}

/// Spawns the glTF humanoid at `path` (relative to the asset root) at
/// `transform`, turned by `yaw_correction` radians under it. Returns the
/// root entity; its [`HumanoidSkeleton`] arrives once the asset has loaded.
pub fn spawn_gltf_humanoid(commands: &mut Commands, asset_server: &AssetServer, path: &str, yaw_correction: f32, transform: Transform) -> Entity {
    let correction = Quat::from_rotation_y(yaw_correction);
    commands
        .spawn((
            WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset(path.to_string()))),
            Transform { rotation: transform.rotation * correction, ..transform },
            FacingCorrection(correction),
            GltfHumanoid,
        ))
        .id()
}

/// This crate's [`Bone`]s by the Unreal Engine Mannequin's joint names, a
/// published convention many third-party rigs follow (`puppet_base.gltf`
/// does, verified against its node dump).
pub const UE_MANNEQUIN_BONE_NAMES: [(Bone, &str); 22] = [
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

/// `bone`'s node among `descendants_by_name`, trying the Mixamo names
/// (`mixamorig:LeftArm`, then bare `LeftArm`; `Bone::name` is Mixamo's)
/// and then [`UE_MANNEQUIN_BONE_NAMES`]. Only a name actually present is
/// returned: returning a table's literal unconfirmed once panicked on the
/// lookup that followed.
pub fn resolve_bone_node_name<'a, T>(bone: Bone, descendants_by_name: &HashMap<&'a str, T>) -> Option<&'a str> {
    let mixamo_prefixed = format!("mixamorig:{}", bone.name());
    if let Some((&found, _)) = descendants_by_name.get_key_value(mixamo_prefixed.as_str()) {
        return Some(found);
    }
    if let Some((&found, _)) = descendants_by_name.get_key_value(bone.name()) {
        return Some(found);
    }
    let ue_mannequin = UE_MANNEQUIN_BONE_NAMES.iter().find(|&&(b, _)| b == bone).map(|&(_, name)| name)?;
    descendants_by_name.get_key_value(ue_mannequin).map(|(&found, _)| found)
}

/// The [`GltfHumanoid`] roots not bound yet, with their proportions if any.
type UnboundHumanoids<'w, 's> =
    Query<'w, 's, (Entity, Option<&'static HumanoidProportions>, Option<&'static FacingCorrection>), (With<GltfHumanoid>, Without<HumanoidSkeleton>)>;

/// Binds each [`GltfHumanoid`] root without a skeleton yet, once every
/// joint it maps has spawned (until then it tries again next frame).
///
/// Each joint's rest local rotation and translation are captured here,
/// before any animation writes them: `HumanoidSkeleton::for_other_rig`
/// needs the rig's own bind (a real rig's joints have non-identity bind
/// rotations, `pelvis` a ~106.6° turn), and the hips need their parent's
/// rest world rotation and scale to convert a solved hip position into
/// the rig's own local frame.
#[allow(clippy::too_many_arguments)]
pub fn bind_gltf_humanoids(
    mut commands: Commands,
    roots: UnboundHumanoids,
    named: Query<(Entity, &Name, &Transform)>,
    children_of: Query<&Children>,
    child_of: Query<&ChildOf>,
    global_transforms: Query<&GlobalTransform>,
    mut skins: Query<&mut SkinnedMesh>,
) {
    for (root, proportions, correction) in &roots {
        // Every named descendant, however deep the importer nests it (this
        // asset has an `Armature` node above `pelvis`), and every skinned
        // mesh: a proportioned segment re-skins only this character.
        let mut descendants_by_name: HashMap<&str, (Entity, Quat, Vec3)> = HashMap::new();
        let mut skinned = Vec::new();
        let mut stack = vec![root];
        while let Some(entity) = stack.pop() {
            if let Ok(children) = children_of.get(entity) {
                stack.extend(children.iter());
            }
            if let Ok((found, name, transform)) = named.get(entity) {
                descendants_by_name.insert(name.as_str(), (found, transform.rotation, transform.translation));
            }
            if skins.contains(entity) {
                skinned.push(entity);
            }
        }

        let mut bones = HashMap::new();
        let mut rest_rotations = HashMap::new();
        let mut rest_directions = HashMap::new();
        let mut rest_translations = HashMap::new();
        let mut complete = true;
        for &bone in &Bone::ALL {
            let Some(node_name) = resolve_bone_node_name(bone, &descendants_by_name) else {
                complete = false;
                break;
            };
            let (entity, rest_rotation, rest_translation) = descendants_by_name[node_name];
            bones.insert(bone, entity);
            rest_rotations.insert(bone, rest_rotation);
            rest_directions.insert(bone, rest_translation.normalize_or_zero());
            rest_translations.insert(bone, rest_translation);
        }
        if !complete {
            continue;
        }

        let hips_entity = bones[&Bone::Hips];
        let Ok(hips_local_transform) = named.get(hips_entity).map(|(_, _, t)| *t) else { continue };
        let Ok(hips_parent) = child_of.get(hips_entity) else { continue };
        let Ok(hips_parent_global) = global_transforms.get(hips_parent.parent()) else { continue };
        let Ok(root_global) = global_transforms.get(root) else { continue };
        // The hips' parent as the asset stands, its facing correction in but
        // not the heading it was spawned at: the bind is a fact about the
        // model. Bound at a heading, a character standing in `relaxed_stand`
        // held both hands overhead (the playground's random spawn yaws); the
        // gallery, always bound at heading zero, never showed it.
        let unturned = correction.map_or(Quat::IDENTITY, |c| c.0) * root_global.rotation().inverse() * hips_parent_global.rotation();
        let hips_parent_global = GlobalTransform::from(Transform {
            rotation: unturned,
            scale: hips_parent_global.scale(),
            translation: hips_parent_global.translation(),
        });

        // The animation's rest reference is the synthetic T-pose's hips,
        // not this asset's scene position: a solved root translation is in
        // this crate's convention whichever skeleton it drives.
        let build = |hips_translation: Vec3| {
            HumanoidSkeleton::for_other_rig(
                bones.clone(),
                rest_rotations.clone(),
                rest_directions.clone(),
                hips_translation,
                hips_parent_global.rotation(),
                hips_parent_global.scale(),
                Bone::Hips.t_pose_world_position(),
            )
        };
        let mut skeleton = build(hips_local_transform.translation);

        // Winter's fractions of stature: each scaled joint moves; a segment
        // with a single child is skinned through a helper scaled along it
        // (a scaled parent would shear a rotated child); the hips rise so
        // the feet stay down. Everything downstream reads the live
        // translations.
        if let Some(config) = proportions {
            use super::proportions::{along, winter_factors};
            let rig = super::plugin::live_rig_geometry(&skeleton, |bone| rest_translations.get(&bone).copied());
            let rescale = winter_factors(&rig, config.stature);
            info!("humanoid: proportioned to Winter's fractions of {:.2} m (hips {:+.3} m)", rescale.stature, rescale.hips_rise);
            for &bone in Bone::ALL.iter() {
                let factor = rescale.factors[bone];
                if bone == Bone::Hips || (factor - 1.0).abs() < 1.0e-4 {
                    continue;
                }
                let (Some(&entity), Some(&translation)) = (bones.get(&bone), rest_translations.get(&bone)) else { continue };
                let Ok((_, _, transform)) = named.get(entity) else { continue };
                commands.entity(entity).insert(Transform { translation: translation * factor, ..*transform });
                let Some(parent) = bone.parent() else { continue };
                let single = Bone::ALL.iter().filter(|b| b.parent() == Some(parent)).count() == 1;
                let Some(&parent_entity) = bones.get(&parent) else { continue };
                if single {
                    let turn = along(translation);
                    let onto = commands.spawn((Transform::from_rotation(turn), ChildOf(parent_entity))).id();
                    let stretch = commands.spawn((Transform::from_scale(Vec3::new(1.0, factor, 1.0)), ChildOf(onto))).id();
                    let back = commands.spawn((Transform::from_rotation(turn.inverse()), ChildOf(stretch))).id();
                    for &mesh in &skinned {
                        let Ok(mut skin) = skins.get_mut(mesh) else { continue };
                        for joint in skin.joints.iter_mut().filter(|joint| **joint == parent_entity) {
                            *joint = back;
                        }
                    }
                }
            }
            let rise = hips_parent_global.rotation().inverse() * (Vec3::Y * rescale.hips_rise) / hips_parent_global.scale();
            let hips_translation = hips_local_transform.translation + rise;
            commands.entity(hips_entity).insert(Transform { translation: hips_translation, ..hips_local_transform });
            skeleton = build(hips_translation);
        }

        for (bone, entity) in skeleton.iter() {
            commands.entity(entity).insert(BoneMarker(bone));
        }
        commands.entity(root).insert(skeleton);
    }
}
