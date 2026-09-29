//! The pose editor panel.
//!
//! A thin translation of clicks and drags into [`super::edit`] calls — all
//! the editing *semantics* live there, and are tested there, without egui.
//! What this file owns is layout and the one thing a UI genuinely decides:
//! how much of a 22-bone rig to show at once.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::character::anim::plugin::{AnimPose, AnimTarget};
use crate::character::skeleton::Bone;

use super::edit::{EditableRotation, PoseEdit};
use super::{save, StudioState};

/// Draws the editor and applies whatever the user did.
pub fn pose_editor_panel(
    mut contexts: EguiContexts,
    mut studio: ResMut<StudioState>,
    mut characters: Query<(&mut AnimTarget, Option<&mut AnimPose>)>,
) -> Result {
    if !studio.open {
        return Ok(());
    }

    // Adopt whatever the rig is already showing, once, before drawing
    // anything.
    //
    // Without this the studio opens holding its default (the REST pose) and
    // the first frame's write stamps that over the character — a live-caught
    // bug where launching the editor silently replaced `relaxed_stand` with
    // a T-pose. Opening an editor must never change the thing being edited.
    // The flag is set only once a character actually exists. The rig is
    // spawned asynchronously from a glTF, so on early frames this query is
    // empty — marking adoption done regardless would skip it entirely and
    // reinstate the bug, which is exactly what happened the first time.
    if !studio.adopted_initial_pose
        && let Some((target, _)) = characters.iter().next()
    {
        studio.adopted_initial_pose = true;
        studio.edit = PoseEdit::from_file(
            &target.pose,
            save::pose_path(&studio.pose_name).to_string_lossy().into_owned(),
        );
    }

    let ctx = contexts.ctx_mut()?;
    let mut edited = false;

    egui::Window::new("Pose editor")
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(10.0, 10.0))
        .default_width(330.0)
        .resizable(true)
        .show(ctx, |ui| {
            edited |= draw_file_row(ui, &mut studio);
            ui.separator();
            edited |= draw_bone_list(ui, &mut studio);
        });

    // Push the edit onto the live rig, so the viewport is the preview.
    // There is no separate preview widget on purpose: the thing being
    // authored is how the character looks, and a small inset rendering of
    // it would be a worse view of exactly the same information.
    if edited {
        let pose = studio.edit.to_pose();
        for (mut target, animated) in &mut characters {
            target.pose = pose;

            // Snap rather than spring toward it while editing. A spring
            // makes every drag lag behind the slider, which reads as the
            // editor being broken; the springs are a runtime feature, not
            // an authoring one.
            if studio.snap_while_editing
                && let Some(mut animated) = animated
            {
                *animated = AnimPose::settled_on(&pose);
            }
        }
    }

    Ok(())
}

/// The load/save row. Returns whether the pose changed.
fn draw_file_row(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut edited = false;

    ui.horizontal(|ui| {
        ui.label("Pose:");
        egui::ComboBox::from_id_salt("studio_pose_name")
            .selected_text(studio.pose_name.clone())
            .show_ui(ui, |ui| {
                for (name, _) in crate::character::anim::poses::all_named_poses() {
                    ui.selectable_value(&mut studio.pose_name, name.to_string(), name);
                }
            });

        if ui.button("Load").clicked() {
            let path = save::pose_path(&studio.pose_name);
            match save::load(&path.to_string_lossy()) {
                Ok(edit) => {
                    studio.edit = edit;
                    studio.status = format!("loaded {}", path.display());
                    edited = true;
                }
                Err(error) => studio.status = format!("load failed: {error}"),
            }
        }

        // Saving over a file is the one destructive action here, so it
        // asks first rather than trusting a single click near a Load
        // button. The confirmation is inline rather than a modal because a
        // modal would hide the rig the user is judging the pose by.
        if studio.confirming_save {
            if ui.button("Confirm overwrite").clicked() {
                let path = save::pose_path(&studio.pose_name);
                match save::save_as(&studio.edit, &path.to_string_lossy()) {
                    Ok(path) => {
                        studio.edit.mark_clean();
                        studio.status = format!("saved {}", path.display());
                    }
                    Err(error) => studio.status = format!("save failed: {error}"),
                }
                studio.confirming_save = false;
            }
            if ui.button("Cancel").clicked() {
                studio.confirming_save = false;
            }
        } else if ui.button("Save").clicked() {
            studio.confirming_save = true;
        }
    });

    ui.horizontal(|ui| {
        if ui.button("Capture from rig").clicked() {
            studio.capture_requested = true;
        }
        if ui.button("Reset all").clicked() {
            studio.edit.reset_all();
            edited = true;
        }
        // ASCII arrows: egui's default font has no glyph for "→" and
        // renders it as a replacement box, which reads as a broken button.
        if ui.button("Mirror L to R").clicked() {
            studio.edit.mirror(true);
            edited = true;
        }
        if ui.button("Mirror R to L").clicked() {
            studio.edit.mirror(false);
            edited = true;
        }
    });

    ui.checkbox(&mut studio.snap_while_editing, "Snap to pose while editing");
    ui.checkbox(&mut studio.drag_enabled, "Drag joints in the viewport");
    ui.checkbox(&mut studio.effectors_enabled, "IK effectors (drag a hand or foot)");
    ui.checkbox(&mut studio.animation_playing, "Play idle animation (off while authoring)");
    ui.checkbox(&mut studio.timeline_open, "Clip timeline");
    ui.checkbox(&mut studio.phase_open, "Phase oscillators");

    ui.horizontal(|ui| {
        if studio.edit.is_dirty() {
            ui.colored_label(egui::Color32::from_rgb(230, 180, 60), "● unsaved changes");
        } else {
            ui.weak("saved");
        }
    });

    if !studio.status.is_empty() {
        ui.weak(studio.status.clone());
    }

    edited
}

/// The per-bone controls. Returns whether the pose changed.
///
/// Grouped by body region and collapsed by default: twenty-two bones of
/// four widgets each is far more than fits on screen at once, and a flat
/// list makes finding "the left elbow" a scrolling exercise. Only the
/// groups a user has opened cost screen space.
fn draw_bone_list(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut edited = false;

    egui::ScrollArea::vertical().show(ui, |ui| {
        for (group, bones) in BONE_GROUPS {
            egui::CollapsingHeader::new(*group)
                .default_open(*group == "Left arm")
                .show(ui, |ui| {
                    for &bone in *bones {
                        edited |= draw_bone(ui, studio, bone);
                    }
                });
        }
    });

    edited
}

/// One bone's controls.
fn draw_bone(ui: &mut egui::Ui, studio: &mut StudioState, bone: Bone) -> bool {
    let mut rotation = studio.edit.rotation(bone);
    let before = rotation;

    ui.horizontal(|ui| {
        ui.strong(bone.name());
        if ui.small_button("reset").clicked() {
            rotation = EditableRotation::default();
        }
    });

    // `max_decimals` on every numeric widget, deliberately.
    //
    // egui writes its DISPLAYED value back into the bound variable, and by
    // default it displays a rounded one. An axis component of -0.6989497
    // shows as "-0.70" and becomes -0.70 — so merely rendering the panel
    // edits the pose, every frame, forever. Live-caught doing exactly that:
    // `LeftShoulder` drifted from its authored 25.56 degrees to 71.36 while
    // nobody touched it, and the pose reported unsaved changes on load.
    //
    // Six decimals is past f32's meaningful precision for these magnitudes,
    // so the round trip is exact.
    ui.add(
        egui::Slider::new(&mut rotation.degrees, -180.0..=180.0)
            .max_decimals(6)
            .text("angle")
            .suffix("°"),
    );

    // Axis as three drag values rather than a slider: an axis is a
    // direction, and dragging one component of a direction between fixed
    // endpoints is meaningless — it is normalized before use anyway.
    ui.horizontal(|ui| {
        ui.label("axis");
        for (label, component) in [
            ("x ", &mut rotation.axis.x),
            ("y ", &mut rotation.axis.y),
            ("z ", &mut rotation.axis.z),
        ] {
            ui.add(
                egui::DragValue::new(component)
                    .speed(0.01)
                    .max_decimals(6)
                    .prefix(label),
            );
        }
    });

    ui.separator();

    if rotation != before {
        studio.edit.set_rotation(bone, rotation);
        return true;
    }
    false
}

/// How the bone list is grouped on screen.
///
/// Manually ordered rather than derived from `Bone::ALL`, because the
/// useful grouping is anatomical ("the left arm") and the enum's order is
/// hierarchical. Every bone appears exactly once — asserted by a test, so a
/// bone added to the rig and forgotten here fails loudly.
const BONE_GROUPS: &[(&str, &[Bone])] = &[
    ("Spine and head", &[Bone::Hips, Bone::Spine, Bone::Spine1, Bone::Spine2, Bone::Neck, Bone::Head]),
    ("Left arm", &[Bone::LeftShoulder, Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand]),
    ("Right arm", &[Bone::RightShoulder, Bone::RightArm, Bone::RightForeArm, Bone::RightHand]),
    ("Left leg", &[Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase]),
    ("Right leg", &[Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase]),
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_bone_appears_in_exactly_one_group() {
        // Guards the hand-maintained layout: a bone missing from here is a
        // bone the editor simply cannot reach, which is invisible without
        // opening the UI and hunting for it.
        for &bone in Bone::ALL.iter() {
            let count = BONE_GROUPS
                .iter()
                .flat_map(|(_, bones)| bones.iter())
                .filter(|&&listed| listed == bone)
                .count();

            assert_eq!(
                count, 1,
                "{} appears in {count} editor groups, expected exactly 1",
                bone.name(),
            );
        }
    }

    #[test]
    fn the_groups_list_nothing_that_is_not_a_bone() {
        // The other direction: a duplicate would give one bone two sets of
        // widgets that silently fight each other.
        let listed: usize = BONE_GROUPS.iter().map(|(_, bones)| bones.len()).sum();
        assert_eq!(
            listed,
            Bone::ALL.len(),
            "the editor groups list {listed} bones but the rig has {}",
            Bone::ALL.len(),
        );
    }
}
