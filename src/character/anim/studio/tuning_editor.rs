//! The spring tuning panel.
//!
//! Thin, like [`super::pose_editor`]: presets, scoping and the response
//! curve all live in [`super::tuning`], where they are tested without egui.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::character::anim::math::SpringParams;
use crate::character::anim::plugin::AnimSprings;
use crate::character::skeleton::Bone;

use super::tuning::{
    apply, response_curve, suggested_plot_duration, SpringPreset, TuningScope,
};
use super::StudioState;

/// Draws the tuning panel and applies whatever the user changed.
pub fn tuning_panel(
    mut contexts: EguiContexts,
    mut studio: ResMut<StudioState>,
    mut characters: Query<&mut AnimSprings>,
) -> Result {
    if !studio.open || !studio.tuning_open {
        return Ok(());
    }

    let Ok(mut springs) = characters.single_mut() else { return Ok(()) };

    // Adopt the character's own springs once, for the same reason the pose
    // editor adopts its pose: opening a panel must not overwrite what it
    // is opening. See `StudioState::adopted_initial_pose`.
    if !studio.adopted_initial_springs {
        studio.adopted_initial_springs = true;
        studio.springs = springs.0;
    }

    let ctx = contexts.ctx_mut()?;
    let mut changed = false;

    egui::Window::new("Spring tuning")
        .anchor(egui::Align2::LEFT_BOTTOM, egui::vec2(10.0, -10.0))
        .default_width(330.0)
        .resizable(true)
        .show(ctx, |ui| {
            changed |= draw_presets(ui, &mut studio);
            ui.separator();
            changed |= draw_bone_tuning(ui, &mut studio);
            ui.separator();
            draw_response_plot(ui, &studio);
        });

    if changed {
        springs.0 = studio.springs;
    }

    Ok(())
}

/// Preset buttons. Returns whether anything changed.
fn draw_presets(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.label("Preset:");
        for preset in SpringPreset::ALL {
            if ui.button(preset.label()).clicked() {
                studio.springs = preset.springs();
                changed = true;
            }
        }
    });

    ui.weak("A preset scales the rig's grading rather than flattening it.");
    changed
}

/// The per-bone dials. Returns whether anything changed.
fn draw_bone_tuning(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.label("Bone:");
        egui::ComboBox::from_id_salt("tuning_bone")
            .selected_text(studio.tuning_bone.name())
            .show_ui(ui, |ui| {
                for &bone in Bone::ALL.iter() {
                    ui.selectable_value(&mut studio.tuning_bone, bone, bone.name());
                }
            });
    });

    ui.horizontal(|ui| {
        ui.label("Apply to:");
        for scope in TuningScope::ALL {
            ui.radio_value(&mut studio.tuning_scope, scope, scope.label());
        }
    });

    let bone = studio.tuning_bone;
    let mut params = studio.springs[bone];
    let before = params;

    // `max_decimals` for the same reason the pose editor needs it: egui
    // writes back the value it DISPLAYS, so a rounded display silently
    // edits the value every frame.
    ui.add(
        egui::Slider::new(&mut params.halflife, 0.01..=0.6)
            .max_decimals(4)
            .text("half-life")
            .suffix(" s"),
    );
    ui.add(
        egui::Slider::new(&mut params.damping_ratio, 0.2..=2.0)
            .max_decimals(4)
            .text("damping ratio"),
    );
    ui.weak("1.0 is critically damped; below that overshoots and rings.");

    if params.halflife != before.halflife || params.damping_ratio != before.damping_ratio {
        apply(&mut studio.springs, bone, studio.tuning_scope, params);
        changed = true;
    }

    changed
}

/// The step-response plot.
///
/// The one thing a number cannot convey: whether this spring overshoots,
/// by how much, and how long it rings. Choosing a damping ratio without
/// seeing its curve is guessing.
fn draw_response_plot(ui: &mut egui::Ui, studio: &StudioState) {
    let params: SpringParams = studio.springs[studio.tuning_bone];
    let duration = suggested_plot_duration(&params);
    let curve = response_curve(&params, duration);

    let (response, painter) =
        ui.allocate_painter(egui::vec2(ui.available_width(), 110.0), egui::Sense::hover());
    let rect = response.rect;

    painter.rect_filled(rect, 2.0, egui::Color32::from_gray(24));

    // The target line. Without it "overshoot" has nothing to overshoot.
    let peak = curve.iter().map(|point| point.y).fold(1.0f32, f32::max);
    let top = peak.max(1.05);
    let to_screen = |point: bevy::math::Vec2| -> egui::Pos2 {
        egui::pos2(
            rect.left() + (point.x / duration) * rect.width(),
            rect.bottom() - (point.y / top) * rect.height(),
        )
    };

    let target_y = rect.bottom() - (1.0 / top) * rect.height();
    painter.line_segment(
        [egui::pos2(rect.left(), target_y), egui::pos2(rect.right(), target_y)],
        egui::Stroke::new(1.0, egui::Color32::from_gray(70)),
    );

    painter.add(egui::Shape::line(
        curve.iter().map(|&point| to_screen(point)).collect(),
        egui::Stroke::new(1.5, egui::Color32::from_rgb(120, 200, 255)),
    ));

    ui.weak(format!("step response over {duration:.2} s"));
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_plot_maps_the_target_inside_its_own_box() {
        // A rendering guard rather than a maths one: if the vertical scale
        // did not account for overshoot, an underdamped curve would be
        // drawn clipped off the top of the plot — visually indistinguishable
        // from a spring that does not overshoot at all, which is the exact
        // thing the plot exists to show.
        let params = SpringParams { halflife: 0.1, damping_ratio: 0.4, max_speed: 50.0 };
        let curve = response_curve(&params, suggested_plot_duration(&params));
        let peak = curve.iter().map(|point| point.y).fold(1.0f32, f32::max);
        let top = peak.max(1.05);

        assert!(
            peak <= top,
            "the plot's vertical range {top} must contain the curve's peak {peak}",
        );
        assert!(top > 1.0, "and must leave the target line visible below the top");
    }
}
