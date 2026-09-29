//! The phase-oscillator editor.
//!
//! The layer that makes a character never perfectly still: additive sine
//! waves on a gait or breath clock, composed onto the pose before the
//! spring smooths them.
//!
//! # What makes this hard to author blind
//!
//! Each oscillator is four numbers, and none of them means much alone.
//! Amplitude is in radians on a bone whose visible motion depends on how
//! far down a limb it sits. A harmonic of 2 does not read as "twice as
//! fast" but as "peaks at each footfall". An offset of `PI/2` is the
//! difference between hip sway reinforcing spinal twist and cancelling it.
//!
//! What matters is the *relationships* — whether two waves are in step,
//! and how big each is against the others. So the panel plots them
//! together on one axis, which is the only way to see that.

use bevy::math::Vec3;
use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::character::anim::phase::{GaitPhase, PhaseClock, PhaseLayer, PhaseOscillator};
use crate::character::anim::plugin::AnimPhaseLayer;
use crate::character::skeleton::Bone;

use super::StudioState;

/// How many points each plotted wave uses.
const CURVE_SAMPLES: usize = 128;
/// How tall the plot is.
const PLOT_HEIGHT: f32 = 120.0;

/// Draws the panel and applies what the user changed.
pub fn phase_editor_panel(
    mut contexts: EguiContexts,
    mut studio: ResMut<StudioState>,
    mut characters: Query<&mut AnimPhaseLayer>,
) -> Result {
    if !studio.open || !studio.phase_open {
        return Ok(());
    }

    // Adopt the character's own layer once, for the same reason every
    // other panel does: opening an editor must not overwrite what it is
    // opening. Note the character may have no layer at all — the studio
    // suspends it while authoring — so the studio's copy is the source of
    // truth and gets pushed back only when it changes.
    if !studio.adopted_initial_phase {
        studio.adopted_initial_phase = true;
        if let Ok(layer) = characters.single() {
            studio.phase_layer = layer.0.clone();
        }
    }

    let ctx = contexts.ctx_mut()?;
    let mut changed = false;

    egui::Window::new("Phase oscillators")
        // Offset well clear of the gallery's own Controls window, which
        // anchors RIGHT_TOP and is about 440 px tall. Overlapping panels
        // are not merely untidy — the one underneath stops receiving
        // clicks, so a control can look present and be unusable.
        .anchor(egui::Align2::RIGHT_BOTTOM, egui::vec2(-10.0, -10.0))
        .default_width(360.0)
        .default_height(300.0)
        .resizable(true)
        .show(ctx, |ui| {
            changed |= draw_presets(ui, &mut studio);
            ui.separator();
            draw_plot(ui, &studio);
            ui.separator();
            changed |= draw_oscillator_list(ui, &mut studio);
        });

    if changed
        && let Ok(mut layer) = characters.single_mut()
    {
        layer.0 = studio.phase_layer.clone();
    }

    Ok(())
}

/// Preset layers and the add button.
fn draw_presets(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        ui.label("Layer:");
        if ui.button("Standing idle").clicked() {
            studio.phase_layer = PhaseLayer::standing_idle();
            changed = true;
        }
        if ui.button("Locomotion").clicked() {
            studio.phase_layer = PhaseLayer::locomotion();
            changed = true;
        }
        if ui.button("None").clicked() {
            studio.phase_layer = PhaseLayer::none();
            changed = true;
        }
    });

    ui.horizontal(|ui| {
        if ui.button("Add oscillator").clicked() {
            // A visible default rather than a zero one: an oscillator that
            // does nothing on creation gives no feedback that the button
            // worked, and the first thing anyone does is drag amplitude up
            // to find out.
            studio.phase_layer.oscillators.push((
                studio.phase_bone,
                PhaseOscillator {
                    clock: PhaseClock::Gait,
                    harmonic: 1.0,
                    amplitude: 0.05,
                    offset: 0.0,
                    axis: Vec3::Z,
                },
            ));
            changed = true;
        }

        egui::ComboBox::from_id_salt("phase_new_bone")
            .selected_text(studio.phase_bone.name())
            .show_ui(ui, |ui| {
                for &bone in Bone::ALL.iter() {
                    ui.selectable_value(&mut studio.phase_bone, bone, bone.name());
                }
            });

        ui.add(
            egui::Slider::new(&mut studio.phase_preview_speed, 0.0..=3.0)
                .max_decimals(2)
                .text("preview speed"),
        );
    });

    changed
}

/// Plots every oscillator over one gait cycle.
///
/// All on one axis, deliberately. A single wave in isolation tells an
/// author almost nothing; what they are tuning is how the waves sit
/// against each other — whether the hip sway leads the spinal twist, and
/// whether the head bob is small enough to read as secondary.
fn draw_plot(ui: &mut egui::Ui, studio: &StudioState) {
    let (response, painter) = ui.allocate_painter(
        egui::vec2(ui.available_width(), PLOT_HEIGHT),
        egui::Sense::hover(),
    );
    let rect = response.rect;
    painter.rect_filled(rect, 2.0, egui::Color32::from_gray(24));

    if studio.phase_layer.oscillators.is_empty() {
        ui.weak("no oscillators — the character holds its pose exactly");
        return;
    }

    // Scale to the largest amplitude present, so a small wave beside a
    // large one is still visible as a small wave rather than a flat line.
    let peak = studio
        .phase_layer
        .oscillators
        .iter()
        .map(|(_, oscillator)| oscillator.amplitude.abs())
        .fold(0.0f32, f32::max)
        .max(1.0e-3);

    // The zero line, so "above" and "below" mean something.
    let middle = rect.center().y;
    painter.line_segment(
        [egui::pos2(rect.left(), middle), egui::pos2(rect.right(), middle)],
        egui::Stroke::new(1.0, egui::Color32::from_gray(60)),
    );

    for (index, (bone, oscillator)) in studio.phase_layer.oscillators.iter().enumerate() {
        let selected = studio.selected_oscillator == Some(index);
        let colour = wave_colour(index, selected);

        let points: Vec<egui::Pos2> = (0..CURVE_SAMPLES)
            .map(|sample| {
                let t = sample as f32 / (CURVE_SAMPLES - 1) as f32;
                let value = sample_wave(oscillator, t);

                egui::pos2(
                    rect.left() + t * rect.width(),
                    middle - (value / peak) * (rect.height() * 0.45),
                )
            })
            .collect();

        painter.add(egui::Shape::line(
            points,
            egui::Stroke::new(if selected { 2.5 } else { 1.2 }, colour),
        ));

        let _ = bone;
    }

    ui.weak("one full cycle; the selected oscillator is drawn heavier");
}

/// An oscillator's value at `t` through one cycle of its own clock.
///
/// Plotting against the oscillator's own clock rather than wall time is
/// what makes two waves comparable: a breath wave and a gait wave run at
/// different rates, and drawing them against real seconds would show one
/// as a flat line whenever the other was interesting.
fn sample_wave(oscillator: &PhaseOscillator, t: f32) -> f32 {
    let phase = t * std::f32::consts::TAU;
    oscillator.amplitude * (oscillator.harmonic * phase + oscillator.offset).sin()
}

/// A stable colour per oscillator, so a wave keeps its identity across
/// frames and matches its row in the list.
fn wave_colour(index: usize, selected: bool) -> egui::Color32 {
    const PALETTE: [egui::Color32; 6] = [
        egui::Color32::from_rgb(120, 200, 255),
        egui::Color32::from_rgb(140, 230, 160),
        egui::Color32::from_rgb(255, 190, 110),
        egui::Color32::from_rgb(220, 150, 230),
        egui::Color32::from_rgb(240, 130, 130),
        egui::Color32::from_rgb(180, 200, 120),
    ];

    let colour = PALETTE[index % PALETTE.len()];
    if selected { colour } else { colour.gamma_multiply(0.55) }
}

/// The per-oscillator rows.
fn draw_oscillator_list(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut changed = false;
    let mut remove = None;

    egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
        for index in 0..studio.phase_layer.oscillators.len() {
            let selected = studio.selected_oscillator == Some(index);
            let (bone, mut oscillator) = studio.phase_layer.oscillators[index];
            let before = oscillator;
            let mut bone = bone;

            ui.horizontal(|ui| {
                // The swatch ties a row to its wave in the plot above.
                let (patch, painter) =
                    ui.allocate_painter(egui::vec2(12.0, 12.0), egui::Sense::hover());
                painter.rect_filled(patch.rect, 2.0, wave_colour(index, true));

                if ui.selectable_label(selected, bone.name()).clicked() {
                    studio.selected_oscillator = Some(index);
                }
                if ui.small_button("remove").clicked() {
                    remove = Some(index);
                }
            });

            if selected {
                egui::ComboBox::from_id_salt(("phase_bone", index))
                    .selected_text(bone.name())
                    .show_ui(ui, |ui| {
                        for &candidate in Bone::ALL.iter() {
                            ui.selectable_value(&mut bone, candidate, candidate.name());
                        }
                    });

                ui.horizontal(|ui| {
                    ui.label("clock:");
                    ui.radio_value(&mut oscillator.clock, PhaseClock::Gait, "gait");
                    ui.radio_value(&mut oscillator.clock, PhaseClock::Breath, "breath");
                });

                // Amplitude in DEGREES. The runtime stores radians, but
                // nobody judges "is this sway too big" in radians, and a
                // value of 0.035 reads as a typo rather than as two
                // degrees.
                let mut degrees = oscillator.amplitude.to_degrees();
                if ui
                    .add(
                        egui::Slider::new(&mut degrees, -30.0..=30.0)
                            .max_decimals(3)
                            .text("amplitude")
                            .suffix("°"),
                    )
                    .changed()
                {
                    oscillator.amplitude = degrees.to_radians();
                }

                ui.add(
                    egui::Slider::new(&mut oscillator.harmonic, 0.5..=4.0)
                        .max_decimals(3)
                        .text("harmonic"),
                );

                let mut offset_degrees = oscillator.offset.to_degrees();
                if ui
                    .add(
                        egui::Slider::new(&mut offset_degrees, -180.0..=180.0)
                            .max_decimals(3)
                            .text("offset")
                            .suffix("°"),
                    )
                    .changed()
                {
                    oscillator.offset = offset_degrees.to_radians();
                }

                ui.horizontal(|ui| {
                    ui.label("axis");
                    for (label, component) in [
                        ("x ", &mut oscillator.axis.x),
                        ("y ", &mut oscillator.axis.y),
                        ("z ", &mut oscillator.axis.z),
                    ] {
                        ui.add(
                            egui::DragValue::new(component)
                                .speed(0.01)
                                .max_decimals(4)
                                .prefix(label),
                        );
                    }
                });
            }

            ui.separator();

            if oscillator != before || bone != studio.phase_layer.oscillators[index].0 {
                studio.phase_layer.oscillators[index] = (bone, oscillator);
                changed = true;
            }
        }
    });

    if let Some(index) = remove {
        studio.phase_layer.oscillators.remove(index);
        // The selection is an index into a list that just shrank, so it
        // has to be cleared rather than left pointing at whatever slid
        // into that slot.
        studio.selected_oscillator = None;
        changed = true;
    }

    changed
}

/// Advances the studio's preview clock and pushes it to the character.
///
/// Separate from the panel so the preview keeps running while the window
/// is collapsed — a paused preview behind a collapsed panel looks like the
/// oscillators have stopped working.
pub fn advance_phase_preview(
    studio: Res<StudioState>,
    time: Res<Time>,
    mut phases: Query<&mut GaitPhase>,
) {
    if !studio.open || !studio.phase_open {
        return;
    }

    for mut phase in &mut phases {
        phase.speed = studio.phase_preview_speed;
        phase.advance(time.delta_secs());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_plotted_wave_completes_exactly_its_harmonic_count() {
        // The harmonic is the parameter most easily got wrong by eye, and
        // the classic error is an off-by-one-factor: a head bob authored
        // at 1.0 instead of 2.0 peaks once per stride rather than at each
        // footfall, which reads as a limp.
        //
        // Counting zero crossings over one plotted cycle is the direct
        // check that the plot shows what the number says.
        for harmonic in [1.0f32, 2.0, 3.0] {
            let oscillator = PhaseOscillator {
                clock: PhaseClock::Gait,
                harmonic,
                amplitude: 0.1,
                offset: 0.0,
                axis: Vec3::Z,
            };

            let mut crossings = 0;
            let mut previous = sample_wave(&oscillator, 0.0);
            for sample in 1..CURVE_SAMPLES {
                let t = sample as f32 / (CURVE_SAMPLES - 1) as f32;
                let value = sample_wave(&oscillator, t);
                if previous.signum() != value.signum() {
                    crossings += 1;
                }
                previous = value;
            }

            // A sine crosses zero twice per cycle.
            assert_eq!(
                crossings,
                (harmonic * 2.0) as i32,
                "a harmonic of {harmonic} should cross zero {} times per cycle",
                harmonic * 2.0,
            );
        }
    }

    #[test]
    fn an_offset_of_a_quarter_cycle_turns_a_sine_into_a_cosine() {
        // How hip sway is kept a quarter-cycle out of step with spinal
        // twist — the relationship the plot exists to make visible.
        let sine = PhaseOscillator {
            clock: PhaseClock::Gait,
            harmonic: 1.0,
            amplitude: 1.0,
            offset: 0.0,
            axis: Vec3::Z,
        };
        let cosine = PhaseOscillator {
            offset: std::f32::consts::FRAC_PI_2,
            ..sine
        };

        assert!(sample_wave(&sine, 0.0).abs() < 1.0e-5, "a sine starts at zero");
        assert!(
            (sample_wave(&cosine, 0.0) - 1.0).abs() < 1.0e-5,
            "a quarter-cycle offset starts at its peak",
        );
    }

    #[test]
    fn the_plot_scales_to_the_largest_amplitude_present() {
        // Without this, a two-degree breath beside a twenty-degree sway
        // draws as a flat line, and the author cannot see the thing they
        // are tuning.
        let large = PhaseOscillator {
            clock: PhaseClock::Gait,
            harmonic: 1.0,
            amplitude: 0.35,
            offset: 0.0,
            axis: Vec3::Z,
        };
        let small = PhaseOscillator { amplitude: 0.02, ..large };

        let peak = [large.amplitude, small.amplitude]
            .into_iter()
            .fold(0.0f32, f32::max);

        // The small wave still has a visible extent once scaled.
        let scaled = sample_wave(&small, 0.25) / peak;
        assert!(
            scaled.abs() > 0.01,
            "a small wave should stay visible against a large one, got {scaled}",
        );
    }

    #[test]
    fn every_oscillator_gets_a_stable_colour() {
        // A wave has to keep its identity across frames, or the plot
        // becomes unreadable the moment anything is added.
        for index in 0..12 {
            assert_eq!(wave_colour(index, true), wave_colour(index, true));
        }

        assert_ne!(
            wave_colour(0, true),
            wave_colour(1, true),
            "neighbouring oscillators must be distinguishable",
        );
        assert_ne!(
            wave_colour(0, true),
            wave_colour(0, false),
            "and the selected one must stand out from the rest",
        );
    }

    #[test]
    fn the_shipped_presets_are_plottable() {
        // A guard on the presets themselves: an oscillator with zero
        // amplitude or a degenerate axis contributes nothing and would
        // draw as a flat line, which is indistinguishable from a bug.
        for (name, layer) in [
            ("standing_idle", PhaseLayer::standing_idle()),
            ("locomotion", PhaseLayer::locomotion()),
        ] {
            assert!(!layer.oscillators.is_empty(), "{name} should have oscillators");

            for (bone, oscillator) in &layer.oscillators {
                assert!(
                    oscillator.amplitude.abs() > 1.0e-4,
                    "{name}'s {} oscillator has no amplitude",
                    bone.name(),
                );
                assert!(
                    oscillator.axis.length() > 1.0e-4,
                    "{name}'s {} oscillator has a degenerate axis",
                    bone.name(),
                );
            }
        }
    }
}
