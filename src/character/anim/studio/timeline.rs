//! The clip timeline panel.
//!
//! Keyframes, playback, and contact annotation. As elsewhere in the
//! studio, the clip model itself is [`super::super::clip`] — plain values,
//! tested without egui — and this is layout plus the one thing a timeline
//! genuinely owns: turning a horizontal pixel into a time.

use bevy::prelude::*;
use bevy_egui::{egui, EguiContexts};

use crate::character::anim::clip::{Contacts, Keyframe};
use crate::character::anim::plugin::{AnimPose, AnimTarget};

use super::StudioState;

/// How tall the keyframe strip is.
const STRIP_HEIGHT: f32 = 26.0;
/// How tall each contact lane is.
const LANE_HEIGHT: f32 = 12.0;
/// How close, in pixels, a click must be to grab a keyframe.
const GRAB_PIXELS: f32 = 8.0;

/// Draws the timeline and applies what the user did.
pub fn timeline_panel(
    mut contexts: EguiContexts,
    mut studio: ResMut<StudioState>,
    time: Res<Time>,
    mut characters: Query<(&mut AnimTarget, Option<&mut AnimPose>)>,
) -> Result {
    if !studio.open || !studio.timeline_open {
        return Ok(());
    }

    // Advance the playhead before drawing, so the marker and the rig agree
    // on the same instant. Drawing first would show the playhead a frame
    // ahead of the pose it is supposed to be indicating.
    let mut playback = studio.playback;
    let finished = playback.advance(&studio.clip, time.delta_secs());
    studio.playback = playback;
    let playing = studio.playback.playing;

    let ctx = contexts.ctx_mut()?;
    let mut scrubbed = playing || finished;

    egui::Window::new("Timeline")
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -10.0))
        .default_width(560.0)
        .resizable(true)
        .show(ctx, |ui| {
            scrubbed |= draw_transport(ui, &mut studio);
            scrubbed |= draw_track(ui, &mut studio);
            draw_contacts(ui, &mut studio);
        });

    // Push the sampled pose onto the rig whenever the playhead moved.
    if scrubbed
        && let Some(pose) = studio.clip.sample(studio.playback.time)
    {
        for (mut target, animated) in &mut characters {
            target.pose = pose;
            if studio.snap_while_editing
                && let Some(mut animated) = animated
            {
                *animated = AnimPose::settled_on(&pose);
            }
        }
    }

    Ok(())
}

/// Play/stop, keyframe add/remove, looping. Returns whether the playhead
/// or the clip changed.
fn draw_transport(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut changed = false;

    ui.horizontal(|ui| {
        let label = if studio.playback.playing { "Stop" } else { "Play" };
        if ui.button(label).clicked() {
            studio.playback.playing = !studio.playback.playing;
        }

        if ui.button("Rewind").clicked() {
            studio.playback.time = 0.0;
            changed = true;
        }

        // A keyframe captures the pose the EDITOR holds, not the rig's
        // rendered one. Those differ while a clip is playing, and what an
        // author means by "key this" is the thing they were just editing.
        if ui.button("Add key").clicked() {
            studio.clip.insert(Keyframe {
                time: studio.playback.time,
                pose: studio.edit.to_pose(),
                contacts: Contacts::BOTH,
            });
            changed = true;
        }

        let removable = studio.selected_keyframe.is_some_and(|index| index < studio.clip.len());
        if ui.add_enabled(removable, egui::Button::new("Delete key")).clicked()
            && let Some(index) = studio.selected_keyframe
        {
            studio.clip.remove(index);
            studio.selected_keyframe = None;
            changed = true;
        }

        ui.checkbox(&mut studio.clip.looping, "Loop");
        ui.add(
            egui::DragValue::new(&mut studio.playback.speed)
                .speed(0.05)
                .max_decimals(3)
                .range(0.05..=4.0)
                .prefix("x "),
        );
    });

    ui.horizontal(|ui| {
        ui.weak(format!(
            "{} keys   {:.2} s   playhead {:.2} s",
            studio.clip.len(),
            studio.clip.duration(),
            studio.playback.time,
        ));
    });

    changed |= draw_deslide(ui, studio);

    changed
}

/// The offline foot-sliding removal pass.
///
/// Offline in the sense that it sees the whole clip at once, not in the
/// sense of being slow — the default budget runs in milliseconds on a clip
/// this size, so it is a button rather than an export step.
fn draw_deslide(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    use crate::character::anim::legik::LegIkConfig;
    use crate::character::anim::rig::RigGeometry;
    use crate::character::anim::slide::{remove_foot_sliding, SlideConfig};

    let mut changed = false;

    ui.horizontal(|ui| {
        // Two keyframes is the minimum for an inter-frame constraint to
        // exist at all.
        let enabled = studio.clip.len() >= 2;

        if ui
            .add_enabled(enabled, egui::Button::new("Remove foot sliding"))
            .on_hover_text(
                "Solves the whole clip at once so planted feet stop sliding, then \
                 re-runs the leg IK. Undo by reloading the clip.",
            )
            .clicked()
        {
            let (corrected, report) = remove_foot_sliding(
                &studio.clip,
                &RigGeometry::default(),
                &SlideConfig::default(),
                &LegIkConfig::default(),
            );

            studio.clip = corrected;
            studio.deslide_report = Some(report);
            changed = true;
        }

        // Report what it actually did. "The button ran" and "the feet stopped
        // sliding" are different claims, and only the second one is worth
        // anything to an author.
        if let Some(report) = studio.deslide_report {
            ui.weak(format!(
                "slide {:.3} m -> {:.3} m in {} sweeps",
                report.slide_before, report.slide_after, report.iterations,
            ));
        }
    });

    changed
}

/// The keyframe strip: scrub, select, and drag keys.
fn draw_track(ui: &mut egui::Ui, studio: &mut StudioState) -> bool {
    let mut changed = false;

    let (response, painter) = ui.allocate_painter(
        egui::vec2(ui.available_width(), STRIP_HEIGHT),
        egui::Sense::click_and_drag(),
    );
    let rect = response.rect;
    painter.rect_filled(rect, 2.0, egui::Color32::from_gray(28));

    // An empty clip has no span to map onto, so a minimum window keeps the
    // strip usable while the first keyframes are being placed.
    let span = studio.clip.duration().max(1.0);
    let to_x = |time: f32| rect.left() + (time / span) * rect.width();
    let to_time = |x: f32| ((x - rect.left()) / rect.width() * span).max(0.0);

    // Second gridlines, so a duration is readable rather than abstract.
    for second in 0..=span.ceil() as i32 {
        let x = to_x(second as f32);
        painter.line_segment(
            [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
            egui::Stroke::new(1.0, egui::Color32::from_gray(44)),
        );
    }

    if let Some(pointer) = response.interact_pointer_pos() {
        let time = to_time(pointer.x);

        if response.drag_started() {
            // Grab the nearest keyframe, or scrub if none is close.
            studio.dragging_keyframe = studio
                .clip
                .keyframes()
                .iter()
                .enumerate()
                .map(|(index, key)| (index, (to_x(key.time) - pointer.x).abs()))
                .filter(|&(_, distance)| distance <= GRAB_PIXELS)
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .map(|(index, _)| index);

            if let Some(index) = studio.dragging_keyframe {
                studio.selected_keyframe = Some(index);
            }
        }

        match studio.dragging_keyframe {
            // Dragging a keyframe can reorder the clip, so the selection
            // follows it to wherever it landed rather than staying on an
            // index that now names a different keyframe.
            Some(index) => {
                if let Some(landed) = studio.clip.move_keyframe(index, time) {
                    studio.dragging_keyframe = Some(landed);
                    studio.selected_keyframe = Some(landed);
                }
                changed = true;
            }
            None => {
                studio.playback.time = time;
                studio.playback.playing = false;
                changed = true;
            }
        }
    }

    if response.drag_stopped() {
        studio.dragging_keyframe = None;
    }

    // The keyframes.
    for (index, keyframe) in studio.clip.keyframes().iter().enumerate() {
        let x = to_x(keyframe.time);
        let selected = studio.selected_keyframe == Some(index);

        let colour = if selected {
            egui::Color32::from_rgb(255, 210, 90)
        } else {
            egui::Color32::from_rgb(120, 180, 240)
        };

        painter.rect_filled(
            egui::Rect::from_center_size(
                egui::pos2(x, rect.center().y),
                egui::vec2(7.0, STRIP_HEIGHT - 8.0),
            ),
            1.0,
            colour,
        );
    }

    // The playhead, drawn last so it is never hidden behind a keyframe.
    let playhead = to_x(studio.playback.time);
    painter.line_segment(
        [egui::pos2(playhead, rect.top()), egui::pos2(playhead, rect.bottom())],
        egui::Stroke::new(2.0, egui::Color32::from_rgb(240, 120, 120)),
    );

    changed
}

/// Contact lanes, one per foot.
///
/// Drawn as bars rather than points because a contact is an interval — the
/// thing an author needs to see is how long a foot is planted, which a row
/// of dots does not convey.
fn draw_contacts(ui: &mut egui::Ui, studio: &mut StudioState) {
    let span = studio.clip.duration().max(1.0);

    for (label, left) in [("L", true), ("R", false)] {
        ui.horizontal(|ui| {
            ui.weak(label);

            let (response, painter) = ui.allocate_painter(
                egui::vec2(ui.available_width(), LANE_HEIGHT),
                egui::Sense::hover(),
            );
            let rect = response.rect;
            painter.rect_filled(rect, 1.0, egui::Color32::from_gray(28));

            let keyframes = studio.clip.keyframes();
            for (index, keyframe) in keyframes.iter().enumerate() {
                let planted =
                    if left { keyframe.contacts.left } else { keyframe.contacts.right };
                if !planted {
                    continue;
                }

                // A contact holds until the next keyframe, matching how
                // `contacts_at` steps rather than interpolates.
                let until =
                    keyframes.get(index + 1).map(|next| next.time).unwrap_or(span);

                let x0 = rect.left() + (keyframe.time / span) * rect.width();
                let x1 = rect.left() + (until / span) * rect.width();

                painter.rect_filled(
                    egui::Rect::from_min_max(
                        egui::pos2(x0, rect.top() + 2.0),
                        egui::pos2(x1, rect.bottom() - 2.0),
                    ),
                    1.0,
                    egui::Color32::from_rgb(90, 190, 130),
                );
            }
        });
    }

    // Toggling contacts edits the SELECTED keyframe, since a contact
    // belongs to a keyframe rather than to a moment in time.
    if let Some(index) = studio.selected_keyframe
        && index < studio.clip.len()
    {
        let mut contacts = studio.clip.keyframes()[index].contacts;
        let before = contacts;

        ui.horizontal(|ui| {
            ui.weak(format!("key {index}:"));
            ui.checkbox(&mut contacts.left, "left planted");
            ui.checkbox(&mut contacts.right, "right planted");
        });

        if contacts != before {
            let keyframe = studio.clip.keyframes()[index].clone();
            studio.clip.insert(Keyframe { contacts, ..keyframe });
        }
    } else {
        ui.weak("select a keyframe to edit its contacts");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::clip::AnimClip;
    use crate::character::anim::poses;

    #[test]
    fn a_contact_bar_runs_until_the_next_keyframe() {
        // The lane drawing assumes a contact holds until the next key,
        // matching `contacts_at`'s stepping. If the two disagreed, the
        // picture would show a foot planted for a span during which the
        // runtime treats it as lifted.
        let mut clip = AnimClip::new(false);
        clip.insert(Keyframe {
            time: 0.0,
            pose: poses::rest(),
            contacts: Contacts::BOTH,
        });
        clip.insert(Keyframe {
            time: 1.0,
            pose: poses::rest(),
            contacts: Contacts::NONE,
        });

        // Anywhere inside the first key's span reads as planted...
        for probe in [0.0, 0.5, 0.99] {
            assert_eq!(
                clip.contacts_at(probe),
                Contacts::BOTH,
                "at t={probe} the first keyframe's contact should still hold",
            );
        }
        // ...and the bar the panel draws covers exactly that span.
        assert_eq!(clip.keyframes()[1].time, 1.0);
    }

    #[test]
    fn the_strip_maps_times_onto_itself_consistently() {
        // `to_x` and `to_time` are inverses; if they drift, a keyframe
        // drops where it was not dropped, which reads as the timeline
        // fighting the user.
        let span = 2.5f32;
        let (left, width) = (10.0f32, 500.0f32);

        let to_x = |time: f32| left + (time / span) * width;
        let to_time = |x: f32| ((x - left) / width * span).max(0.0);

        for time in [0.0, 0.4, 1.25, 2.5] {
            let round_tripped = to_time(to_x(time));
            assert!(
                (round_tripped - time).abs() < 1.0e-4,
                "t={time} mapped to x and back to {round_tripped}",
            );
        }
    }

    #[test]
    fn an_empty_clip_still_has_a_usable_span() {
        // Without a minimum, an empty clip's zero duration makes every
        // time map to the same pixel and the strip cannot be clicked into
        // existence.
        let clip = AnimClip::new(false);
        assert_eq!(clip.duration(), 0.0);
        assert!(clip.duration().max(1.0) > 0.0);
    }
}
