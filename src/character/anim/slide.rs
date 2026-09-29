//! Offline foot-sliding removal — a whole-clip constraint solve.
//!
//! # Why this exists alongside the runtime foot lock
//!
//! [`super::footlock`] fixes sliding *causally*: it sees only the frames
//! that have already happened, so it can pin a foot but cannot know that the
//! pin will need to be somewhere else in forty frames' time. It is the right
//! tool at runtime, where the future genuinely is unknown.
//!
//! An authored clip has no such excuse. Every frame is available at once, so
//! the error can be distributed across the whole contact rather than
//! absorbed at its edges — and the result is baked in, costing nothing at
//! runtime.
//!
//! # The method
//!
//! Position-based dynamics, in the plainest form: hold three arrays of world
//! positions (pelvis, left toe, right toe), then relax a set of constraints
//! over them by repeated Gauss-Seidel sweeps. Each sweep nudges positions a
//! fraction of the way toward satisfying each constraint; thousands of
//! sweeps converge on a configuration that satisfies all of them as well as
//! they can be simultaneously satisfied.
//!
//! Three constraints, following Daniel Holden's formulation:
//!
//! 1. **Contact coherence (strong).** Where consecutive frames are both in
//!    contact, the two toe positions are pulled toward their shared midpoint
//!    and onto the ground. This is what actually removes the slide.
//! 2. **Motion preservation (weak).** Everywhere else, each frame's toe and
//!    the pelvis are pulled toward the position that reproduces the
//!    *original* frame-to-frame offset. Without it the solve would happily
//!    flatten the whole animation into one motionless pose, which satisfies
//!    constraint 1 perfectly.
//! 3. **Limb length (weak).** Pelvis and toe are pulled toward preserving
//!    their original separation, so the legs are not asked to stretch.
//!
//! The strong/weak split is the whole design. Contact is the property worth
//! enforcing exactly; everything else is a preference that should yield to
//! it, and the 18x ratio between the two factors is what encodes that.
//!
//! # What this does not do
//!
//! It produces corrected *positions*. Turning those back into a posed
//! skeleton is [`super::legik`]'s job — see [`resolve_clip`], which runs the
//! leg IK for each frame against the solved targets.

use bevy::math::Vec3;

/// One frame's extracted positions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlideFrame {
    /// The pelvis, world space.
    pub pelvis: Vec3,
    /// The left toe, world space.
    pub left_toe: Vec3,
    /// The right toe.
    pub right_toe: Vec3,
    /// How confident we are that the left foot is planted, 0 to 1.
    ///
    /// Continuous rather than boolean, because
    /// [`super::footlock::annotate_contacts`]' output is worth smoothing
    /// before it drives a constraint: a contact that switches on for one
    /// frame produces a one-frame pin, which reads as a pop.
    pub left_contact: f32,
    /// The same for the right foot.
    pub right_contact: f32,
}

/// How hard each constraint pulls, and for how long.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlideConfig {
    /// How far to move toward satisfying a *contact* constraint each sweep.
    ///
    /// Near 1: contact is the property the whole solve exists to enforce, so
    /// it wins essentially every contest it enters.
    pub hard_factor: f32,
    /// How far to move toward satisfying the motion-preservation and
    /// limb-length constraints each sweep.
    ///
    /// Deliberately tiny. These exist to stop the solve collapsing the
    /// animation, not to compete with contact; a large value here fights
    /// the pinning and reintroduces the slide.
    pub soft_factor: f32,
    /// Contact confidence above which a frame counts as planted.
    pub contact_threshold: f32,
    /// How many Gauss-Seidel sweeps to run.
    ///
    /// The article specifies 25,000, and that is kept as the default — but
    /// it is a budget, not a measured requirement, and the measurement is
    /// worth knowing before paying for it: on a real posed clip sliding is
    /// already down to 0.0124 m after **1,000** sweeps, and the remaining
    /// 24,000 refine it to 0.0033 m. Twenty-five times the work for a
    /// four-fold improvement on a quantity that was invisible either way.
    ///
    /// Drop it for interactive use; keep it for a final bake. Either way
    /// [`SlideReport`] says what was actually achieved.
    pub iterations: usize,
    /// The lowest a toe may sit, world space — the bind pose's own toe
    /// height, as [`super::plugin`] uses for the runtime clamp.
    pub toe_min_height: f32,
}

impl Default for SlideConfig {
    fn default() -> Self {
        Self {
            hard_factor: 0.9,
            soft_factor: 0.05,
            contact_threshold: 0.5,
            iterations: 25_000,
            toe_min_height: 0.0,
        }
    }
}

/// What the solve did, so a caller can tell a converged result from a
/// truncated one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlideReport {
    /// Sweeps actually run. Below `iterations` when the solve converged
    /// early.
    pub iterations: usize,
    /// The largest single-position movement in the final sweep, metres.
    /// Small means converged.
    pub final_movement: f32,
    /// Total sliding before the solve — the summed frame-to-frame toe travel
    /// during contact, which is exactly the quantity being removed.
    pub slide_before: f32,
    /// The same measured after.
    pub slide_after: f32,
}

/// Removes foot sliding from a sequence of extracted frames, in place.
///
/// Returns a report; see [`SlideReport`].
///
/// Sliding is measured and reported rather than merely assumed to have
/// improved, because "the constraint ran" and "the feet stopped sliding" are
/// different claims and only the second one matters.
pub fn solve_sliding(frames: &mut [SlideFrame], config: &SlideConfig) -> SlideReport {
    let slide_before = total_contact_slide(frames, config);

    if frames.len() < 2 {
        return SlideReport {
            iterations: 0,
            final_movement: 0.0,
            slide_before,
            slide_after: slide_before,
        };
    }

    // The original animation, kept intact: every "preserve the motion"
    // constraint is phrased against it, so it must not drift as the solve
    // proceeds.
    let original: Vec<SlideFrame> = frames.to_vec();

    // Converged when the worst movement in a whole sweep is this small. A
    // tenth of a millimetre is far below anything visible, and stopping
    // there rather than burning the remaining budget is free.
    const CONVERGED: f32 = 1.0e-4;

    let mut iterations = 0;
    let mut movement = 0.0;

    for sweep in 0..config.iterations {
        movement = relax_once(frames, &original, config);
        iterations = sweep + 1;

        if movement < CONVERGED {
            break;
        }
    }

    SlideReport {
        iterations,
        final_movement: movement,
        slide_before,
        slide_after: total_contact_slide(frames, config),
    }
}

/// One Gauss-Seidel sweep over every frame. Returns the largest single
/// position movement, for convergence testing.
///
/// Gauss-Seidel rather than Jacobi — each constraint reads the positions the
/// previous one just wrote, within the same sweep. It converges in
/// substantially fewer sweeps than the alternative and needs no second
/// buffer.
fn relax_once(
    frames: &mut [SlideFrame],
    original: &[SlideFrame],
    config: &SlideConfig,
) -> f32 {
    let mut worst = 0.0f32;

    for i in 1..frames.len() {
        for side in [Side::Left, Side::Right] {
            worst = worst.max(relax_inter_frame(frames, original, i, side, config));
        }

        worst = worst.max(relax_pelvis_motion(frames, original, i, config));
    }

    for i in 0..frames.len() {
        for side in [Side::Left, Side::Right] {
            worst = worst.max(relax_limb_length(frames, original, i, side, config));
        }
    }

    worst
}

/// Which foot a constraint is acting on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Side {
    Left,
    Right,
}

impl Side {
    fn toe(self, frame: &SlideFrame) -> Vec3 {
        match self {
            Side::Left => frame.left_toe,
            Side::Right => frame.right_toe,
        }
    }

    fn set_toe(self, frame: &mut SlideFrame, value: Vec3) {
        match self {
            Side::Left => frame.left_toe = value,
            Side::Right => frame.right_toe = value,
        }
    }

    fn contact(self, frame: &SlideFrame) -> f32 {
        match self {
            Side::Left => frame.left_contact,
            Side::Right => frame.right_contact,
        }
    }
}

/// The constraint between frames `i - 1` and `i` for one foot.
///
/// In contact: both toes move most of the way to their shared midpoint,
/// which is what actually removes the slide — two frames that agree on where
/// the foot is cannot slide between them.
///
/// Out of contact: each toe moves a little toward the position that
/// reproduces the ORIGINAL frame-to-frame offset, which preserves the swing.
fn relax_inter_frame(
    frames: &mut [SlideFrame],
    original: &[SlideFrame],
    i: usize,
    side: Side,
    config: &SlideConfig,
) -> f32 {
    let previous = side.toe(&frames[i - 1]);
    let current = side.toe(&frames[i]);

    let in_contact = side.contact(&frames[i - 1]) > config.contact_threshold
        && side.contact(&frames[i]) > config.contact_threshold;

    let (previous_target, current_target, factor) = if in_contact {
        // Both frames pulled to the midpoint of where they currently are,
        // clamped onto the ground. The midpoint uses the CURRENT positions,
        // so a long contact converges to one shared point rather than to
        // whatever the original animation happened to average.
        let midpoint = previous.lerp(current, 0.5);
        let target = Vec3::new(
            midpoint.x,
            midpoint.y.max(config.toe_min_height),
            midpoint.z,
        );
        (target, target, config.hard_factor)
    } else {
        // Preserve the original step. Each frame is pulled toward the other
        // frame's CURRENT position displaced by the ORIGINAL offset, so the
        // shape of the motion survives even as the whole thing shifts.
        let offset = side.toe(&original[i]) - side.toe(&original[i - 1]);
        (
            ground(current - offset, config),
            ground(previous + offset, config),
            config.soft_factor,
        )
    };

    let moved_previous = previous.lerp(previous_target, factor);
    let moved_current = current.lerp(current_target, factor);

    side.set_toe(&mut frames[i - 1], moved_previous);
    side.set_toe(&mut frames[i], moved_current);

    moved_previous.distance(previous).max(moved_current.distance(current))
}

/// The pelvis's own inter-frame constraint: keep the original frame-to-frame
/// motion.
///
/// Weak, like the toe's out-of-contact case. The pelvis has no contact
/// constraint of its own — it is pulled around only by the limb-length
/// constraint — so without this it would drift freely.
fn relax_pelvis_motion(
    frames: &mut [SlideFrame],
    original: &[SlideFrame],
    i: usize,
    config: &SlideConfig,
) -> f32 {
    let previous = frames[i - 1].pelvis;
    let current = frames[i].pelvis;
    let offset = original[i].pelvis - original[i - 1].pelvis;

    let moved_previous = previous.lerp(current - offset, config.soft_factor);
    let moved_current = current.lerp(previous + offset, config.soft_factor);

    frames[i - 1].pelvis = moved_previous;
    frames[i].pelvis = moved_current;

    moved_previous.distance(previous).max(moved_current.distance(current))
}

/// Keeps one frame's pelvis-to-toe distance at what the original animation
/// had, by moving both ends toward it.
///
/// This is the constraint that stops the solve stretching a leg: pinning a
/// toe in place while the body walks past would otherwise be satisfied
/// perfectly by an infinitely long leg.
fn relax_limb_length(
    frames: &mut [SlideFrame],
    original: &[SlideFrame],
    i: usize,
    side: Side,
    config: &SlideConfig,
) -> f32 {
    let rest_length = original[i].pelvis.distance(side.toe(&original[i]));

    let pelvis = frames[i].pelvis;
    let toe = side.toe(&frames[i]);

    let separation = pelvis - toe;
    let distance = separation.length();
    if distance < 1.0e-6 {
        return 0.0;
    }
    let direction = separation / distance;

    // Each end moves toward where it would be if the other end were fixed.
    // Both move, so neither is privileged — the error is shared.
    let moved_pelvis = pelvis.lerp(toe + direction * rest_length, config.soft_factor);
    let moved_toe =
        toe.lerp(ground(pelvis - direction * rest_length, config), config.soft_factor);

    frames[i].pelvis = moved_pelvis;
    side.set_toe(&mut frames[i], moved_toe);

    moved_pelvis.distance(pelvis).max(moved_toe.distance(toe))
}

/// Lifts a position to the minimum toe height if it is below it.
fn ground(position: Vec3, config: &SlideConfig) -> Vec3 {
    Vec3::new(position.x, position.y.max(config.toe_min_height), position.z)
}

/// How far the toes travel between consecutive frames while in contact —
/// the sliding, summed over the clip.
///
/// This is the objective function: it is what the solve exists to minimise,
/// and comparing it before and after is the only honest way to say the solve
/// worked.
pub fn total_contact_slide(frames: &[SlideFrame], config: &SlideConfig) -> f32 {
    let mut total = 0.0;

    for i in 1..frames.len() {
        for side in [Side::Left, Side::Right] {
            if side.contact(&frames[i - 1]) > config.contact_threshold
                && side.contact(&frames[i]) > config.contact_threshold
            {
                total += side.toe(&frames[i]).distance(side.toe(&frames[i - 1]));
            }
        }
    }

    total
}

// ---------------------------------------------------------------------
// The clip pipeline: extract -> solve -> resolve
// ---------------------------------------------------------------------

use super::clip::AnimClip;
use super::legik::{solve_leg_grounded, LegChain, LegIkConfig};
use super::rig::{forward_kinematics_on, RigGeometry};
use crate::character::skeleton::Bone;

/// Reads the positions [`solve_sliding`] works on out of an authored clip.
///
/// Contact confidence comes from the clip's own annotations, as 0 or 1.
/// A caller with a smoothed contact signal — from
/// [`super::footlock::annotate_contacts`] followed by a blur — can overwrite
/// the fields afterwards; the solver reads them as continuous values.
pub fn extract_frames(clip: &AnimClip, rig: &RigGeometry) -> Vec<SlideFrame> {
    clip.keyframes()
        .iter()
        .map(|keyframe| {
            let positions = forward_kinematics_on(&keyframe.pose, rig);
            SlideFrame {
                pelvis: positions[Bone::Hips],
                left_toe: positions[Bone::LeftToeBase],
                right_toe: positions[Bone::RightToeBase],
                left_contact: if keyframe.contacts.left { 1.0 } else { 0.0 },
                right_contact: if keyframe.contacts.right { 1.0 } else { 0.0 },
            }
        })
        .collect()
}

/// Rebuilds a clip's poses from solved positions, by running the leg IK for
/// each keyframe against its corrected toe targets.
///
/// This is the step that turns positions back into rotations. The pelvis
/// correction is applied as root translation, then each leg is solved to its
/// own target — the same solver the runtime uses, so an offline-corrected
/// clip and a runtime-corrected pose cannot disagree about what a leg does.
///
/// Returns a new clip; the input is left alone.
pub fn resolve_clip(
    clip: &AnimClip,
    frames: &[SlideFrame],
    rig: &RigGeometry,
    ik: &LegIkConfig,
) -> AnimClip {
    let mut resolved = AnimClip::new(clip.looping);

    for (keyframe, frame) in clip.keyframes().iter().zip(frames) {
        let mut pose = keyframe.pose;

        // The pelvis moves by whatever the solve decided, as a root
        // translation — the one channel a `LocalPose` has for it.
        let original = forward_kinematics_on(&pose, rig)[Bone::Hips];
        pose.root_translation += frame.pelvis - original;

        // Then each leg reaches for its corrected toe.
        //
        // Ground is `None` deliberately. The solve has already decided where
        // every toe belongs, including its height — it ran its own
        // `toe_min_height` clamp across the whole clip. Handing the leg
        // solver a ground probe here would let it clamp and tilt those
        // positions a second time, on a per-frame basis, undoing the
        // clip-wide agreement the solve just reached.
        for (chain, target) in [
            (LegChain::LEFT, frame.left_toe),
            (LegChain::RIGHT, frame.right_toe),
        ] {
            solve_leg_grounded(&mut pose, chain, target, None, ik, rig);
        }

        resolved.insert(super::clip::Keyframe {
            time: keyframe.time,
            pose,
            contacts: keyframe.contacts,
        });
    }

    resolved
}

/// The whole offline pass: extract, solve, resolve.
///
/// The convenience entry point, and the one a tool should call. Returns the
/// corrected clip and the solve's own report so the caller can log what
/// actually changed rather than assume.
pub fn remove_foot_sliding(
    clip: &AnimClip,
    rig: &RigGeometry,
    config: &SlideConfig,
    ik: &LegIkConfig,
) -> (AnimClip, SlideReport) {
    let mut frames = extract_frames(clip, rig);
    let report = solve_sliding(&mut frames, config);
    (resolve_clip(clip, &frames, rig, ik), report)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A clip of a foot that should be planted but drifts — the canonical
    /// sliding artefact.
    ///
    /// The left foot is annotated as in contact throughout while its position
    /// creeps backward, exactly as a foot does when the root moves at a speed
    /// the animation was not authored for.
    fn sliding_clip(frames: usize, drift_per_frame: f32) -> Vec<SlideFrame> {
        (0..frames)
            .map(|i| {
                let t = i as f32;
                SlideFrame {
                    pelvis: Vec3::new(0.0, 0.95, -t * drift_per_frame),
                    left_toe: Vec3::new(-0.1, 0.0, -t * drift_per_frame),
                    right_toe: Vec3::new(0.1, 0.0, 0.0),
                    left_contact: 1.0,
                    right_contact: 0.0,
                }
            })
            .collect()
    }

    #[test]
    fn a_planted_foot_stops_sliding() {
        // THE property. A foot annotated as in contact must end up in one
        // place, however far the source animation dragged it.
        let mut frames = sliding_clip(30, 0.01);
        let config = SlideConfig::default();

        let report = solve_sliding(&mut frames, &config);

        assert!(
            report.slide_before > 0.25,
            "test setup: the clip should start with real sliding, got {}",
            report.slide_before,
        );
        assert!(
            report.slide_after < report.slide_before * 0.05,
            "sliding should be almost entirely removed, but went from {} to {}",
            report.slide_before,
            report.slide_after,
        );
    }

    #[test]
    fn most_of_the_iteration_budget_buys_very_little() {
        // The article specifies 25,000 iterations. That is a budget, not a
        // measured requirement, and measuring it matters because the cost is
        // linear in it.
        //
        // What the measurement actually shows is NOT clean early
        // convergence: on a synthetic clip the solve hits the 1e-4 movement
        // threshold at ~4,070 sweeps, but on a real posed clip it never does
        // — the tail converges asymptotically and the final sweep is still
        // moving positions by ~1.8e-4 m at 25,000.
        //
        // The useful finding is that it does not matter. Sliding is already
        // down to 0.0124 m at 1,000 sweeps, and the remaining 24,000 buy a
        // refinement to 0.0033 m — 25x the work for a 4x improvement on a
        // quantity that was already far below visible.
        let mut frames = sliding_clip(30, 0.01);
        let full = solve_sliding(&mut frames, &SlideConfig::default());

        let mut cheap = sliding_clip(30, 0.01);
        let cheap_report = solve_sliding(
            &mut cheap,
            &SlideConfig { iterations: 1_000, ..Default::default() },
        );

        assert!(
            cheap_report.slide_after < cheap_report.slide_before * 0.1,
            "1,000 sweeps should already remove most of the slide: {} -> {}",
            cheap_report.slide_before,
            cheap_report.slide_after,
        );

        println!(
            "1,000 sweeps: {:.5} m   |   {} sweeps: {:.5} m",
            cheap_report.slide_after, full.iterations, full.slide_after,
        );
    }

    #[test]
    fn a_clip_with_no_contact_is_left_essentially_alone() {
        // The control. With nothing planted there is no sliding to remove,
        // and the motion-preservation constraints should hold the animation
        // where it is.
        let mut frames = sliding_clip(20, 0.01);
        for frame in &mut frames {
            frame.left_contact = 0.0;
        }
        let before = frames.clone();

        solve_sliding(&mut frames, &SlideConfig::default());

        for (i, (after, original)) in frames.iter().zip(&before).enumerate() {
            assert!(
                after.left_toe.distance(original.left_toe) < 0.01,
                "frame {i}'s toe moved {} m with nothing in contact",
                after.left_toe.distance(original.left_toe),
            );
        }
    }

    #[test]
    fn a_swing_next_to_a_contact_is_carried_along_rather_than_crushed() {
        // The out-of-contact constraint's own test, and getting it to bite
        // took understanding what the constraint actually IS.
        //
        // It pulls each frame toward "the neighbouring frame's CURRENT
        // position, displaced by the ORIGINAL offset". When the positions
        // already reproduce the original offsets, that target IS the current
        // position and the whole term is a no-op — which is why a free swing
        // in isolation tests nothing, and why an earlier version of this
        // test passed with the term deleted entirely.
        //
        // It is a restoring force. It only does work once something else has
        // perturbed the positions, and the only thing that does that is the
        // contact constraint pulling from the far end. So: a contact that
        // moves a long way, and a swing hanging off it that must be carried
        // along intact rather than dragged in.
        let contact_frames = 12;
        let swing_frames = 12;

        let mut frames: Vec<SlideFrame> = Vec::new();

        // A planted foot that the source animation drags backwards a long
        // way — the contact constraint will pull all of these together.
        for i in 0..contact_frames {
            frames.push(SlideFrame {
                pelvis: Vec3::new(0.0, 0.95, -(i as f32) * 0.02),
                left_toe: Vec3::new(-0.1, 0.0, -(i as f32) * 0.02),
                right_toe: Vec3::new(0.1, 0.0, 0.0),
                left_contact: 1.0,
                right_contact: 0.0,
            });
        }

        // Then a swing with a distinctive, even step of 0.03 m per frame.
        let swing_start = frames[contact_frames - 1].left_toe;
        for i in 0..swing_frames {
            let t = (i + 1) as f32;
            frames.push(SlideFrame {
                pelvis: Vec3::new(0.0, 0.95, -(contact_frames as f32 + t) * 0.02),
                left_toe: swing_start + Vec3::new(0.0, 0.0, -t * 0.03),
                right_toe: Vec3::new(0.1, 0.0, 0.0),
                left_contact: 0.0,
                right_contact: 1.0,
            });
        }

        solve_sliding(&mut frames, &SlideConfig::default());

        // The observable effect is at the SEAM. Pinning the contact moves
        // its last frame a long way from where the animation had it, and
        // without the motion-preservation term the swing frames are coupled
        // to nothing — they stay exactly where the input put them, leaving a
        // large jump at the boundary. With it, that displacement is carried
        // into the swing and the seam step stays close to the authored one.
        //
        // Measured: 0.140 m at the seam with the term disabled, against an
        // authored 0.03 m step.
        let seam = frames[contact_frames].left_toe.distance(
            frames[contact_frames - 1].left_toe,
        );

        assert!(
            seam < 0.06,
            "the contact-to-swing seam jumped {seam} m against an authored 0.03 m \
             step — the swing is not being carried along with the pinned contact",
        );

        // And the swing's own steps keep their shape.
        for i in contact_frames + 1..frames.len() {
            let step = frames[i].left_toe.distance(frames[i - 1].left_toe);
            assert!(
                step > 0.02,
                "swing step {i} collapsed to {step} m from an authored 0.03 m",
            );
        }
    }

    #[test]
    fn the_swing_phase_keeps_its_shape() {
        // The constraint that stops the solve collapsing everything to a
        // point. A foot that is NOT in contact must still travel the
        // distance the original animation gave it.
        let mut frames: Vec<SlideFrame> = (0..40)
            .map(|i| {
                let t = i as f32;
                // Twenty frames planted, twenty swinging forward.
                let planted = i < 20;
                SlideFrame {
                    pelvis: Vec3::new(0.0, 0.95, -t * 0.01),
                    left_toe: if planted {
                        Vec3::new(-0.1, 0.0, -t * 0.005)
                    } else {
                        Vec3::new(-0.1, 0.05, -0.1 - (t - 20.0) * 0.02)
                    },
                    right_toe: Vec3::new(0.1, 0.0, 0.0),
                    left_contact: if planted { 1.0 } else { 0.0 },
                    right_contact: 1.0,
                }
            })
            .collect();

        let travel_before =
            frames[39].left_toe.distance(frames[20].left_toe);

        solve_sliding(&mut frames, &SlideConfig::default());

        let travel_after = frames[39].left_toe.distance(frames[20].left_toe);

        assert!(
            travel_after > travel_before * 0.8,
            "the swing should keep its travel, but it shrank from \
             {travel_before} to {travel_after}",
        );
    }

    #[test]
    fn a_toe_is_never_pushed_below_the_ground() {
        let mut frames = sliding_clip(20, 0.01);
        for frame in &mut frames {
            frame.left_toe.y = -0.05;
        }

        let config = SlideConfig { toe_min_height: 0.0, ..Default::default() };
        solve_sliding(&mut frames, &config);

        for (i, frame) in frames.iter().enumerate() {
            assert!(
                frame.left_toe.y >= -1.0e-4,
                "frame {i}'s toe sat at y={} after the solve",
                frame.left_toe.y,
            );
        }
    }

    #[test]
    fn the_toe_min_height_is_respected_above_the_origin() {
        let mut frames = sliding_clip(20, 0.01);
        let config = SlideConfig { toe_min_height: 0.25, ..Default::default() };

        solve_sliding(&mut frames, &config);

        for frame in &frames {
            assert!(
                frame.left_toe.y >= 0.25 - 1.0e-4,
                "the toe sank to {} below a 0.25 m floor",
                frame.left_toe.y,
            );
        }
    }

    #[test]
    fn the_legs_are_not_stretched_to_pin_a_foot() {
        // The failure mode the limb-length constraint exists to prevent:
        // pinning a toe while the body walks past is satisfied perfectly by
        // an infinitely long leg.
        let mut frames = sliding_clip(30, 0.02);
        let original = frames.clone();

        solve_sliding(&mut frames, &SlideConfig::default());

        let mut worst = 0.0f32;
        for (after, before) in frames.iter().zip(&original) {
            let length_before = before.pelvis.distance(before.left_toe);
            let length_after = after.pelvis.distance(after.left_toe);
            worst = worst.max((length_after - length_before).abs());
        }

        // Bounded at a value that actually discriminates. Measured: 0.0043 m
        // with the limb-length constraint, 0.0427 m without it — so a
        // tolerance anywhere above 0.043 passes either way, and an earlier
        // version of this test used 0.12 and did exactly that.
        assert!(
            worst < 0.02,
            "the worst leg-length change was {worst} m — legs are being stretched \
             to pin the feet",
        );
    }

    #[test]
    fn both_feet_are_solved_independently() {
        // A left foot in contact must not drag the right one around.
        let mut frames: Vec<SlideFrame> = (0..20)
            .map(|i| SlideFrame {
                pelvis: Vec3::new(0.0, 0.95, 0.0),
                left_toe: Vec3::new(-0.1, 0.0, -(i as f32) * 0.01),
                right_toe: Vec3::new(0.1, 0.0, -(i as f32) * 0.01),
                left_contact: 1.0,
                right_contact: 0.0,
            })
            .collect();
        let original = frames.clone();

        solve_sliding(&mut frames, &SlideConfig::default());

        // The left foot, in contact, should have been pinned.
        let left_travel = frames[19].left_toe.distance(frames[0].left_toe);
        assert!(left_travel < 0.02, "the planted left toe still travelled {left_travel} m");

        // The right foot, free, should still be travelling.
        let right_travel = frames[19].right_toe.distance(frames[0].right_toe);
        let right_before = original[19].right_toe.distance(original[0].right_toe);
        assert!(
            right_travel > right_before * 0.8,
            "the free right toe lost its motion: {right_before} -> {right_travel}",
        );
    }

    #[test]
    fn the_contact_threshold_decides_what_counts_as_planted() {
        // Contact is a confidence value, not a boolean, so the threshold is
        // what turns it into a decision.
        let mut low = sliding_clip(20, 0.01);
        let mut high = low.clone();
        for frame in low.iter_mut().chain(high.iter_mut()) {
            frame.left_contact = 0.4;
        }

        let permissive = SlideConfig { contact_threshold: 0.3, ..Default::default() };
        let strict = SlideConfig { contact_threshold: 0.7, ..Default::default() };

        let permissive_report = solve_sliding(&mut low, &permissive);
        let strict_report = solve_sliding(&mut high, &strict);

        assert!(
            permissive_report.slide_after < permissive_report.slide_before * 0.1,
            "a 0.4 confidence should count as contact below a 0.3 threshold",
        );
        assert_eq!(
            strict_report.slide_before, 0.0,
            "...and not register as contact at all above a 0.7 one",
        );
    }

    #[test]
    fn the_solve_is_bit_identical_across_runs() {
        // Determinism is a stated requirement for this stack, and it matters
        // more for a BAKE than for anything at runtime: a pass that produced
        // slightly different results each time would make an authored clip
        // depend on when it happened to be exported.
        //
        // There is no RNG here and no hash iteration — the solve is a fixed
        // sequence of float operations over arrays — so this is cheap
        // insurance rather than a live worry.
        let config = SlideConfig::default();

        let mut first = sliding_clip(25, 0.012);
        let mut second = sliding_clip(25, 0.012);

        let report_a = solve_sliding(&mut first, &config);
        let report_b = solve_sliding(&mut second, &config);

        assert_eq!(report_a.iterations, report_b.iterations);
        assert_eq!(report_a.slide_after, report_b.slide_after);

        for (i, (a, b)) in first.iter().zip(&second).enumerate() {
            assert_eq!(a, b, "frame {i} differed between two identical runs");
        }
    }

    #[test]
    fn an_empty_or_single_frame_clip_is_safe() {
        let config = SlideConfig::default();

        let mut empty: Vec<SlideFrame> = Vec::new();
        let report = solve_sliding(&mut empty, &config);
        assert_eq!(report.iterations, 0);

        let mut single = sliding_clip(1, 0.0);
        let report = solve_sliding(&mut single, &config);
        assert_eq!(report.iterations, 0, "one frame has no inter-frame constraint");
    }

    #[test]
    fn the_solve_never_produces_nan() {
        let config = SlideConfig::default();

        // Degenerate inputs: a pelvis exactly on the toe (zero-length limb),
        // and identical frames.
        let mut frames: Vec<SlideFrame> = (0..10)
            .map(|_| SlideFrame {
                pelvis: Vec3::ZERO,
                left_toe: Vec3::ZERO,
                right_toe: Vec3::ZERO,
                left_contact: 1.0,
                right_contact: 1.0,
            })
            .collect();

        solve_sliding(&mut frames, &config);

        for (i, frame) in frames.iter().enumerate() {
            assert!(
                frame.pelvis.is_finite()
                    && frame.left_toe.is_finite()
                    && frame.right_toe.is_finite(),
                "frame {i} went non-finite: {frame:?}",
            );
        }
    }

    #[test]
    fn a_zero_iteration_budget_changes_nothing() {
        let mut frames = sliding_clip(20, 0.01);
        let before = frames.clone();

        let config = SlideConfig { iterations: 0, ..Default::default() };
        let report = solve_sliding(&mut frames, &config);

        assert_eq!(frames, before, "no sweeps should mean no change");
        assert_eq!(report.slide_before, report.slide_after);
    }

    // -----------------------------------------------------------------
    // The clip pipeline
    // -----------------------------------------------------------------

    use crate::character::anim::clip::{Contacts, Keyframe};
    use crate::character::anim::rig::LocalPose;
    use crate::character::anim::stance::stance;

    /// A clip whose planted left foot slides, built from real poses rather
    /// than synthetic position arrays.
    ///
    /// The slide is produced the way a real one is: the root translates while
    /// the leg pose stays put, so the foot is dragged along the ground.
    fn sliding_pose_clip(frames: usize) -> AnimClip {
        let mut clip = AnimClip::new(false);
        let base = stance(&LocalPose::REST);

        for i in 0..frames {
            let mut pose = base;
            pose.root_translation.z -= i as f32 * 0.01;

            clip.insert(Keyframe {
                time: i as f32 / 30.0,
                pose,
                contacts: Contacts { left: true, right: false },
            });
        }

        clip
    }

    #[test]
    fn the_whole_pipeline_removes_sliding_from_a_posed_clip() {
        // End to end, through real poses and the real leg IK — the claim
        // that matters, as opposed to "the array solver converged".
        let rig = RigGeometry::default();
        let clip = sliding_pose_clip(30);

        let (corrected, report) = remove_foot_sliding(
            &clip,
            &rig,
            &SlideConfig::default(),
            &LegIkConfig::default(),
        );

        assert!(
            report.slide_before > 0.25,
            "test setup: the clip should slide, got {} m",
            report.slide_before,
        );
        assert!(
            report.slide_after < report.slide_before * 0.05,
            "the solve should remove the slide: {} -> {}",
            report.slide_before,
            report.slide_after,
        );

        // And the CORRECTED CLIP's own rendered toe positions must be the
        // ones that stopped moving — the solver's arrays agreeing with
        // themselves proves nothing about the poses that come out.
        let toes: Vec<Vec3> = corrected
            .keyframes()
            .iter()
            .map(|k| forward_kinematics_on(&k.pose, &rig)[Bone::LeftToeBase])
            .collect();

        let travel: f32 =
            toes.windows(2).map(|w| w[1].distance(w[0])).sum();

        let before: f32 = clip
            .keyframes()
            .iter()
            .map(|k| forward_kinematics_on(&k.pose, &rig)[Bone::LeftToeBase])
            .collect::<Vec<_>>()
            .windows(2)
            .map(|w| w[1].distance(w[0]))
            .sum();

        assert!(
            travel < before * 0.25,
            "the resolved clip's toe still travels {travel} m against {before} m \
             before — the corrected positions are not reaching the poses",
        );
    }

    #[test]
    fn the_pipeline_preserves_the_clips_timing_and_contacts() {
        let rig = RigGeometry::default();
        let clip = sliding_pose_clip(12);

        let (corrected, _) = remove_foot_sliding(
            &clip,
            &rig,
            &SlideConfig::default(),
            &LegIkConfig::default(),
        );

        assert_eq!(corrected.len(), clip.len(), "keyframe count must not change");

        for (after, before) in corrected.keyframes().iter().zip(clip.keyframes()) {
            assert_eq!(after.time, before.time, "timing must be untouched");
            assert_eq!(after.contacts, before.contacts, "contacts must be untouched");
        }
    }

    #[test]
    fn extraction_reads_the_positions_the_poses_actually_produce() {
        // Pins the extract step against forward kinematics directly, so a
        // wrong bone or a dropped root translation shows up here rather than
        // as a mysteriously bad solve.
        let rig = RigGeometry::default();
        let clip = sliding_pose_clip(5);

        let frames = extract_frames(&clip, &rig);
        assert_eq!(frames.len(), clip.len());

        for (frame, keyframe) in frames.iter().zip(clip.keyframes()) {
            let positions = forward_kinematics_on(&keyframe.pose, &rig);
            assert_eq!(frame.pelvis, positions[Bone::Hips]);
            assert_eq!(frame.left_toe, positions[Bone::LeftToeBase]);
            assert_eq!(frame.right_toe, positions[Bone::RightToeBase]);
            assert_eq!(frame.left_contact, 1.0, "the left foot is annotated planted");
            assert_eq!(frame.right_contact, 0.0);
        }
    }

    #[test]
    fn resolving_unchanged_positions_reproduces_the_clip() {
        // The identity case. Solving nothing and resolving must give back
        // what went in, or the resolve step is adding an error of its own on
        // every frame.
        let rig = RigGeometry::default();
        let clip = sliding_pose_clip(10);

        let frames = extract_frames(&clip, &rig);
        let resolved = resolve_clip(&clip, &frames, &rig, &LegIkConfig::default());

        for (after, before) in resolved.keyframes().iter().zip(clip.keyframes()) {
            let after_toe = forward_kinematics_on(&after.pose, &rig)[Bone::LeftToeBase];
            let before_toe = forward_kinematics_on(&before.pose, &rig)[Bone::LeftToeBase];

            assert!(
                after_toe.distance(before_toe) < 1.0e-3,
                "resolving unchanged positions moved a toe by {} m",
                after_toe.distance(before_toe),
            );
        }
    }

    #[test]
    fn the_resolved_clip_never_stretches_a_bone() {
        // The invariant that outranks everything else here: the correction
        // may move a foot, never lengthen a leg.
        let rig = RigGeometry::default();
        let clip = sliding_pose_clip(30);

        let (corrected, _) = remove_foot_sliding(
            &clip,
            &rig,
            &SlideConfig::default(),
            &LegIkConfig::default(),
        );

        for (i, keyframe) in corrected.keyframes().iter().enumerate() {
            let positions = forward_kinematics_on(&keyframe.pose, &rig);

            for &bone in Bone::ALL.iter() {
                let Some(parent) = bone.parent() else { continue };

                let rest = rig.offsets[bone].length();
                let solved = (positions[bone] - positions[parent]).length();

                assert!(
                    (solved - rest).abs() < 1.0e-4,
                    "frame {i}: {} is {solved} m against a rest length of {rest}",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn a_real_walk_cycle_barely_slides_at_all() {
        // The offline de-slider's first real input, and the measurement that
        // says whether the runtime needs it.
        //
        // The walk is captured the way it actually runs — the gait posed at
        // each phase, with the body advanced by the root velocity
        // `locomotion` publishes — so any slide here is the real residual
        // after the cancellation, not an artefact of a synthetic clip.
        use crate::character::anim::gait::{walk_pose_on, GaitParams};
        use crate::character::anim::locomotion::root_velocity;
        use crate::character::anim::rig::forward_kinematics_on;
        use crate::character::skeleton::Bone;

        let base = stance(&LocalPose::REST);
        // The hand-shaped walk: on this rig — real leg lengths, synthetic
        // feet — recorded angles describe no real leg, and the measured walk
        // is checked on `puppet_base` in `locomotion`'s tests.
        let params = GaitParams::authored_walk();

        // The real asset's leg lengths, parsed rather than transcribed.
        let rig = crate::character::anim::gltf_rig::real_leg_lengths();

        const FRAMES: usize = 120;
        let cadence = 1.0;
        let dt = 1.0 / FRAMES as f32;

        let mut body = Vec3::ZERO;
        let mut frames = Vec::new();

        for i in 0..FRAMES {
            let phase = i as f32 / FRAMES as f32;

            // The body is advanced to THIS frame's position BEFORE the pose
            // is recorded, so the two describe the same instant.
            //
            // Recording first and advancing after leaves the position one
            // frame behind the pose, and the tracked foot then slides by the
            // per-frame travel — measured 0.265 m over a cycle, against the
            // exact cancellation the unit test shows.
            if i > 0 {
                let previous = (i - 1) as f32 / FRAMES as f32;
                body += root_velocity(previous, cadence, &params, &base, &rig) * dt;
            }

            let k = forward_kinematics_on(&walk_pose_on(phase, &params, &base, &rig), &rig);

            let left = leg_phase_is_stance(phase, params.duty_factor);
            let right = leg_phase_is_stance(phase + 0.5, params.duty_factor);

            frames.push(SlideFrame {
                pelvis: body + k[Bone::Hips],
                left_toe: body + k[Bone::LeftToeBase],
                right_toe: body + k[Bone::RightToeBase],
                left_contact: if left { 1.0 } else { 0.0 },
                right_contact: if right { 1.0 } else { 0.0 },
            });
        }

        let config = SlideConfig::default();
        let before = total_contact_slide(&frames, &config);

        let mut solved = frames.clone();
        let report = solve_sliding(&mut solved, &config);

        // Measured: 0.312 m of summed toe travel over a cycle's 72 contact
        // frames, reduced to 0.042 m by the offline pass — an 87% cut.
        //
        // Note what this number IS and is not. It sums BOTH feet's contact
        // travel, and the root velocity can only cancel the one foot it
        // tracks; and it follows the toe, which rolls relative to the ankle
        // by design. The tracked ankle's own residual is far smaller —
        // 0.0074 m over the same cycle, pinned by
        // `locomotion::tests::the_cancellation_holds_across_a_whole_cycle_
        // including_contact_switches`.
        //
        // So this measures the slide that genuinely remains for the foot
        // lock and the offline pass to absorb, which is what they are for.

        // The runtime already cancels the bulk of it — this is the residual
        // the offline pass exists to clean up, and it should be small
        // relative to a stride.
        assert!(
            before < params.stride_length,
            "a walk built on the published root velocity slid {before} m over one \
             cycle, against a {} m stride — the cancellation is not working",
            params.stride_length,
        );

        assert!(
            report.slide_after <= before,
            "the offline pass made the sliding worse: {before} -> {}",
            report.slide_after,
        );
    }

    /// Whether one leg is in stance at `phase`.
    fn leg_phase_is_stance(phase: f32, duty: f32) -> bool {
        crate::character::anim::gait::leg_phase(phase, duty).is_stance()
    }

    #[test]
    fn the_report_measures_the_slide_rather_than_assuming_it() {
        // The report is the only evidence the solve worked, so its own
        // arithmetic is pinned: `slide_before` must match an independent
        // measurement of the input.
        let frames = sliding_clip(10, 0.01);
        let config = SlideConfig::default();

        // Nine gaps of 0.01 m, all in contact on the left only.
        let expected = 9.0 * 0.01;
        let measured = total_contact_slide(&frames, &config);

        assert!(
            (measured - expected).abs() < 1.0e-5,
            "expected {expected} m of slide, measured {measured}",
        );
    }
}
