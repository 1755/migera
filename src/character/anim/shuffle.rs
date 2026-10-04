//! Walking sideways: the side shuffle, a gait cycle of its own.
//!
//! The walk's clock and timing, the feet moving across instead of along:
//! each foot's stance sweeps it under the body toward the trailing side,
//! its swing carries it back out toward the leading side, the left foot at
//! phase 0 and the right at 0.5 ([`super::gait::leg_phase`]). So the
//! walker's whole gait pipeline serves it as it is: the clock and cadence
//! (`locomotion::distance_per_cycle`), the start and the stop
//! (`transition`), and root motion read off the planted feet
//! (`locomotion::root_displacement_between`), which knows no direction.
//!
//! # The feet never cross
//!
//! Walking, the feet pass each other front to back. Sideways they must
//! not: the gap between them swings by a stride (the body's travel a
//! cycle) about its mean, so the mean is widened to [`SHUFFLE_CLOSEST`]
//! plus half a stride, never narrower than the stance stood.
//!
//! # The legs
//!
//! Each foot is placed (`stance::move_pelvis_and_feet`), the pelvis taking
//! the height that keeps the loaded legs their length: spread wide, it
//! sinks, as a shuffling body does.

use bevy::math::{Quat, Vec3};

use super::gait::{leg_phase, stance_load, GaitParams, LegPhase};
use super::rig::{offset_from, LocalPose, RigGeometry};
use crate::character::skeleton::Bone;

/// The nearest the feet come, toe joint to toe joint across, metres.
pub const SHUFFLE_CLOSEST: f32 = 0.14;

/// How high a swinging foot lifts at mid-swing, metres: a shuffle's foot
/// skims, it does not stride over.
pub const SHUFFLE_LIFT: f32 = 0.05;

/// The share of a cycle each foot stands: both down a while between the
/// steps, as in a slow walk.
pub const SHUFFLE_DUTY: f32 = 0.65;

/// The longest a shuffle travels a cycle, metres.
pub const SHUFFLE_LONGEST: f32 = 0.55;

/// How fast a shuffle under way takes a new speed asked, m/s per second:
/// its stride and width ease to the new one rather than jump.
pub const SHUFFLE_REGEAR: f32 = 0.4;

/// A shuffle at `speed` m/s toward `toward` (+1 the rig's left, -1 its
/// right): its stride grows with speed up to [`SHUFFLE_LONGEST`], the
/// cadence carrying the rest.
pub fn shuffling(speed: f32, toward: f32) -> GaitParams {
    let stride = (speed.abs() * 0.8).clamp(0.12, SHUFFLE_LONGEST);
    GaitParams {
        duty_factor: SHUFFLE_DUTY,
        curves: super::gait::LegCurves::Shuffle { step: stride * SHUFFLE_DUTY, toward: toward.signum() },
        ..GaitParams::default()
    }
}

/// Where each foot sits across at `phase`, metres along the travel, from
/// its mean, and how high it is lifted: a stance sweeps it from `+step/2`
/// to `-step/2` at a constant rate (both feet down move alike, so neither
/// slides), a swing carries it back, eased, on an arc.
pub fn foot_at(phase: f32, duty: f32, step: f32) -> (f32, f32) {
    match leg_phase(phase, duty) {
        LegPhase::Stance { progress } => (step * (0.5 - progress), 0.0),
        LegPhase::Swing { progress } => {
            let eased = progress * progress * (3.0 - 2.0 * progress);
            (step * (eased - 0.5), SHUFFLE_LIFT * (std::f32::consts::PI * progress).sin())
        }
    }
}

/// The shuffle's pose at `phase` on `base` (a standing pose), for `step`
/// and `toward` ([`super::gait::LegCurves::Shuffle`]).
pub fn shuffle_pose(phase: f32, params: &GaitParams, step: f32, toward: f32, base: &LocalPose, rig: &RigGeometry) -> LocalPose {
    let mut pose = *base;
    let across = rig.left() * toward;
    // The stance stood, toe to toe across, and the mean the shuffle needs.
    let toe = |bone| offset_from(base, rig, Bone::Hips, bone);
    let stood = (toe(Bone::LeftToeBase) - toe(Bone::RightToeBase)).dot(rig.left()).abs();
    // The gap swings by a stride (the body's travel a cycle, `step / duty`)
    // about its mean, so the mean is the closest plus half of it. Plus a
    // whole `step` either side, as a walk's feet pass, it spread 0.69 m.
    let duty = params.duty_factor;
    let wider = ((SHUFFLE_CLOSEST + 0.5 * step / duty).max(stood) - stood) * 0.5;
    let (mut feet, mut loads) = ([Vec3::ZERO; 2], [0.0f32; 2]);
    for (leg, shift) in [(0usize, 0.0f32), (1, 0.5)] {
        let (along, lift) = foot_at(phase + shift, duty, step);
        // Out to its own side, the left foot to the rig's left.
        let side = if leg == 0 { rig.left() } else { -rig.left() };
        feet[leg] = side * wider + across * along + Vec3::Y * lift;
        // Every foot down counts for the pelvis's height
        // (`move_pelvis_and_feet` heeds a leg over 0.05 of the load): one
        // just set down carries none yet, and left out, the pelvis stood
        // too high for it to reach the floor (10 mm up).
        if let LegPhase::Stance { progress } = leg_phase(phase + shift, duty) {
            loads[leg] = stance_load(progress, duty).max(0.2);
        }
    }
    super::stance::move_pelvis_and_feet(&mut pose, rig, Vec3::ZERO, Quat::IDENTITY, loads, feet, 0.0, feet, [0.0; 2], |needed, _| needed);
    pose
}

#[cfg(test)]
mod tests {
    use super::*;

    fn real_stood() -> (LocalPose, RigGeometry) {
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        (stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig), rig)
    }

    #[test]
    fn a_shuffle_walks_aside_at_the_speed_its_cadence_is_set_for() {
        // The walker sets the cadence from the stride the pose travels
        // (`distance_per_cycle`), and root motion reads it off the planted
        // feet: walked at that cadence a cycle carries the body one stride
        // across, toward the side asked.
        use crate::character::anim::locomotion::{distance_per_cycle, root_displacement_between};
        let (stood, rig) = real_stood();
        for (speed, toward) in [(0.3, 1.0), (0.6, -1.0)] {
            let params = shuffling(speed, toward);
            let stride = distance_per_cycle(&params, &stood, &rig);
            let frames = 240;
            let mut moved = Vec3::ZERO;
            let mut before = crate::character::anim::gait::walk_pose_on(0.0, &params, &stood, &rig);
            for i in 1..=frames {
                let cycle = i as f32 / frames as f32;
                let after = crate::character::anim::gait::walk_pose_on(cycle, &params, &stood, &rig);
                moved += root_displacement_between(&before, &after, cycle - 0.5 / frames as f32, &params, &rig).unwrap();
                before = after;
            }
            let across = moved.dot(rig.left() * toward);
            assert!((across - stride).abs() < 0.01 * stride, "{speed}: a cycle moved {across:.3} m across, the stride is {stride:.3}");
            assert!(moved.dot(rig.forward()).abs() < 0.01, "{speed}: and {:.3} m along", moved.dot(rig.forward()));
        }
    }

    #[test]
    fn the_feet_never_cross_and_stand_planted_through_their_stance() {
        use crate::character::anim::foot::Sole;
        let (stood, rig) = real_stood();
        let params = shuffling(0.6, 1.0);
        let duty = params.duty_factor;
        let frames = 240;
        let toes = |pose: &LocalPose| [Bone::LeftToeBase, Bone::RightToeBase].map(|bone| offset_from(pose, &rig, Bone::Hips, bone) + pose.root_translation);
        let floor = |pose: &LocalPose, ankle| Sole::of(&rig, ankle).points(pose, &rig).iter().map(|p| p.y + pose.root_translation.y + offset_from(pose, &rig, Bone::Hips, Bone::Hips).y).fold(f32::MAX, f32::min);
        let standing = floor(&stood, Bone::LeftFoot);
        let mut closest = f32::MAX;
        let mut previous: Option<[Vec3; 2]> = None;
        for i in 0..frames {
            let cycle = i as f32 / frames as f32;
            let pose = crate::character::anim::gait::walk_pose_on(cycle, &params, &stood, &rig);
            let [left, right] = toes(&pose);
            closest = closest.min((left - right).dot(rig.left()));
            // Both down, they move alike under the body: root motion follows
            // one, and the other would slide by the difference.
            let before = cycle - 1.0 / frames as f32;
            let both_down = |at: f32| leg_phase(at, duty).is_stance() && leg_phase(at + 0.5, duty).is_stance();
            if let Some([was_left, was_right]) = previous
                && both_down(cycle)
                && both_down(before)
            {
                let apart = ((left - was_left) - (right - was_right)) * Vec3::new(1.0, 0.0, 1.0);
                assert!(apart.length() < 5.0e-4, "at {cycle:.3} the feet down moved {:.2} mm apart in a frame", apart.length() * 1e3);
            }
            previous = Some([left, right]);
            for (leg, ankle) in [(0usize, Bone::LeftFoot), (1, Bone::RightFoot)] {
                if leg_phase(cycle + 0.5 * leg as f32, duty).is_stance() {
                    let down = floor(&pose, ankle) - standing;
                    assert!(down.abs() < 2.0e-3, "at {cycle:.3} a standing foot is {:.1} mm off the floor", down * 1e3);
                }
            }
        }
        assert!(closest > SHUFFLE_CLOSEST - 0.005, "the feet came {closest:.3} m apart");
    }
}
