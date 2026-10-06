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
pub const SHUFFLE_CLOSEST: f32 = 0.12;

/// How high a swinging foot lifts at mid-swing, metres: a shuffle's foot
/// skims, it does not stride over.
pub const SHUFFLE_LIFT: f32 = 0.05;

/// How far through its swing a foot has finished moving across, the rest
/// a set-down.
pub const SHUFFLE_ACROSS_BY: f32 = 0.75;

/// The share of a cycle each foot stands: both down a while between the
/// steps, as in a slow walk.
pub const SHUFFLE_DUTY: f32 = 0.65;

/// The longest a shuffle travels a cycle, metres.
pub const SHUFFLE_LONGEST: f32 = 0.45;

/// A shuffle's stride per m/s of speed, metres: short and quick, about
/// 1.8 cycles a second. At 0.8 a stride the stance widened 7.5 cm a side
/// at 0.6 m/s, all within the first swing; the sprung leg standing through
/// it lagged, and its lock let it go 1.7 cm out at lift-off.
pub const SHUFFLE_STRIDE_PER_SPEED: f32 = 0.55;

/// How fast a shuffle under way takes a new speed asked, m/s per second:
/// its stride and width ease to the new one rather than jump.
pub const SHUFFLE_REGEAR: f32 = 0.4;

/// How fast a shuffle under way takes a new diagonal, of its way forward
/// per second.
pub const SHUFFLE_REAIM: f32 = 0.5;

/// A shuffle at `speed` m/s toward `toward` (+1 the rig's left, -1 its
/// right), `ahead` of its way forward (0 straight across, at most
/// [`SHUFFLE_MOST_AHEAD`]; negative, back): its stride grows with speed up
/// to [`SHUFFLE_LONGEST`], the cadence carrying the rest.
pub fn shuffling(speed: f32, toward: f32, ahead: f32) -> GaitParams {
    let stride = (speed.abs() * SHUFFLE_STRIDE_PER_SPEED).clamp(0.1, SHUFFLE_LONGEST);
    GaitParams {
        duty_factor: SHUFFLE_DUTY,
        curves: super::gait::LegCurves::Shuffle {
            step: stride * SHUFFLE_DUTY,
            toward: toward.signum(),
            ahead: ahead.clamp(-SHUFFLE_MOST_AHEAD, SHUFFLE_MOST_AHEAD),
        },
        ..GaitParams::default()
    }
}

/// The most of a shuffle's way that is forward or back, a diagonal at 45°:
/// beyond it the body walks, turned to its way (`walker`).
pub const SHUFFLE_MOST_AHEAD: f32 = std::f32::consts::FRAC_1_SQRT_2;

/// Which feet a shuffle has down (left, right), for the foot locks
/// (`plugin::AnimFootIk::planted`): at `cycle`, the gait in at `weight`.
/// While the gait fades in or out, never the foot the fade swings
/// (`swinging_left`: a start's first, a stop's last): its clock counts it
/// down while it is still being set down (9 mm up, 1 cm short), and locked
/// there it was dropped short.
pub fn planted(cycle: f32, duty: f32, weight: f32, swinging_left: bool) -> [bool; 2] {
    [(0usize, 0.0f32), (1, 0.5)].map(|(leg, shift)| leg_phase(cycle + shift, duty).is_stance() && (weight >= 1.0 || (leg == 0) != swinging_left))
}

/// Where each foot sits across at `phase`, metres along the travel, from
/// its mean, and how high it is lifted: a stance sweeps it from `+step/2`
/// to `-step/2` at a constant rate (both feet down move alike, so neither
/// slides), a swing carries it back, eased, on an arc.
pub fn foot_at(phase: f32, duty: f32, step: f32) -> (f32, f32) {
    match leg_phase(phase, duty) {
        LegPhase::Stance { progress } => (step * (0.5 - progress), 0.0),
        LegPhase::Swing { progress } => {
            // Across by three quarters of the swing, then straight down: on
            // the arc's own schedule the sprung foot met the floor still
            // going, and slid its last 6-15 mm every step.
            let across = (progress / SHUFFLE_ACROSS_BY).min(1.0);
            let eased = across * across * (3.0 - 2.0 * across);
            (step * (eased - 0.5), SHUFFLE_LIFT * (std::f32::consts::PI * progress).sin())
        }
    }
}

/// The shuffle's pose at `phase` on `base` (a standing pose), for `step`,
/// `toward` and `ahead` ([`super::gait::LegCurves::Shuffle`]).
pub fn shuffle_pose(phase: f32, params: &GaitParams, step: f32, toward: f32, ahead: f32, base: &LocalPose, rig: &RigGeometry) -> LocalPose {
    let mut pose = *base;
    // The way it goes: across, and forward or back on a diagonal.
    let sideways = (1.0 - ahead * ahead).max(0.0).sqrt();
    let across = rig.left() * toward * sideways + rig.forward() * ahead;
    // The stance stood, toe to toe across, and the mean the shuffle needs.
    let toe = |bone| offset_from(base, rig, Bone::Hips, bone);
    let stood = (toe(Bone::LeftToeBase) - toe(Bone::RightToeBase)).dot(rig.left()).abs();
    // The gap swings by a stride (the body's travel a cycle, `step / duty`)
    // about its mean, so the mean is the closest plus half of it. Plus a
    // whole `step` either side, as a walk's feet pass, it spread 0.69 m.
    // On a diagonal only the stride's part across: front to back the feet
    // pass each other, apart across, as a walk's do.
    let duty = params.duty_factor;
    let wider = ((SHUFFLE_CLOSEST + 0.5 * sideways * step / duty).max(stood) - stood) * 0.5;
    // Each foot's move from where it stood at `at`, and its load.
    let feet_at = |at: f32| {
        let (mut feet, mut loads) = ([Vec3::ZERO; 2], [0.0f32; 2]);
        for (leg, shift) in [(0usize, 0.0f32), (1, 0.5)] {
            let (along, lift) = foot_at(at + shift, duty, step);
            // Out to its own side, the left foot to the rig's left.
            let side = if leg == 0 { rig.left() } else { -rig.left() };
            feet[leg] = side * wider + across * along + Vec3::Y * lift;
            // Every foot down counts for the pelvis's height
            // (`move_pelvis_and_feet` heeds a leg over 0.05 of the load): one
            // just set down carries none yet, and left out, the pelvis stood
            // too high for it to reach the floor (10 mm up).
            if let LegPhase::Stance { progress } = leg_phase(at + shift, duty) {
                loads[leg] = stance_load(progress, duty).max(0.2);
            }
        }
        (feet, loads)
    };
    let (feet, loads) = feet_at(phase);
    let drop = smoothed_drop(phase, duty, base, rig, &feet_at);
    super::stance::move_pelvis_and_feet(&mut pose, rig, Vec3::ZERO, Quat::IDENTITY, loads, feet, 0.0, feet, [0.0; 2], |_, ceiling| drop.min(ceiling));
    carry_arms(&mut pose, phase, duty, rig);
    pose
}

/// Samples a cycle the pelvis's path is fitted over ([`smoothed_drop`]).
const DROP_SAMPLES: usize = 48;

/// The pelvis's drop at `phase` (negative down, from standing on `base`):
/// one sinusoid a step, fitted at or under the height the feet down allow
/// over the cycle (`feet_at` gives each foot's move and load), as the
/// walk's pelvis is (`walk::BOB_HARMONICS`).
///
/// Each leg down allows the pelvis as high as it reaches its foot,
/// `stance::move_pelvis_and_feet`'s own need, and the pelvis took the
/// lowest. A foot set down out wide counted at once, and the pelvis stepped
/// down for it in a frame: 230-1260 m/s² headless, 8-10 live through the
/// springs, a dip every step. Under the fit the legs bend a little more,
/// and the feet stay where they are placed.
fn smoothed_drop(phase: f32, duty: f32, base: &LocalPose, rig: &RigGeometry, feet_at: &dyn Fn(f32) -> ([Vec3; 2], [f32; 2])) -> f32 {
    use std::f32::consts::TAU;
    // Each leg, socket to ankle, as it stood.
    let legs = [(Bone::LeftUpLeg, Bone::LeftFoot), (Bone::RightUpLeg, Bone::RightFoot)]
        .map(|(socket, ankle)| offset_from(base, rig, Bone::Hips, ankle) - offset_from(base, rig, Bone::Hips, socket));
    // How high the pelvis may sit at `at` for each loaded leg to reach its
    // foot.
    let allowed_at = |at: f32| {
        let (feet, loads) = feet_at(at);
        let total = (loads[0] + loads[1]).max(1.0e-6);
        (0..2)
            .filter(|&leg| loads[leg] / total > 0.05)
            .map(|leg| {
                let to = legs[leg] + feet[leg];
                to.y + (legs[leg].length_squared() - (to.x * to.x + to.z * to.z)).max(0.0).sqrt()
            })
            .fold(0.0f32, f32::min)
    };
    let allowed: Vec<f32> = (0..DROP_SAMPLES).map(|i| allowed_at(i as f32 / DROP_SAMPLES as f32)).collect();
    // And each foot's last instant down, the trailing foot furthest under:
    // between the samples, the path was clipped to it there and kinked
    // (118 m/s² headless at 0.6 m/s).
    let lifts = [duty, duty + 0.5].map(|lift| {
        let at = (lift - 1.0e-4).rem_euclid(1.0);
        (at, allowed_at(at))
    });
    // Mean and the step's harmonic (twice a cycle), pushed under wherever
    // the fit rides above what is allowed.
    let n = DROP_SAMPLES as f32;
    let fit = |target: &[f32]| {
        let mut c = [target.iter().sum::<f32>() / n, 0.0, 0.0];
        for (i, &h) in target.iter().enumerate() {
            let angle = TAU * 2.0 * i as f32 / n;
            c[1] += 2.0 / n * h * angle.cos();
            c[2] += 2.0 / n * h * angle.sin();
        }
        c
    };
    let at = |c: &[f32; 3], p: f32| c[0] + c[1] * (TAU * 2.0 * p).cos() + c[2] * (TAU * 2.0 * p).sin();
    let mut target = allowed.clone();
    let mut c = fit(&target);
    for _ in 0..40 {
        let mut above = 0.0f32;
        for (i, slot) in target.iter_mut().enumerate() {
            let over = at(&c, i as f32 / n) - allowed[i];
            if over > 0.0 {
                *slot -= over;
                above = above.max(over);
            }
        }
        if above < 1.0e-5 {
            break;
        }
        c = fit(&target);
    }
    // Whatever the passes left above, the whole path lowered by: clipped
    // to the legs' reach between, it stepped again.
    let above = (0..DROP_SAMPLES)
        .map(|i| at(&c, i as f32 / n) - allowed[i])
        .chain(lifts.iter().map(|&(p, allowed)| at(&c, p) - allowed))
        .fold(0.0f32, f32::max);
    at(&c, phase.rem_euclid(1.0)) - above
}

/// How far a shuffle carries each upper arm out from the body, radians.
/// Authored, not measured: no recording of the side shuffle's arms was to
/// hand. A walk's arms swing against the legs to cancel the body's twist
/// about the vertical; across, the legs swing in the frontal plane, and the
/// arms are held a little out of it.
pub const SHUFFLE_ARM_OUT: f32 = 0.2;

/// How far each arm sways further out as the opposite leg swings out,
/// radians: the counter-swing, small.
pub const SHUFFLE_ARM_SWAY: f32 = 0.05;

/// How much further a shuffle bends the elbows than standing, radians.
pub const SHUFFLE_ELBOW: f32 = 0.35;

/// Carries the arms a little out, the elbows a little bent, each arm
/// swaying out as the other side's leg swings: on the rig in hand, each
/// axis derived from the arm as posed (as the walk's arm swing is), so the
/// sign holds on any rig.
fn carry_arms(pose: &mut LocalPose, phase: f32, duty: f32, rig: &RigGeometry) {
    use super::rig::delta_after_world_turn;
    let arms = [(Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand, rig.left(), 0.5f32), (Bone::RightArm, Bone::RightForeArm, Bone::RightHand, -rig.left(), 0.0)];
    for (shoulder, elbow, hand, outward, opposite) in arms {
        // Out as the opposite leg swings, by how far through its swing.
        let sway = match leg_phase(phase + opposite, duty) {
            LegPhase::Swing { progress } => (std::f32::consts::PI * progress).sin(),
            LegPhase::Stance { .. } => 0.0,
        };
        let along = offset_from(pose, rig, shoulder, hand).normalize_or_zero();
        // Turning `along` toward `outward`: a positive angle about their
        // cross carries the hand out.
        let out = along.cross(outward).normalize_or_zero();
        if out.length_squared() > 0.25 {
            pose.rotations[shoulder] = delta_after_world_turn(pose, rig, shoulder, Quat::from_axis_angle(out, SHUFFLE_ARM_OUT + SHUFFLE_ARM_SWAY * sway));
        }
        let forearm = offset_from(pose, rig, elbow, hand).normalize_or_zero();
        let fold = forearm.cross(rig.forward()).normalize_or_zero();
        if fold.length_squared() > 0.25 {
            pose.rotations[elbow] = delta_after_world_turn(pose, rig, elbow, Quat::from_axis_angle(fold, SHUFFLE_ELBOW));
        }
    }
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
        // across, toward the side asked; on a diagonal, along it.
        use crate::character::anim::locomotion::{distance_per_cycle, root_displacement_between};
        let (stood, rig) = real_stood();
        for (speed, toward, ahead) in [(0.3, 1.0, 0.0), (0.6, -1.0, 0.0), (0.5, 1.0, 0.6), (0.5, -1.0, -0.5)] {
            let params = shuffling(speed, toward, ahead);
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
            let way = rig.left() * toward * (1.0f32 - ahead * ahead).sqrt() + rig.forward() * ahead;
            let along = moved.dot(way);
            assert!((along - stride).abs() < 0.01 * stride, "{speed}, {ahead}: a cycle moved {along:.3} m its way, the stride is {stride:.3}");
            assert!((moved - way * along).length() < 0.01, "{speed}, {ahead}: and {:.3} m off it", (moved - way * along).length());
        }
    }

    /// A shuffle started from a stand as the walker does it, headless: the
    /// transition, the pose, root motion off the rendered contacts. Per
    /// frame, each ball in the world (the body carried by root motion), the
    /// gait's weight, and which feet the gait has down.
    fn start(speed: f32, toward: f32, frames: usize) -> Vec<([Vec3; 2], f32, [bool; 2])> {
        use crate::character::anim::gait::walk_pose_on;
        use crate::character::anim::locomotion::{distance_per_cycle, root_displacement_between};
        use crate::character::anim::rig::forward_kinematics_on;
        use crate::character::anim::transition::{Transition, TransitionConfig, TransitionEvent};
        const DT: f32 = 1.0 / 60.0;
        let (stood, rig) = real_stood();
        let params = shuffling(speed, toward, 0.0);
        let cadence = speed / distance_per_cycle(&params, &stood, &rig);
        let config = TransitionConfig { mid_swing: params.duty_factor * 0.5, ..Default::default() };
        let mut transition = Transition::standing();
        let (mut cycle, mut body) = (0.0f32, Vec3::ZERO);
        let (mut previous, mut previous_cycle) = (stood, 0.0f32);
        let mut out = Vec::new();
        for _ in 0..frames {
            if let Some(TransitionEvent::FirstStep { cycle: from }) = transition.advance(speed, cycle, &config, DT) {
                cycle = from;
            }
            let weight = transition.weight;
            let mut prepared = stood;
            transition.apply_release(&mut prepared, &rig);
            let pose = if weight <= 0.0 { prepared } else { transition.blend(&prepared, &walk_pose_on(cycle, &params, &stood, &rig), &rig) };
            if weight > 0.0 {
                let middle = previous_cycle + 0.5 * (cycle - previous_cycle).rem_euclid(1.0);
                body += root_displacement_between(&previous, &pose, middle, &params, &rig).unwrap_or(Vec3::ZERO);
            }
            let joints = forward_kinematics_on(&pose, &rig);
            // As the walker tells the foot locks.
            let down = planted(cycle, params.duty_factor, weight, transition.stance < 0.0);
            out.push(([joints[Bone::LeftToeBase] + body, joints[Bone::RightToeBase] + body], weight, down));
            (previous, previous_cycle) = (pose, cycle);
            cycle = (cycle + cadence * DT * f32::from(transition.release >= 1.0 || weight > 0.0)).rem_euclid(1.0);
        }
        out
    }

    #[test]
    fn a_shuffle_carries_the_hands_out_and_forward_both_sides_alike() {
        // Hanging as they stood, the arms read as a body moved by its legs
        // alone. Carried: each hand further out to its own side and a
        // little forward of standing, the two alike.
        let (stood, rig) = real_stood();
        let hands = |pose: &LocalPose| [Bone::LeftHand, Bone::RightHand].map(|bone| offset_from(pose, &rig, Bone::Hips, bone));
        let [stood_left, stood_right] = hands(&stood);
        // At a moment no foot swings: no sway.
        let params = shuffling(0.4, 1.0, 0.0);
        let at = (0..100).map(|i| i as f32 / 100.0).find(|&c| leg_phase(c, params.duty_factor).is_stance() && leg_phase(c + 0.5, params.duty_factor).is_stance()).unwrap();
        let pose = crate::character::anim::gait::walk_pose_on(at, &params, &stood, &rig);
        let [left, right] = hands(&pose);
        let out = [(left - stood_left).dot(rig.left()), (right - stood_right).dot(-rig.left())];
        let ahead = [(left - stood_left).dot(rig.forward()), (right - stood_right).dot(rig.forward())];
        for side in 0..2 {
            assert!(out[side] > 0.04, "hand {side} {:.3} m out", out[side]);
            assert!(ahead[side] > 0.02, "hand {side} {:.3} m forward", ahead[side]);
        }
        assert!((out[0] - out[1]).abs() < 0.02 && (ahead[0] - ahead[1]).abs() < 0.02, "out {out:?}, forward {ahead:?}");
    }

    #[test]
    fn a_shuffle_starts_with_the_feet_down_staying_where_they_are() {
        // Started from a stand through the transition (its release, the
        // first swing fading the gait in), each way: a foot the gait has
        // down stays where it was set in the world, within 8 mm over its
        // whole stance (measured, the toe joint as the first fade passes:
        // 5.1 mm starting left, 3 mm right; live, nothing over 2 mm on the
        // floor).
        for toward in [1.0, -1.0] {
            let frames = start(0.6, toward, 150);
            for leg in 0..2 {
                let mut set: Option<Vec3> = None;
                for (frame, (balls, weight, down)) in frames.iter().enumerate() {
                    if *weight <= 0.0 || !down[leg] {
                        set = None;
                        continue;
                    }
                    let at = *set.get_or_insert(balls[leg]);
                    let moved = Vec3::new(balls[leg].x - at.x, 0.0, balls[leg].z - at.z).length();
                    assert!(moved < 0.008, "toward {toward}: foot {leg} down moved {:.1} mm at frame {frame}", moved * 1e3);
                }
            }
        }
    }

    /// The pelvis's height over a steady shuffle, at the cadence the walker
    /// sets: its sharpest acceleration up or down, m/s².
    fn hardest_pelvis(speed: f32, toward: f32, ahead: f32, base: &LocalPose, rig: &RigGeometry) -> f32 {
        use crate::character::anim::locomotion::distance_per_cycle;
        let params = shuffling(speed, toward, ahead);
        let seconds = distance_per_cycle(&params, base, rig) / speed;
        let n = 240;
        let dt = seconds / n as f32;
        let heights: Vec<f32> = (0..n + 2).map(|i| crate::character::anim::gait::walk_pose_on(i as f32 / n as f32, &params, base, rig).root_translation.y).collect();
        heights.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (dt * dt)).abs()).fold(0.0, f32::max)
    }

    /// The pelvis rides one smooth path a step ([`smoothed_drop`]): through
    /// a steady shuffle it accelerates no harder than 5 m/s² (measured at
    /// most 3.2, at 0.6 m/s). Taking the lowest any foot down allowed, it
    /// stepped down as each foot set down out wide: 230-1260 m/s² over
    /// 1/240 of a cycle, 8-10 live through the springs.
    #[test]
    fn a_shuffles_pelvis_rides_a_smooth_path() {
        let (stood, rig) = real_stood();
        for (speed, toward, ahead) in [(0.3, 1.0, 0.0), (0.4, 1.0, 0.0), (0.6, -1.0, 0.0), (0.5, 1.0, 0.6), (0.5, -1.0, -0.6)] {
            let hardest = hardest_pelvis(speed, toward, ahead, &stood, &rig);
            assert!(hardest < 5.0, "{speed} m/s toward {toward} ahead {ahead}: the pelvis accelerates at {hardest} m/s²");
        }
    }

    /// The locomotion layer composed on a shuffle as the walker does, its
    /// walk sway faded out (`walker::fade_walk_sway`, passed 1 shuffling):
    /// the pelvis still rides smooth. Not faded, the walk's sway re-solved
    /// the pelvis over the feet a walk's stance timing loads, wide apart,
    /// and stepped it down at every step: 403 m/s² headless, a 7 mm dip
    /// live.
    #[test]
    fn under_the_locomotion_layer_a_shuffles_pelvis_rides_smooth_with_the_walk_sway_faded() {
        use crate::character::anim::locomotion::distance_per_cycle;
        use crate::character::anim::phase::{GaitPhase, PhaseLayer};
        let (stood, rig) = real_stood();
        let speed = 0.4;
        let params = shuffling(speed, 1.0, 0.0);
        let seconds = distance_per_cycle(&params, &stood, &rig) / speed;
        let hardest = |faded: bool| {
            let mut layer = PhaseLayer::locomotion();
            if faded {
                crate::character::anim::walker::fade_walk_sway(&mut layer, 1.0);
            }
            let n = 240;
            let dt = seconds / n as f32;
            let heights: Vec<f32> = (0..n + 2)
                .map(|i| {
                    let cycle = i as f32 / n as f32;
                    let mut pose = crate::character::anim::gait::walk_pose_on(cycle, &params, &stood, &rig);
                    let phase = GaitPhase { gait: cycle * std::f32::consts::TAU, speed, ..Default::default() };
                    layer.apply_on(&phase, &mut pose, &rig);
                    pose.root_translation.y
                })
                .collect();
            heights.windows(3).map(|w| ((w[2] - 2.0 * w[1] + w[0]) / (dt * dt)).abs()).fold(0.0, f32::max)
        };
        let (faded, not) = (hardest(true), hardest(false));
        assert!(faded < 5.0, "faded, the pelvis accelerates at {faded} m/s²");
        assert!(not > 50.0, "the walk's sway on a shuffle steps the pelvis only {not} m/s²");
    }

    #[test]
    fn the_feet_never_cross_and_stand_planted_through_their_stance() {
        // Straight across, and on diagonals forward and back.
        for ahead in [0.0, 0.6, -0.6] {
            feet_never_cross_and_stand_planted(ahead);
        }
    }

    fn feet_never_cross_and_stand_planted(ahead: f32) {
        use crate::character::anim::foot::Sole;
        let (stood, rig) = real_stood();
        let params = shuffling(0.6, 1.0, ahead);
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
                assert!(apart.length() < 5.0e-4, "{ahead}: at {cycle:.3} the feet down moved {:.2} mm apart in a frame", apart.length() * 1e3);
            }
            previous = Some([left, right]);
            for (leg, ankle) in [(0usize, Bone::LeftFoot), (1, Bone::RightFoot)] {
                if leg_phase(cycle + 0.5 * leg as f32, duty).is_stance() {
                    let down = floor(&pose, ankle) - standing;
                    assert!(down.abs() < 2.0e-3, "{ahead}: at {cycle:.3} a standing foot is {:.1} mm off the floor", down * 1e3);
                }
            }
        }
        assert!(closest > SHUFFLE_CLOSEST - 0.005, "{ahead}: the feet came {closest:.3} m apart");
    }
}
