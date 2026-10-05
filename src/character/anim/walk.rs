//! The measured walk, made to stand on a specific rig.
//!
//! [`super::gait`] drives a walk's legs with Winter's recorded stride (see
//! [`super::reference`]). Played back on another body the stride is not
//! quite self-consistent, because Winter's subject is proportioned
//! differently: his marker thigh is short for his shank (31.4 to 42.5 cm,
//! a ratio of 0.74) where `puppet_base`, like the book's own Figure 4.1
//! proportions, has the two nearly equal (0.93). The same angles on
//! different segments put the feet in slightly different places.
//!
//! # The legs lead; the pelvis follows
//!
//! The legs take the recorded angles, and the pelvis rides wherever the
//! planted legs hold it — the soft maximum of what each planted foot needs
//! (`foot::support_height`), so no foot passes through the floor. The
//! alternative was measured and rejected: imposing the recording's pelvis
//! path (`reference::Stride::pelvis_bob`, 36 mm, lowest in double support)
//! and bending each planted leg to meet the floor from it. No size of that
//! bob kept the legs near the recording — at 0.25-1.0x Winter's, the thigh
//! strayed 6.7-9.9 degrees and the stance knee 8.7-12.7 — and where the bob
//! asked for more leg than there is, planted feet floated 16 mm and slid
//! 55 mm through the springs. The joint angles are what read as a human
//! walk; on this rig they buy a 14 mm bob, lowest late in single support.
//!
//! # The thigh correction
//!
//! Through double support both feet are down and should stay put, which
//! fixes how the distance between them may change, and on a
//! differently-proportioned leg the recording does not honour it. So each
//! leg's thigh gets a small correction, found once per rig — the amount its
//! contact drifts while it carries the body (by the same support weights the
//! pelvis and root motion use, `foot::bearing`), integrated from footfall,
//! fed back as a rotation holding the contact where it landed (the foot
//! keeps the recording's attitude; see [`WalkCycle::pose_legs`]). Through
//! swing the correction returns to zero, so every stance starts from the
//! recorded stride again. It is 1.4 degrees at most, and halves the planted
//! foot's slide, 10.2 to 5.9 mm a stance, all of it in the weight
//! hand-overs where each foot carries only part of the body.
//!
//! The correction depends only on the gait, the base pose and the rig, so it
//! is memoized: [`walk_cycle`] builds it on first use and hands back the same
//! cycle while those inputs are unchanged.

use std::hash::{Hash, Hasher};
use std::sync::{Arc, Mutex};

use bevy::math::Quat;

use super::foot::{bearing, contact_moved, lowest, support_height, Contacts, Sole};
use super::gait::{self, GaitParams, LegPhase};
use super::reference::{Periodic, WINTER};
use super::rig::{LocalPose, RigGeometry};
use super::stance::KNEE_AXIS;
use crate::character::skeleton::Bone;

/// Samples per cycle in the correction table.
const SAMPLES: usize = 256;

/// Refinement passes over the correction: a corrected leg changes the body
/// motion it was corrected against. Measured on `puppet_base` at the
/// reference speed, total slide per stance of the foot carrying the body:
/// no correction 10.2 mm (worst 1.53 mm in 1/240 of a stride), three passes
/// 5.9 mm (0.61) — the rest is in the weight hand-overs, where each foot
/// carries only part of the body.
const PASSES: usize = 3;

/// Each leg: its phase offset in the cycle, and its hip, knee and ankle.
const LEGS: [(f32, Bone, Bone, Bone); 2] = [
    (0.0, Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
    (0.5, Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
];

/// The measured walk on one rig, from one base pose.
#[derive(Debug, Clone, PartialEq)]
pub struct WalkCycle {
    amplitude: f32,
    duty_factor: f32,
    /// Each leg's hip and foot rotations with the feet brought in to the
    /// walk's step width (`stance::narrow_feet`). The recorded stride is
    /// composed onto these, not onto the base's standing width.
    narrowed: [[Quat; 2]; 2],
    /// The narrowed base pose's own `gait::sagittal_angles`, per leg.
    base_angles: [[f32; 4]; 2],
    /// The bind's shank angle from vertical, per leg: the foot's bind-zeroed
    /// attitude is measured against it.
    bind_shank: [f32; 2],
    /// Thigh correction per leg, radians, sampled over the cycle.
    correction: [[f32; SAMPLES]; 2],
    soles: [Sole; 2],
    /// Each foot's lowest contact under the base pose, hips-relative: the
    /// ground, when standing.
    ground: [f32; 2],
    /// The pelvis height the walk rides at, smoothed: the mean, then a
    /// cosine and a sine per entry of [`BOB_HARMONICS`] (stride harmonics).
    /// `None` while the cycle is still being built. See [`WalkCycle::pose`].
    bob: Option<[f32; BOB_COEFFICIENTS]>,
}

/// Which harmonics of the stride the pelvis path keeps: once per step.
///
/// The height the planted legs ask for, per sample, is lumpy on this rig:
/// measured on `puppet_base` at 1.2 m/s it falls 14 mm through late single
/// support, is caught at the next heel contact and bumps again in the
/// weight hand-over — peaking near 7.7 m/s², the "body drops onto each leg"
/// a viewer sees. One sinusoid per step, fitted under the raw path through
/// single support (see `smoothed_bob`), is the textbook centre-of-mass path
/// (Winter): highest in midstance (0.22 of the stride), lowest just before
/// the heel contact (0.47). Peak acceleration 9.5 → 0.98 m per cycle².
/// Measured against keeping the next harmonic too: 2.6, its lowest point
/// staying late in single support. The planted feet then press up to ~10 mm
/// into the floor in this pose, and the foot IK lifts them by bending the
/// stance knee a little more, toward Winter's own midstance knee.
const BOB_HARMONICS: [usize; 1] = [2];
const BOB_COEFFICIENTS: usize = 1 + 2 * BOB_HARMONICS.len();

impl WalkCycle {
    fn build(params: &GaitParams, amplitude: f32, base: &LocalPose, rig: &RigGeometry) -> Self {
        // A walk puts its feet closer together than a stance does (Winter
        // §11.3.1; see `stance::STEP_WIDTH`).
        let mut narrow = *base;
        super::stance::narrow_feet(&mut narrow, rig, super::stance::step_width(base, rig));
        let narrowed = LEGS.map(|(_, hip, _, ankle)| [narrow.rotations[hip], narrow.rotations[ankle]]);
        let base_angles =
            [0, 1].map(|leg| gait::sagittal_angles(&narrow, rig, gait::leg_joints(LEGS[leg].3)));
        let bind_shank = [0, 1].map(|leg| {
            let [thigh, _, knee, _] =
                gait::sagittal_angles(&LocalPose::REST, rig, gait::leg_joints(LEGS[leg].3));
            thigh - knee
        });
        let soles = [Sole::of(rig, LEGS[0].3), Sole::of(rig, LEGS[1].3)];
        let ground = [0, 1].map(|leg| lowest(&soles[leg].points(base, rig)));
        let mut cycle = Self {
            amplitude,
            duty_factor: params.duty_factor,
            narrowed,
            base_angles,
            bind_shank,
            correction: [[0.0; SAMPLES]; 2],
            soles,
            ground,
            bob: None,
        };
        for _ in 0..PASSES {
            cycle.refine(base, rig);
        }
        cycle.bob = Some(cycle.smoothed_bob(base, rig));
        cycle
    }

    /// Fits [`BOB_HARMONICS`] to the pelvis height the planted legs ask for
    /// over one cycle.
    fn smoothed_bob(&self, base: &LocalPose, rig: &RigGeometry) -> [f32; BOB_COEFFICIENTS] {
        let facing = super::stance::facing_sign(rig);
        let heights: Vec<f32> = (0..SAMPLES)
            .map(|i| {
                let pose = self.pose(base, rig, i as f32 / SAMPLES as f32, facing);
                pose.root_translation.y - base.root_translation.y
            })
            .collect();
        // Where one foot carries the body alone: there a float cannot be
        // made up by the other foot.
        let single: Vec<bool> = (0..SAMPLES)
            .map(|i| {
                let phase = i as f32 / SAMPLES as f32;
                LEGS.iter()
                    .filter(|leg| gait::leg_phase(phase + leg.0, self.duty_factor).is_stance())
                    .count()
                    == 1
            })
            .collect();
        let n = SAMPLES as f32;
        let fit = |target: &[f32]| {
            let mut bob = [0.0; BOB_COEFFICIENTS];
            bob[0] = target.iter().sum::<f32>() / n;
            for (slot, &k) in BOB_HARMONICS.iter().enumerate() {
                for (i, &h) in target.iter().enumerate() {
                    let angle = std::f32::consts::TAU * k as f32 * i as f32 / n;
                    bob[1 + 2 * slot] += 2.0 / n * h * angle.cos();
                    bob[2 + 2 * slot] += 2.0 / n * h * angle.sin();
                }
            }
            bob
        };
        // Kept at or below the raw height through single support: above it
        // the planted foot floats, and the rig has no leg to spare to reach
        // down for it (it stands at critical extension), so the foot IK
        // would drop the pelvis — the lump again. Below it the foot presses
        // into the floor and the IK only bends a knee. A plain fit floated
        // 3.3 mm at 1.2 m/s; each pass pushes the target down wherever the
        // fit still rides above. Not through double support, where the
        // other foot is down too: held there as well, the fit chased the
        // weight hand-over's narrow dips 31 mm down on the synthetic rig.
        let mut target = heights.clone();
        let mut bob = fit(&target);
        for _ in 0..40 {
            let mut above = 0.0f32;
            for (i, slot) in target.iter_mut().enumerate() {
                let over = Self::bob_at(&bob, i as f32 / n) - heights[i];
                if single[i] && over > 0.0 {
                    *slot -= over;
                    above = above.max(over);
                }
            }
            if above < 1.0e-4 {
                break;
            }
            bob = fit(&target);
        }
        bob
    }

    /// The smoothed pelvis height at `phase`.
    ///
    /// Folded to half a stride first: every harmonic is per STEP, so the
    /// value repeats there, and folding makes the two legs, half a cycle
    /// apart, read the same float rather than two roundings of it.
    fn bob_at(bob: &[f32; BOB_COEFFICIENTS], phase: f32) -> f32 {
        let phase = phase.rem_euclid(0.5);
        let mut height = bob[0];
        for (slot, &k) in BOB_HARMONICS.iter().enumerate() {
            let angle = std::f32::consts::TAU * k as f32 * phase;
            height += bob[1 + 2 * slot] * angle.cos() + bob[2 + 2 * slot] * angle.sin();
        }
        height
    }

    /// The walking pose's legs and pelvis at `phase`, composed onto `base`.
    pub fn pose(&self, base: &LocalPose, rig: &RigGeometry, phase: f32, facing: f32) -> LocalPose {
        let mut pose = *base;
        // `ground` stays the standing base's: a foot brought in rises a
        // little (the leg turns whole), and the pelvis comes down for it.
        for (leg, &(_, hip, _, ankle)) in LEGS.iter().enumerate() {
            pose.rotations[hip] = self.narrowed[leg][0];
            pose.rotations[ankle] = self.narrowed[leg][1];
        }
        self.pose_legs(&mut pose, phase, facing);

        let (mut loads, mut needs) = ([0.0; 2], [0.0; 2]);
        for leg in 0..2 {
            if let LegPhase::Stance { progress } = gait::leg_phase(phase + LEGS[leg].0, self.duty_factor) {
                loads[leg] = gait::stance_load(progress, self.duty_factor);
                // How much further below the hips the foot's weight-bearing
                // contact is than when standing: the pelvis rises by that to
                // put it on the ground. The contact, not the ankle — a foot
                // rolling heel to toe keeps whichever end bears weight on the
                // floor, and the ankle climbs 12 cm by toe-off.
                needs[leg] = self.ground[leg] - lowest(&self.soles[leg].points(&pose, rig));
            }
        }
        let Some(asked) = support_height(loads, needs) else {
            return pose;
        };
        let Some(bob) = &self.bob else {
            // Still building: the raw height is what the fit is made from.
            pose.root_translation.y += asked;
            return pose;
        };
        // The smoothed height, not the one asked for: at or below it, so a
        // planted foot may press a little into the floor, never float.
        // Re-solving the legs to keep the feet exactly on the floor was
        // tried and refused: the leg IK re-planes the leg about its fixed
        // hinge, which moved the thigh 2.8 degrees off Winter's, broke the
        // left/right mirror and jumped the root velocity 0.11 m/s.
        let _ = asked;
        pose.root_translation.y += Self::bob_at(bob, phase);
        pose
    }

    /// Composes both legs onto `pose` at `phase`: the recorded thigh and
    /// knee, the foot at its recorded attitude on the ground.
    pub fn pose_legs(&self, pose: &mut LocalPose, phase: f32, facing: f32) {
        let reference = &*WINTER;
        let gentle = self.amplitude.sqrt();
        for (leg, &(shift, hip, knee, ankle)) in LEGS.iter().enumerate() {
            let at = gait::reference_phase(phase + shift, self.duty_factor);
            let about_mean = |c: &Periodic, k: f32| c.mean() + k * (c.at(at) - c.mean());
            let thigh_angle = about_mean(&reference.thigh, self.amplitude) + self.correction_at(leg, phase);
            let knee_angle =
                gait::soft_floor(gentle * reference.knee.at(at), gait::KNEE_FLOOR, gait::KNEE_FLOOR_SOFTNESS);
            // The FOOT is placed by its absolute attitude, like the thigh, and
            // the ankle is whatever joins the shank to it: in the sagittal
            // plane the foot's toe-up angle is `thigh - knee + ankle`.
            //
            // Not the book's ankle angle, because that is measured against
            // the fibula-head line, which is not the knee-to-ankle line a rig
            // bends about: replayed on joint-centre bones it tipped the
            // planted foot 5.6 degrees onto its ball through mid-stance and
            // struck the ground at 12 degrees toe-up instead of 25 — a foot
            // that rolls off its toes early is a leg that is short in late
            // stance, and the pelvis dropped 7 cm to keep it down.
            //
            // Scaled like the thigh: the foot's roll is part of what carries
            // the body forward, and scaled more gently it dominated a short
            // slow stride.
            let toe_up = -self.amplitude * reference.foot_pitch.at(at);
            let shank = thigh_angle - knee_angle;
            let wanted = [thigh_angle, knee_angle, toe_up - shank + self.bind_shank[leg]];
            let [thigh_now, _, knee_now, ankle_now] = self.base_angles[leg];

            // Positive about `KNEE_AXIS` swings a hanging bone forward (see
            // `gait::walk_pose_on`'s authored branch): thigh flexion is
            // positive, knee flexion negative. The foot does not hang, it
            // points forward, and the same positive turn lifts its toe —
            // dorsiflexion is positive. Measured: written negative, the
            // walk's ankle came out mirrored about the standing angle, 2..30
            // degrees against the recording's -20..7.
            let turn = |angle: f32| Quat::from_axis_angle(KNEE_AXIS, facing * angle);
            gait::compose(pose, hip, turn(wanted[0] - thigh_now));
            gait::compose(pose, knee, turn(-(wanted[1] - knee_now)));
            gait::compose(pose, ankle, turn(wanted[2] - ankle_now));
        }
    }

    /// Bends a swinging foot's toes up as far as lifts the tip
    /// [`swing_clearance`] off the floor, and no further, on the finished
    /// walking pose at `phase`, rising from wherever the tip left the floor.
    ///
    /// Held rigid, the tip was 25-31 mm under the floor through pre-swing
    /// (the ball 8-10 mm up: the toes are bending, Winter's toe marker stays
    /// down there) and still 22 mm under early in swing; the foot IK, holding
    /// it out, dragged it 60-75 mm along the floor (200 mm at 1.85 m/s).
    ///
    /// In swing only. Conformed to the floor through stance as well, the
    /// tip slid along it (7 mm a pre-swing at 1 m/s, 18 at 1.6) and root
    /// motion, which follows the contacts, stepped 0.16 m/s as it left; the
    /// support model is built on the rigid foot. In stance the foot IK lifts
    /// the tip out of the floor and holds it where it came down
    /// (`plugin::AnimFootIk::tips`). Not by the foot's pitch either
    /// (`foot::toe_bend`, the run's): bent by the pitch, a tip carrying the
    /// body rose off the floor and the foot floated 9.6 mm. On the finished
    /// pose, not inside [`Self::pose_legs`]: the thigh correction and the
    /// pelvis fit are built on the rigid foot's contacts, and rebuilt on a
    /// bent one they moved. Solved by bisection between straight and
    /// [`super::foot::TOE_BEND`]: a one-sided Newton step overshot and left
    /// the tip 7 mm up.
    pub fn conform_toes(&self, pose: &mut LocalPose, base: &LocalPose, rig: &RigGeometry, phase: f32, facing: f32) {
        let risen = pose.root_translation.y - base.root_translation.y;
        for (leg, &(shift, _, _, ankle)) in LEGS.iter().enumerate() {
            let toes = super::foot::foot_bones(ankle).1;
            let heights = |pose: &LocalPose| self.soles[leg].points(pose, rig).map(|p| risen + p.y - self.ground[leg]);
            let [_, _, tip] = heights(pose);
            let LegPhase::Swing { progress } = gait::leg_phase(phase + shift, self.duty_factor) else {
                continue;
            };
            // Up to the clearance, rising from wherever the tip left the
            // floor: at toe-off it may be well under it, and lifted there at
            // once the toe would jump.
            let x = (progress / TOE_LIFT_BY).min(1.0);
            let target = swing_clearance(progress).min(tip + TOE_SWING_LIFT * x * (2.0 - x));
            // Eased onto its least height rather than clamped at it, so the
            // bend starts without a corner.
            let wanted = gait::soft_floor(tip, target, TOE_CONFORM_SOFTNESS);
            if wanted - tip < 1.0e-5 {
                continue;
            }
            let start = pose.rotations[toes];
            let tip_at = |bend: f32| {
                let mut probe = *pose;
                probe.rotations[toes] = start * Quat::from_axis_angle(KNEE_AXIS, facing * bend);
                heights(&probe)[2]
            };
            let (mut low, mut high) = (0.0, super::foot::TOE_BEND);
            if tip_at(high) <= wanted {
                low = high;
            } else {
                for _ in 0..20 {
                    let mid = 0.5 * (low + high);
                    if tip_at(mid) < wanted {
                        low = mid;
                    } else {
                        high = mid;
                    }
                }
            }
            let bend = 0.5 * (low + high);
            pose.rotations[toes] = start * Quat::from_axis_angle(KNEE_AXIS, facing * bend);
        }
    }

    /// The thigh correction for `leg` at `phase`: periodic Catmull-Rom over
    /// the table, so the correction has no corner anywhere in the cycle.
    fn correction_at(&self, leg: usize, phase: f32) -> f32 {
        let table = &self.correction[leg];
        let x = phase.rem_euclid(1.0) * SAMPLES as f32;
        let i = x.floor() as usize % SAMPLES;
        let t = x - x.floor();
        let at = |k: isize| table[(i as isize + k).rem_euclid(SAMPLES as isize) as usize];
        let (p0, p1, p2, p3) = (at(-1), at(0), at(1), at(2));
        0.5 * (2.0 * p1
            + (p2 - p0) * t
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t * t
            + (3.0 * p1 - p0 - 3.0 * p2 + p3) * t * t * t)
    }

    /// One refinement pass: measure how far each touching contact still
    /// drifts with the current correction, and correct it away.
    fn refine(&mut self, base: &LocalPose, rig: &RigGeometry) {
        let facing = super::stance::facing_sign(rig);
        let forward = rig.forward();
        let soles = self.soles;
        let contacts: Vec<[Contacts; 2]> = (0..=SAMPLES)
            .map(|i| {
                let pose = self.pose(base, rig, i as f32 / SAMPLES as f32, facing);
                [soles[0].points(&pose, rig), soles[1].points(&pose, rig)]
            })
            .collect();

        // How far each planted contact moves in the world over each step:
        // its motion under the hips plus the body's, where the body's is the
        // cancellation `locomotion::root_velocity_of` publishes.
        let mut slide = [[0.0f32; SAMPLES]; 2];
        for i in 0..SAMPLES {
            let middle = (i as f32 + 0.5) / SAMPLES as f32;
            let mut moved = [None; 2];
            let (mut loads, mut heights) = ([0.0f32; 2], [f32::MAX; 2]);
            for leg in 0..2 {
                let LegPhase::Stance { progress } =
                    gait::leg_phase(middle + LEGS[leg].0, self.duty_factor)
                else {
                    continue;
                };
                moved[leg] = Some(contact_moved(&contacts[i][leg], &contacts[i + 1][leg]).dot(forward));
                loads[leg] = gait::stance_load(progress, self.duty_factor);
                // Relative to the hips: both feet share them, so which is
                // lower is the same question as in the world.
                heights[leg] = 0.5 * (lowest(&contacts[i][leg]) + lowest(&contacts[i + 1][leg]));
            }
            let borne = bearing(loads, heights);
            let (mut sum, mut total) = (0.0, 0.0);
            for leg in 0..2 {
                if let Some(step) = moved[leg] {
                    sum += step * borne[leg];
                    total += borne[leg];
                }
            }
            let body = if total > 1.0e-6 {
                -sum / total
            } else {
                let feet = moved.iter().flatten().count().max(1) as f32;
                -moved.iter().flatten().sum::<f32>() / feet
            };
            // A foot's slide counts by the share of the body it carries —
            // the same support weights the pelvis and root motion use. A
            // foot the body is not resting on is not planted, whatever the
            // stance timing says: counted by stance timing, the heel's last
            // 8 cm of forward travel before it took weight bent the whole
            // stance leg back 5.5 degrees; counted by height alone, a heel
            // that had reached the floor but not yet the load bent it 4.2.
            for leg in 0..2 {
                if let Some(step) = moved[leg] {
                    slide[leg][i] = (step + body) * borne[leg];
                }
            }
        }

        // Accumulated from each footfall through stance, and returned to zero
        // through swing: the drift the correction has to take back out.
        //
        // The return is a cubic Hermite that leaves toe-off at the rate the
        // drift was growing and arrives at footfall at the rate the next
        // stance starts it growing. A smoothstep arrives flat instead, and
        // the thigh's angular velocity then jumped at every footfall —
        // caught by `gait::tests::the_cycle_closes_in_position_and_in_velocity`.
        let lever = gait::leg_length_of(rig);
        for leg in 0..2 {
            let footfall = ((1.0 - LEGS[leg].0).rem_euclid(1.0) * SAMPLES as f32).round() as usize;
            let stance_samples = (0..SAMPLES)
                .take_while(|&k| {
                    let phase = ((footfall + k) % SAMPLES) as f32 / SAMPLES as f32;
                    gait::leg_phase(phase + LEGS[leg].0, self.duty_factor).is_stance()
                })
                .count();
            let swing_samples = (SAMPLES - stance_samples).max(1) as f32;
            let rate_at = |k: usize| slide[leg][(footfall + k) % SAMPLES];

            let mut drift = [0.0f32; SAMPLES];
            let mut total = 0.0;
            for (k, slot) in drift.iter_mut().enumerate().take(stance_samples) {
                *slot = total;
                total += rate_at(k);
            }
            let (leave, arrive) = (rate_at(stance_samples.saturating_sub(1)), rate_at(0));
            for (k, slot) in drift.iter_mut().enumerate().skip(stance_samples) {
                let u = (k - stance_samples) as f32 / swing_samples;
                *slot = gait::hermite(total, 0.0, leave * swing_samples, arrive * swing_samples, u);
            }

            for (k, &error) in drift.iter().enumerate() {
                // Drifted forward: swing the thigh back (flexion is positive).
                self.correction[leg][(footfall + k) % SAMPLES] -= error / lever;
            }
        }
    }
}

/// Over how much height the tip is eased onto the floor by
/// [`WalkCycle::conform_toes`], metres.
const TOE_CONFORM_SOFTNESS: f32 = 0.0007;

/// How far [`WalkCycle::conform_toes`] lifts a swinging tip at most over
/// where it left the floor, by [`TOE_LIFT_BY`] of the swing, metres.
const TOE_SWING_LIFT: f32 = 0.05;

/// How far a walking foot's sole keeps off the floor at `progress` through
/// its swing, at least, metres: off at once after toe-off, rising `x(2 - x)`
/// to [`TOE_CLEARANCE`] over the first [`TOE_LIFT_BY`] of the swing, and
/// down to the floor for the landing as `clear_swinging_feet` sets it.
///
/// Kept on the pose ([`WalkCycle::conform_toes`]) and again on the rendered
/// foot (`plugin::AnimFootIk::clear`): the legs' springs trail a foot
/// pitching fast at toe-off, as a run's do (`run::swing_clearance`).
pub fn swing_clearance(progress: f32) -> f32 {
    if progress < 0.5 {
        let x = (progress / TOE_LIFT_BY).min(1.0);
        TOE_CLEARANCE * x * (2.0 - x)
    } else {
        TOE_CLEARANCE * gait::smoothstep((1.0 - progress) / 0.2)
    }
}

/// How much of the swing a walking foot takes to reach [`TOE_CLEARANCE`].
const TOE_LIFT_BY: f32 = 0.15;

/// The least a swinging foot clears the ground by, metres: Winter's measured
/// minimum toe clearance, 1.52 cm (Problem 3.6-4 on Tables A.2(d)), late in
/// swing.
pub const TOE_CLEARANCE: f32 = 0.015;

/// The most [`clear_swinging_feet`] turns an ankle in one Newton step,
/// radians (20 degrees).
const MAX_GUARD_TURN: f32 = 0.35;

/// Lifts any swinging foot that would pass closer to the ground than
/// [`TOE_CLEARANCE`], by turning its ankle — smoothly, and only by what it
/// needs.
///
/// The recording's foot clears the ground by a centimetre and a half; the
/// rig's toes reach further past the toe joint than the subject's toe
/// marker did, and replayed verbatim they brushed the floor in late swing
/// (0 mm at the lowest, measured). Turning the ankle is the smallest change
/// that buys the clearance back.
///
/// Called with the pose complete, hip height included: clearance is a
/// question about where the ground is.
pub fn clear_swinging_feet(
    pose: &mut LocalPose,
    phase: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &RigGeometry,
    facing: f32,
) {
    let risen = pose.root_translation.y - base.root_translation.y;
    for &(shift, _, _, ankle) in &LEGS {
        let LegPhase::Swing { progress } = gait::leg_phase(phase + shift, params.duty_factor) else {
            continue;
        };
        // Zero at both ends of swing, where the foot really is on the ground
        // — leaving it at toe-off and meeting it at heel strike — or the
        // guard would snap the ankle at each hand-over; the full clearance
        // through the middle.
        let wanted = TOE_CLEARANCE * gait::smoothstep(progress.min(1.0 - progress) / 0.2);
        let sole = Sole::of(rig, ankle);
        let ground = lowest(&sole.points(base, rig));
        // Two Newton steps: the lift per radian depends on the foot's pitch.
        for _ in 0..2 {
            let points = sole.points(pose, rig);
            let clearance = risen + lowest(&points) - ground;
            // How far short of the clearance, with the corner at zero rounded
            // over 4 mm — and exactly zero when the foot is clear, so it
            // leaves an unobstructed foot, and every swing's ends, untouched.
            let short = {
                const ROUNDING: f32 = 0.004;
                let x = wanted - clearance;
                if x <= 0.0 {
                    0.0
                } else if x < ROUNDING {
                    x * x / (2.0 * ROUNDING)
                } else {
                    x - ROUNDING * 0.5
                }
            };
            if short.abs() <= 1.0e-5 {
                break;
            }
            // Metres of lift per radian of toe-up: the lowest point's
            // horizontal reach ahead of the ankle — NEGATIVE when it is the
            // heel, late in swing as the foot readies to strike, and then it
            // is the toe going down that lifts it. Signed: clamped positive,
            // a toe-up turn pushed a low heel 10 mm into the floor.
            let ankle_at = super::rig::offset_from(pose, rig, Bone::Hips, ankle);
            let shares = super::foot::shares(&points);
            let reach: f32 = points
                .iter()
                .zip(shares)
                .map(|(p, s)| (*p - ankle_at).dot(rig.forward()) * s)
                .sum();
            if reach.abs() < 0.02 {
                break;
            }
            // An ankle turns only so far. Unclamped, a foot 18.6 cm under
            // the floor (the synthetic rig's shifted leg segments) asked for
            // ~3 rad, where rounding in the phase moved the result 1e-3 rad
            // between the two legs.
            let turn = (short / reach).clamp(-MAX_GUARD_TURN, MAX_GUARD_TURN);
            gait::compose(pose, ankle, Quat::from_axis_angle(KNEE_AXIS, facing * turn));
        }
    }
}

/// Recently built cycles, most recent first.
static CYCLES: Mutex<Vec<(u64, Arc<WalkCycle>)>> = Mutex::new(Vec::new());

/// How many distinct cycles to keep: a handful of characters, speeds and rigs.
const CACHED: usize = 16;

/// The walk cycle for these inputs, built on first use.
pub fn walk_cycle(
    params: &GaitParams,
    amplitude: f32,
    base: &LocalPose,
    rig: &RigGeometry,
) -> Arc<WalkCycle> {
    let key = key_of(params, amplitude, base, rig);
    if let Ok(mut cache) = CYCLES.lock()
        && let Some(found) = cache.iter().position(|(k, _)| *k == key)
    {
        let entry = cache.remove(found);
        let cycle = entry.1.clone();
        cache.insert(0, entry);
        return cycle;
    }

    let cycle = Arc::new(WalkCycle::build(params, amplitude, base, rig));
    if let Ok(mut cache) = CYCLES.lock() {
        cache.insert(0, (key, cycle.clone()));
        cache.truncate(CACHED);
    }
    cycle
}

/// Everything a cycle depends on, hashed bit for bit.
fn key_of(params: &GaitParams, amplitude: f32, base: &LocalPose, rig: &RigGeometry) -> u64 {
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    let mut feed = |values: &[f32]| values.iter().for_each(|v| v.to_bits().hash(&mut hasher));
    feed(&[amplitude, params.duty_factor]);
    for q in base.rotations.0.iter().chain(rig.bind_rotations.0.iter()) {
        feed(&q.to_array());
    }
    feed(&rig.root_rotation.to_array());
    feed(&base.root_translation.to_array());
    for v in rig.offsets.0.iter().chain(rig.toe_end_offsets.0.iter()) {
        feed(&v.to_array());
    }
    hasher.finish()
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::gait::walk_pose_on;

    /// A walking toe tip leaves the floor at once after toe-off and keeps
    /// [`swing_clearance`] off it through the swing ([`WalkCycle::conform_toes`]).
    /// Rigid, it was 22 mm under the floor early in swing at 0.7 m/s.
    /// Through stance the walk's own foot is untouched.
    #[test]
    fn a_walking_toe_swings_clear_of_the_floor() {
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let sole = Sole::of(&rig, Bone::LeftFoot);
        let ground = lowest(&sole.points(&stood, &rig));
        for speed in [0.7f32, 1.2, 1.6] {
            let params = GaitParams::walking_on(speed, &rig);
            let gait::LegCurves::Measured { amplitude } = params.curves else { panic!("a measured walk") };
            let cycle = walk_cycle(&params, amplitude, &stood, &rig);
            let facing = crate::character::anim::stance::facing_sign(&rig);
            for i in 0..200 {
                let p = i as f32 / 200.0;
                let pose = walk_pose_on(p, &params, &stood, &rig);
                let tip = pose.root_translation.y - stood.root_translation.y + sole.points(&pose, &rig)[2].y - ground;
                match gait::leg_phase(p, params.duty_factor) {
                    LegPhase::Swing { progress } if (0.15..0.85).contains(&progress) => {
                        assert!(tip > 0.9 * swing_clearance(progress) - 5.0e-4, "{speed} m/s, phase {p}: the swinging tip only {} mm up", tip * 1e3);
                    }
                    LegPhase::Stance { .. } => {
                        let rigid = cycle.pose(&stood, &rig, p, facing);
                        assert_eq!(pose.rotations[Bone::LeftToeBase], rigid.rotations[Bone::LeftToeBase], "{speed} m/s: a stance toe bent at {p}");
                    }
                    _ => {}
                }
            }
        }
    }
}
