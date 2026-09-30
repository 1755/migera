//! Stage 2 — continuous phase oscillators, the layer that keeps a
//! character alive when nothing else is driving it.
//!
//! # The problem this solves
//!
//! Stage 1 springs settle. Once a character reaches its target pose it
//! stops dead, and a person who stops dead reads as a mannequin — real
//! bodies are never perfectly still. The conventional fix is to hand-key a
//! looping idle, which is slow to author and gives every character the same
//! motion at the same speed forever.
//!
//! Instead, a small set of sine waves is layered additively on top of
//! whatever the target pose says. They cost nothing, never repeat exactly
//! (because their periods are deliberately non-commensurate), and scale
//! automatically with how fast the character is moving.
//!
//! # The analytical stand-in for a learned phase manifold
//!
//! Neural approaches (DeepPhase, Periodic Autoencoders) extract latent
//! periodic structure from large motion-capture corpora via FFT, then
//! synthesize motion from it. The insight worth stealing without the
//! training pipeline is the *inductive bias*: locomotion is fundamentally
//! periodic, and a phase variable plus a few harmonics captures a
//! surprising amount of it.
//!
//! So [`GaitPhase`] is a single scalar advanced by the character's own
//! speed, and each [`PhaseOscillator`] reads it at some multiple and
//! offset. That is the whole model.
//!
//! # What the research says the numbers should be
//!
//! The superseded module's own idle work (Mixamo/mocap-industry
//! breakdowns, recorded in
//! `docs/knowledge/character-animation/lugaru-joint-muscle-system.md`)
//! landed on three findings that the defaults here encode directly:
//!
//! - **Amplitude hierarchy** — weight shift is the largest motion,
//!   breathing is ~1-2 cm at the spine, head bob is smallest. Getting this
//!   order wrong reads as a twitch rather than a breath.
//! - **Layered non-synchronized cycles** — breathing runs at ~3-4 s on top
//!   of a slower ~6-8 s weight shift. Sharing one clock is what makes an
//!   idle feel mechanical.
//! - **Deliberate left/right asymmetry** — perfectly mirrored motion is the
//!   single biggest tell of a robotic idle.

use bevy::math::{Quat, Vec3};
use serde::{Deserialize, Serialize};

use super::rig::LocalPose;
use crate::character::skeleton::Bone;

/// A full turn, in radians. Phase is measured in radians so a harmonic
/// multiplier reads directly as "cycles per stride".
pub const TAU: f32 = std::f32::consts::TAU;

/// The continuously advancing clock every oscillator reads.
///
/// Two clocks, deliberately: [`gait`] tracks strides and therefore scales
/// with speed, while [`breath`] runs at its own steady rate regardless.
/// Merging them would make a character breathe faster simply because it
/// walked faster, which is the mechanical feel the research warns about.
///
/// [`gait`]: Self::gait
/// [`breath`]: Self::breath
#[derive(bevy::ecs::component::Component, Debug, Clone, Copy, PartialEq)]
pub struct GaitPhase {
    /// Stride phase, radians, wrapped to `[0, TAU)`.
    ///
    /// **Never reset this on a state change.** Snapping it to zero
    /// discontinuously teleports every oscillator that reads it, which
    /// reads as a visible hitch — and, once Stage 3 locks feet to it, as a
    /// foot sliding across the ground.
    pub gait: f32,
    /// Breathing phase, radians, wrapped to `[0, TAU)`.
    pub breath: f32,
    /// Strides per second when standing still.
    pub base_frequency_hz: f32,
    /// Extra strides per second per metre-per-second of travel.
    pub speed_coefficient: f32,
    /// Breaths per second. Independent of movement.
    pub breath_frequency_hz: f32,
    /// The character's horizontal speed, m/s. A consumer's locomotion
    /// controller writes this; the plugin only reads it.
    pub speed: f32,
    /// Seconds this clock has run, unwrapped. For events that happen now
    /// and then rather than every cycle — see [`QuietSway::weight_shift`].
    pub elapsed: f32,
}

impl Default for GaitPhase {
    fn default() -> Self {
        Self {
            gait: 0.0,
            breath: 0.0,
            // A slow sway while standing: ~7 s per cycle, the middle of the
            // research's own 6-8 s weight-shift window.
            base_frequency_hz: 1.0 / 7.0,
            // ~1.5 strides/s at a 1.4 m/s walk, which is close to real
            // human cadence.
            speed_coefficient: 0.9,
            // ~4 s per breath, the slow end of the research's 3-4 s range,
            // chosen so it drifts against the sway rather than locking to
            // it (see `the_two_clocks_drift_apart_rather_than_locking`).
            breath_frequency_hz: 1.0 / 4.0,
            speed: 0.0,
            elapsed: 0.0,
        }
    }
}

impl GaitPhase {
    /// Advances both clocks by `dt`.
    ///
    /// Wrapping uses `rem_euclid`, so the phase stays in `[0, TAU)` exactly
    /// and cannot accumulate the drift a repeated subtract-if-greater would
    /// introduce over a long session.
    pub fn advance(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        let gait_hz = self.base_frequency_hz + self.speed_coefficient * self.speed.abs();
        self.elapsed += dt;
        self.gait = (self.gait + TAU * gait_hz * dt).rem_euclid(TAU);
        self.breath = (self.breath + TAU * self.breath_frequency_hz * dt).rem_euclid(TAU);
    }

    /// Current stride frequency, Hz.
    pub fn gait_frequency_hz(&self) -> f32 {
        self.base_frequency_hz + self.speed_coefficient * self.speed.abs()
    }
}

/// Which clock an oscillator reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PhaseClock {
    /// Scales with the character's speed — spinal twist, hip sway, head bob.
    Gait,
    /// Runs at its own steady rate — breathing, and any idle motion that
    /// should not speed up when the character walks.
    Breath,
}

/// One additive sine wave applied to one bone.
///
/// The emitted rotation is
/// `angle = amplitude * sin(harmonic * phase + offset)`
/// about `axis`, in the bone's own rest frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PhaseOscillator {
    /// Which clock to read.
    pub clock: PhaseClock,
    /// Cycles per clock cycle. `1.0` is once per stride; `2.0` is the head
    /// bob, which peaks at each footfall and so runs at twice stride rate.
    pub harmonic: f32,
    /// Peak rotation, radians.
    pub amplitude: f32,
    /// Phase offset, radians. `PI/2` turns a sine into a cosine, which is
    /// how hip sway is kept a quarter-cycle out of step with spinal twist.
    pub offset: f32,
    /// Rotation axis in the bone's own rest frame.
    pub axis: Vec3,
}

impl PhaseOscillator {
    /// The rotation this oscillator contributes right now.
    pub fn sample(&self, phase: &GaitPhase) -> Quat {
        let clock = match self.clock {
            PhaseClock::Gait => phase.gait,
            PhaseClock::Breath => phase.breath,
        };

        let angle = self.amplitude * (self.harmonic * clock + self.offset).sin();
        let axis = self.axis.normalize_or_zero();

        if axis == Vec3::ZERO || angle == 0.0 {
            Quat::IDENTITY
        } else {
            Quat::from_axis_angle(axis, angle)
        }
    }
}

/// Every oscillator driving one character.
///
/// A `Vec` rather than one-per-bone: most bones have none, several have two
/// (a sway and a breath), and the whole set is walked once per frame
/// regardless.
///
/// Not `Serialize`/`Deserialize` directly: `Bone` is a plain enum with no
/// serde impls, and giving it one would key authored files by variant
/// order. The asset layer keys bones by *name* for exactly that reason (a
/// reordered enum must not silently repoint every file), so a serialized
/// phase layer belongs in `asset.rs` alongside the pose format when the
/// studio needs it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PhaseLayer {
    /// `(bone, oscillator)` pairs. A bone may appear more than once; its
    /// contributions compose.
    pub oscillators: Vec<(Bone, PhaseOscillator)>,
    /// Quiet-standing body sway, if any: the pelvis shifted over planted
    /// feet. Needs the rig, so only [`PhaseLayer::apply_on`] applies it.
    pub sway: Option<QuietSway>,
    /// A walking body's side-to-side sway over its stance feet, if any.
    /// Needs the rig, so only [`PhaseLayer::apply_on`] applies it.
    pub walk_sway: Option<WalkSway>,
}

/// The side-to-side sway of a walking body, after Winter §11.3.1.
///
/// Each stance foot carries the centre of pressure, and the body is an
/// inverted pendulum over it (Eq. 11.3, `COP − COM = −K·COM̈`): pulled
/// toward the foot it stands on, and turned back by the next foot's
/// pressure before it gets there. The COM passes just medial of each stance
/// foot and never over it. [`walk_sway_at`] is that periodic path; the
/// pelvis carries it, with both legs turned about their ankles so the feet
/// stay where they are ([`super::stance::sway_over_feet`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WalkSway {
    /// How much of the sway applies, `0..=1`: the walk's weight, while a
    /// walk starts or stops.
    pub gain: f32,
}

/// Stride harmonics [`walk_sway_at`] keeps: the odd ones, where all of a
/// side-to-side path is. The first alone is 98 % of it; the pendulum
/// divides the third by ~30 against the first.
const WALK_SWAY_HARMONICS: [f32; 3] = [1.0, 3.0, 5.0];

/// The walking centre of mass's side-to-side offset at stride position
/// `cycle` (0 = left heel contact), toward the rig's left, metres.
///
/// The centre of pressure is under the left foot (`+width/2`) through left
/// single support and the right (`−width/2`) through right, and crosses
/// linearly through each double support: a trapezoid wave, which is a
/// square wave smoothed over the double support, `duty − 0.5` of the
/// stride. The pendulum's periodic answer divides each harmonic `k` of it
/// by `1 + K·(2πk/T)²`, `T` the stride's seconds.
///
/// Measured against a direct numerical solution of the same model on
/// `puppet_base` (13 cm step width, K = 0.104 s²): ±2.27 / ±1.78 / ±1.51 cm
/// at 0.7 / 1.2 / 1.6 m/s, furthest out at ~0.31 of the stride, in left
/// single support.
pub fn walk_sway_at(cycle: f32, stride_seconds: f32, duty_factor: f32, width: f32, pendulum_k: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    if stride_seconds <= 0.0 || !stride_seconds.is_finite() {
        return 0.0;
    }
    let double = (duty_factor - 0.5).clamp(0.0, 0.5);
    WALK_SWAY_HARMONICS
        .iter()
        .map(|&k| {
            // The square wave's harmonic, 4/(πk) of its half-height, spread
            // over the double support (a box's sinc), then the pendulum.
            let spread = if double > 0.0 { (PI * k * double).sin() / (PI * k * double) } else { 1.0 };
            let omega = TAU * k / stride_seconds;
            let pendulum = 1.0 / (1.0 + pendulum_k * omega * omega);
            4.0 / (PI * k) * spread * pendulum * (TAU * k * (cycle - 0.5 * double)).sin()
        })
        .sum::<f32>()
        * width
        * 0.5
}

/// Gravity, m/s², for the pendulum constant `K = d / g`.
const GRAVITY: f32 = 9.81;

/// The walking pelvis's roll over a stride, as `(harmonic, cos, sin)`
/// terms of the turn about the rig's forward: positive lifts the left side.
///
/// Derived from Winter §7.4.5 (Figs 7.4–7.5, intersubject averages, a
/// ~1 s stride). The stance hip's frontal power over its abductor moment
/// is the hip's frontal angular velocity; integrated through stance it
/// gives the swing side of the pelvis dropping under H1-F absorption to
/// 3.9° at 17 % of the stride, part-lifted by H2-F (17–30 %), held through
/// midstance, then lifted back by H3-F (43–57 %) to within 0.7° of level
/// at the swing foot's heel contact. The right side
/// mirrors it half a stride later, so only odd harmonics appear; three fit
/// the integrated curve within 0.37°. Read off the figures by eye, so
/// ±20 % in size; the timing is firmer than the amplitude.
const OBLIQUITY: [(f32, f32, f32); 3] =
    [(1.0, -0.008_02, 0.055_89), (3.0, -0.015_02, 0.004_86), (5.0, 0.003_85, -0.005_33)];

/// How far the walking pelvis turns about the vertical each way, radians
/// (4°).
///
/// Not Winter's: his transverse hip data (§7.4.5, H1-T) cannot give it.
/// Its moments and powers are small (±0.2 N·m/kg, −0.15 W/kg), and the hip
/// angle they integrate to is the pelvis against a stance femur that itself
/// rotates in the world. Winter gives the timing: the stance hip's external
/// rotators brake the pelvis turning over the stance limb just after heel
/// contact (~10 %), so each side is furthest forward at its own heel
/// contact. The size is the commonly cited gait-lab figure (Perry, *Gait
/// Analysis*, 1992: 4° forward and back at a normal walk).
pub const PELVIC_ROTATION: f32 = 0.070;

/// The walking pelvis's turn about the vertical at stride position `cycle`
/// (0 = left heel contact), radians about +Y: positive brings the right
/// side forward. Each side is furthest forward at its own heel contact:
/// the left at 0, the right at 0.5. See [`PELVIC_ROTATION`].
///
/// About +Y a positive turn swings the rig's forward toward its left, so
/// the right side comes forward, whichever way the rig faces.
pub fn pelvic_rotation_at(cycle: f32) -> f32 {
    -PELVIC_ROTATION * (std::f32::consts::TAU * cycle).cos()
}

/// The walking pelvis's roll at stride position `cycle` (0 = left heel
/// contact), radians about the rig's forward: positive lifts the left
/// side. See [`OBLIQUITY`].
pub fn pelvic_obliquity_at(cycle: f32) -> f32 {
    use std::f32::consts::TAU;
    OBLIQUITY
        .iter()
        .map(|&(k, cos, sin)| cos * (TAU * k * cycle).cos() + sin * (TAU * k * cycle).sin())
        .sum()
}

/// The whole-body centre of mass's height above the ankles, as a multiple
/// of the hips': 0.934 / 0.856 m on `puppet_base` standing as drawn
/// (`anthropometry::centre_of_mass`), about 54% of its height from the
/// floor. A ratio rather than the COM itself: the pendulum needs it every
/// frame, and evaluating the COM there cost ~2 µs a character, as much as
/// the rest of the phase layer and the springs together.
///
/// Was 1.19 until 2026-10-01, measured on plain `puppet_base()`, where the
/// world-axis `relaxed_stand` holds the arms overhead: a COM 62% up.
const COM_OVER_HIPS: f32 = 1.09;

/// [`walk_sway_at`] for a character walking on `phase`'s clock in `pose`.
///
/// The stride's seconds are the clock's own; the step width is the walk's
/// (`stance::step_width`); `K = d / g`, `d` the centre of mass's height
/// above the ankles ([`COM_OVER_HIPS`]), for a point mass. Winter's K for
/// a distributed body is somewhat larger (~0.1 s² at d ≈ 0.9 m); 1.02 m on
/// `puppet_base` gives 0.104.
///
/// Also returns how the body rests on each foot (left, right), from the
/// walk's stance timing, for the vertical that keeps the planted feet down.
fn walk_sway_now(
    phase: &GaitPhase,
    pose: &super::rig::LocalPose,
    rig: &super::rig::RigGeometry,
) -> (f32, [f32; 2]) {
    use super::gait::{leg_phase, stance_load, LegPhase};
    use super::rig::offset_from;
    let hz = phase.gait_frequency_hz();
    let duty = super::gait::GaitParams::walking_on(phase.speed, rig).duty_factor;
    let cycle = phase.gait / std::f32::consts::TAU;
    let loads = [0.0, 0.5].map(|shift| match leg_phase(cycle + shift, duty) {
        LegPhase::Stance { progress } => stance_load(progress, duty),
        _ => 0.0,
    });
    if hz <= 0.0 {
        return (0.0, loads);
    }
    let width = super::stance::step_width(pose, rig);
    let ankles = 0.5
        * (offset_from(pose, rig, Bone::Hips, Bone::LeftFoot).y + offset_from(pose, rig, Bone::Hips, Bone::RightFoot).y);
    let height = -ankles * COM_OVER_HIPS;
    (walk_sway_at(cycle, 1.0 / hz, duty, width, height.max(0.0) / GRAVITY), loads)
}

/// The sway of a body standing still, after Winter §11.2.1.
///
/// Standing is an inverted pendulum balanced by moving the centre of
/// pressure, and the body's sway is the pelvis carrying the trunk over the
/// feet — not the trunk bending over a fixed pelvis. Side to side the hips
/// load one leg and unload the other and the pelvis shifts toward the loaded
/// foot; front to back the body pivots at the ankles. So each is a pelvis
/// translation, with both legs turned about their ankles to keep the feet
/// planted and flat ([`super::stance::sway_over_feet`]).
///
/// Winter's quiet stance (Fig. 11.4) keeps the side-to-side centre of
/// pressure within about +0.8/−1.2 cm, and the centre of mass moves less
/// than the centre of pressure that steers it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuietSway {
    /// Side-to-side pelvis travel, peak, metres, on the gait clock (which a
    /// standing character runs slowly).
    pub lateral: f32,
    /// Front-to-back pelvis travel, peak, metres, on the breath clock —
    /// breathing is one of the drivers of standing sway the book names.
    pub fore_aft: f32,
    /// How far the occasional deliberate weight shift goes, as a fraction
    /// of a full one (`stance::shift_weight`); 0 disables it.
    pub weight_shift: f32,
}

/// Length of one weight-shift slot, seconds. See [`QuietSway::weight_shift`].
const SHIFT_SLOT: f32 = 14.0;
/// How long a weight shift takes to settle in or out, seconds.
const SHIFT_EASE: f32 = 1.6;

impl QuietSway {
    /// The pelvis shift now, in the rig's frame.
    pub fn shift(&self, phase: &GaitPhase, rig: &super::rig::RigGeometry) -> Vec3 {
        rig.left() * (self.lateral * phase.gait.sin()) + rig.forward() * (self.fore_aft * phase.breath.sin())
    }

    /// The deliberate weight shift at `elapsed` seconds: +1 fully onto the
    /// left leg, −1 the right, 0 standing square.
    ///
    /// Winter §11.2.1 separates the two: quiet standing sway is small and
    /// continuous, while a visible shift of the weight onto one leg is a
    /// postural CHANGE, made now and then and held. So time is cut into
    /// [`SHIFT_SLOT`]-second slots, and each either stands square or shifts
    /// onto one leg, easing in and out over [`SHIFT_EASE`] seconds. Which,
    /// is a fixed integer hash of the slot's index: deterministic, so a
    /// replay is the same replay, without looking periodic. The first slot
    /// always stands square, so a character spawns standing evenly.
    pub fn weight_shift(&self, elapsed: f32) -> f32 {
        if self.weight_shift == 0.0 || elapsed < SHIFT_SLOT {
            return 0.0;
        }
        let slot = (elapsed / SHIFT_SLOT).floor();
        let within = elapsed - slot * SHIFT_SLOT;
        // "lowbias32", a full-avalanche integer finalizer: a plain
        // multiplicative hash left five same-side shifts in a row.
        let mut hash = slot as u32;
        hash ^= hash >> 16;
        hash = hash.wrapping_mul(0x7feb_352d);
        hash ^= hash >> 15;
        hash = hash.wrapping_mul(0x846c_a68b);
        hash ^= hash >> 16;
        let side = match hash % 3 {
            0 => 0.0,
            1 => 1.0,
            _ => -1.0,
        };
        let ease = |t: f32| {
            let t = (t / SHIFT_EASE).clamp(0.0, 1.0);
            t * t * (3.0 - 2.0 * t)
        };
        self.weight_shift * side * ease(within) * ease(SHIFT_SLOT - within)
    }
}

impl PhaseLayer {
    /// No procedural motion at all.
    pub fn none() -> Self {
        Self::default()
    }

    /// Composes every oscillator's contribution onto `pose`, in place.
    ///
    /// Applied **before** the spring reads the pose, so the spring smooths
    /// the result rather than the oscillators fighting it. Composing after
    /// the spring would reintroduce the raw sine directly onto the rendered
    /// bone and undo the spring's own smoothing.
    ///
    /// Rotations only; [`PhaseLayer::apply_on`] also applies the
    /// [`QuietSway`], which needs the rig.
    pub fn apply(&self, phase: &GaitPhase, pose: &mut LocalPose) {
        for (bone, oscillator) in &self.oscillators {
            let contribution = oscillator.sample(phase);
            if contribution == Quat::IDENTITY {
                continue;
            }
            pose.set_rotation(*bone, pose.rotation(*bone) * contribution);
        }
    }

    /// [`PhaseLayer::apply`], plus the body's sway over its feet on `rig`.
    pub fn apply_on(&self, phase: &GaitPhase, pose: &mut LocalPose, rig: &super::rig::RigGeometry) {
        self.apply(phase, pose);
        if let Some(sway) = self.sway {
            super::stance::shift_weight(pose, rig, sway.weight_shift(phase.elapsed));
            super::stance::sway_over_feet(pose, rig, sway.shift(phase, rig));
        }
        if let Some(walk) = self.walk_sway
            && walk.gain > 0.0
            && phase.speed > 0.0
        {
            let (shift, loads) = walk_sway_now(phase, pose, rig);
            let cycle = phase.gait / std::f32::consts::TAU;
            let turn = Quat::from_axis_angle(Vec3::Y, walk.gain * pelvic_rotation_at(cycle))
                * Quat::from_axis_angle(rig.forward(), walk.gain * pelvic_obliquity_at(cycle));
            super::stance::move_pelvis_over_feet(pose, rig, rig.left() * (walk.gain * shift), turn, loads);
        }
    }

    /// A layer part-way from `from` to `to`: `from`'s oscillators faded by
    /// `1 − weight`, `to`'s by `weight`, and the sway of whichever side has
    /// most of the weight.
    ///
    /// For a character between standing and walking. Swapping one layer for
    /// the other at a threshold dropped the walk's 0.09 rad spinal twist in
    /// one frame when a stop ended, and ran it at full size through the
    /// standing preparation before a first step.
    pub fn between(from: &Self, to: &Self, weight: f32) -> Self {
        let weight = weight.clamp(0.0, 1.0);
        fn faded(layer: &PhaseLayer, gain: f32) -> impl Iterator<Item = (Bone, PhaseOscillator)> + '_ {
            layer
                .oscillators
                .iter()
                .filter(move |_| gain > 0.0)
                .map(move |(bone, oscillator)| {
                    (*bone, PhaseOscillator { amplitude: oscillator.amplitude * gain, ..*oscillator })
                })
        }
        // A walk's sway fades with its gait, weight for weight.
        let walk_sway = |layer: &PhaseLayer, gain: f32| {
            layer.walk_sway.filter(|_| gain > 0.0).map(|sway| WalkSway { gain: sway.gain * gain })
        };
        Self {
            oscillators: faded(from, 1.0 - weight).chain(faded(to, weight)).collect(),
            sway: if weight < 0.5 { from.sway } else { to.sway },
            walk_sway: walk_sway(to, weight).or(walk_sway(from, 1.0 - weight)),
        }
    }

    /// The research-backed standing idle: a slow weight-shifting sway, a
    /// slower breath, and a small head counter-motion.
    ///
    /// Amplitudes follow the documented hierarchy — sway largest, breathing
    /// next, head smallest — and the left/right pair is deliberately
    /// unequal, because perfectly mirrored motion is the clearest tell of a
    /// robotic idle.
    pub fn standing_idle() -> Self {
        Self {
            // Weight shift: the largest motion. The pelvis carries the body
            // side to side over planted feet and rocks front to back with
            // the breath — see `QuietSway` for why it is the pelvis that
            // moves. This used to bend the spine sideways over a fixed
            // pelvis instead (0.035 + 0.018 rad at `Spine`/`Spine1`), which
            // swung the shoulders 3.2 cm while the centre of mass moved 1.3
            // and barely 0.16 cm front to back.
            //
            // ±6 mm side to side puts the centre of mass through ~1.2 cm,
            // inside Winter's ~2 cm of side-to-side centre of pressure (the
            // mass moves less than the pressure steering it); ±4 mm front
            // to back on the breath clock.
            sway: Some(QuietSway { lateral: 0.006, fore_aft: 0.004, weight_shift: 1.0 }),
            walk_sway: None,
            oscillators: vec![
                // Breathing: the chest rises and falls on its own clock.
                // Smaller than the body's sway: at 0.022 rad it carried the
                // head 6.2 mm, as far as the whole-body weight shift does.
                (
                    Bone::Spine2,
                    PhaseOscillator {
                        clock: PhaseClock::Breath,
                        harmonic: 1.0,
                        amplitude: 0.012,
                        offset: 0.0,
                        axis: Vec3::X,
                    },
                ),
                // The head holds level against the sway — smallest
                // amplitude, and a quarter cycle behind so it lags rather
                // than moving in lockstep.
                (
                    Bone::Neck,
                    PhaseOscillator {
                        clock: PhaseClock::Gait,
                        harmonic: 1.0,
                        amplitude: 0.012,
                        offset: -std::f32::consts::FRAC_PI_2,
                        axis: Vec3::Z,
                    },
                ),
                // Arms hang and swing very slightly with the sway.
                // Deliberately unequal left and right.
                (
                    Bone::LeftArm,
                    PhaseOscillator {
                        clock: PhaseClock::Gait,
                        harmonic: 1.0,
                        amplitude: 0.020,
                        offset: 0.0,
                        axis: Vec3::X,
                    },
                ),
                (
                    Bone::RightArm,
                    PhaseOscillator {
                        clock: PhaseClock::Gait,
                        harmonic: 1.0,
                        amplitude: 0.016,
                        offset: 0.35,
                        axis: Vec3::X,
                    },
                ),
            ],
        }
    }

    /// Locomotion layering: the pelvis's sway and roll over the stance
    /// feet ([`WalkSway`]), a spinal counter-twist, and a head bob at twice
    /// stride rate.
    ///
    /// The spine twists against the hips to conserve angular momentum, and
    /// the head takes a vertical impulse at every footfall — hence twice per
    /// stride, not once. The pelvis's roll used to be an oscillator here too
    /// (0.05 rad about the Hips' Z); it lowered the stance side through late
    /// single support, the reverse of Winter's, and swung each planted sole
    /// 45–48 mm. It is now [`pelvic_obliquity_at`], rolled over the stance
    /// hip.
    pub fn locomotion() -> Self {
        Self {
            // A walking body's rise and fall come from its legs; its
            // side-to-side sway and roll from the pendulum over its stance
            // feet and the hip abductors holding it.
            sway: None,
            walk_sway: Some(WalkSway { gain: 1.0 }),
            oscillators: vec![
                (
                    Bone::Spine1,
                    PhaseOscillator {
                        clock: PhaseClock::Gait,
                        harmonic: 1.0,
                        amplitude: 0.09,
                        // At its extremes at the heel contacts, with the arm
                        // swing and against the pelvis's turn: the right
                        // shoulder furthest forward as the left heel lands.
                        // At 0 the chest turned furthest at midstance,
                        // a fifth of a stride behind the arms (measured on
                        // `puppet_base`).
                        offset: std::f32::consts::FRAC_PI_2,
                        axis: Vec3::Y,
                    },
                ),
                (
                    Bone::Neck,
                    PhaseOscillator {
                        clock: PhaseClock::Gait,
                        harmonic: 2.0,
                        amplitude: 0.03,
                        offset: 0.0,
                        axis: Vec3::X,
                    },
                ),
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::FRAC_PI_2;

    fn advanced(phase: &mut GaitPhase, seconds: f32, dt: f32) {
        let steps = (seconds / dt).round() as usize;
        for _ in 0..steps {
            phase.advance(dt);
        }
    }

    #[test]
    fn the_phase_advances_and_wraps_cleanly() {
        let mut phase = GaitPhase { base_frequency_hz: 1.0, ..Default::default() };

        // One full second at 1 Hz should return to (almost) where it began.
        advanced(&mut phase, 1.0, 1.0 / 240.0);

        assert!(
            phase.gait < 0.02 || phase.gait > TAU - 0.02,
            "after exactly one cycle the phase should be back near zero, got {}",
            phase.gait,
        );
    }

    #[test]
    fn the_phase_never_leaves_its_range_even_after_a_long_session() {
        // `rem_euclid` rather than a subtract loop, so a long run cannot
        // accumulate drift or escape the range.
        let mut phase = GaitPhase { base_frequency_hz: 3.0, ..Default::default() };
        advanced(&mut phase, 600.0, 1.0 / 60.0);

        assert!(
            (0.0..TAU).contains(&phase.gait),
            "the phase must stay in [0, TAU) after 10 minutes, got {}",
            phase.gait,
        );
        assert!((0.0..TAU).contains(&phase.breath));
    }

    #[test]
    fn the_phase_advances_monotonically_within_a_cycle() {
        // A phase that ever went backwards would reverse every oscillator
        // reading it.
        let mut phase = GaitPhase { base_frequency_hz: 0.5, ..Default::default() };
        let dt = 1.0 / 120.0;

        let mut previous = phase.gait;
        for _ in 0..100 {
            phase.advance(dt);
            if phase.gait >= previous {
                assert!(phase.gait > previous, "the phase must actually advance");
            }
            previous = phase.gait;
        }
    }

    #[test]
    fn walking_faster_advances_the_gait_phase_faster() {
        // The whole point of a speed-coupled phase: cadence rises with
        // travel, so a run does not look like a slow walk played fast.
        let dt = 1.0 / 120.0;

        let mut standing = GaitPhase::default();
        let mut walking = GaitPhase { speed: 1.4, ..Default::default() };
        let mut running = GaitPhase { speed: 4.0, ..Default::default() };

        advanced(&mut standing, 0.5, dt);
        advanced(&mut walking, 0.5, dt);
        advanced(&mut running, 0.5, dt);

        assert!(
            walking.gait > standing.gait,
            "walking ({}) should outpace standing ({})",
            walking.gait,
            standing.gait,
        );
        assert!(
            running.gait > walking.gait,
            "running ({}) should outpace walking ({})",
            running.gait,
            walking.gait,
        );
    }

    #[test]
    fn a_stationary_character_still_breathes_and_sways() {
        // Zero speed must not mean zero motion — that is exactly the
        // mannequin this stage exists to prevent.
        let mut phase = GaitPhase::default();
        advanced(&mut phase, 1.0, 1.0 / 60.0);

        assert!(phase.gait > 0.0, "a standing character should still sway");
        assert!(phase.breath > 0.0, "and should still breathe");
    }

    #[test]
    fn the_breath_clock_ignores_walking_speed() {
        // Breathing faster merely because the character walks faster is the
        // mechanical tell the research warns about.
        let dt = 1.0 / 120.0;

        let mut standing = GaitPhase::default();
        let mut sprinting = GaitPhase { speed: 8.0, ..Default::default() };

        advanced(&mut standing, 2.0, dt);
        advanced(&mut sprinting, 2.0, dt);

        assert!(
            (standing.breath - sprinting.breath).abs() < 1.0e-4,
            "the breath clock must be independent of speed, got {} vs {}",
            standing.breath,
            sprinting.breath,
        );
    }

    #[test]
    fn the_two_clocks_drift_apart_rather_than_locking() {
        // Non-commensurate periods are what keep a long idle from settling
        // into an obvious repeat. If the two clocks locked, the whole idle
        // would visibly loop at their shared period.
        let mut phase = GaitPhase::default();
        let dt = 1.0 / 60.0;

        let mut smallest_difference = f32::INFINITY;
        let mut largest_difference: f32 = 0.0;

        for _ in 0..60 * 60 {
            phase.advance(dt);
            let difference = (phase.gait - phase.breath).abs();
            smallest_difference = smallest_difference.min(difference);
            largest_difference = largest_difference.max(difference);
        }

        assert!(
            largest_difference - smallest_difference > 1.0,
            "the two clocks should drift through a wide range of relative phase, but \
             stayed within {} rad of each other — they are locked",
            largest_difference - smallest_difference,
        );
    }

    #[test]
    fn an_oscillator_at_zero_amplitude_contributes_nothing() {
        let oscillator = PhaseOscillator {
            clock: PhaseClock::Gait,
            harmonic: 1.0,
            amplitude: 0.0,
            offset: 0.0,
            axis: Vec3::Z,
        };

        let phase = GaitPhase { gait: 1.2, ..Default::default() };
        assert_eq!(oscillator.sample(&phase), Quat::IDENTITY);
    }

    #[test]
    fn an_empty_layer_leaves_the_pose_bit_identical() {
        // The layer must be a true no-op when disabled, so Stage 2 can be
        // switched off without perturbing Stage 1 at all.
        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::LeftArm, Quat::from_axis_angle(Vec3::Y, 0.5));
        let before = pose;

        PhaseLayer::none().apply(&GaitPhase::default(), &mut pose);

        for &bone in Bone::ALL.iter() {
            assert_eq!(
                pose.rotation(bone),
                before.rotation(bone),
                "{} changed under an empty layer",
                bone.name(),
            );
        }
    }

    #[test]
    fn an_oscillator_only_touches_its_own_bone() {
        let layer = PhaseLayer {
            sway: None,
            walk_sway: None,
            oscillators: vec![(
                Bone::Head,
                PhaseOscillator {
                    clock: PhaseClock::Gait,
                    harmonic: 1.0,
                    amplitude: 0.3,
                    offset: 0.0,
                    axis: Vec3::Z,
                },
            )],
        };

        let mut pose = LocalPose::REST;
        layer.apply(&GaitPhase { gait: FRAC_PI_2, ..Default::default() }, &mut pose);

        assert!(
            !pose.rotation(Bone::Head).abs_diff_eq(Quat::IDENTITY, 1.0e-4),
            "the targeted bone should have moved",
        );
        for &bone in Bone::ALL.iter() {
            if bone == Bone::Head {
                continue;
            }
            assert_eq!(
                pose.rotation(bone),
                Quat::IDENTITY,
                "{} should be untouched",
                bone.name(),
            );
        }
    }

    #[test]
    fn the_layer_composes_onto_the_authored_pose_rather_than_replacing_it() {
        // Additive, not overriding: an idle sway must ride on top of
        // whatever pose the character is holding.
        let authored = Quat::from_axis_angle(Vec3::Y, 0.8);

        let mut pose = LocalPose::REST;
        pose.set_rotation(Bone::Neck, authored);

        PhaseLayer::standing_idle()
            .apply(&GaitPhase { gait: 1.0, ..Default::default() }, &mut pose);

        let result = pose.rotation(Bone::Neck);
        assert!(
            !result.abs_diff_eq(authored, 1.0e-4),
            "the oscillator should have contributed something",
        );
        assert!(
            result.angle_between(authored) < 0.1,
            "...but only a small amount on top of the authored 0.8 rad, got {} rad away",
            result.angle_between(authored),
        );
    }

    #[test]
    fn the_head_bob_completes_two_cycles_per_stride() {
        // The classic harmonic error: a head bob at stride rate looks like
        // a nod. It peaks at each footfall, so it runs at twice stride rate.
        let layer = PhaseLayer::locomotion();
        let (_, head_bob) = layer
            .oscillators
            .iter()
            .find(|(bone, _)| *bone == Bone::Neck)
            .expect("locomotion should drive the neck");

        assert_eq!(head_bob.harmonic, 2.0, "the head bob must run at twice stride rate");

        // Count sign changes over one stride: a 2x harmonic crosses zero
        // four times.
        let mut crossings = 0;
        let mut previous = head_bob.sample(&GaitPhase { gait: 0.0, ..Default::default() });
        for step in 1..=720 {
            let gait = TAU * step as f32 / 720.0;
            let current = head_bob.sample(&GaitPhase { gait, ..Default::default() });

            let previous_angle = previous.to_scaled_axis().dot(Vec3::X);
            let current_angle = current.to_scaled_axis().dot(Vec3::X);
            if previous_angle.signum() != current_angle.signum() {
                crossings += 1;
            }
            previous = current;
        }

        assert_eq!(
            crossings, 4,
            "a 2x harmonic should cross zero four times per stride, got {crossings}",
        );
    }

    /// The same pendulum solved without a Fourier series: the periodic
    /// finite-difference system `(x[i+1] − 2x[i] + x[i−1]) / h² = (x[i] − p[i]) / K`,
    /// by dense Gaussian elimination.
    fn pendulum_by_finite_differences(stride_seconds: f32, duty: f32, width: f32, k: f32) -> Vec<f64> {
        let n = 240usize;
        let h = stride_seconds as f64 / n as f64;
        let (a, double, k) = (width as f64 * 0.5, (duty - 0.5) as f64, k as f64);
        let pressure = |c: f64| {
            let c = c.rem_euclid(1.0);
            if c < double {
                -a + 2.0 * a * c / double
            } else if c < 0.5 {
                a
            } else if c < 0.5 + double {
                a - 2.0 * a * (c - 0.5) / double
            } else {
                -a
            }
        };
        let mut m = vec![vec![0.0f64; n + 1]; n];
        for i in 0..n {
            m[i][(i + n - 1) % n] += 1.0 / (h * h);
            m[i][(i + 1) % n] += 1.0 / (h * h);
            m[i][i] += -2.0 / (h * h) - 1.0 / k;
            m[i][n] = -pressure(i as f64 / n as f64) / k;
        }
        for col in 0..n {
            let pivot = (col..n).max_by(|&x, &y| m[x][col].abs().total_cmp(&m[y][col].abs())).unwrap();
            m.swap(col, pivot);
            for row in 0..n {
                if row != col {
                    let f = m[row][col] / m[col][col];
                    for j in col..=n {
                        m[row][j] -= f * m[col][j];
                    }
                }
            }
        }
        (0..n).map(|i| m[i][n] / m[i][i]).collect()
    }

    #[test]
    fn the_pendulum_height_ratio_matches_the_bodys_centre_of_mass() {
        use crate::character::anim::anthropometry::centre_of_mass;
        use crate::character::anim::rig::offset_from;
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let ankles = offset_from(&stood, &rig, Bone::Hips, Bone::LeftFoot).y;
        let ratio = (centre_of_mass(&stood, &rig).y - ankles) / -ankles;
        assert!((ratio - COM_OVER_HIPS).abs() < 0.01, "the COM stands {ratio:.4}x the hips' height above the ankles");
    }

    #[test]
    fn the_walking_sway_is_the_pendulums_periodic_path() {
        // Winter Eq. 11.3 with the pressure under each stance foot in turn.
        // `puppet_base`'s walk: 13 cm step width, K = 0.104 s², and its
        // strides at 0.7 / 1.2 / 1.6 m/s.
        for (seconds, duty, peak) in [(1.304, 0.641, 0.0227), (1.101, 0.622, 0.0178), (0.988, 0.608, 0.0151)] {
            let direct = pendulum_by_finite_differences(seconds, duty, 0.13, 0.104);
            let n = direct.len();
            let mut worst = 0.0f64;
            let mut furthest = (0.0f32, 0usize);
            for (i, &x) in direct.iter().enumerate() {
                let ours = walk_sway_at(i as f32 / n as f32, seconds, duty, 0.13, 0.104);
                worst = worst.max((ours as f64 - x).abs());
                if ours > furthest.0 {
                    furthest = (ours, i);
                }
            }
            assert!(worst < 5.0e-4, "at a {seconds} s stride the sway is {:.2} mm off the direct solution", worst * 1e3);
            assert!((furthest.0 - peak).abs() < 1.0e-3, "peak {:.4} m, expected ~{peak}", furthest.0);
            // Furthest toward the left foot in left single support, not at a
            // heel contact: the pendulum turns back before it gets there.
            let at = furthest.1 as f32 / n as f32;
            assert!((duty - 0.5..0.5).contains(&at), "the sway peaks at {at:.3} of the stride");
        }
    }

    #[test]
    fn the_pelvis_drops_on_the_swing_side_then_is_lifted_back() {
        // Winter §7.4.5: through left stance (0–0.5 of the stride here) the
        // right side swings, and the pelvis drops on it under H1-F — lowest
        // at ~17 % — then H2-F/H3-F lift it back past level by the next
        // heel contact. Positive lifts the left side.
        let samples: Vec<(f32, f32)> =
            (0..400).map(|i| i as f32 / 400.0).map(|c| (c, pelvic_obliquity_at(c))).collect();
        let (at, lowest_right) = samples
            .iter()
            .copied()
            .filter(|(c, _)| *c < 0.5)
            .max_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        assert!((0.15..0.21).contains(&at), "the swing side is lowest at {at:.3} of the stride");
        assert!(
            (0.059..0.074).contains(&lowest_right),
            "the swing side drops {:.2}°, Winter's derived 3.9°",
            lowest_right.to_degrees()
        );
        // Lifted back: by its own heel contact the swing side is still
        // 0.74° low in the integrated curve (it crosses level only at 56 %,
        // once the other foot has the weight); the fit is within its 0.37°.
        let at_contact = pelvic_obliquity_at(0.5).to_degrees();
        assert!(
            (at_contact - 0.74).abs() < 0.4,
            "at the right heel contact the pelvis rolls {at_contact:.2}°, the derivation 0.74°"
        );
        // The right leg's stance mirrors the left's.
        for (c, roll) in samples {
            assert!((pelvic_obliquity_at(c + 0.5) + roll).abs() < 1.0e-6, "not mirrored at {c}");
        }
    }

    #[test]
    fn the_standing_idle_respects_the_documented_amplitude_hierarchy() {
        // Weight shift > breathing > head. Getting this order wrong reads
        // as a twitch rather than a breath — the research's own finding.
        //
        // Compared by what each MOVES — how far it carries the head, each
        // alone at its peak, on the rig as rendered — because the weight
        // shift is a pelvis translation and the other two are rotations.
        use crate::character::anim::gltf_rig::puppet_base_as_rendered;
        use crate::character::anim::rig::offset_from;
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        let rig = puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let idle = PhaseLayer::standing_idle();
        let head_at = |pose: &LocalPose| pose.root_translation + offset_from(pose, &rig, Bone::Hips, Bone::Head);
        let moved = |layer: PhaseLayer, phase: GaitPhase| {
            let mut pose = stood;
            layer.apply_on(&phase, &mut pose, &rig);
            (head_at(&pose) - head_at(&stood)).length()
        };
        let only = |bone: Bone| PhaseLayer {
            sway: None,
            walk_sway: None,
            oscillators: idle.oscillators.iter().copied().filter(|(b, _)| *b == bone).collect(),
        };
        let peak = |gait: f32, breath: f32| GaitPhase { gait, breath, ..Default::default() };

        let sway = moved(PhaseLayer { sway: idle.sway, walk_sway: None, oscillators: vec![] },peak(FRAC_PI_2, 0.0));
        let breath = moved(only(Bone::Spine2), peak(0.0, FRAC_PI_2));
        let head = moved(only(Bone::Neck), peak(std::f32::consts::PI, 0.0));

        assert!(sway > breath, "weight shift ({sway} m) should exceed breathing ({breath} m)");
        assert!(breath > head, "breathing ({breath} m) should exceed head motion ({head} m)");
    }

    #[test]
    fn the_standing_idle_sways_the_body_over_planted_feet() {
        // Winter §11.2.1: standing sway is the pelvis carrying an upright
        // trunk over the feet — side to side by loading one leg and
        // unloading the other, front to back about the ankles — with the
        // side-to-side centre of pressure inside about +0.8/-1.2 cm and the
        // centre of mass moving less. The old idle bent the spine over a
        // fixed pelvis instead: shoulders 3.2 cm, centre of mass 1.3 cm side
        // to side and 0.16 cm front to back.
        use crate::character::anim::anthropometry::centre_of_mass;
        use crate::character::anim::gltf_rig::puppet_base_as_rendered;
        use crate::character::anim::rig::offset_from;
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        let rig = puppet_base_as_rendered();
        let stood = stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        let layer = PhaseLayer::standing_idle();
        let world = |pose: &LocalPose, bone| pose.root_translation + offset_from(pose, &rig, Bone::Hips, bone);

        let (mut side, mut ahead, mut feet, mut trunk_bend) = (vec![], vec![], 0.0f32, 0.0f32);
        for i in 0..96 {
            let t = i as f32 / 96.0;
            // Two unrelated clocks, as a standing character runs them.
            let phase = GaitPhase { gait: TAU * t, breath: TAU * 2.3 * t, ..Default::default() };
            let mut pose = stood;
            layer.apply_on(&phase, &mut pose, &rig);
            let com = pose.root_translation + centre_of_mass(&pose, &rig);
            side.push(com.dot(rig.left()));
            ahead.push(com.dot(rig.forward()));
            for ankle in [Bone::LeftFoot, Bone::RightFoot] {
                feet = feet.max((world(&pose, ankle) - world(&stood, ankle)).length());
            }
            // The trunk rides the pelvis: shoulders move with the hips.
            let carried = |p: &LocalPose| world(p, Bone::LeftArm) - world(p, Bone::LeftUpLeg);
            trunk_bend = trunk_bend.max((carried(&pose) - carried(&stood)).dot(rig.left()).abs());
        }
        let range = |v: &[f32]| v.iter().copied().fold(f32::MIN, f32::max) - v.iter().copied().fold(f32::MAX, f32::min);

        assert!((0.008..0.016).contains(&range(&side)), "side to side the centre of mass travels {} m", range(&side));
        assert!((0.004..0.012).contains(&range(&ahead)), "front to back it travels {} m", range(&ahead));
        assert!(feet < 5.0e-4, "the planted feet moved {feet} m");
        assert!(trunk_bend < 0.004, "the trunk bent {trunk_bend} m sideways over the pelvis");
    }

    #[test]
    fn the_standing_idle_shifts_its_weight_now_and_then_onto_either_leg() {
        // Occasional, held, both sides, and smooth: never a snap, never a
        // metronome. Sampled over ten minutes of a standing clock.
        let sway = PhaseLayer::standing_idle().sway.expect("the idle sways");
        let samples: Vec<f32> = (0..=6000).map(|i| sway.weight_shift(i as f32 * 0.1)).collect();

        assert_eq!(samples[0], 0.0, "a character spawns standing square");
        let left = samples.iter().filter(|&&s| s > 0.99).count();
        let right = samples.iter().filter(|&&s| s < -0.99).count();
        let square = samples.iter().filter(|&&s| s.abs() < 0.01).count();
        for (name, n) in [("left", left), ("right", right), ("square", square)] {
            assert!(n > samples.len() / 10, "{name} held for only {n} of {} samples", samples.len());
        }
        // Smooth: a 0.1 s step never moves it more than an eased 1.6 s
        // transition allows.
        let worst = samples.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f32::max);
        assert!(worst < 0.1, "the weight shift jumped {worst} in 0.1 s");
    }

    #[test]
    fn the_standing_idle_is_never_perfectly_mirrored() {
        // Mirrored motion is the single biggest tell of a robotic idle.
        let layer = PhaseLayer::standing_idle();
        let left = layer.oscillators.iter().find(|(b, _)| *b == Bone::LeftArm).unwrap().1;
        let right = layer.oscillators.iter().find(|(b, _)| *b == Bone::RightArm).unwrap().1;

        assert!(
            (left.amplitude - right.amplitude).abs() > 1.0e-3
                || (left.offset - right.offset).abs() > 1.0e-3,
            "the left and right arms must differ in amplitude or phase",
        );
    }

    #[test]
    fn the_layer_output_is_continuous_across_a_phase_wrap() {
        // A discontinuity at the wrap point would read as a visible hitch
        // once per cycle — the most obvious possible artefact.
        let layer = PhaseLayer::standing_idle();
        let dt = 1.0 / 240.0;

        let mut phase = GaitPhase { base_frequency_hz: 2.0, ..Default::default() };
        let mut previous: Option<LocalPose> = None;
        let mut largest_step = 0.0f32;

        for _ in 0..2000 {
            phase.advance(dt);

            let mut pose = LocalPose::REST;
            layer.apply(&phase, &mut pose);

            if let Some(previous) = previous {
                for &bone in Bone::ALL.iter() {
                    largest_step = largest_step
                        .max(previous.rotation(bone).angle_between(pose.rotation(bone)));
                }
            }
            previous = Some(pose);
        }

        assert!(
            largest_step < 0.01,
            "the layer should change smoothly frame to frame, but jumped {largest_step} \
             rad — check the phase wrap",
        );
    }

    #[test]
    fn a_non_positive_timestep_does_not_advance_the_clocks() {
        let mut phase = GaitPhase::default();
        let before = phase;

        phase.advance(0.0);
        phase.advance(-1.0);

        assert_eq!(phase, before);
    }

    #[test]
    fn the_gait_frequency_reports_what_the_clock_actually_uses() {
        let phase = GaitPhase { speed: 2.0, ..Default::default() };
        let expected = phase.base_frequency_hz + phase.speed_coefficient * 2.0;

        assert!((phase.gait_frequency_hz() - expected).abs() < 1.0e-6);
    }

    #[test]
    fn a_reversing_character_advances_the_phase_forward() {
        // Walking backwards still takes strides; a negative speed must not
        // run the gait clock in reverse.
        let dt = 1.0 / 120.0;

        let mut forward = GaitPhase { speed: 1.5, ..Default::default() };
        let mut backward = GaitPhase { speed: -1.5, ..Default::default() };

        advanced(&mut forward, 0.5, dt);
        advanced(&mut backward, 0.5, dt);

        assert!(
            (forward.gait - backward.gait).abs() < 1.0e-4,
            "travelling backwards should advance the stride clock just as fast",
        );
    }
}
