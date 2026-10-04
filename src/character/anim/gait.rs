//! The walk cycle: a phase-parameterised pose.
//!
//! # Why this is not a phase oscillator
//!
//! [`super::phase::PhaseOscillator`] already reads the same gait clock, and
//! adding two more of them — one per hip, one per knee — is a twenty-line
//! change that appears to work. It produces a specific, recognisable wrong
//! walk, and avoiding that is this module's entire reason for existing.
//!
//! An oscillator is one sinusoid about one axis:
//!
//! ```text
//! angle = amplitude * sin(harmonic * clock + offset)
//! ```
//!
//! A leg is not a sinusoid, in three separate ways:
//!
//! - **Time asymmetry.** Stance occupies about 60% of the cycle and swing
//!   40%. A sine splits the cycle evenly, so both legs sweep in equal,
//!   mirrored pendulum arcs.
//! - **The knee is not sinusoidal at all.** It stays nearly straight through
//!   stance, bearing weight, then flexes about 50 degrees during swing to
//!   get the foot off the ground. A sine bends it symmetrically on both
//!   halves, which reads as a limp.
//! - **The knee leads the thigh.** Peak knee flexion happens *before* peak
//!   thigh swing — the lower leg folds up and under, then extends forward
//!   into the footfall. This was recorded in the superseded module as "the
//!   single biggest fix" for its own once-live "doesn't look natural"
//!   finding.
//!
//! So the unit here is a **pose at a phase**, with its own piecewise curves,
//! rather than a waveform. The phase layer keeps doing what it is good at:
//! secondary motion — spine twist, breathing, head bob — layered on top of
//! whatever this produces.
//!
//! # Where the numbers come from
//!
//! The superseded `character::muscle` module shipped a tuned walk cycle, and
//! its constants carry the reasoning for their own values along with three
//! documented "doesn't look natural" regressions. They transfer directly,
//! because they are rotation angles and this stack is rotation-space
//! natively. Each is cited at its own constant below.
//!
//! # The stance leg is not a passive rod
//!
//! Its own recorded finding, and worth stating separately: an early version
//! of that module left the stance leg untouched during the other leg's
//! swing, and screenshots showed **no visible front/back leg split at all**.
//! A real stance leg trails behind as the body's weight passes over it. That
//! is [`GaitParams::stance_trail`], and without it a walk reads as a shuffle
//! however good the swing leg looks.

use bevy::math::{Quat, Vec3};

use super::rig::LocalPose;
use super::stance::KNEE_AXIS;
use crate::character::skeleton::Bone;

/// The shape of one walk cycle.
///
/// Angles are radians. The sign convention is [`KNEE_AXIS`]'s: about `+X`
/// (the character's right), **positive swings a bone backward and negative
/// swings it forward**.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GaitParams {
    /// How far one foot travels in a full cycle, metres — as *authored
    /// intent*.
    ///
    /// # It is not the distance the character actually covers
    ///
    /// This feeds [`foot_offset_z`], which describes where the gait means to
    /// put each foot. What the character really travels comes from where the
    /// rotations put the foot, which is not the same number: measured, the
    /// posed foot covers 0.61 m of hip-relative travel per stance against
    /// this 0.45, and [`super::locomotion`] carries the body 0.52 m per
    /// step as a result.
    ///
    /// The two are allowed to differ because the joint angles — not this —
    /// are what the pose is built from, and they are constrained by the
    /// leg's reach (see [`peak_reach_fraction`]). Raising this does not make
    /// a character walk faster; raising [`GaitParams::thigh_swing`] does.
    pub stride_length: f32,
    /// The fraction of the cycle each foot spends on the ground.
    ///
    /// About 0.6 for a walk. Above 0.5 the two stance windows overlap, which
    /// is the double-support phase that distinguishes a walk from a run; at
    /// or below 0.5 there is a moment with no foot down, which is a run.
    pub duty_factor: f32,
    /// Peak forward thigh angle during swing.
    ///
    /// ~35 degrees is a natural mid-swing thigh angle at a walking pace.
    pub thigh_swing: f32,
    /// Peak knee flexion during swing.
    ///
    /// The superseded module's own note: a real walking leg "doesn't swing
    /// as one rigid rod pivoting from the hip, the KNEE leads, bending
    /// noticeably so the lower leg folds up and under before extending
    /// forward again near foot-strike."
    ///
    /// Published walking kinematics put the swing peak at 60-65 degrees
    /// against a stance peak of 15-20 — a ratio near 3.6. The superseded
    /// module's ~50 degrees was toward the low end, and it paired with a
    /// stance knee that was too straight; both are corrected here.
    pub knee_swing_flex: f32,
    /// Knee flexion carried through stance.
    ///
    /// Small but never zero: a stance knee is not locked straight, and a
    /// leg at exactly full extension sits at the reach singularity where it
    /// has no bend direction and the IK has no solution space.
    ///
    /// It also has to clear [`MINIMUM_KNEE_FLEX`], and by more than intuition
    /// suggests. Headroom grows with the SQUARE of the angle at small
    /// flexions, so the difference between a nearly-straight leg and a
    /// usable one is stark: measured on real proportions, 0.10 rad leaves
    /// 1.1 mm of unused reach out of 0.89 m — 0.8% — while 0.30 rad leaves
    /// 10 mm. The first is inside the IK's own soft clamp; the second is a
    /// leg that can actually be solved.
    pub knee_stance_flex: f32,
    /// How far the stance leg trails behind as the body passes over it.
    ///
    /// ~16 degrees. Without it there is no visible front/back leg split.
    pub stance_trail: f32,
    /// Peak ankle articulation — toe-off behind, heel-strike ahead.
    pub ankle_range: f32,
    /// Half the upper arm's swing, radians. The arms counter-swing the
    /// legs, about a point behind the shoulder ([`ARM_SWING_CENTRE`]).
    pub arm_swing: f32,
    /// How far the elbow folds at the front of its swing, on top of the
    /// standing bend.
    pub elbow_bend: f32,
    /// The share of [`Self::elbow_bend`] the elbow keeps at the back of its
    /// swing. A relaxed arm is never a straight one, and a running arm stays
    /// bent all the way round.
    pub elbow_carry: f32,
    /// How far the hips drop at midstance, **as a fraction of leg length**.
    ///
    /// A real gait mechanic, and also what buys the stride its horizontal
    /// reach: dropping the hip trades vertical reach for horizontal. The
    /// rig stands at 100% leg extension, where the horizontal budget is
    /// exactly zero, so without this no stride is reachable at all.
    ///
    /// # Why a fraction and not metres
    ///
    /// This was authored in metres against the synthetic rig, whose leg is
    /// `0.49 m`. The real rig's leg is `0.888 m` — nearly double — so the
    /// same absolute drop is a completely different proportion of the leg,
    /// and the gait's vertical motion went out of scale on any rig but the
    /// one it was tuned on.
    ///
    /// Every other quantity in this struct is already rig-independent: the
    /// angles are angles, and [`peak_reach_fraction`] deliberately reasons
    /// in fractions of leg length for exactly this reason. These two were
    /// the outliers.
    ///
    /// Resolve with [`GaitParams::hip_dip_metres`], never by reading the
    /// field directly.
    pub hip_dip: f32,
    /// Vertical bob amplitude, **as a fraction of leg length**. Peaks twice
    /// per cycle — once per footfall.
    ///
    /// See [`GaitParams::hip_dip`] for why this is a fraction.
    pub vertical_bob: f32,
    /// Where the legs' joint angles come from.
    pub curves: LegCurves,
}

/// Where a gait's hip, knee and ankle angles come from.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LegCurves {
    /// A measured human stride — Winter's Appendix A walk, see
    /// [`super::reference`] — with every excursion scaled by `amplitude`
    /// (1.0 is the recorded stride; the hip scales by it, the knee and ankle
    /// by its square root). The thigh, knee, stance and ankle fields are
    /// unused; the stance leg's trail, the knee's loading dip, the push-off
    /// and the heel strike all come from the data.
    Measured { amplitude: f32 },
    /// The hand-shaped piecewise curves this module's fields describe. What
    /// a run uses, because the reference is a walk and there is no measured
    /// run to draw from.
    Authored,
}

/// The leg length the gait's fractional amplitudes were authored against.
///
/// This crate's synthetic T-pose: femur `0.42` plus shin `0.07`. Dividing
/// the original metre values by this is what converts them into fractions
/// without changing a single number on the rig they were tuned on — which
/// is what keeps every existing gait test measuring the same motion.
///
/// Read from [`Bone::t_pose_offset`] rather than transcribed, so it cannot
/// drift if the synthetic rig is ever re-proportioned. Note which offsets
/// these are: a bone's offset is measured from its PARENT, so the femur is
/// `offsets[LeftLeg]` and the shin is `offsets[LeftFoot]` —
/// `offsets[LeftUpLeg]` is the hips-to-socket step and not part of the leg
/// at all. That off-by-one is the LeftUpLeg-is-the-knee trap this rig's
/// naming sets.
pub fn authored_leg_length() -> f32 {
    Bone::LeftLeg.t_pose_offset().length() + Bone::LeftFoot.t_pose_offset().length()
}

/// The leg length of a specific rig, for scaling the gait's fractional
/// amplitudes onto it.
///
/// Same bone pair as [`authored_leg_length`], read from the rig's own
/// offsets.
pub fn leg_length_of(rig: &super::rig::RigGeometry) -> f32 {
    rig.offsets[Bone::LeftLeg].length() + rig.offsets[Bone::LeftFoot].length()
}

impl GaitParams {
    /// The hip dip in metres, on a specific rig.
    ///
    /// The only correct way to read [`GaitParams::hip_dip`] — the field is
    /// a fraction of leg length and means nothing without a rig.
    pub fn hip_dip_metres(&self, rig: &super::rig::RigGeometry) -> f32 {
        self.hip_dip * leg_length_of(rig)
    }

    /// The vertical bob in metres, on a specific rig. See
    /// [`GaitParams::hip_dip_metres`].
    pub fn vertical_bob_metres(&self, rig: &super::rig::RigGeometry) -> f32 {
        self.vertical_bob * leg_length_of(rig)
    }
}

impl GaitParams {
    /// A run.
    ///
    /// The structural difference from a walk is the **duty factor**: below
    /// 0.5 the two stance windows no longer overlap, so there is a moment
    /// with no foot on the ground at all. That flight phase is what
    /// separates a run from a walk, and it is not a matter of speed —
    /// a fast walk is still a walk.
    ///
    /// Published running kinematics put the stance knee at 25-40 degrees
    /// (against 15-20 walking) and the swing peak past 90, with a longer
    /// stride and a higher foot lift. The angles here are the gait's own
    /// contribution, so the base stance's 0.16 rad adds on top.
    ///
    /// Reach is the binding constraint and it is checked, not assumed:
    /// `the_run_keeps_the_leg_clear_of_its_singularity` measures
    /// `peak_reach_fraction` for these parameters the same way the walk's
    /// own test does.
    pub fn running() -> Self {
        Self {
            // Longer than a walk's, and affordable because the deeper knee
            // flexion below buys back the reach it costs.
            stride_length: 0.75,
            // The flight phase. Below 0.5 by a real margin rather than a
            // hair, so it is unambiguous rather than a rounding artefact.
            duty_factor: 0.4,
            thigh_swing: 0.85,
            knee_swing_flex: 1.35,
            // Deeper than a walk's 0.20: a running stance knee absorbs
            // landing, and it is what keeps the longer stride reachable.
            knee_stance_flex: 0.35,
            stance_trail: 0.40,
            ankle_range: 0.35,
            // A run swings harder.
            arm_swing: 0.55,
            elbow_bend: 1.20,
            // 41-69 degrees on top of the standing bend: a running arm stays
            // folded.
            elbow_carry: 0.6,
            // Fractions of leg length. `0.09 m` and `0.05 m` on the
            // synthetic rig's `0.49 m` leg, expressed so they scale — see
            // [`GaitParams::hip_dip`].
            hip_dip: 0.09 / 0.49,
            vertical_bob: 0.05 / 0.49,
            curves: LegCurves::Authored,
        }
    }

    /// The walk this module's hand-shaped curves describe, from before the
    /// walk was driven by measured data. Kept as the A/B baseline, and for
    /// the tests that pin those curves' own properties.
    pub fn authored_walk() -> Self {
        Self { curves: LegCurves::Authored, duty_factor: 0.6, ..Self::default() }
    }

    /// A walk at `speed` m/s for a body with legs `leg_length` long (thigh
    /// plus shin), driven by the measured stride.
    ///
    /// # Froude scaling
    ///
    /// Geometrically similar walkers move alike at the same `v² / (g·L)`, so
    /// the recorded stride is exactly right at
    /// `reference::SPEED · sqrt(L / reference::LEG_LENGTH)` — 1.57 m/s for
    /// `puppet_base`'s 0.888 m leg — and its excursions scale away from that
    /// as `(speed / that)^0.65`, the growth of a real stride with speed. The
    /// cadence is then whatever carries the stride at `speed`; see
    /// [`super::locomotion::distance_per_cycle`].
    pub fn walking_for(speed: f32, leg_length: f32) -> Self {
        Self::walking_with_steps(speed, leg_length, STRIDE_AMPLITUDE.0)
    }

    /// [`Self::walking_for`], its stride shortening with speed down to
    /// `shortest` of the recorded one's excursions, not half: a walk
    /// placing itself ([`SHORT_STEPS`]).
    pub fn walking_with_steps(speed: f32, leg_length: f32, shortest: f32) -> Self {
        let amplitude = (speed.max(0.0) / froude_speed(leg_length)).powf(STRIDE_GROWTH).clamp(shortest, STRIDE_AMPLITUDE.1);
        Self {
            // A slower walker spends longer with both feet down: stance
            // grows from the recording's 61% toward ~65% of the stride at
            // half its excursion (Winter records one speed; the trend is the
            // gait literature's). Replayed at 61% at every speed, a slow
            // walk squeezed the handover between feet into a lurch — the
            // body's speed spiked to 1.8x its mean at each heel strike.
            duty_factor: (super::reference::STANCE_FRACTION + 0.08 * (1.0 - amplitude)).clamp(0.55, 0.66),
            curves: LegCurves::Measured { amplitude },
            arm_swing: Self::default().arm_swing * stride_scale_for(speed),
            ..Self::default()
        }
    }

    /// [`Self::walking_for`] on a specific rig.
    pub fn walking_on(speed: f32, rig: &super::rig::RigGeometry) -> Self {
        Self::walking_for(speed, leg_length_of(rig))
    }
}

impl GaitParams {
    /// A walk suited to `speed` m/s.
    ///
    /// # Longer strides and quicker steps, not just quicker steps
    ///
    /// A real walker speeds up by lengthening their stride AND quickening
    /// their cadence — cadence roughly as `speed^0.35` and stride as
    /// `speed^0.65`. Scaling only the cadence, as every speed used to, made
    /// a slow walk slow motion and a fast one a scurry.
    ///
    /// So the stride-producing amplitudes scale with speed — the hip's swing
    /// in proportion, the knee's fold and the ankle's roll more gently — and
    /// the caller derives the cadence from the stride the result really
    /// takes: see [`super::locomotion::distance_per_cycle`].
    ///
    /// For a body proportioned like the reference subject; anything driving
    /// a real rig wants [`Self::walking_on`], which scales by that rig's leg.
    pub fn walking_at(speed: f32) -> Self {
        Self::walking_for(speed, super::reference::LEG_LENGTH)
    }

    /// [`Self::walking_at`] with the hand-shaped curves: the pre-reference
    /// walk, scaled the way it always was.
    pub fn authored_walking_at(speed: f32) -> Self {
        let walk = Self::authored_walk();
        let k = stride_scale_for(speed);
        let gentle = k.sqrt();
        Self {
            stride_length: walk.stride_length * k,
            thigh_swing: walk.thigh_swing * k,
            stance_trail: walk.stance_trail * k,
            arm_swing: walk.arm_swing * k,
            knee_swing_flex: walk.knee_swing_flex * gentle,
            ankle_range: walk.ankle_range * gentle,
            ..walk
        }
    }
}

/// A stride grows as speed to this power (`GaitParams::walking_for`).
pub const STRIDE_GROWTH: f32 = 0.65;
/// The measured stride's excursions scale between these: below, a walk is a
/// shuffle; above, the leg runs out of reach.
const STRIDE_AMPLITUDE: (f32, f32) = (0.5, 1.3);
/// How short a walk placing itself steps, of the recorded stride's
/// excursions (`GaitParams::walking_with_steps`): the short steps of
/// someone closing on a chair and turning to sit, a 0.23 m step on
/// `puppet_base`. At half, the walk's 0.39 m steps could not land a stop on
/// its spot, nor follow a turn tighter than 0.25 m.
pub const SHORT_STEPS: f32 = 0.3;

/// The speed the recorded stride is exactly right at for legs `leg_length`
/// long, m/s (`GaitParams::walking_for`).
fn froude_speed(leg_length: f32) -> f32 {
    super::reference::SPEED * (leg_length.max(0.1) / super::reference::LEG_LENGTH).sqrt()
}

/// The speeds, m/s, between which a walk on legs `leg_length` long grows its
/// stride with speed; slower or faster, only its cadence changes. On
/// `puppet_base`, 0.54-2.1 m/s: a walk to a chair paced at 0.3 m/s took the
/// stride of 0.54, and a stop timed by `speed^0.65` overshot 20 cm.
pub fn stride_speeds(leg_length: f32) -> (f32, f32) {
    stride_speeds_with_steps(leg_length, STRIDE_AMPLITUDE.0)
}

/// [`stride_speeds`] for a walk shortening its stride down to `shortest`
/// (`GaitParams::walking_with_steps`).
pub fn stride_speeds_with_steps(leg_length: f32, shortest: f32) -> (f32, f32) {
    let at = |amplitude: f32| froude_speed(leg_length) * amplitude.powf(1.0 / STRIDE_GROWTH);
    (at(shortest), at(STRIDE_AMPLITUDE.1))
}

/// The speed the default walk is authored for, m/s.
pub const REFERENCE_WALK_SPEED: f32 = 1.0;

/// How much a walk at `speed` scales the default's stride.
///
/// `speed^0.65` — a real walker's stride growth — clamped: below half the
/// stride a walk is a shuffle, and above 1.4x the leg runs out of reach and
/// the gait should be a run instead.
pub fn stride_scale_for(speed: f32) -> f32 {
    (speed.max(0.0) / REFERENCE_WALK_SPEED).powf(0.65).clamp(0.5, 1.4)
}

impl Default for GaitParams {
    /// A normal walking pace.
    fn default() -> Self {
        Self {
            // Derived in `stride_for_hip_dip`, not chosen. This is the value
            // that function returns for the default dip on this rig; the
            // test `the_default_stride_is_the_one_the_reach_budget_allows`
            // pins the two together.
            stride_length: 0.45,
            // The recording's own stance share (61%), so the measured curves
            // play back without retiming.
            duty_factor: super::reference::STANCE_FRACTION,
            thigh_swing: 0.61,
            // ~57 degrees once the base stance's 0.16 rad is added, inside
            // the real 60-65 degree band. Authored as the gait's own
            // CONTRIBUTION, like every other angle here.
            knee_swing_flex: 0.90,
            // Enough that the stance knee stays clear of the reach
            // singularity even at its straightest — which is at footfall and
            // again at toe-off, exactly when the leg reaches furthest.
            //
            // This composes ON TOP of the base pose's own bend
            // (`stance::DEFAULT_KNEE_FLEX`, 0.16 rad), so the leg is at
            // 0.36 rad there, not 0.20. See the field's own note for why
            // 0.10 was not enough on its own.
            knee_stance_flex: 0.20,
            stance_trail: 0.28,
            ankle_range: 0.25,
            // Murray, Sepic & Barnard (1967), 30 men at a free pace
            // (1.54 m/s): the upper arm swings from 8 degrees in front of
            // the vertical to 24 behind it, a 32-degree excursion. Scaled by
            // `stride_scale_for` (x1.32 at 1.54 m/s), 0.21 rad gives them;
            // `ARM_SWING_CENTRE` puts the middle behind the shoulder.
            arm_swing: 0.21,
            // Murray's elbow: 17 degrees at the back of the swing, 47 at the
            // front. On top of the standing pose's ~5 degrees, 12 to 42.
            elbow_bend: 0.73,
            elbow_carry: 0.29,
            // Fractions of leg length. `0.06 m` and `0.025 m` on the
            // synthetic rig's `0.49 m` leg, expressed so they scale — see
            // [`GaitParams::hip_dip`].
            hip_dip: 0.06 / 0.49,
            vertical_bob: 0.025 / 0.49,
            // The recorded human stride. Its stance share is the recording's
            // own, so the curves play back untimed.
            curves: LegCurves::Measured { amplitude: 1.0 },
        }
    }
}

/// Which half of the cycle a leg is in, and how far through it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LegPhase {
    /// On the ground, bearing weight. `progress` runs 0 (footfall) to 1
    /// (toe-off).
    Stance { progress: f32 },
    /// In the air, swinging forward. `progress` runs 0 (toe-off) to 1
    /// (footfall).
    Swing { progress: f32 },
}

impl LegPhase {
    /// Whether this foot is on the ground.
    pub fn is_stance(self) -> bool {
        matches!(self, LegPhase::Stance { .. })
    }
}

/// Splits a cycle phase into stance and swing for one leg.
///
/// `phase` wraps to `[0, 1)`. Stance comes first: phase 0 is footfall, which
/// makes the contact windows easy to reason about and puts the
/// discontinuity-prone moment at a known place.
pub fn leg_phase(phase: f32, duty_factor: f32) -> LegPhase {
    let phase = wrap_phase(phase);
    // `clamp` PANICS on a NaN bound and propagates a NaN value, so neither
    // end of this can use it directly.
    let duty = if duty_factor.is_finite() {
        duty_factor.clamp(0.01, 0.99)
    } else {
        0.6
    };

    if phase < duty {
        LegPhase::Stance { progress: phase / duty }
    } else {
        LegPhase::Swing { progress: (phase - duty) / (1.0 - duty) }
    }
}

/// Smoothstep: C1 at both ends, so curves built from it have no velocity
/// discontinuity where they meet.
///
/// That continuity is not cosmetic. A position-continuous but
/// velocity-discontinuous loop produces a visible hitch once per stride, and
/// it is exactly the artefact the spring stage downstream cannot hide,
/// because the spring is chasing a target that jumped.
pub(crate) fn smoothstep(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// The least a knee may bend, radians.
///
/// A leg at exactly full extension sits at the IK's reach singularity: the
/// knee has no bend direction there, so the next frame's solve can flip it
/// either way (the knee-popping artefact) and any numerical error pushes the
/// chain past straight. It also reads as a stiff, stilted walk.
///
/// The same reservation [`super::legik`] already makes — its soft extension
/// clamp holds a chain a few millimetres short of full reach, leaving about
/// this much bend, pinned by
/// `a_chain_reaching_its_limit_keeps_a_small_safety_bend`.
///
/// Expressed as an ANGLE rather than as metres of unused reach, so it means
/// the same thing on a child rig and a giant one.
///
/// Set at 0.25 rad (14 degrees) rather than at the bare minimum the IK needs,
/// for two reasons. Published walking kinematics put the stance knee at
/// 15-20 degrees through its straightest moments, so this is the anatomical
/// floor rather than merely a numerical one. And a lower bar is not
/// discriminating: at 0.12 rad the base stance's own 0.16 rad clears it
/// unaided, so a gait contributing *nothing* would pass — verified by
/// setting `knee_stance_flex` to zero and watching this test stay green
/// while four others failed.
pub const MINIMUM_KNEE_FLEX: f32 = 0.25;

/// How far a leg has to reach, at the worst point in the cycle, as a
/// fraction of its own straight length.
///
/// Returns the peak over a full cycle of `hip-to-ankle distance / (thigh +
/// shin)`. At `1.0` the leg is dead straight and the IK has no solution
/// space; the useful range is comfortably below that.
///
/// The measurement that matters is this ratio rather than the stride,
/// because a stride is only unreachable *relative to* the leg carrying it.
///
/// `base_knee_flex` is whatever bend the pose the gait composes onto already
/// has — [`super::stance::DEFAULT_KNEE_FLEX`] for the standard stance. It is
/// a parameter rather than an assumption because ignoring it makes this
/// disagree with the rig as actually posed: measured, the gait alone
/// predicted 99.5% of straight where forward kinematics on the composed pose
/// measured 98.4%, and the difference is exactly the stance's own 0.16 rad.
pub fn peak_reach_fraction(
    params: &GaitParams,
    base_knee_flex: f32,
    thigh: f32,
    shin: f32,
) -> f32 {
    let straight = thigh + shin;
    if straight <= 1.0e-6 {
        return 0.0;
    }

    let mut worst = 0.0f32;

    for i in 0..SAMPLES_PER_CYCLE {
        let phase = i as f32 / SAMPLES_PER_CYCLE as f32;
        let flex = base_knee_flex + knee_flex(leg_phase(phase, params.duty_factor), params);

        // Law of cosines across the knee: the interior angle is `pi - flex`,
        // so a straighter knee reaches further.
        let interior = std::f32::consts::PI - flex;
        let reach =
            (thigh * thigh + shin * shin - 2.0 * thigh * shin * interior.cos()).sqrt();

        worst = worst.max(reach / straight);
    }

    worst
}

/// How many samples the cycle-wide solves take.
///
/// The curves are piecewise with Hermite segments, so these quantities are
/// measured rather than solved — see [`thigh_cycle_mean`].
const SAMPLES_PER_CYCLE: usize = 64;

/// The cycle position, in `[0, 1)`, for a [`super::phase::GaitPhase`].
///
/// The one correct way to bridge the phase clock's radians onto
/// [`walk_pose`]'s cycle fraction. Exists because the two units are easy to
/// confuse and the failure is silent — see [`walk_pose`]'s own note.
pub fn cycle_of(phase: &super::phase::GaitPhase) -> f32 {
    phase.gait / std::f32::consts::TAU
}

/// Wraps a cycle position into `[0, 1)`, safely.
///
/// `rem_euclid` alone is not enough: it returns NaN for a NaN input and for
/// an infinite one, and `f32::MAX` multiplied by TAU downstream overflows to
/// infinity — which then produces a non-finite rotation and poisons the
/// whole pose. A phase is a position on a circle, so there is always a
/// sensible answer; this returns one rather than propagating the garbage.
fn wrap_phase(phase: f32) -> f32 {
    if !phase.is_finite() {
        return 0.0;
    }

    let wrapped = phase.rem_euclid(1.0);
    if wrapped.is_finite() { wrapped } else { 0.0 }
}

/// Cubic Hermite: goes from `from` to `to` over `t` in `[0, 1]`, leaving at
/// rate `start_slope` and arriving at rate `end_slope`.
///
/// Used where a curve has to meet its neighbour in *velocity* as well as
/// position. A smoothstep always arrives flat, which is right when the next
/// segment also starts flat and a once-per-stride hitch otherwise.
pub(crate) fn hermite(from: f32, to: f32, start_slope: f32, end_slope: f32, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    let t2 = t * t;
    let t3 = t2 * t;

    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;

    h00 * from + h10 * start_slope + h01 * to + h11 * end_slope
}

/// A smooth pulse: 0 at both ends, 1 in the middle.
///
/// Built from two smoothsteps so it is C1 everywhere including its own peak,
/// unlike `sin(pi * t)` which is fine here but would not compose as cleanly
/// with the asymmetric timings below.
pub(crate) fn pulse(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    if t < 0.5 {
        smoothstep(t * 2.0)
    } else {
        smoothstep((1.0 - t) * 2.0)
    }
}

/// The thigh's angle at a point in the cycle.
///
/// Forward (negative, by [`KNEE_AXIS`]'s convention) at footfall, sweeping
/// backward through stance as the body passes over the foot, then forward
/// again through swing.
///
/// The two halves are **not** time-mirrors of each other: stance is the
/// longer one and travels at a near-constant rate (the body moving over a
/// planted foot), while swing is shorter and accelerates. That asymmetry is
/// the whole reason this is not a sine.
#[cfg(test)]
fn thigh_angle(leg: LegPhase, params: &GaitParams) -> f32 {
    thigh_angle_centred(leg, params, thigh_cycle_mean(params))
}

/// [`thigh_angle`] with the cycle mean already computed.
///
/// That mean is a 256-sample solve that depends on the parameters alone, and
/// recomputing it inside every `thigh_angle` call was most of the gait's
/// per-frame cost — a pose evaluates the thigh several times, and root
/// motion evaluates several poses. [`walk_pose_on`] computes it once.
fn thigh_angle_centred(leg: LegPhase, params: &GaitParams, mean: f32) -> f32 {
    // Centred on the cycle, so the body stays balanced over its feet.
    //
    // The stance and swing curves are shaped independently — different
    // durations, different easing — and there is no reason their combined
    // mean should land at zero. It does not: the raw curves leave the foot
    // pattern sitting 0.228 m behind the hips on a real rig's proportions
    // (0.065 m on the synthetic one, which is why a test tuned against the
    // synthetic rig passed it, and why this was only caught by measuring the
    // live rig over BRP). A character whose feet trail its body by half a
    // stride reads as falling forward, not walking.
    //
    // Subtracting the mean is the fix that cannot drift: it holds whatever
    // the two halves' shapes or durations are, rather than requiring the
    // amplitudes to be hand-balanced against each other and re-balanced
    // every time one is touched.
    thigh_angle_raw(leg, params) - mean
}

/// The thigh offset that centres the FOOT under the body over a cycle.
///
/// Centring the thigh angle's own mean is the obvious approach and it is not
/// enough: the knee also moves the foot, folding the shin rearward whenever
/// it flexes, and it flexes far more in swing than in stance. So the foot
/// keeps a rearward bias even with a perfectly balanced thigh curve —
/// measured 0.108 m on real proportions, down from 0.228 m but still a
/// quarter of a stride.
///
/// This instead measures where the foot actually ends up and solves for the
/// thigh offset that zeroes it. The relationship is linear to first order
/// (the foot swings on an arc about the hip, and these are small angles), so
/// one Newton step from two samples lands within a millimetre.
///
/// Computed by sampling rather than in closed form: the curves are piecewise
/// with Hermite segments and a closed form would need rederiving every time
/// a shape changed — exactly the coupling that goes stale silently.
fn thigh_cycle_mean(params: &GaitParams) -> f32 {
    const SAMPLES: usize = 64;

    // Where the feet sit relative to the hips, for a given thigh offset.
    let foot_bias = |offset: f32| -> f32 {
        let mut total = 0.0;

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;

            // Both legs, half a cycle apart — the same pairing `walk_pose`
            // uses.
            for cycle in [phase, phase + 0.5] {
                let leg = leg_phase(cycle, params.duty_factor);

                let thigh = thigh_angle_raw(leg, params) - offset;
                let flex = knee_flex(leg, params);

                // Forward kinematics for one leg in the sagittal plane,
                // using nominal proportions. The absolute lengths do not
                // matter to the ROOT of this — only that the knee's
                // contribution is weighted like a real leg's.
                // Matching `walk_pose`'s own composition: the hip is applied
                // as `-thigh` and the knee as `-flex`, so the shin's total
                // is `-(thigh + flex)`.
                let knee_z = NOMINAL_THIGH * thigh.sin();
                let foot_z = knee_z + NOMINAL_SHIN * (thigh + flex).sin();

                total += foot_z;
            }
        }

        total / (SAMPLES as f32 * 2.0)
    };

    // Two samples give the local slope; one Newton step from there.
    let at_zero = foot_bias(0.0);
    let at_one = foot_bias(0.1);

    let slope = (at_one - at_zero) / 0.1;
    if slope.abs() < 1.0e-6 {
        return 0.0;
    }

    -at_zero / slope
}

/// Nominal leg proportions for the centring solve, metres.
///
/// Only their RATIO matters — the solve finds a thigh angle, not a distance —
/// and it is what weights the knee's contribution against the hip's. Taken
/// from `puppet_base.gltf`'s real femur and shin rather than the synthetic
/// rig's, whose 0.07 m lower segment would weight the knee at almost nothing.
const NOMINAL_THIGH: f32 = 0.43;
const NOMINAL_SHIN: f32 = 0.46;

/// The thigh curve before centring. See [`thigh_angle`].
fn thigh_angle_raw(leg: LegPhase, params: &GaitParams) -> f32 {
    // The thigh angle at footfall, and it is NOT the swing peak.
    //
    // Conflating the two is a real error this module shipped once: it put
    // the forward peak at the end of swing, so a leg early in stance was
    // still strongly forward, and at the moment the other leg was mid-swing
    // both thighs pointed forward within 17 degrees of each other —
    // measured 0.036 m of foot separation, against the 0.15 m that reads as
    // a stride. That is exactly the "no visible front/back leg split"
    // regression the superseded module recorded.
    //
    // Anatomically the thigh peaks near MID-swing and then holds while the
    // knee extends into the footfall, so the leg arrives with the foot
    // reaching ahead but the thigh already easing back.
    let strike = params.thigh_swing * FOOTFALL_THIGH_FRACTION;

    match leg {
        // Stance: from the footfall angle to full trail, near-linear. The
        // foot is planted and the body travels over it at roughly constant
        // speed, so easing here would read as the body hesitating over each
        // foot.
        LegPhase::Stance { progress } => {
            let t = progress.clamp(0.0, 1.0);
            -strike + (strike + params.stance_trail) * t
        }
        // Swing: back to the forward peak at mid-swing, then easing to the
        // footfall angle.
        LegPhase::Swing { progress } => {
            let t = progress.clamp(0.0, 1.0);

            if t < SWING_THIGH_PEAK {
                // Trailing behind, up to the forward peak.
                let u = smoothstep(t / SWING_THIGH_PEAK);
                params.stance_trail - (params.stance_trail + params.thigh_swing) * u
            } else {
                // Easing back from the peak to where the foot lands — and
                // ARRIVING AT THE RATE STANCE LEAVES AT.
                //
                // A smoothstep here is flat at its end (its derivative at
                // u = 1 is zero) while stance departs at a constant
                // `(strike + trail)` per unit progress. The positions match
                // at the seam either way, so the join looks correct in a
                // still frame — but the velocity jumps from 0 to 0.707
                // rad/unit at every footfall, which is a hitch once per
                // step. Measured before this fix: 0 against 6.9 rad/s.
                //
                // A cubic Hermite with a specified end-slope lands at the
                // right value *and* the right rate.
                let span = 1.0 - SWING_THIGH_PEAK;
                let u = ((t - SWING_THIGH_PEAK) / span).clamp(0.0, 1.0);

                // Stance's rate, converted from per-stance-progress into
                // per-swing-progress via the two halves' relative durations.
                let stance_rate = strike + params.stance_trail;
                let duty = params.duty_factor.clamp(0.01, 0.99);
                let end_slope = stance_rate * (1.0 - duty) / duty * span;

                hermite(-params.thigh_swing, -strike, 0.0, end_slope, u)
            }
        }
    }
}

/// How much extra the stance knee yields under load at midstance, as a
/// fraction of [`GaitParams::knee_stance_flex`].
///
/// The knee bends a little as weight comes onto it and straightens as the
/// body passes over — a real mechanic, and small.
const STANCE_KNEE_YIELD: f32 = 0.6;

/// Where in swing the thigh reaches its forward peak.
///
/// Before the footfall, so the thigh holds and the knee extends into the
/// landing rather than the whole leg still swinging forward as it lands.
const SWING_THIGH_PEAK: f32 = 0.6;

/// The thigh's forward angle at footfall, as a fraction of its swing peak.
///
/// The foot lands ahead of the body, but the thigh is already easing back
/// from its peak by then — the reach at footfall comes from the knee
/// extending, not from more hip flexion.
const FOOTFALL_THIGH_FRACTION: f32 = 0.7;

/// The knee's flexion at a point in the cycle. Always non-negative — a knee
/// does not hyperextend.
///
/// Near-straight through stance, with one clear peak early in swing. "Early"
/// is the load-bearing detail: peaking at mid-swing or later would mean the
/// thigh has already swung forward before the lower leg folds, which drags
/// the toe through the ground.
fn knee_flex(leg: LegPhase, params: &GaitParams) -> f32 {
    match leg {
        LegPhase::Stance { progress } => {
            // A small flexion wave just after footfall — the knee yields
            // under load, then straightens as the body passes over. Nowhere
            // near zero, because a locked knee is both the reach singularity
            // and visibly wrong.
            //
            // Starts and ends at exactly `knee_stance_flex`, which is where
            // swing leaves off. An earlier version ran
            // `stance_flex * (0.5 + 0.5 * pulse(t))`, which begins at HALF
            // that — a 0.05 rad jump in the knee at every single footfall.
            // It went unnoticed because the continuity test that should
            // have caught it was sampling below `Quat::angle_between`'s f32
            // precision floor and reading quantisation noise.
            let t = progress.clamp(0.0, 1.0);
            params.knee_stance_flex * (1.0 + STANCE_KNEE_YIELD * pulse(t))
        }
        LegPhase::Swing { progress } => {
            // Peaks at 35% through swing: the knee leads, folding the lower
            // leg up and under before the thigh reaches its own forward
            // peak. See this module's doc comment.
            const KNEE_PEAK: f32 = 0.35;

            let t = progress.clamp(0.0, 1.0);
            let shaped = if t < KNEE_PEAK {
                smoothstep(t / KNEE_PEAK)
            } else {
                // Lands flat, matching the stance knee it hands over to.
                //
                // A smoothstep of the REVERSED parameter is flat at both
                // ends, so it arrives at zero slope — but the reversal also
                // means its argument runs 1 -> 0 as `t` runs peak -> 1, and
                // the chain rule picks up the `-1/(1-KNEE_PEAK)` factor.
                // Over the shorter swing half that works out to ~0.96 rad/s
                // of residual extension still running at footfall, against a
                // stance knee that is flat there.
                //
                // Written as an explicit Hermite instead: value 1 at the
                // peak, 0 at the seam, and zero slope AT THE SEAM stated
                // directly rather than inherited from a reversed curve.
                let u = ((t - KNEE_PEAK) / (1.0 - KNEE_PEAK)).clamp(0.0, 1.0);
                1.0 - smoothstep(u)
            };

            // Never drops below the stance flexion, so the two halves meet
            // continuously at both seams.
            params.knee_stance_flex + (params.knee_swing_flex - params.knee_stance_flex) * shaped
        }
    }
}

/// The ankle's angle: plantarflexed at toe-off, dorsiflexed for heel-strike.
///
/// # The sign is opposite to the leg's, because the bone points elsewhere
///
/// Every other angle here rotates a bone that hangs DOWN (`-Y`), where a
/// positive rotation about `KNEE_AXIS` swings it forward. The foot instead
/// points FORWARD (`-Z`), and for that bone:
///
/// ```text
/// Rx(t) * (0, 0, -L) = (0, L*sin t, -L*cos t)
/// ```
///
/// so a positive `t` lifts the toe and a negative one drives it down. The
/// same axis, the opposite meaning — because the thing being rotated is
/// perpendicular to the leg.
///
/// Missed once: this curve was written with the leg's convention, so the toe
/// pitched DOWN at footfall instead of up, and the toe tip ended up 0.02 m
/// below the ground while the ankle rode 0.12 m high. Measured on the live
/// rig, and reported as "toes look downward".
///
/// So positive is TOE UP here, and the curve is negated at the end.
fn ankle_angle(leg: LegPhase, params: &GaitParams) -> f32 {
    match leg {
        LegPhase::Stance { progress } => {
            // Rolls from heel-strike through flat to toe-off.
            let t = progress.clamp(0.0, 1.0);
            params.ankle_range * (2.0 * t - 1.0)
        }
        LegPhase::Swing { progress } => {
            // Recovers from toe-off back to a heel-strike attitude — and
            // ARRIVES AT THE RATE STANCE LEAVES AT, like the thigh.
            //
            // `ankle_range * (1 - 2*smoothstep(t))` matches in position at
            // the seam and arrives flat, while stance departs at
            // `2 * ankle_range` per unit progress. Same class of kink the
            // thigh had, and equally invisible in a still frame: measured
            // 1.505 vs 0.345 rad/s across the join.
            let t = progress.clamp(0.0, 1.0);

            // Stance's departure rate, converted from per-stance-progress
            // into per-swing-progress by the halves' relative durations.
            let duty = params.duty_factor.clamp(0.01, 0.99);
            let end_slope = 2.0 * params.ankle_range * (1.0 - duty) / duty;

            hermite(params.ankle_range, -params.ankle_range, 0.0, end_slope, t)
        }
    }
}

/// How far the hips sit below their standing height at this phase, metres,
/// on the rig the gait's fractional amplitudes are being scaled onto.
///
/// Twice per cycle — the hips drop at each midstance, not once. Getting this
/// at cycle rate rather than double rate is the classic harmonic error and
/// reads as a limp.
pub fn hip_height_offset_on(
    phase: f32,
    params: &GaitParams,
    rig: &super::rig::RigGeometry,
) -> f32 {
    let doubled = wrap_phase(wrap_phase(phase) * 2.0);
    -params.hip_dip_metres(rig) * pulse(doubled)
        - params.vertical_bob_metres(rig) * (1.0 - pulse(doubled))
}

/// [`hip_height_offset_on`] for the synthetic T-pose rig.
///
/// Correct only when the pose is being measured on that rig too — which is
/// what most of this module's own tests do. Anything driving a real asset
/// wants the rig-aware form.
pub fn hip_height_offset(phase: f32, params: &GaitParams) -> f32 {
    hip_height_offset_on(phase, params, &super::rig::RigGeometry::default())
}

/// Builds the walking pose at a point in the cycle, composed onto `base`.
///
/// `phase` is the cycle position in **`[0, 1)`**, wrapping — a fraction of a
/// stride, NOT radians. [`super::phase::GaitPhase::gait`] holds the same
/// quantity in radians over `[0, TAU)`, so it needs dividing by `TAU` first;
/// [`cycle_of`] does that conversion and is the safer way to bridge the two.
///
/// Passing radians here is silent rather than loud: the value simply wraps,
/// running the legs through six cycles per real stride. The visible symptom
/// is both hips sitting at nearly the same angle every frame instead of half
/// a cycle apart — measured at 11 and 12 degrees when this was first wired
/// into the gallery.
///
/// The right leg reads the phase half a cycle offset from the left, which is
/// what makes the gait alternate.
///
/// The result is a pose, so it composes with everything else: the phase
/// layer adds secondary motion on top, the spring smooths it, and the IK
/// stage adapts the feet to the ground.
pub fn walk_pose(phase: f32, params: &GaitParams, base: &LocalPose) -> LocalPose {
    walk_pose_on(phase, params, base, &super::rig::RigGeometry::default())
}

/// One leg's sagittal angles, radians: `[thigh, hip flexion, knee flexion,
/// ankle dorsiflexion]`.
///
/// Winter's definitions (Appendix A, Tables A.3 and A.4): the thigh's
/// absolute angle from vertical, forward positive; the hip is the thigh
/// relative to the trunk, the knee the shank relative to the thigh, the
/// ankle the foot relative to the shank. Measured as signed angles in the
/// rig's own sagittal plane (its forward and world up), from joint centres.
///
/// # Where each zero is
///
/// **Geometric** for the thigh and knee: vertical, and a straight leg — the
/// hip, knee and ankle in a line. That is what the book's angles are, and
/// NOT what the rig's bind is: `puppet_base`'s T-pose knee sits 6.4 degrees
/// off its hip-to-ankle line. Zeroed on the bind instead, a recorded straight
/// knee became a 6.4-degree bend and the recorded thigh inherited the bind's
/// tilt: the late-stance leg came up short and the pelvis dropped 6 cm, twice
/// the recording's, to keep the foot down.
///
/// **The bind** for the hip and ankle, whose geometric zeros depend on where
/// a rig puts its spine and toe joints; the bind stands straight with its
/// foot flat, which is the anatomical zero.
pub fn sagittal_angles(
    pose: &LocalPose,
    rig: &super::rig::RigGeometry,
    (socket, knee, ankle, toe): (Bone, Bone, Bone, Bone),
) -> [f32; 4] {
    let raw = |pose: &LocalPose| {
        use super::rig::offset_from;
        let forward = rig.forward();
        let from_hips = |bone| offset_from(pose, rig, Bone::Hips, bone);
        let (s, k, a, t) = (from_hips(socket), from_hips(knee), from_hips(ankle), from_hips(toe));
        let trunk = from_hips(Bone::Spine2);
        // Angle from straight DOWN, positive toward forward.
        let hanging = |v: Vec3| v.dot(forward).atan2(-v.y);
        let lean = trunk.dot(forward).atan2(trunk.y);
        let (thigh, shank, foot) = (hanging(k - s), hanging(a - k), hanging(t - a));
        [thigh, thigh + lean, thigh - shank, foot - shank]
    };
    let (now, bind) = (raw(pose), raw(&LocalPose::REST));
    [now[0], now[1] - bind[1], now[2], now[3] - bind[3]]
}

/// The bones [`sagittal_angles`] reads for one side, hip socket first.
pub fn leg_joints(ankle: Bone) -> (Bone, Bone, Bone, Bone) {
    match ankle {
        Bone::RightFoot => (Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot, Bone::RightToeBase),
        _ => (Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::LeftToeBase),
    }
}

/// Maps a leg's cycle position onto the reference stride's, so its toe-off
/// falls at this gait's duty factor rather than the recording's.
///
/// A smooth, monotone warp `p + c·sin(2πp)/2π`: identity when the duty
/// matches the recording, and never a kink — a piecewise-linear retiming
/// would put a velocity step into every joint at footfall and toe-off.
pub(crate) fn reference_phase(cycle: f32, duty_factor: f32) -> f32 {
    use std::f32::consts::TAU;
    let p = wrap_phase(cycle);
    let (s, d) = (super::reference::STANCE_FRACTION, duty_factor.clamp(0.3, 0.9));
    let sine = (TAU * d).sin();
    if (s - d).abs() < 1.0e-6 || sine.abs() < 1.0e-3 {
        return p;
    }
    let c = (TAU * (s - d) / sine).clamp(-0.95, 0.95);
    p + c * (TAU * p).sin() / TAU
}

/// The least a walking knee bends, radians (6.9 degrees).
///
/// The recording's knee is straight at heel contact (-0.6 degrees) and dips
/// to -3 just before it. On a rig whose bind knee is straight that is the
/// reach singularity — the hip, knee and ankle in a line, with no bend
/// direction for the leg IK to keep — so the knee is held a little short:
/// at 6.9 degrees the leg reaches 99.8% of its length (1.6 mm short on
/// `puppet_base`), enough for a defined bend, and within two degrees of the
/// recording's own mid-stance minimum of 5.2. A SOFT floor, so the curve
/// keeps no corner where it meets it.
///
/// The authored walk held 0.25 rad here on the belief that walking knees
/// stay at 15-20 degrees through stance. Winter's data says otherwise.
pub(crate) const KNEE_FLOOR: f32 = 0.12;
/// Width of [`KNEE_FLOOR`]'s rounding, radians (~1.7 degrees).
pub(crate) const KNEE_FLOOR_SOFTNESS: f32 = 0.03;

/// `max(x, floor)`, rounded over `softness` so it has no corner.
pub(crate) fn soft_floor(x: f32, floor: f32, softness: f32) -> f32 {
    let t = (x - floor) / softness;
    if t > 20.0 { x } else { floor + softness * t.exp().ln_1p() }
}

/// How much of the body's weight a stance leg carries at `progress`
/// through its stance: ramping in over the double-support window after
/// footfall, out over the one before toe-off, and 1 in single support.
///
/// Shared by everything that asks "which foot is the body on" — hip height
/// and root motion — so the two agree through every hand-over.
pub fn stance_load(progress: f32, duty_factor: f32) -> f32 {
    let duty = duty_factor.clamp(0.51, 0.99);
    let ramp = (duty - 0.5) / duty;
    smoothstep(progress.min(1.0 - progress) / ramp)
}

/// [`walk_pose`], on a specific rig.
///
/// The rig-aware form, and the one anything driving a real asset wants.
/// [`walk_pose`] is this with the synthetic T-pose.
///
/// # What the rig is for
///
/// Only the vertical amplitudes — [`GaitParams::hip_dip`] and
/// [`GaitParams::vertical_bob`] are fractions of leg length, and resolving
/// them needs to know how long this rig's leg actually is. Every joint
/// angle here is rig-independent and unaffected.
///
/// Posing a real rig through [`walk_pose`] instead scales the body's
/// vertical motion to the synthetic rig's `0.49 m` leg while the real one
/// is `0.888 m`, so the hips move roughly half as far as the gait intends
/// relative to the legs carrying them.
pub fn walk_pose_on(
    phase: f32,
    params: &GaitParams,
    base: &LocalPose,
    rig: &super::rig::RigGeometry,
) -> LocalPose {
    // Which way this rig's legs have to swing. `+1` reproduces exactly what
    // this function did before the rig was a parameter.
    let facing = super::stance::facing_sign(rig);

    // Wrapped once, here, so every curve below sees a sane cycle position
    // and no downstream multiplication can overflow.
    let phase = wrap_phase(phase);
    let mut pose = *base;

    let legs = [
        (phase, Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot),
        (phase + 0.5, Bone::RightUpLeg, Bone::RightLeg, Bone::RightFoot),
    ];

    let mean = match params.curves {
        LegCurves::Authored => thigh_cycle_mean(params),
        LegCurves::Measured { .. } => 0.0,
    };

    // The measured stride: recorded angles, as deltas from the base pose's
    // own. The thigh's is ABSOLUTE — its angle from vertical, not from the
    // trunk; see `reference::Stride::thigh` for why the book's hip angle
    // cannot place a leg — and the knee's and ankle's are relative, like
    // composing a rotation onto a bone, so each joint carries the ones below
    // it exactly as a real leg does. `walk::WalkCycle` holds the per-rig
    // correction that keeps both feet planted through double support.
    let measured = match params.curves {
        LegCurves::Measured { amplitude } => {
            pose = super::walk::walk_cycle(params, amplitude, base, rig).pose(base, rig, phase, facing);
            true
        }
        LegCurves::Authored => false,
    };

    for (leg_cycle, hip, knee, ankle) in legs {
        if measured {
            break;
        }
        let leg = leg_phase(leg_cycle, params.duty_factor);

        let flex = knee_flex(leg, params);
        let thigh = match leg {
            LegPhase::Stance { progress } => {
                stance_thigh(progress, params, mean, base, rig, facing, (hip, knee, ankle))
            }
            LegPhase::Swing { .. } => thigh_angle_centred(leg, params, mean),
        };
        // NEGATED, because the foot bone points forward (`-Z`) while every
        // other bone here hangs down (`-Y`), so the same axis means the
        // opposite thing for it — see `ankle_angle`'s own note.
        let foot = -ankle_angle(leg, params);

        // NEGATED, because this module's curves are authored with "negative
        // is forward" — matching how a hip angle is usually written — while
        // a positive rotation about `KNEE_AXIS` actually swings the leg
        // forward.
        //
        // That axis's own doc comment claimed the opposite until this was
        // measured, and the walk cycle built on it ran BACKWARD: during
        // stance the planted foot travelled toward `-Z` relative to the
        // hips, which is a body moving in reverse over its own feet.
        // Verified directly — `+0.3` rad about `KNEE_AXIS` moves `LeftFoot`
        // from `z = +0.002` to `z = -0.191`.
        // Every angle below is scaled by `facing`, which is `+1` on a rig
        // that faces the way these curves were authored and `-1` on one
        // that faces the other way. Without it a `+Z`-facing rig gets every
        // leg angle inverted, and the knees bend backward: measured on
        // `puppet_base`, the knee sat 0.090 m BEHIND the hip-to-ankle line
        // at phase 0.188 on a leg with real slack, sustained across a
        // quarter of the cycle. See `stance::facing_sign`.
        compose(&mut pose, hip, Quat::from_axis_angle(KNEE_AXIS, facing * -thigh));
        // Negative, because flexion bends the shin BACKWARD relative to the
        // thigh — which is the only direction a knee goes.
        compose(&mut pose, knee, Quat::from_axis_angle(KNEE_AXIS, facing * -flex));
        // The ankle carries the thigh and knee rotations with it, so its own
        // angle is what is left after cancelling theirs — otherwise the foot
        // inherits the whole leg's rotation and points at the sky.
        //
        // What accumulates above it is `(-thigh) + (-flex)`, so cancelling
        // means ADDING both back.
        compose(
            &mut pose,
            ankle,
            Quat::from_axis_angle(KNEE_AXIS, facing * (foot + thigh + flex)),
        );
    }

    // The arms counter-swing: the left arm goes with the right leg. This is
    // the strongest natural-gait cue after the legs themselves, and it is
    // nearly free.
    //
    // Unlike the legs, the swing axis here cannot be a constant. A leg
    // always hangs downward, so `KNEE_AXIS` is always perpendicular to it;
    // an arm's direction depends entirely on the base pose. In the bare
    // T-pose the arms extend along `+/-X` — the same axis — so rotating
    // about it spins each arm around its own length and moves the hand
    // exactly nowhere. Measured: 0.0 m of hand travel, both sides.
    //
    // The superseded module hit the mirror image of this: its arm swing was
    // tuned against a splayed T-pose, and when the idle was re-derived to
    // hang the arms downward the same rotation produced 2.4x too little
    // forward travel.
    //
    // So the axis is derived per-arm: perpendicular to the arm's own
    // direction and to world up, which is the axis that swings a hand
    // forward and back whatever the pose is doing.
    //
    // # Measured on the rig being posed
    //
    // The arm's direction came from `forward_kinematics(&pose)` — the
    // SYNTHETIC rig's — while the character is a real one whose arms point
    // elsewhere. On `puppet_base` the derived axis ran nearly along the arm,
    // so the swing mostly spun each arm about its own length: the hands
    // travelled 22 mm ahead and 31 mm behind. The direction is now read
    // from the rig in hand, and "forward" is the rig's own.
    //
    // # Timed to the opposite foot, with a pendulum's lag
    //
    // The swing was `sin(2π·cycle)`, peaking a quarter-cycle away from the
    // opposite foot's forward peak — measured, the left hand peaked at 0.75
    // against the right foot's 0.445, so at the moment the right foot was
    // ahead both hands sat at their midline. Each arm now peaks just after
    // its opposite footfall ([`ARM_LAG`]): an arm is a pendulum driven from
    // the shoulder, and trails the leg that drives the rhythm.
    //
    // # The upper arm swings back; the forearm carries the hand forward
    //
    // Murray, Sepic & Barnard (1967) measured 30 men: the upper arm spends
    // most of the stride behind the vertical (8 degrees forward, 24 back),
    // and the elbow folds from 17 to 47 degrees as it comes forward. So the
    // hand still travels further forward than back, by the elbow. This walk
    // had it the other way round, the upper arm 28 forward and 10 back on
    // an elbow that moved 10 degrees: a straight arm thrown forward, which
    // reads as marching. The swing is now centred behind the shoulder
    // ([`ARM_SWING_CENTRE`]), and the elbow keeps [`GaitParams::elbow_carry`]
    // of its fold at the back of the swing.
    let forward = rig.forward();
    let arms = [
        // Left arm with the right leg, which lands at 0.5.
        (0.5, Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand),
        (0.0, Bone::RightArm, Bone::RightForeArm, Bone::RightHand),
    ];

    for (opposite_footfall, shoulder, elbow, hand) in arms {
        use super::rig::offset_from;

        let along = offset_from(&pose, rig, shoulder, hand).normalize_or_zero();

        // Rotating about `along × forward` by a positive angle carries the
        // hand along `+forward` — derived, so the sign holds on any rig and
        // either facing.
        let axis = {
            let candidate = along.cross(forward).normalize_or_zero();
            // Degenerate when the arm already points forward.
            if candidate.length_squared() < 0.25 { KNEE_AXIS } else { candidate }
        };

        // Peaks (s = 1) a quarter-cycle after its zero, placed at the
        // opposite footfall plus the pendulum's lag.
        let at = |lag: f32| {
            ((phase - (opposite_footfall + lag - 0.25)) * std::f32::consts::TAU).sin()
        };

        let swing = params.arm_swing * (at(ARM_LAG) + ARM_SWING_CENTRE);
        // The forearm follows through a little behind the upper arm, and
        // never unfolds below its carried bend.
        let carry = params.elbow_carry;
        let fold = params.elbow_bend * (carry + (1.0 - carry) * (0.5 + 0.5 * at(ARM_LAG + ELBOW_FOLLOW)));

        // Turned in the WORLD, about the axis measured on the posed arm —
        // not composed onto the delta, which would apply it in the arm's
        // T-pose frame. See `rig::delta_after_world_turn`.
        pose.rotations[shoulder] = super::rig::delta_after_world_turn(
            &pose,
            rig,
            shoulder,
            Quat::from_axis_angle(axis, swing),
        );
        // Flexion carries the forearm FORWARD relative to the upper arm,
        // which by the axis above is the positive direction. The shoulder's
        // turn is about this same axis, so the axis still holds for the
        // elbow.
        pose.rotations[elbow] = super::rig::delta_after_world_turn(
            &pose,
            rig,
            elbow,
            Quat::from_axis_angle(axis, fold),
        );
    }

    // A walk's height comes from its stance leg; a run's flight phase has
    // no stance leg to derive it from, so it keeps the authored curve.
    // The measured walk has already set its pelvis — from the recording,
    // with the legs planted to meet it (`walk::WalkCycle::pose`).
    pose.root_translation.y += if measured {
        0.0
    } else if params.duty_factor > 0.5 {
        stance_hip_height(phase, params, base, &pose, rig)
    } else {
        hip_height_offset_on(phase, params, rig)
    };

    if measured {
        super::walk::clear_swinging_feet(&mut pose, phase, params, base, rig, facing);
    }

    pose
}

/// How far the hips rise or sink to keep the stance foot where it stood.
///
/// # Why derived rather than authored
///
/// This was an authored curve, and it had two faults. It knew nothing about
/// the legs, so the planted ankle drifted up to 81 mm from where it stood
/// over each stance — pushed into the ground or lifted off it — and foot IK
/// spent every frame undoing it. And it had the walk's vertical motion
/// backwards: lowest at midstance, which is how a RUN moves. A walking body
/// vaults over its stance leg like an inverted pendulum, highest as it
/// passes over the foot and lowest in double support, when both legs are
/// spread.
///
/// Placing the hips wherever puts the stance ankle back at its standing
/// height gets both right by construction: the leg's own geometry is the
/// vault.
///
/// # Two feet down
///
/// In double support each foot's answer is weighted by how much load it
/// carries — ramping in after footfall and out before toe-off over exactly
/// the double-support window — so the body hands over from one foot to the
/// other continuously, and in single support it answers to one foot alone.
fn stance_hip_height(
    phase: f32,
    params: &GaitParams,
    base: &LocalPose,
    pose: &LocalPose,
    rig: &super::rig::RigGeometry,
) -> f32 {
    use super::foot::{lowest, support_height, Sole};

    let (mut loads, mut needs) = ([0.0; 2], [0.0; 2]);
    for (i, (cycle, ankle)) in [(phase, Bone::LeftFoot), (phase + 0.5, Bone::RightFoot)].into_iter().enumerate() {
        let LegPhase::Stance { progress } = leg_phase(cycle, params.duty_factor) else {
            continue;
        };
        loads[i] = stance_load(progress, params.duty_factor);
        // How much further below the hips the foot's lowest point is than
        // when standing: the hips rise by exactly that to put it back on the
        // ground. The LOWEST point, not the ankle — a foot rolling from heel
        // to toe keeps whichever end bears weight on the floor, and holding
        // the ankle at its standing height instead would push the heel
        // through the ground at every heel strike.
        let sole = Sole::of(rig, ankle);
        needs[i] = lowest(&sole.points(base, rig)) - lowest(&sole.points(pose, rig));
    }

    support_height(loads, needs).unwrap_or_else(|| hip_height_offset_on(phase, params, rig))
}

/// How far behind the opposite footfall an arm reaches its forward peak, as
/// a fraction of the cycle. An arm is a pendulum driven from the shoulder.
///
/// Small, because the arm springs add their own lag on top — about 0.08 of
/// a cycle at a normal cadence (see `dho::default_springs`) — and the total
/// is what a viewer sees: ~0.1 of a cycle, a natural pendulum's trail.
const ARM_LAG: f32 = 0.02;

/// Where the upper arm's swing is centred, as a fraction of
/// [`GaitParams::arm_swing`]: at -0.5 it reaches 0.5x forward of the
/// vertical and 1.5x behind it, Murray's 8 and 24 degrees. It was +0.3,
/// from a belief that an arm swings 2:1 forward; that is the HAND, carried
/// forward by the elbow, not the upper arm.
///
/// A fraction rather than an angle, so a gait with its amplitudes zeroed
/// still leaves the pose alone.
const ARM_SWING_CENTRE: f32 = -0.5;

/// How much further the forearm trails the upper arm, as a fraction of the
/// cycle — follow-through.
const ELBOW_FOLLOW: f32 = 0.04;

/// One leg's hip, knee and ankle bones.
type LegBones = (Bone, Bone, Bone);

/// Poses one leg onto `base` exactly as [`walk_pose_on`] does.
fn pose_leg(
    base: &LocalPose,
    facing: f32,
    (hip, knee, ankle): LegBones,
    thigh: f32,
    flex: f32,
    foot: f32,
) -> LocalPose {
    let mut pose = *base;
    compose(&mut pose, hip, Quat::from_axis_angle(KNEE_AXIS, facing * -thigh));
    compose(&mut pose, knee, Quat::from_axis_angle(KNEE_AXIS, facing * -flex));
    compose(&mut pose, ankle, Quat::from_axis_angle(KNEE_AXIS, facing * (foot + thigh + flex)));
    pose
}

/// How far ahead of the hips the ankle sits, along the rig's forward, with
/// one leg posed.
fn ankle_ahead(
    base: &LocalPose,
    rig: &super::rig::RigGeometry,
    facing: f32,
    bones: LegBones,
    (thigh, flex, foot): (f32, f32, f32),
) -> f32 {
    let pose = pose_leg(base, facing, bones, thigh, flex, foot);
    super::rig::offset_from(&pose, rig, Bone::Hips, bones.2).dot(rig.forward())
}

/// The stance thigh angle that moves the foot under the hips at a CONSTANT
/// speed.
///
/// # Why the authored thigh curve was not enough
///
/// Through stance the thigh sweeps back at a constant ANGULAR rate, but the
/// foot's position under the hip is `L_thigh·sin θ + L_shin·sin(θ + flex)`
/// and the knee yields mid-stance — so the foot does not travel back at a
/// constant rate. The body's speed IS that rate (see
/// [`super::locomotion`]), so an uneven sweep is a body that lurches:
/// measured 0.73-1.28 m/s per unit cadence on the real rig, with a jump at
/// every hand-over between feet, and 0.3-1.76 m/s live.
///
/// So the knee and ankle keep their authored curves, and the thigh is
/// SOLVED for the foot to move linearly between where the authored curve
/// puts it at footfall and at toe-off. The endpoints are the authored ones,
/// so the hand-over to swing is unchanged; only the sweep between them is
/// evened out.
///
/// Solved by secant iteration on the rig's own forward kinematics — the
/// foot's position is a smooth, monotone function of the thigh over the
/// stance range, so a few steps land well under a millimetre.
fn stance_thigh(
    progress: f32,
    params: &GaitParams,
    mean: f32,
    base: &LocalPose,
    rig: &super::rig::RigGeometry,
    facing: f32,
    bones: LegBones,
) -> f32 {
    let at = |p: f32| {
        let leg = LegPhase::Stance { progress: p };
        (thigh_angle_centred(leg, params, mean), knee_flex(leg, params), -ankle_angle(leg, params))
    };

    let footfall = ankle_ahead(base, rig, facing, bones, at(0.0));
    let toe_off = ankle_ahead(base, rig, facing, bones, at(1.0));
    let p = progress.clamp(0.0, 1.0);
    let wanted = footfall + (toe_off - footfall) * p;

    let (authored, flex, foot) = at(p);
    let miss = |thigh: f32| ankle_ahead(base, rig, facing, bones, (thigh, flex, foot)) - wanted;

    let (mut a, mut b) = (authored, authored + 0.02);
    let (mut fa, mut fb) = (miss(a), miss(b));
    for _ in 0..4 {
        if fa.abs() < 1.0e-5 || (fb - fa).abs() < 1.0e-9 {
            break;
        }
        let next = b - fb * (b - a) / (fb - fa);
        (a, fa) = (b, fb);
        b = next;
        fb = miss(b);
    }

    // Whichever point is closer — the loop can stop on its FIRST guess when
    // that is already exact, as it is at both ends of stance, and returning
    // the probe beside it then moves a leg that was already right.
    let (best, miss_at_best) = if fa.abs() <= fb.abs() { (a, fa) } else { (b, fb) };

    // A failed solve keeps the authored curve rather than inventing one.
    if best.is_finite() && miss_at_best.is_finite() && miss_at_best.abs() < 0.01 {
        best
    } else {
        authored
    }
}

/// Multiplies a rotation onto whatever the pose already has, so the gait
/// layers onto an authored stance rather than replacing it.
pub(crate) fn compose(pose: &mut LocalPose, bone: Bone, rotation: Quat) {
    pose.set_rotation(bone, pose.rotation(bone) * rotation);
}

/// Where a foot should be, relative to the body, at a point in the cycle.
///
/// Positive `z` is behind the character (it faces `-Z`), so a foot starts a
/// stance ahead of the body and ends it behind.
///
/// This is what [`super::locomotion`] differentiates to get the root
/// velocity, and it is deliberately the *intended* position rather than one
/// read back from the posed skeleton — the whole point is to know where the
/// foot was meant to be, independently of what the IK then did to it.
pub fn foot_offset_z(leg: LegPhase, params: &GaitParams) -> f32 {
    let half = params.stride_length * 0.5;

    match leg {
        // Planted: travels backward relative to the body at a constant rate,
        // because the body is moving forward over a stationary foot.
        LegPhase::Stance { progress } => -half + params.stride_length * progress.clamp(0.0, 1.0),
        // Swinging: returns to the front, eased.
        LegPhase::Swing { progress } => half - params.stride_length * smoothstep(progress),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::character::anim::rig::forward_kinematics;
    use crate::character::anim::stance::stance;

    /// A sampling of the cycle fine enough to catch a discontinuity.
    const SAMPLES: usize = 720;

    fn base() -> LocalPose {
        stance(&LocalPose::REST)
    }

    fn params() -> GaitParams {
        GaitParams::default()
    }

    // -----------------------------------------------------------------
    // Structural invariants — run before any screenshot, per AGENTS.md
    // -----------------------------------------------------------------

    #[test]
    fn no_bone_ever_changes_length_across_the_whole_cycle() {
        // Free by construction — a rotation cannot stretch a bone — and
        // asserted anyway as the guard against a future translation channel.
        // The superseded module measured a leg growing from 0.461 m to
        // 1.689 m; that class of failure must stay impossible.
        let base = base();
        let params = params();

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;
            let positions = forward_kinematics(&walk_pose(phase, &params, &base));

            for &bone in Bone::ALL.iter() {
                let Some(parent) = bone.parent() else { continue };

                let rest = bone.t_pose_offset().length();
                let posed = (positions[bone] - positions[parent]).length();

                assert!(
                    (posed - rest).abs() < 1.0e-5,
                    "at phase {phase}, {} is {posed} m against a rest length of {rest}",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn the_two_legs_are_exact_mirrors_half_a_cycle_apart() {
        // The gait must be symmetric in time, not hand-tuned per side. Any
        // asymmetry here is a limp, and a limp introduced by accident is
        // very hard to spot by eye.
        let base = base();
        let params = params();

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;

            let now = walk_pose(phase, &params, &base);
            let half_later = walk_pose(phase + 0.5, &params, &base);

            // Mirrored across the sagittal plane (`convert::mirrored`'s
            // convention, +X lateral): a turn about X — all a sagittal
            // stride has — is its own mirror, but the feet brought in to
            // the step width turn about forward, opposite ways per side.
            let mirror = |q: Quat| Quat::from_xyzw(q.x, -q.y, -q.z, q.w);
            for (left, right) in [
                (Bone::LeftUpLeg, Bone::RightUpLeg),
                (Bone::LeftLeg, Bone::RightLeg),
                (Bone::LeftFoot, Bone::RightFoot),
            ] {
                assert!(
                    mirror(now.rotation(left)).abs_diff_eq(half_later.rotation(right), 1.0e-5),
                    "at phase {phase}, {} does not match {} half a cycle later",
                    left.name(),
                    right.name(),
                );
            }
        }
    }

    #[test]
    fn the_cycle_closes_in_position_and_in_velocity() {
        // A loop that is position-continuous but velocity-discontinuous
        // hitches once per stride. The spring downstream cannot hide it,
        // because the spring is chasing a target that jumped.
        let base = base();
        let params = params();

        // The step has to clear a precision floor from below and stay local
        // to the seam from above, and both bounds are real.
        //
        // Too small and `Quat::angle_between` returns quantisation noise:
        // its f32 floor near identity is ~9.8e-4 rad (see the project's own
        // note), and a 1e-4 step produces only ~1.2e-4 rad of real
        // rotation — an order of magnitude below it. This test originally
        // used that and its verdicts were meaningless in both directions.
        //
        // Too large and it stops measuring the seam at all: at 0.01 the
        // "before" samples sit at phases 0.98 and 0.99, deep inside swing
        // where the knee is still travelling fast, and compare that against
        // a stance region that is genuinely flat — reporting a 0.96-vs-0
        // "jump" across a join that is provably C1.
        //
        // 0.002 gives ~2.4e-3 rad, twice the floor, across 0.2% of the
        // cycle.
        let step = 0.002;

        for bone in [Bone::LeftUpLeg, Bone::LeftLeg, Bone::LeftFoot, Bone::RightUpLeg] {
            let at = |p: f32| walk_pose(p, &params, &base).rotation(bone);

            // Position: the seam closes.
            assert!(
                at(0.0).abs_diff_eq(at(1.0), 1.0e-6),
                "{}'s cycle does not close",
                bone.name(),
            );

            // Velocity: the one-sided limits at the seam must agree.
            //
            // Compared as extrapolated LIMITS rather than as raw chords.
            // Two samples on each side measure an average rate over a
            // window, and where the curve has real curvature — which it has
            // here, the knee decelerating into footfall — those averages
            // differ even across a perfectly C1 join. Measured on the
            // ankle: rates of -0.875, -0.670, -0.455 approaching and -0.340,
            // -0.330 leaving, a smooth progression that a chord comparison
            // reported as a "0.69 vs 0" jump.
            //
            // Linear extrapolation from two chords on each side removes the
            // curvature term and leaves the quantity actually in question:
            // where each side's rate is HEADING as it reaches the seam.
            // The rotations here are all about one axis, so the signed angle
            // about it is well defined — and necessary.
            // `Quat::angle_between` is UNSIGNED: it cannot tell a rotation
            // slowing down from one reversing, so a curve passing through
            // its minimum reads as a rate of exactly zero on both chords.
            // That is what produced a stubborn "0.58 vs 0" on the ankle
            // across a join measured smooth by direct sampling.
            // Read straight off the quaternion rather than via
            // `to_axis_angle`, which normalises the axis to point whichever
            // way makes the angle positive — so it flips sign as the
            // rotation passes through zero and reports two rates of equal
            // magnitude and opposite sign across a smooth join (measured
            // 1.205 vs -1.178).
            //
            // These rotations are all about X, so `2 * asin(q.x)` is the
            // signed angle directly and is continuous through zero.
            let signed = |q: Quat| 2.0 * q.x.clamp(-1.0, 1.0).asin();

            // Chords taken in INCREASING phase order on both sides, so the
            // two rates are directly comparable. Sampling the "before" side
            // backwards — the natural way to write it, walking away from the
            // seam — negates its differences and reports two equal-magnitude
            // opposite-sign rates across a perfectly smooth join.
            let rate = |from: f32, to: f32| (signed(at(to)) - signed(at(from))) / step;

            let limit_from_below = {
                let near = rate(1.0 - 2.0 * step, 1.0 - step);
                let far = rate(1.0 - 3.0 * step, 1.0 - 2.0 * step);
                near + (near - far) * 0.5
            };
            let limit_from_above = {
                let near = rate(step, 2.0 * step);
                let far = rate(2.0 * step, 3.0 * step);
                near - (far - near) * 0.5
            };

            let (before, after) = (limit_from_below, limit_from_above);

            assert!(
                (before - after).abs() < 0.5,
                "{}'s angular rate jumps across the seam: {before} vs {after}",
                bone.name(),
            );
        }
    }

    #[test]
    fn a_knee_never_hyperextends_anywhere_in_the_cycle() {
        // The most visceral unnatural-motion failure there is. Flexion is
        // non-negative by construction here, which this pins.
        let params = params();

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;
            let flex = knee_flex(leg_phase(phase, params.duty_factor), &params);

            assert!(
                flex >= 0.0,
                "at phase {phase} the knee flexion is {flex} — a knee bending backward",
            );
        }
    }

    #[test]
    fn the_stance_knee_is_never_locked_straight() {
        // A leg at exactly full extension sits at the reach singularity,
        // where it has no bend direction and the IK has no solution space —
        // and it reads as a stiff, stilted walk.
        let params = params();

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;
            let leg = leg_phase(phase, params.duty_factor);

            if leg.is_stance() {
                let flex = knee_flex(leg, &params);
                assert!(
                    flex > 0.02,
                    "at phase {phase} the stance knee is at {flex} rad — effectively locked",
                );
            }
        }
    }

    #[test]
    fn the_knee_leads_the_thigh_through_swing() {
        // THE recorded "single biggest fix" from the superseded module,
        // as an ordering assertion: the lower leg folds up and under BEFORE
        // the thigh reaches its forward peak. Reverse the order and the toe
        // drags through the ground.
        let params = params();

        let mut knee_peak_at = 0.0;
        let mut knee_peak = f32::MIN;
        let mut thigh_peak_at = 0.0;
        let mut thigh_forward_peak = f32::MAX;

        for i in 0..SAMPLES {
            let progress = i as f32 / SAMPLES as f32;
            let leg = LegPhase::Swing { progress };

            let flex = knee_flex(leg, &params);
            if flex > knee_peak {
                knee_peak = flex;
                knee_peak_at = progress;
            }

            // Forward is negative, so the forward peak is the minimum.
            let thigh = thigh_angle(leg, &params);
            if thigh < thigh_forward_peak {
                thigh_forward_peak = thigh;
                thigh_peak_at = progress;
            }
        }

        assert!(
            knee_peak_at < thigh_peak_at,
            "the knee peaks at {knee_peak_at} of swing and the thigh at {thigh_peak_at} — \
             the knee must lead",
        );
    }

    #[test]
    fn the_stance_thigh_sweeps_at_a_constant_rate() {
        // THE guard against the sinusoidal-walk artefact, and it took two
        // attempts to state correctly.
        //
        // The first version compared stance against a time-mirrored swing
        // and asserted they differ. That is not the property: a sine sampled
        // over two unequal-duration halves is ALREADY asymmetric under that
        // comparison — measured 0.432 rad — so the test passed a
        // deliberately substituted `cos()` curve, which is precisely the
        // thing it existed to catch.
        //
        // What actually distinguishes a real gait is the SHAPE of stance.
        // The foot is planted and the body travels over it at constant
        // speed, so the thigh must sweep at a constant ANGULAR RATE. A
        // sinusoid cannot: it is fastest at mid-stance and slowest at the
        // ends, which reads as the body lurching over each foot.
        let params = params();

        let step = 0.01;
        let mut rates = Vec::new();

        for i in 0..99 {
            let a = thigh_angle(LegPhase::Stance { progress: i as f32 * step }, &params);
            let b =
                thigh_angle(LegPhase::Stance { progress: (i + 1) as f32 * step }, &params);
            rates.push((b - a) / step);
        }

        let mean = rates.iter().sum::<f32>() / rates.len() as f32;
        let worst = rates
            .iter()
            .map(|r| (r - mean).abs())
            .fold(0.0f32, f32::max);

        assert!(
            worst < mean.abs() * 0.05,
            "the stance thigh rate varies by {worst} rad about a mean of {mean} — the \
             body is lurching over the planted foot rather than travelling over it at \
             a constant rate, which is the sinusoidal-gait artefact",
        );
    }

    #[test]
    fn the_knee_is_nearly_straight_in_stance_and_sharply_flexed_in_swing() {
        // The second half of the same guard, on the knee rather than the
        // thigh. A sinusoidal knee bends by the same amount on both halves
        // of the cycle; a real one bears weight nearly straight and then
        // folds hard to clear the ground.
        //
        // Stated as a RATIO so it cannot be satisfied by scaling everything
        // down.
        let params = params();

        let mut stance_peak = 0.0f32;
        let mut swing_peak = 0.0f32;

        for i in 0..=SAMPLES {
            let t = i as f32 / SAMPLES as f32;
            stance_peak = stance_peak.max(knee_flex(LegPhase::Stance { progress: t }, &params));
            swing_peak = swing_peak.max(knee_flex(LegPhase::Swing { progress: t }, &params));
        }

        // Bounded against real walking kinematics rather than a round
        // number: published gait data puts the swing peak at 60-65 degrees
        // and the stance peak at 15-20, a ratio near 3.6.
        //
        // An earlier version of this test demanded 4x, which was calibrated
        // against a stance knee that was too straight — it encoded the very
        // bug Phase 2 fixed, and failed when the stance flexion was raised
        // into its correct anatomical range.
        assert!(
            swing_peak > stance_peak * 2.5,
            "the knee peaks at {stance_peak} rad in stance and {swing_peak} in swing — \
             too close to tell the halves apart, which is the sinusoidal-knee artefact",
        );
    }

    #[test]
    fn both_feet_are_down_together_for_part_of_the_cycle() {
        // Double support. Its presence is what makes this a walk rather than
        // a run, and it follows from a duty factor above 0.5 — asserted
        // through the real phase split rather than from the number.
        let params = params();

        let mut both_down = 0;
        let mut none_down = 0;

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;

            let left = leg_phase(phase, params.duty_factor).is_stance();
            let right = leg_phase(phase + 0.5, params.duty_factor).is_stance();

            if left && right {
                both_down += 1;
            }
            if !left && !right {
                none_down += 1;
            }
        }

        assert!(both_down > 0, "a walk needs a double-support phase");
        assert_eq!(
            none_down, 0,
            "a walk never has both feet off the ground — that is a run",
        );
    }

    #[test]
    fn each_foot_is_in_stance_for_the_duty_factor_of_the_cycle() {
        let params = params();

        let stance_samples = (0..SAMPLES)
            .filter(|i| {
                leg_phase(*i as f32 / SAMPLES as f32, params.duty_factor).is_stance()
            })
            .count();

        let measured = stance_samples as f32 / SAMPLES as f32;

        assert!(
            (measured - params.duty_factor).abs() < 0.01,
            "measured a {measured} duty factor against a configured {}",
            params.duty_factor,
        );
    }

    // -----------------------------------------------------------------
    // The phase split
    // -----------------------------------------------------------------

    #[test]
    fn the_phase_clock_converts_to_a_cycle_fraction() {
        // The units bridge, pinned because getting it wrong is SILENT: the
        // raw radian value simply wraps, running six cycles per stride, and
        // the only symptom is both hips sitting at nearly the same angle.
        // Measured at 11 and 12 degrees when the gallery first wired this.
        use crate::character::anim::phase::GaitPhase;

        let at = |gait: f32| cycle_of(&GaitPhase { gait, ..Default::default() });

        assert_eq!(at(0.0), 0.0);

        assert!(
            (at(std::f32::consts::PI) - 0.5).abs() < 1.0e-6,
            "half a turn of the clock is half a cycle, got {}",
            at(std::f32::consts::PI),
        );

        // Just short of a full turn stays inside one cycle.
        let nearly_whole = at(std::f32::consts::TAU - 1.0e-4);
        assert!(nearly_whole < 1.0, "got {nearly_whole}");
    }

    #[test]
    fn phase_zero_is_footfall() {
        // A convention worth pinning, because everything downstream — the
        // contact windows, the foot offset, the root velocity — reads it.
        let LegPhase::Stance { progress } = leg_phase(0.0, 0.6) else {
            panic!("phase 0 should be the start of stance");
        };
        assert!(progress < 1.0e-6, "and at its very beginning, got {progress}");
    }

    #[test]
    fn the_phase_split_wraps_cleanly() {
        for (a, b) in [(0.0, 1.0), (0.25, 1.25), (0.9, -0.1), (0.5, 2.5)] {
            assert_eq!(
                leg_phase(a, 0.6),
                leg_phase(b, 0.6),
                "phases {a} and {b} name the same moment",
            );
        }
    }

    #[test]
    fn progress_runs_zero_to_one_within_each_half() {
        let duty = 0.6;

        for i in 0..SAMPLES {
            let phase = i as f32 / SAMPLES as f32;

            let progress = match leg_phase(phase, duty) {
                LegPhase::Stance { progress } | LegPhase::Swing { progress } => progress,
            };

            assert!(
                (0.0..=1.0).contains(&progress),
                "phase {phase} produced a progress of {progress}",
            );
        }
    }

    #[test]
    fn a_degenerate_duty_factor_is_safe() {
        for duty in [0.0, 1.0, -1.0, 2.0, f32::NAN] {
            let phase = leg_phase(0.3, duty);
            let progress = match phase {
                LegPhase::Stance { progress } | LegPhase::Swing { progress } => progress,
            };
            assert!(progress.is_finite(), "duty {duty} produced {progress}");
        }
    }

    // -----------------------------------------------------------------
    // Foot travel
    // -----------------------------------------------------------------

    #[test]
    fn a_planted_foot_travels_exactly_one_stride_backward_through_stance() {
        // Relative to the body — which is what a planted foot does while the
        // body moves forward over it. This is the quantity
        // `locomotion` differentiates for root velocity, so it has to be
        // exactly the stride, not approximately.
        let params = params();

        let start = foot_offset_z(LegPhase::Stance { progress: 0.0 }, &params);
        let end = foot_offset_z(LegPhase::Stance { progress: 1.0 }, &params);

        assert!(
            (end - start - params.stride_length).abs() < 1.0e-6,
            "the foot travelled {} m against a {} m stride",
            end - start,
            params.stride_length,
        );
    }

    #[test]
    fn the_foot_offset_closes_its_own_cycle() {
        let params = params();

        let at_footfall = foot_offset_z(LegPhase::Stance { progress: 0.0 }, &params);
        let at_end_of_swing = foot_offset_z(LegPhase::Swing { progress: 1.0 }, &params);

        assert!(
            (at_footfall - at_end_of_swing).abs() < 1.0e-6,
            "the swing ends at {at_end_of_swing} but the next stance starts at \
             {at_footfall}",
        );
    }

    #[test]
    fn a_planted_foot_moves_backward_at_a_constant_rate() {
        // Because the body moves over it at constant speed. A varying rate
        // here would mean the body accelerating and decelerating over each
        // step, which reads as a lurch — and would make the root velocity
        // that `locomotion` derives from this jump every frame.
        let params = params();

        let step = 0.01;
        let mut rates = Vec::new();

        for i in 0..99 {
            let a = foot_offset_z(LegPhase::Stance { progress: i as f32 * step }, &params);
            let b =
                foot_offset_z(LegPhase::Stance { progress: (i + 1) as f32 * step }, &params);
            rates.push((b - a) / step);
        }

        let first = rates[0];
        for (i, rate) in rates.iter().enumerate() {
            assert!(
                (rate - first).abs() < 1.0e-3,
                "the stance rate varies: {first} at the start, {rate} at sample {i}",
            );
        }
    }

    // -----------------------------------------------------------------
    // Hip height
    // -----------------------------------------------------------------

    #[test]
    fn probe_what_happens_at_the_synthetic_backward_phases() {
        use crate::character::anim::rig::{forward_kinematics_on, RigGeometry};
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        let rig = RigGeometry::default();
        let base = stance_on_rig(&LocalPose::REST, DEFAULT_KNEE_FLEX, &rig);
        let params = GaitParams::default();

        println!("  phase | leg phase | knee flex | thigh | knee offset | reach frac");
        for i in 10..16 {
            let phase = i as f32 / 32.0;
            let leg = leg_phase(phase + 0.5, params.duty_factor);
            let pose = walk_pose_on(phase, &params, &base, &rig);
            let k = forward_kinematics_on(&pose, &rig);
            let hip = k[Bone::LeftUpLeg];
            let ankle = k[Bone::LeftFoot];
            let span = (ankle - hip).length();
            let straight = rig.offsets[Bone::LeftLeg].length()
                + rig.offsets[Bone::LeftFoot].length();
            println!(
                "  {phase:.3} | {:?} | flex={:.3} | thigh={:.3} | offset={:+.4} | {:.3}",
                leg,
                knee_flex(leg, &params),
                thigh_angle(leg, &params),
                rig.knee_forward_offset(&pose, crate::character::anim::rig::Side::Left),
                span / straight,
            );
        }
    }

    #[test]
    fn the_knee_bends_forward_at_every_phase_on_every_rig() {
        // The anatomical invariant, applied across the WHOLE cycle. A knee
        // bends forward; a leg that folds the other way is an insect's.
        //
        // This is the assertion thirty-odd leg tests in this crate could
        // not make, because they measured the unsigned angle between thigh
        // and shin — identical whichever way the knee folds. The rendered
        // character walked on backward knees past all of them.
        //
        // Checked on a rig with REAL bind rotations: the synthetic rig
        // faces the way these curves were authored, so it cannot disagree.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::{leg_extension_fraction, RigGeometry, Side};
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};

        // # The rig has to be the one the GAME assembles
        //
        // `gltf_rig::puppet_base()` is the asset as the FILE stores it, and
        // testing that alone tests a configuration nothing runs. The live
        // rig comes from `RigGeometry::from_skeleton`, whose
        // `root_rotation` is `hips_root_rotation()` — which carries the
        // gallery's 180-degree `--character-yaw-correction`.
        //
        // That correction flips the facing, so the two disagree about the
        // one quantity this test is about:
        //
        // ```text
        //   parsed file   ankle -> toe  =>  +Z   (facing_sign -1)
        //   as rendered   ankle -> toe  =>  -Z   (facing_sign +1)
        // ```
        //
        // An earlier version of this test used the parsed rig, went green,
        // and the character still walked on backward knees — because the
        // sign it was validating was the opposite of the one the game uses.
        // Measured live to settle it: the rendered `ankle -> toe` z is
        // **-0.147**.
        let corrected = {
            let mut rig = gltf_rig::puppet_base();
            rig.root_rotation =
                Quat::from_rotation_y(std::f32::consts::PI) * rig.root_rotation;
            rig
        };

        // # Measured by the fold, not by the knee's offset
        //
        // `knee_forward_offset` is the intuitive measurement — which side
        // of the hip-to-ankle line the knee sits on — and it is the right
        // one when the leg has a real bend. It is NOT reliable near full
        // extension, where the knee lies on that line by definition and the
        // small residual is dominated by the hip's lateral placement.
        //
        // That cost a wrong diagnosis. The synthetic rig reads `-0.017` at
        // 99.2% extension in late stance, which looked like a second,
        // smaller backward-knee defect and was filed as one. Measuring the
        // fold at those same phases gives `-0.34`: solidly human. The knee
        // was always bending correctly and the instrument was wrong.
        //
        // `knee_fold_direction` compares the two segment DIRECTIONS, so it
        // stays meaningful however extended the leg is — which is what lets
        // this assert on every phase of both rigs with no exemption and no
        // tolerance.
        for (label, rig) in [
            ("synthetic", RigGeometry::default()),
            ("as-rendered", corrected),
        ] {
            let base = stance_on_rig(&LocalPose::REST, DEFAULT_KNEE_FLEX, &rig);

            for params in [GaitParams::default(), GaitParams::running()] {
                for i in 0..32 {
                    let phase = i as f32 / 32.0;
                    let pose = walk_pose_on(phase, &params, &base, &rig);

                    for side in [Side::Left, Side::Right] {
                        let fold = rig.knee_fold_direction(&pose, side);

                        assert!(
                            fold < 0.0,
                            "{label} {side:?} at phase {phase:.3}: the shin folds \
                             {fold} along the rig's own forward relative to the \
                             thigh — positive is a knee bending backward, which no \
                             human leg does (leg at {:.1}% extension)",
                            leg_extension_fraction(&pose, &rig, side) * 100.0,
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn the_vertical_amplitudes_scale_with_the_rig_not_with_metres() {
        // `hip_dip` and `vertical_bob` are fractions of leg length. Every
        // other quantity in `GaitParams` is already rig-independent (the
        // angles are angles); these two were authored in metres against the
        // synthetic rig and did not scale.
        //
        // The consequence on a real rig: the body's vertical motion is
        // sized for a 0.49 m leg while the leg is 0.888 m, so the hips move
        // roughly half as far as the gait intends relative to the legs
        // carrying them — and the foot's vertical swing goes out of scale
        // with the ground it is meant to be planting on.
        use crate::character::anim::gltf_rig;
        use crate::character::anim::rig::RigGeometry;

        let params = GaitParams::default();
        let synthetic = RigGeometry::default();
        let real = gltf_rig::puppet_base();

        let synthetic_leg = leg_length_of(&synthetic);
        let real_leg = leg_length_of(&real);

        // The fixture this test is only meaningful on: two rigs whose legs
        // genuinely differ.
        assert!(
            (synthetic_leg - authored_leg_length()).abs() < 1.0e-6,
            "the synthetic rig IS the authored reference, but measures {synthetic_leg}",
        );
        assert!(
            real_leg > synthetic_leg * 1.5,
            "the real rig's leg ({real_leg}) should be far longer than the \
             synthetic one's ({synthetic_leg}), or this test proves nothing",
        );

        // On the authored rig the metre values are exactly what they always
        // were — this is what keeps every other test in this module valid.
        assert!(
            (params.hip_dip_metres(&synthetic) - 0.06).abs() < 1.0e-6,
            "the synthetic hip dip moved from its authored 0.06 m to {}",
            params.hip_dip_metres(&synthetic),
        );
        assert!(
            (params.vertical_bob_metres(&synthetic) - 0.025).abs() < 1.0e-6,
            "the synthetic vertical bob moved from its authored 0.025 m to {}",
            params.vertical_bob_metres(&synthetic),
        );

        // And on the real rig they scale in proportion to the leg.
        let ratio = real_leg / synthetic_leg;
        assert!(
            (params.hip_dip_metres(&real) - 0.06 * ratio).abs() < 1.0e-6,
            "the real rig's hip dip is {} but should be {} — {ratio:.3}x the \
             authored 0.06 m",
            params.hip_dip_metres(&real),
            0.06 * ratio,
        );

        // The property that actually matters, stated directly: the hips
        // move the same FRACTION of the leg on both rigs.
        for phase in [0.0f32, 0.125, 0.25, 0.5, 0.75] {
            let s = hip_height_offset_on(phase, &params, &synthetic) / synthetic_leg;
            let r = hip_height_offset_on(phase, &params, &real) / real_leg;
            assert!(
                (s - r).abs() < 1.0e-6,
                "at phase {phase} the hips drop {s} of a leg on the synthetic rig \
                 and {r} on the real one — the motion does not scale",
            );
        }
    }

    #[test]
    fn the_hips_drop_twice_per_cycle_not_once() {
        // The classic harmonic error. The hips dip at each midstance — once
        // per footfall, so twice per cycle. At cycle rate it reads as a limp.
        let params = params();

        let mut minima = 0;
        let mut previous = hip_height_offset(-0.5 / SAMPLES as f32, &params);
        let mut current = hip_height_offset(0.0, &params);

        for i in 1..=SAMPLES {
            let next = hip_height_offset(i as f32 / SAMPLES as f32, &params);

            if current < previous && current <= next {
                minima += 1;
            }

            previous = current;
            current = next;
        }

        assert_eq!(minima, 2, "expected two dips per cycle, found {minima}");
    }

    #[test]
    fn the_hips_never_rise_above_their_standing_height() {
        // The offset is a DROP. A positive value would push the character up
        // off its own legs, which no amount of IK can then reach the ground
        // from.
        let params = params();

        for i in 0..SAMPLES {
            let offset = hip_height_offset(i as f32 / SAMPLES as f32, &params);
            assert!(
                offset <= 1.0e-6,
                "at phase {} the hips rose by {offset}",
                i as f32 / SAMPLES as f32,
            );
        }
    }

    // -----------------------------------------------------------------
    // The pose as a whole
    // -----------------------------------------------------------------

    #[test]
    fn the_arms_counter_swing_the_legs() {
        // The strongest natural-gait cue after the legs. Arms swinging WITH
        // the same-side leg is an instantly readable wrongness.
        //
        // This used to check only that the hands moved in OPPOSITE
        // directions at phase 0.25 — which an arm swing timed a quarter
        // cycle wrong also satisfies, and which a correctly timed,
        // forward-biased swing need not (both hands sit slightly forward
        // there). Now: each hand is ahead exactly when the OPPOSITE foot is,
        // on the synthetic rig here and on the real one in
        // `each_hand_swings_forward_with_the_opposite_foot`.
        let base = base();
        let params = params();
        let rig = crate::character::anim::rig::RigGeometry::default();
        let ahead = |pose: &LocalPose, bone| {
            crate::character::anim::rig::offset_from(pose, &rig, Bone::Hips, bone).dot(rig.forward())
        };

        // The right foot lands at 0.5; the left at 0.
        for (phase, forward_hand, back_hand) in
            [(0.5, Bone::LeftHand, Bone::RightHand), (0.0, Bone::RightHand, Bone::LeftHand)]
        {
            let pose = walk_pose(phase, &params, &base);
            let forward = ahead(&pose, forward_hand) - ahead(&base, forward_hand);
            let back = ahead(&pose, back_hand) - ahead(&base, back_hand);
            assert!(
                forward > back,
                "at phase {phase} the {} should lead the {}: {forward:.3} against {back:.3}",
                forward_hand.name(),
                back_hand.name(),
            );
        }
    }

    /// The real character as the gallery walks it: `puppet_base` in its
    /// relaxed stance, arms hanging at the sides.
    fn real_arms() -> (LocalPose, GaitParams, crate::character::anim::rig::RigGeometry) {
        use crate::character::anim::stance::{stance_on_rig, DEFAULT_KNEE_FLEX};
        let rig = crate::character::anim::gltf_rig::puppet_base();
        let stood =
            stance_on_rig(&crate::character::anim::poses::relaxed_stand(), DEFAULT_KNEE_FLEX, &rig);
        (stood, GaitParams::default(), rig)
    }

    /// How far a joint sits ahead of the hips along the rig's forward.
    fn ahead(pose: &LocalPose, rig: &crate::character::anim::rig::RigGeometry, bone: Bone) -> f32 {
        crate::character::anim::rig::offset_from(pose, rig, Bone::Hips, bone).dot(rig.forward())
    }

    /// Elbow flexion: the angle between upper arm and forearm.
    fn elbow_flexion(
        pose: &LocalPose,
        rig: &crate::character::anim::rig::RigGeometry,
        arm: Bone,
        forearm: Bone,
        hand: Bone,
    ) -> f32 {
        use crate::character::anim::rig::offset_from;
        let upper = offset_from(pose, rig, arm, forearm);
        let lower = offset_from(pose, rig, forearm, hand);
        upper.angle_between(lower)
    }

    #[test]
    fn each_hand_swings_forward_with_the_opposite_foot() {
        // Signed, not just "opposite": swinging each arm with its OWN leg
        // also moves the hands in opposite directions.
        let (stood, params, rig) = real_arms();
        // The right foot lands at 0.5, furthest ahead.
        let pose = walk_pose_on(0.5, &params, &stood, &rig);
        let left = ahead(&pose, &rig, Bone::LeftHand) - ahead(&stood, &rig, Bone::LeftHand);
        let right = ahead(&pose, &rig, Bone::RightHand) - ahead(&stood, &rig, Bone::RightHand);
        assert!(
            left > 0.02 && right < -0.02,
            "with the right foot forward the left hand should be ahead and the right behind: \
             {left:.3} and {right:.3} m",
        );
    }

    #[test]
    fn a_walking_arm_swings_further_forward_than_back() {
        let (stood, params, rig) = real_arms();
        let rest = ahead(&stood, &rig, Bone::LeftHand);
        let reach: Vec<f32> = (0..64)
            .map(|i| ahead(&walk_pose_on(i as f32 / 64.0, &params, &stood, &rig), &rig, Bone::LeftHand) - rest)
            .collect();
        let forward = reach.iter().copied().fold(f32::MIN, f32::max);
        let back = -reach.iter().copied().fold(f32::MAX, f32::min);
        assert!(
            forward > back * 1.3,
            "a walking hand swings further in front than behind: {:.0} mm ahead, {:.0} mm behind",
            forward * 1000.0,
            back * 1000.0,
        );
    }

    #[test]
    fn a_walking_arm_swings_back_from_the_shoulder_and_forward_from_the_elbow() {
        // Murray, Sepic & Barnard (1967), 30 men at their free pace of
        // 1.54 m/s: the upper arm from 8 degrees in front of the vertical to
        // 24 behind it, the elbow from 17 to 47 degrees. The walk once had
        // the upper arm 28 forward and 10 back on an elbow that moved 10
        // degrees, a march. Their standard deviations are 6-11 degrees.
        use crate::character::anim::rig::offset_from;
        let (stood, _, rig) = real_arms();
        let params = GaitParams::walking_on(1.54, &rig);
        let standing_upper = offset_from(&stood, &rig, Bone::LeftArm, Bone::LeftForeArm);
        let (mut forward, mut back, mut straightest, mut most_bent) = (f32::MIN, f32::MIN, f32::MAX, f32::MIN);
        for i in 0..64 {
            let pose = walk_pose_on(i as f32 / 64.0, &params, &stood, &rig);
            let upper = offset_from(&pose, &rig, Bone::LeftArm, Bone::LeftForeArm);
            // Signed by whether the elbow went ahead of where it stands.
            let ahead_of_standing = ahead(&pose, &rig, Bone::LeftForeArm) - ahead(&stood, &rig, Bone::LeftForeArm);
            let angle = standing_upper.angle_between(upper).to_degrees();
            if ahead_of_standing > 0.0 {
                forward = forward.max(angle);
            } else {
                back = back.max(angle);
            }
            let elbow = elbow_flexion(&pose, &rig, Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand).to_degrees();
            straightest = straightest.min(elbow);
            most_bent = most_bent.max(elbow);
        }
        assert!(
            (forward - 8.0).abs() < 4.0 && (back - 24.0).abs() < 4.0,
            "the upper arm should swing 8 degrees forward and 24 back: {forward:.1} and {back:.1}",
        );
        assert!(
            (straightest - 17.0).abs() < 4.0 && (most_bent - 47.0).abs() < 4.0,
            "the elbow should fold from 17 to 47 degrees: {straightest:.1} to {most_bent:.1}",
        );
    }

    #[test]
    fn a_walking_elbow_never_straightens_past_its_relaxed_bend() {
        let (stood, params, rig) = real_arms();
        let relaxed = elbow_flexion(&stood, &rig, Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand);
        let straightest = (0..64)
            .map(|i| {
                let pose = walk_pose_on(i as f32 / 64.0, &params, &stood, &rig);
                elbow_flexion(&pose, &rig, Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand)
            })
            .fold(f32::MAX, f32::min);
        assert!(
            straightest >= relaxed - 1.0e-3,
            "the elbow straightened to {:.1} degrees against a relaxed {:.1}",
            straightest.to_degrees(),
            relaxed.to_degrees(),
        );
    }

    #[test]
    fn a_walking_arm_lags_the_leg_it_answers() {
        // An arm is a pendulum driven from the shoulder, so its forward peak
        // trails the opposite foot's.
        let (stood, params, rig) = real_arms();
        let peak_phase = |bone: Bone| {
            (0..256)
                .map(|i| i as f32 / 256.0)
                .max_by(|a, b| {
                    let at = |p| ahead(&walk_pose_on(p, &params, &stood, &rig), &rig, bone);
                    at(*a).total_cmp(&at(*b))
                })
                .unwrap()
        };
        let (hand, foot) = (peak_phase(Bone::LeftHand), peak_phase(Bone::RightFoot));
        let lag = (hand - foot).rem_euclid(1.0);
        assert!(
            (0.02..0.15).contains(&lag),
            "the left hand should peak a little after the right foot: {hand:.3} against {foot:.3}",
        );
    }

    #[test]
    fn there_is_a_visible_front_back_leg_split_at_mid_swing() {
        // The superseded module's own recorded finding: without a trailing
        // stance leg, screenshots showed no leg split at all and the walk
        // read as a shuffle. Asserted in metres of real separation.
        let base = base();
        let params = params();

        // Mid-swing for the left leg.
        let phase = params.duty_factor + (1.0 - params.duty_factor) * 0.5;
        let positions = forward_kinematics(&walk_pose(phase, &params, &base));

        let separation =
            (positions[Bone::LeftFoot].z - positions[Bone::RightFoot].z).abs();

        assert!(
            separation > 0.15,
            "the feet are only {separation} m apart at mid-swing — no visible split",
        );
    }

    #[test]
    fn the_swinging_foot_lifts_off_the_ground() {
        // A foot that does not lift is a foot that drags. The superseded
        // module's own recorded failure was landing short; this is the
        // complementary check.
        //
        // On real proportions, for the same reason as
        // `the_swinging_foot_clears_the_planted_one_by_a_real_margin`: the
        // lift comes from knee flexion, which the synthetic rig's 0.07 m
        // stub cannot produce.
        use crate::character::anim::rig::forward_kinematics_on;

        let base = base();
        let params = params();
        let rig = super::shape::real_proportions();

        let planted = forward_kinematics_on(&base, &rig)[Bone::LeftFoot].y;

        let mut highest = f32::MIN;
        for i in 0..SAMPLES {
            let progress = i as f32 / SAMPLES as f32;
            let phase = params.duty_factor + (1.0 - params.duty_factor) * progress;
            let y = forward_kinematics_on(&walk_pose(phase, &params, &base), &rig)
                [Bone::LeftFoot]
                .y;
            highest = highest.max(y);
        }

        assert!(
            highest > planted + 0.02,
            "the swinging foot peaks at {highest} against a planted {planted} — it is \
             dragging",
        );
    }

    #[test]
    fn a_zero_amplitude_gait_is_the_base_pose() {
        // The identity case: with every amplitude at zero the walk must be
        // exactly the pose it was given, or the gait is injecting a bias.
        let base = base();
        let params = GaitParams {
            stride_length: 0.0,
            duty_factor: 0.6,
            thigh_swing: 0.0,
            knee_swing_flex: 0.0,
            knee_stance_flex: 0.0,
            stance_trail: 0.0,
            ankle_range: 0.0,
            arm_swing: 0.0,
            elbow_bend: 0.0,
            elbow_carry: 0.0,
            hip_dip: 0.0,
            vertical_bob: 0.0,
            // The measured curves hold the recording's mean posture even at
            // zero excursion, so identity is a property of the authored ones.
            curves: LegCurves::Authored,
        };

        for i in 0..20 {
            let phase = i as f32 / 20.0;
            let pose = walk_pose(phase, &params, &base);

            for &bone in Bone::ALL.iter() {
                assert!(
                    pose.rotation(bone).abs_diff_eq(base.rotation(bone), 1.0e-6),
                    "at phase {phase}, {} drifted from the base pose",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn the_pose_never_produces_a_non_finite_rotation() {
        let base = base();
        let params = params();

        for phase in [0.0, 0.5, 1.0, -3.7, 12.3, f32::MIN, f32::MAX] {
            let pose = walk_pose(phase, &params, &base);
            for &bone in Bone::ALL.iter() {
                assert!(
                    pose.rotation(bone).is_finite(),
                    "phase {phase} produced a non-finite rotation on {}",
                    bone.name(),
                );
            }
        }
    }

    #[test]
    fn the_gait_layers_onto_an_authored_pose_rather_than_replacing_it() {
        // A walking character must still be able to hold something, look
        // somewhere, or carry an authored upper-body pose. The gait touches
        // legs and arms; everything else must pass through untouched.
        let mut authored = base();
        let tilt = Quat::from_axis_angle(Vec3::Y, 0.4);
        authored.set_rotation(Bone::Head, tilt);
        authored.set_rotation(Bone::Spine2, tilt);

        let pose = walk_pose(0.3, &params(), &authored);

        for bone in [Bone::Head, Bone::Spine2] {
            assert!(
                pose.rotation(bone).abs_diff_eq(tilt, 1.0e-6),
                "{} lost its authored rotation",
                bone.name(),
            );
        }
    }
}

#[cfg(test)]
mod shape {
    //! The gait's measured shape, pinned.
    //!
    //! The tests above assert *properties* — monotonicity, continuity,
    //! ordering. These pin the actual numbers the default parameters
    //! produce, so a change that alters the walk's character has to say so
    //! out loud rather than drifting silently past a set of inequalities.
    //!
    //! They pin the HAND-SHAPED curves (`GaitParams::authored_walk`), which
    //! the run still uses. The measured walk is pinned against the recording
    //! it replays, on the real rig, in `locomotion`'s tests: posed on the
    //! synthetic rig, whose shin is a 0.07 m stub, recorded angles describe
    //! no leg at all.

    use super::*;
    use crate::character::anim::rig::forward_kinematics;
    use crate::character::anim::stance::stance;

    fn setup() -> (LocalPose, GaitParams) {
        (stance(&LocalPose::REST), GaitParams::authored_walk())
    }

    #[test]
    fn the_knee_folds_hard_in_swing_and_stays_nearly_straight_in_stance() {
        let (_, p) = setup();

        // Measured at the default parameters, as the gait's own
        // contribution — the base stance adds another 0.16 rad on top.
        // Stance sits at `knee_stance_flex` (0.20) at footfall and yields to
        // 0.32 rad at midstance under load; swing peaks at 0.90.
        //
        // Composed with the base, that is 18 degrees in stance and 57 in
        // swing, against published walking kinematics of 15-20 and 60-65.
        let stance_peak = (0..=100)
            .map(|i| knee_flex(LegPhase::Stance { progress: i as f32 / 100.0 }, &p))
            .fold(0.0f32, f32::max);
        let swing_peak = (0..=100)
            .map(|i| knee_flex(LegPhase::Swing { progress: i as f32 / 100.0 }, &p))
            .fold(0.0f32, f32::max);

        assert!(
            (stance_peak - 0.320).abs() < 0.01,
            "stance knee peak moved to {stance_peak} rad (was 0.320)",
        );
        assert!(
            (swing_peak - 0.900).abs() < 0.01,
            "swing knee peak moved to {swing_peak} rad (was 0.900)",
        );
    }

    /// A rig with a real rig's leg proportions.
    ///
    /// Anything measuring foot HEIGHT has to use this. The synthetic rig
    /// leaves `LeftLeg -> LeftFoot` as a 0.07 m stub, so knee flexion —
    /// which is how a real gait lifts its swing foot — moves the sole
    /// almost nowhere, and clearance on the synthetic rig comes out of the
    /// hip angle by accident instead.
    ///
    /// The real rig's leg LENGTHS on an otherwise-synthetic rig.
    ///
    /// Lengths are taken from the parsed asset rather than transcribed, so
    /// they cannot drift; the bind rotations stay at identity deliberately.
    /// That combination is what most of these tests want: a leg whose
    /// segments are real (so knee flexion is visible) posed in this crate's
    /// own upright convention (so `y` reads as height without unpicking a
    /// Z-up correction first).
    ///
    /// For anything about the foot's ORIENTATION use [`real_bind_pose`],
    /// which is the whole asset.
    pub(super) fn real_proportions() -> crate::character::anim::rig::RigGeometry {
        crate::character::anim::gltf_rig::real_leg_lengths()
    }

    /// The real rig, parsed from `puppet_base.gltf`.
    ///
    /// Use this for anything about ORIENTATION. It was previously a
    /// hand-transcribed helper — bind rotations read out of the glTF once
    /// and typed in — which closed the gap on the day it was written and
    /// then drifted: a foot the real rig binds at -69.8 degrees sat flat in
    /// the test and pitched nose-down in the game, reported from a
    /// screenshot while every test passed.
    ///
    /// Parsing the asset removes the transcription step, so the test rig and
    /// the shipped rig cannot disagree. See [`super::super::gltf_rig`].
    fn real_bind_pose() -> crate::character::anim::rig::RigGeometry {
        crate::character::anim::gltf_rig::puppet_base()
    }

    #[test]
    fn the_swinging_foot_clears_the_planted_one_by_a_real_margin() {
        // A foot that clears by only a centimetre catches on everything.
        //
        // Measured on REAL proportions, because that is where the clearance
        // actually comes from: a real gait lifts its swing foot with knee
        // flexion, and the synthetic rig's 0.07 m stub cannot express that.
        // An earlier version measured on the synthetic rig and was really
        // checking a side effect of the hip angle — which is why centring
        // the thigh curve appeared to break it.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, p) = setup();
        let rig = real_proportions();

        let mut swing_peak = f32::MIN;
        let mut stance_min = f32::MAX;

        for i in 0..=200 {
            let phase = i as f32 / 200.0;
            let y =
                forward_kinematics_on(&walk_pose(phase, &p, &base), &rig)[Bone::LeftFoot].y;

            if leg_phase(phase, p.duty_factor).is_stance() {
                stance_min = stance_min.min(y);
            } else {
                swing_peak = swing_peak.max(y);
            }
        }

        let clearance = swing_peak - stance_min;
        assert!(
            clearance > 0.05,
            "only {clearance} m of clearance between the swinging and planted foot",
        );
    }

    #[test]
    fn the_knee_bend_is_visible_on_a_rig_with_a_real_shin() {
        // The synthetic T-pose rig cannot show knee flexion, and finding
        // that out took several wrong diagnoses.
        //
        // Its leg segments are shifted a joint from every real rig's: it
        // puts 0.45 into `Hips -> LeftUpLeg` and leaves `LeftLeg ->
        // LeftFoot` as a 0.07 m stub, where `puppet_base.gltf` has a 0.429 m
        // femur and a 0.459 m shin. So 42 degrees of knee flexion moves the
        // ankle 0.329 m on a real rig and the sole 0.050 m on the synthetic
        // one — 6x — and a perfectly correct walk renders as a straight leg
        // in any synthetic-rig preview.
        //
        // This asserts the flexion produces real displacement once the
        // segment below it has a real length, so the gait is pinned against
        // the rig it actually ships on.
        use crate::character::anim::rig::{forward_kinematics_on, RigGeometry};

        let (base, p) = setup();

        let mut real = RigGeometry::default();
        // A real rig's proportions: the shin carries most of the lower leg.
        real.offsets[Bone::LeftUpLeg] = Vec3::new(-0.1, -0.11, 0.0);
        real.offsets[Bone::LeftLeg] = Vec3::new(0.0, -0.43, 0.0);
        real.offsets[Bone::LeftFoot] = Vec3::new(0.0, -0.46, 0.0);

        // Peak knee flexion, mid-swing.
        let flexed = forward_kinematics_on(&walk_pose(0.75, &p, &base), &real);
        // Nearly straight, mid-stance.
        let straight = forward_kinematics_on(&walk_pose(0.25, &p, &base), &real);

        let hip = |k: &crate::character::anim::rig::BoneSet<Vec3>| k[Bone::LeftUpLeg];
        let ankle = |k: &crate::character::anim::rig::BoneSet<Vec3>| k[Bone::LeftFoot];

        let flexed_reach = ankle(&flexed).distance(hip(&flexed));
        let straight_reach = ankle(&straight).distance(hip(&straight));

        assert!(
            straight_reach - flexed_reach > 0.05,
            "a flexed knee should shorten the hip-to-ankle reach by a visible \
             amount, but it went from {straight_reach} to {flexed_reach}",
        );

        // And the "straight" sample is not actually straight — Phase 2's
        // whole point. Even at its most extended the stance leg keeps a real
        // bend, so the IK always has a solution space.
        let straight = 0.43 + 0.46;
        assert!(
            straight_reach < straight * 0.99,
            "the stance leg reaches {straight_reach} of a {straight} m leg — it is \
             at the singularity",
        );
    }

    #[test]
    fn the_feet_stay_centred_under_the_body_over_a_whole_cycle() {
        // A walk keeps its feet balanced around the body: averaged over a
        // cycle, the foot pattern's centre should sit near the hips. A
        // pattern offset bodily forward or back is a character leaning over
        // trailing feet, which reads as falling rather than walking —
        // observed live at ~0.25 m of backward offset.
        //
        // Sampled across the cycle rather than from one frame, because a
        // single snapshot cannot distinguish a real offset from the moment
        // the cycle happened to be caught at.
        let (base, p) = setup();

        let mut sum = 0.0;
        let samples = 200;

        for i in 0..samples {
            let phase = i as f32 / samples as f32;
            let k = forward_kinematics(&walk_pose(phase, &p, &base));

            let hips = k[Bone::Hips].z;
            let feet = (k[Bone::LeftFoot].z + k[Bone::RightFoot].z) * 0.5;

            sum += feet - hips;
        }

        let synthetic_offset = sum / samples as f32;

        // And on a real rig's proportions, which is where this actually
        // failed: the same rotations act on different segment lengths, so
        // the synthetic rig reported 0.065 m of offset where the real one
        // had 0.228 m. A bound tuned against the synthetic number passed a
        // gait whose feet trailed half a stride behind the body.
        use crate::character::anim::rig::forward_kinematics_on;

        let real = real_proportions();
        let mut real_sum = 0.0;
        for i in 0..samples {
            let phase = i as f32 / samples as f32;
            let k = forward_kinematics_on(&walk_pose(phase, &p, &base), &real);
            real_sum +=
                (k[Bone::LeftFoot].z + k[Bone::RightFoot].z) * 0.5 - k[Bone::Hips].z;
        }
        let real_offset = real_sum / samples as f32;

        // The REAL rig is the one that has to be balanced — it is what
        // ships, and the centring solve is calibrated against its
        // proportions.
        assert!(
            real_offset.abs() < p.stride_length * 0.06,
            "on real proportions the feet average {real_offset} m from under the \
             hips, against a {} m stride — the body is not balanced over its feet",
            p.stride_length,
        );

        // The synthetic rig cannot be balanced by the same correction and is
        // not expected to be: its lower leg is a 0.07 m stub, so the knee
        // contributes almost nothing to foot position there while it
        // contributes a great deal on a real rig. One thigh offset cannot
        // zero both. This bound just catches a gross regression.
        assert!(
            synthetic_offset.abs() < p.stride_length * 0.25,
            "even allowing for its proportions, the synthetic rig's feet average \
             {synthetic_offset} m from under the hips",
        );
    }

    #[test]
    fn the_knee_never_approaches_the_reach_singularity() {
        // THE Phase 2 property. A leg at full extension has no bend
        // direction, so the IK's next solve can flip the knee either way and
        // any numerical error pushes the chain past straight.
        //
        // Measured before this was enforced: the gait demanded 0.8825 m of
        // an 0.8900 m leg — 0.8% headroom — for the WHOLE stance phase, not
        // just an instant. The cause was `knee_stance_flex` at 0.10 rad,
        // which sounds like a real bend and is not: headroom grows with the
        // square of the angle, so 0.10 buys 1.1 mm where 0.30 buys 10 mm.
        let (_, params) = setup();
        let base = crate::character::anim::stance::DEFAULT_KNEE_FLEX;

        for i in 0..=720 {
            let phase = i as f32 / 720.0;

            // The COMPOSED flex — what the leg actually has. The gait layers
            // onto a stance that already carries its own bend, so checking
            // the gait's contribution alone would both understate the real
            // margin and let a gait that relies entirely on the base pose
            // pass.
            let flex = base + knee_flex(leg_phase(phase, params.duty_factor), &params);

            assert!(
                flex >= MINIMUM_KNEE_FLEX,
                "at phase {phase} the knee is at {flex} rad, inside the \
                 {MINIMUM_KNEE_FLEX} rad reserved against the reach singularity",
            );
        }
    }

    // -----------------------------------------------------------------
    // The run
    // -----------------------------------------------------------------

    #[test]
    fn a_run_has_a_flight_phase_and_a_walk_does_not() {
        // THE structural difference, and it is not speed: a fast walk is
        // still a walk. Below a 0.5 duty factor the two stance windows stop
        // overlapping, so there is a moment with no foot down at all.
        let run = GaitParams::running();
        let walk = GaitParams::default();

        let count_airborne = |p: &GaitParams| {
            (0..720)
                .filter(|i| {
                    let phase = *i as f32 / 720.0;
                    !leg_phase(phase, p.duty_factor).is_stance()
                        && !leg_phase(phase + 0.5, p.duty_factor).is_stance()
                })
                .count()
        };

        assert!(
            count_airborne(&run) > 0,
            "a run must leave the ground at some point in the cycle",
        );
        assert_eq!(
            count_airborne(&walk),
            0,
            "a walk must never leave the ground — that is what makes it a walk",
        );
    }

    #[test]
    fn the_run_keeps_the_leg_clear_of_its_singularity() {
        // The constraint a longer stride actually runs into. Checked the
        // same way the walk's own reach test does, because the run's 0.75 m
        // stride is 1.7x the walk's and the leg is the same length.
        let params = GaitParams::running();
        let base = crate::character::anim::stance::DEFAULT_KNEE_FLEX;

        for i in 0..=720 {
            let phase = i as f32 / 720.0;
            let flex = base + knee_flex(leg_phase(phase, params.duty_factor), &params);

            assert!(
                flex >= MINIMUM_KNEE_FLEX,
                "at phase {phase} a running knee is at {flex} rad, inside the \
                 {MINIMUM_KNEE_FLEX} reserved against the reach singularity",
            );
        }

        // Measured: the run demands 96.8% of the leg against the walk's
        // 98.4% — MORE headroom, not less, despite a 1.7x longer stride.
        // The deeper stance knee (0.35 against 0.20) more than pays for it,
        // which is why a run can afford the stride at all.
        //
        // Worth pinning rather than left as an inequality: if a future
        // change to the run's parameters erodes that margin, the walk is
        // the wrong thing to compare against and this says so.
        let fraction = peak_reach_fraction(&params, base, 0.43, 0.46);
        let walking = peak_reach_fraction(&GaitParams::default(), base, 0.43, 0.46);

        assert!(
            fraction < 0.98,
            "the run demands {:.1}% of the leg's straight length",
            fraction * 100.0,
        );
        assert!(
            fraction < walking,
            "the run should sit further from the singularity than the walk \
             ({:.1}% against {:.1}%) — its deeper stance knee is what buys the \
             longer stride",
            fraction * 100.0,
            walking * 100.0,
        );
    }

    #[test]
    fn a_run_lifts_its_feet_higher_than_a_walk() {
        // A run clears the ground by more, which is both real and necessary
        // — the stride is longer, so the swing foot has further to travel
        // in less of the cycle.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, _) = setup();
        let rig = real_proportions();

        let peak_lift = |p: &GaitParams| {
            (0..=400)
                .map(|i| {
                    let phase = i as f32 / 400.0;
                    forward_kinematics_on(&walk_pose(phase, p, &base), &rig)
                        [Bone::LeftFoot]
                        .y
                })
                .fold(f32::MIN, f32::max)
        };

        let run = peak_lift(&GaitParams::running());
        let walk = peak_lift(&GaitParams::authored_walk());

        assert!(
            run > walk,
            "a run should lift its feet higher than a walk, got {run} against {walk}",
        );
    }

    #[test]
    fn the_run_shares_every_structural_property_of_the_walk() {
        // The invariants are properties of the CURVES, not of one parameter
        // set — so they must hold for a run as well, and a run exercises
        // shapes (a flight phase, a deeper knee) the walk never reaches.
        let params = GaitParams::running();
        let (base, _) = setup();

        for i in 0..720 {
            let phase = i as f32 / 720.0;
            let leg = leg_phase(phase, params.duty_factor);

            // A knee never hyperextends.
            assert!(
                knee_flex(leg, &params) >= 0.0,
                "at phase {phase} the knee bends backward",
            );

            // Bone lengths are preserved.
            let positions = forward_kinematics(&walk_pose(phase, &params, &base));
            for &bone in Bone::ALL.iter() {
                let Some(parent) = bone.parent() else { continue };
                let rest = bone.t_pose_offset().length();
                let posed = (positions[bone] - positions[parent]).length();
                assert!(
                    (posed - rest).abs() < 1.0e-5,
                    "at phase {phase}, {} is {posed} against a rest length of {rest}",
                    bone.name(),
                );
            }
        }

        // And the two legs still mirror.
        for i in 0..200 {
            let phase = i as f32 / 200.0;
            let now = walk_pose(phase, &params, &base);
            let later = walk_pose(phase + 0.5, &params, &base);

            assert!(
                now.rotation(Bone::LeftUpLeg)
                    .abs_diff_eq(later.rotation(Bone::RightUpLeg), 1.0e-5),
                "the run's legs do not mirror at phase {phase}",
            );
        }
    }

    #[test]
    fn the_leg_keeps_real_headroom_on_real_proportions() {
        // The same property in the units that matter to the IK: how much of
        // the leg's own length the gait actually demands.
        let (_, params) = setup();

        let fraction = peak_reach_fraction(
            &params,
            crate::character::anim::stance::DEFAULT_KNEE_FLEX,
            0.43,
            0.46,
        );

        assert!(
            fraction < 0.99,
            "the gait demands {:.1}% of the leg's straight length — the IK has \
             almost no solution space left",
            fraction * 100.0,
        );

        // And not so little that the walk reads as a crouch.
        assert!(
            fraction > 0.9,
            "the gait only uses {:.1}% of the leg — that is a crouch, not a walk",
            fraction * 100.0,
        );
    }

    #[test]
    fn a_positive_rotation_about_the_knee_axis_swings_a_leg_forward() {
        // The sign convention, pinned by measurement rather than by a
        // comment — because the comment was wrong, and the walk cycle built
        // on it ran BACKWARD: the planted foot travelled toward `-Z`
        // relative to the hips, which is a body reversing over its own feet.
        //
        // Derivable too: a leg hangs along `-Y`, and
        // `Rx(t) * (0,-L,0) = (0, -L*cos t, -L*sin t)`, so positive `t`
        // drives `z` negative — and `-Z` is forward.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, _) = setup();
        let rig = real_proportions();

        let mut probe = base;
        probe.set_rotation(Bone::LeftUpLeg, Quat::from_axis_angle(KNEE_AXIS, 0.3));

        let rest = forward_kinematics_on(&base, &rig)[Bone::LeftFoot].z;
        let posed = forward_kinematics_on(&probe, &rig)[Bone::LeftFoot].z;

        assert!(
            posed < rest - 0.05,
            "a positive rotation about KNEE_AXIS must swing the foot FORWARD \
             (toward -Z), but it went from {rest} to {posed}",
        );
    }

    #[test]
    fn a_planted_foot_travels_backward_relative_to_the_body() {
        // The direction of travel, which is what "walking forward" means
        // mechanically: the body advances over a foot that stays put, so
        // relative to the body the planted foot moves BACKWARD (+Z).
        //
        // This is the test that would have caught the inverted sign
        // immediately. The earlier suite asserted angles and symmetry —
        // every one of which a backward walk satisfies perfectly.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, p) = setup();
        let rig = real_proportions();

        let relative = |phase: f32| {
            let k = forward_kinematics_on(&walk_pose(phase, &p, &base), &rig);
            k[Bone::LeftFoot].z - k[Bone::Hips].z
        };

        // Across stance, sampled inside it to avoid the seams.
        let early = relative(0.05);
        let late = relative(p.duty_factor - 0.05);

        assert!(
            late > early + 0.2,
            "through stance the planted foot should travel backward relative to the \
             body, but it went from {early} to {late} — the character is walking in \
             reverse",
        );
    }

    #[test]
    fn the_real_rig_stands_with_its_foot_pitched_nose_down() {
        // The measurement that used to need a screenshot.
        //
        // A ~26-degree nose-down foot was reported from the live rig, and
        // attributing it took a round trip through BRP because the test rig
        // could not reproduce an orientation it did not model. Parsed from
        // the asset, it can: this asserts the tilt IS there, in the rest
        // pose, before any gait runs.
        //
        // Pinned as a property of the RIG, not a bug in this module — see
        // `the_gait_does_not_pitch_the_foot_more_than_standing_does` for the
        // part `gait` owns. If the asset is ever re-authored with a flatter
        // foot, this fails and says so rather than leaving a stale comment
        // behind.
        use crate::character::anim::rig::forward_kinematics_on;

        let rig = real_bind_pose();
        let positions = forward_kinematics_on(&LocalPose::REST, &rig);

        let ankle = positions[Bone::LeftFoot];
        let toe = positions[Bone::LeftToeBase];

        let run = (toe.z - ankle.z).abs().max(1.0e-6);
        let pitch = ((ankle.y - toe.y) / run).atan().to_degrees();

        // 26.6 degrees from the parsed asset, against 26.1 measured on the
        // LIVE rig over BRP — a 0.5 degree agreement, which is what says
        // this test rig reproduces the running game rather than
        // approximating it.
        assert!(
            (pitch - 26.6).abs() < 2.0,
            "the rig's rest pose pitches the foot {pitch} degrees nose-down, against \
             the 26.6 this asset has carried — if it was re-authored, the gait's own \
             assertions may need revisiting",
        );
    }

    #[test]
    fn the_gait_does_not_pitch_the_foot_more_than_standing_does() {
        // A ~26-degree nose-down foot was reported from the live rig, and
        // measurement cleared the gait of it: STANDING STILL the real rig
        // pitches the foot 26.1 degrees nose-down, and walking is 22.2 — the
        // gait slightly IMPROVES it.
        //
        // The tilt is the rig's own bind pose. `relaxed_stand` contributes
        // only 4.6 degrees at `LeftFoot`; the rest is how the artist bound
        // the foot, and nothing levels it: `legik`'s normal alignment adds
        // only the ground's DIFFERENCE from level, deliberately, so that it
        // never flattens an authored pose. On flat ground it is a no-op by
        // construction.
        //
        // So this asserts the gait's own contribution, which is the part
        // this module owns.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, p) = setup();
        let rig = real_bind_pose();

        let pitch_of = |pose: &LocalPose| {
            let k = forward_kinematics_on(pose, &rig);
            let ankle = k[Bone::LeftFoot];
            let toe = k[Bone::LeftToeBase];
            let run = (toe.z - ankle.z).abs().max(1.0e-6);
            ((ankle.y - toe.y) / run).atan().to_degrees()
        };

        let standing = pitch_of(&base);

        let mut worst = 0.0f32;
        for i in 0..200 {
            let phase = i as f32 / 200.0;
            worst = worst.max((pitch_of(&walk_pose(phase, &p, &base)) - standing).abs());
        }

        assert!(
            worst < 25.0,
            "the gait swings the foot up to {worst} degrees away from its standing \
             {standing} — more roll than a walking foot does",
        );
    }

    #[test]
    fn the_gait_does_not_add_foot_pitch_beyond_the_rest_pose() {
        // The gait's own contribution to how the foot sits.
        //
        // Measured: the rest pose already drops the toe 0.020 m below the
        // ankle, and across the cycle the gait ranges from -0.015 (toe up,
        // heel strike) to +0.043 (toe down, toe-off) — which is the roll a
        // walking foot does.
        //
        // What it must NOT do is add a standing pitch of its own. A real
        // 21.8-degree nose-down tilt WAS observed live, and it comes from
        // the glTF rig's own bind pose rather than from here — the same
        // class of gap as the knee-visibility one, since `real_proportions`
        // models bone lengths but not a real rig's bind ROTATIONS.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, p) = setup();
        let rig = real_proportions();

        let drop_at = |pose: &LocalPose| {
            let k = forward_kinematics_on(pose, &rig);
            k[Bone::LeftFoot].y - k[Bone::LeftToeBase].y
        };

        let resting = drop_at(&base);

        let mut worst = 0.0f32;
        for i in 0..200 {
            let phase = i as f32 / 200.0;
            worst = worst.max((drop_at(&walk_pose(phase, &p, &base)) - resting).abs());
        }

        assert!(
            worst < 0.05,
            "the gait pitches the foot up to {worst} m away from its resting \
             {resting} m drop — more roll than a walking foot does",
        );
    }

    #[test]
    fn the_toe_is_never_driven_below_the_heel_through_stance() {
        // The foot's pitch, which has its own sign convention and got it
        // wrong once: the foot bone points FORWARD (`-Z`) while every other
        // bone here hangs DOWN (`-Y`), so a rotation about `KNEE_AXIS` lifts
        // the toe rather than swinging it forward.
        //
        // Written with the leg's convention, the toe pitched down at
        // footfall — measured live, the toe tip sat 0.02 m BELOW the ground
        // while the ankle rode 0.12 m high, and it was visible enough to be
        // reported as "toes look downward".
        //
        // Through most of stance the toe should sit at or below the ankle
        // (the foot is flat or rolling onto the ball), and never so far
        // below that the foot is standing on its toe tip.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, p) = setup();
        let rig = real_proportions();

        // Early stance: heel strike, toe up.
        let early = forward_kinematics_on(&walk_pose(0.02, &p, &base), &rig);
        assert!(
            early[Bone::LeftToeBase].y >= early[Bone::LeftFoot].y - 0.02,
            "at heel strike the toe ({}) should not be driven below the ankle ({})",
            early[Bone::LeftToeBase].y,
            early[Bone::LeftFoot].y,
        );

        // And the ankle should be low through stance — a foot pitched
        // steeply nose-down rides its ankle high off the ground.
        let mid = forward_kinematics_on(&walk_pose(0.25, &p, &base), &rig);
        let ankle_above_toe = mid[Bone::LeftFoot].y - mid[Bone::LeftToeBase].y;
        assert!(
            ankle_above_toe < 0.12,
            "mid-stance the ankle sits {ankle_above_toe} m above the toe — the foot \
             is pitched nose-down onto its toe tip",
        );
    }

    #[test]
    fn the_gait_stays_outside_the_ik_soft_clamp() {
        // The property that makes the headroom number mean something.
        //
        // `legik`'s soft extension clamp absorbs targets within a few
        // millimetres of full reach — it exists so an over-reaching target
        // degrades gracefully. But the GAIT's own pose should never be
        // inside that band: if it is, the IK is spending its correction
        // budget compensating for the animation rather than for the ground,
        // and the leg is permanently near-straight.
        //
        // Measured: 0.0143 m of headroom against a 0.005 m softening, so
        // 2.9x clear. Before Phase 2 it was 0.0075 m — inside twice the
        // band and heading for it.
        use crate::character::anim::legik::LegIkConfig;

        let (_, params) = setup();
        let (thigh, shin) = (0.43, 0.46);

        let fraction = peak_reach_fraction(
            &params,
            crate::character::anim::stance::DEFAULT_KNEE_FLEX,
            thigh,
            shin,
        );
        let headroom = (1.0 - fraction) * (thigh + shin);
        let softening = LegIkConfig::default().softening;

        assert!(
            headroom > softening * 2.0,
            "the gait leaves {headroom} m of reach headroom against the IK's \
             {softening} m softening band — the solver would be absorbing the \
             animation's own pose",
        );
    }

    #[test]
    fn more_knee_flexion_buys_more_headroom() {
        // The relationship the default is chosen against, asserted so the
        // reasoning cannot rot: a more bent knee reaches less far.
        let mut previous = 1.1f32;

        for flex in [0.05_f32, 0.10, 0.20, 0.30, 0.45] {
            let params = GaitParams { knee_stance_flex: flex, ..Default::default() };
            let fraction = peak_reach_fraction(
                &params,
                crate::character::anim::stance::DEFAULT_KNEE_FLEX,
                0.43,
                0.46,
            );

            assert!(
                fraction < previous,
                "flexing the knee from the previous step to {flex} rad should reduce \
                 the reach demand, but it went {previous} -> {fraction}",
            );
            previous = fraction;
        }
    }

    #[test]
    fn the_reach_fraction_is_measured_from_the_pose_not_assumed() {
        // Anchors `peak_reach_fraction`'s law-of-cosines shortcut against
        // real forward kinematics — two independent computations that must
        // agree, or one of them is wrong.
        use crate::character::anim::rig::forward_kinematics_on;

        let (base, p) = setup();
        let rig = real_proportions();

        let thigh = rig.offsets[Bone::LeftLeg].length();
        let shin = rig.offsets[Bone::LeftFoot].length();

        let mut measured = 0.0f32;
        for i in 0..400 {
            let phase = i as f32 / 400.0;
            let k = forward_kinematics_on(&walk_pose(phase, &p, &base), &rig);
            measured = measured
                .max(k[Bone::LeftFoot].distance(k[Bone::LeftUpLeg]) / (thigh + shin));
        }

        let predicted = peak_reach_fraction(
            &p,
            crate::character::anim::stance::DEFAULT_KNEE_FLEX,
            thigh,
            shin,
        );

        assert!(
            (measured - predicted).abs() < 0.01,
            "forward kinematics measures a peak reach fraction of {measured} where \
             `peak_reach_fraction` predicts {predicted}",
        );
    }

    #[test]
    fn a_degenerate_leg_does_not_divide_by_zero() {
        let (_, params) = setup();
        assert_eq!(peak_reach_fraction(&params, 0.16, 0.0, 0.0), 0.0);
        assert!(peak_reach_fraction(&params, 0.16, 0.0, 0.46).is_finite());
    }

    #[test]
    fn the_feet_separate_by_most_of_a_stride_at_full_extension() {
        // The visible stride. Measured: 0.279 - (-0.014) = 0.293 m of
        // separation at the widest, against a 0.45 m stride — the feet do
        // not reach the full stride apart because both are mid-travel, and
        // that ratio is itself worth pinning.
        let (base, p) = setup();

        let widest = (0..=200)
            .map(|i| {
                let k = forward_kinematics(&walk_pose(i as f32 / 200.0, &p, &base));
                (k[Bone::LeftFoot].z - k[Bone::RightFoot].z).abs()
            })
            .fold(0.0f32, f32::max);

        assert!(
            widest > p.stride_length * 0.55,
            "the feet only reach {widest} m apart against a {} m stride",
            p.stride_length,
        );
    }
}
