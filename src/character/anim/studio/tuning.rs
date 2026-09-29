//! Spring tuning: the model behind the per-bone DHO editor.
//!
//! Same split as the rest of the studio — presets, the response curve, and
//! the chain-application rules are plain functions over plain values, so
//! what "apply this to the whole limb" means is testable without a UI.
//!
//! # Why tuning needs an editor at all
//!
//! The pose says *where* a character goes; the springs say *how it gets
//! there*, and that is the whole difference between a heavy brute and a
//! quick duellist reading from identical pose data. It is judged entirely
//! by feel, which means it is judged by watching — and watching means
//! adjusting while the thing moves, not editing a table and relaunching.

use bevy::math::Vec2;

use crate::character::anim::math::SpringParams;
use crate::character::anim::rig::BoneSet;
use crate::character::skeleton::Bone;

/// A named starting point for a character's feel.
///
/// Presets exist because the two dials interact in a way that is obvious
/// in motion and unobvious in numbers: halving the half-life while also
/// dropping the damping ratio does not read as "twice as fast, a bit
/// looser", it reads as a completely different character. Starting from a
/// coherent pair and adjusting beats discovering that from scratch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpringPreset {
    /// Heavy and deliberate. A long half-life with slight underdamping, so
    /// mass carries past the target and settles back.
    Heavy,
    /// The shipped defaults: critically damped, graded down the limbs.
    Default,
    /// Quick and precise. Short half-life, critically damped, no overshoot
    /// — a duellist rather than a brawler.
    Quick,
    /// Loose and floaty. Long half-life, well underdamped; useful for
    /// cloth-like secondary motion and for seeing what overshoot does.
    Floaty,
}

impl SpringPreset {
    /// Every preset, for a UI to list.
    pub const ALL: [Self; 4] = [Self::Heavy, Self::Default, Self::Quick, Self::Floaty];

    /// What to call it on screen.
    pub fn label(self) -> &'static str {
        match self {
            Self::Heavy => "Heavy",
            Self::Default => "Default",
            Self::Quick => "Quick",
            Self::Floaty => "Floaty",
        }
    }

    /// The per-bone springs this preset describes.
    ///
    /// Each keeps the *grading* the defaults establish — spine slower than
    /// limbs, extremities fastest — rather than stamping one value across
    /// the rig. A uniform half-life is what makes a rig read as a puppet:
    /// every joint arriving together is the one thing real bodies never do.
    pub fn springs(self) -> BoneSet<SpringParams> {
        let defaults = crate::character::anim::dho::default_springs();

        match self {
            Self::Default => defaults,
            Self::Heavy => BoneSet::from_fn(|bone| SpringParams {
                halflife: defaults[bone].halflife * 1.8,
                damping_ratio: 0.85,
                ..defaults[bone]
            }),
            Self::Quick => BoneSet::from_fn(|bone| SpringParams {
                halflife: defaults[bone].halflife * 0.55,
                damping_ratio: 1.0,
                ..defaults[bone]
            }),
            Self::Floaty => BoneSet::from_fn(|bone| SpringParams {
                halflife: defaults[bone].halflife * 2.4,
                damping_ratio: 0.45,
                ..defaults[bone]
            }),
        }
    }
}

/// Which bones an edit applies to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TuningScope {
    /// Just the selected bone.
    Bone,
    /// The selected bone and everything below it — the natural unit,
    /// since a limb's feel comes from the whole chain, not one joint.
    Chain,
    /// Every bone.
    Rig,
}

impl TuningScope {
    /// Every scope, for a UI to list.
    pub const ALL: [Self; 3] = [Self::Bone, Self::Chain, Self::Rig];

    /// What to call it on screen.
    pub fn label(self) -> &'static str {
        match self {
            Self::Bone => "Bone",
            Self::Chain => "Chain",
            Self::Rig => "Whole rig",
        }
    }
}

/// The bones `scope` covers, starting from `bone`.
pub fn bones_in_scope(bone: Bone, scope: TuningScope) -> Vec<Bone> {
    match scope {
        TuningScope::Bone => vec![bone],
        TuningScope::Rig => Bone::ALL.to_vec(),
        TuningScope::Chain => Bone::ALL
            .iter()
            .copied()
            .filter(|&candidate| is_at_or_below(candidate, bone))
            .collect(),
    }
}

/// Whether `candidate` is `ancestor` or sits beneath it.
fn is_at_or_below(candidate: Bone, ancestor: Bone) -> bool {
    let mut current = Some(candidate);
    while let Some(bone) = current {
        if bone == ancestor {
            return true;
        }
        current = bone.parent();
    }
    false
}

/// Applies `params` to every bone in scope.
pub fn apply(
    springs: &mut BoneSet<SpringParams>,
    bone: Bone,
    scope: TuningScope,
    params: SpringParams,
) {
    for affected in bones_in_scope(bone, scope) {
        springs[affected] = params;
    }
}

/// How many points the response curve is sampled at.
const CURVE_SAMPLES: usize = 96;

/// The spring's step response, for plotting.
///
/// Samples where a bone would be over `duration` seconds if its target
/// jumped from 0 to 1 at t=0. The shape is what the numbers mean: a
/// critically damped spring rises and stops, an underdamped one overshoots
/// and rings back. Seeing that is the difference between choosing a
/// damping ratio and guessing one.
///
/// Returned as `(t, value)` points so a plot can draw it directly.
pub fn response_curve(params: &SpringParams, duration: f32) -> Vec<Vec2> {
    // Fixed substeps, so the curve does not change shape with the sample
    // count — the plot must describe the spring, not the plotting.
    const SUBSTEP: f32 = 1.0 / 480.0;

    let mut position = 0.0f32;
    let mut velocity = 0.0f32;
    let mut elapsed = 0.0f32;

    let mut points = Vec::with_capacity(CURVE_SAMPLES);
    let step = duration.max(1.0e-3) / CURVE_SAMPLES as f32;

    for index in 0..CURVE_SAMPLES {
        let target_time = index as f32 * step;

        while elapsed < target_time {
            let dt = SUBSTEP.min(target_time - elapsed);
            (position, velocity) = crate::character::anim::math::spring_scalar(
                position, velocity, 1.0, params, dt,
            );
            elapsed += dt;
        }

        points.push(Vec2::new(target_time, position));
    }

    points
}

/// How long a plot should cover for these parameters.
///
/// Scaled to the half-life so a slow spring is not squeezed into the first
/// pixel of a fixed window, and a fast one does not become a vertical line
/// followed by empty space.
pub fn suggested_plot_duration(params: &SpringParams) -> f32 {
    (params.halflife * 8.0).clamp(0.25, 4.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_preset_keeps_the_defaults_grading() {
        // A uniform half-life across the rig is what makes a character read
        // as a puppet: every joint arriving at once is the one thing real
        // bodies never do. So a preset may scale the grading, never flatten
        // it.
        for preset in SpringPreset::ALL {
            let springs = preset.springs();

            assert!(
                springs[Bone::Hips].halflife > springs[Bone::LeftHand].halflife,
                "{}: the spine should settle slower than the extremities, but Hips is \
                 {} against LeftHand's {}",
                preset.label(),
                springs[Bone::Hips].halflife,
                springs[Bone::LeftHand].halflife,
            );
        }
    }

    #[test]
    fn the_presets_are_ordered_by_speed() {
        // The names have to mean something: Quick must actually be quicker
        // than Default, which must be quicker than Heavy.
        let quick = SpringPreset::Quick.springs()[Bone::LeftArm].halflife;
        let default = SpringPreset::Default.springs()[Bone::LeftArm].halflife;
        let heavy = SpringPreset::Heavy.springs()[Bone::LeftArm].halflife;
        let floaty = SpringPreset::Floaty.springs()[Bone::LeftArm].halflife;

        assert!(quick < default, "Quick ({quick}) should be faster than Default ({default})");
        assert!(default < heavy, "Default ({default}) should be faster than Heavy ({heavy})");
        assert!(heavy < floaty, "Heavy ({heavy}) should be faster than Floaty ({floaty})");
    }

    #[test]
    fn only_the_underdamped_presets_overshoot() {
        // The damping ratio is the dial nobody can predict from the number
        // alone, so the presets must at least be internally honest about
        // it: Heavy and Floaty are documented as overshooting, the others
        // as not.
        for preset in SpringPreset::ALL {
            let params = preset.springs()[Bone::LeftArm];
            let peak = response_curve(&params, suggested_plot_duration(&params))
                .into_iter()
                .map(|point| point.y)
                .fold(0.0f32, f32::max);

            let overshoots = peak > 1.001;
            let expected = matches!(preset, SpringPreset::Heavy | SpringPreset::Floaty);

            assert_eq!(
                overshoots,
                expected,
                "{} peaked at {peak}, which {} overshoot",
                preset.label(),
                if expected { "should" } else { "should not" },
            );
        }
    }

    #[test]
    fn a_chain_scope_covers_the_bone_and_everything_below_it() {
        let chain = bones_in_scope(Bone::LeftArm, TuningScope::Chain);

        for expected in [Bone::LeftArm, Bone::LeftForeArm, Bone::LeftHand] {
            assert!(
                chain.contains(&expected),
                "{} is at or below LeftArm and should be in scope",
                expected.name(),
            );
        }

        for unexpected in [Bone::LeftShoulder, Bone::RightArm, Bone::Spine, Bone::LeftUpLeg] {
            assert!(
                !chain.contains(&unexpected),
                "{} is not below LeftArm and should not be in scope",
                unexpected.name(),
            );
        }
    }

    #[test]
    fn a_chain_scope_from_the_root_covers_the_whole_rig() {
        let chain = bones_in_scope(Bone::Hips, TuningScope::Chain);
        assert_eq!(
            chain.len(),
            Bone::ALL.len(),
            "everything descends from Hips, so its chain is the whole rig",
        );
    }

    #[test]
    fn a_bone_scope_touches_exactly_one_bone() {
        let mut springs = crate::character::anim::dho::default_springs();
        let before = springs[Bone::LeftForeArm];

        apply(
            &mut springs,
            Bone::LeftArm,
            TuningScope::Bone,
            SpringParams::critical(0.03),
        );

        assert_eq!(springs[Bone::LeftArm].halflife, 0.03);
        assert_eq!(
            springs[Bone::LeftForeArm], before,
            "a bone-scoped edit must not reach the child",
        );
    }

    #[test]
    fn a_chain_scope_reaches_the_children() {
        let mut springs = crate::character::anim::dho::default_springs();

        apply(
            &mut springs,
            Bone::LeftArm,
            TuningScope::Chain,
            SpringParams::critical(0.03),
        );

        assert_eq!(springs[Bone::LeftHand].halflife, 0.03, "the chain reaches the hand");
        assert_ne!(
            springs[Bone::RightHand].halflife, 0.03,
            "but not the other arm",
        );
    }

    #[test]
    fn the_response_curve_rises_from_zero_toward_the_target() {
        let params = SpringParams::critical(0.12);
        let curve = response_curve(&params, suggested_plot_duration(&params));

        assert_eq!(curve.len(), CURVE_SAMPLES);
        assert!(curve[0].y.abs() < 1.0e-6, "the response starts at rest");
        // 0.97 rather than 0.99: a critically damped response is
        // `(1 + yt)·e^(−yt)`, so the linear factor drags the tail out well
        // past what the exponential alone would suggest. At eight
        // half-lives it sits near 0.973, and a window long enough to reach
        // 0.99 would waste most of the plot on a flat line.
        assert!(
            curve.last().unwrap().y > 0.97,
            "and should have essentially arrived by the end of the plot, got {}",
            curve.last().unwrap().y,
        );
    }

    #[test]
    fn a_critically_damped_curve_never_exceeds_its_target() {
        // The property that makes critical damping the safe default, shown
        // on the very curve the editor plots.
        let params = SpringParams::critical(0.1);
        let curve = response_curve(&params, suggested_plot_duration(&params));

        for point in curve {
            assert!(
                point.y <= 1.001,
                "a critically damped spring should not overshoot, but reached {} at \
                 t={}",
                point.y,
                point.x,
            );
        }
    }

    #[test]
    fn an_underdamped_curve_does_exceed_it() {
        // The counterpart: proves the plot can actually show overshoot,
        // rather than the test above passing because the curve is flat.
        let params = SpringParams { halflife: 0.1, damping_ratio: 0.4, max_speed: 50.0 };
        let curve = response_curve(&params, suggested_plot_duration(&params));
        let peak = curve.into_iter().map(|point| point.y).fold(0.0f32, f32::max);

        assert!(peak > 1.02, "an underdamped spring should overshoot, peaked at {peak}");
    }

    #[test]
    fn the_curve_does_not_change_shape_with_its_duration() {
        // The plot must describe the spring, not the plotting. Sampling the
        // same spring over a longer window has to give the same values at
        // the same times.
        let params = SpringParams::critical(0.12);
        let short = response_curve(&params, 0.5);
        let long = response_curve(&params, 1.0);

        // The long curve's first half covers the same span as the short
        // one; compare at a matching time.
        let probe = 0.25f32;
        let sample = |curve: &[Vec2]| -> f32 {
            curve
                .iter()
                .min_by(|a, b| {
                    (a.x - probe).abs().partial_cmp(&(b.x - probe).abs()).unwrap()
                })
                .unwrap()
                .y
        };

        assert!(
            (sample(&short) - sample(&long)).abs() < 0.02,
            "the same spring sampled over different windows disagreed at t={probe}: {} \
             vs {}",
            sample(&short),
            sample(&long),
        );
    }

    #[test]
    fn the_plot_window_scales_with_the_half_life() {
        let fast = SpringParams::critical(0.05);
        let slow = SpringParams::critical(0.4);

        assert!(
            suggested_plot_duration(&fast) < suggested_plot_duration(&slow),
            "a slower spring needs a longer window",
        );
    }
}
