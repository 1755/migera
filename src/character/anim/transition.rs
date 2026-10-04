//! Changing gait: starting, stopping, and speeding up without a lurch.
//!
//! # What was actually wrong
//!
//! This module was planned against two guesses, and measuring the running
//! stack refuted both. Worth recording, because the real problems are not
//! the obvious ones:
//!
//! - *"Stopping leaves a foot mid-swing."* It does not.
//!   [`super::phase::GaitPhase`] computes its frequency as
//!   `base_frequency_hz + speed_coefficient * speed`, and the base term is
//!   `1/7` Hz — a slow idle sway that never reaches zero. So a character
//!   told to stop **keeps stepping forever**, at 0.143 Hz. Measured: the
//!   left foot creeping from `y = -0.093` to `-0.035` over two seconds of
//!   standing still. That reads worse than freezing would.
//! - *"A speed change makes the pose jump."* The phase is continuous, so
//!   the pose does not jump. What steps is the **cadence**: dropping from
//!   1.2 m/s to a standstill takes the clock from 1.223 Hz to 0.143 in one
//!   frame, an 8.6x deceleration. The legs go from walking to crawling
//!   between two frames.
//!
//! So the fixes are about *rate*, not about blending poses. There is
//! nothing to cross-fade — the gait is already one continuous function of
//! phase.
//!
//! # The two mechanisms
//!
//! **A smoothed cadence.** The clock follows the speed's implied frequency
//! through a critically damped spring rather than adopting it instantly.
//! That makes a stride lengthen and shorten the way a real one does, and it
//! is what removes the 8.6x step.
//!
//! **A real stop.** Below a walking threshold the gait fades out entirely
//! and the character stands, rather than mincing at the idle frequency
//! forever. Crucially it fades on a **footfall**, so the stop lands with
//! both feet down instead of freezing mid-swing — the failure the original
//! plan wrongly attributed to the existing code.
//!
//! # Starting and stopping the way a body does (Winter §11.3.2, §11.3.3)
//!
//! **A first step is prepared.** Before either foot lifts, the body moves
//! its weight onto the leg that will stand and begins to tip forward — the
//! *release phase*, about half a stride long in Winter's Figure 11.8. A
//! walk that simply fades in skips it, and the swing foot appears to pull
//! the body along. So a start first runs [`Transition::release`] from 0 to 1
//! with the gait still at zero weight ([`Transition::apply_release`] poses
//! it), and only then fades the gait in. The leg that stands is the one
//! already carrying the weight, if the idle has shifted onto one, so the
//! release continues the idle's posture instead of undoing it.
//!
//! **The fade is placed in the stride, not in time.** The gait joins at the
//! swinging leg's MID-SWING, where the walking pose is closest to standing:
//! stance leg upright under the hip, the other foot passing beside it.
//! From there the gait fades in over the rest of that swing and is fully
//! applied at the first heel contact. A stop is the same in reverse: after
//! a footfall it waits out the double support, then fades from the other
//! foot's toe-off to its mid-swing, so the last foot to move comes from
//! behind to beside the planted one — Winter's final step of about half the
//! normal length — and the feet end side by side. The legs keep the
//! stride's cadence ([`Transition::stride_speed`]) through that last step;
//! dropping the clock to the standing sway's rate there froze the step
//! half-way.
//!
//! **Both fades run only through single support.** With both feet down the
//! walk holds the distance between them, and a blend whose weight is still
//! changing scales it, so one of the two planted feet has to slip: fading
//! through double support slipped the unloading foot up to 12.5 mm a frame.
//!
//! **The first swing is lifted.** Mid-swing is where a walking foot passes
//! closest to the floor (1.5 cm, Winter) at its fastest; blended with a
//! standing foot at zero clearance, the first swing skimmed 94 mm along the
//! floor before it rose 2 mm. Reweighting the joints cannot fix that — a
//! knee leading the blend pointed the toe 2 cm INTO the floor, the whole
//! leg leading still skimmed 9 cm — because the walk itself is lowest there.
//! A real first step picks the foot up first. So [`Transition::blend`]
//! holds the swinging toe at least [`FIRST_SWING_LIFT`] above where it
//! stood, rising early in the fade and gone by heel contact, and the leg IK
//! bends the knee to meet it.

use bevy::math::Vec3;

use super::gait::smoothstep;
use super::legik::{solve_leg_on, LegChain, LegIkConfig};
use super::rig::{forward_kinematics_on, LocalPose, RigGeometry};
use super::stance;

/// How high the first swing carries the toe at least, metres above where
/// it stood. About what the walk's own swing reaches early in swing (the
/// ball ~9 cm up just after toe-off, measured on this rig), halved: this
/// swing starts from beside the planted foot, not from behind it.
pub const FIRST_SWING_LIFT: f32 = 0.05;

/// How high the last swing holds its toe at least, metres above where it
/// will stand, while it is still more than [`LAND_REACH`] from there.
///
/// The last step fades a full swing (it leaves the floor ~0.66 m behind its
/// spot at 1.2 m/s) into standing in ~0.4 s, and the blended foot came down
/// on the way: 258 mm short at 13 mm up, 74 mm short at 5 mm INTO the floor,
/// which the foot IK then slid along it. A real last step is set down onto
/// its spot. The walk's own swing clears ~3 cm through most of its travel.
pub const LAND_LIFT: f32 = 0.03;

/// How close to its standing spot the last swing may descend from
/// [`LAND_LIFT`], metres horizontally: the lift eases out over this, so the
/// foot meets the floor only when it is over its spot.
pub const LAND_REACH: f32 = 0.12;

/// How long a landing stays active after the last step's fade ends,
/// seconds: the rendered foot lags the target through the legs' springs,
/// and has to finish arriving before the lift lets go.
pub const LAND_HOLD: f32 = 0.25;

/// How high a foot being set down is held above its spot, metres, when its
/// toe is `away` metres from it horizontally: [`LAND_LIFT`], eased out over
/// the last [`LAND_REACH`] — level where it joins the full lift, steepest
/// where it meets the floor, so the foot comes DOWN onto its spot. A
/// smoothstep flattens there too, and live the ball crept its last ~18 mm
/// within 3 mm of the floor.
pub fn landing_lift(away: f32) -> f32 {
    let x = (away / LAND_REACH).clamp(0.0, 1.0);
    LAND_LIFT * x * (2.0 - x)
}

/// How far the release tips the body forward over its feet at the
/// recorded stride's speed, metres of pelvis travel.
///
/// Winter §11.3.2: by the first toe-off the centre of mass is ~6 cm
/// forward of quiet standing. The release covers the start of that; the
/// walk takes over the rest. Scaled down for slower walks, after Halliday
/// et al.: the pattern is the same at every speed, sized by the speed.
pub const RELEASE_LEAN: f32 = 0.04;

/// How a character eases between standing and walking.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransitionConfig {
    /// How quickly the cadence follows the speed it implies, as a
    /// half-life in seconds.
    ///
    /// Short enough to feel responsive, long enough that the 8.6x cadence
    /// drop of an abrupt stop is spread over several frames rather than
    /// landing in one.
    pub cadence_halflife: f32,
    /// Below this speed the character is standing, not walking.
    ///
    /// Not zero: a character asked for 0.01 m/s should stand still, not
    /// take one step every two minutes.
    pub walk_threshold: f32,
    /// How long an interrupted fade takes, in seconds: a walk resumed in
    /// the middle of a stop, or a stop asked for before the first step has
    /// fully faded in. The planned start and stop are timed by the stride
    /// instead (`mid_swing`).
    pub blend_seconds: f32,
    /// How long the release before a first step takes, seconds.
    ///
    /// Winter's Figure 11.8 runs quiet standing to the first toe-off over
    /// ~70% of a stride, the release itself ~50%: about half a second at
    /// his subject's 0.99 s stride. The gait here joins at mid-swing, after
    /// the toe-off, so the release covers the whole preparation.
    pub release_seconds: f32,
    /// Stride fraction from a footfall to the other leg's mid-swing, where
    /// the feet pass side by side: half the duty factor.
    ///
    /// Places the first step's fade-in and the last step's fade-out; see
    /// the module docs. Both fades run only through SINGLE support —
    /// [`TransitionConfig::fade`] long — because with both feet down the
    /// walk holds the distance between them, and a changing blend scales
    /// it: the unloading foot slipped up to 12.5 mm a frame.
    pub mid_swing: f32,
}

impl TransitionConfig {
    /// The length of either fade, stride fraction: from mid-swing to the
    /// swinging foot's heel contact (a first step), or from the other foot's
    /// toe-off to its mid-swing (a last step). The same span, `0.5 −
    /// mid_swing`, both ways.
    pub fn fade(&self) -> f32 {
        (0.5 - self.mid_swing).max(0.0)
    }

    /// Stride fraction from a footfall to the other foot's toe-off, where
    /// double support ends: `duty − 0.5`.
    fn toe_off(&self) -> f32 {
        (2.0 * self.mid_swing - 0.5).max(0.0)
    }
}

impl Default for TransitionConfig {
    fn default() -> Self {
        Self {
            cadence_halflife: 0.15,
            walk_threshold: 0.15,
            blend_seconds: 0.25,
            release_seconds: 0.5,
            mid_swing: super::reference::STANCE_FRACTION * 0.5,
        }
    }
}

/// Where a character is between standing and walking.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
enum Stage {
    /// At rest: no gait, no release.
    #[default]
    Standing,
    /// Preparing a first step, or backing out of one.
    Releasing,
    /// Fading the gait in from mid-swing. `from` is the stride position
    /// the fade began at, read on its first frame.
    FirstStep { from: Option<f32> },
    /// Fully walking.
    Walking,
    /// Fading the gait out after the footfall at `from`.
    LastStep { from: f32 },
    /// A fade on the clock, for a start or stop that changed its mind.
    Blending,
}

/// Something the caller has to act on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransitionEvent {
    /// The first step begins: put the gait clock at `cycle`, the swinging
    /// leg's mid-swing, before posing this frame.
    FirstStep { cycle: f32 },
    /// The character has come to rest.
    AtRest,
}

/// A character's progress between standing and walking.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Transition {
    /// How much of the gait is currently applied, 0 (standing) to 1
    /// (walking).
    pub weight: f32,
    /// The cadence actually in use, Hz — the smoothed version of what the
    /// current speed implies.
    pub cadence_hz: f32,
    /// How far the preparation for a first step has gone, 0 to 1.
    pub release: f32,
    /// The leg the first step stands on, +1 the rig's left, −1 its right;
    /// the other leg swings.
    pub stance: f32,
    /// The standing weight shift the release began from, on
    /// [`stance::shift_weight`]'s scale.
    pub release_from: f32,
    /// The standing weight shift now, on the same scale. The caller writes
    /// it while the character stands, so a start knows which leg is loaded.
    pub idle_shift: f32,
    /// The speed the legs are stepping at: the speed asked for, except
    /// through the last step of a stop, which finishes at the walk's own.
    pub stride_speed: f32,
    /// How far through the first step's fade the character is, 0 to 1;
    /// zero outside a first step. Shapes the first swing's lift
    /// ([`Transition::blend`]).
    pub first_swing: f32,
    /// How far through the last step's fade the character is, 0 to 1;
    /// zero outside a last step. Ramps in the last swing's landing lift
    /// ([`Transition::blend`]).
    pub last_swing: f32,
    /// The leg the last step swings, +1 the rig's left, −1 its right.
    pub last_swing_leg: f32,
    /// Seconds the last step's landing stays active after its fade ended
    /// ([`LAND_HOLD`] at the end, counting down at rest).
    pub landing_hold: f32,
    stage: Stage,
}

impl Transition {
    /// A character standing still.
    pub fn standing() -> Self {
        Self::default()
    }

    /// A character already walking at `cadence_hz`.
    pub fn walking(cadence_hz: f32) -> Self {
        Self { weight: 1.0, cadence_hz, stage: Stage::Walking, ..Self::default() }
    }

    /// Whether the gait is doing anything at all.
    pub fn is_standing(&self) -> bool {
        self.weight <= 0.0
    }

    /// Whether the character is fully at rest: no gait, and no first step
    /// being prepared. Standing idle motion belongs only here.
    pub fn is_at_rest(&self) -> bool {
        self.stage == Stage::Standing
    }

    /// Advances toward the gait `speed` implies.
    ///
    /// `phase` is the current cycle position, needed because a fade-out
    /// waits for a footfall rather than stopping wherever it happens to be.
    pub fn advance(
        &mut self,
        speed: f32,
        phase: f32,
        config: &TransitionConfig,
        dt: f32,
    ) -> Option<TransitionEvent> {
        // Counted before the stage advances, so the frame a last step ends
        // starts the hold at its full length.
        self.landing_hold = (self.landing_hold - dt.max(0.0)).max(0.0);
        let event = self.advance_stage(speed, phase, config, dt);
        if self.stage != Stage::Standing {
            self.landing_hold = 0.0;
        }
        // Only a first or last step lifts its swing; their own branches set
        // the progress.
        if !matches!(self.stage, Stage::FirstStep { .. }) {
            self.first_swing = 0.0;
        }
        if !matches!(self.stage, Stage::LastStep { .. }) {
            self.last_swing = 0.0;
        }
        event
    }

    fn advance_stage(
        &mut self,
        speed: f32,
        phase: f32,
        config: &TransitionConfig,
        dt: f32,
    ) -> Option<TransitionEvent> {
        if dt <= 0.0 || !dt.is_finite() {
            return None;
        }

        let walking = speed.abs() >= config.walk_threshold;

        // The weight fades on a fixed deadline rather than a half-life:
        // "the gait is fully out" has to be reachable, and an exponential
        // only ever gets small.
        let per = |seconds: f32| if seconds > 0.0 { dt / seconds } else { 1.0 };
        let step = per(config.blend_seconds);
        let release_step = per(config.release_seconds);
        // How far through a stride-timed fade `phase` is: one that begins
        // `delay` after `from` and lasts `config.fade()`. Measured so that a
        // phase a little BEFORE `from` reads as not started rather than
        // wrapping round to "long finished".
        let through = |from: f32, delay: f32| {
            let since = (phase - from + 0.25).rem_euclid(1.0) - 0.25 - delay;
            if config.fade() > 0.0 {
                (since / config.fade()).clamp(0.0, 1.0)
            } else if since >= 0.0 {
                1.0
            } else {
                0.0
            }
        };

        if walking {
            self.stride_speed = speed;
            self.cadence_hz =
                approach(self.cadence_hz, cadence_for(speed), config.cadence_halflife, dt);
            match self.stage {
                Stage::Standing | Stage::Releasing => {
                    if self.stage == Stage::Standing {
                        // The loaded leg stands; with none loaded, the left,
                        // as in Winter's Figure 11.8.
                        self.stance = if self.idle_shift.abs() > 0.05 { self.idle_shift.signum() } else { 1.0 };
                        self.release_from = self.idle_shift;
                        self.stage = Stage::Releasing;
                    }
                    self.release = (self.release + release_step).min(1.0);
                    if self.release >= 1.0 {
                        self.stage = Stage::FirstStep { from: None };
                        // Mid-swing of the leg that does NOT stand: the right
                        // leg's is `mid_swing` after the left's footfall at 0.
                        let cycle = if self.stance > 0.0 { 0.0 } else { 0.5 } + config.mid_swing;
                        return Some(TransitionEvent::FirstStep { cycle: cycle.rem_euclid(1.0) });
                    }
                }
                Stage::FirstStep { from } => {
                    // Read where the clock really is on the first frame,
                    // rather than trusting the caller moved it.
                    let from = from.unwrap_or(phase);
                    self.stage = Stage::FirstStep { from: Some(from) };
                    self.first_swing = through(from, 0.0);
                    self.weight = self.weight.max(smoothstep(self.first_swing));
                }
                Stage::Walking => {}
                Stage::LastStep { .. } | Stage::Blending => {
                    self.weight = (self.weight + step).min(1.0);
                    self.stage = Stage::Blending;
                }
            }
            if self.weight >= 1.0 {
                self.weight = 1.0;
                self.release = 0.0;
                self.stage = Stage::Walking;
            }
            return None;
        }

        match self.stage {
            Stage::Standing => {
                self.stride_speed = speed;
            }
            Stage::Releasing => {
                self.release = (self.release - release_step).max(0.0);
                if self.release <= 0.0 {
                    self.stage = Stage::Standing;
                    return Some(TransitionEvent::AtRest);
                }
            }
            // Stopping. The character walks its last step OUT rather than
            // stalling mid-stride, so the fade can land on a footfall.
            //
            // That ordering is load-bearing and the first version got it
            // backwards: decaying the cadence while waiting for a footfall
            // is a deadlock — the phase stops advancing, so the footfall the
            // wait is waiting for never arrives. Measured: from phase 0.2 the
            // cadence was near zero within half a second and the phase froze
            // at ~0.3, and the fade never began at all.
            Stage::Walking => {
                if near_footfall(phase, step, self.cadence_hz, dt) {
                    // The footfall itself (0 or 0.5), not the frame's phase,
                    // which may be a frame either side of it.
                    let footfall = ((phase * 2.0).round() * 0.5).rem_euclid(1.0);
                    self.stage = Stage::LastStep { from: footfall };
                } else {
                    // Hold BOTH weight and cadence: still walking, for now.
                    return None;
                }
            }
            Stage::LastStep { from } => {
                // The legs keep the stride's cadence to finish the step. The
                // fade waits out the double support after the footfall and
                // runs from the other foot's toe-off to its mid-swing.
                self.last_swing = through(from, config.toe_off());
                // The foot that just landed stands; the other swings: the
                // left lands at 0, the right at 0.5.
                self.last_swing_leg = if from < 0.25 { -1.0 } else { 1.0 };
                self.weight = self.weight.min(1.0 - smoothstep(self.last_swing));
                if self.weight <= 0.0 {
                    self.stage = Stage::Standing;
                    self.stride_speed = speed;
                    self.landing_hold = LAND_HOLD;
                    return Some(TransitionEvent::AtRest);
                }
                return None;
            }
            Stage::FirstStep { .. } | Stage::Blending => {
                self.stage = Stage::Blending;
                self.weight = (self.weight - step).max(0.0);
                self.release = (self.release - release_step).max(0.0);
                if self.weight <= 0.0 {
                    self.stride_speed = speed;
                    self.stage = if self.release > 0.0 { Stage::Releasing } else { Stage::Standing };
                    if self.stage == Stage::Standing {
                        return Some(TransitionEvent::AtRest);
                    }
                }
            }
        }

        // NOT snapped to zero at rest, deliberately.
        //
        // An earlier version did, and it made the stop untestable:
        // reintroducing an idle-frequency floor in the decay left every test
        // green, because the end state was ASSIGNED rather than reached. A
        // guarantee that hides whether the mechanism producing it works is
        // worth less than the mechanism. `a_stopped_character_actually_stops`
        // measures that the decay reaches zero on its own.
        self.cadence_hz = approach(self.cadence_hz, 0.0, config.cadence_halflife, dt);
        None
    }

    /// The pose between `standing` (with any release already applied) and
    /// `walking`, [`Transition::weight`] of the walk, on `rig`. Through a
    /// first step the swinging toe is also held at least the lift above
    /// where it stood — [`FIRST_SWING_LIFT`], rising over the first fifth of
    /// the fade and eased out over its second half, so it is gone by heel
    /// contact — and the leg re-solved to meet it (see the module docs).
    pub fn blend(&self, standing: &LocalPose, walking: &LocalPose, rig: &RigGeometry) -> LocalPose {
        let mut pose = super::clip::blend(standing, walking, self.weight);
        let p = self.first_swing;
        let lift = FIRST_SWING_LIFT * smoothstep(p / 0.2) * (1.0 - smoothstep((p - 0.5) / 0.5));
        if lift > 1.0e-5 {
            let chain = if self.stance > 0.0 { LegChain::RIGHT } else { LegChain::LEFT };
            let stood = forward_kinematics_on(standing, rig)[chain.toe];
            let toe = forward_kinematics_on(&pose, rig)[chain.toe];
            let short = stood.y + lift - toe.y;
            if short > 0.0 {
                solve_leg_on(&mut pose, chain, toe + Vec3::Y * short, &LegIkConfig::default(), rig);
            }
        }
        // The last swing is set down onto its spot: held at least
        // `LAND_LIFT` up until it is within `LAND_REACH` of where it will
        // stand, ramped in over the fade's first fifth so a toe just off the
        // floor is not yanked up.
        if self.last_swing > 0.0 {
            let chain = if self.last_swing_leg > 0.0 { LegChain::LEFT } else { LegChain::RIGHT };
            let stood = forward_kinematics_on(standing, rig)[chain.toe];
            let toe = forward_kinematics_on(&pose, rig)[chain.toe];
            let away = Vec3::new(toe.x - stood.x, 0.0, toe.z - stood.z).length();
            let lift = smoothstep(self.last_swing / 0.2) * landing_lift(away);
            let short = stood.y + lift - toe.y;
            if short > 0.0 {
                solve_leg_on(&mut pose, chain, toe + Vec3::Y * short, &LegIkConfig::default(), rig);
            }
        }
        pose
    }

    /// The foot the last step is setting down, and where, for the foot IK
    /// (`plugin::AnimFootIk::landing`): through the last step's fade, ramped
    /// in over its first fifth, and for [`LAND_HOLD`] after, while the
    /// rendered foot catches up with the target through the springs.
    /// `standing` is the pose the character comes to rest in.
    pub fn landing(&self, standing: &LocalPose, rig: &RigGeometry) -> Option<super::plugin::Landing> {
        let strength = if self.last_swing > 0.0 {
            smoothstep(self.last_swing / 0.2)
        } else if self.landing_hold > 0.0 {
            1.0
        } else {
            return None;
        };
        let left = self.last_swing_leg > 0.0;
        let chain = if left { LegChain::LEFT } else { LegChain::RIGHT };
        Some(super::plugin::Landing { left, spot: forward_kinematics_on(standing, rig)[chain.toe], strength, place: false })
    }

    /// Poses the release on `pose` (a standing one), on `rig`: the weight
    /// moving from where the idle had it onto the stance leg, and the body
    /// tipping forward over its feet. Nothing when no release is under way.
    pub fn apply_release(&self, pose: &mut LocalPose, rig: &RigGeometry) {
        if self.release <= 0.0 {
            return;
        }
        let eased = smoothstep(self.release);
        let onto = self.release_from + (self.stance - self.release_from) * eased;
        stance::shift_weight(pose, rig, onto);
        let pace = (self.stride_speed.abs() / super::reference::SPEED).min(1.0);
        stance::sway_over_feet(pose, rig, rig.forward() * (RELEASE_LEAN * pace * eased));
    }
}

/// The stride frequency a speed implies.
///
/// Deliberately NOT [`super::phase::GaitPhase::gait_frequency_hz`]'s
/// formula: that one carries a `base_frequency_hz` floor of `1/7` Hz, which
/// is right for an idle sway and wrong here — it is exactly why a stopped
/// character kept stepping. This returns zero for zero speed.
pub fn cadence_for(speed: f32) -> f32 {
    // The same coefficient the phase clock uses, so a walking character's
    // cadence is unchanged by routing through here.
    const STRIDES_PER_METRE_PER_SECOND: f32 = 0.9;

    speed.abs() * STRIDES_PER_METRE_PER_SECOND
}

/// Moves `current` toward `target` with an exponential half-life.
///
/// Frame-rate independent, unlike a per-frame lerp: halving `dt` halves the
/// step rather than changing where the value ends up.
fn approach(current: f32, target: f32, halflife: f32, dt: f32) -> f32 {
    if halflife <= 0.0 {
        return target;
    }

    let decay = (-std::f32::consts::LN_2 * dt / halflife).exp();
    target + (current - target) * decay
}

/// Whether `phase` is within one frame of a footfall.
///
/// Phase 0 is the left foot's footfall and 0.5 is the right's, so either
/// counts — a stop should not wait up to a whole cycle when half will do.
///
/// The window is sized from how far the phase actually moves per frame. A
/// fixed one is wrong in both directions: too small and a fast cadence steps
/// straight over it and waits another half cycle; too large and a slow one
/// stops well before the foot lands.
fn near_footfall(phase: f32, blend_step: f32, cadence_hz: f32, dt: f32) -> bool {
    let cycle = phase.rem_euclid(1.0);

    // One frame of phase travel, with a floor so a stalled cadence can
    // still find the window.
    let per_frame = (cadence_hz * dt).max(1.0e-3);

    // Wide enough to catch the frame before the footfall as well as the one
    // after, and at least as wide as the blend's own step so a long blend
    // does not begin implausibly early.
    let window = (per_frame * 1.5).max(blend_step.min(0.05));

    cycle < window || (cycle - 0.5).abs() < window || cycle > 1.0 - window
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// Runs a transition for `seconds`, advancing the phase at its own
    /// cadence — the loop a real frame does.
    fn run(
        transition: &mut Transition,
        speed: f32,
        seconds: f32,
        config: &TransitionConfig,
    ) -> f32 {
        let mut phase = 0.0;

        let steps = (seconds / DT).round() as usize;
        for _ in 0..steps {
            if let Some(TransitionEvent::FirstStep { cycle }) =
                transition.advance(speed, phase, config, DT)
            {
                phase = cycle;
            }
            phase = (phase + transition.cadence_hz * DT).rem_euclid(1.0);
        }

        phase
    }

    // -----------------------------------------------------------------
    // A real stop — the problem the measurement actually found
    // -----------------------------------------------------------------

    #[test]
    fn a_stopped_character_actually_stops() {
        // THE bug this module exists for. `GaitPhase`'s own frequency never
        // reaches zero — it floors at `base_frequency_hz`, 1/7 Hz — so a
        // character told to stop kept stepping forever. Measured on the
        // running stack: a foot creeping from y = -0.093 to -0.035 over two
        // seconds of standing still.
        let mut transition = Transition::walking(cadence_for(1.2));
        let config = TransitionConfig::default();

        // Sampled mid-fade as well as at the end. The final cadence is
        // forced to zero once the weight reaches it — a belt-and-braces
        // guarantee — which means checking only the end state cannot tell
        // "the cadence decayed" from "I assigned zero". Verified: with the
        // idle floor reintroduced this assertion still passed while
        // `cadence_for_is_zero_at_zero_speed` failed alone.
        let mut mid_fade = None;
        {
            let mut phase = 0.0;
            for i in 0..180 {
                transition.advance(0.0, phase, &config, DT);
                phase = (phase + transition.cadence_hz * DT).rem_euclid(1.0);

                if i == 90 {
                    mid_fade = Some(transition.cadence_hz);
                }
            }
        }

        assert!(
            transition.is_standing(),
            "the gait should have faded out entirely, weight is {}",
            transition.weight,
        );
        assert!(
            mid_fade.expect("sampled") < 0.05,
            "a second and a half into the stop the cadence was still {} Hz — it is \
             decaying toward an idle floor rather than toward zero",
            mid_fade.unwrap(),
        );
        assert!(
            transition.cadence_hz < 1.0e-3,
            "and it decays to zero rather than being assigned there, got {} Hz",
            transition.cadence_hz,
        );
    }

    #[test]
    fn a_crawling_speed_counts_as_standing() {
        // A character asked for 0.01 m/s should stand, not take one step
        // every two minutes.
        let mut transition = Transition::walking(cadence_for(1.2));
        let config = TransitionConfig::default();

        run(&mut transition, 0.01, 3.0, &config);

        assert!(transition.is_standing(), "weight {}", transition.weight);
    }

    #[test]
    fn a_stop_lands_on_a_footfall() {
        // The stop must leave a foot ON THE GROUND rather than frozen
        // mid-swing. The fade holds at full weight until a footfall comes
        // round, waits out the double support after it (see
        // `TransitionConfig::fade`), and runs from the other foot's toe-off.
        let config = TransitionConfig::default();

        // From several starting phases, including mid-swing.
        for start in [0.0_f32, 0.2, 0.35, 0.7, 0.9] {
            let mut transition = Transition::walking(cadence_for(1.2));
                let mut phase = start;

            // The phase where the fade actually began.
            let mut began_at = None;

            for _ in 0..600 {
                let before = transition.weight;
                transition.advance(0.0, phase, &config,DT);

                if began_at.is_none() && transition.weight < before {
                    began_at = Some(phase.rem_euclid(1.0));
                }

                phase = (phase + transition.cadence_hz * DT).rem_euclid(1.0);
            }

            let at = began_at.expect("the fade should have started");
            // Where, after its footfall (0 or 0.5), the fade began.
            let after = (at * 2.0).rem_euclid(1.0) * 0.5;

            assert!(
                (after - config.toe_off()).abs() < 0.03,
                "from phase {start} the fade began at {at}, {after} of a cycle after \
                 a footfall; the other foot's toe-off is {} after it",
                config.toe_off(),
            );
        }
    }

    // -----------------------------------------------------------------
    // Cadence smoothing — the other measured problem
    // -----------------------------------------------------------------

    #[test]
    fn the_cadence_does_not_step_when_the_speed_does() {
        // Measured on the running stack: dropping from 1.2 m/s to a
        // standstill took the clock from 1.223 Hz to 0.143 in ONE frame,
        // an 8.6x deceleration. The pose stays continuous through that —
        // it is the rate that lurches.
        let mut transition = Transition::walking(cadence_for(1.2));
        let config = TransitionConfig::default();

        let before = transition.cadence_hz;
        transition.advance(0.0, 0.3, &config,DT);

        let ratio = before / transition.cadence_hz.max(1.0e-6);
        assert!(
            ratio < 1.3,
            "one frame of stopping changed the cadence by {ratio}x, from {before} \
             to {}",
            transition.cadence_hz,
        );
    }

    #[test]
    fn the_cadence_reaches_the_speed_it_is_given() {
        let mut transition = Transition::standing();
        let config = TransitionConfig::default();

        run(&mut transition, 1.2, 2.0, &config);

        let expected = cadence_for(1.2);
        assert!(
            (transition.cadence_hz - expected).abs() < 0.01,
            "expected {expected} Hz, got {}",
            transition.cadence_hz,
        );
    }

    #[test]
    fn the_cadence_is_frame_rate_independent() {
        // An exponential half-life rather than a per-frame lerp: halving dt
        // must halve the step, not change where the value lands.
        let config = TransitionConfig::default();

        let mut coarse = Transition::standing();
        let mut fine = Transition::standing();

        for _ in 0..60 {
            coarse.advance(1.2, 0.0, &config,1.0 / 60.0);
        }
        for _ in 0..240 {
            fine.advance(1.2, 0.0, &config,1.0 / 240.0);
        }

        assert!(
            (coarse.cadence_hz - fine.cadence_hz).abs() < 0.01,
            "60 Hz reached {} and 240 Hz reached {}",
            coarse.cadence_hz,
            fine.cadence_hz,
        );
    }

    // -----------------------------------------------------------------
    // Starting
    // -----------------------------------------------------------------

    #[test]
    fn a_start_prepares_before_the_first_step() {
        // Winter §11.3.2: the release comes first, with no foot moving —
        // then the gait, from the swinging leg's mid-swing.
        let config = TransitionConfig::default();
        let mut transition = Transition::standing();

        let mut first_step = None;
        let mut frames = 0;
        while first_step.is_none() && frames < 600 {
            let event = transition.advance(1.2, 0.0, &config, DT);
            frames += 1;
            if let Some(TransitionEvent::FirstStep { cycle }) = event {
                first_step = Some(cycle);
            } else {
                assert_eq!(transition.weight, 0.0, "the gait moved during the release");
                assert!(!transition.is_at_rest(), "a release is not rest");
            }
        }
        let seconds = frames as f32 * DT;
        assert!(
            (seconds - config.release_seconds).abs() < 2.0 * DT,
            "the release took {seconds} s, not {}",
            config.release_seconds,
        );
        assert_eq!(transition.release, 1.0);
        // Standing square, the left leg stands and the right swings first.
        assert_eq!(transition.stance, 1.0);
        assert_eq!(first_step, Some(config.mid_swing), "the right leg's mid-swing");

        // The gait then fades in from where the clock was put, and the
        // first frame of it is barely there — no snap.
        let mut phase = first_step.unwrap();
        transition.advance(1.2, phase, &config, DT);
        phase += transition.cadence_hz * DT;
        transition.advance(1.2, phase, &config, DT);
        assert!(
            transition.weight > 0.0 && transition.weight < 0.1,
            "one frame into the first step, weight {}",
            transition.weight,
        );
    }

    #[test]
    fn a_start_stands_on_the_leg_already_carrying_the_weight() {
        // The unloaded leg swings, so the release continues the idle's
        // posture rather than undoing it.
        let config = TransitionConfig::default();
        for (idle, stance, cycle) in [(0.8, 1.0, config.mid_swing), (-0.6, -1.0, 0.5 + config.mid_swing)] {
            let mut transition = Transition { idle_shift: idle, ..Transition::standing() };
            let mut first = None;
            for _ in 0..600 {
                if let Some(TransitionEvent::FirstStep { cycle }) = transition.advance(1.2, 0.0, &config, DT) {
                    first = Some(cycle);
                    break;
                }
            }
            assert_eq!(transition.stance, stance, "idle shift {idle}");
            assert_eq!(transition.release_from, idle);
            assert!((first.unwrap() - cycle).abs() < 1.0e-6, "idle {idle}: joined at {first:?}");
        }
    }

    #[test]
    fn the_last_step_keeps_the_walks_cadence_and_ends_at_mid_swing() {
        // Winter §11.3.3: the final step is walked out, not frozen. The legs
        // keep the stride's speed until the gait is gone, and the fade
        // spans the other leg's toe-off to its mid-swing: single support
        // only, ending with the feet side by side.
        let config = TransitionConfig::default();
        let mut transition = Transition::walking(cadence_for(1.2));
        transition.stride_speed = 1.2;

        let mut phase = 0.37;
        let mut began = None;
        let mut ended = None;
        for _ in 0..600 {
            let before = transition.weight;
            let event = transition.advance(0.0, phase, &config, DT);
            if began.is_none() && transition.weight < before {
                began = Some(phase);
            }
            if event == Some(TransitionEvent::AtRest) {
                ended = Some(phase);
                break;
            }
            assert_eq!(transition.stride_speed, 1.2, "the legs slowed before the last step ended");
            phase = (phase + transition.cadence_hz * DT).rem_euclid(1.0);
        }
        let (began, ended) = (began.expect("a fade"), ended.expect("came to rest"));
        // Both measured from the footfall (0 or 0.5) before them.
        let after_footfall = |at: f32| (at * 2.0).rem_euclid(1.0) * 0.5;
        assert!(
            (after_footfall(began) - config.toe_off()).abs() < 0.03,
            "the fade began {} after the footfall; the other foot's toe-off is {}",
            after_footfall(began),
            config.toe_off(),
        );
        assert!(
            (after_footfall(ended) - config.mid_swing).abs() < 0.03,
            "the fade ended {} after the footfall; mid-swing is {}",
            after_footfall(ended),
            config.mid_swing,
        );
        assert!(transition.is_at_rest());
        assert_eq!(transition.stride_speed, 0.0, "at rest the legs take the asked-for speed");
    }

    #[test]
    fn the_last_step_is_half_a_step_and_ends_beside_the_planted_foot() {
        // The claim the stride-timed fade rests on, measured on the real rig
        // and walk: the pose the fade ends in (standing) has the feet side
        // by side, the swinging foot comes forward the whole way without
        // stepping back, and it travels about half as far as a walking
        // step's swing (Winter's final step of about half the normal length;
        // his lands half a step ahead, ours beside, so the character ends in
        // its standing pose).
        use super::super::gait::{walk_pose_on, GaitParams};
        use super::super::rig::forward_kinematics_on;
        use crate::character::skeleton::Bone;

        let rig = super::super::gltf_rig::puppet_base_as_rendered();
        let stood = stance::stance_on_rig(
            &super::super::poses::relaxed_stand(),
            stance::DEFAULT_KNEE_FLEX,
            &rig,
        );
        let params = GaitParams::walking_on(1.2, &rig);
        let mid = params.duty_factor * 0.5;
        // The right ankle's lead over the left, along the rig's forward.
        let lead = |pose: &LocalPose| {
            let at = forward_kinematics_on(pose, &rig);
            (at[Bone::RightFoot] - at[Bone::LeftFoot]).dot(rig.forward())
        };
        let walking_swing = lead(&walk_pose_on(0.5, &params, &stood, &rig))
            - lead(&walk_pose_on(0.0, &params, &stood, &rig));

        // Ankles, so the front foot rolling from heel to flat through double
        // support carries its ankle forward and the lead dips. Measured: the
        // walk alone dips 23 mm there; the fading step may dip no more than
        // the walk it fades from.
        //
        // The fade as `Transition` runs it: full weight through the double
        // support after the footfall, then out from the other foot's toe-off
        // to its mid-swing.
        let config = TransitionConfig { mid_swing: mid, ..Default::default() };
        const N: usize = 60;
        let start = lead(&walk_pose_on(0.0, &params, &stood, &rig));
        let (mut previous, mut lowest, mut walk_lowest) = (start, start, start);
        for k in 1..=N {
            let cycle = k as f32 / N as f32 * mid;
            let faded = ((cycle - config.toe_off()) / config.fade()).clamp(0.0, 1.0);
            let walking = walk_pose_on(cycle, &params, &stood, &rig);
            let pose = super::super::clip::blend(&stood, &walking, 1.0 - smoothstep(faded));
            previous = lead(&pose);
            lowest = lowest.min(previous);
            walk_lowest = walk_lowest.min(lead(&walking));
        }
        assert!(
            start - lowest <= start - walk_lowest,
            "the foot stepped back {} m, the walk itself {} m",
            start - lowest,
            start - walk_lowest,
        );
        // Where the first step joins the walk: the swinging foot is passing
        // the planted one. Measured 7 cm ahead at 1.2 m/s, the side-by-side
        // point falling a little before mid-swing.
        let walked = lead(&walk_pose_on(mid, &params, &stood, &rig));
        assert!(
            walked.abs() < 0.1 * walking_swing,
            "at mid-swing the walk's feet are {walked} m apart, of a {walking_swing} m swing",
        );
        let travel = previous - start;
        assert!(previous.abs() < 0.02, "the feet end {previous} m apart");
        assert!(
            (0.35..0.65).contains(&(travel / walking_swing)),
            "the last step travelled {travel} m, a walking swing {walking_swing} m",
        );
    }

    /// The gallery's start, headless: the transition, the release, the pose
    /// the gait blends to, and root motion from that pose's contacts —
    /// springs and IK left out. Returns the stance (left) foot's slip in the
    /// world, frame by frame until its toe-off: each sole point's world
    /// motion, weighted by the load it carries. Rolling from heel to tip is
    /// not slip; a loaded point moving over the ground is.
    ///
    /// The world positions come through forward kinematics, which includes
    /// the pose's `root_translation` — a different path from root motion's
    /// own hips-relative contacts, so this can see what that misses.
    ///
    /// `steady` plays the same stretch of stride fully walking instead, from
    /// the same mid-swing: the reference the first step is held to.
    ///
    /// Also returns the swinging (right) foot's path: per frame, how far its
    /// ball has travelled from where it stood, horizontally, and how high it
    /// is above where it stood.
    fn first_step(steady: bool) -> FirstStep {
        use bevy::math::Vec3;
        use super::super::foot::{shares, Sole};
        use super::super::gait::{walk_pose_on, GaitParams};
        use super::super::locomotion::{distance_per_cycle, root_displacement_between};
        use super::super::rig::forward_kinematics_on;
        use crate::character::skeleton::Bone;

        let rig = super::super::gltf_rig::puppet_base_as_rendered();
        let stood = stance::stance_on_rig(
            &super::super::poses::relaxed_stand(),
            stance::DEFAULT_KNEE_FLEX,
            &rig,
        );
        let speed = 1.2;
        let params = GaitParams::walking_on(speed, &rig);
        let cadence = speed / distance_per_cycle(&params, &stood, &rig);
        let config = TransitionConfig { mid_swing: params.duty_factor * 0.5, ..Default::default() };
        let sole = Sole::of(&rig, Bone::LeftFoot);
        let world_sole = |pose: &LocalPose, body: Vec3| {
            let hips = forward_kinematics_on(pose, &rig)[Bone::Hips];
            sole.points(pose, &rig).map(|p| p + hips + body)
        };

        let mut transition = Transition::standing();
        let (mut cycle, mut body) = (0.0_f32, Vec3::ZERO);
        let (mut previous, mut previous_cycle) = (stood, 0.0_f32);
        if steady {
            transition = Transition { stance: 1.0, ..Transition::walking(cadence) };
            cycle = config.mid_swing;
            (previous, previous_cycle) = (walk_pose_on(cycle, &params, &stood, &rig), cycle);
            cycle += cadence * DT;
        }
        let stood_ball = forward_kinematics_on(&stood, &rig)[Bone::RightToeBase];
        let mut out = FirstStep { slips: Vec::new(), swing: Vec::new() };
        for _ in 0..600 {
            if let Some(TransitionEvent::FirstStep { cycle: start }) =
                transition.advance(speed, cycle, &config, DT)
            {
                cycle = start;
            }
            let weight = transition.weight;
            let mut prepared = stood;
            transition.apply_release(&mut prepared, &rig);
            let pose = if weight <= 0.0 {
                prepared
            } else {
                transition.blend(&prepared, &walk_pose_on(cycle, &params, &stood, &rig), &rig)
            };
            let before = world_sole(&previous, body);
            if weight > 0.0 {
                let middle = previous_cycle + 0.5 * (cycle - previous_cycle).rem_euclid(1.0);
                body += root_displacement_between(&previous, &pose, middle, &params, &rig)
                    .unwrap_or(Vec3::ZERO);
                let after = world_sole(&pose, body);
                let ball = forward_kinematics_on(&pose, &rig)[Bone::RightToeBase] + body - stood_ball;
                if cycle < 0.5 {
                    out.swing.push((Vec3::new(ball.x, 0.0, ball.z).length(), ball.y));
                }
                let load = shares(&[0, 1, 2].map(|i| (before[i] + after[i]) * 0.5));
                let slip: Vec3 = (0..3).map(|i| (after[i] - before[i]) * load[i]).sum();
                out.slips.push(Vec3::new(slip.x, 0.0, slip.z).length());
                if cycle >= params.duty_factor {
                    break;
                }
            }
            (previous, previous_cycle) = (pose, cycle);
            cycle = (cycle + cadence * DT * f32::from(transition.release >= 1.0 || weight > 0.0))
                .rem_euclid(1.0);
        }
        assert_eq!(transition.stance, 1.0, "test setup: the left leg stands");
        out
    }

    /// The gallery's stop, headless, like [`first_step`]: walking steadily,
    /// then asked to stand. Returns the swinging foot's ball per frame from
    /// the footfall that starts the last step until well after rest: how far
    /// it still has to go to where it ends, horizontally, and its height
    /// above where it ends.
    fn last_step() -> Vec<(f32, f32)> {
        use bevy::math::Vec3;
        use super::super::gait::{walk_pose_on, GaitParams};
        use super::super::locomotion::{distance_per_cycle, root_displacement_between};
        use super::super::rig::forward_kinematics_on;
        use crate::character::skeleton::Bone;

        let rig = super::super::gltf_rig::puppet_base_as_rendered();
        let stood =
            stance::stance_on_rig(&super::super::poses::relaxed_stand(), stance::DEFAULT_KNEE_FLEX, &rig);
        let speed = 1.2;
        let params = GaitParams::walking_on(speed, &rig);
        let cadence = speed / distance_per_cycle(&params, &stood, &rig);
        let config = TransitionConfig { mid_swing: params.duty_factor * 0.5, ..Default::default() };

        // Walking, just before the left footfall; the right leg swings last.
        let mut transition = Transition { stance: 1.0, ..Transition::walking(cadence) };
        let mut cycle = 0.95_f32;
        let mut previous = walk_pose_on(cycle, &params, &stood, &rig);
        let mut previous_cycle = cycle;
        let mut body = Vec3::ZERO;
        let mut path = Vec::new();
        for _ in 0..240 {
            transition.advance(0.0, cycle, &config, DT);
            let weight = transition.weight;
            let pose = if weight <= 0.0 {
                stood
            } else {
                transition.blend(&stood, &walk_pose_on(cycle, &params, &stood, &rig), &rig)
            };
            if weight > 0.0 || previous_cycle != cycle {
                let middle = previous_cycle + 0.5 * (cycle - previous_cycle).rem_euclid(1.0);
                body += root_displacement_between(&previous, &pose, middle, &params, &rig).unwrap_or(Vec3::ZERO);
            }
            path.push(forward_kinematics_on(&pose, &rig)[Bone::RightToeBase] + body);
            (previous, previous_cycle) = (pose, cycle);
            cycle = (cycle + transition.cadence_hz.max(0.0) * DT * f32::from(weight > 0.0)).rem_euclid(1.0);
            let _ = cadence;
        }
        let end = *path.last().unwrap();
        path.iter()
            .map(|p| (Vec3::new(p.x - end.x, 0.0, p.z - end.z).length(), p.y - end.y))
            .collect()
    }

    #[test]
    fn a_stop_publishes_its_landing_through_the_fade_and_a_hold_after() {
        // The foot IK sets the last swing down on the RENDERED foot, which
        // lags the target through the springs: the landing has to outlast
        // the fade by `LAND_HOLD`, then let go.
        let rig = RigGeometry::default();
        let stood = LocalPose::REST;
        let config = TransitionConfig::default();
        let mut transition = Transition { stance: 1.0, ..Transition::walking(1.0) };
        let mut phase = 0.95_f32;
        let (mut faded_at, mut released_at, mut during_fade) = (None, None, false);
        for frame in 0..240 {
            let event = transition.advance(0.0, phase, &config, DT);
            let landing = transition.landing(&stood, &rig);
            if transition.last_swing > 0.0 {
                during_fade |= landing.is_some_and(|l| !l.left);
            }
            if matches!(event, Some(TransitionEvent::AtRest)) {
                faded_at = Some(frame);
            }
            if faded_at.is_some() && released_at.is_none() && landing.is_none() {
                released_at = Some(frame);
            }
            phase = (phase + transition.cadence_hz * DT * f32::from(transition.weight > 0.0)).rem_euclid(1.0);
        }
        assert!(during_fade, "through the fade the swinging (right) foot should be landing");
        let (faded, released) = (faded_at.expect("the stop comes to rest"), released_at.expect("the landing lets go"));
        let held = (released - faded) as f32 * DT;
        assert!((held - LAND_HOLD).abs() <= DT + 1.0e-4, "the landing was held {held} s after the fade, not {LAND_HOLD}");
    }

    #[test]
    fn the_last_swing_is_set_down_onto_its_spot() {
        // The last step fades a whole swing into standing in ~0.4 s. The
        // blended foot used to come down on the way — 258 mm short at 13 mm
        // up, 74 mm short 5 mm INTO the floor — and live the foot IK slid it
        // along the floor to its spot (the stop's "glide"). Now it meets the
        // floor only over its spot.
        let path = last_step();
        for &(away, up) in &path {
            assert!(up > -1.0e-3, "the last swing's ball went {:.1} mm into the floor, {:.0} mm short", up * 1e3, away * 1e3);
            if up < 0.003 {
                assert!(
                    away < 0.02,
                    "the last swing's ball was down ({:.1} mm up) {:.0} mm short of its spot",
                    up * 1e3,
                    away * 1e3
                );
            }
        }
        assert!(path.iter().any(|&(away, _)| away > 0.5), "test setup: the swing should start well behind its spot");
    }

    /// What [`first_step`] measured.
    struct FirstStep {
        /// The stance foot's slip per frame, metres.
        slips: Vec<f32>,
        /// The swinging foot's ball per frame until its heel contact:
        /// horizontal travel from where it stood, and height above it.
        swing: Vec<(f32, f32)>,
    }

    #[test]
    fn the_first_swing_lifts_before_it_travels() {
        // Joining the walk at mid-swing — where a walking foot passes
        // closest to the floor at its fastest — and fading it in from a
        // standing foot, the first swing skimmed 94 mm along the floor
        // before it rose 2 mm. It is now lifted (`FIRST_SWING_LIFT`). Once
        // the ball has moved off its spot, it must be clear of the floor.
        let swing = first_step(false).swing;
        let skimming: Vec<_> =
            swing.iter().filter(|(travel, lift)| *travel > 0.02 && *lift < 0.01).collect();
        assert!(
            skimming.is_empty(),
            "the swinging ball travelled more than 2 cm while under 1 cm up, (travel, lift) m: {skimming:.3?}",
        );
    }

    #[test]
    fn the_first_step_keeps_the_stance_foot_planted() {
        // The release shifts the hips 4.5 cm sideways and 4 cm forward
        // through `root_translation`, and the first step fades that out.
        // Root motion once measured contacts from the hips only, missed it,
        // and walked the planted foot 47 mm here (~13 cm live).
        //
        // Held to the steady walk over the same stretch of stride (mid-swing
        // to the stance foot's toe-off), which slips too, in the hand-over
        // as the foot unloads (see `walk.rs`). Fading the gait through double
        // support, where the walk fixes the distance between the feet and a
        // changing blend scales it, slipped 35 mm, 12.5 mm in one frame.
        let total = |slips: &[f32]| slips.iter().sum::<f32>();
        let first = first_step(false).slips;
        let steady = first_step(true).slips;
        assert!(first.len() > 10, "the first step ran {} frames", first.len());
        assert!(
            total(&first) <= total(&steady) + 0.002,
            "the stance foot slipped {:.1} mm over the first step; the steady walk {:.1} mm",
            total(&first) * 1000.0,
            total(&steady) * 1000.0,
        );
    }

    #[test]
    fn the_release_moves_the_pelvis_onto_the_stance_leg_and_forward_over_still_feet() {
        use super::super::rig::forward_kinematics_on;
        use crate::character::skeleton::Bone;

        let rig = super::super::gltf_rig::puppet_base_as_rendered();
        let stood = stance::stance_on_rig(
            &super::super::poses::relaxed_stand(),
            stance::DEFAULT_KNEE_FLEX,
            &rig,
        );
        // Both sides: the gallery's first start stands on the LEFT leg, and a
        // right-only version of this test passed while the live left toe
        // moved 4.2 cm under a release — hidden by the standing foot lock
        // until the first step let go of it.
        for side in [1.0, -1.0] {
            let released = Transition {
                release: 1.0,
                stance: side,
                stride_speed: super::super::reference::SPEED,
                ..Transition::standing()
            };
            let mut pose = stood;
            released.apply_release(&mut pose, &rig);

            let moved = pose.root_translation - stood.root_translation;
            assert!(
                (moved.dot(rig.left()) - side * stance::WEIGHT_SHIFT).abs() < 0.005,
                "{side}: toward the stance leg: {}",
                moved.dot(rig.left()),
            );
            assert!(
                (moved.dot(rig.forward()) - RELEASE_LEAN).abs() < 0.005,
                "{side}: forward: {}",
                moved.dot(rig.forward()),
            );
            let (before, after) = (forward_kinematics_on(&stood, &rig), forward_kinematics_on(&pose, &rig));
            for toe in [Bone::LeftToeBase, Bone::RightToeBase] {
                let slid = after[toe] - before[toe];
                assert!(slid.length() < 0.002, "{side}: {toe:?} moved {slid:?}");
            }
        }
    }

    #[test]
    fn starting_reaches_a_full_walk() {
        let mut transition = Transition::standing();
        let config = TransitionConfig::default();

        run(&mut transition, 1.2, 1.0, &config);

        assert_eq!(transition.weight, 1.0, "the gait should be fully applied");
    }

    #[test]
    fn a_walk_that_never_stops_stays_at_full_weight() {
        let mut transition = Transition::walking(cadence_for(1.2));
        let config = TransitionConfig::default();

        run(&mut transition, 1.2, 5.0, &config);

        assert_eq!(transition.weight, 1.0);
    }

    // -----------------------------------------------------------------
    // Edges
    // -----------------------------------------------------------------

    #[test]
    fn the_weight_never_leaves_its_range() {
        let config = TransitionConfig::default();
        let mut transition = Transition::standing();
        let mut phase = 0.0;

        // Flip between walking and stopping repeatedly.
        for i in 0..600 {
            let speed = if (i / 30) % 2 == 0 { 1.5 } else { 0.0 };
            transition.advance(speed, phase, &config,DT);
            phase = (phase + transition.cadence_hz * DT).rem_euclid(1.0);

            assert!(
                (0.0..=1.0).contains(&transition.weight),
                "weight left its range at step {i}: {}",
                transition.weight,
            );
            assert!(
                transition.cadence_hz >= 0.0 && transition.cadence_hz.is_finite(),
                "cadence went bad at step {i}: {}",
                transition.cadence_hz,
            );
        }
    }

    #[test]
    fn a_non_positive_timestep_changes_nothing() {
        let config = TransitionConfig::default();
        let mut transition = Transition::walking(1.0);

        let before = transition;
        transition.advance(0.0, 0.3, &config,0.0);
        assert_eq!(transition, before);

        transition.advance(0.0, 0.3, &config,f32::NAN);
        assert_eq!(transition, before);
    }

    #[test]
    fn a_zero_blend_time_is_instant_rather_than_stuck() {
        let config = TransitionConfig {
            blend_seconds: 0.0,
            release_seconds: 0.0,
            // Mid-swing at heel contact leaves no single support to fade
            // through: `fade()` is zero.
            mid_swing: 0.5,
            ..Default::default()
        };

        // One frame to release, one to walk: the first step needs the
        // clock moved between them.
        let mut transition = Transition::standing();
        transition.advance(1.2, 0.0, &config, DT);
        transition.advance(1.2, 0.0, &config, DT);

        assert_eq!(transition.weight, 1.0, "a zero blend should arrive at once");
    }

    #[test]
    fn cadence_for_is_zero_at_zero_speed() {
        // The whole point of not reusing `GaitPhase::gait_frequency_hz`,
        // whose 1/7 Hz floor is what kept a stopped character stepping.
        assert_eq!(cadence_for(0.0), 0.0);
        assert!(cadence_for(1.2) > 1.0);
        assert_eq!(cadence_for(-1.2), cadence_for(1.2), "walking backward has a cadence");
    }
}
