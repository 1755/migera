//! A push while walking: the walk's next footfalls placed to catch it.
//!
//! A walking body is the same inverted pendulum as a standing one
//! ([`super::balance`]), `COM̈ = (COM − COP)/K`, but its COP is the stance
//! foot, and a new one comes every half stride. Walkers recover from a push
//! by where they put that next foot, not by holding still over the old one:
//! Hof et al. (2005) found the COP placed at a near-constant offset from the
//! extrapolated centre of mass `XcoM = COM + ẋ·√K`, the capture point.
//!
//! [`WalkBalance`] models only the push's DIFFERENCE from the walk. The walk
//! itself (its COM path, its footfalls) is the animation's and is left as it
//! is. The difference is linear in the pendulum: a COM offset `offset` from
//! the walk's own path, its velocity, and each foot's offset from where the
//! walk would have put it (`feet`). Through each stance the offset follows
//! the pendulum about the stance foot's offset, with a small ankle term
//! (the COP within the foot, [`ANKLE_REACH`]). Each swinging foot aims at
//! where the offset's capture point will be when it lands. Landed there,
//! the offset is on the pendulum's stable branch and comes to rest over the
//! new foot: the walk goes on, displaced by the push.
//!
//! # Forward: the walk speeds up
//!
//! A push from behind is not caught that way. A walking step is already
//! most of a leg long, so the longer step it would ask for is out of reach:
//! 0.4 m/s from behind early in a stance wanted a 1.16 m step, then ever
//! longer ones, and fell. A walker pushed from behind walks faster for a few
//! steps instead. The forward part of a push becomes a [surge](WalkBalance::surge)
//! in walking speed, which decays over [`SURGE_SECONDS`]; the caller adds
//! it to the speed the walk is asked for. A surge past [`MAX_SURGE`] is
//! lost.
//!
//! Axes are the rig's own, horizontal: `x` along
//! [`RigGeometry::forward`](super::rig::RigGeometry::forward), `y` along
//! [`RigGeometry::left`](super::rig::RigGeometry::left). Hof's offset is a
//! later formalisation of Winter's pendulum, not Winter's.

use bevy::math::Vec2;

use super::balance::{PUSH_SECONDS, RECOVERY_DAMPING, RECOVERY_FREQUENCY};
use super::gait::{leg_phase, LegPhase};

/// How far the COP may move within the stance foot to steer the offset,
/// metres (forward, sideways), either way. A walking stance foot's COP is
/// already rolling heel to toe; this is the little left to spare.
pub const ANKLE_REACH: Vec2 = Vec2::new(0.03, 0.015);

/// The narrowest step, metres sideways from the stance foot to the landing
/// one, on the landing foot's own side: negative, a crossover. Hof et al.
/// (2010) place the next foot a fixed distance outward of the capture point
/// whichever way the push went; that a foot may cross in front of the
/// stance foot to get there is this module's extrapolation, bounded here.
/// Held at its own side instead, a 0.4 m/s push early in the stance went a
/// whole step uncaught and was lost. The swing moves inward late (see
/// [`WalkBalance::step`]), once past the stance foot, so the feet never
/// meet.
pub const MIN_STEP_WIDTH: f32 = -0.15;

/// How long before its footfall a swinging foot's placement is settled,
/// seconds: a push after that cannot move this step, only the next. Hof et
/// al. (2010) measured the stepping strategy to need at least 300 ms before
/// foot placement (their ankle strategy, ~200 ms, is [`ANKLE_REACH`]).
pub const PLACEMENT_DELAY: f32 = 0.3;

/// How far through its swing a foot starts moving inward to its landing:
/// at mid-swing, where it passes the stance foot, it has gone a fifth of
/// the way.
const INWARD_FROM: f32 = 0.3;

/// The offset from the support at which the body is lost, metres: a lean
/// of about 25° at a 1 m COM height, past what the next foot can still be
/// put under. Through one walking stance the body may stray far past what
/// an ankle holds (Winter's 8° bound, 0.13 m) and still be caught by the
/// footfall that ends it: at 0.3 m, pushes of 0.3 m/s were lost mid-stance
/// that the next foot then caught. Its exponential divergence makes the
/// forecast insensitive to the exact value. This module's choice.
pub const LOST_OFFSET: f32 = 0.45;

/// How long a surge in walking speed takes to die away, seconds: its
/// exponential time constant, about two steps. This module's choice.
pub const SURGE_SECONDS: f32 = 1.0;

/// The largest surge a walker catches, m/s: the forward push the standing
/// stumble catches (1.5 m/s, measured on both rigs). Faster, it is lost.
pub const MAX_SURGE: f32 = 1.5;

/// How far ahead a push is forecast, seconds: several steps.
pub const FORECAST_SECONDS: f32 = 3.0;

/// The longest tick, seconds: a foot lands between ticks.
const MAX_TICK: f32 = 1.0 / 60.0;

/// The walk the push lands in, as the balance needs it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stride {
    /// One cycle (two steps), seconds.
    pub seconds: f32,
    /// The share of the cycle a foot is down (`GaitParams::duty_factor`).
    pub duty_factor: f32,
    /// How far ahead of the stance foot the walk puts the next one, metres.
    pub step_length: f32,
    /// How far apart sideways the feet stand, metres.
    pub step_width: f32,
    /// The longest step, foot to foot, metres: the leg's length
    /// ([`super::gait::leg_length_of`]), each leg then 30° off vertical.
    /// The standing stumble's [`MAX_STEP`](super::balance::MAX_STEP) is no limit here: it is a step
    /// from feet side by side, and a walking step is already nearly that.
    pub max_step: f32,
}

/// The push's difference from the walk. See the module docs.
#[derive(bevy::ecs::component::Component, Debug, Clone, Copy, PartialEq, Default)]
pub struct WalkBalance {
    /// The COM's offset from the walk's own path, metres.
    pub offset: Vec2,
    /// Its velocity, m/s.
    pub velocity: Vec2,
    /// Each foot's offset from where the walk puts it (left, right), metres:
    /// fixed while it is down, moving to its landing while it swings.
    pub feet: [Vec2; 2],
    /// The COP's offset on the last tick: the stance foot's, plus the ankle's.
    pub pressure: Vec2,
    /// Set once no footfall the walk can make catches the body: the
    /// forecast lost it ([`LOST_OFFSET`]), or the surge passed
    /// [`MAX_SURGE`]. It is falling.
    pub falls: bool,
    /// How much faster than asked the walk is going, m/s: the forward part
    /// of the pushes. See the module docs.
    pub surge: f32,
    /// Surge still to be delivered, m/s.
    surge_pending: f32,
    /// The rate it is delivered at, m/s².
    surge_rate: f32,
    /// How long the longest step the push asked for was, metres, before
    /// [`Stride::max_step`] clamped it.
    pub wanted_step: f32,
    /// Where each foot's offset was as it lifted.
    lift_off: [Vec2; 2],
    /// Where each swinging foot is aimed, once its placement is settled
    /// ([`PLACEMENT_DELAY`]).
    settled_aim: [Option<Vec2>; 2],
    /// Each leg's phase on the last tick.
    legs: Option<[LegPhase; 2]>,
    /// Velocity still to be delivered by pushes under way, m/s.
    shove: Vec2,
    /// The rate it is delivered at, m/s².
    shove_rate: Vec2,
    /// The offset as last handed to the caller ([`WalkBalance::moved`]).
    shown: Vec2,
    /// Set on a copy run ahead (it does not look ahead itself).
    forecasting: bool,
}

impl WalkBalance {
    /// A push: a change in the COM's velocity, m/s, delivered over
    /// [`PUSH_SECONDS`], as [`super::balance::Balance::push`]. Its forward
    /// part is a [surge](WalkBalance::surge).
    pub fn push(&mut self, velocity: Vec2) {
        self.surge_pending += velocity.x.max(0.0);
        self.surge_rate = self.surge_pending / PUSH_SECONDS;
        self.shove += Vec2::new(velocity.x.min(0.0), velocity.y);
        self.shove_rate = self.shove / PUSH_SECONDS;
    }

    /// The part of the pushes given that has not been delivered yet, m/s.
    pub fn pending_push(&self) -> Vec2 {
        self.shove + Vec2::X * self.surge_pending
    }

    /// Whether the push is spent: no surge, the offset at rest over the
    /// feet, which stand where it is, within `tolerance` metres and metres
    /// per second.
    pub fn is_settled(&self, tolerance: f32) -> bool {
        self.shove == Vec2::ZERO
            && self.surge_pending == 0.0
            && self.surge.abs() <= tolerance
            && self.velocity.length() <= tolerance
            && self.feet.iter().all(|foot| foot.distance(self.offset) <= tolerance)
    }

    /// How far the COM's offset has moved since this was last called: the
    /// caller moves the character by it, as root motion.
    pub fn moved(&mut self) -> Vec2 {
        let moved = self.offset - self.shown;
        self.shown = self.offset;
        moved
    }

    /// Where foot `leg` goes relative to where the walk puts it, as seen
    /// from the moved character: its offset less the body's.
    pub fn foot_displacement(&self, leg: usize) -> Vec2 {
        self.feet[leg] - self.shown
    }

    /// Advances by `dt` seconds, the walk's cycle running from `from` to
    /// `to` (fractions, either may have wrapped), on pendulum constant `k`
    /// (s², [`super::balance::pendulum_k`]).
    pub fn step(&mut self, stride: &Stride, from: f32, to: f32, k: f32, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() || k <= 0.0 || stride.seconds <= 0.0 {
            return;
        }
        let ticks = (dt / MAX_TICK).ceil().max(1.0) as usize;
        let advance = (to - from).rem_euclid(1.0);
        for tick in 0..ticks {
            let cycle = from + advance * (tick + 1) as f32 / ticks as f32;
            self.tick(stride, cycle, k, dt / ticks as f32);
        }
        if !self.forecasting && !self.falls && !self.is_settled(1.0e-4) && !self.caught(stride, to, k) {
            self.falls = true;
        }
    }

    /// One tick, the cycle now at `cycle`.
    fn tick(&mut self, stride: &Stride, cycle: f32, k: f32, dt: f32) {
        let duty = stride.duty_factor;
        let legs = [leg_phase(cycle, duty), leg_phase(cycle + 0.5, duty)];
        let before = self.legs.unwrap_or(legs);
        for leg in 0..2 {
            if before[leg].is_stance() && !legs[leg].is_stance() {
                self.lift_off[leg] = self.feet[leg];
                self.settled_aim[leg] = None;
            }
        }
        self.legs = Some(legs);

        // The push, as it is delivered.
        if self.shove != Vec2::ZERO {
            let rate = self.shove_rate * dt;
            let delivered = if rate.length() >= self.shove.length() { self.shove } else { rate };
            self.velocity += delivered;
            self.shove -= delivered;
        }
        self.surge *= (-dt / SURGE_SECONDS).exp();
        if self.surge_pending > 0.0 {
            let delivered = (self.surge_rate * dt).min(self.surge_pending);
            self.surge += delivered;
            self.surge_pending -= delivered;
        }
        if self.surge > MAX_SURGE {
            self.falls = true;
        }
        if self.surge_pending == 0.0 && self.surge < 1.0e-4 {
            self.surge = 0.0;
        }

        // The support: the stance foot's offset, or between the two through
        // double support as the weight moves onto the leading foot.
        let since_footfall = |leg: usize| match legs[leg] {
            LegPhase::Stance { progress } => Some(progress * duty),
            LegPhase::Swing { .. } => None,
        };
        let double = (duty - 0.5).max(1.0e-3);
        let support = match (since_footfall(0), since_footfall(1)) {
            (Some(left), Some(right)) => {
                let (leading, trailing, since) = if left < right { (0, 1, left) } else { (1, 0, right) };
                Some(self.feet[trailing].lerp(self.feet[leading], (since / double).clamp(0.0, 1.0)))
            }
            (Some(_), None) => Some(self.feet[0]),
            (None, Some(_)) => Some(self.feet[1]),
            (None, None) => None,
        };

        // The pendulum about it, the ankle steering within the foot as the
        // standing law does (critically damped at `RECOVERY_FREQUENCY`).
        // Stepped exactly for a COP held through the tick: an explicit step
        // of this unstable pendulum put a footfall 2 cm elsewhere at 1/60 s
        // than at 1/240. No foot down, a run's flight: the body coasts.
        match support {
            Some(support) => {
                self.pressure = ankle_pressure(self.offset, self.velocity, support, k);
                let (offset, velocity) = pendulum(self.offset, self.velocity, self.pressure, k, dt);
                self.offset = offset;
                self.velocity = velocity;
            }
            None => self.offset += self.velocity * dt,
        }

        // Each swinging foot: aimed at the capture point it will land on.
        let swing_seconds = (1.0 - duty) * stride.seconds;
        for (leg, phase) in legs.into_iter().enumerate() {
            let LegPhase::Swing { progress } = phase else { continue };
            let stance = 1 - leg;
            let about = support.unwrap_or(self.feet[stance]);
            let left = (1.0 - progress) * swing_seconds;
            // Aimed at the capture point predicted for the footfall, until
            // `PLACEMENT_DELAY` before it; held there after. Until a push
            // comes the aim is the walk's own spot, so a push inside the
            // delay changes nothing about this step.
            let aim = match self.settled_aim[leg] {
                Some(aim) if left < PLACEMENT_DELAY => aim,
                _ => {
                    let target = self.capture_point_after(about, k, left);
                    let aim = self.reachable(stride, leg, target);
                    if left < PLACEMENT_DELAY {
                        self.settled_aim[leg] = Some(aim);
                    }
                    aim
                }
            };
            let ease = |t: f32| t * t * (3.0 - 2.0 * t);
            let along = self.lift_off[leg].x + (aim.x - self.lift_off[leg].x) * ease(progress);
            // Sideways it moves outward from the start, inward only late.
            let side = nominal_step(stride, leg).y.signum();
            let inward = side * (aim.y - self.lift_off[leg].y) < 0.0;
            let across = if inward { ease(((progress - INWARD_FROM) / (1.0 - INWARD_FROM)).max(0.0)) } else { ease(progress) };
            self.feet[leg] = Vec2::new(along, self.lift_off[leg].y + (aim.y - self.lift_off[leg].y) * across);
        }
    }

    /// Where the capture point `offset + velocity·√K` will be after
    /// `seconds` of the pendulum about `support`, the ankle steering as
    /// it does. Predicted without the ankle, an aim held from
    /// [`PLACEMENT_DELAY`] out landed every foot past the body, and a
    /// 0.5 m/s push back was still rocking between the feet 6 s later.
    fn capture_point_after(&self, support: Vec2, k: f32, seconds: f32) -> Vec2 {
        let ticks = (seconds / MAX_TICK).ceil().max(1.0);
        let dt = seconds / ticks;
        let (mut at, mut velocity) = (self.offset, self.velocity);
        for _ in 0..ticks as usize {
            (at, velocity) = pendulum(at, velocity, ankle_pressure(at, velocity, support, k), k, dt);
        }
        at + velocity * k.sqrt()
    }

    /// `target` for swinging foot `leg`, limited to a step the walk can
    /// take from the stance foot: at most [`Stride::max_step`] long and no
    /// closer sideways than [`MIN_STEP_WIDTH`]. Records
    /// [`WalkBalance::wanted_step`].
    fn reachable(&mut self, stride: &Stride, leg: usize, target: Vec2) -> Vec2 {
        let stance = self.feet[1 - leg];
        let nominal = nominal_step(stride, leg);
        let mut step = nominal + (target - stance);
        self.wanted_step = self.wanted_step.max(step.length());
        // Sideways first, then the length from what is left: scaled whole,
        // a long step's width shrank with it and crossed over.
        let side = nominal.y.signum();
        step.y = side * (side * step.y).clamp(MIN_STEP_WIDTH, stride.max_step);
        let along = (stride.max_step * stride.max_step - step.y * step.y).max(0.0).sqrt();
        step.x = step.x.clamp(-along, along);
        stance + step - nominal
    }

    /// Whether the walk's footfalls catch the body: run ahead
    /// [`FORECAST_SECONDS`], it never strays [`LOST_OFFSET`] from its
    /// support.
    fn caught(&self, stride: &Stride, cycle: f32, k: f32) -> bool {
        let mut ahead = Self { forecasting: true, ..*self };
        let dt = MAX_TICK;
        let mut at = cycle;
        for _ in 0..(FORECAST_SECONDS / dt) as usize {
            let next = at + dt / stride.seconds;
            ahead.step(stride, at, next, k, dt);
            at = next;
            if (ahead.offset - ahead.pressure).length() > LOST_OFFSET {
                return false;
            }
            if ahead.is_settled(1.0e-4) {
                return true;
            }
        }
        true
    }

    /// Ends the push's difference once it is spent: what remains is the
    /// common offset the character has already been moved by.
    pub fn settle(&mut self, tolerance: f32) {
        if self.is_settled(tolerance) && self.shown == self.offset {
            *self = Self::default();
        }
    }
}

/// The step the walk itself takes with leg `leg` (0 the left), from the
/// stance foot to the landing one, metres: forward by the step length, out
/// to its own side by the step width.
pub fn nominal_step(stride: &Stride, leg: usize) -> Vec2 {
    let side = if leg == 0 { 1.0 } else { -1.0 };
    Vec2::new(stride.step_length, side * stride.step_width)
}

/// Where the ankle puts the COP to steer `offset`, `velocity` back over
/// `support`: the standing law, critically damped at
/// [`RECOVERY_FREQUENCY`], within [`ANKLE_REACH`] of the support.
fn ankle_pressure(offset: Vec2, velocity: Vec2, support: Vec2, k: f32) -> Vec2 {
    let omega = RECOVERY_FREQUENCY;
    let wish = (offset - support) * (1.0 + k * omega * omega) + velocity * (2.0 * RECOVERY_DAMPING * omega * k);
    support + wish.clamp(-ANKLE_REACH, ANKLE_REACH)
}

/// The pendulum `COM̈ = (COM − COP)/K` run `seconds` from `offset`,
/// `velocity` over a COP held at `pressure`, exactly: `(offset, velocity)`.
fn pendulum(offset: Vec2, velocity: Vec2, pressure: Vec2, k: f32, seconds: f32) -> (Vec2, Vec2) {
    let root = k.sqrt();
    let (sinh, cosh) = ((seconds / root).sinh(), (seconds / root).cosh());
    let x = offset - pressure;
    (pressure + x * cosh + velocity * root * sinh, x / root * sinh + velocity * cosh)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A brisk adult walk on `puppet_base`'s scale: 1.1 s a cycle, 62%
    /// stance, 0.65 m steps, feet 0.18 m apart, an 0.89 m leg, `K` 0.095 s².
    const STRIDE: Stride =
        Stride { seconds: 1.1, duty_factor: 0.62, step_length: 0.65, step_width: 0.18, max_step: 0.89 };
    const K: f32 = 0.095;

    /// Runs `seconds` of walk from cycle `start`, frames of `dt`, pushing
    /// `push` at the start. Returns the balance and the largest offset from
    /// the support seen.
    fn walk(push: Vec2, start: f32, seconds: f32, dt: f32) -> (WalkBalance, f32) {
        let mut balance = WalkBalance::default();
        balance.push(push);
        let mut cycle = start;
        let mut worst: f32 = 0.0;
        for _ in 0..(seconds / dt) as usize {
            let next = cycle + dt / STRIDE.seconds;
            balance.step(&STRIDE, cycle, next, K, dt);
            cycle = next;
            worst = worst.max((balance.offset - balance.pressure).length());
        }
        (balance, worst)
    }

    #[test]
    fn a_push_while_walking_is_caught_by_the_next_footfalls() {
        // Back and to either side, from early and late in a step: the walk
        // goes on, the body at rest over its displaced feet. How hard a push
        // is caught depends on when it lands (`probe_walking_catch_limits`):
        // inside `PLACEMENT_DELAY` of a footfall it can only be caught by
        // the step after, most of a stride later, and is weakest. So: small
        // pushes at every phase, stronger ones early in a stance, when the
        // swinging foot can still be placed.
        let cases = [
            (Vec2::new(0.0, 0.1), [0.05, 0.3, 0.55, 0.8].as_slice()),
            (Vec2::new(0.0, -0.1), &[0.05, 0.3, 0.55, 0.8]),
            (Vec2::new(-0.3, 0.0), &[0.05, 0.3, 0.55, 0.8]),
            (Vec2::new(0.0, 0.3), &[0.1, 0.6]),
            (Vec2::new(0.0, -0.3), &[0.1, 0.6]),
            (Vec2::new(-0.7, 0.0), &[0.1, 0.6]),
            (Vec2::new(-0.3, 0.2), &[0.1, 0.6]),
        ];
        for (push, starts) in cases {
            for &start in starts {
                let (balance, worst) = walk(push, start, 6.0, 1.0 / 60.0);
                assert!(!balance.falls, "push {push} at cycle {start} should be caught");
                assert!(
                    balance.is_settled(2.0e-3),
                    "push {push} at {start}: still moving after 6 s: offset {} velocity {} feet {:?}",
                    balance.offset,
                    balance.velocity,
                    balance.feet
                );
                assert!(worst < LOST_OFFSET, "push {push} at {start} strayed {worst:.3} m from its support");
                // The body has gone with the push: the walk is displaced the
                // way it was pushed, not pulled back to where it was.
                assert!(balance.offset.dot(push) > 0.0, "push {push} at {start} ended at {}", balance.offset);
            }
        }
    }

    #[test]
    fn a_push_from_behind_speeds_the_walk_up_and_dies_away() {
        let mut balance = WalkBalance::default();
        balance.push(Vec2::new(0.6, 0.2));
        let dt = 1.0 / 60.0;
        let mut cycle: f32 = 0.05;
        let mut peak: f32 = 0.0;
        for frame in 0..(6.0 / dt) as usize {
            let next = cycle + dt / STRIDE.seconds;
            balance.step(&STRIDE, cycle, next, K, dt);
            cycle = next;
            peak = peak.max(balance.surge);
            // Its forward part never moves the body off the walk's path:
            // it is all in the speed.
            // (Only a sideways step long enough that the leg's reach
            // shortens it moves it at all.)
            assert!(balance.offset.x.abs() < 0.01, "frame {frame}: forward offset {}", balance.offset.x);
            if frame == (SURGE_SECONDS / dt) as usize + (PUSH_SECONDS / dt) as usize {
                assert!(
                    (balance.surge / peak - (-1.0f32).exp()).abs() < 0.03,
                    "a time constant on, the surge is {:.3} of its {peak:.3} peak",
                    balance.surge / peak
                );
            }
        }
        assert!(peak > 0.55 && peak <= 0.6, "the surge peaked at {peak}");
        assert!(!balance.falls && balance.is_settled(2.0e-3), "{balance:?}");
        assert!(balance.offset.y > 0.0, "the sideways part was caught by the feet: {}", balance.offset);

        // Past what a stumble catches standing, it is lost.
        let (lost, _) = walk(Vec2::new(MAX_SURGE + 0.1, 0.0), 0.3, 1.0, dt);
        assert!(lost.falls);
    }

    #[test]
    fn without_its_footfalls_the_same_push_is_not_caught() {
        // The control: the stance foot alone (the ankle, no step placed)
        // loses the body. Proves the catch above is the footfalls' doing.
        let mut balance = WalkBalance::default();
        balance.push(Vec2::new(0.0, 0.4));
        let mut worst: f32 = 0.0;
        let dt = 1.0 / 60.0;
        for _ in 0..120 {
            balance.forecasting = true;
            // Frozen mid-stance of the right foot: no foot ever swings.
            balance.step(&STRIDE, 0.75, 0.75, K, dt);
            worst = worst.max((balance.offset - balance.pressure).length());
        }
        assert!(worst > LOST_OFFSET, "the ankle alone held it within {worst:.3} m");
    }

    #[test]
    fn a_swinging_foot_lands_on_the_capture_point() {
        // Hof's rule: at footfall, the offset's capture point stands on the
        // new foot, so the offset is on the pendulum's stable branch.
        let mut balance = WalkBalance::default();
        balance.push(Vec2::new(-0.3, 0.2));
        let dt = 1.0 / 240.0;
        // Right foot in stance (left swinging) through cycle 0.62..1.0. The
        // push starts just before the left lifts, so it is all delivered
        // before the left's placement settles (`PLACEMENT_DELAY`).
        let mut cycle = 0.6;
        while cycle < 0.999 {
            let next = (cycle + dt / STRIDE.seconds).min(0.999);
            balance.step(&STRIDE, cycle, next, K, dt);
            cycle = next;
        }
        let capture = balance.offset + balance.velocity * K.sqrt();
        let landed = balance.feet[0];
        assert!(
            capture.distance(landed) < 0.02,
            "the left foot landed at {landed}, the capture point is at {capture}"
        );
    }

    #[test]
    fn a_push_too_hard_for_any_step_falls() {
        let (balance, _) = walk(Vec2::new(0.0, 2.5), 0.3, 1.0, 1.0 / 60.0);
        assert!(balance.falls, "a 2.5 m/s side push while walking should be lost");
    }

    #[test]
    fn a_step_never_crosses_too_far_or_overreaches() {
        // Pushed toward the stance leg, the swinging foot crosses over no
        // further than `MIN_STEP_WIDTH`; pushed hard, no step is longer
        // than the leg.
        for push in [Vec2::new(0.0, -0.8), Vec2::new(0.0, 0.8), Vec2::new(-1.0, 0.0)] {
            let mut balance = WalkBalance::default();
            balance.push(push);
            let dt = 1.0 / 60.0;
            let mut cycle: f32 = 0.0;
            let mut landings = 0;
            for _ in 0..240 {
                let next = cycle + dt / STRIDE.seconds;
                let was = [0, 1].map(|leg| leg_phase(cycle + 0.5 * leg as f32, STRIDE.duty_factor).is_stance());
                balance.step(&STRIDE, cycle, next, K, dt);
                cycle = next;
                for (leg, was_down) in was.into_iter().enumerate() {
                    if !was_down && leg_phase(cycle + 0.5 * leg as f32, STRIDE.duty_factor).is_stance() {
                        landings += 1;
                        let nominal = nominal_step(&STRIDE, leg);
                        let step = nominal + balance.feet[leg] - balance.feet[1 - leg];
                        let side = nominal.y.signum();
                        assert!(side * step.y >= MIN_STEP_WIDTH - 1.0e-3, "push {push}: landed crossed, {step}");
                        assert!(
                            step.length() <= STRIDE.max_step + 1.0e-3,
                            "push {push}: a {:.3} m step",
                            step.length()
                        );
                    }
                }
            }
            assert!(landings >= 6, "only {landings} footfalls checked");
        }
    }

    #[test]
    fn an_unpushed_walk_is_left_exactly_alone() {
        let (balance, worst) = walk(Vec2::ZERO, 0.2, 3.0, 1.0 / 60.0);
        assert_eq!(worst, 0.0);
        assert!(balance.is_settled(0.0) && !balance.falls);
    }

    #[test]
    fn the_catch_does_not_depend_on_the_frame_rate() {
        // Ticks of at most 1/60 s: 20 Hz frames land the same footfalls as
        // 60 Hz ones. (Finer frames tick finer, and sample the ankle law
        // more often: 3.6 cm apart sideways after 2 s at 240 Hz.)
        let (fine, _) = walk(Vec2::new(-0.2, 0.2), 0.05, 2.0, 1.0 / 60.0);
        let (coarse, _) = walk(Vec2::new(-0.2, 0.2), 0.05, 2.0, 1.0 / 20.0);
        for leg in 0..2 {
            assert!(
                fine.feet[leg].distance(coarse.feet[leg]) < 0.01,
                "leg {leg}: {} at 60 Hz, {} at 20 Hz",
                fine.feet[leg],
                coarse.feet[leg]
            );
        }
    }
}
#[cfg(test)]
mod probe {
    use super::*;

    /// The largest push caught, sideways and back, by the phase it lands
    /// at (0 the left footfall). Prints a table.
    #[test]
    #[ignore]
    fn probe_walking_catch_limits() {
        let stride = Stride { seconds: 1.1, duty_factor: 0.62, step_length: 0.65, step_width: 0.18, max_step: 0.89 };
        let caught = |push: Vec2, start: f32| {
            let mut b = WalkBalance::default();
            b.push(push);
            let dt = 1.0 / 60.0;
            let mut c = start;
            for _ in 0..360 {
                let n = c + dt / stride.seconds;
                b.step(&stride, c, n, 0.095, dt);
                c = n;
            }
            !b.falls
        };
        let limit = |dir: Vec2, start: f32| {
            let mut v = 0.0;
            while v < 3.0 && caught(dir * (v + 0.05), start) {
                v += 0.05;
            }
            v
        };
        for i in 0..20 {
            let s = i as f32 * 0.05;
            println!(
                "cycle {s:.2}: left {:.2} right {:.2} back {:.2}",
                limit(Vec2::Y, s),
                limit(Vec2::NEG_Y, s),
                limit(Vec2::NEG_X, s)
            );
        }
    }
}
