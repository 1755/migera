//! Foot locking — pinning a planted foot to the ground so it does not
//! slide.
//!
//! # Foot sliding is a velocity problem, not a position one
//!
//! The instinct is to treat sliding as a physics violation: the foot is
//! touching the ground, friction should hold it, so clamp its position.
//! That framing leads to the wrong fixes.
//!
//! What actually goes wrong is that the foot's velocity *at runtime* stops
//! matching its velocity *in the source animation* — because the root is
//! moving at a speed the animation was not authored for, or because
//! blending joint rotations moved the end of a chain in a way nobody
//! intended. Sliding is that velocity error, made visible by the ground.
//!
//! Two consequences follow, and both are load-bearing here:
//!
//! - **Contacts are detected from speed**, not height. People do not lift
//!   their feet far when walking, so foot height barely separates stance
//!   from swing; toe *speed* separates them cleanly. Height is only a
//!   sanity check, to reject a foot held still in mid-air.
//! - **The flight phase matters too.** Locking only fixes the error where
//!   it is most visible. It does not make the rest correct.
//!
//! # Lock the toe, not the heel
//!
//! In real locomotion the toe is in contact roughly 90% of the time, and
//! pivoting about the toe (heel lifting, ball planted) is extremely
//! common; the reverse — heel planted, toe free — is rare and unstable.
//! A heel lock fights every one of those pivots. So this module locks the
//! toe and lets the heel move freely.
//!
//! # Hysteresis, and why one threshold is not enough
//!
//! A single speed threshold makes a foot hovering near it lock and unlock
//! every frame, which looks far worse than never locking at all. Two
//! thresholds — lock below a low speed, release only above a higher one —
//! give the state somewhere stable to sit. The same applies to the
//! distance check: a lock is abandoned only once the animation has pulled
//! meaningfully away from it.
//!
//! # Releasing with a deadline
//!
//! When a lock releases, the foot is somewhere the animation is not, and
//! that gap has to close. It uses the *cubic* inertializer from
//! [`super::math::inertialize`] rather than the exponential one, because
//! the gap must be provably gone before the next contact — an exponential
//! decay is merely small, and residual offset at the next footfall
//! reintroduces exactly the sliding the lock removed.

use bevy::math::Vec3;

use super::math::inertialize::InertializeCubic;

/// Thresholds governing when a foot locks and releases.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FootLockConfig {
    /// Lock once the toe's speed drops below this, m/s.
    pub lock_speed: f32,
    /// Release once it rises above this, m/s. Must exceed
    /// [`lock_speed`](Self::lock_speed) — the gap is the hysteresis that
    /// stops a foot flickering between states.
    pub unlock_speed: f32,
    /// Reject a lock if the toe is higher than this above the ground, m.
    /// Guards against pinning a foot that is merely being held still in
    /// mid-air.
    pub max_contact_height: f32,
    /// Release if the animation pulls this far from the locked point, m.
    /// Without it a lock can drag a foot arbitrarily far behind a
    /// character that has started moving.
    pub break_distance: f32,
    /// How long the release blend takes, seconds. The cubic inertializer
    /// reaches zero exactly at this deadline.
    pub release_seconds: f32,
}

impl Default for FootLockConfig {
    fn default() -> Self {
        Self {
            // 0.1-0.5 m/s is the band that separates stance from swing in
            // real locomotion data; the low end is chosen so only a
            // genuinely planted foot locks.
            lock_speed: 0.15,
            unlock_speed: 0.45,
            max_contact_height: 0.08,
            break_distance: 0.25,
            release_seconds: 0.15,
        }
    }
}

/// How far the body turned and travelled this frame, and about where.
///
/// Passed to [`FootLock::update_turning`] so a planted foot can pivot with
/// the body instead of being dragged sideways by it, and stay put while the
/// body travels over it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Turn {
    /// The body's own centre, world space — the point the turn is about.
    pub pivot: Vec3,
    /// How far it turned this frame, radians about `+Y`.
    pub yaw_delta: f32,
    /// How far the body travelled this frame, rise and fall included, in
    /// the frame the lock's points are given in (the pose's). On
    /// `plugin::AnimFootIk::turn` it is in WORLD axes, and the IK stage
    /// rotates it into the pose's before handing it here.
    ///
    /// The points a lock sees are the ANIMATION's, relative to the body.
    /// When root motion moves the body, a foot still on the ground moves
    /// backward in that frame by exactly the travel. Without it the anchor
    /// rode along with the body: a foot locked while standing was carried
    /// with the first step, and slid ~14 cm before the lock let go. A
    /// planted foot's speed also read as the walking speed, so the lock
    /// never engaged during a walk.
    pub travel: Vec3,
}

impl Turn {
    /// A character that did not turn.
    pub const NONE: Self = Self { pivot: Vec3::ZERO, yaw_delta: 0.0, travel: Vec3::ZERO };

    /// Where `point` ends up after this turn.
    fn applied_to(self, point: Vec3) -> Vec3 {
        if self.yaw_delta == 0.0 || !self.yaw_delta.is_finite() {
            return point;
        }

        let offset = point - self.pivot;
        let rotated = bevy::math::Quat::from_rotation_y(self.yaw_delta) * offset;

        // Height is taken from the original rather than the rotation: a yaw
        // cannot change it, and reusing the rotated value would accumulate
        // float error into the foot's ground contact over a long turn.
        Vec3::new(self.pivot.x + rotated.x, point.y, self.pivot.z + rotated.z)
    }
}

/// Whether a foot is currently pinned.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LockState {
    /// Following the animation.
    #[default]
    Free,
    /// Pinned to a world point.
    Locked,
}

/// One foot's locking state.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FootLock {
    state: LockState,
    /// Where the toe was pinned, world space.
    anchor: Vec3,
    /// The gap being closed after a release.
    release: InertializeCubic,
    /// Last frame's animated toe position, for the finite-difference
    /// speed estimate.
    previous_animated: Option<Vec3>,
}

impl FootLock {
    /// A foot that starts free.
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the foot is pinned right now.
    pub fn is_locked(&self) -> bool {
        self.state == LockState::Locked
    }

    /// The world point the foot is pinned to, if it is.
    pub fn anchor(&self) -> Option<Vec3> {
        match self.state {
            LockState::Locked => Some(self.anchor),
            LockState::Free => None,
        }
    }

    /// Moves a pinned foot by `by` (the anchor's frame), as its owner keeps
    /// it out of something it must not stand in: a turn pivots a planted
    /// foot about the body, and in front of a chair that carried one into
    /// the chair's leg (`walker`, `approach::Chair::foot_clear`). A free
    /// foot is left alone.
    pub fn shift_anchor(&mut self, by: Vec3) {
        if self.state == LockState::Locked {
            self.anchor += by;
        }
    }

    /// Advances one frame and returns where the toe should actually go.
    ///
    /// `animated` is where the animation alone would put it; `ground_height`
    /// is the ground under it. The returned position is what the IK stage
    /// should solve toward.
    pub fn update(
        &mut self,
        animated: Vec3,
        ground_height: f32,
        config: &FootLockConfig,
        dt: f32,
    ) -> Vec3 {
        self.update_turning(animated, ground_height, config, dt, Turn::NONE)
    }

    /// [`Self::update`], for a character that is also turning.
    ///
    /// # Why a turn needs its own parameter
    ///
    /// A lock pins a foot to a WORLD point, which is right while the body
    /// only translates. Under rotation it is wrong: the body pivots and the
    /// anchor does not, so the leg is dragged sideways and scissors.
    ///
    /// Nor does the lock break and rescue itself. A 90 degree pivot sweeps a
    /// stance foot about `0.11 * sqrt(2) = 0.156 m` for this rig's hip
    /// width, against a [`FootLockConfig::break_distance`] of 0.25 m — so it
    /// stays locked for the whole turn and drags the whole way.
    ///
    /// Rotating the anchor about the body's own centre is what a real
    /// planted foot does: it pivots in place. The turn has to be passed in
    /// because the lock cannot see the body, and deriving it from the
    /// animated position would confuse "the body turned" with "the animation
    /// moved this foot".
    pub fn update_turning(
        &mut self,
        animated: Vec3,
        ground_height: f32,
        config: &FootLockConfig,
        dt: f32,
        turn: Turn,
    ) -> Vec3 {
        self.update_planted(animated, ground_height, config, dt, turn, false)
    }

    /// [`Self::update_turning`] for a foot its caller knows is down
    /// (`planted`): a lock on it is not released by the animated toe's
    /// speed, only by being dragged past
    /// [`FootLockConfig::break_distance`].
    ///
    /// Speed is this module's guess at contact, and a sprung leg under a
    /// fast body fools it: in a 0.7 m/s sideways stumble the planted foot's
    /// animated toe lagged the unsprung pelvis at 0.5-0.8 m/s, the lock let
    /// go four times, and the foot slid 16-21 mm.
    pub fn update_planted(
        &mut self,
        animated: Vec3,
        ground_height: f32,
        config: &FootLockConfig,
        dt: f32,
        turn: Turn,
        planted: bool,
    ) -> Vec3 {
        // The anchor pivots with the body BEFORE anything reads it, so the
        // distance and speed checks below compare against where the foot
        // now is rather than where it was a turn ago.
        //
        // And it stays behind as the body travels: the travel is removed
        // from it, which is what keeps it where it is in the world. Up and
        // down too: walking up a 0.2 grade, the body rose under a foot locked
        // horizontally only, and carried it 9 cm up off the slope through
        // every stance.
        let travel = if turn.travel.is_finite() { turn.travel } else { Vec3::ZERO };
        if self.state == LockState::Locked {
            self.anchor = turn.applied_to(self.anchor) - travel;
        }

        if dt <= 0.0 {
            return self.hold(animated, config);
        }

        // Speed from a finite difference of the animated position, in the
        // WORLD: the body's travel added back, so a foot standing still on
        // the ground under a moving body reads as still. The first frame has
        // no previous sample, so it reports infinity — which deliberately
        // means a foot cannot lock on the very first frame, before there is
        // evidence it is stationary.
        let speed = match self.previous_animated {
            Some(previous) => (animated - previous + travel).length() / dt,
            None => f32::INFINITY,
        };
        self.previous_animated = Some(animated);

        self.release.advance(dt);

        match self.state {
            LockState::Free => {
                let grounded = animated.y - ground_height <= config.max_contact_height;

                if speed < config.lock_speed && grounded {
                    self.state = LockState::Locked;
                    // Pin at ground level, not at wherever the animation
                    // happened to be: a contact by definition touches the
                    // ground, and a few millimetres of float above it would
                    // read as hovering.
                    self.anchor = Vec3::new(animated.x, ground_height, animated.z);
                    return self.anchor;
                }

                self.hold(animated, config)
            }
            LockState::Locked => {
                let dragged = (animated - self.anchor).length() > config.break_distance;

                if speed > config.unlock_speed && !planted || dragged {
                    self.state = LockState::Free;
                    // Start closing the gap from where the foot actually
                    // is, so releasing is continuous rather than a snap.
                    self.release =
                        InertializeCubic::begin(self.anchor, Vec3::ZERO, animated, Vec3::ZERO);
                    return self.hold(animated, config);
                }

                self.anchor
            }
        }
    }

    /// Where a free foot sits: the animation, plus whatever release gap is
    /// still closing.
    fn hold(&self, animated: Vec3, config: &FootLockConfig) -> Vec3 {
        animated + self.release.evaluate(config.release_seconds).0
    }

    /// Whether a release blend is still running.
    pub fn is_releasing(&self, config: &FootLockConfig) -> bool {
        !self.release.is_finished(config.release_seconds)
    }
}

/// Labels contacts across a recorded sequence of toe positions, offline.
///
/// The runtime detector in [`FootLock::update`] is causal — it can only
/// look backwards — which costs it accuracy at the edges of a contact.
/// Given a whole clip, a non-causal filter does better, and the result can
/// be stored alongside the animation.
///
/// Returns one flag per sample.
pub fn annotate_contacts(
    positions: &[Vec3],
    ground_height: f32,
    config: &FootLockConfig,
    dt: f32,
) -> Vec<bool> {
    if positions.len() < 2 || dt <= 0.0 {
        return vec![false; positions.len()];
    }

    // Central differences, so a sample's speed reflects motion on both
    // sides of it rather than lagging half a frame behind.
    let speeds: Vec<f32> = (0..positions.len())
        .map(|i| {
            let previous = positions[i.saturating_sub(1)];
            let next = positions[(i + 1).min(positions.len() - 1)];
            let span = if i == 0 || i == positions.len() - 1 { dt } else { 2.0 * dt };
            (next - previous).length() / span
        })
        .collect();

    let raw: Vec<bool> = positions
        .iter()
        .zip(&speeds)
        .map(|(position, speed)| {
            *speed < config.lock_speed
                && position.y - ground_height <= config.max_contact_height
        })
        .collect();

    median_filter(&raw, 5)
}

/// Removes single-frame flickers from a boolean signal by majority vote
/// over a sliding window.
///
/// A lone spurious frame — a contact that appears for one sample and
/// vanishes — produces a visible pop when it drives a lock. For a binary
/// signal the median and the majority are the same thing.
///
/// `window` is rounded up to the next odd number so there is always a
/// strict majority.
pub fn median_filter(signal: &[bool], window: usize) -> Vec<bool> {
    let window = window.max(1) | 1;
    let half = window / 2;

    (0..signal.len())
        .map(|i| {
            let start = i.saturating_sub(half);
            let end = (i + half + 1).min(signal.len());

            let set = signal[start..end].iter().filter(|&&on| on).count();
            set * 2 > end - start
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// Feeds a foot a stationary toe until it locks.
    fn locked_foot(config: &FootLockConfig) -> (FootLock, Vec3) {
        let mut lock = FootLock::new();
        let planted = Vec3::new(0.2, 0.0, -0.3);

        for _ in 0..5 {
            lock.update(planted, 0.0, config, DT);
        }

        assert!(lock.is_locked(), "test setup: the foot should have locked");
        (lock, planted)
    }

    #[test]
    fn a_stationary_grounded_foot_locks() {
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        assert!(!lock.is_locked(), "a foot starts free");

        lock.update(Vec3::ZERO, 0.0, &config, DT);
        lock.update(Vec3::ZERO, 0.0, &config, DT);

        assert!(lock.is_locked(), "a stationary grounded foot should lock");
    }

    #[test]
    fn a_foot_cannot_lock_on_its_very_first_frame() {
        // There is no previous sample to measure speed against, so locking
        // would be a guess. Better to wait one frame than to pin a foot
        // that turns out to be moving.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        lock.update(Vec3::ZERO, 0.0, &config, DT);

        assert!(!lock.is_locked(), "the first frame has no speed evidence yet");
    }

    #[test]
    fn a_fast_moving_foot_never_locks() {
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let mut position = Vec3::ZERO;
        for _ in 0..30 {
            position.z -= 1.5 * DT; // 1.5 m/s, well above the threshold
            lock.update(position, 0.0, &config, DT);
            assert!(!lock.is_locked(), "a swinging foot must stay free");
        }
    }

    #[test]
    fn a_foot_held_still_in_mid_air_does_not_lock() {
        // The height sanity check. Speed alone would happily pin a foot
        // paused at the top of its swing.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let airborne = Vec3::new(0.0, 0.5, 0.0);
        for _ in 0..10 {
            lock.update(airborne, 0.0, &config, DT);
        }

        assert!(!lock.is_locked(), "a foot well above the ground must not lock");
    }

    #[test]
    fn a_locked_foot_has_zero_world_velocity() {
        // THE property the whole module exists for. While locked, the toe
        // must not move at all, however much the animation does.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let mut animated = planted;
        let mut previous = lock.update(animated, 0.0, &config, DT);

        for _ in 0..20 {
            // The animation drifts, slowly enough not to break the lock.
            animated.z -= 0.002;
            let current = lock.update(animated, 0.0, &config, DT);

            assert!(
                (current - previous).length() < 1.0e-6,
                "a locked toe must not move, but travelled {} m in one frame",
                (current - previous).length(),
            );
            previous = current;
        }
    }

    #[test]
    fn a_lock_pins_the_toe_to_the_ground_not_to_wherever_it_floated() {
        // A contact touches the ground by definition; pinning a few
        // millimetres above it reads as hovering.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let hovering = Vec3::new(0.1, 0.03, 0.0);
        for _ in 0..5 {
            lock.update(hovering, 0.0, &config, DT);
        }

        let anchor = lock.anchor().expect("should be locked");
        assert_eq!(anchor.y, 0.0, "the anchor should sit on the ground");
        assert_eq!(anchor.x, hovering.x, "...without moving horizontally");
    }

    #[test]
    fn a_lock_follows_a_raised_ground_height() {
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let on_a_step = Vec3::new(0.0, 0.3, 0.0);
        for _ in 0..5 {
            lock.update(on_a_step, 0.3, &config, DT);
        }

        assert_eq!(
            lock.anchor().expect("should lock on the step").y,
            0.3,
            "the anchor should sit on the ground it was given",
        );
    }

    #[test]
    fn a_lock_releases_when_the_foot_swings_away() {
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let mut animated = planted;
        for _ in 0..10 {
            animated.z -= 1.0 * DT; // 1 m/s, above unlock_speed
            lock.update(animated, 0.0, &config, DT);
        }

        assert!(!lock.is_locked(), "a foot swinging away should release");
    }

    #[test]
    fn a_planted_foot_holds_its_lock_against_speed_but_not_a_drag() {
        // A stumbling body's sprung leg lags its pelvis, and the animated
        // toe of a foot the balance has down moves at 0.5-0.8 m/s: a
        // caller-planted lock ignores that, and still lets go when dragged
        // past `break_distance`.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);
        let mut animated = planted;
        // Fast, but short of the break distance: well above unlock_speed.
        for _ in 0..6 {
            animated.x += 1.0 * DT;
            let held = lock.update_planted(animated, 0.0, &config, DT, Turn::NONE, true);
            assert!(lock.is_locked() && (held - Vec3::new(planted.x, 0.0, planted.z)).length() < 1.0e-6, "a planted foot let go");
        }
        let (mut unplanted, _) = locked_foot(&config);
        let mut animated = planted;
        for _ in 0..6 {
            animated.x += 1.0 * DT;
            unplanted.update_planted(animated, 0.0, &config, DT, Turn::NONE, false);
        }
        assert!(!unplanted.is_locked(), "the same motion, unplanted, should release");
        for _ in 0..60 {
            animated.x += 1.0 * DT;
            lock.update_planted(animated, 0.0, &config, DT, Turn::NONE, true);
        }
        assert!(!lock.is_locked(), "dragged {:.2} m away, even a planted foot should release", animated.x - planted.x);
    }

    #[test]
    fn hysteresis_stops_a_foot_flickering_between_states() {
        // THE reason for two thresholds. A foot drifting at a speed between
        // them must pick one state and stay there — a foot toggling every
        // frame looks far worse than one that never locks.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        // Right in the hysteresis band: above lock_speed, below
        // unlock_speed.
        let between = (config.lock_speed + config.unlock_speed) * 0.5;

        let mut animated = planted;
        let mut transitions = 0;
        let mut was_locked = lock.is_locked();

        for _ in 0..60 {
            animated.z -= between * DT;
            lock.update(animated, 0.0, &config, DT);

            if lock.is_locked() != was_locked {
                transitions += 1;
                was_locked = lock.is_locked();
            }
        }

        assert!(
            transitions <= 1,
            "a foot in the hysteresis band should settle, but changed state \
             {transitions} times",
        );
    }

    #[test]
    fn a_lock_never_strands_a_foot_further_than_the_break_distance() {
        // Without a distance check, a lock can strand a foot behind a
        // character that has started moving, stretching the leg past its
        // reach.
        //
        // Note the foot does NOT simply end up free: creeping this slowly
        // is below the unlock speed, so once the lock breaks the foot
        // immediately re-plants at its new position. That break-and-replant
        // is the right behaviour for a foot shuffling along, and it is why
        // this asserts on the DISTANCE rather than on the end state — an
        // earlier version checked `!is_locked()` and failed against
        // perfectly correct behaviour.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let mut animated = planted;
        let creep = config.lock_speed * 0.5 * DT;
        let mut furthest = 0.0f32;

        for _ in 0..600 {
            animated.z -= creep;
            let toe = lock.update(animated, 0.0, &config, DT);
            furthest = furthest.max((toe - animated).length());
        }

        assert!(
            furthest <= config.break_distance + 0.01,
            "the toe should never lag more than the {} m break distance behind the \
             animation, but reached {furthest} m",
            config.break_distance,
        );
        assert!(
            (animated - planted).length() > config.break_distance,
            "test setup: the animation should have travelled well past the break \
             distance",
        );
    }

    #[test]
    fn a_shuffling_foot_replants_rather_than_sliding() {
        // The other half of the above: after a break the foot should take
        // up a NEW anchor near where the animation now is, not drift free.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let mut animated = planted;
        let creep = config.lock_speed * 0.5 * DT;
        for _ in 0..600 {
            animated.z -= creep;
            lock.update(animated, 0.0, &config, DT);
        }

        let anchor = lock.anchor().expect("a slowly shuffling foot should stay planted");
        assert!(
            (anchor - animated).length() < config.break_distance,
            "the new anchor should sit near where the animation is now, but is {} m \
             away",
            (anchor - animated).length(),
        );
        assert!(
            (anchor - planted).length() > 0.1,
            "...and should have moved on from the original plant",
        );
    }

    #[test]
    fn releasing_a_lock_is_continuous_rather_than_a_snap() {
        // The foot is somewhere the animation is not when a lock breaks;
        // jumping straight there is a visible pop.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let before = lock.update(planted, 0.0, &config, DT);

        // Break the lock with a fast, distant target.
        let far = planted + Vec3::new(0.0, 0.0, -0.4);
        let after = lock.update(far, 0.0, &config, DT);

        assert!(
            (after - before).length() < 0.02,
            "releasing should ease out of the anchor, but the toe jumped {} m",
            (after - before).length(),
        );
    }

    #[test]
    fn a_release_gap_is_provably_closed_by_its_deadline() {
        // Why the CUBIC inertializer is used rather than the exponential
        // one: leftover offset at the next contact reintroduces exactly the
        // sliding the lock removed. "Small" is not good enough; it has to
        // be gone.
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let far = planted + Vec3::new(0.0, 0.0, -0.4);
        lock.update(far, 0.0, &config, DT);

        let steps = (config.release_seconds / DT).ceil() as usize + 2;
        for _ in 0..steps {
            lock.update(far, 0.0, &config, DT);
        }

        assert!(
            !lock.is_releasing(&config),
            "the release blend should have finished by its deadline",
        );
        assert_eq!(
            lock.update(far, 0.0, &config, DT),
            far,
            "and the toe should then follow the animation exactly",
        );
    }

    #[test]
    fn a_free_foot_follows_the_animation_exactly() {
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let mut position = Vec3::new(0.0, 0.4, 0.0);
        for _ in 0..20 {
            position.z -= 2.0 * DT;
            let output = lock.update(position, 0.0, &config, DT);
            assert_eq!(output, position, "a free foot should pass through untouched");
        }
    }

    #[test]
    fn a_non_positive_timestep_holds_the_current_position() {
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let held = lock.update(planted, 0.0, &config, 0.0);
        assert!(held.is_finite(), "got {held:?}");
        assert!(lock.is_locked(), "a zero timestep must not change state");
    }

    #[test]
    fn the_default_thresholds_leave_a_real_hysteresis_gap() {
        let config = FootLockConfig::default();
        assert!(
            config.unlock_speed > config.lock_speed,
            "without a gap there is no hysteresis at all",
        );
    }

    // -----------------------------------------------------------------
    // Turning
    // -----------------------------------------------------------------

    #[test]
    fn a_planted_foot_pivots_with_the_body_instead_of_being_dragged() {
        // THE property turning needs. A lock pins a foot to a WORLD point,
        // which is right while the body only translates and wrong once it
        // rotates: the body pivots, the anchor does not, and the leg
        // scissors.
        //
        // The lock does not rescue itself either — a 90 degree pivot sweeps
        // a stance foot about 0.156 m for this rig's hip width, inside the
        // 0.25 m break distance, so it stays locked and drags the whole way.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        // A foot planted half a hip-width to the side and a little ahead.
        let planted = Vec3::new(-0.11, 0.0, -0.1);
        for _ in 0..5 {
            lock.update(planted, 0.0, &config, DT);
        }
        assert!(lock.is_locked(), "test setup: the foot should be planted");

        // A quarter turn about the body's centre, in small steps.
        let pivot = Vec3::ZERO;
        let total = std::f32::consts::FRAC_PI_2;
        let steps = 30;

        for _ in 0..steps {
            let turn = Turn { pivot, yaw_delta: total / steps as f32, ..Turn::NONE };
            lock.update_turning(planted, 0.0, &config, DT, turn);
        }

        let anchor = lock.anchor().expect("the foot should still be planted");

        // It pivoted: the anchor moved to where a quarter turn puts it.
        let expected = Vec3::new(-0.1, 0.0, 0.11);
        assert!(
            anchor.distance(expected) < 0.02,
            "after a quarter turn the anchor should be near {expected:?}, got \
             {anchor:?}",
        );

        // And it stayed the same distance from the body's centre — a pivot
        // rotates a foot, it does not move it in or out.
        assert!(
            (anchor.distance(pivot) - planted.distance(pivot)).abs() < 1.0e-3,
            "the foot's distance from the body changed during the turn: {} -> {}",
            planted.distance(pivot),
            anchor.distance(pivot),
        );
    }

    #[test]
    fn a_turn_does_not_lift_or_sink_a_planted_foot() {
        // A yaw cannot change height, and reusing the rotated `y` would
        // accumulate float error into the foot's ground contact over a long
        // turn.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let planted = Vec3::new(-0.11, 0.0, 0.0);
        for _ in 0..5 {
            lock.update(planted, 0.0, &config, DT);
        }

        for _ in 0..600 {
            let turn = Turn { yaw_delta: 0.05, ..Turn::NONE };
            lock.update_turning(planted, 0.0, &config, DT, turn);
        }

        let anchor = lock.anchor().expect("still planted");
        assert!(
            anchor.y.abs() < 1.0e-5,
            "ten full turns moved the foot to y = {}",
            anchor.y,
        );
    }

    #[test]
    fn a_free_foot_is_unaffected_by_turning() {
        // Only a PLANTED foot pivots; a swinging one is following the
        // animation, which is already expressed in the body's frame.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();

        let mut swinging = Vec3::new(-0.11, 0.3, 0.0);
        for _ in 0..10 {
            swinging.z -= 2.0 * DT;
            let turn = Turn { yaw_delta: 0.1, ..Turn::NONE };
            let output = lock.update_turning(swinging, 0.0, &config, DT, turn);

            assert_eq!(
                output, swinging,
                "a free foot should pass through untouched even while turning",
            );
        }
    }

    #[test]
    fn a_zero_turn_is_exactly_the_non_turning_path() {
        // The inertness that lets every existing caller keep working.
        let config = FootLockConfig::default();

        let mut plain = FootLock::new();
        let mut turning = FootLock::new();

        let mut animated = Vec3::new(0.1, 0.0, 0.0);
        for _ in 0..40 {
            animated.z -= 0.001;

            let a = plain.update(animated, 0.0, &config, DT);
            let b =
                turning.update_turning(animated, 0.0, &config, DT, Turn::NONE);

            assert_eq!(a, b, "a zero turn must not change anything");
        }
    }

    #[test]
    fn a_planted_foot_stays_put_in_the_world_while_the_body_travels_over_it() {
        // The first step after standing: the foot was locked while the body
        // stood, and the body starts to move over it. The lock sees the
        // foot in the BODY's frame, where it slides backward by exactly the
        // body's travel. Without the travel the anchor rode along with the
        // body (measured live: ~14 cm of slide in the first step).
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();
        let world_foot = Vec3::new(-0.11, 0.0, -0.1);
        for _ in 0..5 {
            lock.update(world_foot, 0.0, &config, DT);
        }
        assert!(lock.is_locked(), "test setup: the foot should be planted");

        // Accelerating from rest to a walk, well past the unlock speed.
        let (mut body, mut speed) = (Vec3::ZERO, 0.0_f32);
        for frame in 0..40 {
            speed = (speed + 4.0 * DT).min(1.4);
            let travel = Vec3::new(0.0, 0.0, -speed * DT);
            body += travel;
            let animated = world_foot - body;
            let target = lock.update_turning(animated, 0.0, &config, DT, Turn { travel, ..Turn::NONE });
            let slid = (body + target - world_foot).length();
            assert!(lock.is_locked(), "frame {frame}: a planted foot let go at {speed} m/s");
            assert!(slid < 1.0e-4, "frame {frame}: the planted foot slid {slid} m in the world");
        }
    }

    #[test]
    fn a_planted_foot_stays_put_while_the_body_climbs_over_it() {
        // Up a 0.2 grade the body rises as it travels. A lock told only the
        // horizontal part carried the foot up with the body: 9 cm off the
        // slope by the end of a live stance.
        let config = FootLockConfig::default();
        let mut lock = FootLock::new();
        let world_foot = Vec3::new(-0.11, 0.0, -0.1);
        for _ in 0..5 {
            lock.update(world_foot, 0.0, &config, DT);
        }
        let mut body = Vec3::ZERO;
        for frame in 0..30 {
            let travel = Vec3::new(0.0, 0.2 * 1.2 * DT, -1.2 * DT);
            body += travel;
            let animated = world_foot - body;
            // The ground under the foot, in the body's frame, falls as the
            // body rises.
            let target = lock.update_turning(animated, -body.y, &config, DT, Turn { travel, ..Turn::NONE });
            let moved = (body + target - world_foot).length();
            assert!(moved < 1.0e-4, "frame {frame}: the planted foot moved {moved} m in the world");
        }
    }

    #[test]
    fn a_non_finite_turn_is_ignored() {
        let config = FootLockConfig::default();
        let (mut lock, planted) = locked_foot(&config);

        let turn = Turn { yaw_delta: f32::NAN, ..Turn::NONE };
        lock.update_turning(planted, 0.0, &config, DT, turn);

        let anchor = lock.anchor().expect("still planted");
        assert!(anchor.is_finite(), "a NaN turn corrupted the anchor: {anchor:?}");
    }

    // -----------------------------------------------------------------
    // Offline contact annotation
    // -----------------------------------------------------------------

    #[test]
    fn annotation_finds_a_contact_where_the_toe_is_stationary() {
        let config = FootLockConfig::default();

        // Twenty frames swinging, twenty planted, twenty swinging again.
        let mut positions = Vec::new();
        let mut position = Vec3::new(0.0, 0.2, 0.5);
        for _ in 0..20 {
            position.z -= 2.0 * DT;
            positions.push(position);
        }
        let planted = Vec3::new(0.0, 0.0, position.z);
        for _ in 0..20 {
            positions.push(planted);
        }
        for _ in 0..20 {
            position = Vec3::new(0.0, 0.2, position.z - 2.0 * DT);
            positions.push(position);
        }

        let contacts = annotate_contacts(&positions, 0.0, &config, DT);

        assert!(
            contacts[25..35].iter().all(|&on| on),
            "the planted window should be labelled as contact",
        );
        assert!(
            !contacts[5] && !contacts[55],
            "the swinging windows should not be",
        );
    }

    #[test]
    fn annotation_ignores_a_single_frame_flicker() {
        // The median filter's whole purpose: one spurious frame in the
        // middle of a swing would otherwise drive a one-frame lock, which
        // pops.
        let config = FootLockConfig::default();

        let mut positions = Vec::new();
        let mut position = Vec3::new(0.0, 0.2, 0.0);
        for i in 0..40 {
            if i != 20 {
                position.z -= 2.0 * DT;
            }
            positions.push(position);
        }

        let contacts = annotate_contacts(&positions, 0.0, &config, DT);

        assert!(
            contacts.iter().all(|&on| !on),
            "a one-frame pause mid-swing must not be labelled a contact",
        );
    }

    #[test]
    fn annotation_of_a_short_or_empty_clip_is_safe() {
        let config = FootLockConfig::default();

        assert!(annotate_contacts(&[], 0.0, &config, DT).is_empty());
        assert_eq!(annotate_contacts(&[Vec3::ZERO], 0.0, &config, DT), vec![false]);
        assert_eq!(
            annotate_contacts(&[Vec3::ZERO, Vec3::ZERO], 0.0, &config, 0.0),
            vec![false, false],
        );
    }

    #[test]
    fn the_median_filter_removes_lone_spikes_but_keeps_real_runs() {
        let signal = [false, false, true, false, false, true, true, true, true, false];
        let filtered = median_filter(&signal, 5);

        assert!(!filtered[2], "a lone true should be removed");
        assert!(
            filtered[6] && filtered[7],
            "a sustained run should survive: {filtered:?}",
        );
    }

    #[test]
    fn the_median_filter_forces_an_odd_window() {
        // An even window has no strict majority; rounding up keeps the
        // decision unambiguous.
        let signal = [true, false, true, false];
        assert_eq!(median_filter(&signal, 4).len(), signal.len());
        assert_eq!(median_filter(&signal, 0).len(), signal.len());
    }
}
