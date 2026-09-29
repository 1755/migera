//! Which way a character is pointing, and how fast it may change.
//!
//! # Why a scalar yaw rather than a quaternion
//!
//! A character standing on ground turns about one axis. Storing that as a
//! `Quat` makes two easy things hard: "turn the shortest way round" becomes
//! a quaternion-neighbourhood problem (the `q` and `-q` hazard this project
//! has already paid for), and "how far is there left to turn" stops being a
//! subtraction.
//!
//! As a scalar it is a wrap, which is one function and one test. The
//! conversion to a rotation happens at the boundary, where it is used.
//!
//! # What this owns, and what it does not
//!
//! It owns the character's *heading*: where it points now, where it is
//! trying to point, and the rate between them. It does not own position —
//! see [`super::locomotion`]'s own note on why the animation publishes a
//! velocity and something else integrates it. Turning follows the same rule:
//! a facing is state the animation maintains, and a caller decides what to
//! do with it.

use bevy::math::{Quat, Vec3};

/// A character's heading, and how fast it may change.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Facing {
    /// Where the character points now, radians about `+Y`.
    ///
    /// Zero is this rig's own forward, `-Z`.
    pub yaw: f32,
    /// Where it is trying to point.
    pub target_yaw: f32,
    /// How fast it may turn, radians per second.
    ///
    /// A real walking turn is roughly 90-180 degrees per second; a sprint
    /// turn is slower, because momentum has to go somewhere.
    pub turn_rate: f32,
}

impl Default for Facing {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            target_yaw: 0.0,
            // ~115 degrees per second: brisk enough to feel responsive,
            // slow enough that the turn reads as a turn rather than a snap.
            turn_rate: 2.0,
        }
    }
}

impl Facing {
    /// A character already pointing where it wants to.
    pub fn at(yaw: f32) -> Self {
        Self { yaw, target_yaw: yaw, ..Default::default() }
    }

    /// The rotation this heading represents.
    pub fn rotation(&self) -> Quat {
        Quat::from_rotation_y(self.yaw)
    }

    /// The direction this heading points, in world space.
    pub fn forward(&self) -> Vec3 {
        self.rotation() * Vec3::NEG_Z
    }

    /// How far there is left to turn, signed, taking the short way round.
    ///
    /// Always in `[-pi, pi]`, so a character at `+170` degrees asked to face
    /// `-170` turns 20 degrees rather than 340.
    pub fn remaining(&self) -> f32 {
        shortest_angle(self.target_yaw - self.yaw)
    }

    /// Whether the heading has arrived, within a tolerance.
    pub fn is_settled(&self, tolerance: f32) -> bool {
        self.remaining().abs() <= tolerance
    }

    /// Points at a world-space direction. Vertical component ignored.
    ///
    /// A zero or purely vertical direction leaves the target unchanged —
    /// there is no heading in it to adopt, and inventing one would snap the
    /// character somewhere arbitrary.
    pub fn face_direction(&mut self, direction: Vec3) {
        let flat = Vec3::new(direction.x, 0.0, direction.z);
        if flat.length_squared() < 1.0e-8 {
            return;
        }

        self.target_yaw = yaw_of(flat);
    }

    /// Points at a world-space position, from `from`.
    pub fn face_position(&mut self, from: Vec3, target: Vec3) {
        self.face_direction(target - from);
    }

    /// Turns toward the target by at most `turn_rate * dt`.
    ///
    /// Never overshoots: a character one degree away with a large rate
    /// arrives exactly, rather than oscillating around its own target. That
    /// is the difference between a turn-rate limit and a spring, and it is
    /// deliberate — a heading that rings is a character that cannot stand
    /// still while aiming.
    pub fn advance(&mut self, dt: f32) {
        if dt <= 0.0 || !dt.is_finite() {
            return;
        }

        let remaining = self.remaining();
        let step = self.turn_rate.max(0.0) * dt;

        self.yaw = if remaining.abs() <= step {
            self.target_yaw
        } else {
            self.yaw + step * remaining.signum()
        };

        // Kept in `[-pi, pi)` so it cannot drift to a magnitude where f32
        // resolution near the value starts to matter. A character spinning
        // one way for an hour is not exotic.
        self.yaw = shortest_angle(self.yaw);
        self.target_yaw = shortest_angle(self.target_yaw);
    }
}

/// Wraps an angle into `[-pi, pi)`.
///
/// The one place "shortest way round" is decided. Everything else —
/// [`Facing::remaining`], the overshoot guard, the drift guard — is this
/// function applied somewhere.
pub fn shortest_angle(radians: f32) -> f32 {
    use std::f32::consts::{PI, TAU};

    if !radians.is_finite() {
        return 0.0;
    }

    let wrapped = (radians + PI).rem_euclid(TAU) - PI;

    // `rem_euclid` can return exactly `TAU` for a small negative input, so
    // the subtraction lands on `+PI` rather than `-PI`. Both name the same
    // direction; normalising keeps the range half-open as documented.
    if wrapped >= PI { wrapped - TAU } else { wrapped }
}

/// The yaw that points along `direction`.
///
/// The exact inverse of [`Facing::forward`], derived rather than guessed:
///
/// ```text
/// Ry(yaw) * (0, 0, -1) = (-sin yaw, 0, -cos yaw)
/// ```
///
/// so recovering the angle is `atan2(-x, -z)`.
///
/// **A positive yaw points toward `-X`**, which is the character's own
/// LEFT — not the `+X` that "turn positively about up" suggests. That
/// follows from zero being `-Z` and from Bevy's right-handed convention;
/// writing `atan2(x, -z)` on the intuition instead made every yaw come back
/// negated, caught by `yaw_and_direction_round_trip`.
pub fn yaw_of(direction: Vec3) -> f32 {
    (-direction.x).atan2(-direction.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::{FRAC_PI_2, PI};

    #[test]
    fn a_default_facing_points_along_the_rigs_own_forward() {
        let facing = Facing::default();

        assert!(
            facing.forward().abs_diff_eq(Vec3::NEG_Z, 1.0e-6),
            "zero yaw should point at -Z, got {:?}",
            facing.forward(),
        );
    }

    #[test]
    fn yaw_and_direction_round_trip() {
        for yaw in [0.0_f32, 0.5, -0.5, FRAC_PI_2, -FRAC_PI_2, 3.0, -3.0] {
            let facing = Facing::at(yaw);
            let recovered = yaw_of(facing.forward());

            assert!(
                shortest_angle(recovered - yaw).abs() < 1.0e-5,
                "yaw {yaw} produced {:?}, which reads back as {recovered}",
                facing.forward(),
            );
        }
    }

    // -----------------------------------------------------------------
    // Shortest way round — the classic wrap bug
    // -----------------------------------------------------------------

    #[test]
    fn turning_past_the_wrap_takes_the_short_way() {
        // THE test. A character at +170 degrees asked to face -170 must
        // turn 20 degrees, not 340. Getting this wrong is not subtle: the
        // character spins almost all the way round to arrive somewhere it
        // was nearly at already.
        let mut facing = Facing::at(170.0_f32.to_radians());
        facing.target_yaw = (-170.0_f32).to_radians();

        let remaining = facing.remaining();

        assert!(
            (remaining.abs() - 20.0_f32.to_radians()).abs() < 1.0e-4,
            "expected 20 degrees of turn, got {}",
            remaining.to_degrees(),
        );
        assert!(
            remaining > 0.0,
            "and it should turn the POSITIVE way, through 180 — got {}",
            remaining.to_degrees(),
        );
    }

    #[test]
    fn the_short_way_is_taken_in_both_directions() {
        let mut facing = Facing::at((-170.0_f32).to_radians());
        facing.target_yaw = 170.0_f32.to_radians();

        let remaining = facing.remaining();

        assert!(
            (remaining.abs() - 20.0_f32.to_radians()).abs() < 1.0e-4,
            "got {}",
            remaining.to_degrees(),
        );
        assert!(remaining < 0.0, "should turn negative, got {}", remaining.to_degrees());
    }

    #[test]
    fn shortest_angle_lands_in_the_documented_range() {
        // Including the values that tempt an off-by-one at the boundary.
        for turns in -4..=4 {
            for base in [0.0_f32, 0.1, PI - 0.001, PI, -PI, PI + 0.001] {
                let angle = base + turns as f32 * std::f32::consts::TAU;
                let wrapped = shortest_angle(angle);

                assert!(
                    (-PI..PI).contains(&wrapped),
                    "{angle} wrapped to {wrapped}, outside [-pi, pi)",
                );
            }
        }
    }

    #[test]
    fn wrapping_preserves_the_direction_an_angle_names() {
        for angle in [0.0_f32, 1.0, -1.0, 7.0, -7.0, 100.0] {
            let wrapped = shortest_angle(angle);

            let original = Quat::from_rotation_y(angle) * Vec3::NEG_Z;
            let rewrapped = Quat::from_rotation_y(wrapped) * Vec3::NEG_Z;

            assert!(
                original.abs_diff_eq(rewrapped, 1.0e-4),
                "{angle} and its wrap {wrapped} should name the same heading",
            );
        }
    }

    // -----------------------------------------------------------------
    // Turning
    // -----------------------------------------------------------------

    #[test]
    fn a_facing_reaches_its_target_and_stops() {
        let mut facing = Facing::at(0.0);
        facing.target_yaw = 1.0;

        for _ in 0..200 {
            facing.advance(1.0 / 60.0);
        }

        assert!(
            facing.is_settled(1.0e-4),
            "the heading should have arrived, {} rad remaining",
            facing.remaining(),
        );
    }

    #[test]
    fn a_facing_never_overshoots_its_target() {
        // A turn-rate limit rather than a spring, deliberately: a heading
        // that rings is a character that cannot hold still while aiming.
        let mut facing = Facing::at(0.0);
        facing.target_yaw = 0.01;
        facing.turn_rate = 100.0; // vastly more than needed in one step

        facing.advance(1.0 / 60.0);

        assert!(
            (facing.yaw - 0.01).abs() < 1.0e-6,
            "expected to land exactly on the target, got {}",
            facing.yaw,
        );
    }

    #[test]
    fn a_facing_turns_at_its_own_rate() {
        let mut facing = Facing::at(0.0);
        // Deliberately not exactly PI: a half-turn is equally short both
        // ways, so which direction it picks is a tie-break rather than a
        // property — see `an_exact_half_turn_is_ambiguous_and_picks_one_way`.
        facing.target_yaw = 3.0;
        facing.turn_rate = 1.0;

        facing.advance(0.5);

        assert!(
            (facing.yaw - 0.5).abs() < 1.0e-5,
            "half a second at 1 rad/s should be 0.5 rad, got {}",
            facing.yaw,
        );
    }

    #[test]
    fn an_exact_half_turn_is_ambiguous_and_picks_one_way() {
        // Both directions are exactly as short, so there is no "correct"
        // answer — only a consistent one. Pinned so the tie-break is a
        // decision rather than an accident, and so a test that happens to
        // choose PI is not read as finding a bug.
        let mut facing = Facing::at(0.0);
        facing.target_yaw = PI;
        facing.turn_rate = 1.0;

        facing.advance(0.5);

        assert!(
            (facing.yaw.abs() - 0.5).abs() < 1.0e-5,
            "it should turn half a radian whichever way it picks, got {}",
            facing.yaw,
        );

        // And it still arrives.
        for _ in 0..200 {
            facing.advance(1.0 / 60.0);
        }
        assert!(facing.is_settled(1.0e-4), "{} rad remaining", facing.remaining());
    }

    #[test]
    fn turning_toward_a_wrapped_target_goes_the_short_way() {
        // The wrap under actual integration, not just in `remaining`.
        let mut facing = Facing::at(3.0);
        facing.target_yaw = -3.0;
        facing.turn_rate = 1.0;

        facing.advance(0.1);

        // 3.0 -> -3.0 the short way is through +pi, so yaw INCREASES.
        assert!(
            facing.yaw > 3.0,
            "should turn toward +pi, but went from 3.0 to {}",
            facing.yaw,
        );
    }

    #[test]
    fn a_settled_facing_does_not_drift() {
        let mut facing = Facing::at(0.7);

        for _ in 0..600 {
            facing.advance(1.0 / 60.0);
        }

        assert!(
            (facing.yaw - 0.7).abs() < 1.0e-5,
            "a facing with nothing to do drifted to {}",
            facing.yaw,
        );
    }

    #[test]
    fn a_long_spin_does_not_accumulate_magnitude() {
        // A character turning one way for a long time must not wander off
        // into a range where f32 resolution near the value starts to
        // matter.
        let mut facing = Facing::at(0.0);
        facing.turn_rate = 10.0;

        for _ in 0..2000 {
            facing.target_yaw = shortest_angle(facing.yaw + 1.0);
            facing.advance(1.0 / 60.0);
        }

        assert!(
            facing.yaw.abs() <= PI,
            "yaw grew to {} instead of staying wrapped",
            facing.yaw,
        );
    }

    // -----------------------------------------------------------------
    // Aiming
    // -----------------------------------------------------------------

    #[test]
    fn facing_a_direction_sets_the_matching_yaw() {
        let mut facing = Facing::default();

        facing.face_direction(Vec3::NEG_Z);
        assert!(facing.target_yaw.abs() < 1.0e-6, "-Z is zero yaw");

        // A positive yaw points toward -X, the character's own LEFT — see
        // `yaw_of`'s note. Writing this the intuitive way round is what
        // produced the sign error it documents.
        facing.face_direction(Vec3::NEG_X);
        assert!(
            (facing.target_yaw - FRAC_PI_2).abs() < 1.0e-5,
            "-X should be a positive quarter turn, got {}",
            facing.target_yaw,
        );

        facing.face_direction(Vec3::X);
        assert!(
            (facing.target_yaw + FRAC_PI_2).abs() < 1.0e-5,
            "+X should be a negative quarter turn, got {}",
            facing.target_yaw,
        );
    }

    #[test]
    fn facing_ignores_a_directions_vertical_component() {
        let mut level = Facing::default();
        level.face_direction(Vec3::new(1.0, 0.0, -1.0));

        let mut steep = Facing::default();
        steep.face_direction(Vec3::new(1.0, 5.0, -1.0));

        assert!(
            (level.target_yaw - steep.target_yaw).abs() < 1.0e-6,
            "a target overhead should not change the heading",
        );
    }

    #[test]
    fn facing_a_degenerate_direction_changes_nothing() {
        // No heading to adopt, so inventing one would snap the character
        // somewhere arbitrary.
        let mut facing = Facing::at(0.7);

        facing.face_direction(Vec3::ZERO);
        assert_eq!(facing.target_yaw, 0.7);

        facing.face_direction(Vec3::Y);
        assert_eq!(facing.target_yaw, 0.7, "straight up has no heading either");
    }

    #[test]
    fn facing_a_position_aims_from_the_given_point() {
        let mut facing = Facing::default();
        facing.face_position(Vec3::new(0.0, 0.0, 5.0), Vec3::new(0.0, 0.0, 0.0));

        // The target is at -Z from the viewer, which is zero yaw.
        assert!(facing.target_yaw.abs() < 1.0e-6, "got {}", facing.target_yaw);
    }

    #[test]
    fn a_non_finite_timestep_or_angle_is_safe() {
        let mut facing = Facing::at(0.5);

        facing.advance(f32::NAN);
        assert_eq!(facing.yaw, 0.5, "NaN dt must not corrupt the heading");

        facing.advance(-1.0);
        assert_eq!(facing.yaw, 0.5);

        assert_eq!(shortest_angle(f32::NAN), 0.0);
        assert_eq!(shortest_angle(f32::INFINITY), 0.0);
    }

    #[test]
    fn a_zero_turn_rate_holds_the_heading() {
        let mut facing = Facing::at(0.0);
        facing.target_yaw = 1.0;
        facing.turn_rate = 0.0;

        facing.advance(1.0);

        assert_eq!(facing.yaw, 0.0, "a character that cannot turn should not turn");
    }
}
