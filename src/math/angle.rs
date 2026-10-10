//! Scalar angle helpers: wrapping, shortest-path damping, stick shaping and
//! soft limits.
//!
//! Angles are radians. Anything that damps or compares yaw must go through
//! [`angle_delta`]: a raw `target - current` swings the long way round when
//! the two sit either side of ±π.

use bevy::math::Vec2;
use std::f32::consts::{LN_2, PI, TAU};

/// Wraps `angle` into `(-π, π]`.
#[inline]
pub fn wrap_angle(angle: f32) -> f32 {
    let wrapped = (angle + PI).rem_euclid(TAU) - PI;
    // rem_euclid lands -π exactly on -π; fold it onto +π so the range is
    // half-open on the side that keeps 180° positive.
    if wrapped <= -PI {
        wrapped + TAU
    } else {
        wrapped
    }
}

/// The signed shortest rotation from `from` to `to`, in `(-π, π]`.
#[inline]
pub fn angle_delta(from: f32, to: f32) -> f32 {
    wrap_angle(to - from)
}

/// Moves `current` toward `target` along the shortest arc with exponential
/// decay of the given half-life, exactly (frame-rate independent). The
/// result is wrapped.
#[inline]
pub fn damp_angle(current: f32, target: f32, halflife: f32, dt: f32) -> f32 {
    if dt <= 0.0 {
        return current;
    }
    let keep = (-LN_2 / halflife.max(1.0e-5) * dt).exp();
    wrap_angle(current + angle_delta(current, target) * (1.0 - keep))
}

/// Exponential decay of a scalar toward `target` with the given half-life.
#[inline]
pub fn damp(current: f32, target: f32, halflife: f32, dt: f32) -> f32 {
    if dt <= 0.0 {
        return current;
    }
    let keep = (-LN_2 / halflife.max(1.0e-5) * dt).exp();
    target + (current - target) * keep
}

/// A radial deadzone that rescales what is left to the full `0..1` range, so
/// leaving the deadzone does not jump straight to its edge value.
#[inline]
pub fn radial_deadzone(stick: Vec2, deadzone: f32) -> Vec2 {
    let length = stick.length();
    if length <= deadzone || length <= 0.0 {
        return Vec2::ZERO;
    }
    let scaled = ((length - deadzone) / (1.0 - deadzone).max(1.0e-5)).min(1.0);
    stick / length * scaled
}

/// Shapes a stick axis in `-1..1` with a power curve: exponent 1 is linear,
/// larger exponents give fine control near the centre and full speed at the
/// edge (the response curve Nesky recommends over linear).
#[inline]
pub fn stick_curve(axis: f32, exponent: f32) -> f32 {
    axis.signum() * axis.abs().min(1.0).powf(exponent.max(0.01))
}

/// Scales a `rate` that pushes `value` toward one of its limits, so it slows
/// to zero over the last `band` before the limit; a rate moving away from a
/// limit passes unchanged.
#[inline]
pub fn soft_limit_rate(value: f32, rate: f32, min: f32, max: f32, band: f32) -> f32 {
    if band <= 0.0 {
        return rate;
    }
    let room = if rate > 0.0 {
        max - value
    } else if rate < 0.0 {
        value - min
    } else {
        return 0.0;
    };
    rate * (room / band).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrapping_keeps_every_angle_in_the_half_open_range() {
        for i in -2000..=2000 {
            let a = i as f32 * 0.01;
            let w = wrap_angle(a);
            assert!(w > -PI && w <= PI, "{a} wrapped to {w}, outside (-π, π]");
            assert!(
                (w.sin() - a.sin()).abs() < 1.0e-4 && (w.cos() - a.cos()).abs() < 1.0e-4,
                "{a} wrapped to {w}, a different direction",
            );
        }
    }

    #[test]
    fn damping_an_angle_takes_the_short_way_across_the_wrap() {
        // From 170° to -170° is 20° through 180°, not 340° through 0°.
        let from = 170f32.to_radians();
        let to = -170f32.to_radians();
        let next = damp_angle(from, to, 0.1, 1.0 / 60.0);
        let moved = angle_delta(from, next);
        assert!(moved > 0.0, "must move toward +180°, moved {} rad", moved);
        assert!(moved < 20f32.to_radians(), "and not past the target, moved {moved}");
    }

    #[test]
    fn angle_damping_is_frame_rate_independent() {
        let run = |steps: usize| {
            let dt = 1.0 / steps as f32;
            let mut a = 3.0;
            for _ in 0..steps {
                a = damp_angle(a, -3.0, 0.2, dt);
            }
            a
        };
        let (a30, a144) = (run(30), run(144));
        assert!(angle_delta(a30, a144).abs() < 1.0e-4, "{a30} at 30 Hz vs {a144} at 144 Hz");
    }

    #[test]
    fn the_deadzone_swallows_small_input_and_rescales_the_rest() {
        assert_eq!(radial_deadzone(Vec2::new(0.1, 0.05), 0.15), Vec2::ZERO);
        let just_out = radial_deadzone(Vec2::new(0.16, 0.0), 0.15);
        assert!(just_out.x > 0.0 && just_out.x < 0.02, "leaving the deadzone starts near 0");
        assert!((radial_deadzone(Vec2::new(1.0, 0.0), 0.15).x - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn the_stick_curve_keeps_sign_and_endpoints() {
        assert_eq!(stick_curve(1.0, 2.0), 1.0);
        assert_eq!(stick_curve(-1.0, 2.0), -1.0);
        assert!((stick_curve(-0.5, 2.0) + 0.25).abs() < 1.0e-6);
    }

    #[test]
    fn a_soft_limit_slows_toward_the_limit_but_not_away_from_it() {
        let (min, max, band) = (-0.5, 1.0, 0.2);
        assert_eq!(soft_limit_rate(0.0, 1.0, min, max, band), 1.0);
        assert!((soft_limit_rate(0.9, 1.0, min, max, band) - 0.5).abs() < 1.0e-5);
        assert_eq!(soft_limit_rate(1.0, 1.0, min, max, band), 0.0);
        assert_eq!(soft_limit_rate(1.0, -1.0, min, max, band), -1.0);
    }
}
