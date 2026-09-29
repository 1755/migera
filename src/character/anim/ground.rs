//! Where the ground is.
//!
//! The superseded module hardcoded `y = 0` in three separate places, which
//! made terrain, stairs, and slopes structurally impossible. This is the
//! seam that replaces it: one trait, so the flat-plane case stays trivial
//! and a real world can be plugged in without touching the IK.

use bevy::math::Vec3;

/// What the ground looks like beneath a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundHit {
    /// Ground height at the queried position, world space.
    pub height: f32,
    /// Surface normal. Used to level a foot against a slope rather than
    /// leaving it flat and half-buried.
    pub normal: Vec3,
}

impl GroundHit {
    /// Flat ground at a given height.
    pub const fn flat(height: f32) -> Self {
        Self { height, normal: Vec3::Y }
    }
}

/// Answers "what is under this point".
///
/// Implemented by the consumer, because only the consumer knows whether
/// that means a physics raycast, a heightmap lookup, or nothing at all.
/// The animation stack only needs the answer.
pub trait GroundProbe: Send + Sync + 'static {
    /// The ground beneath `world_position`, or `None` where there is none
    /// — over a ledge, say, where a foot should keep following the
    /// animation rather than being planted on empty space.
    fn sample(&self, world_position: Vec3) -> Option<GroundHit>;
}

/// An infinite horizontal plane.
///
/// The default, and enough for a flat test scene. Keeping it as an explicit
/// implementation rather than a special case means the flat path exercises
/// exactly the same code as a real world.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlatGround {
    /// The plane's height.
    pub height: f32,
}

impl Default for FlatGround {
    fn default() -> Self {
        Self { height: 0.0 }
    }
}

impl GroundProbe for FlatGround {
    fn sample(&self, _world_position: Vec3) -> Option<GroundHit> {
        Some(GroundHit::flat(self.height))
    }
}

/// A ground plane tilted about the X axis — a ramp.
///
/// Exists so slopes can be tested without standing up a physics world.
/// `grade` is the rise per metre travelled along `-Z`, the rig's forward.
///
/// # What it took to make slopes work
///
/// Slopes exposed three separate bugs that flat ground hides, because on
/// level ground the surface height is constant and every error cancels:
///
/// 1. **Within-frame feedback.** The ground was sampled from the
///    in-progress solve, so raising a target moved the foot forward, which
///    sampled higher ground, which raised the target again. Now sampled
///    once, from the animated pose.
/// 2. **Cross-frame feedback.** The correction was written into
///    [`super::plugin::AnimPose::state`], so the next frame's "animated"
///    pose was already corrected and the same loop ran a frame at a time.
///    Now kept separately in `AnimFootIk::corrected`.
/// 3. **The proxy rig.** The IK solved in this crate's synthetic T-pose
///    space while ground height is a property of the real world. The two
///    rigs put a toe at `z = -0.168` and `z ≈ 0.088` respectively — on
///    flat ground the same surface, on a slope a 0.06 m difference the
///    solver then chased by swinging the whole leg 72 degrees forward.
///    Fixed by [`super::rig::RigGeometry`].
///
/// The first two were real and neither was sufficient. Diagnosing the
/// third took BRP-measuring the live rig, because every unit test passed
/// throughout — they all measured the synthetic rig, which is exactly the
/// thing that was wrong.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SlopedGround {
    /// Height at the origin.
    pub height: f32,
    /// Rise per metre along `-Z`. Positive slopes upward ahead.
    pub grade: f32,
}

impl GroundProbe for SlopedGround {
    fn sample(&self, world_position: Vec3) -> Option<GroundHit> {
        Some(GroundHit {
            height: self.height + self.grade * -world_position.z,
            // The surface normal of `y = h + g * (-z)`, normalized.
            normal: Vec3::new(0.0, 1.0, self.grade).normalize(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_ground_reports_the_same_height_everywhere() {
        let ground = FlatGround { height: 0.25 };

        for position in [Vec3::ZERO, Vec3::new(10.0, 5.0, -3.0), Vec3::new(-7.0, 0.0, 9.0)] {
            let hit = ground.sample(position).expect("flat ground is everywhere");
            assert_eq!(hit.height, 0.25);
            assert_eq!(hit.normal, Vec3::Y, "a flat plane points straight up");
        }
    }

    #[test]
    fn flat_ground_defaults_to_the_world_origin() {
        assert_eq!(FlatGround::default().height, 0.0);
    }

    #[test]
    fn a_slope_rises_ahead_of_the_character() {
        // The rig faces -Z, so a positive grade should climb as the
        // character advances.
        let ground = SlopedGround { height: 0.0, grade: 0.2 };

        let here = ground.sample(Vec3::ZERO).unwrap().height;
        let ahead = ground.sample(Vec3::new(0.0, 0.0, -1.0)).unwrap().height;
        let behind = ground.sample(Vec3::new(0.0, 0.0, 1.0)).unwrap().height;

        assert!(ahead > here, "the slope should rise ahead: {here} -> {ahead}");
        assert!(behind < here, "...and fall behind: {here} -> {behind}");
        assert!((ahead - here - 0.2).abs() < 1.0e-6, "by exactly the grade");
    }

    #[test]
    fn a_slopes_normal_leans_away_from_vertical() {
        let ground = SlopedGround { height: 0.0, grade: 0.5 };
        let normal = ground.sample(Vec3::ZERO).unwrap().normal;

        assert!(normal.is_normalized(), "a normal must be unit length");
        assert!(normal.y > 0.0, "and must point upward, got {normal:?}");
        assert!(
            normal.dot(Vec3::Y) < 0.99,
            "a 0.5 grade should tilt the normal noticeably, got {normal:?}",
        );
    }

    #[test]
    fn a_flat_slope_is_indistinguishable_from_flat_ground() {
        let sloped = SlopedGround { height: 0.4, grade: 0.0 };
        let flat = FlatGround { height: 0.4 };

        let position = Vec3::new(2.0, 0.0, -3.0);
        assert_eq!(sloped.sample(position), flat.sample(position));
    }
}
