//! Rig shape: what a camera mode *is*, and how an orbit becomes an eye pose.
//!
//! A mode is a [`ModeParams`]: distance, height and FOV as curves of pitch
//! (Nesky's "like gears": the three shift together as the camera tilts),
//! a shoulder offset, pitch limits and whether it recentres. Modes blend by
//! evaluating each at the current pitch and interpolating the resulting
//! scalars ([`RigShape`], [`OrbitShape`]), so a blended camera always stays
//! on an orbit around the pivot rather than cutting the chord between two
//! eye positions.

use bevy::prelude::*;
use serde::{Deserialize, Serialize};

/// A piecewise-linear curve keyed by pitch in **degrees** (elevation:
/// positive = camera above the pivot looking down). Keys must be sorted by
/// pitch; values past either end hold the end key.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct PitchCurve {
    pub keys: Vec<(f32, f32)>,
}

impl PitchCurve {
    /// A curve with the same value at every pitch.
    pub fn constant(value: f32) -> Self {
        Self { keys: vec![(0.0, value)] }
    }

    pub fn new(keys: &[(f32, f32)]) -> Self {
        Self { keys: keys.to_vec() }
    }

    /// The value at `pitch` (radians).
    pub fn at(&self, pitch: f32) -> f32 {
        let degrees = pitch.to_degrees();
        let keys = &self.keys;
        let Some(&(first_pitch, first_value)) = keys.first() else {
            return 0.0;
        };
        if degrees <= first_pitch {
            return first_value;
        }
        for pair in keys.windows(2) {
            let ((p0, v0), (p1, v1)) = (pair[0], pair[1]);
            if degrees <= p1 {
                let t = if p1 > p0 { (degrees - p0) / (p1 - p0) } else { 1.0 };
                return v0 + (v1 - v0) * t;
            }
        }
        keys[keys.len() - 1].1
    }
}

/// One camera mode's parameters. Angles in degrees for authoring.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Reflect)]
pub struct ModeParams {
    /// Boom length from the shoulder point to the eye, metres.
    pub distance: PitchCurve,
    /// Extra height of the shoulder point above the pivot, metres.
    pub height: PitchCurve,
    /// Vertical field of view, degrees.
    pub fov: PitchCurve,
    /// Sideways offset of the shoulder point along the camera's flat right,
    /// metres. Positive = camera over the right shoulder; 0 = centred.
    pub shoulder: f32,
    pub pitch_min: f32,
    pub pitch_max: f32,
    /// The pitch auto-recentring returns to.
    pub pitch_default: f32,
    /// Whether this mode recentres behind movement (1) or never (0).
    /// Blends as a weight.
    pub recenter: f32,
}

impl ModeParams {
    /// Free exploration: a little over the shoulder, closer at a worm's-eye
    /// angle and farther looking down, wider FOV low so the sky shows.
    pub fn exploration() -> Self {
        Self {
            distance: PitchCurve::new(&[(-40.0, 2.0), (15.0, 3.2), (70.0, 4.2)]),
            height: PitchCurve::new(&[(-40.0, 0.15), (15.0, 0.0)]),
            fov: PitchCurve::new(&[(-40.0, 68.0), (15.0, 60.0), (70.0, 58.0)]),
            shoulder: 0.25,
            pitch_min: -40.0,
            pitch_max: 70.0,
            pitch_default: 15.0,
            recenter: 1.0,
        }
    }

    /// Combat: closer and higher (Gothic's melee mode pulls in to 2.5 m at
    /// 35°), wider shoulder, never recentres on its own.
    pub fn combat() -> Self {
        Self {
            distance: PitchCurve::new(&[(-30.0, 1.9), (20.0, 2.6), (70.0, 3.4)]),
            height: PitchCurve::constant(0.1),
            fov: PitchCurve::new(&[(-30.0, 66.0), (20.0, 60.0)]),
            shoulder: 0.35,
            pitch_min: -30.0,
            pitch_max: 70.0,
            pitch_default: 20.0,
            recenter: 0.0,
        }
    }

    pub fn rig_shape(&self, pitch: f32) -> RigShape {
        RigShape {
            distance: self.distance.at(pitch),
            height: self.height.at(pitch),
            fov: self.fov.at(pitch).to_radians(),
            shoulder: self.shoulder,
        }
    }

    pub fn orbit_shape(&self) -> OrbitShape {
        OrbitShape {
            pitch_min: self.pitch_min.to_radians(),
            pitch_max: self.pitch_max.to_radians(),
            pitch_default: self.pitch_default.to_radians(),
            recenter: self.recenter,
        }
    }
}

/// A mode's boom evaluated at one pitch. Radians.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct RigShape {
    pub distance: f32,
    pub height: f32,
    pub fov: f32,
    pub shoulder: f32,
}

impl RigShape {
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            distance: self.distance + (other.distance - self.distance) * t,
            height: self.height + (other.height - self.height) * t,
            fov: self.fov + (other.fov - self.fov) * t,
            shoulder: self.shoulder + (other.shoulder - self.shoulder) * t,
        }
    }
}

/// A mode's orbit limits and recentring weight. Radians.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct OrbitShape {
    pub pitch_min: f32,
    pub pitch_max: f32,
    pub pitch_default: f32,
    pub recenter: f32,
}

impl OrbitShape {
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            pitch_min: self.pitch_min + (other.pitch_min - self.pitch_min) * t,
            pitch_max: self.pitch_max + (other.pitch_max - self.pitch_max) * t,
            pitch_default: self.pitch_default + (other.pitch_default - self.pitch_default) * t,
            recenter: self.recenter + (other.recenter - self.recenter) * t,
        }
    }
}

/// The camera's rotation for an orbit: yaw about world +Y (0 looks along
/// −Z, the walker's convention), then pitch as elevation (positive tilts the
/// view down). Built from two axis rotations, so it has no roll for any
/// input.
#[inline]
pub fn orbit_rotation(yaw: f32, pitch: f32) -> Quat {
    Quat::from_rotation_y(yaw) * Quat::from_rotation_x(-pitch)
}

/// The yaw a camera must have to look along a horizontal `direction`.
#[inline]
pub fn yaw_of(direction: Vec3) -> f32 {
    (-direction.x).atan2(-direction.z)
}

/// The camera pose the rig wants before collision.
#[derive(Debug, Clone, Copy, PartialEq, Reflect)]
pub struct DesiredPose {
    /// The point the camera orbits.
    pub pivot: Vec3,
    /// The pivot raised by the mode's height and moved sideways by its
    /// shoulder offset: where the boom starts.
    pub shoulder: Vec3,
    pub eye: Vec3,
    pub rotation: Quat,
    /// Vertical FOV, radians.
    pub fov: f32,
    pub distance: f32,
}

/// Places the eye for an orbit (`yaw`, `pitch`, radians) around `pivot`.
/// The shoulder offset is applied along the *flat* right (yaw only), so it
/// stays horizontal at any pitch; the camera looks parallel to the orbit's
/// forward, past the character, the way an over-the-shoulder rig does.
pub fn desired_pose(pivot: Vec3, yaw: f32, pitch: f32, shape: &RigShape, zoom: f32) -> DesiredPose {
    let rotation = orbit_rotation(yaw, pitch);
    let flat_right = Quat::from_rotation_y(yaw) * Vec3::X;
    let shoulder = pivot + Vec3::Y * shape.height + flat_right * shape.shoulder;
    let distance = shape.distance * zoom;
    let eye = shoulder + rotation * Vec3::new(0.0, 0.0, distance);
    DesiredPose { pivot, shoulder, eye, rotation, fov: shape.fov, distance }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_orbit_rotation_never_has_roll_for_any_yaw_and_pitch() {
        for yi in -36..=36 {
            for pi in -17..=17 {
                let (yaw, pitch) = (yi as f32 * 0.1745, pi as f32 * 0.0872);
                let right = orbit_rotation(yaw, pitch) * Vec3::X;
                assert!(
                    right.y.abs() < 1.0e-5,
                    "yaw {yaw} pitch {pitch}: camera right {right:?} leaves the horizontal",
                );
            }
        }
    }

    #[test]
    fn positive_pitch_puts_the_eye_above_the_pivot_looking_down() {
        let shape = ModeParams::exploration().rig_shape(0.5);
        let pose = desired_pose(Vec3::ZERO, 0.0, 0.5, &RigShape { shoulder: 0.0, height: 0.0, ..shape }, 1.0);
        assert!(pose.eye.y > 0.0, "elevated camera must sit above the pivot: {:?}", pose.eye);
        assert!(pose.eye.z > 0.0, "yaw 0 looks along -Z, so the eye is behind at +Z");
        let forward = pose.rotation * Vec3::NEG_Z;
        assert!(forward.y < 0.0, "and look down, forward {forward:?}");
        let to_pivot = (pose.pivot - pose.eye).normalize();
        assert!(forward.dot(to_pivot) > 0.9999, "and at the pivot when centred");
    }

    #[test]
    fn the_shoulder_offset_stays_horizontal_at_any_pitch() {
        for pitch in [-0.6, 0.0, 1.1] {
            let shape = RigShape { distance: 3.0, height: 0.0, fov: 1.0, shoulder: 0.3 };
            let pose = desired_pose(Vec3::ZERO, 0.7, pitch, &shape, 1.0);
            assert!(pose.shoulder.y.abs() < 1.0e-6, "pitch {pitch}: shoulder {:?}", pose.shoulder);
            assert!((pose.shoulder.length() - 0.3).abs() < 1.0e-5);
        }
    }

    #[test]
    fn a_pitch_curve_interpolates_and_holds_its_ends() {
        let curve = PitchCurve::new(&[(-40.0, 2.0), (20.0, 3.0)]);
        assert_eq!(curve.at((-80f32).to_radians()), 2.0);
        assert_eq!(curve.at(60f32.to_radians()), 3.0);
        assert!((curve.at((-10f32).to_radians()) - 2.5).abs() < 1.0e-5);
        assert_eq!(PitchCurve::constant(1.5).at(1.0), 1.5);
    }

    #[test]
    fn yaw_of_inverts_the_orbit_forward() {
        for yaw in [-3.0, -1.0, 0.0, 0.5, 2.5] {
            let forward = orbit_rotation(yaw, 0.0) * Vec3::NEG_Z;
            assert!((crate::math::angle::angle_delta(yaw_of(forward), yaw)).abs() < 1.0e-5);
        }
    }
}
