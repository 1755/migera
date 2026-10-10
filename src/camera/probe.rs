//! How the camera asks the world about geometry.
//!
//! [`CameraProbe`] is two questions: where does a sphere swept along a
//! line first touch something, and does a sphere at a point overlap
//! anything. [`AvianProbe`] answers them from avian's spatial query;
//! [`SdfProbe`] answers them analytically from signed distance fields, so
//! collision is unit-tested without a physics world (a sphere swept through
//! an SDF is sphere tracing with the radius subtracted).

use crate::sdf::primitives::{Box3, Sdf};
use avian3d::prelude::*;
use bevy::prelude::*;

/// Where a swept sphere first touched geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProbeHit {
    /// Distance travelled along the sweep; 0 if it started overlapping.
    pub distance: f32,
    /// The surface normal at the touch, pointing out of the geometry.
    pub normal: Vec3,
}

pub trait CameraProbe {
    /// Sweeps a sphere of `radius` from `origin` along the unit `direction`
    /// for up to `max_distance`.
    fn sweep(&self, origin: Vec3, direction: Vec3, max_distance: f32, radius: f32) -> Option<ProbeHit>;
    /// Whether a sphere of `radius` at `point` overlaps geometry.
    fn overlaps(&self, point: Vec3, radius: f32) -> bool;
}

/// A world with nothing in it.
pub struct NoProbe;

impl CameraProbe for NoProbe {
    fn sweep(&self, _: Vec3, _: Vec3, _: f32, _: f32) -> Option<ProbeHit> {
        None
    }

    fn overlaps(&self, _: Vec3, _: f32) -> bool {
        false
    }
}

/// avian's spatial query, filtered to what blocks a camera.
pub struct AvianProbe<'a, 'w, 's> {
    pub query: &'a SpatialQuery<'w, 's>,
    pub filter: SpatialQueryFilter,
    /// Entities that never block (`CameraIgnore`, the target's own
    /// colliders); checked per hit.
    pub ignore: &'a dyn Fn(Entity) -> bool,
}

impl CameraProbe for AvianProbe<'_, '_, '_> {
    fn sweep(&self, origin: Vec3, direction: Vec3, max_distance: f32, radius: f32) -> Option<ProbeHit> {
        let direction = Dir3::new(direction).ok()?;
        // A sweep that starts touching a surface and moves away from it is
        // free: a shoulder point slid up to a wall must still be able to
        // sweep along or away from it. Penetration is `overlaps`'s question.
        let config = ShapeCastConfig { max_distance, ignore_origin_penetration: true, ..ShapeCastConfig::DEFAULT };
        self.query
            .cast_shape_predicate(
                &Collider::sphere(radius),
                origin,
                Quat::IDENTITY,
                direction,
                &config,
                &self.filter,
                &|entity| !(self.ignore)(entity),
            )
            .map(|hit| ProbeHit { distance: hit.distance, normal: hit.normal1 })
    }

    fn overlaps(&self, point: Vec3, radius: f32) -> bool {
        let mut found = false;
        self.query.shape_intersections_callback(
            &Collider::sphere(radius),
            point,
            Quat::IDENTITY,
            &self.filter,
            |entity| {
                if (self.ignore)(entity) {
                    true
                } else {
                    found = true;
                    false
                }
            },
        );
        found
    }
}

/// One shape of an [`SdfProbe`] world.
pub struct SdfShape {
    pub sdf: Box<dyn Sdf>,
    pub center: Vec3,
    pub rotation: Quat,
}

/// Geometry as signed distance fields, for tests.
#[derive(Default)]
pub struct SdfProbe {
    pub shapes: Vec<SdfShape>,
}

impl SdfProbe {
    /// Adds an axis-aligned box of `size` centred on `center`.
    pub fn with_box(mut self, center: Vec3, size: Vec3) -> Self {
        self.shapes.push(SdfShape {
            sdf: Box::new(Box3 { half_extents: size * 0.5 }),
            center,
            rotation: Quat::IDENTITY,
        });
        self
    }

    /// The signed distance to the nearest surface.
    pub fn distance(&self, p: Vec3) -> f32 {
        self.shapes
            .iter()
            .map(|s| s.sdf.distance(s.rotation.inverse() * (p - s.center)))
            .fold(f32::INFINITY, f32::min)
    }

    fn normal(&self, p: Vec3) -> Vec3 {
        let e = 1.0e-3;
        Vec3::new(
            self.distance(p + Vec3::X * e) - self.distance(p - Vec3::X * e),
            self.distance(p + Vec3::Y * e) - self.distance(p - Vec3::Y * e),
            self.distance(p + Vec3::Z * e) - self.distance(p - Vec3::Z * e),
        )
        .normalize_or_zero()
    }
}

impl CameraProbe for SdfProbe {
    fn sweep(&self, origin: Vec3, direction: Vec3, max_distance: f32, radius: f32) -> Option<ProbeHit> {
        // Sphere tracing the field shrunk by the radius. Exact distances
        // (boxes) make every step safe, so no wall, however thin, is
        // stepped over.
        const CONTACT: f32 = 1.0e-4;
        let mut t = 0.0;
        for _ in 0..512 {
            let p = origin + direction * t;
            let d = self.distance(p) - radius;
            if d < CONTACT {
                let normal = self.normal(p);
                // Starting in contact and moving away (as avian's
                // `ignore_origin_penetration`): not a hit.
                if t == 0.0 && d > -CONTACT && normal.dot(direction) > 0.0 {
                    t = 2.0 * CONTACT;
                    continue;
                }
                return Some(ProbeHit { distance: t, normal });
            }
            t += d;
            if t > max_distance {
                return None;
            }
        }
        None
    }

    fn overlaps(&self, point: Vec3, radius: f32) -> bool {
        self.distance(point) < radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_sweep_stops_one_radius_short_of_a_wall() {
        let probe = SdfProbe::default().with_box(Vec3::new(0.0, 0.0, -5.0), Vec3::new(10.0, 10.0, 1.0));
        let hit = probe.sweep(Vec3::ZERO, Vec3::NEG_Z, 10.0, 0.2).expect("the wall is in the way");
        assert!((hit.distance - 4.3).abs() < 1.0e-3, "front face at 4.5, minus 0.2: {}", hit.distance);
        assert!(hit.normal.dot(Vec3::Z) > 0.99, "normal points back at the sweep: {:?}", hit.normal);
    }

    #[test]
    fn a_sweep_never_passes_a_thin_wall() {
        let probe = SdfProbe::default().with_box(Vec3::new(0.0, 0.0, -3.0), Vec3::new(4.0, 4.0, 0.02));
        let hit = probe.sweep(Vec3::ZERO, Vec3::NEG_Z, 10.0, 0.15);
        assert!(hit.is_some_and(|h| h.distance < 3.0), "a 2 cm wall must stop the sweep: {hit:?}");
    }

    #[test]
    fn a_sweep_starting_inside_reports_zero() {
        let probe = SdfProbe::default().with_box(Vec3::ZERO, Vec3::ONE);
        let hit = probe.sweep(Vec3::ZERO, Vec3::X, 5.0, 0.1).unwrap();
        assert_eq!(hit.distance, 0.0);
        assert!(probe.overlaps(Vec3::ZERO, 0.1));
        assert!(!probe.overlaps(Vec3::new(3.0, 0.0, 0.0), 0.1));
    }
}
