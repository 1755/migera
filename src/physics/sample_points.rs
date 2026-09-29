//! Claybook-style fixed local-space sample-point sets, baked once per
//! `PhysicsShape` kind (Dennis Gustafsson / Sebastian Aaltonen, GDC 2018
//! "GPU-Based Clay Simulation and Ray-Tracing Tech in Claybook"): instead
//! of a single closest-point distance query (which gives exactly one
//! contact point — not enough for stable multi-point manifolds, e.g. a box
//! resting flat needs contact at multiple corners to avoid rocking), each
//! body carries a small fixed set of points on its own surface. At contact-
//! generation time (`physics::contacts`), each sample point is transformed
//! to world space and queried against the OTHER body's SDF for depth and
//! normal — turning "one shape vs. one shape" into "N cheap point queries
//! vs. one shape," with zero geometric intersection routine and a natural
//! multi-point manifold for free.
//!
//! Every returned point must lie exactly on its shape's own surface (within
//! floating-point tolerance) — a sample point that's off-surface silently
//! corrupts every downstream contact's depth and normal, so this is the
//! single most important property this module's own tests check.
//!
//! Returned as a fixed-size `SamplePoints` array (a `[Vec3; MAX_SAMPLE_POINTS]`
//! plus a `count`), not a heap-allocated `Vec<Vec3>` — the CPU reference
//! mirrors the GPU port's own actual data shape exactly (no heap allocation,
//! same fixed capacity per shape kind), the same "the reference must model
//! the real algorithm, not just independently reach a correct answer"
//! standard this project's `solve_rigid::Accumulator` already follows for
//! the GPU's atomic scatter targets. `MAX_SAMPLE_POINTS` is the max sample
//! count across every `PhysicsShape` variant (currently `Sphere`'s own 32);
//! every shape kind fills a prefix of the array up to its own `count` and
//! leaves the rest zeroed/unused.

use bevy::prelude::*;

use super::components::PhysicsShape;

const RING_SAMPLES: usize = 6;

/// The max sample count across every `PhysicsShape` variant — the fixed
/// array capacity `SamplePoints` uses for every shape kind, chosen once at
/// this module's largest current need (`Sphere`'s own count, see that
/// constant's own doc comment) so a single array size serves all shapes,
/// matching the GPU port's own fixed-buffer-stride requirement (WGSL has no
/// dynamically-sized per-shape arrays inside a fixed-stride struct).
pub const MAX_SAMPLE_POINTS: usize = 32;

/// Sample count for `Sphere` specifically — higher than `Ellipsoid`'s own
/// default (see `ellipsoid_fibonacci`'s doc comment for why count matters
/// for depth/normal accuracy): spheres are the most common dynamic-body
/// shape in practice, so under-sampling them has an outsized effect on
/// solver quality. 32 was chosen empirically — high enough that two
/// unit-radius spheres converge to within the solver's own convergence
/// test tolerance of exactly touching, not a value derived from a closed-
/// form error bound.
const SPHERE_SAMPLE_COUNT: usize = 32;

/// `Ellipsoid`'s own sample count — kept lower than `Sphere`'s since it
/// hasn't yet needed the same accuracy scrutiny; revisit if ellipsoid-vs-
/// ellipsoid contacts ever show the same under-sampling symptom `Sphere`
/// did (systematically shallow contact depth, a too-close resting
/// equilibrium).
const ELLIPSOID_SAMPLE_COUNT: usize = 12;

/// A fixed-capacity, non-heap-allocated set of local-space sample points —
/// see this module's own doc comment for why this replaced a plain
/// `Vec<Vec3>`. `points[..count]` are the shape's own real sample points;
/// `points[count..]` are unused padding (zeroed, never read by any correct
/// caller since every iteration goes through `as_slice`/`Deref`, which both
/// only ever expose the `count`-length prefix).
#[derive(Clone, Copy, Debug)]
pub struct SamplePoints {
    points: [Vec3; MAX_SAMPLE_POINTS],
    count: u32,
}

impl SamplePoints {
    fn new() -> Self {
        Self { points: [Vec3::ZERO; MAX_SAMPLE_POINTS], count: 0 }
    }

    /// Appends one sample point — panics if `MAX_SAMPLE_POINTS` is exceeded,
    /// since that would mean some shape kind's own sample count grew past
    /// the fixed capacity every other shape kind's array already assumes;
    /// this must be caught immediately (a loud panic in a `cargo test` run),
    /// not silently truncated the way a production per-frame path might
    /// need to tolerate overflow (see the GPU contact-buffer's own
    /// atomic-cursor overflow handling in the broad-phase port for the
    /// production-path convention this CPU-only helper deliberately does
    /// NOT need, since shape sample counts are a fixed, compile-time-known
    /// property of this module, not a runtime-variable count).
    fn push(&mut self, point: Vec3) {
        self.points[self.count as usize] = point;
        self.count += 1;
    }

    pub fn as_slice(&self) -> &[Vec3] {
        &self.points[..self.count as usize]
    }

    pub fn len(&self) -> usize {
        self.count as usize
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

impl std::ops::Deref for SamplePoints {
    type Target = [Vec3];

    fn deref(&self) -> &[Vec3] {
        self.as_slice()
    }
}

impl<'a> IntoIterator for &'a SamplePoints {
    type Item = &'a Vec3;
    type IntoIter = std::slice::Iter<'a, Vec3>;

    fn into_iter(self) -> Self::IntoIter {
        self.as_slice().iter()
    }
}

impl IntoIterator for SamplePoints {
    type Item = Vec3;
    type IntoIter = std::iter::Take<std::array::IntoIter<Vec3, MAX_SAMPLE_POINTS>>;

    fn into_iter(self) -> Self::IntoIter {
        let count = self.count as usize;
        self.points.into_iter().take(count)
    }
}

/// Fixed local-space sample points on `shape`'s own surface, sized to give
/// a stable multi-point manifold per shape kind:
/// - `Sphere`: a spherical-Fibonacci set of surface points (NOT a single
///   center point — see this module's own regression-tested bug history
///   below for why that original design was wrong). A single center point
///   was the original Stage 3 design, on the reasoning that "the analytic
///   radius handles the surface offset" — true for DEPTH, but not for the
///   contact NORMAL: the normal comes from the OTHER body's SDF gradient
///   evaluated AT the sample point, and a sphere's center can sit
///   entirely within a flat region of the other body (e.g. a `RoundedBox`
///   floor's flat top face) even while the sphere's own radius overlaps a
///   nearby ROUNDED CORNER — the gradient at the center has no way to
///   "feel" curvature the center itself never touches, so the sphere
///   could rest overlapping a corner with a purely vertical normal and
///   zero horizontal force, never rolling off even where it visibly
///   should (caught via a live example: a sphere dropped near, but not
///   exactly on, a `RoundedBox` floor's rounded corner sat frozen in
///   place indefinitely instead of grazing/rolling across the curve).
///   Surface points fix this the same way every other shape's own sample
///   set already avoids the problem: each sample is where the sphere's
///   surface ACTUALLY is, so the other body's gradient there reflects
///   whatever geometry that specific point on the sphere is near.
/// - `RoundedBox`: 8 corners, inset by `corner_radius` along each axis so
///   each point lies on the shape's own rounded surface (not the sharp
///   corner of the underlying box) — corners are exactly where a resting
///   box needs contact points to avoid rocking.
/// - `RoundedCylinder`/`HexPrism`: a ring of points around each end cap
///   plus the two axis endpoints.
/// - `Capsule`: a ring around each end-cap plus the two axis endpoints.
/// - `Ellipsoid`: a small spherical-Fibonacci point set (deterministic,
///   evenly distributed without the axis-aligned bias a naive lat/long
///   grid would have).
/// - `BoxFrame`: the same corner convention as `RoundedBox` (with no
///   rounding), since exact tests for the interior frame cavity aren't yet
///   needed for this stage's scope.
pub fn sample_points_local(shape: &PhysicsShape) -> SamplePoints {
    match *shape {
        PhysicsShape::Sphere { radius } => ellipsoid_fibonacci(Vec3::splat(radius), SPHERE_SAMPLE_COUNT),
        PhysicsShape::RoundedBox { half_extents, corner_radius } => box_corners(half_extents, corner_radius),
        PhysicsShape::RoundedCylinder { radius, half_height, edge_radius } => {
            cylinder_rings(radius, half_height, edge_radius)
        }
        PhysicsShape::Capsule { a, b, radius } => capsule_rings(a, b, radius),
        PhysicsShape::Ellipsoid { radii } => ellipsoid_fibonacci(radii, ELLIPSOID_SAMPLE_COUNT),
        PhysicsShape::BoxFrame { half_extents, .. } => box_corners(half_extents, 0.0),
        PhysicsShape::HexPrism { radius, half_height } => cylinder_rings(radius, half_height, 0.0),
    }
}

fn box_corners(half_extents: Vec3, corner_radius: f32) -> SamplePoints {
    let corner_radius = corner_radius.max(0.0);
    let inset = half_extents - Vec3::splat(corner_radius);
    let mut points = SamplePoints::new();
    for &sx in &[-1.0, 1.0] {
        for &sy in &[-1.0, 1.0] {
            for &sz in &[-1.0, 1.0] {
                let corner_center = Vec3::new(sx * inset.x, sy * inset.y, sz * inset.z);
                let offset =
                    if corner_radius > 0.0 { Vec3::new(sx, sy, sz).normalize() * corner_radius } else { Vec3::ZERO };
                points.push(corner_center + offset);
            }
        }
    }
    points
}

/// A ring of points on a rounded-cylinder-style surface: the 2D
/// cross-section (radial distance, height) is an inset rectangle
/// `(core_radius, core_half_height)` whose boundary is offset outward by
/// `edge_radius` — matching `sdf::primitives::RoundedCylinder::distance`'s
/// own `d.x.max(d.y).min(0.0) + d.max(ZERO).length() - edge_radius`
/// formula exactly. A ring at the CORNER of that inset rectangle
/// (`radial == core_radius`, `height == core_half_height`, the case this
/// function is used for) must be pushed outward along the corner's own
/// 2D diagonal by `edge_radius` to land on the true rounded surface, the
/// same "inset core, then offset by the rounding radius along the
/// corner's own normal" pattern `box_corners` uses for `RoundedBox`.
fn ring(center: Vec3, core_radius: f32, core_axis_half: f32, edge_radius: f32, sign: f32, out: &mut SamplePoints) {
    let corner_2d = bevy::math::Vec2::new(core_radius, core_axis_half);
    let offset_2d = if edge_radius > 0.0 { corner_2d.normalize() * edge_radius } else { bevy::math::Vec2::ZERO };
    let ring_radius = corner_2d.x + offset_2d.x;
    let ring_axis_half = corner_2d.y + offset_2d.y;
    for i in 0..RING_SAMPLES {
        let angle = (i as f32 / RING_SAMPLES as f32) * std::f32::consts::TAU;
        out.push(center + Vec3::new(ring_radius * angle.cos(), sign * ring_axis_half, ring_radius * angle.sin()));
    }
}

fn cylinder_rings(radius: f32, half_height: f32, edge_radius: f32) -> SamplePoints {
    let edge_radius = edge_radius.max(0.0);
    let core_radius = radius - edge_radius;
    let core_half_height = half_height - edge_radius;
    let mut points = SamplePoints::new();
    ring(Vec3::ZERO, core_radius, core_half_height, edge_radius, 1.0, &mut points);
    ring(Vec3::ZERO, core_radius, core_half_height, edge_radius, -1.0, &mut points);
    points.push(Vec3::new(0.0, half_height, 0.0));
    points.push(Vec3::new(0.0, -half_height, 0.0));
    points
}

fn capsule_rings(a: Vec3, b: Vec3, radius: f32) -> SamplePoints {
    let axis = (b - a).normalize_or(Vec3::Y);
    let (u, v) = axis.any_orthonormal_pair();
    let mut points = SamplePoints::new();
    for &(center, sign) in &[(a, -1.0f32), (b, 1.0f32)] {
        for i in 0..RING_SAMPLES {
            let angle = (i as f32 / RING_SAMPLES as f32) * std::f32::consts::TAU;
            let radial = u * angle.cos() + v * angle.sin();
            points.push(center + radial * radius);
        }
        points.push(center + axis * (radius * sign));
    }
    points
}

/// Spherical Fibonacci lattice, scaled per-axis to the ellipsoid's radii —
/// deterministic and evenly distributed (unlike a lat/long grid, which
/// clusters points at the poles). `count` trades sample density for
/// accuracy: with too few points, no sample may land near the true
/// deepest-penetration direction between two overlapping spheres,
/// systematically underestimating contact depth and settling the solver
/// at a too-close equilibrium instead of exactly touching (a real bug
/// caught by this module's own `Sphere` regression tests during
/// development — fixed by raising `SPHERE_SAMPLE_COUNT`, not by loosening
/// the tests' tolerances, since the tolerance itself wasn't wrong).
fn ellipsoid_fibonacci(radii: Vec3, count: usize) -> SamplePoints {
    let golden_angle = std::f32::consts::PI * (3.0 - 5.0f32.sqrt());
    let mut points = SamplePoints::new();
    for i in 0..count {
        let y = 1.0 - 2.0 * (i as f32 + 0.5) / count as f32;
        let radius_at_y = (1.0 - y * y).max(0.0).sqrt();
        let theta = golden_angle * i as f32;
        let x = theta.cos() * radius_at_y;
        let z = theta.sin() * radius_at_y;
        // Scaling a unit-sphere point per-axis by an ellipsoid's radii does
        // NOT generally land exactly on the ellipsoid's surface distance
        // field at the same normalized position — but it IS exact here:
        // sd_ellipsoid's surface is defined as the set of points p with
        // |p / radii| == 1, and (x,y,z) is already unit-length by
        // construction (a point on the unit sphere), so radii * (x,y,z)
        // satisfies |p / radii| == |(x,y,z)| == 1 exactly.
        points.push(Vec3::new(x, y, z) * radii);
    }
    points
}

#[cfg(test)]
mod tests {
    use crate::hybrid::cpu_ref;
    use crate::sdf::components::Shape;

    use super::*;

    const EPS: f32 = 1e-3;

    fn assert_on_surface(shape: &PhysicsShape, point: Vec3) {
        let full_shape: Shape = (*shape).into();
        let distance = cpu_ref::local_distance(&full_shape, point);
        assert!(distance.abs() < EPS, "sample point {point:?} is off-surface by {distance} for {shape:?}");
    }

    #[test]
    fn every_sphere_sample_lies_on_the_surface() {
        let shape = PhysicsShape::Sphere { radius: 1.5 };
        for p in sample_points_local(&shape) {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn sphere_samples_are_spread_around_the_surface_not_clustered_at_one_point() {
        // The whole point of moving off a single center sample: multiple
        // DISTINCT surface points, not all landing in the same spot (which
        // would still fail to detect nearby curvature from most
        // directions). Checks every sample is farther than a small
        // tolerance from every other sample.
        let shape = PhysicsShape::Sphere { radius: 1.0 };
        let points = sample_points_local(&shape);
        assert!(points.len() > 1, "expected multiple sample points, got {}", points.len());
        for i in 0..points.len() {
            for j in (i + 1)..points.len() {
                assert!((points[i] - points[j]).length() > 0.1, "samples {i} and {j} are suspiciously close: {:?} vs {:?}", points[i], points[j]);
            }
        }
    }

    #[test]
    fn every_rounded_box_corner_sample_lies_on_the_surface() {
        let shape = PhysicsShape::RoundedBox { half_extents: Vec3::new(1.0, 2.0, 3.0), corner_radius: 0.3 };
        let points = sample_points_local(&shape);
        assert_eq!(points.len(), 8);
        for p in points {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn a_sharp_rounded_box_corner_sample_also_lies_on_the_surface() {
        // corner_radius = 0.0 degenerates to a sharp box — the sample must
        // still land exactly on the (sharp) surface, not off by the
        // now-absent rounding inset.
        let shape = PhysicsShape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 };
        for p in sample_points_local(&shape) {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn every_rounded_cylinder_sample_lies_on_the_surface() {
        let shape = PhysicsShape::RoundedCylinder { radius: 1.0, half_height: 2.0, edge_radius: 0.2 };
        let points = sample_points_local(&shape);
        assert_eq!(points.len(), RING_SAMPLES * 2 + 2);
        for p in points {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn every_capsule_sample_lies_on_the_surface() {
        let shape = PhysicsShape::Capsule { a: Vec3::new(0.0, -1.0, 0.0), b: Vec3::new(0.0, 1.0, 0.0), radius: 0.5 };
        let points = sample_points_local(&shape);
        assert_eq!(points.len(), (RING_SAMPLES + 1) * 2);
        for p in points {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn every_ellipsoid_sample_lies_on_the_surface() {
        let shape = PhysicsShape::Ellipsoid { radii: Vec3::new(1.0, 2.0, 0.5) };
        for p in sample_points_local(&shape) {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn every_box_frame_sample_lies_on_the_surface() {
        // BoxFrame samples reuse the box-corner convention against its
        // outer half_extents (wall_thickness has no rounding role here) —
        // confirmed on-surface the same way as RoundedBox's sharp case.
        let shape = PhysicsShape::BoxFrame { half_extents: Vec3::splat(1.0), wall_thickness: 0.1 };
        for p in sample_points_local(&shape) {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn every_hex_prism_sample_lies_on_the_surface() {
        let shape = PhysicsShape::HexPrism { radius: 1.0, half_height: 0.5 };
        for p in sample_points_local(&shape) {
            assert_on_surface(&shape, p);
        }
    }

    #[test]
    fn sample_point_count_is_a_pinned_regression() {
        // Pins the exact count per shape kind so an accidental change in
        // sampling density is caught explicitly, not just silently
        // absorbed by downstream contact-generation tests.
        assert_eq!(sample_points_local(&PhysicsShape::Sphere { radius: 1.0 }).len(), 32);
        assert_eq!(
            sample_points_local(&PhysicsShape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.1 }).len(),
            8
        );
    }

    #[test]
    fn max_sample_points_is_never_exceeded_by_any_shape_kind() {
        // MAX_SAMPLE_POINTS must stay >= every shape kind's own count --
        // SamplePoints::push panics past capacity, so this test would fail
        // loudly (not silently corrupt data) if a future shape kind's
        // sample count ever grew past the current fixed array size, but
        // pinning it explicitly here catches the mismatch with a clear
        // failure message rather than a panic stack trace during an
        // unrelated test.
        let shapes = [
            PhysicsShape::Sphere { radius: 1.0 },
            PhysicsShape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.1 },
            PhysicsShape::RoundedCylinder { radius: 1.0, half_height: 1.0, edge_radius: 0.1 },
            PhysicsShape::Capsule { a: Vec3::ZERO, b: Vec3::Y, radius: 0.5 },
            PhysicsShape::Ellipsoid { radii: Vec3::new(1.0, 2.0, 0.5) },
            PhysicsShape::BoxFrame { half_extents: Vec3::ONE, wall_thickness: 0.1 },
            PhysicsShape::HexPrism { radius: 1.0, half_height: 0.5 },
        ];
        for shape in shapes {
            let count = sample_points_local(&shape).len();
            assert!(count <= MAX_SAMPLE_POINTS, "{shape:?} produced {count} sample points, exceeding MAX_SAMPLE_POINTS ({MAX_SAMPLE_POINTS})");
        }
    }

    #[test]
    fn sample_points_padding_beyond_count_is_never_exposed() {
        // The fixed array's unused tail (points[count..]) must never leak
        // into anything a caller can observe -- as_slice/len/Deref/
        // IntoIterator all must agree on exactly `count` elements, not
        // MAX_SAMPLE_POINTS.
        let shape = PhysicsShape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.1 };
        let points = sample_points_local(&shape);
        assert_eq!(points.len(), 8);
        assert_eq!(points.as_slice().len(), 8);
        assert_eq!((&points).into_iter().count(), 8);
        assert_eq!(points.into_iter().count(), 8);
    }
}
