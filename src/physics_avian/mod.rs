//! Primary physics engine: `avian3d` (wrapping the mature `parry3d`/Rapier-
//! family solver), adopted after a side-by-side comparison against this
//! project's own custom SDF-native CPU+GPU solver (`crate::physics`) found
//! that every real scene built with the custom engine only ever collided
//! simple convex primitives — exactly avian3d's own native strength, at a
//! fraction of the maintenance cost (avian3d already ships joints, real
//! CCD, and sleeping, none of which the custom engine has). See
//! `crate::physics`'s own module doc comment for the demotion note, and the
//! plan document that recorded this decision for the full comparison.
//!
//! This module owns exactly one thing beyond re-exporting avian3d itself:
//! the shape-mapping bridge from this project's own SDF-renderer-facing
//! `sdf::components::Shape` enum to `avian3d::prelude::Collider`, so a
//! single entity can carry both `Shape` (consumed by the SDF renderer,
//! unchanged) and an avian3d `Collider`+`RigidBody` (consumed by avian3d's
//! own solver) without duplicating shape parameters into a second,
//! avian3d-specific authoring type.
//!
//! `PhysicsPlugins::default()` itself is added directly by each avian3d-
//! based example/app, not wrapped in a plugin here — there is no
//! project-specific configuration of avian3d's own plugin set yet, so
//! wrapping it would be an indirection with no payoff.

pub mod gravity;
pub mod layers;

use avian3d::prelude::Collider;

use crate::sdf::components::Shape;

/// Maps this project's own SDF-renderer-facing `Shape` enum to an
/// avian3d `Collider`, so a `Shape`-carrying entity (already rendered by
/// the SDF pipeline, unchanged) can ALSO carry a physically-accurate
/// avian3d collider with no duplicated shape-parameter authoring.
///
/// Two shapes have no exact avian3d equivalent, and are handled by a
/// deliberate, documented approximation rather than silently dropped:
///
/// - `RoundedCylinder`: avian3d has no edge-rounded-cylinder primitive, so
///   this maps to a plain `Collider::cylinder` using the shape's own outer
///   `radius`/`half_height` — the edge rounding itself has no collision
///   effect under this approximation (a sharp-edged cylinder collides
///   very slightly differently at the very edge of its rounded rim than
///   the true rounded shape would), which is acceptable for every
///   existing use of this shape (crate pyramids etc. in the demoted
///   custom-engine examples use `RoundedBox`, not `RoundedCylinder`, for
///   their own stacking milestones — this shape has never been on a
///   collision-critical path).
/// - `Ellipsoid`: avian3d has no dedicated ellipsoid primitive either, but
///   its own `Collider::sphere` scaling behavior already does the right
///   thing here: a unit sphere collider on an entity whose `Transform.scale`
///   is set to the ellipsoid's own `radii` gets automatically approximated
///   by avian3d as a convex polyhedron once the scale is non-uniform
///   (confirmed via avian3d's own docs: "if the scaling factor is not
///   uniform... the shape is approximated as a convex polygon or
///   polyhedron"). This function therefore returns a unit
///   `Collider::sphere(1.0)` for `Ellipsoid` — the CALLER is responsible
///   for also setting `Transform.scale = radii` on the same entity (this
///   function only builds the collider, it doesn't touch `Transform`,
///   matching every other branch here which likewise assumes the caller's
///   own `Transform` already encodes the shape's world pose).
///
/// `RoundedCone` returns `None`, unsupported — consistent with this
/// project's own independently-tracked `RoundedCone` SDF distance-function
/// bug (see `crate::physics::components`'s own module doc comment): no
/// new support is being built for a shape whose own distance function is
/// already known-broken.
pub fn shape_to_collider(shape: &Shape) -> Option<Collider> {
    match *shape {
        Shape::Sphere { radius } => Some(Collider::sphere(radius)),
        // A `corner_radius <= 0.0` (a plain, sharp-edged box — common for
        // room walls/floors, which have no real edge rounding) must NOT
        // go through `Collider::round_cuboid`: parry's `RoundShape` is a
        // generic GJK/EPA support-map shape whose margin is exactly what
        // makes the Minkowski difference well-conditioned for EPA's
        // penetration-normal computation — at a zero radius that margin
        // collapses, and EPA occasionally returns a garbage contact
        // normal on face/edge/vertex-degenerate configurations. This is
        // a REAL bug found live: a room built from `corner_radius: 0.0`
        // walls (examples/cornell_room.rs) let the solver's own bad
        // normal impulses fling a dynamic sphere hard enough to tunnel
        // through a thin (0.3-unit) wall collider, drifting the scene's
        // BVH bounds unboundedly over hundreds of frames until it fed an
        // absurd probe-grid size into DDGI and crashed on a >2GB buffer
        // allocation — not from an obviously-wrong first-frame explosion,
        // but a slow, stochastic one, which made it easy to misattribute
        // to something else. `Collider::cuboid` has no such margin
        // requirement (parry's `Cuboid` is a genuine closed-form convex
        // primitive, not a support-map approximation), so every existing
        // caller passing `corner_radius: 0.0` for a sharp-edged box gets
        // the exact, well-conditioned shape it actually means.
        Shape::RoundedBox { half_extents, corner_radius } if corner_radius <= 0.0 => {
            Some(Collider::cuboid(half_extents.x * 2.0, half_extents.y * 2.0, half_extents.z * 2.0))
        }
        Shape::RoundedBox { half_extents, corner_radius } => {
            Some(Collider::round_cuboid(half_extents.x * 2.0, half_extents.y * 2.0, half_extents.z * 2.0, corner_radius))
        }
        Shape::RoundedCylinder { radius, half_height, edge_radius: _ } => Some(Collider::cylinder(radius, half_height * 2.0)),
        Shape::Capsule { a, b, radius } => Some(Collider::capsule(radius, (b - a).length())),
        Shape::Ellipsoid { radii: _ } => Some(Collider::sphere(1.0)),
        Shape::BoxFrame { .. } | Shape::HexPrism { .. } | Shape::RoundedCone { .. } => None,
    }
}

#[cfg(test)]
mod tests {
    use bevy::math::Vec3;

    use super::*;

    #[test]
    fn every_shape_kind_this_project_actually_uses_for_collision_maps_to_a_collider() {
        // BoxFrame/HexPrism/RoundedCone deliberately return None (see this
        // module's own doc comment) -- every other variant must succeed.
        assert!(shape_to_collider(&Shape::Sphere { radius: 1.0 }).is_some());
        assert!(shape_to_collider(&Shape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.1 }).is_some());
        assert!(shape_to_collider(&Shape::RoundedCylinder { radius: 1.0, half_height: 0.5, edge_radius: 0.1 }).is_some());
        assert!(shape_to_collider(&Shape::Capsule { a: Vec3::ZERO, b: Vec3::Y, radius: 0.3 }).is_some());
        assert!(shape_to_collider(&Shape::Ellipsoid { radii: Vec3::new(1.0, 2.0, 3.0) }).is_some());
    }

    #[test]
    fn rounded_cone_and_frame_shapes_are_unsupported() {
        assert!(shape_to_collider(&Shape::RoundedCone { a: Vec3::ZERO, b: Vec3::Y, r0: 1.0, r1: 0.5 }).is_none());
        assert!(shape_to_collider(&Shape::BoxFrame { half_extents: Vec3::ONE, wall_thickness: 0.1 }).is_none());
        assert!(shape_to_collider(&Shape::HexPrism { radius: 1.0, half_height: 0.5 }).is_none());
    }

    /// Regression test for a real bug found live: a `RoundedBox` with
    /// `corner_radius: 0.0` (a plain sharp-edged box — the common case for
    /// room walls/floors, which have no real edge rounding) MUST map to a
    /// genuine `Collider::cuboid`, not a degenerate zero-radius
    /// `Collider::round_cuboid`. See this function's own doc comment on
    /// the `corner_radius <= 0.0` branch for the full mechanism (parry's
    /// `RoundShape` support-map margin collapses to zero, producing
    /// occasional ill-conditioned EPA contact normals) — this test pins
    /// the fix at the API boundary, independent of ever reproducing the
    /// solver-level symptom (a sphere tunneling through a thin wall after
    /// hundreds of physics ticks) in a unit test.
    #[test]
    fn a_sharp_edged_rounded_box_maps_to_a_real_cuboid_not_a_degenerate_round_shape() {
        let sharp = shape_to_collider(&Shape::RoundedBox { half_extents: Vec3::new(1.0, 2.0, 3.0), corner_radius: 0.0 })
            .expect("RoundedBox always maps to a collider");
        assert!(
            sharp.shape().as_cuboid().is_some(),
            "corner_radius <= 0.0 must produce a plain Collider::cuboid, not a round_cuboid"
        );
        assert!(sharp.shape().as_round_cuboid().is_none());

        // A genuinely positive corner_radius must still take the
        // round_cuboid path — this guard must not swallow the real case.
        let rounded = shape_to_collider(&Shape::RoundedBox { half_extents: Vec3::ONE, corner_radius: 0.1 })
            .expect("RoundedBox always maps to a collider");
        assert!(rounded.shape().as_round_cuboid().is_some(), "a positive corner_radius must still produce a round_cuboid");
    }

    /// A negative `corner_radius` (never authored deliberately, but not
    /// structurally impossible given `Shape::RoundedBox`'s own field has
    /// no type-level non-negativity guarantee) must take the same safe
    /// `cuboid` path as exactly zero, not fall through to
    /// `Collider::round_cuboid` with a negative radius parry would have
    /// to handle as an even more degenerate case.
    #[test]
    fn a_negative_corner_radius_also_maps_to_a_real_cuboid() {
        let shape = shape_to_collider(&Shape::RoundedBox { half_extents: Vec3::ONE, corner_radius: -0.1 })
            .expect("RoundedBox always maps to a collider");
        assert!(shape.shape().as_cuboid().is_some());
    }
}
