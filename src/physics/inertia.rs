//! Closed-form mass/inertia-tensor formulas for `PhysicsShape` primitives —
//! standard rigid-body-dynamics results (see e.g. "Moments of Inertia for
//! Common Shapes," *Game Engine Gems* Vol. 1), avoiding any need for
//! Monte-Carlo/numerical mass integration for the primitives this renderer's
//! scene is built from.
//!
//! Every formula returns the **diagonal** of the inertia tensor in the
//! shape's own local (unrotated) frame — every current primitive's
//! closed-form inertia is diagonal in that frame, so `Inertia::inverse_tensor_diag`
//! (`components.rs`) never needs off-diagonal terms. `RoundedBox`/
//! `RoundedCylinder` are approximated by their un-rounded base box/cylinder:
//! there is no exact closed form for the rounding's contribution to
//! inertia, and for the corner/edge radii this project's scenes actually
//! use, the rounding is a small fraction of total volume — a standard,
//! documented approximation, not a claimed exactness.
//!
//! `combine_parallel_axis` combines multiple primitives' inertia into one
//! compound-body tensor via the parallel axis theorem — exact for **hard**
//! unions only (see `physics::compound`'s future doc comment for why
//! smooth-blended dynamic compounds are out of scope: the true mass
//! distribution bulges into the blend region with no closed form).

use bevy::prelude::*;

/// Sphere of `radius`, mass `mass`: isotropic `I = (2/5) m r²` on every axis.
pub fn sphere_inertia(mass: f32, radius: f32) -> Vec3 {
    let i = 0.4 * mass * radius * radius;
    Vec3::splat(i)
}

/// Box with the given local-space half-extents, mass `mass`. Standard
/// rectangular-cuboid formula, expressed in full side lengths
/// `(2a, 2b, 2c) = 2 * half_extents`: `Ixx = m(b²+c²)/3`, and cyclically for
/// `Iyy`/`Izz` (the `/3` — not the `/12` a mass-distributed-at-the-corners
/// derivation might suggest — comes from integrating a uniform solid box,
/// not point masses at the corners).
pub fn box_inertia(mass: f32, half_extents: Vec3) -> Vec3 {
    let full = 2.0 * half_extents;
    let (a, b, c) = (full.x, full.y, full.z);
    Vec3::new(
        mass * (b * b + c * c) / 3.0,
        mass * (a * a + c * c) / 3.0,
        mass * (a * a + b * b) / 3.0,
    )
}

/// Solid cylinder, axis along local Y (matching `Shape::RoundedCylinder`'s
/// own `half_height`-along-Y convention — see `sdf::primitives::RoundedCylinder`),
/// mass `mass`: axial `Iyy = (1/2) m r²`, transverse `Ixx = Izz = m(r²/4 + h²/3)`
/// where `h` is the full height `2 * half_height`.
pub fn cylinder_inertia(mass: f32, radius: f32, half_height: f32) -> Vec3 {
    let height = 2.0 * half_height;
    let axial = 0.5 * mass * radius * radius;
    let transverse = mass * (radius * radius / 4.0 + height * height / 12.0);
    Vec3::new(transverse, axial, transverse)
}

/// Capsule: a cylinder of half-segment-length `half_segment` between the two
/// sphere centers, plus two hemispherical caps of `radius`, axis along local
/// Y (matching `Shape::Capsule`'s `a`/`b` endpoints once recentered to the
/// segment midpoint — the caller is expected to have already reduced the
/// capsule to `(radius, half_segment)`, since inertia is about the body's
/// own center of mass, not the raw `a`/`b` world-space endpoints). Closed
/// form: cylinder inertia plus two hemisphere pieces, each hemisphere's own
/// inertia about its own center of mass combined via the parallel axis
/// theorem for its offset from the capsule's overall center of mass (a
/// hemisphere's center of mass sits `(3/8)r` from its flat face, not at the
/// sphere center it was cut from — the standard correction most
/// capsule-inertia derivations point out explicitly).
pub fn capsule_inertia(mass: f32, radius: f32, half_segment: f32) -> Vec3 {
    let r = radius;
    let h = half_segment;

    // Split total mass between the cylindrical middle and the two
    // hemispherical caps (together one full sphere) by volume ratio, so a
    // caller supplying the capsule's total mass gets a physically
    // consistent split rather than needing to pre-partition it themselves.
    let cylinder_volume = std::f32::consts::PI * r * r * (2.0 * h);
    let sphere_volume = (4.0 / 3.0) * std::f32::consts::PI * r * r * r;
    let total_volume = cylinder_volume + sphere_volume;
    let cylinder_mass = mass * cylinder_volume / total_volume;
    let sphere_mass = mass * sphere_volume / total_volume; // both caps combined

    // Cylindrical middle, about its own (= the capsule's) center.
    let cyl = cylinder_inertia(cylinder_mass, r, h);

    // Two hemispheres, each of mass `sphere_mass / 2`, each offset from the
    // capsule's center by `h + (3/8) r` along the axis (half_segment to the
    // flat face, plus the hemisphere's own center-of-mass offset beyond it).
    let hemi_mass = sphere_mass / 2.0;
    let hemi_offset = h + (3.0 / 8.0) * r;
    // A solid hemisphere's inertia about its OWN center of mass:
    // axial (about the symmetry axis, matching a full sphere): (2/5) m r².
    // transverse (about an axis through its own COM, perpendicular to the
    // symmetry axis): (83/320) m r² — standard tabulated hemisphere result.
    let hemi_axial_own = 0.4 * hemi_mass * r * r;
    let hemi_transverse_own = (83.0 / 320.0) * hemi_mass * r * r;
    // Parallel-axis shift only applies to the transverse axes (the axial
    // axis passes through both the hemisphere's own COM and the capsule's
    // COM, since both lie on the symmetry axis).
    let hemi_transverse_shifted = hemi_transverse_own + hemi_mass * hemi_offset * hemi_offset;

    // Two hemispheres (top + bottom cap) contribute their axial term
    // unshifted (twice) and their shifted transverse term (twice).
    let axial = cyl.y + 2.0 * hemi_axial_own;
    let transverse = cyl.x + 2.0 * hemi_transverse_shifted;

    Vec3::new(transverse, axial, transverse)
}

/// Ellipsoid with local-space semi-axes `radii = (a, b, c)`, mass `mass`:
/// `Ixx = (1/5) m (b² + c²)`, and cyclically.
pub fn ellipsoid_inertia(mass: f32, radii: Vec3) -> Vec3 {
    let (a, b, c) = (radii.x, radii.y, radii.z);
    Vec3::new(0.2 * mass * (b * b + c * c), 0.2 * mass * (a * a + c * c), 0.2 * mass * (a * a + b * b))
}

/// Combines multiple primitives' inertia tensors (each already computed
/// about its OWN center of mass, in world/compound-aligned axes — the
/// caller is responsible for rotating each piece's local diagonal tensor
/// into the compound's frame before calling this, since the parallel axis
/// theorem's rotation term is not itself provided here) into one compound
/// tensor about the compound's overall center of mass, via the parallel
/// axis theorem: `I_compound = Σ(I_i + m_i · (|d_i|² · Identity − d_i ⊗ d_i))`,
/// where `d_i` is piece `i`'s offset from the compound's center of mass.
///
/// Exact for **hard** unions of non-overlapping (or only mildly
/// overlapping — overlap double-counts mass slightly, a standard accepted
/// approximation in game physics) primitives. NOT exact for smooth-blended
/// (`SmoothUnion`) compounds, whose true mass distribution bulges into the
/// blend region with no closed form — `physics::compound` (a later stage)
/// only ever calls this for hard-union pieces.
pub fn combine_parallel_axis(pieces: &[(Mat3, f32, Vec3)]) -> Mat3 {
    let mut total = Mat3::ZERO;
    for &(local_tensor, mass, offset) in pieces {
        let d = offset;
        let shift = Mat3::from_diagonal(Vec3::splat(d.length_squared())) - outer_product(d, d);
        total += local_tensor + mass * shift;
    }
    total
}

fn outer_product(a: Vec3, b: Vec3) -> Mat3 {
    Mat3::from_cols(a * b.x, a * b.y, a * b.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f32 = 1e-4;

    fn assert_close(a: Vec3, b: Vec3, eps: f32) {
        assert!((a - b).length() < eps, "expected {b:?}, got {a:?}");
    }

    #[test]
    fn unit_mass_unit_radius_sphere_has_inertia_two_fifths() {
        let i = sphere_inertia(1.0, 1.0);
        assert_close(i, Vec3::splat(0.4), EPS);
    }

    #[test]
    fn sphere_inertia_is_isotropic() {
        let i = sphere_inertia(3.0, 2.0);
        assert!((i.x - i.y).abs() < EPS);
        assert!((i.y - i.z).abs() < EPS);
    }

    #[test]
    fn cube_box_inertia_is_isotropic() {
        // A cube (equal half-extents on every axis) must have equal
        // diagonal inertia terms by symmetry, regardless of the exact
        // formula's derivation — a strong sanity check independent of the
        // specific /3 constant.
        let i = box_inertia(2.0, Vec3::splat(1.5));
        assert!((i.x - i.y).abs() < EPS);
        assert!((i.y - i.z).abs() < EPS);
    }

    #[test]
    fn box_inertia_matches_known_analytic_formula() {
        // Unit-mass box, half-extents (1, 2, 3) -> full sides (2, 4, 6).
        let i = box_inertia(1.0, Vec3::new(1.0, 2.0, 3.0));
        let (a, b, c) = (2.0f32, 4.0, 6.0);
        let expected =
            Vec3::new((b * b + c * c) / 3.0, (a * a + c * c) / 3.0, (a * a + b * b) / 3.0);
        assert_close(i, expected, EPS);
    }

    #[test]
    fn cylinder_axial_matches_half_m_r_squared() {
        let i = cylinder_inertia(1.0, 2.0, 5.0);
        assert!((i.y - 0.5 * 1.0 * 2.0 * 2.0).abs() < EPS);
    }

    #[test]
    fn cylinder_transverse_axes_are_equal() {
        let i = cylinder_inertia(4.0, 1.5, 2.0);
        assert!((i.x - i.z).abs() < EPS);
    }

    #[test]
    fn capsule_reduces_toward_sphere_as_segment_shrinks_to_zero() {
        // A capsule with zero segment length degenerates to a sphere: axial
        // and transverse inertia should both approach the sphere formula
        // (2/5) m r^2 for the same total mass.
        let i = capsule_inertia(1.0, 1.0, 0.0);
        let sphere = sphere_inertia(1.0, 1.0);
        assert_close(i, sphere, 1e-3);
    }

    #[test]
    fn capsule_transverse_axes_are_equal() {
        let i = capsule_inertia(2.0, 0.5, 1.0);
        assert!((i.x - i.z).abs() < EPS);
    }

    #[test]
    fn ellipsoid_reduces_to_sphere_formula_when_radii_equal() {
        let i = ellipsoid_inertia(1.0, Vec3::splat(1.0));
        assert_close(i, sphere_inertia(1.0, 1.0), EPS);
    }

    #[test]
    fn parallel_axis_dumbbell_matches_textbook_formula() {
        // Two unit-mass, zero-radius point-like spheres (small radius to
        // approximate a point mass) offset by +/-2 along X from the
        // dumbbell's center: transverse inertia about the center should
        // approach 2 * m * d^2 (point-mass approximation), since each
        // sphere's own inertia is negligible compared to the parallel-axis
        // shift at this offset-to-radius ratio.
        let radius = 0.01f32;
        let mass = 1.0f32;
        let offset = 2.0f32;
        let own = Mat3::from_diagonal(sphere_inertia(mass, radius));
        let pieces = [
            (own, mass, Vec3::new(offset, 0.0, 0.0)),
            (own, mass, Vec3::new(-offset, 0.0, 0.0)),
        ];
        let combined = combine_parallel_axis(&pieces);
        let expected_transverse = 2.0 * mass * offset * offset;
        assert!((combined.y_axis.y - expected_transverse).abs() / expected_transverse < 1e-3);
        assert!((combined.z_axis.z - expected_transverse).abs() / expected_transverse < 1e-3);
        // About the shared X axis (both point masses lie on it), inertia
        // should stay near-zero (just the point spheres' own tiny inertia).
        assert!(combined.x_axis.x < 1e-2);
    }

    #[test]
    fn combine_parallel_axis_with_no_pieces_is_zero() {
        assert_eq!(combine_parallel_axis(&[]), Mat3::ZERO);
    }
}
