//! Exact/bound SDF primitives, formulas per docs/knowledge/sdf-3d/primitives-and-operators.

use bevy::math::Vec3;

/// Which closed-form distance function a `GpuPrimitive` names — mirrors the `TAG_LEAF_*`
/// constants `assets/shaders/raymarch.wgsl` dispatches on; see `Sdf::gpu_record`'s doc
/// comment for why this exists at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GpuShapeKind {
    Sphere,
    RoundedBox,
    RoundedCylinder,
    Capsule,
    RoundedCone,
    Ellipsoid,
    BoxFrame,
    HexPrism,
}

/// A primitive's shape kind + up to 4 scalar params, in the same field-reuse-by-tag
/// encoding `raymarch::flatten::PrimitiveRecordCpu` uploads to the GPU — see that
/// type's doc comment for the exact per-kind param mapping. Kept here (not in
/// `raymarch::flatten`) so each concrete `Sdf` impl is the one source of truth for its
/// own params, rather than `raymarch::flatten` needing a second `match` over every
/// concrete primitive type (which `Box<dyn Sdf>` can't support anyway — trait objects
/// can't be downcast without `Any`, and adding `Any` just to re-derive what each impl
/// already knows about itself would be strictly more indirection than asking the impl
/// directly).
#[derive(Clone, Copy, Debug)]
pub struct GpuPrimitive {
    pub kind: GpuShapeKind,
    pub params: [f32; 8],
}

/// Anything that can report a signed distance to a point in its own local space.
pub trait Sdf: Send + Sync {
    /// Signed distance from `p` to the surface. Negative = inside.
    fn distance(&self, p: Vec3) -> f32;

    /// This primitive's GPU-uploadable shape kind + params, for `raymarch::flatten` to
    /// bake into a `PrimitiveRecordCpu` — the one piece of type-erased-unfriendly
    /// information a `Box<dyn Sdf>` can't otherwise expose (see `GpuPrimitive`'s doc
    /// comment). Every concrete primitive below implements this trivially, just
    /// re-stating its own already-owned fields in the shared encoding.
    fn gpu_record(&self) -> GpuPrimitive;
}

pub struct Sphere {
    pub radius: f32,
}

impl Sdf for Sphere {
    fn distance(&self, p: Vec3) -> f32 {
        p.length() - self.radius
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::Sphere,
            params: [self.radius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    }
}

/// Sharp-edged box. Unused by the demo scene — the ground uses `RoundedBox3` instead,
/// since `Box3`'s hard edges cause real splat-sampling gaps (see `RoundedBox3`'s doc
/// comment) — kept here as part of the primitive library for completeness.
#[allow(dead_code)]
pub struct Box3 {
    pub half_extents: Vec3,
}

impl Sdf for Box3 {
    fn distance(&self, p: Vec3) -> f32 {
        let q = p.abs() - self.half_extents;
        q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0)
    }

    /// No `TAG_LEAF_BOX` exists in the raymarcher (this primitive is unused by the
    /// demo scene, see this type's doc comment) — reuses `RoundedBox` at
    /// `corner_radius = 0.0`, which the shader's `sdf_rounded_box` formula reduces to
    /// exactly `Box3`'s own formula for.
    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::RoundedBox,
            params: [
                self.half_extents.x,
                self.half_extents.y,
                self.half_extents.z,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        }
    }
}

/// A box with rounded edges/corners: `Box3`'s exact distance minus a fixed corner
/// radius, the standard SDF rounding trick (offsetting a shape's distance field
/// outward by a constant shrinks its hard edges into fillets of that radius, since the
/// zero level set moves outward uniformly in the surface normal direction everywhere).
///
/// Exists because a true `Box3`'s edges are a genuine gradient discontinuity — the
/// surface normal jumps instantly across the crease, which has real, structural
/// consequences for splat baking, not just cosmetic ones: `bake::principal_curvature_frame`'s
/// finite-difference curvature estimate is unreliable exactly at that discontinuity,
/// and `bake::project_to_surface`'s Newton iteration can fail to converge for
/// candidates whose nearest point on the surface sits in the crease itself (a
/// degenerate, medial-axis-like gradient there) — both effects reduce candidate
/// acceptance right at hard edges, which showed up as a visible seam of missing splat
/// coverage along the ground box's flat-top/side crease. A small rounding radius
/// (much smaller than the sampling spacing, so it reads as visually sharp) gives the
/// sampler a continuous, well-defined normal everywhere instead.
pub struct RoundedBox3 {
    pub half_extents: Vec3,
    pub corner_radius: f32,
}

impl Sdf for RoundedBox3 {
    fn distance(&self, p: Vec3) -> f32 {
        let q = p.abs() - self.half_extents + Vec3::splat(self.corner_radius);
        q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0) - self.corner_radius
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::RoundedBox,
            params: [
                self.half_extents.x,
                self.half_extents.y,
                self.half_extents.z,
                self.corner_radius,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        }
    }
}

/// Infinite ground plane through the origin with the given upward normal. Unused by
/// the demo scene (a flattened `RoundedBox3` is used for the ground instead, since an
/// infinite plane would swallow the whole finite bake volume — see
/// docs/knowledge/sdf-3d/primitives-and-operators/primitive-shapes.md), kept here as
/// part of the primitive library for completeness.
#[allow(dead_code)]
pub struct Plane {
    pub normal: Vec3,
}

impl Sdf for Plane {
    fn distance(&self, p: Vec3) -> f32 {
        p.dot(self.normal)
    }

    /// No raymarcher tag represents a true infinite plane (the `TAG_LEAF_*` set mirrors
    /// exactly the 4 `Shape` variants the demo scene actually uses — see
    /// `raymarch::flatten`'s module doc). This primitive is unused dead code with no
    /// call site to ever invoke this, so it panics rather than silently return a
    /// misleading finite approximation.
    fn gpu_record(&self) -> GpuPrimitive {
        unimplemented!(
            "Plane has no raymarcher GPU representation (unused primitive, see this type's doc comment)"
        )
    }
}

/// Capped cylinder, axis-aligned along Y, centered at origin. Unused by the demo scene
/// — the pillar uses `RoundedCylinder` instead, for the same reason the ground uses
/// `RoundedBox3` over `Box3` (see that type's doc comment): a sharp cylinder has two
/// hard edges (top and bottom rim, where the flat cap meets the curved side), each a
/// genuine gradient discontinuity that destabilizes `bake::principal_curvature_frame`'s
/// finite-difference shape operator right at the edge. Unlike the box case (which
/// mainly showed up as a sampling *gap*), an unstable shape operator's noisy
/// eigenvector direction feeds directly into each nearby splat's oriented anisotropic
/// axes (see `bake::rotation_from_tangent_frame`) — so instead of a gap, this showed up
/// as visibly mis-oriented, jagged "winged" splats sticking out of the rim at
/// inconsistent angles. Kept here as part of the primitive library for completeness.
#[allow(dead_code)]
pub struct Cylinder {
    pub radius: f32,
    pub half_height: f32,
}

impl Sdf for Cylinder {
    fn distance(&self, p: Vec3) -> f32 {
        let d = bevy::math::Vec2::new(
            bevy::math::Vec2::new(p.x, p.z).length() - self.radius,
            p.y.abs() - self.half_height,
        );
        d.x.max(d.y).min(0.0) + d.max(bevy::math::Vec2::ZERO).length()
    }

    /// No `TAG_LEAF_CYLINDER` exists in the raymarcher (this primitive is unused by the
    /// demo scene, see this type's doc comment) — reuses `RoundedCylinder` at
    /// `edge_radius = 0.0`, which the shader's `sdf_rounded_cylinder` formula reduces
    /// to exactly `Cylinder`'s own formula for.
    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::RoundedCylinder,
            params: [self.radius, self.half_height, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    }
}

/// A capped cylinder with beveled (rounded) top/bottom edges: `Cylinder`'s exact
/// distance formula, but evaluated against a radius/half-height shrunk by
/// `edge_radius` and then offset back out by the same amount — the same "round trick"
/// `RoundedBox3` uses (see its doc comment), applied to the cylinder's 2D
/// (radial-distance, axial-distance) cross-section instead of a 3D box's three axes.
/// This gives the sampler a continuous, well-defined normal at the rim instead of a
/// sharp discontinuity — see `Cylinder`'s doc comment for why that discontinuity
/// specifically causes mis-oriented "winged" splats near the edge, not just a coverage
/// gap.
pub struct RoundedCylinder {
    pub radius: f32,
    pub half_height: f32,
    pub edge_radius: f32,
}

impl Sdf for RoundedCylinder {
    fn distance(&self, p: Vec3) -> f32 {
        let d = bevy::math::Vec2::new(
            bevy::math::Vec2::new(p.x, p.z).length() - self.radius + self.edge_radius,
            p.y.abs() - self.half_height + self.edge_radius,
        );
        d.x.max(d.y).min(0.0) + d.max(bevy::math::Vec2::ZERO).length() - self.edge_radius
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::RoundedCylinder,
            params: [
                self.radius,
                self.half_height,
                self.edge_radius,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        }
    }
}

/// A sphere swept along a line segment: the Minkowski sum of a sphere of radius
/// `radius` with a line from `a` to `b`. Parametrised by the two endpoints and a
/// uniform radius — the capsule's long axis is determined entirely by `a` and `b`,
/// not by the entity's transform (the GPU record stores endpoints in world-space
/// `param_a..param_d` so the shader skips the entity's local-to-world rotation;
/// see `GpuPrimitive`'s param-by-kind encoding).
pub struct Capsule {
    pub a: Vec3,
    pub b: Vec3,
    pub radius: f32,
}

impl Sdf for Capsule {
    fn distance(&self, p: Vec3) -> f32 {
        let ab = self.b - self.a;
        let ap = p - self.a;
        let t = (ap.dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
        let closest = self.a + ab * t;
        (p - closest).length() - self.radius
    }

    /// Endpoints stored in `param_a..param_f` (world-space, pre-transform); shader
    /// sets `rotation_is_identity = 1` so the entity's local-to-world matrix is
    /// skipped.
    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::Capsule,
            params: [
                self.a.x,
                self.a.y,
                self.a.z,
                self.b.x,
                self.b.y,
                self.b.z,
                self.radius,
                0.0,
            ],
        }
    }
}

/// A cone with spherical caps at both ends, parametrised by two endpoints with
/// independent radii (`r0` at `a`, `r1` at `b`). The body is a linear taper
/// between the two end spheres; the signed distance is exact (no bounding-volume
/// approximation). When `r0 == r1` this degenerates to a `Capsule`.
pub struct RoundedCone {
    pub a: Vec3,
    pub b: Vec3,
    pub r0: f32,
    pub r1: f32,
}

impl Sdf for RoundedCone {
    fn distance(&self, p: Vec3) -> f32 {
        let ba = self.b - self.a;
        let pa = p - self.a;
        let m0 = ba.dot(ba);
        let m1 = ba.dot(pa);
        let m2 = pa.dot(pa);

        let d = m0 - m1;
        let e = m1 - m2;
        let f = m2 + m0 * m0 - 2.0 * m1;
        let g = d * d * m0;
        let h = m0 * (m0 - d);
        let clamped = (d * e * m0 - f * h).max(0.0);
        let t = m0 * (f * d - e * clamped) / (g + h * clamped) - m1;

        let t_clamped = t.clamp(0.0, m0);
        let q = (self.a + ba * t_clamped / m0 - p).length()
            - self.r0
            - (self.r1 - self.r0) * t_clamped / m0;
        q.max(pa.length() - self.r0)
            .max((p - self.b).length() - self.r1)
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::RoundedCone,
            params: [
                self.a.x, self.a.y, self.a.z, self.b.x, self.b.y, self.b.z, self.r0, self.r1,
            ],
        }
    }
}

/// An ellipsoid with three independent radii, axis-aligned in local space. Unlike
/// `Capsule`/`RoundedCone` (whose endpoints are stored in the param slots and skip
/// the entity transform), the entity's transform is applied normally — the rotation
/// rotates the query point into the ellipsoid's local frame, and a non-uniform
/// scale on the entity can additionally stretch it. Only the three radii occupy
/// `param_a..param_c`.
pub struct Ellipsoid {
    pub radii: Vec3,
}

impl Sdf for Ellipsoid {
    fn distance(&self, p: Vec3) -> f32 {
        let k0 = (p / self.radii).length();
        let k1 = (p / (self.radii * self.radii)).length();
        k0 * (k0 - 1.0) / k1
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::Ellipsoid,
            params: [
                self.radii.x,
                self.radii.y,
                self.radii.z,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        }
    }
}

/// A box-shaped frame (hollow box): the boolean difference between a solid box
/// and a slightly smaller interior box, leaving walls of uniform thickness
/// `wall_thickness`. All six inner faces are at exact right angles — useful for
/// architectural detailing, shelving, or any case where a thin-walled rectangular
/// enclosure is needed without the curvature of `RoundedBox3`.
pub struct BoxFrame {
    pub half_extents: Vec3,
    pub wall_thickness: f32,
}

impl Sdf for BoxFrame {
    fn distance(&self, p: Vec3) -> f32 {
        let q = p.abs() - self.half_extents;
        let outer = q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0);
        let inner_q = q + Vec3::splat(self.wall_thickness);
        let inner =
            inner_q.max(Vec3::ZERO).length() + inner_q.x.max(inner_q.y.max(inner_q.z)).min(0.0);
        outer.max(-inner)
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::BoxFrame,
            params: [
                self.half_extents.x,
                self.half_extents.y,
                self.half_extents.z,
                self.wall_thickness,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        }
    }
}

/// A hexagonal prism (hex-nut shape): a regular hexagon cross-section in the XZ
/// plane, extruded to `±half_height` along Y. The hexagon is defined by a single
/// circumradius `radius` (center to vertex). Useful for bolt heads, pillars with
/// faceted sides, or any architectural element where a six-sided profile is
/// preferred over a cylinder's smooth round.
pub struct HexPrism {
    pub radius: f32,
    pub half_height: f32,
}

impl Sdf for HexPrism {
    fn distance(&self, p: Vec3) -> f32 {
        let q = p.abs();
        let k = 0.866025404f32;
        let hex_d =
            q.x.max((0.5 * q.x + k * q.z).abs())
                .max((0.5 * q.x - k * q.z).abs())
                - self.radius;
        let axial_d = q.y - self.half_height;
        hex_d.max(axial_d)
    }

    fn gpu_record(&self) -> GpuPrimitive {
        GpuPrimitive {
            kind: GpuShapeKind::HexPrism,
            params: [self.radius, self.half_height, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
    }
}
