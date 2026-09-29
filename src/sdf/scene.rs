//! A hand-authored SDF scene: a tree of primitives combined with CSG/smooth-blend
//! operators, per docs/knowledge/sdf-3d/primitives-and-operators/combination-operators.md.
//!
//! This is intentionally a plain Rust closure-tree, not a data-driven expression format —
//! see the module doc on `crate::sdf` for why (matches the "procedural" representation
//! family from docs/knowledge/sdf-3d/fundamentals/sdf-representations.md).

use bevy::math::{Quat, Vec3, Vec4};

use super::primitives::Sdf;

/// A leaf's material, mirroring `sdf::components::MaterialLegacy`/`ProceduralPattern` in the
/// plain (non-ECS) scene-tree layer — `assembly` reads each entity's `MaterialLegacy`/
/// `ProceduralPattern` components and folds them into this enum when building a
/// `Node::Leaf`, so material data rides through the CSG tree (and, downstream,
/// `raymarch::flatten`) alongside the geometry it was authored on, rather than as a
/// separate parallel structure that could fall out of sync with the tree's own
/// leaf order.
#[derive(Clone)]
pub enum LeafMaterial {
    /// A flat physically based material — see `sdf::components::MaterialLegacy`.
    Solid {
        base_color: Vec3,
        metallic: f32,
        roughness: f32,
    },
    /// A dynamically dispatched procedural pattern — see
    /// `sdf::components::ProceduralPattern`'s doc comment for the pattern-authoring
    /// contract (`evaluate_pattern(p, mat_a, mat_b, params) -> Material`) this
    /// refers to. Deliberately doesn't carry the pattern's `Handle<Shader>` (unlike
    /// `sdf::components::ProceduralPattern`, which does) — nothing downstream of
    /// `assembly::resolve_leaf_material` (which builds this variant) ever needs it
    /// again: `raymarch::flatten` only threads `import_path` through to
    /// `PrimitiveRecordCpu`'s parallel pattern-path list, and `raymarch::extract`'s
    /// `PatternRegistry` (which does need to hold the handle alive, see
    /// `PatternRegistryEntry`'s doc comment) is built directly from the ECS
    /// `ProceduralPattern` components, not from this type.
    Pattern {
        import_path: String,
        material_a: (Vec3, f32, f32),
        material_b: (Vec3, f32, f32),
        params: Vec4,
    },
}

/// Rigid transform (translation + rotation, no scale) placing a primitive's local
/// space into its parent's — deliberately not a full `Transform`/`Mat4`: SDF
/// primitives already encode their own size via shape fields (e.g. `Sphere::radius`),
/// so non-uniform scale would need to warp the *distance field* itself (no longer a
/// true SDF in general — see docs/knowledge/sdf-3d/fundamentals/what-is-an-sdf.md's
/// Eikonal/exact-vs-bound distinction), not just the query point. Uniform scale would
/// be safe (a scaled exact SDF, divided back down, stays exact) but isn't needed by
/// anything in this demo yet — left out until something actually needs it, per this
/// project's "don't build for hypothetical requirements" convention.
#[derive(Clone, Copy)]
pub struct Isometry {
    pub translation: Vec3,
    pub rotation: Quat,
}

impl Isometry {
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            translation,
            rotation: Quat::IDENTITY,
        }
    }

    /// Transforms a world/parent-space point into this isometry's local space —
    /// inverse of the transform this `Isometry` itself represents (translate-then-
    /// rotate from local to parent, so the inverse is un-translate-then-un-rotate).
    /// Skips the quaternion multiply entirely for an identity rotation (the common
    /// case — most primitives in this demo don't rotate): `distance()` runs this on
    /// the order of 10^7-10^8 times per bake (see `bake::sample_surface`'s doc
    /// comment), so the ~10-flop `Quat` multiply this would otherwise pay on every
    /// single call, even for non-rotated leaves, was a measured ~2x regression
    /// (1.36s -> 2.9s per chunk bake) when this `Isometry` extension first replaced
    /// the old bare-`Vec3`-offset leaf transform.
    fn inverse_transform_point(self, p: Vec3) -> Vec3 {
        let local = p - self.translation;
        if self.rotation == Quat::IDENTITY {
            local
        } else {
            self.rotation.inverse() * local
        }
    }
}

/// A node in the CSG tree. Boxed trait objects: this is a demo, not a hot inner loop
/// (baking runs once, not per-frame — see docs/knowledge/sdf-3dgs-bevy-integration/
/// baking-pipeline/sdf-to-splat-baking.md on why bake cost doesn't need to be
/// per-frame-cheap).
pub enum Node {
    Leaf {
        shape: Box<dyn Sdf>,
        /// Local transform applied to the query point before evaluating `shape`.
        transform: Isometry,
        /// This leaf's material, if any was authored on the ECS entity it came from
        /// (see `LeafMaterial`) — `None` for leaves built directly in Rust (this
        /// project's own hand-written `sdf::world` helper functions, and the pillar's
        /// one-off bite-sphere subtraction in `world::assemble_infinite_scene`) that
        /// predate per-primitive material authoring; `raymarch::flatten` falls back
        /// to a default material for those.
        material: Option<LeafMaterial>,
    },
    /// Hard union: min(a, b). See combination-operators.md. Unused by the demo scene
    /// — every CSG join uses `SmoothUnion` instead, for the same reason `Subtract` is
    /// unused in favor of `SmoothSubtract` (see that variant's doc comment). Kept for
    /// completeness / cases where a genuinely sharp union seam is wanted.
    #[allow(dead_code)]
    Union(Box<Node>, Box<Node>),
    /// Smooth union (polynomial smin, k = blend radius). See combination-operators.md.
    SmoothUnion(Box<Node>, Box<Node>, f32),
    /// Hard intersection: max(a, b). New in the stage-1 op-set completion
    /// (see docs/plan/hybrid-renderer-roadmap.md); unused by existing scenes.
    #[allow(dead_code)]
    Intersect(Box<Node>, Box<Node>),
    /// Smooth intersection (polynomial smax): `-smin(-a, -b, k)`. New alongside
    /// `Intersect`.
    #[allow(dead_code)]
    SmoothIntersect(Box<Node>, Box<Node>, f32),
    /// Hard subtraction: max(a, -b) — "a minus b". Unused by the demo scene — every
    /// CSG boundary uses a smooth variant instead (see `SmoothSubtract`), since a hard
    /// boolean seam is exactly as much a gradient discontinuity as a sharp primitive
    /// corner is, and causes the same class of splat-baking artifact (see
    /// `crate::sdf::primitives::RoundedBox3`'s doc comment for the mechanism — it
    /// applies identically at a CSG seam, not just a single primitive's own edge).
    /// Kept for completeness / cases where a genuinely sharp boolean seam is wanted.
    #[allow(dead_code)]
    Subtract(Box<Node>, Box<Node>),
    /// Smooth subtraction (polynomial smax, k = blend radius): `smax(a, -b, k) =
    /// -smin(-a, b, k)`, the same interpolation trick `SmoothUnion` uses, substituted
    /// into the max-based subtraction formula — see combination-operators.md's
    /// "Smooth intersection and smooth subtraction" section.
    SmoothSubtract(Box<Node>, Box<Node>, f32),
    /// Infinite domain repetition on the XZ plane (Y left unrepeated — this demo tiles
    /// a ground-level cluster across a flat grid, not a 3D lattice): remaps the query
    /// point's X/Z into `[-period/2, period/2)` before evaluating the child, per
    /// docs/knowledge/sdf-3d/primitives-and-operators/domain-operations.md's "Domain
    /// repetition (infinite tiling)" section. Constant-time regardless of how far `p`
    /// is from the origin — the whole infinite field of copies is one child evaluation
    /// in remapped space, not one evaluation per copy. Correctness caveat from that doc
    /// applies: `period` must be comfortably larger than the child's own bounds, or
    /// neighboring copies can reach across a cell boundary and this single-cell remap
    /// returns a too-large (wrong) distance there — see `world::tile_cluster`'s doc
    /// comment for how this demo's period is sized to satisfy that.
    Repeat(Box<Node>, f32),
}

impl Node {
    pub fn leaf(shape: impl Sdf + 'static, offset: Vec3) -> Self {
        Node::Leaf {
            shape: Box::new(shape),
            transform: Isometry::from_translation(offset),
            material: None,
        }
    }

    /// For a shape that's already boxed — the shape a `sdf::components::Shape`
    /// component's `sdf()` produces is already a `Box<dyn Sdf>` (an enum match
    /// returning one of several concrete owned types), so this avoids `leaf`'s
    /// generic bound (which `Box<dyn Sdf>` itself doesn't satisfy — a `Box<dyn Sdf>`
    /// is not an `impl Sdf`, since `Sdf` isn't blanket-implemented for boxes) forcing
    /// a pointless double-box at call sites that already have one, and takes a full
    /// `Isometry` (translation + rotation) rather than `leaf`'s translation-only
    /// `Vec3`, for ECS-authored primitives whose `Transform` may be rotated.
    pub fn leaf_boxed(shape: Box<dyn Sdf>, transform: Isometry) -> Self {
        Node::Leaf {
            shape,
            transform,
            material: None,
        }
    }

    /// Attaches (or clears, given `None`) a `LeafMaterial` to a `Node::Leaf` built via
    /// `leaf`/`leaf_boxed` — a no-op on any other `Node` variant, so `assembly` can
    /// call this unconditionally right after building a leaf without needing to
    /// special-case whether the entity actually authored a `sdf::components::
    /// Material`/`ProceduralPattern`. Material data rides through the CSG tree from
    /// the same call site the geometry itself is built at.
    pub fn with_material(self, material: Option<LeafMaterial>) -> Self {
        match self {
            Node::Leaf {
                shape, transform, ..
            } => Node::Leaf {
                shape,
                transform,
                material,
            },
            other => other,
        }
    }

    #[allow(dead_code)]
    pub fn union(self, other: Node) -> Self {
        Node::Union(Box::new(self), Box::new(other))
    }

    pub fn smooth_union(self, other: Node, k: f32) -> Self {
        Node::SmoothUnion(Box::new(self), Box::new(other), k)
    }

    #[allow(dead_code)]
    pub fn intersect(self, other: Node) -> Self {
        Node::Intersect(Box::new(self), Box::new(other))
    }

    #[allow(dead_code)]
    pub fn smooth_intersect(self, other: Node, k: f32) -> Self {
        Node::SmoothIntersect(Box::new(self), Box::new(other), k)
    }

    #[allow(dead_code)]
    pub fn subtract(self, other: Node) -> Self {
        Node::Subtract(Box::new(self), Box::new(other))
    }

    pub fn smooth_subtract(self, other: Node, k: f32) -> Self {
        Node::SmoothSubtract(Box::new(self), Box::new(other), k)
    }

    pub fn repeat(self, period: f32) -> Self {
        Node::Repeat(Box::new(self), period)
    }

    pub fn distance(&self, p: Vec3) -> f32 {
        match self {
            Node::Leaf {
                shape, transform, ..
            } => shape.distance(transform.inverse_transform_point(p)),
            Node::Union(a, b) => a.distance(p).min(b.distance(p)),
            Node::SmoothUnion(a, b, k) => smin(a.distance(p), b.distance(p), *k),
            Node::Intersect(a, b) => a.distance(p).max(b.distance(p)),
            Node::SmoothIntersect(a, b, k) => smax(a.distance(p), b.distance(p), *k),
            Node::Subtract(a, b) => a.distance(p).max(-b.distance(p)),
            Node::SmoothSubtract(a, b, k) => smax(a.distance(p), -b.distance(p), *k),
            Node::Repeat(a, period) => a.distance(repeat_xz(p, *period)),
        }
    }
}

/// Polynomial smooth minimum, per docs/knowledge/sdf-3d/primitives-and-operators/
/// combination-operators.md.
fn smin(a: f32, b: f32, k: f32) -> f32 {
    if k <= 0.0 {
        return a.min(b);
    }
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    lerp(b, a, h) - k * h * (1.0 - h)
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Polynomial smooth maximum: `smax(a, b, k) = -smin(-a, -b, k)`, per
/// combination-operators.md's "Smooth intersection and smooth subtraction" section —
/// the same interpolation trick `smin` uses, substituted into the max-based
/// intersection/subtraction formulas instead of the min-based union formula.
fn smax(a: f32, b: f32, k: f32) -> f32 {
    -smin(-a, -b, k)
}

/// Remaps `p`'s X/Z into `[-period/2, period/2)`, Y unchanged — the domain-repetition
/// remap per domain-operations.md, using `rem_euclid` (not plain `%`) so the remap is
/// correct for negative coordinates too (Rust's `%` is truncating, not floored, and
/// would otherwise produce a discontinuous jump at `x=0`/`z=0`).
fn repeat_xz(p: Vec3, period: f32) -> Vec3 {
    let half = period * 0.5;
    Vec3::new(
        (p.x + half).rem_euclid(period) - half,
        p.y,
        (p.z + half).rem_euclid(period) - half,
    )
}
