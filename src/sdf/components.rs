//! ECS-authored SDF primitives: ordinary Bevy entities with `Transform` + `Shape`
//! (+ optional `Blend`), assembled into the plain `Node` tree `crate::bake` already
//! knows how to bake — see `assembly::assemble_scene`. This is the authoring layer
//! `sdf::world`'s hand-written `Node`-builder functions predate; the payoff is that
//! moving/animating a primitive becomes literal `Transform` mutation via ordinary
//! Bevy systems (see `main.rs`'s animation system), reusing `GlobalTransform`
//! propagation instead of a bespoke per-primitive animation mechanism.

use bevy::prelude::*;
use bevy::shader::Shader;

use super::primitives::{
    BoxFrame, Capsule, Ellipsoid, HexPrism, RoundedBox3, RoundedCone, RoundedCylinder, Sphere,
};

/// One primitive's shape and size, mirroring `crate::sdf::primitives`' concrete
/// `Sdf` impls. An enum (not `Box<dyn Sdf>`) so it's a plain, `Copy`, queryable
/// component — `assembly::assemble_scene` converts each variant to the matching
/// `Node::leaf_transformed(..)` call, which is where the actual `Box<dyn Sdf>` gets
/// built (see `Node::Leaf`'s doc comment for why baking itself still uses trait
/// objects: it's a demo, not a hot per-frame loop, so the small dynamic-dispatch
/// convenience is worth it there — this component just doesn't need it).
#[derive(Component, Clone, Copy, Debug)]
pub enum Shape {
    Sphere {
        radius: f32,
    },
    RoundedBox {
        half_extents: Vec3,
        corner_radius: f32,
    },
    RoundedCylinder {
        radius: f32,
        half_height: f32,
        edge_radius: f32,
    },
    Capsule {
        a: Vec3,
        b: Vec3,
        radius: f32,
    },
    RoundedCone {
        a: Vec3,
        b: Vec3,
        r0: f32,
        r1: f32,
    },
    Ellipsoid {
        radii: Vec3,
    },
    BoxFrame {
        half_extents: Vec3,
        wall_thickness: f32,
    },
    HexPrism {
        radius: f32,
        half_height: f32,
    },
}

/// How this entity's shape is combined with its sibling/parent shapes during
/// `assembly::assemble_scene` — replaces the old `Blend(f32)` component with a
/// full CSG-operation enum, so ECS-authored scenes can express subtraction
/// (previously only available via one-off hand-written `smooth_subtract` calls in
/// `world::assemble_infinite_scene`). Entities without a `BlendMode` fall back to
/// `assembly::DEFAULT_BLEND` via the legacy `Blend(f32)` component, preserving
/// backward compatibility with existing scene code.
#[derive(Component, Clone, Copy, Debug)]
pub enum BlendMode {
    /// Hard union: `min(a, b)` — a sharp boolean seam, gradient-discontinuous at
    /// the boundary (see `RoundedBox3`'s doc comment on why that matters for
    /// curvature estimation). Kept for completeness; every join in this demo uses
    /// `SmoothUnion` instead.
    #[allow(dead_code)]
    Union,
    /// Smooth union (polynomial smin, `k` = blend radius): the default CSG join —
    /// see `combination-operators.md`. `k` controls how far the blend extends;
    /// zero degenerates to a hard `Union`.
    SmoothUnion(f32),
    /// Smooth subtraction (polynomial smax, `k` = blend radius): `smax(a, -b, k) =
    /// -smin(-a, b, k)`, the same interpolation trick `SmoothUnion` uses,
    /// substituted into the max-based subtraction formula — see
    /// `combination-operators.md`'s "Smooth intersection and smooth subtraction"
    /// section. The child entity is subtracted from the accumulated parent/sibling
    /// result, with a smooth blend seam of radius `k`.
    Subtract(f32),
    /// Hard intersection: `max(a, b)` (stage-1 op-set completion).
    #[allow(dead_code)]
    Intersect,
    /// Smooth intersection (polynomial smax, `k` = blend radius).
    #[allow(dead_code)]
    SmoothIntersect(f32),
}

impl Default for BlendMode {
    fn default() -> Self {
        Self::SmoothUnion(super::assembly::DEFAULT_BLEND)
    }
}

/// Legacy smooth-union blend radius: entities with `Blend(f32)` but no `BlendMode`
/// still work — `assembly` reads `BlendMode` first, falls back to `Blend` if absent.
/// Kept for backward compatibility with existing scene code that doesn't need
/// subtraction.
#[derive(Component, Clone, Copy, Debug)]
pub struct Blend(pub f32);

/// This entity's physically based material — mirrors `Shape`'s "plain, queryable,
/// authored on the same entity" shape: `base_color`/`metallic`/`roughness` are the
/// glTF/Disney metallic-roughness parameterization (see docs/knowledge/sdf-3d/
/// materials-and-texturing/pbr-shading-model.md), carried per-primitive through
/// `assembly`/`raymarch::flatten` into the GPU `PrimitiveRecordCpu` exactly like
/// `Shape`'s own params already are. An entity with a `ProceduralPattern` component
/// uses this `Material` as that pattern's "material A" input rather than shading with
/// it directly — see `ProceduralPattern`'s doc comment.
///
/// Named `MaterialLegacy` (not `Material`) because `src/hybrid` — the
/// fresh-start renderer — has its own separate `hybrid::material::Material`
/// with a fuller PBR field set (reflectance, emissive) that this legacy
/// pipeline (`hybrid_legacy`, `raymarch`, `sdf::world`) has
/// no shading path for. Keeps the two renderers' material models
/// independently evolvable rather than threading new fields through a
/// struct half the call sites don't need — see `src/hybrid/material.rs`'s
/// own doc comment for the new type.
#[derive(Component, Clone, Copy, Debug)]
pub struct MaterialLegacy {
    pub base_color: Vec3,
    pub metallic: f32,
    pub roughness: f32,
}

impl MaterialLegacy {
    pub const fn new(base_color: Vec3, metallic: f32, roughness: f32) -> Self {
        Self {
            base_color,
            metallic,
            roughness,
        }
    }
}

/// Selects a dynamically dispatched, user-authorable WGSL pattern function to resolve
/// this entity's material instead of a flat `Material`, evaluated per-pixel at the
/// world-space hit point — the SDF-native, UV-less texturing technique (see
/// docs/knowledge/sdf-3d/materials-and-texturing/uv-less-texturing.md): no mesh, no
/// UVs, so any procedural pattern works directly from the hit point the raymarcher
/// already has for free.
///
/// `shader` must point at a `.wgsl` asset that declares `#define_import_path
/// <import_path>` (matching this component's own `import_path` field exactly — the
/// pipeline has no way to discover it automatically, since a `Handle<Shader>` alone
/// doesn't expose the shader's logical import path before it's loaded) and exports a
/// function with the fixed signature `fn evaluate_pattern(p: vec3<f32>, mat_a:
/// Material, mat_b: Material, params: vec4<f32>) -> Material` — see
/// `assets/shaders/patterns/checkerboard.wgsl` for the reference implementation this
/// repo's own checkerboard ground uses, authored with no special access: any
/// user-supplied shader following the same contract plugs in identically.
///
/// `mat_a` is this entity's own `MaterialLegacy` component (required alongside
/// `ProceduralPattern` — an entity with `ProceduralPattern` but no `MaterialLegacy` has
/// no "material A" to hand the pattern function, see `assembly`'s flattening of the two
/// components together), `mat_b` is this component's own second material, and
/// `params` is a generic per-pattern tunable (the built-in checkerboard uses
/// `params.x` as cell size; other patterns are free to use all four fields however
/// they like).
///
/// Every distinct `import_path` actually present in a scene becomes part of
/// `raymarch::pipeline::RaymarchPipelineKey` (see that type's doc comment) — the
/// pipeline generates and compiles a dispatcher WGSL module `#import`ing each active
/// pattern by its logical path, so adding a new pattern to the scene triggers a new
/// pipeline specialization/compile, not a `raymarch.wgsl` source change.
#[derive(Component, Clone)]
pub struct ProceduralPattern {
    pub shader: Handle<Shader>,
    pub import_path: String,
    pub material_b: MaterialLegacy,
    pub params: Vec4,
}

/// Marks an entity's `Shape` subtree as rigidly, continuously animated
/// (currently: pure rotation about a fixed world-space pivot),
/// and gives it a stable numeric ID (`Splat::anim_group` tags every splat baked from
/// this subtree with the same ID).
///
/// Per docs/knowledge/sdf-3dgs-bevy-integration/live-editing/deformation-vs-rebake.md's
/// already-decided rule, a pure rigid `Transform` change needs no re-bake at all — only
/// reapplication of the transform to already-baked splats. `AnimGroup` is what makes
/// that possible: `assembly::assemble_scene_split` bakes this entity's subtree in its
/// **rest pose** (the entity's own authored rotation stripped back to identity, but its
/// translation kept as the group's pivot — see that function's doc comment), and the
/// render pipeline reapplies the entity's *live* rotation every frame on the GPU (see
/// `splat::gpu::AnimGroupTransform` and `splat.wgsl`'s vertex shader), instead of the
/// old approach of re-baking the whole chunk from scratch every time the `Transform`
/// changed (`streaming::REANIMATE_INTERVAL_SECS`, now removed).
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct AnimGroup(pub u32);

impl Shape {
    pub fn sdf(self) -> Box<dyn super::primitives::Sdf> {
        match self {
            Shape::Sphere { radius } => Box::new(Sphere { radius }),
            Shape::RoundedBox {
                half_extents,
                corner_radius,
            } => Box::new(RoundedBox3 {
                half_extents,
                corner_radius,
            }),
            Shape::RoundedCylinder {
                radius,
                half_height,
                edge_radius,
            } => Box::new(RoundedCylinder {
                radius,
                half_height,
                edge_radius,
            }),
            Shape::Capsule { a, b, radius } => Box::new(Capsule { a, b, radius }),
            Shape::RoundedCone { a, b, r0, r1 } => Box::new(RoundedCone { a, b, r0, r1 }),
            Shape::Ellipsoid { radii } => Box::new(Ellipsoid { radii }),
            Shape::BoxFrame {
                half_extents,
                wall_thickness,
            } => Box::new(BoxFrame {
                half_extents,
                wall_thickness,
            }),
            Shape::HexPrism {
                radius,
                half_height,
            } => Box::new(HexPrism {
                radius,
                half_height,
            }),
        }
    }
}
