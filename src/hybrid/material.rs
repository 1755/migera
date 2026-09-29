//! The fresh-start `src/hybrid` renderer's own PBR material — deliberately
//! separate from `sdf::components::MaterialLegacy` (see that type's doc
//! comment for why: `src/hybrid` and `src/hybrid_legacy` are independently
//! evolvable, not sharing types, per this project's rewrite-from-scratch
//! convention).

use bevy::math::Vec3;
use bevy::prelude::Component;

/// Metallic-roughness PBR material for a `src/hybrid`-rendered SDF primitive
/// (glTF/Disney parameterization — see docs/knowledge/sdf-3d/
/// materials-and-texturing/pbr-shading-model.md). `reflectance` is a `[0,1]`
/// dial on a dielectric's F0 (Fresnel reflectance at normal incidence),
/// remapped the same way `bevy_pbr` does (`F0 = 0.16 * reflectance^2`) —
/// only matters when `metallic < 1`; `0.5` reproduces the conventional 4%
/// dielectric default. `emissive` is added on top of lit shading, unaffected
/// by any light in the scene. `transmission` is a `[0,1]` dial on how much
/// of the (1 - Fresnel-reflected) energy passes THROUGH the surface as
/// refraction rather than being absorbed/diffused — `0.0` (the default)
/// reproduces every pre-existing opaque material exactly; only meaningful
/// when `metallic < 1` (a transmissive metal has no physical meaning, same
/// restriction glTF's own `KHR_materials_transmission` extension states).
/// `ior` is the medium's index of refraction (Snell's law), `1.5` (typical
/// glass) by default — unused when `transmission == 0.0`.
#[derive(Component, Clone, Copy, Debug)]
pub struct Material {
    pub base_color: Vec3,
    pub metallic: f32,
    pub roughness: f32,
    pub reflectance: f32,
    pub emissive: Vec3,
    pub transmission: f32,
    pub ior: f32,
}

impl Material {
    pub const fn new(base_color: Vec3, metallic: f32, roughness: f32) -> Self {
        Self { base_color, metallic, roughness, reflectance: 0.5, emissive: Vec3::ZERO, transmission: 0.0, ior: 1.5 }
    }

    pub const fn with_reflectance(mut self, reflectance: f32) -> Self {
        self.reflectance = reflectance;
        self
    }

    pub const fn with_emissive(mut self, emissive: Vec3) -> Self {
        self.emissive = emissive;
        self
    }

    pub const fn with_transmission(mut self, transmission: f32) -> Self {
        self.transmission = transmission;
        self
    }

    pub const fn with_ior(mut self, ior: f32) -> Self {
        self.ior = ior;
        self
    }
}
