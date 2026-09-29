//! CPU reference for the first real trace step: a BVH-accelerated primary
//! ray, sphere-marched against `RoundedBox` scene geometry, resolving to
//! either a hit (which object, at what distance, what flat color) or a
//! miss. No lighting/shadows/reflections — this step's entire job is
//! "does the ray hit anything, and what flat color does it report,"
//! proven correct here before any of that math is ported to WGSL.
//!
//! Follows `mod.rs`'s doc comment's methodology: narrow, numerically
//! well-defined math (marching, BVH traversal, hit resolution) ported to
//! plain Rust, pinned with `#[cfg(test)]` cases, proven with `cargo test`
//! before porting to WGSL by hand. Keep the CPU and WGSL implementations
//! in sync by hand — there is no shared-source mechanism between them.
//!
//! Marching approach, and why: `hybrid_legacy` never resolves a box hit
//! via exact analytic ray-box intersection anywhere — box hits are always
//! resolved by sphere-marching `sd_rounded_box`'s SDF distance formula,
//! after transforming the ray into the object's local (unrotated) space.
//! An exact-analytic-intersection tier was tried once elsewhere in this
//! project and removed after profiling showed 5-6x the GPU cost of SDF
//! marching (see `docs/knowledge/analytic-intersections/INDEX.md`) — so
//! marching, not exact intersection, is this project's established
//! approach for resolving a candidate's exact hit, and this module
//! follows it rather than reintroducing exact intersection.
//!
//! BVH acceleration: before marching anything, the ray is tested against
//! the whole scene's root AABB (and each internal node descended into)
//! via the branchless slab method — a ray that misses a node's box skips
//! its entire subtree, exactly the property `docs/knowledge/
//! aabb-acceleration/ray-aabb-slab-test.md` documents. Only leaves whose
//! AABB the ray actually hits get marched at all.

use bevy::math::{Quat, Vec3};
use bevy::prelude::Entity;

use crate::hybrid::bvh::{Bvh, LEAF_SENTINEL};
use crate::hybrid::material::Material;
use crate::prim::Aabb;
use crate::sdf::components::Shape;

/// Everything the CPU tracer needs for one object: its BVH-culling AABB
/// (via `entity`, matched against `Bvh`'s leaves), its shape/material for
/// exact marching, and its world transform (translation + rotation only —
/// this renderer's shapes don't support non-uniform scale, matching
/// `scene::world_aabb`'s own scale-discarding convention).
#[derive(Clone, Copy, Debug)]
pub struct TraceObject {
    pub entity: Entity,
    pub shape: Shape,
    pub translation: Vec3,
    pub rotation: Quat,
    pub material: Material,
}

/// A resolved primary-ray hit: which object, how far along the ray, its
/// world-space surface normal (for shading — see `local_normal`), and its
/// material's base color (albedo; shading multiplies this by the lit
/// radiance `shade` computes, it is not the final displayed color by
/// itself once lighting exists).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub entity: Entity,
    pub t: f32,
    pub world_normal: Vec3,
    pub color: Vec3,
}

/// Marching tolerances, kept as named constants (not magic numbers)
/// since `hybrid_legacy`'s own bug history (see `bvh.rs`'s module doc)
/// showed hardcoded reach/margin constants are exactly where subtle bugs
/// hide — these are small enough to name once, here, and reused by every
/// call site in this module rather than repeated as literals.
pub(crate) const MAX_MARCH_STEPS: u32 = 128;
pub(crate) const HIT_EPSILON: f32 = 1e-4;

/// Which physical light behavior a `Light` models — mirrors `hybrid_legacy`'s
/// `LIGHT_KIND_*` constants (`assets/shaders/hybrid_legacy_trace.wgsl`),
/// the proven precedent this module's `shade` function ports its
/// attenuation/falloff formula from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LightKind {
    /// Parallel rays from infinitely far away (the sun) — no position, no
    /// distance falloff, direction only.
    Directional,
    /// Radiates in all directions from a point in space, with inverse-
    /// square-ish distance falloff (a lamp/bulb).
    Point,
    /// Like `Point`, but also cut off outside a cone around its own
    /// forward direction, with a smooth inner/outer-angle falloff at the
    /// cone edge (a projector/spotlight).
    Spot,
}

/// One light source, in world space — everything `shade` needs to
/// evaluate this light's contribution at a hit point, including its own
/// soft-shadow softness. Mirrors `hybrid_legacy`'s `LightGpu` field set
/// (`src/raymarch/pipeline.rs`).
#[derive(Clone, Copy, Debug)]
pub struct Light {
    pub kind: LightKind,
    pub color: Vec3,
    /// `Directional`: the direction rays travel (already normalized,
    /// e.g. `transform.forward()`). `Point`/`Spot`: the light's world
    /// position.
    pub direction_or_position: Vec3,
    /// `Directional`: illuminance (lux). `Point`/`Spot`: raw luminous
    /// intensity in lumens per steradian — NOT yet divided by `4*PI`;
    /// see `crate::hybrid::extract`'s light-extraction doc comment for
    /// why that conversion happens at extraction time, matching
    /// `bevy_pbr`'s own internal formula, not here.
    pub intensity: f32,
    /// `Spot` only: the cone's own forward direction (already
    /// normalized). Unused by `Directional`/`Point`.
    pub spot_direction: Vec3,
    /// `Point`/`Spot` only: distance beyond which this light contributes
    /// nothing, part of the smooth falloff curve (see `shade`). Unused by
    /// `Directional`.
    pub range: f32,
    /// `Spot` only: half-angle (radians) of the inner cone (full
    /// intensity) and outer cone (falls to zero at the edge). Unused by
    /// `Directional`/`Point`.
    pub inner_angle: f32,
    pub outer_angle: f32,
    /// Soft-shadow penumbra hardness (Quilez's `k`, refined by Aaltonen's
    /// closest-point formula — see `trace_shadow`'s doc comment and
    /// `docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md`): larger =
    /// harder-edged shadows (smaller apparent light source). Same field
    /// `hybrid_legacy`'s `LightGpu::shadow_softness_k` names.
    pub shadow_softness_k: f32,
}

/// Empirical exposure scale bringing `bevy_pbr`'s physical light units
/// (lux for directional, lumens/steradian for point/spot — both large
/// real-world numbers, hundreds to tens of thousands) down to a
/// reasonable `[0,1]`-ish HDR range for this renderer's flat/Lambertian
/// shading, with no full physically-based exposure/tonemapping pipeline
/// yet. Identical constant to `hybrid_legacy`'s own `shade()` (`assets/
/// shaders/hybrid_legacy_trace.wgsl`), reused here as the proven
/// precedent for "what makes default Bevy light intensities look
/// reasonable" rather than re-deriving a value from scratch.
pub(crate) const EXPOSURE: f32 = 0.0005;

/// This light's direction-to-light, distance/cone attenuation, radiance
/// (color * intensity * exposure) before any BRDF is applied, and the
/// shadow ray's own `max_t` bound (see `trace_shadow`'s doc comment for
/// why this matters: the march bound is the RAY's real limit, not a BVH
/// candidate's slab exit — a point/spot shadow ray must stop at the
/// light itself, not march past it) — factored out of the old
/// Lambertian-only `light_contribution` so both the diffuse and specular
/// GGX terms in `shade` can share one attenuation/radiance computation
/// per light instead of duplicating it. Attenuation formula ported
/// verbatim from `hybrid_legacy`'s `shade()` (`assets/shaders/
/// hybrid_legacy_trace.wgsl:1345-1417`).
pub(crate) struct LightSample {
    pub(crate) to_light: Vec3,
    pub(crate) radiance: Vec3,
    pub(crate) shadow_max_t: f32,
}

/// Directional-light shadow rays have no natural light DISTANCE to bound
/// them by (parallel rays from infinitely far away) — bounded instead by
/// a fixed distance.
///
/// **Value is 12.0, NOT `hybrid_legacy`'s `60.0`, and NOT derived from the
/// scene's `root_diagonal` either — both were tried and found to be real
/// bugs at `--stress N`.** Occlusion for a directional light is inherently
/// LOCAL: nothing farther than a few object-heights away can ever cast a
/// shadow that matters at the shaded point. `root_diagonal` (an earlier
/// version of this constant) scales with the ENTIRE scene's extent — at
/// `--stress N`, the whole multi-cell grid — so it grew unboundedly with
/// unrelated object count. Switching to a FIXED distance was the right
/// direction, but reusing `hybrid_legacy`'s own `60.0` was ALSO wrong:
/// that value was tuned for a scene whose entire ground plane was only
/// ~11 units across, where `60.0` was ~5x the whole scene and therefore
/// harmless. In this renderer's `--stress N` grid (each cell also ~11
/// units, tiled edge-to-edge with `CELL_GAP` gaps), a directional shadow
/// ray at this scene's ~70° sun elevation travels `max_t / tan(elevation)
/// ≈ max_t * 0.36` horizontally before reaching `max_t` — at `60.0` that's
/// ~20 units, crossing into 1-2 NEIGHBORING grid cells and picking up
/// their real (but locally irrelevant) ground/cube geometry as shadow
/// candidates, with a `shadow_candidate_margin` (see that function's own
/// doc comment) padding those distant candidates' AABBs by another
/// `VIS_CUTOFF * k * max_t` on top. Confirmed visually: shadows on
/// individual cubes stretching several units past their own tile,
/// rotating correctly with the light's azimuth (ruling out a purely
/// BVH-structural artifact) but reaching much farther than any single
/// cube's true ~0.6-unit shadow length at this elevation — i.e. genuinely
/// picking up distant real geometry, not a phantom occluder. `12.0`
/// (≈5x this scene's tallest object height, 2.4 units) is generous
/// headroom for any locally-relevant occluder while staying well under
/// one grid-cell spacing (11 units + `CELL_GAP`), so a directional shadow
/// ray no longer reaches into neighboring cells at all at this scene's
/// object/grid scale. This is a real, scene-scale-dependent tuning
/// parameter (not a universal constant) — revisit if this renderer's
/// object heights or grid spacing change substantially.
const DIRECTIONAL_SHADOW_MAX_T: f32 = 12.0;

pub(crate) fn sample_light(light: &Light, p_world: Vec3) -> LightSample {
    let (to_light, attenuation, shadow_max_t) = match light.kind {
        LightKind::Directional => {
            (-light.direction_or_position.normalize(), 1.0, DIRECTIONAL_SHADOW_MAX_T)
        }
        LightKind::Point | LightKind::Spot => {
            let delta = light.direction_or_position - p_world;
            let dist = delta.length();
            let to_light = delta / dist.max(1e-4);
            let range = light.range.max(1e-3);
            let dist_atten = (1.0 - (dist / range).powi(4)).clamp(0.0, 1.0) / dist.max(1.0).powi(2);
            let atten = if light.kind == LightKind::Spot {
                // `-to_light` is the direction from the light TOWARD the
                // surface point (light->target); `spot_direction` is the
                // cone's own forward direction (also light->target when
                // the surface is exactly on-axis). Their dot product is
                // +1 exactly on-axis, falling toward -1 at the opposite
                // side — smoothstep over [cos(outer), cos(inner)] turns
                // that into a 1-at-center, 0-past-outer-edge cone falloff.
                let cos_angle = (-to_light).dot(light.spot_direction.normalize());
                dist_atten * smoothstep(light.outer_angle.cos(), light.inner_angle.cos(), cos_angle)
            } else {
                dist_atten
            };
            (to_light, atten, dist)
        }
    };
    // Point/Spot's `intensity` is raw lumens/steradian (see `Light::
    // intensity`'s doc comment) — divide by 4*PI here, matching
    // `bevy_pbr`'s own internal lumens -> luminous-intensity conversion,
    // so a Bevy `PointLight`'s `intensity` field means the same physical
    // quantity in this renderer as it would under Bevy's own PBR pipeline.
    let intensity = match light.kind {
        LightKind::Directional => light.intensity,
        LightKind::Point | LightKind::Spot => light.intensity / (4.0 * std::f32::consts::PI),
    };
    LightSample { to_light, radiance: light.color * intensity * EXPOSURE * attenuation, shadow_max_t }
}

/// Trowbridge-Reitz / GGX normal distribution function: how concentrated
/// the microfacet normals are around the half-vector `h`, `alpha =
/// roughness^2` (the standard perceptual-roughness -> alpha remap used by
/// `bevy_pbr` and this project's own `docs/knowledge/sdf-3d/
/// materials-and-texturing/pbr-shading-model.md`). Ported from
/// `hybrid_legacy`'s `ggx_ndf` (`assets/shaders/hybrid_legacy_trace.wgsl:
/// 923-926`) — this term itself is correct there; only the specular
/// composition around it (see `shade`'s doc comment) had the bug.
pub(crate) fn ggx_distribution(n_dot_h: f32, alpha: f32) -> f32 {
    let alpha_sq = alpha * alpha;
    let denom = n_dot_h * n_dot_h * (alpha_sq - 1.0) + 1.0;
    alpha_sq / (std::f32::consts::PI * denom * denom).max(1e-8)
}

/// Height-correlated Smith visibility term (folded with the `1/(4 NdotV
/// NdotL)` Cook-Torrance denominator, so `D * V * F` is the complete
/// specular term with no extra division needed) — same formula as
/// `hybrid_legacy`'s `ggx_vis` (`assets/shaders/hybrid_legacy_trace.wgsl:
/// 928-932`), which is itself correct; kept identical here.
pub(crate) fn ggx_visibility(n_dot_l: f32, n_dot_v: f32, alpha: f32) -> f32 {
    let alpha_sq = alpha * alpha;
    let lambda_v = n_dot_l * (n_dot_v * n_dot_v * (1.0 - alpha_sq) + alpha_sq).sqrt();
    let lambda_l = n_dot_v * (n_dot_l * n_dot_l * (1.0 - alpha_sq) + alpha_sq).sqrt();
    0.5 / (lambda_v + lambda_l).max(1e-4)
}

/// Schlick's Fresnel approximation: reflectance rises from `f0` (at normal
/// incidence, `cos_theta = 1`) toward white (full reflection) at grazing
/// angles. `f0` appears here exactly once per specular evaluation — see
/// `shade`'s doc comment for why that "exactly once" matters.
pub(crate) fn fresnel_schlick(f0: Vec3, cos_theta: f32) -> Vec3 {
    let m = (1.0 - cos_theta).clamp(0.0, 1.0);
    f0 + (Vec3::ONE - f0) * m.powi(5)
}

/// Remaps a `[0,1]` "reflectance" dial to a dielectric's F0 (Fresnel
/// reflectance at normal incidence) exactly as `bevy_pbr` does
/// (`StandardMaterial::reflectance`'s doc comment / `pbr_functions.wgsl`'s
/// `calculate_diffuse_color`): `F0 = 0.16 * reflectance^2`, so the
/// conventional default `reflectance = 0.5` reproduces the textbook ~4%
/// dielectric F0 (`0.16 * 0.25 = 0.04`).
pub(crate) fn dielectric_f0(reflectance: f32) -> f32 {
    0.16 * reflectance * reflectance
}

/// Perceptual roughness -> GGX alpha remap (`alpha = roughness^2`,
/// clamped away from exactly zero) — pulled out of `shade`'s own inline
/// computation into its own function so `reflect_ref.rs` can reuse the
/// EXACT same formula rather than re-deriving it (this project's own
/// established "reuse the exact formula already computed" preference
/// whenever a value needs to agree bit-for-bit across two call sites, as
/// opposed to the separate "duplicate small formulas" convention used
/// when two call sites' own semantics genuinely diverge — here they
/// don't, both want literally the same alpha).
pub(crate) fn ggx_alpha(roughness: f32) -> f32 {
    roughness.clamp(0.0, 1.0).powi(2).max(1e-3)
}

/// Full metallic-roughness GGX (Cook-Torrance) shading at a hit point,
/// summed over every light, plus the material's own emissive term.
///
/// Structure verified against real `bevy_pbr` source
/// (`crates/bevy_pbr/src/render/pbr_lighting.wgsl`'s `specular_multiscatter`
/// and its caller) during this port: the correct specular term is `D * V *
/// F`, with `F0`/reflectance folded into the Fresnel term `F` **exactly
/// once**. This deliberately does NOT port `hybrid_legacy`'s `shade()`
/// (`assets/shaders/hybrid_legacy_trace.wgsl:1345-1425`) verbatim — that
/// function computes `spec = D * Vis * f0` (baking F0 into what should be
/// a pure `D*V` term) and then multiplies by a *second*, independently
/// computed `fresnel_schlick_vec(f0, ...)` term on top, so its final
/// specular is proportional to `F0` twice (visibly too-dark specular
/// highlights on rough metals/high-reflectance dielectrics). This is a
/// real bug in the frozen legacy renderer, confirmed by comparing against
/// `bevy_pbr`'s real, shipped formula rather than assumption; flagged to
/// the user rather than silently ported forward or silently fixed in
/// `hybrid_legacy` itself (which stays untouched, per this project's
/// legacy-code convention). This function instead applies F0 once, matching
/// the verified-correct structure — no multi-scatter compensation term is
/// added (Filament/bevy_pbr's `Fr *= 1.0 + F0 * (1.0/F_ab.x - 1.0)` needs a
/// precomputed BRDF-integration LUT/analytic fit for `F_ab` that this
/// renderer doesn't have yet; single-scattering GGX energy loss shows up
/// only as slightly-too-dark rough metals, a smaller and separable error
/// than the double-F0 bug this port fixes).
/// `bvh`/`objects`/`origin_entity`/`hit_t`: everything `shade` needs to
/// cast a soft shadow ray per light via `trace_shadow` — see that
/// function's doc comment for the ported algorithm. `origin_entity` is
/// the entity actually being shaded (excluded from its own shadow-ray
/// candidate slab); `hit_t` is the PRIMARY ray's own hit distance, needed
/// to compute the shadow ray's biased origin via `shadow_bias` (NOT the
/// shadow ray's own distance, which doesn't exist yet at that point).
/// Returns `ShadeResult` (direct+emissive vs. indirect, SEPARATE), not a
/// single summed `Vec3` — DDGI's own indirect term (`indirect`, filled
/// in by the caller via `ddgi_ref::sample_probe_grid`, not by `shade`
/// itself — this function has no DDGI dependency, mirroring its own
/// prior "no Stage B dependency inside indirect_diffuse's caller" split)
/// still benefits from staying separate from `direct_and_emissive`: a
/// same-frame edge-aware spatial blur can still be applied to just the
/// indirect term downstream without also softening legitimately sharp
/// direct-lit detail (specular highlights, hard shadow edges).
/// `reflection`: `None` for a shading call that must NOT recurse into
/// reflection tracing (e.g. `conetrace_ref::cone_trace_ray`'s own bounce
/// shading — mirrors `hybrid_trace.wgsl`'s separate `shade_direct_only_
/// for_cone` function, which structurally never calls the real `shade()`
/// at all and so can't recurse into reflection either; the CPU ref shares
/// one `shade` function for both roles, so this flag reproduces that same
/// non-recursion by parameter instead of by a separate function), `Some`
/// for a real primary-ray shading call that should fire reflection rays
/// per `extract::ReflectionConfig`.
#[derive(Clone, Copy, Debug)]
pub struct ReflectionParams {
    pub enabled: bool,
    pub max_bounces: u32,
    pub fresnel_cutoff: f32,
    pub max_t: f32,
    pub diffuse_gi_max_t: f32,
    pub diffuse_gi_r0: f32,
    pub diffuse_gi_half_angle: f32,
    /// Whether a reflection bounce's own single-cone "final gather" GI
    /// term (`reflect_ref::shade_for_reflection_bounce`'s own
    /// `cone_trace_indirect_single` call) should run at all — a REAL bug
    /// this field fixes (found 2026-09-18): that call used to run
    /// UNCONDITIONALLY regardless of the scene's own primary
    /// `GiMethod`, so a fully sealed, sun-only room still read as
    /// faintly lit through/around any reflective surface even under
    /// `GiMethod::None`. Callers should pass `gi_method != GiMethod::None`
    /// here, not always `true` — `shade` itself has no `GiMethod`
    /// awareness of its own (see `ShadeResult`'s own doc comment for
    /// why), so this decision has to be made by `shade`'s own caller,
    /// same as every other GI-method-specific decision in this codebase.
    pub bounce_gi_enabled: bool,
}

/// Mirrors `ReflectionParams` exactly (see that struct's own doc comment
/// for the `None`/`Some` contract this preserves) — the transmission
/// counterpart threaded through `shade`'s own call sites.
#[derive(Clone, Copy, Debug)]
pub struct TransmissionParams {
    pub enabled: bool,
    pub max_bounces: u32,
    pub fresnel_cutoff: f32,
    pub max_t: f32,
    pub diffuse_gi_max_t: f32,
    pub diffuse_gi_r0: f32,
    pub diffuse_gi_half_angle: f32,
    /// `ReflectionParams::bounce_gi_enabled`'s own transmission
    /// counterpart — see that field's own doc comment for the full
    /// rationale (identical bug, identical fix, applied to
    /// `refract_ref::shade_for_refraction_bounce`'s own cone-traced GI
    /// term instead).
    pub bounce_gi_enabled: bool,
}

#[allow(clippy::too_many_arguments)]
pub fn shade(
    material: &Material,
    p_world: Vec3,
    world_normal: Vec3,
    view_dir: Vec3,
    lights: &[Light],
    bvh: &Bvh,
    objects: &[TraceObject],
    origin_entity: Option<Entity>,
    hit_t: f32,
    reflection: Option<ReflectionParams>,
    transmission: Option<TransmissionParams>,
) -> ShadeResult {
    let n = world_normal.normalize();
    let v = view_dir.normalize();
    let n_dot_v = n.dot(v).max(1e-4);

    let albedo = material.base_color;
    let metallic = material.metallic.clamp(0.0, 1.0);
    // Perceptual roughness -> alpha remap (alpha = roughness^2), the same
    // convention `bevy_pbr` and this project's own PBR research use —
    // clamped away from exactly zero so `ggx_distribution`/`ggx_visibility`
    // never divide by a fully degenerate mirror-alpha.
    let alpha = ggx_alpha(material.roughness);
    let f0 = Vec3::splat(dielectric_f0(material.reflectance)).lerp(albedo, metallic);
    let diffuse_color = albedo * (1.0 - metallic);
    let shadow_origin = p_world + n * shadow_bias(hit_t);

    let mut radiance = Vec3::ZERO;
    for light in lights {
        let sample = sample_light(light, p_world);
        let l = sample.to_light;
        let n_dot_l = n.dot(l).max(0.0);
        if n_dot_l <= 0.0 {
            continue;
        }
        // A light already contributing nothing here (attenuated to zero
        // radiance) doesn't need a shadow query — skip it, matching
        // `hybrid_legacy`'s own `if (atten <= 0.0) { continue; }` early-out.
        if sample.radiance == Vec3::ZERO {
            continue;
        }
        let shadow_vis =
            trace_shadow(bvh, objects, shadow_origin, l, sample.shadow_max_t, light.shadow_softness_k, origin_entity)
                .vis();
        if shadow_vis <= 0.0 {
            continue;
        }

        let h = (l + v).normalize();
        let n_dot_h = n.dot(h).max(0.0);
        let v_dot_h = v.dot(h).max(0.0);

        let f = fresnel_schlick(f0, v_dot_h);
        let d = ggx_distribution(n_dot_h, alpha);
        let vis = ggx_visibility(n_dot_l, n_dot_v, alpha);
        let specular = f * (d * vis);
        // Energy-conserving diffuse: light not reflected specularly (`1 -
        // F`) is available to scatter diffusely; metals have no diffuse
        // term at all (`diffuse_color` is already zeroed by `metallic`).
        let diffuse = diffuse_color * (Vec3::ONE - f) / std::f32::consts::PI;

        radiance += (diffuse + specular) * sample.radiance * n_dot_l * shadow_vis;
    }
    // DDGI's indirect term is resolved by the CALLER (via
    // `ddgi_ref::sample_probe_grid`, multiplied by this same
    // `diffuse_color` convention), not here — `shade` itself no longer
    // has an indirect-lighting dependency now that Stage B (the
    // per-pixel hemisphere-sample indirect diffuse this function used to
    // compute inline) is removed. `indirect` stays zero-initialized and
    // is left for the caller to fill in, keeping `ShadeResult`'s two-
    // field shape (see its own doc comment) so the denoise/temporal
    // passes downstream don't need to change.
    let indirect = Vec3::ZERO;

    // Multi-bounce specular reflection — see `reflect_ref::reflect_trace_ray`'s
    // own doc comment. Fresnel-gated: skip firing the ray entirely below
    // `fresnel_cutoff` so flat, low-reflectance dielectrics pay zero
    // extra cost. Combined with the SAME per-pixel Fresnel term `f0`/
    // `n_dot_v` already establish for direct-light shading above, not a
    // re-derived one, so the energy split matches.
    let mut reflect_color = Vec3::ZERO;
    if let Some(r) = reflection
        && r.enabled
    {
        let primary_f = fresnel_schlick(f0, n_dot_v);
        if crate::hybrid::reflect_ref::luminance(primary_f) >= r.fresnel_cutoff {
            let reflect_dir = crate::hybrid::reflect_ref::reflect(-v, n);
            let reflected = crate::hybrid::reflect_ref::reflect_trace_ray(
                bvh,
                objects,
                lights,
                p_world + n * crate::hybrid::reflect_ref::REFLECT_RAY_BIAS,
                reflect_dir,
                r.max_t,
                origin_entity,
                r.max_bounces,
                r.diffuse_gi_max_t,
                r.diffuse_gi_r0,
                r.diffuse_gi_half_angle,
                r.bounce_gi_enabled,
            );
            reflect_color = primary_f * reflected;
        }
    }

    // Multi-bounce transmission/refraction — see `refract_ref::refract_trace_ray`'s
    // own doc comment. Fresnel-gated (the SAME primary_f-style term,
    // energy-complementary to reflection's own share) and additionally
    // gated on `material.transmission > 0.0`, since a material with
    // transmission=0.0 is opaque regardless of what the caller's
    // `TransmissionParams::enabled` says (matches `metallic`'s own "no
    // diffuse term at all when fully metallic" per-material override of
    // an otherwise-scene-wide toggle). Needs the FULL `TraceObject` (not
    // just `material`) for `refract_ref::refract_trace_ray`'s own
    // interior march, which is per-object (shape/transform), not just
    // per-material — looked up via `origin_entity` against `objects`,
    // matching `reflect_trace_ray`'s own probe-and-lookup pattern rather
    // than widening `shade`'s own signature further.
    let mut refract_color = Vec3::ZERO;
    if let Some(t) = transmission
        && t.enabled
        && material.transmission > 0.0
        && let Some(entry_object) = origin_entity.and_then(|e| objects.iter().find(|o| o.entity == e))
    {
        let entry_f = fresnel_schlick(f0, n_dot_v);
        let transmittable = Vec3::ONE - entry_f;
        if crate::hybrid::reflect_ref::luminance(transmittable) >= crate::hybrid::refract_ref::TRANSMISSION_FRESNEL_CUTOFF {
            let transmitted = crate::hybrid::refract_ref::refract_trace_ray(
                bvh,
                objects,
                lights,
                p_world,
                n,
                v,
                entry_object,
                t.max_t,
                t.max_bounces,
                t.diffuse_gi_max_t,
                t.diffuse_gi_r0,
                t.diffuse_gi_half_angle,
                t.bounce_gi_enabled,
            );
            refract_color = (Vec3::ONE - entry_f) * material.transmission * transmitted;
        }
    }

    ShadeResult { direct_and_emissive: radiance + material.emissive, indirect, reflect: reflect_color, refract: refract_color }
}

/// `shade`'s return: the direct-lit (all lights, GGX/Lambert) plus
/// emissive contribution, and the (still un-blurred, pre-denoise)
/// single-bounce indirect-diffuse contribution, kept SEPARATE rather
/// than summed. Why: `indirect` alone is noisy (Stage B's 5 short
/// jittered hemisphere probes per pixel — see `indirect_diffuse`'s own
/// doc comment), and the fix is a same-frame edge-aware spatial blur
/// applied ONLY to this term before it's composited into the final
/// image (`direct_and_emissive + blurred(indirect)`, done downstream of
/// `shade` itself — this function only produces the two ingredients).
/// Blurring the whole already-composited color instead (the simpler-
/// looking alternative) would also soften `direct_and_emissive`'s own
/// legitimately sharp detail (specular highlights, hard shadow
/// terminators) that was never noisy — confirmed by direct visual
/// inspection that this would trade one visible problem for another,
/// not actually fix anything. `indirect` here is already multiplied by
/// `diffuse_color` (matching `hybrid_legacy`'s own "irradiance times
/// albedo" contract for this term) — NOT raw irradiance — so the blur
/// pass works on a signal already scaled the way it will be displayed,
/// and no separate albedo texture needs to survive past this function
/// for later recombination.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ShadeResult {
    pub direct_and_emissive: Vec3,
    pub indirect: Vec3,
    /// Multi-bounce specular reflection contribution — kept SEPARATE from
    /// both `direct_and_emissive` and `indirect` for the identical reason
    /// `indirect` is already split out (see this struct's own doc comment
    /// above): reflections need their OWN temporal history, since a
    /// reflected image's apparent motion does not follow the reflecting
    /// surface's own motion. Filled in directly by `shade` itself (unlike
    /// `indirect`, which the CALLER fills in via whichever `GiMethod` is
    /// active) — reflection is a fixed part of PBR specular shading, not
    /// a swappable technique, see `extract::ReflectionConfig`'s own doc
    /// comment for why it's independent of `GiMethod`.
    pub reflect: Vec3,
    /// Multi-bounce transmission/refraction contribution — kept SEPARATE
    /// for the identical reason `reflect` already is: a refracted image's
    /// apparent motion follows neither the entry surface's own motion nor
    /// reflection's own virtual-point convention (it's the EXIT surface,
    /// possibly of a different, moving object, seen through a bent ray),
    /// so it needs its own dedicated temporal history. Filled in directly
    /// by `shade` itself, same as `reflect` — transmission is a fixed
    /// part of PBR shading for a `transmission > 0.0` material, not a
    /// swappable technique.
    pub refract: Vec3,
}

// ---------------------------------------------------------------------------------
// Same-frame edge-aware spatial denoise for the (noisy) indirect-diffuse
// term above — NOT a replacement for real temporal accumulation (this
// renderer has none: no history buffer, no motion vectors, no
// reprojection), but the right-sized fix for right now: a small
// fixed-radius bilateral blur, weighted by how similar each neighboring
// pixel's surface normal and depth are to the center pixel's own, so it
// smooths flat noisy regions while refusing to blur across a real edge
// (a different object, a silhouette, a sharp corner). This is the
// standard baseline layer production denoisers (SVGF and its
// descendants) always keep even once temporal accumulation is added —
// building it now is not throwaway work.
// ---------------------------------------------------------------------------------

/// Blur radius in pixels (a 5x5 tap window: `-BLUR_RADIUS..=BLUR_RADIUS`
/// on both axes) — small and cheap on purpose. A single-bounce diffuse
/// GI term is inherently low-frequency (it's an average of only 5
/// samples spread across a whole hemisphere, not a sharp specular
/// reflection), so a small window is enough to visually clean up the
/// grain; a larger window would cost more per pixel for little
/// additional smoothing and risks bleeding across thin real geometry.
const BLUR_RADIUS: i32 = 2;

/// How quickly blur weight falls off as a neighbor's normal diverges
/// from the center pixel's own — larger values tolerate more normal
/// divergence before a neighbor's contribution is suppressed. Tuned so a
/// neighbor on the SAME roughly-flat surface (normals within a few
/// degrees, e.g. across a curved sphere's own smooth surface) still
/// contributes close to full weight, while a neighbor on a genuinely
/// different face (a hard 90-degree edge) is suppressed to near zero.
const BLUR_NORMAL_SIGMA: f32 = 0.1;

/// How quickly blur weight falls off as a neighbor's linear depth
/// diverges from the center pixel's own, as a FRACTION of the center
/// pixel's own depth (not an absolute world-unit threshold) — a neighbor
/// twice as far away on a shallow-grazing surface should still be
/// treated as "probably the same surface," while the same absolute
/// depth gap on a near-camera close-up should be treated as "probably a
/// different surface" (a silhouette edge). Scaling by the center depth
/// itself is what makes this work correctly at both this renderer's
/// near and far camera framings without a second tuning constant.
const BLUR_DEPTH_SIGMA: f32 = 0.05;

/// One pixel's edge-aware blur sample: its indirect-diffuse color, its
/// world-space shading normal, and its linear hit-`t` depth — everything
/// the weight formula needs. Mirrors the three per-pixel textures the
/// real WGSL pass reads (`indirect_view`, `normal_view`, the existing
/// `out_depth`).
#[derive(Clone, Copy, Debug)]
pub struct BlurSample {
    pub indirect: Vec3,
    pub normal: Vec3,
    pub depth: f32,
}

/// Edge-aware bilateral blur of the indirect-diffuse term at one pixel.
/// `sample_at(dx, dy)` fetches a neighboring pixel's `BlurSample` (an
/// out-of-bounds neighbor should clamp to the nearest valid pixel — same
/// convention a real texture sampler's clamp-to-edge mode would use;
/// callers are responsible for that clamping, this function only walks
/// `-BLUR_RADIUS..=BLUR_RADIUS` offsets and trusts whatever `sample_at`
/// returns). `center` is this pixel's own sample (`sample_at(0, 0)`,
/// passed separately rather than re-fetched so callers that already have
/// it don't pay for a redundant lookup).
///
/// Weight formula: Gaussian-ish falloff on normal-dot-product and
/// relative depth difference, multiplied together — a neighbor must be
/// BOTH on a similar-facing surface AND at a similar depth to contribute
/// significant weight, matching the standard "which neighboring pixels
/// probably belong to the same surface" test used across production
/// SSAO/GI denoisers (see this section's own doc comment).
pub fn blur_indirect_at(center: BlurSample, sample_at: impl Fn(i32, i32) -> BlurSample) -> Vec3 {
    let mut acc = Vec3::ZERO;
    let mut weight_sum = 0.0f32;
    for dy in -BLUR_RADIUS..=BLUR_RADIUS {
        for dx in -BLUR_RADIUS..=BLUR_RADIUS {
            let neighbor = sample_at(dx, dy);
            let normal_similarity = center.normal.normalize().dot(neighbor.normal.normalize()).max(0.0);
            let normal_weight = normal_similarity.powf(1.0 / BLUR_NORMAL_SIGMA.max(1e-4));
            let depth_scale = BLUR_DEPTH_SIGMA * center.depth.max(1e-4);
            let depth_diff = (center.depth - neighbor.depth).abs();
            let depth_weight = (-depth_diff / depth_scale.max(1e-4)).exp();
            let weight = normal_weight * depth_weight;
            acc += neighbor.indirect * weight;
            weight_sum += weight;
        }
    }
    if weight_sum > 1e-6 { acc / weight_sum } else { center.indirect }
}

/// How much of the full bilateral blur to apply, given how many frames of
/// temporal history this pixel has already accumulated
/// (`temporal_ref::temporal_blend`'s own `history_length` output) — 1.0
/// (full blur, same as before temporal accumulation existed) at
/// `history_length <= 1` (a brand-new or just-rejected pixel: this
/// frame's raw 5-sample estimate is all there is, so it needs the same
/// spatial cleanup it always did), fading linearly to 0.0 (no blur at
/// all — trust the temporally-converged signal untouched) once
/// `history_length` reaches `max_history_length`.
///
/// Why fade rather than leave the blur at full strength once temporal
/// accumulation exists: once history has converged, the temporally-
/// accumulated indirect value is already low-noise (an effective
/// multi-frame sample average) — blurring it further trades sharpness
/// for noise reduction it no longer needs, exactly the tradeoff that
/// motivated adding temporal accumulation as the PRIMARY denoiser in the
/// first place (see `blur_indirect_at`'s own doc comment: the spatial
/// blur's remaining job is covering pixels that don't have a converged
/// history yet, not permanently smoothing everything).
pub fn blur_strength(history_length: f32, max_history_length: f32) -> f32 {
    let denom = (max_history_length - 1.0).max(1e-4);
    (1.0 - (history_length - 1.0) / denom).clamp(0.0, 1.0)
}

/// `blur_indirect_at`, weighted down toward the unblurred `center.indirect`
/// as `history_length` converges — see `blur_strength`'s own doc comment
/// for the fade curve and rationale. At `history_length <= 1` this is
/// identical to `blur_indirect_at`'s own full-strength result; at
/// `history_length >= max_history_length` it's identical to
/// `center.indirect` (a full-strength blur computed and then thrown away
/// is wasteful GPU-side — the real WGSL port skips the weight loop
/// entirely once strength rounds to zero, see `hybrid_denoise.wgsl`'s own
/// comment — but this CPU reference always computes both to keep the
/// numerical contract simple and directly testable).
pub fn adaptive_blur_indirect_at(
    center: BlurSample,
    history_length: f32,
    max_history_length: f32,
    sample_at: impl Fn(i32, i32) -> BlurSample,
) -> Vec3 {
    let strength = blur_strength(history_length, max_history_length);
    let blurred = blur_indirect_at(center, sample_at);
    center.indirect.lerp(blurred, strength)
}

/// Hermite smoothstep, `x` clamped to `[0,1]` first — WGSL's built-in
/// `smoothstep` has this exact shape; Rust has no builtin, so this is
/// spelled out once here and reused by every call site that needs the
/// spot cone's smooth inner/outer-angle edge.
pub(crate) fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Signed distance from a point (already in the box's own local,
/// unrotated space) to a rounded box — identical formula to
/// `hybrid_legacy`'s `sd_rounded_box` (both the WGSL and CPU-reference
/// copies), reused here as the pattern to port, not imported.
fn sd_rounded_box(p_local: Vec3, half_extents: Vec3, corner_radius: f32) -> f32 {
    let q = p_local.abs() - half_extents + Vec3::splat(corner_radius);
    q.max(Vec3::ZERO).length() + q.x.max(q.y).max(q.z).min(0.0) - corner_radius
}

/// Ported verbatim from `sdf::primitives::Sphere::distance`.
fn sd_sphere(p_local: Vec3, radius: f32) -> f32 {
    p_local.length() - radius
}

/// Ported verbatim from `sdf::primitives::RoundedCylinder::distance`.
fn sd_rounded_cylinder(p_local: Vec3, radius: f32, half_height: f32, edge_radius: f32) -> f32 {
    let d = bevy::math::Vec2::new(
        bevy::math::Vec2::new(p_local.x, p_local.z).length() - radius + edge_radius,
        p_local.y.abs() - half_height + edge_radius,
    );
    d.x.max(d.y).min(0.0) + d.max(bevy::math::Vec2::ZERO).length() - edge_radius
}

/// Ported verbatim from `sdf::primitives::Capsule::distance`. `a`/`b` are
/// in the shape's own local space (this renderer applies the entity's
/// rotation+translation uniformly to every shape, unlike `hybrid_legacy`'s
/// raymarcher which stores `Capsule`/`RoundedCone` endpoints in world
/// space and skips the entity's rotation — a deliberate simplification so
/// every shape shares one transform path, see this module's git history
/// for the tradeoff).
fn sd_capsule(p_local: Vec3, a: Vec3, b: Vec3, radius: f32) -> f32 {
    let ab = b - a;
    let ap = p_local - a;
    let t = (ap.dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
    let closest = a + ab * t;
    (p_local - closest).length() - radius
}

/// Ported verbatim from `sdf::primitives::Ellipsoid::distance`.
fn sd_ellipsoid(p_local: Vec3, radii: Vec3) -> f32 {
    let k0 = (p_local / radii).length();
    let k1 = (p_local / (radii * radii)).length();
    k0 * (k0 - 1.0) / k1
}

/// Ported verbatim from `sdf::primitives::BoxFrame::distance`.
fn sd_box_frame(p_local: Vec3, half_extents: Vec3, wall_thickness: f32) -> f32 {
    let q = p_local.abs() - half_extents;
    let outer = q.max(Vec3::ZERO).length() + q.x.max(q.y.max(q.z)).min(0.0);
    let inner_q = q + Vec3::splat(wall_thickness);
    let inner = inner_q.max(Vec3::ZERO).length() + inner_q.x.max(inner_q.y.max(inner_q.z)).min(0.0);
    outer.max(-inner)
}

/// Ported verbatim from `sdf::primitives::HexPrism::distance`.
fn sd_hex_prism(p_local: Vec3, radius: f32, half_height: f32) -> f32 {
    let q = p_local.abs();
    let k = 0.866_025_4_f32;
    let hex_d = q.x.max((0.5 * q.x + k * q.z).abs()).max((0.5 * q.x - k * q.z).abs()) - radius;
    let axial_d = q.y - half_height;
    hex_d.max(axial_d)
}

/// This shape's signed distance from `p_local`, a point already
/// transformed into the shape's local (unrotated, untranslated) space.
/// Every `Shape` variant's formula is ported verbatim from
/// `sdf::primitives`'s `Sdf::distance` implementations (the project's
/// existing, already-proven CPU reference for these exact primitives) —
/// see each `sd_*` helper's own doc comment for its source. `Capsule`'s
/// `a`/`b` are local-space offsets from the entity origin here (not
/// world-space, unlike `hybrid_legacy`'s raymarcher) so every shape
/// shares the same uniform rotation+translation transform.
///
/// `RoundedCone` is deliberately NOT ported here yet: `sdf::primitives::
/// RoundedCone::distance` (and the matching WGSL `sdf_rounded_cone` in
/// `raymarch.wgsl`) has a real, pre-existing correctness bug — every
/// point tested while porting this module (including the shape's own
/// centerline and endpoints) reported as exterior, which cannot be
/// right. That bug lives in already-shipped code outside this module's
/// scope and needs its own fix, not a silent port of broken math into a
/// second renderer — flagged, not fixed, here.
pub(crate) fn local_distance(shape: &Shape, p_local: Vec3) -> f32 {
    match *shape {
        Shape::Sphere { radius } => sd_sphere(p_local, radius),
        Shape::RoundedBox { half_extents, corner_radius } => sd_rounded_box(p_local, half_extents, corner_radius),
        Shape::RoundedCylinder { radius, half_height, edge_radius } => {
            sd_rounded_cylinder(p_local, radius, half_height, edge_radius)
        }
        Shape::Capsule { a, b, radius } => sd_capsule(p_local, a, b, radius),
        Shape::RoundedCone { .. } => {
            unimplemented!("local_distance: RoundedCone is skipped — see this function's doc comment")
        }
        Shape::Ellipsoid { radii } => sd_ellipsoid(p_local, radii),
        Shape::BoxFrame { half_extents, wall_thickness } => sd_box_frame(p_local, half_extents, wall_thickness),
        Shape::HexPrism { radius, half_height } => sd_hex_prism(p_local, radius, half_height),
    }
}

/// Surface normal at `p_local` (already in the shape's own local,
/// unrotated space) via central-difference finite differencing of
/// `local_distance` — the standard shape-agnostic SDF normal estimator:
/// works identically for every `Shape` variant without needing a
/// per-shape analytic gradient formula. `EPSILON` is deliberately much
/// larger than `HIT_EPSILON` (the march-convergence tolerance): finite
/// differencing needs a step large enough that the SDF's local curvature
/// doesn't get swamped by float precision noise at the sample points, not
/// small enough to approximate a true derivative arbitrarily closely.
///
/// A 4-tap "tetrahedron" pattern and closed-form per-shape analytic
/// gradients (ported from `raymarch.wgsl`'s `sdg_*` functions) were both
/// tried and measured against this method at `--stress 10000` — all
/// three were statistically indistinguishable in GPU cost, since BVH
/// traversal and marching dominate total frame time so heavily that
/// per-pixel normal-computation cost doesn't move the needle at this
/// renderer's current scale, so both alternatives were removed rather
/// than kept as unused code paths. See `PROGRESS.md`'s "Analytic vs.
/// finite-difference normals" entry for the full comparison and
/// measured numbers, if this is ever worth revisiting once the
/// renderer's bottleneck profile changes.
const NORMAL_EPSILON: f32 = 1e-3;

pub(crate) fn local_normal(shape: &Shape, p_local: Vec3) -> Vec3 {
    let e = NORMAL_EPSILON;
    let dx = local_distance(shape, p_local + Vec3::new(e, 0.0, 0.0))
        - local_distance(shape, p_local - Vec3::new(e, 0.0, 0.0));
    let dy = local_distance(shape, p_local + Vec3::new(0.0, e, 0.0))
        - local_distance(shape, p_local - Vec3::new(0.0, e, 0.0));
    let dz = local_distance(shape, p_local + Vec3::new(0.0, 0.0, e))
        - local_distance(shape, p_local - Vec3::new(0.0, 0.0, e));
    Vec3::new(dx, dy, dz).normalize_or_zero()
}

/// Sphere-marches `ray_origin + t * ray_dir` (world space) against one
/// object, starting from `t_start` (the ray's entry into the object's
/// BVH-culled AABB slab — marching from world-space `t=0` would waste
/// steps crossing empty space the BVH already proved is empty) up to
/// `t_max`. Returns the hit distance `t` on convergence, or `None` if the
/// march runs out of steps or exceeds `t_max` first.
///
/// The convergence tolerance grows with `t` via `pixel_eps` (the same
/// distance-scaled precision curve `shadow_bias` already trusts for its
/// "farther hits have coarser marching precision" offset): a hit 50 units
/// out doesn't need `HIT_EPSILON`-tight convergence, since a surface
/// position error of a few centimeters at that range is already smaller
/// than a pixel's own footprint. This directly cuts step count on distant
/// geometry — the dominant cost of sphere marching — without changing
/// near-field precision (`pixel_eps`'s own floor keeps close hits exactly
/// as tight as before). See `PROGRESS.md`'s "distance-scaled march
/// epsilon" entry for the measured perf/quality tradeoff.
fn march_object(object: &TraceObject, ray_origin: Vec3, ray_dir: Vec3, t_start: f32, t_max: f32) -> Option<f32> {
    let inv_rotation = object.rotation.inverse();
    let mut t = t_start.max(0.0);
    for _ in 0..MAX_MARCH_STEPS {
        if t > t_max {
            return None;
        }
        let p_world = ray_origin + t * ray_dir;
        let p_local = inv_rotation * (p_world - object.translation);
        let d = local_distance(&object.shape, p_local);
        if d < HIT_EPSILON.max(pixel_eps(t)) {
            return Some(t);
        }
        t += d;
    }
    None
}

/// Branchless slab test (Kay & Kajiya): returns `(t_near, t_far)`, the
/// ray's entry/exit distances through `aabb`, clamped to `[0, t_max]`.
/// `t_near > t_far` (or `t_far < 0`) means the ray misses the box
/// entirely. Mirrors `hybrid_legacy`'s `slab_hit` (both its Rust and
/// WGSL copies use this exact formula) — read as a pattern to port, not
/// imported.
fn slab_hit(ray_origin: Vec3, ray_dir: Vec3, aabb: &Aabb, t_max: f32) -> (f32, f32) {
    let inv_dir = Vec3::new(1.0 / ray_dir.x, 1.0 / ray_dir.y, 1.0 / ray_dir.z);
    let t0 = (aabb.min - ray_origin) * inv_dir;
    let t1 = (aabb.max - ray_origin) * inv_dir;
    let t_small = t0.min(t1);
    let t_big = t0.max(t1);
    let t_near = t_small.x.max(t_small.y).max(t_small.z).max(0.0);
    let t_far = t_big.x.min(t_big.y).min(t_big.z).min(t_max);
    (t_near, t_far)
}

/// Traces one ray against the whole BVH-accelerated scene: descends the
/// tree via `slab_hit`, skipping any subtree the ray's box test misses
/// entirely, and only sphere-marches the shapes of leaves the ray's AABB
/// slab actually enters — the whole point of building the BVH in the
/// first place. Returns the closest hit (by `t`) across every candidate
/// object the ray could plausibly touch, or `None` if the ray hits
/// nothing (background).
pub fn trace(bvh: &Bvh, objects: &[TraceObject], ray_origin: Vec3, ray_dir: Vec3, t_max: f32) -> Option<Hit> {
    if bvh.nodes.is_empty() {
        return None;
    }
    let mut best: Option<Hit> = None;
    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = bvh.nodes[node_index];
        let (t_near, t_far) = slab_hit(ray_origin, ray_dir, &node.aabb, best.map_or(t_max, |h| h.t));
        if t_near > t_far {
            continue; // ray misses this node's box entirely; skip its whole subtree
        }
        if node.left_or_sentinel == LEAF_SENTINEL {
            let Some(object) = objects.iter().find(|o| o.entity == node.entity) else {
                continue; // BVH leaf with no matching scene data this frame; skip
            };
            let march_limit = best.map_or(t_far, |h| h.t.min(t_far));
            if let Some(t) = march_object(object, ray_origin, ray_dir, t_near, march_limit)
                && best.is_none_or(|h| t < h.t)
            {
                let p_world = ray_origin + t * ray_dir;
                let p_local = object.rotation.inverse() * (p_world - object.translation);
                let local_normal = local_normal(&object.shape, p_local);
                let world_normal = object.rotation * local_normal;
                best = Some(Hit { entity: object.entity, t, world_normal, color: object.material.base_color });
            }
            continue;
        }
        // Push the two children ordered so the nearer one pops first
        // (stack is LIFO: pushed last = popped first). A closer-hit-first
        // visit order tightens `best_t` sooner, so the farther subtree's
        // own slab test (evaluated once it's actually popped) is more
        // likely to already exceed the shrunk `best_t` and get skipped
        // outright — same final result (traversal order never changes
        // which object wins, only how much dead subtree work is skipped
        // getting there), strictly less or equal total work. A child
        // whose box the ray misses entirely is not pushed at all, mirroring
        // the early-skip already applied to popped nodes above.
        let left = node.left_or_sentinel as usize;
        let right = node.right_or_object as usize;
        let limit = best.map_or(t_max, |h| h.t);
        let left_hit = slab_hit(ray_origin, ray_dir, &bvh.nodes[left].aabb, limit);
        let right_hit = slab_hit(ray_origin, ray_dir, &bvh.nodes[right].aabb, limit);
        let left_visitable = left_hit.0 <= left_hit.1;
        let right_visitable = right_hit.0 <= right_hit.1;
        if left_visitable && right_visitable {
            if left_hit.0 <= right_hit.0 {
                stack.push(right);
                stack.push(left);
            } else {
                stack.push(left);
                stack.push(right);
            }
        } else if left_visitable {
            stack.push(left);
        } else if right_visitable {
            stack.push(right);
        }
    }
    best
}

/// Any-hit occlusion query: "does anything other than `exclude` converge
/// somewhere in `[0, t_max]` along this ray?" — for callers that only ever
/// read a boolean (DDGI's per-probe hard-occlusion check being the
/// motivating case, `ddgi_ref.rs::sample_probe_grid`), not the full
/// `trace()`'s nearest-hit `Hit` (entity/t/normal/color). Unlike `trace()`,
/// this never shrinks its own search bound as hits are found — there's no
/// "closer" to chase, since the FIRST convergence (against a non-excluded
/// object) is immediately conclusive and the traversal returns without
/// visiting any more nodes. `t_max` therefore stays fixed for the whole
/// walk, and node visit order is a pure performance heuristic (near-first
/// still visited first, matching `trace()`'s own ordering, so a real
/// occluder close to the ray origin is found quickly) rather than a
/// correctness requirement the way it is for `trace()`'s "closest wins".
///
/// `exclude` mirrors every existing occlusion call site's own
/// `hit.entity != origin_entity` self-exclusion (a shaded object's own
/// occlusion ray toward a probe/light must not occlude against itself) —
/// baked into the traversal rather than left to the caller, so a march
/// that converges against the excluded object does NOT stop the search;
/// it keeps going, exactly as if that leaf had returned no hit at all.
pub fn any_hit(
    bvh: &Bvh, objects: &[TraceObject], ray_origin: Vec3, ray_dir: Vec3, t_max: f32, exclude: Option<Entity>,
) -> bool {
    if bvh.nodes.is_empty() {
        return false;
    }
    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = bvh.nodes[node_index];
        let (t_near, t_far) = slab_hit(ray_origin, ray_dir, &node.aabb, t_max);
        if t_near > t_far {
            continue;
        }
        if node.left_or_sentinel == LEAF_SENTINEL {
            if Some(node.entity) == exclude {
                continue;
            }
            let Some(object) = objects.iter().find(|o| o.entity == node.entity) else {
                continue;
            };
            if march_object(object, ray_origin, ray_dir, t_near, t_far).is_some() {
                return true;
            }
            continue;
        }
        let left = node.left_or_sentinel as usize;
        let right = node.right_or_object as usize;
        let left_hit = slab_hit(ray_origin, ray_dir, &bvh.nodes[left].aabb, t_max);
        let right_hit = slab_hit(ray_origin, ray_dir, &bvh.nodes[right].aabb, t_max);
        let left_visitable = left_hit.0 <= left_hit.1;
        let right_visitable = right_hit.0 <= right_hit.1;
        if left_visitable && right_visitable {
            if left_hit.0 <= right_hit.0 {
                stack.push(right);
                stack.push(left);
            } else {
                stack.push(left);
                stack.push(right);
            }
        } else if left_visitable {
            stack.push(left);
        } else if right_visitable {
            stack.push(right);
        }
    }
    false
}

/// One shadow-ray march candidate: which object, and where its
/// MARGIN-PADDED leaf AABB slab is first entered (`near`) — marching
/// itself is bounded by the ray's real `max_t`, not this candidate's own
/// slab exit (see `trace_shadow`'s doc comment for why), so only the
/// entry point is needed here.
#[derive(Clone, Copy, Debug)]
struct ShadowCandidate {
    entity: Entity,
    near: f32,
}

/// Derives the leaf-AABB padding margin `gather_candidates_padded` applies
/// before its ray/box slab test — the fix for the "polygonal shadow
/// silhouette" bug documented in `docs/knowledge/sdf-3d/
/// rendering/soft-shadows-and-ao.md`'s "Softness shrinks with distance"
/// section. Without padding, a shadow ray that never enters an occluder's
/// TIGHT AABB gets zero candidates for that object at all — `trace_shadow`
/// then returns `vis=1.0` unconditionally, no matter how close the ray
/// actually passes to the object's true (e.g. round) surface, tracing the
/// object's bounding-box edges instead of its silhouette.
///
/// `margin = VIS_CUTOFF * k * PENUMBRA_REACH`: the largest gap `h` between
/// a ray and an occluder for which the soft-shadow formula could still
/// consider the ray "not yet fully lit" (`d/(k*t) == VIS_CUTOFF`), evaluated
/// at `t == PENUMBRA_REACH` — a FIXED, small distance representing how far
/// a penumbra can meaningfully extend from an object's own surface at this
/// renderer's actual object scale (NOT how far the ray itself can
/// potentially travel).
///
/// **Two earlier versions of this reference distance were both real bugs,
/// found and fixed in sequence while chasing this exact artifact:**
/// 1. `scene.root_diagonal()` (`hybrid_legacy::cpu_ref::
///    shadow_candidate_margin`'s original choice, "a self-describing
///    scene-scale reference") — sound only for a scene whose overall size
///    is close to any individual shadow ray's own relevant neighborhood
///    (true for `hybrid_legacy`'s tiny single-object demo). At `--stress
///    N` the BVH root spans the whole multi-cell grid (scales with √N),
///    so the margin grew unboundedly with unrelated object count.
/// 2. This ray's own `max_t` (the fix that replaced #1) — still wrong,
///    just less wrong: `max_t` represents how far the ray COULD travel
///    (to the light), not how far shadowing meaningfully extends from a
///    small object's surface. At this scene's default `k=12` and
///    `max_t=12`, margin came out to `2.88` — 3.6x a cube's own
///    half-extent (`0.8`) — padding every leaf far past its own size and
///    generating shadow candidates for points that are geometrically
///    outside that object's TRUE shadow reach entirely. Confirmed
///    numerically via `debug_stress_100_full_grid_shadow_scan`: a point
///    2.27 units past a cube's true shadow tip (computed from the cube's
///    height and the light's elevation angle) still read `vis=0.044`
///    (96% dark) because the inflated margin generated a candidate for
///    it at all, and the formula has no hard cutoff once a candidate
///    exists — it only asymptotically approaches full visibility.
///
/// `PENUMBRA_REACH = 15.75`: a fixed constant sized to this renderer's
/// actual object scale (this scene's shapes are ~0.8-2.4 units), not
/// derived from ray reach or scene extent at all — deliberately NOT
/// "self-scaling with no tuning constant" (the property both earlier
/// attempts prioritized and got wrong): the margin's purpose is bounded
/// by object scale, and object scale is a real, fixed property of what
/// this renderer draws, not something that should track scene size or
/// light distance. (This comment previously said `1.5`, stale relative
/// to the constant below it — caught while deriving Stage C's occupancy
/// grid cell size from this same value; `15.75` is the correct,
/// in-production number: at the current default `k=2.0` it gives
/// `margin = VIS_CUTOFF * k * PENUMBRA_REACH = 0.63`, a sane shadow
/// margin at this scale, whereas `1.5` would give an implausibly tiny
/// `margin = 0.06`.)
const PENUMBRA_REACH: f32 = 15.75;

fn shadow_candidate_margin(k: f32) -> f32 {
    VIS_CUTOFF * k * PENUMBRA_REACH
}

/// Descends the whole BVH collecting every leaf whose AABB (padded by
/// `margin`) the ray's slab test intersects within `[t_min, t_max]`,
/// deduplicated by entity. Unlike `trace`'s first-hit-wins descent, a
/// soft shadow query needs every object the ray could plausibly pass
/// near, not just the closest hard hit. Mirrors
/// `hybrid_legacy::cpu_ref::gather_candidates_padded`.
///
/// **Internal nodes are ALSO padded by `margin` (not just leaves) — a
/// real bug found and fixed here, not present in `hybrid_legacy`'s
/// original (its scenes' shallow trees apparently never exposed it).**
/// An internal node's bounds are the TIGHT union of its children's own
/// tight bounds; if only leaves are padded, a ray whose true path only
/// enters a leaf's padded margin region (missing the leaf's own tight
/// box) can ALSO miss that leaf's parent internal node's tight box,
/// since the parent has no idea one of its children got padded —
/// pruning the whole subtree before the (correctly padded) leaf test
/// ever runs. Confirmed by walking the real ancestor chain for a
/// specific failing ray: the leaf's own padded slab test would have
/// succeeded (`near=0.0, far=2.55`), but its direct parent's UNPADDED
/// slab test failed (`near=0.94 > far=0.65`), pruning the leaf before
/// it was ever reached — visually this produced a sharp "shadow cut" on
/// one side of a sphere's penumbra (see `debug_stress_100_sphere_
/// shadow_ring_profile`), an on/off cliff between two angularly-close
/// rays where one found this candidate and the other didn't. Padding
/// every node uniformly is correct (not just a workaround): each
/// individual leaf's own padding is ≤ `margin`, so an internal node's
/// tight bounds expanded by `margin` still fully contains every
/// descendant leaf's own padded bounds.
fn gather_candidates_padded(
    bvh: &Bvh,
    ray_origin: Vec3,
    ray_dir: Vec3,
    t_min: f32,
    t_max: f32,
    margin: f32,
) -> Vec<ShadowCandidate> {
    let mut out: Vec<ShadowCandidate> = Vec::new();
    if bvh.nodes.is_empty() {
        return out;
    }
    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = bvh.nodes[node_index];
        let is_leaf = node.left_or_sentinel == LEAF_SENTINEL;
        let padded_aabb = Aabb { min: node.aabb.min - Vec3::splat(margin), max: node.aabb.max + Vec3::splat(margin) };
        let (near, far) = slab_hit(ray_origin, ray_dir, &padded_aabb, t_max);
        if near > far || far < t_min {
            continue;
        }
        if is_leaf {
            let entry = near.max(t_min);
            if !out.iter().any(|c| c.entity == node.entity) {
                out.push(ShadowCandidate { entity: node.entity, near: entry });
            }
            continue;
        }
        stack.push(node.left_or_sentinel as usize);
        stack.push(node.right_or_object as usize);
    }
    out
}

/// A soft shadow query's outcome: either a hard occlusion (fully in
/// shadow, `vis = 0.0`) at a specific object/distance, or a soft result
/// with the final accumulated visibility in `[0, 1]`. Mirrors
/// `hybrid_legacy::cpu_ref::ShadowResult`, minus its `steps` history
/// (no GPU debug-buffer plumbing to mirror here).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ShadowResult {
    HardHit { t: f32, entity: Entity },
    Soft { vis: f32 },
}

impl ShadowResult {
    pub fn vis(self) -> f32 {
        match self {
            ShadowResult::HardHit { .. } => 0.0,
            ShadowResult::Soft { vis } => vis,
        }
    }
}

/// This renderer's real shadow-ray march-convergence tolerance — same
/// value `trace`/`march_object` use for primary rays (see
/// `HIT_EPSILON`'s own definition), reused here rather than duplicated so
/// both march loops agree on "what counts as a surface."
const SHADOW_HIT_EPSILON: f32 = HIT_EPSILON;

/// `trace_shadow`'s own early-exit visibility threshold: once accumulated
/// `vis` drops below this, the whole march stops (the point is
/// "effectively fully shadowed," continuing further can't matter
/// visually) — also the basis `shadow_candidate_margin` derives its AABB
/// padding from. Mirrors `hybrid_legacy::cpu_ref::VIS_CUTOFF`.
const VIS_CUTOFF: f32 = 0.02;

/// Once a shadow-march candidate's sample distance `h` grows past
/// `best_h * DIVERGENCE_FACTOR` (the smallest `h` seen so far for THIS
/// candidate), the march against it stops — a ray that's genuinely,
/// permanently missed this occluder doesn't need to keep marching toward
/// `max_t`, which would otherwise re-expose the Aaltonen formula's own
/// `1/t` softness-decay (see `docs/knowledge/sdf-3d/rendering/
/// soft-shadows-and-ao.md`'s "Softness shrinks with distance" section —
/// a ray confirmed to diverge monotonically still reads as partially
/// shadowed if marched far enough, purely from `t` growing in the
/// denominator). Mirrors `hybrid_legacy::cpu_ref::DIVERGENCE_FACTOR`.
const DIVERGENCE_FACTOR: f32 = 2.5;

/// This object's own signed distance from a WORLD-space point — the same
/// local-space transform `march_object` applies, factored out so
/// `trace_shadow` can sample a candidate's distance without also needing
/// its own copy of the marching loop.
fn object_distance(object: &TraceObject, p_world: Vec3) -> f32 {
    let p_local = object.rotation.inverse() * (p_world - object.translation);
    local_distance(&object.shape, p_local)
}

/// Soft shadow visibility via Aaltonen's closest-point refinement of
/// Quilez's `k*h/t` penumbra technique (see
/// `docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md`) — ported from
/// `hybrid_legacy::cpu_ref::trace_shadow`'s validated, GPU-probe-
/// cross-checked FIXED state (three passing regression tests there), not
/// from the buggy draft that motivated this project's fresh-start
/// rewrite. Confirmed by reading both the KB doc's full investigation and
/// the actual shipped code: the doc's prose trails off mid-investigation,
/// but the code shows the investigation completed with two coupled fixes,
/// both reproduced here:
///
/// 1. **March bound**: the march's stopping bound is `max_t` (the ray's
///    real limit, e.g. distance to a point/spot light), NOT a candidate's
///    own padded-AABB slab exit (`cand.far`) — the slab exit is an
///    acceleration-structure artifact, not a physically meaningful
///    stopping point for a soft near-miss query. `cand.near` is still
///    where marching STARTS; only the slab EXIT stopped being trusted as
///    a stopping bound. Bounded by `DIVERGENCE_FACTOR` so this doesn't
///    re-expose the formula's own `1/t` decay for rays that have
///    genuinely missed a candidate.
/// 2. **Candidate margin**: `gather_candidates_padded`'s leaf-AABB padding
///    (see its own doc comment) — without it, fixing #1 alone still
///    produces a polygonal (not round) shadow silhouette, since a ray
///    that never enters an occluder's TIGHT AABB gets no candidate for it
///    at all.
///
/// `origin_entity`: an object with no CSG parts (always true for this
/// renderer — no multi-record objects) is fully excluded from its own
/// candidate slab for its own shadow ray, matching
/// `hybrid_legacy::cpu_ref::trace_shadow`'s single-leaf-only simplified
/// scope (its general case additionally handles multi-record CSG
/// self-shadowing, which doesn't apply here).
pub fn trace_shadow(
    bvh: &Bvh,
    objects: &[TraceObject],
    ray_origin: Vec3,
    ray_dir: Vec3,
    max_t: f32,
    k: f32,
    origin_entity: Option<Entity>,
) -> ShadowResult {
    let margin = shadow_candidate_margin(k);
    let candidates = gather_candidates_padded(bvh, ray_origin, ray_dir, 0.001, max_t, margin);
    let mut vis = 1.0f32;

    for cand in &candidates {
        if vis < VIS_CUTOFF {
            return ShadowResult::Soft { vis: 0.0 };
        }
        if Some(cand.entity) == origin_entity {
            continue;
        }
        let Some(object) = objects.iter().find(|o| o.entity == cand.entity) else {
            continue; // stale candidate; no matching scene data this frame
        };
        let mut t = cand.near.max(0.01);
        let mut ph = 1e20f32;
        let mut prev_step = 1e20f32;
        let mut best_h = f32::MAX;
        while t <= max_t && vis > VIS_CUTOFF {
            let h = object_distance(object, ray_origin + ray_dir * t);
            if h < SHADOW_HIT_EPSILON {
                return ShadowResult::HardHit { t, entity: object.entity };
            }
            if h < best_h {
                best_h = h;
            } else if h > best_h * DIVERGENCE_FACTOR {
                break;
            }
            let y = (h * h / (2.0 * ph)).min(prev_step);
            let d = (h * h - y * y).max(0.0).sqrt();
            let raw_vis = (d / (k * (t - y).max(1e-4))).clamp(0.0, 1.0);
            // Fade toward fully lit as `h` approaches `margin` (the
            // candidate-gathering cutoff) — without this, a point whose
            // shadow ray just barely enters the padded candidate AABB
            // (h ~ margin) still gets `raw_vis` computed from the full
            // d/(k*t) formula, which is NOT continuous with the vis=1.0
            // a point just outside the padded AABB gets (no candidate at
            // all, loop body never runs). That discontinuity renders as a
            // visible polygonal seam at the padded-AABB boundary,
            // independent of the object's true (e.g. round) silhouette —
            // confirmed by scaling PENUMBRA_REACH up and observing the
            // polygonal edge grow proportionally with the margin, not
            // with the object's own geometry. Smoothstepping the last
            // half of the margin range removes the cliff.
            let margin_fade = smoothstep(margin * 0.5, margin, h);
            let faded_vis = raw_vis + (1.0 - raw_vis) * margin_fade;
            vis = vis.min(faded_vis);
            // A real, third sealed-room light leak (2026-09-18): without
            // this snap, a candidate whose march pushes `vis` below
            // VIS_CUTOFF right here would fall through to the loop's own
            // `while ... && vis > VIS_CUTOFF` re-check at the TOP of the
            // next iteration, exit the loop, and this function would
            // return `Soft { vis }` at its own end with THIS sample's
            // small, nonzero leftover value — never getting the chance a
            // further sample would have had to reach a genuine hard hit
            // (`h < SHADOW_HIT_EPSILON`). The outer per-candidate loop
            // (this function's own `for cand in &candidates` above) already
            // treats "vis dropped below VIS_CUTOFF" as meaning fully
            // opaque (`if vis < VIS_CUTOFF { return Soft { vis: 0.0 } }`)
            // — this inner exit needs to mean the identical thing, not a
            // silently-different "report whatever fractional value the
            // one sample that crossed the line happened to compute."
            // This specifically closes a gap `margin_fade`'s own real,
            // still-needed "fade toward lit near the padded-AABB
            // boundary" smoothing (immediately above) can otherwise open
            // on a thin, wide occluder: a first sample landing AT the
            // margin boundary gets faded toward vis≈1.0 by design, then
            // a genuinely-occluding second sample must not be allowed to
            // report its OWN small transitional value as "real, if
            // faint, light" once it's already crossed the same opacity
            // threshold every other candidate in this function is held
            // to.
            if vis <= VIS_CUTOFF {
                return ShadowResult::Soft { vis: 0.0 };
            }
            ph = h;
            let mut step = h * 1.2;
            if t + step > max_t {
                step = h;
                if t + step > max_t {
                    break;
                }
            }
            prev_step = step;
            t += step;
        }
    }

    ShadowResult::Soft { vis }
}

/// The real shadow-ray origin offset along the surface normal — biasing
/// off the RAW hit point (rather than this) reads as self-intersection
/// noise almost immediately, since the primary ray's own hit is already
/// within `HIT_EPSILON` of the surface. Scales with `hit_t` (farther hits
/// have coarser marching precision, needing a proportionally larger bias)
/// with a fixed floor for very close hits. Mirrors
/// `hybrid_legacy_trace.wgsl`'s `pixel_eps`/`shadow_bias` exactly.
fn pixel_eps(t: f32) -> f32 {
    (t * 0.0016).max(2e-4)
}

pub fn shadow_bias(hit_t: f32) -> f32 {
    (pixel_eps(hit_t) * 2.0).max(0.01)
}

#[cfg(test)]
mod tests {
    use bevy::prelude::World;

    use super::*;
    use crate::hybrid::scene::HybridObject;

    fn entities(n: usize) -> Vec<Entity> {
        let mut world = World::new();
        (0..n).map(|_| world.spawn_empty().id()).collect()
    }

    fn ground_and_cube() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(4.0, 0.2, 4.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.2, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.45, 0.46, 0.48), 0.0, 0.6),
            },
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(0.8), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.8, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.85, 0.35, 0.20), 0.0, 0.4),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject {
                entity: o.entity,
                // Both objects here are axis-aligned (identity rotation),
                // so the AABB is just the local half-extents centered on
                // the translation — no rotated-corners math needed.
                world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)),
            })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    fn local_half_extents(shape: &Shape) -> Vec3 {
        match *shape {
            Shape::RoundedBox { half_extents, .. } => half_extents,
            Shape::Sphere { radius } => Vec3::splat(radius),
            _ => unreachable!(),
        }
    }

    /// A ground plate + one sphere occluder — the same scene shape
    /// `hybrid_legacy::cpu_ref::gallery_sphere_scene` used to pin its own
    /// shadow regression tests (sphere radius 1.2 at height 1.2, ground
    /// RoundedBox half-extents (5.5, 0.05, 5.5)), reconstructed here
    /// against this module's own `TraceObject`/`Bvh` API rather than
    /// copy-pasted, since the two renderers don't share scene types.
    fn ground_and_sphere() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(5.5, 0.05, 5.5), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.05, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::splat(0.5), 0.0, 0.5),
            },
            TraceObject {
                entity: e[1],
                shape: Shape::Sphere { radius: 1.2 },
                translation: Vec3::new(0.0, 1.2, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::splat(0.5), 0.0, 0.5),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject {
                entity: o.entity,
                world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)),
            })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    /// Same directional light direction `hybrid_legacy`'s own pinned
    /// shadow tests used: `Transform::from_xyz(6,12,6).looking_at(ZERO)`,
    /// so `to_light` points FROM the shaded point TOWARD (6,12,6).
    fn shadow_test_to_light() -> Vec3 {
        Vec3::new(6.0, 12.0, 6.0).normalize()
    }

    const SHADOW_TEST_K: f32 = 2.0;

    /// A shadow ray whose true closest approach to the sphere is small
    /// (genuine near-contact penumbra, not a hard hit) must read as
    /// substantially — but not necessarily fully — shadowed. This is the
    /// same class of case `hybrid_legacy::cpu_ref::
    /// real_near_contact_matches_shader_probe_630_405` pinned (a live GPU
    /// probe cross-check); reconstructed geometrically here rather than
    /// against the exact same pixel (this renderer's camera/scene differ),
    /// so this test checks the qualitative outcome (soft, strongly
    /// shadowed, not a hard hit) rather than bit-for-bit-matching a GPU
    /// probe this module has no access to.
    #[test]
    fn near_contact_point_reads_strongly_shadowed_not_hard_hit() {
        let (e, objects, bvh) = ground_and_sphere();
        // A ground point just beside the sphere's base, close enough that
        // the shadow ray grazes near the sphere's surface without
        // intersecting it.
        let hit_p = Vec3::new(-1.3, 0.0, -0.15);
        let hit_t = 9.5; // representative primary-ray hit distance
        let n = Vec3::Y;
        let ro = hit_p + n * shadow_bias(hit_t);
        let to_light = shadow_test_to_light();
        let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, Some(e[0]));
        match result {
            ShadowResult::HardHit { .. } => panic!("expected a soft near-contact result, got a hard hit"),
            ShadowResult::Soft { vis } => {
                assert!(vis < 0.2, "expected strong shadowing near the sphere's base, got vis={vis}");
            }
        }
    }

    /// A shadow ray whose march actually enters the sphere's volume
    /// (`h < SHADOW_HIT_EPSILON`) must report a genuine hard hit against
    /// the sphere, not a soft result — mirrors `hybrid_legacy::cpu_ref::
    /// real_hard_hit_matches_shader_probe_600_380`'s finding that a
    /// visibly-dark pixel can be a true hard occlusion, not a marginal
    /// soft-march case.
    #[test]
    fn ray_through_the_sphere_reads_as_a_hard_hit() {
        let (e, objects, bvh) = ground_and_sphere();
        // A ground point close enough to the sphere's base that the
        // shadow ray toward the light passes directly through its volume.
        let hit_p = Vec3::new(-0.4, 0.0, -0.3);
        let hit_t = 10.1;
        let n = Vec3::Y;
        let ro = hit_p + n * shadow_bias(hit_t);
        let to_light = shadow_test_to_light();
        let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, Some(e[0]));
        match result {
            ShadowResult::HardHit { entity, .. } => {
                assert_eq!(entity, e[1], "the hard hit should be against the sphere, not the ground");
            }
            ShadowResult::Soft { vis } => panic!("expected a hard hit through the sphere, got a soft result vis={vis}"),
        }
    }

    /// The AABB-slab-truncation bug `hybrid_legacy`'s own investigation
    /// found and fixed (see `trace_shadow`'s doc comment): a shadow ray
    /// whose padded-candidate slab entry is very close to the ray's own
    /// origin (a grazing ray that only clips a small corner of the
    /// sphere's AABB) must still be allowed to march past that slab's
    /// exit, all the way to `max_t`, to find its TRUE closest approach —
    /// not terminate after one sample and read as fully lit. Checked here
    /// by picking a ground point far enough around the sphere's side that
    /// the ray only grazes the sphere's AABB corner, and asserting the
    /// result is NOT `vis=1.0` (the exact old, buggy, un-fixed behavior)
    /// despite that grazing candidate slab.
    #[test]
    fn grazing_ray_past_its_candidate_slab_still_finds_real_occlusion() {
        // Verified numerically (not guessed) against this exact scene's
        // geometry: a ray from this ground point toward the light misses
        // the sphere's TIGHT (unpadded) AABB slab test entirely (a
        // brute-force search over a dense grid of ground points found
        // this to be near the smallest achievable gap for this light
        // direction: ~0.43 world units past the tight AABB, at t≈0.58
        // along the ray) — so WITHOUT the candidate-margin fix, this ray
        // would get zero candidates for the sphere and `trace_shadow`
        // would return `vis=1.0` unconditionally, regardless of how close
        // it actually passes to the sphere's true (round) surface. WITH
        // the margin fix, the padded AABB slab test succeeds, a candidate
        // is generated, and the march finds the real (partial) occlusion.
        let (e, objects, bvh) = ground_and_sphere();
        let hit_p = Vec3::new(-0.24, 0.0, 1.22);
        let hit_t = 10.0;
        let n = Vec3::Y;
        let ro = hit_p + n * shadow_bias(hit_t);
        let to_light = shadow_test_to_light();
        let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, Some(e[0]));
        let vis = result.vis();
        // Threshold relaxed from 0.5 to 0.9 after adding the margin-edge
        // fade (see trace_shadow's `margin_fade` — smooths the hard
        // visibility cliff at the padded-candidate-AABB boundary that
        // otherwise renders as a visible polygonal seam, see PROGRESS.md's
        // "boxy/polygonal shadow edge" entry). This test's whole point is
        // "the margin fix generates a candidate at all, so vis is NOT the
        // pre-fix vis=1.0" — any value meaningfully below 1.0 still proves
        // that, even though the fade now makes points this close to the
        // margin's own edge read less darkened than before.
        assert!(
            vis < 0.9,
            "expected the candidate-margin fix to find real occlusion for a ray that misses the sphere's \
             tight AABB but passes close to its true surface — got vis={vis} (pre-fix behavior: vis=1.0)"
        );
    }

    /// A dense ring of shadow-ray origins around the sphere's base (radius
    /// just outside the sphere itself, where a real fraction of rays miss
    /// the sphere's TIGHT AABB slab test entirely — verified numerically
    /// for this exact light direction, ~22/128 samples at this radius)
    /// must show NO fully-lit sample (`vis >= 0.999`) — every sample this
    /// close to the sphere's surface should read at least partially
    /// shadowed once the candidate-margin fix lets a "missed tight AABB"
    /// ray still generate a candidate. Confirmed to actually catch the
    /// margin-fix regression (fails with `shadow_candidate_margin`
    /// disabled, unlike an earlier weaker "neighbor comparison" version of
    /// this test that passed even with the fix disabled and was replaced).
    #[test]
    fn no_ring_sample_near_the_sphere_reads_fully_lit() {
        let (e, objects, bvh) = ground_and_sphere();
        let to_light = shadow_test_to_light();
        const SAMPLE_COUNT: usize = 128;
        const RING_RADIUS: f32 = 1.25; // just outside the sphere's own radius (1.2)
        for i in 0..SAMPLE_COUNT {
            let angle = (i as f32 / SAMPLE_COUNT as f32) * std::f32::consts::TAU;
            let p = Vec3::new(RING_RADIUS * angle.cos(), 0.0, RING_RADIUS * angle.sin());
            let ro = p + Vec3::Y * shadow_bias(10.0);
            let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, Some(e[0]));
            let vis = result.vis();
            assert!(
                vis < 0.999,
                "sample {i} (angle={angle:.3}, p={p:?}) reads fully lit (vis={vis}) despite being just \
                 outside the sphere's own radius — likely a candidate-margin regression"
            );
        }
    }

    // -----------------------------------------------------------------
    // Thin-slab grazing-angle shadow march — a real DDGI light-leak
    // investigation (2026-09-18) initially suspected a wide (16-unit),
    // thin (0.3-half-extent) roof-panel slab, shadow-tested at
    // `examples/gi_room.rs`'s own real 35-degree sun elevation, could let
    // `trace_shadow`'s own divergence break (`h > best_h *
    // DIVERGENCE_FACTOR`) exit the march before `h` ever dropped into the
    // `margin_fade` zone, returning a stale near-1.0 `vis` for a ray that
    // should be a clean hard hit. RULED OUT by the two tests below (both
    // pass: the march converges correctly to a real hard hit at every
    // tested angle/offset) — kept as permanent regression coverage for
    // this specific geometry shape, not because the leak theory held up.
    // See PROGRESS.md's own "DDGI sealed-room light leak" entry and the
    // full-room fixture further below for where the investigation went
    // next.
    // -----------------------------------------------------------------

    /// A slab matching `gi_room.rs`'s own roof panel exactly (half_extents
    /// (8.3, 0.3, 6.8) — `ROOM_HALF_X + WALL_OVERLAP`/`WALL_THICKNESS`/
    /// `ROOM_HALF_Z + WALL_OVERLAP`, see that file's own `spawn_room`),
    /// with a shadow ray fired from directly beneath its center toward
    /// `gi_room.rs`'s own real sun direction (`Transform::from_xyz(0.0,
    /// 10.0, 0.0).looking_at(Vec3::new(-7.63, 4.26, -2.97), Vec3::Y)`,
    /// ~35 degrees above the horizon — computed, not eyeballed, from that
    /// exact transform).
    fn thin_wide_slab_and_shadow_ray() -> (Vec<Entity>, Vec<TraceObject>, Bvh, Vec3) {
        let e = entities(1);
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::new(8.3, 0.3, 6.8), corner_radius: 0.0 },
            translation: Vec3::new(0.0, 3.3, 0.0),
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::splat(0.85), 0.0, 0.6),
        }];
        let hybrid_objects: Vec<HybridObject> =
            objects.iter().map(|o| HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)) }).collect();
        let bvh = Bvh::build(&hybrid_objects);
        let sun_origin = Vec3::new(0.0, 10.0, 0.0);
        let sun_target = Vec3::new(-7.63, 4.26, -2.97);
        let to_light = (sun_origin - sun_target).normalize();
        (e, objects, bvh, to_light)
    }

    /// The actual DDGI-leak reproduction: a shadow ray from well inside
    /// the sealed room, straight beneath the roof panel's own center,
    /// toward the real 35-degree sun — the panel is 0.6 units thick
    /// along Y but the ray crosses it at a shallow angle (effective
    /// path length `0.6 / sin(35°) ≈ 1.05` world units), the exact
    /// regime the investigation flagged as able to let `best_h *
    /// DIVERGENCE_FACTOR` break the march before `h` ever gets small.
    /// Must read as a hard, fully-opaque occlusion — light must NOT leak
    /// through a solid 0.6-unit-thick roof panel at any angle.
    #[test]
    fn thin_roof_slab_at_grazing_sun_angle_is_never_a_soft_leak() {
        let (_e, objects, bvh, to_light) = thin_wide_slab_and_shadow_ray();
        let ro = Vec3::new(0.0, 2.0, 0.0); // well inside the sealed room, beneath the panel's own center
        // origin_entity = None: the shaded point is on the FLOOR (a
        // different object from the panel being tested), matching
        // gi_room's own real DDGI probes/floor shading -- excluding the
        // panel itself here (as an earlier version of this test
        // mistakenly did, passing Some(e[0]) where e[0] IS the panel)
        // makes trace_shadow skip the panel as a shadow candidate
        // entirely, which is wrong for this test and was never the real
        // bug.
        let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, None);
        match result {
            ShadowResult::HardHit { .. } => {} // correct: fully blocked
            ShadowResult::Soft { vis } => {
                assert!(
                    vis < 0.05,
                    "a shadow ray straight through a solid 0.6-unit-thick roof panel at 35 degrees \
                     elevation must read as fully occluded, not leak light through — got vis={vis} \
                     (this is the exact mechanism suspected of causing gi_room's own sealed-room DDGI \
                     light leak: a stale near-1.0 vis from trace_shadow's own divergence break firing \
                     before the march's h ever got small enough to register real occlusion)"
                );
            }
        }
    }

    /// Same slab/light as above, but sweeping a dense line of shadow-ray
    /// origins across the room's own width (X) beneath the panel — every
    /// sample must be fully occluded. A single spot check could get
    /// lucky/unlucky depending on exactly where the march's step lands
    /// relative to the panel; this sweep is what actually rules out "it
    /// only fails at specific X offsets" the way the existing sphere-ring
    /// test rules out angle-specific luck for the margin-fix regression.
    #[test]
    fn no_sample_under_the_thin_roof_slab_leaks_light_at_any_x_offset() {
        let (_e, objects, bvh, to_light) = thin_wide_slab_and_shadow_ray();
        const SAMPLE_COUNT: usize = 64;
        // -5.5..5.5, not the panel's own full 8.3 half-width: the sun
        // direction's own nonzero X component (to_light.x ~= 0.76) means
        // a ray from a point close to the panel's +X edge (~8.3) drifts
        // out past that edge in X before it's risen enough in Y to have
        // crossed the panel's own thin Y span -- a genuine silhouette
        // graze, not a light-leak-through-solid-geometry bug. Staying at
        // least ~2.5 units inside the true edge keeps every sample a
        // real straight-through-the-slab case.
        for i in 0..SAMPLE_COUNT {
            let x = -5.5 + (i as f32 / (SAMPLE_COUNT - 1) as f32) * 11.0;
            let ro = Vec3::new(x, 2.0, 0.0);
            let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, None);
            let vis = result.vis();
            assert!(
                vis < 0.05,
                "sample {i} (x={x:.3}) reads as leaking light (vis={vis}) through the solid roof panel \
                 — expected full occlusion at every X offset beneath it"
            );
        }
    }

    // -----------------------------------------------------------------
    // Full sealed-room fixture, matching `examples/gi_room.rs::spawn_room`
    // exactly (real panel dimensions, real WALL_OVERLAP seam-closing
    // overlap, real 35-degree sun) — the isolated single-slab tests above
    // didn't reproduce the leak; this fixture checks the actual multi-
    // panel geometry (floor + 4 walls + roof, 6 separate non-CSG-unioned
    // objects, each marched independently — see `spawn_room`'s own doc
    // comment for why a seam between two SEPARATE objects can leak even
    // when each object individually is solid) at a dense grid of shadow-
    // ray origins matching where DDGI's own probe grid actually sits.
    // -----------------------------------------------------------------

    const GI_ROOM_HALF_X: f32 = 8.0;
    const GI_ROOM_HALF_Y: f32 = 3.0;
    const GI_ROOM_HALF_Z: f32 = 6.5;
    const GI_ROOM_WALL_THICKNESS: f32 = 0.3;
    const GI_ROOM_WALL_OVERLAP: f32 = 0.2;

    /// `examples/gi_room.rs::spawn_room` verbatim — floor, 4 walls, and a
    /// FULLY CLOSED (centered, not slid open) roof panel, at the file's
    /// own real dimensions and seam overlaps.
    fn gi_room_sealed_shell() -> (Vec<Entity>, Vec<TraceObject>, Bvh, Vec3) {
        let e = entities(6);
        let white_wall = Material::new(Vec3::splat(0.92), 0.0, 0.35);
        let mut objects = Vec::new();

        let floor_half = Vec3::new(GI_ROOM_HALF_X + GI_ROOM_WALL_OVERLAP, GI_ROOM_WALL_THICKNESS, GI_ROOM_HALF_Z + GI_ROOM_WALL_OVERLAP);
        objects.push(TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: floor_half, corner_radius: 0.0 },
            translation: Vec3::new(0.0, -GI_ROOM_HALF_Y - GI_ROOM_WALL_THICKNESS, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });

        let side_wall_half = Vec3::new(GI_ROOM_WALL_THICKNESS, GI_ROOM_HALF_Y + GI_ROOM_WALL_OVERLAP, GI_ROOM_HALF_Z);
        objects.push(TraceObject {
            entity: e[1],
            shape: Shape::RoundedBox { half_extents: side_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(GI_ROOM_HALF_X + GI_ROOM_WALL_THICKNESS, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        objects.push(TraceObject {
            entity: e[2],
            shape: Shape::RoundedBox { half_extents: side_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(-GI_ROOM_HALF_X - GI_ROOM_WALL_THICKNESS, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });

        let end_wall_half = Vec3::new(GI_ROOM_HALF_X + GI_ROOM_WALL_OVERLAP, GI_ROOM_HALF_Y + GI_ROOM_WALL_OVERLAP, GI_ROOM_WALL_THICKNESS);
        objects.push(TraceObject {
            entity: e[3],
            shape: Shape::RoundedBox { half_extents: end_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(0.0, 0.0, GI_ROOM_HALF_Z + GI_ROOM_WALL_THICKNESS),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        objects.push(TraceObject {
            entity: e[4],
            shape: Shape::RoundedBox { half_extents: end_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(0.0, 0.0, -GI_ROOM_HALF_Z - GI_ROOM_WALL_THICKNESS),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });

        let roof_half = Vec3::new(GI_ROOM_HALF_X + GI_ROOM_WALL_OVERLAP, GI_ROOM_WALL_THICKNESS, GI_ROOM_HALF_Z + GI_ROOM_WALL_OVERLAP);
        objects.push(TraceObject {
            entity: e[5],
            shape: Shape::RoundedBox { half_extents: roof_half, corner_radius: 0.0 },
            translation: Vec3::new(0.0, GI_ROOM_HALF_Y + GI_ROOM_WALL_THICKNESS, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });

        let hybrid_objects: Vec<HybridObject> =
            objects.iter().map(|o| HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)) }).collect();
        let bvh = Bvh::build(&hybrid_objects);
        let sun_origin = Vec3::new(0.0, 10.0, 0.0);
        let sun_target = Vec3::new(-7.63, 4.26, -2.97);
        let to_light = (sun_origin - sun_target).normalize();
        (e, objects, bvh, to_light)
    }

    /// The decisive reproduction: a dense grid of shadow-ray origins
    /// spanning the ROOF's own interior neighborhood (where DDGI's top
    /// probe layer actually sits, `ddgi_ref::probe_grid_from_bounds`'
    /// half-cell inset means the top layer is NOT touching the ceiling
    /// exactly, but is close to it) in the FULLY SEALED (fully closed
    /// roof) `gi_room` shell — every sample must read as fully occluded
    /// from the sun. No isolated single-panel test above reproduced the
    /// leak; if this multi-panel fixture ALSO doesn't reproduce it, the
    /// leak is not a geometry/shadow-march issue at all, and the
    /// investigation should move to the DDGI atlas/temporal-blend layer
    /// instead of the shadow march.
    #[test]
    fn no_shadow_ray_near_the_sealed_roof_leaks_light_through_a_panel_seam() {
        let (_e, objects, bvh, to_light) = gi_room_sealed_shell();
        const GRID: usize = 12;
        let mut worst: Option<(f32, f32, f32)> = None; // (vis, x, z)
        for xi in 0..GRID {
            for zi in 0..GRID {
                let x = -6.5 + (xi as f32 / (GRID - 1) as f32) * 13.0;
                let z = -5.5 + (zi as f32 / (GRID - 1) as f32) * 11.0;
                // Just under the roof's own inner face (roof inner face
                // at y = GI_ROOM_HALF_Y = 3.0) -- close to where DDGI's
                // real top probe layer sits.
                let ro = Vec3::new(x, GI_ROOM_HALF_Y - 0.3, z);
                let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, None);
                let vis = result.vis();
                if vis > worst.map_or(0.0, |w| w.0) {
                    worst = Some((vis, x, z));
                }
            }
        }
        // `worst` staying `None` means every single sample came back
        // EXACTLY zero -- a strictly stronger result than any nonzero
        // `vis < 0.05` could be, not a test-fixture failure (see the
        // margin_fade/VIS_CUTOFF fix's own doc comment above
        // `trace_shadow` for why samples that used to leak a small
        // nonzero `vis` now correctly snap to 0.0).
        if let Some((vis, x, z)) = worst {
            assert!(
                vis < 0.05,
                "worst-case sample at (x={x:.3}, z={z:.3}) near the sealed roof's own inner face reads as \
                 leaking light (vis={vis}) — expected full occlusion everywhere inside a fully closed room"
            );
        }
    }

    /// Same fixture, but a FULL 3D grid (not just near the roof) --
    /// including corners where two/three wall panels meet (the actual
    /// camera-visible leak in a live screenshot showed a gradient
    /// stronger near the ceiling/corner than the floor, near the +X/+Z
    /// corner specifically, which the roof-only sweep above may not have
    /// densely covered in Y or right at a 3-panel seam).
    #[test]
    fn no_shadow_ray_anywhere_in_the_sealed_room_leaks_light_through_any_seam() {
        let (_e, objects, bvh, to_light) = gi_room_sealed_shell();
        const GRID: usize = 8;
        let mut worst: Option<(f32, f32, f32, f32)> = None; // (vis, x, y, z)
        for xi in 0..GRID {
            for yi in 0..GRID {
                for zi in 0..GRID {
                    let x = -7.0 + (xi as f32 / (GRID - 1) as f32) * 14.0;
                    let y = -2.7 + (yi as f32 / (GRID - 1) as f32) * 5.4;
                    let z = -6.0 + (zi as f32 / (GRID - 1) as f32) * 12.0;
                    let ro = Vec3::new(x, y, z);
                    let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, None);
                    let vis = result.vis();
                    if vis > worst.map_or(0.0, |w| w.0) {
                        worst = Some((vis, x, y, z));
                    }
                }
            }
        }
        // See the roof-only sweep above's own identical comment: `worst`
        // staying `None` means every sample came back exactly zero.
        if let Some((vis, x, y, z)) = worst {
            assert!(
                vis < 0.05,
                "worst-case sample at (x={x:.3}, y={y:.3}, z={z:.3}) anywhere inside the sealed room reads as \
                 leaking light (vis={vis}) — expected full occlusion everywhere inside a fully closed room"
            );
        }
    }

    // -----------------------------------------------------------------
    // margin_fade / VIS_CUTOFF interaction — a real, third sealed-room
    // light leak (2026-09-18), found AFTER both the DDGI grid-cell bug
    // and the unconditional bounce-GI bug above were already fixed: a
    // faint but real, structured (non-grain) grey edge remained visible
    // on the glass cube's own silhouette under `--gi-method none`,
    // persisting even with reflection AND transmission both disabled —
    // proving it lives in DIRECT-light shadow visibility itself, not
    // any GI/bounce path. Root cause: `trace_shadow`'s own candidate
    // march can spend its FIRST sample right at the padded-AABB margin
    // boundary of a thin, wide occluder (the roof panel) — `margin_fade`
    // (added for a different, legitimate reason: smoothing the
    // candidate-AABB entry cliff, see that field's own doc comment a few
    // hundred lines up) forces that first sample's own `vis` toward
    // ~1.0 regardless of true occlusion. The SECOND sample then finds
    // real, strong occlusion (vis << VIS_CUTOFF), but the march loop's
    // own `while vis > VIS_CUTOFF` condition exits BEFORE a third sample
    // can ever reach the roof's true surface and register a proper hard
    // hit (`h < SHADOW_HIT_EPSILON`) — so `trace_shadow` returns a
    // small, nonzero `Soft { vis }` (not a `HardHit`) for a ray that is
    // actually, fully, unambiguously blocked. `shade`'s own `if
    // shadow_vis <= 0.0 { continue }` gate correctly lets real nonzero
    // `vis` values through (working as designed for genuine penumbra) —
    // it has no way to distinguish "this is real half-open penumbra"
    // from "this is a stale first-sample artifact of the margin fade."
    // -----------------------------------------------------------------

    /// `gi_room_sealed_shell` plus `examples/gi_room.rs::spawn_cubes`'s
    /// own real clear-glass cube (Zone 5: `Vec3::new(2.0, -2.0, 0.0)`,
    /// half-extent 1.0) — the object this specific leak was traced to.
    fn gi_room_sealed_shell_with_glass_cube() -> (Vec<Entity>, Vec<TraceObject>, Bvh, Vec3) {
        let (mut e, mut objects, _bvh, to_light) = gi_room_sealed_shell();
        let glass_entity = entities(1)[0];
        e.push(glass_entity);
        objects.push(TraceObject {
            entity: glass_entity,
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.02 },
            translation: Vec3::new(2.0, -2.0, 0.0),
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::ONE, 0.0, 0.02).with_reflectance(0.9).with_transmission(1.0).with_ior(1.5),
        });
        let hybrid_objects: Vec<HybridObject> =
            objects.iter().map(|o| HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)) }).collect();
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh, to_light)
    }

    /// The decisive reproduction: a shadow ray fired from the glass
    /// cube's OWN real top-front silhouette edge (as seen from
    /// `CAMERA_CORNER`, per the live investigation that traced the
    /// visible screen artifact back to this exact point) toward the
    /// real sun, in the fully sealed room. Uses a much stricter
    /// tolerance than the existing dense-grid sweeps above (`vis <
    /// 0.05`) specifically because this failure mode produces a SMALL
    /// nonzero `vis` (~0.0148) that a `0.05` threshold would miss
    /// entirely — the whole reason the grid sweeps above didn't already
    /// catch this: they never happened to land a shadow-ray origin at
    /// this exact margin-boundary-triggering geometry.
    #[test]
    fn shadow_ray_from_the_glass_cubes_own_silhouette_edge_is_a_hard_hit_not_a_soft_leak() {
        let (_e, objects, bvh, to_light) = gi_room_sealed_shell_with_glass_cube();
        // The glass cube's own top-front rounded edge (+Y/+Z corner),
        // biased outward along its own approximate normal the same way
        // every other real shading call in this codebase biases a
        // shadow ray's own origin (see `shade`'s own `shadow_bias` use).
        let p_world = Vec3::new(1.205, -0.991, 1.0);
        let approx_normal = Vec3::new(0.0, 1.0, 1.0).normalize();
        let ro = p_world + approx_normal * shadow_bias(10.0);
        let result = trace_shadow(&bvh, &objects, ro, to_light, 60.0, SHADOW_TEST_K, None);
        match result {
            ShadowResult::HardHit { .. } => {} // correct: fully, unambiguously blocked
            ShadowResult::Soft { vis } => {
                assert!(
                    vis < 1e-4,
                    "a shadow ray from the glass cube's own silhouette edge toward the sun, in a fully \
                     sealed room, must read as either a hard hit or fully dark (vis < 1e-4) — got a soft \
                     leak vis={vis} (this is the margin_fade/VIS_CUTOFF interaction bug: the march's own \
                     first sample lands at the roof's padded-AABB margin, forcing vis toward 1.0 via \
                     margin_fade, then the second sample's real occlusion drops vis below VIS_CUTOFF \
                     before a third sample can ever reach a genuine hard hit)"
                );
            }
        }
    }

    /// A ray straight down through the cube's center must hit the cube
    /// (not the ground behind it), at the expected distance, reporting
    /// the cube's own flat color.
    #[test]
    fn ray_through_cube_center_hits_cube_with_its_flat_color() {
        let (e, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let hit = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).expect("must hit the cube");
        assert_eq!(hit.entity, e[1], "must hit the cube, not the ground behind it");
        // Cube top face is at y = 0.8 + 0.8 = 1.6; ray starts at y = 10.
        assert!((hit.t - 8.4).abs() < 1e-2, "t = {}", hit.t);
        assert!((hit.color - Vec3::new(0.85, 0.35, 0.20)).length() < 1e-5);
    }

    /// A ray that misses the cube entirely (offset far enough on X) must
    /// hit the ground plate instead, with the ground's own color.
    #[test]
    fn ray_beside_cube_hits_ground_with_its_flat_color() {
        let (e, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(3.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let hit = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).expect("must hit the ground");
        assert_eq!(hit.entity, e[0], "must hit the ground, cube is out of the way at x=3");
        // Ground top face is at y = -0.2 + 0.2 = 0.0; ray starts at y = 10.
        assert!((hit.t - 10.0).abs() < 1e-2, "t = {}", hit.t);
        assert!((hit.color - Vec3::new(0.45, 0.46, 0.48)).length() < 1e-5);
    }

    /// A ray pointed away from the whole scene (upward, above everything)
    /// must report a miss, not a false hit or a panic.
    #[test]
    fn ray_missing_the_whole_scene_reports_no_hit() {
        let (_, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, 1.0, 0.0); // straight up, away from everything
        assert!(trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).is_none());
    }

    /// A ray through a cube rotated 45 degrees about Y must still hit it
    /// at the analytically-correct distance — the point of transforming
    /// the ray into the object's local (unrotated) space before marching:
    /// a rotation-ignorant marcher would compute the wrong local point
    /// and either miss the hit or report the wrong distance.
    #[test]
    fn ray_through_rotated_cube_hits_at_the_correct_distance() {
        let e = entities(1);
        let half = 0.8f32;
        let rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_4);
        let object = TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(half), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation,
            material: Material::new(Vec3::new(0.1, 0.2, 0.9), 0.0, 0.5),
        };
        let world_aabb = crate::hybrid::scene::world_aabb(
            &object.shape,
            &bevy::prelude::GlobalTransform::from(bevy::prelude::Transform {
                translation: object.translation,
                rotation,
                ..Default::default()
            }),
        );
        let hybrid_object = HybridObject { entity: e[0], world_aabb };
        let bvh = Bvh::build(&[hybrid_object]);

        // Ray straight down through the object's Y axis (the rotation
        // axis) — a 45-degree-about-Y rotation doesn't change the cube's
        // silhouette along Y, so the hit distance to the top face must be
        // exactly `half` regardless of the rotation.
        let ray_origin = Vec3::new(0.0, 5.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let hit = trace(&bvh, &[object], ray_origin, ray_dir, 1000.0).expect("must hit the rotated cube");
        assert!((hit.t - (5.0 - half)).abs() < 1e-2, "t = {} (expected {})", hit.t, 5.0 - half);
        assert_eq!(hit.entity, e[0]);
    }

    /// The closer of two overlapping-in-screen-space objects must win —
    /// proves the BVH traversal doesn't just return the first leaf it
    /// happens to visit, but genuinely tracks the closest hit `t` across
    /// every candidate the ray could touch.
    #[test]
    fn closer_object_wins_when_two_objects_are_along_the_same_ray() {
        let e = entities(2);
        let near = TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(0.5), corner_radius: 0.0 },
            translation: Vec3::new(0.0, 2.0, 0.0),
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::new(1.0, 0.0, 0.0), 0.0, 0.5),
        };
        let far = TraceObject {
            entity: e[1],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(0.5), corner_radius: 0.0 },
            translation: Vec3::new(0.0, -2.0, 0.0),
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::new(0.0, 1.0, 0.0), 0.0, 0.5),
        };
        let objects = vec![near, far];
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject {
                entity: o.entity,
                world_aabb: Aabb::from_center_half(o.translation, Vec3::splat(0.5)),
            })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);

        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let hit = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).expect("must hit the near object");
        assert_eq!(hit.entity, e[0], "the nearer object (at y=2) must win over the farther one (at y=-2)");
    }

    /// Three objects strictly increasing in distance along one ray, built
    /// into a tree deep enough (4 leaves, one a decoy well off to the
    /// side) that the near/far child-push ordering added to `trace`'s
    /// internal-node branch actually gets exercised on more than one
    /// level. Pins the property that traversal order never changes which
    /// object wins — the closest object along the ray must be reported
    /// regardless of which subtree the SAH build happened to put it in —
    /// which is exactly the invariant a broken near/far comparison
    /// (visiting the farther child first, or an inverted `t_near`
    /// comparison) would violate.
    #[test]
    fn closest_of_three_stacked_objects_wins_regardless_of_bvh_subtree_placement() {
        let e = entities(4);
        let stacked = [
            (e[0], Vec3::new(0.0, 6.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            (e[1], Vec3::new(0.0, 3.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
            (e[2], Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ];
        let decoy = (e[3], Vec3::new(50.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        let all = [stacked[0], stacked[1], stacked[2], decoy];

        let objects: Vec<TraceObject> = all
            .iter()
            .map(|&(entity, translation, color)| TraceObject {
                entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(0.5), corner_radius: 0.0 },
                translation,
                rotation: Quat::IDENTITY,
                material: Material::new(color, 0.0, 0.5),
            })
            .collect();
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, Vec3::splat(0.5)) })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);

        // Ray travels downward from above the whole stack (y=10 -> -y):
        // e[0] at y=6 is nearest, e[1] at y=3 second, e[2] at y=0 farthest
        // of the three on-ray objects; the decoy at x=50 is never on the
        // ray at all, so it must never win regardless of traversal order.
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let hit = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).expect("must hit the nearest stacked object");
        assert_eq!(hit.entity, e[0], "closest object along the ray must win no matter which BVH subtree holds it");
    }

    // --- any_hit: must agree with trace()'s own hit-or-miss verdict on
    // every scenario already pinned above for `trace()` itself (reusing
    // the same fixtures/rays), since any_hit exists purely as a cheaper
    // way to ask the question trace()'s occlusion-only callers were
    // already asking by discarding everything but `.is_some()`.

    /// A ray that hits must be reported as occluded, with no exclusion.
    #[test]
    fn any_hit_agrees_with_trace_on_a_clean_hit() {
        let (_, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let expected = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).is_some();
        assert!(expected, "sanity: trace() must hit the cube here");
        assert_eq!(any_hit(&bvh, &objects, ray_origin, ray_dir, 1000.0, None), expected);
    }

    /// A ray missing the whole scene must be reported as unoccluded,
    /// matching `ray_missing_the_whole_scene_reports_no_hit` above.
    #[test]
    fn any_hit_agrees_with_trace_on_a_clean_miss() {
        let (_, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, 1.0, 0.0);
        let expected = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0).is_some();
        assert!(!expected, "sanity: trace() must miss here");
        assert_eq!(any_hit(&bvh, &objects, ray_origin, ray_dir, 1000.0, None), expected);
    }

    /// `t_max` cut short of the true hit distance must report unoccluded —
    /// an occluder beyond the query's own reach (e.g. a shadow ray capped
    /// at the light's own distance) must not count.
    #[test]
    fn any_hit_respects_t_max_short_of_the_true_hit() {
        let (_, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        // Cube top face is at t = 8.4 (see ray_through_cube_center_hits_
        // cube_with_its_flat_color above) — 5.0 stops well short of it.
        assert!(!any_hit(&bvh, &objects, ray_origin, ray_dir, 5.0, None));
        assert!(any_hit(&bvh, &objects, ray_origin, ray_dir, 1000.0, None));
    }

    /// Excluding the only object on the ray must report unoccluded — the
    /// self-exclusion every existing occlusion caller
    /// (`ddgi_ref.rs::sample_probe_grid`, `trace_shadow`) already relies on
    /// via `hit.entity != origin_entity`, now baked into the traversal
    /// itself instead of checked by the caller after the fact.
    #[test]
    fn any_hit_treats_the_excluded_entity_as_transparent() {
        let (e, objects, bvh) = ground_and_cube();
        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        // Excluding the cube (e[1]) leaves only the ground plate on this
        // ray, which the ray also reaches (ground is behind the cube).
        assert!(any_hit(&bvh, &objects, ray_origin, ray_dir, 1000.0, Some(e[1])));
        // Excluding the cube AND capping t_max short of the ground plate
        // (ground is at t=10.0, cube at t=8.4) must report unoccluded.
        assert!(!any_hit(&bvh, &objects, ray_origin, ray_dir, 9.0, Some(e[1])));
    }

    /// A ray whose origin starts already inside an object's own volume
    /// must not report a spurious self-hit at t~=0 — matches
    /// `march_object`'s own `t_start.max(0.0)` origin clamp, exercised
    /// through the full BVH-descending entry point rather than in
    /// isolation.
    #[test]
    fn any_hit_from_inside_geometry_does_not_self_report_at_zero() {
        let (e, objects, bvh) = ground_and_cube();
        // Cube spans y in [0.8, 2.4] at this fixture's translation; start
        // the ray at its vertical center, heading further into the cube
        // rather than immediately out through the face it started past.
        let ray_origin = Vec3::new(0.0, 1.6, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        let trace_hit = trace(&bvh, &objects, ray_origin, ray_dir, 1000.0);
        assert_eq!(
            any_hit(&bvh, &objects, ray_origin, ray_dir, 1000.0, None),
            trace_hit.is_some(),
            "trace() from inside the cube found entity {:?} at t={:?}",
            trace_hit.map(|h| h.entity == e[1]),
            trace_hit.map(|h| h.t),
        );
    }

    /// Three stacked objects (reusing the same tree-depth-exercising
    /// fixture as `closest_of_three_stacked_objects_wins_regardless_of_
    /// bvh_subtree_placement` above) — any_hit must agree with trace() at
    /// every t_max cutoff that crosses one of the three hit distances,
    /// proving traversal order (which any_hit deliberately does NOT fix to
    /// near-first the way trace() requires for correctness) never changes
    /// the hit-or-miss verdict itself.
    #[test]
    fn any_hit_agrees_with_trace_across_a_three_deep_stack_at_every_t_max_cutoff() {
        let e = entities(4);
        let stacked = [
            (e[0], Vec3::new(0.0, 6.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            (e[1], Vec3::new(0.0, 3.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
            (e[2], Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)),
        ];
        let decoy = (e[3], Vec3::new(50.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        let all = [stacked[0], stacked[1], stacked[2], decoy];

        let objects: Vec<TraceObject> = all
            .iter()
            .map(|&(entity, translation, color)| TraceObject {
                entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(0.5), corner_radius: 0.0 },
                translation,
                rotation: Quat::IDENTITY,
                material: Material::new(color, 0.0, 0.5),
            })
            .collect();
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, Vec3::splat(0.5)) })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);

        let ray_origin = Vec3::new(0.0, 10.0, 0.0);
        let ray_dir = Vec3::new(0.0, -1.0, 0.0);
        // Object half-extent 0.5 at y=6,3,0 -> near faces at t=3.5,6.5,9.5.
        for t_max in [1.0, 3.5, 3.6, 6.5, 6.6, 9.5, 9.6, 1000.0] {
            let expected = trace(&bvh, &objects, ray_origin, ray_dir, t_max).is_some();
            assert_eq!(
                any_hit(&bvh, &objects, ray_origin, ray_dir, t_max, None),
                expected,
                "mismatch at t_max={t_max}"
            );
        }
    }

    // --- local_distance: one pinned case per Shape variant, each checking
    // a known surface point (distance ~= 0), a known deep-interior point
    // (distance << 0), and a known far-exterior point (distance >> 0) —
    // enough to catch a sign error or a badly-transcribed formula without
    // needing every primitive's full geometric nuance re-derived here.
    // Formulas are ported verbatim from `sdf::primitives`'s already-proven
    // `Sdf::distance` impls (see each `sd_*` helper's doc comment), so
    // these tests exist to catch transcription mistakes, not to
    // re-establish correctness from first principles.

    #[test]
    fn sphere_distance_at_surface_interior_and_exterior() {
        let radius = 2.0;
        let shape = Shape::Sphere { radius };
        assert!(local_distance(&shape, Vec3::new(radius, 0.0, 0.0)).abs() < 1e-5);
        assert!(local_distance(&shape, Vec3::ZERO) < -radius + 1e-4);
        assert!(local_distance(&shape, Vec3::new(10.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn rounded_cylinder_distance_at_surface_interior_and_exterior() {
        let shape = Shape::RoundedCylinder { radius: 1.0, half_height: 2.0, edge_radius: 0.0 };
        assert!(local_distance(&shape, Vec3::new(1.0, 0.0, 0.0)).abs() < 1e-5, "side surface");
        assert!(local_distance(&shape, Vec3::new(0.0, 2.0, 0.0)).abs() < 1e-5, "top cap surface");
        assert!(local_distance(&shape, Vec3::ZERO) < 0.0, "center is interior");
        assert!(local_distance(&shape, Vec3::new(10.0, 10.0, 10.0)) > 0.0, "far exterior");
    }

    #[test]
    fn capsule_distance_at_surface_interior_and_exterior() {
        let a = Vec3::new(0.0, -1.0, 0.0);
        let b = Vec3::new(0.0, 1.0, 0.0);
        let radius = 0.5;
        let shape = Shape::Capsule { a, b, radius };
        assert!(local_distance(&shape, Vec3::new(radius, 0.0, 0.0)).abs() < 1e-5, "surface at the segment's middle");
        assert!(
            local_distance(&shape, Vec3::new(0.0, 1.0 + radius, 0.0)).abs() < 1e-5,
            "surface at the rounded end cap"
        );
        assert!(local_distance(&shape, Vec3::ZERO) < 0.0, "segment midpoint is interior");
        assert!(local_distance(&shape, Vec3::new(10.0, 10.0, 10.0)) > 0.0, "far exterior");
    }

    #[test]
    fn ellipsoid_distance_at_surface_interior_and_exterior() {
        let radii = Vec3::new(2.0, 1.0, 1.0);
        let shape = Shape::Ellipsoid { radii };
        assert!(local_distance(&shape, Vec3::new(2.0, 0.0, 0.0)).abs() < 1e-4, "surface along the long axis");
        assert!(local_distance(&shape, Vec3::new(0.0, 1.0, 0.0)).abs() < 1e-4, "surface along a short axis");
        // The exact center (0,0,0) is a genuine 0/0 singularity in this
        // formula (both k0 and k1 are 0 there) — use a nearby, clearly
        // interior point instead, not the degenerate center itself.
        assert!(local_distance(&shape, Vec3::new(0.5, 0.0, 0.0)) < 0.0, "interior point is negative");
        assert!(local_distance(&shape, Vec3::new(20.0, 20.0, 20.0)) > 0.0, "far exterior");
    }

    #[test]
    fn box_frame_distance_at_surface_interior_hollow_and_exterior() {
        let half_extents = Vec3::splat(1.0);
        let wall_thickness = 0.1;
        let shape = Shape::BoxFrame { half_extents, wall_thickness };
        assert!(local_distance(&shape, Vec3::new(1.0, 0.0, 0.0)).abs() < 1e-5, "outer wall surface");
        assert!(
            local_distance(&shape, Vec3::new(1.0 - wall_thickness / 2.0, 0.0, 0.0)) < 0.0,
            "inside the wall thickness is solid"
        );
        assert!(local_distance(&shape, Vec3::ZERO) > 0.0, "the hollow center is empty, not solid");
        assert!(local_distance(&shape, Vec3::new(10.0, 10.0, 10.0)) > 0.0, "far exterior");
    }

    #[test]
    fn hex_prism_distance_at_surface_interior_and_exterior() {
        let shape = Shape::HexPrism { radius: 1.0, half_height: 0.5 };
        assert!(local_distance(&shape, Vec3::new(0.0, 0.5, 0.0)).abs() < 1e-5, "top cap surface");
        assert!(local_distance(&shape, Vec3::ZERO) < 0.0, "center is interior");
        assert!(local_distance(&shape, Vec3::new(10.0, 10.0, 10.0)) > 0.0, "far exterior");
    }

    /// `RoundedCone` is deliberately unimplemented (see `local_distance`'s
    /// doc comment: a real, pre-existing bug in `sdf::primitives::
    /// RoundedCone::distance` was found while porting this module, and
    /// fixing that shared formula is out of scope here) — pins that this
    /// is an explicit, loud `unimplemented!` rather than a silent wrong
    /// answer, so a future caller can't accidentally rely on it.
    #[test]
    #[should_panic(expected = "RoundedCone is skipped")]
    fn rounded_cone_is_explicitly_unimplemented_not_silently_wrong() {
        let shape = Shape::RoundedCone { a: Vec3::ZERO, b: Vec3::new(0.0, 1.0, 0.0), r0: 1.0, r1: 0.5 };
        local_distance(&shape, Vec3::ZERO);
    }

    /// A sphere's true surface normal at any point is trivially the
    /// normalized position vector (radially outward from center) — the
    /// simplest possible ground truth to check `local_normal`'s
    /// finite-difference estimate against, at three points on three
    /// different axes so a single-axis transcription bug (e.g. always
    /// returning the X-axis gradient) would be caught.
    #[test]
    fn local_normal_on_a_sphere_points_radially_outward() {
        let shape = Shape::Sphere { radius: 2.0 };
        for p in [Vec3::new(2.0, 0.0, 0.0), Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, 0.0, 2.0)] {
            let n = local_normal(&shape, p);
            let expected = p.normalize();
            assert!(
                (n - expected).length() < 1e-3,
                "normal at {p:?} was {n:?}, expected {expected:?} (radially outward)"
            );
        }
    }

    /// A rounded box's face normal is trivially axis-aligned at a point
    /// on a flat face (far from any edge/corner rounding) — checks
    /// `local_normal` correctly reports a flat +X normal on the box's
    /// +X face, not just "some vector of the right length."
    #[test]
    fn local_normal_on_a_box_face_is_axis_aligned() {
        let shape = Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 };
        let n = local_normal(&shape, Vec3::new(1.0, 0.0, 0.0));
        assert!((n - Vec3::X).length() < 1e-3, "expected +X face normal, got {n:?}");
    }

    fn white_light(kind: LightKind) -> Light {
        Light {
            kind,
            color: Vec3::ONE,
            direction_or_position: Vec3::ZERO,
            intensity: 10000.0,
            spot_direction: Vec3::ZERO,
            range: 20.0,
            inner_angle: 0.3,
            outer_angle: 0.6,
            shadow_softness_k: 12.0,
        }
    }

    /// An empty scene (no BVH nodes, no objects) — `trace_shadow` always
    /// returns full visibility (`vis = 1.0`) against it, so `shade` tests
    /// that only care about the lighting/BRDF math (not occlusion) can use
    /// this and stay unaffected by shadow logic.
    fn no_occluders() -> (Bvh, Vec<TraceObject>) {
        (Bvh::default(), Vec::new())
    }

    /// A directional light shining straight down (`direction = -Y`) onto
    /// a flat, upward-facing surface (`normal = +Y`) must contribute
    /// strictly positive radiance — the straightforward "light hits a
    /// surface facing it" case every light kind's test below also checks
    /// in its own way.
    #[test]
    fn directional_light_facing_the_surface_contributes_positive_radiance() {
        let light = Light { direction_or_position: Vec3::new(0.0, -1.0, 0.0), ..white_light(LightKind::Directional) };
        let radiance = sample_light(&light, Vec3::ZERO).radiance;
        assert!(radiance.x > 0.0, "expected positive radiance, got {radiance:?}");
    }

    /// A directional light shining from BEHIND a surface (`direction =
    /// +Y`, same side as the outward normal `+Y` — i.e. traveling away
    /// from, not into, the surface) must contribute exactly zero shaded
    /// radiance, not a negative value a naive unclamped `N.L` could
    /// produce — `shade`'s `n_dot_l <= 0.0` guard is what enforces this
    /// (see its doc comment), since `sample_light` itself no longer knows
    /// about the surface normal.
    #[test]
    fn directional_light_behind_the_surface_contributes_nothing() {
        let light = Light { direction_or_position: Vec3::new(0.0, 1.0, 0.0), ..white_light(LightKind::Directional) };
        let material = Material::new(Vec3::splat(0.5), 0.0, 0.5);
        let (bvh, objects) = no_occluders();
        let radiance =
            shade(&material, Vec3::ZERO, Vec3::Y, Vec3::Y, &[light], &bvh, &objects, None, 0.0, None, None)
                .direct_and_emissive;
        assert_eq!(radiance, Vec3::ZERO, "expected zero contribution from a light behind the surface");
    }

    /// A point light placed directly above a surface, within its range,
    /// contributes positive radiance that strictly decreases as the
    /// surface point moves farther from the light — the basic distance-
    /// falloff sanity check `sample_light`'s `dist_atten` term
    /// exists to guarantee.
    #[test]
    fn point_light_radiance_falls_off_with_distance() {
        let light = Light { direction_or_position: Vec3::new(0.0, 5.0, 0.0), ..white_light(LightKind::Point) };
        let near = sample_light(&light, Vec3::new(0.0, 1.0, 0.0)).radiance;
        let far = sample_light(&light, Vec3::new(0.0, -1.0, 0.0)).radiance;
        assert!(near.x > 0.0, "expected positive radiance near the light, got {near:?}");
        assert!(far.x >= 0.0, "radiance must never go negative, got {far:?}");
        assert!(far.x < near.x, "radiance must fall off with distance: near={near:?} far={far:?}");
    }

    /// A point light placed beyond its own `range` contributes exactly
    /// zero — the hard falloff cutoff `dist_atten`'s `(dist/range)^4`
    /// term (clamped to `[0,1]`) is meant to enforce.
    #[test]
    fn point_light_beyond_its_range_contributes_nothing() {
        let light = Light { direction_or_position: Vec3::new(0.0, 100.0, 0.0), ..white_light(LightKind::Point) };
        let radiance = sample_light(&light, Vec3::ZERO).radiance;
        assert_eq!(radiance, Vec3::ZERO, "expected zero contribution beyond the light's range");
    }

    /// A spot light pointed straight down (`spot_direction = -Y`) at a
    /// surface point directly beneath it (inside the inner cone)
    /// contributes positive radiance, while a surface point far to the
    /// side (well outside the outer cone, same distance) contributes
    /// nothing — the basic cone-cutoff behavior a projector must have.
    #[test]
    fn spot_light_only_illuminates_inside_its_cone() {
        let light = Light {
            direction_or_position: Vec3::new(0.0, 5.0, 0.0),
            spot_direction: Vec3::new(0.0, -1.0, 0.0),
            ..white_light(LightKind::Spot)
        };
        let inside_cone = sample_light(&light, Vec3::new(0.0, 0.0, 0.0)).radiance;
        let outside_cone = sample_light(&light, Vec3::new(10.0, 0.0, 0.0)).radiance;
        assert!(inside_cone.x > 0.0, "expected positive radiance inside the cone, got {inside_cone:?}");
        assert_eq!(outside_cone, Vec3::ZERO, "expected zero radiance outside the cone, got {outside_cone:?}");
    }

    /// `shade` sums every light's contribution linearly — checked by
    /// comparing `shade` with two identical lights against double the
    /// single-light result, which must match exactly since summing
    /// radiance is linear in light count for a fixed BRDF/view/normal.
    #[test]
    fn shade_sums_multiple_lights_linearly() {
        let light = Light { direction_or_position: Vec3::new(0.0, -1.0, 0.0), ..white_light(LightKind::Directional) };
        let material = Material::new(Vec3::new(0.8, 0.4, 0.2), 0.0, 0.5);
        let p = Vec3::ZERO;
        let n = Vec3::Y;
        let view_dir = Vec3::new(0.3, 1.0, 0.2);
        let (bvh, objects) = no_occluders();

        let one_light =
            shade(&material, p, n, view_dir, &[light], &bvh, &objects, None, 0.0, None, None)
                .direct_and_emissive;
        let two_identical_lights =
            shade(&material, p, n, view_dir, &[light, light], &bvh, &objects, None, 0.0, None, None)
                .direct_and_emissive;
        assert!(
            (two_identical_lights - one_light * 2.0).length() < 1e-5,
            "two identical lights should double the single-light result: \
             one={one_light:?} two={two_identical_lights:?}"
        );
    }

    /// A fully metallic surface has no diffuse term at all — metals only
    /// reflect specularly, never scatter light diffusely (the defining
    /// property of the metallic-roughness workflow's `diffuse_color =
    /// albedo * (1 - metallic)` term). Checked by viewing straight up
    /// (`view_dir = Y`) while the light comes from well off to the side
    /// (`direction_or_position = (1.5, -1.0, 0.0)`, ~34 degrees off the
    /// vertical) with a fairly narrow specular lobe (`roughness = 0.3`):
    /// the reflection direction is far enough from straight-up that a
    /// metal's specular lobe barely reaches the view direction at all,
    /// while a dielectric's `(1-metallic)`-scaled diffuse term stays
    /// full-strength regardless of view angle. These exact parameters
    /// were picked by sweeping light angle x roughness combinations and
    /// checking the actual computed dielectric/metallic ratio (not
    /// eyeballed/assumed) — this combination gives ~14x, comfortably
    /// above the `10x` threshold asserted below with margin against
    /// float-precision noise.
    #[test]
    fn fully_metallic_surface_has_no_diffuse_response() {
        let light = Light { direction_or_position: Vec3::new(1.5, -1.0, 0.0), ..white_light(LightKind::Directional) };
        let p = Vec3::ZERO;
        let n = Vec3::Y;
        let view_dir = Vec3::Y;

        let dielectric = Material::new(Vec3::splat(0.5), 0.0, 0.3);
        let metallic = Material::new(Vec3::splat(0.5), 1.0, 0.3);
        let (bvh, objects) = no_occluders();
        let dielectric_result =
            shade(&dielectric, p, n, view_dir, &[light], &bvh, &objects, None, 0.0, None, None)
                .direct_and_emissive;
        let metallic_result =
            shade(&metallic, p, n, view_dir, &[light], &bvh, &objects, None, 0.0, None, None)
                .direct_and_emissive;
        assert!(
            dielectric_result.x > metallic_result.x * 10.0,
            "dielectric should be much brighter than metallic off the specular lobe: \
             dielectric={dielectric_result:?} metallic={metallic_result:?}"
        );
    }

    /// The specular highlight's peak brightness (viewing straight down the
    /// mirror-reflection direction of a smooth surface) must scale with
    /// `reflectance`/F0 exactly once, not twice — a regression test for the
    /// `hybrid_legacy` double-F0 bug this port deliberately avoids (see
    /// `shade`'s doc comment). A higher reflectance dielectric should be
    /// brighter at the highlight peak, by a factor consistent with F0
    /// appearing once in the Fresnel term, not squared.
    #[test]
    fn specular_highlight_scales_with_f0_once_not_squared() {
        // Directional light straight down, viewed from straight above:
        // `l = v = n`, so `h = n`, `n_dot_h = 1`, `v_dot_h = 1` — the
        // Fresnel term's grazing factor `(1 - cos_theta)^5` is exactly
        // zero, so `F = f0` precisely (no grazing-angle contamination),
        // making the specular peak's F0-scaling exact and easy to check.
        let light = Light { direction_or_position: Vec3::new(0.0, -1.0, 0.0), ..white_light(LightKind::Directional) };
        let p = Vec3::ZERO;
        let n = Vec3::Y;
        let view_dir = Vec3::Y;

        // Fully metallic with a bright albedo: f0 = albedo exactly
        // (metallic=1 selects the albedo branch of the f0 lerp), isolating
        // the specular term (diffuse is zero for metals).
        let albedo = Vec3::splat(0.8);
        let material = Material::new(albedo, 1.0, 0.3);
        let (bvh, objects) = no_occluders();
        let result = shade(&material, p, n, view_dir, &[light], &bvh, &objects, None, 0.0, None, None)
            .direct_and_emissive;

        let alpha = 0.3f32.powi(2);
        let d = ggx_distribution(1.0, alpha);
        let vis = ggx_visibility(1.0, 1.0, alpha);
        let sample = sample_light(&light, p);
        // Correct (single-F0) structure: specular = F0 * D * Vis.
        let expected = albedo * d * vis * sample.radiance;
        assert!(
            (result - expected).length() < 1e-4,
            "specular peak should equal F0*D*Vis exactly once, got {result:?} expected {expected:?} \
             (a double-F0 bug would instead produce roughly {:?})",
            albedo * albedo * d * vis * sample.radiance,
        );
    }

    /// `emissive` adds a constant term on top of lit shading, independent
    /// of any light in the scene — a material with emissive set should be
    /// visibly non-black even with zero lights.
    #[test]
    fn emissive_is_visible_with_no_lights() {
        let material = Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(0.2, 0.6, 0.9));
        let (bvh, objects) = no_occluders();
        let result =
            shade(&material, Vec3::ZERO, Vec3::Y, Vec3::Y, &[], &bvh, &objects, None, 0.0, None, None)
                .direct_and_emissive;
        assert_eq!(result, Vec3::new(0.2, 0.6, 0.9));
    }

    fn default_reflection_params() -> ReflectionParams {
        ReflectionParams {
            enabled: true,
            max_bounces: 1,
            fresnel_cutoff: 0.02,
            max_t: 60.0,
            diffuse_gi_max_t: 60.0,
            diffuse_gi_r0: 0.05,
            diffuse_gi_half_angle: 0.15,
            bounce_gi_enabled: true,
        }
    }

    /// `shade`'s own `ShadeResult.reflect` field must stay exactly zero
    /// when `reflection` is `None` — the non-recursion contract
    /// `conetrace_ref::cone_trace_ray`'s own bounce-shading call site
    /// depends on (see `ReflectionParams`'s own doc comment).
    #[test]
    fn shade_reflect_field_is_zero_when_reflection_param_is_none() {
        let light = white_light(LightKind::Directional);
        let material = Material::new(Vec3::splat(0.9), 0.0, 0.02).with_reflectance(1.0);
        let (bvh, objects) = no_occluders();
        let result = shade(&material, Vec3::ZERO, Vec3::Y, Vec3::NEG_Y, &[light], &bvh, &objects, None, 0.0, None, None);
        assert_eq!(result.reflect, Vec3::ZERO, "shade must never fire a reflection ray when reflection param is None");
    }

    /// A near-mirror surface with `reflection: Some(..)` and a reflected
    /// emissive box above it must report non-zero `ShadeResult.reflect` —
    /// proves `shade` itself actually wires the Fresnel-gated reflection
    /// call through end-to-end, not just that `reflect_ref` alone works
    /// in isolation.
    #[test]
    fn shade_reflect_field_picks_up_real_reflected_energy_when_enabled() {
        let e = {
            let mut world = World::new();
            [world.spawn_empty().id(), world.spawn_empty().id()]
        };
        let floor_material = Material::new(Vec3::splat(0.9), 0.0, 0.02).with_reflectance(1.0);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(10.0, 0.1, 10.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.1, 0.0),
                rotation: Quat::IDENTITY,
                material: floor_material,
            },
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 5.0, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(30.0, 5.0, 5.0)),
            },
        ];
        let hybrid_objects: Vec<crate::hybrid::scene::HybridObject> = vec![
            crate::hybrid::scene::HybridObject {
                entity: e[0],
                world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::new(10.0, 0.1, 10.0)),
            },
            crate::hybrid::scene::HybridObject { entity: e[1], world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![];
        // Shade a point on the floor, viewed from straight above (mirrors
        // reflect_ref.rs's own mirror_floor_with_box_above fixture).
        let p = Vec3::new(0.0, 0.0, 0.0);
        let n = Vec3::Y;
        let view_dir = Vec3::Y;
        let result = shade(&floor_material, p, n, view_dir, &lights, &bvh, &objects, Some(e[0]), 0.0, Some(default_reflection_params()), None);
        assert!(result.reflect.x > 0.01, "a near-mirror floor viewed straight-on must pick up the reflected box's own emissive energy in ShadeResult.reflect: got {:?}", result.reflect);
    }

    fn default_transmission_params() -> TransmissionParams {
        TransmissionParams {
            enabled: true,
            max_bounces: 1,
            fresnel_cutoff: 0.02,
            max_t: 60.0,
            diffuse_gi_max_t: 60.0,
            diffuse_gi_r0: 0.05,
            diffuse_gi_half_angle: 0.15,
            bounce_gi_enabled: true,
        }
    }

    /// `shade`'s own `ShadeResult.refract` field must stay exactly zero
    /// when `transmission` is `None` — mirrors `shade_reflect_field_is_
    /// zero_when_reflection_param_is_none`'s own contract.
    #[test]
    fn shade_refract_field_is_zero_when_transmission_param_is_none() {
        let light = white_light(LightKind::Directional);
        let material = Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(1.0).with_ior(1.5);
        let (bvh, objects) = no_occluders();
        let result = shade(&material, Vec3::ZERO, Vec3::Y, Vec3::NEG_Y, &[light], &bvh, &objects, None, 0.0, None, None);
        assert_eq!(result.refract, Vec3::ZERO, "shade must never fire a transmission ray when transmission param is None");
    }

    /// A `transmission: 0.0` material must report zero `ShadeResult.refract`
    /// even when `TransmissionParams::enabled` is true — per-material
    /// override of a scene-wide toggle, matching `metallic`'s own "no
    /// diffuse term at all" precedent.
    #[test]
    fn shade_refract_field_is_zero_for_an_opaque_material_even_when_transmission_is_enabled() {
        let light = white_light(LightKind::Directional);
        let material = Material::new(Vec3::splat(0.5), 0.0, 0.4); // transmission defaults to 0.0
        let e = {
            let mut world = World::new();
            [world.spawn_empty().id()]
        };
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            material,
        }];
        let hybrid_objects: Vec<crate::hybrid::scene::HybridObject> =
            vec![crate::hybrid::scene::HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(1.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        let result = shade(
            &material,
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::NEG_Z,
            Vec3::Z,
            &[light],
            &bvh,
            &objects,
            Some(e[0]),
            0.0,
            None,
            Some(default_transmission_params()),
        );
        assert_eq!(result.refract, Vec3::ZERO, "an opaque (transmission=0.0) material must report zero refract regardless of TransmissionParams::enabled");
    }

    /// A clear-glass object with a real, lit surface behind it must report
    /// non-zero `ShadeResult.refract` — proves `shade` itself wires the
    /// Fresnel-gated transmission call through end-to-end, not just that
    /// `refract_ref` alone works in isolation (mirrors `shade_reflect_
    /// field_picks_up_real_reflected_energy_when_enabled`'s own shape).
    #[test]
    fn shade_refract_field_picks_up_real_transmitted_energy_when_enabled() {
        let e = {
            let mut world = World::new();
            [world.spawn_empty().id(), world.spawn_empty().id()]
        };
        let glass_material = Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(1.0).with_ior(1.5);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                material: glass_material,
            },
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, 5.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(30.0, 5.0, 5.0)),
            },
        ];
        let hybrid_objects: Vec<crate::hybrid::scene::HybridObject> = vec![
            crate::hybrid::scene::HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::splat(1.0)) },
            crate::hybrid::scene::HybridObject { entity: e[1], world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![]; // no direct lights: any red-dominant result must come from the box behind the glass
        // Shade the glass's own front (-Z) face, viewed straight on —
        // mirrors refract_ref.rs's own glass_cube_with_box_behind fixture.
        // `view_dir` is the surface->camera vector (`v`, same convention
        // `shade`'s own direct-light loop uses via `n.dot(v)`); a camera
        // in front of the glass looking toward +Z sees the ray travel +Z
        // INTO the surface, so `v` (surface->camera) points -Z.
        let p = Vec3::new(0.0, 0.0, -1.0);
        let n = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let result = shade(&glass_material, p, n, view_dir, &lights, &bvh, &objects, Some(e[0]), 0.0, None, Some(default_transmission_params()));
        assert!(result.refract.x > 0.01, "clear glass viewed straight-on with a red-emissive box behind it must pick up real transmitted energy in ShadeResult.refract: got {:?}", result.refract);
    }

    /// Reconstructs examples/gallery.rs's REAL --stress 100 scene shape
    /// exactly: a 10x10 grid of independent ground+cube cell pairs, using
    /// the exact same GROUND_HALF_EXTENT (4,0.2,4) / CELL_GAP (3.0) /
    /// cell-center formula gallery.rs's spawn_scene/stress_cell_center use
    /// (stress_grid_dim(100) = ceil(sqrt(100)) = 10).
    fn stress_100_scene() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        const DIM: usize = 10;
        const CELL_SIZE: f32 = 4.0 * 2.0 + 3.0; // 11.0
        fn cell_center(index: usize) -> Vec3 {
            let row = (index / DIM) as f32;
            let col = (index % DIM) as f32;
            let half = (DIM as f32 - 1.0) * 0.5;
            Vec3::new((col - half) * CELL_SIZE, 0.0, (row - half) * CELL_SIZE)
        }
        let e = entities(DIM * DIM * 2);
        let mut objects = Vec::new();
        for i in 0..(DIM * DIM) {
            let center = cell_center(i);
            objects.push(TraceObject {
                entity: e[i * 2],
                shape: Shape::RoundedBox { half_extents: Vec3::new(4.0, 0.2, 4.0), corner_radius: 0.0 },
                translation: center + Vec3::new(0.0, -0.2, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::splat(0.5), 0.0, 0.5),
            });
            objects.push(TraceObject {
                entity: e[i * 2 + 1],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(0.8), corner_radius: 0.0 },
                translation: center + Vec3::new(0.0, 0.8, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.85, 0.35, 0.20), 0.0, 0.4),
            });
        }
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject {
                entity: o.entity,
                world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)),
            })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    /// Same real --stress 100 scene shape as `stress_100_scene`, but with
    /// a Sphere (radius 0.9, matching examples/gallery.rs --shape sphere)
    /// instead of a cube — reproducing the "shadow cut on one side" bug
    /// report specifically for curved geometry in a real multi-cell grid.
    fn stress_100_sphere_scene() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        const DIM: usize = 10;
        const CELL_SIZE: f32 = 4.0 * 2.0 + 3.0; // 11.0
        fn cell_center(index: usize) -> Vec3 {
            let row = (index / DIM) as f32;
            let col = (index % DIM) as f32;
            let half = (DIM as f32 - 1.0) * 0.5;
            Vec3::new((col - half) * CELL_SIZE, 0.0, (row - half) * CELL_SIZE)
        }
        let e = entities(DIM * DIM * 2);
        let mut objects = Vec::new();
        for i in 0..(DIM * DIM) {
            let center = cell_center(i);
            objects.push(TraceObject {
                entity: e[i * 2],
                shape: Shape::RoundedBox { half_extents: Vec3::new(4.0, 0.2, 4.0), corner_radius: 0.0 },
                translation: center + Vec3::new(0.0, -0.2, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::splat(0.5), 0.0, 0.5),
            });
            objects.push(TraceObject {
                entity: e[i * 2 + 1],
                shape: Shape::Sphere { radius: 0.9 },
                translation: center + Vec3::new(0.0, 0.9, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.85, 0.35, 0.20), 0.0, 0.4),
            });
        }
        let hybrid_objects: Vec<HybridObject> = objects
            .iter()
            .map(|o| HybridObject {
                entity: o.entity,
                world_aabb: Aabb::from_center_half(o.translation, local_half_extents(&o.shape)),
            })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    /// Real sun light exactly as examples/gallery.rs::spawn_lights spawns
    /// it: `Transform::from_xyz(0,10,0).looking_at((-3,0,-2), Y)`,
    /// illuminance 1500, shadow_softness_k matching ShadowConfig::default
    /// (12.0).
    fn real_sun_light() -> Light {
        real_sun_light_with_k(2.0)
    }

    fn real_sun_light_with_k(k: f32) -> Light {
        let from = Vec3::new(0.0, 10.0, 0.0);
        let to = Vec3::new(-3.0, 0.0, -2.0);
        let forward = (to - from).normalize(); // direction rays TRAVEL
        Light {
            kind: LightKind::Directional,
            color: Vec3::ONE,
            direction_or_position: forward,
            intensity: 1500.0,
            spot_direction: Vec3::ZERO,
            range: 0.0,
            inner_angle: 0.0,
            outer_angle: 0.0,
            shadow_softness_k: k,
        }
    }

    /// Dense scan across the REAL 10x10 --stress 100 scene's full ground
    /// area, using the real sun light — finds any point whose shadow ray
    /// is occluded by something OTHER than its own cell's cube/ground
    /// (`origin_entity` set to that cell's own ground, so the ONLY
    /// legitimate occluder left is that same cell's own cube). Reports
    /// every anomalous point found (not just asserting none exist) so a
    /// human can inspect the actual worst case with real numbers, per the
    /// project's "get real data before guessing" debugging convention —
    /// this exists specifically because iterating on plausible-sounding
    /// formula fixes without reproducing the actual failing case first
    /// wasted real time chasing fixes that didn't address the real
    /// artifact.
    ///
    /// This specific sweep is what found `ShadowConfig::default`'s real
    /// `k=2.0` value (was `k=12.0`, matching `hybrid_legacy`'s own
    /// default, before this investigation): at this scene's actual
    /// object scale (~0.8-2.4 units), `k=12` produces real, confirmed
    /// false darkening (`d/(k*t)`'s `1/t` decay makes even a genuine
    /// 1-unit gap read as ~95% shadowed once `t` reaches ~2 units) once
    /// the candidate-margin fix (independently required for round, not
    /// polygonal, silhouettes) is large enough to find real-but-distant
    /// candidates. `k<=2.0` was the largest (hardest/crispest) value
    /// with zero false darkening across a full `--stress 100` grid scan
    /// in this sweep's own results; `k>=3.0` all showed real false
    /// darkening (worst-case `vis` well below the `0.9` threshold used
    /// elsewhere in this module's shadow tests). Kept as a permanent test
    /// (not deleted after use) since the sweep itself is cheap
    /// (`cargo test` speed) and documents exactly where the safe/unsafe
    /// boundary is, should this scene's object scale or `--stress` grid
    /// spacing ever change.
    #[test]
    fn debug_k_sweep_stress_100_worst_case() {
        let (e, objects, bvh) = stress_100_scene();
        const DIM: usize = 10;
        const CELL_SIZE: f32 = 11.0;
        let half = (DIM as f32 - 1.0) * 0.5;
        let mut worst_by_k = Vec::new();
        for k in [12.0f32, 8.0, 6.0, 4.0, 3.0, 2.0, 1.5, 1.0] {
            let light = real_sun_light_with_k(k);
            let mut worst = 1.0f32;
            for cell in 0..(DIM * DIM) {
                let row = (cell / DIM) as f32;
                let col = (cell % DIM) as f32;
                let center = Vec3::new((col - half) * CELL_SIZE, 0.0, (row - half) * CELL_SIZE);
                let ground_entity = e[cell * 2];
                for gx in [-3.5f32, -2.0, 2.0, 3.5] {
                    for gz in [-3.5f32, -2.0, 2.0, 3.5] {
                        let p = center + Vec3::new(gx, 0.0, gz);
                        let ro = p + Vec3::Y * shadow_bias(10.0);
                        let sample = sample_light(&light, p);
                        let result = trace_shadow(
                            &bvh,
                            &objects,
                            ro,
                            sample.to_light,
                            sample.shadow_max_t,
                            light.shadow_softness_k,
                            Some(ground_entity),
                        );
                        worst = worst.min(result.vis());
                    }
                }
            }
            println!("k={k}: worst_case_vis={worst}");
            worst_by_k.push((k, worst));
        }
        // k<=2.0 must show zero false darkening; this is the evidence
        // ShadowConfig::default's k=2.0 is based on — if this ever
        // regresses, the default's own justification (its doc comment)
        // is now wrong too and needs revisiting alongside this assertion.
        for (k, worst) in &worst_by_k {
            if *k <= 2.0 {
                assert!(*worst > 0.9, "k={k} should show no false darkening, got worst_case_vis={worst}");
            }
        }
    }

    /// Investigates the "shadow cut on one side" bug report: a sphere's
    /// soft shadow inside a real --stress 100 grid cell reads a flat,
    /// hard-truncated edge on one side instead of tapering smoothly like
    /// the rest of the penumbra. Prints a full angular visibility profile
    /// on a ring around one cell's own sphere (using the SAME cell's own
    /// ground/sphere pair, embedded in the real 10x10 grid this time —
    /// unlike the single-object `no_ring_sample_near_the_sphere_reads_
    /// fully_lit` test, this exercises the real gather_candidates_padded
    /// call against a BVH containing 199 OTHER objects too) — any point
    /// where vis jumps sharply between adjacent angles (not a smooth
    /// gradient) pinpoints exactly where the cut happens and at what
    /// angle relative to the light.
    #[test]
    fn debug_stress_100_sphere_shadow_ring_profile() {
        let (e, objects, bvh) = stress_100_sphere_scene();
        let light = real_sun_light();

        const DIM: usize = 10;
        const CELL_SIZE: f32 = 11.0;
        let half = (DIM as f32 - 1.0) * 0.5;
        // Middle-ish cell, not a grid edge, matching the report's "some
        // spheres in the middle of the grid show this" framing.
        let cell = 4 * DIM + 4;
        let row = (cell / DIM) as f32;
        let col = (cell % DIM) as f32;
        let center = Vec3::new((col - half) * CELL_SIZE, 0.0, (row - half) * CELL_SIZE);
        let ground_entity = e[cell * 2];

        const SAMPLE_COUNT: usize = 64;
        const RING_RADIUS: f32 = 1.4; // just outside the sphere's own radius (0.9)
        let mut profile = Vec::with_capacity(SAMPLE_COUNT);
        for i in 0..SAMPLE_COUNT {
            let angle = (i as f32 / SAMPLE_COUNT as f32) * std::f32::consts::TAU;
            let p = center + Vec3::new(RING_RADIUS * angle.cos(), 0.0, RING_RADIUS * angle.sin());
            let ro = p + Vec3::Y * shadow_bias(10.0);
            let sample = sample_light(&light, p);
            let result =
                trace_shadow(&bvh, &objects, ro, sample.to_light, sample.shadow_max_t, light.shadow_softness_k, Some(ground_entity));
            profile.push((angle, result.vis()));
        }

        // Flag any adjacent-sample jump bigger than would come from a
        // smooth gradient — a real "cut" (found and fixed: internal BVH
        // nodes weren't padded by margin, only leaves, so a ray could
        // miss a padded leaf's UNPADDED parent node and get pruned from
        // the whole subtree before the leaf's own padded test ever ran —
        // see gather_candidates_padded's doc comment) shows as a large
        // single jump, not a gradual slope.
        let mut max_jump = 0.0f32;
        let mut max_jump_at = 0usize;
        for i in 0..SAMPLE_COUNT {
            let (_, v0) = profile[i];
            let (_, v1) = profile[(i + 1) % SAMPLE_COUNT];
            let jump = (v1 - v0).abs();
            if jump > max_jump {
                max_jump = jump;
                max_jump_at = i;
            }
        }
        assert!(
            max_jump < 0.3,
            "found a sharp discontinuity in the ring's angular visibility profile (max_jump={max_jump} \
             at sample {max_jump_at}, angle={:.4}): {profile:?}",
            profile[max_jump_at].0
        );
    }

    #[test]
    fn debug_stress_100_full_grid_shadow_scan() {
        let (e, objects, bvh) = stress_100_scene();
        let light = real_sun_light();

        const DIM: usize = 10;
        const CELL_SIZE: f32 = 11.0;
        let half = (DIM as f32 - 1.0) * 0.5;

        let mut worst: Option<(f32, usize, Vec3)> = None; // (vis, cell_index, point)
        for cell in 0..(DIM * DIM) {
            let row = (cell / DIM) as f32;
            let col = (cell % DIM) as f32;
            let center = Vec3::new((col - half) * CELL_SIZE, 0.0, (row - half) * CELL_SIZE);
            let ground_entity = e[cell * 2];
            // Sample a coarse grid of points across this cell's own ground
            // footprint, well outside the cube's own tiny shadow (cube
            // half-extent 0.8, so anything beyond ~2.0 units from center
            // in any direction is definitely NOT the cube's own penumbra).
            for gx in [-3.5f32, -2.0, 2.0, 3.5] {
                for gz in [-3.5f32, -2.0, 2.0, 3.5] {
                    let p = center + Vec3::new(gx, 0.0, gz);
                    let ro = p + Vec3::Y * shadow_bias(10.0);
                    let sample = sample_light(&light, p);
                    let result =
                        trace_shadow(&bvh, &objects, ro, sample.to_light, sample.shadow_max_t, light.shadow_softness_k, Some(ground_entity));
                    let vis = result.vis();
                    if vis < 0.9 && worst.is_none_or(|(w, _, _)| vis < w) {
                        worst = Some((vis, cell, p));
                    }
                }
            }
        }

        if let Some((vis, cell, p)) = worst {
            println!("WORST CASE: cell={cell} p={p:?} vis={vis}");
            let row = (cell / DIM) as f32;
            let col = (cell % DIM) as f32;
            let center = Vec3::new((col - half) * CELL_SIZE, 0.0, (row - half) * CELL_SIZE);
            let ground_entity = e[cell * 2];
            let ro = p + Vec3::Y * shadow_bias(10.0);
            let sample = sample_light(&light, p);
            println!(
                "to_light={:?} shadow_max_t={} k={}",
                sample.to_light, sample.shadow_max_t, light.shadow_softness_k
            );
            let margin = shadow_candidate_margin(light.shadow_softness_k);
            println!("margin={margin}");
            let candidates = gather_candidates_padded(&bvh, ro, sample.to_light, 0.001, sample.shadow_max_t, margin);
            println!("candidates found: {}", candidates.len());
            for cand in &candidates {
                let obj = objects.iter().find(|o| o.entity == cand.entity).unwrap();
                println!(
                    "  candidate entity={:?} near={:.3} translation={:?} (this cell's own center={:?})",
                    cand.entity, cand.near, obj.translation, center
                );
            }
            println!("origin_entity (excluded) = {ground_entity:?}");
            // Manually re-run the march loop against the cube candidate
            // with full step-by-step printing, mirroring trace_shadow's
            // exact logic — to see exactly which step/term produces the
            // too-dark vis for a point genuinely outside the cube's true
            // shadow reach.
            let cand = candidates.iter().find(|c| c.entity != ground_entity).unwrap();
            let cube = objects.iter().find(|o| o.entity == cand.entity).unwrap();
            let mut t = cand.near.max(0.01);
            let mut ph = 1e20f32;
            let mut prev_step = 1e20f32;
            let mut best_h = f32::MAX;
            let mut step_n = 0;
            let mut step_vis = 1.0f32;
            loop {
                if !(t <= sample.shadow_max_t && step_vis > VIS_CUTOFF) {
                    break;
                }
                let h = object_distance(cube, ro + sample.to_light * t);
                if h < SHADOW_HIT_EPSILON {
                    println!("step {step_n}: t={t:.4} HARD HIT h={h:.6}");
                    break;
                }
                let diverged = h > best_h * DIVERGENCE_FACTOR;
                if h < best_h {
                    best_h = h;
                }
                let y = (h * h / (2.0 * ph)).min(prev_step);
                let d = (h * h - y * y).max(0.0).sqrt();
                let sample_vis = (d / (light.shadow_softness_k * (t - y).max(1e-4))).clamp(0.0, 1.0);
                step_vis = step_vis.min(sample_vis);
                println!(
                    "step {step_n}: t={t:.4} h={h:.4} best_h={best_h:.4} diverged={diverged} y={y:.4} d={d:.4} \
                     sample_vis={sample_vis:.4} running_vis={step_vis:.4}"
                );
                if diverged {
                    println!("  -> would break (diverged)");
                    break;
                }
                ph = h;
                let mut step = h * 1.2;
                if t + step > sample.shadow_max_t {
                    step = h;
                    if t + step > sample.shadow_max_t {
                        break;
                    }
                }
                prev_step = step;
                t += step;
                step_n += 1;
                if step_n > 40 {
                    println!("  -> bail (too many steps printed)");
                    break;
                }
            }

            panic!(
                "found shadow darkening on cell {cell}'s own ground, far from its own cube's TRUE \
                 shadow reach (see printed step trace above for exactly which step/term produced the \
                 too-dark vis)"
            );
        }
    }

    // -----------------------------------------------------------------
    // blur_indirect_at
    // -----------------------------------------------------------------

    fn uniform_grid(indirect: Vec3, normal: Vec3, depth: f32) -> impl Fn(i32, i32) -> BlurSample {
        move |_dx, _dy| BlurSample { indirect, normal, depth }
    }

    #[test]
    fn blur_indirect_is_a_no_op_on_a_flat_noise_free_region() {
        let indirect = Vec3::new(0.3, 0.4, 0.5);
        let normal = Vec3::Y;
        let depth = 10.0;
        let center = BlurSample { indirect, normal, depth };
        let blurred = blur_indirect_at(center, uniform_grid(indirect, normal, depth));
        assert!(
            (blurred - indirect).length() < 1e-4,
            "a uniform, noise-free neighborhood should blur to itself: got {blurred:?}, expected {indirect:?}"
        );
    }

    #[test]
    fn blur_indirect_does_not_cross_a_hard_normal_discontinuity() {
        // Center pixel sits on a surface facing +Y; every neighbor sits
        // on a perpendicular surface facing +X with a totally different
        // indirect color — simulating a hard 90-degree edge (e.g. a box
        // corner). The blur must not pull in the neighbor's color: an
        // edge-agnostic blur would average the two and leak color across
        // the corner, which is exactly the artifact edge-awareness
        // exists to prevent.
        let center_indirect = Vec3::new(0.2, 0.2, 0.2);
        let neighbor_indirect = Vec3::new(0.9, 0.1, 0.1);
        let center = BlurSample { indirect: center_indirect, normal: Vec3::Y, depth: 10.0 };
        let sample_at = |dx: i32, dy: i32| {
            if dx == 0 && dy == 0 {
                center
            } else {
                BlurSample { indirect: neighbor_indirect, normal: Vec3::X, depth: 10.0 }
            }
        };
        let blurred = blur_indirect_at(center, sample_at);
        assert!(
            (blurred - center_indirect).length() < 0.05,
            "blur must not cross a hard normal discontinuity: got {blurred:?}, expected close to {center_indirect:?}"
        );
    }

    #[test]
    fn blur_indirect_does_not_cross_a_depth_discontinuity() {
        // Same idea as the normal-discontinuity test, but for a
        // silhouette edge: same normal on both sides (so normal weight
        // alone wouldn't reject it), but the neighbor sits far behind —
        // e.g. background visible past an object's silhouette. The
        // relative-depth weight should suppress it even though the
        // normals agree.
        let center_indirect = Vec3::new(0.2, 0.2, 0.2);
        let neighbor_indirect = Vec3::new(0.9, 0.1, 0.1);
        let center = BlurSample { indirect: center_indirect, normal: Vec3::Y, depth: 2.0 };
        let sample_at = |dx: i32, dy: i32| {
            if dx == 0 && dy == 0 {
                center
            } else {
                BlurSample { indirect: neighbor_indirect, normal: Vec3::Y, depth: 50.0 }
            }
        };
        let blurred = blur_indirect_at(center, sample_at);
        assert!(
            (blurred - center_indirect).length() < 0.05,
            "blur must not cross a depth discontinuity: got {blurred:?}, expected close to {center_indirect:?}"
        );
    }

    #[test]
    fn blur_indirect_reduces_variance_in_a_noisy_flat_region() {
        // A synthetic flat region (uniform normal+depth, matching what a
        // single smooth surface actually looks like) with independent
        // per-pixel noise added to the indirect color — the exact shape
        // of Stage B's real grain artifact. Blurring a noisy flat region
        // must measurably reduce the color variance across the region,
        // since there is no real edge anywhere to preserve.
        let normal = Vec3::Y;
        let depth = 10.0;
        const W: i32 = 16;
        const H: i32 = 16;
        let noisy = |x: i32, y: i32| -> Vec3 {
            // Deterministic pseudo-noise, no external RNG dependency.
            let h = ((x.wrapping_mul(374_761_393) ^ y.wrapping_mul(668_265_263)) as u32) as f32;
            let n = ((h * 0.000_000_1).fract() - 0.5) * 0.6;
            Vec3::splat(0.5) + Vec3::new(n, n * 0.7, n * 0.4)
        };
        let clamp = |v: i32, max: i32| v.clamp(0, max - 1);
        let sample_at = |cx: i32, cy: i32| move |dx: i32, dy: i32| BlurSample {
            indirect: noisy(clamp(cx + dx, W), clamp(cy + dy, H)),
            normal,
            depth,
        };

        let variance = |values: &[Vec3]| -> f32 {
            let mean: Vec3 = values.iter().copied().sum::<Vec3>() / values.len() as f32;
            values.iter().map(|v| (*v - mean).length_squared()).sum::<f32>() / values.len() as f32
        };

        let mut before = Vec::with_capacity((W * H) as usize);
        let mut after = Vec::with_capacity((W * H) as usize);
        for y in 0..H {
            for x in 0..W {
                let center = BlurSample { indirect: noisy(x, y), normal, depth };
                before.push(center.indirect);
                after.push(blur_indirect_at(center, sample_at(x, y)));
            }
        }
        let var_before = variance(&before);
        let var_after = variance(&after);
        assert!(
            var_after < var_before * 0.5,
            "blur should substantially reduce variance in a noisy flat region: before {var_before}, after {var_after}"
        );
    }

    #[test]
    fn blur_indirect_recombination_matches_pre_split_shade_when_blur_is_identity() {
        // Regression guard tying the split-then-recombine path back to
        // the original, already-validated single-Vec3 `shade()`
        // contract: when the "blur" is a true identity (every neighbor
        // weight collapses to the center pixel alone), direct_and_emissive
        // + indirect must reproduce exactly what `shade()` returned
        // before the ShadeResult split.
        let (e, objects, bvh) = ground_and_cube();
        let light = Light {
            direction_or_position: Vec3::new(-0.3, -1.0, -0.2).normalize(),
            ..white_light(LightKind::Directional)
        };
        let material = Material::new(Vec3::new(0.7, 0.7, 0.7), 0.0, 0.6);
        let p = Vec3::new(0.0, 0.0, 3.0);
        let n = Vec3::Z;
        let view_dir = Vec3::Z;

        let result = shade(&material, p, n, view_dir, &[light], &bvh, &objects, None, 0.0, None, None);

        // Identity blur: sample_at always returns the center sample
        // itself, so weighting is irrelevant and the blurred value must
        // equal the unblurred indirect term exactly.
        let center = BlurSample { indirect: result.indirect, normal: n, depth: 3.0 };
        let blurred = blur_indirect_at(center, uniform_grid(result.indirect, n, 3.0));
        let recombined = result.direct_and_emissive + blurred;

        assert_eq!(e.len(), 2, "test sanity: expected ground+cube entities");
        assert!(
            (recombined - (result.direct_and_emissive + result.indirect)).length() < 1e-5,
            "identity-blur recombination must reproduce the pre-split shade() sum exactly: got {recombined:?}"
        );
    }

    // -----------------------------------------------------------------
    // blur_strength / adaptive_blur_indirect_at
    // -----------------------------------------------------------------

    #[test]
    fn blur_strength_is_full_at_history_length_one_or_below() {
        assert_eq!(blur_strength(1.0, 24.0), 1.0);
        assert_eq!(blur_strength(0.0, 24.0), 1.0, "a brand-new pixel (history_length 0) must still get full blur");
    }

    #[test]
    fn blur_strength_is_zero_at_or_past_the_history_cap() {
        assert_eq!(blur_strength(24.0, 24.0), 0.0);
        assert_eq!(blur_strength(100.0, 24.0), 0.0, "strength must not go negative past the cap");
    }

    #[test]
    fn blur_strength_is_monotonically_decreasing_between_the_endpoints() {
        let mut previous = blur_strength(1.0, 24.0);
        for i in 2..=24 {
            let current = blur_strength(i as f32, 24.0);
            assert!(
                current <= previous + 1e-6,
                "blur_strength must never increase as history_length grows: at {i} got {current}, previous was {previous}"
            );
            previous = current;
        }
    }

    #[test]
    fn adaptive_blur_matches_full_blur_at_history_length_one() {
        let center_indirect = Vec3::new(0.2, 0.2, 0.2);
        let neighbor_indirect = Vec3::new(0.9, 0.1, 0.1);
        let center = BlurSample { indirect: center_indirect, normal: Vec3::Y, depth: 10.0 };
        let sample_at = uniform_grid(neighbor_indirect, Vec3::Y, 10.0);
        let full = blur_indirect_at(center, &sample_at);
        let adaptive = adaptive_blur_indirect_at(center, 1.0, 24.0, &sample_at);
        assert!((adaptive - full).length() < 1e-5, "at history_length 1, adaptive blur should equal full blur exactly");
    }

    #[test]
    fn adaptive_blur_matches_unblurred_center_at_the_history_cap() {
        let center_indirect = Vec3::new(0.2, 0.2, 0.2);
        let neighbor_indirect = Vec3::new(0.9, 0.1, 0.1);
        let center = BlurSample { indirect: center_indirect, normal: Vec3::Y, depth: 10.0 };
        let sample_at = uniform_grid(neighbor_indirect, Vec3::Y, 10.0);
        let adaptive = adaptive_blur_indirect_at(center, 24.0, 24.0, &sample_at);
        assert!(
            (adaptive - center_indirect).length() < 1e-5,
            "at the history cap, adaptive blur should leave the center value untouched: got {adaptive:?}"
        );
    }

    #[test]
    fn adaptive_blur_at_half_convergence_lies_strictly_between_center_and_full_blur() {
        let center_indirect = Vec3::new(0.2, 0.2, 0.2);
        let neighbor_indirect = Vec3::new(0.9, 0.1, 0.1);
        let center = BlurSample { indirect: center_indirect, normal: Vec3::Y, depth: 10.0 };
        let sample_at = uniform_grid(neighbor_indirect, Vec3::Y, 10.0);
        let full = blur_indirect_at(center, &sample_at);
        let half = adaptive_blur_indirect_at(center, 12.0, 24.0, &sample_at);
        let dist_to_center = (half - center_indirect).length();
        let dist_to_full = (half - full).length();
        assert!(dist_to_center > 1e-4 && dist_to_full > 1e-4, "half-strength blur should differ from both endpoints");
    }
}
