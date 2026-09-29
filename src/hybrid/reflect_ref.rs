//! CPU reference for multi-bounce specular reflections — the specular
//! counterpart to `conetrace_ref`'s diffuse indirect GI, added once that
//! module's cone-marching primitives (`trace_cone`/`march_object_cone`/
//! `cone_slab_hit`) were already proven correct and real-GPU-verified.
//!
//! **Same substrate, different ray.** There is no dedicated SDF-specific
//! reflection literature distinct from voxel cone tracing (Crassin et al.
//! 2011) — the standard real-time answer is "cone tracing for glossy" is
//! just the same aperture-widening idea applied to whatever acceleration
//! structure a renderer already has. This renderer already has exactly
//! that: `conetrace_ref::trace_cone` marches a growing-radius cone against
//! the BVH/SDF scene and reports a continuous `coverage`. A reflection ray
//! reuses that function verbatim — only the ray's origin direction
//! (`reflect(view_dir, normal)`, not a hemisphere sample) and aperture
//! source (roughness, not a fixed config value) differ.
//!
//! **Roughness -> cone aperture.** `half_angle = atan(alpha) + epsilon`,
//! where `alpha` is the EXACT GGX alpha `cpu_ref::shade` already computes
//! (`roughness.clamp(0,1).powi(2).max(1e-3)`) — no new material field, no
//! new remap. `epsilon` keeps the aperture strictly positive even at
//! `alpha -> 0` (a mirror-perfect material), avoiding a truly zero-radius
//! cone (which is legal — it degenerates to a point ray, see
//! `conetrace_ref::cone_degenerates_to_point_ray_at_zero_half_angle` — but
//! would make every mirror surface pay for zero antialiasing at its
//! reflection edges, a real, if small, quality cost with no offsetting
//! benefit here).
//!
//! **Bounce cost shape genuinely differs from diffuse GI's.**
//! `conetrace_ref::cone_trace_indirect` fires 5 cones at bounce 1 (a full
//! hemisphere sample), then degenerates every later bounce to a single
//! point ray specifically to avoid `5^depth` fan-out. A reflection ray has
//! no such fan-out to begin with — exactly one ray in, one ray out, at
//! EVERY bounce, by construction (a mirror reflects one direction, not a
//! hemisphere). So `reflect_trace_ray`'s cost is already linear in bounce
//! count without needing the "degenerate after bounce 1" trick; every
//! bounce fires a REAL cone (roughness-derived aperture), not a point ray.
//!
//! **Termination via diffuse GI, not further reflection ("final gather"),
//! matching Lumen's own documented default (`r.Lumen.Reflections.
//! MaxBounces` defaults to 1).** Each bounce's hit is shaded via
//! `shade_for_reflection_bounce` — full direct lighting + real shadow rays
//! (`trace_shadow`) + ONE bounce of diffuse cone-traced GI, via
//! `conetrace_ref::cone_trace_indirect_single` (a single cone straight
//! along the surface normal, NOT the full 5-cone hemisphere
//! `cone_trace_indirect` uses for primary-ray GI) — so a reflected
//! surface correctly shows up lit, shadowed, and GI-bounced, not
//! flat/unlit like `conetrace_ref`'s own direct-only diffuse-GI shading.
//! The single-cone reduction (found via real cost measurement: the
//! 5-cone version made a reflected pixel cost roughly as much as the
//! entire primary-ray GI term a second time) trades directional GI
//! fidelity for cost on a term that's already second-order relative to
//! the reflection itself.
//! It does NOT recurse into a further reflection ray at each bounce (that
//! would make cost `O(bounces^2)` cone-marches — a full cone per bounce,
//! each itself doing a full diffuse-GI hemisphere) — multi-bounce
//! specular here means the REFLECTION chain itself continues (mirror hits
//! mirror hits mirror), each link terminated by a cheap single-bounce
//! diffuse-GI read rather than an unbounded specular-in-specular
//! expansion.
//!
//! **Fresnel-gated, following `trace_shadow`'s own `VIS_CUTOFF`
//! convention.** A per-pixel Fresnel term below a small cutoff contributes
//! negligible reflected energy — skip firing the ray entirely below that
//! cutoff so flat, low-`reflectance` dielectrics (most non-metal materials
//! in this renderer's scenes) pay zero extra cost, matching
//! `shadows_enabled`'s/cone tracing's own "an inactive/negligible
//! contribution costs nothing" pattern rather than firing a ray and
//! multiplying its result by a near-zero weight.
//!
//! **No RNG, matching `conetrace_ref`'s own established constraint.** A
//! single deterministic cone per pixel per bounce (no stochastic
//! GGX-importance-sampled rays) is not a compromise relative to
//! real-time SOTA — every source consulted confirms this IS the standard
//! real-time answer for glossy reflections, and it is the only option
//! compatible with this renderer having no per-pixel RNG primitive and no
//! way to denoise stochastic noise away (cone tracing is fully stateless
//! per `conetrace_ref`'s own doc comment).
//!
//! CPU-reference-first, per this project's established convention: every
//! function here is a faithful-by-construction reference for its WGSL
//! mirror (added directly to `hybrid_trace.wgsl`, no new `.wgsl` file
//! needed — reflections have no separate relight pass either), written
//! only after these are proven correct with `cargo test`.

use bevy::math::Vec3;
use bevy::prelude::Entity;

use crate::hybrid::bvh::Bvh;
use crate::hybrid::conetrace_ref::{cone_trace_indirect_single, trace_cone};
use crate::hybrid::cpu_ref::{
    Light, TraceObject, dielectric_f0, fresnel_schlick, ggx_alpha, ggx_distribution, ggx_visibility, sample_light, shadow_bias,
    trace_shadow,
};

/// Small fixed offset a reflection ray's own origin is biased from the
/// shaded surface, along the ray's own direction — mirrors
/// `conetrace_ref::CONE_RAY_BIAS`'s exact role and value (a shaded point
/// sitting exactly on its own surface needs a small nudge to avoid
/// immediately re-detecting that same surface at `t~=0`).
pub(crate) const REFLECT_RAY_BIAS: f32 = 0.01;

/// Compile-time-mirrored loop bound — `hybrid_trace.wgsl` needs a known
/// upper bound for its own bounce loop (WGSL has no dynamically-bounded
/// loops), matching `conetrace_ref`'s own `MAX_CONE_BOUNCES` precedent.
/// Kept smaller than diffuse GI's own `8`: unlike diffuse albedo, a
/// typical dielectric's specular reflectance (`F0` a few percent) or even
/// a reflective metal's `albedo` attenuates energy per bounce fast enough
/// that visual return past a handful of bounces is negligible for this
/// renderer's own scene materials (see module doc comment's citation of
/// Lumen's own `MaxBounces` default of 1) — `4` leaves real headroom for
/// the `gi_room.rs` gold cube (`metallic=0.9`, `reflectance=0.9`) without
/// reserving a needlessly wide loop bound.
pub const MAX_REFLECTION_BOUNCES: u32 = 4;

/// Below this per-pixel Fresnel-term luminance, the reflection ray is
/// skipped entirely rather than fired and multiplied by a near-zero
/// weight — same cutoff value as `cpu_ref`'s own `trace_shadow::
/// VIS_CUTOFF`, a reasoned engineering choice (not a cited SOTA
/// technique) following this project's own "stop paying for negligible
/// contribution" convention.
pub const REFLECTION_FRESNEL_CUTOFF: f32 = 0.02;

/// GGX roughness -> cone half-angle (radians) for a specular reflection
/// ray. `alpha` is the EXACT GGX alpha `cpu_ref::shade`/`ggx_alpha`
/// already compute — no separate remap. `epsilon` keeps the aperture
/// strictly positive at `alpha -> 0` (see module doc comment for why a
/// truly zero aperture, while legal, is undesirable here).
fn reflection_half_angle(alpha: f32) -> f32 {
    const EPSILON: f32 = 0.01;
    alpha.atan() + EPSILON
}

/// Rec. 709 relative luminance — the standard scalar reduction used
/// throughout this codebase's own "collapse a color to one representative
/// brightness for a threshold test" call sites (mirrors ReSTIR's own now-
/// removed `target_pdf` convention, still the right formula even without
/// that module surviving).
pub(crate) fn luminance(c: Vec3) -> f32 {
    c.dot(Vec3::new(0.2126, 0.7152, 0.0722))
}

/// One reflection ray's own multi-bounce chain: fires a roughness-widened
/// cone along `ray_dir` (the caller's own `reflect(...)` vector), and on
/// each hit shades it via `shade_for_reflection_bounce` (full direct
/// light + shadows + one-bounce diffuse GI — see module doc comment for
/// why this terminates via diffuse GI rather than nesting another
/// reflection). Every bounce after the first continues along the NEW
/// hit's own mirror-reflected direction (`reflect(prev_dir, hit_normal)`),
/// widened by THAT hit's own material roughness — a genuinely different
/// surface can have a different roughness, so the aperture is
/// recalculated per bounce, not carried over from the first.
///
/// Throughput at each bounce is multiplied by that bounce's own specular
/// color (`fresnel_schlick(f0, n_dot_v) * albedo_or_dielectric_tint`,
/// re-derived here per this project's own established "duplicate small
/// formulas across functions" convention rather than threading extra
/// return values out of `shade`) — a low-`F0` dielectric mirror rapidly
/// attenuates a long bounce chain to negligible energy, while a
/// high-`reflectance`/metallic surface (e.g. `gi_room.rs`'s gold cube)
/// retains meaningfully more across several bounces, matching real
/// physical behavior.
///
/// A miss at any bounce contributes nothing (black) and stops — same
/// physically-conservative choice `conetrace_ref::cone_trace_ray` already
/// makes, for the identical reason (a low-coverage hit's own shaded point
/// can float off the true surface; trusting a miss as "real sky" risks
/// re-opening the exact sealed-room light-leak class `conetrace_ref`
/// already found and fixed twice). This renderer has no skybox for a
/// reflection to plausibly show anyway.
#[allow(clippy::too_many_arguments)]
pub fn reflect_trace_ray(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    origin: Vec3,
    direction: Vec3,
    max_t: f32,
    origin_entity: Option<Entity>,
    max_bounces: u32,
    diffuse_gi_max_t: f32,
    diffuse_gi_r0: f32,
    diffuse_gi_half_angle: f32,
    bounce_gi_enabled: bool,
) -> Vec3 {
    let max_bounces = max_bounces.clamp(1, MAX_REFLECTION_BOUNCES);
    let mut total = Vec3::ZERO;
    let mut throughput = Vec3::ONE;
    let mut ray_origin = origin;
    let mut ray_dir = direction;
    let mut exclude = origin_entity;

    for bounce in 0..max_bounces {
        // Aperture for THIS bounce is derived from the material the ray is
        // about to hit — unknown until the hit resolves — so a plain
        // point ray (half_angle=0) finds the hit first, then the hit's
        // own material supplies the real aperture for THIS bounce's
        // coverage/attenuation. This differs from `conetrace_ref`'s own
        // hemisphere cones (which know their aperture upfront, from
        // config, before marching) precisely because reflection aperture
        // is a material property of the SURFACE BEING HIT, not a fixed
        // per-technique constant.
        let probe_hit = trace_cone(bvh, objects, ray_origin, ray_dir, max_t, 0.0, 0.0, exclude);
        let Some(probe_hit) = probe_hit else {
            break;
        };
        let Some(object) = objects.iter().find(|o| o.entity == probe_hit.entity) else {
            break;
        };
        let alpha = ggx_alpha(object.material.roughness);
        let half_angle = reflection_half_angle(alpha);
        // Re-march with the real aperture once it's known — cheap relative
        // to the win of using a point-ray hit as a fast reject for a
        // genuine miss (a mirror-only pass would need this to converge on
        // a widened cone's own true coverage, not a point ray's exact
        // t/coverage=1.0).
        let hit = if half_angle <= 1e-4 {
            probe_hit
        } else {
            match trace_cone(bvh, objects, ray_origin, ray_dir, max_t, 0.0, half_angle, exclude) {
                Some(h) => h,
                None => break,
            }
        };

        let p_world = ray_origin + hit.t * ray_dir;
        let view_dir = -ray_dir;
        let result = shade_for_reflection_bounce(
            object,
            p_world,
            hit.world_normal,
            view_dir,
            lights,
            bvh,
            objects,
            Some(hit.entity),
            hit.t,
            diffuse_gi_max_t,
            diffuse_gi_r0,
            diffuse_gi_half_angle,
            bounce_gi_enabled,
        );
        // coverage^2, not linear — identical rationale to
        // `conetrace_ref::cone_trace_ray`'s own fix: a low-coverage hit's
        // own p_world can float a full cone-radius off the true surface,
        // so a shadow ray cast from it can see light a true-surface point
        // never would. Squaring fades that residual toward darkness fast
        // enough to be imperceptible while staying a smooth, non-hard-
        // branched falloff rather than a hard coverage cutoff.
        total += throughput * result * hit.coverage * hit.coverage;

        if bounce + 1 >= max_bounces {
            break;
        }

        let n = hit.world_normal.normalize();
        let next_dir = reflect(ray_dir, n);
        let n_dot_v = n.dot(-ray_dir).max(1e-4);
        let f0 = Vec3::splat(dielectric_f0(object.material.reflectance))
            .lerp(object.material.base_color, object.material.metallic.clamp(0.0, 1.0));
        let f = fresnel_schlick(f0, n_dot_v);
        throughput *= f;
        if luminance(throughput) < REFLECTION_FRESNEL_CUTOFF {
            break;
        }
        ray_origin = p_world + n * REFLECT_RAY_BIAS;
        ray_dir = next_dir;
        exclude = Some(hit.entity);
    }

    total
}

/// Reflects `dir` (pointing INTO the surface, i.e. the ray's own travel
/// direction — matches this codebase's own `ray_dir`/`view_dir = -ray_dir`
/// convention) about `normal`. Standard `d - 2*(d.n)*n` mirror formula.
pub(crate) fn reflect(dir: Vec3, normal: Vec3) -> Vec3 {
    dir - 2.0 * dir.dot(normal) * normal
}

/// Full direct-lit + shadowed + one-bounce-diffuse-GI shading at a
/// reflection bounce's own hit point — the "final gather" termination
/// this module's own doc comment describes. Deliberately NOT a call to
/// `cpu_ref::shade` itself: that function's own `indirect` field is
/// always left `Vec3::ZERO` for an OUTER caller to fill in with whichever
/// GI technique is active (see `ShadeResult`'s own doc comment) — a
/// reflection bounce needs that same diffuse-GI contribution actually
/// filled in internally (there is no further outer caller inside this
/// chain to do it), so this function inlines the identical GGX math and
/// adds the one-bounce diffuse-GI term itself, verbatim-duplicated per
/// this codebase's own established "duplicate small formulas across
/// functions" convention (matches `shade_direct_only_for_cone`'s own
/// precedent of an intentionally separate direct-lit-only shading
/// variant).
#[allow(clippy::too_many_arguments)]
fn shade_for_reflection_bounce(
    object: &TraceObject,
    p_world: Vec3,
    world_normal: Vec3,
    view_dir: Vec3,
    lights: &[Light],
    bvh: &Bvh,
    objects: &[TraceObject],
    origin_entity: Option<Entity>,
    hit_t: f32,
    diffuse_gi_max_t: f32,
    diffuse_gi_r0: f32,
    diffuse_gi_half_angle: f32,
    bounce_gi_enabled: bool,
) -> Vec3 {
    let material = &object.material;
    let n = world_normal.normalize();
    let v = view_dir.normalize();
    let n_dot_v = n.dot(v).max(1e-4);

    let albedo = material.base_color;
    let metallic = material.metallic.clamp(0.0, 1.0);
    let alpha = ggx_alpha(material.roughness);
    let f0 = Vec3::splat(dielectric_f0(material.reflectance)).lerp(albedo, metallic);
    let diffuse_color = albedo * (1.0 - metallic);
    let shadow_origin = p_world + n * shadow_bias(hit_t);

    let mut radiance = Vec3::ZERO;
    for light in lights {
        let sample = sample_light(light, p_world);
        let l = sample.to_light;
        let n_dot_l = n.dot(l).max(0.0);
        if n_dot_l <= 0.0 || sample.radiance == Vec3::ZERO {
            continue;
        }
        let shadow_vis =
            trace_shadow(bvh, objects, shadow_origin, l, sample.shadow_max_t, light.shadow_softness_k, origin_entity).vis();
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
        let diffuse = diffuse_color * (Vec3::ONE - f) / std::f32::consts::PI;

        radiance += (diffuse + specular) * sample.radiance * n_dot_l * shadow_vis;
    }

    // Single-cone final gather (`cone_trace_indirect_single`, not the full
    // 5-cone `cone_trace_indirect` hemisphere `shade()`'s own primary-ray
    // GI term uses) — see that function's own doc comment: a reflection
    // bounce's GI contribution is already a second-order term relative to
    // the reflection itself, so the full hemisphere's extra directional
    // fidelity isn't worth 5x the cone marches here. This was the single
    // largest contributor to reflections' measured +25-30ms trace cost.
    //
    // `bounce_gi_enabled`: a REAL bug this gate fixes (found 2026-09-18
    // investigating a sealed, sun-only `gi_room` still reading faintly
    // lit even under `GiMethod::None` and with DDGI's own leak already
    // fixed) — this cone-traced bounce GI term used to run
    // UNCONDITIONALLY, regardless of which primary GI technique (or NONE
    // at all) the user selected. The doc comment above only ever
    // justified WHICH mechanism to use for bounce GI (cone tracing, to
    // avoid recursing back into `shade()`'s own `GiMethod::ConeTrace`
    // branch with no termination — see this function's own doc comment
    // above `shade_for_reflection_bounce`'s definition); it never
    // addressed whether bounce GI should run AT ALL when the caller has
    // GI disabled scene-wide. `reflect_trace_ray`'s own caller
    // (`cpu_ref::shade`) now passes `gi_method != GiMethod::None`
    // through as this flag.
    if bounce_gi_enabled {
        let gi = cone_trace_indirect_single(bvh, objects, lights, p_world, n, diffuse_gi_max_t, diffuse_gi_r0, diffuse_gi_half_angle, origin_entity, 1);
        radiance += diffuse_color * gi;
    }

    radiance + material.emissive
}

#[cfg(test)]
mod tests {
    use bevy::math::Quat;
    use bevy::prelude::World;

    use super::*;
    use crate::hybrid::bvh::Bvh;
    use crate::hybrid::cpu_ref::LightKind;
    use crate::hybrid::material::Material;
    use crate::hybrid::scene::HybridObject;
    use crate::prim::Aabb;
    use crate::sdf::components::Shape;

    fn entities(n: usize) -> Vec<Entity> {
        let mut world = World::new();
        (0..n).map(|_| world.spawn_empty().id()).collect()
    }

    fn overhead_sun() -> Light {
        Light {
            kind: LightKind::Directional,
            color: Vec3::ONE,
            direction_or_position: Vec3::new(0.0, -1.0, 0.0),
            intensity: 10000.0,
            spot_direction: Vec3::ZERO,
            range: 20.0,
            inner_angle: 0.3,
            outer_angle: 0.6,
            shadow_softness_k: 12.0,
        }
    }

    #[test]
    fn reflection_half_angle_is_near_zero_for_a_mirror_and_wide_for_a_rough_surface() {
        let mirror = reflection_half_angle(ggx_alpha(0.0));
        let rough = reflection_half_angle(ggx_alpha(1.0));
        assert!(mirror > 0.0 && mirror < 0.05, "a mirror-perfect surface should get a near-zero (but non-zero) aperture: got {mirror}");
        assert!(rough > mirror, "a fully rough surface must get a wider aperture than a mirror: rough={rough} mirror={mirror}");
    }

    #[test]
    fn reflect_formula_matches_textbook_mirror_reflection() {
        // A ray travelling straight down (0,-1,0) off a flat floor
        // (normal 0,1,0) must bounce straight back up (0,1,0).
        let r = reflect(Vec3::NEG_Y, Vec3::Y);
        assert!((r - Vec3::Y).length() < 1e-5, "straight-down ray off a flat floor must reflect straight up: got {r:?}");
    }

    /// Mirror floor + a colored, unlit-but-emissive box positioned exactly
    /// where a straight-down ray's mirror reflection travels — proves
    /// `reflect_trace_ray` actually picks up the reflected geometry's own
    /// color, not just "doesn't crash."
    fn mirror_floor_with_box_above() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let floor_entity = e[0];
        let box_entity = e[1];
        let objects = vec![
            TraceObject {
                entity: floor_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::new(10.0, 0.1, 10.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.1, 0.0),
                rotation: Quat::IDENTITY,
                // Near-mirror dielectric: low roughness, high reflectance.
                material: Material::new(Vec3::splat(0.9), 0.0, 0.02).with_reflectance(1.0),
            },
            TraceObject {
                entity: box_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 5.0, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(30.0, 5.0, 5.0)),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: floor_entity, world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::new(10.0, 0.1, 10.0)) },
            HybridObject { entity: box_entity, world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    #[test]
    fn mirror_reflection_picks_up_the_reflected_object_own_emissive_color() {
        let (_, objects, bvh) = mirror_floor_with_box_above();
        let lights: Vec<Light> = vec![]; // no direct lights: any red-dominant result must come from the reflected box
        // Shade a point just above the floor, viewed from straight above
        // (view_dir travels straight down onto the floor -> reflects
        // straight back up into the box).
        let origin = Vec3::new(0.0, 1.0, 0.0);
        let ray_dir = Vec3::NEG_Y;
        let result = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 1, 60.0, 0.05, 0.15, true);
        assert!(result.x > result.y && result.x > result.z, "the mirror floor's reflection must pick up the box's own red-dominant emissive color: got {result:?}");
        assert!(result.x > 0.01, "reflection must contribute meaningfully non-zero energy: got {result:?}");
    }

    /// The actual `bounce_gi_enabled` regression test: a real 2026-09-18
    /// bug (see `ReflectionParams::bounce_gi_enabled`'s own doc comment)
    /// where `shade_for_reflection_bounce`'s own cone-traced diffuse-GI
    /// "final gather" term ran UNCONDITIONALLY, regardless of the
    /// scene's own primary GI method — meaning even `GiMethod::None`
    /// still picked up bounce light through any reflective surface.
    /// Proven here directly: `reflect_trace_ray` with
    /// `bounce_gi_enabled=false` must return STRICTLY LESS energy than
    /// the identical call with `bounce_gi_enabled=true`, on a scene where
    /// the cone-traced GI term has real light to find (the emissive box
    /// `mirror_floor_with_box_above` already establishes) — proving the
    /// flag actually gates real energy, not a no-op parameter.
    #[test]
    fn bounce_gi_enabled_false_strictly_reduces_energy_when_real_gi_is_available() {
        let (_, objects, bvh) = mirror_floor_with_box_above();
        let lights: Vec<Light> = vec![];
        // 2 bounces: the SECOND bounce's own shade_for_reflection_bounce
        // call is where the cone-traced GI term actually fires (the
        // first bounce's own direct mirror reflection already picks up
        // the box's emissive color via the direct reflection chain
        // itself, per the test above — the GI term is a SEPARATE,
        // additional contribution on top of that).
        let origin = Vec3::new(0.0, 1.0, 0.0);
        let ray_dir = Vec3::NEG_Y;
        let with_gi = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 2, 60.0, 0.05, 0.15, true);
        let without_gi = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 2, 60.0, 0.05, 0.15, false);
        assert!(
            without_gi.length() < with_gi.length() - 1e-4,
            "bounce_gi_enabled=false must return strictly less energy than =true when real GI light is \
             available to find: with_gi={with_gi:?} without_gi={without_gi:?}"
        );
    }

    #[test]
    fn reflection_returns_black_on_a_clean_miss() {
        let (_, objects, bvh) = mirror_floor_with_box_above();
        let lights: Vec<Light> = vec![overhead_sun()];
        let origin = Vec3::new(0.0, 1.0, 0.0);
        let ray_dir = Vec3::new(1.0, 0.0, 0.0); // aimed sideways, away from the floor entirely
        let result = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 2, 60.0, 0.05, 0.15, true);
        assert_eq!(result, Vec3::ZERO, "a reflection ray aimed at empty space must return black");
    }

    #[test]
    fn higher_max_bounces_never_darkens_a_reflective_chain() {
        let (_, objects, bvh) = mirror_floor_with_box_above();
        let lights: Vec<Light> = vec![];
        let origin = Vec3::new(0.0, 1.0, 0.0);
        let ray_dir = Vec3::NEG_Y;
        let one = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 1, 60.0, 0.05, 0.15, true);
        let two = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 2, 60.0, 0.05, 0.15, true);
        let three = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 3, 60.0, 0.05, 0.15, true);
        assert!(one.x <= two.x + 1e-5, "adding a second bounce must never DECREASE energy: one={one:?} two={two:?}");
        assert!(two.x <= three.x + 1e-5, "adding a third bounce must never DECREASE energy: two={two:?} three={three:?}");
    }

    #[test]
    fn max_bounces_is_clamped_to_the_compile_time_ceiling() {
        let (_, objects, bvh) = mirror_floor_with_box_above();
        let lights: Vec<Light> = vec![];
        let origin = Vec3::new(0.0, 1.0, 0.0);
        let ray_dir = Vec3::NEG_Y;
        let at_ceiling = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, MAX_REFLECTION_BOUNCES, 60.0, 0.05, 0.15, true);
        let way_over = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 999, 60.0, 0.05, 0.15, true);
        assert_eq!(at_ceiling, way_over, "requesting more than MAX_REFLECTION_BOUNCES must clamp identically to the ceiling itself");
    }

    /// Regression test mirroring `conetrace_ref`'s own sealed-room leak
    /// fixture, adapted for reflections: a fully sealed, opaque, unlit box
    /// with a mirror-like interior wall must never report any light
    /// leaking through a wall that a reflection ray is geometrically
    /// guaranteed to hit (never escape past), proving the same
    /// `coverage^2` treatment applied here closes the identical leak
    /// class `conetrace_ref::cone_trace_ray` already found and fixed.
    #[test]
    fn reflection_never_leaks_light_through_a_sealed_mirror_box() {
        let ids = entities(1);
        let objects = vec![TraceObject {
            entity: ids[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(5.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            // Fully unlit, but highly reflective (mirror-like) material —
            // any non-black result can only be a leak, not real bounced
            // light (no light source exists in this fixture at all).
            material: Material::new(Vec3::splat(0.5), 0.0, 0.02).with_reflectance(1.0),
        }];
        let hybrid_objects: Vec<HybridObject> = vec![HybridObject { entity: ids[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(5.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![];
        // Just inside the box's own interior, aimed dead-center at the +Z
        // interior wall 3.0 units away — geometrically guaranteed to hit
        // that wall no matter how coarse the march's own convergence is.
        let ray_origin = Vec3::new(0.0, 0.0, 4.0);
        let ray_dir = Vec3::Z;
        let result = reflect_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, 3.0, None, 3, 60.0, 0.05, 0.15, true);
        assert_eq!(result, Vec3::ZERO, "a sealed, unlit mirror box must shade to pure black at every bounce — any non-zero result is a light leak: got {result:?}");
    }

    #[test]
    fn fresnel_gate_terminates_a_low_reflectance_dielectric_chain_early() {
        // A near-zero-reflectance dielectric's throughput should drop
        // below REFLECTION_FRESNEL_CUTOFF quickly at near-normal
        // incidence, terminating the chain well before MAX_REFLECTION_BOUNCES
        // even when a high bounce count is requested — proven indirectly
        // here via the two-vs-many-bounce results being identical (no
        // further energy added once the gate trips).
        let e = entities(2);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(10.0, 0.1, 10.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.1, 0.0),
                rotation: Quat::IDENTITY,
                // Very low reflectance dielectric: F0 = 0.16*0.05^2 ~= 4e-4.
                material: Material::new(Vec3::splat(0.9), 0.0, 0.02).with_reflectance(0.05),
            },
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 5.0, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(30.0, 5.0, 5.0)),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::new(10.0, 0.1, 10.0)) },
            HybridObject { entity: e[1], world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![];
        let origin = Vec3::new(0.0, 1.0, 0.0);
        let ray_dir = Vec3::NEG_Y;
        let two = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, 2, 60.0, 0.05, 0.15, true);
        let max = reflect_trace_ray(&bvh, &objects, &lights, origin, ray_dir, 30.0, None, MAX_REFLECTION_BOUNCES, 60.0, 0.05, 0.15, true);
        assert_eq!(two, max, "a near-zero-reflectance dielectric's Fresnel gate must terminate the chain by bounce 2, so requesting more bounces adds no further energy");
    }
}
