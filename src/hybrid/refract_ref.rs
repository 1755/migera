//! CPU reference for multi-bounce transmission/refraction — the
//! transmissive counterpart to `reflect_ref`'s specular reflections, using
//! the identical substrate (`conetrace_ref`'s cone-marching primitives) and
//! the identical no-RNG/final-gather/dedicated-temporal-history shape that
//! module's own doc comment already justifies at length. Only the ray
//! bends (Snell's law, not mirror reflection) and the material dial
//! (`transmission`/`ior`, not roughness alone) differ.
//!
//! **Solid-dielectric model, not thin-shell.** A transmissive object here
//! is treated as a solid volume of a single medium (glass, water, colored
//! resin) — light enters through one surface, travels through the
//! object's own interior, and exits through a second surface further
//! along, refracting at BOTH crossings (Snell's law each time) and
//! absorbing color along the interior path (Beer-Lambert, keyed off
//! `base_color` as the medium's own tint). This is the standard real-time
//! approximation for solid transmissive objects (matches glTF's own
//! `KHR_materials_volume` extension's "thickness" model) — a full
//! multi-surface light-transport simulation through arbitrary refractive
//! geometry is not real-time-tractable on this renderer's own SDF-marching
//! substrate, and this project has already made the equivalent
//! real-time-appropriate call once before for reflections (a single
//! deterministic cone per bounce, no stochastic sampling).
//!
//! **Finding the interior exit point.** Unlike a reflection ray (which
//! bounces OFF a surface into open scene space, so `trace_cone`'s normal
//! whole-scene BVH march already finds the next hit correctly), a
//! refracted ray travels genuinely INSIDE the object it just entered —
//! `trace_cone`'s `origin_entity` exclusion (skip re-hitting the surface
//! you're already standing on) is the wrong tool here, since it excludes
//! exactly one entity when what's needed is "keep marching THIS SAME
//! object's own SDF from the inside until it goes positive again," a
//! single-object interior march (`march_object_interior`, this module's
//! own function) rather than a whole-scene BVH trace.
//!
//! **Total internal reflection (TIR), not skipped.** When Snell's law
//! would require `sin(theta_t) > 1` (a shallow-enough angle inside a
//! denser medium), no real refracted ray exists — 100% of the energy
//! reflects internally instead. `refract()` returns `None` in exactly
//! this case (mirrors GLSL's own `refract()` builtin convention), and the
//! caller falls back to a mirror bounce off the interior surface,
//! continuing the SAME transmission chain rather than terminating it — a
//! real glass object's own visible "bright rim" effect at grazing angles
//! is exactly TIR happening at its far interior surface.
//!
//! **Beer-Lambert absorption**, not a flat pass-through tint: the medium
//! absorbs light exponentially with the distance traveled through it
//! (`transmittance = exp(-absorption_coefficient * distance)`), so a
//! thick slab of colored glass reads darker/more saturated than a thin
//! one of the same material — the standard physically-based volumetric
//! absorption model (Beer-Lambert law), keyed off `1 - base_color` as a
//! per-channel absorption coefficient (a WHITE `base_color` medium
//! absorbs nothing — clear glass; a colored one absorbs its
//! complementary wavelengths over distance, same "color comes from what
//! ISN'T absorbed" logic real stained glass follows).
//!
//! **Fresnel-gated energy split, matching `reflect_ref`'s own
//! convention.** At each surface crossing, `1 - fresnel_schlick(...)` is
//! the fraction of energy available to transmit (the rest already went to
//! `reflect_trace_ray`'s own separate reflection channel — see
//! `cpu_ref::ShadeResult`'s own doc comment for why reflection and
//! refraction are kept as separate accumulated fields, not summed here);
//! multiplied by the material's own `transmission` dial so a
//! `transmission < 1.0` dielectric (partially translucent, not fully
//! clear) still returns some diffuse/specular response from its surface
//! alongside partial transmission.
//!
//! **Termination via diffuse GI, not further transmission ("final
//! gather"), identical reasoning to `reflect_ref`'s own.** Each bounce's
//! EXIT point is shaded via `shade_for_refraction_bounce` — full direct
//! lighting + real shadow rays + ONE single-cone diffuse-GI sample
//! (`conetrace_ref::cone_trace_indirect_single`, not the full 5-cone
//! hemisphere — see that function's own doc comment for why: this
//! renderer's reflection final-gather already found the 5-cone version
//! roughly doubles per-pixel march cost for a term that's already
//! second-order relative to the transmission itself).
//!
//! **No RNG**, matching every other technique in this renderer.
//!
//! **Known limitation: the interior march ignores roughness.** Unlike
//! `reflect_ref::reflect_trace_ray` (which re-marches with a real
//! roughness-derived cone once the hit's material is known), the interior
//! exit march here (`march_object_interior`) always uses a point ray —
//! frosted/rough glass isn't yet visually distinct from clear glass of
//! the same transmission/ior. Left for a follow-up: widening the interior
//! march's own aperture is a genuinely separate problem from reflection's
//! (the cone would need to track TWO refracted directions diverging
//! inside the medium, not one), not a straightforward reuse of the
//! existing aperture-widening code.
//!
//! CPU-reference-first, per this project's established convention: this
//! module is a faithful-by-construction reference for its WGSL mirror
//! (added directly to `hybrid_trace.wgsl`, no new `.wgsl` file — matches
//! `reflect_ref`'s own "no separate relight pass" precedent), written
//! only after these are proven correct with `cargo test`.

use bevy::math::Vec3;
use bevy::prelude::Entity;

use crate::hybrid::bvh::Bvh;
use crate::hybrid::conetrace_ref::{cone_trace_indirect_single, trace_cone};
use crate::hybrid::cpu_ref::{
    HIT_EPSILON, Light, MAX_MARCH_STEPS, TraceObject, dielectric_f0, fresnel_schlick, ggx_alpha, ggx_distribution, ggx_visibility,
    local_distance, local_normal, sample_light, shadow_bias, trace_shadow,
};

/// Small fixed offset a refraction ray's own origin is biased from the
/// shaded surface, along the ray's own direction — mirrors
/// `reflect_ref::REFLECT_RAY_BIAS`'s exact role and value.
pub(crate) const REFRACT_RAY_BIAS: f32 = 0.01;

/// Compile-time-mirrored loop bound (see `reflect_ref::MAX_REFLECTION_BOUNCES`'s
/// own doc comment for why this needs to exist at all — WGSL has no
/// dynamically-bounded loops). Kept equal to reflection's own ceiling: a
/// transmission chain has the identical "each surface crossing loses
/// energy to Fresnel reflection and possibly absorption" attenuation
/// shape, so there's no reason for a different headroom.
pub const MAX_TRANSMISSION_BOUNCES: u32 = 4;

/// Below this per-pixel transmitted-energy luminance, the refraction ray
/// is skipped/terminated entirely rather than fired and multiplied by a
/// near-zero weight — same cutoff value and rationale as
/// `reflect_ref::REFLECTION_FRESNEL_CUTOFF`.
pub const TRANSMISSION_FRESNEL_CUTOFF: f32 = 0.02;

/// Minimum per-step advance while marching INSIDE an object (`local_distance`
/// is negative there) — sphere-tracing by the raw (negative) distance
/// wouldn't advance the ray at all near the entry surface, so the interior
/// march steps by `max(|d|, MIN_INTERIOR_STEP)` instead. Small enough not
/// to overshoot a thin object's own far wall, large enough that a march
/// through open (though nominally "interior," e.g. a thick slab's middle)
/// space converges in a bounded number of steps.
const MIN_INTERIOR_STEP: f32 = 0.01;

/// Rec. 709 relative luminance — see `reflect_ref::luminance`'s own doc
/// comment; duplicated here rather than imported since it's a one-line
/// formula and this module already imports enough from its siblings.
fn luminance(c: Vec3) -> f32 {
    c.dot(Vec3::new(0.2126, 0.7152, 0.0722))
}

/// Snell's law, vector form (mirrors GLSL/WGSL's own built-in `refract`
/// signature and convention: `i` is the INCOMING ray direction, `n` the
/// surface normal on the incoming side, `eta` the ratio `n1/n2` of the
/// medium the ray is LEAVING to the medium it's ENTERING). Returns `None`
/// on total internal reflection (`sin(theta_t) > 1`, i.e. the
/// discriminant below going negative) — see this module's own doc
/// comment for why that case isn't an error, just "no refracted ray
/// exists, use TIR instead."
fn refract(i: Vec3, n: Vec3, eta: f32) -> Option<Vec3> {
    let cos_i = -i.dot(n);
    let sin2_t = eta * eta * (1.0 - cos_i * cos_i);
    if sin2_t > 1.0 {
        return None;
    }
    let cos_t = (1.0 - sin2_t).sqrt();
    Some(eta * i + (eta * cos_i - cos_t) * n)
}

/// Marches `object`'s OWN local SDF from `ray_origin` (already known to be
/// INSIDE the object — `local_distance` negative there) forward along
/// `ray_dir` until it re-emerges (`local_distance` crosses back above
/// `-HIT_EPSILON`), returning the exit distance and the exit point's own
/// outward world-space normal. This is the single-object interior march
/// `reflect_trace_ray`'s whole-scene `trace_cone` can't do (see this
/// module's own doc comment for why) — steps by `max(|d|, MIN_INTERIOR_STEP)`
/// since sphere-tracing the raw (negative) distance would never advance
/// near the entry point.
fn march_object_interior(object: &TraceObject, ray_origin: Vec3, ray_dir: Vec3, t_max: f32) -> Option<(f32, Vec3)> {
    let inv_rotation = object.rotation.inverse();
    let mut t = 0.0;
    for _ in 0..MAX_MARCH_STEPS {
        if t > t_max {
            return None;
        }
        let p_world = ray_origin + t * ray_dir;
        let p_local = inv_rotation * (p_world - object.translation);
        let d = local_distance(&object.shape, p_local);
        if d > -HIT_EPSILON {
            let n_local = local_normal(&object.shape, p_local);
            let n_world = object.rotation * n_local;
            return Some((t, n_world));
        }
        t += d.abs().max(MIN_INTERIOR_STEP);
    }
    None
}

/// One transmission ray's own multi-bounce chain, entering `object` at
/// `entry_p_world`/`entry_normal` (the surface hit the caller already
/// resolved) along `view_dir`'s own refracted direction. Each bounce:
/// refracts INTO the medium (or falls back to an interior mirror bounce
/// on TIR), marches to the interior exit point via `march_object_interior`,
/// applies Beer-Lambert absorption over the interior distance travelled,
/// refracts back OUT at the exit surface (again falling back to TIR-driven
/// interior reflection if Snell's law demands it), shades the exit point's
/// own surroundings via `shade_for_refraction_bounce`, and — if more
/// bounces remain and the exit ray happens to re-enter transmissive
/// geometry — continues. A miss (the exit ray escapes into open scene
/// space with nothing to shade against) still contributes its own
/// diffuse-GI/direct-lit read via `shade_for_refraction_bounce`'s own call
/// signature; open air itself isn't shaded (this renderer has no skybox —
/// see `reflect_ref`'s own identical reasoning).
#[allow(clippy::too_many_arguments)]
pub fn refract_trace_ray(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    entry_p_world: Vec3,
    entry_normal: Vec3,
    view_dir: Vec3,
    object: &TraceObject,
    max_t: f32,
    max_bounces: u32,
    diffuse_gi_max_t: f32,
    diffuse_gi_r0: f32,
    diffuse_gi_half_angle: f32,
    bounce_gi_enabled: bool,
) -> Vec3 {
    let max_bounces = max_bounces.clamp(1, MAX_TRANSMISSION_BOUNCES);
    let mut total = Vec3::ZERO;
    let mut throughput = Vec3::ONE;
    let mut cur_object = *object;
    let mut cur_p_world = entry_p_world;
    let mut cur_normal = entry_normal.normalize();
    let mut cur_incoming = -view_dir.normalize();

    for bounce in 0..max_bounces {
        let ior = cur_object.material.ior.max(1.0001);
        let entering_eta = 1.0 / ior;
        let n_dot_i = cur_normal.dot(cur_incoming);
        // `local_distance`'s normal always points OUTWARD; if the ray is
        // travelling roughly WITH the normal it's exiting, not entering —
        // relevant on later bounces (see below) but irrelevant on bounce 0
        // (the caller always passes an entry surface).
        let (n_for_refract, eta) = if n_dot_i < 0.0 {
            (cur_normal, entering_eta) // entering: ray opposes outward normal
        } else {
            (-cur_normal, ior) // exiting into a denser-to-less-dense boundary from inside
        };

        let refracted = refract(cur_incoming, n_for_refract, eta);
        let interior_dir = match refracted {
            Some(dir) => dir,
            // TIR: no refracted ray exists — the ray reflects internally
            // and stays inside the SAME medium, continuing the chain
            // rather than terminating it.
            None => reflect_about(cur_incoming, n_for_refract),
        };

        let march_origin = cur_p_world + interior_dir * REFRACT_RAY_BIAS;
        let Some((interior_t, exit_normal_outward)) = march_object_interior(&cur_object, march_origin, interior_dir, max_t) else {
            break;
        };
        let exit_p_world = march_origin + interior_dir * interior_t;

        // Beer-Lambert absorption over the interior path just traveled —
        // see module doc comment: a WHITE base_color absorbs nothing.
        let absorption = (Vec3::ONE - cur_object.material.base_color).max(Vec3::ZERO);
        let transmittance =
            Vec3::new((-absorption.x * interior_t).exp(), (-absorption.y * interior_t).exp(), (-absorption.z * interior_t).exp());
        throughput *= transmittance;

        // Refract back OUT at the exit surface.
        let exit_n_dot_i = exit_normal_outward.dot(interior_dir);
        let (exit_n_for_refract, exit_eta) =
            if exit_n_dot_i < 0.0 { (exit_normal_outward, entering_eta) } else { (-exit_normal_outward, ior) };
        let outgoing_dir = match refract(interior_dir, exit_n_for_refract, 1.0 / exit_eta) {
            Some(dir) => dir,
            None => reflect_about(interior_dir, exit_n_for_refract),
        };

        let exit_view_dir = -interior_dir;
        let result = shade_for_refraction_bounce(
            &cur_object,
            exit_p_world,
            exit_normal_outward,
            exit_view_dir,
            lights,
            bvh,
            objects,
            Some(cur_object.entity),
            interior_t,
            diffuse_gi_max_t,
            diffuse_gi_r0,
            diffuse_gi_half_angle,
            bounce_gi_enabled,
        );
        total += throughput * result;

        let f0 = dielectric_f0(cur_object.material.reflectance);
        let exit_n_dot_v = exit_normal_outward.dot(-outgoing_dir).abs().max(1e-4);
        let f = fresnel_schlick(Vec3::splat(f0), exit_n_dot_v);
        throughput *= (Vec3::ONE - f) * cur_object.material.transmission;
        if luminance(throughput) < TRANSMISSION_FRESNEL_CUTOFF {
            break;
        }

        if bounce + 1 >= max_bounces {
            break;
        }

        // Continue the chain: probe with a point ray from the exit
        // surface along the outgoing refracted direction to see whether
        // it re-enters ANOTHER (or the same) transmissive object.
        let next_origin = exit_p_world + outgoing_dir * REFRACT_RAY_BIAS;
        let Some(next_hit) = trace_cone(bvh, objects, next_origin, outgoing_dir, max_t, 0.0, 0.0, Some(cur_object.entity)) else {
            break;
        };
        let Some(next_object) = objects.iter().find(|o| o.entity == next_hit.entity) else {
            break;
        };
        if next_object.material.transmission <= 0.0 {
            // An OPAQUE next object terminates the chain — but it is
            // genuinely VISIBLE through the glass just crossed, so it
            // gets one final shade before the chain ends, mirroring
            // reflect_ref::reflect_trace_ray's own shape: a reflection
            // chain routinely terminates on an opaque surface and has
            // always shaded it (see that function's own unconditional
            // "shade whatever this bounce hit, THEN decide whether to
            // continue" structure). Without this, nothing behind a
            // single pane of glass is ever visible through refraction at
            // all — found live via examples/cornell_room.rs's own
            // "aquarium" camera (outside a sealed room, looking in
            // through one glass wall) rendering pure black: the primary
            // ray hits the glass, refracts through it, finds an opaque
            // sphere on the far side, and the chain silently discarded
            // that hit instead of shading it.
            //
            // origin_entity is next_hit.entity (the OPAQUE object being
            // shaded), NOT cur_object.entity (the glass just exited) —
            // this is the shadow ray's own self-exclusion inside
            // shade_for_refraction_bounce, and using the wrong entity
            // here would let the opaque object self-shadow at t~=0 and
            // read black again, reproducing the exact symptom this fix
            // exists to close. Point-ray probe (r0=0, half_angle=0), so
            // no coverage^2 attenuation applies here (coverage is always
            // 1.0 for a point ray) — same reasoning the exit-surface
            // shade a few lines above already relies on.
            let next_p_world = next_origin + outgoing_dir * next_hit.t;
            let opaque_result = shade_for_refraction_bounce(
                next_object,
                next_p_world,
                next_hit.world_normal,
                -outgoing_dir,
                lights,
                bvh,
                objects,
                Some(next_hit.entity),
                next_hit.t,
                diffuse_gi_max_t,
                diffuse_gi_r0,
                diffuse_gi_half_angle,
                bounce_gi_enabled,
            );
            total += throughput * opaque_result;
            break;
        }
        cur_p_world = next_origin + outgoing_dir * next_hit.t;
        cur_normal = next_hit.world_normal.normalize();
        cur_incoming = outgoing_dir;
        cur_object = *next_object;
    }

    total
}

/// Mirror-reflects `dir` about `normal` — identical formula to
/// `reflect_ref::reflect`, duplicated here (not imported) since TIR
/// fallback is an internal implementation detail of THIS module's own
/// refraction chain, not a public reflection call.
fn reflect_about(dir: Vec3, normal: Vec3) -> Vec3 {
    dir - 2.0 * dir.dot(normal) * normal
}

/// Full direct-lit + shadowed + one-bounce-diffuse-GI shading at a
/// transmission bounce's own exit point — the "final gather" termination
/// this module's own doc comment describes. Verbatim-structured like
/// `reflect_ref::shade_for_reflection_bounce` (see that function's own
/// doc comment for why this isn't a call to `cpu_ref::shade` itself).
#[allow(clippy::too_many_arguments)]
fn shade_for_refraction_bounce(
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

    // `bounce_gi_enabled`: see `reflect_ref::ReflectionParams::
    // bounce_gi_enabled`'s own doc comment for the full rationale — this
    // used to run unconditionally regardless of the scene's own primary
    // GiMethod, letting a fully sealed room read as faintly lit through
    // any transmissive object even under GiMethod::None.
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

    #[test]
    fn refract_matches_textbook_snell_law_at_normal_incidence() {
        // A ray travelling straight down into a flat surface (normal
        // straight up) must pass straight through unbent at normal
        // incidence, regardless of the ior ratio — Snell's law's own
        // degenerate case (sin(0) = 0 on both sides).
        let i = Vec3::NEG_Y;
        let n = Vec3::Y;
        let refracted = refract(i, n, 1.0 / 1.5).expect("normal incidence must never hit TIR");
        assert!((refracted - Vec3::NEG_Y).length() < 1e-4, "normal-incidence refraction must pass straight through: got {refracted:?}");
    }

    #[test]
    fn refract_bends_toward_the_normal_when_entering_a_denser_medium() {
        // A ray entering at a shallow grazing angle should bend TOWARD
        // the normal once inside the denser medium (air ior=1.0 ->
        // glass ior=1.5): the refracted ray's angle from the normal must
        // be smaller than the incoming ray's own angle.
        let i = Vec3::new(0.8, -0.6, 0.0).normalize(); // steep grazing angle
        let n = Vec3::Y;
        let refracted = refract(i, n, 1.0 / 1.5).expect("this angle must not hit TIR entering a denser medium");
        let incoming_angle_cos = (-i).dot(n);
        let refracted_angle_cos = (-refracted).dot(n);
        assert!(refracted_angle_cos > incoming_angle_cos, "entering a denser medium must bend the ray TOWARD the normal: incoming_cos={incoming_angle_cos} refracted_cos={refracted_angle_cos}");
    }

    #[test]
    fn refract_returns_none_on_total_internal_reflection() {
        // Exiting a dense medium (eta = ior/1 = 1.5) at a shallow enough
        // angle must hit TIR — no real refracted ray exists.
        let i = Vec3::new(0.99, -0.14, 0.0).normalize(); // very shallow angle from the surface
        let n = Vec3::Y;
        let refracted = refract(i, n, 1.5);
        assert!(refracted.is_none(), "a shallow enough exit angle from a denser medium must hit total internal reflection: got {refracted:?}");
    }

    /// A directional light whose rays travel BACKWARD along -Z (so
    /// `sample_light`'s own `to_light = -direction_or_position` points
    /// +Z, matching the outward normal of a cube's own far/+Z face) — an
    /// overhead sun would miss that exit face entirely (`n_dot_l <= 0`
    /// there); this one actually illuminates it, the exact exit surface
    /// this module's own straight-through fixtures use.
    fn forward_facing_sun() -> Light {
        Light {
            kind: LightKind::Directional,
            color: Vec3::ONE,
            direction_or_position: Vec3::new(0.0, 0.0, -1.0),
            intensity: 10000.0,
            spot_direction: Vec3::ZERO,
            range: 20.0,
            inner_angle: 0.3,
            outer_angle: 0.6,
            shadow_softness_k: 12.0,
        }
    }

    #[test]
    fn a_lit_scene_behind_clear_glass_contributes_real_direct_light_at_the_exit_surface() {
        // Same clear-glass-cube fixture as the leak-proof test above, but
        // WITH a real light this time and nothing emissive behind the
        // glass — any non-black result must come from shade_for_
        // refraction_bounce's own direct-lit term at the exit surface,
        // proving refraction bounces are genuinely lit, not just a raw
        // pass-through of whatever's directly behind the object.
        let e = entities(1);
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(1.0).with_ior(1.5),
        }];
        let hybrid_objects: Vec<HybridObject> =
            vec![HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(1.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![forward_facing_sun()];
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let result = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, true);
        assert!(result.length() > 0.0, "a light aimed at the glass's own exit surface must contribute real non-zero direct light: got {result:?}");
    }

    /// A solid glass-like cube sitting on an unlit floor, with an
    /// emissive-red box positioned directly behind it (along the
    /// transmission ray's own straight-through path at normal incidence)
    /// — proves `refract_trace_ray` actually looks THROUGH the object and
    /// picks up geometry on the far side, not just "doesn't crash."
    fn glass_cube_with_box_behind() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let glass_entity = e[0];
        let box_entity = e[1];
        let objects = vec![
            TraceObject {
                entity: glass_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                // Clear (white base_color -> zero absorption), fully
                // transmissive, glass-like ior.
                material: Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(1.0).with_ior(1.5),
            },
            TraceObject {
                entity: box_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, 5.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(30.0, 5.0, 5.0)),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: glass_entity, world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::splat(1.0)) },
            HybridObject { entity: box_entity, world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    #[test]
    fn transmission_looks_through_clear_glass_to_the_object_behind_it() {
        let (_, objects, bvh) = glass_cube_with_box_behind();
        let lights: Vec<Light> = vec![]; // no direct lights: any red-dominant result must come from the box behind the glass
        // Entering the front face of the glass cube (-Z side), travelling
        // +Z straight through toward the emissive box behind it.
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z; // camera looking in +Z direction (ray travels +Z into the surface)
        let result = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, true);
        assert!(result.x > result.y && result.x > result.z, "looking through clear glass at a red-emissive box must pick up its red-dominant color: got {result:?}");
        assert!(result.x > 0.01, "transmission must contribute meaningfully non-zero energy: got {result:?}");
    }

    /// The actual `bounce_gi_enabled` regression test — `refract_ref`'s
    /// own counterpart to `reflect_ref::bounce_gi_enabled_false_
    /// strictly_reduces_energy_when_real_gi_is_available` (see that
    /// test's own doc comment for the full 2026-09-18 bug rationale:
    /// `shade_for_refraction_bounce`'s own cone-traced diffuse-GI term
    /// used to run unconditionally regardless of the scene's own primary
    /// GI method). Proven the identical way: `bounce_gi_enabled=false`
    /// must return strictly less energy than `=true` on a scene with
    /// real GI light available (the emissive box `glass_cube_with_box_
    /// behind` already establishes).
    #[test]
    fn bounce_gi_enabled_false_strictly_reduces_energy_when_real_gi_is_available() {
        let (_, objects, bvh) = glass_cube_with_box_behind();
        let lights: Vec<Light> = vec![];
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let with_gi = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, true);
        let without_gi = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, false);
        assert!(
            without_gi.length() < with_gi.length() - 1e-4,
            "bounce_gi_enabled=false must return strictly less energy than =true when real GI light is \
             available to find: with_gi={with_gi:?} without_gi={without_gi:?}"
        );
    }

    #[test]
    fn colored_glass_absorbs_the_complementary_channels_over_distance() {
        let e = entities(2);
        let glass_entity = e[0];
        let box_entity = e[1];
        let objects = vec![
            TraceObject {
                entity: glass_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(3.0), corner_radius: 0.0 },
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                // Green-tinted glass: red/blue channels absorbed over distance.
                material: Material::new(Vec3::new(0.2, 0.95, 0.2), 0.0, 0.0).with_transmission(1.0).with_ior(1.5),
            },
            TraceObject {
                entity: box_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, 8.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::splat(30.0)),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: glass_entity, world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::splat(3.0)) },
            HybridObject { entity: box_entity, world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![];
        let entry_p_world = Vec3::new(0.0, 0.0, -3.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let result = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, true);
        assert!(result.y > result.x && result.y > result.z, "a thick slab of green-tinted glass viewing a white light source must read green-dominant: got {result:?}");
    }

    #[test]
    fn transmission_returns_black_when_nothing_lies_behind_the_glass() {
        let e = entities(1);
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(1.0).with_ior(1.5),
        }];
        let hybrid_objects: Vec<HybridObject> =
            vec![HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(1.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![]; // fully unlit: any non-black result would be a leak, not real light
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let result = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, true);
        assert_eq!(result, Vec3::ZERO, "unlit glass with nothing behind it and no light of its own must shade to pure black");
    }

    /// A directional light aimed from the SIDE (+X-ish) and slightly
    /// forward toward +Z (`direction_or_position = (1,0,0.3)`, so
    /// `to_light = (-1,0,-0.3)` normalized), illuminating the rear box's
    /// near/-Z-facing surface — the face actually seen by a ray that
    /// refracts through the glass at z=0 and continues toward the box at
    /// z=5 — WITHOUT the box's own shadow ray having to cross back
    /// through the glass panel itself to reach the light. This matters
    /// because `trace_shadow` is a pure hard-geometry occlusion test with
    /// no concept of transmission at all (a glass panel is just as opaque
    /// to a shadow ray as any other object) — a light placed straight
    /// behind the camera (`direction_or_position = (0,0,1)`, `to_light =
    /// (0,0,-1)`) would have the box's own shadow ray march straight back
    /// through the glass at z=0 and read as occluded, silently testing
    /// "does light through glass reach a shadow ray" (a real, separate,
    /// NOT-fixed-here limitation) instead of "is the new final-gather
    /// shade wired correctly." Aiming from the side keeps the shadow
    /// ray's own path clear of the glass entirely (verified: at the
    /// glass's own z-depth the shadow ray's x-offset is ~10 units, far
    /// outside the panel's x∈[-1,1] footprint).
    fn side_lit_sun() -> Light {
        Light {
            kind: LightKind::Directional,
            color: Vec3::ONE,
            direction_or_position: Vec3::new(1.0, 0.0, 0.3).normalize(),
            intensity: 10000.0,
            spot_direction: Vec3::ZERO,
            range: 20.0,
            inner_angle: 0.3,
            outer_angle: 0.6,
            shadow_softness_k: 12.0,
        }
    }

    /// Clear glass panel with a NON-EMISSIVE blue opaque box directly
    /// behind it — unlike `glass_cube_with_box_behind`'s emissive rear
    /// box (which was already reachable through `shade_for_refraction_
    /// bounce`'s own GI cone term even before the opaque-termination fix
    /// below), this box's ONLY possible color source is a direct final-
    /// gather shade on the opaque hit itself — the exact "aquarium"
    /// scenario (examples/cornell_room.rs: camera outside a sealed room,
    /// looking in through one glass wall at ordinary lit opaque
    /// furniture) that rendered pure black before this fix.
    fn glass_panel_with_lit_opaque_box_behind() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let glass_entity = e[0];
        let box_entity = e[1];
        let objects = vec![
            TraceObject {
                entity: glass_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(1.0).with_ior(1.5),
            },
            TraceObject {
                entity: box_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, 5.0),
                rotation: Quat::IDENTITY,
                // Opaque (transmission defaults to 0.0/unset), blue, NOT
                // emissive — only a direct shade can ever reveal it.
                material: Material::new(Vec3::new(0.1, 0.1, 0.9), 0.0, 0.5),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: glass_entity, world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::splat(1.0)) },
            HybridObject { entity: box_entity, world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    #[test]
    fn transmission_shades_an_opaque_object_behind_the_glass() {
        // Regression test for the "aquarium" bug: an opaque, lit, non-
        // emissive object directly behind a single pane of glass must be
        // visible through refraction — before this fix, refract_trace_ray
        // discarded the chain's next-hit entirely once it found an
        // opaque object, contributing nothing for it.
        let (_, objects, bvh) = glass_panel_with_lit_opaque_box_behind();
        let lights: Vec<Light> = vec![side_lit_sun()];
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        // bounce_gi_enabled=false isolates the new direct-lit opaque
        // shade from the (already-tested-elsewhere) GI cone term.
        let result = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, false);
        assert!(result.length() > 0.0, "an opaque lit object behind glass must be visible through refraction, not black: got {result:?}");
        assert!(result.z > result.x, "must pick up the rear box's own blue-dominant color: got {result:?}");
    }

    #[test]
    fn opaque_object_behind_glass_stays_black_when_unlit() {
        // The leak-regression counterpart to the test above: with NO
        // lights at all, the same opaque object behind the same glass
        // must still shade to pure black — any non-zero result here
        // would mean the new final-gather shade manufactures light
        // rather than genuinely finding it, the exact bug class commit
        // 646a388 fixed four instances of elsewhere in this renderer.
        let (_, objects, bvh) = glass_panel_with_lit_opaque_box_behind();
        let lights: Vec<Light> = vec![];
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let result = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, false);
        assert_eq!(result, Vec3::ZERO, "an unlit opaque object behind glass must still shade to pure black — any non-zero result is a leak: got {result:?}");
    }

    #[test]
    fn max_bounces_is_clamped_to_the_compile_time_ceiling() {
        let (_, objects, bvh) = glass_cube_with_box_behind();
        let lights: Vec<Light> = vec![];
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let at_ceiling = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, MAX_TRANSMISSION_BOUNCES, 60.0, 0.05, 0.15, true);
        let way_over = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 999, 60.0, 0.05, 0.15, true);
        assert_eq!(at_ceiling, way_over, "requesting more than MAX_TRANSMISSION_BOUNCES must clamp identically to the ceiling itself");
    }

    #[test]
    fn zero_transmission_dial_terminates_the_chain_after_the_first_exit() {
        // transmission=0.0 on a material with a nonzero `ior`/shape setup
        // still resolves ONE exit (the interior march itself doesn't
        // check the dial), but throughput must drop to exactly zero
        // immediately after — a second bounce (if requested) must add
        // nothing further.
        let e = entities(2);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::ZERO,
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ONE, 0.0, 0.0).with_transmission(0.0).with_ior(1.5),
            },
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, 5.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(30.0, 5.0, 5.0)),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::splat(1.0)) },
            HybridObject { entity: e[1], world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![];
        let entry_p_world = Vec3::new(0.0, 0.0, -1.0);
        let entry_normal = Vec3::NEG_Z;
        let view_dir = Vec3::NEG_Z;
        let two = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, 2, 60.0, 0.05, 0.15, true);
        let max = refract_trace_ray(&bvh, &objects, &lights, entry_p_world, entry_normal, view_dir, &objects[0], 30.0, MAX_TRANSMISSION_BOUNCES, 60.0, 0.05, 0.15, true);
        assert_eq!(two, max, "a zero-transmission dial must terminate the chain after the first exit's own throughput cutoff, adding no further energy regardless of requested bounce count");
    }
}
