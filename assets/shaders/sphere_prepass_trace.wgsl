// Write-and-throw spike: a hardcoded sphere + ground plane, ray-traced by a
// compute pass. Writes hit distance + world-space normal to two scratch
// storage textures (as before), PLUS a genuine ray-traced reflection color
// baked at each hit point — real reflections (reflect() + re-intersect
// against the analytic scene + light shading), not a screen-space
// approximation. `sphere_prepass_blit.wgsl` packs the reflection into the
// deferred G-buffer's emissive channel; stock deferred lighting still adds
// direct diffuse/specular/shadows from the scene's real lights on top, so
// this stays as close to "let Bevy's own pipeline do the shading" as
// possible — only the reflection bounce is hand-rolled, since screen-space
// reflections and full custom shading were both explicitly ruled out in
// favor of this middle ground.
//
// No BVH, no CSG, no ObjectGpu — see this spike's earlier doc comments for
// why: the point is Bevy's REAL prepass/deferred G-buffer textures
// downstream, not re-proving this project's existing SDF/BVH pipeline.
//
// Ray generation is copied verbatim from `hybrid_trace.wgsl`'s `trace()`.
//
// Output:
//   scratch_t:        r32float,    linear hit distance along the primary ray (SKY_T = miss)
//   scratch_normal:   rgba32float, world-space normal in .xyz (unused .w)
//   scratch_reflect:  rgba32float, ray-traced reflection color in .rgb (unused .w)

#import bevy_render::view::View

@group(0) @binding(0) var<uniform> view: View;

// `--stress N` (see examples/prepass_write_sphere_orbit.rs): N copies of the
// sphere+plane scene on a grid, all sharing one material/radius/half_size —
// only each instance's world-space center differs (see this project's
// AskUserQuestion decision: identical material across copies, position-only
// variation, to keep this a small fixed-size uniform array rather than a
// full per-instance material array/storage buffer).
const MAX_STRESS_INSTANCES: u32 = 16u;

struct SphereUniform {
    radius: f32,
    base_color: vec3<f32>,
    metallic: f32,
    roughness: f32,
    reflectance: f32,
    res_x: f32,
    res_y: f32,
    instance_count: u32,
    centers: array<vec4<f32>, MAX_STRESS_INSTANCES>, // .xyz used, .w padding
}

struct PlaneUniform {
    half_size: f32,
    base_color: vec3<f32>,
    roughness: f32,
    instance_count: u32,
    // .x = world-space center X, .y = world-space Y (plane height),
    // .z = world-space center Z, .w unused padding. Each plane instance is
    // fully self-describing (NOT paired with sphere.centers by array
    // index) so a --stress grid stays correct even if the sphere and plane
    // query iteration orders ever diverge.
    instances: array<vec4<f32>, MAX_STRESS_INSTANCES>,
}

struct ProbeLight {
    position: vec3<f32>,
    color: vec3<f32>,
    intensity: f32,
}

const MAX_PROBE_LIGHTS: u32 = 4u;

struct ProbeLightsUniform {
    count: u32,
    lights: array<ProbeLight, MAX_PROBE_LIGHTS>,
}

@group(1) @binding(0) var<uniform> sphere: SphereUniform;
@group(1) @binding(1) var<uniform> plane: PlaneUniform;
@group(1) @binding(2) var<uniform> lights: ProbeLightsUniform;
@group(1) @binding(3) var scratch_t: texture_storage_2d<r32float, write>;
@group(1) @binding(4) var scratch_normal: texture_storage_2d<rgba32float, write>;
@group(1) @binding(5) var scratch_reflect: texture_storage_2d<rgba32float, write>;

const SKY_T: f32 = 400.0;
const SKY_COLOR: vec3<f32> = vec3<f32>(0.05, 0.055, 0.07);

struct Hit {
    t: f32,
    // 0 = sphere, 1 = plane, 2 = miss.
    kind: u32,
    // Which grid instance (index into sphere.centers / plane.ys) this hit
    // belongs to — undefined when kind == 2 (miss).
    instance: u32,
}

fn intersect_sphere_at(ro: vec3<f32>, rd: vec3<f32>, center: vec3<f32>) -> f32 {
    let oc = ro - center;
    let b = dot(oc, rd);
    let c = dot(oc, oc) - sphere.radius * sphere.radius;
    let disc = b * b - c;
    if (disc < 0.0) {
        return SKY_T;
    }
    let sq = sqrt(disc);
    let t0 = -b - sq;
    let t1 = -b + sq;
    if (t0 > 0.001) {
        return t0;
    }
    if (t1 > 0.001) {
        return t1;
    }
    return SKY_T;
}

// Finite square plane (not an infinite one) so a reflection ray that exits
// the ground plate's footprint correctly misses it instead of reflecting an
// unbounded floor.
fn intersect_plane_at(ro: vec3<f32>, rd: vec3<f32>, y: f32, center_xz: vec2<f32>) -> f32 {
    if (abs(rd.y) < 1e-5) {
        return SKY_T;
    }
    let t = (y - ro.y) / rd.y;
    if (t < 0.001) {
        return SKY_T;
    }
    let p = ro + rd * t;
    if (abs(p.x - center_xz.x) > plane.half_size || abs(p.z - center_xz.y) > plane.half_size) {
        return SKY_T;
    }
    return t;
}

// Nearest sphere hit across every grid instance.
fn intersect_spheres(ro: vec3<f32>, rd: vec3<f32>) -> Hit {
    var best = Hit(SKY_T, 2u, 0u);
    for (var i = 0u; i < sphere.instance_count; i = i + 1u) {
        let t = intersect_sphere_at(ro, rd, sphere.centers[i].xyz);
        if (t < best.t) {
            best = Hit(t, 0u, i);
        }
    }
    return best;
}

fn trace_scene(ro: vec3<f32>, rd: vec3<f32>) -> Hit {
    var best = intersect_spheres(ro, rd);
    for (var i = 0u; i < plane.instance_count; i = i + 1u) {
        let inst = plane.instances[i];
        let t = intersect_plane_at(ro, rd, inst.y, vec2<f32>(inst.x, inst.z));
        if (t < best.t) {
            best = Hit(t, 1u, i);
        }
    }
    return best;
}

// Analytic shadow ray: true if ANY sphere instance occludes `p`'s view of a
// light `distance` away along `l` — used by shade_lights below so a
// reflected image of a plane correctly shows every sphere's own shadow (the
// reflection bounce is a fully self-contained analytic scene, so this is a
// plain second ray-sphere intersection loop, not a shadow-map sample; see
// sphere_shadow_write.wgsl for the SEPARATE mechanism that makes each sphere
// cast a shadow into Bevy's real shadow maps for the PRIMARY, non-reflected
// view).
fn any_sphere_occludes(p: vec3<f32>, l: vec3<f32>, distance: f32) -> bool {
    for (var i = 0u; i < sphere.instance_count; i = i + 1u) {
        let oc = p - sphere.centers[i].xyz;
        let b = dot(oc, l);
        let c = dot(oc, oc) - sphere.radius * sphere.radius;
        let disc = b * b - c;
        if (disc < 0.0) {
            continue;
        }
        let sq = sqrt(disc);
        let t0 = -b - sq;
        // Only the near root matters for occlusion; require it to be
        // strictly between the shading point (past a small bias) and the
        // light itself.
        if (t0 > 0.02 && t0 < distance) {
            return true;
        }
    }
    return false;
}

// Cheap Lambertian + Blinn-Phong-ish specular against the extracted point
// lights — an approximation layered under stock deferred lighting's own
// direct-light term for the PRIMARY hit (the reflection bounce doesn't need
// to be physically identical to the main PBR path, only plausible), see this
// file's header comment for why the reflection is baked here rather than
// deferred to a second stock lighting evaluation.
fn shade_lights(p: vec3<f32>, n: vec3<f32>, view_dir: vec3<f32>, base_color: vec3<f32>, roughness: f32) -> vec3<f32> {
    var color = base_color * 0.03; // small ambient term, avoids pure-black unlit reflections
    for (var i = 0u; i < lights.count; i = i + 1u) {
        let light = lights.lights[i];
        let to_light = light.position - p;
        let dist = length(to_light);
        let l = to_light / max(dist, 1e-4);
        let ndl = max(dot(n, l), 0.0);
        if (ndl <= 0.0) {
            continue;
        }
        if (any_sphere_occludes(p, l, dist)) {
            continue;
        }
        // light.intensity is raw luminous power (lumens, matching
        // PointLight::intensity) — convert to luminous intensity
        // (lumens/steradian) the same way bevy_pbr's own extraction does
        // (light.rs's `intensity: point_light.intensity / (4.0 * PI)`) before
        // applying inverse-square falloff, so this reflection bounce's
        // brightness is in the same physical ballpark as stock deferred
        // lighting's own direct term instead of an arbitrary fudge factor.
        let luminous_intensity = light.intensity / (4.0 * 3.14159265);
        let attenuation = luminous_intensity / max(dist * dist, 0.01);
        let radiance = light.color * attenuation;
        let diffuse = base_color * radiance * ndl;
        let half_vec = normalize(l + view_dir);
        let ndh = max(dot(n, half_vec), 0.0);
        let shininess = mix(128.0, 8.0, roughness);
        let specular = radiance * pow(ndh, shininess) * (1.0 - roughness);
        color += diffuse + specular;
    }
    return color;
}

// Traces a reflection ray from `p` along `reflect(rd, n)` and shades whatever
// it hits (sphere, plane, or sky) — one bounce only, no recursion (a second
// bounce would need its own reflect-and-retrace, skipped for this spike).
fn trace_reflection(p: vec3<f32>, rd: vec3<f32>, n: vec3<f32>) -> vec3<f32> {
    let reflect_dir = reflect(rd, n);
    let origin = p + n * 0.01; // avoid immediately re-hitting the source surface
    let hit = trace_scene(origin, reflect_dir);

    var shaded: vec3<f32>;
    if (hit.kind == 2u) {
        shaded = SKY_COLOR;
    } else {
        let hit_p = origin + reflect_dir * hit.t;
        var hit_n: vec3<f32>;
        var hit_color: vec3<f32>;
        var hit_roughness: f32;
        if (hit.kind == 0u) {
            hit_n = normalize(hit_p - sphere.centers[hit.instance].xyz);
            hit_color = sphere.base_color;
            hit_roughness = sphere.roughness;
        } else {
            hit_n = vec3<f32>(0.0, 1.0, 0.0);
            hit_color = plane.base_color;
            hit_roughness = plane.roughness;
        }
        shaded = shade_lights(hit_p, hit_n, -reflect_dir, hit_color, hit_roughness);
    }

    // Soften the plane-vs-sky (or plane-vs-sphere) hit/miss transition by
    // blending toward SKY_COLOR near-grazing to the plane, instead of a hard
    // binary switch at the exact silhouette — a real rough surface would
    // blur this transition across many samples; this fakes that with a
    // single sample by fading based on how steep the reflection ray is
    // relative to the plane (shallow ray = near the horizon = blend toward
    // sky), which is cheap and targets the actual geometric discontinuity
    // directly rather than only compressing brightness after the fact.
    let grazing = smoothstep(0.0, 0.35, abs(reflect_dir.y));
    shaded = mix(SKY_COLOR, shaded, grazing);

    // Defensive ceiling against genuine numeric blowup (e.g. a reflection
    // ray landing extremely close to a point light) — the main dynamic-range
    // compression happens in sphere_prepass_blit.wgsl's Reinhard-style
    // roll-off.
    return min(shaded, vec3<f32>(200.0));
}

@compute @workgroup_size(8, 8, 1)
fn trace(@builtin(global_invocation_id) gid: vec3<u32>) {
    if (gid.x >= u32(sphere.res_x) || gid.y >= u32(sphere.res_y)) {
        return;
    }

    let uv = (vec2<f32>(f32(gid.x), f32(gid.y)) + vec2<f32>(0.5)) / vec2<f32>(sphere.res_x, sphere.res_y);
    // Match bevy uv_to_ndc: y flips (frag coord origin is top-left).
    let ndc = uv * vec2<f32>(2.0, -2.0) + vec2<f32>(-1.0, 1.0);
    let near_clip = vec4<f32>(ndc, 1.0, 1.0);
    let near_world = view.world_from_clip * near_clip;
    let near_pos = near_world.xyz / near_world.w;
    let ro = view.world_position;
    let rd = normalize(near_pos - ro);

    // Only spheres are written to the scratch depth/normal textures (planes
    // are real Mesh3d instances in the examples that use this shader, drawn
    // by Bevy's own mesh prepass — see this spike's `mod.rs` doc comment on
    // ordering for why our fullscreen write must never touch a plane's own
    // pixels). trace_scene/intersect_plane_at above exist ONLY for the
    // reflection bounce's own re-intersection math, not primary visibility.
    let hit = intersect_spheres(ro, rd);
    let px = vec2<i32>(i32(gid.x), i32(gid.y));
    textureStore(scratch_t, px, vec4<f32>(hit.t, 0.0, 0.0, 0.0));

    if (hit.t < SKY_T) {
        let p = ro + rd * hit.t;
        let n = normalize(p - sphere.centers[hit.instance].xyz);
        textureStore(scratch_normal, px, vec4<f32>(n, 0.0));

        let reflection = trace_reflection(p, rd, n);
        textureStore(scratch_reflect, px, vec4<f32>(reflection, 0.0));
    }
}
