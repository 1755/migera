//! CPU reference for SDF cone tracing — this renderer's sole indirect-
//! diffuse GI technique (see `extract::GiMethod`'s own doc comment).
//! DDGI, a hash-grid radiance cache, and ReSTIR GI were all built and
//! real-GPU-measured earlier this project's history (see PROGRESS.md's
//! own entries for each, kept as historical record) before this
//! technique was chosen as the sole survivor: cone tracing is
//! architecturally native to an SDF renderer — it marches the exact
//! same signed-distance field primary rays already use, with no spatial
//! quantization (unlike a probe grid or a hashed cell) and no temporal
//! lag (unlike all three of the removed techniques, which amortize/
//! converge across frames).
//!
//! **Stateless.** Cone tracing maintains NO persistent GPU structure at
//! all — every shaded pixel re-marches its own 5-cone hemisphere bundle
//! directly against the scene's existing BVH/SDF representation, fresh,
//! every single frame. This has a real, honest cost tradeoff worth
//! stating plainly rather than glossing over: it has NO temporal
//! amortization whatsoever — its entire cost lands in every frame's own
//! shading-time budget. The upside is a continuous, adaptive-resolution
//! result with no grid/cell quantization artifacts at all — see
//! `PROGRESS.md`'s own cone-tracing entries for real measured numbers.
//!
//! **Single-bounce only**: this renderer's `trace()`-equivalent cost is
//! the dominant expense, and cone tracing specifically has nothing to
//! amortize a second bounce's cost against (no persistent cache) — a
//! second bounce would multiply an already fully-uncached cost with no
//! offsetting benefit.
//!
//! CPU-reference-first, per this project's established convention (see
//! `mod.rs`'s own doc comment): every function here is a faithful-by-
//! construction reference for its WGSL mirror (functions added directly
//! to `hybrid_trace.wgsl` — no new `.wgsl` file needed, since cone
//! tracing has no separate relight pass to give one), written only
//! after these are proven correct with `cargo test`.

use bevy::math::{Mat3, Vec3};
use bevy::prelude::Entity;

use crate::hybrid::bvh::{Bvh, LEAF_SENTINEL};
use crate::hybrid::cpu_ref::{Light, TraceObject, local_distance, local_normal, shade, smoothstep};
use crate::prim::Aabb;

/// Duff et al.'s branchless orthonormal basis construction, `n` as the z
/// axis — numerically stable for every `n` including axis-aligned ones.
/// A private copy (this module's own established self-containment
/// convention, matching `HEMISPHERE_SAMPLES`'s own copy just below):
/// this used to be imported directly from the (now-removed) `ddgi_ref`
/// module, the one real cross-module dependency this file had before
/// DDGI/hash-grid/ReSTIR were deleted — copied here verbatim rather than
/// re-derived, since it's a known-correct, previously-tested formula.
fn cone_tangent_basis(n: Vec3) -> Mat3 {
    let s = if n.z >= 0.0 { 1.0 } else { -1.0 };
    let a = -1.0 / (s + n.z);
    let b = n.x * n.y * a;
    let t = Vec3::new(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
    let bt = Vec3::new(b, s + n.y * n.y * a, -n.y);
    Mat3::from_cols(t, bt, n)
}

/// 5 fixed cosine-weighted-ish hemisphere directions in TANGENT space —
/// this module's own private copy (no shared source remains after
/// DDGI's own removal; kept as a literal array here since it's only 5
/// lines of constant data).
const HEMISPHERE_SAMPLES: [Vec3; 5] = [
    Vec3::new(0.0, 0.0, 1.0),
    Vec3::new(0.6614, 0.0, 0.75),
    Vec3::new(-0.2044, 0.6285, 0.75),
    Vec3::new(-0.5350, -0.3886, 0.75),
    Vec3::new(0.5350, -0.3886, 0.75),
];

/// Result of one whole-scene cone march — unlike `cpu_ref::Hit` (a
/// crisp point-ray boolean hit), a cone has no crisp yes/no: near-miss
/// geometry partially inside the cone's growing footprint should
/// partially attenuate, not binarily hit-or-miss. `coverage` is that
/// continuous `[0,1]` attenuation fraction; `entity`/`t`/`world_normal`
/// are only meaningful when `coverage > 0.0` (mirrors `Hit`'s own
/// convention of fields only meaningful on a real hit).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConeHit {
    pub coverage: f32,
    pub t: f32,
    pub entity: Entity,
    pub world_normal: Vec3,
}

const MAX_CONE_MARCH_STEPS: u32 = 128;

/// Floor on the per-step advance once the cone's own effective radius
/// exceeds the raw SDF distance `d` — without this, `t += max(d -
/// radius(t), MIN_CONE_STEP)` could otherwise emit a near-zero or
/// negative step once `radius(t)` grows past `d`, stalling the march
/// indefinitely at the same `t`. Same order of magnitude as
/// `cpu_ref::HIT_EPSILON` — small enough not to visibly undershoot a
/// true surface, large enough to guarantee real forward progress every
/// iteration (see `no_stall_near_convergence`).
const MIN_CONE_STEP: f32 = 1e-3;

/// Effective hit-test radius at marched distance `t` along a cone whose
/// origin footprint is `r0` and half-angle is `half_angle` (radians) —
/// the textbook cone-radius-at-distance formula. `r0=0.0` with
/// `half_angle=0.0` degenerates this to a zero-radius point ray exactly
/// (see `cone_degenerates_to_point_ray_at_zero_half_angle`).
fn cone_radius_at(r0: f32, half_angle: f32, t: f32) -> f32 {
    r0 + t * half_angle.tan()
}

/// One cone-marched object query — a deliberate parallel sibling to
/// `cpu_ref`'s own (private) `march_object`, not a shared/parameterized
/// function: the hit test, step formula, and return type all genuinely
/// differ (mirrors this codebase's own established precedent of
/// `ddgi_ref::probe_ray` existing alongside `cpu_ref::trace` rather
/// than a shared generic, whenever semantics diverge this much).
///
/// Step size is `t += max(d - radius(t), MIN_CONE_STEP)`, NOT a plain
/// `t += d` — once the cone's own effective hit volume is wider than
/// the raw SDF distance `d`, a naive `t += d` step would overstep past
/// convergence; `MIN_CONE_STEP` prevents the step from stalling once
/// `d <= radius(t)`.
///
/// Returns `Some((t, coverage))` once `d < radius(t)` — `coverage` is a
/// SMOOTH `[0,1]` value (`smoothstep(radius(t), 0.0, d)`), not a hard
/// boolean: `d == radius(t)` (grazing near-miss, just inside the
/// cone's own footprint) gives `coverage ≈ 0`, `d <= 0.0` (dead-center
/// hit) gives `coverage = 1.0`. This smooth-not-hard shape follows this
/// project's own established discipline for continuous quantities
/// approximated by a marched/padded query (see `cpu_ref::trace_shadow`'s
/// own `margin_fade` smoothstep, and this project's own repeated
/// experience fixing hard-branch flicker bugs in DDGI's occlusion
/// fallback this session) — a hard coverage cutoff here would be the
/// identical class of bug in a new place.
///
/// Returns `None` if the march exceeds `MAX_CONE_MARCH_STEPS` or `t_max`
/// first, exactly like `march_object`'s own miss contract.
fn march_object_cone(
    object: &TraceObject,
    ray_origin: Vec3,
    ray_dir: Vec3,
    t_start: f32,
    t_max: f32,
    r0: f32,
    half_angle: f32,
) -> Option<(f32, f32)> {
    let inv_rotation = object.rotation.inverse();
    let mut t = t_start.max(0.0);
    for _ in 0..MAX_CONE_MARCH_STEPS {
        if t > t_max {
            return None;
        }
        let p_world = ray_origin + t * ray_dir;
        let p_local = inv_rotation * (p_world - object.translation);
        let d = local_distance(&object.shape, p_local);
        let radius = cone_radius_at(r0, half_angle, t);
        if d < radius {
            // radius == 0.0 (a degenerate point ray — every bounce after
            // the first, see cone_trace_ray's own doc comment) makes
            // smoothstep's edge0/edge1 both 0.0. This module's own
            // smoothstep divides through that zero and relies on Rust's
            // f32::clamp resolving the resulting +/-inf to a safe
            // 0.0/1.0 — WGSL's native smoothstep intrinsic has no such
            // guarantee (spec-indeterminate, NaN on real hardware for
            // this input), so hybrid_trace.wgsl's own mirror guards this
            // explicitly instead of relying on it; matched here so both
            // stay truly verbatim rather than agreeing by accident.
            let coverage = if radius > 0.0 { smoothstep(radius, 0.0, d).clamp(0.0, 1.0) } else { 1.0 };
            return Some((t, coverage));
        }
        t += (d - radius).max(MIN_CONE_STEP);
    }
    None
}

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

/// Slab-tests `aabb` against a cone whose footprint grows with `t`, by
/// padding with the cone's own radius AT THE NODE'S OWN NEAR DISTANCE
/// rather than a single fixed worst-case margin — see `trace_cone`'s own
/// doc comment for why this is safe. Two-pass fixed-point refinement:
/// pass 1 pads by `cone_radius_at(r0, half_angle, worst_case_margin_t)`
/// (the caller-supplied conservative bound, e.g. `t_max` or the current
/// best hit's own `t`) to get a first, definitely-safe `t_near`; since
/// `cone_radius_at` is monotonically non-decreasing in `t` (a fixed
/// `half_angle >= 0.0`), the true padding this node could ever need is
/// at most `cone_radius_at(r0, half_angle, t_near_pass1)` — pass 2
/// re-pads with THAT tighter bound and re-tests. Each pass's `t_near`
/// only shrinks or holds (never grows the amount of the scene pruned
/// away), so refining can never turn a real hit into a miss: if a
/// tighter margin ever excluded geometry the wider one wouldn't have,
/// `t_near` under it would have to increase past `t_far`, and the
/// fixed-point argument above rules that out for a monotone radius
/// function.
fn cone_slab_hit(
    ray_origin: Vec3,
    ray_dir: Vec3,
    aabb: &Aabb,
    t_query_max: f32,
    r0: f32,
    half_angle: f32,
    worst_case_margin_t: f32,
) -> (f32, f32) {
    let worst_case_margin = cone_radius_at(r0, half_angle, worst_case_margin_t);
    let padded = Aabb { min: aabb.min - Vec3::splat(worst_case_margin), max: aabb.max + Vec3::splat(worst_case_margin) };
    let (t_near, t_far) = slab_hit(ray_origin, ray_dir, &padded, t_query_max);
    if t_near > t_far {
        return (t_near, t_far);
    }
    let tight_margin = cone_radius_at(r0, half_angle, t_near);
    let tight_padded = Aabb { min: aabb.min - Vec3::splat(tight_margin), max: aabb.max + Vec3::splat(tight_margin) };
    slab_hit(ray_origin, ray_dir, &tight_padded, t_query_max)
}

/// Cone-marches the whole BVH-accelerated scene: descends via a
/// MARGIN-PADDED slab test (both leaf and internal-node AABBs expanded
/// by a per-node cone radius — see `cone_slab_hit`'s own doc comment for
/// the two-pass tightening this uses instead of a single fixed
/// worst-case margin), then closest-`t`-wins across every candidate
/// leaf's own `march_object_cone` convergence.
///
/// The padding itself mirrors `cpu_ref::gather_candidates_padded`'s own
/// margin-expansion lesson for soft shadows: an UNPADDED descent (like
/// `cpu_ref::trace`'s own tight slab test) could prune a leaf whose
/// TIGHT AABB excludes it but whose cone footprint — a radius that
/// grows with `t` — genuinely reaches it near the query's own far
/// reach. Since the cone's own radius at any `t` is exactly computable
/// in advance (unlike `trace_shadow`'s own `PENUMBRA_REACH`, a fixed
/// scale constant chosen because no such exact bound exists there), no
/// new named tuning constant is needed here — `cone_radius_at` itself
/// supplies the margin.
///
/// `origin_entity` excludes the shaded object's own entity from its own
/// cone query — mirrors `ddgi_ref::sample_probe_grid`'s/
/// `hashgrid_ref::sample_hashgrid`'s own `origin_entity` exclusion
/// (see those functions' own doc comments for the self-intersection bug
/// this fixes for a rotating object's own occlusion/cone ray).
///
#[allow(clippy::too_many_arguments)]
pub fn trace_cone(
    bvh: &Bvh,
    objects: &[TraceObject],
    ray_origin: Vec3,
    ray_dir: Vec3,
    t_max: f32,
    r0: f32,
    half_angle: f32,
    origin_entity: Option<Entity>,
) -> Option<ConeHit> {
    if bvh.nodes.is_empty() {
        return None;
    }
    let mut best: Option<ConeHit> = None;
    let mut stack = vec![0usize];
    while let Some(node_index) = stack.pop() {
        let node = bvh.nodes[node_index];
        let query_max = best.map_or(t_max, |h| h.t);
        let (t_near, t_far) = cone_slab_hit(ray_origin, ray_dir, &node.aabb, query_max, r0, half_angle, query_max);
        if t_near > t_far {
            continue;
        }
        if node.left_or_sentinel == LEAF_SENTINEL {
            if Some(node.entity) == origin_entity {
                continue;
            }
            let Some(object) = objects.iter().find(|o| o.entity == node.entity) else {
                continue;
            };
            let march_limit = best.map_or(t_far, |h| h.t.min(t_far));
            if let Some((t, coverage)) = march_object_cone(object, ray_origin, ray_dir, t_near, march_limit, r0, half_angle)
                && best.is_none_or(|h| t < h.t)
            {
                let p_world = ray_origin + t * ray_dir;
                let p_local = object.rotation.inverse() * (p_world - object.translation);
                let local_n = local_normal(&object.shape, p_local);
                let world_normal = object.rotation * local_n;
                best = Some(ConeHit { coverage, t, entity: object.entity, world_normal });
            }
            continue;
        }
        let left = node.left_or_sentinel as usize;
        let right = node.right_or_object as usize;
        let limit = best.map_or(t_max, |h| h.t);
        let left_hit = cone_slab_hit(ray_origin, ray_dir, &bvh.nodes[left].aabb, limit, r0, half_angle, limit);
        let right_hit = cone_slab_hit(ray_origin, ray_dir, &bvh.nodes[right].aabb, limit, r0, half_angle, limit);
        if left_hit.0 <= left_hit.1 {
            stack.push(left);
        }
        if right_hit.0 <= right_hit.1 {
            stack.push(right);
        }
    }
    best
}

/// One cone's own indirect-diffuse sample: cone-marches the whole scene
/// (`trace_cone`), and on a hit shades it via the SAME direct-lit-only
/// `shade` DDGI's own `probe_ray` already uses. **Multi-bounce**: up to
/// `max_bounces` hits are chained — after shading bounce N's own hit
/// with direct light only, the ray continues from that hit point
/// straight along its own surface normal (a fixed, deterministic
/// direction; this renderer has no per-pixel RNG primitive, and
/// introducing one here would add un-amortized noise with nothing to
/// temporally denoise it away, since cone tracing is fully stateless —
/// see this module's own doc comment), weighted by the accumulated
/// diffuse-albedo throughput of every bounce so far. Every bounce
/// AFTER the first fires a degenerate point ray (`r0=0.0`,
/// `half_angle=0.0`), not a full cone — keeps total cost linear in
/// `max_bounces` (`5 + (max_bounces - 1)` cones/pixel from
/// `cone_trace_indirect`'s own 5-cone hemisphere) rather than
/// exponential, a deliberate cost/quality tradeoff chosen with the user
/// directly. On a full miss at any bounce, contributes nothing (black)
/// for that bounce and stops.
///
/// **Does NOT fall back to `sky_color` on a miss** — a real, found-by-
/// direct-visual-inspection bug: `gi_room.rs`'s own fully sealed,
/// roof-closed room showed the sky's own blue gradient bleeding onto
/// the ceiling/walls at `max_bounces >= 2`, even though
/// `conetrace_ref.rs`'s own CPU-reference sweep of the identical
/// geometry never reports a miss anywhere in the sealed room — a real
/// GPU-only divergence (RADV/Mesa) in `trace_cone`'s BVH descent that
/// was investigated at length but not root-caused. `sky_color` is only
/// a meaningful fallback for a PRIMARY-style ray that can legitimately
/// see open sky; `cone_trace_ray`'s own rays (especially bounce 2+,
/// which continues along the previous hit's own surface normal) can
/// spuriously report a miss from pure GPU numerical divergence with no
/// real correspondence to "this ray genuinely reached open sky" — so
/// trusting a miss as "real sky" here is not safe. Black is the
/// physically conservative choice (under-lighting on a genuine escape
/// to sky rather than over-lighting a sealed room from a false one).
///
/// **`max_bounces == 1` is bit-for-bit identical to this function's own
/// prior single-bounce behavior** — the loop's first iteration always
/// runs the plain shade-and-return path below regardless of
/// `max_bounces`, and only derives a next-bounce ray / advances the
/// loop when `max_bounces > 1`. This is a deliberate regression
/// invariant, not an accident — every pre-existing test in this
/// module's own `mod tests` calls this function with `max_bounces: 1`
/// and must keep passing unmodified.
///
/// **Does NOT blend a hit's own shaded color toward `sky_color` by
/// `coverage`** — a real, found-by-direct-visual-inspection light-leak
/// bug in an earlier version did exactly that
/// (`result * coverage + sky_color * (1 - coverage)`), on the
/// assumption that `coverage < 1.0` meant "part of the cone's own solid
/// angle escaped past this object to open sky." It doesn't: `coverage`
/// (see `march_object_cone`'s own doc comment) is purely a per-step
/// convergence-quality signal — how close the raw SDF distance `d`
/// landed to `0` relative to `radius(t)` at the exact step marching
/// stopped — with NO relationship to whether the surrounding geometry
/// is actually open to the sky. Inside a fully sealed, opaque room, a
/// cone aimed at a nearby wall from a grazing angle can legitimately
/// converge with LOW coverage (the coarse `MIN_CONE_STEP` floor makes
/// even a genuine solid hit's own final step overshoot past the true
/// surface — see `conetrace_ref.rs`'s own `full_coverage_on_dead_center_hit`
/// test comment for the same finding in a different test), and the old
/// formula was blending in bright sky gradient for that "uncovered"
/// fraction even though the cone never actually reached open sky at
/// all — a real light leak into a scene that's provably fully sealed,
/// found via `gi_room.rs`'s own roof-closed sanity check. `did_hit ==
/// true` from `trace_cone` already means the march converged against
/// REAL geometry (a `march_object_cone` `None` — the whole march
/// exhausting `MAX_CONE_MARCH_STEPS`/`t_max` with no convergence at
/// all — is the only case that means "nothing here," and that's
/// already handled by the `None` arm below, contributing black). A
/// low-confidence convergence still found the same real wall a
/// high-confidence one would; the fix is to trust the hit fully once
/// found, not fade it toward a sky color the cone never actually saw.
/// **This applies at EVERY bounce, not just the first** — a low-
/// coverage hit anywhere in the chain gets the identical `coverage^2`
/// treatment, or the same sealed-room leak this comment describes could
/// reopen at bounce 2+ even though bounce 1 is fixed.
#[allow(clippy::too_many_arguments)]
pub fn cone_trace_ray(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    origin: Vec3,
    direction: Vec3,
    max_t: f32,
    r0: f32,
    half_angle: f32,
    origin_entity: Option<Entity>,
    max_bounces: u32,
) -> Vec3 {
    let max_bounces = max_bounces.max(1);
    let mut total = Vec3::ZERO;
    let mut throughput = Vec3::ONE;
    let mut ray_origin = origin;
    let mut ray_dir = direction;
    let mut bounce_r0 = r0;
    let mut bounce_half_angle = half_angle;

    for bounce in 0..max_bounces {
        // A miss contributes nothing (black), not sky_color — see this
        // function's own doc comment for why trusting a miss as "real
        // sky" here isn't safe.
        let Some(hit) = trace_cone(bvh, objects, ray_origin, ray_dir, max_t, bounce_r0, bounce_half_angle, origin_entity) else {
            break;
        };
        let Some(object) = objects.iter().find(|o| o.entity == hit.entity) else {
            break;
        };
        let view_dir = -ray_dir;
        let p_world = ray_origin + hit.t * ray_dir;
        let result = shade(&object.material, p_world, hit.world_normal, view_dir, lights, bvh, objects, Some(hit.entity), hit.t, None, None);
        // Scale by `coverage`, NOT a flat full-weight return — see this
        // function's own doc comment for the full light-leak rationale
        // (squared, not linear, and applied at every bounce).
        let coverage2 = hit.coverage * hit.coverage;
        total += throughput * result.direct_and_emissive * coverage2;

        let diffuse_color = object.material.base_color * (1.0 - object.material.metallic.clamp(0.0, 1.0));

        if bounce + 1 >= max_bounces {
            // Geometric-series tail: `max_bounces` truncates the chain,
            // but bounces beyond it aren't "no more light" — they're
            // "not traced". Rather than dropping that light outright
            // (making higher max_bounces settings look artificially
            // dimmer than "true" multi-bounce would) or inventing an
            // independent ambient constant (which could leak light into
            // a sealed room with no real light source at all, exactly
            // the bug the coverage^2 fix above and the room-leak
            // regression tests already guard against), approximate the
            // sum of ALL remaining un-traced bounces as a geometric
            // series using this SAME hit's own already-computed
            // direct_and_emissive and diffuse albedo as the per-bounce
            // attenuation ratio: bounce N+1 re-emits
            // `direct_and_emissive * diffuse_color`, bounce N+2 re-emits
            // `direct_and_emissive * diffuse_color^2`, etc. (the
            // homogeneous-albedo approximation this same loop's own
            // per-bounce throughput update already assumes) — summing
            // to `direct_and_emissive * diffuse_color / (1 -
            // diffuse_color)` component-wise. This is derived entirely
            // from real, already-traced light at this hit: a hit whose
            // `direct_and_emissive` is genuinely black (e.g. every
            // surface inside a sealed room with no light source)
            // contributes an exactly-zero tail, so this cannot leak
            // light into a scene that has none — it only extends light
            // that's already real. Guards `diffuse_color` away from 1.0
            // component-wise (a pure-white, fully diffuse material would
            // otherwise divide by ~0, producing an unbounded tail) —
            // same clamping spirit as `REFLECT_ROUGHNESS_GATE`'s own
            // "don't let a near-degenerate denominator blow up" pattern
            // elsewhere in this codebase.
            let safe_rho = diffuse_color.min(Vec3::splat(0.95));
            let tail_ratio = safe_rho / (Vec3::ONE - safe_rho);
            total += throughput * result.direct_and_emissive * coverage2 * tail_ratio;
            break;
        }

        // Continue toward the next bounce: diffuse-albedo-weighted
        // throughput (the standard rendering-equation term — a surface
        // only re-emits the fraction of light its own diffuse albedo
        // reflects, same `albedo * (1.0 - metallic)` formula `shade`
        // itself already computes internally, re-derived here per this
        // module's own established "duplicate small formulas across
        // functions" convention rather than threading it back out of
        // `shade`'s own return value), fixed normal-direction ray (see
        // this function's own doc comment for why this is deterministic
        // rather than randomly sampled), degenerate point ray (r0=0,
        // half_angle=0) for every bounce after the first.
        throughput *= diffuse_color;
        ray_origin = p_world + hit.world_normal * CONE_RAY_BIAS;
        ray_dir = hit.world_normal;
        bounce_r0 = 0.0;
        bounce_half_angle = 0.0;
    }

    total
}

/// Small fixed offset a cone's own origin is biased from the shaded
/// surface, along the cone's own direction — mirrors `ddgi_ref::
/// PROBE_RAY_BIAS`'s exact role and value (same rationale: a shaded
/// point sitting exactly on its own surface needs a small nudge to
/// avoid immediately re-detecting that same surface at `t~=0`).
const CONE_RAY_BIAS: f32 = 0.01;

/// Fires a fixed 5-direction hemisphere bundle (`cone_tangent_basis` +
/// `HEMISPHERE_SAMPLES`, this module's own copies per its own doc
/// comment), one `cone_trace_ray` call per direction, plain-averaged —
/// `HEMISPHERE_SAMPLES` already leans cosine-weighted by construction
/// (more samples cluster near the pole than the rim), so an additional
/// weighting term would double-count that bias rather than add real
/// energy conservation.
///
/// This is cone tracing's ENTIRE indirect-diffuse sample — no
/// persistent structure, no temporal blend: every call re-fires all 5
/// cones fresh (see this module's own doc comment on the resulting
/// structural cost disadvantage vs. DDGI/the hash-grid). `max_bounces`
/// is threaded through to every one of the 5 `cone_trace_ray` calls —
/// see that function's own doc comment for the multi-bounce design
/// (bounce 1 keeps the full cone, every bounce after fires a
/// degenerate point ray, linear not exponential cost in `max_bounces`).
#[allow(clippy::too_many_arguments)]
pub fn cone_trace_indirect(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    world_pos: Vec3,
    surface_normal: Vec3,
    max_t: f32,
    r0: f32,
    half_angle: f32,
    origin_entity: Option<Entity>,
    max_bounces: u32,
) -> Vec3 {
    let basis = cone_tangent_basis(surface_normal);
    let mut acc = Vec3::ZERO;
    for sample in HEMISPHERE_SAMPLES {
        let world_dir = (basis * sample).normalize();
        acc += cone_trace_ray(
            bvh,
            objects,
            lights,
            world_pos + world_dir * CONE_RAY_BIAS,
            world_dir,
            max_t,
            r0,
            half_angle,
            origin_entity,
            max_bounces,
        );
    }
    acc / HEMISPHERE_SAMPLES.len() as f32
}

/// Single-cone approximation of `cone_trace_indirect`, firing exactly one
/// cone straight along `surface_normal` instead of the full 5-direction
/// hemisphere bundle. Exists for call sites where the resulting GI term is
/// already a SECOND-ORDER contribution to the pixel's final color — this
/// renderer's reflection final-gather (`shade_for_reflection_bounce`) is
/// the motivating case: profiling found each reflected pixel was paying
/// for a full 5-cone hemisphere sample INSIDE a reflection bounce that
/// itself already costs 2 cone marches (probe + widened re-march), making
/// one reflected pixel cost roughly as much as the entire primary-ray GI
/// term a second time. A single cone along the normal is the standard
/// "one ambient probe" reduction for a contribution the eye is not
/// expected to resolve directional detail in (unlike primary-ray GI,
/// which IS the dominant indirect term for most on-screen pixels and
/// keeps the full hemisphere). Reuses `cone_trace_ray` verbatim — only
/// the number of directions sampled differs from `cone_trace_indirect`.
#[allow(clippy::too_many_arguments)]
pub fn cone_trace_indirect_single(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    world_pos: Vec3,
    surface_normal: Vec3,
    max_t: f32,
    r0: f32,
    half_angle: f32,
    origin_entity: Option<Entity>,
    max_bounces: u32,
) -> Vec3 {
    let world_dir = surface_normal.normalize();
    cone_trace_ray(bvh, objects, lights, world_pos + world_dir * CONE_RAY_BIAS, world_dir, max_t, r0, half_angle, origin_entity, max_bounces)
}

#[cfg(test)]
mod tests {
    use bevy::math::Quat;
    use bevy::prelude::{Entity, World};

    use super::*;
    use crate::hybrid::bvh::Bvh;
    use crate::hybrid::cpu_ref::LightKind;
    use crate::hybrid::material::Material;
    use crate::hybrid::scene::HybridObject;
    use crate::sdf::components::Shape;

    fn entities(n: usize) -> Vec<Entity> {
        let mut world = World::new();
        (0..n).map(|_| world.spawn_empty().id()).collect()
    }

    /// Same fixture shape as `ddgi_ref::tests::ground_and_box`/
    /// `hashgrid_ref::tests::ground_and_box` (a flat ground plate + one
    /// small box above it), rebuilt fresh here since those modules' own
    /// test helpers are private to their own `mod tests`, unreachable
    /// from here.
    fn ground_and_box() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(20.0, 0.2, 20.0), corner_radius: 0.0 },
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
            .map(|o| {
                let half = match o.shape {
                    Shape::RoundedBox { half_extents, .. } => half_extents,
                    _ => unreachable!(),
                };
                HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, half) }
            })
            .collect();
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
    }

    /// A single isolated small sphere, far from anything else — used by
    /// tests that need a clean, unambiguous single-object target (the
    /// half-angle-smoothing and coverage tests specifically want no
    /// ground plate nearby to confound the result).
    fn lone_sphere() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(1);
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::Sphere { radius: 1.0 },
            translation: Vec3::new(0.0, 0.0, 10.0),
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::new(0.8, 0.2, 0.2), 0.0, 0.5),
        }];
        let hybrid_objects: Vec<HybridObject> =
            vec![HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::splat(1.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        (e, objects, bvh)
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
    fn cone_radius_at_grows_linearly_with_t() {
        assert_eq!(cone_radius_at(0.1, 0.0, 5.0), 0.1);
        let r = cone_radius_at(0.0, std::f32::consts::FRAC_PI_4, 1.0);
        assert!((r - 1.0).abs() < 1e-4, "tan(pi/4) == 1.0, so radius at t=1 should be ~1.0: got {r}");
    }

    #[test]
    fn cone_degenerates_to_point_ray_at_zero_half_angle() {
        // THE correctness anchor: r0 = HIT_EPSILON-scale, half_angle = 0.0
        // must converge at (very close to) the same t a plain point-ray
        // march would, proving the cone generalization is a strict
        // superset of the original point-ray behavior, not a silent
        // change to it.
        let (_, objects, _) = ground_and_box();
        let object = &objects[1]; // the small box, translation (0, 0.8, 0), half-extent 0.8 -> near face at z=-0.8
        let ray_origin = Vec3::new(0.0, 0.8, -10.0);
        let ray_dir = Vec3::Z;
        // Independently-computed expected convergence t via a plain
        // point-ray march (mirrors cpu_ref::march_object's own loop
        // shape exactly, but inlined here so this test's own expected
        // value is derived from the SAME primitives march_object_cone
        // itself calls (local_distance), not a hand-computed geometric
        // guess about the box's own near-face position — the actual bug
        // this test caught during development was exactly a wrong
        // hand-computed guess, not a real algorithm defect).
        let inv_rotation = object.rotation.inverse();
        let mut expected_t = 0.0f32;
        for _ in 0..128 {
            let p_local = inv_rotation * (ray_origin + expected_t * ray_dir - object.translation);
            let d = local_distance(&object.shape, p_local);
            if d < 1e-4 {
                break;
            }
            expected_t += d;
        }

        let cone_result = march_object_cone(object, ray_origin, ray_dir, 0.0, 30.0, 1e-4, 0.0);
        assert!(cone_result.is_some(), "a zero-half-angle cone aimed dead-center must converge");
        let (t, coverage) = cone_result.unwrap();
        assert!(
            (t - expected_t).abs() < 0.01,
            "a zero-half-angle cone must converge at (very close to) the same t a plain point-ray march finds: cone_t={t} point_ray_t={expected_t}"
        );
        assert!(coverage > 0.99, "a dead-center zero-half-angle hit should report ~full coverage: got {coverage}");
    }

    #[test]
    fn full_coverage_on_dead_center_hit() {
        // A near-zero half_angle (essentially a point ray) converges
        // with radius(t) shrinking toward 0 at every step, so the final
        // step's own d lands very close to 0 regardless of MIN_CONE_STEP's
        // own coarse floor — a genuinely wide cone's LAST march step can
        // legitimately overshoot deep into its own (much larger) radius
        // even on a dead-center hit, which is real, expected marching
        // behavior (the same class of step-granularity noise DDGI's own
        // marching already accepts), not something this test should
        // assert away at a wide half_angle.
        let (_, objects, bvh) = lone_sphere();
        let ray_origin = Vec3::new(0.0, 0.0, 0.0);
        let ray_dir = Vec3::Z;
        let hit = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.001, 0.0, None);
        let hit = hit.expect("a cone aimed dead-center at the sphere must hit");
        assert!(hit.coverage > 0.95, "a dead-center hit at a near-zero half_angle should report near-full coverage: got {}", hit.coverage);
    }

    #[test]
    fn partial_coverage_on_grazing_near_miss() {
        let (_, objects, bvh) = lone_sphere();
        // Sphere at (0,0,10), radius 1.0. Offset the ray so it grazes
        // just past the sphere's own silhouette edge, close enough that
        // a WIDE cone's own growing radius still picks up partial
        // coverage from it near the sphere's own far reach.
        let ray_origin = Vec3::new(1.05, 0.0, 0.0);
        let ray_dir = Vec3::Z;
        let half_angle = 0.06; // wide enough for the cone's footprint to graze the sphere by max_t
        let hit = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.02, half_angle, None);
        let hit = hit.expect("a grazing cone should still register SOME convergence given a wide enough half_angle");
        assert!(hit.coverage > 0.0 && hit.coverage < 1.0, "a grazing near-miss must report PARTIAL coverage, not 0 or 1: got {}", hit.coverage);
    }

    #[test]
    fn no_stall_near_convergence() {
        // Construct a case where d and radius(t) start very close
        // together (a wide cone with a large r0, so radius(t) is
        // already close to typical SDF distances from the very first
        // step) — confirm the march still terminates within
        // MAX_CONE_MARCH_STEPS rather than stalling.
        let (_, objects, _) = ground_and_box();
        let object = &objects[1];
        let ray_origin = Vec3::new(0.0, 0.8, -10.0);
        let ray_dir = Vec3::Z;
        let result = march_object_cone(object, ray_origin, ray_dir, 0.0, 30.0, 0.5, 0.4);
        assert!(result.is_some(), "a wide cone must still converge, not stall out past MAX_CONE_MARCH_STEPS");
    }

    #[test]
    fn trace_cone_excludes_the_origin_entity() {
        let (ids, objects, bvh) = ground_and_box();
        let box_entity = ids[1];
        let ray_origin = Vec3::new(0.0, 0.8, -10.0);
        let ray_dir = Vec3::Z;
        // Without exclusion, the box itself is hit at t~=10.
        let with_no_exclusion = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.05, 0.05, None);
        assert!(with_no_exclusion.is_some());
        // With the box excluded as origin_entity, the ray should pass
        // through empty space beyond it (ground plate is far below,
        // out of this ray's own path) and report a miss.
        let with_exclusion = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.05, 0.05, Some(box_entity));
        assert!(with_exclusion.is_none(), "excluding the only object in the ray's path must produce a clean miss, not a self-hit");
    }

    #[test]
    fn cone_slab_hit_tightens_the_margin_for_a_near_node() {
        // A node whose tight AABB the ray passes just outside of at
        // close range — worst-case padding (evaluated at t_query_max,
        // far away) would let the ray graze in, but the tightened
        // (t_near-evaluated) margin should be small enough to correctly
        // exclude it, since the true cone footprint at THIS node's own
        // near distance is much smaller than at the far worst-case t.
        let aabb = Aabb { min: Vec3::new(-1.0, -1.0, 4.0), max: Vec3::new(1.0, 1.0, 5.0) };
        let ray_dir = Vec3::Z;
        let r0 = 0.0;
        let half_angle = 0.2; // tan(0.2) ~= 0.2027
        let t_query_max = 30.0;

        // Worst-case-only margin (the OLD behavior): padding evaluated
        // at the far query bound would be cone_radius_at(0, 0.2, 30.0)
        // ~= 6.08 — comfortably wide enough to pull this near node in.
        let worst_case_margin = cone_radius_at(r0, half_angle, t_query_max);
        assert!(worst_case_margin > 0.3, "sanity: the worst-case margin must be wide enough to hit this node at all");

        // The tightened per-node version must NOT let this obviously-
        // near-distance-only margin in: at t ~= 4..5 (this node's own
        // near/far range), cone_radius_at(0, 0.2, ~4.5) ~= 0.91, which
        // DOES still cover the 0.3 gap — so pick a ray offset the tight
        // margin genuinely excludes but the worst-case one doesn't, to
        // prove real tightening happened, not just "both hit."
        let far_offset_ray_origin = Vec3::new(2.5, 0.0, 0.0); // 1.5 outside the AABB's own X extent
        let (t_near_tight, t_far_tight) =
            cone_slab_hit(far_offset_ray_origin, ray_dir, &aabb, t_query_max, r0, half_angle, t_query_max);
        assert!(
            t_near_tight > t_far_tight,
            "a per-node tightened margin (~0.91 at this node's own near t) must exclude a ray 1.5 units off, even though the worst-case margin (~6.08) would not"
        );

        // The plain (untightened) worst-case-only slab test on the SAME
        // ray must still find it, confirming the exclusion above is a
        // real tightening effect, not a bug that also breaks the
        // worst-case path.
        let worst_case_padded =
            Aabb { min: aabb.min - Vec3::splat(worst_case_margin), max: aabb.max + Vec3::splat(worst_case_margin) };
        let (t_near_worst, t_far_worst) = slab_hit(far_offset_ray_origin, ray_dir, &worst_case_padded, t_query_max);
        assert!(t_near_worst <= t_far_worst, "the untightened worst-case margin must still include this ray (sanity check)");
    }

    #[test]
    fn cone_slab_hit_never_under_pads_a_genuine_far_hit() {
        // A node reachable only near t_query_max, where the tightened
        // per-node margin must converge back to (approximately) the
        // worst-case one — proving the two-pass refinement doesn't
        // erroneously shrink the margin for nodes that ARE at long
        // range, only for nodes that are near.
        let aabb = Aabb { min: Vec3::new(-1.0, -1.0, 29.0), max: Vec3::new(1.0, 1.0, 30.0) };
        let r0 = 0.0;
        let half_angle = 0.2;
        let t_query_max = 30.0;
        let margin_at_far_t = cone_radius_at(r0, half_angle, 29.5);
        let ray_origin = Vec3::new(1.0 + margin_at_far_t * 0.5, 0.0, 0.0); // inside the far-t margin, outside the tight AABB
        let ray_dir = Vec3::Z;
        let (t_near, t_far) = cone_slab_hit(ray_origin, ray_dir, &aabb, t_query_max, r0, half_angle, t_query_max);
        assert!(t_near <= t_far, "a node only reachable near t_query_max must still be found by the tightened margin");
    }

    #[test]
    fn cone_slab_hit_matches_trace_cone_hit_result_for_a_real_scene() {
        // End-to-end proof that the per-node tightening in trace_cone
        // never turns a real hit into a miss: the box in ground_and_box
        // is squarely in the ray's path regardless of margin tightness,
        // so a correct implementation must find it identically to the
        // old fixed-worst-case-margin behavior would have.
        let (_, objects, bvh) = ground_and_box();
        let ray_origin = Vec3::new(0.0, 0.8, -10.0);
        let ray_dir = Vec3::Z;
        let hit = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.05, 0.05, None);
        let hit = hit.expect("a cone aimed dead-center at the box must hit it regardless of margin-tightening changes");
        // Converges slightly before the box's own true near face (z=9.2)
        // since the cone's own growing radius satisfies `d < radius(t)`
        // a bit early — this value is this test's own regression anchor
        // (pinned to the current margin-tightening behavior), not a
        // hand-derived exact geometric distance.
        assert!((hit.t - 8.767).abs() < 0.01, "hit distance should match the pre-tightening reference value (~8.767): got {}", hit.t);
    }

    #[test]
    fn cone_trace_ray_tail_term_adds_real_light_on_a_genuinely_lit_hit() {
        // The geometric-series tail approximates un-traced bounces
        // beyond max_bounces using THIS hit's own real direct_and_
        // emissive and diffuse albedo — a front-lit, non-black,
        // non-white (albedo < 0.95 on every channel, so the tail ratio
        // stays finite) hit must therefore report strictly MORE energy
        // than the same hit reported before this tail term existed.
        let (_, objects, bvh) = lone_sphere();
        let ray_origin = Vec3::new(0.0, 0.0, 0.0);
        let ray_dir = Vec3::Z;
        let front_light = Light { direction_or_position: Vec3::new(0.0, 0.0, 1.0), ..overhead_sun() };
        let lights = vec![front_light];
        let with_tail = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, 30.0, 0.02, 0.02, None, 1);
        // Recompute what the OLD (no-tail) single-bounce result would
        // have been, directly from shade()'s own direct_and_emissive at
        // the same hit, scaled by the same coverage^2 this loop applies
        // — i.e. exactly this function's old return expression before
        // the tail term was added, not a magic re-derivation.
        let hit = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.02, 0.02, None).expect("ray must hit the sphere");
        let object = objects.iter().find(|o| o.entity == hit.entity).expect("hit object must exist");
        let p_world = ray_origin + hit.t * ray_dir;
        let result = shade(&object.material, p_world, hit.world_normal, -ray_dir, &lights, &bvh, &objects, Some(hit.entity), hit.t, None, None);
        let without_tail = result.direct_and_emissive * hit.coverage * hit.coverage;
        assert!(
            without_tail.length() > 0.0,
            "fixture sanity check: the hit must be genuinely lit before comparing tail contribution: {without_tail:?}"
        );
        assert!(
            with_tail.length() > without_tail.length(),
            "the tail term must add real energy on top of the un-tailed single-bounce result: with_tail={with_tail:?} without_tail={without_tail:?}"
        );
    }

    #[test]
    fn cone_trace_ray_tail_term_stays_exactly_zero_when_the_hit_itself_is_unlit() {
        // The tail is derived multiplicatively from THIS hit's own
        // direct_and_emissive — a hit with no light reaching it at all
        // (no lights in the scene) must produce an exactly-zero tail,
        // not a positive constant. This is the property that keeps the
        // tail from being able to leak light into a sealed dark room:
        // see room_leak_regression's own tests, which exercise this at
        // full scene scale.
        let (_, objects, bvh) = lone_sphere();
        let ray_origin = Vec3::new(0.0, 0.0, 0.0);
        let ray_dir = Vec3::Z;
        let lights: Vec<Light> = vec![];
        let result = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, 30.0, 0.02, 0.02, None, 1);
        assert_eq!(result, Vec3::ZERO, "an unlit hit's tail term must stay exactly zero: got {result:?}");
    }

    #[test]
    fn cone_trace_ray_returns_black_on_miss() {
        let (_, objects, bvh) = lone_sphere();
        let ray_origin = Vec3::new(0.0, 0.0, 0.0);
        let ray_dir = Vec3::new(1.0, 0.0, 0.0); // aimed away from the sphere entirely
        let lights = vec![overhead_sun()];
        let result = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, 30.0, 0.02, 0.02, None, 1);
        assert_eq!(result, Vec3::ZERO, "a cone aimed at empty space must return black, not sky_color — see cone_trace_ray's own doc comment for why trusting a miss as real sky isn't safe");
    }

    /// Regression test for a real light-leak bug found by direct visual
    /// inspection in `examples/gi_room.rs`: a fully sealed, opaque room
    /// (no gaps, roof fully closed) read as lit up bright instead of
    /// dark once cone tracing was enabled. Root cause was
    /// `cone_trace_ray`'s own OLD formula blending the hit's own shaded
    /// color toward `sky_color` by `1.0 - coverage` — see that
    /// function's own doc comment for the full explanation of why
    /// `coverage < 1.0` does NOT mean "the cone partially escaped to
    /// open sky." Reproduced here directly: an unlit box, thick enough
    /// (half-extent 5.0) that a cone aimed dead-center at one interior
    /// wall from just inside it is GEOMETRICALLY GUARANTEED to hit that
    /// wall (never escape past it, regardless of any march-step
    /// convergence noise) — so any `sky_color` contribution in the
    /// result at all is proof of a leak, not a legitimate partial
    /// escape.
    #[test]
    fn cone_trace_ray_never_leaks_sky_color_into_a_sealed_box() {
        let ids = entities(1);
        let objects = vec![TraceObject {
            entity: ids[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(5.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            // Fully unlit material (no direct light in this fixture
            // either) — the box's own interior wall must shade to
            // exactly black. Any non-black component in the result can
            // only have come from sky_color leaking in.
            material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.5),
        }];
        let hybrid_objects: Vec<HybridObject> =
            vec![HybridObject { entity: ids[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(5.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        let lights: Vec<Light> = vec![]; // no lights at all: a true hit must shade to pure black
        // Just inside the box's own interior (half-extent 5.0), aimed
        // dead-center at the +Z interior wall 3.0 units away — geometry
        // that structurally cannot let a cone escape to open space no
        // matter how coarse the march's own convergence is (max_t=3.0
        // stops the ray well before it could ever reach the box's own
        // far exterior even if the near wall were somehow skipped).
        let ray_origin = Vec3::new(0.0, 0.0, 4.0);
        let ray_dir = Vec3::Z;
        let result = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, 3.0, 0.05, 0.3, None, 1);
        assert_eq!(
            result,
            Vec3::ZERO,
            "a cone dead-center inside a sealed, unlit box must shade to pure black — any non-zero result is sky_color leaking through a wall that was structurally guaranteed to be hit: got {result:?}"
        );
    }

    /// Real bug this test would have caught: WGSL's native `smoothstep`
    /// intrinsic is spec-indeterminate when `edge0 == edge1`, which
    /// `march_object_cone`'s own `radius == 0.0` case hits on every
    /// degenerate point-ray bounce (bounce 2+, see `cone_trace_ray`'s
    /// own doc comment) — on real hardware this produced NaN coverage,
    /// visible as the exact "bounce 2+ ignores the sealed roof" report
    /// this fixture is named after. `conetrace_ref.rs`'s own hand-rolled
    /// `smoothstep` never showed this (dividing by 0.0 and relying on
    /// `f32::clamp` to resolve the resulting infinity is accidentally
    /// safe), so this asserts the explicit `radius > 0.0` guard directly
    /// rather than relying on that accident.
    #[test]
    fn march_object_cone_with_a_degenerate_point_ray_never_produces_nan_coverage() {
        let (_, objects, _) = lone_sphere();
        // Ray aimed dead-center at the sphere (translation (0,0,10),
        // radius 1.0), starting well inside it so d < 0.0 < radius is
        // guaranteed on the very first march step — r0=0.0/half_angle=0.0
        // is exactly cone_trace_ray's own bounce-2+ continuation shape.
        let ray_origin = Vec3::new(0.0, 0.0, 9.5);
        let ray_dir = Vec3::Z;
        let result = march_object_cone(&objects[0], ray_origin, ray_dir, 0.0, 5.0, 0.0, 0.0);
        let Some((t, coverage)) = result else {
            panic!("a ray starting inside the sphere must hit it immediately");
        };
        assert!(t.is_finite(), "hit t must be finite, got {t}");
        assert!(coverage.is_finite(), "coverage must be finite (not NaN/inf) for a zero-radius cone, got {coverage}");
        assert_eq!(coverage, 1.0, "a zero-radius cone's coverage at any hit (d < radius == 0.0, i.e. already inside the surface) must be full 1.0, got {coverage}");
    }

    #[test]
    fn cone_trace_ray_picks_up_real_lit_color_on_hit() {
        let (_, objects, bvh) = lone_sphere();
        // Ray travels +Z and hits the sphere's own -Z-facing near face —
        // aim the light straight back along -Z (not straight down, as
        // an earlier version of this fixture had it) so the hit point's
        // own surface normal (~(0,0,-1)) actually faces the light
        // (n_dot_l > 0), genuinely front-lit rather than grazing/in
        // shadow from an overhead sun this particular hit point can't
        // see.
        let ray_origin = Vec3::new(0.0, 0.0, 0.0);
        let ray_dir = Vec3::Z;
        let front_light =
            Light { direction_or_position: Vec3::new(0.0, 0.0, 1.0), ..overhead_sun() };
        let lights = vec![front_light];
        let result = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, 30.0, 0.02, 0.02, None, 1);
        assert_ne!(result, Vec3::ZERO, "a genuinely front-lit sphere hit must not report pure black (the same value a miss now falls back to)");
    }

    /// Two-box fixture for the multi-bounce tests below: a small unlit
    /// floor tile (never directly lit by any light in the fixture, only
    /// reachable via a second bounce) sitting directly below a
    /// second-bounce-reachable, brightly emissive "glow box" placed
    /// exactly along the floor hit's own +Y normal — the fixed
    /// second-bounce direction `cone_trace_ray`'s own doc comment
    /// specifies. `ray_origin`/`ray_dir` (straight down, `-Y`) is
    /// chosen so bounce 1 hits the floor tile dead-center, and bounce
    /// 2 (continuing along the floor's own `+Y` normal) flies straight
    /// up into the glow box.
    fn floor_with_glow_box_above() -> (Vec<Entity>, Vec<TraceObject>, Bvh, Vec3, Vec3) {
        let e = entities(2);
        let floor_entity = e[0];
        let glow_entity = e[1];
        let objects = vec![
            TraceObject {
                entity: floor_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::new(5.0, 0.1, 5.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.1, 0.0),
                rotation: Quat::IDENTITY,
                // Fully unlit material and no direct light reaches it in
                // this fixture (no Light in the scene at all) — any
                // radiance measured at a point shaded above this floor
                // can ONLY have arrived via a second bounce off the
                // glow box above.
                material: Material::new(Vec3::splat(0.9), 0.0, 0.6),
            },
            TraceObject {
                entity: glow_entity,
                shape: Shape::RoundedBox { half_extents: Vec3::splat(2.0), corner_radius: 0.0 },
                // Above the floor, not below it — a ray hitting the
                // floor's own TOP surface (approaching from above, as
                // this fixture's own ray_origin/ray_dir below do)
                // converges with an OUTWARD normal pointing +Y (up,
                // away from the solid floor beneath it), so the second
                // bounce (which continues along that exact fixed
                // normal direction per cone_trace_ray's own documented
                // design) travels further UP, not down — the glow box
                // must sit in that direction to be reachable at all.
                // An earlier version of this fixture placed the glow
                // box below the floor, which the second bounce could
                // structurally never reach (its own normal points the
                // opposite way); that version's own tests only passed
                // because a miss travelling up into open air used to
                // fall back to a non-zero sky-color gradient, which
                // this fixture's weak length()-based assertions
                // couldn't distinguish from a real hit. cone_trace_ray
                // no longer falls back to sky color on a miss (see its
                // own doc comment), which exposed this fixture bug.
                translation: Vec3::new(0.0, 5.0, 0.0),
                rotation: Quat::IDENTITY,
                // Bright emissive material — the ONLY source of light in
                // this fixture (no Light entries at all), so any bounced
                // contribution is unambiguously attributable to this
                // box's own emissive term via the second bounce's own
                // direct-lit-only shade call.
                material: Material::new(Vec3::ZERO, 0.0, 0.5).with_emissive(Vec3::new(40.0, 40.0, 40.0)),
            },
        ];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject {
                entity: floor_entity,
                world_aabb: Aabb::from_center_half(objects[0].translation, Vec3::new(5.0, 0.1, 5.0)),
            },
            HybridObject { entity: glow_entity, world_aabb: Aabb::from_center_half(objects[1].translation, Vec3::splat(2.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);
        // Shaded point 1.5 units above the floor, aimed straight down —
        // bounce 1 hits the floor tile dead-center at t~=1.5, on its own
        // top surface (outward normal +Y).
        let ray_origin = Vec3::new(0.0, 1.5, 0.0);
        let ray_dir = Vec3::NEG_Y;
        (e, objects, bvh, ray_origin, ray_dir)
    }

    #[test]
    fn second_bounce_adds_bounced_light_from_a_lit_neighbor() {
        let (_, objects, bvh, ray_origin, ray_dir) = floor_with_glow_box_above();
        let lights: Vec<Light> = vec![]; // no direct lights at all — every non-zero result is bounced/emissive light
        let r0 = 0.02_f32;
        let half_angle = 0.02_f32;
        let max_t = 30.0_f32;

        let one_bounce = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, max_t, r0, half_angle, None, 1);
        let two_bounce = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, max_t, r0, half_angle, None, 2);

        assert_eq!(
            one_bounce,
            Vec3::ZERO,
            "with no direct lights and an unlit floor, bounce 1 alone must shade to pure black: got {one_bounce:?}"
        );
        assert!(
            two_bounce.length() > one_bounce.length(),
            "a second bounce that reaches the glow box's own emissive surface must add measurable light over the 1-bounce result: 1-bounce={one_bounce:?} 2-bounce={two_bounce:?}"
        );
    }

    #[test]
    fn higher_max_bounces_never_darkens_a_scene_with_only_positive_radiance() {
        // Every material/light in this renderer's own fixtures is
        // non-negative radiance, so adding more bounce budget should
        // only ever add non-negative contributions on top of fewer
        // bounces' own result — never REDUCE the total. Guards against
        // a sign/accumulation-order bug in the bounce loop's own
        // running `total` (e.g. overwriting instead of accumulating).
        let (_, objects, bvh, ray_origin, ray_dir) = floor_with_glow_box_above();
        let lights: Vec<Light> = vec![];
        let r0 = 0.02_f32;
        let half_angle = 0.02_f32;
        let max_t = 30.0_f32;

        let b1 = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, max_t, r0, half_angle, None, 1).length();
        let b2 = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, max_t, r0, half_angle, None, 2).length();
        let b3 = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, max_t, r0, half_angle, None, 3).length();

        assert!(b2 >= b1 - 1e-6, "2-bounce total must not be darker than 1-bounce: b1={b1} b2={b2}");
        assert!(b3 >= b2 - 1e-6, "3-bounce total must not be darker than 2-bounce: b2={b2} b3={b3}");
    }

    #[test]
    fn second_bounce_continues_exactly_along_the_hit_normal_not_reflected_or_negated() {
        // A ground-plane hit's own normal points straight up (+Y) — the
        // second bounce must sample EXACTLY that direction (per
        // cone_trace_ray's own documented fixed-direction design), not
        // -normal (which would immediately re-hit the floor from
        // underneath, converging at t~=0) or a reflected view vector
        // (which would send the second bounce off at a shallow angle
        // instead of straight up into the glow box). Distinguishes this
        // by placing the glow box ONLY reachable via the exact +Y
        // normal direction — a wrong direction would miss it entirely
        // and this test would see the same black result as
        // second_bounce_adds_bounced_light_from_a_lit_neighbor's own
        // 1-bounce case.
        let (_, objects, bvh, ray_origin, ray_dir) = floor_with_glow_box_above();
        let lights: Vec<Light> = vec![];
        let r0 = 0.02_f32;
        let half_angle = 0.02_f32;
        let max_t = 30.0_f32;

        let two_bounce = cone_trace_ray(&bvh, &objects, &lights, ray_origin, ray_dir, max_t, r0, half_angle, None, 2);
        assert!(
            two_bounce.length() > 1e-3,
            "the second bounce must travel along +Y (the floor hit's own normal) to reach the glow box directly above it: got {two_bounce:?}"
        );
    }

    #[test]
    fn wider_half_angle_smooths_more_than_narrower() {
        // The actual behavioral proof of cone tracing's whole value
        // proposition: two cones (same origin/direction/r0) fired at
        // the same small isolated object, differing only in half_angle,
        // must show a measurably different result. Geometry is chosen
        // so a NARROW cone's own radius(t) never grows large enough to
        // reach the sphere at all (a genuine, structural miss — the
        // sphere sits entirely outside even the narrow cone's own
        // maximum footprint), while a WIDE cone's own faster-growing
        // radius(t) does reach it — the clearest possible signal that
        // widening the cone changes what geometry contributes at all,
        // not just a noisy coverage-value wobble (coverage's own exact
        // value at convergence is sensitive to MIN_CONE_STEP's step
        // granularity — see full_coverage_on_dead_center_hit's own
        // comment — so hit/no-hit is the more robust signal to assert
        // on here, with the wide cone's own real coverage value checked
        // only as a secondary, weaker assertion).
        let (_, objects, bvh) = lone_sphere();
        let ray_origin = Vec3::new(1.5, 0.0, 0.0); // perpendicular offset 1.5 > sphere radius 1.0: a point ray misses entirely
        let ray_dir = Vec3::Z;
        let narrow = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.02, 0.01, None);
        let wide = trace_cone(&bvh, &objects, ray_origin, ray_dir, 30.0, 0.02, 0.1, None);
        assert!(narrow.is_none(), "a narrow enough cone must structurally miss geometry outside its own max footprint: got {narrow:?}");
        let wide = wide.expect("a wide enough cone must reach the same geometry the narrow cone structurally cannot");
        assert!(wide.coverage > 0.0, "the wide cone's own hit must report non-zero coverage: got {}", wide.coverage);
    }

    #[test]
    fn cone_trace_indirect_is_deterministic() {
        let (_, objects, bvh) = ground_and_box();
        let lights = vec![overhead_sun()];
        let world_pos = Vec3::new(0.0, 1.6, 0.0);
        let normal = Vec3::Y;
        let a = cone_trace_indirect(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        let b = cone_trace_indirect(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        assert_eq!(a, b, "identical inputs must produce identical outputs — no hidden randomness/state");
    }

    #[test]
    fn cone_trace_indirect_picks_up_real_bounced_light_from_a_lit_box_below() {
        let (_, objects, bvh) = ground_and_box();
        let lights = vec![overhead_sun()];
        let world_pos = Vec3::new(0.0, 1.6, 0.0);
        let normal = Vec3::Y;
        let result = cone_trace_indirect(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        assert!(
            result.length() > 0.0,
            "the full 5-cone hemisphere, centered on a normal that points at a real sunlit surface, must not be exactly black: got {result:?}"
        );
    }

    #[test]
    fn cone_trace_indirect_returns_black_when_nothing_nearby() {
        let (_, objects, bvh) = lone_sphere();
        let lights = vec![overhead_sun()];
        // Far from the sphere, aimed away from it entirely along every
        // hemisphere sample direction.
        let world_pos = Vec3::new(-50.0, 0.0, -50.0);
        let normal = Vec3::new(0.0, 0.0, -1.0); // hemisphere points away from the sphere at (0,0,10)
        let result = cone_trace_indirect(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        // Every sample misses (no sky_color fallback anymore — see
        // cone_trace_ray's own doc comment), so the average must be
        // exactly black.
        assert_eq!(result, Vec3::ZERO);
    }

    #[test]
    fn cone_trace_indirect_single_picks_up_real_bounced_light_straight_off_the_normal() {
        let (_, objects, bvh) = ground_and_box();
        let lights = vec![overhead_sun()];
        let world_pos = Vec3::new(0.0, 1.6, 0.0);
        let normal = Vec3::Y;
        let result = cone_trace_indirect_single(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        assert!(result.length() > 0.0, "a single cone straight up into a lit box's own bounced light must not be exactly black: got {result:?}");
    }

    #[test]
    fn cone_trace_indirect_single_returns_black_when_nothing_nearby() {
        let (_, objects, bvh) = lone_sphere();
        let lights = vec![overhead_sun()];
        let world_pos = Vec3::new(-50.0, 0.0, -50.0);
        let normal = Vec3::new(0.0, 0.0, -1.0);
        let result = cone_trace_indirect_single(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        assert_eq!(result, Vec3::ZERO, "a single cone aimed away from all geometry must return exactly black, same as the 5-cone version");
    }

    #[test]
    fn cone_trace_indirect_single_is_deterministic() {
        let (_, objects, bvh) = ground_and_box();
        let lights = vec![overhead_sun()];
        let world_pos = Vec3::new(0.0, 1.6, 0.0);
        let normal = Vec3::Y;
        let a = cone_trace_indirect_single(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        let b = cone_trace_indirect_single(&bvh, &objects, &lights, world_pos, normal, 30.0, 0.05, 0.15, None, 1);
        assert_eq!(a, b, "identical inputs must produce identical outputs — no hidden randomness/state");
    }
}

#[cfg(test)]
mod room_leak_regression {
    use super::*;
    use bevy::math::Quat;
    use bevy::prelude::World;
    use crate::hybrid::bvh::Bvh;
    use crate::hybrid::cpu_ref::LightKind;
    use crate::hybrid::material::Material;
    use crate::hybrid::scene::HybridObject;
    use crate::sdf::components::Shape;

    const ROOM_HALF_X: f32 = 8.0;
    const ROOM_HALF_Y: f32 = 3.0;
    const ROOM_HALF_Z: f32 = 6.5;
    const WALL_THICKNESS: f32 = 0.3;
    const WALL_OVERLAP: f32 = 0.2;

    /// Rebuilds `examples/gi_room.rs`'s own room shell (all 6 panels,
    /// exact same half-extents/`WALL_OVERLAP` seam treatment) plus all 7
    /// cubes, matching that file's own real geometry closely enough to
    /// reproduce a real bug found there — see
    /// `real_room_with_sun_and_no_lamp_never_leaks_light_through_a_
    /// closed_roof`'s own doc comment.
    fn full_room_with_cubes() -> (Vec<TraceObject>, Bvh) {
        let mut world = World::new();
        // Room shell — exactly gi_room.rs::spawn_room's own white_wall
        // (0.92, 0.0, 0.35, reflectance 0.7), corner_radius 0.0.
        let mat = Material::new(Vec3::splat(0.92), 0.0, 0.35).with_reflectance(0.7);
        let mut objs = Vec::new();
        // Single closure taking corner_radius explicitly — a real
        // mismatch found and fixed here: an earlier version of this
        // fixture hardcoded corner_radius 0.0 for every object,
        // including cubes, when gi_room.rs::spawn_cubes actually uses
        // 0.02 for cubes (room-shell panels genuinely are 0.0).
        let mut push = |translation: Vec3, half_extents: Vec3, corner_radius: f32, material: Material| {
            let id = world.spawn_empty().id();
            objs.push(TraceObject { entity: id, shape: Shape::RoundedBox { half_extents, corner_radius }, translation, rotation: Quat::IDENTITY, material });
        };
        // Room shell.
        let floor_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, WALL_THICKNESS, ROOM_HALF_Z + WALL_OVERLAP);
        push(Vec3::new(0.0, -ROOM_HALF_Y - WALL_THICKNESS, 0.0), floor_half, 0.0, mat);
        let side_wall_half = Vec3::new(WALL_THICKNESS, ROOM_HALF_Y + WALL_OVERLAP, ROOM_HALF_Z);
        push(Vec3::new(ROOM_HALF_X + WALL_THICKNESS, 0.0, 0.0), side_wall_half, 0.0, mat);
        push(Vec3::new(-ROOM_HALF_X - WALL_THICKNESS, 0.0, 0.0), side_wall_half, 0.0, mat);
        let end_wall_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, ROOM_HALF_Y + WALL_OVERLAP, WALL_THICKNESS);
        push(Vec3::new(0.0, 0.0, ROOM_HALF_Z + WALL_THICKNESS), end_wall_half, 0.0, mat);
        push(Vec3::new(0.0, 0.0, -ROOM_HALF_Z - WALL_THICKNESS), end_wall_half, 0.0, mat);
        let roof_half = Vec3::new(ROOM_HALF_X + WALL_OVERLAP, WALL_THICKNESS, ROOM_HALF_Z + WALL_OVERLAP);
        push(Vec3::new(0.0, ROOM_HALF_Y + WALL_THICKNESS, 0.0), roof_half, 0.0, mat);

        // Cubes — same positions/sizes/materials as gi_room.rs::spawn_cubes.
        let floor_y = -ROOM_HALF_Y;
        let red = Material::new(Vec3::new(0.75, 0.2, 0.15), 0.0, 0.6);
        let blue = Material::new(Vec3::new(0.2, 0.35, 0.8), 0.1, 0.2).with_reflectance(0.7);
        push(Vec3::new(-6.8, floor_y + 0.9, -4.5), Vec3::splat(0.9), 0.02, red);
        push(Vec3::new(-5.2, floor_y + 0.5, -4.7), Vec3::splat(0.5), 0.02, blue);
        let gold = Material::new(Vec3::new(0.85, 0.65, 0.2), 0.9, 0.25).with_reflectance(0.9);
        let dark_base = Material::new(Vec3::splat(0.08), 0.0, 0.7);
        push(Vec3::new(-6.5, floor_y + 0.6, -1.5), Vec3::splat(0.6), 0.02, dark_base);
        push(Vec3::new(-6.5, floor_y + 1.5, -1.5), Vec3::splat(0.3), 0.02, gold);
        let white_cube = Material::new(Vec3::splat(0.85), 0.0, 0.4).with_reflectance(0.6);
        push(Vec3::new(-6.5, floor_y + 0.5, 2.0), Vec3::splat(0.5), 0.02, white_cube);
        let purple = Material::new(Vec3::new(0.5, 0.25, 0.6), 0.2, 0.45);
        push(Vec3::new(-4.0, floor_y + 1.0, 2.2), Vec3::splat(1.0), 0.02, purple);
        let green = Material::new(Vec3::new(0.25, 0.7, 0.3), 0.0, 0.5);
        push(Vec3::new(-6.5, floor_y + 0.7, 4.8), Vec3::splat(0.7), 0.02, green);

        let hybrid_objects: Vec<HybridObject> =
            objs.iter().map(|o| HybridObject { entity: o.entity, world_aabb: Aabb::from_center_half(o.translation, half_extents_of(o)) }).collect();
        let bvh = Bvh::build(&hybrid_objects);
        (objs, bvh)
    }

    fn half_extents_of(o: &TraceObject) -> Vec3 {
        match o.shape {
            Shape::RoundedBox { half_extents, .. } => half_extents,
            _ => unreachable!(),
        }
    }

    /// Regression test for a real light-leak bug found by direct visual
    /// inspection in `examples/gi_room.rs`: with the roof fully closed
    /// and the lamp off, the room's own ceiling/walls/floor still read
    /// as faintly, visibly lit ("splats") under cone tracing, even after
    /// `cone_trace_ray`'s own coverage-sky-blend bug (a separate,
    /// earlier fix) was corrected.
    ///
    /// Root cause: a cone's own convergence `t` is only accurate to
    /// within `radius(t)` (see `march_object_cone`'s own doc comment) —
    /// a LOW-coverage hit can converge while `p_world = origin + t *
    /// direction` is still a full `radius(t)` away from the true
    /// surface, floating in open room air rather than sitting on solid
    /// ground. Confirmed directly during this bug's own investigation:
    /// one real leaking sample had `p_world` a full 1.03 world units
    /// above the floor's own true y-plane, with `d ≈ radius(t) ≈ 1.03`
    /// at "convergence." Shading from that floating point (with its
    /// own, still locally-correct SDF-gradient normal) can give a
    /// shadow ray toward the sun a genuinely clear, unobstructed path
    /// that a shadow ray cast from the TRUE surface point never would
    /// have — letting real sunlight through a provably sealed room.
    ///
    /// Fixed in `cone_trace_ray` by scaling the final shaded
    /// contribution by `coverage^2` (see that function's own doc
    /// comment for why squared, not linear, and why this fades toward
    /// darkness — physically conservative — rather than toward a second
    /// wrong value).
    /// Shared by both the single-bounce and multi-bounce variants below
    /// — same fixture, same threshold reasoning, only `max_bounces`
    /// differs — so the two tests can never silently diverge in setup.
    fn measure_max_ceiling_leak(max_bounces: u32) -> f32 {
        let (objects, bvh) = full_room_with_cubes();
        let sun = Light {
            kind: LightKind::Directional,
            color: Vec3::new(1.0, 0.97, 0.9),
            // Same ~35-degree elevation gi_room.rs's own sun uses.
            direction_or_position: (Vec3::new(-7.63, 4.26, -2.97) - Vec3::new(0.0, 10.0, 0.0)).normalize(),
            intensity: 20000.0,
            spot_direction: Vec3::ZERO,
            range: 0.0,
            inner_angle: 0.0,
            outer_angle: 0.0,
            shadow_softness_k: 2.0,
        };
        let lights = vec![sun]; // no lamp: isolates this from the (separate, ruled-out) lamp-bounce question
        let r0 = 0.05_f32;
        let half_angle = 0.15_f32;
        let max_t = 28.0_f32;

        let mut max_leak = 0.0f32;
        for xi in -7..=7 {
            for zi in -6..=6 {
                let x = xi as f32;
                let z = zi as f32 * (ROOM_HALF_Z / 6.0);
                let world_pos = Vec3::new(x, ROOM_HALF_Y - 0.1, z);
                let result = cone_trace_indirect(&bvh, &objects, &lights, world_pos, Vec3::NEG_Y, max_t, r0, half_angle, None, max_bounces);
                max_leak = max_leak.max(result.length());
            }
        }
        max_leak
    }

    #[test]
    fn real_room_with_sun_and_no_lamp_never_leaks_light_through_a_closed_roof() {
        let max_leak = measure_max_ceiling_leak(1);

        // Threshold is NOT exact zero — the coverage^2 fix is a real,
        // measured, asymptotic reduction (0.045 -> 0.0014 -> 0.00042
        // across two rounds of tightening during this bug's own
        // investigation), not an exact guarantee for every possible
        // low-coverage hit. 1e-3 is calibrated against this scene's own
        // real light scale: EXPOSURE=0.0005 * sun intensity=20000 gives
        // a genuinely front-lit white surface a direct_and_emissive
        // magnitude around ~2-3 in these same units — 1e-3 is roughly
        // 0.03-0.05% of that, well below any plausible post-tone-
        // mapping display-visible threshold, and comfortably above the
        // ~4.2e-4 residual this fixture's own real geometry produces (a
        // real margin, not a threshold picked to just barely pass).
        assert!(
            max_leak < 1e-3,
            "with the roof fully closed and no lamp, every ceiling point's own cone_trace_indirect must be imperceptibly close to zero: got max_leak={max_leak}"
        );
    }

    #[test]
    fn real_room_with_sun_and_no_lamp_never_leaks_light_through_a_closed_roof_at_three_bounces() {
        let max_leak = measure_max_ceiling_leak(3);
        // Multi-bounce necessarily accumulates more terms (up to 3 per
        // hemisphere sample instead of 1), so a proportionally looser
        // threshold than the single-bounce test is warranted — still
        // far below any plausible display-visible magnitude relative to
        // this scene's own ~2-3 unit lit-surface scale (see the
        // single-bounce test's own doc comment for that derivation).
        assert!(
            max_leak < 3e-3,
            "with the roof fully closed and no lamp, every ceiling point's own 3-bounce cone_trace_indirect must stay imperceptibly close to zero: got max_leak={max_leak}"
        );
    }
}
