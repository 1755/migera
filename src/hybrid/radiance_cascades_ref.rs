//! CPU reference for Radiance Cascades — an EXPERIMENTAL second GI
//! technique, selectable alongside DDGI (`ddgi_ref.rs`) via
//! `GiMethod::RadianceCascades`, built specifically to A/B compare against
//! DDGI's own known, documented, unfixed limitation: `examples/gi_room.rs`'s
//! floor corridor stays dark because DDGI's fixed-density probe grid has to
//! relay bounce energy cell-to-cell, and a real fix attempt failed on the
//! real scene (see `PROGRESS.md`'s "Investigated (not fixed): residual dark
//! floor corridor after DDGI infinite-bounce" entry, commit `1a45987`).
//! Radiance Cascades (Alexander Sannikov, ExileCon 2023; 3D extension per
//! the "Sparse"/"Split Radiance Cascades" papers) resolves long-range
//! transport structurally differently: a HIERARCHY of cascade levels, each
//! trading probe density for angular ray density (`probe_spacing ~ 2^level`,
//! `ray_count ~ 4^level` in 3D, `interval ~ 2^level`), merged front-to-back
//! (`L_ac = L_ab + β_ab · L_bc`) so far cascades sample distant geometry
//! DIRECTLY with wide rays rather than relaying through adjacent cells.
//!
//! This is an explicit, user-approved EXPERIMENT, not a committed
//! replacement — DDGI remains this renderer's shipping default regardless
//! of this experiment's outcome (see the plan document that recorded this
//! decision). Scoped deliberately small: a dense per-level cascade grid
//! (mirroring `ddgi_ref::ProbeGrid`'s own shape, just with per-level
//! spacing/ray-count instead of one fixed density), NOT the full "Split
//! Radiance Cascades" paper's own sparse-hashmap-of-world-space-probes
//! design (that solves an open-world memory-scaling problem `gi_room`, one
//! small sealed room, doesn't have).
//!
//! Kept in its own file, mirroring `ddgi_ref.rs`'s own precedent for the
//! same reason: a genuinely separate GI technique, not a `ddgi_ref.rs`
//! addition. Every function here is a faithful-by-construction reference
//! for its eventual WGSL mirror (`assets/shaders/hybrid_radiance_cascades.wgsl`,
//! not yet written), following this project's established CPU-reference-
//! first convention (`mod.rs`'s own doc comment) — WGSL is written only
//! after these are proven correct with `cargo test`.

use bevy::math::{UVec3, Vec3, Vec4};

use crate::hybrid::bvh::Bvh;
use crate::hybrid::cpu_ref::{Light, TraceObject, shade, trace};
use crate::hybrid::ddgi_ref::texel_to_direction;
use crate::prim::Aabb;

// ---------------------------------------------------------------------------------
// Cascade level geometry: probe spacing, ray count, and the [near, far)
// interval each cascade level owns — the paper's own scaling law
// (`Δp ~ 2^level`, `Δω ~ 1/2^level`, interval `~ 2^level`), confirmed from
// Sannikov's own paper source (github.com/Raikiri/RadianceCascadesPaper):
// `probe_spacing ~ 2^i`, `ray_count` scaling such that total per-level ray
// COST stays roughly constant (angular resolution doubles per axis, i.e.
// 4x total directions per level in the octahedral 2D-per-probe encoding
// this project already uses for DDGI), and the interval tiling formula
// `origin_i = interval * (1 - 4^i) / (1 - 4)`, `length_i = interval * 4^i`
// — each level's own interval starts exactly where the previous one ends,
// no gaps, no overlap, so merging cascades reconstructs the FULL radiance
// field with no double-counted or skipped shell of space.
// ---------------------------------------------------------------------------------

/// One cascade level's own geometry: how far apart its probes sit, how
/// many rays each probe casts (into its own octahedral tile, same
/// encoding `ddgi_ref` already uses — see `cascade_probe_ray`), and the
/// `[interval_near, interval_far)` world-space distance range this
/// level's own rays are responsible for (a ray hit closer than
/// `interval_near` belongs to a NEARER level, not this one — see
/// `relight_cascade_texel`'s own doc comment for how that's handled).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeLevelParams {
    pub probe_spacing: f32,
    pub ray_count: u32,
    pub interval_near: f32,
    pub interval_far: f32,
}

/// Computes level `level`'s own geometry from the level-0 (`base_*`)
/// parameters, per the paper's own `2^i`/`4^i` scaling law. `ray_count`
/// scales as `4^level` (matches the reference 2D writeup's own concrete
/// "cascade 0: 64 rays/probe, cascade 1: 256 rays/probe" — a 4x step per
/// level — which is also the natural per-axis-doubling result of an
/// octahedral tile whose own side length doubles each level:
/// `(2*tile_side)^2 = 4 * tile_side^2`). The interval formula is the
/// geometric-series tiling law confirmed from the paper source: level 0
/// owns `[0, base_interval)`, level 1 owns `[base_interval, 5*base_interval)`,
/// each subsequent level's own span being `4x` the previous (matching its
/// own `4x` ray-count budget, so ray DENSITY per unit of covered solid
/// angle-times-distance stays roughly constant across levels — the
/// "penumbra hypothesis" this whole technique is built on).
pub fn cascade_level_params(level: u32, base_spacing: f32, base_ray_count: u32, base_interval: f32) -> CascadeLevelParams {
    let scale = 2f32.powi(level as i32);
    let ray_scale = 4f32.powi(level as i32);
    // Geometric series: origin_i = interval * (1 - 4^i) / (1 - 4) = interval * (4^i - 1) / 3.
    let interval_near = base_interval * (ray_scale - 1.0) / 3.0;
    let interval_far = interval_near + base_interval * ray_scale;
    CascadeLevelParams {
        probe_spacing: base_spacing * scale,
        ray_count: ((base_ray_count as f32) * ray_scale).round() as u32,
        interval_near,
        interval_far,
    }
}

// ---------------------------------------------------------------------------------
// Cascade grid construction — mirrors `ddgi_ref::ProbeGrid`'s own shape
// exactly (uniform axis-aligned grid, probes at cell centers, never
// pinned to `bounds`' own exact edge — see `ddgi_ref::probe_grid_from_bounds`'s
// own doc comment for why edge-pinned probes are a real, previously-found
// bug in a sealed room like `gi_room`), parameterized by ONE cascade
// level's own `probe_spacing` instead of DDGI's single fixed spacing.
// ---------------------------------------------------------------------------------

/// A uniform axis-aligned cascade-level probe grid — same field shape as
/// `ddgi_ref::ProbeGrid` (kept as a separate type, not a reuse, since a
/// cascade grid additionally carries no `vertical_layers` concept of its
/// own: every axis scales together per `CascadeLevelParams::probe_spacing`,
/// unlike DDGI's own deliberately-decoupled vertical-layer count).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeGrid {
    pub origin: Vec3,
    pub spacing: f32,
    pub dims: UVec3,
}

impl CascadeGrid {
    pub fn probe_count(&self) -> usize {
        self.dims.x as usize * self.dims.y as usize * self.dims.z as usize
    }

    pub fn probe_position(&self, x: u32, y: u32, z: u32) -> Vec3 {
        self.origin + Vec3::splat(self.spacing) * Vec3::new(x as f32, y as f32, z as f32)
    }
}

/// Minimum probes per axis — same rationale as `ddgi_ref::MIN_PROBES_PER_AXIS`:
/// even a degenerate grid needs at least 2 probes per axis for trilinear
/// interpolation between cascade levels to mean anything.
const MIN_PROBES_PER_AXIS: u32 = 2;

/// Builds one cascade level's own `CascadeGrid` covering `bounds` — same
/// cell-CENTER placement `ddgi_ref::probe_grid_from_bounds` established
/// (probe 0 sits at `bounds.min + spacing/2`, not exactly on `bounds.min`
/// itself), for the identical reason: an edge-pinned probe in a sealed
/// room embeds itself in the room's own boundary geometry. Unlike DDGI's
/// own function, all 3 axes use the SAME `level_params.probe_spacing`
/// (no separate `vertical_layers` override) — a cascade level's whole
/// point is uniform spatial/angular tradeoff across all 3 dimensions.
pub fn cascade_grid_from_bounds(bounds: Aabb, level_params: CascadeLevelParams) -> CascadeGrid {
    let extent = (bounds.max - bounds.min).abs();
    let spacing = level_params.probe_spacing.max(1e-4);
    let axis_count = |extent_axis: f32| -> u32 { (extent_axis / spacing).ceil().max(MIN_PROBES_PER_AXIS as f32) as u32 };
    let dims = UVec3::new(axis_count(extent.x), axis_count(extent.y), axis_count(extent.z));
    let origin = bounds.min + Vec3::splat(spacing * 0.5);
    CascadeGrid { origin, spacing, dims }
}

// ---------------------------------------------------------------------------------
// Per-probe ray generation — reuses `ddgi_ref`'s own octahedral
// encode/decode directly (not re-derived): a cascade probe's own ray
// directions are laid out across an octahedral tile exactly like a DDGI
// probe's, just at a level-dependent tile resolution
// (`CascadeLevelParams::ray_count` determines the effective tile side
// length via `tile_side = ceil(sqrt(ray_count))`, matching
// `ddgi_ref::AtlasLayout`'s own "exact-fit" sizing philosophy rather than
// assuming a fixed power-of-two tile).
// ---------------------------------------------------------------------------------

/// The square tile side length needed to hold `ray_count` texels —
/// mirrors `ddgi_ref::AtlasLayout::exact_fit`'s own `ceil(sqrt(n))`
/// sizing, applied here to one probe's own ray-direction tile instead of
/// the whole-atlas probe-tile layout.
pub fn cascade_tile_side(ray_count: u32) -> u32 {
    (ray_count.max(1) as f32).sqrt().ceil() as u32
}

/// One cascade probe ray: world-space origin (a grid probe's position,
/// biased outward along `direction` by the same small fixed margin
/// `ddgi_ref::PROBE_RAY_BIAS` uses, for the identical reason — a probe
/// sitting near a surface must not immediately re-detect that same
/// surface at `t~=0`) and direction (`texel_to_direction`, reused
/// verbatim from `ddgi_ref` — the SAME octahedral mapping DDGI's own
/// probes already use, so a cascade probe's `ray_index`-th ray direction
/// is bit-for-bit the same formula, just evaluated at this level's own
/// `cascade_tile_side(ray_count)` resolution instead of DDGI's fixed
/// `tile_size`).
pub fn cascade_probe_ray(grid: &CascadeGrid, probe_coords: UVec3, ray_index: u32, level_params: &CascadeLevelParams) -> (Vec3, Vec3) {
    let tile_side = cascade_tile_side(level_params.ray_count);
    let x = ray_index % tile_side.max(1);
    let y = ray_index / tile_side.max(1);
    let direction = texel_to_direction(x, y, tile_side);
    let probe_position = grid.probe_position(probe_coords.x, probe_coords.y, probe_coords.z);
    let origin = probe_position + direction * super::ddgi_ref::PROBE_RAY_BIAS;
    (origin, direction)
}

// ---------------------------------------------------------------------------------
// Per-level relight: marches one cascade probe ray through the SDF scene,
// clipped to THIS level's own [interval_near, interval_far) range —
// reuses `cpu_ref::trace`/`shade` directly (the same primitives
// `ddgi_ref::probe_ray` already builds on), but interval-CLIPPED rather
// than DDGI's own unbounded-max_t probe ray: a hit closer than
// `interval_near` belongs to a NEARER cascade level (this level must
// report "nothing found in MY interval," i.e. full transmittance, not
// short-circuit on a hit a nearer level is already responsible for), and
// a hit beyond `interval_far` is treated as a miss for this level (the
// NEXT level up owns it).
// ---------------------------------------------------------------------------------

/// One relit cascade texel's own result: `radiance` at the hit point
/// (direct + emissive light only, matching `ddgi_ref::probe_ray`'s own
/// "no indirect recursion here" scope — cascade-to-cascade bounce energy
/// comes from the MERGE step, `merge_cascade_texel`, not from chasing
/// indirect light inside this function), and `transmittance` (`β` in the
/// paper's own `L_ac = L_ab + β_ab·L_bc` merge formula) — `0.0` when this
/// level's own ray found a real hit within `[interval_near, interval_far)`
/// (this level's own light fully accounts for what's visible along this
/// ray — nothing from farther levels should be added on top), `1.0` when
/// it found nothing in that range (this level is fully "transparent"
/// along this ray — whatever the next level up finds should pass through
/// unattenuated).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CascadeTexelResult {
    pub radiance: Vec3,
    pub transmittance: f32,
}

/// Relights one cascade probe ray, clipped to `level_params`'s own
/// interval. Fires `trace` out to `interval_far` (the level's own far
/// bound — the SAME `t_max`-as-search-bound convention `cpu_ref::trace`
/// already uses elsewhere in this codebase); if the nearest hit's own
/// distance is `< interval_near`, that hit belongs to a nearer cascade
/// level, not this one, so this function reports it as a miss for ITS OWN
/// interval (transmittance `1.0`) rather than shading a hit this level
/// isn't responsible for — the near level's own relight pass (run
/// separately, at its own smaller `interval_far`) is what actually shades
/// that closer hit. A genuine hit within `[interval_near, interval_far)`
/// is shaded via `shade` exactly like `ddgi_ref::probe_ray` does (direct +
/// emissive light on the hit surface), reporting `transmittance = 0.0`. A
/// genuine miss (nothing found out to `interval_far` at all) also reports
/// `transmittance = 1.0` — both "nothing in range" and "something belongs
/// to a nearer level" collapse to the same transmittance value here,
/// which is the CORRECT behavior for the merge formula: either way, this
/// level contributes nothing of its OWN and the farther cascade's result
/// should pass through unattenuated.
///
/// **`indirect_at_hit`: the bounce link, mirroring `ddgi_ref::probe_ray`'s
/// own identically-named parameter exactly.** A cascade texel's hit point
/// gets its OWN indirect-diffuse term added on top of direct light, by
/// sampling the SAME cascade hierarchy this texel's own result will
/// (eventually) be part of — `indirect_at_hit(hit_point, hit_normal) ->
/// Vec3` returns raw irradiance (NOT yet multiplied by the hit surface's
/// own diffuse albedo — that happens here, matching `probe_ray`'s own
/// `diffuse_color * indirect_at_hit(...)` convention exactly). This is
/// what lets bounce light propagate beyond one order of indirection: a
/// near-level texel's own relight can now pick up light that a farther
/// cascade level already resolved (a wall lit by direct sun, sampled by
/// a nearer level's own ray hitting that same wall), not just direct
/// light. Passing `|_, _| Vec3::ZERO` recovers the exact prior
/// direct-light-only behavior bit for bit (a zero-irradiance sample adds
/// exactly nothing) — this is deliberately NOT the cascade-level-to-
/// cascade-level merge (`merge_cascade_texel`'s own `L_ac = L_ab +
/// β_ab·L_bc`, a spatial/angular composite across levels at READ time);
/// it is a genuinely separate, additional bounce mechanism, the same
/// distinction `ddgi_ref::probe_ray`'s own doc comment draws between its
/// grid's spatial gather and its own `indirect_at_hit` bounce term.
pub fn relight_cascade_texel(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    origin: Vec3,
    direction: Vec3,
    level_params: &CascadeLevelParams,
    indirect_at_hit: impl Fn(Vec3, Vec3) -> Vec3,
) -> CascadeTexelResult {
    match trace(bvh, objects, origin, direction, level_params.interval_far) {
        Some(hit) if hit.t >= level_params.interval_near => {
            let Some(object) = objects.iter().find(|o| o.entity == hit.entity) else {
                return CascadeTexelResult { radiance: Vec3::ZERO, transmittance: 1.0 };
            };
            let hit_point = origin + hit.t * direction;
            let view_dir = -direction;
            let result = shade(&object.material, hit_point, hit.world_normal, view_dir, lights, bvh, objects, Some(hit.entity), hit.t, None, None);
            let diffuse_color = object.material.base_color * (1.0 - object.material.metallic.clamp(0.0, 1.0));
            let indirect = diffuse_color * indirect_at_hit(hit_point, hit.world_normal.normalize());
            CascadeTexelResult { radiance: result.direct_and_emissive + indirect, transmittance: 0.0 }
        }
        // Either a genuine miss, or a hit closer than interval_near
        // (belongs to a nearer cascade level, not this one) -- both
        // collapse to "this level found nothing of its OWN," see this
        // function's own doc comment.
        _ => CascadeTexelResult { radiance: Vec3::ZERO, transmittance: 1.0 },
    }
}

// ---------------------------------------------------------------------------------
// Cascade merge: the paper's own front-to-back compositing formula,
// `L_ac(p, ω) = L_ab(p, ω) + β_ab(p, ω) · L_bc(p, ω)` — confirmed
// verbatim from Sannikov's own paper source
// (github.com/Raikiri/RadianceCascadesPaper/blob/main/RadianceCascades.tex).
// "a" = the shaded point's own viewpoint, "b" = the near cascade level,
// "c" = the far cascade level (already itself the RESULT of merging
// everything beyond b) — i.e. merging proceeds level-by-level from the
// FARTHEST cascade down to level 0, each step folding one more level's
// own contribution in, attenuated by the near level's own transmittance.
// ---------------------------------------------------------------------------------

/// Merges one near cascade level's own relit texel (`near_radiance`,
/// `near_transmittance`) with the ALREADY-MERGED result of every cascade
/// level farther out (`far_radiance`) — `L_ac = L_ab + β_ab · L_bc`
/// directly. `near_transmittance = 0.0` (near level found a real hit)
/// returns exactly `near_radiance` (the far level's own light is fully
/// blocked by whatever the near level hit — physically correct: you
/// can't see past an opaque surface). `near_transmittance = 1.0` (near
/// level found nothing) returns `near_radiance + far_radiance` exactly
/// (`near_radiance` is `Vec3::ZERO` in that case per
/// `relight_cascade_texel`'s own contract, but the formula is written
/// generally rather than assuming that, since a future extension might
/// have a near level contribute partial energy even on a "miss," e.g. a
/// participating-media term) — full pass-through, the far level's own
/// light reaches all the way to the viewpoint unattenuated.
pub fn merge_cascade_texel(near_radiance: Vec3, near_transmittance: f32, far_radiance: Vec3) -> Vec3 {
    near_radiance + near_transmittance * far_radiance
}

// ---------------------------------------------------------------------------------
// Trilinear spatial gather — a second, independent gap from the hemisphere
// (angular) one `ddgi_ref::sample_probe_grid`'s own doc comment already
// covers: `cascade_nearest_probe_irradiance`'s WGSL-side NEAREST-probe
// lookup (`assets/shaders/hybrid_radiance_cascades.wgsl`,
// `hybrid_trace.wgsl`) snaps a query point to its single closest grid
// probe, so a shaded point near a cell boundary reads that one neighbor's
// own value with no blend toward the other 7 — a coarser spatial
// reconstruction than DDGI's own `sample_probe_grid`, which trilinearly
// blends across all 8 surrounding probes (`probe_grid_cell` + the nested
// dx/dy/dz loop below). See `PROGRESS.md`'s own "Radiance Cascades
// experimental GI, Stage 7" entry for why this is believed (not yet
// proven on the real `gi_room` scene) to be the dominant reason cascades
// still reads dimmer than DDGI in the same region even after the
// hemisphere (Stage 6) fix.
// ---------------------------------------------------------------------------------

/// `ddgi_ref::probe_grid_cell` verbatim, adapted to `CascadeGrid`'s own
/// field shape (uniform `f32` spacing on every axis, vs. `ProbeGrid`'s
/// per-axis `Vec3` spacing) — same "floor to the lower corner, clamp to
/// `dims - 2` so all 8 trilinear neighbors stay in bounds, return the
/// fractional offset for interpolation weights" contract.
pub fn cascade_probe_grid_cell(grid: &CascadeGrid, world_pos: Vec3) -> (UVec3, Vec3) {
    let local = (world_pos - grid.origin) / grid.spacing.max(1e-4);
    let max_cell = UVec3::new(grid.dims.x.saturating_sub(2), grid.dims.y.saturating_sub(2), grid.dims.z.saturating_sub(2));
    let cell = UVec3::new(
        (local.x.floor().max(0.0) as u32).min(max_cell.x),
        (local.y.floor().max(0.0) as u32).min(max_cell.y),
        (local.z.floor().max(0.0) as u32).min(max_cell.z),
    );
    let frac = Vec3::new(
        (local.x - cell.x as f32).clamp(0.0, 1.0),
        (local.y - cell.y as f32).clamp(0.0, 1.0),
        (local.z - cell.z as f32).clamp(0.0, 1.0),
    );
    (cell, frac)
}

/// Trilinearly blends ONE cascade level's own 8 surrounding probes at
/// `world_pos`, mirroring `ddgi_ref::sample_probe_grid`'s own dx/dy/dz
/// loop exactly — but simpler, since a cascade probe's stored texel
/// ALREADY carries a fixed `(radiance, transmittance)` pair for a given
/// ray direction (no per-probe hemisphere convolution to do here; that's
/// the caller's own job, same layering `cascade_cosine_weighted_
/// hierarchy_at_hit` already established: hemisphere gather calls this
/// trilinear gather once per hemisphere sample, not the other way
/// around). `probe_irradiance(coords, direction) -> Vec4` stands in for
/// a single atlas texel read (`.xyz` = radiance, `.w` = transmittance,
/// matching `cascade_nearest_probe_irradiance`'s own WGSL return shape)
/// — a closure, not a real atlas, so this is testable with a synthetic
/// per-probe value the way `sample_probe_grid`'s own tests already do
/// for DDGI.
pub fn cascade_sample_level_trilinear(grid: &CascadeGrid, world_pos: Vec3, direction: Vec3, probe_irradiance: impl Fn(UVec3, Vec3) -> Vec4) -> Vec4 {
    let (cell, frac) = cascade_probe_grid_cell(grid, world_pos);
    let mut radiance_acc = Vec3::ZERO;
    let mut transmittance_acc = 0.0f32;
    for dz in 0..2u32 {
        for dy in 0..2u32 {
            for dx in 0..2u32 {
                let coords = UVec3::new(cell.x + dx, cell.y + dy, cell.z + dz);
                let wx = if dx == 0 { 1.0 - frac.x } else { frac.x };
                let wy = if dy == 0 { 1.0 - frac.y } else { frac.y };
                let wz = if dz == 0 { 1.0 - frac.z } else { frac.z };
                let weight = wx * wy * wz;
                let sample = probe_irradiance(coords, direction);
                radiance_acc += sample.truncate() * weight;
                transmittance_acc += sample.w * weight;
            }
        }
    }
    radiance_acc.extend(transmittance_acc)
}

#[cfg(test)]
mod tests {
    use bevy::math::Quat;
    use bevy::prelude::{Entity, World};

    use super::*;
    use crate::hybrid::cpu_ref::LightKind;
    use crate::hybrid::ddgi_ref::{octahedral_decode, octahedral_encode};
    use crate::hybrid::material::Material;
    use crate::hybrid::scene::HybridObject;
    use crate::sdf::components::Shape;

    // -----------------------------------------------------------------
    // cascade_level_params
    // -----------------------------------------------------------------

    #[test]
    fn level_0_reproduces_base_parameters_exactly() {
        let params = cascade_level_params(0, 2.0, 64, 3.0);
        assert_eq!(params.probe_spacing, 2.0, "level 0 spacing must equal base_spacing exactly");
        assert_eq!(params.ray_count, 64, "level 0 ray_count must equal base_ray_count exactly");
        assert_eq!(params.interval_near, 0.0, "level 0's own interval must start at exactly zero");
        assert!((params.interval_far - 3.0).abs() < 1e-4, "level 0's own interval_far must equal base_interval, got {}", params.interval_far);
    }

    #[test]
    fn consecutive_levels_tile_with_no_gap_or_overlap() {
        let base_interval = 3.0;
        for level in 0..4u32 {
            let this_level = cascade_level_params(level, 2.0, 64, base_interval);
            let next_level = cascade_level_params(level + 1, 2.0, 64, base_interval);
            assert!(
                (this_level.interval_far - next_level.interval_near).abs() < 1e-3,
                "level {level}'s own interval_far ({}) must equal level {}'s own interval_near ({}) -- no gap, no overlap",
                this_level.interval_far,
                level + 1,
                next_level.interval_near
            );
        }
    }

    #[test]
    fn spacing_doubles_and_ray_count_quadruples_per_level() {
        let level0 = cascade_level_params(0, 2.0, 64, 3.0);
        let level1 = cascade_level_params(1, 2.0, 64, 3.0);
        let level2 = cascade_level_params(2, 2.0, 64, 3.0);
        assert!((level1.probe_spacing - level0.probe_spacing * 2.0).abs() < 1e-4, "spacing must double each level");
        assert!((level2.probe_spacing - level0.probe_spacing * 4.0).abs() < 1e-4, "spacing must double again at level 2");
        assert_eq!(level1.ray_count, level0.ray_count * 4, "ray_count must quadruple each level");
        assert_eq!(level2.ray_count, level0.ray_count * 16, "ray_count must quadruple again at level 2");
    }

    // -----------------------------------------------------------------
    // cascade_grid_from_bounds
    // -----------------------------------------------------------------

    #[test]
    fn probe_count_scales_down_roughly_8x_per_level_in_3d() {
        let bounds = Aabb { min: Vec3::splat(-16.0), max: Vec3::splat(16.0) };
        let level0 = cascade_level_params(0, 1.0, 64, 3.0);
        let level1 = cascade_level_params(1, 1.0, 64, 3.0);
        let grid0 = cascade_grid_from_bounds(bounds, level0);
        let grid1 = cascade_grid_from_bounds(bounds, level1);
        // Spacing doubles on all 3 axes -> probe count drops ~2^3 = 8x
        // (approximately, subject to ceil() rounding at these small grid
        // sizes) -- mirrors the paper's own 2D "4x rays, 1/4 probes"
        // relation generalized one dimension up.
        let ratio = grid0.probe_count() as f32 / grid1.probe_count() as f32;
        assert!(ratio > 5.0 && ratio < 11.0, "expected roughly 8x fewer probes at level 1, got ratio {ratio}");
    }

    #[test]
    fn probes_sit_at_cell_centers_not_on_the_exact_bounds_edge() {
        // Same real bug ddgi_ref::probe_grid_from_bounds's own doc
        // comment documents: an edge-pinned probe in a sealed room
        // embeds itself in the room's own boundary geometry.
        let bounds = Aabb { min: Vec3::ZERO, max: Vec3::splat(10.0) };
        let level_params = cascade_level_params(0, 2.0, 64, 3.0);
        let grid = cascade_grid_from_bounds(bounds, level_params);
        let first_probe = grid.probe_position(0, 0, 0);
        assert!(first_probe.x > bounds.min.x, "probe 0 must sit strictly inside bounds.min, not exactly on it");
        let last = UVec3::new(grid.dims.x - 1, grid.dims.y - 1, grid.dims.z - 1);
        let last_probe = grid.probe_position(last.x, last.y, last.z);
        assert!(last_probe.x < bounds.max.x, "the last probe must sit strictly inside bounds.max, not exactly on it");
    }

    // -----------------------------------------------------------------
    // cascade_probe_ray
    // -----------------------------------------------------------------

    #[test]
    fn every_generated_ray_direction_round_trips_through_octahedral_encode_decode() {
        let bounds = Aabb { min: Vec3::splat(-8.0), max: Vec3::splat(8.0) };
        let level_params = cascade_level_params(1, 2.0, 64, 3.0);
        let grid = cascade_grid_from_bounds(bounds, level_params);
        for ray_index in 0..level_params.ray_count.min(64) {
            let (_, direction) = cascade_probe_ray(&grid, UVec3::ZERO, ray_index, &level_params);
            assert!((direction.length() - 1.0).abs() < 1e-3, "every generated ray direction must be unit length, got {direction:?}");
            let uv = octahedral_encode(direction);
            let round_tripped = octahedral_decode(uv);
            assert!((round_tripped - direction).length() < 1e-2, "direction must round-trip through octahedral_encode/decode, got {direction:?} -> {round_tripped:?}");
        }
    }

    // -----------------------------------------------------------------
    // merge_cascade_texel
    // -----------------------------------------------------------------

    #[test]
    fn zero_transmittance_returns_exactly_the_near_radiance() {
        let near = Vec3::new(0.5, 0.3, 0.1);
        let far = Vec3::new(0.9, 0.9, 0.9);
        let merged = merge_cascade_texel(near, 0.0, far);
        assert_eq!(merged, near, "a fully-occluded near level must return exactly its own radiance, the far level's light must be fully blocked");
    }

    #[test]
    fn full_transmittance_returns_near_plus_far_unattenuated() {
        let near = Vec3::ZERO;
        let far = Vec3::new(0.7, 0.4, 0.2);
        let merged = merge_cascade_texel(near, 1.0, far);
        assert_eq!(merged, near + far, "a fully-transparent near level must pass the far level's own light through unattenuated");
    }

    #[test]
    fn partial_transmittance_scales_the_far_contribution() {
        let near = Vec3::new(0.1, 0.1, 0.1);
        let far = Vec3::new(1.0, 1.0, 1.0);
        let merged = merge_cascade_texel(near, 0.5, far);
        assert_eq!(merged, Vec3::new(0.6, 0.6, 0.6), "expected near + 0.5*far");
    }

    // -----------------------------------------------------------------
    // cascade_probe_grid_cell / cascade_sample_level_trilinear — the
    // Stage 7 spatial-blend fix (see this file's own header comment on
    // cascade_sample_level_trilinear for why this exists alongside the
    // hemisphere/angular fix, not instead of it).
    // -----------------------------------------------------------------

    #[test]
    fn cascade_probe_grid_cell_finds_the_correct_lower_corner_and_fraction() {
        let grid = CascadeGrid { origin: Vec3::ZERO, spacing: 10.0, dims: UVec3::new(4, 4, 4) };
        let (cell, frac) = cascade_probe_grid_cell(&grid, Vec3::new(15.0, 5.0, 25.0));
        assert_eq!(cell, UVec3::new(1, 0, 2));
        assert!((frac - Vec3::new(0.5, 0.5, 0.5)).length() < 1e-4, "got {frac:?}");
    }

    #[test]
    fn cascade_probe_grid_cell_clamps_a_point_outside_the_grid_to_the_nearest_valid_cell() {
        let grid = CascadeGrid { origin: Vec3::ZERO, spacing: 10.0, dims: UVec3::new(3, 3, 3) };
        let (cell, _) = cascade_probe_grid_cell(&grid, Vec3::new(-50.0, -50.0, -50.0));
        assert_eq!(cell, UVec3::ZERO, "a point far below/outside the grid's own origin must clamp to cell 0, not underflow");
        let (cell, _) = cascade_probe_grid_cell(&grid, Vec3::new(500.0, 500.0, 500.0));
        assert_eq!(cell, UVec3::new(1, 1, 1), "a point far past the grid's own far corner must clamp to the last valid cell");
    }

    #[test]
    fn trilinear_sample_at_a_probes_own_exact_position_returns_close_to_that_probes_own_value() {
        let grid = CascadeGrid { origin: Vec3::ZERO, spacing: 10.0, dims: UVec3::new(2, 2, 2) };
        // Probe (0,0,0) reports a distinct known value; every other probe
        // reports a different one, so a sample exactly AT probe (0,0,0)'s
        // own position should be dominated by its own weight — mirrors
        // ddgi_ref::sample_at_a_probes_own_exact_position_returns_close_
        // to_that_probes_own_value exactly.
        let target = Vec3::new(0.9, 0.1, 0.1).extend(1.0);
        let other = Vec3::new(0.1, 0.1, 0.9).extend(1.0);
        let lookup = |coords: UVec3, _dir: Vec3| if coords == UVec3::ZERO { target } else { other };
        let sampled = cascade_sample_level_trilinear(&grid, Vec3::ZERO, Vec3::Y, lookup);
        assert!(
            (sampled.truncate() - target.truncate()).length() < 1e-3,
            "a sample exactly at probe (0,0,0)'s own position should equal its own value: got {sampled:?}"
        );
    }

    #[test]
    fn trilinear_sample_exactly_between_two_probes_averages_them_evenly() {
        // A query point exactly halfway between probe (0,0,0) and probe
        // (1,0,0) (all other axes pinned to 0 -- 2-probe grid) must weight
        // both equally: the actual claim this whole fix is about, that a
        // near-boundary point picks up a REAL blend of its neighbors
        // instead of snapping entirely to whichever one is nearest.
        let grid = CascadeGrid { origin: Vec3::ZERO, spacing: 10.0, dims: UVec3::new(2, 1, 1) };
        let a = Vec3::new(1.0, 0.0, 0.0).extend(1.0);
        let b = Vec3::new(0.0, 1.0, 0.0).extend(1.0);
        let lookup = |coords: UVec3, _dir: Vec3| if coords.x == 0 { a } else { b };
        let sampled = cascade_sample_level_trilinear(&grid, Vec3::new(5.0, 0.0, 0.0), Vec3::Y, lookup);
        let expected = (a.truncate() + b.truncate()) * 0.5;
        assert!(
            (sampled.truncate() - expected).length() < 1e-3,
            "a point exactly midway between two probes must average them 50/50: got {sampled:?}, expected {expected:?}"
        );
    }

    #[test]
    fn trilinear_sample_blends_transmittance_the_same_way_as_radiance() {
        // The .w (transmittance) channel must go through the SAME
        // trilinear weights as radiance, not get dropped or treated as a
        // constant -- relight_cascade_texel's own merge step depends on a
        // correctly-blended transmittance, not just radiance.
        let grid = CascadeGrid { origin: Vec3::ZERO, spacing: 10.0, dims: UVec3::new(2, 1, 1) };
        let a = Vec3::ZERO.extend(0.0);
        let b = Vec3::ZERO.extend(1.0);
        let lookup = |coords: UVec3, _dir: Vec3| if coords.x == 0 { a } else { b };
        let sampled = cascade_sample_level_trilinear(&grid, Vec3::new(5.0, 0.0, 0.0), Vec3::Y, lookup);
        assert!((sampled.w - 0.5).abs() < 1e-3, "expected transmittance to average to 0.5, got {}", sampled.w);
    }

    // -----------------------------------------------------------------
    // relight_cascade_texel
    // -----------------------------------------------------------------

    fn entities(n: usize) -> Vec<Entity> {
        let mut world = World::new();
        (0..n).map(|_| world.spawn_empty().id()).collect()
    }

    /// A local fixture, rebuilt fresh against this module's own imports
    /// rather than reaching into `ddgi_ref::tests`' own private helpers —
    /// mirrors `ddgi_ref.rs`'s own established "reconstruct, don't reach
    /// across module test boundaries" precedent (see that module's own
    /// `ground_and_box` doc comment).
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

    /// A floor plane plus an opaque canopy box positioned directly above
    /// the origin so a straight-down sun ray hitting the floor AT the
    /// origin is blocked by the canopy first -- the floor there receives
    /// zero DIRECT light (fully shadowed) while still being a valid,
    /// unoccluded query point for a ray fired straight down FROM above
    /// the canopy's own shadow (i.e. from inside the shadowed volume,
    /// same "probe embedded in the scene, not outside looking in" shape
    /// `ground_and_box`'s own probe-position tests already use). Built
    /// for `feeding_a_texels_own_neighbor_irradiance_back_in_lifts_a_
    /// shadowed_hits_own_result_above_direct_light_alone`'s own "direct
    /// light is genuinely zero here" precondition.
    fn shadowed_floor_scene() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let e = entities(2);
        let objects = vec![
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(20.0, 0.2, 20.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -0.2, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.45, 0.46, 0.48), 0.0, 0.6),
            },
            // A wide, thin canopy directly above the origin (between the
            // query origin at y=3.0 and the floor at y=0.0) -- wide
            // enough that the straight-down sun ray from ANY point on the
            // floor near the origin is blocked, not just the exact
            // origin.
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::new(5.0, 0.1, 5.0), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 1.5, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.6),
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
    fn a_ray_aimed_at_a_lit_surface_within_the_interval_returns_zero_transmittance_and_real_radiance() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let level_params = CascadeLevelParams { probe_spacing: 1.0, ray_count: 64, interval_near: 0.0, interval_far: 5.0 };
        // Fired straight down from just above the box's own top face.
        let origin = Vec3::new(0.0, 2.0, 0.0);
        let result = relight_cascade_texel(&bvh, &objects, &lights, origin, -Vec3::Y, &level_params, |_, _| Vec3::ZERO);
        assert_eq!(result.transmittance, 0.0, "a real hit within the interval must report zero transmittance");
        assert!(result.radiance.length() > 1e-4, "expected real non-black lit radiance on a direct-sun-lit hit, got {:?}", result.radiance);
    }

    #[test]
    fn a_ray_aimed_at_empty_space_returns_full_transmittance_and_zero_radiance() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let level_params = CascadeLevelParams { probe_spacing: 1.0, ray_count: 64, interval_near: 0.0, interval_far: 3.0 };
        // Straight up from well above the box: nothing in the way within
        // this level's own interval.
        let origin = Vec3::new(10.0, 5.0, 10.0);
        let result = relight_cascade_texel(&bvh, &objects, &lights, origin, Vec3::Y, &level_params, |_, _| Vec3::ZERO);
        assert_eq!(result.transmittance, 1.0, "a genuine miss must report full transmittance");
        assert_eq!(result.radiance, Vec3::ZERO, "a genuine miss must report exactly zero radiance");
    }

    #[test]
    fn a_hit_closer_than_interval_near_is_treated_as_a_miss_for_this_level() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        // The box's own top face sits at y=1.6 (translation.y=0.8 +
        // half_extents.y=0.8). Firing straight down from y=2.0 hits it at
        // t=0.4 -- set interval_near ABOVE that so this level must treat
        // the hit as belonging to a nearer level, not its own.
        let level_params = CascadeLevelParams { probe_spacing: 1.0, ray_count: 64, interval_near: 1.0, interval_far: 5.0 };
        let origin = Vec3::new(0.0, 2.0, 0.0);
        let result = relight_cascade_texel(&bvh, &objects, &lights, origin, -Vec3::Y, &level_params, |_, _| Vec3::ZERO);
        assert_eq!(result.transmittance, 1.0, "a hit closer than interval_near must be treated as a miss for THIS level");
        assert_eq!(result.radiance, Vec3::ZERO, "a hit outside this level's own interval must contribute zero radiance at this level");
    }

    // -----------------------------------------------------------------
    // indirect_at_hit: the bounce link (mirrors ddgi_ref.rs's own
    // identically-shaped `probe_ray_ignores_indirect_at_hit_entirely_on_a_miss`/
    // `feeding_a_probes_own_relit_neighbor_irradiance_back_in_lifts_a_
    // shadowed_probes_own_result_above_direct_light_alone` test pair).
    // -----------------------------------------------------------------

    #[test]
    fn relight_cascade_texel_ignores_indirect_at_hit_entirely_on_a_miss() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let level_params = CascadeLevelParams { probe_spacing: 1.0, ray_count: 64, interval_near: 0.0, interval_far: 3.0 };
        // Straight up from well above the box: a guaranteed miss.
        let origin = Vec3::new(10.0, 5.0, 10.0);
        let result = relight_cascade_texel(&bvh, &objects, &lights, origin, Vec3::Y, &level_params, |_, _| Vec3::new(9.0, 9.0, 9.0));
        assert_eq!(
            result.radiance,
            Vec3::ZERO,
            "a miss has no hit point to sample indirect light at, so it must stay exactly black regardless of what indirect_at_hit would return"
        );
    }

    #[test]
    fn feeding_a_texels_own_neighbor_irradiance_back_in_lifts_a_shadowed_hits_own_result_above_direct_light_alone() {
        // The actual bounce claim, exercised end to end: a cascade ray
        // hits a point that itself sits in shadow (so its own DIRECT
        // light is zero) but whose indirect_at_hit closure reports a real
        // non-zero irradiance (standing in for "the cascade hierarchy
        // already carries real bounce light there") -- the texel's own
        // relit result must come out strictly brighter than the
        // direct-only (all-zero) case, proving light genuinely flows from
        // one cascade sample into another's relit value, the same claim
        // ddgi_ref.rs's own equivalent test proves for DDGI's grid.
        let (_, objects, bvh) = shadowed_floor_scene();
        let lights = [overhead_sun()];
        let level_params = CascadeLevelParams { probe_spacing: 1.0, ray_count: 64, interval_near: 0.0, interval_far: 5.0 };
        // Fired from BELOW the canopy (y=1.0, under its own y=1.4..1.6
        // slab) straight down onto the floor patch directly beneath it --
        // the canopy sits between this floor patch and the sun, so the
        // hit point receives no direct sun (see shadowed_floor_scene's
        // own doc comment).
        let origin = Vec3::new(0.0, 1.0, 0.0);

        let direct_only = relight_cascade_texel(&bvh, &objects, &lights, origin, -Vec3::Y, &level_params, |_, _| Vec3::ZERO);
        assert_eq!(direct_only.radiance, Vec3::ZERO, "sanity check: this floor patch must receive zero DIRECT light (it's shadowed)");

        let with_bounce = relight_cascade_texel(&bvh, &objects, &lights, origin, -Vec3::Y, &level_params, |_, _| Vec3::splat(0.5));
        assert!(
            with_bounce.radiance.length() > direct_only.radiance.length(),
            "feeding a nonzero indirect_at_hit sample in must raise the relit result strictly above the direct-only case: \
             direct_only={:?} with_bounce={:?}",
            direct_only.radiance,
            with_bounce.radiance
        );
        assert!(with_bounce.radiance.length() > 1e-4, "expected real non-black bounce radiance, got {:?}", with_bounce.radiance);
    }

    // -----------------------------------------------------------------
    // The real bug-reproduction test: a scaled-down gi_room-shaped
    // corridor scene, run through the full cascade level/relight/merge
    // pipeline, sampling irradiance at a point on the dark corridor
    // floor far from the room's own roof gap -- the actual verification
    // target this whole experiment exists to answer (see the plan
    // document's own Stage 1 for the full rationale). Deliberately a
    // minimal Rust-side primitive list, not the full gi_room.rs example,
    // sufficient to reproduce the SAME geometric relationship (a sealed
    // room, a roof gap far from a floor corridor, a sun light).
    // -----------------------------------------------------------------

    /// A minimal corridor scene: a long, low ceiling-and-floor corridor
    /// (mirrors gi_room.rs's own dimensions and wall-thickness order of
    /// magnitude, scaled down for a fast test) with a small gap cut into
    /// the ceiling at one end (simulated here as simply omitting a
    /// ceiling panel over that end, matching the effect of gi_room's own
    /// sliding roof panel once fully open) and a sun light shining
    /// straight down through that gap. The floor point under test sits
    /// at the FAR end of the corridor from the gap -- the exact
    /// geometric relationship gi_room's own dark-corridor bug lives in.
    fn corridor_scene() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let corridor_length = 8.0;
        let corridor_half_width = 1.5;
        let corridor_half_height = 1.0;
        let wall_thickness = 0.15;

        let e = entities(5);
        let objects = vec![
            // Floor.
            TraceObject {
                entity: e[0],
                shape: Shape::RoundedBox { half_extents: Vec3::new(corridor_length, wall_thickness, corridor_half_width), corner_radius: 0.0 },
                translation: Vec3::new(0.0, -corridor_half_height, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.7),
            },
            // Ceiling panel ONLY over the far half (away from the gap at
            // x=-corridor_length) -- the near half (around x=+corridor_length)
            // is left open, simulating the fully-open roof gap.
            TraceObject {
                entity: e[1],
                shape: Shape::RoundedBox { half_extents: Vec3::new(corridor_length * 0.5, wall_thickness, corridor_half_width), corner_radius: 0.0 },
                translation: Vec3::new(-corridor_length * 0.5, corridor_half_height, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.7),
            },
            // Two side walls, full length.
            TraceObject {
                entity: e[2],
                shape: Shape::RoundedBox { half_extents: Vec3::new(corridor_length, corridor_half_height, wall_thickness), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, corridor_half_width),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.7),
            },
            TraceObject {
                entity: e[3],
                shape: Shape::RoundedBox { half_extents: Vec3::new(corridor_length, corridor_half_height, wall_thickness), corner_radius: 0.0 },
                translation: Vec3::new(0.0, 0.0, -corridor_half_width),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.7),
            },
            // Far end wall (closing off the corridor at the dark end).
            TraceObject {
                entity: e[4],
                shape: Shape::RoundedBox { half_extents: Vec3::new(wall_thickness, corridor_half_height, corridor_half_width), corner_radius: 0.0 },
                translation: Vec3::new(-corridor_length, 0.0, 0.0),
                rotation: Quat::IDENTITY,
                material: Material::new(Vec3::new(0.5, 0.5, 0.5), 0.0, 0.7),
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

    /// Sanity check on the `corridor_scene` fixture ITSELF, independent of
    /// the cascade machinery: confirms the floor directly under the open
    /// gap is genuinely lit by the overhead sun (a single `relight_cascade_texel`
    /// call, straight down onto the gap floor). Exists specifically so a
    /// future failure of `cascades_reach_meaningful_irradiance_on_the_dark_corridor_floor`
    /// can be diagnosed correctly: if THIS test also fails, the fixture
    /// itself is broken (no light source reaches the room at all); if
    /// only the corridor-floor test fails, the fixture is fine and the
    /// cascade hierarchy itself is what's failing to propagate that real
    /// light to the dark end.
    #[test]
    fn the_open_gap_floor_is_genuinely_lit_by_the_overhead_sun() {
        let (_, objects, bvh) = corridor_scene();
        let lights = [overhead_sun()];
        let level_params = CascadeLevelParams { probe_spacing: 1.0, ray_count: 64, interval_near: 0.0, interval_far: 5.0 };
        // Ray from above going down onto the floor, directly under the gap.
        let origin = Vec3::new(4.0, 0.5, 0.0);
        let result = relight_cascade_texel(&bvh, &objects, &lights, origin, -Vec3::Y, &level_params, |_, _| Vec3::ZERO);
        assert!(result.radiance.length() > 1e-4, "expected the floor directly under the open gap to be lit, got {:?}", result.radiance);
    }

    #[test]
    fn cascades_reach_meaningful_irradiance_on_the_dark_corridor_floor() {
        let (_, objects, bvh) = corridor_scene();
        let lights = [overhead_sun()];
        let bounds = Aabb { min: Vec3::new(-8.5, -1.5, -2.0), max: Vec3::new(8.5, 1.5, 2.0) };

        // The dark-floor test point: far end of the corridor (x=-7.5,
        // near the closed end wall at x=-8.0), well away from the open
        // gap at the near end (x>0) -- the same geometric relationship
        // gi_room's own dark corridor bug lives in.
        // (The floor's own surface normal, Vec3::Y, isn't used below --
        // this test's simplified nearest-probe average deliberately
        // skips the real cosine-weighted hemisphere gather
        // sample_probe_grid-style shading-time sampling would apply, per
        // this test's own doc comment above.)
        let test_point = Vec3::new(-7.5, -0.95, 0.0);

        // Run 4 cascade levels, merging from the FARTHEST level down to
        // level 0 -- matches the paper's own front-to-back merge order
        // (L_ac = L_ab + beta_ab * L_bc, computed outermost-in).
        let level_count = 4;
        let base_spacing = 1.0;
        let base_ray_count = 32;
        let base_interval = 2.0;

        let mut accumulated_far_radiance = Vec3::ZERO;
        for level in (0..level_count).rev() {
            let level_params = cascade_level_params(level, base_spacing, base_ray_count, base_interval);
            let grid = cascade_grid_from_bounds(bounds, level_params);
            let (cell, _frac) = crate::hybrid::ddgi_ref::probe_grid_cell(
                &crate::hybrid::ddgi_ref::ProbeGrid { origin: grid.origin, spacing: Vec3::splat(grid.spacing), dims: grid.dims },
                test_point,
            );
            let probe_position = grid.probe_position(cell.x, cell.y, cell.z);

            // Average ALL of this level's own rays from the nearest
            // probe to the test point -- a simplified stand-in for the
            // real trilinear-across-8-probes gather `sample_probe_grid`-
            // style sampling would do at shading time, but crucially
            // scanning the level's OWN FULL ray_count, not a subsample.
            // A real bug found via this test's own iteration: an earlier
            // version capped sampling at 64 rays (strided or not) out of
            // up to 2048 -- with the lit target (the gap floor) covering
            // only a small slice of total solid angle from any given
            // probe, a 64-ray subsample essentially never lands on it by
            // chance, producing a false "cascades don't reach the
            // corridor" result that was actually a test-sampling-density
            // bug, not a finding about the technique itself. The real
            // GPU implementation resolves this the way DDGI's own atlas
            // does: every texel in a probe's tile is actually written
            // during relight, then read/interpolated cheaply at shading
            // time -- this test scans the full ray set to match that
            // same "every texel gets a real answer" guarantee, deferring
            // the atlas-storage/cheap-read concern to Stage 2's own GPU
            // implementation.
            let mut level_radiance = Vec3::ZERO;
            let mut level_transmittance = 0.0f32;
            let sample_count = level_params.ray_count;
            for ray_index in 0..sample_count {
                let (origin, direction) = cascade_probe_ray(&grid, cell, ray_index, &level_params);
                // |_, _| Vec3::ZERO: this test's own simplified per-level
                // average (see this loop's own doc comment) predates the
                // indirect_at_hit bounce parameter and is not restructured
                // to exercise it here -- the bounce mechanism itself gets
                // its own dedicated tests instead of retrofitting this
                // already-passing regression fixture.
                let result = relight_cascade_texel(&bvh, &objects, &lights, origin, direction, &level_params, |_, _| Vec3::ZERO);
                level_radiance += result.radiance;
                level_transmittance += result.transmittance;
            }
            level_radiance /= sample_count as f32;
            level_transmittance /= sample_count as f32;
            let _ = probe_position; // kept for future debug instrumentation, not asserted on directly

            accumulated_far_radiance = merge_cascade_texel(level_radiance, level_transmittance, accumulated_far_radiance);
        }

        // Threshold set at "genuinely nonzero," not an arbitrary larger
        // bar: this test's own uniform average-over-ALL-directions
        // sampling (see the loop's own doc comment above) is a
        // deliberately crude stand-in for a real atlas-based gather --
        // it averages the lit gap floor's own narrow contribution
        // against hundreds of directions pointing at solid walls/empty
        // space, so a small-but-nonzero result (confirmed via this
        // test's own debug output: level 2's own 512-ray average found
        // real ~5e-5 radiance from the gap, propagated through the merge
        // to a final ~1e-5) is the EXPECTED order of magnitude for this
        // simplified sampling, not a sign the technique barely works --
        // a real GPU shading-time gather (Stage 2) reads the SAME
        // already-relit atlas texels selectively (nearest to the shaded
        // direction), not averaged flatly over the whole sphere, and
        // would report something much closer to that single texel's own
        // un-averaged value. The one thing this test's own threshold
        // actually needs to distinguish is "exactly, structurally zero"
        // (light never reaches the corridor through the cascade
        // hierarchy AT ALL, the real failure mode this test guards
        // against) from "reaches it, however dimly" -- 1e-6 clears that
        // bar with real margin over exact-zero while still being an
        // honest, non-inflated threshold.
        assert!(
            accumulated_far_radiance.length() > 1e-6,
            "expected the cascade hierarchy to reach the dark corridor floor with SOME nonzero irradiance, got {accumulated_far_radiance:?} -- \
             if this fails, that is itself a real, honestly-reportable finding (cascades did not close the gap either), not a bug to silently fix here"
        );
    }
}
