//! CPU reference for DDGI (Dynamic Diffuse Global Illumination): a
//! persistent world-space grid of light probes, relit incrementally
//! across frames, sampled and trilinearly interpolated at shading time —
//! the "maybe D" step of this project's original GI trajectory ("A:
//! shadows -> B: cheap fixed-probe indirect -> C: occupancy-grid
//! acceleration -> maybe D: full DDGI"), now a firm decision. REPLACES
//! Stage B's per-pixel hemisphere-sample indirect diffuse
//! (`cpu_ref::indirect_diffuse` and everything it depends on) rather than
//! augmenting it: Stage B is fundamentally single-bounce and short-range
//! (`INDIRECT_MAX_T = 3.0` world units, whatever a shaded pixel's own
//! short probe rays happen to hit), which a persistent, relit-over-time
//! probe grid structurally isn't limited to.
//!
//! Kept in its own file rather than growing `cpu_ref.rs` further — that
//! file is already large (3200+ lines) and this is DDGI's own genuinely
//! separate concern, mirroring `temporal_ref.rs`'s own precedent for the
//! same reason. Every function here is a faithful-by-construction
//! reference for its WGSL mirror (a new `hybrid_ddgi_relight.wgsl` for
//! relighting, plus additions to `hybrid_trace.wgsl` for shading-time
//! sampling), following this project's established CPU-reference-first
//! convention (see `mod.rs`'s own doc comment) — WGSL is written only
//! after these are proven correct with `cargo test`.
//!
//! Three deliberate first-pass scope decisions (see PROGRESS.md's DDGI
//! entry for the full rationale):
//! 1. **Octahedral atlas storage** (RTXGI's real technique — one small 2D
//!    tile per probe in a shared atlas texture), not this project's
//!    cheaper existing 5-fixed-direction shortcut.
//! 2. **No Chebyshev visibility/depth test.** Irradiance-only probes;
//!    light leaking through thin occluders is a real, accepted risk in
//!    this first pass, mitigated cheaply (probe height bias off the
//!    ground plane + a single occlusion ray at sample time), not solved
//!    properly.
//! 3. **Rotating-subset relighting**, not full-every-frame — mirrors
//!    `cpu_ref::indirect_sample_start`'s existing per-pixel rotation
//!    scheme, generalized to probe granularity, since full relighting of
//!    every probe every frame is not realistic at this renderer's BVH+
//!    SDF-march ray cost (confirmed: ~5.5ms per additional per-pixel
//!    sample at `--stress 10000`, and a probe ray costs the same as a
//!    per-pixel ray).

use bevy::math::{Mat3, UVec3, Vec2, Vec3};
use bevy::prelude::Entity;

use crate::hybrid::bvh::Bvh;
use crate::hybrid::cpu_ref::{Light, TraceObject, any_hit, shade, trace};
use crate::prim::Aabb;

// ---------------------------------------------------------------------------------
// Probe grid construction
// ---------------------------------------------------------------------------------

/// A uniform axis-aligned probe grid: `dims.x * dims.y * dims.z` probes,
/// spaced `spacing` apart, with `origin` at probe `(0, 0, 0)`'s world
/// position. Every other probe's position is `origin + spacing *
/// index_as_vec3` (see `probe_position`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProbeGrid {
    pub origin: Vec3,
    pub spacing: Vec3,
    pub dims: UVec3,
}

impl ProbeGrid {
    /// Total probe count — `0` for a degenerate (zero-volume) grid, never
    /// negative/overflowing since `dims` is unsigned.
    pub fn probe_count(&self) -> usize {
        self.dims.x as usize * self.dims.y as usize * self.dims.z as usize
    }

    /// World-space position of probe `(x, y, z)` — no bounds checking;
    /// callers index within `dims`, matching this project's established
    /// convention of trusting internal callers over defensive validation
    /// at every call site (see CLAUDE.md's "trust internal code" rule).
    pub fn probe_position(&self, x: u32, y: u32, z: u32) -> Vec3 {
        self.origin + self.spacing * Vec3::new(x as f32, y as f32, z as f32)
    }

    /// World-space position of the probe at FLAT index `flat_index`
    /// (`x + y*dims.x + z*dims.x*dims.y`) — the indexing convention the
    /// relight rotation schedule (`ddgi_probe_relight_start`) and the
    /// atlas layout (`AtlasLayout`) both use, since a probe's "turn" in
    /// the rotation and its own atlas tile slot are both naturally flat,
    /// not 3D-coordinate, concepts.
    pub fn probe_position_flat(&self, flat_index: u32) -> Vec3 {
        let plane = (self.dims.x * self.dims.y).max(1);
        let z = flat_index / plane;
        let rem = flat_index % plane;
        let y = rem / self.dims.x.max(1);
        let x = rem % self.dims.x.max(1);
        self.probe_position(x, y, z)
    }
}

// ---------------------------------------------------------------------------------
// Atlas layout: exact-fit packing (per explicit decision — no fixed max-
// width guess, no wasted texture memory) — `tiles_per_row =
// ceil(sqrt(probe_count))`, atlas pixel dimensions
// `tiles_per_row * tile_size` square. Recreated only when grid config
// (probe_count/tile_size) changes, matching storage-allocation pattern
// (a) from this module's own doc comment (fixed-size, recreated on
// config change — not every frame).
// ---------------------------------------------------------------------------------

/// The atlas texture's own layout: how many probe tiles fit per row, and
/// the resulting square atlas pixel dimensions. Computed once from
/// `probe_count`/`tile_size`, not derived ad-hoc at each call site (a
/// hardcoded/guessed tiles-per-row was a real mistake caught before this
/// ever reached WGSL — see this module's own doc comment history).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AtlasLayout {
    pub tiles_per_row: u32,
    pub tile_size: u32,
    pub atlas_pixels: u32,
}

impl AtlasLayout {
    /// Exact-fit: `tiles_per_row = ceil(sqrt(probe_count))`, at least 1
    /// (a `probe_count` of 0 still needs a well-defined, non-zero-sized
    /// atlas texture — WGPU disallows a zero-size texture, matching this
    /// project's own established "empty scene needs a placeholder, not a
    /// zero-size resource" convention already used for `ObjectGpu`/
    /// `BvhNodeGpu`'s own empty-scene fallback records).
    pub fn exact_fit(probe_count: u32, tile_size: u32) -> Self {
        let tiles_per_row = (probe_count.max(1) as f32).sqrt().ceil() as u32;
        let tile_size = tile_size.max(1);
        Self { tiles_per_row, tile_size, atlas_pixels: tiles_per_row * tile_size }
    }

    /// Pixel-space `(x, y)` origin (the tile's own top-left texel) of
    /// probe `flat_index`'s tile within the atlas — row-major packing,
    /// matching this struct's own `tiles_per_row` field.
    pub fn tile_origin(&self, flat_index: u32) -> (u32, u32) {
        let row = flat_index / self.tiles_per_row.max(1);
        let col = flat_index % self.tiles_per_row.max(1);
        (col * self.tile_size, row * self.tile_size)
    }
}

/// Minimum probe count along any axis — even a degenerate (zero-size or
/// negative-extent) input `bounds` still gets a usable 2-probe-per-axis
/// grid (the minimum needed for trilinear interpolation to mean anything
/// at all: 1 probe has no "between" to interpolate across). Prevents the
/// zero-probe-grid edge case from ever reaching a caller.
const MIN_PROBES_PER_AXIS: u32 = 2;

/// Builds a `ProbeGrid` covering `bounds` (typically the scene's root BVH
/// AABB — see `PersistentBvh`/`bvh.rs`) with `vertical_layers` probes
/// stacked along Y regardless of `spacing.y` (this renderer's own test
/// scenes have real height variance — `RoundedBox` through tall
/// `--stress N` cubes — so a single flat horizontal layer would give
/// every shaded point above/below it the same interpolated irradiance
/// regardless of true 3D position, defeating much of DDGI's own
/// purpose). X/Z probe counts are derived from `bounds`' own extent
/// divided by `spacing.x`/`spacing.z`, rounded up so the grid always
/// covers the FULL input bounds (never under-covers, matching this
/// project's own "checked, not assumed" scale-safety convention already
/// established for `INDIRECT_MAX_T` against the `--stress N` grid
/// pitch), then clamped to `MIN_PROBES_PER_AXIS` so a degenerate
/// (zero-size) `bounds` still produces a well-defined, interpolatable
/// grid rather than zero probes.
///
/// **Probes sit at the CENTER of each grid cell, not at `bounds`' own
/// corners/edges** — a real bug found and fixed via `examples/gi_room.rs`
/// (a fully sealed room): the previous version pinned `origin` to
/// `bounds.min` exactly and forced the extreme layer on every axis to
/// land exactly at `bounds.max`, which is ALWAYS exactly on (or inside)
/// some object's own surface, since the root AABB is by definition the
/// union of every object's bounds. For an enclosed scene this means the
/// outermost probe layer is embedded in that boundary geometry itself;
/// a `probe_ray` fired outward from inside solid geometry has nothing
/// to hit and (at the time this bug was found) fell through to a bright
/// mocked-sky gradient — `probe_ray` no longer has that fallback at all
/// (a miss now reports exactly black, see its own doc comment), but the
/// probe-centering fix below is kept regardless, since a probe embedded
/// in solid geometry is still wrong even with a harmless miss value:
/// its short "escape" rays would still mostly re-hit their own
/// enclosing geometry at point-blank range instead of sampling the
/// room's real nearby surfaces. What the old bright-sky version of this
/// bug did next: that gradient got baked into the probe's stored
/// irradiance and — critically — any shaded point near the true scene boundary
/// (e.g. a room's own ceiling) ALWAYS interpolates against that exact
/// corrupted extreme layer, since `probe_grid_cell` clamps its
/// interpolation window to `[0, dims-2]`, which always includes index
/// `dims-1` as one of the two layers nearest the boundary. Insetting
/// every axis by half a cell (`spacing/2`) moves both extreme layers
/// off `bounds`' own exact edge, onto/inside whatever's actually there
/// instead — for a sealed room, `PROBE_RAY_BIAS`'s own outward nudge
/// then re-detects that same nearby surface at short range rather than
/// escaping into open space beyond it. This was invisible in every
/// scene this project tested DDGI against before `gi_room.rs` (open
/// `--stress N` grids have no enclosing shell for a probe to embed
/// itself in) — not a new problem, just a newly-exposed one.
pub fn probe_grid_from_bounds(bounds: Aabb, spacing: Vec3, vertical_layers: u32) -> ProbeGrid {
    let extent = (bounds.max - bounds.min).abs();
    let axis_count = |extent_axis: f32, spacing_axis: f32| -> u32 {
        let spacing_axis = spacing_axis.max(1e-4);
        (extent_axis / spacing_axis).ceil().max(MIN_PROBES_PER_AXIS as f32) as u32
    };
    let dims_x = axis_count(extent.x, spacing.x);
    let dims_z = axis_count(extent.z, spacing.z);
    let dims_y = vertical_layers.max(MIN_PROBES_PER_AXIS);

    // Vertical spacing derived from the input bounds' own Y extent (not
    // the caller's spacing.y) so `vertical_layers` probes always exactly
    // span the bounds top-to-bottom, regardless of how tall the scene
    // actually is — an `--stress N` scene's real object-height extent,
    // not a hardcoded guess. Cell-center layout: `dims_y` cells of size
    // `spacing_y` tile the full extent (not `dims_y - 1` gaps between
    // edge-pinned probes), so `spacing_y = extent.y / dims_y`.
    let spacing_y = extent.y.max(1e-4) / dims_y as f32;
    let spacing = Vec3::new(spacing.x.max(1e-4), spacing_y, spacing.z.max(1e-4));

    // Inset the origin by half a cell on every axis so probe 0's own
    // position sits at the center of the FIRST cell (bounds.min +
    // spacing/2), not exactly on bounds.min itself — see this
    // function's own doc comment for why.
    let origin = bounds.min + spacing * 0.5;

    ProbeGrid { origin, spacing, dims: UVec3::new(dims_x, dims_y, dims_z) }
}

// ---------------------------------------------------------------------------------
// Octahedral encoding — WGSL mirror of `octahedral_encode` already
// vendored (from `bevy_pbr::render::utils`) in
// `assets/shaders/sphere_prepass_blit.wgsl`, ported to Rust here as this
// project's own CPU-testable reference (that file belongs to the
// unrelated, frozen `prepass_probe` spike — `src/hybrid` gets its own
// copy per this project's "read as pattern reference, not reused code"
// convention). `octahedral_decode` is new: no decode function exists
// anywhere in this codebase yet, needed here for reading a probe's
// stored irradiance back out at shading time.
// ---------------------------------------------------------------------------------

/// Direction (unit `Vec3`) -> unit-square UV (`[0,1]^2`) — verbatim port
/// of `sphere_prepass_blit.wgsl`'s own `octahedral_encode`.
pub fn octahedral_encode(v: Vec3) -> Vec2 {
    let n = v / (v.x.abs() + v.y.abs() + v.z.abs());
    let sign = |x: f32| if x > 0.0 { 1.0 } else { -1.0 };
    let octahedral_wrap = Vec2::new((1.0 - n.y.abs()) * sign(n.x), (1.0 - n.x.abs()) * sign(n.y));
    let n_xy = if n.z >= 0.0 { Vec2::new(n.x, n.y) } else { octahedral_wrap };
    n_xy * 0.5 + Vec2::splat(0.5)
}

/// Unit-square UV -> direction (unit `Vec3`) — the inverse of
/// `octahedral_encode`, standard octahedral decode formula (no prior
/// implementation in this codebase to port from; written fresh here and
/// proven via round-trip tests against `octahedral_encode`, both
/// directions, before any WGSL trusts it).
pub fn octahedral_decode(uv: Vec2) -> Vec3 {
    let f = uv * 2.0 - Vec2::splat(1.0);
    let mut n = Vec3::new(f.x, f.y, 1.0 - f.x.abs() - f.y.abs());
    let t = (-n.z).max(0.0);
    let sign = |x: f32| if x >= 0.0 { 1.0 } else { -1.0 };
    n.x -= t * sign(n.x);
    n.y -= t * sign(n.y);
    n.normalize()
}

// ---------------------------------------------------------------------------------
// Probe-to-texel mapping: which texel within a probe's `tile_size x
// tile_size` atlas tile a given ray direction maps to (used when the
// relight pass WRITES a probe's traced irradiance), and the inverse —
// a texel's own direction (used when the relight pass DECIDES which ray
// to fire for that texel, and when shading-time sampling reads a
// probe's stored irradiance back out for a given surface normal).
// ---------------------------------------------------------------------------------

/// Which texel `(x, y)` within a `tile_size x tile_size` probe tile a
/// `direction` maps to — octahedral-encodes the direction, then scales
/// the resulting `[0,1]^2` UV to texel coordinates, clamped to the valid
/// range (a UV of exactly `1.0` would otherwise index one texel past the
/// tile's own last valid column/row).
pub fn direction_to_texel(direction: Vec3, tile_size: u32) -> (u32, u32) {
    let uv = octahedral_encode(direction);
    let max_index = tile_size.saturating_sub(1);
    let x = ((uv.x * tile_size as f32) as u32).min(max_index);
    let y = ((uv.y * tile_size as f32) as u32).min(max_index);
    (x, y)
}

/// The direction a given texel `(x, y)` within a `tile_size x tile_size`
/// probe tile represents — samples at the texel's own CENTER (`+0.5`),
/// the standard texel-to-UV convention (matches `hybrid_trace.wgsl`'s
/// own `generate_primary_ray`'s `pixel + 0.5` convention for the same
/// reason: a texel's "position" is its center, not its corner).
pub fn texel_to_direction(x: u32, y: u32, tile_size: u32) -> Vec3 {
    let uv = Vec2::new((x as f32 + 0.5) / tile_size as f32, (y as f32 + 0.5) / tile_size as f32);
    octahedral_decode(uv)
}

// ---------------------------------------------------------------------------------
// Relight-subset rotation — generalizes cpu_ref::indirect_sample_start's
// existing per-PIXEL rotation formula to per-PROBE rotation: which
// `probes_per_frame`-sized subset of a grid's `total_probes` gets relit
// this frame, cycling through the whole grid over
// `ceil(total_probes / probes_per_frame)` frames. Same deterministic
// round-robin, not randomized, for the same reason
// `indirect_sample_start`'s own doc comment gives: a testable, bounded-
// coverage-guaranteed schedule beats a stochastic one.
// ---------------------------------------------------------------------------------

/// Index of the first probe (by flat grid index, `x + y*dims.x +
/// z*dims.x*dims.y`) in this frame's relit subset — mirrors
/// `cpu_ref::indirect_sample_start`'s exact formula shape, generalized
/// from a compile-time-fixed `INDIRECT_SAMPLE_COUNT` to a runtime
/// `total_probes` (the probe grid's size varies per scene, unlike the
/// fixed 5-direction hemisphere sample set).
pub fn ddgi_probe_relight_start(probes_per_frame: u32, total_probes: u32, frame_index: u32) -> usize {
    let total_probes = total_probes.max(1);
    let probes_per_frame = probes_per_frame.clamp(1, total_probes);
    ((frame_index.wrapping_mul(probes_per_frame)) % total_probes) as usize
}

// ---------------------------------------------------------------------------------
// Probe relighting: fires one ray from a probe's world position, reusing
// `cpu_ref::trace` directly (grounding: `trace`/`Hit`/`TraceObject` carry
// no per-pixel-specific fields, so a probe-sourced ray needs no variant —
// see this module's own doc comment). Reports REAL surface radiance at
// the hit point (direct lighting + emissive, via `cpu_ref::shade` with
// `indirect_enabled: false`), NOT Stage B's `indirect_ray`'s flat-albedo-
// times-falloff-times-0.5 shortcut — a probe's whole point is to capture
// genuine bounced light over a persistent grid, not a short-range
// proximity hint. `indirect_enabled: false` deliberately caps this at
// ONE order of indirection (a probe ray sees direct light on the surface
// it hits, not that surface's own indirect contribution recursing into
// yet another probe lookup) — chasing further bounces is out of this
// first pass's scope (see this module's own doc comment, decision 3's
// sibling: rotating-subset relighting already bounds per-frame ray cost,
// and unbounded recursion would defeat that bound entirely).
// ---------------------------------------------------------------------------------

/// One probe ray: traces from `origin` (a probe's world position, offset
/// along `direction` by a small bias handled by the caller — mirrors
/// `indirect_diffuse`'s own `p + n * bias` convention, except a probe has
/// no surface normal of its own to bias along, so callers bias by a
/// fixed small margin along `direction` itself instead) out to `max_t`.
/// On a hit, evaluates real direct-lit radiance at the hit point via
/// `shade` (indirect disabled inside `shade` itself — see this section's
/// own doc comment), using the hit `TraceObject`'s own `material` field
/// (no separate materials slice needed — `TraceObject` already carries
/// it); on a miss, reports exactly black — matches the "a miss
/// contributes nothing, not a mocked sky gradient" convention established
/// by `conetrace_ref::cone_trace_ray` (this codebase's own sealed-room
/// light-leak fix: trusting a miss as real sky let light leak through
/// gaps that should have stayed dark) rather than this file's original
/// `sky_color(direction)` fallback, which predates that fix and shared
/// its same bug. Passed `None, None` for reflection/transmission (a
/// probe ray never fires a nested specular/transmissive bounce).
///
/// **`indirect_at_hit`: the infinite-bounce link.** A probe ray's hit
/// point gets its OWN indirect-diffuse term added on top of direct
/// light, exactly the way any camera-visible surface already does in
/// `hybrid_trace.wgsl::shade` (`diffuse_color * ddgi_sample_probe_grid(...)`)
/// — sampling the SAME probe grid this ray's own result will (eventually)
/// be blended into. This is what makes bounce light actually propagate
/// probe-to-probe over multiple frames instead of capping at one order of
/// indirection forever: probe A's relit value includes a sample of probe
/// B's last-known irradiance at A's hit point, so by next frame's
/// temporal blend, some of B's light has diffused into A, and so on
/// outward across the whole grid — RTXGI's own published "infinite
/// bounce" trick, achieved with no new storage and no extra ray, just
/// reusing the existing grid sample already available at relight time.
/// `indirect_at_hit(hit_point, hit_normal) -> Vec3` returns raw irradiance
/// (NOT yet multiplied by the hit surface's own diffuse albedo — that
/// happens here, matching `shade`'s own `diffuse_color * irradiance`
/// convention exactly) so callers can pass `sample_probe_grid` (or, in
/// tests, a trivial closure) without duplicating the albedo multiply.
/// Passing `|_, _| Vec3::ZERO` recovers the exact prior single-bounce
/// behavior bit-for-bit (a zero-irradiance sample adds exactly nothing).
///
/// Returns `(radiance, hit_distance)` — `hit_distance` feeds the
/// Chebyshev depth-visibility test (see this module's own doc comment,
/// `sample_probe_grid`'s new distance-aware occlusion weighting) rather
/// than being discarded like before Chebyshev visibility existed. On a
/// miss, distance is reported as `max_t` (RTXGI's own convention: "found
/// nothing all the way out to max_t", the same value a probe that
/// genuinely saw open space that far would report — this is NOT a
/// special case, it naturally falls out of `trace`'s own `t_max`
/// parameter being the search bound), matching a real ray's own
/// farthest possible finding rather than an arbitrary sentinel.
pub fn probe_ray(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    origin: Vec3,
    direction: Vec3,
    max_t: f32,
    indirect_at_hit: impl Fn(Vec3, Vec3) -> Vec3,
) -> (Vec3, f32) {
    match trace(bvh, objects, origin, direction, max_t) {
        Some(hit) => {
            let Some(object) = objects.iter().find(|o| o.entity == hit.entity) else {
                return (Vec3::ZERO, hit.t);
            };
            let view_dir = -direction;
            let hit_point = origin + hit.t * direction;
            let result = shade(
                &object.material,
                hit_point,
                hit.world_normal,
                view_dir,
                lights,
                bvh,
                objects,
                Some(hit.entity),
                hit.t,
                None,
                None,
            );
            let diffuse_color = object.material.base_color * (1.0 - object.material.metallic.clamp(0.0, 1.0));
            let indirect = diffuse_color * indirect_at_hit(hit_point, hit.world_normal.normalize());
            (result.direct_and_emissive + indirect, hit.t)
        }
        None => (Vec3::ZERO, max_t),
    }
}

/// Small fixed offset a probe ray's origin is biased from the probe's
/// own exact grid position, along the ray's own direction — mirrors
/// `indirect_diffuse`'s own `bias = 0.01` (`cpu_ref.rs`), same rationale:
/// a probe sitting exactly on/very near a surface (the ground-height-
/// bias mitigation still leaves probes close to the ground, not
/// arbitrarily far from it) needs a small nudge to avoid immediately
/// re-detecting that same nearby surface at `t~=0`.
pub const PROBE_RAY_BIAS: f32 = 0.01;

/// Blended relight result for one probe texel: irradiance plus the two
/// distance moments (mean, mean-of-squares) the Chebyshev depth-
/// visibility test in `sample_probe_grid` needs — see this module's own
/// doc comment ("Chebyshev depth-aware visibility" section below) for
/// why a single occlusion ray from the SHADED POINT'S side isn't enough
/// on its own: it only catches "something sits directly between the
/// shaded point and the probe," not "this probe's own stored irradiance
/// is itself contaminated because its relight rays skimmed through/near
/// thin geometry." Storing what distance the probe's OWN rays actually
/// found in each direction lets shading time ask "does this probe's own
/// view match reality from where I'm standing," a strictly stronger
/// check.
pub struct RelightedTexel {
    pub irradiance: Vec3,
    pub mean_distance: f32,
    pub mean_distance_squared: f32,
    pub history_length: f32,
}

/// Relights ONE texel of ONE probe's atlas tile: fires `probe_ray` along
/// that texel's own direction (`texel_to_direction`), then blends the
/// fresh irradiance AND the fresh distance/distance^2 with `history` via
/// `temporal_ref::temporal_blend`'s own clamped-EMA formula, VERBATIM
/// reused (not a new blend formula, called twice — once per `Vec3`-
/// shaped quantity, since distance/distance^2 pack into a `Vec3`'s
/// unused third component rather than justifying a whole new 2-component
/// blend function) — applied here at probe-texel granularity instead of
/// per-pixel granularity, the same "reuse the shape, not just the idea"
/// precedent `ddgi_probe_relight_start` already sets for the rotation
/// schedule. Both blends share the SAME `history_length`/`new_length` —
/// irradiance and its own distance stats are relit by the exact same ray
/// on the exact same schedule, so there is only one real "how many
/// samples has this texel accumulated" answer, not two independently
/// drifting ones.
///
/// `indirect_at_hit` is forwarded straight through to `probe_ray` — see
/// that function's own doc comment for why this is the whole infinite-
/// bounce mechanism (a probe's relit value samples the grid's own
/// existing irradiance at its hit point, so light keeps diffusing outward
/// probe-to-probe across frames via the temporal blend below, not just
/// one extra hop).
#[allow(clippy::too_many_arguments)]
pub fn relight_probe_texel(
    bvh: &Bvh,
    objects: &[TraceObject],
    lights: &[Light],
    probe_position: Vec3,
    texel: (u32, u32),
    tile_size: u32,
    max_t: f32,
    history_irradiance: Vec3,
    history_mean_distance: f32,
    history_mean_distance_squared: f32,
    history_length: f32,
    max_history_length: f32,
    indirect_at_hit: impl Fn(Vec3, Vec3) -> Vec3,
) -> RelightedTexel {
    let direction = texel_to_direction(texel.0, texel.1, tile_size);
    let (current_irradiance, current_distance) =
        probe_ray(bvh, objects, lights, probe_position + direction * PROBE_RAY_BIAS, direction, max_t, indirect_at_hit);
    let (blended_irradiance, new_length) =
        crate::hybrid::temporal_ref::temporal_blend(current_irradiance, history_irradiance, history_length, max_history_length);
    // Packs (distance, distance^2, 0.0) into temporal_blend's own Vec3
    // shape purely to reuse its exact clamped-EMA formula without a new
    // 2-component variant — the unused third component is discarded
    // below, not a meaningful third quantity.
    let current_moments = Vec3::new(current_distance, current_distance * current_distance, 0.0);
    let history_moments = Vec3::new(history_mean_distance, history_mean_distance_squared, 0.0);
    let (blended_moments, moments_length) =
        crate::hybrid::temporal_ref::temporal_blend(current_moments, history_moments, history_length, max_history_length);
    debug_assert_eq!(new_length, moments_length, "irradiance and distance-moment blends must share one history_length");
    RelightedTexel {
        irradiance: blended_irradiance,
        mean_distance: blended_moments.x,
        mean_distance_squared: blended_moments.y,
        history_length: new_length,
    }
}

// ---------------------------------------------------------------------------------
// Shading-time sampling: trilinear interpolation over the 8 probes
// surrounding a shaded point, each gated by a cheap single-ray
// visibility check (see this module's own doc comment, decision 2: no
// Chebyshev visibility/depth test in this first pass — a full occlusion
// ray per probe per shaded point is the accepted cheaper substitute,
// catching the worst "probe on the wrong side of a wall" leak case
// without the real visibility-texture infrastructure).
// ---------------------------------------------------------------------------------

/// Which grid cell (the LOWER corner's integer coords) a world point
/// falls in, plus the fractional position within that cell (`[0,1]^3`,
/// used as trilinear weights) — the first step of both sampling and (if
/// ever needed) locating a point's nearest probes for debugging.
/// Coordinates are CLAMPED to `[0, dims-2]` so a point outside the grid's
/// own bounds still resolves to the nearest valid cell (extrapolating
/// from the grid's edge) rather than indexing out of range.
pub fn probe_grid_cell(grid: &ProbeGrid, world_pos: Vec3) -> (UVec3, Vec3) {
    let local = (world_pos - grid.origin) / grid.spacing;
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

// ---------------------------------------------------------------------------------
// Hemisphere convolution: a real gap found by testing DDGI against
// examples/gi_room.rs (a fully closed room) — a probe's own stored
// irradiance is a RAW single-ray sample per octahedral texel, never
// blended across neighboring directions. Reading a probe for a shaded
// point's own normal returns whatever that ONE ray happened to hit in
// that ONE exact direction, not an averaged "how lit is this area"
// answer — real DDGI/RTXGI probes store cosine-CONVOLVED irradiance
// specifically so any read direction gives a plausible diffuse-
// hemisphere result. Without this, surfaces facing away from any
// direct light read pure black (a probe sitting in a well-lit spot
// still reads dark for a normal direction its own single ray happened
// to miss), the overall indirect result reads too dim (no hemisphere
// averaging = no ambient fill, every read is one noisy point sample),
// and the result is noisier frame-to-frame (single-sample variance).
//
// Fixed by convolving at READ time (gather), not write time (scatter):
// each atlas texel is written by exactly one GPU invocation in
// hybrid_ddgi_relight.wgsl, so scattering a traced sample into
// neighboring texels at write time would be a real cross-invocation
// write race; gathering N samples around the read direction at shading
// time touches only the already-safe, already-bounds-clamped
// `ddgi_probe_irradiance` read path, called N times instead of once.
// ---------------------------------------------------------------------------------

/// Duff et al.'s branchless orthonormal basis construction, `n` as the z
/// axis — numerically stable for every `n` including axis-aligned ones
/// (unlike a naive `cross(n, Vec3::Y)`, which degenerates when `n` is
/// itself close to `(0,1,0)`). A DDGI-owned re-derivation of the exact
/// same formula Stage B's own (now-removed) `tangent_basis` used —
/// recovered from git history (commit `19a4cbb`, the commit that
/// removed it) rather than re-derived from scratch, since it's a known-
/// correct, previously-tested formula. Named `ddgi_tangent_basis` (not
/// bare `tangent_basis`) to make clear this is DDGI's own copy, not a
/// surviving Stage B remnant — mirrors this project's established per-
/// module self-containment convention (see e.g.
/// `hybrid_ddgi_relight.wgsl`'s own duplicated trace/march/shade logic).
pub fn ddgi_tangent_basis(n: Vec3) -> Mat3 {
    let s = if n.z >= 0.0 { 1.0 } else { -1.0 };
    let a = -1.0 / (s + n.z);
    let b = n.x * n.y * a;
    let t = Vec3::new(1.0 + s * n.x * n.x * a, s * b, -s * n.x);
    let bt = Vec3::new(b, s + n.y * n.y * a, -n.y);
    Mat3::from_cols(t, bt, n)
}

/// 5 fixed cosine-weighted-ish hemisphere directions in TANGENT space
/// (z-up: index 0 is straight up the normal, 1-4 splay toward the rim)
/// — the exact literal values Stage B's own (now-removed)
/// `INDIRECT_SAMPLES` used, recovered from git history (commit
/// `19a4cbb`) rather than re-derived: a known, previously-shipped-and-
/// tested sample set, not a new guess. Still readable today in the
/// frozen `assets/shaders/hybrid_legacy_trace.wgsl`'s own
/// `INDIRECT_SAMPLES` for direct copy-verification.
const HEMISPHERE_SAMPLES: [Vec3; 5] = [
    Vec3::new(0.0, 0.0, 1.0),
    Vec3::new(0.6614, 0.0, 0.75),
    Vec3::new(-0.2044, 0.6285, 0.75),
    Vec3::new(-0.5350, -0.3886, 0.75),
    Vec3::new(0.5350, -0.3886, 0.75),
];

/// Cosine-weighted hemisphere convolution of ONE probe's own stored
/// irradiance, read for a shaded surface with normal `surface_normal`
/// — builds a tangent basis around the normal, transforms
/// `HEMISPHERE_SAMPLES` into world space, calls the caller-supplied
/// `probe_irradiance` lookup once per sample direction (mirrors
/// `sample_probe_grid`'s own `probe_irradiance` closure convention,
/// narrowed to a single already-selected probe/direction rather than a
/// full grid coordinate), and averages. A plain average (not an
/// explicit extra `dot(sample, n)` weighting term): `HEMISPHERE_SAMPLES`
/// already leans cosine-weighted by construction (its own values were
/// hand-picked so more samples cluster near the pole than the rim, per
/// Stage B's own original doc comment) — an additional weighting term
/// on top would double-count that bias, not add real energy
/// conservation, and was confirmed unnecessary by comparing both
/// against `repeated_relighting_of_a_static_scene_converges_toward_the_
/// true_value`-style known-analytic cases before committing to the
/// simpler form.
pub fn cosine_weighted_probe_irradiance(surface_normal: Vec3, probe_irradiance: impl Fn(Vec3) -> Vec3) -> Vec3 {
    let basis = ddgi_tangent_basis(surface_normal);
    let mut acc = Vec3::ZERO;
    for sample in HEMISPHERE_SAMPLES {
        let world_dir = (basis * sample).normalize();
        acc += probe_irradiance(world_dir);
    }
    acc / HEMISPHERE_SAMPLES.len() as f32
}

/// Roughness-aware variant of `cosine_weighted_probe_irradiance`: pulls
/// each of `HEMISPHERE_SAMPLES`' own off-axis directions toward the
/// pole `(0,0,1)` (straight along `surface_normal`) by `1.0 - roughness`
/// before transforming into world space, so a fully rough (`roughness =
/// 1.0`) surface samples the SAME full spread the original function
/// always used, while a fully smooth/glossy (`roughness = 0.0`) surface
/// converges all 5 samples onto the single mirror-reflection-like
/// direction — mirroring how a real material's own BRDF lobe narrows as
/// it gets glossier (a mirror gathers light from essentially one
/// direction; a matte diffuse surface gathers from its whole
/// hemisphere). This directly answers the "GI bounds disperse based on
/// reflection level" request: a glossy floor's own indirect response
/// should look tighter/more directional than a rough wall's, not
/// identically diffuse regardless of material.
///
/// Re-normalizing each pulled-in sample (not just the accumulated
/// result) keeps every individual probe lookup direction a genuine unit
/// vector — `ddgi_probe_irradiance`'s own octahedral encoding assumes
/// unit-length input, so skipping this per-sample step would silently
/// undersample the atlas at low roughness instead of cleanly
/// converging.
pub fn cosine_weighted_probe_irradiance_roughness_aware(surface_normal: Vec3, roughness: f32, probe_irradiance: impl Fn(Vec3) -> Vec3) -> Vec3 {
    let spread = roughness.clamp(0.0, 1.0);
    let pole = Vec3::new(0.0, 0.0, 1.0);
    let basis = ddgi_tangent_basis(surface_normal);
    let mut acc = Vec3::ZERO;
    for sample in HEMISPHERE_SAMPLES {
        let narrowed = sample.lerp(pole, 1.0 - spread).normalize();
        let world_dir = (basis * narrowed).normalize();
        acc += probe_irradiance(world_dir);
    }
    acc / HEMISPHERE_SAMPLES.len() as f32
}

/// Same hemisphere convolution as `cosine_weighted_probe_irradiance`,
/// over a `(f32, f32)` pair instead of a `Vec3` — used for the Chebyshev
/// distance moments (`sample_probe_grid`'s own new visibility test)
/// rather than irradiance. A second function, not a generic one, per
/// this project's own established convention of small formulas
/// duplicated across call sites rather than abstracted early.
fn cosine_weighted_probe_irradiance_scalar_pair(surface_normal: Vec3, probe_moments: impl Fn(Vec3) -> (f32, f32)) -> (f32, f32) {
    let basis = ddgi_tangent_basis(surface_normal);
    let mut acc = (0.0f32, 0.0f32);
    for sample in HEMISPHERE_SAMPLES {
        let world_dir = (basis * sample).normalize();
        let (mean, mean_sq) = probe_moments(world_dir);
        acc.0 += mean;
        acc.1 += mean_sq;
    }
    let n = HEMISPHERE_SAMPLES.len() as f32;
    (acc.0 / n, acc.1 / n)
}

/// The `weight_sum` value at which `sample_probe_grid`'s own occlusion
/// fallback ramp fully saturates to the pure trilinear-weighted-average
/// path — see that function's own doc comment for the full flicker-fix
/// rationale. Trilinear weights sum to at most `1.0` (a shaded point
/// exactly on a probe), so `0.2` only smooths the "few probes barely
/// unoccluded" regime, not the common well-covered case.
const FALLBACK_RAMP_WEIGHT: f32 = 0.2;

/// Samples the probe grid at `world_pos` for a shaded surface with
/// normal `surface_normal` — the 8 probes surrounding `world_pos`'s own
/// grid cell, each visibility-gated (a probe whose center is occluded
/// from `world_pos` contributes ZERO, not a reduced weight — see this
/// section's own doc comment) and read via `probe_irradiance(grid_coords,
/// direction) -> Vec3` (a caller-supplied lookup — mirrors
/// `blur_indirect_at`'s own `sample_at` closure convention — so this same
/// logic maps directly onto a real WGSL atlas-texture read later, no
/// rewrite needed). Each of the 8 probes' own irradiance now comes from
/// `cosine_weighted_probe_irradiance`'s 5-sample hemisphere gather
/// (see this section's own header comment for why), not a single raw
/// direct-normal texel read. Results are trilinearly blended by the
/// fractional position within the cell. If EVERY one of the 8 probes is
/// occluded (all weights zero), falls back TOWARD the unweighted average
/// of their raw irradiance rather than returning black — a shaded point
/// fully enclosed by geometry on all 8 corners is a genuine limitation
/// of skipping real per-probe visibility (see this module's own doc
/// comment's decision 2), not something this fallback claims to solve;
/// it only avoids a harsher, more visually-wrong all-black result in
/// that specific corner case.
///
/// **The fallback is a smooth ramp (`FALLBACK_RAMP_WEIGHT`), not a hard
/// branch on `weight_sum > 0`** — a real, found-by-direct-visual-
/// inspection flicker bug: a first version switched formulas outright
/// at `weight_sum == 0` (pure trilinear-weighted average of unoccluded
/// probes on one side, pure unweighted average of ALL 8 raw probes —
/// including ones the visibility gate had just excluded — on the
/// other). Near a wall/corner, where several of the 8 surrounding
/// probes sit right at the occlusion boundary, a shaded point's own
/// occlusion state can flip probe-by-probe frame to frame at grazing
/// angles, and `weight_sum` crossing exactly zero flipped the ENTIRE
/// result between two structurally different formulas — a hard visual
/// pop, not a gradual change, even though the underlying geometry only
/// changed by one probe's occlusion state. `smoothstep(0,
/// FALLBACK_RAMP_WEIGHT, weight_sum)` blends continuously between the
/// two formulas instead: at `weight_sum == 0` this reduces to exactly
/// the old fallback (identical result, not just similar), and once
/// `weight_sum` clears `FALLBACK_RAMP_WEIGHT` it reduces to exactly the
/// old weighted-average path — the two endpoints are unchanged, only
/// the transition between them is now continuous. `FALLBACK_RAMP_WEIGHT
/// = 0.2`: trilinear weights sum to at most `1.0` (a shaded point
/// exactly on a probe), so `0.2` is a genuinely small slice of that
/// range — the ramp only smooths the specific "few probes barely
/// unoccluded" regime this bug lives in, not the common well-covered
/// case (confirmed by `an_occluded_probe_contributes_zero_not_a_
/// reduced_weight`'s own real geometry: 4 of 8 probes visible there
/// gives `weight_sum ≈ 0.66`, well clear of the ramp, so that test's
/// own "occluded probe's color must not leak at all" guarantee is
/// completely unaffected).
///
/// `origin_entity` excludes the shaded object itself from its own
/// occlusion query — mirrors `trace_shadow`'s own `origin_entity`
/// exclusion exactly. Without this, a rotating object's own occlusion
/// ray toward a probe can graze back across its own geometry (a face
/// near a corner/edge, or a probe direction nearly edge-on to the
/// surface) at specific rotation angles, self-intersecting and zeroing
/// that probe's contribution for exactly the frames where the rotation
/// puts the ray in the self-clipping regime — a real bug found by
/// direct visual inspection: one face of a spinning `--stress N` cube
/// intermittently flickered dark at specific rotation angles, then
/// recovered, reproducible via deterministic frame-stepped capture (not
/// a threading artifact). `PROBE_RAY_BIAS`'s small fixed offset alone
/// isn't enough margin to prevent this at all rotation angles, so the
/// exclusion (not a larger bias) is the correct fix, matching every
/// other surface-launched ray in this renderer.
// Floor under Chebyshev's own variance term — a probe whose relit rays
// all reported nearly IDENTICAL distances (a flat wall filling its whole
// hemisphere) has variance approaching exactly zero, which would make
// the weight formula divide by a near-zero denominator and swing
// wildly between ~0 and ~1 for a tiny change in distance — the same
// class of "near-degenerate denominator" instability
// `REFLECT_ROUGHNESS_GATE`'s own clamping and `refract_ref::
// refract_trace_ray`'s own `safe_rho` clamp already guard against
// elsewhere in this codebase. RTXGI's own published implementation uses
// this same small-epsilon floor for the identical reason.
const CHEBYSHEV_VARIANCE_FLOOR: f32 = 1e-3;

/// Chebyshev depth-visibility weight in `[0, 1]`: how plausible it is
/// that a probe with mean distance `mean`/mean-squared-distance `mean_sq`
/// (over ALL directions its own relit rays have sampled near this one)
/// can actually see a point `dist` away. `dist <= mean` (the probe's own
/// rays typically travel at least as far as the shaded point) is treated
/// as fully visible (`1.0`) — Chebyshev's one-sided bound only usefully
/// constrains the "shaded point is FARTHER than what the probe usually
/// sees" case (the probe's own rays are being stopped short by
/// something, so seeing further than that is the suspicious case worth
/// down-weighting), matching RTXGI's own published formulation.
fn chebyshev_visibility_weight(dist: f32, mean: f32, mean_sq: f32) -> f32 {
    if dist <= mean {
        return 1.0;
    }
    let variance = (mean_sq - mean * mean).max(CHEBYSHEV_VARIANCE_FLOOR);
    let delta = dist - mean;
    variance / (variance + delta * delta)
}

/// `probe_distance_moments` returns `(mean_distance, mean_distance_squared)`
/// for the SAME cosine-weighted hemisphere convolution
/// `cosine_weighted_probe_irradiance` already gathers for irradiance —
/// see this module's own "Hemisphere convolution" section header for why
/// a probe's raw per-texel value needs convolving at read time in the
/// first place (a probe's own single stored ray per exact direction is
/// too noisy/incomplete on its own); the exact same argument applies to
/// distance, not just irradiance, since both are written by the same
/// one-ray-per-texel relight pass.
#[allow(clippy::too_many_arguments)]
pub fn sample_probe_grid(
    grid: &ProbeGrid,
    bvh: &Bvh,
    objects: &[TraceObject],
    world_pos: Vec3,
    surface_normal: Vec3,
    roughness: f32,
    origin_entity: Option<Entity>,
    probe_irradiance: impl Fn(UVec3, Vec3) -> Vec3,
    probe_distance_moments: impl Fn(UVec3, Vec3) -> (f32, f32),
) -> Vec3 {
    let (cell, frac) = probe_grid_cell(grid, world_pos);
    let mut acc = Vec3::ZERO;
    let mut weight_sum = 0.0f32;
    let mut raw_sum = Vec3::ZERO;
    let mut raw_count = 0u32;
    for dz in 0..2u32 {
        for dy in 0..2u32 {
            for dx in 0..2u32 {
                let coords = UVec3::new(cell.x + dx, cell.y + dy, cell.z + dz);
                let probe_pos = grid.probe_position(coords.x, coords.y, coords.z);
                let wx = if dx == 0 { 1.0 - frac.x } else { frac.x };
                let wy = if dy == 0 { 1.0 - frac.y } else { frac.y };
                let wz = if dz == 0 { 1.0 - frac.z } else { frac.z };
                let trilinear_weight = wx * wy * wz;

                let irradiance =
                    cosine_weighted_probe_irradiance_roughness_aware(surface_normal, roughness, |dir| probe_irradiance(coords, dir));
                raw_sum += irradiance;
                raw_count += 1;

                let to_probe = probe_pos - world_pos;
                let dist = to_probe.length();
                // Two independent visibility signals, combined
                // multiplicatively: the existing binary occlusion ray
                // (catches "something sits directly between the shaded
                // point and the probe," e.g. the probe is on the far
                // side of a wall relative to this exact point) AND the
                // new Chebyshev depth test (catches "this probe's own
                // stored irradiance is itself unreliable in roughly this
                // direction," e.g. its relight rays skimmed through/near
                // a thin wall from the probe's OWN vantage point,
                // regardless of whether THIS shaded point's own occlusion
                // ray happens to be clear). Neither alone catches both
                // failure modes; RTXGI's own published implementation
                // likewise combines a hard shadow-map-style occlusion
                // test with the Chebyshev soft weight rather than relying
                // on either in isolation.
                let hard_occluded = if dist > 1e-4 {
                    any_hit(bvh, objects, world_pos + surface_normal * PROBE_RAY_BIAS, to_probe / dist, dist - PROBE_RAY_BIAS, origin_entity)
                } else {
                    false
                };
                let chebyshev_weight = if dist > 1e-4 {
                    let (mean, mean_sq) = cosine_weighted_probe_irradiance_scalar_pair(surface_normal, |dir| probe_distance_moments(coords, dir));
                    chebyshev_visibility_weight(dist, mean, mean_sq)
                } else {
                    1.0
                };
                let combined_weight = if hard_occluded { 0.0 } else { chebyshev_weight };
                if combined_weight > 0.0 {
                    acc += irradiance * trilinear_weight * combined_weight;
                    weight_sum += trilinear_weight * combined_weight;
                }
            }
        }
    }
    if raw_count == 0 {
        return Vec3::ZERO;
    }
    let fallback = raw_sum / raw_count as f32;
    // smoothstep(0, FALLBACK_RAMP_WEIGHT, weight_sum): 0 at weight_sum
    // == 0 (pure fallback, identical to the old hard branch's own
    // result there), 1 once weight_sum clears FALLBACK_RAMP_WEIGHT
    // (pure weighted average, identical to the old hard branch's other
    // side) — see this function's own doc comment for why the smooth
    // transition in between is the actual fix, not just a stylistic
    // change.
    let t = (weight_sum / FALLBACK_RAMP_WEIGHT).clamp(0.0, 1.0);
    let mix = t * t * (3.0 - 2.0 * t);
    let weighted = acc / weight_sum.max(1e-6);
    fallback.lerp(weighted, mix)
}

#[cfg(test)]
mod tests {
    use bevy::math::Quat;
    use bevy::prelude::{Entity, World};

    use super::*;
    use crate::hybrid::cpu_ref::LightKind;
    use crate::hybrid::material::Material;
    use crate::hybrid::scene::HybridObject;
    use crate::sdf::components::Shape;

    fn entities(n: usize) -> Vec<Entity> {
        let mut world = World::new();
        (0..n).map(|_| world.spawn_empty().id()).collect()
    }

    /// A local fixture — NOT a reuse of `cpu_ref::tests::ground_and_cube`
    /// (that module's test helpers are private to its own `mod tests`,
    /// unreachable from here) — same shape (a flat ground plate + one
    /// small box above it), rebuilt fresh against this module's own
    /// imports, matching this project's own established "reconstruct,
    /// don't reach across module test boundaries" precedent (see
    /// `cpu_ref.rs`'s own `debug_stress_100_sphere_shadow_ring_profile`'s
    /// doc comment for the identical reasoning applied to
    /// `hybrid_legacy`'s scene fixtures).
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

    /// A `probe_distance_moments` closure reporting "every probe's own
    /// rays always found nothing nearby in every direction" — every
    /// `sample_probe_grid` test written BEFORE Chebyshev visibility
    /// existed exercised only the hard-occlusion-ray path, so this
    /// closure makes `chebyshev_visibility_weight` a permanent no-op
    /// (`dist <= mean` is always true when `mean == f32::MAX`) rather
    /// than rewriting every one of those fixtures' own expectations to
    /// also account for a second, unrelated visibility signal they were
    /// never testing in the first place.
    fn always_visible_moments(_coords: UVec3, _dir: Vec3) -> (f32, f32) {
        (f32::MAX, f32::MAX)
    }

    // -----------------------------------------------------------------
    // probe_ray
    // -----------------------------------------------------------------

    #[test]
    fn probe_ray_returns_black_when_nothing_nearby() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        // Straight up from well above the box: nothing in the way within
        // a short reach. A miss must report exactly black, not a mocked
        // sky gradient — see probe_ray's own doc comment for why.
        let (result, distance) = probe_ray(&bvh, &objects, &lights, Vec3::new(10.0, 5.0, 10.0), Vec3::Y, 3.0, |_, _| Vec3::ZERO);
        assert_eq!(result, Vec3::ZERO, "a probe ray miss must report exactly black, got {result:?}");
        assert_eq!(distance, 3.0, "a probe ray miss must report max_t as its own distance, got {distance}");
    }

    #[test]
    fn probe_ray_picks_up_real_lit_color_on_a_hit() {
        let (e, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        // Fired straight down from just above the box's own top face —
        // should hit the box and report ITS lit color, not the ground's.
        let origin = Vec3::new(0.0, 2.0, 0.0);
        let (result, distance) = probe_ray(&bvh, &objects, &lights, origin, -Vec3::Y, 5.0, |_, _| Vec3::ZERO);
        assert!(result.length() > 1e-4, "expected a real non-black lit color on a direct-sun-lit hit, got {result:?}");
        assert!(distance < 5.0, "a real hit's own distance must be less than max_t, got {distance}");
        let box_material_dir = Vec3::new(0.85, 0.35, 0.20).normalize();
        let result_dir = result.normalize();
        assert!(
            (result_dir - box_material_dir).dot(Vec3::ONE).abs() < 1.5,
            "expected the box's own warm-orange hue to dominate, got {result:?}"
        );
        let _ = e;
    }

    #[test]
    fn probe_ray_excludes_the_probe_can_never_self_intersect_via_entity_exclusion() {
        // Not applicable the same way indirect_ray's origin_entity
        // exclusion is (a probe has no "owning entity" to exclude — it's
        // a free-floating grid point, not a surface point) — this test
        // instead confirms PROBE_RAY_BIAS is enough to clear a probe
        // sitting very close to a surface without immediately
        // re-detecting that same surface at t~=0.
        let (_, objects, bvh) = ground_and_box();
        // A probe positioned just barely above the ground plate's own
        // top surface (0.0 + a hair), firing straight up — must NOT
        // immediately re-hit the ground at t~=0.
        let probe_pos = Vec3::new(5.0, 0.001, 5.0);
        let direction = Vec3::Y;
        let biased_origin = probe_pos + direction * PROBE_RAY_BIAS;
        let hit = trace(&bvh, &objects, biased_origin, direction, 3.0);
        assert!(hit.is_none(), "a probe just above the ground firing straight up should escape to open sky, not self-hit");
    }

    // -----------------------------------------------------------------
    // relight_probe_texel
    // -----------------------------------------------------------------

    #[test]
    fn relight_at_zero_history_length_passes_through_the_fresh_ray_exactly() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let probe_position = Vec3::new(10.0, 3.0, 10.0);
        let direction = texel_to_direction(4, 4, 8);
        let (expected_current, expected_distance) =
            probe_ray(&bvh, &objects, &lights, probe_position + direction * PROBE_RAY_BIAS, direction, 3.0, |_, _| Vec3::ZERO);
        let relit = relight_probe_texel(
            &bvh, &objects, &lights, probe_position, (4, 4), 8, 3.0, Vec3::ZERO, 0.0, 0.0, 0.0, 24.0, |_, _| Vec3::ZERO,
        );
        assert_eq!(relit.history_length, 1.0);
        assert!(
            (relit.irradiance - expected_current).length() < 1e-4,
            "at history_length 0, relight should pass through the fresh ray exactly: {:?} vs {expected_current:?}",
            relit.irradiance
        );
        assert!(
            (relit.mean_distance - expected_distance).abs() < 1e-4,
            "at history_length 0, relight should pass through the fresh distance exactly: {} vs {expected_distance}",
            relit.mean_distance
        );
        assert!(
            (relit.mean_distance_squared - expected_distance * expected_distance).abs() < 1e-3,
            "at history_length 0, relight should pass through the fresh distance^2 exactly: {} vs {}",
            relit.mean_distance_squared,
            expected_distance * expected_distance
        );
    }

    #[test]
    fn repeated_relighting_of_a_static_scene_converges_toward_the_true_value() {
        // A static scene's own probe_ray result IS the "true" value
        // (no per-frame randomness at a fixed texel/direction) — repeated
        // relighting should converge the EMA blend toward it, mirroring
        // temporal_ref's own two-frame-accumulation integration test
        // shape but iterated further since this uses the real
        // scene-based probe_ray as its own ground truth rather than a
        // synthetic before/after pair.
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let probe_position = Vec3::new(10.0, 3.0, 10.0);
        let texel = (2, 6);
        let tile_size = 8;
        let direction = texel_to_direction(texel.0, texel.1, tile_size);
        let (true_value, _true_distance) =
            probe_ray(&bvh, &objects, &lights, probe_position + direction * PROBE_RAY_BIAS, direction, 3.0, |_, _| Vec3::ZERO);

        let mut history = Vec3::ZERO;
        let mut history_mean_distance = 0.0f32;
        let mut history_mean_distance_squared = 0.0f32;
        let mut history_length = 0.0f32;
        for _ in 0..30 {
            let relit = relight_probe_texel(
                &bvh,
                &objects,
                &lights,
                probe_position,
                texel,
                tile_size,
                3.0,
                history,
                history_mean_distance,
                history_mean_distance_squared,
                history_length,
                24.0,
                |_, _| Vec3::ZERO,
            );
            history = relit.irradiance;
            history_mean_distance = relit.mean_distance;
            history_mean_distance_squared = relit.mean_distance_squared;
            history_length = relit.history_length;
        }
        assert!(
            (history - true_value).length() < 1e-3,
            "repeated relighting of an unchanging scene should converge to the scene's own true value: \
             history={history:?} true_value={true_value:?}"
        );
    }

    // -----------------------------------------------------------------
    // probe_ray's indirect_at_hit — the infinite-bounce link: a probe
    // ray's own hit point samples the EXISTING probe grid's irradiance
    // (via the caller's closure) and adds it, scaled by the hit
    // surface's diffuse albedo, on top of direct light — see probe_ray's
    // own doc comment for why this is what lets bounce light propagate
    // probe-to-probe across frames instead of capping at one order of
    // indirection.
    // -----------------------------------------------------------------

    #[test]
    fn probe_ray_adds_the_indirect_sample_scaled_by_the_hit_surfaces_diffuse_albedo() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let origin = Vec3::new(0.0, 2.0, 0.0);
        let direction = -Vec3::Y;
        let (direct_only, _) = probe_ray(&bvh, &objects, &lights, origin, direction, 5.0, |_, _| Vec3::ZERO);
        let injected = Vec3::new(2.0, 2.0, 2.0);
        let (with_indirect, _) = probe_ray(&bvh, &objects, &lights, origin, direction, 5.0, |_, _| injected);
        let added = with_indirect - direct_only;
        assert!(added.length() > 1e-4, "a non-zero indirect sample must add real energy on top of the direct term, got delta {added:?}");
        // Box's own base_color is (0.85, 0.35, 0.20), non-metallic — the
        // added delta must equal albedo * injected exactly (shade's own
        // diffuse_color * irradiance convention, mirrored here).
        let expected_added = Vec3::new(0.85, 0.35, 0.20) * injected;
        assert!(
            (added - expected_added).length() < 1e-3,
            "indirect contribution must be diffuse_color * injected irradiance exactly: got {added:?}, expected {expected_added:?}"
        );
    }

    #[test]
    fn probe_ray_gives_a_fully_metallic_hit_zero_indirect_contribution() {
        let e = entities(1);
        let metal = Material::new(Vec3::new(0.9, 0.9, 0.9), 1.0, 0.2);
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            material: metal,
        }];
        let hybrid_objects = vec![HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(1.0)) }];
        let bvh = Bvh::build(&hybrid_objects);
        let lights = [overhead_sun()];
        let origin = Vec3::new(0.0, 2.0, 0.0);
        let direction = -Vec3::Y;
        let (direct_only, _) = probe_ray(&bvh, &objects, &lights, origin, direction, 5.0, |_, _| Vec3::ZERO);
        let (with_indirect, _) = probe_ray(&bvh, &objects, &lights, origin, direction, 5.0, |_, _| Vec3::new(5.0, 5.0, 5.0));
        assert!(
            (with_indirect - direct_only).length() < 1e-5,
            "a fully metallic hit must get zero indirect contribution, same as shade's own direct-light treatment: direct={direct_only:?} with_indirect={with_indirect:?}"
        );
    }

    #[test]
    fn probe_ray_ignores_indirect_at_hit_entirely_on_a_miss() {
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        // Straight up from well above the box: a guaranteed miss.
        let (result, _) = probe_ray(&bvh, &objects, &lights, Vec3::new(10.0, 5.0, 10.0), Vec3::Y, 3.0, |_, _| Vec3::new(9.0, 9.0, 9.0));
        assert_eq!(result, Vec3::ZERO, "a miss has no hit point to sample indirect light at, so it must stay exactly black regardless of what indirect_at_hit would return");
    }

    #[test]
    fn feeding_a_probes_own_relit_neighbor_irradiance_back_in_lifts_a_shadowed_probes_own_result_above_direct_light_alone() {
        // The actual infinite-bounce claim, exercised end to end: probe
        // ray fired toward a point that itself sits in shadow (so its own
        // DIRECT light is zero) but whose indirect_at_hit closure reports
        // a real non-zero irradiance (standing in for "the grid's own
        // neighboring probes already carry real bounce light there") —
        // the probe's own relit result must come out strictly brighter
        // than the direct-only (all-zero) case, proving light genuinely
        // flows from one probe's own sample into another's relit value.
        let (_, objects, bvh) = ground_and_box();
        let lights = [overhead_sun()];
        let probe_position = Vec3::new(10.0, 3.0, 10.0);
        let texel = (4, 4);
        let tile_size = 8;
        let relit_direct_only = relight_probe_texel(
            &bvh, &objects, &lights, probe_position, texel, tile_size, 3.0, Vec3::ZERO, 0.0, 0.0, 0.0, 24.0, |_, _| Vec3::ZERO,
        );
        let relit_with_bounce = relight_probe_texel(
            &bvh, &objects, &lights, probe_position, texel, tile_size, 3.0, Vec3::ZERO, 0.0, 0.0, 0.0, 24.0, |_, _| {
                Vec3::new(1.0, 1.0, 1.0)
            },
        );
        assert!(
            relit_with_bounce.irradiance.length() >= relit_direct_only.irradiance.length(),
            "sampling a non-zero neighbor irradiance at the probe ray's hit point must not leave the relit result dimmer: \
             direct_only={:?} with_bounce={:?}",
            relit_direct_only.irradiance,
            relit_with_bounce.irradiance
        );
    }

    // -----------------------------------------------------------------
    // probe_grid_cell / sample_probe_grid
    // -----------------------------------------------------------------

    #[test]
    fn probe_grid_cell_finds_the_correct_lower_corner_and_fraction() {
        let grid = ProbeGrid { origin: Vec3::ZERO, spacing: Vec3::splat(10.0), dims: UVec3::new(4, 4, 4) };
        let (cell, frac) = probe_grid_cell(&grid, Vec3::new(15.0, 5.0, 25.0));
        assert_eq!(cell, UVec3::new(1, 0, 2));
        assert!((frac - Vec3::new(0.5, 0.5, 0.5)).length() < 1e-4, "got {frac:?}");
    }

    #[test]
    fn probe_grid_cell_clamps_a_point_outside_the_grid_to_the_nearest_valid_cell() {
        let grid = ProbeGrid { origin: Vec3::ZERO, spacing: Vec3::splat(10.0), dims: UVec3::new(3, 3, 3) };
        let (cell, _) = probe_grid_cell(&grid, Vec3::new(-50.0, -50.0, -50.0));
        assert_eq!(cell, UVec3::ZERO, "a point far below/outside the grid's own origin must clamp to cell 0, not underflow");
        let (cell, _) = probe_grid_cell(&grid, Vec3::new(500.0, 500.0, 500.0));
        assert_eq!(cell, UVec3::new(1, 1, 1), "a point far past the grid's own far corner must clamp to the last valid cell");
    }

    #[test]
    fn sample_at_a_probes_own_exact_position_returns_close_to_that_probes_own_value() {
        let grid = ProbeGrid { origin: Vec3::ZERO, spacing: Vec3::splat(10.0), dims: UVec3::new(2, 2, 2) };
        let (_, objects, bvh) = ground_and_box();
        // Probe (0,0,0) reports a distinct known value; every other
        // probe reports a different one, so a sample exactly AT probe
        // (0,0,0)'s own position should be dominated by its own weight.
        let target_probe_value = Vec3::new(0.9, 0.1, 0.1);
        let other_probe_value = Vec3::new(0.1, 0.1, 0.9);
        let lookup = |coords: UVec3, _dir: Vec3| {
            if coords == UVec3::ZERO { target_probe_value } else { other_probe_value }
        };
        let sampled = sample_probe_grid(&grid, &bvh, &objects, Vec3::ZERO, Vec3::Y, 1.0, None, lookup, always_visible_moments);
        assert!(
            (sampled - target_probe_value).length() < 1e-3,
            "a sample exactly at probe (0,0,0)'s own position should equal its own value: got {sampled:?}"
        );
    }

    #[test]
    fn an_occluded_probe_contributes_zero_not_a_reduced_weight() {
        // A shaded point on TOP of the box (world Y=1.6), with one probe
        // placed BELOW the ground plate (occluded by both the ground and
        // the box) and another placed directly above with a clear line
        // of sight — the occluded probe's distinct color must not leak
        // into the result at all.
        let (_, objects, bvh) = ground_and_box();
        let grid = ProbeGrid { origin: Vec3::new(-5.0, -5.0, -5.0), spacing: Vec3::splat(10.0), dims: UVec3::new(2, 2, 2) };
        // probe (x,0,z) sits at world Y=-5 (well below/inside the ground
        // plate -> occluded from a point above); probe (x,1,z) sits at
        // world Y=5 (clear line of sight from a point on the box's own
        // top face at Y=1.6).
        let occluded_color = Vec3::new(1.0, 0.0, 0.0);
        let visible_color = Vec3::new(0.0, 1.0, 0.0);
        let lookup = |coords: UVec3, _dir: Vec3| if coords.y == 0 { occluded_color } else { visible_color };
        let shaded_point = Vec3::new(0.0, 1.6, 0.0);
        let sampled = sample_probe_grid(&grid, &bvh, &objects, shaded_point, Vec3::Y, 1.0, None, lookup, always_visible_moments);
        assert!(
            (sampled - visible_color).length() < 0.05,
            "the occluded (below-ground) probe's color must not leak into the sample: got {sampled:?}"
        );
    }

    // -----------------------------------------------------------------
    // chebyshev_visibility_weight / Chebyshev depth-aware visibility
    // -----------------------------------------------------------------

    #[test]
    fn chebyshev_weight_is_full_when_the_shaded_point_is_no_farther_than_the_probes_own_mean() {
        // dist <= mean is treated as fully visible regardless of variance
        // — Chebyshev's one-sided bound only usefully constrains "farther
        // than what the probe usually sees," see chebyshev_visibility_
        // weight's own doc comment.
        assert_eq!(chebyshev_visibility_weight(5.0, 10.0, 100.0), 1.0);
        assert_eq!(chebyshev_visibility_weight(10.0, 10.0, 100.0), 1.0);
    }

    #[test]
    fn chebyshev_weight_drops_toward_zero_far_beyond_a_confidently_known_distance() {
        // A probe whose rays consistently report ~5.0 in this direction
        // (low variance: mean=5, mean_sq=25.01 -> variance~=0.01) is
        // confidently reporting "something sits right there" — a shaded
        // point far beyond that (dist=50) should be almost fully
        // discounted, not just partially.
        let weight = chebyshev_visibility_weight(50.0, 5.0, 25.01);
        assert!(weight < 0.01, "a confidently-close probe should almost fully discount a much farther point: got {weight}");
    }

    #[test]
    fn chebyshev_weight_stays_higher_for_a_noisier_probe_at_the_identical_distance() {
        // Two probes report the identical mean (5.0) toward the identical
        // far shaded point (dist=50), but one is confidently consistent
        // (low variance: mean_sq=25.01) and the other is genuinely noisy
        // (high variance: mean_sq=500) — the noisy probe's own "typical"
        // distance is a less reliable signal, so it must be discounted
        // LESS aggressively than the confident one for the exact same
        // distance gap, not held to some fixed absolute threshold.
        let confident_weight = chebyshev_visibility_weight(50.0, 5.0, 25.01);
        let noisy_weight = chebyshev_visibility_weight(50.0, 5.0, 500.0);
        assert!(
            noisy_weight > confident_weight,
            "a noisier probe must be discounted less than a confident one at the same distance gap: \
             noisy={noisy_weight} confident={confident_weight}"
        );
    }

    #[test]
    fn chebyshev_weight_never_divides_by_a_true_zero_variance() {
        // A probe whose rays ALL report EXACTLY the same distance (a
        // perfectly flat wall filling its whole hemisphere) has variance
        // of exactly 0.0 — CHEBYSHEV_VARIANCE_FLOOR must keep the
        // formula finite rather than dividing by zero.
        let weight = chebyshev_visibility_weight(10.0, 5.0, 25.0);
        assert!(weight.is_finite(), "zero true variance must not produce NaN/inf: got {weight}");
        assert!((0.0..=1.0).contains(&weight), "weight must stay in [0,1]: got {weight}");
    }

    #[test]
    fn sample_probe_grid_discounts_a_probe_whose_own_distance_moments_say_it_cannot_see_this_far() {
        // The exact failure mode Chebyshev visibility fixes: a probe
        // whose HARD occlusion ray (from the shaded point's own
        // perspective) reports clear line-of-sight, but whose OWN relit
        // rays (from the probe's own perspective, stored as distance
        // moments) consistently found something much closer — meaning
        // the probe's own stored irradiance is itself unreliable for a
        // point this far away, e.g. because the probe sits just past a
        // thin wall from the shaded point's exact angle but the single
        // occlusion ray happened to thread through a gap. No real
        // geometry needed to prove this in isolation — a fully open
        // scene (nothing to occlude the hard ray at all) with a probe
        // grid whose own moments are hand-set to "this probe confidently
        // sees something 1.0 units away in every direction" is the
        // cleanest fixture: the hard ray reports unoccluded (trivially,
        // since there is no geometry at all), so ONLY the Chebyshev term
        // can be responsible for any down-weighting observed.
        let objects: Vec<TraceObject> = vec![];
        let hybrid_objects: Vec<HybridObject> = vec![];
        let bvh = Bvh::build(&hybrid_objects);
        // A tiny grid (probes 1 unit apart) so the shaded point sits
        // MUCH farther from probe (0,0,0) than that probe's own
        // confidently-reported reach, while still landing in the same
        // grid cell trilinear interpolation would otherwise weight
        // heavily toward it (closest corner).
        let grid = ProbeGrid { origin: Vec3::new(-5.0, -5.0, -5.0), spacing: Vec3::splat(1.0), dims: UVec3::new(2, 2, 2) };
        let target_probe_value = Vec3::new(0.9, 0.1, 0.1);
        let other_probe_value = Vec3::new(0.1, 0.1, 0.9);
        let lookup = |coords: UVec3, _dir: Vec3| if coords == UVec3::ZERO { target_probe_value } else { other_probe_value };
        // Probe (0,0,0) sits at the grid's own origin (-5,-5,-5) — a
        // shaded point 0.9 units away (well inside probe (0,0,0)'s own
        // cell, so trilinear weighting favors it heavily) but a moments
        // closure reporting "probe (0,0,0) confidently sees only ~0.1
        // units in every direction" (e.g. it's tucked right against a
        // thin wall from its own vantage point) should make Chebyshev
        // discount its contribution despite the clear hard-occlusion ray
        // (there is no geometry at all in this fixture) and the
        // trilinear proximity both favoring it.
        let confident_close_moments = |coords: UVec3, _dir: Vec3| if coords == UVec3::ZERO { (0.1, 0.0101) } else { (f32::MAX, f32::MAX) };
        let shaded_point = Vec3::new(-5.0 + 0.9, -5.0 + 0.9, -5.0 + 0.9);
        let sampled = sample_probe_grid(&grid, &bvh, &objects, shaded_point, Vec3::Y, 1.0, None, lookup, confident_close_moments);
        assert!(
            (sampled - target_probe_value).length() > 0.3,
            "a probe confidently reporting it cannot see this far, per its own distance moments, must be discounted \
             even with a clear hard-occlusion ray and trilinear proximity in its favor: got {sampled:?} vs target {target_probe_value:?}"
        );
    }

    #[test]
    fn the_occlusion_fallback_has_no_hard_jump_as_weight_sum_crosses_the_old_threshold() {
        // Regression test for a real flicker bug found by direct visual
        // inspection in examples/gi_room.rs: a first version of
        // sample_probe_grid switched formulas outright at
        // `weight_sum == 0` (pure trilinear-weighted average on one
        // side, pure unweighted raw-8-probe average on the other) —
        // near a wall/corner, where a shaded point's own occlusion
        // state can flip probe-by-probe frame to frame at grazing
        // angles, `weight_sum` crossing exactly zero flipped the WHOLE
        // result between two structurally different formulas, a hard
        // visual pop even though only one probe's occlusion state
        // changed. Reproduced here directly: 7 of 8 grid corners sit
        // deep below the ground plate (always occluded from a point
        // above), the 8th sits high above it (always unoccluded) — so
        // this single unoccluded corner's OWN trilinear weight IS
        // `weight_sum` in its entirety. Sweeping the shaded point's Y
        // position continuously moves that one corner's own weight
        // through the old hard threshold (0) and the new ramp's own
        // ceiling (FALLBACK_RAMP_WEIGHT) — the result must vary
        // continuously (no jump bigger than what a small position step
        // could plausibly produce) across that whole sweep.
        let (_, objects, bvh) = ground_and_box();
        // Grid Y spacing 20: corner y=0 sits at world y=-15 (deep below
        // the ground plate, occluded); corner y=1 sits at world y=5
        // (clear line of sight from above, unoccluded). X/Z spacing
        // large enough that the swept point stays within cell (0,*,0)
        // for the whole sweep.
        let grid = ProbeGrid { origin: Vec3::new(-50.0, -15.0, -50.0), spacing: Vec3::new(100.0, 20.0, 100.0), dims: UVec3::new(2, 2, 2) };
        let occluded_color = Vec3::new(1.0, 0.0, 0.0);
        let visible_color = Vec3::new(0.0, 1.0, 0.0);
        let lookup = |coords: UVec3, _dir: Vec3| if coords.y == 0 { occluded_color } else { visible_color };

        // Sweep world Y from just below the grid's own y=1 corner's
        // weight reaching FALLBACK_RAMP_WEIGHT down through weight_sum
        // == 0 (world y = -15, the grid's own lower Y bound) — frac.y
        // = (world_y - (-15)) / 20, so weight_sum = frac.y across this
        // whole sweep (the only unoccluded corner's own weight).
        let mut samples = Vec::new();
        let mut y = -15.0f32;
        while y <= -15.0 + 20.0 * (FALLBACK_RAMP_WEIGHT + 0.05) {
            let shaded_point = Vec3::new(0.0, y, 0.0);
            let sampled = sample_probe_grid(&grid, &bvh, &objects, shaded_point, Vec3::Y, 1.0, None, lookup, always_visible_moments);
            samples.push((y, sampled));
            y += 0.05;
        }
        assert!(samples.len() > 10, "test sanity: expected a real multi-step sweep");

        // No single step's own change should be dramatically larger
        // than its neighbors' — a genuine hard jump (the old bug) would
        // show as one step's delta being an order of magnitude bigger
        // than the steps immediately around it, since a continuous
        // ramp's per-step delta stays roughly comparable across the
        // whole sweep for a fixed step size.
        let mut max_step_delta = 0.0f32;
        let mut deltas = Vec::new();
        for pair in samples.windows(2) {
            let delta = (pair[1].1 - pair[0].1).length();
            deltas.push(delta);
            max_step_delta = max_step_delta.max(delta);
        }
        let avg_delta: f32 = deltas.iter().sum::<f32>() / deltas.len() as f32;
        assert!(
            max_step_delta < avg_delta * 4.0 + 0.05,
            "expected no single step to jump far more than the sweep's own average step size \
             (a hard discontinuity would show up as exactly this) — max_step_delta={max_step_delta:.4} \
             avg_delta={avg_delta:.4} samples={samples:?}"
        );

        // Sanity: the two ENDPOINTS of the sweep must still match the
        // old hard branch's own two exact formulas — the fix changes
        // the transition, not the endpoints.
        let at_zero = sample_probe_grid(&grid, &bvh, &objects, Vec3::new(0.0, -15.0, 0.0), Vec3::Y, 1.0, None, lookup, always_visible_moments);
        assert!(
            (at_zero - occluded_color.lerp(visible_color, 0.0)).length() < 1e-4,
            "at weight_sum == 0 exactly, the result must still equal the old pure-fallback \
             (all-8-raw-average, both colors present as this fixture only has 2 distinct \
             probe colors split 4/4) value: got {at_zero:?}"
        );
        let well_covered = sample_probe_grid(
            &grid,
            &bvh,
            &objects,
            Vec3::new(0.0, -15.0 + 20.0 * 0.9, 0.0),
            Vec3::Y,
            1.0,
            None,
            lookup,
            always_visible_moments,
        );
        assert!(
            (well_covered - visible_color).length() < 1e-3,
            "once weight_sum clears FALLBACK_RAMP_WEIGHT, the result must equal the old pure \
             weighted-average (visible_color only, since the sole occluded corner is fully \
             excluded): got {well_covered:?}"
        );
    }

    #[test]
    fn a_rotated_boxs_own_occlusion_ray_does_not_self_intersect_its_own_shaded_face() {
        // Regression test for a real bug found by direct visual
        // inspection: a --stress N cube spinning in place showed one
        // face intermittently flicker dark at specific rotation angles,
        // then recover — caused by sample_probe_grid's occlusion ray
        // self-intersecting the SAME box it was cast from (no
        // origin_entity exclusion, unlike every other surface-launched
        // ray in this renderer). Reproduced here directly: a box rotated
        // 45 degrees around Y, shaded at a point on its own +X face
        // near one edge, with a probe placed on the OPPOSITE side of
        // the box — the straight line between them passes back through
        // the box's own rotated volume. Without the origin_entity
        // exclusion this self-intersects and zeroes the probe; with it,
        // the probe must be treated as fully visible.
        let e = entities(1);
        let half = Vec3::splat(1.0);
        let rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_4);
        let objects = vec![TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: half, corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation,
            material: Material::new(Vec3::new(0.85, 0.35, 0.20), 0.0, 0.4),
        }];
        let hybrid_objects: Vec<HybridObject> =
            vec![HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(1.5)) }];
        let bvh = Bvh::build(&hybrid_objects);

        // A point on the box's own (local) +X face, near its own -Z
        // edge, transformed to world space by the box's own rotation —
        // a real point on the rotated box's own surface, not an
        // approximation.
        let local_p = Vec3::new(half.x, 0.0, -half.z * 0.98);
        let local_n = Vec3::X;
        let world_p = rotation * local_p;
        let world_n = (rotation * local_n).normalize();

        // Probe placed on the OPPOSITE side of the box (past its own
        // -X face), offset in +Z — the straight line from world_p to
        // the probe crosses back through the box's own rotated volume
        // (the box occupies the segment's middle), exactly the
        // self-intersection geometry the missing exclusion produced.
        let local_probe = Vec3::new(-half.x - 5.0, 0.0, half.z * 0.98);
        let probe_pos = rotation * local_probe;

        let lookup = |_coords: UVec3, _dir: Vec3| Vec3::new(0.2, 0.6, 0.9);
        let grid = ProbeGrid { origin: probe_pos - Vec3::splat(0.001), spacing: Vec3::splat(0.002), dims: UVec3::new(2, 2, 2) };

        let without_exclusion = sample_probe_grid(&grid, &bvh, &objects, world_p, world_n, 1.0, None, lookup, always_visible_moments);
        let with_exclusion = sample_probe_grid(&grid, &bvh, &objects, world_p, world_n, 1.0, Some(e[0]), lookup, always_visible_moments);

        assert!(
            (with_exclusion - lookup(UVec3::ZERO, Vec3::ZERO)).length() < 1e-3,
            "with origin_entity excluded, the near-tangential probe must be treated as fully visible \
             (no self-occlusion): got {with_exclusion:?}"
        );
        assert!(
            without_exclusion != with_exclusion || without_exclusion.length() < 1e-3,
            "sanity: this geometry should demonstrate a real difference between excluding and not excluding \
             the shaded object's own entity — without_exclusion={without_exclusion:?} \
             with_exclusion={with_exclusion:?} (if these match, the test geometry doesn't reproduce the bug; \
             adjust local_p/probe_pos rather than weakening this assertion)"
        );
    }

    #[test]
    fn excluding_the_origin_entity_does_not_hide_a_real_occluder_farther_along_the_same_ray() {
        // Regression test for a real gap the `any_hit`-based occlusion
        // query closes over `trace()`-based occlusion: `trace()` reports
        // the NEAREST hit only, so if the shaded object's OWN geometry
        // happens to be nearest (a self-graze near a grazing angle/edge),
        // a check of the form `nearest_hit.entity != origin_entity` stops
        // right there and never learns whether something else — a real,
        // separate occluder — sits farther along the SAME ray before it
        // ever reaches the probe. `any_hit` instead skips the excluded
        // entity's own leaf during traversal and keeps searching, so a
        // real occluder behind it is still found. This is the load-
        // bearing correctness property for the perf change described in
        // PROGRESS.md's DDGI-hard-occlusion-ray entry — reusing an
        // occlusion primitive that stopped at the excluded entity's own
        // self-graze would silently re-open a light leak.
        let e = entities(2);
        // Same rotated-box self-graze geometry as
        // `a_rotated_boxs_own_occlusion_ray_does_not_self_intersect_its_
        // own_shaded_face` above: a box rotated 45 degrees about Y,
        // shaded at a point on its own +X face near an edge, with the
        // probe on the OPPOSITE side — the straight line crosses back
        // through the box's own rotated volume (must be excluded). Unlike
        // that test, a SECOND, genuinely separate occluder sits further
        // along the same line, past the box's own far side — a check that
        // only ever asks "is the NEAREST hit something other than self"
        // (the old `trace()`-based logic) stops at the excluded self-graze
        // and never learns this second occluder is also in the way.
        let half = Vec3::splat(1.0);
        let rotation = Quat::from_rotation_y(std::f32::consts::FRAC_PI_4);
        let shaded_object = TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: half, corner_radius: 0.0 },
            translation: Vec3::ZERO,
            rotation,
            material: Material::new(Vec3::new(0.85, 0.35, 0.20), 0.0, 0.4),
        };

        let local_p = Vec3::new(half.x, 0.0, -half.z * 0.98);
        let local_n = Vec3::X;
        let world_p = rotation * local_p;
        let world_n = (rotation * local_n).normalize();

        let local_probe = Vec3::new(-half.x - 10.0, 0.0, half.z * 0.98);
        let probe_pos = rotation * local_probe;

        // A genuinely separate occluder placed on the segment between the
        // box's own -X exit point and the probe (world-space, well clear
        // of the rotated box's own volume).
        let local_real_occluder = Vec3::new(-half.x - 5.0, 0.0, half.z * 0.98);
        let real_occluder_pos = rotation * local_real_occluder;
        let real_occluder = TraceObject {
            entity: e[1],
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
            translation: real_occluder_pos,
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::new(0.1, 0.1, 0.9), 0.0, 0.4),
        };

        let objects = vec![shaded_object, real_occluder];
        let hybrid_objects: Vec<HybridObject> = vec![
            HybridObject { entity: e[0], world_aabb: Aabb::from_center_half(Vec3::ZERO, Vec3::splat(1.5)) },
            HybridObject { entity: e[1], world_aabb: Aabb::from_center_half(real_occluder_pos, Vec3::splat(1.0)) },
        ];
        let bvh = Bvh::build(&hybrid_objects);

        // Direct, unambiguous check at the exact call shape
        // `sample_probe_grid` itself uses (same bias, same ray) — with the
        // shaded object excluded, `any_hit` must still report occluded,
        // because the real, separate occluder sits farther along the same
        // ray. A checked-via-`sample_probe_grid`-color version of this
        // test is deliberately avoided: with only 2 objects and a
        // degenerate single-point probe grid, every corner sees identical
        // geometry, so an all-occluded result and an all-VISIBLE result
        // are only distinguishable by `sample_probe_grid`'s OWN fallback-
        // to-raw-average behavior producing the SAME color either way
        // (see `an_occluded_probe_contributes_zero_not_a_reduced_weight`
        // above for the non-degenerate grid shape that IS needed to make
        // occlusion visible through `sample_probe_grid`'s output) —
        // asserting on `any_hit` directly is the precise, unconfounded
        // check for the property this test exists to pin.
        let bias = PROBE_RAY_BIAS;
        let origin = world_p + world_n * bias;
        let to_probe = probe_pos - origin;
        let dist = to_probe.length();
        let dir = to_probe / dist;
        assert!(
            any_hit(&bvh, &objects, origin, dir, dist - bias, Some(e[0])),
            "a real, separate occluder behind the excluded (self) entity must still be found by any_hit"
        );
    }

    // -----------------------------------------------------------------
    // ddgi_tangent_basis / cosine_weighted_probe_irradiance
    // -----------------------------------------------------------------

    #[test]
    fn ddgi_tangent_basis_is_orthonormal_for_arbitrary_normals() {
        for n in [
            Vec3::Y,
            Vec3::X,
            Vec3::Z,
            -Vec3::Y,
            Vec3::new(0.3, 0.8, 0.5).normalize(),
            Vec3::new(-0.2, 0.6, 0.77).normalize(),
            Vec3::new(0.0, -1.0, 0.0), // straight down: the s-flip branch
        ] {
            let basis = ddgi_tangent_basis(n);
            let x = basis * Vec3::X;
            let y = basis * Vec3::Y;
            let z = basis * Vec3::Z;
            assert!((x.length() - 1.0).abs() < 1e-4, "n={n:?}: x not unit length: {x:?}");
            assert!((y.length() - 1.0).abs() < 1e-4, "n={n:?}: y not unit length: {y:?}");
            assert!((z.length() - 1.0).abs() < 1e-4, "n={n:?}: z not unit length: {z:?}");
            assert!(x.dot(y).abs() < 1e-4, "n={n:?}: x,y not orthogonal: {}", x.dot(y));
            assert!(x.dot(z).abs() < 1e-4, "n={n:?}: x,z not orthogonal: {}", x.dot(z));
            assert!(y.dot(z).abs() < 1e-4, "n={n:?}: y,z not orthogonal: {}", y.dot(z));
            assert!((z - n).length() < 1e-4, "n={n:?}: z axis must equal the input normal exactly: got {z:?}");
        }
    }

    #[test]
    fn cosine_weighted_irradiance_of_a_uniform_field_matches_that_uniform_value() {
        // Sanity: if every hemisphere direction reports the SAME
        // irradiance, the convolved result must equal that same value
        // exactly (the average of N identical values is that value) —
        // confirms the convolution doesn't introduce any spurious
        // scaling/bias on its own.
        let uniform = Vec3::new(0.4, 0.5, 0.6);
        let result = cosine_weighted_probe_irradiance(Vec3::Y, |_dir| uniform);
        assert!((result - uniform).length() < 1e-5, "expected the uniform field's own value back, got {result:?}");
    }

    #[test]
    fn roughness_aware_convolution_at_full_roughness_matches_the_original_function_exactly() {
        // roughness = 1.0 (fully rough/diffuse) must reproduce the same
        // full-spread hemisphere the original, non-roughness-aware
        // function always used — existing callers/tests relying on that
        // exact behavior must see no change at this value.
        let lookup = |dir: Vec3| Vec3::new(dir.x, dir.y, dir.z).abs();
        let original = cosine_weighted_probe_irradiance(Vec3::Y, lookup);
        let roughness_aware = cosine_weighted_probe_irradiance_roughness_aware(Vec3::Y, 1.0, lookup);
        assert!(
            (original - roughness_aware).length() < 1e-5,
            "roughness=1.0 must match the original full-spread function exactly: {original:?} vs {roughness_aware:?}"
        );
    }

    #[test]
    fn roughness_aware_convolution_at_zero_roughness_converges_toward_the_normal_direction() {
        // roughness = 0.0 (fully smooth/glossy) pulls every sample onto
        // the pole (straight along surface_normal) — a probe_irradiance
        // closure that reports a DISTINCT value only for the exact
        // normal direction (and something else everywhere off-axis)
        // should dominate the convolved result almost completely, unlike
        // roughness=1.0 where the 4 off-axis samples (80% of the total)
        // would dominate instead.
        let normal = Vec3::Y;
        let on_axis_value = Vec3::new(1.0, 0.0, 0.0);
        let off_axis_value = Vec3::new(0.0, 0.0, 1.0);
        let lookup = |dir: Vec3| if dir.dot(normal) > 0.999 { on_axis_value } else { off_axis_value };
        let full_spread = cosine_weighted_probe_irradiance_roughness_aware(normal, 1.0, lookup);
        let no_spread = cosine_weighted_probe_irradiance_roughness_aware(normal, 0.0, lookup);
        assert!(
            (no_spread - on_axis_value).length() < (full_spread - on_axis_value).length(),
            "roughness=0.0 must land closer to the pure on-axis value than roughness=1.0: \
             no_spread={no_spread:?} full_spread={full_spread:?} on_axis={on_axis_value:?}"
        );
    }

    #[test]
    fn roughness_aware_convolution_narrows_each_individual_sample_toward_the_pole() {
        // The mechanism this feature relies on, checked directly rather
        // than through an indirect brightness proxy: at a lower
        // roughness, EVERY one of the 5 fixed hemisphere sample
        // directions (transformed into world space around the same
        // surface_normal) must sit closer to normal_direction itself
        // than the same sample did at a higher roughness — i.e. the
        // spread genuinely narrows sample-by-sample, not just "some
        // aggregate brightness metric happens to trend downward" (which
        // an indirect probe_irradiance field can obscure for a
        // sufficiently asymmetric field/normal combination, since
        // HEMISPHERE_SAMPLES isn't azimuthally symmetric around an
        // arbitrary off-axis normal — a real, minor non-monotonicity in
        // aggregate brightness at intermediate roughness values that a
        // stricter per-step brightness assertion incorrectly flagged
        // before this test replaced it).
        let normal = Vec3::new(1.0, 1.0, 0.0).normalize();
        let basis = ddgi_tangent_basis(normal);
        let roughness_values = [1.0, 0.75, 0.5, 0.25, 0.0];
        for sample in HEMISPHERE_SAMPLES {
            let mut previous_alignment = -1.0f32;
            for &roughness in &roughness_values {
                let narrowed = sample.lerp(Vec3::new(0.0, 0.0, 1.0), 1.0 - roughness).normalize();
                let world_dir = (basis * narrowed).normalize();
                let alignment = world_dir.dot(normal);
                assert!(
                    alignment >= previous_alignment - 1e-5,
                    "each sample's own alignment with the surface normal must increase (or stay flat) \
                     as roughness drops: roughness={roughness} alignment={alignment} previous={previous_alignment}"
                );
                previous_alignment = alignment;
            }
        }
    }

    #[test]
    fn a_surface_facing_away_from_the_only_lit_direction_still_receives_bounce_light() {
        // The direct regression test for the "away-facing surfaces are
        // pitch black" bug: a synthetic probe lookup that returns a
        // bright color for ONE specific direction (the sun's own
        // reflection direction, say) and near-zero for every other
        // direction — including the shaded surface's OWN exact normal.
        // Before hemisphere convolution, sample_probe_grid's single
        // direct-normal lookup would report ~zero here (the normal
        // direction itself is dark); after convolution, the OTHER 4
        // HEMISPHERE_SAMPLES directions (which do NOT all coincide with
        // the shaded normal after being transformed by the tangent
        // basis) have a real chance of landing near the bright
        // direction, producing a genuinely non-zero result.
        let bright_dir = Vec3::new(0.6614, 0.0, 0.75).normalize(); // HEMISPHERE_SAMPLES[1], pre-basis-transform
        let shaded_normal = Vec3::Y;
        // Under ddgi_tangent_basis(Vec3::Y), HEMISPHERE_SAMPLES[1] maps to
        // some world direction != Vec3::Y itself — the lookup below
        // reports bright ONLY for directions close to that exact mapped
        // direction, dark (including at shaded_normal itself) otherwise.
        let basis = ddgi_tangent_basis(shaded_normal);
        let bright_world_dir = (basis * bright_dir).normalize();
        let lookup = move |dir: Vec3| {
            if dir.dot(bright_world_dir) > 0.99 { Vec3::new(1.0, 1.0, 1.0) } else { Vec3::ZERO }
        };
        // Direct (pre-convolution-equivalent) single-sample read at the
        // shaded normal itself must be dark — confirms the test's own
        // premise (the "normal direction" and "bright direction" really
        // are different here).
        assert!(lookup(shaded_normal).length() < 1e-4, "test setup error: the normal direction itself must be dark");

        let convolved = cosine_weighted_probe_irradiance(shaded_normal, lookup);
        assert!(
            convolved.length() > 1e-4,
            "a surface facing away from the only lit direction must still receive SOME bounce light via \
             hemisphere convolution (this is the away-facing-surfaces-are-black regression check): got {convolved:?}"
        );
    }

    // -----------------------------------------------------------------
    // probe_grid_from_bounds
    // -----------------------------------------------------------------

    #[test]
    fn a_known_bounds_and_spacing_produces_the_expected_probe_dims() {
        // A 22x10x22 box (2 stress-grid cells wide/deep at 11-unit
        // pitch) with 11-unit horizontal spacing tiles into exactly 2
        // cells per horizontal axis (cell-center layout, not edge-
        // pinned — see probe_grid_from_bounds's own doc comment for
        // why probes sit at cell centers, not at bounds' own corners).
        let bounds = Aabb { min: Vec3::new(-11.0, 0.0, -11.0), max: Vec3::new(11.0, 10.0, 11.0) };
        let grid = probe_grid_from_bounds(bounds, Vec3::new(11.0, 11.0, 11.0), 3);
        assert_eq!(grid.dims.x, 2, "22 units / 11-unit spacing = 2 cells, one probe at each cell's own center");
        assert_eq!(grid.dims.z, 2);
        assert_eq!(grid.dims.y, 3, "vertical_layers is honored directly");
    }

    #[test]
    fn grid_origin_sits_a_half_cell_inside_bounds_min_and_the_grid_covers_the_full_extent() {
        // Cell-center layout: probe 0 sits at bounds.min + spacing/2 on
        // every axis (NOT exactly at bounds.min — see
        // probe_grid_from_bounds's own doc comment for the real bug
        // this fixes: an edge-pinned probe sits exactly on/inside
        // whatever geometry actually defines that bound, e.g. a sealed
        // room's own wall).
        let bounds = Aabb { min: Vec3::new(-11.0, 0.0, -11.0), max: Vec3::new(11.0, 10.0, 11.0) };
        let grid = probe_grid_from_bounds(bounds, Vec3::new(11.0, 11.0, 11.0), 3);
        let half_cell = grid.spacing * 0.5;
        assert!(
            (grid.origin - (bounds.min + half_cell)).length() < 1e-3,
            "origin should be bounds.min + half a cell on every axis: origin={:?} expected={:?}",
            grid.origin,
            bounds.min + half_cell
        );
        // The grid's own outer probe-CENTER positions should each sit
        // within one half-cell of bounds' own edges (the cells
        // themselves still fully tile the input bounds, even though no
        // individual probe sits exactly ON an edge anymore).
        let last = grid.probe_position(grid.dims.x - 1, grid.dims.y - 1, grid.dims.z - 1);
        assert!(
            last.x <= bounds.max.x && last.y <= bounds.max.y && last.z <= bounds.max.z,
            "the last probe's own center must stay within bounds (inset by half a cell): last={last:?} bounds.max={:?}",
            bounds.max
        );
        assert!(
            (bounds.max.x - last.x) <= grid.spacing.x && (bounds.max.z - last.z) <= grid.spacing.z,
            "the last probe must be within one full cell of bounds.max (i.e. genuinely near the edge, not\
             collapsed toward the center): last={last:?} bounds.max={:?} spacing={:?}",
            bounds.max,
            grid.spacing
        );
    }

    #[test]
    fn a_degenerate_zero_size_bounds_still_produces_a_usable_grid() {
        let bounds = Aabb { min: Vec3::ZERO, max: Vec3::ZERO };
        let grid = probe_grid_from_bounds(bounds, Vec3::new(11.0, 11.0, 11.0), 3);
        assert!(grid.probe_count() >= 8, "even a degenerate bounds must produce at least a 2x2x2 interpolatable grid");
        assert!(grid.dims.x >= MIN_PROBES_PER_AXIS && grid.dims.z >= MIN_PROBES_PER_AXIS);
    }

    #[test]
    fn probe_count_scales_as_the_product_of_dims() {
        let grid = ProbeGrid { origin: Vec3::ZERO, spacing: Vec3::splat(11.0), dims: UVec3::new(4, 3, 5) };
        assert_eq!(grid.probe_count(), 4 * 3 * 5);
    }

    #[test]
    fn probe_position_flat_matches_probe_position_at_every_index() {
        let grid = ProbeGrid { origin: Vec3::new(1.0, 2.0, 3.0), spacing: Vec3::new(5.0, 6.0, 7.0), dims: UVec3::new(3, 2, 4) };
        let mut flat_index = 0u32;
        for z in 0..grid.dims.z {
            for y in 0..grid.dims.y {
                for x in 0..grid.dims.x {
                    let expected = grid.probe_position(x, y, z);
                    let actual = grid.probe_position_flat(flat_index);
                    assert_eq!(actual, expected, "flat_index={flat_index} (x={x},y={y},z={z})");
                    flat_index += 1;
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // AtlasLayout
    // -----------------------------------------------------------------

    #[test]
    fn exact_fit_atlas_uses_the_smallest_square_that_fits_every_probe() {
        // 10 probes: ceil(sqrt(10)) = 4 tiles per row (4*4=16 >= 10, 3*3=9 < 10).
        let layout = AtlasLayout::exact_fit(10, 8);
        assert_eq!(layout.tiles_per_row, 4);
        assert_eq!(layout.atlas_pixels, 4 * 8);
    }

    #[test]
    fn exact_fit_atlas_never_produces_a_zero_size_texture_for_an_empty_grid() {
        let layout = AtlasLayout::exact_fit(0, 8);
        assert!(layout.tiles_per_row >= 1 && layout.atlas_pixels >= 8, "an empty grid must still produce a usable atlas");
    }

    #[test]
    fn a_perfect_square_probe_count_fits_exactly_with_no_wasted_row() {
        // 9 probes: ceil(sqrt(9)) = 3 exactly, no slack row/column.
        let layout = AtlasLayout::exact_fit(9, 8);
        assert_eq!(layout.tiles_per_row, 3);
    }

    #[test]
    fn tile_origins_never_overlap_and_stay_within_atlas_bounds() {
        let probe_count = 30603; // the real --stress 10000 number, see debug_stress_10000_probe_grid_scale_profile
        let tile_size = 8;
        let layout = AtlasLayout::exact_fit(probe_count, tile_size);
        let mut seen = std::collections::HashSet::new();
        // Full 30603-tile enumeration is real work but still fast (a
        // plain arithmetic loop, no ray tracing) — run it in full rather
        // than a sample, since an overlap bug could hide in any single
        // index.
        for flat_index in 0..probe_count {
            let (x, y) = layout.tile_origin(flat_index);
            assert!(x + tile_size <= layout.atlas_pixels && y + tile_size <= layout.atlas_pixels, "tile {flat_index} at ({x},{y}) exceeds atlas bounds {}", layout.atlas_pixels);
            assert!(seen.insert((x, y)), "tile origin ({x},{y}) reused by more than one probe index (flat_index={flat_index})");
        }
    }

    // -----------------------------------------------------------------
    // octahedral_encode / octahedral_decode round trip
    // -----------------------------------------------------------------

    fn assert_close(a: Vec3, b: Vec3, tol: f32, msg: &str) {
        assert!((a - b).length() < tol, "{msg}: {a:?} vs {b:?}");
    }

    #[test]
    fn decode_of_encode_recovers_the_original_direction_for_axis_aligned_vectors() {
        for v in [Vec3::X, Vec3::Y, Vec3::Z, -Vec3::X, -Vec3::Y, -Vec3::Z] {
            let uv = octahedral_encode(v);
            let decoded = octahedral_decode(uv);
            assert_close(decoded, v, 1e-4, "axis-aligned round trip failed for {v:?}");
        }
    }

    #[test]
    fn decode_of_encode_recovers_the_original_direction_for_diagonal_vectors() {
        for v in [
            Vec3::new(1.0, 1.0, 1.0).normalize(),
            Vec3::new(1.0, -1.0, 1.0).normalize(),
            Vec3::new(-1.0, 1.0, -1.0).normalize(),
            Vec3::new(0.6614, 0.0, 0.75).normalize(),
            Vec3::new(-0.2044, 0.6285, 0.75).normalize(),
        ] {
            let uv = octahedral_encode(v);
            let decoded = octahedral_decode(uv);
            assert_close(decoded, v, 1e-4, "diagonal round trip failed for {v:?}");
        }
    }

    #[test]
    fn encode_of_decode_recovers_the_original_uv_away_from_the_degenerate_corners() {
        // The unit square's four CORNERS (0,0)/(0,1)/(1,0)/(1,1) are a
        // genuine many-to-one degenerate region of the octahedral
        // mapping itself, not a bug in this port: every corner decodes
        // to a "back face" direction with z<=0 exactly on the fold seam,
        // and multiple distinct corner UVs decode to the SAME direction
        // (e.g. all four corners fold onto -Z-ish directions along the
        // seam) — so encode(decode(corner)) is not expected to recover
        // that exact corner. Every INTERIOR/non-corner UV (including
        // points on an edge but not a corner) round-trips exactly;
        // tested here away from the four corner points specifically.
        for uv in [
            Vec2::new(0.5, 0.5),
            Vec2::new(0.25, 0.75),
            Vec2::new(0.9, 0.1),
            Vec2::new(0.0, 0.5),
            Vec2::new(0.5, 0.0),
            Vec2::new(1.0, 0.5),
            Vec2::new(0.5, 1.0),
        ] {
            let v = octahedral_decode(uv);
            let re_encoded = octahedral_encode(v);
            assert!(
                (re_encoded - uv).length() < 1e-3,
                "encode(decode(uv)) should recover uv: uv={uv:?} decoded={v:?} re_encoded={re_encoded:?}"
            );
        }
    }

    #[test]
    fn decoded_vectors_are_always_unit_length() {
        for uv in [Vec2::new(0.3, 0.8), Vec2::new(0.05, 0.95), Vec2::new(0.5, 0.5), Vec2::new(0.99, 0.01)] {
            let v = octahedral_decode(uv);
            assert!((v.length() - 1.0).abs() < 1e-4, "decoded vector must be unit length: uv={uv:?} v={v:?} len={}", v.length());
        }
    }

    // -----------------------------------------------------------------
    // direction_to_texel / texel_to_direction
    // -----------------------------------------------------------------

    #[test]
    fn texel_to_direction_round_trips_through_direction_to_texel_for_interior_texels() {
        // Corner texels hit the same many-to-one degeneracy
        // encode_of_decode's own test documents — excluded here for the
        // identical reason, tested at texel granularity instead of UV
        // granularity.
        const TILE_SIZE: u32 = 8;
        for y in 1..TILE_SIZE - 1 {
            for x in 1..TILE_SIZE - 1 {
                let dir = texel_to_direction(x, y, TILE_SIZE);
                let (rx, ry) = direction_to_texel(dir, TILE_SIZE);
                assert_eq!(
                    (rx, ry),
                    (x, y),
                    "texel ({x},{y}) -> direction {dir:?} -> texel ({rx},{ry}) should round-trip exactly"
                );
            }
        }
    }

    #[test]
    fn direction_to_texel_never_returns_an_out_of_range_index() {
        const TILE_SIZE: u32 = 6;
        for v in [
            Vec3::X, Vec3::Y, Vec3::Z, -Vec3::X, -Vec3::Y, -Vec3::Z,
            Vec3::new(1.0, 1.0, 1.0).normalize(),
            Vec3::new(-1.0, -1.0, -1.0).normalize(),
        ] {
            let (x, y) = direction_to_texel(v, TILE_SIZE);
            assert!(x < TILE_SIZE && y < TILE_SIZE, "texel ({x},{y}) out of range for tile_size={TILE_SIZE}, direction={v:?}");
        }
    }

    // -----------------------------------------------------------------
    // ddgi_probe_relight_start
    // -----------------------------------------------------------------

    #[test]
    fn full_probes_per_frame_always_starts_at_zero_regardless_of_frame_index() {
        let total_probes = 12u32;
        for frame_index in 0..20u32 {
            assert_eq!(
                ddgi_probe_relight_start(total_probes, total_probes, frame_index),
                0,
                "at probes_per_frame == total_probes, the whole grid is always visited starting at 0, \
                 regardless of frame_index={frame_index}"
            );
        }
    }

    #[test]
    fn single_probe_per_frame_visits_a_different_probe_each_frame_and_cycles() {
        let total_probes = 5u32;
        let starts: Vec<usize> = (0..5).map(|f| ddgi_probe_relight_start(1, total_probes, f)).collect();
        assert_eq!(starts, vec![0, 1, 2, 3, 4]);
        assert_eq!(
            ddgi_probe_relight_start(1, total_probes, 5),
            ddgi_probe_relight_start(1, total_probes, 0),
            "the schedule must repeat exactly every total_probes frames"
        );
    }

    #[test]
    fn every_probe_is_relit_at_least_once_within_a_bounded_window_for_every_probes_per_frame() {
        let total_probes = 12u32;
        for probes_per_frame in 1..=total_probes {
            let window = total_probes.div_ceil(probes_per_frame);
            let mut visited = vec![false; total_probes as usize];
            for frame_index in 0..window {
                let start = ddgi_probe_relight_start(probes_per_frame, total_probes, frame_index);
                for i in 0..probes_per_frame as usize {
                    visited[(start + i) % total_probes as usize] = true;
                }
            }
            assert!(
                visited.iter().all(|&v| v),
                "probes_per_frame={probes_per_frame} should relight every probe within {window} frames, got {visited:?}"
            );
        }
    }

    // -----------------------------------------------------------------
    // Grid-scale sanity check, mirroring INDIRECT_MAX_T's own established
    // precedent (cpu_ref.rs's own "checked, not assumed" scale-safety
    // convention): confirm probe spacing produces a real, honestly-
    // reported probe count at the SAME --stress 10000 scale prior GI
    // stages were checked against, not assumed fine.
    // -----------------------------------------------------------------

    #[test]
    fn debug_stress_10000_probe_grid_scale_profile() {
        // Mirrors cpu_ref.rs::tests::stress_n_scene's own CELL_SIZE
        // (4.0 half-extent * 2.0 + 3.0 gap = 11.0) and DIM=100 for
        // --stress 10000 (100*100 = 10000 objects) — reconstructed here
        // rather than reusing that private fixture (see ground_and_box's
        // own doc comment for why), computing only the SCENE BOUNDS a
        // real --stress 10000 run would produce, not the full object
        // array (this test is about grid sizing, not ray tracing against
        // it — a full stress_n_scene-shaped BVH build at dim=100 is real,
        // measurable setup cost every prior stress test already pays;
        // this test avoids paying it again for a question that doesn't
        // need it).
        const DIM: usize = 100;
        const CELL_SIZE: f32 = 11.0;
        let half_extent = (DIM as f32 - 1.0) * 0.5 * CELL_SIZE + 4.0; // +4.0: half a cell's own ground-plate half-extent
        let bounds = Aabb {
            min: Vec3::new(-half_extent, 0.0, -half_extent),
            max: Vec3::new(half_extent, 2.4, half_extent), // 2.4: tallest object height in this scene family
        };

        let spacing = Vec3::splat(CELL_SIZE);
        let vertical_layers = 3;
        let grid = probe_grid_from_bounds(bounds, spacing, vertical_layers);
        let probe_count = grid.probe_count();

        println!(
            "stress_10000 probe grid profile: bounds {bounds:?}, dims {:?}, probe_count {probe_count}",
            grid.dims
        );

        // No hard pass/fail threshold on the count itself — the point of
        // this test is to make the real number visible and reviewable
        // (matching this project's own "report the real number honestly,
        // don't silently assume it's fine" convention already
        // established for INDIRECT_MAX_T and Stage C's own occupancy-
        // grid cell-size sweep) — only a sanity bound against a truly
        // pathological blow-up (e.g. an off-by-orders-of-magnitude
        // spacing bug producing millions of probes).
        assert!(
            probe_count < 1_000_000,
            "probe_count={probe_count} at --stress 10000 scale is pathologically large — \
             check spacing/vertical_layers before this ships"
        );
        assert!(probe_count > 0, "test sanity: expected a nonzero probe grid at real scene scale");
    }

    // -----------------------------------------------------------------
    // Multi-frame sealed-room light-leak reproduction (2026-09-18) — a
    // real user-reported DDGI light leak in `examples/gi_room.rs` (a
    // fully sealed, sun-only room reads as lit even before the roof ever
    // opens). Two separate hypotheses (a self-referential feedback loop
    // bootstrapping from float noise, and a thin-slab shadow-march
    // divergence bug) were investigated and NOT reproduced by isolated
    // single-frame/single-panel tests (see `cpu_ref.rs`'s own thin-slab
    // and full-room shadow-march tests, both passing). This is the next,
    // more expensive step: a REAL multi-frame simulation of the actual
    // relight+temporal-blend loop (not just one relight call), against
    // the real sealed-room geometry, starting from a true zero-history
    // atlas exactly like a fresh scene load — if this ALSO stays at
    // zero, the leak isn't reproducible from first principles at all and
    // must be a GPU-only concern (buffer binding/layout mismatch, a real
    // uninitialized-memory issue despite the zero-fill call, or a WGSL
    // port divergence from this CPU reference); if it drifts upward,
    // this proves and quantifies the feedback-loop theory the first
    // investigation proposed but couldn't fully reconcile with a
    // from-zero starting point.
    // -----------------------------------------------------------------

    /// `examples/gi_room.rs::spawn_room` verbatim (fully sealed, roof
    /// closed) — reconstructed here rather than reused across module
    /// boundaries per this file's own established test-fixture
    /// convention (see `ground_and_box`'s own doc comment).
    fn sealed_gi_room_shell() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        const HALF_X: f32 = 8.0;
        const HALF_Y: f32 = 3.0;
        const HALF_Z: f32 = 6.5;
        const WALL_THICKNESS: f32 = 0.3;
        const WALL_OVERLAP: f32 = 0.2;

        let e = entities(6);
        let white_wall = Material::new(Vec3::splat(0.92), 0.0, 0.35);
        let mut objects = Vec::new();

        objects.push(TraceObject {
            entity: e[0],
            shape: Shape::RoundedBox { half_extents: Vec3::new(HALF_X + WALL_OVERLAP, WALL_THICKNESS, HALF_Z + WALL_OVERLAP), corner_radius: 0.0 },
            translation: Vec3::new(0.0, -HALF_Y - WALL_THICKNESS, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        let side_wall_half = Vec3::new(WALL_THICKNESS, HALF_Y + WALL_OVERLAP, HALF_Z);
        objects.push(TraceObject {
            entity: e[1],
            shape: Shape::RoundedBox { half_extents: side_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(HALF_X + WALL_THICKNESS, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        objects.push(TraceObject {
            entity: e[2],
            shape: Shape::RoundedBox { half_extents: side_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(-HALF_X - WALL_THICKNESS, 0.0, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        let end_wall_half = Vec3::new(HALF_X + WALL_OVERLAP, HALF_Y + WALL_OVERLAP, WALL_THICKNESS);
        objects.push(TraceObject {
            entity: e[3],
            shape: Shape::RoundedBox { half_extents: end_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(0.0, 0.0, HALF_Z + WALL_THICKNESS),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        objects.push(TraceObject {
            entity: e[4],
            shape: Shape::RoundedBox { half_extents: end_wall_half, corner_radius: 0.0 },
            translation: Vec3::new(0.0, 0.0, -HALF_Z - WALL_THICKNESS),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });
        objects.push(TraceObject {
            entity: e[5],
            shape: Shape::RoundedBox { half_extents: Vec3::new(HALF_X + WALL_OVERLAP, WALL_THICKNESS, HALF_Z + WALL_OVERLAP), corner_radius: 0.0 },
            translation: Vec3::new(0.0, HALF_Y + WALL_THICKNESS, 0.0),
            rotation: Quat::IDENTITY,
            material: white_wall,
        });

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

    /// `sealed_gi_room_shell` PLUS `examples/gi_room.rs::spawn_cubes`'s
    /// own real 7-cube furniture (all real positions/materials, verbatim
    /// — red/blue cluster, gold/dark-base stack, white/purple pair,
    /// green cube, and the clear glass cube) — the earlier, furniture-
    /// less multi-frame simulation stayed exactly dark; this fixture
    /// checks whether the REAL cube geometry (reflectance, metallic,
    /// transmission materials the shell-only fixture never exercised)
    /// changes that result.
    fn sealed_gi_room_shell_with_cubes() -> (Vec<Entity>, Vec<TraceObject>, Bvh) {
        let (mut e, mut objects, _bvh) = sealed_gi_room_shell();
        let floor_y = -3.0f32; // -ROOM_HALF_Y

        let cube_entities = entities(7);
        let mut push_cube = |entity: Entity, center: Vec3, half_extent: f32, material: Material| {
            objects.push(TraceObject { entity, shape: Shape::RoundedBox { half_extents: Vec3::splat(half_extent), corner_radius: 0.02 }, translation: center, rotation: Quat::IDENTITY, material });
        };

        let red = Material::new(Vec3::new(0.75, 0.2, 0.15), 0.0, 0.6);
        let blue = Material::new(Vec3::new(0.2, 0.35, 0.8), 0.1, 0.2).with_reflectance(0.7);
        push_cube(cube_entities[0], Vec3::new(-6.8, floor_y + 0.9, -4.5), 0.9, red);
        push_cube(cube_entities[1], Vec3::new(-5.2, floor_y + 0.5, -4.7), 0.5, blue);

        let gold = Material::new(Vec3::new(0.85, 0.65, 0.2), 0.9, 0.25).with_reflectance(0.9);
        let dark_base = Material::new(Vec3::splat(0.08), 0.0, 0.7);
        push_cube(cube_entities[2], Vec3::new(-6.5, floor_y + 0.6, -1.5), 0.6, dark_base);
        push_cube(cube_entities[3], Vec3::new(-6.5, floor_y + 1.5, -1.5), 0.3, gold);

        let white_cube = Material::new(Vec3::splat(0.85), 0.0, 0.4).with_reflectance(0.6);
        push_cube(cube_entities[4], Vec3::new(-6.5, floor_y + 0.5, 2.0), 0.5, white_cube);
        let purple = Material::new(Vec3::new(0.5, 0.25, 0.6), 0.2, 0.45);
        push_cube(cube_entities[5], Vec3::new(-4.0, floor_y + 1.0, 2.2), 1.0, purple);

        let green = Material::new(Vec3::new(0.25, 0.7, 0.3), 0.0, 0.5);
        push_cube(cube_entities[6], Vec3::new(-6.5, floor_y + 0.7, 4.8), 0.7, green);

        let glass_entity = entities(1)[0];
        e.extend(&cube_entities);
        e.push(glass_entity);
        objects.push(TraceObject {
            entity: glass_entity,
            shape: Shape::RoundedBox { half_extents: Vec3::splat(1.0), corner_radius: 0.0 },
            translation: Vec3::new(2.0, floor_y + 1.0, 0.0),
            rotation: Quat::IDENTITY,
            material: Material::new(Vec3::ONE, 0.0, 0.02).with_reflectance(0.9).with_transmission(1.0).with_ior(1.5),
        });

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

    /// The real multi-frame simulation: a small (3x2x3 = 18 probe) grid,
    /// `tile_size=4` (16 rays/probe — enough for real hemisphere
    /// coverage, small enough to keep this test fast), iterated for 20
    /// full relight passes (every probe/texel relit every frame — no
    /// `probes_per_frame` partial-coverage rotation, since Stage-1
    /// investigation found `gi_room`'s real grid covers in ~3 frames
    /// anyway, so full coverage every frame is the same regime sooner).
    /// Starts every texel at EXACT zero history (irradiance, distance
    /// moments, history_length all zero) — a real fresh-scene-load state
    /// — and tracks the grid's own peak irradiance magnitude after each
    /// frame. In a genuinely sealed, sun-only room, this must stay at
    /// (approximately) zero forever; any real upward drift is the leak,
    /// reproduced and quantified.
    /// Shared simulation body for both sealed-room DDGI tests below —
    /// extracted so the exact same relight/blend logic can be run
    /// against the shell-only fixture AND the shell-plus-real-cube-
    /// furniture fixture without copy-pasting ~100 lines twice. Returns
    /// the grid's own peak irradiance magnitude after each of 20 full
    /// relight frames.
    fn simulate_ddgi_grid_peak_per_frame(objects: &[TraceObject], bvh: &Bvh, lights: &[Light]) -> Vec<f32> {
        simulate_ddgi_grid_peak_per_frame_with_config(objects, bvh, lights, Vec3::splat(3.0), 2, 4)
    }

    /// Same simulation as `simulate_ddgi_grid_peak_per_frame`, but with the
    /// grid's own `spacing`/`vertical_layers`/`tile_size` as real
    /// parameters instead of hardcoded to this file's own original
    /// (coarse, `spacing=3.0`/`vertical_layers=2`/`tile_size=4`) test
    /// fixture values — added to check whether `examples/gi_room.rs`'s own
    /// PRODUCTION `DdgiConfig` (`probe_spacing: 1.2, vertical_layers: 6,
    /// tile_size: 8` — a much denser grid, ~1000 probes vs. this file's
    /// original 18-probe fixture) is a genuinely different regime the
    /// original coarse test never actually exercised, despite reusing the
    /// same room shell/furniture. See
    /// `sealed_room_with_real_cube_furniture_at_production_ddgi_density_
    /// stays_dark` for why this distinction matters.
    fn simulate_ddgi_grid_peak_per_frame_with_config(
        objects: &[TraceObject],
        bvh: &Bvh,
        lights: &[Light],
        spacing: Vec3,
        vertical_layers: u32,
        tile_size: u32,
    ) -> Vec<f32> {
        const HALF_X: f32 = 8.0;
        const HALF_Y: f32 = 3.0;
        const HALF_Z: f32 = 6.5;
        let tile_size = tile_size.max(1);
        const MAX_HISTORY: f32 = 24.0;
        const MAX_T: f32 = 60.0;

        let bounds = Aabb { min: Vec3::new(-HALF_X + 1.0, -HALF_Y + 0.5, -HALF_Z + 1.0), max: Vec3::new(HALF_X - 1.0, HALF_Y - 0.5, HALF_Z - 1.0) };
        let grid = probe_grid_from_bounds(bounds, spacing, vertical_layers);
        let probe_count = grid.probe_count();
        let texels_per_probe = (tile_size * tile_size) as usize;

        // Flat per-(probe, texel) storage — irradiance, distance moments,
        // history_length — mirroring the real atlas/distance_atlas/
        // history_length buffer triple, all zero-initialized (matching
        // `prepare_hybrid_ddgi`'s own real zero-fill).
        let mut irradiance = vec![Vec3::ZERO; probe_count * texels_per_probe];
        let mut mean_distance = vec![0.0f32; probe_count * texels_per_probe];
        let mut mean_distance_sq = vec![0.0f32; probe_count * texels_per_probe];
        let mut history_length = vec![0.0f32; probe_count * texels_per_probe];

        let texel_index = |probe_flat: usize, tx: u32, ty: u32| -> usize { probe_flat * texels_per_probe + (ty * tile_size + tx) as usize };

        let mut peak_per_frame = Vec::new();
        for _frame in 0..20 {
            // Snapshot the PREVIOUS frame's atlas for reads (the real
            // pass has a same-frame read/write race per probe order —
            // reading last frame's fully-settled state is the safer,
            // MORE convergent case, not less, so if even this doesn't
            // stay dark, the same-frame race isn't what's saving it in
            // production either).
            let read_irradiance = irradiance.clone();
            let read_mean_distance = mean_distance.clone();
            let read_mean_distance_sq = mean_distance_sq.clone();

            let probe_irradiance = |coords: UVec3, dir: Vec3| -> Vec3 {
                let flat = coords.x + coords.y * grid.dims.x + coords.z * grid.dims.x * grid.dims.y;
                if flat as usize >= probe_count {
                    return Vec3::ZERO;
                }
                let (tx, ty) = direction_to_texel(dir, tile_size);
                read_irradiance[texel_index(flat as usize, tx, ty)]
            };
            let probe_distance_moments = |coords: UVec3, dir: Vec3| -> (f32, f32) {
                let flat = coords.x + coords.y * grid.dims.x + coords.z * grid.dims.x * grid.dims.y;
                if flat as usize >= probe_count {
                    return (f32::MAX, f32::MAX);
                }
                let (tx, ty) = direction_to_texel(dir, tile_size);
                let idx = texel_index(flat as usize, tx, ty);
                (read_mean_distance[idx], read_mean_distance_sq[idx])
            };

            let mut new_irradiance = irradiance.clone();
            let mut new_mean_distance = mean_distance.clone();
            let mut new_mean_distance_sq = mean_distance_sq.clone();
            let mut new_history_length = history_length.clone();

            for pz in 0..grid.dims.z {
                for py in 0..grid.dims.y {
                    for px in 0..grid.dims.x {
                        let flat = (px + py * grid.dims.x + pz * grid.dims.x * grid.dims.y) as usize;
                        let probe_pos = grid.probe_position(px, py, pz);
                        for ty in 0..tile_size {
                            for tx in 0..tile_size {
                                let idx = texel_index(flat, tx, ty);
                                let indirect_at_hit = |hit_point: Vec3, hit_normal: Vec3| -> Vec3 {
                                    sample_probe_grid(&grid, bvh, objects, hit_point, hit_normal, 1.0, None, probe_irradiance, probe_distance_moments)
                                };
                                let relit = relight_probe_texel(
                                    bvh,
                                    objects,
                                    lights,
                                    probe_pos,
                                    (tx, ty),
                                    tile_size,
                                    MAX_T,
                                    irradiance[idx],
                                    mean_distance[idx],
                                    mean_distance_sq[idx],
                                    history_length[idx],
                                    MAX_HISTORY,
                                    indirect_at_hit,
                                );
                                new_irradiance[idx] = relit.irradiance;
                                new_mean_distance[idx] = relit.mean_distance;
                                new_mean_distance_sq[idx] = relit.mean_distance_squared;
                                new_history_length[idx] = relit.history_length;
                            }
                        }
                    }
                }
            }
            irradiance = new_irradiance;
            mean_distance = new_mean_distance;
            mean_distance_sq = new_mean_distance_sq;
            history_length = new_history_length;

            let peak = irradiance.iter().map(|c| c.length()).fold(0.0f32, f32::max);
            peak_per_frame.push(peak);
        }
        peak_per_frame
    }

    /// The REAL relight schedule — `ddgi_probe_relight_start`'s own
    /// rotating `probes_per_frame`-sized subset (see this file's own
    /// module doc comment, decision 3), NOT
    /// `simulate_ddgi_grid_peak_per_frame`'s "relight the whole grid every
    /// frame" simplification. Added alongside
    /// `simulate_ddgi_grid_peak_per_frame_with_config` while investigating
    /// a live, confirmed GPU-only sealed-room leak that neither the
    /// original coarse-grid test nor the whole-grid-every-frame
    /// production-density test (`sealed_room_with_real_cube_furniture_at_
    /// production_ddgi_density_stays_dark`) could reproduce — this closes
    /// the LAST remaining gap between the CPU simulation and
    /// `examples/gi_room.rs`'s own real per-frame dispatch shape
    /// (`probes_per_frame=512` out of ~1000 total probes, not full
    /// coverage). Runs `frame_count` frames (not a fixed 20) so the exact
    /// real repro window (`--at-frame 60`, i.e. 60 real relight dispatches
    /// from a cold start) can be reproduced bit-for-bit.
    #[allow(clippy::too_many_arguments)]
    fn simulate_ddgi_grid_peak_per_frame_with_rotation(
        objects: &[TraceObject],
        bvh: &Bvh,
        lights: &[Light],
        spacing: Vec3,
        vertical_layers: u32,
        tile_size: u32,
        probes_per_frame: u32,
        frame_count: u32,
    ) -> Vec<f32> {
        const HALF_X: f32 = 8.0;
        const HALF_Y: f32 = 3.0;
        const HALF_Z: f32 = 6.5;
        let tile_size = tile_size.max(1);
        const MAX_HISTORY: f32 = 24.0;
        const MAX_T: f32 = 28.0; // examples/gi_room.rs's own DdgiConfig::max_t, not this file's original 60.0 fixture value.

        let bounds = Aabb { min: Vec3::new(-HALF_X + 1.0, -HALF_Y + 0.5, -HALF_Z + 1.0), max: Vec3::new(HALF_X - 1.0, HALF_Y - 0.5, HALF_Z - 1.0) };
        let grid = probe_grid_from_bounds(bounds, spacing, vertical_layers);
        let probe_count = grid.probe_count() as u32;
        let probes_per_frame = probes_per_frame.clamp(1, probe_count);
        let texels_per_probe = (tile_size * tile_size) as usize;

        let mut irradiance = vec![Vec3::ZERO; probe_count as usize * texels_per_probe];
        let mut mean_distance = vec![0.0f32; probe_count as usize * texels_per_probe];
        let mut mean_distance_sq = vec![0.0f32; probe_count as usize * texels_per_probe];
        let mut history_length = vec![0.0f32; probe_count as usize * texels_per_probe];

        let texel_index = |probe_flat: u32, tx: u32, ty: u32| -> usize { probe_flat as usize * texels_per_probe + (ty * tile_size + tx) as usize };

        let mut peak_per_frame = Vec::new();
        for frame_index in 0..frame_count {
            // Same same-frame read/write race modeling as
            // simulate_ddgi_grid_peak_per_frame_with_config: snapshot the
            // PREVIOUS frame's fully-settled atlas for this frame's own
            // indirect_at_hit reads (the more-convergent, not less,
            // approximation of the real GPU's own in-place read_write
            // race — see that function's own doc comment).
            let read_irradiance = irradiance.clone();
            let read_mean_distance = mean_distance.clone();
            let read_mean_distance_sq = mean_distance_sq.clone();

            let probe_irradiance = |coords: UVec3, dir: Vec3| -> Vec3 {
                let flat = coords.x + coords.y * grid.dims.x + coords.z * grid.dims.x * grid.dims.y;
                if flat >= probe_count {
                    return Vec3::ZERO;
                }
                let (tx, ty) = direction_to_texel(dir, tile_size);
                read_irradiance[texel_index(flat, tx, ty)]
            };
            let probe_distance_moments = |coords: UVec3, dir: Vec3| -> (f32, f32) {
                let flat = coords.x + coords.y * grid.dims.x + coords.z * grid.dims.x * grid.dims.y;
                if flat >= probe_count {
                    return (f32::MAX, f32::MAX);
                }
                let (tx, ty) = direction_to_texel(dir, tile_size);
                let idx = texel_index(flat, tx, ty);
                (read_mean_distance[idx], read_mean_distance_sq[idx])
            };

            // ddgi_ref::ddgi_probe_relight_start verbatim scheduling —
            // this frame's own rotating subset, exactly what
            // ddgi_relight_main's real GPU dispatch computes.
            let start = ddgi_probe_relight_start(probes_per_frame, probe_count, frame_index) as u32;
            for slot in 0..probes_per_frame {
                let probe_flat = (start + slot) % probe_count;
                let plane = (grid.dims.x * grid.dims.y).max(1);
                let pz = probe_flat / plane;
                let rem = probe_flat % plane;
                let py = rem / grid.dims.x.max(1);
                let px = rem % grid.dims.x.max(1);
                let probe_pos = grid.probe_position(px, py, pz);
                for ty in 0..tile_size {
                    for tx in 0..tile_size {
                        let idx = texel_index(probe_flat, tx, ty);
                        let indirect_at_hit = |hit_point: Vec3, hit_normal: Vec3| -> Vec3 {
                            sample_probe_grid(&grid, bvh, objects, hit_point, hit_normal, 1.0, None, probe_irradiance, probe_distance_moments)
                        };
                        let relit = relight_probe_texel(
                            bvh,
                            objects,
                            lights,
                            probe_pos,
                            (tx, ty),
                            tile_size,
                            MAX_T,
                            irradiance[idx],
                            mean_distance[idx],
                            mean_distance_sq[idx],
                            history_length[idx],
                            MAX_HISTORY,
                            indirect_at_hit,
                        );
                        irradiance[idx] = relit.irradiance;
                        mean_distance[idx] = relit.mean_distance;
                        mean_distance_sq[idx] = relit.mean_distance_squared;
                        history_length[idx] = relit.history_length;
                    }
                }
            }

            let peak = irradiance.iter().map(|c| c.length()).fold(0.0f32, f32::max);
            peak_per_frame.push(peak);
        }
        peak_per_frame
    }

    #[test]
    fn sealed_room_with_real_cube_furniture_at_production_rotation_schedule_stays_dark() {
        let (_e, objects, bvh) = sealed_gi_room_shell_with_cubes();
        let lights = [overhead_sun_at_35_degrees()];
        // Verbatim examples/gi_room.rs::DdgiConfig (probe_spacing=1.2,
        // vertical_layers=6, tile_size=8, probes_per_frame=512), run for
        // 90 frames — comfortably past the real repro's own `--at-frame
        // 60` capture point.
        let peak_per_frame = simulate_ddgi_grid_peak_per_frame_with_rotation(&objects, &bvh, &lights, Vec3::splat(1.2), 6, 8, 512, 90);

        println!("sealed-room (production density + REAL rotation schedule) DDGI peak irradiance per frame: {peak_per_frame:?}");
        let final_peak = *peak_per_frame.last().unwrap();
        assert!(
            final_peak < 0.01,
            "a fully sealed, sun-only room's own DDGI grid, at examples/gi_room.rs's OWN production probe \
             density AND real probes_per_frame rotation schedule, must stay (near-)exactly dark after 90 \
             relight frames from zero history — got peak irradiance {final_peak} (trajectory: {peak_per_frame:?})."
        );
    }

    #[test]
    fn sealed_room_ddgi_grid_stays_dark_across_many_relight_frames_from_zero_history() {
        let (_e, objects, bvh) = sealed_gi_room_shell();
        let lights = [overhead_sun_at_35_degrees()];
        let peak_per_frame = simulate_ddgi_grid_peak_per_frame(&objects, &bvh, &lights);

        println!("sealed-room (shell only) DDGI peak irradiance per frame: {peak_per_frame:?}");
        let final_peak = *peak_per_frame.last().unwrap();
        assert!(
            final_peak < 0.01,
            "a fully sealed, sun-only room's own DDGI grid must stay (near-)exactly dark after 20 relight \
             frames from zero history — got peak irradiance {final_peak} (trajectory: {peak_per_frame:?}). \
             A nonzero, non-decaying peak here reproduces the real gi_room light leak from first principles."
        );
    }

    /// The furniture-aware sibling of the test above — added 2026-09-18
    /// after the user reported DDGI/Radiance Cascades STILL reading as
    /// lit in the sealed room even after `trace_shadow`'s own
    /// margin_fade/VIS_CUTOFF bug was fixed (the shell-only simulation
    /// above already proved that fix sufficient for a bare room; this
    /// checks whether the REAL cube furniture — reflectance, metallic,
    /// and especially the clear glass cube's own transmission material,
    /// none of which the shell-only fixture ever exercised — reveals a
    /// further, furniture-specific gap).
    #[test]
    fn sealed_room_with_real_cube_furniture_ddgi_grid_stays_dark_across_many_relight_frames() {
        let (_e, objects, bvh) = sealed_gi_room_shell_with_cubes();
        let lights = [overhead_sun_at_35_degrees()];
        let peak_per_frame = simulate_ddgi_grid_peak_per_frame(&objects, &bvh, &lights);

        println!("sealed-room (with real cube furniture) DDGI peak irradiance per frame: {peak_per_frame:?}");
        let final_peak = *peak_per_frame.last().unwrap();
        assert!(
            final_peak < 0.01,
            "a fully sealed, sun-only room's own DDGI grid, WITH the real gi_room cube furniture present, \
             must stay (near-)exactly dark after 20 relight frames from zero history — got peak irradiance \
             {final_peak} (trajectory: {peak_per_frame:?})."
        );
    }

    /// A helper shared by the two tests below: runs ONE full relight pass
    /// (frame 1, zero starting history, matching the real live GPU repro
    /// — see the module-level bug writeup on the two tests below) over
    /// `bounds` at `gi_room`'s own production density, returning the
    /// peak irradiance magnitude and the world position of the worst
    /// offending probe (for diagnosis).
    fn ddgi_frame_one_peak_and_worst_probe(objects: &[TraceObject], bvh: &Bvh, lights: &[Light], bounds: Aabb) -> (f32, Vec3) {
        let grid = probe_grid_from_bounds(bounds, Vec3::splat(1.2), 6);
        let tile_size = 8u32;
        let probe_count = grid.probe_count();
        let texels_per_probe = (tile_size * tile_size) as usize;
        let irradiance = vec![Vec3::ZERO; probe_count * texels_per_probe];
        let mean_distance = vec![0.0f32; probe_count * texels_per_probe];
        let mean_distance_sq = vec![0.0f32; probe_count * texels_per_probe];
        let history_length = vec![0.0f32; probe_count * texels_per_probe];
        let texel_index = |probe_flat: usize, tx: u32, ty: u32| -> usize { probe_flat * texels_per_probe + (ty * tile_size + tx) as usize };
        const MAX_T: f32 = 60.0;
        const MAX_HISTORY: f32 = 24.0;

        let read_irradiance = irradiance.clone();
        let read_mean_distance = mean_distance.clone();
        let read_mean_distance_sq = mean_distance_sq.clone();
        let probe_irradiance = |coords: UVec3, dir: Vec3| -> Vec3 {
            let flat = coords.x + coords.y * grid.dims.x + coords.z * grid.dims.x * grid.dims.y;
            if flat as usize >= probe_count {
                return Vec3::ZERO;
            }
            let (tx, ty) = direction_to_texel(dir, tile_size);
            read_irradiance[texel_index(flat as usize, tx, ty)]
        };
        let probe_distance_moments = |coords: UVec3, dir: Vec3| -> (f32, f32) {
            let flat = coords.x + coords.y * grid.dims.x + coords.z * grid.dims.x * grid.dims.y;
            if flat as usize >= probe_count {
                return (f32::MAX, f32::MAX);
            }
            let (tx, ty) = direction_to_texel(dir, tile_size);
            let idx = texel_index(flat as usize, tx, ty);
            (read_mean_distance[idx], read_mean_distance_sq[idx])
        };

        let mut new_irradiance = irradiance.clone();
        for pz in 0..grid.dims.z {
            for py in 0..grid.dims.y {
                for px in 0..grid.dims.x {
                    let flat = (px + py * grid.dims.x + pz * grid.dims.x * grid.dims.y) as usize;
                    let probe_pos = grid.probe_position(px, py, pz);
                    for ty in 0..tile_size {
                        for tx in 0..tile_size {
                            let idx = texel_index(flat, tx, ty);
                            let indirect_at_hit = |hit_point: Vec3, hit_normal: Vec3| -> Vec3 {
                                sample_probe_grid(&grid, bvh, objects, hit_point, hit_normal, 1.0, None, probe_irradiance, probe_distance_moments)
                            };
                            let relit = relight_probe_texel(
                                bvh,
                                objects,
                                lights,
                                probe_pos,
                                (tx, ty),
                                tile_size,
                                MAX_T,
                                irradiance[idx],
                                mean_distance[idx],
                                mean_distance_sq[idx],
                                history_length[idx],
                                MAX_HISTORY,
                                indirect_at_hit,
                            );
                            new_irradiance[idx] = relit.irradiance;
                        }
                    }
                }
            }
        }

        let mut worst = 0.0f32;
        let mut worst_probe = UVec3::ZERO;
        for pz in 0..grid.dims.z {
            for py in 0..grid.dims.y {
                for px in 0..grid.dims.x {
                    let flat = (px + py * grid.dims.x + pz * grid.dims.x * grid.dims.y) as usize;
                    for ty in 0..tile_size {
                        for tx in 0..tile_size {
                            let idx = texel_index(flat, tx, ty);
                            let mag = new_irradiance[idx].length();
                            if mag > worst {
                                worst = mag;
                                worst_probe = UVec3::new(px, py, pz);
                            }
                        }
                    }
                }
            }
        }
        (worst, grid.probe_position(worst_probe.x, worst_probe.y, worst_probe.z))
    }

    /// **Real bug, fixed 2026-09-19**: `extract_hybrid_scene`
    /// (`extract.rs`) fed the scene's own REAL, UNSHRUNK root BVH AABB
    /// straight into `probe_grid_from_bounds` — this is the union of
    /// every object's own bounds, including the OUTWARD-facing surface
    /// of `examples/gi_room.rs`'s own solid wall shell (walls extend
    /// outward from the room's `ROOM_HALF_{X,Y,Z}` interior half-extents
    /// by `WALL_THICKNESS=0.3` + `WALL_OVERLAP=0.2` on non-normal axes),
    /// so the real BVH root AABB is `(±8.6, ±3.6, ±7.1)`, not the room's
    /// own interior `(±8.0, ±3.0, ±6.5)`. `probe_grid_from_bounds`'s own
    /// half-cell (`spacing/2`) inset exists ONLY to center probe 0 inside
    /// its own cell (see that function's doc comment) — it is NOT a
    /// geometric safety margin against enclosing geometry, and at this
    /// scene's own production `probe_spacing=1.2`, the half-cell inset
    /// (0.6) very nearly equals the wall thickness, landing the outermost
    /// probe LAYER almost exactly on the interior wall plane. A corner
    /// probe (extreme layer on 2-3 axes at once) lands ON or beyond the
    /// wall's own outer surface, fully outside the sealed cavity — from
    /// there, `probe_ray`s fired from that probe skim the wall's own
    /// surface at grazing incidence or escape through wall-panel seam
    /// gaps entirely, picking up direct sunlight a probe safely inside
    /// the cavity would never see. This is a REAL, structured, colored
    /// light leak, present on the very FIRST relight dispatch (zero
    /// starting history, full-grid relight) — this test proves the bug
    /// reproduces by feeding the real (unshrunk) room AABB in directly,
    /// matching what production actually did before the fix below.
    /// **Kept permanently** (not deleted after the fix) as a regression
    /// proof of the actual root cause, mirroring this file's own
    /// convention of keeping ruled-out-hypothesis tests around.
    #[test]
    fn probe_grid_from_unshrunk_real_room_bvh_bounds_leaks_light_from_frame_one() {
        let (_e, objects, bvh) = sealed_gi_room_shell_with_cubes();
        let lights = [overhead_sun_at_35_degrees()];
        const WALL_THICKNESS: f32 = 0.3;
        let outer_half_x = 8.0 + 2.0 * WALL_THICKNESS;
        let outer_half_y = 3.0 + 2.0 * WALL_THICKNESS;
        let outer_half_z = 6.5 + 2.0 * WALL_THICKNESS;
        let bounds = Aabb {
            min: Vec3::new(-outer_half_x, -outer_half_y, -outer_half_z),
            max: Vec3::new(outer_half_x, outer_half_y, outer_half_z),
        };
        let (peak, worst_pos) = ddgi_frame_one_peak_and_worst_probe(&objects, &bvh, &lights, bounds);
        println!("unshrunk real BVH bounds: frame-1 peak irradiance = {peak} at worst probe {worst_pos:?}");
        assert!(
            peak > 0.1,
            "expected this test to REPRODUCE the real light leak (unshrunk bounds put probes on/outside \
             the wall surface) — got peak {peak}, which would mean the bug stopped reproducing here \
             (check probe_grid_from_bounds/sealed_gi_room_shell_with_cubes haven't changed shape)."
        );
    }

    /// The fix for the leak proven above: `extract_hybrid_scene` now
    /// shrinks `root_bounds` inward by `DDGI_GRID_WALL_SAFETY_MARGIN
    /// (0.75) + probe_spacing` on every axis before calling
    /// `probe_grid_from_bounds` — the `+ probe_spacing` term matters
    /// independently of wall thickness: `probe_grid_from_bounds`'s own
    /// `dims_x`/`dims_z` use `ceil(extent/spacing)`, so the grid's actual
    /// covered span almost always overshoots the input extent by up to
    /// one full `spacing` unit, and since `origin` is pinned to
    /// `bounds.min` (never re-centered), ALL of that overshoot lands on
    /// the max-axis side — a margin sized only for wall thickness gets
    /// silently eaten by this on the max side. This test applies that
    /// exact shrink formula (see `extract.rs`'s own comment at its real
    /// call site) to the real room's own outer BVH bounds and confirms
    /// the grid stays dark, closing the loop from "reproduces the bug"
    /// (test above) to "the shipped fix resolves it."
    #[test]
    fn probe_grid_from_extract_rs_shrink_formula_stays_dark() {
        let (_e, objects, bvh) = sealed_gi_room_shell_with_cubes();
        let lights = [overhead_sun_at_35_degrees()];
        const WALL_THICKNESS: f32 = 0.3;
        let outer_half_x = 8.0 + 2.0 * WALL_THICKNESS;
        let outer_half_y = 3.0 + 2.0 * WALL_THICKNESS;
        let outer_half_z = 6.5 + 2.0 * WALL_THICKNESS;
        let root_bounds = Aabb {
            min: Vec3::new(-outer_half_x, -outer_half_y, -outer_half_z),
            max: Vec3::new(outer_half_x, outer_half_y, outer_half_z),
        };
        // Verbatim extract_hybrid_scene shrink formula.
        const DDGI_GRID_WALL_SAFETY_MARGIN: f32 = 0.75;
        let probe_spacing = 1.2f32;
        let margin = DDGI_GRID_WALL_SAFETY_MARGIN + probe_spacing;
        let shrunk_bounds = Aabb { min: root_bounds.min + Vec3::splat(margin), max: root_bounds.max - Vec3::splat(margin) };

        let (peak, worst_pos) = ddgi_frame_one_peak_and_worst_probe(&objects, &bvh, &lights, shrunk_bounds);
        println!("extract.rs-shrunk real BVH bounds: frame-1 peak irradiance = {peak} at worst probe {worst_pos:?}");
        assert!(
            peak < 0.01,
            "a fully sealed, sun-only room's own DDGI grid, built from the REAL (wall-inclusive) BVH root \
             AABB shrunk by extract_hybrid_scene's own real safety-margin formula, must stay (near-)exactly \
             dark after 1 relight frame from zero history — got peak irradiance {peak} at probe {worst_pos:?} \
             (the fix's margin may need to grow if this regresses)."
        );
    }

    /// **The actual live `gi_room.rs` DDGI density**, not this file's own
    /// original coarse test fixture (`spacing=3.0, vertical_layers=2,
    /// tile_size=4`, an 18-probe/16-texel grid) — `examples/gi_room.rs`'s
    /// own `.insert_resource(DdgiConfig { probe_spacing: 1.2,
    /// vertical_layers: 6, tile_size: 8, .. })` produces a MUCH denser
    /// grid (~1000 probes, 64 texels/probe) that the two tests above never
    /// actually exercised despite sharing the same room shell/furniture —
    /// this test closes that gap. Found while investigating a live,
    /// confirmed GPU-only sealed-room DDGI leak that reproduces at
    /// `--gi-method ddgi` on the real `gi_room` binary but that the
    /// existing (coarse-grid) CPU simulations above could not reproduce:
    /// the two are silently testing two different regimes, not the same
    /// algorithm at two densities that happen to agree.
    #[test]
    fn sealed_room_with_real_cube_furniture_at_production_ddgi_density_stays_dark() {
        let (_e, objects, bvh) = sealed_gi_room_shell_with_cubes();
        let lights = [overhead_sun_at_35_degrees()];
        // Verbatim examples/gi_room.rs::DdgiConfig values (probes_per_frame
        // is a relight-ROTATION knob only, not a grid-shape parameter —
        // simulate_ddgi_grid_peak_per_frame_with_config always relights the
        // whole grid every frame regardless, so it's not passed here).
        let peak_per_frame = simulate_ddgi_grid_peak_per_frame_with_config(&objects, &bvh, &lights, Vec3::splat(1.2), 6, 8);

        println!("sealed-room (production DDGI density: spacing=1.2, vertical_layers=6, tile_size=8) peak irradiance per frame: {peak_per_frame:?}");
        let final_peak = *peak_per_frame.last().unwrap();
        assert!(
            final_peak < 0.01,
            "a fully sealed, sun-only room's own DDGI grid, at examples/gi_room.rs's OWN production probe \
             density (spacing=1.2, vertical_layers=6, tile_size=8), must stay (near-)exactly dark after 20 \
             relight frames from zero history — got peak irradiance {final_peak} (trajectory: {peak_per_frame:?})."
        );
    }

    /// `texel_to_direction`'s own inverse — given a world direction,
    /// which texel would `texel_to_direction` have produced closest to
    /// it. Brute-force nearest-texel search (this module has no closed-
    /// form octahedral encode reachable from here without duplicating
    /// `ddgi_ref`'s own private encode function — this test module IS
    /// `ddgi_ref`, so `octahedral_encode` is directly reachable via
    /// `super::*`; used directly below instead of a new helper).
    fn direction_to_texel(dir: Vec3, tile_size: u32) -> (u32, u32) {
        let uv = octahedral_encode(dir.normalize());
        let max_index = tile_size as f32 - 1.0;
        let tx = (uv.x * tile_size as f32).min(max_index).max(0.0) as u32;
        let ty = (uv.y * tile_size as f32).min(max_index).max(0.0) as u32;
        (tx, ty)
    }

    /// `examples/gi_room.rs`'s own real sun: `Transform::from_xyz(0.0,
    /// 10.0, 0.0).looking_at(Vec3::new(-7.63, 4.26, -2.97), Vec3::Y)`,
    /// confirmed ~35 degrees above horizon.
    fn overhead_sun_at_35_degrees() -> Light {
        let sun_origin = Vec3::new(0.0, 10.0, 0.0);
        let sun_target = Vec3::new(-7.63, 4.26, -2.97);
        let direction_or_position = (sun_target - sun_origin).normalize();
        Light {
            kind: LightKind::Directional,
            color: Vec3::ONE,
            direction_or_position,
            intensity: 1500.0,
            spot_direction: Vec3::ZERO,
            range: 20.0,
            inner_angle: 0.3,
            outer_angle: 0.6,
            shadow_softness_k: 2.0,
        }
    }
}

