//! Main-world -> render-world extraction: turns the current frame's
//! `SdfSceneRoot`-rooted scene into a flat, GPU-uploadable payload
//! (`RenderHybridScene`) the render world can read every frame.
//!
//! **Camera/view data needs no extraction of its own** — deliberately not
//! duplicated here. `docs/knowledge/hybrid-architecture/
//! bevy-native-integration.md` documents this as already-correct
//! precedent, and this module follows it: `assets/shaders/hybrid_trace.wgsl`
//! and `hybrid_blit.wgsl` bind Bevy's own `View` uniform directly (ray
//! origin/direction from `view.world_position`/`view.world_from_clip`,
//! depth reconstruction from `view.clip_from_world`) instead of a
//! hand-duplicated `view_proj` matrix. `hybrid_legacy`'s own `SceneUniform`
//! duplicated `view_proj` mainly to carry `tan_half_y` (a screen-space cone
//! constant for its pixel-footprint antialiasing) alongside it — this
//! renderer's flat-color-only step has no such per-pixel footprint math, so
//! there's nothing that actually needs a second copy of the projection
//! matrix; Bevy's real `View` already has everything both shaders need.
//!
//! Pattern used: manual `Extract<Query<...>>` aggregating many entities
//! into one resource — option 4 of the four documented extraction patterns
//! (`docs/knowledge/bevy-rendering/architecture/
//! entity-sync-and-extraction-patterns.md`), the right fit here since the
//! job is "collect every SdfSceneRoot's shapes into one flat array," not
//! "mirror one component per entity."
//!
//! `HybridDebugFlags` is the natural registration point once a debug toggle
//! gets added to `examples/gallery.rs`'s egui `controls_panel()` — the flag
//! lives here, the panel just flips bits. No bits are defined yet since no
//! feature exists to gate (this step is flat-color-only, no
//! lighting/shadows/AO to kill-switch).

use bevy::math::{Quat, Vec3};
use bevy::prelude::*;
use bevy::render::Extract;
use bevy::render::ExtractSchedule;
use bevy::render::RenderApp;
use bevy::render::render_resource::ShaderType;

use crate::hybrid::bvh::{BvhNodeGpu, PersistentBvh, to_gpu_nodes};
use crate::hybrid::material::Material;
use crate::hybrid::scene;
use crate::sdf::assembly::SdfSceneRoot;
use crate::sdf::components::Shape;

/// Which `cpu_ref::LightKind` a `LightGpu` record names — mirrors
/// `hybrid_legacy`'s `LIGHT_KIND_*` constants
/// (`assets/shaders/hybrid_legacy_trace.wgsl`).
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LightKindGpu {
    Directional = 0,
    Point = 1,
    Spot = 2,
}

/// One light's GPU-uploadable record. Field layout mirrors `cpu_ref::
/// Light` exactly (minus its `LightKind` enum, replaced by the `kind` tag
/// above) — see that struct's doc comment for what each field means per
/// light kind. `bytemuck::Pod`/`Zeroable` need every field to be
/// plain-old-data with no padding ambiguity, hence the explicit `_pad*`
/// fields keeping this 16-byte aligned throughout (matches `ObjectGpu`'s
/// own convention in this same file).
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct LightGpu {
    pub kind: u32,
    pub _pad_kind0: u32,
    pub _pad_kind1: u32,
    pub _pad_kind2: u32,
    pub color_r: f32,
    pub color_g: f32,
    pub color_b: f32,
    pub intensity: f32,
    pub direction_or_position_x: f32,
    pub direction_or_position_y: f32,
    pub direction_or_position_z: f32,
    pub range: f32,
    pub spot_direction_x: f32,
    pub spot_direction_y: f32,
    pub spot_direction_z: f32,
    pub inner_angle: f32,
    pub outer_angle: f32,
    /// Soft-shadow penumbra hardness — see `cpu_ref::Light::
    /// shadow_softness_k`'s doc comment.
    pub shadow_softness_k: f32,
    pub _pad_tail1: f32,
    pub _pad_tail2: f32,
}

/// Which of the 3 fixed light sources this session's gallery spawns are
/// currently enabled — read during extraction to decide whether each
/// light contributes to `RenderHybridScene::lights` this frame. A
/// disabled light is extracted as if its entity didn't exist at all
/// (not extracted with zero intensity), so a disabled light costs
/// nothing in the shading loop, not just "contributes zero." Lives here
/// (not in `examples/gallery.rs`) since extraction is the one place that
/// needs to read it every frame — the CLI/egui toggle code in
/// `gallery.rs` only ever writes to this resource, following the same
/// "one shared resource, CLI and UI both drive it" pattern `DebugGizmos`
/// already established there.
#[derive(Resource, Clone, Copy, Debug)]
pub struct LightToggles {
    pub sun: bool,
    pub lamp: bool,
    pub projector: bool,
}

impl Default for LightToggles {
    fn default() -> Self {
        Self { sun: true, lamp: true, projector: true }
    }
}

/// Global soft-shadow config, read during extraction and applied to every
/// light equally — one shared `k` (penumbra hardness) to start, rather
/// than a per-light dial, matching how `EXPOSURE` and other shading
/// constants are currently shared across all lights too. `enabled` is the
/// `DBG_SHADOWS_OFF`-equivalent kill switch: when off, `hybrid_trace.wgsl`
/// skips the shadow ray entirely (full visibility unconditionally), not
/// just multiplies by `vis=1.0` — same "costs nothing when disabled, not
/// just contributes nothing" principle `LightToggles` already established.
#[derive(Resource, Clone, Copy, Debug)]
pub struct ShadowConfig {
    pub enabled: bool,
    pub k: f32,
}

impl Default for ShadowConfig {
    fn default() -> Self {
        // k=2.0, NOT hybrid_legacy's own k=12.0 default — confirmed via a
        // real k-sweep (see cpu_ref.rs::tests::
        // debug_k_sweep_stress_100_worst_case) that k=12 produces real,
        // confirmed-false shadow darkening on points genuinely outside an
        // occluder's true shadow reach, once the candidate-margin fix
        // (needed for round, not polygonal, silhouettes — see
        // shadow_candidate_margin's doc comment) is large enough to find
        // real-but-distant candidates: the `d/(k*t)` formula's `1/t`
        // softness decay (documented, never fully eliminated, in
        // docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md) makes
        // even a genuine 1-unit gap read as ~95% shadowed at k=12 once
        // t reaches ~2 units — too aggressive for this renderer's actual
        // object scale (~0.8-2.4 units). k=2.0 is the largest (hardest/
        // crispest) value the sweep found with zero false-darkening
        // across a full --stress 100 grid scan, while still passing the
        // round-silhouette regression test (`no_ring_sample_near_the_
        // sphere_reads_fully_lit`) — i.e. still soft/round, not a
        // degenerate always-fully-lit result.
        Self { enabled: true, k: 2.0 }
    }
}

/// Same-frame edge-aware spatial denoise for the indirect-diffuse term —
/// originally built for Stage B's per-pixel hemisphere-sample noise (see
/// `cpu_ref.rs::blur_indirect_at`'s doc comment for the full rationale:
/// this renderer has no temporal accumulation to average per-frame
/// jitter across, so a same-frame spatial blur on just this noisy
/// channel was the right-sized fix). Stage B itself, and every
/// indirect-diffuse technique tried after it (DDGI, a hash-grid radiance
/// cache, ReSTIR GI — see PROGRESS.md's own entries for each), have
/// since been superseded by SDF cone tracing, this renderer's sole
/// surviving technique — this pass is kept live rather than deleted: it
/// still runs on whatever `shade`'s indirect term currently is, and
/// whether cone tracing's own result still benefits from the same
/// treatment is an open, not-yet-resolved question — kept rather than
/// assumed unnecessary. Mirrors `ShadowConfig`'s exact shape — a kill switch to start; the
/// blur radius/edge-weight tunables (`BLUR_RADIUS`/`BLUR_NORMAL_SIGMA`/
/// `BLUR_DEPTH_SIGMA` in `cpu_ref.rs`) stay hardcoded — expose them live
/// only if a first pass shows they're worth tuning at runtime.
#[derive(Resource, Clone, Copy, Debug)]
pub struct DenoiseConfig {
    pub enabled: bool,
}

impl Default for DenoiseConfig {
    fn default() -> Self {
        Self { enabled: true }
    }
}

/// Temporal accumulation of the indirect-diffuse term across frames — the
/// architectural fix for the banding `DenoiseConfig`'s same-frame spatial
/// blur alone can't fully hide (see `temporal_ref.rs`'s own module doc
/// comment for the full picture). Mirrors `DenoiseConfig`'s exact shape:
/// a real kill switch (`enabled`, "costs one cheap copy-pass instead of a
/// skipped dispatch when off" — same reasoning as
/// `SceneUniform::denoise_enabled`'s own doc comment) plus one genuinely
/// live-tunable scalar.
///
/// `max_history_length` needs a real sweep once this feature is up and
/// visually testable — 24.0 is a typical SVGF/TAA starting value, NOT a
/// validated number for this renderer's own noise/ghosting tradeoff yet
/// (same "checked, not assumed" discipline as `ShadowConfig::k`/
/// `INDIRECT_MAX_T`'s own doc comments establish).
#[derive(Resource, Clone, Copy, Debug)]
pub struct TemporalConfig {
    pub enabled: bool,
    pub max_history_length: f32,
}

impl Default for TemporalConfig {
    fn default() -> Self {
        Self { enabled: true, max_history_length: 24.0 }
    }
}

/// Sub-pixel jitter on the PRIMARY ray — step 1a of a temporal-upscaling
/// experiment (see `taa_ref.rs`'s own module doc comment for the full
/// rationale and the specific risk this is testing: whether
/// `hybrid_temporal.wgsl`'s existing depth/normal disocclusion thresholds
/// and `hybrid_dof.wgsl`'s own same-pixel history check already tolerate
/// sub-pixel jitter as ordinary noise, before any resolution change is
/// introduced). Defaults OFF (`enabled: false`) — this is an experimental
/// A/B toggle being live-measured, not yet a proven improvement; every
/// existing scene/example keeps its exact current unjittered behavior
/// unless a caller explicitly opts in via `--jitter`.
///
/// `ring_size` bounds the Halton index's own growth (see `taa_ref::
/// taa_jitter_offset`'s doc comment for why this differs from `DofConfig`'s
/// `max_history_length`-as-ring-size convention — Halton is aperiodic, not
/// a fixed sample set to cycle through) — reusing `TemporalConfig::
/// max_history_length`'s own typical value as a starting point rather than
/// inventing an unrelated third number, since both exist to bound how many
/// frames of jitter diversity get contributed before this renderer's own
/// accumulation windows would have converged anyway.
#[derive(Resource, Clone, Copy, Debug)]
pub struct JitterConfig {
    pub enabled: bool,
    pub ring_size: u32,
}

impl Default for JitterConfig {
    fn default() -> Self {
        Self { enabled: false, ring_size: 64 }
    }
}

/// Trace-resolution scale factor — step 1b of the same temporal-upscaling
/// experiment `JitterConfig`'s own doc comment describes step 1a of.
/// `scale < 1.0` renders the entire hybrid pipeline (primary trace,
/// temporal accumulation, reflection/transmission, denoise, DOF) at a
/// SMALLER resolution than the real output, then `hybrid_blit.wgsl`
/// bilinearly upscales the result to the true `ViewTarget` resolution —
/// the standard TAAU/checkerboard-successor shape (see this project's own
/// SOTA research summary, referenced in `PROGRESS.md`'s "primary-ray
/// sub-pixel jitter" entry, for why this beats plain spatial downscale:
/// combined with `JitterConfig`'s own per-frame sub-pixel jitter, the
/// temporal accumulator effectively supersamples back toward full detail
/// over several frames rather than just showing a blurrier image).
///
/// Defaults to `1.0` (no scaling, bit-for-bit today's existing behavior)
/// — like `JitterConfig`, this is an experimental A/B toggle being live-
/// measured, not a proven improvement, so every existing scene/example
/// keeps its exact current full-resolution behavior unless a caller
/// explicitly opts in via `--render-scale`.
#[derive(Resource, Clone, Copy, Debug)]
pub struct RenderScaleConfig {
    pub scale: f32,
}

impl Default for RenderScaleConfig {
    fn default() -> Self {
        Self { scale: 1.0 }
    }
}

/// Which indirect-diffuse technique is currently active — this
/// renderer's single, mutually-exclusive GI selector. `shade()`
/// (`hybrid_trace.wgsl`) writes into one `indirect: vec3<f32>` slot
/// consumed once by the denoise pass.
///
/// DDGI, a hash-grid radiance cache, and ReSTIR GI were all built and
/// real-GPU-measured earlier in this project's history (see PROGRESS.md's
/// own entries for each — kept as historical record) before this project
/// briefly consolidated on `ConeTrace` alone (SDF cone tracing, fully
/// in-shader with no separate dispatch pass — see
/// `crate::hybrid::conetrace_ref`'s own module doc comment) for
/// single-technique maintenance simplicity. `Ddgi` was revived as the
/// default after three independent visual reviews of the same scene all
/// converged on the same structural gap cone tracing cannot fix: its
/// 5-cone hemisphere is centered on a shaded surface's OWN normal, so a
/// surface facing away from a bright light source (e.g. a small
/// object's own front face, with a bright wall behind/beside it out of
/// that hemisphere's view) can never sample that light even indirectly
/// — no single-hop hemisphere sample can see light that requires
/// bouncing off an intermediate surface first. DDGI's persistent
/// world-space probe grid solves this structurally: a probe positioned
/// near that same object independently has its OWN visibility to the
/// bright wall (probes sample many directions from a fixed point, not
/// one hemisphere from a specific surface normal), so a nearby shaded
/// point interpolating between probes picks up real wrap-around bounce
/// light regardless of its own normal's direction. `ConeTrace` is kept
/// in the codebase (not deleted) but is no longer the default — see
/// `conetrace_ref`'s own module doc comment for its own continued
/// tradeoffs (no temporal lag, no spatial quantization, cheaper at
/// small scale) should it need reviving as a selectable alternative
/// again.
///
/// `RadianceCascades` (discriminant `2`, the previously-reserved/unused
/// "HashGrid" slot — ReSTIR/the hash-grid radiance cache were never
/// revived, so this slot was free) is a SEPARATE, EXPERIMENTAL addition,
/// not a `Ddgi` replacement: an A/B alternative built specifically to
/// test whether a cascade-of-levels hierarchy (see
/// `crate::hybrid::radiance_cascades_ref`'s own module doc comment)
/// closes DDGI's own known dark-corridor gap (`PROGRESS.md`'s
/// "Investigated (not fixed)" entry) differently than DDGI's fixed-
/// density grid can. `Ddgi` remains the shipping default regardless of
/// this experiment's outcome — see the plan document that recorded this
/// decision.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum GiMethod {
    None = 0,
    Ddgi = 1,
    RadianceCascades = 2,
    ConeTrace = 3,
}

#[derive(Resource, Clone, Copy, Debug)]
pub struct GiMethodConfig {
    pub method: GiMethod,
}

impl Default for GiMethodConfig {
    fn default() -> Self {
        // Ddgi: see GiMethod's own doc comment for why this replaces
        // ConeTrace as the default — three independent visual reviews
        // converged on the exact structural gap DDGI's spatial probe
        // cache solves and cone tracing's per-pixel single hemisphere
        // cannot.
        Self { method: GiMethod::Ddgi }
    }
}

/// DDGI (persistent world-space probe grid) config — see
/// `crate::hybrid::ddgi_ref`'s own module doc comment for the full
/// technique and `GiMethod`'s own doc comment for why this is the
/// default indirect-diffuse GI method. None of these defaults have been
/// validated by a real sweep against this renderer's specific scenes —
/// first-pass starting points carried over unchanged from when DDGI was
/// originally built and measured (see PROGRESS.md's own DDGI entries).
///
/// `probe_spacing`/`vertical_layers` drive
/// `ddgi_ref::probe_grid_from_bounds` — the grid ITSELF (origin/spacing/
/// dims) is derived once from the scene's own root BVH AABB, not
/// hand-authored, since `--stress N` scenes vary in size.
#[derive(Resource, Clone, Copy, Debug)]
pub struct DdgiConfig {
    pub probes_per_frame: u32,
    pub tile_size: u32,
    pub max_history_length: f32,
    pub max_t: f32,
    pub probe_spacing: f32,
    pub vertical_layers: u32,
}

impl Default for DdgiConfig {
    fn default() -> Self {
        // probe_spacing=11.0: matches the --stress N object cell pitch
        // this renderer's own test scenes already establish (see
        // ddgi_ref::probe_grid_from_bounds's own doc comment) — checked
        // against real scene scale (30,603 probes at --stress 10000,
        // see ddgi_ref.rs::tests::debug_stress_10000_probe_grid_scale_profile),
        // not assumed. vertical_layers=3: ground level, a mid layer, a
        // layer above typical object height — this renderer's shapes
        // have real height variance, so a single flat horizontal layer
        // would give every shaded point above/below it the same
        // interpolated irradiance regardless of true 3D position.
        Self { probes_per_frame: 512, tile_size: 8, max_history_length: 24.0, max_t: 60.0, probe_spacing: 11.0, vertical_layers: 3 }
    }
}

/// Radiance Cascades config — see `crate::hybrid::radiance_cascades_ref`'s
/// own module doc comment for the technique and `GiMethod::RadianceCascades`'s
/// own doc comment for why this is an EXPERIMENTAL alternative to DDGI,
/// not a replacement. Level 0's own `base_*` parameters drive every other
/// level via `radiance_cascades_ref::cascade_level_params`'s `2^i`/`4^i`
/// scaling — see that function's own doc comment. `level_count` fixed at
/// `RADIANCE_CASCADES_LEVEL_COUNT` (not a tunable field): the WGSL side
/// (`hybrid_radiance_cascades.wgsl`'s own `LEVEL_COUNT` const) and the
/// GPU-side level-uniform array are both sized to this exact constant, so
/// a runtime-variable level count would need a very different (dynamic
/// buffer) design — out of scope for this experiment (see the plan
/// document's own "small fixed level count" guidance).
#[derive(Resource, Clone, Copy, Debug)]
pub struct RadianceCascadesConfig {
    pub base_spacing: f32,
    pub base_ray_count: u32,
    pub base_interval: f32,
    pub base_tile_size: u32,
    /// How many times `hybrid_radiance_cascades.wgsl`'s own relight
    /// dispatch runs per frame — the bounce-depth knob. Cascades has no
    /// cross-frame temporal accumulation (unlike DDGI's own EMA blend,
    /// which gets effectively unbounded bounce depth for free over many
    /// frames — see `ddgi_ref.rs::probe_ray`'s own `indirect_at_hit` doc
    /// comment), so a single dispatch's own bounce read
    /// (`relight_cascade_texel`'s own `indirect_at_hit`-equivalent call,
    /// `cascade_sample_hierarchy_at_hit` in the WGSL) only ever samples
    /// whatever the SAME dispatch already wrote earlier this same frame —
    /// inconsistent, GPU-scheduling-order-dependent, not a real N-bounce
    /// depth. Re-dispatching the SAME relight pass `bounce_passes` times
    /// per frame, each one a genuinely separate `begin_compute_pass` call
    /// (WGPU/Vulkan's own pass-boundary-as-barrier guarantee makes pass
    /// N+1's reads see pass N's writes), gives an explicit, real N-bounce
    /// depth per frame instead — pass 1 has nothing written yet to
    /// sample (pure direct light, `bounce_passes: 1` recovers the exact
    /// original single-pass behavior), pass 2 reads pass 1's now-fully-
    /// written atlas (one real bounce), pass 3 reads pass 2's (two real
    /// bounces), and so on. Cost scales roughly linearly with this value
    /// (`bounce_passes` full relight dispatches instead of 1) — see
    /// `pass.rs`'s own dispatch loop.
    pub bounce_passes: u32,
}

/// Matches `hybrid_radiance_cascades.wgsl`'s own `LEVEL_COUNT` const —
/// kept in sync by hand, no shared import, per this codebase's established
/// per-pass-file self-containment convention (see that shader's own header
/// comment).
pub const RADIANCE_CASCADES_LEVEL_COUNT: u32 = 4;

impl Default for RadianceCascadesConfig {
    fn default() -> Self {
        // base_spacing=11.0/base_tile_size=8: same starting point as
        // DdgiConfig's own probe_spacing/tile_size (untested-by-sweep,
        // carried over as a like-for-like comparison baseline for this
        // A/B experiment). base_ray_count=64/base_interval=3.0: matches
        // radiance_cascades_ref.rs's own test fixtures (level 0 = 64
        // rays, interval [0, 3)) — real values this experiment's own CPU
        // reference has already exercised, not fresh guesses.
        // bounce_passes=1: preserves this experiment's own prior
        // single-pass, direct-plus-one-stale-read behavior exactly as
        // the default (no surprise cost increase for existing callers) —
        // opt in to real multi-bounce via an explicit override, same
        // "new capability off by default" shape `ConeTraceConfig::
        // max_bounces`'s own doc comment already establishes.
        Self { base_spacing: 11.0, base_ray_count: 64, base_interval: 3.0, base_tile_size: 8, bounce_passes: 1 }
    }
}

/// SDF cone tracing config — this renderer's sole indirect-diffuse GI
/// technique's own tunables. See `crate::hybrid::conetrace_ref`'s own
/// module doc comment for the technique itself.
///
/// Cone tracing maintains NO persistent structure — every field here is
/// read directly at shading time each frame, not by a separate relight/
/// update pass (see `conetrace_ref`'s own doc comment for the resulting
/// "no temporal amortization" cost tradeoff).
#[derive(Resource, Clone, Copy, Debug)]
pub struct ConeTraceConfig {
    /// Cone half-angle in radians — controls how fast the cone's
    /// effective hit-test radius grows with marched distance (see
    /// `conetrace_ref::cone_radius_at`). Wider = softer/blurrier
    /// indirect result (more geometry contributes partial coverage per
    /// march); `half_angle=0.0` degenerates exactly to a point ray (see
    /// `conetrace_ref::tests::cone_degenerates_to_point_ray_at_zero_half_angle`).
    pub cone_half_angle: f32,
    /// The cone's own footprint radius AT ITS ORIGIN (`t=0`) — a real
    /// tunable rather than a fixed bias, since a cone (unlike a point
    /// ray) has a meaningful non-zero starting radius by construction.
    pub cone_origin_radius: f32,
    /// Reach (world units) of each cone, applied directly at shading
    /// time (this technique has no separate relight pass).
    pub max_t: f32,
    /// How many bounces of indirect light each cone chains — see
    /// `conetrace_ref::cone_trace_ray`'s own doc comment for the full
    /// multi-bounce design (bounce 1 keeps the full cone from
    /// `cone_trace_indirect`'s own 5-cone hemisphere; every bounce
    /// after fires a single degenerate point ray continuing straight
    /// along the previous hit's own surface normal — a deterministic,
    /// RNG-free direction, since this renderer has no per-pixel RNG
    /// primitive and cone tracing has no temporal accumulation to
    /// denoise added noise away). Cost is linear in this value
    /// (`5 + (max_bounces - 1)` cones/pixel), not exponential — a
    /// deliberate cost/quality tradeoff. `1` (this default) is
    /// EXACTLY today's pre-multi-bounce behavior — every existing test
    /// and every existing visual result is pinned to `max_bounces: 1`,
    /// so changing this default would be a real, unintended visual
    /// regression, not just a config change. Clamped to
    /// `[1, MAX_CONE_BOUNCES]` (`hybrid_trace.wgsl`'s own compile-time
    /// loop-bound constant, currently `8`) on both the CPU-ref and WGSL
    /// sides.
    pub max_bounces: u32,
}

impl Default for ConeTraceConfig {
    fn default() -> Self {
        // cone_half_angle=0.15 rad (~8.6 degrees): a FIRST-PASS STARTING
        // POINT, explicitly NOT yet validated by a real sweep against
        // this renderer's own object scale — matching this project's
        // own established "measure real data before claiming validated"
        // convention (see the soft-shadow k=12->k<=2.0 sweep precedent
        // in docs/knowledge/sdf-3d/rendering/soft-shadows-and-ao.md — a
        // value blindly ported from elsewhere was wrong for THIS
        // renderer's own scale until swept). Chosen only as "narrow
        // enough not to immediately blur across this renderer's own
        // object spacing at a plausible cone reach," not derived from
        // first principles. cone_origin_radius=0.05: small relative to
        // that same object scale, same "starting point, not validated"
        // caveat. max_t=60.0: a like-for-like reach with this project's
        // own established relight-ray-reach starting points from the
        // now-removed techniques' own history. max_bounces=1: preserves
        // this renderer's existing single-bounce behavior exactly —
        // multi-bounce is opt-in, not a changed default.
        Self { cone_half_angle: 0.15, cone_origin_radius: 0.05, max_t: 60.0, max_bounces: 1 }
    }
}

/// Multi-bounce specular reflection config — independent of `GiMethod`
/// (reflections are a real part of PBR specular shading, not a swappable
/// indirect-diffuse *technique*; unlike `GiMethod`'s mutually-exclusive
/// choice, reflections apply on top of whichever diffuse-GI technique is
/// active). See `crate::hybrid::reflect_ref`'s own module doc comment for
/// the full technique (roughness-derived cone aperture reusing
/// `conetrace_ref::trace_cone`, Fresnel-gated ray firing, one-bounce
/// diffuse-GI termination instead of nested reflection).
#[derive(Resource, Clone, Copy, Debug)]
pub struct ReflectionConfig {
    /// `0` = reflection rays are skipped entirely (costs nothing, not
    /// just contributes nothing — matches `shadows_enabled`'s/`gi_method`'s
    /// own "an inactive feature costs zero" convention). `1` = on.
    pub enabled: bool,
    /// How many specular bounces each reflection ray chains — see
    /// `reflect_ref::reflect_trace_ray`'s own doc comment for why this is
    /// linear-cost by construction (unlike diffuse GI's hemisphere
    /// fan-out) and why the default is deliberately small (`1`, matching
    /// Lumen's own documented `MaxBounces` default) rather than inheriting
    /// `ConeTraceConfig::max_bounces`'s own precedent value. Clamped to
    /// `[1, MAX_REFLECTION_BOUNCES]` (`reflect_ref`'s own compile-time
    /// ceiling, `4`) on both the CPU-ref and WGSL sides.
    pub max_bounces: u32,
    /// Per-pixel Fresnel-term luminance below which a reflection ray is
    /// skipped entirely rather than fired and multiplied by a near-zero
    /// weight — see `reflect_ref::REFLECTION_FRESNEL_CUTOFF`'s own doc
    /// comment (same value/reasoning as `trace_shadow`'s own
    /// `VIS_CUTOFF`).
    pub fresnel_cutoff: f32,
    /// Reach (world units) of each reflection ray — same role as
    /// `ConeTraceConfig::max_t`, kept as an independent tunable since a
    /// mirror reflection may legitimately want a longer/shorter reach than
    /// the diffuse-GI cones.
    pub max_t: f32,
}

impl Default for ReflectionConfig {
    fn default() -> Self {
        // enabled=true: reflections only actually fire on pixels whose
        // per-pixel Fresnel term clears fresnel_cutoff (see
        // reflect_ref's own doc comment), so most of this scene's
        // low-reflectance dielectrics pay zero cost by default already —
        // unlike max_bounces=1 precedent set for an unmeasured new
        // feature's DEFAULT VALUE (ConeTraceConfig's own convention),
        // there is no pre-existing "no reflections" visual baseline this
        // renderer needs to preserve by defaulting off, since reflections
        // are new functionality with no prior behavior to regress.
        // max_bounces=1: Lumen's own documented default, and this
        // renderer's own established "an unmeasured feature's default
        // should be its cheapest real setting" convention.
        // fresnel_cutoff=0.02: matches trace_shadow's own VIS_CUTOFF.
        // max_t=60.0: matches ConeTraceConfig's own max_t default.
        Self { enabled: true, max_bounces: 1, fresnel_cutoff: 0.02, max_t: 60.0 }
    }
}

/// Multi-bounce transmission/refraction config — mirrors `ReflectionConfig`
/// exactly (see that struct's own doc comment for the shared reasoning).
/// Independent of both `GiMethod` and `ReflectionConfig`: a material can
/// be simultaneously reflective (via its `reflectance`/Fresnel term) AND
/// transmissive (via `Material::transmission`) — that's exactly how real
/// glass behaves (a bright reflection at grazing angles, transmission
/// through the rest). See `crate::hybrid::refract_ref`'s own module doc
/// comment for the full technique.
#[derive(Resource, Clone, Copy, Debug)]
pub struct TransmissionConfig {
    /// `0` = transmission rays are skipped entirely (costs nothing) —
    /// same convention as `ReflectionConfig::enabled`. Per-material
    /// `transmission > 0.0` is a SEPARATE, additional gate (see
    /// `cpu_ref::shade`'s own doc comment on that call site) — this flag
    /// is the scene-wide kill switch, not a substitute for it.
    pub enabled: bool,
    /// How many surface crossings (entry+exit pairs) each transmission
    /// ray chains — see `refract_ref::refract_trace_ray`'s own doc
    /// comment. Clamped to `[1, MAX_TRANSMISSION_BOUNCES]` on both the
    /// CPU-ref and WGSL sides.
    pub max_bounces: u32,
    /// Per-pixel transmittable-energy (`1 - fresnel`) luminance below
    /// which a transmission ray is skipped entirely — see
    /// `refract_ref::TRANSMISSION_FRESNEL_CUTOFF`'s own doc comment.
    pub fresnel_cutoff: f32,
    /// Reach (world units) of the interior march / continuation probe —
    /// same role as `ReflectionConfig::max_t`.
    pub max_t: f32,
}

/// Post-tonemap "lens/sensor" pass config (film grain, vignette,
/// chromatic aberration) — see `crate::hybrid::post`'s own module doc
/// comment for why this runs as a separate pass after Bevy's own
/// tonemapping system rather than folded into `hybrid_blit.wgsl`.
/// Deliberately NOT part of `SceneUniform`/`RenderHybridScene`: those
/// feed the trace/temporal/denoise pipeline, which this pass has no
/// bind-group relationship with at all (it reads Bevy's own `ViewTarget`
/// post-tonemap, not any hybrid-owned storage texture).
#[derive(Resource, Clone, Copy, Debug)]
pub struct HybridPostConfig {
    /// `0.0` disables grain entirely (the pass still runs — see
    /// `post::HybridPostUniform`'s own doc comment on why "always run,
    /// coefficient zero" was chosen over a second dispatch-skip branch).
    pub grain_strength: f32,
    /// `0.0` disables vignette darkening entirely.
    pub vignette_strength: f32,
    /// `0.0` disables the chromatic-aberration UV offset entirely.
    pub aberration_strength: f32,
}

/// Stochastic (jittered-lens) depth-of-field config — see
/// `dof_ref`'s own module doc comment for the full technique (jitters
/// the PRIMARY ray's origin on a simulated aperture disk each frame,
/// re-aimed through the same focal-plane point, converged via a
/// dedicated temporal accumulator). UNLIKE `HybridPostConfig`, this DOES
/// feed `SceneUniform`/the trace pipeline directly — the jitter happens
/// at primary-ray generation, not in an isolated post pass, so
/// `enabled` gates real ray-generation behavior, not just a cosmetic
/// pass. Defaults to disabled: unlike grain/vignette/CA (subtle,
/// harmless-by-default cosmetic passes), a nonzero aperture measurably
/// changes what every downstream system (shading, reflections,
/// transmission, GI) sees at each pixel, so this should be an explicit
/// per-scene opt-in, not an always-on-but-tiny default.
#[derive(Resource, Clone, Copy, Debug)]
pub struct DofConfig {
    /// `false` skips the jitter entirely — `generate_primary_ray`
    /// returns the exact unperturbed ray, bit-for-bit identical to
    /// before this feature existed, and the dedicated DOF temporal
    /// accumulator pass copies its input straight through (same "always
    /// run, coefficient/flag says do nothing" convention
    /// `SceneUniform::denoise_enabled` already establishes).
    pub enabled: bool,
    /// World-space distance (same units as `ConeTraceConfig::max_t`/
    /// `DdgiConfig::max_t`, i.e. scene meters) from the camera along its
    /// own optical axis to the perfectly-in-focus plane.
    pub focal_distance: f32,
    /// Aperture f-number (f-stop) — smaller values (e.g. 1.4) mean a
    /// WIDER aperture and stronger/shallower depth of field; larger
    /// values (e.g. 16.0) mean a narrower aperture and progressively
    /// less blur, approaching a pinhole camera. See
    /// `dof_ref::aperture_radius`'s own doc comment for the exact
    /// `A = f / N` relationship.
    pub aperture_f_stops: f32,
    /// Convergence window (in frames) for the dedicated DOF temporal
    /// accumulator — a SEPARATE tunable from
    /// `TemporalConfig::max_history_length` (the GI accumulator's own),
    /// not a shared value: DOF's per-frame ray jitter (a full aperture-
    /// radius world-space offset) is a much LARGER per-frame
    /// perturbation than GI's own hemisphere-sample noise, so the two
    /// accumulators converge at genuinely different rates and shouldn't
    /// be forced to share one knob (see PROGRESS.md's own DOF research
    /// entry for why this was flagged as a real risk to check, not
    /// assumed identical).
    pub max_history_length: f32,
}

impl Default for DofConfig {
    fn default() -> Self {
        Self { enabled: false, focal_distance: 8.0, aperture_f_stops: 4.0, max_history_length: 32.0 }
    }
}

impl Default for HybridPostConfig {
    fn default() -> Self {
        // Small, subtle defaults — "sells the photograph" per the visual
        // review this feature was built from, not a heavy stylized look.
        Self { grain_strength: 0.015, vignette_strength: 0.25, aberration_strength: 0.0015 }
    }
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        // Same reasoning as ReflectionConfig::default(): enabled=true
        // costs nothing on the vast majority of (transmission=0.0)
        // materials already in these example scenes, since the
        // per-material gate in cpu_ref::shade skips the ray entirely.
        Self { enabled: true, max_bounces: 1, fresnel_cutoff: 0.02, max_t: 60.0 }
    }
}

/// Marks the one `DirectionalLight` entity `examples/gallery.rs` spawns
/// as "the sun" — lets extraction (and the gallery's own debug HUD/log)
/// identify this specific light among possibly-multiple `DirectionalLight`
/// entities in the world, and lets `LightToggles::sun` gate it
/// specifically. Mirrors `Spinning`'s existing marker-component pattern
/// in `gallery.rs`.
#[derive(Component)]
pub struct SunLight;

/// Marks the one `PointLight` entity as "the lamp" — see `SunLight`'s
/// doc comment for why a marker exists at all.
#[derive(Component)]
pub struct LampLight;

/// Marks the one `SpotLight` entity as "the projector" — see `SunLight`'s
/// doc comment for why a marker exists at all.
#[derive(Component)]
pub struct ProjectorLight;

/// Which `Shape` variant an `ObjectGpu` record names — mirrors `sdf::
/// primitives::GpuShapeKind`'s tag-per-variant pattern (the project's
/// established convention for a GPU-uploadable shape discriminant), kept
/// as a fresh set of tags here rather than reusing that enum since this
/// renderer's `march_object` dispatches on `Shape` directly (via `cpu_ref::
/// local_distance`), not on `sdf::primitives::Sdf` trait objects. Numeric
/// values match `Shape`'s own declaration order in `sdf::components` —
/// purely a convention for readability, `hybrid_trace.wgsl`'s `case`
/// dispatch doesn't require any particular ordering.
///
/// `RoundedCone` intentionally has no tag: `cpu_ref::local_distance`
/// doesn't implement it (a real, pre-existing bug found in `sdf::
/// primitives::RoundedCone::distance`, out of scope to fix here — see
/// that function's doc comment), so there is nothing for a GPU tag to
/// name yet.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeKindGpu {
    Sphere = 0,
    RoundedBox = 1,
    RoundedCylinder = 2,
    Capsule = 3,
    Ellipsoid = 4,
    BoxFrame = 5,
    HexPrism = 6,
}

/// One object's GPU-uploadable record: a shape-kind tag plus up to 8
/// generic scalar params (mirrors `sdf::primitives::GpuPrimitive`'s
/// kind+params encoding — the project's established pattern for a
/// GPU-uploadable shape discriminant with per-kind field reuse, applied
/// fresh here rather than reused since this renderer's `ObjectGpu` also
/// carries transform/color fields `GpuPrimitive` doesn't), world
/// transform (translation + inverse rotation, since `march_object` always
/// needs the INVERSE to bring a world point into local space — doing that
/// inversion once here beats doing it every march step per pixel on the
/// GPU), and the full `hybrid::material::Material` PBR field set (base
/// color, metallic, roughness, reflectance, transmission, ior, emissive)
/// `hybrid_trace.wgsl`'s `shade` needs for its GGX Cook-Torrance BRDF (see
/// `cpu_ref::shade`'s doc comment for the formula this mirrors). Mirrors `cpu_ref::TraceObject`
/// field-for-field (minus `entity`, which has no GPU representation — a
/// leaf's `BvhNodeGpu::right_or_object` names this array's index instead).
///
/// Per-kind param layout (`hybrid_trace.wgsl`'s `march_object` must match
/// this exactly):
/// - `Sphere`: `[radius]`
/// - `RoundedBox`: `[half_extents.x, .y, .z, corner_radius]`
/// - `RoundedCylinder`: `[radius, half_height, edge_radius]`
/// - `Capsule`: `[a.x, .y, .z, b.x, .y, .z, radius]` — `a`/`b` are
///   LOCAL-space offsets from the entity origin (this renderer applies
///   the entity's rotation+translation uniformly to every shape, unlike
///   `hybrid_legacy`'s raymarcher, which stores `Capsule`/`RoundedCone`
///   endpoints in world space and skips the entity's rotation for them —
///   a deliberate simplification so every shape shares one transform
///   path).
/// - `Ellipsoid`: `[radii.x, .y, .z]`
/// - `BoxFrame`: `[half_extents.x, .y, .z, wall_thickness]`
/// - `HexPrism`: `[radius, half_height]`
#[derive(Clone, Copy, Debug, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct ObjectGpu {
    pub shape_kind: u32,
    pub _pad_kind0: u32,
    pub _pad_kind1: u32,
    pub _pad_kind2: u32,
    pub param_0: f32,
    pub param_1: f32,
    pub param_2: f32,
    pub param_3: f32,
    pub param_4: f32,
    pub param_5: f32,
    pub param_6: f32,
    pub param_7: f32,
    pub translation_x: f32,
    pub translation_y: f32,
    pub translation_z: f32,
    pub _pad_translation: f32,
    /// Inverse rotation as a quaternion (x, y, z, w) — `march_object`
    /// transforms `p_world - translation` by this to reach the shape's
    /// local space, exactly like `cpu_ref::march_object`'s
    /// `object.rotation.inverse()`.
    pub inv_rotation_x: f32,
    pub inv_rotation_y: f32,
    pub inv_rotation_z: f32,
    pub inv_rotation_w: f32,
    pub base_color_r: f32,
    pub base_color_g: f32,
    pub base_color_b: f32,
    pub metallic: f32,
    pub roughness: f32,
    pub reflectance: f32,
    /// Reuses what was previously `_pad_material0`/`_pad_material1` — no
    /// std140 layout change, just filling in two fields that were always
    /// zero padding before transmission existed.
    pub transmission: f32,
    pub ior: f32,
    pub emissive_r: f32,
    pub emissive_g: f32,
    pub emissive_b: f32,
    pub _pad_emissive: f32,
    /// Last frame's translation/inverse-rotation — the temporal-
    /// accumulation pass's `reproject_world_point` (`temporal_ref.rs`)
    /// undoes THIS frame's rigid transform (via `inv_rotation_*` above)
    /// then reapplies LAST frame's, to find where a currently-hit surface
    /// point was one frame ago. Defaults to the current frame's own
    /// transform for a freshly-spawned object with no prior frame yet —
    /// see `motion::PreviousShapeTransform`'s doc comment: "no motion" is
    /// the correct first-frame answer, not a missing/garbage value.
    pub prev_translation_x: f32,
    pub prev_translation_y: f32,
    pub prev_translation_z: f32,
    pub _pad_prev_translation: f32,
    pub prev_inv_rotation_x: f32,
    pub prev_inv_rotation_y: f32,
    pub prev_inv_rotation_z: f32,
    pub prev_inv_rotation_w: f32,
}

fn object_gpu_from(
    shape: &Shape,
    translation: Vec3,
    rotation: Quat,
    material: &Material,
    previous: Option<&crate::hybrid::motion::PreviousShapeTransform>,
) -> ObjectGpu {
    let (shape_kind, params) = match *shape {
        Shape::Sphere { radius } => (ShapeKindGpu::Sphere, [radius, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        Shape::RoundedBox { half_extents, corner_radius } => (
            ShapeKindGpu::RoundedBox,
            [half_extents.x, half_extents.y, half_extents.z, corner_radius, 0.0, 0.0, 0.0, 0.0],
        ),
        Shape::RoundedCylinder { radius, half_height, edge_radius } => {
            (ShapeKindGpu::RoundedCylinder, [radius, half_height, edge_radius, 0.0, 0.0, 0.0, 0.0, 0.0])
        }
        Shape::Capsule { a, b, radius } => {
            (ShapeKindGpu::Capsule, [a.x, a.y, a.z, b.x, b.y, b.z, radius, 0.0])
        }
        Shape::Ellipsoid { radii } => (ShapeKindGpu::Ellipsoid, [radii.x, radii.y, radii.z, 0.0, 0.0, 0.0, 0.0, 0.0]),
        Shape::BoxFrame { half_extents, wall_thickness } => (
            ShapeKindGpu::BoxFrame,
            [half_extents.x, half_extents.y, half_extents.z, wall_thickness, 0.0, 0.0, 0.0, 0.0],
        ),
        Shape::HexPrism { radius, half_height } => {
            (ShapeKindGpu::HexPrism, [radius, half_height, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        }
        // RoundedCone has no GPU tag yet — see `ShapeKindGpu`'s doc
        // comment. Falls back to a degenerate (zero-radius) sphere at the
        // entity origin rather than panicking, so a scene that happens to
        // contain one doesn't crash extraction outright; it simply
        // renders as a non-hittable point.
        Shape::RoundedCone { .. } => (ShapeKindGpu::Sphere, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    };
    let inv_rotation = rotation.inverse();
    let (prev_translation, prev_rotation) = match previous {
        Some(p) => (p.translation, p.rotation),
        None => (translation, rotation),
    };
    let prev_inv_rotation = prev_rotation.inverse();
    ObjectGpu {
        shape_kind: shape_kind as u32,
        _pad_kind0: 0,
        _pad_kind1: 0,
        _pad_kind2: 0,
        param_0: params[0],
        param_1: params[1],
        param_2: params[2],
        param_3: params[3],
        param_4: params[4],
        param_5: params[5],
        param_6: params[6],
        param_7: params[7],
        translation_x: translation.x,
        translation_y: translation.y,
        translation_z: translation.z,
        _pad_translation: 0.0,
        inv_rotation_x: inv_rotation.x,
        inv_rotation_y: inv_rotation.y,
        inv_rotation_z: inv_rotation.z,
        inv_rotation_w: inv_rotation.w,
        base_color_r: material.base_color.x,
        base_color_g: material.base_color.y,
        base_color_b: material.base_color.z,
        metallic: material.metallic,
        roughness: material.roughness,
        reflectance: material.reflectance,
        transmission: material.transmission,
        ior: material.ior,
        emissive_r: material.emissive.x,
        emissive_g: material.emissive.y,
        emissive_b: material.emissive.z,
        _pad_emissive: 0.0,
        prev_translation_x: prev_translation.x,
        prev_translation_y: prev_translation.y,
        prev_translation_z: prev_translation.z,
        _pad_prev_translation: 0.0,
        prev_inv_rotation_x: prev_inv_rotation.x,
        prev_inv_rotation_y: prev_inv_rotation.y,
        prev_inv_rotation_z: prev_inv_rotation.z,
        prev_inv_rotation_w: prev_inv_rotation.w,
    }
}

/// Per-frame scalars the trace/blit shaders need beyond what Bevy's own
/// `View` uniform already carries: how many objects/BVH nodes are actually
/// live this frame (the storage buffers are sized to the frame's content,
/// but a resize doesn't necessarily shrink allocated capacity, so the
/// shader needs an explicit count) plus the flat background color a missed
/// ray reports.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct SceneUniform {
    pub object_count: u32,
    pub bvh_node_count: u32,
    pub light_count: u32,
    /// `0` = shadow rays are skipped entirely (full visibility
    /// unconditionally, not just multiplied by `vis=1.0` — costs nothing
    /// when disabled, matching `LightToggles`'s own "costs nothing, not
    /// just contributes nothing" principle), `1` = shadows on. Driven by
    /// `ShadowConfig::enabled`.
    pub shadows_enabled: u32,
    pub background_r: f32,
    pub background_g: f32,
    pub background_b: f32,
    /// `0` = `hybrid_denoise.wgsl` skips its edge-aware blur weight loop
    /// entirely and copies `indirect_view` straight through unblurred
    /// (still one pass — the alternative, skipping the pass's dispatch
    /// altogether, would need the blit path to conditionally read
    /// `indirect_view` vs. a blurred texture depending on this same
    /// flag, which is more branching than one cheap copy-pass costs).
    /// `1` = denoise on. Driven by `DenoiseConfig::enabled`.
    pub denoise_enabled: u32,
    /// `0` = `hybrid_temporal.wgsl` skips reprojection/blend entirely and
    /// copies `indirect_view` straight through to
    /// `accumulated_indirect_view` unblended (same "one cheap copy-pass,
    /// not a conditional dispatch skip" reasoning as `denoise_enabled`'s
    /// own doc comment). `1` = temporal accumulation on. Driven by
    /// `TemporalConfig::enabled`.
    pub temporal_enabled: u32,
    /// Cap on accumulated history length (frames) — driven by
    /// `TemporalConfig::max_history_length`.
    pub temporal_max_history_length: f32,
    /// Which indirect-diffuse technique's shading-time sample function
    /// `hybrid_trace.wgsl`'s `shade()` calls this frame — `GI_METHOD_NONE`/
    /// `_CONETRACE` (WGSL constants mirroring `extract::GiMethod`'s own
    /// `repr(u32)` discriminants exactly). Selecting `None` costs nothing
    /// — same "costs nothing when disabled" shape `temporal_enabled`
    /// above already established. Driven by `GiMethodConfig::method`.
    pub gi_method: u32,
    /// Cone half-angle (radians) — driven by `ConeTraceConfig::
    /// cone_half_angle`. Read by `hybrid_trace.wgsl`'s `cone_trace_
    /// indirect` at shading time only (`GI_METHOD_CONETRACE` branch) —
    /// cone tracing maintains no persistent structure and no separate
    /// relight pass reads this; see `conetrace_ref`'s own doc comment.
    pub conetrace_half_angle: f32,
    /// Cone origin-footprint radius — driven by `ConeTraceConfig::
    /// cone_origin_radius`.
    pub conetrace_origin_radius: f32,
    /// Reach (world units) of each cone — driven by `ConeTraceConfig::
    /// max_t`.
    pub conetrace_max_t: f32,
    /// Bounce count — driven by `ConeTraceConfig::max_bounces`.
    pub conetrace_max_bounces: u32,
    /// `0` = reflection rays are skipped entirely (costs nothing). `1` =
    /// on. Driven by `ReflectionConfig::enabled`.
    pub reflection_enabled: u32,
    /// Bounce count — driven by `ReflectionConfig::max_bounces`. See
    /// `reflect_ref::reflect_trace_ray`'s own doc comment for why this is
    /// linear-cost by construction, unlike diffuse GI's hemisphere
    /// fan-out.
    pub reflection_max_bounces: u32,
    /// Per-pixel Fresnel-luminance cutoff below which a reflection ray is
    /// skipped — driven by `ReflectionConfig::fresnel_cutoff`.
    pub reflection_fresnel_cutoff: f32,
    /// Reach (world units) of each reflection ray — driven by
    /// `ReflectionConfig::max_t`.
    pub reflection_max_t: f32,
    /// `0` = transmission rays are skipped entirely (costs nothing). `1`
    /// = on. Driven by `TransmissionConfig::enabled`.
    pub transmission_enabled: u32,
    /// Bounce count — driven by `TransmissionConfig::max_bounces`.
    pub transmission_max_bounces: u32,
    /// Per-pixel transmittable-energy luminance cutoff below which a
    /// transmission ray is skipped — driven by
    /// `TransmissionConfig::fresnel_cutoff`.
    pub transmission_fresnel_cutoff: f32,
    /// Reach (world units) of the interior march / continuation probe —
    /// driven by `TransmissionConfig::max_t`.
    pub transmission_max_t: f32,
    /// How many probe TEXELS `hybrid_ddgi_relight.wgsl` relights this
    /// frame — driven by `DdgiConfig::probes_per_frame` * `tile_size^2`
    /// (computed once in `prepare_hybrid_scene`, not stored per-config
    /// field, since the shader only ever needs the final texel count).
    /// Only meaningful when `gi_method == GI_METHOD_DDGI` — see
    /// `ddgi_ref`'s own module doc comment for the whole-grid-over-
    /// several-frames cycling this drives.
    pub ddgi_probes_per_frame: u32,
    /// Total probe count in the grid this frame — `ddgi_ref::ProbeGrid::
    /// probe_count`. Needed by the relight pass to know when a frame's
    /// rotating relight window wraps back to probe 0.
    pub ddgi_total_probes: u32,
    /// Octahedral atlas tile size (texels per probe, per side) — driven
    /// by `DdgiConfig::tile_size`.
    pub ddgi_tile_size: u32,
    /// Render-world frame counter driving WHICH `ddgi_probes_per_frame`
    /// texels get relit this frame (`ddgi_ref::ddgi_probe_relight_start`)
    /// — NOT the same as Bevy's own `FrameCount` (this one resets to 0
    /// whenever the grid itself is rebuilt, e.g. on a scene-bounds
    /// change), see `HybridDdgiFrameIndex`'s own doc comment.
    pub ddgi_frame_index: u32,
    /// Cap on accumulated per-texel history length (frames) for the
    /// relight pass's own clamped-EMA blend — same role as
    /// `temporal_max_history_length` above, driven by
    /// `DdgiConfig::max_history_length`.
    pub ddgi_max_history_length: f32,
    /// Reach (world units) of each probe ray — driven by
    /// `DdgiConfig::max_t`.
    pub ddgi_max_t: f32,
    /// `0` = `generate_primary_ray` returns the exact unperturbed ray
    /// (no aperture jitter at all) — costs nothing extra (same "costs
    /// nothing when disabled" shape every other technique toggle above
    /// already establishes). `1` = DOF jitter on. Driven by
    /// `DofConfig::enabled`.
    pub dof_enabled: u32,
    /// World-space distance (scene meters) to the in-focus plane —
    /// driven by `DofConfig::focal_distance`. See `dof_ref::
    /// dof_jittered_ray`'s own doc comment for how this locates the
    /// focus-plane point each jittered ray is re-aimed through.
    pub dof_focal_distance: f32,
    /// The camera's own simulated aperture RADIUS (world units) —
    /// PRE-COMPUTED once per frame in `prepare_hybrid_scene` from
    /// `DofConfig::aperture_f_stops` and the camera's own vertical FOV
    /// via `dof_ref::focal_length_from_vertical_fov`/`aperture_radius`
    /// (not stored as raw f-stops here, since deriving focal length from
    /// FOV needs the camera's `Projection`, which `SceneUniform`'s own
    /// per-frame Rust-side prepare step already has on hand and the WGSL
    /// side does not).
    pub dof_aperture_radius: f32,
    /// Which Vogel-disk sample (out of `dof_max_history_length`-many)
    /// this frame's jitter uses — `dof_ref::dof_sample_index`'s own
    /// rotating-index output, computed once per frame from the render
    /// world's own frame counter (mirrors `ddgi_frame_index`'s identical
    /// per-frame-counter-driven shape above).
    pub dof_frame_index: u32,
    /// Ring size for the Vogel-disk sample rotation — driven by
    /// `DofConfig::max_history_length` (see that field's own doc comment
    /// for why the ring size and the accumulator's own convergence
    /// window are naturally the same number, not two independent
    /// tunables).
    pub dof_max_history_length: f32,
    /// `0` = the primary ray is generated exactly as `frag_coord + 0.5`
    /// unprojection (today's existing, unjittered behavior) — `1` = a
    /// per-frame Halton(2,3) sub-pixel offset (`taa_ref::taa_jitter_
    /// offset`) is added in NDC space before unprojection. Driven by
    /// `JitterConfig::enabled`. See `taa_ref.rs`'s own module doc comment
    /// for why this exists and what it's step 1a of, and `hybrid_trace.
    /// wgsl`'s own `generate_primary_ray` doc comment for the specific
    /// disocclusion-test risk this flag lets a live A/B measurement
    /// answer rather than assume.
    pub jitter_enabled: u32,
    /// This frame's NDC-space jitter offset — PRE-COMPUTED once here
    /// (Rust side) from `taa_ref::taa_jitter_offset`/`jitter_texels_to_
    /// ndc` and the real viewport size, rather than recomputed
    /// independently in WGSL: every one of this renderer's `generate_
    /// primary_ray` copies (`hybrid_trace.wgsl`, `hybrid_blit.wgsl`,
    /// `hybrid_dof.wgsl`) needs the IDENTICAL offset for a given frame,
    /// and computing it once here (rather than three times, once per
    /// WGSL copy, each needing its own `frame_count`/`ring_size` plumbed
    /// through) guarantees they can't drift apart. Zero when `jitter_
    /// enabled == 0`.
    pub jitter_offset_x: f32,
    pub jitter_offset_y: f32,
    /// The trace pass's own working resolution, in texels — driven by
    /// `RenderScaleConfig::scale * real viewport size`, PRE-COMPUTED once
    /// here (Rust side, `pipeline.rs`'s own `prepare_hybrid_scene`, which
    /// already has the real viewport size on hand) rather than derived in
    /// WGSL from `view.viewport`, since `view.viewport` reports the REAL
    /// output resolution, not this renderer's own (possibly smaller)
    /// trace-resolution storage textures. Every trace-resolution compute
    /// pass (`trace_main`, `dof_main`) uses this — not `view.viewport` —
    /// for both its own dispatch-bounds check and its ray-gen UV
    /// conversion; `hybrid_blit.wgsl`'s own fragment shader is the one
    /// place that legitimately still needs the REAL `view.viewport` (it
    /// runs once per REAL output pixel, upscaling the smaller trace
    /// result via a bilinear sample). Equal to the real viewport size
    /// exactly when `RenderScaleConfig::scale == 1.0` (the default).
    pub trace_size_x: u32,
    pub trace_size_y: u32,
}

/// Flat background color reported on a primary-ray miss — magenta, the
/// conventional "nothing was hit / undefined" debug color (same
/// convention as a missing-texture checkerboard), chosen so a genuine
/// miss is immediately, unmistakably visible on screen rather than
/// blending in as a plausible-looking sky. Not meant to be a real sky
/// color — no lighting exists yet to make one meaningful at this step.
pub const BACKGROUND_COLOR: Vec3 = Vec3::new(1.0, 0.0, 1.0);

/// Render-world resource holding this frame's flat GPU payload: the object
/// array, and the BVH flattened to index into it. The object array is
/// rebuilt fresh every frame (cheap — it's a linear walk + per-kind field
/// copy, no BVH-shaped tree work). The BVH itself is NOT rebuilt here —
/// `extract_hybrid_scene` flattens whatever `bvh::PersistentBvh` already
/// refit this frame (see that resource's doc comment for why a from-
/// scratch rebuild here was a real, measured bug at scale, not a
/// hypothetical one).
#[derive(Resource, Clone, Default)]
pub struct RenderHybridScene {
    pub objects: Vec<ObjectGpu>,
    pub nodes: Vec<BvhNodeGpu>,
    pub lights: Vec<LightGpu>,
    pub shadows_enabled: bool,
    pub denoise_enabled: bool,
    pub temporal_enabled: bool,
    pub temporal_max_history_length: f32,
    /// `GiMethod`'s own `repr(u32)` discriminant — see `SceneUniform::
    /// gi_method`'s doc comment.
    pub gi_method: u32,
    pub ddgi_probes_per_frame: u32,
    pub ddgi_tile_size: u32,
    pub ddgi_max_history_length: f32,
    pub ddgi_max_t: f32,
    /// The probe grid itself — computed once per frame from the current
    /// scene's root BVH AABB (see `extract_hybrid_scene`'s own DDGI
    /// section) via `ddgi_ref::probe_grid_from_bounds`. Recomputing this
    /// every frame is cheap (pure arithmetic on one AABB, no BVH
    /// traversal of its own) even though the grid's own CONTENTS (the
    /// atlas) only change when this changes shape.
    pub ddgi_grid: crate::hybrid::ddgi_ref::ProbeGrid,
    /// Radiance Cascades' own `base_*` level-0 parameters (level 1..3
    /// derived from these via `radiance_cascades_ref::cascade_level_params`
    /// — see `RadianceCascadesConfig`'s own doc comment) plus the scene's
    /// own root BVH AABB each level's grid is built from (same source
    /// `ddgi_grid` above uses) — `prepare_hybrid_radiance_cascades`
    /// (`pipeline.rs`) turns this into the 4 `CascadeLevelUniform`
    /// entries `hybrid_radiance_cascades.wgsl` reads, mirroring how
    /// `prepare_hybrid_ddgi` turns `ddgi_grid` into `DdgiGridUniform`.
    /// Only meaningful when `gi_method == GiMethod::RadianceCascades`.
    pub radiance_cascades_base_spacing: f32,
    pub radiance_cascades_base_ray_count: u32,
    pub radiance_cascades_base_interval: f32,
    pub radiance_cascades_base_tile_size: u32,
    pub radiance_cascades_root_bounds_min: Vec3,
    pub radiance_cascades_root_bounds_max: Vec3,
    /// Bounce-depth pass count — driven by `RadianceCascadesConfig::
    /// bounce_passes`. Read by `pass::hybrid_pass`'s own dispatch loop,
    /// not by any WGSL shader directly (WGSL has no notion of "which
    /// dispatch iteration is this" — each pass is a plain, identical
    /// relight dispatch; the loop itself is what creates the bounce
    /// depth, via successive passes' own read-after-write on the shared
    /// atlas).
    pub radiance_cascades_bounce_passes: u32,
    pub conetrace_half_angle: f32,
    pub conetrace_origin_radius: f32,
    pub conetrace_max_t: f32,
    pub conetrace_max_bounces: u32,
    pub reflection_enabled: bool,
    pub reflection_max_bounces: u32,
    pub reflection_fresnel_cutoff: f32,
    pub reflection_max_t: f32,
    pub transmission_enabled: bool,
    pub transmission_max_bounces: u32,
    pub transmission_fresnel_cutoff: f32,
    pub transmission_max_t: f32,
    pub dof_enabled: bool,
    pub dof_focal_distance: f32,
    /// Pre-computed once per frame from `DofConfig::aperture_f_stops`
    /// and the camera's own vertical FOV — see `SceneUniform::
    /// dof_aperture_radius`'s own doc comment for why this is resolved
    /// here (Rust side, where the camera `Projection` is available) and
    /// not in WGSL.
    pub dof_aperture_radius: f32,
    pub dof_frame_index: u32,
    pub dof_max_history_length: f32,
    /// Raw inputs to `taa_ref::taa_jitter_offset`/`jitter_texels_to_ndc`
    /// — NOT the final NDC offset itself, since converting texels to NDC
    /// needs the real viewport size, which this render-world extraction
    /// step doesn't have (`ExtractedView` isn't queried here — see this
    /// module's own top doc comment on why camera/view data generally
    /// isn't extracted). `pipeline.rs`'s own `prepare_hybrid_scene` DOES
    /// already read `extracted_views` for exactly this reason (see its
    /// own `res_x`/`res_y` locals), so the texel-to-NDC conversion happens
    /// there instead, right before `SceneUniform` is built.
    pub jitter_enabled: bool,
    pub jitter_frame_index: u32,
    pub jitter_ring_size: u32,
    /// Driven by `RenderScaleConfig::scale` — see that resource's own doc
    /// comment. Read by `pipeline.rs`'s own `prepare_hybrid_scene` to size
    /// `HybridTargets`/the history buffers; NOT part of `SceneUniform`
    /// (no WGSL shader needs to know this value directly — trace-
    /// resolution shaders derive their own working size from `scene.
    /// trace_size` instead, see that field's own doc comment).
    pub render_scale: f32,
}

pub struct HybridExtractionPlugin;

impl Plugin for HybridExtractionPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<LightToggles>();
        app.init_resource::<ShadowConfig>();
        app.init_resource::<DenoiseConfig>();
        app.init_resource::<TemporalConfig>();
        app.init_resource::<JitterConfig>();
        app.init_resource::<RenderScaleConfig>();
        app.init_resource::<GiMethodConfig>();
        app.init_resource::<DdgiConfig>();
        app.init_resource::<RadianceCascadesConfig>();
        app.init_resource::<ConeTraceConfig>();
        app.init_resource::<ReflectionConfig>();
        app.init_resource::<TransmissionConfig>();
        app.init_resource::<HybridPostConfig>();
        app.init_resource::<DofConfig>();
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .init_resource::<RenderHybridScene>()
            .init_resource::<RenderHybridPostConfig>()
            .add_systems(ExtractSchedule, (extract_hybrid_scene, extract_hybrid_lights, extract_hybrid_post_config));
    }
}

/// Render-world mirror of `HybridPostConfig` — a plain `Clone` copy, not
/// folded into `RenderHybridScene`/`SceneUniform` (see `HybridPostConfig`'s
/// own doc comment for why the post pass has no relationship with that
/// pipeline's bind groups at all).
#[derive(Resource, Clone, Copy, Debug, Default)]
pub struct RenderHybridPostConfig(pub HybridPostConfig);

fn extract_hybrid_post_config(config: Extract<Res<HybridPostConfig>>, mut out: ResMut<RenderHybridPostConfig>) {
    out.0 = **config;
}

/// `ExtractSchedule` system (manual `Extract<Query<...>>` aggregation, see
/// this module's doc comment for why): walks every `SdfSceneRoot`'s shape
/// entities in the MAIN world, flattens the object array, and flattens
/// `bvh::PersistentBvh` (already refit this frame by
/// `bvh::update_persistent_bvh`, which runs earlier in the main app's
/// `Update` schedule — see that system's doc comment) against this
/// frame's object order — into the render-world `RenderHybridScene`
/// resource this frame's `pipeline.rs` uploads.
///
/// Does NOT build its own BVH — extracting `PersistentBvh` via
/// `Extract<Res<...>>` instead of rebuilding from scratch here is the fix
/// for a real, measured bug: at `--stress 10000` a from-scratch full SAH
/// rebuild every single frame (even though 9,999 of 10,000 objects never
/// move after spawn) was the dominant cost in the renderer's frame time.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn extract_hybrid_scene(
    roots: Extract<Query<Entity, With<SdfSceneRoot>>>,
    shapes_for_data: Extract<
        Query<(
            Entity,
            &Shape,
            &GlobalTransform,
            Option<&Material>,
            Option<&ChildOf>,
            Option<&crate::hybrid::motion::PreviousShapeTransform>,
        )>,
    >,
    persistent_bvh: Extract<Res<PersistentBvh>>,
    gi_method_config: Extract<Res<GiMethodConfig>>,
    ddgi_config: Extract<Res<DdgiConfig>>,
    radiance_cascades_config: Extract<Res<RadianceCascadesConfig>>,
    conetrace_config: Extract<Res<ConeTraceConfig>>,
    reflection_config: Extract<Res<ReflectionConfig>>,
    transmission_config: Extract<Res<TransmissionConfig>>,
    dof_config: Extract<Res<DofConfig>>,
    jitter_config: Extract<Res<JitterConfig>>,
    render_scale_config: Extract<Res<RenderScaleConfig>>,
    camera_projection: Extract<Query<&Projection, With<Camera3d>>>,
    frame_count: Extract<Res<bevy::diagnostic::FrameCount>>,
    mut out: ResMut<RenderHybridScene>,
) {
    let data = scene::collect_data(&roots, &shapes_for_data);
    // object_order must be indexed identically to `objects` below, so
    // `to_gpu_nodes` (which resolves leaf entities against this exact
    // slice) produces indices that actually line up with the uploaded
    // object array.
    let object_order: Vec<Entity> = data.iter().map(|o| o.entity).collect();
    let objects: Vec<ObjectGpu> = data
        .iter()
        .map(|o| object_gpu_from(&o.shape, o.translation, o.rotation, &o.material, o.previous.as_ref()))
        .collect();

    out.objects = objects;
    out.nodes = to_gpu_nodes(&persistent_bvh.0, &object_order);

    // DDGI probe grid: derived from the scene's own root BVH AABB, not
    // hand-authored — see `ddgi_ref::probe_grid_from_bounds`'s own doc
    // comment for why. An empty BVH (`nodes` empty — no shapes in the
    // scene yet, e.g. the very first frame) has no meaningful root AABB;
    // `probe_grid_from_bounds`'s own degenerate-bounds handling (a
    // minimum 2x2x2 grid, see that function's own doc comment) already
    // covers this without a special case here.
    let root_bounds =
        persistent_bvh.0.nodes.first().map(|node| node.aabb).unwrap_or(crate::prim::Aabb { min: Vec3::ZERO, max: Vec3::ZERO });
    // Real, confirmed bug (2026-09-19): `probe_grid_from_bounds`'s own
    // half-cell inset (`spacing/2`) exists ONLY to center probe 0 inside
    // its own cell (see that function's doc comment) — it is NOT a
    // geometric safety margin, and was never checked against how thick
    // this scene's OWN enclosing/boundary geometry actually is.
    // `root_bounds` is the union of every object's own AABB, including
    // the OUTWARD-facing surface of any solid boundary shell (e.g.
    // `examples/gi_room.rs`'s walls, which extend outward from the
    // room's interior by `WALL_THICKNESS + WALL_OVERLAP` up to ~0.5
    // units). At that example's own production `probe_spacing=1.2`, the
    // half-cell inset (0.6) very nearly equals that wall thickness,
    // landing the outermost probe LAYER almost exactly on the interior
    // wall plane — worst case, a corner probe (extreme layer on 2-3 axes
    // at once) lands ON or beyond the outer wall surface, fully outside
    // the sealed cavity. From there, `probe_ray`s fired from that probe
    // either skim the wall's own surface at grazing incidence or escape
    // through the wall-panel seam gaps entirely, picking up direct
    // sunlight that a probe safely inside the cavity would never see —
    // a real, structured, colored light leak reproduced on the very
    // FIRST relight dispatch, confirmed via a CPU-ref diagnostic feeding
    // the real (unshrunk) room AABB into `probe_grid_from_bounds`
    // (`ddgi_ref.rs`'s own `probe_grid_from_unshrunk_real_room_bvh_
    // bounds_leaks_light_from_frame_one` test — peak irradiance 0.43 at
    // a corner probe sitting at `(8.8, -3.0, -6.5)`, outside the room's
    // own outer wall face).
    //
    // A second, compounding effect makes a plain fixed inset insufficient
    // on its own: `probe_grid_from_bounds`'s own `dims_x`/`dims_z` use
    // `(extent/spacing).ceil()`, so the grid's actual covered span
    // (`dims * spacing`) almost always OVERSHOOTS the input extent by up
    // to just under one full `spacing` unit — and since that function
    // pins `origin` to `bounds.min + spacing/2` (never re-centers), 100%
    // of that overshoot lands on the MAX-axis side. A margin sized only
    // for wall thickness gets silently eaten by this overshoot on the
    // max side while doing nothing wrong on the min side. Sizing the
    // margin as wall-thickness-safety PLUS a full `probe_spacing` unit
    // covers the worst case on both sides (min side ends up with more
    // clearance than strictly needed, which is harmless).
    //
    // `ddgi_ref`'s own CPU tests already assumed exactly this kind of
    // pre-shrink by hand (`sealed_gi_room_shell_with_cubes`'s callers
    // inset by `1.0`/`0.5`/`1.0` before calling `probe_grid_from_bounds`)
    // — this makes the real GPU pipeline finally match that assumption
    // instead of silently violating it.
    const DDGI_GRID_WALL_SAFETY_MARGIN: f32 = 0.75;
    let ddgi_margin = DDGI_GRID_WALL_SAFETY_MARGIN + ddgi_config.probe_spacing.max(0.0);
    let ddgi_bounds_shrunk =
        crate::prim::Aabb { min: root_bounds.min + Vec3::splat(ddgi_margin), max: root_bounds.max - Vec3::splat(ddgi_margin) };
    // A tiny/degenerate scene (extent smaller than 2x the margin) would
    // invert min/max — `probe_grid_from_bounds` takes `.abs()` of the
    // extent internally so it wouldn't crash, but an inverted box's own
    // `origin` (`bounds.min + spacing/2`) would land on the WRONG side.
    // Fall back to the unshrunk `root_bounds` rather than risk that.
    let ddgi_bounds = if (ddgi_bounds_shrunk.max.x > ddgi_bounds_shrunk.min.x)
        && (ddgi_bounds_shrunk.max.y > ddgi_bounds_shrunk.min.y)
        && (ddgi_bounds_shrunk.max.z > ddgi_bounds_shrunk.min.z)
    {
        ddgi_bounds_shrunk
    } else {
        root_bounds
    };
    out.ddgi_grid = crate::hybrid::ddgi_ref::probe_grid_from_bounds(
        ddgi_bounds,
        Vec3::splat(ddgi_config.probe_spacing),
        ddgi_config.vertical_layers,
    );
    out.ddgi_probes_per_frame = ddgi_config.probes_per_frame;
    out.ddgi_tile_size = ddgi_config.tile_size;
    out.ddgi_max_history_length = ddgi_config.max_history_length;
    out.ddgi_max_t = ddgi_config.max_t;

    // Radiance Cascades: same root_bounds source as ddgi_grid above (the
    // scene's own root BVH AABB) — the per-level CascadeGrid/atlas layout
    // itself is built in prepare_hybrid_radiance_cascades (pipeline.rs),
    // mirroring how ddgi_grid is computed here but DdgiGridUniform is
    // built in prepare_hybrid_ddgi.
    out.radiance_cascades_base_spacing = radiance_cascades_config.base_spacing;
    out.radiance_cascades_base_ray_count = radiance_cascades_config.base_ray_count;
    out.radiance_cascades_base_interval = radiance_cascades_config.base_interval;
    out.radiance_cascades_base_tile_size = radiance_cascades_config.base_tile_size;
    out.radiance_cascades_root_bounds_min = root_bounds.min;
    out.radiance_cascades_root_bounds_max = root_bounds.max;
    out.radiance_cascades_bounce_passes = radiance_cascades_config.bounce_passes.max(1);

    out.gi_method = gi_method_config.method as u32;
    out.conetrace_half_angle = conetrace_config.cone_half_angle;
    out.conetrace_origin_radius = conetrace_config.cone_origin_radius;
    out.conetrace_max_t = conetrace_config.max_t;
    out.conetrace_max_bounces = conetrace_config.max_bounces.max(1);
    out.reflection_enabled = reflection_config.enabled;
    out.reflection_max_bounces = reflection_config.max_bounces.clamp(1, crate::hybrid::reflect_ref::MAX_REFLECTION_BOUNCES);
    out.reflection_fresnel_cutoff = reflection_config.fresnel_cutoff;
    out.reflection_max_t = reflection_config.max_t;
    out.transmission_enabled = transmission_config.enabled;
    out.transmission_max_bounces = transmission_config.max_bounces.clamp(1, crate::hybrid::refract_ref::MAX_TRANSMISSION_BOUNCES);
    out.transmission_fresnel_cutoff = transmission_config.fresnel_cutoff;
    out.transmission_max_t = transmission_config.max_t;

    out.dof_enabled = dof_config.enabled;
    out.dof_focal_distance = dof_config.focal_distance;
    // Vertical FOV -> focal length -> aperture radius: only meaningful
    // for a real perspective camera. An orthographic/custom projection
    // has no physical "focal length" this thin-lens model applies to —
    // rather than a special WGSL-side branch for a case this renderer's
    // own examples never actually use (every camera in gi_room.rs/
    // gallery.rs is Camera3d's own default perspective projection),
    // falling back to a zero aperture radius (no blur at all,
    // equivalent to a pinhole camera) is the correct, safe degradation:
    // `dof_ref::aperture_radius`'s own `f_stop.max(1e-3)` clamp already
    // guards the reverse case (a tiny/zero f-stop), so a zero radius
    // here just means "this camera type can't produce DOF blur," not a
    // divide-by-zero or NaN.
    let vertical_fov = camera_projection.iter().find_map(|p| match p {
        Projection::Perspective(persp) => Some(persp.fov),
        _ => None,
    });
    out.dof_aperture_radius = match vertical_fov {
        Some(fov) if dof_config.enabled => {
            let focal_length = crate::hybrid::dof_ref::focal_length_from_vertical_fov(crate::hybrid::dof_ref::SENSOR_HEIGHT_METERS, fov);
            crate::hybrid::dof_ref::aperture_radius(focal_length, dof_config.aperture_f_stops)
        }
        _ => 0.0,
    };
    out.dof_frame_index = crate::hybrid::dof_ref::dof_sample_index(frame_count.0, dof_config.max_history_length.max(1.0) as u32);
    out.dof_max_history_length = dof_config.max_history_length;

    out.jitter_enabled = jitter_config.enabled;
    out.jitter_frame_index = frame_count.0;
    out.jitter_ring_size = jitter_config.ring_size.max(1);

    out.render_scale = render_scale_config.scale.clamp(0.1, 1.0);
}

/// `ExtractSchedule` system: queries Bevy's own `DirectionalLight`/
/// `PointLight`/`SpotLight` components directly (one query per type,
/// following `docs/knowledge/hybrid-architecture/
/// bevy-native-integration.md`'s established convention — `bevy_pbr`'s
/// clustered light list has no public bind group a custom pipeline could
/// reuse, confirmed, so hand-rolled extraction is the correct approach,
/// not a compromise) and flattens whichever ones are enabled per
/// `LightToggles` into `RenderHybridScene::lights`.
///
/// A disabled light (per `LightToggles`) is skipped entirely, not
/// extracted with zero intensity — it costs nothing in the GPU shading
/// loop, not just "contributes zero," matching `LightToggles`'s own doc
/// comment. Marker components (`SunLight`/`LampLight`/`ProjectorLight`)
/// identify which specific entity each toggle applies to, since a scene
/// could in principle contain more than one `PointLight` etc. even though
/// `examples/gallery.rs` only ever spawns exactly one of each today.
///
/// Intensity conversion: `PointLight`/`SpotLight::intensity` is raw
/// lumens (their own documented unit) — NOT divided by `4*PI` here; that
/// conversion happens in `cpu_ref::light_contribution`/`hybrid_trace.
/// wgsl`'s shading code, matching `bevy_pbr`'s own internal lumens ->
/// luminous-intensity formula exactly (same deferred-conversion pattern
/// `src/prepass_probe::extract_probe_lights` already established and
/// documents in its own doc comment). `DirectionalLight::illuminance` is
/// already in the right unit (lux) and needs no conversion at all.
#[allow(clippy::too_many_arguments)]
fn extract_hybrid_lights(
    toggles: Extract<Res<LightToggles>>,
    shadow_config: Extract<Res<ShadowConfig>>,
    denoise_config: Extract<Res<DenoiseConfig>>,
    temporal_config: Extract<Res<TemporalConfig>>,
    sun: Extract<Query<(&DirectionalLight, &GlobalTransform), With<SunLight>>>,
    lamp: Extract<Query<(&PointLight, &GlobalTransform), With<LampLight>>>,
    projector: Extract<Query<(&SpotLight, &GlobalTransform), With<ProjectorLight>>>,
    mut out: ResMut<RenderHybridScene>,
) {
    let mut lights = Vec::new();
    let k = shadow_config.k;

    if toggles.sun {
        for (light, transform) in &sun {
            let color = light.color.to_linear().to_vec3();
            // Direction rays travel: `forward()`, NOT `up()` — this exact
            // mistake previously broke a shadow angle in this project
            // (per `docs/knowledge/hybrid-architecture/
            // bevy-native-integration.md`), so it's worth naming
            // explicitly here rather than trusting it stays correct by
            // accident.
            let direction = transform.forward().as_vec3();
            lights.push(LightGpu {
                kind: LightKindGpu::Directional as u32,
                _pad_kind0: 0,
                _pad_kind1: 0,
                _pad_kind2: 0,
                color_r: color.x,
                color_g: color.y,
                color_b: color.z,
                intensity: light.illuminance,
                direction_or_position_x: direction.x,
                direction_or_position_y: direction.y,
                direction_or_position_z: direction.z,
                range: 0.0,
                spot_direction_x: 0.0,
                spot_direction_y: 0.0,
                spot_direction_z: 0.0,
                inner_angle: 0.0,
                outer_angle: 0.0,
                shadow_softness_k: k,
                _pad_tail1: 0.0,
                _pad_tail2: 0.0,
            });
        }
    }

    if toggles.lamp {
        for (light, transform) in &lamp {
            let color = light.color.to_linear().to_vec3();
            let position = transform.translation();
            lights.push(LightGpu {
                kind: LightKindGpu::Point as u32,
                _pad_kind0: 0,
                _pad_kind1: 0,
                _pad_kind2: 0,
                color_r: color.x,
                color_g: color.y,
                color_b: color.z,
                intensity: light.intensity,
                direction_or_position_x: position.x,
                direction_or_position_y: position.y,
                direction_or_position_z: position.z,
                range: light.range,
                spot_direction_x: 0.0,
                spot_direction_y: 0.0,
                spot_direction_z: 0.0,
                inner_angle: 0.0,
                outer_angle: 0.0,
                shadow_softness_k: k,
                _pad_tail1: 0.0,
                _pad_tail2: 0.0,
            });
        }
    }

    if toggles.projector {
        for (light, transform) in &projector {
            let color = light.color.to_linear().to_vec3();
            let position = transform.translation();
            let direction = transform.forward().as_vec3();
            lights.push(LightGpu {
                kind: LightKindGpu::Spot as u32,
                _pad_kind0: 0,
                _pad_kind1: 0,
                _pad_kind2: 0,
                color_r: color.x,
                color_g: color.y,
                color_b: color.z,
                intensity: light.intensity,
                direction_or_position_x: position.x,
                direction_or_position_y: position.y,
                direction_or_position_z: position.z,
                range: light.range,
                spot_direction_x: direction.x,
                spot_direction_y: direction.y,
                spot_direction_z: direction.z,
                inner_angle: light.inner_angle,
                outer_angle: light.outer_angle,
                shadow_softness_k: k,
                _pad_tail1: 0.0,
                _pad_tail2: 0.0,
            });
        }
    }

    out.lights = lights;
    out.shadows_enabled = shadow_config.enabled;
    out.denoise_enabled = denoise_config.enabled;
    out.temporal_enabled = temporal_config.enabled;
    out.temporal_max_history_length = temporal_config.max_history_length;
}
