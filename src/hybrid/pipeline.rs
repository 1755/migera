//! Pipelines, buffers, bind groups, and intermediate targets for the
//! fresh-start hybrid trace + blit pipeline. Freshly authored against
//! `hybrid_legacy::pipeline`'s bind-group/dispatch *shape* as precedent
//! (read-only reference, not reused code) — see this crate's top-level
//! module doc for the self-shading/one-connected-pipeline architecture this
//! implements.
//!
//! Three GPU programs:
//! - `hybrid_trace.wgsl @compute`: one invocation per pixel — BVH traversal
//!   plus per-object sphere-marching, faithfully ported from `cpu_ref.rs`.
//!   Writes direct+emissive HDR color, linear-t depth, world normal, and
//!   indirect-diffuse color (kept separate for the next pass to blur).
//! - `hybrid_denoise.wgsl @compute`: one invocation per pixel — same-frame
//!   edge-aware spatial blur of just the indirect-diffuse term (see
//!   `cpu_ref.rs::blur_indirect_at`'s doc comment), recombined with
//!   direct+emissive into a dedicated denoised-color storage texture.
//! - `hybrid_blit.wgsl @fragment`: fullscreen blit of the denoised color
//!   into the view target, reconstructing `frag_depth` from the stored t
//!   via Bevy's own View uniform (reverse-Z consistent).
//!
//! Bind groups:
//! - group 0 (shared trace+blit): Bevy's own View uniform (the denoise
//!   pass needs no per-view data, so it has no group 0 at all).
//! - group 1 (trace): `SceneUniform` uniform, object/BVH-node storage
//!   buffers (read-only), color+depth+normal+indirect storage textures
//!   (write-only).
//! - group 0 (denoise): `SceneUniform` uniform + color/indirect/normal/
//!   depth as sampled textures + denoised-color storage texture
//!   (write-only) — a fresh bind group, not shared with trace's group 1,
//!   since WGPU disallows binding the same storage texture as write-only
//!   in one pass and read (even via a different view) in another within
//!   one bind group.
//! - group 1 (blit): `SceneUniform` uniform (same buffer) + denoised-
//!   color/depth as sampled textures.

use bevy::core_pipeline::FullscreenShader;
use bevy::core_pipeline::core_3d::CORE_3D_DEPTH_FORMAT;
use bevy::core_pipeline::prepass::{PreviousViewData, PreviousViewUniformOffset, PreviousViewUniforms};
use bevy::math::{UVec2, Vec2, Vec4};
use bevy::prelude::*;
use bevy::render::render_resource::binding_types::{
    sampler, storage_buffer, storage_buffer_read_only, storage_buffer_read_only_sized, texture_2d, texture_storage_2d, uniform_buffer,
};
use bevy::render::render_resource::*;
use bevy::render::renderer::{RenderDevice, RenderQueue};
use bevy::render::view::{ExtractedView, ViewUniform, ViewUniforms};
use bytemuck::Zeroable;

use crate::hybrid::bvh::BvhNodeGpu;
use crate::hybrid::extract::{LightGpu, ObjectGpu, RenderHybridScene, SceneUniform, BACKGROUND_COLOR};

// ---------------------------------------------------------------------------------
// Layouts & pipelines
// ---------------------------------------------------------------------------------

fn hybrid_view_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_view_layout",
        &BindGroupLayoutEntries::single(
            ShaderStages::FRAGMENT | ShaderStages::COMPUTE,
            uniform_buffer::<ViewUniform>(true),
        ),
    )
}

fn hybrid_compute_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_compute_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                storage_buffer_read_only::<ObjectGpu>(false),
                storage_buffer_read_only::<BvhNodeGpu>(false),
                storage_buffer_read_only::<LightGpu>(false),
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // out_color
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // out_depth
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // normal_view
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // indirect_view
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // motion_view (reprojected previous-frame world pos)
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // reflect_view
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // reflect_motion_view (reflected hit's own reprojected previous-frame world pos)
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // refract_view
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // refract_motion_view (transmitted ray's own first-exit reprojected previous-frame world pos)
            ),
        ),
    )
}

/// `hybrid_trace.wgsl`'s own group-2 bind group: read-only access to the
/// DDGI atlas that `hybrid_ddgi_relight.wgsl` already relit THIS frame
/// (relight runs first in `hybrid_pass`, before trace — see
/// `pass::hybrid_pass`'s own dispatch ordering). Read-only here (unlike
/// `hybrid_ddgi_layout`'s own `read_write` atlas binding) since
/// `trace_main` only ever samples this frame's already-relit values,
/// never writes them.
fn hybrid_trace_ddgi_read_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_trace_ddgi_read_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                storage_buffer_read_only::<Vec4>(false), // atlas, array<vec4<f32>>
                storage_buffer_read_only::<Vec2>(false), // distance_atlas, array<vec2<f32>> (mean, mean^2)
                uniform_buffer::<DdgiGridUniform>(false),
            ),
        ),
    )
}

/// `hybrid_trace.wgsl`'s own group-3 bind group: read-only access to the
/// Radiance Cascades atlas that `hybrid_radiance_cascades.wgsl` already
/// relit THIS frame — same "separate group, always bound, only actually
/// sampled when the matching GiMethod is active" shape as
/// `hybrid_trace_ddgi_read_layout` above, kept as its own group (not
/// folded into group 2) since the two techniques' own atlas shapes are
/// unrelated.
fn hybrid_trace_radiance_cascades_read_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_trace_radiance_cascades_read_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                storage_buffer_read_only::<Vec4>(false), // atlas, array<vec4<f32>>
                // levels, WGSL array<CascadeLevelUniform, 4> — a FIXED-SIZE
                // array, unlike every other storage_buffer_read_only::<T>
                // binding in this file (which describe an unsized
                // array<T>): min_binding_size must cover all 4 elements,
                // not element_size::<T>() (storage_buffer_read_only::<T>'s
                // own default), or WGPU's pipeline-layout validation
                // rejects the shader (a real error found via a live
                // gallery.rs --gi-method cascades run: "Buffer structure
                // size 256 ... ended up greater than the given
                // min_binding_size, which is 64" — 256 = 4 * 64).
                storage_buffer_read_only_sized(false, radiance_cascades_levels_min_binding_size()),
            ),
        ),
    )
}

/// The DDGI probe relight pass's own single-group bind group — scene/
/// object/BVH/light data duplicated (per `hybrid_ddgi_relight.wgsl`'s
/// own self-containment header comment, same reason every other pass
/// file in this codebase duplicates rather than shares), plus the atlas
/// and its per-probe history-length buffer as `read_write` storage
/// buffers (valid for storage BUFFERS, unlike storage TEXTURES, which
/// this codebase has already confirmed don't support it anywhere — see
/// `hybrid_denoise_layout`'s own doc comment history — which is also
/// why the atlas itself is a manually-indexed `array<vec4<f32>>`
/// storage buffer rather than a `texture_storage_2d`), deliberately NOT
/// ping-ponged (see `HybridDdgiAtlas`'s own doc comment: each relit
/// probe only ever touches its own texels, so there's no cross-
/// invocation race the way temporal reprojection genuinely has — a
/// probe not relit this frame is simply never touched, so its data
/// persists correctly with no swap to lose track of it).
fn hybrid_ddgi_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_ddgi_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                storage_buffer_read_only::<ObjectGpu>(false),
                storage_buffer_read_only::<BvhNodeGpu>(false),
                storage_buffer_read_only::<LightGpu>(false),
                storage_buffer::<Vec4>(false), // atlas, read_write, array<vec4<f32>>
                storage_buffer::<Vec2>(false), // distance_atlas, read_write, array<vec2<f32>> (mean, mean^2)
                storage_buffer::<f32>(false),  // probe_history_length, read_write
                uniform_buffer::<DdgiGridUniform>(false),
            ),
        ),
    )
}

/// Radiance Cascades' own single-group bind group — mirrors
/// `hybrid_ddgi_layout`'s own shape (scene/object/BVH/light data
/// duplicated per `hybrid_radiance_cascades.wgsl`'s own self-containment
/// header comment), but with a `CascadeLevelUniform` array (one shared
/// atlas buffer, `read_write` for the same "each invocation only ever
/// touches its own texel" reason `hybrid_ddgi_layout`'s own doc comment
/// gives) in place of DDGI's atlas + distance_atlas + history_length +
/// grid-uniform bindings — this experimental technique has no distance
/// atlas (no Chebyshev visibility test, see
/// `radiance_cascades_ref.rs`'s own module doc comment for the
/// deliberately-smaller scope) and no temporal history (relit fresh every
/// frame, see `hybrid_radiance_cascades.wgsl`'s own
/// `radiance_cascades_relight_main` doc comment).
fn hybrid_radiance_cascades_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_radiance_cascades_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                storage_buffer_read_only::<ObjectGpu>(false),
                storage_buffer_read_only::<BvhNodeGpu>(false),
                storage_buffer_read_only::<LightGpu>(false),
                storage_buffer::<Vec4>(false), // atlas, read_write, array<vec4<f32>>
                // levels — same fixed-size-array sizing fix as
                // hybrid_trace_radiance_cascades_read_layout's own
                // identical binding, see that function's own doc comment.
                storage_buffer_read_only_sized(false, radiance_cascades_levels_min_binding_size()),
            ),
        ),
    )
}

/// `min_binding_size` for the `levels` binding shared by
/// `hybrid_radiance_cascades_layout` and
/// `hybrid_trace_radiance_cascades_read_layout` — WGSL's own
/// `array<CascadeLevelUniform, 4>` fixed-size array, so the binding must
/// cover all `RADIANCE_CASCADES_LEVEL_COUNT` elements, not one (see
/// `hybrid_trace_radiance_cascades_read_layout`'s own doc comment for the
/// real validation error this fixes).
fn radiance_cascades_levels_min_binding_size() -> Option<core::num::NonZeroU64> {
    let element_size = <CascadeLevelUniform as ShaderType>::min_size().get();
    core::num::NonZeroU64::new(element_size * crate::hybrid::extract::RADIANCE_CASCADES_LEVEL_COUNT as u64)
}

/// Reads what `hybrid_trace.wgsl`/`hybrid_temporal.wgsl` wrote (color/
/// indirect/normal/depth/history_length, all sampled here as plain
/// `texture_2d` — WGPU disallows binding the same storage texture as
/// both write-only in one pass and read in another within the same bind
/// group, and there is no `read_write` storage texture precedent
/// anywhere in this codebase, see `hybrid_denoise.wgsl`'s own doc
/// comment) and writes the recombined, adaptively-blurred result into a
/// dedicated `denoised_color_view` the blit pass then reads in
/// `color_view`'s place.
///
/// `history_length` here is THIS FRAME's own freshly-written ping-pong
/// slot (`HybridTemporalBindGroup`'s `write` group's own
/// `history_length_out`, not the `read` group's previous-frame one) —
/// this bind group is therefore built alongside the temporal pass's own
/// bind groups in `prepare_hybrid_temporal`, NOT in `prepare_hybrid_scene`
/// where every other bind group in this file lives, since it depends on
/// the same per-frame ping-pong parity the temporal bind groups do (see
/// `HybridTemporalBindGroup`'s own doc comment for why that means
/// rebuilding every frame, not just on resize/buffer-growth).
fn hybrid_denoise_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_denoise_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_2d(TextureSampleType::Float { filterable: false }),
                // accumulated_reflect_view — composited straight into
                // denoised_color_out with NO spatial blur (see
                // hybrid_denoise.wgsl's own doc comment for why mirror
                // reflections should not receive the diffuse bilateral
                // blur: a low-roughness reflection needs sharpness
                // preserved, a high-roughness one already self-blurred
                // via its own wide reflection cone).
                texture_2d(TextureSampleType::Float { filterable: false }),
                // accumulated_refract_view — same "no spatial blur" reasoning
                // as accumulated_reflect_view above, applied to transmission.
                texture_2d(TextureSampleType::Float { filterable: false }),
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly),
            ),
        ),
    )
}

/// Temporal-accumulation pass's own bind group — group 0 (no shared
/// per-view group here, unlike trace/blit: `PreviousViewUniforms` IS this
/// pass's per-view data, bound directly as its own dynamic-offset uniform
/// rather than via a separate view-layout group, since nothing else in
/// this pass needs Bevy's *current*-frame `View` uniform at all — every
/// current-frame quantity it needs (depth, normal, indirect color,
/// motion) already arrived pre-baked from `trace_main`'s own textures).
///
/// Reads this frame's `indirect_view`/`out_depth`/`normal_view`/
/// `motion_view` (written by `trace_main`) plus the previous-frame
/// ping-pong slot's `history_color`/`history_length`/`history_depth`/
/// `history_normal` (all sampled `texture_2d`, matching
/// `hybrid_denoise_layout`'s own "no filtering, exact texel loads, WGPU
/// forbids read+write-binding the same storage texture in one bind
/// group" reasoning) — and `PreviousViewUniforms`, mirroring
/// `hybrid_view_layout`'s existing `uniform_buffer::<ViewUniform>(true)`
/// shape exactly but for last frame's camera matrices instead of this
/// frame's (see `bevy_pbr::prepass::PreviousViewData`/`PreviousViewUniforms`
/// — populated automatically by `PrepassPlugin`, which any `Material`/
/// `PbrPlugin` load always wires up in this app; no extraction of our own
/// needed, matching this module's own "camera/view data needs no
/// extraction of its own" convention already established in `extract.rs`'s
/// doc comment).
///
/// Writes `accumulated_indirect_view` (read next by `hybrid_denoise.wgsl`
/// in `indirect_view`'s place) plus the OTHER ping-pong slot's
/// `history_color`/`history_length`/`history_depth`/`history_normal`,
/// seeding next frame's history.
fn hybrid_temporal_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_temporal_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                uniform_buffer::<PreviousViewData>(true),
                // This frame's trace-pass outputs.
                texture_2d(TextureSampleType::Float { filterable: false }), // indirect_view
                texture_2d(TextureSampleType::Float { filterable: false }), // out_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // normal_view
                texture_2d(TextureSampleType::Float { filterable: false }), // motion_view
                // Previous-frame ping-pong slot (history to blend with).
                texture_2d(TextureSampleType::Float { filterable: false }), // history_color
                texture_2d(TextureSampleType::Float { filterable: false }), // history_length
                texture_2d(TextureSampleType::Float { filterable: false }), // history_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // history_normal
            ),
        ),
    )
}

/// Second bind group (group 1, since WGPU bind groups cap at a limited
/// entry count per group and this pass already has 10 entries in group 0
/// — split for headroom, not because these logically differ from group
/// 0's own entries): this frame's OWN ping-pong slot, written as
/// write-only storage textures, seeding next frame's history read.
fn hybrid_temporal_write_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_temporal_write_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // accumulated_indirect_view
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // history_color (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_length (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_depth (this frame's slot)
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // history_normal (this frame's slot)
            ),
        ),
    )
}

/// Specular reflection's own temporal-accumulation pass bind group —
/// mirrors `hybrid_temporal_layout` exactly in shape (same 10-entry
/// sequence: scene uniform, previous-view uniform, this-frame trace
/// outputs, previous-frame history read slot), but reads `reflect_view`/
/// `reflect_motion_view` in `indirect_view`/`motion_view`'s place. A
/// SEPARATE layout/pipeline/dispatch from the diffuse temporal pass, not
/// a parameterized shared one, since the two channels read/write
/// entirely different textures and buffers — see `HybridReflectHistory`'s
/// own doc comment for why they need independent history in the first
/// place.
fn hybrid_reflect_temporal_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_reflect_temporal_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                uniform_buffer::<PreviousViewData>(true),
                texture_2d(TextureSampleType::Float { filterable: false }), // reflect_view
                texture_2d(TextureSampleType::Float { filterable: false }), // out_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // normal_view
                texture_2d(TextureSampleType::Float { filterable: false }), // reflect_motion_view
                texture_2d(TextureSampleType::Float { filterable: false }), // history_color
                texture_2d(TextureSampleType::Float { filterable: false }), // history_length
                texture_2d(TextureSampleType::Float { filterable: false }), // history_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // history_normal
            ),
        ),
    )
}

/// This frame's own reflection ping-pong write slot, plus
/// `accumulated_reflect_view` — mirrors `hybrid_temporal_write_layout`
/// exactly in shape.
fn hybrid_reflect_temporal_write_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_reflect_temporal_write_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // accumulated_reflect_view
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // history_color (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_length (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_depth (this frame's slot)
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // history_normal (this frame's slot)
            ),
        ),
    )
}

/// Transmission's own temporal-accumulation pass bind group — mirrors
/// `hybrid_reflect_temporal_layout` exactly in shape, reading
/// `refract_view`/`motion_view`(`.w` = refracting_roughness)/`refract_motion_view`
/// in `reflect_view`/`normal_view`/`reflect_motion_view`'s place. A
/// SEPARATE layout/pipeline/dispatch from BOTH the diffuse and reflection
/// temporal passes — see `HybridTransmitHistory`'s own doc comment for
/// why transmission needs independent history from reflection's, not a
/// shared one (their motion characteristics differ: reflection's virtual
/// point is the reflected surface, transmission's is the EXIT surface
/// seen through a bent ray).
fn hybrid_transmit_temporal_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_transmit_temporal_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                uniform_buffer::<PreviousViewData>(true),
                texture_2d(TextureSampleType::Float { filterable: false }), // refract_view
                texture_2d(TextureSampleType::Float { filterable: false }), // out_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // motion_view (.w = refracting_roughness)
                texture_2d(TextureSampleType::Float { filterable: false }), // refract_motion_view
                texture_2d(TextureSampleType::Float { filterable: false }), // history_color
                texture_2d(TextureSampleType::Float { filterable: false }), // history_length
                texture_2d(TextureSampleType::Float { filterable: false }), // history_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // history_normal
            ),
        ),
    )
}

/// This frame's own transmission ping-pong write slot, plus
/// `accumulated_refract_view` — mirrors `hybrid_reflect_temporal_write_layout`
/// exactly in shape.
fn hybrid_transmit_temporal_write_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_transmit_temporal_write_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // accumulated_refract_view
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // history_color (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_length (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_depth (this frame's slot)
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // history_normal (this frame's slot)
            ),
        ),
    )
}

/// Stochastic depth-of-field's own read-side bind group (group 1 in
/// `hybrid_dof.wgsl` — group 0 is the shared `hybrid_view_layout`,
/// reused as-is via the existing per-view `HybridViewBindGroup` rather
/// than a new view bind group, since this pass needs the CURRENT frame's
/// `View` uniform, identical to every other pass that already binds it).
/// Needs its own full `objects`/`bvh` trace machinery (see
/// `hybrid_dof.wgsl`'s own header comment on why this pass duplicates
/// rather than shares `hybrid_trace.wgsl`'s own trace() — self-contained
/// pass files, this codebase's established convention) plus a FILTERING
/// sampler (a genuinely new binding shape in this codebase's compute
/// passes — every existing pass reads via exact `textureLoad`, see
/// `hybrid_denoise_layout`'s own doc comment for why — because DOF's own
/// resample step needs real bilinear filtering at an arbitrary
/// reprojected UV, not a texel snap, or a defocused image would show
/// blocky, unfiltered resampling artifacts).
fn hybrid_dof_read_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_dof_read_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<SceneUniform>(false),
                storage_buffer_read_only::<ObjectGpu>(false),
                storage_buffer_read_only::<BvhNodeGpu>(false),
                texture_2d(TextureSampleType::Float { filterable: true }), // sharp_color_tex (hybrid_denoise.wgsl's own denoised_color_view)
                sampler(SamplerBindingType::Filtering),
                texture_2d(TextureSampleType::Float { filterable: false }), // depth_tex (sharp/unperturbed)
                texture_2d(TextureSampleType::Float { filterable: false }), // normal_tex (sharp/unperturbed)
                // Previous-frame ping-pong slot (history to blend with).
                texture_2d(TextureSampleType::Float { filterable: false }), // history_color
                texture_2d(TextureSampleType::Float { filterable: false }), // history_length
                texture_2d(TextureSampleType::Float { filterable: false }), // history_depth
                texture_2d(TextureSampleType::Float { filterable: false }), // history_normal
            ),
        ),
    )
}

/// Stochastic depth-of-field's own write-side bind group (group 2 in
/// `hybrid_dof.wgsl`) — the resolved DOF color `hybrid_blit.wgsl` reads
/// in `denoised_color_view`'s place, plus this frame's OWN ping-pong
/// slot, seeding next frame's history read.
fn hybrid_dof_write_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_dof_write_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // dof_color_out
                texture_storage_2d(TextureFormat::Rgba16Float, StorageTextureAccess::WriteOnly), // history_color (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_length (this frame's slot)
                texture_storage_2d(TextureFormat::R32Float, StorageTextureAccess::WriteOnly), // history_depth (this frame's slot)
                texture_storage_2d(TextureFormat::Rgba32Float, StorageTextureAccess::WriteOnly), // history_normal (this frame's slot)
            ),
        ),
    )
}

fn hybrid_blit_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "hybrid_blit_layout",
        &BindGroupLayoutEntries::sequential(
            ShaderStages::FRAGMENT,
            (
                uniform_buffer::<SceneUniform>(false),
                texture_2d(TextureSampleType::Float { filterable: true }),
                // Bilinear sampler for color_tex — needed once trace
                // resolution can differ from the real output resolution
                // (RenderScaleConfig experiment, step 1b): this pass's own
                // fragment shader upscales via textureSampleLevel instead
                // of the flat textureLoad it used before that experiment.
                // Bit-for-bit equal to the old textureLoad at
                // RenderScaleConfig::scale == 1.0 (a bilinear sample
                // exactly AT a texel center returns that texel unchanged).
                sampler(SamplerBindingType::Filtering),
                texture_2d(TextureSampleType::Float { filterable: false }),
            ),
        ),
    )
}

#[derive(Resource)]
pub struct HybridPipeline {
    pub view_layout: BindGroupLayoutDescriptor,
    pub compute_layout: BindGroupLayoutDescriptor,
    pub trace_ddgi_read_layout: BindGroupLayoutDescriptor,
    pub trace_radiance_cascades_read_layout: BindGroupLayoutDescriptor,
    pub ddgi_layout: BindGroupLayoutDescriptor,
    pub radiance_cascades_layout: BindGroupLayoutDescriptor,
    pub denoise_layout: BindGroupLayoutDescriptor,
    pub temporal_read_layout: BindGroupLayoutDescriptor,
    pub temporal_write_layout: BindGroupLayoutDescriptor,
    pub reflect_temporal_read_layout: BindGroupLayoutDescriptor,
    pub reflect_temporal_write_layout: BindGroupLayoutDescriptor,
    pub transmit_temporal_read_layout: BindGroupLayoutDescriptor,
    pub transmit_temporal_write_layout: BindGroupLayoutDescriptor,
    pub dof_read_layout: BindGroupLayoutDescriptor,
    pub dof_write_layout: BindGroupLayoutDescriptor,
    /// Filtering sampler for `hybrid_dof.wgsl`'s own bilinear resample
    /// step — see `hybrid_dof_read_layout`'s own doc comment for why
    /// this pass alone (among this codebase's compute passes) needs
    /// real texture filtering rather than exact `textureLoad`.
    pub dof_sampler: Sampler,
    pub blit_layout: BindGroupLayoutDescriptor,
    pub blit_shader: Handle<Shader>,
    pub fullscreen_shader: FullscreenShader,
    /// Queued at startup; polled per frame until compiled.
    pub trace_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled.
    pub denoise_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled.
    pub temporal_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled. Same shader
    /// FILE as `temporal_pipeline` (`hybrid_temporal.wgsl`), different
    /// entry point (`reflect_temporal_main`) — see that function's own
    /// doc comment for why it's a separate entry point in the same file
    /// rather than a new `.wgsl` file.
    pub reflect_temporal_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled. Same shader
    /// FILE as `temporal_pipeline`/`reflect_temporal_pipeline`
    /// (`hybrid_temporal.wgsl`), different entry point
    /// (`transmit_temporal_main`).
    pub transmit_temporal_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled. Only
    /// dispatched when `GiMethod::Ddgi` is the active technique — see
    /// `pass::hybrid_pass`'s own dispatch-level gate.
    pub ddgi_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled. Only
    /// dispatched when `GiMethod::RadianceCascades` is the active
    /// technique — see `pass::hybrid_pass`'s own dispatch-level gate.
    pub radiance_cascades_pipeline: CachedComputePipelineId,
    /// Queued at startup; polled per frame until compiled. Runs even
    /// when `DofConfig::enabled` is false — see `hybrid_dof.wgsl`'s own
    /// `scene.dof_enabled == 0u` copy-through branch (same "always
    /// dispatch, coefficient/flag says do nothing" convention this
    /// codebase already establishes elsewhere).
    pub dof_pipeline: CachedComputePipelineId,
}

#[derive(Clone, PartialEq, Eq, Hash)]
pub struct HybridBlitKey {
    pub target_format: TextureFormat,
    pub has_depth: bool,
}

impl SpecializedRenderPipeline for HybridPipeline {
    type Key = HybridBlitKey;

    fn specialize(&self, key: Self::Key) -> RenderPipelineDescriptor {
        RenderPipelineDescriptor {
            label: Some("hybrid_blit_pipeline".into()),
            layout: vec![self.view_layout.clone(), self.blit_layout.clone()],
            vertex: self.fullscreen_shader.to_vertex_state(),
            fragment: Some(FragmentState {
                shader: self.blit_shader.clone(),
                entry_point: Some(if key.has_depth { "fragment" } else { "fragment_nodepth" }.into()),
                targets: vec![Some(ColorTargetState {
                    format: key.target_format,
                    blend: None,
                    write_mask: ColorWrites::ALL,
                })],
                ..default()
            }),
            primitive: PrimitiveState::default(),
            // frag_depth requires a depth-stencil state; Always+write
            // because hybrid content unconditionally owns this frame's
            // depth (reverse-Z values come from the same projection matrix
            // Bevy uses, via the real View uniform).
            depth_stencil: key.has_depth.then(|| DepthStencilState {
                format: CORE_3D_DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(CompareFunction::Always),
                stencil: StencilState::default(),
                bias: DepthBiasState::default(),
            }),
            multisample: MultisampleState::default(),
            immediate_size: 0,
            zero_initialize_workgroup_memory: false,
        }
    }
}

pub fn init_hybrid_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    fullscreen_shader: Res<FullscreenShader>,
    pipeline_cache: ResMut<PipelineCache>,
    render_device: Res<RenderDevice>,
) {
    let trace_shader = asset_server.load("shaders/hybrid_trace.wgsl");
    let compute_layout = hybrid_compute_layout();
    let trace_ddgi_read_layout = hybrid_trace_ddgi_read_layout();
    let trace_radiance_cascades_read_layout = hybrid_trace_radiance_cascades_read_layout();
    let trace_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_trace_pipeline".into()),
        layout: vec![hybrid_view_layout(), compute_layout.clone(), trace_ddgi_read_layout.clone(), trace_radiance_cascades_read_layout.clone()],
        shader: trace_shader,
        entry_point: Some("trace_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let ddgi_shader = asset_server.load("shaders/hybrid_ddgi_relight.wgsl");
    let ddgi_layout = hybrid_ddgi_layout();
    let ddgi_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_ddgi_pipeline".into()),
        layout: vec![ddgi_layout.clone()],
        shader: ddgi_shader,
        entry_point: Some("ddgi_relight_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let radiance_cascades_shader = asset_server.load("shaders/hybrid_radiance_cascades.wgsl");
    let radiance_cascades_layout = hybrid_radiance_cascades_layout();
    let radiance_cascades_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_radiance_cascades_pipeline".into()),
        layout: vec![radiance_cascades_layout.clone()],
        shader: radiance_cascades_shader,
        entry_point: Some("radiance_cascades_relight_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let denoise_shader = asset_server.load("shaders/hybrid_denoise.wgsl");
    let denoise_layout = hybrid_denoise_layout();
    let denoise_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_denoise_pipeline".into()),
        layout: vec![denoise_layout.clone()],
        shader: denoise_shader,
        entry_point: Some("denoise_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let temporal_shader = asset_server.load("shaders/hybrid_temporal.wgsl");
    let temporal_read_layout = hybrid_temporal_layout();
    let temporal_write_layout = hybrid_temporal_write_layout();
    let temporal_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_temporal_pipeline".into()),
        layout: vec![temporal_read_layout.clone(), temporal_write_layout.clone()],
        shader: temporal_shader.clone(),
        entry_point: Some("temporal_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let reflect_temporal_read_layout = hybrid_reflect_temporal_layout();
    let reflect_temporal_write_layout = hybrid_reflect_temporal_write_layout();
    let reflect_temporal_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_reflect_temporal_pipeline".into()),
        layout: vec![reflect_temporal_read_layout.clone(), reflect_temporal_write_layout.clone()],
        shader: temporal_shader.clone(),
        entry_point: Some("reflect_temporal_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let transmit_temporal_read_layout = hybrid_transmit_temporal_layout();
    let transmit_temporal_write_layout = hybrid_transmit_temporal_write_layout();
    let transmit_temporal_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_transmit_temporal_pipeline".into()),
        layout: vec![transmit_temporal_read_layout.clone(), transmit_temporal_write_layout.clone()],
        shader: temporal_shader,
        entry_point: Some("transmit_temporal_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });

    let dof_shader = asset_server.load("shaders/hybrid_dof.wgsl");
    let dof_read_layout = hybrid_dof_read_layout();
    let dof_write_layout = hybrid_dof_write_layout();
    let dof_pipeline = pipeline_cache.queue_compute_pipeline(ComputePipelineDescriptor {
        label: Some("hybrid_dof_pipeline".into()),
        layout: vec![hybrid_view_layout(), dof_read_layout.clone(), dof_write_layout.clone()],
        shader: dof_shader,
        entry_point: Some("dof_main".into()),
        shader_defs: vec![],
        immediate_size: 0,
        zero_initialize_workgroup_memory: false,
    });
    let dof_sampler = render_device.create_sampler(&SamplerDescriptor {
        mag_filter: FilterMode::Linear,
        min_filter: FilterMode::Linear,
        ..default()
    });

    commands.insert_resource(HybridPipeline {
        view_layout: hybrid_view_layout(),
        compute_layout,
        trace_ddgi_read_layout,
        trace_radiance_cascades_read_layout,
        ddgi_layout,
        radiance_cascades_layout,
        denoise_layout,
        temporal_read_layout,
        temporal_write_layout,
        reflect_temporal_read_layout,
        reflect_temporal_write_layout,
        transmit_temporal_read_layout,
        transmit_temporal_write_layout,
        dof_read_layout,
        dof_write_layout,
        dof_sampler,
        blit_layout: hybrid_blit_layout(),
        blit_shader: asset_server.load("shaders/hybrid_blit.wgsl"),
        fullscreen_shader: fullscreen_shader.clone(),
        trace_pipeline,
        denoise_pipeline,
        temporal_pipeline,
        reflect_temporal_pipeline,
        transmit_temporal_pipeline,
        ddgi_pipeline,
        radiance_cascades_pipeline,
        dof_pipeline,
    });
}

// ---------------------------------------------------------------------------------
// Buffers, targets, bind groups
// ---------------------------------------------------------------------------------

#[derive(Resource)]
pub struct HybridBuffers {
    pub scene_uniform: UniformBuffer<SceneUniform>,
    pub objects: RawBufferVec<ObjectGpu>,
    pub nodes: RawBufferVec<BvhNodeGpu>,
    pub lights: RawBufferVec<LightGpu>,
}

#[derive(Resource, Default)]
pub struct HybridTargetsRes(pub Option<HybridTargets>);

pub struct HybridTargets {
    pub size: UVec2,
    pub color_view: TextureView,
    pub depth_view: TextureView,
    pub normal_view: TextureView,
    pub indirect_view: TextureView,
    pub denoised_color_view: TextureView,
    /// Written by `trace_main`: this pixel's REPROJECTED PREVIOUS-FRAME
    /// WORLD position (the per-object rigid-motion undo/reapply step,
    /// `temporal_ref::reproject_world_point`'s WGSL mirror) — NOT yet a
    /// screen-space UV; `hybrid_temporal.wgsl` finishes the reprojection
    /// into last frame's screen space itself (it has `PreviousViewData`
    /// bound, `trace_main` doesn't). See `temporal_ref.rs`'s own doc
    /// comment for the full two-pass split and why.
    pub motion_view: TextureView,
    /// Written by the temporal-accumulation pass: this frame's
    /// temporally-blended indirect color, read next by
    /// `hybrid_denoise.wgsl` in `indirect_view`'s place.
    pub accumulated_indirect_view: TextureView,
    /// Written by `trace_main`: this pixel's raw multi-bounce specular
    /// reflection color (`ShadeResult.reflect`) — kept SEPARATE from
    /// `indirect_view` since it needs its OWN temporal history (see
    /// `HybridReflectHistory`'s own doc comment for why reflection motion
    /// does not follow the reflecting surface's own motion).
    pub reflect_view: TextureView,
    /// Written by `trace_main`: the REFLECTED hit's own reprojected
    /// previous-frame world position (a "virtual point" — reflects the
    /// motion of whatever geometry the reflection ray actually hit, NOT
    /// the reflecting surface's own motion). Same two-pass reprojection
    /// split as `motion_view` (finished into previous-frame screen UV by
    /// `hybrid_temporal.wgsl`'s own `reflect_temporal_main`).
    pub reflect_motion_view: TextureView,
    /// Written by the temporal-accumulation pass: this frame's
    /// temporally-blended reflection color, composited directly into
    /// `denoised_color_view` by `hybrid_denoise.wgsl` (no spatial blur —
    /// see that file's own doc comment for why mirror reflections should
    /// not receive the diffuse bilateral blur).
    pub accumulated_reflect_view: TextureView,
    /// Written by `trace_main`: this pixel's raw multi-bounce transmission/
    /// refraction color (`ShadeResult.refract`) — kept SEPARATE from both
    /// `indirect_view` and `reflect_view` for the identical reason
    /// `reflect_view` already is (see `HybridTransmitHistory`'s own doc
    /// comment for why transmission needs its own dedicated history too).
    pub refract_view: TextureView,
    /// Written by `trace_main`: the transmitted ray's own first-EXIT hit
    /// reprojected previous-frame world position — same "virtual point"
    /// idea as `reflect_motion_view`, keyed to the EXIT surface (possibly
    /// a different, moving object seen through a bent ray) rather than
    /// the entry surface.
    pub refract_motion_view: TextureView,
    /// Written by the transmission temporal-accumulation pass: this
    /// frame's temporally-blended transmission color, composited directly
    /// into `denoised_color_view` by `hybrid_denoise.wgsl` (no spatial
    /// blur — same reasoning as `accumulated_reflect_view`).
    pub accumulated_refract_view: TextureView,
    /// Written by `hybrid_dof.wgsl`: `denoised_color_view` (the sharp,
    /// fully-composited image) with stochastic depth-of-field resolved
    /// on top — read by `hybrid_blit.wgsl` in `denoised_color_view`'s
    /// own place. Bit-for-bit equal to `denoised_color_view` when
    /// `DofConfig::enabled` is false (`hybrid_dof.wgsl`'s own copy-
    /// through branch) — see that shader's own header comment.
    pub dof_color_view: TextureView,
}

/// One ping-pong slot of the temporal-accumulation history buffer —
/// see `HybridHistory`'s own doc comment for why two of these exist and
/// why they're a SEPARATE resource from `HybridTargets`.
pub struct HybridHistorySlot {
    pub color_view: TextureView,
    pub length_view: TextureView,
    pub depth_view: TextureView,
    pub normal_view: TextureView,
}

/// Ping-ponged (A/B) temporal history: the accumulate pass reads ONE
/// slot (last frame's result) while writing the OTHER (this frame's
/// result, becoming "last frame's" for the NEXT frame) — required, not
/// merely convenient, because reprojection reads history at a DIFFERENT
/// UV than the pass writes at, so different invocations cross-read each
/// other's texels; an in-place single-texture read+write would race
/// under WGPU's unordered-invocation model (same reasoning
/// `hybrid_denoise.wgsl`'s own doc comment already establishes for why
/// `denoised_color_view` couldn't be `out_color` itself).
///
/// A SEPARATE resource from `HybridTargets` (not folded in): a resize
/// must RESET history content (there is no valid previous-frame data at
/// a new resolution), unlike `HybridTargets`' own resize path, which
/// only ever recreates same-shaped, this-frame-only textures with no
/// cross-frame semantic content to invalidate.
pub struct HybridHistory {
    pub size: UVec2,
    /// Which `GiMethod` (its own `repr(u32)` discriminant) the currently
    /// accumulated history was built from — see `prepare_hybrid_temporal`'s
    /// own doc comment for why a `gi_method` CHANGE, not just a resize,
    /// must also reset this history: a real, found-by-direct-visual-
    /// inspection light-leak bug (`gi_room.rs`'s own fully sealed, roof-
    /// closed room reading as lit) traced back to switching `GiMethod`
    /// live leaving the PREVIOUS technique's own (possibly much
    /// brighter, e.g. pre-fix cone-tracing's own sky-leak) blended
    /// history sitting in this buffer, which then kept blending into
    /// the NEWLY selected technique's own correct fresh values for up
    /// to `temporal_max_history_length` frames — different techniques'
    /// indirect values are not comparable to blend across, the same way
    /// a resize's own stale-resolution content isn't.
    pub gi_method: u32,
    /// Same invalidation reasoning as `gi_method` above, extended to
    /// cone tracing's own `max_bounces` — a real, found-by-direct-
    /// visual-inspection bug (`gi_room.rs`'s own sealed, roof-closed
    /// room reading as lit/ghosted once `max_bounces` was raised past
    /// 1 live) traced back to the identical root cause `gi_method`'s
    /// own doc comment describes: changing `max_bounces` left the
    /// PREVIOUS bounce count's own blended history sitting in this
    /// buffer, which then kept blending into the newly selected bounce
    /// count's own correct fresh values for up to
    /// `temporal_max_history_length` frames — a 1-bounce and a 2-/3-
    /// bounce indirect value are not comparable to blend across, for
    /// the identical reason two different `GiMethod`s' values aren't.
    pub conetrace_max_bounces: u32,
    pub slots: [HybridHistorySlot; 2],
}

#[derive(Resource, Default)]
pub struct HybridHistoryRes(pub Option<HybridHistory>);

/// Ping-ponged (A/B) temporal history for the specular reflection
/// channel — a SEPARATE buffer from `HybridHistory` (diffuse indirect),
/// not a shared/parameterized one, because reflection needs a
/// DIFFERENT reprojection: `HybridHistory`'s own `motion_view` reprojects
/// using the REFLECTING SURFACE's own rigid motion (correct for diffuse
/// GI, which lives ON that surface), but a reflected image's apparent
/// motion depends on the REFLECTED geometry's own motion and the
/// viewer's parallax, not the mirror surface's motion at all — bolting
/// reflection into `HybridHistory` would reproject it with the wrong
/// object's motion the moment either the mirror or the reflected object
/// moves, causing ghosting distinct from (and worse than) ordinary
/// disocclusion. See `reflect_motion_view`'s own doc comment on
/// `HybridTargets` for the "virtual point" this reprojects instead.
pub struct HybridReflectHistory {
    pub size: UVec2,
    /// Same invalidation reasoning as `HybridHistory::gi_method`, applied
    /// to reflection's own `max_bounces` config (a 1-bounce and a 2-/3-
    /// bounce reflection value are not comparable to blend across).
    pub reflection_max_bounces: u32,
    pub slots: [HybridHistorySlot; 2],
}

#[derive(Resource, Default)]
pub struct HybridReflectHistoryRes(pub Option<HybridReflectHistory>);

/// Independent ping-pong parity counter for the reflection history —
/// SEPARATE from `HybridFrameParity` (not shared) so a future resize of
/// one history buffer alone (unlikely today, since both always resize
/// together with `HybridTargets`, but the two histories' own
/// invalidation conditions already differ — `gi_method`/`conetrace_
/// max_bounces` vs. `reflection_max_bounces` — so their parity counters
/// are kept independent on the same principle, avoiding a hidden
/// coupling that would only bite once the two conditions genuinely
/// diverge in a future change).
#[derive(Resource, Default)]
pub struct HybridReflectFrameParity(pub u32);

/// Ping-ponged (A/B) temporal history for the transmission/refraction
/// channel — a SEPARATE buffer from BOTH `HybridHistory` (diffuse
/// indirect) and `HybridReflectHistory` (specular reflection), not a
/// shared one, because transmission needs a THIRD DIFFERENT
/// reprojection: the virtual point here is the EXIT surface's own motion
/// (possibly a different, moving object seen through a bent ray), which
/// matches neither the entry surface's own rigid motion (diffuse GI's
/// case) nor the REFLECTED surface's motion (reflection's case) — see
/// `refract_motion_view`'s own doc comment on `HybridTargets` for the
/// "virtual point" this reprojects instead.
pub struct HybridTransmitHistory {
    pub size: UVec2,
    /// Same invalidation reasoning as `HybridReflectHistory::
    /// reflection_max_bounces`, applied to transmission's own
    /// `max_bounces` config.
    pub transmission_max_bounces: u32,
    pub slots: [HybridHistorySlot; 2],
}

#[derive(Resource, Default)]
pub struct HybridTransmitHistoryRes(pub Option<HybridTransmitHistory>);

/// Independent ping-pong parity counter for the transmission history —
/// SEPARATE from both `HybridFrameParity` and `HybridReflectFrameParity`,
/// same reasoning as `HybridReflectFrameParity`'s own doc comment.
#[derive(Resource, Default)]
pub struct HybridTransmitFrameParity(pub u32);

/// Ping-ponged (A/B) temporal history for stochastic depth-of-field — a
/// SEPARATE buffer from `HybridHistory`/`HybridReflectHistory`/
/// `HybridTransmitHistory`, not a shared one, because DOF's own
/// disocclusion test runs against a genuinely different signal: the
/// SHARP (unperturbed) primary ray's own depth/normal (see
/// `hybrid_dof.wgsl`'s own header comment for why this must stay
/// completely independent of the jittered ray fired only to determine
/// defocus displacement). Also converges over its own window
/// (`DofConfig::max_history_length`, typically much larger than the GI
/// accumulator's own — DOF's per-frame perturbation, a full aperture-
/// radius resample-position jump, is much bigger than GI's own
/// hemisphere-sample noise and needs more frames to fully converge).
pub struct HybridDofHistory {
    pub size: UVec2,
    /// Same invalidation reasoning as `HybridReflectHistory::
    /// reflection_max_bounces` — a resolution change invalidates history
    /// content outright. `DofConfig::enabled`/`focal_distance`/
    /// `aperture_f_stops` do NOT need to invalidate this history the
    /// same way: `hybrid_dof.wgsl`'s own disocclusion-rejection test
    /// already forces a reset in one frame flat once
    /// `scene.dof_enabled == 0u` (its own copy-through branch stores
    /// `history_length = 0`), and a live focal-distance/aperture change
    /// is meant to visibly re-converge over the next few frames exactly
    /// like a live GI config change already does elsewhere in this
    /// codebase (`ConeTraceConfig`'s own live-tunable egui sliders never
    /// force a hard history reset either) — not a correctness bug to
    /// guard against, an expected/desired live-tuning behavior.
    pub slots: [HybridHistorySlot; 2],
}

#[derive(Resource, Default)]
pub struct HybridDofHistoryRes(pub Option<HybridDofHistory>);

/// Independent ping-pong parity counter for the DOF history — same
/// reasoning as `HybridReflectFrameParity`'s own doc comment.
#[derive(Resource, Default)]
pub struct HybridDofFrameParity(pub u32);

/// Which `HybridHistory` slot index holds "last frame's" result this
/// frame — flips every frame (`prepare_hybrid_temporal`). Render-world-
/// local (not extracted from Bevy's own `FrameCount`), matching
/// `HybridTargetsRes`'s own fully-render-world-local convention: nothing
/// outside this pass's own bookkeeping needs to know or drive this
/// value.
#[derive(Resource, Default)]
pub struct HybridFrameParity(pub u32);

/// Render-world frame counter driving WHICH `probes_per_frame`-sized
/// subset of the DDGI probe grid relights each frame (see
/// `ddgi_ref::ddgi_probe_relight_start`'s own doc comment) — a plain
/// wrapping `u32`, incremented once per frame in `prepare_hybrid_scene`
/// (the system that already builds `SceneUniform` every frame).
/// Deliberately NOT `HybridFrameParity` reused: that one is mod-2 by
/// construction (ping-pong slot selection) and lives in a different
/// system's local scope — coupling two independent concerns onto one
/// counter would make either one harder to reason about separately.
#[derive(Resource, Default)]
pub struct HybridDdgiFrameIndex(pub u32);

/// The DDGI probe atlas + its per-probe history-length buffer — a
/// SINGLE copy of each, deliberately NOT ping-ponged. Each relit probe
/// only ever reads/writes its OWN texels, so there is no cross-
/// invocation race the way `HybridHistorySlot`'s reprojection genuinely
/// has. The atlas is a manually-indexed `array<vec4<f32>>` storage
/// buffer (`atlas_pixels * atlas_pixels` texels, row-major) rather than
/// a texture, since `read_write` access isn't available on storage
/// textures in this codebase.
pub struct HybridDdgiAtlas {
    pub atlas_pixels: u32,
    pub total_probes: u32,
    pub tile_size: u32,
    pub tiles_per_row: u32,
    pub grid: crate::hybrid::ddgi_ref::ProbeGrid,
    pub atlas: Buffer,
    /// `array<vec2<f32>>`, same row-major indexing as `atlas` — per-texel
    /// `(mean_distance, mean_distance_squared)`, the Chebyshev depth-
    /// visibility test's own input (see `ddgi_ref::sample_probe_grid`'s
    /// own doc comment for why a single hard occlusion ray isn't enough
    /// on its own: it only catches "something sits between the shaded
    /// point and the probe," not "this probe's own stored irradiance is
    /// itself unreliable because its relight rays skimmed through/near
    /// thin geometry"). A SEPARATE buffer from `atlas` rather than a
    /// wider per-texel stride in the same one, since `atlas`'s own
    /// `vec4<f32>` already uses all 4 channels for irradiance's `.rgb`
    /// (`.a` reserved, unused) — packing 2 more floats in would need a
    /// `vec4` -> larger-struct promotion touching every existing atlas
    /// read/write site, versus one new buffer touching only the sites
    /// that actually need distance.
    pub distance_atlas: Buffer,
    pub history_length: Buffer,
    pub grid_uniform: UniformBuffer<DdgiGridUniform>,
}

#[derive(Resource, Default)]
pub struct HybridDdgiAtlasRes(pub Option<HybridDdgiAtlas>);

/// Radiance Cascades' own atlas — ONE shared buffer across all
/// `RADIANCE_CASCADES_LEVEL_COUNT` levels (each level's own exact-fit
/// tile packing occupying a consecutive sub-region, see
/// `CascadeLevelUniform::atlas_texel_offset`'s own doc comment), plus the
/// `CascadeLevelUniform` array itself. A single shared buffer (not one
/// per level, and not ping-ponged) for the same "simple, not production-
/// polish" scoping this whole technique's own module doc comment
/// establishes — 4 small levels don't need 4 separate GPU allocations
/// tracked independently, and `read_write` in-place is correct here too
/// (each invocation only ever writes its own texel, see
/// `hybrid_radiance_cascades_layout`'s own doc comment).
pub struct HybridRadianceCascadesAtlas {
    pub level_count: u32,
    pub total_atlas_texels: u32,
    pub atlas: Buffer,
    pub levels: RawBufferVec<CascadeLevelUniform>,
}

#[derive(Resource, Default)]
pub struct HybridRadianceCascadesAtlasRes(pub Option<HybridRadianceCascadesAtlas>);

#[derive(Resource)]
pub struct HybridRadianceCascadesBindGroup {
    pub value: BindGroup,
}

/// `hybrid_trace.wgsl`'s own group-3 read-only view into the same atlas
/// `HybridRadianceCascadesBindGroup` writes — mirrors
/// `HybridTraceDdgiBindGroup`'s own doc comment for why the same buffer
/// is safely bound read_write in one pass and read-only in the other
/// within a single frame (each relit texel is only ever touched by its
/// own invocation).
#[derive(Resource)]
pub struct HybridTraceRadianceCascadesBindGroup {
    pub value: BindGroup,
}

/// GPU-uploadable mirror of `hybrid_ddgi_relight.wgsl`'s own
/// `DdgiGridUniform` struct — grid placement + exact-fit atlas layout,
/// grid-config-scoped rather than per-frame, so kept as its own small
/// uniform rather than folded into `SceneUniform`.
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct DdgiGridUniform {
    pub origin_x: f32,
    pub origin_y: f32,
    pub origin_z: f32,
    pub _pad0: f32,
    pub spacing_x: f32,
    pub spacing_y: f32,
    pub spacing_z: f32,
    pub tiles_per_row: u32,
    pub dims_x: u32,
    pub dims_y: u32,
    pub dims_z: u32,
    pub _pad2: u32,
}

/// GPU-uploadable mirror of `hybrid_radiance_cascades.wgsl`'s own
/// `CascadeLevelUniform` struct, field-for-field (including padding, to
/// match WGSL's own `vec3<f32>`/`vec3<u32>` alignment) — one entry per
/// cascade level (`RADIANCE_CASCADES_LEVEL_COUNT`), built from
/// `radiance_cascades_ref::cascade_level_params`/`cascade_grid_from_bounds`
/// plus this level's own exact-fit atlas sub-region within the ONE shared
/// atlas buffer (see `HybridRadianceCascadesAtlas`'s own doc comment for
/// why levels share a buffer rather than each getting their own).
#[derive(Clone, Copy, Debug, Default, ShaderType, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
pub struct CascadeLevelUniform {
    pub probe_spacing: f32,
    pub ray_count: u32,
    pub interval_near: f32,
    pub interval_far: f32,
    pub grid_origin_x: f32,
    pub grid_origin_y: f32,
    pub grid_origin_z: f32,
    pub _pad0: f32,
    pub grid_dims_x: u32,
    pub grid_dims_y: u32,
    pub grid_dims_z: u32,
    pub _pad1: u32,
    pub tile_size: u32,
    pub tiles_per_row: u32,
    pub atlas_texel_offset: u32,
    pub total_probes: u32,
}

#[derive(Resource)]
pub struct HybridComputeBindGroup {
    pub value: BindGroup,
}

#[derive(Resource)]
pub struct HybridDenoiseBindGroup {
    pub value: BindGroup,
}

#[derive(Resource)]
pub struct HybridBlitBindGroup {
    pub value: BindGroup,
}

/// The DDGI relight pass's own single-group bind group. Rebuilt EVERY
/// frame — grid placement can shift as the scene's own root AABB moves
/// (see `extract_hybrid_scene`'s own DDGI section), so `DdgiGridUniform`
/// needs a fresh upload each frame even when the atlas buffer itself
/// doesn't need reallocating.
#[derive(Resource)]
pub struct HybridDdgiBindGroup {
    pub value: BindGroup,
}

/// `hybrid_trace.wgsl`'s own group-2 read-only view into the same atlas
/// `HybridDdgiBindGroup` writes — NOT ping-ponged, see `HybridDdgiAtlas`'s
/// own doc comment for why the same buffer is safely bound read_write in
/// one pass and read-only in the other within a single frame.
#[derive(Resource)]
pub struct HybridTraceDdgiBindGroup {
    pub value: BindGroup,
}

/// Both of the temporal-accumulation pass's bind groups (group 0: this
/// frame's trace outputs + previous-frame history, read-only; group 1:
/// this frame's own write targets). Rebuilt EVERY frame, unlike every
/// other `Hybrid*BindGroup` resource in this file — ping-pong parity
/// flips which physical texture is "history" vs. "this frame's write
/// target" every single frame, so the existing skip-when-unchanged
/// optimization `prepare_hybrid_scene` applies to
/// `HybridComputeBindGroup`/`HybridDenoiseBindGroup`/`HybridBlitBindGroup`
/// (valid there because those bind groups' underlying textures never
/// change IDENTITY frame-to-frame absent a resize) does not apply here.
#[derive(Resource)]
pub struct HybridTemporalBindGroup {
    pub read: BindGroup,
    pub write: BindGroup,
}

/// Specular reflection's own temporal-accumulation bind groups — mirrors
/// `HybridTemporalBindGroup` exactly (same "rebuilt every frame, ping-pong
/// parity" reasoning), but a SEPARATE resource since the two channels'
/// underlying textures/history buffers are entirely independent (see
/// `HybridReflectHistory`'s own doc comment).
#[derive(Resource)]
pub struct HybridReflectTemporalBindGroup {
    pub read: BindGroup,
    pub write: BindGroup,
}

/// Transmission's own temporal-accumulation bind groups — mirrors
/// `HybridReflectTemporalBindGroup` exactly, but a SEPARATE resource
/// since transmission's own textures/history buffer are independent of
/// both diffuse GI's and reflection's (see `HybridTransmitHistory`'s own
/// doc comment).
#[derive(Resource)]
pub struct HybridTransmitTemporalBindGroup {
    pub read: BindGroup,
    pub write: BindGroup,
}

/// Stochastic depth-of-field's own resolve+accumulate bind groups — see
/// `HybridDofHistory`'s own doc comment for why this needs an
/// independent history from the GI/reflection/transmission channels.
#[derive(Resource)]
pub struct HybridDofBindGroup {
    pub read: BindGroup,
    pub write: BindGroup,
}

#[derive(Component)]
pub struct HybridViewBindGroup {
    pub value: BindGroup,
}

pub fn init_hybrid_buffers(mut commands: Commands) {
    commands.insert_resource(HybridBuffers {
        scene_uniform: UniformBuffer::default(),
        objects: RawBufferVec::new(BufferUsages::STORAGE),
        nodes: RawBufferVec::new(BufferUsages::STORAGE),
        lights: RawBufferVec::new(BufferUsages::STORAGE),
    });
    commands.init_resource::<HybridTargetsRes>();
    commands.init_resource::<HybridHistoryRes>();
    commands.init_resource::<HybridFrameParity>();
    commands.init_resource::<HybridDdgiFrameIndex>();
    commands.init_resource::<HybridDdgiAtlasRes>();
    commands.init_resource::<HybridRadianceCascadesAtlasRes>();
    commands.init_resource::<HybridReflectHistoryRes>();
    commands.init_resource::<HybridReflectFrameParity>();
    commands.init_resource::<HybridTransmitHistoryRes>();
    commands.init_resource::<HybridTransmitFrameParity>();
    commands.init_resource::<HybridDofHistoryRes>();
    commands.init_resource::<HybridDofFrameParity>();
}

/// Per-view group 0: Bevy's own View uniform, exactly like the legacy pass.
pub fn prepare_hybrid_view_bind_groups(
    mut commands: Commands,
    render_device: Res<RenderDevice>,
    pipeline_cache: Res<PipelineCache>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    view_uniforms: Res<ViewUniforms>,
    views: Query<Entity, With<ExtractedView>>,
) {
    let Some(hp) = hybrid_pipeline else { return };
    let Some(view_binding) = view_uniforms.uniforms.binding() else {
        return;
    };
    for entity in &views {
        let bind_group = render_device.create_bind_group(
            "hybrid_view_bind_group",
            &pipeline_cache.get_bind_group_layout(&hp.view_layout),
            &BindGroupEntries::single(view_binding.clone()),
        );
        commands.entity(entity).insert(HybridViewBindGroup { value: bind_group });
    }
}

/// Uploads this frame's scene (from `RenderHybridScene`, populated by
/// `extract::extract_hybrid_scene`), recreates storage textures on resize,
/// and builds both group-1 bind groups.
#[allow(clippy::too_many_arguments)]
pub fn prepare_hybrid_scene(
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    pipeline_cache: Res<PipelineCache>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    mut buffers: ResMut<HybridBuffers>,
    mut targets: ResMut<HybridTargetsRes>,
    scene_data: Res<RenderHybridScene>,
    extracted_views: Query<&ExtractedView>,
    existing_compute_bg: Option<Res<HybridComputeBindGroup>>,
    existing_blit_bg: Option<Res<HybridBlitBindGroup>>,
    mut ddgi_frame_index: ResMut<HybridDdgiFrameIndex>,
    mut commands: Commands,
) {
    let Some(hp) = hybrid_pipeline else { return };
    ddgi_frame_index.0 = ddgi_frame_index.0.wrapping_add(1);

    // `RawBufferVec::write_buffer` only allocates a new `wgpu::Buffer` when
    // capacity needs to grow (see its `reserve`'s `capacity > self.capacity`
    // check) — capacity never shrinks, so at a stable-or-shrinking object/
    // node count the same `Buffer` is reused frame to frame. Bind groups
    // referencing a buffer are only invalidated when that buffer's identity
    // changes, so tracking capacity before/after lets bind-group creation
    // below be skipped whenever nothing it depends on actually changed —
    // real, measured cost at `--stress 10000` (`prepare_hybrid_scene`'s own
    // window was 1.1-3.4ms, of which two `create_bind_group` calls are a
    // component), not a hypothetical one.
    let objects_capacity_before = buffers.objects.capacity();
    let nodes_capacity_before = buffers.nodes.capacity();
    let lights_capacity_before = buffers.lights.capacity();

    buffers.objects.clear();
    for o in &scene_data.objects {
        buffers.objects.push(*o);
    }
    if buffers.objects.is_empty() {
        // Empty-scene placeholder so the storage buffer is never
        // zero-sized (WGPU disallows a zero-size binding); the shader
        // early-outs on object_count == 0 / node_count == 0 regardless, so
        // an all-zero record (including inv_rotation as a zero quaternion,
        // never actually evaluated) is fine — `ObjectGpu: Zeroable` makes
        // this a one-liner instead of naming every field by hand.
        buffers.objects.push(ObjectGpu::zeroed());
    }
    buffers.objects.write_buffer(&render_device, &render_queue);

    buffers.nodes.clear();
    for n in &scene_data.nodes {
        buffers.nodes.push(*n);
    }
    if buffers.nodes.is_empty() {
        buffers.nodes.push(BvhNodeGpu {
            min_x: 0.0,
            min_y: 0.0,
            min_z: 0.0,
            max_x: 0.0,
            max_y: 0.0,
            max_z: 0.0,
            left_or_sentinel: crate::hybrid::bvh::LEAF_SENTINEL,
            right_or_object: 0,
        });
    }
    buffers.nodes.write_buffer(&render_device, &render_queue);

    buffers.lights.clear();
    for l in &scene_data.lights {
        buffers.lights.push(*l);
    }
    if buffers.lights.is_empty() {
        // Same empty-scene placeholder rationale as `objects` above —
        // `light_count == 0` in `SceneUniform` makes the shader skip the
        // shading loop entirely regardless of this placeholder's content.
        buffers.lights.push(LightGpu::zeroed());
    }
    buffers.lights.write_buffer(&render_device, &render_queue);

    let (res_x, res_y) = extracted_views
        .iter()
        .next()
        .map(|v| (v.viewport.z as f32, v.viewport.w as f32))
        .unwrap_or((1280.0, 720.0));

    // The trace pass's own working resolution — see `SceneUniform::
    // trace_size_x`'s own doc comment for why this can differ from the
    // real `(res_x, res_y)` viewport size. `.max(1.0)` guards the
    // degenerate case of an extremely small real viewport combined with a
    // small `render_scale` rounding to zero.
    let trace_size = bevy::math::Vec2::new(res_x, res_y) * scene_data.render_scale;
    let trace_size = bevy::math::UVec2::new(trace_size.x.round().max(1.0) as u32, trace_size.y.round().max(1.0) as u32);

    // Texel-to-NDC jitter conversion happens here, not in `extract.rs` —
    // this is the only place in the render-world prepare/extract pipeline
    // that already has the real viewport size on hand (see `RenderHybrid
    // Scene::jitter_frame_index`'s own doc comment for why). Sized against
    // TRACE resolution, not the real viewport — the jitter needs to move
    // by a fraction of a TRACE texel (the grid ray-gen actually samples
    // on), not a fraction of a real output pixel; at `render_scale < 1.0`
    // those are different sizes.
    let jitter_offset = if scene_data.jitter_enabled {
        let jitter_texels = crate::hybrid::taa_ref::taa_jitter_offset(scene_data.jitter_frame_index, scene_data.jitter_ring_size);
        crate::hybrid::taa_ref::jitter_texels_to_ndc(jitter_texels, trace_size.as_vec2())
    } else {
        bevy::math::Vec2::ZERO
    };

    buffers.scene_uniform.set(SceneUniform {
        object_count: scene_data.objects.len() as u32,
        bvh_node_count: scene_data.nodes.len() as u32,
        light_count: scene_data.lights.len() as u32,
        shadows_enabled: scene_data.shadows_enabled as u32,
        background_r: BACKGROUND_COLOR.x,
        background_g: BACKGROUND_COLOR.y,
        background_b: BACKGROUND_COLOR.z,
        denoise_enabled: scene_data.denoise_enabled as u32,
        temporal_enabled: scene_data.temporal_enabled as u32,
        temporal_max_history_length: scene_data.temporal_max_history_length,
        gi_method: scene_data.gi_method,
        conetrace_half_angle: scene_data.conetrace_half_angle,
        conetrace_origin_radius: scene_data.conetrace_origin_radius,
        conetrace_max_t: scene_data.conetrace_max_t,
        conetrace_max_bounces: scene_data.conetrace_max_bounces,
        reflection_enabled: scene_data.reflection_enabled as u32,
        reflection_max_bounces: scene_data.reflection_max_bounces,
        reflection_fresnel_cutoff: scene_data.reflection_fresnel_cutoff,
        reflection_max_t: scene_data.reflection_max_t,
        transmission_enabled: scene_data.transmission_enabled as u32,
        transmission_max_bounces: scene_data.transmission_max_bounces,
        transmission_fresnel_cutoff: scene_data.transmission_fresnel_cutoff,
        transmission_max_t: scene_data.transmission_max_t,
        ddgi_probes_per_frame: scene_data.ddgi_probes_per_frame,
        ddgi_total_probes: scene_data.ddgi_grid.probe_count() as u32,
        ddgi_tile_size: scene_data.ddgi_tile_size,
        ddgi_frame_index: ddgi_frame_index.0,
        ddgi_max_history_length: scene_data.ddgi_max_history_length,
        ddgi_max_t: scene_data.ddgi_max_t,
        dof_enabled: scene_data.dof_enabled as u32,
        dof_focal_distance: scene_data.dof_focal_distance,
        dof_aperture_radius: scene_data.dof_aperture_radius,
        dof_frame_index: scene_data.dof_frame_index,
        dof_max_history_length: scene_data.dof_max_history_length,
        jitter_enabled: scene_data.jitter_enabled as u32,
        jitter_offset_x: jitter_offset.x,
        jitter_offset_y: jitter_offset.y,
        trace_size_x: trace_size.x,
        trace_size_y: trace_size.y,
    });
    buffers.scene_uniform.write_buffer(&render_device, &render_queue);

    // (Re)create storage textures on size change. Sized against TRACE
    // resolution (see `SceneUniform::trace_size_x`'s own doc comment) —
    // equal to the real viewport size exactly when `render_scale == 1.0`.
    let want = trace_size;
    let needs_new = match targets.0.as_ref() {
        Some(t) => t.size != want,
        None => true,
    };
    if needs_new && want.x > 0 && want.y > 0 {
        let make = |format: TextureFormat, label: &str| {
            render_device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d { width: want.x, height: want.y, depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::STORAGE_BINDING | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        targets.0 = Some(HybridTargets {
            size: want,
            color_view: make(TextureFormat::Rgba16Float, "hybrid_color"),
            depth_view: make(TextureFormat::R32Float, "hybrid_depth"),
            normal_view: make(TextureFormat::Rgba32Float, "hybrid_normal"),
            indirect_view: make(TextureFormat::Rgba16Float, "hybrid_indirect"),
            denoised_color_view: make(TextureFormat::Rgba16Float, "hybrid_denoised_color"),
            motion_view: make(TextureFormat::Rgba32Float, "hybrid_motion"),
            accumulated_indirect_view: make(TextureFormat::Rgba16Float, "hybrid_accumulated_indirect"),
            reflect_view: make(TextureFormat::Rgba16Float, "hybrid_reflect"),
            reflect_motion_view: make(TextureFormat::Rgba32Float, "hybrid_reflect_motion"),
            accumulated_reflect_view: make(TextureFormat::Rgba16Float, "hybrid_accumulated_reflect"),
            refract_view: make(TextureFormat::Rgba16Float, "hybrid_refract"),
            refract_motion_view: make(TextureFormat::Rgba32Float, "hybrid_refract_motion"),
            accumulated_refract_view: make(TextureFormat::Rgba16Float, "hybrid_accumulated_refract"),
            dof_color_view: make(TextureFormat::Rgba16Float, "hybrid_dof_color"),
        });
    }
    let Some(targets) = targets.0.as_ref() else {
        return;
    };

    // Bind groups only need rebuilding when something they actually
    // reference changed identity: a storage texture resize (`needs_new`,
    // above) or a storage buffer reallocation (`RawBufferVec::write_buffer`
    // only allocates a new `Buffer` when capacity grows — see this
    // function's earlier comment). The `scene_uniform`'s own buffer never
    // reallocates after its first `write_buffer` (`UniformBuffer` is a
    // single fixed-size slot), so it isn't part of this check. Skipping
    // recreation when nothing changed avoids two `create_bind_group` calls
    // (and their `Resource` reinsertion) every single frame regardless of
    // whether the scene is static.
    let buffers_grew = buffers.objects.capacity() > objects_capacity_before
        || buffers.nodes.capacity() > nodes_capacity_before
        || buffers.lights.capacity() > lights_capacity_before;
    let bind_groups_exist = existing_compute_bg.is_some() && existing_blit_bg.is_some();
    if !needs_new && !buffers_grew && bind_groups_exist {
        return;
    }

    let Some(scene_binding) = buffers.scene_uniform.binding() else {
        return;
    };
    let Some(objects_binding) = buffers.objects.buffer().map(|b| b.as_entire_binding()) else {
        return;
    };
    let Some(nodes_binding) = buffers.nodes.buffer().map(|b| b.as_entire_binding()) else {
        return;
    };
    let Some(lights_binding) = buffers.lights.buffer().map(|b| b.as_entire_binding()) else {
        return;
    };

    let compute_bg = render_device.create_bind_group(
        "hybrid_compute_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.compute_layout),
        &BindGroupEntries::sequential((
            scene_binding.clone(),
            objects_binding,
            nodes_binding,
            lights_binding,
            &targets.color_view,
            &targets.depth_view,
            &targets.normal_view,
            &targets.indirect_view,
            &targets.motion_view,
            &targets.reflect_view,
            &targets.reflect_motion_view,
            &targets.refract_view,
            &targets.refract_motion_view,
        )),
    );
    commands.insert_resource(HybridComputeBindGroup { value: compute_bg });

    let blit_bg = render_device.create_bind_group(
        "hybrid_blit_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.blit_layout),
        // Reads dof_color_view (hybrid_dof.wgsl's own resolved output),
        // NOT denoised_color_view directly — see HybridTargets::
        // dof_color_view's own doc comment: bit-for-bit equal to
        // denoised_color_view when DofConfig::enabled is false, so this
        // swap is safe regardless of whether DOF is actually active.
        &BindGroupEntries::sequential((scene_binding, &targets.dof_color_view, &hp.dof_sampler, &targets.depth_view)),
    );
    commands.insert_resource(HybridBlitBindGroup { value: blit_bg });
}

/// (Re)allocates the DDGI atlas + per-probe history-length buffer
/// whenever `total_probes`/`tile_size` changes (a scene-bounds change or
/// a `DdgiConfig::tile_size` edit — either reshapes the atlas), then
/// uploads the grid-config-scoped `DdgiGridUniform` and rebuilds
/// `HybridDdgiBindGroup`/`HybridTraceDdgiBindGroup` — UNCONDITIONALLY
/// every frame, same reasoning as `prepare_hybrid_temporal`'s own doc
/// comment (grid placement can shift every frame as the scene's own root
/// AABB moves, so the small `DdgiGridUniform` upload needs to happen
/// every frame even when the atlas buffer itself doesn't need
/// reallocating).
#[allow(clippy::too_many_arguments)]
pub fn prepare_hybrid_ddgi(
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    pipeline_cache: Res<PipelineCache>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    buffers: Res<HybridBuffers>,
    scene_data: Res<RenderHybridScene>,
    mut atlas: ResMut<HybridDdgiAtlasRes>,
    mut commands: Commands,
) {
    let Some(hp) = hybrid_pipeline else { return };
    let Some(scene_binding) = buffers.scene_uniform.binding() else { return };

    let total_probes = (scene_data.ddgi_grid.probe_count() as u32).max(1);
    let tile_size = scene_data.ddgi_tile_size.max(1);
    let layout = crate::hybrid::ddgi_ref::AtlasLayout::exact_fit(total_probes, tile_size);

    let needs_new = match atlas.0.as_ref() {
        Some(a) => a.total_probes != total_probes || a.tile_size != tile_size,
        None => true,
    };
    if needs_new {
        // array<vec4<f32>> sized atlas_pixels * atlas_pixels, row-major —
        // see hybrid_ddgi_layout's own doc comment for why this is a
        // manually-indexed buffer, not a texture.
        let atlas_texel_count = (layout.atlas_pixels as u64) * (layout.atlas_pixels as u64);
        let atlas_byte_size = atlas_texel_count * 16; // vec4<f32>
        let atlas_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("hybrid_ddgi_atlas"),
            size: atlas_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&atlas_buffer, 0, &vec![0u8; atlas_byte_size as usize]);

        // array<vec2<f32>> sized atlas_pixels * atlas_pixels, row-major —
        // same indexing as atlas above, see HybridDdgiAtlas::
        // distance_atlas's own doc comment. Zero-initialized like every
        // other freshly (re)created buffer here: a fresh (mean=0,
        // mean_sq=0) texel makes chebyshev_visibility_weight discount
        // any real (dist > 0) shaded point toward zero, which is exactly
        // consistent with that same fresh probe's irradiance ALSO
        // starting black and history_length starting at 0 — "no history
        // yet" already means "don't trust this probe," this just extends
        // that same convention to the new visibility signal too.
        let distance_atlas_byte_size = atlas_texel_count * 8; // vec2<f32>
        let distance_atlas_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("hybrid_ddgi_distance_atlas"),
            size: distance_atlas_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&distance_atlas_buffer, 0, &vec![0u8; distance_atlas_byte_size as usize]);

        // Zero-initialize: a freshly (re)created grid's probes all start
        // at history_length == 0 — "no history yet," the same fallback
        // `temporal_ref::temporal_blend`'s own `history_length == 0`
        // contract already establishes for a brand-new/rejected pixel,
        // applied here at probe granularity.
        let history_byte_size = (total_probes as u64) * 4; // f32 per probe
        let history_length_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("hybrid_ddgi_history_length"),
            size: history_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&history_length_buffer, 0, &vec![0u8; history_byte_size as usize]);

        atlas.0 = Some(HybridDdgiAtlas {
            atlas_pixels: layout.atlas_pixels,
            total_probes,
            tile_size,
            tiles_per_row: layout.tiles_per_row,
            grid: scene_data.ddgi_grid,
            atlas: atlas_buffer,
            distance_atlas: distance_atlas_buffer,
            history_length: history_length_buffer,
            grid_uniform: UniformBuffer::default(),
        });
    }
    let Some(atlas) = atlas.0.as_mut() else { return };

    // Grid placement can change even when total_probes/tile_size don't
    // (the scene's own root AABB shifts frame to frame as objects move —
    // see extract_hybrid_scene's own DDGI section) — upload every frame,
    // cheap (one small uniform, same class of cost SceneUniform's own
    // per-frame upload already pays).
    atlas.grid = scene_data.ddgi_grid;
    atlas.grid_uniform.set(DdgiGridUniform {
        origin_x: atlas.grid.origin.x,
        origin_y: atlas.grid.origin.y,
        origin_z: atlas.grid.origin.z,
        _pad0: 0.0,
        spacing_x: atlas.grid.spacing.x,
        spacing_y: atlas.grid.spacing.y,
        spacing_z: atlas.grid.spacing.z,
        tiles_per_row: atlas.tiles_per_row,
        dims_x: atlas.grid.dims.x,
        dims_y: atlas.grid.dims.y,
        dims_z: atlas.grid.dims.z,
        _pad2: 0,
    });
    atlas.grid_uniform.write_buffer(&render_device, &render_queue);
    let Some(grid_binding) = atlas.grid_uniform.binding() else { return };

    let Some(objects_binding) = buffers.objects.buffer().map(|b| b.as_entire_binding()) else { return };
    let Some(nodes_binding) = buffers.nodes.buffer().map(|b| b.as_entire_binding()) else { return };
    let Some(lights_binding) = buffers.lights.buffer().map(|b| b.as_entire_binding()) else { return };

    // NOT ping-ponged — the SAME atlas/history_length buffer is bound
    // read_write here and read-only in trace_main's own group 2 below.
    // See HybridDdgiAtlas's own doc comment for why this is correct
    // (each probe only ever touches its own texels) and why the prior
    // ping-ponged version was a real bug.
    let bg = render_device.create_bind_group(
        "hybrid_ddgi_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.ddgi_layout),
        &BindGroupEntries::sequential((
            scene_binding,
            objects_binding,
            nodes_binding,
            lights_binding,
            atlas.atlas.as_entire_buffer_binding(),
            atlas.distance_atlas.as_entire_buffer_binding(),
            atlas.history_length.as_entire_buffer_binding(),
            grid_binding.clone(),
        )),
    );
    commands.insert_resource(HybridDdgiBindGroup { value: bg });

    let trace_ddgi_bg = render_device.create_bind_group(
        "hybrid_trace_ddgi_read_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.trace_ddgi_read_layout),
        &BindGroupEntries::sequential((atlas.atlas.as_entire_buffer_binding(), atlas.distance_atlas.as_entire_buffer_binding(), grid_binding)),
    );
    commands.insert_resource(HybridTraceDdgiBindGroup { value: trace_ddgi_bg });
}

/// Builds this frame's 4 `CascadeLevelUniform` entries and (re)allocates
/// the shared atlas buffer when the total texel count changes — same
/// "rebuild the small per-level uniforms every frame, only reallocate the
/// big buffer on an actual size change" split `prepare_hybrid_ddgi`
/// already establishes, applied here to a whole level ARRAY instead of
/// one grid uniform. Unconditional every frame (not gated on
/// `GiMethod::RadianceCascades` being active): the atlas/levels buffers
/// need to exist and be correctly sized before `hybrid_pass`'s own
/// dispatch-level gate can even check the pipeline is ready, mirroring
/// `prepare_hybrid_ddgi`'s identical unconditional-every-frame shape.
#[allow(clippy::too_many_arguments)]
pub fn prepare_hybrid_radiance_cascades(
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
    pipeline_cache: Res<PipelineCache>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    buffers: Res<HybridBuffers>,
    scene_data: Res<RenderHybridScene>,
    mut atlas: ResMut<HybridRadianceCascadesAtlasRes>,
    mut commands: Commands,
) {
    let Some(hp) = hybrid_pipeline else { return };
    let Some(scene_binding) = buffers.scene_uniform.binding() else { return };

    let bounds = crate::prim::Aabb { min: scene_data.radiance_cascades_root_bounds_min, max: scene_data.radiance_cascades_root_bounds_max };
    let level_count = crate::hybrid::extract::RADIANCE_CASCADES_LEVEL_COUNT;

    // Build all 4 levels' own geometry + exact-fit atlas sub-region,
    // laying each level's own region out consecutively within one shared
    // buffer (atlas_texel_offset accumulates as we go) — mirrors
    // AtlasLayout::exact_fit's own per-level sizing, just summed across
    // levels instead of computed once for a single grid.
    let mut level_uniforms: Vec<CascadeLevelUniform> = Vec::with_capacity(level_count as usize);
    let mut running_offset: u32 = 0;
    for level in 0..level_count {
        let level_params = crate::hybrid::radiance_cascades_ref::cascade_level_params(
            level,
            scene_data.radiance_cascades_base_spacing,
            scene_data.radiance_cascades_base_ray_count,
            scene_data.radiance_cascades_base_interval,
        );
        let grid = crate::hybrid::radiance_cascades_ref::cascade_grid_from_bounds(bounds, level_params);
        let total_probes = (grid.probe_count() as u32).max(1);
        let tile_size = crate::hybrid::radiance_cascades_ref::cascade_tile_side(level_params.ray_count).max(1);
        let layout = crate::hybrid::ddgi_ref::AtlasLayout::exact_fit(total_probes, tile_size);
        let level_texel_count = layout.atlas_pixels * layout.atlas_pixels;

        level_uniforms.push(CascadeLevelUniform {
            probe_spacing: level_params.probe_spacing,
            ray_count: level_params.ray_count,
            interval_near: level_params.interval_near,
            interval_far: level_params.interval_far,
            grid_origin_x: grid.origin.x,
            grid_origin_y: grid.origin.y,
            grid_origin_z: grid.origin.z,
            _pad0: 0.0,
            grid_dims_x: grid.dims.x,
            grid_dims_y: grid.dims.y,
            grid_dims_z: grid.dims.z,
            _pad1: 0,
            tile_size,
            tiles_per_row: layout.tiles_per_row,
            atlas_texel_offset: running_offset,
            total_probes,
        });
        running_offset += level_texel_count;
    }
    let total_atlas_texels = running_offset.max(1);

    let needs_new = match atlas.0.as_ref() {
        Some(a) => a.total_atlas_texels != total_atlas_texels,
        None => true,
    };
    if needs_new {
        // array<vec4<f32>> sized total_atlas_texels, row-major within
        // each level's own consecutive region — see
        // hybrid_radiance_cascades_layout's own doc comment for why this
        // is a manually-indexed buffer, not a texture (same reasoning
        // HybridDdgiAtlas's own doc comment gives).
        let atlas_byte_size = (total_atlas_texels as u64) * 16; // vec4<f32>
        let atlas_buffer = render_device.create_buffer(&BufferDescriptor {
            label: Some("hybrid_radiance_cascades_atlas"),
            size: atlas_byte_size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        render_queue.write_buffer(&atlas_buffer, 0, &vec![0u8; atlas_byte_size as usize]);

        atlas.0 = Some(HybridRadianceCascadesAtlas {
            level_count,
            total_atlas_texels,
            atlas: atlas_buffer,
            levels: RawBufferVec::new(BufferUsages::STORAGE),
        });
    }
    let Some(atlas) = atlas.0.as_mut() else { return };

    atlas.levels.clear();
    for lvl in &level_uniforms {
        atlas.levels.push(*lvl);
    }
    atlas.levels.write_buffer(&render_device, &render_queue);

    let Some(objects_binding) = buffers.objects.buffer().map(|b| b.as_entire_binding()) else { return };
    let Some(nodes_binding) = buffers.nodes.buffer().map(|b| b.as_entire_binding()) else { return };
    let Some(lights_binding) = buffers.lights.buffer().map(|b| b.as_entire_binding()) else { return };
    let Some(levels_binding) = atlas.levels.buffer().map(|b| b.as_entire_binding()) else { return };

    let bg = render_device.create_bind_group(
        "hybrid_radiance_cascades_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.radiance_cascades_layout),
        &BindGroupEntries::sequential((scene_binding, objects_binding, nodes_binding, lights_binding, atlas.atlas.as_entire_buffer_binding(), levels_binding.clone())),
    );
    commands.insert_resource(HybridRadianceCascadesBindGroup { value: bg });

    let trace_bg = render_device.create_bind_group(
        "hybrid_trace_radiance_cascades_read_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.trace_radiance_cascades_read_layout),
        &BindGroupEntries::sequential((atlas.atlas.as_entire_buffer_binding(), levels_binding)),
    );
    commands.insert_resource(HybridTraceRadianceCascadesBindGroup { value: trace_bg });
}

/// (Re)creates the ping-pong history textures on resize, on a
/// `GiMethod` change, OR on a `max_bounces` change (all three reset
/// history content — there is no valid previous-frame data at a new
/// resolution, and neither a different GI technique's own indirect
/// values nor a different bounce count's own indirect values are
/// comparable to blend across, see `HybridHistory::gi_method`'s and
/// `HybridHistory::conetrace_max_bounces`'s own doc comments for the
/// real light-leak/ghosting bugs this fixes — unlike `HybridTargets`'
/// own resize path, which only ever recreates same-shaped, this-frame-
/// only textures with no cross-frame semantic content to invalidate), flips
/// `HybridFrameParity`, and rebuilds `HybridTemporalBindGroup` —
/// UNCONDITIONALLY every frame, unlike `prepare_hybrid_scene`'s bind
/// groups, since ping-pong parity means "which physical texture is
/// history" changes every single frame (see `HybridTemporalBindGroup`'s
/// own doc comment for why the usual skip-when-unchanged optimization
/// doesn't apply here).
#[allow(clippy::too_many_arguments)]
pub fn prepare_hybrid_temporal(
    render_device: Res<RenderDevice>,
    pipeline_cache: Res<PipelineCache>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    buffers: Res<HybridBuffers>,
    targets: Res<HybridTargetsRes>,
    scene_data: Res<RenderHybridScene>,
    mut history: ResMut<HybridHistoryRes>,
    mut parity: ResMut<HybridFrameParity>,
    mut reflect_history: ResMut<HybridReflectHistoryRes>,
    mut reflect_parity: ResMut<HybridReflectFrameParity>,
    mut transmit_history: ResMut<HybridTransmitHistoryRes>,
    mut transmit_parity: ResMut<HybridTransmitFrameParity>,
    previous_view_uniforms: Res<PreviousViewUniforms>,
    views: Query<&PreviousViewUniformOffset>,
    mut commands: Commands,
) {
    let Some(hp) = hybrid_pipeline else { return };
    let Some(targets) = targets.0.as_ref() else { return };
    let Some(scene_binding) = buffers.scene_uniform.binding() else { return };
    let Some(previous_view_binding) = previous_view_uniforms.uniforms.binding() else { return };
    let Some(previous_view_offset) = views.iter().next() else { return };

    let want = targets.size;
    let needs_new = match history.0.as_ref() {
        Some(h) => h.size != want || h.gi_method != scene_data.gi_method || h.conetrace_max_bounces != scene_data.conetrace_max_bounces,
        None => true,
    };
    if needs_new && want.x > 0 && want.y > 0 {
        let make = |format: TextureFormat, label: &str| {
            render_device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d { width: want.x, height: want.y, depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::STORAGE_BINDING | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        let make_slot = |label_prefix: &str| HybridHistorySlot {
            color_view: make(TextureFormat::Rgba16Float, &format!("{label_prefix}_color")),
            length_view: make(TextureFormat::R32Float, &format!("{label_prefix}_length")),
            depth_view: make(TextureFormat::R32Float, &format!("{label_prefix}_depth")),
            normal_view: make(TextureFormat::Rgba32Float, &format!("{label_prefix}_normal")),
        };
        history.0 = Some(HybridHistory {
            size: want,
            gi_method: scene_data.gi_method,
            conetrace_max_bounces: scene_data.conetrace_max_bounces,
            slots: [make_slot("hybrid_history_a"), make_slot("hybrid_history_b")],
        });
        // A resize, a GiMethod change, OR a max_bounces change
        // invalidates any in-flight history — restart parity at 0 so
        // both slots start from a
        // consistent, freshly-created state
        // (their content is undefined until the temporal pass's own
        // `SceneUniform::temporal_enabled`-gated first write, exactly
        // like a brand-new pixel's `history_length == 0` fallback in
        // `temporal_ref::temporal_blend`).
        parity.0 = 0;
    }
    let Some(history) = history.0.as_ref() else { return };

    // Flip parity every frame: this frame reads whatever slot last frame
    // WROTE, and writes into the other one, becoming next frame's read
    // slot in turn. Incrementing here (not on a resize-only path) is
    // what keeps ping-pong alternating every single frame regardless of
    // whether a resize also happened this frame.
    parity.0 = parity.0.wrapping_add(1);
    let read_slot = &history.slots[(parity.0 % 2) as usize];
    let write_slot = &history.slots[((parity.0 + 1) % 2) as usize];

    let read_bg = render_device.create_bind_group(
        "hybrid_temporal_read_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.temporal_read_layout),
        &BindGroupEntries::sequential((
            scene_binding.clone(),
            previous_view_binding.clone(),
            &targets.indirect_view,
            &targets.depth_view,
            &targets.normal_view,
            &targets.motion_view,
            &read_slot.color_view,
            &read_slot.length_view,
            &read_slot.depth_view,
            &read_slot.normal_view,
        )),
    );
    let write_bg = render_device.create_bind_group(
        "hybrid_temporal_write_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.temporal_write_layout),
        &BindGroupEntries::sequential((
            &targets.accumulated_indirect_view,
            &write_slot.color_view,
            &write_slot.length_view,
            &write_slot.depth_view,
            &write_slot.normal_view,
        )),
    );
    commands.insert_resource(HybridTemporalBindGroup { read: read_bg, write: write_bg });

    // Specular reflection's own ping-pong history — same resize/
    // invalidation shape as the diffuse history just above, gated on
    // `reflection_max_bounces` instead of `gi_method`/`conetrace_
    // max_bounces` (see `HybridReflectHistory`'s own doc comment for why
    // this is a genuinely separate buffer, not a shared/parameterized
    // one).
    let reflect_needs_new = match reflect_history.0.as_ref() {
        Some(h) => h.size != want || h.reflection_max_bounces != scene_data.reflection_max_bounces,
        None => true,
    };
    if reflect_needs_new && want.x > 0 && want.y > 0 {
        let make = |format: TextureFormat, label: &str| {
            render_device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d { width: want.x, height: want.y, depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::STORAGE_BINDING | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        let make_slot = |label_prefix: &str| HybridHistorySlot {
            color_view: make(TextureFormat::Rgba16Float, &format!("{label_prefix}_color")),
            length_view: make(TextureFormat::R32Float, &format!("{label_prefix}_length")),
            depth_view: make(TextureFormat::R32Float, &format!("{label_prefix}_depth")),
            normal_view: make(TextureFormat::Rgba32Float, &format!("{label_prefix}_normal")),
        };
        reflect_history.0 = Some(HybridReflectHistory {
            size: want,
            reflection_max_bounces: scene_data.reflection_max_bounces,
            slots: [make_slot("hybrid_reflect_history_a"), make_slot("hybrid_reflect_history_b")],
        });
        reflect_parity.0 = 0;
    }
    if let Some(reflect_history_ref) = reflect_history.0.as_ref() {
        reflect_parity.0 = reflect_parity.0.wrapping_add(1);
        let read_slot = &reflect_history_ref.slots[(reflect_parity.0 % 2) as usize];
        let write_slot = &reflect_history_ref.slots[((reflect_parity.0 + 1) % 2) as usize];

        let read_bg = render_device.create_bind_group(
            "hybrid_reflect_temporal_read_bind_group",
            &pipeline_cache.get_bind_group_layout(&hp.reflect_temporal_read_layout),
            &BindGroupEntries::sequential((
                scene_binding.clone(),
                previous_view_binding.clone(),
                &targets.reflect_view,
                &targets.depth_view,
                &targets.normal_view,
                &targets.reflect_motion_view,
                &read_slot.color_view,
                &read_slot.length_view,
                &read_slot.depth_view,
                &read_slot.normal_view,
            )),
        );
        let write_bg = render_device.create_bind_group(
            "hybrid_reflect_temporal_write_bind_group",
            &pipeline_cache.get_bind_group_layout(&hp.reflect_temporal_write_layout),
            &BindGroupEntries::sequential((
                &targets.accumulated_reflect_view,
                &write_slot.color_view,
                &write_slot.length_view,
                &write_slot.depth_view,
                &write_slot.normal_view,
            )),
        );
        commands.insert_resource(HybridReflectTemporalBindGroup { read: read_bg, write: write_bg });
    }

    // Transmission's own ping-pong history — same resize/invalidation
    // shape as reflection's own just above, gated on
    // `transmission_max_bounces` instead of `reflection_max_bounces` (see
    // `HybridTransmitHistory`'s own doc comment for why this is a
    // genuinely separate buffer from both diffuse GI's and reflection's).
    let transmit_needs_new = match transmit_history.0.as_ref() {
        Some(h) => h.size != want || h.transmission_max_bounces != scene_data.transmission_max_bounces,
        None => true,
    };
    if transmit_needs_new && want.x > 0 && want.y > 0 {
        let make = |format: TextureFormat, label: &str| {
            render_device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d { width: want.x, height: want.y, depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::STORAGE_BINDING | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        let make_slot = |label_prefix: &str| HybridHistorySlot {
            color_view: make(TextureFormat::Rgba16Float, &format!("{label_prefix}_color")),
            length_view: make(TextureFormat::R32Float, &format!("{label_prefix}_length")),
            depth_view: make(TextureFormat::R32Float, &format!("{label_prefix}_depth")),
            normal_view: make(TextureFormat::Rgba32Float, &format!("{label_prefix}_normal")),
        };
        transmit_history.0 = Some(HybridTransmitHistory {
            size: want,
            transmission_max_bounces: scene_data.transmission_max_bounces,
            slots: [make_slot("hybrid_transmit_history_a"), make_slot("hybrid_transmit_history_b")],
        });
        transmit_parity.0 = 0;
    }
    if let Some(transmit_history_ref) = transmit_history.0.as_ref() {
        transmit_parity.0 = transmit_parity.0.wrapping_add(1);
        let read_slot = &transmit_history_ref.slots[(transmit_parity.0 % 2) as usize];
        let write_slot = &transmit_history_ref.slots[((transmit_parity.0 + 1) % 2) as usize];

        let read_bg = render_device.create_bind_group(
            "hybrid_transmit_temporal_read_bind_group",
            &pipeline_cache.get_bind_group_layout(&hp.transmit_temporal_read_layout),
            &BindGroupEntries::sequential((
                scene_binding.clone(),
                previous_view_binding,
                &targets.refract_view,
                &targets.depth_view,
                &targets.motion_view,
                &targets.refract_motion_view,
                &read_slot.color_view,
                &read_slot.length_view,
                &read_slot.depth_view,
                &read_slot.normal_view,
            )),
        );
        let write_bg = render_device.create_bind_group(
            "hybrid_transmit_temporal_write_bind_group",
            &pipeline_cache.get_bind_group_layout(&hp.transmit_temporal_write_layout),
            &BindGroupEntries::sequential((
                &targets.accumulated_refract_view,
                &write_slot.color_view,
                &write_slot.length_view,
                &write_slot.depth_view,
                &write_slot.normal_view,
            )),
        );
        commands.insert_resource(HybridTransmitTemporalBindGroup { read: read_bg, write: write_bg });
    }

    // Denoise pass's own bind group lives here, not prepare_hybrid_scene
    // (where every OTHER bind group in this file is built) — see
    // hybrid_denoise_layout's own doc comment for why: it needs THIS
    // frame's own freshly-written history_length (write_slot, the same
    // ping-pong slot the temporal pass above just wrote into), which only
    // exists once parity has been resolved here. Reflection's own
    // `accumulated_reflect_view` needs no history_length input here (no
    // spatial blur applies to it — see hybrid_denoise_layout's own doc
    // comment on that binding), so it's added as a plain extra read, not
    // gated on the reflect-history ping-pong above having run this frame.
    let denoise_bg = render_device.create_bind_group(
        "hybrid_denoise_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.denoise_layout),
        &BindGroupEntries::sequential((
            scene_binding,
            &targets.accumulated_indirect_view,
            &targets.normal_view,
            &targets.depth_view,
            &targets.color_view,
            &write_slot.length_view,
            &targets.accumulated_reflect_view,
            &targets.accumulated_refract_view,
            &targets.denoised_color_view,
        )),
    );
    commands.insert_resource(HybridDenoiseBindGroup { value: denoise_bg });

    // Suppress the "assigned but only read via .iter().next()" concern:
    // `previous_view_offset`'s dynamic-offset VALUE is consumed by
    // `hybrid_pass` (via the same `PreviousViewUniformOffset` component,
    // re-queried there) at dispatch time, not here — this function only
    // needed its PRESENCE to confirm the render-world view entity has
    // been populated with previous-view data before building bind groups
    // that reference the same underlying `PreviousViewUniforms` buffer.
    let _ = previous_view_offset;
}

/// Stochastic depth-of-field's own bind-group prepare step — same overall
/// shape as `prepare_hybrid_temporal`'s own reflect/transmit history
/// sections (resize-or-config-change invalidation, ping-pong parity,
/// read+write bind groups), kept as its own function rather than folded
/// into `prepare_hybrid_temporal` since DOF has no `PreviousViewData`
/// dependency at all (its own reprojection uses the CURRENT frame's
/// `View.clip_from_world`, read directly in `hybrid_dof.wgsl` via the
/// shared `hybrid_view_layout` group already built once per view by
/// `prepare_hybrid_view_bind_groups` — see `HybridViewBindGroup`'s own
/// doc comment) — genuinely independent of `PreviousViewUniforms`
/// unlike every other temporal pass in this file.
pub fn prepare_hybrid_dof(
    render_device: Res<RenderDevice>,
    pipeline_cache: Res<PipelineCache>,
    hybrid_pipeline: Option<Res<HybridPipeline>>,
    buffers: Res<HybridBuffers>,
    targets: Res<HybridTargetsRes>,
    mut dof_history: ResMut<HybridDofHistoryRes>,
    mut dof_parity: ResMut<HybridDofFrameParity>,
    mut commands: Commands,
) {
    let Some(hp) = hybrid_pipeline else { return };
    let Some(targets) = targets.0.as_ref() else { return };
    let Some(scene_binding) = buffers.scene_uniform.binding() else { return };
    let Some(objects_binding) = buffers.objects.buffer().map(|b| b.as_entire_binding()) else { return };
    let Some(nodes_binding) = buffers.nodes.buffer().map(|b| b.as_entire_binding()) else { return };

    let want = targets.size;
    let needs_new = match dof_history.0.as_ref() {
        Some(h) => h.size != want,
        None => true,
    };
    if needs_new && want.x > 0 && want.y > 0 {
        let make = |format: TextureFormat, label: &str| {
            render_device
                .create_texture(&TextureDescriptor {
                    label: Some(label),
                    size: Extent3d { width: want.x, height: want.y, depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: TextureDimension::D2,
                    format,
                    usage: TextureUsages::STORAGE_BINDING | TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&TextureViewDescriptor::default())
        };
        let make_slot = |label_prefix: &str| HybridHistorySlot {
            color_view: make(TextureFormat::Rgba16Float, &format!("{label_prefix}_color")),
            length_view: make(TextureFormat::R32Float, &format!("{label_prefix}_length")),
            depth_view: make(TextureFormat::R32Float, &format!("{label_prefix}_depth")),
            normal_view: make(TextureFormat::Rgba32Float, &format!("{label_prefix}_normal")),
        };
        dof_history.0 = Some(HybridDofHistory { size: want, slots: [make_slot("hybrid_dof_history_a"), make_slot("hybrid_dof_history_b")] });
        dof_parity.0 = 0;
    }
    let Some(dof_history) = dof_history.0.as_ref() else { return };

    dof_parity.0 = dof_parity.0.wrapping_add(1);
    let read_slot = &dof_history.slots[(dof_parity.0 % 2) as usize];
    let write_slot = &dof_history.slots[((dof_parity.0 + 1) % 2) as usize];

    let read_bg = render_device.create_bind_group(
        "hybrid_dof_read_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.dof_read_layout),
        &BindGroupEntries::sequential((
            scene_binding,
            objects_binding,
            nodes_binding,
            &targets.denoised_color_view,
            &hp.dof_sampler,
            &targets.depth_view,
            &targets.normal_view,
            &read_slot.color_view,
            &read_slot.length_view,
            &read_slot.depth_view,
            &read_slot.normal_view,
        )),
    );
    let write_bg = render_device.create_bind_group(
        "hybrid_dof_write_bind_group",
        &pipeline_cache.get_bind_group_layout(&hp.dof_write_layout),
        &BindGroupEntries::sequential((
            &targets.dof_color_view,
            &write_slot.color_view,
            &write_slot.length_view,
            &write_slot.depth_view,
            &write_slot.normal_view,
        )),
    );
    commands.insert_resource(HybridDofBindGroup { read: read_bg, write: write_bg });
}
