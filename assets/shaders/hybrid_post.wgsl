// Post-tonemap "lens/sensor" pass: film grain + vignette + chromatic
// aberration, applied to the ALREADY-TONEMAPPED, display-referred image —
// deliberately scheduled after Bevy's own Node3d-equivalent tonemapping
// system (Core3dSystems::PostProcess, .after(tonemapping) — see
// HybridRenderPlugin's own doc comment), not folded into hybrid_blit.wgsl,
// because these are conventionally camera/sensor artifacts applied to the
// final display image, not to scene-linear radiance: chromatic aberration
// simulates a lens's per-wavelength focal shift on the image the sensor
// already captured, grain simulates the sensor's own read noise on the
// developed image, and vignette simulates the lens's own light falloff —
// none of these are physically about scene radiance itself.
//
// Reads/writes via ViewTarget's own post_process_write() ping-pong, same
// mechanism bevy_core_pipeline's tonemapping/upscaling passes use (see
// their own node.rs source, read as a pattern reference here, not
// imported) — this pass's own source texture is whatever tonemapping just
// wrote.

#import bevy_render::view::{View, frag_coord_to_uv}

@group(0) @binding(0) var<uniform> view: View;
@group(0) @binding(1) var source_tex: texture_2d<f32>;
@group(0) @binding(2) var source_sampler: sampler;

struct PostUniform {
    // 0.0 disables the whole pass (dispatch still runs — this is a
    // plain fragment shader with no separate dispatch-skip path, see
    // HybridPostConfig's own doc comment for why "always run, coefficient
    // zero" costs one cheap fullscreen pass rather than needing a second
    // render-graph branch).
    grain_strength: f32,
    vignette_strength: f32,
    // World-space-agnostic: a fraction of the max per-channel UV offset
    // at the frame corner, growing toward the edges — see
    // `chromatic_aberration_offset`'s own doc comment.
    aberration_strength: f32,
    // A cheap, deterministic hash seed (frame count) — NOT real per-pixel
    // randomness (this renderer has no RNG primitive, see refract_ref.wgsl's
    // own doc comment on that constraint); grain uses a spatial hash of
    // pixel coordinates plus this seed so the noise pattern still changes
    // frame to frame instead of being a static fixed dither pattern.
    frame_seed: f32,
}

@group(1) @binding(0) var<uniform> post: PostUniform;

// Cheap, deterministic hash -> [0,1) — same "no RNG primitive, use a
// spatial/temporal hash instead" approach the rest of this renderer
// already relies on (see docs/knowledge for the no-RNG constraint).
fn hash12(p: vec2<f32>) -> f32 {
    let p3 = fract(vec3<f32>(p.x, p.y, p.x) * 0.1031);
    let p3b = p3 + dot(p3, p3.yzx + 33.33);
    return fract((p3b.x + p3b.y) * p3b.z);
}

// Radial falloff toward the frame edges (0 at center, 1 at the corner) —
// used by both vignette (darkening) and chromatic aberration (offset
// magnitude), matching how both are driven by the same lens-radius effect
// on a real camera.
fn radial_falloff(uv: vec2<f32>) -> f32 {
    let centered = uv - vec2<f32>(0.5, 0.5);
    // Normalized so the frame corner (uv at (0,0)/(1,1)) reaches exactly
    // 1.0 regardless of aspect ratio.
    return clamp(length(centered) / length(vec2<f32>(0.5, 0.5)), 0.0, 1.0);
}

@fragment
fn fragment(@builtin(position) frag_coord: vec4<f32>) -> @location(0) vec4<f32> {
    let uv = frag_coord_to_uv(frag_coord.xy, view.viewport);
    let falloff = radial_falloff(uv);

    // Chromatic aberration: sample R/G/B at slightly different UVs,
    // offset radially outward from center by an amount that grows with
    // falloff — the standard "per-channel focal shift grows toward the
    // edges" approximation (zero at dead center, matching a real lens's
    // near-zero on-axis aberration).
    let dir = normalize(uv - vec2<f32>(0.5, 0.5) + vec2<f32>(1e-6, 0.0));
    let max_offset = post.aberration_strength * falloff * falloff;
    let r = textureSample(source_tex, source_sampler, uv - dir * max_offset).r;
    let g = textureSample(source_tex, source_sampler, uv).g;
    let b = textureSample(source_tex, source_sampler, uv + dir * max_offset).b;
    var color = vec3<f32>(r, g, b);

    // Vignette: multiplicative darkening growing toward the edges —
    // smoothstep rather than a hard-edged falloff so it reads as a lens's
    // continuous light falloff, not a drawn circle.
    let vignette = 1.0 - post.vignette_strength * smoothstep(0.2, 1.0, falloff);
    color *= vignette;

    // Film grain: signed noise around 0, applied additively post-
    // vignette — hashed on pixel coordinate + frame_seed so it flickers
    // frame to frame like real sensor noise rather than a fixed dither
    // pattern baked into the image. Scaled by a signal-dependent factor,
    // NOT a flat coefficient: real sensor noise is far more visible in
    // shadows than highlights (read noise is a roughly fixed floor, so
    // in relative terms it dominates a small signal and is swamped by a
    // large one) — approximated here as inversely proportional to
    // luminance, floored so near-black pixels don't produce an
    // unbounded/infinite grain multiplier and ceilinged at 1.0 so bright
    // highlights still get a small amount (real sensors are never
    // perfectly noiseless even when well-exposed).
    let luminance = dot(color, vec3<f32>(0.2126, 0.7152, 0.0722));
    let signal_factor = clamp(1.0 - luminance, 0.15, 1.0);
    let noise_luma = hash12(frag_coord.xy + vec2<f32>(post.frame_seed, post.frame_seed * 1.618)) - 0.5;

    // Small deliberate CHROMA component, not a bug fix for "colored
    // speckling was reported" (this pass's own math was already
    // correct — CA samples the clean pre-grain image, grain is added
    // once to all three channels identically, see this fragment's own
    // call order above) but a deliberate realism addition: real color
    // film has genuinely independent per-emulsion-layer grain (three
    // physically separate silver-halide crystal populations, one per
    // dye layer), and real digital sensor noise has a real, if smaller,
    // chrominance component from demosaicing amplifying each channel's
    // white-balance gain differently — both well-established in
    // photographic/imaging literature. AV1's own film-grain-synthesis
    // spec and DaVinci Resolve's own grain tool both model chroma noise
    // as a SMALL component correlated with (not independent of, and not
    // identical to) luma noise, not left out entirely — matched here:
    // two more hash samples (different additive salts, so they're
    // independent samples of the same hash function rather than reused
    // scaled copies of noise_luma) partially blended toward noise_luma
    // rather than fully independent, then added at a fraction of the
    // luma amplitude.
    // grain_ref.rs::chroma_grain_rgb verbatim — perturbs each channel by
    // the CHROMA sample's own deviation from the luma sample (not the
    // chroma sample's raw value), so this degenerates to exactly
    // monochromatic grain whenever the chroma hash happens to agree with
    // the luma hash, rather than always injecting a nonzero per-channel
    // offset regardless of whether the samples agree.
    let noise_cr = hash12(frag_coord.xy + vec2<f32>(post.frame_seed * 2.718, post.frame_seed * 0.577)) - 0.5;
    let noise_cb = hash12(frag_coord.xy + vec2<f32>(post.frame_seed * 1.414, post.frame_seed * 3.146)) - 0.5;
    const CHROMA_LUMA_COUPLING: f32 = 0.5;
    let cr_delta = (noise_cr - noise_luma) * (1.0 - CHROMA_LUMA_COUPLING);
    let cb_delta = (noise_cb - noise_luma) * (1.0 - CHROMA_LUMA_COUPLING);
    const CHROMA_GRAIN_RATIO: f32 = 0.2;
    let grain_rgb = vec3<f32>(
        noise_luma + cr_delta * CHROMA_GRAIN_RATIO,
        noise_luma - (cr_delta + cb_delta) * CHROMA_GRAIN_RATIO * 0.5,
        noise_luma + cb_delta * CHROMA_GRAIN_RATIO,
    );
    color += grain_rgb * post.grain_strength * signal_factor;

    return vec4<f32>(clamp(color, vec3<f32>(0.0), vec3<f32>(1.0)), 1.0);
}
