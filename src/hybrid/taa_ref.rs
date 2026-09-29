//! CPU reference for the primary ray's own sub-pixel jitter — step 1a of
//! a temporal-upscaling experiment (see `PROGRESS.md`'s own "distance-
//! scaled march-convergence epsilon" entry for the full 3-step plan this
//! is item 2 of). This module covers ONLY the jitter-sequence generator
//! and its NDC-offset conversion; no resolution change happens yet — the
//! goal of this step is narrowly to prove jitter alone doesn't destabilize
//! `hybrid_temporal.wgsl`'s disocclusion tests or `hybrid_dof.wgsl`'s own
//! same-pixel (no-reprojection) history check before combining it with an
//! actual sub-resolution trace target.
//!
//! `hybrid_trace.wgsl`'s own `generate_primary_ray` doc comment (right
//! above that function) documents a real, previously-correct argument
//! against jittering the primary ray: its hit point feeds `out_depth`,
//! the GI temporal accumulator's disocclusion test, and the reflection/
//! refraction virtual-point reprojection, none of which distinguish
//! "sub-pixel jitter" from a real disocclusion. This module doesn't
//! invalidate that argument — it tests whether the EXISTING rejection
//! thresholds (`TEMPORAL_DEPTH_SIGMA`, `TEMPORAL_NORMAL_COS_THRESHOLD` in
//! `temporal_ref.rs`) are already loose enough to absorb sub-pixel jitter
//! as normal noise rather than a disocclusion, which is an empirical
//! question this step's live A/B measurement answers.

use bevy::math::Vec2;

/// Halton(2, 3) low-discrepancy sequence, 1-indexed (`index=0` returns the
/// sequence's own `n=1` term, not the degenerate `n=0` term which is
/// exactly `(0, 0)` — a jitter offset of zero on the very first frame
/// would be a fine start numerically, but every OTHER value in the
/// sequence is well spread across `(0, 1)`, so 1-indexing keeps the
/// window this function is sampled over (`0..ring_size`) free of that one
/// special-cased point, matching `dof_ref::vogel_disk_sample`'s own
/// "shift index by a fixed offset so no sample lands exactly at the
/// degenerate point" precedent). Base 2 drives the X axis, base 3 the Y
/// axis — the standard TAAU/TAA jitter sequence (used by, among others,
/// Unreal Engine's own TemporalAA implementation) precisely because two
/// coprime bases never repeat a coordinate pair within any reasonably
/// sized ring and cover the unit square far more evenly than a uniform
/// grid or a naive PRNG would over a short window.
fn halton(mut index: u32, base: u32) -> f32 {
    let mut result = 0.0f32;
    let mut fraction = 1.0f32;
    while index > 0 {
        fraction /= base as f32;
        result += fraction * (index % base) as f32;
        index /= base;
    }
    result
}

/// This frame's sub-pixel jitter offset, in TEXEL units, both axes in
/// `[-0.5, 0.5]` — centered on the pixel (unlike raw Halton's own `[0,
/// 1)` range) since a jitter that only ever pushes the sample toward one
/// corner would bias the accumulated result rather than converging to an
/// unbiased box filter over many frames. `frame_index` is this frame's
/// raw counter (mirrors `dof_ref::dof_sample_index`'s own input); unlike
/// that function this one does NOT reduce mod `ring_size` first — Halton
/// itself is aperiodic (each new index refines the sequence's coverage
/// rather than cycling a fixed set of points), so `ring_size` here exists
/// only to bound how large `index` is allowed to grow before wrapping,
/// keeping the float precision in `halton`'s own accumulation loop well
/// clear of any degenerate large-index behavior over a long-running
/// session — NOT to select from a fixed small set the way DOF's Vogel-
/// disk ring does.
pub fn taa_jitter_offset(frame_index: u32, ring_size: u32) -> Vec2 {
    let index = (frame_index % ring_size.max(1)) + 1;
    Vec2::new(halton(index, 2) - 0.5, halton(index, 3) - 0.5)
}

/// Converts a texel-space jitter offset into the NDC-space offset
/// `generate_primary_ray` adds after its own `uv_to_ndc` step — texel
/// jitter is symmetric per-axis in PIXELS, but NDC's `x`/`y` axes each
/// span `2.0` (from `-1` to `1`) over `viewport_size` pixels, so the
/// conversion is `jitter_texels * (2.0 / viewport_size)`. Kept as its own
/// function (rather than inlined at each of the three `generate_primary_
/// ray` call sites this renderer duplicates the ray-gen math at) so the
/// `2.0 / size` factor is derived once, here, and every caller/port
/// agrees on it — the same "one small function everyone calls" shape
/// `pixel_eps`/`shadow_bias` already establish elsewhere in this module
/// family.
pub fn jitter_texels_to_ndc(jitter_texels: Vec2, viewport_size: Vec2) -> Vec2 {
    jitter_texels * (2.0 / viewport_size)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn halton_base_2_index_1_is_one_half() {
        // The textbook first term of the radical-inverse sequence in any
        // base b is always exactly 1/b (index=1 has a single "digit" of
        // value 1 in the least-significant place).
        assert!((halton(1, 2) - 0.5).abs() < 1e-6, "halton(1, 2) should be exactly 0.5, got {}", halton(1, 2));
    }

    #[test]
    fn halton_base_3_index_1_is_one_third() {
        assert!((halton(1, 3) - (1.0 / 3.0)).abs() < 1e-6, "halton(1, 3) should be exactly 1/3, got {}", halton(1, 3));
    }

    #[test]
    fn halton_index_zero_is_exactly_zero() {
        assert_eq!(halton(0, 2), 0.0, "the degenerate n=0 term of any Halton sequence is exactly 0.0");
    }

    #[test]
    fn halton_stays_within_the_unit_interval_across_many_indices() {
        for i in 0..500u32 {
            let x = halton(i, 2);
            let y = halton(i, 3);
            assert!((0.0..1.0).contains(&x), "halton(2) index {i} out of [0,1): {x}");
            assert!((0.0..1.0).contains(&y), "halton(3) index {i} out of [0,1): {y}");
        }
    }

    #[test]
    fn taa_jitter_offset_is_centered_within_plus_minus_half_texel() {
        for frame in 0..64u32 {
            let j = taa_jitter_offset(frame, 8);
            assert!((-0.5..=0.5).contains(&j.x), "jitter.x out of [-0.5, 0.5]: {j:?} at frame {frame}");
            assert!((-0.5..=0.5).contains(&j.y), "jitter.y out of [-0.5, 0.5]: {j:?} at frame {frame}");
        }
    }

    #[test]
    fn taa_jitter_offset_visits_more_than_one_distinct_value_across_a_ring() {
        // A degenerate implementation that always returns the same offset
        // (e.g. a copy-paste bug reusing DOF's own single-sample-index
        // convention without actually varying per frame) would defeat the
        // entire point of jittering: every frame would resample the exact
        // same sub-pixel position, so accumulation could never converge
        // to anything beyond that one point.
        let distinct: std::collections::HashSet<[u32; 2]> =
            (0..16u32).map(|f| taa_jitter_offset(f, 16)).map(|v| [v.x.to_bits(), v.y.to_bits()]).collect();
        assert!(distinct.len() > 8, "expected a well-spread jitter sequence, got only {} distinct values across 16 frames", distinct.len());
    }

    #[test]
    fn taa_jitter_offset_never_lands_exactly_on_the_pixel_center() {
        // A jitter of exactly (0, 0) on some frame would make that frame
        // indistinguishable from the un-jittered case, which is harmless
        // in isolation but would mean the sequence isn't doing its job
        // uniformly — 1-indexing (skipping Halton's own degenerate n=0
        // term) is what this test is actually verifying.
        for frame in 0..64u32 {
            let j = taa_jitter_offset(frame, 64);
            assert!(j.x != 0.0 || j.y != 0.0, "jitter offset was exactly zero at frame {frame}");
        }
    }

    #[test]
    fn jitter_texels_to_ndc_scales_by_two_over_viewport_size() {
        let ndc = jitter_texels_to_ndc(Vec2::new(0.5, -0.5), Vec2::new(1280.0, 720.0));
        assert!((ndc.x - (1.0 / 1280.0)).abs() < 1e-7, "expected 2*0.5/1280, got {}", ndc.x);
        assert!((ndc.y - (-1.0 / 720.0)).abs() < 1e-7, "expected 2*-0.5/720, got {}", ndc.y);
    }

    #[test]
    fn jitter_texels_to_ndc_is_zero_for_zero_jitter() {
        let ndc = jitter_texels_to_ndc(Vec2::ZERO, Vec2::new(1920.0, 1080.0));
        assert_eq!(ndc, Vec2::ZERO);
    }
}
