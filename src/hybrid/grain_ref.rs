//! CPU reference for `hybrid_post.wgsl`'s film-grain chrominance
//! component — see that shader's own doc comment (right above its
//! `grain_rgb` composition) for the full rationale: real color film has
//! genuinely independent per-emulsion-layer grain, and real digital
//! sensor noise has a real (if smaller) chrominance component from
//! demosaicing amplifying each channel's own white-balance gain
//! differently, both well-established in photographic/imaging
//! literature — AV1's own film-grain-synthesis spec and DaVinci
//! Resolve's own grain tool both model a SMALL chroma-noise component
//! correlated with (not independent of, and not identical to) luma
//! noise, rather than leaving grain purely monochromatic.
//!
//! Only this new composition function gets a CPU-ref — the surrounding
//! hash/signal-factor grain machinery in `hybrid_post.wgsl` predates this
//! project's CPU-reference-first convention and has no existing Rust
//! mirror to extend; this module covers what's new here, not a
//! retroactive full port of pre-existing WGSL-only code.

use bevy::math::Vec3;

/// How much of the chroma noise samples ride on top of the luma noise
/// sample vs. being independent hash draws — `0.0` = fully independent
/// per-channel noise (the "colored speckling" look reviews originally
/// flagged as looking wrong), `1.0` = chroma noise exactly equals luma
/// noise (back to fully monochromatic). `0.5` is a middle ground: real
/// chroma noise is correlated with luma noise (same underlying photon-
/// count/crystal-density variation drives both) but not identical to it
/// (independent per-layer/per-channel crystal populations and gain
/// differences are real, physical sources of divergence) — see this
/// module's own doc comment for the imaging-literature basis.
pub const CHROMA_LUMA_COUPLING: f32 = 0.5;

/// How large the chroma-noise contribution is relative to the luma
/// noise's own amplitude — kept deliberately SMALL (real color-grading
/// tools like DaVinci Resolve default chroma noise well below luma
/// noise; AV1's own film-grain synthesis scales chroma noise as a
/// function of luma, never dominating it) so this stays a subtle
/// realism addition, not a reversion to the "chroma noise is as strong
/// as luma noise" look that inspired the original bug report.
pub const CHROMA_GRAIN_RATIO: f32 = 0.2;

/// Composes the final per-channel grain contribution from one luma noise
/// sample and two independent chroma hash samples (`noise_cr`/
/// `noise_cb`, expected to come from a DIFFERENT hash input than
/// `noise_luma` — see `hybrid_post.wgsl`'s own call site for the salt
/// values used) — a crude luma+Cr/Cb -> RGB perturbation (not a real
/// YCbCr-to-RGB matrix; this is a small ADDITIVE noise perturbation, not
/// a color-space conversion of the image itself, so the approximation is
/// deliberately cheap).
///
/// Returns `(luma_noise, luma_noise, luma_noise)` exactly (bit-for-bit
/// monochromatic) when `noise_cr == noise_cb == noise_luma` — i.e. this
/// function is a strict GENERALIZATION of the prior pure-monochromatic
/// behavior, not a replacement that could reintroduce a regression if
/// the two new hash samples ever happened to degenerate to the luma
/// sample. Achieved by perturbing each channel by the chroma sample's
/// own DEVIATION from the luma sample (`cr_delta`/`cb_delta`), not by
/// the chroma sample's raw value — a raw-value blend would add a
/// nonzero per-channel offset even when the chroma "noise" happens to
/// exactly equal the luma noise, which is not the intended "extra
/// divergence on top of luma" semantics.
pub fn chroma_grain_rgb(noise_luma: f32, noise_cr: f32, noise_cb: f32) -> Vec3 {
    let cr_delta = (noise_cr - noise_luma) * (1.0 - CHROMA_LUMA_COUPLING);
    let cb_delta = (noise_cb - noise_luma) * (1.0 - CHROMA_LUMA_COUPLING);
    Vec3::new(
        noise_luma + cr_delta * CHROMA_GRAIN_RATIO,
        noise_luma - (cr_delta + cb_delta) * CHROMA_GRAIN_RATIO * 0.5,
        noise_luma + cb_delta * CHROMA_GRAIN_RATIO,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identical_noise_samples_reproduce_exactly_monochromatic_grain() {
        let g = chroma_grain_rgb(0.3, 0.3, 0.3);
        assert!((g.x - 0.3).abs() < 1e-6, "R channel must equal the luma noise exactly when all samples agree: got {g:?}");
        assert!((g.y - 0.3).abs() < 1e-6, "G channel must equal the luma noise exactly when all samples agree: got {g:?}");
        assert!((g.z - 0.3).abs() < 1e-6, "B channel must equal the luma noise exactly when all samples agree: got {g:?}");
    }

    #[test]
    fn distinct_chroma_samples_produce_distinct_per_channel_output() {
        let g = chroma_grain_rgb(0.1, 0.4, -0.2);
        assert!(
            (g.x - g.y).abs() > 1e-4 && (g.y - g.z).abs() > 1e-4,
            "distinct Cr/Cb hash samples must produce genuinely different R/G/B channel values, not degenerate back to monochromatic: got {g:?}"
        );
    }

    #[test]
    fn chroma_contribution_stays_small_relative_to_luma() {
        // The whole point of CHROMA_GRAIN_RATIO: even a maximally
        // divergent Cr/Cb pair must not swing any channel far from the
        // luma value — chroma noise is a SUBTLE addition, not a
        // dominant one.
        let g = chroma_grain_rgb(0.0, 0.5, -0.5);
        let luma = 0.0;
        assert!((g.x - luma).abs() < 0.3, "R channel deviated too far from luma noise at maximal Cr/Cb divergence: {g:?}");
        assert!((g.y - luma).abs() < 0.3, "G channel deviated too far from luma noise at maximal Cr/Cb divergence: {g:?}");
        assert!((g.z - luma).abs() < 0.3, "B channel deviated too far from luma noise at maximal Cr/Cb divergence: {g:?}");
    }

    #[test]
    fn green_channel_is_the_negatively_correlated_channel() {
        // grain_rgb's own R/G/B formula: R gets +cr, B gets +cb, G gets
        // -(cr+cb)/2 — mirroring a real YCbCr->RGB-style relationship
        // where boosting Cr/Cb pulls R/B up and pulls G down (this is
        // NOT a real color-matrix conversion, see this module's own doc
        // comment, but the sign relationship should still hold: positive
        // Cr/Cb chroma noise should visibly redden/blue an area while
        // dimming its green contribution, not boost all three together).
        let g = chroma_grain_rgb(0.0, 0.6, 0.6);
        assert!(g.x > 0.0, "positive Cr/Cb noise must push R upward: {g:?}");
        assert!(g.z > 0.0, "positive Cr/Cb noise must push B upward: {g:?}");
        assert!(g.y < 0.0, "positive Cr/Cb noise must push G downward (negatively correlated channel): {g:?}");
    }
}
